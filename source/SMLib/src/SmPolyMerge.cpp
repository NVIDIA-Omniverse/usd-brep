// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************//**
* FILE NAME --- SmPolyMerge.cpp 
* PURPOSE: Implementation of Poly Brep merging methods.  
**********************************************************************/

#include "StdAfx.h"

#include <SmGraphicsExtern.h>
#include <SmPolyMerge.h>
#include <SmRelation.h>
#include <SmAttribute.h>
#include <SmGrid.h>
#include <SmRayTracer.h>
#include <SmSolutionArray.h>
#include <SmGeomUtility.h>
#include <SmPolySolver.h>
#include <SmAssertArray.h>

// Following define puts lot of topology pointer checking in to this code.
// It slows things down substantially, and should be used only for debugging.
//#define SM_VALIDATE_TOPOLOGY 1

/*******************************************************************//**
PURPOSE: Constructor for a Merge object.  It simply sets the
    Merged Set and the Brep which will be merged into it.

NOTES: 
***********************************************************************/
SmPolyMerge::SmPolyMerge
  (const SmContext & crContext,
   SmPolyBrep      * pBrep,  
   SmPolyBrep      * pOtherBrep,
   double            dApproxTol3d, 
   double            dAngleTolRadians)
 : m_bRemoveCoPlanarEdges(TRUE), 
   m_bRemoveOnlyInvisibleEdges(FALSE), 
   m_bManifoldBoolean(TRUE), 
   m_bIntersectionLoopClosed(TRUE), 
   m_bImprinting(FALSE),
   m_bImprintAndClassify(FALSE), 
   m_bCookieCutter(FALSE),
   m_bCheckLicense(FALSE),
   m_vPI(crContext,pBrep,pOtherBrep,dApproxTol3d,dAngleTolRadians)
{
  SM_ASSERT(pBrep != NULL);
  SM_ASSERT(pOtherBrep != NULL);
}

/*******************************************************************//**
PURPOSE: Constructor for a Merge object.  It simply sets the
    Merged Set and the Brep which will be merged into it.

NOTES: 
***********************************************************************/
SmPolyMerge::SmPolyMerge
  (const SmContext & crContext)
 : m_bRemoveCoPlanarEdges(TRUE), 
   m_bRemoveOnlyInvisibleEdges(FALSE),
   m_bManifoldBoolean(TRUE), 
   m_bIntersectionLoopClosed(TRUE), 
   m_bImprinting(FALSE),
   m_bImprintAndClassify(FALSE),
   m_bCookieCutter(FALSE), 
   m_bCheckLicense(FALSE),
   m_vPI(crContext,NULL,NULL,SM_EFF_ZERO_SQ,SM_DEG2RAD(30))
{
} // end SmPolyMerge::SmPolyMerge constructor

/*******************************************************************//**
PURPOSE: Check whether a face/side list collection contains a given face/side.

NOTES: If eSide==SM_OT_UNKNOWN, return True for either side, if the face is there.
***********************************************************************/
static SmBoolean sm_ContainsFaceSide
 ( SmPolyFace                          * pPF,         // in
   SmOrientType                          eSide,       // in
   SmTArray< SmTArray<SmPolyFace*> *>  & rFaceLists,  // in
   SmTArray< SmTArray<SmOrientType> *> & rSideLists ) // in
{
  ULONG ii, jj, lNumLists = rFaceLists.GetSize();
  for ( ii=0; ii<lNumLists; ii++ )
  {
      SmTArray< SmPolyFace *> *pFaces = rFaceLists[ii];
      SmTArray< SmOrientType> *pSides = rSideLists[ii];
      for ( jj=0; jj<pFaces->GetSize(); jj++ )
      {
          if ( (*pFaces)[jj] == pPF && ( eSide==SM_OT_UNKNOWN || (*pSides)[jj] == eSide ) )
            { return TRUE; }
      }
  }

  return FALSE;

} // end sm_ContainsFaceSide

/*******************************************************************//**
PURPOSE: Remove nonmanifold face/side sets from a collection.

NOTES:
   The side lists are not used in deciding which to remove,
   but they must be kept in sync with the face lists.
***********************************************************************/
static void sm_RemoveNonmanifoldFaceSets(
          SmTArray< SmTArray<SmPolyFace*> *>  & rFaceLists,
          SmTArray< SmTArray<SmOrientType> *> & rSideLists )
{
  ULONG ii, jj, lNumLists = rFaceLists.GetSize();
  ULONG lLamina, lNonManifold, lManifold;
  SmBoolean bModified = FALSE;

  for ( ii=0; ii<lNumLists; ii++ )
  {
      SmTArray<SmPolyFace *> *pFaces = rFaceLists[ii];
      SmTArray<SmOrientType> *pSides = rSideLists[ii];
      for ( jj=0; jj<pFaces->GetSize(); jj++ )
      {
          SmPolyFace *pPF = (*pFaces)[jj];
          pPF->GetPolyFaceDetail( lLamina, lNonManifold, lManifold );
          if ( lLamina>0 /* || lNonManifold>0 */ ) // Allow spine edges.
          {
              // Remove this set from the lists.
              delete pFaces;
              delete pSides;
              rFaceLists[ii] = NULL;
              rSideLists[ii] = NULL;
              bModified = TRUE;
              break;
          }
      }
  }

  if ( bModified )
  {
      rFaceLists.CompressZeros();
      rSideLists.CompressZeros();
  }

  return;

} // end sm_RemoveNonmanifoldFaceSets

/*******************************************************************//**
PURPOSE: This operation will clean up adjacent coplanar faces of each
    brep by removing edges between those adjacent faces.

NOTES: 
  This method is identical to SmPolyMerge::RemoveCoPlanarEdges() except:
  - This method processes both m_pBrep and m_pOther, and
  - This method optionally skips visible edges.
  If you change one, check whether the other should be changed as well.
***********************************************************************/
SmStatus SmPolyMerge::CleanupCoPlanarFaces()
{

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe=FALSE;
if (bDebugMe) 
  {
    SM_ASSERT_VALID(m_vPI.m_pBrep) ; 
    SM_ASSERT_VALID(m_vPI.m_pOther) ; 
  }
#endif

  // Locals
  ULONG ii, jj ;
  SM_PTR_ARRAY(sPolyEdges,   SmPolyEdge,  64) ; // SmTArray<SmPolyEdge *>
  SM_PTR_ARRAY(sRmPolyEdges, SmPolyEdge,  64) ; // SmTArray<SmPolyEdge *>

  ULONG       lIdx1,  lIdx2;
  SmPolyEdge *pDel1, *pDel2, *pSurvive1, *pSurvive2;
  SmBoolean   bWasSqueezed;

  // for 2 passes - this PolyBrep and Other PolyBrep
  for (ii=0; ii<2; ii++) 
    {
      // Process m_pBrep on first pass, m_pOther on second.
      SmPolyBrep * pPolyBrep = (ii == 0) ? m_vPI.m_pBrep 
                                         : m_vPI.m_pOther;

      // no work - don't run twice on same brep
      if (ii == 1 && m_vPI.m_pOther == m_vPI.m_pBrep )
        { break; } // Don't repeat on the same PolyBrep. [B367 B368]

      pPolyBrep->GetPolyEdges(sPolyEdges);

      // Assign each edge a unique IndexValue.
      ULONG lNumEdges = sPolyEdges.GetSize();
      for (jj=0; jj<lNumEdges; jj++) 
        {
          sPolyEdges[jj]->SetIndexValue(jj);
        }

      // For each edge, check whether it can be removed.
      // Also check its radial mate, if any.
      for (jj=0; jj<lNumEdges; jj++) 
        {
          SmPolyEdge * pPolyEdge = sPolyEdges[jj];
          if (pPolyEdge == NULL) { continue; }

          // when asked - don't remove visible edges
          if (m_bRemoveOnlyInvisibleEdges && pPolyEdge->m_bVisibility) 
            { continue; }

          // When edge is connected to (a strut? or to) its own end 
          if ( pPolyEdge->GetCCWPolyEdge()->GetCCWPolyEdge() == pPolyEdge )
            {
              SmPolyFace *pPolyFace = pPolyEdge->GetPolyFace();

              // squeeze face when it contains only two PolyEdges and glue any attached radial partners
              //   gwc - should be extended to more general cases: face too small, sliver face, etc.
              SE( pPolyBrep->SqueezeFaceIfDegenerate( pPolyFace,                 // in : target PolyFace to delete
                                                      pPolyBrep->GetTolerance(), // in : not used
                                                      bWasSqueezed,              // out: TRUE=PolyFace was squeezed, FALSE=not
                                                      pDel1,                     // out: STALE pointer to deleted PolyEdge1, NULL=none deleted, don't use as an object
                                                      pDel2,                     // out: STALE pointer to deleted PolyEdge2, NULL=none deleted, don't use as an object
                                                      pSurvive1,                 // out: Surviving radial partner to deleted PolyEdge1, NULL=none
                                                      pSurvive2 ));              // out: Surviving radial partner to deleted PolyEdge2, NULL=none
                                                                                 // out: optional array of deleted PolyVertices, NULL to ignore, default:[NULL]
                                                                                 // out: optional associated surviving PolyVertices, NULL to ignore, default:[NULL]
              if (bWasSqueezed) 
                {
                  // Clear the edges from the sPolyEdges array and move on to next edge.
                  if ( sPolyEdges.FindElement(pDel1,lIdx1) ) { sPolyEdges[lIdx1] = NULL; }  // GWC: could lIndexValue to save linear search time
                  if ( sPolyEdges.FindElement(pDel2,lIdx2) ) { sPolyEdges[lIdx2] = NULL; }
                  continue;
                }
            } // end edge connects to itself check

          // Do the same for the edge's radial mate, if present.
          SmPolyEdge *pMate = pPolyEdge->GetRadial();
          if ( pMate && pMate->GetCCWPolyEdge()->GetCCWPolyEdge() == pMate )
            {
              SmPolyFace *pPolyFace = pMate->GetPolyFace();

              // squeeze face when it contains only two PolyEdges and glue any attached radial partners
              //   gwc - should be extended to more general cases: face too small, sliver face, etc.
              SE( pPolyBrep->SqueezeFaceIfDegenerate(pPolyFace,                 // in : target PolyFace to delete
                                                     pPolyBrep->GetTolerance(), // in : not used
                                                     bWasSqueezed,              // out: TRUE=PolyFace was squeezed, FALSE=not
                                                     pDel1,                     // out: STALE pointer to deleted PolyEdge1, NULL=none deleted, don't use as an object
                                                     pDel2,                     // out: STALE pointer to deleted PolyEdge2, NULL=none deleted, don't use as an object
                                                     pSurvive1,                 // out: Surviving radial partner to deleted PolyEdge1, NULL=none
                                                     pSurvive2 ));              // out: Surviving radial partner to deleted PolyEdge2, NULL=none
                                                                                // out: optional array of deleted PolyVertices, NULL to ignore, default:[NULL]
                                                                                // out: optional associated surviving PolyVertices, NULL to ignore, default:[NULL]
              if (bWasSqueezed) 
                {
                  // Clear the edges from the sPolyEdges array and move on to next edge.
                  if ( sPolyEdges.FindElement(pDel1,lIdx1) ) { sPolyEdges[lIdx1] = NULL; }  // GWC: could lIndexValue to save linear search time
                  if ( sPolyEdges.FindElement(pDel2,lIdx2) ) { sPolyEdges[lIdx2] = NULL; }
                  continue;
                }
            } // end edge->mate connects to itself check

          // Remove processed edges from sPolyEdges array.
          sPolyEdges[jj] = NULL;
          if (pMate) 
            {
              ULONG lRemoveMate = pMate->GetIndexValue();
              sPolyEdges[lRemoveMate] = NULL; // Only need to check this one from one direction
            }

          // merge two faces into one when coplanar
          pPolyBrep->RemoveEdgeBetweenCoplanarFaces(pPolyEdge, sRmPolyEdges);

        } // end iter jj, for every polyBrep PolyEdge

#ifdef SM_VALIDATE_TOPOLOGY 
      pPolyBrep->ValidatePointers();
#endif

      // Clean up coplanar edges.
      SER(pPolyBrep->RemoveVerticesBetweenColinearEdges());

#ifdef SM_VALIDATE_TOPOLOGY 
      pPolyBrep->ValidatePointers();
#endif
    } // end iter ii, for 2 passes - this PolyBrep and Other PolyBrep

#ifdef SM_DEBUG_CODE
if (bDebugMe) 
  {
    SM_ASSERT_VALID( m_vPI.m_pBrep );
    SM_ASSERT_VALID( m_vPI.m_pOther);
  }
#endif
  
  // all done 
  return SM_SUCCESS;

} // end SmPolyMerge::CleanupCoPlanarFaces

/*******************************************************************//**
PURPOSE: Remove Coplanar edges after Boolean

NOTES: 
  This method is identical to SmPolyMerge::CleanupCoPlanarFaces() except:
  - This method processes only m_pBrep, and not m_pOther, and
  - This method does not skip visible edges.
  If you change one, check whether the other should be changed as well.
***********************************************************************/
SmStatus SmPolyMerge::RemoveCoPlanarEdges()
{
  // Locals
  SM_PTR_ARRAY(sPolyEdges,   SmPolyEdge,  64) ; // SmTArray<SmPolyEdge *>
  SM_PTR_ARRAY(sRmPolyEdges, SmPolyEdge,  64) ; // SmTArray<SmPolyEdge *>

  SmPolyBrep * pPolyBrep = m_vPI.m_pBrep;
  ULONG        lIdx1, lIdx2;
  SmPolyEdge * pDel1, * pDel2, * pSurvive1, *pSurvive2;
  SmBoolean    bWasSqueezed;

  // get PolyBrep->PolyEdges  
  pPolyBrep->GetPolyEdges(sPolyEdges);

  // Assign each edge a unique index value.
  ULONG ii, lNumEdges = sPolyEdges.GetSize();
  for (ii=0; ii<lNumEdges; ii++)
    {
      sPolyEdges[ii]->SetIndexValue(ii);
    }

  // For each edge, check whether it can be squeezed.
  // Also check its radial mate, if any.
  for (ii=0; ii<lNumEdges; ii++)
    {
      SmPolyEdge * pPolyEdge = sPolyEdges[ii];
      if (pPolyEdge == NULL) { continue; }

      // When edge is connected to (a strut? or to) its own end
      if ( pPolyEdge->GetCCWPolyEdge()->GetCCWPolyEdge() == pPolyEdge )
        {
          SmPolyFace *pPolyFace = pPolyEdge->GetPolyFace();

          // squeeze face when it contains only two PolyEdges and glue any attached radial partners
          //   gwc - should be extended to more general cases: face too small, sliver face, etc.
          SE( pPolyBrep->SqueezeFaceIfDegenerate( pPolyFace,                 // in : target PolyFace to delete
                                                  pPolyBrep->GetTolerance(), // in : not used
                                                  bWasSqueezed,              // out: TRUE=PolyFace was squeezed, FALSE=not
                                                  pDel1,                     // out: STALE pointer to deleted PolyEdge1, NULL=none deleted, don't use as an object
                                                  pDel2,                     // out: STALE pointer to deleted PolyEdge2, NULL=none deleted, don't use as an object
                                                  pSurvive1,                 // out: Surviving radial partner to deleted PolyEdge1, NULL=none
                                                  pSurvive2 ));              // out: Surviving radial partner to deleted PolyEdge2, NULL=none
                                                                             // out: optional array of deleted PolyVertices, NULL to ignore, default:[NULL]
                                                                             // out: optional associated surviving PolyVertices, NULL to ignore, default:[NULL]
          if (bWasSqueezed)
            {
              // Clear the edges from the sPolyEdges array and move on to next edge.
              if ( sPolyEdges.FindElement(pDel1,lIdx1) ) { sPolyEdges[lIdx1] = NULL; }  // GWC: could lIndexValue to save linear search time
              if ( sPolyEdges.FindElement(pDel2,lIdx2) ) { sPolyEdges[lIdx2] = NULL; }
              continue;
            }
        } // end edge connects to itself check

      // Do the same for the edge's radial mate, if present.
      SmPolyEdge *pPolyMate = pPolyEdge->GetRadial();
      if ( pPolyMate && pPolyMate->GetCCWPolyEdge()->GetCCWPolyEdge() == pPolyMate )
        {
          SmPolyFace *pPolyFace = pPolyMate->GetPolyFace();

          // squeeze face when it contains only two PolyEdges and glue any attached radial partners
          //   gwc - should be extended to more general cases: face too small, sliver face, etc.
          SE( pPolyBrep->SqueezeFaceIfDegenerate( pPolyFace,                 // in : target PolyFace to delete
                                                  pPolyBrep->GetTolerance(), // in : not used
                                                  bWasSqueezed,              // out: TRUE=PolyFace was squeezed, FALSE=not
                                                  pDel1,                     // out: STALE pointer to deleted PolyEdge1, NULL=none deleted, don't use as an object
                                                  pDel2,                     // out: STALE pointer to deleted PolyEdge2, NULL=none deleted, don't use as an object
                                                  pSurvive1,                 // out: Surviving radial partner to deleted PolyEdge1, NULL=none
                                                  pSurvive2 ));              // out: Surviving radial partner to deleted PolyEdge2, NULL=none
                                                                             // out: optional array of deleted PolyVertices, NULL to ignore, default:[NULL]
                                                                             // out: optional associated surviving PolyVertices, NULL to ignore, default:[NULL]
          if (bWasSqueezed)
            {
              // Clear the edges from the sPolyEdges array and move on to next edge.
              if ( sPolyEdges.FindElement(pDel1,lIdx1) ) { sPolyEdges[lIdx1] = NULL; } // GWC: could lIndexValue to save linear search time
              if ( sPolyEdges.FindElement(pDel2,lIdx2) ) { sPolyEdges[lIdx2] = NULL; }
              continue;
            }
        } // end edge->mate connects to itself check

      // Remove processed edges from sPolyEdges array.
      sPolyEdges[ii] = NULL;
      if (pPolyMate)
        {
          ULONG lRmMate = pPolyMate->GetIndexValue();
          if ( lRmMate != SM_UNDEF_ULONG )
            { sPolyEdges[lRmMate] = NULL; } // Only need to check this one from one direction
        }
      pPolyBrep->RemoveEdgeBetweenCoplanarFaces(pPolyEdge,sRmPolyEdges);

    } // end for each Edge

#ifdef SM_VALIDATE_TOPOLOGY
  pPolyBrep->ValidatePointers();
#endif

  // Clean up coplanar edges.
  SER(pPolyBrep->RemoveVerticesBetweenColinearEdges());

#ifdef SM_VALIDATE_TOPOLOGY
  pPolyBrep->ValidatePointers();
#endif

  // all done
  return SM_SUCCESS;

} // end SmPolyMerge::RemoveCoPlanarEdges

/*******************************************************************//**
PURPOSE: Mark a poly vertex

NOTES:
***********************************************************************/
static void sm_MarkPolyVertex
 (SmPolyVertex * pV,
  SmMarkType     eMarkType)
{
    SmPolyVertex * pCurrV = pV;
    pCurrV->Mark(eMarkType);
}

/*******************************************************************//**
PURPOSE: Mark a poly edge and all of its radials.

NOTES:
***********************************************************************/
static void sm_MarkPolyEdge
 (SmPolyEdge * pE,
  SmMarkType   eMarkType)
{
    SmPolyEdge * pCurrEdge = pE;
    while (TRUE) {
        pCurrEdge->Mark(eMarkType);
        pCurrEdge = pCurrEdge->GetRadial();
        if (pCurrEdge == NULL || pCurrEdge == pE) break;
    }
}

/*******************************************************************//**
PURPOSE: Collect the faces which are adjacent and not marked without
   going through marked edges.

NOTES:
***********************************************************************/
static void sm_CollectFaces
 (SmPolyFace            * pStartFace,
  SmTArray<SmPolyFace*> & rCollectedFaces,
  SmMarkType              eMarkType)
{
    // Init output.
    rCollectedFaces.ReSet();

    SmMapPtrToPtr<SmPolyFace, SmPolyFace> sMap;
    if (pStartFace->IsMarked(eMarkType)) { return; }

    SmPolyFace * sFData[64];
    SmTArray<SmPolyFace*> sStack(64,sFData);
    sStack.Add( pStartFace );
    sMap.Insert( pStartFace, pStartFace );

    SmPolyEdge * sData[64];
    SmTArray<SmPolyEdge*> sEdges(64,sData);

    SmPolyEdge * sData2[64];
    SmTArray<SmPolyEdge*> sRadialEdges(64,sData2);


    while (sStack.GetSize() > 0)
    {
        SmPolyFace *pFaceStart = sStack.GetLast();
        sStack.RemoveLast();
        
        if (pFaceStart->IsMarked(eMarkType)) { continue; }
        rCollectedFaces.Add( pFaceStart );
        pFaceStart->Mark(eMarkType);
        
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
        if (bDebugMe) {
            sm_GraphicsLoop();
            smgfx_ChangeColor(TRUE); pFaceStart->DrawDebug(); sm_GraphicsLoop();
            sm_GraphicsLoop();
        }
#endif
        
        // Now try to traverse adjacent edges to pick up
        // surrounding faces.
        pFaceStart->GetPolyEdges( sEdges );
        ULONG ii, lNumEdges = sEdges.GetSize();
        for (ii=0; ii<lNumEdges; ii++)
        {
            SmPolyEdge *pEdge = sEdges[ii];
            if (pEdge->IsMarked(eMarkType)) { continue; }
            sm_MarkPolyEdge(pEdge, eMarkType);
            SmPolyVertex *pV = pEdge->GetStartPolyVertex();
            sm_MarkPolyVertex(pV, eMarkType);
            pV = pEdge->GetEndPolyVertex();
            sm_MarkPolyVertex(pV, eMarkType);
#ifdef SM_DEBUG_CODE
            if (bDebugMe) {
                sm_GraphicsLoop();
                smgfx_SetLook(4,6, 1,0,0); pEdge->Draw(); sm_GraphicsLoop();
                sm_GraphicsLoop();
            }
#endif
            pEdge->GetAllRadials(sRadialEdges);
            ULONG j, lNumRadialEdges = sRadialEdges.GetSize();
            for (j=0; j<lNumRadialEdges; j++) {
                SmPolyFace *pF = sRadialEdges[j]->GetPolyFace();
                if ( pF == NULL )
                  { continue; }

                if (!pF->IsMarked(eMarkType))
                {
                    if (sMap.At(pF) == NULL)
                    {
                        sStack.Add(pF);
                        sMap.Insert(pF,pF);
                    }
                }
            } // end for radial polyEdge
        } // end for each polyEdge in this face
    } // end while stack size > 0

    return;

} // end sm_CollectFaces


/*******************************************************************//**
PURPOSE: Quickly determine whether two faces are coincident

NOTES:
***********************************************************************/
static SmBoolean sm_CheckCoincidentFaces(SmPolyFace * pFace,
                                         SmPolyFace * pOtherFace)
{
    SmTArray<SmPolyEdge*> sEdges, sOtherEdges;
    pFace->GetPolyEdges(sEdges);
    pOtherFace->GetPolyEdges(sOtherEdges);
    double dMaxTol = pFace->GetTolerance();
    if (sEdges.GetSize() == sOtherEdges.GetSize()) {
        SmPoint3d sSumPnt(0.0,0.0,0.0);
        SmPoint3d sOtherSumPnt(0.0,0.0,0.0);
        ULONG ii, lNumEdges = sEdges.GetSize();
        for (ii=0; ii<lNumEdges; ii++) {
            sSumPnt = sSumPnt + sEdges[ii]->GetStartPoint();
            if (sEdges[ii]->GetTolerance() > dMaxTol) {
                dMaxTol = sEdges[ii]->GetTolerance();
            }
            sOtherSumPnt = sOtherSumPnt + sOtherEdges[ii]->GetStartPoint();
            if (sOtherEdges[ii]->GetTolerance() > dMaxTol) {
                dMaxTol = sOtherEdges[ii]->GetTolerance();
            }
        }
        SmPoint3d sAve = sSumPnt/sEdges.GetSize();
        SmPoint3d sAveOther = sOtherSumPnt/sEdges.GetSize();
        double dTol = dMaxTol * 2.0;
        if (sAve.DistanceBetween(sAveOther) > dTol*1000.0) {
            return FALSE; 
        }
    }

    return TRUE;

} // end sm_CheckCoincidentFaces

/*******************************************************************//**
PURPOSE: Classify the face of an edge relative to a radial sector defined by
   a face in the other Brep and its radial mate.

NOTES:
  Output:
   if      (rpRegionOfSector != NULL) - Inside of the sector,
   else if (rpCoincidentFace != NULL) - Coincident with pEdgeOfSector's face
   else - Outside

   Since Regions do not have the same meaning with Polys as they do in
   the Brep world, this is really a yes/no function: if inside a closed Region,
   we return a pointer, and if outside, return Null.

   If pEdgeOfSector is Lamina, and the faces are not coincident
   (i.e., are on opposite sides of the edge), then this will always return
   a Null Region: it's not inside.  You (the caller) might want to check that,
   if it makes a difference in what you're doing.

***********************************************************************/
static SmStatus sm_RadialSectoring( SmPolyEdge    * pEdgeToClassify,  // in: classify this edge's face...
                                    SmPolyEdge    * pEdgeOfSector,    // in: ... against this edge's face and its radial mate.
                                    SmOrientType    eSectorSide,      // in: which side of the sector face
                                    SmPolyRegion *& rpRegionOfSector, // out: non-Null indicates Interior
                                    SmPolyFace   *& rpCoincidentFace) // out: face coincident with pEdgeToClassify's face
{
  // Init outputs.
  rpRegionOfSector = NULL;
  rpCoincidentFace = NULL;

  SmVector3d sVecEdgeToClassify  = pEdgeToClassify->GetEndPoint() - pEdgeToClassify->GetStartPoint();
  SmPolyFace * pFaceToClassify   = pEdgeToClassify->GetPolyFace();
  SmVector3d sNormalToClassify   = pFaceToClassify->GetNormal();
  SmVector3d sBinormalToClassify = sNormalToClassify * sVecEdgeToClassify;

  SmVector3d sVecEdgeOfSector    = pEdgeOfSector->GetEndPoint() - pEdgeOfSector->GetStartPoint();
  SmPolyFace * pFaceOfSector     = pEdgeOfSector->GetPolyFace();
  SmVector3d sNormalOfSector1    = pFaceOfSector->GetNormal();
  SmVector3d sBinormalOfSector1  = sNormalOfSector1 * sVecEdgeOfSector;

  // Need a reasonable angle limit for checking coincidence.  If it's too small,
  // smallish faces can be missed.  Moreover, it doesn't hurt to check with
  // sm_CheckCoincidentFaces(), if they're not coincident, that will tell us so.
  // [B209]
  double dAngLim = SM_DEG2RAD( 2.0 );
  

  if ( pEdgeOfSector->IsLamina() )
  {
      // The edge we're sectoring against is lamina, so it's not part of
      // any containing region, so return NUll for the Region.
      rpRegionOfSector = NULL;

      // But check for coincidence first.
      double dAng;
      SER( sBinormalToClassify.AngleBetween( sBinormalOfSector1, dAng ));
      if ( smos_Fabs( dAng ) < dAngLim )
      {
          // Possible coincidence with pEdgeOfSector's face.
          if ( sm_CheckCoincidentFaces( pFaceToClassify, pFaceOfSector ))
          {
              rpCoincidentFace = pFaceOfSector;
              return SM_SUCCESS;
          }
      }

      // Not coincident, return not-contained.
      return SM_SUCCESS;
  }

  // Not lamina, so more than one sector: do the geometric classification.
  // Method: get binormal vectors into the two faces that bound the sector
  // of pEdgeOfSector, and see whether the binormal into the face to
  // classify lies between them.

  // The radial edge is on the other face, in the same sector.
  // Get its normal and binormal.
  SmPolyEdge * pRadialEdge      = pEdgeOfSector->GetRadial();
  if ( ! pEdgeOfSector->IsManifold() )
    {
      // Spine edge.  Do the work to find the proper radial mate.
      SmOrientType eRadialSide;
      double       dAngle;
      pEdgeOfSector->GetTopologicalRadial( eSectorSide,
                            pRadialEdge, eRadialSide, dAngle );
    }

  SmPolyFace * pRadialFace      = pRadialEdge->GetPolyFace();
  SmVector3d sNormalOfSector2   = pRadialFace->GetNormal();
  SmVector3d sBinormalOfSector2 = sNormalOfSector2 * (-sVecEdgeOfSector);

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
  if (bDebugMe) {
      smgfx_Erase();
      smgfx_SetLook(2,3, 1,0,0); pEdgeToClassify->Draw();      sm_GraphicsLoop();
      smgfx_SetLook(1,2, 0,0,1); pFaceToClassify->DrawDebug(); sm_GraphicsLoop();
      smgfx_SetLook(1,2, 0,0,0); pFaceOfSector->DrawDebug();   sm_GraphicsLoop();
      smgfx_SetLook(3,5, 0,1,0); pRadialFace->DrawDebug();     sm_GraphicsLoop();
      sm_GraphicsLoop();
  }
#endif

  // Check for coincidence with either of the sector bounds.
  double dAng1, dAng2;
  SER( sBinormalToClassify.AngleBetween( sBinormalOfSector1, dAng1 ));
  SER( sBinormalToClassify.AngleBetween( sBinormalOfSector2, dAng2 ));

  if ( smos_Fabs( dAng1 ) < dAngLim )
  {
      // Possible coincident with pEdgeOfSector's face
      if ( sm_CheckCoincidentFaces( pFaceToClassify, pFaceOfSector ))
        { rpCoincidentFace = pFaceOfSector; }
  }
  else if ( smos_Fabs( dAng2 ) < dAngLim )
  {
      // Possible coincident with pRadialEdge's face
      if ( sm_CheckCoincidentFaces( pFaceToClassify, pRadialFace ))
        { rpCoincidentFace = pRadialFace; }
  }

  if ( eSectorSide == SM_OT_OPPOSITE )
    { sNormalOfSector1 *= -1.0; }  // Negated because the face normals point outward.
  
  // Ok, if we haven't found a coincident face,
  // do the geometric test on the binormals.
  if ( rpCoincidentFace == NULL )
  {
      if ( sBinormalToClassify.IsInsideSector(
               sBinormalOfSector1,
               sNormalOfSector1,
               sBinormalOfSector2
        ))
      {
          // Inside sector, return region here
          rpRegionOfSector = pFaceOfSector->GetPolyShell()->GetPolyRegion();
          NER( rpRegionOfSector );
      }
  }

  return SM_SUCCESS;

} // end sm_RadialSectoring


/*******************************************************************//**
PURPOSE: Determine whether a point lies on an edge

NOTES:
***********************************************************************/
static SmBoolean sm_TestPointOnEdge(SmPolyEdge * pEdge,
                                    const SmPoint3d & crTestPoint)
{
    SmPoint3d sEdgeStart = pEdge->GetStartPoint();
    SmPoint3d sEdgeEnd = pEdge->GetEndPoint();

    SmVector3d sLineVec = sEdgeEnd - sEdgeStart;
    double dParam;
    smgu_LineClosestPoint(sEdgeStart,sLineVec,crTestPoint,dParam);
    if (dParam < 0.0 || dParam > 1.0) return FALSE;
    SmPoint3d sFoundPnt = sEdgeStart + dParam * sLineVec;
    double dDistance = sFoundPnt.DistanceBetween(crTestPoint);
    if (dDistance > pEdge->GetTolerance()) return FALSE;

    return TRUE;

} // end sm_TestPointOnEdge

/*******************************************************************//**
PURPOSE: Fire a ray from the given point in the given direction and see what
   we hit.

NOTES: If you hit nothing then the ray is in the infinite region.
   Otherwise it will return the face that was hit.
***********************************************************************/
static SmStatus sm_FireRay
 (SmRayTracer & sRayTracer,  // in : 
  SmPolyBrep  * pBrep,       // NotUsed: in : 
  SmPoint3d   & rRayPoint,   // in : 
  SmVector3d  & rRayVector,  // in : 
  SmPolyFace *& rpFace)      // out: 
{
  SM_REF1(pBrep) ;
    rpFace = NULL;

    SmVector3d sAdjustVec(0.0123,0.0134,0.0132);   // gwc: why does this function try a couple of directions? Should it be changed to just one?
    ULONG ii, jj;
    for (ii=0; ii<2; ii++)
    {
        double sDData[256];
        double sDData2[256];
        SmGridElement *sEData[256];
        SmTArray<double> sMinDistances(256,sDData);
        SmTArray<double> sMaxDistances(256,sDData2);
        SmTArray<SmGridElement*> sGridElements(256,sEData);
        SmSolution sSolution;
        SmBoolean bHitsSomething = FALSE;
        SmVector3d sRay = rRayVector+ii*sAdjustVec;
        SER(sRay.Unitize());
        SER(sRayTracer.FireRay(rRayPoint,sRay,
                               SM_BIG_DOUBLE,
                               bHitsSomething,
                               sSolution,
                               sGridElements,
                               sMinDistances,
                               sMaxDistances));
        if (!bHitsSomething)
          { break; }

        SmVector3d sDir = rRayVector * sSolution.m_vStart[0];
        SmPoint3d sRayEnd = rRayPoint + sDir;
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
        if (bDebugMe) {
            smgfx_SetLook(2,3, 1,0,0); rRayPoint.Draw(); sDir.Draw(&rRayPoint); sm_GraphicsLoop();
            smgfx_SetLook(2,3, 0,0,1); sRayEnd.Draw(); sm_GraphicsLoop();
            sm_GraphicsLoop();
        }
#endif
        // Determine if the ray ends in the interior of the face
        SmPolyFace * pF = SM_CAST_PTR(SmPolyFace,sSolution.m_apObjects[0]);
        
        SmTArray<SmPolyEdge*> sFaceEdges;
        pF->GetPolyEdges(sFaceEdges);
        SmBoolean bTryRayFireAgain = FALSE;
        for (jj=0; jj<sFaceEdges.GetSize(); jj++) {
            if (sm_TestPointOnEdge(sFaceEdges[jj],sRayEnd)) {
                bTryRayFireAgain = TRUE;
                break; // 
            }
        }
        if (bTryRayFireAgain == FALSE) {
            rpFace = pF;
            break;
        }
    }
    
    return SM_SUCCESS;

} // end sm_FireRay

/*******************************************************************//**
PURPOSE: Before a manifold solid volumetric Boolean operation, snap
   vertices of one brep to vertices of the other to help resolving
   near coincidence situations

NOTES: You might want to call this method before boolean operation
   if a regular Boolean of two solids returned some lamina & non-manifold edges.

   This method is not currently called in SMLib.
***********************************************************************/
SmStatus SmPolyMerge::DoVertexSnappingBeforeBoolean(double dSnappingTol)
{
    SmExtent3d sBrepBBox;
    SER(m_vPI.m_pBrep->CalculateBoundingBox(sBrepBBox));
    
    // Try snapping vertices (from m_pBrep) to vertices (from m_pOther).
    {
        SmTree *pVertTree = new (m_vPI.m_crContext) SmTree(sBrepBBox);
        SmObjDelete sClean(pVertTree);
        
        // Now load the VertexTree;
        SM_PTR_ARRAY(sVertices,SmPolyVertex,512);
        m_vPI.m_pBrep->GetPolyVertices(sVertices);
        for(ULONG ii=0; ii<sVertices.GetSize(); ii++)
        {
            SmPolyVertex *pV = sVertices[ii];
            SmExtent3d sVBBox(pV->GetPoint());
            sVBBox.ExpandAbsolute(pV->GetTolerance());
            SER(pVertTree->AddToSpatialTree(sVBBox,pV));
        }
        
        SmSolution sSData[4];
        SmSolutionArray sSolutions(4,sSData);
        double dTol = m_vPI.m_dThisApproxTol3d;
        double dBestAnswerSoFarSq=SM_BIG_DOUBLE;
        SmPolySolver sPS(SM_SO_MINIMIZE,SM_SR_SINGLE,dTol,
            dBestAnswerSoFarSq,NULL,sSolutions);
        SmPLSPointVertex sPSLPV(sPS);
        ULONG lCount=0;
        m_vPI.m_pOther->GetPolyVertices(sVertices);
        ULONG lNumVerts = sVertices.GetSize();
        for(ULONG ii=0; ii<lNumVerts; ii++)
        {
            sSolutions.ReSet();
            
            SmPolyVertex * pV = sVertices[ii];
            SmTreeNode sTreeNode;
            SmObjectList sObjectList;
            SmPoint3d sPoint = pV->GetPoint();
            SmTree sPointTree(sPoint,0,&sTreeNode,&sObjectList);
            
            sPSLPV.SetPoint(sPoint);
            SER(sPS.SolveTrees(sPointTree,*pVertTree,sPSLPV));
            
            // Parse the solutions
            if (sSolutions.GetSize() > 0)
            {
                SmSolution sSolution = sSolutions[0];
                SmPoint3d sSolPnt;
                
                sSolPnt.x = sSolution.m_vStart[3];
                sSolPnt.y = sSolution.m_vStart[4];
                sSolPnt.z = sSolution.m_vStart[5];
                if (sPoint.DistanceBetween(sSolPnt) > dSnappingTol) { continue; }
                
                pV->SetPoint(sSolPnt);
                lCount++;
                
                SM_PTR_ARRAY(sStartEdges,SmPolyEdge,64);
                pV->GetStartingPolyEdges(sStartEdges);
                
                for(ULONG jj=0; jj<sStartEdges.GetSize(); jj++)
                {
                    SmPolyEdge *pE = sStartEdges[jj];
                    SmPolyVertex * pEndV = pE->GetEndPolyVertex();
                    SmPoint3d sEndPnt = pEndV->GetPoint();
                    
                    if (sEndPnt.DistanceBetweenSquared(sSolPnt) < SM_EFF_ZERO_SQ) {
                        // Sqeeze pE;
                        SmPolyEdge *pRadialE = pE->GetRadial();
                        if (pE == pRadialE) {
                            SER(SM_ERR);
                        }
                        m_vPI.m_pOther->DeletePolyFace(pE->GetPolyFace());
                        m_vPI.m_pOther->DeletePolyFace(pRadialE->GetPolyFace());
                        SM_PTR_ARRAY(sEdgesBetween,SmPolyEdge,4);
                        SER(m_vPI.m_pOther->GlueVertices(pEndV,pV,sEdgesBetween));
                        break;
                    }
                }
                continue;

            } // end if SolveTrees found a solution
        } // end for each vertex

        ULONG lNumEdgesStitched = 0;
        ULONG lNumberLaminaRemaining = 0;
        dTol = m_vPI.m_pOther->GetTolerance();

        SER( m_vPI.m_pOther->Stitch( dTol,                      // note: increments unlocked mark value
                                  TRUE, FALSE, FALSE,
                                  lNumEdgesStitched,
                                  lNumberLaminaRemaining ));

    } // End snapping vertices (from m_pBrep) to vertices (from m_pOther).
    
    // Try snapping vertices to edges.
    {
        SmTree *pEdgeTree = new (m_vPI.m_crContext) SmTree(sBrepBBox);
        pEdgeTree->GetTopNode()->m_eGeomType = SM_NG_LINE_SEG;
        SmObjDelete sClean(pEdgeTree);
        
        // Now load the EdgeTree 
        SM_PTR_ARRAY(sEdges,SmPolyEdge,512);
        m_vPI.m_pBrep->GetPolyEdges(sEdges);

        for(ULONG ii=0; ii<sEdges.GetSize(); ii++) {
            SmPolyEdge *pE = sEdges[ii];
            SmExtent3d sEBBox;
            SER(pE->CalculateBoundingBox(sEBBox));
            sEBBox.ExpandAbsolute(pE->GetTolerance());
            SER(pEdgeTree->AddToSpatialTree(sEBBox,pE,SM_NG_LINE_SEG));
        }

        SmSolution sSData[4];
        SmSolutionArray sSolutions(4,sSData);
        double dTol = m_vPI.m_dThisApproxTol3d;
        double dBestAnswerSoFarSq=SM_BIG_DOUBLE;
        SmPolySolver sPS(SM_SO_MINIMIZE,SM_SR_SINGLE,dTol,
            dBestAnswerSoFarSq,NULL,sSolutions);
        SmPLSPointEdge sPLSPE(sPS);
        ULONG lCount=0;
        SM_PTR_ARRAY(sVertices,SmPolyVertex,512);
        m_vPI.m_pOther->GetPolyVertices(sVertices);
        for(ULONG ii=0; ii<sVertices.GetSize(); ii++) {
            sSolutions.ReSet();
            
            SmPolyVertex * pV = sVertices[ii];
            SmTreeNode sTreeNode;
            SmObjectList sObjectList;
            SmPoint3d sPoint = pV->GetPoint();
            SmTree sPointTree(sPoint,0,&sTreeNode,&sObjectList);
            
            sPLSPE.SetPoint(sPoint);
            SER(sPS.SolveTrees(sPointTree,*pEdgeTree,sPLSPE));
            
            // Parse the solutions
            if (sSolutions.GetSize() > 0) {
                SmSolution sSolution = sSolutions[0];
                SmPoint3d sSolPnt;
                
                sSolPnt.x = sSolution.m_vStart[3];
                sSolPnt.y = sSolution.m_vStart[4];
                sSolPnt.z = sSolution.m_vStart[5];
                if (sPoint.DistanceBetween(sSolPnt) > dSnappingTol) { continue; }
                
                pV->SetPoint(sSolPnt);
                lCount++;
                
                SM_PTR_ARRAY(sStartEdges,SmPolyEdge,64);
                pV->GetStartingPolyEdges(sStartEdges);
                
                for(ULONG jj=0; jj<sStartEdges.GetSize(); jj++) {
                    SmPolyEdge *pE = sStartEdges[jj];
                    SmPolyVertex * pEndV = pE->GetEndPolyVertex();
                    SmPoint3d sEndPnt = pEndV->GetPoint();
                    
                    if (sEndPnt.DistanceBetweenSquared(sSolPnt) < dSnappingTol) {
                        // Sqeeze pE;
                        SmPolyEdge *pRadialE = pE->GetRadial();
                        if (pE == pRadialE) {
                            SER(SM_ERR);
                        }
                        m_vPI.m_pOther->DeletePolyFace(pE->GetPolyFace());
                        m_vPI.m_pOther->DeletePolyFace(pRadialE->GetPolyFace());
                        SM_PTR_ARRAY(sEdgesBetween,SmPolyEdge,4);
                        SER(m_vPI.m_pOther->GlueVertices(pEndV,pV,sEdgesBetween));
                        break;
                    }
                }
                continue;
            }
        }

    } // End snapping vertices to edges.
        
    // Try snapping vertices to Faces.
    {
        SmTree *pFaceTree = new (m_vPI.m_crContext) SmTree(sBrepBBox);
        pFaceTree->GetTopNode()->m_eGeomType = SM_NG_POLYGON;
        SmObjDelete sClean(pFaceTree);
        
        // Now load the FaceTree;
        SmTArray<SmPolyFace*> sPolyFaces;
        m_vPI.m_pBrep->GetPolyFaces(sPolyFaces);
        for (ULONG ii=0; ii<sPolyFaces.GetSize(); ii++) {
            SmPolyFace *pFace = sPolyFaces[ii];
            SmExtent3d sFBBox;
            SER(pFace->CalculateBoundingBox(sFBBox));
            SER(pFaceTree->AddToSpatialTree(sFBBox,pFace,SM_NG_POLYGON));
        }
        
        SmSolution sSData[4];
        SmSolutionArray sSolutions(4,sSData);
        double dTol = m_vPI.m_dThisApproxTol3d;
        double dBestAnswerSoFarSq=SM_BIG_DOUBLE;
        SmPolySolver sPS(SM_SO_MINIMIZE,SM_SR_SINGLE,dTol,
            dBestAnswerSoFarSq,NULL,sSolutions);
        SmPLSPointFace sPLSPF(sPS);
        ULONG lCount=0;
        SM_PTR_ARRAY(sVertices,SmPolyVertex,512);
        m_vPI.m_pOther->GetPolyVertices(sVertices);
        for(ULONG ii=0; ii<sVertices.GetSize(); ii++)
        {
            sSolutions.ReSet();
            
            SmPolyVertex * pV = sVertices[ii];
            SmTreeNode sTreeNode;
            SmObjectList sObjectList;
            SmPoint3d sPoint = pV->GetPoint();
            SmTree sPointTree(sPoint,0,&sTreeNode,&sObjectList);
            
            sPLSPF.SetPoint(sPoint);
            SER(sPS.SolveTrees(sPointTree,*pFaceTree,sPLSPF));
            
            // Parse the solutions
            if (sSolutions.GetSize() > 0)
            {
                SmSolution sSolution = sSolutions[0];
                SmPoint3d sSolPnt;
                
                sSolPnt.x = sSolution.m_vStart[3];
                sSolPnt.y = sSolution.m_vStart[4];
                sSolPnt.z = sSolution.m_vStart[5];
                if (sPoint.DistanceBetween(sSolPnt) > dSnappingTol) { continue; }
                
                pV->SetPoint(sSolPnt);
                lCount++;
                
                SM_PTR_ARRAY(sStartEdges,SmPolyEdge,64);
                pV->GetStartingPolyEdges(sStartEdges);
                
                for(ULONG jj=0; jj<sStartEdges.GetSize(); jj++)
                {
                    SmPolyEdge *pE = sStartEdges[jj];
                    SmPolyVertex * pEndV = pE->GetEndPolyVertex();
                    SmPoint3d sEndPnt = pEndV->GetPoint();
                    
                    if (sEndPnt.DistanceBetweenSquared(sSolPnt) < dSnappingTol) {
                        // Sqeeze pE;
                        SmPolyEdge *pRadialE = pE->GetRadial();
                        if (pE == pRadialE) {
                            SER(SM_ERR);
                        }
                        m_vPI.m_pOther->DeletePolyFace(pE->GetPolyFace());
                        m_vPI.m_pOther->DeletePolyFace(pRadialE->GetPolyFace());
                        SM_PTR_ARRAY(sEdgesBetween,SmPolyEdge,4);
                        SER(m_vPI.m_pOther->GlueVertices(pEndV,pV,sEdgesBetween));
                        break;
                    }
                } // end for each StartEdge

                continue;

            } // end if SolveTrees found a solution
        } // end for eachg vertex

#ifdef SM_DEBUG_CODE
        m_vPI.m_pOther->Dump();
        m_vPI.m_pOther->ValidatePointers();
#endif

        // Stitch m_pOther.
        ULONG lNumEdgesStitched = 0;
        ULONG lNumberLaminaRemaining = 0;
        dTol = m_vPI.m_pOther->GetTolerance();
        SER(m_vPI.m_pOther->Stitch( dTol,                      // note: increments unlocked mark value
                                 TRUE, FALSE, FALSE,
                                 lNumEdgesStitched, 
                                 lNumberLaminaRemaining ));

#ifdef SM_DEBUG_CODE
        m_vPI.m_pOther->Dump();
        m_vPI.m_pOther->ValidatePointers();
#endif


    } // End snapping vertices to Faces.
        
    // Recompute all face normals
    SM_PTR_ARRAY(sFaces,SmPolyFace,512);
    m_vPI.m_pOther->GetPolyFaces(sFaces);

    for(ULONG ii=0; ii<sFaces.GetSize(); ii++) {
        SmPolyFace *pPFace = sFaces[ii];
        pPFace->GetNormal(TRUE,TRUE);
    }

    return SM_SUCCESS;

} // end DoVertexSnappingBeforeBoolean

/*******************************************************************//**
PURPOSE: Perform a manifold solid volumetric Boolean operation on 
    the two breps.  It also performs a general merge operation which
    combines the topology of two breps.  

NOTES: You should use this method if you wish to do Boolean
    operations between two solids.   
***********************************************************************/
SmStatus SmPolyMerge::ManifoldBoolean                                             
 (SmPolyBooleanOperationType eOperation,         // in : one of SM_PBO_UNION,        SM_PBO_IMPRINT
                                                 //             SM_PBO_MERGE,        SM_PBO_PARTIAL_MERGE
                                                 //             SM_PBO_INTERSECTION, SM_PBO_EXTRACT_SEPARATE
                                                 //             SM_PBO_DIFFERENCE,                    
  SmPolyBrep              *& rpResult,           // out: Resulting PolyBrep
  SmMarkType               * pOptBrepMarkType,   // in : Optional Mark used for Brep,  NULL to ignore, default:[NULL]
  SmMarkType               * pOptOtherMarkType)  // in : Optional Mark used for Other, NULL to ignore, default:[NULL]
{

  m_vPI.m_pBrep->Notify(SM_NO_PRE_EDIT, m_vPI.m_pBrep, NULL, NULL);
  m_vPI.m_pOther->Notify(SM_NO_PRE_EDIT, m_vPI.m_pOther, NULL, NULL);

  // Do the intersections.
  SER(m_vPI.IntersectInsertRelate());

  // If we are imprinting just delete the second brep and return here.
  if( eOperation == SM_PBO_IMPRINT || m_bImprinting ) 
    {
      SM_ASSERT(m_vPI.m_pOther != NULL) ; delete m_vPI.m_pOther ; m_vPI.m_pOther = NULL ;
      rpResult = m_vPI.m_pBrep;
      return SM_SUCCESS;
    }

#ifdef SM_VALIDATE_TOPOLOGY
  m_vPI.m_pBrep->ValidatePointers();
  m_vPI.m_pOther->ValidatePointers();
#endif // SM_VALIDATE_TOPOLOGY
    
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
  if (bDebugMe) 
    {
      m_vPI.m_pBrep->Dump();
      m_vPI.m_pOther->Dump();
      m_vPI.Dump();
      smgfx_Erase();
      smgfx_SetLook( 1,2, 0,0,1 ); m_vPI.m_pBrep-> Draw(TRUE); sm_GraphicsLoop();
      smgfx_SetLook( 1,2, 0,1,0 ); m_vPI.m_pOther->Draw(TRUE); sm_GraphicsLoop();
      smgfx_SetLook( 1,2, 1,0,1 ); m_vPI.DrawRelatedObjects(); sm_GraphicsLoop();
      sm_GraphicsLoop();
#ifdef SM_VALIDATE_TOPOLOGY
      m_vPI.m_pOther->ValidatePointers();
      m_vPI.m_pBrep->ValidatePointers();
#endif // SM_VALIDATE_TOPOLOGY
    }
#endif // SM_DEBUG_CODE
    
  SmPolyEdge * sPE1Data[64];
  SmPolyEdge * sPE2Data[64];
  SmPolyEdge * sPE3Data[64];
  SmPolyEdge * sPE4Data[64];
  SmPolyVertex * sPV1Data[64];
  SmPolyVertex * sPV2Data[64];
  SmPolyVertex * sPV3Data[64];
  SmTArray<SmPolyEdge*> sEdges(64,sPE1Data);
  SmTArray<SmPolyEdge*> sOtherEdges(64,sPE2Data);
  SmTArray<SmPolyEdge*> sFaceEdges(64,sPE3Data);
  SmTArray<SmPolyEdge*> sOFaceEdges(64,sPE4Data);
  SmTArray<SmPolyVertex*> sVertices(64,sPV1Data);
  SmTArray<SmPolyVertex*> sOtherVertices(64,sPV2Data);
  SmTArray<SmPolyVertex*> sSingleVertices(64,sPV3Data);

  m_vPI.GetCommonEdges(sEdges,sOtherEdges);
  m_vPI.GetCommonVertices(sVertices,sOtherVertices);

  // increment and lock any unlocked mark for both pBrepContext and pOtherContext
  SmContext *pBrepContext  = (SmContext *)m_vPI.m_pBrep->GetContext() ;
  SmContext *pOtherContext = (SmContext *)m_vPI.m_pOther->GetContext() ;

  SmNewMarkAndLock sMarkLock(pBrepContext, pOtherContext) ;
  SmMarkType       eBrepMarkType  = sMarkLock.GetMarkType1() ;
  SmMarkType       eOtherMarkType = sMarkLock.GetMarkType2() ;

  if(pOptBrepMarkType)  { * pOptBrepMarkType  = eBrepMarkType ;  }
  if(pOptOtherMarkType) { * pOptOtherMarkType = eOtherMarkType ; }

  SmBoolean bTryReintersect = FALSE;

  // For each pair of related vertices,
  // - count the number of related edges connected to it
  // - if exactly one, then add the vertex to sSingleVertices array
  //   and set bTryReintersect = TRUE.
  // Then, if any such vertices are found, try to FixGaps.
  // If FixGaps does nothing, IntersectInsertRelate again.
  ULONG ii, lNumVerts = sVertices.GetSize();
  for (ii=0; ii<lNumVerts; ii++)
    {
      ULONG lNumInt = 0;
      SmPolyVertex * pVertex      = sVertices[ii];
      SmPolyVertex * pOtherVertex = sOtherVertices[ii];

      pVertex->GetPolyEdges(sFaceEdges);
      ULONG ie, lNumFaceEdges = sFaceEdges.GetSize();
      for (ie=0; ie<lNumFaceEdges; ie++) 
        {
          SmPolyEdge *pE = sFaceEdges[ie];
          if(m_vPI.GetOtherMate(pE) != NULL) 
            { lNumInt ++; }
        }

      if (lNumInt == 1) 
        {
          sSingleVertices.Add(pVertex);
          bTryReintersect = TRUE;
        }

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe1 = FALSE;
      if (bDebugMe1) 
        {
          if (ii==0)
            { smgfx_Erase(); }
          if (lNumInt == 1)
            { smgfx_SetLook(1,4, 1,0,1); }
          else if (lNumInt > 2)
            { smgfx_SetLook(1,4, 0,1,1); }
          else
            { smgfx_SetLook(1,2, 0,0,1); }

          pVertex->GetPoint().Draw(); sm_GraphicsLoop();
          pOtherVertex->GetPoint().Draw(); sm_GraphicsLoop();
          sm_GraphicsLoop();
      }
#endif // SM_DEBUG_CODE

      sm_MarkPolyVertex(pVertex,      eBrepMarkType);
      sm_MarkPolyVertex(pOtherVertex, eOtherMarkType);

    } // end for each pair of related vertices

  SmStatus eStat;

  if (bTryReintersect)
    {
      SmBoolean bModifiedTopology;
      eStat = m_vPI.FixGaps( sSingleVertices, bModifiedTopology ); // (was SER [B105])
      if ( eStat == SM_SUCCESS && bModifiedTopology )
        {
          bTryReintersect = FALSE;
          m_vPI.GetCommonEdges(sEdges,sOtherEdges);
          m_vPI.GetCommonVertices(sVertices,sOtherVertices);
          sMarkLock.NewMark();  // new mark for both m_pBrep and m_pOther
        }
    }

  // Don't do this second call.  The Breps have been modified,
  // and this can lead to a crash.  [B252]
  // Also, we have seen no cases where it helps.
//  if (bTryReintersect)
//    {
//      eStat = m_vPI.IntersectInsertRelate(); // (was SER)
//      m_vPI.GetCommonEdges(sEdges,sOtherEdges);
//      m_vPI.GetCommonVertices(sVertices,sOtherVertices);
//      sBrepMarkLock.NewMark();
//      sOtherMarkLock.NewMark();
//    }

  // Check for face containment.
  SmBoolean bFaceIsOutsideOther = TRUE;

  // Collect sets of manifold sheets/shells for each PolyBrep.
  SmTArray< SmTArray<SmPolyFace*>  *>      sCollectedFaceLists1;
  SmTArray< SmTArray<SmOrientType> *>      sCollectedSideLists1;
  SmTArray< SmTArray<SmPolyFace*>  *>      sCollectedFaceLists2;
  SmTArray< SmTArray<SmOrientType> *>      sCollectedSideLists2;
  SmObjsDelete < SmTArray<SmPolyFace*> *>  sCleanCFL1( &sCollectedFaceLists1 ), sCleanCFL2( &sCollectedFaceLists2 );
  SmObjsDelete < SmTArray<SmOrientType> *> sCleanCSL1( &sCollectedSideLists1 ), sCleanCSL2( &sCollectedSideLists2 );
  
  SER( m_vPI.m_pBrep->CollectPolyFaceShells( sCollectedFaceLists1, 
                                             sCollectedSideLists1 )); // note: increments two unlocked mark values for nonManifold PolyBreps

  
  SER( m_vPI.m_pOther->CollectPolyFaceShells( sCollectedFaceLists2, 
                                              sCollectedSideLists2 )); // note: increments two unlocked mark values for nonManifold PolyBreps

  // Note, the face collections can contain other-brep faces only if
  // they're closed.  Remove any non-closed collections.
  // A collection is closed iff it has no nonmanifold edges.
  sm_RemoveNonmanifoldFaceSets( sCollectedFaceLists1, sCollectedSideLists1 );
  sm_RemoveNonmanifoldFaceSets( sCollectedFaceLists2, sCollectedSideLists2 );

  // Done checking for single-edge vertices.

  // Mark each common edge (pair).
  ULONG lNumEdges = sEdges.GetSize();
  for (ii=0; ii<lNumEdges; ii++)
    {
      SmPolyEdge *pEdge = sEdges[ii];
      SmPolyEdge *pOtherEdge = sOtherEdges[ii];
      sm_MarkPolyEdge(pEdge, eBrepMarkType);
      sm_MarkPolyEdge(pOtherEdge, eOtherMarkType);

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe1 = FALSE;
      if (bDebugMe1) 
        {
          if (ii==0)
            { smgfx_Erase(); }
          smgfx_SetLook(1,2, 1,0,0);
          pEdge->Draw();      sm_GraphicsLoop();
          pOtherEdge->Draw(); sm_GraphicsLoop();
          sm_GraphicsLoop();
        }
#endif // SM_DEBUG_CODE
    } // end iter every edge - marking common edges

  // Now we decide which faces to keep and which to delete.
  // For the first pass, we process all faces that are connected
  // to common edges -- those that are intersections of faces.
  // (All faces have been intersected, and the faces have been
  // split at those intersections, which are the common edges.)
  // For each common edge, we classify each face in one Brep
  // as being inside or outside the other Brep.  We then put
  // the face into the appropriate Keep or Delete list, which
  // depends on which Boolean operation is taking place.
  // We also do a face traversal to find all other faces that
  // are connected to the face in question, in the same region,
  // and put those collected faces into the Keep or Delete lists
  // as well.

  SmPolyFace * sPF1Data[64];
  SmPolyFace * sPF2Data[64];
  SmPolyFace * sPF3Data[64];
  SmPolyFace * sPF4Data[64];
  SmPolyFace * sPF5Data[64];
  SmPolyEdge * sPREData[64];
  SmTArray<SmPolyFace*> sKeepFaces(64,sPF1Data);
  SmTArray<SmPolyFace*> sKeepOFaces(64,sPF2Data);
  SmTArray<SmPolyFace*> sDelFaces(64,sPF3Data);
  SmTArray<SmPolyFace*> sDelOFaces(64,sPF4Data);
  SmTArray<SmPolyFace*> sCollectedFaces(64,sPF5Data);
  SmTArray<SmPolyEdge*> sRadialEdges(64,sPREData);

  // Cache sizes for new PolyBrep, in SM_PBO_EXTRACT_SEPARATE case.
  ULONG lNum, lNumV, lNumE, lNumL, lNumCF, lNumF, lNumS, lNumR;

  // For each common edge, first classify the faces around Brep's edge
  // against the face sectors of the Other edge,
  // then do each Other edge's faces agains Brep's edge's sectors.

  lNumEdges = sEdges.GetSize();
  for (ii=0; ii<lNumEdges; ii++)
    {
      SmPolyEdge *pEdge = sEdges[ii];
      SmPolyEdge *pOtherEdge = sOtherEdges[ii];
        
      // First classify the faces around Brep's edge
      // against the face sectors of the Other edge,
      // Get the faces incident on this edge via the radial edges here.
      pEdge->GetAllRadials( sRadialEdges );

      // Classify Edge's radial faces against OtherEdge.
      ULONG jj, lNumRadials = sRadialEdges.GetSize();
      for (jj=0; jj<lNumRadials; jj++)
        {
          SmPolyEdge *pRadialE = sRadialEdges[jj];
          SmPolyFace *pFace = pRadialE->GetPolyFace();

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe2 = FALSE;
          if (bDebugMe2) 
            {
              smgfx_Erase();
              if ( pFace->IsMarked(eBrepMarkType)) { smgfx_SetLook(4,6, 0,1,0); }
              else                                 { smgfx_SetLook(4,6, 1,0,0); }
                                         pRadialE->Draw();       sm_GraphicsLoop();
              smgfx_SetLook(3,5, 0,0,1); pFace->DrawDebug();     sm_GraphicsLoop();
              SmPolyFace *pOtherPFace = pOtherEdge->GetPolyFace();
              smgfx_SetLook(3,5, 0,1,0); pOtherPFace->DrawDebug();   sm_GraphicsLoop();
              smgfx_SetLook(1,2, 0,1,0); m_vPI.m_pOther->Draw(TRUE); sm_GraphicsLoop();
              smgfx_SetLook(1,2, 0,0,1); m_vPI.m_pBrep->Draw(TRUE);  sm_GraphicsLoop();
              sm_GraphicsLoop();
            }
#endif // SM_DEBUG_CODE

          // If the face is marked we already classified it
          if (pFace->IsMarked(eBrepMarkType)) { continue; }

          SmPolyRegion * pRegionOfSector = NULL;
          SmPolyFace   * pCoincidentFace = NULL;

          SmPolyFace   * pThisPFace      = pRadialE->GetPolyFace();
          SmPolyFace   * pOtherPFace     = pOtherEdge->GetPolyFace();

          bFaceIsOutsideOther = TRUE;  // init.

          // Note, the following is obsolete, because we now ensure that
          // the radials are all in the proper geometric order.
          // Moreover, classifying against remote sectors can give
          // incorrect results. [B255]
          //// Classify this face against the sectors around the other edge.
          //// Check against all sectors (in case of spine edge).  [B158]
          //SmTArray< SmPolyEdge* > sOtherRadials;
          //pOtherEdge->GetAllRadials( sOtherRadials );
          //ULONG kk, lNumOtherRads = sOtherRadials.GetSize();
          //for ( kk=0; kk < lNumOtherRads; kk += 2 )
          //  {
          //    SmPolyEdge *pOtherRad = sOtherRadials[kk];
          //    SER( sm_RadialSectoring( pRadialE, pOtherRad, pRegionOfSector, pCoincidentFace ));
          //    if ( pRegionOfSector != NULL || pCoincidentFace != NULL )
          //      { break; }
          //  }

          // Do the classification.  pRadialE's PFace can be inside OtherBrep only if
          // pOtherPFace is part of a closed shell of PFaces, in which case it will
          // be in the collected face/side lists.

          // Note, sm_RadialSectoring() also checks for coincidence.
          // If we don't call that, we have to check coincidence here.
          SmBoolean bDidRadialSectoring = FALSE;

          // Check Opposite side first, that's the usual case.
          if ( bFaceIsOutsideOther )
            {
              if(sm_ContainsFaceSide( pOtherPFace, SM_OT_OPPOSITE,
                                      sCollectedFaceLists2, sCollectedSideLists2 ) )
                {
                  bDidRadialSectoring = TRUE;
                  SER( sm_RadialSectoring( pRadialE, pOtherEdge, SM_OT_OPPOSITE,
                                           pRegionOfSector, pCoincidentFace ));
                  bFaceIsOutsideOther = ( pRegionOfSector == NULL );
                }
            }

          // If that didn't find it to be interior, check Same side.
          if ( bFaceIsOutsideOther && pCoincidentFace == NULL )
            {
              if ( sm_ContainsFaceSide( pOtherPFace, SM_OT_SAME,
                                        sCollectedFaceLists2, sCollectedSideLists2 ) )
                {
                  bDidRadialSectoring = TRUE;
                  SER( sm_RadialSectoring( pRadialE, pOtherEdge, SM_OT_SAME,
                                           pRegionOfSector, pCoincidentFace ));
                  bFaceIsOutsideOther = ( pRegionOfSector == NULL );
                }
            }

          if ( bDidRadialSectoring == FALSE )
            {
              // Check for coincidence.
              double dAng, dAngLim = SM_DEG2RAD( 2.0 ); // as in sm_RadialSectoring...
              SmVector3d sBinormalToClassify = pRadialE  ->ComputeBinormal();
              SmVector3d sBinormalOfSector   = pOtherEdge->ComputeBinormal();
              SER( sBinormalToClassify.AngleBetween( sBinormalOfSector, dAng ));
              if ( smos_Fabs( dAng ) < dAngLim )
                {
                  // Possible coincidence with pEdgeOfSector's face.
                  if ( sm_CheckCoincidentFaces( pThisPFace, pOtherPFace ))
                    {
                      pCoincidentFace = pOtherPFace;
                    }
                }
            }

          // State: we have classified pThisPFace against OtherBrep:
          // - pCoincidentFace is set if appropriate,
          // - otherwise bFaceIsOutsideOther is set appropriately.

          if (pCoincidentFace == NULL)
            {
              // Usual case, no coincidences.

              // Collect adjacent faces, they all classify the same.
              sm_CollectFaces( pFace, sCollectedFaces, eBrepMarkType );  // Will mark:[eBrepMarkType] faces

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe6 = FALSE;
              if (bDebugMe6) 
                {
                  smgfx_Erase();
                  smgfx_SetLook( 2,3, 1,0,0 ); pFace->DrawDebug(); sm_GraphicsLoop();
                  smgfx_SetLook( 1,2, 0,0,1 );
                  for (ULONG ifac=0; ifac<sCollectedFaces.GetSize(); ifac++) 
                    {
                      sCollectedFaces[ifac]->DrawDebug(); sm_GraphicsLoop();
                      sm_GraphicsLoop();
                    }
                }   
#endif // SM_DEBUG_CODE
              if (eOperation == SM_PBO_EXTRACT_SEPARATE)
                {
                  lNum = sCollectedFaces.GetSize();
                  lNumV=lNum*4;
                  lNumE=lNum*4;
                  lNumL=lNum*2;
                  lNumCF=lNum;
                  lNumF=lNum*2;
                  lNumS=lNum*2;
                  lNumR=8;
                  SmPolyBrep *pPolyBrep = new (*m_vPI.m_pBrep->GetContext()) SmPolyBrep(m_vPI.m_pBrep->GetTolerance(),
                                                                                        lNumV + lNumE + lNumL +
                                                                                        lNumCF + lNumF + lNumS +
                                                                                        lNumR);
                  m_vPI.m_pBrep->CopyFaces(sCollectedFaces,pPolyBrep, eBrepMarkType);
                  m_vOneBrepPer_OrigBrepConnectedFaceSet.Add(pPolyBrep);
                  continue;
                }

              switch (eOperation) 
                {
                  case SM_PBO_UNION        :
                  case SM_PBO_DIFFERENCE   : if ( bFaceIsOutsideOther ) { sKeepFaces.Append(sCollectedFaces); }
                                             else                       { sDelFaces.Append(sCollectedFaces); }
                                             break;
                  case SM_PBO_INTERSECTION : if ( bFaceIsOutsideOther ) { sDelFaces.Append(sCollectedFaces); }
                                             else                       { sKeepFaces.Append(sCollectedFaces); }
                                             break;
                  case SM_PBO_PARTIAL_MERGE:
                  case SM_PBO_MERGE        : sKeepFaces.Append(sCollectedFaces);
                                             break;
                  case SM_PBO_EXTRACT_SEPARATE:
                  case SM_PBO_IMPRINT         : break;
                } // end switch on keep/delete faces
            } // end pCoincidentFace == NULL branch
          else // pCoincidentFace != NULL
            {
              // Found a coincident face.

              if (pCoincidentFace->IsMarked(eOtherMarkType)) { continue; }

              if (eOperation == SM_PBO_EXTRACT_SEPARATE) 
                {
                  sCollectedFaces.ReSet(); 
                  sCollectedFaces.Add(pFace);

                  sEdges.RemoveAll();
                  pFace->GetPolyEdges( sEdges );
                  lNum = sEdges.GetSize();
                  lNumV=lNum*2;
                  lNumE=lNum*2;
                  lNumL=8;
                  lNumCF=8;
                  lNumF=8;
                  lNumS=8;
                  lNumR=8;

                  SmPolyBrep *pPolyBrep = new (*m_vPI.m_pBrep->GetContext()) SmPolyBrep(m_vPI.m_pBrep->GetTolerance(),
                                                                                        lNumV + lNumE + lNumL +
                                                                                        lNumCF + lNumF + lNumS +
                                                                                        lNumR);

                  m_vPI.m_pBrep->CopyFaces(sCollectedFaces,pPolyBrep, eBrepMarkType);
                  m_vOneBrepPer_OrigBrepConnectedFaceSet.Add(pPolyBrep);
                  continue;
                }

              SER(m_vPI.Relate(pFace,pCoincidentFace));
#ifdef SM_DEBUG_CODE    
SmBoolean bDebugMe3 = FALSE;
              if (bDebugMe3) 
                {
                  this->Dump();
                  smgfx_Erase();
                  smgfx_SetLook( 1,2, 0,1,1 ); m_vPI.DrawRelatedObjects();   sm_GraphicsLoop();
                  smgfx_SetLook( 1,2, 0,0,1 ); pFace->DrawDebug();           sm_GraphicsLoop();
                  smgfx_SetLook( 1,2, 0,1,0 ); pCoincidentFace->DrawDebug(); sm_GraphicsLoop();
                  sm_GraphicsLoop();
                }
#endif // SM_DEBUG_CODE
 
              pFace->Mark(eBrepMarkType);
              pCoincidentFace->Mark(eOtherMarkType);

              // Determine if two faces are on the same side
              SmVector3d sNorm       = pFace->GetNormal();
              SmVector3d sOtherNorm  = pCoincidentFace->GetNormal();
              SmBoolean  bOnSameSide = TRUE;
              if (sNorm.Dot(sOtherNorm) < 0.0) 
                { bOnSameSide =  FALSE; }
              switch (eOperation) 
                {
                  case SM_PBO_UNION           :
                  case SM_PBO_INTERSECTION    : if (bOnSameSide) { sKeepFaces.Add(pFace); sDelOFaces.Add(pCoincidentFace); }
                                                else             { sDelFaces.Add(pFace);  sDelOFaces.Add(pCoincidentFace); }
                                                break;
                  case SM_PBO_DIFFERENCE      : if (bOnSameSide) { sDelFaces.Add(pFace); sDelOFaces.Add(pCoincidentFace); }
                                                else { sKeepFaces.Add(pFace); sDelOFaces.Add(pCoincidentFace); }
                                                break;
                  case SM_PBO_PARTIAL_MERGE   :
                  case SM_PBO_MERGE           : sKeepFaces.Add(pFace); sDelOFaces.Add(pCoincidentFace);
                                                break;
                  case SM_PBO_EXTRACT_SEPARATE:
                  case SM_PBO_IMPRINT         : break;
                } // end switch keep/delete OtherFaces
            } // end  end pCoincidentFace != NULL branch
        } // For jj each radial face around this edge

      // Now classify the faces around OtherBrep's edge
      // against the face sectors of the Brep's edge,
        
      pOtherEdge->GetAllRadials(sRadialEdges);

      // Classify Edge's radials against OtherEdge
      lNumRadials = sRadialEdges.GetSize();
      for (jj=0; jj<lNumRadials; jj++)
        {
          SmPolyEdge *pRadialE = sRadialEdges[jj];
          SmPolyFace *pOFace = pRadialE->GetPolyFace();

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe77 = FALSE;
          if (bDebugMe77) 
            {
              SmPolyFace *pThisPFace  = pEdge->GetPolyFace();

              smgfx_Erase();
              if ( pOFace->IsMarked(eOtherMarkType)) { smgfx_SetLook(2,4, 0,1,0); }
              else                                   { smgfx_SetLook(2,4, 1,0,0); }
                                         pRadialE->Draw();       sm_GraphicsLoop();
              smgfx_SetLook(3,5, 0,1,0); pOFace->DrawDebug();    sm_GraphicsLoop();
              smgfx_SetLook(3,5, 0,0,1); pThisPFace->DrawDebug();    sm_GraphicsLoop();
              smgfx_SetLook(1,2, 0,0,1); m_vPI.m_pBrep->Draw(TRUE);  sm_GraphicsLoop();
              smgfx_SetLook(1,2, 0,1,0); m_vPI.m_pOther->Draw(TRUE); sm_GraphicsLoop();
              sm_GraphicsLoop();
          }
#endif // SM_DEBUG_CODE

          // If the face is marked we already classified it
          if (pOFace->IsMarked(eOtherMarkType)) { continue; }

          SmPolyRegion * pRegionOfSector = NULL;
          SmPolyFace   * pCoincidentFace = NULL;

          SmPolyFace   * pThisPFace      = pEdge->GetPolyFace();
          SmPolyFace   * pOtherPFace     = pRadialE->GetPolyFace();

          bFaceIsOutsideOther = TRUE;  // init.

          // Note, the following is obsolete, because we now ensure that
          // the radials are all in the proper geometric order.
          // Moreover, classifying against remote sectors can give
          // incorrect results. [B255]
          //// Classify other face against the sectors around the this edge.
          //// Check against all sectors (in case of spine edge).  [B158]
          //SmTArray< SmPolyEdge* > sThisRadials;
          //pEdge->GetAllRadials( sThisRadials );
          //ULONG kk, lNumThisRads = sThisRadials.GetSize();
          //for ( kk=0; kk < lNumThisRads; kk += 2 )
          //  {
          //    SmPolyEdge *pThisEdge = sThisRadials[kk];
          //    SER( sm_RadialSectoring( pRadialE, pThisEdge, pRegionOfSector, pCoincidentFace ));
          //    if ( pRegionOfSector != NULL || pCoincidentFace != NULL )
          //      { break; }
          //  }

          // Do the classification.  pRadialE's PFace can be inside Brep only if
          // pThisPFace is part of a closed shell of PFaces, in which case it will
          // be in the collected face/side lists.

          // Note, sm_RadialSectoring() also checks for coincidence.
          // If we don't call that, we have to check coincidence here.
          SmBoolean bDidRadialSectoring = FALSE;

          // Check Opposite first, that's the usual case.
          if ( bFaceIsOutsideOther )
            {
              if(sm_ContainsFaceSide(pThisPFace, SM_OT_OPPOSITE,
                                     sCollectedFaceLists1, sCollectedSideLists1 ) )
                {
                  bDidRadialSectoring = TRUE;
                  SER( sm_RadialSectoring( pRadialE, pEdge, SM_OT_OPPOSITE,
                                           pRegionOfSector, pCoincidentFace ));
                  bFaceIsOutsideOther = ( pRegionOfSector == NULL );
                }
            }

          // If that didn't find it to be interior:
          if ( bFaceIsOutsideOther && pCoincidentFace == NULL )
            {
              if ( sm_ContainsFaceSide( pThisPFace, SM_OT_SAME,
                                        sCollectedFaceLists1, sCollectedSideLists1 ) )
                {
                  bDidRadialSectoring = TRUE;
                  SER( sm_RadialSectoring( pRadialE, pEdge, SM_OT_SAME,
                                           pRegionOfSector, pCoincidentFace ));
                  bFaceIsOutsideOther = ( pRegionOfSector == NULL );
                }
            }

          if ( bDidRadialSectoring == FALSE )
            {
              // Check for coincidence.
              double     dAng, dAngLim = SM_DEG2RAD( 2.0 ); // as in sm_RadialSectoring...
              SmVector3d sBinormalToClassify = pRadialE->ComputeBinormal();
              SmVector3d sBinormalOfSector   = pEdge   ->ComputeBinormal();

              SER( sBinormalToClassify.AngleBetween( sBinormalOfSector, dAng ));
              if ( smos_Fabs( dAng ) < dAngLim )
                {
                  // Possible coincidence with pEdgeOfSector's face.
                  if ( sm_CheckCoincidentFaces( pThisPFace, pOtherPFace ))
                    {
                      pCoincidentFace = pThisPFace;
                    }
                }
            }

          // State: we have classified pOtherPFace against Brep:
          // - pCoincidentFace is set if appropriate,
          // - otherwise bFaceIsOutsideOther is set appropriately.

          if ( pCoincidentFace == NULL )
            {
              // Collect adjacent faces, they all classify the same.
              sm_CollectFaces(pOFace, sCollectedFaces, eOtherMarkType);  // Will Mark:[eOtherMarkType] faces
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe7 = FALSE;
              if (bDebugMe7) 
                {
                  smgfx_Erase();
                  smgfx_SetLook( 2,3, 1,0,0 ); pOFace->DrawDebug(); sm_GraphicsLoop();
                  smgfx_SetLook( 1,2, 0,0,1 );
                  for (ULONG ifac=0; ifac<sCollectedFaces.GetSize(); ifac++) 
                    {
                      sCollectedFaces[ifac]->DrawDebug(); sm_GraphicsLoop();
                      sm_GraphicsLoop();
                    }
                }
#endif // SM_DEBUG_CODE

              if (eOperation == SM_PBO_EXTRACT_SEPARATE) 
                {
                  SmPolyBrep *pPolyBrep = new (*m_vPI.m_pBrep->GetContext()) SmPolyBrep(m_vPI.m_pBrep->GetTolerance());
                  m_vPI.m_pOther->CopyFaces(sCollectedFaces,pPolyBrep,eOtherMarkType);
                  m_vOneBrepPer_OtherBrepConnectedFaceSet.Add(pPolyBrep);
                  continue;
                }

              switch (eOperation) 
                {
                  case SM_PBO_UNION           : if ( bFaceIsOutsideOther ) { sKeepOFaces.Append(sCollectedFaces); }
                                                else                       { sDelOFaces.Append(sCollectedFaces); }
                                                break;
                  case SM_PBO_DIFFERENCE      :
                  case SM_PBO_INTERSECTION    : if ( bFaceIsOutsideOther ) { sDelOFaces.Append(sCollectedFaces); }
                                                else                       { sKeepOFaces.Append(sCollectedFaces); }
                                                break;
                  case SM_PBO_PARTIAL_MERGE   :
                  case SM_PBO_MERGE           : sKeepOFaces.Append(sCollectedFaces);
                                                break;
                  case SM_PBO_EXTRACT_SEPARATE:
                  case SM_PBO_IMPRINT         : break;
                } // end switch on Keep/delete OtherFaces
            } // end pCoincidentFace == NULL branch
          else // pCoincidentFace != NULL
            {
              // Found a coincident face.

              if (pCoincidentFace->IsMarked(eBrepMarkType)) { continue; }

              if (eOperation == SM_PBO_EXTRACT_SEPARATE) 
                {
                  sCollectedFaces.ReSet(); 
                  sCollectedFaces.Add(pOFace);
                  SmPolyBrep *pPolyBrep = new (*m_vPI.m_pBrep->GetContext()) SmPolyBrep(m_vPI.m_pBrep->GetTolerance());
                  m_vPI.m_pOther->CopyFaces(sCollectedFaces,pPolyBrep,eOtherMarkType);
                  m_vOneBrepPer_OtherBrepConnectedFaceSet.Add(pPolyBrep);
                  continue;
                }

              SER(m_vPI.Relate(pCoincidentFace,pOFace));

#ifdef SM_DEBUG_CODE    
SmBoolean bDebugMe5 = FALSE;
              if (bDebugMe5)
                {
                  smgfx_Erase();
                  smgfx_SetLook( 1,2, 0,0,0 ); pCoincidentFace->DrawDebug(); sm_GraphicsLoop();
                  smgfx_SetLook( 1,2, 0,1,0 ); pOFace->DrawDebug(); sm_GraphicsLoop();
                  sm_GraphicsLoop();
               }
#endif // SM_DEBUG_CODE
 
              pOFace->Mark(eOtherMarkType);
              pCoincidentFace->Mark(eBrepMarkType);

              // Determine if two faces are on the same side
              SmVector3d sNorm = pCoincidentFace->GetNormal();
              SmVector3d sOtherNorm = pOFace->GetNormal();
              SmBoolean bOnSameSide = TRUE;
              if (sNorm.Dot(sOtherNorm) < 0.0) 
                { bOnSameSide =  FALSE; }

              switch (eOperation) 
                {
                  case SM_PBO_UNION           :
                  case SM_PBO_INTERSECTION    : if (bOnSameSide) { sDelOFaces.Add(pOFace); sDelFaces.Add(pCoincidentFace); }
                                                else             { sDelOFaces.Add(pOFace);  sKeepFaces.Add(pCoincidentFace); }
                                                break;
                  case SM_PBO_DIFFERENCE      : if (bOnSameSide) { sDelOFaces.Add(pOFace); sDelFaces.Add(pCoincidentFace); }
                                                else             { sDelOFaces.Add(pOFace); sKeepFaces.Add(pCoincidentFace); }
                                                break;
                  case SM_PBO_PARTIAL_MERGE   :
                  case SM_PBO_MERGE           : sDelOFaces.Add(pOFace); sKeepFaces.Add(pCoincidentFace);
                                                break;
                  case SM_PBO_EXTRACT_SEPARATE:
                  case SM_PBO_IMPRINT         : break;
              } // end switch keep/delete otherFaces
          } // end pCoincidentFace != NULL branch
      } // For jj each radial face around this edge
  } // end for ii each common edge

  // Second pass: classify and merge the rest of the faces (non-intersecting).
  // First classify m_pBrep faces against m_pOther.
  SmPolyFace          * sPBFData[64];
  SmTArray<SmPolyFace*> sBrepFaces(64,sPBFData);
  
  // begin scope for classify and merge non-intersecting shells,  
    {
      // locals
      double     dTol          = m_vPI.m_pOther->GetTolerance();
      double     dExpansionTol = dTol*1.0e2;
      SmExtent3d sBBox;
      ULONG      lSize[3];
      lSize[0] = 10;
      lSize[1] = 10;
      lSize[2] = 10;

      m_vPI.m_pOther->CalculateBoundingBox(sBBox);
      sBBox.ExpandAbsolute(dExpansionTol);

      SmGrid    * pGrid = new (*m_vPI.m_pOther->GetContext()) SmGrid(sBBox,lSize);
      SmRayTracer sRayTracer(*m_vPI.m_pOther->GetContext(),pGrid,dTol,dTol);
      SER(sRayTracer.AddPolyBrepToGrid(m_vPI.m_pOther,dExpansionTol));
      sRayTracer.m_bHitAnyThing = FALSE;
        
      m_vPI.m_pBrep->GetPolyFaces(sBrepFaces);
      ULONG kk, lNumFaces = sBrepFaces.GetSize();

      // for every Brep PolyFace
      for (kk=0; kk<lNumFaces; kk++)
        {
          SmPolyFace *pFace = sBrepFaces[kk];

#ifdef SM_DEBUG_CODE    
SmBoolean bDebugMe3 = FALSE;
          if (bDebugMe3) 
            {
              smgfx_Erase();
              if ( pFace->IsMarked(eBrepMarkType)) { smgfx_SetLook(2,4, 0,1,0); }
              else                                 { smgfx_SetLook(2,4, 1,0,0); }
                                         pFace->DrawDebug();     sm_GraphicsLoop();
              smgfx_SetLook(1,2, 0,1,1); m_vPI.m_pOther->Draw(TRUE); sm_GraphicsLoop();
              smgfx_SetLook(1,2, 0,1,0); m_vPI.m_pBrep->Draw(TRUE);  sm_GraphicsLoop();
              sm_GraphicsLoop();
            }
#endif // SM_DEBUG_CODE

          if (pFace->IsMarked(eBrepMarkType)) { continue; }
            
          if (eOperation == SM_PBO_EXTRACT_SEPARATE) 
            {
              sm_CollectFaces(pFace, sCollectedFaces, eBrepMarkType);
              SmPolyBrep *pPolyBrep = new (*m_vPI.m_pBrep->GetContext()) SmPolyBrep(m_vPI.m_pBrep->GetTolerance());
              m_vPI.m_pBrep->CopyFaces(sCollectedFaces,pPolyBrep,eBrepMarkType);
              m_vOneBrepPer_OrigBrepConnectedFaceSet.Add(pPolyBrep);
              continue;
            }
            
          // Do nothing here if we do a partial merge just keep all
          // of these kind of faces.
          if(   eOperation == SM_PBO_PARTIAL_MERGE
             || eOperation == SM_PBO_MERGE) 
            {
              sKeepFaces.Add(pFace);
              continue;
            }

          SmPolyEdge * pEdge = pFace->GetOuterPolyLoop()->GetFirstPolyEdge();
          SmPoint3d    sRayPoint, sRayVector(1.0,1.0,0.1);
          SER(pEdge->EvaluatePoint(0.45678,sRayPoint));
            
          // Do ray-firing here
          SmPolyFace * pHitFace = NULL;
          SER(sm_FireRay(sRayTracer,m_vPI.m_pOther,sRayPoint,sRayVector,pHitFace));
          SmBoolean bHitNullRegion = TRUE;
          if (pHitFace) 
            {
              SmVector3d sNorm = pHitFace->GetNormal();
              sRayVector.Unitize();
              if(sNorm.Dot(sRayVector) > 0.0)
                { bHitNullRegion = FALSE; }
            }
            
          sm_CollectFaces(pFace, sCollectedFaces, eBrepMarkType);

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe4 = FALSE;
              if (bDebugMe4) 
                {
                  smgfx_Erase();
                  smgfx_SetLook( 2,3, 1,0,0 ); pFace->DrawDebug(); sm_GraphicsLoop();
                  smgfx_SetLook( 1,2, 0,0,1 );
                  for (ULONG ifac=0; ifac<sCollectedFaces.GetSize(); ifac++) 
                    {
                      sCollectedFaces[ifac]->DrawDebug(); sm_GraphicsLoop();
                      sm_GraphicsLoop();
                    }
                }
#endif // SM_DEBUG_CODE

          switch (eOperation) 
            {
              case SM_PBO_UNION           :
              case SM_PBO_DIFFERENCE      : if (bHitNullRegion) { sKeepFaces.Append(sCollectedFaces); }
                                            else                { sDelFaces.Append(sCollectedFaces); }
                                            break;
              case SM_PBO_INTERSECTION    : if (bHitNullRegion) { sDelFaces.Append(sCollectedFaces); }
                                            else                { sKeepFaces.Append(sCollectedFaces); }
                                            break;
              case SM_PBO_PARTIAL_MERGE   :
              case SM_PBO_MERGE           : sKeepFaces.Append(sCollectedFaces);
                                            break;
              case SM_PBO_EXTRACT_SEPARATE:
              case SM_PBO_IMPRINT         : break;
          } // end switch keep/delete faces
      } // end for kk each face
  } // end scope for classify and merge non-intersecting shells,
    // m_pBrep faces vs. m_pOther.
    
// Now classify m_pOther faces against m_pBrep.
  {
      double     dTol = m_vPI.m_pBrep->GetTolerance();
      double     dExpansionTol = dTol*1.0e2;
      SmExtent3d sBBox;
      ULONG      lSize[3];
      lSize[0] = 10;
      lSize[1] = 10;
      lSize[2] = 10;

      m_vPI.m_pBrep->CalculateBoundingBox(sBBox);
      sBBox.ExpandAbsolute(dExpansionTol);

      SmGrid    * pGrid = new (*m_vPI.m_pBrep->GetContext()) SmGrid(sBBox,lSize);
      SmRayTracer sRayTracer(*m_vPI.m_pBrep->GetContext(),pGrid,dTol,dTol);
      SER(sRayTracer.AddPolyBrepToGrid(m_vPI.m_pBrep,dExpansionTol));
      sRayTracer.m_bHitAnyThing = FALSE;
        
      m_vPI.m_pOther->GetPolyFaces(sBrepFaces);
      ULONG mm, lNumFaces = sBrepFaces.GetSize();

      // for every OtherBrep PolyFace
      for (mm=0; mm<lNumFaces;  mm++)
        {
          SmPolyFace *pOFace = sBrepFaces[mm];

#ifdef SM_DEBUG_CODE    
SmBoolean bDebugMe2 = FALSE;
          if (bDebugMe2) 
            {
              smgfx_Erase();
              if ( pOFace->IsMarked(eOtherMarkType)) { smgfx_SetLook(2,4, 0,1,0); }
              else                                   { smgfx_SetLook(2,4, 1,0,0); }
                                         pOFace->DrawDebug();    sm_GraphicsLoop();
              smgfx_SetLook(1,2, 0,0,1); m_vPI.m_pBrep->Draw(TRUE);  sm_GraphicsLoop();
              smgfx_SetLook(1,2, 0,1,1); m_vPI.m_pOther->Draw(TRUE); sm_GraphicsLoop();
              sm_GraphicsLoop();
            }
#endif // SM_DEBUG_CODE

          if (pOFace->IsMarked(eOtherMarkType)) { continue; }
            
          if (eOperation == SM_PBO_EXTRACT_SEPARATE) 
            {
              sm_CollectFaces(pOFace, sCollectedFaces, eOtherMarkType);
              SmPolyBrep *pPolyBrep = new (*m_vPI.m_pBrep->GetContext()) SmPolyBrep(m_vPI.m_pBrep->GetTolerance());
              m_vPI.m_pOther->CopyFaces(sCollectedFaces,pPolyBrep,eOtherMarkType);
              m_vOneBrepPer_OtherBrepConnectedFaceSet.Add(pPolyBrep);
              continue;
            }

          // Do nothing here if we do a partial merge just keep all
          // of these kind of faces.
          if(   eOperation == SM_PBO_PARTIAL_MERGE
             || eOperation == SM_PBO_MERGE) 
            {
              sKeepOFaces.Add(pOFace);
              continue;
            }
          SmPolyEdge * pOEdge = pOFace->GetOuterPolyLoop()->GetFirstPolyEdge();
          SmPoint3d    sRayPoint, sRayVector(1.0,1.0,0.1);
          SER(pOEdge->EvaluatePoint(0.45678,sRayPoint));
            
          // Do ray-firing here
          SmPolyFace * pHitFace = NULL;
          SER(sm_FireRay(sRayTracer,m_vPI.m_pBrep,sRayPoint,sRayVector,pHitFace));
          SmBoolean bHitNullRegion = TRUE;
          if (pHitFace) 
            {
              SmVector3d sNorm = pHitFace->GetNormal();
              sRayVector.Unitize();
              if (sNorm.Dot(sRayVector) > 0.0)
                { bHitNullRegion = FALSE; }
            }
            
          sm_CollectFaces(pOFace, sCollectedFaces, eOtherMarkType);

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe9 = FALSE;
          if (bDebugMe9) 
            {
              smgfx_Erase();
              smgfx_SetLook( 2,3, 1,0,0 ); pOFace->DrawDebug(); sm_GraphicsLoop();
              smgfx_SetLook( 1,2, 0,0,1 );
              for (ULONG ifac=0; ifac<sCollectedFaces.GetSize(); ifac++) 
                {
                  sCollectedFaces[ifac]->DrawDebug(); sm_GraphicsLoop();
                  sm_GraphicsLoop();
                }
            }
#endif // SM_DEBUG_CODE

          switch (eOperation) 
            {
              case SM_PBO_UNION           : if (bHitNullRegion) { sKeepOFaces.Append(sCollectedFaces); }
                                            else                { sDelOFaces.Append(sCollectedFaces); }
                                            break;
              case SM_PBO_DIFFERENCE      :
              case SM_PBO_INTERSECTION    : if (bHitNullRegion) { sDelOFaces.Append(sCollectedFaces); }
                                            else                { sKeepOFaces.Append(sCollectedFaces); }
                                            break;
              case SM_PBO_PARTIAL_MERGE   :
              case SM_PBO_MERGE           : sKeepOFaces.Append(sCollectedFaces);
                                            break;
              case SM_PBO_EXTRACT_SEPARATE:
              case SM_PBO_IMPRINT         : break;
            } // end switch keep/delete PolyFaces
        } // end for mm each face
    } // end scope for classify and merge non-intersecting shells,
    
  // m_pBrep faces vs. m_pOther.
    
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe10 = FALSE;
SmBoolean bDebugMe12 = FALSE;
  SmBoolean bBrepOK = TRUE, bOtherOK = TRUE;
#endif // SM_DEBUG_CODE

#ifdef SM_VALIDATE_TOPOLOGY
  m_vPI.m_pBrep->ValidatePointers();
  m_vPI.m_pOther->ValidatePointers();
#endif // SM_VALIDATE_TOPOLOGY

  // Now all faces are classified.
  //
  // If we are simply classifying faces, use attributes to label each face.
  //    (Currently we only label faces for deletion.)
  // Exit gracefully with input objects intact but with modified topology.
  if( m_bImprintAndClassify )
    {
      pBrepContext  = SM_CONST_CAST(SmContext*,m_vPI.m_pBrep->GetContext());
      pOtherContext = SM_CONST_CAST(SmContext*,m_vPI.m_pOther->GetContext());
  
      ULONG       fi, lNumDelFaces = sDelFaces.GetSize();

      // Label the delete faces on object A
      if ( lNumDelFaces > 0 )
        {
          SmLongAttribute *pDeleteAttribute = new (*pBrepContext)
              SmLongAttribute( SM_AI_BOOLEAN_DELETE, 1, SM_AB_REFERENCE );

          for( fi=0; fi<lNumDelFaces; fi++ ) 
            {
              if( sDelFaces[fi]->FindAttribute( SM_AI_BOOLEAN_DELETE ) == NULL )
                { sDelFaces[fi]->AddAttribute( pDeleteAttribute ); }
            }
        }

      // Label the delete faces on object B
      lNumDelFaces = sDelOFaces.GetSize();
      if ( lNumDelFaces > 0 )
        {
          SmLongAttribute *pDeleteOAttribute = new (*pOtherContext)
              SmLongAttribute( SM_AI_BOOLEAN_DELETE, 1, SM_AB_REFERENCE );

          for( fi=0; fi<lNumDelFaces; fi++ )
            {
              if( sDelOFaces[fi]->FindAttribute( SM_AI_BOOLEAN_DELETE ) == NULL )
                { sDelOFaces[fi]->AddAttribute( pDeleteOAttribute ); }
            }
        }

      // restore state
      pBrepContext->SetDoingBoolean(FALSE);
      pOtherContext->SetDoingBoolean(FALSE);

      // set output value although input breps remain intact
      rpResult = m_vPI.m_pBrep;

      // all done
      return SM_SUCCESS ;

    } // end if m_bImprintAndClassify

  // Create or delete faces as indicated.

  // For each face in sKeepOFaces array, create new face in the first brep
  ULONG nn, lNumKeepOFaces = sKeepOFaces.GetSize();
  for (nn=0; nn<lNumKeepOFaces; nn++)
    {
      SmPolyFace * pOFace = sKeepOFaces[nn];

      if (eOperation == SM_PBO_EXTRACT_SEPARATE) { break; }

      // reverse the orientation if DIFFERENCE
      SmBoolean bReverseOrientation = FALSE;
      if (eOperation == SM_PBO_DIFFERENCE) 
        { bReverseOrientation = TRUE; }
    
#ifdef SM_DEBUG_CODE
      if ( bDebugMe10 ) 
        {
          if ( nn==0 ) 
            { smgfx_Erase();
              smgfx_SetLook( 1,2, 0,0,0 ); m_vPI.m_pBrep->Draw(TRUE); sm_GraphicsLoop();
              smgfx_SetLook( 1,2, 0,0,0 ); m_vPI.m_pOther->Draw(TRUE); sm_GraphicsLoop();
            }
          smgfx_SetLook( 3,5, 1,0,0 ); pOFace->Draw(TRUE); sm_GraphicsLoop();
          sm_GraphicsLoop();
        }
      if ( bDebugMe12 )
        { bBrepOK  = SM_ASSERT_VALID(m_vPI.m_pBrep) ;
          bOtherOK = SM_ASSERT_VALID(m_vPI.m_pOther) ;

          if ( !( bBrepOK && bOtherOK ) )
            { SM_ASSERT( FALSE ); } // breakpoint: problem before MoveFace().
        }
#endif // SM_DEBUG_CODE

#ifdef SM_VALIDATE_TOPOLOGY
      m_vPI.m_pOther->ValidatePointers();
#endif // SM_VALIDATE_TOPOLOGY

      // The cookie cutter does not keep the faces from the other brep
      if (!m_bCookieCutter)
        { SER( MoveFace( pOFace, bReverseOrientation )); }

#ifdef SM_DEBUG_CODE
      if ( bDebugMe12 ) 
        {
          SmBoolean bBrepOK_2  = SM_ASSERT_VALID(m_vPI.m_pBrep) ;
          SmBoolean bOtherOK_2 = SM_ASSERT_VALID(m_vPI.m_pOther) ;

          if ( !( bBrepOK && bOtherOK && bBrepOK_2 && bOtherOK_2 ) )
            { SM_ASSERT( FALSE ); } // breakpoint: any problem.
          if ( ( bBrepOK && !bBrepOK_2 ) || ( bOtherOK && !bOtherOK_2 ) )
            { SM_ASSERT( FALSE ); } // breakpoint: was good, MoveFace() broke it.
        }
#endif // SM_DEBUG_CODE

#ifdef SM_VALIDATE_TOPOLOGY
      m_vPI.m_pOther->ValidatePointers();
#endif
    } // end for nn each KeepOFace

  // Delete all faces in sDelFaces array
  SER(m_vPI.DeleteFaces(sDelFaces));

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe6 = FALSE;
  if (bDebugMe6) 
    {
      m_vPI.m_pBrep->Dump();
      smgfx_Erase();
      smgfx_SetLook( 1,2, 0,0,0 ); m_vPI.m_pBrep->Draw(TRUE); sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

#ifdef SM_DEBUG_CODE
  if ( bDebugMe12 ) 
    {
      bBrepOK  = SM_ASSERT_VALID(m_vPI.m_pBrep) ;
      bOtherOK = SM_ASSERT_VALID(m_vPI.m_pOther) ;
      if ( !( bBrepOK && bOtherOK ) )
        { SM_ASSERT( FALSE ); } // breakpoint: problem before MoveFace().
    }
#endif // SM_DEBUG_CODE

#ifdef SM_VALIDATE_TOPOLOGY
  m_vPI.m_pBrep->ValidatePointers();
  m_vPI.m_pOther->ValidatePointers();
#endif // SM_VALIDATE_TOPOLOGY

  // Now see whether we have to create new shells.
  sMarkLock.NewMark();
  SM_PTR_ARRAY(sRegions,SmPolyRegion,16);
  SM_PTR_ARRAY(sShells,SmPolyShell,64);
  m_vPI.m_pBrep->GetPolyRegions(sRegions);
  ULONG ireg, lNumRegions = sRegions.GetSize();

  //
  for (ireg=0; ireg<lNumRegions; ireg++) 
    {
      if (eOperation == SM_PBO_EXTRACT_SEPARATE) { break; }

      SmPolyRegion * pReg = sRegions[ireg];
      pReg->GetPolyShells(sShells);
      ULONG ish, lNumShells = sShells.GetSize();

      //
      for (ish=0; ish<lNumShells; ish++)
        {
          SmPolyShell *pSh = sShells[ish];
          pSh->GetPolyFaces(sKeepFaces);
          SmBoolean bFirst = TRUE;
          ULONG ifac, lNumKeepFaces = sKeepFaces.GetSize();
          for (ifac=0; ifac<lNumKeepFaces; ifac++)
            {
              SmPolyFace *pF = sKeepFaces[ifac];
              if (pF->IsMarked(eBrepMarkType)) 
                { continue; }

              if (bFirst) 
                {  // Belongs to pSh shell
                  bFirst = FALSE;
                  sm_CollectFaces(pF, sCollectedFaces, eBrepMarkType);
                }
              else 
                {  // Need to make a new shell for these faces
                  SmPolyShell *pNewShell = NULL;
                  if (ish == 0) 
                    { // Split Outer shell make new region
                      SmPolyRegion *pNewReg = new (m_vPI.m_pBrep) SmPolyRegion(m_vPI.m_pBrep);
                      // SmPolyRegion *pNewReg = SmPolyRegion::NewPolyRegion(*m_vPI.m_pBrep);
                      pNewShell = new (m_vPI.m_pBrep) SmPolyShell(pNewReg);
                      // pNewShell = SmPolyShell::NewPolyShell (*m_vPI.m_pBrep, pNewReg);
                    }
                  else 
                    { // Split inner shell
                      pNewShell = new (m_vPI.m_pBrep) SmPolyShell(pReg);
                      // pNewShell = SmPolyShell::NewPolyShell (*m_vPI.m_pBrep, pReg);
                    }
                  NER(pNewShell);
                  sm_CollectFaces(pF, sCollectedFaces, eBrepMarkType);

                  ULONG icf, lNumCollFaces = sCollectedFaces.GetSize();
                  for (icf=0; icf<lNumCollFaces; icf++) 
                    {
                      pSh->Remove(sCollectedFaces[icf]);
                      pNewShell->PostInsert(sCollectedFaces[icf]);
                    }
                }
            } // end for all keep faces
        } // end for all shells in this region
    } // end for all regions

  //
  m_vPI.m_pOther->Notify(SM_NO_POST_EDIT, m_vPI.m_pOther, NULL, NULL);

  if (eOperation != SM_PBO_PARTIAL_MERGE) 
    {
#ifdef SM_VALIDATE_TOPOLOGY
      m_vPI.m_pOther->ValidatePointers();
#endif
      SM_ASSERT(m_vPI.m_pOther != NULL) ; delete m_vPI.m_pOther ; m_vPI.m_pOther = NULL ;
    }

  if (m_bRemoveCoPlanarEdges) 
    { SER(RemoveCoPlanarEdges()); }

  SmTArray<SmCPolyFace*> sCFaces;
  m_vPI.m_pBrep->GetCPolyFaces(sCFaces);
  ULONG icf;
  for (icf=0; icf<sCFaces.GetSize(); icf++)
    {
      SM_ASSERT(sCFaces[icf] != NULL) ; delete sCFaces[icf] ; sCFaces[icf] = NULL ;
    }

  // Cleanup those shells that meant to be deleted
  m_vPI.m_pBrep->RemoveDegenerateFaces();

  rpResult = m_vPI.m_pBrep;

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe7 = FALSE;
  if (bDebugMe7) 
    {
      m_vPI.m_pBrep->Dump();
      smgfx_Erase();
      smgfx_SetLook( 1,2, 0,0,0 ); m_vPI.m_pBrep->Draw(TRUE); sm_GraphicsLoop();
      sm_GraphicsLoop();

#ifdef SM_VALIDATE_TOPOLOGY
      m_vPI.m_pBrep->ValidatePointers();
#endif // SM_VALIDATE_TOPOLOGY
    }
#endif // SM_DEBUG_CODE

  //
  m_vPI.m_pBrep->Notify(SM_NO_POST_EDIT, m_vPI.m_pBrep, NULL, NULL);

  // all done
  return SM_SUCCESS;

} // end SmPolyMerge::ManifoldBoolean

/*******************************************************************//**
PURPOSE: Move a PolyFace from the pOtherBrep into pBrep.

NOTES: pOPolyFace is removed from pOtherBrep

  If MemoryBlockManagement is used, the value of pOFace will be changed
  to be a pointer within m_pBrep's memory instead of m_pOther's.
***********************************************************************/
SmStatus SmPolyMerge::MoveFace
 (SmPolyFace *& pOPolyFace,           // in : face to move to this m_pBrep
  SmBoolean     bReverseOrientation)  // in : TRUE = 
                                      //      FALSE=
{
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
  if (bDebugMe) 
    {
      smgfx_Erase();
      smgfx_SetLook(1,2, 0,0,1); m_vPI.m_pBrep->Draw(TRUE); sm_GraphicsLoop();
      smgfx_SetLook(2,3, 1,0,0); pOPolyFace->DrawDebug();   sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
#endif

  // lcoals
  ULONG ii, jj, kk ;
  SmPolyBrep   * pBrep   = m_vPI.m_pBrep;
  SmPolyShell  * pShell  = NULL;
  //SmPolyBrep   * pOBrep  = m_vPI.m_pOther;
  //SmPolyRegion * pRegion = NULL;

  SM_PTR_ARRAY(sOPolyLoopEdges,      SmPolyEdge, 16);  // SmTArray<SmPolyEdge *>
  SM_PTR_ARRAY(sOPolyFaceEdgeRadials,SmPolyEdge, 16);  // SmTArray<SmPolyEdge *>

  // get pOPolyFace->PolyEdges
  pOPolyFace->GetPolyEdges(sOPolyLoopEdges);

#ifdef SM_VALIDATE_TOPOLOGY
  pBrep->ValidatePointers();
  pOBrep->ValidatePointers();
#endif

  // Determine whether the face is already in the Brep:
  // see whether the face is attached to any shell of this brep.
  ULONG lNumOEdges = sOPolyLoopEdges.GetSize();
  for (ii=0; ii<lNumOEdges; ii++) 
    {
      SmPolyEdge *pOEdge    = sOPolyLoopEdges[ii];
      SmPolyEdge *pORadialE = pOEdge->GetRadial();
      SmPolyEdge *pEdge     = SM_REINTERPRET_CAST(SmPolyEdge*,m_vPI.GetBrepMate(pOEdge));
      if (!pEdge && pORadialE) 
        {
          // Relationship may exist on radial edge
          pEdge = SM_REINTERPRET_CAST(SmPolyEdge*,m_vPI.GetBrepMate(pORadialE));
        }
      if (pEdge) 
        {
          // Have related edges
          pShell = pEdge->GetPolyFace()->GetPolyShell();
          break;
        }
      else if (!pShell && pORadialE) 
        {
          SmPolyShell *pRadialShell = pORadialE->GetPolyFace()->GetPolyShell();
          if (pRadialShell->GetPolyRegion()->GetPolyBrep() == pBrep) {
              pShell = pRadialShell;
            }
        }
    } // end iter ii, every OPolyFace->PolyEdge

  SmPolyShell * pOPolyShell = pOPolyFace->GetPolyShell();
  if (pOPolyShell == pShell) 
    { // Face has already been moved to this brep
      return SM_SUCCESS;
    }

  // Ok, we have to create a new face in pBrep.

//cbi This should work, but it doesn't pass prog_test:
//  SER( CopyFace( pOPolyFace, bReverseOrientation ));
//  pOBrep->DeletePolyFace( pOPolyFace );
//  return SM_SUCCESS;

  // Pull the old one out of the other Brep.
  pOPolyShell->Remove(pOPolyFace);

  // Another way that pOPolyFace could still be in pOtherBrep:
  // if it's a member of a CPolyFace, and not the CPolyFace itself.
  SmCPolyFace *pCPolyFace = pOPolyFace->GetCPolyFace();
  if ( pCPolyFace != NULL && pCPolyFace != pOPolyFace )
    {
      pCPolyFace->RemovePolyFace( pOPolyFace );
      pOPolyFace->SetCPolyFace( NULL );
    }

  // Install the other Face in pBrep.
  if (pShell) 
    {
      pShell->PostInsert(pOPolyFace);
    }
  else 
    {
      // Make new shell by creating a new face and then remove it.
      // The SmPolyFace constructor, given a Brep, will create a new region
      // and shell for the face in the Brep.  Add the face to be copied
      // to that shell, then delete the newly-created face.

      SmVector3d sPlaneNormal = pOPolyFace->GetNormal() ;
      double     sNormalSize  = sPlaneNormal.Length() ;

      SmPolyFace *pNewFace = new (pBrep) SmPolyFace(SM_EFF_ZERO, pBrep, NULL, NULL, sNormalSize < 1.1 ? &sPlaneNormal : NULL ); 
      // SmPolyFace *pNewFace = SmPolyFace::NewPolyFace (*pBrep, SM_EFF_ZERO, pBrep);
      pShell = pNewFace->GetPolyShell();
      pShell->PostInsert(pOPolyFace);
      SM_ASSERT(pNewFace != NULL) ; delete pNewFace ; pNewFace = NULL ;
   }

#ifdef SM_VALIDATE_TOPOLOGY
  pBrep->ValidatePointers();
  pOBrep->ValidatePointers();
#endif

  // when asked - reverse the pOPolyFace
  if (bReverseOrientation) 
    {
      SER(pOPolyFace->ReverseOrientation());
    }

  // Process each PolyEdge, going PolyLoop by PolyLoop.

  // Some locals:
  SM_PTR_ARRAY(sOPolyFaceLoops,SmPolyLoop,       64);
  SM_PTR_ARRAY(sPolyEdgesBetween,SmPolyEdge,     16);
  SM_PTR_ARRAY(sDeletedPolyVertices,SmPolyVertex,32) ;
  SM_PTR_ARRAY(sSurvivingPolyVertices,SmPolyVertex,32) ;

  // get PolyFace->PolyLoops
  pOPolyFace->GetPolyLoops(sOPolyFaceLoops);
  ULONG lNumOLoops = sOPolyFaceLoops.GetSize();

  // for every OPolyFace->PolyLoop
  for (ii=0; ii<lNumOLoops; ii++)
    {
      SmPolyLoop * pOPolyFaceLoop = sOPolyFaceLoops[ii];

      pOPolyFaceLoop->GetPolyEdges(sOPolyLoopEdges);
      sOPolyFaceEdgeRadials.ReSet();

      // For each edge in this loop,
      //  Remove it from its start vertex,
      //  and from its radial partners,
      //  and attach it to a vertex in pBrep -- either
      //  an existing, mated vertex or a newly created one.
      // Also, collect a list of each edge's radial partner, for later use.

      ULONG lNumLoopEdges = sOPolyLoopEdges.GetSize();
      for (jj=0; jj<lNumLoopEdges; jj++)
        {
          SmPolyEdge * pOPolyLoopEdge = sOPolyLoopEdges[jj];

          // Collect a list of original radials for later use.
          SmPolyEdge * pORadialE = pOPolyLoopEdge->GetRadial();
          sOPolyFaceEdgeRadials.Add(pORadialE);

          // Detach this OEdge from its start vertex.
          //  Note, this also removes it from the m_pPrevEdgeAtV
          //  and m_NextEdgeAtV doubly linked list.
          SmPolyVertex * pOVert = pOPolyLoopEdge->GetStartPolyVertex();
          pOVert->RemovePolyEdge(pOPolyLoopEdge);

          // Detach the edge from its radial partners.
          pOPolyLoopEdge->MakeLamina();
            
          // See whether pOVert has a mate in pBrep.
          SmPolyVertex * pVert = SM_REINTERPRET_CAST(SmPolyVertex*,m_vPI.GetBrepMate(pOVert));
            
          // If not, make a new one, and relate it.
          if (!pVert) 
            {
              pVert = new (pBrep) SmPolyVertex(pOVert->GetPoint(), pOVert->GetTolerance()) ; NER(pVert) ;
              pBrep->m_pPolyVertexListHead->PostInsert(pVert);

              // propogate the IndexValue from OPolyBrep->PolyFace to PolyBrep->PolyFace - odds are it can't have a meaning
              if(pOVert->IsIndexValueInit()) { pVert->SetIndexValue(pOVert->GetIndexValue()); }
              m_vPI.Relate(pVert,pOVert);
            }
            
          pVert->AddPolyEdge(pOPolyLoopEdge);

        } // end iter jj, each OEdge in this OLoop.
        
#ifdef SM_VALIDATE_TOPOLOGY
      pBrep->ValidatePointers();
      pOBrep->ValidatePointers();
#endif
        
      // For each edge from Other face in this loop:
      //   If it already has a mate in pBrep,
      //     Move the relationship to its Radial partner,
      //     so that the relationship will still be between pBrep and pOBrep.
      //   If the Radial now has a mate,
      //     Glue the mates together (make them radial neighbors).
      //   Else (edge not mated):
      //     Relate pOEdge and its radial partner.
      //     If the Other Radial is in pBrep,
      //     and its Shell is not the same as the shell that now contains the new face,
      //     then merge the shells.

      // for every OPolyLoop->Edge
      for (jj=0; jj<lNumLoopEdges; jj++)
        {
          SmPolyEdge * pOEdge    = sOPolyLoopEdges  [jj];
          SmPolyEdge * pORadialE = sOPolyFaceEdgeRadials[jj];

          // See if it (or its radial) is related to any edge
          SmPolyEdge * pEdge = SM_REINTERPRET_CAST(SmPolyEdge*,m_vPI.GetBrepMate(pOEdge));
          if (pEdge) 
            {
              // pEdge is pOEdge's mate in pBrep.
              // Move its relationship from pOEdge to pOEdge's radial partner.
              if (pORadialE) 
                {
                  m_vPI.RemoveRelationship(pEdge,pOEdge);
                  m_vPI.Relate(pEdge,pORadialE);
                }
            }
          else if (pORadialE) 
            {
              // pOEdge has no mate in pBrep, see if its radial partner does.
              pEdge = SM_REINTERPRET_CAST(SmPolyEdge*,m_vPI.GetBrepMate(pORadialE));
            }

          // when 
          if (pEdge) 
            {
              // Have related edges.
              // At this point, pEdge is related to pORadialE.

              // Make sure they're coincident.
              SmOrientType eOrient;
              double dTolerance = pOEdge->GetTolerance() + pEdge->GetTolerance();
              if (!pEdge->IsCoincidentWith(dTolerance,pOEdge,eOrient)) 
                {
                  SE(SM_ERR);
                }

#ifdef SM_VALIDATE_TOPOLOGY
              pBrep->ValidatePointers();
              pOBrep->ValidatePointers();
#endif // SM_VALIDATE_TOPOLOGY

              // Glue (make radial partners) pEdge and pOEdge without deleting any PolyEdge,
              //   glue endPVerts deleting pOtherEdge->PolyVerts          
              SER(pBrep->GlueEdges(pEdge,                     // in : Target PolyEdge1
                                   pOEdge,                    // in : Target PolyEdge2
                                   eOrient,                   // in : oneof SM_OT_SAME, SM_OT_OPPOSITE
                                   sPolyEdgesBetween,         // out: PolyEdges between glued vertices (made zero length by gluing) 
                                   &sDeletedPolyVertices,     // out: PolyVerts deleted when PolyVerts were glued (stale pointers), NULL to ignore, default:[NULL]
                                   &sSurvivingPolyVertices)); // out: associated surviving PolyVerts when PolyVerts were glued, NULL to ignore, default:[NULL]

#ifdef SM_VALIDATE_TOPOLOGY
              pBrep->ValidatePointers();
              pOBrep->ValidatePointers();
#endif // SM_VALIDATE_TOPOLOGY

              // If any deleted vertices had mates, update the relationships.
              ULONG lNumDelVerts = sDeletedPolyVertices.GetSize();
              for (kk=0; kk<lNumDelVerts; kk++) 
                {
                  SmPolyVertex * pDelV   = sDeletedPolyVertices[kk];
                  SmPolyVertex * pSrvV   = sSurvivingPolyVertices[kk] ;
                  SmPolyVertex * pOtherV = SM_REINTERPRET_CAST(SmPolyVertex*,m_vPI.GetOtherMate(pDelV)); // from m_pBToO
                  if (pOtherV) 
                    { m_vPI.RemoveRelationship(pDelV,pOtherV); 
                      m_vPI.Relate(pSrvV,pOtherV); // GWC: change to check
                    }
                }
            } // end if pEdge branch

          // No mated edge.
          // Relate pOEdge and its radial partner (which could still be in pOBrep),
          // if it had one.
          // Then, if the Other Radial is in pBrep,
          // and it's in a different shell than the shell that now contains the new face,
          // then merge the shells.

          else if (pORadialE)
            {
              m_vPI.Relate(pOEdge,pORadialE);

              SmPolyShell *pRadialShell = pORadialE->GetPolyFace()->GetPolyShell();
              if (    pRadialShell->GetPolyRegion()->GetPolyBrep() == pBrep
                   && pRadialShell != pShell)
                {
                  // Merge shells: preserve the larger one.
                  if (pRadialShell->GetSize() <= pShell->GetSize()) 
                    {
                      pBrep->MergeShells(pShell,pRadialShell);
                    }
                  else 
                    {
                      pBrep->MergeShells(pRadialShell,pShell);
                      pShell = pRadialShell;
                    }
                }
            } // end if pORadialE branch
        } // end iter jj, each edge in this loop.
    } // end iter ii, every PolyFace->PolyLoop

  // This will make sure all tolerances are updated.
  SmVector3d sNorm = pOPolyFace->GetNormal(TRUE,TRUE);

//cbi: should check this:
constexpr int sbDeleteDups = TRUE;

  // We have copied some entity pointers directly from pOBrep into pBrep.
  // If pOBrep is using the memory block manager, then those pointers
  // will become stale when pOBrep is deleted.
  // They must be replaced with memory allocated from pBrep.
  // The pointers in question are the face itself, all of the edges,
  // and all of the Loops that the Face points to;
  // everything else should be in pBrep already.
  //      if ( pOBrep->GetUseBlockManager() )
    {
      pOPolyFace->GetPolyEdges( sOPolyLoopEdges );
      lNumOEdges = sOPolyLoopEdges.GetSize();

      for (ii=0; ii<lNumOEdges; ii++)
        {
          SmPolyEdge *pOldEdge = sOPolyLoopEdges[ii];
          SmPolyEdge *pNewEdge = new (pBrep) SmPolyEdge( pOldEdge->GetTolerance() ); 
          // SmPolyEdge *pNewEdge = SmPolyEdge::NewPolyEdge (*pBrep, pOldEdge->GetTolerance() );

          pNewEdge->ReplacePolyEdge( pOldEdge );

          // If mated, replace that.
          SmTopology *pMate = SM_REINTERPRET_CAST(SmTopology*, m_vPI.GetBrepMate( pOldEdge ));
          if ( pMate != NULL )
            {
              m_vPI.RemoveRelationship( pMate, pOldEdge );
              m_vPI.Relate            ( pMate, pNewEdge );
            }
          pMate = SM_REINTERPRET_CAST(SmTopology*, m_vPI.GetOtherMate( pOldEdge ));
          if ( pMate != NULL )
            {
              m_vPI.RemoveRelationship( pOldEdge, pMate );
              m_vPI.Relate            ( pNewEdge, pMate );
            }

//cbi: this unhooks the topology, which Replace just did.
//cbi  Besides, MemBlockMgr will take care of it.
if ( sbDeleteDups ) 
{
          SM_ASSERT(pOldEdge != NULL) ; delete pOldEdge ; pOldEdge = NULL ;
} // sbDeleteDups

        } // end for each edge.

      // Now do the face's Loops.
      // Note, we can't loop on the topology because that will change in place.
      // So get the list in an array.
      SmTArray< SmPolyLoop* > sLoops;
      pOPolyFace->GetPolyLoops( sLoops );
      for ( ii = 0; ii < sLoops.GetSize(); ii++ )
        {
          SmPolyLoop *pOldLoop = sLoops[ii];
          SmPolyLoop *pNewLoop = new (pBrep) SmPolyLoop();
          // SmPolyLoop *pNewLoop = SmPolyLoop::NewPolyLoop (*pBrep) ;

          if ( pOldLoop == NULL || pNewLoop == NULL )
            { SM_ASSERT(FALSE); break; }

          pNewLoop->ReplacePolyLoop( pOldLoop );

if ( sbDeleteDups ) 
{
        SM_ASSERT(pOldLoop != NULL) ; delete pOldLoop ; pOldLoop = NULL ;
} // sbDeleteDups

        } // end for each loop

      // Finally do the face itself.
      SmPolyFace *pNewFace = new (pBrep) SmPolyFace( pOPolyFace->GetTolerance());
      // SmPolyFace *pNewFace = SmPolyFace::NewPolyFace (*pBrep, pOPolyFace->GetTolerance() );

      pNewFace->ReplacePolyFace( pOPolyFace, pBrep );

if ( sbDeleteDups ) 
{
    SM_ASSERT(pOPolyFace != NULL) ; delete pOPolyFace ; pOPolyFace = NULL ;
} // sbDeleteDups

      // the deep attribute copy has already been done by pNewFace->ReplacePolyFace( .. );
      // pOPolyFace->Notify(SM_NO_COPY, pNewFace, SM_NO_GET_POLYBREP(pNewFace), SM_NO_GET_POLYBREP(pOPolyFace));

      // For our caller:
      pOPolyFace = pNewFace;
    }

  // all done
  return SM_SUCCESS;

} // end SmPolyMerge::MoveFace

/*******************************************************************//**
PURPOSE: Copy a face from the Other Brep into the Brep.

NOTES: 
***********************************************************************/
SmStatus SmPolyMerge::CopyFace(SmPolyFace * & pOFace,
                               SmBoolean bReverseOrientation)
{
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
    if (bDebugMe) {
        smgfx_Erase();
        smgfx_SetLook(1,2, 0,0,1); m_vPI.m_pBrep->Draw(TRUE); sm_GraphicsLoop();
        smgfx_SetLook(2,3, 1,0,0); pOFace->DrawDebug();   sm_GraphicsLoop();
        sm_GraphicsLoop();
    }
#endif
    SmPolyBrep   * pBrep   = m_vPI.m_pBrep;
    //SmPolyBrep   * pOBrep  = m_vPI.m_pOther;
    //SmPolyRegion * pRegion = NULL;
    SmPolyShell  * pShell  = NULL;

#ifdef SM_VALIDATE_TOPOLOGY
    pBrep->ValidatePointers();
    pOBrep->ValidatePointers();
#endif

    // Determine whether the face is already in the Brep:
    // see whether the face is attached to any shell of this brep.
    SM_PTR_ARRAY(sORadials,SmPolyEdge,64);
    SM_PTR_ARRAY(sOEdges,SmPolyEdge,64);
    pOFace->GetPolyEdges(sOEdges);
    ULONG ii, lNumOEdges = sOEdges.GetSize();
    for (ii=0; ii<lNumOEdges; ii++)
    {
        SmPolyEdge *pOEdge = sOEdges[ii];
        SmPolyEdge *pORadialE = pOEdge->GetRadial();
        SmPolyEdge *pEdge = SM_REINTERPRET_CAST(SmPolyEdge*,m_vPI.GetBrepMate(pOEdge));
        if (!pEdge && pORadialE) {
            // Relationship may exist on radial edge
            pEdge = SM_REINTERPRET_CAST(SmPolyEdge*,m_vPI.GetBrepMate(pORadialE));
        }
        if (pEdge) {
            // Have related edges
            pShell = pEdge->GetPolyFace()->GetPolyShell();
            break;
        }
        else if (!pShell && pORadialE) {
            SmPolyShell *pRadialShell = pORadialE->GetPolyFace()->GetPolyShell();
            if (pRadialShell->GetPolyRegion()->GetPolyBrep() == pBrep) {
                pShell = pRadialShell;
            }
        }
    }

    SmPolyShell * pOShell = pOFace->GetPolyShell();
    if (pOShell == pShell) { // Face has already been moved to this brep
        return SM_SUCCESS;
    }


  // Ok, we have to create a new face in pBrep.

  // Call CreatePoly() on each PolyLoop.
  SmPolyFace *pNewPFace = NULL;  // Gets created on first call to CreatePolyLoop().

  SmTArray<SmPolyLoop*> sOLoops;
  SmTArray<SmPoint3d>   sVtxPts;

  pOFace->GetPolyLoops( sOLoops );
  ULONG lNumOLoops = sOLoops.GetSize();

  for (ii=0; ii<lNumOLoops; ii++)
  {
      SmPolyLoop * pOLoop = sOLoops[ii];

      sVtxPts.ReSet();

      pOLoop->GetPolyEdges( sOEdges );
      ULONG jj, lNumEdges = sOEdges.GetSize();

      for ( jj=0; jj<lNumEdges; jj++ )
      {
          sVtxPts.Add( sOEdges[jj]->GetStartPoint() );
      }

      pBrep->CreatePoly( NULL, sVtxPts, pNewPFace );
  }

#ifdef SM_VALIDATE_TOPOLOGY
    pBrep->ValidatePointers();
    pOBrep->ValidatePointers();
#endif

    if (bReverseOrientation) {
        SER(pOFace->ReverseOrientation());
    }

    pOFace->Notify(SM_NO_COPY, pNewPFace, SM_NO_GET_POLYBREP(pNewPFace), SM_NO_GET_POLYBREP(pOFace)) ;

    return SM_SUCCESS;

} // end SmPolyMerge::CopyFace

/*******************************************************************//**
PURPOSE: Setup a trimming plane to faciliate SmPolyBrep::TrimWithPlane.

NOTES: 
***********************************************************************/
void SmPolyMerge::SetTrimmingPlane
 (const SmPoint3d  & crPlanePoint,
  const SmVector3d & crPlaneNormal)
{
  m_vPI.m_vTrimPlanePoint = crPlanePoint;
  m_vPI.m_vTrimPlaneNormal = crPlaneNormal;
  m_vPI.m_bTrimWithPlane= TRUE;
  return;

} // end SetTrimmingPlane
