// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************
* FILE NAME --- SmMerge.cpp
* PURPOSE: Implementation of brep merging methods.
**********************************************************************/

#include "StdAfx.h"

#include <SmMerge.h>

#include <SmBrepData.h>
#include <SmBrepCache.h>
#include <SmCacheMgrBrep.h>
#include <SmRelation.h>
#include <SmAttribute.h>
#include <SmPrimitiveCreation.h>
#include <SmTrimmingTools.h>
#include <SmTopologyTraverser.h>

#include <SmTree.h>
// Remove Composites
//  #include <SmCFace.h>
#include <SmAssertArray.h>


/*******************************************************************//**
PURPOSE: Set default values for all boolean flags

NOTES:   Should be the same as SmMerge constructors
***********************************************************************/
SmMergeOptions::SmMergeOptions
  (SmBoolean bCookieCutter,              // in : TRUE = Skip adding faces (only remove faces) to BrepA, FALSE = don't skip
   SmBoolean bImprinting,                // in : TRUE = Imprint (A intersect B) on BrepA and quit (sames SM_BO_IMPRINT), FALSE = don't quit
   SmBoolean bBooleanPostProcess,         // in : TRUE = remove topological edges and vertices after Boolean, FALSE = don't remove
   SmBoolean bImprintAndClassifyFaces,   // in : TRUE = Imprint (A xsect B) on BrepA, add SM_AI_BOOLEAN_DELETED attrib to del faces, quit. FALSE = don't
   SmBoolean bKeepOtherBrep)             // in : TRUE = Do not delete BrepB, Do not delete BrebA with SM_BO_EXTRACT_SEPERATE, FALSE = delete other Brep
 : m_bCookieCutter           (bCookieCutter),
   m_bImprinting             (bImprinting),
   m_bBooleanPostProcess     (bBooleanPostProcess),
   m_bImprintAndClassifyFaces(bImprintAndClassifyFaces),
   m_bKeepOtherBrep          (bKeepOtherBrep)
{ }

/*******************************************************************//**
PURPOSE: Copy constructor for SmMergeOptions

NOTES:
***********************************************************************/
SmMergeOptions::SmMergeOptions
  (const SmMergeOptions & crSource)                 // in : options to copy
 : m_bCookieCutter(crSource.m_bCookieCutter),
   m_bImprinting(crSource.m_bImprinting),
   m_bBooleanPostProcess(crSource.m_bBooleanPostProcess),
   m_bImprintAndClassifyFaces(crSource.m_bImprintAndClassifyFaces),
   m_bKeepOtherBrep(crSource.m_bKeepOtherBrep)
{ }


// Following define puts lot of topology pointer checking in to this
// code.  It should be used only for debugging.  It slows things down
// substantially.
// #define SM_VALIDATE_TOPOLOGY

/*******************************************************************//**
PURPOSE: Constructor for a Merge object.  It simply sets the
    Merged Set and the Brep which will be merged into it.

NOTES:
***********************************************************************/
SmMerge::SmMerge
  (const SmContext & crContext,
   SmBrep          * pBrep,
   SmBrep          * pOtherBrep,
   double            dApproxTol3d,          // default 0.0 causes smlib to calculate a tolerance based on the geometry
   double            dAngleTolRadians,      // default 0.0 causes smlib to use 20 degrees
   SmMergeOptions  * pOptMergeOptions )     // default Null
 : m_vTI(crContext,pBrep,pOtherBrep,dApproxTol3d,dAngleTolRadians),
   m_bManifoldBoolean(TRUE),
   m_bCookieCutter(FALSE),
   m_bImprinting(FALSE),
   m_bBooleanPostProcess(TRUE),
   m_bIntersectionLoopClosed(TRUE),
   m_bImprintAndClassifyFaces(FALSE),
   m_bKeepOtherBrep(FALSE),
   m_lTagID(0)
{
  SM_ASSERT(pBrep != NULL);
  // SM_ASSERT(pOtherBrep != NULL);  This can now be Null, for the Stitch methods.

  SetMergeOptions( pOptMergeOptions );

  // The user can ask us to calculate a tolerance based on the geometry by specifying a zero tolerance
  // It seems more successful than guessing
  if( dApproxTol3d < SM_EFF_ZERO )
    {
      // The tolerance should not be bigger than the Breps' tolerances.
      // That leads to combining vertices that shouldn't be combined.
      // Tests for combining topology always include the tols of the
      // topology objects themselves, and those are never smaller than
      // the Brep tol, so there's no sense in having this tol bigger than that.

      // I will leave a static switch here for various possibilities.
      // It is only for a convenience for debugging: if a case arises where
      // this value makes a difference, this makes it easier to test.
      // But if you find a case where a tolerance less than or equal to the
      // Breps' tolerances fails while greater than Brep tolerance succeeds,
      // dive in and find out why, because that's not how it's supposed to work.
      // [c.f. 396 441 458 459 460]

static constexpr int siWhichTol = 3;
      // The tolerances included here are all the Min of dEdgeSize*0.33 and a value
      // derived from the Breps' tolerances: dBrepTol =
      //  0:  4 * Max( Brep tol )  -- this should not be used.
      //  1:  4 * Sum of the two Brep's tols.
      //  2:  Sum of the two Brep's tols.  -- I'd say this is the very maximum, probably too big.
      //  3:  1/2 of Min of two Brep's tols.
      // Remember, these are only for debugging convenience...

      // Get the minimum Edge length.
      SmEdge *pE1 = NULL, *pE2 = NULL; // unused
      double dEdge1Size = 0.0, dEdge2Size = 0.0;
      pBrep->GetMinEdgeBBoxSize(dEdge1Size, pE1);
      pOtherBrep->GetMinEdgeBBoxSize(dEdge2Size, pE2);
      double dEdgeSize = smos_Min(dEdge1Size, dEdge2Size);

      // Get the Brep tol value to use.
      double dBrepTol;
      switch ( siWhichTol )
      {
        case 0:
          dBrepTol = 4.0 * smos_Max( pBrep->GetTolerance(), pOtherBrep->GetTolerance() );
          break;

        case 1:
          dBrepTol = 4.0 * ( pBrep->GetTolerance() + pOtherBrep->GetTolerance() );
          break;

        case 2:
          dBrepTol =       ( pBrep->GetTolerance() + pOtherBrep->GetTolerance() );
          break;

        default:
          dBrepTol = 0.5 * smos_Min( pBrep->GetTolerance(), pOtherBrep->GetTolerance() );
          break;
      }

      m_vTI.m_dThisApproxTol3d = smos_Min( dBrepTol, dEdgeSize*0.33 );

    } // Done setting m_dThisApproxTol3d

  // The user can ask us to calculate an angle tolerance
  // Use 20 degrees as a default angle tolerance
  if( dAngleTolRadians == 0.0 )
    {
      m_vTI.m_dThisAngTolRad = SM_DEG2RAD(20.0);
    }
} // end SmMerge constructor

/*******************************************************************//**
PURPOSE: Constructor for a Merge object.  It simply sets the
    Merged Set and the Brep which will be merged into it.

NOTES:
***********************************************************************/
SmMerge::SmMerge
  (const SmContext & crContext)
 : m_vTI(crContext,NULL,NULL,SM_EFF_ZERO_SQ,SM_DEG2RAD(30)),
   m_bCookieCutter(FALSE),
   m_bImprinting(FALSE),
   m_bBooleanPostProcess(TRUE),
   m_bImprintAndClassifyFaces(FALSE),
   m_bKeepOtherBrep(FALSE),
   m_lTagID(0)
{
}

//
// Local routines:
//
/*******************************************************************//**
PURPOSE: Find the Region in this Brep that contains a given Faceuse
            from Other Brep,
***********************************************************************/
static SmRegion * sm_FindBrepRegionOfOtherFace
  (SmFaceuse             * pFaceuseInOtherBrep,  // in : Target Face to classify
   SmTopologyIntersector & rTopoIntersector)     // in : Current Merge operator control state

{
  SmRegion *pRegion = NULL;  // return value.

  // First no-work check: if there's only one region:
  if ( rTopoIntersector.GetPrimaryBrep()->GetSize() == 1 )
    {
      SmTopology *pList = rTopoIntersector.GetPrimaryBrep()->GetList();
      pRegion = SM_CAST_PTR( SmRegion, pList );
      if ( pRegion != NULL )
        { return pRegion; }
    }

  SmStatus eStat;

  // First try something relatively quick.
  // If any edge in the given faceuse has a mate in the pBrep,
  // then find its radial sector.
  SmEdgeuse * pData1[256];
  SmTArray<SmEdgeuse*> sEdgeuses(256,pData1);

  pFaceuseInOtherBrep->GetEdgeuses( sEdgeuses );
  ULONG ii, lNumEUs = sEdgeuses.GetSize();
  for( ii=0; ii<lNumEUs; ii++ )
    {
      // look for mated edge
      SmEdgeuse *pEU       = sEdgeuses[ii];
      SmEdge    *pEdge     = pEU->GetEdge();
      SmEdge    *pBrepEdge = (SmEdge*)rTopoIntersector.GetBrepMate(pEdge);

      // skip edges without mates
      if (!pBrepEdge) { continue; }

      // classify the OtherBrep edgeuse against the Brep Edge sectors
      SmEdgeuse *pFoundEU = NULL;
      SmFaceuse *pFoundFU = NULL;
      if (pBrepEdge->FindRadialSector(pEU,NULL,SM_OT_SAME,pFoundEU,pFoundFU) != SM_SUCCESS)
        {
          // skip OtherBrep edgeuse that fail to classify
          continue;
        }

      if ( pFoundEU ) // Classified to the sector of pFoundEU.  Use that sector's region.
      {
          pRegion = pFoundEU->GetShell()->GetRegion();
      }
      else if ( pFoundFU ) // If there's a coincident Face, use that.
      {
          pRegion = pFoundFU->GetShell()->GetRegion();
      }
      else // It's an error not to classify to something.
      {
          return NULL;
      }


#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
      if(bDebugMe)
        {
          smgfx_Erase();
          smgfx_SetLook(1,2, 0,0,1); rTopoIntersector.GetPrimaryBrep()->Draw(TRUE); sm_GraphicsLoop();
          smgfx_SetLook(1,2, 0,1,0); pFaceuseInOtherBrep->Draw(SM_DM_CROSSHATCH,10,10); sm_GraphicsLoop();
          smgfx_SetLook(3,4, 1,0,0); pEdge->Draw(); sm_GraphicsLoop() ;
          smgfx_SetLook(4,5, 0,1,1); pEU->Draw(); sm_GraphicsLoop() ;
          if ( pFoundEU ) {smgfx_SetLook(4,5, 1,0,1); pFoundEU->Draw(); sm_GraphicsLoop() ; }
          if ( pFoundFU ) {smgfx_SetLook(4,5, 1,0,1); pFoundFU->Draw(); sm_GraphicsLoop() ; }
          if ( pRegion  ) {smgfx_SetLook(1,2, 0,0,0); pRegion ->Draw(); sm_GraphicsLoop() ; }
          sm_GraphicsLoop();
        }
#endif // SM_DEBUG_CODE

      break;  // Found one, we're done.

    } // end iter every input face->edgeuse

  // If we found one, we're done.
  if ( pRegion != NULL ) {
      return pRegion;
  }

// The quick region-classify trick using mated edges failed to find a region.
// We have to do actual point classification against pBrep.

  // for every input face edgeuse
  pFaceuseInOtherBrep->GetEdgeuses(sEdgeuses);
  lNumEUs = sEdgeuses.GetSize();
  for(ii=0; ii<lNumEUs; ii++)
    {
      // If this Edgeuse had a mated Brep Edge, skip it.
      SmEdgeuse * pEU            = sEdgeuses[ii];
      SmEdge    * pEdge          = pEU->GetEdge();
      SmZoneTol3d sEdgeZoneTol3d = SmTol::GetZoneTol3d(pEdge) ;
      SmEdge    * pBrepEdge      = (SmEdge*)rTopoIntersector.GetBrepMate(pEdge);

      if ( pBrepEdge != NULL )
        { continue; }

      // There is no mated Brep Edge.
      // Classify a point on the curve against the Brep.
      SmPoint3d sPnt;
      eStat = pEdge->GetCurve()->EvaluatePoint(
              pEdge->GetInterval().Evaluate(0.456789), sPnt );
      if ( eStat != SM_SUCCESS )
        { continue; }

      SmPointClassification sPC(sEdgeZoneTol3d, pEdge->GetContext()) ;
      SmTemporaryChangeValue<SmBoolean> sChange(rTopoIntersector.GetPrimaryBrep()->m_bEditingEnabled, FALSE );
      eStat = rTopoIntersector.GetPrimaryBrep()->Point3DClassify( sPnt,
                                                                  0.0, FALSE, sPC );
      if ( eStat != SM_SUCCESS )
        { continue; }

      // skip points that don't classify to a region
      if ( sPC.GetPointClass() != SM_PC_REGION )
        { continue; }

      // use the point's classified region as the input face's Brep region
      pRegion = (SmRegion*)sPC.GetObject();
      break;

    } // end iter every face edgeuse

    return pRegion;

} // end sm_FindBrepRegionOfOtherFace()

/*******************************************************************//**
PURPOSE: Set options from an SmMergeOptions object.
***********************************************************************/
void SmMerge::SetMergeOptions( SmMergeOptions *pOptions )
{
  if ( pOptions == NULL ) { return; } // ok to pass in Null.

  SetBooleanPostProcess ( pOptions->GetBooleanPostProcess()      );
  SetCookieCutter       ( pOptions->GetCookieCutter()            );
  SetClassifyFaces      ( pOptions->GetImprintAndClassifyFaces() );
  SetImprinting         ( pOptions->GetImprinting()              );
  SetKeepOtherBrep      ( pOptions->GetKeepOtherBrep()           );

} // end SmMerge::SetOptions

/*******************************************************************//**
PURPOSE: Merge the Other Brep into the Original one face at a time.
   This is a good thing to do if you have a Brep where it may
   have self-intersections in the 'Other' brep.

NOTES: The m_vTI.m_pOther will not be deleted by this merge.

  When using the bCreateNewBrepsForFaces == TRUE option, each face
  is treated as a sheet
***********************************************************************/
SmStatus SmMerge::PiecewiseMerge
  (SmRelation<SmSurface,SmSurface> * pSurfacePairsToSkipSSI, // in : This relation contains pairs of surfaces to skip Surf/Surf intersection.
   SmBoolean bUseExistingTopology,                           // in : TRUE = use existing edges between surfaces to eliminate intersections between them.
                                                             //      (not currently used in this function)
   SmBoolean bCreateNewBrepsForFaces,                        // in : TRUE = create a new Brep for each face prior to the merge. This treats each
                                                             //             pOther face as a sheet face being merged into pBrep.
   SmBrep *& rpResult)                                       // out: Pointer to resulting Brep or NULL for failure
{
  // let sSurfaces = m_pOther Surface list
  SmTArray<SmSurface*> sSurfaces;
  m_vTI.m_pOther->GetSurfaces(sSurfaces);

  // locals
  SmTArray<SmObject*>        sSurfaceSubset;
  SmTArray<SmSurface*>       sSurfUsers, sSkipSurfs, sSkipUsers;
  SmTArray<SmAObject*>       sAObjectUsers ;
  SmTArray<SmCurve*>         s3DCurves;
  SmTArray<SmCurve*>         sUV1Curves, sUV2Curves;
  SmTArray<SmTsectCurveType> sOrientations;
  SmTArray<double>           sDeviations;

  // local target Brep pointers
  SmBrep *pFinalResult = m_vTI.m_pBrep;
  SmBrep *pOtherBrep   = m_vTI.m_pOther;

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe     = FALSE ;
SmBoolean bDebugMe1    = FALSE ;
SmBoolean bDrawRegions = FALSE;
SmBoolean bDrawShells  = FALSE;
SmBoolean bDrawFaces   = TRUE;
ULONG lCount      = 1 ; lCount++ ;
  if(bDebugMe)
    {
      pFinalResult->Dump() ;  // m_vTI.m_pBrep
      pOtherBrep->Dump() ;    // m_vTI.m_pOther

      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0,0,1) ; m_vTI.m_pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,0) ; m_vTI.m_pOther->Draw(TRUE) ; sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

  // Since we loop over surfaces, not Faces, we have to have Faces and
  // surfaces in 1-1 correspondences (when not making Composite Faces).
  // Do that by one initial call to IntersectInsertRelate().

  // Possible optimization: every call to ManifoldBoolean() does this again.
  // After the first call, it's always the same, so could we skip it on each
  // iteration?  No, ManifoldBoolean() needs the Relate part each time.
  // Maybe think of a way around repeating Intersect and Insert each time...

  m_vTI.IntersectInsertRelate();
  m_vTI.m_pOther->GetSurfaces( sSurfaces );

  // reinitizialize Brep<=>Other entity maps
  m_vTI.m_pBToO->RemoveAll();
  m_vTI.m_pOToB->RemoveAll();

  // If we set up the skip stuff now then it will take care of the case where some of
  // the surfaces are already merged into the target brep.

  // when asked to skip specified surface pairs
  //  make sure there is an SM_AI_SURFACEMAP attribute on every otherBrep surface
  //  make sure there is an SM_AI_SURFACEMAP attribute on every Brep surface being skipped
  ULONG ii, jj, lNumSurfs = sSurfaces.GetSize();
  if (pSurfacePairsToSkipSSI)
    {
      // for every OtherBrep surface
      for(ii=0; ii<lNumSurfs; ii++)
        {
          SmSurface *pSurface = sSurfaces[ii];

          // Add an SM_AI_SURFACEMAP attribute to the surface
          SmPointerAttribute *pPointerAtt =
              new (m_vTI.m_crContext) SmPointerAttribute(SM_AI_SURFACEMAP,pSurface,SM_AB_REFERENCE);
          pSurface->AddAttribute(pPointerAtt);

          // get list of Brep surfaces to skip for this surface
          pSurfacePairsToSkipSSI->GetSeconds(pSurface,sSkipSurfs);

          // for every Brep surface being skipped
          ULONG lNumSkipSurfs = sSkipSurfs.GetSize();
          for(jj=0; jj<lNumSkipSurfs; jj++)
            {
              SmSurface *pSkip = sSkipSurfs[jj];

              // make sure the skipped Brep surfaces have an SM_AI_SURFACEMAP attribute
              SmPointerAttribute *pOtherAtt = (SmPointerAttribute*)pSkip->FindAttribute(SM_AI_SURFACEMAP);
              if (!pOtherAtt)
                {
                  pOtherAtt =  new (m_vTI.m_crContext) SmPointerAttribute(SM_AI_SURFACEMAP,pSkip,SM_AB_REFERENCE);
                  pSkip->AddAttribute(pOtherAtt);
                } // end pOtherAtt == FALSE check
            } // end iter every sSkipSurfs
        } // end iter every sSurfaces
    } // end pSurfacePairsToSkipSSI != NULL check

  // for every OtherBrep surface
  for(ii=0; ii<lNumSurfs; ii++)
    {
      // set m_vTI.m_pSubset = { pSurface }
      SmSurface *pSurface = sSurfaces[ii];
      sSurfaceSubset.ReSet();
      sSurfaceSubset.Add(pSurface);
      m_vTI.SetSubset(sSurfaceSubset);

      // future accelerator option - register OtherBrep intersections
      //  to prevent them from being found a second time as the surfaces
      //  are added to Brep.
      if (bUseExistingTopology) {
          // Here we could get faces of surface and use
          // the topology to eliminate intersections that
          // are along edges that were created when registering them.
      }

      // when asked to place each OtherBrep face in its own Brep
      if (bCreateNewBrepsForFaces)
        {
          // create a new Brep and place that in m_vTI
          SmBrep *pNewBrep = new (m_vTI.m_crContext) SmBrep();
          // gwc: removed next line (moved to constructor) - sets pNewBrep->Tol = m_vTI.m_crContext::ZoneTol3d
          // jgu: restoring next line for old tol model.
#ifndef SM_USE_NEWTOL
          pNewBrep->SetTolerance(pFinalResult->GetTolerance());
#endif // !SM_USE_NEWTOL

          
          pNewBrep->m_bEditingEnabled = TRUE;

// Remove Composites
// //cbi_CEdge: 4
// #ifndef SM_NO_COMPOSITES
//           pNewBrep->m_bMakeComposites = TRUE;
// #endif  // SM_NO_COMPOSITES

          m_vTI.m_pOther = pOtherBrep;
          m_vTI.m_pBrep  = pNewBrep;
        }

      // pointer to manifold boolean output
      SmBrep *pResult;

      // skip wireEdges, no lamina edge
      m_vTI.m_bIntersectWireEdges   = FALSE;
      m_vTI.m_lIntersectLaminaEdges = 0;  // 3 = Only intersect lamina edges of other

#ifdef SM_DEBUG_CODE
      if(bDebugMe)
        {
          m_vTI.m_pBrep->Dump() ;   // bCreateNewBrepsForFaces == TRUE: new empty Brep - to get one face from original Brep being merged
                                    // bCreateNewBrepsForFaces == FALSE: pFinalResult
          m_vTI.m_pOther->Dump() ;  // original Brep being piecewise merged
          if(   pFinalResult != m_vTI.m_pBrep
             && pFinalResult != m_vTI.m_pOther) pFinalResult->Dump() ;
          if(   pOtherBrep   != m_vTI.m_pBrep
             && pOtherBrep   != m_vTI.m_pOther) pOtherBrep->Dump() ;

          smgfx_Erase() ;
          smgfx_SetLook(1,2, 0,0,1) ; if(m_vTI.m_pBrep) m_vTI.m_pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
          smgfx_SetLook(2,3, 0,1,0) ; if(m_vTI.m_pOther) m_vTI.m_pOther->Draw(TRUE) ; sm_GraphicsLoop() ;
          smgfx_SetLook( 3, 4, 0, 1, 1 ); if(pFinalResult != m_vTI.m_pBrep && pFinalResult != m_vTI.m_pOther)
                                            { pFinalResult->Draw( TRUE ); sm_GraphicsLoop(); }
          smgfx_SetLook( 4, 5, 1, .5, 0 ); if(pOtherBrep != m_vTI.m_pBrep && pOtherBrep != m_vTI.m_pOther)
                                            { pOtherBrep->Draw( TRUE ); sm_GraphicsLoop(); }
          smgfx_SetLook(1,2, .5,1,0); pSurface->DrawUV(4,4) ; sm_GraphicsLoop() ;
          sm_GraphicsLoop() ;
        }
#endif // SM_DEBUG_CODE
      // Merge one m_vTI.m_pOther surface and its faces (specified by subset group) into
      // (bCreateNewBrepsForFaces == TRUE ) pNewBrep making a 1 face stand-alone
      //                                    intermediate Brep (no intersections yet - another
      //                                    ManifoldBoolean call will be made that merges this single
      //                                    face into pFinalBrep complete with intersections), or
      // (bCreateNewBrepsForFaces == FALSE) pFinalBrep incrementing the number faces
      //                                    with each iteration.
        {
          SmNewMarkAndLock sMarkLock ;
          SER(ManifoldBoolean(SM_BO_PARTIAL_MERGE,
                              pResult,
                              &sMarkLock)) ; // i/o : sets and incs MarkValue, NotNULL=marks valid after call, NULL=not valid
                                             //      default:[NULL]
          // on the last pass
          // Do all of wire edges and shell vertices on last time around
          if (ii==lNumSurfs-1)
            {
              SmTArray<SmEdge*> sKeepAsWires ;
              SER(ProcessWiresAndShellVertices(SM_BO_MERGE,                 // in :
                                               sKeepAsWires,                // out: list of all Brep Edges mated to
                                                                            //      Other wires that should be kept in the
                                                                            //      final output should all the Brep faces
                                                                            //      attached to these edges be deleted.
                                               sMarkLock.GetMarkType1(),    // in : MarkType for m_vTI.m_pBrep  from call ManifoldBoolean(), not incremented
                                               sMarkLock.GetMarkType2()));  // in : MarkType for m_vTI.m_pOther from call ManifoldBoolean(), not incremented
            }
        } // end scope for sMarkLock

      // re-initialize relationships
      m_vTI.m_pBToO->RemoveAll();
      m_vTI.m_pOToB->RemoveAll();

#ifdef SM_DEBUG_CODE
      if (bDebugMe)
        {
          pResult->Dump() ;
          SM_ASSERT_VALID(pResult) ;

          smgfx_Erase();
          smgfx_SetLook(1,2, 1,0,0) ; pResult->Draw(TRUE); sm_GraphicsLoop() ;
          smgfx_SetLook(1,2) ; pFinalResult->Draw(TRUE); sm_GraphicsLoop();

          if(bDrawFaces)
            {
              SmTArray<SmFace *> sFaces ;
              pFinalResult->GetFaces(sFaces) ;
              ULONG di, lNumFaces = sFaces.GetSize();
              for(di=0; di<lNumFaces; di++)
                {
                  SmFaceuse *pFaceuseUp, *pFaceuseDown ;
                  SmFace *pFace = sFaces[di] ;
                  pFace->GetFaceuses(pFaceuseUp, pFaceuseDown) ;

                  smgfx_Erase() ;
                  smgfx_SetLook(1,2) ; pFinalResult->Draw(TRUE); sm_GraphicsLoop();
                  smgfx_SetLook(3,5, 0,1,0) ; pFace->Draw(SM_DM_CROSSHATCH) ; sm_GraphicsLoop() ;
                  smgfx_SetLook(1,2, 0,1,0) ; pFaceuseUp->Draw(SM_DM_FACEUSENEIGHBORS) ; sm_GraphicsLoop() ;
                  sm_GraphicsLoop() ;

                  smgfx_Erase() ;
                  smgfx_SetLook(1,2) ; pFinalResult->Draw(TRUE); sm_GraphicsLoop();
                  smgfx_SetLook(3,5, 0,1,0) ; pFace->Draw(SM_DM_CROSSHATCH) ; sm_GraphicsLoop() ;
                  smgfx_SetLook(1,2, 0,1,0) ; pFaceuseDown->Draw(SM_DM_FACEUSENEIGHBORS) ; sm_GraphicsLoop() ;
                  sm_GraphicsLoop() ;

                  pFace->Dump() ;
                }
            }
          sm_GraphicsLoop();
        }
#endif // SM_DEBUG_CODE

      // Here we should set a map from the original surface
      // to the new surface.

      // when asked to create newBreps for each Face
      // need a 2nd Merge operation to merge the 1 face Brep (m_vTI.m_pOther) into pFinalBrep
      if (bCreateNewBrepsForFaces)
        {
//            delete m_vTI.m_pOther;  // not needed - the upcoming ManifoldBoolean call will delete this Brep
          // when given surface pairs to skip
          if (pSurfacePairsToSkipSSI)
            {
              // get list of pSurface SurfUsers through its SM_AI_SURFACEMAP attribute.
              SmAttribute *pSurfAttr = pSurface->FindAttribute(SM_AI_SURFACEMAP);

              // Temp workaround to appease Linux gcc compiler
              // Original code with typecast has warning: dereferencing type-punned pointer will break strict-aliasing rules [-Wstrict-aliasing]
              // pSurfAttr->GetUsers(SM_REINTERPRET_CAST(SmTArray<SmAObject*>&,sSurfUsers));
              pSurfAttr->GetUsers(sAObjectUsers) ;
              sSurfUsers.ReSet();
              for(jj=0;jj<sAObjectUsers.GetSize();jj++) { sSurfUsers.Add((SmSurface *)sAObjectUsers[jj]) ; }

              // get all surfaces to skip for this Surface
              pSurfacePairsToSkipSSI->GetSeconds(pSurface,sSkipSurfs);

              // for every SurfUser face
              ULONG jjj, lNumSurfUsers = sSurfUsers.GetSize();
              for(jjj=0; jjj<lNumSurfUsers; jjj++)
                {
                  SmSurface *pSurfUser = sSurfUsers[jjj];

                  // for every skip surface
                  ULONG kkk, lNumSkipSurfs = sSkipSurfs.GetSize();
                  for(kkk=0; kkk<lNumSkipSurfs; kkk++)
                    {
                      SmSurface *pSkip = sSkipSurfs[kkk];

                      // for skip surface labelled with SM_AI_SURFACEMAP attributes
                      SmAttribute *pSkipAttr = pSkip->FindAttribute(SM_AI_SURFACEMAP);
                      if (!pSkipAttr) continue;

                      // get list of Users of skipSurface
                      // Temp workaround to appease Linux gcc compiler (can we reuse the array?)
                      // Original code with typecast has warning: dereferencing type-punned pointer will break strict-aliasing rules [-Wstrict-aliasing]
                      // pSkipAttr->GetUsers(SM_REINTERPRET_CAST(SmTArray<SmAObject*>&,sSkipUsers));
                      pSkipAttr->GetUsers(sAObjectUsers) ;
                      sSkipUsers.ReSet();
                      for(jj=0;jj<sAObjectUsers.GetSize();jj++) { sSkipUsers.Add((SmSurface *)sAObjectUsers[jj]) ; }

                      // for every skipSurface user
                      ULONG lll, lNumSkipUsers = sSkipUsers.GetSize();
                      for(lll=0; lll<lNumSkipUsers; lll++)
                        {
                          SmSurface *pSkip2 = sSkipUsers[lll];

                          // register existing surfaces in m_vTI.m_pIntersectionTable
                          SER(m_vTI.RegisterExistingSSI(pSkip2,pSurfUser,s3DCurves,sUV1Curves,
                              sUV2Curves,sOrientations,sDeviations));
                        } // end iter every sSkipUsers
                    } // end iter every sSkipSurfs
                }  // end iter every surface to skip
            } // end pSurfacePairsToSkipSSI == TRUE check

          // set up the upcoming Manifold Boolean parameters
          m_vTI.m_pOther = m_vTI.m_pBrep;         // use new Brep as next Boolean's OtherBrep
          m_vTI.m_pBrep  = pFinalResult;          // build Boolean results into FinalOutput
          sSurfaceSubset.ReSet();                 // clear the PartialMerge SubSet
          m_vTI.SetSubset(sSurfaceSubset);

          m_vTI.m_bIntersectWireEdges   = FALSE;  // skip Wire edges
          m_vTI.m_lIntersectLaminaEdges = 3;      // Only intersect lamina edges of other
          m_bManifoldBoolean            = FALSE;  // don't force manifold results


          // Now Merge the new one-surface pNewBrep into Brep pFinalResult.
          //   side-effect: delete m_vTI.m_pOther which is equal to pNewBrep,
          //                the temp one-surface Brep made in this function.
          pFinalResult->Notify(SM_NO_PRE_EDIT, pFinalResult, NULL, NULL);
          //SER(ManifoldBoolean(SM_BO_MERGE,
          //                    pResult));

          { // JLMCC hunting memory leaks
              SmStatus sErr = (ManifoldBoolean(SM_BO_MERGE,pResult));
              if (sErr != SM_SUCCESS)
              {
                  if (m_vTI.m_pOther)
                  {
                      delete m_vTI.m_pOther;
                      m_vTI.m_pOther = NULL;
                  }
                  smos_ErrorMessage(sErr, FILE_NAME, LINE_NUMBER, (TCHAR*)0, (TCHAR*)0, FUNC_NAME);
                  return (sErr);
              }
          }

          // Clear up relationships and stored SSI curves
          m_vTI.m_pBToO->RemoveAll();
          m_vTI.m_pOToB->RemoveAll();
          if (pSurfacePairsToSkipSSI)
            {
              m_vTI.ClearExistingSSI();
            }
#ifdef SM_DEBUG_CODE
          // arrive here after adding the next surface (and its faces) from the input Brep to the pFinalBrep
          if (bDebugMe1)
            {
              SM_ASSERT(pResult == pFinalResult) ;
              SM_ASSERT_VALID(pFinalResult) ;
              pFinalResult->DumpTopology();
              pFinalResult->Dump();  // accumulation of all faces merged one at a time

              SmShell     *pShell   = NULL ;
              SmFaceuse   *pFaceuse = NULL ;
              SmVertex    *pVertex  = NULL ;
              if(pShell)   pShell->DumpTopology() ;
              if(pFaceuse) pFaceuse->GetFace()->DumpTopology() ;
              if(pVertex)  pVertex->DumpTopology() ;

              // draw the last surface to be inserted and the resulting brep
              smgfx_Erase();
              smgfx_SetLook(1,2, .5,1,0); pSurface->DrawUV(4,4) ; sm_GraphicsLoop() ;
              smgfx_SetLook(2,3) ; pResult->Draw(TRUE); sm_GraphicsLoop() ;
              smgfx_SetLook(4,5, 0,1,1) ; if(pShell)   pShell->Draw() ; sm_GraphicsLoop() ;
              smgfx_SetLook(6,7, 1,1,0) ; if(pFaceuse) pFaceuse->Draw() ; sm_GraphicsLoop() ;
              smgfx_SetLook(8,9, 1,0,0) ; if(pVertex)  pVertex->Draw() ; sm_GraphicsLoop() ;
              sm_GraphicsLoop();
              if (bDrawFaces)
                {
                  SmTArray<SmFace *> sFaces ;
                  pResult->GetFaces(sFaces) ;
                  ULONG di, lNumFaces = sFaces.GetSize();
                  for(di=0; di<lNumFaces; di++)
                    {
                      SmFaceuse *pFaceuseUp, *pFaceuseDown ;
                      SmFace *pFace = sFaces[di] ;
                      pFace->GetFaceuses(pFaceuseUp, pFaceuseDown) ;

                      smgfx_Erase() ;
                      smgfx_SetLook(1,2) ; pFinalResult->Draw(TRUE); sm_GraphicsLoop();
                      smgfx_SetLook(3,5, 0,1,0) ; pFace->Draw(SM_DM_CROSSHATCH) ; sm_GraphicsLoop() ;
                      smgfx_SetLook(1,2, 0,1,0) ; pFaceuseUp->Draw(SM_DM_FACEUSENEIGHBORS) ; sm_GraphicsLoop() ;
                      sm_GraphicsLoop() ;

                      smgfx_Erase() ;
                      smgfx_SetLook(1,2) ; pFinalResult->Draw(TRUE); sm_GraphicsLoop();
                      smgfx_SetLook(3,5, 0,1,0) ; pFace->Draw(SM_DM_CROSSHATCH) ; sm_GraphicsLoop() ;
                      smgfx_SetLook(1,2, 0,1,0) ; pFaceuseDown->Draw(SM_DM_FACEUSENEIGHBORS) ; sm_GraphicsLoop() ;
                      sm_GraphicsLoop() ;

                      pFace->Dump() ;
                    }
                }
              sm_GraphicsLoop() ;

              if(bDrawShells)
                {
                  SmTArray<SmRegion*> sRegions;
                  pResult->GetRegions(sRegions);
                  ULONG nn, lNumRegions = sRegions.GetSize();
                  for(nn=0; nn<lNumRegions; nn++)
                    {
                      SmRegion *pR = sRegions[nn];
                      if (bDrawRegions)
                        {
                          smgfx_Erase();
                          smgfx_SetLook(1,2, 0,1,1) ; pR->Draw(); sm_GraphicsLoop() ;
                          sm_GraphicsLoop();
                        }
                      SmTArray<SmShell*> sShells;
                      pR->GetShells(sShells);
                      ULONG nnn, lNumShells = sShells.GetSize();
                      for(nnn=0; nnn<lNumShells; nnn++)
                        {
                          if (bDrawShells)
                            {
                              smgfx_Erase();
                              smgfx_SetLook(1,2, 0,1,1) ; sShells[nnn]->Draw(); sm_GraphicsLoop() ;
                              sm_GraphicsLoop();
                            }
                        }
                    }
                }
              sm_GraphicsLoop() ;
            } // end bDebugMe3 check
#endif // SM_DEBUG_CODE

        } // end bCreateNewBrepsForFaces == TRUE check

    } // end iter ii every sSurfaces

  // set the output
  rpResult = m_vTI.m_pBrep;
  if(bCreateNewBrepsForFaces)
    {
      SM_ASSERT(   m_vTI.GetOtherBrep() == NULL
                || m_vTI.GetOtherBrep() == pOtherBrep) ;
      m_vTI.m_pOther = pOtherBrep ;
    }

#ifdef SM_DEBUG_CODE
  if (bDebugMe)
    {
      smgfx_Erase();
      smgfx_SetLook(1,2) ;        m_vTI.m_pBrep->Draw(); sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 1,0,0) ; m_vTI.m_pOther->Draw(TRUE); sm_GraphicsLoop();
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

  // all done
  return SM_SUCCESS;

} // end SmMerge::PiecewiseMerge

/*******************************************************************//**
PURPOSE: Post process after the Boolean operation and clean up
  topological edges and vertices.

NOTES:
  Topological edges and vertices are those that could be removed without
  changing the shape of the model.  Topological edges are those that bound
  two faces that are, or could be, on the same surface.  When an edge is
  removed, the surface of one of its faces is expanded to cover the other
  surface -- the expanded surface must be coincident with the other surface
  over the domain of the other.  Similarly, when a topological vertex is
  removed, its two edges become a single edge on a curve that covers
  both originals.

  This method does essentially the same thing as
  SmBrep::RemoveTopologicalEdgesAndVertices().
  If you change one, you should check whether the other should be changed as well.

  housekeeping note: removing an edge can cause other topology objects, like
  a face or a vertex to be deleted as well leaving stale pointers in the m_vTI
  BrepToOther entity maps.  Although m_vTI will contain stale pointers
  after this call, that does not affect the merge operator due to the way it's
  written.

  Unless we modify the DeleteEdge, DeleteVertex, DeleteTopologicalVertex,
  and TransferChildrenTo functions to track and return all the entities they
  delete this function can't remove all the stale pointers from those lists.

  When debugging - don't dump m_vTI after this call.

***********************************************************************/
SmStatus SmMerge::BooleanPostProcess
  (SmBoolean bCheckAllEdges)          // in : TRUE = check all Brep edges
                                      //      FALSE= check only edges with an OtherBrep mate
{
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe=FALSE;
  if (bDebugMe)
    {
      smgfx_Erase();
      smgfx_SetLook(1,2, 0,0,1) ; m_vTI.m_pBrep->Draw(TRUE); sm_GraphicsLoop() ;
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

  // locals
  // SM_PTR_ARRAY(sFaces,        SmFace,    32) ;
  SM_PTR_ARRAY(sEdgeVertices, SmVertex,  16) ;
  SM_PTR_ARRAY(sEdges,        SmEdge,   128) ;

  // Get Edges to Check
  if(bCheckAllEdges) { m_vTI.m_pBrep->GetEdges( sEdges ); }
  else               { sEdges = m_sIntersectionEdges; } // This list is put together during the Boolean operation.

  // locals
  ULONG ii ;
  SmSurface * pSurfToKeep = NULL;
  double      dDistBetween = SM_BIG_DOUBLE ;
  ULONG       lNumEdges = sEdges.GetSize();

  // for every edge - find and remove topological edges (maybe merging faces) without changing the Brep shape
  for ( ii=0; ii<lNumEdges; ii++ )
    {
      SmEdge *pEdge = sEdges[ii];

#ifdef SM_DEBUG_CODE
      if (bDebugMe)
        {
          smgfx_Erase();
          smgfx_SetLook(1,2, 0,0,1); if(m_vTI.m_pBrep) m_vTI.m_pBrep->Draw(TRUE); sm_GraphicsLoop() ;
          smgfx_SetLook(3,4, 1,0,0); pEdge->Draw(); sm_GraphicsLoop() ;
          sm_GraphicsLoop();
        }
#endif // SM_DEBUG_CODE

      // pEdge may have been deleted.  If so, it will have been removed from the maps.
      if ( ! bCheckAllEdges )
        {
          SmObject *pOtherObj = m_vTI.GetOtherMate( pEdge );
          if ( pOtherObj == NULL )
            { continue; }
        }

      // skip NonTopological edges - side effect - one Surface may be expanded to 'cover' the other
      SmExtent2d sMergedFaceDomain;
      if(!pEdge->IsTopological(pSurfToKeep, dDistBetween, 10.0, &sMergedFaceDomain)) // note: ApproxTol3d Gain = 10.0 is different than
        { continue ; }                                           //       Gain  1.0 Tol used in SmBrep::RemoveTopologicalEdgesAndVertices()

      // code moved to SmEdge::IsTopological()
      //    // skip nonManifold edges - these are not "topological"
      //    if ( !pEdge->IsManifold() )
      //      { continue; }
      //
      //    // get Edge faces
      //    pEdge->GetFaces(sFaces);
      //    ULONG lNumFaces = sFaces.GetSize();
      //
      //    if ( lNumFaces == 0 || lNumFaces > 2 )
      //      { continue; }  // SER(SM_ERR);
      //
      //    // We need to check whether it's a seam on the whole underlying surface;
      //    // this test just checks for the faces being different.
      //    // Example: two identical cylinders, with two 90-degree faces on them,
      //    // with a common (straight) edge.  The edge could be deleted, but not if
      //    // it's a seam of the surviving surface.
      //    //   [090505; Fillets 1 2 126 409]
      //    // if (pEdge->IsSeam(*sFaces[0]->GetSurface()))
      //    //   { continue; }
      //
      //    // Do the more specific seam test.
      //    SmBoolean bIsSeam     = FALSE;
      //    SmBoolean bIsTangent  = TRUE; // We'll also check whether the edge is tangent.
      //    SmBoolean bIsGoodGap  = TRUE;
      //
      //    // Work from Edgeuses, because they have uv values for SmSurface::IsOnSeam().
      //    SmTArray<SmEdgeuse*> sEUs ;
      //    pEdge->GetEdgeuses( sEUs ) ;
      //    ULONG jj, lIdx, lNumEUs = sEUs.GetSize() ;
      //    SmTArray<SmSurface*> sSurfsSeen ; // To prevent checking the same surface twice.
      //    SmPoint2d sUV ;
      //    SmPoint3d sUV3d ;
      //    double dEdgeTol = pEdge->GetTolerance();
      //
      //    // for every edgeuse - see if it's a seam - seam edges should not be removed
      //    for ( jj = 0; jj < lNumEUs; jj += 2 )
      //      {
      //        SmEdgeuse *pEU   = sEUs[jj];
      //        SmFace    *pFace = pEU->GetFace();
      //        SmSurface *pSurf = pFace->GetSurface();
      //        if ( sSurfsSeen.FindElement( pSurf, lIdx ) )
      //          { continue; }
      //        sSurfsSeen.Add( pSurf );
      //
      //        // Check this surface.
      //        // Check a couple of points, but not end points,
      //        // which are also on an adjacent edge.
      //        // IsOnSeam() works on the surface's entire domain, which is what we want.
      //        pEU->NormalizedEvaluate( 0.37, TRUE, sUV3d );  // TRUE = UV Eval, FALSE = 3d Eval
      //        sUV.Set( sUV3d.x, sUV3d.y );
      //
      //        // Do a check of the normals while we're here.
      //        // It's cheaper than checking OnSeam, or covering,
      //        // and it catches over 2/3 of the cases in prog_test.
      //        // Only need to check one: we know the edge is manifold.
      //        // Also look at the gap size at the test point - if the gap is too large
      //        //  don't treat this as a topological edge - leave the too big gap in the
      //        //  database to be found later by an AssertValid check.
      //        if ( jj == 0 )
      //          {
      //            if ( lNumFaces == 2 && sFaces[0]->GetSurface() != sFaces[1]->GetSurface() )
      //              {
      //                // evaluate surfaces on either side of the edge
      //                SmVector3d sNorm1, sNorm2;
      //                pSurf->EvaluateNormal( sUV, FALSE, FALSE, sNorm1 );
      //                SmEdgeuse *pEU2 = pEU->GetRadial()->GetMate();  // same orientation
      //                pEU2->NormalizedEvaluate( 0.37, TRUE, sUV3d );  // TRUE = UV Eval, FALSE = 3d Eval
      //                SmPoint2d sUV2( sUV3d.x, sUV3d.y );
      //                SmSurface *pSurf2 = pEU2->GetFace()->GetSurface();
      //                pSurf2->EvaluateNormal( sUV2, FALSE, FALSE, sNorm2 );
      //
      //                // tangent check
      //                if ( smos_Fabs( sNorm1.Dot( sNorm2 ) ) < 1.0 - SM_EFF_ZERO_SQRT )
      //                  {
      //                    bIsTangent = FALSE;
      //                    break;
      //                  }
      //
      //                // gap check
      //                double dGapTol = sFaces[0]->GetTolerance() + sFaces[1]->GetTolerance() + pEdge->GetTolerance() ;
      //                SmPoint3d sPt1, sPt2;
      //                pSurf->EvaluatePoint( sUV, sPt1 );
      //                pSurf2->EvaluatePoint( sUV2, sPt2 );
      //                if ( sPt1.DistanceBetween( sPt2 ) >dGapTol )
      //                  {
      //                    bIsGoodGap = FALSE ;
      //                    break ;
      //                  }
      //              }
      //          } // end Normals test.
      //
      //        bIsSeam = pSurf->IsOnSeam( sUV, &dEdgeTol );
      //        if ( bIsSeam )
      //          {
      //            pEU->NormalizedEvaluate( 0.57, TRUE, sUV3d );  // TRUE = UV Eval, FALSE = 3d Eval
      //            sUV.Set( sUV3d.x, sUV3d.y );
      //            bIsSeam = pSurf->IsOnSeam( sUV, &dEdgeTol );
      //          }
      //        if ( bIsSeam )
      //          {
      //            pEU->NormalizedEvaluate( 0.82, TRUE, sUV3d );  // TRUE = UV Eval, FALSE = 3d Eval
      //            sUV.Set( sUV3d.x, sUV3d.y );
      //            bIsSeam = pSurf->IsOnSeam( sUV, &dEdgeTol );
      //          }
      //        if ( bIsSeam )
      //          { break; }
      //
      //      } // end iter every edgeuse testing for a seam
      //
      //    // skip seams - those edges are always needed
      //    if ( bIsSeam || !bIsTangent || !bIsGoodGap)
      //      { continue; }
      //
      //    // End seam check.
      //
      //    pSurfToKeep = NULL;
      //
      //    // when edge is connected to 2 different faces
      //    if ( lNumFaces == 2 )
      //      {
      //        // When the faces are not on the same surface, see if surfaces are coincident
      //        // and one could be contained in the other, possibly expanded.
      //        if (sFaces[0]->GetSurface() != sFaces[1]->GetSurface())
      //          {
      //            SmSurface *pSurface0 = sFaces[0]->GetSurface();
      //            SmSurface *pSurface1 = sFaces[1]->GetSurface();
      //
      // #ifdef SM_DEBUG_CODE
      //            if (bDebugMe)
      //              {
      //                SM_ASSERT_VALID(m_vTI.m_pBrep) ;
      //
      //                smgfx_Erase();
      //                smgfx_SetLook(1,2, 0,0,1) ; m_vTI.m_pBrep->Draw(TRUE); sm_GraphicsLoop() ;
      //                smgfx_SetLook(3,4, 1,0,0) ; pEdge->Draw(); sm_GraphicsLoop();
      //                smgfx_SetLook(2,3, 0,1,1) ; pSurface0->DrawUV() ; sm_GraphicsLoop() ;
      //                smgfx_SetLook(4,5, 0,0,1) ; pSurface1->DrawUV(5,5) ; sm_GraphicsLoop() ;
      //                sm_GraphicsLoop();
      //              }
      // #endif // SM_DEBUG_CODE
      //
      //            // Get tolerance.
      //            // (Note: in SmBrep::RemoveTopologicalEdgesAndVertices() tolerance handling is different.)
      //            double dTol = (sFaces[0]->GetTolerance() + sFaces[1]->GetTolerance()) / 10.0 ;
      //
      //            double dDistanceBetween = SM_BIG_DOUBLE;
      //
      //           // Do the coincidence check.
      //           eStat = pSurface0->CoverOtherCoincidentSurface( pSurface1, dTol,
      //                                                            dDistanceBetween,
      //                                                            pSurfToKeep );
      //
      //            if ( pSurfToKeep == NULL || eStat != SM_SUCCESS )
      //             { continue; }  // Surfaces not coincident or one couldn't cover the other.
      //  end code moved to SmEdge::IsTopological()


      // Arrive here when pEdge is known to be Manifold and Topological,
      //   pSurfToKeep contains the other, with coincidence,
      //   and its domain has been updated if necessary.
      // Next: - Set pSurfaceToKeep->Faces->Domain = pSurfToKeep->GetUVDomain
      //       - make list of Edge->Vertices for later
      //       - Delete Edge

      // locals
      SM_PTR_ARRAY(sEdgeuses, SmEdgeuse, 8) ;
      pEdge->GetEdgeuses(sEdgeuses) ;
      SM_ASSERT(sEdgeuses.GetSize() == 4) ;

      // 1st edgeuse Surface
      SmSurface * pSurf0 = sEdgeuses[0]->GetFace()->GetSurface() ;

      // 3rd edgeuse Surface = surface on opposite side of the edge
      SmSurface * pSurf1 = sEdgeuses[2]->GetFace()->GetSurface() ;

      // when edge is connected to 2 different surfaces - set faces connected to pSurfToKeep to use newly expanded Surface domain
      if(pSurf0 != pSurf1) // when pSurf0 == pSurf1, the call to DeleteEdge() will set pSurf->Face->UVDomain
        {
          SM_ASSERT(   pSurfToKeep == pSurf0
                    || pSurfToKeep == pSurf1) ;

// Remove Composites - moved block up from below
          SM_ASSERT_MSG(pSurfToKeep->GetFace() != NULL, _T("SmMerge::BooleanPostProcess - Remove Composite single face assumption wrong here - needs debug")) ;
          SmFace * pFace = pSurfToKeep->GetFace() ;

          // Update UVDomain
          SmExtent2d sNewDomain = pSurfToKeep->GetNaturalUVDomain();
          pFace->SetUVDomain( sNewDomain );

          // Update tolerance
          if (dDistBetween > pFace->GetTolerance()/10.0)
            {
#ifdef SM_USE_OLDTOL
              SM_OLDTOL_LINE pFace->SetTolerance(pFace->GetTolerance() + dDistBetween);
#endif // SM_USE_OLDTOL
            }

// Remove Composites
//           // locals
//           SmTArray<SmFace*> sAllFaces;
//           SmBrep::GetFacesOfSurface( pSurfToKeep, sAllFaces );  // when Surface->Owner is a Face : [Surface->Owner]
//                                                                 // when Surface->Owner is a CFace: [list of children Faces without CFace owner]
//           ULONG      lNumFaces  = sAllFaces.GetSize() ;
//           ULONG      lIters     = lNumFaces == 1 ? 1 : lNumFaces + 1 ;  // one extra iter for CompositeFaces
//           // Get the expanded domain from pSurfToKeep:
//           // SmEdge::IsTopological() expands pSurfToKeep to be big enough for both Faces.
//           SmExtent2d sNewDomain = pSurfToKeep->GetNaturalUVDomain();
// 
//           // for every Face using pSurfToKeep - update pFace->Domain and Tolerance
//           for(jj=0;jj<lIters;jj++)
//             {
//               // good for composite and standalone Faces
//               SmFace * pFace = (jj+1) == lIters
//                                ? SM_CAST_PTR( SmFace, pSurfToKeep->GetOwner()) // Owner on last iteration
//                                : sAllFaces[jj] ;                               // composite children on all but last iterations
// 
//               // Update UVDomain
//               pFace->SetUVDomain( sNewDomain );
// 
//               // Update tolerance
//               if (dDistBetween > pFace->GetTolerance()/10.0)
//                 {
// #ifdef SM_USE_OLDTOL
//                   SM_OLDTOL_LINE pFace->SetTolerance(pFace->GetTolerance() + dDistBetween);
// #endif // SM_USE_OLDTOL
//                 }
//             } // end updating all owning Faces

        } // end edge connects two different surfaces check

#ifdef SM_DEBUG_CODE
      if (bDebugMe)
        {
          SM_ASSERT_VALID(m_vTI.m_pBrep) ;

          SM_PTR_ARRAY(sFaces, SmFace, 32);
          pEdge->GetFaces( sFaces );
          sFaces[0]->GetSurface()->Dump();

          smgfx_Erase();
          smgfx_SetLook(1,2, 0,0,1); m_vTI.m_pBrep->Draw(TRUE); sm_GraphicsLoop();
          smgfx_SetLook(1,2, 0,1,0); if(sEdgeuses[0]->GetFace()) sEdgeuses[0]->GetFace()->GetSurface()->Draw(); sm_GraphicsLoop();
          smgfx_SetLook(1,2, 0,0,1); if(sEdgeuses[0]->GetFace()) sEdgeuses[0]->GetFace()->Draw(SM_DM_CROSSHATCH); sm_GraphicsLoop();
          smgfx_SetLook(3,4, 1,0,0); pEdge->Draw(); sm_GraphicsLoop();
          sm_GraphicsLoop();
        }
#endif // SM_DEBUG_CODE

      // If make it to here we have an edge with coincident surfaces or the same surface
      // on either side of it and it is not a seam edge.

      // Collect the edge's vertices into sEdgeVertices array.
      SmVertex *pV1 = pEdge->GetVertex();
      SmVertex *pV2 = pEdge->GetOtherVertex(pV1);
      sEdgeVertices.AddUnique(pV1);
      sEdgeVertices.AddUnique(pV2);

      // delete the edge - leave the vertices behind
      SER(m_vTI.m_pBrep->DeleteEdge(pEdge, pSurfToKeep));

      // Preserve the face-domain union across DeleteEdge's possible surface swap.
      pSurfToKeep->GetFace()->SetUVDomain(sMergedFaceDomain);

#ifdef SM_DEBUG_CODE
      if (bDebugMe)
        {
          SM_DUMP_AND_ASSERT_VALID(m_vTI.m_pBrep) ;

          smgfx_Erase();
          smgfx_SetLook(1,2, 0,0,1) ; if(m_vTI.m_pBrep) m_vTI.m_pBrep->Draw(TRUE); sm_GraphicsLoop() ;
          smgfx_SetLook(3,4, 1,0,0); pSurfToKeep->Draw(); sm_GraphicsLoop();
          SmObject *pObj = pSurfToKeep->GetOwner();
          SmFace *pFace = SM_CAST_PTR( SmFace, pObj );
          smgfx_SetLook(3,4, 1,0,0); if ( pFace ) pFace->Draw(SM_DM_CROSSHATCH); sm_GraphicsLoop();
          sm_GraphicsLoop();
        }
#endif // SM_DEBUG_CODE
    } // end iter every edge

  // Arrive here when done removing all edges.
  // Next, remove any orphan vertices which are topological
  //   (they separate a pair of edges which can be merged)
  // These are all of the vertices in the Intersection entity maps
  // plus those of the edges that we just removed.

  // Vertices to check
  SmTArray< SmVertex* > sVertices;
  sVertices = m_sIntersectionVertices;  // (Assignment operator)

  // Add in the vertices of the edges that we just removed.
  // (Although they're probably already in there.)
  for(ii=0; ii<sEdgeVertices.GetSize(); ii++)
    {
      sVertices.AddUnique( sEdgeVertices[ii] );
    }
  //ULONG lNumEdgeVerts = sVertices.GetSize() ;

#ifdef SM_DEBUG_CODE

  if (bDebugMe)
    {
      SM_DUMP_AND_ASSERT_VALID(m_vTI.m_pBrep) ;

      ULONG di ;
      ULONG lNumVerts1 = m_sIntersectionVertices.GetSize();
      ULONG lNumVerts2 = sEdgeVertices.GetSize();

      smgfx_Erase();
      smgfx_SetLook(1,2, 0,0,1) ; m_vTI.m_pBrep->Draw(TRUE); sm_GraphicsLoop() ;
      smgfx_SetLook(3,4, 0,1,0) ; // Careful, these can be stale, which is ok.
                                  for(di=0;di<lNumVerts1;di++)
                                    { m_sIntersectionVertices[di]->Draw(); sm_GraphicsLoop() ; }
      smgfx_SetLook(5,6, 1,0,0);  for(di=0;di<lNumVerts2;di++)
                                    { sEdgeVertices[di]->Draw(); sm_GraphicsLoop() ; }
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

  // Process each of the collected vertices.
  ULONG lNumVerts = sVertices.GetSize();
  for(ii=0;ii<lNumVerts;ii++)
    {
      SmVertex *pVertex = sVertices[ii];

      // pVertex may have been deleted.  If so, it will have been removed from the maps.
      if ( ! bCheckAllEdges )
        {
          SmObject *pOtherObj = m_vTI.GetOtherMate( pVertex );
          if ( pOtherObj == NULL )
            { continue; }
        }

      // There are still intermitent cases when deleted vertices get here
      if (pVertex->GetSize() > SM_BIG_ULONG) {
          continue;
      }

#ifdef SM_DEBUG_CODE
      if (bDebugMe)
        {
          SM_DUMP_AND_ASSERT_VALID(m_vTI.m_pBrep) ;

          smgfx_Erase();
          smgfx_SetLook(1,2, 0,0,1) ; m_vTI.m_pBrep->Draw(TRUE); sm_GraphicsLoop() ;
          smgfx_SetLook(6,8, 1,0,0); pVertex->Draw(); sm_GraphicsLoop();
          sm_GraphicsLoop();
        }
#endif // SM_DEBUG_CODE

      // delete LoopVertices (not connected to an Edge, connected to just one face)
      if(pVertex->IsLoopVertex())
        {
          SER(m_vTI.m_pBrep->DeleteVertex(pVertex));
          continue ;
        }

      // delete topological vertices
      if(pVertex->IsTopologicalVertex())
        {
#ifdef SM_DEBUG_CODE
          pVertex->GetEdges(sEdges) ;
          SM_ASSERT(sEdges.GetSize() == 2) ;
#endif // SM_DEBUG_CODE

          // Delete the topological vertex that splits the edges.
          // Not passing in an edge to delete will remove the shorter edge.
          if (m_vTI.m_pBrep->DeleteTopologicalVertex(pVertex) != SM_SUCCESS)
            {
              continue;  // Unable to combine edges;
            }
        } // end vertex IsTopological check
    } // end iter every vertex with an otherVertex mate

  // GWC: The following should be healer code - In a valid Brep,
  //      a FaceuseShell should contain all the wire geometry
  //      connected to that Shell and the following code
  //      should never be needed.  For now, lets see if
  //      this case ever comes up - if it does, then
  //      the following code should become a check in SmShell::AssertValid()
  //      and the action should become a Healer method.
  //
    { // begin Scope of code probably not needed
      // RMB July, 2004
      // Combine a wire shell with a faceuse shell
      // if they share a common vertex
      // For now we will only process the case when
      // there is one wire shell and one faceuse shell
      SmTArray<SmShell*> sShells;

      // when Brep has just 2 shells
      m_vTI.m_pBrep->GetShells(sShells);
      if (sShells.GetSize() == 2)
        {
          SmShell *pShell0 = sShells[0];
          SmShell *pShell1 = sShells[1];
          SmShell *pWShell = NULL; // the wire shell
          SmShell *pFShell = NULL; // the faceuse shell

          // check for a wireShell/FaceShell combination
          if (pShell0->IsWireShell())
            {
              pWShell = pShell0;
              if (pShell1->IsFaceuseShell()) pFShell = pShell1;
            }
          else if (pShell1->IsWireShell())
            {
              pWShell = pShell1;
              if (pShell0->IsFaceuseShell()) pFShell = pShell0;
            }

          // when 1 shell is a wire and the other is a FaceShell
          if(   pWShell != NULL
             && pFShell != NULL)
            {
              // If Brep and Other share a vertex then combine shells
              m_vTI.GetCommonVertices(m_vVertices,m_vOtherVertices);
              if (m_vVertices.GetSize() > 0)
                {
                  SM_DBG_WARN(_T("SmMerge::BooleanPostProcess found a 2 Shell Brep (a Sheet and a wire) that should be connected and aren't - review")) ;

                  // Transfer the wire shell to the face shell
                  SmRegion *pWR = pWShell->GetRegion();
                  pWShell->TransferChildrenTo(pFShell,NULL);
                  pWR->Remove(pWShell);
                  // delete pWShell; // protected.
                }
            } // end 2 shells are a wire and a face shell check
        } // end Brep has 2 shells case
    } // end scope of code probably not needed - see comment above

  return SM_SUCCESS;

} // end SmMerge::BooleanPostProcess

/*******************************************************************//**
PURPOSE: repeatedly apply binary merge operator to a list of Breps
                  until one final Brep is made as:
                                MergeOp
                                /    \
                            MergeOp  Brep[0]
                             .   \
                            .    Brep[1]
                          .
                        MergeOp
                        /    \
                   Brep[N] Brep[N-1]

NOTES: repeatedly replace last two Breps on the list
       with the result of MergeOperation(LastBrep, NextToLastBrep) until
           just one Brep remains on the list so that:

  Given  rBreps     = { ptr0, ptr1, ... ptrN-1, ptrN }
         loperation = (0 - union, 1 - intersection, 2 - difference, 3 - merge)

  Note: lOperation is a ULONG and not to be confused with SmBooleanOperationType

  Return ( MergeOp( ... MergeOp( MergeOp(ptrN, ptrN-1), ptrN-2) ..., ptr0)
***********************************************************************/
SmStatus SmMerge::merge_breps
  (SmTArray < SmBrep*> & rBreps,            // i/o: Two or more unique, non-NULL Breps; consumed after validation
   ULONG                 lOperation,        // in : 0 - union, 1 - intersection, 2 - difference, 3 - merge
   SmBrep             *& rpResult,          // out: resulting Brep; NULL on failure
   SmMergeOptions     * pOptMergeOptions )  // in : Optional boolean flags for SmMerge operation (default = NULL)
{
  // Validate the complete list before moving any entries into the Boolean
  // tree.  Besides making failure deterministic, this keeps rejected calls
  // non-consuming: callers retain the original array and every Brep in it.
  rpResult = NULL;
  const ULONG lCount = rBreps.GetSize();
  if (lCount < 2 || lOperation > 3)
    {
      SER(SM_ERR_INVALID_INPUT);
    }
  for (ULONG ii = 0; ii < lCount; ii++)
    {
      if (rBreps[ii] == NULL)
        {
          SER(SM_ERR_INVALID_INPUT);
        }
      for (ULONG jj = 0; jj < ii; jj++)
        {
          if (rBreps[ii] == rBreps[jj])
            {
              SER(SM_ERR_INVALID_INPUT);
            }
        }
    }

  // locals
  SmTArray < SmBrep*> sOrderedBreps;
  ULONG ii;
  SmTArray < long> sBoolTree;

  // Build the postfix tree in the longstanding last-to-first order while
  // draining the caller's array as required by this consuming operation.
  sBoolTree.Add(0);                    // index of first Brep
  sOrderedBreps.Add(rBreps.GetLast()); // pointer to first Brep
  rBreps.RemoveLast();

  for(ii = 1; ii < lCount; ii++)
    {
      sOrderedBreps.Add(rBreps.GetLast());
      rBreps.RemoveLast();
      sBoolTree.Add(ii);
      sBoolTree.Add(-(static_cast<long>(lOperation) + 1)); // Convert to different form of Boolean numbering
    }

  // pass the call along
  SER(SmMerge::BooleanTrees(sOrderedBreps, sBoolTree, pOptMergeOptions));
  if (sOrderedBreps.GetSize() != 1 || sOrderedBreps[0] == NULL)
      SER(SM_ERR); // Something went wrong

  // all done
  rpResult = sOrderedBreps[0];
  return SM_SUCCESS;

} // end SmMerge::merge_breps

/******************************************************************
PURPOSE:  Boolean between lists of Geometry breps or surfaces

USER NOTES --- This method allows for a boolean operation between two lists.
            These lists can be breps or surfaces.
            The output may be split into a list of breps,
                 when there are separate solids.
METHOD ---
  1. Build a pBrep model:
        When given pBreps1, combine them into a single pBrep
        when not given pBreps1, turn all pSurfaces1 (expected to lie
          in a single 2d-plane) into faces and combine those into a single pBrep.
  2. Build a pOther model:
        When given pBreps2, combine them into a single pOther
        when not given pBreps2, turn all pSurfaces2 (expected to lie
          in a single 2d-plane) into faces and combine those into a single pOther.
  3. Run the Merge
        For 3d breps call SmMergeManifoldBoolean()
        For 2d breps call SmPrimitiveCreation::Boolean2D()
  4. set the output
        for 3d Breps - turn every infinite region into its own Brep
                       and load those into the output
        for 2d Breps - turn every face into its own surface and load those
                       into the output
  5. all done - return success

  increments an unlocked mark value
******************************************************************/
SmStatus SmMerge::BooleanLists
  (SmContext                   * pContext,          // in : Output Context
   SmTArray <SmBrep *>   const & pBreps1,           // in : List of input breps must not be modified
   SmTArray <SmSurface*> const & pSurfaces1,        // in : List of input surfaces. All surfaces should be SmPlanes for planar2d
   SmTArray <SmBrep *>   const & pBreps2,           // in : List of input breps. input breps must not be modified
   SmTArray <SmSurface*> const & pSurfaces2,        // in : List of input surfaces.  All surfaces should be SmPlanes for planar2d
   int                           operation,         // in : 0,1,2,3,4 = AND, OR, XOR, AND NOT, NOT AND
                                                    //      0,1,2,3,4 = intersect,Union,Xor,A-B, B-A
   SmTArray <SmBrep *>         & pBreps3,           // out: Ouput if breps or non-planar surfs
   SmTArray <SmSurface*>       & pSurfaces3,        // out: Output surfaces if all inputs 2d coplanar
   SmMergeOptions              * pOptMergeOptions ) // in: Optional boolean flags for SmMerge operation (default = NULL)

{
  // locals
  SmBrep    *pBrep1      = NULL ;
  SmBrep    *pBrep2      = NULL;
  SmBrep    *pBrep3      = NULL;
  SmBoolean Both2DPlanar = FALSE;
  SmBoolean Planar1      = FALSE ;
  SmBoolean Planar2      = FALSE;

  // init output
  pBreps3.ReSet();
  pSurfaces3.ReSet();

  //------------------------------------ brep 1 -------------------------
  // when input pBreps1 is not empty - copy pBreps1[0] into pBrep1 using pContext and
  //                                   Combine (copy and insert without intersection all topology objects)
  //                                   any subsequent pBreps1[i] topology data into pBrep1
  // When pBreps1 is empty and pSurfaces1 is not - create new pBrep1 and make a face in pBrep1 for every
  //                                   surface in pSurfaces1 using pContext
  ULONG ii, num = pBreps1.GetSize();
  if (num > 0)
    {
      // copy 1st input brep using pContext
      if (num >= 1)
        {
          SmBrep *pOrig = pBreps1[0];
          pBrep1 = new(*pContext) SmBrep(*pOrig);
        }

      // CombineOnly all other breps into the copy Brep using the output Context
      if (num > 1)
        {
          for(ii = 1; ii < num; ii++)
            {
              // todo: need to check the Context pointers in both the new copies and original topology objects
              pBrep1->MergeBrep(*pBreps1[ii]);
            }
        }
//        SER(pBrep1->ShrinkGeometry());
    }
  else  // no brep, try surface
    {
      num = pSurfaces1.GetSize();
      if (num < 1)
          return (SM_ERR); // no brep OR surface
      pBrep1 = new(*pContext) SmBrep();
      // NOTE: using default tolerance SM_ZONE_TOL_3D

      // surfaces, make into single brep with multiple faces.
      for(ii = 0; ii < num; ii++)
        {
          SmSurface *pOrig = pSurfaces1[ii];
          SmSurface *pSurfaceCopy = new(*pContext) SmSurface(*pOrig);
          SmFace *pFace = NULL;
          SER(pBrep1->CreateFaceFromSurface(pSurfaceCopy, pSurfaceCopy->GetNaturalUVDomain(), pFace));
          Planar1 = pSurfaceCopy->IsKindOf(SmPlane_TYPE); // we hope they are all the same
        }
    }

  // we are working with copies or merged copies
  // this is essential
  SER(pBrep1->StitchAndOrient());
//    pBrep1->TurnToNURBS();
//    pBrep1->ValidatePointers();
  SmObjDelete sClean1(pBrep1);


  //------------------------------------ brep 2 -------------------------
  // when input pBreps2 is not empty - copy pBreps2[0] into pBrep2 using pContext and
  //                                   Combine (copy and insert without intersection all topology objects)
  //                                   any subsequent pBreps2[i] topology data into pBrep2
  // When pBreps2 is empty and pSurfaces2 is not - create new pBrep2 and make a face in pBrep2 for every
  //                                   surface in pSurfaces2 using pContext

  num = pBreps2.GetSize();
  if (num > 0)
  {
      // copy 1st Brep using pContext
      if (num >= 1)
      {
          SmBrep *pOrig = pBreps2[0];
          pBrep2 = new(*pContext) SmBrep(*pOrig);
      }

      // CombineOnly all other breps into the copy Brep using the output Context
      if (num > 1)
        {
          for(ii = 1; ii < num; ii++)
            {
              // todo: need to check the Context pointers in both the new copies and original topology objects
              pBrep2->MergeBrep(*pBreps2[ii]);
            }
        }
//        SER(pBrep2->ShrinkGeometry());
  }
  else  // no brep, try surface
  {
      num = pSurfaces2.GetSize();
      if (num < 1)
          return (SM_ERR); // no brep OR surface
      pBrep2 = new(*pContext) SmBrep();
      // NOTE: using default tolerance SM_ZONE_TOL_3D

      // surfaces, make into single brep with multiple faces.
      for(ii = 0; ii < num; ii++)
      {
          SmSurface *pOrig = pSurfaces2[ii];
          SmSurface *pSurfaceCopy = new(*pContext) SmSurface(*pOrig);
          SmFace *pFace = NULL;
          SER(pBrep2->CreateFaceFromSurface(pSurfaceCopy, pSurfaceCopy->GetNaturalUVDomain(), pFace));
          Planar2 = pSurfaceCopy->IsKindOf(SmPlane_TYPE); // we hope they are all the same
      }
  }

  // This is essential
  SER(pBrep2->StitchAndOrient());
//  pBrep2->TurnToNURBS();
//    pBrep2->ValidatePointers();
  SmObjDelete sClean2(pBrep2);

  // arrive here after all inputs have been copied under pContext
  // so that we are now working with just two Breps both sharing the same
  // pContext pointer.

  //------------------------------------ boolean -------------------------

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
  if (bDebugMe)
{
  pBrep1->WriteToFile(_T("./MergeBrep1.smb"), SM_ASCII, TRUE, FALSE, 0.0);
  pBrep2->WriteToFile(_T("./MergeBrep2.smb"), SM_ASCII, TRUE, FALSE, 0.0);
}
#endif // SM_DEBUG_CODE

  //double angleTol = 20.0*SM_PI/180.0;
  //double dTol =  smos_Max(pBrep1->GetTolerance(),pBrep2->GetTolerance());
  //double dBooleanTol = dTol*10.0;
  // Let's allow SmMerge to calculate tolerances
  double angleTol = 0.0;
  double dBooleanTol = 0.0;

  //SmStatus status = SM_SUCCESS;
  if (Planar1 && Planar2)
      Both2DPlanar = TRUE;

  // logical AND = INTERSECTION;
  if (operation == 0)
    {
      if (!Both2DPlanar)
        {
          SmMerge sMerge(*pContext, pBrep1, pBrep2, dBooleanTol, angleTol, pOptMergeOptions );
          SER(sMerge.NonManifoldBoolean(SM_BO_INTERSECTION, pBrep3));
        }
      else // planar surfaces 2d
        {
          SER(SmPrimitiveCreation::Boolean2D(pBrep1, pBrep2, SM_2D_INTERSECTION, pBrep3));  // note: increments unlocked mark value
        }
    }

  // logical OR  = UNION
  else if (operation == 1)
    {
      if (!Both2DPlanar)
        {
          SmMerge sMerge(*pContext, pBrep1, pBrep2, dBooleanTol, angleTol, pOptMergeOptions );
          SER(sMerge.NonManifoldBoolean(SM_BO_UNION, pBrep3));
        }
      else // planar surfaces 2d
        {
          SER(SmPrimitiveCreation::Boolean2D(pBrep1, pBrep2, SM_2D_UNION, pBrep3)); // note: increments unlocked mark value
        }
    }

  // logical exclusive'or'  XOR  = (A U B) - (A ^ B)
  else if (operation == 2)
    {
      if (!Both2DPlanar)
        {
          SmMerge sMerge(*pContext, pBrep1, pBrep2, dBooleanTol, angleTol, pOptMergeOptions );
          SER(sMerge.NonManifoldBoolean(SM_BO_EXCLUSIVE_OR, pBrep3));

        }
      else
        {
          SER(SmPrimitiveCreation::Boolean2D(pBrep1, pBrep2, SM_2D_EXCLUSIVE_OR, pBrep3));  // note: increments unlocked mark value
        }
    }

  //  AND NOT = DIFFERENCE  = A - (A ^ B)   = A - B
  else if (operation == 3)
    {
      if (!Both2DPlanar)
        {
          SmMerge sMerge(*pContext, pBrep1, pBrep2, dBooleanTol, angleTol, pOptMergeOptions );
          SER(sMerge.NonManifoldBoolean(SM_BO_DIFFERENCE, pBrep3));
        }
      else
        {
          SER(SmPrimitiveCreation::Boolean2D(pBrep1, pBrep2, SM_2D_DIFFERENCE, pBrep3));  // note: increments unlocked mark value
        }
    }

  // NOT AND  = DIFFERENCE  = B - (A ^ B) = B - A
  else if (operation == 4)
    {
      if (!Both2DPlanar)
        {
          SmMerge sMerge(*pContext, pBrep2, pBrep1, dBooleanTol, angleTol, pOptMergeOptions );
          SER(sMerge.NonManifoldBoolean(SM_BO_DIFFERENCE, pBrep3));
        }
      else
        {
          SER(SmPrimitiveCreation::Boolean2D(pBrep2, pBrep1, SM_2D_DIFFERENCE, pBrep3));  // note: increments unlocked mark value
        }
    }

  // else operation is an unsupported value
  else
    {
      return (SM_ERR_INVALID_INPUT);  // no other operation possible
    }

  // make the output surfaces as small as possible

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe2 = FALSE;
  if (bDebugMe2)
{
  pBrep3->WriteToFile(_T("Brep1m2.smb"), SM_ASCII, TRUE, FALSE, 0.0);
}
#endif // SM_DEBUG_CODE

  // now that the boolean worked we don't have to clean up inputs.
  sClean1.Clear();
  sClean2.Clear();
  SER(pBrep3->ShrinkGeometry());
  SER(pBrep3->StitchAndOrient());
//    pBrep3->ValidatePointers();

  if (Both2DPlanar)
    {
      // copy just the surfaces out for 2D surfaces boolean.
      SmTArray <SmSurface *> sSrfs;
      pBrep3->GetSurfaces(sSrfs);
      num = sSrfs.GetSize();
      for( ii=0; ii<num; ii++)
        {
          SmSurface *pSrf = sSrfs[ii];
          SmSurface *pSrfCopy = new(*pContext) SmSurface (*pSrf);
          pSurfaces3.Add(pSrfCopy);
        }
      delete pBrep3; pBrep3 = NULL;
    }
  else  // output multiple breps for each shell in infinite region
        // may not work for nested shells
    {
      SmRegion *pRegion = pBrep3->GetInfiniteRegion();
      SmTArray < SmShell *> sShells;
      pRegion->GetShells(sShells);
      num = sShells.GetSize();
      if (num == 0)
          return (SM_ERR);

      // only 1 shell = only 1 output brep
      if (num == 1)
          pBreps3.Add(pBrep3);

      else // more than 1 shell; make 1 brep for each connected set of faces
        {
          SmTArray <SmFace *> sFaces;
          pBrep3->GetFaces(sFaces);

          while( sFaces.GetSize() > 0 )
            {
             // increment and lock an unlocked mark
             SmNewMarkAndLock sMarkLock( pBrep3->GetContext(), SM_MT_ALLMARKS) ; // increment and lock any unlocked mark
             SmMarkType eMarkType = sMarkLock.GetMarkType() ;

             SmTArray <SmFace *> sFacesToCopy;
             SmTopologyTraverser sTraverser;
             SER(sTraverser.CollectFaces(sFaces[0],     // in : Seed face (gets marked)
                                         sFacesToCopy,  // out: List of connected faces (Get marked)
                                         eMarkType));   // in : specify mark for target objects (not incremented)

             // if we are to use all remaining faces, then simply add brep
             // to list of resulting breps (no need to copy and delete)
             if( sFacesToCopy.GetSize() == sFaces.GetSize() )
             {
                 pBreps3.Add(pBrep3);
                 break;
             }
             else
             {
                 SmBrep *pBrep4 = new(*pContext) SmBrep;
                 SER(pBrep3->CopyFaces(sFacesToCopy, pBrep4 ));
                 pBrep4->StitchAndOrient();
                 pBreps3.Add(pBrep4);

                 pBrep3->m_bEditingEnabled = TRUE;
                 pBrep3->RemoveFaces(sFacesToCopy);  // increments unlocked mark value

                 sFaces.ReSet();
                 pBrep3->GetFaces(sFaces);
             }

             // note we have to test which breps are inside each other
            }
        }
    } // end Both2DPlanar == FALSE branch - output 1 Brep for every connected set of faces

  return (SM_SUCCESS);

} // end SmMerge::BooleanLists

/*******************************************************************//**
PURPOSE: Perform a set of Boolean operations using a post fix notation
     of a CSG tree.

       A boolean Tree is a CSG tree of boolean operations
       and leaf node Brep objects that specify how to combine the leaf nodes
       into a single Brep model.  This tree is represented as a linear
       list of boolean operation tokens and Brep indices in post fix notation.
       Boolean Tree token values are oneof
         -1                     = Union
         -2                     = Intersect
         -3                     = Subtract
         -4                     = Merge
         -5                     = Exclusive or
          0 and Positive values = Brep leaf node object indices

       Post Fix Notation is read from right to left.

Example:
       crBooleanTreeNodes = { 0 1 -1 2 -3 3 4 -4 -2 }
       represents the CSG Tree:

                            INTERSECT
                          /           \
                   SUBTRACT            MERGE
                   /     \           /       \
                UNION rBreps[2]  rBreps[3]  rBreps[4]
                /    \
          rBreps[0]  rBreps[1]

       Where 0-4 are indices into rBreps array of Breps.


NOTES:
    1. On output, the breps array will be appended at the beginning
       with the new breps in the order that they were created followed
       by any original breps that were not used in the CSG tree.

    2. All breps used in the Brep tree will cease to exist.
       A Brep can only be used one time in the TREE.  If you need to use
       it more than one time you must copy it prior to this method.

    3. The post fix tree will reinitialized to reflect the successful
        Booleans.

    4. One Brep model will be produced for each CSG tree specified
       in the rPostFixTrees array.  It is possible to specify any
       number of trees in the rPostFixTrees array.
       Example:  { 0 1 -1 2 3 -1 } is 2 trees as

           UNION               UNION
          /     \             /     \
        Brep[0]  Brep[1]    Brep[2]  Brep[3]

    5. Operations currently supported are:
         SM_BO_UNION
         SM_BO_INTERSECTION
         SM_BO_DIFFERENCE
         SM_BO_MERGE
         SM_BO_EXCLUSIVE_OR
         SM_BO_SLICE
***********************************************************************/
SmStatus SmMerge::BooleanTrees
  (SmTArray<SmBrep*> & rBreps,         // i/o: in  - array of Breps to be combined
                                       //      out - Tree combination result plus any
                                       //            unused Breps appended to the end
   SmTArray<long> & rPostFixTrees,     // in : tree specifying how to combine rBreps
   SmMergeOptions * pOptMergeOptions ) // in: Optional boolean flags for SmMerge operation (default = NULL)
{
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
  if (bDebugMe)
    {
      static ULONG lWriteCount = 1;
      TCHAR sBuff[SM_TBLOCK_SIZE];
      if (lWriteCount < 10) { smos_sprintf(sBuff,_T("BoolTree_%1ld.smp"),lWriteCount); }
      else if (lWriteCount < 100) {smos_sprintf(sBuff,_T("BoolTree_%2ld.smp"),lWriteCount); }
      else if (lWriteCount < 1000) {smos_sprintf(sBuff,_T("BoolTree_%3ld.smp"),lWriteCount); }
      SmTArray<SmCurve*> sCurves;
      SmTArray<SmSurface*> sSurfaces;
      SER(SmBrepData::WritePartToFile(sBuff,sCurves,sSurfaces,rPostFixTrees,rBreps,SM_ASCII,TRUE,FALSE,SM_ZONE_TOL_3D));
    }
#endif // SM_DEBUG_CODE

  SmTArray<SmBrep*> sStack;

  // for every PostFixTrees entry
  ULONG ii, lNumTrees = rPostFixTrees.GetSize();
  for(ii=0; ii<lNumTrees; ii++)
    {
      long lNode = rPostFixTrees[ii];

      // Get Brep specified by index lNode >= 0
      if (lNode >=0)
        {
          // retrieve Brep pointer and clear rBreps entry
          SmBrep *pBrep = rBreps[lNode];
          rBreps[lNode] = NULL;
          sStack.Add(pBrep);

#ifdef SM_DEBUG_CODE
          if (bDebugMe)
            {
              TCHAR sBuff[SM_TBLOCK_SIZE];
              smos_sprintf(sBuff,_T("\nBrep[%ld] = # %ld"),ii,lNode);
              smos_WriteBuffer(sBuff);
            }
#endif // SM_DEBUG_CODE

        } // end node is a Brep branch
      else // node is an operation branch
        {
          // check state - problem with tree - its not binary
          if (sStack.GetSize() < 2)
            { SER(SM_ERR);
            }

          // get Breps from the stack
          SmBrep *pBrep2 = sStack.GetLast();
          sStack.RemoveLast();
          SmBrep *pBrep1 = sStack.GetLast();
          sStack.RemoveLast();

          // Get tolerance values.
          //double dTol      = pBrep1->GetTolerance() + pBrep2->GetTolerance(); // Max is too tight [B396]
          //double dAngleTol = SM_DEG2RAD(20);
          // Let's allow SmMerge to calculate tolerances
          double dTol = 0.0;
          double dAngleTol = 0.0;

          // boolean operation locals
          SmMerge sMerge(*pBrep1->GetContext(),pBrep1,pBrep2,dTol,dAngleTol, pOptMergeOptions );
          SmBrep *pResult = NULL;
          SmBoolean bBadBoolean = FALSE;

#ifdef SM_DEBUG_CODE
          if (bDebugMe)
            {
              TCHAR sBuff[SM_TBLOCK_SIZE];
              smos_sprintf(sBuff,_T("\nStarting Boolean Node # %ld"),ii);
              smos_WriteBuffer(sBuff);
            }

static constexpr ULONG lBooleanDebug = SM_BIG_ULONG;
SmBoolean bDebugMe1 = FALSE;
SmBoolean bDebugMeA = FALSE ;
          if (bDebugMe1 || ii == lBooleanDebug)
            {
              TCHAR sBuff[SM_TBLOCK_SIZE];
              smos_sprintf(sBuff,_T("%s"),_T("BooleanBug.smp"));
              SmTArray<SmCurve*> sCurves;
              SmTArray<SmSurface*> sSurfaces;
              SmTArray<SmBrep*> sBreps;
              SmTArray<SmAttribute*> sAttributes;
              sBreps.Add(pBrep1);
              sBreps.Add(pBrep2);
              SmTArray<long> sTree;
              sTree.Add(0);
              sTree.Add(1);
              sTree.Add(lNode);
              SER(SmBrepData::WritePartToFile(sBuff,sCurves,sSurfaces,sTree,sBreps,SM_BINARY,TRUE,FALSE,SM_ZONE_TOL_3D));
              if (bDebugMeA) {
                  smos_sprintf(sBuff,_T("%s"), _T("BooleanTreeDebug.smp"));
                  ULONG jjj;
                  for(jjj=ii+1; jjj<rPostFixTrees.GetSize(); jjj++) {
                      long lId = rPostFixTrees[jjj];
                      if (lId < 0) {
                          sTree.Add(lId);
                      }
                      else {
                          SmBrep *pBrep = rBreps[lId];
                          sTree.Add(sBreps.GetSize());
                          sBreps.Add(pBrep);
                      }
                  }
                  SER(SmBrepData::WritePartToFile(sBuff,sCurves,sSurfaces,sTree,sBreps,SM_BINARY,TRUE,FALSE,SM_ZONE_TOL_3D));
              }
            }
#endif // SM_DEBUG_CODE

static constexpr SmBoolean sbTestIO = FALSE;
          if (sbTestIO)
            {
              SER(pBrep2->ValidatePointers());
              SER(pBrep1->ValidatePointers());
              SER(pBrep2->WriteToFile(_T("Brep2.smb"), SM_ASCII, TRUE, FALSE, 0.0));
              SER(pBrep1->WriteToFile(_T("Brep1.smb"), SM_ASCII, TRUE, FALSE, 0.0));
            }

          // select the operation
          SmBooleanOperationType eOperation =   (lNode == -1) ? SM_BO_UNION
                                              : (lNode == -2) ? SM_BO_INTERSECTION
                                              : (lNode == -3) ? SM_BO_DIFFERENCE
                                              : (lNode == -4) ? SM_BO_MERGE
                                              : (lNode == -5) ? SM_BO_EXCLUSIVE_OR
                                              : (lNode == -6) ? SM_BO_SLICE
                                              : SM_BO_UNKNOWN ;
          if(eOperation == SM_BO_UNKNOWN) SER(SM_ERR) ;

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMeB = FALSE ;
           // draw input Breps (blue and green)

           if(bDebugMeB)
             {
               sMerge.GetTopologyIntersector().GetPrimaryBrep()->Dump() ;
               sMerge.GetTopologyIntersector().GetOtherBrep()->Dump() ;

               //SmDrawModeType eDMType     = smgfx_GetDrawingMode() ;
               //SmBoolean      bShadedMode = smgfx_GetShadedMode() ;
               // smgfx_SetDrawingMode(SM_DM_FACETS_NLIB) ;
               // smgfx_SetShadedMode(TRUE) ;

               smgfx_Erase() ;
               smgfx_SetLook( 1, 2, 0,0,1 );
               sMerge.GetTopologyIntersector().GetPrimaryBrep()->Draw(TRUE) ; sm_GraphicsLoop() ;
               smgfx_SetLook(3,5, 0,1,0) ;
               sMerge.GetTopologyIntersector().GetOtherBrep()->Draw(TRUE) ; sm_GraphicsLoop() ;
               sm_GraphicsLoop() ;
             }
#endif // SM_DEBUG_CODE

          // execute the boolean operation
          if (sMerge.ManifoldBoolean(eOperation, pResult) != SM_SUCCESS)
            {
              bBadBoolean = TRUE;
            }

          // Quit when things fail (before putting result on stack: [090913]).
          if (bBadBoolean)
            {
              return ( SM_ERR);
              //break;
            }

          // place result on the stack
          sStack.Add(pResult);

#ifdef SM_DEBUG_CODE
          if (bDebugMe1)
            {
             pResult->Dump();
            }

          // draw pResult in cyan
          if(bDebugMeB)
            {
              smgfx_SetLook(2,4, 0,1,1) ; pResult->Draw(TRUE) ; sm_GraphicsLoop() ;
              sm_GraphicsLoop() ;
            }

static constexpr SmBoolean sbValidate = FALSE;
          int iBreakPointHere=0;
          if ( sbValidate ) {
            if ( ! pResult->AssertValid() )
              { iBreakPointHere++; }
          }

static constexpr SmBoolean sbManifold = FALSE;
          if ( sbManifold )
          {
            if ( ! pBrep1->IsManifoldSolid() )
              { iBreakPointHere++; }
          }
#endif // SM_DEBUG_CODE

          // when testing I/O
          if (sbTestIO)
            {
              SmTArray<SmVertex*> sVertices;
              pResult->GetVertices(sVertices);
              SmTArray<SmEdge*> sEdges;
              pResult->GetEdges(sEdges);
// Remove Composites
//              SmTArray<SmCFace*> sCFaces;
//              pResult->GetCFaces(sCFaces);
//              SmTArray<SmCEdge*> sCEdges;
//              pResult->GetCEdges(sCEdges);

              // locals for upcoming merge
              SmContext sContext;
              SmBrep *pIOResult = NULL;

              // duplicated Brep1 and Brep2 to be merged into pIOResult
              SmBrep *pBrep11 = new (sContext) SmBrep();
              SmBrep *pBrep22 = new (sContext) SmBrep();

              SER(pBrep11->ReadFromFile(sContext,_T("Brep1.brp"), SM_ASCII, FALSE));
              SER(pBrep22->ReadFromFile(sContext,_T("Brep2.brp"), SM_ASCII, FALSE));
#ifdef SM_DEBUG_CODE
              if (bDebugMe1) {
                  pBrep11->Dump();
                  pBrep22->Dump();
              }
#endif // SM_DEBUG_CODE
              // reexecute the operation on the read objects
              SmMerge sIOMerge(sContext,pBrep11,pBrep22,dTol,dAngleTol, pOptMergeOptions );
              SER(sIOMerge.ManifoldBoolean(eOperation, pIOResult));

              SmObjDelete sClean(pIOResult) ;

              // verify that original and written/read brep results are the same
              SmTArray<SmVertex*> sVertices2;
              pIOResult->GetVertices(sVertices2);
              SM_ASSERT(sVertices2.GetSize() == sVertices.GetSize());

              SmTArray<SmEdge*> sEdges2;
              pIOResult->GetEdges(sEdges2);
              SM_ASSERT(sEdges2.GetSize() == sEdges.GetSize());

// Remove Composites
//              SmTArray<SmCFace*> sCFaces2;
//              pIOResult->GetCFaces(sCFaces2);
//              SM_ASSERT(sCFaces2.GetSize() == sCFaces.GetSize());

// Remove Composites
//              SmTArray<SmCEdge*> sCEdges2;
//              pIOResult->GetCEdges(sCEdges2);
//              SM_ASSERT(sCEdges2.GetSize() == sCEdges.GetSize());

#ifdef SM_DEBUG_CODE
              if (bDebugMe1) {
                  pResult->Dump();
                  pIOResult->Dump();

                  smgfx_Erase();
                  smgfx_SetLook(1,2, 1,0,0); pResult  ->Draw(); sm_GraphicsLoop();
                  smgfx_SetLook(1,2, 0,0,1); pIOResult->Draw(); sm_GraphicsLoop();
                  sm_GraphicsLoop();
              }
#endif // SM_DEBUG_CODE
            }  //end I/O Test
        } // end PostFixTree Node is an operation branch
    } // end iter every PostFixTree Node

  // place unused Breps into rStack
  ULONG jj, lNumBreps = rBreps.GetSize();
  for(jj=0; jj<lNumBreps; jj++)
    {
      if (rBreps[jj] != NULL)
        {
          sStack.Add(rBreps[jj]);
        }
    }

  // init and set output
  rBreps.ReSet();
  rBreps.Append(sStack);

  return SM_SUCCESS;

} // end SmMerge::BooleanTrees

/*******************************************************************//**
PURPOSE: This method fixes up the case where we have found a face
    coincidence in the post processing stage of the Merge.  It will check
    for the case where all surrounding edges are related.  If they are
    it will just assume that the faces are coincident.  If not then
    it will try to merge coincident surfaces then do the test again.

NOTES: All the edges connected to a pair of coincident faces
  from two different breps must also be coincident to merge the faces.

  This function detects the case where the boundaries of two faces
  known to be coincident are not the same.  It fixes this by projecting
  the edges of each face into the other Brep.  Such projections may
  insert new edges and split faces.  After the projections are complete,
  the original pair of coincident faces will share a set of coincident
  edges.

***********************************************************************/
SmStatus SmMerge::FixFoundFaceCoincidence
  (SmFace *pBrepFace,                  // in : Face from Brep coincident with
   SmFace *pOtherFace,                 // in : Face from other Brep
   SmTArray<SmFace*> & rBrepFaces,     // out: faces in rBrep known to be the same as
   SmTArray<SmFace*> & rOtherFaces,    // out: faces in rOther
   SmBoolean & rbModifiedTopology)     // out: TRUE=topology graphs were modified
{
  // init return values
  rbModifiedTopology = FALSE;
  rBrepFaces.ReSet();
  rOtherFaces.ReSet();

  // locals
  SmBrep *pBrep = pBrepFace->GetBrep();
  SmBrep *pOther = pOtherFace->GetBrep();

  // get all edges attached to coincident faces
  SmTArray<SmEdge*> sBrepFaceEdges;
  SmTArray<SmEdge*> sOtherFaceEdges;
  pBrepFace->GetEdges(sBrepFaceEdges);
  pOtherFace->GetEdges(sOtherFaceEdges);

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
  if (bDebugMe)
    {
      m_vTI.Dump() ;
      sBrepFaceEdges.Dump() ;
      sOtherFaceEdges.Dump() ;

      // draw breps, faces and edges
      ULONG ii ;
      smgfx_Erase();
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,0) ; if(pOther) pOther->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,1) ; pBrepFace->Draw(SM_DM_CROSSHATCH, 8, 8); sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 1,1,0) ; pOtherFace->Draw(SM_DM_CROSSHATCH, 8, 8); sm_GraphicsLoop() ;
      smgfx_SetLook(3,4, 1,0,0) ; for(ii=0;ii<sBrepFaceEdges.GetSize();ii++)
                                   { if(sBrepFaceEdges[ii]) sBrepFaceEdges[ii]->Draw(); sm_GraphicsLoop() ; }
      smgfx_SetLook(5,6, 0,.5,1) ;for(ii=0;ii<sOtherFaceEdges.GetSize();ii++)
                                   { if(sOtherFaceEdges[ii]) sOtherFaceEdges[ii]->Draw(); sm_GraphicsLoop() ; }
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

  // detect if any edges of pBrepFace don't match edges of pOtherFace
  SmBoolean bMisMatch = FALSE;

  // when edge counts are the same
  if (sBrepFaceEdges.GetSize() == sOtherFaceEdges.GetSize())
    {
      // for every pBrepface Edge
      ULONG i, lNumEdges = sBrepFaceEdges.GetSize();
      for(i=0; i<lNumEdges; i++)
        {
          // any pBrepFace->Edge has no matching pOtherFace->edge
          if (m_vTI.GetOtherMate(sBrepFaceEdges[i]) == NULL) {
              // there is a mismatch
              bMisMatch = TRUE;
              break;
          }

          // any pBrepFace->Edge has no matching pOtherFace->edge
          if (m_vTI.GetBrepMate(sOtherFaceEdges[i]) == NULL) {
              // there is a mismatch
              bMisMatch = TRUE;
              break;
          }
        } // end ite every edge

      // When all the edges match - assume faces are the same
      if (!bMisMatch)
        {
          // set up the return lists and exit
          rBrepFaces.Add(pBrepFace);
          rOtherFaces.Add(pOtherFace);
          return SM_SUCCESS;
        } // end all face->edges match check

#ifdef SM_DEBUG_CODE
      if (bDebugMe)
        {
          m_vTI.Dump() ;
          sBrepFaceEdges.Dump() ;
          sOtherFaceEdges.Dump() ;

          // draw breps, faces, edges, and edge mates
          ULONG ii ;
          smgfx_Erase();
          smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
          smgfx_SetLook(1,2, 0,1,0) ; if(pOther) pOther->Draw(TRUE) ; sm_GraphicsLoop() ;
          smgfx_SetLook(1,2, 0,1,1) ; pBrepFace->Draw(SM_DM_CROSSHATCH, 8, 8); sm_GraphicsLoop() ;
          smgfx_SetLook(1,2, 1,1,0) ; pOtherFace->Draw(SM_DM_CROSSHATCH, 8, 8); sm_GraphicsLoop() ;
          for(ii=0;ii<sBrepFaceEdges.GetSize();ii++)
                                       { SmEdge *pEdge = sBrepFaceEdges[ii] ;
                                         SmObject *pOtherObject = pEdge ? m_vTI.GetOtherMate(pEdge) : NULL ;
                                         SmEdge *pOtherEdge = (pOtherObject && pOtherObject->IsKindOf(SmEdge_TYPE)) ? (SmEdge *)pOtherObject : NULL ;
                                         smgfx_SetLook(3,4, 1,0,0) ; if(pEdge) pEdge->Draw() ; sm_GraphicsLoop() ;
                                         smgfx_SetLook(5,6, 0,.5,1) ; if(pOtherEdge) pOtherEdge->Draw() ; sm_GraphicsLoop() ;
                                       }
          for(ii=0;ii<sOtherFaceEdges.GetSize();ii++)
                                       { SmEdge *pOtherEdge = sOtherFaceEdges[ii] ;
                                         SmObject *pObject = pOtherEdge ? m_vTI.GetBrepMate(pOtherEdge) : NULL ;
                                         SmEdge *pEdge = (pObject && pObject->IsKindOf(SmEdge_TYPE)) ? (SmEdge *)pObject : NULL ;
                                         smgfx_SetLook(7,8, 0,.5,1) ; if(pOtherEdge) pOtherEdge->Draw() ; sm_GraphicsLoop() ;
                                         smgfx_SetLook(9,10, 1,0,0) ; if(pEdge) pEdge->Draw() ; sm_GraphicsLoop() ;
                                       }
          sm_GraphicsLoop();
        }
#endif // SM_DEBUG_CODE
    } // end same edgecount check

  // get all pBrep and pOther edges
  SmTArray<SmEdge*> sBrepEdges;
  SmTArray<SmEdge*> sOtherEdges;
  pBrep->GetEdges(sBrepEdges);
  pOther->GetEdges(sOtherEdges);

  // merge coincident surfaces
  //   make sure all edges of the coincident surface pair are
  //   also coincident by projecting the edges of each surface onto the other.
  SER(m_vTI.MergeCoincidentSurfaces(pBrepFace->GetSurface(),
                                    pOtherFace->GetSurface(),
                                    rBrepFaces,
                                    rOtherFaces));

  // check new pBrep and pOther total edge counts
  // to determine if new topology has been added
  SmTArray<SmEdge*> sBrepEdges2;
  SmTArray<SmEdge*> sOtherEdges2;
  pBrep->GetEdges(sBrepEdges2);
  pOther->GetEdges(sOtherEdges2);

  // when edge counts vary - remember that topology has been modified
  if (sBrepEdges2.GetSize() != sBrepEdges.GetSize())
    {
      rbModifiedTopology = TRUE;
    }
  if (sOtherEdges2.GetSize() != sOtherEdges.GetSize())
    {
      rbModifiedTopology = TRUE;
    }

  return SM_SUCCESS;

} // end SmMerge::FixFoundFaceCoincidence

//      /*******************************************************************//**
//      PURPOSE: Determine if collection of faces point in the same general
//                  direction as given normal
//                  Static helper function for SM_BO_SLICE operation
//
//      NOTES:
//      ***********************************************************************/
//      static SmBoolean FaceSameDirection( SmVector3d& rSheetNormal, SmTArray<SmFace*>& rFaces )
//      {
//          SmFace* pFace = NULL;
//          SmVector3d sAverageNormal( 0,0,0 ), sNormal;
//          for( ULONG i = 0; i < rFaces.GetSize(); i++ ) {
//              pFace = rFaces[i];
//              pFace->GetSurface()->EvaluateNormal( pFace->GetUVDomain().Evaluate(0.5,0.5), TRUE, TRUE, sNormal );
//              sAverageNormal = sAverageNormal + sNormal;
//          }
//
//          sAverageNormal.Unitize();
//          if( rSheetNormal.Dot( sAverageNormal ) < 0 )
//              return FALSE;
//
//          return TRUE;
//
//      } // end FaceSameDirection

/*******************************************************************//**
PURPOSE: Static helper for ManifoldBoolean(): check whether an SmEdge
   bounds any Solid Region.

NOTES:
   Used (initially anyway) only as a Boolean -- yes or no -- but we have
   to find the Shell anyway, so may as well return it, could be useful.
   The Shell returned is the first one found that's in a Solid region.
***********************************************************************/
static SmShell * sm_EdgeBordersSolid( SmEdge *pE )
{
  SmTArray< SmEdgeuse* > sEUs;
  pE->GetEdgeuses( sEUs );
  ULONG ii, lNumEUs = sEUs.GetSize();
  for ( ii = 0; ii < lNumEUs; ii++ )
    {
      SmEdgeuse *pEU  = sEUs[ii];
      SmShell *pShell = pEU->GetShell();
      if ( pShell == NULL ) { continue; }  // No shell?  No solid.

      if ( ! pShell->GetRegion()->IsVoid() )
        { return pShell; } // Found a solid region.
    }

  // Found no solid regions.
  return NULL;

} // end sm_EdgeBordersSolid

/*******************************************************************//**
PURPOSE: Class for transforming Brep(s) back to near the origin,
   to reduce numerical noise for Breps that are far from the origin.

NOTES:
   This is set up a its own class so that the reverse transformation will
   take place regardless of where we leave scope, as in class SmObjDelete.
***********************************************************************/
class SmTemporaryBrepTransform
{
  SmTopologyIntersector *m_pTI;
  SmPoint3d m_sCenter;
  SmBoolean m_bNeedsTranslation;

public:

  // Constructor
  SmTemporaryBrepTransform( SmTopologyIntersector *pTI )
    : m_pTI( pTI )
  {
    m_bNeedsTranslation = FALSE;

    // get parts bbox
    SmExtent3d sBBox1, sBBox2;
    m_pTI->GetPrimaryBrep()->CalculateBoundingBox( sBBox1 );
    m_pTI->GetOtherBrep()  ->CalculateBoundingBox( sBBox2 );
    sBBox1.Union( sBBox2, sBBox1 );

    // when bbox is very large - translate its center to the origin
    m_sCenter = sBBox1.Evaluate( 0.5, 0.5, 0.5 );
    if ( m_sCenter.LengthSquared() > 1.0e+6 ) // Artibrary: > 1000.  Don't think it makes much difference.
      {
        m_bNeedsTranslation = TRUE;
        SmAxis2Placement sXfm;
        m_sCenter = -m_sCenter;
        sXfm.Translate( m_sCenter );
        m_pTI->GetPrimaryBrep()->Transform( sXfm );
        m_pTI->GetOtherBrep()  ->Transform( sXfm );
      }
  } // end SmTemporaryBrepTransform constructor

  // Destructor
  ~SmTemporaryBrepTransform()
  {
      if ( m_bNeedsTranslation )
      {
        SmAxis2Placement sXfm;
        m_sCenter = -m_sCenter;
        sXfm.Translate( m_sCenter );
        if ( m_pTI->GetPrimaryBrep() != NULL )
          {  m_pTI->GetPrimaryBrep()->Transform( sXfm ); }
        if ( m_pTI->GetOtherBrep() != NULL )
          {  m_pTI->GetOtherBrep()->Transform( sXfm ); }
      }
  } // end SmTemporaryBrepTransform destructor

}; // End class SmTemporaryBrepTransform


/*******************************************************************//**
PURPOSE: Perform a manifold general Boolean operation on
   the two breps contained in this SmMerge object.
   It also performs a general merge operation which
   combines the topology of two breps.

NOTES: The main difference between this and ManifoldBoolean is the
   creation of spine edges -- edges connecting more than two faces.
   ManifoldBoolean does not allow them, it will instead leave coincident
   edges in the model, if more than two faces meet.
   This can be useful for intermediate results.
***********************************************************************/
SmStatus SmMerge::NonManifoldBoolean
  (SmBooleanOperationType eOperation,   // in : oneof SM_BO_UNION
                                        //            SM_BO_INTERSECTION
                                        //            SM_BO_DIFFERENCE
                                        //            SM_BO_EXCLUSIVE_OR
                                        //            SM_BO_MERGE
                                        //            SM_BO_PARTIAL_MERGE
                                        //            SM_BO_IMPRINT
                                        //            SM_BO_IMPRINT_CLASSIFY  (rather: use Union,XSect,Diff... and 
                                        //                                             m_bImprintAndClassifyFaces == TRUE)
                                        //            SM_BO_EXTRACT_SEPARATE
                                        //            SM_BO_SLICE
   SmBrep *& rpResult)                  // out: pointer to result or NULL for failure
{
  // set state parameters for nonManifold booleans
  SmTemporaryChangeValue<SmBoolean> sChange(m_bManifoldBoolean,FALSE);
  m_vTI.m_lIntersectLaminaEdges = 1;

  // pass the call along
  SER(ManifoldBoolean(eOperation, rpResult));
  return SM_SUCCESS;

} // end SmMerge::NonManifoldBoolean

/*******************************************************************//**
PURPOSE: Perform a manifold solid volumetric Boolean operation on
   the two breps.  It also performs a general merge operation which
   combines the topology of two breps.

NOTES: You should use this method if you wish to do Boolean
   operations between two solids.  It will do extra checks to try
   to close loops and fix gaps that the NonManifoldBoolean will not do.
   In addition, ManifoldBoolean will not create spine edges; this can
   result in coincident manifold or lamina edges.

   Note that NonManifoldBoolean() simply calls ManifoldBoolean()
   with m_bManifold flag set to False, and m_vTI.m_lIntersectLaminaEdges = 1
   (intersect lamina edges in both Breps).

   See SmMerge.h for descriptions of the various operation types (SM_BO_UNION, etc)
   and the control flags (m_bManifoldBoolean, etc.).  There are additional control
   flags in the topology intersector (m_vTI), which are described in
   SmTopologyIntersector.h.

   Memory Management:
     SM_BO_UNION,             rpResult = modified m_vTI.m_pBrep, delete m_vTI.m_pOther
     SM_BO_INTERSECTION,      rpResult = modified m_vTI.m_pBrep, delete m_vTI.m_pOther
     SM_BO_DIFFERENCE,        rpResult = modified m_vTI.m_pBrep, delete m_vTI.m_pOther
     SM_BO_EXCLUSIVE_OR,      rpResult = modified m_vTI.m_pBrep, delete m_vTI.m_pOther
     SM_BO_MERGE,             rpResult = modified m_vTI.m_pBrep, delete m_vTI.m_pOther
     SM_BO_PARTIAL_MERGE,     rpResult = modified m_vTI.m_pBrep, save   m_vTI.m_pOther
     SM_BO_IMPRINT,           rpResult = modified m_vTI.m_pBrep, save   m_vTI.m_pOther
     SM_BO_IMPRINT_CLASSIFY,  rpResult = modified m_vTI.m_pBrep, save   m_vTI.m_pOther
     SM_BO_EXTRACT_SEPARATE,  output   = m_vOneBrepPer_OrigBrepConnectedFaceSet array of Breps,
                                         delete m_vTI.m_pBrep, delete m_vTI.m_pOther,
                                         rpResult not used
     SM_BO_SLICE              rpResult = modified m_vTI.m_pBrep, delete m_vTI.m_pOther

   These base behaviors can be modified by SmMerge::Control-parameter values.
     When m_bKeepOtherBrep           == TRUE, save both modified m_vTI.m_pBrep and modified m_vTI.m_pOther
          m_bImprintAndClassifyFaces == TRUE, save both modified m_vTI.m_pBrep and modified m_vTI.m_pOther
          m_bImprinting              == TRUE, save both modified m_vTI.m_pBrep and modified m_vTI.m_pOther

METHOD ---
    1. call m_vTI.IntersectInsertRelate()
       - intersect all faces of pBrep with faces of pOtherBrep
       - merge (insert) intersection geometry into each Brep TopologyGraph
       - record all matched pieces of geometry between the 2 Breps

    2. For every common edge (inserted intersection curve result)
       - Classify the edge->edgeuse->Face against the matching OtherEdge
       - Classify the OtherEdge->Edgeuse->Face against the matching Edge
         - For faces classifed as coincident to a face in the other brep
           - make all edges of the coincident face pair coincident as well
             by projecting the curves of each face onto the others.
           - add the face to the KeepFace or DeleteFace list depending
             on the SameSide/NonSolid face-pair attribute and the operation.
         - For faces that classify to a radial-sector region
           - add the face to the keepFace or DeleteFace list depending
             on the type of the region (Void/NotVoid) and the operation.

    3. For every remaining unmarked face
       - determine which region contains it in the other Brep
       - decide to keep or delete the face based on the operation.

    4. Classify and merge Wire Edges and Shell Vertices

    5. Insert KeepOFaces into Brep from OtherBrep (when not in cookie cutter mode)

    6. Delete DeleteFaces

    7. Propagate shell status (void/notVoid) for matched shells

    8. Find and set the infinite region if not already marked

    9. When m_bBooleanPostProcess is set for
        operations union/intersection/difference
       - remove edges between planar faces and faces which have the same
         surface but are not seam edges.

   10. Set output pointer, rpResult = m_vTI.m_pBrep
         Except operation SEPARATE -
            then make new Breps from each set of disjoint topology and place in
            m_vOneBrepPer_OrigBrepConnectedFaceSet and
            when !m_bKeepOtherBrep, delete m_vTI.m_pBrep

   11. Depending on eOperation, m_bKeepOtherBrep, m_bImprintAndClassifyFaces, and m_bImprinting values
         delete m_vTI.m_pOther
***********************************************************************/
SmStatus SmMerge::ManifoldBoolean
  (SmBooleanOperationType eOperation,     // in : Oneof 
                                          //      SM_BO_UNION        SM_BO_INTERSECTION
                                          //      SM_BO_DIFFERENCE   SM_BO_EXCLUSIVE_OR
                                          //      SM_BO_MERGE        SM_BO_PARTIAL_MERGE
                                          //      SM_BO_IMPRINT      SM_BO_IMPRINT_CLASSIFY 
                                          //                            (rather: use Union,XSect,Diff... and 
                                          //                                     m_bImprintAndClassifyFaces == TRUE)
                                          //      SM_BO_SLICE        SM_BO_EXTRACT_SEPARATE
   SmBrep                *& rpResult,     // out: Boolean Result
   SmNewMarkAndLock       * pOptMarkLock) // i/o : sets and incs MarkValue, NotNULL=valid marks after call, NULL=not valid
                                          //      default:[NULL]
{
  // check input state
  NER(m_vTI.m_pBrep);
  NER(m_vTI.m_pOther);

  // If the Breps are very far from the origin, translate them to the origin.
  // This can help a lot with numerical noise.
  SmTemporaryBrepTransform sTempXfm( &m_vTI );

  // check input: SM_BO_SLICE only defined when otherBrep is an oriented sheet
  if(eOperation == SM_BO_SLICE)
    {
      if(m_vTI.m_pOther->IsSheet())
        {
          if(!m_vTI.m_pOther->IsOrientedSheet())
            {
              // just go ahead and orient the sheet for the user and carry on.
              SmTArray<SmFace*> sFaces ;
              m_vTI.m_pOther->GetFaces(sFaces) ;
              m_vTI.m_pOther->OrientSheetSurfaces(sFaces[0], TRUE) ;
            }
        }
      else
        { return(SM_ERR) ; }
    } // end input check for operation == SM_BO_SLICE

  // set brep contexts for booleans - commonly the same context but not always
  SmContext *pBrepContext  = SM_CONST_CAST(SmContext*,m_vTI.m_pBrep->GetContext());
  SmContext *pOtherContext = SM_CONST_CAST(SmContext*,m_vTI.m_pOther->GetContext());
  pBrepContext->SetDoingBoolean (TRUE);
  pOtherContext->SetDoingBoolean(TRUE);

#ifdef SM_USE_GLOBAL_CACHE
  // temporarily increase both context's global cache queue lengths using the SmCacheMgr mechanism.
  //  The lengths revert to their current values when the new SmCacheMgr objects go out of scope and
  //  get destructed.
  SmCacheMgrBrep sCache (*pBrepContext, SM_EXTENDED_MAXCOUNT_CURVECACHE,
                                        SM_EXTENDED_MAXCOUNT_SURFACECACHE,
                                        SM_EXTENDED_MAXCOUNT_TRIMSRFCACHE,
                                        SM_EXTENDED_MAXCOUNT_BREPCACHE) ;
  SmCacheMgrBrep sCache2(*pOtherContext,SM_EXTENDED_MAXCOUNT_CURVECACHE,
                                        SM_EXTENDED_MAXCOUNT_SURFACECACHE,
                                        SM_EXTENDED_MAXCOUNT_TRIMSRFCACHE,
                                        SM_EXTENDED_MAXCOUNT_BREPCACHE) ;
#endif

  // locals
  ULONG jj, kk, lCount;                       // iterators
  SM_PTR_ARRAY(sBrepFaces, SmFace,256);       // list of coincident faces and when operation == SM_BO_EXTRACT_SEPARATE
  SM_PTR_ARRAY(sOtherFaces,SmFace,256);       //   sBrepFaces  = faces of merged pBrep
  SmTArray<ULONG> sBrepFacesCount;            //   sOtherFaces = faces of merged pOtherBrep
  SmTArray<ULONG> sOtherFacesCount;

#ifdef SM_DEBUG_CODE // dump and draw ManifoldBoolean() input Breps
  ULONG di;
SmBoolean bDebugMe  = FALSE;
SmBoolean bDebugMeX = FALSE ;
static ULONG lThisCount  = 1 ; lThisCount++ ;
static ULONG lDebugCount = 0 ;
  if (bDebugMe || lDebugCount == lThisCount)
    {
      SM_DUMP_AND_ASSERT_VALID(m_vTI.m_pBrep) ;
      SM_DUMP_AND_ASSERT_VALID(m_vTI.m_pOther) ;

      TCHAR sBuff[SM_TBLOCK_SIZE];
      smos_sprintf(sBuff,_T("%s"), _T("m_vTI.m_pBrep"));
      m_vTI.m_pBrep->DumpRegionsAndAttributes (sBuff) ;
      smos_sprintf(sBuff,_T("%s"), _T("m_vTI.m_pOther"));
      m_vTI.m_pOther->DumpRegionsAndAttributes(sBuff) ;

      SmTArray<SmObject*> sKeys ;
      if(m_vTI.GetSubset()) m_vTI.GetSubset()->GetAllKeys(sKeys) ;

      // draw input Breps
      smgfx_Erase();
      smgfx_SetLook(1,2, 0,0,1) ; m_vTI.m_pBrep->Draw(TRUE); sm_GraphicsLoop() ;
      smgfx_SetLook(2,3, 0,1,0) ; m_vTI.m_pOther->Draw(TRUE); sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,1) ; for(ULONG ii=0;ii<sKeys.GetSize();ii++)
                                   { SmObject *pObj = (SmObject *)sKeys[ii] ;
                                     if(pObj->IsKindOf(SmSurface_TYPE))
                                       { SmSurface *pSurf = (SmSurface*)pObj ;
                                         pSurf->DrawUV() ; sm_GraphicsLoop();
                                       }
                                   }
      if ( FALSE ) {
          smgfx_SetLook(1,2, 0,0,1); m_vTI.m_pBrep ->GetInfiniteRegion()->Draw(); sm_GraphicsLoop() ;
          smgfx_SetLook(1,2, 0,1,0); m_vTI.m_pOther->GetInfiniteRegion()->Draw(); sm_GraphicsLoop() ;
      }
      sm_GraphicsLoop();

      // check and try to fix gap sizes
      SmBoolean bViolatedTolerances;
      double dMaxEdgeFaceTrimCurveGap, dMaxEdgeFaceGap ;
      double dMaxVertexEdgeGap, dMaxVertexFaceGap, dMaxVertexFaceTrimCurveGap;
      SER(m_vTI.m_pBrep->ValidateAndUpdateTolerances(TRUE,
                                                     bViolatedTolerances,
                                                     dMaxEdgeFaceTrimCurveGap,
                                                     dMaxVertexEdgeGap,
                                                     dMaxVertexFaceGap,
                                                     NULL, SM_LEVEL_0, NULL,
                                                     NULL, NULL, NULL, NULL, NULL,
                                                    &dMaxEdgeFaceGap,
                                                    &dMaxVertexFaceTrimCurveGap));  // note: increments unlocked mark value
      if (bViolatedTolerances)
        { SE(SM_ERR); }

      SER(m_vTI.m_pOther->ValidateAndUpdateTolerances(TRUE,
                                                      bViolatedTolerances,
                                                      dMaxEdgeFaceTrimCurveGap,
                                                      dMaxVertexEdgeGap,
                                                      dMaxVertexFaceGap,
                                                      NULL, SM_LEVEL_0, NULL,
                                                      NULL, NULL, NULL, NULL, NULL,
                                                     &dMaxEdgeFaceGap,
                                                     &dMaxVertexFaceTrimCurveGap));  // note: increments unlocked mark value
      if (bViolatedTolerances)
        { SE(SM_ERR); }

    }
#endif // SM_DEBUG_CODE

  // Set up the Breps for editing.
  { 
    SmTemporaryChangeValue<SmBoolean> sStack1(m_vTI.m_pBrep->m_bEditingEnabled, TRUE);
    SmTemporaryChangeValue<SmBoolean> sStack2(m_vTI.m_pOther->m_bEditingEnabled,TRUE);

    // GWC:tried Removing 2 lines - caused prog_test failures - SmMerge without Composites needs work
    // 
// Remove Composites
// //cbi_CEdge: 5
// #ifndef SM_NO_COMPOSITES
//     m_vTI.m_pBrep->m_bMakeComposites  = TRUE;
//     m_vTI.m_pOther->m_bMakeComposites = TRUE;
// #endif  // SM_NO_COMPOSITES

  // STEP 1. - Intersect Insert Relate
  {
    // Intersect the entities of these two breps, insert intersections into
    // the corresponding topologies and set up the relationships.
    SER(m_vTI.IntersectInsertRelate());

#ifdef SM_DEBUG_CODE
    // draw Breps with newly intersected and inserted geometry
    if (bDebugMeX || lDebugCount == lThisCount)
      {
        SM_DUMP_AND_ASSERT_VALID(m_vTI.m_pBrep) ;
        SM_DUMP_AND_ASSERT_VALID(m_vTI.m_pOther) ;

        TCHAR sBuff[SM_TBLOCK_SIZE];
        smos_sprintf(sBuff,_T("%s"), _T("m_vTI.m_pBrep"));
        m_vTI.m_pBrep->DumpRegionsAndAttributes(sBuff);
        smos_sprintf(sBuff,_T("%s"), _T("m_vTI.m_pOther"));
        m_vTI.m_pOther->DumpRegionsAndAttributes(sBuff) ;

        if(FALSE)
          { m_vTI.m_pBrep->DumpAttributes() ; }
        m_vTI.Dump() ;

        //SmBoolean bClosedIntersectionLoops = TRUE ;
        //if(FALSE)
        //  { bClosedIntersectionLoops = m_vTI.AreIntersectionLoopsClosed() ; }

        smgfx_Erase() ;
        smgfx_SetLook(1,6, 0,0,1) ; m_vTI.m_pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
        smgfx_SetLook(2,7, 0,1,0) ; m_vTI.m_pOther->Draw(TRUE) ; sm_GraphicsLoop() ;
        smgfx_SetLook(4,8, 0,1,1) ; m_vTI.Draw(4.0, 6.0, 8.0, 10.0) ; sm_GraphicsLoop() ;
        if(FALSE)
          { // the DrawDetails() call is full of expensive gap calcs - only call when you want to see the extra information
            smgfx_SetLook(4,5, 0,1,1) ; m_vTI.DrawDetails(3.0, 4.0) ; sm_GraphicsLoop() ;
          }
        if(FALSE)
          {
            smgfx_SetLook(1,2, 0,0,1); m_vTI.m_pBrep ->GetInfiniteRegion()->Draw(); sm_GraphicsLoop() ;
            smgfx_SetLook(1,2, 0,1,0); m_vTI.m_pOther->GetInfiniteRegion()->Draw(); sm_GraphicsLoop() ;
          }
        sm_GraphicsLoop() ;
      } // end bDebugMe check

#endif // SM_DEBUG_CODE

#ifdef SM_VALIDATE_TOPOLOGY
    smos_WriteBuffer(_T("SmMerge.cpp - Validating Topology is ON!"),_T(""));
  // replace when debugging a failed case
  //     m_vTI.m_pBrep->ValidatePointers();
  //     m_vTI.m_pOther->ValidatePointers();
#endif // SM_VALIDATE_TOPOLOGY

    // Now our entity maps contain all of the entities that were directly involved
    // in the Boolean operation.  Later, we will add to the maps all those entities
    // that get copied from one Brep to the other.  But for BooleanPostProcess(),
    // where we remove "topological" edges and vertices, we don't want to process
    // entities that already existed in either Brep, only those that arise during
    // the Boolean operation.  That is exactly the entities that are in the maps
    // at this point.  Copy them into our m_sIntersectionEdges/Vertices lists.

    SmTArray< SmObject* > sObjs;
    m_vTI.GetMappedBrepObjects( SmEdge_TYPE, sObjs );
    ULONG lNumObjs = sObjs.GetSize();
    for ( ULONG ii=0; ii<lNumObjs; ii++ )
      { m_sIntersectionEdges.Add( SM_CAST_PTR( SmEdge, sObjs[ii] ) ); }

    m_vTI.GetMappedBrepObjects( SmVertex_TYPE, sObjs );
    lNumObjs = sObjs.GetSize();
    for ( ULONG ii=0; ii<lNumObjs; ii++ )
      { m_sIntersectionVertices.Add( SM_CAST_PTR( SmVertex, sObjs[ii] ) ); }

  } // end STEP 1 - Intersect Insert Relate

  // locals
  SmTArray<SmEdge*>    sKeepAsWires, sFaceEdges, sOFaceEdges;
  SmTArray<SmEdgeuse*> sEdgeuses;

  SmTArray<SmFace*> sKeepFaces, sKeepOFaces, sDelFaces, sDelOFaces;
  SmTArray<SmFace*> sCollectedFaces, sFaces, sOFaces ;

  // STEP 2 = Loop - Build up KeepFaces and DeleteFaces lists
  //          depending on the operation type and how
  //          the faces of one Brep classify against the edges of the other Brep.
  // 1. after 1st iteration - remove KeepFaces and DelFaces from common entities map
  //                          to cut down on upcoming loop work that has already been done.
  // 2. ReSet KeepFaces, KeepOFaces, DelFaces, DelOFaces, and CollectedFaces lists
  // 3. Loop to Mark all common vertices and edges
  //    also detect and try to fix single-vertices
  //      (which can make new topology - requiring marking to be done in a loop)
  // 4. Check all common vertices for single-vertices,
  //    if any - set m_bIntersectionLoopClosed = FALSE
  // 5. if imprinting only - done - free m_vTI.m_pOther, clean-up and exit
  // 6. if merging a subset -
  //     6a. mark all m_pOther surfaces not in the m_pSubset list
  // 7. For every Common Edge - classify its faces against the other Brep Edges
  //   7A. classify Edge->Edgeuse->Face against matched OtherEdge
  //     to decide which pBrep and pOtherBrep faces to keep or delete
  //      7.1. skip Shell edgeuses and marked Edgeuse->Faces
  //      7.2. Classify Edgeuse against matched other Edge
  //      7.3. if(edgeuse->Face classified coincident to some face attached to Edge)
  //         7.3a check Faceuse regions for Faceuse-pair SameSide/NonSolid status
  //         7.3b Skip OtherFaces already marked
  //         7.3c Call FixFoundFaceCoincidence -
  //              make sure all the edges of the coincident faces are
  //              also coincident by merging the surfaces -
  //                this may make new topology - forcing another iteration
  //         7.3d if operation == SM_BO_EXTRACT_SEPARATE
  //              add coincident faces to sBrepFaces list
  //         7.3e for every (after merge) coincident face
  //              record coincident face relationship
  //              update Keep and Delete lists based on operation type:
  //               Union/Intersection: if(sameSide || NonSolid) KeepFace
  //                                   else                     DeleteFace
  //               Difference        : if(sameSide || NonSolid) DeleteFace
  //                                   else                     KeepFace
  //               PartialMerge/Merge: KeepFace
  //      7.4. else if(edgeuse->Face classified to a radial sector) branch
  //         7.4a collect unmarked faces
  //         7.4b if operation == Separate, add collected faces to SBrepFace lists
  //         7.4c Get classified radial sector's region
  //         7.4d update Keep and Delete lists based on operation type:
  //               Union/Intersection: if(region==Void)         KeepCollectedFaces
  //                                   else                     DeleteCollectedFaces
  //               Difference        : if(region==void)         DeleteCollectedFaces
  //                                   else                     KeepCollectedFaces
  //               PartialMerge/Merge: KeepCollectedFaces
  //      7.5 If topology graphs were modified - break for another loop iteration
  //     7.B same as 7.A with pBrep and pOtherBrep interchanged.
  //         for every OtherEdge->Edgeuse->Face
  //         classify OtherEdge->Edgeuse->Face against matched Edge
  //         to decide which pBrep and pOtherBrep faces to keep or delete

  // lock marks for duration of this scope - causes debug only warning when mark is incremented

  // increment and lock unlocked mark for both pBrepContext and pOtherContext
  SmNewMarkAndLock sMarkLock, * pMarkLock = pOptMarkLock ? pOptMarkLock : & sMarkLock ;
  pMarkLock->SetContext(pBrepContext, pOtherContext) ;
  SmMarkType eBrepMarkType  = pMarkLock->GetMarkType1() ;
  SmMarkType eOtherMarkType = pMarkLock->GetMarkType2() ;

  // STEP 2.
  // Classify the common edge->edgeuse->faces and OtherEdge->Edgeuse->Faces against
  //  their partner's edge->sectors to decide whether to keep or delete the faces.
  // Build KeepFaces and DeleteFaces lists
  //  also 1. detect and fix single-vertex intersection-loop gaps
  //       2. detect coincident faces and make their edge's coincident as well
    {
      ULONG lOuterTries = 0;
      SmBoolean bDone = FALSE;
      while (!bDone)
        {
          lOuterTries ++;
          if (lOuterTries > 10) { break; }
          bDone = TRUE;

          // 1. In case we are looping -
          //    Remove m_vTI.Mate relationships for Keep and Delete Faces
          //    to prevent reclassifying faces already classified.
          lCount = sKeepFaces.GetSize();
          for(ULONG ii=0; ii<lCount; ii++)
            {
              SmFace *pF = sKeepFaces[ii];
              SmFace *pOF = (SmFace*)m_vTI.GetOtherMate(pF);
              if (pOF)
                {
                  m_vTI.RemoveRelationship(pF,pOF);
                }
            }
          lCount = sDelFaces.GetSize();
          for(ULONG ii=0; ii<lCount; ii++)
            {
              SmFace *pF = sDelFaces[ii];
              SmFace *pOF = (SmFace*)m_vTI.GetOtherMate(pF);
              if (pOF)
                {
                  m_vTI.RemoveRelationship(pF,pOF);
                }
            }

          // 2. clear the accumulation lists
          sKeepFaces.ReSet();
          sKeepOFaces.ReSet();
          sDelFaces.ReSet();
          sDelOFaces.ReSet();
          sCollectedFaces.ReSet();

          // 3. Mark all common edges and vertices - used for skipping already processed topology.
          //    Also fix single-vertices
          // 3.1. Also find all single-vertices - they represent gaps in intersection loops
          // 3.2. for every single-vertex
          //    2a. Call m_vTI.CloseIntersectionLoop() - makes new topoology
          //    2b. if still a single-vertex, call m_vTI.ReIntersectAtVertex - makes new topology
          //    2c. if added or deleted topology objects - iterate mark loop once again
          //        else vertex still single - add vertex to sSingleVertices list.
          // 3.3. if all commons are marked and sSingleVertices list not NULL
          //    3a. Call m_vTI.FixGaps() to try to merge vertices close to one another
          //    3b. if added or deleted topology objects - iterate mark loop once again
          //
          SmBoolean bDoneMarking = FALSE;
          ULONG lTries = 0;
          while (!bDoneMarking)
            {
              lTries ++;
              if (lTries > 10) { break; }
              bDoneMarking = TRUE;

              // increment the Brep->Context's and mp_Other->Context's mark values - this function locked the mark so skip debug warnings
              pMarkLock->NewMark() ;

              // Mark all common edges and vertices as already processed.
              // note: 1. a common object is any topology object (vertex, edge, or face)
              //          already added (or mapped) to both of the Breps by the boolean object.
              //          These common objects represent the intersection of one Brep's
              //          boundary with the other.
              //
              //       2. while marking, detect single-vertices and attempt
              //          to correct them.  In a manifold operation a single
              //          vertex (connected to just one common edge) is an error
              //          probably due to a gap in an intersection loop or the
              //          mistaken addition of 2 vertices when just one was needed.
              SmTArray<SmVertex*> sSingleVertices;

                { // 3.a next - mark all the common edges
                  // get common edges mapped between the 2 breps
                  m_vTI.GetCommonEdges(m_vEdges,m_vOtherEdges);

                  // mark common edges
                  lCount = m_vEdges.GetSize();
                  for(ULONG ii=0; ii<lCount; ii++)
                    {
                      SmEdge *pEdge      = m_vEdges[ii];
                      SmEdge *pOtherEdge = m_vOtherEdges[ii];
                      pEdge->Mark(eBrepMarkType);
                      pOtherEdge->Mark(eOtherMarkType);
#ifdef SM_DEBUG_CODE
                      if (bDebugMe)
                        {
                          pEdge->GetCurve()->Dump();
                          pEdge->GetInterval().Dump();

                          if (ii==0) { smgfx_Erase();
                                       smgfx_SetLook(1,2, 0,0,1) ; m_vTI.m_pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
                                       smgfx_SetLook(1,2, 0,1,0) ; m_vTI.m_pOther->Draw(TRUE) ; sm_GraphicsLoop() ;
                                     }
                          smgfx_SetLook(3, 4, 1,0,0); pEdge->Draw(); sm_GraphicsLoop();
                          smgfx_SetLook(6, 7, 1,0,1); pOtherEdge->Draw(); sm_GraphicsLoop();
                          sm_GraphicsLoop();
                        }
#endif // SM_DEBUG_CODE
                    } // end iter every common edge
                } // end mark common edges block

                { // 3.b next - mark common vertices - detect/correct single vertices
                  m_vTI.GetCommonVertices(m_vVertices,m_vOtherVertices);

                  // mark common vertices
                  lCount = m_vVertices.GetSize();
                  for(ULONG ii=0; ii<lCount; ii++)
                    {
                      SmVertex *pVertex      = m_vVertices[ii];
                      SmVertex *pOtherVertex = m_vOtherVertices[ii];
                      pVertex->Mark(eBrepMarkType);
                      pOtherVertex->Mark(eOtherMarkType);

                      SmEdge * sEData[32];
                      SmTArray<SmEdge*> sVEdges(32,sEData);
                      pVertex->GetEdges(sVEdges);
                      ULONG lECount = 0;
                      SmBoolean bLamina = FALSE;

                      // 3.1 detect single and lamina vertices
                      //     single-vertex: connected to one common edge that's not closed
                      ULONG lNumVEdges = sVEdges.GetSize();
                      for(jj=0; jj<lNumVEdges; jj++)
                        {
                          SmEdge *pEdge = sVEdges[jj];
                          if (pEdge->IsLamina()) { bLamina = TRUE; }
                          if (m_vTI.GetOtherMate(pEdge) != NULL)
                            {
                              lECount++;
                            }
                          if (pEdge->IsClosed())
                            {
                              lECount++;
                            }

                        }  // end iter edges

                      // 3.2 if the vertex is a single vertex
                      //     note: a vertex connected to one non-lamina closed edge
                      //           will have an lECount of 2
                      if (lECount == 1 && !bLamina)
                        {
                          SmBoolean bMadeNewTopology = FALSE;
                          m_vTI.m_bTopologyDeleted = FALSE;
                          // and this is a manifold operation
                          if (!bLamina && m_bManifoldBoolean)
                            {
                              // 3.2a find any edges connected to pVertex that can be
                              //      intersected, inserted, and related to m_pOther to
                              //      try and plug gaps in intersection loops.
                              SmStatus eStat = m_vTI.CloseIntersectionLoop( pVertex, bMadeNewTopology );
                              // (Was SER. [B601])

                              // Note: ReIntersectAtVertex() has never been known to help anything,
                              // and it's expensive and complicates the algorithm.
                              // If you see a case where it might help, try reinstating it.
                              //if ( eStat == SM_SUCCESS && ! bMadeNewTopology )
                              //  {
                              //    // 3.2b if that didn't work - try reintersection to find missing edges at this vertex
                              //    eStat = m_vTI.ReIntersectAtVertex( pVertex, pOtherVertex, bMadeNewTopology );
                              //    // RCLxx chaged SER to SE
                              //  }

                              if ( eStat != SM_SUCCESS )
                                { bMadeNewTopology = FALSE; } // Don't try doing anything else.

                            } // end non-manifold vertex check

                          // 3.2c if we've added or removed topology - iterate the mark loop once again
                          if (bMadeNewTopology || m_vTI.m_bTopologyDeleted)
                            {
                              bDoneMarking = FALSE;
                              break;
                            }
                          else // if fix attempts failed - place vertes on sSingleVertices list
                            {
                              // add to list of single vertices
                              sSingleVertices.Add(pVertex);
                            }
                        } // end common vertex is non-manifold check
#ifdef SM_DEBUG_CODE
                      if (bDebugMe)
                        {
                          // SmEdge * sEdgeData[32];
                          // SmTArray<SmEdge*> sVEdges(32, sEdgeData );
                          // pVertex->GetEdges(sVEdges);

                          if (lECount != 2) { smgfx_SetLook(4,8, 1,0,0); }
                          else              { smgfx_SetLook(4,6, 0,0,0); }
                          pVertex->Draw(); sm_GraphicsLoop();
                          sm_GraphicsLoop();
                        }
#endif // SM_DEBUG_CODE
                    } // end iter every common vertex
                } // end mark common vertices block

              // 3.3 If we are done marking and still have single vertices then we need to
              //     check to see if two of the single vertices are close to each other
              //     and need to be merged together.
              if (   bDoneMarking
                  && sSingleVertices.GetSize() > 1
                  && m_bManifoldBoolean)
                {
                  SmBoolean bModifiedTopology;
#ifdef SM_VALIDATE_TOPOLOGY
    // replace when debugging a failed case
    //     m_vTI.m_pBrep->ValidatePointers();
    //     m_vTI.m_pOther->ValidatePointers();
#endif // SM_VALIDATE_TOPOLOGY
                  // 3.3a try to merge nearly coincident single-vertices

                  // This could delete entities in our stored pointers:
                  SmTArray< SmVertex* > sDelVerts;
                  SmTArray< SmEdge*   > sDelEdges;

                  SER( m_vTI.FixGaps( sSingleVertices, bModifiedTopology, sDelVerts, sDelEdges ));

                  m_sIntersectionVertices.RemoveElements( sDelVerts, m_sIntersectionVertices );
                  m_sIntersectionEdges   .RemoveElements( sDelEdges, m_sIntersectionEdges    );

#ifdef SM_VALIDATE_TOPOLOGY
    // replace when debugging a failed case
    //     m_vTI.m_pBrep->ValidatePointers();
    //     m_vTI.m_pOther->ValidatePointers();
#endif // SM_VALIDATE_TOPOLOGY
                  // 3.3b if fix added new topology - iterate mark loop once again
                  if (bModifiedTopology)
                    {
                      bDoneMarking = FALSE;
                    }
                } // single-vertex existence check
            } // while not done marking

          // arive here after all common Edges and Vertices have been marked (pMarkLock)
          // and first attempts to fix open intersection loop gaps have been tried

            {  // 4. set      m_bIntersectionLoopClosed = TRUE when all intersection loops are closed
               //    else set m_bIntersectionLoopClosed = FALSE.
              m_bIntersectionLoopClosed = TRUE;
              m_vTI.GetCommonVertices(m_vVertices,m_vOtherVertices);

              // for every common vertex - count number of common edges - if any vertex has just one - m_bIntersectionLoopClosed = FALSE
              lCount = m_vVertices.GetSize();
              for(ULONG ii=0; ii<lCount; ii++)
                {
                  SmVertex *pVertex      = m_vVertices[ii];
                  //SmVertex *pOtherVertex = m_vOtherVertices[ii];

                  SmEdge * sEData[32];
                  SmTArray<SmEdge*> sVEdges(32,sEData);
                  pVertex->GetEdges(sVEdges);
                  ULONG     lECount = 0;

                  // Count number of common edges at this vertex (one extra if Edge is closed)
                  ULONG lNumVEdges = sVEdges.GetSize();
                  for(jj=0; jj<lNumVEdges; jj++)
                    {
                      SmEdge *pEdge = sVEdges[jj];
                      if (m_vTI.GetOtherMate(pEdge) != NULL)
                        {
                          lECount++;
                        }
                      if (pEdge->IsClosed())
                        {
                          lECount++;
                        }
#ifdef SM_DEBUG_CODE
                      if (bDebugMe)
                        {
                          pEdge->GetCurve()->Dump(); pEdge->GetInterval().Dump();

                          if (jj==0) { smgfx_Erase();
                                       smgfx_SetLook(1,2, 0,0,1) ; m_vTI.m_pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
                                       // smgfx_SetLook(1,2, 0,1,0) ; m_vTI.m_pOther->Draw(TRUE) ; sm_GraphicsLoop() ;
                                     }
                          smgfx_SetLook(3, 4+jj, 1,((double)(jj+1))/lNumVEdges,0); pEdge->Draw(); sm_GraphicsLoop();
                          sm_GraphicsLoop();
                        }
#endif // SM_DEBUG_CODE

                    }
                  // note unclosed Loops if any vertex is a single-vertex and move on
                  if (lECount == 1)
                    {
                      m_bIntersectionLoopClosed = FALSE;
                      break;
                    }
                } // end iter every common vertex
            } // end setting m_bIntersectionLoopClosed = TRUE or FALSE block

          // arrive here after all common Edges and Vertices have been marked
          // and first attempts to fix open intersection loop gaps have been tried
          // and m_bIntersectionLoopClosed is set TRUE or FALSE to mark open intersection loops

          // 5. If we are only imprinting then we are done -
          //    remove m_pOther and clean up.
          if ( eOperation == SM_BO_IMPRINT || m_bImprinting )
            {
              sStack2.Clear();
              if( !m_bKeepOtherBrep )
              {
                  SM_ASSERT(m_vTI.m_pOther != NULL) ; delete m_vTI.m_pOther ;
                  m_vTI.m_pOther = NULL;
              }

              // restore state
              pBrepContext->SetDoingBoolean(FALSE);
              pOtherContext->SetDoingBoolean(FALSE);

              // set output value
              rpResult = m_vTI.m_pBrep;

              // all done
              return SM_SUCCESS;

            } // end Imprinting == TRUE check

          // 6. mark all OtherBrep->faces to be ignored in a PartialMerge
          SmTArray<SmFace*> sTFaces ;
          if (eOperation == SM_BO_PARTIAL_MERGE)
            {
              m_vTI.m_pOther->GetFaces(sTFaces);

              // iter all other brep faces
              ULONG lNumTFaces = sTFaces.GetSize();
              for(ULONG ii=0; ii<lNumTFaces; ii++)
                {
                  SmFace    *pOF   = sTFaces[ii];
                  SmSurface *pSurf = pOF->GetSurface();

                  // mark OtherBrep->surface not in the given subset
                  if (   m_vTI.m_pSubset
                      && m_vTI.m_pSubset->At(pSurf) == NULL)
                    {
                      pOF->Mark(eOtherMarkType);
                    }
                } // end iter all other brep faces
              sTFaces.ReSet();

            } // end marking faces to ignore in a partial_merge

          // arrive here when all common edges and vertices are marked
          //        - intersection loop gaps have been fixed
          //        - faces to be ignored by SM_BO_PARTIAL_MERGE are marked

          // 7. Now classify faces of the common edges
          m_vTI.GetCommonEdges   (m_vEdges,   m_vOtherEdges);
          m_vTI.GetCommonVertices(m_vVertices,m_vOtherVertices);

          SmTArray<SmEdge*> sEdges, sOtherEdges;

          // 7. for every ThisBrep->common edge - classify attached faces
          ULONG lNumEUs, lNumFaces;
          lCount = m_vEdges.GetSize();
          for(ULONG ii=0; ii<lCount; ii++)
            {
              SmEdge *pEdge      = m_vEdges[ii];
              SmEdge *pOtherEdge = m_vOtherEdges[ii];

              // get all ThisBrep->CommonEdge->edgeuses (one edgeuse - one faceuse)
              pEdge->GetEdgeuses(sEdgeuses);

              // 7.A for every other ThisBrep->CommonEdge->Edgeuse->Face
              //      classify Face against matched OtherEdge->Sectors
              //      to decide which pBrep faces to keep or delete
              lNumEUs = sEdgeuses.GetSize();
              for(jj=0; jj<lNumEUs; jj+=2)
                {
                  SmEdgeuse *pEU = sEdgeuses[jj];

                  // 7.1. skip shell edgeuses - they don't have faces
                  if (pEU->IsShellEdgeuse()) { continue; }

                  // 7.1. skip marked edgeuse->faces - its already classified or to be ignored
                  SmFace *pFace = pEU->GetFace();
                  if (pFace->IsMarked(eBrepMarkType)) { continue; }

#ifdef SM_DEBUG_CODE
                  if (bDebugMe)
                    {
                      smgfx_Erase();
                      smgfx_SetLook(1,2, 0,0,1) ; m_vTI.m_pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
                      smgfx_SetLook(2,3, 0,1,0) ; m_vTI.m_pOther->Draw(TRUE) ; sm_GraphicsLoop() ;
                      smgfx_SetLook(3,5, 1,0,0) ; pEdge->Draw(); sm_GraphicsLoop();
                      smgfx_SetLook(4,6, 1,0,1) ; pEU->Draw() ; sm_GraphicsLoop();
                      smgfx_SetLook(1,2, 0,1,1) ; pEU->GetFaceuse()->Draw(SM_DM_CROSSHATCH,8,8); sm_GraphicsLoop();
                      smgfx_SetLook(1,2, 0,0,0) ; pEU->GetFaceuse()->GetFace()->Draw(SM_DM_CROSSHATCH,5,5); sm_GraphicsLoop() ;
                      sm_GraphicsLoop();
                    }
#endif // SM_DEBUG_CODE

                  // 7.2 Classify ThisBrep->Edgeuse->face relative to otherBrep->Edge->Sectors
                  SmEdgeuse *pFoundEU = NULL;
                  SmFaceuse *pFoundFU = NULL;
                  if(SM_SUCCESS != pOtherEdge->FindRadialSector(pEU,         // in pBrep
                                                                NULL,
                                                                SM_OT_SAME,
                                                                pFoundEU,    // in pOtherBrep
                                                                pFoundFU))   // in pOtherBrep
                    {
                      continue; // This was too difficult an edge to sector - let it choose
                                // a different one later on - just skip it
                    }

                  // 7.3 First handle the coincident face case
                  //     The pEU->Face was coincident to some face attached to pOtherEdge.
                  //     The coincident face is stored in pFoundFU.
                  if (pFoundFU)
                    {
                      // Other Faceuse locals
                      SmFace   *pOFace       = pFoundFU->GetFace();
                      SmShell  *pOShell      = pFoundFU->GetShell();
                      SmRegion *pORegion     = pOShell->GetRegion();
                      SmShell  *pOShellMate  = pFoundFU->GetMate()->GetShell();
                      SmRegion *pORegionMate = pOShellMate->GetRegion();

                      // this Faceuse locals
                      SmShell  *pShell       = pEU->GetShell();
                      SmRegion *pRegion      = pShell->GetRegion();
                      SmShell  *pShellMate   = pEU->GetMate()->GetShell();
                      SmRegion *pRegionMate  = pShellMate->GetRegion();

                      // 7.3a check Faceuse and Faceuse->Mate regions for Faceuse-pair SameSide/NonSolid status
                      SmBoolean bOnSameSide  = FALSE;  // TRUE = when manifold OtherInside overlaps ThisInside at this face
                      SmBoolean bNonSolid    = FALSE;  // TRUE = other/this region->voids match but other/this regionMate->voids don't
                      if (pORegion->IsVoid() == pRegion->IsVoid())
                        {
                          //
                          if (pORegionMate->IsVoid() != pRegionMate->IsVoid())
                            {
                              bNonSolid = TRUE;
                            }
                          else
                            {
                              bOnSameSide = TRUE;
                            }
                        } // end SameSide/NonSolid check

                      // 7.3b skip marked OtherFaces - already processed
                      if (pOFace->IsMarked(eOtherMarkType)) continue;

                      // 7.3c make sure all the edges of the coincident faces are
                      //      also coincident
                      SmBoolean bModifiedTopology;
                      SER(FixFoundFaceCoincidence(pFace,  pOFace,
                                                  sFaces, sOFaces,
                                                  bModifiedTopology));

                      // if topology graph changes - run through the big loop again
                      if (bModifiedTopology)
                        {
                          bDone = FALSE;
                          break;
                        }

                      // 7.3d add coincident faces to sBrepFaces list
                      if (eOperation == SM_BO_EXTRACT_SEPARATE)
                        {
                          sBrepFaces.Append(sFaces);
                          sBrepFacesCount.Add(sFaces.GetSize());
                        }

                      // 7.3e relate and mark every coincident/compatible face pair
                      lNumFaces = sFaces.GetSize();
                      for(kk=0; kk<lNumFaces; kk++)
                        {
                          // record coincident face relationship
                          pFace  = sFaces[kk];
                          pOFace = sOFaces[kk];
                          SER(m_vTI.Relate(pFace,pOFace));
                          pFace->Mark(eBrepMarkType);
                          pOFace->Mark(eOtherMarkType);

#ifdef SM_DEBUG_CODE
                          if (bDebugMe)
                            {
                              smgfx_Erase();
                              smgfx_SetLook(1,2, 0,0,1) ; m_vTI.m_pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
                              smgfx_SetLook(1,2, 0,1,0) ; m_vTI.m_pOther->Draw(TRUE) ; sm_GraphicsLoop() ;
                              smgfx_SetLook(1,2, 0,1,1) ; pFace->Draw(SM_DM_CROSSHATCH, 8, 8); sm_GraphicsLoop() ;
                              smgfx_SetLook(1,2, 1,1,0) ; pOFace->Draw(SM_DM_CROSSHATCH, 8, 8); sm_GraphicsLoop() ;
                              sm_GraphicsLoop();
                            }
#endif // SM_DEBUG_CODE

                          // 7.3e update Keep and Delete lists based on operation type
                          switch (eOperation)
                            {
                              case SM_BO_UNION:
                              case SM_BO_INTERSECTION:
                                  if (bOnSameSide || bNonSolid) { sKeepFaces.Add(pFace); sKeepOFaces.Add(pOFace); }
                                  else                          { sDelFaces.Add(pFace); sDelOFaces.Add(pOFace); }
                                  break;

                              case SM_BO_DIFFERENCE:
                              case SM_BO_SLICE:
                                  if (bOnSameSide && !bNonSolid) { sDelFaces.Add(pFace);  sDelOFaces.Add(pOFace); }
                                  else                           { sKeepFaces.Add(pFace); sKeepOFaces.Add(pOFace); }
                                  break;
                              case SM_BO_EXCLUSIVE_OR:
                                  sDelFaces.Add(pFace);  sDelOFaces.Add(pOFace);
                                  break ;
                              case SM_BO_PARTIAL_MERGE:
                              case SM_BO_MERGE:
                                  sKeepFaces.Add(pFace); sKeepOFaces.Add(pOFace);
                                  break;
                              case SM_BO_UNKNOWN:
                              case SM_BO_EXTRACT_SEPARATE:
                              case SM_BO_IMPRINT:
                              case SM_BO_IMPRINT_CLASSIFY:
                                  break;
                           } // end switch on operation
                        } // end iter every (after merge) coincident face
                    } // end EU classified to a coincident face branch
                  else if (pFoundEU)
                    {
                      // 7.4 else EU classified to a containing radial sector branch
                      SmTopologyTraverser sTraverser;
                      SmFace *pF = pEU->GetFace();

                      // 7.4a collect faces attached to pF that are not marked
                      //      without going through marked edges
                      SER(sTraverser.CollectFaces(pF,            // in : Seed face (gets marked)
                                                  sCollectedFaces,  // out: List of connected faces (Get marked)
                                                  eBrepMarkType));  // in : specify mark for target objects (not incremented)
#ifdef SM_DEBUG_CODE
                      if (bDebugMe)
                        {
                          smgfx_Erase();
                          smgfx_SetLook(1,2, 0,0,1) ; m_vTI.m_pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
                          smgfx_SetLook(2,3, 0,1,0) ; m_vTI.m_pOther->Draw(TRUE) ; sm_GraphicsLoop() ;
                          smgfx_SetLook(5,6, 1,0,0) ; pEdge->Draw(); sm_GraphicsLoop() ;
                          smgfx_SetLook(7,8, 1,0,1) ; pEU->Draw() ; sm_GraphicsLoop();
                          smgfx_SetLook(7,8, 1,1,0) ; pFoundEU->Draw(); sm_GraphicsLoop() ;
                          for(di=0; di<sCollectedFaces.GetSize(); di++)
                            { smgfx_SetLook(1,2, 0,1,1) ; sCollectedFaces[di]->Draw(SM_DM_CROSSHATCH); sm_GraphicsLoop() ;
                            }
                          sm_GraphicsLoop();
                        }
#endif // SM_DEBUG_CODE

                      // 7.4b when separating - place collected faces onto sBrepFaces lists
                      if (eOperation == SM_BO_EXTRACT_SEPARATE)
                        {
                          sBrepFaces.Append(sCollectedFaces);
                          sBrepFacesCount.Add(sCollectedFaces.GetSize());
                        }

                      // 7.4c get classified radial sector's region
                      SmShell  *pOShell  = pFoundEU->GetShell();
                      SmRegion *pORegion = pOShell->GetRegion();

                      // 7.4d update Keep and Delete lists based on operation type:
                      switch (eOperation)
                        {
                          case SM_BO_UNION:
                          case SM_BO_DIFFERENCE:
                              if (pORegion->IsVoid()) {sKeepFaces.Append(sCollectedFaces);}
                              else                    { sDelFaces.Append(sCollectedFaces);}
                              break;
                          case SM_BO_SLICE:
                              if (pFoundEU->GetFaceuse()->GetOrientation() == SM_OT_SAME)
                                   { sKeepFaces.Append(sCollectedFaces); }
                              else { sDelFaces.Append(sCollectedFaces);  }
                              break;
                          case SM_BO_INTERSECTION:
                              if (pORegion->IsVoid()) { sDelFaces.Append(sCollectedFaces);}
                              else                    {sKeepFaces.Append(sCollectedFaces);}
                              break;
                          case SM_BO_EXCLUSIVE_OR:
                          case SM_BO_PARTIAL_MERGE:
                          case SM_BO_MERGE:
                              sKeepFaces.Append(sCollectedFaces);
                              break;
                          case SM_BO_UNKNOWN:
                          case SM_BO_EXTRACT_SEPARATE:
                          case SM_BO_IMPRINT:
                          case SM_BO_IMPRINT_CLASSIFY:
                              break;
                        } // end switch on operation
                    } // end EU classified to a radial sector branch

                  // any pBEdge->Edgeuse->Face that fails to classify
                  // against the matched OtherEdge is an error.
                  else
                    {
                      SER(SM_ERR);
                    }

                } // end iter every commonedge->edgeuse classifying faces as Keep or Delete

              // arrive here after all faces of currentCommonThisEdge are classified
              //   as Keep or Delete.
              //

              // 7.5 if topology graphs were modified by making coincident faces compatible
              //     - break for another big loop iteration
              if (!bDone) 
                { break; }

              // now the partner, get all OtherBrep->CommonEdge->edgeuses (one edgeuse - one faceuse)
              pOtherEdge->GetEdgeuses(sEdgeuses);

              // 7.B for every other OtherBrep->CommonEdge->Edgeuse->Face
              //      classify Face against matched ThisEdge->Sectors
              //      to decide which pOtherBrep faces to keep or delete
              lNumEUs = sEdgeuses.GetSize();
              for(jj=0; jj<lNumEUs; jj+=2)
                {
                  SmEdgeuse *pOEU    = sEdgeuses[jj];

                  // skip Shell Edgeuses - they don't have faces
                  if (pOEU->IsShellEdgeuse()) { continue; }

                  // skip marked edgeuse->faces - they're already classified or to be ignored
                  SmFace    *pOFace = pOEU->GetFace();
                  if (pOFace->IsMarked(eOtherMarkType)) { continue; }

#ifdef SM_DEBUG_CODE
                  if (bDebugMe)
                    {
                      smgfx_Erase();
                      smgfx_SetLook(1,2, 0,0,1) ; m_vTI.m_pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
                      smgfx_SetLook(1,2, 0,1,0) ; m_vTI.m_pOther->Draw(TRUE) ; sm_GraphicsLoop() ;
                      smgfx_SetLook(4,6, 1,0,0) ; pOtherEdge->Draw(); sm_GraphicsLoop();
                      smgfx_SetLook(3,5, 1,0,0) ; pEdge->Draw(); sm_GraphicsLoop();  // expect pEdge and pOtherEdge coincidence
                      smgfx_SetLook(5,7, 1,0,1) ; pOEU->Draw() ; sm_GraphicsLoop();
                      smgfx_SetLook(1,2, 0,1,1) ; if(pOEU->GetFaceuse()) pOEU->GetFaceuse()->Draw(SM_DM_CROSSHATCH,8,8); sm_GraphicsLoop();
                      sm_GraphicsLoop();
                    }
#endif // SM_DEBUG_CODE

                  // Classify OtherBrep->Edgeuse->Face relative to ThisEdge->Sectors
                  SmEdgeuse *pFoundEU = NULL;
                  SmFaceuse *pFoundFU = NULL;
                  if(SM_SUCCESS != pEdge->FindRadialSector(pOEU,       // in pOtherBrep
                                                           NULL,
                                                           SM_OT_SAME,
                                                           pFoundEU,   // in pBrep
                                                           pFoundFU))  // in pBrep
                    {
                      continue;  // If we fail just go to different edgeuse
                    }

                  // First handle the coincident face case
                  //     The pEU->Face was coincident to some face attached to pEdge.
                  //     The coincident face is stored in pFoundFU.
                  if (pFoundFU)
                    {
                      // this Faceuse locals
                      SmFace   *pFace        = pFoundFU->GetFace();
                      SmShell  *pShell       = pFoundFU->GetShell();
                      SmShell  *pShellMate   = pFoundFU->GetMate()->GetShell();
                      SmRegion *pRegionMate  = pShellMate->GetRegion();
                      SmRegion *pRegion      = pShell->GetRegion();

                      // Other Faceuse locals
                      SmShell  *pOShell      = pOEU->GetShell();
                      SmShell  *pOShellMate  = pOEU->GetMate()->GetShell();
                      SmRegion *pORegionMate = pOShellMate->GetRegion();
                      SmRegion *pORegion     = pOShell->GetRegion();

                      // check Faceuse and Faceuse->Mate regions for Faceuse-pair SameSide/NonSolid status
                      SmBoolean bOnSameSide  = FALSE;
                      SmBoolean bNonSolid    = FALSE;
                      if (pORegion->IsVoid() == pRegion->IsVoid())
                        {
                          if (pORegionMate->IsVoid() != pRegionMate->IsVoid())
                            {
                              bNonSolid = TRUE;
                            }
                          else
                            {
                              bOnSameSide = TRUE;
                            }
                        }

                      // skip marked ThisFaces - already processed
                      if (pFace->IsMarked(eBrepMarkType)) { continue; }

                      // make sure all the edges of the coincident faces are also coincident
                      SmBoolean bModifiedTopology;
                      SER(FixFoundFaceCoincidence(pFace,pOFace,sFaces,sOFaces,bModifiedTopology));
                      if (bModifiedTopology)
                        {
                          bDone = FALSE;
                          break;
                        }

                      // add coincident faces to sBrepFaces list
                      if (eOperation == SM_BO_EXTRACT_SEPARATE)
                        {
                          sBrepFaces.Append(sFaces);
                          sBrepFacesCount.Add(sFaces.GetSize());
                        }

                      // relate and mark every coincident/compatible face pair
                      lNumFaces = sFaces.GetSize();
                      for(kk=0; kk<lNumFaces; kk++)
                        {
                          pFace  = sFaces[kk];
                          pOFace = sOFaces[kk];
                          SER(m_vTI.Relate(pFace,pOFace));
                          pFace->Mark(eBrepMarkType);
                          pOFace->Mark(eOtherMarkType);

#ifdef SM_DEBUG_CODE
                          if (bDebugMe)
                            {
                              smgfx_Erase();
                              smgfx_SetLook(1,2, 0,0,1) ; m_vTI.m_pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
                              smgfx_SetLook(1,2, 0,1,0) ; m_vTI.m_pOther->Draw(TRUE) ; sm_GraphicsLoop() ;
                              smgfx_SetLook(3,4, 0,1,1) ; m_vTI.Draw(3,5, 7,9) ; sm_GraphicsLoop() ;
                              smgfx_SetLook(1,2, 0,1,1) ; pFace->Draw(SM_DM_CROSSHATCH); sm_GraphicsLoop();
                              smgfx_SetLook(1,2, 1,1,0) ; pOFace->Draw(SM_DM_CROSSHATCH); sm_GraphicsLoop();
                              sm_GraphicsLoop();
                            }
#endif // SM_DEBUG_CODE

                          // update Keep and Delete lists based on operation type
                          switch (eOperation)
                            {
                              case SM_BO_UNION:
                              case SM_BO_INTERSECTION:
                                  if (bOnSameSide || bNonSolid) { sKeepFaces.Add(pFace); sKeepOFaces.Add(pOFace); }
                                  else                          { sDelFaces.Add(pFace); sDelOFaces.Add(pOFace); }
                                  break;
                              case SM_BO_DIFFERENCE:
                              case SM_BO_SLICE:
                                  if (bOnSameSide && !bNonSolid) { sDelFaces.Add(pFace); sDelOFaces.Add(pOFace); }
                                  else                           { sKeepFaces.Add(pFace); sKeepOFaces.Add(pOFace); }
                                  break;
                              case SM_BO_EXCLUSIVE_OR:
                                  sDelFaces.Add(pFace); sDelOFaces.Add(pOFace);
                                  break;
                              case SM_BO_PARTIAL_MERGE:
                              case SM_BO_MERGE:
                                  sKeepFaces.Add(pFace); sKeepOFaces.Add(pOFace);
                                  break;
                              case SM_BO_UNKNOWN:
                              case SM_BO_EXTRACT_SEPARATE:
                              case SM_BO_IMPRINT:
                              case SM_BO_IMPRINT_CLASSIFY:
                                  break;
                           } // end switch on operation
                        } // end iter every (after merge) coincident face
                    } // end EU classified to a coincident face branch
                  else if (pFoundEU)
                    {
                      // else OEU classified to a containing radial sector branch
                      SmTopologyTraverser sTraverser;
                      SmFace *pFace = pOEU->GetFace();

                      // collect faces attached to pFace that are not marked
                      // without going through marked edges
                      SER(sTraverser.CollectFaces(pFace,           // in : Seed face (gets marked)
                                                  sCollectedFaces,  // out: List of connected faces (Get marked)
                                                  eOtherMarkType)); // in : specify mark for target objects (not incremented)
#ifdef SM_DEBUG_CODE
                      if (bDebugMe)
                        {
                          smgfx_Erase();
                          smgfx_SetLook(1,2, 0,0,1) ; m_vTI.m_pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
                          smgfx_SetLook(1,2, 0,1,0) ; m_vTI.m_pOther->Draw(TRUE) ; sm_GraphicsLoop() ;
                          smgfx_SetLook(5,6, 0,0,0) ; pEdge->Draw(); sm_GraphicsLoop() ;
                          smgfx_SetLook(7,8, 1,0,1) ; pOEU->Draw() ; sm_GraphicsLoop();
                          smgfx_SetLook(7,8, 1,1,0) ; pFoundEU->Draw(); sm_GraphicsLoop() ;
                          for(di=0; di<sCollectedFaces.GetSize(); di++)
                            { smgfx_SetLook(1,2, 0,1,1) ; sCollectedFaces[di]->Draw(SM_DM_CROSSHATCH); sm_GraphicsLoop();
                            }
                          sm_GraphicsLoop() ;
                        }
#endif // SM_DEBUG_CODE

                      // when separating - place collected faces onto sOtherFaces lists
                      if (eOperation == SM_BO_EXTRACT_SEPARATE)
                        {
                          sOtherFaces.Append(sCollectedFaces);
                          sOtherFacesCount.Add(sCollectedFaces.GetSize());
                        }

                      // get classified radial sector's region
                      SmShell  *pShell  = pFoundEU->GetShell();
                      SmRegion *pRegion = pShell->GetRegion();

                      // update Keep and Delete lists based on operation type:
                      switch (eOperation)
                        {
                          case SM_BO_UNION:
                              if (pRegion->IsVoid()) { sKeepOFaces.Append(sCollectedFaces); }
                              else                   { sDelOFaces.Append(sCollectedFaces); }
                              break;
                          case SM_BO_DIFFERENCE:
                          case SM_BO_INTERSECTION:
                          case SM_BO_SLICE:
                              if (pRegion->IsVoid()) { sDelOFaces.Append(sCollectedFaces); }
                              else                   { sKeepOFaces.Append(sCollectedFaces); }
                              break;
                          case SM_BO_EXCLUSIVE_OR:
                          case SM_BO_PARTIAL_MERGE:
                          case SM_BO_MERGE:
                              sKeepOFaces.Append(sCollectedFaces);
                              break;
                          case SM_BO_UNKNOWN:
                          case SM_BO_EXTRACT_SEPARATE:
                          case SM_BO_IMPRINT:
                          case SM_BO_IMPRINT_CLASSIFY:
                              break;
                        }
                    } // end OEU classified to a radial sector branch

                  // any pOtherEdge->Edgeuse->Face that fails to classify
                  // against the matched ThisEdge is an error.
                  else SER(SM_ERR);
                } // end iter every OtherEdge->Edgeuse->Face

              // if topology graph was modified - break for another big loop iteration
              if (!bDone) break;

            } // end iter every common edge
        } // end while not bDone
          //     merging coincident faces,
          //     fixing intersection loop gaps, and
          //      assigning Faces, connected to CommonEdge->Faces, to Keep and Delete lists
    } // end STEP 2

  // arrive here when all common edges and vertices are marked
  //        - faces to be ignored by SM_BO_PARTIAL_MERGE are marked
  //        - faces classified into Keep or Delete lists are marked
  //        - intersection loop gaps have been fixed
  //        - coincident faces have been made compatible
  //        - coincident facepairs have been added to m_vTI->PtrToPtr maps
  //        - every commonEdge->Face has been classified against its partnerEdge->Sectors and
  //           assigned to either the Keep or Delete lists

#ifdef SM_DEBUG_CODE
  // Draw keep and delete faces with current Breps
  if (bDebugMe)
    {
      SM_DUMP_AND_ASSERT_VALID(m_vTI.m_pBrep);
      SM_DUMP_AND_ASSERT_VALID(m_vTI.m_pOther) ;
      m_vTI.Dump() ;

      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0,0,1) ; m_vTI.m_pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(2,3, 0,1,0) ; m_vTI.m_pOther->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(3,4, 0,1,1) ; m_vTI.Draw(3,5, 7,9) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,1) ;
      for(di=0; di<sKeepFaces.GetSize(); di++)
        { sKeepFaces[di]->Draw(SM_DM_CROSSHATCH); sm_GraphicsLoop();
          sm_GraphicsLoop();
        }
      smgfx_SetLook(1,2, 0,0,1) ;
      for(di=0; di<sKeepOFaces.GetSize(); di++)
        { sKeepOFaces[di]->Draw(SM_DM_CROSSHATCH); sm_GraphicsLoop();
          sm_GraphicsLoop();
        }
      smgfx_SetLook(2,3, 1,0,0) ;
      for(di=0; di<sDelFaces.GetSize(); di++)
        { sDelFaces[di]->Draw(SM_DM_CROSSHATCH); sm_GraphicsLoop();
          sm_GraphicsLoop();
        }
      smgfx_SetLook(2,3, 1,0,1) ;
      for(di=0; di<sDelOFaces.GetSize(); di++)
        { sDelOFaces[di]->Draw(SM_DM_CROSSHATCH); sm_GraphicsLoop();
          sm_GraphicsLoop();
        }
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

  // STEP 3. For every remaining unmarked face in pBrep and pOtherBrep
  //     (these are faces not connected to faces which are connected to common edges)
  //   - determine which region contains it in the other Brep
  //   - decide to keep or delete the face based on the operation.
  // Operation:
  //   partial_merge/merge: add faces to KeepFaces
  //   separate           : add faces to BrepFaces
  //   union/difference   : if(region==Void) KeepFaces
  //                        else             DeleteFaces
  //   intersection       : if(region==Void) DeleteFaces
  //                        else             KeepFaces
  //   partial_merge/merge: KeepFaces
  //   exclusive_or       : KeepFaces
    {
      SmTArray<SmEdge*> sFacesEdges;
      SmTArray<SmFace*> sBrepFaces2;
      // No longer used [BD 060718]
      //SmTArray<SmFace*> sShellFaces; // list of all OBrep->Faces that may split
                                       // a Brep->Region when added to Brep

      // 3A. for every reminaing unmarked Brep->Face - decide to Keep or Delete it
      m_vTI.m_pBrep->GetFaces(sBrepFaces2);
      lCount = sBrepFaces2.GetSize();
      for(ULONG ii=0; ii<lCount; ii++)
        {
          SmFace *pFace = sBrepFaces2[ii];

          // skip marked faces - they've already been placed on the Keep or Delete list
          if (pFace->IsMarked(eBrepMarkType)) { continue ; }

          // at this time all common edges, vertices, and commonEdge->faces are marked

          // for merge operations - keep the face - move to next face
          if (   eOperation == SM_BO_EXCLUSIVE_OR
              || eOperation == SM_BO_PARTIAL_MERGE
              || eOperation == SM_BO_MERGE)
            {
              sKeepFaces.Add(pFace);
              continue;
            }

          // get all unmarked face's connected to face not stepping through marked edges
          SmTopologyTraverser sTraverser;
          SER(sTraverser.CollectFaces(pFace,           // in : Seed face (gets marked)
                                      sCollectedFaces, // out: List of connected faces (Get marked)
                                      eBrepMarkType));   // in : specify mark for target objects (not incremented)

#ifdef SM_DEBUG_CODE
          if (bDebugMe)
            {
              smgfx_Erase() ;
              smgfx_SetLook(1,2, 0,0,1) ; m_vTI.m_pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
              smgfx_SetLook(2,3, 0,1,0) ; m_vTI.m_pOther->Draw(TRUE) ; sm_GraphicsLoop() ;
              smgfx_SetLook(3,4, 1,0,0) ; pFace->Draw() ; sm_GraphicsLoop() ;
              for ( di=1; di<sCollectedFaces.GetSize(); di++ )
              {
                  smgfx_ChangeColor(ii != 0);
                  sCollectedFaces[di]->Draw(); sm_GraphicsLoop();
                  sm_GraphicsLoop();
              }
              sm_GraphicsLoop();
            }
#endif // SM_DEBUG_CODE

          // for separate operations - add faces to sBrepFaces list - move to next face
          if (eOperation == SM_BO_EXTRACT_SEPARATE)
            {
              sBrepFaces.Append(sCollectedFaces);
              sBrepFacesCount.Add(sCollectedFaces.GetSize());
              continue;
            }

          // get all face edges
          pFace->GetEdges(sFaceEdges);
          SM_ASSERT(sFaceEdges.GetSize() > 0);

          // Classify the Other face by classifying a point from its edges.
          // - at this time all edges of all unmarked faces will NOT be common edges
          SmPoint3d sPnt;
          SmRegion *pRegion = NULL;

          // for every OtherFace->Edge
          ULONG lNumFEs = sFaceEdges.GetSize();
          for(ULONG jjj=0; jjj<lNumFEs; jjj++)
            {
              SmEdge    * pEdge          = sFaceEdges[0];
              SmZoneTol3d sEdgeZoneTol3d = SmTol::GetZoneTol3d(pEdge) ;
              SER(pEdge->GetCurve()->EvaluatePoint(pEdge->GetInterval().Evaluate(0.45678),sPnt));

              // classify the Point (and hence the edge, and the face, and the whole set of collected faces)
              // against the other Brep
              SmPointClassification sPointClass(sEdgeZoneTol3d, pOtherContext) ;  // should be sPnt ZoneTol3d
                {
                  SmTemporaryChangeValue<SmBoolean> sChange(m_vTI.m_pOther->m_bEditingEnabled,FALSE);
                  if (m_vTI.m_pOther->Point3DClassify(sPnt,SM_EFF_ZERO,FALSE,sPointClass) != SM_SUCCESS )
                    { continue; }
                }

              // check state: Edge should map to a region.
              //              All edges that map to otherBrep Faces, Edges, and Vertices
              //              should have been detected, related, labeled as common, and marked
              // done once we get a valid classification
              pRegion = SM_CAST_PTR( SmRegion, sPointClass.GetObject() );

              if ( pRegion != NULL )
                { break; } // success

            } // end for each Edge of Face.

            // note: faces that don't classify to regions are already marked as an error
            if ( pRegion == NULL )
              { continue; }

#ifdef SM_DEBUG_CODE
          // Draw classify point and found region
              if (bDebugMe)
                {
                  smgfx_SetLook(3,4, 1,0,0) ; sPnt.Draw() ; sm_GraphicsLoop();
                  smgfx_SetLook(2,3, 0,1,0) ; pRegion->Draw() ; sm_GraphicsLoop() ;
                  sm_GraphicsLoop();
                }
#endif // SM_DEBUG_CODE

          // classify the point (and hence its face) based on its region and operation type
          switch (eOperation)
            {
              case SM_BO_UNION:
              case SM_BO_DIFFERENCE:
              case SM_BO_SLICE:
                  if (pRegion->IsVoid()) {sKeepFaces.Append(sCollectedFaces);}
                  else                   { sDelFaces.Append(sCollectedFaces);}
                  break;
              case SM_BO_INTERSECTION:
                  if (pRegion->IsVoid()) { sDelFaces.Append(sCollectedFaces);}
                  else                   {sKeepFaces.Append(sCollectedFaces);}
                  break;
              case SM_BO_EXCLUSIVE_OR:
              case SM_BO_PARTIAL_MERGE:
              case SM_BO_MERGE:
                  sKeepFaces.Append(sCollectedFaces);
                  break;
              case SM_BO_UNKNOWN:
              case SM_BO_EXTRACT_SEPARATE:
              case SM_BO_IMPRINT:
              case SM_BO_IMPRINT_CLASSIFY:
                  break;
            } // end switch on operation
        } // end iter every Brep->Face placing it on the Keep or Delete list

#ifdef SM_DEBUG_CODE
      if (bDebugMe)
        {
          SM_DUMP_AND_ASSERT_VALID(m_vTI.m_pBrep);
          SM_DUMP_AND_ASSERT_VALID(m_vTI.m_pOther);
          m_vTI.Dump() ;

          smgfx_Erase() ;
          smgfx_SetLook(1,2, 0,0,1) ; m_vTI.m_pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
          smgfx_SetLook(1,2, 0,1,0) ; m_vTI.m_pOther->Draw(TRUE) ; sm_GraphicsLoop() ;
          smgfx_SetLook(3,4, 0,1,1) ; m_vTI.Draw(3,5, 7,9) ; sm_GraphicsLoop() ;
          smgfx_SetLook(1,2, 0,1,1) ;
          for(di=0; di<sKeepFaces.GetSize(); di++)
            { sKeepFaces[di]->Draw(SM_DM_CROSSHATCH); sm_GraphicsLoop();
              sm_GraphicsLoop();
            }
          smgfx_SetLook(1,2, 1,1,0) ;
          for(di=0; di<sKeepOFaces.GetSize(); di++)
            { sKeepOFaces[di]->Draw(SM_DM_CROSSHATCH); sm_GraphicsLoop();
              sm_GraphicsLoop();
            }
          smgfx_SetLook(2,3, 1,0,0) ;
          for(di=0; di<sDelFaces.GetSize(); di++)
            { sDelFaces[di]->Draw(SM_DM_CROSSHATCH); sm_GraphicsLoop();
              sm_GraphicsLoop();
            }
          smgfx_SetLook(2,3, 1,0,1) ;
          for(di=0; di<sDelOFaces.GetSize(); di++)
            { sDelOFaces[di]->Draw(SM_DM_CROSSHATCH); sm_GraphicsLoop();
              sm_GraphicsLoop();
            }
          sm_GraphicsLoop();

        }
#endif // SM_DEBUG_CODE

      // arrive here after adding all unmarked Brep->Faces to Keep and Delete lists

      // 3B. for every unmarked OtherBrep->Face - decide to Keep or Delete it
      //     (These are faces not connected to faces which connect to common edges)
      m_vTI.m_pOther->GetFaces(sBrepFaces2);
      lCount = sBrepFaces2.GetSize();
      for(ULONG ii=0; ii<lCount; ii++)
        {
          SmFace *pFace = sBrepFaces2[ii];

          // skip marked faces - they've already been placed on the Keep or Delete list
          if (pFace->IsMarked(eOtherMarkType)) { continue ; }

          // at this time all common edges, vertices,
          // commonEdge->faces and faces connected to those faces are marked

          // get all face edges
          pFace->GetEdges(sFaceEdges);
          SM_ASSERT(sFaceEdges.GetSize() > 0);

          // classify the Other face by classifying a point from one of its edges
          SmPoint3d sPnt;
          SmRegion *pRegion = NULL;

          // for every OtherFace->Edge
          ULONG lNumFEs = sFaceEdges.GetSize();
          for(ULONG jjj=0; jjj<lNumFEs; jjj++)
            {
              // classify the Other face by classifying a point from one of its edges
              SmEdge    * pEdge          = sFaceEdges[jjj];
              SmZoneTol3d sEdgeZoneTol3d = SmTol::GetZoneTol3d(pEdge) ;
              SER(pEdge->GetCurve()->EvaluatePoint(pEdge->GetInterval().Evaluate(0.456789),sPnt));
              SmPointClassification sPointClass(sEdgeZoneTol3d, pBrepContext); // should be sPnt ZoneTol3d
              {
                  SmTemporaryChangeValue<SmBoolean> sChange(m_vTI.m_pBrep->m_bEditingEnabled,FALSE);
                  if (m_vTI.m_pBrep->Point3DClassify(sPnt,SM_EFF_ZERO,FALSE,sPointClass) != SM_SUCCESS )
                    { continue; }
              }

              // check state: Edge should map to a region.
              //              All edges that map to otherBrep Faces, Edges, and Vertices
              //              should have been detected, related, labeled as common, and marked
              if (sPointClass.GetPointClass() != SM_PC_REGION)
                { continue; } // Try another Edge.

              // done once we get a valid classification
              pRegion = SM_CAST_PTR( SmRegion, sPointClass.GetObject() );
              if ( pRegion )
                { break; } // success

            } // end iter every OtherFace->Edge

          // note: faces that don't classify to regions are already marked as an error
          if ( !pRegion )
            { continue; } // Go on to the next Face.

          // add OtherFace to ThisBrep ShellFaces list
          // sShellFaces.Add(pFace);

          // get all unmarked face's connected to face not stepping through marked edges
          SmTopologyTraverser sTraverser;
          SER(sTraverser.CollectFaces(pFace,             // in : Seed face (gets marked)
                                      sCollectedFaces,   // out: List of connected faces (Get marked)
                                      eOtherMarkType));  // in : specify mark for target objects (not incremented)

          // for separate operations - add faces to sOtherFaces list - move to next face
          if (eOperation == SM_BO_EXTRACT_SEPARATE)
            {
              sOtherFaces.Append(sCollectedFaces);
              sOtherFacesCount.Add(sCollectedFaces.GetSize());
            }

          // classify the point (and hence its face) based on its region and operation type
          switch (eOperation)
            {
              case SM_BO_UNION:
                  if (pRegion->IsVoid()) { sKeepOFaces.Append(sCollectedFaces); }
                  else  { sDelOFaces.Append(sCollectedFaces); }
                  break;
              case SM_BO_DIFFERENCE:
              case SM_BO_INTERSECTION:
              case SM_BO_SLICE:
                  if (pRegion->IsVoid()) { sDelOFaces.Append(sCollectedFaces); }
                  else                   { sKeepOFaces.Append(sCollectedFaces); }
                  break;
              case SM_BO_EXCLUSIVE_OR:
              case SM_BO_PARTIAL_MERGE:
              case SM_BO_MERGE:
                  sKeepOFaces.Append(sCollectedFaces);
                  break;
              case SM_BO_UNKNOWN:
              case SM_BO_EXTRACT_SEPARATE:
              case SM_BO_IMPRINT:
              case SM_BO_IMPRINT_CLASSIFY:
                  break;
            }
        } // end iter every unmarked OtherBrep->Face placing it on the Keep or Delete list

      // arrive here when
      //  - all common edges and vertices are marked
      //  - all Brep->Faces and Other->Faces have been marked
      //  - intersection loop gaps have been fixed
      //  - coincident faces have been made compatible
      //  - all faces classified and placed into Keep or Delete lists

      // the input objects are imprinted - but still intact.

      // some classify confusions lead to confused keepLists and delLists - check and correct that problem.
      // Note, it's different for EXTRACT_SEPARATE though.  [B486]
      if ( eOperation != SM_BO_EXTRACT_SEPARATE )
        {
          // we expect that the keepLists and delLists are confusion free at this point,
          //   but check and correct, just in case.

          // protect from errors - when the keep and delete lists are confused, i.e.
          //    - duplicate entries on one list (perhaps caused by a confused IntersectInsertRelate() result)
          //    - a face in the input breps that is not on the keep or delete lists
          //    - a face on the keep or delete lists that is not in the input lists
          SmBoolean bDuplicates  = sKeepFaces.RemoveDuplicates() ;
          bDuplicates           |= sDelFaces.RemoveDuplicates() ;
          bDuplicates           |= sKeepOFaces.RemoveDuplicates() ;
          bDuplicates           |= sDelOFaces.RemoveDuplicates() ;
          SmTArray<SmFace *> sTmpBrepFaces, sTmpOtherFaces ;
          m_vTI.m_pBrep->GetFaces(sTmpBrepFaces) ;
          m_vTI.m_pOther->GetFaces(sTmpOtherFaces) ;
          long lBrepClassifyCnt = (long)sTmpBrepFaces.GetSize()
                                  - (long) sKeepFaces.GetSize()
                                  - (long) sDelFaces .GetSize() ;
          long lOtherClassifyCnt = (long)sTmpOtherFaces.GetSize()
                                  - (long) sKeepOFaces.GetSize()
                                  - (long) sDelOFaces .GetSize() ;

          // when the BrepClassify Face count is off (not zero)
          if(lBrepClassifyCnt != 0)
            {
              // SM_ASSERT(lBrepClassifyCnt < 0) ;

              // remove entries from sKeepFaces and sDelFaces not in sTmpBrepFaces
              SmTArray<SmFace*> sStaleFaces ;

              // cull sKeepFaces
              sKeepFaces.FindUniqueElements(sTmpBrepFaces, sStaleFaces) ;
              SM_ASSERT_MSG(sStaleFaces.GetSize() == 0, _T("BooleanWarn - stale DelFace/KeepFace entries")) ;
              if(sStaleFaces.GetSize() > 0)
                { sKeepFaces.RemoveElements(sStaleFaces, sKeepFaces) ; }

              // cull sDelFaces
              sDelFaces.FindUniqueElements(sTmpBrepFaces, sStaleFaces) ;
              SM_ASSERT_MSG(sStaleFaces.GetSize() == 0, _T("BooleanWarn - stale DelFace/KeepFace entries")) ;
              if(sStaleFaces.GetSize() > 0)
                { sDelFaces.RemoveElements(sStaleFaces, sDelFaces) ; }
            } // end lBrepClassifyCnt != 0 check

          // when the lOtherClassifyCnt Face count is off (not zero)
          if(lOtherClassifyCnt != 0 && eOperation != SM_BO_PARTIAL_MERGE)
            {
              // SM_ASSERT(lOtherClassifyCnt < 0) ;

              // remove entries from sKeepFaces and sDelFaces not in sTmpBrepFaces
              SmTArray<SmFace*> sStaleFaces ;

              // cull sKeepOFaces
              sKeepOFaces.FindUniqueElements(sTmpOtherFaces, sStaleFaces) ;
              SM_ASSERT_MSG(sStaleFaces.GetSize() == 0, _T("BooleanWarn - stale DelOFace/KeepOFace entries")) ;
              if(sStaleFaces.GetSize() > 0)
                { sKeepOFaces.RemoveElements(sStaleFaces, sKeepOFaces) ; }

              // cull sDelOFaces
              sDelOFaces.FindUniqueElements(sTmpOtherFaces, sStaleFaces) ;
              SM_ASSERT_MSG(sStaleFaces.GetSize() == 0, _T("BooleanWarn - stale DelOFace/KeepOFace entries")) ;
              if(sStaleFaces.GetSize() > 0)
                { sDelOFaces.RemoveElements(sStaleFaces, sDelOFaces) ; }
            } // end lOtherClassifyCnt != 0 check

          SmBoolean bConfused =    bDuplicates
                                || lBrepClassifyCnt != 0
                                || (lOtherClassifyCnt != 0 && eOperation != SM_BO_PARTIAL_MERGE) ;

          SM_ASSERT_MSG(bDuplicates == FALSE, _T("BooleanWarn - duplicates DelFace/KeepFace entries")) ;
          SM_ASSERT_MSG( 0 == (  (long)sTmpBrepFaces.GetSize()
                               - (long) sKeepFaces.GetSize()
                               - (long) sDelFaces .GetSize()), _T("BooleanWarn - BrepFaces not in DelFace/KeepFace lists")) ;
          SM_ASSERT_MSG( 0 == (  (long)sTmpOtherFaces.GetSize()
                               - (long) sKeepOFaces.GetSize()
                               - (long) sDelOFaces .GetSize())
                        || eOperation == SM_BO_PARTIAL_MERGE, _T("BooleanWarn - OtherFaces not in DelOFace/KeepOFace lists")) ;
          // GWC: letting a bConfused case complete, after removing duplicate and stale entries
          //      allows the boolean operation to succeed in some cases.
          //      Output an error notice and continue
          if(bConfused)
            { SE_MSG(SM_ERR,_T("confused keep_list or del_list detected and corrected - continuing")) ; }

        } // end scope: keepList and delList review and correct section

      // If we are classifying faces, use attributes to label each face.
      // Exit gracefully with input objects intact but with modified topology.
      if ( eOperation == SM_BO_IMPRINT_CLASSIFY || m_bImprintAndClassifyFaces )
        {
          ULONG fi ; 
          ULONG lNumDelFaces  = sDelFaces.GetSize() ;
          ULONG lNumSaveFaces = sKeepFaces.GetSize() ;
          // ULONG lNumKeepWires = sKeepAsWires.GetSize() ;

          // make intersect attribute as needed
          SmLongAttribute *pIntersectAttribute = NULL ;
          if(   m_sIntersectionEdges.GetSize() > 0 
             || m_sIntersectionVertices.GetSize() > 0)
            {
              pIntersectAttribute = new (*pBrepContext) SmLongAttribute( SM_AI_BOOLEAN_INTERSECT, 
                                                                         1, 
                                                                         SM_AB_REFERENCE ) ;
            }

          // label IntersectionEdges on object A
          for( fi=0; fi<m_sIntersectionEdges.GetSize(); fi++ ) 
            {
              if( m_sIntersectionEdges[fi]->FindAttribute( SM_AI_BOOLEAN_INTERSECT ) == NULL )
                { m_sIntersectionEdges[fi]->AddAttribute( pIntersectAttribute ) ; }
            }

          // label InersectionVertices on object A
          for( fi=0; fi<m_sIntersectionVertices.GetSize(); fi++ ) 
            {
              if( m_sIntersectionVertices[fi]->FindAttribute( SM_AI_BOOLEAN_INTERSECT ) == NULL )
                { m_sIntersectionVertices[fi]->AddAttribute( pIntersectAttribute ) ; }
            }

          // label the delete faces on object A
          if ( lNumDelFaces > 0 )
            {
              SmLongAttribute *pDeleteAttribute = new (*pBrepContext) SmLongAttribute( SM_AI_BOOLEAN_DELETE, 
                                                                                       1, 
                                                                                       SM_AB_REFERENCE ) ;
              for( fi=0; fi<lNumDelFaces; fi++ )
                {
                  if( sDelFaces[fi]->FindAttribute( SM_AI_BOOLEAN_DELETE ) == NULL )
                    { sDelFaces[fi]->AddAttribute( pDeleteAttribute ); }
                }
            }

          // label the keep faces on object A
          if ( lNumSaveFaces > 0 )
            {
              SmLongAttribute *pSaveAttribute = new (*pBrepContext) SmLongAttribute( SM_AI_BOOLEAN_SAVE, 
                                                                                     1, 
                                                                                     SM_AB_REFERENCE ) ;
              for( fi=0; fi<lNumSaveFaces; fi++ ) 
                {
                  if( sKeepFaces[fi]->FindAttribute( SM_AI_BOOLEAN_SAVE ) == NULL )
                    { sKeepFaces[fi]->AddAttribute( pSaveAttribute ) ; }
                }
            }

          // label the delete faces on object B
          lNumDelFaces = sDelOFaces.GetSize();
          if ( lNumDelFaces > 0 )
            {
              SmLongAttribute *pDeleteOAttribute = new (*pOtherContext) SmLongAttribute( SM_AI_BOOLEAN_DELETE, 1, SM_AB_REFERENCE );

              for( fi=0; fi<lNumDelFaces; fi++ )
                {
                  if( sDelOFaces[fi]->FindAttribute( SM_AI_BOOLEAN_DELETE ) == NULL )
                    { sDelOFaces[fi]->AddAttribute( pDeleteOAttribute ); }
                }
            }

          // label the keep faces on object B
          lNumSaveFaces = sKeepOFaces.GetSize();
          if ( lNumSaveFaces > 0 )
            {
              SmLongAttribute *pSaveOAttribute = new (*pBrepContext)
                  SmLongAttribute( SM_AI_BOOLEAN_SAVE, 1, SM_AB_REFERENCE );

              for( fi=0; fi<lNumSaveFaces; fi++ ) 
                {
                  if( sKeepOFaces[fi]->FindAttribute( SM_AI_BOOLEAN_SAVE ) == NULL )
                    { sKeepOFaces[fi]->AddAttribute( pSaveOAttribute ); }
                }
            }

          // exit gracefully
          sStack2.Clear();

          // restore state
          pBrepContext->SetDoingBoolean(FALSE);
          pOtherContext->SetDoingBoolean(FALSE);

          // set output value although input breps remain intact
          rpResult = m_vTI.m_pBrep;

          // all done
          return SM_SUCCESS ;
        } // end eOperation == SM_BO_IMPRINT_CLASSIFY || m_bImprintAndClassifyFaces
    } // end STEP 3

#ifdef SM_DEBUG_CODE
  // draw
  if (bDebugMe)
    {
      SM_DUMP_AND_ASSERT_VALID(m_vTI.m_pBrep);
      SM_DUMP_AND_ASSERT_VALID(m_vTI.m_pOther);
      m_vTI.Dump() ;

      //      // check that every brep face will be deleted or kept just once
      //      SmTArray<SmFace *> sTmpBrepFaces, sTmpOtherFaces ;
      //      m_vTI.m_pBrep->GetFaces(sTmpBrepFaces) ;
      //      m_vTI.m_pOther->GetFaces(sTmpOtherFaces) ;
      //
      //      SmTArray<SmFace *> sTmpKeepFaces(sKeepFaces),   sKeepDuplicates ;
      //      SmTArray<SmFace *> sTmpDelFaces(sDelFaces),     sDelDuplicates ;
      //      SmTArray<SmFace *> sTmpKeepOFaces(sKeepOFaces), sKeepODuplicates ;
      //      SmTArray<SmFace *> sTmpDelOFaces(sDelOFaces),   sDelODuplicates ;
      //
      //      // check for confusion - Cnts should both be zero
      //      long lBrepClassifyCnt = (long)sTmpBrepFaces.GetSize()
      //                              - (long) sKeepFaces.GetSize()
      //                              - (long) sDelFaces .GetSize() ;
      //      long lOtherClassifyCnt = (long)sTmpOtherFaces.GetSize()
      //                              - (long) sKeepOFaces.GetSize()
      //                              - (long) sDelOFaces .GetSize() ;
      //
      //      // check for duplicate entries - these arrays should all be empty
      //      sTmpKeepFaces .GetDuplicates( sKeepDuplicates ) ;
      //      sTmpDelFaces  .GetDuplicates(  sDelDuplicates ) ;
      //      sTmpKeepOFaces.GetDuplicates(  sKeepODuplicates ) ;
      //      sTmpDelOFaces .GetDuplicates(  sDelODuplicates ) ;
      //
      //      // every Keep/Del face is a current BrepFace - this should empty these arrays
      //      sTmpKeepFaces .RemoveElements(sTmpBrepFaces,  sTmpKeepFaces) ;
      //      sTmpDelFaces  .RemoveElements(sTmpBrepFaces,  sTmpDelFaces) ;
      //      sTmpKeepOFaces.RemoveElements(sTmpOtherFaces, sTmpKeepOFaces) ;
      //      sTmpDelOFaces .RemoveElements(sTmpOtherFaces, sTmpDelOFaces ) ;
      //
      //      // every Brep/other face is classified - this should empty these arrays
      //      sTmpBrepFaces.RemoveElements (sKeepFaces,   sTmpBrepFaces) ;
      //      sTmpBrepFaces.RemoveElements (sDelFaces,    sTmpBrepFaces) ;
      //      sTmpOtherFaces.RemoveElements(sKeepOFaces,  sTmpOtherFaces) ;
      //      sTmpOtherFaces.RemoveElements(sDelOFaces,   sTmpOtherFaces) ;

      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0,0,1) ; m_vTI.m_pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(2,3, 0,1,0) ; m_vTI.m_pOther->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(3,4, 0,1,1) ; m_vTI.Draw(3,5, 7,9) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,1) ; for(di=0; di<sKeepFaces.GetSize(); di++)
                                    { sKeepFaces[di]->Draw(SM_DM_CROSSHATCH); sm_GraphicsLoop();
                                    }
      smgfx_SetLook(1,2, 1,1,0) ; for(di=0; di<sKeepOFaces.GetSize(); di++)
                                    { sKeepOFaces[di]->Draw(SM_DM_CROSSHATCH); sm_GraphicsLoop();
                                    }
      smgfx_SetLook(2,3, 1,0,0) ; for(di=0; di<sDelFaces.GetSize(); di++)
                                    { sDelFaces[di]->Draw(SM_DM_CROSSHATCH); sm_GraphicsLoop();
                                    }
      smgfx_SetLook(2,3, 1,0,1) ; for(di=0; di<sDelOFaces.GetSize(); di++)
                                    { sDelOFaces[di]->Draw(SM_DM_CROSSHATCH); sm_GraphicsLoop();
                                    }
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

  // STEP 4: Remove Brep  WireEdges and ShellVerticies that need to go
  //         Merge  OBrep WireEdges and ShellVerticies that need to be added to Brep
    {
      if ( ! m_bCookieCutter )
        {
          if ( eOperation != SM_BO_PARTIAL_MERGE )
            {
              SER( ProcessWiresAndShellVertices( eOperation,
                                                 sKeepAsWires,
                                                 eBrepMarkType,
                                                 eOtherMarkType ));  // checks object Marks
            }                                                        // does not increment any mark values
        }
    } // end STEP 4

  // (Is algorithm done with Marks here? gwc: probably, marks are used to build the Keep and Delete lists)
#ifdef SM_DEBUG_CODE
  if (bDebugMe)
    {
      SM_DUMP_AND_ASSERT_VALID(m_vTI.m_pBrep);
      SM_DUMP_AND_ASSERT_VALID(m_vTI.m_pOther);
      m_vTI.Dump() ;

      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0,0,1) ; m_vTI.m_pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,0) ; m_vTI.m_pOther->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(3,4, 0,1,1) ; m_vTI.Draw(3,5, 7,9) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,1) ;
      for(di=0; di<sKeepFaces.GetSize(); di++)
                                    { sKeepFaces[di]->Draw(SM_DM_CROSSHATCH); sm_GraphicsLoop();
                                    }
      smgfx_SetLook(1,2, 1,1,0) ; for(di=0; di<sKeepOFaces.GetSize(); di++)
                                    { sKeepOFaces[di]->Draw(SM_DM_CROSSHATCH); sm_GraphicsLoop();
                                    }
      smgfx_SetLook(2,3, 1,0,0) ; for(di=0; di<sDelFaces.GetSize(); di++)
                                    { sDelFaces[di]->Draw(SM_DM_CROSSHATCH); sm_GraphicsLoop();
                                    }
      smgfx_SetLook(2,3, 1,0,1) ; for(di=0; di<sDelOFaces.GetSize(); di++)
                                    { sDelOFaces[di]->Draw(SM_DM_CROSSHATCH); sm_GraphicsLoop();
                                    }
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE
 
  // locals set in STEP 5 and used later
  SmTArray<SmRegion*> sBrepRegions;
  SmBoolean bInfiniteRegionCurrent = TRUE ;

  // STEP 5: Make Faces in the Brep From the OtherBrep KeepOFaces if we are
  //         not in cookie cutter mode.
    {
      if (   !m_bCookieCutter
          && eOperation != SM_BO_EXTRACT_SEPARATE)
        {
          // see if the new faces expand the extent of BrepCache->SpatialTrees
          SmBrepCache *pBrepCache = (SmBrepCache*)SmCacheMgr::GetObjectCache(SM_OC_BREP,m_vTI.m_pBrep);
          if(pBrepCache)
            {
              // save time - bump BrepCache->SpatialTree size up for all faces at once,
              //             rather than one at a time.
              pBrepCache->ReSizeTrees(&sKeepOFaces, NULL, NULL) ;

            } // end adding multiple wires to a Brep with a cache check

          // for every KeepOFace
          ULONG lll, lNumKOFs = sKeepOFaces.GetSize();
          for(lll=0; lll<lNumKOFs; lll++)
            {
              SmFace *pFace = sKeepOFaces[lll];

              // get the faceuses
              SmFaceuse *pFU1, *pFU2;
              pFace->GetFaceuses(pFU1,pFU2);

              // let pFU1 = faceuse connected to OtherBrep void or infinite region
              //     pFU2 = other faceuse (not void for manifold models)
              // Note: it doesn't matter which we pass in, because MakeFaceuse()
              // will set them as it chooses. [bd, Jan 07]
              //
              //  SmRegion *pRegionFU2 = pFU2->GetShell()->GetRegion();
              // if (   pRegionFU2 == m_vTI.m_pOther->GetInfiniteRegion()
              //     || pRegionFU2->IsVoid())
              //   {
              //     SmFaceuse *pFUTemp = pFU1;
              //     pFU1               = pFU2;
              //     pFU2               = pFUTemp;
              //   }

#ifdef SM_DEBUG_CODE
              // draw
              if (bDebugMe)
                {
                  SmTArray<SmLoopuse*> sLoopuses;
                  pFU1->GetLoopuses(sLoopuses);

                  smgfx_Erase();
                  smgfx_SetLook(1,2, 0,0,1) ; m_vTI.m_pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
                  smgfx_SetLook(1,2, 0,1,0) ; m_vTI.m_pOther->Draw(TRUE) ; sm_GraphicsLoop() ;
                  smgfx_SetLook(3,5, 1,0,0) ;
                  for(di=0;di<sLoopuses.GetSize();di++)
                                               { sLoopuses[di]->Draw(); sm_GraphicsLoop() ; }
                  smgfx_SetLook(1,2, 0,1,1) ; pFU1->Draw(SM_DM_CROSSHATCH); sm_GraphicsLoop() ;
                  sm_GraphicsLoop();
                }
#endif // SM_DEBUG_CODE

              // make the face in Brep based on the OBrep KeepFace
              //   side-effect: MakeFaceuse() can split a region into two,
              //       creating 1 new region and 1 new shell
              //
              // MakeFaceuse() has another side effect which we do not want.
              // If Brep's infinite region is being split, it calls
              // FindAndSetInfiniteRegion().
              // Before resetting the infinite region pointer in Brep,
              // it sets that region's void status to be whatever the
              // newly-found infinite region was.  At this incomplete stage,
              // that region could be an interior region of the other brep.
              // At any rate, for our purposes, we do not want the void status
              // of any region in Brep to be changed: we'll figure that out
              // when we have enough info to do the whole job properly.
              // [060718]

              // Find the region in Brep that Other's face is going into.
              // We'll need that to check whether its void/solid status was
              // switched, if MakeFaceuse() splits it.
              // Note, this can be an expensive call, but MakeFaceuse() will
              // do it anyway, if we don't pass it the region.
              SmRegion *pOrigRegion  = sm_FindBrepRegionOfOtherFace( pFU1, m_vTI ) ;
              SmBoolean bOrigRegVoid = (pOrigRegion != NULL) ? pOrigRegion->IsVoid() : FALSE ;

              SmRegion *pNewRegion=NULL, *pOldRegion=NULL ;
              SmTArray<SmFace*> NewFs;

              switch ( eOperation )
                {
                  case SM_BO_UNION:
                  case SM_BO_INTERSECTION:
                  case SM_BO_DIFFERENCE:
                  case SM_BO_PARTIAL_MERGE:
                  case SM_BO_MERGE:
                  case SM_BO_EXCLUSIVE_OR:
                  case SM_BO_SLICE:

                    // Make a face from the OtherBrep in m_vTI.m_pBrep
                    SER(MakeFaceuse(pFU1,        // in : OtherBrep face to add to Brep
                                    pOldRegion,  // out: Brep Region receiving NewFace
                                    pNewRegion,  // out: New Region made when adding NewFace splits OldRegion
                                    pOrigRegion, // in : Optional this Brep region known to contain input faceuse->face
                                    &NewFs));    // out: Newly constructed Face - used for LocalMerge, NULL to ignore
                    break;

                  case SM_BO_UNKNOWN:
                  case SM_BO_EXTRACT_SEPARATE:
                  case SM_BO_IMPRINT:
                  case SM_BO_IMPRINT_CLASSIFY:
                    break;
                } // end switch on operation

#ifdef SM_DEBUG_CODE
              if (bDebugMe)
                {
                  SM_DUMP_AND_ASSERT_VALID(m_vTI.m_pBrep) ;
                  SM_DUMP_AND_ASSERT_VALID(m_vTI.m_pOther) ;
                  SM_DUMP_AND_ASSERT_VALID(pOrigRegion) ;
                  if(pOrigRegion) pOrigRegion->DumpTopology();
                  if(pOldRegion) pOldRegion->DumpTopology();
                  if(pNewRegion) pNewRegion->DumpTopology();

                  smgfx_Erase();
                  smgfx_SetLook(1,2, 0,0,1); m_vTI.m_pBrep->Draw(TRUE); sm_GraphicsLoop();
                  smgfx_SetLook(2,3, 0,1,0); m_vTI.m_pOther->Draw(TRUE); sm_GraphicsLoop();

                  smgfx_SetLook(1,2, 0,1,1); pFU1->Draw(SM_DM_CROSSHATCH); sm_GraphicsLoop();
                  smgfx_SetLook(1,2, 1,0,1); if(pOrigRegion)pOrigRegion->Draw(); sm_GraphicsLoop();
                  smgfx_SetLook(1,2, 1,0,0); if(pOldRegion) pOldRegion->Draw(); sm_GraphicsLoop();
                  smgfx_SetLook(1,2, 0,1,0); if(pNewRegion) pNewRegion->Draw(); sm_GraphicsLoop();
                  sm_GraphicsLoop();
                }
#endif // SM_DEBUG_CODE

              // If a new region was created, MakeFaceuse() might reset the
              // void/solid status of the new region.  Also, we're not sure
              // whether pNewRegion or pOldRegion gets changed.
              // Generally, that's what we want, as when adding the sixth
              // face of a cube: the infinite region stays Void while the
              // internal region switches to Solid.  But there are times
              // when it shouldn't change, as in [060718], merging two manifold
              // bodies with internal voids.
              //
              // That switch is not really part of Boolean solid-modeling theory,
              // so there are no solid rules about when to do it.  For any rule,
              // some counterexample can be contrived.
              // Here is a heuristic that gives appropriate results for all
              // reasonable real-world cases: If a region was split and one of the
              // new regions comes back with a different void/solid status, then
              // allow the switch iff the face that split the region is nonmanifold.
              // If the face doing the splitting was manifold, then the subsequent
              // Boolean solid-modeling operation (below) will all work properly.
              // In that case, switch the region back to what it was.
              //
              // one more rule: when the inserted face is a sheet face (both its
              //   face uses bounded IsVoid regions) then any split region
              //   should have the same IsVoid state in both the original region
              //   and the new region.
              if (   pNewRegion  != NULL // a region was split...
                  && pOrigRegion != NULL // if not set, then neither is bOrigRegVoid...
                  && (    pNewRegion->IsVoid() != bOrigRegVoid
                       || pOldRegion->IsVoid() != bOrigRegVoid )
                 )
                {
                  SmRegion *pRegThatChanged =   pOldRegion->IsVoid() != bOrigRegVoid
                                              ? pOldRegion 
                                              : pNewRegion ;

                  // Now, the test.  See if the splitting face was manifold
                  // in the other Brep or a shell face.  If so, switch it back.

                  SmBoolean bOtherReg1IsVoid = pFU1->GetShell()->GetRegion()->IsVoid();
                  SmBoolean bOtherReg2IsVoid = pFU2->GetShell()->GetRegion()->IsVoid();
                  if(   bOtherReg1IsVoid != bOtherReg2IsVoid
                     || (   bOtherReg1IsVoid == TRUE
                         && bOtherReg2IsVoid == TRUE))
                    {
                      // Added face was manifold or a sheet: change it back.
                      pRegThatChanged->SetIsVoid( bOrigRegVoid );
                    }
                }

              // If a new region was created, associate new and old regions.
              // Can't use m_vTI.Relate() to associate them
              // because it's not 1-1 : multiple regions of the new Brep
              // can map back to the same region in Other.
              if ( pNewRegion != NULL  &&  NewFs.GetSize() > 0 )
                {
                  // Assume OtherFace and NewFace share same directions.

                  // map OtherRegion source for new ThisBrep->Region on NewFace Upward side
                  SmFaceuse * pThisFU       = NewFs[0]->GetUpwardFaceuse();
                  SmFaceuse * pOtherFU      = pFace->GetUpwardFaceuse();
                  SmRegion  * pThisRegion1  = pThisFU->GetShell()->GetRegion();
                  SmRegion  * pThisRegion2  = pThisFU ->GetMate()->GetShell()->GetRegion();
                  SmRegion  * pOtherRegion1 = pOtherFU->GetShell()->GetRegion();
                  SmRegion  * pOtherRegion2 = pOtherFU->GetMate()->GetShell()->GetRegion();
                  SM_ASSERT(pThisRegion1 == pOldRegion || pThisRegion1 == pNewRegion);
                  SM_ASSERT(pThisRegion2 == pOldRegion || pThisRegion2 == pNewRegion);

                  // gwc: for region attribute propagation - remember this region's OtherBrep source Region
                  SmMergeRegionAttribute * pMergeRegionAttrib1 = (SmMergeRegionAttribute *)pThisRegion1->FindAttribute(SM_AI_OTHER_REGION_ID) ;
                  SmMergeRegionAttribute * pMergeRegionAttrib2 = (SmMergeRegionAttribute *)pThisRegion2->FindAttribute(SM_AI_OTHER_REGION_ID) ;
                  SmMergeRegionAttribute * pOldRegionAttrib    = (pThisRegion1 == pOldRegion) ? pMergeRegionAttrib1 : pMergeRegionAttrib2 ;

#ifdef SM_DEBUG_CODE
                  if (bDebugMe)
                    {
                      TCHAR sBuff[SM_TBLOCK_SIZE];
                      smos_sprintf(sBuff, _T("%s"),_T("m_vTI.m_pBrep"));
                      m_vTI.m_pBrep ->DumpRegionsAndAttributes(sBuff) ;
                      smos_sprintf(sBuff, _T("%s"),_T("m_vTI.m_pOther"));
                      m_vTI.m_pOther->DumpRegionsAndAttributes(sBuff) ;

                      if(pMergeRegionAttrib1) pMergeRegionAttrib1->Dump() ;
                      if(pMergeRegionAttrib2) pMergeRegionAttrib2->Dump() ;
                    }
#endif // SM_DEBUG_CODE

                  // See if these New ThisBrep Region Children came from a parent ThisBrep InfiniteRegion
                  SmBoolean bOldInfiniteRegion =    (pOldRegionAttrib && pOldRegionAttrib->HasThisInfiniteRegion())
                                                 || (pOldRegion == m_vTI.m_pBrep->GetInfiniteRegion()) ;  // hack - depends on knowing that Split reuses parent region as one of the children
                  SmBoolean bThisInfiniteRegion =   (pMergeRegionAttrib1 && pMergeRegionAttrib1->HasThisInfiniteRegion())
                                                 || (pMergeRegionAttrib2 && pMergeRegionAttrib2->HasThisInfiniteRegion()) 
                                                 || (pThisRegion1 == m_vTI.m_pBrep->GetInfiniteRegion())    // hack - depends on knowing that Split reuses parent region as one of the children
                                                 || (pThisRegion2 == m_vTI.m_pBrep->GetInfiniteRegion()) ;  // hack - depends on knowing that Split reuses parent region as one of the children
                  SmBoolean bOtherInfiniteRegion1 = (pOtherRegion1 == m_vTI.m_pOther->GetInfiniteRegion()) ;
                  SmBoolean bOtherInfiniteRegion2 = (pOtherRegion2 == m_vTI.m_pOther->GetInfiniteRegion()) ;

                  ULONG lOldSourceType   = bOldInfiniteRegion  ? 2 : pOldRegion->IsVoid()   ? 1 : 0 ;
                  ULONG lThisSourceType1 = bThisInfiniteRegion ? 2 : pThisRegion1->IsVoid() ? 1 : 0 ;
                  ULONG lThisSourceType2 = bThisInfiniteRegion ? 2 : pThisRegion2->IsVoid() ? 1 : 0 ;

                  ULONG lOtherSourceType1 = bOtherInfiniteRegion1 ? 2 : pOtherRegion1->IsVoid() ? 1 : 0 ;
                  ULONG lOtherSourceType2 = bOtherInfiniteRegion2 ? 2 : pOtherRegion2->IsVoid() ? 1 : 0 ;

                  // Use temp Attrib on Child Region1 to remember OtherBrep Region source and Infinite Region ancestry
                  if(pMergeRegionAttrib1 == NULL)
                    {
                      SmMergeRegionAttribute *pMergeRegionAttrib = new (*pBrepContext) SmMergeRegionAttribute(pOldRegion,
                                                                                                              lOldSourceType,
                                                                                                              pOtherRegion1,
                                                                                                              lOtherSourceType1) ;
                      pThisRegion1->AddAttribute(pMergeRegionAttrib) ;
#ifdef SM_DEBUG_CODE
                      if (bDebugMe)
                        {
                          TCHAR sBuff[SM_TBLOCK_SIZE];
                          smos_sprintf(sBuff, _T("%s"),_T("m_vTI.m_pBrep"));
                          m_vTI.m_pBrep->DumpRegionsAndAttributes(sBuff) ;
                          if(pMergeRegionAttrib) pMergeRegionAttrib->Dump() ;
                        }
#endif // SM_DEBUG_CODE

                    }
                  else
                    {
#ifdef SM_DEBUG_CODE
                      if(   pMergeRegionAttrib1->GetThisRegionsSize()  != 1
                         || pMergeRegionAttrib1->GetOtherRegionsSize() != 1)
                        {
                          SM_ASSERT_MSG(   pMergeRegionAttrib1->GetThisRegionsSize()  == 1
                                        && pMergeRegionAttrib1->GetOtherRegionsSize() == 1,
                                        _T("MergeRegionAttirubte data assumption of just one ThisRegion during the split phase is broken.")) ;
                        }
#endif // SM_DEBUG_CODE
                      pMergeRegionAttrib1->SetRegions(0, pMergeRegionAttrib1->GetThisRegion(0), lThisSourceType1, pOtherRegion1, lOtherSourceType1) ;
#ifdef SM_DEBUG_CODE
                      if (bDebugMe)
                        {
                          TCHAR sBuff[SM_TBLOCK_SIZE];
                          smos_sprintf(sBuff,_T("%s"), _T("m_vTI.m_pBrep"));
                          m_vTI.m_pBrep->DumpRegionsAndAttributes(sBuff) ;
                          if(pMergeRegionAttrib1) pMergeRegionAttrib1->Dump() ;
                        }
#endif // SM_DEBUG_CODE

                    }

                  // map OtherRegion source for new ThisBrep->Region on NewFace downward side

                  // Use temp Attrib on Child Region2 to remember OtherBrep Region source and Infinite Region ancestry
                  if(pMergeRegionAttrib2 == NULL)
                    {
                      SmMergeRegionAttribute *pMergeRegionAttrib = new (*pBrepContext) SmMergeRegionAttribute(pOldRegion,
                                                                                                              lOldSourceType,
                                                                                                              pOtherRegion2,
                                                                                                              lOtherSourceType2) ;
                      pThisRegion2->AddAttribute(pMergeRegionAttrib) ;
#ifdef SM_DEBUG_CODE
                      if (bDebugMe)
                        {
                          TCHAR sBuff[SM_TBLOCK_SIZE];
                          smos_sprintf(sBuff,_T("%s"), _T("m_vTI.m_pBrep"));
                          m_vTI.m_pBrep->DumpRegionsAndAttributes(sBuff) ;
                          if(pMergeRegionAttrib) pMergeRegionAttrib->Dump() ;
                        }
#endif // SM_DEBUG_CODE
                    }
                  else
                    {
#ifdef SM_DEBUG_CODE
                      if(   pMergeRegionAttrib2->GetThisRegionsSize()  != 1
                         || pMergeRegionAttrib2->GetOtherRegionsSize() != 1)
                        {
                          SM_ASSERT_MSG(   pMergeRegionAttrib2->GetThisRegionsSize()  == 1
                                        && pMergeRegionAttrib2->GetOtherRegionsSize() == 1,
                                        _T("MergeRegionAttirubte data assumption of just one ThisRegion during the split phase is broken.")) ;
                        }
#endif // SM_DEBUG_CODE
                      pMergeRegionAttrib2->SetRegions(0, pMergeRegionAttrib2->GetThisRegion(0), lThisSourceType2, pOtherRegion2, lOtherSourceType2) ;
#ifdef SM_DEBUG_CODE
                      if (bDebugMe)
                        {
                          TCHAR sBuff[SM_TBLOCK_SIZE];
                          smos_sprintf(sBuff,_T("%s"), _T("m_vTI.m_pBrep"));
                          m_vTI.m_pBrep->DumpRegionsAndAttributes(sBuff) ;
                          if(pMergeRegionAttrib2) pMergeRegionAttrib2->Dump() ;
                        }
#endif // SM_DEBUG_CODE
                    }

#ifdef SM_DEBUG_CODE
                  if (bDebugMe)
                    {
                      smgfx_Erase();
                      smgfx_SetLook(1,2, 0,0,1); m_vTI.m_pBrep->Draw(TRUE); sm_GraphicsLoop();
                      smgfx_SetLook(1,2, 0,1,0); m_vTI.m_pOther->Draw(TRUE); sm_GraphicsLoop();

                      smgfx_SetLook(1,2, 0,1,1); pFU1->Draw(SM_DM_CROSSHATCH); sm_GraphicsLoop();
                      smgfx_SetLook(1,2, 1,1,0); pThisRegion1->Draw(); sm_GraphicsLoop();
                      smgfx_SetLook(1,2, 1,0,0); pOtherRegion1->Draw(); sm_GraphicsLoop();
                      smgfx_SetLook(1,2, 0,1,0); pThisRegion2->Draw(); sm_GraphicsLoop();
                      smgfx_SetLook(1,2, 0,1,1); pOtherRegion2->Draw(); sm_GraphicsLoop();
                      sm_GraphicsLoop();
                    }
#endif // SM_DEBUG_CODE
                }
            } // end iter every KeepOFace
       } // end m_bCookieCutter && operation != SEPARATE check

      // arrive here after all OBrep KeepOFaces have been added to Brep
      // all added newThisFace/OldFace pairs have been added to This/Other object pointer map

      // update the infinite region - if the InfiniteRegion 
      //   was not split - no work needs to be done, it's still the InfiniteRegion.
      //   was split - it will contain an SmMergeRegionAttribute attribute
      //      and all other split region children whose source was the InfiniteRegion
      //      need to be checked to find those whose OtherBrep->OriginalRegions
      //      are OtherBrep->InfiniteRegions.  When there is only 1, that is the new InfiniteRegion.
      //      When there are more than 1 of those, one is the infinite region and the rest are new internal
      //      voids within the OutputBrep
      SmBoolean bInfiniteRegionWasSplit = (NULL != m_vTI.m_pBrep->GetInfiniteRegion()->FindAttribute(SM_AI_OTHER_REGION_ID)) ;
      if(bInfiniteRegionWasSplit)
        {
#ifdef SM_DEBUG_CODE
          if (bDebugMe)
            {
              TCHAR sBuff[SM_TBLOCK_SIZE];
              smos_sprintf(sBuff,_T("%s"), _T("m_vTI.m_pBrep"));
              m_vTI.m_pBrep ->DumpRegionsAndAttributes(sBuff) ;

              smos_sprintf(sBuff,_T("%s"), _T("m_vTI.m_pOther"));
              m_vTI.m_pOther->DumpRegionsAndAttributes(sBuff) ;
            }
#endif // SM_DEBUG_CODE

          lCount = 0 ; 
          m_vTI.m_pBrep->GetRegions( sBrepRegions );

          // iter all Regions - looking for infinite region data
          for(ULONG ii=0; ii<sBrepRegions.GetSize(); ii++)
            {
              SmRegion               * pThisRegion        = sBrepRegions[ii];
              SmMergeRegionAttribute * pMergeRegionAttrib = pThisRegion ? (SmMergeRegionAttribute *)(pThisRegion->FindAttribute(SM_AI_OTHER_REGION_ID)) : NULL ;

#ifdef SM_DEBUG_CODE
                  if (bDebugMe)
                    {
                      TCHAR sBuff[SM_TBLOCK_SIZE];
                      smos_sprintf(sBuff,_T("%s"), _T("m_vTI.m_pBrep"));
                      m_vTI.m_pBrep->DumpRegionsAndAttributes(sBuff) ;
                      if(pMergeRegionAttrib) pMergeRegionAttrib->Dump() ;
                    }
#endif // SM_DEBUG_CODE

              // skip regions which were not split
              if(pMergeRegionAttrib == NULL) 
                { continue; }

              // when This child Region's origin regions were both infinite regions
              if(   pMergeRegionAttrib->HasThisInfiniteRegion()
                 && pMergeRegionAttrib->HasOtherInfiniteRegion())
                {
                  // save it as the new InfiniteRegion
                  m_vTI.m_pBrep->SetInfiniteRegion(pThisRegion) ;
                  lCount++ ;
                }
            } // end iter all Regions - looking for infinite region data

          // infinite region is not current when more than one Region had two InfiniteRegion sources
          if(lCount > 1)
            { bInfiniteRegionCurrent = FALSE ; }
        } // end ThisBrep->InfiniteRegion was split check

#ifdef SM_DEBUG_CODE
      if (bDebugMe)
        {
          SM_DUMP_AND_ASSERT_VALID(m_vTI.m_pBrep);
          SM_DUMP_AND_ASSERT_VALID(m_vTI.m_pOther);
          m_vTI.Dump();

          smgfx_Erase() ;
          smgfx_SetLook(1,2, 0,0,1) ; m_vTI.m_pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
          smgfx_SetLook(1,2, 0,1,0) ; m_vTI.m_pOther->Draw(TRUE) ; sm_GraphicsLoop() ;
          smgfx_SetLook(3,4, 0,1,1) ; m_vTI.Draw(3,5, 7,9) ; sm_GraphicsLoop() ;
          sm_GraphicsLoop() ;
        }
#endif // SM_DEBUG_CODE

      // One more little complication.  Suppose we're doing an Intersection on
      // two sheet faces: faces that don't separate void from solid, say, just
      // hanging in the infinite region.  All faces will be deleted, leaving only
      // a wire edge.  Normally this operation deletes wire edges, but in this case,
      // we would want to keep it.  (Or, for example, take a solid sphere and set
      // its interior region to Void, and Intersect it with a plane.)  This will be
      // the case if (1) the operation is Intersect, and (2) the edge borders no
      // Solid regions.  [also B144]

      if ( eOperation == SM_BO_INTERSECTION )
        {
          // For each intersection edge, add it to sKeepAsWires
          // if it does not border a Solid region.
          SmTArray<SmEdge *> sBrepEdges, sOtherEdges ;
          m_vTI.GetCommonEdges(sBrepEdges, sOtherEdges) ;

          ULONG kkk, lNumEdges = sBrepEdges.GetSize();
          for ( kkk = 0; kkk < lNumEdges; kkk++ )
          {
              SmEdge *pE = sBrepEdges[kkk];
              if ( sm_EdgeBordersSolid( pE ) == NULL )
                { sKeepAsWires.AddUnique( pE ); }
          }
        } // end checking for edges to keep
    } // end STEP 5

  // STEP 6. delete Brep->faces on the DelFaces list
  //          note: when Infinite region happens to be merged, it's kept and the other merge region is deleted
    {
      SER(m_vTI.DeleteFaces(sDelFaces, &sKeepAsWires));
    
    } // end STEP 6 - delete Brep->faces on the DelFaces list

  // arrive here after all OBrep KeepOFaces have been added to Brep
  //               and all Brep  DeleteFaces have been removed from Brep

#ifdef SM_DEBUG_CODE
SmBoolean bDebugRadialEdges = FALSE ;
  if (bDebugMe)
    {
      SM_DUMP_AND_ASSERT_VALID(m_vTI.m_pBrep);
      SM_DUMP_AND_ASSERT_VALID(m_vTI.m_pOther);
      // don't dump BrepToOther entity maps after topology deletes - those lists may have stale pointers

      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0,0,1) ; m_vTI.m_pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,0) ; m_vTI.m_pOther->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(3,4, 0,1,1) ; m_vTI.Draw(3,5, 7,9) ; sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;
      if(bDebugRadialEdges)
        {
          // get all Brep mapped edges
          SmTArray<SmObject *> sObjects ;
          m_vTI.GetMappedBrepObjects(SmEdge_TYPE, sObjects) ;

          // for every edge
          for(ULONG ii=0; ii<sObjects.GetSize(); ii++)
            {
              SmTArray<SmEdgeuse *> sEU ;
              SmEdge *pEdge = (SmEdge *)sObjects[ii] ;
              pEdge->GetEdgeuses( sEU ) ;

              // for every other edgeuse - draw face and radial mate face
              for(jj=0; jj < sEU.GetSize(); jj+=2)
                {
                  SmEdgeuse *pEdgeuse    = sEU[jj] ;
                  SmEdgeuse *pRadial     = pEdgeuse->GetRadial() ;
                  //SmFace    *pFace       = pEdgeuse->GetFace() ;   // unused
                  //SmFace    *pRadialFace = pRadial->GetFace() ;    // unused
                  SM_ASSERT(pRadial->GetRadial() == pEdgeuse) ;

                  // draw breps with radial face pair
                  smgfx_Erase() ;
                  smgfx_SetLook(1,2, 0,0,1) ; m_vTI.m_pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
                  smgfx_SetLook(1,2, 0,1,0) ; m_vTI.m_pOther->Draw(TRUE) ; sm_GraphicsLoop() ;
                  // smgfx_SetLook(3,4, 0,1,1) ; m_vTI.Draw(3.0, 4.0) ; sm_GraphicsLoop() ;
                  smgfx_SetLook(7,8, 1,0,0) ; pEdgeuse->GetEdge()->Draw() ; sm_GraphicsLoop() ;
                  smgfx_SetLook(1,2, 1,0,0) ; pEdgeuse->GetFaceuse()->Draw(SM_DM_CROSSHATCH) ; sm_GraphicsLoop() ;
                  smgfx_SetLook(1,2, 1,0,0) ; pRadial->GetFaceuse()->Draw(SM_DM_CROSSHATCH) ; sm_GraphicsLoop() ;
                  sm_GraphicsLoop() ;

                } // end iter every other edgeuse
            } // end iter every mapped Brep Edge
        } // end debug radial edge ordering check
    }
#endif // SM_DEBUG_CODE

  // STEP 7. Calculate whether each region is void or solid,
  // based on their previous status and the operation taking place.
  // This is done for regions whose other-brep-region pointer
  // has been set: that happens when Brep regions are split.

  // note: Region Attribute propagation:
  //
  //       Every OutputBrep Vertex, Edge, Face, and Region topology object originates
  //       from topology objects in ThisBrep, OtherBrep, or Both. Attribute propagation
  //       for all topology objects except Regions is already done.  
  //
  //       At this point every OutputBrep Region has an attribute:[id=SM_AI_OTHER_REGION_ID]
  //       whose pointer value is set to a Region in the OtherBrep and remembers original InfiniteRegion ancestry, or has no 
  //       no such attribute(was never split by the inclusion of OtherBrep faces).
  //       Regions with no attribute:[id=SM_AI_OTHER_REGION_ID] objects
  //       are Regions originating from ThisBrep Regions and their
  //       attributes are already in place.
  //
  //       Regions containing attribute:[id=SM_AI_OTHER_REGION_ID] objects need to propagate
  //       OtherBrep Regions attributes.
  //
  //       Attribute propagation for vertices, edges, and faces copied from or merged into
  //       the OutputBrep from the OtherBrep happen at the time those topology objects
  //       are copied or merged.  Attribute propagation for Regions happens here because
  //       OtherBrep Regions are not copied into the OutputBrep.  Instead, OutputBrep
  //       Regions are created by splitting ThisBrep Regions as a consequence of copying
  //       OtherBrep topology into the OutputBrep. When a ThisBrep region is split, it
  //       saves a temporary OtherBrepRegion pointer to be used in this section to
  //       decide on the proper attribute propagation behavior.
    {
      m_vTI.m_pBrep->GetRegions( sBrepRegions );
      lCount = sBrepRegions.GetSize();

#ifdef SM_DEBUG_CODE
      if (bDebugMe)
        {
          TCHAR sBuff[SM_TBLOCK_SIZE];
          smos_sprintf(sBuff,_T("%s"), _T("m_vTI.m_pBrep"));
          m_vTI.m_pBrep->DumpRegionsAndAttributes (sBuff) ;
          smos_sprintf(sBuff,_T("%s"), _T("m_vTI.m_pOther"));
          m_vTI.m_pOther->DumpRegionsAndAttributes(sBuff) ;
        }
#endif // SM_DEBUG_CODE

      // for every Brep Region - update Region IsVoid state
      for( ULONG ii=0; ii<lCount; ii++ )
        {
          SmRegion * pThisRegion  = sBrepRegions[ii];

          // next line obsolete: new attribute mechanism replaces old member method of passing OtherRegion data
          // removed:gwc  SmRegion * pOtherRegion = pThisRegion ? pThisRegion->GetOtherBrepRegion() : NULL ;

          // get region's OtherRegionAttribute
          SmMergeRegionAttribute * pMergeRegionAttrib = pThisRegion ? (SmMergeRegionAttribute *)(pThisRegion->FindAttribute(SM_AI_OTHER_REGION_ID)) : NULL ;
          // removed:gwc  SM_ASSERT_MSG(   pThisRegion == NULL
          // removed:gwc                || ((pOtherRegionAttrib == NULL) && (pThisRegion->GetOtherBrepRegion() == NULL))
          // removed:gwc                || ((pOtherRegionAttrib != NULL) && (((SmRegion *)pOtherRegionAttrib->GetValue()) == pThisRegion->GetOtherBrepRegion())),
          // removed:gwc                _T("New Attribute based RegionAttribute code does not reproduce previous behavior")) ;

          // no work - no MergeRegionAttribute, the ResultRegion was a ThisBrep Region that was not modified by the merge operation
          if(pMergeRegionAttrib == NULL)
            { continue ; }

          // get OtherRegion from OtherRegionAttribute's value
          // SmRegion * pOtherRegion = pMergeRegionAttrib ? pMergeRegionAttrib->GetOtherRegion(0) : NULL ;

#ifdef SM_DEBUG_CODE
          if (bDebugMe)
            {
              if (m_vTI.m_pBrep) {
                  TCHAR sBuff[SM_TBLOCK_SIZE];
                  smos_sprintf(sBuff,_T("%s"), _T("m_vTI.m_pBrep"));
                  m_vTI.m_pBrep->DumpRegionsAndAttributes(sBuff);
              }
              if (m_vTI.m_pOther) {
                  TCHAR sBuff[SM_TBLOCK_SIZE];
                  smos_sprintf(sBuff,_T("%s"), _T("m_vTI.m_pOther"));
                  m_vTI.m_pOther->DumpRegionsAndAttributes(sBuff);
              }
              if(pMergeRegionAttrib) pMergeRegionAttrib->Dump() ;
            }
#endif // SM_DEBUG_CODE

          // Calculate the new region's void status.
          // It's easier to work with 'is solid' status.
          SmBoolean bThisIsSolid = ! pThisRegion->IsVoid();
          //SmBoolean bOthrIsSolid = ! pOtherRegion->IsVoid();
          SmBoolean bResultIsSolid;
          switch( eOperation )
            { case SM_BO_UNION :
              case SM_BO_MERGE :
              case SM_BO_PARTIAL_MERGE :
                bResultIsSolid = pMergeRegionAttrib->HasAnySolid(3) ; // 3 = check both This and Other Src Regions 
                break;
              case SM_BO_DIFFERENCE :
              case SM_BO_SLICE :
                bResultIsSolid =     pMergeRegionAttrib->HasAnySolid(1)   // 1 = Check This Src Regions
                                 && !pMergeRegionAttrib->HasAnySolid(2) ; // 2 = check Other Src Regions
                break;
              case SM_BO_INTERSECTION :
                bResultIsSolid =    pMergeRegionAttrib->HasAnySolid(1)   // 1 = Check This Src Regions
                                 && pMergeRegionAttrib->HasAnySolid(2) ; // 2 = check Other Src Regions
                break;
              case SM_BO_EXCLUSIVE_OR :
                bResultIsSolid =    pMergeRegionAttrib->HasAnySolid(1)   // 1 = Check This Src Regions
                                 != pMergeRegionAttrib->HasAnySolid(2) ; // 2 = check Other Src Regions
                break;
              case SM_BO_EXTRACT_SEPARATE :
              case SM_BO_UNKNOWN :
              default:
                bResultIsSolid = bThisIsSolid;
            } // end switch on eOperation

          pThisRegion->SetIsVoid( ! bResultIsSolid );

        } // end iter every Brep Region updating Region->IsVoid state
    } // end STEP 7

#ifdef SM_DEBUG_CODE
  if (bDebugMe)  // after Region->IsVoid state update
    {
      if (m_vTI.m_pBrep) {
          TCHAR sBuff[SM_TBLOCK_SIZE];
          smos_sprintf(sBuff,_T("%s"), _T("m_vTI.m_pBrep"));
          m_vTI.m_pBrep->DumpRegionsAndAttributes(sBuff);
      }
    }
#endif // SM_DEBUG_CODE

  // STEP 8. When needed - find and set the infinite region if missing or incorrect
  //  NOTE: It's currently getting set incorrectly above for SM_BO_SLICE, need to fix that. [B600]
    {
      if (   (   bInfiniteRegionCurrent == FALSE )
          || ( ! m_vTI.m_pBrep->GetInfiniteRegion()->IsVoid() ) )
      // gwc replaced by above new conditional:  m_vTI.m_pBrep->GetRegions(sBrepRegions);
      // gwc replaced by above new conditional:  if (sBrepRegions.GetSize() > 2 || !m_bManifoldBoolean)
        {
          m_vTI.m_pBrep->FindAndSetInfiniteRegion();
        }
    } // end STEP 8 - find and set the infinite region if missing

  // STEP 9. Propagate OtherBrep SrcRegion attributes to ResultRegion attributes 
  //         - use Notify mechanims to broadcast the moves to callers
  //         - remove temporary MergeRegionAttrib attributes from the ResultRegions - we're done with them
    {
      PropagateOtherSrcRegionAttribs(eOperation) ;
    
    } // end STEP 9 - Propagate OtherBrep SrcRegion attributes to ResultRegion attributes

#ifdef SM_DEBUG_CODE
  // draw
  if (bDebugMe)
    {
      SM_DUMP_AND_ASSERT_VALID(m_vTI.m_pBrep);
      SM_DUMP_AND_ASSERT_VALID(m_vTI.m_pOther);

      TCHAR sBuff[SM_TBLOCK_SIZE];
      smos_sprintf( sBuff, _T( "%s" ), _T( "m_vTI.m_pBrep" ) );
      m_vTI.m_pBrep->DumpRegionsAndAttributes( sBuff ) ;
      m_vTI.m_pBrep->GetRegions(sBrepRegions) ;

      // don't dump BrepToOther entity maps after topology deletes - those lists may have stale pointers

      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0,0,1) ; m_vTI.m_pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,0) ; m_vTI.m_pOther->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(3,4, 0,1,1) ; m_vTI.Draw(3,5, 7,9) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 1,1,0) ; for(di=0;di<sBrepRegions.GetSize();di++) // void regions
                                    { SmRegion *pRegion = sBrepRegions[di] ;
                                      if(   pRegion->IsVoid()
                                         && pRegion != m_vTI.m_pBrep->GetInfiniteRegion())
                                        {  pRegion->Draw() ; sm_GraphicsLoop() ; }
                                    }
      smgfx_SetLook(1,2, 1,0,1) ; for(di=0;di<sBrepRegions.GetSize();di++) // non void regions
                                    { SmRegion *pRegion = sBrepRegions[di] ;
                                      if(   !pRegion->IsVoid())
                                        {  pRegion->Draw() ; sm_GraphicsLoop() ; }
                                    }
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

  // STEP 10
  // BooleanPostProcess will remove edges between planar faces
  // and faces which have the same surface but are not seam edges.
  // If its argument is True, it will check every edge in the resulting Brep;
  // if False, it checks only those edges involved in the Boolean operation.
    {
      if (   eOperation == SM_BO_UNION
          || eOperation == SM_BO_INTERSECTION
          || eOperation == SM_BO_DIFFERENCE
          || eOperation == SM_BO_EXCLUSIVE_OR
          || eOperation == SM_BO_SLICE )
        {
          if (m_bBooleanPostProcess)
              SER(BooleanPostProcess( FALSE ));
          // note: face and vertex pointers might be deleted as
          //       a side effect of BooleanPostProcessremoving edges
          //       that are left as stale pointers in m_vTI. This
          //       function is written to allow for that.  However,
          //       when debugging, beware of m_vTI dumps after this call.
        }
    } // end STEP 10

  } // end Set up the Breps for editing Block
    //   - temporary variable value changes are reversed
    //     when exiting scope

#ifdef SM_DEBUG_CODE
  // draw
  if (bDebugMe)
    {
      SM_DUMP_AND_ASSERT_VALID(m_vTI.m_pBrep);
      SM_DUMP_AND_ASSERT_VALID(m_vTI.m_pOther);
      // don't dump BrepToOther entity maps after topology deletes - those lists may have stale pointers

      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0,0,1) ; m_vTI.m_pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,0) ; m_vTI.m_pOther->Draw(TRUE) ; sm_GraphicsLoop() ;
      // watch out - stale pointers inside m_vTI!
      //    smgfx_SetLook(3,4, 0,1,1) ; m_vTI.Draw(3,4) ; sm_GraphicsLoop() ;
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

  // STEP 10: for operation SM_BO_EXTRACT_SEPARATE, place
  //          each set of disjoint topology into its own Brep object
  //          by copying Faces into new Breps that get placed
  //          into m_vOneBrepPer_OrigBrepConnectedFaceSet
  if (eOperation == SM_BO_EXTRACT_SEPARATE)
    {
      SM_PTR_ARRAY(sFaces,SmFace,256);

      ULONG jjj, kkk;
      lCount = sBrepFacesCount.GetSize();
      for(ULONG iii=0; iii<lCount; iii++)
        {
          sFaces.ReSet();
          for(jjj=0; jjj<sBrepFacesCount[iii]; jjj++)
            {
              sFaces.Add(sBrepFaces[jjj]);
            }
          sBrepFaces.RemoveAt(0,sBrepFacesCount[iii]);

          // GWC:CHANGED_OCTOBER_2004
          SmBrep *pBrep = new (*m_vTI.m_pBrep->GetContext()) SmBrep();
          // gwc: removed next line - sets pBrep->Tol = m_vTI.m_pBrep->GetContext()::ZoneTol3d
          //  pBrep->SetTolerance(m_vTI.m_pBrep->GetTolerance());

          // removed line: SmBrep *pBrep = new (*m_vTI.m_pOther->GetContext()) SmBrep();
          // removed line: pBrep->SetTolerance(m_vTI.m_pOther->GetTolerance());

          // copy faces (keeping existing topology between faces)
          m_vTI.m_pBrep->CopyFaces(sFaces,pBrep);
          m_vOneBrepPer_OrigBrepConnectedFaceSet.Add(pBrep);
        }
      lCount = sOtherFacesCount.GetSize();
      for(kkk=0; kkk<lCount; kkk++)
        {
          sFaces.ReSet();
          for(jjj=0; jjj<sOtherFacesCount[kkk]; jjj++)
            {
              sFaces.Add(sOtherFaces[jjj]);
            }
          sOtherFaces.RemoveAt(0,sOtherFacesCount[kkk]);
          SmBrep *pBrep = new (*m_vTI.m_pOther->GetContext()) SmBrep();
          // gwc: removed next line - sets pBrep->Tol = m_vTI.m_pBrep->GetContext()::ZoneTol3d
          //  pBrep->SetTolerance(m_vTI.m_pOther->GetTolerance());
          m_vTI.m_pOther->CopyFaces(sFaces,pBrep);
          m_vOneBrepPer_OtherBrepConnectedFaceSet.Add(pBrep);
        }

      if (!m_bKeepOtherBrep)
        { SM_ASSERT(m_vTI.m_pBrep != NULL) ; delete m_vTI.m_pBrep ; m_vTI.m_pBrep = NULL ; }
    } // end eOperation == SM_BO_EXTRACT_SEPARATE check
  else
    {
      rpResult = m_vTI.m_pBrep;
    }

  // delete m_vTI.m_pOther for all cases except eOperation == SM_BO_PARTIAL_MERGE
  //   so that SmMerge::PiecewiseMerge() can repeatedly call ManifoldBoolean() without
  //   having to copy the m_vTI.m_pOther Brep before each call.
  if (eOperation != SM_BO_PARTIAL_MERGE  && !m_bKeepOtherBrep  )
    {
      SM_ASSERT(m_vTI.m_pOther != NULL) ; delete m_vTI.m_pOther ; m_vTI.m_pOther = NULL ;
    }

#ifdef SM_DEBUG_CODE

static int cbiEndDump=0;
if ( cbiEndDump>0 ) {
  TCHAR sBuff[SM_TBLOCK_SIZE];
  ULONG lNumFaces = m_vTI.m_pBrep->GetNumFaces();
  smos_sprintf(sBuff, _T("\nAfter Boolean: Num Faces %ld\n"), lNumFaces );
  smos_WriteBuffer(sBuff);
}

  // show final result
  if (bDebugMeX)
    {
      m_vTI.m_pBrep->Dump();
      if(m_vTI.m_pOther) m_vTI.m_pOther->Dump();
      // don't dump BrepToOther entity maps after topology deletes - those lists may have stale pointers

      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0,0,1) ; m_vTI.m_pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,0) ; if(m_vTI.m_pOther) m_vTI.m_pOther->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(3,4, 0,1,1) ; if(m_vTI.m_pOther) m_vTI.Draw(3,5, 7,9) ; sm_GraphicsLoop() ;
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

  // all done
  if ( m_vTI.m_pBrep != NULL )
    {
      m_vTI.m_pBrep->DeleteStaleCaches();  // [B352]
    }

  pBrepContext->SetDoingBoolean(FALSE);
  pOtherContext->SetDoingBoolean(FALSE);
  return SM_SUCCESS;

} // end SmMerge::ManifoldBoolean

// ==============================================================

#ifdef SM_DEBUG_CODE
// cbi: Locals for debugging:
static int sDumpVerts = 0;
static int sDumpSplit = 0;
static int sDumpEdges = 0;

static void DebugDraw
 (SmBrep              * pBrep,
  SmTArray<SmFace*  > * pFaces1,  // lt green
  SmTArray<SmEdge*  > * pEdges1,
  SmTArray<SmVertex*> * pVerts1,
  SmTArray<SmFace*  > * pFaces2,  // cyan
  SmTArray<SmEdge*  > * pEdges2,
  SmTArray<SmVertex*> * pVerts2,
  SmTArray<SmFace*  > * pFaces3,  // yellow
  SmTArray<SmEdge*  > * pEdges3,  // magenta
  SmTArray<SmVertex*> * pVerts3)  // red
{
SmBoolean bDebugMe = FALSE;
if ( ! bDebugMe ) { return; }

  if ( pBrep != NULL )
  {
      smgfx_Erase();

      smgfx_SetLook( 1,2, 0,0,0 ); pBrep->Draw( TRUE ); sm_GraphicsLoop();
      sm_GraphicsLoop();
  }

  SmFace   *pThisFace;
  SmEdge   *pThisEdge;
  SmVertex *pThisVert;

  ULONG ii, lCnt = ( pFaces1 == NULL ) ? 0 : (*pFaces1).GetSize();
  smgfx_SetLook( 2,4, 0,0.6,0 ); // Lt Green
  for ( ii = 0; ii < lCnt; ii++ )
  {
    pThisFace = (*pFaces1)[ii];
    if ( pThisFace == NULL ) { continue; }
    pThisFace->Draw(SM_DM_CROSSHATCH,4,4); sm_GraphicsLoop();
    sm_GraphicsLoop();
  }
  lCnt = ( pEdges1 == NULL ) ? 0 : (*pEdges1).GetSize();
  for ( ii = 0; ii < lCnt; ii++ )
  {
    pThisEdge = (*pEdges1)[ii];
    if ( pThisEdge == NULL ) { continue; }
    pThisEdge->Draw(); sm_GraphicsLoop();
    sm_GraphicsLoop();
  }
  lCnt = ( pVerts1 == NULL ) ? 0 : (*pVerts1).GetSize();
  for ( ii = 0; ii < lCnt; ii++ )
  {
    pThisVert = (*pVerts1)[ii];
    if ( pThisVert == NULL ) { continue; }
    pThisVert->Draw(); sm_GraphicsLoop();
    sm_GraphicsLoop();
  }

  lCnt = ( pFaces2 == NULL ) ? 0 : (*pFaces2).GetSize();
  smgfx_SetLook( 2,4, 0,1,1 );  // Cyan
  for ( ii = 0; ii < lCnt; ii++ )
  {
    pThisFace = (*pFaces2)[ii];
    if ( pThisFace == NULL ) { continue; }
    pThisFace->Draw(SM_DM_CROSSHATCH,4,4); sm_GraphicsLoop();
    sm_GraphicsLoop();
  }
  lCnt = ( pEdges2 == NULL ) ? 0 : (*pEdges2).GetSize();
  for ( ii = 0; ii < lCnt; ii++ )
  {
    pThisEdge = (*pEdges2)[ii];
    if ( pThisEdge == NULL ) { continue; }
    pThisEdge->Draw(); sm_GraphicsLoop();
    sm_GraphicsLoop();
  }
  lCnt = ( pVerts2 == NULL ) ? 0 : (*pVerts2).GetSize();
  for ( ii = 0; ii < lCnt; ii++ )
  {
    pThisVert = (*pVerts2)[ii];
    if ( pThisVert == NULL ) { continue; }
    pThisVert->Draw(); sm_GraphicsLoop();
    sm_GraphicsLoop();
  }

  smgfx_SetLook( 4,6, 1,1,0 );  // Yellow
  lCnt = ( pFaces3 == NULL ) ? 0 : (*pFaces3).GetSize();
  for ( ii = 0; ii < lCnt; ii++ )
  {
    pThisFace = (*pFaces3)[ii];
    if ( pThisFace == NULL ) { continue; }
    pThisFace->Draw(SM_DM_CROSSHATCH,4,4); sm_GraphicsLoop();
    sm_GraphicsLoop();
  }
  smgfx_SetLook( 4,6, 1,0,1 ); // Magenta
  lCnt = ( pEdges3 == NULL ) ? 0 : (*pEdges3).GetSize();
  for ( ii = 0; ii < lCnt; ii++ )
  {
    pThisEdge = (*pEdges3)[ii];
    if ( pThisEdge == NULL ) { continue; }
    pThisEdge->Draw(); sm_GraphicsLoop();
    sm_GraphicsLoop();
  }
  smgfx_SetLook( 4,6, 1,0,0 ); // Red
  lCnt = ( pVerts3 == NULL ) ? 0 : (*pVerts3).GetSize();
  for ( ii = 0; ii < lCnt; ii++ )
  {
    pThisVert = (*pVerts3)[ii];
    if ( pThisVert == NULL ) { continue; }
    pThisVert->Draw(); sm_GraphicsLoop();
    sm_GraphicsLoop();
  }

  sm_GraphicsLoop();
  return;

} // end DebugDraw()
#    endif // SM_DEBUG_CODE

/*******************************************************************//**
PURPOSE: For an added new Face, glue coincident geometry and intersect,
         insert, and merge intersecting geometry.

NOTES: This can be useful after calls to the CreateFace... methods, which
   add a Face but do not connect it geometrically/topologically.
***********************************************************************/
SmStatus SmMerge::MergeAddedFace(
    SmFace  * pMergeFace,              // in :
    ULONG   & rnNumEdgesStitched,      // out:
    double  & rdMaxVertGap,            // out:
    double  & rdMaxEdgeGap,            // out:
    double  & rdMinUnstitchedVertGap,  // out:
    double  & rdMinUnstitchedEdgeGap,  // out:
    SmBoolean  // bDoGluing : not yet implemented, always True.
  )
{
  // Wrap up a call to MergeAddedTopology().
  SmTArray< SmFace*   > sMergeFaces;  sMergeFaces.Add( pMergeFace );
  SmTArray< SmEdge*   > sMergeEdges;  pMergeFace->GetEdges   ( sMergeEdges );
  SmTArray< SmVertex* > sMergeVerts;  pMergeFace->GetVertices( sMergeVerts );

  SmBoolean m_bMakingManifoldSolid = FALSE;
  SmBoolean m_bFastEdgeCompare = TRUE;
  SmBoolean m_bDoRegionNesting = TRUE;
  SmBoolean m_bIgnoreProblems  = TRUE;

  SmStatus eStat = this->MergeAddedTopology(
    &sMergeFaces, &sMergeEdges, &sMergeVerts,
    rnNumEdgesStitched,
    rdMaxVertGap,  // in/out
    rdMaxEdgeGap,
    rdMinUnstitchedVertGap,
    rdMinUnstitchedEdgeGap,
    m_bMakingManifoldSolid,
    m_bFastEdgeCompare,
    m_bDoRegionNesting,
    m_bIgnoreProblems
  );

  return eStat;

}  // end SmMerge::MergeAddedFace

/*******************************************************************//**
PURPOSE: For added new Faces, Edges, and Vertices,
         glue coincident geometry and intersect, insert, and merge
         intersecting geometry

NOTES:

METHOD: uses bounding boxes and processed lists to minimize the number of checks.
        1. Find all AddedVertex/ExistingVertex coincidences
           1a. Call StitchVertexPairs() for all coincident pairs
           1b. update relationships as needed
        2. Find all AddedVertices splitting ExistingEdges
           2a. Call SplintEdgesWithVertices() for all AddedVertex/ExistingEdge XSects
        3. Find all ExistingVertices splitting AddedEdges
           3a. Call SplintEdgesWithVertices() for all ExistingVertex/AddedEdge XSects
        4. Find all ExistingEdge/AddedEdge coincidences
           4a. with call to StitchEdgesOfVertices()
           4b. update relationships as needed
        5. Find all AddedVertex/ExistingFace XSects
           5a. Call VertexFaceIntersect()
        6. Find all ExistingVertex/ThisAddedFace XSects
           6a. Call VertexFaceIntersect()
        7. Find all ExistingEdge/AddedEdge XSects
           7a. Call EdgeEdgeIntersect()
        8. Find all AddedEdge/ExistingFace XSects
           8a. Call EdgeFaceIntersect()
        9. Find all ExistingEdge/ThisAddedFace XSects
           9a. Call EdgeFaceIntersect()
       10. Find all AddedEdge/ExistingFace coincidences
          10a. Call EdgeFaceCoincidence()
       11. Find all ExistingFace/ThisAddedFace XSects
          11a. Call FaceFaceIntersect()
       12. Find all ExistingFace/ThisAddedFace coincidences
          12a. Call FaceFaceCoincidence()
       13. Fix up mapping
       14. Fix up region nesting
       15. Fix up face orientations
***********************************************************************/
SmStatus SmMerge::MergeAddedTopology(
    SmTArray< SmFace*   > *pMergeFaces,  // in :
    SmTArray< SmEdge*   > *pMergeEdges,  // in :
    SmTArray< SmVertex* > *pMergeVerts,  // in :
    ULONG   & rnNumEdgesStitched,        // out:
    double  & rdMaxVertGap,              // out:
    double  & rdMaxEdgeGap,              // out:
    double  & rdMinUnstitchedVertGap,    // out:
    double  & rdMinUnstitchedEdgeGap,    // out:
    SmBoolean m_bMakingManifoldSolid,    // in :
    SmBoolean m_bFastEdgeCompare,        // in :
    SmBoolean m_bDoRegionNesting,        // in :
    SmBoolean m_bIgnoreProblems,         // in :
    SmBoolean  // bDoGluing : not yet implemented, always True.
)
{
    SmTopologyIntersector & rTopoInt = this->GetTopologyIntersector();

    SmBrep *pMasterBrep = rTopoInt.GetPrimaryBrep();    // in/out: modified in place

    // for naming convenience:
    double d3dTol = rTopoInt.GetThisApproxTol3d();

    // We reset our tol in this method.  Make sure it ends up where it started when we're done.
    SmTemporaryChangeValue< double > sTolReset(rTopoInt.m_dThisApproxTol3d, d3dTol);

    // set Brep to allow modifications
    SmTemporaryChangeValue<SmBoolean> sChange(pMasterBrep->m_bEditingEnabled, TRUE);
    pMasterBrep->Notify(SM_NO_PRE_EDIT, pMasterBrep, NULL, NULL);


    // 0: all at given tol
    // 1: 4x
    // 2: 10x
    // 3: 25x
    static constexpr int cbiToleranceSetting = 1;

#ifdef SM_DEBUG_CODE
    SmBoolean bDoGraphics = FALSE;
    if (bDoGraphics)
    {
        if (FALSE)
        {
            smgfx_Erase();
        }
        smgfx_SetLook(1, 2, 0, 0, 0); pMasterBrep->Draw(TRUE); sm_GraphicsLoop();
        if (pMergeFaces && pMergeFaces->GetSize() > 0) {
            smgfx_SetLook(2, 4, 0, 1, 1); (*pMergeFaces)[0]->Draw(); sm_GraphicsLoop();
        }
        sm_GraphicsLoop();
    }

    // Timing:
    clock_t tStart, tEnd;
    static double dTime0 = 0, dTime1 = 0, dTime2 = 0, dTime3 = 0, dTime4 = 0;
    static double dTime5 = 0, dTime6 = 0, dTime7 = 0, dTime8 = 0, dTime9 = 0;
    static double dTime10 = 0, dTime11 = 0;

    tStart = clock();

    // timing:
    SmBoolean sbRegNest = TRUE;
    m_bDoRegionNesting &= sbRegNest;

    // Check Breps on input.
    //  0: no checks
    //  1: check, and complain if ok on input and not on output
    //  2: check, and complain if bad on input
    //  3: check, and return SM_ERR if bad on input.
    int iBrepChecks = 0; // cbi: was  2; can still set the static flag interactively.

    SmBoolean bMasterBrepCheckOK = TRUE;
    SmBoolean bThisBrepCheckOK = TRUE;
    SmBoolean bResultBrepCheckOK = TRUE;
    SmBoolean bWriteFiles = FALSE;
    

    if (bDoGraphics) {
        if (iBrepChecks > 0)
        {
            bMasterBrepCheckOK = pMasterBrep->AssertValid();
            if (!bMasterBrepCheckOK || !bThisBrepCheckOK)
            {
                if (iBrepChecks > 1)
                {
                    SM_ASSERT_MSG(bMasterBrepCheckOK, _T("SMS_AddInOneBrep: Invalid Master Brep on input"));
                    SM_ASSERT_MSG(bThisBrepCheckOK, _T("SMS_AddInOneBrep: Invalid This   Brep on input"));
                }
                if (iBrepChecks > 2)
                {
                    return SM_ERR_INVALID_INPUT;
                }
            }
        }

        // Debug write to file.
        
        if (bWriteFiles)
        {
            pMasterBrep->WriteToFile(_T("C:/Bill/v14Dev/master.smb"));
        }
    }
#endif // SM_DEBUG_CODE



  // init some output.
  rnNumEdgesStitched = 0;
  rdMaxVertGap = 0.0;
  rdMaxEdgeGap = 0.0;
  rdMinUnstitchedVertGap = SM_BIG_DOUBLE;
  rdMinUnstitchedEdgeGap = SM_BIG_DOUBLE;

  ULONG ii, jj;

  // Collect vertices, edges, and faces to be merged in.
  SmTArray< SmFace*   > sAddedFaces;
  SmTArray< SmEdge*   > sAddedEdges;
  SmTArray< SmVertex* > sAddedVerts;
  if ( pMergeFaces != NULL )
    { sAddedFaces = *pMergeFaces; } // assignment operator.

  ULONG lNumAddedFaces = sAddedFaces.GetSize();

  if ( pMergeEdges != NULL )
    { sAddedEdges = *pMergeEdges; }
  else if ( lNumAddedFaces > 0 )
  {
      SmTArray< SmEdge* > sTempEdges;
      for ( ii=0; ii<lNumAddedFaces; ii++ )
      {
          SmFace *pF = sAddedFaces[ii];
          pF->GetEdges( sTempEdges );
          for ( jj=0; jj<sTempEdges.GetSize(); jj++ )
            { sAddedEdges.AddUnique( sTempEdges[jj] ); }
      }
  }
  ULONG lNumAddedEdges = sAddedEdges.GetSize();

  if ( pMergeVerts != NULL )
    { sAddedVerts = *pMergeVerts; }
  else
  {
      SmTArray< SmVertex* > sTempVerts;
      // Collect from both Faces and Edges.
      if ( lNumAddedFaces > 0 )
      {
          for ( ii=0; ii<lNumAddedFaces; ii++ )
          {
              SmFace *pF = sAddedFaces[ii];
              pF->GetVertices( sTempVerts );
              for ( jj=0; jj<sTempVerts.GetSize(); jj++ )
                { sAddedVerts.AddUnique( sTempVerts[jj] ); }
          }
      }
      if ( lNumAddedEdges > 0 )
      {
          for ( ii=0; ii<lNumAddedEdges; ii++ )
          {
              SmEdge *pE = sAddedEdges[ii];
              pE->GetVertices( sTempVerts );
              for ( jj=0; jj<sTempVerts.GetSize(); jj++ )
                { sAddedVerts.AddUnique( sTempVerts[jj] ); }
          }
      }
  }
  ULONG lNumAddedVerts = sAddedVerts.GetSize();


  if ( lNumAddedFaces + lNumAddedEdges + lNumAddedVerts < 1 )
    { return SM_SUCCESS; } // Nothing to add in.


  // Double check that all added topology is in pMasterBrep.
  for ( ii=0; ii<lNumAddedFaces; ii++ )
  {
      if ( sAddedFaces[ii]->GetBrep() != pMasterBrep )
        { return SM_ERR_INVALID_INPUT; }
  }
  for ( ii=0; ii<lNumAddedEdges; ii++ )
  {
      if ( sAddedEdges[ii]->GetBrep() != pMasterBrep )
        { return SM_ERR_INVALID_INPUT; }
  }
  for ( ii=0; ii<lNumAddedVerts; ii++ )
  {
      if ( sAddedVerts[ii]->GetBrep() != pMasterBrep )
        { return SM_ERR_INVALID_INPUT; }
  }


  // Collect vertices, edges, and faces from original Master (before merging),
  // and remove those in the Added arrays.

  SmTArray< SmFace* > sMasterFaces;
  pMasterBrep->GetFaces( sMasterFaces );
  sMasterFaces.RemoveElements( sAddedFaces, sMasterFaces );

  SmTArray< SmEdge* > sMasterEdges;
  pMasterBrep->GetEdges( sMasterEdges );
  sMasterEdges.RemoveElements( sAddedEdges, sMasterEdges );

  SmTArray< SmVertex* > sMasterVerts;
  pMasterBrep->GetVertices( sMasterVerts );
  sMasterVerts.RemoveElements( sAddedVerts, sMasterVerts );


  ULONG lNumMasterFaces = sMasterFaces.GetSize();
  ULONG lNumMasterEdges = sMasterEdges.GetSize();
  ULONG lNumMasterVerts = sMasterVerts.GetSize();

  if ( lNumMasterFaces + lNumMasterEdges + lNumMasterVerts < 1 )
    { return SM_SUCCESS; } // Nothing to add to.



  // Bounding Box checks.
  // TRUE: approx, just do verts and edges, don't worry about interiors of Faces.

  SmExtent3d sMasterBox, sThisBox;
  SmExtent3d sTempBox;

  // Master BBox:
  for ( ii=0; ii<lNumMasterFaces; ii++ )
  {
      SmFace *pF = sMasterFaces[ii];
      if ( pF == NULL ) { continue; }
      pF->CalculateBoundingBox( sTempBox );
      sMasterBox.Union( sTempBox, sMasterBox );
  }
  for ( ii=0; ii<lNumMasterEdges; ii++ )
  {
      SmEdge *pE = sMasterEdges[ii];
      if ( pE == NULL ) { continue; }
      pE->CalculateBoundingBox( &sTempBox );
      sMasterBox.Union( sTempBox, sMasterBox );
  }
  for ( ii=0; ii<lNumMasterVerts; ii++ )
  {
      SmVertex *pV = sMasterVerts[ii];
      if ( pV == NULL ) { continue; }
      sMasterBox.AddPoint3d( pV->GetPoint() );
  }

  // Added BBox:
  for ( ii=0; ii<lNumAddedFaces; ii++ )
  {
      SmFace *pF = sAddedFaces[ii];
      if ( pF == NULL ) { continue; }
      pF->CalculateBoundingBox( sTempBox );
      sThisBox.Union( sTempBox, sThisBox );
  }
  for ( ii=0; ii<lNumAddedEdges; ii++ )
  {
      SmEdge *pE = sAddedEdges[ii];
      if ( pE == NULL ) { continue; }
      pE->CalculateBoundingBox( &sTempBox );
      sThisBox.Union( sTempBox, sThisBox );
  }
  for ( ii=0; ii<lNumAddedVerts; ii++ )
  {
      SmVertex *pV = sAddedVerts[ii];
      if ( pV == NULL ) { continue; }
      sThisBox.AddPoint3d( pV->GetPoint() );
  }

  // We did cheap (loose) BBox checks, so expand them a bit:
  sThisBox  .ExpandAbsolute( 10 * d3dTol ); //cbiTol: 10 * tol?  was 100 * tol...
  sMasterBox.ExpandAbsolute( 10 * d3dTol );

  if ( sMasterBox.AreDisjoint( sThisBox ) )
    { return SM_SUCCESS; }



  // Cull all topology lists to the other's bounding box.
  SmExtent3d sBBox;
  SmTArray< SmVertex* > sTempVerts;
  SmTArray< SmEdge  * > sTempEdges;
  SmTArray< SmFace  * > sTempFaces;

  // Cull ThisVerts to Master box.
  sTempVerts.ReSet();
  for ( ii = 0; ii < lNumAddedVerts; ii++ )
  {
    if ( sMasterBox.ContainsPoint3d( sAddedVerts[ii]->GetPoint() ) )
      { sTempVerts.Add( sAddedVerts[ii] ); }
  }
  if ( sTempVerts.GetSize() != lNumAddedVerts )
  {
      sAddedVerts    = sTempVerts;  // assignment operator
      lNumAddedVerts = sAddedVerts.GetSize();
  }
  // Cull ThisEdges to Master box.
  sTempEdges.ReSet();
  for ( ii = 0; ii < lNumAddedEdges; ii++ )
  {
    SmEdge *pE = sAddedEdges[ ii ];
    pE->CalculateBoundingBox( &sBBox );
    if ( ! sMasterBox.AreDisjoint( sBBox, pE->GetTolerance() ) )
      { sTempEdges.Add( pE ); }
  }
  if ( sTempEdges.GetSize() != lNumAddedEdges )
  {
      sAddedEdges    = sTempEdges;
      lNumAddedEdges = sAddedEdges.GetSize();
  }
  // Cull ThisFaces to Master box.
  sTempFaces.ReSet();
  for ( ii = 0; ii < lNumAddedFaces; ii++ )
  {
    SmFace *pF = sAddedFaces[ ii ];
    pF->CalculateBoundingBox( sBBox );
    if ( ! sMasterBox.AreDisjoint( sBBox, pF->GetTolerance() ) )
      { sTempFaces.Add( pF ); }
  }
  if ( sTempFaces.GetSize() != lNumAddedFaces )
  {
      sAddedFaces    = sTempFaces;
      lNumAddedFaces = sAddedFaces.GetSize();
  }

  // Cull Master Verts to This box.
  sTempVerts.ReSet();
  for ( ii = 0; ii < lNumMasterVerts; ii++ )
  {
    if ( sThisBox.ContainsPoint3d( sMasterVerts[ii]->GetPoint() ) )
      { sTempVerts.Add( sMasterVerts[ii] ); }
  }
  if ( sTempVerts.GetSize() != lNumMasterVerts )
  {
      sMasterVerts    = sTempVerts;  // assignment operator
      lNumMasterVerts = sMasterVerts.GetSize();
  }
  // Cull Master Edges to This box.
  sTempEdges.ReSet();
  for ( ii = 0; ii < lNumMasterEdges; ii++ )
  {
    SmEdge *pE = sMasterEdges[ ii ];
    pE->CalculateBoundingBox( &sBBox );
    if ( ! sThisBox.AreDisjoint( sBBox, pE->GetTolerance() ) )
      { sTempEdges.Add( pE ); }
  }
  if ( sTempEdges.GetSize() != lNumMasterEdges )
  {
      sMasterEdges    = sTempEdges;
      lNumMasterEdges = sMasterEdges.GetSize();
  }
  // Cull Master Faces to This box.
  sTempFaces.ReSet();
  for ( ii = 0; ii < lNumMasterFaces; ii++ )
  {
    SmFace *pF = sMasterFaces[ ii ];
    pF->CalculateBoundingBox( sBBox );
    if ( ! sThisBox.AreDisjoint( sBBox, pF->GetTolerance() ) )
      { sTempFaces.Add( pF ); }
  }
  if ( sTempFaces.GetSize() != lNumMasterFaces )
  {
      sMasterFaces    = sTempFaces;
      lNumMasterFaces = sMasterFaces.GetSize();
  }

#ifdef SM_DEBUG_CODE
  tEnd = clock();
  double dTimeTemp = ( tEnd - tStart ) / 1000.0;
  dTime0 += dTimeTemp;
#endif // SM_DEBUG_CODE
// ---------------------------------------------

  // Done culling topology lists to other's bounding box.


  SmTArray< SmEdge* > sDebugEdges;

  // State: we have two lists of vertices to work on,
  //   sMasterVerts and sAddedVerts.
  // They are all the vertices within each other's bounding boxes.
  // sMasterVerts were from the original pMasterBrep,
  // and sAddedVerts are what were just added in, from pThisBrep.
  // They are all in pMasterBrep; pThisBrep means nothing now.

/* ****    Vert-Vert Coinc:    *********************** */


  // Now check for coincident vertices.


  SmTArray< SmVertex* > sProcessedVerts;
  SmTArray< SmVertex* > sDeletedVerts;
  double dThisGap, dThisMinUnstitched;

  // Tolerance: We know that we often have to glue things together
  // that are much bigger than the tolerances listed with the topology.
  // Fortunately, StitchVertexPairs() will glue only the closest other
  // vertex, if there is more than one within tolerance.
  // So we can safely pass in a big value.

  double dThisTol = d3dTol;
  switch ( cbiToleranceSetting )
  {
    case 0: { dThisTol = d3dTol *   1;  break; }
    case 1: { dThisTol = d3dTol *   4;  break; }
    case 2: { dThisTol = d3dTol *  10;  break; }
    case 3: { dThisTol = d3dTol *  25;  break; }
  }


  SmStitchCallback sCB;
  SmStitch sStitcher( sCB, dThisTol );

  // The three SmStitch methods that we call use only these four...
  sStitcher.m_bMakingManifoldSolid = m_bMakingManifoldSolid; //cbi: these are args to this method.
  sStitcher.m_bIgnoreProblems      = m_bIgnoreProblems;
  sStitcher.m_bDoRegionNesting     = m_bDoRegionNesting;
  sStitcher.m_bFastEdgeCompare     = m_bFastEdgeCompare;

  // ... but specify these anyway, to be safe.
  sStitcher.m_bValidateResult         = FALSE;
  sStitcher.m_bSqueezeSmallEdges      = FALSE;
  sStitcher.m_bSplitEdgesWithVertices = TRUE;
  sStitcher.m_bRemoveLaminarSlivers   = FALSE;

#ifdef SM_DEBUG_CODE
  tStart = clock();
#endif // SM_DEBUG_CODE

  // StitchVertexPairs() will preserve the vertex pointers in the first
  // argument passed to it.  If we preserve the pointers in the Master
  // Brep, it might help callers to keep track of pointers.

  sStitcher.StitchVertexPairs( sMasterVerts, sAddedVerts,
                         sProcessedVerts, sDeletedVerts, dThisGap, dThisMinUnstitched );

  rdMaxVertGap       = dThisGap;
  rdMinUnstitchedVertGap = dThisMinUnstitched;

#ifdef SM_DEBUG_CODE
  if (sDumpVerts > 0) {
      TCHAR sBuff[SM_TBLOCK_SIZE];
      smos_sprintf(sBuff, _T("\n   Stitch Verts: Processed: %ld "), sProcessedVerts.GetSize());
      smos_WriteBuffer(sBuff);
  }
  if (bDoGraphics) {
      DebugDraw(0, 0, 0, &sMasterVerts, 0, 0, &sAddedVerts, 0, 0, &sProcessedVerts);
  }
#endif // SM_DEBUG_CODE

  // Now all coincident vertices have been glued.
  //   sAddedVerts and sMasterVerts have all glued vertices set to NULL.
  //   sProcessedVerts lists all of the (surviving) glued vertices,
  //   and sDeletedVerts the corresponding deleted glued vertices.

  // But the only problem with preserving the Master vertices
  // in favor of the New vertices is that the original This vertices
  // were mapped to the New vertices.  If a New vertex was glued,
  // is has been deleted, and its mapping should be switched
  // to the vertex (in Master) that it was glued to.

  ULONG lNumGlued = sDeletedVerts.GetSize();
  SM_ASSERT( lNumGlued == sProcessedVerts.GetSize() );

  for ( ii = 0; ii < lNumGlued; ii++ )
  {
      SmVertex * pDelVtx = sDeletedVerts[ ii ];
      SmObject * pTmpObj = rTopoInt.GetOtherMate( pDelVtx );
      SmVertex * pThisVtxPtr = SM_CAST_PTR( SmVertex, pTmpObj );

      if ( pThisVtxPtr != NULL )
      {
          rTopoInt.RemoveRelationship(  pDelVtx,  pThisVtxPtr );
          rTopoInt.Relate( sProcessedVerts[ ii ], pThisVtxPtr );
      }
  } // end for each glued vertex


#ifdef SM_DEBUG_CODE
  tEnd = clock();
  dTimeTemp = ( tEnd - tStart ) / 1000.0;
  dTime1 += dTimeTemp;

  tStart = clock();
#endif // SM_DEBUG_CODE



/* ****    Vert-Edge xSect:    *********************** */

  // Now look for vertices splitting edges.

//  // Build new Brep Edge spatial tree.
//  SmTArray< SmEdge* > sEdges;
//  pMasterBrep->GetEdges(sEdges);
//  ULONG lNumPerNode = smos_Max(500,sEdges.GetSize()/10);
//  SmTree *pEdgeTree = new (*pMasterBrep->GetContext()) SmTree(sNewBrepBBox,lNumPerNode,lNumPerNode/10);
//  if ( ! pEdgeTree ) { return SM_ERR; }
//  SmObjDelete sCleanEdgeTree(pEdgeTree);
//
//  ULONG iii;
//  for(iii=0; iii<sEdges.GetSize(); iii++)
//  {
//      SmExtent3d sEBBox;
//      SmEdge  *pEdge  = sEdges[iii];
//      SmCurve *pCurve = pEdge->GetCurve();
//      if ( SM_SUCCESS != pCurve->CalculateBoundingBox(pEdge->GetInterval(),&sEBBox)) { return SM_ERR; }
//      if ( SM_SUCCESS != pEdgeTree->AddToSpatialTree(sEBBox,pEdge)) { return SM_ERR; }
//  }

  // First see about This vertices splitting Master edges.

  // Note, all vertices that were close to existing vertices
  // have been processed and removed from the list,
  // so any remaining vertex that coincides with an edge
  // will be in the interior, not at a vertex.

  SmTArray< SmEdge*   > sSplitEdges, sNewEdges;
  SmTArray< SmVertex* > sSplittingVerts;

  // Here, we don't want to grab edges that are too far away.
  // dThisTol = d3dTol * 10;
  switch ( cbiToleranceSetting )
  {
    case 0: { dThisTol = d3dTol *   1;  break; }
    case 1: { dThisTol = d3dTol *   4;  break; }
    case 2: { dThisTol = d3dTol *  10;  break; }
    case 3: { dThisTol = d3dTol *  25;  break; }
  }
  sStitcher.SetStitchTol3d( dThisTol );

  sStitcher.SplitEdgesWithVertices(
            sAddedVerts,     // in/out: processed verts set to Null.
            sMasterEdges,    // in/out: processed edges set to Null.
            sSplittingVerts, // out: processed verts (from sAddedVerts) added.
            sSplitEdges,     // out: existing edges (from sMasterEdges) that were split
            sNewEdges,       // out: newly-created edges from the splits

            dThisGap,        // out
            dThisMinUnstitched   // out
          );

  rdMaxVertGap       = smos_Max( rdMaxVertGap,       dThisGap );
  rdMinUnstitchedVertGap = smos_Min( rdMinUnstitchedVertGap, dThisMinUnstitched );

  sProcessedVerts.Append( sSplittingVerts );

#ifdef SM_DEBUG_CODE
  if (sDumpSplit > 0) {
      TCHAR sBuff[SM_TBLOCK_SIZE]; 
      smos_sprintf(sBuff, _T("\n   Split Edges: Processed: %ld "), sProcessedVerts.GetSize());
      smos_WriteBuffer(sBuff);
  }
  if (bDoGraphics) {
     DebugDraw(0, 0, 0, 0, 0, 0, &sSplittingVerts, 0, 0, 0);
  }
#endif // SM_DEBUG_CODE


  // Now see about Master vertices splitting This edges.

  sStitcher.SplitEdgesWithVertices(
            sMasterVerts,    // in/out: processed verts set to Null.
            sAddedEdges,     // in/out: processed edges set to Null.
            sSplittingVerts, // out: processed verts (from sMasterVerts) added.
            sSplitEdges,     // out: existing edges (from sAddedEdges) that were split
            sNewEdges,       // out: newly-created edges from the splits

            dThisGap,        // out
            dThisMinUnstitched   // out
          );

  rdMaxVertGap       = smos_Max( rdMaxVertGap,       dThisGap );
  rdMinUnstitchedVertGap = smos_Min( rdMinUnstitchedVertGap, dThisMinUnstitched );

  // New (split) edges must be added to Added list.
  // ... this is now done in the routine.
  // sAddedEdges.Append( sNewEdges );

  sProcessedVerts.Append( sSplittingVerts );

//cbi: mapping?

#ifdef SM_DEBUG_CODE
  if (sDumpSplit > 0) {
      TCHAR sBuff[SM_TBLOCK_SIZE]; 
      smos_sprintf(sBuff, _T("\n   Split Edges: Processed: %ld "), sProcessedVerts.GetSize());
      smos_WriteBuffer(sBuff);
  }
  if (bDoGraphics) {
     DebugDraw(0, 0, 0, &sSplittingVerts, 0, 0, 0, 0, 0, 0);
  }

  tEnd = clock();
  dTimeTemp = ( tEnd - tStart ) / 1000.0;
  dTime2 += dTimeTemp;

  tStart = clock();
#endif // SM_DEBUG_CODE



/* ****    Edge-Edge Coinc:    *********************** */


  // Now glue edges whose vertices have been glued.
  // We need to look at only the vertices that have been processed.

  SmTArray< SmEdge* > sGluedEdges, sDeletedEdges;
  SmTArray< SmVertex* > sProcessedVerts2;

  // Corresponding vertices are already glued, so if the edges are
  // anywhere near each other, they're probably meant to be glued.
  // So a bigger tol is in order.
  switch ( cbiToleranceSetting ) // These are bigger, per comment above.
  {
    case 0: { dThisTol = d3dTol *   2;  break; }
    case 1: { dThisTol = d3dTol *   8;  break; }
    case 2: { dThisTol = d3dTol *  20;  break; }
    case 3: { dThisTol = d3dTol *  50;  break; }
  }
  sStitcher.SetStitchTol3d( dThisTol );

  sStitcher.StitchEdgesOfVertices(
            sProcessedVerts,  // in, unchanged: need to look at only these already-glued vertices.

            sGluedEdges,      // out: survivors of the glued edge pairs
            sDeletedEdges,    // out: deleted   of the glued edge pairs (stale pointers)
            sProcessedVerts2, // out: vertices that had edges glued.

            dThisGap,         // out
            dThisMinUnstitched    // out

          );

  rdMaxEdgeGap       = smos_Max( rdMaxEdgeGap,       dThisGap );
  rdMinUnstitchedEdgeGap = smos_Min( rdMinUnstitchedEdgeGap, dThisMinUnstitched );

#ifdef SM_DEBUG_CODE
  if ( sDumpEdges > 0 )
  {
      TCHAR sBuff[SM_TBLOCK_SIZE];
      smos_sprintf(sBuff, _T("\n   Glue Edges: Processed: %ld Edges, %ld Verts"),
              sGluedEdges.GetSize(), sProcessedVerts.GetSize() );
      smos_WriteBuffer(sBuff);
  }
  if (bDoGraphics) {
      DebugDraw(0, 0, 0, 0, 0, 0, 0, 0, &sGluedEdges, 0);
  }
#endif // SM_DEBUG_CODE

  // Same thing here as after stitching vertices: Edges in the
  // mapping may have been deleted, so mapping needs to be updated.
  // Also, remove deleted edges from Master/Added edges lists.
  // Also, any glued edges are edges that were originally in both Breps.
  // As such, those edges will not enter into any of the subsequent
  // operations: intersecting other edges or faces in their interiors,
  // or being coincident with faces.  So remove them from the two lists as well.

  lNumGlued = sDeletedEdges.GetSize();
  SM_ASSERT( lNumGlued == sGluedEdges.GetSize() );

  for ( ii = 0; ii < lNumGlued; ii++ )
  {
      SmEdge * pDelEdge = sDeletedEdges[ ii ];

      SmObject * pTmpObj = rTopoInt.GetOtherMate( pDelEdge );
      SmEdge   * pThisEdgePtr = SM_CAST_PTR( SmEdge, pTmpObj );
      if ( pThisEdgePtr != NULL )
      {
          rTopoInt.RemoveRelationship( pDelEdge, pThisEdgePtr );
          rTopoInt.Relate( sGluedEdges[ ii ], pThisEdgePtr );
      }

      // Remove pDelEdge from Master or Added list.
      ULONG lIdx;
      if ( sMasterEdges.FindElement( pDelEdge, lIdx ) )
        { sMasterEdges[ lIdx ] = NULL; }
      if ( sAddedEdges.FindElement( pDelEdge, lIdx ) )
        { sAddedEdges[ lIdx ] = NULL; }

      // Also remove glued edge from Master or Added list.
      SmEdge * pGluedEdge = sGluedEdges[ ii ];
      if ( sMasterEdges.FindElement( pGluedEdge, lIdx ) )
        { sMasterEdges[ lIdx ] = NULL; }
      if ( sAddedEdges.FindElement( pGluedEdge, lIdx ) )
        { sAddedEdges[ lIdx ] = NULL; }
  }



#ifdef SM_DEBUG_CODE
  tEnd = clock();
  dTimeTemp = ( tEnd - tStart ) / 1000.0;
  dTime3 += dTimeTemp;

  tStart = clock();
#endif // SM_DEBUG_CODE



/* ****    Vertex-Face xSect:    *********************** */


//cbi New 3/4/10 start:
  // Now look for vertices in Face interiors.
  // Note, all vertices that were close to existing vertices or edges
  // have been processed and removed from the list,
  // so any remaining vertex that coincides with a face
  // will be in the interior, not on an edge or vertex.

//  // Build new Brep Face spatial tree.
//  SmTArray< SmFace* > sFaces;
//  pMasterBrep->GetFaces(sFaces);
//  lNumPerNode = smos_Max(500,sFaces.GetSize()/10);
//  SmTree *pFaceTree = new (*pMasterBrep->GetContext()) SmTree(sNewBrepBBox,lNumPerNode,lNumPerNode/10);
//  if ( ! pFaceTree ) { return SM_ERR; }
//  SmObjDelete sCleanFaceTree(pFaceTree);
//
//  for(iii=0; iii<sFaces.GetSize(); iii++)
//  {
//      SmExtent3d sFBBox;
//      SmFace  *pFace  = sFaces[iii];
//      SmSurface *pSurface = pFace->GetSurface();
//      if ( SM_SUCCESS != pSurface->CalculateBoundingBox(pFace->GetUVDomain(),&sFBBox)) {return SM_ERR;}
//      if ( SM_SUCCESS != pFaceTree->AddToSpatialTree(sFBBox,pFace)) { return SM_ERR; }
//  }

  // First see about This vertices sitting in Master faces.

  SmTArray< SmFace* > sProcessedFaces;

  // Here, we don't want to grab faces that are too far away.
  // dThisTol = d3dTol * 10;
  switch ( cbiToleranceSetting )
  {
    case 0: { dThisTol = d3dTol *   1;  break; }
    case 1: { dThisTol = d3dTol *   4;  break; }
    case 2: { dThisTol = d3dTol *  10;  break; }
    case 3: { dThisTol = d3dTol *  25;  break; }
  }
  rTopoInt.SetThisApproxTol3d( dThisTol );

  rTopoInt.VertexFaceIntersect(
            sAddedVerts,      // in/out: processed vertices set to Null.
            sMasterFaces,     // in:  (processed faces not set to Null.)
            sSplittingVerts,  // out: processed vertices from input vertex list
            sProcessedFaces,  // out: processed faces from input face list
            dThisGap,         // out:
            dThisMinUnstitched,   // out:

            m_bMakingManifoldSolid,
            m_bIgnoreProblems
          );

  rdMaxVertGap       = smos_Max( rdMaxVertGap,       dThisGap );
  rdMinUnstitchedVertGap = smos_Min( rdMinUnstitchedVertGap, dThisMinUnstitched );

  sProcessedVerts.Append( sSplittingVerts );

#ifdef SM_DEBUG_CODE
if ( sDumpSplit > 0 )
{
  TCHAR sBuff[SM_TBLOCK_SIZE]; 
  smos_sprintf(sBuff, _T("\n   Imprint Faces: Processed (total): %ld "), sProcessedVerts.GetSize() );
  smos_WriteBuffer(sBuff);
}
#endif // SM_DEBUG_CODE


  // Now see about Master vertices sitting in This faces.
  rTopoInt.VertexFaceIntersect(
            sMasterVerts,     // in/out: processed vertices set to Null.
            sAddedFaces,      // in:  (processed faces not set to Null.)
            sSplittingVerts,  // out: processed vertices from input vertex list
            sProcessedFaces,  // out: processed faces from input face list
            dThisGap,         // out:
            dThisMinUnstitched,   // out:

            m_bMakingManifoldSolid,
            m_bIgnoreProblems
          );

  rdMaxVertGap       = smos_Max( rdMaxVertGap,       dThisGap );
  rdMinUnstitchedVertGap = smos_Min( rdMinUnstitchedVertGap, dThisMinUnstitched );

  sProcessedVerts.Append( sSplittingVerts );

  // Note: At this point we are done with the sMasterVerts and sAddedVerts lists
  // so get rid of them, in case anything within them gets deleted,
  // so that we don't have to worry about using stale pointers.
  // We will still need the Edge and Face lists though.
  sMasterVerts.ReSet();
  sAddedVerts .ReSet();

#ifdef SM_DEBUG_CODE
  if ( sDumpSplit > 0 )
  {
    TCHAR sBuff[SM_TBLOCK_SIZE]; 
    smos_sprintf(sBuff, _T("\n   Imprint Faces: Processed (total): %ld "), sProcessedVerts.GetSize() );
    smos_WriteBuffer(sBuff);
  }

  tEnd = clock();
  dTimeTemp = ( tEnd - tStart ) / 1000.0;
  dTime4 += dTimeTemp;

  tStart = clock();
#endif // SM_DEBUG_CODE


/* ****    Edge-Edge xSect:    *********************** */


//cbi New 3/4/10 end.

//cbi New 3/10/10 start:

  SmTArray< SmVertex* > sNewVerts;
  SmTArray< SmEdge  * > sSplitEdges2, sNewEdges2;


  // Here, we don't want to grab edges that are too far away.
  // dThisTol = d3dTol * 10;
  switch ( cbiToleranceSetting )
  {
    case 0: { dThisTol = d3dTol *   1;  break; }
    case 1: { dThisTol = d3dTol *   4;  break; }
    case 2: { dThisTol = d3dTol *  10;  break; }
    case 3: { dThisTol = d3dTol *  25;  break; }
  }
  rTopoInt.SetThisApproxTol3d( dThisTol );

  rTopoInt.EdgeEdgeIntersect(
            sMasterEdges,  // in: unchanged
            sAddedEdges,   // in: unchanged

            sSplitEdges,   // out: Edges from sMasterEdges list that were split
            sNewEdges,     // out: new Edges split from Edges in sMasterEdges
            sSplitEdges2,  // out: Edges from sAddedEdges list that were split
            sNewEdges2,    // out: new Edges split from Edges in sAddedEdges
            sNewVerts,     // out: new Vertices that split the Edges
            dThisGap,      // out
            dThisMinUnstitched,// out

            m_bMakingManifoldSolid,
            m_bIgnoreProblems
        );

  rdMaxVertGap       = smos_Max( rdMaxVertGap,       dThisGap );
  rdMinUnstitchedVertGap = smos_Min( rdMinUnstitchedVertGap, dThisMinUnstitched );

  sProcessedVerts.Append( sNewVerts );

  // Must add new (split) edges to sMasterEdges and sAddedEdges.
  sMasterEdges.Append( sNewEdges );
  sAddedEdges .Append( sNewEdges2 );


#ifdef SM_DEBUG_CODE
  sDebugEdges = sNewEdges; sDebugEdges.Append( sNewEdges2 );
  if (bDoGraphics) {
      DebugDraw(pMasterBrep, 0, &sMasterEdges, 0, 0, &sAddedEdges, 0, 0, &sDebugEdges, &sNewVerts);
  }
  tEnd = clock();
  dTimeTemp = ( tEnd - tStart ) / 1000.0;
  dTime5 += dTimeTemp;

  tStart = clock();
#endif // SM_DEBUG_CODE


/* ****    Edge-Face xSect:    *********************** */

  // Now look for edges intersecting faces, in both of their interiors.

  // Same tolerances as for Vertex-Face.
  // dThisTol = d3dTol * 10;
  switch ( cbiToleranceSetting )
  {
    case 0: { dThisTol = d3dTol *   1;  break; }
    case 1: { dThisTol = d3dTol *   4;  break; }
    case 2: { dThisTol = d3dTol *  10;  break; }
    case 3: { dThisTol = d3dTol *  25;  break; }
  }
  rTopoInt.SetThisApproxTol3d( dThisTol );

  // First, Added edges intersecting Master faces.

  rTopoInt.EdgeFaceIntersect(
        sAddedEdges,     // in: unchanged
        sMasterFaces,    // in: unchanged
        sNewVerts,       // out: new vertices.
        sSplitEdges,     // out: originals that were split.
        sNewEdges,       // out: new, created by split.
        sProcessedFaces, // out: contain a new vertex.
        dThisGap,
        dThisMinUnstitched,

        m_bMakingManifoldSolid,
        m_bIgnoreProblems
    );

  rdMaxEdgeGap       = smos_Max( rdMaxEdgeGap,       dThisGap );
  rdMinUnstitchedEdgeGap = smos_Min( rdMinUnstitchedEdgeGap, dThisMinUnstitched );

  sProcessedVerts.Append( sNewVerts );

  // Must add new (split) edges to sAddedEdges.
  sAddedEdges.Append( sNewEdges );

  //cbi: mapping? It would have to be 1 -> 2 whenever an added edge is split...

#ifdef SM_DEBUG_CODE
  if (bDoGraphics) {
      // Display only the new verts.
      DebugDraw(0, 0, 0, 0, 0, 0, 0, 0, &sNewEdges, &sNewVerts);
  }
#endif // SM_DEBUG_CODE

  // Now, original Master edges intersecting Added faces.

  rTopoInt.EdgeFaceIntersect(
        sMasterEdges,    // in: unchanged
        sAddedFaces,     // in: unchanged
        sNewVerts,       // out
        sSplitEdges,     // out
        sNewEdges,       // out
        sProcessedFaces, // out
        dThisGap,
        dThisMinUnstitched,

        m_bMakingManifoldSolid,
        m_bIgnoreProblems
    );

  rdMaxEdgeGap       = smos_Max( rdMaxEdgeGap,       dThisGap );
  rdMinUnstitchedEdgeGap = smos_Min( rdMinUnstitchedEdgeGap, dThisMinUnstitched );

  sProcessedVerts.Append( sNewVerts );

  // Must add new (split) edges to sMasterEdges.
  sMasterEdges.Append( sNewEdges );

//cbi: mapping?

#ifdef SM_DEBUG_CODE
  if (bDoGraphics) {
      // Display only the new verts.
      DebugDraw(0, 0, 0, 0, 0, 0, 0, 0, 0, &sNewVerts);
  }
#endif // SM_DEBUG_CODE



  // Note: At this point we are done with the sMasterEdges and sAddedEdges lists
  // so get rid of them, in case anything within them gets deleted,
  // so that we don't have to worry about using stale pointers.
  // We will still need the Face lists though.
  sMasterEdges.ReSet();
  sAddedEdges .ReSet();



#ifdef SM_DEBUG_CODE
  tEnd = clock();
  dTimeTemp = ( tEnd - tStart ) / 1000.0;
  dTime6 += dTimeTemp;

  tStart = clock();
#endif // SM_DEBUG_CODE



/* ****    Edge-Face Coinc:    *********************** */

  // Now check for Edges of one Brep lying in the interior of a face of the other.
  //
  // The edges must have (both) vertices lying in a face of the other,
  // which must both have been processed, so look at only processed vertices.
  // Also, if an edge has been glued, then it was originally coincident with
  // an edge, and so won't be in the interior of a face, so we can skip those.
  // Also, don't check against a face that already contains the edge.
  // So: for each prpocessed vertex,
  //       for each of its edges,
  //         if edge is not glued, and its other vertex is processed,
  //            for each face that contains both vertices,
  //               if the face doesn't contain the edge,
  //                  check for edge/face coincidence
  //
  // Note: Existing edges that are imprinted into a face are added
  // to sGluedEdges, and are used when checking Face-Face coincidence.



  // Same comment here as for Edge-Edge Coinicidence:
  // Corresponding vertices are already glued, so if the edge and face are
  // anywhere near each other, they're probably meant to be glued.
  // So a bigger tol is in order.
  // dThisTol = d3dTol * 100;
  switch ( cbiToleranceSetting ) //cbi: these are indeed bigger, per comment above.
  {
    case 0: { dThisTol = d3dTol *   2;  break; }
    case 1: { dThisTol = d3dTol *   8;  break; }
    case 2: { dThisTol = d3dTol *  20;  break; }
    case 3: { dThisTol = d3dTol *  50;  break; }
  }
  rTopoInt.SetThisApproxTol3d( dThisTol );

  SmTArray< SmFace* > sImprintedFaces, sNewFaces, sSplitFaces;

  rTopoInt.EdgeFaceCoincidence(
            sProcessedVerts,  // in: these will bound any coincident Edges.
            sGluedEdges,      // in: don't imprint these.
            sSplitEdges,      // out: edges imprinted into a face.
            sImprintedFaces,  // out: faces with edges imprinted in them, including new faces.
            sNewFaces,        // out: if an imprinted edge split a face.
            sSplitFaces,      // out: what sNewFaces were split from.

            dThisGap,         // out
            dThisMinUnstitched,   // out

            m_bIgnoreProblems,
            m_bDoRegionNesting
         );

  rdMaxEdgeGap       = smos_Max( rdMaxEdgeGap,       dThisGap );
  rdMinUnstitchedEdgeGap = smos_Min( rdMinUnstitchedEdgeGap, dThisMinUnstitched );


  // Imprinted edges get added to GluedEdges list.
  // Since it didn't imprint any already-glued edges, we can just append:
  sGluedEdges.Append( sSplitEdges );


  // Also, if a face was split,
  // add the new face to sMasterFaces or sAddedFaces as appropriate.
  ULONG lIdx, lNumSplit = sSplitFaces.GetSize();
  SM_ASSERT( lNumSplit == sNewFaces.GetSize() );
  for ( ii = 0; ii < lNumSplit; ii++ )
  {
      SmFace *pSplitFace = sSplitFaces[ ii ];
      if ( sMasterFaces.FindElement( pSplitFace, lIdx ) )
      {
          sMasterFaces.Add( sNewFaces[ ii ] );
      }
      else if ( sAddedFaces.FindElement( pSplitFace, lIdx ) )
      {
          sAddedFaces.Add( sNewFaces[ ii ] );
      }
  }


#ifdef SM_DEBUG_CODE
  if (bDoGraphics) {
      DebugDraw(pMasterBrep, &sSplitFaces, 0, 0, 0, &sSplitEdges, 0, &sNewFaces, 0, 0);
  }
  tEnd = clock();
  dTimeTemp = ( tEnd - tStart ) / 1000.0;
  dTime7 += dTimeTemp;

  tStart = clock();
#endif // SM_DEBUG_CODE



/* ****    Face-Face xSect:    *********************** */

  // smaller tol for srf-srf intersection:
  // dThisTol = d3dTol;
  switch ( cbiToleranceSetting )
  {
    case 0: { dThisTol = d3dTol *   1;  break; }
    case 1: { dThisTol = d3dTol *   4;  break; }
    case 2: { dThisTol = d3dTol *  10;  break; }
    case 3: { dThisTol = d3dTol *  25;  break; }
  }
  rTopoInt.SetThisApproxTol3d( dThisTol );

  rTopoInt.FaceFaceIntersect(
            sProcessedVerts, // in: intersections will start and end on these.
            sMasterFaces,    // in: don't intersect two faces that are in
            sAddedFaces,     // in:     the same list.

            sProcessedFaces, // out: faces that were intersected
            sNewFaces,       // out: if faces were split
            sNewEdges,       // out: on face/face intersections

            dThisGap,        // out
            dThisMinUnstitched,  // out

            m_bIgnoreProblems,
            m_bDoRegionNesting
         );

  rdMaxEdgeGap       = smos_Max( rdMaxEdgeGap,       dThisGap );
  rdMinUnstitchedEdgeGap = smos_Min( rdMinUnstitchedEdgeGap, dThisMinUnstitched );

  // Any new intersection edges go into sGluedEdges.
  sGluedEdges.Append( sNewEdges );

  // Add intersected faces to ImprintedFaces list.
  ULONG lNumImprinted = sProcessedFaces.GetSize();
  for ( ii = 0; ii < lNumImprinted; ii++ )
  {
      sImprintedFaces.AddUnique( sProcessedFaces[ii] );
  }
  lNumImprinted = sNewFaces.GetSize();
  for ( ii = 0; ii < lNumImprinted; ii++ )
  {
      sImprintedFaces.AddUnique( sNewFaces[ii] );
  }


#ifdef SM_DEBUG_CODE
  tEnd = clock();
  dTimeTemp = ( tEnd - tStart ) / 1000.0;
  dTime8 += dTimeTemp;

  tStart = clock();
#endif // SM_DEBUG_CODE



/* ****    Face-Face Coinc:    *********************** */

  // Now check for coincident faces.
  // For each glued edge,
  //  if any pair of its faces has the same set of edges in both faces,
  //  then check those faces for coincidence.

  // Tol: same as edge-edge and edge-face coincidence.
  // dThisTol = d3dTol * 100;
  switch ( cbiToleranceSetting ) //cbi: these are indeed bigger, per comment above.
  {
    case 0: { dThisTol = d3dTol *   2;  break; }
    case 1: { dThisTol = d3dTol *   8;  break; }
    case 2: { dThisTol = d3dTol *  20;  break; }
    case 3: { dThisTol = d3dTol *  50;  break; }
  }
  rTopoInt.SetThisApproxTol3d( dThisTol );

  SmTArray< SmFace* > sSurvivingFaces, sDeletedFaces;

  rTopoInt.FaceFaceCoincidence(
            sGluedEdges,      // in: coincident faces will contain these edges
            sAddedFaces,      // in: candidates for deletion.
            sSurvivingFaces,  // out: glued and preserved
            sDeletedFaces,    // out: glued and deleted

            dThisGap,         // out
            dThisMinUnstitched,   // out

            m_bIgnoreProblems,
            m_bDoRegionNesting
  );

  rdMaxEdgeGap       = smos_Max( rdMaxEdgeGap,       dThisGap );
  rdMinUnstitchedEdgeGap = smos_Min( rdMinUnstitchedEdgeGap, dThisMinUnstitched );

  // Add intersected faces to ImprintedFaces list.
  lNumImprinted = sSurvivingFaces.GetSize();
  for ( ii = 0; ii < lNumImprinted; ii++ )
  {
      sImprintedFaces.AddUnique( sSurvivingFaces[ii] );
  }
  // Also remove deleted faces from that list.
  lNumImprinted = sDeletedFaces.GetSize();
  for ( ii = 0; ii < lNumImprinted; ii++ )
  {
      SmFace *pF = sDeletedFaces[ii];
      if ( sImprintedFaces.FindElement( pF, lIdx ) )
        { sImprintedFaces.RemoveAt( lIdx ); }
  }

  // Update mapping.
  // Same thing here as after stitching vertices, and edges: Faces in the
  // mapping may have been deleted, so mapping needs to be updated.

  ULONG lNumDeleted = sDeletedFaces.GetSize();

  // If for some reason the deleted and surviving sizes are not the same,
  // the following loop would crash.  Flag a warning and skip the loop.
  if ( lNumDeleted != sSurvivingFaces.GetSize() )
  {
      SM_ASSERT( FALSE );
      lNumDeleted = 0; // prevents the following loop...
  }

  for ( ii = 0; ii < lNumDeleted; ii++ )
  {
      SmFace   * pDelFace = sDeletedFaces[ ii ];
      SmObject * pTmpObj  = rTopoInt.GetOtherMate( pDelFace );
      SmFace   * pThisFacePtr = SM_CAST_PTR( SmFace, pTmpObj );
      if ( pThisFacePtr != NULL )
      {
          rTopoInt.RemoveRelationship( pDelFace, pThisFacePtr );
          rTopoInt.Relate( sSurvivingFaces[ ii ], pThisFacePtr );
      }

      // Also while we're here, add to ProcessedFace list.
      sProcessedFaces.AddUnique( sSurvivingFaces[ ii ] );
  }


#ifdef SM_DEBUG_CODE
  tEnd = clock();
  dTimeTemp = ( tEnd - tStart ) / 1000.0;
  dTime9 += dTimeTemp;

  tStart = clock();
#endif // SM_DEBUG_CODE

  // ****************************************************

  // Done with combining topology.
  // Fix up region nesting.

  SmFaceuse *pFU1 = NULL, *pFU2 = NULL;
  SmShell   *pNewSh = NULL;
  SmRegion  *pNewReg = NULL;
  lNumImprinted = sImprintedFaces.GetSize();
  for ( ii = 0; ii < lNumImprinted; ii++ )
  {
      SmFace *pFace = sImprintedFaces[ii];
      if ( pFace == NULL ) { continue; }

      pFace->GetFaceuses( pFU1, pFU2 );

      // If they don't already separate two regions:
      if ( pFU1->GetShell() == pFU2->GetShell() )
      {
          pMasterBrep->TestClosureSplitRegion( TRUE, pFU1, pNewSh, pNewReg );
      }
  }

  pMasterBrep->FindAndSetInfiniteRegion();

#ifdef SM_DEBUG_CODE
  tEnd = clock();
  dTimeTemp = ( tEnd - tStart ) / 1000.0;
  dTime10 += dTimeTemp;

  tStart = clock();
#endif // SM_DEBUG_CODE


  // If the resulting Brep is Manifold, orient its faces.
  // If it's not Manifold, then see about intersecting the interiors of faces.
  if ( pMasterBrep->IsManifoldSolid() )
  {
      SmBoolean bMaybeNotClosed;
      pMasterBrep->OrientTrimmedSurfaces( TRUE, bMaybeNotClosed, TRUE );  // increments unlocked mark value
  }
  else
  {
      // See about intersecting the interiors of faces.
      //cbi SmFace::FaceIntersect() does a lot.
  }

  rnNumEdgesStitched = sGluedEdges.GetSize();

  pMasterBrep->Notify( SM_NO_POST_EDIT, pMasterBrep, NULL, NULL );

  // Note: the method ends here, the rest is just Debug.



#ifdef SM_DEBUG_CODE
  SmBoolean bDebugDump = FALSE;
  if ( bDebugDump ) {
      pMasterBrep->Dump( _T("SMS_AddInOneBrep after Stitch:") );
  }

  if ( FALSE ) { // Inspect Regions and Shells.
      SmTArray< SmRegion* > sRegions;
      SmTArray< SmShell * > sShells;
      pMasterBrep->GetRegions( sRegions );
      pMasterBrep->GetShells ( sShells  );
      smgfx_Erase();
      smgfx_SetLook( 1,3, 0,0,0 ); pMasterBrep->Draw(1); sm_GraphicsLoop();

      static constexpr SmBoolean bRefresh = TRUE;
      for ( ii = 0; ii < sRegions.GetSize(); ii++ ) {
          if ( bRefresh ) {
              smgfx_Erase();
              smgfx_SetLook( 1,3, 0,0,0 ); pMasterBrep->Draw(1); sm_GraphicsLoop();
          }
          SmRegion *pReg = sRegions[ii];
          smgfx_SetLook( 1,3, 0,1,0 ); pReg->Draw(); sm_GraphicsLoop();
          sm_GraphicsLoop();
      }

      for ( ii = 0; ii < sShells.GetSize(); ii++ ) {
          if ( bRefresh ) {
              smgfx_Erase();
              smgfx_SetLook( 1,3, 0,0,0 ); pMasterBrep->Draw(1); sm_GraphicsLoop();
          }
          SmShell *pSh = sShells[ii];
          smgfx_SetLook( 1,3, 0,1,0 ); pSh->Draw(); sm_GraphicsLoop();
          sm_GraphicsLoop();
      }
      sm_GraphicsLoop();
  }
  if ( iBrepChecks > 0 )
  {
      bResultBrepCheckOK = pMasterBrep->AssertValid();

      if ( ! bResultBrepCheckOK )
      {
          SM_ASSERT_MSG( FALSE, _T("SMS_AddInOneBrep: Invalid Result Brep") );
          if ( bMasterBrepCheckOK && bThisBrepCheckOK )
          {
              // We have damaged it.
              if ( iBrepChecks > 1 ) // Breakpoint.
                { return SM_ERR; }
          }
      }
  }

  if ( bWriteFiles )
  {
      pMasterBrep->WriteToFile( _T("C:/Bill/v14Dev/result.smb") );
  }

  tEnd = clock();
  dTimeTemp = ( tEnd - tStart ) / 1000.0;
  dTime11 += dTimeTemp;

  if ( FALSE ) {
      TCHAR sBuff[SM_TBLOCK_SIZE];
      smos_sprintf( sBuff, _T("MergeAddedTopology Timings: %9.3f %9.3f %9.3f %9.3f %9.3f %9.3f\n"),
              dTime0, dTime1, dTime2, dTime3,  dTime4,  dTime5 );
      MYPRINTF( sBuff );
      smos_sprintf( sBuff, _T("                            %9.3f %9.3f %9.3f %9.3f %9.3f %9.3f\n"),
              dTime6, dTime7, dTime8, dTime9,  dTime10,  dTime11 );
      MYPRINTF( sBuff );
  }
#endif // SM_DEBUG_CODE

  return SM_SUCCESS;

} // end SmMerge::MergeAddedTopology

/*******************************************************************//**
PURPOSE: Merge an edge from the other brep into the brep.

NOTES:
  If input pEdgeuseInOther->Edge does not have a mated BrepEdge,
    A new wire Edge is created and added to Brep.

  The OtherBrepEdge is Notified that its been copied - to propagate attributes
  A common tolerance is selected for the mated edges.

  Other Side Effects:
    1. OtherBrepEdge curve is copied if it has no mate in Brep or Composite Edges are disabled
    2. OtherBrepEdge vertices without Brep mates are merged into Brep
    3. returned rpEdgeInBrep is a wire edge connected to a Brep Curve and Brep Vertices

***********************************************************************/
SmStatus SmMerge::MergeEdge
  (SmEdgeuse * pEdgeuseInOther,  // in : Edgeuse from OtherBrep
   SmRegion  * pRegionInBrep,    // in : region in Brep containing otherBrep edge
   SmEdge   *& rpEdgeInBrep)     // out: new matching edge Brep
{
    // init output
    rpEdgeInBrep = NULL;

    if ( pEdgeuseInOther == NULL )
      {
        SM_ASSERT_MSG(FALSE, _T("Null Edgeuse passed to SmMerge::MergeEdge()"));
        return SM_SUCCESS;  // The caller knows better whether this is a problem.
      }

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
static ULONG lCount      = 1 ; lCount++ ;
static ULONG lDebugCount = 0 ;
  if (bDebugMe || lCount == lDebugCount) {
      smgfx_Erase();
      smgfx_SetLook(3,5, 1,0,0); pEdgeuseInOther->Draw(); sm_GraphicsLoop();
      smgfx_SetLook(1,2, 0,0,1); m_vTI.m_pBrep  ->Draw(); sm_GraphicsLoop();
      sm_GraphicsLoop();
  }
#endif // SM_DEBUG_CODE

    // get context for new object construction
    const SmContext *pContext = m_vTI.m_pBrep->GetContext();

    // get target edge
    SmEdge *pOtherEdge = pEdgeuseInOther->GetEdge();

    // get Brep matching edge (probably NULL)
    SmEdge *pBrepEdge  = (SmEdge*)m_vTI.GetBrepMate(pOtherEdge);

    // Make Brep wire edge if it does not already exist
    if (!pBrepEdge)
      {
        // get otherBrep curve and matching Brep curve (commonly NULL)
        SmCurve  *pCurve     = pOtherEdge->GetCurve();
        SmObject *pBrepCurve = m_vTI.GetBrepMate(pCurve);
        SmCurve  *pNewCurve  = SM_CAST_PTR(SmCurve,pBrepCurve);

       // if BrepCurve is NULL
// Remove Composites
//       // if BrepCurve is NULL or if composites are disallowed
//       if (!(pNewCurve && m_vTI.m_pBrep->m_bMakeComposites))
       if(pNewCurve == NULL)
          {
            // copy the curve and add pair to entity maps
            SER(pCurve->Copy(*pContext,pNewCurve));
            if (pBrepCurve == NULL)
              {
                SER(m_vTI.Relate(pNewCurve,pCurve));
              }
          } // end need to copy pCurve for BrepEdge check

        // get OtherBrep edge->vertices
        SmVertex *pV  = pEdgeuseInOther->GetVertexuse()->GetVertex();
        SmVertex *pV2 = pOtherEdge->GetOtherVertex(pV);

        // Swap vertices if the edgeuse is oriented opposite to the edge.
        if (pEdgeuseInOther->GetOrientation() == SM_OT_OPPOSITE)
          {
            SmVertex *pTempV = pV;
            pV               = pV2;
            pV2              = pTempV;
          }

        // Get or make matching Brep vertices
        SmVertex *pBV = (SmVertex*)m_vTI.GetBrepMate(pV);
        if (!pBV) { SER(MergeVertex(pV,pRegionInBrep,pBV));
                  }
        NER(pBV);

        // Get or make matching Brep vertices
        SmVertex *pBV2 = (SmVertex*)m_vTI.GetBrepMate(pV2);
        if (!pBV2) { SER(MergeVertex(pV2,pRegionInBrep,pBV2));
                   }
        NER(pBV2);

        // make the new Brep wire edge
        SER(m_vTI.m_pBrep->MakeWireEdge(pRegionInBrep,pBV,pBV2,pNewCurve,
            pOtherEdge->GetInterval(),SM_OT_SAME,pBrepEdge));

        // mate new edge with input otherBrep edge
        SER(m_vTI.Relate(pBrepEdge,pOtherEdge));
      }

    // match tolerances for the mated edges
    if (pOtherEdge->GetTolerance() > pBrepEdge->GetTolerance())
      {
#ifdef SM_USE_OLDTOL
        SM_OLDTOL_LINE pBrepEdge->SetTolerance(pOtherEdge->GetTolerance());
#endif // SM_USE_OLDTOL
      }

    // notify OtherBrepEdge of the copy (so it can propagate attributes as needed)
    pOtherEdge->Notify(SM_NO_COPY, pBrepEdge, SM_NO_GET_BREP(pBrepEdge), SM_NO_GET_BREP(pOtherEdge));

    // set outputs
    rpEdgeInBrep = pBrepEdge;

    return SM_SUCCESS;

} // SmMerge::MergeEdge

/*******************************************************************//**
PURPOSE: Merge a vertex from otherBrep into Brep as a Shell Vertex.

NOTES:
  1. Returned vertex and input vertex are mated
  2. Returned vertex shares common tolerance with input vertex
  3. input vertex notified that it was copied to output vertex - propagates attributes
***********************************************************************/
SmStatus SmMerge::MergeVertex
  (SmVertex *pVertexInOther,      // in : target OtherBrep Vertex to merge
   SmRegion *pRegionInBrep,       // in : Brep Region containing pVertexInOther
   SmVertex *& rpVertexInBrep)    // out: New Brep Vertex
{
  // init output
  rpVertexInBrep = NULL;

  if ( pVertexInOther == NULL )
    {
      SM_ASSERT_MSG(FALSE, _T("Null Vertex passed to SmMerge::MergeVertex()"));
      return SM_SUCCESS;  // The caller knows better whether this is a problem.
    }

  // get vertex Brep mate (commonly NULL)
  SmVertex *pBrepVertex = (SmVertex*)m_vTI.GetBrepMate(pVertexInOther);

  // when pVertexInOther has no Brep mate
  if (!pBrepVertex)
    {
      // Make shell vertex in Brep
      SmShell *pNewShell = NULL;
      SER(m_vTI.m_pBrep->MakeShellVertex(pRegionInBrep,
          pVertexInOther->GetPoint(),pNewShell,pBrepVertex));
      SER(m_vTI.Relate(pBrepVertex,pVertexInOther));
    }

  // get a common tolerance for the mated vertices
  if (pVertexInOther->GetTolerance() > pBrepVertex->GetTolerance())
    {
#ifdef SM_USE_OLDTOL
      SM_OLDTOL_LINE pBrepVertex->SetTolerance(pVertexInOther->GetTolerance()) ;
#endif // SM_USE_OLDTOL
    }

  // notify input vertex that it has been copied to propagate attributes
  pVertexInOther->Notify(SM_NO_COPY, pBrepVertex, SM_NO_GET_BREP(pBrepVertex), SM_NO_GET_BREP(pVertexInOther)) ;

  // set output
  rpVertexInBrep = pBrepVertex;

  return SM_SUCCESS;

} // end SmMerge::MergeVertex

/*******************************************************************//**
PURPOSE: Make a face from the OtherBrep in m_vTI.m_pBrep.

NOTES:
***********************************************************************/
SmStatus SmMerge::MakeFaceuse
  (SmFaceuse         * pFaceuseToMakeInBrep,   // in : OtherBrep face to add to Brep
   SmRegion          *&pOldRegion,             // out: Brep Region receiving NewFace
   SmRegion          *&pNewRegion,             // out: New Region made when adding NewFace splits OldRegion
   SmRegion          * pOptRegion,             // in : Optional this Brep region known to contain input faceuse->face
   SmTArray<SmFace*> * pNewFaces)              // out: Newly constructed Face - used for LocalMerge, NULL to ignore
{
  // init output
  if(pNewFaces) pNewFaces->ReSet() ;
  pOldRegion = NULL ;
  pNewRegion = NULL ;

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe  = FALSE;
SmBoolean bDebugMe2 = FALSE;
static ULONG lCount      = 1 ; lCount++ ;
static ULONG lDebugCount = 0 ;
    // draw target faceuse(red),
    if (bDebugMe2 || lCount == lDebugCount)
      {
        if(pOptRegion) { pOptRegion->DumpTopology() ;
                         SM_ASSERT_VALID(pOptRegion) ;
                       }
        smgfx_Erase();
        smgfx_SetLook(1,2, 0,0,1); m_vTI.m_pBrep->Draw(TRUE); sm_GraphicsLoop();
        smgfx_SetLook(2,4, 0,1,0); pFaceuseToMakeInBrep->Draw(SM_DM_CROSSHATCH,4,4); sm_GraphicsLoop();
        smgfx_SetLook(1,2, 1,0,1); if(pOptRegion)pOptRegion->Draw(); sm_GraphicsLoop();
        sm_GraphicsLoop();
      }
#endif // SM_DEBUG_CODE

    // If the Face of the input Faceuse bounds the other Brep's infinite region,
    // then set it up so that pFaceuseToMakeInBrep is the one on the side
    // of the infinite region: swap it with its mate if necessary.

    SmFaceuse *pFaceuseMate = pFaceuseToMakeInBrep->GetMate();

    SmRegion *pOtherInfRegion = m_vTI.m_pOther->GetInfiniteRegion();
    if (   pFaceuseMate->GetShell()->GetRegion()         == pOtherInfRegion
        && pFaceuseToMakeInBrep->GetShell()->GetRegion() != pOtherInfRegion )
      {
        pFaceuseToMakeInBrep = pFaceuseMate;
      }

    // Check to see if face is already mated to a Brep face
    SmFace *pFace = (SmFace*)m_vTI.GetBrepMate(pFaceuseToMakeInBrep->GetFace());

    // low work - faces already mated - notify of copy to propagate attributes and return
    if (pFace)
      {
        pFaceuseToMakeInBrep->GetFace()->Notify(SM_NO_COPY, pFace, SM_NO_GET_BREP(pFace), SM_NO_GET_BREP(pFaceuseToMakeInBrep->GetFace()));
        return SM_SUCCESS;
      }

    // context for new object construction
    const SmContext *pContext = m_vTI.m_pBrep->GetContext();

    SmEdgeuse * pData1[256];
    SmTArray<SmEdgeuse*> sEdgeuses(256,pData1);

    // when not given a Brep Region
    //  try to find containing Brep Region by classifying input face->edgeuses
    //  against sectors of mated edges  (this is quick)
    SmRegion *pRegion = pOptRegion;

    if ( pRegion == NULL )
      {
        // try finding region through topology pointers - else do a point classify
        pRegion = sm_FindBrepRegionOfOtherFace( pFaceuseToMakeInBrep, m_vTI );
      }

    // error - unable to classify input OtherBrep faceuse into a Brep region
    if ( pRegion == NULL )
      { SER(SM_ERR) ; }

#ifdef SM_DEBUG_CODE
    if (bDebugMe)
      {
        if(pRegion) { pRegion->DumpTopology() ;
                      SM_ASSERT_VALID(pRegion) ;
                    }
        m_vTI.Dump();

        ULONG lll;
        pFaceuseToMakeInBrep->GetEdgeuses(sEdgeuses);

        smgfx_Erase();
        smgfx_SetLook(1,2, 0,0,1); m_vTI.m_pBrep->Draw(TRUE); sm_GraphicsLoop();
        smgfx_SetLook(1,2, 0,1,0); m_vTI.m_pOther->Draw(TRUE); sm_GraphicsLoop();
        smgfx_SetLook(3,4, 1,0,0); for(lll=0; lll<sEdgeuses.GetSize(); lll++)
                                     {
                                       SmEdgeuse *pDrawEU = sEdgeuses[lll];
                                       pDrawEU->Draw(); sm_GraphicsLoop() ;
                                       pDrawEU->GetVertexuse()->GetVertex()->Draw(); sm_GraphicsLoop() ;
                                     }
        smgfx_SetLook(2,4, 0,0,0); pFaceuseToMakeInBrep->Draw(SM_DM_CROSSHATCH,10,10); sm_GraphicsLoop();
        smgfx_SetLook(4,5, 1,0,0); pRegion->Draw(); sm_GraphicsLoop();
        sm_GraphicsLoop();
      }
#endif // SM_DEBUG_CODE

    // Now merge all unrelated input face->vertices into Brep's Region as shell vertices
    //   Later we will try a little more efficient way of doing it by making
    //     a vertex and a wire edge together.

    SmEdgeuse   *pData7[256];
    SmVertex    *pData2[256];
    SmLoopuse   *pData3[256];
    ULONG        pData4[32];
    SmVertexuse *pData5[32];
    SmTArray<SmEdgeuse*>   sFaceEdgeuses(256,pData7);
    SmTArray<SmVertex*>    sLoopVertices(256,pData2);
    SmTArray<SmLoopuse*>   sLoopuses    (256,pData3);
    SmTArray<ULONG>        sEdgeLoops   (32, pData4);
    SmTArray<SmVertexuse*> sVertexuses  (32, pData5);

    // for every Faceuse->Loopuse
    pFaceuseToMakeInBrep->GetLoopuses(sLoopuses);
    ULONG jj, lNumLUs = sLoopuses.GetSize();
    for(jj=0; jj<lNumLUs; jj++)
      {
        SmLoopuse *pLU = sLoopuses[jj];

        // get this loopuse's edgeuses
        pLU->GetEdgeuses(sEdgeuses);

        // load edge counts into sEdgeLoops array
        if ( !pLU->IsVertexLoopuse() )  { sEdgeLoops.Add( sEdgeuses.GetSize() ); }

        // for every Loopuse->Vertexuse
        pLU->GetVertexuses(sVertexuses);
        ULONG kk, lNumVUs = sVertexuses.GetSize();
        for(kk=0; kk<lNumVUs; kk++)
          {
            SmVertex *pV          = sVertexuses[kk]->GetVertex();
            SmVertex *pBrepVertex = NULL ;

            // Merge the Vertex
            SER(MergeVertex(pV,pRegion,pBrepVertex));

            // load VertexLoopuses into sLoopVertices
            if (kk==0 && pLU->IsVertexLoopuse())
              {
                SM_ASSERT(pLU->GetVertexuse() == sVertexuses[0]) ;
                sLoopVertices.Add(pBrepVertex);
              }
          } // end iter every Loopuse->Vertexuse

#ifdef SM_VALIDATE_TOPOLOGY
        if (bDebugMe)
          {
            m_vTI.m_pBrep->ValidatePointers();
          }
#endif // SM_VALIDATE_TOPOLOGY

#ifdef SM_DEBUG_CODE
        if (bDebugMe)
          {
            if(pRegion) { pRegion->DumpTopology() ;
                          SM_ASSERT_VALID(pRegion) ;
                        }
            m_vTI.Dump();

            ULONG lll;
            pFaceuseToMakeInBrep->GetEdgeuses(sEdgeuses);

            smgfx_Erase();
            smgfx_SetLook(1,2, 0,0,1); m_vTI.m_pBrep->Draw(TRUE); sm_GraphicsLoop();
            smgfx_SetLook(1,2, 0,1,0); m_vTI.m_pOther->Draw(TRUE); sm_GraphicsLoop();
            smgfx_SetLook(3,4, 1,0,0); for(lll=0; lll<sEdgeuses.GetSize(); lll++)
                                         {
                                           SmEdgeuse *pDrawEU = sEdgeuses[lll];
                                           pDrawEU->Draw(); sm_GraphicsLoop() ;
                                           pDrawEU->GetVertexuse()->GetVertex()->Draw(); sm_GraphicsLoop() ;
                                         }
            smgfx_SetLook(2,4, 0,0,0); pFaceuseToMakeInBrep->Draw(SM_DM_CROSSHATCH,10,10); sm_GraphicsLoop();
            smgfx_SetLook(4,5, 1,0,0); pRegion->Draw(); sm_GraphicsLoop();
            sm_GraphicsLoop();
          }
#endif // SM_DEBUG_CODE

        // for every Loopuse->Edgeuse
        // Now pick up the appropriate edgeuses from the loopuse
        ULONG ll, lNumEUs = sEdgeuses.GetSize();
        for(ll=0; ll<lNumEUs; ll++)
          {
            SmEdgeuse *pEU    = sEdgeuses[ll];
            SmEdge    *pEdge  = pEU->GetEdge();
            SmEdge    *pBrepEdge = NULL;

#ifdef SM_VALIDATE_TOPOLOGY
            if (bDebugMe)
              {
                m_vTI.m_pBrep->ValidatePointers();
              }
#endif // SM_VALIDATE_TOPOLOGY

            // merge the edge
            SER(MergeEdge(pEU,pRegion,pBrepEdge));

            // Find the orientation of the Edgeuse to look for in the radial sector.
            // It should be opposite as pEU relative to the orientation of the vertices,
            // because it's in an adjacent face and will end up being Radial.

            SmVertex    * pV              = pEU->GetVertexuse()->GetVertex();
            SmVertex    * pV2             = pEdge->GetOtherVertex(pV);
            SmEdgeuse   * pPrimBEU        = pBrepEdge->GetPrimaryEdgeuse();
            SmVertex    * pVB             = pPrimBEU->GetVertexuse()->GetVertex();
            SmVertex    * pVB2            = pBrepEdge->GetOtherVertex(pVB);
            SmOrientType  eEUOrientToFind = SM_OT_UNKNOWN;
            if (pV == pV2)
              {
                // Here need to do additional work for closed loop stuff
                SmPoint3d sPnt;
                SmVector3d sTan1,sTan2, sTanRef;
                SER(pEU->NormalizedEvaluate(0.0,FALSE,sPnt,&sTan1));  // TRUE = UV Eval, FALSE = 3d Eval
                SER(pEU->NormalizedEvaluate(1.0,FALSE,sPnt,&sTan2));  // TRUE = UV Eval, FALSE = 3d Eval
                sTan2 = - sTan2;  // Pointing from the vertex into the curve...
                SER(pPrimBEU->NormalizedEvaluate(0.0,FALSE,sPnt,&sTanRef));  // TRUE = UV Eval, FALSE = 3d Eval
                SER(sTan1.Unitize());
                SER(sTan2.Unitize());
                SER(sTanRef.Unitize());
                if (sTan1.Dot(sTanRef) > sTan2.Dot(sTanRef))
                  {
                    eEUOrientToFind = pPrimBEU->GetOrientation();
                  }
                else
                  {
                    eEUOrientToFind = SM_REVERSE_ORIENTATION(pPrimBEU->GetOrientation());
                  }
              } // end pV == pV2 branch
            else
              {  // pV != pV2 branch
                SmVertex *pVBRel = (SmVertex*)m_vTI.GetBrepMate(pV); NER(pVBRel);
                if (pVB == pVBRel)
                  {
                    eEUOrientToFind = pPrimBEU->GetOrientation();
                  }
                else if (pVB2 == pVBRel)
                  {
                    eEUOrientToFind = SM_REVERSE_ORIENTATION(pPrimBEU->GetOrientation());
                  }
                else
                  {
                    SER(SM_ERR);
                  }
              } // end pV != pV2 Branch for setting eEUOrientToFind value

            // classify Edgeuse->Face against BrepEdge
            SmEdgeuse *pFoundEU = NULL;
            SmFaceuse *pFoundFU = NULL;
            SmStatus eStat = pBrepEdge->FindRadialSector(pEU,pRegion,eEUOrientToFind,pFoundEU,pFoundFU);
            if ( eStat != SM_SUCCESS  ||  !pFoundEU )
              { SER(SM_ERR); }

#ifdef SM_DEBUG_CODE
            if (bDebugMe2)
              {
                smgfx_Erase();
                smgfx_SetLook(1,2, 0,0,1); m_vTI.m_pBrep->Draw(TRUE); sm_GraphicsLoop();
                smgfx_SetLook(1,2, 0,1,0); m_vTI.m_pOther->Draw(TRUE); sm_GraphicsLoop();
                smgfx_SetLook(3,4, 1,0,0); pFoundEU->Draw(); sm_GraphicsLoop();
                smgfx_SetLook(3,4, 1,0,0); pFoundEU->GetVertexuse()->GetVertex()->Draw(); sm_GraphicsLoop();
                smgfx_SetLook(3,4, 1,1,0); pEU->Draw(); sm_GraphicsLoop();
                smgfx_SetLook(3,4, 1,1,0); pEU->GetVertexuse()->GetVertex()->Draw(); sm_GraphicsLoop();
//                pPrimBEU->Draw();
//                pPrimBEU->GetVertexuse()->GetVertex()->Draw();
//                pFoundEU->GetLoopuse()->Draw();
//                pFoundEU->GetLoopuse()->GetFaceuse()->GetFace()->DrawUV(10,10);
                sm_GraphicsLoop();
              }
#endif // SM_DEBUG_CODE
            // add the classified radial sector's Edgeuse to the sFaceEdgeuses list
            sFaceEdgeuses.Add(pFoundEU);
          } // end iter every loopuse->edgeuse
      } // end iter every Loopuse

#ifdef SM_DEBUG_CODE
    if (bDebugMe)
      {
        if(pRegion) { pRegion->DumpTopology() ;
                      SM_ASSERT_VALID(pRegion) ;
                    }
      }
#endif // SM_DEBUG_CODE

    SmFace    *pFaceToMake = pFaceuseToMakeInBrep->GetFace();
    SmSurface *pOldSurface = pFaceToMake->GetSurface();
    SmObject  *pBrepObj    = m_vTI.GetBrepMate(pOldSurface);
    SmSurface *pNewSurface = SM_CAST_PTR(SmSurface,pBrepObj);

    // if Brep Surface does not exist
// Remove Composites
//    // if Brep Surface does not exist or Composites are prohibited
//    if (!(pNewSurface && m_vTI.m_pBrep->m_bMakeComposites))
    if (pNewSurface == NULL)
      {
        // Copy pOldSurface, when possible as an analytic surface
        SER(pOldSurface->CopyAndAddAnalytics(*pContext,pNewSurface));
        NER(pNewSurface);
        m_vTI.Relate(pNewSurface,pOldSurface);

#ifdef SM_DEBUG_CODE
        // draw
        if(bDebugMe)
          {
            pOldSurface->Dump() ;
            pNewSurface->Dump() ;

            smgfx_Erase();
            smgfx_SetLook(1,2, 0,0,1); m_vTI.m_pBrep->Draw(TRUE); sm_GraphicsLoop();
            smgfx_SetLook(1,2, 0,1,0); m_vTI.m_pOther->Draw(TRUE); sm_GraphicsLoop();
            smgfx_SetLook(1,2, 0,1,1); pOldSurface->DrawUV(5,5); sm_GraphicsLoop();
            smgfx_SetLook(1,2, 1,1,0); pNewSurface->DrawUV(7,7); sm_GraphicsLoop();
            sm_GraphicsLoop();
          }
#endif // SM_DEBUG_CODE

      } // end need to make Surface copy check

    // locals
    SmFace   *pNewFace = NULL;
    SmShell  *pNewShell = NULL;

    // If we have the case of a faceuse bordering an infinite region in the
    // other brep being merged into the infinite region in this brep, we can
    // skip a lot of work in MakeFace().  If that's the case, temporarily set
    // Brep's infinite region to NULL.
    SmRegion *pSavedInfiniteRegion = NULL;

    // Check that condition:
    if (   pRegion == m_vTI.m_pBrep->GetInfiniteRegion()
        && pFaceuseToMakeInBrep->GetShell()->GetRegion() == m_vTI.m_pOther->GetInfiniteRegion()
        && pFaceuseToMakeInBrep->GetMate()->GetShell()->GetRegion() != m_vTI.m_pOther->GetInfiniteRegion())
      {
        // new face is partitioning the Brep infinite region - clear that pointer for now
        pSavedInfiniteRegion = m_vTI.m_pBrep->GetInfiniteRegion();
        m_vTI.m_pBrep->SetInfiniteRegion(NULL);
      }

    // make the new Brep Face within the classified Brep region
    //    connecting to the geometry and boundary that
    //    has already been mated to the input OtherBrep face boundaries and geometry

    SmStatus eStat = m_vTI.m_pBrep->MakeFace
                      (pRegion,                                // in : Region to contain face. Edges and verts must in this region.
                                                               //      NULL for infinite region.
                       sEdgeLoops,                             // in : Number of edges in each loop.  One entry per loop, outer loop 1st.
                       sFaceEdgeuses,                          // in : New Face edgeuses. ordered:[OuterLoop=counter clockwise, InnerLoop=clockwise]
                                                               //      relative to surface normal as modified by the surface orientation below.]
                       sLoopVertices,                          // in : Any vertices to be made into LoopVertices (not connected to edges) in NewFace
                       pNewSurface,                            // in : NewFace->Surface. Edge must lie on this surface (to within tolerance).
                                                               //      If surface has an owner, owner must be a face or composite face in this brep.
                       pFaceToMake->GetUVDomain(),             // in : Surface domain of interest for making NewFace.
                       pFaceuseToMakeInBrep->GetOrientation(), // in : SM_OT_SAME     = OuterLoop runs counterclockwise on PSurface to  pSurface->Normal
                                                               //      SM_OT_OPPOSITE = OuterLoop runs counterclockwise on PSurface to -pSurface->Normal
                       pNewRegion,                             // out: If the face splits a region, then this is the newly created region.
                       pNewShell,                              // out: If the face splits a region, then this is the newly created shell.
                       pNewFace) ;                             // out: the newly created face.
                                                               // in : TRUE = For nonManifold Models - verify the new face is inserted to proper
                                                               //             radial sectors on each edge
                                                               //      FALSE= Save time, the sectors are known to be good, don't check
                                                               //      default:[FALSE]
                                                               // in : TRUE = attach or create UVTrimCurve to every Edgeuse, FALSE=don't
                                                               //      default:[FALSE]
                                                               // in : optional array of UVTrimCurves to assign to NewFace->Edgeuses, default:[NULL]
    if ( pSavedInfiniteRegion != NULL ) 
      {
        m_vTI.m_pBrep->SetInfiniteRegion(pSavedInfiniteRegion);
      }

#ifdef SM_DEBUG_CODE
    // dump
    if(bDebugMe)
      {
        if(pRegion) pRegion->DumpTopology() ;
        if(pNewRegion) pNewRegion->DumpTopology() ;
        if(pNewShell) pNewShell->DumpTopology() ;
      }
#endif // SM_DEBUG_CODE

    if ( eStat != SM_SUCCESS )
      { SER( eStat ); }

    SER( m_vTI.Relate( pNewFace, pFaceToMake ));

    pFaceToMake->Notify(SM_NO_COPY, pNewFace, SM_NO_GET_BREP(pNewFace), SM_NO_GET_BREP(pFaceToMake)) ;

    // set output
    pOldRegion = pRegion;
    if (pNewFaces) { pNewFaces->Add(pNewFace) ; } // LocalMerge

    // all done
    return SM_SUCCESS;

} // end SmMerge::MakeFaceuse

/*******************************************************************//**
PURPOSE: Do merging and removal of wireframe geometry (wire edges and
    shell vertices) which do not have corresponding intersections.

NOTES:
  1. This function assumes it is being called after IntersectInsertRelate()
     so that every wire/PartnerBrep intersection already has a common
     vertex that is currently marked.
     When a wire from either Brep is classified against
     the partner Brep, it is expected to either classify to a region or
     be coincident with an edge or a face.

  2. Classify Brep  WireEdges and ShellVerticies to decide which to get rid of
     Classify OBrep WireEdges and ShellVerticies to decide which to merge into Brep
***********************************************************************/
SmStatus SmMerge::ProcessWiresAndShellVertices
  (SmBooleanOperationType eOperation,  // in : one of SM_BO_UNION
                                       //             SM_BO_INTERSECTION
                                       //             SM_BO_DIFFERENCE
                                       //             SM_BO_EXCLUSIVE_OR
                                       //             SM_BO_MERGE
                                       //             SM_BO_PARTIAL_MERGE
                                       //             SM_BO_EXTRACT_SEPARATE
                                       //             SM_BO_DIFFERENCE
   SmTArray<SmEdge*> &rKeepAsWires,    // out: list of all Brep Edges mated to
                                       //      Other wires that should be kept in the
                                       //      final output should all the Brep faces
                                       //      attached to these edges be deleted.
   SmMarkType         eBrepMarkType,   // in : // in : MarkType for m_vTI.m_pBrep  from call ManifoldBoolean(), not incremented
   SmMarkType         eOtherMarkType)  // in : // in : MarkType for m_vTI.m_pOther from call ManifoldBoolean(), not incremented
{
  // init output
  rKeepAsWires.ReSet() ;

  // locals
  ULONG ii, jj, lNumEdges, lNumVerts;
  SmTArray<SmEdge*> sEdges;
  SmTArray<SmEdge*> sCollectedEdges;

  // Now classify and merge non-intersecting wire edges
  //   - remove unneeded Brep wires from Brep topology graph and
  //   - merge needed otherBrep wires into Brep topology graph.

  // Make a list of all Brep Edges mapped to Other Brep wires
  //  to be used later during DeleteFaces() to make sure these
  //  edges are not deleted in the event that all the faces to
  //  which they are attached are deleted.

  // First remove unneeded wires from Brep.
  // Repeat process until no more wires are removed.
  SmBoolean bDone = FALSE;

  // Note, this loop does nothing if eOperation is not one of these:
  if (   eOperation != SM_BO_UNION
      && eOperation != SM_BO_INTERSECTION
      && eOperation != SM_BO_DIFFERENCE
      && eOperation != SM_BO_SLICE )
    {
      bDone = TRUE;
    }

  while (!bDone)
    {
      bDone = TRUE;

      // for every Brep edge
      m_vTI.m_pBrep->GetEdges(sEdges);
      lNumEdges = sEdges.GetSize();
      for(ii=0; ii<lNumEdges; ii++)
        {
          SmEdge    * pEdge          = sEdges[ii];
          SmZoneTol3d sEdgeZoneTol3d = SmTol::GetZoneTol3d(pEdge) ;
          sEdges[ii] = NULL ;

          // pEdge will be NULL if its already processed
          if(pEdge == NULL) continue ;

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
          if(bDebugMe)
            {
              smgfx_Erase() ;
              smgfx_SetLook(1,2, 0,0,1) ; m_vTI.m_pBrep->Draw() ; sm_GraphicsLoop() ;
              smgfx_SetLook(1,2, 0,1,0) ; m_vTI.m_pOther->Draw() ; sm_GraphicsLoop() ;
              smgfx_SetLook(3,4, 1,0,1) ; pEdge->Draw() ; sm_GraphicsLoop() ;
              sm_GraphicsLoop() ;
            }
#endif // SM_DEBUG_CODE

          // skip marked and nonWire edges (common edges are marked)
          if (   !pEdge->IsWire()
              ||  pEdge->IsMarked(eBrepMarkType)) { continue; }

          // Classify Brep wire MidPoint against otherBrep.
          SmPoint3d sPnt;
          SER( pEdge->GetCurve()->EvaluatePoint( pEdge->GetInterval().Evaluate(0.5), sPnt ));

          SmPointClassification sPointClass(sEdgeZoneTol3d, &m_vTI.GetContext()) ;  // should be sPnt ZoneTol3d
            {
              SmTemporaryChangeValue<SmBoolean> sChange(m_vTI.m_pOther->m_bEditingEnabled,FALSE);
              SER( m_vTI.m_pOther->Point3DClassify( sPnt, SM_EFF_ZERO, TRUE, sPointClass ));
            }

          // Get the otherBrep Region containing this wire.
          SmPointClassificationType eClassType = sPointClass.GetPointClass();

          SmRegion *pRegion = NULL;
          if ( eClassType == SM_PC_REGION )
            { pRegion = (SmRegion*)(sPointClass.GetObject()); }
          else if ( eClassType == SM_PC_FACE )
            {
              SmFace *pF = (SmFace*)(sPointClass.GetObject());
              pRegion = pF->GetUpwardFaceuse()->GetShell()->GetRegion();
            }
          else if ( eClassType == SM_PC_EDGE )
            {
              SmEdge *pE = (SmEdge*)(sPointClass.GetObject());
              pRegion = pE->GetPrimaryEdgeuse()->GetShell()->GetRegion();
            }
          else
            {
              // Expect wires to classify to regions
              // or to be coincident with faces or edges.
              SE( SM_ERR );
              SM_ASSERT( FALSE );
              continue;
            }

          // get all WireEdges connected to pEdge through vertex/edge connections.
          //   stops at marked vertices and all common vertices are currently marked
          //   so the connected wires won't cross shell boundaries.
          SmTopologyTraverser sTraverser;
          SER(sTraverser.CollectWireEdges(pEdge,           // in : target wire (gets marked)
                                          sCollectedEdges, // out: all wires connected to target wire (get marked)
                                          eBrepMarkType)); // in : specify mark for target objects (not incremented)

// GWC:TODO - extend logic to handle coincident cases
//            // decide if the wire is to be removed from Brep
//            SmBoolean bRemove = FALSE ;
//            switch(eOperation)
//              { case SM_BO_UNION            : // remove wires 'inside' otherBrep
//                                              if(   (eClassType == SM_PC_REGION && !pRegion->IsVoid())
//                                                 || eClassType == SM_PC_FACE
//                                                 || eClassType == SM_PC_EDGE)
//                                                 bRemove = TRUE ;
//                                              break ;
//                case SM_BO_DIFFERENCE       :
//                case SM_BO_INTERSECTION     :
//                case SM_BO_EXCLUSIVE_OR     :
//                case SM_BO_MERGE            :
//                case SM_BO_PARTIAL_MERGE    :
//                case SM_BO_EXTRACT_SEPARATE :
//                case SM_BO_SLICE            :
//
//
//              } // end switch on eOperation

          // if Brep wire is no longer needed
          //   - operation is union/difference in nonVoid region
          //               or intersection     in Void

          if (   (   (   eOperation == SM_BO_UNION
                      || eOperation == SM_BO_DIFFERENCE
                      || eOperation == SM_BO_SLICE )
                  && (!pRegion->IsVoid()))
              || (   eOperation == SM_BO_INTERSECTION
                  && pRegion->IsVoid()) )
            {
              // remove every EdgeWire connected to pEdge
              ULONG lNumCollEdges = sCollectedEdges.GetSize();
              for(jj=0; jj<lNumCollEdges; jj++)
                {
                  // locals
                  bDone           = FALSE;
                  SmEdge   *pE = sCollectedEdges[jj];
                  SmVertex *pV1   = pE->GetVertex();
                  SmVertex *pV2   = pE->GetOtherVertex(pV1);

                  // clear the processed pE from the sEdges array
                  ULONG lFoundIndex = 0 ;
                  if(sEdges.FindElement(pE, lFoundIndex))
                    {
                      sEdges[lFoundIndex] = NULL ;
                    }

                  // Edit Brep Topology Graph to remove pE - delete pE
                  SER(m_vTI.m_pBrep->DeleteEdge(pE));

                  // when pE->vertex1 was turned into a shell vertex
                  if (pV1->IsShellVertex())
                    {
                      // if vertex1 does not match an OVertex - no need to keep it - delete it
                      SmVertex *pOVertex = (SmVertex*)m_vTI.GetOtherMate(pV1);
                      if (!pOVertex)
                        {
                          SER(m_vTI.m_pBrep->DeleteVertex(pV1));
                        }
                    } // end need to delete vertex1 after it was turned into a shell check

                  // when distinct pE->vertex2 was turned into a shell vertex
                  if (pV1 != pV2 && pV2->IsShellVertex())
                    {
                      // if vertex2 does not match an OVertex - no need to keep it - delete it
                      SmVertex *pOVertex = (SmVertex*)m_vTI.GetOtherMate(pV2);
                      if (!pOVertex)
                        {
                          SER(m_vTI.m_pBrep->DeleteVertex(pV2));
                        }
                    } // end need to delete vertex2 after it was turned into a shell check
                } // end iter connected EdgeWire
            } // end need to remove Brep wire check

          // GWC:REMOVED_ONE_LINE breaking here turns this into
          // an N**2 operation on the number of wires.  We should
          // be able to go through all the wires before repeating whole loop.
          // gwc:remove_line: if (!bDone) break;

        } // end iter every Brep edge
    } // end while (!bDone)

  // 2nd merge every otherBrep needed wire into Brep
  m_vTI.m_pOther->GetEdges(sEdges);
  lNumEdges = sEdges.GetSize();
  for(ii=0; ii<lNumEdges; ii++)
    {
      SmEdge    * pEdge          = sEdges[ii];
      SmZoneTol3d sEdgeZoneTol3d = SmTol::GetZoneTol3d(pEdge) ;
      SmBoolean bIsWire = pEdge->IsWire() ;

      // when other wire is mapped to a Brep Edge
      if(   bIsWire
         && pEdge->IsMarked(eOtherMarkType))
        {
          SmObject *pBrepObject = m_vTI.GetThisMate(pEdge) ;
          SM_ASSERT(pBrepObject != NULL && pBrepObject->IsKindOf(SmEdge_TYPE)) ;
          SmEdge *pBrepEdge = (SmEdge*)pBrepObject ;

          // When the edge has to be in the output either as a wire
          //  or even merged into a face add it to rKeepAsWires
          switch (eOperation)
            {
              case SM_BO_UNION:
              case SM_BO_INTERSECTION:
              case SM_BO_MERGE:
              case SM_BO_PARTIAL_MERGE: rKeepAsWires.Add(pBrepEdge) ;
                                        break;
              case SM_BO_DIFFERENCE:
              case SM_BO_SLICE:
              case SM_BO_EXCLUSIVE_OR:
              case SM_BO_EXTRACT_SEPARATE:
                                        break ;
              case SM_BO_UNKNOWN:
              default: SE(SM_ERR) ;
            }

          // go to next edge
          continue ;
        }
      // skip marked and nonWire edges
      else if (   pEdge->IsMarked(eOtherMarkType)
               || !bIsWire)
        { continue; }

      // Classify otherBrep wire MidPoint against Brep.
      SmPointClassification sPointClass(sEdgeZoneTol3d, &m_vTI.GetContext()) ;  // should be sPnt ZoneTol3d
      SmPoint3d             sPnt;
      SER(pEdge->GetCurve()->EvaluatePoint(pEdge->GetInterval().Evaluate(0.5),sPnt));
        {
          SmTemporaryChangeValue<SmBoolean> sChange(m_vTI.m_pBrep->m_bEditingEnabled,FALSE);
          SER(m_vTI.m_pBrep->Point3DClassify(sPnt,SM_EFF_ZERO,TRUE,sPointClass));
        }

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
      if(bDebugMe)
        {
          smgfx_Erase() ;
          smgfx_SetLook(1,2, 0,0,1) ; m_vTI.m_pBrep->Draw() ; sm_GraphicsLoop() ;
          smgfx_SetLook(1,2, 0,1,0) ; m_vTI.m_pOther->Draw() ; sm_GraphicsLoop() ;
          smgfx_SetLook(3,4, 1,0,1) ; pEdge->Draw() ; sm_GraphicsLoop() ;
          sm_GraphicsLoop() ;
        }
#endif // SM_DEBUG_CODE

      // skip otherBrep wires that don't classify to a Brep region
      if (sPointClass.GetPointClass() != SM_PC_REGION)
        { SE(SM_ERR);
          continue;
        }

      // get the Brep region containing otherWire MidPoint
      SmRegion *pRegion = (SmRegion*)sPointClass.GetObject();

      // collect all wires connected to pEdge through vertex/edge connections
      //   stops at marked vertices and all common vertices are currently marked
      //   so the connected wires won't cross shell boundaries.
      SmTopologyTraverser sTraverser;
      SER(sTraverser.CollectWireEdges(pEdge,            // in : target wire (gets marked)
                                      sCollectedEdges,  // out: all wires connected to target wire (get marked)
                                      eOtherMarkType)); // in : specify mark for target objects (not incremented)

      // decide if this wire is kept or deleted
      SmBoolean bKeep = FALSE;
      switch (eOperation)
        {
          case SM_BO_UNION:         if (pRegion->IsVoid()) { bKeep = TRUE; }
                                    break;
          case SM_BO_DIFFERENCE:
          case SM_BO_SLICE:
          case SM_BO_INTERSECTION:  if (!pRegion->IsVoid()) { bKeep = TRUE; }
                                    break;
          case SM_BO_EXCLUSIVE_OR:
          case SM_BO_MERGE:
          case SM_BO_PARTIAL_MERGE: bKeep = TRUE;
                                    break ;
          case SM_BO_UNKNOWN:
          case SM_BO_EXTRACT_SEPARATE:
          default: SE(SM_ERR) ;
        }

      // for edges being kept
      if (bKeep)
        {
          SmBrepCache *pBrepCache = (SmBrepCache*)SmCacheMgr::GetObjectCache(SM_OC_BREP,m_vTI.m_pBrep);

          // when there is a BrepCache
          if(pBrepCache)
            {
              // to save time - get the bounding box of all these wires
              // and make sure the Brep->pBrepCache->GetCurveTree()
              // is sized to accomodate all the wires and their vertices being added
              pBrepCache->ReSizeTrees(NULL, &sCollectedEdges, NULL) ;

            } // end adding multiple wires to a Brep with a cache check

          // for every wire connected to pEdge wire
          ULONG lNumCollEdges = sCollectedEdges.GetSize();
          for(jj=0; jj<lNumCollEdges; jj++)
            {
              // merge the edge into the Brep model
              SmEdge *pE = sCollectedEdges[jj];
              SmEdge *pOEdge;
              SER(MergeEdge(pE->GetPrimaryEdgeuse(),pRegion,pOEdge));
            }
        } // end keep otherBrep Wire check
    } // end iter every otherBrep edge


  // Now look at marked shell vertices.
  // If they classify to a Region (as they should),
  // then possibly delete them, depending on the operation.

  SmTArray<SmVertex*> sVertices;

  // Note, this loop does nothing if eOperation is not one of these,
  // so skip it if not.  (Skip it by not filling up sVertices.)
  if (   eOperation == SM_BO_UNION
      || eOperation == SM_BO_INTERSECTION
      || eOperation == SM_BO_DIFFERENCE
      || eOperation == SM_BO_SLICE )
    {
      m_vTI.m_pBrep->GetVertices(sVertices);
    }

  lNumVerts = sVertices.GetSize();
  for(ii=0; ii<lNumVerts; ii++)
    {
      SmVertex  * pVertex          = sVertices[ii];
      SmZoneTol3d sVertexZoneTol3d = SmTol::GetZoneTol3d(pVertex) ;

      // skip marked or nonShell vertices
      if (   pVertex->IsMarked(eBrepMarkType)
          || !pVertex->IsShellVertex())
        { continue; }


      // class the Brep shellVertex against otherBrep
      SmPointClassification sPointClass(sVertexZoneTol3d, &m_vTI.GetContext()) ;
        {
          SmTemporaryChangeValue<SmBoolean> sChange(m_vTI.m_pOther->m_bEditingEnabled,FALSE);
          SER(m_vTI.m_pOther->Point3DClassify(pVertex->GetPoint(),SM_EFF_ZERO,FALSE,sPointClass));
        }

      //
      if (sPointClass.GetPointClass() != SM_PC_REGION)
        { SER(SM_ERR);
        }

      // get otherBrep region containing vertex
      SmRegion *pRegion = (SmRegion*)sPointClass.GetObject();

      // when Brep ShellVertex is not needed - remove it from Brep and delete it
      if (   (   (   eOperation == SM_BO_UNION
                  || eOperation == SM_BO_DIFFERENCE
                  || eOperation == SM_BO_SLICE )
              && (!pRegion->IsVoid()))
          ||(   eOperation == SM_BO_INTERSECTION
             && pRegion->IsVoid()) )
        {
          SER(m_vTI.m_pBrep->DeleteVertex(pVertex));
       }

    } // end iter every Brep Vertex looking for unneeded shell vertices to delete

  // Now iter every otherBrep vertex looking for needed shell vertices to merge into Brep.
  SmTArray<SmVertex*> sKeepOVertices ;
  SmTArray<SmRegion*> sKeepORegions ;
  m_vTI.m_pOther->GetVertices(sVertices);
  lNumVerts = sVertices.GetSize();
  for(ii=0; ii<lNumVerts; ii++)
    {
      SmVertex  * pVertex          = sVertices[ii];
      SmZoneTol3d sVertexZoneTol3d = SmTol::GetZoneTol3d(pVertex) ;
      // skip marked or nonShell vertices
      if (   pVertex->IsMarked(eOtherMarkType)
          || !pVertex->IsShellVertex())
        { continue; }


      // class otherBrep ShellVertex against Brep
      SmPointClassification sPointClass(sVertexZoneTol3d, &m_vTI.GetContext()) ;
        {
          SmTemporaryChangeValue<SmBoolean> sChange(m_vTI.m_pBrep->m_bEditingEnabled,FALSE);
          SER(m_vTI.m_pBrep->Point3DClassify(pVertex->GetPoint(),SM_EFF_ZERO,FALSE,sPointClass));
        }

      //
      if (sPointClass.GetPointClass() != SM_PC_REGION)
        { SER(SM_ERR); }

      // get Brep region containing Shell Vertex
      SmRegion *pRegion = (SmRegion*)sPointClass.GetObject();

      // decide if Shell Vertex is needed
      SmBoolean bKeep = FALSE;
      switch (eOperation)
        {
          case SM_BO_UNION:         if ( pRegion->IsVoid()) { bKeep = TRUE; }
                                    break;
          case SM_BO_DIFFERENCE:
          case SM_BO_SLICE:
          case SM_BO_INTERSECTION:  if (!pRegion->IsVoid()) { bKeep = TRUE; }
                                    break;
          case SM_BO_EXCLUSIVE_OR:
          case SM_BO_PARTIAL_MERGE:
          case SM_BO_MERGE:         bKeep = TRUE;
                                    break;
          case SM_BO_UNKNOWN:
          case SM_BO_EXTRACT_SEPARATE:
          case SM_BO_IMPRINT:
          case SM_BO_IMPRINT_CLASSIFY:
                                    break;
        }

      // when otherBrep Shell vertex is needed
      if (bKeep)
        {
          // keep list of other Brep vertices to merge into Brep
          sKeepOVertices.Add(pVertex) ;
          sKeepORegions. Add(pRegion) ;
        }
    }  // end iter every otherBrep vertex looking for needed shell vertices to merge into Brep

  // Merge the collected KeepOVertices into Brep.
  if ( sKeepOVertices.GetSize() > 0 )
    {
      SmVertex *pOVertex;

      SmBrepCache *pBrepCache = (SmBrepCache*)SmCacheMgr::GetObjectCache(SM_OC_BREP,m_vTI.m_pBrep);
      if(pBrepCache)
        {
          // save time - bump BrepCache->SpatialTree size up for all vertices at once,
          //             rather than one at a time.
          pBrepCache->ReSizeTrees(NULL, NULL, &sKeepOVertices) ;

        } // end adding multiple wires to a Brep with a cache check

      // for every keep vertex
      lNumVerts = sKeepOVertices.GetSize();
      for(ii=0; ii<lNumVerts; ii++)
        {
          // merge otherBrep shell vertex into Brep
          SER(MergeVertex(sKeepOVertices[ii],sKeepORegions[ii],pOVertex));
        }
    } // end if had KeepOVertices to merge into Brep.

  // all done
  return SM_SUCCESS;

} // end SmMerge::ProcessWiresAndShellVertices

/*******************************************************************//**
PURPOSE: Copy/Merge Attributes from OtherSrcRegions to Target ResultBrep Region

NOTES: Only run this method after The IsVoid flag is set on all ResultBrep Regions.

      removes temporary MergeRegionAttrib attributes from the ResultRegions 
                   - we're done with them
      

      The Boolean operation will create new ResultBrep Regions whose sources
      come for one or more ThisBreb Regions and one or more OtherBrep Regions.
      Some ResultBrep Regions are created by merging ThisBrep SourceRegions
      when the list of DeleteFaces is removed from the ResultBrep.  Attributes
      from the ThisBrep SourceRegions are propagated to the ResultBrep Region when
      the Notify(SM_MERGE_IN_BREP) call in turn calls SmAttribute::Merge().
      
      ResultBrep Regions are also created by splitting when the list of OtherKeepFaces
      is copied into the ResultBrep.  A call to Notify(SM_SPLIT_IN_BREP)
      propagates ThisBrep SourceRegion attribute data to the ResultBrep Region.  
      That mechanism does not propagate OtherBrep Region attribute data to the 
      ResultBrep region.  This method does the work to move
      OtherBrep SourceRegion attribute data to ResultBrep Regions.
      
      All ResultBrep Regions created by the Boolean operation have been
      given an MergeRegionAttrib (id=SM_AI_OTHER_REGION_ID) that maintains the lists
      of both the ThisBrep and OtherBrep SourceRegions.  The SM_AI_OTHER_REGION_ID 
      attribute also remembers the state for each SourceRegion as 1=Void, 2=Solid, 
      3=InfiniteRegion. The ResultBrep regions which are inherited without modification
      directly from the ThisBrep Region set will not have an SM_AI_OTHER_REGION_ID attribute
      and do not have the need to propagate OtherBrep->Region data.
      
      At the time this method is called, 
        1. ThisBrep SourceRegion attribute data has propagated to the ResultBrep regions,
             (some of it inappropriately) 
        2. ThisBrep SourceRegion pointers are stale,
        3. OtherBrep SourceRegion pointers are valid,
        4. OtherBrep SourceRegion atrribute data needs to be propagated to the ResultBrep.
        5. ResultBrep Region IsVoid bits have been set.

      Only Region Attributes that return TRUE for the SmAttribute::sPropagatedThroughBooleans()
      call are managed by this call.  (See SmColorAttribute class example).
      The propagation model for such attributes implemented in this model includes:

        a. Propagated attributes are not placed on ResultBrep Void Regions. If needed,
           such attributes propagated from ThisBrep SourceRegions through the Notify
           mechanism are removed.

        b. What is wanted is something like:
           if(ResultRegion_i->IsVoid) { No Attrib(ResultRegion_i) }
           else                       { Attrib(ResultRegion_i) = Merge(AllSolid(ResultRegion_i->ThisSrcRegions), 
                                                                       AllSolid(ResultRegion_i->OtherSrcRegions))
                                      }
        c. But due to the Notify Mechanism's use for SM_SPLIT_IN_BREP and SM_SPLIT_IN_MERGE,
           what was built::
           if(ResultRegion_i->IsVoid) { No Attrib(ResultRegion_i) }
           else                       { Attrib(ResultRegion_i) = Merge(All(ResultRegion_i->ThisSrcRegions), 
                                                                       AllSolid(ResultRegion_i->OtherSrcRegions))
                                      }
           I [gwc] decided that in most imaginable cases this will show the behavior desired and if
           a user case arises that requires a more sophisticated model, we'll add that sophistication 
           to this routing.

        d. And what is actually built branches on the eOperation value, IsVoid state
           and the aggregate isVoid state of all the OtherBrep SourceRegions.  Read the code
           to see how that was implemented.

***********************************************************************/
SmStatus SmMerge::PropagateOtherSrcRegionAttribs
 (SmBooleanOperationType eOperation)  // in : 
{
  // locals
  ULONG ii, jj ;
  SM_PTR_ARRAY(sRegions, SmRegion, 16) ;

  // get current ResultBrep Regions
  m_vTI.m_pBrep->GetRegions(sRegions) ;

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  if(bDebugMe)
    {
      TCHAR sBuff[SM_TBLOCK_SIZE];
      smos_sprintf( sBuff, _T("%s"), _T("m_vTI.m_pBrep"));
      m_vTI.m_pBrep->DumpRegionsAndAttributes( sBuff ) ;
    }
#endif // SM_DEBUG_CODE

  // for every ResultBrep->Region - propagate Region attribute data
  for(ii=0;ii<sRegions.GetSize();ii++)
    {
      SmRegion * pRegion = sRegions[ii] ;

      // no work - no region (something is wrong)
      if(pRegion == NULL)
        { continue ; }

      // locals
      SmTArray<SmAttribute*> sThisAttribs, sOtherAttribs ;
      pRegion->GetAttributes(sThisAttribs) ;

      // get SmMergeRegionAttribute
      SmMergeRegionAttribute * pMergeRegionAttrib = (SmMergeRegionAttribute*)pRegion->FindAttribute(SM_AI_OTHER_REGION_ID) ;

      // Void branch on Region Void status - remove any Propagated through boolean attributes on the region
      if(pRegion->IsVoid() == TRUE)
        {
          // remove any Region Propagating attributes from the Region
          for(jj=0;jj<sThisAttribs.GetSize();jj++)
            {
              SmAttribute *pAttribute = sThisAttribs[jj] ;
              
              // no work - no attrib or attrib not a propagating attribute
              if(pAttribute == NULL || pAttribute->IsPropagatedThroughBooleans() == FALSE)
                { continue ; }

              // remove this attribute from its Void Region Owner
              pRegion->RemoveAttribute(pAttribute) ;
              sThisAttribs.SetAt(jj, NULL) ;
               
            } // end iter every VoidRegion->Attribute
        } // end Region IsVoid branch
      else
        { // Region is a solid branch

          // no work - this ResultRegion has not been merged with OtherBrep->Regions by this Merge operation
          if(pMergeRegionAttrib == NULL)
            { continue ; }

          // remember the boolean operation type
          pMergeRegionAttrib->SetOperation(eOperation) ;

          // use Notify to propagate the Region attributes and notify the world of the transaction
          //  The SM_NO_REG_PROPATATION behavior is currently implemented in SmAObject::Notify() because
          //   the behavior is an iteration and Notify should only be called once for the pRegion.
          pRegion->Notify(SM_NO_REG_PROPAGATION, &pMergeRegionAttrib->GetThisRegions(), &pMergeRegionAttrib->GetOtherRegions(), pRegion) ; // goes to SmTopology::Notify() then SmAObject::Notify()
        
        } // end Region IsSolid branch

      // remove temp MergeRegionAttrib attrib - we're done with it
      if(pMergeRegionAttrib) 
        { pRegion->RemoveAttribute(pMergeRegionAttrib) ; }

    } // end iter all ThisBrep->Regions

  // all done
  return(SM_SUCCESS) ;

} // end SmMerge::PropagateOtherSrcRegionAttribs
