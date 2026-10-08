// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************//**
* FILE NAME --- SmShell.cpp
* PURPOSE: Source file for SmShell class methods.
**********************************************************************/

#include "StdAfx.h"

#include <SmShell.h>

#include <SmTopologySolver.h>                   
#include <SmShape.h>
#include <SmGraphicsOutput.h>
#include <SmTopologyTraverser.h>
#include <SmLine.h>
#include <SmAssertArray.h>

//    // For debugging only - a set of pointers that can be assigned and inspected through the watch window
//    #ifdef SM_DEBUG_CODE                  
//    
//    SmShell * dbgShell1 = NULL ;
//    SmShell * dbgShell2 = NULL ;
//    
//    #endif // SM_DEBUG_CODE

/*******************************************************************//**
PURPOSE: 

NOTES: 
***********************************************************************/
SmShell::SmShell()
{
  m_tShellType = SmUnknown_TYPE ;

  // report construction at SmObject::Notify level - skip other levels
  SmObject::Notify(SM_NO_CONSTRUCTION, this, NULL, NULL);

} // end SmShell::SmShell Default Constructor

/*******************************************************************//**
PURPOSE: 

NOTES: 
***********************************************************************/
SmShell::~SmShell()
{
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
    if (bDebugMe) {
        Dump();
    }
#endif
  Notify(SM_NO_DESTRUCTION, this, NULL, NULL);

  m_tShellType = SmUnknown_TYPE ;

} // end SmShell::~SmShell destructor

/*******************************************************************//**
PURPOSE: Calculate the bounding box for this shell.

NOTES: 
***********************************************************************/
SmStatus SmShell::CalculateBoundingBox
 ( SmExtent3d & rBBox,       // out: Bounding Box containing region
   SmBoolean    bTight)      // in : TRUE = compute minimal box for each contained face and edge (expensive)      
                             //      FALSE= compute any box larger for each contained face and edge (cheaper)
                             //      default:[FALSE]
 const
{
  rBBox.Init();

  SmExtent3d sThisBox;
  ULONG i;

  switch(m_tShellType)
    {
      case SmVertexuse_TYPE:
        { 
          rBBox.AddPoint3d( GetVertex()->GetPoint() );
          break;
        }
      case SmEdgeuse_TYPE:
        { 
          SmTArray< SmEdge* > sEdges;
          GetWireEdges( sEdges );
          ULONG lNumEdges = sEdges.GetSize();
          for ( i = 0; i < lNumEdges; i++ )
          {
              SER( sEdges[i]->CalculateBoundingBox( &sThisBox, NULL, NULL, bTight ) );
              rBBox.Union( sThisBox, rBBox );
          }

          break ;
        }
      case SmFaceuse_TYPE:
        {
          SmTArray<SmFaceuse*> sFaceuses;
          GetFaceuses( sFaceuses );

          ULONG lNumFaceuses = sFaceuses.GetSize();
          for ( i = 0; i < lNumFaceuses; i++ )
          {
              SER( sFaceuses[i]->GetFace()->CalculateBoundingBox( sThisBox, bTight ) );
              rBBox.Union( sThisBox, rBBox );
          }

          break ;
        }
    }  // end switch

    return SM_SUCCESS;

} // end SmShell::CalculateBoundingBox

/*******************************************************************//**
PURPOSE: Get all of the wire edges associated with a shell.

NOTES: Works for all m_tShellType types including faceuse shells, wire shells, and vertex shells
       because a faceuse shell can have wireEdges.
       
       This current implementation is somewhat inefficient.
       A more efficient scheme would be to do direct traversal of the shell.

METHOD --- 1. Get all Shell->Brep->Edges
           2. add all wire edges whose EUs point back to this shell to output
***********************************************************************/
SmStatus SmShell::GetWireEdges
  (SmTArray<SmEdge*> & rWireEdges) // out: appended with all wires owned by this shell
 const
{
  // init output
  rWireEdges.ReSet() ;

  // case: VertexShell - no wire edges
  if(IsVertexShell())
    { return SM_SUCCESS ; }

  // case: WireShell - gather connected edges
  if(IsWireShell())
    {
      // grab a mark - released when deleted
      SmNewMarkAndLock sMarkLock(GetContext(), SM_MT_ALLMARKS) ;
      SmMarkType eMarkType = sMarkLock.GetMarkType() ;

      // locals
      ULONG ii, jj ;
      SM_PTR_ARRAY(sEdges,     SmEdge, 32) ; // SmTArray<SmEdge*> used as stack in upcoming loop
      SM_PTR_ARRAY(sVertEdges, SmEdge, 16) ; // SmTArray<SmEdge*> used as stack in upcoming loop
      SM_PTR_ARRAY(sVertices, SmVertex, 4) ; // expect only two at a time

      // Get the first WireEdge and its vertices
      SmEdge * pVertEdge, *pEdge = GetWireEdge() ;
      sEdges.Push(pEdge) ;

      // While edges are on the Stack
      while(sEdges.Pop(pEdge))
        {
          pEdge->GetVertices(sVertices) ;

          // for every pEdge->Vertex
          for(ii=0;ii<sVertices.GetSize();ii++)
            {
              SmVertex *pVertex = sVertices[ii] ;

              // skip already marked vertices - mark the others
              if(pVertex->IsMarked(eMarkType))
                { continue ; }
              pVertex->Mark(eMarkType) ;

              // for every Vertex->Edge
              pVertex->GetEdges(sVertEdges) ;
              for(jj=0;jj<sVertEdges.GetSize();jj++)
                {
                  pVertEdge = sVertEdges[jj] ;

                  // skip marked edges - mark the rest
                  if(pVertEdge->IsMarked(eMarkType))
                    { continue ; }
                  pVertEdge->Mark(eMarkType) ;

                  // check state - all connected edges should be wires
                  SM_ASSERT_MSG(pVertEdge->IsWire(), _T("SmShell::GetWireEdges - found a connected Edge that is not a wire in a WireShell.  There should be only wires.")) ;

                  // add edge to output
                  rWireEdges.Add(pVertEdge) ;

                  // put edge on the stack
                  sEdges.Push(pVertEdge) ;

                } // end iter every Vertex->Edge
            } // end iter every pEdge->Vertex
        } // end do While edges are on the stack

      // all done
      return(SM_SUCCESS) ;

    } // end IsWireShell branch

  // arrive here when Shell type is SmFaceuse_TYPE or SmUnknown_TYPE 

  // get shell->brep edges
  SmBrep *pBrep = GetBrep();
  SmTArray<SmEdge*> sAllEdges;
  pBrep->GetEdges(sAllEdges);

  // for every Brep edge
  for (ULONG i=0; i<sAllEdges.GetSize(); i++) 
    {
      SmEdge *pEdge = (SmEdge*)sAllEdges[i];

      // add wire edges that belong to this shell to the child list
      if (pEdge->IsWire()) 
        {
          SmEdgeuse *pEdgeuse = pEdge->GetPrimaryEdgeuse();

          if (pEdgeuse->m_pSorLU == (SmTopology*)this) 
            {
              rWireEdges.Add(pEdge);

            } // end owned by shell check
        } // end wire check
    } // end iter all brep edges

  return SM_SUCCESS;

} // end SmShell::GetWireEdges

/*******************************************************************//**
PURPOSE: Get all shellVertices associated with a shell.

NOTES: This function is for checking and debugging use only.
           There should only be one vertex associated with a shell
           and it can always be found by following the pointers as implemented in
           SmShell::GetVertex() ;

METHOD --- 1. Get all Shell->Brep->Vertices
           2. add all vertices whose VUs point back to this shell to output
***********************************************************************/
static SmStatus sm_GetShellVertices
  (const SmShell *pShell,                // in : target shell
   SmTArray<SmVertex*> & rShellVertices) // out: appended with all ShellVertices owned by this shell
                                         // note: When SmShell->m_tShellType == SmVertexuse_TYPE
                                         //          rShellVertices.GetSize() should equal 1,
                                         //       otherwise
                                         //          rShellVertices.GetSize() should equal 0.
{
  // init output
  rShellVertices.ReSet() ;

  // get shell->brep edges
  SmBrep *pBrep = pShell->GetBrep();
  SmTArray<SmVertex*> sAllVertices;
  pBrep->GetVertices(sAllVertices);

  // for every Brep vertex
  for (ULONG i=0; i<sAllVertices.GetSize(); i++) 
    {
      SmVertex *pVertex = sAllVertices[i];

      // add ShellVertices that belong to this shell to the child list
      if (pVertex->IsShellVertex()) 
        {
          SmVertexuse *pVertexuse = pVertex->GetPrimaryVertexuse();

          if (pVertexuse->GetShell() == (SmTopology*)pShell) 
            {
              rShellVertices.Add(pVertex);

            } // end owned by shell check
        } // end ShellVertex check
    } // end iter all brep vertices

  // check for consistency
  SM_ASSERT(   (!pShell->IsVertexShell() && rShellVertices.GetSize() == 0)
            || ( pShell->IsVertexShell() && rShellVertices.GetSize() == 1 
                                         && pShell->GetVertex()      == rShellVertices[0])) ;
  // all done
  return SM_SUCCESS;

} // end sm_GetShellVertices

/*******************************************************************//**
PURPOSE: Get all ShellFaceuses associated with a shell.

NOTES: This function is for checking and debugging use only.
           The faceuses are listed directly in the topology graph and should
           be retrieved with a call to 
           SmShell::GetFaceuses() ;

METHOD --- 1. Get all Shell->Brep->Faceuses
           2. add all faceuses whose m_pListOwner points back to this shell to output
***********************************************************************/
static SmStatus sm_GetShellFaceuses
  (const SmShell *pShell,                   // in : target shell
   SmTArray<SmFaceuse*> & rShellFaceuses)   // out: appended with all ShellFaces owned by this shell
                                            // note: rShellFaces.GetSize() should equal pShell->GetFaceuses().GetSize()
{
  // init output
  rShellFaceuses.ReSet() ;

  // get shell->brep edges
  SmBrep *pBrep = pShell->GetBrep();
  SmTArray<SmFace*> sAllFaces;
  pBrep->GetFaces(sAllFaces);

  // for every Brep face
  for (ULONG i=0; i<sAllFaces.GetSize(); i++) 
    {
      SmFace *pFace = sAllFaces[i];
      SmFaceuse *pFaceuse1, *pFaceuse2 ;
      pFace->GetFaceuses(pFaceuse1, pFaceuse2) ;

      if(pFaceuse1 && pFaceuse1->GetShell() == pShell) rShellFaceuses.Add(pFaceuse1) ; 
      if(pFaceuse2 && pFaceuse2->GetShell() == pShell) rShellFaceuses.Add(pFaceuse2) ; 

    } // end iter all brep faces

  // check for consistency
  SmTArray<SmFaceuse*> sAllFaceuses ;
  pShell->GetFaceuses(sAllFaceuses) ;

  SM_ASSERT(rShellFaceuses.GetSize() == sAllFaceuses.GetSize()) ;

  // all done
  return SM_SUCCESS;


} // end sm_GetShellFaceuses

/*******************************************************************//**
PURPOSE: Transfer all children of this to the recipient.  

NOTES: 
    Make all things which point to this shell point to the recipient shell.
    This is a specialized routine designed to be used by other methods which
    combine shells (e.g. MakeEdgeTopology).  It will convert the shell
    type to the highest dimension entity.
    
    Shell Rules to follow
    1. Shell->m_pList points to one connected element of the highest dimension type
        to which the shell is connected.
    2. Shell->m_tShellType value matches the type of the object pointed to by Shell->m_pList
       m_tShellType == SmVertexuse_TYPE, m_pList is a pointer to a SmVertexuse (shell vertex)
       m_tShellType == SmEdgeuse_TYPE, m_pList is a pointer to a SmEdgeuse     (wire edge)
       m_tShellType == SmFaceuse_TYPE, m_pList is a pointer to a SmFaceuse     
    3. every object connected directly to a shell has a back pointer to that shell
       SmVertexuse->m_pSorLUorEU
       SmEdgeuse->m_pSorLU
       SmFaceuse->m_pListOwner
    4. mixed dimension shells - only SmFacuse
       a. m_tShellType == SmVertexuse_TYPE;  1 Vertexuse use points back to Shell.
                                                No Edgeuses, No Faceuses
       b. m_tShellType == SmEdgeuse_TYPE;    1 or more sets of two Edgeuses 
                                                   (one set for each wire edge) point back to Shell; 
                                                No Vertexuses, No Faceuses
       c. m_tShellType == SmFaceuse_TYPE;    Any number of Faceuses point back to Shell
                                                0, 1, or more sets of two Edgeuses 
                                                   (one set for each wire edge) point back to Shell;
                                                No Vertexuses
***********************************************************************/
SmStatus SmShell::TransferChildrenTo
  (SmShell * pRecipient,                   // in : Shell to receive children
   SmTArray<SmEdge*> * pOptBrepWireEdges)  // in : optional list of wire edges
                                           //      to check for shell ownership.
                                           //      If null, all shell->Brep edges are checked.
{
  // pRecipient is often a reused empty shell - make sure its classification is correct
  if(pRecipient->GetSize() == 0)
    {
      SM_ASSERT(pRecipient->m_pList == NULL) ;
      
      // assume there are no leftover pieces pointing back to this shell
      // note: we might have to do an expensive check here and gather all
      //       geometry that points back to the shell and then set 
      //       shell m_pList, m_lListSize, m_tShellType appropriately.
      //       I'm hoping the calling functions are doing a good job of 
      //       taking care of the topology graph and this function can skip
      //       these very expensive checks.
      pRecipient->m_tShellType = SmUnknown_TYPE ;

     // low work - moving vertexShells into empty shells
     if(this->IsVertexShell()) 
       {
         // move lone vertexuse pointer from this to recipient and set backpointer
         pRecipient->m_tShellType = SmVertexuse_TYPE ;
         pRecipient->m_lListSize  = 1 ;
         pRecipient->m_pList      = this->m_pList ;
         SM_ASSERT(pRecipient->m_pList->IsKindOf(SmVertexuse_TYPE)) ;
         ((SmVertexuse*)pRecipient->m_pList)->m_pSorLUorEU = pRecipient ;

         // clear this shell ownership pointers
         this->m_tShellType       = SmUnknown_TYPE ;
         this->m_lListSize        = 0 ;
         this->m_pList            = NULL ;

         // all done
         return SM_SUCCESS;
       }

    } // end recipient shell is empty branch
  else // recipient shell is not empty
    {
      SM_ASSERT(pRecipient->m_pList != NULL) ;

      // it is illegal to move geometry into a VertexShell
      if(pRecipient->IsVertexShell())
        {
          if(this->GetSize() == 0)
            {
              // ok - no geometry to transfer
              return(SM_SUCCESS) ;
            }
          else // problem - been asked to do an illegal move
            {
              return(SM_ERR) ;
            }
        } // end recipient is a vertex shell check

      // arrive here when recipient is not a vertex shell

      // it is illegal to move a ShellVertex into a nonEmpty Shell
      if(this->IsVertexShell())
        {
          // problem - been asked to do an illegal move
          return(SM_ERR) ;
        }
          
    } // end recipient shell is not empty branch

  // arrive here when this shell is an edgeuseShell or a faceuseShell
  ULONG ii ;

  // get ThisShell WireEdges
  SmTArray<SmEdge*>    sTargetWireEdges;
  SmTArray<SmEdgeuse*> sEdgeuses;
  if (pOptBrepWireEdges) 
    {
      // for every optional wireEdge
      for(ii=0; ii<pOptBrepWireEdges->GetSize(); ii++) 
        {
          SmEdge    *pE  = (*pOptBrepWireEdges)[ii];
          SmEdgeuse *pEU = pE->GetPrimaryEdgeuse();

          // if its owned by this shell
          if (pEU->m_pSorLU == this) 
            {
              // add it to the children list
              sTargetWireEdges.Add(pE);
            }
        }
    } // end given pOptBrepWireEdges branch
  else 
    {
      // get all shell->Brep wire edges owned by this shell 
      GetWireEdges(sTargetWireEdges);
    }

  // when ThisShell has WireEdges to move
  if (sTargetWireEdges.GetSize() > 0) 
    {
      // Set recipient type if it is currently unknown
      if (pRecipient->IsUnknownShell()) 
        {
          SmEdgeuse *pEU           = sTargetWireEdges[0]->GetPrimaryEdgeuse() ;
          pRecipient->m_pList      = (SmTopology*)pEU ; 
          pRecipient->m_tShellType = SmEdgeuse_TYPE;
          pRecipient->UpdateListSize() ;
        }

      // Set every TargetWireEdge->Edgeuse owner to pRecipient Shell
      for(ii=0; ii<sTargetWireEdges.GetSize(); ii++) 
        {
          SmEdge *pWireEdge = (SmEdge*)sTargetWireEdges[ii];
          pWireEdge->GetEdgeuses(sEdgeuses);
          for (ULONG j=0; j<sEdgeuses.GetSize(); j++) 
            {
              SmEdgeuse *pEU = (SmEdgeuse*)sEdgeuses[j];
              pEU->m_pSorLU  = pRecipient;
            }
        } // end iter every TargetWireEdge
    } // end need to move wire edges check

  // get ThisShell faceuses
  SmTArray<SmFaceuse*> sTargetFaceuses;
  GetFaceuses(sTargetFaceuses);

  // When ThisShell has Faceuses to move
  if (sTargetFaceuses.GetSize() > 0) 
    {
      if (!pRecipient->IsFaceuseShell()) 
        {
          pRecipient->m_tShellType = SmFaceuse_TYPE;
          pRecipient->m_pList      = NULL ;
          pRecipient->m_lListSize  = 0 ;
        }

      // move every thisShell->faceuse to pRecipientShell->Faceuse list
      for(ii=0; ii<sTargetFaceuses.GetSize(); ii++) 
        {
          SmFaceuse *pFU = (SmFaceuse*)sTargetFaceuses[ii];
          SE(Remove(pFU));
          SE(pRecipient->PostInsert(pFU));  //. sets the pFU->m_pListOwner back pointer as well
    
        } // end iter every ThisShell->faceuse

    } // end need to init recipient to FaceuseShell type

  SM_ASSERT(!pRecipient->IsUnknownShell()) ;

  // arrive here when all ThisShell children have been moved

  // clear ThisShell values
  m_tShellType = SmUnknown_TYPE ;
  m_pList      = NULL;
  m_lListSize  = 0 ;

  // all done
  return SM_SUCCESS;

} // end SmShell::TransferChildrenTo

/*******************************************************************//**
PURPOSE: Get list of all shells which share a face boundary
            with this shell.

NOTES: 
  If this shell contains a lamina Face, it will return 'this' shell in the list
  (because 'this' is on the other side of a lamina Face).

SIDE EFFECTS ---
  Uses (increments) an used Mark value.
***********************************************************************/
SmStatus SmShell::GetMatedShells
  (SmTArray<SmShell *> &rMatedShells) // out: list of shells sharing a face
 const                                //      boundary with this one
{
  // init output
  rMatedShells.ReSet() ;

  // no work - not a SmFaceuse_TYPE shell
  if(m_tShellType != SmFaceuse_TYPE)
    { return(SM_SUCCESS) ; }

  // locals
  ULONG ii ;

  // use a mark system - first get a new mark value
  SmNewMarkAndLock sMarkLock((SmContext *)GetContext(), SM_MT_ALLMARKS) ; // increment and lock any unlocked mark
  SmMarkType eMarkType = sMarkLock.GetMarkType() ;

  // get all this shell faceuses
  SmTArray<SmFaceuse *> sFaceuses ;
  GetFaceuses(sFaceuses) ;

  // for every faceuse
  for(ii=0;ii<sFaceuses.GetSize();ii++)
    {
      SmFaceuse *pFaceuse = sFaceuses[ii] ;

      // get faceuse mate shell
      SmShell *pMateShell = pFaceuse->GetMate()->GetShell();

      // skip marked shells
      if(pMateShell->IsMarked(eMarkType))
        { continue ; }

      // mark the mated shell and add it to the output
      pMateShell->Mark(eMarkType) ;
      rMatedShells.Add(pMateShell) ;

    } // end iter every faceuse looking for mated shells

  // all done
  return(SM_SUCCESS) ;

} // end SmShell::GetMatedShells

/*******************************************************************//**
PURPOSE: Fire a ray from the given point in the given direction and
         see what you hit on this shell if anything.

NOTES: 1. This fires rays only against faces which border the shell,
          i.e., which have a different shell on the other side; it skips faces
          which are contained within this shell.
       2. Increments unused Mark value

    returns SM_ERR when test point is within tolerance
***********************************************************************/
SmStatus SmShell::RayFire
  (const SmPoint3d       & crRayStartPoint,    // in : Point location to classify
   const SmVector3d      & crRayVector,        // in : ray direction
   SmZoneTol3d             sRayZoneTol3d,      // in : RayZoneTol3d - sizes XSectTol3d dist for finding objects
   SmRegion             *& rpRegionOfPoint,    // out: Region containing point or 
                                               //      NULL when ray does not intersect this Shell
   SmPointClassification & rRayIntersection,   // out: Point classification of ray intersection
                                               //      classification = SM_PC_UNKNOWN when ray
                                               //      does not intersect this shell.
   double                * pOptSphereBound)    // in : Optional spherical bound to avoid BBox construction (this method can be called many times in succession for the same brep).
  const
{
  // locals
  SmBrep     *pBrep = GetBrep();

  double      dSphRadius;

  if (pOptSphereBound)
  {
      SM_ASSERT(*pOptSphereBound > 0);
      dSphRadius = *pOptSphereBound;
  }
  else
  {
      SmPoint3d sSphCenter;
      SmExtent3d sBBox;
      SER(pBrep->CalculateBoundingBox(sBBox));
      sBBox.ComputeSphereBound(sSphCenter, dSphRadius);
  }
  
  // size ray to Brep Size
  SmPoint3d  sRayEnd = crRayStartPoint + (3.01 * dSphRadius) * crRayVector;
  SmVector3d sVecs[2];
  sVecs[0] = sRayEnd - crRayStartPoint;
  SER(sVecs[0].Unitize());

  // Get Shell->Faceuses
  SmSolution           sSData[64];
  SmSolutionArray      sSolutions(64,sSData);
  SmTArray<SmFace*>    sFaces;
  SmTArray<SmFaceuse*> sFaceuses;
  GetFaceuses(sFaceuses);

  // pick an unsed mark and increment it
    {
      SmNewMarkAndLock sMarkLock((SmContext *)GetContext()) ; // increment and lock any unlocked mark
      SmMarkType       eMarkType = sMarkLock.GetMarkType() ;

      // get list of all non-sheet faces bounding this shell
      ULONG i, lNumFUs = sFaceuses.GetSize();
      for (i=0; i<lNumFUs; i++) 
        {
          SmFace *pF = sFaceuses[i]->GetFace();
          if (pF->IsMarked(eMarkType)) continue;
          pF->Mark(eMarkType);
          SmFaceuse *pFU        = sFaceuses[i];
          SmFaceuse *pFUMate    = pFU->GetMate();
          SmShell   *pMateShell = pFUMate->GetShell() ;

          // when face is on the shell boundary
          if ( pMateShell != this ) 
            {
              sFaces.Add(pF);
            }
        } // end iter every shell->faceuse
    } // end scope for sMarkLock

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  if(bDebugMe)
    {
      smgfx_Erase();
      smgfx_SetLook(1,2, 0,0,1); if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ; 
      smgfx_SetLook(1,2, 0,1,1); for(ULONG ii=0;ii<sFaces.GetSize();ii++) { sFaces[ii]->Draw(SM_DM_CROSSHATCH) ; sm_GraphicsLoop() ; }
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

  // locals - make a shape containing just the boundary faces of this shell
  SmTArray<SmBrep*>   sBreps;
  SmTArray<SmEdge*>   sEdges;
  SmTArray<SmVertex*> sVertices;
  SmShape sShape(sBreps,sFaces,sEdges,sVertices);  // increment unlocked Mark value
  sShape.SetContext(this->GetContext()) ;

#ifdef SM_DEBUG_CODE
  if(bDebugMe)
    {
      sShape.Dump() ;
    }
#endif // SM_DEBUG_CODE

  // classify a ray against all of this shell's boundary faces
  SER(SmTopologySolver::ShapePointSolve(&sShape, 
                                        crRayStartPoint, 
                                        SM_SO_RAYFIRE,
                                        SM_SR_ALL, 
                                        sRayZoneTol3d, 
                                        SM_BIG_DOUBLE, 
                                        sVecs,
                                        sSolutions));

  // NOTE: for Rayfire, ShapePointSolve returns, in sSolutions:
  //   m_dSolutionValue <- parameter along ray
  //   m_adParameters[0] <- U value on surface
  //   m_adParameters[1] <- V value on surface

#ifdef SM_DEBUG_CODE
  if (bDebugMe) 
    {
      sSolutions.Dump();
      SmVector3d sRay = sRayEnd - crRayStartPoint ; 
      smgfx_Erase();
      smgfx_SetLook(5,6, 1,0,0); sSolutions.Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook(3,4, 0,1,0); crRayStartPoint.Draw(); sm_GraphicsLoop() ;
                                 sRay.Draw(&crRayStartPoint); sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,1); Draw(); sm_GraphicsLoop() ;
      sm_GraphicsLoop();
    }
#endif
  if (sSolutions.GetSize() == 0) 
    {
      rpRegionOfPoint = NULL;
      rRayIntersection.SetClassObject(SM_PC_UNKNOWN, NULL);
    }
  else 
    {
      SmSolution sSol = sSolutions[0];
      double dRayParam0 = sSol.m_vStart.m_dSolutionValue;

      for (ULONG i=1; i<sSolutions.GetSize(); i++) 
        {
          SmSolution & rSol = sSolutions[i];
          double dRayParam1 = rSol.m_vStart.m_dSolutionValue;

          if ( dRayParam1 - dRayParam0 < SM_EFF_ZERO_SQRT) 
            {
              // Hit the same thing.  Take the lower dimensional element
              // -- it is the one with the lowest number of variables
              // If there is a tie just use the start value below.
              if (rSol.m_lNumVariables < sSol.m_lNumVariables) 
                { sSol = rSol;
                  continue;
                }
              if (rSol.m_lNumVariables > sSol.m_lNumVariables) 
                { continue; }
                
            }
          if (rSol.m_vStart[0] < sSol.m_vStart[0]) 
            {
              sSol = rSol;
            }
        }
      
      // Check ray intercept zero or negaitve.
      dRayParam0 = sSol.m_vStart.m_dSolutionValue;
      if ( dRayParam0 < -SM_EFF_ZERO )
        {
          rpRegionOfPoint = NULL;
          rRayIntersection.SetClassObject(SM_PC_UNKNOWN, NULL);
        }

      double dTol = SM_EFF_ZERO * (1.0 + crRayStartPoint.GetMaxDimension());
      if ( dRayParam0 < dTol )
        {
          return SM_ERR;
        }

      SER(rRayIntersection.LoadFromIntersection(sRayZoneTol3d, sSol.m_apObjects[0],sSol.m_vStart,0, sSol.m_vStart.m_dSolutionValue));
      SmPoint3d sEndPnt = crRayStartPoint + sSol.m_vStart.m_dSolutionValue * sVecs[0];
      SmBSplineCurve *pRay = NULL ;
      SER(SmBSplineCurve::CreateLineSegment(*pBrep->GetContext(),3,crRayStartPoint,sEndPnt,pRay));
      NER(pRay);
      SmObjDelete sClean(pRay);
      SmExtent1d sIvl = pRay->GetNaturalInterval();
      SER(pBrep->LocalCurveSector(*pRay,sIvl,SM_OT_OPPOSITE,
                                  rRayIntersection,
                                  rpRegionOfPoint));
    }

  return SM_SUCCESS;

} // end SmShell::RayFire

/*******************************************************************//**
PURPOSE: For a closed shell, determine whether this shell is inner or outer.

NOTES: 
   The Faces of our Faceuses must form a closed region.
   That is not explicitly checked for,
   but if there are no non-lamina faces, or if no ray-fires hit
   any of the faces, then rbIsInner is returned UNSURE,
   and this routine returns SM_ERR.

METHOD ---
   We use SmTopologyTraverser::IsInnerShell().
   Fire rays from a faceuse and from its mate -- opposite directions.
   The ray from the inner faceuse will hit another face in the collection,
   while the one from the outer will not.
   There are always exceptions of course, so be careful.
***********************************************************************/
SmStatus SmShell::IsInnerShell( SmBoolean & rbIsInner ) const
{
  // Init output
  rbIsInner = UNSURE;

  SmTArray< SmFaceuse* > sFUs;
  this->GetFaceuses( sFUs );

  return SmTopologyTraverser::IsInnerShell( sFUs, rbIsInner );

} // end IsInnerShell

/*******************************************************************//**
PURPOSE: See if this shell is an inner shell within
  the infinite region or a region bounded by an optional OuterShell argument.

NOTES:

RETURNS ---
  TRUE when (pOptOuterShell is NULL or is Closed) and a ray cast
            from this shell determines that this shell is contained
            directly within the pOptOuterShell.  
  FALSE when if pOptShell is not NULL and not a closed shell
             or if a ray cast shows that this shell is not contained
             directly within pOptOuterShell when given or
             is contained by some other shell when it's being tested
             as an inner shell for the infinite region (pOptOuterShell == NULL).
  
  note: If this Shell is contained within a region which is contained 
        in pOptOuterShell then FALSE is returned, i.e. FALSE for nested 
        containment.  TRUE is only returned for simple Parent-Child containment.

  increments unused Mark value
***********************************************************************/
SmBoolean SmShell::IsInnerShellOf
  (const SmShell *pOptOuterShell)     // in : pOptOuterShell == NULL for infinite region
 const
{
  // locals
  SmBrep *pBrep = GetBrep() ;
  SM_ASSERT_MSG(pBrep != NULL, _T("IsInnerShellOf() input argument shell has no Brep")) ;

  // check state - if given pOptOuterShell needs to be a Faceuse_TYPE shell that is closed
  if(pOptOuterShell)
    {
      SmTopologyTraverser sTopologyTraverser ;
      SmBoolean bIsClosed = FALSE ;
      
      if(pOptOuterShell->IsFaceuseShell())
        { 
          SmNewMarkAndLock sMarkLock(GetContext(), SM_MT_ALLMARKS) ; // increment and lock any unlocked mark
          sTopologyTraverser.ClosureTraversal(pOptOuterShell->GetFirstFaceuse(), // in : target faceuse (gets marked)
                                              bIsClosed,                         // out: TRUE=faceuse connects to closed faceuse set, FALSE=doesn't                                                                         
                                              NULL,                              // out: List of all connected faces (get marked), NULL to ignore.
                                              sMarkLock) ;                       // in : specify mark for target objects (incremented)
        }

      if(bIsClosed == FALSE)
        { return FALSE ; }
    } // end shell is closed faceuse_TYPE check

  // get size based on Approx Brep bounding box
  SmExtent3d sBBox ;
  pBrep->CalculateBoundingBox(sBBox, TRUE) ;
  sBBox.ExpandAbsolute(1.0) ;
  double dSize = 2.0 * sBBox.GetSize().Length() ;
  double dTol  = SM_EFF_ZERO * (1.0 + dSize) ;

  // build a ray curve
  SmTArray<SmPoint2d> sSmpUV ;
  SmTArray<SmPoint3d> sSmpPV ;
  SmPoint3d sStartPnt, sStopPnt ;

  // search the faceuses of the shells for likely ray casts (those that hit only faces)
  ULONG ii, jj, kk ;
  ULONG lSmpCnt = 3 ;
  SmTArray<SmFaceuse *> sInnerFaceuses ;
  SmTArray<SmEdge *> sInnerWireEdges ;
  SmVertex *pInnerShellVertex = GetVertex() ; 
  GetFaceuses(sInnerFaceuses) ;
  GetWireEdges(sInnerWireEdges) ; 

  SmBoolean bFoundSolution = FALSE ;

  ULONG iCnt = IsFaceuseShell() ? sInnerFaceuses.GetSize() : 1 ;
  for(ii=0;ii<iCnt && bFoundSolution == FALSE;ii++)
    {
      SmFaceuse *pInnerFaceuse  = IsFaceuseShell() ? sInnerFaceuses[ii] : NULL ;
      SmEdge    *pInnerWireEdge = IsWireShell() ? sInnerWireEdges[0] : NULL ;

      // sPV[0,2,4,...] = point to start ray for each SmpPoint,
      // sPV[1,3,5,...] = direction to run ray for each SmpPoint
      if(IsFaceuseShell())     { pInnerFaceuse->GetPointsInFace(lSmpCnt, sSmpUV, sSmpPV) ; }
      else if(IsWireShell())   { SmExtent1d sIvl = pInnerWireEdge->GetInterval() ;
                                 SmPoint3d sPoint, sDir ;
                                 srand((unsigned)time(NULL)) ;
                                 for(jj=0;jj<lSmpCnt;jj++)
                                   {
                                     SmCurve *pCurve = pInnerWireEdge->GetCurve() ;
                                     // sample the curve and pick varying directions for the vector
                                     pCurve->EvaluatePoint(sIvl.Evaluate((double)(jj + 1) / (double)(lSmpCnt + 1)),
                                                           sPoint) ;
                                     sDir.Set( 2.0*(((double)rand()) / ((double)RAND_MAX)) - 1.0 ,
                                               2.0*(((double)rand()) / ((double)RAND_MAX)) - 1.0 ,
                                               2.0*(((double)rand()) / ((double)RAND_MAX)) - 1.0) ;
                                     sDir.Unitize() ;
                                     sSmpPV.Add(sPoint) ;
                                     sSmpPV.Add(sDir) ;
                                   }
                               }
      else if(IsVertexShell()) { SmVector3d sPoint = pInnerShellVertex->GetPoint() ;
                                 SmVector3d sDir ;
                                 srand((unsigned)time(NULL)) ;
                                 for(jj=0;jj<lSmpCnt;jj++)
                                   {
                                     // pick varying directions for the vector
                                     sDir.Set( 2.0*(((double)rand()) / ((double)RAND_MAX)) - 1.0 ,
                                               2.0*(((double)rand()) / ((double)RAND_MAX)) - 1.0 ,
                                               2.0*(((double)rand()) / ((double)RAND_MAX)) - 1.0) ;
                                     sDir.Unitize() ;
                                     sSmpPV.Add(sPoint) ;
                                     sSmpPV.Add(sDir) ;
                                   }
                               }
      else { break ; }

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
      // draw 
      if(bDebugMe)
        {
          smgfx_Erase() ;
          smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
          smgfx_SetLook(1,2, 0,1,1) ; if(pInnerFaceuse) pInnerFaceuse->Draw(SM_DM_CROSSHATCH) ; sm_GraphicsLoop() ;
          smgfx_SetLook(1,2, 0,1,1) ; if(pInnerWireEdge) pInnerWireEdge->Draw() ; sm_GraphicsLoop() ;
          smgfx_SetLook(3,4, 0,1,1) ; if(pInnerShellVertex) pInnerShellVertex->Draw() ; sm_GraphicsLoop() ;
          smgfx_SetLook(4,5, 0,1,0) ; for(ii=0;ii<sSmpPV.GetSize();ii+=2)
                                       { sSmpPV[ii+1].Draw(&sSmpPV[ii]) ; sm_GraphicsLoop() ; }
          sm_GraphicsLoop() ;
        }
#endif // SM_DEBUG_CODE

      // for every sample point (a position and a direction vector)
      for(jj=0;jj<sSmpPV.GetSize();jj+=2)
        {
          bFoundSolution      = TRUE ;
          SmPoint3d  sLinePnt = sSmpPV[jj] ;
          SmVector3d sLineVec = sSmpPV[jj+1] ;

          // build a ray line
          SmLine sLine( sLinePnt, sLineVec, SmExtent1d(0.0, dSize), 1.0, 3, pBrep->GetContext()) ;

          // intersect line with Brep
          SmSolutionArray sSolutions ;
          SmTopologySolver::BrepCurveSolve(pBrep,
                                           sLine,
                                           sLine.GetNaturalInterval(),
                                           SM_SO_INTERSECT,
                                           SM_SR_ALL,
                                           dTol,
                                           0.0,
                                           NULL,
                                           sSolutions) ;
#ifdef SM_DEBUG_CODE
          // draw 
          if(bDebugMe)
            {
              sSolutions.Dump() ;

              smgfx_Erase() ;
              smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
              smgfx_SetLook(1,2, 0,1,1) ; if(pInnerFaceuse) pInnerFaceuse->Draw(SM_DM_CROSSHATCH) ; sm_GraphicsLoop() ;
              smgfx_SetLook(3,4, 0,1,0) ; sLine.Draw() ; sm_GraphicsLoop() ;
              smgfx_SetLook(5,6, 1,0,0) ; sSolutions.Draw() ; sm_GraphicsLoop() ;
              sm_GraphicsLoop() ;
            }
#endif // SM_DEBUG_CODE

          // keep things simple - only use classifications which have point intersections with faces
          //  ignore intersection with vertex and wireedge shells - they won't change the classification

          for(kk=0;kk<sSolutions.GetSize();kk++)
            {
              SmSolution &rSolution = sSolutions[kk] ;

              // keep the first solution - but ignore (by removing from the solutions array) 
              // any subseqent wireedge or vertexshell intersections
              if(   rSolution.m_eSolutionType == SM_ST_SINGLE_VALUE
                 && (
                        (   rSolution.m_apObjects[1]->IsKindOf(SmEdge_TYPE) 
                            && ((SmEdge *)rSolution.m_apObjects[1])->IsWire()
                        )
                     || (   rSolution.m_apObjects[1]->IsKindOf(SmVertex_TYPE)
                          && (   ((SmVertex *)rSolution.m_apObjects[1])->IsShellVertex()
                              || ((SmVertex *)rSolution.m_apObjects[1])->IsWireVertex()
                             )
                        )
                    )
                )
                {
                  // remove this solution from the sSolutions array
                  sSolutions.RemoveAt(kk) ;
                  continue ;
 
                } // end is wireedge or shellvertex solutions to be ignored check

              // skip range solutions (coincident with some geomtery) and
              // intersections with anything but faces.  Remember the first
              // intersection will be a vertex for a vertexShell, an edge for a wireShell
              // or a face for an faceuseShell.
              if(   rSolution.m_eSolutionType == SM_ST_RANGE_OF_VALUES
                 || (kk == 0 && IsFaceuseShell() && FALSE == rSolution.m_apObjects[1]->IsKindOf(SmFace_TYPE))
                 || (kk == 0 && IsWireShell()    && FALSE == rSolution.m_apObjects[1]->IsKindOf(SmEdge_TYPE))
                 || (kk == 0 && IsVertexShell()  && FALSE == rSolution.m_apObjects[1]->IsKindOf(SmVertex_TYPE))
                 || (kk > 0 && FALSE == rSolution.m_apObjects[1]->IsKindOf(SmFace_TYPE)))
                {
                   // skip this solution
                   bFoundSolution = FALSE ;
                   break ;
                }
            } // end checking every solution for a simple case 

          // try again
          if(bFoundSolution == FALSE)
            { continue ; }

          // load solution into a CurveClassification
          SmCurveClassification sCurveClass(&sLine, sLine.GetNaturalInterval(), NULL, dTol) ;
          sCurveClass.InsertTopologyIntersections(sSolutions, 0) ;

#ifdef SM_DEBUG_CODE
          // draw 
          if(bDebugMe)
            {
              //SmBoolean bManifold = pBrep ? pBrep->IsManifoldSolid() : FALSE ;
              sCurveClass.Dump() ;

              smgfx_Erase() ;
              smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
              smgfx_SetLook(1,2, 0,1,1) ; if(pInnerFaceuse) pInnerFaceuse->Draw(SM_DM_CROSSHATCH) ; sm_GraphicsLoop() ;
              smgfx_SetLook(3,4, 0,1,0) ; sLine.Draw() ; sm_GraphicsLoop() ;
              smgfx_SetLook(5,6, 1,0,0) ; sSolutions.Draw() ; sm_GraphicsLoop() ;
              smgfx_SetLook(6,7, 1,0,1) ; sCurveClass.Draw() ; sm_GraphicsLoop() ;
              sm_GraphicsLoop() ;
            }
#endif // SM_DEBUG_CODE

          // parse the CurveClassification

          // the ray starts with an intersection of a faceuse and moves into
          // the region bounded by the shell.

          // when the CurveClassification has only one interval
          // then the ray moved into the infinite region and sailed on out into space - we're done
          if(sCurveClass.GetSize() == 1)
            {
              // shell is an inner shell of the infinite region
              return( pOptOuterShell == NULL ? TRUE : FALSE) ;
            }
  
          // The shell will be classified by counting the number
          //  of ray/shell intersections between the start point
          //  of the ray and its end which is assumed to be
          //  long beyond the extent of the current Brep.
          //
          // since shells are nicely nested the ray intersections 
          //    will always happen in enter and exit pairs. We
          //    call these paired intersections, 'balanced' and
          //    ingore them for classification purposes.
          //
          // Every Ray/Face intersection represents two ray/Shell 
          //    intersections - first an exit from some shell and
          //      then an enter of another shell.
          //
          //    Ignoring the first intersection where the ray starts 
          //    the first cases where the intersections don't happen
          //    in pairs determine the shell's classification as:
          //
          //   1. the shell is really an outer shell - The ray's first
          //          unbalanced intersection will be an 
          //          exit from the 'this' shell intersection. Return FALSE.
          //   2. the shell is an inner shell in a nonInfinite region.
          //          The ray's first unbalanced intersection will be an
          //          exit from the containing shell.  When that shell
          //          equals the pOptOuterShell pointer value return TRUE.
          //   3. the shell is an inner shell of the Infinite region.
          //          The ray will have no unbalanced intersections.
          //          when pOptOuterShell is NULL return TRUE.

          SmTArray<SmShell *> sShellStack ;

          // for every classification but the first - it starts on the current faceuse 
          for(ii=1;ii<sCurveClass.GetSize();ii++)
            {
              SmCurveInterval &rCurveInterval = sCurveClass[ii] ;

              // get the face intersection point
              SmFace   *pFace = rCurveInterval.m_vStart.GetFaceObject() ;
              SmPoint2d sUV   = rCurveInterval.m_vStart.GetUVParam() ;
              SmFaceuse *pUpwardFaceuse   = pFace->GetUpwardFaceuse() ;
              SmFaceuse *pDownwardFaceuse = pUpwardFaceuse->GetMate() ;

              // skip faces which are interior to a single shell
              //   They are a balanced intersection all on their own and should be ignored.
              if(pUpwardFaceuse->GetShell() == pDownwardFaceuse->GetShell())
                { continue ; }

              // Evaluate the pFace->Surface
              SM_ASSERT(pFace != NULL) ;
              if(pFace == NULL)
                { return(UNSURE) ; }

              SmSurface *pSurface = pFace->GetSurface() ;
              SmPoint3d sPV[2] ;
              pSurface->EvaluatePoint (sUV, sPV[0]) ;
              pSurface->EvaluateNormal(sUV, TRUE, TRUE, sPV[1]) ;

              // figure out which shell is being exited and which shell is being entered
              SmShell *pOrdered[2] ;
              if(sPV[1].Dot(sLineVec) < 0.0) { pOrdered[0] = pUpwardFaceuse->GetShell() ;
                                               pOrdered[1] = pDownwardFaceuse->GetShell() ;
                                             }
              else                           { pOrdered[0] = pDownwardFaceuse->GetShell() ;                  
                                               pOrdered[1] = pUpwardFaceuse->GetShell() ;
                                             }
                                           
              // parse the ordered shells into the shell stack
              for(jj=0;jj<2;jj++)
                {
                  // jj==0 exiting shell
                  // jj==1 entering shell
                  if(   sShellStack.GetSize() > 0
                     && sShellStack.GetLast() == pOrdered[jj])
                    {  // this is a balanced intersection - remove the last stack member
                       sShellStack.RemoveLast() ;
                    }
                  else // this is an unbalanced intersection - add it to the stack
                    {  
                       sShellStack.Add(pOrdered[jj]) ;
                    }
                } // end parsing the exiting and the entering shell events into the shell stack 
            } // end iter every interval classification
      
          // the intersections have been parsed - now figure out the retun value
        
          // check the stack to pick the return value
          if(sShellStack.GetSize() == 0)
            {
              // this is an inner shell of the infinite region
              return( (pOptOuterShell == NULL) ? TRUE : FALSE) ;
            }

          // when pOptOuterShell == NULL we were testing for an infinite region inner shell
          if(pOptOuterShell == NULL)
            { return(FALSE) ; }

          // when pOptOuterShell == sShellStack[0] then this was an inner shell of the given outer shell
          // all other cases return FALSE
          return( (pOptOuterShell == sShellStack[0]) ? TRUE : FALSE) ;
      
        } // end iter every sample point on this faceuse
    } // end iter every faceuse in the shell looking for a simple classification situation

  // arrive here when no candidate ray produced a simple set of intersections
  // to parse.  If this case comes up, either sample more points or 
  // figure out how to parse more complicated cases
  return(UNSURE) ;

} // end SmShell::IsInnerShellOf

/*******************************************************************//**
PURPOSE: Return TRUE when all Shell geometries fit within BBox less than given Tol

NOTES: 
***********************************************************************/
SmBoolean SmShell::IsDegenerate // rtn: TRUE = Shell is degenerate        
 (double d3DTol)                // in : min distance between distinct points,
                                //      default:[SM_EFF_ZERO]
const 
{
  // Vertex Shells are degenerate
  if(IsVertexShell()) 
    { return(TRUE) ; }

  // locals
  SmExtent3d sBBox ;

  // Do expensive check
  CalculateBoundingBox(sBBox) ;
  SmBoolean bRtn = sBBox.IsPointSized(d3DTol) ;

  // all done
  return(bRtn) ;

} // end SmShell::IsDegenerate

/*******************************************************************//**
PURPOSE: Determine whether a Shell encloses a region.


NOTES:
  This assumes that the Brep is valid, i.e., all Regions/Shells/Faceuses,
  etc. are properly connected.  If that's not the case, use the TopologyTraverser.

  Note that a Shell can have nonmanifold Faces and still be closed:
  it can have 'fins' sticking out (or in).
***********************************************************************/
SmBoolean SmShell::IsClosed() const
{
  // A Shell is closed if it separates two regions.
  // That is the case if and only if there is a Face
  // with different Shells on both sides.

  // Only Faceuse shells can be closed.
  if ( ! IsFaceuseShell() ) { return FALSE; }

  SmTArray< SmFaceuse* > sFUs;
  GetFaceuses( sFUs );

  ULONG ii, lNumFUs = sFUs.GetSize();
  for ( ii = 0; ii < lNumFUs; ii++ )
    {
      SmFaceuse *pMate = sFUs[ii]->GetMate();
      if ( pMate != NULL && pMate->GetShell() != this )
        { return TRUE; }
    }
  return FALSE;

} // end IsClosed

/*******************************************************************//**
PURPOSE: Add to new or open displayList added to global displayList array.

NOTES:
***********************************************************************/
SmDisplayList * SmShell::Draw
 (double          dNormalGain,    // in : Amount to scale the Normal vector, 0.0 = Guess from Shell BBox Size, default:[0.0]
  SmGfxArraySet * pOptGfxSet)     // i/o: used for OpenGL ES 2.0. When given, accumulate GfxVertexArrays not GL calls.
                                  //      NULL to ignore, default:[NULL]
 const
{
  SmDisplayList *pRtn = NULL ;

#ifdef SM_GFX_CODE

  // Get global SmGraphicsExtern.cpp:s_Disp display parameters
  SmDisplayParameters &rDisp = smgfx_RefGlobalDisplayParameters() ;

  // pick a global drawing mode for BrepShells
  SmDrawModeType eLastDrawModeType = rDisp.m_eLastMode ;
  smgfx_SetDrawingMode(SM_DM_BREP_SHELLS) ;
  smgfx_SetFaceuseNormalGain(dNormalGain) ;

  // start new displayList (unless one is already open)
  SmVector3d sColor = smgfx_GetOutputColor(pOptGfxSet) ;
  smgfx_Open(smgfx_GetRuleColor(this, rDisp.GetShadedColorRule()), NULL, NULL, FALSE, pOptGfxSet);

  // for all Shell types - output wireEdges
  SmTArray<SmEdge*> sWireEdges;
  GetWireEdges(sWireEdges);
  for (ULONG i=0; i<sWireEdges.GetSize(); i++) 
    {
      sWireEdges[i]->OutputGraphics(rDisp, pOptGfxSet);
    }

  // when drawing FaceuseShells
  if (this->IsFaceuseShell()) 
    {
      // override global display parameters to show just midPoint surfNormal and crosshatched faces
      SmDrawModeType eDrawModeType = rDisp.m_eLastMode ;
      if(rDisp.m_bDrawNormals   == FALSE) { rDisp.m_bDrawNormals   = TRUE ; }
      if(rDisp.m_bDrawWireFrame == FALSE) { rDisp.m_bDrawWireFrame = TRUE ; }
      rDisp.m_bDrawCrossHatch = TRUE; 

      // for every faceuse
      SmTArray<SmFaceuse*> sFaceuses;
      GetFaceuses(sFaceuses);
      SmTArray<SmEdgeuse*> sEdgeuses;
      for (ULONG j=0; j<sFaceuses.GetSize(); j++) 
        {
          SmFaceuse * pFU   = sFaceuses[j];
          SmFace    * pFace = pFU->GetFace();

          // set normals to point outward for NLIB and OpenGL facets
          SmBoolean bReverseNormals = pFU->GetOrientation() == SM_OT_OPPOSITE;

          // always draw surface normal for this Faceuse - load rDisp Faceuse data to tell SmFace::Outputgraphics 
          rDisp.m_bDrawFaceuse  = TRUE ;
          rDisp.m_bDrawLoopuses = TRUE ;
          rDisp.m_pFaceuse      = pFU ;

          // draw surface normal
          pFace->OutputGraphics(rDisp, bReverseNormals, NULL, pOptGfxSet) ;

          // clear global display parameter DrawFaceuse data
          rDisp.m_bDrawFaceuse  = FALSE ;
          rDisp.m_bDrawLoopuses = FALSE ;
          rDisp.m_pFaceuse      = NULL ;

//                // draw every faceuse->edgeuse
//                sFaceuses[j]->GetEdgeuses(sEdgeuses);
//                for (ULONG k=0; k<sEdgeuses.GetSize(); k++)
//                  {
//                    // architecture TODO: build SmEdgeuse::OutputGraphics() method
//                    sEdgeuses[k]->Draw();
//                  }
        } // end iter every face

      smgfx_SetDisplayParameters(eDrawModeType, rDisp) ;
    } // end drawing faceuseShell

  // drawing VertexShell
  else if (this->IsVertexShell()) 
    {
      // output vertex graphics
      SmVertex *pV = this->GetVertex();
      pV->OutputGraphics(pOptGfxSet);
    }

  // restore the global display parameters to previous drawing mode
  smgfx_SetDrawingMode(eLastDrawModeType) ;
  smgfx_SetFaceuseNormalGain(0.0) ;

  // end display list
  smgfx_OutputColor(sColor, pOptGfxSet) ;
  pRtn = smgfx_Close(pOptGfxSet) ;

#else
  SM_REF2(dNormalGain, pOptGfxSet);
#endif // end SM_GFX_CODE
  return(pRtn) ;

} // end SmShell::Draw

/*******************************************************************//**
PURPOSE: Return one point representative of this shells location
            for graphics purposes.

NOTES:
***********************************************************************/
SmPoint3d SmShell::GetGraphicsPoint() const
{
  SmPoint3d sGraphicsPoint ;

  // VertexShell
  if (this->IsVertexShell()) 
    {
      // output vertex graphics
      SmVertex *pV = this->GetVertex();
      sGraphicsPoint = pV->GetPoint() ;
    }

  // WireShell
  else if(this->IsWireShell())
    {
      SmTArray<SmEdge*> sWireEdges;
      GetWireEdges(sWireEdges);
      SmEdge  *pEdge  = sWireEdges.GetAt(0) ;
      SmCurve *pCurve = pEdge->GetCurve() ;

      pCurve->EvaluatePoint(pEdge->GetInterval().Evaluate(.7), sGraphicsPoint) ;
    }
    
  // FaceuseShell
  else if(this->IsFaceuseShell()) 
    {
      // for every faceuse
      SmTArray<SmFaceuse*> sFaceuses;
      GetFaceuses(sFaceuses);
      SmFaceuse *pFaceuse = sFaceuses.GetAt(0) ;

      SmTArray<SmPoint2d> sUV ;
      SmTArray<SmPoint3d> sXYZ ;
      pFaceuse->GetFace()->GetPointsInFace(1, sUV, sXYZ) ;
      sGraphicsPoint = sXYZ[0] ;
    }

  return(sGraphicsPoint) ;

} // end SmShell::GetGraphicsPoint

/*******************************************************************//**
PURPOSE: Get the first wire edge of a wire shell.

NOTES: 
***********************************************************************/
SmEdge * SmShell::GetWireEdge() const
{
  SmEdge *pRet = NULL;
  if (m_tShellType == SmEdgeuse_TYPE) 
    {
      SmEdgeuse *pEU  = SM_CAST_PTR(SmEdgeuse,m_pList);
      if (pEU)   pRet = pEU->GetEdge();
      else SE(SM_ERR);
    }
  return pRet;
}


/******************************
PURPOSE: Returns true when the items in our m_pList
    point back to us with their SmTopology::m_pListOwner pointer.

USAGE NOTES -- Helper for AssertValid() calls.
*********************/
SmBoolean SmShell::TypicalListOwnerUse() const
{
  // True for Faceuse type only: Edgeuse and Vertexuse types do it differently.
  return ( m_tShellType == SmFaceuse_TYPE );
}

/*******************************************************************//**
PURPOSE: Get the region that owns a shell.

NOTES: 
***********************************************************************/
SmRegion* SmShell::GetRegion() const 
{ SM_ASSERT(m_pListOwner != NULL); 
  return (SmRegion*)m_pListOwner; 
}

/*******************************************************************//**
PURPOSE: Get the brep of a shell.

NOTES: 
***********************************************************************/
SmBrep* SmShell::GetBrep() const 
{ SmRegion *pR = GetRegion();
  SM_ASSERT(pR != NULL);
  return ((pR == NULL) ? NULL: pR->GetBrep()); }

/*******************************************************************//**
PURPOSE: Get the faceuses owned by a shell.

NOTES: 
***********************************************************************/
void SmShell::GetFaceuses(SmTArray<SmFaceuse*> & rFaceuses) const 
{ 
  // no faceuses when Shell is not a FaceuseType
  if(m_tShellType != SmFaceuse_TYPE) { rFaceuses.ReSet() ;
                                       return ; 
                                     }
  GetAll(SM_REINTERPRET_CAST(SmTArray<class SmTopology*>&,rFaceuses)); 

} // end SmShell::GetFaceuses

/*******************************************************************//**
PURPOSE: Is this shell a single vertex shell (no edges or faces)?

NOTES: 
***********************************************************************/
SmBoolean SmShell::IsVertexShell() const
{ return( (m_tShellType == SmVertexuse_TYPE) ) ; }

/*******************************************************************//**
PURPOSE: Is this shell a wire shell (no faces)?

NOTES: 
***********************************************************************/
SmBoolean SmShell::IsWireShell() const
{ return( (m_tShellType == SmEdgeuse_TYPE) ) ; }

/*******************************************************************//**
PURPOSE: Is this shell a shell with faceuses?

NOTES: 
***********************************************************************/
SmBoolean SmShell::IsFaceuseShell() const
{ return(  (m_tShellType == SmFaceuse_TYPE) ) ; }

/*******************************************************************//**
PURPOSE: Is this shell an unused shell?

NOTES: 
***********************************************************************/
SmBoolean SmShell::IsUnknownShell() const
{ return(  (m_tShellType == SmUnknown_TYPE) ) ; }

/*******************************************************************//**
PURPOSE: Set the type of the shell.  Internal use only.

NOTES: 
***********************************************************************/
void SmShell::SetShellType(SM_TYPE tShellType) 
{ m_tShellType = tShellType; }

/*******************************************************************//**
PURPOSE: Get the vertex of a vertex shell.

NOTES: 
***********************************************************************/
SmVertex * SmShell::GetVertex() const
{
  SmVertex *pRet = NULL;
  if (m_tShellType == SmVertexuse_TYPE) 
    {
      SmVertexuse *pVU = SM_CAST_PTR(SmVertexuse,m_pList);
      if (pVU) pRet = pVU->GetVertex();
      else SE(SM_ERR);
    }
  return pRet;
}

/*******************************************************************//**
PURPOSE: Get the first faceuse of a faceuse shell.

NOTES: 
***********************************************************************/
SmFaceuse * SmShell::GetFirstFaceuse() const
{  
  SmFaceuse *pRet = NULL;
  if (m_tShellType != SmFaceuse_TYPE) { SE(SM_ERR); }
  else { pRet = SM_CAST_PTR(SmFaceuse, m_pList); 
         if (!pRet) SE(SM_ERR);
       }
  return pRet;
}

/*******************************************************************//**
PURPOSE: AssertValid() Test reporting static labels
***********************************************************************/
SmAssertReportLabel sAssertShell_list[] =
{
  /*  0 */ {SM_AT_POINTER,     _T("Pointer"),           _T("m_pList is non-NULL") },   
  /*  1 */ {SM_AT_SIZE,        _T("SmVertexuse_TYPE"),  _T("sShellVertices has one vertex") },   
  /*  2 */ {SM_AT_SIZE,        _T("SmVertexuse_TYPE"),  _T("sWireEdges is empty ") },   
  /*  3 */ {SM_AT_SIZE,        _T("SmVertexuse_TYPE"),  _T("sShellFaceuses is empty ") },   
  /*  4 */ {SM_AT_POINTER,     _T("SmVertexuse_TYPE"),  _T("pVu share same context") },   
  /*  5 */ {SM_AT_TYPE,        _T("SmVertexuse_TYPE"),   _T("pVu is vertexuse type") },   
  /*  6 */ {SM_AT_POINTER,     _T("SmVertexuse_TYPE"),  _T("pVU->m_pSorLUorEU points back to this") },   
  /*  7 */ {SM_AT_POINTER,     _T("SmEdgeuse_TYPE"),    _T("pEu1 shares the same context") },   
  /*  8 */ {SM_AT_TYPE,        _T("SmEdgeuse_TYPE"),    _T("pEu1 is edgeuse type") },   
  /*  9 */ {SM_AT_POINTER,     _T("SmEdgeuse_TYPE"),    _T("pEu1->m_pSorLU points back to this") },   
  /* 10 */ {SM_AT_SIZE,        _T("SmEdgeuse_TYPE"),    _T("sShellVertices is empty") },   
  /* 11 */ {SM_AT_SIZE,        _T("SmEdgeuse_TYPE"),    _T("sShellFaceuses is empty") },   
  /* 12 */ {SM_AT_SIZE,        _T("SmEdgeuse_TYPE"),    _T("sEdgeuses1 and sEdgeuses2 share the same size") },   
  /* 13 */ {SM_AT_POINTER,     _T("SmEdgeuse_TYPE"),    _T("pEu1 shares same context") },   
  /* 14 */ {SM_AT_POINTER,     _T("SmEdgeuse_TYPE"),    _T("pEu2 shares same context") },   
  /* 15 */ {SM_AT_TYPE,        _T("SmEdgeuse_TYPE"),    _T("pEu1 is edgeuse") },   
  /* 16 */ {SM_AT_POINTER,     _T("SmEdgeuse_TYPE"),    _T("pEu1->m_pSorLU points back to this") },   
  /* 17 */ {SM_AT_TYPE,        _T("SmEdgeuse_TYPE"),    _T("pEu2 is edgeuse ") },   
  /* 18 */ {SM_AT_POINTER,     _T("SmEdgeuse_TYPE"),    _T("pEu2->m_pSorLU points back to this ") },   
  /* 19 */ {SM_AT_POINTER,     _T("SmFaceuse_TYPE"),    _T("pFu shares the same context") },   
  /* 20 */ {SM_AT_POINTER,     _T("SmFaceuse_TYPE"),    _T("pFu->m_pListOwner points back to this") },   
  /* 21 */ {SM_AT_SIZE,        _T("SmFaceuse_TYPE"),    _T("sFaceuses1 and sFaceuses2 are equal") },   
  /* 22 */ {SM_AT_SIZE,        _T("SmFaceuse_TYPE"),    _T("sShellFaceuses and sFaceuses2 are equal ") },   
  /* 23 */ {SM_AT_SIZE,        _T("SmFaceuse_TYPE"),    _T("sShellVertices is empty ") },   
  /* 24 */ {SM_AT_POINTER,     _T("SmFaceuse_TYPE"),    _T("sFaceuses1 shares the same context") },   
  /* 25 */ {SM_AT_POINTER,     _T("SmFaceuse_TYPE"),    _T("sFaceuses1->GetShell() points back to this") },   
  /* 26 */ {SM_AT_POINTER,     _T("SmFaceuse_TYPE"),    _T("sFaceuses2[ii]->GetShell() points back to this") },   
  /* 27 */ /* obsolete */ {SM_AT_POINTER,     _T("SmFaceuse_TYPE"),    _T("pEu shares the same context") },   
  /* 28 */ /* obsolete */ {SM_AT_POINTER,     _T("SmFaceuse_TYPE"),    _T("pShellRadial points back to this") },   
  /* 29 */ /* obsolete */ {SM_AT_POINTER,     _T("SmFaceuse_TYPE"),    _T("pEu and pEuRadial share the same edge") },   
  /* 30 */ /* obsolete */ {SM_AT_DISTANCE,    _T("SmFaceuse_TYPE"),    _T("dDist < dTol") },   
  /* 31 */ /* obsolete */ {SM_AT_DIRECTION,   _T("SmFaceuse_TYPE"),    _T("sThisEuTangent == -sRadialTangent") },   
  /* 32 */ /* obsolete */ {SM_AT_ANGLE,       _T("SmFaceuse_TYPE"),    _T("dNormAngle = dBiAngle - Pi") },    
  /* 33 */ {SM_AT_NESTED_TEST, _T("Bad SubTopology"),   _T("A Shell SubTopology Face, Loop, Edge, or Vertex object failed its AssertValid checks") },
  /* 34 */ {SM_AT_POINTER,     _T("SmFaceuse_TYPE"),    _T("Faceuse->Loopuse is NULL") },   

} ; // end sAssertShell_List

/*******************************************************************//**
PURPOSE:

RETURNS --- TRUE when shell passes all consistency checks

    Shell Rules to follow
    1. Shell->m_pList points to one connected element of the highest dimension type
        to which the shell is connected.
    2. Shell->m_tShellType value matches the type of the object pointed to by Shell->m_pList
       m_tShellType == SmVertexuse_TYPE, m_pList is a pointer to a SmVertexuse (shell vertex)
       m_tShellType == SmEdgeuse_TYPE, m_pList is a pointer to a SmEdgeuse     (wire edge)
       m_tShellType == SmFaceuse_TYPE, m_pList is a pointer to a SmFaceuse     
    3. every object connected directly to a shell has a back pointer to that shell
       SmVertexuse->m_pSorLUorEU
       SmEdgeuse->m_pSorLU
       SmFaceuse->m_pListOwner
    4. mixed dimension shells - only SmFacuse
       a. m_tShellType == SmVertexuse_TYPE;  1 Vertexuse use points back to Shell.
                                                No Edgeuses, No Faceuses
       b. m_tShellType == SmEdgeuse_TYPE;    1 or more sets of two Edgeuses 
                                                   (one set for each wire edge) point back to Shell; 
                                                No Vertexuses, No Faceuses
       c. m_tShellType == SmFaceuse_TYPE;    Any number of Faceuses point back to Shell
                                                0, 1, or more sets of two Edgeuses 
                                                   (one set for each wire edge) point back to Shell;
                                                No Vertexuses
NOTES ---
  increments SM_MT_MARKIO

RETURNS ---  TRUE  = OK
             FALSE = Problem
***********************************************************************/
SmBoolean SmShell::AssertValid
 (SmAssertArray    * pAList,        // i/o: Accumulating list of failed Asserts, NULL to ignore, default:[NULL] 
  SmAssertTestLevel  eTestLevel,    // in : SM_LEVEL_0=fewest tests, SM_LEVEL_1, SM_LEVEL_2=all tests, 
                                    //      SM_LEVEL_GIVEN = run tests in order requested in pTestRequests 
                                    //      default:[SM_LEVEL_0] 
  SmAssertWalking    eWalkTree,     // in : SM_WALK = run AssertValid on any topology graph descendants, SM_NO_WALK=don't, default:[SM_WALK] 
  SmTArray<ULONG>  * pTestRequests) // in : when eTestLevel == SM_LEVEL_GIVEN, run these tests in this order, default:[NULL]
 const
{
  SmBoolean bRtn  = TRUE ;
  
  // call the base class AssertValid
  bRtn &= (  (eTestLevel != SM_LEVEL_GIVEN)
           ? SmOwningTopology::AssertValid(pAList, eTestLevel, SM_NO_WALK, pTestRequests) 
           : TRUE ) ; 

  //  check m_pList/m_lListSize for consistent pointers of (faceuses, edgeuses, or vertexuse)
  //  check m_pListOwner/m_pNext/m_pLast for consistent Region pointers
  bRtn &= SM_ASSERT_BOOLEAN_REPORT(0, SM_LEVEL_0, (m_pList != NULL), _T("") ) ;

  if( m_pList == NULL )
    { 
      return FALSE; 
    }

  // search all brep vertices, edges, and faces for direct back pointers to this shell
  SmTArray<SmVertex*> sShellVertices ;
  SmTArray<SmEdge *>  sWireEdges ;
  SmTArray<SmFaceuse *> sShellFaceuses ;

  sm_GetShellVertices(this, sShellVertices) ;
  GetWireEdges(sWireEdges) ;
  sm_GetShellFaceuses(this, sShellFaceuses) ;

  // check for pointer/geometric consistency
  switch(m_tShellType)
    {
      case SmVertexuse_TYPE:
        { 
          // check vertexuse back pointers to shell
          SmVertexuse *pVu = (SmVertexuse *) m_pList ;
          bRtn &= SM_ASSERT_BOOLEAN_REPORT(1, SM_LEVEL_0, (sShellVertices.GetSize() == 1 && sShellVertices[0] == GetVertex()), _T("") ) ;

          // there should be no wire edges or shell faceuses
          bRtn &= SM_ASSERT_BOOLEAN_REPORT(2, SM_LEVEL_0, (sWireEdges.GetSize() == 0), _T("") ) ;
          bRtn &= SM_ASSERT_BOOLEAN_REPORT(3, SM_LEVEL_0, (sShellFaceuses.GetSize() == 0), _T("") ) ;

          bRtn &= SM_ASSERT_BOOLEAN_REPORT(4, SM_LEVEL_0, (GetContext() == pVu->GetContext()), _T("") ) ;
          bRtn &= SM_ASSERT_BOOLEAN_REPORT(5, SM_LEVEL_0, (pVu->IsShellVertexuse()), _T("") ) ;
          bRtn &= SM_ASSERT_BOOLEAN_REPORT(6, SM_LEVEL_0, ((SmShell *)pVu->m_pSorLUorEU == this), _T("") ) ;
          break ;
        }

      case SmEdgeuse_TYPE  :
        { 
          // check edgeuse back pointers to shell
          SmEdgeuse *pEu2, *pEu1 = (SmEdgeuse *) m_pList ;

          bRtn &= SM_ASSERT_BOOLEAN_REPORT(7, SM_LEVEL_0, (GetContext() == pEu1->GetContext()), _T("") ) ;
          bRtn &= SM_ASSERT_BOOLEAN_REPORT(8, SM_LEVEL_0, (pEu1->IsShellEdgeuse()), _T("") ) ;
          bRtn &= SM_ASSERT_BOOLEAN_REPORT(9, SM_LEVEL_0, ((SmShell *)pEu1->m_pSorLU == this), _T("") ) ;

          // collect all connected edges in two ways - each should be in this shell
          SmTArray<SmEdgeuse *> sEdgeuses1, sEdgeuses2, sEus ;
            {
              // increment and lock an unlocked mark
              SmNewMarkAndLock sMarkLock( pEu1->GetContext(), SM_MT_ALLMARKS) ; // increment and lock any unlocked mark
              SmMarkType eMarkType = sMarkLock.GetMarkType() ;

              pEu1->GetConnectedEdgeuses(sEdgeuses1, eMarkType) ; // uses without incrementing eMarkType value
            }

          // there should be no ShellVertices nor ShellFaceuses
          bRtn &= SM_ASSERT_BOOLEAN_REPORT(10, SM_LEVEL_0, (sShellVertices.GetSize() == 0), _T("") ) ;
          bRtn &= SM_ASSERT_BOOLEAN_REPORT(11, SM_LEVEL_0, (sShellFaceuses.GetSize() == 0), _T("") ) ;

          // get all WireEdge edgeuses
          ULONG ii ;
          for(ii=0;ii<sWireEdges.GetSize();ii++)
            {
              sWireEdges[ii]->GetEdgeuses(sEus) ;
              sEdgeuses2.Append(sEus) ; 
            }

          // number of edgeuses should be same
          bRtn &= SM_ASSERT_BOOLEAN_REPORT(12, SM_LEVEL_0, (sEdgeuses1.GetSize() == sEdgeuses2.GetSize()), _T("") ) ;
           
          // check every edgeuse back pointers to shell
          for( ii=0; ii<sEdgeuses1.GetSize() && ii<sEdgeuses2.GetSize(); ii++)
            {
              pEu1    = sEdgeuses1[ii] ;
              pEu2    = sEdgeuses2[ii] ;
              bRtn &= SM_ASSERT_BOOLEAN_REPORT(13, SM_LEVEL_0, (GetContext() == pEu1->GetContext()), _T("") ) ;
              bRtn &= SM_ASSERT_BOOLEAN_REPORT(14, SM_LEVEL_0, (GetContext() == pEu2->GetContext()), _T("") ) ;
              bRtn &= SM_ASSERT_BOOLEAN_REPORT(15, SM_LEVEL_0, (pEu1->IsShellEdgeuse()), _T("") ) ;
              bRtn &= SM_ASSERT_BOOLEAN_REPORT(16, SM_LEVEL_0, ((SmShell *)pEu1->m_pSorLU == this), _T("") ) ;
              bRtn &= SM_ASSERT_BOOLEAN_REPORT(17, SM_LEVEL_0, (pEu2->IsShellEdgeuse()), _T("") ) ;
              bRtn &= SM_ASSERT_BOOLEAN_REPORT(18, SM_LEVEL_0, ((SmShell *)pEu2->m_pSorLU == this), _T("") ) ;
            }
          break ;
        }

      case SmFaceuse_TYPE  :
        { 
          // check faceuse back pointers to shell
          SmFaceuse *pFu = (SmFaceuse *) m_pList ;
          bRtn &= SM_ASSERT_BOOLEAN_REPORT(19, SM_LEVEL_0, (GetContext() == pFu->GetContext()), _T("") ) ;
          bRtn &= SM_ASSERT_BOOLEAN_REPORT(20, SM_LEVEL_0, ((SmShell *)pFu->m_pListOwner == this), _T("") ) ;

          // Collect all connected faceuses three ways:
          // 1: Faceuses1 : our topology list m_pList, obtained by GetFaceuses().
          // 2: Faceuses2 : by doing a topology traversal.
          // 3: sShellFaceuses : All FU's in the Brep that point back to this Shell
          //            (already collected).

          // get faceuses through faceuse next/last linked list
          SmTArray<SmFaceuse *> sFaceuses1, sFaceuses2 ;
          GetFaceuses(sFaceuses1) ; 

          // get 1st shell vertexuse
          SmFaceuse *pFU = GetFirstFaceuse() ;
          SmLoopuse *pLU = (SmLoopuse *)pFU->m_pList ;
          if (!pLU)
          {
              bRtn &= SM_ASSERT_BOOLEAN_REPORT(34, SM_LEVEL_0, FALSE, _T(""));
              return bRtn;
          }
          bRtn &= SM_ASSERT_BOOLEAN_REPORT(21, SM_LEVEL_0, (pLU->IsEdgeLoopuse()), _T("") ) ;
          SmEdgeuse *pEU = (SmEdgeuse *)pLU->m_pEUorVU ;

          // get faceuses through a shell traversal
          // side-effect: increment unused Mark value
          if(pEU->GetVertexuse() != NULL)
            {
              SmTopologyTraverser sTT;
              SmNewMarkAndLock sMarkLock(GetContext(), SM_MT_ALLMARKS) ; // increment and lock any unlocked mark
              sTT.ShellTraversal(pEU->GetVertexuse()->GetVertex(),  // in : target vertex
                                 (SmShell *)this,                   // in : target shell
                                 NULL,                              // out: optional list of edges in shell (not marked)
                                 &sFaceuses2,                       // out: optional list of faces in shell (get marked)
                                 sMarkLock) ;                       // in : specify mark for target objects (incremented)
            }

          // number of faceuses collected all ways should be the same
          bRtn &= SM_ASSERT_BOOLEAN_REPORT(22, SM_LEVEL_0, (sFaceuses1.GetSize() == sFaceuses2.GetSize()), _T("") ) ;
          bRtn &= SM_ASSERT_BOOLEAN_REPORT(23, SM_LEVEL_0, (sShellFaceuses.GetSize() == sFaceuses2.GetSize()), _T("") ) ;

          // there can be wire edges but no shell vertices
          bRtn &= SM_ASSERT_BOOLEAN_REPORT(24, SM_LEVEL_0, (sShellVertices.GetSize() == 0), _T("") ) ;

          if(sFaceuses1.GetSize() != sFaceuses2.GetSize() )
            {
              ULONG ii ;
              SmTArray<SmFaceuse*> sFaceuses1Minus2 ;
              SmTArray<SmFaceuse*> sFaceuses2Minus1 ;
              SmTArray<SmFaceuse*> sTNeighbors1Minus2, sNeighbors1Minus2 ;
              SmTArray<SmFaceuse*> sTNeighbors2Minus1, sNeighbors2Minus1 ;
              SmTArray<SmFaceuse*> sNeighbors ;

              // get the unique faceuses
              sFaceuses1.RemoveElements(sFaceuses2, sFaceuses1Minus2) ; 
              sFaceuses2.RemoveElements(sFaceuses1, sFaceuses2Minus1) ;

              // to check connectivities - get neighbor faces to unique faces
              for(ii=0;ii<sFaceuses1Minus2.GetSize();ii++)
                { SmFaceuse *pFUse    = sFaceuses1Minus2[ii] ;
                  pFUse->GetNeighbors(sNeighbors) ;
                  sTNeighbors1Minus2.Append(sNeighbors) ;
                  sNeighbors.ReSet() ;
                }
              for(ii=0;ii<sFaceuses2Minus1.GetSize();ii++)
                { SmFaceuse *pFUse    = sFaceuses2Minus1[ii] ;
                  pFUse->GetNeighbors(sNeighbors) ;
                  sTNeighbors2Minus1.Append(sNeighbors) ;
                  sNeighbors.ReSet() ;
                }

              // remove the unique faceuses from the neighbor lists 
              sTNeighbors1Minus2.RemoveElements(sFaceuses1Minus2, sNeighbors1Minus2) ; 
              sTNeighbors2Minus1.RemoveElements(sFaceuses2Minus1, sNeighbors2Minus1) ;


#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
              // draw 
              if(bDebugMe)
                {
                  SmBrep *pBrep = GetBrep() ;

                  smgfx_Erase() ;
                  smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
                  smgfx_SetLook(1,2, 0,1,1) ; for(ii=0;ii<sFaceuses1Minus2.GetSize();ii++)
                                                { SmFaceuse *pFUse    = sFaceuses1Minus2[ii] ;
                                                  SmFace    *pFace  = pFUse->GetFace() ;
                                                  if(pFace) { pFace->Draw( SM_DM_CROSSHATCH ); sm_GraphicsLoop(); }
                                                }
                  smgfx_SetLook(1,2, 1,0,1) ; for(ii=0;ii<sNeighbors1Minus2.GetSize();ii++)
                                                { SmFaceuse *pFUse    = sNeighbors1Minus2[ii] ;
                                                  SmFace    *pFace  = pFUse->GetFace() ;
                                                  if(pFace) { pFace->Draw( SM_DM_CROSSHATCH ); sm_GraphicsLoop(); }
                                                }
                  smgfx_SetLook(1,2, 1,0,0) ; for(ii=0;ii<sFaceuses2Minus1.GetSize();ii++)
                                                { SmFaceuse *pFUse    = sFaceuses2Minus1[ii] ;
                                                  SmFace    *pFace  = pFUse->GetFace() ;
                                                  if(pFace) { pFace->Draw( SM_DM_CROSSHATCH ); sm_GraphicsLoop(); }
                                                }
                  smgfx_SetLook(1,2, 1,1,0) ; for(ii=0;ii<sNeighbors2Minus1.GetSize();ii++)
                                                { SmFaceuse *pFUse    = sNeighbors2Minus1[ii] ;
                                                  SmFace    *pFace  = pFUse->GetFace() ;
                                                  if(pFace) { pFace->Draw( SM_DM_CROSSHATCH ); sm_GraphicsLoop(); }
                                                }
                  sm_GraphicsLoop() ;
                }
#endif // SM_DEBUG_CODE

            }

          // for every faceuse pair (should be same number)
          ULONG NumFaceUses = smos_Min(sFaceuses1.GetSize(), sFaceuses2.GetSize());
          for(ULONG ii=0;ii<NumFaceUses;ii++)
            {
              // every faceuse should point to this shell
              bRtn &= SM_ASSERT_BOOLEAN_REPORT(24, SM_LEVEL_0, (GetContext() == sFaceuses1[ii]->GetContext()), _T("") ) ;
              bRtn &= SM_ASSERT_BOOLEAN_REPORT(25, SM_LEVEL_0, (sFaceuses1[ii]->GetShell() == this), _T("") ) ;
              bRtn &= SM_ASSERT_BOOLEAN_REPORT(26, SM_LEVEL_0, (sFaceuses2[ii]->GetShell() == this), _T("") ) ;

              // Radial shell membership and sector geometry (former tests 27-32)
              // are owned by SmEdgeuse 8-10 / 22-24 and SmEdge 9-11 / 14.
            } // end iter every faceuse

          break ;
        } // end case SmFaceuse_TYPE

      default: bRtn = FALSE ;

    } // end switch on Shell type

  // when asked - AssertValid for a topology graph traversal
  if(eWalkTree == SM_WALK)
    {
      bRtn &= SmBrep::AssertSubTopology( (SmTopology &)*this, pAList, eTestLevel) ;
      // bRtn &= SM_ASSERT_BOOLEAN_REPORT(33, SM_LEVEL_0, (SmBrep::AssertSubTopology( (SmTopology &)*this, pAList, eTestLevel)), _T("")) ;

    } // end asked to walk the tree check

  // all done

  return(bRtn) ;

} // end SmShell::AssertValid

// obsolete
// /*******************************************************************//**
// PURPOSE: Fix Assert Report failures reported by an AssertValid call
// 
// RETURNS ---  TRUE  = OK
//              FALSE = Problem
// ***********************************************************************/
// SmBoolean SmShell::AssertHeal
//  (SmAssertReport & rAReport,  // in : a report generated by AssertValid
//   SmAssertArray  * pAList)    // in : AssertArray holding rAReport 
// {
//   SmBoolean bRtn = FALSE ;
// 
//   // check state - no work
//   if(rAReport.m_bOK == TRUE)
//     { return( TRUE ) ; }
// 
//   // check state - not the class that generated this report - pass call to parent class
//   if(rAReport.m_lReportingType != GetClassType())
//     {
//       // pass the call along to the parent - return ( Parent::AssertHeal(rAReport, pAList) ) ;
//       return ( SmOwningTopology::AssertHeal(rAReport, pAList) ) ;
//     }
//    
//   // branch on the report type
//   switch(rAReport.m_lTestIndex)
//     {
//       case 99 : { // set case number appropriately - run fix code here
//                   // if fix works set rAReport.m_bOK = TRUE ; 
//                 }
//                 break ;
// 
//       default: rAReport.m_eAssertType  = SM_AT_NO_HEAL_YET ;  
//                rAReport.m_pHealMessage = _T("SmShell::AssertHeal fix not yet supported") ;  
//                 
//     }
// 
//   // all done
//   return(bRtn) ;
// 
// } // end SmShell::AssertHeal
// end obsolete

/*******************************************************************//**
PURPOSE:
NOTES:
***********************************************************************/
SmBoolean SmShell::IsKindOf( SM_TYPE t ) const
{
  return ((SmShell_TYPE == t) ? TRUE : SmOwningTopology::IsKindOf( (t) ));
}


/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
void SmShell::Dump(void) const
{
  // region
  //SmRegion *pRegion = GetRegion() ;

  // Get the lists of things contained in this shell
  SmTArray<SmFaceuse*> sFaceuses ;  GetFaceuses(sFaceuses) ;
  SmTArray<SmEdge*>    sWireEdges ; GetWireEdges(sWireEdges) ;
  SmVertex            *pVertex    = GetVertex() ;

  // write Shell ePointerValue and Type
  TCHAR sBuff[SM_TBLOCK_SIZE], sBuffForFile[SM_TBLOCK_SIZE];
  smos_sprintf(sBuff, _T("\nShell[0x%p] = %s - m_pListOwner(Region)[0x%p] - m_pList(%s)[0x%p] length:%ld, faceuses:[%ld], WireEdges:[%ld], Vertices:[%d]\n"), 
             this,
               IsVertexShell()  ? _T("Vertex Shell")
             : IsWireShell()    ? _T("Wire Shell")
             : IsFaceuseShell() ? _T("Faceuse Shell")
             : _T("UnDefined Shell Type"),
             m_pListOwner,
               m_pList == NULL  ? _T("No List")
             : m_pList->IsKindOf(SmFaceuse_TYPE)   ? _T("Faceuse")
             : m_pList->IsKindOf(SmEdgeuse_TYPE)   ? _T("Edgeuse")
             : m_pList->IsKindOf(SmVertexuse_TYPE) ? _T("Vertexuse")
             : _T("BadType"),
             m_pList, m_lListSize, 
             sFaceuses.GetSize(), sWireEdges.GetSize(), pVertex ? 1 : 0);
  smos_sprintf(sBuffForFile, _T("\nShell[%s] = %s - m_pListOwner(Region)[%s] - m_pList(%s)[%s] length:%ld, faceuses:[%ld], WireEdges:[%ld], Vertices:[%d]\n"), 
             _T("notNULL"),
               IsVertexShell()  ? _T("Vertex Shell")
             : IsWireShell()    ? _T("Wire Shell")
             : IsFaceuseShell() ? _T("Faceuse Shell")
             : _T("UnDefined Shell Type"),
             m_pListOwner ? _T("notNULL") : _T("NULL"),
               m_pList == NULL  ? _T("No List")
             : m_pList->IsKindOf(SmFaceuse_TYPE)   ? _T("Faceuse")
             : m_pList->IsKindOf(SmEdgeuse_TYPE)   ? _T("Edgeuse")
             : m_pList->IsKindOf(SmVertexuse_TYPE) ? _T("Vertexuse")
             : _T("BadType"),
             m_pList ? _T("notNULL") : _T("NULL"), m_lListSize, 
             sFaceuses.GetSize(), sWireEdges.GetSize(), pVertex ? 1 : 0);
  smos_WriteBuffer(sBuff, sBuffForFile);

  // output Shell's owned topology objects
  ULONG i ;

  // faceuses
  for(i=0;i<sFaceuses.GetSize();i++)
    {
      SmFaceuse   * pFaceuse = sFaceuses[i] ;
      SmFace      * pFace    = pFaceuse->GetFace() ;
      SmShell     * pShell   = pFaceuse->GetShell() ;
      SmOrientType  eOrient  = pFaceuse->GetOrientation() ;

      smos_sprintf(sBuff, _T("  Faceuse[%2ld]  = 0x%p (%s) -> Face[0x%p] -> Shell[0x%p]\n"),
                  i, pFaceuse,
                  eOrient == SM_OT_SAME ? _T("SAME    ") : _T("OPPOSITE"),
                  pFace, pShell) ; 
      smos_sprintf(sBuffForFile, _T("  Faceuse[%2ld]  = %s (%s) -> Face[%s] -> Shell[%s]\n"),
                  i, pFaceuse ? _T("notNULL") : _T("NULL"),
                  eOrient == SM_OT_SAME ? _T("SAME    ") : _T("OPPOSITE"),
                  pFace ? _T("notNULL") : _T("NULL"), 
                  pShell ? _T("notNULL") : _T("NULL")) ; 
      smos_WriteBuffer(sBuff, sBuffForFile);
    } // end iter every faceuse 

  // wire edges
  for(i=0;i<sWireEdges.GetSize();i++)
    {
      SmTArray<SmEdgeuse*> sEdgeuses ;
      SmEdge *pWireEdge = sWireEdges[i] ;
      pWireEdge->GetEdgeuses(sEdgeuses) ;
      SM_ASSERT(sEdgeuses.GetSize() == 2) ;
      SmShell *pShell1 = sEdgeuses[0]->GetShell() ;
      SmShell *pShell2 = sEdgeuses[1]->GetShell() ;
      SM_ASSERT(pShell1 == pShell2) ;

      smos_sprintf(sBuff, _T("  WireEdge[%2ld] = 0x%p -> EUs[0x%p 0x%p] -> Shell[0x%p 0x%p] (should be same shells)\n"),
                  i, pWireEdge, sEdgeuses[0], sEdgeuses[1], pShell1, pShell2) ; 
      smos_sprintf(sBuffForFile, _T("  WireEdge[%2ld] = %s -> EUs[%s %s] -> Shell[%s %s] (should be same shells)\n"),
                  i, pWireEdge ? _T("notNULL") : _T("NULL"), 
                  sEdgeuses[0] ? _T("notNULL") : _T("NULL"), 
                  sEdgeuses[1] ? _T("notNULL") : _T("NULL"), 
                  pShell1 ? _T("notNULL") : _T("NULL"), 
                  pShell2 ? _T("notNULL") : _T("NULL")) ; 
      smos_WriteBuffer(sBuff, sBuffForFile);
    } // end iter every wire edge 

  // vertex
  if(pVertex)
    {
      SmTArray<SmVertexuse*> sVertexuses ;
      pVertex->GetVertexuses(sVertexuses) ;
      SM_ASSERT(sVertexuses.GetSize() == 1) ;
      SmShell *pShell = sVertexuses[0]->GetShell() ;

      smos_sprintf(sBuff, _T("  Vertex[ 0]   = 0x%p -> VU[0x%p] -> Shell[0x%p]\n"),
                 pVertex, sVertexuses[0], pShell) ; 
      smos_sprintf(sBuffForFile, _T("  Vertex[ 0]   = %s -> VU[%s] -> Shell[%s]\n"),
                 pVertex ? _T("notNULL") : _T("NULL"), 
                 sVertexuses[0] ? _T("notNULL") : _T("NULL"), 
                 pShell ? _T("notNULL") : _T("NULL")) ; 
      smos_WriteBuffer(sBuff, sBuffForFile);          
    } // end iter every vertex

  // extra line
  smos_WriteBuffer(_T("\n"));

} // end SmShell::Dump

/*******************************************************************//**
PURPOSE:  Pretty print pointer values for this Shell showing how
             it connects to its neighbor Region and Faces, Edges, or Vertices
             as appropriate in the topology graph.

NOTES: Only good for debugging because pointer values don't
                stay constant from run to run.
***********************************************************************/
void SmShell::DumpTopology
  (ULONG lWalkDepth) // in : 0 = no walking, 1 = faces, wires, and shellVertices, ... 99 = walk to bottom
 const
{
  // locals
  ULONG        ii, nFoundIndex ;
  TCHAR        sBuff[SM_TBLOCK_SIZE], sBuffForFile[SM_TBLOCK_SIZE] = {};
  SmFaceuse   *pFu1, *pFu2 ;
  SmFace      *pFace ;
  SmEdge      *pEdge ;
  SmEdgeuse   *pEu1, *pEu2 ;
  SmVertexuse *pVu ;

  // count the number of things directly connected to this shell
  SmTArray<SmFaceuse*>   sFaceuses ;  GetFaceuses(sFaceuses) ;
  SmTArray<SmEdge*>      sWireEdges ; GetWireEdges(sWireEdges) ;
  SmTArray<SmVertexuse*> sVertexuses ; 
  SmVertex *pVertex = GetVertex() ;

  // output Region connection
  smos_sprintf(sBuff,_T("  Shell  [0x%p] -> Region [0x%p] and has %ld Faceuses, %ld Edgeuses, and %d Vertices (Type = %s)\n"), 
             this, m_pListOwner, sFaceuses.GetSize(), sWireEdges.GetSize()*2, pVertex == NULL ? 0 : 1,
               m_tShellType == SmFaceuse_TYPE   ? _T("Faceuse Shell")
             : m_tShellType == SmEdgeuse_TYPE   ? _T("WireEdge Shell")
             : m_tShellType == SmVertexuse_TYPE ? _T("Vertex Shell")
             : _T("BadValue")) ;
  smos_WriteBuffer(sBuff, sBuffForFile);

  // output owned topology objects 

  // Faceuses
  if(sFaceuses.GetSize() > 0)
    {
      SmFaceuse* pFaceuse   = (SmFaceuse *)m_pList ;
      SM_ASSERT(sFaceuses.GetSize() == m_lListSize) ;

      // output every owned faceuse
      for(ii=0;ii<m_lListSize;ii++,pFaceuse=(SmFaceuse *)pFaceuse->m_pNext)
        {
          pFace = pFaceuse->GetFace() ;
          pFace->GetFaceuses(pFu1, pFu2) ;

          // check for pointer consistency as possible
          SM_ASSERT(pFaceuse->m_pNext->GetLast() == pFaceuse) ;
          SM_ASSERT((SmShell *)(pFaceuse->m_pListOwner) == this) ;
          SM_ASSERT(   pFaceuse == pFu1
                    || pFaceuse == pFu2) ;
          smos_sprintf(sBuff,_T("    Shell[0x%p] <- Faceuse[0x%p] -> Face[0x%p], NextFaceuse[0x%p] -> Face[0x%p]\n"), 
                    pFaceuse->GetShell(), 
                    pFaceuse, 
                    pFaceuse->GetFace(), 
                    pFaceuse->m_pNext, 
                    ((SmFaceuse*)pFaceuse->m_pNext)->GetFace()) ;
          smos_WriteBuffer(sBuff, sBuffForFile);

        } // end iter every faceuse

    } //end owned faceuse check

  // WireEdges
  if(sWireEdges.GetSize() > 0)
    {
      // check that m_pList value is ok
      if(m_tShellType == SmEdgeuse_TYPE)
        {
          pEu1   = (SmEdgeuse *)m_pList ;
          pEdge  = pEu1->GetEdge() ;
          nFoundIndex = 0;
          SM_ASSERT(sWireEdges.FindElement(pEdge, nFoundIndex)) ;
        } // end extra Edgeuse Type shell check

      // for every wireEdge - output 2 edgeuse connectivity reports
      for(ii=0;ii<sWireEdges.GetSize();ii++)
        {
          // get both edge->edgeuses - make sure there are exactly 2 which sit in this shell
          pEdge = sWireEdges[ii] ;
          pEu1  = (SmEdgeuse *)pEdge->GetList() ;
          pEu2  = (SmEdgeuse *)pEu1->GetNext() ;
          SM_ASSERT(pEu1 == (SmEdgeuse *)pEu2->GetNext()) ;
          SM_ASSERT(pEu1->GetShell() == this) ;
          SM_ASSERT(pEu2->GetShell() == this) ;
          SM_ASSERT(pEu1->GetEdge() == pEdge) ;
          SM_ASSERT(pEu2->GetEdge() == pEdge) ;
          smos_sprintf(sBuff,_T("    Shell[0x%p] -> Edgeuse[0x%p] (MateEdgeuse [0x%p]) -> Edge[0x%p]\n"), 
                    pEu1->GetShell(), pEu1, pEu2, pEu1->GetEdge()) ;
          smos_WriteBuffer(sBuff, sBuffForFile);
          smos_sprintf(sBuff,_T("    Shell[0x%p] -> MateEdgeuse[0x%p] (Edgeuse [0x%p]) -> Edge[0x%p]\n"), 
                    pEu2->GetShell(), pEu2, pEu1, pEu2->GetEdge()) ;
          smos_WriteBuffer(sBuff, sBuffForFile);
        } // end iter every WireEdge

     } // end WireEdge existence check              

   // Vertex
   if(pVertex)
     {
       // check shell type consistency
       pVertex->GetVertexuses(sVertexuses) ;
       SM_ASSERT(m_tShellType == SmVertexuse_TYPE) ;
       SM_ASSERT(sVertexuses.GetSize() == 1) ;
       SM_ASSERT(m_lListSize == 1) ;

       // check back pointers
       pVu = sVertexuses[0] ;

       SM_ASSERT(this == pVu->GetShell()) ;
       SM_ASSERT(pVu->GetVertex() == pVertex) ;

       // output contained vertex report
       smos_sprintf(sBuff,_T("    Shell[0x%p] -> Vertexuse[0x%p] -> Vertex[0x%p]\n"), 
                 pVu->GetShell(), pVu, pVertex) ;
       smos_WriteBuffer(sBuff, sBuffForFile);
    } // end switch on m_tShellType

  if(lWalkDepth > 0)
    {
      for(ii=0;ii<sFaceuses.GetSize();ii++)
        {
          SmFaceuse *pFaceuse = sFaceuses[ii] ;
          if(pFaceuse && pFaceuse->GetFace()) pFaceuse->GetFace()->DumpTopology(lWalkDepth == 99 ? lWalkDepth : lWalkDepth - 1) ;
        }

      for(ii=0;ii<sWireEdges.GetSize();ii++)
        {
          SmEdge *pWireEdge = sWireEdges[ii] ;
          if(pWireEdge) pWireEdge->DumpTopology(lWalkDepth == 99 ? lWalkDepth : lWalkDepth - 1) ;
        }

      if(pVertex) pVertex->DumpTopology(lWalkDepth == 99 ? lWalkDepth : lWalkDepth - 1) ;
    } 

} // end SmShell::DumpTopology

