// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************//**
* FILE NAME --- SmLoopuse.cpp
* PURPOSE: Source file for SmLoopuse class methods.
**********************************************************************/

#include "StdAfx.h"

#include <SmLoopuse.h>

#ifndef __SMBREP_H__
#include <SmBrep.h>
#endif

#ifndef __SMFACE_H__
#include <SmFace.h>
#endif

#ifndef __SMLOOP_H__
#include <SmLoop.h>
#endif

#ifndef __SMEDGE_H__
#include <SmEdge.h>
#endif

#ifndef __SMEDGEUSE_H__
#include <SmEdgeuse.h>
#endif

#ifndef __SMVERTEX_H__
#include <SmVertex.h>
#endif

#ifndef __UNORDERED_SET__
#define __UNORDERED_SET__
#include <unordered_set>
#endif

#include <SmGraphicsOutput.h>
#include <SmPlane.h>
#include <SmCrvOnSurf.h>
#include <SmAssertArray.h>
#include <SmPseudoBox.h>

//    // For debugging only - a set of pointers that can be assigned and inspected through the watch window
//    #ifdef SM_DEBUG_CODE
//
//    SmLoopuse * dbgLoopuse1 = NULL ;
//    SmLoopuse * dbgLoopuse2 = NULL ;
//
//    #endif // SM_DEBUG_CODE

/*******************************************************************//**
PURPOSE: Form EdgeLoopuse: Connect owner Loopuse with owned 
         Edgeuse->CW/CCW_LinkedList Edgeuses and set TYPE vals

NOTES: Set this Loopuse type     :[SmEdgeuse_TYPE]
                        m_pEUorVU:[TgtEdgeuse]
       Set every Edgeuse on the TgtEdgeuse's CW/CCW doubly linked list
                        type     :[SmLoopuse_TYPE]
                        m_pSorLU :[this Loopuse]
***********************************************************************/
SmStatus SmLoopuse::CollectEdgeuses
  (SmEdgeuse * pEdgeuse)            // in : Edgeuse to be Loopuse->PrimaryEU
{
  // check state - input must be NonNULL
  NER(pEdgeuse);

  // local
  ULONG lCount = 0;

  // Set Loopuse's 1st Edgeuse m_pEUorVU BackPtr and Type to Tgt Edgeuse
  m_pEUorVU      = pEdgeuse ;
  m_tLoopuseType = SmEdgeuse_TYPE ;

  // for every sibling Edgeuse
  while (TRUE)
    {
      // check state - Edgeuse must be NonNULL
      if ( pEdgeuse == NULL )
        { NER (NULL); }

      // set pEdgeuse useType to SmLoopuse_TYPE and its BackPtr to this Loopuse
      pEdgeuse->m_tEdgeuseType = SmLoopuse_TYPE;
      pEdgeuse->m_pSorLU       = this;

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
      if (bDebugMe) // Draw Edgeuse just changed
        {
         smgfx_ChangeColor(TRUE); pEdgeuse->Draw(); sm_GraphicsLoop();
         sm_GraphicsLoop();
        }
#endif // SM_DEBUG_CODE

      // quit when finished walking the circular linked list
      if (pEdgeuse->m_pCCW == m_pEUorVU)
        { break ; }

      // check state - make sure CCW and CW ptrs form a double linked list
      SM_ASSERT(pEdgeuse->m_pCCW->m_pCW == pEdgeuse);
      SM_ASSERT(pEdgeuse->m_pCW->m_pCCW == pEdgeuse);

      // increment to the next sibling Edgeuse
      pEdgeuse = pEdgeuse->m_pCCW;

      // reality check - It is highly unlikely that we will ever have
      // more than 100k edges.  Error if we get that far.
      lCount++;
      if (lCount > 100000) { SER(SM_ERR); }

    }  // end while walking Edgeuse sibling list

  // all done
  return SM_SUCCESS;

} // end SmLoopuse::CollectEdgeuses

/*******************************************************************//**
PURPOSE: Get all of the Edges belonging to this loop.

NOTES: Returns Edges in CCW order but lists an Edge with
       two edguses in this loop (seams and dividers) just once. Overloaded per FS.
***********************************************************************/
void SmLoopuse::GetEdges   // not GetEdgeuses
(std::unordered_set<SmEdge*>& rEdges,           // out: array of collected topology objects
  ULONG* pOptAttributeId)  // in : only include objects containing an attribute with this id
                                        //      NULL to ignore, default:[NULL]
  const
{
  // no work - no Edgeuses
  if (IsVertexLoopuse())
  {
    return;
  }

  // locals
  SmEdgeuse* pStartEU = SM_CAST_PTR(SmEdgeuse, m_pEUorVU); SM_ASSERT(pStartEU != NULL);
  SmEdgeuse* pEU = pStartEU;

  // for every Edgeuse until we get back tothe StartEU
  do {
    // for NULL terminated lists
    if (pEU == NULL)
    {
      return;
    }

    // locals
    SmEdge* pE = pEU->GetEdge();

    // build output
    if (pOptAttributeId == NULL
      || pE->FindAttribute(*pOptAttributeId) != NULL)
    {
      rEdges.insert(pE);
    }

    // next iter
    pEU = pEU->m_pCCW;
    SM_ASSERT(pEU != NULL);

    // quit when we get back to the Starting Edgeuse
  } while (pEU != pStartEU);

} // end SmLoopuse::GetEdges

/*******************************************************************//**
PURPOSE: Get the Vertices associated with the loopuse.

NOTES: Returns Vertices in CCW order.
***********************************************************************/
void SmLoopuse::GetVertices
(std::unordered_set<SmVertex*>& rVertices,        // out: array of collected topology objects
  ULONG* pOptAttributeId)  // in : only include objects containing an attribute with this id
                                          //      NULL to ignore, default:[NULL]
  const
{
  // When Loopuse_TYPE
  if (IsVertexLoopuse())
  {
    // get the only Vertex
    SmVertexuse* pVU = (SmVertexuse*)m_pEUorVU; SM_ASSERT(pVU != NULL);
    SmVertex* pV = pVU->GetVertex();

    // set output
    if (pOptAttributeId == NULL
      || pV->FindAttribute(*pOptAttributeId) != NULL)
    {
      rVertices.insert(pV);
    }

    // all done
    return;

  } // end Loopuse_TYPE check

// locals
  SmEdgeuse* pStartEU = (SmEdgeuse*)m_pEUorVU;
  SM_ASSERT(pStartEU != NULL);
  SmEdgeuse* pEU = pStartEU;

  // for every Edgeuse until we get back to the StartEU
  do {
    // for NULL terminated lists
    if (pEU == NULL)
    {
      return;
    }

    // locals 
    SmVertexuse* pVU = pEU->GetVertexuse();
    SmVertex* pV = pVU->GetVertex();

    // build output
    if (pOptAttributeId == NULL
      || pV->FindAttribute(*pOptAttributeId) != NULL)
    {
      rVertices.insert(pV);
    }

    // next iter
    pEU = pEU->m_pCCW; SM_ASSERT(pEU != NULL);

    // quit when we get back to the Starting Edgeuse
  } while (pEU != pStartEU);
} // end SmLoopuse::GetVertices
//!FS


/*******************************************************************//**
PURPOSE: Get all of the Edges belonging to this loop.

NOTES: Returns Edges in CCW order but lists an Edge with
       two edguses in this loop (seams and dividers) just once.
***********************************************************************/
void SmLoopuse::GetEdges   // not GetEdgeuses
 (SmTArray<SmEdge*> & rEdges,           // out: array of collected topology objects
  ULONG             * pOptAttributeId)  // in : only include objects containing an attribute with this id
                                        //      NULL to ignore, default:[NULL]
 const
{
  // init output
  rEdges.ReSet();

  // no work - no Edgeuses
  if (IsVertexLoopuse()) 
    { return; }

  // locals
  SmEdgeuse *pStartEU = SM_CAST_PTR(SmEdgeuse, m_pEUorVU ) ; SM_ASSERT(pStartEU != NULL) ;
  SmEdgeuse *pEU      = pStartEU ;

  // for every Edgeuse until we get back tothe StartEU
  do {
       // for NULL terminated lists
       if ( pEU == NULL )
         { return; }

       // locals
       SmEdge *pE = pEU->GetEdge() ;

       // build output
       if(   pOptAttributeId == NULL
          || pE->FindAttribute(*pOptAttributeId) != NULL)  
         { rEdges.AddUnique(pE) ; }

       // next iter
       pEU = pEU->m_pCCW;
       SM_ASSERT(pEU != NULL);

       // quit when we get back to the Starting Edgeuse
     } while (pEU != pStartEU);

} // end SmLoopuse::GetEdges

/*******************************************************************//**
PURPOSE: Get all of the Edgeuses belonging to this loop.

NOTES: Returns Edgeuses in CCW order. 
  where: thisEdgeuse->EndPt == CCWEdgeuse->StartPt (with tolerances) 

  Unlike GetEdges, two edgeuses
  from one edge being used in this loopuse both get listed.
***********************************************************************/
void SmLoopuse::GetEdgeuses                // not GetEdges
 (SmTArray<SmEdgeuse*> & rEdgeuses,        // out: array of collected topology objects
  ULONG                * pOptAttributeId,       // in : only include objects containing an attribute with this id
                                                //      NULL to ignore, default:[NULL]
  SmTopology           * pOptConnectedToTarget) // in : only include objects connected to this topology object
                                           //      NULL to ignore, default:[NULL]
 const
{
  // init output
  rEdgeuses.ReSet() ;

  // no work - no Edgeuses
  if (IsVertexLoopuse())
    { return; }

  // locals
  SmEdgeuse * pStartEU = (SmEdgeuse*)m_pEUorVU; SM_ASSERT(pStartEU != NULL) ;
  SmEdgeuse * pEU      = pStartEU ;

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ; 
          if(bDebugMe)
            {
              ULONG di ;
              TCHAR sBuff[SM_TBLOCK_SIZE] ;

              SmTArray<SmEdgeuse *> sEdgeuses ; 
              this->GetEdgeuses(sEdgeuses) ; 
              SmBrep * pBrep = GetBrep() ; 

              smgfx_Erase() ;
              smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
              smgfx_SetLook(2,3, 1,0,0) ; for(di=0;di<sEdgeuses.GetSize();di++)
                { smos_sprintf(sBuff, _T("\n [%2lu]: Edgeuse:[0x%p], Edge:[0x%p]"), 
                                    di, sEdgeuses[di], sEdgeuses[di]->GetEdge()) ; smos_WriteBuffer(sBuff);
                  sEdgeuses[di]->Draw() ; sm_GraphicsLoop() ; 
                }
              sm_GraphicsLoop() ;
            }
#endif // SM_DEBUG_CODE

  // for every Edgeuse until we get back to the StartEU
  do {
       // for NULL terminated lists
       if ( pEU == NULL )
         { return; }

       // build output
       if(   (pOptAttributeId == NULL       || pEU->FindAttribute(*pOptAttributeId) != NULL) 
          && (pOptConnectedToTarget == NULL || pEU->IsConnectedTo(pOptConnectedToTarget)))
         { rEdgeuses.Add(pEU) ; }

       // next iter
       pEU = pEU->m_pCCW ;
       SM_ASSERT(pEU != NULL) ;

       // quit when we get back to the Starting Edgeuse
     } while (pEU != pStartEU) ;

} // end SmLoopuse::GetEdgeuses

/*******************************************************************//**
PURPOSE: Get the Vertexuses associated with the loopuse.

NOTES: Returns Vertexuses in CCW order.
***********************************************************************/
void SmLoopuse::GetVertexuses
 (SmTArray<SmVertexuse*> & rVertexuses,      // out: array of collected topology objects
  ULONG                  * pOptAttributeId)  // in : only include objects containing an attribute with this id
                                             //      NULL to ignore, default:[NULL]
 const
{
  // init output
  rVertexuses.ReSet() ;

  // When Loopuse_TYPE
  if (IsVertexLoopuse())
    {
      // get the only Vertexuse
      SmVertexuse * pVU = (SmVertexuse*)m_pEUorVU; SM_ASSERT(pVU != NULL);

      // set output
      if(   pOptAttributeId == NULL
         || pVU->FindAttribute(*pOptAttributeId) != NULL)  
        { rVertexuses.Add(pVU); }

      // all done
      return;

    } // end Loopuse_TYPE check

  // locals
  SmEdgeuse * pStartEU = (SmEdgeuse*)m_pEUorVU; SM_ASSERT(pStartEU != NULL);
  SmEdgeuse * pEU = pStartEU;
  
  // for every Edgeuse until we get back to the StartEU
  do {
       // for NULL terminated lists
       if ( pEU == NULL )
         { return; }

       // locals
       SmVertexuse * pVU = pEU->GetVertexuse();
      
       // build output
       if(   pOptAttributeId == NULL
          || pVU->FindAttribute(*pOptAttributeId) != NULL)  
         { rVertexuses.Add(pVU); }

       // next iter
       pEU = pEU->m_pCCW; SM_ASSERT(pEU != NULL);

       // quit when we get back to the Starting Edgeuse
     } while (pEU != pStartEU);

} // end SmLoopuse::GetVertexuses

/*******************************************************************//**
PURPOSE: Get the Vertices associated with the loopuse.

NOTES: Returns Vertices in CCW order.
***********************************************************************/
void SmLoopuse::GetVertices
 (SmTArray<SmVertex*> & rVertices,        // out: array of collected topology objects
  ULONG               * pOptAttributeId)  // in : only include objects containing an attribute with this id
                                          //      NULL to ignore, default:[NULL]
 const
{
  // init output
  rVertices.ReSet();

  // When Loopuse_TYPE
  if (IsVertexLoopuse())
    {
      // get the only Vertex
      SmVertexuse * pVU = (SmVertexuse*)m_pEUorVU; SM_ASSERT(pVU != NULL);
      SmVertex    * pV = pVU->GetVertex();
      
      // set output
      if(   pOptAttributeId == NULL
         || pV->FindAttribute(*pOptAttributeId) != NULL)  
        { rVertices.Add(pV); }

      // all done
      return;

    } // end Loopuse_TYPE check

  // locals
  SmEdgeuse * pStartEU = (SmEdgeuse*)m_pEUorVU;
  SM_ASSERT(pStartEU != NULL);
  SmEdgeuse *pEU = pStartEU;
  
  // for every Edgeuse until we get back to the StartEU
  do {
       // for NULL terminated lists
       if ( pEU == NULL )
         { return; }

       // locals 
       SmVertexuse * pVU = pEU->GetVertexuse();
       SmVertex    * pV = pVU->GetVertex();
      
       // build output
       if(   pOptAttributeId == NULL
         || pV->FindAttribute(*pOptAttributeId) != NULL)  
        { rVertices.AddUnique(pV); }

       // next iter
       pEU = pEU->m_pCCW; SM_ASSERT(pEU != NULL);

       // quit when we get back to the Starting Edgeuse
     } while (pEU != pStartEU);

} // end SmLoopuse::GetVertices

/*******************************************************************//**
PURPOSE: Get all of the Edgeuses and Vertexuses belonging to this loop.

NOTES: Returns Edgeuses and Vertexuses in CCW order.
***********************************************************************/
void SmLoopuse::GetEdgeusesAndVertexuses
 (SmTArray<SmEdgeuse*>   & rEdgeuses,        // out: array of collected topology objects
  SmTArray<SmVertexuse*> & rVertexuses,      // out: array of collected topology objects
  ULONG                  * pOptAttributeId)  // in : only include objects containing an attribute with this id
                                              //      NULL to ignore, default:[NULL]
 const
{
  // init output
  rEdgeuses.ReSet();
  rVertexuses.ReSet();

  // When Loopuse_TYPE - no Edgeuses, 1 vertexuse
  if (IsVertexLoopuse())
    {
      // get the only Vertexuse
      SmVertexuse * pVU = (SmVertexuse*)m_pEUorVU; SM_ASSERT(pVU != NULL);

      // set output
      if(   pOptAttributeId == NULL
         || pVU->FindAttribute(*pOptAttributeId) != NULL)  
        { rVertexuses.Add(pVU); }

      // all done
      return;

    } // end Loopuse_TYPE check

  // locals
  SmEdgeuse * pStartEU = (SmEdgeuse*)m_pEUorVU; SM_ASSERT(pStartEU != NULL);
  SmEdgeuse * pEU      = pStartEU;

  // for every Edgeuse until we get back to the StartEU
  do {
       // for NULL terminated lists
       if ( pEU == NULL )
         { return; }

       // locals
       SmVertexuse * pVU = pEU->GetVertexuse();

       // build edgeuse output
       if(   pOptAttributeId == NULL
          || pEU->FindAttribute(*pOptAttributeId) != NULL)  
         { rEdgeuses.Add(pEU); }

       // build vertexuse output
       if(   pOptAttributeId == NULL
          || pVU->FindAttribute(*pOptAttributeId) != NULL)  
         { rVertexuses.Add(pVU); }

       // next iter
       pEU = pEU->m_pCCW ; SM_ASSERT(pEU != NULL);

       // quit when we get back to the Starting Edgeuse
     } while (pEU != pStartEU);

} // end SmLoopuse::GetEdgeusesAndVertexuses

/*******************************************************************//**
PURPOSE: Get the shell of a loopuse.

NOTES: 
***********************************************************************/
SmShell* SmLoopuse::GetShell() const
{
  const SmFaceuse* faceusePtr = SM_CAST_PTR(SmFaceuse,m_pListOwner);
  
  return(  (faceusePtr != NULL)
         ? faceusePtr->SmFaceuse::GetShell()
         : NULL) ;
} // end SmLoopuse::GetShell

/*******************************************************************//**
PURPOSE: Walk the loop in CCW direction accumulating every edgeuse
 and vertexuse turning angle

NOTES: A valid OuterLoop will have a 360 deg UVspace TurningAng indicating 
       it is both closed and enclosing a finite set of points in UV space.

       A loop crossing a seam without the SeamEdge (an invalid condition)
       will have a 0 deg UVSpace TurningAng indicating it is closed
       but not enclosing a finite set of points in UV space.

       Healing loops of non-360 deg turning angles due to missing SeamEdges
       is the job SmSurface::SplitAtSeam().
***********************************************************************/
double SmLoopuse::GetTurningAngDeg() const
{
  // return value
  double dTurnAngDeg = 0.0 ;

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
#endif // SM_DEBUG_CODE

  // locals
  ULONG ii ;
  SmTArray<SmEdgeuse*>   sEdgeuses ;
  SmTArray<SmVertexuse*> sVertexuses ;

  // get Edgeuses and Vertexuses in CCW direction
  GetEdgeusesAndVertexuses(sEdgeuses, sVertexuses) ;

  // check state - equal sized arrays
  SM_ASSERT_MSG(sEdgeuses.GetSize() == sVertexuses.GetSize(), _T("SmLoopuse::GetTurningAngDeg() - assumption that Loop's Vertexuse and Edgeuse counts are equal is broken - needs debugging")) ; 
  
  // for every Vertexuse/Edgeuse pair - accumulate Turning AngDeg
  for(ii=0;ii<sVertexuses.GetSize();ii++)
    {
      double dVertexTurnAngDeg = sVertexuses[ii]->GetTurningAngDeg() ;
      double dEdgeTurnAndDeg   = sEdgeuses[ii]->GetTurningAngDeg() ;
      dTurnAngDeg += dVertexTurnAngDeg ;
      dTurnAngDeg += dEdgeTurnAndDeg ; 

#ifdef SM_DEBUG_CODE
  if(bDebugMe)
    {
      // Draw Loop use in space
      SmBrep      * pBrep       = GetBrep() ;

      smgfx_Erase() ; 
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep)           pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(3,4, 1,0,0) ; if(sVertexuses[ii]) sVertexuses[ii]->Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook(3,4, 0,1,0) ; if(sEdgeuses[ii])   sEdgeuses[ii]->Draw() ; sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

    
    } // end iter every Vertexuse accumulating Turning AngDeg 

#ifdef SM_DEBUG_CODE
  if(bDebugMe)
    {
      SM_DUMP_AND_ASSERT_VALID(this) ; 

      // pretty print TurningAngDegs
      ULONG di ;
      TCHAR sBuff[SM_TBLOCK_SIZE] ;
      SmTArray<double> sEUAngDeg, sVUAngDeg ;
      double dEUAngDeg, dVUAngDeg ; 
      
      // init output stream
      smos_sprintf(sBuff,_T("\n  LoopTurningAngDeg:[%16.16lf]\n  [indx]:  EdgeuseAngDeg     VertexAngDeg"), dTurnAngDeg) ;
      smos_WriteBuffer(sBuff) ;
      
      // for every Edgeuse and Vertexuse
      for(di=0;di<sEdgeuses.GetSize();di++)
        {
          dEUAngDeg = sEdgeuses[di]->GetTurningAngDeg() ; 
          dVUAngDeg = sVertexuses[di]->GetTurningAngDeg() ;

          sEUAngDeg.Add(dEUAngDeg) ;
          sEUAngDeg.Add(dVUAngDeg) ;

          // pretty print turning angles
          smos_sprintf(sBuff,_T("\n   [%3ld]: %16.16lf  %16.16lf"), di, dEUAngDeg, dVUAngDeg) ;
          smos_WriteBuffer(sBuff) ;
        } // end iter accumlating and printing TurningAngDeg

      // Draw Loop use in space
      SmBrep      * pBrep         = GetBrep() ;
      SmFaceuse   * pFaceuse      = GetFaceuse() ;
      SmEdgeuse   * p1stEdgeuse   = (SmEdgeuse*)GetEUorVU() ;
      SmVertexuse * p1stVertexuse = p1stEdgeuse->GetVertexuse() ; 

      smgfx_Erase() ; 
      smgfx_SetLook( 1, 2, 0, 0, 1 ); if(pBrep) { pBrep->Draw( TRUE ); sm_GraphicsLoop(); }
      smgfx_SetLook( 2, 3, 0, 1, 0 ); if(pFaceuse) { pFaceuse->Draw( SM_DM_CROSSHATCH ); sm_GraphicsLoop(); } // TRUE = add UVPlane graphics for loop on nearby plane
      smgfx_SetLook(3,4, 0,1,1) ; this->Draw(TRUE) ; sm_GraphicsLoop() ; // TRUE = add UVPlane graphics for loop on nearby plane
      smgfx_SetLook( 7, 8, 1, 0, 0 ); if(p1stVertexuse) { p1stVertexuse->Draw(); sm_GraphicsLoop(); }
      smgfx_SetLook( 5, 6, 1, 0, 1 ); if(p1stEdgeuse) { p1stEdgeuse->Draw(); sm_GraphicsLoop(); }
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

  // all done
  return(dTurnAngDeg) ; 

} // end SmLoopuse::GetTurningAngDeg

/*******************************************************************//**
PURPOSE: Allow the starting edgeuse of a loopuse to be
changed, this allows us to cycle the definition of the loopuse,
e.g  edgeuse order 1->2->3->4  can be changed to  2->3->4->1
by changing the starting edgeuse stored by the loopuse.
NOTES:
***********************************************************************/
SmStatus SmLoopuse::SetEdgeuse(SmEdgeuse* pEdgeuse)
{
  if(  IsEdgeLoopuse()
     && pEdgeuse != NULL
     && pEdgeuse->GetLoopuse() == this)
    {
      m_pEUorVU = pEdgeuse;
      return SM_SUCCESS;
    }

  return SM_ERR;
}  // end SmLoopuse::SetEdgeuse

/*******************************************************************//**
PURPOSE: Calculate Loopuse->edges Bounding Box.

NOTES:
***********************************************************************/
SmStatus SmLoopuse::CalculateBoundingBox
  (SmExtent3d & rBBox3d,       // out: Loopuse->edges bounding box in 3d
   SmPseudoBox *pOptPseudoBox, // out: 3d pseudo box, NULL to ignore
                               //      default:[NULL]
   SmExtent2d * pOptBBoxUV)    // out: Loopuse->edges bounding box in UV space, NULL to ignore
                               //      default:[NULL]
 const
{
  SmExtent2d    sBBoxUV ;
  SmExtent2d  * pBBoxUV = pOptBBoxUV ? pOptBBoxUV : &sBBoxUV ;
  SmPseudoBox   sPseudoBox ;
  SmPseudoBox * pPseudoBox = pOptPseudoBox ? pOptPseudoBox : &sPseudoBox ;

  // init output
  rBBox3d.Init() ;
  pBBoxUV->Init() ;
  pPseudoBox->Init() ;  // init intervals - leave basis vectors alone

  // locals
  ULONG ii ;
  SmExtent3d  sBBox3d ;
  SmExtent2d  sBBox2d ;
  SmPseudoBox sPBox ;
  SmPoint2d   sUVPoint ;
  SmTArray<SmEdge *>   sEdges ;
  SmTArray<SmVertex *> sVertices ;
  GetEdges(sEdges) ;
  GetVertices(sVertices) ;
  SmFaceuse *pFaceuse = GetFaceuse() ;
  SmFace *pFace = pFaceuse ?  pFaceuse->GetFace() : NULL ;

  // union together the tight bounding box of every edge and vertex (in case of tolerance problems)

  // for every Vertex - add Point to BBox3d and BBox2d
  for(ii=0;ii<sVertices.GetSize();ii++)
    {
      SmVertex    * pVertex    = sVertices[ii] ;
      SmVertexuse * pVertexuse = pVertex->GetVertexuseOfFace(pFace) ;

      // build 3d BBox
      rBBox3d.AddPoint3d(pVertex->GetPoint()) ;
      pPseudoBox->AddPoint3d(pVertex->GetPoint()) ;

      // when vertex has a vertexuse to this face (it should)
      if(pVertexuse)
        {
          // build 2d BBox
          pVertexuse->ComputeUVPoint(sUVPoint) ;
          pBBoxUV->AddPoint2d(sUVPoint) ;
        }
    } // end iter every LoopVertex

  // for every Edge - union Edge BBox to BBox3d and BBox2d
  for(ii=0;ii<sEdges.GetSize();ii++)
    {
      SmEdge         * pEdge    = sEdges[ii] ;
      SmEdgeuse      * pEdgeuse = pEdge->GetPrimaryEdgeuse() ;
      SmBSplineCurve * pUVCurve = NULL;
      if(pEdgeuse)
        {
          pEdgeuse->GetOrCreateUVTrimCurve(pUVCurve) ;
        }

      // Get this Edge's tight 3dbox
      SER( pEdge->CalculateBoundingBox( &sBBox3d, &sPBox) );

      // union edge->3dBBox into the output 3dBox
      rBBox3d.Union( sBBox3d, rBBox3d );
      pPseudoBox->Union( sPBox, pPseudoBox->GetBasis(), *pPseudoBox) ;

      // When edge has a UVTrimCurve
      if(pUVCurve)
        {
          // Get this Edge->UVTrimCurve's tight 2dbox
          pUVCurve->CalculateBoundingBox(pUVCurve->GetNaturalInterval(), &sBBox3d) ;
          sBBox2d.SetMinMax(sBBox3d.GetUMin(), sBBox3d.GetVMin(),
                            sBBox3d.GetUMax(), sBBox3d.GetVMax()) ;

          // union edge->2dBBox with output 2BBox
          pBBoxUV->Union(sBBox2d, *pBBoxUV) ;
        }

    } // end iter every edge

  // all done
  return SM_SUCCESS;

} // end SmLoopuse::CalculateBoundingBox

/*******************************************************************//**
PURPOSE: return TRUE when all Loopuse->edgeuses form a closed HeadToTail
         loop where all EdgeToEdge Gap3ds are within XSectTol3d(Edge,Edge)

NOTES: returns TRUE  for VertexLoopuse
       returns TRUE  for ClosedLoops with struts
       returns FALSE for WireLoops
***********************************************************************/
SmBoolean SmLoopuse::IsClosed3d           // rtn: TRUE when all EdgeEnd/EdgeEnd Gap3ds are within their associated XSectTol3d limit values
 (SmBoolean       bCheckAllGaps,   // in : TRUE = Check all gaps after finding the first open EdgeuseEnd-EdgeuseEnd gap to get true MaxGap values
                                   //      FALSE= quit after finding the 1st open EdgeuseEnd-EdgeuseEnd gap to save time.
  SmEdgeuse    ** ppOptEdgeuse,    // out: When any gaps are out-of-tol, the 1st edgeuse of the gap that exceeds its XSectTol3d value the most,
                                   //      When all gaps are in-tol, the largest gap between Edgeuse ends.
                                   //      CCW Edgeuse is the other end of gap. NULL to ignore, Default:[NULL]
  double        * pdOptMaxGap,     // out: Dist for gap being returned by ppOptEdgeuse
                                   //      NULL to ignore, Default:[NULL]
  SmXSectTol3d  * psOptXSectTol3d) // out: XSectTol3d value for gap being returned by ppOptEdgeuse. 
                  const            //      NULL to ignore, Default:[NULL]
{
  // init output
  if(ppOptEdgeuse)    { *ppOptEdgeuse    = NULL ; }
  if(pdOptMaxGap)     { *pdOptMaxGap     = 0.0 ; }
  if(psOptXSectTol3d) { *psOptXSectTol3d = 0.0 ; }

  // no work - no Edgeuses
  if (IsVertexLoopuse())
    { return TRUE ; }

  // locals
  ULONG ii ;
  SmPoint3d    sThisEndPoint[2] ;
  SmPoint3d    sNextStartPoint[2] ;
  SmXSectTol3d sXSectTol3d ;
  SmXSectTol3d sXSectTol3dSq ;
  SmEdgeuse  * pMaxGapEdgeuse           = NULL ;
  double       dMaxGapSq                = 0.0;
  SmXSectTol3d dMaxGap_XSectTol3d       = 0.0 ;
                                        
  SmEdgeuse  * pMaxOverTolEdgeuse       = NULL ;
  double       dMaxOverTol              = 0.0 ;
  SmXSectTol3d dMaxOverTol_XSectTol3d   = 0.0 ;

  // obsolete - was using face tolerances - now using XSect(Edge,Edge) tolerances
  // SmZoneTol3d          sZoneTol3d   =  (GetFaceuse() && GetFaceuse()->GetFace())
  //                                     ? SmTol::GetZoneTol3d(GetFaceuse()->GetFace())
  //                                     : SmTol::GetZoneTol3d(GetContext()) ;
  // SmZoneTol3d          sZoneTol3dSq = sZoneTol3d * sZoneTol3d ;

  SmTArray<SmEdgeuse*> sEdgeuses;
  GetEdgeuses(sEdgeuses) ;
  ULONG lNum = sEdgeuses.GetSize() ;

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  if(bDebugMe)
    {
      double dP, dN = 1/(double)(lNum) ;
      smgfx_Erase() ;
      for(ii=0;ii<lNum;ii++) { SmEdge *pE = sEdgeuses[ii]->GetEdge() ; dP=(ii+1)*dN ;
                               SmExtent1d pEInterval (pE->GetInterval());
                               smgfx_SetLook(1,4, dP,0,1-dP) ; pE->GetCurve()->DrawParams(&pEInterval) ; sm_GraphicsLoop() ;
                             }
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

  // for every Edgeuse - check for EdgeuseEnd-EdgeuseEnd gap sizes
  for(ii=0;ii<lNum;ii++)
    {
      // This Edgeuse End Point
      SmEdgeuse    * pThisEdgeuse   = sEdgeuses[ii] ;
      SmZoneTol3d    sThisZoneTol3d = SmTol::GetZoneTol3d(pThisEdgeuse) ;
      SmOrientType   eThisOrient    = pThisEdgeuse->GetOrientation() ;
      SmEdge       * pThisEdge      = pThisEdgeuse->GetEdge() ;
      SmCurve      * pThisCurve     = pThisEdge->GetCurve() ;
      SmExtent1d     sThisIvl       = pThisEdge->GetInterval() ;
      double         dThisEndParam  =   eThisOrient == SM_OT_SAME
                                         ? sThisIvl.GetMax()
                                         : sThisIvl.GetMin() ;
      pThisCurve->EvaluatePoint(dThisEndParam, sThisEndPoint[0]) ;

      // Next Edgeuse Start Point
      SmEdgeuse    * pNextEdgeuse    = sEdgeuses[(ii+1)%lNum] ;
      SmZoneTol3d    sNextZoneTol3d  = SmTol::GetZoneTol3d(pNextEdgeuse) ;
      SmOrientType   eNextOrient     = pNextEdgeuse->GetOrientation() ;
      SmEdge       * pNextEdge       = pNextEdgeuse->GetEdge() ;
      SmCurve      * pNextCurve      = pNextEdge->GetCurve() ;
      SmExtent1d     sNextIvl        = pNextEdge->GetInterval() ;
      double         dNextStartParam =   eNextOrient == SM_OT_SAME
                                       ? sNextIvl.GetMin()
                                       : sNextIvl.GetMax() ;
      pNextCurve->EvaluatePoint(dNextStartParam, sNextStartPoint[0]) ;

      // ThisEndPoint to NextStartPoint gap
      double dGap3dSq = sThisEndPoint[0].DistanceBetweenSquared(sNextStartPoint[0]) ;

      // EdgeEnd/EdgeEnd tolerance
      sXSectTol3d    = SmTol::GetXSectTol3d(sThisZoneTol3d, sNextZoneTol3d) ;
      sXSectTol3dSq  = sXSectTol3d * sXSectTol3d ;

      // When Gap is largest - save maxGap values
      if(dGap3dSq > dMaxGapSq)
        {
          pMaxGapEdgeuse     = pThisEdgeuse ;
          dMaxGapSq          = dGap3dSq ;
          dMaxGap_XSectTol3d = sXSectTol3d ;
        } // end MaxGap3d check

      // When gap is bigger than XSectTol3d(ThisEdgeuse, NextEdgeuse)
      if(dGap3dSq > sXSectTol3dSq )
        { 
          double dOverTol    = sXSectTol3d - smos_Sqrt(dGap3dSq) ;

          // when OverTol is largest - save MaxOvertTol vals
          if(dOverTol > dMaxOverTol)
            {
              pMaxOverTolEdgeuse     = pThisEdgeuse ;
              dMaxOverTol            = dOverTol ;
              dMaxOverTol_XSectTol3d = sXSectTol3d ;
            } // end MaxOverTol check

          // when asked to quit early - quit
          if(!bCheckAllGaps)
            { break; }

        } // end out of tol gap check
    } // end iter all Edgeuses

  // set output
  SmBoolean bIsClosed = pMaxOverTolEdgeuse == NULL ;
  if(pdOptMaxGap) { *pdOptMaxGap = smos_Sqrt(dMaxGapSq) ; }
  if(bIsClosed)   { if(ppOptEdgeuse)    { *ppOptEdgeuse    = pMaxGapEdgeuse ; }
                    if(pdOptMaxGap)     { *pdOptMaxGap     = smos_Sqrt(dMaxGapSq) ; }
                    if(psOptXSectTol3d) { *psOptXSectTol3d = dMaxGap_XSectTol3d ; }
                  }
  else            { if(ppOptEdgeuse)    { *ppOptEdgeuse    = pMaxOverTolEdgeuse ; }
                    if(pdOptMaxGap)     { *pdOptMaxGap     = dMaxOverTol + dMaxOverTol_XSectTol3d ; }
                    if(psOptXSectTol3d) { *psOptXSectTol3d = dMaxOverTol_XSectTol3d ; }
                  }

  // return status
  return(bIsClosed) ;

} // end SmLoopuse::IsClosed3d

/*******************************************************************//**
PURPOSE: return TRUE when Edgeuse CCW Linked list is closed

NOTES: returns TRUE  for VertexLoopuse
       returns TRUE  for ClosedLoops with struts
       returns FALSE for WireLoops
***********************************************************************/
SmBoolean SmLoopuse::IsClosedPtrs() const
{
  // no work - no Edgeuses
  if (IsVertexLoopuse())
    { return TRUE ; }

  // locals
  SmEdgeuse * pStartEU = (SmEdgeuse*)m_pEUorVU; SM_ASSERT(pStartEU != NULL) ;
  SmEdgeuse * pEU      = pStartEU ;

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ; 
  if(bDebugMe)
    {
      ULONG di ;
      TCHAR sBuff[SM_TBLOCK_SIZE] ;

      SmTArray<SmEdgeuse *> sEdgeuses ; 
      this->GetEdgeuses(sEdgeuses) ; 
      SmBrep * pBrep = GetBrep() ; 

      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(2,3, 1,0,0) ; for(di=0;di<sEdgeuses.GetSize();di++)
        { smos_sprintf(sBuff, _T("\n [%2lu]: Edgeuse:[0x%p], Edge:[0x%p]"), 
                            di, sEdgeuses[di], sEdgeuses[di]->GetEdge()) ; smos_WriteBuffer(sBuff);
          sEdgeuses[di]->Draw() ; sm_GraphicsLoop() ; 
        }
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

  // for every Edgeuse until we get back to the StartEU
  do {
       // for NULL terminated lists
       if ( pEU == NULL )
         { return FALSE; }

       // next iter
       pEU = pEU->m_pCCW ;
       SM_ASSERT(pEU != NULL) ;

       // quit when we get back to the Starting Edgeuse
     } while (pEU != pStartEU) ;

  // arrive here when CCW Linked List is closed

  // all done
  return(TRUE) ;

} // end SmLoopuse::IsClosedPtrs

/*******************************************************************//**
PURPOSE: Return True when Edgeuse is connected to given Target Topology object

NOTES: 1. the input Target cpConnectTgt may be NULL, or of type,
             SmVertex,
             SmVertexuse,
             SmEdge,
             SmEdgeuse,
             SmLoop,
             SmFace,
             SmShell,
             SmRegion
        2. returns TRUE for NULL and for any other unsupported Topology TYPE
***********************************************************************/
SmBoolean SmLoopuse::IsConnectedTo
  ( const SmTopology *cpConnectTgt )    // in : target Topology
 const
{
  // no work - no Topo
  if(cpConnectTgt == NULL) 
    { return TRUE ; }

  // switch on cpConnectTgt type
  switch(cpConnectTgt->GetType())
    {
      case SmVertexuse_TYPE: { return( ((SmVertexuse *)cpConnectTgt)->IsConnectedTo(this) ) ; 
                             } break ;
      case SmVertex_TYPE   : { return( ((SmVertex *)cpConnectTgt)->IsConnectedTo(this) ) ;
                             } break ;
      case SmEdgeuse_TYPE  : { return( ((SmEdgeuse *)cpConnectTgt)->IsConnectedTo(this) ) ;
                             } break ; 
      case SmEdge_TYPE     : { return( ((SmEdge *)cpConnectTgt)->IsConnectedTo(this) ) ; 
                             } break ;
      case SmFace_TYPE     : { return( GetFaceuse()->GetFace() == ((SmFace *)cpConnectTgt) ) ;
                             } break ;
      case SmFaceuse_TYPE  : { return( GetFaceuse() == ((SmFaceuse *)cpConnectTgt) ) ;
                             } break ;
      case SmLoop_TYPE     : { return( GetLoop() == (SmLoop *)cpConnectTgt ) ;
                             } break ;
      case SmLoopuse_TYPE  : { return( this == (SmLoopuse *)cpConnectTgt ) ;
                             } break ;
      case SmShell_TYPE    : { return( GetShell() == (SmShell *)cpConnectTgt) ;
                             } break ;
      case SmRegion_TYPE   : { return( GetShell() && GetShell()->GetRegion() == (SmRegion *)cpConnectTgt ) ;
                             } break ;
      default: break ;

    } // end switch on type

  // arrive here when cpConnectTgt is an unsupported type - return TRUE
  return(TRUE) ;

} // end SmLoopuse::IsConnectedTo

/*******************************************************************//**
PURPOSE: Return FALSE when Edgeuse is on Surface boundary and edgeuse side
bounding 'into' the face is pointing outside the SurfaceUVDomain.

NOTES: Ths predicate tests for an illegal database configuration.
       A heal function will have to be built that fixes this condition.
       rtn: TRUE = OK use of edgeuse  - its bounding a region in its Face->Surface.
            FALSE= bad use of edgeuse - its bounding a region off its Face->Surface.

       returns TRUE (OK) for Edgeuses not yet connected to Faces.
***********************************************************************/
SmBoolean SmLoopuse::IsLoopuseSideInSurface
 (SmBoolean * pOptPeriodicU,         // in : TRUE=Surface U Periodic, FALSE=Not, NULL to ignore, default:[NULL]
  SmBoolean * pOptPeriodicV,         // in : TRUE=Surface U Periodic, FALSE=Not, NULL to ignore, default:[NULL]
  SmBoolean * pOptOnNaturalBoundary, // in : TRUE=at least 1 edgeuse is on SurfNatBndry, FALSE=Not, NULL to ignore, default:[NULL]
  ULONG     * pOptEdgeuseIndx)       // out: When FALSE is returned, optional Index value 
                                     //        of 1st Edgeuse that fails the Side test, NULL to ignore, default:[NULL]
 const 
{  
  // init return value
  SmBoolean bRtn = TRUE ;
  if(pOptOnNaturalBoundary) { *pOptOnNaturalBoundary = FALSE ; }
  if(pOptEdgeuseIndx)       { *pOptEdgeuseIndx = 0 ; }

  // locals
  ULONG ii ;
  ULONG lSmpCount = 6 ; 
  SmBoolean bThisRtn = TRUE ;
  SmTArray<SmEdgeuse*> sEdgeuses ;
  GetEdgeuses(sEdgeuses) ;

  // for every edgeuse - check Bounding Edge Side status
  for(ii=0;ii<sEdgeuses.GetSize();ii++)
    {
      SmEdgeuse * pEdgeuse = sEdgeuses[ii] ;

      // check and remember Edgeuse Side status
      SmBoolean bThisOnBoundary ;
      bThisRtn = pEdgeuse->IsEdgeuseSideInSurface(lSmpCount, pOptPeriodicU, pOptPeriodicV, &bThisOnBoundary) ;
      if(bThisOnBoundary && pOptOnNaturalBoundary) { *pOptOnNaturalBoundary = TRUE ; }
      bRtn &= bThisRtn ;

      // inform the public
      if(FALSE == bThisRtn)
        {
          if(pOptEdgeuseIndx) { *pOptEdgeuseIndx = ii ; }
          break ; 
        }

    } // end iter every edgeuse

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;

  if(bDebugMe)
    {
      ULONG di ; 
      SmBrep    * pBrep    = GetBrep() ;
      SmFace    * pFace    = GetFaceuse()->GetFace() ; 
      SmSurface * pSurface = pFace ? pFace->GetSurface() : NULL ; 

      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook( 2, 3, 1, 0, 0 ); 
      for(di = 0; di < sEdgeuses.GetSize(); di++)
      {
        if(sEdgeuses[di]) { sEdgeuses[di]->GetEdge()->DrawParams(); sm_GraphicsLoop(); }
        if(sEdgeuses[di]) { sEdgeuses[di]->Draw(); sm_GraphicsLoop(); }
      }
      smgfx_SetLook( 1, 2, 0, 1, 1 ); if(pSurface) { pSurface->DrawUV(); sm_GraphicsLoop(); }
      smgfx_SetLook(4,5, 1,0,1) ; this->Draw() ; sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE  // all done
  return(bRtn) ; 

} // end SmEdgeuse::IsLoopuseSideInSurface

/*******************************************************************//**
PURPOSE: return TRUE when any Loop->VertexSector spans a missing seam

NOTES: Checks the vertex sectors on all the loop->Vertices to see if any
  spans a seam boundary that is not marked with a seam edge.  The method only
  checks the Vertexuses at the loop vertices.  So when fixing a loop that
  crosses a seam boundary not marked by a seam edge, first split all the
  loop->edges at every loopEdge/SeamCurve intersection to ensure that this
  won't miss a loop/missingSeamEdge problem because the loop vertex needed on
  the seam boundary is missing.

  Loops that span Seams have to be broken up by inserting the 
  missing SeamEdges. See SmFace::SplitAtSeam()
***********************************************************************/
SmBoolean SmLoopuse::IsMissingSeamAtVertices() const
{ 
  // init output
  SmBoolean bRtn = FALSE ;

  // locals
  SmSurface * pSurface = (GetFaceuse() && GetFaceuse()->GetFace()) ? GetFaceuse()->GetFace()->GetSurface() : NULL ;

  // low work - no surface or surface not closed
  if(   pSurface == NULL
     || (   FALSE == pSurface->IsClosed(pSurface->GetNaturalUVDomain(), SM_SP_U)
         && FALSE == pSurface->IsClosed(pSurface->GetNaturalUVDomain(), SM_SP_V)))
    { return(bRtn) ; }

  // locals
  ULONG ii ;
  // double    sUVSectorAngDeg ;
  // SmPoint2d sUVSectorUV, sUVSectorBegTan, sUVSectorEndTan ;

  SM_PTR_ARRAY(sVertexuses, SmVertexuse, 32) ;
  GetVertexuses( sVertexuses );

  // for every Vertexuse
  for(ii=0;ii<sVertexuses.GetSize();ii++)
    {
      SmVertexuse * pVertexuse = sVertexuses[ii] ;

      // skip NULL Vertexuses - should never happen
      if(pVertexuse == NULL)
        { continue ; }

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
      if(bDebugMe)
        {
          SmBrep *pBrep = pVertexuse->GetBrep() ;

          smgfx_Erase() ;
          smgfx_SetLook(1,2, 0,0,1) ; pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
          smgfx_SetLook(3,4, 0,1,0) ; pVertexuse->Draw() ; sm_GraphicsLoop() ;
          sm_GraphicsLoop() ;
        }
#endif // SM_DEBUG_CODE

      // sector properties
      SmUVSectorIO sUVSectorIO ;
      pVertexuse->ComputeUVSector(sUVSectorIO,   // out: Optional Sector Properties.  NULL to ignore, default:[NULL]
                                  TRUE) ;        // in : TRUE  = Use LocalSolve with guesses from UVTrimCurve
                                                 //      FALSE = don't make UVTrimCurve, if missing use GlobalSolve, default:[TRUE]

      // check for missing Seam
      if(sUVSectorIO.m_bSectorMissingSeamU || sUVSectorIO.m_bSectorMissingSeamV)
        {
          bRtn = TRUE ;
          break ;
        }
    } // end iter every Vertexuse

  // all done
  return(bRtn) ;

} // end SmLoopuse::IsMissingSeamAtVertices()

/*******************************************************************//**
PURPOSE:  add loop->edges or loop->vertex graphics to new or open
             displayList added to global displayList array.

NOTES: The caller must free the returned pOptOutPlane after asking
       for it to be built with
         bDrawUVPlane == TRUE and
         pOptOutPlane != NULL.
***********************************************************************/
SmDisplayList * SmLoopuse::Draw
 (ULONG           bVUAndEUDraw,       // in : oneof 1=Draw VUs, 2=Draw EUs, 3=Draw VUs and EUs, default:[3]
  SmBoolean       bDrawUVPlane,    // in : TRUE = Display UVTrimCurves on a UVplane icon near the surface, FALSE = don't
                                   //      default:[FALSE]
  SmPlane      ** pOptOutPlane,    // out: Set to Plane used for 2d graphics when bDrawUVPlane == TRUE,
                                   //      NULL to ignore. default:[NULL],
                                   //      caller must delete this returned object
  SmBoolean       bDrawUVTrimCurves,  // in : TRUE = Draw UVTrimCurves when present, FALSE=don't
                                      //        note: when UVTrimCurves are good - they draw on top of the edges and aren't seen
                                      //              when UVTrimCurves are bad - they vary widely from the edges and indicate debugging is needed
                                      //      default:[TRUE]
  SmGfxArraySet * pOptGfxSet)      // i/o: used for OpenGL ES 2.0. When given, accumulate GfxVertexArrays not GL calls.
                                   //      NULL to ignore. default:[NULL]
 const
{
  SmDisplayList *pRtn = NULL ;
  if(pOptOutPlane) { *pOptOutPlane = NULL ; }

#ifdef SM_GFX_CODE
  //const SmDisplayParameters &rDisp = smgfx_RefGlobalDisplayParameters() ;

  // start new displayList (unless one is already open)
  SmVector3d sColor = smgfx_GetOutputColor(pOptGfxSet) ;
  double dLineWidth = smgfx_GetOutputLineWidth(pOptGfxSet) ;
  smgfx_Open(smgfx_GetRuleColor(this),NULL,NULL,FALSE,pOptGfxSet);

  // output vertexuseType graphics
  SmTArray<SmEdgeuse*> sEdgeuses;
  if (IsVertexLoopuse())
    {
      SmVertexuse *pVU = (SmVertexuse*)m_pEUorVU;
      SmVertex    *pV  = pVU->GetVertex();
      pV->OutputGraphics(pOptGfxSet);
    }
  else // output edgeuseType graphics
    {
      GetEdgeuses(sEdgeuses);

      // when asked - Draw UVDomain Icon
      SmContext     sContext ;
      SmPlane     * pUVPlane  = NULL ;
      SmObjDelete   sCleanPlane(NULL) ;
      SmFaceuse   * pFaceuse  = GetFaceuse() ;
      SmFace      * pFace     = pFaceuse ? pFaceuse->GetFace() : NULL ;
      SmSurface   * pSurface  = pFace ? pFace->GetSurface() : NULL ;

      if(bDrawUVPlane && pSurface)
        {
          SmExtent3d s3dBox, sBox ;

          // Size the UVTrimCurves
          for (ULONG j=0; j<sEdgeuses.GetSize(); j++)
            {
              SmEdgeuse *pEU = (SmEdgeuse*)sEdgeuses[j];
              SmBSplineCurve * pUVCurve = NULL ;
              pEU->GetOrCreateUVTrimCurve(pUVCurve) ;

              if(pUVCurve)
                {
                  pUVCurve->CalculateBoundingBox(pUVCurve->GetNaturalInterval(), &sBox) ;
                  if(j==0) s3dBox = sBox ;
                  else     s3dBox.Union(sBox, s3dBox) ;
                }
            } // end iter every edgeuse
          SmExtent2d sUVDomain = pSurface->GetNaturalUVDomain() ;
          SmExtent2d sUVBox(s3dBox) ;
          sUVBox.Intersect(sUVDomain, sUVBox) ;

          // Draw the UVPlane Icon positioned and sized to the UVTrimCurve boundary
          pSurface->DrawUVDomain(NULL,&sUVBox,TRUE,TRUE,FALSE,1.0,&sContext,&pUVPlane,pOptGfxSet) ;
          if(pOptOutPlane) { *pOptOutPlane = pUVPlane ;
                             pUVPlane->SetContext(NULL) ;
                           }
          else             { sCleanPlane.SetObj(pUVPlane) ;
                           }
        }

      if(bDrawUVPlane && pSurface && dLineWidth <= 2) smgfx_SetLineWidth(3, pOptGfxSet) ;

      // draw the edgeuses (and UVTrimCurves)
      for (ULONG j=0; j<sEdgeuses.GetSize(); j++)
        {
          SmEdgeuse   * pEU = (SmEdgeuse*)sEdgeuses[j];
          SmVertexuse * pVU = pEU ? pEU->GetVertexuse() : NULL ;

          // when asked - output UVTrimCurves nearby
          if(bDrawUVPlane)
            {
              smgfx_ChangeColor(j!=0, pOptGfxSet) ;

              SmBSplineCurve * pUVCurve = NULL ;
              pEU->GetOrCreateUVTrimCurve(pUVCurve) ;
              if(pUVCurve)
                {
                  //      smgfx_OutputColor(smgfx_GetColor()) ;
                  SmCrvOnSurf sCrvOnSurf(*pUVCurve, *pUVPlane) ;
                  sCrvOnSurf.Draw(NULL, FALSE, pUVPlane, pOptGfxSet) ;
                }
            } // end asked to DrawUVPlane check

          // pEU->GetEdge()->OutputGraphics(rDisp);
          if(bVUAndEUDraw & 2) pEU->Draw(1.0, bDrawUVTrimCurves, pOptGfxSet) ;    // FALSE = don't draw UVTrimCurves

          // pVU->ComputeUVSector(), then Draw SectorGraphics
          if(bVUAndEUDraw & 1) pVU->Draw() ; 

        } // end iter every edgeuse
    } // output edgeuseType graphics branch

// OBSOLETE
//            ULONG ii ;
//            SmPoint3d   sOrigin(0,0,0) ;
//            SmVector3d  sNorm(0,0,1) ;
//            SmPlane     sPlane(sOrigin, sNorm) ;
//            SmExtent2d  sBBoxUV ;
//            SmBoolean   bSeam[4]  = { FALSE, FALSE, FALSE, FALSE } ; // UMin, UMax, VMin, VMax
//            SmBoolean   bSing[4]  = { FALSE, FALSE, FALSE, FALSE } ; // UMin, UMax, VMin, VMax
//            SmExtent2d  sUVDomain ;
//
//            // set up the Plane surface when asked to Draw UVPlane
//            if(bDrawUVPlane)
//              {
//                SmFaceuse * pFaceuse  = GetFaceuse() ;
//                SmFace    * pFace     = pFaceuse ? pFaceuse->GetFace() : NULL ;
//                SmSurface * pSurface  = pFace ? pFace->GetSurface() : NULL ;
//
//                if(pSurface) // classify the surface boundary seams and singularities
//                  {
//                    sUVDomain = pSurface->GetNaturalUVDomain() ;
//                    bSeam[0]  = bSeam[1] = pSurface->IsClosed(sUVDomain, SM_SP_U) ;
//                    bSeam[2]  = bSeam[3] = pSurface->IsClosed(sUVDomain, SM_SP_V) ;
//                    pSurface->FindSingularities(bSing[0], bSing[1], bSing[2], bSing[3]) ;
//                  }
//
//                SmExtent3d  sBBox3d ;
//                SmPseudoBox sPseudoBox ;
//
//                // Get the loop's bounding boxes
//                CalculateBoundingBox(sBBox3d, &sPseudoBox, &sBBoxUV) ;
//
//                // orient the plane for drawing UVPlane graphics to the pseudo box
//                double dT ;
//                ULONG sIndex[3] = {0, 1, 2} ;
//                SmPoint3d sSizes = sPseudoBox.GetIntervalSizes() ;
//
//                // order the PseudoBox intervals from longest to shortest
//                if(sSizes[0] < sSizes[1]) { dT = sSizes[0] ; sSizes[0] = sSizes[1] ; sSizes[1] = dT ;
//                                            dT = sIndex[0] ; sIndex[0] = sIndex[1] ; sIndex[1] = dT ;
//                                          }
//                if(sSizes[1] < sSizes[2]) { dT = sSizes[1] ; sSizes[1] = sSizes[2] ; sSizes[2] = dT ;
//                                            dT = sIndex[1] ; sIndex[1] = sIndex[2] ; sIndex[2] = dT ;
//                                          }
//                if(sSizes[0] < sSizes[1]) { dT = sSizes[0] ; sSizes[0] = sSizes[1] ; sSizes[1] = dT ;
//                                            dT = sIndex[0] ; sIndex[0] = sIndex[1] ; sIndex[1] = dT ;
//                                          }
//                // orient the plane with the longest pseudo box interval
//                SmVector3d sX = sPseudoBox.GetBasis(sIndex[0]) ;
//                SmVector3d sY = sPseudoBox.GetBasis(sIndex[1]) ;
//
//                // orthogonalize sY and sZ
//                sY = sX * sY * sX ;
//                sY.Unitize() ;
//
//                SmVector3d sZ = sX * sY ;
//                sZ.Unitize() ;
//
//                // Set Scale = 1 and place all the plane size in the UVDomain
//                SmVector2d sUVScale(sBBoxUV.XLength() > SM_EFF_ZERO ? sSizes[0]/sBBoxUV.XLength() : sSizes[0],
//                                    sBBoxUV.YLength() > SM_EFF_ZERO ? sSizes[1]/sBBoxUV.YLength() : sSizes[1]) ;
//
//                // offset the plane origin from the min corner in the shortest interval direction
//                sOrigin =   sPseudoBox.Evaluate(0,0,0)
//                          - sX * sUVScale.x * sBBoxUV.GetUMin()
//                          - sY * sUVScale.y * sBBoxUV.GetVMin()
//                          + sZ * sSizes[2] * 1.3 ;
//
//                // define the plane
//                SmPlane sThisPlane(sOrigin, sX, sY, sUVScale, sBBoxUV) ;
//
//                // save the plane
//                SmAxis2Placement sPosition( sOrigin, sX, sY ) ;
//                sPlane.SetUVScale(sUVScale) ;
//                sPlane.SetPosition(sPosition) ;
//                sPlane.UpdateAnalyticalDomain(sBBoxUV) ;
//                sPlane.MakeNurb() ;
//              } // end asked to set up DrawUVPlane check
//
//            // draw the edgeuses (and UVTrimCurves)
//            GetEdgeuses(sEdgeuses);
//            for (ULONG j=0; j<sEdgeuses.GetSize(); j++)
//              {
//                SmEdgeuse *pEU = (SmEdgeuse*)sEdgeuses[j];
//
//                // when asked - output UVTrimCurves nearby
//                if(bDrawUVPlane)
//                  {
//                    smgfx_ChangeColor(j != 0) ;
//
//                    SmBSplineCurve * pUVCurve = NULL ;
//                    pEU->GetOrCreateUVTrimCurve(pUVCurve) ;
//                    if(pUVCurve)
//                      {
//                        //      smgfx_OutputColor(smgfx_GetColor()) ;
//                        SmCrvOnSurf sCrvOnSurf(*pUVCurve, sPlane) ;
//                        sCrvOnSurf.Draw() ;
//                      }
//                  } // end asked to DrawUVPlane check
//
//                // pEU->GetEdge()->OutputGraphics(rDisp);
//                pEU->Draw() ;
//
//              } // end iter every edgeuse
//
//            // when asked draw the UVPlane icon (4 clor coded boundary curves)
//            if(bDrawUVPlane)
//              {
//                SmPoint3d   sPnt1, sPnt2 ;
//
//                smgfx_OutputLineWidth(1) ;
//
//                // render the 4 sides - colored for seams and singularities
//                for(ii=0;ii<4;ii++)
//                  {
//                    switch(ii)
//                      { case 0: // UMIN
//                                sPlane.EvaluatePointFast(sUVDomain.Evaluate(0,0),  sPnt1) ;
//                                sPlane.EvaluatePointFast(sUVDomain.Evaluate(0,1),  sPnt2) ;
//                                break ;
//                        case 1: // UMAX
//                                sPlane.EvaluatePointFast(sUVDomain.Evaluate(1,0),  sPnt1) ;
//                                sPlane.EvaluatePointFast(sUVDomain.Evaluate(1,1),  sPnt2) ;
//                                break ;
//                        case 2: // VMIN
//                                sPlane.EvaluatePointFast(sUVDomain.Evaluate(0,0),  sPnt1) ;
//                                sPlane.EvaluatePointFast(sUVDomain.Evaluate(1,0),  sPnt2) ;
//                                break ;
//                        case 3: // VMAX
//                                sPlane.EvaluatePointFast(sUVDomain.Evaluate(0,1),  sPnt1) ;
//                                sPlane.EvaluatePointFast(sUVDomain.Evaluate(1,1),  sPnt2) ;
//                                break ;
//                      }
//
//                    if(bSeam[ii])      smgfx_OutputColor(.3,.6,.6) ;
//                    else if(bSing[ii]) smgfx_OutputColor(.6,.0,.3) ;
//                    else               smgfx_OutputColor(.4,.4,.4) ;
//                    smgfx_OutputLine(sPnt1.x, sPnt1.y, sPnt1.z, sPnt2.x, sPnt2.y, sPnt2.z) ;
//
//                  } // end render 4 sides of Surface Natural Boundary
//              } // end if asked bDrawUVPlane check
//          } // output edgeuseType graphics branch

  // end displayList
  smgfx_SetColor(sColor, pOptGfxSet) ;
  smgfx_OutputLineWidth(dLineWidth, pOptGfxSet) ;
  pRtn = smgfx_Close(pOptGfxSet) ;

#else
  SM_REF4(bVUAndEUDraw, bDrawUVPlane, bDrawUVTrimCurves, pOptGfxSet);
#endif // SM_GFX_CODE
  return(pRtn) ;

} // end SmLoopuse::Draw

/*******************************************************************//**
PURPOSE: AssertValid() Test reporting static labels
***********************************************************************/
SmAssertReportLabel sAssertLoopuse_list[] =
{
 /*  0 */ {SM_AT_POINTER,     _T("MisMatched Context ptrs"),    _T("Loopuse and member m_pL contexts differ") },
 /*  1 */ {SM_AT_POINTER,     _T("MisMatched Context ptrs"),    _T("Loopuse and member m_pLUMate contexts differ") },
 /*  2 */ {SM_AT_POINTER,     _T("MisMatched Context ptrs"),    _T("Loopuse and member m_pEUorVU contexts differ") },
 /*  3 */ {SM_AT_TOPOLOGICAL, _T("No Edgeuse or Vertexuse"),    _T("When Loopuse is used in a Faceuse, it must have a Vertexuse or Edgeuse") },
 /*  4 */ /* obsolete */ {SM_AT_TOPOLOGICAL, _T("Bad Missing Seam"),           _T("a Loop->Vertexuse->Sector contains a FaceSrf->Seam not marked by SeamEdge") },
 /*  5 */ {SM_AT_GEOMETRIC,   _T("Edgeuses out of tolerance"),  _T( "Gaps between Loopuse Edgeuses are greater than XSectTol3d" ) }
} ;

/*******************************************************************//**
PURPOSE:

RETURNS ---  TRUE  = OK
             FALSE = Problem
***********************************************************************/
SmBoolean SmLoopuse::AssertValid
 (SmAssertArray    * pAList,        // i/o: Accumulating list of failed Asserts, NULL to ignore, default:[NULL]
  SmAssertTestLevel  eTestLevel,    // in : SM_LEVEL_0=fewest tests, SM_LEVEL_1, SM_LEVEL_2=all tests,
                                    //      SM_LEVEL_GIVEN = run tests in order requested in pTestRequests
                                    //      default:[SM_LEVEL_0]
  SmAssertWalking    eWalkTree,     // NotUsed: in : SM_WALK = run AssertValid on any topology graph descendants, SM_NO_WALK=don't, default:[SM_WALK]
  SmTArray<ULONG>  * pTestRequests) // in : when eTestLevel == SM_LEVEL_GIVEN, run these tests in this order, default:[NULL]
 const
{
  TCHAR sBuff[SM_TBLOCK_SIZE] ;
  SM_REF1(eWalkTree) ; 
  // init rtn value
  SmBoolean bRtn = TRUE;

  // call the base class AssertValid
  bRtn &= (  (eTestLevel != SM_LEVEL_GIVEN)
           ? SmTopology::AssertValid(pAList, eTestLevel, SM_NO_WALK, pTestRequests)
           : TRUE ) ;

  /* 0, 1, 2 */ // SmLoopuse and the objects it attaches to need to share common contexts
  if(m_pL)      { bRtn &= SM_ASSERT_BOOLEAN_REPORT(0, SM_LEVEL_0, (GetContext() == m_pL->GetContext()), _T("") ) ; }
  if(m_pLUMate) { bRtn &= SM_ASSERT_BOOLEAN_REPORT(1, SM_LEVEL_0, (GetContext() == m_pLUMate->GetContext()), _T("") ) ; }
  if(m_pEUorVU) { bRtn &= SM_ASSERT_BOOLEAN_REPORT(2, SM_LEVEL_0, (GetContext() == m_pEUorVU->GetContext()), _T("") ) ; }

  /*  3 */ // When Loopuse is used in a Faceuse, it must have a Vertexuse or Edgeuse
  bRtn &= SM_ASSERT_BOOLEAN_REPORT(3, SM_LEVEL_0, (m_pListOwner == NULL || m_pEUorVU != NULL), _T("") ) ;

  /*  4 */ /* obsolete */ // missing-seam at loop vertices: unique owner is Vertexuse 5 (IsSectorMissingSeam).
                          // IsMissingSeamAtVertices() remains for SmBrep::MakeEdgeInFaceWithVU.

  /* 5 */ // a Loopuse should form a closed loop in 3d.
          // Unique owner is the primary loopuse: the mate walks the same edges backwards.
  if( m_pL != NULL && m_pL->GetLoopuse() == this )
    {
      SmEdgeuse    * pEdgeuse = NULL ; 
      double         dMaxGap = 0.0 ;
      SmXSectTol3d   sXSectTol3d = 0.0 ;
      SmBoolean      bIsClosed3d = IsClosed3d( TRUE, &pEdgeuse, &dMaxGap, &sXSectTol3d) ;
      if(bIsClosed3d == TRUE) { smos_sprintf(sBuff, _T("%s"), _T(""))  ; }  // no error to report
      else                    { smos_sprintf(sBuff, _T("SmLoopuse::AssertValid - found loop with out of tol of Gap/XSectTol3d:[%16.16lf/%16.16lf] starting on Edge:[0x%p] "),
                                             dMaxGap,
                                             sXSectTol3d,
                                             pEdgeuse->GetEdge()) ;
                              }
      bRtn &= SM_ASSERT_VALUE_REPORT(5, SM_LEVEL_0, bIsClosed3d, sXSectTol3d, dMaxGap, sBuff) ;
    }

 // all done
  // SM_ASSERT(bRtn) ;
  return(bRtn) ;

} // end SmLoopuse::AssertValid

// obsolete
// /*******************************************************************//**
// PURPOSE: Fix Assert Report failures reported by an AssertValid call
// 
// RETURNS ---  TRUE  = OK
//              FALSE = Problem
// ***********************************************************************/
// SmBoolean SmLoopuse::AssertHeal
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
//       return ( SmTopology::AssertHeal(rAReport, pAList) ) ;
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
//                rAReport.m_pHealMessage = _T("SmLoopuse::AssertHeal fix not yet supported") ;
// 
//     }
// 
//   // all done
//   return(bRtn) ;
// 
// } // end SmLoopuse::AssertHeal
// end obsolete

/*******************************************************************//**
PURPOSE:
NOTES:
***********************************************************************/
SmBoolean SmLoopuse::IsKindOf( SM_TYPE t ) const
{
  return ((SmLoopuse_TYPE == t) ? TRUE : SmTopology::IsKindOf( (t) ));
}

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
void SmLoopuse::Dump
  (void)
 const
{
  // pass the call along - abbreviate output (skip long UVTrimCurve dumps)
  Dump(TRUE) ;
} // end SmLoopuse::Dump

/*******************************************************************//**
PURPOSE: Pretty Print Loopuse

NOTES: Prints: header:{ object:[id], Orientation:[Same/Opposite], Loop:[id], Faceuse:[id], Type:[Edgeuse/Vertexuse]
               BaseClass SmTopology::Dump() 
               if(SmEdgeuse_TYPE):  EdgeuseCnt, ForEach{ [cnt] Edgeuse:[id], 
                                                               StartVtx:[id], Edgeuse_StartUV:[u v],
                                                               Vertexuse_StartUV:[u v] 
                                                               StartUVGap:[d], 
                                                               EndVtx:[id], Edgeuse_EndUV:[u v] 
***********************************************************************/
void SmLoopuse::Dump
( SmBoolean bAbbrev ) // in : TRUE=abbreviate, FALSE=Add, default:[TRUE]
const
{
  // locals
  TCHAR sBuff[SM_TBLOCK_SIZE], sBuffForFile[SM_TBLOCK_SIZE];

  // label
  smos_WriteBuffer( _T( "\nBegin SmLoopuse::Dump()" ) );

  // header
  smos_sprintf( sBuff, _T( "\n  %s SmLoopuse:[0x%p], " ),
    (m_tLoopuseType == SmEdgeuse_TYPE ? _T( "Edgeuse_TYPE" ) : _T( "Vertexuse_TYPE" )), this );

  smos_sprintf( sBuffForFile, _T( "\n  %s SmLoopuse:[%s], " ),
    (m_tLoopuseType == SmEdgeuse_TYPE ? _T( "Edgeuse_TYPE" ) : _T( "Vertexuse_TYPE" )), _T( "NotNULL" ) );

  smos_WriteBuffer( sBuff, sBuffForFile );

  // orientation
  smos_sprintf( sBuff, _T( " Orientation:[%s], " ), (m_eOrientation == SM_OT_SAME) ? _T( "SAME" )
                                             : (m_eOrientation == SM_OT_OPPOSITE) ? _T( "OPPOSITE" )
                                             : _T( "UnKnown" ) );
  smos_WriteBuffer( sBuff );

  // owner Loop, Faceuse, and Face
  SmFaceuse *pFU = this->GetFaceuse();
  SmFace    *pF  = ( pFU != NULL ) ? pFU->GetFace() : NULL;
  smos_sprintf( sBuff, _T( "Loop:[0x%p], Faceuse:[0x%p], Face:[0x%p], " ), m_pL, pFU, pF );
  smos_sprintf( sBuffForFile, _T( "Loop:[%s], Faceuse:[%s], Face:[%s], " ),
              m_pL ? _T( "NotNULL" ) : _T( "NULL" ),
              pFU  ? _T( "NotNULL" ) : _T( "NULL" ),
              pF   ? _T( "NotNULL" ) : _T( "NULL" ) );
  smos_WriteBuffer( sBuff, sBuffForFile );

  // Edgeuse or Vertexuse object
  smos_sprintf( sBuff, _T( "m_pEUorVU:[0x%p]" ), m_pEUorVU );
  smos_sprintf( sBuffForFile, _T( "m_pEUorVU:[%s]" ), m_pEUorVU ? _T( "NotNULL" ) : _T( "NULL" ) );
  smos_WriteBuffer( sBuff, sBuffForFile );

  // base class dump
  SmTopology::Dump();

  // When Loopuse is an EdgeLoopuse
  if(IsEdgeLoopuse())
  {
    ULONG ii;
    SmTArray< SmEdgeuse* > sEdgeuses;
    GetEdgeuses( sEdgeuses );
    ULONG           lNum = sEdgeuses.GetSize();
    double          dPrevStart_UVGap, dVertEdge_MaxUVGap; // sVUSectorAngDeg, double dVU_PtSector_UVGap ;
    SmPoint2d       sVUPointUV, sVUSectorUV, sVUSectorBegTan, sVUSectorEndTan;
    SmPoint3d       sEUStartUV, sEUEndUV, sEUPrevEndUV, sEUNextStartUV;
    SmVertexuse   * pEndVertexuse;
    SmVertex      * pStartVertex, *pEndVertex = NULL, *pPrevEndVertex = NULL;
    SmUVSectorIO    sUVSectorIO;

    // prepare for iter - get Last:[sEUEndUV, StartVtx, EndVtx]
    if(lNum > 0)
    {
      sEdgeuses[lNum - 1]->NormalizedEvaluate( 1.0, TRUE, sEUEndUV, NULL, TRUE );  // TRUE = UV Eval, FALSE = 3d Eval
      pStartVertex = sEdgeuses[lNum - 1]->GetVertexuse() ? sEdgeuses[lNum - 1]->GetVertexuse()->GetVertex() : NULL;
      pEndVertex = sEdgeuses[lNum - 1]->GetEdge() ? sEdgeuses[lNum - 1]->GetEdge()->GetOtherVertex( pStartVertex ) : NULL;
    }

    // list label
    smos_sprintf( sBuff, _T( "\nEdgeuseCnt:[%ld]" ), sEdgeuses.GetSize() );
    smos_WriteBuffer( sBuff );

    // for every edgeuse
    for(ii = 0; ii < lNum; ii++)
    {
      SmEdgeuse * pEdgeuse = sEdgeuses[ii];
      SmEdgeuse * pNextEdgeuse = sEdgeuses[(ii + 1) % lNum];
      // SmEdgeuse * pPrevEdgeuse = sEdgeuses[(ii+lNum-1)%lNum] ;

      // report and skip NULL edgeuses
      if(pEdgeuse == NULL)
      {
        smos_sprintf( sBuff, _T( "\n[%3ld] Edgeuse  :[NULL]" ), ii );
        smos_WriteBuffer( sBuff );
        continue;
      }

      // from previous iteration
      sEUPrevEndUV = sEUEndUV;
      pPrevEndVertex = pEndVertex;

      // locals
      pStartVertex = pEdgeuse->GetVertexuse() ? pEdgeuse->GetVertexuse()->GetVertex() : NULL;
      pEndVertexuse = pNextEdgeuse->GetVertexuse();
      pEndVertex = pEndVertexuse ? pEndVertexuse->GetVertex() : NULL;

      pEdgeuse->NormalizedEvaluate( 0.0, TRUE, sEUStartUV, NULL, TRUE );         // TRUE = UV Eval, FALSE = 3d Eval
      pEdgeuse->NormalizedEvaluate( 1.0, TRUE, sEUEndUV, NULL, TRUE );           // TRUE = UV Eval, FALSE = 3d Eval
      pNextEdgeuse->NormalizedEvaluate( 0.0, TRUE, sEUNextStartUV, NULL, TRUE ); // TRUE = UV Eval, FALSE = 3d Eval

      // sector properties
      // sVUPointUV and sVUSectorUV are now computed by the same internal call - they are always the same now.
      pEndVertexuse->ComputeUVPoint( sVUPointUV );
      pEndVertexuse->ComputeUVSector(sUVSectorIO,      // out: Optional Sector Properties.  NULL to ignore, default:[NULL]
                                     TRUE) ;           // in : TRUE  = Use LocalSolve with guesses from UVTrimCurve
                                                       //      FALSE = don't make UVTrimCurve, if missing use GlobalSolve, default:[TRUE]
       sVUSectorUV     = sUVSectorIO.m_sBegEdgeUVPoint ;
       sVUSectorBegTan = sUVSectorIO.m_sBegEdgeUVTan ;
       sVUSectorEndTan = sUVSectorIO.m_sEndEdgeUVTan ;
       // sVUSectorAngDeg = sUVSectorIO.m_dSectorAngDeg ;   
                                     
      // Sector UV gaps - Edge/Edge and Vert/Edge 
      dPrevStart_UVGap = (sEUStartUV - sEUPrevEndUV).Length();
      dVertEdge_MaxUVGap = smos_Max( (sVUPointUV - sEUEndUV).Length(),
        (sVUPointUV - sEUNextStartUV).Length() );
      // dVU_PtSector_UVGap = (sVUPointUV-sVUSectorUV).Length() ;

#ifdef SM_DEBUG_CODE
      if((sVUPointUV - sVUSectorUV).Length() >= SM_EFF_ZERO / 10.0)
      {
        SM_ASSERT_MSG( (sVUPointUV - sVUSectorUV).Length() < SM_EFF_ZERO / 10.0,
                      _T( "SmLoopuse::Dump() - Found a Vertexuse whose ComputeUVPoint differs from ComputeUVSector point" ) );
      }
#endif // SM_DEBUG_CODE

      // Edgeuse and Edgeuse->StartVertex report line
      {
        smos_sprintf( sBuff, _T( "\n[%3ld] Edgeuse  :[0x%p] " ), ii, pEdgeuse );
        smos_sprintf( sBuffForFile, _T( "\n[%3ld] Edgeuse  :[%s] " ), ii, pEdgeuse ? _T( "NotNULL" ) : _T( "NULL" ) );
        smos_WriteBuffer( sBuff, sBuffForFile );

        // For Zero UVGap - "Same Prev UV" else - [u v]
        if(dPrevStart_UVGap < SM_EFF_ZERO / 10.0)
        {
          smos_sprintf( sBuff, _T( "%s" ) , _T("StartEU_UV :[Same as Prev EndEU_UV (should be same)]  ") );
        }
        else { smos_sprintf( sBuff, _T( "StartEU_UV :[%16.16lf, %16.16lf]  " ), sEUStartUV.x, sEUStartUV.y ); }
        smos_WriteBuffer( sBuff );

        // Edgeuse->StartVertex
        if(pStartVertex == pPrevEndVertex)
        {
          smos_sprintf( sBuff, _T( "%s" ) , _T("StartVertex:[ Same Prev Vertex ] ")  );
          smos_sprintf( sBuffForFile, _T( "%s" ) , _T("StartVertex:[ Same Prev Vertex ] ") );
        }
        else
        {
          smos_sprintf( sBuff, _T( "StartVertex:[0x%p] " ), pStartVertex );
          smos_sprintf( sBuffForFile, _T( "StartVertex:[%s] " ), pStartVertex ? _T( "NotNULL" ) : _T( "NULL" ) );
        }
        smos_WriteBuffer( sBuff, sBuffForFile );

        // Edgeuse->EndVertex->XYZ
        if(pStartVertex == pPrevEndVertex)
        {
          smos_sprintf( sBuff, _T( "%s" ) , _T("XYZ:[ Same as Prev XYZ (should be same)                        ]") );
          smos_WriteBuffer( sBuff );
        }
        else
        {
          smos_WriteBuffer( _T( "XYZ:" ) );
          pStartVertex->GetPoint().Dump();
        }
      } // end Edgeuse->StartVertex ReportLine

    // Edgeuse->EndVertex report line
      {
        // Edgeuse->EndUV
        smos_sprintf( sBuff, _T( "\n                                     EndEU_UV   :[%16.16lf, %16.16lf]  " ),
                   sEUEndUV.x,
                   sEUEndUV.y );
        smos_WriteBuffer( sBuff );

        // Edgeuse->EndVertex
        if(pEndVertex == pPrevEndVertex)
        {
          smos_sprintf( sBuff, _T( "%s" ) , _T("EndVertex:[ Same Prev Vertex ] ") );
        }
        else
        {
          smos_sprintf( sBuff, _T( "EndVertex  :[0x%p] " ), pEndVertex );
          smos_sprintf( sBuffForFile, _T( "EndVertex  :[%s] " ), pEndVertex ? _T( "NotNULL" ) : _T( "NULL" ) );
        }
        smos_WriteBuffer( sBuff, sBuffForFile );

        // for End==EndVertex - Same Prev XYZ, else - [x y z]
        if(pEndVertex == pPrevEndVertex)
        {
          smos_sprintf( sBuff, _T( "%s" ) , _T("XYZ:[ Same Prev XYZ                                            ]")  );
          smos_WriteBuffer( sBuff );
        }
        else // output Vertex->Point3d
        {
          smos_WriteBuffer( _T( "XYZ:" ) );
          pEndVertex->GetPoint().Dump();
        }
      } // end Edgeuse->EndVertex ReportLine

    // Vertex on Singularity and/or Closed Surface ReportLine
      smos_sprintf( sBuff, _T( "\n      Vertex OnSingularity:[%s] OnClosedU:[%s] OnClosedV:[%s] " ),
                 sUVSectorIO.m_bVertexOnPole    ? _T( "TRUE" ) : _T( "FALSE" ),
                 sUVSectorIO.m_bVertexOnClosedU ? _T( "TRUE" ) : _T( "FALSE" ),
                 sUVSectorIO.m_bVertexOnClosedV ? _T( "TRUE" ) : _T( "FALSE" ) );
      smos_WriteBuffer( sBuff );

      // Missing Seam ReportLine
      if(sUVSectorIO.m_bSectorMissingSeamU || sUVSectorIO.m_bSectorMissingSeamV)
      {
        smos_sprintf( sBuff, _T( "\nERROR: Missing %s Seam in Sector, Expect large UVGaps" ),
                    sUVSectorIO.m_bSectorMissingSeamU && sUVSectorIO.m_bSectorMissingSeamV ? _T( "U & V" )
                  : sUVSectorIO.m_bSectorMissingSeamU ? _T( "U" )
                  : _T( "V" ) );
        smos_WriteBuffer( sBuff );
      } // end MissingSeam check

    // Edgeuse->Vertexuse (End) ReportLines
      {
        smos_sprintf( sBuff, _T( "\n      Vertexuse:[0x%p] " ), pEndVertexuse );
        smos_sprintf( sBuffForFile, _T( "\n      Vertexuse:[%s] " ), pEndVertexuse ? _T( "NotNULL" ) : _T( "NULL" ) );
        smos_WriteBuffer( sBuff, sBuffForFile );

        smos_sprintf( sBuff, _T( "VUPoint_UV :[%16.16lf, %16.16lf]    (shoud be near EndEU_UV)" ), sVUPointUV.x, sVUPointUV.y );
        smos_WriteBuffer( sBuff );

        // sVUSectorUV and sVUPointUV are now computed by the same internal calls - they'll always be the same
        //  
        //  // Vertex Sector UV
        //  smos_WriteBuffer(_T("\n                                     VUSector_UV:")) ; 
        //  if(dVU_PtSector_UVGap < SM_EFF_ZERO/10.0)
        //       { smos_sprintf(sBuff,_T("[Same as VU_PointUV    (should be same)]    (okay)"), sVUSectorUV.x, sVUSectorUV.y) ;
        //       }
        //  else { smos_sprintf(sBuff,_T("[%16.16lf, %16.16lf]    (bad: should be same)"), sVUSectorUV.x, sVUSectorUV.y) ;
        //       }
        //  smos_WriteBuffer(sBuff);
      } // end Edgeuse->Vertexuse ReportLines

    // UVGaps EdgeEdge_StartUVGap, VertEdge_MaxUVGap
      {
        // EdgeEnd_EdgeEnd UVGap - should be less than Tol2d
        smos_sprintf( sBuff, _T( "\n         UVGaps:  EUtoEU:[%16.16lf]%s" ),
                   dPrevStart_UVGap,
        (dPrevStart_UVGap < SM_EFF_ZERO_PARAM)
                   ? _T( "                                                (okay)" )
                   : _T( "                                                (bad : should be less than Tol2d)" ) );
        smos_WriteBuffer( sBuff );

        // Max Vertexuse_ThisEdgeEnd   UVGap 
        //     Vertexuse_NextEdgeStart UVGap - should be less than Tol2d
        smos_sprintf( sBuff, _T( "\n              Max_VUtoEU:[%16.16lf]%s" ),
                   dVertEdge_MaxUVGap,
        (dVertEdge_MaxUVGap < SM_EFF_ZERO_PARAM)
                   ? _T( "                                                (okay)" )
                   : _T( "                                                (bad : should be less than Tol2d)" ) );
        smos_WriteBuffer( sBuff );
      } // end Gap scope

    } // end iter every Edgeuse

  // when asked for more
    if(bAbbrev == FALSE)
    {
      // list label
      smos_sprintf( sBuff, _T( "\nDumping UVTrimCurves for [%ld] Loopuse->Edgeuses" ), sEdgeuses.GetSize() );
      smos_WriteBuffer( sBuff );

      // for every edgeuse - dump UVTrimCurve
      for(ii = 0; ii < lNum; ii++)
      {
        SmEdgeuse      * pEdgeuse = sEdgeuses[ii];

        // report and skip NULL edgeuses
        if(pEdgeuse == NULL)
        {
          smos_sprintf( sBuff, _T( "\n[%3ld] Edgeuse:[NULL]" ), ii );
          smos_WriteBuffer( sBuff );
          continue;
        }

        SmBSplineCurve * pUVTrimCurve = pEdgeuse->GetUVTrimCurve();

        // report and skip NULL UVTrimCurve
        if(pUVTrimCurve == NULL)
        {
          smos_sprintf( sBuff, _T( "\n[%3ld] pUVTrimCurve:[NULL]" ), ii );
          smos_WriteBuffer( sBuff );
          continue;
        }

        // Dump the UVTrimCurve
        smos_sprintf( sBuff, _T( "\n[%3ld] pUVTrimCurve Dump:" ), ii );
        smos_WriteBuffer( sBuff );

        pUVTrimCurve->Dump();

      } // end iter every edgeuse dumping UVTrimCurves

    // list label
      smos_sprintf( sBuff, _T( "\nEnd Dumping UVTrimCurves for [%ld] Loopuse->Edgeuses" ), sEdgeuses.GetSize() );
      smos_WriteBuffer( sBuff );

    } // end bAbbrev == FALSE check
  } // end IsEdgeLoopuse check

  smos_WriteBuffer( _T( "\nEnd SmLoopuse::Dump()\n" ) );

} // end SmLoopuse::Dump
