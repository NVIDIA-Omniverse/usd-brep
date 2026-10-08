// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*******************************************************************//**
* FILE NAME --- SmBrepData.cpp
* PURPOSE:
**********************************************************************/

#include "StdAfx.h"

#include <SmBrepData.h>

#ifndef __SMBREP_H__
#include <SmBrep.h>
#endif

#ifndef __SMREGION_H__
#include <SmRegion.h>
#endif

#ifndef __SMSHELL_H__
#include <SmShell.h>
#endif

#ifndef __SMFACEUSE_H__
#include <SmFaceuse.h>
#endif

#ifndef __SMFACE_H__
#include <SmFace.h>
#endif

#ifndef __SMLOOP_H__
#include <SmLoop.h>
#endif

#ifndef __SMVERTEX_H__
#include <SmVertex.h>
#endif

#include <SmStitch.h>
#include <SmEdgeuse.h>
#include <SmEdge.h>
#include <SmBSplineSurface.h>
#include <SmOffsetSurface.h>
#include <SmCompositeCurve.h>
// Remove Composites
// #include <SmCFace.h>
// Remove Composites
// #include <SmCEdge.h>
#include <nurbs.h>
#include <SmAssembly.h>
#include <stdio.h>



/*******************************************************************//**
 File Function declarations
***********************************************************************/

//      // only used for SM_CURRENT_DATABASE_VERSION 32 and below
//      static SmStatus  sm_InputCurve  (const SmContext & crContext, gw_CURVE   *& cur, SmDatabaseIOFile & rDB);
//      static SmStatus  sm_InputSurface(const SmContext & crContext, gw_SURFACE *& sur, SmDatabaseIOFile & rDB);
//      static SmStatus  sm_OutputSurface(gw_SURFACE *sur, SmDatabaseIOFile & rDB);
//      static SmStatus  sm_OutputCurve  (gw_CURVE   *cur, SmDatabaseIOFile & rDB);

#define MAXSIZE  256  // will be undefined at end of File

// Macro to skip a line during read. Used to skip comment lines written out to ASCII files
#define GOTO_NEXT_LINE  rFileIn.ignore(MAXSIZE,'\n')

/*******************************************************************//**
PURPOSE: SmBrepConstructor Constructor

NOTES: This class provides a top down programatic interface for
        creating Breps with open shells and solids.
***********************************************************************/
SmBrepConstructor::SmBrepConstructor()
 : m_pContext(NULL),
   m_pBrep(NULL),
   m_pRegion(NULL),
   m_pOuterRegion(NULL),
   m_pOuterShell(NULL),
   m_pInnerShell(NULL),
   m_pFace(NULL),
   m_pLoop(NULL),
   m_pEdge(NULL),
   m_pEdgeuse(NULL)
{
} // end SmBrepConstructor::SmBrepConstructor

/*******************************************************************//**
PURPOSE: Start the Brep construction process.

NOTES:
***********************************************************************/
SmBrep * SmBrepConstructor::StartBrep
 (const SmContext & crContext,          // in : context for new object construction
  double            dModelSizeEstimate) // NotUsed: in : OldTol: Def ZoneTol3d assigned to m_pBrep to use for all Brep Contained objects
                                        //      NewTol: Def ZoneTol3d assigned to Context to use for all Context contained objects
{
  SM_REF1(dModelSizeEstimate) ;
  if (m_pBrep != NULL)
    { NERN(NULL); }

  // set SmContext ZoneTol3d value - without SM_USE_NEWTOL must be before Brep constructor call
#ifdef SM_USE_NEWTOL
  SM_NEWTOL_LINE SM_ASSERT(dModelSizeEstimate > SM_EFF_ZERO);
  SM_NEWTOL_LINE if(dModelSizeEstimate != crContext.GetThisModelSizeEstimate())
  SM_NEWTOL_LINE   { crContext.SetThisModelSizeEstimate(dModelSizeEstimate) ; }
#endif // SM_USE_NEWTOL

  m_pContext = SM_CONST_CAST(SmContext*,& crContext);
  m_pBrep = new (crContext) SmBrep(
#ifdef SM_USE_NEWTOL
                                   SM_NEWTOL_LINE dModelSizeEstimate
#endif // SM_USE_NEWTOL
                                  ) ; 

  return m_pBrep;

} // end SmBrepConstructor::StartBrep

/*******************************************************************//**
PURPOSE: End the Brep construction process.

NOTES:
***********************************************************************/
SmStatus SmBrepConstructor::EndBrep
  ()
{
    // Make sure the Brep is active and everything else has been shut
    // down within the brep using the 'End**' command.
    if (   m_pBrep       == NULL
        || m_pRegion     != NULL
        || m_pOuterShell != NULL
        || m_pInnerShell != NULL
        || m_pFace       != NULL
        || m_pLoop       != NULL)
      {
        SER(SM_ERR);
      }
    m_pBrep = NULL;
    return SM_SUCCESS;

} // end SmBrepConstructor::EndBrep

/*******************************************************************//**
PURPOSE: Start the construction of a new Region.  A region is basically
    a solid.  If you are constructing an open shell you will not need
    a region.  It will automatically be created inside of the infinite
    region.

NOTES: If you are constructing a solid with inner voids you should
    first construct the outermost shell using the infinite region as the
    outer region.  Then for the void you should start a new region and use
    the previously created solid region as the outer region.
***********************************************************************/
SmRegion * SmBrepConstructor::StartRegion
  (SmBoolean ,                            // in : bIsSolidRegion = not used in this function
   SmRegion *pOuterRegion)                // in : new shell's outer region
{
    // Make sure the brep is open and there are no open regions.
    if (   m_pBrep   == NULL
        || m_pRegion != NULL)
      {
        NERN(NULL);
      }

    m_pRegion = new(m_pBrep) SmRegion();
    NERN(m_pRegion);
    m_pBrep->PostInsert(m_pRegion);
    m_pOuterRegion = pOuterRegion;
    return m_pRegion;

} // end SmBrepConstructor::StartRegion

/*******************************************************************//**
PURPOSE: End the construction of a new Region.

NOTES:
***********************************************************************/
SmStatus SmBrepConstructor::EndRegion()
{
    // Make sure the region is open and all contained shells are closed
    if (   m_pRegion     == NULL
        || m_pOuterShell != NULL
        || m_pInnerShell != NULL)
      {
        SER(SM_ERR);
      }

    m_pRegion      = NULL;
    m_pOuterRegion = NULL;

    return SM_SUCCESS;

} // end SmBrepConstructor::EndRegion

/*******************************************************************//**
PURPOSE: Start a new Shell.  If we are bounding a region then
    start two shells.  One inner and one outer.

NOTES: To create voids inside of solids start a new region for
    each void with the solids region as the outer region
    and create the corresponding shell.  The new region will
    represent the space inside of the void.  The faces created should
    be oriented such that the outer shell of the original region points
    points away from the solid volume and the outer shells of the voids
    also point away from the solid (non-void) volume.
***********************************************************************/
SmStatus SmBrepConstructor::StartShell
  (SmShell *& rpOuterShell,
   SmShell *& rpInnerShell)    // in : only used if bounding a solid region
                               //      for open shells this will be NULL
{
  if (   m_pBrep == NULL
      || m_pOuterShell != NULL
      || m_pInnerShell != NULL)
    {
      SER(SM_ERR);
    }

  rpOuterShell = NULL;
  rpInnerShell = NULL;

  m_pOuterShell = new(m_pBrep) SmShell(); NER(m_pOuterShell);
  m_pOuterShell->SetShellType(SmFaceuse_TYPE);
  rpOuterShell = m_pOuterShell;

  // If m_pOuterRegion is NULL then everything goes into the infinite
  // region.
  if (m_pOuterRegion == NULL)
    {
      m_pOuterRegion = m_pBrep->GetInfiniteRegion();
    }

  // Test for open shell case
  m_pOuterRegion->PostInsert(m_pOuterShell);

  m_pInnerShell = NULL;

  if (m_pRegion == NULL)
    {
      return SM_SUCCESS;
    }
  m_pInnerShell = new(m_pBrep) SmShell(); NER(m_pInnerShell);
  m_pInnerShell->SetShellType(SmFaceuse_TYPE);
  m_pRegion->PostInsert(m_pInnerShell);
  rpInnerShell = m_pInnerShell;

  return SM_SUCCESS;

} // end SmBrepConstructor::StartShell

/*******************************************************************//**
PURPOSE: End the construction of this shell.

NOTES:
***********************************************************************/
SmStatus SmBrepConstructor::EndShell
  (SmBoolean bStitchShell)
{
  if (   m_pOuterShell == NULL
      || m_pFace       != NULL)
    {
      SER(SM_ERR);
    }

  if (bStitchShell)
    {
      m_pBrep->ValidatePointers();
      SmTArray<SmFaceuse*> sFaceuses;
      SmTArray<SmEdge*>    sShellEdges;
      SmTArray<SmEdge*>    sFaceEdges;
      SmTArray<SmVertex*>  sShellVertices;
      SmTArray<SmVertex*>  sFaceVertices;
      m_pOuterShell->GetFaceuses(sFaceuses);

      for (ULONG i=0; i<sFaceuses.GetSize(); i++)
        {
          sFaceuses[i]->GetFace()->GetEdges(sFaceEdges);
          sFaceuses[i]->GetFace()->GetVertices(sFaceVertices);
          sShellEdges.Append(sFaceEdges);
          sShellVertices.Append(sFaceVertices);
        }

      SmStitchCallback sCallBack;
      SmStitch sStitch(sCallBack,m_pBrep->GetTolerance(),FALSE,TRUE);
      ULONG lNumEdgesStitched, lLaminaEdges;
      double dMaxEGap, dMaxVGap;
      SER(sStitch.DoStitching(m_pBrep,&sShellVertices,&sShellEdges,lNumEdgesStitched,
                        lLaminaEdges,dMaxEGap,dMaxVGap));
    }

  m_pOuterShell = NULL;
  m_pInnerShell = NULL;
  return SM_SUCCESS;

} // end SmBrepConstructor::EndShell

/*******************************************************************//**
PURPOSE: Create a new face object along with its faceuses and store
    them into the proper shells.

NOTES:
***********************************************************************/
SmFace * SmBrepConstructor::StartFace
  (SmOrientType eOutsideFaceuse) // in : If the natural surface normal of the face
                                 //      points to the inside of the
                                 //      solid use OPPOSITE.  Otherwise use SAME.

{
  if (   m_pBrep       == NULL
      || m_pOuterShell == NULL)
    {
      SE(SM_ERR);
      return NULL;
    }

  m_pFace = new(m_pBrep) SmFace(); NERN(m_pFace);
  m_pBrep->m_pFaceListHead->PostInsert(m_pFace);

  SM_OLDTOL_LINE m_pFace->m_sZoneTol3d = (m_pBrep->GetTolerance()) ;

  SmFaceuse *pFU1 = new(m_pBrep) SmFaceuse(); NERN(pFU1);
  SmFaceuse *pFU2 = new(m_pBrep) SmFaceuse(); NERN(pFU2);

  // Connect faceuses with face
  pFU1->m_pF = m_pFace;
  pFU2->m_pF = m_pFace;

  // Connect up faceuses to each other
  pFU1->m_pFUMate  = pFU2;
  pFU2->m_pFUMate  = pFU1;

  // Set orientations
  m_pFace->m_pFU = pFU1;                 // set pFU1 as the upward faceuse
  pFU1->m_eOrientation = SM_OT_SAME;
  pFU2->m_eOrientation = SM_OT_OPPOSITE;
  // Connect up to shells
  if (eOutsideFaceuse == SM_OT_SAME)
    {
      m_pOuterShell->PostInsert(pFU1);
      if (m_pInnerShell)
        { m_pInnerShell->PostInsert(pFU2); }
      else
        { m_pOuterShell->PostInsert(pFU2); }
    }
  else
    {
      if (m_pInnerShell)
        { m_pInnerShell->PostInsert(pFU1); }
      else
        { m_pOuterShell->PostInsert(pFU1); }
      m_pOuterShell->PostInsert(pFU2);
    }

  return m_pFace;

} // end SmBrepConstructor::StartFace

/*******************************************************************//**
PURPOSE: Finish this face.

NOTES:
***********************************************************************/
SmStatus SmBrepConstructor::EndFace()
{
    if (   m_pFace == NULL
        || m_pLoop != NULL)
      {
        SER(SM_ERR);
      }

    m_pFace = NULL;
    return SM_SUCCESS;

} // end SmBrepConstructor::EndFace

/*******************************************************************//**
PURPOSE: Start and end a single vertex loop in the currently open face.

NOTES:
***********************************************************************/
SmLoop * SmBrepConstructor::StartAndEndSingleVertexLoop
  (const SmPoint3d & crVertexPoint)
{
  if (   m_pBrep == NULL
      || m_pFace == NULL)
    {
      SE(SM_ERR);
      return NULL;
    }

  SmLoop *pLoop = StartLoop(SM_OT_SAME);  // default behavior
  NERN(pLoop);
  SmLoopuse *pLU1, *pLU2;
  pLoop->GetLoopuses(pLU1,pLU2);

  SmVertexuse * pVU1 = new(m_pBrep) SmVertexuse();
  SmVertexuse * pVU2 = new(m_pBrep) SmVertexuse();
  SmVertex    * pV   = new(m_pBrep) SmVertex();

  // Connect vertex and vertexuses
  pV->PostInsert(pVU1);
  pV->PostInsert(pVU2);

  // Connect vertexuses to loop
  pLU1->m_pEUorVU = pVU1;
  pLU2->m_pEUorVU = pVU2;
  pLU1->m_tLoopuseType = SmVertexuse_TYPE;
  pLU2->m_tLoopuseType = SmVertexuse_TYPE;
  pVU1->m_pSorLUorEU = pLU1;
  pVU1->m_tVertexuseType = SmLoopuse_TYPE;
  pVU2->m_pSorLUorEU = pLU2;
  pVU2->m_tVertexuseType = SmLoopuse_TYPE;

  // Set Vertex data
  pV->m_vPoint          = crVertexPoint;
  SM_OLDTOL_LINE pV->m_sZoneTol3d = (m_pBrep->GetTolerance()) ;

  // Add vertex to brep
  m_pBrep->m_pVertexListHead->PostInsert(pV);

  m_pLoop = NULL;

  return pLoop;

} // end SmBrepConstructor::StartAndEndSingleVertexLoop

/*******************************************************************//**
PURPOSE: Allocate a new Vertex recorded in the m_pBrep Vertex list
         and given a point position

NOTES:
***********************************************************************/
SmVertex* SmBrepConstructor::StartVertexOfLoop
 (SmPoint3d sPnt)
{
  SmVertex* pV = new(m_pBrep) SmVertex();

  m_pBrep->m_pVertexListHead->PostInsert(pV);

  SM_OLDTOL_LINE pV->m_sZoneTol3d = (m_pBrep->GetTolerance()) ;

  pV->SetPoint(sPnt);

  return pV;

} // end SmBrepConstructor::StartVertexOfLoop

/*******************************************************************//**
PURPOSE: Start a loop.  Outer loops need to have their edges go
    counter clockwise relative to the natural surface normal.  Inner
    loops go clockwise relative to the natural surface normal.

NOTES:
***********************************************************************/
SmLoop * SmBrepConstructor::StartLoop(SmOrientType eOrientationToSurface)
{
  if (   m_pBrep == NULL
      || m_pFace == NULL)
    {
      SE(SM_ERR);
      return NULL;
    }

  m_pEdge = NULL;
  m_pLoop = new(m_pBrep) SmLoop(); NERN(m_pLoop);
  SmLoopuse *pLU1 = new(m_pBrep) SmLoopuse(); NERN(pLU1);
  SmLoopuse *pLU2 = new(m_pBrep) SmLoopuse(); NERN(pLU2);

  // Connect up Loop and loopuses
  pLU1->m_pL = m_pLoop;
  pLU2->m_pL = m_pLoop;

  m_pFace->m_pFU->PostInsert(pLU1);  // connect pLU1 to upward faceuse
  m_pFace->m_pFU->m_pFUMate->PostInsert(pLU2);
  m_pLoop->m_pLU = pLU1;             // Primary loopuse is the one connected to upward faceuse

  // cache the loop orientation relative to the surface
  m_eLoopSurfaceOrientation = eOrientationToSurface;
  // Set loop orientation
  if (pLU1->m_pNext == pLU1)
    { // First loopuse must be outer one
      pLU1->m_eOrientation = SM_OT_SAME;
      pLU2->m_eOrientation = SM_OT_SAME;
    }
  else
    {
      pLU1->m_eOrientation = SM_OT_OPPOSITE;
      pLU2->m_eOrientation = SM_OT_OPPOSITE;
    }

  // Connect loopuses to each other
  pLU1->m_pLUMate = pLU2;
  pLU2->m_pLUMate = pLU1;

  return m_pLoop;

} // end SmBrepConstructor::StartLoop

/*******************************************************************//**
PURPOSE: Finish off a loop and construct vertices.

NOTES:
***********************************************************************/
SmStatus SmBrepConstructor::EndLoop()
{
  if (m_pLoop == NULL)
    {
      SER(SM_ERR);
    }

  // Here we need to loop through and create vertices for each vertexuse.

  SmVertexuse *sVUData[64];
  SmTArray<SmVertexuse*> sVertexuses(64,sVUData);
  m_pLoop->m_pLU->GetVertexuses(sVertexuses);

  for (ULONG i=0; i<sVertexuses.GetSize(); i++)
    {
      SmVertexuse * pVU        = sVertexuses[i];
      SmEdgeuse   * pEU        = SM_CAST_PTR(SmEdgeuse,pVU->m_pSorLUorEU);
      SmEdge      * pE         = pEU->GetEdge();
      SmVertex    * pV         = NULL;
      SmEdgeuse   * pMate      = pEU->GetMate();
      SmEdgeuse   * pMateCCW   = pMate->m_pCCW;
      SmVertexuse * pVUMateCCW = pMateCCW->GetVertexuse();
      SmEdgeuse   * pRadial    = pEU->GetRadial();

      SmVertexuse *pVURadialCCW = NULL;
      if (pRadial != pMate)
        {
          pVURadialCCW = pRadial->m_pCCW->GetVertexuse();
        }

      if (pVU->m_pListOwner == NULL)
        {
          if (pVUMateCCW->m_pListOwner != NULL)
            {
              pV = (SmVertex*)pVUMateCCW->m_pListOwner;
            }
          else if (pVURadialCCW && pVURadialCCW->m_pListOwner != NULL)
            {
              pV = (SmVertex*)pVURadialCCW->m_pListOwner;
            }
          else if (pRadial &&
              pRadial->GetVertexuse()->m_pListOwner)
            {
              SmBoolean bClosed = pE->GetCurve()->IsClosed(pE->GetInterval(),pE->GetTolerance());
              if (bClosed)
                {
                  pV = (SmVertex*)pRadial->GetVertexuse()->m_pListOwner;
                }
            }

          if (!pV)
            {
              pV = new(m_pBrep) SmVertex();
              m_pBrep->m_pVertexListHead->PostInsert(pV);
  
              SM_OLDTOL_LINE pV->m_sZoneTol3d = (m_pBrep->GetTolerance()) ;

              NER(pEU);
              SmEdge *pEdge = pEU->GetEdge();
              SmCurve *pCurve = pEdge->GetCurve();
              double dT = pEdge->GetInterval().GetMin();
              if (pEU->GetOrientation() == SM_OT_OPPOSITE)
                {
                  dT = pEdge->GetInterval().GetMax();
            }
              SmPoint3d sPnt;
              SER(pCurve->EvaluatePoint(dT,sPnt));
              pV->SetPoint(sPnt);
            }
          pV->PostInsert(pVU);
        }
      else
        {
          pV = (SmVertex*)pVU->m_pListOwner;
        }

      // Now hook up vertexuse on the bottom loopuse
      if (pVUMateCCW->m_pListOwner == NULL)
        {
          pV->PostInsert(pVUMateCCW);
        }
      SmEdgeuse *pCW = pEU->GetCWEdgeuse();
      SmEdgeuse *pCWMate = pCW->GetMate();
      if (pCWMate != pMateCCW)
        {
          SER(SM_ERR);
        }

      // Check for coincident vertices within the loop
      // Not sure this is a good place for this. Deleting vertices during construction can cause
      // stale pointers for callers that are tracking vertices.
      // Also, this is not a sufficient check for coincident verts.
      // Is there any real advantage to doing it here instead of in the healer?
//      SmVertex* pThisVertex = pVU->GetVertex();
//      SmVertex* pOtherVertex = pCW->GetMate()->GetVertexuse()->GetVertex();
//      if (pThisVertex != pOtherVertex)
//      {
//#ifdef IW_DEBUG_CODE
//          SmPoint3d sThisPoint = pThisVertex->GetPoint();
//          SmPoint3d sOtherPoint = pOtherVertex->GetPoint();
//          IW_ASSERT(
//              IW_IS_ZERO_TO_TOL((sThisPoint - sOtherPoint).Length(), SmTol::GetXSectTol3d(pThisVertex, pOtherVertex)));
//#endif
//
//          m_pBrep->GlueVertices(pVU->GetVertex(), pCW->GetMate()->GetVertexuse()->GetVertex());
//      }

    }

  m_pLoop = NULL;
  m_pEdge = NULL;
  m_eLoopSurfaceOrientation = SM_OT_UNKNOWN;
  return SM_SUCCESS;

} // end SmBrepConstructor::EndLoop

/*******************************************************************//**
PURPOSE: Start the construction of an edge by building the major
    topological relationships.  If an edge already exists then just
    insert the new edgeuse pair.  Please note that this does not handle
    cases where more than 2 faces belong to an edge.

NOTES: Perhaps we need to do something here for seam edges
    which are duplicate or for Poles which may map to nothing.
***********************************************************************/
SmEdge * SmBrepConstructor::StartEdge
 (SmOrientType eOrientationInLoop,
  SmEdge * pOptExistingEdge,           // If specified
                                       // it will automatically join to an existing
                                       // edge.  If you do this than you will not
                                       // need to specify stitching to take place after
                                       // the shell is created.
  SmVertex * pOptExistingStartVertex,  // If pOptExisting
                                       // Edge is NULL then the start and end vertex may be
                                       // specified.  The order of start/end vertex should
                                       // be given relative to the loop orientation.  The
                                       // Start Vertex to the End Vertex is Counter Clockwise
                                       // in the loop.  You do not need to specify both
                                       // the start and end vertex.  You may have one of them
                                       // as NULL and the other specified.  The ordering
                                       // in formation still holds however.
  SmVertex * pOptExistingEndVertex)

{
  if (m_pLoop == NULL)
    {
      SE(SM_ERR);
      return NULL;
    }

  SmEdge *pEdge = NULL;
  if (pOptExistingEdge == NULL)
    {
      pEdge = new(m_pBrep) SmEdge();
      m_pBrep->m_pEdgeListHead->PostInsert(pEdge);

      SM_OLDTOL_LINE pEdge->m_sZoneTol3d = (m_pBrep->GetTolerance()) ;

    }
  else
    {
      pEdge = pOptExistingEdge;
    }
  SmEdgeuse   * pEU1 = new(m_pBrep) SmEdgeuse();
  SmEdgeuse   * pEU2 = new(m_pBrep) SmEdgeuse();
  SmVertexuse * pVU1 = new(m_pBrep) SmVertexuse();
  SmVertexuse * pVU2 = new(m_pBrep) SmVertexuse();

  // Hook up vertexuses and edgeuses
  pEU1->m_pVU            = pVU1;
  pEU2->m_pVU            = pVU2;
  pVU1->m_pSorLUorEU     = pEU1;
  pVU2->m_pSorLUorEU     = pEU2;
  pVU1->m_tVertexuseType = SmEdgeuse_TYPE;
  pVU2->m_tVertexuseType = SmEdgeuse_TYPE;

  // Set orientation
  if (eOrientationInLoop == m_eLoopSurfaceOrientation)
    {
      pEU1->m_eOrientation = SM_OT_SAME;
      pEU2->m_eOrientation = SM_OT_OPPOSITE;
    }
  else
    {
      pEU1->m_eOrientation = SM_OT_OPPOSITE;
      pEU2->m_eOrientation = SM_OT_SAME;

      SmVertex * pTmp = pOptExistingStartVertex;
      pOptExistingStartVertex = pOptExistingEndVertex;
      pOptExistingEndVertex = pTmp;
    }

  // Set edgeuse type
  pEU1->m_tEdgeuseType = SmLoopuse_TYPE;
  pEU2->m_tEdgeuseType = SmLoopuse_TYPE;

  // Set Loopuse pointers
  pEU1->m_pSorLU = m_pLoop->m_pLU;
  m_pLoop->m_pLU->m_pEUorVU = pEU1;
  pEU2->m_pSorLU = m_pLoop->m_pLU->m_pLUMate;
  m_pLoop->m_pLU->m_pLUMate->m_pEUorVU = pEU2;

  m_pLoop->m_pLU->m_tLoopuseType = SmEdgeuse_TYPE;
  m_pLoop->m_pLU->m_pLUMate->m_tLoopuseType = SmEdgeuse_TYPE;

  // Set CCW and CW pointers
  if (m_pEdge == NULL)
    {
      // first edge make loop closed and add other edgeuses later
      pEU1->m_pCCW = pEU1;
      pEU1->m_pCW = pEU1;
      pEU2->m_pCCW = pEU2;
      pEU2->m_pCW = pEU2;
    }
  else if ( m_eLoopSurfaceOrientation == SM_OT_SAME)
    {
      // If we're walking the loop with the left hand rule for the upward side of the surface
      SmEdgeuse *pPrevEU1 = m_pEdgeuse;
      SmEdgeuse *pPrevEU2 = pPrevEU1->GetMate();
      pEU1->m_pCCW = pPrevEU1->m_pCCW;
      pEU1->m_pCCW->m_pCW = pEU1;
      pPrevEU1->m_pCCW = pEU1;
      pEU1->m_pCW = pPrevEU1;

      pEU2->m_pCW = pPrevEU2->m_pCW;
      pEU2->m_pCW->m_pCCW = pEU2;
      pPrevEU2->m_pCW = pEU2;
      pEU2->m_pCCW = pPrevEU2;
    }
    else
    {
      // If we're walking the loop with the left hand rule for the downward side of the surface
      SmEdgeuse *pPrevEU1 = m_pEdgeuse;
      SmEdgeuse *pPrevEU2 = pPrevEU1->GetMate();
      pEU1->m_pCW = pPrevEU1->m_pCW;
      pEU1->m_pCW->m_pCCW = pEU1;
      pPrevEU1->m_pCW = pEU1;
      pEU1->m_pCCW = pPrevEU1;

      pEU2->m_pCCW = pPrevEU2->m_pCCW;
      pEU2->m_pCCW->m_pCW = pEU2;
      pPrevEU2->m_pCCW = pEU2;
      pEU2->m_pCW = pPrevEU2;
    }

  // Connect up edge and edgeuses.
  // Note that the first edgeuse has to be same orientation as the edge.
  if (pOptExistingEdge == NULL)
    {
      if (pOptExistingStartVertex)
        {
          pOptExistingStartVertex->PostInsert(pVU1);
        }
      if (pOptExistingEndVertex)
        {
          pOptExistingEndVertex->PostInsert(pVU2);
        }
      if (pEU1->GetOrientation() == SM_OT_SAME)
        {
          pEdge->PostInsert(pEU1);
          pEdge->PostInsert(pEU2);
        }
      else
        {  // pEU2 is same orient
          pEdge->PostInsert(pEU2);
          pEdge->PostInsert(pEU1);
        }
    }
  else
    {
      SmEdgeuse * pEU     = pEdge->GetPrimaryEdgeuse();
      SmEdgeuse * pEUMate = pEU->GetMate();
      SmEdgeuse* pEUSector = NULL;
      SmVertexuse * pEUVU   = pEU->GetVertexuse();

      // Attach new EUs to existing VUs
      if (pEU->GetOrientation() == pEU1->GetOrientation())
        {
          SmVertex *pV = pEUVU->GetVertex();
          if (pV) { pV->PostInsert(pVU1); }

          pV = pEUMate->GetVertexuse()->GetVertex();
          if (pV) { pV->PostInsert(pVU2); }
        }
      else {
          SmVertex *pV = pEUVU->GetVertex();
          if (pV) { pV->PostInsert(pVU2); }
          pV = pEUMate->GetVertexuse()->GetVertex();
          if (pV) { pV->PostInsert(pVU1); }
        }

      // Find the Edgeuse sector for the new EU's
      if (pEdge->IsLamina())
        { pEUSector = pEUMate; } // Simple if manifold
      else
        {
          // If not manifold, find the radial sector this face lives in.

          // Set BrepConstructor manifold flag. We'll clean up Regions and Shells in EndBrep.
          m_bIsManifold = FALSE;

          // Create a temporary edge
          SmEdge * pTempEdge = new (m_pBrep) SmEdge();
          SmCurve* pTempCurve = NULL;
          m_pBrep->m_pEdgeListHead->PostInsert(pTempEdge);
          SM_OLDTOL_LINE pTempEdge->m_sZoneTol3d = (m_pBrep->GetTolerance());
          pTempEdge->SetInterval(pEdge->GetInterval());
          if (pEU1->GetOrientation() == SM_OT_SAME)
          {
              pTempEdge->PostInsert(pEU1);
              pTempEdge->PostInsert(pEU2);
          }
          else
          { // pEU2 is same orient
              pTempEdge->PostInsert(pEU2);
              pTempEdge->PostInsert(pEU1);
          }
          pEdge->GetCurve()->Copy(*GetContext(), pTempCurve);
          pTempEdge->SetCurve(pTempCurve);
          SmObjDelete sCleanEdge(pTempEdge);

          // Can't use this function, because it assumes the Brep is valid, not under construction
          //m_pBrep->GlueEdgesGeneral(pEdge, SM_OT_SAME, 0., FALSE, pTempEdge);

          // Find the
          SmFaceuse* pCoincidentFaceuse = NULL;
          SmOrientType eEdgeuseOT = SM_OT_SAME; // This doesn't matter, as AddOrientedEUPair adjusts for either case
          SE(pEdge->FindRadialSector(pEU1,                      // in : contains face whose geometry will be checked
                                     NULL,                      // in : If NULL it will
                                                                //      test sectors in all regions for a possible answer.
                                     eEdgeuseOT,                // in : The answer must be the edgeuse which has this orientation
                                                                //      of the faceuse of an edgeuse with this orientation.
                                     pEUSector,                 // out: If an answer is found then this will be the
                                                                //      resulting edgeuse; otherwise Null.  In case of
                                                                //      coincidence this will be the edgeuse that's in
                                                                //      rpCoincidentFaceuse.
                                     pCoincidentFaceuse));      // out: If there is no coincidence this will be NULL.  If a
                                                                //      coincidence is found then this is the coincident
                                                                //      faceuse.  Note that we should never get a
                                                                //      coincidence if the region is specified.

          // JGU: BrepConstructor extended for non-manifold imports from PRC, which I don't think allows
          // coincident faces sharing an edge.
          // See SmBrep::GlueEdgeGeneral for possible remedy if this occurs
          SM_ASSERT_MSG(pCoincidentFaceuse == NULL, _T("Unexpected coincident face. Investigate."));

          // Fallback option so we have something
          if (pEUSector == NULL)
          { pEUSector = pEUMate; }

          pTempEdge->Remove(pEU1);
          pTempEdge->Remove(pEU2);
        } // end 

        SE(pEdge->AddOrientedEUPair(pEU1, pEU2, pEUSector));
    }

  m_pEdge = pEdge;
  m_pEdgeuse = pEU1;

  return m_pEdge;

} // end SmBrepConstructor::StartEdge

/*******************************************************************//**
PURPOSE: Set the boundaries of an edge.  Note that the start
    vertex node and end vertex node correspond to the minimum and
    maximum curve parameters.  The parameter produced by StartVertexNode
    is always less than the parameter produced by EndVertexNode.

NOTES: Needs to have an open edge with a curve.
***********************************************************************/
SmStatus SmBrepConstructor::SetLimits
  (const SmVertexNode & crStartVertexNode,
   const SmVertexNode & crEndVertexNode)
{
    if (m_pEdge == NULL ||
        m_pEdge->GetCurve() == NULL) {
        SER(SM_ERR);
    }
    double dMin, dMax;

    // Set the limits on the current edge.
    if (crStartVertexNode.m_lType == 1) {
        dMin = crStartVertexNode.m_dParameter;
    }
    else {
        SmCurve *pCurve = m_pEdge->GetCurve();
        double dDistToCurve, dDroppedParam;
        SmBoolean bSuccess;
        SER(pCurve->DropPoint(pCurve->GetNaturalInterval(), // in : target curve allowed domain
                              crStartVertexNode.m_vPoint,   // in : Point to drop to curve
                              NULL,                         // in : For Snapping, NULL=NoSnap, Vec pointing to CrvInside from desired EndPt for snapping.
                                                            //      when DropPt is within dDistTol in cpVecToCrvInside dir of EndPt, Snap to Ivl EndPt.
                                                            //      Vec handles ambiguities and makes sure that curves are not just touching at the ends.
                              m_pEdge->GetTolerance(),      // in : Used for SnapDist, and SM_SO_INTERSECT sXSectTol3d distance.
                                                            //      sXSectTol3d = dDistTol ;      SM_SO_INTERSECT 
                                                            //      sXSectTol3d = SM_BIG_DOUBLE ; SM_SO_MINIMIZE, SM_SO_NORMALIZE
                                                            //      sSnapTol3d  = dDistTol ;      SM_SO_MINIMIZE, SM_SO_NORMALIZE, SM_SO_INTERSECT    
                              NULL,                         // in : If given, call cheap LocalPointSolve() else call expensive GlobalPointSolve()
                              bSuccess,                     // out: TRUE = found a drop point
                              dDroppedParam,                // out: found drop curve param
                              dDistToCurve)) ;              // out: found drop distance
                                                            // in : SM_SO_MINIMIZE = allow nonNormal drops near endPoints
                                                            //      SM_SO_INTERSECT= like Minimize but point must be within dDistTol
                                                            //      SM_SO_NORMALIZE= exclude nonNormal drops near endPoints
                                                            //      default:[SM_SO_MINIMIZE] to preserve original behavior
        if(!bSuccess) 
          {
            SER(pCurve->DropPoint(pCurve->GetNaturalInterval(),  // in : target curve allowed domain
                                  crStartVertexNode.m_vPoint,    // in : Point to drop to curve
                                  NULL,                          // in : For Snapping, NULL=NoSnap, Vec pointing to CrvInside from desired EndPt for snapping.
                                                                 //      when DropPt is within dDistTol in cpVecToCrvInside dir of EndPt, Snap to Ivl EndPt.
                                                                 //      Vec handles ambiguities and makes sure that curves are not just touching at the ends.
                                  m_pEdge->GetTolerance()*100.0, // in : Used for SnapDist, and SM_SO_INTERSECT sXSectTol3d distance.
                                                                 //      sXSectTol3d = dDistTol ;      SM_SO_INTERSECT 
                                                                 //      sXSectTol3d = SM_BIG_DOUBLE ; SM_SO_MINIMIZE, SM_SO_NORMALIZE
                                                                 //      sSnapTol3d  = dDistTol ;      SM_SO_MINIMIZE, SM_SO_NORMALIZE, SM_SO_INTERSECT    
                                  NULL,                          // in : If given, call cheap LocalPointSolve() else call expensive GlobalPointSolve()
                                  bSuccess,                      // out: TRUE = found a drop point
                                  dDroppedParam,                 // out: found drop curve param
                                  dDistToCurve)) ;               // out: found drop distance
                                                                 // in : SM_SO_MINIMIZE = allow nonNormal drops near endPoints
                                                                 //      SM_SO_INTERSECT= like Minimize but point must be within dDistTol
                                                                 //      SM_SO_NORMALIZE= exclude nonNormal drops near endPoints
                                                                 //      default:[SM_SO_MINIMIZE] to preserve original behavior
            if (!bSuccess) SER(SM_ERR);
          }
        dMin = dDroppedParam;

        // snap dMin to curve->interval->min when 3d point is within tol
        // Accounts for dropped point falling on wrong side of seam
        SmPoint3d sEdgeMin, sCurveMin;
        pCurve->EvaluatePoint(dMin, sEdgeMin);
        pCurve->EvaluatePoint(pCurve->GetNaturalInterval().GetMin(), sCurveMin);
        SmZoneTol3d sTol = SmTol::GetZoneTol3d(m_pEdge);

        if (  sEdgeMin.DistanceBetweenSquared(sCurveMin) < sTol*sTol)
          { dMin = pCurve->GetNaturalInterval().GetMin(); }
    }

    if (crEndVertexNode.m_lType == 1) {
        dMax = crEndVertexNode.m_dParameter;
    }
    else 
      {
        SmCurve *pCurve = m_pEdge->GetCurve();
        double dDistToCurve, dDroppedParam;
        SmBoolean bSuccess;
        SER(pCurve->DropPoint(pCurve->GetNaturalInterval(), // in : target curve allowed domain
                              crEndVertexNode.m_vPoint,     // in : Point to drop to curve
                              NULL,                         // in : For Snapping, NULL=NoSnap, Vec pointing to CrvInside from desired EndPt for snapping.
                                                            //      when DropPt is within dDistTol in cpVecToCrvInside dir of EndPt, Snap to Ivl EndPt.
                                                            //      Vec handles ambiguities and makes sure that curves are not just touching at the ends.
                              m_pEdge->GetTolerance(),      // in : Used for SnapDist, and SM_SO_INTERSECT sXSectTol3d distance.
                                                            //      sXSectTol3d = dDistTol ;      SM_SO_INTERSECT 
                                                            //      sXSectTol3d = SM_BIG_DOUBLE ; SM_SO_MINIMIZE, SM_SO_NORMALIZE
                                                            //      sSnapTol3d  = dDistTol ;      SM_SO_MINIMIZE, SM_SO_NORMALIZE, SM_SO_INTERSECT    
                              NULL,                         // in : If given, call cheap LocalPointSolve() else call expensive GlobalPointSolve()
                              bSuccess,                     // out: TRUE = found a drop point
                              dDroppedParam,                // out: found drop curve param
                              dDistToCurve)) ;              // out: found drop distance
                                                            // in : SM_SO_MINIMIZE = allow nonNormal drops near endPoints
                                                            //      SM_SO_INTERSECT= like Minimize but point must be within dDistTol
                                                            //      SM_SO_NORMALIZE= exclude nonNormal drops near endPoints
                                                            //      default:[SM_SO_MINIMIZE] to preserve original behavior

        if (!bSuccess) 
          {
            SER(pCurve->DropPoint(pCurve->GetNaturalInterval(),  // in : target curve allowed domain
                                  crEndVertexNode.m_vPoint,      // in : Point to drop to curve
                                  NULL,                          // in : For Snapping, NULL=NoSnap, Vec pointing to CrvInside from desired EndPt for snapping.
                                                                 //      when DropPt is within dDistTol in cpVecToCrvInside dir of EndPt, Snap to Ivl EndPt.
                                                                 //      Vec handles ambiguities and makes sure that curves are not just touching at the ends.
                                  m_pEdge->GetTolerance()*100.0, // in : Used for SnapDist, and SM_SO_INTERSECT sXSectTol3d distance.
                                                                 //      sXSectTol3d = dDistTol ;      SM_SO_INTERSECT 
                                                                 //      sXSectTol3d = SM_BIG_DOUBLE ; SM_SO_MINIMIZE, SM_SO_NORMALIZE
                                                                 //      sSnapTol3d  = dDistTol ;      SM_SO_MINIMIZE, SM_SO_NORMALIZE, SM_SO_INTERSECT    
                                  NULL,                          // in : If given, call cheap LocalPointSolve() else call expensive GlobalPointSolve()
                                  bSuccess,                      // out: TRUE = found a drop point
                                  dDroppedParam,                 // out: found drop curve param
                                  dDistToCurve)) ;               // out: found drop distance
                                                                 // in : SM_SO_MINIMIZE = allow nonNormal drops near endPoints
                                                                 //      SM_SO_INTERSECT= like Minimize but point must be within dDistTol
                                                                 //      SM_SO_NORMALIZE= exclude nonNormal drops near endPoints
                                                                 //      default:[SM_SO_MINIMIZE] to preserve original behavior

            if (!bSuccess) SER(SM_ERR);
          }
        dMax = dDroppedParam;

        // snap dMax to curve->interval->max when 3d point is within tol
        // Accounts for dropped point falling on wrong side of seam
        SmPoint3d sEdgeMax, sCurveMax;
        pCurve->EvaluatePoint(dMax, sEdgeMax);
        pCurve->EvaluatePoint(pCurve->GetNaturalInterval().GetMax(), sCurveMax);
        SmZoneTol3d sTol = SmTol::GetZoneTol3d(m_pEdge);

        if (  sEdgeMax.DistanceBetweenSquared(sCurveMax) < sTol*sTol)
          { dMax = pCurve->GetNaturalInterval().GetMax(); }
      }

    SER(m_pEdge->m_vInterval.SetMinMax(dMin, dMax));

    return SM_SUCCESS;

} // end SmBrepConstructor::SetLimits

/*******************************************************************//**
PURPOSE: Finish construction of the edge.

NOTES:
***********************************************************************/
SmStatus SmBrepConstructor::EndEdge()
{
    if (m_pEdge == NULL) {
        SER(SM_ERR);
    }

    return SM_SUCCESS;

} // end SmBrepConstructor::EndEdge

/*******************************************************************//**
// Remove Composites
//   PURPOSE: Associate this surface to the face. If the surface belongs
//   to more than one face it will automatically build a composite face.
   PURPOSE: Associate this surface to the face. If the surface already
   belongs to a face - a copy of the surface will be bound to the face.

NOTES:
***********************************************************************/
SmStatus SmBrepConstructor::SetFaceSurface
  (SmSurface        * pSurface,    // in : Face's new Surface (or Srf to copy when pSurface already has a Face owner)
   const SmExtent2d & crUVDomain)  // in : Face's new UVDomain
{
  SmFace    * pFOrig      = SM_CAST_PTR(SmFace, pSurface->GetOwner()) ;
  SmSurface * pTgtSurface = pSurface ; 

  // copy surface when pSurface already has a Face owner
  if(pFOrig != NULL)
    {
      pTgtSurface = new (GetContext()) SmSurface(*pSurface) ; 
    }

  // set Face<->surface pointers
  pTgtSurface->SetOwner(m_pFace);
  m_pFace->m_pSurface = pTgtSurface;
  m_pFace->m_vUVDomain = crUVDomain;

// Remove Composites
//  SmFace * pFOrig = SM_CAST_PTR(SmFace, pSurface->GetOwner());
//   if (pFOrig == NULL) 
//     {
//       pSurface->SetOwner(m_pFace);
//       m_pFace->m_pSurface = pSurface;
//     }
//   else 
//     {
//       SmCFace *pCFace = NULL;
//       pCFace = SM_CAST_PTR(SmCFace,pFOrig);
//       if (pCFace) 
//         {
//           pCFace->AddFace(m_pFace);
//         }
//       else 
//         {
//           pCFace = new(m_pBrep) SmCFace(pFOrig,m_pFace);
//           SE(m_pBrep->m_pCFaceListHead->PostInsert(pCFace));
//         }
//     }
//   m_pFace->m_vUVDomain = crUVDomain;

  // all done
  return SM_SUCCESS;

} // end SmBrepConstructor::SetFaceSurface

/*******************************************************************//**
PURPOSE: Associate a curve to the edge.  The curve can either be
   a UV curve or a 3D curve.  If UV then a new 3D curve is created
   by lifting.  If 3D a UV curve is not created.  This allows us to
   fix periodic stuff first.

NOTES:
***********************************************************************/
SmStatus SmBrepConstructor::SetEdgeCurve
  (SmCurve *pCurve,
   SmBoolean bUVCurve)
{
    if (m_pEdge == NULL ||
        m_pEdgeuse == NULL) {
        SER(SM_ERR);
    }

    SmFace *pFace = m_pEdgeuse->GetFace();
    SmSurface *pSurf = pFace->GetSurface();

    if (!bUVCurve) {
        if (m_pEdge->m_pCurve == NULL) {
            m_pEdge->m_pCurve = pCurve;
            pCurve->SetOwner(m_pEdge);
        }
        else {
            SER(SM_ERR); // Don't need to set curve of this edge it
            // has already been done.
        }
    }
    else {
#ifdef SM_DEBUG_CODE
              if(pCurve != NULL && pCurve->GetDim() != 2)
                {
                  SM_ASSERT_MSG(pCurve == NULL || pCurve->GetDim() == 2, _T("SmBrepConstructor::SetEdgeCurve passed a non-2D UVTrimCurve")) ;
                }
#endif // SM_DEBUG_CODE
        m_pEdgeuse->m_pUVTrimCurve = SM_CAST_PTR(SmBSplineCurve,pCurve);
        if(pCurve) { pCurve->SetOwner(m_pEdgeuse) ; }
        NER(m_pEdgeuse->m_pUVTrimCurve);
        if (m_pEdge->m_pCurve == NULL) {
            double dMaxDist = 0.0;
            SmBSplineCurve *p3DCurve = NULL ;
            const SmContext *pContext = m_pBrep->GetContext();
            pFace = SM_CAST_PTR(SmFace,pSurf->GetOwner());
            pSurf->SetOwner(NULL); // To prevent trimmed surface cache
            SER(pSurf->LiftCurve(*pContext,
                                  pFace->GetUVDomain(),
                                 *m_pEdgeuse->m_pUVTrimCurve,
                                  pCurve->GetNaturalInterval(),
                                  SM_CAST_APPROXTOL3D(m_pEdge->GetTolerance()),
                                  dMaxDist,
                                  p3DCurve,
                                  TRUE)) ;

            pSurf->SetOwner(pFace);
            NER(p3DCurve);
            m_pEdge->m_pCurve = p3DCurve;
            p3DCurve->SetOwner(m_pEdge);

#ifdef SM_DEBUG_CODE
            smgfx_SetLook(3, 4, 1, 0, 0);  p3DCurve->Draw(); sm_GraphicsLoop();
#endif
        }
    }

    return SM_SUCCESS;

} // end SmBrepConstructor::SetEdgeCurve

/*******************************************************************//**
PURPOSE: 

NOTES:
***********************************************************************/
SmVertex * SmBrepConstructor::StartVertex
  (const SmPoint3d & crPoint)
{
    SmVertex * pV = new (m_pBrep) SmVertex();
    m_pBrep->m_pVertexListHead->PostInsert(pV);

    SM_OLDTOL_LINE pV->m_sZoneTol3d = (m_pBrep->GetTolerance());

    //m_sEdgeToVertex.SetAt(m_pEdge, pV);
    pV->SetPoint(crPoint);

    m_pVertex = pV;

    return pV;
} // end SmBrepConstructor::StartVertex

/*******************************************************************//**
PURPOSE: 

NOTES:
***********************************************************************/
SmStatus  SmBrepConstructor::EndVertex ()
{
    m_pVertex = NULL;

    return SM_SUCCESS;
} // end SmBrepConstructor::EndVertex

/*******************************************************************//**
PURPOSE: Write an attribute to the stream

NOTES:
***********************************************************************/
SmStatus SmBrepData::WriteAttributeToDB
  (const SmAttribute * pAttribute,    // in : target attribute
   SmDatabaseIO      & rDB)           // in : I/O data
{
  // no work - skip attributes of type SM_AB_TEMP
  if(pAttribute->GetBehavior() == SM_AB_TEMP)
    { return(SM_SUCCESS) ;
    }

  SmFileType eType = rDB.GetFileType();

  ULONG lNumLong   = pAttribute->GetNumLongElements();
  ULONG lNumDouble = pAttribute->GetNumDoubleElements();
  ULONG lNumChar   = pAttribute->GetNumCharacterElements();

  const long   * pLong         = pAttribute->GetLongElementsAddress();
  const double * pDouble       = pAttribute->GetDoubleElementsAddress();
  const char   * pChar         = pAttribute->GetCharacterElementsAddress();
  ULONG          lAttrID       = pAttribute->GetAttributeID();
  long           lAttrBehavior = (long)pAttribute->GetBehavior();

  if (eType == SM_ASCII)
    {
      std::ostream & rFileOut = *rDB.GetOutStreamPtr();
      rFileOut << lAttrID << " " << lAttrBehavior << " " <<
          lNumLong << " " << lNumDouble << " " << lNumChar << "\n";

      // output the longs
      for(ULONG i=0; i<lNumLong; i++)   { rFileOut << pLong[i] << " ";
                                        }
      if(lNumLong > 0)                  { rFileOut << "\n";
                                        }

      // output the doubles
      for(ULONG k=0; k<lNumDouble; k++) { rFileOut << pDouble[k] << " ";
                                        }
      if(lNumDouble > 0)                { rFileOut << "\n";
                                        }

      // output the characters
      for(ULONG j=0; j<lNumChar; j++)   { rFileOut << pChar[j];
                                        }
      if(lNumChar > 0)                  { rFileOut << "\n";
                                        }
    }
  else
    {// Binary
      SER(rDB.WriteLong(lAttrID));
      SER(rDB.WriteLong(lAttrBehavior));
      SER(rDB.WriteLong(lNumLong));
      SER(rDB.WriteLong(lNumDouble));
      SER(rDB.WriteLong(lNumChar));

      if (lNumLong   > 0) { SER(rDB.WriteLongs((ULONG*)pLong,lNumLong));
                          }
      if (lNumDouble > 0) { SER(rDB.WriteDoubles(pDouble,lNumDouble));
                          }
      if (lNumChar   > 0) { SER(rDB.WriteCharacters(pChar,lNumChar));
                          }
    }
  return SM_SUCCESS;

} // end SmBrepData::WriteAttributeToDB

/*******************************************************************//**
PURPOSE: Read an attribute from the stream

NOTES:
***********************************************************************/
SmStatus SmBrepData::ReadAttributeFromDB
  (const SmContext & crContext,       // in : context for new object construction
   SmAttribute    *& rpNewAttribute,  // out: read attribute
   SmDatabaseIO    & rDB)             // in : contains target stream
{
  SmFileType eType = rDB.GetFileType();

  long sLData[MAXSIZE];
  SmTArray<long> sLongs(MAXSIZE,sLData);
  double sDData[MAXSIZE];
  SmTArray<double> sDoubles(MAXSIZE,sDData);
  char sCData[MAXSIZE];
  SmTArray<char> sChars(MAXSIZE,sCData);
  ULONG lAttrID, lNumLong, lNumDouble, lNumChar;
  long lAttrBehavior = 0;

  if (eType == SM_ASCII)
    {
      std::istream & rFileIn = *rDB.GetInStreamPtr();
      rFileIn >> lAttrID >> lAttrBehavior >> lNumLong >> lNumDouble >> lNumChar;
      GOTO_NEXT_LINE;
      sLongs.SetSize(lNumLong);
      sDoubles.SetSize(lNumDouble);
      sChars.SetSize(lNumChar);

      // read the longs
      for(ULONG i=0; i<lNumLong; i++)   { long lLong = 0;
                                          rFileIn >> lLong;
                                          sLongs[i] = lLong;
                                        }
      if(lNumLong > 0)                  { GOTO_NEXT_LINE;
                                        }
      // read the doubles
      for(ULONG k=0; k<lNumDouble; k++) { double dDouble = 0.0;
                                          rFileIn >> dDouble;
                                          sDoubles[k] = dDouble;
                                        }
      if(lNumDouble > 0)                { GOTO_NEXT_LINE;
                                        }
      // read the characters

      for(ULONG j=0; j<lNumChar; j++)   { char cChar = '0';
                                          rFileIn.get(cChar);
                                          sChars[j] = cChar;
                                        }
      if(lNumChar > 0)                  { GOTO_NEXT_LINE;
                                        }
    }
  else
    {// Binary
      SER(rDB.ReadLong(lAttrID));
      SER(rDB.ReadLong(lAttrBehavior));
      SER(rDB.ReadLong(lNumLong));
      SER(rDB.ReadLong(lNumDouble));
      SER(rDB.ReadLong(lNumChar));

      sLongs.SetSize(lNumLong);
      sDoubles.SetSize(lNumDouble);
      sChars.SetSize(lNumChar);

      if (lNumLong   > 0) { SER(rDB.ReadLongs((ULONG*)sLongs.GetDataArray(),lNumLong));
                          }
      if (lNumDouble > 0) { SER(rDB.ReadDoubles(sDoubles.GetDataArray(),lNumDouble));
                          }
      if (lNumChar   > 0) { SER(rDB.ReadCharacters(sChars.GetDataArray(),lNumChar));
                          }
    }

  rpNewAttribute = SmAttribute::CreateAttribute(crContext,lAttrID,
                                                (SmAttributeBehaviorType)lAttrBehavior,
                                                sLongs,sDoubles,sChars);

  return SM_SUCCESS;

} // end SmBrepData::ReadAttributeFromDB

/*******************************************************************//**
PURPOSE: Read a part using the database corresponding to rDB.

NOTES:
***********************************************************************/
SmStatus SmBrepData::ReadPartFromDB
  (const SmContext      & crContext,         // in : context for constructing objects from stream
   SmTArray<SmCurve*>   & r3DCurves,         // out: Array of curves to get from stream
   SmTArray<SmSurface*> & rSurfaces,         // out: Array of surfaces to get from stream
   SmTArray<long>       & rBooleanTreeNodes, // out: Boolean Trees to get from stream (see SmMerge::BooleanTreeNodes)
   SmTArray<SmBrep*>    & rBreps,            // out: Array of SmBrep objects to get from stream
   SmDatabaseIO         & rDB)               // in : contains target stream

{
    SmFileType eType = rDB.GetFileType();

//  SmContext *pContext = SM_CONST_CAST(SmContext*,&crContext);
    ULONG lDBVersionNumber = 30;
    ULONG lHealerVersion   = 0;

    SmTArray<SmAttribute*> sAttributes;

    SER(rDB.BeginReading());

    ULONG lTotalCurves = 0;
    ULONG lTotalSurfaces = 0;
    ULONG lTotalBreps = 0;
    ULONG lTotalTreeNodes = 0;
    ULONG lIndex = 0;

    // read counts for stand-alone curves 
    char sBuff[MAXSIZE];
    if (eType == SM_ASCII)
      {
        std::istream & fin = *rDB.GetInStreamPtr();
        fin.getline(sBuff,MAXSIZE);
        SSCANF(sBuff, "//[Output Summary]\n");
        fin.getline(sBuff,MAXSIZE);
        SSCANF(sBuff,"//   (1) %ld Curves\n",&lTotalCurves);
      }
    else
      {// SM_BINARY
        SER(rDB.ReadLong(lTotalCurves));
      }

    // extract version number embedded within the curve count
    if (lTotalCurves > 1000000)
      {
        lDBVersionNumber = lTotalCurves / 1000000;
        lTotalCurves     = lTotalCurves % 1000000;
        rDB.SetVersionNum( lDBVersionNumber );
      }
    else
     {
       WARN(_T("Database being read has no lDBVersionNumber")) ;
     }

    // read counts for stand-alone surfaces 
    if (eType == SM_ASCII)
      {
        std::istream & fin = *rDB.GetInStreamPtr();
        fin.getline(sBuff,MAXSIZE);
        SSCANF(sBuff,"//   (2) %ld Surfaces\n",&lTotalSurfaces);
      }
    else
      {// SM_BINARY
        if ( lDBVersionNumber >= 40 )
        {
            std::istream & fin = *rDB.GetInStreamPtr();
            fin.ignore( sizeof( long long ) - sizeof( long ) ); // for compatibility between Unix and Windows (different sized longs)
        }
            
        SER(rDB.ReadLong(lTotalSurfaces));
      }

    // extract Healer version number embedded within the Surface count
    if (lTotalSurfaces > 1000000)
      {
        lHealerVersion = lTotalSurfaces / 1000000;
        lTotalSurfaces = lTotalSurfaces % 1000000;
      }
    else
     {
       WARN(_T("Database being read has no lHealerVersion")) ;
     }

    // read counts for stand-alone boolean trees, and brep Models
    if (eType == SM_ASCII)
      {
        std::istream & fin = *rDB.GetInStreamPtr();
        fin.getline(sBuff,MAXSIZE);
        SSCANF(sBuff,"//   (3) %ld Boolean Tree Nodes\n",&lTotalTreeNodes);
        fin.getline(sBuff,MAXSIZE);
        SSCANF(sBuff,"//   (4) %ld Breps\n",&lTotalBreps);
      }
    else
      {// SM_BINARY
        SER(rDB.ReadLong(lTotalTreeNodes));
        SER(rDB.ReadLong(lTotalBreps));
      }

    // stand-alone curves

    // allocate memory for each stand-alone curve pointer
    r3DCurves.SetSize(lTotalCurves);

    // read each curve - no approximations
    for (ULONG i=0; i<lTotalCurves; i++)
      {
        if (eType == SM_ASCII)
          {
            std::istream & fin = *rDB.GetInStreamPtr();
            fin.getline(sBuff,MAXSIZE);
            fin.getline(sBuff,MAXSIZE);
            SSCANF(sBuff,"[Curve #%ld]\n",&lIndex);
          }
        // read the curve - no approximations
        SER(SmBrepData::ReadCurveFromDB(crContext,3,
                                        lDBVersionNumber,
                                        sAttributes,
                                        r3DCurves[i],
                                        rDB,
                                        TRUE));
      } // end iter every curve

    // stand-alone surfaces

    // allocate memory for each stand-alone surface pointer
    rSurfaces.SetSize(lTotalSurfaces);

    // read each surface - no approximations
    for (ULONG j=0; j<lTotalSurfaces; j++)
      {
        if (eType == SM_ASCII)
          {
            std::istream & fin = *rDB.GetInStreamPtr();
            fin.getline(sBuff,MAXSIZE);
            fin.getline(sBuff,MAXSIZE);
            SSCANF(sBuff,"[Surface #%ld]\n",&lIndex);
          }
        // read the surface - no approximations
        SER(SmBrepData::ReadSurfaceFromDB(crContext,
                                          lDBVersionNumber,sAttributes,
                                          rSurfaces[j],rDB,TRUE));
      } // end iter every curve

    // boolean trees

    // allocate a long for every boolean tree node
    rBooleanTreeNodes.SetSize(lTotalTreeNodes);

    if (eType == SM_ASCII)
      {
        // Read Line With title
        std::istream & fin = *rDB.GetInStreamPtr();
        fin.getline(sBuff,MAXSIZE);
        fin.getline(sBuff,MAXSIZE);
      }

    // read and store every boolean tree node
    for (ULONG m=0; m<lTotalTreeNodes; m++)
      {
        long lNodeValue;
        if (eType == SM_ASCII)
          {
            std::istream & fin = *rDB.GetInStreamPtr();
            fin.getline(sBuff,MAXSIZE);
            SSCANF(sBuff,"%ld",&lNodeValue);
          }
        else
          {
            SER(rDB.ReadLong(lNodeValue));
          }
        rBooleanTreeNodes[m] = lNodeValue;
      } // end iter every boolean tree node

    // ASCII white space
    if (eType == SM_ASCII) 
      {
        std::istream & fin = *rDB.GetInStreamPtr();
        fin.getline(sBuff,MAXSIZE);
        fin.getline(sBuff,MAXSIZE);
      }

    // Brep Models

    // allocate a Brep pointer for every Brep model to read
    rBreps.SetSize(lTotalBreps);

    // read every Brep model
    for (ULONG k=0; k<lTotalBreps; k++)
      {
        SmBrepData * pBrepData = NULL;
        if (eType == SM_ASCII)
          {
            std::istream & fin = *rDB.GetInStreamPtr();
            fin.getline(sBuff,MAXSIZE);
            fin.getline(sBuff,MAXSIZE);
            fin.getline(sBuff,MAXSIZE);
            SSCANF(sBuff,"[Brep #%ld]\n",&lIndex);
          }

        // allocate an empty and initialized SmBrep object
        rBreps[k] = new (crContext) SmBrep();

        // read the BrepData
        SER(SmBrepData::ReadFromDB(crContext,lDBVersionNumber,sAttributes,pBrepData,rDB,TRUE));

        // Set pBrepData Healer Version
        pBrepData->SetHealerVersion( lHealerVersion );

        // Build Brep Model from BrepData
        //   FALSE: do not force creation of uv trim curves.
        SER(rBreps[k]->MakeTopologyFromData(pBrepData,sAttributes,FALSE));

        // clean up
        SM_ASSERT(pBrepData != NULL) ; delete pBrepData ; pBrepData = NULL ;

      } // end iter every Brep model

    // Now read in attributes - note that the temporary attributes collected in the
    // sAttributes array will be replaced by the real attributes that are read in.
    // The temporary attributes are just place holders until the real attributes
    // get read.  They should be in the same order as the attributes are written
    // to the file.
    if (lDBVersionNumber > 30)
      {
        // read total attribute count
        ULONG lTotalAttributes;
        SmObjsDelete<SmAttribute *> sCleanAttributes( &sAttributes );
        if (eType == SM_ASCII)
          {
            std::istream & fin = *rDB.GetInStreamPtr();
            fin.getline(sBuff,MAXSIZE);
            fin.getline(sBuff,MAXSIZE);
            SSCANF(sBuff,"%ld // Total number of attributes\n",&lTotalAttributes);
          }
        else
          {// SM_BINARY
            SER(rDB.ReadLong(lTotalAttributes));
          }

        // read each attribute
        for (ULONG jjj=0; jjj<lTotalAttributes; jjj++)
          {
            SmAttribute *pAttribute;
            if (eType == SM_ASCII)
              {
                // clear ascii white space
                std::istream & fin = *rDB.GetInStreamPtr();
                fin.getline(sBuff,MAXSIZE);
                fin.getline(sBuff,MAXSIZE);
              }

            // read the attribute
            SER(SmBrepData::ReadAttributeFromDB(crContext,pAttribute,rDB));

            // Replace and update attribute in sAttributes array
            SmAttribute *pTempAttribute = sAttributes[jjj];

            // get every object using this attribute
            SmAObject *sData[32];
            SmTArray<SmAObject*> sUsers(32,sData);
            pTempAttribute->GetUsers(sUsers);
            ULONG lNumUsers = sUsers.GetSize();

            //User may have been deleted by heal sequence in MakeTopologyFromData
            if ( lNumUsers == 0 )
            { delete pAttribute; pAttribute = NULL; }
            else
            {
                // for every attribute user
                for ( ULONG kkk = 0; kkk < sUsers.GetSize(); kkk++ )
                {
                    // Substitute attribute
                    SmAObject * pUser = sUsers[kkk];
                    pUser->RemoveAttribute( pTempAttribute );
                    pUser->AddAttribute( pAttribute );
                } // end iter every attribute user
            }
          } // end iter every attribute
      } // end (lDBVersionNumber > 30) check

    SER(rDB.EndReading());

    return SM_SUCCESS;

} // end SmBrepData::ReadPartFromDB

/*******************************************************************//**
PURPOSE: Write an SMLib SmBrep Geometry translated into a SmBrepData
  object to specified stream.

NOTES: Internal function called by WritePartToFile after a stream
  has been opened for output.

***********************************************************************/
SmStatus SmBrepData::WritePartToDB
  (const SmTArray<SmCurve*>   & cr3DCurves,         // in : Array of curves to place in stream
   const SmTArray<SmSurface*> & crSurfaces,         // in : Array of surfaces to place in stream
   const SmTArray<long>       & crBooleanTreeNodes, // in : Boolean Trees to place in file (see SmMerge::BooleanTreeNodes)
   const SmTArray<SmBrep*>    & crBreps,            // in : Array of SmBrep objects to place in strea
   SmDatabaseIO               & rDB,                // in : contains target stream
   SmBoolean                    bWriteAsBSplines,   // in : TRUE = Approx nonBSplineGeometry and write out as BSplineShapes
                                                    //      FALSE= Write native formats for nonBSplines
                                                    //      default:[FALSE]
   SmApproxTol3d                sApproxTol3d)       // in : only used when bWriteAsBSplines==TRUE
                                                    //      when approximating geometry as BSplines for output.
                                                    //      0.0 = use m_sApproxTol3d, default:[0.0]
{
  SmFileType eType = rDB.GetFileType();

  ULONG lWriteDBVersionNumber = SM_CURRENT_DATABASE_VERSION;
  ULONG lWriteHealerNumber    = SM_HEALER_VERSION;

  // get context shared by input objects
  SmContext const *pContext =   (cr3DCurves.GetSize() > 0) ? cr3DCurves[0]->GetContext()
                              : (crSurfaces.GetSize() > 0) ? crSurfaces[0]->GetContext()
                              : (crBreps.GetSize()    > 0) ? crBreps[0]->GetContext()
                              : NULL ;
  SM_ASSERT(pContext != NULL) ;
  SmContext sContext;
  if(pContext == NULL) { pContext = &sContext ; }

  // set output counts - Embed version number into Curve Count
  ULONG lTotalCurves           = cr3DCurves.GetSize() + 1000000 * lWriteDBVersionNumber;
  ULONG lTotalSurfaces         = crSurfaces.GetSize() + 1000000 * lWriteHealerNumber;
  ULONG lTotalBreps            = crBreps.GetSize();
  ULONG lTotalBooleanTreeNodes = crBooleanTreeNodes.GetSize();

  // local attribute accumulation array and map
  SmTArray<SmAttribute*> sAllAttributes;
  SmMapTypeToType<SmAttribute *,ULONG> sAttrMap;

  // init writing
  SER(rDB.BeginWriting());

  // ASCII/BINARY branch
  if (eType == SM_ASCII)
    {
      std::ostream & fout = *rDB.GetOutStreamPtr();
      fout << "//[Output Summary]\n";
      fout << "//   (1) " << lTotalCurves           << " Curves\n";
      fout << "//   (2) " << lTotalSurfaces         << " Surfaces\n";
      fout << "//   (3) " << lTotalBooleanTreeNodes << " Boolean Tree Nodes\n";
      fout << "//   (4) " << lTotalBreps            << " Breps\n";
    }
  else // SM_BINARY
    {
      SER(rDB.WriteLong(lTotalCurves));
      SER(rDB.WriteLong(lTotalSurfaces));
      SER(rDB.WriteLong(lTotalBooleanTreeNodes));
      SER(rDB.WriteLong(lTotalBreps));
    }

  // for every curve
  for (ULONG i=0; i<cr3DCurves.GetSize(); i++)
    {
      if (eType == SM_ASCII) 
        { std::ostream & fout = *rDB.GetOutStreamPtr();
          fout << "\n[Curve #" << i << "]\n";
        }
      SmBrepData::WriteCurveToDB(*pContext, cr3DCurves[i],
                                  lWriteDBVersionNumber, sAllAttributes,
                                  sAttrMap, rDB,
                                  bWriteAsBSplines, sApproxTol3d, TRUE);
    }

  // for every surface
  for (ULONG j=0; j<crSurfaces.GetSize(); j++)
    {
      if (eType == SM_ASCII) 
        {
          std::ostream & fout = *rDB.GetOutStreamPtr();
          fout << "\n[Surface #" << j << "]\n";
        }
      SmBrepData::WriteSurfaceToDB(*pContext, crSurfaces[j],
                                   lWriteDBVersionNumber, sAllAttributes,
                                   sAttrMap, rDB,
                                   bWriteAsBSplines, sApproxTol3d, TRUE);
    }

  // add header for boolean trees
  if (eType == SM_ASCII)
    {
      std::ostream & fout = *rDB.GetOutStreamPtr();
      fout << "\n// Boolean Trees\n";
    }

  // for every boolean tree node
  for (ULONG m=0; m<lTotalBooleanTreeNodes; m++)
    {
      long lNodeValue = crBooleanTreeNodes[m];
      if (eType == SM_ASCII)
        {
          std::ostream & fout = *rDB.GetOutStreamPtr();
          switch (lNodeValue)
            {
              case -1:
                  fout << lNodeValue << "   // UNION\n";
                  break;
              case -2:
                  fout << lNodeValue << "   // SUBTRACT\n";
                  break;
              case -3:
                  fout << lNodeValue << "   // INTERSECT\n";
                  break;
              case -4:
                  fout << lNodeValue << "   // MERGE\n";
                  break;
              default:
                  fout << lNodeValue << "   // Brep #" << lNodeValue << "\n";
                  break;
            }
        }
      else // SM_BINARY
        {
          std::ostream & fout = *rDB.GetOutStreamPtr();
          SER(rDB.WriteLong(lNodeValue));
          fout.write((char*)&lNodeValue, sizeof(long long));
        }
    } // end iter every boolean tree node

  if (eType == SM_ASCII)
    {
      std::ostream & fout = *rDB.GetOutStreamPtr();
      fout << "\n\n";
    }

  // for every Brep
  for (ULONG k=0; k<lTotalBreps; k++)
    {
      // output Brep index value
      if (eType == SM_ASCII)
        {
          std::ostream & fout = *rDB.GetOutStreamPtr();
          fout << "\n\n[Brep #" << k << "]############################################################################\n";
        }

      // build a SmBrepData for this SmBrep
      SmBrepData * pBrepData = new (*pContext) SmBrepData(FALSE, SM_DS_SMLIB); NER(pBrepData);
      SmObjDelete sClean(pBrepData);
      pBrepData->FromBrep(*crBreps[k],sAllAttributes);

      // output the SmBrepData to stream
      SER(pBrepData->WriteToDB(*pContext, lWriteDBVersionNumber,
                                sAllAttributes, sAttrMap,
                                rDB,
                                bWriteAsBSplines, sApproxTol3d, TRUE));

    } // end iter every SmBrep

  // Now write out attributes collected during writing process

  ULONG lTotalAttributes = sAllAttributes.GetSize();

  // output total number of attributes
  if (eType == SM_ASCII)
    {
      std::ostream & fout = *rDB.GetOutStreamPtr();
      fout << "############################################################################\n";
      fout << lTotalAttributes << " // Total number of attributes\n";
    }
  else
    {
      SER(rDB.WriteLong(lTotalAttributes));
    }

  // for every attribute
  for (ULONG kk=0; kk<sAllAttributes.GetSize(); kk++)
    {
      // output attribute index
      if (eType == SM_ASCII)
        {
          std::ostream & fout = *rDB.GetOutStreamPtr();
          fout << "\nAttribute #" << kk << "\n";
        }

      // output the attribute
      SmBrepData::WriteAttributeToDB(sAllAttributes[kk],rDB);
    }

  // all done
  SER(rDB.EndWriting());

  return SM_SUCCESS;

} // end SmBrepData::WritePartToDB

/*******************************************************************//**
PURPOSE: Write SmGeometry out to a file.
    This may include any combination of the following:

    A list of curves   (not contained within an SmBrep structure)
    A list of surfaces (not contained within an SmBrep Structure)
    A Boolean Tree node list output as a list of integer values.
    A list of Brep objects.

NOTES:
    See notes SmBrepData::WritePartToFile (TCHAR version)
    This is a pass through function for standard strings

***********************************************************************/
SmStatus SmBrepData::WritePartToFile
  (const std::string          & cOutputFileName,    // in : target File name
   const SmTArray<SmCurve*>   & cr3DCurves,         // in : Array of curves to place in file
   const SmTArray<SmSurface*> & crSurfaces,         // in : Array of surfaces to place in file
   const SmTArray<long>       & crBooleanTreeNodes, // in : Boolean Trees to place in file (see SmMerge::BooleanTreeNodes)
   const SmTArray<SmBrep*>    & crBreps,            // in : Array of SmBrep objects to place in file
   SmFileType                   eType,              // in : Specify output type: oneof
                                                    //       SM_ASCII  = Database is an ASCII file
                                                    //       SM_BINARY = Database is a Binary format
   SmBoolean                    bNewFile,           // in : TRUE = open file and rewrite contents
                                                    //      FALSE= open file and append to end
                                                    //      default:[FALSE]
   SmBoolean                    bWriteAsBSplines,   // in : TRUE = Approx nonBSplineGeometry and write out as BSplineShapes
                                                    //      FALSE= Write native formats for nonBSplines
                                                    //      default:[FALSE]
   SmApproxTol3d                sApproxTol3d)       // in : only used when bWriteAsBSplines is TRUE      
                                                    //      default:[SM_APPROX_TOL_3D = SM_ZONE_TOL_3D/2 = 5.0e-6]
{
    // convert to TCHAR and then call existing SmBrepData::WriteToFile
#ifdef UNICODE
    std::wstring wPath(cOutputFileName.begin(), cOutputFileName.end());
    return WritePartToFile(wPath.c_str(), cr3DCurves, crSurfaces, crBooleanTreeNodes, crBreps, eType, bNewFile, bWriteAsBSplines, sApproxTol3d);
#else
    return WritePartToFile(cOutputFileName.c_str(), cr3DCurves, crSurfaces, crBooleanTreeNodes, crBreps, eType, bNewFile, bWriteAsBSplines, sApproxTol3d);
#endif

}  // end SmBrepData::WritePartToFile

/*******************************************************************//**
PURPOSE: Write SmGeometry out to a file.
    This may include any combination of the following:

    A list of curves   (not contained within an SmBrep structure)
    A list of surfaces (not contained within an SmBrep Structure)
    A Boolean Tree node list output as a list of integer values.
    A list of Brep objects.

NOTES:
    1. Any updates after Release 3.1 have a version number
       embedded in the lTotalCurves.  This will allow us to change things
       in the rest of the file and maintain backward compatability.

    2. Any file version after 3.0 write attributes associated with the
       curves, surfaces, and Breps to file.

    3. A boolean Tree is a CSG tree of boolean operations
       and leaf node Brep objects that specify how to combine the leaf nodes
       into a single Brep model.  This tree is represented as a linear
       list of boolean operation values and Brep indices in post fix notation.
       Boolean Tree token values are oneof
         -1                     = Union
         -2                     = Intersect
         -3                     = Subtract
         -4                     = Merge
          0 and Positive values = Brep leaf node object indices

       Post Fix Notation is read from right to left.

Example:
       crBooleanTreeNodes = { 0 1 -1 2 -3 3 4 -4 -2 }
       represents the CSG Tree:

                         INTERSECT
                         INTERSECT
                       /           \
                SUBTRACT            MERGE
                /     \           /       \
             UNION  rBreps[2]  rBreps[3]  rBreps[4]
             /    \
       rBreps[0]  rBreps[1]

       Where 0-4 are indices into an array of Breps.
***********************************************************************/
SmStatus SmBrepData::WritePartToFile
  (const TCHAR                * cOutputFileName,    // in : target File name
   const SmTArray<SmCurve*>   & cr3DCurves,         // in : Array of curves to place in file
   const SmTArray<SmSurface*> & crSurfaces,         // in : Array of surfaces to place in file
   const SmTArray<long>       & crBooleanTreeNodes, // in : Boolean Trees to place in file (see SmMerge::BooleanTreeNodes)
   const SmTArray<SmBrep*>    & crBreps,            // in : Array of SmBrep objects to place in file
   SmFileType                   eType,              // in : Specify output type: oneof
                                                    //       SM_ASCII  = Database is an ASCII file
                                                    //       SM_BINARY = Database is a Binary format
   SmBoolean                    bNewFile,           // in : TRUE = open file and rewrite contents
                                                    //      FALSE= open file and append to end
                                                    //      default:[FALSE]
   SmBoolean                    bWriteAsBSplines,   // in : TRUE = Approx nonBSplineGeometry and write out as BSplineShapes
                                                    //      FALSE= Write native formats for nonBSplines
                                                    //      default:[FALSE]
   SmApproxTol3d                sApproxTol3d)       // in : only used when bWriteAsBSplines is TRUE      
                                                    //      default:[SM_APPROX_TOL_3D = SM_ZONE_TOL_3D/2 = 5.0e-6]
{
  SmDatabaseIOFile sDB ;
  TCHAR sBuff[SM_TBLOCK_SIZE] ;

  // create the databaseIO object
  if(SM_SUCCESS != sDB.OpenFileForWrite(cOutputFileName, // in : target file name
                                        eType,           // in : oneof: SM_ASCII    = write ascii file with comment lines
                                                         //             SM_BINARY   = write binary file from system format to LittleEndian
                                                         //             SM_BYTESWAP = write binary file - forcing byte swap
                                                         //      default:[SM_ASCII]
                                        bNewFile))       // in : bNewFile = TRUE  = open file and rewrite contents
                                                         //                 FALSE = open file and append to end
                                                         //                 default:[FALSE]
    {
      smos_sprintf(sBuff,_T("  *** ERROR File not found - %s\n"),cOutputFileName);
      smos_WriteBuffer(sBuff);
      return SM_ERR;
    }

  // pass the call along
  SER(SmBrepData::WritePartToDB(cr3DCurves, crSurfaces, crBooleanTreeNodes, crBreps,
                                sDB, bWriteAsBSplines, sApproxTol3d));

  // all done - close up and return
  return SM_SUCCESS;

} // end SmBrepData::WritePartToFile

/*******************************************************************//**
PURPOSE:  Read what constitutes a part from a file and populate the
    input arrays.

NOTES:
***********************************************************************/
SmStatus SmBrepData::ReadPartFromFile
  (const SmContext      & crContext,         // in : memory context for constructing objects from file
   const TCHAR          * cInputFileName,    // in : target file to read
   SmTArray<SmCurve*>   & r3DCurves,         // out: array of curves in file
   SmTArray<SmSurface*> & rSurfaces,         // out: array of surfaces in file
   SmTArray<long>       & rBooleanTreeNodes, // out: array of boolean trees in file
   SmTArray<SmBrep*>    & rBreps,            // out: array of Brep models
   SmFileType             eType)             // in : oneof SM_ASCII, SM_BINARY, default:[SM_ASCII]

{
  SmDatabaseIOFile sDB;
  TCHAR sBuff[SM_TBLOCK_SIZE] ;

  // create databaseIO for operation - stream is closed when sDB is destructed
  if(SM_SUCCESS != sDB.OpenFileForRead( cInputFileName, eType))
    {
      smos_sprintf(sBuff,_T("  *** ERROR File not found - %s\n"),cInputFileName);
      smos_WriteBuffer(sBuff);
      return SM_ERR;
    }

  // pass the call along
  SER(SmBrepData::ReadPartFromDB(crContext,r3DCurves,rSurfaces,rBooleanTreeNodes,rBreps,sDB));

  // all done
  return SM_SUCCESS;

} // end SmBrepData::ReadPartFromFile

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
SmStatus SmBrepData::WriteAssemblyToDB(
    const SmAssembly & crAssembly,       ///< [in] : Root SmAssembly of the assemlby tree to place in stream           <br>
    SmDatabaseIO     & rDB,              ///< [in] : contains target stream                                            <br>
    SmBoolean          bWriteAsBSplines, ///< [in] : TRUE = Approx nonBSplineGeometry and write out as BSplineShapes   <br>
                                         ///<        FALSE= Write native formats for nonBSplines                       <br>
    SmApproxTol3d      sApproxTol3d      ///< [in] : only used when bWriteAsBSplines==TRUE                             <br>
                                         ///<        when approximating geometry as BSplines for output.               <br>
                                         ///<         0.0 = use m_sApproxTol3d, default:[0.0]                          <br>
)
{
    SM_REF3(rDB, bWriteAsBSplines, sApproxTol3d);

    //SmFileType eType = rDB.GetFileType();

    //ULONG lWriteDBVersionNumber = SM_CURRENT_DATABASE_VERSION;
    //ULONG lWriteHealerNumber = SM_HEALER_VERSION;

    // get context
    const SmContext * cpContext = crAssembly.GetContext();
    SM_ASSERT(cpContext != NULL);


    // 1) Get all Objects
    // 1a) Breps
    // 1b) Assemblies
    // 2) All Breps and Assemblies get a number (index in list).
    // 2a) one list or 2?
    // 2b) Breps start at 0, Assemblies start at 100000?
    // 3) Assemblies written w/ list of AIs
    // 4) write AI w/transform + index of component

    return SM_SUCCESS;
} // end SmBrepData::WriteAssemblyToDB

/*******************************************************************//**
PURPOSE: 

NOTES:
***********************************************************************/
SmStatus SmBrepData::ReadAssemblyFromDB
(
    const SmContext  & crContext,            ///< [in] : context for constructing objects from stream                         <br>
    SmAssembly      *& crBreps,              ///< [out]: Root SmAssembly of the assembly tree                                 <br>
    SmDatabaseIO     & rDB                   ///< [in] : contains target stream                                               <br>
)
{
    SM_REF3(crContext, crBreps, rDB);

    return SM_SUCCESS;
} // end SmBrepData::ReadAssemblyFromDB

/*******************************************************************//**
PURPOSE: 

NOTES:
***********************************************************************/
SmStatus SmBrepData::ReadAssemblyFromFile
(
  const SmContext & crContext,          ///< [in] : context for new object construction     <br>
  const TCHAR     & crInputFileName,    ///< [in] : target file to read                     <br>
  SmAssembly     *& rpNewAssembly,      ///< [out]: Root of assembly tree                   <br>
  SmFileType        eType               ///< [in] : one of SM_ASCII or SM_BINARY            <br>
)
{
  SmDatabaseIOFile sDB;
  TCHAR sBuff[SM_TBLOCK_SIZE] ;

  // create databaseIO for operation - stream is closed when sDB is destructed
  if(SM_SUCCESS != sDB.OpenFileForRead( &crInputFileName, eType))
    {
      smos_sprintf(sBuff,_T("  *** ERROR File not found - %s\n"),&crInputFileName);
      smos_WriteBuffer(sBuff);
      return SM_ERR;
    }

  // pass the call along
  SER(SmBrepData::ReadAssemblyFromDB(crContext,rpNewAssembly,sDB));

  // all done
  return SM_SUCCESS;

} // end SmBrepData::ReadAssemblyFromFile

/*******************************************************************//**
PURPOSE: 

NOTES:
***********************************************************************/
SmStatus SmBrepData::WriteAssemblyToFile
(
  const TCHAR      & crOutputFileName,  ///< [in] : target file name                                                <br>
  const SmAssembly & crAssembly,        ///< [in] : Root SmAssembly of the assemlby tree to place in file           <br>
  SmFileType         eType,             ///< [in] : oneof SM_ASCII or SM_BINARY                                     <br>
  SmBoolean          bNewFile,          ///< [in] : TRUE = open file and rewrite contents                           <br>
                                        //      FALSE= open file and append to end                                  <br>
  SmBoolean          bWriteAsBSplines,  ///< [in] : TRUE = Approx nonBSplineGeometry and write out as BSplineShapes <br>
                                        //      FALSE= Write native formats for nonBSplines                         <br>
                                        //      default:[FALSE]                                                     <br>
  SmApproxTol3d      sApproxTol3d       ///< [in] : only used when bWriteAsBSplines==TRUE                           <br>
                                        //      when approximating geometry as BSplines for output.                 <br>
                                        //      0.0 = use m_sApproxTol3d, default:[0.0]                             <br>
)
{
  SmDatabaseIOFile sDB ;
  TCHAR sBuff[SM_TBLOCK_SIZE] ;

  // create the databaseIO object
  if(SM_SUCCESS != sDB.OpenFileForWrite(&crOutputFileName, // in : target file name
                                        eType,             // in : oneof: SM_ASCII    = write ascii file with comment lines
                                                           //             SM_BINARY   = write binary file from system format to LittleEndian
                                                           //             SM_BYTESWAP = write binary file - forcing byte swap
                                                           //      default:[SM_ASCII]
                                        bNewFile))         // in : bNewFile = TRUE  = open file and rewrite contents
                                                           //                 FALSE = open file and append to end
                                                           //                 default:[FALSE]
    {
      smos_sprintf(sBuff,_T("  *** ERROR File not found - %s\n"),&crOutputFileName);
      smos_WriteBuffer(sBuff);
      return SM_ERR;
    }

  // pass the call along
  SER(SmBrepData::WriteAssemblyToDB( crAssembly, sDB, bWriteAsBSplines, sApproxTol3d));

  // all done - close up and return
  return SM_SUCCESS;

} // end SmBrepData::WriteAssemblyToFile

/*******************************************************************//**
PURPOSE: SmBrepData constructor

NOTES:
***********************************************************************/
SmBrepData::SmBrepData
 (SmBoolean        bManifold,      // in : default:[FALSE]
  SmDataSourceType eDSType)        // in : default:[SM_DS_UNKNOWN]
: m_eDataSourceType(eDSType),
  m_bManifold      (bManifold),

#ifdef SM_USE_NEWTOL
  SM_NEWTOL_LINE m_dThisModelSizeEstimate(SmTol::GetModelSizeEstimate()),
  SM_NEWTOL_LINE m_dThisLargeSmallSizeRatio(SmTol::GetLargeSmallSizeRatio()),
#else // SM_USE_OLDTOL
  SM_OLDTOL_LINE m_sZoneTol3d(SM_ZONE_TOL_3D),  // GWC:NewTolerance - to be changed to SM_UNDEF_DOUBLE???
#endif // SM_USE_OLDTOL

  m_dThisModelSizeEstimate  (SM_MODEL_SIZE_ESTIMATE   ),  
  m_dThisLargeSmallSizeRatio(SM_LARGE_SMALL_SIZE_RATIO),
  m_sApproxTol3d   (SM_APPROX_TOL_3D),
  m_lStartRegion   (0),
  m_lNumRegions    (0),
  m_vColor         (0.0,0.0,0.0),
  m_bIsGeomBorrowed(FALSE),
  m_lHealerVersion(0),

  m_vRegions (*(new (*GetContext()) SmTArray<SmRegionData>(*GetContext()))),
  m_vShells  (*(new (*GetContext()) SmTArray<SmShellData>(*GetContext()))),
  m_vCFaces  (*(new (*GetContext()) SmTArray<ULONG>(*GetContext()))),

  m_vFaceuses(*(new (*GetContext()) SmTArray<SmFaceuseData>(*GetContext()))),
  m_vFaces   (*(new (*GetContext()) SmTArray<SmFaceData>(*GetContext()))),
  m_vLoops   (*(new (*GetContext()) SmTArray<SmLoopData>(*GetContext()))),

  m_vCEdges  (*(new (*GetContext()) SmTArray<ULONG>(*GetContext()))),
  m_vEdgeuses(*(new (*GetContext()) SmTArray<SmEUData>(*GetContext()))),
  m_vEdges   (*(new (*GetContext()) SmTArray<SmEdgeData>(*GetContext()))),
  m_vVertices(*(new (*GetContext()) SmTArray<SmVertexData>(*GetContext()))),

  m_vUVCurves(*(new (*GetContext()) SmTArray<SmCurve*>(*GetContext()))),
  m_v3DCurves(*(new (*GetContext()) SmTArray<SmCurve*>(*GetContext()))),
  m_vSurfaces(*(new (*GetContext()) SmTArray<SmSurface*>(*GetContext()))),
  m_vAttributes(*(new (*GetContext()) SmTArray<ULONG>(*GetContext())))
{

} // end SmBrepData::SmBrepData constructor

/*******************************************************************//**
PURPOSE: Reset an SmBrepData object back to the empty just-constructed state

NOTES:
***********************************************************************/
void SmBrepData::ReSet()
{
  // member data
  m_eDataSourceType          = SM_DS_UNKNOWN ;
  m_bManifold                = FALSE ;
  m_sZoneTol3d               = SM_ZONE_TOL_3D ;
  m_dThisModelSizeEstimate   = SM_MODEL_SIZE_ESTIMATE;    
  m_dThisLargeSmallSizeRatio = SM_LARGE_SMALL_SIZE_RATIO;
  m_sApproxTol3d             = SM_APPROX_TOL_3D ;
  m_lStartRegion             = 0 ;
  m_lNumRegions              = 0 ;
  m_vColor.Set(0.0,0.0,0.0) ;
  m_bIsGeomBorrowed          = FALSE ;
  m_lHealerVersion           = 0 ;

  // Array data
  m_vRegions.ReSet() ;
  m_vShells .ReSet() ;
  m_vCFaces .ReSet() ;

  m_vFaceuses.ReSet() ;
  m_vFaces   .ReSet() ;
  m_vLoops   .ReSet() ;

  m_vCEdges  .ReSet() ;
  m_vEdgeuses.ReSet() ;
  m_vEdges   .ReSet() ;
  m_vVertices.ReSet() ;

  m_vUVCurves.ReSet() ;
  m_v3DCurves.ReSet() ;
  m_vSurfaces.ReSet() ;
  m_vAttributes.ReSet() ;

} // end SmBrepData::ReSet

/*******************************************************************//**
PURPOSE: Destructor for the SmBrepData object

NOTES:
***********************************************************************/
SmBrepData::~SmBrepData()
{
  // locals
  ULONG ii ;

  // clean up memory within the items of the internal arrays
  for(ii=0;ii<m_vRegions.GetSize();ii++)  { m_vRegions[ii].m_sAttributes.SetSize(0) ; }
  for(ii=0;ii<m_vShells.GetSize();ii++)   { m_vShells[ii].m_sAttributes.SetSize(0) ; }
  for(ii=0;ii<m_vFaceuses.GetSize();ii++) { m_vFaceuses[ii].m_sAttributes.SetSize(0) ; }
  for(ii=0;ii<m_vFaces.GetSize();ii++)    { m_vFaces[ii].m_sAttributes.SetSize(0) ; }
  for(ii=0;ii<m_vLoops.GetSize();ii++)    { m_vLoops[ii].m_sAttributes.SetSize(0) ; }
  for(ii=0;ii<m_vEdgeuses.GetSize();ii++) { m_vEdgeuses[ii].m_sAttributes.SetSize(0) ; }
  for(ii=0;ii<m_vVertices.GetSize();ii++) { m_vVertices[ii].m_sAttributes.SetSize(0) ; }
  if(m_bIsGeomBorrowed == FALSE)
    { SmObjsDelete<SmCurve*>   sDelUVCurves(&m_vUVCurves) ;
      SmObjsDelete<SmCurve*>   sDel3DCurves(&m_v3DCurves) ;
      SmObjsDelete<SmSurface*> sDelSurfaces(&m_vSurfaces) ;
    } // end scope that forces all items within internal arrays to be deleted

  // delete the internal arrays
  delete &m_vRegions ;
  delete &m_vShells ;
  delete &m_vCFaces ;
  delete &m_vFaceuses ;
  delete &m_vFaces ;
  delete &m_vLoops ;
  delete &m_vCEdges ;
  delete &m_vEdgeuses ;
  delete &m_vEdges ;
  delete &m_vVertices ;
  delete &m_vUVCurves ;
  delete &m_v3DCurves ;
  delete &m_vSurfaces ;
  delete &m_vAttributes ;

} // end SmBrepData::~SmBrepData destructor

/*******************************************************************//**
PURPOSE:  Retrieves long value associated with input pointer
             (valueInMap-1) or returns SM_NO_OBJECT when input pointer
             is not in sMap.

NOTES:
  1. signals error when entry for void * a is not found.
  2. sMap stores objectIndex+1 values; this function returns objectIndex values.

***********************************************************************/
static ULONG sm_GetMap                      // eff: return long value associated with input, a
 (SmMapTypeToType<SmObject*, ULONG> & sMap, // in : map containing relationships to examine
  SmObject                          * pObj) // in : target search object
{
  // retrieve ObjectIndex+1 value from sMap associated with a
  ULONG lRet = sMap.GetValueAt(pObj);

  // when object was not found - signal error and return SM_NO_OBJECT.
  //   This correctly allows Edgeuses with no UVTrimCurves to be written.
  //   Otherwise entry into this path only happens for corrupted SmBrep models.
  //   Where some topology pointer value is set to NULL when it should be
  //   set to some SmTopology Object.
  //   Any such cases need to be studied to find and fix how the NULL value
  //   got this far.
  if (lRet == 0)  { TCHAR sBuff[SM_TBLOCK_SIZE] ;
                    smos_sprintf(sBuff,_T("Given Object [0x%p] not registered in ObjectList"),pObj);
                    SE_MSG(SM_ERR,sBuff);
                    return SM_NO_OBJECT;
                  }

  // else return associated value-1
  return lRet - 1;

} // end sm_GetMap

#define ADD_TO_MAP(pObj,lIndx) sMap.SetAt((pObj),((lIndx)+1))
#define GET_MAP(pObj)          sm_GetMap(sMap,(pObj))
#define HAS_MAP(pObj)          ((sMap.GetValueAt(pObj) == 0) ? (FALSE) : (TRUE))

/*******************************************************************//**
PURPOSE:  retrieves edgeuse index for an edgeuse pointer

NOTES: signals an error when neither the Map nor the MapMate
   maps do not contain an entry for this pEdgeuse pointer.

***********************************************************************/
static long sm_GetEdgeuseIndex
 (SmMapTypeToType<SmObject*,  ULONG> & sMap,       // in : map containing relationships to examine
  SmMapTypeToType<SmEdgeuse*, ULONG> & rEUMapMate, // in : map of mates containing relationships to examine
  SmEdgeuse                          * pEdgeuse)
{
  long lIndex = rEUMapMate.GetValueAt(pEdgeuse);
  if (lIndex != 0) { lIndex = -lIndex;  // negate lIndex
                   }
  else             { lIndex = GET_MAP(pEdgeuse);
                   }
  return lIndex;

} // end sm_GetEdgeuseIndex

/*******************************************************************//**
PURPOSE: create SmBrepData Object lists from SmBrep Topology Graph
    replace object pointers with object indices,
    replace attribute pointers with attribute indices
    and place all pointers on rAllAttributes list
   - mirror function = SmBrep::MakeTopologyFromData

NOTES:
  1. sets m_vColor Color to {1.0 0.0 0.0}
  2. When an attribute pointer is replaced by an index value it is placed
     on the rAllAttributes accumulation list so that it can be output
     to file at a later time.
***********************************************************************/
SmStatus SmBrepData::FromBrep
  (const SmBrep           & crBrep,          // in : target Brep
   SmTArray<SmAttribute*> & rAllAttributes)  // i/o: attribute accumulation array
{
  ULONG i ;
  SmMapTypeToType<SmObject*,    ULONG> sMap;       // map ObjectPointers to indices
  SmMapTypeToType<SmEdgeuse*,   ULONG> sEUMateMap; // [Edge->primaryEdgeUse->Mate,edgeuseIndex+1]  saves 0 as the not in array value
  SmMapTypeToType<SmAttribute*, ULONG> sAttrMap;   // map AttributePointers to indices

  // locals
  const SmContext *pContext = crBrep.GetContext();

  // init output
  ReSet() ;

  // set Brep properties
  m_eDataSourceType = SM_DS_SMLIB ; 
  m_bManifold       = crBrep.IsManifoldSolid();
// Remove Composites
//   m_bMakeComposites = crBrep.m_bMakeComposites;  // Does this Brep allow composites?

#ifdef SM_USE_NEWTOL
  SM_NEWTOL_LINE m_dThisModelSizeEstimate   = crBrep.GetThisModelSizeEstimate  (); // rwith lDBVersionNumber == 42
  SM_NEWTOL_LINE m_dThisLargeSmallSizeRatio = crBrep.GetThisLargeSmallSizeRatio(); // rwith lDBVersionNumber == 42
#else // SM_USE_OLDTOL
  SM_OLDTOL_LINE m_sZoneTol3d               = crBrep.GetTolerance();               // renamed from m_dTol with lDBVersionNumber == 38
#endif // SM_USE_OLDTOL

  m_lStartRegion    = 0;

  // Set the brep color if it exists. Otherwise default to red
  SmVector3dAttribute* sm_color = (SmVector3dAttribute *)crBrep.FindAttribute (SM_AI_COLOR);
  if(sm_color != NULL) { m_vColor.Set(sm_color->GetValue().x, sm_color->GetValue().y, sm_color->GetValue().z ); }
  else                 { m_vColor.Set(1.0,0.0,0.0); }

  // get Brep attributes (not attributes of objects in the Brep structure)
  SER(sm_ExtractAttributes(*pContext,        // in : current context for object creation
                           &crBrep,          // in : target object potentially containing attributes
                            m_vAttributes,   // out: rAttributes index array for this object's attributes
                            rAllAttributes,  // i/o: accumulation of all attributes on all entities
                            sAttrMap));      // i/o: accumulation of [attribute,ObjectAttributeIndexValue+1] pairs

  // Regions

  // get all regions
  SmTArray<SmRegion*> sRegions;
  crBrep.GetRegions(sRegions);

  // place infinite region at head of list - no output yet
  if(sRegions.GetSize() > 1 && sRegions[0] != crBrep.GetInfiniteRegion())
    { for (i=1; i<sRegions.GetSize(); i++)
        { if (sRegions[i] == crBrep.GetInfiniteRegion())
            { sRegions.RemoveAt(i);
              sRegions.InsertAt(0,crBrep.GetInfiniteRegion());
              break;
            }
        }
    }

  SmTArray<SmShell*> sAllShells;
  ULONG lTotalRegions = sRegions.GetSize();
  ULONG lTotalShells  = 0;
  m_lNumRegions       = lTotalRegions;

  // allocate a SmRegionData Object for each region
  m_vRegions.SetSize(lTotalRegions);

  // for every region
  for (i=0; i<lTotalRegions; i++)
    {
      SmRegion * pRegion = sRegions[i];

      // Add [region,RegionIndex+1] pair to sMap
      ADD_TO_MAP(pRegion,i);

      // get this region's shells
      SmTArray<SmShell*> sShells;
      pRegion->GetShells(sShells);
      ULONG lNumShells            = sShells.GetSize();

      // set this regionData values - for backward compatibility: map SM_UNDEF_ULONG flag and index values to zero
      m_vRegions[i].m_lStartShell = lTotalShells;   // First (outer) shell of this Region
      m_vRegions[i].m_lNumShells  = lNumShells;     // Number of shells in this region
      m_vRegions[i].m_bIsVoidFlag = pRegion->IsVoid();
      m_vRegions[i].m_lFlags      = (pRegion->GetAllFlags()   == SM_UNDEF_ULONG) ? 0 : pRegion->GetAllFlags() ;
      m_vRegions[i].m_lUserIndex1 = (pRegion->GetUserIndex1() == SM_UNDEF_ULONG) ? 0 : pRegion->GetUserIndex1() ;
      m_vRegions[i].m_lUserIndex2 = (pRegion->GetUserIndex2() == SM_UNDEF_ULONG) ? 0 : pRegion->GetUserIndex2() ;
      m_vRegions[i].m_pUserPtr1   = pRegion->GetUserPtr1();
      m_vRegions[i].m_pRegion     = pRegion;

      // get region's attribute indices
      SER(sm_ExtractAttributes(*pContext,                    // in : current context for object creation
                                pRegion,                     // in : target object potentially containing attributes
                                m_vRegions[i].m_sAttributes, // out: rAttributes index array for this object's attributes
                                rAllAttributes,              // i/o: accumulation of all attributes on all entities
                                sAttrMap));                  // i/o: accumulation of [attribute,ObjectAttributeIndexValue+1] pairs

      // accumulate all shells and their count
      lTotalShells += lNumShells;
      sAllShells.Append(sShells);

    } // end iter every region

  // Get all vertices
  SmTArray<SmVertex*> sAllVertices;
  crBrep.GetVertices(sAllVertices);
  ULONG lTotalVertices = sAllVertices.GetSize();

  // allocate an SmVertexData Object for each Vertex
  m_vVertices.SetSize(lTotalVertices);

  // for every vertex
  for (i=0; i<lTotalVertices; i++)
    {
      SmVertex * pVertex = sAllVertices[i];
      // add [vertex, vertexIndex+1] pair to sMap
      ADD_TO_MAP(pVertex,i);

      // set Vertex values - for backward compatibility: map SM_UNDEF_ULONG flag and index values to zero
      SM_OLDTOL_LINE m_vVertices[i].m_sZoneTol3d = pVertex->GetTolerance();

      m_vVertices[i].m_vPoint          = pVertex->GetPoint();
      m_vVertices[i].m_lFlags          = (pVertex->GetAllFlags()   == SM_UNDEF_ULONG) ? 0 : pVertex->GetAllFlags() ;
      m_vVertices[i].m_lUserIndex1     = (pVertex->GetUserIndex1() == SM_UNDEF_ULONG) ? 0 : pVertex->GetUserIndex1() ;
      m_vVertices[i].m_lUserIndex2     = (pVertex->GetUserIndex2() == SM_UNDEF_ULONG) ? 0 : pVertex->GetUserIndex2() ;
      m_vVertices[i].m_pUserPtr1       = pVertex->GetUserPtr1();
      m_vVertices[i].m_pVertex         = pVertex;

      // get vertex attribute indices
      SER(sm_ExtractAttributes(*pContext,                     // in : current context for object creation
                                pVertex,                      // in : target object potentially containing attributes
                                m_vVertices[i].m_sAttributes, // out: rAttributes index array for this object's attributes
                                rAllAttributes,               // i/o: accumulation of all attributes on all entities
                                sAttrMap));                   // i/o: accumulation of [attribute,ObjectAttributeIndexValue+1] pairs

    } // end iter every vertex

  // Get all Edges
  SmTArray<SmEdge*> sAllEdges;
  crBrep.GetEdges(sAllEdges);
  ULONG lTotalEdges    = sAllEdges.GetSize();
  ULONG lTotalUVCurves = 0;

  // For every Edge - add [edge,edgeIndex+1] pairs into sMap
  // For Wire Edges - add [Edge->primaryEdgeUse,      edgeuseIndex]   pairs into sMap
  //                  add [Edge->primaryEdgeUse->Mate,edgeuseIndex+1] pairs into sEUMateMap
  //                  add [Edge->PrimaryEdgeuse] to sAllEdgeuses array
  // For loop Edges - add [loop->TopLoopuse->Edgeuse_ii, edgeuseIndex]         pairs into sMap       for_ii loop->edgeuses{ii]
  //                  add [loop->TopLoopuse->Edgeuse_ii->Mate, edgeuseIndex+1] pairs into sEUMateMap for_ii loop->edgeuses{ii]
  SmTArray<SmEdgeuse*> sAllEdgeuses; // array of all TopFU->Edgeuses and wire->PrimEdgeuses, does not contain BotFU->Edgeuses
  ULONG lTotalEdgeuses = 0;          // 1 Edgeuse per wire + Loop->EdgeuseCnt per loop
  for (i=0; i<sAllEdges.GetSize(); i++)
    {
      SmEdge *pE = sAllEdges[i];

      // add [edge,edgeIndex+1] pair to sMap
      ADD_TO_MAP(pE,i);

      // For WireEdges; add WireEdge->PrimEdgeuse to sMap and WireEdge->PrimEdgeuse->Mate to sEUMateMap
      if (pE->IsWire())
        {
          SmEdgeuse *pPrimEU = pE->GetPrimaryEdgeuse();

          // add [edge->primaryEdgeuse,edgeuseIndex] pair to sMap
          ADD_TO_MAP(pPrimEU,sAllEdgeuses.GetSize());

          // add [edge->primaryEdgeuse->Mate,edgeuseIndex+1] pair to sEUMateMap
          sEUMateMap.SetAt(pPrimEU->GetMate(),sAllEdgeuses.GetSize()+1);
          sAllEdgeuses.Add(pPrimEU);
          lTotalEdgeuses ++;

        } // end isWire check
    } // end iter all edges looking for wires

  // Shells

  // allocate an SmShellData Object for each Shell
  m_vShells.SetSize(lTotalShells);

  // locals
  ULONG lTotalFaceuses = 0;
  SmTArray<SmFaceuse*> sAllFaceuses;

  // for every shell
  for (i=0; i<lTotalShells; i++)
    {
      SmShell * pShell = sAllShells[i];

      // add [shell,shellIndex+1] pair to sMap
      ADD_TO_MAP(pShell,i);

      // set shellData values for different shell types
      if (pShell->IsVertexShell())    { SmVertex * pV = pShell->GetVertex();
                                        m_vShells[i].m_lShellType = 2;
                                        m_vShells[i].m_lVertex    = GET_MAP(pV);
                                      }
      else if (pShell->IsWireShell()) { SmEdge * pE = pShell->GetWireEdge();
                                        m_vShells[i].m_lShellType = 1;
                                        m_vShells[i].m_lEdge      = GET_MAP(pE);
                                      }
      else                            { // Normal(Faceuse)
                                        m_vShells[i].m_lShellType   = 0;

                                        // get this shell's faceuses
                                        SmTArray<SmFaceuse*> sFaceuses;
                                        pShell->GetFaceuses(sFaceuses);
                                        m_vShells[i].m_lFaceuseStart = lTotalFaceuses;
                                        ULONG lNumFaceuses           = sFaceuses.GetSize();
                                        m_vShells[i].m_lNumFaceuses  = lNumFaceuses;

                                        // accumulate all faceuses
                                        lTotalFaceuses += lNumFaceuses;
                                        sAllFaceuses.Append(sFaceuses);
                                      }

      // set base class TopologyData values - for backward compatibility: map SM_UNDEF_ULONG flag and index values to zero
      m_vShells[i].m_lFlags      = (pShell->GetAllFlags()   == SM_UNDEF_ULONG) ? 0 : pShell->GetAllFlags() ;
      m_vShells[i].m_lUserIndex1 = (pShell->GetUserIndex1() == SM_UNDEF_ULONG) ? 0 : pShell->GetUserIndex1() ;
      m_vShells[i].m_lUserIndex2 = (pShell->GetUserIndex2() == SM_UNDEF_ULONG) ? 0 : pShell->GetUserIndex2() ;
      m_vShells[i].m_pUserPtr1   = pShell->GetUserPtr1();
      m_vShells[i].m_pShell1     = pShell;

      // get shell attribute indices
      SER(sm_ExtractAttributes(*pContext,                   // in : current context for object creation
                                pShell,                     // in : target object potentially containing attributes
                                m_vShells[i].m_sAttributes, // out: rAttributes index array for this object's attributes
                                rAllAttributes,             // i/o: accumulation of all attributes on all entities
                                sAttrMap));                 // i/o: accumulation of [attribute,ObjectAttributeIndexValue+1] pairs

    } // end iter every shell

  // Get all faces
  SmTArray<SmFace*> sAllFaces;
  crBrep.GetFaces(sAllFaces);
  ULONG lTotalFaces = sAllFaces.GetSize();
  for (i=0; i<sAllFaces.GetSize(); i++)
    {
      // add [face,faceIndex+1] pair to sMap
      ADD_TO_MAP(sAllFaces[i],i);
    }

  // Faceuses

  // allocate a SmFaceuseData Object for each faceuse
  m_vFaceuses.SetSize(lTotalFaceuses);

  // for every faceuse
  for (i=0; i<lTotalFaceuses; i++)
    {
      SmFaceuse * pFaceuse = sAllFaceuses[i];
      SmFace    * pFace    = pFaceuse->GetFace();

      // add [faceuse,faceuseIndex+1] pair to sMap
      ADD_TO_MAP(pFaceuse,i);

      // set faceuse data - for backward compatibility: map SM_UNDEF_ULONG flag and index values to zero
      m_vFaceuses[i].m_bOrientation =   (pFaceuse->GetOrientation() == SM_OT_SAME)
                                      ? TRUE
                                      : FALSE ;
      m_vFaceuses[i].m_lFace        = GET_MAP(pFace);
      m_vFaceuses[i].m_lFlags       = (pFaceuse->GetAllFlags()   == SM_UNDEF_ULONG) ? 0 : pFaceuse->GetAllFlags() ;
      m_vFaceuses[i].m_lUserIndex1  = (pFaceuse->GetUserIndex1() == SM_UNDEF_ULONG) ? 0 : pFaceuse->GetUserIndex1() ;
      m_vFaceuses[i].m_lUserIndex2  = (pFaceuse->GetUserIndex2() == SM_UNDEF_ULONG) ? 0 : pFaceuse->GetUserIndex2() ;
      m_vFaceuses[i].m_pUserPtr1    = pFaceuse->GetUserPtr1();

      // get faceuse attribute indices
      SER(sm_ExtractAttributes(*pContext,                     // in : current context for object creation
                                pFaceuse,                     // in : target object potentially containing attributes
                                m_vFaceuses[i].m_sAttributes, // out: rAttributes index array for this object's attributes
                                rAllAttributes,               // i/o: accumulation of all attributes on all entities
                                sAttrMap));                   // i/o: accumulation of [attribute,ObjectAttributeIndexValue+1] pairs

    } // end iter every faceuse

  // Get all surfaces
  crBrep.GetSurfaces(m_vSurfaces);
  ULONG lTotalSurfaces = m_vSurfaces.GetSize();
  for (i=0; i<lTotalSurfaces; i++)
    {
      // add [surface,surfaceIndex+1] pair to sMap
      ADD_TO_MAP(m_vSurfaces[i],i);
    }

  // Faces

  // allocate a SmFaceData Object for each Face
  m_vFaces.SetSize(lTotalFaces);

  // locals
  SmTArray<SmLoop*> sAllLoops;
  ULONG lTotalLoops = 0;

  // for every face
  for (i=0; i<lTotalFaces; i++)
    {
      SmFace    * pFace    = sAllFaces[i];
      SmSurface * pSurface = pFace->GetSurface();

      // get face loops (1 loop has 2 loopuses = {TopFU->Loopuse, BotFu->Loopuse})
      SmTArray<SmLoop*> sLoops;
      pFace->GetLoops(sLoops);
      ULONG lNumLoops = sLoops.GetSize();

      // set faceData values - for backward compatibility: map SM_UNDEF_ULONG flag and index values to zero
      m_vFaces[i].m_lSurface         = GET_MAP(pSurface);
      SM_OLDTOL_LINE m_vFaces[i].m_sZoneTol3d = pFace->GetTolerance();

      m_vFaces[i].m_bRectangularTrim = pFace->GetRectangularTrim();
      m_vFaces[i].m_vUVDomain        = pFace->GetUVDomain();
      m_vFaces[i].m_lStartLoop       = lTotalLoops;
      m_vFaces[i].m_lNumLoops        = lNumLoops;
      m_vFaces[i].m_lFlags           = (pFace->GetAllFlags()   == SM_UNDEF_ULONG) ? 0 : pFace->GetAllFlags() ;
      m_vFaces[i].m_lUserIndex1      = (pFace->GetUserIndex1() == SM_UNDEF_ULONG) ? 0 : pFace->GetUserIndex1() ;
      m_vFaces[i].m_lUserIndex2      = (pFace->GetUserIndex2() == SM_UNDEF_ULONG) ? 0 : pFace->GetUserIndex2() ;
      m_vFaces[i].m_pUserPtr1        = pFace->GetUserPtr1();
      m_vFaces[i].m_pFace            = pFace;

      // get faceuse attribute indices
      SER(sm_ExtractAttributes(*pContext,                  // in : current context for object creation
                                pFace,                     // in : target object potentially containing attributes
                                m_vFaces[i].m_sAttributes, // out: rAttributes index array for this object's attributes
                                rAllAttributes,            // i/o: accumulation of all attributes on all entities
                                sAttrMap));                // i/o: accumulation of [attribute,ObjectAttributeIndexValue+1] pairs

      // accumulate all the loops
      lTotalLoops += lNumLoops;
      sAllLoops.Append(sLoops);

    } // end iter all faces

  // Loops

  // allocate a SmLoopData Object for each Loop
  m_vLoops.SetSize(lTotalLoops);

  // for every loop
  for (i=0; i<lTotalLoops; i++)
    {
      SmLoop * pLoop = sAllLoops[i];

      // add [loop,loopIndex+1] pair to sMap
      ADD_TO_MAP(pLoop,i);

      // set pLUT1 = topFU->loopuse
      SmLoopuse *pLUT1, *pLUT2;
      pLoop->GetLoopuses(pLUT1,pLUT2);
      if (pLUT1->GetFaceuse()->GetOrientation() == SM_OT_OPPOSITE)
        {
          pLUT1 = pLUT2;
        }

      // set LoopData values based on LoopType (VertexLoopType and EdgeuseType)
      if (pLUT1->IsVertexLoopuse()) { // vertex loop
                                      SmTArray<SmVertex*> sSingleVertex;
                                      pLUT1->GetVertices(sSingleVertex);
                                      SmVertex * pVertex = sSingleVertex[0];
                                      m_vLoops[i].m_lLoopType = 1; // Vertex loop
                                      m_vLoops[i].m_lVertex   = GET_MAP(pVertex);
                                    }
      else                          { // Edgeuse loop
                                      SmTArray<SmEdgeuse*> sEdgeuses;
                                      pLUT1->GetEdgeuses(sEdgeuses);
                                      ULONG lNumEdgeuses = sEdgeuses.GetSize();

                                      m_vLoops[i].m_lLoopType = 0;
                                      m_vLoops[i].m_lStartEU  = lTotalEdgeuses;
                                      m_vLoops[i].m_lNumEU    = lNumEdgeuses;

                                      // accumulate edgeuses
                                      lTotalEdgeuses += lNumEdgeuses;
                                      for (ULONG lll=0; lll<sEdgeuses.GetSize(); lll++)
                                        {
                                          SmEdgeuse *pEU = sEdgeuses[lll];

                                          // add [Edgeuse,      EdgeuseIndex]   assoc to sMap
                                          // add [Edgeuse->Mate,EdgeuseIndex+1] assoc to sEUMateMap
                                          ADD_TO_MAP(pEU,sAllEdgeuses.GetSize());
                                          sEUMateMap.SetAt(pEU->GetMate(),sAllEdgeuses.GetSize()+1);
                                          sAllEdgeuses.Add(pEU);
                                        }
                                     }

      // set loop values - for backward compatibility: map SM_UNDEF_ULONG flag and index values to zero
      m_vLoops[i].m_lFlags      = (pLoop->GetAllFlags()   == SM_UNDEF_ULONG) ? 0 : pLoop->GetAllFlags() ;
      m_vLoops[i].m_lUserIndex1 = (pLoop->GetUserIndex1() == SM_UNDEF_ULONG) ? 0 : pLoop->GetUserIndex1() ;
      m_vLoops[i].m_lUserIndex2 = (pLoop->GetUserIndex2() == SM_UNDEF_ULONG) ? 0 : pLoop->GetUserIndex2() ;
      m_vLoops[i].m_pUserPtr1   = pLoop->GetUserPtr1();
      m_vLoops[i].m_pLoop       = pLoop;

      // get loop, upperLoopuse, and lowerLoopuse attribute indices
      SER(sm_ExtractAttributes(*pContext, pLoop, m_vLoops[i].m_sAttributes,        rAllAttributes, sAttrMap));
      SER(sm_ExtractAttributes(*pContext, pLUT1, m_vLoops[i].m_sLUUpperAttributes, rAllAttributes, sAttrMap));
      SER(sm_ExtractAttributes(*pContext, pLUT2, m_vLoops[i].m_sLULowerAttributes, rAllAttributes, sAttrMap));
    } // end iter every loop

  // Edgeuses

  // allocate a SmEUData Object for each TopFU->Loopuse->edgeuse and wire->PrimEdgeuse not mates
  m_vEdgeuses.SetSize(lTotalEdgeuses);

  // for every edgeuse (counting only TopFU->Loopuse->edgeuses and wire->PrimEdgeuses not mates)
  for (i=0; i<lTotalEdgeuses; i++)
    {
      SmEdgeuse * pEdgeuse = sAllEdgeuses[i];

        { // set m_lNextEu and m_lMateNextEU indices
          // negative index values are EUs in the sEUMateMap which are BotFU->edgeuses and wire->PrimEdgeuse->Mates
          // negative index values are incremented by 1
          //   m_lNextEU = -3 means the next Edgeuse is the 2nd EUPointer on Edgeuse2
          SmEdgeuse *pNext             = (SmEdgeuse*)pEdgeuse->GetNext(); NER(pNext);
          m_vEdgeuses[i].m_lNextEU     = sm_GetEdgeuseIndex(sMap,sEUMateMap,pNext);
          SmEdgeuse *pMate             = pEdgeuse->GetMate(); NER(pMate);
          SmEdgeuse *pMateNext         = (SmEdgeuse*)pMate->GetNext(); NER(pMateNext);
          m_vEdgeuses[i].m_lMateNextEU = sm_GetEdgeuseIndex(sMap,sEUMateMap,pMateNext);
        }

      SmEdge * pEdge = pEdgeuse->GetEdge();
      m_vEdgeuses[i].m_lEdge = GET_MAP(pEdge);

      // set EdgeuseData values based on Edgeuse type
      if (pEdgeuse->IsShellEdgeuse())
        { // SmShell_TYPE edgeuse connects to wireEdge
          m_vEdgeuses[i].m_lEUType      = 2;
          m_vEdgeuses[i].m_bOrientation =  (pEdgeuse->GetOrientation() == SM_OT_SAME)
                                          ? TRUE
                                          : FALSE ;
          SmShell * pShell              = pEdgeuse->GetShell();
          m_vEdgeuses[i].m_lShell       = GET_MAP(pShell);
        }
      else
        { // SmLoopuse_TYPE edgeuse connects an edge in a loop connecting edges to a face
          SmLoopuse      * pLoopuse     = pEdgeuse->GetLoopuse();
          SmLoop         * pLoop        = pLoopuse->GetLoop();
          SmBSplineCurve * pUVCurve     = pEdgeuse->GetUVTrimCurve();

          m_vEdgeuses[i].m_lEUType      = 0;
          m_vEdgeuses[i].m_lLoop        = GET_MAP(pLoop);
          m_vEdgeuses[i].m_bOrientation =  (pEdgeuse->GetOrientation() == SM_OT_SAME)
                                           ? TRUE
                                           : FALSE ;

          if (pUVCurve) { m_vUVCurves.Add((SmCurve*)pUVCurve);
                          m_vEdgeuses[i].m_lUVCurve = lTotalUVCurves++;
                        }
          else          { m_vEdgeuses[i].m_lUVCurve = SM_NO_OBJECT;
                        }
       }

      m_vEdgeuses[i].m_lFlags      = pEdgeuse->GetAllFlags();
      m_vEdgeuses[i].m_lUserIndex1 = pEdgeuse->GetUserIndex1();
      m_vEdgeuses[i].m_lUserIndex2 = pEdgeuse->GetUserIndex2();
      m_vEdgeuses[i].m_pUserPtr1   = pEdgeuse->GetUserPtr1();
      SER(sm_ExtractAttributes(*pContext,                     // in : current context for object creation
                                pEdgeuse,                     // in : target object potentially containing attributes
                                m_vEdgeuses[i].m_sAttributes, // out: rAttributes index array for this object's attributes
                                rAllAttributes,               // i/o: accumulation of all attributes on all entities
                                sAttrMap));                   // i/o: accumulation of [attribute,ObjectAttributeIndexValue+1] pairs

      m_vEdgeuses[i].m_lMateFlags      = pEdgeuse->GetMate()->GetAllFlags();
      m_vEdgeuses[i].m_lMateUserIndex1 = pEdgeuse->GetMate()->GetUserIndex1();
      m_vEdgeuses[i].m_lMateUserIndex2 = pEdgeuse->GetMate()->GetUserIndex2();
      SER(sm_ExtractAttributes(*pContext,                         // in : current context for object creation
                                pEdgeuse->GetMate(),              // in : target object potentially containing attributes
                                m_vEdgeuses[i].m_sMateAttributes, // out: rAttributes index array for this object's attributes
                                rAllAttributes,                   // i/o: accumulation of all attributes on all entities
                                sAttrMap));                       // i/o: accumulation of [attribute,ObjectAttributeIndexValue+1] pairs

    } // end iter all edgeuses

  // Init array for all 3DCurves
  m_v3DCurves.ReSet();

  // Edges - set EdgeData objects and store Curves

  // allocate a SmEdgeData Object for every edge
  m_vEdges.SetSize(lTotalEdges);
  for (i=0; i<lTotalEdges; i++)
    {
      SmEdge * pEdge = sAllEdges[i];
      m_vEdges[i].m_lPrimEU = sm_GetEdgeuseIndex(sMap,sEUMateMap,pEdge->GetPrimaryEdgeuse());

      SmVertex * pStartVert;
      SmVertex * pEndVert;
      if (pEdge->GetPrimaryEdgeuse()->GetOrientation() == SM_OT_SAME)
        { pStartVert = pEdge->GetVertex();
          pEndVert   = pEdge->GetOtherVertex(pStartVert);
        }
      else
        { pEndVert   = pEdge->GetVertex();
          pStartVert = pEdge->GetOtherVertex(pEndVert);
        }

      m_vEdges[i].m_lStartVertex = GET_MAP(pStartVert);
      m_vEdges[i].m_lEndVertex   = GET_MAP(pEndVert);
      m_vEdges[i].m_pEdge        = pEdge;

      SmCurve * pCurve = pEdge->GetCurve();
//      SmBoolean bFound = FALSE;

      // when curve is not in sMap
      if (!HAS_MAP(pCurve))
        {
          m_vEdges[i].m_lCurve = m_v3DCurves.GetSize();

          // add [Curve,CurveIndex+1] pair to sMap and place Curve in m_v3DCurves array
          ADD_TO_MAP(pCurve,m_v3DCurves.GetSize());
          m_v3DCurves.Add(pEdge->GetCurve());
        }
      else // just get the curve's index from sMap
        {
          m_vEdges[i].m_lCurve = GET_MAP(pCurve);
        }

      SM_OLDTOL_LINE m_vEdges[i].m_sZoneTol3d = pEdge->GetTolerance();

      // set edge values - for backward compatibility: map SM_UNDEF_ULONG flag and index values to zero
      m_vEdges[i].m_vInterval       = pEdge->GetInterval();
      m_vEdges[i].m_lFlags          = (pEdge->GetAllFlags()   == SM_UNDEF_ULONG) ? 0 : pEdge->GetAllFlags() ;
      m_vEdges[i].m_lUserIndex1     = (pEdge->GetUserIndex1() == SM_UNDEF_ULONG) ? 0 : pEdge->GetUserIndex1() ;
      m_vEdges[i].m_lUserIndex2     = (pEdge->GetUserIndex2() == SM_UNDEF_ULONG) ? 0 : pEdge->GetUserIndex2() ;
      m_vEdges[i].m_pUserPtr1       = pEdge->GetUserPtr1();
      m_vEdges[i].m_pEdge           = pEdge;

      // get edge attribute indices
      SER(sm_ExtractAttributes(*pContext,                  // in : current context for object creation
                                pEdge,                     // in : target object potentially containing attributes
                                m_vEdges[i].m_sAttributes, // out: rAttributes index array for this object's attributes
                                rAllAttributes,            // i/o: accumulation of all attributes on all entities
                                sAttrMap));                // i/o: accumulation of [attribute,ObjectAttributeIndexValue+1] pairs

    } // end iter every edge

// Remove Composites
//  // Note that right now attributes do not work for CFaces and CEdges because
//  // there are no SmCFaceData or SmCEdgeData structures to place an index.
//  // As an alternative you can place the attribute on the surface.

// Remove Composites
//  // Composite Faces
//  SmTArray<SmCFace*> sCFaces;
//  crBrep.GetCFaces(sCFaces);
//  ULONG lTotalCFaces = sCFaces.GetSize();
//  if (lTotalCFaces > 0)
//    {
//      m_vCFaces.Add(lTotalCFaces);
//      for (i=0; i<lTotalCFaces; i++)
//        {
//          SmCFace * pCFace = sCFaces[i];
//          SmTArray<SmFace*> sFaces;
//          pCFace->GetFaces(sFaces);
//          ULONG lNumFacesInCFace = sFaces.GetSize();
//          SM_ASSERT(lNumFacesInCFace > 1);
//          m_vCFaces.Add(lNumFacesInCFace);//face count
//          for (j=0; j<lNumFacesInCFace; j++)
//            {
//              SmFace * pFace = sFaces[j];
//              m_vCFaces.Add(GET_MAP(pFace));
//            }
//        }
//    }

// Remove Composites
//  // Composite Edges
//  SmTArray<SmCEdge*> sCEdges;
//  crBrep.GetCEdges(sCEdges);
//  ULONG lTotalCEdges = sCEdges.GetSize();
//  if (lTotalCEdges > 0)
//    {
//      m_vCEdges.Add(lTotalCEdges);
//      for (i=0; i<lTotalCEdges; i++)
//        {
//          SmCEdge * pCEdge = sCEdges[i];
//          SmTArray<SmEdge*> sEdges;
//          pCEdge->GetEdges(sEdges);
//          ULONG lNumEdgesInCEdge = sEdges.GetSize();
//          SM_ASSERT(lNumEdgesInCEdge > 1);
//          m_vCEdges.Add(lNumEdgesInCEdge);
//          for (j=0; j<lNumEdgesInCEdge; j++)
//            {
//              SmEdge * pEdge = sEdges[j];
//              m_vCEdges.Add(GET_MAP(pEdge));
//            }
//        }
//    }

  m_bIsGeomBorrowed = TRUE;   // All curves & surfaces are borrowed from Brep

  return SM_SUCCESS;

} // end SmBrepData::FromBrep

/*******************************************************************//**
PURPOSE: Read this BrepData from a stream (file) in either ASCII or
    binary format.

NOTES:
  1. Attributes are only read when lDBVersionNumber > 30
  2.
***********************************************************************/
SmStatus SmBrepData::ReadFromDB
 (const SmContext        & crContext,        // in : context for constructing objects from stream
  ULONG                    lDBVersionNumber, // in : target stream's version number
  SmTArray<SmAttribute*> & rAllAttributes,   // out: Array of all read attributes when lDBVersionNumber > 30
  SmBrepData            *& rpNewBrepData,    // out: Array of all read Brep Models
  SmDatabaseIO           & rDB,              // in : contains target stream
  SmBoolean                bPersistAttribs)  // in : TRUE = read attributes, false = don't, default:[TRUE]
{
  // locals
  ULONG lCount;
  ULONG index;
  ULONG i;

  // allocate new BrepData object
  SmBrepData * pBrepData = new(crContext) SmBrepData(TRUE, SM_DS_SMLIB);
  NER(pBrepData);

  // local file type
  SmFileType eType = rDB.GetFileType();

  // read Brep properties: color, tol, bManifold, bmakeComposites, numRegions, StartRegion
  if (eType == SM_ASCII)
    {
      std::istream & rFileIn = *rDB.GetInStreamPtr();

      GOTO_NEXT_LINE; //Brep Starts

      if (lDBVersionNumber > 30 && bPersistAttribs == TRUE)
        {
          SmAttributeData::ReadIndexedAttributesFromDB(crContext,rAllAttributes,
                                                       pBrepData->m_vAttributes,rDB);
        }
      else
        {
          rFileIn >> pBrepData->m_vColor.x >> pBrepData->m_vColor.y >> pBrepData->m_vColor.z;
          GOTO_NEXT_LINE;
        }

      // tolerance
      if (lDBVersionNumber > 43) // for NewTol Model
        {
#ifdef SM_USE_NEWTOL
          SM_NEWTOL_LINE rFileIn >> pBrepData->m_dThisModelSizeEstimate ; GOTO_NEXT_LINE;  
          SM_NEWTOL_LINE rFileIn >> pBrepData->m_dThisLargeSmallSizeRatio ;
#endif // SM_USE_NEWTOL
          
        } // for oldTol model
      else 
        { 
#ifdef SM_USE_OLDTOL
          SM_OLDTOL_LINE rFileIn >> pBrepData->m_sZoneTol3d;   // renamed from m_dTol with lDBVersionNumber == 38
#endif // SM_USE_OLDTOL
        }

      GOTO_NEXT_LINE;

      // manifold flag
      rFileIn >> pBrepData->m_bManifold;
      GOTO_NEXT_LINE;

// Remove Composites
// // makeComposites flag
//      rFileIn >> pBrepData->m_bMakeComposites;
      SmBoolean bMakeComposites ;
      rFileIn >> bMakeComposites;
      GOTO_NEXT_LINE;
      GOTO_NEXT_LINE; // blank line
      GOTO_NEXT_LINE; //Region Data

      //Region Data
      rFileIn >> pBrepData->m_lNumRegions;
      GOTO_NEXT_LINE;
      rFileIn >> pBrepData->m_lStartRegion;
      GOTO_NEXT_LINE;
      GOTO_NEXT_LINE; //Index, Flags, UserId1, UserId2, Number Of Shells, First Shell, Is void Region
    }
  else // Binary
    {
      if (lDBVersionNumber > 30 && bPersistAttribs == TRUE)
        {
          SmAttributeData::ReadIndexedAttributesFromDB(crContext,
                                                       rAllAttributes,
                                                       pBrepData->m_vAttributes,
                                                       rDB);
        }
      else
        {
          SER(rDB.ReadDoubles(&pBrepData->m_vColor.x,3));
        }

      if (lDBVersionNumber > 43) // for NewTol Model
        {
#ifdef SM_USE_NEWTOL
          SM_NEWTOL_LINE SER(rDB.ReadDouble(pBrepData->m_dThisModelSizeEstimate)) ; 
          SM_NEWTOL_LINE SER(rDB.ReadDouble(pBrepData->m_dThisLargeSmallSizeRatio)) ;
#endif // SM_USE_NEWTOL
        } // for oldTol model
      else 
        { 
#ifdef SM_USE_OLDTOL
          SM_OLDTOL_LINE SER(rDB.ReadDouble(pBrepData->m_sZoneTol3d));   // renamed from m_dTol with lDBVersionNumber == 38
#endif // SM_USE_OLDTOL
        }

      SER(rDB.ReadBoolean(pBrepData->m_bManifold));
// Remove Composites
//     SER(rDB.ReadBoolean(pBrepData->m_bMakeComposites));
      SmBoolean bMakeComposites = FALSE ;
      SER(rDB.ReadBoolean(bMakeComposites));
      SER(rDB.ReadLong(pBrepData->m_lNumRegions));
      SER(rDB.ReadLong(pBrepData->m_lStartRegion));
    }

  // Region Data

  // allocate one SmRegionData Object for every Region
  pBrepData->m_vRegions.SetSize(pBrepData->m_lNumRegions);

  // read every region
  for (i=0; i<pBrepData->m_lNumRegions; i++)
    {
      pBrepData->m_vRegions[i].ReadFromDB(crContext,lDBVersionNumber,rAllAttributes,rDB,bPersistAttribs);
    }

  // Shells
  if (eType == SM_ASCII)
    {
      std::istream & rFileIn = *rDB.GetInStreamPtr();
      GOTO_NEXT_LINE; // blank line
      GOTO_NEXT_LINE; //Shell Data
      rFileIn >> lCount;
      GOTO_NEXT_LINE;
      GOTO_NEXT_LINE; //Index, flags, UserId1, UserId2, 0(Faceuse), Orientation, Number Of Faceuses, Start Faceuse
      GOTO_NEXT_LINE; //Index, flags, UserId1, UserId2, 1(Wire Edge), Edge
      GOTO_NEXT_LINE; //Index, flags, UserId1, UserId2, 2(Vertex), Vertex
    }
  else // Binary
    {
      SER(rDB.ReadLong(lCount));
    }

  // allocate one SmShellData Object for every Shell
  pBrepData->m_vShells.SetSize(lCount);

  // read every shell
  for (i=0; i<lCount; i++)
    {
      pBrepData->m_vShells[i].ReadFromDB(crContext,lDBVersionNumber,rAllAttributes,rDB,bPersistAttribs);
    }

  // Composite Faces
  if (eType == SM_ASCII)
    {
      std::istream & rFileIn = *rDB.GetInStreamPtr();
      GOTO_NEXT_LINE; // blank line
      GOTO_NEXT_LINE; //Composite Face Data
      rFileIn >> lCount;
      GOTO_NEXT_LINE;
    }
  else // Binary
    {
      SER(rDB.ReadLong(lCount));
    }
  if (lCount != 0)
    {
      if (eType == SM_ASCII)
        {
          std::istream & rFileIn = *rDB.GetInStreamPtr();
          GOTO_NEXT_LINE; //Index, Face Count, Face1, Face2, ...
        }

      // add number of CFaces to CFaces list
      pBrepData->m_vCFaces.Add(lCount);

      // CFace structure is stored as a list of longs
      //  [ CFaceCount 1stCFaceFaceCount Face1Index, Face2Index, . . .
      //               2ndCFaceFaceCount Face1Index, Face2Index, . . .
      //               . . . ]
      for (i=0; i<lCount; i++)
        {
          ULONG lFaceCount=0;
          ULONG lFace=0;
          if (eType == SM_ASCII)
            {
              std::istream & rFileIn = *rDB.GetInStreamPtr();
              rFileIn >> index;
              rFileIn >> lFaceCount;
              pBrepData->m_vCFaces.Add(lFaceCount);
              for (ULONG j=0; j<lFaceCount; j++)
                {
                  rFileIn >> lFace;
                  pBrepData->m_vCFaces.Add(lFace);
                }
              GOTO_NEXT_LINE;
            }
          else // Binary
            {
              SER(rDB.ReadLong(lFaceCount));
              pBrepData->m_vCFaces.Add(lFaceCount);
              for (ULONG j=0; j<lFaceCount; j++)
                {
                  SER(rDB.ReadLong(lFace));
                  pBrepData->m_vCFaces.Add(lFace);
                }
            } // end binary branch
        } // end iter every composite face
    } // end lCount !=> 0 check

  // Faceuse Data
  if (eType == SM_ASCII)
    {
      std::istream & rFileIn = *rDB.GetInStreamPtr();
      GOTO_NEXT_LINE; // blank line
      GOTO_NEXT_LINE; //Faceuse Data
      rFileIn >> lCount;
      GOTO_NEXT_LINE;
      GOTO_NEXT_LINE; //Index, Flags, UserId1, UserId2, Face, Orientation
    }
  else // Binary
    {
      SER(rDB.ReadLong(lCount));
    }

  // allocate one SmFaceuseData Object for each faceuse
  pBrepData->m_vFaceuses.SetSize(lCount);

  // read every faceuse
  for (i=0; i<lCount; i++)
    {
      pBrepData->m_vFaceuses[i].ReadFromDB(crContext,lDBVersionNumber,rAllAttributes,rDB,bPersistAttribs);
    }

  // Face Data
  if (eType == SM_ASCII)
    {
      std::istream & rFileIn = *rDB.GetInStreamPtr();
      GOTO_NEXT_LINE; // blank line
      GOTO_NEXT_LINE; //Face Data
      rFileIn >> lCount;
      GOTO_NEXT_LINE;
      GOTO_NEXT_LINE; //Index, Flags, UserId1, UserId2, Tol, Surface, Outer Loop, # of Loops, Naturally Trimmed
      GOTO_NEXT_LINE; //UMin, UMax, VMin, VMax
    }
  else // Binary
    {
//        rFileIn.read((char*)&lCount,sizeof(ULONG));
      SER(rDB.ReadLong(lCount));
    }

  // allocate one SmFaceData Object for each face
  pBrepData->m_vFaces.SetSize(lCount);

  // read every face
  for (i=0; i<lCount; i++)
    {
      pBrepData->m_vFaces[i].ReadFromDB(crContext,lDBVersionNumber,rAllAttributes,rDB,bPersistAttribs);
    }

  // Loop Data
  if (eType == SM_ASCII)
    {
      std::istream & rFileIn = *rDB.GetInStreamPtr();
      GOTO_NEXT_LINE; // blank line
      GOTO_NEXT_LINE; //Loop Data
      rFileIn >> lCount;
      GOTO_NEXT_LINE;
      GOTO_NEXT_LINE; //Index, 0(Edgeuse), # Of Edgeuses, Start Edgeuse
      GOTO_NEXT_LINE; //Index, 1(Vertex), Vertex
    }
  else // Binary
    {
      SER(rDB.ReadLong(lCount));
    }

  // allocate one SmLoopData Object for each loop
  pBrepData->m_vLoops.SetSize(lCount);

  // read every loop
  for (i=0; i<lCount; i++)
    {
      pBrepData->m_vLoops[i].ReadFromDB(crContext,lDBVersionNumber,rAllAttributes,rDB,bPersistAttribs);
    }

  // Composite Edge
  if (eType == SM_ASCII)
    {
      std::istream & rFileIn = *rDB.GetInStreamPtr();
      GOTO_NEXT_LINE; // blank line
      GOTO_NEXT_LINE; //Composite Edge Data
      rFileIn >> lCount;
      GOTO_NEXT_LINE;
    }
  else // Binary
    {
      SER(rDB.ReadLong(lCount));
    }
  if (lCount != 0)
    {
      if (eType == SM_ASCII)
        {
          std::istream & rFileIn = *rDB.GetInStreamPtr();
          GOTO_NEXT_LINE; //Index, Edge Count, Edge1, Edge2, ...
        }

      // Read CEdge list of ULONG values
      // CEdge structure is stored as a list of longs
      //  [ CEdgeCount 1stCEdgeEdgeCount Edge1Index, Edge2Index, . . .
      //               2ndCEdgeEdgeCount Edge1Index, Edge2Index, . . .
      //               . . . ]
      pBrepData->m_vCEdges.Add(lCount);
      for (i=0; i<lCount; i++)
        {
          ULONG lEdgeCount;
          ULONG lEdge;
          if (eType == SM_ASCII)
            {
              std::istream & rFileIn = *rDB.GetInStreamPtr();
              rFileIn >> index;
              rFileIn >> lEdgeCount;
              pBrepData->m_vCEdges.Add(lEdgeCount);
              for (ULONG j=0; j<lEdgeCount; j++)
                {
                  rFileIn >> lEdge;
                  pBrepData->m_vCEdges.Add(lEdge);
                }
              GOTO_NEXT_LINE;
            }
          else // Binary
            {
              SER(rDB.ReadLong(lEdgeCount));
              pBrepData->m_vCEdges.Add(lEdgeCount);
              for (ULONG j=0; j<lEdgeCount; j++)
                {
                  SER(rDB.ReadLong(lEdge));
                  pBrepData->m_vCEdges.Add(lEdge);
                }
            }
        }
    }

  // Edgeuse Data
  if (eType == SM_ASCII)
    {
      std::istream & rFileIn = *rDB.GetInStreamPtr();
      GOTO_NEXT_LINE; // blank line
      GOTO_NEXT_LINE; //Edgeuse Data
      rFileIn >> lCount;
      GOTO_NEXT_LINE;
      GOTO_NEXT_LINE; //Index, Flags, UserId1, UserId2, MateFlags, MateUserId1, MateUserId2, 0(Edge), Edge, Loop, Orientation, UV Curve
      GOTO_NEXT_LINE; //Index, Flags, UserId1, UserId2, MateFlags, MateUserId1, MateUserId2, 1(Vertex), Vertex, Loop, Orientation, UV Curve
      GOTO_NEXT_LINE; //Index, Flags, UserId1, UserId2, MateFlags, MateUserId1, MateUserId2, 2(Wire Edge), Edge, Shell, Orientation, UV Curve
      GOTO_NEXT_LINE; //Index, Flags, UserId1, UserId2, MateFlags, MateUserId1, MateUserId2, 3(Vertex At Pole), Vertex, Loop, Orientation, UV Curve
    }
  else // Binary
    {
      SER(rDB.ReadLong(lCount));
    }

  // allocate one SmEUData OBject for each edge
  pBrepData->m_vEdgeuses.SetSize(lCount);

  // read every edgeuse
  for (i=0; i<lCount; i++)
    {
      pBrepData->m_vEdgeuses[i].ReadFromDB(crContext,lDBVersionNumber,rAllAttributes,rDB,bPersistAttribs);
    }

  // Edge Data
  if (eType == SM_ASCII)
    {
      std::istream & rFileIn = *rDB.GetInStreamPtr();
      GOTO_NEXT_LINE; // blank line
      GOTO_NEXT_LINE; //Edge Data
      rFileIn >> lCount;
      GOTO_NEXT_LINE;
      GOTO_NEXT_LINE; //Index, Flags, UserId1, UserId2, Tol, Curve, Start Vertex, End Vertex, Param Min, Param Max
    }
  else // Binary
    {
      SER(rDB.ReadLong(lCount));
    }

  // allocate one SmEdgeData OBject for each edge
  pBrepData->m_vEdges.SetSize(lCount);

  // read every edge
  for (i=0; i<lCount; i++)
    {
      pBrepData->m_vEdges[i].ReadFromDB(crContext,lDBVersionNumber,rAllAttributes,rDB,bPersistAttribs);
    }

  // Vertex Data
  if (eType == SM_ASCII)
    {
      std::istream & rFileIn = *rDB.GetInStreamPtr();
      GOTO_NEXT_LINE; // blank line
      GOTO_NEXT_LINE; //Vertex Data
      rFileIn >> lCount;
      GOTO_NEXT_LINE;
      GOTO_NEXT_LINE; //Index, Flags, UserId1, UserId2, Tol, X, Y, Z
    }
  else // Binary
    {
      SER(rDB.ReadLong(lCount));
    }

  // allocate one SmVertexData OBject for each vertex
  pBrepData->m_vVertices.SetSize(lCount);

  // read every vertex
  for (i=0; i<lCount; i++)
    {
      pBrepData->m_vVertices[i].ReadFromDB(crContext,lDBVersionNumber,rAllAttributes,rDB,bPersistAttribs);
    }

  // UV-Curve Data
  if (eType == SM_ASCII)
    {
      std::istream & rFileIn = *rDB.GetInStreamPtr();
      GOTO_NEXT_LINE; // blank line
      GOTO_NEXT_LINE; //UV-Curve Data
      rFileIn >> lCount;
      GOTO_NEXT_LINE;
    }
  else // Binary
    {
      SER(rDB.ReadLong(lCount));
    }

  // allocate one SmCurve Pointer for each UVCurve
  pBrepData->m_vUVCurves.SetSize(lCount);

  // read every UVCurve
  for (i=0; i<lCount; i++)
    {
      if (eType == SM_ASCII)
        {
          std::istream & rFileIn = *rDB.GetInStreamPtr();
          rFileIn >> index;
          GOTO_NEXT_LINE;
        }

      // read build  the UVCurve
      SER(SmBrepData::ReadCurveFromDB(crContext,
                                      2,
                                      lDBVersionNumber,
                                      rAllAttributes,
                                      pBrepData->m_vUVCurves[i],
                                      rDB,
                                      bPersistAttribs));
    }

  // 3D-Curve Data
  if (eType == SM_ASCII)
    {
      std::istream & rFileIn = *rDB.GetInStreamPtr();
      GOTO_NEXT_LINE;
      GOTO_NEXT_LINE; //3D-Curve Data
      rFileIn >> lCount;
      GOTO_NEXT_LINE;
    }
  else // Binary
    {
      SER(rDB.ReadLong(lCount));
    }

  // allocate one SmCurve pointer for each 3DCurve
  pBrepData->m_v3DCurves.SetSize(lCount);

  // read every 3DCurve
  for (i=0; i<lCount; i++)
    {
      if (eType == SM_ASCII)
        {
          std::istream & rFileIn = *rDB.GetInStreamPtr();
          rFileIn >> index;
          GOTO_NEXT_LINE;
        }

      // read, build the 3DCurve
      SER(SmBrepData::ReadCurveFromDB(crContext,
                                      3,
                                      lDBVersionNumber,
                                      rAllAttributes,
                                      pBrepData->m_v3DCurves[i],
                                      rDB,
                                      bPersistAttribs));
    }

  // Surface Data
  if (eType == SM_ASCII)
    {
      std::istream & rFileIn = *rDB.GetInStreamPtr();
      GOTO_NEXT_LINE;
      GOTO_NEXT_LINE; //Surface Data
      rFileIn >> lCount;
      GOTO_NEXT_LINE;
    }
  else // Binary
    {
      SER(rDB.ReadLong(lCount));
    }

  // allocate one SmSurface Pointer for each Surface
  pBrepData->m_vSurfaces.SetSize(lCount);
  for (i=0; i<lCount; i++)
    {
      if (eType == SM_ASCII)
        {
          std::istream & rFileIn = *rDB.GetInStreamPtr();
          rFileIn >> index;
          GOTO_NEXT_LINE;
        }

      // read and build the surface
      SER(SmBrepData::ReadSurfaceFromDB(crContext,
                                        lDBVersionNumber,rAllAttributes,
                                        pBrepData->m_vSurfaces[i],rDB,bPersistAttribs));
    }

  rpNewBrepData = pBrepData;

  return SM_SUCCESS;

} // end SmBrepData::ReadFromDB

/*******************************************************************/ /**
 PURPOSE: Write a single brep out to a file.

 NOTES: Pass through funciton to SmBrepData::WriteToFile (TCHAR version)
        See Notes SmBrepData::WriteToFile (TCHAR version)
 ***********************************************************************/
SmStatus SmBrepData::WriteToFile(
    const std::string& cOutputFileName,  // in : target file name
    SmFileType eType,                    // in : oneof SM_ASCII or SM_BINARY
    SmBoolean bNewFile,                  // in : TRUE = open file and rewrite contents
                                         //      FALSE= open file and append to end
    SmBoolean bWriteAsBSplines,          // in : TRUE = Approx nonBSplineGeometry and write out as
                                         // BSplineShapes
                                         //      FALSE= Write native formats for nonBSplines
                                         //      default:[FALSE]
    SmApproxTol3d sApproxTol3d)          // in : only used when bWriteAsBSplines==TRUE
                                         //      when approximating geometry as BSplines for output.
                                         //      0.0 = use m_sApproxTol3d, default:[0.0]
    const
{
    // convert to TCHAR and then call existing SmBrepData::WriteToFile
#ifdef UNICODE
    std::wstring wPath(cOutputFileName.begin(), cOutputFileName.end());
    return WriteToFile(wPath.c_str(), eType, bNewFile, bWriteAsBSplines, sApproxTol3d);
#else
    return WriteToFile(cOutputFileName.c_str(), eType, bNewFile, bWriteAsBSplines, sApproxTol3d);
#endif

} // SmBrepData::WriteToFile

/*******************************************************************//**
PURPOSE: Write a single brep out to a file and append it to the end
    of the existing file.

NOTES: Opens a file for write -
       calls virtual BeginWriting() for derived SmBrepDataIO classes - base class does nothing
       passes call to WriteToDB()
       calls virtual EndWriting() for derived SmBrepDataIO classes - base class does nothing
       Closes file
***********************************************************************/
SmStatus SmBrepData::WriteToFile
  (const TCHAR * cOutputFileName,    // in : target file name
   SmFileType    eType,              // in : oneof SM_ASCII or SM_BINARY
   SmBoolean     bNewFile,           // in : TRUE = open file and rewrite contents
                                     //      FALSE= open file and append to end
   SmBoolean     bWriteAsBSplines,   // in : TRUE = Approx nonBSplineGeometry and write out as BSplineShapes
                                     //      FALSE= Write native formats for nonBSplines
                                     //      default:[FALSE]
   SmApproxTol3d sApproxTol3d)       // in : only used when bWriteAsBSplines==TRUE
                                     //      when approximating geometry as BSplines for output.
                                     //      0.0 = use m_sApproxTol3d, default:[0.0]
  const
{
  SmDatabaseIOFile sDB ;
  TCHAR sBuff[SM_TBLOCK_SIZE] ;

  // open file for output
  if(SM_SUCCESS != sDB.OpenFileForWrite(cOutputFileName, // in : target file name
                                        eType,           // in : oneof: SM_ASCII    = write ascii file with comment lines
                                                         //             SM_BINARY   = write binary file from system format to LittleEndian
                                                         //             SM_BYTESWAP = write binary file - forcing byte swap
                                                         //      default:[SM_ASCII]                       
                                        bNewFile))       // in : bNewFile = TRUE  = open file and rewrite contents
                                                         //                 FALSE = open file and append to end
                                                         //                 default:[FALSE]
    {
      smos_sprintf(sBuff,_T("  *** ERROR File not found - %s\n"),cOutputFileName);
      smos_WriteBuffer(sBuff);
      return SM_ERR;
    }

  // output locals
  const SmContext *cpContext = GetContext() ;
  SM_ASSERT_MSG(cpContext != NULL, _T("SmBrepData::WriteToFile(): this->m_cpContext == NULL")) ;
  SmTArray<SmAttribute*>               sAllAttributes;
  SmMapTypeToType<SmAttribute *,ULONG> sAttrMap;

  // virtual preWrite for derived classes - base class does nothing
  SER(sDB.BeginWriting()) ;

  // starting with version 38 write out a header for this file switching on file type
  if (eType == SM_ASCII)
    {
      std::ostream & rFileOut = *sDB.GetOutStreamPtr();
      rFileOut << "Version " << SM_CURRENT_DATABASE_VERSION << "\n" ;
      rFileOut << "Healer Version " << SM_HEALER_VERSION << "\n";
    }
  else // Binary
    {
      // output a one character header and the Database version number
      SER(sDB.WriteChar('V')) ;
      SER(sDB.WriteLong(SM_CURRENT_DATABASE_VERSION));

      // output a one character header and the Healer version number
      SER(sDB.WriteChar('H')) ;
      SER(sDB.WriteLong(SM_HEALER_VERSION));
    }

  // output BrepData to stream
  SER(WriteToDB(*cpContext,
                 SM_CURRENT_DATABASE_VERSION,
                 sAllAttributes,
                 sAttrMap,
                 sDB,
                 bWriteAsBSplines,
                 sApproxTol3d,
                 FALSE));         // in : FALSE = skip attributes

  // virtual postWrite for derived classes - base class does nothing
  SER(sDB.EndWriting());

  // all done
  return SM_SUCCESS;

} // end SmBrepData::WriteToFile

/*******************************************************************//**
PURPOSE: Read a single brep from a file of the existing file
  without reading attributes.

NOTES: Opens a file for Read -
       calls virtual BeginReading() for derived SmBrepDataIO classes - base class does nothing
       passes call to ReadFromDB()
       calls virtual EndReading() for derived SmBrepDataIO classes - base class does nothing
       Closes file
***********************************************************************/
SmStatus SmBrepData::ReadFromFile
 (const SmContext & crContext,       // in : context for new object construction
  const TCHAR     * cInputFileName,  // in : target file to read
  SmBrepData     *& rpNewBrepData,   // out: read BrepData
  SmFileType        eType)           // in : one of SM_ASCII or SM_BINARY
{
  SmDatabaseIOFile sDB;
  TCHAR sBuff[SM_TBLOCK_SIZE] ;

  // build and init the stream structure
  if(SM_SUCCESS != sDB.OpenFileForRead(cInputFileName, eType))
    {
      smos_sprintf(sBuff,_T("  *** ERROR File not found - %s\n"),cInputFileName);
      smos_WriteBuffer(sBuff);
      return SM_ERR;
    }

  // virtual preRead for derived classes - base class does nothing
  SER(sDB.BeginReading());

  // starting with version 38 write out a header for this file switching on file type
  std::istream & rFileIn = *sDB.GetInStreamPtr();

  char c1stChar    = (char)rFileIn.get() ;
  if ( c1stChar == 10 )
      c1stChar = (char)rFileIn.get() ;
  rFileIn.unget();  // peek was not working in VS10

  ULONG lDBVersionNumber, lHealerVersion = 0 ;
  if (c1stChar != 'V')
    {
      lDBVersionNumber = 30 ;
    }
  else
    {
      if (eType == SM_ASCII)
        {
          char sHeader[8] ; // for "Version "
          // std::ostream & rFileOut = *sDB.GetOutStreamPtr();
          rFileIn >> sHeader >> lDBVersionNumber ;
          GOTO_NEXT_LINE; // to get off line "Version VersionNumber"
        }
      else // Binary
        {
          // output a one character header and the version number
          char sHeader ;  // for char 'V'
          SER(sDB.ReadChar(sHeader)) ;
          sDB.ReadLong( lDBVersionNumber );
          if ( lDBVersionNumber >= 40 )
          { rFileIn.ignore( sizeof( long long ) - sizeof( long ) ); }
        }
    }

  sDB.SetVersionNum( lDBVersionNumber );

  // SM_CURRENT_DATABASE_VERSION 42 added the SM_HEALER_VERSION to the files.
  if ( lDBVersionNumber > 41 )
  {
      if (eType == SM_ASCII)
        {
          char sHeader[8] ; // for "Healer Version "
          rFileIn >> sHeader >> sHeader >> lHealerVersion ;
          GOTO_NEXT_LINE; // to get off line "Healer Version HealerVersionNumber"
        }
      else // Binary
        {
          // output a one character header and the version number
          char sHeader ;  // for char 'H'
          SER(sDB.ReadChar(sHeader)) ;
          sDB.ReadLong( lHealerVersion );
        }
      }

  // pass the call along
  SmTArray<SmAttribute*> sAllAttributes;
  SER(SmBrepData::ReadFromDB(crContext,
                             lDBVersionNumber,  // Use old version without attributes
                             sAllAttributes,
                             rpNewBrepData,
                             sDB,
                             FALSE));    // FALSE = skip attributes

  rpNewBrepData->SetHealerVersion( lHealerVersion );

  // virtual postRead for derived classes - base class does nothing
  SER(sDB.EndReading());

  // all done
  return SM_SUCCESS;

} // end SmBrepData::ReadFromFile

/*******************************************************************//**
PURPOSE: Write the Brep Data out to a stream or file.

NOTES:
  1. attributes on curves and surfaces.

     UV Trim Curves, 3D Curves and 3D Surfaces are stored
     as SmCurve and SmSurface objects in SmBrepData.  When these are
     written to stream by this function, all their attribute pointers are
     replaced by attribute indices and the attribute pointer is placed
     on the rAllAttributes accumulation attribute list.  That list
     already contains all the attribute pointers connected to SmTopology
     objects.  They were put their when the SmBrepData lists were built
     by the SmBrepData::FromBrep() Call.  After this call rAllAttributes
     will contain all the attributes connected to the objects of the
     original SmBrep Object.
***********************************************************************/
SmStatus SmBrepData::WriteToDB
  (const SmContext                      & crContext,          // in : context for constructing objects
   ULONG                                  lDBVersionNumber,   // in : target stream's version number
   SmTArray<SmAttribute*>               & rAllAttributes,     // out: Array of all attributes accumulated during write
   SmMapTypeToType<SmAttribute *,ULONG> & rAttrMap,           // out: accumulated attribute to object map
   SmDatabaseIO                         & rDB,                // in : contains target stream
   SmBoolean                              bWriteAsBSplines,   // in : TRUE = Approx nonBSplineGeometry and write out as BSplineShapes
                                                              //      FALSE= Write native formats for nonBSplines
                                                              //      default:[FALSE]
   SmApproxTol3d                          sApproxTol3d,       // in : only used when bWriteAsBSplines==TRUE
                                                              //      when approximating geometry as BSplines for output.
                                                              //      0.0 = use m_sApproxTol3d, default:[0.0]
   SmBoolean                              bPersistAttribs)    // in : TRUE = write attributes, false = don't, default:[TRUE]
  const
{
  SmFileType eType = rDB.GetFileType();

  ULONG i;
  ULONG lCount;

  // output brep properties: color, tolerance, manifold flag,
  //                         composites flag, numRegions, startRegion
  if (eType == SM_ASCII)
    {
      std::ostream & rFileOut = *rDB.GetOutStreamPtr();

      rFileOut << "//Brep Starts\n";
      if (lDBVersionNumber > 30 && bPersistAttribs == TRUE)
        {
          SmAttributeData::WriteIndexedAttributesToDB(m_vAttributes,rDB);
        }
      else
        {
          rFileOut << m_vColor.x << " " << m_vColor.y << " " << m_vColor.z << " - Color\n";
        }

      if (lDBVersionNumber > 43) // for NewTol Model
        {
          SM_NEWTOL_LINE rFileOut << m_dThisModelSizeEstimate   << " - Brep->SizeEstimate - Sets BrepZoneTol3d value\n";     // renamed from m_dTol with lDBVersionNumber == 38
          SM_NEWTOL_LINE rFileOut << m_dThisLargeSmallSizeRatio << " - Brep->LargeSmallRation - for Pinhole in Battleship models\n" ;
        } // for oldTol model
      else 
        { SM_OLDTOL_LINE rFileOut << m_sZoneTol3d << " - Brep ZoneTol3d\n";     // renamed from m_dTol with lDBVersionNumber == 38
        }

      rFileOut << m_bManifold << " - is Manifold:[1 = is, 0 = isn't]\n";
// Remove Composities
//      rFileOut << m_bMakeComposites << " - has Composites:[1 = does, 0 = doesn't]\n";
      SmBoolean bMakeComposites = FALSE ;
      rFileOut << bMakeComposites << " - has Composites:[1 = does, 0 = doesn't]\n";
      rFileOut << "\n//Region Data\n";
      rFileOut << m_vRegions.GetSize() << " Regions In Brep\n";
      rFileOut << m_lStartRegion << " - Start Region\n";
      if(lDBVersionNumber > 30)
        rFileOut << "//Index, Flags, UserId1, UserId2, Number Of Shells, First Shell, Is void Region\n";
      else
        rFileOut << "//Index, Number Of Shells, First Shell, Is void Region\n";
    }
  else // Binary
    {
      if (lDBVersionNumber > 30 && bPersistAttribs == TRUE)
        { SmAttributeData::WriteIndexedAttributesToDB(m_vAttributes,rDB);
        }
      else
        {
          SER(rDB.WriteDoubles(&m_vColor.x,3));
        }

      if (lDBVersionNumber > 43) // for NewTol Model
        {
          SM_NEWTOL_LINE SER(rDB.WriteDouble(m_dThisModelSizeEstimate));   // Brep->SizeEstimate - Sets BrepZoneTol3d value\n";     // renamed from m_dTol with lDBVersionNumber == 38
          SM_NEWTOL_LINE SER(rDB.WriteDouble(m_dThisLargeSmallSizeRatio)); // Brep->LargeSmallRation - for Pinhole in Battleship models" ;
        } // for oldTol model
      else 
        { SM_OLDTOL_LINE SER(rDB.WriteDouble(m_sZoneTol3d));  // renamed from m_dTol with lDBVersionNumber == 38
        }

      SER(rDB.WriteBoolean(m_bManifold));
// Remove Composites
//      SER(rDB.WriteBoolean(m_bMakeComposites));
      SmBoolean bMakeComposites = FALSE ; 
      SER(rDB.WriteBoolean(bMakeComposites));
      SER(rDB.WriteLong(m_lNumRegions));
      SER(rDB.WriteLong(m_lStartRegion));
    }

  // Region Data
  for (i=0; i<m_vRegions.GetSize(); i++)
    {
      if (eType == SM_ASCII)
        {
          std::ostream & rFileOut = *rDB.GetOutStreamPtr();
          rFileOut << i;//Region Index
        }
      m_vRegions[i].WriteToDB(lDBVersionNumber,rDB,bPersistAttribs);
    }

  // Shells
  lCount = m_vShells.GetSize();
  if (eType == SM_ASCII)
    {
      std::ostream & rFileOut = *rDB.GetOutStreamPtr();
      rFileOut << "\n//Shell Data\n";
      rFileOut << lCount << " Shells In Brep\n";

      if(lDBVersionNumber > 30)
        {
          rFileOut << "//Index, Flags, UserId1, UserId2, 0(Faceuse), Orientation, Number Of Faceuses, Start Faceuse\n";
          rFileOut << "//Index, Flags, UserId1, UserId2, 1(WireEdge), Edge\n";
          rFileOut << "//Index, Flags, UserId1, UserId2, 2(Vertex), Vertex\n";
        }
      else
        {
          rFileOut << "//Index, 0(Faceuse), Orientation, Number Of Faceuses, Start Faceuse\n";
          rFileOut << "//Index, 1(WireEdge), Edge\n";
          rFileOut << "//Index, 2(Vertex), Vertex\n";
        }
    }
  else // Binary
    {
      SER(rDB.WriteLong(lCount));
    }

  // for every shell
  for (i=0; i<lCount; i++)
    {
      if (eType == SM_ASCII)
        {
          std::ostream & rFileOut = *rDB.GetOutStreamPtr();
          rFileOut << i;//Shell Index
        }
      m_vShells[i].WriteToDB(lDBVersionNumber,rDB,bPersistAttribs);
    }

  // Composite Faces
  if (   m_vCFaces.GetSize()     == 0
      || (lCount = m_vCFaces[0]) == 0) lCount = 0;
  if (eType == SM_ASCII)
    {
      std::ostream & rFileOut = *rDB.GetOutStreamPtr();
      rFileOut << "\n//Composite Face Data\n";
      rFileOut << lCount << " Composite Faces In Brep\n";
    }
  else // Binary
    {
      SER(rDB.WriteLong(lCount));
    }

  // when there are Composite Faces
  if (lCount > 0)
    {
      if (eType == SM_ASCII)
        {
          std::ostream & rFileOut = *rDB.GetOutStreamPtr();
          if(lDBVersionNumber > 30)
            {
              rFileOut << "//Index, Flags, UserId1, UserId2, Face Count, Face1, Face2, ...\n";
            }
          else
            {
              rFileOut << "//Index, Face Count, Face1, Face2, ...\n";
            }
        }
      ULONG lCurrIndx = 1;

      // for every composite face
      for (i=0; i<lCount; i++)
        {
          ULONG lFaceCount = m_vCFaces[lCurrIndx++];
          if (eType == SM_ASCII)
            {
              std::ostream & rFileOut = *rDB.GetOutStreamPtr();
              rFileOut << i << " " << lFaceCount;
              for (ULONG j=0; j<lFaceCount; j++)
                  rFileOut << " " << m_vCFaces[lCurrIndx++];
              rFileOut << "\n";
            }
          else // Binary
            {
              SER(rDB.WriteLong(lFaceCount));
              for (ULONG j=0; j<lFaceCount; j++)
                {
                  ULONG lFace = m_vCFaces[lCurrIndx++];
                  SER(rDB.WriteLong(lFace));
                }
            }
        } // end iter every composite face
    } // end composite face existence check

  // Faceuse Data
  lCount = m_vFaceuses.GetSize();
  if (eType == SM_ASCII)
    {
      std::ostream & rFileOut = *rDB.GetOutStreamPtr();
      rFileOut << "\n//Faceuse Data\n";
      rFileOut << lCount << " Faceuses In Brep\n";
      if(lDBVersionNumber > 30)
        {
          rFileOut << "//Index, Flags, UserId1, UserId2, Face, Orientation\n";
        }
      else
        {
          rFileOut << "//Index, Face, Orientation\n";
        }
    }
  else // Binary
    {
      SER(rDB.WriteLong(lCount));
    }

  for (i=0; i<lCount; i++)
    {
      if (eType == SM_ASCII)
        {
          std::ostream & rFileOut = *rDB.GetOutStreamPtr();
          rFileOut << i;//Index
        }
      m_vFaceuses[i].WriteToDB(lDBVersionNumber,rDB,bPersistAttribs);
    }

  // Face Data
  lCount = m_vFaces.GetSize();
  if (eType == SM_ASCII)
    {
      std::ostream & rFileOut = *rDB.GetOutStreamPtr();
      rFileOut << "\n//Face Data\n";
      rFileOut << lCount << " Faces In Brep\n";
      if(lDBVersionNumber > 30)
        {
          rFileOut << "//Index, Flags, UserId1, UserId2, Tol, Surface, Outer Loop, # of Loops, Naturally Trimmed\n";
        }
      else
        {
         rFileOut << "//Index, Tol, Surface, Outer Loop, # of Loops, Naturally Trimmed\n";
        }
      rFileOut << "//UMin, UMax, VMin, VMax\n";
    }
  else // Binary
    {
      SER(rDB.WriteLong(lCount));
    }

  for (i=0; i<lCount; i++)
    {
      if (eType == SM_ASCII)
        {
          std::ostream & rFileOut = *rDB.GetOutStreamPtr();
          rFileOut << i;//Index
        }
      m_vFaces[i].WriteToDB(lDBVersionNumber,rDB,bPersistAttribs);
    }

  // Loop Data
  lCount = m_vLoops.GetSize();
  if (eType == SM_ASCII)
    {
      std::ostream & rFileOut = *rDB.GetOutStreamPtr();
      rFileOut << "\n//Loop Data\n";
      rFileOut << lCount << " Loops In Brep\n";
      if(lDBVersionNumber > 30)
        {
          rFileOut << "//Index, Flags, UserId1, UserId2, 0(Edgeuse), # Of Edgeuses, Start Edgeuse\n";
          rFileOut << "//Index, Flags, UserId1, UserId2, 1(Vertex), Vertex\n";
        }
      else
        {
          rFileOut << "//Index, 0(Edgeuse), # Of Edgeuses, Start Edgeuse\n";
          rFileOut << "//Index, 1(Vertex), Vertex\n";
        }
    }
  else // Binary
    {
      SER(rDB.WriteLong(lCount));
    }

  for (i=0; i<lCount; i++)
    {
      if (eType == SM_ASCII)
        {
          std::ostream & rFileOut = *rDB.GetOutStreamPtr();
          rFileOut << i;//Index
        }
      m_vLoops[i].WriteToDB(lDBVersionNumber,rDB,bPersistAttribs);
    }

  // Composite Edges
  if (   m_vCEdges.GetSize()     == 0
      || (lCount = m_vCEdges[0]) == 0)
      lCount = 0;

  if (eType == SM_ASCII)
    {
      std::ostream & rFileOut = *rDB.GetOutStreamPtr();
      rFileOut << "\n//Composite Edge Data\n";
      rFileOut << lCount << " Composite Edges In Brep\n";
    }
  else // Binary
    {
      SER(rDB.WriteLong(lCount));
    }

  if (lCount > 0)
    {
      if (eType == SM_ASCII)
        {
          std::ostream & rFileOut = *rDB.GetOutStreamPtr();
          if(lDBVersionNumber > 30)
            {
              rFileOut << "//Index, Flags, UserId1, UserId2, Edge Count, Edge1, Edge2, ...\n";
            }
          else
            {
              rFileOut << "//Index, Edge Count, Edge1, Edge2, ...\n";
            }
        }
      ULONG lCurrIndx = 1;
      for (i=0; i<lCount; i++)
        {
          ULONG lEdgeCount = m_vCEdges[lCurrIndx++];
          if (eType == SM_ASCII)
            {
              std::ostream & rFileOut = *rDB.GetOutStreamPtr();
              rFileOut << i << " " << lEdgeCount;
              for (ULONG j=0; j<lEdgeCount; j++)
                  rFileOut << " " << m_vCEdges[lCurrIndx++];
              rFileOut << "\n";
            }
          else // Binary
            {
              SER(rDB.WriteLong(lEdgeCount));
              for (ULONG j=0; j<lEdgeCount; j++)
                {
                  ULONG lEdge = m_vCEdges[lCurrIndx++];
                  SER(rDB.WriteLong(lEdge));
                }
            }
        }
    }

  // Edgeuse Data
  lCount = m_vEdgeuses.GetSize();
  if (eType == SM_ASCII)
    {
      std::ostream & rFileOut = *rDB.GetOutStreamPtr();
      rFileOut << "\n//Edgeuse Data\n";
      rFileOut << lCount << " Edgeuses In Brep\n";
      if(lDBVersionNumber > 30)
        {
          rFileOut << "//Index, Flags, UserId1, UserId2, MateFlags, MateUserId1, MateUserId2, 0(Edge), Edge, Loop, Orientation, UV Curve, Next EU, Mate Next EU\n";
          rFileOut << "//Index, Flags, UserId1, UserId2, MateFlags, MateUserId1, MateUserId2, 1(Vertex), Vertex, Loop, Orientation, UV Curve\n";
          rFileOut << "//Index, Flags, UserId1, UserId2, MateFlags, MateUserId1, MateUserId2, 2(Wire Edge), Edge, Shell, Orientation, UV Curve\n";
          rFileOut << "//Index, Flags, UserId1, UserId2, MateFlags, MateUserId1, MateUserId2, 3(Vertex At Pole), Vertex, Loop, Orientation, UV Curve\n";
        }
      else
        {
          rFileOut << "//Index, 0(Edge), Edge, Loop, Orientation, UV Curve, Next EU, Mate Next EU\n";
          rFileOut << "//Index, 1(Vertex), Vertex, Loop, Orientation, UV Curve\n";
          rFileOut << "//Index, 2(Wire Edge), Edge, Shell, Orientation, UV Curve\n";
          rFileOut << "//Index, 3(Vertex At Pole), Vertex, Loop, Orientation, UV Curve\n";
        }
    }
  else // Binary
    {
      SER(rDB.WriteLong(lCount));
    }

  for (i=0; i<lCount; i++)
    {
      if (eType == SM_ASCII)
        {
          std::ostream & rFileOut = *rDB.GetOutStreamPtr();
          rFileOut << i;//Index
        }
      m_vEdgeuses[i].WriteToDB(lDBVersionNumber,rDB,bPersistAttribs);
    }

  // Edge Data
  lCount = m_vEdges.GetSize();
  if (eType == SM_ASCII)
    {
      std::ostream & rFileOut = *rDB.GetOutStreamPtr();
      rFileOut << "\n//Edge Data\n";
      rFileOut << lCount << " Edges In Brep\n";
      if(lDBVersionNumber > 30)
        {
          rFileOut << "//Index, Flags, UserId1, UserId2, Tol, Curve, Start Vertex, End Vertex, Primary Edgeuse, Param Min, Param Max\n";
        }
      else
        {
          rFileOut << "//Index, Tol, Curve, Start Vertex, End Vertex, Primary Edgeuse, Param Min, Param Max\n";
        }
    }
  else// Binary
    {
      SER(rDB.WriteLong(lCount));
    }

  for (i=0; i<lCount; i++)
    {
      if (eType == SM_ASCII)
        {
          std::ostream & rFileOut = *rDB.GetOutStreamPtr();
          rFileOut << i;//Index
        }
      m_vEdges[i].WriteToDB(lDBVersionNumber,rDB,bPersistAttribs);
    }

  // Vertex Data
  lCount = m_vVertices.GetSize();
  if (eType == SM_ASCII)
    {
      std::ostream & rFileOut = *rDB.GetOutStreamPtr();
      rFileOut << "\n//Vertex Data\n";
      rFileOut << lCount << " Vertices In Brep\n";
      if(lDBVersionNumber > 30)
        {
          rFileOut << "//Index, Flags, UserId1, UserId2, Tol, X, Y, Z\n";
        }
      else
        {
          rFileOut << "//Index, Tol, X, Y, Z\n";
        }
    }
  else // Binary
    {
      SER(rDB.WriteLong(lCount));
    }

  for (i=0; i<lCount; i++)
    {
      if (eType == SM_ASCII)
        {
          std::ostream & rFileOut = *rDB.GetOutStreamPtr();
          rFileOut << i;//Index
        }
      m_vVertices[i].WriteToDB(lDBVersionNumber,rDB,bPersistAttribs);
    }

  // set approx tolerance
  if(   bWriteAsBSplines == TRUE
     && sApproxTol3d == 0.0)
   {
     sApproxTol3d = m_sApproxTol3d ;
   }

  // UV-Curve Data
  lCount = m_vUVCurves.GetSize();
  if (eType == SM_ASCII)
    {
      std::ostream & rFileOut = *rDB.GetOutStreamPtr();
      rFileOut << "\n// UV-Curve Data\n";
      rFileOut << lCount << " UV-Curves In Brep\n";
    }
  else
    { // Binary
      SER(rDB.WriteLong(lCount));
    }

  for (i = 0; i < lCount; i++)
    {
      if (eType == SM_ASCII)
        {
          std::ostream & rFileOut = *rDB.GetOutStreamPtr();
          rFileOut << i << " - UV-Curve:\n";
        }

      // convert all 2d UV curves into x, y, NL_NOZ
      gw_CURVE *cur = ((SmBSplineCurve *)m_vUVCurves[i])->GetGwNurbPointer() ;
      gw_INDEX   jj, nn, mm;
      gw_DEGREE  pp;
      gw_REAL    *UU;
      gw_CPOINT  *Pww;
      gw_FLAG    SetNoz = 1;

      /* Get local notation */
      N_CrvGetCPtsDegreeAndKnots(cur, &nn, &Pww, &pp, &mm, &UU);


      /* Convert to 2D in place if all z = 0.0 */
      if (Pww[0].z != NL_NOZ)
        {
          for (jj = 0; jj <= nn; jj++)
            {
              if (Pww[jj].z != 0.0)
                {
                  SetNoz = 0;
                  jj = nn + 1;
                }
            }
           if (SetNoz == 1)
             {
               for (jj = 0; jj <= nn; jj++)
                 {
                   Pww[jj].z = NL_NOZ;
                 }
             }
        } // end z != NL_NOZ check

      SmBrepData::WriteCurveToDB(crContext, m_vUVCurves[i],
                                 lDBVersionNumber, rAllAttributes,
                                 rAttrMap, rDB,
                                 bWriteAsBSplines, sApproxTol3d, bPersistAttribs);
    } // end iter every m_vUVCurves[i]

  // 3D-Curve Data
  lCount = m_v3DCurves.GetSize();   
  if (eType == SM_ASCII)
    {
      std::ostream & rFileOut = *rDB.GetOutStreamPtr();
      rFileOut << "\n// 3D-Curve Data\n";
      rFileOut << lCount << " 3D-Curves In Brep\n";
    }
  else
    { // Binary
      SER(rDB.WriteLong(lCount));
    }

  for (i = 0; i < lCount; i++)
    {
      if (eType == SM_ASCII)
        {
          std::ostream & rFileOut = *rDB.GetOutStreamPtr();
          rFileOut << i << " - 3D-Curve\n";
        }
      SmBrepData::WriteCurveToDB(crContext, m_v3DCurves[i],
                                 lDBVersionNumber, rAllAttributes,
                                 rAttrMap, rDB,
                                 bWriteAsBSplines, sApproxTol3d, bPersistAttribs);
    } // end iter every m_v3DCurves[i]

  // Surface Data
  lCount = m_vSurfaces.GetSize();  
  if (eType == SM_ASCII)
    {
      std::ostream & rFileOut = *rDB.GetOutStreamPtr();
      rFileOut << "\n//Surface Data\n";
      rFileOut << lCount << " Surfaces In Brep\n";
    }
  else // Binary
    {
      SER(rDB.WriteLong(lCount));
    }

  for (i=0; i<lCount; i++)
    {
      if (eType == SM_ASCII)
        {
          std::ostream & rFileOut = *rDB.GetOutStreamPtr();
          rFileOut << i << " - Surface\n";
        }
      SmBrepData::WriteSurfaceToDB(crContext,m_vSurfaces[i],
                                   lDBVersionNumber,rAllAttributes,
                                   rAttrMap, rDB,
                                   bWriteAsBSplines, sApproxTol3d, bPersistAttribs);
    } // end iter every m_vSurfaces[i]

  // all done
  return SM_SUCCESS;

} // end SmBrepData::WriteToDB

// SmTopologyData Methods
/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
SmStatus SmTopologyData::ReadFromDB
  (ULONG          lDBVersionNumber, // in : target stream's version number
  SmDatabaseIO  & rDB,              // in : contains target stream
  SmBoolean       bPersistAttribs)  // NotUsed: in : TRUE = read attributes, false = don't, default:[TRUE]
{
  SM_REF1(bPersistAttribs) ;
  if (lDBVersionNumber > 31) // changed from 30 RCLxx
    {
      SmFileType eType = rDB.GetFileType();

      if (eType == SM_ASCII)
        {
          std::istream & rFileIn = *rDB.GetInStreamPtr();
          rFileIn >> m_lFlags >> m_lUserIndex1 >> m_lUserIndex2;
        }
      else // Binary
        {
          SER(rDB.ReadLong(m_lFlags));
          SER(rDB.ReadLong(m_lUserIndex1));
          SER(rDB.ReadLong(m_lUserIndex2));
        }

      if (lDBVersionNumber > 43) // for NewTol Model
        {
          if (eType == SM_ASCII)
            { 
#ifdef SM_USE_NEWTOL
              SM_NEWTOL_LINE rFileIn >> m_bIsSmallTopology ;
#endif // SM_USE_NEWTOL
            }
          else // Binary
            { 
#ifdef SM_USE_NEWTOL
              SM_NEWTOL_LINE SER(rDB.ReadBoolean(m_bIsSmallTopology));
#endif // SM_USE_NEWTOL
            }
        }
      else { 
#ifdef SM_USE_NEWTOL
              SM_NEWTOL_LINE m_bIsSmallTopology = FALSE ; 
#endif // SM_USE_NEWTOL
           }
    }
  else // lDBVersionNumber <= 31 branch
    {
      m_lFlags = 0;
      m_lUserIndex1 = 0;
      m_lUserIndex2 = 0;
    }
  return SM_SUCCESS;

} // end SmTopologyData::ReadFromDB

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
SmStatus SmTopologyData::WriteToDB
  (ULONG          lDBVersionNumber,  // in : target stream's version number
   SmDatabaseIO & rDB,               // in : contains target stream
   SmBoolean      bPersistAttribs)   // NotUsed: in : TRUE = write attributes, false = don't, default:[TRUE]
{
  SM_REF1(bPersistAttribs) ;
    SmFileType eType = rDB.GetFileType();

    if (lDBVersionNumber > 31)  // changed form 30 RCLxx
      {
        if (eType == SM_ASCII)
          {
            std::ostream & rFileOut = *rDB.GetOutStreamPtr();
            rFileOut << " " << m_lFlags;
            rFileOut << " " << m_lUserIndex1 << " " << m_lUserIndex2;
#ifdef SM_USE_NEWTOL
            SM_NEWTOL_LINE rFileOut << " " << m_bIsSmallTopology
#endif // SM_USE_NEWTOL
          }
        else // Binary
          {
            SER(rDB.WriteLong(m_lFlags));
            SER(rDB.WriteLong(m_lUserIndex1));
            SER(rDB.WriteLong(m_lUserIndex2));
#ifdef SM_USE_NEWTOL
            SM_NEWTOL_LINE( SER(rDB.WriteBoolean(m_bIsSmallTopology));
#endif // SM_USE_NEWTOL
          }
      }

    if (lDBVersionNumber > 43)  // NewTol changes
      {
        if (eType == SM_ASCII)
          { 
#ifdef SM_USE_NEWTOL
            SM_NEWTOL_LINE rFileOut << " " << m_bIsSmallTopology
#endif // SM_USE_NEWTOL
          }
        else // Binary
          { 
#ifdef SM_USE_NEWTOL
            SM_NEWTOL_LINE( SER(rDB.WriteBoolean(m_bIsSmallTopology));
#endif // SM_USE_NEWTOL
          }
      }

    return SM_SUCCESS;

} // end SmTopologyData::WriteToDB

// SmRegionData Methods
/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
SmStatus SmRegionData::ReadFromDB
 (const SmContext        & crContext,        // in : context for constructing objects from stream
  ULONG                    lDBVersionNumber, // in : target stream's version number
  SmTArray<SmAttribute*> & rAllAttributes,   // out: Array of all read attributes when lDBVersionNumber > 30
  SmDatabaseIO           & rDB,              // in : contains target stream
  SmBoolean                bPersistAttribs)  // in : TRUE = read attributes, false = don't, default:[TRUE]
{
  ULONG index;
  SmFileType eType = rDB.GetFileType();

  if (eType == SM_ASCII)
    {
      std::istream & rFileIn = *rDB.GetInStreamPtr();
      rFileIn >> index;
      SmTopologyData::ReadFromDB(lDBVersionNumber,rDB,bPersistAttribs);
      rFileIn >> m_lNumShells >> m_lStartShell >> m_bIsVoidFlag;
      GOTO_NEXT_LINE;
    }
  else // Binary
    {
      SmTopologyData::ReadFromDB(lDBVersionNumber,rDB,bPersistAttribs);
      SER(rDB.ReadLong(m_lNumShells));
      SER(rDB.ReadLong(m_lStartShell));
      SER(rDB.ReadBoolean(m_bIsVoidFlag));
    }

  if (lDBVersionNumber > 30 && bPersistAttribs == TRUE)
    {
      SmAttributeData::ReadIndexedAttributesFromDB(crContext,rAllAttributes,m_sAttributes,rDB);
    }

  return SM_SUCCESS;

} // end SmRegionData::ReadFromDB

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
SmStatus SmRegionData::WriteToDB
  (ULONG          lDBVersionNumber,  // in : target stream's version number
   SmDatabaseIO & rDB,               // in : contains target stream
   SmBoolean      bPersistAttribs)   // in : TRUE = write attributes, false = don't, default:[TRUE]
{
    SmFileType eType = rDB.GetFileType();

    SmTopologyData::WriteToDB(lDBVersionNumber,rDB,bPersistAttribs);

    if (eType == SM_ASCII)
      {
        std::ostream & rFileOut = *rDB.GetOutStreamPtr();
        rFileOut << " " << m_lNumShells << " " << m_lStartShell << " " << m_bIsVoidFlag << "\n";
      }
    else // Binary
      {
        SER(rDB.WriteLong(m_lNumShells));
        SER(rDB.WriteLong(m_lStartShell));
        SER(rDB.WriteBoolean(m_bIsVoidFlag));
      }

    if (lDBVersionNumber > 30 && bPersistAttribs == TRUE)
      {
        SmAttributeData::WriteIndexedAttributesToDB(m_sAttributes,rDB);
      }
    return SM_SUCCESS;

} // end SmRegionData::WriteToDB

// SmShellData Methods
/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
SmStatus SmShellData::ReadFromDB
 (const SmContext        & crContext,        // in : context for constructing objects from stream
  ULONG                    lDBVersionNumber, // in : target stream's version number
  SmTArray<SmAttribute*> & rAllAttributes,   // out: Array of all read attributes when lDBVersionNumber > 30
  SmDatabaseIO           & rDB,              // in : contains target stream
  SmBoolean                bPersistAttribs)  // in : TRUE = read attributes, false = don't, default:[TRUE]
{
  ULONG index;
  SmFileType eType = rDB.GetFileType();
  SmBoolean bOrientation = 1 ; // used to remove m_bOrientation - not needed for shells - kept to keep smb file structure constant

  if (eType == SM_ASCII)
    {
      std::istream & rFileIn = *rDB.GetInStreamPtr();
      rFileIn >> index;
      SmTopologyData::ReadFromDB(lDBVersionNumber,rDB,bPersistAttribs);
      rFileIn >> m_lShellType;
      switch (m_lShellType)
        {
          case 0: // Faceuse
              rFileIn >> bOrientation >> m_lNumFaceuses >> m_lFaceuseStart;
              break;
          case 1: // Wire Edge
              rFileIn >> m_lEdge;
              break;
          case 2: // Vertex
              rFileIn >> m_lVertex;
              break;
        }
      GOTO_NEXT_LINE;
    }
  else // Binary
    {
      SmTopologyData::ReadFromDB(lDBVersionNumber,rDB,bPersistAttribs);
      SER(rDB.ReadLong(m_lShellType));
      switch (m_lShellType)
        {
          case 0: // Faceuse
              SER(rDB.ReadLong(m_lNumFaceuses));
              SER(rDB.ReadLong(m_lFaceuseStart));
              SER(rDB.ReadBoolean(bOrientation));  // for backward compatibility - read value into a local variable that no longer exists in the SmShellData obj
              break;
          case 1: // Wire Edge
              SER(rDB.ReadLong(m_lEdge));
              break;
          case 2: // Vertex
              SER(rDB.ReadLong(m_lVertex));
              break;
        }
    }

  if (lDBVersionNumber > 30 && bPersistAttribs == TRUE)
    {
      SmAttributeData::ReadIndexedAttributesFromDB(crContext,rAllAttributes,m_sAttributes,rDB);
    }

  return SM_SUCCESS;

} // end SmShellData::ReadFromDB

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
SmStatus SmShellData::WriteToDB
  (ULONG          lDBVersionNumber,  // in : target stream's version number
   SmDatabaseIO & rDB,               // in : contains target stream
   SmBoolean      bPersistAttribs)   // in : TRUE = write attributes, false = don't, default:[TRUE]
{
    SmTopologyData::WriteToDB(lDBVersionNumber,rDB,bPersistAttribs);
    SmBoolean bOrientation = 1 ; // used to remove m_bOrientation - not needed for shells - kept to keep smb file structure constant

    SmFileType eType = rDB.GetFileType();
    if (eType == SM_ASCII) 
      {
        std::ostream & rFileOut = *rDB.GetOutStreamPtr();
        switch (m_lShellType) 
          {
            case 0: // Faceuse
             // for backwared compatibility - writing bOrientation value preserves the smb file structure even though SmShellData no longer has m_bOrientation
                rFileOut << " " << m_lShellType << " " << bOrientation
                    << " " << m_lNumFaceuses << " " << m_lFaceuseStart << "\n";
                break;
            case 1: // Wire Edge
                rFileOut << " " << m_lShellType << " " << m_lEdge << "\n";
                break;
            case 2: // Vertex
                rFileOut << " " << m_lShellType << " " << m_lVertex << "\n";
                break;
          }
      }
    else // Binary  
      {
        switch (m_lShellType) 
          {
            case 0: // Faceuse
                SER(rDB.WriteLong(m_lShellType));
                SER(rDB.WriteLong(m_lNumFaceuses));
                SER(rDB.WriteLong(m_lFaceuseStart));
                SER(rDB.WriteBoolean(bOrientation));  // gwc:replace m_bOrientation with bOrientation
                break;
            case 1: // Wire Edge
                SER(rDB.WriteLong(m_lShellType));
                SER(rDB.WriteLong(m_lEdge));
                break;
            case 2: // Vertex
                SER(rDB.WriteLong(m_lShellType));
                SER(rDB.WriteLong(m_lVertex));
                break;
          }
      }
    if (lDBVersionNumber > 30 && bPersistAttribs == TRUE) 
      {
        SmAttributeData::WriteIndexedAttributesToDB(m_sAttributes,rDB);
      }

    return SM_SUCCESS;

} // end SmShellData::WriteToDB

// SmFaceuseData Methods
/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
SmStatus SmFaceuseData::ReadFromDB
 (const SmContext        & crContext,        // in : context for constructing objects from stream
  ULONG                    lDBVersionNumber, // in : target stream's version number
  SmTArray<SmAttribute*> & rAllAttributes,   // out: Array of all read attributes when lDBVersionNumber > 30
  SmDatabaseIO           & rDB,              // in : contains target stream
  SmBoolean                bPersistAttribs)  // in : TRUE = read attributes, false = don't, default:[TRUE]
{
  SmFileType eType = rDB.GetFileType();
  ULONG index;
  if (eType == SM_ASCII)
    {
      std::istream & rFileIn = *rDB.GetInStreamPtr();
      rFileIn >> index;
      SmTopologyData::ReadFromDB(lDBVersionNumber,rDB,bPersistAttribs);
      rFileIn >> m_lFace >> m_bOrientation;
      GOTO_NEXT_LINE;
    }
  else
    {// Binary
      SmTopologyData::ReadFromDB(lDBVersionNumber,rDB,bPersistAttribs);
      SER(rDB.ReadLong(m_lFace));
      SER(rDB.ReadBoolean(m_bOrientation));
    }

  if (lDBVersionNumber > 30 && bPersistAttribs == TRUE)
    {
      SmAttributeData::ReadIndexedAttributesFromDB(crContext,rAllAttributes,m_sAttributes,rDB);
    }

  return SM_SUCCESS;

} // end SmFaceuseData::ReadFromDB

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
SmStatus SmFaceuseData::WriteToDB
  (ULONG          lDBVersionNumber,  // in : target stream's version number
   SmDatabaseIO & rDB,               // in : contains target stream
   SmBoolean      bPersistAttribs)   // in : TRUE = write attributes, false = don't, default:[TRUE]
{
  SmTopologyData::WriteToDB(lDBVersionNumber,rDB,bPersistAttribs);

  SmFileType eType = rDB.GetFileType();
  if (eType == SM_ASCII)
    {
      std::ostream & rFileOut = *rDB.GetOutStreamPtr();
      rFileOut << " " << m_lFace << " " << m_bOrientation << "\n";
    }
  else // Binary
    {
      SER(rDB.WriteLong(m_lFace));
      SER(rDB.WriteBoolean(m_bOrientation));
    }

  if (lDBVersionNumber > 30 && bPersistAttribs == TRUE)
    {
      SmAttributeData::WriteIndexedAttributesToDB(m_sAttributes,rDB);
    }
  return SM_SUCCESS;

} // end SmFaceuseData::WriteToDB

// SmFaceData Methods
/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
SmStatus SmFaceData::ReadFromDB
 (const SmContext        & crContext,        // in : context for constructing objects from stream
  ULONG                    lDBVersionNumber, // in : target stream's version number
  SmTArray<SmAttribute*> & rAllAttributes,   // out: Array of all read attributes when lDBVersionNumber > 30
  SmDatabaseIO           & rDB,              // in : contains target stream
  SmBoolean                bPersistAttribs)  // in : TRUE = read attributes, false = don't, default:[TRUE]
{
  SmFileType eType = rDB.GetFileType();
  ULONG index;
  SmPoint2d sMin;
  SmPoint2d sMax;
  if (eType == SM_ASCII)
    {
      std::istream & rFileIn = *rDB.GetInStreamPtr();
      rFileIn >> index ;
      SmTopologyData::ReadFromDB(lDBVersionNumber,rDB,bPersistAttribs);

#ifdef SM_USE_NEWTOL
      SM_NEWTOL_LINE double dDummyVal ;
      SM_NEWTOL_LINE rFileIn >> dDummyVal ;     // not used in NEWTOL modeler
#else // SM_USE_OLDTOL
      SM_OLDTOL_LINE rFileIn >> m_sZoneTol3d ;  // renamed from m_dTol with lDBVersionNumber == 38
#endif // SM_USE_OLDTOL

      rFileIn >> m_lSurface ;
      rFileIn >> m_lStartLoop ;
      rFileIn >> m_lNumLoops ;
      rFileIn >> m_bRectangularTrim ;
      GOTO_NEXT_LINE;
      rFileIn >> sMin.x ;
      rFileIn >> sMax.x ;
      rFileIn >> sMin.y ;
      rFileIn >> sMax.y ;
      GOTO_NEXT_LINE;
    }
  else // Binary
    {
      SmTopologyData::ReadFromDB(lDBVersionNumber,rDB,bPersistAttribs);
                                 SER(rDB.ReadDouble(sMin.x));
                                 SER(rDB.ReadDouble(sMax.x));
                                 SER(rDB.ReadDouble(sMin.y));
                                 SER(rDB.ReadDouble(sMax.y));

#ifdef SM_USE_NEWTOL
                                 SM_NEWTOL_LINE double dDummyVal ;
                                 SM_NEWTOL_LINE SER(rDB.ReadDouble(dDummyVal));     // not used in NEWTOL modeler
#else // SM_USE_OLDTOL
                                 SM_OLDTOL_LINE SER(rDB.ReadDouble(m_sZoneTol3d));  // renamed from m_dTol with lDBVersionNumber == 38
#endif // SM_USE_OLDTOL

                                 SER(rDB.ReadLong(m_lSurface));
                                 SER(rDB.ReadLong(m_lStartLoop));
                                 SER(rDB.ReadLong(m_lNumLoops));
                                 SER(rDB.ReadBoolean(m_bRectangularTrim));
    }

  m_vUVDomain.SetMinMax(sMin, sMax);

  if (lDBVersionNumber > 30 && bPersistAttribs == TRUE)
    {
      SmAttributeData::ReadIndexedAttributesFromDB(crContext,rAllAttributes,m_sAttributes,rDB);
    }

  return SM_SUCCESS;

} // end SmFaceData::ReadFromDB

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
SmStatus SmFaceData::WriteToDB
  (ULONG          lDBVersionNumber,  // in : target stream's version number
   SmDatabaseIO & rDB,               // in : contains target stream
   SmBoolean      bPersistAttribs)   // in : TRUE = write attributes, false = don't, default:[TRUE]
{
  SmTopologyData::WriteToDB(lDBVersionNumber,rDB,bPersistAttribs);

  SmFileType eType = rDB.GetFileType();
  SmPoint2d sMin = m_vUVDomain.GetMin();
  SmPoint2d sMax = m_vUVDomain.GetMax();
  if (eType == SM_ASCII)
    {
      std::ostream & rFileOut = *rDB.GetOutStreamPtr();

#ifdef SM_USE_NEWTOL
      SM_NEWTOL_LINE rFileOut << " " << 0.0 ;          // dummy holder - not used in NEWTOL model
#else  // SM_USE_OLDTOL
      SM_OLDTOL_LINE rFileOut << " " << m_sZoneTol3d ; // renamed from m_dTol with lDBVersionNumber == 38
#endif // SM_USE_OLDTOL
      rFileOut << " "  << m_lSurface ;
      rFileOut << " "  << m_lStartLoop ;
      rFileOut << " "  << m_lNumLoops ;
      rFileOut << " "  << m_bRectangularTrim ;
      rFileOut << "\n" << sMin.x ;
      rFileOut << " "  << sMax.x ;
      rFileOut << " "  << sMin.y ;
      rFileOut << " "  << sMax.y ;
      rFileOut << "\n" ;
    }
  else // Binary
    {
      SER(rDB.WriteDouble(sMin.x));
      SER(rDB.WriteDouble(sMax.x));
      SER(rDB.WriteDouble(sMin.y));
      SER(rDB.WriteDouble(sMax.y));

#ifdef SM_USE_NEWTOL
      SM_NEWTOL_LINE SER(rDB.WriteDouble(0.0));            //  dummy holder - not used in NEWTOL model
#else  // SM_USE_OLDTOL
      SM_OLDTOL_LINE SER(rDB.WriteDouble(m_sZoneTol3d));   // renamed from m_dTol with lDBVersionNumber == 38
#endif // SM_USE_OLDTOL

      SER(rDB.WriteLong(m_lSurface));
      SER(rDB.WriteLong(m_lStartLoop));
      SER(rDB.WriteLong(m_lNumLoops));
      SER(rDB.WriteBoolean(m_bRectangularTrim));
    }

  if (lDBVersionNumber > 30 && bPersistAttribs == TRUE)
    {
      SmAttributeData::WriteIndexedAttributesToDB(m_sAttributes,rDB);
    }
  return SM_SUCCESS;

} // end SmFaceData::WriteToDB

// SmLoopData Methods
/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
SmStatus SmLoopData::ReadFromDB
 (const SmContext        & crContext,        // in : context for constructing objects from stream
  ULONG                    lDBVersionNumber, // in : target stream's version number
  SmTArray<SmAttribute*> & rAllAttributes,   // out: Array of all read attributes when lDBVersionNumber > 30
  SmDatabaseIO           & rDB,              // in : contains target stream
  SmBoolean                bPersistAttribs)  // in : TRUE = read attributes, false = don't, default:[TRUE]
{
  SmFileType eType = rDB.GetFileType();
  ULONG index;
  if (eType == SM_ASCII)
    {
      std::istream & rFileIn = *rDB.GetInStreamPtr();
      rFileIn >> index;
      SmTopologyData::ReadFromDB(lDBVersionNumber,rDB,bPersistAttribs);
      rFileIn >> m_lLoopType;
      switch (m_lLoopType)
        {
          case 0: // Edgeuse
              rFileIn >> m_lNumEU >> m_lStartEU;
              break;
          case 1: // Vertex
              rFileIn >> m_lVertex;
              break;
        }
      GOTO_NEXT_LINE;
    }
  else
    {// Binary
      SmTopologyData::ReadFromDB(lDBVersionNumber,rDB,bPersistAttribs);
      SER(rDB.ReadLong(m_lLoopType));
      switch (m_lLoopType)
        {
          case 0: // Edgeuse
              SER(rDB.ReadLong(m_lNumEU));
              SER(rDB.ReadLong(m_lStartEU));
              break;
          case 1: // Vertex
              SER(rDB.ReadLong(m_lVertex));
              break;
        }
    }

  if (lDBVersionNumber > 30 && bPersistAttribs == TRUE)
    {
      SmAttributeData::ReadIndexedAttributesFromDB(crContext,rAllAttributes,m_sAttributes,rDB);
      SmAttributeData::ReadIndexedAttributesFromDB(crContext,rAllAttributes,m_sLUUpperAttributes,rDB);
      SmAttributeData::ReadIndexedAttributesFromDB(crContext,rAllAttributes,m_sLULowerAttributes,rDB);
    }
  return SM_SUCCESS;

} // SmLoopData::ReadFromDB

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
SmStatus SmLoopData::WriteToDB
  (ULONG          lDBVersionNumber,  // in : target stream's version number
   SmDatabaseIO & rDB,               // in : contains target stream
   SmBoolean      bPersistAttribs)   // in : TRUE = write attributes, false = don't, default:[TRUE]
{
  SmTopologyData::WriteToDB(lDBVersionNumber,rDB,bPersistAttribs);

  SmFileType eType = rDB.GetFileType();
  if (eType == SM_ASCII)
    {
      std::ostream & rFileOut = *rDB.GetOutStreamPtr();
      switch (m_lLoopType)
        {
          case 0: // Edgeuse
              rFileOut << " " << m_lLoopType << " " << m_lNumEU << " " << m_lStartEU << "\n";
              break;
          case 1: // Vertex
              rFileOut << " " << m_lLoopType << " " << m_lVertex << "\n";
              break;
        }
    }
  else
    {// Binary
      switch (m_lLoopType)
        {
          case 0: // Edgeuse
              SER(rDB.WriteLong(m_lLoopType));
              SER(rDB.WriteLong(m_lNumEU));
              SER(rDB.WriteLong(m_lStartEU));
              break;
          case 1: // Vertex
              SER(rDB.WriteLong(m_lLoopType));
              SER(rDB.WriteLong(m_lVertex));
              break;
        }
    }

  if (lDBVersionNumber > 30 && bPersistAttribs == TRUE)
    {
      SmAttributeData::WriteIndexedAttributesToDB(m_sAttributes,rDB);
      SmAttributeData::WriteIndexedAttributesToDB(m_sLUUpperAttributes,rDB);
      SmAttributeData::WriteIndexedAttributesToDB(m_sLULowerAttributes,rDB);
    }
  return SM_SUCCESS;

} // end SmLoopData::WriteToDB

// SmEUData Methods
/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
SmStatus SmEUData::ReadFromDB
 (const SmContext        & crContext,        // in : context for constructing objects from stream
  ULONG                    lDBVersionNumber, // in : target stream's version number
  SmTArray<SmAttribute*> & rAllAttributes,   // out: Array of all read attributes when lDBVersionNumber > 30
  SmDatabaseIO           & rDB,              // in : contains target stream
  SmBoolean                bPersistAttribs)  // in : TRUE = read attributes, false = don't, default:[TRUE]
{
  SmFileType eType = rDB.GetFileType();
  ULONG index;
  if (eType == SM_ASCII)
    {
      std::istream & rFileIn = *rDB.GetInStreamPtr();
      rFileIn >> index;
    }

  SmTopologyData::ReadFromDB(lDBVersionNumber,rDB,bPersistAttribs);

  if (lDBVersionNumber > 31)
    { // changed from 30 RCLxx
      if (eType == SM_ASCII)
        {
          std::istream & rFileIn = *rDB.GetInStreamPtr();
          rFileIn >> m_lMateFlags >> m_lMateUserIndex1 >> m_lMateUserIndex2;
        }
      else // Binary
        {
          SER(rDB.ReadLong(m_lMateFlags));
          SER(rDB.ReadLong(m_lMateUserIndex1));
          SER(rDB.ReadLong(m_lMateUserIndex2));
        }
    }
  else
    {
      m_lMateFlags = 0;
      m_lMateUserIndex1 = 0;
      m_lMateUserIndex2 = 0;
    }

  if (eType == SM_ASCII)
    {
      std::istream & rFileIn = *rDB.GetInStreamPtr();
      rFileIn >> m_lEUType;
      switch (m_lEUType)
        {
          case 0: // Edge
              rFileIn >> m_lEdge >> m_lLoop >> m_bOrientation >> m_lUVCurve >> m_lNextEU  >> m_lMateNextEU;
              break;
          case 1: // Vertex
              rFileIn >> m_lVertex >> m_lLoop >> m_bOrientation >> m_lUVCurve;
              break;
          case 2: // Wire Edge
              rFileIn >> m_lEdge >> m_lShell >> m_bOrientation >> m_lUVCurve;
              break;
          case 3: // Vertex At Pole
              rFileIn >> m_lVertex >> m_lLoop >> m_bOrientation >> m_lUVCurve;
              break;
        }
      GOTO_NEXT_LINE;
    }
  else
    {// Binary
      SER(rDB.ReadLong(m_lEUType));
      switch (m_lEUType)
        {
          case 0: // Edge
              SER(rDB.ReadLong(m_lEdge));
              SER(rDB.ReadLong(m_lLoop));
              break;
          case 1: // Vertex
              SER(rDB.ReadLong(m_lVertex));
              SER(rDB.ReadLong(m_lLoop));
              break;
          case 2: // Wire Edge
              SER(rDB.ReadLong(m_lEdge));
              SER(rDB.ReadLong(m_lShell));
              break;
          case 3: // Vertex At Pole
              SER(rDB.ReadLong(m_lVertex));
              SER(rDB.ReadLong(m_lLoop));
              break;
        }
      SER(rDB.ReadBoolean(m_bOrientation));
      SER(rDB.ReadLong(m_lUVCurve));
      if (m_lEUType == 0)
        {
          SER(rDB.ReadLong(m_lNextEU));
          SER(rDB.ReadLong(m_lMateNextEU));
        }
    }

  if (lDBVersionNumber > 30 && bPersistAttribs == TRUE)
    {
      SmAttributeData::ReadIndexedAttributesFromDB(crContext,rAllAttributes,m_sAttributes,rDB);
      SmAttributeData::ReadIndexedAttributesFromDB(crContext,rAllAttributes,m_sMateAttributes,rDB);
    }
  return SM_SUCCESS;

} // end SmEUData::ReadFromDB

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
SmStatus SmEUData::WriteToDB
  (ULONG          lDBVersionNumber,  // in : target stream's version number
   SmDatabaseIO & rDB,               // in : contains target stream
   SmBoolean      bPersistAttribs)   // in : TRUE = write attributes, false = don't, default:[TRUE]
{
  SmTopologyData::WriteToDB(lDBVersionNumber,rDB,bPersistAttribs);
  SmFileType eType = rDB.GetFileType();

  if (lDBVersionNumber > 31)
    {  // changed from 30  RCLxx
      if (eType == SM_ASCII)
        {
          std::ostream & rFileOut = *rDB.GetOutStreamPtr();
          rFileOut << " " << m_lFlags;
          rFileOut << " " << m_lUserIndex1 << " " << m_lUserIndex2;
        }
      else // Binary
        {
          SER(rDB.WriteLong(m_lFlags));
          SER(rDB.WriteLong(m_lUserIndex1));
          SER(rDB.WriteLong(m_lUserIndex2));
        }
    }

  if (eType == SM_ASCII)
    {
      std::ostream & rFileOut = *rDB.GetOutStreamPtr();
      switch (m_lEUType)
        {
          case 0: // Edge
              rFileOut << " " << m_lEUType      << " " << m_lEdge       << " " << m_lLoop << " "
                              << m_bOrientation << " " << m_lUVCurve    << " "
                              << m_lNextEU      << " " << m_lMateNextEU << "\n";
              break;
          case 1: // Vertex
              rFileOut << " " << m_lEUType      << " " << m_lVertex  << " " << m_lLoop << " "
                              << m_bOrientation << " " << m_lUVCurve << "\n";
              break;
          case 2: // Wire Edge
              rFileOut << " " << m_lEUType      << " " << m_lEdge    << " " << m_lShell << " "
                              << m_bOrientation << " " << m_lUVCurve << "\n";
              break;
          case 3: // Vertex At Pole
              rFileOut << " " << m_lEUType      << " " << m_lVertex << " " << m_lLoop << " "
                              << m_bOrientation << " " << m_lUVCurve << "\n";
              break;
        }
    }
  else // Binary
    {
      SER(rDB.WriteLong(m_lEUType));
      switch (m_lEUType)
        {
          case 0: // Edge
              SER(rDB.WriteLong(m_lEdge));
              SER(rDB.WriteLong(m_lLoop));
              break;
          case 1: // Vertex
              SER(rDB.WriteLong(m_lVertex));
              SER(rDB.WriteLong(m_lLoop));
              break;
          case 2: // Wire Edge
              SER(rDB.WriteLong(m_lEdge));
              SER(rDB.WriteLong(m_lShell));
              break;
          case 3: // Vertex At Pole
              SER(rDB.WriteLong(m_lVertex));
              SER(rDB.WriteLong(m_lLoop));
              break;
        }
      SER(rDB.WriteBoolean(m_bOrientation));
      SER(rDB.WriteLong(m_lUVCurve));
      if (m_lEUType == 0)
        {
          SER(rDB.WriteLong(m_lNextEU));
          SER(rDB.WriteLong(m_lMateNextEU));
        }
    }

  if (lDBVersionNumber > 30 && bPersistAttribs == TRUE)
    {
      SmAttributeData::WriteIndexedAttributesToDB(m_sAttributes,rDB);
      SmAttributeData::WriteIndexedAttributesToDB(m_sMateAttributes,rDB);
    }
  return SM_SUCCESS;

} // end SmEUData::WriteToDB

// SmEdgeData Methods
/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
SmStatus SmEdgeData::ReadFromDB
 (const SmContext        & crContext,        // in : context for constructing objects from stream
  ULONG                    lDBVersionNumber, // in : target stream's version number
  SmTArray<SmAttribute*> & rAllAttributes,   // out: Array of all read attributes when lDBVersionNumber > 30
  SmDatabaseIO           & rDB,              // in : contains target stream
  SmBoolean                bPersistAttribs)  // in : TRUE = read attributes, false = don't, default:[TRUE]
{
  SmFileType eType = rDB.GetFileType();
  double dMin, dMax;
  ULONG index;
  if (eType == SM_ASCII)
    {
      std::istream & rFileIn = *rDB.GetInStreamPtr();
      rFileIn >> index;
      SmTopologyData::ReadFromDB(lDBVersionNumber,rDB,bPersistAttribs);

#ifdef SM_USE_NEWTOL
      SM_NEWTOL_LINE double dDummyVal ;
      SM_NEWTOL_LINE rFileIn >> dDummyVal ;     // not used in NEWTOL modeler
#else // SM_USE_OLDTOL
      SM_OLDTOL_LINE rFileIn >> m_sZoneTol3d ;  // renamed from m_dTol with lDBVersionNumber == 38
#endif // SM_USE_OLDTOL

      rFileIn >> m_lCurve ;
      rFileIn >> m_lStartVertex ;
      rFileIn >> m_lEndVertex ;
      rFileIn >> m_lPrimEU ;
      rFileIn >> dMin ;
      rFileIn >> dMax ;
      GOTO_NEXT_LINE;
    }
  else // Binary
    {
      SmTopologyData::ReadFromDB(lDBVersionNumber,rDB,bPersistAttribs);

#ifdef SM_USE_NEWTOL
      SM_NEWTOL_LINE double dDummyVal ;
      SM_NEWTOL_LINE SER(rDB.ReadDouble(dDummyVal));     // not used in NEWTOL modeler
#else // SM_USE_OLDTOL
      SM_OLDTOL_LINE SER(rDB.ReadDouble(m_sZoneTol3d));  // renamed from m_dTol with lDBVersionNumber == 38
#endif // SM_USE_OLDTOL

      SER(rDB.ReadLong(m_lCurve));
      SER(rDB.ReadLong(m_lStartVertex));
      SER(rDB.ReadLong(m_lEndVertex));
      SER(rDB.ReadLong(m_lPrimEU));
      SER(rDB.ReadDouble(dMin));
      SER(rDB.ReadDouble(dMax));
    }

  m_vInterval.SetMinMax(dMin, dMax);

  if (lDBVersionNumber > 30 && bPersistAttribs == TRUE)
    {
      SmAttributeData::ReadIndexedAttributesFromDB(crContext,rAllAttributes,m_sAttributes,rDB);
    }

  return SM_SUCCESS;

} // end SmEdgeData::ReadFromDB

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
SmStatus SmEdgeData::WriteToDB
  (ULONG          lDBVersionNumber,  // in : target stream's version number
   SmDatabaseIO & rDB,               // in : contains target stream
   SmBoolean      bPersistAttribs)   // in : TRUE = write attributes, false = don't, default:[TRUE]
{
  SmTopologyData::WriteToDB(lDBVersionNumber,rDB,bPersistAttribs);

  SmFileType eType = rDB.GetFileType();
  double dMin = m_vInterval.GetMin();
  double dMax = m_vInterval.GetMax();
  if (eType == SM_ASCII)
    {
      std::ostream & rFileOut = *rDB.GetOutStreamPtr();

#ifdef SM_USE_NEWTOL
      SM_NEWTOL_LINE rFileOut << " " << 0.0 ;          // dummy holder - not used in NEWTOL model
#else  // SM_USE_OLDTOL
      SM_OLDTOL_LINE rFileOut << " " << m_sZoneTol3d ; // renamed from m_dTol with lDBVersionNumber == 38
#endif // SM_USE_OLDTOL

      rFileOut << " " << m_lCurve ;
      rFileOut << " " << m_lStartVertex ;
      rFileOut << " " << m_lEndVertex ;
      rFileOut << " " << m_lPrimEU ;
      rFileOut << " " << dMin ;
      rFileOut << " " << dMax ;
      rFileOut << "\n";
    }
  else // Binary
    {
#ifdef SM_USE_NEWTOL
      SM_NEWTOL_LINE SER(rDB.WriteDouble(0.0)) ;          // dummy holder - not used in NEWTOL model
#else  // SM_USE_OLDTOL
      SM_OLDTOL_LINE SER(rDB.WriteDouble(m_sZoneTol3d)) ; // renamed from m_dTol with lDBVersionNumber == 38
#endif // SM_USE_OLDTOL

      SER(rDB.WriteLong(m_lCurve));
      SER(rDB.WriteLong(m_lStartVertex));
      SER(rDB.WriteLong(m_lEndVertex));
      SER(rDB.WriteLong(m_lPrimEU));
      SER(rDB.WriteDouble(dMin));
      SER(rDB.WriteDouble(dMax));
    }

  if (lDBVersionNumber > 30 && bPersistAttribs == TRUE)
    {
      SmAttributeData::WriteIndexedAttributesToDB(m_sAttributes,rDB);
    }
  return SM_SUCCESS;

} // end SmEdgeData::WriteToDB

// SmVertexData Methods
/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
SmStatus SmVertexData::ReadFromDB
 (const SmContext        & crContext,        // in : context for constructing objects from stream
  ULONG                    lDBVersionNumber, // in : target stream's version number
  SmTArray<SmAttribute*> & rAllAttributes,   // out: Array of all read attributes when lDBVersionNumber > 30
  SmDatabaseIO           & rDB,              // in : contains target stream
  SmBoolean                bPersistAttribs)  // in : TRUE = read attributes, false = don't, default:[TRUE]
{
  SmFileType eType = rDB.GetFileType() ;
  ULONG index;
  if (eType == SM_ASCII)
    {
      std::istream & rFileIn = *rDB.GetInStreamPtr() ;
      rFileIn >> index ;
      SmTopologyData::ReadFromDB(lDBVersionNumber,rDB,bPersistAttribs) ;

#ifdef SM_USE_NEWTOL
      SM_NEWTOL_LINE double dDummyVal ;
      SM_NEWTOL_LINE rFileIn >> dDummyVal ;     // not used in NEWTOL modeler
#else // SM_USE_OLDTOL
      SM_OLDTOL_LINE rFileIn >> m_sZoneTol3d ;  // renamed from m_dTol with lDBVersionNumber == 38
#endif // SM_USE_OLDTOL

      rFileIn >> m_vPoint.x ;
      rFileIn >> m_vPoint.y ;
      rFileIn >> m_vPoint.z ;
      GOTO_NEXT_LINE;
    }
  else // Binary
    {
      SmTopologyData::ReadFromDB(lDBVersionNumber,rDB,bPersistAttribs) ;

#ifdef SM_USE_NEWTOL
      SM_NEWTOL_LINE double dDummyVal ;
      SM_NEWTOL_LINE SER(rDB.ReadDouble(dDummyVal));     // not used in NEWTOL modeler
#else // SM_USE_OLDTOL
      SM_OLDTOL_LINE SER(rDB.ReadDouble(m_sZoneTol3d));  // renamed from m_dTol with lDBVersionNumber == 38
#endif // SM_USE_OLDTOL

      SER(rDB.ReadDouble(m_vPoint.x)) ;
      SER(rDB.ReadDouble(m_vPoint.y)) ;
      SER(rDB.ReadDouble(m_vPoint.z)) ;
    }

  if (lDBVersionNumber > 30 && bPersistAttribs == TRUE)
    {
      SmAttributeData::ReadIndexedAttributesFromDB(crContext,rAllAttributes,m_sAttributes,rDB) ;
    }
  return SM_SUCCESS ;

} // end SmVertexData::ReadFromDB

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
SmStatus SmVertexData::WriteToDB
  (ULONG          lDBVersionNumber,  // in : target stream's version number
   SmDatabaseIO & rDB,               // in : contains target stream
   SmBoolean      bPersistAttribs)   // in : TRUE = write attributes, false = don't, default:[TRUE]
{
  SmTopologyData::WriteToDB(lDBVersionNumber,rDB,bPersistAttribs) ;

  SmFileType eType = rDB.GetFileType() ;
  if (eType == SM_ASCII)
    {
      std::ostream & rFileOut = *rDB.GetOutStreamPtr() ;

#ifdef SM_USE_NEWTOL
      SM_NEWTOL_LINE rFileOut << " " << 0.0 ;          // dummy holder - not used in NEWTOL model
#else  // SM_USE_OLDTOL
      SM_OLDTOL_LINE rFileOut << " " << m_sZoneTol3d ; // renamed from m_dTol with lDBVersionNumber == 38
#endif // SM_USE_OLDTOL

      rFileOut << " "  << m_vPoint.x ;
      rFileOut << " "  << m_vPoint.y ;
      rFileOut << " "  << m_vPoint.z ;
      rFileOut << "\n" ;
    }
  else // Binary
    {
#ifdef SM_USE_NEWTOL
      SM_NEWTOL_LINE SER(rDB.WriteDouble(0.0)) ;          // dummy holder - not used in NEWTOL model
#else  // SM_USE_OLDTOL
      SM_OLDTOL_LINE SER(rDB.WriteDouble(m_sZoneTol3d)) ; // renamed from m_dTol with lDBVersionNumber == 38
#endif // SM_USE_OLDTOL

      SER(rDB.WriteDouble(m_vPoint.x)) ;
      SER(rDB.WriteDouble(m_vPoint.y)) ;
      SER(rDB.WriteDouble(m_vPoint.z)) ;
    }

  if (lDBVersionNumber > 30 && bPersistAttribs == TRUE)
    {
      SmAttributeData::WriteIndexedAttributesToDB(m_sAttributes,rDB) ;
    }

  return SM_SUCCESS ;

} // end SmVertexData::WriteToDB

/*******************************************************************//**
PURPOSE: Read one curve from stream.

NOTES:
  1. Attribute array rAllAttributes is augmented with curve's attributes
     when lDBVersionNumber > 30.
  2. Composite curves are not approximated

***********************************************************************/
SmStatus SmBrepData::ReadCurveFromDB
  (const SmContext        & crContext,        // in : context for new object construction
   ULONG                    lDim,             // in : size of each control Point, i.e. 2 for 2d
   ULONG                    lDBVersionNumber, // in : controls backward compatible reads
   SmTArray<SmAttribute*> & rAllAttributes,   // i/o: augmented with curve's attributes when lDBVersionNumber > 30
   SmCurve               *& rpNewCurve,       // out: SmCurve built from SmCurve Data
   SmDatabaseIO           & rDB,              // in : contains target stream
   SmBoolean                bPersistAttribs)  // in : TRUE = read attributes, false = don't, default:[TRUE]
{
    // init output
    rpNewCurve = NULL ;

    // local file type
    SmFileType eType = rDB.GetFileType();

    //      if (eType == SM_ASCII)
    //        {
    //          std::istream & rFileIn = *rDB.GetInStreamPtr();
    //          char sBuff[MAXSIZE];
    //          rFileIn >> sBuff >> sBuff >> lType; // Curve Type:
    //          GOTO_NEXT_LINE;
    //        }
    //      else // Binary
    //        {
    //          SER(rDB.ReadLong(lType));
    //        }

    // branch on database version number
    if(lDBVersionNumber > 32)
      {
        // read curve type
        SM_TYPE lReadType;
        ULONG   lReadDim ;
        SER(rDB.ReadType(lReadType, &lReadDim)) ;

        // pass the call to appropriate derived class ReadFromDB call
        SER(SmCurve::ReadFromDB(lReadType, rDB, lReadDim, crContext, rpNewCurve, lDBVersionNumber)) ;

      } // end branch for Database Versions above 32
    else // database versions 32 and before - limited to BSplineCurves and SmCompositeCurve
      {

        // read curve type
        SM_TYPE lType;
        SER(rDB.ReadType(lType)) ;

        switch (lType)
          {
            case SmBSplineCurve_TYPE:
                SER(SmBSplineCurve::ReadFromDB(SmBSplineCurve_TYPE, rDB, lDim, crContext, rpNewCurve, lDBVersionNumber)) ;
                break ;

                //      gw_CURVE *Cur;
                //      SER(sm_InputCurve(crContext,Cur,rDB));
                //
                //      // fast constructor for already allocated NURBs
                //      rpNewCurve = new (crContext) SmBSplineCurve(Cur,lDim,FALSE);
                //
                //
                //      if (N_CrvIs3d(Cur) && lDim == 2)
                //        {
                //          SER(rpNewCurve->ConvertTo2D());
                //        }
                //      break;

            case SmCompositeCurve_TYPE:
                {
                    SE(SM_ERR); // Untested code - will not be used because there
                                // is no corresponding write method at the moment.
                    ULONG lNumSegments, lClosed, lDimension;
                    if (eType == SM_ASCII)
                      {
                        std::istream & rFileIn = *rDB.GetInStreamPtr();
                        rFileIn >> lNumSegments >> lDimension >> lClosed;
                        GOTO_NEXT_LINE;
                      }
                    else
                      {
                        SER(rDB.ReadLong(lNumSegments));
                        SER(rDB.ReadLong(lDimension));
                        SER(rDB.ReadLong(lClosed));
                      }

                    SmBoolean bClosed =  (lClosed != 0)
                                        ? TRUE
                                        : FALSE;
                    SmTArray<SmCurve*> sCurves;
                    SmTArray<SmBoolean> sSenses;
                    SmTArray<double> sGaps;

                    // for every segment
                    for (ULONG ii=0; ii<lNumSegments; ii++)
                      {
                        double dGapStart, dGapEnd;
                        SmBoolean bSameSense;
                        ULONG lSameSense=0;
                        if (eType == SM_ASCII)
                          {
                            std::istream & rFileIn = *rDB.GetInStreamPtr();
                            rFileIn >> lSameSense >> dGapStart >> dGapEnd;
                            GOTO_NEXT_LINE;
                          }
                        else
                          {
                            SER(rDB.ReadLong(lSameSense));
                            SER(rDB.ReadDouble(dGapStart));
                            SER(rDB.ReadDouble(dGapEnd));
                          }

                        bSameSense =   (lSameSense != 0)
                                     ? TRUE
                                     : FALSE;
                        if (ii==0)
                          {
                            sGaps.Add(dGapStart);
                          }
                        sGaps.Add(dGapEnd);
                        sSenses.Add(bSameSense);
                        SmCurve *pCurve;

                        // recursively read the contained curve
                        SER(SmBrepData::ReadCurveFromDB(crContext,
                                                        lDim,
                                                        lDBVersionNumber,
                                                        rAllAttributes,
                                                        pCurve,
                                                        rDB,
                                                        bPersistAttribs));
                        NER(pCurve);
                        sCurves.Add(pCurve);
                      } // end iter every segment

                    SmCompositeCurve *pCC = new(crContext) SmCompositeCurve(lDimension,
                                                                            sCurves,
                                                                            bClosed,&sSenses,&sGaps);
                    rpNewCurve = pCC;
                }
                break;

            default:
                SER(SM_ERR);
                break;
          } // end switch on lType
      } // end branch for database version 32 and before

    // handle attributes
    if (lDBVersionNumber > 30 && bPersistAttribs == TRUE)
      {
        SmTArray<ULONG> sAttributes ;
        SmAttributeData::ReadIndexedAttributesFromDB(crContext,rAllAttributes,sAttributes,rDB);
        // SmObjDelete sClean(pAttributes);
        if (sAttributes.GetSize() > 0)
          {
            // for every attribute
            for (ULONG i=0; i<sAttributes.GetSize(); i++)
              {
                SmAttribute *pAttr = rAllAttributes[sAttributes[i]];
                NER(pAttr);
                rpNewCurve->AddAttribute(pAttr);
              } // end iter every attribute
          } // end pAttributes existence check
      } // end lDBVersionNumber > 30 check

    return SM_SUCCESS;

} // end SmBrepData::ReadCurveFromDB

/*******************************************************************//**
PURPOSE: Output a curve to a stream - replace attribute pointers with
  attribute indices and place attribute pointers on rAllAttributes list
  so they can be written to file later on.

NOTES: Writes every curve out as a BSpline of type SmBSplineCurve_TYPE.
  Adds curve's attributes to the rALLAttributes accumulation list.

    Outputs:
      Curve type
      Number of Control Points
      BSpline Degree
      rational flag
      dimension
      array of control points
      array of knots (listing multiple knots multiple times).
      total number of attributes
      an index value for each attribute.

***********************************************************************/
SmStatus SmBrepData::WriteCurveToDB
  (const SmContext                      & crContext,        // in : context for new object construction
   SmCurve                              * pCrv,             // in : target curve to output
   ULONG                                  lDBVersionNumber, // in : database version to get proper sequence of writes
   SmTArray<SmAttribute*>               & rAllAttributes,   // i/o: attribute accumulation list
   SmMapTypeToType<SmAttribute *,ULONG> & rAttrMap,         // i/o: attribute to object map for each accumulated attribute
   SmDatabaseIO                         & rDB,              // in : contains target stream
   SmBoolean                              bWriteAsBSplines, // in : TRUE = Approx nonBSplineGeometry and write out as BSplineShapes
                                                            //      FALSE= Write native formats for nonBSplines
                                                            //      default:[FALSE]
   SmApproxTol3d                          sApproxTol3d,     // in : only used when bWriteAsBSplines==TRUE
                                                            //      when approximating geometry as BSplines for output.
                                                            //      0.0 = use m_sApproxTol3d, default:[0.0]
   SmBoolean                              bPersistAttribs)  // in : TRUE = write attributes, false = don't, default:[TRUE]
{
  // SmFileType eType = rDB.GetFileType();
  // Note this method appends to file

  // branch on database version
  if(lDBVersionNumber > 32)
    {
      // branch when only writing BSplines
      if(bWriteAsBSplines)
        {
          // preface Curve Write with Type and Dim data
          SM_TYPE lType = SmBSplineCurve_TYPE ;
          ULONG   lDim  = pCrv->GetDim() ;
          rDB.WriteType(lType, &lDim) ;

          SmBSplineCurve *pBSC = SM_CAST_PTR(SmBSplineCurve, pCrv) ;
          SmObjDelete sClean(NULL) ;
          if(pBSC == NULL)
            {
              double                dAchievedTol ;
              SmExtent1d            sIvl = pCrv->GetNaturalInterval() ;
              SmTArray<double>      sBreakParams ;
              sBreakParams.Add(sIvl.GetMin()) ;
              sBreakParams.Add(sIvl.GetMax()) ;

              // approximate curve with a BSpline
              pCrv->ApproximateCurve(crContext, sBreakParams, sApproxTol3d, dAchievedTol, pBSC,
                                     FALSE,    // in : bOptCreateAnalytics
                                     TRUE,     // in : bOptMatchParameterization
                                     TRUE) ;   // in : bJustCopyBSplines
              sClean.SetObj(pBSC) ;

              // inform the public
              SE_MSG(dAchievedTol < sApproxTol3d ? SM_SUCCESS : SM_ERR, _T("WriteCurveToDB Warning: BSpline Approximation not within tolerance.")) ;
            }

          // pass write along to SmBSplineCurve class
          pBSC->SmBSplineCurve::WriteToDB(rDB, lDBVersionNumber) ;

        } // end write as BSplineCurve branch
      else // write native formats
        {
          // preface Curve Write with Type and Dim data
          SM_TYPE lType = pCrv->GetType() ;
          ULONG   lDim  = pCrv->GetDim() ;
          rDB.WriteType(lType, &lDim) ;

          // pass write along to appropriate derived Curve type
          pCrv->WriteToDB(rDB, lDBVersionNumber) ;
        }

    } // end branch for database versions greater than 32
  else // database versions 32 and before - limited to BSplineCurves and SmCompositeCurve
    {
      //ULONG lType = pCrv->GetType();
      ULONG lType = SmBSplineCurve_TYPE;

      // output Curve Type
      SER(rDB.WriteType(lType)) ;
      //      if (eType == SM_ASCII)
      //        {
      //          std::ostream & rFileOut = *rDB.GetOutStreamPtr();
      //          rFileOut << "Curve Type " << lType << "\n";
      //        }
      //      else
      //        {
      //          SER(rDB.WriteLong(lType));
      //        }

      // output curve description, control points, and knots
      SmBSplineCurve * pBSP = NULL;
      switch (lType)
        {
          case SmBSplineCurve_TYPE:
              pBSP = SM_CAST_PTR(SmBSplineCurve,pCrv);
              NER(pBSP) ;
              SER(pBSP->SmBSplineCurve::WriteToDB(rDB, lDBVersionNumber)) ;
              // SER(sm_OutputCurve(pBSP->GetOrCreateGwNurbPointer(),rDB));
              break;
          default:
              break;
        }
    } // end branch for database versions 32 and below

  // gather and output curve attributes
  if (lDBVersionNumber > 30 && bPersistAttribs == TRUE)
    {
      SmTArray<ULONG> sAttributes ;
      SER(sm_ExtractAttributes(crContext,       // in : current context for object creation
                               pCrv,            // in : target object potentially containing attributes
                               sAttributes,     // out: rAttributes index array for this object's attributes, or NULL for none,
                                                //      expected NULL on input, owned by user, must be deleted by caller
                               rAllAttributes,  // i/o: accumulation of all attributes on all entities
                               rAttrMap));      // i/o: accumulation of [attribute,ObjectAttributeIndexValue+1] pairs
      SmAttributeData::WriteIndexedAttributesToDB(sAttributes,rDB);
      // if (pAttributes) { delete pAttributes ; pAttributes = NULL ; }
    }

  return SM_SUCCESS;

} // end SmBrepData::WriteCurveToDB

/*******************************************************************//**
PURPOSE: read one surface from stream - replace attribute indices
       with attributes passed in the rAllAttributes array.

NOTES:
  1. Attributes
    a. After version 30 attributes are read/written with objects.
    b. Attribute indices are written to file with each object followed
       by the list of attributes themselves.
    c. rAllAttributes contains a list of place holders for all
       attributes to be read from file.  Any attributes indices found in
       the stream will be used to select and add an attribute to the
       surface object being constructed.
    d. Later in the read process - place holder attributes are replaced
       with attributes read from file.
    e. As far as this function is concerned the rAllAttributes array
       needs to be larger than the largest attribute index value stored
       with this surface.  That number is found when reading the file.
***********************************************************************/
SmStatus SmBrepData::ReadSurfaceFromDB
  (const SmContext        & crContext,        // in : context for new object construction
   ULONG                    lDBVersionNumber, // in : controls backward compatible reads
   SmTArray<SmAttribute*> & rAllAttributes,   // in : array of attributes to access with attribute indices read from stream.
                                              //      assumes:[array is larger than larger attribute index stored for this surface]
   SmSurface             *& rpNewSurface,     // out: read surface
   SmDatabaseIO           & rDB,              // in : contains target stream
   SmBoolean                bPersistAttribs)  // in : TRUE = read attributes, false = don't, default:[TRUE]
{
    // init output
    rpNewSurface = NULL ;

    // local file type
    SmFileType eType = rDB.GetFileType();

    // read surface type
    SM_TYPE lType;
    SER(rDB.ReadType(lType)) ;

    // branch on database version number
    if(lDBVersionNumber > 32)
      {
        // pass the call to appropriate derived class ReadFromDB call
        SER(SmSurface::ReadFromDB(lType, rDB, crContext, rpNewSurface, lDBVersionNumber)) ;

      } // end branch for Database Versions above 32
    else // database versions 32 and before - limited to BSplineCurves and SmCompositeCurve
      {

        // switch on surface type
        switch (lType)
          {
            case SmBSplineSurface_TYPE:
                { SER(SmBSplineSurface::ReadFromDB(SmBSplineSurface_TYPE, rDB, crContext, rpNewSurface, lDBVersionNumber)) ;
                  //      gw_SURFACE *Sur;
                  //      SER(sm_InputSurface(crContext,Sur,rDB));
                  //
                  //      // fast constructor for already allocated NURB
                  //      rpNewSurface = new (crContext) SmBSplineSurface(Sur,FALSE);

                } break;

            case SmOffsetSurface_TYPE:
                {
                  // read offsetRadius
                  double dOffsetRadius;
                  if (eType == SM_ASCII)
                    {
                      std::istream & rFileIn = *rDB.GetInStreamPtr();
                      rFileIn >> dOffsetRadius;
                      GOTO_NEXT_LINE;
                    }
                  else
                    {
                      SER(rDB.ReadDouble(dOffsetRadius));

                    }

                  // read the base surface
                  SmSurface *pBase = NULL ;
                  SER(SmBSplineSurface::ReadFromDB(SmBSplineSurface_TYPE, rDB, crContext, pBase, lDBVersionNumber)) ;

                  //      gw_SURFACE *Sur;
                  //      SER(sm_InputSurface(crContext,Sur,rDB));
                  //
                  //      // fast constructor for already allocated NURB and OffsetSurface constructor
                  //      SmSurface *pBase = new (crContext) SmBSplineSurface(Sur,FALSE);
                  rpNewSurface     = new(crContext) SmOffsetSurface(dOffsetRadius,*pBase,TRUE);

                } break;

            default:
                { SER(SM_ERR) ; }
                break;
          } // end switch on surface type
      } // end branch for database versions 32 and below

    // for current versions - read attributes
    if (lDBVersionNumber > 30 && bPersistAttribs == TRUE)
      {
        SmTArray<ULONG> sAttributes ;
        SmAttributeData::ReadIndexedAttributesFromDB(crContext,rAllAttributes,sAttributes,rDB);
        // SmObjDelete sClean(pAttributes);

        // when there are attributes
        if (sAttributes.GetSize())
          {
            // for every attribute
            for (ULONG i=0; i<sAttributes.GetSize(); i++)
              {
                SmAttribute *pAttr = rAllAttributes[sAttributes[i]];
                  if (pAttr)
                  {
                      rpNewSurface->AddAttribute(pAttr); // JLMCC changed from NER(pAttr) per FS
                  }
              } // end iter every attribute
          } // end pAttributes existence check
      } // end lDBVersionNumber > 30 check

    return SM_SUCCESS;

} // end SmBrepData::ReadSurfaceFromDB

/*******************************************************************//**
PURPOSE:  Write Surface to stream.

NOTES: Writes every surface out as a BSpline of type SmBSplineSurface_TYPE
  or as an offset surface of type SmOffsetSurface_TYPE.
  Adds surfaces's attributes to the rALLAttributes accumulation list.

    Outputs:
      Curve type
      Number of Control Points
      BSpline Degree
      rational flag
      dimension
      array of control points
      array of knots (listing multiple knots multiple times).
      total number of attributes
      an index value for each attribute.

***********************************************************************/
SmStatus SmBrepData::WriteSurfaceToDB
 (const SmContext                      & crContext,        // in : context for new object construction
  SmSurface                            * pSrf,             // in : target surface to output
  ULONG                                  lDBVersionNumber, // in : database version to get proper sequence of writes
  SmTArray<SmAttribute*>               & rAllAttributes,   // i/o: attribute accumulation list
  SmMapTypeToType<SmAttribute *,ULONG> & rAttrMap,         // i/o: attribute to object map for each accumulated attribute
  SmDatabaseIO                         & rDB,              // in : contains target stream
  SmBoolean                              bWriteAsBSplines, // in : TRUE = Approx nonBSplineGeometry and write out as BSplineShapes
                                                           //      FALSE= Write native formats for nonBSplines
                                                           //      default:[FALSE]
  SmApproxTol3d                          sApproxTol3d,     // in : only used when bWriteAsBSplines==TRUE
                                                           //      when approximating geometry as BSplines for output.
                                                           //      0.0 = use m_sApproxTol3d, default:[0.0]
  SmBoolean                              bPersistAttribs)  // in : TRUE = write attributes, false = don't, default:[TRUE]
{
    SmFileType eType = rDB.GetFileType();

    // branch on database version
    if(lDBVersionNumber > 32)
      {
        // branch when only writing BSplines
        if(bWriteAsBSplines)
          {
            // preface Surface Write with Type and Dim data
            SM_TYPE lType = SmBSplineSurface_TYPE ;
            rDB.WriteType(lType) ;

            SmBSplineSurface *pBSS = SM_CAST_PTR(SmBSplineSurface, pSrf) ;
            SmObjDelete sClean(NULL) ;

            // when Surface is not a BSpline - approximate it as one
            if(pBSS == NULL)
              {
                double dAchievedTol = 0.0 ;
                pSrf->ApproximateSurface(crContext, sApproxTol3d, dAchievedTol, 3, pBSS) ;
                sClean.SetObj(pBSS) ;

                // inform the public
                SE_MSG(dAchievedTol < sApproxTol3d ? SM_SUCCESS : SM_ERR, _T("WriteSurfaceToDB Warning: BSpline Approximation not within tolerance.")) ;
              }

            // pass write along to SmBSplineSurface class
		    if (pBSS) 
			{ 
            	pBSS->SmBSplineSurface::WriteToDB(rDB, lDBVersionNumber) ;
			}
          } // end write as BSplineSurface branch
        else // write native formats
          {
            // preface Surface Write with Type and Dim data
            SM_TYPE lType = pSrf->GetType() ;
            rDB.WriteType(lType) ;

            // pass write along to appropriate derived Surface type
            pSrf->WriteToDB(rDB, lDBVersionNumber) ;
          }

      } // end branch for database versions greater than 32
    else // database versions 32 and before - limited to BSplineCurves and SmCompositeCurve
      {
        //ULONG lType = pSrf->GetType();
        SmOffsetSurface *pOff = SM_CAST_PTR(SmOffsetSurface,pSrf);
        ULONG lType           =   pOff
                                ? SmOffsetSurface_TYPE
                                : SmBSplineSurface_TYPE;

        // output surface type
        SER(rDB.WriteType(lType)) ;

        // output surface type
        SmBSplineSurface * pBSP = NULL;
        switch (lType)
          {
            case SmBSplineSurface_TYPE: { pBSP = SM_CAST_PTR(SmBSplineSurface,pSrf);
                                          NER(pBSP);
                                          SER(pBSP->SmBSplineSurface::WriteToDB(rDB, lDBVersionNumber)) ;
                                          // SER(sm_OutputSurface(pBSP->GetOrCreateGwNurbPointer(),rDB));

                                        } break;

            case SmOffsetSurface_TYPE:  {
                                          double dOffsetDistance = pOff->GetOffsetDistance();
                                          if (eType == SM_ASCII)
                                            {
                                              std::ostream & rFileOut = *rDB.GetOutStreamPtr();
                                              rFileOut << dOffsetDistance << " Offset Distance \n";
                                            }
                                          else
                                            {
                                              SER(rDB.WriteDouble(dOffsetDistance));
                                  //            rFileOut.write((char*)&dOffsetDistance,sizeof(double));
                                            }
                                          pBSP = SM_CAST_PTR(SmBSplineSurface,pOff->GetBaseSurface());
                                          if (pOff->GetExtendedBaseSurface())
                                            {
                                              pBSP = SM_CAST_PTR(SmBSplineSurface,pOff->GetExtendedBaseSurface());
                                            }
                                          NER(pBSP);
                                          SER(pBSP->SmBSplineSurface::WriteToDB(rDB, lDBVersionNumber)) ;
                                          // SER(sm_OutputSurface(pBSP->GetOrCreateGwNurbPointer(),rDB));

                                        } break;
            default:
            break;
          } // end switch on surface type
      } // end branch for database versions 32 and below

    // add surface attributes to accumulation list and output attribute indices to file
    if (lDBVersionNumber > 30 && bPersistAttribs == TRUE)
      {
        SmTArray<ULONG> sAttributes ;
        SER(sm_ExtractAttributes(crContext,       // in : current context for object creation
                                 pSrf,            // in : target object potentially containing attributes
                                 sAttributes,     // out: rAttributes index array for this object's attributes, or NULL for none,
                                                  //      expected NULL on input, owned by user, must be deleted by caller
                                 rAllAttributes,  // i/o: accumulation of all attributes on all entities
                                 rAttrMap));      // i/o: accumulation of [attribute,ObjectAttributeIndexValue+1] pairs
        SmAttributeData::WriteIndexedAttributesToDB(sAttributes,rDB);
        // if(pAttributes != NULL) { delete pAttributes ; pAttributes = NULL ;  }
      }

    return SM_SUCCESS;

} // end SmBrepData::WriteSurfaceToDB

//      /*******************************************************************//**
//      PURPOSE:  read a gw_Curve from a target stream.
//
//      NOTES:
//      ***********************************************************************/
//      SmStatus  sm_InputCurve
//        (const SmContext & crContext,   // in : context for new object construction
//         gw_CURVE *& cur,               // out: read curve
//         SmDatabaseIOFile & rDB)            // in : contains target stream
//      {
//          SmFileType eType = rDB.GetFileType();
//
//        gw_FLAG    dim, rat, type, error = NL_NO;
//
//        gw_INDEX   i, n, m;
//
//        gw_DEGREE  p;
//
//        gw_CPOINT  *Pw;
//
//        gw_REAL    *U, wx, wy, wz, w;
//
//
//        /* Get parameters from the top */
//
//
//        if (eType == SM_ASCII) {
//            std::istream & rFileIn = *rDB.GetInStreamPtr();
//            rFileIn >> n >> p >> rat >> dim;
//            GOTO_NEXT_LINE;
//        }
//        else {// Binary
//            SER(rDB.ReadLong(n));
//            SER(rDB.ReadShort(p));
//            SER(rDB.ReadShort(rat));
//            SER(rDB.ReadShort(dim));
//        }
//
//        m = n+p+1;
//
//
//        /* See if memory is needed */
//
//        cur = sm_AllocateNurbCurve(n,p,m);
//
//        if (cur == NULL) NL_OUT;
//
//        N_CrvGetCPtsAndKnots(cur,&Pw,&U);
//
//
//        /* Get different types of input */
//
//
//        if( rat EQ NL_NO )
//        {
//          if( dim EQ 2 )  type = 1;  else  type = 2;
//        }
//        else
//        {
//          if( dim EQ 2 )  type = 3;  else  type = 4;
//        }
//
//
//        /* Read in data */
//
//
//        switch( type )
//        {
//
//          case 1 :  /* 2-D non-rational */
//
//            for( i=0; i<=n; i++ )
//            {
//              if (eType == SM_ASCII) {
//                  std::istream & rFileIn = *rDB.GetInStreamPtr();
//                rFileIn >> wx >> wy;
//                GOTO_NEXT_LINE;
//              }
//              else {// Binary
//                  SER(rDB.ReadDouble(wx));
//                  SER(rDB.ReadDouble(wy));
//              }
//              N_CPtFromWxWyWz(wx,wy,NL_NOZ,NL_NOW,&Pw[i]);
//            }
//            break;
//
//
//          case 2 :  /* 3-D non-rational */
//
//            for( i=0; i<=n; i++ )
//            {
//              if (eType == SM_ASCII) {
//                  std::istream & rFileIn = *rDB.GetInStreamPtr();
//                rFileIn >> wx >> wy >> wz;
//                GOTO_NEXT_LINE;
//              }
//              else {// Binary
//                  SER(rDB.ReadDouble(wx));
//                  SER(rDB.ReadDouble(wy));
//                  SER(rDB.ReadDouble(wz));
//              }
//              N_CPtFromWxWyWz(wx,wy,wz,NL_NOW,&Pw[i]);
//            }
//            break;
//
//
//          case 3 :  /* 2-D rational */
//
//            for( i=0; i<=n; i++ )
//            {
//              if (eType == SM_ASCII) {
//                  std::istream & rFileIn = *rDB.GetInStreamPtr();
//                rFileIn >> wx >> wy >> w;
//                GOTO_NEXT_LINE;
//              }
//              else {// Binary
//                  SER(rDB.ReadDouble(wx));
//                  SER(rDB.ReadDouble(wy));
//                  SER(rDB.ReadDouble(w));
//              }
//              N_CPtFromWxWyWz(wx,wy,NL_NOZ,w,&Pw[i]);
//            }
//            break;
//
//
//          case 4 :  /* 3-D rational */
//
//            for( i=0; i<=n; i++ )
//            {
//              if (eType == SM_ASCII) {
//                  std::istream & rFileIn = *rDB.GetInStreamPtr();
//                rFileIn >> wx >> wy >> wz >> w;
//                GOTO_NEXT_LINE;
//              }
//              else {// Binary
//                  SER(rDB.ReadDouble(wx));
//                  SER(rDB.ReadDouble(wy));
//                  SER(rDB.ReadDouble(wz));
//                  SER(rDB.ReadDouble(w));
//              }
//              N_CPtFromWxWyWz(wx,wy,wz,w,&Pw[i]);
//            }
//            break;
//
//
//          default :  /* Wrong type */
//
//            gw_ERROR(CAL_ERR);
//        }
//
//        for( i=0; i<=m; i++ )
//        {
//          if (eType == SM_ASCII) {
//              std::istream & rFileIn = *rDB.GetInStreamPtr();
//            rFileIn >> U[i];
//            GOTO_NEXT_LINE;
//          }
//          else {// Binary
//              SER(rDB.ReadDouble(U[i]));
//          }
//        }
//
//
//        /* Exit */
//
//
//        EXIT:
//
//        if (error == NL_YES) return SM_ERR;
//        return SM_SUCCESS;
//
//      } // end sm_InputCurve

//      /*******************************************************************//**
//      PURPOSE: output a Curve to a target stream
//
//      NOTES:
//      ***********************************************************************/
//      SmStatus sm_OutputCurve
//        (gw_CURVE *cur,       // in : target curve
//         SmDatabaseIOFile & rDB)  // in : contains target stream
//      {
//
//        SmFileType eType = rDB.GetFileType();
//
//        gw_FLAG    dim, rat, type, error = NL_NO;
//
//        gw_INDEX   i, n, m;
//
//        gw_DEGREE  p;
//
//        gw_CPOINT  *Pw;
//
//        gw_REAL    *U, wx, wy, wz, w;
//
//
//        /* Get local notation for better understanding */
//
//
//        N_CrvGetCPtsDegreeAndKnots(cur,&n,&Pw,&p,&m,&U);
//
//
//        /* Get different types of output */
//
//
//        if( N_IsCrvRat(cur) )  rat = 1;  else  rat = 0;
//        if( N_CrvIs3d(cur) )  dim = 3;  else  dim = 2;
//
//        if( rat EQ 0 )
//        {
//          if( dim EQ 2 )  type = 1;  else  type = 2;
//        }
//        else
//        {
//          if( dim EQ 2 )  type = 3;  else  type = 4;
//        }
//
//
//        /* Create the output file */
//
//        // output header, n, p, rat, and dim
//        if (eType == SM_ASCII) {
//            std::ostream & rFileOut = *rDB.GetOutStreamPtr();
//            rFileOut << n << "\n";
//            rFileOut << p << "\n";
//            rFileOut << rat << "\n";
//            rFileOut << dim << "\n";
//        }
//        else {// Binary
//            SER(rDB.WriteLong(n));
//            SER(rDB.WriteShort(p));
//            SER(rDB.WriteShort(rat));
//            SER(rDB.WriteShort(dim));
//        }
//
//        // switch on curve type
//        //    2d non-rational
//        //    3d non-rational
//        //    2d rational
//        //    3d rational
//        switch( type )
//          {
//            case 1 :  /* 2-D non-rational */
//
//              for( i=0; i<=n; i++ )
//              {
//                N_CPtToWxWyWz(Pw[i],&wx,&wy,&wz,&w);
//                if (eType == SM_ASCII) {
//                    std::ostream & rFileOut = *rDB.GetOutStreamPtr();
//                    rFileOut << wx << " " << wy << "\n";
//                }
//                else {// Binary
//                    SER(rDB.WriteDouble(wx));
//                    SER(rDB.WriteDouble(wy));
//                }
//              }
//              break;
//
//
//            case 2 :  /* 3-D non-rational */
//
//              for( i=0; i<=n; i++ )
//              {
//                N_CPtToWxWyWz(Pw[i],&wx,&wy,&wz,&w);
//                if (eType == SM_ASCII) {
//                    std::ostream & rFileOut = *rDB.GetOutStreamPtr();
//                    rFileOut << wx << " " << wy << " " << wz << "\n";
//                }
//                else {// Binary
//                    SER(rDB.WriteDouble(wx));
//                    SER(rDB.WriteDouble(wy));
//                    SER(rDB.WriteDouble(wz));
//                }
//              }
//              break;
//
//
//            case 3 :  /* 2-D rational */
//
//              for( i=0; i<=n; i++ )
//              {
//                N_CPtToWxWyWz(Pw[i],&wx,&wy,&wz,&w);
//                if (eType == SM_ASCII) {
//                    std::ostream & rFileOut = *rDB.GetOutStreamPtr();
//                    rFileOut << wx << " " << wy << " " << w << "\n";
//                }
//                else {// Binary
//                    SER(rDB.WriteDouble(wx));
//                    SER(rDB.WriteDouble(wy));
//                    SER(rDB.WriteDouble(w));
//                }
//              }
//              break;
//
//
//            case 4 :  /* 3-D rational */
//
//              for( i=0; i<=n; i++ )
//              {
//                N_CPtToWxWyWz(Pw[i],&wx,&wy,&wz,&w);
//                if (eType == SM_ASCII) {
//                    std::ostream & rFileOut = *rDB.GetOutStreamPtr();
//                    rFileOut << wx << " " << wy << " " << wz << " " << w << "\n";
//                }
//                else {// Binary
//                    SER(rDB.WriteDouble(wx));
//                    SER(rDB.WriteDouble(wy));
//                    SER(rDB.WriteDouble(wz));
//                    SER(rDB.WriteDouble(w));
//
//                }
//              }
//              break;
//
//
//            default :  /* Wrong type */
//
//            gw_ERROR(CAL_ERR);
//          } // end switch on curve type to output control points
//
//        // for every knot
//        for( i=0; i<=m; i++ )
//          {
//            if (eType == SM_ASCII)
//              {
//                std::ostream & rFileOut = *rDB.GetOutStreamPtr();
//                rFileOut << U[i] << "\n";
//              }
//            else // Binary
//              {
//                SER(rDB.WriteDouble(U[i]));
//
//              }
//          } // end iter every knot
//
//
//        /* Exit */
//
//
//        EXIT:
//
//        if (error == NL_YES) return SM_ERR;
//        return SM_SUCCESS;
//
//      } // end sm_OutputCurve

//      /*******************************************************************//**
//      PURPOSE: read a surface from a target stream
//
//      NOTES:
//      ***********************************************************************/
//      SmStatus sm_InputSurface
//        (const SmContext & crContext,  // in : context for new object construction
//         gw_SURFACE *& sur,            // out: read surface
//         SmDatabaseIOFile & rDB)           // in : contains target stream
//      {
//          SmFileType eType = rDB.GetFileType();
//
//        gw_FLAG    rat, error = NL_NO;
//
//        gw_INDEX   i, j, n, m, r, s;
//
//        gw_DEGREE  p, q;
//
//        gw_CPOINT  **Pw;
//
//        gw_REAL    *U, *V, wx, wy, wz, w;
//
//
//        /* Get parameters from the top */
//
//        if (eType == SM_ASCII) {
//              std::istream & rFileIn = *rDB.GetInStreamPtr();
//          rFileIn >> n >> m;
//          GOTO_NEXT_LINE;
//          rFileIn >> p >> q;
//          GOTO_NEXT_LINE;
//          rFileIn >> rat;
//          GOTO_NEXT_LINE;
//        }
//        else {// Binary
//            SER(rDB.ReadLong(n));
//            SER(rDB.ReadLong(m));
//            SER(rDB.ReadShort(p));
//            SER(rDB.ReadShort(q));
//            SER(rDB.ReadShort(rat));
//
//      //    rFileIn.read((char*)&n,sizeof(gw_INDEX));
//      //    rFileIn.read((char*)&m,sizeof(gw_INDEX));
//      //    rFileIn.read((char*)&p,sizeof(gw_DEGREE));
//      //    rFileIn.read((char*)&q,sizeof(gw_DEGREE));
//      //    rFileIn.read((char*)&rat,sizeof(gw_FLAG));
//        }
//
//        r = n+p+1;
//        s = m+q+1;
//
//
//        /* See if memory is needed */
//
//        sur = sm_AllocateNurbSurface(n,m,p,q,r,s);
//
//        if( sur == NULL)  NL_OUT;
//
//        N_SrfGetCPtsAndKnots(sur,&Pw,&U,&V);
//
//
//        /* Read in data */
//
//
//        switch( rat )
//        {
//
//          case NL_NO :  /* Non-rational */
//
//            for( i=0; i<=n; i++ )
//            {
//              for( j=0; j<=m; j++ )
//              {
//                if (eType == SM_ASCII) {
//                    std::istream & rFileIn = *rDB.GetInStreamPtr();
//                  rFileIn >> wx >> wy >> wz;
//                  GOTO_NEXT_LINE;
//                }
//                else {// Binary
//                  SER(rDB.ReadDouble(wx));
//                  SER(rDB.ReadDouble(wy));
//                  SER(rDB.ReadDouble(wz));
//      //            rFileIn.read((char*)&wx,sizeof(gw_REAL));
//      //            rFileIn.read((char*)&wy,sizeof(gw_REAL));
//      //            rFileIn.read((char*)&wz,sizeof(gw_REAL));
//                }
//                N_CPtFromWxWyWz(wx,wy,wz,NL_NOW,&Pw[i][j]);
//              }
//            }
//            break;
//
//
//          case NL_YES :  /* Rational */
//
//            for( i=0; i<=n; i++ )
//            {
//              for( j=0; j<=m; j++ )
//              {
//                if (eType == SM_ASCII) {
//                    std::istream & rFileIn = *rDB.GetInStreamPtr();
//                  rFileIn >> wx >> wy >> wz >> w;
//                  GOTO_NEXT_LINE;
//                }
//                else {// Binary
//                  SER(rDB.ReadDouble(wx));
//                  SER(rDB.ReadDouble(wy));
//                  SER(rDB.ReadDouble(wz));
//                  SER(rDB.ReadDouble(w));
//      //            rFileIn.read((char*)&wx,sizeof(gw_REAL));
//      //            rFileIn.read((char*)&wy,sizeof(gw_REAL));
//      //            rFileIn.read((char*)&wz,sizeof(gw_REAL));
//      //            rFileIn.read((char*)&w,sizeof(gw_REAL));
//                }
//                N_CPtFromWxWyWz(wx,wy,wz,w,&Pw[i][j]);
//              }
//            }
//            break;
//
//
//          default :  /* Wrong type */
//
//            gw_ERROR(CAL_ERR);
//        }
//
//        for( i=0; i<=r; i++ )
//        {
//          if (eType == SM_ASCII) {
//              std::istream & rFileIn = *rDB.GetInStreamPtr();
//            rFileIn >> U[i];
//            GOTO_NEXT_LINE;
//          }
//          else {// Binary
//              SER(rDB.ReadDouble(U[i]));
//      //      rFileIn.read((char*)&U[i],sizeof(gw_REAL));
//          }
//        }
//
//        for( j=0; j<=s; j++ )
//        {
//          if (eType == SM_ASCII) {
//              std::istream & rFileIn = *rDB.GetInStreamPtr();
//            rFileIn >> V[j];
//            GOTO_NEXT_LINE;
//          }
//          else {// Binary
//              SER(rDB.ReadDouble(V[j]));
//      //      rFileIn.read((char*)&V[j],sizeof(gw_REAL));
//          }
//        }
//
//
//        /* Exit */
//
//
//        EXIT:
//
//        if (error == NL_YES) return SM_ERR;
//        return SM_SUCCESS;
//
//      } // end sm_InputSurface

//      /*******************************************************************//**
//      PURPOSE:  output a surface to a stream.
//
//      NOTES:
//      ***********************************************************************/
//      SmStatus  sm_OutputSurface
//        (gw_SURFACE *sur,        // in : target surface
//         SmDatabaseIOFile & rDB)     // in : contains target stream
//      {
//
//        gw_FLAG    rat, error = NL_NO;
//
//        gw_INDEX   i, j, n, m, r, s;
//
//        gw_DEGREE  p, q;
//
//        gw_CPOINT  **Pw;
//
//        gw_REAL    *U, *V, wx, wy, wz, w;
//
//        SmFileType eType = rDB.GetFileType();
//
//        /* Get local notation */
//
//
//        N_SrfGetCPtsDegreesAndKnots(sur,&n,&m,&Pw,&p,&q,&r,&s,&U,&V);
//
//
//        /* Get rational flag */
//
//
//        if( N_IsSrfRat(sur) )  rat = NL_YES;  else  rat = NL_NO;
//
//
//        /* Create the output file */
//
//        if (eType == SM_ASCII)
//          {
//            std::ostream & rFileOut = *rDB.GetOutStreamPtr();
//            rFileOut << n << " " << m << "\n";
//            rFileOut << p << " " << q << "\n";
//            rFileOut << rat << "\n";
//          }
//        else // Binary
//          {
//            SER(rDB.WriteLong(n));
//            SER(rDB.WriteLong(m));
//            SER(rDB.WriteShort(p));
//            SER(rDB.WriteShort(q));
//            SER(rDB.WriteShort(rat));
//      //      rFileOut.write((char*)&n,sizeof(gw_INDEX));
//      //      rFileOut.write((char*)&m,sizeof(gw_INDEX));
//      //      rFileOut.write((char*)&p,sizeof(gw_DEGREE));
//      //      rFileOut.write((char*)&q,sizeof(gw_DEGREE));
//      //      rFileOut.write((char*)&rat,sizeof(gw_FLAG));
//          }
//
//        // switch on rationa/nonRational to output control points
//        switch( rat )
//          {
//
//            case NL_NO :  /* Non-rational */
//
//              // for every control point
//              for( i=0; i<=n; i++ )
//                {
//                  for( j=0; j<=m; j++ )
//                    {
//                      N_CPtToWxWyWz(Pw[i][j],&wx,&wy,&wz,&w);
//                      if (eType == SM_ASCII)
//                        {
//                          std::ostream & rFileOut = *rDB.GetOutStreamPtr();
//                          rFileOut << wx << " " << wy << " " << wz << "\n";
//                        }
//                      else // Binary
//                        {
//                          SER(rDB.WriteDouble(wx));
//                          SER(rDB.WriteDouble(wy));
//                          SER(rDB.WriteDouble(wz));
//              //            rFileOut.write((char*)&wx,sizeof(gw_REAL));
//              //            rFileOut.write((char*)&wy,sizeof(gw_REAL));
//              //            rFileOut.write((char*)&wz,sizeof(gw_REAL));
//                        }
//                    }
//                }
//              break;
//
//
//            case NL_YES :  /* Rational */
//
//              for( i=0; i<=n; i++ )
//              {
//                for( j=0; j<=m; j++ )
//                {
//                  N_CPtToWxWyWz(Pw[i][j],&wx,&wy,&wz,&w);
//                  if (eType == SM_ASCII) {
//                      std::ostream & rFileOut = *rDB.GetOutStreamPtr();
//                      rFileOut << wx << " " << wy << " " << wz << " " << w << "\n";
//                  }
//                  else {// Binary
//                    SER(rDB.WriteDouble(wx));
//                    SER(rDB.WriteDouble(wy));
//                    SER(rDB.WriteDouble(wz));
//                    SER(rDB.WriteDouble(w));
//        //            rFileOut.write((char*)&wx,sizeof(gw_REAL));
//        //            rFileOut.write((char*)&wy,sizeof(gw_REAL));
//        //            rFileOut.write((char*)&wz,sizeof(gw_REAL));
//        //            rFileOut.write((char*)&w,sizeof(gw_REAL));
//                  }
//                }
//              }
//              break;
//
//
//            default :  /* Wrong type */
//
//              gw_ERROR(CAL_ERR);
//          } // end switch on rational/nonRational to output control points
//
//        // output u knots
//        for( i=0; i<=r; i++ )
//          {
//            if (eType == SM_ASCII)
//              {
//                std::ostream & rFileOut = *rDB.GetOutStreamPtr();
//                rFileOut << U[i] << "\n";
//              }
//            else // Binary
//              {
//                SER(rDB.WriteDouble(U[i]));
//        //      rFileOut.write((char*)&U[i],sizeof(gw_REAL));
//              }
//          }
//
//        // output v knots
//        for( j=0; j<=s; j++ )
//         {
//           if (eType == SM_ASCII)
//             {
//               std::ostream & rFileOut = *rDB.GetOutStreamPtr();
//               rFileOut << V[j] << "\n";
//             }
//           else // Binary
//             {
//               SER(rDB.WriteDouble(V[j]));
//       //      rFileOut.write((char*)&V[j],sizeof(gw_REAL));
//             }
//         }
//
//        /* Exit */
//
//
//        EXIT:
//
//        if (error == NL_YES) return SM_ERR;
//        return SM_SUCCESS;
//
//      } // end sm_OutputSurface
//
//      #undef MAXSIZE
//
