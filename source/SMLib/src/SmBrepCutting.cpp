// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************//**
* FILE NAME --- SmBrepCutting.cpp
* PURPOSE: Implementation of brep cutting methods.
**********************************************************************/

#include "StdAfx.h"

#include <SmBrepCutting.h>

#include <SmFace.h>
#include <SmEdge.h>
#include <SmVertex.h>

#ifdef SM_DEBUG_CODE
#include <SmAssertArray.h>
#endif


/*******************************************************************//**
PURPOSE: Constructor for a Brep cutting object.  It simply sets the
    Brep which is to be cut.

NOTES:  
You could also try CookieCutter option in Merge as in:   
       SmMerge sMerge(*pContext, pBrep1, pBrep2, approxTol, dAngleTol);
       sMerge.SetCookieCutter(TRUE) ; // <=== HERE
       SmStatus status = sMerge.NonManifoldBoolean(SM_BO_DIFFERENCE, pResult);
 
***********************************************************************/
SmBrepCutting::SmBrepCutting(SmBrep *pBrepToCut)
: m_pBrepToCut(pBrepToCut)
{
    SM_ASSERT(pBrepToCut != NULL);
} // end SmBrepCutting::SmBrepCutting constructor

/*******************************************************************//**
PURPOSE: This method actually cuts away portions of a brep based on
    the given cutter.  Note that if the Brep contains trimmed surfaces
    a single trimmed surface may sometimes be cut into more than one
    trimmed surface.  It is possible to do the cut without removing the
    extra pieces by setting the remove flag to FALSE.

NOTES: 
***********************************************************************/
SmStatus SmBrepCutting::DoCut
 (SmCutter * pCutter,                  // in : object cutting this->m_pBrepToCut
  double     d3DTolerance,             // in : boolean tolerance
  SmBoolean  bRemoveCutPortions,       // in : TRUE = remove cut portions
                                       //      FALSE= don't
  SmTArray<SmFace *> *pOptSubsetFaces) // in : optional subset of faces (default NULL)
{
  SM_ASSERT(pCutter != NULL);

  // set m_pBrepToCut EditingEnabled flag to TRUE for upcoming boolean
  SmTemporaryChangeValue< SmBoolean > sStack1(m_pBrepToCut->m_bEditingEnabled,TRUE);
  const SmContext * pContext = m_pBrepToCut->GetContext();
  SmTemporaryChangeValue< SmBoolean > sDoingBool( SM_CONST_CAST(SmContext*, pContext)->GetDoingBooleanRef(), TRUE );

  // store this cutter object
  m_pCutter = pCutter;

  // Do intersection of cutter with brep
#ifdef SM_DEBUG_CODE
  SmBoolean bDebugMe = FALSE ;
  if(bDebugMe)
    { m_pBrepToCut->Dump() ; }
#endif // SM_DEBUG_CODE

  SER(m_pCutter->MergeIntersection(m_pBrepToCut, d3DTolerance, pOptSubsetFaces));

#ifdef SM_DEBUG_CODE
    if(bDebugMe)
      {
        SM_ASSERT_VALID(m_pBrepToCut) ; 

        smgfx_Erase() ; 
        smgfx_SetColor(0.0,0.0,0.0) ; m_pBrepToCut->Draw(TRUE) ; sm_GraphicsLoop() ; 
        smgfx_SetColor(0.0,1.0,0.0) ; m_pCutter->Draw(m_pBrepToCut) ; sm_GraphicsLoop() ;
        smgfx_SetColor(0.0,0.0,0.0) ;
        sm_GraphicsLoop() ;
      }
#endif // SM_DEBUG_CODE

  // no work - when not removing cut portions
  if (!bRemoveCutPortions)
    { return SM_SUCCESS ; }

  // Now classify faces, edges, and vertices - return list of them
  // which are to be deleted from the brep.
  SmTArray<SmFace*> sFacesToDel;
  SmTArray<SmEdge*> sEdgesToDel;
  SmTArray<SmVertex*> sVerticesToDel;
  SER(m_pCutter->Classification(m_pBrepToCut, 
                                d3DTolerance,   // can increment an unlocked mark value
                                sFacesToDel, 
                                sEdgesToDel, 
                                sVerticesToDel));

  ULONG ii;
#ifdef SM_DEBUG_CODE
  if(bDebugMe)
    {

      smgfx_Erase() ; 
      smgfx_SetLook(1,2, 0,0,0) ; m_pBrepToCut->Draw(TRUE) ; sm_GraphicsLoop() ; 
      smgfx_SetLook(1,2, 0,1,0) ; m_pCutter->Draw(m_pBrepToCut) ; sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;

      smgfx_SetLook(4,6, 1,0,0) ; 
      for(ii=0;ii<sEdgesToDel.GetSize();ii++)
      { 
		  if(sEdgesToDel[ii]) 
			  sEdgesToDel[ii]->Draw() ; 
		  sm_GraphicsLoop() ; 
	  }
      for(ii=0;ii<sVerticesToDel.GetSize();ii++)
      {
		  if(sVerticesToDel[ii]) 
			  sVerticesToDel[ii]->Draw() ; 
		  sm_GraphicsLoop() ; 
	  }
      for(ii=0;ii<sFacesToDel.GetSize();ii++) { 
		  if(sFacesToDel[ii]) 
			  sFacesToDel[ii]->Draw(SM_DM_CROSSHATCH,18,18) ; 
		  sm_GraphicsLoop() ; 
	  }
      sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,0,0) ;
    }
#endif // SM_DEBUG_CODE
 
  // Now delete things which have been cutaway from the brep
  SmTArray<SmTopology *> sThisDeletedEdgesVertices, sDeletedEdgesVertices ;
  ULONG lFoundIndex ;

  // delete every face on sFacesToDel list from targetBrep
  for (ii=0; ii<sFacesToDel.GetSize(); ii++) 
    {                                    
      SmFace *pF = sFacesToDel[ii];

      // For BiLinear cutters, tell DeleteFace to
      // remove all hanging Wire Edges and Shell Vertices.
      // (Planar cutter takes care of that.)
      SmBoolean bDelWireFlag = FALSE;
      if ( pCutter->GetType() == SmSurfaceBiLinearCutter_TYPE )
        { bDelWireFlag = TRUE; }

      m_pBrepToCut->DeleteFace(pF, bDelWireFlag, TRUE, &sThisDeletedEdgesVertices);
      sDeletedEdgesVertices.Append(sThisDeletedEdgesVertices) ;
    }

  if (pCutter->GetType() != SmSurfaceBiLinearCutter_TYPE)
    {
      // delete every edge on sEdgesToDel list from targetBrep
      for (ii=0; ii<sEdgesToDel.GetSize(); ii++) 
        {
          SmEdge *pE = sEdgesToDel[ii];

          // skip already deleted edges
          if(sDeletedEdgesVertices.FindElement(pE, lFoundIndex))
            { continue ; }

          m_pBrepToCut->DeleteEdge(pE);
        }

      // delete every vertex on sVerticesToDel list from targetBrep
      for (ULONG k=0; k<sVerticesToDel.GetSize(); k++) 
        {
          SmVertex *pV = sVerticesToDel[k];

          // skip already deleted vertices
          if(sDeletedEdgesVertices.FindElement(pV, lFoundIndex))
            { continue ; }

          m_pBrepToCut->DeleteVertex(pV);
        }

#ifdef SM_DEBUG_CODE
	  if (bDebugMe) {
		  smgfx_Erase();
		  smgfx_SetLook(2, 2, 0, 1, 0);
		  m_pBrepToCut->Draw();
		  sm_GraphicsLoop();
	  }
#endif
	  
	  m_pBrepToCut->StitchAndOrient();

    } // end pCutter type is not SmSurfaceBiLinearCutter_TYPE check

  return SM_SUCCESS;

} // end SmBrepCutting::DoCut


