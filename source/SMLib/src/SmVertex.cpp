// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************//**
* FILE NAME --- SmVertex.cpp
* PURPOSE: Source file for SmVertex class methods.
**********************************************************************/

#include "StdAfx.h"

#include <SmVertex.h>

#ifndef __SMBREP_H__
#include <SmBrep.h>
#endif

#ifndef __SMLOOP_H__
#include <SmLoop.h>
#endif

#ifndef __SMEDGE_H__
#include <SmEdge.h>
#endif

// Remove Composite
// #ifndef __SMCFACE_H__
// #include <SmCFace.h>
// #endif

#include <SmGraphicsOutput.h>
#include <SmGap.h>
#include <SmAssertArray.h>
#include <SmSurfaceIntersector.h>

//    // For debugging only - a set of pointers that can be assigned and inspected through the watch window
//    #ifdef SM_DEBUG_CODE
//
//    SmVertex * dbgVertex1 = NULL ;
//    SmVertex * dbgVertex2 = NULL ;
//
//    #endif // SM_DEBUG_CODE

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
void SmPolyVertexList::Dump() const
{
  TCHAR sBuff[SM_TBLOCK_SIZE], sBuffForFile[SM_TBLOCK_SIZE];

  smos_sprintf(sBuff,        _T("\nSmPolyVertexList[0x%p]: "), this);
  smos_sprintf(sBuffForFile, _T("%s"),_T("\nSmPolyVertexList: "));
  smos_WriteBuffer(sBuff, sBuffForFile);

  smos_WriteBuffer(_T("  m_pPolyVertex")) ;

  smos_sprintf(sBuff,        _T("[0x%p]"), m_pPolyVertex);
  smos_sprintf(sBuffForFile, _T("[%s]"), m_pPolyVertex ? _T("NotNULL"): _T("NULL"));
  smos_WriteBuffer(sBuff, sBuffForFile);


} // end SmPolyVertexList::Dump

/*******************************************************************//**
PURPOSE: Are the Two given edges tangent at this vertex?

NOTES: Both edges have to have at least end on this vertex.
***********************************************************************/
SmBoolean SmVertex::AreEdgesTangent
  (const SmEdge * pE1,           // in :
   const SmEdge * pE2,           // in :
   double         dAngleTolDeg)  // in :
  const
{
  SmVector3d sTan1, sTan2;
  if (pE1->GetStartVertex() == this)
    {
      SE(pE1->GetEndTangent(TRUE,sTan1));
    }
  else
    {
      SE(pE1->GetEndTangent(FALSE,sTan1));
      sTan1 = - sTan1;
    }

  // Note that sTan1 is pointing to the inside of the curve
  // and sTan2 will be pointing to the outside of the curve.
  // That should make them parallel.

  if (pE2->GetStartVertex() == this)
    {
      SE(pE2->GetEndTangent(TRUE,sTan2));
      sTan2 = - sTan2;
    }
  else
    {
      SE(pE2->GetEndTangent(FALSE,sTan2));
    }

  double dAngle;
  SE(sTan1.AngleBetween(sTan2,dAngle));
  if (dAngle*180.0/SM_PI < dAngleTolDeg)
    {
      return TRUE;
    }
  return FALSE;

} // end SmVertex::AreEdgesTangent

/*******************************************************************//**
PURPOSE: Return TRUE when stored m_sZoneTol3d value == SmTol::GetZoneTol3d(this,0.0,TRUE)

NOTES: This is for making old style tolerances consistent.
  In old style tolerancing, the value (now renamed to m_sZoneTol3d)
  is the ZoneTol3d value for this Vertex.  In new style tolerancing,
  it's an optional over ride value for the SmContext::ZoneTol3d value.

  So, this function always returns TRUE when compiled with SM_USE_NEWTOL
  else returns TRUE when m_sZoneTol3d == SmTol::GetZoneTol3d(this,0.0,TRUE)
***********************************************************************/
SmBoolean SmVertex::IsZoneTol3dConsistent
  (SmZoneTol3d * pOptZoneTol3d) // out: computed SmTol::GetZoneTol3d value for this vertex, NULL to ignore
                                //      default:[NULL]
 const
{
#ifdef SM_USE_NEWTOL
  SM_NEWTOL_LINE return(TRUE) ;
#endif // SM_USE_NEWTOL

  // SmBoolean bUseNewTol     = TRUE ;  // force consistent tolerance computation

  SmZoneTol3d sThisTol3d = GetTolerance() ;
  SmZoneTol3d sZoneTol3d = SmTol::GetZoneTol3d(this) ;

  SmBoolean   bRtn       = SM_ARE_SAME(sZoneTol3d, sThisTol3d) ;
  // SmBoolean   bRtn       = TRUE ; // SM_ARE_SAME(sZoneTol3d, sThisTol3d) ;

  // set output
  if(pOptZoneTol3d) { *pOptZoneTol3d = sZoneTol3d ; }

  // all done
  return(bRtn) ;

} // end SmVertex::IsZoneTol3dConsistent()

/*******************************************************************//**
PURPOSE: Return True when Vertex is connected to given Target Topology object

NOTES: 1. the input Target cpConnectTgt may be NULL, or of type,
             SmVertex,
             SmEdge,
             SmLoop,
             SmFace,
             SmShell,
             SmRegion
        2. returns TRUE for NULL and for any other unsupported Topology TYPE
***********************************************************************/
SmBoolean SmVertex::IsConnectedTo
  ( const SmTopology *cpConnectTgt )    // in : target Topology
 const
{
  // no work - no Topo
  if(cpConnectTgt == NULL) 
    { return TRUE ; }

  // switch on cpConnectTgt type
  switch(cpConnectTgt->GetType())
    {
      case SmVertex_TYPE   : { return( this == (SmVertex *)cpConnectTgt ) ; 
                             } break ;
      case SmVertexuse_TYPE: { return( this == ((SmVertexuse *)cpConnectTgt)->GetVertex() ) ;
                             } break ;
      case SmEdge_TYPE     : { SmTArray<SmVertex *> sVertices ;
                               ((SmEdge *)cpConnectTgt)->GetVertices(sVertices) ; 
                               return( sVertices.IsIn((SmVertex *)this) ) ;
                             } break ;
      case SmEdgeuse_TYPE  : { return(   ((SmEdgeuse *)cpConnectTgt)->GetVertexuse() 
                                      && this == ((SmEdgeuse *)cpConnectTgt)->GetVertexuse()->GetVertex() ) ;
                             } break ;
      case SmFace_TYPE     : { SmTArray<SmFace *> sFaces ;
                               GetFaces(sFaces) ; 
                               return( sFaces.IsIn((SmFace *)cpConnectTgt) ) ;
                             } break ;
      case SmFaceuse_TYPE  : { SmTArray<SmFaceuse *> sFaceuses ; // 
                               GetFaceuses(sFaceuses) ; 
                               return( sFaceuses.IsIn((SmFaceuse *)cpConnectTgt) ) ;
                             } break ;
      case SmLoop_TYPE     : { SmTArray<SmVertex *> sVertices ;
                               ((SmLoop *)cpConnectTgt)->GetVertices(sVertices) ; 
                               return( sVertices.IsIn((SmVertex *)this) ) ;
                             } break ;
      case SmLoopuse_TYPE  : { SmTArray<SmVertex *> sVertices ;
                               ((SmLoopuse *)cpConnectTgt)->GetVertices(sVertices) ; 
                               return( sVertices.IsIn((SmVertex *)this) ) ;
                             } break ;
      case SmShell_TYPE    : { SmTArray<SmShell *> sShells ;
                               GetShells(sShells) ; 
                               return( sShells.IsIn((SmShell *)cpConnectTgt) ) ;
                             } break ;
      case SmRegion_TYPE   : { SmTArray<SmRegion *> sRegions ;
                               GetRegions(sRegions) ; 
                               return( sRegions.IsIn((SmRegion *)cpConnectTgt) ) ;
                             } break ;
      default: break ;     

    } // end switch on type

  // arrive here when cpConnectTgt is an unsupported type - return TRUE
  return(TRUE) ;

} // end SmVertex::IsConnectedTo

/*******************************************************************//**
PURPOSE: Find any Vertex->Vertexuse which connects to target Face or CFace.

NOTES: Return null if none exists.
    There will generally be two Vertexuses for the Face, one for each Faceuse;
    this returns the Vertexuse on the Upward Faceuse if there is one
    else returns whatever Vertexuse it finds.

    When pFace is a CFace returns 1st Vertexuse found connected to any
     of the CFace->MemberFaces
***********************************************************************/
SmVertexuse * SmVertex::GetVertexuseOfFace
  ( const SmFace *pFace )    // in : target face
 const
{
  // init return
  SmVertexuse *pRet = NULL;

  // no work - no face
  if(pFace == NULL)
    { return pRet; }

  // locals
  ULONG ii;
  SM_PTR_ARRAY(sVertexuses, SmVertexuse, 32) ; 
// Remove Composite
//   SmBoolean bIsComposite = pFace->GetType() == SmCFace_TYPE ; 

  // for every Vertexuse - check for a face == pFace match
  GetVertexuses( sVertexuses );
  for( ii=0; ii < sVertexuses.GetSize(); ii++ )
    {
      SmVertexuse * pVU       = sVertexuses[ii] ;
      SmFaceuse   * pFU       = pVU->GetFaceuse() ;

      // skip vertices not connected to a Face
      if ( pFU == NULL )
        { continue; }

      SmFace * pThisFace = pFU->GetFace() ;
      SM_ASSERT_MSG(pThisFace != NULL, _T("SmVertex::GetVertexuseOfFace - found a Faceuse without a Face point - needs debug")) ; 

// Remove Composite
//      // when VU connects to TgtFace (or one of its member faces)
//      if(   (!bIsComposite && pThisFace == pFace)
//         || ( bIsComposite && ((SmCFace*)pFace)->ContainsFace(pThisFace)))
        {
          // remember any found Vertexuses
          pRet = pVU;

          // Use the first Vertexuse found on an upward Faceuse.
          if ( pThisFace->GetUpwardFaceuse() == pFU )
            { break; }
        }
    } // end iter every Vertexuse

  // all done
  return pRet;

} // end SmVertex::GetVertexuseOfFace

/*******************************************************************//**
PURPOSE: Find all Vertex->Vertexuses which connect to target Face.

NOTES: when pOptConnectTgt != NULL, only Vertexuses connected to 
       pOptConnectTgt are returned.  pOptConnectTgt was created
       to find Vertexuses connected to a Face on a TgtLoop
***********************************************************************/
void SmVertex::GetVertexusesOfFace
 (const SmFace           * cpFace,         // in : target face
  SmTArray<SmVertexuse*> & rVertexuses,    // out: all Vertexuses connecting Vertex to Face
  const SmTopology       * pOptConnectTgt) // in : NotNULL        = rtn Vertexuses connected to cpFace and pOptConnectTgt 
                                           //      default:[NULL] = rtn all Vertexuses connected to cpFace
 const
{
  // init return
  rVertexuses.ReSet() ;

  // no work - no face
  if(cpFace == NULL)
    { return  ; }

  // locals
  ULONG ii;
  SM_PTR_ARRAY(sVertexuses, SmVertexuse, 32) ;

  // get all vertexuses
  GetVertexuses( sVertexuses );

  // for every Vertexuse - check for a face == pFace match
  for( ii=0; ii < sVertexuses.GetSize(); ii++ )
    {
      SmVertexuse * pVU = sVertexuses[ii];
      SmFaceuse   * pFU = pVU->GetFaceuse();
      if(pFU == NULL)
        { continue; }

      if( pFU->GetFace() == cpFace )
        {
          // remember any found Vertexuses that pass the ConnectTgt test
          if(   pOptConnectTgt == NULL
             || pVU->IsConnectedTo(pOptConnectTgt))
            { rVertexuses.Add(pVU) ; }
        }
    } // end iter every Vertexuse

} // end SmVertex::GetVertexusesOfFace

/*******************************************************************//**
PURPOSE: Find any Loop on Target Face which connects to this Vertex.

NOTES: Return null if none exists.

    When pFace is a CFace returns 1st Loop found connected to any
     of the CFace->MemberFaces
***********************************************************************/
SmLoop * SmVertex::GetLoopOfFace
  ( const SmFace *pFace )    // in : target face
 const
{
  // init return
  SmLoop *pRet = NULL;

  // no work - no face
  if(pFace == NULL)
    { return pRet; }

  // locals
  ULONG ii;
  SM_PTR_ARRAY(sVertexuses, SmVertexuse, 32) ; 
// Remove Composite
//   SmBoolean bIsComposite = pFace->GetType() == SmCFace_TYPE ; 

  // for every Vertexuse - check for a face == pFace match
  GetVertexuses( sVertexuses );
  for( ii=0; ii < sVertexuses.GetSize(); ii++ )
    {
      SmVertexuse * pVU       = sVertexuses[ii] ;
      SmFaceuse   * pFU       = pVU->GetFaceuse() ;

      // skip vertices not connected to a Face
      if ( pFU == NULL )
        { continue; }

#ifdef SM_DEBUG_CODE
      SmFace * pThisFace = pFU->GetFace() ;
      SM_ASSERT_MSG(pThisFace != NULL, _T("SmVertex::GetLoopOfFace - found a Faceuse without a Face owner - needs debug")) ; 
#endif // SM_DEBUG_CODE

// Remove Composite
//      // when VU connects to TgtFace (or one of its member faces)
//      if(   (!bIsComposite && pThisFace == pFace)
//         || ( bIsComposite && ((SmCFace*)pFace)->ContainsFace(pThisFace)))
        {
          // remember any found Vertexuses
          pRet = pVU->GetLoopuse()->GetLoop() ;

          // Use the first Loop found on Face
          if(pRet != NULL)
            { break; }
        }
    } // end iter every Vertexuse

  // all done
  return pRet;

} // end SmVertex::GetLoopOfFace

/*******************************************************************//**
PURPOSE: Find any Vertex->Vertexuse which connects to target edge.

NOTES: Return null if none exists.
***********************************************************************/
SmVertexuse * SmVertex::GetVertexuseOfEdge
  ( const SmEdge *pEdge )    // in : target edge
 const
{
  // no work - no edge
  if(pEdge == NULL)
    { return NULL ; }

  // locals
  SmVertexuse *pData[32];
  SmTArray<SmVertexuse*> sVertexuses(32,pData);
  ULONG i;

  // for every Vertexuse - check for a edge == pEdge match
  GetVertexuses( sVertexuses );
  for( i=0; i < sVertexuses.GetSize(); i++ )
    {
      SmVertexuse *pVU = sVertexuses[i];

      SmEdgeuse *pEU = pVU->GetEdgeuse();
      if ( pEU == NULL )
        { continue; }

      if( pEU->GetEdge() == pEdge )
        {
          // remember any found Vertexuses
          return( pVU ) ;
        }
    } // end iter every Vertexuse

  // arrive here when no connecting VU was found - all done
  return NULL;

} // end SmVertex::GetVertexuseOfEdge

/*******************************************************************//**
PURPOSE: Find all Vertex->Vertexuse which connect to target Edge.

NOTES: 
***********************************************************************/
void SmVertex::GetVertexusesOfEdge
 (const SmEdge           * cpEdge,         // in : target face
  SmTArray<SmVertexuse*> & rVertexuses,    // out: all Vertexuses connecting Vertex to Edge
  const SmTopology       * pOptConnectTgt) // in : NotNULL        = rtn all Vertexuses connected to cpFace
                                           //      default:[NULL] = rtn on Vertexuses connected to cpFace and ConnectTgt
 const
{
  // init return
  rVertexuses.ReSet() ;

  // no work - no face
  if(cpEdge == NULL)
    { return  ; }

  // locals
  ULONG i;
  SM_PTR_ARRAY(sVertexuses, SmVertexuse, 32) ;

  // get all vertexuses
  GetVertexuses( sVertexuses );

  // for every Vertexuse - check for a Edge == pEdge match
  for( i=0; i < sVertexuses.GetSize(); i++ )
    {
      SmVertexuse * pVU = sVertexuses[i];
      SmEdgeuse   * pEU = pVU->GetEdgeuse();
      if(pEU == NULL)
        { continue; }

      if( pEU->GetEdge() == cpEdge )
        {
          // remember any found Vertexuses that pass the ConnectTgt test
          if(   pOptConnectTgt == NULL
             || pVU->IsConnectedTo(pOptConnectTgt))
            { rVertexuses.Add(pVU) ; }
        }
    } // end iter every Vertexuse

} // end SmVertex::GetVertexusesOfEdge

/*******************************************************************//**
PURPOSE: Get the list of edges that two vertices have in common.

NOTES:
***********************************************************************/
void SmVertex::GetCommonEdges
  (SmVertex          * pOtherVertex,   // in : Other Vertex of interest
   SmTArray<SmEdge*> & rCommonEdges, // out: list of edges between this and pOtherVertex
   SmFace            * pOptFace)     // in : NotNULL        = Only return Edges connected to OptFace
                                     //      default:[NULL] = Return all Edges 
  const
{
  // init output
  rCommonEdges.ReSet();

  // locals
  ULONG ii ;
  SmEdge *pEData1[32];
  SmTArray<SmEdge*> sThisEdges(32,pEData1);
  SmVertex *pV1, *pV2 ;

  // get edges on this vertex
  GetEdges(sThisEdges, pOptFace) ;

  // For every vertex->Edge
  for(ii=0;ii<sThisEdges.GetSize();ii++)
    {
      SmEdge *pEdge = sThisEdges[ii] ;
      pEdge->GetVertices(pV1, pV2) ;

      // when an edge connects to both vertices (works for closed edges too)
      if(   (pV1 == this && pV2 == pOtherVertex)
         || (pV2 == this && pV1 == pOtherVertex))
        {
          rCommonEdges.Add(pEdge) ;
        }
    } // end iter all edges on this vertex

} // end SmVertex::GetCommonEdges

/*******************************************************************//**
PURPOSE: Get the edges adjacent to this vertex.

NOTES:
***********************************************************************/
void SmVertex::GetEdges
  (SmTArray<SmEdge*> & rEdges,   // out: edges connected to vertex
   SmFace            * pOptFace) // in : NotNULL        = Only return Edges connected to OptFace
                                 //      default:[NULL] = Return all Edges
 const
{
    // init output
    rEdges.ReSet();

    // vertexuses
    SmVertexuse *pArrayData[20];
    SmTArray<SmVertexuse*> sVertexuses(20,pArrayData);
    GetVertexuses(sVertexuses);

    // for every vertexuse
    for (ULONG i=0; i<sVertexuses.GetSize(); i++)
      {
        SmVertexuse *pVU = sVertexuses[i];

        // when vertexuse is an edgeVertexuse type
        if (pVU->IsEdgeVertexuse())
          {
            SmFace * pFace = pVU->GetEdgeuse()->GetFace() ; 
            SmEdge * pEdge = pVU->GetEdgeuse()->GetEdge() ;

            // add the edge to the list
            if(pOptFace == NULL || pOptFace == pFace)
              {
                rEdges.AddUnique(pEdge);
              } // end option connected to face check
          } // end Edgeuse Type check
      } // end iter every Vertexuse

} // end SmVertex::GetEdges

/*******************************************************************//**
PURPOSE: Get the faces adjacent to this vertex.

NOTES:
***********************************************************************/
void SmVertex::GetFaces
  (SmTArray<SmFace*> & rFaces)
 const
{
  // init output
  rFaces.ReSet();

  // locals
  SmVertexuse *pArrayData[20];
  SmTArray<SmVertexuse*> sVertexuses(20,pArrayData);

  // get vertex->vertexuses
  GetVertexuses(sVertexuses);

  // for every vertexuse
  for (ULONG i=0; i<sVertexuses.GetSize(); i++)
    {
      SmVertexuse *pVU = sVertexuses[i];

      // when vertexuse is not a ShellVertex
      if (!pVU->IsShellVertexuse())
        {
          SmFaceuse *pFU = pVU->GetFaceuse();
          if (pFU)
            {
              rFaces.AddUnique(pFU->GetFace());
            }
        }  // end not a shell vertex check
    }  // end iter every vertexuse

} // end SmVertex::GetFaces

/*******************************************************************//**
PURPOSE: Get the faceuses adjacent to this vertex.

NOTES:
***********************************************************************/
void SmVertex::GetFaceuses
  (SmTArray<SmFaceuse*> & rFaceuses)
 const
{
  // init output
  rFaceuses.ReSet();

  // locals
  SmVertexuse *pArrayData[20];
  SmTArray<SmVertexuse*> sVertexuses(20,pArrayData);

  // get vertex->vertexuses
  GetVertexuses(sVertexuses);

  // for every vertexuse
  for (ULONG i=0; i<sVertexuses.GetSize(); i++)
    {
      SmVertexuse *pVU = sVertexuses[i];

      // when vertexuse is not a ShellVertex
      if (!pVU->IsShellVertexuse())
        {
          SmFaceuse *pFU = pVU->GetFaceuse();
          rFaceuses.Add(pFU);

        }  // end not a shell vertex check
    }  // end iter every vertexuse

} // end SmVertex::GetFaceuses

/*******************************************************************//**
PURPOSE: Get the Shells which surround a vertex.

NOTES:
***********************************************************************/
void SmVertex::GetShells
  (SmTArray<SmShell *> & rShells)
 const
{
    rShells.ReSet();
    SmVertexuse *pArrayData[20];
    SmTArray<SmVertexuse*> sVertexuses(20,pArrayData);
    GetVertexuses(sVertexuses);
    for (ULONG i=0; i<sVertexuses.GetSize(); i++)
      {
        SmVertexuse * pVU    = (SmVertexuse*)sVertexuses[i];
        SmShell     * pShell = pVU->GetShell();
        if (pShell)
            rShells.AddUnique(pShell);
      }

} // end SmVertex::GetShells

/*******************************************************************//**
PURPOSE: Get the regions which surround a vertex.

NOTES:
***********************************************************************/
void SmVertex::GetRegions
  (SmTArray<SmRegion*> & rRegions)
 const
{
    rRegions.ReSet();
    SmVertexuse *pArrayData[20];
    SmTArray<SmVertexuse*> sVertexuses(20,pArrayData);
    GetVertexuses(sVertexuses);
    for (ULONG i=0; i<sVertexuses.GetSize(); i++)
      {
        SmVertexuse *pVU = (SmVertexuse*)sVertexuses[i];
        SmShell *pShell = pVU->GetShell();
        if (pShell)
            rRegions.AddUnique(pShell->GetRegion());
      }

} // end SmVertex::GetRegions

/*******************************************************************//**
PURPOSE: This method gets the shell of a vertex corresponding to
    a given region.

NOTES: returns SM_ERR and sets rpShell to NULL when no shell can be found
***********************************************************************/
SmStatus SmVertex::GetShellInRegion
  (const SmRegion * cpRegion,        // in : target region
   SmShell       *& rpShell)         // out: region->Shell connected to this Vertex
 const
{
  SmVertexuse *pArrayData[20];
  SmTArray<SmVertexuse*> sVertexuses(20,pArrayData);

  // init output
  rpShell = NULL ; 

  // for every vertexuse
  GetVertexuses(sVertexuses);
  for (ULONG i=0; i<sVertexuses.GetSize(); i++)
    {
      SmVertexuse *pVU    = (SmVertexuse*)sVertexuses[i];
      SmShell     *pShell = pVU->GetShell();

      // find and return the shell that connect to the target region
      if ( pShell && pShell->GetRegion() == cpRegion)
        {
          rpShell = pShell;
          return SM_SUCCESS;
        }
    } // end iter every vertexuse

  return SM_ERR;

} // end SmVertex::GetShellInRegion

/*******************************************************************//**
PURPOSE: Determine if a vertex is a topological vertex.

NOTES: A 'topological' vertex
        - can be removed without changing the shape of its Brep.
        - is a EdgeVertex,and
           o. has exactly two edges, and
             *. Edge->Curves are the same, and
                Vertexuse->EdgeParams are the same, or
             *. Edge->Curves can be joined without a kink (i.e. they meet G1)
        - and is NOT a ShellVertex or a LoopVertex
        - and is NOT a Seam Vertex on a closed Edge
        - and is NOT on a pole on a singular Surface
        - and is NOT on a Periodic Curve's Periodic Boundary (should be redundant to Not a Seam Vertex)
***********************************************************************/
SmBoolean SmVertex::IsTopologicalVertex() const
{
  // locals
  SM_PTR_ARRAY(sEdges, SmEdge, 64) ;
  GetEdges( sEdges );

  // only edges connected to exactly 2 edges are Topological
  if (sEdges.GetSize() != 2)
    { return FALSE; }

  SmEdge  *pEdge1  = sEdges[0];
  SmEdge  *pEdge2  = sEdges[1];
  SmCurve *pCurve1 = pEdge1->GetCurve();
  SmCurve *pCurve2 = pEdge2->GetCurve();

  // skip seam vertices - those vertices can't be deleted - they are required to mark the seam
  if( pEdge1 == pEdge2 )
    { return FALSE; }

  // skip verices marking poles
  if(IsOnPole())
    { return(FALSE) ; }

  // No, this now works for Wire edges as well.
  // // skip vertices not connected to just two manifold edges
  // if( sVUs.GetSize() != 4 )
  //   { continue ; }

  // remember if curves are Lines
  SmPoint3d  sLinePnt;
  SmVector3d sTanVec1, sTanVec2;
  SmBoolean  bIsLine1 = pCurve1 && pCurve1->IsLine(5,pEdge1->GetTolerance()/1000.0,sLinePnt,sTanVec1) ;
  SmBoolean  bIsLine2 = pCurve2 && pCurve2->IsLine(5,pEdge2->GetTolerance()/1000.0,sLinePnt,sTanVec2) ;

  // when either curve is not a line - Curve params must be the same
  if(!bIsLine1 || !bIsLine2)
    {
      // else vertices are topological when on two edges share a common param point

      // Check start/end ends of both edges.
      SmBoolean bAtStart1 = ( pEdge1->GetStartVertex() == this );
      SmBoolean bAtStart2 = ( pEdge2->GetStartVertex() == this );

      // First check that they are at the same parameter value.
      double dParam1 = bAtStart1 ? pEdge1->GetInterval().GetMin()
                                 : pEdge1->GetInterval().GetMax();
      double dParam2 = bAtStart2 ? pEdge2->GetInterval().GetMin()
                                 : pEdge2->GetInterval().GetMax();

      // Only vertices connected to the same params are Topological
      if(!SM_ARE_SAME(dParam1, dParam2))
        {
          return FALSE;
        }

      // If different curves, get tangent directions at this vertex.
      if ( pCurve1 != pCurve2 )
        {
          if ( bAtStart1 ) { SER( pEdge1->GetEndTangent( TRUE,  sTanVec1 )); }
          else             { SE ( pEdge1->GetEndTangent( FALSE, sTanVec1 )); }
          if ( bAtStart2 ) { SER( pEdge2->GetEndTangent( TRUE,  sTanVec2 )); }
          else             { SE ( pEdge2->GetEndTangent( FALSE, sTanVec2 )); }

        }
    } // end curves are not both SmLines branch

  // check the angle, to within 1 degree.
  // NOTE: 1 degree seems pretty loose.  But in practice, it's generally either exact or not close.
  double dAngLimitRad = SM_DEG2RAD(SM_CONTINUITY_ANGLE) ;

  double dAngleRad;
  if(pCurve1 != pCurve2) { sTanVec1.AngleBetween(sTanVec2, dAngleRad) ; }
  else                   { dAngleRad = 0.0 ; }

  // when lines are not G1 - Vertex is not topological
  if(   smos_Fabs(dAngleRad)         > dAngLimitRad
     && smos_Fabs(dAngleRad - SM_PI) > dAngLimitRad   // gwc: is this a bug? it counts cusps
     && smos_Fabs(dAngleRad + SM_PI) > dAngLimitRad)
    {
      return(FALSE) ;
    }

#ifdef SM_DEBUG_CODE
  SM_PTR_ARRAY(sFaces1, SmFace, 4) ;
  SM_PTR_ARRAY(sFaces2, SmFace, 4) ;
  pEdge1->GetFaces(sFaces1) ;
  pEdge2->GetFaces(sFaces2) ;
  SM_ASSERT_MSG(sFaces1.GetSize() == sFaces2.GetSize(), _T("SmVertex::IsTopologicalVertex - found a topological vertex whose edges are not connected to the same num of Faces - needs review")) ;
#endif // SM_DEBUG_CODE

  // Passed all our tests.
  return TRUE;

} // end SmVertex::IsTopologicalVertex

/*******************************************************************//**
PURPOSE: return TRUE = vertex marks a pole on one of the face->Surfaces
  to which it connects

NOTES:
***********************************************************************/
SmBoolean SmVertex::IsOnPole() const
{
  SmBoolean bOnPole = FALSE ;

  // locals
  ULONG ii ;
  SmVertexuse          * sVUData[16];
  SmTArray<SmVertexuse*> sVUs(16,sVUData);
  GetVertexuses( sVUs );

  // See if vertex marks a pole on one the surfaces to which it connects
  for(ii=0;ii<sVUs.GetSize() && bOnPole==FALSE;ii++)
    {
      // see if the vertexuse use marks a pole on the surface to which ic connects
      bOnPole |= sVUs[ii]->IsOnPole() ;

    } // end iter every vertexuse

  // all done
  return(bOnPole) ;

} // end SmVertex::IsOnPOle

/*******************************************************************//**
PURPOSE: return TRUE = vertex marks a seam on one of the face->Surfaces
  to which it connects

NOTES:
***********************************************************************/
SmBoolean SmVertex::IsOnSeam() const
{
  SmBoolean bOnSeam = FALSE ;

  // locals
  ULONG ii ;
  SmVertexuse          * sVUData[16];
  SmTArray<SmVertexuse*> sVUs(16,sVUData);
  GetVertexuses( sVUs );

  // See if vertex marks a seam on one the surfaces to which it connects
  for(ii=0;ii<sVUs.GetSize() && bOnSeam==FALSE;ii++)
    {
      // see if the vertexuse use marks a seam on the surface to which ic connects
      bOnSeam |= sVUs[ii]->IsOnSeam() ;

    } // end iter every vertexuse

  // all done
  return(bOnSeam) ;

} // end SmVertex::IsOnSeam

/*******************************************************************//**
PURPOSE: Return TRUE if Vertex is connected to Edge, else return FALSE.

NOTES:
***********************************************************************/
SmBoolean SmVertex::IsConnectedToEdge
  (const SmEdge *cpEdge)           // in : Target Edge
  const
{
  SmTArray<SmEdge *> sEdges ;
  GetEdges(sEdges) ;

  for(ULONG ii=0;ii<sEdges.GetSize();ii++)
    {
      if(sEdges[ii] == cpEdge)
        { return(TRUE) ; }
    }

  // arrive here when edge is not connected to face
  return(FALSE) ;

} // end SmEdge::IsConnectedToEdge

/*******************************************************************//**
PURPOSE: Return TRUE if Vertex is connected to Face, else return FALSE.

NOTES:
***********************************************************************/
SmBoolean SmVertex::IsConnectedToFace
  (const SmFace *cpFace)           // in : Target Face
  const
{
  SmTArray<SmFace *> sFaces ;
  GetFaces(sFaces) ;

  for(ULONG ii=0;ii<sFaces.GetSize();ii++)
    {
      if(sFaces[ii] == cpFace)
        { return(TRUE) ; }
    }

  // arrive here when edge is not connected to face
  return(FALSE) ;

} // end SmFace::IsConnectedToFace

/*******************************************************************//**
PURPOSE: Determine if this vertex is one of two or more vertices
  being used to represent a tolerant corner where more than 3 faces
  come together.

NOTES:
***********************************************************************/
SmBoolean SmVertex::IsComplexCornerMember
  (SmTArray<SmTopology*> *pOptComplexCornerElements, // out: optional list of vertices and edges making up the corner
                                                     //      NULL to ignore, default:[NULL].
   SmTArray<SmFace*>     *pOptComplexCornerFaces)    // out: optional list of faces meeting at this corner
                                                     //      NULL to ignore, default:[NULL].
  const
{
  // init output
  SmBoolean bRtn = FALSE ;
  if(pOptComplexCornerElements) pOptComplexCornerElements->ReSet() ;
  if(pOptComplexCornerFaces)    pOptComplexCornerFaces->ReSet() ;

  // locals
  ULONG ii, jj ;
  SmTArray<SmEdge*> sEdges ; GetEdges(sEdges) ;
  SmTArray<SmFace*> sFaces ; GetFaces(sFaces) ;
  SmTArray<SmEdge*> sOtherEdges ;
  SmTArray<SmFace*> sOtherFaces ;
  SmTArray<SmTopology*> sComplexCornerElements ;
  double dTol  = 2.0 * GetTolerance() ;
  double dTol2 = dTol * dTol ;

  // init the complex corner element list
  sComplexCornerElements.Add((SmTopology *)this) ;

  // start list of unique faces
  SmTArray<SmFace*> sComplexCornerFaces ;
  sComplexCornerFaces.Append(sFaces) ;

  // gather vertices part of a complex corner
  //  - They are within distance of this vertex
  //  - connected to this vertex via a short edge
  //  - are connected to more than two edges
  //     * check vertices connected to just two edges
  //       to see if IsTopologicalVertex() == TRUE and
  //       perhaps run SmBrep::DeleteTopologicalVertex().
  //       Maybe do this in a higher level function.
  //  - are connected to more than two faces.
  for(ii=0;ii<sEdges.GetSize();ii++)
    {
      SmEdge *pEdge = sEdges[ii] ;

      // get neighbor vertex and distance
      SmVertex *pOtherVertex = pEdge->GetOtherVertex(this) ;
      double    dDist2       = pOtherVertex->m_vPoint.DistanceBetweenSquared(m_vPoint) ;

      // look for distinct vertices within tolerance distance
      if(   pOtherVertex != this
         && dDist2 < dTol2)
        {
          // that are connected thru a short length edge
          double dEdgeLength = pEdge->GetCurve()->ApproximateLength(pEdge->GetInterval(), 4) ;
          if(dEdgeLength < dTol)
            {
              // get the other vertice's unique faces
              pOtherVertex->GetFaces(sOtherFaces) ;
              pOtherVertex->GetEdges(sOtherEdges) ;

              // when OtherVertex is connected to more than two edges
              if(sOtherEdges.GetSize() > 2)
                {
                  // when the OtherVertex is connected to more than two faces
                  if(sOtherFaces.GetSize() > 2)
                    {
                      // remember that this vertex is part of a complex corner
                      bRtn = TRUE ;

                      // add the vertex and the edge to the complex corner list
                      sComplexCornerElements.Add(pOtherVertex) ;
                      sComplexCornerElements.Add(pEdge) ;

                      // build the unique set of faces coming into this corner
                      for(jj=0;jj<sOtherFaces.GetSize();jj++)
                        {
                          SmFace *pOtherFace = sOtherFaces[jj] ;
                          sComplexCornerFaces.AddUnique(pOtherFace) ;
                        } // end iter every OtherFace building list of unique corner faces
                    } // end OtherVertex connected to more than two faces check
                }  // end OtherVertex connected to more than two edges check
              else if(sOtherEdges.GetSize() == 2)
                {
                  // SmBoolean bTopologyVertex = pOtherVertex->IsTopologicalVertex() ;

                  // it might be wise to remove Topology vertices which are close
                  // to other vertices.  It doesn't feel right to do that here
                  // so just signal a warning and in the future if this happens
                  // we can think about where we might want to clean up this structure.
                  WARN(_T("Found a TopologicalVertex near another vertex - perhaps it should be removed.")) ;
                }
              else
                {
                  WARN(_T("Found a vertex near another that is not part of a Complex Corner")) ;
                }
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
              if(bDebugMe)
                {
                  // draw Brep(blue), Vertex(red), ComplexMembers(orange), Current OtherVertex(green) and edge(cyan)
                  SmBrep *pBrep = GetBrep() ;
                  double dcnt   = (double)sComplexCornerFaces.GetSize() + 1 ;

                  smgfx_Erase() ;
                  smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
                  smgfx_SetLook(5,6, 1,0,0) ; Draw() ; sm_GraphicsLoop() ;
                  for(jj=0;jj<sComplexCornerElements.GetSize();jj++)
                    { SmTopology *pObject = sComplexCornerElements[jj] ;
                      if(pObject->IsKindOf(SmEdge_TYPE))
                           { smgfx_SetLook(2,3, 1,0,.5); ((SmEdge  *)pObject)->Draw() ; sm_GraphicsLoop() ; }
                      else { smgfx_SetLook(3,4, 1,.5,0); ((SmVertex*)pObject)->Draw() ; sm_GraphicsLoop() ; }
                    }
                  smgfx_SetLook(5,6, 0,0,1) ; if(pOtherVertex) pOtherVertex->Draw() ; sm_GraphicsLoop() ;
                  smgfx_SetLook(3,4, 0,1,1) ; if(pEdge)        pOtherVertex->Draw() ; sm_GraphicsLoop() ;
                  for(jj=0;jj<sComplexCornerFaces.GetSize();jj++)
                    { smgfx_SetLook(1,2, 0, (dcnt-jj-1)/(dcnt), 1.0 - (dcnt-jj-1)/(dcnt)) ;
                      sComplexCornerFaces[jj]->Draw(SM_DM_CROSSHATCH, 4,4) ; sm_GraphicsLoop() ; }
                }
#endif // SM_DEBUG_CODE
            }  // end vertices are connected through a short edge
        } // end vertices are distinct and close to one another check
    } // end iter every edge connected to this vertex

  // all done
  if(pOptComplexCornerElements) pOptComplexCornerElements->Append(sComplexCornerElements) ;
  if(pOptComplexCornerFaces)    pOptComplexCornerFaces->Append(sComplexCornerFaces) ;
  return(bRtn) ;

} // end SmBoolean SmVertex::IsComplexCornerMember

/*******************************************************************//**
PURPOSE:

NOTES: Normally in the .h file, this method is here to create a breakpoint
***********************************************************************/
void SmVertex::SetPolyVertex
 (SmPolyVertex *pPolyVertex)
{
  m_pPolyVertex3D = pPolyVertex ;

} // end SmVertex::SetPolyVertex

/*******************************************************************//**
PURPOSE: Determine if this vertex is a wire vertex (has only wire
    edges).

NOTES:
***********************************************************************/
SmBoolean SmVertex::IsWireVertex
  ()
 const
{
    SmEdge * sEData[64];
    SmTArray<SmEdge*> sEdges(64,sEData);
    GetEdges(sEdges);
    for (ULONG i=0; i<sEdges.GetSize(); i++)
      {
        SmEdge *pE = sEdges[i];
        if (!pE->IsWire()) return FALSE;
      }
    return TRUE;

} // end SmVertex::IsWireVertex

/*******************************************************************//**
PURPOSE: Determine if this vertex is a lamina vertex (has one or more
    lamina edges)

NOTES:
***********************************************************************/
SmBoolean SmVertex::IsLaminaVertex() const
{
  SmEdge * sEData[64];
  SmTArray<SmEdge*> sEdges(64,sEData);
  GetEdges(sEdges);

  for (ULONG i=0; i<sEdges.GetSize(); i++)
    {
      SmEdge *pE = sEdges[i];
      if (pE->IsLamina()) return TRUE;
    }

  return FALSE;

} // end SmVertex::IsLaminaVertex

/*******************************************************************//**
PURPOSE: Old Tolerance Model for setting the tolerance vector of the vertex.

NOTES: If the bUpdateOnlyIfLarger flag is TRUE then we will
     use the given tolerance only if it is larger than the existing one.
     If FALSE then we just change the value of the tolerance.
***********************************************************************/
#ifdef SM_USE_OLDTOL
void SmVertex::SetTolerance
 (SmZoneTol3d sNewZoneTol3d,        // in : Desired New ZoneTol3d Value
  SmBoolean   bUpdateOnlyIfLarger,  // in : default:[TRUE] = set tolerance only if sZoneTol3d > m_sZoneTol3d
                                    //      FALSE          = always set tolerance (allow shrinking TolValues)
  SmBoolean   bCascadeToBndries)    // NotUsed: in : not used here: default:[TRUE] = Chg VtxTol to be >= EdgeTol (previous behavior)
                                    //                     FALSE          = never change a VtxTol
{
  SM_REF1(bCascadeToBndries) ;
#ifdef SM_DEBUG_CODE
static constexpr int bDebugMe = FALSE;

  SmBrep *pBrep = GetBrep();

  // when tolerances get too large - inform the public
  if (   sNewZoneTol3d > m_sZoneTol3d
      && pBrep != NULL
      && sNewZoneTol3d > pBrep->GetTolerance()*3.0)
    {
      if ( bDebugMe)  // place for a breakpoint
        {
          TCHAR sBuff[SM_TBLOCK_SIZE], sBuffForFile[SM_TBLOCK_SIZE];
          smos_sprintf(sBuff,_T("Tolerance of Vertex = %16.16f - Getting Rather Large\n"), (double)sNewZoneTol3d);
          smos_sprintf(sBuffForFile, _T("%s"), _T("Tolerance of Vertex - Getting Rather Large\n"));
          WARN2(sBuff, sBuffForFile);
        }
    } // end large tolerance check

#endif // SM_DEBUG_CODE

  // set tolerances - when asked skip shrinking tolerances
  if(!bUpdateOnlyIfLarger || sNewZoneTol3d > m_sZoneTol3d)
    { m_sZoneTol3d = sNewZoneTol3d ; }

} // end SmVertex::SetTolerance
#endif // SM_USE_OLDTOL

/*******************************************************************//**
PURPOSE: Tries to place the Vertex location to minimize vertex/face
  distances.

NOTES: This assumes face->Surfaces are the way the model wants
  them to be and requests that the vertices be moved to sit on those
  faces
***********************************************************************/
SmStatus SmVertex::RefineGeometry
  (SmMarkType eMarkType,          // in : SM_MT_NOMARK = ignore marks
                                  //      else skip vertices which are currently marked
                                  //      mark all other vertices
   SmBoolean &bMadeChange)        // out: TRUE = Changed VertexPoint to a surf/surf/surf intersection point
                                  //             and modified associated tolerance.
                                  //      FALSE= no changes made
{
  // init output
  bMadeChange = FALSE ;

  // manage vertice's mark
  if(eMarkType != SM_MT_NOMARK)
    {
      // no work - checking marks and vertex is already marked
      if(IsMarked(eMarkType))
        { return(SM_SUCCESS) ; }
      else // mark this vertex
        { Mark(eMarkType) ; }
   } // end managing marks

  // locals
  SmBoolean   bFoundPoint ;
  double      dMaxGap3d ;
  SmPoint3d   sBestPoint ;
  SmFace    * pMaxGapFace ;

  // look for a surf/surf/surf intersection to define the vertex location
  CalcCornerFromFaces(bFoundPoint, dMaxGap3d, pMaxGapFace, sBestPoint) ;

  // when BestPoint was found
  if(bFoundPoint)
    {
      // inform the public - upcoming change
      Notify(SM_NO_PRE_EDIT, this, SM_NO_GET_BREP(this), NULL) ;

      // save Vertex location
      SetPoint(sBestPoint) ;

      // set new tolerance
      SmZoneTol3d sBrepZoneTol3d = GetBrep()->GetTolerance() ;

      SM_OLDTOL_LINE SetTolerance(smos_Max( (double)sBrepZoneTol3d, dMaxGap3d * 2.0), FALSE) ;

      // inform the public - done with change
      Notify(SM_NO_POST_EDIT, this, SM_NO_GET_BREP(this), NULL) ;
      bMadeChange = TRUE ;
    }

  // all done
  return SM_SUCCESS;

} // end SmVertex::RefineGeometry

/*******************************************************************//**
PURPOSE: returns max Face - Vertex - Face gap of all Vertex->Faces
         that are parallel to one another within SM_ANG_TOL_DEG
         of one another

NOTES: Uses cached Vertex/Edge Gaps stored on the Vertex->Vertexuses
       and updates those as necessary.

       When bForceCalc == TRUE, all Vertex/Edge gaps have been made
            current when this routine exits.
***********************************************************************/
SmBoolean SmVertex::HasParallelFaces
 (double    * pOptAngTolDeg,          // in : optional max angle deg between parallel vectors,
                                      //      NULL=SM_ANG_TOL_DEG, default:[NULL]
  double    * pOptMinDihedralAngDeg,  // out: Min Dihedral AngDeg seen between Faces
  double    * pOptMaxParallelFaceGap, // out: when Rtn == TRUE, Max gap between parallel face pairs
  SmFace   ** pOptFace1,              // out: when Rtn == TRUE, optional face1 of MaxParallelFace/Face Gap pair
  SmFace   ** pOptFace2,              // out: when Rtn == TRUE, optional face2 of MaxParallelFace/Face Gap pair
  SmPoint2d * pOptUV1,                // out: when Rtn == TRUE, optional face1 UVPt of MaxParallelFace/Face Gap pair
  SmPoint2d * pOptUV2)                // out: when Rtn == TRUE, optional face2 UVPt of MaxParallelFace/Face Gap pair
 const
 {
  // return value
  SmBoolean bRtn = FALSE ;

  // init output
  if(pOptMinDihedralAngDeg)  { *pOptMinDihedralAngDeg  = SM_BIG_DOUBLE ; }
  if(pOptMaxParallelFaceGap) { *pOptMaxParallelFaceGap = 0.0 ; }
  if(pOptFace1)              { *pOptFace1 = NULL ; }
  if(pOptFace2)              { *pOptFace2 = NULL ; }
  if(pOptUV1  )              {  pOptUV1->SetUninitialized() ; }
  if(pOptUV2  )              {  pOptUV2->SetUninitialized() ; }

  // locals
  ULONG ii, jj ;
  SmTArray<SmFace*>      sFaces ;
  SmTArray<SmVertexuse*> sVertexuses ;
  double                 dMaxParallelFaceGap = 0.0 ;
  double                 dDihedralAngRad = 0.0, dDihedralAngDeg = 0.0;
  double                 dAngTolDeg = pOptAngTolDeg ? *pOptAngTolDeg : SM_ANG_TOL_DEG ;
  SmPoint3d              sPN1[2], sPN2[2] ;
  GetFaces(sFaces) ;

  // Build Vertexuse array on one vertex use to each Vertex->Face
  for(ii=0;ii<sFaces.GetSize();ii++)
    {
      SmFace * pFace = sFaces[ii] ;
      sVertexuses.Add(GetVertexuseOfFace(pFace)) ;

    } // end iter to build sVertexuses array

  // for every FacePair 1st element - get MaxParallelFaceGap
  for(ii=0;ii<sVertexuses.GetSize()-1;ii++)
    {
      // Get 1st end of Face/Face Gap
      SmVertexuse     * pVertexuse1     = sVertexuses[ii] ;
      SmSurface       * pSurface1       = pVertexuse1->GetFaceuse()->GetFace()->GetSurface() ;
      SmVertexFaceGap * pVertexFaceGap1 = pVertexuse1->GetVertexFaceGap() ;

      // skip Vertexuses not connected to Faces
      if(pVertexFaceGap1 == NULL)
        { continue ; }

      pSurface1->EvaluatePoint ( pVertexFaceGap1->GetFaceUV(), sPN1[0]) ;
      pSurface1->EvaluateNormal( pVertexFaceGap1->GetFaceUV(), TRUE, TRUE, sPN1[1]) ;

      // for every FacePair 2nd element
      for(jj=ii+1;jj<sVertexuses.GetSize();jj++)
        {
          // Get 2nd end of Face/Face Gap
          SmVertexuse     * pVertexuse2     = sVertexuses[jj] ;
          SmSurface       * pSurface2       = pVertexuse2->GetFaceuse()->GetFace()->GetSurface() ;
          SmVertexFaceGap * pVertexFaceGap2 = pVertexuse2->GetVertexFaceGap() ;

          // skip Vertexuses not connected to Faces
          if(pVertexFaceGap2 == NULL)
            { continue ; }

          pSurface2->EvaluatePoint ( pVertexFaceGap2->GetFaceUV(), sPN2[0]) ;
          pSurface2->EvaluateNormal( pVertexFaceGap2->GetFaceUV(), TRUE, TRUE, sPN2[1]) ;

          sPN1[1].AngleBetween(sPN2[1], dDihedralAngRad) ;
          dDihedralAngDeg = SM_RAD2DEG(dDihedralAngRad) ;
          if(pOptMinDihedralAngDeg && *pOptMinDihedralAngDeg > dDihedralAngDeg)
            { *pOptMinDihedralAngDeg = dDihedralAngDeg ; }

          // for parallel faces
          if(dDihedralAngDeg < dAngTolDeg)
            {
              // set output
              bRtn = TRUE ;

              // save the MaxGap
              double dGap3d = sPN1[0].DistanceBetween(sPN2[0]) ;
              if(dMaxParallelFaceGap <= dGap3d)
                {
                  dMaxParallelFaceGap = dGap3d ;
                  if(pOptMaxParallelFaceGap) { *pOptMaxParallelFaceGap = dGap3d ; }
                  if(pOptFace1) { *pOptFace1 = pVertexuse1->GetFaceuse()->GetFace() ; }
                  if(pOptFace2) { *pOptFace2 = pVertexuse2->GetFaceuse()->GetFace() ; }
                  if(pOptUV1  ) { *pOptUV1   = pVertexFaceGap1->GetFaceUV() ; }
                  if(pOptUV2  ) { *pOptUV2   = pVertexFaceGap2->GetFaceUV() ; }
                } // end MaxGap check
            } // end ParallelFace check
        } // end iter jj all Vertex->Face pairs
    } // end iter ii all Vertex->Face pairs

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  if(bDebugMe)
    {
      ULONG di ;
      SmBrep *pBrep = GetBrep() ;
      SmTArray<SmPoint2d>  sUVs ;
      SmTArray<SmPoint3d>  sPts ;
      SmTArray<SmVector3d> sNorms ;
      SmTArray<SmEdge*>    sEdges ;
      for(di=0;di<sFaces.GetSize();di++)
        {
          SmFace          * pFace          = sFaces[di] ;
          SmSurface       * pSurface       = pFace->GetSurface() ;
          SmVertexuse     * pVertexuse     = GetVertexuseOfFace(pFace) ;
          SmVertexFaceGap * pVertexFaceGap = pVertexuse->GetVertexFaceGap() ; 
          SM_ASSERT_MSG(pVertexFaceGap!=NULL, _T("Unexpected NULL Gap ptr - debug")) ;

          SmPoint2d         sUV            = pVertexFaceGap->GetFaceUV() ;
          SmPoint3d         sPt, sNorm ;
          pSurface->EvaluatePoint(sUV, sPt) ;
          pSurface->EvaluateNormal(sUV, TRUE, TRUE, sNorm) ;
          sUVs.Add(sUV) ;
          sPts.Add(sPt) ;
          sNorms.Add(sNorm) ;
        }
      GetEdges(sEdges) ;

      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(7,9, 0,1,0) ; Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook(3,4, 1,0,0) ; for(di=0;di<sFaces.GetSize();di++)
                                    { sPts[di].Draw() ; sNorms[di].Draw(&sPts[di]) ; sm_GraphicsLoop() ; }
      smgfx_SetLook(1,2, 0,0,0) ; for(di=0;di<sFaces.GetSize();di++)
                                    { sFaces[di]->DrawUV(8,8,FALSE,NULL,TRUE) ; sm_GraphicsLoop() ; }
      smgfx_SetLook(5,6, 0,1,0) ; for(di=0;di<sEdges.GetSize();di++)
                                    { SmExtent1d sEdgesInterval (sEdges[di]->GetInterval());
                                      sEdges[di]->GetCurve()->DrawParams(&sEdgesInterval) ; sm_GraphicsLoop() ; }
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

  // all done
  return(bRtn) ;

} // end SmVertex::HasParallelFaces

/*******************************************************************//**
PURPOSE: Returns TRUE when every use of this Vertex has
           consistent Vertex/Loop/Face/Shell pointers.
NOTES:
   Rules: The Three kinds of Vertex connections are made through Vertexuse
          of types: SmShell_TYPE,
                    SmLoopuse_TYPE,
                    SmEdgeuse_TYPE

   Rules: 1. A ShellVertex connects directly to just 1 shell with no other connections.
          2. When not a ShellVertex, a vertex may have any number of
             Loopuse and/or Edgeuse connections.
             2a. A Vertex may connect to any number of faces through just one loop per face
                 as either a - LoopVertex =(sole member of a Face's VertexLoop) or an
                             - EdgeVertex =(part of a Face's EdgeLoop)
             2b. A vertex may not connect to multiple loops in one face.
             2c. A vertex may connect to any number of wire edges through Edgeuse
                 connections.  Each Wire Edgeuse may connect to a different Shell
                 (only when the vertex is also connected to faces which separate the shells).

   1.) A ShellVertex is represented with 1 Vertex, 1 Vertexuse
                                         1 Shell

          Vertex->VU<--->Shell

       No other Vertex->VU[i] connections are allowed when the vertex is a ShellVertex.

   2.) A LoopVertex is represented with 1 Vertex,   2 Vertexuses,
                                        1 Loop,     2 Loopuses,
                                        1 Face, and 2 Faceuses.
          omitting many details as:

          Vertex<-->VU[i]<->LU1<------------>FU1----+
                            ^ |              ^      |
                            | |              |      |
                        Mates +->CommonLoop  Mates  +-->CommonFace
                            | |              |      |
                            v |              v      |
          Vertex<---VU[j]<->LU2<------------>FU2----+

   3.) An EdgeVertex Part of an EdgeShell is represented with 1 Vertex,   1 Vertexuses per Edge
                                                              1 Edge,     1 Edgeuse
                                                              1 Shell
          omitting many details as:

          Vertex<->VU[i]<->EU1--+
                           ^    |
                           |    |
                       Mates    +-->CommonShell
                           |    |
          Other            v    |
          Vertex<->VU[j]<->EU2--+

   4.) An EdgeVertex Part of a Loopuse is represented with 1 Vertex,   2 Vertexuses per Edge
                                                           2 Edges,    2 Edgeuses per Edge
                                                           1 Loop,     2 Loopuses
                                                           1 Face, and 2 Faceuses.
          omitting many details as:

          Other<-->VU[j]<->EU2---->LU2-+<----------->FU2----+
          Vertex           ^       ^   |             ^      |
                           |       |   |             |      |
                       Mates   Mates   |         Mates      |
                           |       |   |             |      |
                           v       v   |             v      |
          Vertex-->VU[i]<->EU1---->LU1-+<----------->FU1----+
                           ^           |                    |
                      Orientation      |                    |
                       ?CCW:CW         +->CommonLoop        +-->CommonFace
                           |           |                    |
                           v           |                    |
          Other<-->VU[k]<->EU3---->LU1-+<----------->FU1----+
          Vertex           ^       ^   |             ^      |
                           |       |   |             |      |
                       Mates   Mates   |         Mates      |
                           |       |   |             |      |
                           v       v   |             v      |
          Vertex-->VU[l]<->EU4---->LU2-+<----------->FU2----+

***********************************************************************/
SmBoolean SmVertex::CheckPointers() const
{
  // locals
  ULONG     ii ;
  SmBoolean bRtn = TRUE ;
  SmTArray<SmVertexuse *> sVertexuses ;
  GetVertexuses(sVertexuses) ;

  // For every Face - find all the Vertexuses
  for(ii=0;bRtn==TRUE && ii<sVertexuses.GetSize();ii++)
    {
      SmVertexuse * pVUi    = sVertexuses[ii] ;
      SM_TYPE       eVUType = pVUi->GetVertexuseType() ; // 16005=SmEdgeuse_TYPE,16004=SmLoopuse_TYPE,16013=SmShell_TYPE

      // switch on Vertexuse Type
      switch(eVUType)
        {
          case SmShell_TYPE   :
            { // ShellVertex checks
              //  When VU Type is SmShell_TYPE
              //  1 - VU is sole member of Vertex->Vertexuse list
              //  2 - Shell->m_pList = Vertexuse
              //  3 - Shell->m_pList->m_lListSize == 1

              // locals
              SmShell *pShell = pVUi->GetShell() ;

              // checks
              bRtn &= (pShell != NULL) ;
              if(bRtn)
                {
                  /* 1 */ bRtn &= sVertexuses.GetSize() == 1 ;
                  /* 2 */ bRtn &= pShell->GetList()     == (SmTopology*)pVUi ;
                  /* 3 */ bRtn &= pShell->GetSize()     == 1 ;
                }
            } break ;
          case SmLoopuse_TYPE :
            {
              // locals
              //  When VU Type is SmLoopuse_TYPE
              //   with(LU1   = VU[i]->m_pSorLUorEU (to connected LU)
              //        LU2   = VU[i]->m_pSorLUorEU->LUMate
              //        VU[j] = LU2->m_pEUorVU
              //        V1    = VU[i]->m_pListOwner (to owning vertex)
              //        V2    = VU[j]->m_pListOwner (to owning vertex)
              //        Loop1 = LU1->m_pL
              //        Loop2 = LU2->m_pL
              //        FU1   = LU1->m_pListOwner (to owning Faceuse)
              //        FU2   = LU2->m_pListOwner (to owning Faceuse)
              //        Face1 = FU1->m_pF
              //        Face2 = FU2->m_pF)
              SmLoopuse   * pLU1   = pVUi->GetLoopuse() ;
              SmLoopuse   * pLU2   = pLU1 ? pLU1->GetOtherLoopuse() : NULL ;
              SmVertexuse * pVUj   = pLU2 ? pLU2->GetVertexuse() : NULL ;
              SmVertex    * pV1    = pVUi ? pVUi->GetVertex() : NULL ;
              SmVertex    * pV2    = pVUj ? pVUj->GetVertex() : NULL ;
              SmLoop      * pLoop1 = pLU1 ? pLU1->GetLoop() : NULL ;
              SmLoop      * pLoop2 = pLU2 ? pLU2->GetLoop() : NULL ;
              SmFaceuse   * pFU1   = pLU1 ? pLU1->GetFaceuse() : NULL ;
              SmFaceuse   * pFU2   = pLU2 ? pLU2->GetFaceuse() : NULL ;
              SmFace      * pFace1 = pFU1 ? pFU1->GetFace() : NULL ;
              SmFace      * pFace2 = pFU2 ? pFU2->GetFace() : NULL ;

              // LoopVertex checks - review the picture in the header
              //   1 - (V1 == V2 == this)                                    // unique Vertex
              //   2 - (Loop1 == Loop2)                                      // unique Loop
              //   3 - (Face1 == Face2)                                      // unique Face
              //   4 - (LU1->LUMate == LU2) && (LU2->LUMate == LU1)          // Loopuses are Mates
              //   5 - (FU1->FUMate == FU2) && (FU2->FUMate == FU1)          // Faceuses are Mates
              //   6 - (Face->m_pFU == (FU1 or FU2))                         // Face points back to Faceuses
              //   7 - (Loop->m_pLU == (LU1 or LU2))                         // Loop points back to Loopuses
              //   8 - (FU1->m_pList has LU1)                                // Faceuse points back to Loopuse
              //   9 - (FU2->m_pList has LU2)                                // Faceuse points back to Loopuse
              //  10 - (LU1->m_pEUorVU == VU[i])                             // Loopuse points back to Vertexuse
              //  11 - (LU2->m_pEUorVU == VU[j])                             // Loopuse points back to Vertexuse
              //  12 - VU[i]->m_pListOwner == VU[j]->m_pListOwner == Vertex  // Vertexuse points back to Vertex
              //  13 - (LU1->m_tLoopuseType == LU2->m_tLoopuseType == SmVertexuse_TYPE) // consistent Loopuse types
              // checks
              bRtn &= (   pLU1 && pLU2 && pVUj && pV1 && pV2 && pLoop1 && pLoop2
                       && pFU1 && pFU2 && pFace1 && pFace2) ;
              if(bRtn)
                {
                  /*  1 */ bRtn &= ((pV1 == this) && (pV2 == this)) ;
                  /*  2 */ bRtn &= (pLoop1 == pLoop2) ;
                  /*  3 */ bRtn &= (pFace1 == pFace2) ;
                  /*  4 */ bRtn &= ((pLU1->GetOtherLoopuse() == pLU2) && (pLU2->GetOtherLoopuse() == pLU1)) ;
                  /*  5 */ bRtn &= ((pFU1->GetMate() == pFU2) && (pFU2->GetMate() == pFU1)) ;
                  /*  6 */ bRtn &= ((pFace1->GetUpwardFaceuse() == pFU1) || (pFace1->GetUpwardFaceuse() == pFU2)) ;
                  /*  7 */ bRtn &= ((pLoop1->GetLoopuse() == pLU1) || (pLoop1->GetLoopuse() == pLU2)) ;
                  /*  8 */ bRtn &= (pFU1->IsInList(pLU1)) ;
                  /*  9 */ bRtn &= (pFU2->IsInList(pLU2)) ;
                  /* 10 */ bRtn &= (pLU1->GetVertexuse() == pVUi) ;
                  /* 11 */ bRtn &= (pLU2->GetVertexuse() == pVUj) ;
                  /* 12 */ bRtn &= ((pVUi->GetVertex() == this) && (pVUj->GetVertex() == this)) ;
                  /* 13 */ bRtn &= ((pLU1->GetLoopuseType() == SmVertexuse_TYPE) && (pLU2->GetLoopuseType() == SmVertexuse_TYPE)) ;
                }
            } break ;
          case SmEdgeuse_TYPE :
            {
              // locals
              SmEdgeuse * pEU1    = pVUi->GetEdgeuse() ;
              SmEdgeuse * pEU2    = pEU1 ? pEU1->GetMate() : NULL ;
              SM_TYPE     eEUType = pEU1 ? pEU1->GetEdgeuseType() : SmUnknown_TYPE ;
                                    // 16013=SmShell_TYPE, 16004=SmLoopuse_TYPE
              // switch on Edgeuse TYpe
              switch(eEUType)
                {
                  case SmShell_TYPE   :
                    {
                      // locals
                      // EdgeVertex Part of a Shell
                      //  When VU Type is SmEdgeuse_TYPE and EU1 Type is SmShell_TYPE
                      //   with(EU1   = VU[i]->m_pSorLUorEU (to connected EU)
                      //        EU2   = VU[i]->m_pSorLUorEU->Mate()
                      //        Shell = EU1->m_pSorLU (to connected Shell))
                      SmShell * pShell = pEU1 ? pEU1->GetShell() : NULL ;

                      // Vertex->WireEdge checks - review the picture in the header
                      //   1 - (Shell == EU1->m_pSorLU == EU2->m_pSorLU)   // unique Shell
                      //   2 - (EU1->Mate = EU2->Mate)                     // Edgeuses are Mates
                      //   3 - (EU1->m_pVU == VU[i])                       // Edgeuse points back to Vertexuse
                      //   4 - ((EU1->m_tEdgeuseType == SmShell_TYPE) && (EU2->m_tEdgeuseType->SmShell_TYPE)) // consistent EU types

                      bRtn &= (pEU1 && pEU2 && pShell) ;
                      if(bRtn)
                        {
                          /*  1 */ bRtn &= ((pShell == pEU1->GetShell()) && (pShell == pEU2->GetShell())) ;
                          /*  2 */ bRtn &= ((pEU1->GetMate() == pEU2) && (pEU2->GetMate() == pEU1)) ;
                          /*  3 */ bRtn &= (pEU1->GetVertexuse() == pVUi) ;
                          /*  4 */ bRtn &= ((pEU1->GetEdgeuseType() == SmShell_TYPE) && (pEU2->GetEdgeuseType() == SmShell_TYPE)) ;
                        }

                    } break ;
                  case SmLoopuse_TYPE :
                    {
                      // locals
                      // EdgeVertex Part of an Loopuse
                      //  When VU Type is SmEdgeuse_TYPE and EU1 Type is SmLoopuse_TYPE
                      //   with(EU1   = VU[i]->m_pSorLUorEU (to connected EU)
                      //        EU2   = EU1->Mate()
                      //        EU3   = EU1 ? EU1->CCW : EU1->CW
                      //        EU4   = EU3->Mate
                      //        LU1   = EU1->m_pSorLU (to connected Loopuse)
                      //        LU2   = EU2->m_pSorLU (to connected Loopuse)
                      //        LU3   = EU3->m_pSorLU (to connected Loopuse)
                      //        LU4   = EU4->m_pSorLU (to connected Loopuse)
                      //        Loop1 = LU1->m_pL
                      //        Loop2 = LU2->m_pL
                      //        Loop3 = LU3->m_pL
                      //        Loop4 = LU4->m_pL
                      //        FU1   = EU1->m_pListOwner (to connected Faceuse)
                      //        FU2   = EU2->m_pListOwner (to connected Faceuse)
                      //        FU3   = EU3->m_pListOwner (to connected Faceuse)
                      //        FU4   = EU4->m_pListOwner (to connected Faceuse)
                      //        Face1 = FU1->m_pF
                      //        Face2 = FU2->m_pF
                      //        Face3 = FU3->m_pF
                      //        Face4 = FU4->m_pF
                      //        VUi   = EU1->m_pVU
                      //        VUj   = EU2->m_pVU
                      //        VUk   = EU3->m_pVU
                      //        VUl   = EU4->m_pVU
                      //        Vi    = VUi->m_pListOwner (to connected Vertex)
                      //        Vj    = VUj->m_pListOwner (to connected Vertex)
                      //        Vk    = VUk->m_pListOwner (to connected Vertex)
                      //        Vl    = VUl->m_pListOwner (to connected Vertex)
                      SmEdgeuse   * pEU3   = pEU1 ? pEU1->GetCornerMateEdgeuse(this) : NULL ;
                      SmEdgeuse   * pEU4   = pEU3 ? pEU3->GetMate() : NULL ;
                      SmLoopuse   * pLU1   = pEU1 ? pEU1->GetLoopuse() : NULL ;
                      SmLoopuse   * pLU2   = pEU2 ? pEU2->GetLoopuse() : NULL ;
                      SmLoopuse   * pLU3   = pEU3 ? pEU3->GetLoopuse() : NULL ;
                      SmLoopuse   * pLU4   = pEU4 ? pEU4->GetLoopuse() : NULL ;
                      SmLoop      * pLoop1 = pLU1 ? pLU1->GetLoop() : NULL ;
                      SmLoop      * pLoop2 = pLU2 ? pLU2->GetLoop() : NULL ;
                      SmLoop      * pLoop3 = pLU3 ? pLU3->GetLoop() : NULL ;
                      SmLoop      * pLoop4 = pLU4 ? pLU4->GetLoop() : NULL ;
                      SmFaceuse   * pFU1   = pEU1 ? pEU1->GetFaceuse() : NULL ;
                      SmFaceuse   * pFU2   = pEU2 ? pEU2->GetFaceuse() : NULL ;
                      SmFaceuse   * pFU3   = pEU3 ? pEU3->GetFaceuse() : NULL ;
                      SmFaceuse   * pFU4   = pEU4 ? pEU4->GetFaceuse() : NULL ;
                      SmFace      * pFace1 = pFU1 ? pFU1->GetFace() : NULL ;
                      SmFace      * pFace2 = pFU2 ? pFU2->GetFace() : NULL ;
                      SmFace      * pFace3 = pFU3 ? pFU3->GetFace() : NULL ;
                      SmFace      * pFace4 = pFU4 ? pFU4->GetFace() : NULL ;

                      SmVertexuse * pVUj   = pEU2 ? pEU2->GetVertexuse() : NULL ;
                      SmVertexuse * pVUk   = pEU3 ? pEU3->GetVertexuse() : NULL ;
                      SmVertexuse * pVUl   = pEU4 ? pEU4->GetVertexuse() : NULL ;

                      SmVertex    * pVi    = pVUi ? pVUi->GetVertex() : NULL ;
                      SmVertex    * pVj    = pVUj ? pVUj->GetVertex() : NULL ;
                      SmVertex    * pVk    = pVUk ? pVUk->GetVertex() : NULL ;
                      SmVertex    * pVl    = pVUl ? pVUl->GetVertex() : NULL ;

                      // Vertex->LoopEdge checks - review the picture in the header
                      //   1 - (Vi == Vl == Vertex)                           // unique Vertex
                      //   2 - (Loop1 == Loop2 == Loop3 == Loop4)             // unique Loop
                      //   3 - (face1 == Face2 == Face3 == Face4)             // unique Face
                      //   4 - (EU1->Mate() == EU2 && EU2->Mate() == EU1)     // EUs are Mates
                      //       (EU3->Mate() == EU4 && EU4->Mate() == EU3)     // EUs are Mates
                      //   5 - (LU1->Mate() == LU2 && LU2->Mate() == LU1)     // LUs are Mates
                      //   6 - (FU1->Mate() == FU2 && FU2->Mate() == FU1)     // FUs are Mates
                      //   7 - (Face->m_pFU == (FU1 or FU2)                   // Face points back to Faceuses
                      //   8 - (Loop->m_pLU == (LU1 or LU2)                   // Loop points back to Loopuses
                      //   9 - (FU1->m_pList has LU1)                         // Faceuse points back to Loopuse
                      //  10 - (FU2->m_pList has LU2)                         // Faceuse points back to Loopuse
                      //  11 - (EU1->m_pVU == VUi && EU2->m_pVU == VUj && EU3->m_pVU == VUk && EU4->m_pVU == VUl) // EU points back to VU
                      //  12 - (pVUi->VUType  == pVUj->VUType  == pVUk->VUType  == pVUl->VUType  == SmEdgeuse_TYPE) // consistent VU types
                      //  13 - (pEU1->EUType == pEU2->EUType == pEU3->EUType == pEU4->EUType == SmLoopuse_TYPE) // consistent EU types
                      //  14 - (pLU1->LUType == pLU2->LUType == pLU3->LUType == pLU4->LUType == SmEdgeuse_TYPE) // consistent LU types
                      bRtn &= (   pEU1   && pEU2   && pEU3   && pEU4
                               && pLU1   && pLU2   && pLU3   && pLU4
                               && pLoop1 && pLoop2 && pLoop3 && pLoop4
                               && pFU1   && pFU2   && pFU3   && pFU4
                               && pFace1 && pFace2 && pFace3 && pFace4
                               && pVUi   && pVUj   && pVUk   && pVUl
                               && pVi    && pVj    && pVk    && pVl) ;
                      if(bRtn)
                        {
                          /*  1 */ bRtn &= ((pVi == this) && (pVl == this)) ;
                          /*  2 */ bRtn &= ((pLoop1 == pLoop2) && (pLoop1 == pLoop3) && (pLoop1 == pLoop4)) ;
                          /*  3 */ bRtn &= ((pFace1 == pFace2) && (pFace1 == pFace3) && (pFace1 == pFace4)) ;
                          /*  4 */ bRtn &= (   (pEU1->GetMate() == pEU2) && (pEU2->GetMate() == pEU1)
                                            && (pEU3->GetMate() == pEU4) && (pEU4->GetMate() == pEU3)) ;
                          /*  5 */ bRtn &= ((pLU1->GetOtherLoopuse() == pLU2) && (pLU2->GetOtherLoopuse() == pLU1)) ;
                          /*  6 */ bRtn &= ((pFU1->GetMate() == pFU2) && (pFU2->GetMate() == pFU1)) ;
                          /*  7 */ bRtn &= ((pFace1->GetUpwardFaceuse() == pFU1) || (pFace1->GetUpwardFaceuse() == pFU2)) ;
                          /*  8 */ bRtn &= ((pLoop1->GetLoopuse() == pLU1) || (pLoop1->GetLoopuse() == pLU2)) ;
                          /*  9 */ bRtn &= (pFU1->IsInList(pLU1)) ;
                          /* 10 */ bRtn &= (pFU2->IsInList(pLU2)) ;
                          /* 11 */ bRtn &= (   (pEU1->GetVertexuse() == pVUi) && (pEU2->GetVertexuse() == pVUj)
                                            && (pEU3->GetVertexuse() == pVUk) && (pEU4->GetVertexuse() == pVUl)) ;
                          /* 12 */ bRtn &= (   (pVUi->GetVertexuseType() == SmEdgeuse_TYPE)
                                            && (pVUj->GetVertexuseType() == SmEdgeuse_TYPE)
                                            && (pVUk->GetVertexuseType() == SmEdgeuse_TYPE)
                                            && (pVUl->GetVertexuseType() == SmEdgeuse_TYPE)) ;
                          /* 13 */ bRtn &= (   (pEU1->GetEdgeuseType() == SmLoopuse_TYPE)
                                            && (pEU2->GetEdgeuseType() == SmLoopuse_TYPE)
                                            && (pEU3->GetEdgeuseType() == SmLoopuse_TYPE)
                                            && (pEU4->GetEdgeuseType() == SmLoopuse_TYPE)) ;
                          /* 14 */ bRtn &= (   (pLU1->GetLoopuseType() == SmEdgeuse_TYPE)
                                            && (pLU2->GetLoopuseType() == SmEdgeuse_TYPE)
                                            && (pLU3->GetLoopuseType() == SmEdgeuse_TYPE)
                                            && (pLU4->GetLoopuseType() == SmEdgeuse_TYPE)) ;
                        }

                    } break ;
                  default: bRtn = FALSE ;
                } // end switch on EdgeuseType

            } break ;
          default:
            {
              bRtn = FALSE ;
            }
        } // switch on Vertexuse->VertexuseType
    } // end iter every Vertexuse

  // all done
  return(bRtn) ;

} // end SmVertex::CheckPointers

/*******************************************************************//**
PURPOSE: returns TRUE when all Vertex/Face gaps are less than
         NEWTOL XSectTol3d(Context)
         OLDTOL XSectTol3d(GetBrep)

NOTES: Uses cached Vertex/Face Gaps stored on the Vertex->Vertexuses
       and updates those as necessary.

       When bForceCalc == TRUE, all Vertex/Face gaps have been made
            current when this routine exits.
***********************************************************************/
SmBoolean SmVertex::IsWithinXSectTol3dOfFaces
 (double  * pOptMaxGap3d,   // out: optional max Vertex/Face gap3d found
  SmFace ** pOptMaxGapFace, // out: optional associated Face for found max Edge/Face gap3d
  SmBoolean bForceCalc)     // in : TRUE = force gap evaluation, FALSE=use cache if available
 const                     //      default:[FALSE]
{
  // return value
  SmBoolean bRtn = TRUE ;

  // init output
  if(pOptMaxGap3d)   { *pOptMaxGap3d = 0.0 ; } 
  if(pOptMaxGapFace) { *pOptMaxGapFace = NULL ; }

  // locals
#ifdef SM_USE_NEWTOL
  SmXSectTol3d sXSectTol3d = SmTol::GetXSectTol3d(GetContext()) ;
#else 
  SmXSectTol3d sXSectTol3d = SmTol::GetXSectTol3d(GetBrep()) ;
#endif // SM_USE_NEWTOL
  ULONG ii ;
  SmTArray<SmVertexuse*> sVertexuses ;
  GetVertexuses(sVertexuses) ;

  // when asked - clear Vertex/Face gap cache to force gap calculations for every Vertexuse
  if(bForceCalc)
    {
      for(ii=0;ii<sVertexuses.GetSize();ii++)
        {
          sVertexuses[ii]->m_sVertexFaceGap3d.ReSet() ;
        }
    } // end need to clear all Vertex/Face gap caches check

  // for every Vertexuse
  for(ii=0;ii<sVertexuses.GetSize();ii++)
    {
      SmVertexuse *pVertexuse = sVertexuses[ii] ;

      // look for Vertex/Face gaps that exceed XSectTol3d
      if(pVertexuse->HasVertexFaceGap())
        {
          // side effect: Calc Vertex/Face gap on 1st fetch
          SmVertexFaceGap * pVertexFaceGap = pVertexuse->GetVertexFaceGap(FALSE) ;
          double dGap3d = pVertexFaceGap->m_dGap3d ; // FALSE=calc gap for 1st vertexuse/Face call, fetch for subsequent calls

          // check for gaps that exceed XSectTol3d
          bRtn &= dGap3d < sXSectTol3d ;

          // set output
          if(pOptMaxGap3d && *pOptMaxGap3d < dGap3d)
            { *pOptMaxGap3d = dGap3d ; 
              if(pOptMaxGapFace) { *pOptMaxGapFace = pVertexFaceGap->GetFace() ; }
            }
        }
    } // end iter very Vertexuse

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  if(bDebugMe)
    {
      ULONG di ;
      SmTArray<SmFace*> sFaces ;
      SmTArray<SmEdge*> sEdges ;
      SmBrep *pBrep = GetBrep() ;
      GetFaces(sFaces) ;
      GetEdges(sEdges) ;

      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(7,9, 1,0,0) ; Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook(3,4, 0,1,1) ; for(di=0;di<sFaces.GetSize();di++)
                                    { sFaces[di]->DrawUV(8,8,FALSE,NULL,TRUE) ; sm_GraphicsLoop() ;}
      smgfx_SetLook(5,6, 0,1,0) ; for(di=0;di<sEdges.GetSize();di++)
                                    { SmExtent1d sEdgesInterval (sEdges[di]->GetInterval());
                                      sEdges[di]->GetCurve()->DrawParams(&sEdgesInterval) ; sm_GraphicsLoop() ; }
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

  // all done
  return(bRtn) ;

} // end SmVertex::IsWithinXSectTol3dOfFaces

/*******************************************************************//**
PURPOSE: returns TRUE when all Vertex/Edge gaps are less than
         XSectTol3d(Context)

NOTES: Uses cached Vertex/Edge Gaps stored on the Vertex->Vertexuses
       and updates those as necessary.

       When bForceCalc == TRUE, all Vertex/Edge gaps have been made
            current when this routine exits.
***********************************************************************/
SmBoolean SmVertex::IsWithinXSectTol3dOfEdges
 (double *pOptMaxGap3d,  // out: optional max Vertex/Edge gap3d found
  SmBoolean bForceCalc)  // in : TRUE = force gap evaluation, FALSE=use cache if available
 const                   //      default:[FALSE]
{
  // return value
  SmBoolean bRtn = TRUE ;

  // init output
  if(pOptMaxGap3d) { *pOptMaxGap3d = 0.0 ; }

  // locals
  ULONG ii ;
  SmTArray<SmVertexuse*> sVertexuses ;
  SmXSectTol3d sXSectTol3d = SmTol::GetXSectTol3d(GetContext()) ;
  GetVertexuses(sVertexuses) ;

  // when asked - clear Vertex/Edge gap cache to force gap calculations for every Vertexuse
  if(bForceCalc)
    {
      for(ii=0;ii<sVertexuses.GetSize();ii++)
        {
          sVertexuses[ii]->m_sVertexEdgeGap3d.ReSet() ;
        }
    } // end need to clear all Vertex/Edge gap caches check

  // for every Vertexuse
  for(ii=0;ii<sVertexuses.GetSize();ii++)
    {
      SmVertexuse *pVertexuse = sVertexuses[ii] ;

      // look for Vertex/Edge gaps that exceed XSectTol3d
      if(pVertexuse->HasVertexEdgeGap())
        {
          // Cside effect: Calc Vertex/Edge gap on 1st fetch
          double dGap3d = pVertexuse->GetVertexEdgeGap(FALSE)->m_dGap3d ; // FALSE=calc gap for 1st vertexuse/Edge call, fetch for subsequent calls

          // check for gaps that exceed XSectTol3d
          bRtn &= dGap3d < sXSectTol3d ;

          // set output
          if(pOptMaxGap3d && *pOptMaxGap3d < dGap3d)
            { *pOptMaxGap3d = dGap3d ; }
        }
    } // end iter very Vertexuse

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  if(bDebugMe)
    {
      ULONG di ;
      SmTArray<SmEdge*> sEdges ;
      SmTArray<SmFace*> sFaces ;
      SmBrep *pBrep = GetBrep() ;
      GetEdges(sEdges) ;
      GetEdges(sEdges) ;

      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(7,9, 1,0,0) ; Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook(3,4, 0,1,1) ; for(di=0;di<sFaces.GetSize();di++)
                                    { sFaces[di]->DrawUV(8,8,FALSE,NULL,TRUE) ; sm_GraphicsLoop() ;}
      smgfx_SetLook(5,6, 0,1,0) ; for(di=0;di<sEdges.GetSize();di++)
                                    { SmExtent1d sEdgesInterval (sEdges[di]->GetInterval());
                                      sEdges[di]->GetCurve()->DrawParams(&sEdgesInterval) ; sm_GraphicsLoop() ; }
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

  // all done
  return(bRtn) ;

} // end SmVertex::IsWithinXSectTol3dOfEdges

/*******************************************************************//**
PURPOSE:
idea 2: returns TRUE when all Vertex/Face gaps are less than
        XSectTol3d(Context)
        (became SmVertex::IsWithinXSectTol3dOfFaces)

idea 1: returns TRUE when Vertex->m_vPoint is within ScaledZero of the
                     best point on connected faces that define this vertex,
                  or when the connected faces don't define a best point pos.
        returns FALSE when gap3d between current and best points exceeds
                      SmTol::GetScaledZero(this)
        (became SmVertex::IsOnFaceCorner)

NOTES:
JGU:    ScaledZero of a point doesn't make any sense. A point doesn't have
        a size the way a vector does.
***********************************************************************/
SmBoolean SmVertex::IsOnFaceCorner
 (SmBoolean * pbOptFoundPoint, // out: TRUE = found a Point on connected faces, FALSE=didn't
  double    * pdOptMaxGap3d,   // out: when bFoundPoint == TRUE, Max VertexFace gap from current vertex position
  SmPoint3d * pOptBestPoint)   // out: When bFoundPoint == TRUE, Best possible vertex position to minimize Vertex/Face gaps
 const                         //      otherwise set to uninitialized
{
  // return value
  SmBoolean bRtn = FALSE ;

  // locals
  SmBoolean    bFoundPoint, * pbFoundPoint = pbOptFoundPoint ? pbOptFoundPoint : &bFoundPoint ;
  double       dMaxGap3d,   * pdMaxGap3d   = pdOptMaxGap3d   ? pdOptMaxGap3d   : &dMaxGap3d ;
  SmFace     * pMaxGapFace ;  
  SmPoint3d    sBestPoint,  * psBestPoint  = pOptBestPoint   ? pOptBestPoint   : &sBestPoint ;

  // look for a best vertex point position defined by the faces connected to the vertex
  CalcCornerFromFaces(*pbFoundPoint,   // out: TRUE = found a Point on connected faces, FALSE=didn't
                      *pdMaxGap3d,     // out: when bFoundPoint == TRUE, Max VertexFace gap from current vertex position
                       pMaxGapFace,    // out: when bFoundPoint == TRUE, Face of max VertexFace gap
                      *psBestPoint ) ; // out: When bFoundPoint == TRUE, Best possible vertex position to minimize Vertex/Face gaps
                                       //      otherwise set to uninitialized
  // check the gap
  SmScaledZero sScaledZero = SmTol::GetScaledZero(*this) ;
  //SmXSectTol3d sXSectTol3d = SmTol::GetXSectTol3d(this, pMaxGapFace) ;
  bRtn =    *pbFoundPoint == FALSE  || SmTol::InTol(*pdMaxGap3d, sScaledZero) ;

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  if(bDebugMe)
    {
      ULONG di ;
      SmTArray<SmFace*> sFaces ;
      SmTArray<SmEdge*> sEdges ;
      SmBrep *pBrep = GetBrep() ;
      GetFaces(sFaces) ;
      GetEdges(sEdges) ;

      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(7,9, 1,0,0) ; Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook(3,4, 0,1,1) ; for(di=0;di<sFaces.GetSize();di++)
                                    { sFaces[di]->DrawUV(8,8,FALSE,NULL,TRUE) ; sm_GraphicsLoop() ;}
      smgfx_SetLook(5,6, 0,1,0) ; for(di=0;di<sEdges.GetSize();di++)
                                    { SmExtent1d sEdgesInterval (sEdges[di]->GetInterval() );
                                      sEdges[di]->GetCurve()->DrawParams(&sEdgesInterval) ; sm_GraphicsLoop() ; }
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

  // all done
  return(bRtn) ;

} // end SmVertex::IsOnFaceCorner

/*******************************************************************//**
PURPOSE: Calc Vertex location that minimizes vertex/face
         distances based on current vertex->Face->Surface shapes.

NOTES: This assumes face->Surfaces are the way the model wants
  them to be and finds the best vertex point position that sits
  on those faces.

  When Vertex connects to
    3 nonTangent surfs - rsBestPoint = surf/surf/surf XSect
    2 nonTangent surfs - rsBestPoint = DropPoint(VertexPoint, surf/surf XSect)
    1 nonTangent surfs - rsBestPoint = DropPoint(VertexPoint, surf)
    0 surfs returns SM_SUCCESS with rbFoundPoint = FALSE

***********************************************************************/
SmStatus SmVertex::CalcCornerFromFaces
 (SmBoolean & rbFoundPoint,  // out: TRUE = found a Point on connected faces, FALSE=didn't
  double    & rdMaxGap3d,    // out: when bFoundPoint == TRUE, Max VertexFace gap from current vertex position
  SmFace   *& rpMaxGapFace,  // out: when bFoundPoint == TRUE, Face of max VertexFace gap
  SmPoint3d & rsBestPoint)   // out: When bFoundPoint == TRUE, Best possible vertex position to minimize Vertex/Face gaps
 const                       //      otherwise set to uninitialized
{
  // init output
  rbFoundPoint =  FALSE ;
  rdMaxGap3d   = -1.0 ;
  rpMaxGapFace = NULL ;
  rsBestPoint.SetUninitialized() ;

  // locals
  ULONG ii, lIndx ;
  SmBoolean bFoundAnswer ;
  SmBoolean bIsMulti;
  SmPoint2d sUV ;
  SmPoint3d sSurfaceNormal ;
  SmTArray<SmVertexuse *> sVertexuses, sChkVertexuses ;
  SmTArray<SmFace *>      sFaces ;
  SmTArray<SmFace *>      sTgtFaces ;
  SmTArray<SmPoint2d>     sTgtUVs ;
  SmTArray<SmPoint3d>     sTgtNormals ;

  // get all Vertex topological connections
  GetVertexuses(sVertexuses) ;
  GetFaces(sFaces) ;

  // no work - no faces
  if(sFaces.GetSize() == 0)
    { return(SM_SUCCESS) ; }

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  if(bDebugMe)
    {
      SmTArray<SmFace*> sFaces1 ;
      GetFaces( sFaces1 );

      SmTArray<SmEdge*> sEdges ;
      GetEdges(sEdges) ;

      SmBrep *pBrep = GetBrep();

      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(7,9, 1,0,0) ; Draw() ; sm_GraphicsLoop() ;
      for(ULONG di=0;di<sFaces1.GetSize();di++)
        { SmSurface *pSurface = sFaces1[di] ? sFaces1[di]->GetSurface() : NULL ;
          smgfx_SetLook(3,4, 1,0,1) ; sFaces1[di]->DrawUV(8,8,FALSE,NULL,TRUE) ; sm_GraphicsLoop() ;
          smgfx_SetLook(3,4, 1,0,1) ; sFaces1[di]->Draw() ; sm_GraphicsLoop() ;
          smgfx_SetLook(1,2, 0,1,.2); if(pSurface) pSurface->DrawUV(5,5,0,NULL,TRUE) ; sm_GraphicsLoop() ;
          smgfx_SetLook(3,4, 0,1,1) ; if(pSurface) pSurface->DrawSeams() ; sm_GraphicsLoop() ;
          smgfx_SetLook(3,4, 0,1,1) ; if(pSurface) pSurface->DrawParams() ; sm_GraphicsLoop() ;
        }
      smgfx_SetLook(5,6, 0,1,0) ; for(ULONG di=0;di<sEdges.GetSize();di++)
                                    { SmExtent1d sEdgesInterval (sEdges[di]->GetInterval());
                                      sEdges[di]->GetCurve()->DrawParams(&sEdgesInterval) ; sm_GraphicsLoop() ; }
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

  // Check for Parallel Faces
  double      dAngTolDeg          = SM_ANG_TOL_DEG ;
  double      dMinDihedralAngDeg  = SM_BIG_DOUBLE ;
  double      dMaxParallelFaceGap = 0.0 ;
  SmFace    * pFace1              = NULL ;
  SmFace    * pFace2              = NULL ;
  SmPoint2d   sUV1 ;
  SmPoint2d   sUV2 ;
  SmApproxTol3d sApproxTol3d = SmTol::GetApproxTol3d(GetContext()) ;

  // get Parallel Face MaxGap3d (if any)
  SmBoolean bParallelFaces = HasParallelFaces
                              (&dAngTolDeg,          // in : optional max angle deg between parallel vectors,
                                                     //      NULL=SM_ANG_TOL_DEG, default:[NULL]
                               &dMinDihedralAngDeg,  // out: Min Dihedral AngDeg seen between Faces
                               &dMaxParallelFaceGap, // out: when Rtn == TRUE, Max gap between parallel face pairs
                               &pFace1,              // out: when Rtn == TRUE, optional face1 of MaxParallelFace/Face Gap pair
                               &pFace2,              // out: when Rtn == TRUE, optional face2 of MaxParallelFace/Face Gap pair
                               &sUV1,                // out: when Rtn == TRUE, optional face1 UVPt of MaxParallelFace/Face Gap pair
                               &sUV2) ;              // out: when Rtn == TRUE, optional face2 UVPt of MaxParallelFace/Face Gap pair

  // When Vertex is connected to parallel faces with significant offsets
  if(   bParallelFaces
     && dMaxParallelFaceGap > sApproxTol3d)
    {
      // let Vertex best point be the mid point between the two Faces
      SmPoint3d sPt1, sPt2 ;
      pFace1->GetSurface()->EvaluatePoint(sUV1, sPt1) ;
      pFace2->GetSurface()->EvaluatePoint(sUV2, sPt2) ;

      // set output
      rbFoundPoint = TRUE ;
      rsBestPoint  = (sPt1 + sPt2) / 2.0 ;

      // Init current Vertexuse->VertexFaceGaps
      GetVertexuses(sVertexuses) ;

      // for every Vertexuse - initialize VertexFaceGap values
      for(ii=0;ii<sVertexuses.GetSize();ii++)
        { sVertexuses[ii]->InitVertexFaceGap() ; }

      // Temp change vertex Position to use IsWithinXSectTol3dOfFaces
      SmTemporaryChangeValue<SmPoint3d> sChange1((SmPoint3d&)m_vPoint, rsBestPoint) ;

      // calculate associated MaxGap3d
      IsWithinXSectTol3dOfFaces(&rdMaxGap3d, &rpMaxGapFace, TRUE) ;

      // for every Vertexuse - initialize VertexFaceGap values
      for(ii=0;ii<sVertexuses.GetSize();ii++)
        { sVertexuses[ii]->InitVertexFaceGap() ; }

      // all done
      return(SM_SUCCESS) ;

    } // end Vertex has parallel faces with large offset check

  // arrive here when Vertex is not connected to a parallel faces with a large offset

  // look for up to any 3 connected faces that are not tangent to one another
  for(ii=0;ii<sVertexuses.GetSize();ii++)
    {
      SmVertexuse *pVertexuse = sVertexuses[ii] ;
      SmFaceuse   *pFaceuse   = pVertexuse->GetFaceuse() ;

      // skip vertexuses not connected to a face - typically SmShell_TYPE
      if(pFaceuse == NULL) continue ;

      // Get Vertex->Face dropPoint
      pVertexuse->ComputeUVPoint(sUV, FALSE) ; // FALSE = Use GlobalSolve without touching UVTrimCurve
      SmSurface *pSurface = pFaceuse->GetFace()->GetSurface() ;

      // skip faces without surfaces
      if(pSurface == NULL) continue ;

      // Compute Surface normal for point closest to vertex
      if(SM_SUCCESS != pSurface->EvaluateNormal(sUV, TRUE, TRUE, sSurfaceNormal))
        { continue ; }

      // skip face points already targeted
      if(   sTgtFaces.FindElement(pFaceuse->GetFace(), lIndx)
         && sTgtUVs[lIndx].CloserThan(SmTol::MapTo2d(SmTol::GetZoneTol3d(pFaceuse->GetFace()), sUV, *pSurface), sUV))
       { continue ; }

      // switch on current number of TgtFaces
      switch(sTgtFaces.GetSize())
        {
          case 2 : // skip surfaces with normals parallel to 2nd target
                   if(sSurfaceNormal.IsParallelTo(sTgtNormals[1], 1.0))
                     { break ; }
          case 1 : // skip surfaces with normals parallel to 1st target
                   if(sSurfaceNormal.IsParallelTo(sTgtNormals[0], 1.0))
                     { break ; }
          case 0 : // arrive here - save the face as next target
                   sTgtFaces  .Add(pFaceuse->GetFace()) ;
                   sTgtUVs    .Add(sUV) ;
                   sTgtNormals.Add(sSurfaceNormal) ;
                   break ;
          default: // add vertexuse to the chkVertexuses array for later tolerance computation
                   sChkVertexuses.Add(pVertexuse) ;
                   break ;
        } // end switch on number of found targets
    } // end iter all vertex connections looking for 3 nonTangent surfaces

#ifdef SM_DEBUG_CODE
  if(bDebugMe)
    {
      ULONG di ;
      SmTArray<SmEdge*> sEdges ;
      SmBrep          * pBrep = GetBrep() ;
      SmPoint3d         sPt ;
      GetEdges(sEdges) ;

      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(7,9, 1,0,0) ; Draw() ; sm_GraphicsLoop() ;
      for(di=0;di<sTgtFaces.GetSize();di++)
        { SmSurface *pSurface = sTgtFaces[di] ? sTgtFaces[di]->GetSurface() : NULL ;
          if(pSurface) { pSurface->EvaluatePoint(sTgtUVs[di], sPt) ; }
          smgfx_SetLook(10,12, 0,1,0) ; if(pSurface) { sPt.Draw() ; sTgtNormals[di].Draw(&sPt) ; } sm_GraphicsLoop() ;
          smgfx_SetLook(1,2, 0,0,0) ; sTgtFaces[di]->DrawUV(8,8,FALSE,NULL,TRUE) ; sm_GraphicsLoop() ;
          smgfx_SetLook(3,4, 1,0,1) ; sTgtFaces[di]->Draw() ; sm_GraphicsLoop() ;
          smgfx_SetLook(1,2, 0,1,.2); if(pSurface) pSurface->DrawUV(5,5,0,NULL,TRUE) ; sm_GraphicsLoop() ;
          smgfx_SetLook(3,4, 0,1,1) ; if(pSurface) pSurface->DrawSeams() ; sm_GraphicsLoop() ;
          smgfx_SetLook(3,4, 0,1,1) ; if(pSurface) pSurface->DrawParams() ; sm_GraphicsLoop() ;
          sm_GraphicsLoop() ;
        }
      smgfx_SetLook(5,6, 0,1,0) ; for(di=0;di<sEdges.GetSize();di++)
                                    { SmExtent1d sEdgesInterval (sEdges[di]->GetInterval());
                                      sEdges[di]->GetCurve()->DrawParams(&sEdgesInterval) ; sm_GraphicsLoop() ; }
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

  // switch on the number of TgtFaces
  switch(sTgtFaces.GetSize())
    {
      case 0: {
                SM_DBG_WARN(_T("SmVertex::CalcCornerFromFaces: Unexpected number of TgtFaces - must be a logic bug")) ;
                return(SM_SUCCESS) ;
              }
              break ;

      case 1: { // Drop Current Vertex->Point to Face->Surface
                SmSurface * pSurface = sTgtFaces[0]->GetSurface() ;

                pSurface->DropPoint(GetPoint(), // in : target point to drop
                                    pSurface->GetNaturalUVDomain(), // in : target surface domain
                                    &sTgtUVs[0], // in : When given, uses only local solves
                                    bFoundAnswer, // out: TRUE=Point dropped successfully, FALSE=didn't
                                    sUV, // out: drop result UVPoint
                                    rdMaxGap3d, // out: distance of found point to Pt to drop
                                    bIsMulti); // in : SM_SO_MINIMIZE = allow nonNormal drops near boundaries
                                                  //      SM_SO_NORMALIZE= exclude nonNormal drops near boundaries
                                                  //      SM_SO_INTERSECT= point must be on surface, to Tol

                // no work - no intersection found
                if(bFoundAnswer == FALSE)
                  { return(SM_SUCCESS) ; }
              }
              break ;

      case 2: { // Refine Tgt DropPoints to Intersection Curve between two faces
                SmSurface *pSurface1 = sTgtFaces[0]->GetSurface() ;
                SmSurface *pSurface2 = sTgtFaces[1]->GetSurface() ;
                SmPoint2d  sUVs[2] ;

                // SurfaceIntersector object
                SmSurfaceIntersector sSSI(*pSurface1, pSurface1->GetNaturalUVDomain(),
                                          *pSurface2, pSurface2->GetNaturalUVDomain()) ;

                // refine the DropPoint solutions to a point on the surf/surf intersection
                sSSI.RefinePoint
                  (sTgtUVs[0],     // in : pSurface1 guess point
                   sTgtUVs[1],     // in : pSurface2 guess point
                   SM_IP_CROSSING, // in : CROSSING here works for Tangent curve cases as well
                   NULL,           // in : if given, Force Solution to plane [guessAverage, Normal]
                   NULL,           // in : if given, prevent stepping back to same solution
                   FALSE,          // in : TRUE = Look for Surface/SurfaceBoundary intersections
                   bFoundAnswer,   // out: TRUE = refinement succeeded
                   sUVs) ;         // out: refined UVPoints clamped to Surface Boundaries

                // no work - no intersection found
                if(bFoundAnswer == FALSE)
                  { return(SM_SUCCESS) ; }

                // save UVPoint for 1st surface
                sUV.Set(sUVs[0].x, sUVs[0].y) ;

#ifdef SM_DEBUG_CODE
                if(bDebugMe)
                  {
                    SmPoint3d sSurfacePt1, sSurfacePt2 ;

                    pSurface1->EvaluatePoint(sUVs[0], sSurfacePt1) ; 
                    pSurface2->EvaluatePoint(sUVs[1], sSurfacePt2) ; 

                    //double dSurfSurfDist  = sSurfacePt1.DistanceBetween(sSurfacePt2) ;
                    //double dVertSurf1Dist = sSurfacePt1.DistanceBetween(m_vPoint) ;
                    //double dVertSurf2Dist = sSurfacePt2.DistanceBetween(m_vPoint) ;
                    //double dMaxDist = smos_3Max(dSurfSurfDist, dVertSurf1Dist, dVertSurf2Dist) ;
                  }                             
#endif // SM_DEBUG_CODE

              }
              break ;

      case 3: { // find the surf/surf/surf intersection of the 3 face->Surfaces
                SmSolution sSolution ;
                sTgtFaces[0]->GetSurface()->LocalSurfaceSurfaceIntersect
                  ( sTgtFaces[0]->GetUVDomain(),
                   *sTgtFaces[1]->GetSurface(),
                    sTgtFaces[1]->GetUVDomain(),
                   *sTgtFaces[2]->GetSurface(),
                    sTgtFaces[2]->GetUVDomain(),
                    sTgtUVs[0],
                    sTgtUVs[1],
                    sTgtUVs[2],
                    bFoundAnswer,
                    sSolution) ;

                // no work - no intersection found, this can happen for nearly parallel surfaces that don't intersect over their domain
                if(bFoundAnswer == FALSE)
                  { return(SM_SUCCESS) ; }

                // save UVPoint for 1st surface
                sUV.Set(sSolution.m_vStart[0], sSolution.m_vStart[1]) ;
              }
              break ;
      default:
               SM_DBG_WARN(_T("SmVertex::CalcCornerFromFaces: Unexpected number of TgtFaces - must be a logic bug")) ;
               return(SM_ERR) ;
               break ;
    } // end switch on number of TargetFaces

  // arrive here when sUV contains the new BestPoint on sTgtFaces[0]->GetSurface()
  SM_ASSERT_MSG(bFoundAnswer == TRUE, _T("SmVertex::CalcCornerFromFaces: logic bug - bFound Answer should be TRUE for all paths - must be a bug")) ;

  // set output
  rbFoundPoint = TRUE ;

  // compute 3D point from tgtFace[0]
  sTgtFaces[0]->GetSurface()->EvaluatePoint(sUV, rsBestPoint) ;

  // FaceCornerPoint to VertexPoint gap3d
  rdMaxGap3d = rsBestPoint.DistanceBetween(m_vPoint) ;

  //  // FaceGap to BestPoint
  //  double dMaxFaceGap3d = sSolution.m_vStart.m_dSolutionValue ;
  //
  //  for(ii=0;ii<sChkVertexuses.GetSize();ii++)
  //    {
  //      SmVertexuse   * pVertexuse     = sChkVertexuses[ii] ;
  //      SmVertexFaceGap *pVertexFaceGap = pVertexuse->GetVertexFaceGap( FALSE ) ;
  //
  //      // save largest gap
  //      if(pVertexFaceGap && pVertexFaceGap->GetLength() > rdMaxGap3d)
  //        { dMaxFaceGap3d = pVertexFaceGap->GetLength() ; }
  //
  //    } // end iter all vertexuses not used for surf/surf/surf intersection looking for max gap
  //
  //  // should we add the MaxFaceGap3d value to the rdMaxGap3d value?  GWC: to be decided
  //  rdMaxGap3d += dMaxFaceGap3d ;

  // all done
  return(SM_SUCCESS) ;

} // end SmVertex::CalcCornerFromFaces

/*******************************************************************//**
PURPOSE: Update vertex tolerance to the consistent Tolerance model
         encoded in SmTol::GetTol3d()

NOTES: Also updates tolerance values to the consistent Tolerance model
         For all Edges and Faces connected to this Vertex.
***********************************************************************/

#ifdef SM_USE_OLDTOL
SM_OLDTOL_LINE SmStatus SmVertex::RefreshTolerance( SmBoolean bNestedRefreshes ) // in : TRUE =Refresh tols of connected Faces                              
SM_OLDTOL_LINE {
SM_OLDTOL_LINE   // when asked
SM_OLDTOL_LINE   if(bNestedRefreshes)
SM_OLDTOL_LINE     {
SM_OLDTOL_LINE       // locals
SM_OLDTOL_LINE       ULONG ii ;
SM_OLDTOL_LINE       SmTArray<SmEdge*> sEdges ; GetEdges(sEdges) ;
SM_OLDTOL_LINE       SmTArray<SmFace*> sFaces ; GetFaces(sFaces) ;
SM_OLDTOL_LINE 
SM_OLDTOL_LINE       // for every connected Face and Edge - Set Tol to Consistent tolerance
SM_OLDTOL_LINE       for(ii=0;ii<sFaces.GetSize();ii++) { sFaces[ii]->SetTolerance(SmTol::GetZoneTol3d(sFaces[ii])) ; } 
SM_OLDTOL_LINE       for(ii=0;ii<sEdges.GetSize();ii++) { sEdges[ii]->SetTolerance(SmTol::GetZoneTol3d(sEdges[ii])) ; } 
SM_OLDTOL_LINE                                                                                       
SM_OLDTOL_LINE     }
SM_OLDTOL_LINE 
SM_OLDTOL_LINE   // Set Vertex to consistent tolerance
SM_OLDTOL_LINE   SetTolerance(SmTol::GetZoneTol3d(this)) ;
SM_OLDTOL_LINE                                    // TRUE)) ;           // in : TRUE = force recalc of cached ZoneTol3d
SM_OLDTOL_LINE 
SM_OLDTOL_LINE   // all done
SM_OLDTOL_LINE   return SM_SUCCESS;
SM_OLDTOL_LINE 
SM_OLDTOL_LINE } // end SmVertex::RefreshTolerance
#endif // SM_USE_OLDTOL

//cbi Probably won't do this:
// /*******************************************************************//**
// PURPOSE: Update the tolerance value on this Vertex according to actual gaps.
//
// NOTES: 
// ***********************************************************************/
// SmStatus SmVertex::UpdateToleranceFromGaps()
// {
// #ifdef SM_USE_OLDTOL
// 
//   SM_OLDTOL_LINE SmGapArray sGapArray;
//   SM_OLDTOL_LINE const SmGap *pMaxGap = this->GetMaxGap3d( sGapArray );
//   SM_OLDTOL_LINE double dNewTol = *pMaxGap * 1.01;   //cbi: 1.01 ?
//   SM_OLDTOL_LINE 
//   SM_OLDTOL_LINE // No smaller than any of our Edges' tols.
//   SM_OLDTOL_LINE SmTArray< SmEdge* > sEdges;
//   SM_OLDTOL_LINE this->GetEdges( sEdges );
//   SM_OLDTOL_LINE ULONG ii, lNumEdges = sEdges.GetSize();
//   SM_OLDTOL_LINE for ( ii=0; ii<lNumEdges; ii++ )
//   SM_OLDTOL_LINE {
//   SM_OLDTOL_LINE     if ( dNewTol < sEdges[ii]->GetTolerance() )
//   SM_OLDTOL_LINE       { dNewTol = sEdges[ii]->GetTolerance(); }
//   SM_OLDTOL_LINE }
//   SM_OLDTOL_LINE 
//   SM_OLDTOL_LINE this->SetTolerance( dNewTol, FALSE ); // False: Not only if larger.
// 
// #endif  // SM_USE_OLDTOL
// 
//   return SM_SUCCESS;
// }

/*******************************************************************//**
PURPOSE: Find and remove vertexuse which belongs to a given edge.

NOTES: Will return the removed vertexuse or NULL if not found
***********************************************************************/
SmVertexuse * SmVertex::FindAndRemoveVertexuseByEdge
  (SmEdge * pEdge)
{
    SmTArray<SmVertexuse*> sVertexuses;
    GetVertexuses(sVertexuses);
    for (ULONG i=0; i<sVertexuses.GetSize(); i++)
      {
        SmVertexuse * pVU = sVertexuses[i];
        if (pVU->GetEdgeuse()->GetEdge() == pEdge)
          {
            Remove(pVU);
            return pVU;
          }
      }
    return NULL;

} // end SmVertex::FindAndRemoveVertexuseByEdge

/*******************************************************************//**
PURPOSE: Receive notification of things happening to the object and
take appropriate actions.

  NOTES: For example cleaning up the cache of an
object which is being deleted or edited.  Also clean up any attributes
and relations specific to this class which are not handled automatically
by construtors.
***********************************************************************/
void SmVertex::Notify                  // expected calls: caller->Notify(Event, pData1, pData2, pData2)
 (SmNotifyOperation  eNotifyOperation, //       event                | caller      |  pData1  | pData2                | pData3
  SmObject         * pData1,           //----------------------------+-------------+----------+-----------------------+--------------------------
  SmObject         * pData2,           // SM_NO_ADD_TO_BREP          | Brep        | AddObj   | Brep                  | AddObj's GeomPtr or NULL
  SmObject         * pData3)           // SM_NO_SPLIT_IN_BREP        | Brep/TopoObj| OrigObj  | Child1                | Child2
                                       // SM_NO_MERGE_IN_BREP        | Brep/TopoObj| SurvObj  | DelObj                | Brep
                                       // SM_NO_TRIM_NO_SPLIT_IN_BREP| Brep        | TgtObj   | AddedBndryObj         | NULL                     
                                       // SM_NO_COINCIDENT           | BrepA       | BrepAObj | BrepBObj              | BrepB
                                       // SM_NO_RM_FROM_BREP         | Brep        | RmObj    | Brep                  | RmObj's GeomPtr or NULL
                                       // SM_NO_CHANGE_GEOMETRY      | TopoObj     | NewGeom  | Brep or NULL          | OldGeom or NULL
                                       // SM_NO_CHANGE_OWNER         | GeomObj     | NewOwner | NewOwner Brep or NULL | OldOwner or NULL
                                       // SM_NO_CONSTRUCTION         | NewObj      |  NewObj  | CopyFromObj or NULL   | NULL
                                       // SM_NO_COPY                 | FromObj     | ToObj    | ToObj's Owner or NULL | FromObj's Owner or NULL
                                       // SM_NO_PRE_EDIT             | EditObj     | EditObj  | EditObj Owner or NULL | NULL
                                       // SM_NO_POST_EDIT            | EditObj     | EditObj  | EditObj Owner or NULL | NULL
                                       // SM_NO_SPLIT                | SplitGeomObj| Child1   | Child2                | SplitObj's Owner or NULL
                                       // SM_NO_MERGE                | MergeGeomObj| OrigObj1 | OrigObj2              | MergeObj's Owner or NULL
                                       // SM_NO_REG_PROPAGATION      | MergeReg    | ThisRegs | OtherBrep->SrcRegs    | ThisBrep->MergeReg
                                       // SM_NO_DESTRUCTION          | DelObj      | DelObj   |  NULL                 |  NULL                  
{
  switch (eNotifyOperation)
    {
      case SM_NO_PRE_EDIT              : break ;
                                       
      case SM_NO_POST_EDIT             :
      case SM_NO_SPLIT_IN_BREP         :
      case SM_NO_MERGE_IN_BREP         :
      case SM_NO_CHANGE_GEOMETRY       :
                                         // pass the notify call along to all Vertexuses, Edgeuses, and Edgeuse->Vertexuses
                                         // so that those can clean up their cached gaps
                                         {
                                           ULONG ii ;
                                           SM_PTR_ARRAY(sVertexuses, SmVertexuse, 32) ;
                                           GetVertexuses(sVertexuses) ;
                                           for(ii=0;ii<sVertexuses.GetSize();ii++)
                                             { sVertexuses[ii]->Notify(eNotifyOperation, pData1, pData2, pData3) ; }
                                         }
                                         break ;
                                       
      case SM_NO_ADD_TO_BREP           : break ;
      case SM_NO_TRIM_NO_SPLIT_IN_BREP : break ;
      case SM_NO_COINCIDENT            : break ;
      case SM_NO_RM_FROM_BREP          : break ;
      case SM_NO_CHANGE_OWNER          : break ;
      case SM_NO_CONSTRUCTION          : break ;
      case SM_NO_COPY                  : break ;
      case SM_NO_SPLIT                 : break ;
      case SM_NO_MERGE                 : break ;
      case SM_NO_REG_PROPAGATION       : break ;
      case SM_NO_DESTRUCTION           : break ;
      case SM_NO_UNKNOWN:                { SE_MSG(SM_ERR, _T("SmEdge::Notify - SM_NO_UNKNOWN event signalled")) ; }
                                         break ;
    }

  // Propagate notification up hierarchy
  SmTopology::Notify(eNotifyOperation,pData1,pData2,pData3);

} // end SmVertex::Notify

/*******************************************************************//**
PURPOSE: Output vertex->Point graphics

NOTES:
***********************************************************************/
SmStatus SmVertex::OutputGraphics
 (SmGfxArraySet * pOptGfxSet)   // i/o: used for OpenGL ES 2.0. When given, accumulate GfxVertexArrays not GL calls.
                                //      NULL to ignore. default:[NULL]
 const
{
#ifdef SM_GFX_OUTPUT_CODE
    smgfx_OutputPoint(m_vPoint.x,m_vPoint.y,m_vPoint.z, pOptGfxSet);
#else
  SM_REF1(pOptGfxSet);
#endif

    return SM_SUCCESS;

} // end SmVertex::OutputGraphics

/*******************************************************************//**
PURPOSE: Compute the total size of the memory used by the SmVertex,
            all its SmVertexuses, and their attributes.

NOTES:
***********************************************************************/
ULONG SmVertex::GetMemoryUsed        // rtn: smaller size of actually used memory in bytes
  (ULONG    & rlMemoryAllocated,     // out: bigger size of all allocated memory in bytes
   SmMarkType eMarkType)             // in : uses without increment eMarkType value
  const
{
  // locals
  ULONG ii, lThisAllocated ;

  // in case this method is called directly - get a mark for attribute memory usage
  SmNewMarkAndLock sMarkLock ;
  if(eMarkType == SM_MT_NOMARK)
    {
      eMarkType = sMarkLock.SetContext((SmContext *)GetContext()) ;
    }

  // This + attribute memory
  ULONG lUsed       = sizeof(*this) + this->GetAttributeMemoryUsed(lThisAllocated,
                                                                   eMarkType) ;  // note: uses without increment eMarkType value
  rlMemoryAllocated = sizeof(*this) + lThisAllocated ;

  // for every Vertexuse - add in object and attribute memory
  SmVertexuse *pVU, *pVertexuse = (SmVertexuse *)m_pList ;
  for(pVU  = pVertexuse,   ii=0;
      pVU != pVertexuse || ii==0;
      pVU  = (SmVertexuse*)pVU->m_pNext, ii++)
    {
      lUsed             += sizeof(*pVU) + pVU->GetAttributeMemoryUsed(lThisAllocated,
                                                                      eMarkType) ;  // note: uses without increment eMarkType value
      rlMemoryAllocated += sizeof(*pVU) + lThisAllocated ;
    }

  // all done
  return(lUsed) ;

} // end SmVertex::GetMemoryUsed

/*******************************************************************//**
PURPOSE: AssertValid() Test reporting static labels
***********************************************************************/
SmAssertReportLabel sAssertVertex_list[] =
{
  /*  0 */ {SM_AT_GEOMETRIC,   _T("Bad Vertex Tol"), _T("stored Vertex ZoneTol not equal the SmTol::Consistent Tolerance Model value") },
  /*  1 */ {SM_AT_GEOMETRIC,   _T("Bad Vertex Gap"), _T("Vertex-Face Gap exceeds XSectTol3d for one (or more) Faces") },
  /*  2 */ {SM_AT_TOPOLOGICAL, _T("Bad Topology"),   _T("Related TopologyGraph Ptrs are not consistent for all Vertex->Vertexuses.") }
} ; // end sAssertVertex_list[]

/*******************************************************************//**
AssertRule Predicate names and return values:
  predicate = SmBoolean sm_AssertTestClassNameRule#() ; Rtn: TRUE=OK,    FALSE=Bad,
***********************************************************************/

/*******************************************************************//**
PURPOSE  : SmVertex AssertRule 1 -
  When Vertex is connected to multiple faces, Vertex should be tightly
  placed on the vertex->faces' surf/surf/surf intersection point.

  predicate = return TRUE when Vertex is not connected to 3 distinct faces or
                          when Vertex->m_vPoint is within SmTol::GetScaledZero()
                          of the surf/surf/surf Point.

  action    = when Vertex is connected to 3 distinct faces
              move Vertex->m_vPoint to surf/surf/surf intersection,
              otherwise don't move the Vertex->m_vPoint
***********************************************************************/
SmBoolean sm_AssertTestVertex1          // rtn: TRUE = okay, FALSE = problem
 (const SmVertex * pVertex,             // in : test target
  double         * pOptMaxGap3d=NULL,   // out: current max Vertex/Face gap
  SmFace        ** pOptMaxGapFace=NULL, // out: current max Vertex/Face face
  SmBoolean      * pOptFoundPoint=NULL, // out: TRUE = Corner Faces define a corner point (surf/surf/surf xsect)
  double         * pOptMove3d=NULL,     // out: opt Vertex->m_vPoint to Surf/Surf/Surf intersection point dist
  SmPoint3d      * pOptBestPoint=NULL)  // out: opt Surf/Surf/Surf intersection point
{
  SmBoolean bFoundPoint ;
  SmFace  * pThisMaxGapFace,  ** pMaxGapFace = pOptMaxGapFace ? pOptMaxGapFace : &pThisMaxGapFace ; 

  // Vertex/Face gaps <= XSectTol3d
  SmBoolean bRtn = pVertex->IsWithinXSectTol3dOfFaces(pOptMaxGap3d, pMaxGapFace) ;

  // bRtn == FALSE when Vertex Point is more than ScaledZero from best corner pos defined by connected faces
  if(bRtn || pOptFoundPoint || pOptMove3d || pOptBestPoint)
    {
      bRtn &= pVertex->IsOnFaceCorner(pOptFoundPoint ? pOptFoundPoint
                                                     : &bFoundPoint,
                                      pOptMove3d,
                                      pOptBestPoint) ;
    }

  // GWC_NEEDS_WORK Decide_which_of_the_two_above_tests_to_use GWC_LINE ;

  // all done
  return( bRtn ) ;

} // end sm_AssertTestVertex1

// obsolete
// /*******************************************************************//**
// PURPOSE  : SmVertex AssertRule 1 = Place Vertex at surf/surf/surf XSect
// ***********************************************************************/
// SmBoolean sm_AssertHealVertex1
//  (SmVertex       * pVertex,   // in :
//   SmAssertReport & rAReport,  // in :
//   SmAssertArray  * pAList)    // NotUsed: in :
// {
//   SM_REF1(pAList) ;
//   // remember Heal has run on this AssertReport
//   rAReport.m_eAssertType = SM_AT_HEALER ;
// 
//   // locals
//   double    dMaxGap3d ;   // current max Vertex/Face gap
//   SmFace  * pMaxGapFace ; // current max Vertex/Face face
//   SmBoolean bFoundPoint ; // TRUE = Vertex corner faces define a corner point
//   SmPoint3d sBestPoint ;  // Best Vertex move to point from CalcCornerFromFaces()
//   double    dMove3d ;     // Vertex->m_vPoint to CalcCornerFromFaces() sBestPoint point dist
//   double    dMaxVertexFaceGap3d ; // after move max Vertex/Face Gap
//   SmFace  * pMaxVertexFaceGapFace ; 
//   SmPoint3d sVertexPoint = pVertex->GetPoint() ;
// 
//   // check test - other fix functions may have already fixed this one
//   rAReport.m_bOK = sm_AssertTestVertex1(pVertex, &dMaxGap3d, & pMaxGapFace, &bFoundPoint, &dMove3d, &sBestPoint) ;
//   if(TRUE == rAReport.m_bOK)
//     { return(rAReport.m_bOK) ; }
// 
//   // fix: Move Vertex to sBestPoint if new BestPoint is different than current Vertex->Point
//   if(bFoundPoint && dMove3d > 0.0)
//     { pVertex->SetPoint(sBestPoint) ; } // FALSE = always update m_sZoneTol3d
// 
//   // check the change - not all Gap problems can be fixed by moving the Vertex->Point
//   //                    some faces are parallel with a large offset between them.
//   // rAReport.m_bOK = sm_AssertTestVertex0(pVertex, &sZoneTol3d) ;
//   rAReport.m_bOK = pVertex->IsWithinXSectTol3dOfFaces(&dMaxVertexFaceGap3d, &pMaxVertexFaceGapFace, TRUE) ;
// 
//   // do no harm
//   if(dMaxVertexFaceGap3d > dMaxGap3d)
//     {
//       // go back to original Vertex point
//       pVertex->SetPoint(sVertexPoint) ;
//     }
// 
//   // Good Citizenship - log all Object changes with pAList here:
//   //                      pAList->LogReplaceObject(),
//   //                      pAList->LogSplitObject(),
//   //                      pAList->LogMergeObject(),
//   //                      pAList->LogDeleteObject().
//   // No Object changes to log.
// 
//   // all done
//   if(rAReport.m_bOK) { rAReport.m_pHealMessage = _T("Bad SmVertex::Point3d moved to attached Surf/Surf/Surf intersection point.") ; }
//   else               { rAReport.m_pHealMessage = _T("Bad SmVertex::Point3d move couldn't tighten Vertex/Face gaps(often due to offset parallel surfaces)") ; }
//   return(rAReport.m_bOK) ;
// 
// } // end sm_AssertHealVertex1
// end obsolete

/*******************************************************************//**
PURPOSE  : SmVertex AssertRule 0 -
  Vertex stored ZoneTol must equal SmTol::GetZoneTol3d() Consistent Tolerance Model value

  predicate = if(SM_USE_NEWTOL) return TRUE
              else return SM_ARE_SAME(m_sZoneTol3d, SmTol::GetZoneTol3d(this))

  action    = if(SM_USE_NEWTOL) no action
              else if m_sZoneTol3d not equal SmTol::GetZoneTol3d(this)
                      set m_sZoneTol3d = SmTol::GetZoneTol3d(this)
***********************************************************************/
SmBoolean sm_AssertTestVertex0         // rtn: TRUE = okay, FALSE = problem
 (const SmVertex * pVertex,            // in : test target
  SmZoneTol3d    * pOptZoneTol3d=NULL) // out: opt SmTol::GetZoneTol3d(pVertex) value, NULL to ignore
{
  SmBoolean bRtn = pVertex->IsZoneTol3dConsistent(pOptZoneTol3d) ;

  return( bRtn ) ;

} // end sm_AssertTestVertex0

// obsolete
// /*******************************************************************//**
// PURPOSE  : SmVertex AssertRule 0 - Vertex stored ZoneTol must equal
//            SmTol::GetZoneTol3d() Consistent Tolerance Model value
// ***********************************************************************/
// SmBoolean sm_AssertHealVertex0
//  (SmVertex       * pVertex,   // in :
//   SmAssertReport & rAReport,  // in :
//   SmAssertArray  * pAList)    // NotUsed: in :
// {
//   SM_REF1(pAList) ;
//   // remember Heal has run on this AssertReport
//   rAReport.m_eAssertType = SM_AT_HEALER ;
// 
//   // locals
//   SmZoneTol3d sZoneTol3d ;
// 
//   // check test - other fix functions may have already fixed this one
//   rAReport.m_bOK = sm_AssertTestVertex0(pVertex, &sZoneTol3d) ;
//   if(TRUE == rAReport.m_bOK)
//     { return(rAReport.m_bOK) ; }
// 
//   // fix broken stored ZoneTol3d value - by replacing it with the SmTol::GetZoneTol3d() value.
//   pVertex->SetTolerance(sZoneTol3d, FALSE) ;  // FALSE = always update m_sZoneTol3d
// 
//   // Good Citizenship - log all Object changes with pAList here:
//   //                      pAList->LogReplaceObject(),
//   //                      pAList->LogSplitObject(),
//   //                      pAList->LogMergeObject(),
//   //                      pAList->LogDeleteObject().
//   // No Object changes to log.
// 
//   // no need to check the change - we know that this problem is fixed
//   // rAReport.m_bOK = sm_AssertTestVertex0(pVertex, &sZoneTol3d) ;
//   rAReport.m_bOK = TRUE ;
// 
//   // all done
//   if(rAReport.m_bOK) { rAReport.m_pHealMessage = _T("InConsistent SmVertex::ZoneTol3d value reset to SmTol::GetZoneTol3d(pVertex) value.") ; }
//   else               { rAReport.m_pHealMessage = _T("InConsistent SmVertex::ZoneTol3d value could not be fixed - Something went wrong, report as bug") ; }
//   return(rAReport.m_bOK) ;
// 
// } // end sm_AssertHealVertex0
// end obsolete
  
/*******************************************************************//**
PURPOSE:

RETURNS ---  TRUE  = OK
             FALSE = Problem
***********************************************************************/
SmBoolean SmVertex::AssertValid
 (SmAssertArray    * pAList,        // i/o: Accumulating list of failed Asserts, NULL to ignore, default:[NULL]
  SmAssertTestLevel  eTestLevel,    // in : SM_LEVEL_0=fewest tests, SM_LEVEL_1, SM_LEVEL_2=all tests,
                                    //      SM_LEVEL_GIVEN = run tests in order requested in pTestRequests
                                    //      default:[SM_LEVEL_0]
  SmAssertWalking    eWalkTree,     // NotUsed: in : SM_WALK = run AssertValid on any topology graph descendants, SM_NO_WALK=don't, default:[SM_WALK]
  SmTArray<ULONG>  * pTestRequests) // in : when eTestLevel == SM_LEVEL_GIVEN, run these tests in this order, default:[NULL]
 const
{
  SM_REF1(eWalkTree) ;
  // init rtn value
  SmBoolean bRtn = TRUE ;

  // call the base class AssertValid
  bRtn &= (  (eTestLevel != SM_LEVEL_GIVEN)
           ? SmOwningTopology::AssertValid(pAList, eTestLevel, SM_NO_WALK, pTestRequests)
           : TRUE ) ;

  // double  dBrepTol = GetBrep() ? (double)GetBrep()->GetTolerance() : 0.0 ;

  /* 0 */ // Vertex stored ZoneTol must equal the SmTol::Consistent Tolerance Model value
#ifdef SM_USE_OLDTOL
#ifdef SM_NMTLIB_7166
  bRtn &= SM_ASSERT_BOOLEAN_REPORT(0, SM_LEVEL_2, sm_AssertTestVertex0(this) == TRUE, _T("")) ;
#endif // SM_NMTLIB_7166
#endif // SM_USE_OLDTOL

  /*  1 */ // Vertex location must be tightly located on FaceCorner (surf/surf/surf XSect)
  double    dMaxGap3d ;
  SmFace  * pMaxGapFace ; 
  SmBoolean bVertLocTest = sm_AssertTestVertex1(this, &dMaxGap3d, &pMaxGapFace) ;
  if(bVertLocTest == FALSE) // Max Gap larger than SmTol::GetScaledZero(this)
    {
#ifdef SM_USE_NEWTOL
      SmXSectTol3d sXSectTol3d = SmTol::GetXSectTol3d(GetContext()) ;
#else
      SmXSectTol3d sXSectTol3d = SmTol::GetXSectTol3d(GetBrep()) ;
#endif // SM_USE_NEWTOL
      // SmScaledZero sScaledZero = SmTol::GetScaledZero(*this) ;
      if(dMaxGap3d > sXSectTol3d)
        { bRtn &= SM_ASSERT_VALUE_REPORT(1, SM_LEVEL_2, bVertLocTest == TRUE, sXSectTol3d, dMaxGap3d, _T("")) ; }
      else // dMaxGap is greater than sScaledZero and smaller than XSectTol3d
        { 
          // GWC_NEEDS_WORK ADD_AN_ASSERT_REPORT_WARNING_CAPABILITY GWC_LINE ;
          // gwc: need code here that puts out a warning - bigger than ScaledZero but less than XSectTol3d
          //      so that the heal method can be run to tighten up the tolerances.
        }
    } // end bVertLocTest failed check

  /*  2 */ // Vertex has multiple Vertexuse Vertex/Face connections one of which is a VertexLoop.
  bRtn &= SM_ASSERT_BOOLEAN_REPORT(2, SM_LEVEL_0, CheckPointers() == TRUE, _T("")) ;

  // Check all connected Vertexuses
  SmTArray<SmVertexuse *> sVertexuses ;
  GetVertexuses(sVertexuses) ;
  ULONG ii ;
  for(ii=0;ii<sVertexuses.GetSize();ii++)
    {
      if(sVertexuses[ii]) { bRtn &= sVertexuses[ii]->AssertValid(pAList, eTestLevel, SM_NO_WALK) ; }
    }

  // all done
  // SM_ASSERT(bRtn) ;
  return(bRtn) ;

} // end SmVertex::AssertValid

// obsolete
// /*******************************************************************//**
// PURPOSE: Fix Assert Report failures reported by an AssertValid call
// 
// RETURNS ---  TRUE  = OK
//              FALSE = Problem
// ***********************************************************************/
// SmBoolean SmVertex::AssertHeal
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
//       case 0  : // Vertex stored ZoneTol must equal the SmTol::Consistent Tolerance Model value
//                 bRtn = sm_AssertHealVertex0(this, rAReport, pAList); break;
// 
//       case 1  : // Vertex location must be tightly located on FaceCorner (surf/surf/surf XSect)
//                 bRtn = sm_AssertHealVertex1(this, rAReport, pAList); break;
// 
//       case 99 : { // set case number appropriately - run fix code here
//                   // if fix works set rAReport.m_bOK = TRUE ;
//                 }
//                 break ;
// 
//       default: rAReport.m_eAssertType  = SM_AT_NO_HEAL_YET ;
//                rAReport.m_pHealMessage = _T("SmVertex::AssertHeal fix not yet supported") ;
// 
//     }
// 
//   // all done
//   return(bRtn) ;
// 
// } // end SmVertex::AssertHeal
// end obsolete

/*******************************************************************//**
PURPOSE: Draw Vertex

NOTES:
***********************************************************************/

SmDisplayList * SmVertex::Draw
  (SmGfxArraySet * pOptGfxSet)   // i/o: used for OpenGL ES 2.0. When given, accumulate GfxVertexArrays not GL calls.
                                 //      NULL to ignore. default:[NULL]
 const
{
  SmDisplayList *pRtn = NULL ;

#ifdef SM_GFX_CODE
  // Get global SmGraphicsExtern.cpp:s_Disp display parameters
  SmDisplayParameters sDisp ;
  smgfx_GetGlobalDisplayParameters(sDisp) ;

  // look for neighbor drawing
  if(sDisp.m_bDrawNeighbors)
    { return(DrawNeighbors(pOptGfxSet)) ; }

  // start new displayList (unless one is already open)
  SmVector3d sColor = smgfx_GetOutputColor(pOptGfxSet) ;
  smgfx_Open(smgfx_GetRuleColor(this),NULL,NULL,FALSE,pOptGfxSet);

  // output the graphics
  smgfx_OutputPoint(m_vPoint.x,m_vPoint.y,m_vPoint.z,pOptGfxSet);

  // end display list
  pRtn = smgfx_Close(pOptGfxSet) ;
  smgfx_OutputColor(sColor,pOptGfxSet) ;

#else
  SM_REF1(pOptGfxSet);
#endif // SM_GFX_CODE
  return(pRtn) ;

} // end SmVertex::Draw

/*******************************************************************//**
PURPOSE: Draw Vertex and any edgeuses (and edgeuse mates) to which it connects

NOTES:
***********************************************************************/
SmDisplayList * SmVertex::DrawNeighbors
 (SmGfxArraySet * pOptGfxSet)   // i/o: used for OpenGL ES 2.0. When given, accumulate GfxVertexArrays not GL calls.
                                //      NULL to ignore. default:[NULL]
 const
{
  SmDisplayList *pRtn = NULL ;

#ifdef SM_GFX_CODE
  SmTArray <SmVertexuse *> sVertexuses ;
  GetVertexuses(sVertexuses) ;

  // start new displayList (unless one is already open)
  SmVector3d sColor = smgfx_GetOutputColor(pOptGfxSet) ;
  smgfx_Open(smgfx_GetRuleColor(this),NULL,NULL,FALSE,pOptGfxSet);

  // output the vertex graphics
  smgfx_OutputPoint(m_vPoint.x,m_vPoint.y,m_vPoint.z,pOptGfxSet);

  // for every vertexuse - draw the edgeuse and edgeuse->mate
  ULONG ii ;
  for(ii=0;ii<sVertexuses.GetSize();ii++)
    {
      SmEdgeuse *pEdgeuse = sVertexuses[ii]->GetEdgeuse() ;
      SmEdgeuse *pMateuse = pEdgeuse ? pEdgeuse->GetMate() : NULL ;

      // when there is an edgeuse connected to this vertexuse
      if(pEdgeuse)
        {
          SmEdge *pEdge = pEdgeuse->GetEdge() ;

          // draw connected edgeuse in the neighbor color
          SmVector3d sColor2 = smgfx_SetColor(smgfx_GetRuleColor(this, SM_CR_NEIGHBOR),pOptGfxSet) ;
          pEdge->DrawParams(pOptGfxSet) ;
          pEdgeuse->Draw(1.0, FALSE, pOptGfxSet) ;    // FALSE = don't draw UVTrimCurves
          smgfx_SetColor(sColor2, pOptGfxSet) ;
        }

      // when there is an edgeuse mate connected to this vertexuse
      if(pMateuse)
        {
          // draw connected edgeuse's mate in the neighborMate color
          SmVector3d sColor2 = smgfx_SetColor(smgfx_GetRuleColor(this, SM_CR_NEIGHBORMATE), pOptGfxSet) ;
          pMateuse->Draw(1.0, FALSE, pOptGfxSet) ;    // FALSE = don't draw UVTrimCurves
          smgfx_SetColor(sColor2, pOptGfxSet) ;
        }
    } // end iter every vertexuse

  // end display list
  pRtn = smgfx_Close(pOptGfxSet) ;
  smgfx_OutputColor(sColor, pOptGfxSet) ;

#else
  SM_REF1(pOptGfxSet);
#endif // SM_GFX_CODE
  return(pRtn) ;

} // end SmVertex::DrawNeighbors

/*******************************************************************//**
PURPOSE: Draw Micro view of geometry connecting to vertex

NOTES:
***********************************************************************/
SmDisplayList * SmVertex::DrawMicro
 (SmGfxArraySet * pOptGfxSet)   // i/o: used for OpenGL ES 2.0. When given, accumulate GfxVertexArrays not GL calls.
                                //      NULL to ignore. default:[NULL]
 const
{
  SmDisplayList *pRtn = NULL ;

#ifdef SM_GFX_CODE
  SmTArray <SmVertexuse *> sVertexuses ;
  GetVertexuses(sVertexuses) ;

  // start new displayList (unless one is already open)
  SmVector3d sColor = smgfx_GetOutputColor(pOptGfxSet) ;
  smgfx_Open(smgfx_GetRuleColor(this),NULL,NULL,FALSE,pOptGfxSet);

  // output the vertex graphics
  smgfx_OutputPoint(m_vPoint.x,m_vPoint.y,m_vPoint.z,pOptGfxSet);

  // for every vertexuse - draw Vertexuse MicroGraphics
  ULONG ii ;
  for(ii=0;ii<sVertexuses.GetSize();ii++)
    {
      // draw MicroGraphics
      sVertexuses[ii]->DrawMicro(pOptGfxSet) ;

    } // end iter every vertexuse

  // end display list
  pRtn = smgfx_Close(pOptGfxSet) ;
  smgfx_OutputColor(sColor, pOptGfxSet) ;

#else
  SM_REF1(pOptGfxSet);
#endif // SM_GFX_CODE
  return(pRtn) ;

} // end SmVertex::DrawMicro

/*************************************************************
PURPOSE:  return vertex's largest Vertex/Edge or Vertex/Face gap
             or NULL for no gaps(not connected)
NOTES:
**************************************************************/
const SmGap * SmVertex::GetMaxUpDimGap3d
 (SmGapArray * pOptGapArray, // out: optional List of all Vertex/Edge and Edge/Face Gap3ds, NULL to ignore, default:[NULL]
  SmTol3d    * pOptTol3d)    // in : NotNULL        = only load Gaps larger than OptTol3d into OptGapArray.
                             //      default:[NULL] = load all Gaps into OptGapArray
 const
{
  // init output
  if(pOptGapArray) { pOptGapArray->ReSet() ; }

  // locals
  ULONG ii ;
  SM_PTR_ARRAY(sVertexuses, SmVertexuse, 16) ;
  GetVertexuses(sVertexuses) ;

  // iter init
  SmGap * pMaxGap = NULL ;

  // for every
  for(ii=0;ii<sVertexuses.GetSize();ii++)
    {
      // Vertex/Edge Gap
      SmGap * pVertexEdgeGap = sVertexuses[ii]->GetVertexEdgeGap() ;
      if(pVertexEdgeGap)
        {
          // Build Gap Array
          if(pOptTol3d == NULL || pVertexEdgeGap->GetLength() > pOptTol3d->val)
            { pOptGapArray->Add(pVertexEdgeGap) ; } 

          // Set Output
          if(pMaxGap == NULL || *pMaxGap < *pVertexEdgeGap) 
            { pMaxGap = pVertexEdgeGap ; }
        } // end pVertexEdgeGap existence check

      // Vertex/Face Gap
      SmGap * pVertexFaceGap = sVertexuses[ii]->GetVertexFaceGap() ;
      if(pVertexFaceGap)
        {
          // Build Gap Array
          if(pOptTol3d == NULL || pVertexFaceGap->GetLength() > pOptTol3d->val)
            { pOptGapArray->Add(pVertexFaceGap) ; } 

          // Set Output
          if(pMaxGap == NULL || *pMaxGap < *pVertexFaceGap) 
            { pMaxGap = pVertexFaceGap ; }
        } // end pVertexFaceGap existence check

    } // end iter every vertexuse

  // all done
  return(pMaxGap) ;

} // end SmVertex::GetMaxUpDimGap3d

/*************************************************************
PURPOSE:  Get this vertex number within brep

NOTES:
  Returns -1 if this is not a vertex in a Brep.
**************************************************************/
ULONG SmVertex::GetVertexNumberInBrep()
 const
{
  SmBrep *pBrep = GetBrep();
  SM_ASSERT(pBrep != NULL) ;

  // sadly SMLib uses ULONG and not ints for indexing
  // one can not set a ULONG value to -1
  //      ULONG lVertexNum = -1;
  ULONG lVertexNum = 0 ;

  SmTArray<SmVertex*> sAllVerts;
  pBrep->GetVertices( sAllVerts );

  for (ULONG i = 0; i < sAllVerts.GetSize(); i++ )
  {
      if ( sAllVerts[i] == this )
      {
          lVertexNum = i;
          break;
      }
  }

  return lVertexNum;

} // end SmVertex::GetVertexNumberInBrep

/*******************************************************************//**
PURPOSE: Is this vertex a single shell - not connected to an edge
            located in a region.

NOTES:
***********************************************************************/
SmBoolean SmVertex::IsShellVertex() const
{
  SmVertexuse * pVU  = (SmVertexuse*)m_pList;
  SM_ASSERT(pVU != NULL);
  if (pVU == NULL)
      { return FALSE; }
  SmBoolean    bRet = (pVU->m_tVertexuseType == SmShell_TYPE) ;
  return bRet;

} // end SmVertex::IsShellVertex

/*******************************************************************//**
PURPOSE: Is this vertex a stand-alone vertexloop - connected to
         just one face and to no edges.

NOTES: Only returns TRUE for a Vertex connnected to just one Face
       with no other Face or Edge connections.

  As a consequence, returns FALSE for a vertex which is both
    a Face LoopVertex AND connected to an edge,
    (ex: a wire attached to the middle of a face)
    or for a vertex which is a loop vertex in two different Faces.
    (ex: two spheres tangent to one another)

  To see if this vertex is serving as a LoopVertex for any face,
    call HasLoopVertex().
***********************************************************************/
SmBoolean SmVertex::IsLoopVertex() const
{
  SmVertexuse *pVU = (SmVertexuse*)m_pList;

  // check for common problems
  SM_ASSERT(pVU != NULL);
#ifdef DEBUG_CODE
  SM_ASSERT_BREAK_MSG((pVU->m_tVertexuseType != SmLoopuse_TYPE) || GetSize() > 1,
                      _T("Found a problem LoopVertex with just one Vertexuse connection to a face - should have two, one for each face side")) ;
#endif

  SmBoolean bRet =    GetSize() == 2     // GWC: this should 2
                  && (pVU->m_tVertexuseType == SmLoopuse_TYPE) ;
  return bRet;

} // end SmVertex::IsLoopVertex()

/*******************************************************************//**
PURPOSE: This vertex connects to at least one face as a LoopVertex

NOTES: This Vertex may also be connected to other faces and regions
         through any combination of WireEdges, LoopEdges, and LoopVertices.
***********************************************************************/
SmBoolean SmVertex::HasWireEdge() const
{
  SmVertexuse *pVU = (SmVertexuse*)m_pList;
  SM_ASSERT(pVU != NULL);
  if (pVU == NULL) 
    { return FALSE; }

  // check the first Vertexuse for a WireEdge
  SmBoolean bRtn = (   pVU->m_tVertexuseType == SmEdgeuse_TYPE
                    && pVU->GetEdgeuse()
                    && pVU->GetEdgeuse()->GetEdgeuseType() == SmShell_TYPE) ;
  pVU = ((SmVertexuse*)pVU->m_pNext) ;

  // Look at all vertexuses for a WireEdge
  while(pVU != NULL && pVU != (SmVertexuse*)m_pList && !bRtn)
    {
      bRtn |= (   pVU->m_tVertexuseType == SmEdgeuse_TYPE
               && pVU->GetEdgeuse()
               && pVU->GetEdgeuse()->GetEdgeuseType() == SmShell_TYPE) ;
      pVU = (SmVertexuse *)pVU->GetNext() ;
    }

  // all done
  return bRtn;

} // end SmVertex::HasWireEdge()

/*******************************************************************//**
PURPOSE: This vertex connects to at least one face as a LoopVertex

NOTES: This Vertex may also be connected to other faces and regions
         through any combination of WireEdges, LoopEdges, and LoopVertices.
***********************************************************************/
SmBoolean SmVertex::HasLoopEdge() const
{
  SmVertexuse *pVU = (SmVertexuse*)m_pList;
  SM_ASSERT(pVU != NULL);
  if (pVU == NULL)
    { return FALSE; }

  // check the first Vertexuse
  SmBoolean bRtn = (   pVU->m_tVertexuseType == SmEdgeuse_TYPE
                    && pVU->GetEdgeuse()
                    && pVU->GetEdgeuse()->GetEdgeuseType() == SmLoopuse_TYPE) ;
  pVU = ((SmVertexuse*)pVU->m_pNext) ;

  // Look at all vertexuses for a VertexLoop
  while(pVU != NULL && pVU != (SmVertexuse*)m_pList && !bRtn)
    {
      bRtn |= (   pVU->m_tVertexuseType == SmEdgeuse_TYPE
               && pVU->GetEdgeuse()
               && pVU->GetEdgeuse()->GetEdgeuseType() == SmLoopuse_TYPE) ;
      pVU = (SmVertexuse *)pVU->GetNext() ;
    }

  // all done
  return bRtn;

} // end SmVertex::HasLoopEdge()

/*******************************************************************//**
PURPOSE: This vertex connects to at least one face as a LoopVertex

NOTES: This Vertex may also be connected to other faces and regions
         through any combination of WireEdges, LoopEdges, and LoopVertices.
***********************************************************************/
SmBoolean SmVertex::HasLoopVertex() const
{
  SmVertexuse *pVU = (SmVertexuse*)m_pList;
  SM_ASSERT(pVU != NULL);
  if (pVU == NULL)
    { return FALSE; }

  // check the first Vertexuse
  SmBoolean bRtn = pVU->m_tVertexuseType == SmLoopuse_TYPE ;
  pVU = ((SmVertexuse*)pVU->m_pNext) ;

  // Look at all vertexuses for a VertexLoop
  while(pVU != NULL && pVU != (SmVertexuse*)m_pList && !bRtn)
    {
      bRtn |= pVU->m_tVertexuseType == SmLoopuse_TYPE ;
      pVU = (SmVertexuse *)pVU->GetNext() ;
    }

  // all done
  return bRtn;

} // end SmVertex::HasLoopVertex()

/*******************************************************************//**
PURPOSE: This vertex to exactly one Region as a ShellVertex

NOTES: This Vertex may also be connected to other faces and regions
         through any combination of WireEdges, LoopEdges, and LoopVertices.
***********************************************************************/
SmBoolean SmVertex::HasShellVertex() const
{
  return(IsShellVertex()) ;

} // end SmVertex::HasShellVertex()

/*******************************************************************//**
PURPOSE: Is this vertex connected to an edge

NOTES: returns TRUE for a Vertex connnected to any edges.

  As a consequence, returns TRUE for a vertex which is both
    a Face LoopVertex AND connected to an edge.
    (ex: a wire attached to the middle of a face)
***********************************************************************/
SmBoolean SmVertex::IsEdgeVertex() const
{
  // vertexuses
  SmVertexuse *pArrayData[20];
  SmTArray<SmVertexuse*> sVertexuses(20,pArrayData);
  GetVertexuses(sVertexuses);

  // for every vertexuse
  for (ULONG i=0; i<sVertexuses.GetSize(); i++)
    {
      SmVertexuse *pVU = sVertexuses[i];

      if(pVU->m_tVertexuseType == SmEdgeuse_TYPE)
        return TRUE ;
    }

  return FALSE ;

} // end SmVertex::IsEdgeVertex()

/*******************************************************************//**
PURPOSE: Set the 3-D point of a vertex.

NOTES:
***********************************************************************/
void SmVertex::SetPoint(const SmPoint3d & crPoint)
{
  Notify(SM_NO_CHANGE_GEOMETRY, NULL, SM_NO_GET_BREP(this), NULL) ;
  m_vPoint = crPoint;

} // end SmVertex::SetPoint

/*******************************************************************//**
PURPOSE: Get the primary (first) vertexuses of a vertex.

NOTES:
***********************************************************************/
SmVertexuse * SmVertex::GetPrimaryVertexuse() const
{
  // GWC: Modified Assert for new Notify()
  SM_ASSERT(m_pList != NULL || m_cpContext == NULL || m_cpContext->GetDoingBoolean() );

  return (SmVertexuse*)m_pList;

} // end SmVertex::GetPrimaryVertexuse

/*******************************************************************//**
PURPOSE: Get the brep of a vertex.

NOTES:
***********************************************************************/
SmBrep * SmVertex::GetBrep() const
{
  SmBrep      *pRet = NULL;
  SmVertexuse *pVU  = GetPrimaryVertexuse();
  if (pVU == NULL) { // GWC: Modified Assert for new Notify()
                     if(m_cpContext != NULL && !m_cpContext->GetDoingBoolean())
                       { SE(SM_ERR); }
                   }
  else             { SmShell *pS = pVU->GetShell();
                     if (pS == NULL) { SE(SM_ERR);
                                     }
                     else            { pRet = pS->GetBrep();
                                     }
                   }
  return pRet;

} // end SmVertex::GetBrep

/*******************************************************************//**
PURPOSE: Get all of the Vertexuses in a Vertex.

NOTES:
***********************************************************************/
void SmVertex::GetVertexuses
 (SmTArray<SmVertexuse*> & rVertexuses,     // out:
  ULONG                  * pOptAttributeId) // in : only include objects containing an attribute with this id
 const                                      //      NULL to ignore, default:[NULL]
{ 
  GetAll(SM_REINTERPRET_CAST(SmTArray<class SmTopology*>&, rVertexuses), pOptAttributeId) ; 
  
} // end SmVertex::GetVertexuses

/*******************************************************************//**
PURPOSE:
NOTES:
***********************************************************************/
SmBoolean SmVertex::IsKindOf( SM_TYPE t ) const
{
  return ((SmVertex_TYPE == t) ? TRUE : SmOwningTopology::IsKindOf( (t) ));
}

/*******************************************************************//**
PURPOSE: Dump preceded by a one line message

NOTES:
***********************************************************************/
void SmVertex::Dump
  (const TCHAR * message)
 const
{
   TCHAR sBuff[SM_TBLOCK_SIZE];
   smos_sprintf(sBuff,_T("\n%s "), message);
   smos_WriteBuffer(sBuff);
   this->Dump();

} // end SmVertex::Dump

/*******************************************************************//**
PURPOSE:  Dump preceded by a ULONG

NOTES:
***********************************************************************/
void SmVertex::Dump
  (ULONG i)
 const
{
   TCHAR sBuff[SM_TBLOCK_SIZE];
   smos_sprintf(sBuff,_T("\n%ld "), i);
   smos_WriteBuffer(sBuff);
   this->Dump();

} // end SmVertex::Dump

/*******************************************************************//**
PURPOSE:  Dump

NOTES:
***********************************************************************/
void SmVertex::Dump
  (void)
 const
{
   this->Dump(FALSE);

} // end SmVertex::Dump

/*******************************************************************//**
PURPOSE: Formatted Write for debug purposes

NOTES:
***********************************************************************/
void SmVertex::Dump
  (SmBoolean bAbbrev)  // NotUsed: in : TRUE = Vertex ptr, Num Edges, Num Faces, point
                       //               FALSE= pluse Vertexuse, connectedEdge and connectedFace list reports
 const
{
  SM_REF1(bAbbrev) ;
    // TCHAR sBuff[SM_TBLOCK_SIZE], sBuffForFile[SM_TBLOCK_SIZE];
    // TCHAR sTmpBuff[SM_TBLOCK_SIZE] = {};

  ULONG ii ;
  SmTArray<SmEdge*>      sEdges ;
  SmTArray<SmFace*>      sFaces ;
  SmTArray<SmVertexuse*> sVertexuses ;

  GetEdges(sEdges) ;
  GetFaces(sFaces) ;
  GetVertexuses(sVertexuses) ;

  // ULONG lNumEdges    = sEdges.GetSize();
  // ULONG lNumFaces    = sFaces.GetSize();
  ULONG lClosedEdges = 0 ;
  // ULONG lVtxNum      = GetVertexNumberInBrep();

  // count closed edges
  for(ii=0;ii<sEdges.GetSize();ii++)
    {
      if(sEdges[ii]->IsClosed())
        { lClosedEdges++ ; }
    }

  // get max gap values
  double dMaxVertexEdgeGap = 0.0, dMaxVertexFaceGap = 0.0 ;
  for(ii=0;ii<sVertexuses.GetSize();ii++)
    {
      SmVertexuse *pVertexuse = sVertexuses[ii] ;

      SmVertexEdgeGap * pVertexEdgeGap = pVertexuse->GetVertexEdgeGap( FALSE ) ;
      SmVertexFaceGap * pVertexFaceGap = pVertexuse->GetVertexFaceGap( FALSE ) ;

      if(pVertexEdgeGap && pVertexEdgeGap->GetLength() > dMaxVertexEdgeGap) 
        { dMaxVertexEdgeGap = pVertexEdgeGap->GetLength() ; }

      if(pVertexFaceGap && pVertexFaceGap->GetLength() > dMaxVertexFaceGap) 
        { dMaxVertexFaceGap = pVertexFaceGap->GetLength() ; }
    }
    /*
  // output abbreviated report
  if (bAbbrev)
    {
    
      if(lClosedEdges > 0) { smos_sprintf(sTmpBuff, _T(":(%ld closed)"), lClosedEdges) ; }

      smos_sprintf(sBuff,        _T("SmVertex 0x%p BrepIndx:[%3d] Edges:[%ld%s], Faces:[%ld]"),
                 this,
                 (int)lVtxNum,
                 lNumEdges,
                 sTmpBuff,
                 lNumFaces) ;
      smos_sprintf(sBuffForFile, _T("SmVertex %s BrepIndx:[%3d] Edges:[%ld%s], Faces:[%ld]"),
                 _T("notNULL"),
                 (int)lVtxNum,
                 lNumEdges,
                 sTmpBuff,
                 lNumFaces) ;
      smos_WriteBuffer(sBuff, sBuffForFile);

#ifdef SM_USE_NEWTOL
      SM_NEWTOL_LINE smos_sprintf(sBuff,  _T(" ZoneTol3d:[%5.7lf], Gaps: MaxGap[%5.7lf] MaxEdge[%5.7lf] MaxFace[%5.7lf], %s XYZ="),
      SM_NEWTOL_LINE            SmTol::GetZoneTol3d(this),
      SM_NEWTOL_LINE            GetMaxGap3d(),
      SM_NEWTOL_LINE            dMaxVertexEdgeGap, dMaxVertexFaceGap,
#else // SM_USE_OLDTOL
      SM_OLDTOL_LINE smos_sprintf(sBuff,  _T(" dTol=%5.7lf, Gaps: MaxEdge[%5.7lf] MaxFace[%5.7lf]\n    %s XYZ="),
      SM_OLDTOL_LINE            (double)GetTolerance(),
      SM_OLDTOL_LINE            dMaxVertexEdgeGap, dMaxVertexFaceGap,
#endif // SM_USE_OLDTOL

#ifdef SM_TOLERANT_WATCH
                 IsExact() ? _T("Exact") : _T("Tolerant")
#else // no SM_TOLERANT_WATCH
                 _T("")
#endif // NO SM_TOLERANT_WATCH
                 );
      smos_WriteBuffer(sBuff);

      // output xyz location
      m_vPoint.Dump(bAbbrev);

      // all done
      smos_WriteBuffer(_T("\n"));
     
    }
  else // not abbreviated  (same output with longer values plus an edge list and a vertexuse list)
    {
      if(lClosedEdges > 0) { smos_sprintf(sTmpBuff, _T(":[%ld closed]"), lClosedEdges) ; }

      smos_sprintf(sBuff,        _T("SmVertex 0x%p BrepIndx:[%3d] Edges:[%ld%s], Faces:[%ld]"),
                 this,
                 (int)lVtxNum,
                 lNumEdges,
                 sTmpBuff,
                 lNumFaces) ;
      smos_sprintf(sBuffForFile, _T("SmVertex %s BrepIndx:[%3d] Edges:[%ld%s], Faces:[%ld]"),
                 _T("notNULL"),
                 (int)lVtxNum,
                 lNumEdges,
                 sTmpBuff,
                 lNumFaces) ;
      smos_WriteBuffer(sBuff, sBuffForFile);

      smos_sprintf(sBuff,  _T(" dTol=%5.16lf, Gaps: MaxEdge:[%5.16lf] MaxFace:[%5.16lf]\n    %s XYZ="),
                 (double) GetTolerance(),
                 dMaxVertexEdgeGap, dMaxVertexFaceGap,
#ifdef SM_TOLERANT_WATCH
                 IsExact() ? _T("Exact") : _T("Tolerant")
#else // no SM_TOLERANT_WATCH
                 _T("")
#endif // no SM_TOLERANT_WATCH
                 );
      smos_WriteBuffer(sBuff);

      

      // output xyz location
      m_vPoint.Dump(bAbbrev);

      // Vertexuse report
      ULONG i, lNumVertexuses = sVertexuses.GetSize() ;

      if(lNumVertexuses == 0) { smos_WriteBuffer(_T("\n  No Vertexuses connected to this Vertex\n")); }
      else                    { smos_WriteBuffer(_T("\n  Vertexuses connected to this Vertex\n")); }

      for(i=0;i<lNumVertexuses;i++)
        {
          SmVertexuse *pVertexuse = sVertexuses[i] ;

          smos_sprintf( sBuff, _T( "   [%3lu] 0x%p:%s%p]"),
                      i, pVertexuse,
                        pVertexuse->IsShellVertexuse() ? _T("[VertexShell] to Shell:[")
                      : pVertexuse->IsLoopVertexuse()  ? _T("[VertexLoop ] to Face :[")
                      :                                  _T("[VertexEdge ] to Edge :["),
                        pVertexuse->IsShellVertexuse() ? (SmTopology*) pVertexuse->GetShell()
                      : pVertexuse->IsLoopVertexuse()  ? (SmTopology*) pVertexuse->GetFaceuse()->GetFace()
                      :                                  (SmTopology*) pVertexuse->GetEdgeuse()->GetEdge()) ;
          smos_sprintf( sBuffForFile, _T( "   [%3lu] 0x%s:%s%s]"),
                      i, pVertexuse?_T("NotNULL"):_T("NULL"),
                        pVertexuse->IsShellVertexuse() ? _T("[VertexShell] to Shell:[")
                      : pVertexuse->IsLoopVertexuse()  ? _T("[VertexLoop ] to Face :[")
                      :                                  _T("[VertexEdge ] to Edge :["),
                        pVertexuse->IsShellVertexuse() ? (pVertexuse->GetShell()?_T("NotNULL"):_T("NULL"))
                      : pVertexuse->IsLoopVertexuse()  ? (pVertexuse->GetFaceuse()->GetFace()?_T("NotNULL"):_T("NULL"))
                      :                                  (pVertexuse->GetEdgeuse()->GetEdge()?_T("NotNULL"):_T("NULL"))) ;
          smos_WriteBuffer( sBuff, sBuffForFile );

          if(pVertexuse->IsEdgeVertexuse())
            {
              SmFaceuse *pFU = pVertexuse->GetFaceuse();
              smos_sprintf( sBuff, _T( " to Face :[%p]"), (pFU ? pFU->GetFace() : NULL) );
              smos_WriteBuffer( sBuff, sBuffForFile );
            }

          smos_WriteBuffer( _T("\n")) ;

        } // end iter every Vertexuse

      // connected edge report
      if(lNumEdges == 0) { smos_WriteBuffer(_T("  No Edges connected to this Vertex\n")); }
      else               { smos_WriteBuffer(_T("  Edges connected to this Vertex\n")); }

      ULONG lEdgeNum;
      for ( i = 0; i < lNumEdges; i++ )
        {
          lEdgeNum = sEdges[i]->GetEdgeNumberInBrep();
          smos_sprintf( sBuff, _T( "   [%3lu] 0x%p, BrepIndx:[%3d] \n"), i, sEdges[i], (int)lEdgeNum );
          smos_sprintf( sBuffForFile, _T(  "   [%3lu] BrepIndx:[%3d] \n"), i, (int)lEdgeNum );
          smos_WriteBuffer( sBuff, sBuffForFile );
        }

      // connected Face report
      if(lNumFaces == 0) { smos_WriteBuffer(_T("  No Faces connected to this Vertex\n")); }
      else               { smos_WriteBuffer(_T("  Faces connected to this Vertex\n")); }

      ULONG lFaceNum;
      for ( i = 0; i < lNumFaces; i++ )
        {
          lFaceNum = sFaces[i]->GetFaceNumberInBrep();
          smos_sprintf( sBuff, _T( "   [%3lu] 0x%p, BrepIndx:[%3d] \n"), i, sFaces[i], (int)lFaceNum );
          smos_sprintf( sBuffForFile, _T(  "   [%3lu] BrepIndx:[%3d] \n"), i, (int)lFaceNum );
          smos_WriteBuffer( sBuff, sBuffForFile );
        }
  } // end not abbreviated branch
  */
} // end SmVertex::Dump

/*******************************************************************//**
PURPOSE:  Pretty print pointer values for this Vertex showing how
             it connects to its neighbor Shell, Loop, or Edge
             as appropriate in the topology graph.

NOTES: Only good for debugging because pointer values don't
                stay constant from run to run.
***********************************************************************/
void SmVertex::DumpTopology
  (ULONG lWalkDepth) // NotUsed: in : not used - vertices are at the bottom of the topology graph
 const
{
  SM_REF1(lWalkDepth) ;
  // locals
  ULONG         ii ;
  TCHAR         sBuff[SM_TBLOCK_SIZE], sBuffForFile[SM_TBLOCK_SIZE] = {};
  SmVertexuse  *pVertexuse = (SmVertexuse*) m_pList;

  // count the number of loopuses directly connected to this Vertex
  SmTArray<SmVertexuse*> sVertexuses ; GetVertexuses(sVertexuses) ;

  SM_ASSERT(sVertexuses.GetSize() == m_lListSize) ;

  // output Shell/Loop-Vertex connections - check back pointers
  smos_sprintf(sBuff,_T("  Vertex  [0x%p] has %ld Vertexuses\n"),
             this, m_lListSize) ;
  smos_WriteBuffer(sBuff, sBuffForFile);

  // Vertexuses
  if(m_lListSize > 0)
    {
      // output every owned Vertexuse and its Loopuse/Shell and Vertexuse connections
      for(ii=0;ii<m_lListSize;ii++,pVertexuse=(SmVertexuse *)pVertexuse->m_pNext)
        {
          // check for pointer consistency as possible
          SM_ASSERT(pVertexuse->GetNext()->GetLast() == pVertexuse) ;
          SM_ASSERT((SmVertex*)pVertexuse->GetVertex() == this) ;
          if(pVertexuse->m_tVertexuseType == SmShell_TYPE)
            {
              smos_sprintf(sBuff,_T("    Vertex[0x%p] -> Vertexuse[0x%p] -> Shell[0x%p]\n"),
                         this,
                         pVertexuse,
                         pVertexuse->GetShell()) ;
              smos_WriteBuffer(sBuff, sBuffForFile);
            }
          else if(pVertexuse->m_tVertexuseType == SmLoopuse_TYPE)
            {
              smos_sprintf(sBuff,_T("    Vertex[0x%p] -> Vertexuse[0x%p] -> Loopuse[0x%p] -> Faceuse[0x%p] -> Face[0x%p] \n"),
                         this,
                         pVertexuse,
                         pVertexuse->GetLoopuse(),
                         pVertexuse->GetLoopuse()->GetFaceuse(),
                         pVertexuse->GetLoopuse()->GetFaceuse()->GetFace()) ;
              smos_WriteBuffer(sBuff, sBuffForFile);
            }
          else
            {
              SM_ASSERT(pVertexuse->m_tVertexuseType == SmEdgeuse_TYPE) ;
              smos_sprintf(sBuff,_T("    Vertex[0x%p] -> Vertexuse[0x%p] -> Edgeuse[0x%p] -> Edge[0x%p] \n"),
                         this,
                         pVertexuse,
                         pVertexuse->GetEdgeuse(),
                         pVertexuse->GetEdgeuse()->GetEdge()) ;
              smos_WriteBuffer(sBuff, sBuffForFile);
            }
        } // end iter every Vertexuse
    } // end m_lListSize > 0 check

} // end SmVertex::DumpTopology
