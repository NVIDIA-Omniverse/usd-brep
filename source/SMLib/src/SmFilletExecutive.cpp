// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************//**
* FILE NAME --- SmFilletExec.cpp
* PURPOSE: Source code file for SmFilletExecutive object.
**********************************************************************/

#include "StdAfx.h"

#ifndef __SMFILLETEXECUTIVE_H__
#include <SmFilletExecutive.h>
#endif

#ifndef __SMFILLETSTANDARDSOLVER_H__
#include <SmFilletStandardSolver.h>
#endif

#include <SmCurveCache.h>
#include <SmRelation.h>
#include <SmStitch.h>
// Remove Composites
//  #include <SmCFace.h>
#include <SmAssertArray.h>
#include <SmAttribute.h>

//#define VALIDATE_TOPOLOGY 1

/*******************************************************************//**
    SmFilletErrorInfo Implementation routines.
***********************************************************************/

/*******************************************************************//**
PURPOSE: SmFilletErrorInfo constructor with no arguments

NOTES:
***********************************************************************/
SmFilletErrorInfo::SmFilletErrorInfo()
{
  set();

} // end SmFilletErrorInfo::SmFilletErrorInfo

/*******************************************************************//**
PURPOSE: SmFilletErrorInfo constructor passing in FilletExec

NOTES:
***********************************************************************/
SmFilletErrorInfo::SmFilletErrorInfo
 (SmFilletExecutive *pFilletExec)
{
  set();
  m_pFilExec = pFilletExec;

} // end SmFilletErrorInfo::SmFilletErrorInfo

/*******************************************************************//**
PURPOSE: Assignment operator

NOTES:
***********************************************************************/
SmFilletErrorInfo& SmFilletErrorInfo::operator=
 (const SmFilletErrorInfo & src)
{
  // Note, assignment works on every element, so we could
  // get away without this operator (at least for now),
  // but it's safest to do it explicitly.

  m_eErrorCode    = src.m_eErrorCode;
  m_eFilletStatus = src.m_eFilletStatus;
  m_eCornerType   = src.m_eCornerType;
  m_eEdgeType     = src.m_eEdgeType;

  m_pFilExec      = src.m_pFilExec;

  m_pEndFace        = src.m_pEndFace;
  m_pFilletedEdge   = src.m_pFilletedEdge;
  m_pFilletedVertex = src.m_pFilletedVertex;

  m_pFilletSurfaces = src.m_pFilletSurfaces;
  m_pFilletEdges    = src.m_pFilletEdges;

  return *this;

} // end SmFilletErrorInfo::operator=

/*******************************************************************//**
PURPOSE: Assignment operator

NOTES:
***********************************************************************/
void SmFilletErrorInfo::set()
{
  m_eErrorCode   = SM_FILERR_OK;
  m_eFilletStatus= SM_FIL_UNPROCESSED;
  m_eCornerType  = SM_FCR_UNKNOWN;
  m_eEdgeType    = SM_FE_UNKNOWN;
  m_pFilExec        = NULL;
  m_pSideFace       = NULL;
  m_pEndFace        = NULL;
  m_pFilletedEdge   = NULL;
  m_pFilletedVertex = NULL;
    
  m_pFilletEdges.ReSet();
  m_pFilletSurfaces.ReSet();
    
  smos_sprintf( m_cComment, _T("%s"), _T("<no comment>\n")   );
  smos_sprintf( m_cEntityName, _T("%s"), _T("\n") ) ;

} // end SmFilletErrorInfo::set

/*******************************************************************//**
PURPOSE: Record a generic fillet error.

NOTES:
***********************************************************************/
void SmFilletErrorInfo::NoteError
 (SmFilletErrorType eErrorCode,
  const TCHAR     * cComment)
{
  m_eErrorCode = eErrorCode;
  if ( cComment != NULL ) { smos_sprintf( m_cComment, _T("%s"), (TCHAR *)cComment ); }

} // end SmFilletErrorInfo::NoteError

/*******************************************************************//**
PURPOSE: Record a fillet error at a vertex.

NOTES:
***********************************************************************/
void SmFilletErrorInfo::NoteError
 (const SmVertex    * pBadVert,       // in :
  SmFilStatus         eVertexStatus,  // NotUsed: in :
  SmFilletErrorType   eErrorCode,     // NotUsed: in :
  const TCHAR       * cComment)       // NotUsed: in :
{
  SM_REF3(eVertexStatus, eErrorCode, cComment) ;
    SM_ASSERT( pBadVert != NULL ); if ( !pBadVert ) { return; }

    ULONG lVertNum = pBadVert->GetVertexNumberInBrep();
    SM_ASSERT( lVertNum >= 0 );

} // end SmFilletErrorInfo::NoteError

/*******************************************************************//**
PURPOSE: Record a fillet error at an edge.

NOTES:
***********************************************************************/
void SmFilletErrorInfo::NoteError
 (const SmEdge    * pBadEdge,
  SmFilStatus       eStatus,
  SmFilletErrorType eErrorCode,
  const TCHAR     * cComment)
{
  SM_ASSERT( pBadEdge != NULL ); if ( !pBadEdge ) { return; }

  m_pFilletedEdge = pBadEdge;
  m_eErrorCode = eErrorCode;
  m_eFilletStatus = eStatus;

  smos_sprintf( m_cComment, _T("%s"), cComment );

} // end SmFilletErrorInfo::NoteError

/*******************************************************************//**
PURPOSE: Record a fillet error at an edge.

NOTES:
   //cbi: this one was set up for the rTI.MergeCurveOnSurfaces() error
   //cbi  in SmFilletGeom::InsertIntersectionTopology().
***********************************************************************/
void SmFilletErrorInfo::NoteError
 (SmFilletErrorType eErrorCode,
  SmFilStatus eEdgeStatus,
  const SmEdge *pFilletedEdge,
  const SmFace *pSideFace,
  const SmFilletEdge *pFilletEdge,
  const SmSurface *pFilletSurface,
  const TCHAR * cComment)
{
  m_eErrorCode    = eErrorCode;
  m_eFilletStatus = eEdgeStatus;

  m_pFilletedEdge   = pFilletedEdge;
  m_pSideFace       = pSideFace;
  m_pFilletEdges.AddUnique( pFilletEdge );
  m_pFilletSurfaces.AddUnique( pFilletSurface );
  smos_sprintf( m_cComment, _T("%s"), cComment );

} // end SmFilletErrorInfo::NoteError

/*******************************************************************//**
PURPOSE: Record a fillet error at a vertex.

NOTES:
    This could arise from failure to intersect two fillet surfaces.
***********************************************************************/
void SmFilletErrorInfo::NoteError 
 (SmFilletErrorType  eErrorCode,
  SmFilletCornerType eCornerType,
  SmFilStatus eFilletStatus,
  TCHAR     * cCornerName,
  const SmVertex  * pFilletedVertex,
  const SmSurface * pFilletSurface1,
  const SmSurface * pFilletSurface2,
  const SmFilletVertex * pStartFV,
  const SmFilletVertex * pEndFV,
  const TCHAR * cComment)
{
  m_eErrorCode    = eErrorCode;
  m_eFilletStatus = eFilletStatus;
  m_eCornerType   = eCornerType;

  m_pFilletedVertex = pFilletedVertex;
  m_pFilletSurfaces.AddUnique( pFilletSurface1 );
  m_pFilletSurfaces.AddUnique( pFilletSurface2 );
  m_pFilletVertex1  = pStartFV;
  m_pFilletVertex2  = pEndFV;
  smos_sprintf( m_cComment,  _T("%s"), cComment );
  smos_sprintf( m_cEntityName, _T("%s"), cCornerName );

} // end SmFilletErrorInfo::NoteError

/*******************************************************************//**
PURPOSE: Record a fillet error at a fillet edge.

NOTES: Note that it's a fillet edge -- newly created, in the
    Fillet Brep -- and not a filleted edge of the original target Brep.
***********************************************************************/
void SmFilletErrorInfo::NoteError 
 (SmFilletErrorType eErrorCode,
  SmFilStatus eFilletStat,
  SmFilletEdgeType  eFEType,
  const SmFilletEdge *pFilletEdge,
  const TCHAR *cEdgeName,
  const TCHAR *cComment)
{
  m_eErrorCode    = eErrorCode;
  m_eFilletStatus = eFilletStat;
  m_eEdgeType     = eFEType;

  m_pFilletEdges.AddUnique( pFilletEdge );

  smos_sprintf( m_cComment, _T("%s"), cComment );
  smos_sprintf( m_cEntityName, _T("%s"), cEdgeName );

} // end SmFilletErrorInfo::NoteError

/*******************************************************************//**
PURPOSE: Record a fillet error along an edge.

NOTES:
    This could arise from failure to intersect base surfaces
***********************************************************************/
void SmFilletErrorInfo::NoteError 
 (SmFilletErrorType             eErrorCode,
  SmFilStatus                   eFilletStatus,
  const SmEdge                * pFilletedEdge,
  // gwc: I think pSideSurface no longer has to be a BBSplineSurface - replace 1 line
  // rm : SmTArray<SmBSplineSurface*> & rapFilSurfs,
  SmTArray<SM_FILLETSURF_TYPE*> & rapFilSurfs,
  SmTArray<SmBSplineCurve*>   & rapFilCurvs,
  SmTArray<SmFilletEdge*>     & rapFilEdges,
  SmPoint3d                   & rTsectPos,
  const TCHAR                 * cComment)
{
  m_eErrorCode    = eErrorCode;
  m_eFilletStatus = eFilletStatus;
  m_eEdgeType     = SM_FE_UNKNOWN;  // (it's not a fillet edge)

  m_pFilletedEdge = pFilletedEdge;
  ULONG i;
  for(i=0;i<rapFilSurfs.GetSize();i++)
    {  m_pFilletSurfaces.AddUnique( rapFilSurfs[i] ); }

  for(i=0;i<rapFilCurvs.GetSize();i++)
    { m_pFilletCurves.AddUnique( rapFilCurvs[i] ); }

  for(i=0;i<rapFilEdges.GetSize();i++)
    { m_pFilletEdges.AddUnique( rapFilEdges[i] ); }

  m_vLocation = rTsectPos;
  smos_sprintf( m_cComment, _T("%s"), cComment );

} // end SmFilletErrorInfo::NoteError

/*******************************************************************//**
PURPOSE: Print out the errors recorded in this object.

NOTES:
***********************************************************************/
void SmFilletErrorInfo::Report()
{
  if ( m_eErrorCode == SM_FILERR_OK )
      return;

  TCHAR sBuff[SM_TBLOCK_SIZE], sBuffForFile[SM_TBLOCK_SIZE];;
  ULONG i;

  smos_sprintf( sBuff, _T("%s"),_T("\nFillet Error Report\n" ) );
  smos_WriteBuffer(sBuff);

  smos_sprintf( sBuff, _T("  Error Number %d, Fillet Status %d\n" ),
      m_eErrorCode, m_eFilletStatus );
  smos_WriteBuffer(sBuff);

  smos_sprintf( sBuff, _T("%s"), _T("  Comment: " ) );          smos_WriteBuffer(sBuff);
  smos_sprintf( sBuff, _T("%.512s\n"), m_cComment);      smos_WriteBuffer(sBuff);
  smos_sprintf( sBuff, _T("%s"), _T("\n") );      smos_WriteBuffer(sBuff);

  //cbi char name[256];
  //cbi: 'name' is a good thing, could we work with
  //cbi  SmFilletCorner instead of SmVertex?

  if ( m_pFilletedVertex != NULL ) {
      ULONG lVtxNum = m_pFilletedVertex->GetVertexNumberInBrep();
      SmPoint3d vPt = m_pFilletedVertex->GetPoint();
      smos_sprintf( sBuff, _T("Original Vertex 0x%p: Number %lu in Brep\n" ),
                  m_pFilletedVertex, lVtxNum ); 
      smos_sprintf( sBuffForFile, _T("Original Vertex: Number %lu in Brep\n" ),
                  lVtxNum ); 
      smos_WriteBuffer(sBuff, sBuffForFile);
      smos_sprintf( sBuff, _T("  Location ( %f %f %f )\n" ),
          vPt.x, vPt.y, vPt.z ); smos_WriteBuffer(sBuff);
  }

  if ( m_pFilletedEdge != NULL ) {
      ULONG lEdgNum = m_pFilletedEdge->GetEdgeNumberInBrep();
      SmVertex *pVtx = m_pFilletedEdge->GetStartVertex();
      SmPoint3d vPt1 = pVtx->GetPoint();
      pVtx = m_pFilletedEdge->GetOtherVertex( pVtx );
      SmPoint3d vPt2 = pVtx->GetPoint();
      smos_sprintf( sBuff,        _T("Original Edge 0x%p: Number %lu in Brep\n" ),
                  m_pFilletedEdge, lEdgNum ); 
      smos_sprintf( sBuffForFile, _T("Original Edge: Number %lu in Brep\n" ),
                  lEdgNum ); 
      smos_WriteBuffer(sBuff, sBuffForFile);
      smos_sprintf( sBuff, _T("  End points ( %f %f %f ) ( %f %f %f )\n" ),
          vPt1.x, vPt1.y, vPt1.z, vPt2.x, vPt2.y, vPt2.z );
      smos_WriteBuffer(sBuff);
  }

  if ( m_pFilletVertex1 != NULL )
  {
      SmFilletVertexType eType = m_pFilletVertex1->GetFilletVertexType();
      SmFilStatus        eStat = m_pFilletVertex1->GetStatus();
      SmPoint3d vPt = m_pFilletVertex1->GetPoint();
      smos_sprintf( sBuff, _T("Fillet Vertex 0x%p: Type %d Status %d\n" ),
                  m_pFilletedVertex, eType, eStat ); 
      smos_sprintf( sBuffForFile, _T("Fillet Vertex: Type %d Status %d\n" ),
                  eType, eStat ); 
      smos_WriteBuffer(sBuff, sBuffForFile);
      smos_sprintf( sBuff, _T("  Location ( %f %f %f )\n" ),
          vPt.x, vPt.y, vPt.z );
      smos_WriteBuffer(sBuff);
  }
  if ( m_pFilletVertex2 != NULL )
  {
      SmFilletVertexType eType = m_pFilletVertex2->GetFilletVertexType();
      SmFilStatus        eStat = m_pFilletVertex2->GetStatus();
      SmPoint3d vPt = m_pFilletVertex2->GetPoint();
      smos_sprintf( sBuff, _T("Fillet Vertex 0x%p: Type %d Status %d\n" ),
                  m_pFilletedVertex, eType, eStat ); 
      smos_sprintf( sBuffForFile, _T("Fillet Vertex: Type %d Status %d\n" ),
                  eType, eStat ); 
      smos_WriteBuffer(sBuff, sBuffForFile);
      smos_sprintf( sBuff, _T("  Location ( %f %f %f )\n" ),
          vPt.x, vPt.y, vPt.z );
      smos_WriteBuffer(sBuff);
  }

  for ( i = 0; i < m_pFilletEdges.GetSize(); i++ ) {
      smos_sprintf( sBuff, _T("Edge %lu of fillet surface:\n"), i );
      smos_WriteBuffer(sBuff);
      SmVertex *pVtx = m_pFilletEdges[i]->GetStartVertex();
      SmPoint3d vPt1 = pVtx->GetPoint();
      pVtx = m_pFilletEdges[i]->GetOtherVertex( pVtx );
      SmPoint3d vPt2 = pVtx->GetPoint();
      smos_sprintf( sBuff, _T("  End points ( %f %f %f ) ( %f %f %f )\n" ),
          vPt1.x, vPt1.y, vPt1.z, vPt2.x, vPt2.y, vPt2.z );
      smos_WriteBuffer(sBuff);
  }

  if ( m_vLocation.IsInitialized() )
  {
      smos_sprintf( sBuff, _T("Location of error: ( %f %f %f )\n"),
          m_vLocation.x, m_vLocation.y, m_vLocation.z );
      smos_WriteBuffer(sBuff);
  }

  // Graphics:
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  if ( bDebugMe )
    {
      smgfx_Erase();
      SmBrep *pBrep = (m_pFilExec) ? m_pFilExec->GetTargetBrep() : NULL;
      if ( pBrep != NULL ) 
        {
          smgfx_SetLook( 1, 1, 0,0,0 );  // black
          pBrep->Draw(TRUE); sm_GraphicsLoop();
        }

      pBrep = (m_pFilExec) ? m_pFilExec->GetFilletBrep() : NULL;
      if ( pBrep != NULL ) 
        {
          smgfx_SetLook( 2, 1, 0,1,1 );  // cyan
          pBrep->Draw(TRUE); sm_GraphicsLoop();
        }

      if ( m_pSideFace != NULL )
        {
          smgfx_SetLook( 2, 1, 0,0,1 );  // blue
          m_pSideFace->Draw(); sm_GraphicsLoop();
        }

      if ( m_pEndFace != NULL ) 
        {
          smgfx_SetLook( 2, 1, 0,0,1 );  // blue
          m_pEndFace->Draw(); sm_GraphicsLoop();
        }

      for ( i = 0; i < m_pFilletSurfaces.GetSize(); i++ )
        {
          smgfx_SetLook( 2, 1, 1,1,0 );  // yellow for the first one
          m_pFilletSurfaces[i]->DrawUV(4,4,TRUE); sm_GraphicsLoop();
          smgfx_SetLook( 2, 1, 0.6,0.6,0 );  // light yellow for others
        }   

      if ( m_pFilletedEdge != NULL ) 
        {
          smgfx_SetLook( 6, 1, 1,0,0 );  // red
          m_pFilletedEdge->Draw(); sm_GraphicsLoop();
        }

      for ( i = 0; i < m_pFilletEdges.GetSize(); i++ )
        {
          smgfx_SetLook( 6, 1, 1,0,1 );  // magenta for the first one
          m_pFilletEdges[i]->Draw(); sm_GraphicsLoop();
          smgfx_SetLook( 6, 1, 0.5,0,0.5 );  // light magenta for others
        }

      for ( i = 0; i < m_pFilletCurves.GetSize(); i++ )
        {
          smgfx_SetLook( 6, 1, 0,0,1 );  // blue for the first one
          m_pFilletCurves[i]->Draw(); sm_GraphicsLoop();
          smgfx_SetLook( 6, 1, 0,0,0.5 );  // light blue for others
        }

      if ( m_pFilletedVertex != NULL ) 
        {
          smgfx_SetLook( 1, 7, 0,1,0 );  // green
          m_pFilletedVertex->Draw(); sm_GraphicsLoop();
        }

      if ( m_pFilletVertex1 != NULL )
        {
          smgfx_SetLook( 1, 7, 1,0,0 );  // blue
          m_pFilletVertex1->Draw(); sm_GraphicsLoop();
        }

      if ( m_pFilletVertex2 != NULL ) 
        {
          smgfx_SetLook( 1, 7, 1,0,0 );  // blue
          m_pFilletVertex2->Draw(); sm_GraphicsLoop();
        }

      if ( m_vLocation.IsInitialized() )
        {
          smgfx_SetLook( 1, 8, 0,0,0 );  // black
          m_vLocation.Draw(); sm_GraphicsLoop();
        }

      sm_GraphicsLoop();
  }
#endif // SM_DEBUG_CODE

} // end SmFilletErrorInfo::Report()

// End of SmFilletErrorInfo implementation.

/*******************************************************************//**
PURPOSE: Constructor for the SmFilletExecutive object.

NOTES:
***********************************************************************/
SmFilletExecutive::SmFilletExecutive
 ( const SmContext & crContext,   // in : context for new object construction
   SmBrep          * pTargetBrep) // in, optional : Brep to be filleted
 : SmMerge(crContext),
   m_crContext(crContext),
   m_pTargetBrep(pTargetBrep),
   m_pFilletBrep(NULL),
   m_bDoPiecewiseMerge(FALSE),
   m_bDoTopologyInsertion(TRUE),
   m_bDoGlobalMerge(FALSE),
   m_bDoTrimming(TRUE),
   m_bDoClassification(TRUE),
   m_pAttributeE(NULL)
{
  m_pPseudoBrep = new (crContext) SmFilletBrep();
  m_pPseudoBrep->m_bEditingEnabled = TRUE;
  m_vFilletSolvers.ReSet();
  m_vFilletCorners.ReSet();
  m_vTopoEdges.ReSet();
  m_vFSGs.ReSet();

  m_pSelfIntersectionHandler = NULL;
  m_vTI.m_bIgnoreTangency = TRUE;
  m_pAttributeE = new (crContext)SmAttribute(SM_AI_ALL_FILLET_EDGES,SM_AB_STANDALONE_REFERENCE);

  m_sErrorInfo.m_pFilExec = this;

  m_iDebugLevel = 0;

} // end SmFilletExecutive::SmFilletExecutive constructor

/*******************************************************************//**
PURPOSE: Destructor for the SmFilletExecutive object.

NOTES:
***********************************************************************/
SmFilletExecutive::~SmFilletExecutive()
{
  ULONG i;
  if (m_pPseudoBrep) { delete m_pPseudoBrep; m_pPseudoBrep = NULL ; }

  // delete extended surfaces
  SmTArray<SmSurface*> sExtSurfaces;
  m_vExtendedSurfacesMap.GetAllValues(sExtSurfaces);
  for (ULONG ii=0; ii < sExtSurfaces.GetSize(); ++ii)
    {
      SmSurface * pSurf = sExtSurfaces[ii];
      if (pSurf) { delete pSurf; pSurf = NULL ; }
    }

  // delete filletCorners
  for (i=0; i<m_vFilletCorners.GetSize(); i++)
    {
      SM_ASSERT(m_vFilletCorners[i] != NULL) ; delete m_vFilletCorners[i] ; m_vFilletCorners[i] = NULL ;
    }

  // delete SmFilletSolvers
  for (i=0; i<m_vFilletSolvers.GetSize(); i++)
    {
      SM_ASSERT(m_vFilletSolvers[i] != NULL) ; delete m_vFilletSolvers[i] ; m_vFilletSolvers[i] = NULL ;
    }

  // delete SmFilletSurfaceGenerators if present.
  for (i=0; i<m_vFSGs.GetSize(); i++)
    {
      SM_ASSERT(m_vFSGs[i] != NULL) ; delete m_vFSGs[i] ; m_vFSGs[i] = NULL ;
    }

  // delete the filletAttribute
  SmTArray<SmAObject*> sObjs;
  if (m_pAttributeE)
    {
      m_pAttributeE->GetUsers(sObjs);
      for (ULONG m=0; m<sObjs.GetSize(); m++)
        {
          //m_vAttributeE.RemoveUser(sObjs[m],TRUE);
          sObjs[m]->RemoveAttribute(m_pAttributeE);
        }
      SM_ASSERT(m_pAttributeE != NULL) ; delete m_pAttributeE ; m_pAttributeE = NULL ;
    }

} // end SmFilletExecutive::~SmFilletExecutive

/*******************************************************************//**
PURPOSE: High-level call to set up filleting parameters,
    using edge numbers, with the same parameters on each edge.

NOTES: Some version of this must be called prior to calling DoFillet().
***********************************************************************/
SmStatus SmFilletExecutive::SetFilletParameters // same type fillet on all edges.
 (SmBrep                     * pBrep,
  SmTArray< ULONG >          & rEdgeNums,
  SmFilletSurfaceGeneratorType eXSectType,
  SmFilletSolverType           eRadiusType,
  double                       dRadius,
  SmFilletLaw                * pOptVarRadLaw,
  double                       dApproxTol,     // default (if <= 0): 100 * Brep tol.
  double                       dAngTolDeg,     // default (if <= 0): 20 degrees
  double                       dTanTolDeg,     // default (if <= 0): 10 degrees
  double                       dXSectTol,      // default (if <= 0): 0.05
  ULONG                        lContinuity,    // 1, 2, or 3 for G1, G2, or G3
                                               // Used only when eXSectType == SM_FSG_BLEND_CURVE
                                               // default:[1]
  double                       dThumbweight)   // Used only when eXSectType == SM_FSG_BLEND_CURVE
                                               // default:[1.0]
{
  m_bIsInitialized = FALSE;
  TCHAR sBuff[SM_TBLOCK_SIZE];

  // Convert Edge numbers to SmEdge* pointers.
  SmTArray< SmEdge* > sEdgePtrs;
  SmTArray< SmEdge* > sAllEdges;
  pBrep->GetEdges( sAllEdges );
  ULONG lNumAllEdges = sAllEdges.GetSize();
  ULONG lNumEdgeNums = rEdgeNums.GetSize();
  ULONG i;
  for ( i = 0; i < lNumEdgeNums; i++ )
    {
      // First check the input:
      if ( rEdgeNums[i] >= lNumAllEdges )
      {
          // If the caller expects this edge to be filleted,
          // then there is probably some fundamental problem,
          // so error out.
          smos_sprintf( sBuff, _T("Error: Invalid Input Edge Number %2lu\n"), rEdgeNums[i] );
          GetFilletErrorInfo()->NoteError( SM_FILERR_INVALID_INPUT, sBuff );
          return SM_ERR_INVALID_INPUT;
      }
      sEdgePtrs.Add( sAllEdges[ rEdgeNums[ i ] ] );
    }

  return SetFilletParameters(pBrep,
                             sEdgePtrs,
                             eXSectType,
                             eRadiusType,
                             dRadius,
                             pOptVarRadLaw,
                             dApproxTol,
                             dAngTolDeg,
                             dTanTolDeg,
                             dXSectTol,
                             lContinuity,
                             dThumbweight);

} // end SetFilletParameters ( edge numbers, same type fillet on all edges )

/*******************************************************************//**
PURPOSE: High-level call to set up filleting parameters,
    using edge numbers, with possibly different parameters on each edge.

NOTES: Some version of this must be called prior to calling DoFillet().
***********************************************************************/
SmStatus SmFilletExecutive::SetFilletParameters
 (SmBrep                                   * pBrep,
  SmTArray< ULONG >                        & rEdgeNums,
  SmTArray< SmFilletSurfaceGeneratorType > & rXSectTypes,
  SmTArray< SmFilletSolverType >           & rRadiusTypes,
  SmTArray< double >                       & rRadiusValues,
  SmTArray< SmFilletLaw *>                 * pOptVarRadLaws,
  double                                     dApproxTol,         // default (if <= 0): 100 * Brep tol.
  double                                     dAngTolDeg,         // default (if <= 0): 20 degrees
  double                                     dTanTolDeg,         // default (if <= 0): 10 degrees
  double                                     dXSectTol,          // default (if <= 0): 0.05
  ULONG                                      lContinuity,        // 1, 2, or 3 for G1, G2, or G3
                                                                 // Used only when eXSectType == SM_FSG_BLEND_CURVE
                                                                 // default:[1]
  double                                     dThumbweight)       // Used only when eXSectType == SM_FSG_BLEND_CURVE
                                                                 // default:[1.0]
{                                                               
  m_bIsInitialized = FALSE;
  TCHAR sBuff[SM_TBLOCK_SIZE];

  // Convert Edge numbers to SmEdge* pointers.
  SmTArray< SmEdge* > sEdgePtrs;
  SmTArray< SmEdge* > sAllEdges;
  pBrep->GetEdges( sAllEdges );
  ULONG lNumAllEdges = sAllEdges.GetSize();
  ULONG lNumEdgeNums = rEdgeNums.GetSize();
  ULONG i;
  for ( i = 0; i < lNumEdgeNums; i++ )
    {
      // First check the input:
      if ( rEdgeNums[i] >= lNumAllEdges )
        {
          // If the caller expects this edge to be filleted,
          // then there is probably some fundamental problem,
          // so error out.
          smos_sprintf( sBuff, _T("Error: Invalid Input Edge Number %2lu\n"), rEdgeNums[i] );
          GetFilletErrorInfo()->NoteError( SM_FILERR_INVALID_INPUT, sBuff );
          return SM_ERR_INVALID_INPUT;
        }
      sEdgePtrs.Add( sAllEdges[ rEdgeNums[ i ] ] );
    }

  return SetFilletParameters(pBrep,
                             sEdgePtrs,
                             rXSectTypes,
                             rRadiusTypes,
                             rRadiusValues,
                             pOptVarRadLaws,
                             dApproxTol,
                             dAngTolDeg,
                             dTanTolDeg,
                             dXSectTol,
                             lContinuity,
                             dThumbweight);

} // end SetFilletParameters ( edge numbers, possibly different parameters on each edge )

/*******************************************************************//**
PURPOSE: High-level call to set up filleting parameters,
    using edge pointers, with the same parameters on each edge.

NOTES: Some version of this must be called prior to calling DoFillet().
***********************************************************************/
SmStatus SmFilletExecutive::SetFilletParameters
 (SmBrep                     * pBrep,
  SmTArray< SmEdge* >        & rEdgePtrs,
  SmFilletSurfaceGeneratorType eXSectType,
  SmFilletSolverType           eRadiusType,
  double                       dRadius,
  SmFilletLaw                * pOptVarRadLaw,
  double                       dApproxTol,       // default (if <= 0):  4 * Edge tol.
  double                       dAngTolDeg,       // default (if <= 0): 20 degrees
  double                       dTanTolDeg,       // default (if <= 0): 10 degrees
  double                       dXSectTol,        // default (if <= 0): 0.05
  ULONG                        lContinuity,      // 1, 2, or 3 for G1, G2, or G3                   
                                                 // Used only when eXSectType == SM_FSG_BLEND_CURVE
                                                 // default:[1]                                    
  double                       dThumbweight)     // Used only when eXSectType == SM_FSG_BLEND_CURVE
                                                 // default:[1.0]
{
  m_bIsInitialized = FALSE;

  if ( pBrep != NULL )
    { m_pTargetBrep = pBrep; }

  TCHAR sBuff[SM_TBLOCK_SIZE];

  // First set some standard flags.
  // Set to the most common options; can be overridden after this call if desired.
  m_bDoPiecewiseMerge    = FALSE;
  m_bDoTopologyInsertion = TRUE;
  m_bDoGlobalMerge       = FALSE;
  m_bDoTrimming          = TRUE;
  m_bDoClassification    = TRUE;

  ULONG lNumFilletedEdges = rEdgePtrs.GetSize();
  if ( lNumFilletedEdges < 1 ) { return SM_SUCCESS; }

  if ( eRadiusType == SM_FS_VARIABLE_RADIUS && pOptVarRadLaw == NULL )
    {
      smos_sprintf( sBuff,_T("%s"), _T("No variable-radius law specified\n") );
      GetFilletErrorInfo()->NoteError( SM_FILERR_INVALID_INPUT, sBuff );
      return SM_ERR_INVALID_INPUT;
    }

  const SmContext * pContext = pBrep->GetContext();

  // Note, dApproxTol will presumably be reset for each edge.
  if ( dApproxTol <= SM_EFF_ZERO ) { dApproxTol =  4 * pBrep->GetTolerance(); }
  if ( dAngTolDeg <= SM_EFF_ZERO ) { dAngTolDeg = 20.0;  }
  if ( dTanTolDeg <= SM_EFF_ZERO ) { dTanTolDeg = 10.0;  }
  if ( dXSectTol  <= SM_EFF_ZERO ) { dXSectTol  =  0.05; }
  double dAngTolRad = SM_DEG2RAD( dAngTolDeg );
  double dTanTolRad = SM_DEG2RAD( dTanTolDeg );

  // Fillet Surface Generator (cross section):
  // create one and use it for all filleted edges.
  SmFilletSurfaceGenerator *pFSG = NULL;

  if ( eXSectType == SM_FSG_LINEAR )
    { pFSG = new (*pContext) SmLinearCrossSectionFSG; }
  else if ( eXSectType == SM_FSG_CIRCULAR )
    { pFSG = new (*pContext) SmCircularCrossSectionFSG( TRUE, dXSectTol ); }
  else if ( eXSectType == SM_FSG_BLEND_CURVE )
    { pFSG = new (*pContext) SmBlendCurveCrossSectionFSG( dThumbweight, lContinuity) ; }
  else
    {
      GetFilletErrorInfo()->NoteError( SM_FILERR_INVALID_INPUT,
                 _T("SetFilletParameters: Invalid Fillet Surface Generator type.") );
      return SM_ERR_INVALID_INPUT;
    }
  // Save it, so that we can delete it when filleting is finished.
  AddFilletSurfaceGenerator( pFSG );

  // Now for each Filleted Edge.
  ULONG ii;
  for ( ii = 0; ii < lNumFilletedEdges; ii++ )
    {
      // 1. Find the appropriate Edgeuse for the fillet.
      SmEdge* pEdge = rEdgePtrs[ii];
      SmEdgeuse *pEU = pEdge->GetBlendEdgeuse();

      if ( pEU == NULL )
        {
          // pEU being NULL means that this edge is not filletable.
          // If the caller expects this edge to be filleted,
          // then there is probably some fundamental problem,
          // so error out.
          smos_sprintf( sBuff, _T("Edge[ %2lu ] == %p: cannot be filleted\n"), ii, pEdge );
          GetFilletErrorInfo()->NoteError( pEdge, SM_FIL_FAILURE, SM_FILERR_INVALID_INPUT, sBuff );
          return SM_ERR_INVALID_INPUT;
        }

      // 2. Fillet Solver: switch on radius type.
      SmFilletSolver *pFS = NULL;

      // Note, this works well in practice.  [B332]
      static constexpr double dTolFactor = 4.0;
      dApproxTol = dTolFactor * pEdge->GetTolerance();

      switch ( eRadiusType )
        {
          case SM_FS_CONST_RADIUS:
            {

              pFS = new(*pContext) SmConstantRadiusFS( *pContext,
                  dApproxTol, dAngTolRad, dTanTolRad,
                  dRadius, pEU );
              break;
            }
          case SM_FS_CONST_DIST:
            {

              pFS = new(*pContext) SmConstantDistanceFS( *pContext,
                  dApproxTol, dAngTolRad, dTanTolRad,
                  dRadius, pEU );
              break;
            }
          case SM_FS_VARIABLE_RADIUS:
            {
              pFS = new(*pContext) SmVariableRadiusFS( *pContext,
                  dApproxTol, dAngTolRad, dTanTolRad,
                  dRadius, pEU, *pOptVarRadLaw, FALSE );
              break;
            }
          case SM_FS_SURF_SURF:
          case SM_FS_CURVE_BASED:
          case SM_FS_CONST_RADIUS_ASSISTED:
            {
              pFS = NULL; // Not set up for these yet.
            }

          case SM_FS_UNKNOWN:
              break;
        } // end switch

      if ( pFS == NULL )
        {
          smos_sprintf( sBuff,_T("%s"), _T("Error: invalid radius type (Fillet Solver)\n") );
          GetFilletErrorInfo()->NoteError( pEdge, SM_FIL_FAILURE, SM_FILERR_INVALID_INPUT, sBuff );
          return SM_ERR_INVALID_INPUT;
        }
      this->LoadFilletSolver( pFS );

      // 3. Fillet Surface Generator: switch on cross-section type.
      pFS->SetFilletSurfaceGenerator( pFSG );

    } // end for each filleted edge

  // Give the ok to proceed with the filleting.
  m_bIsInitialized = TRUE;

  return SM_SUCCESS;

} // end SetFilletParameters ( edge pointers, same type fillet on all edges )

/*******************************************************************//**
PURPOSE: High-level call to set up filleting parameters,
    using edge pointers, with possibly different parameters on each edge.

NOTES: Some version of this must be called prior to calling DoFillet().
   If, for any of the input arrays of parameters (sXSectTypes, sRadiusTypes,
   or rRadiusValues), all values are the same, then the array may contain only
   a single entry, of that common value.
***********************************************************************/
SmStatus SmFilletExecutive::SetFilletParameters
 (SmBrep                                   * pBrep,
  SmTArray< SmEdge* >                      & rEdgePtrs,
  SmTArray< SmFilletSurfaceGeneratorType > & rXSectTypes,
  SmTArray< SmFilletSolverType >           & rRadiusTypes,
  SmTArray< double >                       & rRadiusValues,
  SmTArray< SmFilletLaw *>                 * pOptVarRadLaws,
  double                                     dApproxTol,      // default (if <= 0):  4 * Edge tol.
  double                                     dAngTolDeg,      // default (if <= 0): 20 degrees
  double                                     dTanTolDeg,      // default (if <= 0): 10 degrees
  double                                     dXSectTol,       // default (if <= 0): 0.05
  ULONG                                      lContinuity,     // 1, 2, or 3 for G1, G2, or G3                   
                                                              // Used only when rXSectTypes[ii] == SM_FSG_BLEND_CURVE
                                                              // default:[1]                                    
  double                                     dThumbweight)    // Used only when eXSectType == SM_FSG_BLEND_CURVE
                                                              // default:[1.0]
{
  m_bIsInitialized = FALSE;

  if ( pBrep != NULL )
    { m_pTargetBrep = pBrep; }

  TCHAR sBuff[SM_TBLOCK_SIZE];

  // First set some standard flags.
  // Set to the most common options; can be overridden after this call if desired.
  m_bDoPiecewiseMerge    = FALSE;
  m_bDoTopologyInsertion = TRUE;
  m_bDoGlobalMerge       = FALSE;
  m_bDoTrimming          = TRUE;
  m_bDoClassification    = TRUE;

  ULONG lNumFilletedEdges = rEdgePtrs.GetSize();
  if ( lNumFilletedEdges < 1 ) { return SM_SUCCESS; }

  SmBoolean bSingleXSectType   =    (rXSectTypes.GetSize() == 1)                   ? TRUE
                                 :  (rXSectTypes.GetSize() == lNumFilletedEdges)   ? FALSE
                                 : UNSURE;
  SmBoolean bSingleRadiusType  =    (rRadiusTypes.GetSize() == 1)                  ? TRUE
                                 :  (rRadiusTypes.GetSize() == lNumFilletedEdges)  ? FALSE
                                 : UNSURE;
  SmBoolean bSingleRadiusValue =    (rRadiusValues.GetSize() == 1)                 ? TRUE
                                 :  (rRadiusValues.GetSize() == lNumFilletedEdges) ? FALSE
                                 : UNSURE;

  if ( bSingleXSectType == UNSURE || bSingleRadiusType == UNSURE || bSingleRadiusValue == UNSURE )
    {
      GetFilletErrorInfo()->NoteError( SM_FILERR_INVALID_INPUT,
                 _T("SetFilletParameters: Input array size mismatch.") );
      return SM_ERR_INVALID_INPUT;
    }

  const SmContext * pContext = pBrep->GetContext();

  // Note, dApproxTol will presumably be reset for each edge.
  if ( dApproxTol <= SM_EFF_ZERO ) { dApproxTol = 10 * pBrep->GetTolerance(); }
  if ( dAngTolDeg <= SM_EFF_ZERO ) { dAngTolDeg = 20.0; }
  if ( dTanTolDeg <= SM_EFF_ZERO ) { dTanTolDeg = 10.0; }
  if ( dXSectTol  <= SM_EFF_ZERO ) { dXSectTol  =  0.05; }
  double dAngTolRad = SM_DEG2RAD( dAngTolDeg );
  double dTanTolRad = SM_DEG2RAD( dTanTolDeg );

  // We will use these values throughout, if Single is requested.
  SmFilletSolverType           eRadiusType = rRadiusTypes [0];
  SmFilletSurfaceGeneratorType eXSectType  = rXSectTypes  [0];
  double                       dRadius     = rRadiusValues[0];
  SmFilletSurfaceGenerator    *pFSG        = NULL;

  if ( bSingleXSectType )
    {
      if ( eXSectType == SM_FSG_LINEAR )
        { pFSG = new (*pContext) SmLinearCrossSectionFSG; }
      else if ( eXSectType == SM_FSG_CIRCULAR )
        { pFSG = new (*pContext) SmCircularCrossSectionFSG( TRUE, dXSectTol ); }
      else if ( eXSectType == SM_FSG_BLEND_CURVE )
        { pFSG = new (*pContext) SmBlendCurveCrossSectionFSG( dThumbweight, lContinuity); }
      else
        {
          GetFilletErrorInfo()->NoteError( SM_FILERR_INVALID_INPUT,
                     _T("SetFilletParameters: Invalid Fillet Surface Generator type.") );
          return SM_ERR_INVALID_INPUT;
        }
      // Save it, so that we can delete it when filleting is finished.
      AddFilletSurfaceGenerator( pFSG );
    }

  // Now for each Filleted Edge.
  ULONG ii;
  for(ii=0;ii<lNumFilletedEdges;ii++)
    {
      // 1. Find the appropriate Edgeuse for the fillet.
      SmEdge* pEdge = rEdgePtrs[ii];
      SmEdgeuse *pEU = pEdge->GetBlendEdgeuse();

      if ( pEU == NULL )
        {
          // pEU being NULL means that this edge is not filletable.
          // If the caller expects this edge to be filleted,
          // then there is probably some fundamental problem,
          // so error out.
          smos_sprintf( sBuff, _T("Edge[ %2lu ] == %p: cannot be filleted\n"), ii, pEdge );
          GetFilletErrorInfo()->NoteError( pEdge, SM_FIL_FAILURE, SM_FILERR_INVALID_INPUT, sBuff );
          return SM_ERR_INVALID_INPUT;
        }

      // 2. Fillet Solver: switch on radius type.
      SmFilletSolver *pFS = NULL;

      if ( ! bSingleRadiusType  ) { eRadiusType = rRadiusTypes [ii]; }
      if ( ! bSingleRadiusValue ) { dRadius     = rRadiusValues[ii]; }

      // Note, this works well in practice.  [B332]
      static constexpr double dTolFactor = 4.0;
      dApproxTol = dTolFactor * pEdge->GetTolerance();

      switch ( eRadiusType )
        {
          case SM_FS_CONST_RADIUS:
            {
              pFS = new(*pContext) SmConstantRadiusFS( *pContext,
                  dApproxTol, dAngTolRad, dTanTolRad,
                  dRadius, pEU );
              break;
            }
          case SM_FS_CONST_DIST:
            {
              pFS = new(*pContext) SmConstantDistanceFS( *pContext,
                  dApproxTol, dAngTolRad, dTanTolRad,
                  dRadius, pEU );
              break;
            }
          case SM_FS_VARIABLE_RADIUS:
            {
              if ( pOptVarRadLaws->GetSize() < ii+1 || (*pOptVarRadLaws)[ii] == NULL )
                {
                  smos_sprintf( sBuff, _T("Edge[ %2lu ] == %p: no variable-radius law specified\n"), ii, pEdge );
                  GetFilletErrorInfo()->NoteError( pEdge, SM_FIL_FAILURE, SM_FILERR_INVALID_INPUT, sBuff );
                  return SM_ERR_INVALID_INPUT;
                }
              pFS = new(*pContext) SmVariableRadiusFS( *pContext,
                  dApproxTol, dAngTolRad, dTanTolRad,
                  dRadius, pEU, *((*pOptVarRadLaws)[ii]), FALSE );
              break;
            }
          case SM_FS_SURF_SURF:
          case SM_FS_CURVE_BASED:
          case SM_FS_CONST_RADIUS_ASSISTED:
            {
              pFS = NULL; // Not set up for these yet.
            }

          case SM_FS_UNKNOWN:
              break;
        } // end switch

      if ( pFS == NULL )
        {
          smos_sprintf( sBuff, _T("Edge[ %2lu ] == %p: invalid radius type (Fillet Solver)\n"), ii, pEdge );
          GetFilletErrorInfo()->NoteError( pEdge, SM_FIL_FAILURE, SM_FILERR_INVALID_INPUT, sBuff );
          return SM_ERR_INVALID_INPUT;
        }
      this->LoadFilletSolver( pFS );


      // 3. Fillet Surface Generator: switch on cross-section type.
      if ( ! bSingleXSectType )
        {
          eXSectType = rXSectTypes[ii];

          if ( eXSectType == SM_FSG_LINEAR )
            { pFSG = new (*pContext) SmLinearCrossSectionFSG; }
          else if ( eXSectType == SM_FSG_CIRCULAR )
            { pFSG = new (*pContext) SmCircularCrossSectionFSG( TRUE, 0.1 ); }
          else if ( eXSectType == SM_FSG_BLEND_CURVE )
            { pFSG = new (*pContext) SmBlendCurveCrossSectionFSG( dThumbweight, lContinuity) ; }
          else
            {
              GetFilletErrorInfo()->NoteError( SM_FILERR_INVALID_INPUT,
                         _T("SetFilletParameters: Invalid Fillet Surface Generator type.") );
              return SM_ERR_INVALID_INPUT;
            }
          // Save it, so that we can delete it when filleting is finished.
          AddFilletSurfaceGenerator( pFSG );
        }
    pFS->SetFilletSurfaceGenerator( pFSG );
    
  } // end for each filleted edge

  // Give the ok to proceed with the filleting.
  m_bIsInitialized = TRUE;

  return SM_SUCCESS;

} // end SetFilletParameters ( edge pointers, different parameters on each edge )

/*******************************************************************//**
PURPOSE: High-level call to preview filleting surfaces,

NOTES: Create all corners and fillet solvers (fillet surface, rails, and spine)
   Optionally Trim rail curves to the fillet geometry (not to the target Brep)
   Return untrimmed sillet surfaces
   In the future we can return corners as well
***********************************************************************/
SmStatus SmFilletExecutive::GetPreviewFilletSurfaces
(
  SmTArray<SmSurface*>& rFilletSurfaces       ///< [out] :        <br>
)
{
  // check state - must have a target Brep
  NER( m_pTargetBrep );

  // locals
  ULONG cornerIdx, solverIdx;
  TCHAR sBuff[SM_TBLOCK_SIZE]; // Use this for error reporting as well as debug.

  // Create FilletCorner in m_vFilletCorners for every m_vFilletedVertices vertex and 
  // add [vertex,SmFilletCorner] entry pair to m_vVertexCornerMap 
  SmStatus eStat = CreateFilletCorners();
  SER( eStat );

  // Process corners by calculating the vertices first.
  // Those vertices will determine the connectivity between
  // corner surfaces and the adjacent edge fillets.
  for(cornerIdx = 0; cornerIdx < m_vFilletCorners.GetSize(); cornerIdx++)
  {
    SmFilletCorner * pCorner = m_vFilletCorners[cornerIdx];

    if(SM_SUCCESS != pCorner->MakeCornerTopology())
    {
      // when MakeCornerTopology failed - note the error
      pCorner->GetName( sBuff, SM_TBLOCK_SIZE );
      NoteFilletError( SM_FILERR_VERTEX_PROBLEM, pCorner->GetCornerType(),
                       SM_FIL_FAILURE, sBuff, pCorner->GetFilletedVertex(),
                       NULL, NULL, NULL, NULL,
                       _T( "Fillet Corner Error: Unable to Make Topology" ) );
      FILEXEC_CORNER_ERR( pCorner, _T( "Unable to Make Topology" ) );
    }

    // Set point positions for the new filletVertices 
    if(SM_SUCCESS != pCorner->CalcCornerVertGeom())
    {
      // when MakeCornerTopology failed - note the error
      pCorner->GetName( sBuff, SM_TBLOCK_SIZE );
      NoteFilletError( SM_FILERR_VERTEX_PROBLEM, pCorner->GetCornerType(),
                       SM_FIL_FAILURE, sBuff, pCorner->GetFilletedVertex(),
                       NULL, NULL, NULL, NULL,
                       _T( "Fillet Corner Error: Unable to Compute Geometry" ) );
      FILEXEC_CORNER_ERR( pCorner, _T( "Vertex Computation Failure" ) );
    }

  } // end of loop over corners, creating topology and vertex geometry


  // We have created the topology for the corners, and then filled in
  // their geometry.  Sometimes, however, the topology can depend
  // on the geometry.  An example would be the 3x2 case, if two rails
  // intersect the un-filleted edge in different locations, a new fillet
  // edge and vertex will be inserted.
  // Another example is a degenerate corner, which is handled separately.
  // So for every corner, adjust topology as required.
  for(cornerIdx = 0; cornerIdx < m_vFilletCorners.GetSize(); cornerIdx++)
  {
    SmFilletCorner * pCorner = m_vFilletCorners[cornerIdx];

    // Check for and deal with degenerate corner.
    SmBoolean bCornerIsDegen = pCorner->CheckAndFixDegeneracy();

    if(!bCornerIsDegen)
    {
      // If this is not a degenerate corner,
      // re-adjust topology per errors or user's requests
      // such as 'Set-back' or '3x2-Bevel with non-equal radii'
      // This can also adjust things automatically, such as in the 3x2
      // case, if two rails intersect the un-filleted edge in different
      // places, a new fillet vertex and edge will be inserted.

      if(SM_SUCCESS != pCorner->AdjustTopology())
      {
        // when AdjustTopology failed - note the error
        pCorner->GetName( sBuff, SM_TBLOCK_SIZE );
        NoteFilletError( SM_FILERR_VERTEX_PROBLEM, pCorner->GetCornerType(),
                         SM_FIL_FAILURE, sBuff, pCorner->GetFilletedVertex(),
                         NULL, NULL, NULL, NULL,
                         _T( "Fillet Corner Error: Topology Adjustment Failure" ) );
        FILEXEC_CORNER_ERR( pCorner, _T( "Topology Adjustment Failure" ) );
      }
    }

  } // end of loop over corners, adjusting topology for non-degenerate cases

#ifdef SM_DEBUG_CODE
  if(m_iDebugLevel >= 10)
  {
    smgfx_Erase();
    smgfx_SetLook( 1, 2, 0, 0, 0 ); m_pTargetBrep->Draw( TRUE ); sm_GraphicsLoop();
    smgfx_SetLook( 3, 4, 0, 0, 1 ); m_pPseudoBrep->Draw( TRUE ); sm_GraphicsLoop();
    smgfx_SetLook( 5, 6, 1, 0, 0 );
    for(cornerIdx = 0; cornerIdx < m_vFilletCorners.GetSize(); cornerIdx++)
    {
      SmFilletCorner * pCorner = m_vFilletCorners[cornerIdx];
      pCorner->Draw(); sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
    sm_GraphicsLoop();
  }
#endif // SM_DEBUG_CODE

  // Calculate the filletSurface geometry for each edge fillet
  for(solverIdx = 0; solverIdx < m_vFilletSolvers.GetSize(); solverIdx++)
  {
    SmFilletSolver * pFilSolver = m_vFilletSolvers[solverIdx];

    // Compute untrimmed filletSurface and railCurve geometry
    if(SM_SUCCESS != pFilSolver->CalcFilletGeom())
    {
      const SmEdge* pE = pFilSolver->GetEdgeuse( 0 )->GetEdge();
      GetFilletErrorInfo()->NoteError( pE, SM_FS_SURF_TRACING_FAILURE, SM_FILERR_EDGE_PROBLEM,
                                       _T( "Fillet Surface Computation Failure" ) );
      FILEXEC_SOLVER_ERR( pFilSolver, _T( "Fillet Surface Computation Failure" ) );
    }

  } // end CalcFilletGeom loop for every edge being filleted

  // Fillet surfaces and rail curves exist now, not trimmed to anything.
  // For example, new fillet vertices that are supposed to lie in a face
  // of the target Brep might lie outside of the face.

#ifdef SM_DEBUG_CODE
  if(m_iDebugLevel > 5)
  {
    smgfx_Erase();
    smgfx_SetLook( 1, 2, 0, 0, 1 ); m_pTargetBrep->Draw( TRUE ); sm_GraphicsLoop();

    // Draw corners first (big), then solvers.
    smgfx_SetLook( 5, 6, 1, 0, 0 );
    for(cornerIdx = 0; cornerIdx < m_vFilletCorners.GetSize(); cornerIdx++)
    {
      SmFilletCorner * pCorner = m_vFilletCorners[cornerIdx];
      pCorner->Draw(); sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
    sm_GraphicsLoop();

    smgfx_SetLook( 5, 6, 1, 0, 1 );
    for(solverIdx = 0; solverIdx < m_vFilletSolvers.GetSize(); solverIdx++)
    {
      SmFilletSolver * pFilSolver = m_vFilletSolvers[solverIdx];
      smgfx_ChangeColor(solverIdx != 0); pFilSolver->Draw(); sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
  }
#endif // SM_DEBUG_CODE

  // bTrimRails This is optional?
  if ( TRUE )
  {
    // Trim rail curves by the rail ends
    // This is strictly within the fillet Solvers: no trimming to Target or Fillet Breps yet.
    for(solverIdx = 0; solverIdx < m_vFilletSolvers.GetSize(); solverIdx++)
    {
      SmFilletSolver * pFilSolver = m_vFilletSolvers[solverIdx];
      if(pFilSolver->TrimRails() != SM_SUCCESS)
      {
        const SmEdge* pE = pFilSolver->GetEdgeuse( 0 )->GetEdge();
        GetFilletErrorInfo()->NoteError( pE, SM_FV_NO_INT_RAIL_EU, SM_FILERR_EDGE_PROBLEM,
                                         _T( "Fillet Surface Computation Failure: Unable to Trim Rail Curve" ) );
        FILEXEC_SOLVER_ERR( pFilSolver, _T( "Unable to Trim Rail Curve" ) );
      }

#ifdef SM_DEBUG_CODE
      if(m_iDebugLevel >= 5)
      {
        smgfx_Erase();
        smgfx_SetLook( 1, 2, 0, 0, 1 ); m_pTargetBrep->Draw( TRUE ); sm_GraphicsLoop();
        smgfx_SetLook( 5, 6, 1, 0, 0 ); pFilSolver->Draw(); sm_GraphicsLoop();
        sm_GraphicsLoop();
      }
#endif // SM_DEBUG_CODE

    } // end iter m_vFilletSolvers

  }  // end if bTrimRails


#ifdef SM_DEBUG_CODE
  // Display corners and fillet surfaces.
  if(m_iDebugLevel > 0)
  {
    smgfx_Erase();
    smgfx_SetLook( 1, 2, 0, 0, 1 ); m_pTargetBrep->Draw( TRUE ); sm_GraphicsLoop();

    // Draw corners first (big), then solvers.
    smgfx_SetLook( 5, 6, 1, 0, 0 );
    for(cornerIdx = 0; cornerIdx < m_vFilletCorners.GetSize(); cornerIdx++)
    {
      SmFilletCorner * pCorner = m_vFilletCorners[cornerIdx];
      pCorner->Draw(); sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
    sm_GraphicsLoop();

    smgfx_SetLook( 5, 6, 1, 0, 1 );
    for(solverIdx = 0; solverIdx < m_vFilletSolvers.GetSize(); solverIdx++)
    {
      SmFilletSolver * pFilSolver = m_vFilletSolvers[solverIdx];
      smgfx_ChangeColor(solverIdx != 0); pFilSolver->Draw(); sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
  }
#endif // SM_DEBUG_CODE

  if(m_vFilletSolvers.GetSize() < 1)
    return SM_ERR;

  for(solverIdx = 0; solverIdx < m_vFilletSolvers.GetSize(); solverIdx++)
  {
    SmFilletSolver * pFilSolver = m_vFilletSolvers[solverIdx];

    SmTArray<SmFilletGeom*> sFG;
    pFilSolver->GetFilletGeoms( sFG );
    for(ULONG ii = 0; ii < sFG.GetSize(); ii++)
    {
      SmSurface* pSrf = sFG[ii]->GetFilletSurface();
      rFilletSurfaces.Add( pSrf );
    }
  }

  // all done
  return SM_SUCCESS;

} // end SmFilletExecutive::GetPreviewFilletSurfaces

/*******************************************************************//**
PURPOSE: High-level call to set up filleting parameters,
         for surface-surface filleting.

NOTES:  This is different from the topology-based (edge-based) filleting
    user interface in that a single method sets up the parameters and performs
    all of the filleting.  This is because new geometry and topology must be
    calculated and created before most of the parameters can be set.
***********************************************************************/
SmStatus SmFilletExecutive::SurfaceSurfaceFillet
 (const SmContext      & crContext,       // in : Creation context for the new brep that contains the results.
  SmSurface            * pSur1,           // in : Surface to be filleted (can be from different Breps)
  SmSurface            * pSur2,           // in : Surface to be filleted (can be from different Breps)
  double                 dFilletRadius1,  // in : signed radius from pSur1 (FilletCenter = Surf/Surf XSect of pSur1Offset(SignedDist1))
  double                 dFilletRadius2,  // in : signed radius from pSur2 (FilletCenter = Surf/Surf XSect of pSur2Offset(SignedDist2))
  double                 dTolerance,      // in : Tolerance: max dist from rails to base surfaces
  SmBrep              *& rpResult,        // out: Brep containing the new fillet surface(s)
  ULONG                  lXSectType,      // in : 0=linear, 1=approx circular, 2=circular (rational)
                                          //      default:[2]
  double                 dXSectAccuracy,  // in : Relative accuracy of approx circular
                                          //      default:[0.1]
  SmBoundaryTrimmingType eFilTrimType,    // in : SM_BT_NONE,   = No fillet triming just generate entire fillet
                                          //      SM_BT_MINIMAL = Trim using ISO curves to minimal intersections on face boundaries
                                          //      SM_BT_MAXIMAL = Trim using ISO curves to maximal intersections on face boundaries
                                          //      SM_BT_BEVEL,  = Create a bevel (line in parameter space) between minimal and maximal
                                          //                      intersections on each end of fillet.
                                          //      SM_BT_BLEND   = Create a blend (Hermite in parameter space) between minimal and 
                                          //                      maximal intersections on each end of fillet.
                                          //      default:[SM_BT_MAXIMAL]
  ULONG                  lBaseTrimType,   // in : base surface trimming: 0=none; 1=normal; 2=reverse trim.
                                          //      default:[0]
  SmBoolean              bMirror,         // in : TRUE = mirror fillet to produce an inside out fillet, FALSE=don't 
                                          //      default:[FALSE]
  SmBoolean              bComplement)     // in : TRUE = complement circular and approx circular cross sections, FALSE=don't.
                                          //      default:[FALSE]
                                          // note: It is possible to both mirror and complement circular cross sections.
{
  // Init outputs
  rpResult = NULL;

  // Create temporary offset surfaces
  SmOffsetSurface * pOff1 = new(crContext) SmOffsetSurface(dFilletRadius1,*pSur1,FALSE);
  SmObjDelete sCleanSrf1(pOff1);
  
  SmOffsetSurface * pOff2 = new(crContext) SmOffsetSurface(dFilletRadius2,*pSur2,FALSE);
  SmObjDelete sCleanSrf2(pOff2);

#ifdef SM_GFX_CODE
SmBoolean bDebugMe = FALSE;
  if (bDebugMe) 
   {
      smgfx_Erase() ;
      smgfx_SetLook(1,2, 1,0,0); pSur1->Draw() ; sm_GraphicsLoop();
      smgfx_SetLook(1,2, 1,0,1); pOff1->DrawUV(5,5); sm_GraphicsLoop();
      smgfx_SetLook(1,2, 0,0,1); pSur2->Draw() ; sm_GraphicsLoop();
      smgfx_SetLook(1,2, 0,1,1); pOff2->DrawUV(5,5); sm_GraphicsLoop();
      sm_GraphicsLoop();
  }
#endif // SM_GFX_CODE
  
  // Intersect the offset surfaces.
  // We will use only the two uv curves for our purposes.

  // locals for GlobalSurfaceIntersect()
  SmTArray<SmCurve*> sUVCurves1, sUVCurves2;
  SmObjsDelete<SmCurve*> sClean1(&sUVCurves1);
  SmObjsDelete<SmCurve*> sClean2(&sUVCurves2);

  SmBoolean bUseSurfaceEdges[2];
  bUseSurfaceEdges[0] = TRUE;
  bUseSurfaceEdges[1] = TRUE;
  double dTol = (smos_Fabs(dFilletRadius1)+smos_Fabs(dFilletRadius2)) / 1000.0;

  // Intersect offset surfaces to get UV curves
  SER(pOff1->GlobalSurfaceIntersect
        (crContext,                      // in : Context for new object construction
         pOff1->GetNaturalUVDomain(),    // in : this Surface intersection limits
         *pOff2,                         // in : target 2nd intersecting surface
         pOff2->GetNaturalUVDomain(),    // in : 2nd Surface intersection limits
         bUseSurfaceEdges,               // in : TRUE = Find intersection curve start points by intersecting
                                         //             the edges of one surface with the other.
                                         //      Normally both are TRUE unless you know
                                         //      that the edges of one surface do not intersect
                                         //      the other surface.  It is a slight optimization
                                         //      to set the flag to FALSE
         SM_CAST_APPROXTOL3D_PTR(&dTol), // in : If not given it uses 1/1000 of surface size
                                         //      approximation tolerance
         NULL,                           // in : If not given it uses 30 degrees
         NULL,                           // out: 3D curves produced by intersection
         &sUVCurves1,                    // out: UV curves on this surface produced by intersection
         &sUVCurves2,                    // out: UV curves on crOtherSurface produced by intersection
         NULL,                           // out: produced Curve types, NULL to ignore
                                         //      oneof: SM_TC_TOUCHING   - single point intersection (surf norms parallel)
                                         //             SM_TC_CROSSING   - curve intersection (surf norms not parallel)
                                         //             SM_TC_TANGENT    - curve intersection (surf norms parallel)
                                         //             SM_TC_COINCIDENT - curve intersection (surf norms parallel & cross-tangents equal)
                                         //             SM_TC_NEAR_TANGENT    - curve has small angle of intersection
                                         //             SM_TC_REGION_BOUNDARY - curve bounds region within which the surfs are coincident
         NULL));                         // out: produced Curve Deviations from each UVCurve to true xSect, NULL to ignore
  
  // no work - no intersection
  if (sUVCurves1.GetSize() == 0) return SM_SUCCESS;

  // When more than one intersection curve was found,
  // check for self-intersections: we don't do those.
  ULONG ii, jjj;
  if (sUVCurves1.GetSize() > 1) 
    {
      // test for self-intersections by intersecting uvCurves
      for (ii=0; ii<sUVCurves1.GetSize(); ii++) 
        {
          SmCurve    * pUVCurve1 = sUVCurves1[ii];
          SmExtent1d   sIvl1     = pUVCurve1->GetNaturalInterval();
          for (jjj=ii+1; jjj<sUVCurves1.GetSize(); jjj++) 
            {
              SmCurve    * pUVCurve2 = sUVCurves1[jjj];
              SmExtent1d   sIvl2     = pUVCurve2->GetNaturalInterval();
              SmSolutionArray sSolutions;
              SER(pUVCurve1->GlobalCurveIntersect(sIvl1,*pUVCurve2,sIvl2,
                                                  dTol,sSolutions));
              // Self-intersecting fillets are not yet handled
              if (sSolutions.GetSize() > 0) 
                {
                  // Self-intersection found, exit for now
                  return SM_ERR;
                }
            }
        }
    } // end more than 1 intersection curve check
  
  // Got the offset intersection.
  // Create the fillet executive object.
  SmFilletExecutive sFillExec(crContext);
  
  // Create the fillet surface generator for the cross section.
  // The three possibilites are:
  //   0: SmLinearCrossSectionFSG
  //   1: SmCircularCrossSectionFSG approximated.
  //   2: SmCircularCrossSectionFSG with exact rational (default)

  // Create one of each (Linear and Circular) on the stack, and set
  // the pFSG pointer appropriately.

  SmLinearCrossSectionFSG sFSGLinear;

  // Note, all of the user options apply only to Circular.
  SmBoolean bApprox = (lXSectType == 1);
  SmCircularCrossSectionFSG sFSGCircular( bApprox, dXSectAccuracy, bMirror, bComplement );

  SmFilletSurfaceGenerator *pFSG =   (lXSectType == 0)
                                   ? (SmFilletSurfaceGenerator *)(&sFSGLinear)
                                   : (SmFilletSurfaceGenerator *)(&sFSGCircular);

  // Info for trimming the base surfaces:
  SmBoolean bReverseTrim = ( lBaseTrimType == 2 );

  // Each curve produced during intersection may produce a 
  // separate fillet surface.
  ULONG kk;
  for ( kk=0; kk<sUVCurves1.GetSize(); kk++ ) 
    {
      // Use the UV parameters near the mid points of the 
      // intersection curves as seed points for the Surface/Surface 
      // fillet solver.
      // This is the point where the ball starts rolling.
      // We then roll it in both directions.
      SmCurve    *pUVCurve1 = sUVCurves1[kk];
      SmExtent1d  sIvl1     = pUVCurve1->GetNaturalInterval();
      SmCurve    *pUVCurve2 = sUVCurves2[kk];
      SmExtent1d  sIvl2     = pUVCurve2->GetNaturalInterval();

      SmPoint3d sPnt1, sPnt2;
      SER( pUVCurve1->EvaluatePoint( sIvl1.Evaluate(0.4567), sPnt1 ));
      SER( pUVCurve2->EvaluatePoint( sIvl2.Evaluate(0.4567), sPnt2 ));
      SmPoint2d sGuess1, sGuess2;
      sGuess1.x = sPnt1.x; sGuess1.y = sPnt1.y;
      sGuess2.x = sPnt2.x; sGuess2.y = sPnt2.y;

#ifdef SM_GFX_CODE
SmBoolean bDebugMe1 = FALSE;
      if (bDebugMe1) 
        {
          smgfx_SetLook(1,2, 0,1,0);
          // centerline
          SmCrvOnSurf sCenterCrvOnSurf(*pUVCurve1,*pOff1);
          sCenterCrvOnSurf.SetContext(NULL);
          sCenterCrvOnSurf.Draw(); sm_GraphicsLoop();

          // rails
          SmCrvOnSurf sRailCrvOnSurf1(*pUVCurve1,*pSur1);
          sRailCrvOnSurf1.SetContext(NULL);
          sRailCrvOnSurf1.Draw(); sm_GraphicsLoop();

          SmCrvOnSurf sRailCrvOnSurf2(*pUVCurve2,*pSur2);
          sRailCrvOnSurf2.SetContext(NULL);
          sRailCrvOnSurf2.Draw(); sm_GraphicsLoop();

          sm_GraphicsLoop();
        }
#endif // SM_DEBUG_CODE

      // Create the fillet solver for filleting two surfaces.
      SmBoolean bOrient1 = (dFilletRadius1 < 0.0) ? FALSE : TRUE ;
      SmBoolean bOrient2 = (dFilletRadius2 < 0.0) ? FALSE : TRUE ;

      // A couple of default tolerances.
      static double dAngTol = SM_DEG2RAD( 30.0 );
      static double dTanTol = SM_DEG2RAD(  1.0 );

      SmSurfaceSurfaceFS *pFS = new(crContext) SmSurfaceSurfaceFS
          (crContext,                    // in : Creation context for new geometry 
           dTolerance,                   // in : Tolerance used to specify maximal distance between 3D
                                         //      rail curves and the surface on which it lies.
           dAngTol,                      // in : The largest angle that the rail curves can 
                                         //      traverse before generating a new knot.  
           dTanTol,                      // in : Angle used to determine quality of tangency angle 
                                         //      between fillet surface and rail surfaces.
           smos_Fabs( dFilletRadius1 ),  // in : Radius of the rolling ball used to generate the fillet
           smos_Fabs( dFilletRadius2 ),  //      for each surface.  Note that the radii are signed and  
                                         //      correspond to the offset direction of the surface that 
                                         //      is used to generate the centerline curve of the fillet.
           *pSur1,                       // in : Underlying Orig surface1, stored as BaseSurface of new OffsetSurfaces
           *pSur2,                       // in : Underlying Orig surface2, stored as BaseSurface of new OffsetSurfaces
           bOrient1,                     // in : offset orientation for Sur1
           bOrient2,                     // in : offset orientation for Sur2
           sGuess1,                      // in : UV position to start rolling the ball on pSur1.
           sGuess2);                     // in : UV position to start rolling the ball on pSur2.  
                                         //      note that these points do not have to lie exactly on the  
                                         //      rails but should be near them.
      
      // Set the user-specified quantities for this solver.
      pFS->SetFilletTrimType( eFilTrimType );
      pFS->SetReverseTrim( bReverseTrim );
      
      // Attach the fillet surface generator to the fillet solver
      pFS->SetFilletSurfaceGenerator( pFSG );
      
      // Load the fillet solver into the fillet executive
      SER(sFillExec.LoadFilletSolver( pFS ));

    } // end iter every intersection curve pair
  
  // The following allows us to turn off trimming of the original fillet geometry.
  if ( lBaseTrimType == 0 )
    { sFillExec.SetDoTrimming( FALSE ); }
  
  // Now that we have everything set up - go out and do the job of 
  // generating the surface fillet.
  SER( sFillExec.DoSurfaceFilleting() );
  
  // Get the resulting Brep that contains the fillet surfaces.
  rpResult = sFillExec.GetFilletBrep();
  
#ifdef SM_GFX_CODE
  if ( bDebugMe && rpResult != NULL ) 
    {
      smgfx_Erase() ;
      smgfx_SetLook(1,2, 1,1,0); rpResult->Draw(TRUE); sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
#endif // SM_GFX_CODE
  
  return SM_SUCCESS;

} // end SmFilletExecutive::SurfaceSurfaceFillet

/*******************************************************************//**
PURPOSE: High-level call to execute the filleting on the Brep.

NOTES: SetFilletingParameters() must have been called prior to this call.
***********************************************************************/
SmStatus SmFilletExecutive::DoFillet()
{
  // locals
  SmStatus eStat;

  // check state - must be Initialized
  if ( ! m_bIsInitialized )
    {
      m_sErrorInfo.NoteError(SM_FILERR_EXEC_UNINITIALIZED,
                             _T("ERROR: fillet parameters not set up.") );
      SER( SM_ERR_INVALID_INPUT );
    }

  // Create FilletCorner in m_vFilletCorners for every m_vFilletedVertices vertex and 
  // add [vertex,SmFilletCorner] entry pair to m_vVertexCornerMap 
  eStat = CreateFilletCorners();
  SER( eStat );

  // pass the call along - for actual filleting 
  eStat = DoFilleting();  // note: increments unlocked mark value
  SER( eStat );

  // all done
  return SM_SUCCESS;

} // end SmFilletExecutive::DoFillet

/*******************************************************************//**
PURPOSE: Get the Fillet Corner associated with a vertex if there is
    one.  If not return NULL.

NOTES:
***********************************************************************/
SmFilletCorner * SmFilletExecutive::GetFilletCornerOfVertex
 (SmVertex* pVertex)
 const
{
  SmFilletCorner * pFilCorner = (SmFilletCorner*)m_vVertexCornerMap.At(pVertex);

  return pFilCorner;

} // end SmFilletExecutive::GetFilletCornerOfVertex

/*******************************************************************//**
PURPOSE: Get the Fillet Solver associated with an Edge if there is one.
   If not return NULL.

NOTES:
***********************************************************************/
SmFilletSolver * SmFilletExecutive::GetFilletSolverOfEdge(SmEdge* pEdge) 
const
{
  SmFilletSolver * pFilSolver = NULL;
  SmTArray< SmEdgeuse* > sEUs;
  pEdge->GetEdgeuses( sEUs );

  for ( ULONG ii=0; ii<sEUs.GetSize(); ii++ )
  {
      SmEdgeuse *pEU = sEUs[ii];
      pFilSolver = this->GetFilletSolverOfEdgeuse( pEU );
      if ( pFilSolver != NULL )
        { break; }
  }

  return pFilSolver;

} // end SmFilletExecutive::GetFilletSolverOfEdge

/*******************************************************************//**
PURPOSE: Get the Fillet Solver associated with an edgeuse if there is
    one.  If not return NULL.

NOTES:
***********************************************************************/
SmFilletSolver * SmFilletExecutive::GetFilletSolverOfEdgeuse
 (SmEdgeuse* pEdgeuse) 
const
{
  SmFilletSolver * pFilSolver = m_vEdgeuseSolverMap.At(pEdgeuse);

  return pFilSolver;

} // end SmFilletExecutive::GetFilletSolverOfEdgeuse

/*******************************************************************//**
PURPOSE: Get the Rail corresponding to an edgeuse of an edge if
   one exists.  If not return NULL.

NOTES:
***********************************************************************/
SmStatus SmFilletExecutive::GetRailOfEdgeuse
 (SmEdgeuse       * /* pEdgeuse       */,
  SmFilletSolver *& /* rpFilletSolver */,
  ULONG           & /* rlRailIndex    */)
 const
{
  return SM_ERR;

} // end SmFilletExecutive::GetRailOfEdgeuse

/*******************************************************************//**
PURPOSE: Register each FilletSolver object (i.e. let Fillet Executive
    know which fillet needs to be solved)

NOTES:
  1. add each G1 continuity point in the filletEdge->Curve to the pFS->m_vG1Knots array
  2. add SmFilletSolver, pFS, to m_vFilletSolvers array
  3. Set pFS backPointer to this SmFilletExecutive
  4. for both filletEdge->Vertices and filletSector edgeuses
         - make sure vertex is listed in m_vFilletedVertices
         - make sure each edgeuse is listed in the vertices list
             of filleted edges stored in m_vVertexCornerMap
***********************************************************************/
SmStatus SmFilletExecutive::LoadFilletSolver
 (SmFilletSolver * pFS)  // in : target FilletSolver to add to this FilletExecutive
                         //      One FilletSolver for each edge to be filleted
{
  // check state - FilletSolver should always find two offset surfaces to fillet
  NER( pFS->GetSurface(0) );
  NER( pFS->GetSurface(1) );

  ULONG ii, jj;

  // get sector (represented by Edgeuse) to fillet
  SmEdgeuse * pEdgeuse = pFS->GetEdgeuse(0);
  if (pEdgeuse)
    {
      // get fillet curve locals - knots, cache, and continuities
      SmCurve * pCurve = pEdgeuse->GetEdge()->GetCurve(); NER(pCurve);
      SmTArray<double> sKnots;
      SER(pCurve->GetKnots(sKnots));
      SmCurveCache *pCC1 = (SmCurveCache*)SmCacheMgr::GetOrCreateObjectCache(SM_OC_CURVE,pCurve); NER(pCC1);
      const SmTArray<SmContinuityType> & rConts = pCC1->GetContinuitiesArray();

      // for every continuity point in the curve
      for (ii=0; ii<rConts.GetSize(); ii++)
        {
          // place internal G1 continuity points into m_vG1Knots array
          if (rConts[ii] == SM_CT_G1)
            {
              SM_DBG_WARN(_T("Edge with G1 Continuity found. Possible filleting problems"));
              pFS->m_vG1Knots.Add(sKnots[ii]);
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
              if (bDebugMe) {
                  SmPoint3d sPnt;
                  SER(pCurve->EvaluatePoint(sKnots[ii],sPnt));
                  smgfx_SetLook(5,8, 1,0,0); sPnt.Draw(); sm_GraphicsLoop();
                  sm_GraphicsLoop();
              }
#endif // SM_DEBUG_CODE
            } // end G1 continuity point check
        } // end iter every continuity point in the filletEdge->Curve
    } // end pEdgeuse existence check

  // Store target FilletSolver in FilletExecutive FillerSolver array and set back pointer
  m_vFilletSolvers.Add(pFS);
  pFS->SetFilletExecutive(this);

  // get filletSector's edgeuses and filletEdge vertices
  //   for both edgeuses - add a [Edgeuse, FilletSolver] pair entry to m_vEdgeuseSolverMap
  SmEdgeuse * pEU[2];
  SmVertex  * pVerts[2];
  for (ii=0; ii<2; ii++)
    {
      pEU[ii] = pFS->GetEdgeuse(ii);
      if (pEU[ii] != NULL)
        {
          // Create an entry in the Edgeuse-Solver Map
          m_vEdgeuseSolverMap.Insert(pEU[ii], pFS);

          // Get start vertex of edgeuse
          pVerts[ii] = pEU[ii]->GetVertexuse()->GetVertex();
        }
    } // end iter both sector edgeuses

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe2 = FALSE;
  if (bDebugMe2) 
    {
      smgfx_SetLook(1,2, 1,0,0); pEU[0]->Draw(); sm_GraphicsLoop();
      smgfx_SetLook(1,2, 0,0,1); pEU[1]->Draw(); sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

  // for both filletEdge->Vertices and filletSector edgeuses
  //   - make sure vertex is listed in m_vFilletedVertices
  //   - make sure each edgeuse is listed in the vertices list
  //       of filleted edges stored in m_vVertexCornerMap
  for (ii=0; ii<2; ii++)
    {
      SmTArray<SmEdgeuse*> * pSolverEUs = NULL;
      SmBoolean bFound = FALSE;

      // for every vertex listed in m_vFilletedVertices
      for (ULONG kk=0; kk<m_vFilletedVertices.GetSize(); kk++)
        {
          SmVertex * pFillVert = m_vFilletedVertices[kk];

          // when this filletedEdge->Vertex == m_vFilletedVertices member
          if (pVerts[ii] == pFillVert)
            {
              // remember that this vertex was found
              bFound = TRUE;

              // get list of filleted edgeuses connected to this vertex
              pSolverEUs = (SmTArray<SmEdgeuse*>*)m_vVertexCornerMap.At(pFillVert);
              NER(pSolverEUs);

              // see if this edge is already in the list of filleted edges connected to this vertex
              SmBoolean bFoundCoincidentEdge = FALSE;
              for (jj=0; jj<pSolverEUs->GetSize(); jj++)
                {
                  SmEdgeuse * pSolverEU = (*pSolverEUs)[jj];
                  if (pEU[ii]->GetEdge() == pSolverEU->GetEdge())
                    {
                      bFoundCoincidentEdge = TRUE;
                      break;
                    }
                }

              // Add Solver edgeuse to the corner's filleted edgeuse array
              if (!bFoundCoincidentEdge)
                {
                  pSolverEUs->Add(pEU[ii]);
                }
              break;
            } // end this filletedEdge->Vertex == m_vFilletedVertices member check
        } // end iter both filletedEdge->Vertices

      // when the vertex was not in the m_vFilletedVertices list
      if (!bFound && pEU[0] != NULL)
        {
          // Create array of to-be-filleted edgeuses connected to this vertex
          SmTArray<SmEdgeuse*> * pSolverEdgeUse = new(*GetContext()) SmTArray<SmEdgeuse*>(*GetContext());
          NER( pSolverEdgeUse );
          pSolverEdgeUse->Add(pEU[ii]);
          m_vFilletedVertices.Add(pVerts[ii]);
          // Create an entry in the Vertex-Corner Map
          m_vVertexCornerMap.Insert(pVerts[ii], (SmObject*) pSolverEdgeUse);
        }
    } // end iter (ii) both filletSector edgeuses and filletEdge vertices

  return SM_SUCCESS;

} // end SmFilletExecutive::LoadFilletSolver

/*******************************************************************//**
PURPOSE: This routine allocates a fillet corner of appropriate type
    for every member of the m_vFilletedVertices list and adds it
    to the m_vFilletCorners array.  Each vertex/FilletVertex
    relationship is stored as a pair of pointers in m_vVertexCornerMap.

NOTES:
  For every m_vFilletedVertices vertex
    a. Create an SmFilletCorner Object
    b. Add SmFilletCorner Object to m_vFilletCorners array
    c. Add [vertex,SmFilletCorner] entry pair to m_vVertexCornerMap

  The appropriate type of FilletCorner is determined by the
  number of edges meeting at the corner vertex, the number of
  those edges being filleted, and if any of the fillet edges
  are closed or open.

***********************************************************************/
SmStatus SmFilletExecutive::CreateFilletCorners
 (SmFilletErrorInfo * pOptFilletErrorInfo)
{
  // for every vertex listed in m_vFilletedVertices array
  ULONG ii, lNumFVs = m_vFilletedVertices.GetSize();
  for (ii=0; ii<lNumFVs; ii++)
    {
      SmVertex * pVert = m_vFilletedVertices[ii];

      // get list of all edgeuses being filleted connected to this vertex
      SmTArray<SmEdgeuse*> * pSolverEUs = (SmTArray<SmEdgeuse*>*)m_vVertexCornerMap.At(pVert);
      NER(pSolverEUs);

      // create a SmFilletCorner Object
      //  Selects a derived type SmFilletCorner based on the
      //    number of to-be-filleted edgeuses and total-number of edgeuses connected to vertex
      //    and edge properties to create the new topology needed for the filleted vertex
      SmFilletCorner * pCorner = SmFilletCorner::Create(m_crContext,pVert,pSolverEUs,this);

      // inform the public when no SmFilletCorner derived type handles this corner case
      if (!pCorner)
        {
          SM_DBG_WARN(_T("Error or Unimplemented corner case"));
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
          if (bDebugMe) {
              smgfx_SetLook(4,10, 1,0,0); pVert->Draw(); sm_GraphicsLoop();
              sm_GraphicsLoop();
          }
#endif // SM_DEBUG_CODE
          continue;
        } // end no corner check

      // add FilletCorner Object to m_vFilletCorners
      // and [pVert,FilletCorner] pair entry to m_vVertexCornerMap
      m_vFilletCorners.Add(pCorner);
      m_vVertexCornerMap.Insert(pVert, (SmObject*) pCorner);

    } // end iter every m_vFilletedVertices vertex

  if ( pOptFilletErrorInfo != NULL )
    {
      *pOptFilletErrorInfo = *( GetFilletErrorInfo() );
    }

  return SM_SUCCESS;

} // end SmFilletExecutive::CreateFilletCorners

/*******************************************************************//**
PURPOSE: This routine create fillets and their topologies in m_pFilletBrep
    for the geometry in m_pTargetBrep. 

NOTES: The result is m_pFilletBrep, a brep which consists of open shells 
    (i.e. fillets and corners) in which the fillet and corner faces are 
    stitched together along common edges.
***********************************************************************/
SmStatus SmFilletExecutive::CreateFilletBrep
 (SmFilletErrorInfo * pOptFilletErrorInfo)
{
  // check state - must have a target Brep
  NER(m_pTargetBrep);

#ifdef VALIDATE_TOPOLOGY
  m_pTargetBrep->ValidatePointers();
#endif // VALIDATE_TOPOLOGY

  // locals
  ULONG cornerIdx, solverIdx, i;
  TCHAR sBuff[SM_TBLOCK_SIZE]; // Use this for error reporting as well as debug.

#ifdef SM_DEBUG_CODE
  ULONG idbg;
  int iDebugLevel = DebugLevel();
  if ( iDebugLevel > 0 ) 
    {
      smos_sprintf(sBuff,_T("%s"),_T("\nFillet: Enter CreateFilletBrep()\n"));
      smos_WriteBuffer(sBuff);
    }

  SmBoolean bDebugMe0=FALSE;
  if ( bDebugMe0 ) {
      smgfx_Erase();
      smgfx_SetLook( 1,2, 0,0,1 ); m_pTargetBrep->Draw(TRUE); sm_GraphicsLoop();
      sm_GraphicsLoop();
      for ( idbg=0; idbg<m_vFilletCorners.GetSize(); idbg++ )
        { m_vFilletCorners[idbg]->Draw();  sm_GraphicsLoop(); }
      for ( idbg=0; idbg<m_vFilletSolvers.GetSize(); idbg++ )
        { m_vFilletSolvers[idbg]->Draw();  sm_GraphicsLoop(); }
      sm_GraphicsLoop();
  }

  static constexpr SmBoolean sbDumpBrep=FALSE;
  if ( sbDumpBrep )
    { m_pTargetBrep->Dump(); }

  static constexpr SmBoolean sbWriteBrep=FALSE;
  if ( sbWriteBrep )
    { m_pTargetBrep->WriteToFile( _T("C:/SMLib/Bugs/Bug570/targetBrep.smb"), SM_ASCII ); }

#endif // SM_DEBUG_CODE

  // Process smooth sequences of filleted Edges.
  // If this fails, the fillet can still continue.
  this->ProcessSmoothEdgeSequences();

  // Process corners by calculating the vertices first.
  // Those vertices will determine the connectivity between
  // corner surfaces and the adjacent edge fillets.
  for ( cornerIdx=0; cornerIdx<m_vFilletCorners.GetSize(); cornerIdx++ )
    {
      SmFilletCorner * pCorner = m_vFilletCorners[cornerIdx];

#ifdef SM_DEBUG_CODE
      if ( iDebugLevel > 0 ) 
        {
          smos_sprintf(sBuff, _T("\nExec: Creating Corner # %ld : "), cornerIdx );
          pCorner->DumpLevel( iDebugLevel, sBuff) ;
          smgfx_Erase();
          smgfx_SetLook(1,2, 0,0,0); m_pTargetBrep->Draw(TRUE); sm_GraphicsLoop();
          smgfx_SetLook(3,4, 0,0,1); m_pPseudoBrep->Draw(TRUE); sm_GraphicsLoop();
          smgfx_SetLook(5,6, 1,0,0); pCorner->Draw(); sm_GraphicsLoop();
          sm_GraphicsLoop();
        }
#endif // SM_DEBUG_CODE

      // Create corner topology
      //  - allocate and set SmFilletVertex objects and add them to
      //    the pCorner->m_vVertices array
      //  - as a side effect allocate 1 FilletEdge, 2 FilletEdgeuses,
      //    2 FilletVertexuse for every fillet edge needed to connect a pair
      //    of new SmFilletVertex objects at this corner -
      //  - if needed, allocate SmFilletEdge objects for rails
      //    and connect rails to FilletVertex with 
      //    FilletEdge->FilletEdgeuse->FilletVertexuse->FilletVertex sequences.
      //    The shape of new filletEdges will be computed later
      //    by the FilletSolver.
      if(SM_SUCCESS != pCorner->MakeCornerTopology())
        {
          // when MakeCornerTopology failed - note the error
          pCorner->GetName( sBuff, SM_TBLOCK_SIZE );
          NoteFilletError( SM_FILERR_VERTEX_PROBLEM, pCorner->GetCornerType(),
                           SM_FIL_FAILURE, sBuff, pCorner->GetFilletedVertex(),
                           NULL, NULL, // For now, we don't generally have just two surfaces.
                           NULL, NULL, // For now, we don't generally have just two edgeuses.
                           _T("Fillet Corner Error: Unable to Make Topology") );
          FILEXEC_CORNER_ERR(pCorner,_T("Unable to Make Topology"));
        }

      // For some fillets, we want to calculate the intersections at this corner as if
      // a variable-radius fillet were actually constant radius, at the end value of the
      // radius function.  See class SmTemporaryRadiusChange.
      //
      // Do const-rad if:
      //  it's a blending corner, unless it's interior to a smooth sequence. So:
      //  - 2x2 : if interior, no.
      //  - Nx2 : if interior, no.
      //  - Nx1 closed: yes, it's not interior.

      SmBoolean bDoConstRad = pCorner->IsBlendingCorner();
      if ( bDoConstRad )
      {
          // Don't do const rad for Nx2 tangent corners.
          if ( pCorner->IsTangentCorner() )
          {
              // Being Tangent narrows it down to either 2x2, Nx2, or Nx1 Closed.
              // We do want const-rad if it's the start or end of a smooth sequence. [B683]
              // That includes Nx1 Closed.  For Nx2, check whether it's interior in a smooth sequence.
              // Do that by checking the distance from this filleted vertex to the ends of the edge curve.
              SmTArray< SmFilletSolver* > sFilSolvers;
              pCorner->GetFilletSolvers( sFilSolvers );
              if ( sFilSolvers.GetSize() == 2 )
              {
                  SmFilletSolver * pFS1 = sFilSolvers[0];

                  // This corner is between two edges that meet smoothly.
                  // The curve of the filleted Edge in a smooth sequence extends through
                  // the entire sequence.  See whether this corner is the start/end of
                  // that curve, or not.
                  SmEdge *pE = pFS1->GetEdgeuse(0)->GetEdge();
                  SmCurve *pCurve = pE->GetCurve();
                  SmPoint3d sPt1, sPt2, sCornerPt;
                  pCurve->GetEnds( sPt1, sPt2 );  // Should be the same.
                  sCornerPt = pCorner->GetFilletedVertex()->GetPoint();
                  double dDist1 = sPt1.DistanceBetween( sCornerPt );
                  double dDist2 = sPt2.DistanceBetween( sCornerPt );
                  double dTol = 2.0 * pCorner->GetThisApproxTol3d();
                  if ( dDist1 > dTol  &&  dDist2 > dTol )
                    { bDoConstRad = FALSE; }
              }
          }
      }  // end if bDoConstRad further checks.

      pCorner->SetCalcAsConstRad( bDoConstRad );

      // Set point positions for the new filletVertices 

      if(SM_SUCCESS != pCorner->CalcCornerVertGeom())
        {
          // when MakeCornerTopology failed - note the error
          pCorner->GetName( sBuff, SM_TBLOCK_SIZE );
          NoteFilletError( SM_FILERR_VERTEX_PROBLEM, pCorner->GetCornerType(),
                           SM_FIL_FAILURE, sBuff, pCorner->GetFilletedVertex(),
                           NULL, NULL, // For now, we don't generally have just two surfaces.
                           NULL, NULL, // For now, we don't generally have just two edgeuses.
                           _T("Fillet Corner Error: Unable to Compute Geometry") );
          FILEXEC_CORNER_ERR(pCorner,_T("Vertex Computation Failure"));
        }

#ifdef SM_DEBUG_CODE
      if ( iDebugLevel > 0 ) 
        {
          smos_sprintf(sBuff, _T("\nExec: After Creating Corner # %ld Topo and Geom : "), cornerIdx );
          pCorner->DumpLevel( iDebugLevel, sBuff) ;
          smgfx_Erase();
          smgfx_SetLook(1,2, 0,0,0); m_pTargetBrep->Draw(TRUE); sm_GraphicsLoop();
          smgfx_SetLook(3,4, 0,0,1); m_pPseudoBrep->Draw(TRUE); sm_GraphicsLoop();
          smgfx_SetLook(5,6, 1,0,0); pCorner->Draw(); sm_GraphicsLoop();
          sm_GraphicsLoop();
        }
#endif // SM_DEBUG_CODE

    } // end of loop over corners, creating topology and vertex geometry


  // We have created the topology for the corners, and then filled in
  // their geometry.  Sometimes, however, the topology can depend
  // on the geometry.  An example would be the 3x2 case, if two rails
  // intersect the un-filleted edge in different locations, a new fillet
  // edge and vertex will be inserted.
  // Another example is a degenerate corner, which is handled separately.
  // So for every corner, adjust topology as required.
  for ( cornerIdx=0; cornerIdx<m_vFilletCorners.GetSize(); cornerIdx++ )
    {
      SmFilletCorner * pCorner = m_vFilletCorners[cornerIdx];

#ifdef SM_DEBUG_CODE
      if ( iDebugLevel > 0 ) 
        {
          ULONG lIdx = FindCornerIndex( pCorner );

          smos_sprintf(sBuff,_T("\nExec: Adjusting corner # %ld, [%4ld] :\n"), cornerIdx, lIdx );
          smos_WriteBuffer(sBuff);


          smgfx_Erase();
          smgfx_SetLook(1,2, 0,0,0); m_pTargetBrep->Draw(TRUE); sm_GraphicsLoop();
          smgfx_SetLook(3,4, 0,0,1); m_pPseudoBrep->Draw(TRUE); sm_GraphicsLoop();
          smgfx_SetLook(5,6, 1,0,0); pCorner->Draw(); sm_GraphicsLoop();
          sm_GraphicsLoop();
        }
#endif // SM_DEBUG_CODE

      // Check for and deal with degenerate corner.
      SmBoolean bCornerIsDegen = pCorner->CheckAndFixDegeneracy();

      if ( ! bCornerIsDegen )
        {
          // If this is not a degenerate corner,
          // re-adjust topology per errors or user's requests
          // such as 'Set-back' or '3x2-Bevel with non-equal radii'
          // This can also adjust things automatically, such as in the 3x2
          // case, if two rails intersect the un-filleted edge in different
          // places, a new fillet vertex and edge will be inserted.

          if(SM_SUCCESS != pCorner->AdjustTopology())
            {
              // when AdjustTopology failed - note the error
              pCorner->GetName( sBuff, SM_TBLOCK_SIZE );
              NoteFilletError( SM_FILERR_VERTEX_PROBLEM, pCorner->GetCornerType(),
                               SM_FIL_FAILURE, sBuff, pCorner->GetFilletedVertex(),
                               NULL, NULL, // For now, we don't generally have just two surfaces.
                               NULL, NULL, // For now, we don't generally have just two edgeuses.
                               _T("Fillet Corner Error: Topology Adjustment Failure") );
              FILEXEC_CORNER_ERR(pCorner,_T("Topology Adjustment Failure"));
            }
        }

#ifdef SM_DEBUG_CODE
      if (iDebugLevel > 0 ) 
        {
          smos_sprintf(sBuff,_T("\nExec: After Adjusting corner # %ld :\n"), cornerIdx);
          pCorner->DumpLevel( iDebugLevel, sBuff);

          smgfx_Erase();
          smgfx_SetLook(1,2, 0,0,0); m_pTargetBrep->Draw(TRUE); sm_GraphicsLoop();
          smgfx_SetLook(3,4, 0,0,1); m_pPseudoBrep->Draw(TRUE); sm_GraphicsLoop();
          smgfx_SetLook(5,6, 1,0,0); pCorner->Draw(); sm_GraphicsLoop();
          sm_GraphicsLoop();
        }
#endif // SM_DEBUG_CODE

  } // end of loop over corners, adjusting topology for non-degenerate cases

#ifdef SM_DEBUG_CODE
  if (iDebugLevel > 0 ) 
    {
      smos_sprintf(sBuff,_T("%s"),_T("\nFilletExecutive: All Corners after Create and Adjust") );
      smos_WriteBuffer(sBuff);

      smgfx_Erase();
      smgfx_SetLook(1,2, 0,0,0); m_pTargetBrep->Draw(TRUE); sm_GraphicsLoop();
      smgfx_SetLook(3,4, 0,0,1); m_pPseudoBrep->Draw(TRUE); sm_GraphicsLoop();
      smgfx_SetLook(5,6, 1,0,0);
      for ( cornerIdx=0; cornerIdx<m_vFilletCorners.GetSize(); cornerIdx++ )
        {
          SmFilletCorner * pCorner = m_vFilletCorners[cornerIdx];
          pCorner->Draw(); sm_GraphicsLoop();
          sm_GraphicsLoop();
        }
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

  // Calculate the filletSurface geometry for each edge fillet
  for(solverIdx=0; solverIdx<m_vFilletSolvers.GetSize(); solverIdx++)
    {
      SmFilletSolver * pFilSolver = m_vFilletSolvers[solverIdx];

#ifdef SM_DEBUG_CODE
      if ( iDebugLevel > 0 ) 
        {
          ULONG lIdx = FindSolverIndex( pFilSolver );
          smos_sprintf(sBuff,_T("\nExec: Calculating fillet surface geom # %ld, [%4ld] :\n"), solverIdx, lIdx );
          smos_WriteBuffer(sBuff);

          smgfx_Erase();
          smgfx_SetLook(1,2, 0,0,1); m_pTargetBrep->Draw(TRUE) ; sm_GraphicsLoop();
          smgfx_SetLook(3,4, 0,0,1); m_pPseudoBrep->Draw(TRUE); sm_GraphicsLoop();
          smgfx_SetLook(5,6, 1,0,0); pFilSolver->Draw(); sm_GraphicsLoop();
          sm_GraphicsLoop();
        }
#endif // SM_DEBUG_CODE

      // Compute untrimmed filletSurface and railCurve geometry
      if(SM_SUCCESS != pFilSolver->CalcFilletGeom())
        {
          const SmEdge* pE = pFilSolver->GetEdgeuse(0)->GetEdge();
          GetFilletErrorInfo()->NoteError( pE, SM_FS_SURF_TRACING_FAILURE, SM_FILERR_EDGE_PROBLEM,
                                           _T("Fillet Surface Computation Failure") );
          FILEXEC_SOLVER_ERR(pFilSolver,_T("Fillet Surface Computation Failure"));
        }

#ifdef SM_DEBUG_CODE
      if (iDebugLevel > 0 ) 
        {
          smos_sprintf(sBuff,_T("\nExec: After CalcFilletGeom for FilletSolver # %ld :\n"), solverIdx);
          pFilSolver->DumpLevel( iDebugLevel, sBuff);

          smgfx_Erase();
          smgfx_SetLook(1,2, 0,0,1); m_pTargetBrep->Draw(TRUE) ; sm_GraphicsLoop();
          smgfx_SetLook(3,4, 0,0,1); m_pPseudoBrep->Draw(TRUE); sm_GraphicsLoop();
          smgfx_SetLook(5,6, 1,0,0); pFilSolver->Draw(); sm_GraphicsLoop();
          sm_GraphicsLoop();
        }

      // Inspect the radius function, and continuity at the ends of our geometry.
      SmVariableRadiusFS *pVRFS = SM_CAST_PTR( SmVariableRadiusFS, pFilSolver );
      if ( pVRFS  &&  iDebugLevel > 20 ) {
          SmFilletLaw *pFilLaw = SM_CAST_PTR( SmFilletLaw, pVRFS->GetFilletLaw() );
          SmExtent1d sLawIvl = pVRFS->GetFilletLawInterval();
          SmExtent1d sEdgeMap = pFilLaw->m_vEdgeMap;

          // static int nIters = 3;
          // double dFrac, dT1, dRad1;
          // for ( int ie=0; ie<=nIters; ie++ ) {
          //     dFrac = double(ie) / double(nIters);
          //     dT1 = sLawIvl.Evaluate( dFrac );
          //     dRad1 = pVRFS->GetFilletRadius( dT1 );
          // }

          // Let's look at the spine crvs.  Could also look at rails.
          SmTArray<SmFilletGeom*> sFGs;
          pFilSolver->GetFilletGeoms( sFGs );
          SmFilletGeom *pFG = sFGs[0];
          SmBSplineCurve* pSpine = pFG->GetCenterLineCurve();
          SmCurve* pRail0 = pFG->GetRail(0)->GetCurve();
          SmCurve* pRail1 = pFG->GetRail(1)->GetCurve();

          SmVector3d sSpinePV0[3], sRail0PV0[3], sRail1PV0[3];
          SmVector3d sSpinePV1[3], sRail0PV1[3], sRail1PV1[3];

          SmExtent1d sDomain = pSpine->GetNaturalInterval();
          pSpine->Evaluate( sDomain.GetMin(), 2, TRUE, sSpinePV0 );
          pSpine->Evaluate( sDomain.GetMax(), 2, TRUE, sSpinePV1 );

          sDomain = pRail0->GetNaturalInterval();
          pRail0->Evaluate( sDomain.GetMin(), 2, TRUE, sRail0PV0 );
          pRail0->Evaluate( sDomain.GetMax(), 2, TRUE, sRail0PV1 );

          sDomain = pRail1->GetNaturalInterval();
          pRail1->Evaluate( sDomain.GetMin(), 2, TRUE, sRail1PV0 );
          pRail1->Evaluate( sDomain.GetMax(), 2, TRUE, sRail1PV1 );

      }

#endif // SM_DEBUG_CODE

    } // end CalcFilletGeom loop for every edge being filleted

  // Fillet surfaces and rail curves exist now, not trimmed to anything.
  // For example, new fillet vertices that are supposed to lie in a face
  // of the target Brep might lie outside of the face.

#ifdef SM_DEBUG_CODE
  if (iDebugLevel > 0 ) 
    {
      smgfx_Erase();
      smgfx_SetLook(1,2, 0,0,1); m_pTargetBrep->Draw(TRUE) ; sm_GraphicsLoop();
      smgfx_SetLook(3,4, 0,0,1); m_pPseudoBrep->Draw(TRUE); sm_GraphicsLoop();

      // Draw corners first (big), then solvers.
      smgfx_SetLook(5,6, 1,0,0);
      for ( cornerIdx=0; cornerIdx<m_vFilletCorners.GetSize(); cornerIdx++ )
        {
          SmFilletCorner * pCorner = m_vFilletCorners[cornerIdx];
          pCorner->Draw(); sm_GraphicsLoop();
          sm_GraphicsLoop();
        }
      sm_GraphicsLoop();

      smgfx_SetLook(5,6, 1,0,1);
      for(solverIdx=0; solverIdx<m_vFilletSolvers.GetSize(); solverIdx++)
        {
          SmFilletSolver * pFilSolver = m_vFilletSolvers[solverIdx];

          pFilSolver->DumpLevel( iDebugLevel, _T("\nExec: All Solvers after CalcFilletGeom") );

          smgfx_ChangeColor(solverIdx != 0); pFilSolver->Draw(); sm_GraphicsLoop();
          sm_GraphicsLoop();
        }
    }
#endif // SM_DEBUG_CODE

  // for every m_vFilletCorner - Adjust tangent corners and recalculate geometry.
  for(cornerIdx=0; cornerIdx<m_vFilletCorners.GetSize(); cornerIdx++)
    {
      SmFilletCorner * pCorner = m_vFilletCorners[cornerIdx];

#ifdef SM_DEBUG_CODE
      if ( iDebugLevel > 0 ) 
        {
          ULONG lIdx = FindCornerIndex( pCorner );
          smos_sprintf(sBuff,_T("\nExec: Calc Corner edge geom, # %ld, [%4ld] :\n"), cornerIdx, lIdx );
          smos_WriteBuffer(sBuff);
          smos_sprintf(sBuff,_T("%s"),_T("   First AdjustTangentCorner, then calc remaining vert geom, then calc edge geom.\n") );
          smos_WriteBuffer(sBuff);
        }
#endif // SM_DEBUG_CODE

      // Check for further topology adjustments.
      // This is similar to AdjustTopology(), but the tests
      // require more geometry such as rails curves, etc.
      if(SM_SUCCESS != pCorner->AdjustTangentCorner())
        {
          // when AdjustTangentCorner failed - note the error
          pCorner->GetName( sBuff, SM_TBLOCK_SIZE );
          NoteFilletError( SM_FILERR_VERTEX_PROBLEM, pCorner->GetCornerType(),
                           SM_FIL_FAILURE, sBuff, pCorner->GetFilletedVertex(),
                           NULL, NULL, // For now, we don't generally have just two surfaces.
                           NULL, NULL, // For now, we don't generally have just two edgeuses.
                           _T("Fillet Corner Error: Topology Adjustment Failure") );
          FILEXEC_CORNER_ERR(pCorner,_T("Topology Adjustment Failure"));
        }

      // Calculate the remaining corner vertices for each corner(Phase 2)
      // Those vertices whose position depends upon the filletSurface shape
      //   such as vertices classified as SM_FV_FILLET_X_FILLET
      // Also catches those that have been adjusted.
      if(SM_SUCCESS != pCorner->CalcCornerVertGeom())
        {
          // when AdjustTopology failed - note the error
          pCorner->GetName( sBuff, SM_TBLOCK_SIZE );
          NoteFilletError( SM_FILERR_VERTEX_PROBLEM, pCorner->GetCornerType(),
                           SM_FIL_FAILURE, sBuff, pCorner->GetFilletedVertex(),
                           NULL, NULL, // For now, we don't generally have just two surfaces.
                           NULL, NULL, // For now, we don't generally have just two edgeuses.
                           _T("Fillet Corner Error: Vertex Computation Failure") );
          FILEXEC_CORNER_ERR(pCorner,_T("Vertex Computation Failure"));
        }

      // Calculate corner boundary (FilletSurface trim) curves --
      // some already done as side effects of CalcCornerVertGeom()
      // Also, again, some undone due to adjustments.
      //   [cbi532 probably handle it down in this call:]
      if(SM_SUCCESS != pCorner->CalcCornerEdgeGeom())
        {
          // when CalcCornerEdgeGeom failed - note the error
          pCorner->GetName( sBuff, SM_TBLOCK_SIZE );
          NoteFilletError( SM_FILERR_VERTEX_PROBLEM, pCorner->GetCornerType(),
                           SM_FIL_FAILURE, sBuff, pCorner->GetFilletedVertex(),
                           NULL, NULL, // For now, we don't generally have just two surfaces.
                           NULL, NULL, // For now, we don't generally have just two edgeuses.
                           _T("Fillet Corner Error: Corner Edge Computation Failure") );
          FILEXEC_CORNER_ERR(pCorner,_T("Edge Computation Failure"));
        }
#ifdef SM_DEBUG_CODE
      if ( iDebugLevel > 0 ) 
        {
          smos_sprintf(sBuff,_T("\nExec: After Calc Geom for FilletCorner # %ld :\n"), cornerIdx);
          pCorner->DumpLevel( iDebugLevel, sBuff);

          smgfx_Erase();
          smgfx_SetLook(1,2, 0,0,1); m_pTargetBrep->Draw(TRUE) ; sm_GraphicsLoop();
          smgfx_SetLook(3,4, 0,0,1); m_pPseudoBrep->Draw(TRUE); sm_GraphicsLoop();
          smgfx_SetLook(5,6, 1,0,0); pCorner->Draw(); sm_GraphicsLoop();
          sm_GraphicsLoop();
        }
#endif // SM_DEBUG_CODE
    } // end calc vert & edge geom for every FilletCorner

  // Trim rail curves by the rail ends
  // This is strictly within the fillet Solvers: no trimming to Target or Fillet Breps yet.
  for ( solverIdx=0; solverIdx<m_vFilletSolvers.GetSize(); solverIdx++ )
    {
      SmFilletSolver * pFilSolver = m_vFilletSolvers[solverIdx];
      if ( pFilSolver->TrimRails() != SM_SUCCESS )
        {
          const SmEdge* pE = pFilSolver->GetEdgeuse(0)->GetEdge();
          GetFilletErrorInfo()->NoteError( pE, SM_FV_NO_INT_RAIL_EU, SM_FILERR_EDGE_PROBLEM,
                                           _T("Fillet Surface Computation Failure: Unable to Trim Rail Curve") );
          FILEXEC_SOLVER_ERR(pFilSolver,_T("Unable to Trim Rail Curve"));
        }
#ifdef SM_DEBUG_CODE
      if ( iDebugLevel >= 1 ) 
        {
          smos_sprintf(sBuff,_T("\nExec: After TrimRails for FilletSolver # %ld :\n"), solverIdx);
          pFilSolver->DumpLevel( iDebugLevel, sBuff);

          smgfx_Erase();
          smgfx_SetLook(1,2, 0,0,1); m_pTargetBrep->Draw(TRUE) ; sm_GraphicsLoop();
          smgfx_SetLook(3,4, 0,0,1); m_pPseudoBrep->Draw(TRUE); sm_GraphicsLoop();
          smgfx_SetLook(5,6, 1,0,0); pFilSolver->Draw(); sm_GraphicsLoop();
          sm_GraphicsLoop();
        }
#endif // SM_DEBUG_CODE

    } // end iter m_vFilletSolvers

  // when asked - intersect every rail curve with its originalFace->boundary
  // Edges and Vertices to find if the rail curve 'rolled' over onto another
  // surface.  If so adjust the fillets as needed to roll onto other surfaces.
  if ( m_bDoClassification )
    {
      SmTArray<SmFilletCorner*> sAdjustedCorners; 

      // for every FilletSolver
      for(solverIdx=0; solverIdx<m_vFilletSolvers.GetSize(); solverIdx++)
        {
          SmFilletSolver * pFilSolver = m_vFilletSolvers[solverIdx];
          if ( pFilSolver->TestRollOver( sAdjustedCorners ) != SM_SUCCESS )
            {
              const SmEdge* pE = pFilSolver->GetEdgeuse(0)->GetEdge();
              GetFilletErrorInfo()->NoteError( pE, SM_FV_NO_INT_RAIL_EU, SM_FILERR_EDGE_PROBLEM,
                                               _T("Fillet Solver Error: Unable to Process Rollover") );
              FILEXEC_SOLVER_ERR(pFilSolver,_T("Unable to Process RollOver"));
            }
#ifdef SM_DEBUG_CODE
          if (iDebugLevel >= 1 )
            { 
              smos_sprintf(sBuff,_T("\nExec: After TestRollOver for FilletSolver # %ld :\n"), solverIdx);
              pFilSolver->DumpLevel( iDebugLevel, sBuff);
            }
#endif // SM_DEBUG_CODE

        } // end iter every to-be-filleted edgeuse

      // Recompute edges of sAdjustedCorners if necessary
      // Typically, these corners were adjusted because of rollover
      for(cornerIdx=0; cornerIdx<sAdjustedCorners.GetSize(); cornerIdx++)
        {
          SmFilletCorner * pCorner = sAdjustedCorners[cornerIdx];
          pCorner->ReInitializeCornerEdges();
          if(SM_SUCCESS != pCorner->CalcCornerEdgeGeom())
            {
              // when CalcCornerEdgeGeom failed - note the error
              pCorner->GetName( sBuff, SM_TBLOCK_SIZE );
              NoteFilletError( SM_FILERR_VERTEX_PROBLEM, pCorner->GetCornerType(),
                               SM_FIL_FAILURE, sBuff, pCorner->GetFilletedVertex(),
                               NULL, NULL, // For now, we don't generally have just two surfaces.
                               NULL, NULL, // For now, we don't generally have just two edgeuses.
                               _T("Fillet Corner Error: Edge Computation Failure After Rollover Adjustment") );
              FILEXEC_CORNER_ERR(pCorner,_T("Edge Computation Failure"));
            }

#ifdef SM_DEBUG_CODE
          if (iDebugLevel >= 1 ) 
            {
              smos_sprintf(sBuff,_T("\nExec: After 2nd EdgeGeom Calc for FilletCorner # %ld :\n"), cornerIdx);
              pCorner->DumpLevel( iDebugLevel, sBuff);

              smgfx_Erase();
              smgfx_SetLook(1,2, 0,0,1); m_pTargetBrep->Draw(TRUE) ; sm_GraphicsLoop();
              smgfx_SetLook(3,4, 0,0,1); m_pPseudoBrep->Draw(TRUE); sm_GraphicsLoop();
              smgfx_SetLook(5,6, 1,0,0); pCorner->Draw(); sm_GraphicsLoop();
              sm_GraphicsLoop();
            }
#endif // SM_DEBUG_CODE
        }
    } // end if m_bDoClassification

  // build the intermediate FilletBrep object,
  // used to hold new filletFace->shells
  m_pFilletBrep = new (m_crContext) SmBrep();
  NER( m_pFilletBrep );
  m_pFilletBrep->SetTolerance( m_pTargetBrep->GetTolerance() );
  m_pFilletBrep->m_bEditingEnabled = TRUE;

  // See if the corners need to have a corner-surface patch created - build any faces in m_pFilletBrep
  for(cornerIdx=0; cornerIdx<m_vFilletCorners.GetSize(); cornerIdx++)
    {
      SmFilletCorner * pCorner = m_vFilletCorners[cornerIdx];

      // Build Corner->FilletSurface and store that in pCorner->m_vSurfaces
      if(SM_SUCCESS != pCorner->CalcCornerGeom())
        {
          // when CalcCornerGeom() failed - Note FilletError
          pCorner->GetName( sBuff, SM_TBLOCK_SIZE );
          NoteFilletError( SM_FILERR_VERTEX_PROBLEM, pCorner->GetCornerType(),
                           SM_FIL_FAILURE, sBuff, pCorner->GetFilletedVertex(),
                           NULL, NULL, // For now, we don't generally have just two surfaces.
                           NULL, NULL, // For now, we don't generally have just two edgeuses.
                           _T("Fillet Corner Error: Corner Geometry Computation Failure") );
          FILEXEC_CORNER_ERR(pCorner,_T("Corner Geometry Computation Failure"));
        }
      
      // Create Face (and Edges and Verts) in m_pFilletBrep from this m_pFilletSurface with call to pBrep->MakeFaceWithCurves()
      //if (pCorner->MakeFaceBrep(m_pFilletBrep) != SM_SUCCESS) 
      //  {
      //    FILEXEC_CORNER_ERR(pCorner,_T("Corner Face Creation Error"));
      //  }
#ifdef SM_DEBUG_CODE
      if (iDebugLevel >= 1 ) 
        {
          smos_sprintf(sBuff,_T("\nExec: After Surface Calc for FilletCorner # %ld :\n"), cornerIdx);
          pCorner->DumpLevel( iDebugLevel, sBuff);
          SM_DUMP_AND_ASSERT2_VALID(m_pFilletBrep) ;
          SM_DUMP(m_pPseudoBrep) ;

          SM_PTR_ARRAY(sFilletBrepFaces, SmFace, 32) ;
          SM_PTR_ARRAY(sPseudoBrepFaces, SmFace, 32) ;
          m_pFilletBrep->GetFaces(sFilletBrepFaces) ;
          m_pPseudoBrep->GetFaces(sPseudoBrepFaces) ;

          smgfx_Erase();
          smgfx_SetLook(1,1, 0,0,1); m_pTargetBrep->Draw(TRUE) ; sm_GraphicsLoop();
          smgfx_SetLook(2,1, 0,1,0); if(m_pFilletBrep) m_pFilletBrep->Draw(TRUE) ; sm_GraphicsLoop();
          smgfx_SetLook(3,4, 1,0,0); pCorner->Draw(); sm_GraphicsLoop();
          smgfx_SetLook(1,2, 0,1,1); for(idbg=0;idbg<sFilletBrepFaces.GetSize();idbg++)
                                       { if(sFilletBrepFaces[idbg]) sFilletBrepFaces[idbg]->DrawUV() ; sm_GraphicsLoop() ; }
          smgfx_SetLook(1,2, 1,0,1); for(idbg=0;idbg<sPseudoBrepFaces.GetSize();idbg++)
                                       { if(sPseudoBrepFaces[idbg]) sPseudoBrepFaces[idbg]->DrawUV() ; sm_GraphicsLoop() ; }
          sm_GraphicsLoop();
        }
#endif // SM_DEBUG_CODE

    } // end iter every m_vFilletCorners member
 
  // Create face topology with geometry in m_pFilletBrep for each FilletSolver - (FilletSolver: Data needed to fillet one target BrepEdge)
  for ( solverIdx=0; solverIdx<m_vFilletSolvers.GetSize(); solverIdx++ )
    {
      SmFilletSolver * pFilSolver       = m_vFilletSolvers[solverIdx];
      double           dThisApproxTol3d = pFilSolver->GetThisApproxTol3d();

      // Note: Generally, a Brep's tolerance is meant to be a lower bound on
      // all of its topologies' tolerances, not an upper bound. [B308]
      // However, m_pFilletBrep starts out with no topology at all, it's created here.
      // This is the only way we have to communicate the tol specified in the FilletSolver
      // down through MakeFaceBrep() to SmBrep::MakeFaceWithCurves().  [B551]
      if(m_pFilletBrep->GetTolerance() < dThisApproxTol3d)
        {
#ifdef SM_USE_NEWTOL                              
          SM_NEWTOL_LINE m_pFilletBrep->SetZoneTol3d( dThisApproxTol3d );  // GWC: this is wrong - approx and neigh tol are different concepts
#else // SM_USE_OLDTOL
          SM_OLDTOL_LINE if(m_pFilletBrep->GetTolerance() < dThisApproxTol3d)
          SM_OLDTOL_LINE    { m_pFilletBrep->SetTolerance( dThisApproxTol3d ); }
#endif // SM_USE_OLDTOL       
        }
 
      // for every FilletSolver->FilletGeom (FilletGeom contains geom for a single FilletSurface: FilletSurf, rails, centerline, and more)
      for ( i=0; i<pFilSolver->m_vFilletGeoms.GetSize(); i++ )
        {
          SmFilletGeom * pFilletGeom = pFilSolver->m_vFilletGeoms[i];

          // Create Face (and Edges and Verts) in m_pFilletBrep from this m_pFilletSurface with call to pBrep->MakeFaceWithCurves()
          if ( pFilletGeom->MakeFaceBrep(m_pFilletBrep) != SM_SUCCESS )
            {
              const SmEdge* pE = pFilSolver->GetEdgeuse(0)->GetEdge();
              GetFilletErrorInfo()->NoteError( pE, SM_FIL_FAILURE, SM_FILERR_EDGE_PROBLEM,
                                              _T("Fillet Surface Computation Failure: Fillet Face Creation Error") );
              FILEXEC_SOLVER_ERR( pFilSolver,_T("Fillet Face Creation Error") );
            }
        }
#ifdef SM_DEBUG_CODE
      if (iDebugLevel > 0 ) 
        {
          smos_sprintf(sBuff,_T("\nExec: After MakeFaceBrep for FilletSolver # %ld :\n"), solverIdx);
          pFilSolver->DumpLevel( iDebugLevel, sBuff);
          SM_DUMP_AND_ASSERT_VALID(m_pFilletBrep) ;
          SM_DUMP(m_pPseudoBrep) ;

          SM_PTR_ARRAY(sFilletBrepFaces, SmFace, 32) ;
          SM_PTR_ARRAY(sPseudoBrepFaces, SmFace, 32) ;
          m_pFilletBrep->GetFaces(sFilletBrepFaces) ;
          m_pPseudoBrep->GetFaces(sPseudoBrepFaces) ;

          smgfx_Erase();
          smgfx_SetLook(1,1, 0,0,1); m_pTargetBrep->Draw(TRUE) ; sm_GraphicsLoop();
          smgfx_SetLook(2,1, 0,1,0); if(m_pFilletBrep) m_pFilletBrep->Draw(TRUE) ; sm_GraphicsLoop();
          smgfx_SetLook(2,1, 0,1,0); pFilSolver->Draw(); sm_GraphicsLoop();
          smgfx_SetLook(1,2, 0,1,1); for(idbg=0;idbg<sFilletBrepFaces.GetSize();idbg++)
                                       { if(sFilletBrepFaces[idbg]) sFilletBrepFaces[idbg]->DrawUV() ; sm_GraphicsLoop() ; }
          smgfx_SetLook(1,2, 1,0,1); for(idbg=0;idbg<sPseudoBrepFaces.GetSize();idbg++)
                                       { if(sPseudoBrepFaces[idbg]) sPseudoBrepFaces[idbg]->DrawUV() ; sm_GraphicsLoop() ; }
          sm_GraphicsLoop();
        }
#endif // SM_DEBUG_CODE

    } // end iter every to-be-filleted edge

#ifdef SM_DEBUG_CODE
  if ( iDebugLevel > 0 ) 
  // draw original TargetBrep and FilletBrep with all the new FilletFaces
    {
      ULONG ii ;
      m_pFilletBrep->Dump();
      m_pFilletBrep->ValidatePointers();

      SmTArray<SmSurface*> sSurfaces ;
      m_pFilletBrep->GetSurfaces(sSurfaces) ;

      smgfx_Erase();
      smgfx_SetLook(1,1, 0,0,1); m_pTargetBrep->Draw(TRUE) ; sm_GraphicsLoop();
      smgfx_SetLook(1,1, 0,1,0); m_pFilletBrep->Draw(TRUE) ; sm_GraphicsLoop();
      smgfx_SetLook(1,1, 0,1,1); for(ii=0;ii<sSurfaces.GetSize();ii++) 
                                  { sSurfaces[ii]->DrawUV(8,8,FALSE,NULL,TRUE); sm_GraphicsLoop() ; }
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

#ifdef VALIDATE_TOPOLOGY
  m_pFilletBrep->ValidatePointers();
#endif // VALIDATE_TOPOLOGY

  // Add m_pAttributeE attribute to every m_pFilletBrep->Edge place 
  SmTArray<SmEdge*> sFilletEdges;
  m_pFilletBrep->GetEdges( sFilletEdges );
  ULONG ii;
  for ( ii=0; ii<sFilletEdges.GetSize(); ii++ )
    {
      sFilletEdges[ii]->AddAttribute( m_pAttributeE );
    }

  // stitch the fillet faces along edges where they coincide
  if (1)
    {
      ULONG lNumStitched, lNumLamina;
      double dMaxVGap, dMaxEGap;
      if(SM_SUCCESS != m_pFilletBrep->StitchFaces
                         (m_pFilletBrep->GetTolerance(), // in : max 3d distance between coincident vertex and edge pairs
                          lNumStitched,                      // out: number of edges stitched
                          lNumLamina,                    // out: number of lamina edges remaining after stitch
                          dMaxVGap,                      // out: max gap found between coincident vertices considered for glueing
                          dMaxEGap ))                    // out: max gap found between coincident edges    considered for glueing
                                                         // in : only stitch topology connected to these faces, NULL=all faces
                                                         //      default:[NULL]
                                                         // in : if true this allows formation of spine edges. See c below.
                                                         //      default:[FALSE]
                                                         // NOTE: some coincident vertex and edge pairs do not get glued due to
                                                         //       a. gaps exceeding tolerances
                                                         //       b. a failure within the glue edge function
                                                         //       c. not being lamina when m_bMakingManifoldSolid == TRUE
        {
          GetFilletErrorInfo()->NoteError( SM_FILERR_BAD_MERGE,
                                           _T("Fillet Stitching Failure: Error Stitching Fillets Together") );
          FILEXEC_ERR(_T("Error Stitching Fillets Together"));
        }

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
      // validate and dump m_pFilletBrep, draw m_pTargetBrep and m_pFilletBrep
      if ( bDebugMe && iDebugLevel > 0 )
        {
          // validate and dump FilletBrep
          smos_sprintf(sBuff,_T("\nStitch Faces - # Stitched Edges = %ld, # Lamina = %ld,  Max V Gap = %le, Max E Gap = %le\n"),
                     lNumStitched,lNumLamina,dMaxVGap,dMaxEGap);
          smos_WriteBuffer(sBuff);

          ULONG di ;
          SmTArray<SmFace *> sFaces ;      m_pFilletBrep->GetFaces(sFaces) ; 
          SmTArray<SmEdge *> sEdges ;      m_pFilletBrep->GetEdges(sEdges) ;
          SmTArray<SmVertex *> sVertices ; m_pFilletBrep->GetVertices(sVertices) ;
          SM_DUMP_AND_ASSERT_VALID(m_pFilletBrep);

          // draw targetBrep and filletBrep
          smgfx_Erase();
          smgfx_SetLook(1,2, 0,0,0) ; m_pTargetBrep->Draw(TRUE); sm_GraphicsLoop();
          smgfx_SetLook(3,4, 0,1,1) ; m_pFilletBrep->Draw(TRUE); sm_GraphicsLoop();
          smgfx_SetLook(1,2, 0,1,0) ; for(di=0;di<sFaces.GetSize();di++)
                                        { if(sFaces[di]) sFaces[di]->DrawUV() ; sm_GraphicsLoop(); }
          smgfx_SetLook(3,4, 1,0,1) ; for(di=0;di<sEdges.GetSize();di++)
                                        { if(sEdges[di]) sEdges[di]->DrawParams() ; sm_GraphicsLoop(); }
          smgfx_SetLook(3,4, 1,0,0) ; for(di=0;di<sVertices.GetSize();di++)
                                        { if(sVertices[di]) sVertices[di]->Draw() ; sm_GraphicsLoop(); }
          sm_GraphicsLoop();
        }
#endif // SM_DEBUG_CODE
    } // end m_pFilletBrep->StitchFaces() Scope

  if ( pOptFilletErrorInfo != NULL )
    {
      *pOptFilletErrorInfo = *( GetFilletErrorInfo() );
    }

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
      // validate and dump m_pFilletBrep, draw m_pTargetBrep and m_pFilletBrep
      if ( bDebugMe && iDebugLevel > 0 )
        {
          // validate and dump FilletBrep
          SM_DUMP_AND_ASSERT_VALID(m_pFilletBrep);
          smos_sprintf(sBuff,_T("%s"),_T("\nCreateFilletBrep All Done\n"));
          smos_WriteBuffer(sBuff);

          // draw targetBrep and filletBrep
          smgfx_Erase();
          smgfx_SetLook( 1,2, 0,0,0); m_pTargetBrep->Draw(TRUE); sm_GraphicsLoop();
          smgfx_SetLook( 3,4, 0,0,1); m_pFilletBrep->Draw(TRUE); sm_GraphicsLoop();
          sm_GraphicsLoop();
        }
#endif // SM_DEBUG_CODE

  // all done
  return SM_SUCCESS;

} // end SmFilletExecutive::CreateFilletBrep

/*******************************************************************//**
PURPOSE: Do a piecewise merge of the fillet brep into the original brep.

NOTES: increments unlocked mark value
***********************************************************************/
SmStatus SmFilletExecutive::DoPiecewiseMerge()
{
  // locals
  ULONG ii, jj, kk;
  const SmContext * cpContext      = m_pTargetBrep->GetContext();
  SmBoolean         bManifoldSolid = m_pTargetBrep->IsManifoldSolid() ;
  SmTArray<SmRegion*> sRegions;

  // get list of all current topological edges and vertices - these will remain after the fillets are done
  SM_PTR_ARRAY(sOrigTopoEdges, SmEdge, 16) ;
  SM_PTR_ARRAY(sOrigTopoVerts, SmVertex, 16) ;
  m_pTargetBrep->GetTopologicalEdges(sOrigTopoEdges) ;
  m_pTargetBrep->GetTopologicalVertices(sOrigTopoVerts) ;

  // for every Original m_pTarget->Region - add SM_AI_FILLET_REGIONMAP attribute
  m_pTargetBrep->GetRegions(sRegions);
  for (ii=0; ii<sRegions.GetSize(); ii++) 
    {
      SmRegion * pR = sRegions[ii];
      SmLongAttribute *pAtt = new (*cpContext) SmLongAttribute(SM_AI_FILLET_REGIONMAP,pR->IsVoid(),SM_AB_REFERENCE);
      pR->AddAttribute(pAtt);
    }

  // Set up non-intersecting pairs into a relationship
  // This is a heuristic to speed things up.  It is possible
  // that this will fail in some cases where surfaces self-intersect
    {
      SmRelation<SmSurface,SmSurface> sNoSSI(m_crContext);
      SmTArray<SmFilletGeom*>         sFilletGeoms;
      SmTArray<SmFace*>               sVertFaces;

      // for every FilletSolver (edge being filleted) - relate FilletSurf/OrigSurf across each rail and add SM_AI_FILLET_EDGEMAP attribute to each FilletSurf
      for (ii=0; ii<m_vFilletSolvers.GetSize(); ii++)
        {
          SmFilletSolver * pFilletSolver = m_vFilletSolvers[ii];
          pFilletSolver->GetFilletGeoms(sFilletGeoms);

          // for every FilletSolver->FilletGeom
          for (jj=0; jj<sFilletGeoms.GetSize(); jj++)
            {
              SmFilletGeom * pFilletGeom = sFilletGeoms[jj];

              // Don't use cliff side patch for classification purposes
              if (pFilletGeom->GetFilletGeomType() == SM_FG_CLIFF_SIDE_PATCH) 
                { continue; }

              SmEdgeuse * pEU    = pFilletSolver->GetEdgeuse(0); NER(pEU);
              SmEdge    * pOrigE = pEU->GetEdge();               NER(pOrigE);

              // Get flag to see if fillet is inside of SOLID or outside of SOLID.
              SmShell  * pShell        = pEU->GetShell(); NER(pShell);
              SmRegion * pRegion       = pShell->GetRegion();
              SmBoolean  bInsideFillet = pRegion->IsVoid() ; // TRUE  = Adding to the inside
                                                            // FALSE = Adding to the outside
              // for both rails
              for (kk=0; kk<2; kk++)
                {
                  SmFilletEdge * pFE         = pFilletGeom->GetRail(kk);
                  SmSurface    * pFilletSurf = pFilletGeom->GetFilletSurface();
                  SmFace       * pOrigFace   = pFE->GetOriginalFace();
                  SmSurface    * pOrigSurf   = pOrigFace ? pOrigFace->GetSurface() : NULL ;

                  // skip rails without OrigFaces or OrigSurfaces (gwc: does this happen?)
                  if(   !pOrigFace   
                     || !pOrigSurf
                     || !pFilletSurf) 
                    { continue; }

                    // relate FilletSurf to OrigSurf and add SM_AI_FILLET_EDGEMAP Attribute to FilletSurf
                  SmLongAttribute *pAtt = new (*cpContext) SmLongAttribute(SM_AI_FILLET_EDGEMAP,bInsideFillet,SM_AB_REFERENCE);
                  pFilletSurf->AddAttribute(pAtt);
                  sNoSSI.RelatePair(pFilletSurf,pOrigSurf);
                }
            }
        } // end iter every FilletSolver - relating Edge->FilletSurf/OrigSurf across each rail and add SM_AI_FILLET_EDGEMAP attribute to FilletSurf

      // for every FilletCorner (vertex being filleted) - relate Vertex->FilletSurf/OrigSurf across each and add SM_AI_FILLET_EDGEMAP attribute to related OrigSurf
      for (ii=0; ii<m_vFilletCorners.GetSize(); ii++)
        {
          SmFilletCorner * pFilletCorner = m_vFilletCorners[ii];

          // 
          SER(pFilletCorner->InsertIntersectionTopology());

          if (pFilletCorner->m_bBlending)
            {
              pFilletCorner->m_cpVertex->GetFaces(sVertFaces);

              for (jj=0; jj<sVertFaces.GetSize(); jj++)
                {
                  SmFace    * pVF       = sVertFaces[jj];
                  SmSurface * pOrigSurf = pVF->GetSurface();

                  // skip fillet vertices not on an OrigFace
                  if (!pOrigSurf) 
                    { continue; }

                  for (kk=0; kk<pFilletCorner->m_vSurfaces.GetSize(); kk++)
                    {
                      SmSurface *pFBSurface = pFilletCorner->m_vSurfaces[kk];
                      sNoSSI.RelatePair(pFBSurface,pOrigSurf);
                      SmLongAttribute *pAtt = new (*cpContext) SmLongAttribute(SM_AI_FILLET_VERTMAP,0,SM_AB_REFERENCE);
                      pFBSurface->AddAttribute(pAtt);
                    }
                }
            } // end iter every FilletCorner (vertex being filleted) - adding SM_AI_FILLET_VERTMAP attirbute

          // Merge each edge of corner into originating face
          for (jj=0; jj<pFilletCorner->m_vEdges.GetSize(); jj++)
            {
              SmFilletEdge * pFE = pFilletCorner->m_vEdges[jj];
              if (!pFE) continue;
              SmFace *pFace = pFE->GetOriginalFace();
              if (!pFace) continue;
              SmSurface *pOrigSurf = pFace->GetSurface();
              if (!pOrigSurf) continue;
              for (kk=0; kk<pFilletCorner->m_vSurfaces.GetSize(); kk++)
                {
                  SmSurface *pFBSurface = pFilletCorner->m_vSurfaces[kk];
                  sNoSSI.RelatePair(pFBSurface,pOrigSurf);
                }
            }
        }

      SmBrep *pResult;
      m_vTI.SetThisApproxTol3d(m_vTI.GetOtherBrep()->GetTolerance());
      SER(PiecewiseMerge(&sNoSSI,TRUE,TRUE,pResult));

    } // end scope - needed to clean up sNoSSI before destruction
      //             Set up non-intersecting pairs into a relationship
      //             This is a heuristic to speed things up.  It is possible
      //             that this will fail in some cases where surfaces self-intersect

  SmTArray<SmFace*> sFaces;
  m_pTargetBrep->GetFaces(sFaces);

  // increment and lock an unlocked mark
  SmNewMarkAndLock sMarkLock( m_pTargetBrep->GetContext(), SM_MT_ALLMARKS) ; // increment and lock any unlocked mark
  SmMarkType eMarkType = sMarkLock.GetMarkType() ;

  for (ii=0; ii<sFaces.GetSize(); ii++)
    {
      SmFace          * pF    = sFaces[ii]; NER(pF);
      SmSurface       * pSurf = pF->GetSurface(); NER(pSurf);
      SmLongAttribute * pAtt  = (SmLongAttribute*)pSurf->FindAttribute(SM_AI_FILLET_EDGEMAP);
      if (!pAtt) { continue; }
      SmVector3d sNorm;
      SER(pSurf->EvaluateNormal(pF->GetUVDomain().Evaluate(0.5,0.5),TRUE,TRUE,sNorm));
      SmPoint3d sPnt, sDU, sDV, sDUV, sDUU, sDVV;
      SER(pSurf->Evaluate2ndDerivatives(pF->GetUVDomain().Evaluate(0.5,0.5),TRUE,TRUE,
                                        sPnt,sDU,sDV,sDUV,sDUU,sDVV));

      SmBoolean   bInsideFillet = pAtt->GetValue();
      SmFaceuse * pFU           = pF->GetUpwardFaceuse();

      // See if the top Faceuse on the inside of the fillet
      if (sDVV.Length() < SM_EFF_ZERO) 
        { continue; }

      if (sNorm.Dot(sDVV) > 0.0) 
        { pFU = pFU->GetMate(); }

      SmRegion *pReg = pFU->GetShell()->GetRegion();
      if (pFU->GetMate()->GetShell()->GetRegion() == pReg) 
        { continue; }

      if (pReg->IsMarked(eMarkType)) 
        { continue; }

      SmLongAttribute *pRegAtt = (SmLongAttribute*)pReg->FindAttribute(SM_AI_FILLET_REGIONMAP);
      SmBoolean bIsVoid = pRegAtt->GetValue();
      bInsideFillet =  (bIsVoid) ? FALSE : TRUE ;

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
      if (bDebugMe) 
        {
          smgfx_Erase();
          smgfx_SetLook(1,2, 0,0,1); pFU->GetFace()->Draw(); sm_GraphicsLoop();
          smgfx_SetLook(1,2, 0,0,0); pReg->Draw(); sm_GraphicsLoop();
          sm_GraphicsLoop();
        }
#endif // SM_DEBUG_CODE

      pReg->Mark(eMarkType);
      if (bInsideFillet) { pReg->SetIsVoid(TRUE); }
      else               { pReg->SetIsVoid(FALSE); }
    }

  // Now clean up attributes
  for(ii=0;ii<sFaces.GetSize();ii++) 
    {
      SmFace          * pF    = sFaces[ii]; NER(pF);
      SmSurface       * pSurf = pF->GetSurface(); NER(pSurf);

      SmLongAttribute * pAtt  = (SmLongAttribute*)pSurf->FindAttribute(SM_AI_FILLET_EDGEMAP);
      if (pAtt) { pSurf->RemoveAttribute(pAtt); }

      pAtt = (SmLongAttribute*)pSurf->FindAttribute(SM_AI_FILLET_VERTMAP);
      if (pAtt) { pSurf->RemoveAttribute(pAtt); }
    }

  // Find solid regions and make manifold using them
  SmTArray<SmRegion*> sKeepRegions;
  m_pTargetBrep->GetRegions(sRegions) ;

  for(ii=0;ii<sRegions.GetSize();ii++) 
    {
      SmRegion *pR = sRegions[ii];
      if (!pR->IsVoid()) 
        {
          if (pR != m_pTargetBrep->GetInfiniteRegion()) 
            { sKeepRegions.Add(pR); }
        }
    }

  if (sKeepRegions.GetSize() > 0 || !bManifoldSolid) 
    {
      SER(m_pTargetBrep->MakeManifold(&sKeepRegions,FALSE));

      // remove any new Topological Edges and Vertices
      SER(m_pTargetBrep->RemoveTopologicalEdgesAndVertices(&sOrigTopoEdges, &sOrigTopoVerts)) ;  // gwc: added 
    }

  // clean up 
  SM_ASSERT(m_pFilletBrep != NULL) ; delete m_pFilletBrep ;
  m_pFilletBrep = NULL;

  // all done
  return SM_SUCCESS;

} // end SmFilletExecutive::DoPiecewiseMerge

/*******************************************************************//**
PURPOSE: Do edge & corner filleting per users specification. Creates
    m_pFilletBrep fillet-brep first, then merges the results into 
    m_pTargetBrep original brep.
    
NOTES: Stitching operations will be performed after fillet surfaces are created.
       increments unlocked mark value
***********************************************************************/
SmStatus SmFilletExecutive::DoFilleting
  ( SmFilletErrorInfo * pOptFilletErrorInfo )
{
  SmTemporaryChangeValue< SmBoolean > sDoingBool( SM_CONST_CAST(SmContext*, &m_crContext)->GetDoingBooleanRef(), TRUE );

  // remember that we're using the SmTopologyIntersector and the SmCurveClassification code for fillets
  m_vTI.m_bDoingFillets = TRUE ;

  // Create m_pFilletBrep - A shell Brep containing the new Fillet Faces, 
  //  Edges, and Vertices created by filleting m_pTargetBrep Edges and Corners
  if(SM_SUCCESS != CreateFilletBrep( pOptFilletErrorInfo ))
    {
      // Note, do not NoteFilletError here, that has presumably
      // been done, at a point where they knew more about it
      // than we do here.
      FILEXEC_ERR(_T("Error Creating Fillet Brep"));
    }

  // Another failure check.  [B308]
  if(m_pFilletBrep->GetNumFaces() < 1)
    {
      // Again, do not NoteFilletError here.
      FILEXEC_ERR(_T("Error Creating Fillet Brep"));
    }

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  TCHAR sBuff[SM_TBLOCK_SIZE];
  int iDebugLevel = DebugLevel();
  if ( bDebugMe || iDebugLevel > 0 ) 
    {
      smos_sprintf(sBuff,_T("%s"),_T("\nFillet: After CreateFilletBrep()\n"));
      smos_WriteBuffer(sBuff);

      SM_DUMP_AND_ASSERT_VALID(m_pTargetBrep) ;
      SM_DUMP_AND_ASSERT_VALID(m_pFilletBrep) ;

      ULONG  di ;
      double dCH = .1 ; 
      SmTArray<SmSurface *> sTgtSurfs, sFltSurfs ;
      m_pTargetBrep->GetSurfaces(sTgtSurfs) ;
      m_pFilletBrep->GetSurfaces(sFltSurfs) ;

      smgfx_Erase();
      dCH = smgfx_SetLook( 1,2, 0,0,1, .001 ); m_pTargetBrep->Draw(TRUE); sm_GraphicsLoop();
      smgfx_SetLook( 2,4, 0,1,0, .001 ); m_pFilletBrep->Draw(TRUE); sm_GraphicsLoop();
      smgfx_SetChordHeight(dCH) ;
      for(di=0;di<sTgtSurfs.GetSize();di++)
        { if(sTgtSurfs[di]) 
          { SM_ASSERT_VALID(sTgtSurfs[di]) ;
            smgfx_SetLook( 1,2, 0,1,1) ; sTgtSurfs[di]->DrawUV(8,8,FALSE,NULL,TRUE) ; sm_GraphicsLoop() ;
            if(sTgtSurfs[di]->GetOwner()) 
              { smgfx_SetLook(1,2, 0,0,0) ; ((SmFace *)sTgtSurfs[di]->GetFace())->Draw(SM_DM_CROSSHATCH,9,9) ; sm_GraphicsLoop() ; }
        } }
      for(di=0;di<sFltSurfs.GetSize();di++)
        { if(sFltSurfs[di]) 
          { SM_ASSERT_VALID(sFltSurfs[di]) ;
            smgfx_SetLook( 1,2, 0,1,.5) ; sFltSurfs[di]->DrawUV(8,8,FALSE,NULL,TRUE) ; sm_GraphicsLoop() ;
            if(sFltSurfs[di]->GetOwner()) 
              { smgfx_SetLook(1,2, 0,0,0) ; ((SmFace *)sFltSurfs[di]->GetFace())->Draw(SM_DM_CROSSHATCH,9,9) ; sm_GraphicsLoop() ; }
        } }
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

  // Shrink the geometry in the new m_pFilletBrep.
  // The fillet surfaces are made way long, and we can get rid of lots of control points.
  // Also, a variable-radius function can go negative out there.
  //
  // But, shrinking everything can cause minor problems, [Fillet tests 410, 412]
  // so shrink only the main fillet surfaces, which are on the first
  // FilletGeoms in each solver.
  //   m_pFilletBrep->ShrinkGeometry();
  //
  // Shrink the first filletGeom in each Solver:
  ULONG ii, jj;
  for ( ii=0; ii<m_vFilletSolvers.GetSize(); ii++ )
    {
      SmFilletSolver * pFilSolver = m_vFilletSolvers[ii];

      if ( pFilSolver->m_vFilletGeoms.GetSize() > 0 )
        {
          SmFilletGeom * pFilletGeom    = pFilSolver->m_vFilletGeoms[0];
          SmSurface    * pFilletSurface = pFilletGeom->m_pFilletSurface;
          SmObject     * pOwner         = pFilletSurface->GetOwner();
          SmFace       * pFace          = SM_CAST_PTR( SmFace, pOwner );
          if ( pFace != NULL )
            {
              pFace->ShrinkGeometry();  // in m_pFilletBrep
            }
        } // end FilletSolver->FilletGeom[0] existence check
    } // end iter every FilletSolver, shrinking new fillet Surfaces

#ifdef SM_DEBUG_CODE
  if ( bDebugMe || iDebugLevel > 0 ) 
    {
      smos_sprintf(sBuff,_T("%s"),_T("\nFillet: After ShrinkGeometry()\n"));
      smos_WriteBuffer(sBuff);

      SM_DUMP_AND_ASSERT_VALID(m_pTargetBrep) ;
      SM_DUMP_AND_ASSERT_VALID(m_pFilletBrep) ;

      ULONG  di ;
      double dCH = .1 ; 

      SmTArray<SmSurface *> sTgtSurfs, sFltSurfs ;
      m_pTargetBrep->GetSurfaces(sTgtSurfs) ;
      m_pFilletBrep->GetSurfaces(sFltSurfs) ;

      smgfx_Erase();
      dCH = smgfx_SetLook( 1,2, 0,0,1, .001 ); m_pTargetBrep->Draw(TRUE); sm_GraphicsLoop();
      smgfx_SetLook( 2,4, 0,1,0, .001 ); m_pFilletBrep->Draw(TRUE); sm_GraphicsLoop();
      smgfx_SetChordHeight(dCH) ;
      for(di=0;di<sTgtSurfs.GetSize();di++)
        { if(sTgtSurfs[di]) 
          { SM_ASSERT_VALID(sTgtSurfs[di]) ;
            smgfx_SetLook( 1,2, 0,1,1) ; sTgtSurfs[di]->DrawUV(8,8,FALSE,NULL,TRUE) ; sm_GraphicsLoop() ;
            if(sTgtSurfs[di]->GetOwner()) 
              { smgfx_SetLook(1,2, 0,0,0) ; ((SmFace *)sTgtSurfs[di]->GetFace())->Draw(SM_DM_CROSSHATCH,9,9) ; sm_GraphicsLoop() ; }
        } }
      for(di=0;di<sFltSurfs.GetSize();di++)
        { if(sFltSurfs[di]) 
          { SM_ASSERT_VALID(sFltSurfs[di]) ;
            smgfx_SetLook( 1,2, 0,1,.5) ; sFltSurfs[di]->DrawUV(8,8,FALSE,NULL,TRUE) ; sm_GraphicsLoop() ;
            if(sFltSurfs[di]->GetOwner()) 
              { smgfx_SetLook(1,2, 0,0,0) ; ((SmFace *)sFltSurfs[di]->GetFace())->Draw(SM_DM_CROSSHATCH,9,9) ; sm_GraphicsLoop() ; }
        } }
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

  SmTemporaryChangeValue<SmBoolean> sChange(m_pTargetBrep->m_bEditingEnabled,TRUE);

  // load SmMerge state parameters

// Remove Composites
// #ifndef SM_NO_COMPOSITES
//   m_pTargetBrep->m_bMakeComposites = TRUE;
// #endif // SM_NO_COMPOSITES
  
  m_vTI.SetPrimaryBrep(m_pTargetBrep);
  m_vTI.SetOtherBrep(m_pFilletBrep);

  // Merge the m_pFilletBrep geometry into the original m_pTargetBrep one face at a time
  if ( m_bDoPiecewiseMerge )
    {
      SER( DoPiecewiseMerge() ); // note: increments unlocked mark value

      if ( pOptFilletErrorInfo != NULL )
        {
          *pOptFilletErrorInfo = *( GetFilletErrorInfo() );
        }
      return SM_SUCCESS;
    } // end m_bDoPiecewiseMerge == TRUE branch

  // Generate map From FilletSolver and FilletCorner FilletEdges stored in m_pPseudoBrep
  // to m_pFilletBrep->Edges by finding coincident mid-Points.
  SER(SetFilletBrepEdgeMap());

  SmTArray<SmFilletGeom*> sFilletGeoms;

#ifdef SM_DEBUG_CODE
  if (bDebugMe || iDebugLevel > 0 ) 
    {
      SM_DUMP_AND_ASSERT_VALID(m_pTargetBrep) ;
      SM_DUMP_AND_ASSERT_VALID(m_pFilletBrep) ;

      smgfx_Erase();
      double dCH = smgfx_SetLook( 1,0, 0,0,1, .0001); m_pTargetBrep->Draw(TRUE); sm_GraphicsLoop();
      smgfx_SetLook( 2,4, 0,1,0, .0001 ); m_pFilletBrep->Draw(TRUE); sm_GraphicsLoop();
      smgfx_SetLook( 3,5, 1,0,1 ); m_vTI.Draw(3,7, 5,9); sm_GraphicsLoop();
      smgfx_SetChordHeight(dCH) ; 
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

  // when asked - insert FilletSolver and FilletCorner intersection topology to m_pBrep
  if (m_bDoTopologyInsertion)
    {
      // for every FilletSolver - insert intersection topology
      for (ii=0; ii<m_vFilletSolvers.GetSize(); ii++)
        {
          SmFilletSolver * pFilletSolver = m_vFilletSolvers[ii];
          pFilletSolver->GetFilletGeoms(sFilletGeoms);
          for (jj=0; jj<sFilletGeoms.GetSize(); jj++)
            {
              SmFilletGeom * pFilletGeom = sFilletGeoms[jj];

#ifdef SM_DEBUG_CODE
              if(bDebugMe || iDebugLevel > 20) 
                {
                  smos_sprintf(sBuff,_T("\nFillet: Iter:[%ld] Insert FilletGeometry Loop\n"), ii); smos_WriteBuffer(sBuff);
                  SM_DUMP_AND_ASSERT_VALID(pFilletGeom) ;
                }
#endif // SM_DEBUG_CODE

              SER(pFilletGeom->InsertIntersectionTopology());
            }
        } // end iter every m_vFilletSolver

#ifdef SM_DEBUG_CODE
      if(bDebugMe || iDebugLevel > 0) 
        {
          smos_sprintf(sBuff,_T("%s"),_T("\nFillet: After InsertIntersectionTopology() loop: Solvers\n"));
          smos_WriteBuffer(sBuff);

          SM_DUMP_AND_ASSERT_VALID(m_pTargetBrep) ;
          SM_DUMP_AND_ASSERT_VALID(m_pFilletBrep) ;
          m_vTI.Dump();

          smgfx_Erase();
          smgfx_SetLook( 1,0, 0,0,1 ); m_pTargetBrep->Draw(TRUE); sm_GraphicsLoop();
          smgfx_SetLook( 2,4, 0,1,0 ); m_pFilletBrep->Draw(TRUE); sm_GraphicsLoop();
          smgfx_SetLook( 3,5, 1,0,1 ); m_vTI.Draw(3,5, 7,9); sm_GraphicsLoop();
          sm_GraphicsLoop();
      }
#endif // SM_DEBUG_CODE

      // for every fillet corner - insert intersection topology (only those edges that lie on m_pTargetBrep faces)
      for(ii=0;ii<m_vFilletCorners.GetSize();ii++)
        {
          SmFilletCorner * pFilletCorner = m_vFilletCorners[ii];

#ifdef SM_DEBUG_CODE
          if(bDebugMe || iDebugLevel > 20) 
            {
              smos_sprintf(sBuff,_T("\nFillet: Iter:[%ld] Insert  FilletCorner Loop\n"), ii); smos_WriteBuffer(sBuff);
              SM_DUMP_AND_ASSERT_VALID(pFilletCorner) ;
            }
#endif // SM_DEBUG_CODE

          SER(pFilletCorner->InsertIntersectionTopology());

#ifdef SM_DEBUG_CODE
          if(bDebugMe || iDebugLevel > 0) 
            {
              smos_sprintf(sBuff,_T("%s"),_T("\nFillet: After InsertIntersectionTopology() loop: Corners\n"));
              smos_WriteBuffer(sBuff);

              SM_DUMP_AND_ASSERT_VALID(m_pTargetBrep) ;
              SM_DUMP_AND_ASSERT_VALID(m_pFilletBrep) ;
              m_vTI.Dump();

              smgfx_Erase();
              smgfx_SetLook( 1,0, 0,0,1 ); m_pTargetBrep->Draw(TRUE); sm_GraphicsLoop();
              smgfx_SetLook( 2,4, 0,1,0 ); m_pFilletBrep->Draw(TRUE); sm_GraphicsLoop();
              smgfx_SetLook( 3,5, 1,0,1 ); m_vTI.Draw(3,5, 7,9); sm_GraphicsLoop();
              sm_GraphicsLoop();
          }
#endif // SM_DEBUG_CODE

        } // end iter every m_vFilletCorner

#ifdef SM_DEBUG_CODE
      if(bDebugMe || iDebugLevel > 0) 
        {
          smos_sprintf(sBuff,_T("%s"),_T("\nFillet: After InsertIntersectionTopology() loop: Corners\n"));
          smos_WriteBuffer(sBuff);

          SM_DUMP_AND_ASSERT_VALID(m_pTargetBrep) ;
          SM_DUMP_AND_ASSERT_VALID(m_pFilletBrep) ;
          m_vTI.Dump();

          smgfx_Erase();
          smgfx_SetLook( 1,0, 0,0,1 ); m_pTargetBrep->Draw(TRUE); sm_GraphicsLoop();
          smgfx_SetLook( 2,4, 0,1,0 ); m_pFilletBrep->Draw(TRUE); sm_GraphicsLoop();
          smgfx_SetLook( 3,5, 1,0,1 ); m_vTI.Draw(3,5, 7,9); sm_GraphicsLoop();
          sm_GraphicsLoop();
      }
#endif // SM_DEBUG_CODE

    } // end m_bDoTopologyInsertion == TRUE branch

  // Mapping topology to enable us to remove topological edges that are
  // created during the filleting process:
  // Not implemented yet, see the comment right before
  // the call to BooleanPostProcess().

#define cbi_MAP_TOPO_EDGES 0  // 0: don't do anything yet.

#if ( cbi_MAP_TOPO_EDGES == 1 )
  // Copy the entities from the entity maps into the arrays for
  // BooleanPostProcess() to process.  This is necessary because the
  // Boolean code in Merge uses this same code, and there, entities
  // get added to the maps that should not be processed.
  // BUT: These all get deleted somewhere between here and BooleanPostProcess().
  SmTArray< SmObject* > sObjs;
  m_vTI.GetMappedBrepObjects( SmEdge_TYPE, sObjs );
  ULONG lNumObjs = sObjs.GetSize();
  for ( ii=0; ii<lNumObjs; ii++ )
    { m_sIntersectionEdges.Add( SM_CAST_PTR( SmEdge, sObjs[ii] ) ); }

  m_vTI.GetMappedBrepObjects( SmVertex_TYPE, sObjs );
  lNumObjs = sObjs.GetSize();
  for ( ii=0; ii<lNumObjs; ii++ )
    { m_sIntersectionVertices.Add( SM_CAST_PTR( SmVertex, sObjs[ii] ) ); }
#endif // cbi_MAP_TOPO_EDGES

  // 
  if (m_bDoGlobalMerge)
    {

      SmApproxTol3d sApproxTol3d = SmTol::GetApproxTol3d(m_pTargetBrep) ; 
      m_vTI.SetThisApproxTol3d(sApproxTol3d);
      SER(m_vTI.IntersectInsertRelate());

#ifdef SM_DEBUG_CODE
      if(bDebugMe || iDebugLevel > 0)
        {
          smos_sprintf(sBuff,_T("%s"),_T("\nFillet: After IntersectInsertRelate:\n"));
          smos_WriteBuffer(sBuff);

          m_pTargetBrep->Dump();
          m_pFilletBrep->Dump();
          m_vTI.Dump();
          smgfx_Erase();
          smgfx_SetLook( 1,0, 0,0,1 ); m_pTargetBrep->Draw(TRUE); sm_GraphicsLoop();
          smgfx_SetLook( 2,4, 0,1,0 ); m_pFilletBrep->Draw(TRUE); sm_GraphicsLoop();
          smgfx_SetLook( 3,5, 1,0,1 ); m_vTI.Draw(3,5, 7,9); sm_GraphicsLoop();
          sm_GraphicsLoop();
        }
#endif // SM_DEBUG_CODE

    } // end m_bDoGlobalMerge check - use m_vTI.IntersectInsertRelate()

#ifdef VALIDATE_TOPOLOGY
      m_pTargetBrep->ValidatePointers();
      m_pFilletBrep->ValidatePointers();
#endif // VALIDATE_TOPOLOGY

  // when asked - delete the parts of the m_pTargetBrep that get removed by adding the Fillet
  if (m_bDoTrimming)
    {
      SmTopologyIntersector & rTopoInt = GetTopologyIntersector();

      // Mark all common edges and vertices.

      //  Marks are tricky here because DoSurfaceFilleting() can construct
      //  fillet geometry between surfaces from different Breps. Different Breps 
      //  can have different contexts and each context manages its own marks.
      //  So, for each unique context, lock a mark for this operation and
      //  then use the appropriate mark for each context later in the TrimOriginalSurfaces calls
      SmTArray<SmNewMarkAndLock *> sMarkLocks ;               // container for all upcoming MarkLocks
      SmNewMarkAndLock sFilletLock((SmContext *)(&m_crContext)) ;
      SmNewMarkAndLock sOrigLock ;
      SmMarkType       eFilletMarkType = sFilletLock.GetMarkType() ;
      SmMarkType       eOrigMarkType   = eFilletMarkType ;

      // add 1st mark to the Mark list
      sMarkLocks.Add(&sFilletLock) ;

      // when m_pOrigBrep and m_pFilletBrep are in different contexts (expected) - add a 2nd MarkLock to the Mark list
      if(&m_crContext != m_pTargetBrep->GetContext())
        { 
          sOrigLock.SetContext((SmContext *)m_pTargetBrep->GetContext()) ; 
          sMarkLocks.Add(&sOrigLock) ;
          eOrigMarkType = sOrigLock.GetMarkType() ;
        }

      //  The marked common Edges and Vertices will give us a closed loop of 
      //  Edges from the each OrigFaces Brep. We later delete every Face inside this closed loop,
      //  and stitch the Fillet Brep in.

      // Mark all common vertices.
        {
          SmTArray<SmVertex*> sBrepVertices;
          m_pTargetBrep->GetVertices(sBrepVertices);
          for (ii=0; ii<sBrepVertices.GetSize(); ii++)
            {
              SmVertex *pOVert = (SmVertex*)rTopoInt.GetOtherMate(sBrepVertices[ii]);
              if (pOVert)                                               
                {
                  SM_ASSERT(pOVert->GetBrep() == m_pFilletBrep) ; // gwc: check that OVert is from proper Brep used to
                                                                  // set the marks in the above SmNewMarkAndLock constructor call
                  pOVert->Mark(eFilletMarkType);
                  sBrepVertices[ii]->Mark(eOrigMarkType);
                }
            }
        }

      // Mark all common edges.
        {
          SmTArray<SmEdge*> sBrepEdges;
          m_pTargetBrep->GetEdges(sBrepEdges);

          // mark every brepEdge that has an OtherEdge mate  
          for (ii=0; ii<sBrepEdges.GetSize(); ii++)
            {
              SmEdge *pEdge = sBrepEdges[ii];
              SmEdge *pOtherEdge = (SmEdge*)rTopoInt.GetOtherMate(pEdge);
              if (pOtherEdge)
                {
                  SM_ASSERT(pOtherEdge->GetBrep() == m_pFilletBrep) ; // gwc: check that pOtherEdge is from proper Brep used to
                                                                      // set the marks in the above SmNewMarkAndLock constructor call
                  pEdge->Mark(eOrigMarkType);
                  pOtherEdge->Mark(eFilletMarkType);

#ifdef SM_DEBUG_CODE
                  if(bDebugMe || iDebugLevel > 20) 
                    {
                      if ( FALSE ) 
                        {
                          smgfx_Erase();
                          smgfx_SetLook( 1,2, 0,0,0 ); m_pTargetBrep->Draw( TRUE ); sm_GraphicsLoop();
                        }
                      smgfx_SetLook( 2,4, 1,0,0 ); pEdge->Draw();      sm_GraphicsLoop();
                      smgfx_SetLook( 4,6, 0,0,1 ); pOtherEdge->Draw(); sm_GraphicsLoop();
                      sm_GraphicsLoop();
                    }
#endif // SM_DEBUG_CODE

                } // end edge has an OtherEdge mate check
            } // end iter every Brep Edge marking all edges with OtherEdge mates
        } // end mark all common edges scope block

#ifdef SM_DEBUG_CODE
      ULONG lLastDelFace= 0, lLastKeepFace = 0;

      if(bDebugMe || iDebugLevel > 10)
        {
          SmTArray< SmEdge* > sEdges;
          m_pTargetBrep->GetEdges( sEdges );

          smgfx_SetLook( 3,5, 1,0,1 ); for (ULONG ie=0; ie<sEdges.GetSize(); ie++ ) 
                                         { if(sEdges[ie]->IsMarked(eOrigMarkType))
                                             { SM_ASSERT_VALID(sEdges[ie]); sEdges[ie]->DrawParams(); sm_GraphicsLoop(); }
                                         }
          sm_GraphicsLoop();
        }
#endif // SM_DEBUG_CODE

      // for every fillet solver - build sKeptFBFaces and sDelFaces arrays
      SmTArray<SmFace*> sDelFaces;
      SmTArray<SmFace*> sKeptFBFaces;
      for ( ii=0; ii < m_vFilletSolvers.GetSize(); ii++ )
        {
          SmFilletSolver * pFilletSolver = m_vFilletSolvers[ii];
          pFilletSolver->GetFilletGeoms( sFilletGeoms );
          for ( jj=0; jj < sFilletGeoms.GetSize(); jj++ )
            {
              SmFilletGeom * pFilletGeom = sFilletGeoms[jj];

              // Trim the original surfaces in cases where the fillet edge
              // has split the face and collects faces for FacesKept and FacesDelete lists.
              SER( pFilletGeom->TrimOriginalSurfaces( sKeptFBFaces, // out: accumulating list of faces in m_pFilletBrep to keep 
                                                      sDelFaces,    // out: accumulating list of faces from OrigBrep(s) to delete
                                                      sMarkLocks)); // in : list of all marks for all contexts used in this fillet
                                                                    //      the first entry is for SmFilletExecutive::m_crContext
                                                                    // note: Checks and sets topology mark values, does not increment context mark values
#ifdef SM_DEBUG_CODE
              if(bDebugMe || iDebugLevel > 10) 
                {
                  ULONG ie;
                  if ( FALSE ) 
                    { smgfx_Erase();
                      smgfx_SetLook( 1,2, 0,0,0 ); m_pTargetBrep->Draw(TRUE); sm_GraphicsLoop();
                      smgfx_SetLook( 1,2, 0,0,0 ); m_pTargetBrep->DrawUV(TRUE); sm_GraphicsLoop();
                      sm_GraphicsLoop();
                    }
                  smgfx_SetLook( 2,4, 1,0,0); for (ie=lLastDelFace; ie<sDelFaces.GetSize(); ie++) 
                                                { sDelFaces[ie]->Draw(SM_DM_CROSSHATCH); sm_GraphicsLoop(); }
                  smgfx_SetLook( 3,5, 0,1,0); for (ie=lLastKeepFace; ie<sKeptFBFaces.GetSize(); ie++) 
                                                 { sKeptFBFaces[ie]->Draw(SM_DM_CROSSHATCH); sm_GraphicsLoop(); }
                  lLastDelFace  = sDelFaces.GetSize();
                  lLastKeepFace = sKeptFBFaces.GetSize();
                  sm_GraphicsLoop();
                }
#endif // SM_DEBUG_CODE

            }  // end iter every FilletSolver->FilletGeom
        } // end iter every fillet solver - building sKeptFBFaces and sDelFaces arrays

#ifdef VALIDATE_TOPOLOGY
      m_pTargetBrep->ValidatePointers();
      m_pFilletBrep->ValidatePointers();
#endif // SM_DEBUG_CODE

#ifdef SM_DEBUG_CODE
      if(bDebugMe || iDebugLevel > 0) 
        {
          smos_sprintf(sBuff,_T("%s"),_T("\nFillet: After Trimming Original Faces:\n"));
          smos_WriteBuffer(sBuff);
          m_vTI.Dump();

          ULONG ll;
          smgfx_Erase();
          smgfx_SetLook( 1,0, 0,0,0 ); m_pTargetBrep->Draw(TRUE); sm_GraphicsLoop();
          smgfx_SetLook( 2,4, 1,0,0 ); m_pFilletBrep->Draw(TRUE); sm_GraphicsLoop();
          smgfx_SetLook( 3,5, 1,0,1 ); m_vTI.Draw(3,5, 7,9); sm_GraphicsLoop();
          smgfx_SetLook( 2,4, 0,1,1 ); for(ll=0;ll<sKeptFBFaces.GetSize();ll++)
                                         { if(sKeptFBFaces[ll]) sKeptFBFaces[ll]->Draw(SM_DM_CROSSHATCH) ; sm_GraphicsLoop() ; }
          smgfx_SetLook( 2,4, 1,0,1 ); for(ll=0;ll<sDelFaces.GetSize();ll++)
                                         { if(sDelFaces[ll]) sDelFaces[ll]->Draw(SM_DM_CROSSHATCH) ; sm_GraphicsLoop() ; }
          sm_GraphicsLoop() ;
        }
#endif // SM_DEBUG_CODE

      // Now that we have done all of the marking.  
      // Place OrigBrep faces that are being replaced by the new Fillet Faces into sDelFaces array.
      SmTArray<SmFace*> sFBFaces;
      m_pFilletBrep->GetFaces(sFBFaces);
      for (ii=0; ii<sFBFaces.GetSize(); ii++)
        {
          SmFace *pF = sFBFaces[ii];
          ULONG lFoundIndex;
          if (!sKeptFBFaces.FindElement(pF,lFoundIndex))
            {
#ifdef SM_DEBUG_CODE
              if(bDebugMe || iDebugLevel > 10) 
                {
                  smgfx_SetLook( 4,6, 1,0,0 ); pF->Draw(); sm_GraphicsLoop();
                  sm_GraphicsLoop();
                }
#endif // SM_DEBUG_CODE

              sDelFaces.AddUnique(pF);
            }
        } // end iter all FilletBrep faces building sDelFaces array

#ifdef SM_DEBUG_CODE
      if(bDebugMe || iDebugLevel > 0) 
        {
          smos_sprintf(sBuff,_T("%s"),_T("\nFillet: After Trimming Original Faces:\n"));
          smos_WriteBuffer(sBuff);
          m_vTI.Dump();

          smgfx_Erase();
          smgfx_SetLook( 1,0, 0,0,0 ); m_pTargetBrep->Draw(TRUE); sm_GraphicsLoop();
          smgfx_SetLook( 2,4, 1,0,0 ); m_pFilletBrep->Draw(TRUE); sm_GraphicsLoop();
          smgfx_SetLook( 3,5, 1,0,1 ); m_vTI.Draw(3,5, 7,9); sm_GraphicsLoop();
          smgfx_SetLook( 2,4, 1,0,1 ); for(ULONG id=0;id<sDelFaces.GetSize();id++)
                                         { if(sDelFaces[id]) sDelFaces[id]->Draw(SM_DM_CROSSHATCH) ; sm_GraphicsLoop() ; }
          sm_GraphicsLoop() ;
        }
#endif // SM_DEBUG_CODE

      // delete the faces on sDelFaces array
      SER(GetTopologyIntersector().DeleteFaces(sDelFaces));

#ifdef SM_DEBUG_CODE
      if(bDebugMe || iDebugLevel > 0) 
        {
          smos_sprintf(sBuff,_T("%s"),_T("\nFillet: After Deleting Faces:\n"));
          smos_WriteBuffer(sBuff);

          m_pTargetBrep->Dump();
          m_pFilletBrep->Dump();
          m_vTI.Dump();
          smgfx_Erase();
          smgfx_SetLook( 1,0, 0,0,1 ); m_pTargetBrep->Draw(TRUE); sm_GraphicsLoop();
          smgfx_SetLook( 2,4, 0,1,0 ); m_pFilletBrep->Draw(TRUE); sm_GraphicsLoop();
          smgfx_SetLook( 3,5, 1,0,1 ); m_vTI.Draw(3,5, 7,9); sm_GraphicsLoop();
          sm_GraphicsLoop();
        }
#endif // SM_DEBUG_CODE

#ifdef VALIDATE_TOPOLOGY
      m_pTargetBrep->ValidatePointers();
      m_pFilletBrep->ValidatePointers();
#endif // VALIDATE_TOPOLOGY

      // add an attribute to every SmFilletEdge on m_vTopoEdges
      SmTArray<SmAttribute*> sAttributes;
      SmObjsDelete<SmAttribute*> sCleansAttr(&sAttributes);
        {
          SmTArray<SmEdge*> sBrepEdges;
          m_pFilletBrep->GetEdges(sBrepEdges);
          // Use attributes to keep those topo edges
          for (ii=0; ii<m_vTopoEdges.GetSize(); ii++)
            {
              SmFilletEdge * pFE = (SmFilletEdge*)m_vTopoEdges[ii];
              if (   pFE == NULL
                  || pFE->GetFilletEdgeType() != SM_FE_ON_EDGE
                  || pFE->GetCurve() == NULL)
                {
                  continue;
                }
              SmEdge * pE = pFE->m_pFilletBrepEdge1;
              if (pE == NULL) { continue; }
              ULONG lIndex;
              if (!sBrepEdges.FindElement(pE,lIndex)) { continue; }
              SmAttribute * pAtt = new (m_crContext) SmAttribute(SM_AI_FILLET_EDGES,SM_AB_STANDALONE_REFERENCE);
              pE->AddAttribute(pAtt);
              sAttributes.Add(pAtt);

            } // end for each edge in m_vTopoEdges

#if ( cbi_MAP_TOPO_EDGES == 7 )
            {
              // put attr on every Brep edge with a FilletBrep mate.
              for ( ii=0; ii<sBrepEdges.GetSize(); ii++ )
                {
                  SmEdge *pFilEdge = sBrepEdges[ii];
                  SmObject *pBrepObj = m_vTI.GetBrepMate( pFilEdge );
                  SmEdge *pBrepEdge = SM_CAST_PTR( SmEdge, pBrepObj );
                  if ( pBrepEdge == NULL )
                    { continue; }
      
                  SmAttribute * pAtt = new (m_crContext) SmAttribute(SM_AI_FILLET_EDGES,SM_AB_STANDALONE_REFERENCE);
                  pBrepEdge->AddAttribute( pAtt );
                  sAttributes.Add(pAtt);
                }
            }
#endif // cbi_MAP_TOPO_EDGES

        } // end scope adding an attribute to every SmFilletEdge on m_vTopoEdges

      // combine the new Fillet Geometry into the original m_pTargetBrep
      SER(m_pTargetBrep->MergeBrep(*m_pFilletBrep));

#ifdef SM_DEBUG_CODE
      if(bDebugMe || iDebugLevel > 0) 
        {
          smos_sprintf(sBuff,_T("%s"),_T("\nFillet: After MergeBrep():\n"));
          smos_WriteBuffer(sBuff);

          m_pTargetBrep->Dump();
          m_pFilletBrep->Dump();
          m_vTI.Dump();
          smgfx_Erase();
          smgfx_SetLook( 1,2, 0,0,1 ); m_pTargetBrep->Draw(TRUE); sm_GraphicsLoop();
          smgfx_SetLook( 2,4, 0,1,0 ); m_pFilletBrep->Draw(TRUE); sm_GraphicsLoop();
          smgfx_SetLook( 3,5, 1,0,1 ); m_vTI.Draw(3,5, 7,9); sm_GraphicsLoop();
          sm_GraphicsLoop();
        }
#endif // SM_DEBUG_CODE

      // Memory cleanup: Delete m_pFilletBrep (its topology and geometry have been copied into original m_pTargetBrep)
      // Before deleting:
      if ( m_vTI.GetOtherBrep()   == m_pFilletBrep ) { m_vTI.SetOtherBrep( NULL ); } // the usual case
      if ( m_vTI.GetPrimaryBrep() == m_pFilletBrep ) { m_vTI.SetPrimaryBrep( NULL ); } // not likely, but...

      SM_ASSERT(m_pFilletBrep != NULL) ; delete m_pFilletBrep ; m_pFilletBrep = NULL ;
      
      // Stitch up all fillets
      if (1)
        {
          // Stitch up all fillets
          ULONG lNumStitched, lNumLamina;
          double dMaxVGap, dMaxEGap;
          double dStitchTol3d = m_pTargetBrep->GetTolerance();
          SmStitchCallback sCallBack;
          SmStitch sStitch(sCallBack,dStitchTol3d);
          sStitch.m_bDoRegionNesting = TRUE;
          sStitch.m_bIgnoreProblems = TRUE;
          //sStitch.m_bFastEdgeCompare = TRUE;
          //sStitch.m_bValidateResult = FALSE;
          //sStitch.m_bSqueezeSmallEdges = TRUE;
            SmStatus eStat;
            ULONG iNumTries = 5;

            for (ii=0; ii<iNumTries; ii++)
              {
                eStat = sStitch.DoStitching(m_pTargetBrep, // in : target Brep                                                                                
                                      NULL,          // in : only glue coincident vertices on this list, NULL = do all vertices                                                                     
                                      NULL,          // in : only glue coincident edge pairs on this list, NULL = do all edges                                                                        
                                      lNumStitched,      // out: number of edges stitched                                                                       
                                      lNumLamina,    // out: number of lamina edges remaining after stitch                                                 
                                      dMaxVGap,      // out: max gap found between coincident vertices considered for glueing                           
                                      dMaxEGap);     // out: max gap found between coincident edges    considered for glueing                           
                                                     //      NOTE: some coincident vertex and edge pairs do not get glued due to                        
                                                     //            a. gaps exceeding tolerances                                                         
                                                     //            b. geometries not listed within optional candidate lists                             
                                                     //            c. a failure within the glue edge function                                           
                                                     //            d. not being lamina when m_bMakingManifoldSolid == TRUE                              
                if ( eStat==SM_SUCCESS && lNumLamina==0 )  // [B390]
                  { break; } // Good result.

                if (ii == iNumTries-1 && eStat!=SM_SUCCESS)
                  {
                    //cbi probably just a warning:
                    GetFilletErrorInfo()->NoteError( SM_FILERR_BAD_MERGE,
                          _T("Fillet Stitching Failure: Error Stitching Fillets To Original Brep") );
                    FILEXEC_ERR(_T("Error Stitching Fillets To Original Brep"));
                  }

              dStitchTol3d *= 2.0;
              sStitch.SetStitchTol3d(dStitchTol3d) ;
            }

#ifdef SM_DEBUG_CODE
          if(bDebugMe || iDebugLevel > 0)
            {
              smos_sprintf(sBuff,_T("%s"),_T("\nFillet: After Stitching:\n"));
              smos_WriteBuffer(sBuff);
              m_pTargetBrep->Dump();
              m_vTI.Dump(FALSE); // False: 'other' is not valid here.
              smgfx_Erase();
              smgfx_SetLook( 1,2, 0,0,1 ); m_pTargetBrep->Draw(TRUE); sm_GraphicsLoop();
              // objects not valid:
              // smgfx_SetLook( 3,5, 1,0,1 ); m_vTI.Draw(3,5); sm_GraphicsLoop();
              sm_GraphicsLoop();
            }
#endif // SM_DEBUG_CODE

          // Get rid of edges which are interior to faces.
          SmTopologyIntersector & rTopoIntersector = GetTopologyIntersector();
          rTopoIntersector.SetPrimaryBrep(m_pTargetBrep);

          m_vTopoEdges.ReSet();
          for (ii=0; ii<sAttributes.GetSize(); ii++)
            {
              SmAttribute * pAtt = sAttributes[ii];
              SmTArray<SmAObject*> sMapEdges;
              pAtt->GetUsers(sMapEdges);
              if (sMapEdges.GetSize() == 0) continue;
              SM_ASSERT(sMapEdges.GetSize() == 1);
              m_vTopoEdges.Add((SmEdge*)sMapEdges[0]);
            }

#if ( cbi_MAP_TOPO_EDGES == 2 )
          // Copy the entities from the entity maps into the arrays for
          // BooleanPostProcess() to process.  This is necessary because the
          // Boolean code in Merge uses this same code, and there, entities
          // get added to the maps that should not be processed.
          SmTArray< SmObject* > sObjs;
          m_vTI.GetMappedBrepObjects( SmEdge_TYPE, sObjs );
          ULONG lNumObjs = sObjs.GetSize();
          for ( ii=0; ii<lNumObjs; ii++ )
            { m_sIntersectionEdges.Add( SM_CAST_PTR( SmEdge, sObjs[ii] ) ); }

          m_vTI.GetMappedBrepObjects( SmVertex_TYPE, sObjs );
          lNumObjs = sObjs.GetSize();
          for ( ii=0; ii<lNumObjs; ii++ )
            { m_sIntersectionVertices.Add( SM_CAST_PTR( SmVertex, sObjs[ii] ) ); }
#endif // cbi_MAP_TOPO_EDGES

          // Remove topological edges that have been recorded.
          SER( RemoveTopoEdges() );

          // RemoveTopoEdge() removes the edges that have been recorded
          // in m_vTopoEdges.  I think that the filleting process can create
          // topological edges that are not on that array, so we call
          // BooleanPostProcess() to check other edges.
          // Currently this removes all topological edges in the entire model.
          // We should restrict it to topological edges that are created during
          // the filleting process.  That's a little tricky because we don't
          // currently map the edges created during the fillet.
          // If we passed False into BooleanPostProcess, it would check only
          // those entities in m_sIntersectionEdges and m_sIntersectionVertices.
          // The task is to put the edges (and vertices) that we create into
          // those arrays.  Some preliminary ideas are #ifdef'd out under
          // cbi_MAP_TOPO_EDGES.

          SmBoolean bCheckAllEdges = TRUE; // Better if we could say False...
          SER(BooleanPostProcess( bCheckAllEdges ));
          // note: face and vertex pointers might be deleted as 
          //       a side effect of BooleanPostProcessremoving edges
          //       that are left as stale pointers in m_vTI. This 
          //       function is written to allow for that.  However, 
          //       when debugging, beward of m_vTI dumps after this call.

          // Reset cache
          SmTArray<SmFace*> sFaceList;
          m_pTargetBrep->GetFaces(sFaceList);
          for (ii=0; ii<sFaceList.GetSize(); ii++)
          {
              sFaceList[ii]->Notify(SM_NO_PRE_EDIT, sFaceList[ii], SM_NO_GET_BREP(sFaceList[ii]), NULL);
          }
        } // end stitch up all fillets

        SmBoolean bMaybeNotSolid = false;
        m_pTargetBrep->OrientTrimmedSurfaces(true, bMaybeNotSolid, false);

    } // end m_bDoTrimming check

#ifdef SM_DEBUG_CODE
  if(bDebugMe || iDebugLevel > 0)
    {
      smos_sprintf(sBuff,_T("%s"),_T("\nFillet: After m_bDoTrimming:\n"));
      smos_WriteBuffer(sBuff);
      m_pTargetBrep->Dump();
      smgfx_Erase();
      smgfx_SetLook( 1,2, 0,0,1 ); m_pTargetBrep->Draw(TRUE); sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

  // look for new spine edges made by this fillet call - probably an error
  SmTArray<SmEdge*> sEdges;
  m_pTargetBrep->GetEdges(sEdges);
  for (ii=0; ii<sEdges.GetSize(); ii++)
    {
      SmEdge * pE = sEdges[ii];
      if (   !pE->IsManifold()
          && !pE->IsLamina()
          && !pE->IsWire())
        {
          SM_DBG_WARN(_T("Edges with more than two faces found - Possible filleting errors"));
          break;
        }
    } // end iter every Edge looking for Spine Edges

  // when fillet worked - set output
  if ( pOptFilletErrorInfo != NULL )
    {
      *pOptFilletErrorInfo = *( GetFilletErrorInfo() );
    }

  // all done
  return SM_SUCCESS;

} // end SmFilletExecutive::DoFilleting

/*******************************************************************//**
PURPOSE: DoSurfaceFilleting helper function to match a context 
         to a NewMarkAndLock value

NOTES: 1. returns NewMarkAndLock for given context or NULL for no match
       2. same function appears in both SmFilletGeom.cpp and
                                        SmFilletExecutive.cpp (yuk)
***********************************************************************/
SmNewMarkAndLock * sm_FindMarkForContext1
 ( SmTArray<SmNewMarkAndLock *> & rMarkLocks,  // in : list of marks to check
   const SmContext              * cpContext)   // in : context to find amongst marks
{
  ULONG ii ;
  for(ii=0;ii<rMarkLocks.GetSize();ii++)
    {
      if(cpContext == rMarkLocks[ii]->GetContext())
        { return( rMarkLocks[ii] ) ; }
    }

  return NULL ;

} // end sm_FindMarkForContext1

/*******************************************************************//**
PURPOSE: This is the high level controling method to do surface based
         filleting.

NOTES: increments unlocked mark
***********************************************************************/
SmStatus SmFilletExecutive::DoSurfaceFilleting
 (SmFilletErrorInfo * pOptFilletErrorInfo)
{
  SmTemporaryChangeValue< SmBoolean > sDoingBool( SM_CONST_CAST(SmContext*, &m_crContext)->GetDoingBooleanRef(), TRUE );

  // create output Fillet Brep to hold all the new Fillet Faces
  m_pFilletBrep = new (m_crContext) SmBrep();
  NER(m_pFilletBrep);

  // init FilletBrep for editing
  SmTemporaryChangeValue<SmBoolean> sChange1(m_pFilletBrep->m_bEditingEnabled,TRUE);

// Remove Composites
// #ifndef SM_NO_COMPOSITES
//  m_pFilletBrep->m_bMakeComposites = TRUE;
// #endif // SM_NO_COMPOSITES

  // for every FilletSolver create a FilletGeom (offset surfaces, FilletEdges, FilletVertices, Surfaces, Curves in m_crContext)
  double dTol = 0.0;
  for (ULONG j=0; j<m_vFilletSolvers.GetSize(); j++) 
    {
      if (m_vFilletSolvers[j]->CalcFilletGeom() != SM_SUCCESS) 
        {
          continue;
        }
      dTol = smos_Max(dTol,m_vFilletSolvers[j]->GetThisApproxTol3d());
    }

#ifdef SM_USE_NEWTOL                              
  SM_NEWTOL_LINE m_pFilletBrep->SetTolerance(dTol);  // gwc: this assigns an ApproxTol to ZoneTol
#else // SM_USE_OLDTOL                         
  SM_OLDTOL_LINE m_pFilletBrep->SetTolerance(dTol);
#endif // SM_USE_OLDTOL                        

  // for every FilletSolver->FilletGeom->FilletSurface, make a Face in m_pFilletBrep
  SmTArray<SmFilletGeom*> sFilletGeoms;
  for (ULONG m=0; m<m_vFilletSolvers.GetSize(); m++) 
    {
      SmFilletSolver * pFilletSolver = m_vFilletSolvers[m];
      pFilletSolver->GetFilletGeoms(sFilletGeoms);
      for (ULONG i=0; i<pFilletSolver->m_vFilletGeoms.GetSize(); i++) 
        {
          SmFilletGeom * pFilletGeom = sFilletGeoms[i];
          if (pFilletGeom->GetFilletSurface() == NULL) 
            { continue; }

          // Create Face (and Edges and Verts) in m_pFilletBrep from this m_pFilletSurface with call to pBrep->MakeFaceWithCurves()
          SER(pFilletGeom->MakeFaceBrep(m_pFilletBrep));

        } // end iter every FilletGeom
    } // end iter every FilletSolver

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe0 = FALSE;
  if (bDebugMe0) 
    {
      m_pFilletBrep->Dump();
      smgfx_SetLook(1,2, 1,0,0); m_pFilletBrep->Draw(TRUE); sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

  // For every Edge in m_pFilletBrep - add attribute(m_pAttributeE)
  SmTArray<SmEdge*> sFilletEdges;
  m_pFilletBrep->GetEdges(sFilletEdges);
  for (ULONG f=0; f<sFilletEdges.GetSize(); f++) 
    {
      sFilletEdges[f]->AddAttribute(m_pAttributeE);
    }

  // when appropriate - stitch all topology in m_pFilletBrep
  if (1) 
    {
      ULONG lNumStitched, lNumLamina;
      double dMaxVGap, dMaxEGap;
//        m_pFilletBrep->Dump();
      m_pFilletBrep->m_bEditingEnabled = TRUE;
      SER(m_pFilletBrep->StitchFaces(dTol,lNumStitched,lNumLamina,dMaxVGap,dMaxEGap));
//        TCHAR sBuff[SM_TBLOCK_SIZE];
//        smos_sprintf(sBuff, _T("Stitch Faces - # Stitched Edges = %ld, # Lamina = %ld,  Max V Gap = %le, Max E Gap = %le\n"),
//            lNumStitched,lNumLamina,dMaxVGap,dMaxEGap);
//        smos_WriteBuffer(sBuff);
//        m_pFilletBrep->Dump();
    }

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe1 = FALSE;
  if (bDebugMe1) 
    {
      m_pFilletBrep->Dump();
      smgfx_SetLook(1,2, 1,0,0); m_pFilletBrep->Draw(TRUE); sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

  // Set up a map between the fillet curves in the FilletGeoms and corresponding fillet brep edges.
  SER(SetFilletBrepEdgeMap());

  // when asked
  if (m_bDoTopologyInsertion) 
    {
      // for every FilletSovler
      for (ULONG j=0; j<m_vFilletSolvers.GetSize(); j++) 
        {
          SmFilletSolver * pFilletSolver = m_vFilletSolvers[j];
          pFilletSolver->GetFilletGeoms(sFilletGeoms);

          // for every FilletSolver->FilletGeom
          for (ULONG i=0; i<sFilletGeoms.GetSize(); i++) 
            {
              SmFilletGeom * pFilletGeom = sFilletGeoms[i];
              if (pFilletGeom->GetFilletSurface() == NULL) 
                { continue; }

              // Merge each rail into originating face and more. . .
              SER(pFilletGeom->InsertIntersectionTopology());

            } // end iter every FilletSolver->FilletGeom
        } // end iter every FilletSolver
    } // end need to DoTopologyInsertion check

  // done with m_pFilletBrep editing
  m_pFilletBrep->m_bEditingEnabled = FALSE;

  // when asked for Trimming of 
  if (m_bDoTrimming) 
    {
      SmTopologyIntersector & rTopoInt = GetTopologyIntersector();

      // Mark all common edges and vertices.
      // Marking is tricky here because the original surfaces do not have to be from the same Breps
      // so many Contexts might be tracking marks.  
      //  We have to tell each context to lock a mark for this operation and
      //  then use the appropriate mark for each context later in the TrimOriginalSurfaces calls
      SmTArray<SmNewMarkAndLock *>     sMarkLocks ;               // container for all upcoming MarkLocks
      SmObjsDelete<SmNewMarkAndLock *> sCleanLocks(&sMarkLocks) ; // all marks unlocked when sCleanLocks goes out of scope

      // the first mark on the Mark list is for SmFilletExecutive::m_crContext used for all new FilletGeometry,
      // the subsequent marks are for all other context objs being used for the original geometry.
      SmNewMarkAndLock * pFilletMark     = new SmNewMarkAndLock((SmContext *)(&m_crContext)) ;
      SmMarkType         eFilletMarkType = pFilletMark->GetMarkType() ;
      sMarkLocks.Add(pFilletMark) ;

        { // mark common vertices
          SmTArray<SmVertex*> sBrepVertices;
          m_pFilletBrep->GetVertices(sBrepVertices);
          for (ULONG ii=0; ii<sBrepVertices.GetSize(); ii++) 
            {
              SmVertex *pVert = (SmVertex*)rTopoInt.GetBrepMate(sBrepVertices[ii]);
              if (pVert) 
                {
                  const SmContext  *pOrigContext = pVert->GetContext() ;
                  SmNewMarkAndLock *pOrigMark    = sm_FindMarkForContext1(sMarkLocks, pOrigContext) ;
                  if(pOrigMark == NULL) { pOrigMark = new SmNewMarkAndLock((SmContext *)pOrigContext) ;
                                          sMarkLocks.Add(pOrigMark) ;
                                        }
                  pVert->Mark(pOrigMark->GetMarkType());
                  sBrepVertices[ii]->Mark(eFilletMarkType);
                }
            }
        } // end marking common vertices

        { // mark common edges
          SmTArray<SmEdge*> sBrepEdges;
          m_pFilletBrep->GetEdges(sBrepEdges);

          for (ULONG j=0; j<sBrepEdges.GetSize(); j++) 
            {
              SmEdge *pOtherEdge = sBrepEdges[j];
              SmEdge *pEdge = (SmEdge*)rTopoInt.GetBrepMate(pOtherEdge);
              if (pEdge) 
                {
                  const SmContext  *pOrigContext = pEdge->GetContext() ;
                  SmNewMarkAndLock *pOrigMark    = sm_FindMarkForContext1(sMarkLocks, pOrigContext) ;
                  if(pOrigMark == NULL) { pOrigMark = new SmNewMarkAndLock((SmContext *)pOrigContext) ;
                                          sMarkLocks.Add(pOrigMark) ;
                                        }
                  pEdge->Mark(pOrigMark->GetMarkType());
                  pOtherEdge->Mark(eFilletMarkType);
                }
            }
        } // end marking common Edges

      SmTArray<SmFace*> sKeptFBFaces;
      SmTArray<SmFace*> sDelFaces;

      // for every FilletSolver
      for (ULONG j=0; j<m_vFilletSolvers.GetSize(); j++) 
        {
          SmFilletSolver * pFilletSolver = m_vFilletSolvers[j];
          pFilletSolver->GetFilletGeoms(sFilletGeoms);

          // for every FilletSolver->FilletGeom
          for (ULONG i=0; i<sFilletGeoms.GetSize(); i++) 
            {
              SmFilletGeom * pFilletGeom = sFilletGeoms[i];
              if (pFilletGeom->GetFilletSurface() == NULL)
                { continue; }

              // find faces in OrigBrep replaced by pFilletGeom->FilletFaces
              SER(pFilletGeom->TrimOriginalSurfaces(sKeptFBFaces, // out: list of faces in m_pFilletBrep to keep 
                                                    sDelFaces,    // out: list of faces from OrigBrep(s) to delete
                                                    sMarkLocks)); // in : list of all marks for all contexts used in this fillet
                                                                  //      the first entry is for SmFilletExecutive::m_crContext
                                                                  // note: Checks and sets topology mark values, does not increment context mark values
            } // end iter every FilletSolver->FilletGeom
        } // end iter every FilletSolver

      SER(GetTopologyIntersector().DeleteFaces(sDelFaces));

    } // end need to do Trimming of OriginalSurfaces check

  if ( pOptFilletErrorInfo != NULL )
    {
      *pOptFilletErrorInfo = *( GetFilletErrorInfo() );
    }

  return SM_SUCCESS;

} // end SmFilletExecutive::DoSurfaceFilleting

/*******************************************************************//**
PURPOSE: Post process after the Boolean operation and clean up
    edges which are inside of faces.

NOTES:  A topoEdge is an edge which can be removed from the
  topology graph without affecting the shape being modeled.  Things
  like strut edges. 
***********************************************************************/
SmStatus SmFilletExecutive::RemoveTopoEdges()
{
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe=FALSE;
  if (bDebugMe) 
    {
      smgfx_Erase();
      smgfx_SetLook(1,2, 0,0,1); m_pTargetBrep->Draw(TRUE); sm_GraphicsLoop() ;
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE
  ULONG ii, jj;

  // for every Edge in the m_vTopoEdges list
  for (ii=0; ii<m_vTopoEdges.GetSize(); ii++) 
    {
      SmEdge *pEdge = m_vTopoEdges[ii];

      // locals
      SmTArray<SmVertex*> sVerts;
      SmTArray<SmFace*> sFaces;
      pEdge->GetVertices(sVerts);
      pEdge->GetFaces(sFaces);
      
      // for manifold edges
      if (sFaces.GetSize() == 2) 
        {
          // when faces are defined by different surfaces
          // copy one surface, delete the other, and map all the other->Faces to the copy Surface
          if(   sFaces[0]->GetSurface() 
             != sFaces[1]->GetSurface()) 
            {
              // Try to expand the coincident surface domains - for some closed
              // surfaces and general NURBS this may not be possible.
              SmSurface *pSurface0 = sFaces[0]->GetSurface();
              SmSurface *pSurface1 = sFaces[1]->GetSurface();
              SmExtent2d sDomain0 = pSurface0->GetNaturalUVDomain();
              SmExtent2d sDomain1 = pSurface1->GetNaturalUVDomain();
              SmSurface * pKeptSurface   = NULL;
              SmSurface * pDeleteSurface = NULL;

              // pick a surface to delete
              if (sDomain0.IsContainedBy(sDomain1, SM_EFF_ZERO)) 
                {
                  pKeptSurface   = pSurface1;
                  pDeleteSurface = pSurface0;
                }
              else 
                {
                  pKeptSurface   = pSurface0;
                  pDeleteSurface = pSurface1;
                }

              // Copy the KeptSurface
              SmSurface * pNewSurface = NULL;
              
              // Copy pKeptSurface, when possible as an analytic surface
              SER(pKeptSurface->CopyAndAddAnalytics(m_crContext,pNewSurface));
              SmExtent2d sFaceDomain = pNewSurface->GetNaturalUVDomain();

// Remove Composites - move block up from below
              SM_ASSERT_MSG(pDeleteSurface->GetFace() != NULL, _T("SmFilletExecutive::RemoveTopoEdges - Remove Composite single face assumption wrong here - needs debug")) ;
              SmFace * pFace = pDeleteSurface->GetFace() ;
              pFace->SetUVDomain(sFaceDomain);
              pFace->SetSurface(pNewSurface);
              pNewSurface->SetOwner(pFace);

//              // get all faces defined by the DeleteSurface
//              SmTArray<SmFace*> sAllFaces;
//              SmBrep::GetFacesOfSurface(pDeleteSurface,sAllFaces);
//
//              // Set NewSurface->Owner = Face and Face->Surface(NewSurface)
//              for (jj=0; jj<sAllFaces.GetSize(); jj++) 
//                {
//                  SmFace *pFace = sAllFaces[jj];
//                  pFace->SetUVDomain(sFaceDomain);
//                  pFace->SetSurface(pNewSurface);
//                  pNewSurface->SetOwner(pFace);
//                }
//
//              // when DeleteSurface is a compositeFace, Set Owner pointers for composite faces
//              if (sAllFaces.GetSize() > 1) 
//                {
//                  SmCFace *pCFace = (SmCFace*)pDeleteSurface->GetOwner();
//                  pCFace->SetUVDomain(sFaceDomain);
//                  pCFace->SetSurface(pNewSurface);
//                  pNewSurface->SetOwner(pCFace);
//                }

              // delete the DeleteSurface
              SM_ASSERT(pDeleteSurface != NULL) ; delete pDeleteSurface ; pDeleteSurface = NULL ;
            } // end when faces are defined by different surfaces
        } // end when edge attaches to two faces

      // Now delete the edge
      SER(m_pTargetBrep->DeleteEdge(pEdge));

      // Delete topological vertex and combine two adjacent edges
      for (jj=0; jj<sVerts.GetSize(); jj++) 
        {
          SmVertex * pVertex = sVerts[jj];
          SmTArray<SmEdge*> sEdges;
          pVertex->GetEdges(sEdges);
          switch (sEdges.GetSize()) 
            {
              case 0:
                  SER(m_pTargetBrep->DeleteVertex(pVertex));
                  break;
              case 2:
                  SmEdge *pE1 = sEdges[0];
                  SmEdge *pE2 = sEdges[1];
                  SmTArray<SmVertexuse*> sVUs;
                  pVertex->GetVertexuses(sVUs);

                  // when vertex is connected to just two manifold edges
                  if(   sVUs.GetSize() == 4
                     && (   pE1->IsManifold() 
                         && pE2->IsManifold())) 
                    { 
                      // when edges are different
                      if (pE1 != pE2) 
                        {
                          // Here is where we will do some squeezing to clean up
                          // vertices that split edges.
                          // Not passing in an edge to delete will remove the shorter edge.
                          if (m_pTargetBrep->DeleteTopologicalVertex(pVertex) != SM_SUCCESS) 
                            {
#ifdef SM_VALIDATE_TOPOLOGY
                              m_pTargetBrep->ValidatePointers();
#endif // SM_VALIDATE_TOPOLOGY
                              continue;  // Unable to really combine edges;
                            }
                        } // end edges are different check
                    } // end vertex is connected to just two manifod edges check
            } // end switch on number of edges attached to the vertex
        } // end iter every vertex
    } // for iter every topoedge

  // all done
  return SM_SUCCESS;

} // end SmFilletExecutive::RemoveTopoEdges

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
SmStatus SmFilletExecutive::SetBevelCorner(SmVertex * pCornerVert)
{
    SmFilletCorner * pFilletCorner = (SmFilletCorner*) m_vVertexCornerMap.At(pCornerVert);
    NER(pFilletCorner);
    pFilletCorner->SetBevel(TRUE);

    return SM_SUCCESS;

} // end SmFilletExecutive::SetBevelCorner

/*******************************************************************//**
PURPOSE: Generate map From FilletSolver and FilletCorner FilletEdges
         stored in m_pPseudoBrep to m_pFilletBrep->Edges by finding 
         coincident mid-Points.

NOTES:
  For every m_pPseudoBrep->FilletEdge (created by every FilletSolver and FilletCorner), 
  find the m_pFilletBrep->Edge that has the same shape by testing the
  distance between midPoints.
***********************************************************************/
SmStatus SmFilletExecutive::SetFilletBrepEdgeMap()
{
   // get edges from FilletBrep (fillet edges already created)
   SmTArray<SmEdge*> sEdges;
   m_pFilletBrep->GetEdges(sEdges);
   SmTArray<SmFilletGeom*> sFilletGeoms;

   ULONG ii, jj, kk;

   // for every FilletEdgeSolver
   for (ii=0; ii<m_vFilletSolvers.GetSize(); ii++)
     {
       SmFilletSolver * pFilletSolver = m_vFilletSolvers[ii];

       // get all FilletGeoms in m_vFilletGeoms
       pFilletSolver->GetFilletGeoms(sFilletGeoms);

       // for every FilletGeom
       for (jj=0; jj<sFilletGeoms.GetSize(); jj++)
         {
           SmFilletGeom * pFilletGeom = sFilletGeoms[jj];

           // Collect all rails
           SmTArray<SmFilletEdge*> sAllRails;
           sAllRails.Add(pFilletGeom->m_vRails[0]);
           sAllRails.Add(pFilletGeom->m_vRails[1]);
           sAllRails.Append(pFilletGeom->m_vOtherRails1);
           sAllRails.Append(pFilletGeom->m_vOtherRails2);

           // set every pRail->m_pFilletBrepEdge1 = FilletBrepEdge that has same geometry
           for (kk=0; kk<sAllRails.GetSize(); kk++)
             {
               SmFilletEdge * pRail = sAllRails[kk];
               SER(pRail->FindFilletBrepEdge(sEdges));
             }

           // set every nonRail FilletEdge->m_pFilletBrepEdge1 = FilletBrepEdge that has same geometry
           for (kk=0; kk<pFilletGeom->m_vSideEUs.GetSize(); kk++)
             {
               SmFilletEdgeuse *pEU   = pFilletGeom->m_vSideEUs[kk];
               SmFilletEdge    *pFE   = (SmFilletEdge*)pEU->GetEdge();
               SmFace          *pFace = pFE->GetOriginalFace();
               if (!pFace) continue;
               SmStatus eStat = pFE->FindFilletBrepEdge(sEdges);
               if ( eStat != SM_SUCCESS )
               {
                   // when FindFilletBrepEdge failed - note the error
                   NoteFilletError( SM_FILERR_EDGE_PROBLEM, pFE,
                                   _T("Unable to map new Fillet Edge back to Edge in Target Brep") );
                   SER( eStat );
               }
             }
         } // end iter every FilletGeom
     } // end iter every FilletEdgeSolver

   // for every FilletCorner
   for (ii=0; ii<m_vFilletCorners.GetSize(); ii++)
     {
       SmFilletCorner * pFilCorner = m_vFilletCorners[ii];

       // Setback corner might have split the edge
       SmBoolean bFindMultipleEdges = (pFilCorner->GetSetBackDist() > SM_EFF_ZERO)
                                      ? TRUE
                                      : FALSE;

       // for every filletEdge built by the corner
       for (jj=0; jj<pFilCorner->m_vEdges.GetSize(); jj++)
         {
           SmFilletEdge *pFE = pFilCorner->m_vEdges[jj];
           if (pFE)
             {
               // set every Corner FilletEdge->m_pFilletBrepEdge1 = FilletBrepEdge that has same geometry
               if (   pFE->GetOriginalFace() != NULL
                   || pFE->GetOriginalEdge() != NULL)
                 {
                   SER(pFE->FindFilletBrepEdge(sEdges,bFindMultipleEdges));
                 }
               //SmFilletEdgeType eFEType = pFE->GetFilletEdgeType();
               //if (eFEType == SM_FE_RAIL_RAIL_INTERPOLATION ||
               //    eFEType == SM_FE_ON_EDGE ||
               //    eFEType == SM_FE_ON_EXTEND_EDGE  ||
               //    eFEType == SM_FE_SETBACK_RAIL ||
               //    pFE->GetOriginalFace() != NULL) {
               //    SER(pFE->FindFilletBrepEdge(sEdges,bFindMultipleEdges));
               //}
             } // end pFE existence check
         } // end iter every filletCorner->FilletEdge
     } // end iter every FilletCornerSolver

   return SM_SUCCESS;

} // end SmFilletExecutive::SetFilletBrepEdgeMap

#define  SM_REVERSE_FILLET_EDGE(e) \
  { SmFilletEdgeuse * pPrimEU = (SmFilletEdgeuse*)(e)->GetPrimaryEdgeuse(); \
    e->Remove(pPrimEU); \
    e->PostInsert(pPrimEU); \
  }

#define SM_MODIFY_CORNEREDGE_FILLET_RELATION(e,fg) \
  { SmFilletEdgeuse * pPrimEU = (SmFilletEdgeuse*)(e)->GetPrimaryEdgeuse(); \
    SmFilletEdgeuse * pMateEU = (SmFilletEdgeuse*)pPrimEU->GetMate(); \
    if (pPrimEU->GetFilletGeom() == (fg)) \
      { pPrimEU->SetFilletGeom(pMateEU->GetFilletGeom()); } \
    pMateEU->SetFilletGeom(NULL); \
  }

/*******************************************************************//**
PURPOSE: Test a fillet geom next to a 3x2-tangent corner and see if
    it can be converted to a 3x3 corner to fix the big-radius case.
    This is an attempt to accommodate a large-radius fillet on a
    smaller-radius edge: the rolling ball will not fit in the corner.
    We can detect the big-radius case when two boundary cross-sectional edges
    'cross' each other (or two rails have 'opposite' orientations)

NOTES:
***********************************************************************/
SmStatus SmFilletExecutive::TestAndConvertBigRadFillet
 (SmFilletGeom * pFG,
  SmBoolean    & rbConvertedToCorner)
{
    rbConvertedToCorner = FALSE;

    // Collect info about the ends of the two rails:
    // start and end SmFilletVertex's and SmFilletCorner's,
    // and vectors from start to end of each.

    SmFilletVertex *pStartV[2] = { NULL, NULL };
    SmFilletVertex *pEndV[2] = { NULL, NULL };
    SmFilletCorner * pStartCorner = NULL;
    SmFilletCorner * pEndCorner = NULL;
    SmVector3d sRailDir[2];

    ULONG ii;
    for (ii=0; ii<2; ii++)
    {
        SmFilletEdge * pRail = pFG->GetRail(ii);
        SmEdgeuse * pPrimEU = pRail->GetPrimaryEdgeuse();
        SmEdgeuse * pMateEU = pPrimEU->GetMate();
        pStartV[ii] = (SmFilletVertex*)pPrimEU->GetVertexuse()->GetVertex();
        pEndV[ii] = (SmFilletVertex*)pMateEU->GetVertexuse()->GetVertex();
        if (ii==0)
        {
            pStartCorner = pStartV[ii]->GetFilletCorner(); NER(pStartCorner);
            pEndCorner = pEndV[ii]->GetFilletCorner(); NER(pEndCorner);
            if (pStartCorner->m_bTopologyAdjusted ||
                pEndCorner->m_bTopologyAdjusted) {
                // Unable to determine if the conversion can be achieved
                return SM_SUCCESS;
            }
        }
        if (  pStartCorner->GetCornerType() != SM_FCR_N_x_2
             || pEndCorner->GetCornerType() != SM_FCR_N_x_2
             || pStartV[ii]->IsProcessed() == FALSE
             ||   pEndV[ii]->IsProcessed() == FALSE
           )
        {
            return SM_SUCCESS;
        }
        SmPoint3d sPnt1 = pStartV[ii]->GetPoint();
        SmPoint3d sPnt2 = pEndV[ii]->GetPoint();
        sRailDir[ii] = sPnt2 - sPnt1;
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
        if (bDebugMe) {
            smgfx_SetLook(4,6, 1,0,0); sPnt1.Draw(); sm_GraphicsLoop();
            smgfx_SetLook(4,6, 0,0,1); sPnt2.Draw(); sm_GraphicsLoop();
            sm_GraphicsLoop();
        }
#endif // SM_DEBUG_CODE
    }

    // Check whether the ends of the rails go in generally the same direction;
    // if not, then it's twisting back on itself, as in a too-tight corner.
    // Also, if one rail is stationary, that's not the large-radius case.

    double dLen1 = sRailDir[0].Length();
    double dLen2 = sRailDir[1].Length();
    SmFilletSolver * pFS = pFG->GetFilletSolver();
    double dTol = pFS->GetThisApproxTol3d();
    if (    dLen1 <= dTol
         || dLen2 <= dTol
         || sRailDir[0].Dot( sRailDir[1] ) > 0.0
       )
    {
        // Valid FG, two rails
        return SM_SUCCESS;
    }

    // Two rails go on opposite directions, sqeeze one of the
    // rail and recompute its vertex geometry
    ULONG lRailToSqeeze = 0;
    if ( pStartV[lRailToSqeeze]->GetFilletVertexType() != SM_FV_MATE
        || pEndV[lRailToSqeeze]->GetFilletVertexType() != SM_FV_MATE
       )
    {
        lRailToSqeeze = 1;
        if (  pStartV[lRailToSqeeze]->GetFilletVertexType() != SM_FV_MATE
             || pEndV[lRailToSqeeze]->GetFilletVertexType() != SM_FV_MATE
           )
        {
            SER(SM_ERR);
        }
    }

    SM_ASSERT((pFG->GetRail(lRailToSqeeze)) != NULL) ; delete (pFG->GetRail(lRailToSqeeze)) ;
    pFG->SetRail(lRailToSqeeze,NULL);

    SmTArray<SmVertexuse*> sVUs;
    pEndV[lRailToSqeeze]->GetVertexuses(sVUs);
    for (ULONG j=0; j<sVUs.GetSize(); j++) {
        SmVertexuse * pVU = sVUs[j];
        pEndV[lRailToSqeeze]->Remove(pVU);
        pStartV[lRailToSqeeze]->PostInsert(pVU);
    }

    SM_ASSERT(pEndV[lRailToSqeeze] != NULL) ; delete pEndV[lRailToSqeeze] ; pEndV[lRailToSqeeze] = NULL ;
    pStartV[lRailToSqeeze]->SetFilletVertexType(SM_FV_RAIL_X_RAIL);
    for (ii=0; ii<2; ii++) {
        SmFilletVertex * pMate = pStartV[lRailToSqeeze]->GetMate(ii);
        if (pMate) {
            if (pStartV[lRailToSqeeze] == pMate->GetMate(0)) {
                pMate->SetMate(0,NULL);
            }
            else if (pStartV[lRailToSqeeze] == pMate->GetMate(1)) {
                pMate->SetMate(1,NULL);
            }
        }
    }
    pStartV[lRailToSqeeze]->SetMate(0,NULL);
    pStartV[lRailToSqeeze]->SetMate(1,NULL);
    pStartV[lRailToSqeeze]->SetStatus(SM_FIL_UNPROCESSED);
    // Compute vertex geometry
    pStartV[lRailToSqeeze]->CalcRailIntRail();

    // Make rail an edge for the new corner
    SmFilletEdge * pRail = pFG->GetRail(1-lRailToSqeeze);
    pRail->SetFilletEdgeType(SM_FE_CROSS_SECTION);
    SmFilletEdgeuse * pPrimEU1 = (SmFilletEdgeuse*)pRail->GetPrimaryEdgeuse();
    SmFilletEdgeuse * pMateEU1 = (SmFilletEdgeuse*)pPrimEU1->GetMate();
    pPrimEU1->SetFilletGeom(NULL);
    pMateEU1->SetFilletGeom(NULL);
    pFG->SetRail(1-lRailToSqeeze,NULL);

    SmFilletCorner * pNewCorner = new(m_crContext) SmFilletConvexNxNCorner(
        m_crContext,NULL,NULL,NULL,this,dTol,pFS->GetTangencyTolerance());
    SmObjDelete sDelete(pNewCorner);

    // Insert start & end vertices to the new corner
    pStartV[lRailToSqeeze]->SetFilletCorner(pNewCorner);
    pStartV[1-lRailToSqeeze]->SetFilletCorner(pNewCorner);
    pEndV[1-lRailToSqeeze]->SetFilletCorner(pNewCorner);
    pNewCorner->m_vVertices.Add(pStartV[1-lRailToSqeeze]);
    pNewCorner->m_vVertices.Add(pEndV[1-lRailToSqeeze]);
    pNewCorner->m_vVertices.Add(pStartV[lRailToSqeeze]);

    // Insert remaining rail & 2 cross-section's to
    // the new corner in an oriented order
    pRail->SetFilletCorner(pNewCorner);
    pNewCorner->m_vEdges.Add(pRail);
    if (pEndCorner->m_vEdges.GetSize() != 1) {
        SER(SM_ERR);
    }
    SmFilletEdge * pEndCrossSection = pEndCorner->m_vEdges[0];
    if (pEndCrossSection->GetVertex() != pEndV[1-lRailToSqeeze]) {
        // Reverse the edge
        SM_REVERSE_FILLET_EDGE(pEndCrossSection);
    }
    SM_MODIFY_CORNEREDGE_FILLET_RELATION(pEndCrossSection,pFG);
    pEndCorner->m_vVertices.ReSet();
    pEndCrossSection->SetFilletCorner(pNewCorner);
    pNewCorner->m_vEdges.Add(pEndCrossSection);

    if (pStartCorner->m_vEdges.GetSize() != 1) {
        SER(SM_ERR);
    }
    SmFilletEdge * pStartCrossSection = pStartCorner->m_vEdges[0];
    if (pStartCrossSection->GetVertex() != pStartV[lRailToSqeeze]) {
        // Reverse the edge
        SM_REVERSE_FILLET_EDGE(pStartCrossSection);
    }
    SM_MODIFY_CORNEREDGE_FILLET_RELATION(pStartCrossSection,pFG);
    pStartCorner->m_vVertices.ReSet();
    pStartCrossSection->SetFilletCorner(pNewCorner);
    pNewCorner->m_vEdges.Add(pStartCrossSection);

    // Delete two original corners (pStartCorner & pEndCorner) and
    // remove them from the corners array.  Insert the new corner
    // to the empty slot of the array with the lower index.
    // Note: this will avoid revisiting the same corner again from the caller.
    ULONG lFoundIndex1, lFoundIndex2;
    if (m_vFilletCorners.FindElement(pStartCorner,lFoundIndex1) == FALSE) {
        SER(SM_ERR);
    }
    if (m_vFilletCorners.FindElement(pEndCorner,lFoundIndex2) == FALSE) {
        SER(SM_ERR);
    }
    if (lFoundIndex1 > lFoundIndex2) {
        SM_SWAP(ULONG,lFoundIndex1,lFoundIndex2);
    }
    m_vFilletCorners.SetAt(lFoundIndex1,pNewCorner);
    m_vFilletCorners.RemoveAt(lFoundIndex2);
    const SmVertex * pOrigVertS = pStartCorner->GetFilletedVertex(); NER(pOrigVertS);
    const SmVertex * pOrigVertE = pEndCorner->GetFilletedVertex(); NER(pOrigVertE);
    //SmFilletCorner * p1 = (SmFilletCorner*)m_vVertexCornerMap.At(pOrigVertS);
    //SmFilletCorner * p2 = (SmFilletCorner*)m_vVertexCornerMap.At(pOrigVertE);
    m_vVertexCornerMap.Insert((SmVertex*) pOrigVertS, (SmObject*) pNewCorner);
    m_vVertexCornerMap.Insert((SmVertex*) pOrigVertE, (SmObject*) pNewCorner);

    SM_ASSERT(pStartCorner != NULL) ; delete pStartCorner ; pStartCorner = NULL ;
    SM_ASSERT(pEndCorner   != NULL) ; delete pEndCorner ;   pEndCorner   = NULL ;

    // Delete fillet solver
    ULONG lFoundIndex;
    if (m_vFilletSolvers.FindElement(pFS,lFoundIndex) == FALSE) {
        SER(SM_ERR);
    }
    m_vFilletSolvers.RemoveAt(lFoundIndex);
    SM_ASSERT(pFS != NULL) ; delete pFS ; pFS = NULL ;

    rbConvertedToCorner = TRUE;
    sDelete.Clear();

    return SM_SUCCESS;
}  // end SmFilletExecutive::TestAndConvertBigRadFillet()

/*******************************************************************//**
PURPOSE: Check whether any split Face pairs have to be updated.

NOTES:
   Imprinting a FilletEdge can split a Face.  If there were other FilletEdges
   or FilletCorners defined on the original Face, they might be defined on the
   portion of the Face that is now a new Face.  In that case, move it to the new Face.
***********************************************************************/
SmStatus SmFilletExecutive::UpdateTopologyChanges(
                SmAttribute  *pFaceAttr, // in: Attribute whose users are the original and split Faces
                SmFace *pOrigFace )      // in: the Face that pThisRail was in
{
  SmTArray< SmAObject* > sAllObjects;
  pFaceAttr->GetUsers( sAllObjects );
  if ( sAllObjects.GetSize() < 2 )
    { return SM_SUCCESS; }

  // Collect all new Faces in an array.
  SmTArray< SmFace* > sFaces;
  for ( ULONG ii=0; ii<sAllObjects.GetSize(); ii++ )
    {
      SmFace *pF = SM_CAST_PTR( SmFace, sAllObjects[ii] );
      if ( pF != NULL && pF != pOrigFace )
        { sFaces.Add( pF ); }
    }
  if ( sFaces.GetSize() < 1 )
    { return SM_SUCCESS; }

  // We have to look at all FilletSolvers (FilletGeoms) and all FilletCorners.
  SmTArray<SmFilletSolver*> & rSolvers = this->GetFilletSolvers();
  SmTArray<SmFilletGeom*> rFGs;

  for ( ULONG ii=0; ii<rSolvers.GetSize(); ii++ )
  {
      rSolvers[ii]->GetFilletGeoms( rFGs );

      for ( ULONG jj=0; jj<rFGs.GetSize(); jj++ )
      {
          SmFilletGeom *pFilGeom = rFGs[jj];

          for ( ULONG kk=0; kk < sFaces.GetSize(); kk++ )
          {
              SmFace *pNewFace = sFaces[kk];
              pFilGeom->UpdateSplitFace( pOrigFace, pNewFace );
          }

      } // end for every FilletGeom in this FilletSolver
  } // end for every FilletSolver

  // FilletCorners:
  SmTArray<SmFilletCorner*> & rCorners = this->GetFilletCorners();
  for ( ULONG ii=0; ii<rCorners.GetSize(); ii++ )
  {
      for ( ULONG kk=0; kk < sFaces.GetSize(); kk++ )
      {
          SmFace *pNewFace = sFaces[kk];
          rCorners[ii]->UpdateSplitFace( pOrigFace, pNewFace );
      }
  } // end for every FilletCorner

  return SM_SUCCESS;

} // end SmFilletExecutive::UpdateTopologyChanges

/*******************************************************************//**
PURPOSE: Find all sequences of smoothly-meeting Filleted Edges,
   and combine all of their curves into a single curve.

NOTES: This ensures proper behavior when a FilletSolver has to process
   its Filleted Edge outside of the Edge's domain.
***********************************************************************/
SmStatus SmFilletExecutive::ProcessSmoothEdgeSequences()
{
  // Copy the FilletSolver pointers into a separate array.
  // CollectTangentEdgeSequence() will identify one smooth sequence
  // and put it into sSmoothSequence, in order.  It will also Null out
  // some or all FilletSolver pointers that are not in smooth sequences.
  // So just continue until it's empty.

  SmTArray< SmFilletSolver*> sFilletSolvers( m_vFilletSolvers ); // copy ptrs
  SmTArray< SmFilletSolver*> sSmoothSequence;

  while ( sFilletSolvers.GetSize() > 1 )  // If only 1, no work.
  {
      SER( this->CollectTangentEdgeSequence( sFilletSolvers,      // in/out
                                             sSmoothSequence ));  // out
      SER( this->CreateSmoothEdgeSequence( sSmoothSequence ));   // in
  }
  return SM_SUCCESS;

}  // end SmFilletExecutive::ProcessSmoothEdgeSequences

/*******************************************************************//**
PURPOSE: Find a sequence of smoothly-meeting Filleted Edges.

NOTES: Puts a sequence into sSmoothSequence array.  Removes those, plus some
   FilletSolvers that are not in smooth sequences, from sFilletSolvers.
***********************************************************************/
SmStatus SmFilletExecutive::CollectTangentEdgeSequence(
                                SmTArray< SmFilletSolver* > & rFilletSolvers,    // in/out
                                SmTArray< SmFilletSolver* > & rSmoothSequence )  // out
{
  rSmoothSequence.ReSet();

  if ( rFilletSolvers.GetSize() < 2 )
    { return SM_SUCCESS; }

  // Check whether these FilletSolvers are in a Law sequence.
  // They would have been put there by CreateSequenceOfLaws(), called by the user.
  // If so, rSmoothSequence will begin at the start of that sequence, traversing forwards.

  SmEdge* pStartEdge = NULL;

  for ( ULONG ii=0; ii<m_sLawSequenceEdgeList.GetSize(); ii++ )
  {
      SmTArray< SmEdge* > &rThisList = m_sLawSequenceEdgeList[ii];

      for ( ULONG jj=0; jj<rFilletSolvers.GetSize(); jj++ )
      {
          SmEdge *pThisEdge = rFilletSolvers[jj]->GetEdgeuse(0)->GetEdge();

          if ( rThisList.IsIn( pThisEdge ) )
          {
              pStartEdge = rThisList[0];  // First in list.
              break;
          }
      }
      if ( pStartEdge != NULL )
        { break; }
  }

  SmFilletCorner *pCorner0, *pCorner1, *pCurrCorner;
  SmFilletSolver *pCurrFS = NULL, *pPrevFS = NULL;
  ULONG lIdx = 0;

  for ( ULONG ii=0; ii< rFilletSolvers.GetSize(); ii++ )
  {
      SmFilletSolver *pFS = rFilletSolvers[ii];
      if ( pFS == NULL ) { continue; }

      rFilletSolvers[ii] = NULL;

      // Traverse backwards, unless pStartEdge is given.
      if ( pStartEdge == NULL )
      {
          pCorner0 = pFS->GetFilletCorner( 0 );
          NER( pCorner0 );

          pCurrFS = pFS;
          pPrevFS = pCorner0->GetTangentFilletSolver( pFS );

          if ( pPrevFS == pCurrFS )
            { continue; } // Single closed loop.

          // If there is one, put the current FS into the list.
          if ( pPrevFS != NULL )
            { rSmoothSequence.Add( pFS ); }

          pCurrCorner = pCorner0;

          while ( pPrevFS != NULL && pPrevFS != pFS )
          {
              // Add it to the list, and remove it from the main list.
              rSmoothSequence.InsertAt( 0, pPrevFS );
              rFilletSolvers.FindElement( pPrevFS, lIdx );
              rFilletSolvers[lIdx] = NULL;

              SmFilletCorner *pPrevCorner = pPrevFS->GetOtherFilletCorner( pCurrCorner );
              pCurrFS = pPrevFS;
              pPrevFS = pPrevCorner->GetTangentFilletSolver( pCurrFS );
              pCurrCorner = pPrevCorner;
          }
      } // end if pStartEdge not given, traverse backwards.


      // Traverse forwards.
      if ( pStartEdge != NULL )
        { pFS = this->GetFilletSolverOfEdge( pStartEdge );  NER( pFS ); }

      pCorner0 = pFS->GetFilletCorner( 0 );
      pCorner1 = pFS->GetFilletCorner( 1 );
      NER( pCorner1 );

      if ( pCorner1 == pCorner0 )
        { continue; } // Single closed loop.

      pCurrFS = pFS;
      SmFilletSolver *pNextFS = pCorner1->GetTangentFilletSolver( pFS );
      if ( pNextFS != NULL  &&  rSmoothSequence.GetSize() == 0 )
        { rSmoothSequence.Add( pFS ); }

      if ( pNextFS == pCurrFS )
        { continue; }

      if ( rSmoothSequence.IsIn( pNextFS ) )
        { break; }  // Already got this one: must be a closed loop.

      pCurrCorner = pCorner1;

      while ( pNextFS != NULL && pNextFS != pFS )
      {
          // Add it to the list, and remove it from the main list.
          rSmoothSequence.Add( pNextFS );
          rFilletSolvers.FindElement( pNextFS, lIdx );
          rFilletSolvers[lIdx] = NULL;

          SmFilletCorner *pNextCorner = pNextFS->GetOtherFilletCorner( pCurrCorner );
          pCurrFS = pNextFS;
          pNextFS = pNextCorner->GetTangentFilletSolver( pCurrFS );
          pCurrCorner = pNextCorner;
      }

      // If we have a sequence, we're done.
      if ( rSmoothSequence.GetSize() > 1 )
        { break; }

  } // end for each FilletSolver

  rFilletSolvers.CompressZeros();

  return SM_SUCCESS;

} // end CollectTangentEdgeSequence

/*******************************************************************//**
PURPOSE: Find all sequences of smoothly-meeting Filleted Edges,
   and combine all of their curves into a single curve.

NOTES: This ensures proper behavior when a FilletSolver has to process
   its Filleted Edge outside of the Edge's domain.
***********************************************************************/
SmStatus SmFilletExecutive::CreateSmoothEdgeSequence( SmTArray< SmFilletSolver* > & rSmoothSequence )
{
  if ( rSmoothSequence.GetSize() < 2 )
    { return SM_SUCCESS; }

  // Work with SmEdges ... mainly so we can use SmEdge::OrientEdgeSequence().

// Remove Composites
//  // What if it is already an SmCEdge?  Then the curves are the same curves [B570, alt 0]
//  // Then we're already set up properly, and don't have to do anything.
//  //  Unhandled case: Smooth Sequence where part of it is CEdge, part is not.
//  //    If all edges in list are in the same CEdge, then don't need to do anything.
//  //    If some are in CEdge but not all, then break up CEdge and proceed.
//  //  Or: Just skip this if there is any CEdge.

  SmTArray< SmEdge* > sEdges;
  for ( ULONG ii=0; ii<rSmoothSequence.GetSize(); ii++ )
  {
      SmFilletSolver *pFS = rSmoothSequence[ii];
      SmEdge *pE = pFS->GetEdgeuse(0)->GetEdge();

// Remove Composites
//      // Check for CEdge:
//      if ( pE->GetCurve()->GetOwner() != pE )
//        { return SM_SUCCESS; }

      sEdges.Add( pE );
  }

  // Make sure that the edges are all oriented in the direction of the sequence.
  // This calls ReverseOrientation() as necessary.
  SmEdge::OrientEdgeSequence( sEdges );

  SmExtent1d sEdgeIvl, sPrevIvl;
  SmTArray< SmBSplineCurve* > sBSCurves;

  ULONG lNumEdges = sEdges.GetSize();

  for ( ULONG ii=0; ii<lNumEdges; ii++ )
  {
      SmEdge *pEdge = sEdges[ii];
      sEdgeIvl = pEdge->GetInterval();
      SmCurve *pCrv = pEdge->GetCurve();

      // Make the parameterizations consecutive, and match the Edge's curve.
      // cbi todo: don't trim beginning of 1st or end of last, unless it's closed.
      // We don't want curves extending into adjacent Edges.
      // They will be Joined at the Edge boundaries.
      pCrv->Trim( sEdgeIvl );

      if ( ii > 0 )
      {
          // Make the curves meet with true C1 continuity.
          SmEdge *pPrevE = sEdges[ii-1];

          SmCurve *pPrevCrv = pPrevE->GetCurve();
          SmCurve *pCurrCrv = pEdge->GetCurve();
          double dPrevParam1 = sPrevIvl.GetMax();
          double dCurrParam0 = sEdgeIvl.GetMin();
          SmVector3d sPtDerPrev[2], sPtDerCurr[2];
          pPrevCrv->Evaluate( dPrevParam1, 1, FALSE, sPtDerPrev );
          pCurrCrv->Evaluate( dCurrParam0, 1, FALSE, sPtDerCurr );
          SM_ASSERT( sPtDerPrev[1].IsParallelTo( sPtDerCurr[1],
              SM_RAD2DEG( rSmoothSequence[ii]->GetTangencyTolerance()) ));
          double dPrevLen = sPtDerPrev[1].Length();
          double dCurrLen = sPtDerCurr[1].Length();

          // Keep the Min of the current interval the same, scale its Max by the deriv length ratio.
          double dScale = dCurrLen / dPrevLen;
          double dScaledEnd = sEdgeIvl.GetMin() + dScale * sEdgeIvl.GetLength();

          sEdgeIvl.SetMinMax( sEdgeIvl.GetMin(), dScaledEnd );

          // Domains are the right lengths, make sure they're consecutive.
          if ( ! SM_ARE_SAME( sEdgeIvl.GetMin(), sPrevIvl.GetMax() ) )
          {
              sEdgeIvl.Translate( sPrevIvl.GetMax() - sEdgeIvl.GetMin() );
          }

          pCrv->EditParameterization( sEdgeIvl );
          pEdge->SetInterval( sEdgeIvl );
      }

      // Also collect the Curves.
      SmBSplineCurve *pBSC = SM_CAST_PTR( SmBSplineCurve, pEdge->GetCurve() );
      sBSCurves.Add( pBSC );

      sPrevIvl = sEdgeIvl;
  }

  // Join all of the curves, in order.
  SmBSplineCurve *pCompCurve = NULL;
  SmBSplineCurve::CreateByJoining( m_crContext, sBSCurves, NULL, pCompCurve );
  NER( pCompCurve );

  // Set the Joined curve as each Edge's curve.
  SmCurve *pNewCurve = NULL;
  for ( ULONG ii = 0; ii<lNumEdges; ii++ )
  {
      SmEdge *pE = sEdges[ii];

      if ( ii == 0 )
        { pNewCurve = pCompCurve; }
      else
        { pCompCurve->Copy( m_crContext, pNewCurve ); }

      // We're going to delete the Edges' curves.  Those are probably the same curve
      // that is stored in a CurveBasedFilletSolver's m_pOriginal slot.
      // If so, replace it.
      SmCurveBasedFS *pCrvFS = SM_CAST_PTR( SmCurveBasedFS, rSmoothSequence[ii] );
      if ( pCrvFS != NULL )
      {
          if ( pCrvFS->GetOriginalCurve() == pE->GetCurve() )
            { pCrvFS->SetOriginalCurve( pNewCurve ); }  // Does not delete its current OriginalCurve.
                                                        // That will be deleted next, when we reset the Edge's curve.
      }

      // If it's a VariableRadius FS, it also has m_vLawIvl which should be the same as the Edge domain.
      SmVariableRadiusFS *pVarRadFS = SM_CAST_PTR( SmVariableRadiusFS, rSmoothSequence[ii] );
      if ( pVarRadFS != NULL )
      {
          pVarRadFS->SetFilletLawInterval( pE->GetInterval() );
      }

      pE->SetCurve( pNewCurve, TRUE );  // TRUE: Delete existing curve.
  }

  return SM_SUCCESS;
} // end CreateSmoothEdgeSequence

/*******************************************************************//**
PURPOSE: Given a Fillet Law and a list of smoothly-joining edges,
   extend the given Law across all of the Edges in the sequence.

NOTES: 
   The Law might be a variable-radius function which is intended to be
   applied over more than one smoothly-meeting Edge.

   The Edges must meet smoothly, so that a fillet will roll smoothly
   from one to the next.  They must be ordered, in the direction of the fillet.
   The laws returned are in the same order in their array.

   The input pLaw is used in the first of the output laws.
   A copy of the Law is created for each Edge in the list.

   To use: First create an SmFilletLaw, pLaw, and an SmTArray< SmEdge* > sFilletEdges
   containing the ordered sequence of edges to be filleted.  They must be in order,
   but you don't have to worry about their orientations.


   SmTArray< SmFilletLaw* > sLawList;
   SmStatus eStat = SmFilletLaw::CreateSequenceOfLaws( *pContext, pLaw, sFilletEdges, sLawList );

   SmCircularCrossSectionFSG sFSG( TRUE, 0.001 ),  // (for example)

   for ( ii=0; ii<lNumEdges; ii++ )
   {
       SmEdgeuse *pEU = sFilletEdges[ii]->GetBlendEdgeuse( );

       SmVariableRadiusFS *pFS1 = new(*pContext) SmVariableRadiusFS(    // (for example)
                    *pContext, 0.001, SM_DEGREES_TO_RADIANS(30.0), SM_DEGREES_TO_RADIANS(2.0),
                    1.0, pEU, *( sLawList[ii] ), FALSE );

       // Set up the cross section generator and the Fillet executive.
       // Creates circular cross section fillet
       pFS1->SetFilletSurfaceGenerator( &sFSG );

       pFilExec->LoadFilletSolver(pFS1);
   }

   Side effect: calls OrientEdgeSequence(), so the Edges in the sequence will all
   be oriented in the direction of the sequence.  (Should not be a problem, the
   edges will disappear in the Fillet operation anyway.)

   Side effect: Adds an array of SmEdge pointers to our m_sLawSequenceEdgeList.  [B695]
   That is used in CollectTangentEdgeSequence(), called by ProcessSmoothEdgeSequences().
   If present, the tangent Edges will be collected in the order of the Edges in that list.
***********************************************************************/
SmStatus SmFilletExecutive::CreateSequenceOfLaws
(
  const SmContext & crContext,
  SmFilletLaw *pLaw,                         ///< [in] : the single Law to be copied on each Edge     <br>
  SmTArray< SmEdge* > & rEdges,              ///< [in] : the Edge sequence, ordered                   <br>
  SmTArray< SmFilletLaw* > & rLaws           ///< [out]: one Law for each input Edge                  <br>
)
{
  // First call the SmFilletLaw version to do the main work.
  SmStatus eStat = SmFilletLaw::CreateLawSequence( crContext, pLaw, rEdges, rLaws );
  if ( eStat != SM_SUCCESS )
    { SER( eStat ); }

  // Add this ordered list of Edges to our array of Law sequences.  [B695]
  m_sLawSequenceEdgeList.Add( rEdges );

  return SM_SUCCESS;

}  // end SmFilletExecutive::CreateSequenceOfLaws


/*******************************************************************//**
PURPOSE: Record a fillet error on a fillet edge.

NOTES: Note that it's a fillet edge -- newly created, in the
    Fillet Brep -- and not a filleted edge of the original target Brep.
***********************************************************************/
void SmFilletExecutive::NoteFilletError
 (SmFilletErrorType  eErrorType,  // in :
  SmFilletEdge     * pFilletEdge, // in :
  const TCHAR      * cComment)    // in :
{
  TCHAR cEdgeName[SM_TBLOCK_SIZE];
  pFilletEdge->GetName( cEdgeName, SM_TBLOCK_SIZE );

  SmFilletEdgeType eFEType     = pFilletEdge->GetFilletEdgeType();
  SmFilStatus      eFilletStat = pFilletEdge->GetStatus();

  // pass the call along to the SmFilletErrorInfo object 
  GetFilletErrorInfo()->NoteError(eErrorType, 
                                  eFilletStat, 
                                  eFEType, 
                                  pFilletEdge, 
                                  cEdgeName, 
                                  (TCHAR *)cComment );

} // end SmFilletExecutive::NoteFilletError

/*******************************************************************//**
PURPOSE: Return the index of a given FilletCorner in this Executive.

NOTES: Used in debugging.
    Returns error 9999 if the given FilletCorner is not in our list.
***********************************************************************/
ULONG SmFilletExecutive::FindCornerIndex( const SmFilletCorner *pCorner ) const
{
    if ( pCorner == NULL ) { return 9999; }
    ULONG lIdx;
    if ( m_vFilletCorners.FindElement( SM_CONST_CAST(SmFilletCorner*, pCorner), lIdx ) )
      { return lIdx; }
    return 9999; // Error return
} // end SmFilletExecutive::FindCornerIndex

/*******************************************************************//**
PURPOSE: Return the index of a given FilletSolver in this Executive.

NOTES: Used in debugging.
    Returns error 9999 if the given FilletSolver is not in our list.
***********************************************************************/
ULONG SmFilletExecutive::FindSolverIndex( const SmFilletSolver *pSolver ) const
{
    if ( pSolver == NULL ) { return 9999; }
    ULONG lIdx;
    if ( m_vFilletSolvers.FindElement( SM_CONST_CAST(SmFilletSolver*, pSolver), lIdx ) )
      { return lIdx; }
    return 9999; // Error return

} // end SmFilletExecutive::FindSolverIndex


/*******************************************************************//**
PURPOSE: Set up this object: set the radius to be constant, at the
   value at the proper end.

NOTES: Called by the constructor.
***********************************************************************/
SmStatus SmTemporaryRadiusChange::SetUp()
{
  // Indicate no change, to start.
  m_pSavedBSp = NULL;
  m_dStartRad = m_dEndRad = -1.0;

  SmVariableRadiusFS * pVRFS = SM_CAST_PTR( SmVariableRadiusFS, m_pFS );
  if ( pVRFS == NULL )
    { return SM_SUCCESS; }

  if ( ! pVRFS->OffsetRadiiCanChange() )
    { return SM_SUCCESS; }

  SmFilletLaw *pLaw = pVRFS->GetFilletLaw();

  // Do SmBSplineFilletLaw and SmLinearFilletLaw.
  SmBSplineFilletLaw * pBSpLaw = SM_CAST_PTR( SmBSplineFilletLaw, pLaw );
  SmLinearFilletLaw  * pLinLaw = SM_CAST_PTR( SmLinearFilletLaw , pLaw );

  if ( pBSpLaw == NULL && pLinLaw == NULL )
    { return SM_SUCCESS; }

  // Get the radius at this end.
  // Find which end.
  SmBoolean bAtStart = UNSURE;
  SmVertex *pEdgeV1, *pEdgeV2, *pSideV1, *pSideV2;
  SmEdgeuse *pEU = m_pFS->GetEdgeuse( m_lRailIdx );    NER( pEU ); // ... both should have the same owner...
  SmEdge *pE = SM_CAST_PTR( SmEdge, pEU->GetOwner() ); NER( pE  );
  pEdgeV1 = pE->GetStartVertex();
  pEdgeV2 = pE->GetOtherVertex( pEdgeV1 );

  SmEdge *pSideEdge = SM_CAST_PTR( SmEdge, m_pSideEU->GetOwner() ); NER( pSideEdge );
  pSideV1 = pSideEdge->GetStartVertex();
  pSideV2 = pSideEdge->GetOtherVertex( pSideV1 );

  if ( pEdgeV1 != pEdgeV2 ) // non-closed edge
    {
      if ( pEdgeV1 == pSideV1 || pEdgeV1 == pSideV2 )
        { bAtStart = TRUE; }
      if ( pEdgeV2 == pSideV1 || pEdgeV2 == pSideV2 )
        { bAtStart = FALSE; }
    }
  else
    {
      // Shouldn't matter: the fillet is closed; better have same start & end radii.
      bAtStart = TRUE;
    }

  if ( pVRFS->GetFilletLawOrientation() == TRUE )
    { bAtStart = ! bAtStart; }

  // Don't do anything if this is an interior location.  [B570]
  double dLawT = ( bAtStart ) ? pLaw->m_vEdgeMap.GetMin() : pLaw->m_vEdgeMap.GetMax();
  SmExtent1d sUnitIvl( 0, 1 );
  if ( ! sUnitIvl.IsValueOnBoundary( dLawT, SM_EFF_ZERO ) )
    { return SM_SUCCESS; }

  // Evaluate the radius at the proper end.
  SmExtent1d sEdgeIvl = pVRFS->GetFilletLawInterval();

  // Don't do anything if this has no range
  if( sEdgeIvl.GetLength() <= 0.0 )
    { return SM_SUCCESS; }

  double dEdgeT = ( bAtStart ) ? sEdgeIvl.GetMin() : sEdgeIvl.GetMax();

  SmBoolean bRev = FALSE;  // Already checked this, and don't care about derivs.
  double adValues[3];
  pLaw->Evaluate( dEdgeT, sEdgeIvl, bRev, adValues );
  double dEndRad = adValues[0];

  if ( pBSpLaw != NULL )
    {
      // Create a law curve that's constant at that value.
      SmPoint3d sPt1( dEndRad, 1, 0 );
      SmPoint3d sPt2( dEndRad, 2, 0 );
      SmBSplineCurve *pConstBSp = NULL;
      SmBSplineCurve::CreateLineSegment( *( pBSpLaw->GetContext() ), 3, sPt1, sPt2, pConstBSp );
      NER( pConstBSp );

      // The rad function should always be parameterized on [0-1].
      pConstBSp->EditParameterization( sUnitIvl );

      // Swap the new radius function info in.
      m_pSavedBSp = pBSpLaw->GetLawCurve();
      pBSpLaw->SetLawCurve( pConstBSp );

      SmBSplineCurve *pOldExtended = pBSpLaw->GetExtendedCurve();
      if ( pOldExtended != NULL )
        {
          pBSpLaw->SetExtendedCurve( NULL );
          delete pOldExtended;
        }
    }
  else if ( pLinLaw != NULL )
    {
      m_dStartRad = pLinLaw->GetStartRadius();
      m_dEndRad   = pLinLaw->GetEndRadius();

      pLinLaw->SetStartRadius( dEndRad );
      pLinLaw->SetEndRadius  ( dEndRad );
    }

  return SM_SUCCESS;

}  // end SmTemporaryRadiusChange::Setup

/*******************************************************************//**
PURPOSE: Revert the radius function back to its original variable radius.

NOTES: Called by the destructor.
   If m_psavedBSp is not Null, this was a SmBSplineFillet Law.
   If m_dStartRad and m_dEndRad are both non-negative, this was a SmLinearFillet Law.
   Otherwise, it was unset and did nothing, and there's nothing to do.
***********************************************************************/
SmStatus SmTemporaryRadiusChange::Revert()
{
  SmVariableRadiusFS * pSavedVRFS = SM_CAST_PTR( SmVariableRadiusFS, m_pFS );
  if ( pSavedVRFS == NULL )
    { return SM_SUCCESS; }

  SmFilletLaw *pLaw = pSavedVRFS->GetFilletLaw();

  if ( m_pSavedBSp != NULL )
    {
      SmBSplineFilletLaw * pBSpLaw = SM_CAST_PTR( SmBSplineFilletLaw, pLaw );
      if ( pBSpLaw == NULL )
        { return SM_SUCCESS; }

      SmBSplineCurve *pConstBSP = pBSpLaw->GetLawCurve();
      pBSpLaw->SetLawCurve( m_pSavedBSp );
      if ( pConstBSP != NULL ) { delete ( pConstBSP ); pConstBSP = NULL; }

      SmBSplineCurve *pExtBSp = pBSpLaw->GetExtendedCurve();
      pBSpLaw->SetExtendedCurve( NULL );
      if ( pExtBSp != NULL ) { delete ( pExtBSp ); pExtBSp = NULL; }
    }

  else if ( m_dStartRad > -SM_EFF_ZERO && m_dEndRad > -SM_EFF_ZERO ) // 0.0 is ok...
    {
      SmLinearFilletLaw * pLinLaw = SM_CAST_PTR( SmLinearFilletLaw, pLaw );
      if ( pLinLaw == NULL )
        { return SM_SUCCESS; }

      pLinLaw->SetStartRadius( m_dStartRad );
      pLinLaw->SetEndRadius  ( m_dEndRad );
    }

    return SM_SUCCESS;

} // end SmTempRadiusChange::Revert


/*******************************************************************//**
PURPOSE: Default Constructor for SmFilletLaw

NOTES:
***********************************************************************/
SmFilletLaw::SmFilletLaw
()
    : m_vEdgeMap(0.0, 1.0),
    m_dEdgeArcLength(0.0),
    m_pExtended(NULL)
{

} // end SmFilletLaw::SmFilletLaw constructor

/*******************************************************************//**
PURPOSE: Copy Constructor

NOTES:
***********************************************************************/
SmFilletLaw::SmFilletLaw(const SmFilletLaw & crOther)
{
  *this = crOther;

} // end SmFilletLaw::SmFilletLaw copy constructor

/*******************************************************************//**
PURPOSE: Destructor for SmFilletLaw

NOTES:
***********************************************************************/
SmFilletLaw::~SmFilletLaw()
{
    if (m_pExtended) { delete m_pExtended; m_pExtended = NULL; }

} // end SmFilletLaw::~SmFilletLaw destructor

/*******************************************************************//**
PURPOSE: Assignment operator

NOTES:
***********************************************************************/
SmFilletLaw & SmFilletLaw::operator=(const SmFilletLaw & crOther)
{
  m_vEdgeMap = crOther.m_vEdgeMap;
  m_dEdgeArcLength = crOther.m_dEdgeArcLength;
  m_pExtended = NULL;  // This gets created as needed.

  return *this;

} // end SmFilletLaw::operator=

/*******************************************************************//**
PURPOSE: Copy method.

NOTES: (Only way to make a virtual copy operator.)
***********************************************************************/
SmStatus SmFilletLaw::Copy
 (const SmContext & crContext,        // NotUsed: in :
  SmFilletLaw    *& rpNewFilletLaw)   // in :
 const
{
  SM_REF1(crContext) ;
  rpNewFilletLaw = NULL;
  SER(SM_ERR);  // Should not be called at this level, only for derived classes.
  return SM_ERR;  // (silence compiler warning)

} // end SmFilletLaw::Copy

/*******************************************************************//**
PURPOSE: Given a Fillet Law and a list of smoothly-joining edges,
   extend the given Law across all of the Edges in the sequence.

NOTES:
   This is now an internal method.  Users should call
   SmFilletExecutive::CreateSequenceOfLaws().  [B695]

   The Law might be a variable-radius function which is intended to be
   applied over more than one smoothly-meeting Edge.

   The Edges must meet smoothly, so that a fillet will roll smoothly
   from one to the next.  They must be ordered, in the direction of the fillet.
   The laws returned are in the same order in their array.

   The input pLaw is used in the first of the output laws.
   A copy of the Law is created for each Edge in the list.

   To use: First create an SmFilletLaw, pLaw, and an SmTArray< SmEdge* > sFilletEdges
   containing the ordered sequence of edges to be filleted.  They must be in order,
   but you don't have to worry about their orientations.

   Side effect: calls OrientEdgeSequence(), so the Edges in the sequence will all
   be oriented in the direction of the sequence.  (Should not be a problem, the
   edges will disappear in the Fillet operation anyway.)
***********************************************************************/
SmStatus SmFilletLaw::CreateLawSequence(
  const SmContext & crContext,
  SmFilletLaw *pLaw,                // in:  the single Law to be copied on each Edge
  SmTArray< SmEdge* > & rEdges,     // in:  the Edge sequence, ordered
  SmTArray< SmFilletLaw* > & rLaws  // out: one Law for each input Edge
)
{
  // Init output.
  rLaws.ReSet();

  // Make sure that the edges are all oriented in the direction of the sequence.
  // This calls ReverseOrientation() as necessary.
  SmEdge::OrientEdgeSequence( rEdges );

  // Calculate and accumulate the edge lengths.
  SmTArray< double > sLengths;
  double dTotalLength = 0;
  SmExtent1d sEdgeIvl;

  ULONG ii, lNumEdges = rEdges.GetSize();

  for ( ii = 0; ii<lNumEdges; ii++ )
  {
      SmEdge *pEdge = rEdges[ii];
      sEdgeIvl = pEdge->GetInterval();
      SmCurve *pCrv = pEdge->GetCurve();
      double dLen = pCrv->ApproximateLength(sEdgeIvl, 20);
      sLengths.Add(dLen);
      dTotalLength += dLen;
  }

  // Create separate laws with the appropriate EdgeMaps.
  double dStartParam = 0;
  SmFilletLaw *pNewLaw = NULL;
  for ( ii = 0; ii<lNumEdges; ii++ )
  {
      if ( ii == 0 )
        { pNewLaw = pLaw; }
      else
        { pLaw->Copy(crContext, pNewLaw); }

      double dFraction = sLengths[ii] / dTotalLength;
      double dEndParam = dStartParam + dFraction;
      if ( ii == lNumEdges-1 ) { dEndParam = 1.0; }  // Just to avoid roundoff.
      SmExtent1d sIvl( dStartParam, dEndParam );
      pNewLaw->SetEdgeMap( sIvl );
      pNewLaw->SetEdgeArcLength( sLengths[ii] );
      rLaws.Add( pNewLaw );

      dStartParam = dEndParam;
  }

  return SM_SUCCESS;

} // end SmFilletLaw::CreateLawSequence

/*******************************************************************//**
PURPOSE:
NOTES:
***********************************************************************/
SmBoolean SmFilletLaw::IsKindOf( SM_TYPE t ) const
{
  return ((SmFilletLaw_TYPE == t) ? TRUE : SmObject::IsKindOf( (t) ));
}

/*******************************************************************//**
PURPOSE: Dump a Fillet Law.

NOTES:
***********************************************************************/
void SmFilletLaw::Dump() const
{
  TCHAR sBuff[SM_TBLOCK_SIZE], sBuffForFile[SM_TBLOCK_SIZE];

  smos_sprintf(sBuff, _T("\n  Dump of SmFilletLaw 0x%p"), this);
  smos_sprintf(sBuffForFile, _T("%s"),_T("\n  Dump of SmFilletLaw"));
  smos_WriteBuffer(sBuff, sBuffForFile);

  smos_sprintf(sBuff,_T("%s"), _T("\n    m_vEdgeMap:"));
  smos_sprintf(sBuffForFile, _T("%s"), _T("\n    m_vEdgeMap:"));
  smos_WriteBuffer(sBuff, sBuffForFile);

  m_vEdgeMap.Dump();

  smos_sprintf(sBuff, _T("\n    m_dEdgeArcLength: %lf"), m_dEdgeArcLength);
  smos_sprintf(sBuffForFile, _T("\n    m_dEdgeArcLength: %lf"), m_dEdgeArcLength);
  smos_WriteBuffer(sBuff, sBuffForFile);

  smos_sprintf(sBuff, _T("\n    m_pExtended: 0x%p"), m_pExtended);
  smos_sprintf(sBuffForFile, _T("\n    m_pExtended: %s"), m_pExtended ? _T("notNULL") : _T("NULL"));
  smos_WriteBuffer(sBuff, sBuffForFile);

  if (FALSE)
  {
      m_pExtended->Dump();
  }

  smos_sprintf(sBuff,_T("%s"), _T("\n  End of SmFilletLaw Dump."));
  smos_sprintf(sBuffForFile, _T("%s"), _T("\n  End of SmFilletLaw Dump."));
  smos_WriteBuffer(sBuff, sBuffForFile);

  return;

} // end SmFilletLaw::Dump

/*******************************************************************//**
PURPOSE: Constructor for SmLinearFilletLaw

NOTES:
***********************************************************************/
SmLinearFilletLaw::SmLinearFilletLaw
(double dStartValue,                 // in : radius value at fillet edge start
        double dEndValue,            // in : radius value at fillet edge end
        SmExtent1d * pOptEdgeMap)    // in : 
    : SmFilletLaw(),
    m_dStartValue(dStartValue),
    m_dEndValue(dEndValue)
{
  if (pOptEdgeMap) {
      m_vEdgeMap = *pOptEdgeMap;
  }
  if (m_vEdgeMap.GetLength() < SM_EFF_ZERO) {
      m_vEdgeMap.SetMinMax(0, 1);
  }

} // end SmLinearFilletLaw::SmLinearFilletLaw constructor

/*******************************************************************//**
PURPOSE: Copy Constructor

NOTES:
***********************************************************************/
SmLinearFilletLaw::SmLinearFilletLaw(const SmLinearFilletLaw & crOther)
{
  *this = crOther;

} // end SmLinearFilletLaw copy constructor

/*******************************************************************//**
PURPOSE: Assignment operator

NOTES:
***********************************************************************/
SmLinearFilletLaw & SmLinearFilletLaw::operator=(const SmLinearFilletLaw & crOther)
{
  if (&crOther == this) { return *this; }
  SmFilletLaw::operator=(crOther);  // Do parent stuff.

  m_dStartValue = crOther.m_dStartValue;
  m_dEndValue = crOther.m_dEndValue;

  return *this;

} // end SmLinearFilletLaw::operator=

/*******************************************************************//**
PURPOSE: Copy method.

NOTES: (Only way to make a virtual copy operator.)
***********************************************************************/
SmStatus SmLinearFilletLaw::Copy
 (const SmContext & crContext,
  SmFilletLaw    *& rpNewFilletLaw) 
 const
{
  SmLinearFilletLaw *pNewLaw = new(crContext) SmLinearFilletLaw(*this);
  rpNewFilletLaw             = pNewLaw;

  return SM_SUCCESS;

} // end SmLinearFilletLaw::Copy

/*******************************************************************//**
PURPOSE: Evaluate a Linear Fillet Law.  Note that the evaluation will
work outside of the curve domain.

NOTES:
***********************************************************************/
SmStatus SmLinearFilletLaw::Evaluate
(double dParameter,                   // in : target parameter - range:[crCurveInterval.Min,Max]
  const SmExtent1d & crCurveInterval, // in : interval defining range of fillet edge
  SmBoolean bReverseOrientation,      // in : TRUE = parameters run from interval end to interval start
  double dValues[3])                  // out: dValues[0] = fillet-radius at dParameter value
                                      //      dValues[1] = 1st derivative: d(fillet-radius)/d(Parameter)
                                      //      dValues[2] = 2nd derivative: d2(fillet-radius)/d(Parameter)**2
const
{
  // Map dParameter into domain of Law which is contained in the edge map.
  double dMin = crCurveInterval.GetMin();
  double dMax = crCurveInterval.GetMax();
  double dTNorm = (bReverseOrientation)
                  ? (dMax - dParameter) / (dMax - dMin)
                  : (dParameter - dMin) / (dMax - dMin);
  double dT = m_vEdgeMap.GetMin() + dTNorm * (m_vEdgeMap.GetMax() - m_vEdgeMap.GetMin());

  // evaluate function, 1st, and 2nd deriviatives at param = dParameter
  dValues[0]            = m_dStartValue + dT                  * (m_dEndValue - m_dStartValue);
  double dEndMapValue   = m_dStartValue + m_vEdgeMap.GetMax() * (m_dEndValue - m_dStartValue);
  double dStartMapValue = m_dStartValue + m_vEdgeMap.GetMin() * (m_dEndValue - m_dStartValue);
  dValues[1] = (dEndMapValue - dStartMapValue) / crCurveInterval.GetLength();
  dValues[2] = 0.0;

  // negate 1st derivatives for reversed orientations
  if (bReverseOrientation)
  {
      dValues[1] = -dValues[1];
  }

  // all done
  return SM_SUCCESS;

} // end SmLinearFilletLaw::Evaluate

/*******************************************************************//**
PURPOSE:
NOTES:
***********************************************************************/
SmBoolean SmLinearFilletLaw::IsKindOf( SM_TYPE t ) const
{
  return ((SmLinearFilletLaw_TYPE == t) ? TRUE : SmFilletLaw::IsKindOf( (t) ));
}

/*******************************************************************//**
PURPOSE: Dump a Linear Fillet Law.

NOTES:
***********************************************************************/
void SmLinearFilletLaw::Dump() const
{
  TCHAR sBuff[SM_TBLOCK_SIZE], sBuffForFile[SM_TBLOCK_SIZE];

  smos_sprintf(sBuff, _T("\nDump of SmLinearFilletLaw 0x%p"), this);
  smos_sprintf(sBuffForFile, _T("%s"),_T("\nDump of SmLinearFilletLaw"));
  smos_WriteBuffer(sBuff, sBuffForFile);

  smos_sprintf(sBuff, _T("\n  Start and End Radii %lf  %lf"), m_dStartValue, m_dEndValue);
  smos_sprintf(sBuffForFile, _T("\n  Start and End Radii %lf  %lf"), m_dStartValue, m_dEndValue);
  smos_WriteBuffer(sBuff, sBuffForFile);

  smos_sprintf(sBuff,_T("%s"), _T("\n  parent SmFilletLaw:"));
  smos_sprintf(sBuffForFile, _T("%s"), _T("\n  parent SmFilletLaw:"));
  smos_WriteBuffer(sBuff, sBuffForFile);

  SmFilletLaw::Dump();

  smos_sprintf(sBuff,_T("%s"), _T("\nEnd of SmLinearFilletLaw Dump."));
  smos_sprintf(sBuffForFile, _T("%s"), _T("\nEnd of SmLinearFilletLaw Dump."));
  smos_WriteBuffer(sBuff, sBuffForFile);

  return;

} // end SmLinearFilletLaw::Dump

/*******************************************************************//**
PURPOSE: Constructor for SmBSplineFilletLaw that creates a hermite
                 curve blending between two radii.

NOTES:
***********************************************************************/
SmBSplineFilletLaw::SmBSplineFilletLaw
(const SmContext & crContext,
        double dStartValue,
        double dEndValue,
        double dStartDeriv,
        double dEndDeriv,
        SmExtent1d * pOptEdgeMap)
        : SmFilletLaw(),
        m_pLawCurve(NULL),
        m_pExtendedLaw(NULL)
{
  m_pLawCurve = m_pExtendedLaw = NULL;
  if (pOptEdgeMap != NULL)
  {
      m_vEdgeMap = *pOptEdgeMap;
  }
  if (m_vEdgeMap.GetLength() < SM_EFF_ZERO) {
      m_vEdgeMap.SetMinMax(0, 1);
  }

  ULONG lDeg = 3;
  SmBSplineCurve *pLawCurve = NULL;

  static constexpr SmBoolean sbMakeSimpleHermite = TRUE;

  if (sbMakeSimpleHermite)
  {
      // Make a single-span cubic: just set the
      // control points according to the input data.
      SmExtent1d sDomain(0, 1);
      SmTArray< double > aKnots(2);
      aKnots.Add(sDomain.GetMin());
      aKnots.Add(sDomain.GetMax());
      SmTArray< ULONG > aMults(2);
      aMults.Add(4);
      aMults.Add(4);
      SmTArray< SmPoint3d > aCPts(4);
      SmPoint3d sPt(dStartValue, 0, 0);
      aCPts.Add(sPt);
      sPt.Set(dStartValue + dStartDeriv / 3, 1, 0);
      aCPts.Add(sPt);
      sPt.Set(dEndValue - dEndDeriv / 3, 2, 0);
      aCPts.Add(sPt);
      sPt.Set(dEndValue, 3, 0);
      aCPts.Add(sPt);
      // SmBSplineCurve * pNewBSplineCurve = NULL;
      SmBSplineCurve::CreateCanonical(crContext,
                        3, 3,
                        aCPts, SM_CF_UNSPECIFIED,
                        aMults, aKnots, SM_KT_UNSPECIFIED,
                        NULL, &sDomain,
                        pLawCurve);

#ifdef SM_DEBUG_CODE
      SmBoolean bDebugMe = FALSE;
      if (bDebugMe) {
          smgfx_SetLook(2, 3, 1, 0, 0); pLawCurve->Draw(); sm_GraphicsLoop();
          sm_GraphicsLoop();
      }
#endif // SM_DEBUG_CODE

  }
  else
  {
      // Old way: create an SmHermiteCurve, sample it at 101 points,
      // then interpolate those points to create the curve.
      double dSV = 1.0;
      double dEV = 2.0;

      if (dStartValue > dEndValue) {
                        double dTemp = dSV;
                        dSV = dEV;
                        dEV = dTemp;
      }
      SmPoint3d sStartPnt(0.0, dStartValue, 0.0);
      SmPoint3d sEndPnt(1.0, dEndValue, 0.0);
      SmVector3d sStartVec(dSV, dStartDeriv, 0.0);
      SmVector3d sEndvec(dEV, dEndDeriv, 0.0);

      SmHermiteCurve sHerm(sStartPnt, sStartVec, sEndPnt, sEndvec, 2);
      SmExtent1d sIvl = sHerm.GetNaturalInterval();

      SmTArray<SmPoint3d> sPoints;
      SmTArray<double> sParams;

      ULONG ii;
      for (ii = 0; ii <= 100; ii++) {
          double dT = sIvl.Evaluate( ii / 100.0 );
          SmPoint3d sPnt;
          sHerm.EvaluatePoint(dT, sPnt);
          double dRad = sPnt.y;
          sPoints.Add(SmPoint3d(dRad, sPnt.x, 0.0));
          sParams.Add(sPnt.x);
      }

      SE(SmBSplineCurve::InterpolatePoints(crContext, sPoints, &sParams, lDeg, NULL, NULL, FALSE,
                        SM_IT_CHORDLENGTH, pLawCurve));

#ifdef SM_DEBUG_CODE
      SmBoolean bDebugMe = FALSE;
      if (bDebugMe) {
          smgfx_SetLook(2, 3, 1, 0, 0); sHerm.Draw(); sm_GraphicsLoop();
          smgfx_SetLook(2, 3, 0, 0, 1); pLawCurve->Draw(); sm_GraphicsLoop();
          sm_GraphicsLoop();
      }
#endif // SM_DEBUG_CODE

  } // end if old or new method

  m_pLawCurve = pLawCurve;

} // end SmBSplineFilletLaw::SmBSplineFilletLaw

/*******************************************************************//**
PURPOSE: Constructor for SmBSplineFilletLaw that creates a blend
using Sine/Cosine to achieve elliptical curves on the off surface
and a tangent blend on each end.

NOTES:
***********************************************************************/
SmBSplineFilletLaw::SmBSplineFilletLaw
 (const SmContext & crContext,     // in : 
  double            dStartRadius,  // in : Start Radius of Blending Function
  double            dEndRadius,    // in : End Radius of Blending Function
  double            dOuterRadius,  // in : Outer Radius of Blending Function - usually
                                   //      the radius of the largest fillet of the 3
  double            dAngleSpan,    // in : Angle span of arc that has dOuterRadius
  double            dStartDeriv,   // NotUsed: in : Start Derivative if we don't start with a
                                   //      constant radius thing.
  double            dEndDeriv,     // NotUsed: in : 
  SmExtent1d      * pOptEdgeMap)   // in : 
: SmFilletLaw(),
  m_pLawCurve(NULL),
  m_pExtendedLaw(NULL)
{
  SM_REF2(dStartDeriv, dEndDeriv) ;
  if (pOptEdgeMap != NULL)
  {
      m_vEdgeMap = *pOptEdgeMap;
  }
  if (m_vEdgeMap.GetLength() < SM_EFF_ZERO) {
      m_vEdgeMap.SetMinMax(0, 1);
  }

  SmTArray<SmPoint3d> sPoints;
  SmTArray<double> sParams;

  if (dOuterRadius <= dEndRadius ||
      dOuterRadius <= dStartRadius) {
      SE(SM_ERR);
      dOuterRadius = dEndRadius + dStartRadius;
  }

  // Change from Hermite to Elliptical cosine/sine blending
  // In the case where the dTotalAngle is 90 degrees it will produce
  // an ellipse in the box case.
  SmExtent1d sIvl(0.0, 1.0);

  double dB = dOuterRadius - dEndRadius;
  double dA = dOuterRadius - dStartRadius;
  for (ULONG i = 0; i <= 100; i++) {
      double dT = sIvl.Evaluate(i / 100.0);
      double dTAng = dT * dAngleSpan;
      double dTan = smos_Tangent(dTAng);
      double dVal = dTan * dA / dB;
      double dTAng2 = smos_ArcTangent(dVal);
      double x = dA * smos_Cosine(dTAng2);
      double y = dB * smos_Sine(dTAng2);
      double dRad = dOuterRadius - smos_Sqrt(x*x + y*y);
      sPoints.Add(SmPoint3d(dRad, dTAng, 0.0));
      sParams.Add(dT);
  }

  ULONG lDeg = 2;
  SmBSplineCurve *pLawCurve = NULL;
  SE(SmBSplineCurve::InterpolatePoints(crContext, sPoints, &sParams, lDeg, NULL, NULL, FALSE,
                SM_IT_CHORDLENGTH, pLawCurve));

  m_pLawCurve = pLawCurve;

#ifdef SM_DEBUG_CODE
  SmBoolean bDebugMe = FALSE;
  if (bDebugMe) {
      SmVector3d sPV[2];
      m_pLawCurve->Evaluate(0.0, 1, TRUE, sPV);
      smgfx_Erase();
      smgfx_SetColor(1, 0, 0);
      pLawCurve->Draw();
      sm_GraphicsLoop();
  }
#endif // SM_DEBUG_CODE

} // end SmBSplineFilletLaw::SmBSplineFilletLaw

/*******************************************************************//**
PURPOSE: Constructor for the B-Spline based fillet law.  Note that
                the X values of the curve correspond to the fillet radius.

NOTES:
***********************************************************************/
SmBSplineFilletLaw::SmBSplineFilletLaw( SmBSplineCurve * pCurve,           // in : fillet-radius function
                                        const SmExtent1d & crEdgeMap)      // in : Subset of the curve which maps to the edge.
                                                                           //      That way we have control of what happens outside of the interval of the edge.
    : SmFilletLaw(),
      m_pLawCurve(pCurve),
      m_pExtendedLaw(NULL)
{
  m_vEdgeMap = crEdgeMap;
  if (m_vEdgeMap.GetLength() < SM_EFF_ZERO) {
      m_vEdgeMap.SetMinMax(0, 1);
  }

} // end SmBSplineFilletLaw::SmBSplineFilletLaw constructor

/*******************************************************************//**
PURPOSE: Alternate constructor for BSplineFilletLaw that uses an array of
   parameters that map to an array of fillet-radii.
   It will internally create a law curve by interpolation.

NOTES:
   SmLawBlendingType:
   1: This uses standard cubic interpolation with chord-length parameterization,
      and some sort of constraints on the slopes at the radius fixes.
   2: This creates a well-behaved function that is specialized to be a
      variable-radius fillet radius function.  It will not overshoot radius
      values (relative minima and maxima), and hence will not go negative.
      It returns a piecewise cubic Bezier function, in B-Spline format.
      It will return zero slopes at the ends of the function.
   3: Same as option 2 except that the end conditions are 'unclamped':
      zero curvatures at the ends of the function.
   4: Same as option 2 with both zero slopes and zero curvatures at the ends.
      Otherwise:
      If two radius fixes are given, create a linear function,
      Else if three are given, create a quadratic function,
      Else use standard cubic interpolation with chord-length parameterization.
***********************************************************************/
SmBSplineFilletLaw::SmBSplineFilletLaw (const SmContext & crContext,            // in : context for new object construction
                                        const SmTArray<double> & crParameters,  // in : given parameter values
                                        const SmTArray<double> & crRadii,       // in : corresponding fillet-radii values
                                              ULONG lLawBlendingType,           // in : see Usage Notes.
                                        const SmExtent1d & crEdgeMap,           // in : Subset of the curve which maps to the
                                                                                //      edge.  That way we have control of what
                                                                                //      happens outside of the interval of the edge.
                                                                                //      If passed in Null, will use the domain of the
                                                                                //      B-Spline law curve that we create here.
                                              SmBoolean & rbError)              // out: TRUE = failed to build pLawCurve
                                                                                //      FALSE= OK
    : SmFilletLaw(),
      m_pLawCurve(NULL),
      m_pExtendedLaw(NULL)
{
  rbError = FALSE;

  // save map interval
  m_vEdgeMap = crEdgeMap;
  if (m_vEdgeMap.GetLength() < SM_EFF_ZERO) {
      m_vEdgeMap.SetMinMax(0, 1);
  }

  // Locals, so that we can add to them.
  SmTArray< double > sParams = crParameters;
  SmTArray< double > sRads = crRadii;

  if (lLawBlendingType == 4)
  {
      sRads.InsertAt(1, 0.8*crRadii[0] + 0.2*crRadii[1]);
      ULONG lIdx = crRadii.GetSize();
      sRads.InsertAt(lIdx, 0.8*crRadii[lIdx - 1] + 0.2*crRadii[lIdx - 2]);

      lIdx = crParameters.GetSize() - 1;
      sParams.InsertAt(lIdx, 0.7*crParameters[lIdx] + 0.3*crParameters[lIdx - 1]);
      sParams.InsertAt(1, 0.7*crParameters[0] + 0.3*crParameters[1]);
  }

  // Let's first see about how many points we are going to need on our curve
  SmTArray<SmPoint3d> sPoints;
  SmTArray<SmVector3d> sVectors;

  // create 3d point and vector for each given radius value
  for (ULONG i = 0; i<sRads.GetSize(); i++)
  {
      // Note: put something monotonic into the y-coordinate.
      // This will prevent the interpolating routines from thinking that
      // they are creating curves that double back on themselves,
      // or have coincident interpolating points.
      double dRad = sRads[i];
      sPoints.Add(SmPoint3d(dRad, i, 0));
      sVectors.Add(SmVector3d(1, i, 0));  // (derivative of +1 at each point?)
  }

  SmBSplineCurve *pLawCurve = NULL;
  ULONG lDeg = 3;

  // branch on lLawBlendingType
  if (lLawBlendingType == 1)
  {
      // interpolate point and tangent directions with a
      // chord-length parameterized, degree 3 BSpline
      SE(SmBSplineCurve::CreateInterpolatingCurve(crContext,
                        SM_CP_CHORDLENGTH, 3, 3, sPoints, sVectors, NULL, TRUE, pLawCurve));
  }

  else if (lLawBlendingType == 2 || lLawBlendingType == 3 || lLawBlendingType == 4)
  {
      // Each radius fix will be a breakpoint of a piecewise cubic Hermite curve.
      ULONG lNumFixes = sParams.GetSize();

      ULONG lNumCPts = 3 * lNumFixes - 2;
      SmTArray< SmPoint3d > sCtrlPts(lNumCPts, NULL, lNumCPts);
      // Knots: since they're passed to the constructor as [distinct knots]
      // and [multiplicities], then the sizes of those arrays is just lNumFixes.
      // Also, we can just use sParams as the parameter array for the constructor.

      // Another consideration: The x-coordinate is all that's used in this
      // curve, but if we put 0's into y and z, then if any two consectuve
      // radius values are equal, an illegal 'curve' will result.
      // So put something into the y-coordinates to prevent that.
      // We'll just put the control-point index, giving a monotonic curve.

      SmTArray< ULONG > sMultiplicities(lNumFixes, NULL, lNumFixes);

      // Set 1st and last control points and knot multiplicities.
      ULONG lPtIdx = 0;
      ULONG lFixIdx = 0;
      sCtrlPts[lPtIdx].Set(sRads[lFixIdx], lPtIdx, 0);
      lPtIdx = lNumCPts - 1;
      lFixIdx = lNumFixes - 1;
      sCtrlPts[lPtIdx].Set(sRads.GetLast(), lPtIdx, 0);  // [B551]

      sMultiplicities[0] = sMultiplicities[lNumFixes - 1] = 4;

      // For type 4 (zero slope and zero curvature at ends), add an extra segment at start and end.
      if (lLawBlendingType == 4)
      {
          lPtIdx = 0;
          lFixIdx = 0;
          sCtrlPts[lPtIdx + 1].Set(sRads[lFixIdx], 0.2, 0);
          sCtrlPts[lPtIdx + 2].Set(sRads[lFixIdx], 0.4, 0);

          sCtrlPts[lPtIdx + 3].Set(0.8*sRads[lFixIdx] + 0.2*sRads[lFixIdx + 1], 0.6, 0);
          sCtrlPts[lPtIdx + 4].Set(0.4*sRads[lFixIdx] + 0.6*sRads[lFixIdx + 1], 0.8, 0);

          lPtIdx = lNumCPts - 1;
          lFixIdx = lNumFixes - 1;
          double dPtIdx = lPtIdx;
          sCtrlPts[lPtIdx - 1].Set(sRads[lFixIdx], dPtIdx - 0.2, 0);
          sCtrlPts[lPtIdx - 2].Set(sRads[lFixIdx], dPtIdx - 0.4, 0);

          sCtrlPts[lPtIdx - 3].Set(0.8*sRads[lFixIdx] + 0.2*sRads[lFixIdx - 1], dPtIdx - 0.6, 0);
          sCtrlPts[lPtIdx - 4].Set(0.4*sRads[lFixIdx] + 0.6*sRads[lFixIdx - 1], dPtIdx - 0.8, 0);

          sMultiplicities[1] = sMultiplicities[lNumFixes - 2] = 3;
      }

      // For each interior fix, set the control point and knot multiplicity,
      // and the previous and following control points.
      ULONG ii;
      ULONG lStart = 1, lStop = lNumFixes - 1;
      if (lLawBlendingType == 4)
      {
          lStart++;
          lStop--;
      }

      for (ii = lStart; ii < lStop; ii++)
      {
          // Control point:
          ULONG lPtIndex = 3 * ii;
          double dThisRad = sRads[ii];
          double dThisParam = sParams[ii];
          sCtrlPts[lPtIndex].Set(dThisRad, lPtIndex, 0);

          sMultiplicities[ii] = 3;

          // x-values of prev and next control points are 1/3 of the way
          // before and after this knot value.
          double dPrevParam = (2 * sParams[ii] + sParams[ii - 1]) / 3.0;
          double dNextParam = (2 * sParams[ii] + sParams[ii + 1]) / 3.0;

          // Interior points: Here's where the algorithm comes in.
          // If this point is a relative min or max, then the slope is zero.
          double dPrevRad = sRads[ii - 1];
          double dNextRad = sRads[ii + 1];

          double dDel0 = dThisRad - dPrevRad;
          double dDel1 = dNextRad - dThisRad;
          if (dDel0 * dDel1 < SM_EFF_ZERO_SQRT)
          {
              // Relative extremum: zero slope.
              sCtrlPts[lPtIndex - 1].Set(dThisRad, lPtIndex - 1, 0);
              sCtrlPts[lPtIndex + 1].Set(dThisRad, lPtIndex + 1, 0);
          }
          else
          {
              // Not an extremum, use an F-Mill type of thing: average slope
              // between prev and next points.
              double dSlope = (dNextRad - dPrevRad) / (dNextParam - dPrevParam);

              double dDeltaT = (dThisParam - dPrevParam) / 3.0;
              sCtrlPts[lPtIndex - 1].Set(dThisRad - dDeltaT*dSlope, lPtIndex - 1, 0);

              dDeltaT = (dNextParam - dThisParam) / 3.0;
              sCtrlPts[lPtIndex + 1].Set(dThisRad + dDeltaT*dSlope, lPtIndex + 1, 0);
          }
      } // end loop on each radius fix.

      // Now we can set the penultimate control points, which will set
      // the behavior of the function at the ends.
      //   If lLawBlendingType == 2, we'll use zero slope at the ends.
      //   If lLawBlendingType == 3, use an unclamped end condition:
      //      zero curvature of the function.
      //   If lLawBlendingType == 4, we've already set zero slopes and curvatures.

      if (lLawBlendingType == 2)
      {
          // Zero slope: equal function values.
          sCtrlPts[1] = sCtrlPts[0];
          sCtrlPts[1].y = 1.0;
          sCtrlPts[lNumCPts - 2] = sCtrlPts[lNumCPts - 1];
          sCtrlPts[lNumCPts - 2].y = lNumCPts - 2;
      }
      else if (lLawBlendingType == 3)
      {
          // Zero curvature: set up three collinear control points.
          lPtIdx = 1;
          sCtrlPts[lPtIdx] = (sCtrlPts[lPtIdx - 1] + sCtrlPts[lPtIdx + 1]) / 2;
          lPtIdx = lNumCPts - 2;
          sCtrlPts[lPtIdx] = (sCtrlPts[lPtIdx - 1] + sCtrlPts[lPtIdx + 1]) / 2;
      }

      SE(SmBSplineCurve::CreateCanonical(crContext, 3, 3,
                        sCtrlPts,
                        SM_CF_UNSPECIFIED,
                        sMultiplicities,
                        sParams,
                        SM_KT_PIECEWISE_BEZIER_KNOTS,
                        NULL, NULL,
                        pLawCurve)
      );

  } // end if lLawBlendingType == 2 or 3, special var-rad-fillet radius function.

  else
  {
      // lLawBlendingType is unspecified.  Use linear or quadratic if
      // given 2 or 3 points respectively, else standard cubic interpolation.

      if (sRads.GetSize() == 2)
      {
          SE(SmBSplineCurve::CreateLineSegment(crContext, 3, sPoints[0], sPoints[1], pLawCurve));
      }
      else
      {
          if (sRads.GetSize() == 3) lDeg = 2;
          SE(SmBSplineCurve::InterpolatePoints(crContext, sPoints, &sParams, lDeg, NULL, NULL, FALSE,
                                SM_IT_CHORDLENGTH, pLawCurve));
      }
  } // end if-else on lLawblendingType


  m_pLawCurve = pLawCurve;

  rbError = (pLawCurve == NULL);

  return;

} // end SmBSplineFilletLaw::SmBSplineFilletLaw constructor

/*******************************************************************//**
PURPOSE: Copy Constructor

NOTES:
***********************************************************************/
SmBSplineFilletLaw::SmBSplineFilletLaw
 (const SmBSplineFilletLaw & crOther)
: SmFilletLaw(crOther)
{ 
  *this = crOther; 
  
} // end SmBSplineFilletLaw copy constructor

/*******************************************************************//**
PURPOSE: Destructor for SmBSplineFilletLaw

NOTES:
***********************************************************************/
SmBSplineFilletLaw::~SmBSplineFilletLaw()
{
  SM_ASSERT(m_pLawCurve != NULL); delete m_pLawCurve; m_pLawCurve = NULL;
  if (m_pExtendedLaw) { delete m_pExtendedLaw; m_pExtendedLaw = NULL; }

} // end SmBSplineFilletLaw::~SmBSplineFilletLaw destructor

/*******************************************************************//**
PURPOSE: Assignment operator

NOTES:
***********************************************************************/
SmBSplineFilletLaw & SmBSplineFilletLaw::operator=(const SmBSplineFilletLaw & crOther)
{
  if (&crOther == this) { return *this; }
  SmFilletLaw::operator=(crOther);  // Do parent stuff.

  m_pLawCurve = NULL;
  SmCurve *pCurve = NULL;
  if (crOther.m_pLawCurve != NULL)
  {
      crOther.m_pLawCurve->Copy(*(this->GetContext()), pCurve);
  }
  m_pLawCurve = SM_CAST_PTR(SmBSplineCurve, pCurve);

  m_pExtendedLaw = NULL;  // Gets created as needed.

  return *this;

} // end SmBSplineFilletLaw::operator=

/*******************************************************************//**
PURPOSE: Copy method.

NOTES: (Only way to make a virtual copy operator.)
***********************************************************************/
SmStatus SmBSplineFilletLaw::Copy
 (const SmContext & crContext,
  SmFilletLaw    *& rpNewFilletLaw) 
 const
{
  SmBSplineFilletLaw *pNewLaw = new(crContext) SmBSplineFilletLaw(*this);
  rpNewFilletLaw = pNewLaw;

  return SM_SUCCESS;

}  // end SmBSplineFilletLaw::Copy

/*******************************************************************//**
PURPOSE: Evaluate a B-Spline Fillet Law.

NOTES:  If the given parameter is outside of the Law's domain,
   the Law curve will be extended, into m_bExtendedLaw.
***********************************************************************/
SmStatus SmBSplineFilletLaw::Evaluate 
 (double             dParameter,          // in : target parameter - range:[crCurveInterval.Min,Max]
  const SmExtent1d & crCurveInterval,     // in : interval defining range of fillet edge
  SmBoolean          bReverseOrientation, // in : TRUE = parameters run from interval end to interval start
  double             dValues[3])          // out: dValues[0] = fillet-radius at dParameter value
 const                                    //      dValues[1] = 1st derivative: d(fillet-radius)/d(Parameter)
                                          //      dValues[2] = 2nd derivative: d2(fillet-radius)/d(Parameter)**2
{
  if( crCurveInterval.GetLength() == 0.0 )
    SER( SM_ERR );

  // select function to evaluate
  SmCurve *pLawCurve = (m_pExtendedLaw == NULL)
                ? m_pLawCurve
                : m_pExtendedLaw;

  // Compute the scale difference between the parameters of the map and the curve.
  SmExtent1d sIvl = pLawCurve->GetNaturalInterval();
  double dCrvIvlSize = crCurveInterval.GetLength();
  double dMapIvlSize = m_vEdgeMap.GetLength();
  double dParamScale = dCrvIvlSize / dMapIvlSize;

  // Map parameter to unitized range accounting for bReverseOrientation
  double dMin = crCurveInterval.GetMin();
  double dMax = crCurveInterval.GetMax();

  double dT = (bReverseOrientation)
                ? (dMax - dParameter) / (dMax - dMin)
                : (dParameter - dMin) / (dMax - dMin);

  // Map parameter back to interval of the curve.
  double dLawT = m_vEdgeMap.Evaluate(dT);

  // when parameter is beyond current range
  if (!sIvl.ContainsValue(dLawT))
  {
      // select param space tolerance
      double dScaledTol = SM_EFF_ZERO * (1.0 + smos_Fabs(sIvl.GetMin()) + smos_Fabs(sIvl.GetMax()));

      // Try to snap to begin point.
      if (    dLawT              < sIvl.GetMin()
           && dLawT + dScaledTol > sIvl.GetMin())
      {
          dLawT = sIvl.GetMin();
      }

      // Try to snap to end point.
      if (    dLawT              > sIvl.GetMax()
           && dLawT - dScaledTol < sIvl.GetMax())
      {
          dLawT = sIvl.GetMax();
      }

      // when parameter is still beyond current range
      if (!sIvl.ContainsValue(dLawT))
      {
          // create an extension of the curve to the requested param point
          if (pLawCurve->IsKindOf(SmBSplineCurve_TYPE))
          {
              double          dExtParam;
              SmBSplineCurve *pBSC = SM_CAST_PTR(SmBSplineCurve, pLawCurve); NER(pBSC);
              SmBSplineCurve *pExtendedLaw;

              // But extend farther than requested, so that we won't have to do this too often.
              // This doubles the extension distance:
              dExtParam = (dLawT<sIvl.GetMin()) ? dLawT - (sIvl.GetMin() - dLawT)
                                            : dLawT + (dLawT - sIvl.GetMax());
              // But never do a really tiny extension: it might get snapped to the end,
              // and therefor not extended at all.  [Fillet 308]
              double dLen = sIvl.GetLength();
              if ( dExtParam < sIvl.GetMin() )
              {
                  if (dExtParam > sIvl.GetMin() - dLen / 10)
                  {   dExtParam = sIvl.GetMin() - dLen / 10; }
              }
              else
              {
                  if (dExtParam < sIvl.GetMax() + dLen / 10)
                  {   dExtParam = sIvl.GetMax() + dLen / 10; }
              }

              static constexpr SmBoolean bPreciseExtension = TRUE;          // Otherwise it adds another length of sIvl.
              static constexpr SmContinuityType eExtCont = SM_CT_CINFINITY; // C-infinity works better than G1_G2.  [B551]

              SER(pBSC->CreateExtendedCurve(*pLawCurve->GetContext(), dExtParam, eExtCont,
                                             pExtendedLaw, bPreciseExtension));
              if (m_pExtendedLaw) { delete m_pExtendedLaw; ((SmBSplineFilletLaw*)this)->m_pExtendedLaw = NULL; }

              // save and use the extended curve
              ((SmBSplineFilletLaw*)this)->m_pExtendedLaw = pExtendedLaw;
              pLawCurve = pExtendedLaw;
          }
          else
            { SER(SM_ERR); } // Unable to extend non-BSpline Laws

      } // end need to extend fillet-radius function check after snapping to endPoints
  } // end need to extend fillet-radius function check

  SmVector3d sPV[3];
  SER(pLawCurve->Evaluate(dLawT, 2, TRUE, sPV));

  dValues[0] = sPV[0].x;
  dValues[1] = sPV[1].x / dParamScale;
  if (bReverseOrientation)
    { dValues[1] = -dValues[1]; }
  dValues[2] = sPV[2].x / (dParamScale * dParamScale);

  return SM_SUCCESS;

} // end SmBSplineFilletLaw::Evaluate

/*******************************************************************//**
PURPOSE:
NOTES:
***********************************************************************/
SmBoolean SmBSplineFilletLaw::IsKindOf( SM_TYPE t ) const
{
  return ((SmBSplineFilletLaw_TYPE == t) ? TRUE : SmFilletLaw::IsKindOf( (t) ));
}

/*******************************************************************//**
PURPOSE: Dump a BSpline Fillet Law.

NOTES:
***********************************************************************/
void SmBSplineFilletLaw::Dump() const
{
  TCHAR sBuff[SM_TBLOCK_SIZE], sBuffForFile[SM_TBLOCK_SIZE];

  smos_sprintf(sBuff, _T("\nDump of SmBSplineFilletLaw 0x%p"), this);
  smos_sprintf(sBuffForFile, _T("%s"),_T("\nDump of SmBSplineFilletLaw"));
  smos_WriteBuffer(sBuff, sBuffForFile);

  smos_sprintf(sBuff, _T("\n    m_pLawCurve: 0x%p"), m_pLawCurve);
  smos_sprintf(sBuffForFile, _T("\n    m_pLawCurve: %s"), m_pLawCurve ? _T("notNULL") : _T("NULL"));
  smos_WriteBuffer(sBuff, sBuffForFile);

  smos_sprintf(sBuff, _T("\n    m_pExtendedLaw: 0x%p"), m_pExtendedLaw);
  smos_sprintf(sBuffForFile, _T("\n    m_pExtendedLaw: %s"), m_pExtendedLaw ? _T("notNULL") : _T("NULL"));
  smos_WriteBuffer(sBuff, sBuffForFile);

  if (FALSE) {
      if (m_pLawCurve) { m_pLawCurve->Dump(); }
      if (m_pExtendedLaw) { m_pExtendedLaw->Dump(); }
  }

  smos_sprintf(sBuff, _T("%s"),_T("\n  parent SmFilletLaw:"));
  smos_sprintf(sBuffForFile, _T("%s"),_T("\n  parent SmFilletLaw:"));
  smos_WriteBuffer(sBuff, sBuffForFile);

  SmFilletLaw::Dump();

  smos_sprintf(sBuff, _T("%s"),_T("\nEnd of SmLinearFilletLaw Dump."));
  smos_sprintf(sBuffForFile, _T("%s"),_T("\nEnd of SmLinearFilletLaw Dump."));
  smos_WriteBuffer(sBuff, sBuffForFile);

  return;

} // end SmBSplineFilletLaw::Dump
