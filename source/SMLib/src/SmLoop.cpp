// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************//**
* FILE NAME --- SmLoop.cpp
* PURPOSE: Source file for SmLoop class methods.
**********************************************************************/

#include "StdAfx.h"

#include <SmLoop.h>

#ifndef __SMBREP_H__
#include <SmBrep.h>
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

#include <SmTrimmingTools.h>
#include <SmGeomUtility.h>
#include <SmAssertArray.h>


#ifdef SM_DEBUG_CODE
  #include <SmPlane.h>
#endif // SM_DEBUG_CODE

//    // For debugging only - a set of pointers that can be assigned and inspected through the watch window
//    #ifdef SM_DEBUG_CODE
//
//    SmLoop * dbgLoop1 = NULL ;
//    SmLoop * dbgLoop2 = NULL ;
//
//    #endif // SM_DEBUG_CODE

/*******************************************************************//**
PURPOSE: Flip the orientation of a loop by swaping the edgeuses or
    vertexuse of its loopuses.

NOTES: 
***********************************************************************/
SmStatus SmLoop::FlipLoopOrientation()
{
  SmLoopuse * pLoopuse1, * pLoopuse2 ;
  GetLoopuses(pLoopuse1,pLoopuse2) ; NER(pLoopuse1) ; NER(pLoopuse2) ;
  SmFaceuse * pFaceuse = pLoopuse1->GetFaceuse() ;
  SmFace * pFace = pFaceuse ? pFaceuse->GetFace() : NULL ;
  if(pFace) { pFace->Notify(SM_NO_PRE_EDIT, pFace, SM_NO_GET_BREP(pFace), NULL) ; }

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;

  if(bDebugMe)
    {
      SmBrep    * pBrep    = GetBrep() ;
      SmSurface * pSurface = pFace ? pFace->GetSurface() : NULL ;
      
      SM_DUMP_AND_ASSERT_VALID(pBrep) ; 

      smgfx_Erase() ;
      smgfx_SetLook( 1, 2, 0, 0, 1 ); if(pBrep) { pBrep->Draw( TRUE ); sm_GraphicsLoop(); }
      smgfx_SetLook(3,4, 1,0,0) ; this->Draw(3,FALSE,NULL,FALSE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(3,4, 1,0,0) ; if(pLoopuse1) { pLoopuse1->Draw() ; sm_GraphicsLoop() ;   }
      smgfx_SetLook(3,4, 1,0,0) ; if(pLoopuse2) { pLoopuse2->Draw() ; sm_GraphicsLoop() ;   }
      smgfx_SetLook(1,2, 0,1,1) ; if(pSurface)  { pSurface->DrawUV() ; sm_GraphicsLoop() ;  }
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

  if (pLoopuse1->IsVertexLoopuse()) 
    {
      // get Vertexuses for both mated Loopuses
      SmVertexuse * pVertexuse1 = (SmVertexuse*)pLoopuse1->m_pEUorVU; NER(pVertexuse1);
      SmVertexuse * pVertexuse2 = (SmVertexuse*)pLoopuse2->m_pEUorVU; NER(pVertexuse2);

      // swap which vertexuse goes to which Loopuse - remember to set Vertexuse back pointers
      pLoopuse1->m_pEUorVU      = pVertexuse2;
      pLoopuse2->m_pEUorVU      = pVertexuse1;
      pVertexuse2->m_pSorLUorEU = pLoopuse1;
      pVertexuse1->m_pSorLUorEU = pLoopuse2;
    }
  else // IsEdgeuseLoopuse() branch
    {
      // get first Edgeuses for both mated Loopuses
      SmEdgeuse * pEdgeuse1 = (SmEdgeuse*)pLoopuse1->m_pEUorVU; NER(pEdgeuse1);
      SmEdgeuse * pEdgeuse2 = (SmEdgeuse*)pLoopuse2->m_pEUorVU; NER(pEdgeuse2);

      SmEdge * pEdge1 = pEdgeuse1->GetEdge() ; 
      SmEdge * pEdge2 = pEdgeuse2->GetEdge() ; 
      SM_REF2(pEdge1, pEdge2) ;
      SM_ASSERT_MSG(pEdge1 == pEdge2, _T("SmLoop::FlipLoopOrientation: Assumption that Loop->LoopuseMate->PrimaryEdgeuses share common Edge is FALSE.")) ;

      // swap which Edgeuse goes to which Loopuse and rebuild the Edgeuse->CW/CCW LinkedLists and back pointers
      SER(pLoopuse2->CollectEdgeuses(pEdgeuse1));
      SER(pLoopuse1->CollectEdgeuses(pEdgeuse2));

      // // gwc: this needs to be rethought:  swap Edge->Edgeuse order to preserve radial ordering
      // pEdge1->SwapEdgeuseOrder(*pEdgeuse1, *pEdgeuse2) ;

    } // end IsEdgeuseLoopuse() branch

#ifdef SM_DEBUG_CODE
  if(bDebugMe)
    {
      SmBrep    * pBrep    = GetBrep() ;
      SmSurface * pSurface = pFace ? pFace->GetSurface() : NULL ;
      
      SM_DUMP_AND_ASSERT_VALID(pBrep) ; 

      smgfx_Erase() ;
      smgfx_SetLook( 1, 2, 0, 0, 1 ); if(pBrep) { pBrep->Draw( TRUE ); sm_GraphicsLoop(); }
      smgfx_SetLook(3,4, 1,0,0) ; this->Draw(3,FALSE,NULL,FALSE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(3,4, 1,0,0) ; if(pLoopuse1) { pLoopuse1->Draw() ; sm_GraphicsLoop() ;  }
      smgfx_SetLook(3,4, 1,0,0) ; if(pLoopuse2) { pLoopuse2->Draw() ; sm_GraphicsLoop() ;  }
      smgfx_SetLook(1,2, 0,1,1) ; if(pSurface)  { pSurface->DrawUV() ; sm_GraphicsLoop() ; }
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

  if(pFace) { pFace->Notify(SM_NO_POST_EDIT, pFace, SM_NO_GET_BREP(pFace), NULL) ; }
  return SM_SUCCESS;

} // end SmLoop::FlipLoopOrientation

/*************************************************************
PURPOSE:  return Loop's largest EdgeEnd/EdgeEnd LoopGap
          or NULL when Loop has no Edgeuses
NOTES: 
**************************************************************/
const SmGap * SmLoop::GetMaxSameDimGap3d
 (SmGapArray * pOptGapArray, // out: optional List of all Vertex/Edge and Edge/Face Gap3ds, NULL to ignore, default:[NULL]
  SmTol3d    * pOptTol3d)    // in : NotNULL        = only load Gaps larger than OptTol3d into OptGapArray.
                             //      default:[NULL] = load all Gaps into OptGapArray
 const
{
  // init output
  if(pOptGapArray) { pOptGapArray->ReSet() ; }

  // locals
  SM_PTR_ARRAY(sEdgeuses, SmEdgeuse, 16) ;
  GetEdgeuses(sEdgeuses) ;

  // no work - no or just one Edgeuse in Loop
  if(sEdgeuses.GetSize() <= 1)
    { return(NULL) ; }

  // iter init
  SmGap * pMaxGap = NULL ;

  // for every
  for(ULONG ii=0;ii<sEdgeuses.GetSize();ii++)
    {
      // Start EdgeEnd/EdgeEnd Gap
      SmGap * pEdgeEdgeGap = sEdgeuses[ii]->GetEdgeCCWEdgeGap() ;

      if(pEdgeEdgeGap)
        {
          // build Gap List
          if(pOptGapArray && (pOptTol3d == NULL || pEdgeEdgeGap->GetLength() > pOptTol3d->val)) 
            { pOptGapArray->Add(pEdgeEdgeGap) ; }

          // set output
          if(pMaxGap == NULL || *pMaxGap < *pEdgeEdgeGap) { pMaxGap = pEdgeEdgeGap ; }
        } // end pEdgeEdgeGap existence check
    } // end iter every vertexuse

  // all done
  return(pMaxGap) ;

} // end SmLoop::GetMaxSameDimGap3d

/*******************************************************************//**
PURPOSE: Determine if this loop is the outer loop in a face.  The
    first Loop in the Loop->Faceuse->LoopuseList.

NOTES: 
***********************************************************************/
SmBoolean SmLoop::IsOuterLoop() const
{
  SmLoopuse *pLU = m_pLU; SM_ASSERT(pLU != NULL);
  SmFaceuse *pFU = pLU->GetFaceuse();
  SmTArray<SmLoopuse*> sLoopuses;
  pFU->GetLoopuses(sLoopuses);
  if (sLoopuses[0] == pLU) return TRUE;
  return FALSE;

} // end SmLoop::IsOuterLoop

/*******************************************************************//**
PURPOSE: return TRUE when all edges in loop are lamina edges

NOTES: 
***********************************************************************/
SmBoolean SmLoop::IsLamina() const
{
  ULONG ii ;
  SmTArray<SmEdge*> sEdges;
  GetEdges(sEdges) ;

  for(ii=0;ii<sEdges.GetSize();ii++)
    {
      if( ! sEdges[ii]->IsLamina())
        { return FALSE ; }
    }

  // arrive here for lamina loops
  return(TRUE) ;

} // end SmLoop::IsLamina

/*******************************************************************//**
PURPOSE: return TRUE when all edges in loop are manifold edges

NOTES: 
***********************************************************************/
SmBoolean SmLoop::IsManifold() const
{
  ULONG ii ;
  SmTArray<SmEdge*> sEdges;
  GetEdges(sEdges) ;

  for(ii=0;ii<sEdges.GetSize();ii++)
    {
      if( ! sEdges[ii]->IsManifold())
        { return FALSE ; }
    }

  // arrive here for manifold loops
  return(TRUE) ;

} // end SmLoop::IsManifold

/*******************************************************************//**
PURPOSE: TRUE when all EdgeEnd/EdgeEnd Gap3ds are withthin XSectTol3d in 3d

NOTES: returns TRUE for VertexLoopuse
***********************************************************************/
SmBoolean SmLoop::IsClosed3d       // rtn: TRUE when all EdgeEnd/EdgeEnd Gap3ds are within their associated XSectTol3d limit values
 (SmBoolean       bCheckAllGaps,   // in : TRUE = Check all gaps after finding the first open EdgeuseEnd-EdgeuseEnd gap to get true MaxGap values
                                   //      FALSE= quit after finding the 1st open EdgeuseEnd-EdgeuseEnd gap to save time.
  SmEdgeuse    ** ppOptEdgeuse,    // out: When any gaps are out-of-tol, the 1st edgeuse of the gap that exceeds its XSectTol3d value the most,
                                   //      When all gaps are in-tol, the largest gap between Edgeuse ends.
                                   //      CCW Edgeuse is the other end of gap. NULL to ignore, Default:[NULL]
  double        * pdOptMaxGap,     // out: Dist for gap being returned by ppOptEdgeuse
                                   //      NULL to ignore, Default:[NULL]
  SmXSectTol3d  * psOptXSectTol3d) // out: XSectTol3d value for gap being returned by ppOptEdgeuse. 
 const                             //      NULL to ignore, Default:[NULL]
{
  // no work - no Loopuse, report as ok
  if(GetLoopuse() == NULL)
    { return TRUE ; }

  // pass the call along
  return( GetLoopuse()->IsClosed3d(bCheckAllGaps, ppOptEdgeuse, pdOptMaxGap, psOptXSectTol3d ) ) ;

} // end SmLoop::IsClosed3d

/*******************************************************************//**
PURPOSE: TRUE when Edgeuse LinkedList of CCWEdgeuse ptrs form a closed LinkedList

NOTES: returns TRUE for VertexLoopuse
***********************************************************************/
SmBoolean SmLoop::IsClosedPtrs() const
{
  // no work - no Loopuse, report as ok
  if(GetLoopuse() == NULL)
    { return TRUE ; }

  // pass the call along
  return( GetLoopuse()->IsClosedPtrs() ) ;

} // end SmLoop::IsClosedPtrs

/*******************************************************************//**
PURPOSE: Return True when SmLoop is connected to given Target Topology object

NOTES: 1. the input Target cpConnectTgt may be NULL, or of type,
             SmVertex,
             SmVertexuse,
             SmEdge,
             SmEdgeuse,
             SmLoop,
             SmFace,
             SmShell,
             SmRegion
        2. returns TRUE for NULL and for any other Unsupported Topology TYPE
***********************************************************************/
SmBoolean SmLoop::IsConnectedTo
  ( const SmTopology *cpConnectTgt )    // in : target topology
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
      case SmFace_TYPE     : { return( GetFace() == (SmFace *)cpConnectTgt ) ; 
                             } break ;
      case SmFaceuse_TYPE  : { return( GetFace() == ((SmFaceuse *)cpConnectTgt)->GetFace() ) ; 
                             } break ;
      case SmLoop_TYPE     : { return( this == ((SmLoop *)cpConnectTgt) ) ;
                             } break ;
      case SmLoopuse_TYPE  : { return( this == ((SmLoopuse *)cpConnectTgt)->GetLoop() ) ;
                             } break ;
      case SmShell_TYPE    : { SmFaceuse *pFaceuse1, * pFaceuse2 ; 
                               GetFace()->GetFaceuses(pFaceuse1, pFaceuse2) ; 
                               return(   pFaceuse1->GetShell() == (SmShell *)cpConnectTgt
                                      || pFaceuse2->GetShell() == (SmShell *)cpConnectTgt) ;
                             } break ;
      case SmRegion_TYPE   : { SmFaceuse *pFaceuse1, * pFaceuse2 ; 
                               GetFace()->GetFaceuses(pFaceuse1, pFaceuse2) ; 
                               return(   pFaceuse1->GetShell()->GetRegion() == (SmRegion *)cpConnectTgt
                                      || pFaceuse2->GetShell()->GetRegion() == (SmRegion *)cpConnectTgt) ;
                             } break ;
      default: break ;

    } // end switch on type

  // arrive here when cpConnectTgt is an unsupported type - return TRUE
  return(TRUE) ;

} // end SmLoop::IsConnectedTo

/*******************************************************************//**
PURPOSE: return TRUE when when all edges are embedded in owner face 
         loop within Face->ZoneTol3d in 3d

NOTES: returns TRUE for VertexLoopuse
***********************************************************************/
SmBoolean SmLoop::IsEmbedded
 (SmBoolean bFaceHasSeam)    // in : TRUE = Face known to have SeamEdge - do Expensive IsSeam() check
                             //      FALSE= Face known to have no SeamEdge - Skip IsSeam() check
                             //      default:[TRUE]
 const
{
  // locals 
  ULONG ii ;
  SmFace          * pFace = GetFace() ; 
  SmTArray<SmEdge*> sEdges ;
  GetEdges(sEdges) ;

  // for every edge - return FALSE at 1st NonEmbeded Edge
  for(ii=0;ii<sEdges.GetSize();ii++)
    {
      SmEdge * pEdge = sEdges[ii] ;

      // Embedded Edge check
      if(FALSE == pEdge->IsEmbedded(pFace, bFaceHasSeam))
        { return(TRUE) ; }
    } // end iter every Loop->Edge

  // arrive here for VertexLoops and when all Edges in EdgeLoops are Embedded 
  return(TRUE) ; 

} // end SmLoop::IsEmbedded

/*******************************************************************//**
PURPOSE: Return list of UVBoundaries that are touched by the 
  corners of this loop in UVSpace.

NOTES: This query is used to determine when a loop has a corner
  on the boundaries of the given UVDomain.  This information is
  used to determine if a loop may be bounded by a surface's singular
  boundary (a pole).
***********************************************************************/
ULONG SmLoop::GetUVCornerBoundaries  // rtn: SM_SS_NONE or orof:[SM_SS_UMIN SM_SS_VMIN SM_SS_UMAX SM_SS_VMAX]
 (const SmExtent2d  & crUVDomain,    // in : UVDomain to test
  double              dTol,          // in : UV domain tolerance
  SmTArray<SmEdge *> & rOnEdges,     // out: list of edges with endPoints on a crUVDomain boundary
  SmTArray<ULONG>    & rOnBndries,   // out: associated list of boundaries for rOnEdges list
  SmTArray<double>   & rOnParams)    // out: associated edge param for rOnEdges list
{
  // init return values
  ULONG lBndries = SM_SS_NONE ;
  rOnEdges.ReSet() ; 
  rOnBndries.ReSet() ;
  rOnParams.ReSet() ;

  // check state - no loopuse - no classification
  if(m_pLU == NULL)
    { return(lBndries) ; }

  // locals
  ULONG ii ;
  SmPoint2d        sUVPntBeg, sUVPntEnd ;
  SmPoint3d        sUVBeg, sUVEnd ;
  SmExtent1d       sIvl ;
  SmBSplineCurve * pUVTrimCurve = NULL ;
  ULONG            lTheseBndries ;

  // get edgeuses to check
  SmTArray<SmEdgeuse *> sEdgeuses ;
  m_pLU->GetEdgeuses(sEdgeuses) ;
  
  // for every edgeuse - classify the end points
  for(ii=0;ii<sEdgeuses.GetSize();ii++)
    {
      SmEdgeuse *pEdgeuse = sEdgeuses[ii] ;

      // get UVTrimCurve
      pEdgeuse->GetOrCreateUVTrimCurve(pUVTrimCurve) ;

      // Use point drop for problem edgeuses
      if ( pUVTrimCurve == NULL )
      {
          SM_DBG_WARN(_T("Failed to build UVTrimCurve. Recovery code proceeds by dropping 3D points."))
          double    dGap = 0.0;
          SmPoint3d sBeg, sEnd;
          SmBoolean bSuccess;
          SmBoolean bIsMulti;
          SmEdge   *pE = pEdgeuse->GetEdge();
          SmCurve  *pC = pE->GetCurve();
          SmFace   *pF = pEdgeuse->GetFace();
          SmSurface *pS = pF->GetSurface();
          SmExtent2d sUVDomain = pF->GetUVDomain();
          sIvl = pE->GetInterval();

          pC->EvaluatePoint( sIvl.GetMin(), sBeg );
          pC->EvaluatePoint( sIvl.GetMax(), sEnd );

          SE( pS->DropPoint( sBeg, sUVDomain, NULL, bSuccess, sUVPntBeg, dGap, bIsMulti ) );
          SE( pS->DropPoint( sEnd, sUVDomain, NULL, bSuccess, sUVPntEnd, dGap, bIsMulti ) );
      }
      else
      {
          // end points
          sIvl = pUVTrimCurve->GetNaturalInterval();
          pUVTrimCurve->EvaluatePoint( sIvl.Evaluate( 0.0 ), sUVBeg );
          pUVTrimCurve->EvaluatePoint( sIvl.Evaluate( 1.0 ), sUVEnd );

          sUVPntBeg.Set( sUVBeg.x, sUVBeg.y );
          sUVPntEnd.Set( sUVEnd.x, sUVEnd.y );
      }

      // classify the BegPoint 

      lTheseBndries = crUVDomain.GetPoint2dBoundaries(sUVPntBeg, dTol) ;
      lBndries |= lTheseBndries ;
      if(lTheseBndries) { rOnEdges.Add(pEdgeuse->GetEdge()) ;
                          rOnBndries.Add(lTheseBndries) ;
                          rOnParams.Add(sIvl.Evaluate(0.0)) ;
                        }

      // classify the endPoint - in a perfect world we can skip this, but allow for tolerance errors
      lTheseBndries = crUVDomain.GetPoint2dBoundaries(sUVPntEnd, dTol) ;
      lBndries |= lTheseBndries ;
      if(lTheseBndries) { rOnEdges.Add(pEdgeuse->GetEdge()) ;
                          rOnBndries.Add(lTheseBndries) ;
                          rOnParams.Add(sIvl.Evaluate(1.0)) ;
                        }

    } // end iter every loop->edgeuse->UVTtrimCurve checking end points against domain boundaries
  
  // all done
  return(lBndries) ;

} // end SmLoop::GetUVCornerBoundaries   

/*******************************************************************//**
PURPOSE: return the quadrant number containing crTestPt for a 2d coordinate
         system centered on crOrigin 

NOTES:    1 | 0      quadrant numbering
        ----+-----
          2 | 3
***********************************************************************/
static ULONG sm_CalcQuadNum( const SmPoint3d &crOrigin, const SmPoint3d &crTestPt )
{
  ULONG lRetVal = ( crTestPt.x >= crOrigin.x ) ? ( ( crTestPt.y  < crOrigin.y ) ? 3 : 0 )
                                               : ( ( crTestPt.y >= crOrigin.y ) ? 1 : 2 ) ;
  return lRetVal;

} // end sm_CalcQuadNum

/*******************************************************************//**
PURPOSE: Determine whether this loop contains a given uv point.

RETURNS: oneof SM_POC_INSIDE, SM_POC_OUTSIDE, SM_POC_ON_BOUNDARY when successful
         SM_POC_UNKNOWN =    UVTrimCurve can't be built,
                        = or BadDatabase Loop crosses a seam
                        = or any other error.

NOTES: 
  Checks containment for this loop only; doesn't check for other interior loops.

  note: Points on surface singular boundaries map as SM_POC_ON_BOUNDARY
        to any loop that has any point on the same singular boundary
***********************************************************************/
SmPointObjectContainmentType SmLoop::ContainsUVPoint // rtn: oneof SM_POC_INSIDE, SM_POC_OUTSIDE, SM_POC_ON_BOUNDARY or SM_POC_UNKNONW = Error 
 (const SmPoint2d   & crUVPoint,                     // in : Point to classify
  double              dTol3d,                        // NotUsed: in : 3-space tolerance.
  SmEdge          * & rpOnEdge,                      // out: only set when rtn value is SM_POC_ON_BOUNDARY
  double            & rdEdgeParam,                   // out: only set when rtn value is SM_POC_ON_BOUNDARY
  SmOrientType      & eOrientation)                  // out: only set when rtn value is SM_POC_INSIDE
{    
  SM_REF1(dTol3d) ; 
  // Init outputs:
  rpOnEdge     = NULL;
  rdEdgeParam  = 0;
  eOrientation = SM_OT_UNKNOWN;

  // Work from Loopuses, because they will give us an ordered list of Edgeuses.
  SmTArray< SmEdgeuse* > apEUs;
  m_pLU->GetEdgeuses( apEUs );
  ULONG lNumEUs = apEUs.GetSize();

  if ( lNumEUs < 1 )  // could be a vertex loopuse
    { return SM_POC_OUTSIDE; }

  // locals
  SmFace     *pFace             = GetFace() ;
  SmSurface  *pSurface          = pFace ? pFace->GetSurface() : NULL ;

  // check state - must have a Face and surface
  if(pFace == NULL || pSurface == NULL)
    {
      SM_DBG_WARN(_T("SmLoop::ContainsUVPoint Bad State: Loop can't find owning Face - database is corrupt - returning SM_POC_UNKONW classification")) ;
      return(SM_POC_UNKNOWN) ;
    }

  SmExtent2d  sBigDomain( -SM_BIG_DOUBLE, -SM_BIG_DOUBLE, SM_BIG_DOUBLE, SM_BIG_DOUBLE) ;
  SmExtent2d  sSrfUVDomain      = pSurface ? pSurface->GetNaturalUVDomain() : sBigDomain ;
  // SmBoolean   bCrossedSeam      = FALSE ;    // gwc: removed new feature - it caused cascading failures
  SmPoint3d   s1stUV ;
  SmPoint3d   sPoint3d;
  pSurface->EvaluatePoint( crUVPoint, sPoint3d );

  // Get a tolerance in uv space.
  // Scale the input tolerance by the UV deriv size.
  SmZoneTol3d sZoneTol3d = SmTol::GetZoneTol3d( pSurface ); 
  SmTol2d sTolUV = SmTol::MapTo2d( sZoneTol3d, crUVPoint, *pSurface );
  double dTolUV = sTolUV.val;
  //SmPoint3d sSrfPt;
  //SmVector3d sSu, sSv;
  //pSurface->Evaluate1stDerivatives( crUVPoint, FALSE, FALSE, sSrfPt, sSu, sSv );
  //double dTolUV = smos_Max( dTol3d, (double)pFace->GetTolerance() );
  //double dScale = smos_Max( sSu.Length(), sSv.Length() );
  //dTolUV /= dScale;

  //cbiTol: This method would be a good place to separate u- and v-tol values.
  //cbiTol  That will be difficult, in part because of subroutines called
  //cbiTol  from here, such as LocalPointSolve().
  
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  
  // for debug graphics: function scope, plane, crUVPoint, sULine, and sVLine
  SmBrep     *pBrep    = pFace ? pFace->GetBrep() : NULL ;
  SmPlane    *pPlane   = NULL ;
  SmObjDelete sCleanPlane(pPlane) ;
  SmTArray<SmPoint2d> sULine(2,NULL,2), sVLine(2,NULL,2) ;
  SmPoint2d sTmp = crUVPoint ; 
  sTmp.x -=  5.0 ; sULine.SetAt(0,sTmp) ;
  sTmp.x += 10.0 ; sULine.SetAt(1,sTmp) ;
  sTmp = crUVPoint ; 
  sTmp.y -=  5.0 ; sVLine.SetAt(0,sTmp) ;
  sTmp.y += 10.0 ; sVLine.SetAt(1,sTmp) ;

  // draw loop in 2d and 3d with associated point
  if(bDebugMe)
    {
      SM_ASSERT_VALID(pBrep) ;
      SM_ASSERT_VALID(this) ;
      SM_ASSERT_VALID(pSurface) ;

      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,1) ; if(pSurface) pSurface->DrawUV(8,8,FALSE,NULL,TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,0,0) ; if(pFace) pFace->Draw(SM_DM_CROSSHATCH) ; sm_GraphicsLoop() ;
      smgfx_SetLook(2,3, 1,0,0) ; if(pFace) pFace->DrawUVCurves() ; sm_GraphicsLoop() ;
      smgfx_SetLook(3,4, 0,1,0) ; if(m_pLU) m_pLU->Draw(3, TRUE, &pPlane) ; sCleanPlane.ReplaceObj(pPlane) ; sm_GraphicsLoop() ;
      smgfx_SetLook(7,8, 1,0,0) ; if(pSurface) pSurface->DrawAt(crUVPoint) ; sm_GraphicsLoop() ;
      smgfx_SetLook(7,8, 1,0,0) ; if(pPlane) pPlane->DrawAt(crUVPoint) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 1,0,0) ; if(pPlane) pPlane->DrawUVPolyline(sULine) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 1,0,0) ; if(pPlane) pPlane->DrawUVPolyline(sVLine) ; sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

  // watch out for poles
  //  note: The following are bit arrays = orof:[SM_SS_UMIN SM_SS_VMIN SM_SS_UMAX SM_SS_VMAX]
  SmTArray<SmEdge *> sOnEdges ;
  SmTArray<ULONG>    sOnBndries ;
  SmTArray<double>   sOnParams ;

  // minimize pole checking - if UVPnt on boundary -then-> if Loop on boundary -then-> if GetSingularities -then-> expensive check

  // Is crUVPoint on Surface Natural UVBoundary (where poles are allowed)
  ULONG lUVBoundaries   = sSrfUVDomain.GetPoint2dBoundaries(crUVPoint, dTolUV) ;

  // When crUVPoint is on Surface UVBoundary - does Loop have any edgeuse ends on Surface Natural UVBoundary
  ULONG lLoopBoundaries = lUVBoundaries ? GetUVCornerBoundaries(sSrfUVDomain, dTolUV, sOnEdges, sOnBndries, sOnParams) : 0 ;
  
  // When point and loop touch the surface UVBoundaries - does surface have poles  
  ULONG lPoles          = (lUVBoundaries & lLoopBoundaries) ? pSurface->GetSingularities() : 0 ;

  // remember when checking for points on poles is required
  ULONG lCommonPoles    = lPoles & lLoopBoundaries & lUVBoundaries ;
  
  // when crUVPoint and Loop share a common singular edge - classify the point
  if(lCommonPoles) 
    {
      // set output with first hit on a singluar boundary
      for(ULONG ii=0;ii<sOnEdges.GetSize();ii++)
        {
          if(sOnBndries[ii] & lCommonPoles )
            {
              rpOnEdge    = sOnEdges[ii] ;
              rdEdgeParam = sOnParams[ii] ;
              break ;
            }
        } // end iter every sOnEdge hit looking for 1st on a singular boundary

      // UVPoints on Singular boundaries shared by the loop are always classified ON_BOUNDARY
      return(SM_POC_ON_BOUNDARY) ; 

    } // end UVPoint was on a surface pole shared by this loop check                                   

  // Method: Winding algorithm: step along each Edgeuse in uv space,
  // checking which quadrant each sample point is in relative to the test point,
  // and accumulating the changes in quadrant number.  If the test point
  // is inside the loop, the winding number should be +- 4, and 0 if outside.
  // Also check for 'on' while looping.
  //
  // Issues:
  // Loops are not always closed in uv space, such as at the poles of a sphere.
  // (The curve along the degenerate uv boundary would be degenerate in 3d space,
  // so no degenerate edge is added, so no Edgeuse, and no uv curve along the
  // degenerate boundary.)  Therefore, also include the step from the end of
  // the previous edgeuse to the beginning of the current one.
  //
  // Pole handling: Edgeuse UVTrimCurves can only start or end on poles.  Any
  //        UVTrimCurve EndPoint coincidence with a boundary classifies the 
  //        entire loop as being on the boundary.  When crUVPoint and the Loop
  //        are on a common singular edges of the loop->Face->Surface NaturalUVDomain
  //        then the crUVPoint is classifed as ON_Boundary to this loop.
  //
  // Outline:
  // First grab the end uv point of the final edgeuse, in case of an open loop.
  // FOR each Edgeuse
  //    Get start point of first Edgeuse, check vs. last end point
  //    While not done with this EU, march along it:
  //       figure next parameter
  //       Eval point there
  //       figure quadrant and accumulate
  //    end While
  //    update last end point
  // end For
  //
  // State at top of inner loop:
  // - dThisParam    -- current point parameter on EU
  // - sThisUV       -- current point on EU
  // - sThisUVTan    -- current (unit) tangent on EU
  // - dThisUVTanLen -- length of current deriv on EU
  // - sThisVec      -- (unit) vector from crUVPoint to sThisUV
  // - dThisDist     -- distance from crUVPoint to sThisUV
  // - dThisCos      -- dot pr. of sThisUVTan and sThisVec, to check approaching/receding
  // - lThisQuadNum  -- of sThisUV

  // check input:
  if ( dTolUV < SM_EFF_ZERO*100 )
    { dTolUV = SM_EFF_ZERO*100; }

  // Locals:
  int   iAccum       = 0;  // This is the main result: 0, 4 or -4 when done.
  ULONG lThisQuadNum = 0;  // Quadrant numbers: 0-3 going CCW.
  ULONG lPrevQuadNum; 
  int   iDelta;                         
                                        
  SmBoolean bFoundAnswer;               
  SmSolution sSol;                      

  SmStatus   eStat;
  SmEdgeuse *pThisEU;
  SmCurve   *pThisUVCurve;

  SmOrientType eOrient;
  SmExtent1d   sDomain;
  double       dEndParam;

  double     dPrevParam;
  SmPoint3d  sPrevUV;
  SmVector3d sPrevUVTan, sPrevVec;
  double     dPrevUVTanLen, dPrevDist, dPrevCos;

  double     dThisParam = 0.0;
  SmPoint3d  sThisUV;
  SmVector3d sThisUVTan, sThisVec;
  double     dThisUVTanLen, dThisDist, dThisCos;

  double dSegDist, dSegParam;

  // Also: sometimes loops are not closed, such as at singularities.
  // In that case, we need to compare the end of each Edgeuse with
  // the start of the next, if they are different points in uv space.
  // For that, keep track of the end of the previous Edgeuse.
  // We will need:
  // - sLastEndUV      : uv eval of pLastEndEU at dLastEndParam
  // - lLastEndQuadNum : quadrant number there.
  //
  // And for return arguments:
  // - pLastEndEU      : ptr to previous Edgeuse
  // - dLastEndParam   : start or end param (depending on orientation)
  //
  // Start with the end of the final Edgeuse.

  SmEdgeuse *pLastEndEU;
  double     dLastEndParam;
  SmPoint3d  sLastEndUV;
  ULONG      lLastEndQuadNum;

  pLastEndEU = apEUs[ lNumEUs-1 ];
  dLastEndParam = ( pLastEndEU->GetOrientation() == SM_OT_SAME )
                    ? pLastEndEU->GetEdge()->GetInterval().GetMax()
                    : pLastEndEU->GetEdge()->GetInterval().GetMin();
  SmStatus stat = pLastEndEU->GetOrCreateUVTrimCurve( pThisUVCurve );
  if( stat != SM_SUCCESS || pThisUVCurve == NULL ) 
    {
      SE( SM_ERR );
      return SM_POC_UNKNOWN;
    }

  pThisUVCurve->EvaluatePoint( dLastEndParam, sLastEndUV );
  lLastEndQuadNum = sm_CalcQuadNum( crUVPoint, sLastEndUV );

  // Now start the outer loop, on Edgeuses.

  ULONG idx;
  for ( idx = 0; idx < lNumEUs; idx++ )
    {
      pThisEU = apEUs[ idx ];
      SmEdge  *pEdge = pThisEU->GetEdge();
      SmCurve *pEdgeCurve = pEdge->GetCurve();
      SmZoneTol3d sEdgeZoneTol3d = SmTol::GetZoneTol3d( pThisEU->GetEdge() );
      double dParam, dDist;
      SmBoolean bSuccess;

      // low work: see if TgtPoint is on this EdgeCurve
      pEdgeCurve->DropPoint( pEdge->GetInterval(), sPoint3d, NULL, sEdgeZoneTol3d, NULL, bSuccess, dParam, dDist, SM_SO_INTERSECT );
      if ( bSuccess )
      {
          // TgtPoint is on EdgeCurve - set output and return
          rpOnEdge = pEdge;
          rdEdgeParam = dParam;
          return SM_POC_ON_BOUNDARY;
      } // end TgtPoint on EdgeCurve check

      eStat = pThisEU->GetOrCreateUVTrimCurve( pThisUVCurve );
      if ( eStat != SM_SUCCESS || pThisUVCurve == NULL )  // we can't work.
        {
          SE( SM_ERR );
          return SM_POC_UNKNOWN;
        }

#ifdef SM_DEBUG_CODE // draw uv curve in 2d
      if ( bDebugMe ) 
        {
          if(FALSE)
            { smgfx_Erase() ; }
          smgfx_SetLook(5,6, 1,0,0) ; if(pSurface && pThisUVCurve) pSurface->DrawUVCurve(*pThisUVCurve) ; sm_GraphicsLoop() ; 
          smgfx_SetLook(5,6, 1,0,0) ; if(pPlane   && pThisUVCurve) pPlane->DrawUVCurve(*pThisUVCurve) ; sm_GraphicsLoop() ; 

          smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
          smgfx_SetLook(1,2, 0,1,1) ; if(pSurface) pSurface->DrawUV(8,8,FALSE,NULL,TRUE) ; sm_GraphicsLoop() ;
          smgfx_SetLook(1,2, 0,0,0) ; if(pFace) pFace->Draw(SM_DM_CROSSHATCH) ; sm_GraphicsLoop() ;
          smgfx_SetLook(2,3, 1,0,0) ; if(pFace) pFace->DrawUVCurves() ; sm_GraphicsLoop() ;
          smgfx_SetLook(3,4, 0,1,0) ; if(m_pLU) m_pLU->Draw(3, TRUE, &pPlane) ; sCleanPlane.ReplaceObj(pPlane) ; sm_GraphicsLoop() ;
          smgfx_SetLook(7,8, 1,0,0) ; if(pSurface) pSurface->DrawAt(crUVPoint) ; sm_GraphicsLoop() ;
          smgfx_SetLook(7,8, 1,0,0) ; if(pPlane) pPlane->DrawAt(crUVPoint) ; sm_GraphicsLoop() ;
          smgfx_SetLook(1,2, 1,0,0) ; if(pPlane) pPlane->DrawUVPolyline(sULine) ; sm_GraphicsLoop() ;
          smgfx_SetLook(1,2, 1,0,0) ; if(pPlane) pPlane->DrawUVPolyline(sVLine) ; sm_GraphicsLoop() ;
          sm_GraphicsLoop() ;
       }
#endif // SM_DEBUG_CODE

      // Grab some info about this EU and its uv curve.
      eOrient    = pThisEU->GetOrientation();
      sDomain    = pThisEU->GetEdge()->GetInterval();
      dPrevParam = sDomain.GetMin();
      dEndParam  = sDomain.GetMax();
      if ( eOrient == SM_OT_OPPOSITE )
        {
          dPrevParam = sDomain.GetMax();
          dEndParam  = sDomain.GetMin();
        }

      // These have to do with step size:
      double dDefaultParamStep = sDomain.GetLength() / pThisUVCurve->GetNumberControlPoints();
      double dMinParamStep     = sDomain.GetLength() / 1000.0;
      double dMaxParamStep     = sDomain.GetLength() / 20.0;
      SmBoolean bCurveIsLinear = pThisUVCurve->IsLinear( dTolUV );

      // Eval info for start point of this EU.
      SmVector3d sPtDer[2];
      pThisUVCurve->Evaluate( dPrevParam, 1, FALSE, sPtDer );
      sPrevUV       = sPtDer[0];
      sPrevUVTan    = sPtDer[1];
      if ( eOrient == SM_OT_OPPOSITE )
        { sPrevUVTan = -sPrevUVTan; }
      dPrevUVTanLen = sPrevUVTan.Length();
      if ( dPrevUVTanLen < SM_EFF_ZERO )
        { dPrevUVTanLen = 1.0; }
      sPrevUVTan /= dPrevUVTanLen;
      sPrevVec    = sPrevUV - crUVPoint;
      dPrevDist   = sPrevVec.Length();
      if ( dPrevDist > SM_EFF_ZERO )
        { sPrevVec /= dPrevDist; }
      dPrevCos = sPrevVec.Dot( sPrevUVTan );

      // save 1st eval of 1st edgeuse for gap check to last eval of last edgeuse
      if( idx == 0)
        { s1stUV = sPrevUV ; }

      // gwc: removed new feature - it caused cascading failures
      //  // check for a large EdgeuseEnd_to_EdgeuseEnd UVGap not on a pole (probably crossed a seam - bad database case) 
      //  if(idx > 0)
      //    {
      //      SmPoint2d sPrev2d(sPrevUV.x, sPrevUV.y) ;
      //      SmSurfParamType eSurfParam ;
      //      SmVector3d sEdgeuseToEdgeuseGapUV = sPrevUV - sLastEndUV ;
      //      if(   (   smos_Fabs(sEdgeuseToEdgeuseGapUV.x) > sSrfUVDomain.XLength()/2
      //             || smos_Fabs(sEdgeuseToEdgeuseGapUV.y) > sSrfUVDomain.YLength()/2)
      //         && FALSE == pSurface->IsSingularity(sPrev2d, eSurfParam))
      //        { bCrossedSeam = TRUE ; }
      //    }
      // end removed new feature

      // Note: if dPrevDist < tol, then this uv pt is on the edge.
      if ( dPrevDist < dTolUV )
        {
          // We should refine the point.
          pThisUVCurve->LocalPointSolve(pThisEU->GetEdge()->GetInterval(), // in : search interval
                                        SM_SO_MINIMIZE,                    // in : specify specific operation to optimize
                                        crUVPoint,                         // in : point specializing this search
                                        &dTolUV,                           // in : Opt Max allowed solution distance (NULL to ignore) for
                                                                           //      SM_SO_RAYFIRE
                                                                           //      SM_SO_3D_SIGNED_DIRECTED_MINIMIZE
                                        NULL,                              // in : required curve/TestPoint desired dist (NULL when not used) for
                                                                           //      SM_SO_AT_DISTANCE
                                        NULL,                              // in : required Vector direction (NULL when not used) for
                                                                           //      SM_SO_RAYFIRE
                                                                           //      SM_SO_3D_SIGNED_DIRECTED_MINIMIZE
                                        dPrevParam,                        // in : search starting parameter
                                        bFoundAnswer,                      // out: TRUE=converged,FALSE=didn't
                                        sSol) ;                            // out: solution container for solver

          // since the guess was good enough - we expect the refined solution to be as good or better
          if ( bFoundAnswer && sSol.m_vStart.m_dSolutionValue <= dTolUV )
            { // return the refined answer.
              rpOnEdge    = pThisEU->GetEdge();
              rdEdgeParam = sSol.m_vStart[0];
              return SM_POC_ON_BOUNDARY;
            }

          // Should not arrive here...
          SM_DBG_WARN(_T("Warning: Inconsistent classification in SmLoop::ContainsUVPoint(). Continuing.\n") );
        }

      // Calculate the initial quadrant.
      lPrevQuadNum = sm_CalcQuadNum( crUVPoint, sPrevUV );

      // Check here for non-closed loops.

      iDelta = lPrevQuadNum - lLastEndQuadNum;

      // Check for and accumulate any change in quadrant.
      if ( iDelta != 0 )
        {
          // First check 'on'.
          // Note, this would presumably mean that the given point is at
          // a singularity, but in between the ends of the Edgeuses,
          // in uv space (because we've already checked it being the same
          // as either of the two points).
          // If that happens, return whichever end point it's closer to.

          eStat = smgu_LinePointDistance( sLastEndUV, (sPrevUV-sLastEndUV), crUVPoint,
                  dSegDist, &dSegParam );
          if ( dSegDist <= dTolUV && dSegParam > -SM_EFF_ZERO && dSegParam < 1.0+SM_EFF_ZERO )
            {
              // On. Return closer end point.
              if ( dSegParam <= 0.5 )
                {
                  rpOnEdge = pLastEndEU->GetEdge();
                  rdEdgeParam = dLastEndParam;
                }
              else
                {
                  rpOnEdge = pThisEU->GetEdge();
                  rdEdgeParam = dPrevParam;
                }

              return SM_POC_ON_BOUNDARY;
            }

          if ( iDelta == 1 || iDelta == -3 )
            { iAccum++; }
          else if ( iDelta == -1 || iDelta == 3 )
            { iAccum--; }
          else
            {
              // +- 2: diagonal.  Here, we're just shooting from one point
              // to the next, so treat it as a straight line.
              // See whether we're heading CW or CCW around the test point:
              // use a cross product.

              SmVector3d sCross = ( sPrevUV-sLastEndUV ) * ( crUVPoint - sLastEndUV );
              if ( sCross.z > 0 )
                { iAccum += 2; } // CCW
              else
                { iAccum -= 2; } // CW
            }
        }  // end check for change from end of last curve

      // A flag to be used in this next loop:
      SmBoolean bDidLocalSolveLastTime = FALSE;

      // Now step along this Edgeuse.
      int iLoopCount1 = 0;
      while ( smos_Fabs( dEndParam - dPrevParam ) > SM_EFF_ZERO )
        {
          iLoopCount1++;
          if (iLoopCount1 > 1000)
          {
              SM_DBG_WARN(_T("Error: SmLoop::ContainsUVPoint() Infinite Loop.\n") );
              return SM_POC_UNKNOWN;
          }
          // Get the parameter for the next sample point.

          double dParamStep = dDefaultParamStep;

          // For straight lines, we can shoot right to the end.
          if ( bCurveIsLinear )
            {
              dParamStep = smos_Fabs( dEndParam - dPrevParam );
            }
          else
            {
              // When we're far from the test point, we can take bigger steps.
              // Also increase step size if we're moving away from the test point.
              // Calculate a step that is a fraction of the distance between the
              // current sample point and the test point, where the fraction
              // depends on the direction:
              // - directly away: step equals distance
              // - perpendicular: step is half the distance
              // - directly towards: step is 1/4 of the distance.
              // These three number pairs define a quadratic, mapping the cosine
              // of the angle to the fraction. ( [-1:0.25], [0:0.5], [1:1] )
              // Fraction = 1/8 cos^2  +  3/8 cos  +  1/2.

              double dFrac     = dPrevCos * ( dPrevCos + 3 ) / 8 + 0.5;
              double dMoveDist = dFrac * dPrevDist;
              dParamStep       = dMoveDist / dPrevUVTanLen;

              if ( dParamStep < dMinParamStep)
                {    dParamStep = dMinParamStep; }
              if ( dParamStep > dMaxParamStep )
                {    dParamStep = dMaxParamStep; }
            }

          // Got param step, set dThisParam

          if ( eOrient == SM_OT_SAME )
            {
              dThisParam = dPrevParam + dParamStep;
              if ( dThisParam > dEndParam )
                { dThisParam = dEndParam; }
            }
          else
            {
              dThisParam = dPrevParam - dParamStep;
              if ( dThisParam < dEndParam )
                { dThisParam = dEndParam; }
            }

          // Here we have the next candidate curve parameter, for a step.
          // Evaluate the curve and get all of the state variables we need.
          // Note, it might turn out that this parameter is not appropriate
          // for one reason or another: too big a step, etc.
          // When a better value is found, just come back to here.

          int iLoopCount2 = 0;
Got_dThisParam:

          iLoopCount2++;
          if (iLoopCount2 > 1000)
          {
              SM_DBG_WARN(_T("Error: SmLoop::ContainsUVPoint() Infinite Loop2.\n") );
              return SM_POC_UNKNOWN;
          }

          pThisUVCurve->Evaluate( dThisParam, 1, FALSE, sPtDer );
          sThisUV = sPtDer[0];
          sThisUVTan = sPtDer[1];
          if ( eOrient == SM_OT_OPPOSITE )
            { sThisUVTan = -sThisUVTan; }
          dThisUVTanLen = sThisUVTan.Length();
          if ( dThisUVTanLen < SM_EFF_ZERO )
            { dThisUVTanLen = 1.0; }
          sThisUVTan   /= dThisUVTanLen;
          sThisVec      = sThisUV - crUVPoint;
          dThisDist     = sThisVec.Length();
          if ( dThisDist > SM_EFF_ZERO )
            { sThisVec /= dThisDist; }
          dThisCos      = sThisVec.Dot( sThisUVTan );

          // A check for too big a step: if the direction changed
          // by more than, say, 90 degrees.
          // But, if the curve is linear, and it doubles back on itself
          // (it happens), we don't have to step along, can just shoot to
          // the end. [Fillet regression 2:237]
          if ( !bCurveIsLinear && sPrevUVTan.Dot( sThisUVTan ) < 0 )
            {
              // Cusps happen.  In case of a turnaround in a very short interval,
              // just continue on with the next parameter value.
              // Should be safe enough at this granularity.
              if ( smos_Fabs( dThisParam - dPrevParam ) > dMinParamStep )
                {
                  dThisParam = ( dThisParam + dPrevParam ) / 2;
                  goto Got_dThisParam;
                }

#ifdef SM_DEBUG_CODE
              // arrive here when Ivl TurnAng is still > 180 deg but the 
              //  ParamStep size is less than dMinParamStep size.
              //  this can happen for ValidModel very small radius turns 
              //    (eg. where rolling ball blends taper out.)
              //  and for InvalidModel cusps 
              //    (eg. a BSpline where the end couple CtrlPts are on a line but out of order due to bad snapping)
              //  In either case this is a scheme for continuing processing
              if( TRUE == !bCurveIsLinear && sPrevUVTan.Dot( sThisUVTan ) < 0 )
                {
                  SM_DBG_WARN( _T("Info: SmLoop::ContainsUVPoint(): SampleBisectionStep below MinParamStep Size(stepping thru SmallRadiusTurn or InvalidCusp) - Not a bug") );

              if(bDebugMe)
                {
                      ULONG di ;
                  SmPoint2d sProblemUV( sThisUV.x, sThisUV.y );
                  SmBSplineCurve *pNewUVCurve = NULL ;

                  // to debug the problem UVTrimCurve build sequence: rebuild the UVTrimCurve here
                  if(FALSE)
                    {
                      double rdMaxDistanceToSurface ;
                      pThisEU->CreateUVTrimCurve(rdMaxDistanceToSurface, pNewUVCurve) ;
                    }
                  SmObjDelete sClean(pNewUVCurve) ;

                  smgfx_Erase() ; 
                  smgfx_SetLook(3,4, 0,1,0) ; if(m_pLU) m_pLU->Draw(3, TRUE, &pPlane) ; sCleanPlane.ReplaceObj(pPlane) ; sm_GraphicsLoop() ;
                  smgfx_SetLook(5,6, 1,0,0) ; if(pSurface && pThisUVCurve) pSurface->DrawUVCurve(*pThisUVCurve) ; sm_GraphicsLoop() ; 
                  smgfx_SetLook(5,6, 1,0,0) ; if(pPlane   && pThisUVCurve) pPlane->DrawUVCurve(*pThisUVCurve) ; sm_GraphicsLoop() ; 
                  smgfx_SetLook(9,10,1,0,1) ; if(pSurface) pSurface->DrawAt(sProblemUV) ; sm_GraphicsLoop() ;
                  smgfx_SetLook(9,10,1,0,1) ; if(pPlane) pPlane->DrawAt(sProblemUV) ; sm_GraphicsLoop() ;

                  smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
                  smgfx_SetLook(1,2, 0,1,1) ; if(pSurface) pSurface->DrawUV(8,8,FALSE,NULL,TRUE) ; sm_GraphicsLoop() ;
                  smgfx_SetLook(3,4, .2,.2,.2) ; if(pSurface) pSurface->DrawPoles() ; sm_GraphicsLoop() ;
                  smgfx_SetLook(3,4, .5,.5,.5) ; if(pSurface) pSurface->DrawSeams() ; sm_GraphicsLoop() ;
                  smgfx_SetLook(1,2, 0,0,0) ; if(pFace) pFace->Draw(SM_DM_CROSSHATCH) ; sm_GraphicsLoop() ;
                  smgfx_SetLook(2,3, 1,0,0) ; if(pFace) pFace->DrawUVCurves() ; sm_GraphicsLoop() ;
                  smgfx_SetLook(7,8, 1,0,0) ; if(pSurface) pSurface->DrawAt(crUVPoint) ; sm_GraphicsLoop() ;
                  smgfx_SetLook(7,8, 1,0,0) ; if(pPlane) pPlane->DrawAt(crUVPoint) ; sm_GraphicsLoop() ;
                  smgfx_SetLook(1,2, 1,0,0) ; if(pPlane) pPlane->DrawUVPolyline(sULine) ; sm_GraphicsLoop() ;
                  smgfx_SetLook(1,2, 1,0,0) ; if(pPlane) pPlane->DrawUVPolyline(sVLine) ; sm_GraphicsLoop() ;
                  sm_GraphicsLoop() ;

                  SmTArray<SmEdge*> sEdges ; GetEdges(sEdges) ; 
                      for(di=0;di<sEdges.GetSize();di++)
                        { SmEdge  *pDebugEdge  = sEdges[di] ;
                      SmCurve *pCurve = pDebugEdge->GetCurve() ;
                      if(pSurface) { SmBSplineCurve *pUVTrimCurve = pDebugEdge->GetUVTrimCurveOfSurface(pSurface) ;
                                     pUVTrimCurve->Dump() ;
                                   }
                      smgfx_SetLook(2,3, 0,0,1) ; pDebugEdge->Draw(); sm_GraphicsLoop() ;
                      smgfx_SetLook(3,4, 1,0,1) ; pCurve->DrawControlPoints() ; sm_GraphicsLoop() ;
                      smgfx_SetLook(4,5, 0,0,1) ; pCurve->DrawParams() ; sm_GraphicsLoop() ;
                      sm_GraphicsLoop() ;
                        } // end iter di, every Edge
                    } // end if(DebugMe) for graphics
                } // end has a problem check
#endif // SM_DEBUG_CODE
            } // end too big of a step because the tangent direction changed by more than 180 deg check

          // Here we need to check whether this segment contains the test point.
          SmBoolean bCheckOnCurve = FALSE;

          // If we do check on-curve, we will use dSegParam to find a guess
          // parameter for LocalPointSolve.  (dSegParam is where the test
          // point projects to the chord from prev pt to this pt.  Interpolate
          // the current curve interval with it.)
          dSegParam = 0.5;  // just in case nothing else sets it...

          // First, if the current point equals the test point:
          if ( dThisDist <= dTolUV )
            {
              bCheckOnCurve = TRUE;
              dSegParam = 1.0; // end of segment
            }

          // If the previous point was approaching and this point is receding:
          // Note if this cosine is zero, then we're at a relative min (or max).
          // That would be because we did a LocalSolve last time,
          // so don't do one again.
          // Note: It's possible that, even with a local solve, a cosine could
          // be fairly large, e.g. > 0.0001.  This can happen on small geometry.
          // So use a Boolean flag to avoid numerical issues.

          if (    ! bCheckOnCurve
               && ! bDidLocalSolveLastTime
               && ( dPrevCos < -0.001 && dThisCos > 0.001 )
             )
            {
              eStat = smgu_LinePointDistance( sPrevUV, (sThisUV-sPrevUV), crUVPoint,
                      dSegDist, &dSegParam );

              // Check whether we should check whether the pt is on the curve.
              bCheckOnCurve = ( dSegDist <= dTolUV );

              // Note, because of the signs of the cosines, we know that
              // the test point will project to the interior of this
              // curve segment.  It might project exterior to the line segment,
              // however, so just tidy that up.  Not a big deal.
              if ( dSegParam < 0 ) { dSegParam = 0; }
              if ( dSegParam > 1 ) { dSegParam = 1; }

              // Check whether this curve segment could possibly reach the test pt.
              if ( !bCheckOnCurve && !bCurveIsLinear )
                {
                  // Do more detailed check(s).
                  // Extend start & end tangents to see whether they could
                  // reach as far as the test point.
                  SmPoint3d sChkPt;
                  double dChkPtDist, dChkPtParam;

                  // We know that dPrevCos < 0...
                  sChkPt = sPrevUV + sPrevUVTan * dPrevUVTanLen
                              * smos_Fabs( dThisParam - dPrevParam );

                  eStat = smgu_LinePointDistance( sPrevUV, (sThisUV-sPrevUV), sChkPt,
                      dChkPtDist, &dChkPtParam );

                  bCheckOnCurve = ( dChkPtDist >= dSegDist );

                  // ... and that dThisCos > 0.
                  if ( !bCheckOnCurve )
                    {
                      // Try other end of segment.
                      sChkPt = sThisUV - sThisUVTan * dThisUVTanLen
                                  * smos_Fabs( dThisParam - dPrevParam );

                      eStat = smgu_LinePointDistance( sPrevUV, (sThisUV-sPrevUV), sChkPt,
                          dChkPtDist, &dChkPtParam );

                      bCheckOnCurve = ( dChkPtDist >= dSegDist );
                    }
                } // end more-detailed check for possible on-curve.
            } // end cosine-change check, for passing the test point.

          bDidLocalSolveLastTime = FALSE;

          if ( bCheckOnCurve )
            {
              // We have a very good guess parameter for a local solve.
              double dGuessT = dPrevParam + dSegParam * ( dThisParam - dPrevParam );

              // Interval: between prev pt and this pt.  Loosen up a bit.
              SmExtent1d sCrvIvl;
              sCrvIvl.AddValue( dPrevParam );
              sCrvIvl.AddValue( dThisParam );
              sCrvIvl.ExpandAbsolute( SM_EFF_ZERO );

              pThisUVCurve->LocalPointSolve(sCrvIvl,         // in : search interval
                                            SM_SO_MINIMIZE,  // in : specify specific operation to optimize
                                            crUVPoint,       // in : point specializing this search
                                            &dTolUV,         // in : Opt Max allowed solution distance (NULL to ignore) for
                                                             //      SM_SO_RAYFIRE
                                                             //      SM_SO_3D_SIGNED_DIRECTED_MINIMIZE
                                            NULL,            // in : required curve/TestPoint desired dist (NULL when not used) for
                                                             //      SM_SO_AT_DISTANCE
                                            NULL,            // in : required Vector direction (NULL when not used) for
                                                             //      SM_SO_RAYFIRE
                                                             //      SM_SO_3D_SIGNED_DIRECTED_MINIMIZE
                                            dGuessT,         // in : search starting parameter
                                            bFoundAnswer,    // out: TRUE=converged,FALSE=didn't
                                            sSol) ;          // out: solution container for solver

              if ( bFoundAnswer )
                {
                  if ( sSol.m_vStart.m_dSolutionValue <= dTolUV )
                    {
                      rpOnEdge    = pThisEU->GetEdge();
                      rdEdgeParam = sSol.m_vStart[0];
                      return SM_POC_ON_BOUNDARY;
                    }

                  // We have a local min/max to the curve, it would have to
                  // be a min because of the geometry used to do the LocalSolve.
                  // Do another point at the solution value, to avoid the case
                  // where the test point is between the segment chord and
                  // the actual curve.
                  // Note, the solution will have a cosine of zero:
                  // use that fact to avoid getting stuck.

                  double dSolveT = sSol.m_vStart[0];

                  if ( sCrvIvl.ContainsValue( dSolveT, 0.0001 ) )
                    {
                      dThisParam = dSolveT;
                      bDidLocalSolveLastTime = TRUE;
                      goto Got_dThisParam;
                    }
                } // end if bFoundAnswer
            } // end LocalPointSolve check for test pt on curve


          // Got the next sample point, find its quadrant.

          lThisQuadNum = sm_CalcQuadNum( crUVPoint, sThisUV );

          iDelta = lThisQuadNum - lPrevQuadNum;

          // Check for and accumulate any change in quadrant.
          if ( iDelta == 0 )
            {}
          else if ( iDelta ==  1 || iDelta == -3 )
            { iAccum++; }
          else if ( iDelta == -1 || iDelta ==  3 )
            { iAccum--; }
          else
            {
              // +- 2: diagonal.  This really shouldn't happen, because of the
              // previous checks for testing on-curve, but it can sometimes
              // because of numerical noise.
              // Just take a smaller step; is doesn't have to be much smaller.
              dThisParam = dPrevParam + 0.75 * ( dThisParam - dPrevParam );
              goto Got_dThisParam;
            }

            // Update This/Prev data.
            dPrevParam    = dThisParam;
            sPrevUV       = sThisUV;
            sPrevUVTan    = sThisUVTan;
            dPrevUVTanLen = dThisUVTanLen;
            sPrevVec      = sThisVec;
            dPrevDist     = dThisDist;
            dPrevCos      = dThisCos;
            lPrevQuadNum  = lThisQuadNum;

        }  // end while stepping along this EU

      // Update the last-end stuff, for checking non-closed loops.
      pLastEndEU      = pThisEU;
      dLastEndParam   = dThisParam;
      sLastEndUV      = sThisUV;
      lLastEndQuadNum = lThisQuadNum;

      // gwc: removed new feature - it caused cascading failures
      //  // check for a large LastEdgeuseEnd_to_FirstEdgeuseEnd UVGap not on a pole (probably crossed a seam - bad database case)
      //  if(idx == lNumEUs-1)
      //    {
      //      SmPoint2d s1st2d(s1stUV.x, s1stUV.y) ;
      //      SmSurfParamType eSurfParam ;
      //      SmVector3d sFinalEdgeuseToEdgeuseGapUV = s1stUV - sLastEndUV ;
      //      if(   (   smos_Fabs(sFinalEdgeuseToEdgeuseGapUV.x) > sSrfUVDomain.XLength()/2
      //             || smos_Fabs(sFinalEdgeuseToEdgeuseGapUV.y) > sSrfUVDomain.YLength()/2)
      //        && FALSE == pSurface->IsSingularity(s1st2d, eSurfParam))
      //       { bCrossedSeam = TRUE ; }
      //    }
      // end removed new feature

    }  // end loop on all EU's

  // one known problem:
  //   When a BadDatabase loop crosses a seam - the iAccum value will be off

  // gwc: removed new feature - it caused cascading failures
  //   returning UNKNOWN for bad loops as opposed to returning bad answers is causing cascading failures
  //   except when the healer is running we do not expect to see bad loops -
  //   this feature was only built to simplify debugging the healer (when there are corrupt databases)
  //   but its causing regular cases to fail - so turn this off
  // if(bCrossedSeam == TRUE)
  //   {
  //     eOrientation = SM_OT_UNKNOWN ;
  //     return SM_POC_UNKNOWN ;
  //   }
  // end removed new feature

  // Check the result: iAccum
  if ( iAccum == 0 )
    { return SM_POC_OUTSIDE; }
  else if ( iAccum == 4 || iAccum == -4 )
    {
      eOrientation = ( iAccum > 0 ) ? SM_OT_SAME : SM_OT_OPPOSITE;
      return SM_POC_INSIDE;
    }

  // arrive here when iAccum is off from a normal successful case
  
  // Ok, this is probably not a problem.  It's common for uv points to have
  // 'exactly' the same u or v value as the input point.  In that case,
  // a single bit of noise can change the quadrant classification by one.
  // So we'll allow a one-quadrant error here.

  if ( iAccum == 1 || iAccum == -1 )
    { return SM_POC_OUTSIDE; }
  else if (iAccum == 3 || iAccum == -3 || iAccum == 5 || iAccum == -5 )
    {
      eOrientation = ( iAccum > 0 ) ? SM_OT_SAME : SM_OT_OPPOSITE;
      return SM_POC_INSIDE;
    }

  // This is an error.  The only bit-noise error should be at the start/end
  // of the loop, and it can't be off by two.

  SM_DBG_WARN(_T("Error: SmLoop::ContainsUVPoint() Failed.\n") );
  return SM_POC_UNKNOWN;

} // end SmLoop::ContainsUVPoint

/*******************************************************************//**
PURPOSE: Recreate a Face from uv trim curves created from its existing Edges.

NOTES:
   Recreates (re-drops) all uv trim curves.
   Reorders the curves, in uv space.

   Useful when Edges' uv trim curves are bad,
   and/or when Loops have Edges in the wrong order.
***********************************************************************/
SmStatus SmLoop::RebuildFromEdgeCurves()
{
  SmTArray<SmEdgeuse*> sEdgeuses(256);
  m_pLU->GetEdgeuses( sEdgeuses );

  // Re-drop each Edgeuse.  Also collect their Edges.
  // Note, OrderCurvesIntoLoops() requires SmCurve*, not SmBSplineCurve* ...
  SmTArray< SmCurve* > sUVCurves;
  SmBSplineCurve * pUVBSpl = NULL;
  double dMaxDist, dMaxDev;

  ULONG lNumEUs = sEdgeuses.GetSize();
  for (ULONG ii=0; ii<lNumEUs; ii++)
    {
      // Re-drop the edgeuse's UVTrimCurve.
      SmEdgeuse *pEU = sEdgeuses[ii];
      pEU->SetUVTrimCurve( NULL, 0.0, TRUE) ; // TRUE = delete existing UVTrimCurve
      pEU->CreateUVTrimCurve( dMaxDist, pUVBSpl, &dMaxDev );
      NER( pUVBSpl ); // Can't do this algorithm if any fails.

      pEU->SetUVTrimCurve( pUVBSpl, dMaxDist );  // default: FALSE = don't delete existing UVTrimCurve

      sUVCurves.Add( pUVBSpl );
    }

  // Now sEdgeuses and sUVCurves correspond, in order.

  SmFace *pFace = this->GetFace();
  double d3dTol = pFace->GetTolerance();

// Here we could do SmTrimmingTools::CheckLoop() -- maybe that's all it needed.
// But that's expensive, probably more so than finishing this algorithm.
// Do it in Debug mode only for now.
#ifdef SM_DEBUG_CODE
  ULONG lClosure;
  SmStatus eStat = SmTrimmingTools::CheckLoop( pFace->GetSurface(), this, d3dTol, lClosure );
  if ( eStat == SM_SUCCESS && lClosure < 2 )
    { return SM_SUCCESS; } // Breakpoint here.
#endif

  // Now we have a list of all parameter-space trim curves.
  // Order them into Loops -- in uv space, not 3d.

  //cbiTol:
  // Tolerance: We pass in curves in uv space, so we need a uv-space tolerance.
  // Convert 3d tol to some kind of average u- and v-space tol.

  SmSurface  *pSurf  = pFace->GetSurface();
  SmExtent2d sDomain = pFace->GetUVDomain();
  SmVector2d sDerivLengths = pSurf->ApproxDerivativeLengths( sDomain );
  // For the 'average', we'll use the larger, to get a tighter uv tol.
  double dSdT = sDerivLengths.GetMaxDimension();
  // Delta t = Delta s / dSdT
  double dUVTol = d3dTol / dSdT;


  // Do the re-ordering.
  SmTArray<SmCurve*>     sOrderedUVCurves;
  SmTArray<ULONG>        sLoopCounts;
  SmTArray<SmOrientType> sOrients;
  SmPoint3d              sPlanePt;
  SmVector3d             sPlaneNorm;
  SmExtent3d             sBBox;

  SER( SmTrimmingTools::OrderCurvesIntoLoops(
    sUVCurves,
    dUVTol,
    sOrderedUVCurves,
    sLoopCounts,
    sOrients,
    sPlanePt, sPlaneNorm, sBBox
  ));

  // (There should be only one Loop.)
  SM_ASSERT( sLoopCounts.GetSize() == 1 );
  SM_ASSERT( sLoopCounts[0] == lNumEUs );


  // Get ordered EU list.
  // sUVCurves and sOrderedCurves contain the same set of pointers,
  // so we can do the mapping from those.

  SmTArray< SmEdgeuse* > sOrderedEUs;
  SmBoolean bFound = FALSE;

  ULONG lIndex, lNumUV = sUVCurves.GetSize();
  for ( ULONG ii=0; ii<lNumUV; ii++ )
  {
      SmCurve *pOrderedCurve = sOrderedUVCurves[ii];
      bFound = sUVCurves.FindElement( pOrderedCurve, lIndex );
      if ( ! bFound )
        { break; }

      sOrderedEUs.Add( sEdgeuses[ lIndex ] );
  }
  if ( ! bFound )
    { SER( SM_ERR ); }


  // Note, there should be only one Loop in sLoopCounts,
  // but we'll program for the general case anyway.
  ULONG lNumLoops = sLoopCounts.GetSize();
  
  // OrderCurvesIntoLoops() can order them backwards.
  // Since they're all uv curves (z == 0), the returned normal will be
  // either (0,0,1) or (0,0,-1).  CCW would be (0,0,1).
  if ( sPlaneNorm.z < 0.0 )  // (Same as Dot with (0,0,1).)
  {
      ULONG lStart, lEnd = 0;

      for ( ULONG ii=0; ii<lNumLoops; ii++ )
      {
          lStart = lEnd + 1;
          lEnd   = lEnd + sLoopCounts[ ii ];

          sOrderedEUs.ReverseArray( lStart, lEnd );
      }
  }

  // Now fix the order of the EUs within the Loops.
  // Just run down sOrderedEUs hooking up their CW and CCW's.
  SmEdgeuse *pThisEU, *pThisMate;
  SmEdgeuse *pNextEU, *pNextMate;
  ULONG lStart, lEnd = 0;

  for ( ULONG ii=0; ii<lNumLoops; ii++ )
  {
      lStart = lEnd;
      lEnd   = lEnd + sLoopCounts[ ii ];

      for ( ULONG jj=lStart; jj<lEnd; jj++ )
      {
          pThisEU = sOrderedEUs[jj];
          pNextEU = ( jj < lEnd-1 ) ? sOrderedEUs[ jj+1 ] : sOrderedEUs[ lStart ]; // cyclical

          // Hook up pThisEU with pNextEU as CCW.
          pThisEU->m_pCCW = pNextEU;
          pNextEU->m_pCW  = pThisEU;

          // And their Mates: they are opposite vis-a-vis CW and CCW.
          pThisMate = pThisEU->GetMate();
          pNextMate = pNextEU->GetMate();

          pThisMate->m_pCW  = pNextMate;
          pNextMate->m_pCCW = pThisMate;
      }
  }

  return SM_SUCCESS;

} // end RebuildFromEdgeCurves

/***********************************************************************
PURPOSE:  add primary loopuse loop->edges or loop->vertex graphics to new or open
             displayList added to global displayList array.

NOTES:
***********************************************************************/
SmDisplayList * SmLoop::Draw
 (ULONG           bVUAndEUDraw,    // in : oneof: 0=NoObjuses, 1=Draw VUs, 2=Draw EUs, default:[3]=Draw VUs and EUs
  SmBoolean       bDrawUVPlane,      // in : TRUE = Draw a UV Plane display of Loop near to 3d Loop rendering, default:[FALSE]
  SmPlane      ** pOptOutPlane,      // out: Set to Plane used for 2d graphics when bDrawUVPlane == TRUE, 
                                     //      NULL to ignore. default:[NULL],
                                     //      caller must delete this returned object
  SmBoolean       bDrawUVTrimCurves, // in : TRUE = Draw UVTrimCurves when present, FALSE=don't
                                     //        note: when UVTrimCurves are good - they draw on top of the edges and aren't seen
                                     //              when UVTrimCurves are bad - they vary widely from the edges and indicate debugging is needed
                                     //      default:[TRUE]
  SmGfxArraySet * pOptGfxSet)        // i/o: used for OpenGL ES 2.0. When given, accumulate GfxVertexArrays not GL calls.
                                   //      NULL to ignore. default:[NULL]
 const
{
  // pass the call along
  SmDisplayList *pRtn = m_pLU ? m_pLU->Draw(bVUAndEUDraw, bDrawUVPlane, pOptOutPlane, bDrawUVTrimCurves, pOptGfxSet) : NULL ;
  return(pRtn) ;

} // end SmLoop::Draw

/*******************************************************************//**
PURPOSE: AssertValid() Test reporting static labels

NOTES:
   See also SmTRimmingTools::SmAssertReportLabel sTrimmingCheckLoop_list[]
     lOwnerLabelIndex == 3
***********************************************************************/
SmAssertReportLabel sAssertLoop_list[] =
{
  /*  0 */ {SM_AT_POINTER,     _T("Bad Context"),      _T("Loop and m_pLU have different contexts") },
  /*  1 */ {SM_AT_NESTED_TEST, _T("Bad SubTopology"),  _T("A Loop SubTopology Edge or Vertex object failed its AssertValid checks") },
  /*  2 */ {SM_AT_GEOMETRIC,   _T("Bad Loop Orient"),  _T("A UpperLoopuse->Edgeuse's 'Inside' is Outside Surface Natural Boundary - loop may have wrong orientation") },
  /*  3 */ {SM_AT_POINTER,     _T("Face->Loop"),       _T("Face->EdgeLoopuse must have at least 1 edgeuse and Face->VertexLoopuses must have no edgesuses") },
  /*  4 */ {SM_AT_POINTER,     _T("Face->Loop"),       _T("Every loop->Loopuse->Edge must by used by both loop->loopuses") },
  /*  5 */ {SM_AT_POINTER,     _T("Face->Loop"),       _T("both loop->loopuses must connect to the same number of loop->edges") },
  /*  6 */ {SM_AT_POINTER,     _T("Face->Loop->Edgeuse lists"), _T("The order of edgeuses in paired face->loopuses must match") }
} ;

/*******************************************************************//**
PURPOSE:

RETURNS ---  TRUE  = OK
             FALSE = Problem
***********************************************************************/
SmBoolean SmLoop::AssertValid
 (SmAssertArray    * pAList,        // i/o: Accumulating list of failed Asserts, NULL to ignore, default:[NULL] 
  SmAssertTestLevel  eTestLevel,    // in : SM_LEVEL_0=fewest tests, SM_LEVEL_1, SM_LEVEL_2=all tests, 
                                    //      SM_LEVEL_GIVEN = run tests in order requested in pTestRequests 
                                    //      default:[SM_LEVEL_0] 
  SmAssertWalking    eWalkTree,     // in : SM_WALK = run AssertValid on any topology graph descendants, SM_NO_WALK=don't, default:[SM_WALK] 
  SmTArray<ULONG>  * pTestRequests) // in : when eTestLevel == SM_LEVEL_GIVEN, run these tests in this order, default:[NULL]
 const
{
  // init rtn value
  SmBoolean bRtn = TRUE;
  
  // call the base class AssertValid
  bRtn &= (  (eTestLevel != SM_LEVEL_GIVEN)
           ? SmTopology::AssertValid(pAList, eTestLevel, SM_NO_WALK, pTestRequests) 
           : TRUE ) ; 

  /*  0 */ // Loop and the objects it attaches to need to share common contexts
  if(m_pLU)  
    { 
      bRtn &= SM_ASSERT_BOOLEAN_REPORT(0, SM_LEVEL_0, (GetContext() == m_pLU->GetContext() ), _T("") ) ;

      bRtn &= SM_ASSERT_BOOLEAN_REPORT(2, SM_LEVEL_0, (TRUE == GetLoopuse()->IsLoopuseSideInSurface()), _T("") ) ;
    }

  // Check Loop closure.
  SmFace *pFace = this->GetFace();
  if ( pFace != NULL )
    {
      SmSurface *pSurface = pFace->GetSurface();
      if ( pSurface != NULL )
        {
          ULONG lClosure;
          double dTol = pFace->GetTolerance();

          // This adds its own entry to pAList (if present), so we don't:
          SmTrimmingTools::CheckLoop( pSurface, this, dTol, lClosure, pAList, eTestLevel );
        }
    }

  // Check all connected Loopuses
  SmLoopuse *pLoopuse1, *pLoopuse2 ;
  GetLoopuses(pLoopuse1, pLoopuse2) ;

  // Paired loopuse order (former Face 14-16, 18). Copy the single-use-edge offset
  // logic so test 6 only fires when the two EU lists can be uniquely aligned.
  if( pLoopuse1 && pLoopuse2 )
    {
      SmTArray< SmEdgeuse* > sEdgeuses1;
      SmTArray< SmEdgeuse* > sEdgeuses2;
      pLoopuse1->GetEdgeuses( sEdgeuses1 ); // returns the edgeuses in CCW order.
      pLoopuse2->GetEdgeuses( sEdgeuses2 );

      // both loop->loopuses must connect to the same number of loop->edges
      bRtn &= SM_ASSERT_BOOLEAN_REPORT(5, SM_LEVEL_0, (sEdgeuses1.GetSize() == sEdgeuses2.GetSize()), _T("")) ;

      // Face->EdgeLoopuse must have at least 1 edgeuse and Face->VertexLoopuses must have no edgesuses
      bRtn &= SM_ASSERT_BOOLEAN_REPORT(3, SM_LEVEL_0, (   (   pLoopuse1->IsEdgeLoopuse()   && sEdgeuses1.GetSize() >  0)
                                                        || (   pLoopuse1->IsVertexLoopuse() && sEdgeuses1.GetSize() == 0) ),
                                                       _T("")) ;

      // don't bother checking single-edge loops and if this gets called
      // while face is under construction - the edgeuse count is allowed to be zero.
      if (sEdgeuses1.GetSize() >= 1)
        {
          ULONG idx1, idx2;

          // It's ok for the loopuses to start on different edges.
          // Look for the common edge and compare the orders from there.
          // Only use edges that are in the loop one time to avoid mismatches
          // Some checks only execute on loops that have at least one edge that is used just one time.
          SmBoolean bHasSingleUseEdge = FALSE ;
          for(idx1=0;idx1<sEdgeuses1.GetSize();idx1++)
            {
              ULONG lCnt = 0 ;
              for(idx2=0;idx2<sEdgeuses1.GetSize();idx2++)
                {
                  if(sEdgeuses1[idx1]->GetEdge() == sEdgeuses1[idx2]->GetEdge())
                    { lCnt++ ; }
                }

              // when we found an edge only used once in the array
              if(lCnt == 1)
                { bHasSingleUseEdge = TRUE ;
                  break ;
                }
            } // end iter idx1 looking for an edge used just once in the loop

          // get SingleUseEdge if available - else just get the 1st edge
          SmEdge *pEdge1 = sEdgeuses1[bHasSingleUseEdge ? idx1 : 0]->GetEdge() ;
          SmEdge *pEdge2 = NULL;

          // find the index in sEdgeuse2 of the edgeuse pointing to the same edge
          for(idx2=0;idx2<sEdgeuses2.GetSize();idx2++)
            {
              pEdge2 = sEdgeuses2[idx2]->GetEdge() ;

              // see if we found the common edgeuse
              if(pEdge1 == pEdge2)
                { break ; }

            } // end iter looking for common edge

          // Every loop->Loopuse->Edge must by used by both loop->loopuses
          SmTArray<SmEdge*> sEdges1, sEdges2;
          pLoopuse1->GetEdges(sEdges1);
          pLoopuse2->GetEdges(sEdges2);
          sEdges1.RemoveElements(sEdges2, sEdges1);
          bRtn &= SM_ASSERT_BOOLEAN_REPORT(4, SM_LEVEL_0, sEdges1.GetSize() == 0, _T("")) ;

          // Single-edge loops have no remaining order to compare (former Face 18).
          if ( sEdgeuses1.GetSize() > 1 )
            {
              // compute the offset from sEdgeuses1[0] to sEdgeuse2[off]
              //  knowing that sEdgeuses1[idx1]->GetEdge() == sEdgeuses2[idx2]->GetEdge()
              //  This offset is valid when bHasSingleUseEdge == TRUE
              ULONG off = (idx1 + idx2) % sEdgeuses1.GetSize() ;

              // for every edgeuse
              for ( idx1 = 1; idx1 < sEdgeuses1.GetSize(); idx1++ )
                {
                  // skip this test when bHasSingleUseEdge == FALSE because we may not have matched up the edgeuses in the proper order
                  // The order of edgeuses in paired face->loopuses must match
                  idx2 = (sEdgeuses1.GetSize()-idx1+off)%sEdgeuses1.GetSize() ;
                  if(bHasSingleUseEdge == TRUE)
                    {
                      bRtn &= SM_ASSERT_BOOLEAN_REPORT(6, SM_LEVEL_0, (sEdgeuses1[idx1]->GetEdge() == sEdgeuses2[idx2]->GetEdge()), _T("")) ;
                    }

                }  // end iter every sEdgeuses1 in Loop
            }
        }
    }

  if(pLoopuse1) { bRtn &= pLoopuse1->AssertValid(pAList, eTestLevel, SM_NO_WALK) ; }
  if(pLoopuse2) { bRtn &= pLoopuse2->AssertValid(pAList, eTestLevel, SM_NO_WALK) ; }

  /*  1 */ // when asked - AssertValid for a topology graph traversal
  if(eWalkTree == SM_WALK)
    {
      bRtn &= SmBrep::AssertSubTopology( (SmTopology &)*this, pAList, eTestLevel) ;
      // bRtn &= SM_ASSERT_BOOLEAN_REPORT(1, SM_LEVEL_0, (SmBrep::AssertSubTopology( (SmTopology &)*this, pAList, eTestLevel)), _T("")) ;

    } // end asked to walk the tree check

  // all done
  // SM_ASSERT(bRtn) ;
  return(bRtn) ;

} // end SmLoop::AssertValid

// obsolete
// /*******************************************************************//**
// PURPOSE: Fix Assert Report failures reported by an AssertValid call
// 
// RETURNS ---  TRUE  = OK
//              FALSE = Problem
// ***********************************************************************/
// SmBoolean SmLoop::AssertHeal
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
//       // Could be a TrimmingTools report.
//       if ( rAReport.m_lReportingType == SmTrimmingTools::GetClassType() )
//       {
//           SmObject *pObj = ( SmObject* ) rAReport.m_pOwner;
//           SmLoop *pLoop = SM_CAST_PTR( SmLoop, pObj );
//           if ( pLoop == NULL )
//             { return FALSE; }
// 
//           return pLoop->RebuildFromEdgeCurves();
//       }
// 
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
//                rAReport.m_pHealMessage = _T("SmLoop::AssertHeal fix not yet supported") ;  
//                 
//     }
// 
//   // all done
//   return(bRtn) ;
// 
// } // end SmLoop::AssertHeal
// end obsolete

/*******************************************************************//**
PURPOSE:
NOTES:
***********************************************************************/
SmBoolean SmLoop::IsKindOf( SM_TYPE t ) const
{
  return ((SmLoop_TYPE == t) ? TRUE : SmTopology::IsKindOf( (t) ));
}

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
void SmLoop::Dump() const
{
  // pass the call along
  Dump(FALSE) ;

} // end SmLoop::Dump

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
void SmLoop::Dump
 (SmBoolean bAbbrev) // in : FALSE=Also Dump UVTrimCurves;Dump Edges and Surface(bAbbrev=FALSE), TRUE=don't, default:[FALSE]
 const
{
  TCHAR sBuff[SM_TBLOCK_SIZE], sBuffForFile[SM_TBLOCK_SIZE] ;

  smos_WriteBuffer(_T("\nBegin SmLoop::Dump()\n")) ;

  smos_sprintf( sBuff, _T("SmLoop object [0x%p]:"), this );
  smos_sprintf( sBuffForFile, _T("SmLoop object [%s]:"), _T("NotNULL"));
  smos_WriteBuffer( sBuff, sBuffForFile );

  SmFace *pFace = GetFace();

  smos_sprintf( sBuff, _T(" Face:[0x%p], "), pFace );
  smos_sprintf( sBuffForFile, _T(" Face:[%s], "), pFace ? _T("NotNULL") : _T("NULL"));
  smos_WriteBuffer( sBuff, sBuffForFile );

  SmTArray < SmEdge*> sEdges;
  GetEdges(sEdges); 
  ULONG lNumEdges = sEdges.GetSize();

  if(lNumEdges > 0)
    {
      smos_sprintf( sBuff, _T("%lu Edges in "), lNumEdges );
      smos_WriteBuffer( sBuff );

      if(m_pLU->GetOrientation() == SM_OT_SAME) smos_sprintf( sBuff, _T("%s"), _T("Outer Loop, Edgeuse Orients:[ ") );
      else                                      smos_sprintf( sBuff, _T("%s"), _T("Inner Loop, Edgeuse Orients:[ ") );
      smos_WriteBuffer( sBuff );

      // orientations of primary loopuse->edgeuses
      SmEdgeuse *pEdgeuse = (SmEdgeuse *)m_pLU->GetEUorVU() ;
      for(ULONG ii=0;ii<lNumEdges;ii++)
        {
          if(pEdgeuse->GetOrientation() == SM_OT_SAME) smos_sprintf( sBuff, _T("%s"), _T("SAME ") );
          else                                         smos_sprintf( sBuff, _T("%s"), _T("OPPOSITE ") );
          smos_WriteBuffer( sBuff );
          pEdgeuse = pEdgeuse->GetCCWEdgeuse() ;
        }
      smos_sprintf( sBuff, _T("%s"), _T("]") );
      smos_WriteBuffer( sBuff );
    }
  else
    {
      smos_sprintf( sBuff, _T("%s"), _T(" 1 Vertex") );
      smos_WriteBuffer( sBuff );
    }
  
#ifdef SM_DEBUG_CODE // draw Brep, Face, and edges of loop - Dump UVTrimCurves
SmBoolean bDebugMe = FALSE;
  if(bDebugMe)
    {
      ULONG ii;
      SmSurface *pSurface = pFace ? pFace->GetSurface() : NULL;

      // smgfx_Erase();
        smgfx_SetLook(2, 4, 0, 0, 1); 
        if (pSurface)
            pSurface->DrawUV(1, 1);
        sm_GraphicsLoop();
      for(ii=0;ii<sEdges.GetSize();ii++)
        {
            SmEdge  *pEdge  = sEdges[ii];
          SmCurve *pCurve = pEdge->GetCurve();
          SmExtent1d sIvl = pEdge->GetInterval();
          smgfx_ChangeColor(ii != 0);
            if (pSurface != NULL)
            {
              SmBSplineCurve *pUVTrimCurve = pEdge->GetUVTrimCurveOfSurface(pSurface);
                pUVTrimCurve->Dump(bAbbrev);
              // Draw trim curve in space (in xy plane).
                smgfx_SetLineWidth(1);
                pUVTrimCurve->DrawParams(&sIvl);
                sm_GraphicsLoop();
          }
          // Same color as uv curve:
            smgfx_SetLineWidth(4);
            pCurve->DrawParams(&sIvl);
            sm_GraphicsLoop();
          // smgfx_SetLook(6,8, 1,0,0); pEdge->Draw(); sm_GraphicsLoop();
          sm_GraphicsLoop();
        }
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

  if (!bAbbrev) 
    {
      smos_sprintf( sBuff,        _T("Primary Loopuse:[0x%p], Mate Loopuse:[0x%p]\nPrimary LU Dump:"), m_pLU, m_pLU->GetOtherLoopuse() );
      smos_sprintf( sBuffForFile, _T("Primary Loopuse:[%s], Mate Loopuse:[%s]\nPrimary LU Dump:"), 
                  m_pLU ? _T("NotNULL") : _T("NULL"), 
                  m_pLU->GetOtherLoopuse() ? _T("NotNULL") : _T("NULL"));
      smos_WriteBuffer( sBuff, sBuffForFile );


      m_pLU->Dump( );
    }

  smos_WriteBuffer(_T("End SmLoop::Dump()")) ;

  return;
} // end SmLoop::Dump( bAbbrev )
