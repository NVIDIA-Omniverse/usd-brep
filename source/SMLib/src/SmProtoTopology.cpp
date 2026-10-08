// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************
* FILE NAME --- SmProto.cpp
* PURPOSE: Implementation of Proto Topology classes
**********************************************************************/

#include "StdAfx.h"

#include <SmBrep.h>
#include <SmTopology.h>
#include <SmFace.h>
#include <SmEdge.h>
#include <SmVertex.h>
#include <SmCurveClass.h>
#include <SmTopologySolver.h>
#include <SmTopologyIntersector.h>
#include <SmBrepCache.h>
#include <SmSolutionArray.h>
#include <SmAssertArray.h>

#include <SmProtoTopology.h>

/* Tolerance considerations:
- tol stored in SmTopologySolver: I'm pretty sure it adds that tol to the
  Topology's tol.  But sometimes it appears to ignore the topology tol,
  e.g. a Vertex solver will just call the Point solver and ignore the Vtx's tol.
  -- Shouldn't the value in SmTopologySolver be an override tol?

- What about tols stored in SmPointClass / SmCurveClass?
  - They've already been processed for tols.
  PC's have both SrcZoneTol and ObjZoneTol: Src is stored val, Obj is what's on the obj.
  - in PC: Passed to srf::GlobalPtSolve w/ INTERSECT: so max dist.

- Probably don't have to pass tols all around...
 */

#ifdef SM_DEBUG_CODE
static constexpr int cbiDumpLevel = 3; // minimal.
#endif

///////////////////////////////////////////
// static helper methods
///////////////////////////////////////////

/*******************************************************************//**
PURPOSE: Static helper: Return whether a PointClassificationType can be imprinted.

USAGE NOTES---- 
***********************************************************************/
static SmBoolean sm_CanImprint( SmPointClassificationType eType )
{
  return(   eType == SM_PC_REGION
         || eType == SM_PC_FACE
         || eType == SM_PC_EDGE
         || eType == SM_PC_VERTEX) ;

} // end sm_CanImprint

/*******************************************************************//**
PURPOSE: Static helper: return the dimensionality of the given SmTopology object.

USAGE NOTES---- 
***********************************************************************/
static int sm_TopoDimension( SmTopology *pTopo )
{
  return(  pTopo->IsKindOf(SmVertex_TYPE) ? 0   // vertex
         : pTopo->IsKindOf(SmEdge_TYPE  ) ? 1   // edge
         : pTopo->IsKindOf(SmFace_TYPE  ) ? 2   // face
         : -1 ) ;                                 // error

} // end sm_TopoDimension

/*******************************************************************//**
PURPOSE: Get AvgPosition and Deviation of two SmPointClass classifications.

USAGE NOTES---- 
   What makes this nontrivial is that PCs that classify to Faces
   do not usually have their uv values set.  But given a pair
   (from corresponding CurveClassifications from a Face/Face
   intersection), at least one should be ok.
***********************************************************************/
static SmStatus sm_FindPositionFromPCs
 (SmPointClassification & rPC1,          // in : Point Classification 1
  SmPointClassification & rPC2,          // in : Point Classification 2
  SmPoint3d             & rPosition,          // out: Avg of Surf1(UV1) and Surf2(UV2)
  double                & rdDeviation )  // out: Dist between Surf1(UV1) and Surf2(UV2)
{
  SmPoint3d sPt1, sPt2;

  // These should have been set in the SmVertexDefinition constructor.
  // More information was available at that point, so if that failed, we can't improve on it.
  SmBoolean bFoundPos1 = rPC1.AreParametersSet() ;
  SmBoolean bFoundPos2 = rPC2.AreParametersSet() ;

  // error - both Parameters are not Set
  if(!bFoundPos1 && !bFoundPos2 )
    { SER( SM_ERR) ; }

  if(bFoundPos1) { SER( rPC1.FindObjPoint3d( sPt1 )); }
  if(bFoundPos2) { SER( rPC2.FindObjPoint3d( sPt2 )); }

  // locals
  SmXSectTol3d dXSectTol3d = SmTol::GetXSectTol3d( rPC1.GetObject(), rPC2.GetObject()) ;

  // deviation = dist between PtClassifyObjects when both point classifies have ParametersSet
  //             else 0.0
  rdDeviation = (bFoundPos1 && bFoundPos2) ? sPt1.DistanceBetween( sPt2 ) : 0.0;

  // inform the public when PCGap exceeds XSectTol3d
  if(rdDeviation > dXSectTol3d)
    { 
      TCHAR sBuff[SM_TBLOCK_SIZE];
      smos_sprintf(sBuff, _T(" PointClassifications in disagreement about where to place a VertexDefinition \n\t Deviation [%16.16lf] Tolerance [%16.16lf] "), (double) rdDeviation, (double) dXSectTol3d) ; 
      SE_MSG( SM_ERR, sBuff) ;
    }

  // No:  rPosition = ( bFoundPos1 && bFoundPos2 ) ? 0.5 * ( sPt1 + sPt2 ) : ( bFoundPos1 ) ? sPt1 : sPt2;
  // Not just average: give lower-dimensional topology more weight.
  if(! bFoundPos1 )  // found sPt2 only
    {
      // See if we can find a position for rPC1 from sPt2.
      SmStatus eStat = rPC1.ComputePointParameters( sPt2) ;
      if(eStat == SM_SUCCESS  &&  rPC1.AreParametersSet() )
        {
          SER( rPC1.FindObjPoint3d( sPt1 ));
          bFoundPos1 = TRUE;
        }
      else
        {
          rPosition = sPt2;
        }
    } // end missing PC1 Parameters branch

  else if(! bFoundPos2 )  // found sPt1 only
    {
      // See if we can find a position for rPC2 from sPt1.
      SmStatus eStat = rPC2.ComputePointParameters( sPt1) ;
      if(eStat == SM_SUCCESS  &&  rPC2.AreParametersSet() )
        {
          SER( rPC2.FindObjPoint3d( sPt2 ));
          bFoundPos2 = TRUE;
        }
      else
        {
          rPosition = sPt1;
        }
    } // end missing PC2 Parameters branch

  // when both PCs have Parameters
  if(bFoundPos1 && bFoundPos2 )  // found both
    {
      SmTopology *pTopo1 = SM_CAST_PTR( SmTopology, rPC1.GetObject()) ;
      int iDim1 = ( pTopo1 != NULL ) ? sm_TopoDimension( pTopo1 ) : 3;  //  high dimension means low weight.
      SmTopology *pTopo2 = SM_CAST_PTR( SmTopology, rPC2.GetObject()) ;
      int iDim2 = ( pTopo2 != NULL ) ? sm_TopoDimension( pTopo2 ) : 3;  //  high dimension means low weight.

      // Give more weight to the one with lower dimension.
      if(iDim1 < iDim2 )
        { rPosition = sPt1; }
      else if(iDim2 < iDim1 )
        { rPosition = sPt2; }
      else
        {
          // Same dimension.  Give more weight to the one with the tighter gap.
          double dGap1 = rPC1.GetGap3d();
          double dGap2 = rPC2.GetGap3d();
          double dSum  = dGap1 + dGap2;
          double dFrac = ( dSum > SM_EFF_ZERO ) ? ( dGap1 / dSum ) : 0.5;
          rPosition = ( 1-dFrac ) * sPt1  +  dFrac * sPt2;
        }
    } // end both PCs have parameters check

  // all done
  return SM_SUCCESS;

} // end static sm_FindPositionFromPCs

/*******************************************************************//**
PURPOSE: Static helper: Classify a uv position against two Faces.

USAGE NOTES---- 
   returns:
    1: on pFace1.
    2: on pFace2.
    0: Couldn't determine: possibly in both.
***********************************************************************/
static int sm_ClassifyParamOnFaces    // rtn: Pt Classify value: 1 = On pFace1, 2 = On pFace2, 0 = not classifable (could be on both Faces)
 (const SmPoint2d  & sFaceUVParam,    // in : Target UVParam to classify
  SmZoneTol3d        sMySrcZoneTol3d, // in : obj ZoneTol3d assoc with UVParam, usually not this face
                                      //      if none, use:SmTol::GetZoneTol3d(BREP_CONTEXT_OR_NULL)
  SmFace           * pFace1,          // in : 1st TgtFace to check for pt containment
  const SmExtent2d & sDomain1,        // in : TgtFace1 UV Domain
  SmFace           * pFace2,          // in : 2nd TgtFace to check for pt containment
  const SmExtent2d & sDomain2)        // in : TgtFace2 UV Domain
{
  // quick BBox containment check
  SmBoolean bInFace1 = sDomain1.ContainsPoint2d( sFaceUVParam, SM_EFF_ZERO) ; //cbiTol: need uv-space tol.
  SmBoolean bInFace2 = sDomain2.ContainsPoint2d( sFaceUVParam, SM_EFF_ZERO) ;

  // low work case - TgtUVParam is in only one TgtFace Domain BBox
  if     (bInFace1==TRUE  && bInFace2==FALSE) { return 1 ; }
  else if(bInFace1==FALSE && bInFace2==TRUE ) { return 2 ; }

  // arrive here when UVPoint has to be classified against the Faces' boundaries

  // ZoneTol3d. When input ZoneTol3d is zero - set ZoneTol3d = Max(pFace1->ZoneTol3d, pFace2->ZoneTol3d)
  SmZoneTol3d sSrcZoneTol3d = ( sMySrcZoneTol3d > SM_EFF_ZERO ) ? sMySrcZoneTol3d
                                                                : SmTol::GetZoneTol3d_ForXSectResult(pFace1, pFace2);

  // classify sFaceUVParam against pFace1
  SmPointClassification ePC1(sSrcZoneTol3d, pFace1->GetContext()) ;
  pFace1->PointClassify(sFaceUVParam,  // in : UVPoint to Classify 
                        sSrcZoneTol3d, // NotUsed: in : Obj ZoneTol3d assoc with UVPoint, not this face  
                                       //               (if none, use: SmTol::GetZoneTol3d(Brep_Context_Or_NULL))
                        TRUE,          // NotUsed: in : TRUE = return point face->Vertex/Edge intersections when found 
                                       //               FALSE= return loop-containment classifications. 
                        TRUE,          // NotUsed: in : no longer used 
                        ePC1) ;        // out: Classification of the point. 
                                       //      when InFace     : m_ePointClass = SM_PC_FACE 
                                       //           OutOfFace  : m_ePointClass = SM_PC_UNKNOWN 
                                       //           OnFaceBndry: m_ePointClass = SM_PC_EDGE or SM_PC_VERTEX 

  // when UVParam classifies to pFace1 - return 1                                        
  SmPointClassificationType ePC1Type = ePC1.GetPointClass();
  if(ePC1Type == SM_PC_FACE)
    { return 1 ; }

  // classify sFaceUVParam against pFace2
  SmPointClassification ePC2( sSrcZoneTol3d, pFace2->GetContext()) ;
  pFace2->PointClassify(sFaceUVParam,  // in : UVPoint to Classify 
                        sSrcZoneTol3d, // NotUsed: in : Obj ZoneTol3d assoc with UVPoint, not this face  
                                       //               (if none, use: SmTol::GetZoneTol3d(Brep_Context_Or_NULL))
                        TRUE,          // NotUsed: in : TRUE = return point face->Vertex/Edge intersections when found 
                                       //               FALSE= return loop-containment classifications. 
                        TRUE,          // NotUsed: in : no longer used 
                        ePC2) ;        // out: Classification of the point. 
                                       //      when InFace     : m_ePointClass = SM_PC_FACE 
                                       //           OutOfFace  : m_ePointClass = SM_PC_UNKNOWN 
                                       //           OnFaceBndry: m_ePointClass = SM_PC_EDGE or SM_PC_VERTEX 

  // when UVParam classifies to pFace2 - return 2                                        
  SmPointClassificationType ePC2Type = ePC2.GetPointClass();
  if(ePC2Type == SM_PC_FACE)
    { return 2 ; }

  // when UVParam is on both FaceBndrys its probably on the XSectCrve between the faces - no action required
  if(   ePC1Type == SM_PC_EDGE 
     && ePC2Type == SM_PC_EDGE)
    { return 0 ; }

  // arrive here when the UVPoint did not classify cleanly to one face or the other or to the Boundary between them - GWC: probably should have an err return value - for now 0 means no actions
  return 0; // Problem? Breakpoint here.

} // end sm_ClassifyParamOnFaces

/*******************************************************************//**
PURPOSE: Static helper: Check whether two curves are generally the same.

USAGE NOTES----
  This is a quick and inexpensive check assuming that two XSect curves
  from the same set of Boolean intersections are meant to be the same
  when the curves are loosely the same.  As such,

  returns TRUE when two curves and their intervals are the same, or
               or when the DropPoint distance of Crv1->MidPoint to Crv2
                  is less than the given dTol.

  // Here's an example where we know that the end points are the same, but
  // the curves may or may not be identical or nowhere near.
  //   picture a cylinder with two seams: cut it with a plane perpendicular to
  //   its axis, so that the intersection is two semi-circles that 
  //   share common end points but are nowhere else being close to the same.
   
  Checking a midPoint drop distance between two curves will show that the
  results are either meant to be identical or nowhere near.
***********************************************************************/
static SmBoolean sm_AreSameCurve
 (const SmCurve    * pCrv1,   // in : 1st Curve
  const SmExtent1d & crIvl1,  // in : 1st Curve Ivl
  const SmCurve    * pCrv2,   // in : 2nd Curve
  const SmExtent1d & crIvl2,  // in : 2nd Curve Ivl
  double             dTol )   // in : XSectTol3d value to check for coincidence
{
  // low work - same curve.
  if(pCrv1 == pCrv2)
    {
      // return TRUE when intervals are the same
      return(crIvl1 == crIvl2) ;
    }

  // Just a rough check.  If two intersection curves between two Breps have the same starts, 
  // ends, and midpoints, we'll assume they're meant to be the same Edge in the end.

  // locals
  double    dParam1 = crIvl1.GetMid() ;
  double    dGuess  = crIvl2.GetMid();
  double    dParam2 ;
  double    dDist ;
  SmPoint3d sMidPt1;
  SmBoolean bSuccess = FALSE;

  // Crv1 mid point
  pCrv1->EvaluatePoint( dParam1, sMidPt1) ;

  // Drop Crv1 mid point to Crv2 - get drop distance
  pCrv2->DropPoint(crIvl2,            // in : target curve allowed domain
                   sMidPt1,           // in : Point to drop to curve
                   NULL,              // in : For Snapping, NULL=NoSnap, Vec pointing to CrvInside from desired EndPt for snapping.
                                      //      when DropPt is within dDistTol in cpVecToCrvInside dir of EndPt, Snap to Ivl EndPt.
                                      //      Vec handles ambiguities and makes sure that curves are not just touching at the ends.
                   dTol,              // in : Used for SnapDist, and SM_SO_INTERSECT sXSectTol3d distance.
                                      //      sXSectTol3d = dDistTol ;      SM_SO_INTERSECT
                                      //      sXSectTol3d = SM_BIG_DOUBLE ; SM_SO_MINIMIZE, SM_SO_NORMALIZE
                                      //      sSnapTol3d  = dDistTol ;      SM_SO_MINIMIZE, SM_SO_NORMALIZE, SM_SO_INTERSECT
                   &dGuess,           // in : If given, call cheap LocalPointSolve() else call expensive GlobalPointSolve()
                   bSuccess,          // out: TRUE = found a drop point
                   dParam2,           // out: found drop curve param
                   dDist,             // out: found drop distance
                   SM_SO_INTERSECT) ; // in : SM_SO_MINIMIZE = allow nonNormal drops near endPoints
                                      //      SM_SO_INTERSECT= like Minimize but point must be within dDistTol
                                      //      SM_SO_NORMALIZE= exclude nonNormal drops near endPoints
                                      //      default:[SM_SO_MINIMIZE] to preserve original behavior

  // return TRUE when DropPoint() succeeded and DropDist < dTol                                       
  return ( bSuccess && dDist < dTol) ;

} // end sm_AreSameCurve

/*******************************************************************//**
PURPOSE: static helper: Determine whether a Vertex and a Face are connected.

USAGE NOTES----  gwc note: why not use the SmVertex::IsConnectTo() method?
***********************************************************************/
static SmBoolean sm_AreConnected( const SmVertex *pVtx, const SmFace *pFace )
{
  SmTArray< SmVertexuse* > sVUs;
  pVtx->GetVertexuses( sVUs) ;
  SmVertexuse *pVU=NULL;
  ULONG ii, lNumVUs = sVUs.GetSize();
  for(ii=0;ii<lNumVUs;ii++)
  {
      pVU = sVUs[ii];
      SmFaceuse *pFU = pVU->GetFaceuse();
      if(pFU )
      {
          SmFace *pF = pFU->GetFace();
          if(pF == pFace )
            { return TRUE; }
      }
  }

  return FALSE;

} // end sm_AreConnected( Vertex, Face )

/*******************************************************************//**
PURPOSE: static helper: Determine whether an Edge and a Face are connected.

USAGE NOTES----  gwc note: why not use the SmEdge::IsConnectTo() method?
***********************************************************************/
static SmBoolean sm_AreConnected( const SmEdge *pEdge, const SmFace *pFace )
{
  SmTArray< SmEdgeuse* > sEUs;
  pEdge->GetEdgeuses( sEUs) ;
  SmEdgeuse *pEU;
  ULONG ii, lNumEUs = sEUs.GetSize();
  for(ii=0;ii<lNumEUs;ii++)
  {
      pEU = sEUs[ii];
      if(pEU->GetFace() == pFace )
        { return TRUE; }
  }

  return FALSE;

} // end sm_AreConnected( Edge, Face )

/*******************************************************************//**
PURPOSE: static helper: Determine whether a Vertex and an Edge are connected.

USAGE NOTES----      gwc note: why not use the SmVertex::IsConnectTo() method?
***********************************************************************/
static SmBoolean sm_AreConnected( const SmVertex *pVertex, const SmEdge *pEdge )
{
  SmVertex *pVtx = pEdge->GetStartVertex();
  if(pVtx == pVertex )
    { return TRUE; }

  pVtx = pEdge->GetOtherVertex( pVtx) ;
  if(pVtx == pVertex )
    { return TRUE; }

  return FALSE;

} // end sm_AreConnected( Vertex, Edge )

/*******************************************************************//**
PURPOSE: static helper: Determine whether two topology objects are connected.

USAGE NOTES---- connected object pairs:
  Face - EdgeBoundary
  Face - VertexBoundary
  Edge - VertexBoundary

  gwc note: why not use the SmTopology::IsConnectTo() method?
***********************************************************************/
static SmBoolean sm_AreConnected // rtn: TRUE = Topos are connected neighbors in the same Brep Topology Graph
 (const SmTopology * pTopo1,     // in : TgtTopology1 o
  const SmTopology * pTopo2)     // in : TgtTopology2 o
{
  if(pTopo1 == pTopo2 )
    { return TRUE; }

  const SmFace *pFace1 = dynamic_cast<const SmFace*> ( pTopo1) ;
  if(pFace1 != NULL )
  {
      const SmEdge *pEdge2 = dynamic_cast<const SmEdge*> ( pTopo2) ;
      if(pEdge2 != NULL )
        { return sm_AreConnected( pEdge2, pFace1) ; }

      const SmVertex *pVertex2 = dynamic_cast<const SmVertex*> ( pTopo2) ;
      if(pVertex2 != NULL )
        { return sm_AreConnected( pVertex2, pFace1) ; }
  }

  const SmEdge *pEdge1 = dynamic_cast<const SmEdge*> (pTopo1) ;

  if(pEdge1 != NULL )
  {
      const SmFace *pFace2 = dynamic_cast<const SmFace*> ( pTopo2) ;
      if(pFace2 != NULL )
        { return sm_AreConnected( pEdge1, pFace2) ; }

      const SmVertex *pVertex2 = dynamic_cast<const SmVertex*> ( pTopo2) ;
      if(pVertex2 != NULL )
        { return sm_AreConnected( pVertex2, pEdge1) ; }
  }

  const SmVertex *pVertex1 = dynamic_cast<const SmVertex*> ( pTopo1) ;

  if(pVertex1 != NULL )
  {
      const SmFace *pFace2 =  dynamic_cast<const SmFace*> ( pTopo2) ;
      if(pFace2 != NULL )
        { return sm_AreConnected( pVertex1, pFace2) ; }

      const SmEdge *pEdge2 =  dynamic_cast<const SmEdge*> ( pTopo2) ;
      if(pEdge2 != NULL )
        { return sm_AreConnected( pVertex1, pEdge2) ; }
  }

  // I've checked, these are ok.
  // WARN( _T("SmProto: sm_AreConnected() called on unexpected topology: should be investigated." ));
  return FALSE;

} // end sm_AreConnected

/*******************************************************************//**
PURPOSE: static helper for SplitIntersectingProtoEdges() 
   return either the input PV that split this edge, 
              or the PE->EndPV that's within tol of the PE->SplitPt, 
              or NULL for none 
   to be considered for combining with existing PVs

USAGE NOTES---- 
   If the ProtoEdge was split, then pSplitPV is non Null: use it always.
   Otherwise return the original ProtoEdge's within tol start or end ProtoVertex 
   or NULL for none.
***********************************************************************/
static SmProtoVertex* sm_WhichPVToCombine
 (SmProtoEdge   * pOrigPE,     // in : Tgt PE being split
  double          dSplitParam, // in : Param of Split point
  SmProtoVertex * pSplitPV )   // in : If the ProtoEdge was split, then pSplitPV is non Null
                               //      NULL on input
{
  // when ProtoEdge was split, then pSplitPV is non Null - return it
  if(pSplitPV != NULL )
    { return pSplitPV ; }

  // pOrigPE was not split at dSplitParam.
  // That should mean that dSplitParam is one end of the PE's domain,
  // but there have been examples where it's outside the domain.
  // Check proximity to the start and end PVs.

  // locals
  SmPoint3d       sSplitPt ;
  SmProtoVertex * sStartPV = pOrigPE->GetStartProtoVertex() ;
  SmProtoVertex * sEndPV   = pOrigPE->GetEndProtoVertex() ;

  // Eval PE at SplitParam
  pOrigPE->Evaluate( dSplitParam, &sSplitPt) ;

  // return matching End PV or NULL for none
  if     (sStartPV->ContainsPoint(sSplitPt)) { return sStartPV ; }
  else if(sEndPV  ->ContainsPoint(sSplitPt)) { return sEndPV ; }
  else                                       { return NULL ; }

} // end static sm_WhichPVToCombine

#ifdef SM_DEBUG_CODE
/*******************************************************************//**
PURPOSE: static helper: Draw the Edgeuses of a SmVertexDefinition

USAGE NOTES---- 
***********************************************************************/
static void sm_DrawEUsOfVtxDef
 (const SmVertexDefinition * pVD, 
  int                        iWhichBrep )
{
  SmTopology *pTopo = pVD->GetSrcTopo( iWhichBrep) ;
  SmVertex *pVtx = SM_CAST_PTR( SmVertex, pTopo) ;
  if(pVtx==NULL ) { return; }

  SmTArray< SmVertexuse* > sVUs;
  pVtx->GetVertexuses( sVUs) ;
  ULONG ii, lNumVUs = sVUs.GetSize();
  for(ii=0;ii<lNumVUs;ii++)
  {
      SmVertexuse *pVU = sVUs[ii];
      if(! pVU->IsEdgeVertexuse() )
        { continue; }
      SmEdgeuse   *pEU = pVU->GetEdgeuse();
      if(pEU==NULL ) { continue; }
      pEU->Draw(); sm_GraphicsLoop();
      sm_GraphicsLoop();
  }
  sm_GraphicsLoop();

  return;

} // end sm_DrawEUsOfVtxDef

/*******************************************************************//**
PURPOSE: static helper: Draw an SmTopology object that is an SmVertex, SmEdge, or SmFace.

USAGE NOTES---- 
***********************************************************************/
static void sm_DrawTopology
 (SmTopology *pTopo,      // in : TgtTopology to draw
  double dLineWidth=1.0,  // in : linewidth
  double dPointSize=2.0,  // in : PointSize
  double dRed      =0.0,  // in : Red
  double dGreen    =0.0,  // in : Green
  double dBlue     =1.0)  // in : Blue
{
  // locals
  SmVertex * pVert = SM_CAST_PTR( SmVertex, pTopo) ;
  SmEdge   * pEdge = SM_CAST_PTR( SmEdge, pTopo) ;
  SmFace   * pFace = SM_CAST_PTR( SmFace, pTopo) ;

  // pick look
  smgfx_SetLook( dLineWidth, dPointSize, dRed, dGreen, dBlue) ;

  // Draw object
  if(pVert != NULL) { pVert->Draw(); }
  if(pEdge != NULL) { pEdge->Draw(); }
  if(pFace != NULL) { pFace->Draw( SM_DM_CROSSHATCH, 3, 4) ; }

} // end sm_DrawTopology
#endif // if SM_DEBUG_CODE

///////////////////////////////////////////
// SmVertexDefinition methods
///////////////////////////////////////////

/*******************************************************************//**
PURPOSE: SmVertexDefinition constructor

USAGE NOTES---- 
***********************************************************************/
SmVertexDefinition::SmVertexDefinition
 (const SmContext    * pCtx,
  const SmPointClass * cpPC1,
  const SmPointClass * cpPC2,
  const SmPoint3d    & rPosition,
        double         dTol,
        double         dGap,
        SmProtoVertex *pOwner )
    : m_sPC1( dTol, pCtx ),
      m_sPC2( dTol, pCtx ),
      m_sPosition( rPosition ),
      m_dTol( dTol ),
      m_dGap( dGap ),
      m_pMyProtoVertex( pOwner ),
      m_pFinalVtx1( NULL ),
      m_pFinalVtx2( NULL )
{
  SetContext( pCtx) ;

  m_sPC1 = *cpPC1;
  m_sPC2 = *cpPC2;

  if(! m_sPC1.AreParametersSet() )
    { m_sPC1.ComputePointParameters( rPosition) ; }
  if(! m_sPC2.AreParametersSet() )
    { m_sPC2.ComputePointParameters( rPosition) ; }

  if(dGap <= 0.0 )
    {
      SmPoint3d sPt1, sPt2;
      m_sPC1.FindObjPoint3d( sPt1) ;
      m_sPC2.FindObjPoint3d( sPt2) ;
      m_dGap = sPt1.DistanceBetween( sPt2) ;
      m_dGap = smos_Max( m_dGap, cpPC1->GetGap3d()) ;
      m_dGap = smos_Max( m_dGap, cpPC2->GetGap3d()) ;
    }

} // end SmVertexDefinition constructor

/*******************************************************************//**
PURPOSE: SmVertexDefinition destructor

USAGE NOTES---- 
***********************************************************************/
SmVertexDefinition::~SmVertexDefinition()
{}

/*******************************************************************//**
PURPOSE: Return the ProtoTopologyManager for this VertexDefinition.

USAGE NOTES---- 
***********************************************************************/
SmProtoTopologyManager * SmVertexDefinition::GetProtoTopologyManager() const
{ return m_pMyProtoVertex->GetProtoTopologyManager(); }

/*******************************************************************//**
PURPOSE: Return the count of ProtoEdges connected to our ProtoVertex.

USAGE NOTES---- 
***********************************************************************/
  ULONG SmVertexDefinition::GetNumProtoEdges() const { return m_pMyProtoVertex->GetNumProtoEdges(); }

/*******************************************************************//**
PURPOSE: Return the Brep1 topology object this vertex definition maps to.

USAGE NOTES---- Not inline: <windows.h> defines GetObject as GetObjectW, which
breaks a header call to it.
***********************************************************************/
SmTopology * SmVertexDefinition::GetBrep1Topo() const
{ return SM_CAST_PTR( SmTopology, m_sPC1.GetObject()) ; }

/*******************************************************************//**
PURPOSE: Return the Brep2 topology object this vertex definition maps to.

USAGE NOTES---- Not inline; see GetBrep1Topo.
***********************************************************************/
SmTopology * SmVertexDefinition::GetBrep2Topo() const
{ return SM_CAST_PTR( SmTopology, m_sPC2.GetObject()) ; }

/*******************************************************************//**
PURPOSE: Set the Brep topology object in the specified Brep.

USAGE NOTES---- 
***********************************************************************/
SmStatus SmVertexDefinition::SetSrcTopo( int iWhichBrep, SmPointClassificationType eType, SmTopology *pTopo )
{
  if(iWhichBrep==1 )
    { m_sPC1.SetClassObject( eType, pTopo) ; }
  else if(iWhichBrep==2 )
    { m_sPC2.SetClassObject( eType, pTopo) ; }
  else
    { SER( SM_ERR) ; }

  return SM_SUCCESS;

} // end SmVertexDefinition::SetSrcTopo

/*******************************************************************//**
PURPOSE: Set the Brep geometry parameters in the PointClass objects.

USAGE NOTES---- 
***********************************************************************/
SmStatus SmVertexDefinition::SetPointParameters()
{
  m_sPC1.ComputePointParameters( m_sPosition) ;
  m_sPC2.ComputePointParameters( m_sPosition) ;
  return SM_SUCCESS;

} // end SmVertexDefinition::SetPointParameters


/*******************************************************************//**
PURPOSE: Resolve this VertexDefinition.

USAGE NOTES---- 
   The riStatus return argument was used to indicate that this VertexDefinition
   should not be used at all, but that is no longer relevant.  Feel free to use
   it to tell the caller what to do.
***********************************************************************/
SmStatus SmVertexDefinition::Resolve( int & riStatus ) // out: Unused; see Usage notes
{
  riStatus = 0;

//cbiNew: Bug 9: VD's are too far apart: we grabbed bigger than tol to make sure.
// So we'll have to break into two VD's.  Note, don't connect them with a PE.

//cbi No: the overlapping-Edge check also makes sure that those VDs aren't created either.
//cbi   And then the first test in prog_test is a counterexample to this.
//cbi   I'll leave this in until debugging is complete, so that I don't try it again...
//  // The surface intersector can return intersections that start or end on
//  // the interior of coincident Edges.  The overlapping-Edge check makes sure
//  // that those partial coincidences don't create a ProtoEdge between such PVs.
//  // So, if we have a VD whose PV has no PEs attached, if neither Brep entity
//  // is already a Vertex, then delete this VD.
//  if(this->GetNumProtoEdges() == 0 )
//  {
//      SmPointClassificationType eType1 = this->GetSrcTopo_TYPE( 1) ;
//      SmPointClassificationType eType2 = this->GetSrcTopo_TYPE( 2) ;
//      if(eType1 != SmVertex_TYPE  &&  eType2 != SmVertex_TYPE )
//      {
//          riStatus = 1;
//          return SM_SUCCESS;
//      }
//  }

  double dDist1 = m_sPC1.GetGap3d();
  double dDist2 = m_sPC2.GetGap3d();

  SmPoint3d sPt1, sPt2;
  m_sPC1.FindObjPoint3d( sPt1) ;
  m_sPC2.FindObjPoint3d( sPt2) ;

  double dDist = sPt1.DistanceBetween( sPt2) ;

  // Do we really want to combine these into one Vertex?

  // Tol: we have all kinds of things:
  //  this->m_dTol : currently set to just the gap.
  //  (two) PC.GetSrcZoneTol3d() : currently 0.
  //  (two) PC.GetObjZoneTol3d() : from the object(s)
  //  m_pMyProtoVertex:
  //    m_dTol : currently the gap
  //    m_pProtoTopoMgr:
  //      m_dTol  <- this one might be good.
  SmProtoTopologyManager *pProtoTopoMgr = this->GetProtoTopologyManager();
  double dTol = pProtoTopoMgr->GetTolerance();

  if(dDist > dTol )
  {
      // Assume (cbi for now) that dDist1 and dDist2 are based on dist to topology.

      // Put it proportionally closer to the topo object with the smaller gap, but not out of Tolerance
      if(dDist1 + dDist2 > SM_EFF_ZERO )
      {
          // 
          double      dFrac = 0.5;
          SmVector3d  sVec = sPt2 - sPt1;
          double      dVecLength = sVec.Length();
          SmZoneTol3d sTol1 = SmTol::GetZoneTol3d( m_sPC1.GetObject()) ;
          SmZoneTol3d sTol2 = SmTol::GetZoneTol3d( m_sPC2.GetObject()) ;
          double      dFracMin = 1. - sTol2.val / dVecLength;
          double      dFracMax = sTol1.val / dVecLength;
          SmExtent1d  sFracIvl( 0, 1) ;
          if(dFracMax < dFracMin )
          { SM_ASSERT_MSG( dFracMin < dFracMax, _T( "VertexDefinition objects don't intersect." )) ; }
          else
          { sFracIvl.SetMinMax( dFracMin, dFracMax) ; }

          dFrac = dDist1 / ( dDist1 + dDist2) ;
          dFrac = sFracIvl.ClampValue( dFrac) ;
          m_sPosition = sPt1 + dFrac * sVec;
          //m_sPosition = (1-dFrac)*sPt1 + dFrac*sPt2;
      }
  }

  return SM_SUCCESS;

} // end SmVertxDefinition::Resolve()

/*******************************************************************//**
PURPOSE: Imprint this VertexDefinition into both Breps.

USAGE NOTES---- 
   Both Breps' EditingEnabled flags must be set to True.
***********************************************************************/
SmStatus SmVertexDefinition::Imprint()
{
#ifdef SM_DEBUG_CODE
int iDebugLevel = -1 ; // iDumpLevel: -1 = no output
                       //              0 = headers, ProtoVert, Edge, and Face Counts
                       //              5 = List of ProtoVerts, Edges, Faces. Repeate ProtoVert, Edge, and Face counts
                       //             15 = Brep1, Brep2, and SmTopologyIntersector dumps
int iDebugMe = 0 ;
SmProtoTopologyManager * pProtoTopoMgr = m_pMyProtoVertex ? m_pMyProtoVertex->GetProtoTopologyManager() : NULL ; 
SmBrep                 * pBrep1        = pProtoTopoMgr ? pProtoTopoMgr->GetWhichBrep(1) : NULL ; 
SmBrep                 * pBrep2        = pProtoTopoMgr ? pProtoTopoMgr->GetWhichBrep(2) : NULL ;
  if(iDebugMe>0 )  
  {
      this->Dump( cbiDumpLevel) ;
      if(pBrep1) pBrep1->Dump() ;
      if(pBrep2) pBrep2->Dump() ;
      pProtoTopoMgr->Dump(iDebugLevel) ; // iDumpLevel: -1 = no output
                                         //              0 = headers, ProtoVert, Edge, and Face Counts
                                         //              5 = List of ProtoVerts, Edges, Faces. Repeate ProtoVert, Edge, and Face counts
                                         //             10 = PE->PVs, List of PE overlappingPE and CoinPF indices, and PE->EdgeDef dumps
                                         //             15 = Brep1, Brep2, and SmTopologyIntersector dumps
      if(iDebugMe>5 ) {
          smgfx_Erase();
          smgfx_SetLook( 1,2, 0,0,0) ; this->GetProtoTopologyManager()->DrawDebug(); sm_GraphicsLoop();
          sm_GraphicsLoop();
      }

      smgfx_SetLook( 2,4, 1,0,0) ; this->Draw(); sm_GraphicsLoop();
      sm_GraphicsLoop();
  }
#endif // SM_DEBUG_CODE

  SmTArray< SmEdge*   > sNewEdges1;
  SmTArray< SmVertex* > sNewVerts1;

  // If we're on an Edge, save that Edge for checking splits, after merge.
  SmEdge *pThisEdge = m_sPC1.GetEdgeObject();

  // MergeIntoObject() resets the PointClass' Object to the newly-created Vertex
  // (if it creates one).  We store that info in m_pFinalVtx, and it's handy to
  // have the original topology available.  So save it and restore after the call. (DD4)
  SmPointClassificationType ePCType   = m_sPC1.GetPointClass();
  SmObject                * pPCObject = m_sPC1.GetObject();

  SER( m_sPC1.MergeIntoObject( m_sPosition, /* m_dTol, */ &sNewEdges1, &sNewVerts1 ));

  if(sNewVerts1.GetSize() == 1 )
  { SetFinalVertex( 1, sNewVerts1[0]) ; } 
  else if(this->GetSrcTopo_TYPE( 1 ) == SM_PC_VERTEX )
  { SetFinalVertex( 1, m_sPC1.GetVertexObject()) ; } 
  else
    { SER( SM_ERR) ; }

  // Restore the PointClass' original object. (DD4)
  m_sPC1.SetClassObject( ePCType, pPCObject) ;

  // If we split an edge, then one side of the Edge we were on is now
  // a different Edge.  Any ProtoVertices or ProtoEdges on that changed Edge
  // will have to be checked to see whether they're now on the new one, and
  // updated if so.

#ifdef SM_DEBUG_CODE
  if(iDebugMe>0 )
  {
      this->Dump( cbiDumpLevel) ;
      if(pBrep1) pBrep1->Dump() ;
      if(pBrep2) pBrep2->Dump() ;
      pProtoTopoMgr->Dump(iDebugLevel) ; // iDumpLevel: -1 = no output
                                         //              0 = headers, ProtoVert, Edge, and Face Counts
                                         //              5 = List of ProtoVerts, Edges, Faces. Repeate ProtoVert, Edge, and Face counts
                                         //             10 = PE->PVs, List of PE overlappingPE and CoinPF indices, and PE->EdgeDef dumps
                                         //             15 = Brep1, Brep2, and SmTopologyIntersector dumps
      sm_DrawEUsOfVtxDef( this, 1) ;
      sm_GraphicsLoop();
  }
#endif // SM_DEBUG_CODE

  // imprint Vertex int m_pBrep - commonly splits edge
  SER( this->GetProtoTopologyManager()->UpdateEdges( 1, pThisEdge, sNewEdges1, this ));

#ifdef SM_DEBUG_CODE
  if(iDebugMe>0 )
  {
      this->Dump( cbiDumpLevel) ;
      if(pBrep1) pBrep1->Dump() ;
      if(pBrep2) pBrep2->Dump() ;
      pProtoTopoMgr->Dump(iDebugLevel) ; // iDumpLevel: -1 = no output
                                         //              0 = headers, ProtoVert, Edge, and Face Counts
                                         //              5 = List of ProtoVerts, Edges, Faces. Repeate ProtoVert, Edge, and Face counts
                                         //             10 = PE->PVs, List of PE overlappingPE and CoinPF indices, and PE->EdgeDef dumps
                                         //             15 = Brep1, Brep2, and SmTopologyIntersector dumps
      sm_DrawEUsOfVtxDef( this, 1) ;
      sm_GraphicsLoop();
  }
#endif // SM_DEBUG_CODE

  // Repeat for Brep 2.
  pThisEdge = m_sPC2.GetEdgeObject();

  SmTArray< SmEdge*   > sNewEdges2;
  SmTArray< SmVertex* > sNewVerts2;

  // Save the original topology: see above. (DD4)
  ePCType   = m_sPC2.GetPointClass();
  pPCObject = m_sPC2.GetObject();

  SER( m_sPC2.MergeIntoObject( m_sPosition, /* m_dTol, */ &sNewEdges2, &sNewVerts2 ));

  if(sNewVerts2.GetSize() == 1 )
  { SetFinalVertex( 2, sNewVerts2[0]) ; }
  else if(this->GetSrcTopo_TYPE( 2 ) == SM_PC_VERTEX )
  { SetFinalVertex( 2, m_sPC2.GetVertexObject()) ; } 
  else
    { SER( SM_ERR) ; }

  SER( this->GetProtoTopologyManager()->UpdateEdges( 2, pThisEdge, sNewEdges2, this ));

  // Restore the PointClass' original object.
  m_sPC2.SetClassObject( ePCType, pPCObject) ; // DD4

#ifdef SM_DEBUG_CODE
  if(iDebugMe>0 )
  {
      this->Dump( cbiDumpLevel) ;
      if(pBrep1) pBrep1->Dump() ;
      if(pBrep2) pBrep2->Dump() ;
      pProtoTopoMgr->Dump(iDebugLevel) ; // iDumpLevel: -1 = no output
                                         //              0 = headers, ProtoVert, Edge, and Face Counts
                                         //              5 = List of ProtoVerts, Edges, Faces. Repeate ProtoVert, Edge, and Face counts
                                         //             10 = PE->PVs, List of PE overlappingPE and CoinPF indices, and PE->EdgeDef dumps
                                         //             15 = Brep1, Brep2, and SmTopologyIntersector dumps
      sm_DrawEUsOfVtxDef( this, 2) ;
      sm_GraphicsLoop();
  }
#endif // SM_DEBUG_CODE

  return SM_SUCCESS;

} // end SmVertexDefinition::Imprint

///////////////////////////////////////////
// SmEdgeDefinition methods
///////////////////////////////////////////

/*******************************************************************//**
PURPOSE: SmEdgeDefinition destructor

USAGE NOTES---- 
***********************************************************************/
SmEdgeDefinition::~SmEdgeDefinition()
{
  // We have:  m_pXSectCurve3d  m_pXSectUVCurve1  m_pXSectUVCurve2
  // Not deleting these, they get consumed by MakeEdgeInFace().
  // But if they don't, and you have to get rid of them:  Destruct().

  // If this EdgeDefinition was defined on an Edge in either Brep, then our curves
  // were not consumed, by being implanted into the Breps in MakeEdgeInFace(), and so
  // must be deleted.
  SmPointClassificationType eType1 = this->GetSrcTopo_TYPE( 1) ;
  SmPointClassificationType eType2 = this->GetSrcTopo_TYPE( 2) ;

  if(eType1 == SM_PC_EDGE )
    {
      delete m_pXSectCurve3d; m_pXSectCurve3d = NULL;
      delete m_pXSectUVCurve1  ; m_pXSectUVCurve1   = NULL;
    }
  if(eType2 == SM_PC_EDGE )
    {
      delete m_pXSectUVCurve2  ; m_pXSectUVCurve2   = NULL;
    }
} // end SmEdgeDefinition destructor

/*******************************************************************//**
PURPOSE: Delete the geometry in an EdgeDefinition.

USAGE NOTES---- 
   Because the destructor might not delete the geometry,
   this is a separate routine to do that when necessary.
***********************************************************************/
SmStatus SmEdgeDefinition::Destruct()
{
    delete m_pXSectCurve3d ;  m_pXSectCurve3d  = NULL;
    delete m_pXSectUVCurve1 ; m_pXSectUVCurve1 = NULL;
    delete m_pXSectUVCurve2 ; m_pXSectUVCurve2 = NULL;

    return SM_SUCCESS ;

} // end SmEdgeDefinition::Destruct

/*******************************************************************//**
PURPOSE: Get the Brep topology type in the specified Brep.

USAGE NOTES---- 
***********************************************************************/
SmPointClassificationType SmEdgeDefinition::GetSrcTopo_TYPE( int iWhichBrep ) const
{
  //SmTopology *pTopo = ( iWhichBrep==1 ) ? m_pSrcTopo1 : m_pSrcTopo2;
  SM_TYPE eType = ( iWhichBrep == 1 ) ? m_pSrcTopo1_TYPE : m_pSrcTopo2_TYPE;
  switch( eType )
  {
    case SmVertex_TYPE:
      { return SM_PC_VERTEX; }
    case SmEdge_TYPE:
      { return SM_PC_EDGE; }
    case SmFace_TYPE:
      { return SM_PC_FACE; }
    case SmRegion_TYPE:
      { return SM_PC_REGION; }
    default:
      { return SM_PC_UNKNOWN; }
  }

} // end SmEdgeDefinition::GetSrcTopo_TYPE


/*******************************************************************//**
PURPOSE: Get the Start ProtoVertex of this EdgeDefinition.

USAGE NOTES---- 
***********************************************************************/
SmProtoVertex * SmEdgeDefinition::GetStartProtoVertex() const
{ return m_pProtoEdge->GetStartProtoVertex(); }

/*******************************************************************//**
PURPOSE: Get the End ProtoVertex of this EdgeDefinition.

USAGE NOTES---- 
***********************************************************************/
SmProtoVertex * SmEdgeDefinition::GetEndProtoVertex() const
{ return m_pProtoEdge->GetEndProtoVertex(); }

/*******************************************************************//**
PURPOSE: Return the ProtoTopologyManager for this EdgeDefinition.

USAGE NOTES---- 
***********************************************************************/
SmProtoTopologyManager * SmEdgeDefinition::GetProtoTopologyManager() const
{ return m_pProtoEdge->GetProtoTopologyManager(); }

/*******************************************************************//**
PURPOSE: Set the Curve Domain of this EdgeDefinition.

USAGE NOTES---- Accounts for reversal: if dMin > dMax, reverses orientation.
***********************************************************************/
void SmEdgeDefinition::SetXSectInterval( double dMin, double dMax )
{
  if(dMin <= dMax )
    { m_pXSectInterval.SetMinMax( dMin, dMax) ; }
  else
  {
    this->ReverseOrientation();
    m_pXSectInterval.SetMinMax( dMax, dMin) ;
  }
} // end SmEdgeDefinition::SetXSectInterval

/*******************************************************************//**
PURPOSE: Get the location of an end of this EdgeDef bounded by a
         given TgtProtoVertex

USAGE NOTES---- 
   The intput VertexDef should be either our StartPV or EndPV.
   If it is neither, this returns SM_ERR, and rPosition uninitialized.
***********************************************************************/
SmStatus SmEdgeDefinition::GetEndPosition
 (SmProtoVertex * pWhichPV,    // in : TgtProtoVertex
  SmPoint3d     & rPosition )  // out: XSectCurve3d Pos3d for CurveEnd bounded by TgtProtoVertex 
 const
{
  // set output
  rPosition.SetUninitialized();

  // locals
  double dT;
  SmProtoVertex * pStartPV = m_pProtoEdge->GetStartProtoVertex(); // DD2
  SmProtoVertex * pEndPV   = m_pProtoEdge->GetEndProtoVertex();

  // get Param value for tgt ProtoVertex
  if     (pWhichPV == pStartPV) { dT =  m_pXSectInterval.GetMin(); }
  else if(pWhichPV == pEndPV)   { dT =  m_pXSectInterval.GetMax(); }
  else                          { return SM_ERR; }

  // evaluate XSectCurve3d(dT) Pos3d 
  m_pXSectCurve3d->EvaluatePoint( dT, rPosition) ;

  // all done
  return SM_SUCCESS;

}  // end EdgeDefinition::GetEndPosition

/*******************************************************************//**
PURPOSE: Get the location of an end of this EdgeDefinition.

USAGE NOTES---- 
***********************************************************************/
SmStatus SmEdgeDefinition::GetEndPosition
 (SmBoolean   bAtStart,   // in : TRUE = ProtoEdgeStart, FALSE= ProtoEdge->End
  SmPoint3d & rPosition ) // out: ProtoEdge end Pos3d
 const
{
  // set output
  rPosition.SetUninitialized();

  // This has to be more precise, i.e., Start PV == End PV.  [ 090406_ell... ]
  //  if(bAtStart )
  //    { return this->GetEndPosition( m_pProtoEdge->GetStartProtoVertex(), rPosition) ; }
  //  else
  //    { return this->GetEndPosition( m_pProtoEdge->GetEndProtoVertex(), rPosition) ; }

  // pick end param
  double dParam = bAtStart ? m_pXSectInterval.GetMin() 
                           : m_pXSectInterval.GetMax() ;

  // evaluate Pos3d
  m_pXSectCurve3d->EvaluatePoint( dParam, rPosition) ;

  // all done
  return SM_SUCCESS;

} // end SmEdgeDefinition::GetEndPosition

// DD4:
// SmVertex * SmEdgeDefinition::GetStartBrepVertex( int iWhichBrep ) const
// SmVertex * SmEdgeDefinition::GetEndBrepVertex  ( int iWhichBrep ) const

/*******************************************************************//**
PURPOSE: Get the Brep Edge at the start of this EdgeDefinition, if there is one.

USAGE NOTES---- 
   If the ProtoVertex at the start of this EdgeDefinition has an SmEdge
   as its Brep topology, return that, else Null.
***********************************************************************/
SmEdge * SmEdgeDefinition::GetStartBrepEdge( int iWhichBrep ) const
{
  SmProtoVertex *pPV = this->GetStartProtoVertex();
  NE( pPV) ; if(pPV == NULL ) { return NULL; }

  return pPV->GetBrepEdge( iWhichBrep, this, TRUE) ;

} // end SmEdgeDefinition::GetStartBrepEdge

/*******************************************************************//**
PURPOSE: Get the Brep Edge at the end of this EdgeDefinition, if there is one.

USAGE NOTES---- 
   If the ProtoVertex at the end of this EdgeDefinition has an SmEdge
   as its Brep topology, return that, else Null.
***********************************************************************/
SmEdge * SmEdgeDefinition::GetEndBrepEdge( int iWhichBrep ) const
{
  SmProtoVertex *pPV = this->GetEndProtoVertex();
  NE( pPV) ; if(pPV == NULL ) { return NULL; }

  return pPV->GetBrepEdge( iWhichBrep, this, FALSE) ;

} // end SmEdgeDefinition::GetEndBrepEdge


/*******************************************************************//**
PURPOSE: Evaluate the location at a parameter along this EdgeDefinition.

USAGE NOTES---- 
***********************************************************************/
SmStatus SmEdgeDefinition::Evaluate( double       dParam, // in
                                     SmPoint3d  * pPos,   // out, optional. Default: NULL
                                     SmVector3d * pTan    // out, optional. Default: NULL
    ) const
{
  if(pPos == NULL && pTan == NULL )
    { return SM_SUCCESS; }

  if(pTan == NULL )
  {
      return m_pXSectCurve3d->EvaluatePoint( dParam, *pPos) ;
  }

  // Caller wants the deriv vector.
  SmVector3d sPtVec[2];
  SER( m_pXSectCurve3d->Evaluate( dParam, 1, TRUE, sPtVec )) ;

  if(pPos != NULL ) { *pPos = sPtVec[0]; }
  if(pTan != NULL ) { *pTan = sPtVec[1]; }

  return SM_SUCCESS;

} // end SmEdgeDefinition::Evaluate

/*******************************************************************//**
PURPOSE: Evaluate the midpoint of this EdgeDefinition.

USAGE NOTES---- Convenience method, helps clarify code.
***********************************************************************/
SmStatus SmEdgeDefinition::EvaluateMid( SmPoint3d  * pPos,  // out, optional. Default: NULL
                                        SmVector3d * pTan   // out, optional. Default: NULL
    ) const
{
  double dParam = m_pXSectInterval.GetMid();

  return this->Evaluate( dParam, pPos, pTan) ;

} // end SmEdgeDefinition::EvaluateMid

/*******************************************************************//**
PURPOSE: Reparameterize this EdgeDefinition.

USAGE NOTES---- 
   Reparameterize m_pXSectCurve3d, and the two uv curves if they exist.
   Reversed the parameterization if bReverse is True.
***********************************************************************/
SmStatus SmEdgeDefinition::ReverseOrientation()
{
  if(m_pXSectCurve3d != NULL )
    { SER( m_pXSectCurve3d->ReverseParameterization( m_pXSectInterval, m_pXSectInterval )); }
  if(m_pXSectUVCurve1 != NULL )
    { SER( m_pXSectUVCurve1 -> ReverseParameterization( m_pXSectInterval, m_pXSectInterval )); }
  if(m_pXSectUVCurve2 != NULL )
    { SER( m_pXSectUVCurve2 -> ReverseParameterization( m_pXSectInterval, m_pXSectInterval )); }

  return SM_SUCCESS;

} // end SmEdgeDefinition::ReverseOrientation

/*******************************************************************//**
PURPOSE: Reparameterize this EdgeDefinition.

USAGE NOTES---- 
   Reparameterize m_pXSectCurve3d, and the two uv curves if they exist.
   Reversed the parameterization if bReverse is True.
***********************************************************************/
SmStatus SmEdgeDefinition::Reparameterize( const SmExtent1d & rNewDomain, // in
                                                 SmBoolean    bReverse )  // in, opt. Default: FALSE
{
  if(bReverse )
    { this->ReverseOrientation(); }

  if(rNewDomain == m_pXSectInterval )
    { return SM_SUCCESS; }

  this->m_pXSectInterval = rNewDomain;

  SER( m_pXSectCurve3d->EditParameterization( rNewDomain ));

  if(m_pXSectUVCurve1 != NULL )
    { SER( m_pXSectUVCurve1->EditParameterization( rNewDomain )); }

  if(m_pXSectUVCurve2 != NULL )
    { SER( m_pXSectUVCurve2->EditParameterization( rNewDomain )); }

  return SM_SUCCESS;

}  // end SmEdgeDefinition::Reparameterize

/*******************************************************************//**
PURPOSE: return true when XSecting Interval has same Shape as this 
         EdgeDefinition. Set bIsIdentical = TRUE when SrcTopos are also the same

USAGE NOTES----
  with
   Shape  = TRUE when XSecting Crv Shapes are the same  (ED runs between TgtPVs and ED->Crv3d is within CaptureDistance of TgtCurve3d)
   Source = TRUE when XSecting Crv Sources are the same (ThisED->SrcTopos and the TgtClassification->CrvInterval->MidPt objects are the same)

  OUTPUT:
    bIsIdentical = Source ;
    return(Shape) ;

  Three output cases are possible:
   case1: return = TRUE, bIsIdentical = TRUE  : PE with DE already exists for this face pair XSect case : no work for user
   case2: return = TRUE, bIsIdentical = FALSE : PE without DE exists for this face pair XSect case      : caller will add a DE to this PE
   case3: return = FALSE                      : No PE exists for this face pair XSect case              : caller will add a PE and a DE to the ProtoManager
***********************************************************************/
SmBoolean SmEdgeDefinition::IsSameEdgeDefinition // rtn: TRUE = TgtXSectCrv has same Shape as a contained EdgeDefinition
 (SmProtoVertex               * pTgtStartPV,     // in : Tgt Start ProtoVertex
  SmProtoVertex               * pTgtEndPV,       // in : Tgt End   ProtoVertex
  const SmPointClassification & crMidPC1,        // in : MidPt Classification for face1->CrvClass->TgtCrvInterval
  const SmPointClassification & crMidPC2,        // in : MidPt Classification for face2->CrvClass->TgtCrvInterval
  const SmCurve               * pTgtCurve3d,     // in : shared XSectCurve3d being classified against face1/face2 pair
  const SmExtent1d            & rTgtCurveDomain, // in : TgtCrvInterval domain
  SmBoolean                   & rbIsIdentical)   // out: TRUE = this and TgtXSectCrv->Interval->MidPt classifications
 const                                           //             map to the same pair of Brep1 and Brep2 Src topology objects.
{
  // Init output
  rbIsIdentical = FALSE;

  // locals - thisEdgeDefinition->ProtoEdge->EndProtoVertices
  SmProtoVertex * pStartPV = m_pProtoEdge->GetStartProtoVertex() ;
  SmProtoVertex * pEndPV   = m_pProtoEdge->GetEndProtoVertex() ;

  // First, it would have to have the same two ProtoVertices.
  // (By definition, if the ends are the same, they have to be on the same ProtoVertices.)

  //cbi Do we need this?  Should just look at locations?
  SmBoolean bForward =   (pStartPV == pTgtStartPV && pEndPV   == pTgtEndPV) ? TRUE    // ThisPE runs from Tgt start to end PVs
                       : (pEndPV   == pTgtStartPV && pStartPV == pTgtEndPV) ? FALSE   // ThisPE runs from Tgt end to start PVs
                       : UNSURE ;                                                       // This PE does not run between Tgt PVs

  // ThisPE does not run between TgtPVs - not the same
  if(bForward == UNSURE )
    { return FALSE; }

  // The 3d curves have to be roughly the same (XSectTol3d is too small - use a stored larger CaptureDistance). ([B488] - gain of 4.0 was not big enough)
  double dTol = GetProtoTopologyManager()->GetCaptureDistance();

  // Very simple check for coincident TgtCrv and ThisED->Crv geometry - When crvs are not coincident - return FALSE
  //   - TRUE  = Midpts from both crvs are within tol of one another
  //   - FALSE = MidPts from both crvs are more than tol from one another
  if(FALSE == sm_AreSameCurve(m_pXSectCurve3d,   // in : 1st Curve
                              m_pXSectInterval,  // in : 1st Curve Ivl
                              pTgtCurve3d,       // in : 2nd Curve
                              rTgtCurveDomain,   // in : 2nd Curve Ivl
                              dTol ) )           // in : XSectTol3d value to check for coincidence
    { return FALSE; }

  // arrive here when ED runs between Tgt Protovertices and
  //                  ED->Crv3d is within CaptureDistance of TgtCurve3d
  
  // next check whether they're defined on the same topology.
  //    note: The brep1 and brep2 SrcTopos will be the same whether ED->Crv3d and TgtCrv3d run in the same or different directions

  // when ED->SrcTopos are the same as the TgtInterval->MidPt->Objects, this ED is same as Tgt inputs
   if(   crMidPC1.GetObject() == m_pSrcTopo1
      && crMidPC2.GetObject() == m_pSrcTopo2)
    { 
      // then the proposed EdgeDefintion is identical to this EdgeDefinition
      rbIsIdentical = TRUE; 
    }

  // Passed all the tests.
  return TRUE;

} // end SmEdgeDefinition::IsSameEdgeDefinition

/*******************************************************************//**
PURPOSE: Determine whether a position is interior to this EdgeDefinition

USAGE NOTES---- 
   bAtStart: Are we checking the start or end.  For the current use of this method,
   this is significant.  If you want to check both ends, call this twice.
***********************************************************************/
SmBoolean SmEdgeDefinition::IsInteriorPoint(
    const SmPoint3d   & crPosition, // in : Position to be tested
          SmXSectTol3d  sTol3d,     // in : max allowed dist from an end point
          SmBoolean     bAtStart,   // in : Start or end of this EdgeDef
          double      & rdParam     // out: parameter of crPosition on this EdgeDef
    ) const
{
  // Init output
  double dGuessParam = ( bAtStart ) ? m_pXSectInterval.GetMin() : m_pXSectInterval.GetMax();
  rdParam = dGuessParam;

  SmPoint3d sEndPos;
  this->GetEndPosition( bAtStart, sEndPos) ;

  double dDist = crPosition.DistanceBetween( sEndPos) ;

  if(dDist < sTol3d )
    { return FALSE; }

  // Drop the point to our 3d curve.
  SmBoolean bSuccess;
  double dDropDist;
  SmStatus eStat = this->m_pXSectCurve3d->DropPoint( m_pXSectInterval, crPosition, NULL, sTol3d, &dGuessParam,
    bSuccess, rdParam, dDropDist, SM_SO_MINIMIZE) ;

  if(eStat != SM_SUCCESS  ||  ! bSuccess )
    { return FALSE; }

  // Sometimes for very short Edges, the point will be at or near the far end.  We don't want that.
  if(bAtStart == TRUE  &&  rdParam > m_pXSectInterval.GetMid() )
    { return FALSE; }
  if(bAtStart == FALSE &&  rdParam < m_pXSectInterval.GetMid() )
    { return FALSE; }

  // See whether the split point is interior to this EdgeDef (in 3d).
  SmPoint3d sEdgePt;
  this->Evaluate( rdParam, &sEdgePt) ;

  dDist = sEdgePt.DistanceBetween( sEndPos) ;

  if(dDist > sTol3d )
    { return TRUE; }

  return FALSE;

} // end SmEdgeDefinition::IsInteriorPoint


/*******************************************************************//**
PURPOSE: Imprint this EdgeDefinition into both Breps.

USAGE NOTES---- 
   When this is called, the VertexDefintions in both ProtoVertices should have been imprinted.
***********************************************************************/
SmStatus SmEdgeDefinition::Imprint()
{
  // Make sure both SmVertices are in place before proceeding.

  SmProtoVertex *pStartPV = this->GetStartProtoVertex();
  SmProtoVertex *pEndPV   = this->GetEndProtoVertex();

  SmVertex *pStartVtx1 = pStartPV->GetFinalVertex( 1, this, TRUE ) ;
  if(pStartVtx1 == NULL )
    { NER( pStartVtx1) ; } // ... place for a breakpoint when debugging ...
  SmVertex *pEndVtx1   = pEndPV  ->GetFinalVertex( 1, this, FALSE) ;
  if(pEndVtx1 == NULL )
    { NER( pEndVtx1) ; }

  SmVertex *pStartVtx2 = pStartPV->GetFinalVertex( 2, this, TRUE ) ;
  if(pStartVtx2 == NULL )
    { NER( pStartVtx2) ; }
  SmVertex *pEndVtx2   = pEndPV  ->GetFinalVertex( 2, this, FALSE) ;
  if(pEndVtx2 == NULL )
    { NER( pEndVtx2) ; }

  SmBrep *pBrep1 = this->GetProtoTopologyManager()->GetWhichBrep( 1) ;
  SmBrep *pBrep2 = this->GetProtoTopologyManager()->GetWhichBrep( 2) ;

  SmPointClassificationType eType1 = this->GetSrcTopo_TYPE( 1) ;
  SmPointClassificationType eType2 = this->GetSrcTopo_TYPE( 2) ;

#ifdef SM_DEBUG_CODE
static int iDebugMe=0;
  if(iDebugMe>0 )
  {
      this->Dump( cbiDumpLevel) ;

      if(iDebugMe > 5 ) {
          pBrep1->Dump();
          pBrep2->Dump();

          smgfx_Erase();
          smgfx_SetLook( 1,2, 0,0,1) ; pBrep1->Draw(TRUE); sm_GraphicsLoop();
          smgfx_SetLook( 1,2, 0,1,0) ; pBrep2->Draw(TRUE); sm_GraphicsLoop();
          sm_GraphicsLoop();
      }
      if(iDebugMe > 10 ) {
          SM_ASSERT_VALID( pBrep1) ;
          SM_ASSERT_VALID( pBrep2) ;
      }

      smgfx_SetLook( 2,4, 1,0,0) ; this->Draw(); sm_GraphicsLoop();
      sm_GraphicsLoop();
  }
#endif // SM_DEBUG_CODE


  // First do Brep 1.
  SmEdge *pNewE1 = NULL;
  SmLoop *pNewL1 = NULL;
  SmFace *pNewF1 = NULL;
  SmFace *pOldF1 = NULL;

  SmStatus eStat;

  if(eType1 == SM_PC_FACE )
  {
      SmTopology *pBrepTopo = GetSrcTopo(1);
      pOldF1 = SM_CAST_PTR( SmFace, pBrepTopo) ; NER( pOldF1) ;
      SmBSplineCurve *pTempUV = SM_CAST_PTR( SmBSplineCurve, m_pXSectUVCurve1) ;

#ifdef SM_DEBUG_CODE
static int iDebugMe2=0;
      if(iDebugMe2>0 )
      {
          if(iDebugMe2>5 ) {
              smgfx_Erase();
              smgfx_SetLook( 1,2, 0,0,1) ; pBrep1->Draw(TRUE); sm_GraphicsLoop();
              smgfx_SetLook( 1,2, 0,1,0) ; pBrep2->Draw(TRUE); sm_GraphicsLoop();
              smgfx_SetLook( 2,4, 1,0,0) ; this->Draw();       sm_GraphicsLoop();
              sm_GraphicsLoop();
          }
          smgfx_SetLook( 1,2, 1,0,1) ; pOldF1->DrawUV(2,2); sm_GraphicsLoop();

          sm_GraphicsLoop();
      }
#endif // SM_DEBUG_CODE

      double dThisTol = smos_Max( (double)pOldF1->GetTolerance(), this->m_dTol) ;

      // Note, input curves are consumed:
      eStat = pBrep1->MakeEdgeInFace( pOldF1, pStartVtx1, pEndVtx1,
                  m_pXSectCurve3d, pTempUV, m_pXSectInterval, SM_OT_SAME, dThisTol,
                  pNewE1, pNewL1, pNewF1) ;

      if(eStat != SM_SUCCESS )
      {
          this->Destruct(); // Get rid of our curves, which were not consumed.
          SER( eStat) ;
      }

      m_pFinalEdge1 = pNewE1;

#ifdef SM_DEBUG_CODE
      SmBoolean bDebugMe = FALSE;
      if(bDebugMe )
      {
          SM_ASSERT_VALID( pOldF1) ;
          SM_ASSERT_VALID( pNewF1) ;
      }

      if(iDebugMe2>0 && pNewF1 != NULL )
      {
          if(iDebugMe2>5 ) {
              smgfx_Erase();
              smgfx_SetLook( 1,2, 0,0,1) ; pBrep1->Draw(TRUE); sm_GraphicsLoop();
              smgfx_SetLook( 1,2, 0,1,0) ; pBrep2->Draw(TRUE); sm_GraphicsLoop();
              smgfx_SetLook( 2,4, 1,0,0) ; this->Draw();       sm_GraphicsLoop();
              sm_GraphicsLoop();
          }
          smgfx_SetLook( 1,2, 1,0,1) ; pOldF1->DrawUV(2,2); sm_GraphicsLoop();
          if(pNewF1 != NULL )
            { smgfx_SetLook( 1,2, 0,1,1) ; pNewF1->DrawUV(3,3); sm_GraphicsLoop(); }

          sm_GraphicsLoop();
      }
#endif // SM_DEBUG_CODE

      // If a Face was split, any ProtoTopology that was defined on the original Face
      // is now on either the original or the new Face.  Move any that is now on the
      // new Face from the original to the new.
      // Note: Defer this until after the Edge is imprinted into Brep2.
      //      if(pNewF1 != NULL )
      //      {
      //          SmEdge *pOtherEdge = SM_CAST_PTR( SmEdge, GetSrcTopo(2)) ;
      //          SER( this->GetProtoTopologyManager()->UpdateFaces( 1, pOldF1, pNewF1, pNewE1, pOtherEdge ));
      //      }

  }
  else if(eType1 == SM_PC_EDGE )
  {
      SmTopology *pBrepTopo = GetSrcTopo(1);
      m_pFinalEdge1 = SM_CAST_PTR( SmEdge, pBrepTopo) ;

      SM_ASSERT( m_pFinalEdge1 != NULL) ;
  }


  // Now do Brep 2.
  SmEdge *pNewE2 = NULL;
  SmLoop *pNewL2 = NULL;
  SmFace *pNewF2 = NULL;
  SmFace *pOldF2 = NULL;

  if(eType2 == SM_PC_FACE )
  {
      // m_pXSectCurve3d went into Brep1.  Make a copy for the other Brep.
      SmCurve *pCopyCurve = NULL;
      eStat = m_pXSectCurve3d->Copy( *m_cpContext, pCopyCurve) ;

      SmTopology *pBrepTopo = GetSrcTopo(2);
      pOldF2 = SM_CAST_PTR( SmFace, pBrepTopo) ; NER( pOldF2) ;

      double dThisTol = smos_Max( (double)pOldF2->GetTolerance(), this->m_dTol) ;

      SmBSplineCurve *pTempUV = SM_CAST_PTR( SmBSplineCurve, m_pXSectUVCurve2) ;
      eStat = pBrep2->MakeEdgeInFace( pOldF2, pStartVtx2, pEndVtx2,
                  pCopyCurve, pTempUV, m_pXSectInterval, SM_OT_SAME, dThisTol,
                  pNewE2, pNewL2, pNewF2) ;

      if(eStat != SM_SUCCESS )
      {
          // We have already created an Edge in Brep1.  When this routine returns Error,
          // the caller will call Destruct() on this EdgeDefinition, which will delete
          // our three curves.  But m_pXSectCurve3d and m_pXSectUVCurve1 (if not null) have been
          // implanted into Brep1, so we can't delete them.  So just null them out here.
          // Note, we'll just leave the Edge in Brep1.
          m_pXSectCurve3d = m_pXSectUVCurve1 = NULL;

          // We also have to delete the copy, which would have gone into Brep2.
          delete pCopyCurve; pCopyCurve = NULL;

          SER( eStat) ;
      }

      m_pFinalEdge2 = pNewE2;

      // If a Face was split, any ProtoTopology that was defined on the original Face
      // is now on either the original or the new Face.  Move any that is now on the
      // new Face from the original to the new.
      if(pNewF2 != NULL )
      {
          SmTopology *pBrepTopology = GetSrcTopo(1);
          SmEdge *pOtherEdge = SM_CAST_PTR( SmEdge, pBrepTopology) ;
          SER( this->GetProtoTopologyManager()->UpdateFaces( 2, pOldF2, pNewF2, pNewE2, pOtherEdge, this ));
      }
#ifdef SM_DEBUG_CODE
      SmBoolean bDebugMe = FALSE;
      if(bDebugMe )
      {
          SM_ASSERT_VALID( pOldF2) ;
          SM_ASSERT_VALID( pNewF2) ;
      }
#endif
  }
  else if(eType2 == SM_PC_EDGE )
  {
      SmTopology *pBrepTopo = GetSrcTopo(2);
      m_pFinalEdge2 = SM_CAST_PTR( SmEdge, pBrepTopo) ;

      SM_ASSERT( m_pFinalEdge2 != NULL) ;
  }

  // Deferred:
  if(pNewF1 != NULL )
  {
      SmTopology *pBrepTopo = GetSrcTopo(2);
      SmEdge *pOtherEdge = SM_CAST_PTR( SmEdge, pBrepTopo) ;
      SER( this->GetProtoTopologyManager()->UpdateFaces( 1, pOldF1, pNewF1, pNewE1, pOtherEdge, this ));
  }

#ifdef SM_DEBUG_CODE
  if(iDebugMe > 10 ) {
      SM_DUMP_AND_ASSERT_VALID( pBrep1) ;
      SM_DUMP_AND_ASSERT_VALID( pBrep2) ;
  }
#endif // SM_DEBUG_CODE

  // If this EdgeDefinition was defined on an Edge in either Brep, then our curves
  // were not consumed, by being implanted into the Breps in MakeEdgeInFace(), and so
  // must be deleted.
  // But, we can't do this yet, because these might still be used later, by UpdateFaces().
  // Do it in the destructor instead.
  // if(eType1 == SM_PC_EDGE )
  // {
  //     delete m_pXSectCurve3d; m_pXSectCurve3d = NULL;
  //     delete m_pXSectUVCurve1  ; m_pXSectUVCurve1   = NULL;
  // }
  // if(eType2 == SM_PC_EDGE )
  // {
  //     delete m_pXSectCurve3d; m_pXSectCurve3d = NULL;
  //     delete m_pXSectUVCurve2  ; m_pXSectUVCurve2   = NULL;
  // }

  return SM_SUCCESS;

} // end SmEdgeDefinition::Imprint

/*******************************************************************//**
PURPOSE: Split a SmEdgeDefinition at the given parameter after splitting
         the owning ProtoEdge

USAGE NOTES---- 
   If dT is not interior to our curve (to 3d tolerance), does not split.

   After a successful split, 'this' will be the lower part of the domain
   and rpNewPE will be the higher part.

   Makes sure that all of the new ProtoTopology is properly connected
   and installed in the owning SmProtoTopologyManager.
***********************************************************************/
SmStatus SmEdgeDefinition::Split
 (double                 dParamPE, // in : Split parameter on ProtoEdge to be used to split this EdgeDefinition
  SmProtoVertex      *   pMidPV,   // in : pMidPV = ProtoVertex at ProtoEdge split point
  SmProtoEdge        *   pTopPE,   // in : pTopPE = new ProtoEdge for ProtoEdge UpperSplitChild
  SmVertexDefinition *& rpNewVD,   // out: new VertexDefinition at Split point. Gets added to pMidPV
  SmEdgeDefinition   *& rpNewED)   // out: new Split UpperChild EdgeDefinition. Gets added to pTopPE
                                   // note: this EdgeDefinition gets reused as Split LowerChild EdgeDefinition
{
  rpNewVD = NULL;
  rpNewED = NULL;

  // Check that parameter is interior to this EdgeDef.
  // The first check just tests whether it's exterior.  The following 3d tests
  // deal with tolerances, so this check doesn't have to.
  if(! m_pXSectInterval.ContainsValue( dParamPE, -SM_EFF_ZERO ) )
    { return SM_SUCCESS; }   //cbiTol breakpoint: how does this happen?

  SmPoint3d sCrvPt, sPt;
  this->Evaluate( dParamPE, &sCrvPt) ;  // Takes reversal into account.

static constexpr double cbiFudge = 10.0;  //cbiTol: this is not working right.  Tol is too small.

  // Use distance to end of this EdgeDef, not distance to our VertexDefs,
  // because there could be tolerance issues with that.
  SmProtoVertex *pStartPV = m_pProtoEdge->GetStartProtoVertex(); // DD2
  SmProtoVertex *pEndPV   = m_pProtoEdge->GetEndProtoVertex();

  SER( this->GetEndPosition( pStartPV, sPt ));
  if(sCrvPt.DistanceBetween( sPt ) < m_dTol * cbiFudge )
    { return SM_SUCCESS; }

  SER( this->GetEndPosition( pEndPV, sPt ));
  if(sCrvPt.DistanceBetween( sPt ) < m_dTol * cbiFudge )
    { return SM_SUCCESS; }


  // Ok, go ahead and split.
  // Need to classify point. Can't just use topo objects from edges
  SmBrep *pBrep1 = this->GetProtoTopologyManager()->GetWhichBrep( 1) ;
  SmBrep *pBrep2 = this->GetProtoTopologyManager()->GetWhichBrep( 2) ;
  SmPointClassification sPC1( m_dTol, pBrep1->GetContext()) , sPC2( m_dTol, pBrep2->GetContext());
  pBrep1->Point3DClassify( sCrvPt, m_dTol, TRUE, sPC1) ;
  pBrep2->Point3DClassify( sCrvPt, m_dTol, TRUE, sPC2) ;
  
  SmVertexDefinition *pNewVD = NULL;
  SmBoolean bCreatedNew = FALSE;
  pMidPV->AddVertexDefinition( sPC1, sPC2, pNewVD, &bCreatedNew) ;

  NER( pNewVD) ;

  //pNewVD->SetProtoVertex( pMidPV) ;
  //pMidPV->AddVertexDefinition( pNewVD) ;
  

  // For cleanup object: don't do it if it was 'found' (vs. created).
  SmVertexDefinition *pCleanVD = ( bCreatedNew ) ? pNewVD : NULL;
  SmObjDelete sCleanVD( pCleanVD) ;

  SmEdgeDefinition *pNewED = new( this->GetProtoTopologyManager() ) SmEdgeDefinition( *this) ; // copy c'tor
  NER( pNewED) ;
  sCleanVD.Clear();

  // Get the parameterizations set correctly before calling AddEdgeDefinition().

  this  ->m_pXSectInterval.SetMax( dParamPE) ;
  pNewED->m_pXSectInterval.SetMin( dParamPE) ;

  pTopPE->AddEdgeDefinition( pNewED) ;


  // Here we have to:
  // - For three curves, m_pXSectCurve3d, m_pXSectUVCurve1, m_pXSectUVCurve2:
  //   - copy curve, because some places ( EU::NormalizedEvaluate() ) assume uv crv Natural Ivl is Edge domain.
  //   - replace curves in pNewED
  //   - trim curve in 'this'
  //   - trim curve in pNewED.

  SmCurve *pNewCrv3d = NULL;
  SER( this->m_pXSectCurve3d->Copy( *m_cpContext, pNewCrv3d ));  NER( pNewCrv3d) ;
  pNewED->SetXSectCurve3d( pNewCrv3d) ;
  this  ->m_pXSectCurve3d->Trim( this  ->m_pXSectInterval) ;  // may snap sIvl by tol to existing knots
  pNewED->m_pXSectCurve3d->Trim( pNewED->m_pXSectInterval) ;  // may snap sIvl by tol to existing knots

  if(m_pXSectUVCurve1 != NULL )
  {
      SmCurve *pNewCrvUV1 = NULL;
      SER( m_pXSectUVCurve1->Copy( *m_cpContext, pNewCrvUV1 ));  NER( pNewCrvUV1) ;
      pNewED->SetXSectCurveUV( 1, pNewCrvUV1) ;
      pNewED->m_pXSectUVCurve1->Trim( pNewED->m_pXSectInterval) ; // may snap sIvl by tol to existing knots
      this  ->m_pXSectUVCurve1->Trim( this  ->m_pXSectInterval) ; // may snap sIvl by tol to existing knots
  }
  if(m_pXSectUVCurve2 != NULL )
  {
      SmCurve *pNewCrvUV2 = NULL;
      SER( m_pXSectUVCurve2->Copy( *m_cpContext, pNewCrvUV2 ));  NER( pNewCrvUV2) ;
      pNewED->SetXSectCurveUV( 2, pNewCrvUV2) ;
      pNewED->m_pXSectUVCurve2->Trim( pNewED->m_pXSectInterval) ; // may snap sIvl by tol to existing knots
      this  ->m_pXSectUVCurve2->Trim( this  ->m_pXSectInterval) ; // may snap sIvl by tol to existing knots
  }

  rpNewVD = pNewVD;
  rpNewED = pNewED;

  return SM_SUCCESS;

} // end SmEdgeDefinition::Split






///////////////////////////////////////////
// SmProtoVertex methods
///////////////////////////////////////////

/*******************************************************************//**
PURPOSE: SmProtoVertex constructor for empty ProtoVertex

USAGE NOTES---- created ProtoVerex has no VertexDefinitions
***********************************************************************/
SmProtoVertex::SmProtoVertex
 (const SmPoint3d        & crPos,   // in : new ProtoVertex Position (commonly from a XSectPt3d solution) 
  double                   dTol,    // in : new ProtoVertex ZoneTol3d
  SmProtoTopologyManager * pOwner ) // in : new ProtoVertex owning ProtoManager
    : m_sPosition    ( crPos ),
      m_dTol         ( dTol ),
      m_pProtoTopoMgr( pOwner )
{
  // init contained arrays
  m_sVertexDefinitions.ReSet() ;
  m_sProtoEdges.ReSet() ;

  // set context
  SetContext( pOwner->GetContext()) ;

} // end SmProtoVertex constructor

/*******************************************************************//**
PURPOSE: SmProtoVertex constructor given PointClassifications

USAGE NOTES---- 
***********************************************************************/
SmProtoVertex::SmProtoVertex
 (const SmPointClassification & crPC1,     // in : Face1 PtClassification of XSectPt3d solution
  const SmPointClassification & crPC2,     // in : Face2 PtClassification of XSectPt3d solution
  const SmPoint3d             & rPosition, // in : new ProtoVertex Position (commonly from a XSectPt3d solution)
  double                        dTol,      // in : new ProtoVertex ZoneTol3d
  SmProtoTopologyManager      * pOwner )   // in : new ProtoVertex owning ProtoManager
    : m_sPosition    ( rPosition ),
      m_dTol         ( dTol ),
      m_pProtoTopoMgr( pOwner )
{
  // init contained arrays
  m_sVertexDefinitions.ReSet() ;
  m_sProtoEdges .ReSet() ;

  // context
  const SmContext * pCtx = pOwner->GetContext() ;

  // make new VertexDefintion
  SmVertexDefinition * pVD  = new ( pOwner ) SmVertexDefinition( pCtx, &crPC1, &crPC2, rPosition, dTol, -1.0, this) ;

  // add VD to new ProtoVertex
  m_sVertexDefinitions.Add( pVD) ;

  // set context for new ProtoVertex
  SetContext( pCtx) ;

} // end SmProtoVertex constructor

/*******************************************************************//**
PURPOSE: SmProtoVertex constructor given a VertexDefinition

USAGE NOTES---- 
***********************************************************************/
SmProtoVertex::SmProtoVertex
 (SmVertexDefinition     * pVtxDef,  // in : new ProtoVertex 1st VertexDef
  SmProtoTopologyManager * pOwner )  // in : new ProtoVertex owning ProtoManager
{
  m_pProtoTopoMgr = pOwner;

  m_sVertexDefinitions.ReSet();
  m_sProtoEdges .ReSet();

  if(pVtxDef != NULL )
  {
      m_sVertexDefinitions.Add( pVtxDef) ;

      m_sPosition = pVtxDef->GetPosition();
      m_dTol = pVtxDef->GetTolerance();
  }

  SetContext( pOwner->GetContext()) ;

} // end SmProtoVertex constructor

/*******************************************************************//**
PURPOSE: SmProtoVertex constructor given topology and position

USAGE NOTES----   cbi likely delete this one.
***********************************************************************/
/*
SmProtoVertex::SmProtoVertex( SmTopology *pTopo1, SmTopology *pTopo2, SmPoint3d &rPosition, double dTol )
    : m_pSrcTopo1( pTopo1 ),
      m_pSrcTopo2( pTopo2 ),
      m_sPosition( rPosition ),
      m_dTol( dTol ),
      m_pFinalVtx1( NULL ),
      m_pFinalVtx2( NULL )
{
cbi context?

  switch ( pTopo1->GetType() )
  {
    case SmVertex_TYPE :
      { m_ePC1 = SM_PC_VERTEX; break; }
    case SmEdge_TYPE :
      { m_ePC1 = SM_PC_EDGE;   break; }
    case SmFace_TYPE :
      { m_ePC1 = SM_PC_FACE;   break; }
    case SmRegion_TYPE :
      { m_ePC1 = SM_PC_REGION; break; }
    default:
    {
      m_ePC1 = SM_PC_UNKNOWN;
      ERR_MSG( _T("Error: SmProtoVertex constructor with invalid topology\n")) ;
    }
  }

  switch ( pTopo2->GetType() )
  {
    case SmVertex_TYPE :
      { m_ePC2 = SM_PC_VERTEX; break; }
    case SmEdge_TYPE :
      { m_ePC2 = SM_PC_EDGE;   break; }
    case SmFace_TYPE :
      { m_ePC2 = SM_PC_FACE;   break; }
    case SmRegion_TYPE :
      { m_ePC2 = SM_PC_REGION; break; }
    default:
    {
      m_ePC2 = SM_PC_UNKNOWN;
      ERR_MSG( _T("Error: SmProtoVertex constructor with invalid topology\n")) ;
    }
  }
} // end SmProtoVertex constructor

cbi. */

/*******************************************************************//**
PURPOSE: SmProtoVertex destructor

USAGE NOTES---- 
***********************************************************************/
SmProtoVertex::~SmProtoVertex()
{
  ULONG ii, lNumVDs = m_sVertexDefinitions.GetSize();
  for(ii=0;ii<lNumVDs;ii++)
  {
      delete m_sVertexDefinitions[ii];
  }

  // Also, we have ProtoEdges pointing to this PV.
  ULONG lNumPEs = m_sProtoEdges.GetSize();
  for(ii=0;ii<lNumPEs;ii++)
  {
      m_sProtoEdges[ii]->RemoveProtoVertex( this) ;
  }
} // end SmProtoVertex destructor

/*******************************************************************//**
PURPOSE: Get the Capture Distance for this ProtoVertex.

USAGE NOTES---- 
***********************************************************************/
double SmProtoVertex::GetCaptureDistance() const
{ return m_pProtoTopoMgr->GetCaptureDistance(); }

/*******************************************************************//**
PURPOSE: Determine whether a position shoould be considered 'in' this ProtoVertex.

USAGE NOTES---- 
***********************************************************************/
SmBoolean SmProtoVertex::ContainsPoint( const SmPoint3d &crPosition )
{
  // m_dTol >= Local object Tols. Takes precedence over CaptureDistance [B673]
  return ( m_sPosition.CloserThan( smos_Max(m_dTol, GetCaptureDistance()), crPosition )) ;

} // end SmProtoVertex::ContainsPoint

/*******************************************************************//**
PURPOSE: Return VertexDefinition matching Tgt Brep1/Brep2 SrcTopo objs
         or NULL for none

USAGE NOTES---- 
***********************************************************************/
SmVertexDefinition * SmProtoVertex::FindMatchingVertexDefinition
 (const SmTopology * pTopo1, // in : Brep1 SrcTopo for desired ProtoDefinition
  const SmTopology * pTopo2) // in : Brep2 SrcTopo for desired ProtoDefinition
{
  // locals
  ULONG ii ;
  ULONG lNumVDs = m_sVertexDefinitions.GetSize();

  // for every VertexDefinition - find the first that was defined by the two input Topology objs
  for(ii=0;ii<lNumVDs;ii++)
    {
      SmVertexDefinition *pVD = m_sVertexDefinitions[ii];
      if(   pVD->GetSrcTopo( 1 ) == pTopo1
         && pVD->GetSrcTopo( 2 ) == pTopo2)
        { return pVD; }
    }

  // all done - arrive here with no matching VDs 
  return NULL;

} // end SmProtoVertex::FindMatchingVertexDefinition

/*******************************************************************//**
PURPOSE: return TRUE when one of the ProtoEdges connected to this 
         ProtoVertex is part of a OverlappingGroup

USAGE NOTES---- 
***********************************************************************/
SmBoolean SmProtoVertex::IsOverlappingPEGroupMember() const
{
  // locals
  ULONG ii ;
  SmTArray<SmProtoEdge *> sOverlappingGroup ; 
  SmTArray<SmProtoEdge *> & rPEs  = m_pProtoTopoMgr->GetProtoEdges() ;

  // for every ProtoEdge - look for one in a OverlapGroup
  for(ii=0;ii<rPEs.GetSize();ii++)
    {
      SmProtoEdge * pPE = rPEs[ii] ;
      pPE->GetOverlappingPEGroup(sOverlappingGroup) ;

      // when in a OverlapGroup - return TRUE
      if(sOverlappingGroup.GetSize() > 0)
        { return(TRUE) ; }
     
    } // end iter every PV->PEs looking for Group membership

  // arrive here when not in a group
  return(FALSE) ; 

} // end SmProtoVertex::IsOverlappingPEGroupMember

/*******************************************************************//**
PURPOSE: Find closest SmProtoVertex within m_dTol of a Tgt position

USAGE NOTES----  returns NULL when no ProtoVertices are within Tol of rPosition
***********************************************************************/
SmProtoVertex * SmProtoTopologyManager::FindProtoVertex
 (const SmPoint3d & rPosition, // in : Target position
  double          * pdDist)    // out: Deviation from found ProtoVertex to TgtPos, NULL to ignore, default:[NULL]
{
  // Init outputs.
  SmProtoVertex *pRetVal = NULL;
  if(pdDist != NULL) { *pdDist = SM_BIG_DOUBLE; }

  // locals
  ULONG ii ;
  ULONG           lNumPVs  = m_sPVs.GetSize() ;
  double          dMinDist = SM_BIG_DOUBLE ;
  double          dThisDist ;
  SmProtoVertex * pPV      = NULL ;

  // for every ProtoVertex - find the closest one to rPosition that's within max(m_dTol, GetCaptureDistance())
  for(ii=0;ii<lNumPVs;ii++)
    {
      pPV = m_sPVs[ii];
      if(pPV->ContainsPoint( rPosition ) ) // checks ProtoVertex to be within m_dTol (ZoneTol3d) of rPosition
        {
          dThisDist = pPV->DistanceTo( rPosition) ;
          if(dThisDist < dMinDist )
            {
              pRetVal = pPV;
              dMinDist = dThisDist;
            } // end is closest ProtoVertex within tol check
        } // end Pos is within dist of ProtoVertex check
    } // end iter every ProtoVertex

  // set output
  if(pdDist != NULL ) { *pdDist = dMinDist; }

  // all done
  return pRetVal;

} // end SmProtoTopologyManager::FindProtoVertex

/*******************************************************************//**
PURPOSE: Find the SmEdgeDefinition that refers to a TgtEdge.

USAGE NOTES---- 1. WARNING: pTargetEdge will likely be a stale pointer, 
                            so do not dereference it.
***********************************************************************/
SmEdgeDefinition * SmProtoTopologyManager::FindEdgeDefinitionFromEdge
 (SmEdge * pTargetEdge, // in : TgtEdge referenced by desired EdgeDefinition
  int      iWhichBrep)  // in : iWhichBrep == 1 ? m_Brep1 : m_Brep2
{
  // We could optimize by pointing from SmEdge to SmEdgeDef.  Attribute?
  // But for now just search the list.

  // locals
  ULONG ii ;
  SmEdge           * pE      = NULL ;
  SmTopology       * pTopo   = NULL ;
  SmEdgeDefinition * pED     = NULL ;
  SmTArray<SmEdgeDefinition*> sEDs;
  this->GetAllEdgeDefinitions(sEDs) ;
  ULONG              lNumEDs = sEDs.GetSize();

  // for every EdgeDefinition
  for(ii=0;ii<lNumEDs;ii++)
    {
      pED   = sEDs[ii];

      // 1st check the EdgeDefinition->BrepNTopo
      pTopo = pED->GetSrcTopo(iWhichBrep) ;
      if(pTopo == pTargetEdge )
        { return pED; }

      // 2nd check EdgeDefinition->FinalEdge in case there's a 
      // Final SmEdge and it's different from the BrepNTopo
      pE = pED->GetFinalEdge( iWhichBrep) ;
      if(pE == pTargetEdge )
        { return pED; }
    } // end iter every EdgeDefinition

  // all done
  return NULL ;

} // end SmProtoTopologyManager::FindEdgeDefinitionFromEdge

/*******************************************************************//**
PURPOSE: Remove input VertexDefinition from this ProtoVertex's list.

USAGE NOTES---- 
  Returns SM_ERR when pVertexDef is not in this PV->m_sVertexDefs list else returns SM_SUCCESS
***********************************************************************/
SmStatus SmProtoVertex::RemoveVertexDefinition
 (SmVertexDefinition * pVD) // in : TgtVertexDef to remove from this ProtoVertex
{
  ULONG idx ;
  
   // when pVD is in this ProtoVertex->VertexDef list - remove it
  if(m_sVertexDefinitions.FindElement(pVD, idx)) { m_sVertexDefinitions.RemoveAt(idx) ; }
  else                                           { return SM_ERR; }

  // all done
  return SM_SUCCESS;

} // end SmProtoVertex::RemoveVertexDefinition

/*******************************************************************//**
PURPOSE: Remove a ProtoEdge from this ProtoVertex's list.

USAGE NOTES---- 
   Returns Error if pPE is not in our list.
   Does not delete the ProtoEdge.
***********************************************************************/
SmStatus SmProtoVertex::RemoveProtoEdge( SmProtoEdge *pPE )
{
  ULONG idx;
  if(m_sProtoEdges.FindElement( pPE, idx ) )
    { m_sProtoEdges.RemoveAt( idx) ; }
  else
    { return SM_ERR; }

  return SM_SUCCESS;

} // end SmProtoVertex::RemoveProtoEdge

/*******************************************************************//**
PURPOSE: Find the VertexDefinition closest to the given position.

USAGE NOTES---- 
***********************************************************************/
SmVertexDefinition * SmProtoVertex::GetClosestVertexDefinition( const SmPoint3d & crPos ) const
{
  ULONG ii ;
  SmVertexDefinition *pRet = NULL;
  double dDist, dMinDist = SM_BIG_DOUBLE;
  ULONG lNumVDs = m_sVertexDefinitions.GetSize();
  for(ii=0;ii<lNumVDs;ii++)
  {
      dDist = crPos.DistanceBetweenSquared( m_sVertexDefinitions[ii]->GetPosition()) ;
      if(dDist < dMinDist )
      {
          pRet = m_sVertexDefinitions[ii];
          dMinDist = dDist;
      }
  }
  return pRet;

} // end SmProtoVertex::GetClosestVertexDefinition

/*******************************************************************//**
PURPOSE: Get the Brep SmVertex that corresponds to the given EdgeDefinition.

USAGE NOTES---- 
   Returns an SmVertex if it's connected to the Edge in the given EdgeDef.
   If more than one VertexDefinition has the proper topological connections,
   returns the closest one.
***********************************************************************/
SmVertex * SmProtoVertex::GetFinalVertex
 (int                      iWhichBrep,  // in : 1 == m_pSrcTopo1, !1 == m_pSrcTopo2
  const SmEdgeDefinition * pEdgeDef,    // in : rtns SmVertex connected to this TgtEdgeDef
  SmBoolean                bAtStart )   // in : TRUE = pEdgeDef->start, FALSE = pEdgeDef->End
 const
{
  ULONG ii, lNumVDs = m_sVertexDefinitions.GetSize();
  if(lNumVDs == 0 )
    { return NULL; }

  //cbi If only one VD, should we just use that, or always check connected?
  //cbi I'd say always check connected: if this is wrong it will bomb out
  //cbi somewhere, and will probably be harder to find.
  //cbi if(lNumVDs == 1 )
  //cbi   { return m_sVertexDefinitions[0]->GetFinalVertex( iWhichBrep) ; }

  SmVertex *pRetVal = NULL;
  double    dDist, dMaxDist = SM_BIG_DOUBLE;

//cbi breakpoint:
  SmTopology *pBrepTopoED = pEdgeDef->GetSrcTopo( iWhichBrep) ;

  SmPoint3d sEDPos;
  SmStatus eStat = pEdgeDef->GetEndPosition( bAtStart, sEDPos) ;
  if(eStat != SM_SUCCESS )
    { return NULL; }

  for(ii=0;ii<lNumVDs;ii++)
  {
      SmVertexDefinition *pVD = m_sVertexDefinitions[ii];
      SmVertex *pVtx = pVD->GetFinalVertex( iWhichBrep) ;
      if(pVtx == NULL ) { continue; }

#ifdef SM_DEBUG_CODE
ULONG di ;
SmBoolean bDebugMe = FALSE;
      if(bDebugMe )
      {
          SmFace *pFace = SM_CAST_PTR( SmFace, pBrepTopoED) ;
          if(pFace != NULL )
          {
              SmTArray<SmFace*> sVFaces;
              pVtx->GetFaces( sVFaces) ;

              smgfx_Erase();
              smgfx_SetLook( 1,2, 0,0,1) ; if(pVtx->GetBrep() ) pVtx->GetBrep()->Draw(); sm_GraphicsLoop();
              pVD->GetProtoTopologyManager()->DrawDebug();
              sm_GraphicsLoop();
              smgfx_ChangeColor(FALSE) ;
              for(di=0;di<sVFaces.GetSize();di++)
                { sVFaces[di]->Draw(SM_DM_CROSSHATCH); sm_GraphicsLoop(); }
              smgfx_SetLook( 3,5, 1,0,1) ; pFace->Draw(SM_DM_CROSSHATCH); sm_GraphicsLoop();
              smgfx_SetLook( 1,5, 1,0,0) ; pVtx->Draw(); sm_GraphicsLoop();
              sm_GraphicsLoop();
          } // end BrepTopoED == #SmFace# check

          SmEdge *pEdge = SM_CAST_PTR( SmEdge, pBrepTopoED) ;
          if(pEdge != NULL )
          {
              SmTArray<SmEdge*> sVEdges;
              pVtx->GetEdges( sVEdges) ;

              smgfx_Erase();
              if(pVtx->GetBrep()) { smgfx_SetLook(1,2, 0, !(iWhichBrep == 1), iWhichBrep == 1) ; pVtx->GetBrep()->Draw(); sm_GraphicsLoop(); }
               // TRUE = change color
              for(di=0;di<sVEdges.GetSize();di++)
                { smgfx_SetLook(3,4, TRUE) ; sVEdges[di]->Draw(); sm_GraphicsLoop(); }   // DrawEdge = this#SmProtoVertex# -> m_sVertexDefinitions[ii]#SmVertexDefinition# -> GetFinalVertex#SmVertex#->GetEdges[di]
              smgfx_SetLook(5,6, TRUE) ; pEdge->Draw(); sm_GraphicsLoop();               // DrawEdge = SM_CAST_PTR(SmEdge, pEdgeDef#SmEdgeDefinition# -> GetSrcTopo(iWhichBrep))
              smgfx_SetLook(7,8, 1,0,0);  pVtx->Draw(); sm_GraphicsLoop();               // DrawVtx  = this#SmProtoVertex# -> m_sVertexDefinitions[ii]#SmVertexDefinition# -> GetFinalVertex#SmVertex#
              sm_GraphicsLoop();
          } // end BrepTopoED == #SmEdge# check
      }
#endif // SM_DEBUG_CODE

      if(sm_AreConnected( pBrepTopoED, pVtx ) )
      {
          //cbi Reg_090406_ellaby: short ED, both ends on same PV.
          //cbi return pVD->GetFinalVertex( iWhichBrep) ;

          dDist = sEDPos.DistanceBetween( pVtx->GetPoint()) ;
          if(dDist < dMaxDist )
          {
              dMaxDist = dDist;
              pRetVal  = pVtx;
          }
      }
  }

  return pRetVal;

} // end SmProtoVertex::GetFinalVertex

/*******************************************************************//**
PURPOSE: Get the Brep SmEdge that corresponds to the given EdgeDefinition.

USAGE NOTES---- 
   Returns an SmEdge if it's the same as the Edge in the given EdgeDef.
***********************************************************************/
SmEdge * SmProtoVertex::GetBrepEdge
 (int                      iWhichBrep,  // in :
  const SmEdgeDefinition * pEdgeDef,    // in :
  SmBoolean                bAtStart)    // NotUsed: in :
 const
{
  SM_REF1(bAtStart) ;
  ULONG ii, lNumVDs = m_sVertexDefinitions.GetSize();
  if(lNumVDs == 0 )
    { return NULL; }

  SmTopology *pBrepTopoED = pEdgeDef->GetSrcTopo( iWhichBrep) ;
  SmEdge *pEdgeED = SM_CAST_PTR( SmEdge, pBrepTopoED) ;
  if(pEdgeED == NULL )
    { return NULL; }

  for(ii=0;ii<lNumVDs;ii++)
  {
      SmVertexDefinition *pVD = m_sVertexDefinitions[ii];
      SmTopology *pBrepTopoVD = pVD->GetSrcTopo( iWhichBrep) ;
      SmEdge   *pEdge = SM_CAST_PTR( SmEdge, pBrepTopoVD) ;
      if(pEdge == pEdgeED )
        { return pEdge; }
  }

  return NULL;

} // end SmProtoVertex::GetBrepEdge

/*******************************************************************//**
PURPOSE: Resolve this ProtoVertex.

USAGE NOTES---- 
   If this ProtoVertex has more than one VertexDefinition, then they will
   be examined and combined when appropriate.  The result of this method
   will be that each remaining VertexDefinition should be imprinted as an
   SmVertex into each Brep.

cbi from mail_ideas.160501:
If Resolve() sees different topology in two VertexDefs, it will see
whether one can be moved to the other.  In this case, it can move from
the Face to the Edge if the Face and Edge are connected, and it's
close enough.  I imagine any such movement would always be from higher
to lower dimensional topology.  But in the case of unstitched, lamina Edges,
we can't move one to the other, and so must create two distinct VertexDefs. 

***********************************************************************/
SmStatus SmProtoVertex::Resolve()
{
// ---------    cbi comment:    -----------
//cbiNew Bug 9: It finds one PV (-1.799876  -1.006614  0.000000) that's kind of spurious,
//cbiNew  close to the ends of near-tangent crv.  --> It ends up with no PE's connected. <--
//cbiNew  If we just leave it out, Bug 9 works.
//cbiNew  We do that by getting rid of its VD's.
//cbiNewer (Aug 2019): Can't do that. prog_test 1st tests are wire & solid Breps.
//cbiNewer A Wire-Face intersection produces PV's with no PE's.
//cbiNewer So Bug 9 will have to do something else.
//
//  ULONG lNumPEs = this->GetProtoEdges().GetSize();
//  if(lNumPEs < 1 )  //cbi just for Bug 9.
//  {
//      this->ClearVertexDefinitions();
//      return SM_SUCCESS;
//  }

  // locals
  ULONG ii, jj ;
  ULONG                lNumVDs = m_sVertexDefinitions.GetSize() ;
  SmVertexDefinition * pVD1    = NULL ; 
  SmVertexDefinition * pVD2    = NULL ;

  int iRetStatus = 0;  //  1  : DeleteMe.
                       // cbi : The first of these return statuses needed is DeleteMe.  If we need more, make an enum.
                       // cbi : If we don't need any more, then maybe make it a Boolean.

  for(ii=0;ii<lNumVDs;ii++)
  {
      pVD1 = m_sVertexDefinitions[ii];
      pVD1->Resolve( iRetStatus) ;
      if(iRetStatus == 1 )
      {
          // Don't use RemoveVertexDefinition(), that removes it from the array.
          delete pVD1; pVD1 = NULL;
          m_sVertexDefinitions[ii] = NULL;
      }
  }

  m_sVertexDefinitions.CompressZeros();
  lNumVDs = m_sVertexDefinitions.GetSize();

  // No further checks for a single VertexDef.
  if(lNumVDs < 2 )
    { return SM_SUCCESS; }

  ULONG lDeleteWhich; // 0: neither; 1: 1st argument; 2: 2nd argument.

  // Pairwise:
  for(ii=0;ii<lNumVDs-1;ii++)
  {
      pVD1 = m_sVertexDefinitions[ii];
      if(pVD1 == NULL ) { continue; }

      for(jj=ii+1;jj<lNumVDs;jj++)
      {
          pVD2 = m_sVertexDefinitions[jj];
          if(pVD2 == NULL ) { continue; }

          m_pProtoTopoMgr->CanCombineVertexDefinitions( pVD1, pVD2, lDeleteWhich) ;

          if(lDeleteWhich == 1 )
          {
              // m_pProtoTopoMgr->CombineVertexDefinitions( pVD2, pVD1) ;

              // No, don't call RemoveV.D.(), that shrinks the array.
              // Set it to Null, then process after this double loop.
              // this->RemoveVertexDefinition( pVD1) ;
              delete pVD1; pVD1 = NULL;

              m_sVertexDefinitions[ii] = NULL;
              jj = lNumVDs + 1; // We're done with PV[ii].
          }
          else if(lDeleteWhich == 2 )
          {
              // m_pProtoTopoMgr->CombineVertexDefinitions( pVD1, pVD2) ;
              // this->RemoveVertexDefinition( pVD2) ;
              delete pVD2; pVD2 = NULL;

              m_sVertexDefinitions[jj] = NULL;
          }
      }
  }

  m_sVertexDefinitions.CompressZeros();

  return SM_SUCCESS;

} // end SmProtoVertex::Resolve

/*******************************************************************//**
PURPOSE: Glue SmVertices contained in two VertexDefinitions together.

USAGE NOTES---- 
   Glues pVD2 to pVD1: moves all info from pVD2 to pVD1,
   leaving pVD2 empty and ready to be deleted.

   Does not check geometry: if they're not coincident, you can get into trouble.
***********************************************************************/
SmStatus SmProtoTopologyManager::GlueVertexDefinitions( SmVertexDefinition *pVD1,
                                                        SmVertexDefinition *pVD2 )
{
  // If both created SmVertex's exist in Brep1, glue them.
  SmVertex *pBrep1Vtx1 = pVD1->GetFinalVertex( 1) ;
  SmVertex *pBrep1Vtx2 = pVD2->GetFinalVertex( 1) ;
  SmBoolean bDeleted1 = FALSE, bDeleted2 = FALSE;

  if(   pBrep1Vtx1 != NULL
     && pBrep1Vtx2 != NULL
     && pBrep1Vtx1 != pBrep1Vtx2)
  {
      GetWhichBrep(1)->GlueVertices( pBrep1Vtx1, pBrep1Vtx2) ;
      pVD2->SetFinalVertex( 1, NULL) ;
      bDeleted1 = TRUE;
  }

  // Same for Brep2's vertices.
  SmVertex *pBrep2Vtx1 = pVD1->GetFinalVertex( 2) ;
  SmVertex *pBrep2Vtx2 = pVD2->GetFinalVertex( 2) ;
  if(   pBrep2Vtx1 != NULL
     && pBrep2Vtx2 != NULL
     && pBrep2Vtx1 != pBrep2Vtx2)
  {
      GetWhichBrep(2)->GlueVertices( pBrep2Vtx1, pBrep2Vtx2) ;
      pVD2->SetFinalVertex( 2, NULL) ;
      bDeleted2 = TRUE;
  }

  if(bDeleted1 || bDeleted2 ) // If either was deleted, don't relate.
  {
    if(m_pTI->IsMatedPair( pBrep1Vtx2, pBrep2Vtx2 ) )
      { SE( m_pTI->RemoveRelationship( pBrep1Vtx2, pBrep2Vtx2 )); }
  }

  return SM_SUCCESS;

} // end SmProtoVertex::GlueVertexDefinitions

/*******************************************************************//**
PURPOSE: add input VertexDefinition to ProtoVertex->Dinition list

USAGE NOTES---- 
***********************************************************************/
SmStatus SmProtoVertex::AddVertexDefinition
 (SmVertexDefinition * pNewVD)  // in : VertexDefinition to add to ProtoVertex->Definition list
{
  // add VertexDef to ProtoVertex list. Set PrototDef/ProtoVertex Owner/member pointers 
    m_sVertexDefinitions.Add( pNewVD) ;
    pNewVD->SetProtoVertex( this) ;
  
  // all done
    return SM_SUCCESS;

} // end SmProtoVertex::AddVertexDefinition

/*******************************************************************//**
PURPOSE: Find or Create a VertexDefinition for input PtClass pair

USAGE NOTES---- 
   The VertexDef is defined by two SmPointClassifications.
***********************************************************************/
SmStatus SmProtoVertex::AddVertexDefinition
 (SmPointClassification & rPC1,          // in : Brep1 PointClassification for Brep1/Brep2 XSectPt3d
  SmPointClassification & rPC2,          // in : Brep2 PointClassification for Brep1/Brep2 XSectPt3d
  SmVertexDefinition   *& rpNewVD,       // out: default [NULL]
  SmBoolean             * pbWasCreated ) // out: optional flag: TRUE = rpNewVD was created, NULL to ignore, default:[NULL]
                                         //                     FALSE= rpNewVD was found
{
  // set output
  if(pbWasCreated) { *pbWasCreated = FALSE ; }

  // locals
  ULONG ii ;
  SmTopology * pTopo1 = SM_CAST_PTR( SmTopology, rPC1.GetObject()) ; NER(pTopo1) ;
  SmTopology * pTopo2 = SM_CAST_PTR( SmTopology, rPC2.GetObject()) ; NER(pTopo2) ;

  // get VertexDefinition matching Tgt SrcTopo objs or NULL for none
  SmVertexDefinition *pVD = this->FindMatchingVertexDefinition(pTopo1, pTopo2) ;

  // no work - VertexDefintion with input Tgt SrcTopos already exists
  if(pVD != NULL) { // set output and return
      rpNewVD = pVD;
      return SM_SUCCESS;
  }

  // arrive here when a new VertexDefinition for these SrcTopo objs has to be created

  // create Vertex definition for input PointClassifications
  SER(m_pProtoTopoMgr->CreateVertexDefinition(rPC1, rPC2, pVD) );
  NER(pVD) ;

  // add VertexDef to ProtoVertex list. Set PrototDef/ProtoVertex Owner/member pointers
  pVD->SetProtoVertex(this) ;
  m_sVertexDefinitions.Add(pVD) ;

  // locals
  SmPoint3d sAvgPt(0,0,0) ;
  ULONG     lNumVDs = m_sVertexDefinitions.GetSize();

  // ProtoVertex postion = Avg(ProtoDef->Positions), note: this pos gets used if VDs or final Brep SmVertices get combined
  for(ii=0;ii<lNumVDs;ii++)
  {
      sAvgPt += m_sVertexDefinitions[ii]->GetPosition() ;
  }
  m_sPosition = sAvgPt / lNumVDs;

  // ProtoVertex ZoneTol3d
  SmZoneTol3d rPC1ObjTol = SmTol::GetZoneTol3d( rPC1.GetObject()) ;
  SmZoneTol3d rPC2ObjTol = SmTol::GetZoneTol3d( rPC2.GetObject()) ;
  m_dTol = smos_3Max( m_dTol, (double)rPC1ObjTol, (double)rPC2ObjTol) ;

  // set output
  rpNewVD = pVD;
  if(pbWasCreated) { *pbWasCreated = TRUE ; }

  // all done
  return SM_SUCCESS;

} // end SmProtoVertex::AddVertexDefinition

/*******************************************************************//**
PURPOSE: Find or Create a VertexDefinition for input PtClass pair and
         assign it an input Pos3d

USAGE NOTES---- 
   The VertexDef is defined by two SmTopology objects and a position.
***********************************************************************/
SmStatus SmProtoVertex::AddVertexDefinition
 (SmTopology          * pTopo1,        // in : Brep1 PointClassification for Brep1/Brep2 XSectPt3d
  SmTopology          * pTopo2,        // in : Brep2 PointClassification for Brep1/Brep2 XSectPt3d
  const SmPoint3d     & crPos,         // in : Pos3d to assign to new or found VertexDef
  SmVertexDefinition *& rpNewVD,       // out: default [NULL]
  SmBoolean           * pbWasCreated ) // out: optional flag: TRUE = rpNewVD was created, NULL to ignore, default:[NULL]
                                       //                     FALSE= rpNewVD was found
        //cbi should add arg: Gap.
{
  // Init outputs
  //  if(rpNewVD    != NULL ) { *rpNewVD     = NULL; }
  if(pbWasCreated != NULL ) { *pbWasCreated = FALSE; }

//  // Always, update tol.   //cbiTol: order dependence?!?
//  if(dTol > m_dTol )
//    { m_dTol = dTol; }

//  // Check for a match.
//  if(sm_IsMatchingPV( this, pTopo1, pTopo2, crPos, dTol ) )  //cbi not obsolete version.
//    { return SM_SUCCESS; }


  SmVertexDefinition *pVD = this->FindMatchingVertexDefinition( pTopo1, pTopo2) ;
  if(pVD != NULL )
  {
      rpNewVD = pVD;
      return SM_SUCCESS;
  }

  SER( m_pProtoTopoMgr->CreateVertexDefinition( pTopo1, pTopo2, crPos, pVD ));
  NER( pVD) ;

  m_sVertexDefinitions.Add( pVD) ;

  rpNewVD = pVD;

  // We have to update m_sPosition and m_dTol.
  // m_sPosition is the average of all VDs.
  ULONG lNumVDs = m_sVertexDefinitions.GetSize() - 1;  // (Number before adding.)
  SmPoint3d sNewPt( lNumVDs * m_sPosition  +  crPos) ;
  lNumVDs++;
  sNewPt /= lNumVDs;

      //cbi Update m_dTol.


//  if(rpNewVD      != NULL ) { *rpNewVD      = pVD; }
  if(pbWasCreated != NULL ) { *pbWasCreated = TRUE; }


#ifdef SM_DEBUG_CODE
  SmBoolean bDebugMe=FALSE;
  if(bDebugMe ) {
      smgfx_SetLook( 5,7, 1,0,0) ; this->Draw(); sm_GraphicsLoop();
      sm_GraphicsLoop();
  }
#endif // SM_DEBUG_CODE


  return SM_SUCCESS;  //cbi breakpoint

} // end SmProtoVertex::AddVertexDefinition

/*******************************************************************//**
PURPOSE: Combine two ProtoVertices: Add in the contents of another ProtoVertex into this ProtoVertex.

USAGE NOTES---- 
***********************************************************************/
void SmProtoVertex::CombineProtoVertex( SmProtoVertex *pOtherPV )
{
  ULONG ii ;

  // If they're the same, do nothing.
  if(pOtherPV == this )
    { return; }

  m_dTol = smos_Max( m_dTol, pOtherPV->m_dTol) ;

  // We will have to update m_sPosition: it will be the average of all VDs.
  // Start with what is here already.
  ULONG lNumVDs = this->m_sVertexDefinitions.GetSize();
  SmPoint3d sAvePos( lNumVDs * m_sPosition) ;

  // Don't worry about combining these here, that will happen in Resolve().
  for(ii=0;ii<pOtherPV->m_sVertexDefinitions.GetSize();ii++)
  {
      this->AddVertexDefinition( pOtherPV->m_sVertexDefinitions[ii]) ;
      sAvePos += pOtherPV->m_sPosition;
  }
  lNumVDs = this->m_sVertexDefinitions.GetSize();
  m_sPosition = sAvePos / lNumVDs;

  // Replace pOtherPV with 'this' in all of pOtherPV's ProtoEdges.
  for(ii=0;ii<pOtherPV->m_sProtoEdges.GetSize();ii++)
  {
      pOtherPV->m_sProtoEdges[ii]->ReplaceProtoVertex( pOtherPV, this) ;
  }

  m_sProtoEdges.Append( pOtherPV->m_sProtoEdges) ;

} // end SmProtoVertex::CombineProtoVertex



///////////////////////////////////////////
// SmProtoEdge methods
///////////////////////////////////////////

/*******************************************************************//**
PURPOSE: SmProtoEdge constructor

USAGE NOTES---- 
***********************************************************************/
SmProtoEdge::SmProtoEdge
 (SmProtoVertex          * pStartPV,  // in : ProtoVertex bounding the start of this ProtoEdge
  SmProtoVertex          * pEndPV,    // in : ProtoVertex bounding the end   of this ProtoEdge
  const SmExtent1d       & crIvl,     // in : Interval of SmEdgeDefinition XSectCrvs represented by this ProtoEdge
  SmProtoTopologyManager * pOwner )   // in : Back-pointer to owning SmProtoTopologyManager object
    : m_pXSectInterval( crIvl ), 
      m_pStartPV      ( pStartPV ),
      m_pEndPV        ( pEndPV   ),
      m_dTol          ( pOwner->GetTolerance() ),
      m_pProtoTopoMgr ( pOwner )
{
  // clear the internal arrays
  m_sEdgeDefs.ReSet() ;
  m_sOverlappingPEs.ReSet() ;
  m_sCoincPFs.ReSet() ;

  // context
  SetContext( pOwner->GetContext()) ;

} // end SmProtoEdge constructor

/*******************************************************************//**
PURPOSE: SmProtoEdge destructor

USAGE NOTES---- 
***********************************************************************/
SmProtoEdge::~SmProtoEdge()
{
  // Delete our EdgeDefs.
  // Note, don't delete start and end PVs.

  ULONG ii ;
  ULONG lNumEDs = m_sEdgeDefs.GetSize() ;

  // delete nested owned objects
  for(ii=0;ii<lNumEDs;ii++) { if(m_sEdgeDefs[ii]) delete m_sEdgeDefs[ii] ; m_sEdgeDefs[ii] = NULL ; }

  // Remove PE from its End PV->PE lists
  if(    m_pStartPV)              { m_pStartPV->RemoveProtoEdge(this) ; }
  if(    m_pEndPV 
     && (m_pEndPV != m_pStartPV)) { m_pEndPV->RemoveProtoEdge(this) ; }

  // clear contained objects
  m_pXSectInterval.Init() ;
  m_pStartPV = NULL ;       
  m_pEndPV   = NULL ; 
  
  m_sEdgeDefs.ReSet() ;      
  m_dTol = 0.0 ;           
  m_pProtoTopoMgr = NULL ; 
  
  m_sOverlappingPEs.ReSet() ;
  m_sCoincPFs.ReSet() ;

} // end SmProtoEdge destructor

/*******************************************************************//**
PURPOSE: Find a single intersection to use in place of conflicting
    overlapping ProtoEdges

USAGE NOTES---- 
***********************************************************************/
void SmProtoEdge::GetOverlappingPEGroup
 (SmTArray<SmProtoEdge *> & rOverlappingGroup) // out: Unique list of all overlapping PEs including cases where A overlaps B and C, while B and C are disjoint
 const
{
  // init output
  rOverlappingGroup.ReSet() ;

  // locals
  ULONG ii ;
  ULONG lLastIterPECnt = 0 ; 
  ULONG lThisIterPECnt = 0 ; 

  // start with this PE's overlapping PEs
  rOverlappingGroup.Append(((SmProtoEdge *)this)->GetOverlappingPEs()) ;

  // chase Overlapping PE associations until the whole group is found
  while(lThisIterPECnt < rOverlappingGroup.GetSize())
    {
      // update iter variables
      lLastIterPECnt = lThisIterPECnt ;
      lThisIterPECnt = rOverlappingGroup.GetSize() ;

      // for every newly added PE
      for(ii=lLastIterPECnt;ii<lThisIterPECnt;ii++)
        {
          SmProtoEdge *pPE = rOverlappingGroup[ii] ; 

          // append PE[ii]->OverlappingPEs not already in the group
          rOverlappingGroup.AppendUnique(pPE->m_sOverlappingPEs) ;
        } // end iter ii, every last iter PE adding nested PE->Overlapping PEs to the OverlapGroup

    } // end more PEs were added in the last iter check

  // all done
  
} // end SmProtoEdge::GetOverlappingPEGroup

/*******************************************************************//**
PURPOSE: Get the location of an end of this ProtoEdge.

USAGE NOTES---- 
   The input ProtoVertex should be either our StartPV or EndPV.
   If it is neither, this returns SM_ERR, and rPosition uninitialized.
  cbi: if both?  For now, just return Start.  S/B same anyway.
***********************************************************************/
SmStatus SmProtoEdge::GetEndPosition
 (SmProtoVertex * pWhichPV,   // in :   (pWhichPV == m_pStartPV) ? ProtoEdgeStart
                              //      : (pWhichPV == m_pStartPV) ?  ProtoEdge->End
                              //      : else return SM_ERR
  SmPoint3d     & rPosition ) // out: ProtoEdge XSectCrv3d end Pos3d
 const
{
  // init output
  rPosition.SetUninitialized();

  // return ProtoEdge End Pos bounded by pWhichPV
  if     (pWhichPV == m_pStartPV) { return this->GetEndPosition( TRUE, rPosition) ; }
  else if(pWhichPV == m_pEndPV)   { return this->GetEndPosition( FALSE, rPosition) ; }

  // arrive here when pWhichPV does not bound this ProtoEdge - return error
  return SM_ERR;

} // end SmProtoEdge::GetEndPosition

/*******************************************************************//**
PURPOSE: Get the location of an end of this ProtoEdge.

USAGE NOTES---- 
   If we have no EdgeDefinitions, return the position of our start or end ProtoVertex.
***********************************************************************/
SmStatus SmProtoEdge::GetEndPosition
 (SmBoolean   bAtStart,  // in : TRUE = ProtoEdgeStart, FALSE= ProtoEdgeEnd
  SmPoint3d & rPosition) // out: Avg(ProtoEdge->EdgeDefinition->XSectCrv3d end Pos3d values)
 const
{
  // init outpu
  rPosition.SetUninitialized();

  // locals
  ULONG ii ;
  ULONG lNumEDs = m_sEdgeDefs.GetSize() ;

  // when ProtoEdge has no EdgeDefinitions - return the attached ProtoVertex position
  if(lNumEDs == 0)
    {
      if(bAtStart) { rPosition = m_pStartPV->GetPosition(); }
      else         { rPosition = m_pEndPV->GetPosition(); }

      // all done
      return SM_SUCCESS;
    } // end no EdgeDefinitions check

  // arrive here when ProtoEdge has 1 or more DegeDefinitions
  //  cbi note: currently returns: average position of all EdgeDefs.

  // locals - for Position average
  SmPoint3d sPt;
  SmPoint3d sAvePt( 0,0,0) ;

  // for every EdgeDefinition
  for(ii=0;ii<lNumEDs;ii++)
    {
      // Get EdgeDefinition end points
      SmEdgeDefinition * pED = m_sEdgeDefs[ii];
      SER( pED->GetEndPosition(bAtStart,  // in : TRUE = ProtoEdge->XSectCrvStart, FALSE= ProtoEdge->XSectCrveEnd
                               sPt)) ;    // out: ProtoEdge XSectCrv3d end Pos3d

      // accumulate positions
      sAvePt += sPt;
    }

  // get average position
  rPosition = sAvePt / lNumEDs;

  // all done
  return SM_SUCCESS;

} // end SmProtoEdge::GetEndPosition

/*******************************************************************//**
PURPOSE: Get the Capture Distance for this ProtoEdge.

USAGE NOTES---- 
***********************************************************************/
double SmProtoEdge::GetCaptureDistance() const
{
  return m_pProtoTopoMgr->GetCaptureDistance(); 
} // end SmProtoEdge::GetCaptureDistance

/*******************************************************************//**
PURPOSE: Relate this ProtoEdge to a ProtoFace remembering that
         this ProtoEdge is part of the trim boundary of the coincident ProtoFace

USAGE NOTES---- 
***********************************************************************/
void SmProtoEdge::RelateCoincidentPF
 ( SmProtoFace * pCoincPF) 
{ 
  m_sCoincPFs.AddUnique( pCoincPF) ; 
  pCoincPF->GetCoincProtoEdges()->AddUnique(this) ;

} // end SmProtoEdge::RelateCoincidentPF

/*******************************************************************//**
PURPOSE: Evaluate the location at a parameter along this ProtoEdge.

USAGE NOTES---- 
   Just the average of all of our EdgeDefs.
***********************************************************************/
SmStatus SmProtoEdge::Evaluate( double dParam, SmPoint3d * pPos, SmVector3d * pTan )
{
  ULONG ii, lNumEDs = m_sEdgeDefs.GetSize();

  if(lNumEDs == 0 )
    { SER( SM_ERR_INVALID_INPUT) ; }

  SmPoint3d  sThisPt,  sSumPts ( 0,0,0) ;
  SmVector3d sThisTan, sSumTans( 0,0,0) ;
  SmPoint3d  *pThisPtPtr  = ( pPos != NULL ) ? &sThisPt  : NULL;
  SmVector3d *pThisTanPtr = ( pTan != NULL ) ? &sThisTan : NULL;

  for(ii=0;ii<lNumEDs;ii++)
  {
      SER( m_sEdgeDefs[ii]->Evaluate( dParam, pThisPtPtr, pThisTanPtr )) ;
      if(pPos ) { sSumPts  += *pThisPtPtr ; }
      if(pTan ) { sSumTans += *pThisTanPtr; }
  }

  if(pPos ) { *pPos = sSumPts  / lNumEDs; }
  if(pTan ) { *pTan = sSumTans / lNumEDs; }

  return SM_SUCCESS;

} // end SmProtoEdge::Evaluate

/*******************************************************************//**
PURPOSE: Evaluate the midpoint of this ProtoEdge.

USAGE NOTES---- Convenience method, helps clarify code.
***********************************************************************/
SmStatus SmProtoEdge::EvaluateMid( SmPoint3d  * pPos,   // out, optional. Default: NULL
                                   SmVector3d * pTan )  // out, optional. Default: NULL
{
  double dParam = m_pXSectInterval.GetMid();

  return this->Evaluate( dParam, pPos, pTan) ;

} // end SmProtoEdge::EvaluateMid

/*******************************************************************//**
PURPOSE: return true when XSecting TgtInterval has same Shape as any one 
         of thisProtoVertex->EdgeDefinitions. Set bIsIdentical = TRUE 
         when the same shape EdgeDef also has the same SrcTopos as the input.

USAGE NOTES----
  with
   Shape  = TRUE when XSecting Crv Shapes are the same  (for any i, PV->ED[i] runs between TgtPVs and PV->ED[i]->Crv3d is within CaptureDistance of TgtCurve3d)
   Source = TRUE when XSecting Crv Sources are the same (for any i, PV->ED[i]->SrcTopos and the TgtClassification->CrvInterval->MidPt objects are the same)

  OUTPUT:
    bIsIdentical = Source ;
    return(Shape) ;

  Three output cases are possible:
   case1: return = TRUE, bIsIdentical = TRUE  : PE with DE already exists for this face pair XSect case : no work for user
   case2: return = TRUE, bIsIdentical = FALSE : PE without DE exists for this face pair XSect case      : caller will add a DE to this PE
   case3: return = FALSE                      : No PE exists for this face pair XSect case              : caller will add a PE and a DE to the ProtoManager
***********************************************************************/
SmBoolean SmProtoEdge::IsSameProtoEdge              // rtn: TRUE = XSecting Interval has same Shape as a contained EdgeDefinition
 ( SmProtoVertex               * pTgtStartPV,       // in : Tgt Start ProtoVertex
   SmProtoVertex               * pTgtEndPV,         // in : Tgt End ProtoVertex
   const SmPointClassification & crMidPC1,          // in : face1->CrvClass1->TgtCrvInterval->Mid PtClassification
   const SmPointClassification & crMidPC2,          // in : face2->CrvClass2->TgtCrvInterval->Mid PtClassification
   const SmCurve               * pTgtCurve3d,       // in : shared XSectCurve3d being classified against face1/face2 pair
   const SmExtent1d            & rTgtCurveDomain,   // in : XSectCrv domain for this TgtCrvInterval
   SmBoolean                   & rbIsIdentical)     // out: TRUE = the same-shape EdgeDefinition also shares the same SrcTopos as the input XSect Interval
  const                                             //      FALSe= the same-shape EdgeDefinition comes from a different SrcTopo XSection
{
  // Init output
  rbIsIdentical = FALSE;

  // locals
  ULONG ii ;
  ULONG     lNumEDs             = m_sEdgeDefs.GetSize();
  SmBoolean bFoundSameShapeOnly = FALSE;

  // does this PE run from StartToEnd, EndToStart, or not between Start and End
  SmBoolean bForward =   (pTgtStartPV == m_pStartPV && pTgtEndPV == m_pEndPV  ) ? TRUE     // PE runs from TgtStart to TgtEnd
                       : (pTgtStartPV == m_pEndPV   && pTgtEndPV == m_pStartPV) ? FALSE    // PE runs from TgtEnd to TgtStart
                       :                                                          UNSURE ; // PE does not run between TgtPVs

  // low work - PEs not running between TgtPVs are not Same
  if(bForward == UNSURE)
    { return FALSE; }

  // for every EdgeDefinition - Check for a match
  for(ii=0;ii<lNumEDs;ii++)
  {
      SmEdgeDefinition *pED = m_sEdgeDefs[ii];

      // Check for a match
      if(pED->IsSameEdgeDefinition(pTgtStartPV,           // in : Tgt Start ProtoVertex
                                   pTgtEndPV,             // in : Tgt End   ProtoVertex
                                   crMidPC1,              // in : MidPt Classification for face1->CrvClass->TgtCrvInterval
                                   crMidPC2,              // in : MidPt Classification for face2->CrvClass->TgtCrvInterval
                                   pTgtCurve3d,           // in : shared XSectCurve3d being classified against face1/face2 pair
                                   rTgtCurveDomain,       // in : TgtCrvInterval domain
                                   rbIsIdentical ) )      // out: TRUE = this and TgtXSectCrv->Interval->MidPt classifications
        {                                                 //             map to the same pair of Brep1 and Brep2 Src topology objects.
          // when this ED is same shape (within CaptureDistance) of TgtCurve3d and shares the same XSecting SrcTopos as the input
          if(rbIsIdentical)
            { 
              // all done - Found an existing ED identical to the Tgt input XSect result data
              return TRUE; 
            }

          // remember - found a ED with matching shape that has different XSecting SrcTopos
          bFoundSameShapeOnly = TRUE;
  }
    } // end iter every EdgeDefinition looking for matching shapes and SrcTopos

  // arrive here when no ED was found with same shape and SrcTopos as the Input TgtXSect descrition.
  //   bFoundSameShapeOnly:[TRUE ] = some EdgeDefinition is within CaptureDistance of the TgtCurve3d, but does not share the same XSecting SrcTopos
  //                       [FALSE] = no EdgeDefinition was found within CaptureDistance of TgtCurve3d

  // check output
  SM_ASSERT_MSG(rbIsIdentical == FALSE, _T("SmProtoEdge::IsSameProtoEdge: logic for setting rbIsIdentical is off - needs debug")) ;  

  // all done - return TRUE when a matching shape ED was found, else return FALSE
  return(bFoundSameShapeOnly) ; 

} // end SmProtoEdge::IsSameProtoEdge

/*******************************************************************//**
PURPOSE: If this ProtoEdge points to pOldPV ProtoVertex, replace it with pNewPV.

USAGE NOTES---- 
***********************************************************************/
void SmProtoEdge::ReplaceProtoVertex
 (SmProtoVertex * pOldPV,  // in : ProtoEdge->ProtoVertex to replace
  SmProtoVertex * pNewPV)  // in : new ProtoVertex to replace the old
{
  if(m_pStartPV == pOldPV) { m_pStartPV = pNewPV; }
  if(m_pEndPV   == pOldPV) { m_pEndPV = pNewPV; }

} // end SmProtoEdge::ReplaceProtoVertex

/*******************************************************************//**
PURPOSE: Split a SmProtoEdge at the given parameter creating one
         new ProtoVertex, one new ProtoEdge (upperSplitChild) and
         reusing this ProtoEdge (lowerSplitChild)

USAGE NOTES---- 
   1. If dT is not interior to our curve (to 3d tolerance), does not split.

   2. After a successful split, 'this' will be the lower part of the domain
      and rpNewPE will be the higher part.

   3. after split, 
       - all ProtoEdge->EdgeDefitions are also split creating new VertexDefinitions and EdgeDefinitions
       - all topological connection relationships between and among the new and old ProtoTopology and TopoDefinition objs are updated
***********************************************************************/
SmStatus SmProtoEdge::Split
 (double           dT,      // in : Tgt Split param, checked to be interior value
  SmProtoVertex *& rpNewPV, // out:  When dT is interior, new ProtoVertex at Split point
  SmProtoEdge   *& rpNewPE) // out:  When dT is interior, new Upper Child Split ProtoEdge:[dT, IvlMax]
                            // note: When dT is interior, this PE reused for Lower Child split ProtoEdge:[IvlMin, dT]
{
  // init output
  rpNewPV = NULL ;
  rpNewPE = NULL ;

  // no work - dT is not interior to this ProtoEdge.
  //   The first check just tests whether it's exterior or on the boundary in param space. 
  //   The following 3d tests deal with tolerances, so this check doesn't have to.
  if(m_pXSectInterval.ContainsValue(dT, -SM_EFF_ZERO) == FALSE)
    { return SM_SUCCESS; }   //cbiTol breakpoint: how does this happen?

  // locals
  ULONG ii ;
  SmPoint3d sCrvPt, sEndPt, sStartPt ;
  this->Evaluate( dT, &sCrvPt) ;
  SER( this->GetEndPosition(TRUE, sStartPt)) ; // TRUE  = get ProtoEdge->EdgeDefinitions->StartPos3d average
  SER( this->GetEndPosition(FALSE, sEndPt)) ;  // FALSE = get ProtoEdge->EdgeDefinitions->EndPos3d average

  // no work - Split Pt is within tol of ProtoEdge end point
  //    Use distance to end of this ProtoEdge, not distance to our ProtoVertices,
  //    because there could be tolerance issues with that.
  if(sCrvPt.DistanceBetween( sStartPt ) < m_dTol) { return SM_SUCCESS; }
  if(sCrvPt.DistanceBetween( sEndPt )   < m_dTol) { return SM_SUCCESS; }

  // Split ourself first, then all of our EdgeDefs.
  // That way we'll have the new ProtoEdge to connect the new EdgeDefs to.

  // locals
  SmProtoVertex * pNewPV         = NULL;
  SmBoolean       bWasCreatedNew = FALSE;
  m_pProtoTopoMgr->FindOrCreateProtoVertex(sCrvPt,            // in : TgtPosition
                                           pNewPV,            // out: either an existing ProtoVertex within Tol of crPos, or a new one created at crPos
                                           &bWasCreatedNew) ; // out: optional output, TRUE =rpNewPV is a new ProtoVertex
                                                              //                       FALSE=rpNewPV is an existing one, 
                                                              //      NULL to ignore, default:[NULL]
  NER( pNewPV) ;

  // For cleanup object: don't do it if it was 'found' (vs. created).
  SmProtoVertex * pCleanPV = ( bWasCreatedNew ) ? pNewPV : NULL ;
  SmObjDelete     sCleanPV( pCleanPV) ;

  // create new upper Split child ProtoEdge
  SmProtoEdge *pNewPE = new( m_pProtoTopoMgr ) SmProtoEdge( *this) ; 
  NER( pNewPE) ;

  // make pNewPV persistent
  sCleanPV.Clear() ;

  // set Split child ProtoEdge intervals
  this  ->m_pXSectInterval.SetMinMax( m_pXSectInterval.GetMin(), dT) ;
  pNewPE->m_pXSectInterval.SetMinMax( dT, m_pXSectInterval.GetMax()) ;

  // add split child ProtoEdges to pNewPV connected ProtoEdge list
  pNewPV->AddProtoEdge( this) ;
  pNewPV->AddProtoEdge( pNewPE) ;

  // set split child ProtoEdge End ProtoVertices
  pNewPE->m_pEndPV   = this->m_pEndPV;
  pNewPE->m_pStartPV = pNewPV;
  this  ->m_pEndPV   = pNewPV;
  // already done: this  ->m_pStartPV = this->m_pStartPV ; 

  // edit upper split child -> EndProtoVertex -> ProtoEdge list to remove the lowerSplitChild ProtoEdge and to add the upperSplitChildEdge
  pNewPE->m_pEndPV->AddProtoEdge   ( pNewPE) ;
  pNewPE->m_pEndPV->RemoveProtoEdge( this  ) ;

  // Add the new ProtoVertex and ProtoEdge to the ProtoTopoMgr ProtoTopo lists
  // m_pProtoTopoMgr->Add( pNewPV) ; //cbi Already done, in CreateProtoVertex(); cbi clean this up.
  m_pProtoTopoMgr->AddProtoEdge( pNewPE) ;

  // set outout
  rpNewPV = pNewPV;
  rpNewPE = pNewPE;

  // Now split all of the original ProtoEdge->EdgeDefs.

  // locals
  SmEdgeDefinition   * pED ;
  SmEdgeDefinition   * pNewED ;
  SmVertexDefinition * pNewVD ;
  ULONG                lNumEDs = m_sEdgeDefs.GetSize();

  // clear new UpperSplitChild->EdgeDefinitions
  pNewPE->ClearEdgeDefinitions();
  //pNewPV->ClearVertexDefinitions();

  // for every original ProtoEdge->EdgeDef
  for(ii=0;ii<lNumEDs;ii++)
    {
      pED = m_sEdgeDefs[ii] ;

      // split the EdgeDefinition
      pED->Split(dT,       // in : Split parameter on ProtoEdge to be used to split this EdgeDefinition
                 pNewPV,   // in : pMidPV = ProtoVertex at ProtoEdge split point
                 pNewPE,   // in : pTopPE = new ProtoEdge for ProtoEdge UpperSplitChild
                 pNewVD,   // out: new VertexDefinition at Split point. Gets added to pMidPV
                 pNewED) ; // out: new Split UpperChild EdgeDefinition. Gets added to pTopPE
                           // note: this EdgeDefinition gets reused as Split LowerChild EdgeDefinition
    } // iter every Original ProtoEdge->EdgeDefinition, splitting

  // next: relate newPE to this_PE->CoincPFs when this_PE was split
  if(pNewPE != NULL)
    {
      // for every ProtoFace that remembers that this_PE is in its boundary
      for(ii=0;ii<m_sCoincPFs.GetSize();ii++)
        {
          SmProtoFace * pPF = m_sCoincPFs[ii] ;
      
          // relate NewPE to ProtoFace->CoincPEs 
          pNewPE->RelateCoincidentPF( pPF ) ; 

        } // end iter every ProtoFace bounded by this_PE
    } // end this_PE was split check

  // all done
  return SM_SUCCESS;

} // end SmProtoEdge::Split

/*******************************************************************//**
PURPOSE: Find an existing EdgeDefinition in this ProtoEdge that 
         matches a proposed EdgeDefinition defined by its
         XSectCrv's Brep1 and Brep2 MidInterval classifications.

USAGE NOTES---- 
   Must have the same ends, be on the same or coincident curves,
   and be defined on the same Topology objects in both Breps.
***********************************************************************/
SmEdgeDefinition * SmProtoEdge::FindMatchingEdgeDefinition
 (const SmCurveInterval & crCI1,  // in : proposed EdgeDefinitions Brep1 XSectCrv's midInterval classification
  const SmCurveInterval & crCI2,  // in : proposed EdgeDefinitions Brep2 XSectCrv's midInterval classification
  SmBoolean             & rbRev ) // in : [not used] //cbi probably deal with this.
{
  // init output
  SmEdgeDefinition * pRetED = NULL;
  rbRev                     = FALSE;    //cbi will have to deal with this.

  // locals
  ULONG ii ;
  SmBoolean       bIsIdentical = FALSE ;
  const SmCurve * pXSectCrv    = crCI1.GetCurve();
  SmExtent1d      sIvl         = crCI1.GetInterval();
  ULONG           lNumEDs      = m_sEdgeDefs.GetSize();

  // for every EdgeDefinition
  for(ii=0;ii<lNumEDs;ii++)
    {
      SmEdgeDefinition *pThisED = m_sEdgeDefs[ii];

      // when this and iter EdgeDefinitions are the same - geometrically and topologically
      if(pThisED->IsSameEdgeDefinition(m_pStartPV,    // in : proposed EdgeDefinition's start ProtoVertex 
                                         m_pEndPV,      // in : proposed EdgeDefinition's end   ProtoVertex
                                         crCI1.m_vMid,  // in : XSectCrv Brep1 Interval MidPoint classification for proposed EdgeDefinition
                                         crCI2.m_vMid,  // in : XSectCrv Brep2 Interval MidPoint classificaiton for proposed EdgeDefinition
                                         pXSectCrv,     // in : XSectCurve3d
                                         sIvl,          // in : XSectCurve3d domain interval
                                         bIsIdentical)) // out: TRUE = this and proposed EdgeDefinition XSectCrv Interval MidPt classifications
                                                        //             map to the same pair of Brep1 and Brep2 topology objects.
        {
          // when the EdgeDefinitions are not only the same but also come from the same pair of intersecting Brep Topology objects
          if(bIsIdentical)
            {
              // found the Same ProtoEdge - set output 
              pRetED = pThisED;
              break;
            } // end found an identical ProtoEdge check
        } // end found an equivalent ProtoEdge Check
    } // end iter every EdgeDefinition looking for one that matches the proposed EdgeDefinition defined by the input SmCurveInterval classifications

  // all done - return the found identical EdgeDefinition or NULL for none
  return pRetED;

}  // end SmProtoEdge::FindMatchingEdgeDefinition

/*******************************************************************//**
PURPOSE: Add EdgeDef to this ProtoEdge's EdgeDef list

USAGE NOTES---- 
   The domains of all EdgeDefinitions must match the domain of this ProtoEdge.
   If the given interval does not match ours, then:
    If we have no EdgeDef's stored already, update our m_pXSectInterval to the new one,
    else reparameterize the new interval.

   If the given EdgeDef is already owned by a different ProtoEdge, it will be removed from its owner.
***********************************************************************/
SmStatus SmProtoEdge::AddEdgeDefinition
 (SmEdgeDefinition * pEdgeDef)  // in : TgtEdgeDefinition to add to this ProtoEdge
{
  // locals
  SmExtent1d    sIvl      = pEdgeDef->GetXSectInterval();
  SmProtoEdge * pOwningPE = pEdgeDef->GetProtoEdge();

  // 1st edge definition branch
  if(m_sEdgeDefs.GetSize() == 0)
    {
      // set ProtoEdge->Interval = EdgeDefinition->Interval 
      m_pXSectInterval = sIvl;
    }
  else // add additional EdgeDefinitions branch
    {
      SmVector3d sProtoEdgeTan, sEdgeDefTan;
      this     ->EvaluateMid( NULL, &sProtoEdgeTan) ;
      pEdgeDef ->EvaluateMid( NULL, &sEdgeDefTan) ;

      // remember when ProtoEdge->Tan is opposite to EdgeDef->Tan 
      SmBoolean bReverse = ( sProtoEdgeTan.Dot( sEdgeDefTan ) < 0.0) ;

      // when EdgeDef parameter opposes ProtoEdges or Ivls don't match - reparameterize EdgeDef
      if(bReverse  ||  ! ( sIvl == m_pXSectInterval ) )
        {
          pEdgeDef->Reparameterize( this->m_pXSectInterval, bReverse) ;
        } // end need to reparameterize EdgeDef Check
    } // end add additional EdgeDef to ProtoEdge Branch

  // EdgeDef->Owner is not ProtoEdge - remove EdgeDef from its owner.
  if(   pOwningPE != NULL 
     && pOwningPE != this)
    {
      // remove EdgeDef from its previous owner
      pOwningPE->RemoveEdgeDefinition( pEdgeDef) ;
    }

  // add EdgeDef to this ProtoEdge
  m_sEdgeDefs.Add( pEdgeDef) ;

  // set EdgeDef->Owner = This ProtoEdge
  pEdgeDef->SetProtoEdge( this) ;

  // all done
  return SM_SUCCESS ;

} // end SmProtoEdge::AddEdgeDefinition

/*******************************************************************//**
PURPOSE: Add more definition info to a ProtoEdge.

USAGE NOTES---- 
   The curve intervals must match.  If the given interval does not match ours, then:
    If we have no EdgeDef's stored already, update our m_pXSectInterval to the new one,
    else reparameterize the new interval, possibly reversing it.
   That is done in this->AddEdgeDefinition( pNewED ).
***********************************************************************/
SmStatus SmProtoEdge::AddEdgeDefinition
 (const SmCurveInterval &crCI1, // in : proposed EdgeDefinitions Brep1 XSectCrv's midInterval classification
  const SmCurveInterval &crCI2) // in : proposed EdgeDefinitions Brep2 XSectCrv's midInterval classification
{
  // locals
  SmBoolean          bRev;
  SmEdgeDefinition * pMatchingED = this->FindMatchingEdgeDefinition( crCI1, crCI2, bRev) ;
  
  // no work - identical EdgeDefinition already exists
  if(pMatchingED != NULL)
    { return SM_SUCCESS; }

// DD2: don't need start and end PVs for this.
//  SmProtoVertex *pStartPV = NULL, *pEndPV = NULL;
// 
//  SER( m_pProtoTopoMgr->FindOrCreateProtoVertex( crCI1.m_vStart, crCI2.m_vStart, pStartPV ));
//  NER( pStartPV) ;
//  SmTopology *pTopo1 = SM_CAST_PTR( SmTopology, crCI1.m_vStart.GetObject()) ;
//  SmTopology *pTopo2 = SM_CAST_PTR( SmTopology, crCI2.m_vStart.GetObject()) ;
//  SmVertexDefinition *pStartVD = pStartPV->FindMatchingVertexDefinition( pTopo1, pTopo2) ;
//  NER( pStartVD) ;
// 
//  SER( m_pProtoTopoMgr->FindOrCreateProtoVertex( crCI1.m_vEnd,   crCI2.m_vEnd,   pEndPV   ));
//  NER( pEndPV) ;
//  pTopo1 = SM_CAST_PTR( SmTopology, crCI1.m_vEnd.GetObject()) ;
//  pTopo2 = SM_CAST_PTR( SmTopology, crCI2.m_vEnd.GetObject()) ;
//  SmVertexDefinition *pEndVD = pEndPV->FindMatchingVertexDefinition( pTopo1, pTopo2) ;
//  NER( pEndVD) ;

  // locals
  SmExtent1d   sIvl(crCI1.m_vInterval) ;
  SmCurve    * pXSectCurve = NULL;
  SmTopology * pTopo1      = SM_CAST_PTR( SmTopology, crCI1.m_vMid.GetObject()) ;
  SmTopology * pTopo2      = SM_CAST_PTR( SmTopology, crCI2.m_vMid.GetObject()) ;
  double       dTol        = smos_Max(SmTol::GetSrcZoneTol3d(&crCI1), 
                                      SmTol::GetSrcZoneTol3d(&crCI2)) ;

  // copy and trim the XSectCurve3d to place in new EdgeDefinition
  SER(crCI1.GetCurve()->Copy( *( m_pProtoTopoMgr->GetContext() ), pXSectCurve)) ;
  NER(pXSectCurve) ;
  pXSectCurve->Trim( sIvl) ; // may snap sIvl by tol to existing knots  // Things can get confusing if the curve is longer than the domain.

//cbi just out of curiosity:  Not Hit in prog_test!
//double cbiTol = smos_Max( SmTol::GetObjZoneTol3d( &crCI1 ), SmTol::GetObjZoneTol3d( &crCI2 )) ;

  // create new EdgeDefinition
  SmEdgeDefinition *pNewED = new( m_pProtoTopoMgr ) SmEdgeDefinition(pXSectCurve, // in : Intersection 3d curve
                                                                     sIvl,        // in : XSectCurve3d domain interval
                                                                     pTopo1,      // in : source intersecting pBrep1 topology object (an SmFace or SmEdge)
                                                                     pTopo2,      // in : source intersecting pBrep1 topology object (an SmFace or SmEdge)
                                                                     dTol,        // in : probably the SmXSectTol3d value used by the intersector to generate this intersection curve
                                                                     NULL,        // in : associated UVTrimCurve on a SmFace in pBrep1, or NULL  //cbi double-check these.
                                                                     NULL,        // in : associated UVTrimCurve on a SmFace in pBrep2, or NULL  //cbi double-check these.
                                                                     this) ;      // in : Back pointer to the SmProtoEdge that owns this SmEdgeDefinition
  this->AddEdgeDefinition( pNewED) ;

  return SM_SUCCESS;

} // end SmProtoEdge::AddEdgeDefinition

/*******************************************************************//**
PURPOSE: Combine two ProtoEdges: Add in the contents of another ProtoEdge into this ProtoEdge.

USAGE NOTES---- 
   Moves the EdgeDefinitions from pOtherPE into this PE.
***********************************************************************/
void SmProtoEdge::CombineProtoEdge( SmProtoEdge *pOtherPE )
{
  ULONG ii ;

  // If they're the same, do nothing.
  if(pOtherPE == this )
    { return; }

  m_dTol = smos_Max( m_dTol, pOtherPE->m_dTol) ;

  pOtherPE->m_pProtoTopoMgr = this->m_pProtoTopoMgr; // Presumably the same anyway...

  m_sOverlappingPEs.Append( pOtherPE->m_sOverlappingPEs) ;

  SmProtoVertex *pThisStartPV  = this->GetStartProtoVertex();
  SmProtoVertex *pOtherStartPV = pOtherPE->GetStartProtoVertex();
  if(pOtherStartPV != pThisStartPV )
  {
      pOtherPE->SetStartProtoVertex( pThisStartPV) ;

      pOtherStartPV->RemoveProtoEdge( pOtherPE) ;
      pThisStartPV->AddProtoEdge( this) ;
  }

  SmProtoVertex *pThisEndPV  = this->GetEndProtoVertex();
  SmProtoVertex *pOtherEndPV = pOtherPE->GetEndProtoVertex();
  if(pOtherEndPV != pThisEndPV )
  {
      pOtherPE->SetEndProtoVertex( pThisEndPV) ;

      pOtherEndPV->RemoveProtoEdge( pOtherPE) ;
      pThisEndPV->AddProtoEdge( this) ;
  }

  // Add in other PE's EdgeDefinitions, and remove them from pOtherPE.
  // Don't worry about combining these here, that will happen in Resolve().
  for(ii=0;ii<pOtherPE->m_sEdgeDefs.GetSize();ii++)
  {
      SmEdgeDefinition *pED = pOtherPE->m_sEdgeDefs[ii];
      this->AddEdgeDefinition( pED) ;
      pOtherPE->RemoveEdgeDefinition( pED) ;
  }

} // end SmProtoEdge::CombineProtoEdge

/*******************************************************************//**
PURPOSE: Remove input EdgeDefinition from this ProtoEdge's EdgeDef list.

USAGE NOTES---- 
   Returns SM_ERR when pEdgeDef is not in this PE->m_sEdgeDefs list else returns SM_SUCCESS
***********************************************************************/
SmStatus SmProtoEdge::RemoveEdgeDefinition
 (SmEdgeDefinition * pED)  // in : TgtEdgeDef to remove from this ProtoEdge
{
  ULONG idx ;

  // when pED is in this ProtoEdge->EdgeDef list - remove it
  if(m_sEdgeDefs.FindElement(pED, idx)) { m_sEdgeDefs.RemoveAt(idx) ; }
  else                                  { return SM_ERR; }

  // all done
  return SM_SUCCESS;

} // end SmProtoEdge::RemoveEdgeDefinition

/*******************************************************************//**
PURPOSE: Remove a ProtoVertex from this ProtoEdge.

USAGE NOTES---- 
   Returns Error if pPV is not in our list.
   Does not delete the ProtoVertex.
***********************************************************************/
SmStatus SmProtoEdge::RemoveProtoVertex( SmProtoVertex *pPV )
{
  SmBoolean bDeleted = FALSE;
  if(pPV == m_pStartPV )
  {
      m_pStartPV = NULL;
      bDeleted   = TRUE;
  }
  if(pPV == m_pEndPV )
  {
      m_pEndPV = NULL;
      bDeleted = TRUE;
  }

  return ( bDeleted ) ? SM_SUCCESS : SM_ERR;

} // end SmProtoEdge::RemoveProtoEdge

/*******************************************************************//**
PURPOSE: Resolve this ProtoEdge.

USAGE NOTES---- 
***********************************************************************/
SmStatus SmProtoEdge::Resolve()
{
  ULONG ii, jj, lNumEDs = m_sEdgeDefs.GetSize();
  SmEdgeDefinition *pED1, *pED2;

  if(lNumEDs < 1)
    { return SM_SUCCESS; }

//  for(ii=0;ii<lNumEDs;ii++)
//  {
//      pED1 = m_sEdgeDefs[ii];
//      pED1->Resolve();
//  }

  // Check for degenerate ProtoEdge: Start and End on same ProtoVertex,
  // and very short.
  if(m_pStartPV == m_pEndPV)
    {
      SmCurve *pIntCurve = m_sEdgeDefs[0]->GetXSectCurve3d();
      double dLen = pIntCurve->ApproximateLength( m_pXSectInterval, 5) ;

      if(dLen < 2*m_dTol)
        {
          // Delete ourself?  Actually, we can just remove all of our EdgeDefinitions.
          // If the caller chooses, they can check if we come back empty, and delete if so.
          for(ii=0;ii<lNumEDs;ii++)
            {
              pED1 = m_sEdgeDefs[ii];
              pED1->Destruct(); // Its destructor does not clear out geometry because that normally is consumed.
              delete( pED1) ;  pED1 = NULL;
              m_sEdgeDefs[ii] = NULL;
            }
          m_sEdgeDefs.ReSet();
        }
      else
        {
          // Our IntCurve is longer than tolerance.  Is it closed?
          // If not, maybe we should make an Edge out of it, with two distinct Vertices.
          // We would do that by adding a VertexDef to the ProtoVertex.
          //   (Or possibly split the PV into two?  Adding a VD would be more in keeping w/ philosophy.)
          // Check start & end points.
          SmPoint3d sPt0, sPt1;
          pIntCurve->GetEnds(sPt0, sPt1) ;
          double dDist = sPt0.DistanceBetween( sPt1) ;
          if(dDist > 2*m_dTol)
            {
              // Check whether there are two VertexDefs, one for each end of the int curves.
              SmBoolean bAddVtxDef = FALSE;
              // We'll need this later:
              SmVertexDefinition *pVD0 = m_pStartPV->GetClosestVertexDefinition( sPt0) ;
              if(m_pStartPV->GetNumVertexDefinitions() < 2 )
                { bAddVtxDef = TRUE; }
              else
                {
                  SmVertexDefinition *pVD1 = m_pStartPV->GetClosestVertexDefinition( sPt1) ;
                  if(pVD0 == pVD1)
                    { bAddVtxDef = TRUE; }
                }

              if(bAddVtxDef)
                {
                  // Create a new VtxDef, and set pVD0 to sPt0, and NewVD to sPt1.
                  SmVertexDefinition *pNewVD = NULL;
                  SER( m_pProtoTopoMgr->CreateVertexDefinition( *(pVD0->GetBrepPointClassif(1)),
                                                                  *(pVD0->GetBrepPointClassif(2)), pNewVD ));
                  NER( pNewVD) ;

                  pVD0  ->SetPosition( sPt0) ;
                  pVD0  ->SetPointParameters();
                  pNewVD->SetPosition( sPt1) ;
                  pNewVD->SetPointParameters();

                  m_pStartPV->AddVertexDefinition( pNewVD) ;
                }
            } // end if our IntCurve is long enough to be its own Edge, with distinct Vertices.
        } // end else, our Intcurve is not shorter than tolerance
    } // end if start & end ProtoVertices are the same.


  // Now check EdgeDefs.
  if(m_sEdgeDefs.GetSize() < 2)
    { return SM_SUCCESS; }

  ULONG lDeleteWhich; // 0: neither; 1: 1st argument; 2: 2nd argument.

    // Pairwise:
    for(ii=0;ii<m_sEdgeDefs.GetSize() - 1;ii++)
     {
        pED1 = m_sEdgeDefs[ii];
        if(pED1 == NULL ) { continue; }

        for(jj=ii + 1;jj<m_sEdgeDefs.GetSize();jj++)
         {
          pED2 = m_sEdgeDefs[jj];
            if(pED2 == NULL ) { continue; }

            m_pProtoTopoMgr->CanCombineEdgeDefinitions( pED1, pED2, lDeleteWhich) ;

            if(lDeleteWhich == 1 )
             {
                //m_pProtoTopoMgr->CombineEdgeDefinitions( pED2, pED1) ;
                this->RemoveEdgeDefinition( pED1) ;
                pED1->Destruct();
                delete pED1;  pED1 = NULL;
                ii--;
                break;
                //jj = lNumEDs + 1; // We're done with PE[ii].
             }
            else if(lDeleteWhich == 2 )
            {
                //m_pProtoTopoMgr->CombineEdgeDefinitions( pED1, pED2) ;
                this->RemoveEdgeDefinition( pED2) ;
                pED2->Destruct();
                delete pED2;  pED2 = NULL;
                jj--;
              }
          }
      }

    m_sEdgeDefs.CompressZeros();

    return SM_SUCCESS;

}  // End SmProtoEdge::Resolve()

///////////////////////////////////////////
// SmProtoTopologyManager methods
///////////////////////////////////////////

/*******************************************************************//**
PURPOSE: SmProtoTopologyManager constructor.

USAGE NOTES---- 
***********************************************************************/
SmProtoTopologyManager::SmProtoTopologyManager
 (SmBrep                * pBrep1,           // in : 1st Brep of Boolean arg list
  SmBrep                * pBrep2,           // in : 2nd Brep of Boolean arg list
  SmTopologyIntersector * pTI,              // in : TopologyIntersector obj being used for the Boolean operation
  double                  dCaptureDistance, // in : 
  double                  dOverrideTol)     // seems to be an override XSectTol3d = max dist between coincident points
                                            //   pos     : Intersect TopoObjs to this XSectTol3d value
                                            //   0 or neg: m_dtol = if(set) m_pTI->ApproxTol3d value
                                            //             m_dtol = else    (Brep1->Tol + Brep2->Tol + MinEdgeSize / 2.0) ;
 : m_pBrep1(pBrep1),
   m_pBrep2(pBrep2),
   m_dTol(dOverrideTol),  // in: if zero or negative, we calculate our own.
   m_dCaptureDistance(dCaptureDistance) ,
   m_pTI(pTI) 
{
  m_cpContext = pBrep1->GetContext();

  // Get tol from pTI, which is from the SmMerge constructor.
  m_dTol = pTI->GetThisApproxTol3d();

  if(m_dTol < SM_EFF_ZERO)
    {
      //cbi: what's in SmMerge c'tor, modified to my liking.
      double dBrepTol = m_pBrep1->GetTolerance() + m_pBrep2->GetTolerance();
      SmEdge *pE1 = NULL, *pE2 = NULL; // unused
      double dEdge1Size = 0.0, dEdge2Size = 0.0;
      m_pBrep1->GetMinEdgeBBoxSize(dEdge1Size, pE1);
      m_pBrep2->GetMinEdgeBBoxSize(dEdge2Size, pE2);
      double dEdgeSize = smos_Min(dEdge1Size, dEdge2Size);

      m_dTol = smos_Min(dBrepTol*1.25, dEdgeSize*0.5);
    }

} // end SmProtoTopologyManager constructor


/*******************************************************************//**
PURPOSE: SmProtoTopologyManager destructor.

USAGE NOTES---- 
***********************************************************************/
SmProtoTopologyManager::~SmProtoTopologyManager()
{
  // Let these guys do the work.
  SmObjsDelete< SmProtoVertex* > sCleanPVs( &m_sPVs) ;
  SmObjsDelete< SmProtoEdge*   > sCleanPEs( &m_sPEs) ;
  SmObjsDelete< SmProtoFace*   > sCleanPFs( &m_sPFs) ;

} // end SmProtoTopologyManager destructor

/*******************************************************************//**
PURPOSE: Reset (Initialize) this SmProtoTopologyManager.

USAGE NOTES----  sets PrototArray sizes to zero
***********************************************************************/
void SmProtoTopologyManager::Reset()
{
  m_sPVs.ReSet() ;
  m_sPEs.ReSet() ;
  m_sPFs.ReSet() ;
} // end SmProtoTopologyManager::Reset

/*******************************************************************//**
PURPOSE: map ProtoVertex pointer to ProtoTopoMgr list item index

USAGE NOTES----  returns '99999' when Tgt PV is not in PV list
***********************************************************************/
ULONG SmProtoTopologyManager::GetProtoVertexId
 (const SmProtoVertex * pPV)  // in : Tgt ProtoVertex Ptr to map
const
{
  ULONG ii ;

  // linear search for pPV in m_sPVs
  for(ii=0;ii<m_sPVs.GetSize();ii++) { if(pPV == m_sPVs[ii])
                                         { return ii ; }
                                     }

  // pPV is not in m_sPVs
  return 99999 ; 

} // end SmProtoTopologyManager::GetProtoVertexId

/*******************************************************************//**
PURPOSE: map ProtoEdge pointer to ProtoTopoMgr list item index

USAGE NOTES----  returns '99999' when Tgt PE is not in PE list
***********************************************************************/
ULONG SmProtoTopologyManager::GetProtoEdgeId
 (const SmProtoEdge * pPE)  // in : Tgt ProtoEdge Ptr to map
const
{
  ULONG ii ;

  // linear search for pPE in m_sPEs
  for(ii=0;ii<m_sPEs.GetSize();ii++) { if(pPE == m_sPEs[ii])
                                         { return ii ; }
                                     }

  // pPE is not in m_sPEs
  return 99999 ; 

} // end SmProtoTopologyManager::GetProtoEdgeId

/*******************************************************************//**
PURPOSE: map ProtoFace pointer to ProtoTopoMgr list item index

USAGE NOTES----  returns '99999' when Tgt PF is not in PF list
***********************************************************************/
ULONG SmProtoTopologyManager::GetProtoFaceId
 (const SmProtoFace * pPF)  // in : Tgt ProtoFace Ptr to map
const
{
  ULONG ii ;

  // linear search for pPF in m_sPFs
  for(ii=0;ii<m_sPFs.GetSize();ii++) { if(pPF == m_sPFs[ii])
                                         { return ii ; }
                                     }

  // pPF is not in m_sPFs
  return 99999 ; 

} // end SmProtoTopologyManager::GetProtoFaceId

/*******************************************************************//**
PURPOSE: Get all VertexDefinitions in this ProtoTopologyManager.

USAGE NOTES---- Each VertexDefinition is part of only one ProtoVertex.
***********************************************************************/
void SmProtoTopologyManager::GetAllVertexDefinitions
 (SmTArray<SmVertexDefinition* > & rVtxDefs)
{
  ULONG ii ;
  ULONG lNumPVs = m_sPVs.GetSize() ;
  //SmTArray< SmVertexDefinition* > * pVtxDefList = NULL;

  // init output
  rVtxDefs.ReSet() ;

  // for every ProtoVertex - Get all VertexDefinitions
  for(ii=0;ii<lNumPVs;ii++)
    {
      SmProtoVertex * pPV = m_sPVs[ii] ;
      SmTArray<SmVertexDefinition*> & rVtxDefList = pPV->GetVertexDefinitions() ;
      rVtxDefs.Append( rVtxDefList ) ;
    }

}  // end SmProtoTopologyManager::GetAllVertexDefinitions

/*******************************************************************//**
PURPOSE: Get all EdgeDefinitions in this ProtoTopologyManager.

USAGE NOTES---- Each EdgeDefinition is part of only one ProtoEdge.
***********************************************************************/
void SmProtoTopologyManager::GetAllEdgeDefinitions
 (SmTArray< SmEdgeDefinition* > & rEdgeDefs)
{
  ULONG ii ;
  ULONG lNumPEs = m_sPEs.GetSize();

  // init output
  rEdgeDefs.ReSet();

  // for every ProtoEdge - get all EdgeDefinitions
  for(ii=0;ii<lNumPEs;ii++)
    {
      SmProtoEdge *pPE = m_sPEs[ii];
      SmTArray< SmEdgeDefinition* > & rEdgeDefList = pPE->GetEdgeDefinitions();
      rEdgeDefs.Append( rEdgeDefList ) ;
    }

}  // end SmProtoTopologyManager::GetAllEdgeDefinitions

/*******************************************************************//**
PURPOSE: Find or create a VertexDefinition given two SmTopology objects and a 3d position.

USAGE NOTES---- 
   Looks for existing VertexDefinition, and creates new if not found.
   Does not add the new VertexDefinition to 'this', nor do anything else with it.

***********************************************************************/
SmStatus SmProtoTopologyManager::FindOrCreateVertexDefinition
 (SmTopology          * pTopo1,        // in :
  SmTopology          * pTopo2,        // in :
  const SmPoint3d     & crPos,         // in :
  SmVertexDefinition *& rpNewVD,       // out:
  SmBoolean           * pbWasCreated ) // out: TRUE = created new VertexDefinition, FALSE=didn't, NULL to ignore, default:[NULL]
{
  // Init output
  rpNewVD = NULL;
  if(pbWasCreated != NULL) { *pbWasCreated = FALSE; }

  rpNewVD = this->FindVertexDefinition( crPos, pTopo1, pTopo2, m_dTol) ; //cbiTol

  if(rpNewVD != NULL )
    { return SM_SUCCESS; }

  // Didn't find one, need to create one.

  SmStatus eStat = this->CreateVertexDefinition( pTopo1, pTopo2, crPos, rpNewVD) ;

  if(eStat == SM_SUCCESS && rpNewVD != NULL )
  {
      if(pbWasCreated != NULL ) { *pbWasCreated = TRUE; }
  }

  return eStat;

} // end SmProtoTopologyManager::FindOrCreateVertexDefinition

/*******************************************************************//**
PURPOSE: Create a VertexDefinition from two PointClassification objects

USAGE NOTES---- 
   Does not add the new VertexDefinition to 'this', nor do anything else with it.
   Does not provide the new VertexDefinition with an owning ProtoVertex.
***********************************************************************/
SmStatus SmProtoTopologyManager::CreateVertexDefinition( SmPointClass        & rPC1,
                                                         SmPointClass        & rPC2,
                                                         SmVertexDefinition *& rpNewVD )
{
  // Init outputs
  rpNewVD = NULL;

  // Locals
  SmPoint3d sPt;
  double dGap;

  sm_FindPositionFromPCs( rPC1, rPC2, sPt, dGap) ;

  double dTol = smos_Max( rPC1.GetTolerance(), rPC2.GetTolerance()) ;
  dTol = smos_Max( dTol, this->m_dTol) ;

  dGap = smos_Max( dGap, rPC1.GetGap3d()) ;
  dGap = smos_Max( dGap, rPC2.GetGap3d()) ;

  rpNewVD = new( this ) SmVertexDefinition( this->GetContext(), &rPC1, &rPC2, sPt, dTol, dGap, NULL) ;

  return SM_SUCCESS;

} // end SmProtoTopologyManager::CreateVertexDefinition

/*******************************************************************//**
PURPOSE: Create a VertexDefinition from two SmTopology objects and a 3d position.

USAGE NOTES---- 
   Does not add the new VertexDefinition to 'this', nor do anything else with it.
   Does not provide the new VertexDefinition with an owning ProtoVertex.
***********************************************************************/
SmStatus SmProtoTopologyManager::CreateVertexDefinition( SmTopology         *  pTopo1,
                                                         SmTopology         *  pTopo2,
                                                         const SmPoint3d     & crPos,
                                                         SmVertexDefinition *& rpNewVD )
{
  // Init outputs
  rpNewVD = NULL;

  // Locals
  SmBoolean bSuccess;
  SmPoint2d sUV;
  double dGap1 = 0.0;
  double dGap2 = 0.0;
  double dT = 0.0;
  SmBoolean bIsMulti;


  // Set up sPC1.
  SmPointClassification sPC1 ( m_dTol, pTopo1->GetContext()) ;

  SmFace *pBrep1Face = SM_CAST_PTR( SmFace, pTopo1) ;
  SmEdge *pBrep1Edge = SM_CAST_PTR( SmEdge, pTopo1) ;
  bSuccess = FALSE;
  if(pBrep1Face != NULL )
  {
      sPC1.SetClassObject( SM_PC_FACE, pBrep1Face) ;
      pBrep1Face->GetSurface()->DropPoint(crPos, pBrep1Face->GetUVDomain(), NULL,
                                          bSuccess, sUV, dGap1, bIsMulti) ;
      if(bSuccess )
        { sPC1.SetUVParam( sUV) ; }
  }
  else if(pBrep1Edge != NULL )
  {
      sPC1.SetClassObject( SM_PC_EDGE, pBrep1Edge) ;
      SmExtent1d sDomain = pBrep1Edge->GetCurve()->GetNaturalInterval();
      pBrep1Edge->GetCurve()->DropPoint( sDomain, // in : target curve allowed domain
                                         crPos,   // in : Point to drop to curve
                                         NULL,    // in : For Snapping, NULL=NoSnap, Vec pointing to CrvInside from desired EndPt for snapping.
                                                  //      when DropPt is within dDistTol in cpVecToCrvInside dir of EndPt, Snap to Ivl EndPt.
                                                  //      Vec handles ambiguities and makes sure that curves are not just touching at the ends.
                                        m_dTol,   // in : Used for SnapDist, and SM_SO_INTERSECT sXSectTol3d distance.
                                                  //      sXSectTol3d = dDistTol ;      SM_SO_INTERSECT
                                                  //      sXSectTol3d = SM_BIG_DOUBLE ; SM_SO_MINIMIZE, SM_SO_NORMALIZE
                                                  //      sSnapTol3d  = dDistTol ;      SM_SO_MINIMIZE, SM_SO_NORMALIZE, SM_SO_INTERSECT
                                        NULL,     // in : If given, call cheap LocalPointSolve() else call expensive GlobalPointSolve()
                                        bSuccess, // out: TRUE = found a drop point
                                        dT,       // out: found drop curve param
                                        dGap1) ;  // out: found drop distance
                                                  // in : SM_SO_MINIMIZE = allow nonNormal drops near endPoints
                                                  //      SM_SO_INTERSECT= like Minimize but point must be within dDistTol
                                                  //      SM_SO_NORMALIZE= exclude nonNormal drops near endPoints
                                                  //      default:[SM_SO_MINIMIZE] to preserve original behavior
      if(bSuccess )
        { sPC1.SetTParam( dT) ; }
  }
  else
    { SER( SM_ERR_INVALID_INPUT) ; }

  if(bSuccess )
  {
#ifdef SM_USE_NEWTOL
      SM_NEWTOL_LINE sPC1.SetMaxGap3d( dGap2) ;
#else // SM_USE_OLDTOL
      SM_OLDTOL_LINE sPC1.SetGap3d( dGap2) ;
#endif
  }



  // Same for sPC2.
  SmPointClassification sPC2 ( m_dTol, pTopo2->GetContext()) ;

  SmFace *pBrep2Face = SM_CAST_PTR( SmFace, pTopo2) ;
  SmEdge *pBrep2Edge = SM_CAST_PTR( SmEdge, pTopo2) ;
  bSuccess = FALSE;
  if(pBrep2Face != NULL )
  {
      sPC2.SetClassObject( SM_PC_FACE, pBrep2Face) ;
      pBrep2Face->GetSurface()->DropPoint(crPos, pBrep2Face->GetUVDomain(), NULL,
                                          bSuccess, sUV, dGap2, bIsMulti) ;
      if(bSuccess )
        { sPC2.SetUVParam( sUV) ; }
  }
  else if(pBrep2Edge != NULL )
  {
      sPC2.SetClassObject( SM_PC_EDGE, pBrep2Edge) ;
      SmExtent1d sDomain = pBrep2Edge->GetCurve()->GetNaturalInterval();
      pBrep2Edge->GetCurve()->DropPoint
                  (sDomain,  // in : target curve allowed domain
                   crPos,    // in : Point to drop to curve
                   NULL,     // in : For Snapping, NULL=NoSnap, Vec pointing to CrvInside from desired EndPt for snapping.
                             //      when DropPt is within dDistTol in cpVecToCrvInside dir of EndPt, Snap to Ivl EndPt.
                             //      Vec handles ambiguities and makes sure that curves are not just touching at the ends.
                   m_dTol,   // in : Used for SnapDist, and SM_SO_INTERSECT sXSectTol3d distance.
                             //      sXSectTol3d = dDistTol ;      SM_SO_INTERSECT
                             //      sXSectTol3d = SM_BIG_DOUBLE ; SM_SO_MINIMIZE, SM_SO_NORMALIZE
                             //      sSnapTol3d  = dDistTol ;      SM_SO_MINIMIZE, SM_SO_NORMALIZE, SM_SO_INTERSECT
                   NULL,     // in : If given, call cheap LocalPointSolve() else call expensive GlobalPointSolve()
                   bSuccess, // out: TRUE = found a drop point
                   dT,       // out: found drop curve param
                   dGap2) ;  // out: found drop distance
                             // in : SM_SO_MINIMIZE = allow nonNormal drops near endPoints
                             //      SM_SO_INTERSECT= like Minimize but point must be within dDistTol
                             //      SM_SO_NORMALIZE= exclude nonNormal drops near endPoints
                             //      default:[SM_SO_MINIMIZE] to preserve original behavior
      if(bSuccess )
        { sPC2.SetTParam( dT) ; }
  }
  else
    { SER( SM_ERR_INVALID_INPUT) ; }

  if(bSuccess )
  {
#ifdef SM_USE_NEWTOL
      SM_NEWTOL_LINE sPC2.SetMaxGap3d( dGap2) ;
#else // SM_USE_OLDTOL
      SM_OLDTOL_LINE sPC2.SetGap3d( dGap2) ;
#endif // SM_USE_OLDTOL
  }


  double dGap = smos_Max( dGap1, dGap2) ;

  rpNewVD = new( this ) SmVertexDefinition( this->GetContext(), &sPC1, &sPC2, crPos, m_dTol, dGap) ;

  return SM_SUCCESS;

} // end SmProtoTopologyManager::CreateVertexDefinition

/*******************************************************************//**
PURPOSE:  Find existing VertexDefinition within a ProtoVertex near 
          TgtPos and defined by the input Topo pair
USAGE NOTES---- 
***********************************************************************/
SmVertexDefinition *SmProtoTopologyManager::FindVertexDefinition
 (const SmVector3d & crPos,   // in : Tgt Position - used to find nearby ProtoVertices
  const SmTopology * pTopo1,  // in : Topo1 of VertexDefinition TopoPair to be found in ProtoVertex within tol of crPos
  const SmTopology * pTopo2,  // in : Topo2 of VertexDefinition TopoPair to be found in ProtoVertex within tol of crPos
  double             dTol )   // NotUsed: in : Max dist between crPos and found ProtoVertex 
{
  // init return value
  SM_REF1(dTol) ;
  SmVertexDefinition *pRet = NULL;

  // find ProtoVertex within tol of CrPos
  SmProtoVertex *pPV = this->FindProtoVertex( crPos) ;

  // no work - no ProtoVertex found
  if(pPV == NULL )
    { return pRet; }

  // search found PV->List of VertexDefinitions to find the first defined by the Tgt TopoObjs
  pRet = pPV->FindMatchingVertexDefinition( pTopo1, pTopo2) ;
   
  // all done
  return pRet;

} // end SmProtoTopologyManager::FindVertexDefinition

/*******************************************************************//**
PURPOSE: Create a ProtoVertex from two SmPointClassifications.

USAGE NOTES---- 
   If both PCs classify to topology, creates an SmProtoVertex.
   Adds the new ProtoVertex to 'this', and returns it as rpNewPV.

   else returns NULL.
***********************************************************************/
SmStatus SmProtoTopologyManager::CreateProtoVertex
 (SmPointClassification & rPC1,    // in : Brep1 PtClassification for an XSectPt3d
  SmPointClassification & rPC2,    // in : Brep1 PtClassification for an XSectPt3d
  SmProtoVertex        *& rpNewPV) // out: New ProtoVertex for input PtClassifies when both pts classify to Topology, 
                                   //      else NULL for no new PV
{
  // Init outputs
  rpNewPV = NULL;

  // Check inputs
  if(! sm_CanImprint( rPC1.GetPointClass() ) ) { return SM_SUCCESS; }
  if(! sm_CanImprint( rPC2.GetPointClass() ) ) { return SM_SUCCESS; }

  // Locals
  SmPoint3d sPt;
  double dDev;

  // Get SmPointClassifications AvgPosition and Deviation
  sm_FindPositionFromPCs(rPC1,   // in : Point Classification 1
                         rPC2,   // in : Point Classification 2
                         sPt,    // out: Avg of Surf1(UV1) and Surf2(UV2)
                         dDev) ; // out: Dist between Surf1(UV1) and Surf2(UV2)

  // let ZoneTol3d = Max(PtClass and PtClass->Object ZoneTol3ds)
  SmZoneTol3d sPC1ObjTol = SmTol::GetZoneTol3d( rPC1.GetObject()) ;
  SmZoneTol3d sPC2ObjTol = SmTol::GetZoneTol3d( rPC2.GetObject()) ;
  double dTol = smos_4Max( dDev, m_dTol, (double)sPC1ObjTol, (double)sPC2ObjTol) ;  //cbiTol ???

  // New ProtoVertex
  SmProtoVertex *pPV = new (this) SmProtoVertex(rPC1, rPC2, sPt, dTol, this) ;  NER(pPV) ;

  // Add new PV to ProtoManager PV list
  this->AddProtoVertex(pPV) ;

  // set output
  rpNewPV = pPV;

  // all done
  return SM_SUCCESS;

} // end SmProtoTopologyManager::CreateProtoVertex

/*******************************************************************//**
PURPOSE: Find or create a ProtoVertex from a 3d position.

USAGE NOTES---- 
   Looks for existing ProtoVertex, and creates a new, empty ProtoVertex if not found.
   Adds the new ProtoVertex to 'this', and returns it as rpNewPV.
***********************************************************************/
SmStatus SmProtoTopologyManager::FindOrCreateProtoVertex
 (const SmPoint3d  & crPos,         // in : TgtPosition
  SmProtoVertex   *& rpNewPV,       // out: either an existing ProtoVertex within Tol of crPos, or a new one created at crPos
  SmBoolean        * pbWasCreated)  // out: optional output, TRUE =rpNewPV is a new ProtoVertex
                                    //                       FALSE=rpNewPV is an existing one, 
                                    //      NULL to ignore, default:[NULL]
{
  // Init outputs
  rpNewPV = NULL;
  if(pbWasCreated != NULL) { *pbWasCreated = FALSE; }

  // search for closest existing ProtoVertex within max(m_dTol, GetCaptureDistance()) of Tgt crPos
  SmProtoVertex *pPV = this->FindProtoVertex(crPos) ;

  // when no ProtoVertex is found within tol of crPos
  if(pPV == NULL )
    {
      // make a new one
      pPV = new ( this ) SmProtoVertex( crPos, m_dTol, this) ;  NER( pPV) ;

      // add it to the ProtoVertex list
      this->AddProtoVertex( pPV) ;

      // remember it was created
      if(pbWasCreated != NULL ) { *pbWasCreated = TRUE; }
    } // end no found ProtoVertex check

  // set output
  rpNewPV = pPV;

  // all done
  return SM_SUCCESS;

} // end SmProtoTopologyManager::FindOrCreateProtoVertex

/*******************************************************************//**
PURPOSE: Create a ProtoVertex from two SmTopology objects and a 3d position.

USAGE NOTES---- 
   Creates a new SmProtoVertex, add it to 'this', and returns it as rpNewPV.
***********************************************************************/

/* cbi :not needed?

SmStatus SmProtoTopologyManager::CreateProtoVertex( SmTopology *pTopo1,
                                         SmTopology *pTopo2,
                                         const SmPoint3d &crPt,
                                         SmProtoVertex *&rpNewPV )
{
  // Init output
  rpNewPV = NULL;

. SmVertexDefinition *pNewVD = NULL;

. SER( this->CreateVertexDefinition( pTopo1, pTopo2, crPt, pNewVD )) ;
. NER( pNewVD) ;

. rpNewPV = new( m_pProtoTopoMgr ) SmProtoVertex( pNewVD) ;

. this->Add( pPV) ;

. return SM_SUCCESS;

} // end SmProtoTopologyManager::CreateProtoVertex

cbi. */

/*******************************************************************//**
PURPOSE: Find or create a ProtoVertex from two SmPointClassifications.

USAGE NOTES---- 
   Looks for existing ProtoVertex, and creates a new one if not found.
   If both PCs classify to topology, creates an SmProtoVertex.
   Adds the new ProtoVertex to 'this', and returns it as rpNewPV.
***********************************************************************/
SmStatus SmProtoTopologyManager::FindOrCreateProtoVertex
 (SmPointClassification & rPC1,           // in : Brep1 PointClassification for face pair XSectPt3d
  SmPointClassification & rPC2,           // in : Brep2 PointClassification for face pair XSectPt3d
  SmProtoVertex        *& rpNewPV,        // out: SmProtoVertex for face pair XSectPt3d
  SmBoolean             * pbWasCreated )  // out: TRUE = Created output NewPV
                                          //      FALSE= Found   output NewPV
{
  // Init output
  rpNewPV = NULL;
  if(pbWasCreated != NULL ) { *pbWasCreated = FALSE; }

  // Skip PtXSects that don't classify to Region, Face, Edge, or Vertex in both Breps
  if(sm_CanImprint(rPC1.GetPointClass()) == FALSE) { return SM_SUCCESS ; }
  if(sm_CanImprint(rPC2.GetPointClass()) == FALSE) { return SM_SUCCESS ; }

  // Locals
  SmPoint3d sPt;
  double    dDev;
  sm_FindPositionFromPCs(rPC1,    // in : Brep1 PointClassification
                         rPC2,    // in : Brep2 PointClassification
                         sPt,     // out: Avg of Surf1(UV1) and Surf2(UV2) 
                         dDev) ;  // out: Dist between Surf1(UV1) and Surf2(UV2) 

  // find closest ProtoVertex within m_dTol to sPt
  rpNewPV = this->FindProtoVertex(sPt) ;

  // when nearby PV exists
  if(rpNewPV)
    {
      // make sure ProtoVertex contains a VertexDef for input PointClassifications
      SmVertexDefinition *pNewVD = NULL;
      rpNewPV->AddVertexDefinition(rPC1,     // in : Brep1 PointClassification for Brep1/Brep2 XSectPt3d
                                   rPC2,     // in : Brep2 PointClassification for Brep1/Brep2 XSectPt3d
                                   pNewVD) ; // out: default [NULL]
                                             // out: optional flag: TRUE = rpNewVD was created, NULL to ignore, default:[NULL]
                                             //                     FALSE= rpNewVD was found
      return SM_SUCCESS;
    }

  // arrive here when a new ProtoVertex has to be created

  // create NewPV
  SmStatus eStat = this->CreateProtoVertex( rPC1, rPC2, rpNewPV) ;

  // set output
  if(   eStat   == SM_SUCCESS 
     && rpNewPV != NULL)
  {
      if(pbWasCreated != NULL ) { *pbWasCreated = TRUE; }
  }

  return eStat;

}  // end SmProtoTopologyManager::FindOrCreateProtoVertex

/*******************************************************************//**
PURPOSE: Return contained PE with ED->XSectCrv3d within CaptureDistance of 
         TgtCurve3d or NULL. Set rbIsIdentical == TRUE when returned PE's 
         same-shape ED also shares the same SrcTopos as the input XSect Interval

USAGE NOTES---- 
***********************************************************************/
SmProtoEdge * SmProtoTopologyManager::FindProtoEdge // rtn: found PE with ED with same shape (ED->XSectCrv3d within CaptureDistance) as TgtCurve3d
 (SmProtoVertex               * pTgtStartPV,        // in : Tgt Start ProtoVertex
  SmProtoVertex               * pTgtEndPV,          // in : Tgt End   ProtoVertex
  const SmPointClassification & crMidPC1,           // in : Tgt MidPt Classification for face1->CrvClass->TgtCrvInterval
  const SmPointClassification & crMidPC2,           // in : Tgt MidPt Classification for face2->CrvClass->TgtCrvInterval
  const SmCurve               * pTgtCurve3d,        // in : shared Tgt XSectCurve3d being classified against face1/face2 pair
  const SmExtent1d            & rTgtCurveDomain,    // in : TgtCrvInterval domain
  SmBoolean                   & rbIsIdentical)      // out: TRUE  = returned PE->ED also shares the same SrcTopos as the input XSect Interval
 const                                              //      FALSE = returned PE->ED only has the same shape as input XSectInterval but different SrcTopos
{
  // Init output
  rbIsIdentical = FALSE;

  // no work - missing a start or end ProtoVertex
  if(   pTgtStartPV == NULL 
     || pTgtEndPV   == NULL)
    { return NULL; }

  // locals
  ULONG ii ; 
  SmTArray< SmProtoEdge* > sCommonPEs;
  SmTArray< SmProtoEdge* > & rPV1Edges = pTgtStartPV->GetProtoEdges();
  SmTArray< SmProtoEdge* > & rPV2Edges = pTgtEndPV->GetProtoEdges();
  SmProtoEdge *pPE;

  // find all existing PEs between the start and end PVs.
  rPV1Edges.FindCommonElements( rPV2Edges, sCommonPEs) ;
  ULONG lNumPEs = sCommonPEs.GetSize();

  // for every CommonPE - look for a match
  for(ii=0;ii<lNumPEs;ii++)
  {
      pPE = sCommonPEs[ii] ;

      // look for matching ProtoEdge
      if(pPE->IsSameProtoEdge(pTgtStartPV, 
                              pTgtEndPV, 
                              crMidPC1, 
                              crMidPC2, 
                              pTgtCurve3d, 
                              rTgtCurveDomain, 
                              rbIsIdentical) )
        { return pPE; }
    } // end iter every CommonPE looking for a matching PE

  // arrive here with no matching ProtoEdge - return NULL
  return NULL;

} // end SmProtoTopologyManager::FindProtoEdge

/*******************************************************************//**
PURPOSE: Create a new SmProtoEdge in this SmProtoTopologyManager
         defined by XSect SmCurveIntervals and bounded by 
         given ProtoVertices.

USAGE NOTES---- 
***********************************************************************/
SmStatus SmProtoTopologyManager::CreateProtoEdge
 (SmProtoVertex         * pStartPV,  // in : start ProtoVertex
  SmProtoVertex         * pEndPV,    // in : end ProtoVertex
  const SmCurveInterval & crCI1,     // in : CurveIvl from SrcTopo 1
  const SmCurveInterval & crCI2,     // in : CurveIvl from SrcTopo 2
  SmProtoEdge          *& rpNewPE )  // out: new ProtoEdge
{
  // For convenience, get back-pointers to the SmCurveClass objects.
  const SmCurveClass *pCC1 = crCI1.m_pCurveClassification;
  const SmCurveClass *pCC2 = crCI2.m_pCurveClassification;

  // Collect data for a new ProtoEdge.
  const SmCurve *p3dCurve = pCC1->GetCurve();
  SM_ASSERT( pCC2->GetCurve() == p3dCurve) ;

  // Create a copy of the input curve and trim it to this CurveInterval.
  SmCurve *pCopyCurve = NULL;
  SmStatus eStat = p3dCurve->Copy( *m_cpContext, pCopyCurve) ;
  SER( eStat) ;  NER( pCopyCurve) ;

  SmExtent1d sCurveDomain = crCI1.m_vInterval;
  SER( pCopyCurve->Trim( sCurveDomain ));  // may snap sIvl by tol to existing knots

  // See about uv curves.
  SmCurve *pUVCurve1 = NULL;
  const SmCurve *pCCUVCurve = pCC1->GetUVCurve1();
  if(pCCUVCurve != NULL )
  {
      eStat = pCCUVCurve->Copy( *m_cpContext, pUVCurve1) ;  //cbi <- delete in d'tor?
      if(pUVCurve1 != NULL )
        { pUVCurve1->Trim( sCurveDomain) ; }  // may snap sIvl by tol to existing knots
  }

  SmCurve *pUVCurve2 = NULL;
  pCCUVCurve = pCC2->GetUVCurve1();
  if(pCCUVCurve != NULL )
  {
      eStat = pCCUVCurve->Copy( *m_cpContext, pUVCurve2) ;  //cbi <- delete in d'tor?
      if(pUVCurve2 != NULL )
        { pUVCurve2->Trim( sCurveDomain) ; }  // may snap sIvl by tol to existing knots
  }

  // Create it.
  rpNewPE = new (this) SmProtoEdge( pStartPV, pEndPV, sCurveDomain, this) ;
  NER( rpNewPE) ;

  // Make an EdgeDefinition for the new ProtoEdge.
  SmTopology * pBrep1Topo = SM_CAST_PTR( SmTopology, crCI1.m_vMid.GetObject()) ; NER( pBrep1Topo) ;
  SmTopology * pBrep2Topo = SM_CAST_PTR( SmTopology, crCI2.m_vMid.GetObject()) ; NER( pBrep2Topo) ;

  SmEdgeDefinition *pNewED = new (this) SmEdgeDefinition( pCopyCurve, sCurveDomain,
                                                          pBrep1Topo, pBrep2Topo,
                                                          m_dTol,
                                                          pUVCurve1, pUVCurve2, rpNewPE) ;

  // Hook up pointers.
  rpNewPE->AddEdgeDefinition( pNewED) ;

  this->AddProtoEdge( rpNewPE) ;

  pStartPV->AddProtoEdge( rpNewPE) ;
  pEndPV  ->AddProtoEdge( rpNewPE) ; // DD1

  return SM_SUCCESS;

} // end SmProtoTopologyManager::CreateProtoEdge

/*******************************************************************//**
PURPOSE: Remove but don't delete given SmVertexDefinition from its ownerPV->m_sVertexDefs list

USAGE NOTES---- 
   Does not delete SmVertexDefinitions.
   bRemoveOwnerIfEmpty: If the owning SmProtoVertex has no VertexDefinitions
   left after removing pVD, remove and delete the owning SmProtoVertex as well.
***********************************************************************/
SmStatus SmProtoTopologyManager::RemoveVertexDefinition
 (SmVertexDefinition * pVD,                  // in : VertexDef to remove
  SmBoolean            bRemoveOwnerIfEmpty ) // in : TRUE  = remove and delete PVs with no other VertexDefs from ProtoMgr
                                             //      FALSE = don't
{
  SmProtoVertex * pOwner = pVD->GetProtoVertex() ;

  // remove input VertexDef from ownerPV->m_sVertexDefs list
  pOwner->RemoveVertexDefinition(pVD) ;

  // when asked - remove and delete ownerPVs with no other VertexDefs
  if(   bRemoveOwnerIfEmpty
     && pOwner->GetNumVertexDefinitions() < 1)
    { 
      this->RemoveProtoVertex(pOwner) ;
      delete pOwner ; pOwner = NULL ;
    }

  // all done
  return SM_SUCCESS ;

} // end SmProtoTopologyManager::RemoveVertexDefinition

/*******************************************************************//**
PURPOSE: Remove the given SmProtoVertex from this ProtoTopoMgr PV list

USAGE NOTES---- 
   Does not delete anything.
   Warning: ProtoEdges without ProtoVertices are a database error, so
    presumably this is only used on PVs with no PEs.
***********************************************************************/
SmStatus SmProtoTopologyManager::RemoveProtoVertex
 (SmProtoVertex * pPV)  // in: ProtoVertex to remove
{
  ULONG idx;

  // check state - dont' remove a ProtoVertex connected to ProtoEdges making stale ProtoEdge->m_pStartPV and m_pEndPV pointers
  SM_ASSERT_MSG(pPV->GetNumProtoEdges() == 0, _T("SmProtoTopologyManager::RemoveProtoVertex error: removing a ProtoVertex connected to a ProtoEdge - only remove PVs not connected to PEs")) ; 

  // remove input pPV from the PV list when it's listed - else signal an error
  if(m_sPVs.FindElement(pPV, idx )) { m_sPVs.RemoveAt( idx) ; }
  else                              { return SM_ERR; }

  // all done
  return SM_SUCCESS;

} // end SmProtoTopologyManager::RemoveProtoVertex

/*******************************************************************//**
PURPOSE: Remove but don't delete given SmEdgeDefinition from its ownerPE->m_sEdgeDefs list

USAGE NOTES---- 
   Does not delete SmEdgeDefinitions.
   bRemoveOwnerIfEmpty: If the owning SmProtoEdge has no EdgeDefinitions
   left after removing pED, remove and delete the owning SmProtoEdge as well.
***********************************************************************/
SmStatus SmProtoTopologyManager::RemoveEdgeDefinition
 (SmEdgeDefinition * pED,                  // in : EdgeDef to remove
  SmBoolean          bRemoveOwnerIfEmpty ) // in : TRUE  = remove and delete PEs with no other EdgeDefs from ProtoMgr
                                           //      FALSE = don't
{
  SmProtoEdge * pOwner = pED->GetProtoEdge() ;

  // remove input EdgeDef from ownerPE->m_sEdgeDefs list
  pOwner->RemoveEdgeDefinition(pED) ;

  // when asked - remove and delete ownerPEs with no other EdgeDefs
  if(   bRemoveOwnerIfEmpty
     && pOwner->GetNumEdgeDefinitions() < 1)
    { 
      this->RemoveProtoEdge(pOwner) ;
      delete pOwner; pOwner = NULL;
    }

  // all done 
  return SM_SUCCESS;

} // end SmProtoTopologyManager::RemoveEdgeDefinition

/*******************************************************************//**
PURPOSE: Remove the given SmProtoEdge from our data structures.

USAGE NOTES---- 
   Does not delete anything.
***********************************************************************/
SmStatus SmProtoTopologyManager::RemoveProtoEdge( SmProtoEdge * pPE )
{
  ULONG idx;
  if(m_sPEs.FindElement(pPE, idx) )
    { m_sPEs.RemoveAt(idx) ; }
  else
    { return SM_ERR; }

  // remove End PV->PE back pointers
  SmProtoVertex * pStartPV = pPE->GetStartProtoVertex() ;
  SmProtoVertex * pEndPV   = pPE->GetEndProtoVertex() ;

  if(pStartPV) pStartPV->RemoveProtoEdge(pPE) ;
  if(pEndPV)   pEndPV->RemoveProtoEdge(pPE) ;

  // all done
  return SM_SUCCESS;

} // end SmProtoTopologyManager::RemoveProtoEdge

// -------------------------------------
// THIS IS CURRENTLY UNUSED
// -------------------------------------
#if 0
/*******************************************************************//**
PURPOSE: static helper: is a given CurveInterval partially coincident with an SmEdge or SmFace?

USAGE NOTES---- 
   Partially coincident means that the Mid is on an SmEdge or SmFace, and the Start or End
   is on the same entity.
***********************************************************************/
static SmBoolean sm_IsPartiallyCoincident( const SmCurveInterval &crCI, // in: 
                                           SmBoolean &rbIsCoincStart,   // out:
                                           SmBoolean &rbIsCoincEnd,     // out:
                                           SmEdge *& pOnEdge,           // out:
                                           SmFace *& pOnFace )          // out:
{
  // Init outputs.
  rbIsCoincStart = rbIsCoincEnd = FALSE;
  pOnEdge = NULL;
  pOnFace = NULL;

  // Check coinc Edge.
  pOnEdge = crCI.m_vMid.GetEdgeObject();

  if(pOnEdge != NULL )
  {
      if(crCI.m_vStart.GetEdgeObject() == pOnEdge )
        { rbIsCoincStart = TRUE; }

      if(crCI.m_vEnd.GetEdgeObject()   == pOnEdge )
        { rbIsCoincEnd   = TRUE; }
  }
  if(rbIsCoincStart || rbIsCoincEnd )
    { return TRUE; }

  pOnEdge = NULL;

  // Now check Face.
  pOnFace = crCI.m_vMid.GetFaceObject();
  rbIsCoincStart = rbIsCoincEnd = FALSE;

  if(pOnFace != NULL )
  {
      if(crCI.m_vStart.GetFaceObject() == pOnFace )
        { rbIsCoincStart = TRUE; }

      if(crCI.m_vEnd.GetFaceObject()   == pOnFace )
        { rbIsCoincEnd   = TRUE; }
  }
  if(rbIsCoincStart || rbIsCoincEnd )
    { return TRUE; }
  
  pOnFace = NULL;
  return FALSE;

} // end sm_IsPartiallyCoincident
#endif

/*******************************************************************//**
PURPOSE: static helper: Is a curve parameter inside of the TgtCrv->Interval, 
                        not outside or on the bndry

USAGE NOTES---- return FALSE When  
         1. Param < DomainMin || Param > DomainMax  (its either an outside or bndry pt)
         2. ParamPt3d in tol of CurveStart3d        (its a bndry pt)
         3. ParamPt3d in tol of CurveEnd3d          (its a bndry pt)
       Otherwise return TRUE.
***********************************************************************/
static SmBoolean sm_IsInterior
 (const SmCurve    * cpCurve,     // in : TgtCurve
  const SmExtent1d & crDomain,    // in : TgtCurve Ivl domain
  double             dParam,      // in : TgtParam to test
  SmZoneTol3d        sZoneTol3d ) // in : Max dist between a test point and Tgt Ivl end considered in the interior
{
  // locals
  // SmScaledZero sScaleZero = SmTol::GetScaledZero(crDomain) ;
  // SmZoneTol3d  sParamTol  = sScaleZero.val;
  SmPoint3d    sStartPt ;
  SmPoint3d    sEndPt ;
  SmPoint3d    sCrvPt ;

  // low work - When Param is outside of Curve domain - it's not interior
  if(FALSE == crDomain.ContainsValue( dParam,0.0) )
    { return FALSE ; }

  // next: Check 3d Pts. if this param's Pt3d is within ZoneTol3d dist of a crv endpoint it on the boundary not the interior

  // locals
  cpCurve->EvaluatePoint( dParam, sCrvPt) ;
  cpCurve->EvaluatePoint( crDomain.GetMin(), sStartPt) ;
  cpCurve->EvaluatePoint( crDomain.GetMax(), sEndPt) ;

  // param is Boundary not Interior when ParamPt3d is on the start boundary of the curve
  if(sCrvPt.CloserThan(sZoneTol3d, sStartPt) )
    { return FALSE ; }

  // param is Boundary not Interior when ParamPt3d is on the end boundary of the curve
  if(sCrvPt.CloserThan(sZoneTol3d, sEndPt) )
    { return FALSE ; }

  // all done
  return TRUE ;

} // end static sm_IsInterior

/*******************************************************************//**
PURPOSE: Assuming the CurveInterval->Curve and EdgeDef->Curves come
         from common sources, returns TRUE when the curve intervals
         intersect over a segment.

USAGE NOTES---- 
    Assumes the CurveInterval->CurveClassification-Curve and the EdgeDefinition->XSectCurve
            are the same or different approximations to the same curve shape. For example
            one intersection found many times by a boolean operation.
    so this function checks to see if the endpoints of one curve
       drop to the interior of the other curve.

    This check will catch all the cases where there is an XSecting segment between the two curves as:
       ED curve interval:     +----------+  +----------+    +----------+     +----------+   
       CI curve interval:   +----------+       +----+     +--------------+      +----------+
    Cases where there is no intersecting segment between the curves are not overlapping.   
***********************************************************************/
static SmBoolean sm_AreOverlapping
 (const SmCurveInterval  & crCI,  // in : Tgt CurveInterval to test for Overlap with
  const SmEdgeDefinition & crED)  // in : Tgt EdgeDefinition 
{
  // check assumptions - gwc: I added this check - is it true?
  SM_ASSERT_MSG(   crED.GetSrcTopo(1) == crCI.m_vMid.GetObject()
                || crED.GetSrcTopo(2) == crCI.m_vMid.GetObject(), _T("sm_AreOverlapping CurveInteval and EdgeDefinition come from the same source assumption is broken - needs review")) ; 
  // locals
  const SmCurve   * cpCICurve = crCI.GetCurveClassification()->GetCurve();
        SmExtent1d  sCIDomain = crCI.GetInterval();
                    
  const SmCurve   * cpEDCurve = crED.GetXSectCurve3d();
        SmExtent1d  sEDDomain = crED.GetXSectInterval();

  // tolerance 
  SmZoneTol3d sZoneTol3d = smos_Max( SmTol::GetSrcZoneTol3d( &crCI ), crED.GetTolerance()) ;

  // Drop ED->Curve Start and End Pts to CI->CurveClassification->Curve
    {
      // locals
      SmBoolean     bOk;
      SmPoint3d     sStartPt;
      SmPoint3d     sEndPt;
      double        dGuess = sCIDomain.Evaluate( 0.1) ;
      double        dDist ;
      double        dDrop ;
      //SmScaledZero  sScaleZero = SmTol::GetScaledZero( sCIDomain) ;
      //SmZoneTol3d   sParamTol = sScaleZero.val;

      // EdgeDefinition->Curve->StartPt
      cpEDCurve->EvaluatePoint( sEDDomain.GetMin(), sStartPt) ;

      // Drop EdgeDefinition>XSectCurve MinPt to CurveClassification Curve
      SmStatus eStat = cpCICurve->DropPoint( sCIDomain,         // in : target curve allowed domain
                                             sStartPt,            // in : Point to drop to curve
                                             NULL,              // in : For Snapping, NULL=NoSnap, Vec pointing to CrvInside from desired EndPt for snapping.
                                                                //      when DropPt is within dDistTol in cpVecToCrvInside dir of EndPt, Snap to Ivl EndPt.
                                                                //      Vec handles ambiguities and makes sure that curves are not just touching at the ends.
                                             sZoneTol3d,        // in : Used for SnapDist, and SM_SO_INTERSECT sXSectTol3d distance.
                                                                //      sXSectTol3d = dDistTol ;      SM_SO_INTERSECT 
                                                                //      sXSectTol3d = SM_BIG_DOUBLE ; SM_SO_MINIMIZE, SM_SO_NORMALIZE
                                                                //      sSnapTol3d  = dDistTol ;      SM_SO_MINIMIZE, SM_SO_NORMALIZE, SM_SO_INTERSECT
                                             &dGuess,           // in : If given, call cheap LocalPointSolve() else call expensive GlobalPointSolve()
                                             bOk,               // out: TRUE = found a drop point
                                             dDrop,             // out: found drop curve param
                                             dDist,             // out: found drop distance
                                             SM_SO_INTERSECT) ; // in : SM_SO_MINIMIZE = allow nonNormal drops near endPoints
                                                                //      SM_SO_INTERSECT= like Minimize but point must be within dDistTol
                                                                //      SM_SO_NORMALIZE= exclude nonNormal drops near endPoints
                                                                //      default:[SM_SO_MINIMIZE] to preserve original behavior  
      // When StartPoint dropped within Tol to CC->Curve, check for overlap
      if(eStat == SM_SUCCESS && bOk )
        {
          // when EDCurve->StartPt is interior to CICurve - the CI and CD Curves overlap
          if(sm_IsInterior(cpCICurve,    // in : TgtCurve
                           sCIDomain,    // in : TgtCurve Ivl domain
                           dDrop,        // in : TgtParam to test
                           sZoneTol3d))  // in : Max dist between a test point and Tgt Ivl end considered in the interior
            { return TRUE; }
        }

      // EdgeDefinition->Curve->EndPt
      cpEDCurve->EvaluatePoint( sEDDomain.GetMax(), sEndPt) ;
      dGuess = sCIDomain.Evaluate( 0.9) ;

      // Drop EdgeDefinition>XSectCurve EndPt to CurveClassification Curve
      eStat = cpCICurve->DropPoint( sCIDomain,          // in : target curve allowed domain
                                    sEndPt,             // in : Point to drop to curve
                                    NULL,               // in : For Snapping, NULL=NoSnap, Vec pointing to CrvInside from desired EndPt for snapping.
                                                        //      when DropPt is within dDistTol in cpVecToCrvInside dir of EndPt, Snap to Ivl EndPt.
                                                        //      Vec handles ambiguities and makes sure that curves are not just touching at the ends.
                                    sZoneTol3d,         // in : Used for SnapDist, and SM_SO_INTERSECT sXSectTol3d distance.
                                                        //      sXSectTol3d = dDistTol ;      SM_SO_INTERSECT 
                                                        //      sXSectTol3d = SM_BIG_DOUBLE ; SM_SO_MINIMIZE, SM_SO_NORMALIZE
                                                        //      sSnapTol3d  = dDistTol ;      SM_SO_MINIMIZE, SM_SO_NORMALIZE, SM_SO_INTERSECT
                                    &dGuess,            // in : If given, call cheap LocalPointSolve() else call expensive GlobalPointSolve()
                                    bOk,                // out: TRUE = found a drop point
                                    dDrop,              // out: found drop curve param
                                    dDist,              // out: found drop distance
                                    SM_SO_INTERSECT) ;  // in : SM_SO_MINIMIZE = allow nonNormal drops near endPoints
                                                        //      SM_SO_INTERSECT= like Minimize but point must be within dDistTol
                                                        //      SM_SO_NORMALIZE= exclude nonNormal drops near endPoints
                                                        //      default:[SM_SO_MINIMIZE] to preserve original behavior
      // When EndPoint dropped within Tol to CC->Curve, check for overlap
      if(eStat == SM_SUCCESS && bOk )
        {
          // when EDCurve->EndPt is interior to CICurve - the CI and CD Curves overlap
          if(sm_IsInterior(cpCICurve,   // in : TgtCurve
                           sCIDomain,   // in : TgtCurve Ivl domain
                           dDrop,       // in : TgtParam to test
                           sZoneTol3d)) // in : Max dist between a test point and Tgt Ivl end considered in the interior
            { return TRUE; }
        }
    } // end scope dropping ED->Curve->EndPts to CI->Curve

  // arrive here when: EdgeDefininition Start and End Pt3ds do not drop to the interior of the CI XSectCurve
  // next: see if CI XSectCurve Start and End Pt3ds do drop to the interior of the ED curve

  // Drop CI->Curve->EndPts to ED->Curve
    {
      SmPoint3d     sStartPt;
      SmPoint3d     sEndPt;
      double        dGuess = sCIDomain.Evaluate( 0.1) ;
      double        dDist  = 0.0 ; 
      double        dDrop  = 0.0 ;
      //SmScaledZero  sScaleZero = SmTol::GetScaledZero( sCIDomain) ;
      //SmZoneTol3d   sParamTol = sScaleZero.val;
      SmBoolean     bOk;

      // CurveClassification Curve StartPt
      cpCICurve->EvaluatePoint( sCIDomain.GetMin(), sStartPt) ;

      // Drop CurveClassification Curve StartPt to EdgeDefinition>Curve
      SmStatus eStat = cpEDCurve->DropPoint(sEDDomain, 
                                            sStartPt, 
                                            NULL, 
                                            sZoneTol3d, 
                                            &dGuess,
                                            bOk, 
                                            dDrop, 
                                            dDist, 
                                            SM_SO_INTERSECT) ;

      // When StartPoint dropped within Tol to ED->Curve, check for overlap
      if(eStat == SM_SUCCESS && bOk )
        {
          // when CICurve->StartPt is interior to the EDCurve - the CI and CD Curves overlap
          if(sm_IsInterior( cpEDCurve, sEDDomain, dDrop, sZoneTol3d ) )
            { return TRUE; }
        }

      // CurveClassification Curve EndPt
      cpCICurve->EvaluatePoint( sCIDomain.GetMax(), sEndPt) ;
      dGuess = sEDDomain.Evaluate( 0.9) ;

      // Drop CurveClassification Curve EndPt to EdgeDefinition>Curve
      eStat = cpEDCurve->DropPoint(sEDDomain, 
                                   sEndPt, 
                                   NULL, 
                                   sZoneTol3d, 
                                   &dGuess,
                                   bOk, 
                                   dDrop, 
                                   dDist, 
                                   SM_SO_INTERSECT) ;

      // When EndPoint dropped within Tol to ED->Curve, check for overlap
      if(eStat == SM_SUCCESS && bOk )
        {
          // when CICurve->MaxPt is interior to the EDCurve - the CI and CD Curves overlap
          if(sm_IsInterior( cpEDCurve, sEDDomain, dDrop, sZoneTol3d ) )
            { return TRUE; }
        }
    } // end scope dropping CI to ED

  // arrive here when none of the CI or CD curve endpoints drop to the interior of the other curve - CI and CD curves do not overlap
  return FALSE;

} // end sm_AreOverlapping

/*******************************************************************//**
PURPOSE: Gather list of ProtoEdges whose Edgedefinition->CurveIntervals
         overlap with the input CurveIntervalPair->CurveInterval

USAGE NOTES---- 
   Outputs:
    rOverlappingPEs : List of SmProtoEdges that overlap with this CurveInterval
***********************************************************************/
SmStatus SmProtoTopologyManager::CheckOverlappingProtoEdges
 (const SmCurveInterval  & crCI1,           // in : Brep 1 CurveInterval Classification 
  const SmCurveInterval  & crCI2,           // in : Brep 2 CurveInterval Classification 
  SmTArray<SmProtoEdge*> & rOverlappingPEs) // out: List of SmProtoEdges that overlap with this CurveInterval
{
#ifdef SM_DEBUG_CODE
  SmBoolean bDebugMe = FALSE;
  if(bDebugMe )
    {
      crCI1.Dump();
      crCI2.Dump();
    }
#endif // SM_DEBUG_CODE
  
  // locals
  const SmPointClass & crMid1     = crCI1.m_vMid ;           // MidPt CurveInterval Classifications tell if this XSectCurve Ivl sits on a Brep1 Region, Face, or Edge.
  const SmPointClass & crMid2     = crCI2.m_vMid ;           // MidPt CurveInterval Classifications tell if this XSectCurve Ivl sits on a Brep2 Region, Face, or Edge.
  SmEdge             * pBrepEdge1 = crMid1.GetEdgeObject() ; // get MidPtClassification's Edge Objects or NULL when Object is not an Edge
  SmEdge             * pBrepEdge2 = crMid2.GetEdgeObject() ; // get MidPtClassification's Edge Objects or NULL when Object is not an Edge
  
  // no work - neither MidPt-CurveInterval-Classification object is an edges. gwc note: is the rule that this has to be a pair, then the && should be a ||
  if(   pBrepEdge1 == NULL 
     && pBrepEdge2 == NULL)
    { return SM_SUCCESS; }
  
  // arrive here when at least one of the input CI's classifies to an Existing Brep->Edge.
  
  // locals
  ULONG ii ;
  SmTopology * pCIMid_Object1 = SM_CAST_PTR( SmTopology, crMid1.GetObject()) ;
  SmTopology * pCIMid_Object2 = SM_CAST_PTR( SmTopology, crMid2.GetObject()) ;
  SmTArray< SmEdgeDefinition* > sEdgeDefs ;
  this->GetAllEdgeDefinitions( sEdgeDefs) ;
  ULONG lNumEDs = sEdgeDefs.GetSize() ;

  // for every EdgeDefinition in this SmProtoTopologyManager
  for(ii=0;ii<lNumEDs;ii++)
    {
      SmEdgeDefinition *pED = sEdgeDefs[ii];
  
      // no work - EdgeDefinition source TopoObjs are not the same as the input CuerveInterval classification midIvl objects.
      if(   pED->GetSrcTopo(1) != pCIMid_Object1 
         || pED->GetSrcTopo(2) != pCIMid_Object2)
        { continue; }
  
      // When the CurveInterval and EdgeDef curve intervals intersect over a segment. 
      if(sm_AreOverlapping( crCI1, *pED ) )
        {
          // remember that the ED->ProtoEdge shape overlaps with this CI->Interval of the CI->CC->Curve shape
          SmProtoEdge * pPE = pED->GetProtoEdge();
          rOverlappingPEs.Add(pPE) ;
        }
  
    } // end iter every EdgeDef looping for overlap
  
  // all done
  return SM_SUCCESS;
  
} // end SmProtoTopologyManager::CheckOverlappingProtoEdges

/*******************************************************************//**
PURPOSE: Find or create a ProtoEdge from two SmCurveIntervals.

USAGE NOTES---- 
   If both CurveIntervals classify to topology, creates an SmProtoEdge.
   Looks for existing ProtoVertices for each end, and creates new if not found.
   Adds the new ProtoEdge to 'this', and returns it as rpNewPE.
***********************************************************************/
SmStatus SmProtoTopologyManager::FindOrCreateProtoEdge
 (SmCurveInterval & rCI1,          // in : Face1 CrvInterval classification from a face pair XSection
  SmCurveInterval & rCI2,          // in : Face2 CrvInterval classification from a face pair XSection
  SmProtoEdge    *& rpNewPE,       // out: new or found ProtoEdge
  SmBoolean       * pbWasCreated ) // out: optional, TRUE  = rpNewPE was created, NULL to ignore, default:[NULL]
                                   //                FALSE = rpNewPE was found
{
  // Locals
  ULONG ii;
  
  // Init output.
  rpNewPE = NULL;
  if(pbWasCreated != NULL ) { *pbWasCreated = FALSE; }

  // locals
  SmStatus                 eStat        = SM_SUCCESS ;
  const SmCurve          * pCC3dCurve   = rCI1.GetCurve() ;
  SmExtent1d               sThisIvl     = rCI1.GetInterval() ;
  double                   dTol         = m_pTI->GetThisApproxTol3d() ; //cbiTol.
  SmBoolean                bIsIdentical = FALSE;
  SmTArray<SmProtoEdge*>   sOverlappingPEs ;

  // DD3: Check for overlapping PEs.
  // For now, just find overlapping PEs. Will be handled during Resolve()
  CheckOverlappingProtoEdges( rCI1, rCI2, sOverlappingPEs) ;

  // when CrvInterval is degenerate - find or create a ProtoVertex for Interval
  if(pCC3dCurve->IsDegenerate( dTol ) )    // in : min distance between unique points, 0 = use dScaledZero
                                           //      default:[SM_EFF_ZERO]
                                           // in : interval to examine, NULL = use Natural Interval 
                                           //      default:[NULL]
    {
      SmProtoVertex *pPV = NULL ;

      // gwc: Is this the right thing to do here?  Creating ProtoVertices from degenerate intersections make sense
      //       for points of intersection that don't show up in any other intersection solutions.
      //       However when a point of intersection does show up in other intersection solutions maybe the right thing to do
      //       is skip it since that would be adding multiple definitions of the same intersection to the SmProtoTopologyMgr.
      //      1. Add the VerteDef to an existing ProtoVertex for coincident Point solutions.
      //      2. For XSectPoint solutions coincident with other XSectInterval solutions:
      //         a. do we need a check here that prevents adding a vertex solution where an existing ProtoEdge-EdgeDef already exists?
      //            or create a new list of SmProtoEdge->m_sOverlappingPVs (just like the m_sOverlappingPEs list) and let Resolve figure it out.)
      //         b. do we need a check in the next nonDegen-CurveInterval branch that finds when a new protoEdge intersects
      //             an existing ProtoVertex and removes that ProtoVertex or create a new list of SmProtoEdge->m_sOverlappingPVs 
      //               (just like the m_sOverlappingPEs list) and let Resolve figure it out.)

      // find or create ProtoVertex for CrvClass->PtClassification (start, mid, or end) that classifies to topo in both Faces
      eStat =   (   sm_CanImprint(rCI1.m_vStart.GetPointClass())
                 && sm_CanImprint(rCI2.m_vStart.GetPointClass()) ) ? FindOrCreateProtoVertex( rCI1.m_vStart, rCI2.m_vStart, pPV)
              : (   sm_CanImprint(rCI1.m_vMid.  GetPointClass())
                 && sm_CanImprint(rCI2.m_vMid.  GetPointClass()) ) ? FindOrCreateProtoVertex( rCI1.m_vMid,   rCI2.m_vMid,   pPV)
              : (   sm_CanImprint(rCI1.m_vEnd.  GetPointClass())
                 && sm_CanImprint(rCI2.m_vEnd.  GetPointClass()) ) ? FindOrCreateProtoVertex( rCI1.m_vEnd,   rCI2.m_vEnd,   pPV)
              : SM_SUCCESS ;

      // all done
      return eStat;

    } // end Degen-CrvInterval branch
  
  // arrive here for NonDegen CrvIntervals
    { // start scope for NonDegen-CrvInterval branch
  
      // Find or create PVs for the ends.
      SmProtoVertex * pStartPV = NULL;
      SmProtoVertex * pEndPV   = NULL;
    
      // Start EndPt
      SmPointClassificationType eStartType1 = rCI1.m_vStart.GetPointClass();
      SmPointClassificationType eStartType2 = rCI2.m_vStart.GetPointClass();
    
      if(   sm_CanImprint(eStartType1) 
         && sm_CanImprint(eStartType2))
        {
          eStat = this->FindOrCreateProtoVertex( rCI1.m_vStart, rCI2.m_vStart, pStartPV) ;
        }
    
      // End EndPt
      SmPointClassificationType eEndType1 = rCI1.m_vEnd.GetPointClass();
      SmPointClassificationType eEndType2 = rCI2.m_vEnd.GetPointClass();
    
      if(   sm_CanImprint(eEndType1)  
         && sm_CanImprint(eEndType2))
        {
          eStat = this->FindOrCreateProtoVertex( rCI1.m_vEnd, rCI2.m_vEnd, pEndPV) ;
        }
    
      // Q: If something fails, do we want to keep the new PVs?
      // A: Yes. (prog_test, 1st case.  c.f. bMergeSingularities in SmTopologyIntersector.cpp.)
    
      // Can't create a ProtoEdge without both PVs.
      if(   pStartPV == NULL 
         || pEndPV   == NULL )
        { return SM_SUCCESS; }
    
      // arrive here with Start and End ProtoVerts
    
      // Find or create SmProtoEdge for the Mid, if appropriate.  
      SmPointClassificationType eMidType1 = rCI1.m_vMid.GetPointClass();
      SmPointClassificationType eMidType2 = rCI2.m_vMid.GetPointClass();
    
      // quit - IntervalMid does not classify to topology in both SrcBreps (gwc: can this test be made sooner to save some time)
      if(   sm_CanImprint(eMidType1) == FALSE  
         || sm_CanImprint(eMidType2) == FALSE)
        { return SM_SUCCESS; }
    
      // quit - SrcFaces intersect at a single Vertex, then CurveClassification->Intervals will  
      // have been created with Start, Mid, and End all on that Vertex.    (gwc: can this test be made sooner to save some time)
      if(   eMidType1 == SM_PC_VERTEX  
         || eMidType2 == SM_PC_VERTEX)
        { return SM_SUCCESS; }
    
      // quit - degenerate interval. [B488],                               (gwc: can this test be made sooner to save some time)
      if(pCC3dCurve->IsDegenerate(dTol, 
                                  &sThisIvl))
        { return SM_SUCCESS; }
    
      // See whether a ProtoEdge already exists in this location.
      SmProtoEdge * pNewPE = this->FindProtoEdge      // rtn: found PE with ED with same shape (ED->XSectCrv3d within CaptureDistance) as TgtCurve3d
                                     (pStartPV,       // in : Tgt Start ProtoVertex
                                      pEndPV,         // in : Tgt End   ProtoVertex
                                      rCI1.m_vMid,    // in : Tgt MidPt Classification for face1->CrvClass->TgtCrvInterval
                                      rCI2.m_vMid,    // in : Tgt MidPt Classification for face2->CrvClass->TgtCrvInterval
                                      pCC3dCurve,     // in : shared Tgt XSectCurve3d being classified against face1/face2 pair
                                      sThisIvl,       // in : TgtCrvInterval domain
                                      bIsIdentical) ; // out: TRUE  = returned PE->ED also shares the same SrcTopos as the input XSect Interval
                                                      //      FALSE = returned PE->ED only has the same shape as input XSectInterval but different SrcTopos
      
      // when no ProtoEdge already exists in this location branch
      if(pNewPE == NULL)
        {
          // create new Protoedge and EdgeDefinition
          eStat = this->CreateProtoEdge(pStartPV,  // in : start ProtoVertex
                                        pEndPV,    // in : end ProtoVertex
                                        rCI1,      // in : CurveIvl from SrcTopo 1
                                        rCI2,      // in : CurveIvl from SrcTopo 2
                                        pNewPE) ;  // out: new ProtoEdge
    
          // when New ProtoEdge was created
          if(eStat == SM_SUCCESS && pNewPE != NULL)
            { 
              // set output
              if(pbWasCreated != NULL) { *pbWasCreated = TRUE; } 
    
              // Add list of ProtoEdges whose EdgeDefs have an overlapping segment with this CurveInterval->Curve
              pNewPE->AppendOverlappingPEs(sOverlappingPEs) ;
    
              // for every OverlappingPE - add packpointers to pNewPE
              for(ii=0;ii<sOverlappingPEs.GetSize();ii++)
                { 
                  sOverlappingPEs[ii]->AddOverlappingPE(pNewPE) ; 
                }
            } // end New ProtoEdge was created check
        } // end need to create new ProtoEdge branch
    
      // else when a NonIdentical ProtoEdge exists in this location branch
      else if(bIsIdentical == FALSE)
        {
          // Create new EdgeDef and add it to NewPE's EdgeDef list
          pNewPE->AddEdgeDefinition(rCI1, 
                                    rCI2) ;
        } // end when a NonIdentical ProtoEdge exists in this location branch
    
      // else an identical ProtoEdge already exists branch
      //  - nothing to do - don't add multiple copies of identical ProtoEdge definitions
    
      // set output
      rpNewPE = pNewPE ;

    } // end scope for NonDegen-CrvInterval branch 

  // all done
  return SM_SUCCESS;

} // end SmProtoTopologyManager::FindOrCreateProtoEdge

/*******************************************************************//**
PURPOSE: Create a ProtoFace.

USAGE NOTES---- 
***********************************************************************/
SmStatus SmProtoTopologyManager::CreateProtoFace
 (SmFace                   * pF1,        // in : Brep1->face of Brep1/Brep2 coincident face pair
  SmFace                   * pF2,        // in : Brep2->face of Brep1/Brep2 coincident face pair
  SmTArray< SmProtoEdge* > & rCoincPEs,  // in : array Brep1/Brep2 Coincident edge pairs that bound pF1 and pF2 repsectively
  SmProtoFace             *& pNewPF )    // out: The new SmProtoFace object
{

// cbi latest thought: create only one PF for each coincidence, and give it all of the PE's
//                     that are created for that intersection.  When an imprint splits off
//                     a new face, check all PE's: if any are unprocessed, OR are not
//                     connected to the new face, create a new PF with them.

// cbi Earlier thought: Create a ProtoFace for each set of connected ProtoEdges.
//                      This is because two Faces can overlap in more than one place.

  // new SmProtoFace
  pNewPF = new ( this ) SmProtoFace( pF1, pF2, rCoincPEs, this) ;  NER( pNewPF) ;

  // update the owning SmProtoTopologyManager m_sPFs list
  this->AddProtoFace( pNewPF) ;

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
  if(bDebugMe)
    {
      SmBrep * pBrep1 = pF1->GetBrep() ;
      SmBrep * pBrep2 = pF2->GetBrep() ;

      smgfx_Erase();
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep1) pBrep1->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,0) ; if(pBrep2) pBrep2->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook( 2,4, 1,0,1) ; this->DrawDebug(); sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

// < code removed after v. 8370. >

  // all done
  return SM_SUCCESS;

} // end SmProtoTopologyManager::CreateProtoFace

/*******************************************************************//**
PURPOSE: If a Brep Edge was split, update all ProtoTopology that sat on that edge.

USAGE NOTES---- 
  1. Currently only called in SmVertexDefinition::Imprint()
     which I think is only called in SmTopologyIntersector::ImprintIntersections()

gwc: notes:
  1. I don't see how trim works as the way to update a PE when an Edge is split.
     Trimming an original PE to match the interval of one of the SplitEdge children
     only modifies one child PE and doesn't create a 2nd PE child for the 2nd Edge split child.
     The only thing I can imagine is that UpdateEdges() is only being called in 
     SmTopologyIntersector::ImprintIntersections() and by the time its called, 
       a. properly splitting the PE edges has already been done by SplitIntersectingProtoEdges() 
          and SPlitProtoEdgeAtProtoVertex(), or
       b. ImprintIntersections() is already done using this data and any errors here 
          don't affect the boolean outcome.
     This concern needs to be reviewed.
  2. A list of ProtoEdges is stored on the PF->m_sCoincPEs list. This method
     is not currently updating those PE lists.  Again, I'm not sure that errors in those lists
     affect the final Boolean Brep modification.  And another point, as long as this method
     is not splitting PEs because its trimming them instead, then the PF->m_sConincPEs list
     are not getting out of date.  There's no new PE objects to add to those lists.
  3. I suspect this function is not doing what its advertised as doing but for reasons
     I have not yet seen, this does not impact the final Boolean Brep result.
***********************************************************************/
SmStatus SmProtoTopologyManager::UpdateEdges
 (int                        iWhichBrep,  // in : 1 = From Brep1, 2 = from Brep2
  SmEdge                   * pOldEdge,    // in : original Edge
  const SmTArray<SmEdge *> & crNewEdges,  // in : result of splitting pOldEdge with new Vertex, expect crNewEdges->GetSize() == 0 or 2
  SmVertexDefinition       * pCallingVD ) // in : Optional VertexDefinition being imprinted, NULL to ignore, default:[NULL]
{
  // no work - no split edge
  if(pOldEdge == NULL ) // Not a problem.
    { return SM_SUCCESS ; }

  // check state - crNewEdges Count should be 0 or 2
  SM_ASSERT_MSG( crNewEdges.GetSize() == 0 || crNewEdges.GetSize() == 2, _T("Warning: SmProtoTopologyManager::UpdateEdges(): Unexpected number of edges.\n")) ;
  
  // no work - not 2 NewEdges
  if(crNewEdges.GetSize() != 2)
    { return SM_SUCCESS ; }

  // arrive here when we split an edge in two.  
  // next: Check all ProtoVertices and ProtoEdges that sit on the original SmEdge.

  // locals 
  ULONG ii ;
  SmEdge    * pNewEdge =   (crNewEdges[0] == pOldEdge) ? crNewEdges[1]
                      : (crNewEdges[1] == pOldEdge) ? crNewEdges[0]
                      : NULL ; 
  NER_MSG(pNewEdge, _T("Error: SmProtoTopologyManager::UpdateEdges(): Input NewEdges list did not contain the pOldBrep ptr.\n")) ;
  SmExtent1d  sOldIvl  = pOldEdge->GetInterval();
  SmExtent1d  sNewIvl  = pNewEdge->GetInterval();

//cbi optimize: should be able to point directly from pOldEdge to all PVs and PEs on it.
//cbi Attributes? Maps?

  // move VertexDefinitions between split Child1 (pOldEdge) and Child2 (pNewEdge)
  SmTArray< SmVertexDefinition* > sVtxDefs;
  this->GetAllVertexDefinitions( sVtxDefs) ;

  // for every VertexDefinition - move ProtoVertices classifying to ChildEdge2
  for(ii=0;ii<sVtxDefs.GetSize();ii++)
    {
      SmVertexDefinition *pVD = sVtxDefs[ii];

      // no work - no VD
      if(pVD == NULL )
        { continue; }

      // no work - VD is CallingVD
      if(pVD == pCallingVD ) // No need to process the one being imprinted.
        { continue; }

      // no Work pVD SrcTopo is not an Edge
      if(pVD->GetSrcTopo_TYPE( iWhichBrep ) != SM_PC_EDGE )
        { continue; }

      // locals
      SmTopology * pBrepTopo = pVD->GetSrcTopo( iWhichBrep) ;
      SmEdge     * pEdge     = SM_CAST_PTR( SmEdge, pBrepTopo) ;
      if(pEdge == NULL) { WARN( _T("Warning: SmProtoTopologyManager::UpdateEdges(): Type/Topology mismatch.\n")) ; }

      // no work - pVD->Edge is not Tgt SplitEdge
      if(pEdge != pOldEdge )
        { continue; }

      // arrive here when pVD   = ProtoVertex on the Edge that was split.
      //                  pEdge = pVD SrcTopo
      // next: classify pVD to pNewEdge and pOldEdge
      //       Note: pVD can be on both -- if it has already been imprinted.
      //             In that case, it can stay on the original.  
      //             So if bOnOldEdge is True, then we don't have to do anything.  
      //             else we'll move pVD to the new Edge

      // locals
      double       dEdgeTParam = pVD->GetBrepTParam( iWhichBrep) ;
      SmScaledZero dScaledZero = SmTol::GetScaledZero( sOldIvl) ;
      SmBoolean    bOnOldEdge = sOldIvl.ContainsValue( dEdgeTParam, dScaledZero) ;
      dScaledZero = SmTol::GetScaledZero( sNewIvl) ;

      // classify dEdgeTParam against New Edge interval
      SmBoolean bOnNewEdge = sNewIvl.ContainsValue( dEdgeTParam, dScaledZero) ;

      // when pVD is only on the new Edge - change its srcTopo to the new Edge
      if(bOnNewEdge && ! bOnOldEdge )
        {
          pVD->SetSrcTopo( iWhichBrep, SM_PC_EDGE, pNewEdge) ;
        }
    } // end iter every VertexDef chaning the SrcTopo for those only on NewEdge 

  // arrive here: After all ProtoVertices have been moved to their containing ChildEdges
  // next: move EdgeDefinitions between split Child1 (pOldEdge) and Child2 (pNewEdge)
  SmTArray< SmEdgeDefinition* > sEdgeDefs;
  this->GetAllEdgeDefinitions( sEdgeDefs) ;

  // Get the split point:  It's one end of pOldEdge.
  SmPoint3d sSplitPt;
  double    dEndParam =   (sOldIvl.GetMid() < sNewIvl.GetMid() )  //cbi always TRUE I think
                        ? sOldIvl.GetMax()
                        : sOldIvl.GetMin();
  pOldEdge->GetCurve()->EvaluatePoint( dEndParam, sSplitPt) ;

  // locals
  SmProtoVertex * pCallingPV = pCallingVD->GetProtoVertex();

  // Temporary: possibly split ProtoEdge instead of trim.
  // I think Split is correct, but currently Trim is working better.
static constexpr SmBoolean cbiDoSplit = FALSE;

  // for every EdgeDef
  for(ii=0;ii<sEdgeDefs.GetSize();ii++)
    {
      SmEdgeDefinition *pED = sEdgeDefs[ii];

      // First check whether this EdgeDef starts or ends on the ProtoVertex
      // of the VertexDef that split the SmEdge.
      SmProtoEdge * pThisPE = pED->GetProtoEdge() ;
      SmPoint3d     sEndPos ;
      double        dSplitParam ;
      double        dOtherParam ;
      SmBoolean     bDoTrim = FALSE ;

      // CallingPV is EdgeDef Start
      if(pThisPE->GetStartProtoVertex() == pCallingPV)
        {
          SmVertex    * pSplitVtx = pCallingVD->GetFinalVertex( iWhichBrep) ;
          SmXSectTol3d  sTol3d    = SmTol::GetXSectTol3d( pOldEdge, pSplitVtx) ;

          // Split pt is inside the pED - trim pED
          bDoTrim = pED->IsInteriorPoint( sSplitPt, sTol3d, TRUE, dSplitParam) ;
          if(bDoTrim)
            {
              // First check: if we start and end on the same ProtoVertex,
              // make sure we're interior w.r.t. the other end as well.
              if(pThisPE->GetStartProtoVertex() == pThisPE->GetEndProtoVertex() )
                { bDoTrim = pED->IsInteriorPoint( sSplitPt, sTol3d, FALSE, dOtherParam) ; }

              if(bDoTrim)
                {
                  // switch between split and trim - cbi: currently, trim is working better
                  //                                 gwc: I don't see how trim works. When an edge is split into two children
                  //                                      the associated protoEdge has to be split into two PEs and I don't
                  //                                      see trim doing that.  It just modifies one PE.
                  if(cbiDoSplit) 
                    { //cbi split instead of trim.
                      SmProtoVertex *pNewPV;
                      SmProtoEdge   *pNewPE;
                      pThisPE->Split( dSplitParam, pNewPV, pNewPE) ;
                    } 
                  else //cbi trim instead of split.
                    {
                      SmExtent1d sDomain( pED->GetXSectInterval()) ;
                      pED->SetXSectInterval( dSplitParam, sDomain.GetMax()) ;
                      
                      // Also have to trim the curves: we want these to be in sync.
                      sDomain = pED->GetXSectInterval();  // Just updated.
                      pED->GetXSectCurve3d()->Trim( sDomain) ;
                      
                      SmCurve * pIntCurveUV = pED->GetXSectUVCurve( 1) ;
                      if(pIntCurveUV != NULL )
                        { pIntCurveUV->Trim( sDomain) ; }

                      pIntCurveUV = pED->GetXSectUVCurve( 2) ;
                      if(pIntCurveUV != NULL )
                        { pIntCurveUV->Trim( sDomain) ; }

                      //cbi: what about our PE and its other EDs?

                    } //cbi. end trim instead of split branch
                } // end 2nd do Trim check
            } // end 1st do Trim check
        } // end CallingPV is EdgeDef Start check

      // CallingPV is EdgeDef end
      if(pThisPE->GetEndProtoVertex() == pCallingPV)
        {
          SmVertex *pSplitVtx = pCallingVD->GetFinalVertex( iWhichBrep) ;
          SmXSectTol3d sTol3d = SmTol::GetXSectTol3d( pOldEdge, pSplitVtx) ;

          // Split pt is inside the pED - trim pED
          bDoTrim = pED->IsInteriorPoint( sSplitPt, sTol3d, FALSE, dSplitParam) ;
          if(bDoTrim)
            {
              // First check: if we start and end on the same ProtoVertex,
              // make sure we're interior w.r.t. the other end as well.
              if(pThisPE->GetStartProtoVertex() == pThisPE->GetEndProtoVertex() )
                { bDoTrim = pED->IsInteriorPoint( sSplitPt, sTol3d, TRUE, dOtherParam) ; }

              if(bDoTrim)
                {
                  // switch between split and trim - cbi: currently, trim is working better
                  //                                 gwc: I don't see how trim works. When an edge is split into two children
                  //                                      the associated protoEdge has to be split into two PEs and I don't
                  //                                      see trim doing that.  It just modifies one PE.
                  if(cbiDoSplit) 
                    { //cbi split instead of trim.
                      SmProtoVertex *pNewPV;
                      SmProtoEdge   *pNewPE;
                      pThisPE->Split( dSplitParam, pNewPV, pNewPE) ;
                    } 
                  else //cbi trim instead of split
                    {
                      SmExtent1d sDomain( pED->GetXSectInterval()) ;
                      pED->SetXSectInterval( sDomain.GetMin(), dSplitParam) ;
                      
                      // Also have to trim the curves: we want these to be in sync.
                      sDomain = pED->GetXSectInterval();  // Just updated.
                      pED->GetXSectCurve3d()->Trim( sDomain) ;
                      
                      SmCurve *pIntCurveUV = pED->GetXSectUVCurve( 1) ;
                      if(pIntCurveUV != NULL )
                        { pIntCurveUV->Trim( sDomain) ; }
                      pIntCurveUV = pED->GetXSectUVCurve( 2) ;
                      if(pIntCurveUV != NULL )
                        { pIntCurveUV->Trim( sDomain) ; }
                      
                      //cbi: what about our PE and its other EDs?
                    } //cbi.              
                } // end 2nd do Trim check
            } // end 1st do Trim check
        } // end CallingPV is EdgeDef End check

      // Now check whether this EdgeDefintion is defined on the split SmEdge.
      if(pED->GetSrcTopo_TYPE( iWhichBrep ) != SM_PC_EDGE )
        { continue; }

      // locals
      SmTopology * pBrepTopo = pED->GetSrcTopo( iWhichBrep) ;
      SmEdge     * pEdge     = SM_CAST_PTR( SmEdge, pBrepTopo) ;
      if(pEdge == NULL) { WARN( _T("Warning: SmProtoTopologyManager::UpdateEdges(): Type/Topology mismatch.\n")) ; }

      // no work - pVD->Edge is not Tgt SplitEdge
      if(pEdge != pOldEdge)
        { continue ; }

      // Found a ProtoEdge on the edge that was split.
      // (Must be a coincident case.)
      // We have to do a geometric test here.
      // Drop the midpoint of each Edge (original and new) to our curve.
      // We have the parameter range on that curve, m_pXSectInterval.
      // One should be the midpoint of our interval on the curve.
      // But no: that works only if it was originally one piece and just
      // got split into two.  If the curve is classified into many different
      // intervals, such as on a Face with holes in it, then the midpoints of
      // the two Edge pieces will not drop to the interiors of all of the
      // Face-classified intervals.  So, we drop the midpoint of each Curve Interval
      // to both Edge intervals: it should be in the interior of one, and not in
      // the other.
      SmSolutionArray sSolsOld, sSolsNew;

      SmPoint3d sMidPt;
      pED->EvaluateMid( &sMidPt) ;

      // Drop to pOldEdge and pNewEdge using sOldIvl and sNewIvl.
      // Use GlobalPointSolve (we don't have guesses),
      // with Intersect (should be on one and not on the other).
      
//cbiTol blatantly made up:
      // Tolerance: with INTERSECT, the tol is the max allowable distance.
      // Should be pretty big; I don't think this can get us into trouble here.
      double dThisTol = smos_Max( 10*m_dTol, 4*pOldEdge->GetTolerance()) ;

      SmStatus eStat1 = pOldEdge->GetCurve()->GlobalPointSolve( sOldIvl, SM_SO_INTERSECT, sMidPt,
              dThisTol, &dThisTol, NULL, SM_SR_SINGLE, sSolsOld) ;

      dThisTol = smos_Max( 10*m_dTol, 4*pNewEdge->GetTolerance()) ;

      SmStatus eStat2 = pNewEdge->GetCurve()->GlobalPointSolve( sNewIvl, SM_SO_INTERSECT, sMidPt,
              dThisTol, &dThisTol, NULL, SM_SR_SINGLE, sSolsNew) ;

      if(   (eStat1 == SM_SUCCESS && sSolsOld.GetSize() > 0)
         || (eStat2 == SM_SUCCESS && sSolsNew.GetSize() > 0))
        {
          if(eStat1 != SM_SUCCESS || sSolsOld.GetSize() == 0 )
            {
              // Drop to Old failed (and to New succeeded).
              pED->SetSrcTopo( iWhichBrep, SM_PC_EDGE, pNewEdge) ;
              continue;  // Done with this ProtoEdge.
            }
          else if(eStat2 != SM_SUCCESS || sSolsNew.GetSize() == 0 )
            {
              // Drop to New failed (and to Old succeeded).
              // (Don't have to do anything.)
              continue;  // Done with this ProtoEdge.
            }

          // arrive here if both drops worked.  Pick the closer one.

          if(sSolsNew[0].m_vStart.m_dSolutionValue < sSolsOld[0].m_vStart.m_dSolutionValue )
            {
              // Drop to New is closer.
              pED->SetSrcTopo( iWhichBrep, SM_PC_EDGE, pNewEdge) ;
            }

          // else do nothing.
          continue ;
        }

      // If that didn't work, drop using Minimize, which should always
      // find something, and use the distances and locations in the
      // intervals to make a decision.

      continue ;  //cbi breakpoint

    } // end for each EdgeDef

  // all done
  return SM_SUCCESS;

} // end SmProtoTopologyManager::UpdateEdges

/*******************************************************************//**
PURPOSE: If a Brep Face was split, update all ProtoTopology that sat on that Face.

USAGE NOTES---- 
***********************************************************************/
SmStatus SmProtoTopologyManager::UpdateFaces
  (int                iWhichBrep,     // in : 1 = From Brep1, 2 = from Brep2
   SmFace           * pOldFace,       // in : Original Face that was split. Currently reused as SplitFace->Child1
   SmFace           * pNewFace,       // in : New Face that was split from pOldFace. SplitFace->Child2
   SmEdge           * pSplittingEdge, // in : The Edge that split the Faces.
   SmEdge           * pOtherEdge,     // in : Corresponding Edge in other Brep.
   SmEdgeDefinition * pCallingED)     // in : optional calling EdgeDef: don't have to process this one. NULL to ignore. default:[NULL]
{
  // init return
  SmStatus eStat;

  // We will be testing uv points for Face containment.
  // We can rule some out using (fairly) tight uv domains.

  // locals
  ULONG ii, jj, kk ;
  SmExtent2d sOldDomain ;
  SmExtent2d sNewDomain ;

  // Face UVDomains = Min(current FaceUVDomain, UVTrimCurve ControlPoint domain bounding box)
  eStat = pOldFace->CalculateUVDomainFromUVTrimCurves(sOldDomain) ;
  eStat = pNewFace->CalculateUVDomainFromUVTrimCurves(sNewDomain) ;

  // when appropriate reduce child OldFace UVDomain to the newly calculated one
  if(! pOldFace->GetUVDomain().IsContainedBy ( sOldDomain ) )
    { pOldFace->SetUVDomain( sOldDomain) ; }

  // when appropriate reduce child NewFace UVDomain to the newly calculated one
  if(! pNewFace->GetUVDomain().IsContainedBy ( sNewDomain ) )
    { pNewFace->SetUVDomain( sNewDomain) ; }

  // set iWhich:  1 = pOtherFace = pOldFace
  //              2 = pOtherFace = pNewFace
  //              0 = unclassified
  int iWhich = 0;

  //cbi Possible optimization: should be able to point directly from pOldFace to all PVs, PEs, and PFs on it.
  //cbi Attributes? Maps?

  // move VertexDefinitions between split Child1 (pOldFace) and Child2 (pNewFace)
  SmTArray< SmVertexDefinition* > sVtxDefs;
  this->GetAllVertexDefinitions( sVtxDefs ) ;

  // for every VertexDefinition - move ProtoVertices classifying to ChildFace2
  for(ii=0;ii<sVtxDefs.GetSize();ii++)
    {
      SmVertexDefinition *pVD = sVtxDefs[ii];

      // skip VtxDef whose SrcTopo(iWhichBrep) is not a Face
      if(pVD->GetSrcTopo_TYPE( iWhichBrep ) != SM_PC_FACE )
        { continue; }

      // source Face
      SmTopology * pBrepTopo = pVD->GetSrcTopo( iWhichBrep) ;
      SmFace     * pFace     = SM_CAST_PTR( SmFace, pBrepTopo) ;
      AE_MSG(pFace != NULL, _T("Warning: SmProtoTopologyManager::UpdateFaces(): Type/Topology mismatch.\n")) ;

      // skip VertDefs whose SrcTopo is not pOldface
      if(pFace != pOldFace )
        { continue; }

      // arrive here: when VertexDef belongs to a ProtoVertex on the ParentFace (preSplit pOldFace) that was split
      // next: deterimine if ProtoVertex belongs on Child1 (postSplit pOldFace) or Child2 (pNewFace)

      // locals
      SmPoint2d   sFaceUVParam   = pVD->GetBrepUVParam( iWhichBrep) ;                     // this is a ParentFace (preSplit pOldFace) value
      SmZoneTol3d sFaceZoneTol3d = SmTol::GetZoneTol3d_ForXSectResult( pFace, pNewFace) ; // here pFace == pOldFace

      // classify ParentFace UVParam value against the 2 ChildFaces for containment
      iWhich = sm_ClassifyParamOnFaces // rtn: Pt Classify value: 1 = On pFace1, 2 = On pFace2, 0 = not classifable (could be on both Faces)
                 (sFaceUVParam,        // in : Target UVParam to classify
                  sFaceZoneTol3d,      // in : obj ZoneTol3d assoc with UVParam, usually not this face
                                       //      if none, use:SmTol::GetZoneTol3d(BREP_CONTEXT_OR_NULL)
                  pOldFace,            // in : 1st TgtFace to check for pt containment
                  sOldDomain,          // in : TgtFace1 UV Domain
                  pNewFace,            // in : 2nd TgtFace to check for pt containment
                  sNewDomain) ;        // in : TgtFace2 UV Domain

      // when UVParam classifies to just Child1 face
      if (iWhich == 1)
        {
          // no work - ParentFace UVParam is in ChildFace1 - since ParentFace ptr was reused as ChildFace1 ptr
          continue ; 
        } // end UVParam on ChildFace1 branch
      else if (iWhich == 2)  // UVParam classified to just Child2 Face
        {
          // move the VertexDefinition
          pVD->SetSrcTopo( iWhichBrep, SM_PC_FACE, pNewFace) ;
          continue ;
        }

      // arrive here when UVParam for VertexDefinition did not classify to either ChildFace - do nothing
      continue;  //cbi breakpoint only: here is trouble.

  } // end iter ii, every VertexDefinition moving ChildFace2 ProtoVertices as needed

  // arrive here: After all ProtoVertices have been moved to their containing ChildFaces
  // next: move EdgeDefinitions between split Child1 (pOldFace) and Child2 (pNewFace)
  SmTArray< SmEdgeDefinition* > sEdgeDefs;
  this->GetAllEdgeDefinitions( sEdgeDefs) ;

  // for every EdgeDefinition 
  for(ii=0;ii<sEdgeDefs.GetSize();ii++)
    {
      SmEdgeDefinition *pED = sEdgeDefs[ii];

      // skip NULL EdgeDefinition ptrs
      if(pED == NULL)
        { continue; }

      // no work - EdgeDef being imprinted already processed
      if(pED == pCallingED) 
        { continue; }

      // locals
      SmTopology * pBrepTopo = pED->GetSrcTopo( iWhichBrep) ;
      SmFace     * pInFace   = SM_CAST_PTR( SmFace, pBrepTopo) ;

      // skip EdgeDefinitions not in the Oldface (when OldFace changed from Parent to Child1 some of the original OldFace Edges might have to be moved to Child1)
      if(pInFace != pOldFace)
        { continue; }

      // arrive here when ProtoEdge is on the Face that was split.
      // next: classify the midPoint of ProtoEdge to see which ChildFace it belongs in

      // locals
      SmPoint2d  sFaceUVParam ;
      SmCurve  * pUVCurve     = pED->GetXSectUVCurve( iWhichBrep) ;
      double     dMidParam    = pED->GetXSectInterval().GetMid() ;
      SmBoolean  bGotUV       = FALSE ;

      // Use UVCurve if available branch
      if(pUVCurve != NULL)
        {
          SmPoint3d sPt;
          if(pUVCurve->EvaluatePoint( dMidParam, sPt ) ==  SM_SUCCESS)
            {
              sFaceUVParam.Set( sPt.x, sPt.y) ;
              bGotUV = TRUE ;
            }
        } // end have UVCurve to get sFaceUVParam

      // when there is no UVCURVE branch
      if(bGotUV == FALSE)
        {
          // drop a 3d MidPoint to the surface.

          // locals
          double                dGap           = 0.0;
          SmBoolean             bSuccess ;
          SmBoolean             bIsMulti ;
          SmSurface           * pSrf           = pOldFace->GetSurface() ;
          SmExtent2d            sSurfaceDomain = pSrf->GetNaturalUVDomain() ;
          SmPoint3d             sMidPt ;
          SmSolverOperationType eOpType        = SM_SO_NORMALIZE; // Don't want off-boundary drops.

          // mid Point3d
          pED->EvaluateMid( &sMidPt) ;

          // Drop MidPt3d to Surface
          eStat = pSrf->DropPoint(sMidPt,         // in : target point to drop
                                  sSurfaceDomain, // in : target surface domain
                                  NULL,           // in : When given, uses only local solves
                                  bSuccess,       // out: TRUE=Point dropped successfully, FALSE=didn't
                                  sFaceUVParam,   // out: drop result UVPoint
                                  dGap,           // out: distance of found point to Pt to drop
                                  bIsMulti,       // out: TRUE = solution is multivalued, FALSE = Single valued
                                  eOpType) ;      // in : SM_SO_MINIMIZE = [default] allow nonNormal drops near boundaries
                                                  //      SM_SO_NORMALIZE= exclude nonNormal drops near boundaries
                                                  //      SM_SO_INTERSECT= point must be on surface, to Tol
          if(eStat == SM_SUCCESS && bSuccess )
            { bGotUV = TRUE; }
        } // end no UVCurve branch to get sFaceUVParam branch

      // check state - Not having a UVPoint here is a bug
      SM_ASSERT_MSG(bGotUV == TRUE, _T("SmProtoTopologyManager::UpdateFaces - Could not find a UVPoint to classify for a ProtoEdge - needs debug"))
      if(bGotUV == FALSE)
        {  continue ; }  //cbi breakpoint: trouble

      // arrive here when sFaceUVParam has been set to MidPoint for EdgeDefinition being classified
      // next: see which Face this uv point is in.

      // classify pEU->MidPoint against both split child faces
      SmZoneTol3d sFaceZoneTol3d = SmTol::GetZoneTol3d_ForXSectResult( pOldFace, pNewFace) ;
      iWhich = sm_ClassifyParamOnFaces  // rtn: Pt Classify value: 1 = On pFace1, 2 = On pFace2, 0 = not classifable (could be on both Faces)
                 (sFaceUVParam,         // in : Target UVParam to classify
                  sFaceZoneTol3d,       // in : obj ZoneTol3d assoc with UVParam, usually not this face
                                        //      if none, use:SmTol::GetZoneTol3d(BREP_CONTEXT_OR_NULL)
                  pOldFace,             // in : 1st TgtFace to check for pt containment
                  sOldDomain,           // in : TgtFace1 UV Domain
                  pNewFace,             // in : 2nd TgtFace to check for pt containment
                  sNewDomain) ;         // in : TgtFace2 UV Domain

      // when UVParam classifies to just Child1 face - no work - pED->PE is already in the right child face
      if     (iWhich == 1)
        { continue ; } 

      // when UVParam classifies to just Child2 Face 
      else if(iWhich == 2)  
        {
          // when pED->PE is in Child1 - move pED->PE to child2 face
          if(pInFace == pOldFace )
            { pED->SetSrcTopo( iWhichBrep, SM_PC_FACE, pNewFace) ; }
          continue ;
        }

      // arrive here when UVParam for EdgeDefinition did not classify to just one ChildFace - do nothing
      continue;  //cbi breakpoint only: here is trouble.

  } // end iter ii, every EdgeDefinition moving owning ProtoVertices to containing ChildFaces

  // arrive here after every ProtoEdge and ProtoVertex has been moved to its containing ChildFace
  // next: See if ProtoFaces are the SplitFace need to be Split as well

  // locals
  double      dMidParam;
  SmVector3d  sBinormOld, sBinormNew, sBinormOther, sBinormPt;
  SmBoolean   bGotBinormals = FALSE;

  // Note: this loop can create new ProtoFaces, which get added to the m_sPFs list.
  //       We know that they get added to the end of the list.  We do not want to process
  //       those new PFs in this loop, so iterate on the current size of m_sPFs.
  ULONG lNumPFs = m_sPFs.GetSize();

  // for every ProtoFace
  for(ii=0;ii<lNumPFs;ii++)
    {
      SmProtoFace * pPF       = m_sPFs[ii];
      SmTopology  * pBrepTopo = pPF->GetSrcTopo( iWhichBrep) ;
      SmFace      * pFace     = SM_CAST_PTR( SmFace, pBrepTopo) ;

      // check state - pPF->SourceTopology should be a Face
      SM_ASSERT_MSG(pFace != NULL, _T("SmProtoManager::UpdateFaces WARNING - found a ProtoFace/Topology mismatch - needs debug")) ;
      if(pFace == NULL) { WARN( _T("Warning: SmProtoTopologyManager::UpdateFaces(): Type/Topology mismatch.\n")) ;
                          continue;
                        }

      // skip faces that are not part of the ParentFace that just got split - those don't need to be sorted
      if(pFace != pOldFace )
        { continue; }

      // Found a ProtoFace on (coincident with) the Face that was split.

      // Now, we have the Edge that split pNewFace off of pOldFace.
      // We might also have the corresponding Edge in the other Brep.
      // If we have that other Edge, and if that other Edge is an Edge
      // of the Face in the other Brep (the other member of this ProtoFace,
      // the coincident Face), then we can work with binormals, and avoid
      // having to find a UV value that is inside this Face.

      // locals
      pBrepTopo                  = pPF->GetSrcTopo( 3 - iWhichBrep) ;
      SmFace    * pOtherFace     = SM_CAST_PTR( SmFace, pBrepTopo) ;
      SmEdgeuse * pEU            = (pOtherFace != NULL && pOtherEdge != NULL) ? pOtherEdge->GetEdgeuseOfFace( pOtherFace) : NULL ; 
      SmBoolean   bDidReclassify = FALSE;

      // check state - pOtherFace should exist
      SM_ASSERT_MSG(pOtherFace != NULL, _T("SmProtoManager::UpdateFaces WARNING - found a ProtoFace without a matching coincident face in the OtherBrep - needs debug")) ;
      if(pOtherFace == NULL )
        { SE( SM_ERR) ; continue ; }

      // When pEU was found - classify pEU, iWhich: 1 = OtherBrep->SplitEdge->EU bounds OtherBrep->ChildFace1
      //                                            2 = OtherBrep->SplitEdge->EU bounds OtherBrep->ChildFace2
      //                                            0 = not classified to one ChildFace or the other
      if(pEU != NULL)
        {
          // Find the binormals pointing in both directions into the two
          // new pieces, and the binormal pointing into the old Face.
          // Its binormal must match either that of pNewFace 'xor' pOldFace.

          if(bGotBinormals == FALSE)
            {
              // ThisBrep->SplittingEdge Child1 EU
              SmEdgeuse *pThisEU = pSplittingEdge->GetEdgeuseOfFace( pOldFace) ;

              // when ThisBrep->SplittingEdge->EU exists
              if(pThisEU)
                {
                  // Evaluate ThisBrep->Split_Child1 EU->MidPoint Binormal
                  dMidParam = pSplittingEdge->GetInterval().GetMid();
                  pThisEU->EvaluateBinormal( dMidParam, FALSE, sBinormPt, sBinormOld) ;

                  // ThisBrep->Split_Child2 EU
                  pThisEU = pSplittingEdge->GetEdgeuseOfFace( pNewFace) ;
                  if(pThisEU)
                    {
                      // Evaluate ThisBrep->Split_Child2 EU->MidPoint Binormal
                      pThisEU->EvaluateBinormal( dMidParam, FALSE, sBinormPt, sBinormNew) ;
                      bGotBinormals = TRUE;

                    } // end ThisBrep->Split_Child2->EU existence check
                } // end ThisBrep->Split_Child1->EU existence check 
            } // end Have Binormals check

          // OtherBrep->SplitEdge->EU MidPt
          dMidParam = pOtherEdge->GetInterval().GetMid();
          pEU->EvaluateBinormal( dMidParam, FALSE, sBinormPt, sBinormOther) ;

          // test OtherBrep->SplitEdge->EU Binorm against ThisBrep->SplitEdge->EU to classify OtherBrep->SplitEdge->EU ChildFace containment
          double dDotOld = sBinormOld.Dot( sBinormOther) ;
          double dDotNew = sBinormNew.Dot( sBinormOther) ;
          iWhich = ( dDotOld > 0 && dDotNew < 0 ) ? 1 :      // OtherBrep->SplitEdge->EU bounds OtherBrep->ChildFace1
                   ( dDotOld < 0 && dDotNew > 0 ) ? 2 : 0 ;  // OtherBrep->SplitEdge->EU bounds OtherBrep->ChildFace2

          // when pEU bounds one ChildFace and not the other, then we're done.
          if      (iWhich == 1) { bDidReclassify = TRUE ; } // It's in the old face, no changes.
          else if (iWhich == 2) { // pPF->SetSrcTopo( iWhichBrep, SM_PC_FACE, pNewFace) ;  // We don't want to actually do this yet: see just below.
                                  bDidReclassify = TRUE;
                                }

      } // end pOtherEdge->pEU exists check 

      // arrive here when 
      //   pOtherFace = OtherBrep->SplitFace
      //   pEU        = OtherBrep->SplittingEdge->Edgeuse
      //   iWhich     = pEU classification it bounds either OtherBrep Child1 or Child2 or was not classified as:
      //                  iWhich: 1 = OtherBrep->SplitEdge->EU bounds OtherBrep->ChildFace1
      //                          2 = OtherBrep->SplitEdge->EU bounds OtherBrep->ChildFace2
      //                          0 = not classified to one ChildFace or the other 

      // PF bounding ProtoEdges
      SmTArray< SmProtoEdge * > sCoincPEs;
      pPF->GetCoincProtoEdges( sCoincPEs) ;

      // when OtherBrep->SplittingEdge->Edgeuse did not classify to one of OtherBrep->Child1 or Child2 faces
      if(FALSE == bDidReclassify)
        {
          // The binormal check didn't work.
          // This could be a second ProtoFace on the Brep Face that got split,
          // and on a different Face in the unsplit Face's Brep.
          //
          // Here we have to find a uv point inside the other Face, and see whether that
          // point is in after split Child1 (OldFace) or Child2 (NewFace).
          //
          //   Note: 1. a random point in either Face might not drop to the other Face:
          //            because partially coincident faces might only overlap by a small amount.
          //         2. This ProtoFace has a list of coincident ProtoEdges, which must overlap
          //            either the new Face or the old one, and not both.  
          // Drop the midpoints of the PF's coincident PE's to the old and new Faces.
          //   Note: 3. Can't drop ProtoVertices to the faces because they will classify to Edges, so that's no help.

          // locals
          SmPoint3d sMidPt ;
          ULONG lNumCoincPEs = sCoincPEs.GetSize() ;

          // for every PF->CoincPE
          for(jj=0;jj<lNumCoincPEs;jj++)
            {
              SmProtoEdge *pCoincPE = sCoincPEs[jj];

              // Get this PE's midpoint.
              pCoincPE->EvaluateMid( &sMidPt) ;

              // Drop that 3d point to the surface of the split face.
              SmSurface           * pSrf           = pOldFace->GetSurface();
              SmExtent2d            sSurfaceDomain = pSrf->GetNaturalUVDomain();
              double                dGap           = 0.0;
              SmSolverOperationType eOpType        = SM_SO_NORMALIZE; // Don't want off-boundary drops.
              SmPoint2d             sFaceUVParam ;
              SmBoolean             bSuccess ;
              SmBoolean             bIsMulti ;

              // drop PF->CoincPE MidPt3d to ThisBrep->ParentFace->Surface  (note: the Surface covers both Child1 and Child2 domains so mid point should drop)
              eStat = pSrf->DropPoint(sMidPt,         // in : target point to drop
                                      sSurfaceDomain, // in : target surface domain
                                      NULL,           // in : When given, uses only local solves
                                      bSuccess,       // out: TRUE=Point dropped successfully, FALSE=didn't
                                      sFaceUVParam,   // out: drop result UVPoint
                                      dGap,           // out: distance of found point to Pt to drop
                                      bIsMulti,       // out: TRUE = solution is multivalued, FALSE = Single valued
                                      eOpType) ;      // in : SM_SO_MINIMIZE = [default] allow nonNormal drops near boundaries
                                                      //      SM_SO_NORMALIZE= exclude nonNormal drops near boundaries
                                                      //      SM_SO_INTERSECT= point must be on surface, to Tol
              if(eStat != SM_SUCCESS || bSuccess == FALSE)
                {
                  WARN( _T("SmProtoTopologyManager::UdateFaces(): Dropping pF->CoincPE->MidPt to ThisBrep->ParentFace->Surface with DropPoint() failed - this is a bug")) ;
                  continue;  // Breakpoint: trouble.
                }

              // classify sFaceUVParam against Child1 (pOldFace) and Child2 (pNewFace) Faces.
              SmZoneTol3d sFaceZoneTol3d = SmTol::GetZoneTol3d_ForXSectResult( pOldFace, pNewFace) ;
              iWhich = sm_ClassifyParamOnFaces // rtn: Pt Classify value: 1 = On pFace1, 2 = On pFace2, 0 = not classifable (could be on both Faces)
                         (sFaceUVParam,        // in : Target UVParam to classify
                          sFaceZoneTol3d,      // in : obj ZoneTol3d assoc with UVParam, usually not this face
                                               //      if none, use:SmTol::GetZoneTol3d(BREP_CONTEXT_OR_NULL)
                          pOldFace,            // in : 1st TgtFace to check for pt containment
                          sOldDomain,          // in : TgtFace1 UV Domain
                          pNewFace,            // in : 2nd TgtFace to check for pt containment
                          sNewDomain) ;        // in : TgtFace2 UV Domain

              // If it's in one and not in the other, then we're done.
              if     (iWhich == 1) { bDidReclassify = TRUE ; } // It's in the old face, no changes.
              else if(iWhich == 2) { // pPF->SetSrcTopo( iWhichBrep, SM_PC_FACE, pNewFace) ;  // But we don't want to actually do this yet:
                                     bDidReclassify = TRUE;
                                   }

              // when we found a PF->CoinPE that classifies to either Child1 or Child2 face (not their edges) were done with this loop
              if(bDidReclassify )
                { break; }

            } // end iter jj, every pF->CoincPE looking for one whose MidPt3d drops to either ChildFace1 or ChildFace2

        }  // end OtherBrep->SplittingEdge->Edgeuse did not classify to one of OtherBrep->Child1 or Child2 face check

      // check state - expect the Edgeuse-Binormal or the PF->CoinPE->MidPt3d Drop to ParentSurface classification to have worked
      if(bDidReclassify == FALSE)
        { continue ; }  // Brepakpoint: how did this happen?

      // < Old algorithm removed after v. 8370. >

      // arrive here when: 
      //   iWhich: 1 = pOldFace is coincident with pOtherFace  (locally)
      //           2 = pNewFace is coincident with pOtherFace  (locally)
      // next:
      //   sort every pPF->ProtoEdge into those that stay on Child1 (pOldFace)
      //   and those that need to move to Child2 (pNewFace) by moving
      //   the PEs on Child2 from the pPF->CoincPEs list to a ToMove list.
      //    note: some of the ProtoEdges in this ProtoFace are now on pOldFace
      //          and some are on pNewFace, then we have to make a new ProtoFace
      //          for the ProtoEdges on pNewFace.

      // Set pThisFace and pThatFace according to iWhich.
      //   note: 1. pThisFace will stay with this ProtoFace, pPF.
      //            It will keep all ProtoEdges that are connected to the Face portion
      //           (pOldFace or pNewFace) that is coincident with pOtherFace.
      //         2. If this ProtoFace has any CoincPEs that are not connected to pThisFace,
      //            create a new ProtoFace that contains them.

      SmFace *pThisFace = ( iWhich==1 ) ? pOldFace : pNewFace;  // pThisFace will be our Face.
      SmFace *pThatFace = ( iWhich==1 ) ? pNewFace : pOldFace;

      // Now we can set pPF->SrcTopo to pThisFace
      pPF->SetSrcTopo( iWhichBrep, SM_PC_FACE, pThisFace) ;

      // next: FOR each coincident PE:
      //         IF its Brep topo is in a Face,
      //           then its Brep topo has already been processed, above.
      //         IF its Brep topo is on an Edge,
      //           then if that Edge is connected to pThisFace, do nothing
      //           else move it.
      //       END FOR

      // locals
      SmTArray< SmProtoEdge* > sPEsToMove;
      ULONG lNumCoincPEs = sCoincPEs.GetSize();

      // for every pPF->CoincPE
      for(jj=0;jj<lNumCoincPEs;jj++)
        {
          SmProtoEdge                  * pPE         = sCoincPEs[jj] ;
          SmBoolean                      bInThisFace = TRUE ;
          SmTArray<SmEdgeDefinition *> & raEdgeDefs  = pPE->GetEdgeDefinitions();
          ULONG                          lNumEDs     = raEdgeDefs.GetSize();

          // Just do this for each EdgeDef; break when we find one that works.
          for(kk=0;kk<lNumEDs;kk++)
            {
              SmEdgeDefinition        * pED           = raEdgeDefs[kk];
              if(pED == NULL) { continue ; }
              SmPointClassificationType eBrepTopoType = pED->GetSrcTopo_TYPE( iWhichBrep) ; // this is the Src XSecting ThisBrep's CurveClassification Interval MidPointClassification

              if(eBrepTopoType == SM_PC_FACE)
                {
                  // If it's in a Face, then its BrepTopo has already been (re)set
                  // to the Face it's actually in, in the above loop on ProtoEdges.
                  SmTopology * pBrepTopology = pED->GetSrcTopo( iWhichBrep) ;
                  SmFace     * pF            = SM_CAST_PTR( SmFace, pBrepTopology) ;

                  if(pF == NULL) { WARN( _T("Warning: Topology mismatch in ProtoEdge")) ; }
                  else           { if(pF != pThisFace)
                                     { bInThisFace = FALSE; }
                                 }
                  break ;
                } // end eBrepTopoType == SM_PC_FACE branch

              else if(eBrepTopoType == SM_PC_EDGE)
                {
                  // If PE->EdgeDefinition is not on a Face, then it should be on an Edge.

                  SmTopology * pBrepTopology = pED->GetSrcTopo( iWhichBrep) ;
                  SmEdge     * pE            = SM_CAST_PTR( SmEdge, pBrepTopology) ;
                  if(pE == NULL) { WARN( _T("Warning: Topology mismatch in ProtoEdge")) ; } // error - pE->EdgeEdfeinition->Src should be a face or an edge
                  else           { if(pE->IsConnectedToFace( pThisFace ) == FALSE)          // when pE is not connected to this face
                                     { bInThisFace = FALSE; }                               // remember the edge has to be moved
                                 }
                  break;  //cbi How do I know it's the right ED?
                } // end eBrepTopoType == SM_PC_EDGE branch
              else
                { WARN( _T("Warning: Topology mismatch in ProtoEdge")) ; } // Breakpoint here.

            } // end iter kk, every ED of this PE

          // when this PE classifies to the other ChildFace
          if(bInThisFace == FALSE)
            {
              // move PEs in the pF->CoincPEs list to the ToMove list
              sPEsToMove.Add( pPE) ;
              sCoincPEs[ jj ] = NULL; // (Just Null for now; compress after loop.)
            }
        } // end iter jj, every pPF->CoincPEs finding which ones have to move to the other Child's CoincPEs list

      // compress the pPF->sCoincPEs list to remove the NULL entries left behind when a PE was moved to the ToMove list
      sCoincPEs.CompressZeros();

      // chck state: Sorted PE count = original PE count
      SM_ASSERT( sCoincPEs.GetSize() + sPEsToMove.GetSize() == lNumCoincPEs) ;

      // arrive here when: 
      //   iWhich: 1 = pOldFace is coincident with pOtherFace  (locally)
      //           2 = pNewFace is coincident with pOtherFace  (locally)
      //   sPEsToMove = pPF->ProtoEdges that have to move from pOldFace to pNewFace
      //   pOldFace and pNewFace have been sorted into pThisFace and pThatFace.
      //       pThisFace = Face assigned to pOldFace
      //       pThatFace = Face to be used for the new ProtoFace that has to be built when sPEsToMove is not empty.

      // locals 
      SmProtoFace *pNewPF = NULL;

      // when there are pPF->CoincPEs to move
      if(sPEsToMove.GetSize() > 0)
        {
          // At least one ProtoEdge has been moved to pNewFace.
          // If they have all been moved, then just move this ProtoFace
          // over to pNewFace, otherwise, create a new ProtoFace on pNewFace,
          // with the ProtoEdges in pPEsToMove.

          // PEs are in both Child1 and Child2 split faces branch 
          if(sCoincPEs.GetSize() > 0)
            {
              // Make a new ProtoFace
              if(iWhichBrep == 1) { this->CreateProtoFace( pThatFace, pOtherFace, sPEsToMove, pNewPF) ; }
              else                { this->CreateProtoFace( pOtherFace, pThatFace, sPEsToMove, pNewPF) ; }
            }
          else // all PEs have moved to child2 face branch
            {
              // All PEs were moved to pNewPF.  Just update this ProtoFace.
              pPF->SetSrcTopo( iWhichBrep, SM_PC_FACE, pNewFace) ;
            }
        } // end there are PEs to move check

#ifdef SM_DEBUG_CODE
static int iDebugLevel=0;
      if(iDebugLevel>0 )
        {
          smgfx_Erase();
          smgfx_SetLook( 1,2, 0,0,1) ; m_pBrep1->Draw(TRUE); sm_GraphicsLoop();
          smgfx_SetLook( 1,2, 0,1,0) ; m_pBrep2->Draw(TRUE); sm_GraphicsLoop();
          smgfx_SetLook( 2,4, 0,1,1) ; pPF->Draw(); sm_GraphicsLoop();
          if(pNewPF != NULL )
            { smgfx_SetLook( 2,4, 1,0,1) ; pNewPF->Draw(); sm_GraphicsLoop(); }
          sm_GraphicsLoop();
        }
#endif // SM_DEBUG_CODE

    } // end iter ii, for each ProtoFace

  // all done
  return SM_SUCCESS;

} // end SmProtoTopologyManager::UpdateFaces

/*******************************************************************//**
PURPOSE: After the Breps have been imprinted with the intersections,
   Glue() any coincident vertices left from that operation.

USAGE NOTES---- 
***********************************************************************/
SmStatus SmProtoTopologyManager::GlueCoincidentBrepVertices()
{
  // Coincident vertices would be multiple VertexDefinitions in the same ProtoVertex.
  SmTArray< SmProtoVertex* > sPVs = this->GetProtoVertices();
  ULONG ii, jj, kk, ll, mm, nn, lNumPVs = sPVs.GetSize();

  // We need to check whether these are in any other ProtoVertices, ProtoEdges, or ProtoFaces:
  SmTArray<SmVertex*> sDeletedVertices1, sDeletedVertices2;
  SmTArray<SmEdge  *> sDeletedEdges1,    sDeletedEdges2;

  SmStatus eStat, eRet = SM_SUCCESS;

  for(ii=0;ii<lNumPVs;ii++)
  {
      SmProtoVertex *pPV = sPVs[ii];   if(pPV == NULL ) { continue; }

      SmTArray< SmVertexDefinition* > sVDs = pPV->GetVertexDefinitions();
      ULONG lNumVDs = sVDs.GetSize();
      if(lNumVDs < 2 ) { continue; }

      // Glue the first VD to all others.
      SmVertexDefinition *pVD1 = sVDs[0];
      SmPoint3d sPt2, sPt1 = pVD1->GetPosition();

      // Keep track deleted Vertices and corresponding VD
      sDeletedVertices1.ReSet();
      sDeletedVertices1.SetSize( lNumVDs) ;
      sDeletedVertices2.ReSet();
      sDeletedVertices2.SetSize( lNumVDs) ;

#ifdef SM_DEBUG_CODE
      SmBoolean bDebugMe = FALSE ;
      if(bDebugMe )
      {
          TCHAR sBuff[SM_MAXSIZE];
          smos_sprintf( sBuff, _T( "ProtoVertex #%lu, VertexDefinition #0 \n" ), ii) ;
          smos_WriteBuffer( sBuff) ;
          SmVertex *pDBrep1Vtx1 = pVD1->GetFinalVertex( 1) ;  
          SmVertex *pDBrep2Vtx1 = pVD1->GetFinalVertex( 2) ;  

          SM_DUMP( pDBrep1Vtx1) ;
          SM_DUMP( pDBrep2Vtx1) ;
      }
#endif

      for(jj=1;jj<lNumVDs;jj++)
      {
          SmVertexDefinition *pVD2 = sVDs[jj];

#ifdef SM_DEBUG_CODE
          if(bDebugMe )
          {
              TCHAR sBuff[SM_MAXSIZE];
              smos_sprintf( sBuff, _T( "ProtoVertex #%lu, VertexDefinition #%lu \n" ), ii, jj) ;
              smos_WriteBuffer( sBuff) ;
              SmVertex *pDBrep1Vtx2 = pVD2->GetFinalVertex( 1) ;
              SmVertex *pDBrep2Vtx2 = pVD2->GetFinalVertex( 2) ;

              SM_DUMP( pDBrep1Vtx2) ;
              SM_DUMP( pDBrep2Vtx2) ;
          }
#endif // SM_DEBUG_CODE

          // Use FinalVertex Tolerances (below) [B673]
          //// Do not glue if they're not within tolerance of each other.
          //sPt2 = pVD2->GetPosition();
          //double dDist = sPt1.DistanceBetween( sPt2) ;
          //double dLimit = m_dTol + pVD1->GetGap() + pVD2->GetGap();
          //if(dDist >= dLimit )
          //  { continue; }

          // If there is no Edge connecting the two Vertices, then Glue them.
          // If there is a connecting Edge, then if that Edge is too short, Squeeze it.
          SmVertex *pBrep1Vtx1 = pVD1->GetFinalVertex( 1) ;
            if(pBrep1Vtx1== NULL  ||  sDeletedVertices1.IsIn( pBrep1Vtx1 ) )
            { continue; }
          SmVertex *pBrep1Vtx2 = pVD2->GetFinalVertex( 1) ;
            if(pBrep1Vtx2== NULL  ||  sDeletedVertices1.IsIn( pBrep1Vtx2 ) )
            { continue; }
          SmVertex *pBrep2Vtx1 = pVD1->GetFinalVertex( 2) ;
            if(pBrep2Vtx1== NULL  ||  sDeletedVertices2.IsIn( pBrep2Vtx1 ) )
            { continue; }
          SmVertex *pBrep2Vtx2 = pVD2->GetFinalVertex( 2) ;
            if(pBrep2Vtx2== NULL  ||  sDeletedVertices2.IsIn( pBrep2Vtx2 ) )
            { continue; }

          double dDist1 = SM_BIG_DOUBLE, dDist2 = SM_BIG_DOUBLE;
          SmXSectTol3d sXSectTol1, sXSectTol2;

          SmBoolean bDeleted1 = FALSE, bDeleted2 = FALSE;
          ULONG lIdx;
          if(sDeletedVertices1.FindElement(pBrep1Vtx2, lIdx) )
          { 
              pVD2->SetFinalVertex( 1, NULL) ;
              bDeleted1 = TRUE;
          }
          else
          {
              dDist1 = pBrep1Vtx1->GetPoint().DistanceBetween( pBrep1Vtx2->GetPoint()) ;
              sXSectTol1 = SmTol::GetXSectTol3d( pBrep1Vtx1, pBrep1Vtx2) ;
          }

          if(sDeletedVertices2.FindElement( pBrep2Vtx2, lIdx) )
          { 
              pVD2->SetFinalVertex( 2, NULL) ;
              bDeleted2 = TRUE; 
          }
          else
          {
              dDist2 = pBrep2Vtx1->GetPoint().DistanceBetween( pBrep2Vtx2->GetPoint()) ;
              sXSectTol2 = SmTol::GetXSectTol3d( pBrep2Vtx1, pBrep2Vtx2) ;
          }

          SmTArray< SmEdge* > sBrep1Edges, sBrep2Edges;
          pBrep1Vtx1->GetCommonEdges( pBrep1Vtx2, sBrep1Edges) ;
          pBrep2Vtx1->GetCommonEdges( pBrep2Vtx2, sBrep2Edges) ;

          ULONG lNumEdges1 = sBrep1Edges.GetSize();
          ULONG lNumEdges2 = sBrep2Edges.GetSize();

          // Note, these can be different: Sweep a Loop Vertex (embedded directly in a Face)
          // by 360 degrees, we get a wire Edge that is a full circle, but the start and end
          // vertices are different: coincident.  Those two map to the same Vertex in the
          // other Brep.  Perhaps that should be changed in the Sweep code, but we handle it here.
          //if(lNumEdges1 != lNumEdges2 )
          //{
          //    ERR_MSG( _T("Error: Incompatible Brep topology found after imprinting.\n")) ;
          //    continue;
          //}
          // But they can be different for other reasons as well. [Reg_091215_S..._Closed]

          if(dDist1 > sXSectTol1 )
          {}
          else if(lNumEdges1 == 0 )
          {
              if(pBrep1Vtx1 != pBrep1Vtx2 )
              {
                  GetWhichBrep(1)->GlueVertices( pBrep1Vtx1, pBrep1Vtx2) ;
                  sDeletedVertices1[jj] = pBrep1Vtx2;
                  pVD2->SetFinalVertex( 1, NULL) ;
                  bDeleted1 = TRUE;
              }
          }
          else
          {
              for(kk=0;kk<lNumEdges1;kk++)
              {
                  SmEdge *pE1 = sBrep1Edges[kk];
                  double dLen1 = pE1->GetCurve()->ApproximateLength( pE1->GetInterval(), 5) ;
                  if(dLen1 < sXSectTol1 )
                  {
                      // If any to-be-deleted topology is mated, remove the relationship when done.
                      // (They shouldn't be mated at this point though, but just in case.)
                      SmEdge   *pEdgeMate     = SM_CAST_PTR( SmEdge,   m_pTI->GetOtherMate( pE1 ));
                      SmVertex *pOtherVtxMate = SM_CAST_PTR( SmVertex, m_pTI->GetOtherMate( pBrep1Vtx2 ));

                      SmVertex *pOtherVtx = pE1->GetOtherVertex( pBrep1Vtx1) ;
                      SmCurve  *pCurve    = pE1->GetCurve();
                      if(pOtherVtx == pBrep1Vtx1 )
                      {
                          eStat = this->GetWhichBrep(1)->DeleteEdge( pE1) ; // Can't call SqueezeEdge() in this case.
                      }
                      else
                      {
                          eStat = this->GetWhichBrep(1)->SqueezeEdge( pE1, pBrep1Vtx1) ;  // This deletes pBrep1Vtx2.
                          sDeletedVertices1[jj] = pBrep1Vtx2;
                      }
                      sDeletedEdges1.Add( pE1) ;

                      if(eStat == SM_SUCCESS )
                      {
                          bDeleted1 = TRUE;

                          if(pEdgeMate != NULL )
                            { m_pTI->RemoveRelationship( pE1, pEdgeMate) ; }
                          if(pOtherVtxMate != NULL )
                            { m_pTI->RemoveRelationship( pBrep2Vtx2, pOtherVtxMate) ; }


                          //cbi Update this ProtoTopoMgr: need to find the EdgeDefs for pE1, pE2.
                          //cbi Should probably adapt GetCommonEdges() to ProtoTopoMgr, and work from that.

                          SmEdgeDefinition *pEDToDelete = this->FindEdgeDefinitionFromEdge( pE1, 1) ;
                          if(pEDToDelete != NULL )
                          {
                              // Squeezing or Deleting pE1 deleted pCurve, which pEDToDelete may point to [B673]
                              if(pCurve == pEDToDelete->GetXSectCurve3d() )
                                { pEDToDelete->SetXSectCurve3d( NULL) ; }

                              // Clear out the UV Curves to prevent a crash. May leaks curves, though...
                              pEDToDelete->SetXSectCurveUV( 1, NULL) ; // May have been deleted by DeleteEdge or SqueezeEdge
                              pEDToDelete->SetXSectCurveUV( 2, NULL) ; // Could be used by Brep2

                              // Don't remove the PE if empty.  Else would have to keep track,
                              // and remove them from the PFs.  And, empty PEs don't hurt anything.
                              this->RemoveEdgeDefinition( pEDToDelete, FALSE) ;
                              pEDToDelete->Destruct();
                              delete pEDToDelete; pEDToDelete = NULL;
                          }
                      }
                      else
                        { eRet = eStat; }

                  } // end if Edge length below tolerance.
                  else
                  {
                      // Here we have coincident Vertices connected by a long Edge.
                      // We should Glue these.
                      if(pBrep1Vtx1 != pBrep1Vtx2 )
                      {
                          GetWhichBrep(1)->GlueVertices( pBrep1Vtx1, pBrep1Vtx2) ;
                          sDeletedVertices1[jj] = pBrep1Vtx2;
                          pVD2->SetFinalVertex( 1, NULL) ;
                          bDeleted1 = TRUE;
                      }
                  }

              } // end loop kk on common Edges

          } // end else (at least one common Edge)

          // Repeat for Brep 2.

          if(dDist2 > sXSectTol2 )
          {}
          else if(lNumEdges2 == 0 )
          {
              if(pBrep2Vtx1 != pBrep2Vtx2 )
              {
                  GetWhichBrep(2)->GlueVertices( pBrep2Vtx1, pBrep2Vtx2) ;
                  sDeletedVertices2[jj] = pBrep2Vtx2;
                  pVD2->SetFinalVertex( 2, NULL) ;
                  bDeleted2 = TRUE;
              }
          }
          else
          {
              for(kk=0;kk<lNumEdges2;kk++)
              {
                  SmEdge *pE2 = sBrep2Edges[kk];
                  double dLen2 = pE2->GetCurve()->ApproximateLength( pE2->GetInterval(), 5) ;
                  if(dLen2 < sXSectTol2 )
                  {
                      // If any to-be-deleted topology is mated, remove the relationship when done.
                      // (Note, they shouldn't be mated at this point though, but just in case.)
                      SmEdge   *pEdgeMate     = SM_CAST_PTR( SmEdge,   m_pTI->GetOtherMate( pE2 ));
                      SmVertex *pOtherVtxMate = SM_CAST_PTR( SmVertex, m_pTI->GetOtherMate( pBrep2Vtx2 ));

                      SmVertex *pOtherVtx = pE2->GetOtherVertex( pBrep2Vtx1) ;
                      if(pOtherVtx == pBrep2Vtx1 )
                      {
                          eStat = this->GetWhichBrep(2)->DeleteEdge( pE2) ; // Can't call SqueezeEdge() in this case.
                      }
                      else
                      {
                          eStat = this->GetWhichBrep(2)->SqueezeEdge( pE2, pBrep2Vtx1) ;  // This deletes pBrep2Vtx2.
                          sDeletedVertices2[jj] = pBrep2Vtx2;
                      }
                      sDeletedEdges2.Add( pE2) ;

                      if(eStat == SM_SUCCESS )
                      {
                          bDeleted2 = TRUE;

                          if(pEdgeMate != NULL )
                            { m_pTI->RemoveRelationship( pE2, pEdgeMate) ; }
                          if(pOtherVtxMate != NULL )
                            { m_pTI->RemoveRelationship( pBrep2Vtx2, pOtherVtxMate) ; }

                          // Note, the EdgeDef for the Brep1 Edge would presumably be the same
                          // as that for the Brep2 Edge, so deleting the first one will probably
                          // take care of the second one's EdgeDef.
                          // We should do it this way though, and not in pairs, because if either
                          // one of an EdgeDef's SmEdges is deleted, the EdgeDef should go too.

                          SmEdgeDefinition *pEDToDelete = this->FindEdgeDefinitionFromEdge( pE2, 2) ;
                          if(pEDToDelete != NULL )
                          {
                              // Don't delete IntCurve3d, which is a Brep1 object. If we reach here, it's being used as an Edge->Curve [B673]
                              pEDToDelete->SetXSectCurve3d( NULL) ;

                              // Clear out the UV Curves to prevent a crash. May leaks curves, though...
                              pEDToDelete->SetXSectCurveUV( 1, NULL) ; // In use in Brep1
                              pEDToDelete->SetXSectCurveUV( 2, NULL) ; // Possibly deleted by DeleteEdge or SqueezeEdge

                              // Don't remove the PE if empty.  Else would have to keep track,
                              // and remove them from the PFs.  And, empty PEs don't hurt anything.
                              this->RemoveEdgeDefinition( pEDToDelete, FALSE) ;
                              pEDToDelete->Destruct();
                              delete pEDToDelete; pEDToDelete = NULL;
                          }
                      }
                      else
                        { eRet = eStat; }

                  } // end if Edge length below tolerance.
                  else
                  {
                      // Here we have coincident Vertices connected by a long Edge.
                      // We should Glue these.
                      if(pBrep2Vtx1 != pBrep2Vtx2 )
                      {
                          GetWhichBrep(2)->GlueVertices( pBrep2Vtx1, pBrep2Vtx2) ;
                          sDeletedVertices2[jj] = pBrep2Vtx2;
                          pVD2->SetFinalVertex( 2, NULL) ;
                          bDeleted2 = TRUE;
                      }
                  }

              } // end loop kk on common Edges

          } // end else (at least one common Edge)


          if(bDeleted1 || bDeleted2 )
          {
              pPV->RemoveVertexDefinition( pVD2) ;
              sVDs[jj] = NULL;
              delete pVD2; pVD2 = NULL;
          }

      } // end loop jj on all VD's in this PV


      // Check for stale Brep topology: we might have deleted some Final SmVerts
      // that are pointed to by other PVs.  Check that.

      sDeletedVertices1.CompressZeros();
      sDeletedVertices2.CompressZeros();

      if(sDeletedVertices1.GetSize() > 0  ||  sDeletedVertices2.GetSize() > 0 )
      {
          for(ll=0;ll<lNumPVs;ll++)
          {
              SmProtoVertex *pThisPV = sPVs[ll];
              if(pThisPV == NULL ) { continue; }

              SmTArray< SmVertexDefinition* > & rVtxDefs = pThisPV->GetVertexDefinitions();

              for(mm=0;mm<rVtxDefs.GetSize();mm++)
              {
                  SmVertexDefinition *pThisVD = rVtxDefs[mm];

                  if(sDeletedVertices1.IsIn( pThisVD->GetFinalVertex( 1 ) )
                    || sDeletedVertices2.IsIn( pThisVD->GetFinalVertex( 2 ) ) )
                  {
                      // No, this shrinks the array that we're working on (by reference):
                      //  pThisPV->RemoveVertexDefinition( pThisVD) ;
                      // Just set to Null and compress zeros when done.
                      rVtxDefs[mm] = NULL;
                      delete pThisVD; pThisVD = NULL;
                  }
              }
              rVtxDefs.CompressZeros();
          }
      } // end if there were deleted Vertices.

      // Also have to check deleted SmEdges.
      if(sDeletedEdges1.GetSize() > 0  ||  sDeletedEdges2.GetSize() > 0 )
      {
          SmTArray< SmProtoEdge* > & rPEs = this->GetProtoEdges();
          ULONG lNumPEs = rPEs.GetSize();
          for(ll=0;ll<lNumPEs;ll++)
          {
              SmProtoEdge *pThisPE = rPEs[ll];
              if(pThisPE == NULL ) { continue; }

              SmTArray< SmEdgeDefinition* > & rEdgeDefs = pThisPE->GetEdgeDefinitions();

              for(mm=0;mm<rEdgeDefs.GetSize();mm++)
              {
                  SmEdgeDefinition *pThisED = rEdgeDefs[mm];

                  if(sDeletedEdges1.IsIn( pThisED->GetFinalEdge( 1 ) )
                    || sDeletedEdges2.IsIn( pThisED->GetFinalEdge( 2 ) ) )
                  {
                      pThisPE->RemoveEdgeDefinition( pThisED) ;
                      pThisED->Destruct();
                      delete pThisED; pThisED = NULL;
                  }
              }
          }

          // ... and ProtoFaces have EdgeDefinitions too.
          SmTArray< SmProtoFace* > & rPFs = this->GetProtoFaces();
          ULONG lNumPFs = rPFs.GetSize();
          for(ll=0;ll<lNumPFs;ll++)
          {
              SmProtoFace *pThisPF = rPFs[ll];
              if(pThisPF == NULL ) { continue; }

              SmTArray< SmProtoEdge* > * pPEs = pThisPF->GetCoincProtoEdges();

              for(mm=0;mm<pPEs->GetSize();mm++)
              {
                  SmProtoEdge *pThisPE = (*pPEs)[mm];

                  SmTArray< SmEdgeDefinition* > & rEdgeDefs = pThisPE->GetEdgeDefinitions();

                  for(nn=0;nn<rEdgeDefs.GetSize();nn++)
                  {
                      SmEdgeDefinition *pThisED = rEdgeDefs[nn];

                      if(sDeletedEdges1.IsIn( pThisED->GetFinalEdge( 1 ) )
                        || sDeletedEdges2.IsIn( pThisED->GetFinalEdge( 2 ) ) )
                      {
                          this->RemoveEdgeDefinition( pThisED, FALSE) ;

                          // This EdgeDef would have to be in one of the ProtoEdges,
                          // and so would have been destroyed in the previous loop.
                          // pThisED->Destruct();
                          // delete pThisED; pThisED = NULL;
                      }
                  }
              } // end for every coinident ProtoEdge in this ProtoFace.
          } // end for every ProtoFace.

      } // end if there were deleted Edges.

  } // end loop ii on all PV's

  return eRet;

} // end SmProtoTopologyManager::GlueCoincidentBrepVertices()

/*******************************************************************//**
PURPOSE: See whether two VertexDefinitions can be combined into one.

USAGE NOTES----  1. Uses the VertexDefinition->Tolerances as ZoneTol3d values
***********************************************************************/
SmStatus SmProtoTopologyManager::CanCombineVertexDefinitions
 (SmVertexDefinition * pVD1,       // in : TgtVertexDef 1
  SmVertexDefinition * pVD2,       // in : TgtVertexDef 2
  ULONG              &rlCanDelete) // out: 0: neither - both pVD1 and pVD2 are needed 
                                   //      1: pVD1 is redundant and can be deleted 
                                   //      2: pVD2 is redundant and can be deleted
{
  // Note that it's possible to end up in the case where bForceCombine = TRUE,
  // but topology aren't connected so we can't combine. We attempt to resolve
  // this during Imprint, by gluing related vertices
  rlCanDelete = 0;

  SmTopology * pVD1Topo1 = pVD1->GetSrcTopo(1); NER( pVD1Topo1) ;
  SmTopology * pVD1Topo2 = pVD1->GetSrcTopo(2); NER( pVD1Topo2) ;
  SmTopology * pVD2Topo1 = pVD2->GetSrcTopo(1); NER( pVD2Topo1) ;
  SmTopology * pVD2Topo2 = pVD2->GetSrcTopo(2); NER( pVD2Topo2) ;

  // If both VDs are on the same Vertex in either Brep, then definitely combine them.  [Reg_091215_s..._closed]
  SmBoolean bForceCombine = FALSE;
  SmVertex *pVert1 = SM_CAST_PTR( SmVertex, pVD1Topo1) ;
  SmVertex *pVert2 = SM_CAST_PTR( SmVertex, pVD2Topo1) ;
  if(pVert1 != NULL && pVert1 == pVert2 )
  {
      // Definitely get rid of one of them.
      bForceCombine = TRUE;
  }
  pVert1 = SM_CAST_PTR( SmVertex, pVD1Topo2) ;
  pVert2 = SM_CAST_PTR( SmVertex, pVD2Topo2) ;
  if(pVert1 != NULL && pVert1 == pVert2 )
  {
      // Definitely get rid of one of them.
      bForceCombine = TRUE;
  }

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
  if(bDebugMe ) 
   {
      smgfx_Erase();
      if(FALSE ) {
          smgfx_SetLook( 1,2, 0,0,1) ; m_pBrep1->Draw(TRUE); sm_GraphicsLoop();
          smgfx_SetLook( 1,2, 0,1,0) ; m_pBrep2->Draw(TRUE); sm_GraphicsLoop();
          sm_GraphicsLoop();
      }
      sm_DrawTopology( pVD1Topo1, 3,5, 0,0,1) ; sm_GraphicsLoop();
      sm_DrawTopology( pVD1Topo2, 4,6, 0,1,1) ; sm_GraphicsLoop();
      sm_DrawTopology( pVD2Topo1, 3,5, 1,0,1) ; sm_GraphicsLoop();
      sm_DrawTopology( pVD2Topo2, 4,6, 0,1,0) ; sm_GraphicsLoop();
      sm_GraphicsLoop();

      smgfx_SetLook( 4,7, 1,0,0) ; pVD1->Draw(); sm_GraphicsLoop();
      smgfx_SetLook( 4,8, 1,1,0) ; pVD2->Draw(); sm_GraphicsLoop();
      sm_GraphicsLoop();
  }
#endif // SM_DEBUG_CODE

  // Now, if they're sitting on the same topology in both Breps, combine.
  if(pVD1Topo1 == pVD2Topo1  &&  pVD1Topo2 == pVD2Topo2 )
  {
      bForceCombine = TRUE;
  }

  // If they're on different topology, we have to keep them both.
  // Ex: unstitched Edges pierce a Face, and they'll be glued later. [Offset iter 2]
  //  But: Just comparing output, the original has all coincident vertices combined.
  //  Not coincident Edges though.
  //  In this case, go ahead and create both SmVertices, and see about Gluing them later.

  //  - Vtx1  - Vtx2
  //  - Face1 - Vtx2  (same Vtx)
  //  - Edge1 - Edge2
  // Vtx1, Face1, and Edge1 are all topologically connected, as are Vtx2 and Edge2.
  // We want to keep only the first one, on both Verts.
  // Comparing 1st and 3rd, neither topo is the same, so have to check both connectedness.

  // If either pair is not connected in their Brep, can't combine.
  if(! sm_AreConnected( pVD1Topo1, pVD2Topo1 ) )
    { return SM_SUCCESS; }
  if(! sm_AreConnected( pVD1Topo2, pVD2Topo2 ) )
    { return SM_SUCCESS; }

  // Check distance: if they're too far apart, we can't combine them.
  // Use the same criteria as in CanCombineVertexDefinitions().
  // We were generous with tols while collecting (CaptureDistance), so we should test here.
  // Unfortunately, if we use the tightest logical tolerance, this causes problems.
  // We have to increase it somewhat.  4.0 is not quite enough.
  // We should look closely into this.
  // [Reg_091215_s....; Reg_091215_s...._closed]
  static constexpr double sdTolFactor = 5.0;
  SmPoint3d sPos1( pVD1->GetPosition()) ;
  SmPoint3d sPos2( pVD2->GetPosition()) ;
  double dDist = sPos1.DistanceBetween( sPos2) ;
  double dTol  = pVD1->GetTolerance() + pVD2->GetTolerance(); // tolerance used as a ZoneTol3d
  dTol *= sdTolFactor; // (ouch)
  if(dDist > dTol )
    { return SM_SUCCESS; }

  // They're both connected.  Since they're within tolerance, get rid of the ones
  // on the higher-dimensional Brep objects.

  int lDim11 = sm_TopoDimension( pVD1Topo1) ;
  int lDim12 = sm_TopoDimension( pVD1Topo2) ;
  int lDim21 = sm_TopoDimension( pVD2Topo1) ;
  int lDim22 = sm_TopoDimension( pVD2Topo2) ;

  // If dimensionality is different in Topo 1, check whether Topo 2 also allows a deletion.
  if(lDim11 < lDim21 )
  {
      if(lDim12 <= lDim22 )
        { rlCanDelete = 2; }
  }
  if(lDim21 < lDim11 )
  {
      if(lDim22 <= lDim12 )
        { rlCanDelete = 1; }
  }

  // If dimensionality is different in Topo 2, check whether Topo 1 also allows a deletion.
  if(lDim12 < lDim22 )
  {
      if(lDim11 <= lDim21 )
        { rlCanDelete = 2; }
  }
  if(lDim22 < lDim12 )
  {
      if(lDim21 <= lDim11 )
        { rlCanDelete = 1; }
  }

  // More checks, if we haven't decided yet.
  if(bForceCombine && rlCanDelete == 0 )
  {
      // Keep the one with the tighter gap.
      rlCanDelete = ( pVD1->GetGap() > pVD2->GetGap() ) ? 1 : 2;
      return SM_SUCCESS;
  }

  return SM_SUCCESS;

} // end SmProtoTopologyManager::CanCombineVertexDefinitions

/*******************************************************************//**
PURPOSE: Helper function for CanCombineEdgeDefinitions below

USAGE NOTES---- rED1E and rED2E should be edges in the same Brep
***********************************************************************/
SmStatus sm_FindEDOnDegenFace
 (SmEdge * pED1E,       // in : TgtEdge 1
  SmEdge * pED2E,       // in : TgtEdge 2
  ULONG  & rlCanDelete) // out: 0: neither 
                        //      1: pED1E 
                        //      2: pED2E 
{
  // Check inputs
  SmBrep * pBrep1 = pED1E->GetBrep();
  SmBrep * pBrep2 = pED2E->GetBrep();
  AERN(pBrep1 == pBrep2, SM_ERR_ASSERT_FAILURE) ;

  // init output
  rlCanDelete = 0 ; 
 
  // Locals
  ULONG ii ;
  SmTArray<SmFace*> sED1Fs ;
  SmTArray<SmFace*> sED2Fs ;
  SmTArray<SmFace*> sCommonFs ;
 
  // Find shared faces
  pED1E->GetFaces(sED1Fs) ;
  pED2E->GetFaces(sED2Fs) ;
  sED1Fs.FindCommonElements(sED2Fs, sCommonFs) ;
  ULONG lNumCommon = sCommonFs.GetSize();
  SM_DBG_WARN_IF(lNumCommon > 1, _T( "Bad input into boolean, suspect multiple degenerate faces sharing 2 edges" )) ;
 
  // Degenerate face has 2 edges within tol of edge on other brep
  for(ii=0;ii<lNumCommon;ii++)
    {
      SmFace* pFace = sCommonFs[ii];
      SmTArray<SmEdge*> sEdges;
      pFace->GetEdges( sEdges) ;
      if(sEdges.GetSize() == 2 )
        {
          SmTArray<SmTopology*> sTopos;
          ULONG   nFound;
          if(pED1E->IsLamina() && pED2E->IsLamina() )
            {
              // Free floating degenerate face. How to resolve? Delete both?
              return SM_ERR;
            }
 
          // False: Do not delete Edges that become Wires:
          // those Edges might still exist in EdgeDefinitions, and will be dereferenced.
          pBrep1->DeleteFace( pFace, FALSE, TRUE, &sTopos) ;
 
          if(sTopos.FindElement( pED1E, nFound ) )
            {
              rlCanDelete = 1;
              return SM_SUCCESS;
            }
          if(sTopos.FindElement( pED2E, nFound ) )
            {
              rlCanDelete = 2;
              return SM_SUCCESS;
            }
        }
      else
        {
          SM_DBG_WARN( _T( "Bad input into boolean, face has unusual degeneracy." )) ;
          return SM_ERR;
        }
    } // end for each common face
 
  // If no common faces, or deleted common face w/out deleting edge, then we have coincident unstitched edges
  SM_DBG_WARN( _T( "Suspected that Brep in Boolean has coincident unstitched edges. Try stitching, then doing Boolean." )) ;
 
  // all done
  return SM_SUCCESS;
 
} // end sm_FindEDOnDegenFace

/*******************************************************************//**
PURPOSE: See whether two EdgeDefinitions can be combined into one.

USAGE NOTES---- 
***********************************************************************/
SmStatus SmProtoTopologyManager::CanCombineEdgeDefinitions
 (SmEdgeDefinition * pED1,         // in : TgtEdgeDef1 to test 
  SmEdgeDefinition * pED2,         // in : TgtEdgeDef2 to test
  ULONG            & rlCanDelete)  // out: 0 = neither  
                                   //      1 = pED1  
                                   //      2 = pED2 
{
  // init output
  rlCanDelete = 0 ;

  // locals
  SmTopology * pED1Topo1 = pED1->GetSrcTopo(1) ; NER( pED1Topo1) ;
  SmTopology * pED1Topo2 = pED1->GetSrcTopo(2) ; NER( pED1Topo2) ;
  SmTopology * pED2Topo1 = pED2->GetSrcTopo(1) ; NER( pED2Topo1) ;
  SmTopology * pED2Topo2 = pED2->GetSrcTopo(2) ; NER( pED2Topo2) ;
                                               
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
  if(bDebugMe) 
    {
      smgfx_Erase();
      if(FALSE) 
        { smgfx_SetLook( 1,2, 0,0,1) ; m_pBrep1->Draw(TRUE); sm_GraphicsLoop();
          smgfx_SetLook( 1,2, 0,1,0) ; m_pBrep2->Draw(TRUE); sm_GraphicsLoop();
          sm_GraphicsLoop();
        }
      sm_DrawTopology(pED1Topo1, 3,5, 0,0,1); sm_GraphicsLoop() ;
      sm_DrawTopology(pED1Topo2, 4,6, 0,1,1); sm_GraphicsLoop() ;
      sm_DrawTopology(pED2Topo1, 3,5, 1,0,1); sm_GraphicsLoop() ;
      sm_DrawTopology(pED2Topo2, 4,6, 0,1,0); sm_GraphicsLoop() ;
      sm_GraphicsLoop();

      smgfx_SetLook( 4,7, 1,0,0) ; pED1->Draw(); sm_GraphicsLoop() ;
      smgfx_SetLook( 4,8, 1,1,0) ; pED2->Draw(); sm_GraphicsLoop() ;
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

  // If they're sitting on the same topology in both Breps, combine. Remember these are 2 EdgeDefs on 1 ProtoEdge - EdgeDefs will have the same intervals 
  if(   pED1Topo1 == pED2Topo1  
     && pED1Topo2 == pED2Topo2)
    {
      rlCanDelete = 2 ;  //cbi arbitrary so far...  Look at Deviations (which we don't have yet)?
                         //cbi Face::FaceIntersect() doesn't return gaps, but it could.
                         //cbi SmPointClass objects have m_dGap3d.  Try that.
      return SM_SUCCESS ;
    }

  // If either pair is not connected in their Brep, can't combine.
  if(   sm_AreConnected(pED1Topo1, pED2Topo1) == FALSE
     || sm_AreConnected(pED1Topo2, pED2Topo2) == FALSE)
    { return SM_SUCCESS ; }

  // Check distance: if they're too far apart, we can't combine them.
  SmCurve    * pIntCurve1 = pED1->GetXSectCurve3d() ;
  SmCurve    * pIntCurve2 = pED2->GetXSectCurve3d() ;
  SmExtent1d   sDomain1   = pED1->GetXSectInterval() ;
  SmExtent1d   sDomain2   = pED2->GetXSectInterval() ;

  // Use the same criteria as in CanCombineVertexDefinitions().
  static double sdTolFactor = 5.0; // (ouch)
  double dTol  = pED1->GetTolerance() + pED2->GetTolerance();
  dTol *= sdTolFactor;
  if(! sm_AreSameCurve( pIntCurve1, sDomain1, pIntCurve2, sDomain1, dTol ) )
    { return FALSE; }

  // Check dimensionality - [vertex:0, edge:1, face:2]
  int lDim11 = sm_TopoDimension(pED1Topo1) ; 
  int lDim12 = sm_TopoDimension(pED1Topo2) ; 
  int lDim21 = sm_TopoDimension(pED2Topo1) ; 
  int lDim22 = sm_TopoDimension(pED2Topo2) ;

  // If dimensionality is different in Topo 1, check whether Topo 2 also allows a deletion.
  if(lDim11 < lDim21) { if(lDim12 <= lDim22)
                          { rlCanDelete = 2 ; }
                      }
  if(lDim21 < lDim11) { if(lDim22 <= lDim12)
                          { rlCanDelete = 1 ; }
                      }

  // If dimensionality is different in Topo 2, check whether Topo 1 also allows a deletion.
  if(lDim12 < lDim22) { if(lDim11 <= lDim21)
                          { rlCanDelete = 2; }
                      }
  if(lDim22 < lDim12) { if(lDim21 <= lDim11 )
                          { rlCanDelete = 1; }
                      }

  // check for coincident edges (e.g. unstitched edges or degenerate faces)
  SmEdge * pED1E1 = SM_CAST_PTR(SmEdge, pED1Topo1) ;
  SmEdge * pED1E2 = SM_CAST_PTR(SmEdge, pED1Topo2) ;
  SmEdge * pED2E1 = SM_CAST_PTR(SmEdge, pED2Topo1) ;
  SmEdge * pED2E2 = SM_CAST_PTR(SmEdge, pED2Topo2) ;

  // Confirm all topologies are edges
  if(   pED1E1 != NULL 
     && pED1E2 != NULL 
     && pED2E1 != NULL 
     && pED2E2 != NULL)
    {
      if(   pED1E1 != pED2E1
         && pED1E2 == pED2E2)
        {
          ULONG lCanDelete = 0 ;
          sm_FindEDOnDegenFace(pED1E1, pED2E1, lCanDelete) ;
          if(lCanDelete != 0)
            {
              rlCanDelete = lCanDelete;
              return SM_SUCCESS;
            }
        }
      else if(   pED1E1 == pED2E1 
              && pED1E2 != pED2E2)
        {
          ULONG lCanDelete = 0 ;
          sm_FindEDOnDegenFace(pED1E2, pED2E2, lCanDelete) ;
          if(lCanDelete != 0)
            {
              rlCanDelete = lCanDelete;
              return SM_SUCCESS;
            }
        }
    } // end all SrcTopos are edges check

  // all done
  return SM_SUCCESS;

} // end SmProtoTopologyManager::CanCombineEdgeDefinitions

/*******************************************************************//**
PURPOSE: Split intersecting ProtoEdges at the XSect points while
         updating the ProtoTopoMgr ProtoLists and connection data between 
         and among the ProtoTopos and their TopoDefitions as needed.

USAGE NOTES---- 
   1. If an interior point XSect is found, 
       then split the PEs (and their EDs) at the intersection,
            creating a new ProtoVertex and ProtoEdge (and VD and ED) for each split.

       After a successful split, the originals will be the lower part of the domain
            of the SmProtoEdges, and the higher part will be new.

   2. if a range XSect is found,  Splits the ProtoEdges (and their EdgeDefinitions) 
       at both the start and stop of the range intersection 

   3. Makes sure that all of the new ProtoTopology is installed in the owning SmProtoTopologyManager.
***********************************************************************/
SmStatus SmProtoTopologyManager::SplitIntersectingProtoEdges
 (SmProtoEdge * pPE1, // in : 1st Tgt of ProtoEdge/ProtoEdge XSection
  SmProtoEdge * pPE2) // in : 2nd Tgt of ProtoEdge/ProtoEdge XSection
{
  // no work - ProtoEdges already meet at a ProtoVertex.
  if(   (pPE1->GetStartProtoVertex() == pPE2->GetStartProtoVertex())
     || (pPE1->GetStartProtoVertex() == pPE2->GetEndProtoVertex()  )
     || (pPE1->GetEndProtoVertex()   == pPE2->GetStartProtoVertex())
     || (pPE1->GetEndProtoVertex()   == pPE2->GetEndProtoVertex()  ))
    { return SM_SUCCESS; }

  // locals
  SmTArray<SmEdgeDefinition *> rEdgeDefs1       = pPE1->GetEdgeDefinitions() ;
  SmTArray<SmEdgeDefinition *> rEdgeDefs2       = pPE2->GetEdgeDefinitions() ;
  SmTArray<SmProtoEdge *>      sOverlappingPE1s ; pPE1->GetOverlappingPEs(sOverlappingPE1s) ;

  // no work - ProtoEdge does not have any EdgeDefinitions 
  if(   rEdgeDefs1.GetSize() < 1 
     || rEdgeDefs2.GetSize() < 1)
    { return SM_ERR; }   // [Reg_091215_simon_closed]

  // no work - Skip the pair if they're known to overlap. Will be handled during Resolve
  ULONG lIdx ;
  if(TRUE == sOverlappingPE1s.FindElement(pPE2, lIdx) )
    { return SM_SUCCESS ; }

  // locals - geometry,   cbi TODO: here's a little problem: where to get the Curves?  for now, just the first EdgeDef.
  SmCurve   * pCrv1 = rEdgeDefs1[0]->GetXSectCurve3d() ;
  SmCurve   * pCrv2 = rEdgeDefs2[0]->GetXSectCurve3d() ;
  SmExtent1d  sDom1 = pPE1->GetXSectInterval() ;
  SmExtent1d  sDom2 = pPE2->GetXSectInterval() ;

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe3=FALSE;
  if(bDebugMe3) 
    {                                
      if(FALSE) {
          smgfx_Erase() ;
          smgfx_SetLook( 1,2, 0,0,0) ; this->Draw(); sm_GraphicsLoop() ;
        }
      smgfx_SetLook(5,6, 1,0,0) ; if(pCrv1) pCrv1->Draw( &sDom1 ) ; sm_GraphicsLoop() ;
      smgfx_SetLook(5,6, 0,1,0) ; if(pCrv2) pCrv2->Draw( &sDom2 ) ; sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

  // XSect ProtoEdge curves
  SmSolutionArray sSols;
  SmStatus eStat = pCrv1->GlobalCurveIntersect( sDom1,  // in : thisCurve's interval for intersection
                                               *pCrv2,  // in : target OtherCurve
                                               sDom2,   // in : OtherCurve's interval for intersection
                                               m_dTol,  // in : XSectTol3d = max 3d distance between intersecting points
                                               sSols) ; // out: Array of Found Solutions: sSol.m_vStart[0] = thisCurve param
                                                        //                                sSol.m_vStart[1] = otherCurve param
  if(eStat != SM_SUCCESS || sSols.GetSize() == 0 )
    { return SM_SUCCESS; }

  // locals - new Topology
  SmProtoVertex * pNewPV = NULL ;
  SmProtoEdge   * pNewPE = NULL ;

  // Found an intersection.  If it's interior to either ProtoEdge,
  // then split it.  (The Split routine will check whether it's interior.)
  ULONG ii, lNumSols = sSols.GetSize();
  for(ii=0;ii<lNumSols;ii++)
    {
      SmSolution rSol = sSols[ii];

      // Point intersection branch
      if(rSol.m_eSolutionType == SM_ST_SINGLE_VALUE)
        {
          // split ProtoEdge1 at XSect - creates new ProtoVertex and ProtoEdge - reuses the old ProtoEdge
          double dT1 = rSol.m_vStart[0];
          double dT2 = rSol.m_vStart[1];

          // when Pt solution is in PE1 interior
          if(pPE1->GetXSectInterval().ContainsValue(dT1, -SM_EFF_ZERO))
            {
              pPE1->Split( dT1,      // in : Tgt Split param, checked to be interior value
                           pNewPV,   // out:  When dT is interior, new ProtoVertex at Split point
                           pNewPE) ; // out:  When dT is interior, new Upper Child Split ProtoEdge:[dT, IvlMax]
                                     // note: When dT is interior, this PE reused for Lower Child split ProtoEdge:[IvlMin, dT]
            }

          // when Pt solution is in PE2 interior
          if(pPE2->GetXSectInterval().ContainsValue(dT2, -SM_EFF_ZERO))
            {
              pPE2->Split( dT2,      // in : Tgt Split param, checked to be interior value
                           pNewPV,   // out:  When dT is interior, new ProtoVertex at Split point
                           pNewPE) ; // out:  When dT is interior, new Upper Child Split ProtoEdge:[dT, IvlMax]
                                     // note: When dT is interior, this PE reused for Lower Child split ProtoEdge:[IvlMin, dT]
            }
        } // end point XSect branch

      else if(rSol.m_eSolutionType == SM_ST_RANGE_OF_VALUES)
        {  // interval XSect branch

          // We handle this now.  [A..0612A]
          // WARN( _T("Intersecting ProtoEdges found a Range solution: review this case.\n")) ;

          SmProtoVertex *pNewPV11 = NULL, *pNewPV12 = NULL, *pNewPV21 = NULL, *pNewPV22 = NULL;
          SmProtoEdge   *pNewPE11 = NULL, *pNewPE12 = NULL, *pNewPE21 = NULL, *pNewPE22 = NULL;

          // Check whether the two PEs run in the same direction or opposite.
          // Note, for pPE1 (in [0]), rSol.m_vStart[0] < rSol.m_vEnd[0] always.  Check pPE2, in [1].
          SmBoolean bSameDir = ( rSol.m_vStart[1] < rSol.m_vEnd[1]) ;

          // T1:
          double dT11 = rSol.m_vStart[0];
          pPE1->Split( dT11,       // in : Tgt Split param, checked to be interior value
                       pNewPV11,   // out:  When dT is interior, new ProtoVertex at Split point
                       pNewPE11) ; // out:  When dT is interior, new Upper Child Split ProtoEdge:[dT, IvlMax]
                                   // note: When dT is interior, this PE reused for Lower Child split ProtoEdge:[IvlMin, dT]

          double dT21 = rSol.m_vStart[1];
          pPE2->Split( dT21,       // in : Tgt Split param, checked to be interior value
                       pNewPV21,   // out:  When dT is interior, new ProtoVertex at Split point
                       pNewPE21) ; // out:  When dT is interior, new Upper Child Split ProtoEdge:[dT, IvlMax]
                                   // note: When dT is interior, this PE reused for Lower Child split ProtoEdge:[IvlMin, dT]

          // Combine PVs:
          SmProtoVertex * pPV1ToCombine = sm_WhichPVToCombine(pPE1, dT11, pNewPV11) ;
          SmProtoVertex * pPV2ToCombine = sm_WhichPVToCombine(pPE1, dT21, pNewPV21) ;

          if(   pPV1ToCombine != NULL 
             && pPV2ToCombine != NULL
             && pPV1ToCombine != pPV2ToCombine)
            {
              pPV1ToCombine->CombineProtoVertex( pPV2ToCombine) ;

              pPV2ToCombine->ClearVertexDefinitions();
              this->RemoveProtoVertex( pPV2ToCombine) ;
              delete pPV2ToCombine; pPV2ToCombine=NULL;
            }

          // T2:
          // First decide which pieces to split.
          SmProtoEdge *pPE1ToSplit = pPE1;
          if(pNewPE11 != NULL )
            { pPE1ToSplit = pNewPE11; }

          SmProtoEdge *pPE2ToSplit = pPE2;
          if(bSameDir && pNewPE21 != NULL ) // if opposite direction, always split the original.
            { pPE2ToSplit = pNewPE21; }

          double dT12 = rSol.m_vEnd[0];
          pPE1ToSplit->Split( dT12,       // in : Tgt Split param, checked to be interior value
                              pNewPV12,   // out:  When dT is interior, new ProtoVertex at Split point
                              pNewPE12) ; // out:  When dT is interior, new Upper Child Split ProtoEdge:[dT, IvlMax]
                                          // note: When dT is interior, this PE reused for Lower Child split ProtoEdge:[IvlMin, dT]

          double dT22 = rSol.m_vEnd[1];
          pPE2ToSplit->Split( dT22,       // in : Tgt Split param, checked to be interior value
                              pNewPV22,   // out:  When dT is interior, new ProtoVertex at Split point
                              pNewPE22) ; // out:  When dT is interior, new Upper Child Split ProtoEdge:[dT, IvlMax]
                                          // note: When dT is interior, this PE reused for Lower Child split ProtoEdge:[IvlMin, dT]

          // Combine PVs:
          pPV1ToCombine = sm_WhichPVToCombine(pPE1ToSplit, dT12, pNewPV12) ;
          pPV2ToCombine = sm_WhichPVToCombine(pPE2ToSplit, dT22, pNewPV22) ;

          if(   pPV1ToCombine != NULL 
             && pPV2ToCombine != NULL
             && pPV1ToCombine != pPV2ToCombine)
            {
              pPV1ToCombine->CombineProtoVertex( pPV2ToCombine) ;

              pPV2ToCombine->ClearVertexDefinitions();
              this->RemoveProtoVertex( pPV2ToCombine) ;
              delete pPV2ToCombine; pPV2ToCombine=NULL;
            }

          // Combine ProtoEdges
          SmProtoEdge * pPE1ToCombine = (pNewPE11 != NULL) ? pNewPE11 : pPE1 ;
          SmProtoEdge * pPE2ToCombine = (bSameDir) ? ((pNewPE21 != NULL) ? pNewPE21 : pPE2 )  
                                                   : ((pNewPE22 != NULL) ? pNewPE22 : pPE2 ) ;

          pPE1ToCombine->CombineProtoEdge( pPE2ToCombine) ;

          this->RemoveProtoEdge( pPE2ToCombine) ;
          delete pPE2ToCombine; pPE2ToCombine=NULL;

        } // end Range solution branch.

    } // end iter each solution

  // all done
  return SM_SUCCESS ;

} // end SmProtoTopologyManager::SplitIntersectingProtoEdges

/*******************************************************************//**
PURPOSE: Remove NoEdge ProtoVertices coincident with other ProtoEdges

USAGE NOTES---- 
   1. legit NoEdge ProtoVertices can be generated when a pair of faces
      touch at a point like a bowl on a tabletop.
   2. Redundant No Edge ProtoVertices are generated by Face/Face XSections
      which XSect along a coincident Edge when that Edge is part of network
      of intersecting curves forming a critical point.
 This method is an attempt to find and remove those redudant ProtoVertices
 before entering the Resolve phase of the Boolean where they can confuse things.
***********************************************************************/
SmBoolean SmProtoTopologyManager::RemoveCoincidentNoEdgeProtoVertex // rtn: TRUE = pPV was coincident with pPE and removed, FALSE = not Coincident and no changes
 (SmProtoEdge   * pPE,  // in : Tgt ProtoEdge   of ProtoEdge/ProtoVertex XSection
  SmProtoVertex * pPV)  // in : Tgt ProtoVertex of ProtoEdge/ProtoVertex XSection
{
  // no work - ProtoEdge is already terminated by ProtoVertex.
  if(   (pPE->GetStartProtoVertex() == pPV)
     || (pPE->GetEndProtoVertex()   == pPV))
    { return FALSE ; }                                                                                                                                                                                                         

  // no work - ProtoEdge does not have any EdgeDefinitions 
  if(pPE->GetEdgeDefinitions().GetSize() < 1)
    { return FALSE ; }   // [Reg_091215_simon_closed] - the system retains No-EdgeDef ProtoEdges when it removes a ProtoEdge from the ProtoTopoMgr.

  // locals
  // ULONG ii ;
  SmTArray<SmEdgeDefinition *> rEdgeDefs        = pPE->GetEdgeDefinitions() ;
  SmPoint3d                    sVertexPos3d     = pPV->GetPosition() ;
  SmZoneTol3d                  sEdgeZoneTol3d   = pPE->GetTolerance() ;
  // SmZoneTol3d                  sVertexZoneTol3d = pPV->GetTolerance() ; not used
  SmXSectTol3d                 sXSectTol3d      = sEdgeZoneTol3d + sEdgeZoneTol3d ; 

  // locals - geometry,   cbi TODO: here's a little problem: where to get the Curves?  for now, just the first EdgeDef.
  SmCurve   * pCrv    = rEdgeDefs[0]->GetXSectCurve3d() ;
  SmExtent1d  sCrvIvl = pPE->GetXSectInterval() ;

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe3=FALSE;
  if(bDebugMe3) 
    {                                
      if(FALSE) 
        { smgfx_Erase() ;
          smgfx_SetLook( 1,2, 0,0,0) ; this->Draw(); sm_GraphicsLoop() ;
        }
      smgfx_SetLook(5,6, 1,0,0) ; if(pCrv) pCrv->Draw( &sCrvIvl ) ; sm_GraphicsLoop() ;
      smgfx_SetLook(5,6, 0,1,0) ; if(pPV) pPV->Draw() ; sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

  // XSect ProtoEdgeCurve/ProtoVertexPosition
  SmSolutionArray sSols;
  SmStatus eStat = pCrv->GlobalPointSolve(sCrvIvl,               // in : search curve interval                                                                 
                                          SM_SO_INTERSECT,       // in : which solver operation to perform                                                     
                                          sVertexPos3d,          // in : Euclidean target point                                                                
                                          sXSectTol3d,           // in : Basically the distance tolerance sets up a range for the                              
                                                                 //    : distance measurements where additional answers may exist.                             
                                                                 //    : For example if the distance between two local minima/maxima                           
                                                                 //    : is greater than this tolerance, both answers will be returned.                        
                                                                 //    : For DropPointFast: Skip Solutions whose drop distance is too far away                 
                                                                 //    : operation == MINIMIZE save solution if cpdOptTargetDistance == NULL                   
                                                                 //    :                   or DropDist < cpdOptTargetDistance + dDistTol                       
                                                                 //    : operation == INTERSECT save solution if DropDist < dDistTol                           
                                          NULL,                  // in : If not NULL it will be:                                                               
                                                                 //    : SM_SO_AT_DISTANCE       = target distance,                                            
                                                                 //    : SM_SO_MINIMIZE/MAXIMIZE = distance limit.                                             
                                                                 //    : For example, to find a minimum value only if it less than                             
                                                                 //    : the target distance or maximum value only if it is greater than the target distance   
                                          NULL,                  // in : Vectors used in some of the solvers.                                                  
                                                                 //    : SM_SO_RAYFIRE,                  SM_SO_3D_SIGNED_DIRECTED_MINIMIZE                     
                                                                 //    : SM_SO_SIGNED_DIRECTED_MINIMIZE, SM_SO_DIRECTED_MINIMIZE                               
                                                                 //    : SM_SO_DIRECTED_MAXIMIZE,        SM_SO_PROJECTED_MINIMIZE                              
                                                                 //    : SM_SO_PROJECTED_MAXIMIZE                                                              
                                                                 //    : SM_SO_PROJECTED_TANGENT_THROUGH_POINT                                                 
                                          SM_SR_ALL,             // in : SM_SR_SINGLE=get best solution, SM_SR_ALL=get all solutions                           
                                          sSols) ;                // out: array of problem solutions reported as curve parameter values                         
  if(eStat != SM_SUCCESS || sSols.GetSize() == 0 )
    { return SM_SUCCESS; }

  // First idea that was causing problems
  //  // locals - new Topology
  //  SmProtoVertex * pNewPV = NULL ;
  //  SmProtoEdge   * pNewPE = NULL ;
  //  
  //  // for every XSect - split ProtoEdge  (GWC: should there only be one XSect and Split here?)
  //  for(ii=0;ii<sSols.GetSize();ii++)
  //    {
  //      SmSolution rSol = sSols[ii];
  //      double     dT   = rSol.m_vStart[0];
  //  
  //      // Point intersection branch
  //      if(   rSol.m_eSolutionType == SM_ST_SINGLE_VALUE
  //         && pPE->GetXSectInterval().ContainsValue(dT, -SM_EFF_ZERO))
  //        {
  //          // split ProtoEdge1 at XSect - creates new ProtoVertex and ProtoEdge - reuses the old ProtoEdge
  //          pPE->Split(dT,       // in : Tgt Split param, checked to be interior value
  //                     pNewPV,   // out:  When dT is interior, new ProtoVertex at Split point
  //                     pNewPE) ; // out:  When dT is interior, new Upper Child Split ProtoEdge:[dT, IvlMax]
  //                               // note: When dT is interior, this PE reused for Lower Child split ProtoEdge:[IvlMin, dT]
  //                     
  //        } // end point XSect branch
  //  
  //    } // end iter each solution
  // end First idea that was causing problems

  // Second idea remove NoEdge ProtoVertices coincident with other ProtoEdges 
  // When the ProtoVertex is Edgeless and XSects a ProtoEdge - remove it
  if(   pPV->GetNumProtoEdges() == 0
     && sSols.GetSize() > 0)
    {
      // the ProtoVertex is a redundant critical point coincident with an existing ProtoEdge - remove it
      RemoveProtoVertex(pPV) ; // does not delete pPV

      // all done
      return TRUE ;
    }
  else
    {
      // found a unmarked PV/PE intersection - thats okay if the Vertex connects to a PE that's in the same overlappingPEs group as this XSecting PE
      //  overlapping PEs are handled in Resolve and will remove this unmarked PV/PE intersection by replacing the overlappingPEs group with nonOverlapping PEs
      SM_ASSERT_MSG(pPV->IsOverlappingPEGroupMember() == TRUE, _T("SmProtoTopologyManager::RemoveCoincidentNoEdgeProtoVertex - Error - found an unhandled and unmarked PV/PE intersection not part of a OverlapGroup. May need a PE Split - this is a bug")) ; 
    }

  // all done
  return FALSE ;

} // end SmProtoTopologyManager::RemoveCoincidentNoEdgeProtoVertex

/*******************************************************************//**
PURPOSE: replace overlapping ProtoEdge groups generated from Edge/Edge and 
  Face/Face coincident XSections with new NonOverlapping PEs from 
  cleaner known-to-be-coincident Edge->Curve/Face classifications. 

USAGE NOTES---- The Face/Face intersection phase of the 
SmTopologyIntersector::IntersectTopology() method often finds multiple
intersesction and coincident interval solutions all describing the same geometry
of the Edge/Edge or Edge/Face coincidence but with overlapping intervals that
generate a group of ProtoEdges sharing the same Src Curve shape but
with overlapping (partially coincident) intervals.  

This method finds and replaces all the PEs in an overlapping group with a 
single set of NonOverlapping ProtoEdges generated by classifying the now 
known-to-be-coincident curve against an appropriate face in each input Brep.
The homgenized intervals of that CrvClassification pair that map
to topology in both Breps are used to create the new set of NonOverlapping
PEs representing the coincident geometry.
***********************************************************************/
SmStatus SmProtoTopologyManager::ResolveOverlappingProtoEdges()
{
  // locals 
  ULONG ii, jj, kk ;
  SmTArray<SmProtoEdge *>     & rPEs = m_sPEs ; 
  SmTArray<SmProtoEdge *>       sOverLappingGroup ; // group of mutually overlapping PEs
  SmTArray<SmProtoVertex *>     sOverlapPVs ;
  SmTArray<SmEdgeDefinition*>   sOverlapEDs ;
  SmTArray<SmVertexDefinition*> sOverlapVDs ;
  SmBoolean bFoundOverlapGroup = FALSE ;
#ifdef SM_DEBUG_CODE // for now - gather this list only in debug mode to check the assumption that a a OverlapPE group will have just 0 or 1 CoinPFs
  SmTArray<SmProtoFace *>       sCoinPFGroup ;      // list of CoinPFs to any of the PEs in the sOverlappingGroup
#endif // SM_DEBUG_CODE

#ifdef SM_DEBUG_CODE
ULONG iDumpLevel = 5 ; // -1 = no output
                       //  0 = headers, ProtoVert, Edge, and Face counts
                       //  5 = List of ProtoVerts, Edges, Faces. Repeat ProtoVert, Edge, and Face counts
                       // 15 = Brep1, Brep2, and SmTopologyIntersector dumps
SmBoolean bDebugMe=FALSE;
  if(bDebugMe) 
    {   
      smos_WriteBuffer(_T("\n\\************************\nEntering ResolveOverlappingProtoEdges()\n**************\\")) ; 
      Dump(iDumpLevel) ; 

      if(FALSE) 
        { smgfx_Erase() ; }
      smgfx_SetLook( 3,4, 0,0,0) ; this->Draw(); sm_GraphicsLoop() ;
      smgfx_SetLook( 1,2, 0,0,1) ; GetWhichBrep(1)->Draw(); sm_GraphicsLoop() ;
      smgfx_SetLook( 1,2, 0,1,0) ; GetWhichBrep(2)->Draw(); sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE
  
  // For each overlap: find one good intersection based on lowest dimensional topology.
  
  // For each PE (at least one of the SrcTopos should be an edge.)
  for(ii=0;ii<rPEs.GetSize();ii+=bFoundOverlapGroup ? 0 : 1)  // repeat index when rPEs gets changed by removing a OverlappingPEs group
    {
      SmProtoEdge * pPE = rPEs[ii] ;
      pPE->GetOverlappingPEGroup(sOverLappingGroup) ;
      bFoundOverlapGroup = sOverLappingGroup.GetSize() > 0 ;

      // skip PEs not in a Overlapping PE Group
      if(bFoundOverlapGroup == FALSE )
        { continue ; }
  
      // arrive here when 
      //  sOverlappingPEs = accumulated list of pPE[ii]->OverlappingPE[jj]->OverlappingPEs
  
      // lowest topology search locals
      SmTopology * pTopo1 = NULL ;
      SmTopology * pTopo2 = NULL ;
      ULONG        lDim1  = 3 ;
      ULONG        lDim2  = 3 ;
  
      // For every Overlap PE - 1. accumulate list of CoincPFs, // for now - just in debug mode to check assumptions
      //                        2. save lowest order SrcTopology
      for(jj=0;jj<sOverLappingGroup.GetSize();jj++)
        {
          SmProtoEdge * pOverlapPE = sOverLappingGroup[jj] ; 

#ifdef SM_DEBUG_CODE
          // accumulate CoincPFs group
          sCoinPFGroup.AppendUnique(*pOverlapPE->GetCoincProtoFaces()) ;
#endif // SM_DEBUG_CODE          

          // search all EdgeDefs to save lowest dim SrcTopopology
          sOverlapEDs = pOverlapPE->GetEdgeDefinitions() ;

          // skip PEs with no EdgeDefs
          if(sOverlapEDs.GetSize() == 0 )
            { continue; }  // Empty ProtoEdges are not a problem.
  
          // arbitrary:  use the 1st EdgeDef - could be some kind of average of EdgeDefs
          SmEdgeDefinition * pED = sOverlapEDs[0];
  
          SmTopology * pThisTopo1 = pED->GetSrcTopo(1) ;
          SmTopology * pThisTopo2 = pED->GetSrcTopo(2) ;
          ULONG        lThisDim1  = sm_TopoDimension(pThisTopo1) ;
          ULONG        lThisDim2  = sm_TopoDimension(pThisTopo2) ;
  
          // check and save lowest dim Topo1 object
          if(lThisDim1 == 0)          { SM_DBG_WARN( _T( "Unexpected vertex topology on EdgeDefinition" )) ; }
          else if(lThisDim1 < lDim1 ) { lDim1  = lThisDim1 ;
                                        pTopo1 = pThisTopo1 ;
                                      }
  
          // check and save lowest dim Topo2 object
          if(lThisDim2 == 0 )         { SM_DBG_WARN( _T( "Unexpected vertex topology on EdgeDefinition" )) ; }
          else if(lThisDim2 < lDim2 ) { lDim2  = lThisDim2 ;
                                        pTopo2 = pThisTopo2 ;
                                      }
        } // end iter overlapping ProtoEdges finding lowest order SrcTopologies
  
      // arrive here when:
      //   pTopo1             = 1st min dim Brep1 src object found, lDim1 = dim of pTopo1
      //   pTopo2             = 1st min dim Brep2 src object found, lDim2 = dim of pTopo2
      //     Expect one Topo object an Edge, the other a Face or an edge
      //   sCoinPFGroup       = list of all ProtoFaces listed in any OverlappingPE m_sCoincPFs list - for now just in debug mode
      //   bFoundOverlapGroup = TRUE - found a Overlapping Group to process
  
      // locals
      SmBoolean                 bReplaceCoinPE = FALSE ;
      SmProtoFace             * pPF            = NULL ;
      SmTArray<SmProtoFace *>   sPFs           = GetProtoFaces() ;
      
      // gwc - does this next section need to be rewritten for multiple CoinPF cases
#ifdef SM_DEBUG_CODE
        {
          SM_ASSERT_MSG(sCoinPFGroup.GetSize() <= 1, _T("SmProtoTopologyManager::ResolveOverlappingProtoEdges - Failed single CoincPF assumption - rewrite next section for multiple-CoincPFs using the sCoinPFGroup")) ;
        }
#endif // SM_DEBUG_CODE

      // PFs keep a list of coincident PEs. Remember when any Overlap PE is also a PF coincident PE so that stale pointers can be refreshed.
      
      // remember the first pPF where any of the sOverLappingGroup are listed as a pPF->CoinProtoEdge - gwc: should we check all pPF and keep a list?
      for(jj=0;jj<sPFs.GetSize();jj++)
        {
          // remember 1st ProtoFace found with CoinPEs from this OverlappingPE group - gwc: should we check all pPF and keep a list?
          pPF = sPFs[jj] ; 

          // locals
          SmTArray<SmProtoEdge *> * pCoinPEs = pPF->GetCoincProtoEdges() ;
          ULONG                     sSize    = pCoinPEs->GetSize() ;
  
          // remove all Overlapping PEs from the pF->CoinPE list, set  pCoinPEs = pCoinPEs - sOverLappingGroup
          pCoinPEs->RemoveElements(sOverLappingGroup, *pCoinPEs) ;

          // remember when elements were removed from pCoinPEs
          if(pCoinPEs->GetSize() != sSize )
            {
              bReplaceCoinPE = TRUE ;
              break ;
            }
        } // end check for these PEs on existing PFs
  
      // next: Delete all OverlappingPE->EdgeDef and any PV that has no remaining PEs
      sOverlapPVs.ReSet() ;

      // for every sOverlappingPE - delete it after removing PE's from ProtoTopoMgr model - accumulate Overlapping->EndProtoVertices
      //   gwc: I believe the intent here is to remove the PE from the ProtoTopo structure and then delete it.
      //          1. removeOverlap PE from its End PV->PE lists
      //          2. remove all the OverlappE->Overlapping PEs
      //          3. clear the OverlapPE->End PV pointers
      //          4. make sure CoincPFGroup pointers are saved - already done
      //          5. Remove OverlapPE from ProtoTopoMgr
      //          6. Delete the OverlapPE
      for(jj=0;jj<sOverLappingGroup.GetSize();jj++)
        {
          SmProtoEdge * pOverlapPE  = sOverLappingGroup[jj];
                        sOverlapEDs = pOverlapPE->GetEdgeDefinitions();

          // accumulate the End and Start ProtoVertices - later find and delete the PVs that end up with no PEs because those are just artifacts of intersecting Breps with coincident edges
          sOverlapPVs.AddUnique(pOverlapPE->GetStartProtoVertex()) ;
          sOverlapPVs.AddUnique(pOverlapPE->GetEndProtoVertex()) ;

          // Remove and delete all EdgeDefs from this OverlappingPE
          for(kk=0;kk<sOverlapEDs.GetSize();kk++)
            { 
              SmEdgeDefinition * pOverlapED = sOverlapEDs[kk] ;

              RemoveEdgeDefinition( pOverlapED, FALSE) ; // FALSE =  Don't remove the PE if empty, otherwise keep track and remove from PF.  note: empty PEs don't hurt anything
              pOverlapED->Destruct() ; delete pOverlapED ; pOverlapED = NULL ;
            }

          // remove PE from End and Start ProtoVertex->PE list 
          pOverlapPE->GetStartProtoVertex()->RemoveProtoEdge(pOverlapPE) ; 
          pOverlapPE->GetEndProtoVertex()->RemoveProtoEdge(pOverlapPE) ; 

          // remove the OverlappingPEs from this PE (don't need to do back pointers because all back pointers belong to PEs in this sOverlappingGroup and they'll be cleaned in subsequent iters)
          pOverlapPE->GetOverlappingPEs().ReSet() ; 

          // clear the PEs end ProtoVertex pointers
          pOverlapPE->SetStartProtoVertex(NULL) ;
          pOverlapPE->SetEndProtoVertex  (NULL) ;

          // remove OverlapPE from ProtoTopoMgr
          RemoveProtoEdge(pOverlapPE) ; 

          // all done with this pOverlap PE
          if(pOverlapPE) { delete pOverlapPE ; pOverlapPE = NULL ; }
  
        } // End iter jj, deleting the EdgeDefs of every overlappingPe
        
      // for every accumulated OverlappingPE->End ProtoVertex with no other PEs, delete it after removing and deleting all its VertexDefs
      for(jj=0;jj<sOverlapPVs.GetSize();jj++)
        {
          SmProtoVertex *pOverlapPV = sOverlapPVs[jj] ;

          // when PV has no other PEs - delete it after deleting all its VertexDefs
          if(pOverlapPV->GetNumProtoEdges() == 0 )
            {
              sOverlapVDs = pOverlapPV->GetVertexDefinitions() ;
              for(kk=0;kk<sOverlapVDs.GetSize();kk++)
                { 
                  SmVertexDefinition *pVD = sOverlapVDs[kk];
                  RemoveVertexDefinition( sOverlapVDs[kk], TRUE) ; // TRUE = delete ProtoVertexOwner when PVOwner->VertexDefCnt goes to zero
                  delete pVD;  pVD = NULL;
                } // end iter kk, every 
            } // end zombie PV check
        } // end deletion of unused PVs
  
#ifdef SM_DEBUG_CODE
  if(bDebugMe) 
    {   
      // gwc note - verify the assumption that the sPEs list remains constant for entries < ii after the OVerlap Group has been removed and replaced by the NonOverlap Group
      smos_WriteBuffer(_T("\n\\************************\nAfter removing Overlapping Group from ProtoTopoMgr in ResolveOverlappingProtoEdges()\n**************\\")) ; 
      Dump(iDumpLevel) ; 
      rPEs.Dump() ;

      if(FALSE) 
        { smgfx_Erase() ; }
      smgfx_SetLook( 3,4, 0,0,0) ; this->Draw(); sm_GraphicsLoop() ;
      smgfx_SetLook( 1,2, 0,0,1) ; GetWhichBrep(1)->Draw(); sm_GraphicsLoop() ;
      smgfx_SetLook( 1,2, 0,1,0) ; GetWhichBrep(2)->Draw(); sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

      // arrive here when:
      //   pTopo1         = 1st min dim Brep1 src object found, lDim1 = dim of pTopo1
      //   pTopo2         = 1st min dim Brep2 src object found, lDim2 = dim of pTopo2
      //   bReplaceCoinPE = TRUE when any Overlap PE is also a Face coincident PE
      //   pF             = First Face where any Overlapping PE was listed as a CoinFace (bReplaceCoinPE == TRUE), else NULL
      // all PE[ii]->OverlapGroupPEs have been removed from ProtoTopo model and deleted
      // all PE[ii]->OverlapGroupPVs whose PE counts went to zero have been deleted
      
      // next:
      // Expect one Topo object to be an Edge, the other a Face or an Edge
      // classify the Edge->Curve against SrcFaces connecting to that edge in both Breps
      //  homogenize the pair of CurveClassifications and use the CurveIntervals to create a new non-overlapping set of PEs to represent the Edge coincidence.
      //  restore the CoincPFs lists

      // locals
      SmEdge * pE1 = SM_CAST_PTR( SmEdge, pTopo1) ;
      SmEdge * pE2 = SM_CAST_PTR( SmEdge, pTopo2) ;
      SmFace * pF1 = SM_CAST_PTR( SmFace, pTopo1) ;
      SmFace * pF2 = SM_CAST_PTR( SmFace, pTopo2) ;
  
      SmTArray<SmCurveClassification *> sCC1s ;
      SmTArray<SmCurveClassification *> sCC2s ;
      SmObjsDelete< SmCurveClassification *> sClean1( &sCC1s) ;
      SmObjsDelete< SmCurveClassification *> sClean2( &sCC2s) ;
  
      // Get curves to generate PEs from

      // When SrcTopo is Edge-Edge branch
      if(pE1 && pE2) 
        {
          // SM_DBG_WARN(_T("A non-symmetric boolean operation is occuring."))  // gwc: I don't understand this warning
  
          // get the curves fromm the Brep SrcEdges
          SmSolutionArray    sSols;
          SmBoolean          bAdditionalWorkNeeded;
          SmTArray<SmFace*>  sFaces1 ; pE1->GetFaces(sFaces1) ;
          SmTArray<SmFace*>  sFaces2 ; pE2->GetFaces(sFaces2) ;

          // get XSect Curves from SrcTopo Edges
          SmCurve * pC1 = pE1->GetCurve() ;
          SmCurve * pC2 = pE2->GetCurve() ;
          
          // arbitrary - pick XSectFace as first Edge Face - (gwc: does this work for radial spine edges - probably)
          pF1 = sFaces1[0] ;
          pF2 = sFaces2[0] ;
  
          // Find the Edge->Curve coincidences
          pC1->GlobalCoincidenceChecker( pE1->GetInterval(),    // in : 'this' curve interval to examine
                                        *pC2,                   // in : other curve
                                         pE2->GetInterval(),    // in : other-curve interval to examine
                                         m_dTol,                // in : max separation distances
                                         bAdditionalWorkNeeded, // out: TRUE = may be additional intersections between
                                                                //             the curve.
                                                                //      FALSE= all of one of the curves has been used in the
                                                                //             coincidence and no more solutions can exist unless
                                                                //             there is something strange like a self intersection.
                                         sSols) ;               // out: array of coincident segments 
                                                                // in : TRUE = snap to ends within tolerance 
                                                                //      FALSE= don't snap near end solutions to endPoints 
                                                                //      default:[TRUE] for compatibility with original behavior
          // for every coincident solution
          for(jj=0;jj<sSols.GetSize();jj++)
            {
              SmSolution sSol = sSols[jj];
  
              if(sSol.m_eSolutionType == SM_ST_RANGE_OF_VALUES )
                {
                  // Classify the coincidence against each face using the 1st set of data in the Coincidence Solutions
                  // PEs expect the same curve classified against each face
                  SmExtent1d              sIvl1(sSol.m_vStart[0], sSol.m_vEnd[0]) ;
                  SmCurveClassification * pCrvClass1 = new ( GetContext() ) SmCurveClassification(pC1, sIvl1, NULL, m_dTol) ;
                  SmCurveClassification * pCrvClass2 = new ( GetContext() ) SmCurveClassification(pC1, sIvl1, NULL, m_dTol) ;

                  // Classify the coincident curves against their faces
                  pF1->CurveOnClassify(TRUE, *pCrvClass1) ;
                  pF2->CurveOnClassify(TRUE, *pCrvClass2) ;
  
                  // ExtractGeometry locals
                  SmCurve               * aCData1[16];
                  SmTArray<SmCurve*>      s3DCurves2( 16, aCData1) ;
                  SmObjsDelete<SmCurve*>  sClean3DC( &s3DCurves2) ;
  
                  // get curves for each intervalClassification contained by both faces
                  SER( pCrvClass1->ExtractGeometry(*GetContext(), // in : context for constructing new geometry
                                                   *pCrvClass2,   // in : other target CurveClassification
                                                   s3DCurves2)) ; // out: a curve for every common interval classified to topology in both input CrvClassifications
  
                  // Make sure that (one of) m_vTParam, m_vUVParam, or m_vUVWParam is set in each PointClass object.
                  pCrvClass1->SetPointClassParameters() ;
                  pCrvClass2->SetPointClassParameters() ;
  
                  // accumulate the curve classifications
                  sCC1s.Add( pCrvClass1) ;
                  sCC2s.Add( pCrvClass2) ;
                } // end is range solution check
            } // end iter jj, every coincident solution
  
#ifdef SM_DEBUG_CODE
          // One curve has to be chosen for the ProtoEdges. 
          //  In the release loop above, the 1st set of data from sSol is used.
          //  In the debug Loop below,   the 2nd set of data from sSol is used.
          // Compare The list of curve classifications made from the two sets of Sol data to see the impact of curve choice.
          ULONG di ;
          SmBoolean bDebugSymmetry = FALSE;
          if(bDebugSymmetry)
            {
              SmTArray<SmCurveClassification *>     sDebugCC1s ;
              SmTArray<SmCurveClassification *>     sDebugCC2s ;
              SmObjsDelete<SmCurveClassification *> sDebugClean1(&sDebugCC1s) ;
              SmObjsDelete<SmCurveClassification *> sDebugClean2(&sDebugCC2s) ;

              // for every coincidence solution
              for(di=0;di<sSols.GetSize();di++)
                {
                  SmSolution sSol = sSols[di];

                  if(sSol.m_eSolutionType == SM_ST_RANGE_OF_VALUES )
                    {
                      // Classify the coincidence against each face using the 2nd set of data in the Coincidence Solutions
                      // PEs expect the same curve classified against each face
                      SmExtent1d sIvl( sSol.m_vStart[1], sSol.m_vEnd[1]) ;             // <-- here's the change: index values changed from 0 to 1
                      SmCurveClassification * pCrvClass1 = new ( GetContext() ) SmCurveClassification( pC2, sIvl, NULL, m_dTol) ;
                      SmCurveClassification * pCrvClass2 = new ( GetContext() ) SmCurveClassification( pC2, sIvl, NULL, m_dTol) ;
                     
                      // Classify the coincident curves against their faces
                      pF1->CurveOnClassify( TRUE, *pCrvClass1) ;
                      pF2->CurveOnClassify( TRUE, *pCrvClass2) ;

                      // ExtractGeometry locals
                      SmCurve               * aCData1[16];
                      SmTArray<SmCurve*>      s3DCurves2( 16, aCData1) ;
                      SmObjsDelete<SmCurve*>  sClean3DC( &s3DCurves2) ;

                      // get curves for each interval contained by both faces
                      SER( pCrvClass1->ExtractGeometry(*GetContext(),  // in : context for constructing new geometry
                                                       *pCrvClass2,    // in : other target CurveClassification
                                                       s3DCurves2)) ;  // out: a curve for every common interval classified to topology in both input CrvClassifications

                      // Make sure that (one of) m_vTParam, m_vUVParam, or m_vUVWParam is set in each PointClass object.
                      pCrvClass1->SetPointClassParameters();
                      pCrvClass2->SetPointClassParameters();

                      // accumulate the curve classifications
                      sDebugCC1s.Add( pCrvClass1) ;
                      sDebugCC2s.Add( pCrvClass2) ;
                    } // end for each ST Range
                } // end for each solution

              // Pretty print Curve classifications...
              TCHAR sBuff[SM_TBLOCK_SIZE];
              smos_sprintf(sBuff, _T("%s"), _T("\nDump of SmCurveClassifications from default curve against Face 1: \n")) ; for(di=0;di<sCC1s.GetSize();di++)
                                                                                                                    { sCC1s[di]->Dump(); }
              smos_sprintf(sBuff, _T("%s"), _T("\nDump of SmCurveClassifications from default curve against Face 2: \n")) ; for(di=0;di<sCC2s.GetSize();di++)
                                                                                                                    { sCC2s[di]->Dump(); }
              smos_sprintf(sBuff, _T("%s"), _T("\nDump of SmCurveClassifications from *other* curve against Face 1: \n")) ; for(di=0;di<sDebugCC1s.GetSize();di++)
                                                                                                                    { sDebugCC1s[di]->Dump(); }
              smos_sprintf(sBuff, _T("%s"), _T("\nDump of SmCurveClassifications from *other* curve against Face 2: \n")) ; for(di=0;di<sDebugCC2s.GetSize();di++)
                                                                                                                    { sDebugCC2s[di]->Dump(); }
            }
#endif // SM_DEBUG_CODE 
        } // end SrcTopo Edge-Edge branch

      // else SrcTopo Edge-Face branch
      else if(   (pE1 && pF2) 
              || (pF1 && pE2))
        { 
          SmTArray<SmFace*> sFaces;

          // When we have one Edge and one Face, use the Edge->Curve
          SmEdge  * pE = ( pE1 ) ? pE1 : pE2;
          SmCurve * pC = pE->GetCurve();

          // Create CrvClassifications for PC over the PE interval to be classified against the SrcBrep XSectingFaces
          SmCurveClassification * pCrvClass1 = new ( GetContext() ) SmCurveClassification( pC, pE->GetInterval(), NULL, m_dTol) ;
          SmCurveClassification * pCrvClass2 = new ( GetContext() ) SmCurveClassification( pC, pE->GetInterval(), NULL, m_dTol) ;

          // Pick a SrcFace object for the SrcEdge object
          pE->GetFaces(sFaces) ;
          if(pE == pE1) { pF1 = sFaces[0] ; }
          else          { pF2 = sFaces[0] ; }

          // Classify Edge->Curve against both SrcFaces
          pF1->CurveOnClassify(TRUE, *pCrvClass1) ;
          pF2->CurveOnClassify(TRUE, *pCrvClass2) ;

          // ExtractGeometry locals
          SmCurve               * aCData1[16];
          SmTArray<SmCurve*>      s3DCurves2( 16, aCData1) ;
          SmObjsDelete<SmCurve*>  sClean3DC( &s3DCurves2) ;

          // get curves for each interval contained by both faces
          SER( pCrvClass1->ExtractGeometry(*GetContext(),  // in : context for constructing new geometry
                                           *pCrvClass2,    // in : other target CurveClassification
                                           s3DCurves2 )) ; // out: a curve for every common interval classified to topology in both input CrvClassifications

          // Make sure that (one of) m_vTParam, m_vUVParam, or m_vUVWParam is set in each PointClass object.
          pCrvClass1->SetPointClassParameters() ;
          pCrvClass2->SetPointClassParameters() ;

          // accumulate the curve classifications
          sCC1s.Add( pCrvClass1) ;
          sCC2s.Add( pCrvClass2) ;
        } // end SrcTopo Edge-Face branch

      // For each curve classification, create the new PEs
      for(jj=0;jj<sCC1s.GetSize();jj++)
        {
          SmCurveClassification * pCrvClass1 = sCC1s[jj] ;
          SmCurveClassification * pCrvClass2 = sCC2s[jj] ;

#ifdef SM_DEBUG_CODE
          SmBoolean bDebugMe2 = FALSE;
          if(bDebugMe2 )
            {
              pCrvClass1->Dump();
              pCrvClass2->Dump();

              smgfx_SetLook( 5, 7, 1, 0, 0) ;
              pCrvClass1->Draw( FALSE) ; sm_GraphicsLoop();  // FALSE: don't draw Objects
              sm_GraphicsLoop();
            }
#endif // SM_DEBUG_CODE

          // for every CrvClassification interval
          for(kk=0;kk<pCrvClass1->GetSize();kk++)
            {
              SmProtoEdge * pNewPE = NULL ;

              // Find or create a ProtoEdge from two SmCurveIntervals.
              FindOrCreateProtoEdge((*pCrvClass1)[kk], // in : Face1 CrvInterval classification from a face pair XSection
                                    (*pCrvClass2)[kk], // in : Face2 CrvInterval classification from a face pair XSection
                                    pNewPE) ;          // out: new or found ProtoEdge
                                                       // out: optional, TRUE  = rpNewPE was created, NULL to ignore, default:[NULL]
                                                       //                FALSE = rpNewPE was found
                                                       // 
              // when any OverlappingPE was a CoinPE in pPF - add all New or Found PEs back into the PF->CoincPE list
              if(pNewPE && bReplaceCoinPE)
                {
                  SmTArray< SmProtoEdge* > * pCoinPEs;
                  pCoinPEs = pPF->GetCoincProtoEdges();
                  pCoinPEs->AddUnique(pNewPE) ;
                }
            } // end for each curve interval
        } // end for each curve classification

      // no need to refresh the rPEs list here since its a reference to the m_sPEs list which has been updated by the above edits

#ifdef SM_DEBUG_CODE
      if(bDebugMe) 
        {   
          // gwc note - verify the assumption that the sPEs list remains constant for entries < ii after the OVerlap Group has been removed and replaced by the NonOverlap Group
          smos_WriteBuffer(_T("\n\\************************\nAfter adding new NonOverlapping Group to ProtoTopoMgr in ResolveOverlappingProtoEdges()\n**************\\")) ; 
          Dump(iDumpLevel) ; 
          rPEs.Dump() ; 
     
          if(FALSE) 
            { smgfx_Erase() ; }
          smgfx_SetLook( 3,4, 0,0,0) ; this->Draw(); sm_GraphicsLoop() ;
          smgfx_SetLook( 1,2, 0,0,1) ; GetWhichBrep(1)->Draw(); sm_GraphicsLoop() ;
          smgfx_SetLook( 1,2, 0,1,0) ; GetWhichBrep(2)->Draw(); sm_GraphicsLoop() ;
          sm_GraphicsLoop() ;
        }
#endif // SM_DEBUG_CODE

    } // end for each ProtoEdge

#ifdef SM_DEBUG_CODE
  if(bDebugMe) 
    {   
      smos_WriteBuffer(_T("\n\\************************\nExiting ResolveOverlappingProtoEdges()\n**************\\")) ; 
      Dump(iDumpLevel) ; 

      if(FALSE) 
        { smgfx_Erase() ; }
      smgfx_SetLook( 3,4, 0,0,0) ; this->Draw(); sm_GraphicsLoop() ;
      smgfx_SetLook( 1,2, 0,0,1) ; GetWhichBrep(1)->Draw(); sm_GraphicsLoop() ;
      smgfx_SetLook( 1,2, 0,1,0) ; GetWhichBrep(2)->Draw(); sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

  // all done
  return SM_SUCCESS;

} // end SmProtoTopologyManager::ResolveOverlappingProtoEdges()

///////////////////////////////////////////
// Utility methods: Draw(), Dump(), etc.
///////////////////////////////////////////

/*******************************************************************//**
PURPOSE: Dump this SmVertexDefinition

USAGE NOTES---- 
 SmVertexDefinition::Dump()
  Dump VertexDef given lIdx val, 
                       thisPtr, 
                       tol, 
                       position,
                       SrcTopo1 topology type (SmVertex, SmEdge, SmFace)
                       SrcTopo2 topology type (SmVertex, SmEdge, SmFace)
  iDumpLevel >= 20 Dump  associated PointClassification1->Object
                         associated PointClassification2->Object
***********************************************************************/
void SmVertexDefinition::Dump(int iDumpLevel, ULONG * lIdx) const
{
  TCHAR sBuff[SM_TBLOCK_SIZE];

  if(lIdx != NULL )
    { smos_sprintf(sBuff, _T("\n    Dump of SmVertexDefinition %lu: 0x%p:  Tolerance %5.8lf\n      Position:  "), *lIdx, this, m_dTol) ; }
  else
    { smos_sprintf(sBuff, _T("\n    Dump of SmVertexDefinition 0x%p:  Owning ProtoVertex 0x%p        Tolerance %5.8lf\n      Position:  "), this, m_pMyProtoVertex, m_dTol) ; }
  smos_WriteBuffer(sBuff);
  m_sPosition.Dump();

  SmPointClassificationType eType1 = this->GetSrcTopo_TYPE( 1) ;
  TCHAR sTypeString1[32];
  smos_snprintf( sTypeString1, 32, _T("%s"), ( eType1==SM_PC_VERTEX ) ? _T("SmVertex ")
                          : ( eType1==SM_PC_EDGE   ) ? _T("SmEdge   ")
                          : ( eType1==SM_PC_FACE   ) ? _T("SmFace   ")
                          : _T("<Unknown Type> ")) ;
  SmPointClassificationType eType2 = this->GetSrcTopo_TYPE( 2) ;
  TCHAR sTypeString2[32];
  smos_snprintf( sTypeString2, 32, _T("%s"), ( eType2==SM_PC_VERTEX ) ? _T("SmVertex ")
                          : ( eType2==SM_PC_EDGE   ) ? _T("SmEdge   ")
                          : ( eType2==SM_PC_FACE   ) ? _T("SmFace   ")
                          : _T("<Unknown Type> ")) ;

  smos_sprintf(sBuff, _T("\n      Topology object in Brep 1: %s 0x%p  Brep 2: %s 0x%p\n"), sTypeString1, m_sPC1.GetObject(), sTypeString2, m_sPC2.GetObject() ) ;
  smos_WriteBuffer(sBuff);
  if(iDumpLevel >= 20 ) 
    {
      m_sPC1.GetObject()->Dump();
      m_sPC2.GetObject()->Dump();
    }

  smos_sprintf(sBuff, _T("      Created Vertices in Breps: 0x%p  0x%p\n"), m_pFinalVtx1, m_pFinalVtx2) ;
  smos_WriteBuffer(sBuff);

} // end of SmVertexDefinition::Dump

/*******************************************************************//**
PURPOSE: Dump this SmProtoVertex

USAGE NOTES----
 SmProtoVertex::Dump()
  Dump thisPtr, VertexDefCnt, ProtoEdgesCnt, tol, position
  iDumpLevel >= 10: repeat ProtoEdgeCnt, 
                    for every ProtoEdge: dump ProtoMgr's ProtoEdgeIndex (not its pointer - easier to read indices than ptrs)
                                         dump every ProtoEdge->EdgeDef 
                                         iDumpLevel >= 20 with EdgeDef->m_PCs->Object dumps
***********************************************************************/
void SmProtoVertex::Dump(int iDumpLevel, ULONG * lIdx) const
{
  TCHAR sBuff[SM_TBLOCK_SIZE];
  ULONG ii ;
  ULONG lNumVDs = m_sVertexDefinitions.GetSize();
  ULONG lNumPEs = m_sProtoEdges.GetSize();

  // this, VertexDefs Cnt, ProtoEdge Cnt, tol
  if(lIdx != NULL) { smos_sprintf(sBuff, _T("\n    [%ld] "), *lIdx) ; smos_WriteBuffer(sBuff); }
  else             { smos_sprintf(sBuff,_T("%s"), _T("\n       ")) ; smos_WriteBuffer(sBuff); }

 smos_sprintf(sBuff, _T("SmProtoVertex:[0x%p], id[%lu], Tol:[%5.8lf], VertexDefCnt:[%lu], ProtoEdgeCnt:[%lu]"), 
                    this, m_pProtoTopoMgr->GetProtoVertexId(this), m_dTol, lNumVDs, lNumPEs) ; 
 smos_WriteBuffer(sBuff);

  // PV->PE ids
  smos_WriteBuffer(_T(", ProtoEdgeIds:[")) ;
  for(ii=0;ii<lNumPEs;ii++)
    { smos_sprintf(sBuff, _T("%lu"), m_pProtoTopoMgr->GetProtoEdgeId(m_sProtoEdges[ii])) ; smos_WriteBuffer(sBuff) ;
      if(ii<lNumPEs-1) { smos_WriteBuffer(_T(", ")) ; }
      else             { smos_WriteBuffer(_T(" ")) ; }
    }
  smos_WriteBuffer(_T("]")) ;
  // PV position
  smos_WriteBuffer(_T("\n      Position:")) ;
  m_sPosition.Dump();

  // when asked - complete dump
  if(iDumpLevel >= 10)
    { 
      // Note, SmTArray::Dump() doesn't dump the pointers...
      smos_sprintf(sBuff, _T("\n  ProtoEdges:  %ld\n"), lNumPEs) ;
      smos_WriteBuffer(sBuff);
     
      // locals: Print out the index in the ProtoManager: easier to track than the pointers.
      const SmTArray< SmProtoEdge* > & crAllPEs = GetProtoTopologyManager()->GetProtoEdges();
      SmProtoEdge *pPE = NULL;
      ULONG lPEIdx = 0;
     
      // for every protoEdge - dump protoMgr's ProtoEdgeIndex val
      for(ii=0;ii<lNumPEs;ii++)
        {
          pPE = m_sProtoEdges[ii];
          crAllPEs.FindElement( pPE, lPEIdx) ; // ProtoMgr's ProtoEdge Index for this PE
          smos_sprintf(sBuff, _T("    0x%p  [%4lu ]\n"), pPE, lPEIdx) ;
          smos_WriteBuffer(sBuff);
        }
     
      // VertexDefinition header
      smos_sprintf(sBuff, _T("\n  %ld VertexDefinitions:\n"), lNumVDs) ;
      smos_WriteBuffer(sBuff);
     
      // for every VertexDefinition
      for(ii=0;ii<lNumVDs;ii++)
        {
          m_sVertexDefinitions[ii]->Dump( iDumpLevel, &ii) ;
        }
    } // end iDumpLevel >= 10 check

} // end of SmProtoVertex::Dump

/*******************************************************************//**
PURPOSE: Dump this SmEdgeDefinition

USAGE NOTES---- 
***********************************************************************/
void SmEdgeDefinition::Dump(int iDumpLevel, ULONG * lIdx) const
{
  TCHAR sBuff[SM_TBLOCK_SIZE];

  if(lIdx != NULL )
    { smos_sprintf(sBuff, _T("\n    Dump of SmEdgeDefinition %lu: 0x%p: Tolerance %5.8lf\n"), *lIdx, this, m_dTol) ; }
  else
    { smos_sprintf(sBuff, _T("\n    Dump of SmEdgeDefinition 0x%p: Tolerance %5.8lf\n"), this, m_dTol) ; }
  smos_WriteBuffer(sBuff);

  // when asked - complete dump
  if(iDumpLevel >= 10)
    { 

    //cbi No sense dumping S&E PVs, the ProtoEdge does that.
    //  SmPoint3d sPos = GetStartProtoVertex()->GetPosition();
    //  smos_sprintf(sBuff, _T("     Start ProtoVertex 0x%p : %18.16f, %18.16f, %18.16f\n"), GetStartProtoVertex(),
    //      sPos.x, sPos.y, sPos.z) ;
    //  smos_WriteBuffer(sBuff);
    //  if(iDumpLevel >= 15 ) { GetStartProtoVertex()->Dump(); }
    //
    //  sPos = GetEndProtoVertex()->GetPosition();
    //  smos_sprintf(sBuff, _T("     End   ProtoVertex 0x%p : %18.16f, %18.16f, %18.16f\n"), GetEndProtoVertex(),
    //      sPos.x, sPos.y, sPos.z) ;
    //  smos_WriteBuffer(sBuff);
    //  if(iDumpLevel >= 15 ) { GetEndProtoVertex()->Dump(); }
    
      ULONG lCtrlPtCount = ( m_pXSectCurve3d != NULL ) ? m_pXSectCurve3d->GetNumberControlPoints() : 0;
    
      smos_sprintf(sBuff, _T("  Domain [ %18.16f, %18.16f ]  Int curves 0x%p 0x%p 0x%p    control point count %ld\n"),
         m_pXSectInterval.GetMin(), m_pXSectInterval.GetMax(), m_pXSectCurve3d, m_pXSectUVCurve1, m_pXSectUVCurve2, lCtrlPtCount) ;
    
      SmPointClassificationType eType1 = this->GetSrcTopo_TYPE( 1) ;
      TCHAR sTypeString1[32];
      smos_snprintf( sTypeString1, 32, _T("%s"),( eType1==SM_PC_VERTEX ) ? _T("SmVertex ")
                              : ( eType1==SM_PC_EDGE   ) ? _T("SmEdge   ")
                              : ( eType1==SM_PC_FACE   ) ? _T("SmFace   ")
                              : _T("<Unknown Type> ")) ;
      SmPointClassificationType eType2 = this->GetSrcTopo_TYPE( 2) ;
      TCHAR sTypeString2[32];
      smos_snprintf( sTypeString2, 32, _T("%s"),( eType2==SM_PC_VERTEX ) ? _T("SmVertex ")
                              : ( eType2==SM_PC_EDGE   ) ? _T("SmEdge   ")
                              : ( eType2==SM_PC_FACE   ) ? _T("SmFace   ")
                              : _T("<Unknown Type> ")) ;
    
      smos_sprintf(sBuff, _T("\n      Topology object in Brep 1: %s 0x%p  Brep 2: %s 0x%p\n"),
              sTypeString1, m_pSrcTopo1, sTypeString2, m_pSrcTopo2) ;
      smos_WriteBuffer(sBuff);
    
      if(iDumpLevel >= 30 ) 
        {
          m_pSrcTopo1->Dump();
          m_pSrcTopo2->Dump();
        }
    
      smos_sprintf(sBuff, _T("\n     3D Intersection curve: 0x%p ; UV curve 1: 0x%p ; UV curve 2: 0x%p\n"),
                                    m_pXSectCurve3d, m_pXSectUVCurve1, m_pXSectUVCurve2) ;
      smos_WriteBuffer(sBuff);
    
      smos_sprintf(sBuff, _T("     Created Edges in Breps: 0x%p  0x%p\n"), m_pFinalEdge1, m_pFinalEdge2) ;
      smos_WriteBuffer(sBuff);
    
      if(iDumpLevel >= 30 )
        {
          if(m_pXSectCurve3d != NULL ) { m_pXSectCurve3d->Dump( FALSE) ; }
          if(m_pXSectUVCurve1   != NULL ) { m_pXSectUVCurve1  ->Dump( FALSE) ; }
          if(m_pXSectUVCurve2   != NULL ) { m_pXSectUVCurve2  ->Dump( FALSE) ; }
        }
    }  // end iDumpLevel >= 10 check

} // end SmEdgeDefinition::Dump

/*******************************************************************//**
PURPOSE: Dump this SmProtoEdge

USAGE NOTES---- 
  SmProtoEdge::Dump()
    all Dump levels   : thisPtr
    all Dump levels   : EdgeDefCnt
    all Dump levels   : Overlapping PE Cnt
    all Dump levels   : CoincPF Cnt
    all Dump levels   : tol
    all Dump levels   : ProtoMgr backPtr
    iDumpLevel   >= 10: XSect Interval domain
    iDumpLevel   >= 10: StartProtoVertex: ptr, ProtoMgrIndx, position
      iDumpLevel >= 15:   and Dump StartProtoVertex
    iDumpLevel   >= 10: EndProtoVertex: ptr, ProtoMgrIndx, position
      iDumpLevel >= 15:   and Dump EndProtoVertex
    iDumpLevel   >= 10: Overlapping PE indices
    iDumpLevel   >= 10: CoincPF indices
    iDumpLevel   >= 10: every EdgeDef
***********************************************************************/
void SmProtoEdge::Dump(int iDumpLevel, ULONG * lIdx) const
{
  ULONG ii ;
  TCHAR sBuff[SM_TBLOCK_SIZE] ;
  ULONG lEDCount      = m_sEdgeDefs.GetSize() ;
  ULONG lOverlapCount = m_sOverlappingPEs.GetSize() ;
  ULONG lCoincPFCount = m_sCoincPFs.GetSize() ;
  ULONG lPVIdx        = 0 ;
  ULONG lPEIdx        = 0 ;
  ULONG lPFIdx        = 0 ;

  // this, EdgeDef cnt, tol
  if(lIdx != NULL) { smos_sprintf(sBuff, _T("\n    [%ld] "), *lIdx) ; smos_WriteBuffer(sBuff); }
  else             { smos_sprintf(sBuff, _T("%s"), _T("\n       ")) ; smos_WriteBuffer(sBuff); }
  smos_sprintf(sBuff, _T("SmProtoEdge:[0x%p], id:[%lu], Tol:[%5.8lf], EdgeDefCnt:[%lu], OverlappingPECnt:[%lu], CoincPFCnt:[%lu], StartPVId:[%lu], EndPVId:[%lu]"), 
                          this, m_pProtoTopoMgr->GetProtoEdgeId(this), m_dTol, lEDCount, lOverlapCount, lCoincPFCount,
                          m_pStartPV ? m_pProtoTopoMgr->GetProtoVertexId(m_pStartPV) : 99999, 
                          m_pEndPV   ? m_pProtoTopoMgr->GetProtoVertexId(m_pEndPV)   : 99999) ; 
  smos_WriteBuffer(sBuff); 
  
  // PE->OverlappingPE ids
  smos_WriteBuffer(_T(" \n  OverlappingPEIds:["));
  for(ii=0;ii<m_sOverlappingPEs.GetSize();ii++)
   { smos_sprintf(sBuff, _T("%lu"), m_pProtoTopoMgr->GetProtoEdgeId(m_sOverlappingPEs[ii])) ; smos_WriteBuffer(sBuff) ;
     if(ii<m_sOverlappingPEs.GetSize()-1) { smos_WriteBuffer(_T(", ")) ; }
     else                                 { smos_WriteBuffer(_T(" ")) ; }
   }
  smos_WriteBuffer(_T("]")) ;

  // PE->m_sCoincPFs ids
  smos_WriteBuffer(_T(" \n  CoincPFIds      :["));
  for(ii=0;ii<m_sCoincPFs.GetSize();ii++)
   { smos_sprintf(sBuff, _T("%lu"), m_pProtoTopoMgr->GetProtoFaceId(m_sCoincPFs[ii])) ; smos_WriteBuffer(sBuff) ;
     if(ii<m_sCoincPFs.GetSize()-1) { smos_WriteBuffer(_T(", ")) ; }
     else                           { smos_WriteBuffer(_T(" ")) ; }
   }
  smos_WriteBuffer(_T("]")) ;

  // when asked - complete dump
  if(iDumpLevel >= 10 )
    { 
      // edge->curve domain
      smos_sprintf(sBuff, _T(" Domain [ %18.16f, %18.16f ]\n"), m_pXSectInterval.GetMin(), m_pXSectInterval.GetMax()) ;
      smos_WriteBuffer(sBuff);
    
      // Print out ProtoManager indices instead of PV, PE, PF pointers: easier to track than the pointers.
      const SmTArray< SmProtoVertex* > & crAllPVs = GetProtoTopologyManager()->GetProtoVertices();
      const SmTArray< SmProtoEdge  * > & crAllPEs = GetProtoTopologyManager()->GetProtoEdges();
      const SmTArray< SmProtoFace  * > & crAllPFs = GetProtoTopologyManager()->GetProtoFaces();
    
      // start ProtoVertex
      SmPoint3d & rPos1 = m_pStartPV->GetPosition();
      crAllPVs.FindElement( m_pStartPV, lPVIdx) ;
      smos_sprintf(sBuff, _T(" Start ProtoVertex 0x%p  [%4lu ] : %18.16f, %18.16f, %18.16f\n"),
          m_pStartPV, lPVIdx, rPos1.x, rPos1.y, rPos1.z) ;
      smos_WriteBuffer(sBuff);
      if(iDumpLevel >= 15 ) { m_pStartPV->Dump(); }
    
      // end ProtoVertex
      SmPoint3d & rPos2 = m_pEndPV->GetPosition();
      crAllPVs.FindElement( m_pEndPV, lPVIdx) ;
      smos_sprintf(sBuff, _T(" End   ProtoVertex 0x%p  [%4lu ] : %18.16f, %18.16f, %18.16f\n"),
                          m_pEndPV, lPVIdx, rPos2.x, rPos2.y, rPos2.z) ;
      smos_WriteBuffer(sBuff);
      if(iDumpLevel >= 15 ) { m_pEndPV->Dump(); }

      // Overlapping PE indices
      smos_sprintf(sBuff, _T("\nOverlapping PEs:[%lu] - indices: "), lOverlapCount) ;
      smos_WriteBuffer(sBuff);
      for(ii=0;ii<lOverlapCount;ii++)
        { 
          SmProtoEdge *pOPE = m_sOverlappingPEs[ii] ; 
          crAllPEs.FindElement( pOPE, lPEIdx) ;
          smos_sprintf(sBuff, _T(" [%lu]"), lPEIdx) ;
          smos_WriteBuffer(sBuff);
        }

      // Coincident PF indices
      smos_sprintf(sBuff, _T("\nCoincident PFs:[%lu] - indices: "), lCoincPFCount) ;
      smos_WriteBuffer(sBuff);
      for(ii=0;ii<lCoincPFCount;ii++)
        { 
          SmProtoFace *pCPF = m_sCoincPFs[ii] ; 
          crAllPFs.FindElement( pCPF, lPFIdx) ;
          smos_sprintf(sBuff, _T(" [%lu]"), lPFIdx) ;
          smos_WriteBuffer(sBuff);
        }

      // Edge definitions
      smos_sprintf(sBuff, _T("\nEdgeDefinitions:  %3ld\n"), lEDCount) ;
      smos_WriteBuffer(sBuff);
      for(ii=0;ii<lEDCount;ii++)
        { m_sEdgeDefs[ii]->Dump( iDumpLevel, &ii) ; }
    } // end iDumpLevel >= 10 check

} // end SmProtoEdge::Dump

/*******************************************************************//**
PURPOSE: Dump this SmProtoFace

USAGE NOTES---- 
***********************************************************************/
void SmProtoFace::Dump(int iDumpLevel, ULONG * lIdx) const
{
  ULONG ii ;
  TCHAR sBuff[SM_TBLOCK_SIZE];

  if(lIdx != NULL) { smos_sprintf(sBuff, _T("\n    [%ld] "), *lIdx) ; smos_WriteBuffer(sBuff); }
  else             { smos_sprintf(sBuff, _T("%s"), _T("\n       ")) ; smos_WriteBuffer(sBuff); }
  smos_sprintf(sBuff, _T("SmProtoFace:[0x%p], id:[%lu], Brep1 Face:[0x%p], Brep2 Face:[0x%p]"), 
                      this, m_pProtoTopoMgr->GetProtoFaceId(this), m_pBrep1Face, m_pBrep2Face) ; 
  smos_WriteBuffer(sBuff);

  // PF->m_sCoincPEs ids
  smos_WriteBuffer(_T(" \n  CoincPEIds      :["));
  for(ii=0;ii<m_sCoincPEs.GetSize();ii++)
   { smos_sprintf(sBuff, _T("%lu"), m_pProtoTopoMgr->GetProtoEdgeId(m_sCoincPEs[ii])) ; smos_WriteBuffer(sBuff) ;
     if(ii<m_sCoincPEs.GetSize()-1) { smos_WriteBuffer(_T(", ")) ; }
     else                           { smos_WriteBuffer(_T(" ")) ; }
   }
  smos_WriteBuffer(_T("]")) ;

  // when asked - complete dump
  if(iDumpLevel >= 10 ) 
    {
      m_pBrep1Face->Dump();
      m_pBrep2Face->Dump();
    }

} // end of SmProtoFace::Dump

/*******************************************************************//**
PURPOSE: Draw this SmVertexDefinition

USAGE NOTES---- 
***********************************************************************/
void SmVertexDefinition::Draw()
{
  m_sPosition.Draw(); sm_GraphicsLoop();
  sm_GraphicsLoop();

} // end SmVertexDefinition::Draw

/*******************************************************************//**
PURPOSE: Draw this SmProtoVertex

USAGE NOTES---- 
***********************************************************************/
void SmProtoVertex::Draw()
{
  m_sPosition.Draw(); sm_GraphicsLoop();
  sm_GraphicsLoop();

} // end SmProtoVertex::Draw

/*******************************************************************//**
PURPOSE: Draw this SmEdgeDefinition

USAGE NOTES---- 
***********************************************************************/
void SmEdgeDefinition::Draw()
{
  double lLineWidth = smgfx_GetLineWidth();
  double lPointSize = smgfx_GetPointSize();

  // Make vertices bigger; same color.
  smgfx_SetPointSize( lLineWidth + 2) ;

  GetStartProtoVertex()->GetPosition().Draw();  sm_GraphicsLoop();
  sm_GraphicsLoop();

  GetEndProtoVertex()->GetPosition().Draw();  sm_GraphicsLoop();
  sm_GraphicsLoop();

  smgfx_SetPointSize( lPointSize) ;  // Restore this.

  m_pXSectCurve3d->Draw( &m_pXSectInterval, TRUE) ;
  // m_pXSectCurve3d->DrawParams( &m_pXSectInterval) ;

  sm_GraphicsLoop();

} // end SmEdgeDefinition::Draw

/*******************************************************************//**
PURPOSE: Draw this SmProtoEdge

USAGE NOTES---- 
***********************************************************************/
void SmProtoEdge::Draw()
{
  double lLineWidth = smgfx_GetLineWidth();
  double lPointSize = smgfx_GetPointSize();

  // Make vertices bigger; same color.
  smgfx_SetPointSize( lLineWidth + 2) ;

  m_pStartPV->GetPosition().Draw();  sm_GraphicsLoop();
  sm_GraphicsLoop();

  m_pEndPV->GetPosition().Draw();  sm_GraphicsLoop();
  sm_GraphicsLoop();

  smgfx_SetPointSize( lPointSize) ;  // Restore this.

  ULONG ii, lCount = m_sEdgeDefs.GetSize() ;
  for(ii=0;ii<lCount;ii++)
    {
      smgfx_SetLook( lLineWidth+2+ii, lLineWidth+ii) ; m_sEdgeDefs[ii]->Draw();  sm_GraphicsLoop();
    }
  sm_GraphicsLoop();

} // end SmProtoEdge::Draw

/*******************************************************************//**
PURPOSE: Draw this SmProtoFace

USAGE NOTES---- 
***********************************************************************/
void SmProtoFace::Draw()
{
  smgfx_SetColor( 0,  0,  0) ; if(m_pBrep1Face) m_pBrep1Face->DrawUV(3,3); sm_GraphicsLoop();
  smgfx_SetColor(.3, .3, .3) ; if(m_pBrep2Face) m_pBrep2Face->DrawUV(5,5); sm_GraphicsLoop();
  sm_GraphicsLoop();

  // Make Coinc Edges bigger; same color.
  double lLineWidth = smgfx_GetLineWidth();
  smgfx_SetLineWidth( lLineWidth + 4) ;
  smgfx_SetColor(1.0, .5, 0) ;  // orange

  ULONG ii, lNumEdges = m_sCoincPEs.GetSize();
  for(ii=0;ii<lNumEdges;ii++)
    { m_sCoincPEs[ii]->Draw(); sm_GraphicsLoop(); }
  sm_GraphicsLoop();

  smgfx_SetLineWidth( lLineWidth) ;  // Restore lineWidth
  smgfx_SetColor(0,0,0) ;  // black

} // end SmProtoFace::Draw

/*******************************************************************//**
PURPOSE:
NOTES:
***********************************************************************/
SmBoolean SmProtoTopologyManager::IsKindOf( SM_TYPE t ) const
{
  return ((SmProtoTopologyManager_TYPE == t) ? TRUE : SmTopology::IsKindOf( (t) ));
}

/*******************************************************************//**
PURPOSE: Dump this SmProtoTopologyManager: custom version

USAGE NOTES---- 
  iDumpLevel: -1 = no output
               0 = headers, ProtoVert, Edge, and Face counts
               5 = List of ProtoVerts, Edges, Faces. Repeat ProtoVert, Edge, and Face counts
              15 = Brep1, Brep2, and SmTopologyIntersector dumps
***********************************************************************/
void SmProtoTopologyManager::Dump(int iDumpLevel) const
{
  if(iDumpLevel < 0 ) { return; }

  TCHAR sBuff[SM_TBLOCK_SIZE];

  ULONG ii ;
  ULONG lNumPVs = m_sPVs.GetSize();
  ULONG lNumPEs = m_sPEs.GetSize();
  ULONG lNumPFs = m_sPFs.GetSize();

  // header
  smos_sprintf(sBuff, _T("\nBegin Dump SmProtoTopologyManager:[0x%p], Tol:[%5.8lf]"), this, m_dTol) ;
  smos_WriteBuffer(sBuff);

  // ProtoTopo counts
  smos_sprintf(sBuff, _T("\n  ProtoTopo Cnts: ProtoVertices, ProtoEdges, ProtoFaces:[%lu %lu %lu]"), lNumPVs, lNumPEs, lNumPFs) ;
  smos_WriteBuffer(sBuff);         

  // When asked - output more
  if(iDumpLevel >= 5)
    { 

      if(iDumpLevel > 15 )
        {
          m_pBrep1->Dump( _T("\n  Brep 1:")) ;
          m_pBrep2->Dump( _T("\n  Brep 2:")) ;
          m_pTI->Dump();
        }
      
      // ProtoVertex list
      smos_sprintf(sBuff, _T("\n  ProtoVert List: count:[%lu]"), lNumPVs) ; smos_WriteBuffer(sBuff) ;
      for(ii=0;ii<lNumPVs;ii++)
        { m_sPVs[ii]->Dump( iDumpLevel, &ii) ; }
      if(lNumPVs > 0) { smos_WriteBuffer(_T("\n")) ; }
      
      // ProtoEdge list
      smos_sprintf(sBuff, _T("\n  ProtoEdge List: count:[%lu]"), lNumPEs) ; smos_WriteBuffer(sBuff) ;
      for(ii=0;ii<lNumPEs;ii++)
        { m_sPEs[ii]->Dump( iDumpLevel, &ii) ; }
      if(lNumPEs > 0) { smos_WriteBuffer(_T("\n")) ; }
      
      // ProtoFace list
      smos_sprintf(sBuff, _T("\n  ProtoFace List: count:[%lu]"), lNumPFs) ; smos_WriteBuffer(sBuff) ;
      for(ii=0;ii<lNumPFs;ii++)
        { m_sPFs[ii]->Dump( iDumpLevel, &ii) ; }
      if(lNumPFs > 0) { smos_WriteBuffer(_T("\n")) ; }
      
      // ProtoTopo counts - again
      smos_sprintf(sBuff, _T("\n  Counts again  : ProtoVertices, ProtoEdges, ProtoFaces:[%lu %lu %lu]"), lNumPVs, lNumPEs, lNumPFs) ;
      smos_WriteBuffer(sBuff) ;

    } // end iDumpLevel >= 5 check
      
  // footer
  smos_sprintf(sBuff, _T("\nEnd Dump SmProtoTopologyManager:[0x%p]\n"), this) ;
  smos_WriteBuffer(sBuff) ;

} // end SmProtoTopologyManager::Dump

/*******************************************************************//**
PURPOSE: Dump this SmProtoTopologyManager: default version

USAGE NOTES---- 
***********************************************************************/
void SmProtoTopologyManager::Dump() const
{
  Dump( 0) ;

} // end SmProtoTopologyManager::Dump

/*******************************************************************//**
PURPOSE: Draw this SmProtoTopologyManager object, basic version.

USAGE NOTES---- 
***********************************************************************/
void SmProtoTopologyManager::Draw() const
{
  ULONG ii = 0 ;

#ifdef SM_DEBUG_CODE
  if(ii != 0)
    { smgfx_Erase() ; }
#endif // SM_DEBUG_CODE

  // Draw ProtoVertices incrementing color and size
  ULONG lLineWidth = 5 ;
  ULONG lPointSize = 7 ;
  for(ii=0;ii<m_sPVs.GetSize();ii++)
    {
      SmProtoVertex *pPV = m_sPVs[ii] ;
      smgfx_SetLook(lLineWidth, lPointSize+ii, ii > 0) ; pPV->GetPosition().Draw() ; sm_GraphicsLoop();
    }

  // Draw ProtoEdges incrementing color and size
  lLineWidth = 2 ;
  lPointSize = 5;
  for(ii=0;ii< m_sPEs.GetSize();ii++)
    {
      SmProtoEdge *pPE = m_sPEs[ii];
      smgfx_SetLook(lLineWidth+ii, lPointSize, ii > 0) ; pPE->Draw() ;  sm_GraphicsLoop();
    }

  // Draw ProtFaces last, black.
  lLineWidth = 1 ;
  lPointSize = 2 ;

  for(ii=0;ii<m_sPFs.GetSize();ii++)
    {
      SmProtoFace *pPF = m_sPFs[ii];
      smgfx_SetLook(lLineWidth+ii, lPointSize+ii, ii > 0) ; pPF->Draw() ; sm_GraphicsLoop();
    }

} // end SmProtoTopologyManager::Draw

/*******************************************************************//**
PURPOSE: Draw this SmProtoTopologyManager object, item by item, for debugging.

USAGE NOTES----  Red   = ProtoVertices positions
                 Cyan  = ProtoEdges
                 Black = ProtoFaces
***********************************************************************/
void SmProtoTopologyManager::DrawDebug
 (SmBoolean bDoDebugDraw,     // in : False = do basic Draw, default:[TRUE]
  ULONG     lFaceDrawSize)    // in : size to draw ProtoFaces, default:[1]  
                              //      LineWidth = lFaceDrawSize + 4 ;
                              //      PointSize = lFaceDrawSize + 6
  const
{
  // low work - no debugDraw request - just do basic SmProtoTopologyManager draw
  if(bDoDebugDraw == FALSE)
    { this->Draw(); return; }

  // locals
  ULONG ii ;
  SmBoolean bDrawOnePE = FALSE ; 
  ULONG lNumPVs = m_sPVs.GetSize() ;
  ULONG lNumPEs = m_sPEs.GetSize();
  ULONG lNumPFs = m_sPFs.GetSize();

  // set ProotVertex LineWidth, PointSize, color(red)
  ULONG lLineWidth = lFaceDrawSize + 4;
  ULONG lPointSize = lFaceDrawSize + 6;
  smgfx_SetLook(lLineWidth, lPointSize, 1,0,0) ;

  // Draw ProtoVertices->Positions
  for(ii=0;ii<lNumPVs;ii++)
    {
      SmProtoVertex *pPV = m_sPVs[ii];
      pPV->GetPosition().Draw();  sm_GraphicsLoop();
      sm_GraphicsLoop();
    }

  // Optional draw Breps - only useful when debugging line by line.
  if(FALSE)  // Might want to see Edges separately.
    {
      smgfx_Erase();
      smgfx_SetLook( 1,2, 0,0,1) ; m_pBrep1->Draw(TRUE); sm_GraphicsLoop();
      smgfx_SetLook( 1,2, 0,1,0) ; m_pBrep2->Draw(TRUE); sm_GraphicsLoop();
      sm_GraphicsLoop();
    }

  // set ProtoEdge LineWidth, PointSize, color(cyan)
  lLineWidth = lFaceDrawSize + 2;
  lPointSize = lFaceDrawSize + 4;
  smgfx_SetLook( lLineWidth, lPointSize, 1,0,1) ;

  // Draw ProtoEdges - give each PE its own color 
  smgfx_SetLook(lLineWidth, lPointSize, FALSE) ;  // FALSE = Init Color sequence (to red)
  for(ii=0;ii<lNumPEs;ii++)
    {
      SmProtoEdge *pPE = m_sPEs[ii];
      if(bDrawOnePE) smgfx_Erase() ;
      smgfx_SetLook(lLineWidth, lPointSize, TRUE) ; pPE->Draw();  sm_GraphicsLoop();  // TRUE = increment color as red - yellow - magenta - green - blue - cyan - red . . .
      sm_GraphicsLoop();
    }

  // for debug - opportunity to erase image - Might want to see Faces separately.
  if(FALSE )
    { smgfx_Erase() ; } 

  // set ProtoFace LineWidth, PointSize, color(black)
  lLineWidth = lFaceDrawSize;
  lPointSize = lFaceDrawSize + 2;
  smgfx_SetLook( lLineWidth, lPointSize, 0,0,0) ;

  // option to skip faces - might want to avoid clutter
  static SmBoolean bDrawFaces = TRUE ;
  if(bDrawFaces == FALSE)
    { lNumPFs = 0; }

  // Draw ProtoFaces - with UVLines and Coincident Edges highlighted
  for(ii=0;ii<lNumPFs;ii++)
    {
      SmProtoFace *pPF = m_sPFs[ii];
      pPF->Draw();  sm_GraphicsLoop();
      sm_GraphicsLoop();
    }

} // end SmProtoTopologyManager::DrawDebug
