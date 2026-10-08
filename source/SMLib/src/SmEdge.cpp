// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************//**
* FILE NAME --- SmEdge.cpp
* PURPOSE: Source file for SmEdge class methods.
**********************************************************************/

#include "StdAfx.h"

#include <SmEdge.h>

#ifndef __SMVERTEX_H__
#include <SmVertex.h>
#endif

#ifndef __SMBREP_H__
#include <SmBrep.h>
#endif

#ifndef __SMFACE_H__
#include <SmFace.h>
#endif

#ifndef __SMLOOP_H__
#include <SmLoop.h>
#endif

#ifndef __UNORDERED_SET__
#define __UNORDERED_SET__
#include <unordered_set>
#endif

#include <SmCurve.h>
#include <SmOffsetCurve.h>
#include <SmIntegrator.h>
#include <SmPolarBox.h>
#include <SmCrvOnSurf.h>
#include <SmGraphicsOutput.h>
// Remove Composites
// #include <SmCEdge.h>
#include <SmAssertArray.h>
#include <SmTrimmingTools.h>
#include <SmPseudoBox.h>

//    // For debugging only - a set of pointers that can be assigned and inspected through the watch window
//    #ifdef SM_DEBUG_CODE                  
//    
//    SmEdge * dbgEdge1 = NULL ;
//    SmEdge * dbgEdge2 = NULL ;
//    
//    #endif // SM_DEBUG_CODE

/*******************************************************************//**
PURPOSE: Destructor for the edge - it cleans up the curve

NOTES: 
***********************************************************************/
SmEdge::~SmEdge() 
{ 
  // locals
  SmBoolean bIsDoingBoolean = FALSE;
  SmContext *pContext = SM_CONST_CAST(SmContext*,GetContext());

  // when there is a context
  if ( pContext != NULL )
    {
      // remember it's DoingBoolean state
      bIsDoingBoolean = pContext->GetDoingBoolean();

      // set it's DoingBoolean state to TRUE - delays SurfaceCache construction
      pContext->SetDoingBoolean( TRUE );
    }

// Remove Composites
//  // when Edge has a curve and is a member of a composite edge
//  if (m_pCurve && m_pCurve->GetOwner() != this) 
//    {
//      // remove edge from Composite Edge lists
//      SmCEdge *pCEdge = (SmCEdge*)m_pCurve->GetOwner();
//      if (pCEdge && pCEdge->IsKindOf(SmCEdge_TYPE)) 
//        {
//          // gwc: RemoveEdge now calls Notify which deletes all pEdge->AnyEdgeuse->UVTrimCurves - is that a problem in the delete sequence?
//          pCEdge->RemoveEdge(this);  // note: pEdge->Curve set to NULL not deleted, UVTrimCurves are deleted.
//        }
//    }

  Notify(SM_NO_DESTRUCTION, this, NULL, NULL);

  // when Edge has a curve that is owned by this edge
  if (m_pCurve && m_pCurve->GetOwner() == this) 
    {
      // delete the curve - leaves the UVTrimCurves unchanged
      delete m_pCurve; m_pCurve = NULL ; 
    }

  // restore the context DoingBoolean state
  if ( pContext != NULL )
    { pContext->SetDoingBoolean( bIsDoingBoolean ); }

} // end SmEdge::~SmEdge destructor

/*************************************************************
PURPOSE:

NOTES:
**************************************************************/
class SmODEEdgeLengthIFEO : public SmODEIntegFuncEvalObj
{
private:
    double            m_dCrossSectionRadius;
    const SmPoint3d   m_vOriginOfComputation;
    const SmCurve   & m_crCurve;
public:
    SmODEEdgeLengthIFEO
      (const SmCurve & crCurve,
       const SmPoint3d & crOriginOfComputation, 
       double dCrossSectionRadius)
     : m_dCrossSectionRadius(dCrossSectionRadius), 
       m_vOriginOfComputation(crOriginOfComputation),
       m_crCurve(crCurve) 
       {}

    virtual SmStatus Evaluate(double dT,                             // in :
                              SmTArray<double> & rYValues,           // NotUsed: out:
                              SmTArray<double> & rDyDxValues) const; // out:

} ; // end class SmODEEdgeLengthIFEO

/*************************************************************
PURPOSE:

NOTES:
**************************************************************/
SmStatus SmODEEdgeLengthIFEO::Evaluate
  (double dT,                       // in :
   SmTArray<double> & rYValues,     // NotUsed: out:
   SmTArray<double> & rDyDxValues)  // out:
  const
{
  SM_REF1(rYValues) ;
    for (ULONG j=0; j<rDyDxValues.GetSize(); j++) {
        rDyDxValues[j] = 0.0;
    }

    SmPoint3d sPV[2];
    SER(m_crCurve.Evaluate(dT,1,TRUE,sPV));
    sPV[0] = sPV[0] - m_vOriginOfComputation;

    SmPoint3d sIPnt = sPV[0];  // Point to integrate
    rDyDxValues[1] = 0.0;
    if (sPV[1].Length() > 0) { // We integrate over the length of the curve
        rDyDxValues[1] = sPV[1].Length();
    }

    double dCircumference = 2.0 * SM_PI * m_dCrossSectionRadius;
    double dAreaDelta = dCircumference * sPV[1].Length();

    rDyDxValues[2] = dAreaDelta;

    // Note that to get this computation more precise we should actually
    // integrate around the circumference not just at the center point.
        
    // Compute area static moments 
    rDyDxValues[4] = sIPnt.x * dAreaDelta;
    rDyDxValues[5] = sIPnt.y * dAreaDelta;
    rDyDxValues[6] = sIPnt.z * dAreaDelta;
            
    // Compute area Ixx, Iyy, Izz
    rDyDxValues[7] = (sIPnt.x*sIPnt.x) * dAreaDelta;
    rDyDxValues[8] = (sIPnt.y*sIPnt.y) * dAreaDelta;
    rDyDxValues[9] = (sIPnt.z*sIPnt.z) * dAreaDelta;
            
    // Compute area Iyz, Izx, Ixy
    rDyDxValues[10] = (sIPnt.y*sIPnt.z) * dAreaDelta;
    rDyDxValues[11] = (sIPnt.z*sIPnt.x) * dAreaDelta;
    rDyDxValues[12] = (sIPnt.x*sIPnt.y) * dAreaDelta;
        
    // Compute area second moments Ixx, Iyy, Izz about coordinate axes
    rDyDxValues[13] = (sIPnt.y*sIPnt.y + sIPnt.z*sIPnt.z) * dAreaDelta;
    rDyDxValues[14] = (sIPnt.x*sIPnt.x + sIPnt.z*sIPnt.z) * dAreaDelta;
    rDyDxValues[15] = (sIPnt.x*sIPnt.x + sIPnt.y*sIPnt.y) * dAreaDelta;
        
    // Do volumetric property
    double dArea = SM_PI * m_dCrossSectionRadius * m_dCrossSectionRadius;
    double dVolumeDelta = dArea * sPV[1].Length();

    rDyDxValues[3] = dVolumeDelta;
    
    // Compute volume static moments
    rDyDxValues[16] = sIPnt.x * dVolumeDelta;
    rDyDxValues[17] = sIPnt.y * dVolumeDelta;
    rDyDxValues[18] = sIPnt.z * dVolumeDelta;
    
    // Compute Volume Moments Ixx, Iyy, Izz
    rDyDxValues[19] = (sIPnt.x*sIPnt.x) * dVolumeDelta;
    rDyDxValues[20] = (sIPnt.y*sIPnt.y) * dVolumeDelta;
    rDyDxValues[21] = (sIPnt.z*sIPnt.z) * dVolumeDelta;
    
    // Compute Volume Moments Iyz, Izx, Ixy
    rDyDxValues[22] = (sIPnt.y*sIPnt.z) * dVolumeDelta;
    rDyDxValues[23] = (sIPnt.z*sIPnt.x) * dVolumeDelta;
    rDyDxValues[24] = (sIPnt.x*sIPnt.y) * dVolumeDelta;
    
    // Compute Volume Second Moments Ixx, Iyy, Izz about Coordinate Axes
    rDyDxValues[25] = (sIPnt.y*sIPnt.y + sIPnt.z*sIPnt.z) * dVolumeDelta;
    rDyDxValues[26] = (sIPnt.x*sIPnt.x + sIPnt.z*sIPnt.z) * dVolumeDelta;
    rDyDxValues[27] = (sIPnt.x*sIPnt.x + sIPnt.y*sIPnt.y) * dVolumeDelta;
    
    return SM_SUCCESS;

} // end SmODEEdgeLengthIFEO::Evaluate

/*******************************************************************//**
PURPOSE: Bounding box based on fine tessellation sample points.
         SmBSplineCurve uses controlPolygon of containing spans.

NOTES: Bounding Boxes are based on sampling of the Edge->Curve's tessellation.
       For BSplineCurves the Bounding box is the bounding box of the control points.

       For a tight Axis Aligned BBOx use CalculateTightBoundingBox which
       will be called by this function when BTight == TRUE.

  bTight == TRUE is very expensive, it requires 6 SM_SR_ALL global curve calls per edge.

  BoundingBoxes are intended to be a cheap and dirty method to help other algorithms to
  run faster by culling geometry that is far from an area of interest.
  That goal is achieved with the CalculateBoundingBox() algorithm which
  often returns a BoundingBox larger than necessary. Typically, calculating a 
  tight bounding box is more expensive than the algorithm being helped.
  
  So, either attempt to write algorithms that work with the often oversized
  results from CalculateBoundingBox, i.e. don't write algorithms that depend on
  the true minimal size of a bounding box.  Or only call CalculateTightBoundingBox
  once for each geometry object and save and reuse the reuslts many times.
***********************************************************************/
SmStatus SmEdge::CalculateBoundingBox
 (SmExtent3d  * pNormalBox, // out: axis aligned bounding box, NULL to ignore, default:[NULL] 
  SmPseudoBox * pPseudoBox, // out: non-Axis Aligned boundingBox, NULL to ignore, default:[NULL]
  SmPolarBox  * pPolarBox,  // out: Curve tangent vector field bounding box, NULL to ignore, default:[NULL]
  SmBoolean     bTight)     // in : TRUE = compute minimal box (expensive)
                            //      FALSE= compute any box larger than this Face (cheaper)
                            //      default:[FALSE]

 const
{
  if ( pNormalBox != NULL ) { pNormalBox->Init(); }
  if ( pPseudoBox != NULL ) { pPseudoBox->Init(); } // init intervals - leave basis vectors alone
  if ( pPolarBox  != NULL ) { pPolarBox->ReSet(); }

  // no work - no curve
  if ( m_pCurve == NULL ) { return SM_SUCCESS; }

  // when asked for a tight Axis Aligned Box
  if(bTight)
    {
      SmStatus sStatus1 = SM_SUCCESS ;
      SmStatus sStatus2 = SM_SUCCESS ;

      if(pNormalBox)
        {
          sStatus1 = CalculateTightBoundingBox(pNormalBox) ;
        }

      if(pPseudoBox || pPolarBox)
        {
          sStatus2 = m_pCurve->CalculateBoundingBox( m_vInterval,
                                                     NULL, 
                                                     pPseudoBox, 
                                                     pPolarBox ) ;
        }

      // all done
      return( (   sStatus1 == SM_SUCCESS
               && sStatus2 == SM_SUCCESS) ? SM_SUCCESS : SM_ERR ) ;
    }
  else // bTight == FALSE
    {
      // get all the bounding boxes from the curve
      return m_pCurve->CalculateBoundingBox( m_vInterval,
                                             pNormalBox, 
                                             pPseudoBox, 
                                             pPolarBox ) ;
    }

} // end SmEdge::CalculateBoundingBox

/*******************************************************************//**
PURPOSE: Calculate the precise bounding box.

NOTES:
  CalculateTightBoundingBox() is very expensive, it requires 
  6 SM_SR_ALL global curve calls per edge.

  BoundingBoxes are intended to be a cheap and dirty method to help other algorithms to
  run faster by culling geometry that is far from an area of interest.
  That goal is achieved with the CalculateBoundingBox() algorithm which
  often returns a BoundingBox larger than necessary. Typically, calculating a 
  tight bounding box is more expensive than the algorithm being helped.
  
  So, either attempt to write algorithms that work with the often oversized
  results from CalculateBoundingBox, i.e. don't write algorithms that depend on
  the true minimal size of a bounding box.  Or only call CalculateTightBoundingBox
  once for each geometry object and save and reuse the reuslts many times.
***********************************************************************/
SmStatus SmEdge::CalculateTightBoundingBox( SmExtent3d * pBBox ) const
{
  NER( pBBox );
  ULONG i;

  // Method: get the BBox of the vertices.
  // Then get the loose BBox of the curve.
  // If it's within the BBox so far (from the vertices),
  // then we're done,
  // otherwise calculate the tight BBox of the curve and union it in.

  SmExtent3d sEdgeBBox;
  SmTArray< SmVertex* > sVerts(2);
  GetVertices( sVerts );
  for ( i = 0; i < sVerts.GetSize(); i++ )
    {
      sEdgeBBox.AddPoint3d( sVerts[i]->GetPoint() );
    }

  // Now our curve.
  if ( m_pCurve == NULL )
    { return SM_SUCCESS; }

  SmExtent3d sCurveBBox;
  SER( m_pCurve->CalculateBoundingBox( m_vInterval, &sCurveBBox ) );

  double dTol = SM_EFF_ZERO * ( 1.0 + sCurveBBox.GetMaxDimension() );

  if ( ! sCurveBBox.IsContainedBy( sEdgeBBox, dTol ) )
    {
      // Do the tight BBox computation.
      SER( m_pCurve->CalculateTightBoundingBox( m_vInterval, &sCurveBBox ) );

      sEdgeBBox.Union( sCurveBBox, sEdgeBBox );
    }

  *pBBox = sEdgeBBox;

  return SM_SUCCESS;

} // end SmEdge::CalculateTightBoundingBox

/*******************************************************************//**
PURPOSE: Compute the precise properties of an edge.  

NOTES: Note that the
   edge may be treated like a tube if the cross section area is not
   zero and it will compute volume and area properties for it.
***********************************************************************/
SmStatus SmEdge::ComputePreciseProperties
  (double                 dDesiredAccuracy,
   const SmPoint3d      & crOriginOfComputation,
   double                 dCrossSectionRadius,
   double               & rdEdgeLength,
   double               & rdDeltaArea,
   double               & rdDeltaVolume,
   SmTArray<SmVector3d> & rDeltaMoments) 
  const
{
  rdEdgeLength  = 0.0;
  rdDeltaArea   = 0.0;
  rdDeltaVolume = 0.0;
    
  rDeltaMoments.SetSize(8);
  for (ULONG ii=0; ii<rDeltaMoments.GetSize(); ii++) 
    {
      rDeltaMoments[ii].Set(0,0,0);
    }
    
  SmODEEdgeLengthIFEO sEval(*GetCurve(),crOriginOfComputation,dCrossSectionRadius);
  SmODEIntegrator sIntegrator(sEval);
    
  double dApproxLeng = GetCurve()->ApproximateLength(GetInterval(),10);
    
  SmTArray<double> dY(27,NULL,27);
  double dStep = GetInterval().GetMax() - GetInterval().GetMin();
    
  SmTArray<double> yscale(27,NULL,27);
  for (ULONG iii=0; iii<yscale.GetSize(); iii++) { yscale[iii] = SM_BIG_DOUBLE; }
  yscale[0] = dApproxLeng/10.0;
    
  ULONG lNumOK, lNumBad;
  SER(sIntegrator.RungeKuttaODEIntegrate(dY,GetInterval().GetMin(),
                                         GetInterval().GetMax(), dDesiredAccuracy, dStep/2.0, 0.0, lNumOK, lNumBad,
                                         &yscale));
  rdEdgeLength = dY[0];
  rdDeltaArea = dY[1];
  rdDeltaVolume = dY[2];
    
  ULONG kk=0; // Area Moments
  rDeltaMoments[0].x += dY[3]; 
  rDeltaMoments[0].y += dY[4]; 
  rDeltaMoments[0].z += dY[5]; 
  rDeltaMoments[1].x += dY[6]; 
  rDeltaMoments[1].y += dY[7]; 
  rDeltaMoments[1].z += dY[8]; 
  rDeltaMoments[2].x += dY[9]; 
  rDeltaMoments[2].y += dY[10]; 
  rDeltaMoments[2].z += dY[11]; 
  rDeltaMoments[3].x += dY[12]; 
  rDeltaMoments[3].y += dY[13]; 
  rDeltaMoments[3].z += dY[14]; 
    
  kk++;  // Volume moments
  rDeltaMoments[4].x += dY[15]; 
  rDeltaMoments[4].y += dY[16]; 
  rDeltaMoments[4].z += dY[17]; 
  rDeltaMoments[5].x += dY[18]; 
  rDeltaMoments[5].y += dY[19]; 
  rDeltaMoments[5].z += dY[20]; 
  rDeltaMoments[6].x += dY[21]; 
  rDeltaMoments[6].y += dY[22]; 
  rDeltaMoments[6].z += dY[23];
  rDeltaMoments[7].x += dY[24]; 
  rDeltaMoments[7].y += dY[25]; 
  rDeltaMoments[7].z += dY[26];
    
  return SM_SUCCESS;

} // end SmEdge::ComputePreciseProperties

/*******************************************************************//**
PURPOSE: Find the closest point on an edge to a point.

NOTES: Uses SmCurve::GlobalPointSolve(SM_SO_MINIMIZE) ;
***********************************************************************/
SmStatus SmEdge::ClosestPoint
  (const SmPoint3d & crPoint,     // in : Point to test
   SmBoolean       & rbSuccess,   // out: TRUE = found a point, false= didn't
   double          & rdParameter, // out: curve param of closest point
   double          & rdDistance)  // out: dist to closest point
{
  // init output
  rbSuccess = FALSE;

  // locals
  SmSolution      sData[16];
  SmSolutionArray sSolutions(16,sData);

  // global point solve
  SER(GetCurve()->GlobalPointSolve(GetInterval(),
                                   SM_SO_MINIMIZE,
                                   crPoint,
                                   GetTolerance(),
                                   NULL,
                                   NULL,
                                   SM_SR_ALL,
                                   sSolutions));
  // no solutions
  if (sSolutions.GetSize() < 1) 
    { return SM_SUCCESS; } // Nothing found

  // found a closest point
  SmSolution & rSol = sSolutions[0];

  // set output
  rbSuccess = TRUE;
  rdParameter = rSol.m_vStart[0];
  rdDistance = rSol.m_vStart.m_dSolutionValue;

  // all done
  return SM_SUCCESS;

} // end SmEdge::ClosestPoint

/*******************************************************************//**
PURPOSE: Calculate tangent vector at the end of an edge.

NOTES: 
***********************************************************************/
SmStatus SmEdge::GetEndTangent
  (SmBoolean bAtStart, 
   SmVector3d & rTangent) 
  const
{
    SmCurve * p3DCurve = GetCurve();
    SmVector3d sPtVec[2];
    SmExtent1d sIvl = GetInterval();
    double dParam;
    if (bAtStart)  dParam = sIvl.GetMin();
    else  dParam = sIvl.GetMax();
    SER(p3DCurve->Evaluate(dParam,1,TRUE,sPtVec));
    rTangent = sPtVec[1];

    return SM_SUCCESS;

} // end SmEdge::GetEndTangent

/*******************************************************************//**
PURPOSE: Return TRUE if the Edge is manifold and its two Faces
    are G2 to each other along the Edge.

NOTES: 
***********************************************************************/
SmContinuityType SmEdge::GetContinuity
  (double) 
 const
{
  // must be manifold
  if (!this->IsManifold() )
    { return SM_CT_UNDEFINED; }

  // get sector continuity for primary edgeuse
  SmEdgeuse       * pPrimEU     = GetPrimaryEdgeuse();
  SmContinuityType  eContinuity = pPrimEU->GetSectorContinuity() ;

  // return boolean
  return(eContinuity) ; 

} // end SmEdge::GetContinuity

/*******************************************************************//**
PURPOSE: Get/create the trim curve on the edge which is on the face. 

NOTES:  If one
    does not exist then create one and attach it to an edgeuse of the edge.
***********************************************************************/
SmBSplineCurve * SmEdge::GetUVTrimCurveOfSurface
  (const SmSurface * pSurface)
{
    SmBSplineCurve *pRet = NULL;
    SmStatus sRtn ;

    SmEdgeuse *pData[32];
    SmTArray<SmEdgeuse*> sEdgeuses(32,pData);

    // for every edgeuse
    GetEdgeuses(sEdgeuses);
    for (ULONG i=0; i<sEdgeuses.GetSize(); i++) 
      {
        SmEdgeuse *pEU = sEdgeuses[i];
        if (pSurface == pEU->GetFace()->GetSurface()) 
          {
            sRtn = pEU->GetOrCreateUVTrimCurve(pRet);
            SM_ASSERT(pRet != NULL) ; 
            SE(sRtn) ;
            return pRet;

          } // end Edgeuse->Surface == TargetSurface check
      } // end iter every edgeuse

    return pRet;

} // end SmEdge::GetUVTrimCurveOfSurface

/*******************************************************************//**
PURPOSE: Is the edge closed?

NOTES:  TRUE when GetVertex() == GetOtherVertex()
***********************************************************************/
SmBoolean SmEdge::IsClosed() const
{
  return(   GetPrimaryEdgeuse()->GetVertexuse()->GetVertex()
         == GetPrimaryEdgeuse()->GetMate()->GetVertexuse()->GetVertex()) ;

} // end SmEdge::IsClosed

/*******************************************************************//**
PURPOSE: Is the edge a wire edge (no faces)?

NOTES: TRUE when Edge->PrimaryEdgeuse()->EdgeuseType == SmShell_Type
***********************************************************************/
SmBoolean SmEdge::IsWire() const
{
  SmEdgeuse* pEU = GetPrimaryEdgeuse();
  return( pEU && pEU->m_tEdgeuseType == SmShell_TYPE ) ;

} // end SmEdge::IsWire

/*******************************************************************//**
PURPOSE: Create a trimmed nurbs curve corresponding to this edge.

NOTES: 
  This creates a 3D BSplineCurve copy of this edge's curve
  whose natural interval has been trimmed to be equal to the interval 
  of this edge.
***********************************************************************/
SmBSplineCurve * SmEdge::CreateTrimmedNURBSCurve(const SmContext & crContext) const    // in : context for new object construction
{
  SmBSplineCurve* pCurveCopy3D = SM_CAST_PTR(SmBSplineCurve, GetCurve());
  NERN(pCurveCopy3D);

  // make temporary copy of Curve
  pCurveCopy3D = new (crContext) SmBSplineCurve(*pCurveCopy3D);

  //attempt to address CrvOnSurf (suspect)
  //// make sure ThisEdge->Curve is a BSplineCurve
  //SmBSplineCurve* pCurveCopy3D;

  //// ApproximateCurve locals
  //SmExtent1d sCurveIvl = GetCurve()->GetNaturalInterval();
  //double dMaxGap3d;
  //SmTArray<double> sBreaks;
  //sBreaks.Add(sCurveIvl.GetMin());
  //sBreaks.Add(sCurveIvl.GetMax());
  //SmTol::GetApproxTol3d(this);

  //// Approximate the curve (copy BSplines)
  //GetCurve()->ApproximateCurve(crContext, sBreaks, SmTol::GetApproxTol3d(this), dMaxGap3d, pCurveCopy3D,
  //                             FALSE, // in : bOptCreateAnalytics
  //                             FALSE, // in : bOptMatchParameterization
  //                             TRUE); // in : bJustCopyBSplines

  //// Clean the curve if necessary
  SmObjDelete sClean3D(pCurveCopy3D); // temp only

  // Trim the CurveCopy to the EdgeInterval
  SmExtent1d sIvl = this->GetInterval();
  if (pCurveCopy3D->Trim(sIvl) != SM_SUCCESS)  // may snap sIvl by tol to existing knots
    {
      SE(SM_ERR);
      return NULL;
    }

  // preserve and return the CurveCopy
  sClean3D.Clear();
  return pCurveCopy3D;

} // end SmEdge::CreateTrimmedNURBSCurve

/*******************************************************************//**
PURPOSE: Get an edgeuse of the edge which corresponds to the face.
    
NOTES: Return null if none exists.
    There may be two Edgeuses for the face, one for each Faceuse;
    this returns the edgeuse with the SM_OT_SAME orientation if there is one
    else returns whatever edgeuse it finds.
***********************************************************************/
SmEdgeuse * SmEdge::GetEdgeuseOfFace
  ( const SmFace *pFace )    // in : target face
 const
{
  // init return
  SmEdgeuse *pRet = NULL;

  // no work - no face
  if(pFace == NULL)
    { return pRet; }

#ifdef SM_DEBUG_CODE
  SmBoolean bDebugMe=FALSE;
  if ( bDebugMe )
  {
      if ( FALSE )
        { smgfx_Erase(); }

      smgfx_SetLook( 1,2, 0,0,0 ); pFace->Draw(); sm_GraphicsLoop();
      sm_GraphicsLoop();
  }
#endif

  // locals
// Remove Composites
//  ULONG      lIndex;
  SmEdgeuse *pData[32];
  SmTArray<SmEdgeuse*> sEdgeuses(32,pData);
// Remove Composites
//   SmBoolean  bIsCFace = pFace->IsKindOf( SmCFace_TYPE );

  // for every edgeuse - check for a face == pFace match
  // for a CFace, we check against all child faces
  GetEdgeuses(sEdgeuses);

// Remove Composites
//  if ( bIsCFace ) 
//    {
//      SmFace* pFaceData[8];
//      SmTArray<SmFace*> sFaces( 8, pFaceData );
//      SmCFace *pCFace = SM_CAST_PTR( SmCFace, pFace );
//      pCFace->GetFaces( sFaces );
//
//      for ( ULONG i = 0; i < sEdgeuses.GetSize(); i++ )
//        {
//          SmEdgeuse *pEU = sEdgeuses[i];
//
//#ifdef SM_DEBUG_CODE
//          if ( bDebugMe )
//            {
//              smgfx_SetLook( 3, 5, TRUE ); pEU->Draw(); sm_GraphicsLoop();
//              sm_GraphicsLoop();
//            }
//#endif
//          if ( sFaces.FindElement( pEU->GetFace(), lIndex ) )
//            {
//              // remember any found edgeuses
//              pRet = pEU;
//
//              // use the 1st same oriented edgeuse found
//              if ( pEU->GetOrientation() == SM_OT_SAME )
//                { break; }
//            }
//        } // end iter every edgeuse
//    } // end CFace branch
//  else // Not a composite face branch
    {
      for ( ULONG i = 0; i < sEdgeuses.GetSize(); i++ )
        {
          SmEdgeuse *pEU = sEdgeuses[i];

#ifdef SM_DEBUG_CODE
          if ( bDebugMe )
            {
              smgfx_SetLook( 3, 5, TRUE ); pEU->Draw(); sm_GraphicsLoop();
              sm_GraphicsLoop();
            }
#endif
          if ( pFace == pEU->GetFace() )
            {
              // remember any found edgeuses
              pRet = pEU;

              // use the 1st same oriented edgeuse found
              if ( pEU->GetOrientation() == SM_OT_SAME )
              { break; }
            }
        } // end iter every edgeuse
    } // end not a composite face branch

  // all done
  return pRet;

} // end SmEdge::GetEdgeuseOfFace

/*******************************************************************//**
PURPOSE: Get Loop of the edge which is connected to the TgtFace.
    
NOTES: Return null if none exists.
***********************************************************************/
SmLoop * SmEdge::GetLoopOfFace
  ( const SmFace *pFace )    // in : target face
 const
{
  // no work - no face
  if(pFace == NULL)
    { return NULL ; }

  // locals
  SmEdgeuse *pEdgeuse = GetEdgeuseOfFace(pFace) ;

  // no work - no Edgeuse on Face
  if(pEdgeuse == NULL)
    { return NULL ; }

#ifdef SM_DEBUG_CODE
  SmBoolean bDebugMe=FALSE;
  if ( bDebugMe )
    {
      SmBrep * pBrep = GetBrep() ; 
      SmEdge * pEdge = pEdgeuse->GetEdge() ; 
      SM_ASSERT(pEdge!=NULL) ;

      if ( FALSE )
        { smgfx_Erase(); }

      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ; 
      smgfx_SetLook(2,3, 0,1,1) ; pFace->Draw(); sm_GraphicsLoop() ;
      smgfx_SetLook(4,5, 1,0,0) ; if(pEdgeuse) pEdgeuse->Draw() ; sm_GraphicsLoop() ;
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

  // get output
  SmLoop * pRet = (pEdgeuse && pEdgeuse->GetLoopuse()) ? pEdgeuse->GetLoopuse()->GetLoop() : NULL ; 

  // all done
  return pRet;

} // end SmEdge::GetLoopOfFace

/*******************************************************************//**
PURPOSE: Get Edge->edgeuse that connects Edge to Face through Face->UpwardFaceuse
    
NOTES:   Return null if none exists.
***********************************************************************/
SmEdgeuse * SmEdge::GetUpwardEdgeuseOfFace
  ( const SmFace *pFace )    // in : target face
 const
{
  // init return
  SmEdgeuse *pRet = NULL;

  // no work - no face
  if(pFace == NULL)
    { return pRet; }

  // locals
  ULONG ii ;
  SmFaceuse *pUpwardFaceuse = pFace->GetUpwardFaceuse() ;
  SM_PTR_ARRAY(sEdgeuses, SmEdgeuse, 32) ;  // SmTArray<SmEdgeuse*> sEdgeuses
  GetEdgeuses(sEdgeuses);

  // for every edgeuse - check for a face == pFace && faceuse == pUpwardFaceuse match
  for(ii=0;ii<sEdgeuses.GetSize();ii++)
    {
      SmEdgeuse *pEdgeuse = sEdgeuses[ii];

      if(   pFace          == pEdgeuse->GetFace()
         && pUpwardFaceuse == pEdgeuse->GetFaceuse())
        {
          pRet = pEdgeuse ; 
          break;
        }
    } // end iter every edgeuse

  // all done
  return pRet;

} // end SmEdge::GetUpwardEdgeuseOfFace


/*******************************************************************//**
PURPOSE: Find a string of edges where the ends of the edges are 
     tangent at their shared vertices and where all other surrounding
     edges are edges between tangent surfaces.  

NOTES: For example the Union
     of a cylinder with radius 1 with a 2x2x2 box such the center of the
     cylinder faces were exactly in the middle of two of the edge of the
     box would produce a pair of {Line, 180 degree arc, Line} tangent
     strings.  
     
     Used to grab tangent edges for filleting, sweeping or
     other such operations.  
***********************************************************************/
SmStatus SmEdge::FindTangentEdgeString
  (double dAngleTolDeg,
   SmTArray<SmEdge*>      & rEdgeString,  // String of 
                                          // connected edges ordered relative to the curve orientation
                                          // of the first edges's curve.  Edges which connect to the
                                          // this->GetInterval()->GetMin() end of the edge will be 
                                          // before this in the list.  Those which connect near the
                                          // high parameter end of the edge's curve will be after this
                                          // edge in the list.  
   SmTArray<SmOrientType> & rEdgeOrients) // The orientation
                                          // of this edge will be SM_OT_SAME.  The other edges orientations
                                          // will be how their curve aligns in the string relative to the
                                          // edge.  The result should be that we can take the curves from
                                          // the edges and produce a composite directly or with little work.   
{
    rEdgeString.ReSet();
    rEdgeOrients.ReSet();

    rEdgeString.Add(this);
    rEdgeOrients.Add(SM_OT_SAME);
    if (this->IsClosed()) {
        return SM_SUCCESS;
    }

    SM_PTR_ARRAY(sEdges,SmEdge,16);

    // First walk start of curve.
    SmBoolean bDone = FALSE;
    while (!bDone) {
        bDone = TRUE;
        SmEdge *pE = rEdgeString[0];
        SmVertex *pV = pE->GetStartVertex();
        if (rEdgeOrients[0] == SM_OT_OPPOSITE) {
            pV = pE->GetOtherVertex(pV);
        }
        pV->GetEdges(sEdges);
        SmEdge *pTanEdge = NULL;
        SmOrientType sTanOrient = SM_OT_SAME;
        for (ULONG i=0; i<sEdges.GetSize(); i++) {
            SmEdge *pTestE = sEdges[i];
            ULONG lFoundIndex;
            if (rEdgeString.FindElement(pTestE,lFoundIndex)) continue;
            if (pTestE->IsClosed()) break;
            if (pV->AreEdgesTangent(pE,pTestE,dAngleTolDeg)) {
                pTanEdge = pTestE;
                if (pTestE->GetStartVertex() == pV) {
                    sTanOrient = SM_OT_OPPOSITE;
                }
                continue;
            }
            // If made it to here it must be a tangent sector edge if we are 
            // good.  Otherwise we need to stop
            if (!pTestE->GetPrimaryEdgeuse()->IsTangentSector(dAngleTolDeg)) {
                pTanEdge = NULL;
                break;
            }
        }
        if (pTanEdge) {
            rEdgeString.InsertAt(0,pTanEdge,1);
            rEdgeOrients.InsertAt(0,sTanOrient,1);
            bDone = FALSE;
        }
    }

    // First walk end of curve.
    bDone = FALSE;
    while (!bDone) {
        bDone = TRUE;
        SmEdge *pE = rEdgeString.GetLast();
        SmVertex *pV = pE->GetStartVertex();
        if (rEdgeOrients.GetLast() == SM_OT_SAME) {
            pV = pE->GetOtherVertex(pV);
        }
        pV->GetEdges(sEdges);
        SmEdge *pTanEdge = NULL;
        SmOrientType sTanOrient = SM_OT_OPPOSITE;
        for (ULONG i=0; i<sEdges.GetSize(); i++) {
            SmEdge *pTestE = sEdges[i];
            ULONG lFoundIndex;
            if (rEdgeString.FindElement(pTestE,lFoundIndex)) continue;
            if (pTestE->IsClosed()) break;
            if (pV->AreEdgesTangent(pE,pTestE,dAngleTolDeg)) {
                pTanEdge = pTestE;
                if (pTestE->GetStartVertex() == pV) {
                    sTanOrient = SM_OT_SAME;
                }
                continue;
            }
            // If made it to here it must be a tangent sector edge if we are 
            // good.  Otherwise we need to stop
            if (!pTestE->GetPrimaryEdgeuse()->IsTangentSector(dAngleTolDeg)) {
                pTanEdge = NULL;
                break;
            }
        }
        if (pTanEdge) {
            rEdgeString.Add(pTanEdge);
            rEdgeOrients.Add(sTanOrient);
            bDone = FALSE;
        }
    }

    return SM_SUCCESS;

} // end SmEdge::FindTangentEdgeString

/*******************************************************************//**
PURPOSE: Classifies a face contained by a target Edgeuse
    relative to a radial sector bounded by one of the edge's
    edgeuse->Face and its edgeuse->RadialPartner->Face in the neighborhood
    of one given edge point.  
    
    The Face will be classified as in, out, or coincident to one of the 
    boundary faces. 
    
    A target face that is tangent to, but not coincident with, one of 
    the radial sector boundary faces will be classified in or 
    out depending on the shapes of the boundary and target surfaces
    as they leave the edgePoint being studied.  

    For cases where the face is bounded by this edge's curve but not
    connected to this edge.
        
NOTES:
    Returns 0 = target Edgeuse is outside of the region,
        and 1 = target Edgeuse is inside the region.

      When the target Edgeuse is classified as coincident,
      the coincident boundary is specified in the return value as oneof
            2 = the edgeuse->Faceuse, 
            3 = the edgeuse->Mate->Faceuse,
            4 = the edgeuse->RadialPartner->Faceuse, 
            5 = the edgeuse->RadialPartner->Mate->Faceuse.

      When coincident faces are found already on this edge -- a zero sector --
      and the given face is in their sector
            6 = the edgeuse->Faceuse points in the right direction
            7 = the edgeuse->Mate (or Radial) Faceuse points in the right direction

      Cases 2/3, 4/5, and 6/7 differ by surface normals, being on different
      sides of the same face.

METHOD --- 
    Classifies the Edgeuse by examining the distances between the positions
    and the angles amongst the binormal vectors found at a sequence of 
    stepOffPoint evaluations made on the target edgeuse->face and 
    the radial sector boundary faces.
 
    A sequence of increasing stepOffDistances are examined to distinguish
    between a face that is merely tangent to a boundary face and one
    that is coincident to that boundary.

    The primary classification loop is:

  while(not classified)
   {
     // eval binormals of 3 surfaces at the next of
     //   a sequence of increasing step off distances

     // evaluate distances between stepOffPoints
     // evaluate angles between stepOffPoint Binormal vectors

     // If its possible to classify the StepOffPoint 
     // as in or out with given tolerances,
     //    do so and return

     // If stepOffPoints and Binormals are nearly the same
     //    count a coincidence event,
     //    increase the stepDistance, and 
     //    continue

     // after 8 consecutive coincidence events classify the
     // point as coincident and return

   }

    Return Conditions:
     1. Stepped off a surface on iteration 1: return SM_ERR
     2. Stepped off a surface and last binormal angles indicated coincidence or zero sector:
          return oneof 2,3,4,5, 6 or 7.
     3. Stepped off a surface and last binormals were not coincident:
         if( inside) return 1
         if(!inside) return 0
     4. After 2 iterations and min_dist between stepOffPoints exceeds tolerance: 
          a. If lamina edge 
               and angle between surfaces > 0: return 1  
               and angle == 0:                 return 2 or 3
          b. If zero sector
               and only two faces total: return 6, 7, 2 or 3
          c. Else
               if( inside) return 1
               if(!inside) return 0
     5. If angle between Edgeuse and Classify Edgeuse binormal vectors exceeds 10 degrees
        for a lamina edge return 1
     6. If 8 different step-offDistances have been coincident
        determine which face is coincident and return 

MODIFICATION --- LocalMerge
  AllowCoincidence:

  When a check of this->GetBrep()->GetLocalMerge() is TRUE the behavior
  of this function is modified.  Coincident faces are not classified
  as coincident (return values of 2, 3, 4, or 5) but as in or out (return
  value of 0 or 1).

  As we have emphasized to SMLib Users, a Brep with coincident objects is 
  a corrupted brep, but some user applications choose to build such models.
  Functions are being added to the interface to work with these models
  in a limited fashion.  SmBrep::LocalMergeEdge() takes 
  two coincident edges and merges them by moving the edgeuses from 
  one of the edges to the other edge and then deleting what becomes a 
  "wire" edge. This is done by using Brep::GlueEdgesGeneral and that
  process uses SmEdge::RadialSectorClassify() to figure out how to insert the 
  Edgeuses into correct sector around the edge. As the face of an 
  edgeuse moves to being coincident the problem is how to figure out 
  the radial ordering. What the new code has done is fool the code in 
  that is not aware of the fact that the face of the edgeuse is coincident
  to some other face. This gives the correct ordering of edgeuses around
  the edge.
***********************************************************************/
SmStatus SmEdge::RadialSectorClassify     // eff:
 (double dNormalizedParam,                // in : edge location (normalized units)
  const SmEdgeuse * cpEdgeuseOfSector,    // in : specifies sector region being tested
  const SmEdgeuse * cpEdgeuseToClassify,  // in : contains face to classify
  ULONG & rlResult,                       // out: 0 - outside, 
                                          //      1 - inside, 
                                          //      2 - coincident with cpEdgeuseOfSector's face
                                          //      3 - coincident with cpEdgeuseOfSector's mate face
                                          //      4 - coincident with radial EU's face
                                          //      5 - coincident with radial mate's face
                                          //      6 - zero sector, in cpEdgeuseOfSector's face
                                          //      7 - zero sector, in cpEdgeuseOfSector's mate face
 SmTArray<SmFaceuse*> * pInsideFaceuses)  // NotUsed: in : Faceuses that are 'inside' //LocalMerge
   
 const                                         
{
  SM_REF1(pInsideFaceuses) ;
  // locals
  const SmEdgeuse *cpEU           = cpEdgeuseOfSector;
  const SmEdgeuse *cpEURadial     = cpEU->GetRadial();
  // SmZoneTol3d      sEdgeZoneTol3d = SmTol::GetZoneTol3d(this) ;
  SmCurve         *pEdgeCurve     = GetCurve();
  SmBoolean        bLaminaEdge    = IsLamina();

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe=FALSE;
  if (bDebugMe) 
    {
      SmBrep *pBrep = GetBrep() ;
      SmFaceuse *pFUSectorStart  = cpEU->GetFaceuse();
      SmFaceuse *pFUSectorEnd    = cpEURadial->GetFaceuse();
      SmFaceuse *pFUTarget       = cpEdgeuseToClassify->GetFaceuse();
      
      smgfx_Erase();
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(2,3, 1,0,1) ; this->Draw(); sm_GraphicsLoop() ;
      smgfx_SetLook(3,4, 1,0,0) ; cpEdgeuseToClassify->GetEdge()->Draw() ; sm_GraphicsLoop();
      smgfx_SetLook(1,2, 0,1,1) ; if (pFUSectorStart)  pFUSectorStart->Draw(SM_DM_CROSSHATCH) ; sm_GraphicsLoop();
      smgfx_SetLook(3,4, 0,1,1) ; if (cpEU) cpEU->Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,0,0) ; if (pFUSectorEnd) pFUSectorEnd->Draw(SM_DM_CROSSHATCH) ; sm_GraphicsLoop();
      smgfx_SetLook(3,4, 0,0,0) ; if (cpEU) cpEURadial->Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 1,0,0) ; if (pFUTarget) pFUTarget->Draw(SM_DM_CROSSHATCH) ; sm_GraphicsLoop();
      smgfx_SetLook(5,6, 1,0,0) ; if (cpEU) cpEdgeuseToClassify->Draw() ; sm_GraphicsLoop() ;
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

  // get edge's position for given param 
  double dThisParam = m_vInterval.Evaluate(dNormalizedParam);
  SmPoint3d sStartPoint;
  SER(pEdgeCurve->EvaluatePoint(dThisParam,sStartPoint));

  // classify edge locals
  SmEdge  *pOtherEdge  = cpEdgeuseToClassify->GetEdge(); NER(pOtherEdge);
  SmCurve *pOtherCurve = pOtherEdge->GetCurve(); NER(pOtherCurve);
  double   dTolerance  = pOtherEdge->GetTolerance() + GetTolerance();
  double   dOtherParam;

  // get the corresponding classify edge parameter value
  // 1. when the curves are the same - reuse param value
  // 2. when curves differ - drop point to curve (over a sequence of larger seek areas)
  if (pOtherCurve == pEdgeCurve) 
    {
      dOtherParam = dThisParam;
    }
  else 
    {
      SmBoolean bSuccess=FALSE;
      double dParam= 0.0, dDist;
      double dScale = 1.0;
      // try a sequence of drop distances
      while (!bSuccess && dScale < 1000000.0) 
        {
          SER(pOtherCurve->DropPoint(pOtherEdge->GetInterval(), // in : target curve allowed domain
                                     sStartPoint,               // in : Point to drop to curve
                                     NULL,                      // in : For Snapping, NULL=NoSnap, Vec pointing to CrvInside from desired EndPt for snapping.
                                                                //      when DropPt is within dDistTol in cpVecToCrvInside dir of EndPt, Snap to Ivl EndPt.
                                                                //      Vec handles ambiguities and makes sure that curves are not just touching at the ends.
                                     dTolerance*dScale,         // in : Used for SnapDist, and SM_SO_INTERSECT sXSectTol3d distance.
                                                                //      sXSectTol3d = dDistTol ;      SM_SO_INTERSECT 
                                                                //      sXSectTol3d = SM_BIG_DOUBLE ; SM_SO_MINIMIZE, SM_SO_NORMALIZE
                                                                //      sSnapTol3d  = dDistTol ;      SM_SO_MINIMIZE, SM_SO_NORMALIZE, SM_SO_INTERSECT    
                                     NULL,                      // in : If given, call cheap LocalPointSolve() else call expensive GlobalPointSolve()
                                     bSuccess,                  // out: TRUE = found a drop point
                                     dParam,                    // out: found drop curve param
                                     dDist)) ;                  // out: found drop distance
                                                                // in : SM_SO_MINIMIZE = allow nonNormal drops near endPoints
                                                                //      SM_SO_INTERSECT= like Minimize but point must be within dDistTol
                                                                //      SM_SO_NORMALIZE= exclude nonNormal drops near endPoints
                                                                //      default:[SM_SO_MINIMIZE] to preserve original behavior
          dScale = dScale * 10.0;
        }
      if (!bSuccess) SER(SM_ERR); // Unable to drop point to other curve.
      dOtherParam = dParam;
    } // end get classify edge's parameter value

#ifndef GWC_UNFINISHED_CHANGED_SECTION
  // locals for geometric classification
  /*
  //    \- cpEU=EdgeUse->Face->Surface
  //     \ 
  //      \     |- ClassifyEdgeUse->Face->Surface
  //       \    |
  //        \   |   /         (shown in the 'IN' position)
  //         \  |  /
  //          \ | /- cpEURadial=RadialEdgeuse->Face->Surface
  //           \|/
  //            + - edge
  */
  SmBoolean bDone = FALSE;
  SmBoolean bInside = FALSE;
  double dStepoffDistance = 0.0;
  ULONG lCount = 0;
  double dMinDist  = 0.0;
  double dLastMin  = 0.0;
  double dLast2Min = 0.0;
  double dAng1 = 0.0, dAng2 = 0.0, dAng3 = 0.0;
  double dLastStepoffDistance = 0.0;

  // detect special case: all planar surfaces
  SmBoolean bAllPlanes = FALSE;
  if (   cpEU->GetFace()->GetSurface()->IsKindOf(SmPlane_TYPE)
      && cpEURadial->GetFace()->GetSurface()->IsKindOf(SmPlane_TYPE)
      && cpEdgeuseToClassify->GetFace()->GetSurface()->IsKindOf(SmPlane_TYPE)) 
    {
      bAllPlanes = TRUE;
    }

  ULONG lNumConsecutiveCoincidences = 0;

  // locals sBin = Binormal Vector
  SmVector3d sPnt1, sBin1, sTan1, sNorm1;
  SmVector3d sPnt2, sBin2, sTan2, sNorm2;
  SmVector3d sPnt3, sBin3, sTan3, sNorm3;
  //SmOrientType eOr1, eOr2, eOr3;
  SmPoint3d sFirstPnt1;
      
  while (!bDone) 
    {
      // starting at the edge and moving inward as needed 
      //   to resolve tangent-noncoincident faces,
      // get vectors pointing towards inner part of the
      // 1. Edgeuse->Face,
      // 2. Edgeuse->RadialPartner->face, and
      // 3. ClassifyEdge->Face

      // 1. get Edgeuse->Face stepoff binormal
      SmBoolean bSteppedOffSurface = FALSE;
      if (cpEU->EvaluateBinormalStepOff(dThisParam,dStepoffDistance,
          sPnt1,sBin1,&sTan1,NULL,&sStartPoint) != SM_SUCCESS) 
        {
          bSteppedOffSurface = TRUE;
        }
      //eOr1 = cpEU->GetOrientation();
      sNorm1 = sTan1 * sBin1;
      
      // 2. get Edgeuse->RadialPartner->face stepoff binormal
      if (cpEURadial->EvaluateBinormalStepOff(dThisParam,dStepoffDistance,
          sPnt2,sBin2,&sTan2,NULL,&sStartPoint,&sTan1) != SM_SUCCESS) 
        {
          bSteppedOffSurface = TRUE;
        }
      //eOr2 = cpEURadial->GetOrientation();
      sNorm2 = sTan2 * sBin2;
      // 3. get ClassifyEdge->face stepoff binormal
      if (cpEdgeuseToClassify->EvaluateBinormalStepOff(dOtherParam,dStepoffDistance,
          sPnt3,sBin3,&sTan3,NULL,&sStartPoint,&sTan1) != SM_SUCCESS) 
        {
          bSteppedOffSurface = TRUE;
        }

      //eOr3 = cpEdgeuseToClassify->GetOrientation();
      sNorm3 = sTan3 * sBin3;

      // Project the binormal, sBin vectors, into the plane defined by sTan1
      // This takes care of cases where the edges are slightly different and produce 
      // binormals that are not exactly coincident.
      SER( sTan1.Unitize() );
      sBin3 = sTan1 * sBin3 * sTan1;

      // if any StepOff calculation stepped off a surface
      if (bSteppedOffSurface) 
        {
          // its an error to step off on first pass
          if (lCount == 0) { return SM_ERR; }

          // We have a coincidence if we are not diverging
          // and the angle is less than 1/2 degree.
          // (Note, we're using the angle values from the previous iteration.)

          if (dAng2 <= dAng3+SM_EFF_ZERO && dAng2 < 0.5 * SM_PI / 180.0) 
            { 
              rlResult = ( sNorm2.Dot(sNorm3) > 0.0 ) ? 2 : 3;
              return SM_SUCCESS;
            }
          else if (dAng3 <= dAng2+SM_EFF_ZERO && dAng3 < 0.5 * SM_PI / 180.0) 
            {
              rlResult = ( sNorm2.Dot(sNorm3) > 0.0 ) ? 4 : 5;
              return SM_SUCCESS;
            }
          else if ( dAng1 < 0.5 * SM_PI / 180.0 )
            {
              // Zero sector.
              rlResult = ( sBin3.Dot(sNorm1) > 0.0 ) ? 6 : 7;
              return SM_SUCCESS;
            }

          // Just take the best answer that we could find.
          if (bInside) {rlResult = 1;}
          else         {rlResult = 0;}

          return SM_SUCCESS;

        } // end steppedOffSurface check

#ifdef SM_DEBUG_CODE
      if (bDebugMe) 
        {
          SmFaceuse *pFU  = cpEU->GetFaceuse();
          SmFaceuse *pFU2 = cpEURadial->GetFaceuse();
          SmFaceuse *pFU3 = cpEdgeuseToClassify->GetFaceuse();
      
          smgfx_Erase();
          smgfx_SetLook(1,2, 1,0,1) ; this->Draw(); sm_GraphicsLoop() ;
          smgfx_SetLook(3,4, 1,0,0) ; cpEdgeuseToClassify->GetEdge()->Draw() ; sm_GraphicsLoop();
          smgfx_SetLook(1,2, 0,0,0) ; if (pFU)  pFU->Draw(SM_DM_CROSSHATCH) ; sm_GraphicsLoop();
          smgfx_SetLook(1,2, 0,0,1) ; if (pFU2) pFU2->Draw(SM_DM_CROSSHATCH) ; sm_GraphicsLoop();
          smgfx_SetLook(1,2, 1,0,0) ; if (pFU3) pFU3->Draw(SM_DM_CROSSHATCH) ; sm_GraphicsLoop();

          smgfx_SetLook(5,6,  0.5,1,0); sPnt1.Draw();        sm_GraphicsLoop();
                                        sBin1.Draw(&sPnt1);  sm_GraphicsLoop();
          smgfx_SetLook(5,6,  0,0,1);   sNorm1.Draw(&sPnt1); sm_GraphicsLoop();
          smgfx_SetLook(5,8,  0,1,1);   sPnt2.Draw();        sm_GraphicsLoop();
                                        sBin2.Draw(&sPnt2);  sm_GraphicsLoop();
          smgfx_SetLook(5,10, 1,0,1);   sPnt3.Draw();        sm_GraphicsLoop();
                                        sBin3.Draw(&sPnt3);  sm_GraphicsLoop();
          sm_GraphicsLoop();
        }
#endif // SM_DEBUG_CODE

      // classify the binormal vector set
      // First check for a zero sector.
      if ( bLaminaEdge )
        { bInside = TRUE; }
      else
        { bInside = sBin3.IsInsideSector( sBin1, sNorm1, sBin2 ); }

      bDone = TRUE;

#else // GWC_UNFINISHED_CHANGED_SECTION - needs to be debugged before being used
  // locals for geometric classification
  /*
  //    \- cpEU=EdgeUse->Face->Surface (object 1)
  //     \ 
  //      \     |- ClassifyEdgeUse->Face->Surface  (Object 3)
  //       \    |
  //        \   |   /         (shown in the 'IN' position)
  //         \  |  /
  //          \ | /- cpEURadial=RadialEdgeuse->Face->Surface  (Object 2)
  //           \|/
  //            + - edge
  */
  SmBoolean bDone = FALSE ;
  SmBoolean bInside = UNSURE ;
  SmBoolean bLastInside = UNSURE ;
  SmBoolean bInsideChanged = FALSE ;
  double dStepoffDistance = 0.0;
  ULONG lCount = 0;
  double dMinDist  = 0.0;
  double dLastMin  = 0.0;
  double dLast2Min = 0.0;
  double dAng1 = 0.0, dAng2 = 0.0, dAng3 = 0.0;
  double dLastStepoffDistance = 0.0;

  // detect special case: all planar surfaces
  SmBoolean bAllPlanes = FALSE;
  if (   cpEU->GetFace()->GetSurface()->IsKindOf(SmPlane_TYPE)
      && cpEURadial->GetFace()->GetSurface()->IsKindOf(SmPlane_TYPE)
      && cpEdgeuseToClassify->GetFace()->GetSurface()->IsKindOf(SmPlane_TYPE)) 
    {
      bAllPlanes = TRUE;
    }

  ULONG lNumConsecutiveCoincidences = 0;

 SmPoint3d sFirstPnt1;
  while (!bDone) 
    {
      // locals sBin = Binormal Vector
      SmVector3d   sPnt1, sBin1, sTan1, sNorm1, sStepNorm1 ;
      SmVector3d   sPnt2, sBin2, sTan2, sNorm2, sStepNorm2 ;
      SmVector3d   sPnt3, sBin3, sTan3, sNorm3, sStepNorm3 ;
      SmVector2d   sStepUV1, sStepUV2, sStepUV3 ;
      SmOrientType eOr1, eOr2, eOr3;
      
      // starting at the edge and moving inward as needed 
      //   to resolve tangent-noncoincident faces,
      // get vectors pointing towards inner part of the
      // 1. Edgeuse->Face,
      // 2. Edgeuse->RadialPartner->face, and
      // 3. ClassifyEdge->Face

      // 1. get Edgeuse->Face stepoff binormal
      SmBoolean bSteppedOffFaceUVDomain = FALSE;
      if (cpEU->EvaluateBinormalStepOff(dThisParam,dStepoffDistance,
                                        sPnt1,sBin1,&sTan1,&sStepNorm1,
                                        &sStartPoint, NULL, 
                                        &sStepUV1) != SM_SUCCESS) 
        {
          bSteppedOffFaceUVDomain = TRUE;  
        }
      eOr1 = cpEU->GetOrientation();
      sNorm1 = sTan1 * sBin1;
      
      // 2. get Edgeuse->RadialPartner->face stepoff binormal
      if (cpEURadial->EvaluateBinormalStepOff(dThisParam,dStepoffDistance,
                                              sPnt2,sBin2,&sTan2,&sStepNorm2,
                                              &sStartPoint, NULL, 
                                              &sStepUV2) != SM_SUCCESS) 
        {
          bSteppedOffFaceUVDomain = TRUE;
        }
      eOr2 = cpEURadial->GetOrientation();
      sNorm2 = sTan2 * sBin2;
      // 3. get ClassifyEdge->face stepoff binormal
      if (cpEdgeuseToClassify->EvaluateBinormalStepOff(dOtherParam,dStepoffDistance,
                                                       sPnt3,sBin3,&sTan3,&sStepNorm3,
                                                       &sStartPoint, NULL, 
                                                       &sStepUV3) != SM_SUCCESS) 
        {
          bSteppedOffFaceUVDomain = TRUE;
        }

      eOr3 = cpEdgeuseToClassify->GetOrientation();
      sNorm3 = sTan3 * sBin3;

      // Project the binormal, sBin vectors, into the plane defined by sTan1
      // This takes care of cases where the edges are slightly different and produce 
      // binormals that are not exactly coincident.
      SER(sNorm3.Unitize());
      sBin3 = sNorm3 * sBin3 * sNorm3;

      // classify the binormal vector set
      bLastInside = bInside ;
      if (sBin3.IsInsideSector(sBin1,sNorm1,sBin2)) { if(bInside == FALSE) bInsideChanged = TRUE ; 
                                                      bInside = TRUE;  
                                                    }
      else                                          { if(bInside == TRUE) bInsideChanged = TRUE ;
                                                      bInside = FALSE; 
                                                    }
      if (bLaminaEdge) 
        { bInside = TRUE; }
      bDone = TRUE;

      // after bInsideChange becomes TRUE, check step off points to see if they
      //   are still inside the trim boundaries of the faces.  The idea is that
      //   it's likely that the search has crossed a trim boundary when the classification
      //   of in/out changes.
      //   NOTE: Face trim boundaries can be very much smaller than Face UVDomains.
      if(bInsideChanged && bSteppedOffFaceUVDomain == FALSE)
        {
          // check sStepUV1 against cpEU->GetFace UVTrimBoundaries
          SmPointClassification sPointClassification ;
          cpEU->GetFace()->PointClassify(sStepUV1, sEdgeZoneTol3d, FALSE, FALSE, sPointClassification) ;
          if(sPointClassification.GetPointClass() == SM_PC_UNKNOWN)
            { bSteppedOffFaceUVDomain = TRUE ; }

          // check sStepUV2 against cpEURadial->GetFace() UVTrimBoundaries
          if(bSteppedOffFaceUVDomain == FALSE)
            {
              cpEURadial->GetFace()->PointClassify(sStepUV2, sEdgeZoneTol3d, FALSE, FALSE, sPointClassification) ;
              if(sPointClassification.GetPointClass() == SM_PC_UNKNOWN)
                { bSteppedOffFaceUVDomain = TRUE ; }
            }

          // check sStepUV3 against cpEdgeuseToClassify->GetFace() UVTrimBoundaries
          if(bSteppedOffFaceUVDomain == FALSE)
            {
              cpEdgeuseToClassify->GetFace()->PointClassify(sStepUV3, sEdgeZoneTol3d, FALSE, FALSE, sPointClassification) ;
              if(sPointClassification.GetPointClass() == SM_PC_UNKNOWN)
                { bSteppedOffFaceUVDomain = TRUE ; }
            }
        } // end need to do expensive point containment tests

      // if any StepOff calculation stepped off a surface
      if (bSteppedOffFaceUVDomain) 
        {
          // its an error to step off on first pass
          if (lCount == 0) return SM_ERR;

          // We have a coincidence if we are not diverging
          // and the angle is less than 1/2 degree.
          if (dAng2 <= dAng3+SM_EFF_ZERO && dAng2 < 0.5 * SM_PI / 180.0) 
            { 
              rlResult = ( sNorm1.Dot(sNorm3) > 0.0 ) ? 2 : 3;
              return SM_SUCCESS;
            }
          else if (dAng3 <= dAng2+SM_EFF_ZERO && dAng3 < 0.5 * SM_PI / 180.0) 
            {
              rlResult = ( sNorm2.Dot(sNorm3) > 0.0 ) ? 4 : 5;
              return SM_SUCCESS;
            }

          // Just take the best answer that we could find.
          SM_ASSERT(bInside != UNSURE) ;
          rlResult =  (bLastInside != UNSURE)
                    ? (bLastInside ? 1 : 0) 
                    : (bInside     ? 1 : 0) ;

          return SM_SUCCESS;

        } // end steppedOffSurface check

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
      if (bDebugMe) 
        {
          SmFaceuse *pFU  = cpEU->GetFaceuse();
          SmFaceuse *pFU2 = cpEURadial->GetFaceuse();
          SmFaceuse *pFU3 = cpEdgeuseToClassify->GetFaceuse();
      
          smgfx_Erase();
          smgfx_SetLook(1,2, 1,0,1) ; this->Draw(); sm_GraphicsLoop() ;
          smgfx_SetLook(3,4, 1,0,0) ; cpEdgeuseToClassify->GetEdge()->Draw() ; sm_GraphicsLoop();
          smgfx_SetLook(1,2, 0,0,0) ; if (pFU)  pFU->Draw(SM_DM_CROSSHATCH) ; sm_GraphicsLoop();
          smgfx_SetLook(1,2, 0,0,1) ; if (pFU2) pFU2->Draw(SM_DM_CROSSHATCH) ; sm_GraphicsLoop();
          smgfx_SetLook(1,2, 1,0,0) ; if (pFU3) pFU3->Draw(SM_DM_CROSSHATCH) ; sm_GraphicsLoop();

          smgfx_SetLook(5,6,  0.5,1,0); sPnt1.Draw();        sm_GraphicsLoop();
                                        sBin1.Draw(&sPnt1);  sm_GraphicsLoop();
          smgfx_SetLook(5,6,  0,0,1);   sNorm1.Draw(&sPnt1); sm_GraphicsLoop();
          smgfx_SetLook(5,8,  0,1,1);   sPnt2.Draw();        sm_GraphicsLoop();
                                        sBin2.Draw(&sPnt2);  sm_GraphicsLoop();
          smgfx_SetLook(5,10, 1,0,1);   sPnt3.Draw();        sm_GraphicsLoop();
                                        sBin3.Draw(&sPnt3);  sm_GraphicsLoop();
          sm_GraphicsLoop();
        }
#endif // SM_DEBUG_CODE
    }  // end while not done
#endif // GWC_UNFINISHED_CHANGED_SECTION

      // check distances between stepOff points to detect coincidence
      //   Note that once we stepoff we should start testing the vectors from the
      //   start point of each curve to the current step point.  We should stepoff
      //   until the distance between them is greater the dTolerance.  
      //double dDiff1 = sPnt1.DistanceBetween(sPnt2);
      double dDiff2 = sPnt1.DistanceBetween(sPnt3);
      double dDiff3 = sPnt2.DistanceBetween(sPnt3);
      dMinDist      = smos_Min(dDiff2,dDiff3);
      //if (bLaminaEdge) { dDiff1 = SM_BIG_DOUBLE; }

      // after 2 iterations and given nicely separated stepoff points
      if (lCount > 2 && dMinDist > dTolerance) 
        {
          // and working with a lamina edge
          if (bLaminaEdge) 
            {
              if (dAng2 > SM_EFF_ZERO_SQRT) 
                { rlResult = 1; } // face is inside
              else 
                // face is coincident with cpEdgeuseOfSector face
                // Return 2 or 3 depending on direction.
                { rlResult = ( sNorm1.Dot( sNorm3 ) > 0.0 ) ? 2 : 3; }

              return SM_SUCCESS;
            }

          // If sector angle is zero (faces of EU and its radial partner are parallel)
          // and there are no other faces, that's the same as a lamina edge.
          // [110321]
          if ( dAng1 < SM_EFF_ZERO_SQRT )
            {
              SmTArray< SmFace* > sFaces;
              this->GetFaces( sFaces );
              if ( sFaces.GetSize() < 3 )
                {
                  if (dAng2 > SM_EFF_ZERO_SQRT) 
                    // Zero sector.  Return 6 or 7 depending on direction.
                    { rlResult = ( sBin3.Dot( sNorm1 ) > 0.0 ) ? 6 : 7; }
                  else 
                    // face is coincident with cpEdgeuseOfSector face
                    // Return 2 or 3 depending on direction.
                    { rlResult = ( sBin3.Dot( sNorm1 ) > 0.0 ) ? 2 : 3; }

                  return SM_SUCCESS;
                }
            } // end 'pseudo-lamina' check.

          // get out of big loop - main exit point
          break;
        } // end beyond coincidence check

      // calculate the angles between the 3 different binormal vectors
      SER(sBin1.AngleBetween(sBin2,dAng1));
//        if (bLaminaEdge) { dAng1 = SM_PI; }
      SER(sBin1.AngleBetween(sBin3,dAng2));
      SER(sBin2.AngleBetween(sBin3,dAng3));

      // when any of the binormal angles are less than 10 degrees 
      // then classify the surfaces at an increased step off distance from
      // the edge
      if (   dAng1 < 10.0 * SM_PI / 180.0
          || dAng2 < 10.0 * SM_PI / 180.0
          || dAng3 < 10.0 * SM_PI / 180.0) 
        {
          if (bLaminaEdge) 
            {
              if (dAng2 > 10.0 * SM_PI / 180) 
                {
                  rlResult = 1;
                  return SM_SUCCESS;
                }
            }
          lCount ++;

          // let stepoffDistance be
          //   1st iteration: dTolerance
          //   2nd iteration: 2*dTolerance
          //   more         : if(minDist==LastMinDist) 2*LastStepOffDistance 
          //                  else a heurestic guess for growth or shrinkage
          //

          if (lCount < 20) { bDone = FALSE; }
          if (lCount == 1) { dStepoffDistance = dTolerance; }
          if (lCount == 2) 
            {
              dStepoffDistance = dTolerance * 2.0;
            }
          if (lCount > 2) 
            {
              double dDenom = dMinDist - dLastMin;
              if (smos_Fabs(dDenom) < SM_EFF_ZERO) 
                {
                  dLastStepoffDistance = dStepoffDistance;
                  dStepoffDistance = 2.0 * dLastStepoffDistance;
                }
              else 
                { 
                  double dRatio = (dStepoffDistance - dLastStepoffDistance) / dDenom;
                  dLastStepoffDistance = dStepoffDistance;
                  dStepoffDistance = dTolerance * dRatio;
                  // Don't let step distance grow or shrink by more than a factor of 3
                  if (dStepoffDistance < 0.0) 
                    {
                      dStepoffDistance = 2.0 * dLastStepoffDistance;
                    }
                  else if (dStepoffDistance > 3.0 * dLastStepoffDistance) 
                    {
                      dStepoffDistance = 3.0 * dLastStepoffDistance;
                    }
                  else if (dStepoffDistance < dLastStepoffDistance / 3.0) 
                    {
                      dStepoffDistance = dLastStepoffDistance / 3.0;
                    }
                }
            } // end iteration > 2 check
        } // end nearly parallel binormal vector check

      // Check for coincidence -- Note that it may be possible
      // for this check to go wrong in either direction.
      SmBoolean bCoincidence = FALSE;

      // when minDist is not changing over several iterations
      if (   lCount > 2 
          && smos_Fabs(dMinDist-dLastMin)  < dMinDist/100.0+SM_EFF_ZERO
          && smos_Fabs(dMinDist-dLast2Min) < dMinDist/100.0+SM_EFF_ZERO) 
        {
          bCoincidence = TRUE;
        }

      // when angle between binormals has been very small over several iterations
      if (   lCount > 2 
          && (   dAng2 < SM_DEG2RAD( 0.01 )   // was 0.001 [B162]
              || dAng3 < SM_DEG2RAD( 0.01 ))) 
        {
          bCoincidence = TRUE;
        }

      // Make sure we have 8 consecutive coincidences prior to making
      // the final call on the coincidence situation.
      // (Note, 4 is not enough: Reg_Bug_58.)
      if (bCoincidence) 
        {
          lNumConsecutiveCoincidences ++;
          if (lNumConsecutiveCoincidences < 8) 
            {
              bCoincidence = FALSE;
            }
        }
      else 
        { 
          lNumConsecutiveCoincidences = 0;
        }

      if (bAllPlanes) 
        {
          if (dAng2 < SM_DEG2RAD(0.001) || dAng3 < SM_DEG2RAD(0.001)) 
            {
              bCoincidence = TRUE;
            }
        }

      if (bCoincidence) 
        {
          // We have a coincidence if we are not diverging
          // and the angle is less than 1/2 degree
          
          // arrive here when ClassifyFaceuse is coincident with one of the sector faceuse bounds
          // Check faceuse normals to set
          //      rlResult = 2 - coincident with cpEdgeuseOfSector's face     
          //                 3 - coincident with cpEdgeuseOfSector's mate face
          //                 4 - coincident with radial EU's face             
          //                 5 - coincident with radial mate's face           

#ifdef SM_DEBUG_CODE
          if (bDebugMe) 
            {
              SmFaceuse *pFU  = cpEU->GetFaceuse();
              SmFaceuse *pFU2 = cpEURadial->GetFaceuse();
              SmFaceuse *pFU3 = cpEdgeuseToClassify->GetFaceuse();
      
              smgfx_Erase();
              smgfx_SetLook(1,2, 1,0,1) ; this->Draw(); sm_GraphicsLoop() ;
              smgfx_SetLook(3,4, 1,0,0) ; cpEdgeuseToClassify->GetEdge()->Draw() ; sm_GraphicsLoop();
              smgfx_SetLook(1,2, 0,0,0) ; if (pFU)  pFU->Draw(SM_DM_CROSSHATCH) ; sm_GraphicsLoop();
              smgfx_SetLook(1,2, 0,0,1) ; if (pFU2) pFU2->Draw(SM_DM_CROSSHATCH) ; sm_GraphicsLoop();
              smgfx_SetLook(1,2, 1,0,0) ; if (pFU3) pFU3->Draw(SM_DM_CROSSHATCH) ; sm_GraphicsLoop();
              sm_GraphicsLoop() ;
            }
#endif
          // when ang2 (TargetFaceuse/ClassifyFaceuse angle) is small and less than ang3
          if (   dAng2 <= dAng3+SM_EFF_ZERO 
              && dAng2 < 0.5 * SM_PI / 180.0) 
            { 
              rlResult = 2;
              if (sNorm1.Dot(sNorm3) < 0.0) 
                {
                  rlResult = 3;
                }
              return SM_SUCCESS;
            }

          // else when ang3 (RadialMateFaceuse/ClassifyFaceuse angle) is small and less than ang2
          else if (   dAng3 <= dAng2+SM_EFF_ZERO 
                   && dAng3 < 0.5 * SM_PI / 180.0) 
            {
              rlResult = 4;
              SmVector3d sNorm4 = sTan2 * sBin2;
              SmVector3d sNorm5 = sTan3 * sBin3;
              if (sNorm4.Dot(sNorm5) < 0.0) 
                {
                  rlResult = 5;
                }
              return SM_SUCCESS;
            }
        } // end ClassifyFaceuse coincident to one of sector Faceuses check

      if (bAllPlanes) 
        {
          break;
        }

      dLast2Min = dLastMin;
      dLastMin  = dMinDist;

    } // end while !bDone

  if (bInside) {rlResult = 1;}
  else         {rlResult = 0;}

  if (bLaminaEdge) 
    {
      rlResult = 1;
    }

  return SM_SUCCESS;

} // end SmEdge::RadialSectorClassify

/*******************************************************************//**
PURPOSE: Find the radial sector containing (or the face coincident with)
     a given face.

    For cases where the face is bounded by this edge's curve but not
    connected to this edge.

NOTES:
    When a face is contained by a radial sector
      o The return value rpRadialEdgeuse is set to indicate which sector,
      o The value of rpCoincidentFaceuse is set to NULL

    When a face is coincident with another face
      o The return value rpRadialEdgeuse is set to indicate which face,
      o The value of rpCoincidentFaceuse is set to the coincident faceuse.

    In a manifold model, the edge is attached to each of its regions
    through 2 edgeuse objects moving in opposite directions.  Use the
    eRequiredEUOrientation to specify the preferred return edgeuse
    direction.

    Coincidence is reported with faceuse objects indicating which side
    of the face is coincident.  This is detected by measuring tolerance
    sized displacements between the face and the coincident face.

    If two existing faces are already coincident with each other, and the
    given face is in their radial sector, then rpCoincidentFaceuse will
    be set as well.  Currently, the caller of this routine treats both of
    those situations in the same way; if that changes and we have to
    differentiate between those cases, we will probably have to add an
    argument to this routine.

    When cpRegionOfSector is specified only look in that region
    and only look for non-coincident situations.  When the edge
    has only 1 edgeuse attached to the cpRegionOfSector in the required
    orientation, then that edgeuse is returned without checking
    the geometry.

    Both the results will be NULL if no valid answer is found.

METHOD ---
   Classifies the face in the neighborhood of a sequence of 10
   edgePoints along the length of the Edge
   and returns the first successful in or coincident classification
   of any of those points.

OVERVIEW NOTES ---
    An edge's radial sectors are the regions attached to that edge
    through edgeuse->faceuse->shell->region pointers.  Each radial
    sector is bounded by the faces attached to one of the edge's
    edgeuse and that edgeuse's RadialPartner.

    A face is classified to a sector when that region completely
    contains the face in the neighborhood of the edge. A Face
    which is tangent to a sector boundary along the edge but
    diverges from the face somewhere in its interior will not be
    classified as coincident but as in/out of the region depending
    on how the face diverges from the boundary face.

***********************************************************************/
SmStatus SmEdge::FindRadialSector
  (const SmEdgeuse * cpEdgeuseToClassify,    // in : contains face whose geometry will be checked
   const SmRegion  * cpRegionOfSector,       // in : If NULL it will
                                             //      test sectors in all regions for a possible answer.
   SmOrientType      eRequiredEUOrientation, // in : The answer must be the edgeuse which has this orientation
                                             //      of the faceuse of an edgeuse with this orientation.
   SmEdgeuse      *& rpRadialEdgeuse,        // out: If an answer is found then this will be the
                                             //      resulting edgeuse; otherwise Null.  In case of
                                             //      coincidence this will be the edgeuse that's in
                                             //      rpCoincidentFaceuse.
   SmFaceuse      *& rpCoincidentFaceuse,    // out: If there is no coincidence this will be NULL.  If a
                                             //      coincidence is found then this is the coincident
                                             //      faceuse.  Note that we should never get a
                                             //      coincidence if the region is specified.
   SmTArray<SmFaceuse*> * pInsideFaceuses)   // in : Inside face uses // LocalMerge
    const
{
  // init returns
  rpRadialEdgeuse     = NULL;
  rpCoincidentFaceuse = NULL;

  // locals
  ULONG ii, jj;
  SmEdgeuse *pData[32], *pData2[32];
  SmTArray<SmEdgeuse*> sEdgeuses(32,pData);   // list of culled edgeuses
  SmTArray<SmEdgeuse*> sEUs(32,pData2);       // list of all edgeuses

  // get edgeuses that pass input region and orientation specifications
  GetEdgeuses(sEUs);
  for (ii=0; ii<sEUs.GetSize(); ii++) 
    {
      SmEdgeuse *pEU     = sEUs[ii];
      SmShell   *pShell  = pEU->GetShell();
      SmRegion  *pRegion = pShell->GetRegion();
      // when edgeuse's region and orientation match input specs
      if (   (   cpRegionOfSector == NULL 
              || pRegion == cpRegionOfSector) 
          && pEU->GetOrientation() == eRequiredEUOrientation) 
        {
          // add it to the list
          sEdgeuses.Add(pEU);
        }
    } // end iter every edgeuse

  // no work - no candidate edgeuses after culling
  if (sEdgeuses.GetSize() == 0) 
    { return SM_ERR; }

  // Special case: one possible answer and a region
  if (   sEdgeuses.GetSize() == 1 
      && cpRegionOfSector != NULL) 
    {
      // use the answer and return
      rpRadialEdgeuse = sEdgeuses[0];
      return SM_SUCCESS;
    }

  // Special case: wire edge
  if (sEdgeuses.GetSize() == 1 && IsWire()) 
    {
      rpRadialEdgeuse = sEdgeuses[0];
      return SM_SUCCESS;
    }

  ULONG lResult;

  // for 10 points along the edge until one classifies uniquely
  for (ii=0; ii<10; ii++) 
    {
      double dParam = (ii == 0) ? 0.5 : ii/11.0 ;

      // for every candidate edgeuse
      for (jj=0; jj<sEdgeuses.GetSize(); jj++) 
        {
          SmEdgeuse *pEU = sEdgeuses[jj];

          // classify given edgeuse against current candidate edgeuse
          if(   RadialSectorClassify(dParam,pEU,cpEdgeuseToClassify,lResult, 
                                     pInsideFaceuses) // LocalMerge
             != SM_SUCCESS) 
            {
              return SM_ERR;
            }

          // branch on classification = in/out/coincident
          switch(lResult)
            {
              // case : out 
              case 0 : continue ;                                 
              
              // case : in 
              case 1 : rpRadialEdgeuse = pEU;                    
                       return SM_SUCCESS;

              // cases : coincident
              case 2 :  rpRadialEdgeuse     = pEU;
                        rpCoincidentFaceuse = rpRadialEdgeuse->GetFaceuse(); 
                        return SM_SUCCESS;

              case 3 :  rpRadialEdgeuse     = pEU->GetMate();
                        rpCoincidentFaceuse = rpRadialEdgeuse->GetFaceuse();
                        return SM_SUCCESS;

              case 4 :  rpRadialEdgeuse     = pEU->GetRadial();
                        rpCoincidentFaceuse = rpRadialEdgeuse->GetFaceuse();
                        return SM_SUCCESS;

              case 5 :  rpRadialEdgeuse     = pEU->GetRadial()->GetMate();
                        rpCoincidentFaceuse = rpRadialEdgeuse->GetFaceuse();
                        return SM_SUCCESS;

              case 6 :
              case 7 :
                      // Zero sector: the faces of pEU and its radial partner
                      // are parallel (coincident).  If there's only one face,
                      // then the sector is the whole 360 degrees, and we're in it.
                      // Also, if there are two faces, and there's a zero sector,
                      // then they must be coincident, so there's still only one sector.
                      // In the case of coincident faces, it doesn't matter which EU
                      // we return, they're effectively the same thing.
                      // We'll assume here that there are not going to be more than
                      // two coincident faces, so if there are more than two faces,
                      // we'll find the correct sector with one of the edgeuses.
                      // [110321]
                      {
                        SmTArray< SmFace* > sFaces;
                        this->GetFaces( sFaces );
                        ULONG lNumFaces = sFaces.GetSize();
                        if ( lNumFaces > 2 )
                          { continue; } // Try a different EU.

                        // RadialSectorClassify will return the EU or mate
                        // whose faceuse points in the direction of cpEdgeuseToClassify.
                        rpRadialEdgeuse = ( lResult == 6 ) ? pEU : pEU->GetRadial();

                        // Currently, we'll communicate this zero-sector information
                        // in the same way as a coincident face: the caller of this
                        // routine treats them identically.  If that situation changes,
                        // it may become necessary to change this, which would probably
                        // mean adding an argument to this call.
                        rpCoincidentFaceuse = rpRadialEdgeuse->GetFaceuse(); 
                        return SM_SUCCESS;
                      }

            } // end switch on lResult
        }  // end iter every candidate edgeuse
    }  // end iter 10 times

  // No valid answers found is a valid case.
  return SM_ERR;

} // SmEdge::FindRadialSector

/*******************************************************************//**
PURPOSE: Find the radial sector (or the face it is coincident with)
  of a curve along an edge. 

  For cases where a curve's end point is bounded by this edge.

NOTES: Both the results may be NULL if no valid answer is 
    found.

    See: FindRadialSector comments
***********************************************************************/
SmStatus SmEdge::FindRadialSectorOfCurve
  (double             dEdgeParam,            // in : given edgePoint to examine
   const SmCurve    & crCurve,               // in : curve to classify
   const SmExtent1d & crInterval,            // in : interval of curve
   SmOrientType       eOrientationToSector,  // in : If SM_OT_SAME - classify along positive direction of
                                             //                      curve starting at crInterval.GetMin()
                                             //      If SM_OT_OPPOSITE - classify along reverse direction of 
                                             //                          curve starting at crInterval.GetMax()
   SmBoolean          bForceSectoring,       // in : If TRUE will force the finding of RadialEdgeuse
                                             //      and not return coincident faceuse.
   SmEdgeuse       *& rpRadialEdgeuse,       // out: Edgeuse of containing region or coincident face
   SmFaceuse       *& rpCoincidentFaceuse)   // out: NULL for coincident curves
                                             //      else Faceuse containing given curve.
    const   
{
  // init return values
  rpRadialEdgeuse     = NULL;
  rpCoincidentFaceuse = NULL;

  // Wire Edges contain all curves in their primary edgeuse
  if (IsWire()) 
    {
      rpRadialEdgeuse = GetPrimaryEdgeuse();
      return SM_SUCCESS;
    }
  
  SmEdgeuse *pData[32];
  SmTArray<SmEdgeuse*> sEdgeuses(32,pData);

  // for every Edgeuse
  GetEdgeuses(sEdgeuses);
  for (ULONG j=0; j<sEdgeuses.GetSize(); j++) 
    {
      SmEdgeuse *pEU = sEdgeuses[j];
      ULONG lResult;

      // classify the curve against this edgeuse
      if (RadialSectorOfCurveClassify(pEU, dEdgeParam,
                                      crCurve, crInterval,
                                      eOrientationToSector,
                                      bForceSectoring,
                                      lResult) != SM_SUCCESS) 
        { continue; }

      // switch on the classification result
      switch(lResult)
        {
          // case out :
          case 0 : continue ;

          // case in : 
          case 1 : rpRadialEdgeuse = pEU;
                   return SM_SUCCESS;

          // cases: coincidence
          case 2 : rpCoincidentFaceuse = pEU->GetFaceuse();
                   return SM_SUCCESS;

          case 3 : rpCoincidentFaceuse = pEU->GetRadial()->GetFaceuse();
                   return SM_SUCCESS;
        }
    } // end iter every edgeuse

  // No valid answers found is a valid case.
  return SM_SUCCESS;

} // end SmEdge::FindRadialSectorOfCurve


/*******************************************************************//**
PURPOSE: Classifies a curve relative to a radial sector defined by
    an edgeuse and its radial pointer.  

    For cases where the curve's end point is bounded by this edge

NOTES: 
***********************************************************************/
SmStatus SmEdge::RadialSectorOfCurveClassify // eff: find sector containing or face coincident with curve
  (const SmEdgeuse  * cpEdgeuseOfSector,     // in : contains face and radialPartner->face that define sector boundary
   double             dEdgeParam,            // in : point on edge to examine
   const SmCurve    & crCurve,               // in : curve to classify
   const SmExtent1d & crInterval,            // in : interval of curve
   SmOrientType       eOrientationToSector,  // in : If SM_OT_SAME - classify curve along positive direction
                                             //                      starting at crInterval.GetMin()
                                             //      If SM_OT_OPPOSITE - classify curve along reverse direction 
                                             //                          starting at crInterval.GetMax()
   SmBoolean          bForceSectoring,       // in : Will force a sector resolution even if coincidence
                                             //      is found.  Result will be either 0 or 1 never 2 or 3.
   ULONG            & rlResult)              // out: 0 - outside, 
                                             //      1 - inside, 
                                             //      2 - coincident with cpEdgeuseOfSector's face
                                             //      3 - coincident with radial EU's face
 const
{
  // locals
  const SmEdgeuse *cpEU       = cpEdgeuseOfSector;
  const SmEdgeuse *cpEURadial = cpEU->GetRadial();
  //SmCurve         *pEdgeCurve = GetCurve();
  double dTolerance  = GetTolerance();
  double dStartParam = (eOrientationToSector == SM_OT_SAME)
                       ? crInterval.GetMin()
                       : crInterval.GetMax();
  SmBoolean bDone            = FALSE;
  SmBoolean bInside          = FALSE;
  ULONG     lCount           = 0;
  double    dMinDist         = 0.0;
  double    dLastMin         = 0.0;
  double    dLast2Min        = 0.0;
  SmPoint3d sFirstPnt1;

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe=FALSE;
  if (bDebugMe) 
    {
      SmBrep    *pBrep = GetBrep() ;
      SmFaceuse *pFU   = cpEU->GetFaceuse();
      SmFaceuse *pFU2  = cpEURadial->GetFaceuse();
      
      smgfx_Erase();
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(3,4, 0,1,0) ; cpEU->Draw();                 sm_GraphicsLoop() ;
      smgfx_SetLook(3,4, 1,0,1) ; cpEURadial->Draw();           sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,1) ; if (pFU) pFU->Draw(SM_DM_CROSSHATCH);   sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 1,1,0) ; if (pFU2) pFU2->Draw(SM_DM_CROSSHATCH); sm_GraphicsLoop();
      smgfx_SetLook(1,2, 0,0,1) ; pFU->GetShell()->Draw(); sm_GraphicsLoop();
      smgfx_SetLook(5,6, 1,0,0) ; crCurve.Draw(); sm_GraphicsLoop();
      smgfx_SetLook(5,8, 0,0,1) ; crCurve.DrawAt(dStartParam,0); sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
#endif

  //
  double dStepoffDistance = 0.0;
  while (!bDone) 
    {
      double dStepParam;
      SmVector3d sPnt1, sBin1, sTan1, sNorm1;
      SmVector3d sPnt2, sBin2, sTan2;
      SmVector3d sPnt3, sBin3;
      SmBoolean bClampped;

      // calculate this faceuse stepOff, binormal and normal vectors.
      SER(cpEU->EvaluateBinormalStepOff(dEdgeParam, dStepoffDistance,
                                        sPnt1, sBin1, &sTan1)) ;
      sNorm1 = sTan1 * sBin1;

      // calculate radialMate faceuse stepOff and binormal vector positions.
      SER(cpEURadial->EvaluateBinormalStepOff(dEdgeParam, dStepoffDistance,
                                              sPnt2, sBin2, &sTan2)) ;

      // calculate ClassificationCurve stepOff point and tangent
      crCurve.EuclidianStepOff(crInterval, dStartParam,
                               eOrientationToSector,
                               dStepoffDistance, dStepoffDistance/100.0,
                               bClampped, dStepParam);
      SmVector3d sPV[2];
      SER(crCurve.Evaluate(dStepParam,1,TRUE,sPV));
      if (eOrientationToSector == SM_OT_OPPOSITE) { sPV[1] = - sPV[1]; }
      sPnt3 = sPV[0];
      sBin3 = sPV[1];

      // Project binormals into plane which is perpendicular to edge
      SER(sTan1.Unitize());
      SER(sNorm1.Unitize());
      sBin2 = sTan1 * sBin2 * sTan1;
      sBin3 = sTan1 * sBin3 * sTan1;
      if (sBin3.LengthSquared() < SM_EFF_ZERO_SQ) 
        {
          return SM_ERR;
        }

#ifdef SM_DEBUG_CODE
      if (bDebugMe) 
        {
          SmBrep    *pBrep = GetBrep() ;
          SmFaceuse *pFU   = cpEU->GetFaceuse();
          SmFaceuse *pFU2  = cpEURadial->GetFaceuse();
      
          smgfx_Erase();
          smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
          smgfx_SetLook(3,3, 0,1,0) ; cpEU->Draw();                 sm_GraphicsLoop() ;
          smgfx_SetLook(3,3, 1,0,1) ; cpEURadial->Draw();           sm_GraphicsLoop() ;
          smgfx_SetLook(1,2, 0,1,1) ; if (pFU) pFU->Draw(SM_DM_CROSSHATCH);   sm_GraphicsLoop() ;
          smgfx_SetLook(1,2, 1,1,0) ; if (pFU2) pFU2->Draw(SM_DM_CROSSHATCH); sm_GraphicsLoop();
          smgfx_SetLook(1,2, 0,0,1) ; pFU->GetShell()->Draw();                sm_GraphicsLoop();
          smgfx_SetLook(5,4, 1,0,0) ; crCurve.Draw();                sm_GraphicsLoop();
          smgfx_SetLook(5,5, 0,0,1) ; crCurve.DrawAt(dStartParam,0); sm_GraphicsLoop();
          
          smgfx_SetLook(6,8, 0.5,1,0); sPnt1.Draw();        sm_GraphicsLoop();
                                       sBin1.Draw(&sPnt1);  sm_GraphicsLoop();
          smgfx_SetLook(6,8, 0,0,1);   sNorm1.Draw(&sPnt1); sm_GraphicsLoop();
          smgfx_SetLook(7,9, 0,1,1);   sPnt2.Draw();        sm_GraphicsLoop();
                                       sBin2.Draw(&sPnt2);  sm_GraphicsLoop();
          smgfx_SetLook(8,10, 1,0,1);  sPnt3.Draw();        sm_GraphicsLoop();
                                       sBin3.Draw(&sPnt3);  sm_GraphicsLoop();
          sm_GraphicsLoop();
        }
#endif
      // test the curve against the sector boundaries
      bInside = sBin3.IsInsideSector(sBin1,sNorm1,sBin2) ;
      bDone   = TRUE;

      // If some vectors are less than 10 degrees from each other then
      // reclassify after stepping off a little.

      double dAng1, dAng2, dAng3;
      SER(sBin1.AngleBetween(sBin2,dAng1));
      SER(sBin1.AngleBetween(sBin3,dAng2));
      SER(sBin2.AngleBetween(sBin3,dAng3));
      if (   dAng1 < 10.0 * SM_PI / 180.0
          || dAng2 < 10.0 * SM_PI / 180.0
          || dAng3 < 10.0 * SM_PI / 180.0) 
        {
          if (IsLamina()) 
            {
              if (dAng2 > 10.0 * SM_PI / 180) 
                {
                  rlResult = 1;
                  return SM_SUCCESS;
                }
            }
          lCount ++;
          if (lCount < 20) bDone = FALSE;
          if (lCount == 1) dStepoffDistance = dTolerance;
          if (lCount == 2) dStepoffDistance = dTolerance * 2.0;
          if (lCount >  2) dStepoffDistance = 4.0 * dStepoffDistance;
        }

      // Note that once we stepoff we should start testing the vectors from the
      // start point of each curve to the current step point.  We should stepoff
      // until the distance between them is greater the dTolerance.  
     // double dDiff1 = sPnt1.DistanceBetween(sPnt2);
      double dDiff2 = sPnt1.DistanceBetween(sPnt3);
      double dDiff3 = sPnt2.DistanceBetween(sPnt3);
      dMinDist = smos_Min(dDiff2,dDiff3);
      if (lCount > 2 && dMinDist > dTolerance) 
        {
          if (IsLamina()) 
            {
              if (dAng2 > SM_EFF_ZERO_SQRT) 
                {
                  rlResult = 1;
                  return SM_SUCCESS;
                }
            }
          bDone = TRUE;
        }

      // Check for coincidence -- Note that it may be possible
      // for this check to go wrong in either direction.
      SmBoolean bCoincidence = FALSE;
      if (   lCount > 2 
          && smos_Fabs(dMinDist-dLastMin)  < dMinDist/100.0+SM_EFF_ZERO
          && smos_Fabs(dMinDist-dLast2Min) < dMinDist/100.0+SM_EFF_ZERO) 
        {
          bCoincidence = TRUE;
        }
      if (   lCount > 2 
          && (   dAng2 < SM_EFF_ZERO 
              || dAng3 < SM_EFF_ZERO)) 
        {
          bCoincidence = TRUE;
        }

      if (bCoincidence) 
        {
          // We have a coincidence if we are not diverging
          // and the angle is less than 1/2 degree.
          if (bForceSectoring) break; // Jump out and set current result

          if (   dAng2 < dAng3 
              && dAng2 < 0.5 * SM_PI / 180.0) 
            { 
              rlResult = 2;
              return SM_SUCCESS;
            }
          else if (   dAng3 < dAng2
                   && dAng3 < 0.5 * SM_PI / 180.0) 
            {
              rlResult = 3;
              return SM_SUCCESS;
            }
        } // end Curve is coincident with sector face check

      //
      dLast2Min = dLastMin;
      dLastMin  = dMinDist;

    } // end sampling points until making classification

  if (bInside) {rlResult = 1;}
  else {rlResult = 0;}

  return SM_SUCCESS;
} // end SmEdge::RadialSectorOfCurveClassify

/*******************************************************************//**
PURPOSE: Set the primary edgeuse, taking care of pointer connections.

NOTES: 
***********************************************************************/
SmStatus SmEdge::ResetPrimaryEdgeuse( SmEdgeuse *pNewPrimary )
{
  if ( pNewPrimary == m_pList ) { return SM_SUCCESS; } // nothing to do.

  // Given EU must be one of ours.
  if ( ! this->IsInList( pNewPrimary ) )
    { return SM_ERR_INVALID_INPUT; }

  // Do the update.
  m_pList = SM_CAST_PTR( SmTopology, pNewPrimary );

  // Check to see whether we have to reverse the list, in order
  // to keep the Edgeuses' GetRadial() and GetMate() correct.
  // For this we check whether the Mate actually looks like the Radial.
  // Mate must be on the same Face, and also (for a seam) in a different Loopuse.
  // [B250]
  SmEdgeuse *pPrimMate = pNewPrimary->GetMate();
  if (    pNewPrimary->GetFace()    != pPrimMate->GetFace()
       || pNewPrimary->GetLoopuse() == pPrimMate->GetLoopuse() )
    { this->ReverseList(); }

  UpdateListSize();

  return SM_SUCCESS;
  
} // end SmEdge::ResetPrimaryEdgeuse

/*******************************************************************//**
PURPOSE: Update edge tolerance to the consistent Tolerance model
         encoded in SmTol::GetTol3d()

NOTES: Also updates tolerance values to the consistent Tolerance model
         For Faces connected to this Edge.
       If pVertex->RefreshTolerance() has been called for one of this
         Edges vertices, no need to call pEdge->RefreshTolerance().
         It's already been run.
***********************************************************************/
#ifdef SM_USE_OLDTOL
SmStatus SmEdge::RefreshTolerance
 (SmBoolean bNestedRefreshes)   // in : TRUE =Refresh tols of connected Faces
                                //      FALSE=don't, default:[TRUE]
{
  // locals
  ULONG ii ;
  SmTArray<SmFace*> sFaces ; GetFaces(sFaces) ;

#ifdef SM_USE_OLDTOL

  SM_OLDTOL_LINE // for every connected Face - Set Tol to Consistent tolerance
  SM_OLDTOL_LINE if(bNestedRefreshes)
  SM_OLDTOL_LINE   {
  SM_OLDTOL_LINE     for(ii=0;ii<sFaces.GetSize();ii++) { sFaces[ii]->SetTolerance(SmTol::GetZoneTol3d(sFaces[ii])) ; }
  SM_OLDTOL_LINE   }
  SM_OLDTOL_LINE 
  SM_OLDTOL_LINE // Set Edge to consistent tolerance
  SM_OLDTOL_LINE SetTolerance(SmTol::GetZoneTol3d(this)) ; 

#else // SM_USE_NEWTOL 

  SM_NEWTOL_LINE // locals
  SM_NEWTOL_LINE SmZoneTol3d sBrepZoneTol3d = GetBrep()->GetTolerance() ;
  SM_NEWTOL_LINE 
  SM_NEWTOL_LINE // for every connected Face - Set Tol to Consistent tolerance
  SM_NEWTOL_LINE if(bNestedRefreshes)
  SM_NEWTOL_LINE   {
  SM_NEWTOL_LINE     for(ii=0;ii<sFaces.GetSize();ii++) { sFaces[ii]->RefreshTolerance() ; }
  SM_NEWTOL_LINE   }
  SM_NEWTOL_LINE 
  SM_NEWTOL_LINE // increase small tolerances to target tol value
  SM_NEWTOL_LINE if (GetTolerance() < sBrepZoneTol3d) 
  SM_NEWTOL_LINE   {
  SM_NEWTOL_LINE     SetTolerance(sBrepZoneTol3d);
  SM_NEWTOL_LINE   }
  SM_NEWTOL_LINE 
#endif // SM_USE_NEWTOL

  // all done
  return SM_SUCCESS;

} // end SmEdge::RefreshTolerance
#endif // SM_USE_OLDTOL

/*******************************************************************//**
PURPOSE: Get the brep of an edge.

NOTES:
***********************************************************************/
SmBrep * SmEdge::GetBrep() const
{
  SmBrep    * pRet = NULL ;
  SmEdgeuse * pEU  = GetPrimaryEdgeuse() ;

  if(pEU == NULL) 
    { // pEU can be null during destruction. [L]
      if(m_cpContext != NULL && !m_cpContext->GetDoingBoolean()) 
        { SE(SM_ERR) ; }
    } 
  else             
    { SmShell *pS = pEU->GetShell() ;
      if(pS == NULL) { SE(SM_ERR) ; } 
      else           { pRet = pS->GetBrep() ; }
     }

  return pRet;

} // end SmEdge::GetBrep

// Remove Composites
//  /*******************************************************************//**
//  PURPOSE: If this edge is part of a CEdge, 
//           return parent CEdge else return NULL
//  
//  NOTES: A composite Edge's curve owner is an SmCEdge object.  
//    When a Edge is not a member of a composite Edge the Edge's curve owner
//    will be this Edge.
//  ***********************************************************************/
//  SmCEdge * SmEdge::GetCompositeEdgeOwner() const 
//  { 
//    // locals
//    SmCurve *pCurve = GetCurve() ;
//    SmEdge  *pEdge  = pCurve ? (SmEdge *)pCurve->GetEdge() : NULL ;
//  
//    // when this edge is part of a CEdge
//    if(pEdge && this != pEdge)
//      {
//        // return pEdge as a SmCEdge
//        if(pEdge->IsKindOf(SmCEdge_TYPE)) 
//          {
//            return( (SmCEdge *)pEdge) ;
//          }
//        else // something is wrong
//          { SE_MSG(SM_ERR,_T("SmEdge::GetCompositeEdgeOwner Composite Edge pointers are broken")) ; 
//            return NULL ;
//          }
//      }
//  
//    // arrive here - this edge is not composite
//    return(NULL) ; 
//  
//  } // end SmEdge::GetCompositeEdgeOwner

/*******************************************************************//**
PURPOSE: Set Edgeuse->UVTrimCurve = pUVTrimCurve for Edgeuse connecting thisEdge to TargetFace

NOTES: deletes preExisting Edgeuse->UVTrimCurve
***********************************************************************/
SmStatus SmEdge::SetTrimCurve
  (SmFace         * pFace,                  // in : Face connected to this edge
   SmBSplineCurve * pUVTrimCurve,           // in : UVTrimCurve to store
   double           dMaxDistanceToSurface,  // in : known Max Edge/>FaceTrimCurve gap
   SmBoolean        bDoValidation)          // in : TRUE = run ValidateGeometry() after change
                                            //      FALSE=
{
  SM_PTR_ARRAY(sEdgeuses, SmEdgeuse, 32) ; // SmTArray<SmEdgeuse*>
  GetEdgeuses(sEdgeuses);

  // for every Edge->Edgeuse
  for (ULONG i=0; i<sEdgeuses.GetSize(); i++) 
    {
      SmEdgeuse * pEdgeuse = (SmEdgeuse*)sEdgeuses[i];

      // Find the Edgeuse connecting this Edge to the target Face->UpwardFaceuse
      if(   pEdgeuse->GetFace()        == pFace
         && pEdgeuse->GetOrientation() == SM_OT_SAME) 
        {
          // Set the Edgeuse->UVTrimCurve
          pEdgeuse->SetUVTrimCurve(pUVTrimCurve, 
                                   dMaxDistanceToSurface, 
                                   TRUE) ; // TRUE = delete preExisting UVTrimCurve

          // when asked
          if ( bDoValidation )
            {  // Check the Geometry - output errors. Side effects: change Tol and Interval values 
               SER(ValidateGeometry(TRUE)); 
            }

          // all dpone
          return SM_SUCCESS;
        }
    }

  // should never arrive there
  SER(SM_ERR);
  return SM_ERR;

} // end SmEdge::SetTrimCurve

/*******************************************************************//**
// Remove Composites
//  PURPOSE: Replace Edge->3DCurve for both Edges and CEdges.
PURPOSE: Replace Edge->3DCurve for Edges.
         Deletes Edge's Curve and UVTrimCurves 

NOTES: Use  pBrep->Notify(SM_NO_PRE_EDIT, this, NULL, NULL) on the brep to delete the brep cache,
       otherwise the curve tree in the SmBrepCache object is still storing
       the deleted Edge pointer
***********************************************************************/
SmStatus SmEdge::Replace3DCurve
  (SmCurve *p3DCurve)
{
// Remove Composites
//  // when Owner is a CEdge - remove Edge from CompositeEdge 
//  if(GetCurve() && IsCompositeEdge()) 
//    {
//      // remove edge from CEdge - leave oldCurve alone for the CEdge
//      SmCEdge *pCEdge = (SmCEdge*) GetCurve()->GetOwner();
//      pCEdge->RemoveEdge(this);  // note: pEdge->Curve set to NULL not deleted, UVTrimCurves are deleted.
//    }

  // set Edge->Curve and Curve->Owner pointers
  SetCurve(p3DCurve,  // side effect: delete current this->UVTrimCurves
           TRUE) ;    // TRUE = delete preExisting Edge->Curve

  // when given a new 3DCurve - set back ptrs
  if ( p3DCurve != NULL )
    {
      p3DCurve->SetOwner( this ) ;
      SetInterval( p3DCurve->GetNaturalInterval()) ;
    }

  // all done
  return SM_SUCCESS;

} // end SmEdge::Replace3DCurve

/*******************************************************************//**
PURPOSE: Set the curve pointer of an edge - internal use only.

NOTES:
   Deletes all Edge->UVTrimCurves   (through Notify call)
   When bDeleteOldCurve == TRUE,  delete preExisting Edge->Curve
                        == FALSE, retain it and clear its owner if it is this edge.
   The caller remains responsible for setting the new curve's owner.

// Remove Composites
//   Caution: this method does not deal with SmCEdges (composite edges).
//   When appropriate, the caller must check for this Edge being a CEdge,
//   or a member of CEdge.  See examples in SmBrep.cpp.
***********************************************************************/
void SmEdge::SetCurve
 (SmCurve  * pCurve,           // in : New Curve for Edge
  SmBoolean  bDeleteOldCurve,  // in : TRUE = delete Old Curve, FALSE=don't, 
                               //      default:[FALSE]
  SmBoolean  bDbgWarnLeaks,    // in : TRUE = Signal DBGWarn when saving new curve over old, FALSE=don't, 
                               //      default:[TRUE]
  SmBoolean  bCallNotify)      // in : TRUE = SideEffect: delete all UVTrimCurves, FALSE=don't, 
                               //      default:[TRUE]
{ 
  if ( pCurve == m_pCurve )
    { return; }

  SmBrep *pBrep = ( m_pList == NULL ) ? NULL : this->GetBrep();

  // side effect: delete all UVTrimCurves
  if(bCallNotify)
    {
      Notify(SM_NO_CHANGE_GEOMETRY, pCurve, SM_NO_GET_BREP(this), m_pCurve);

      if ( pBrep != NULL ) // side effect: remove from spatial cache.
        { pBrep->Notify(SM_NO_RM_FROM_BREP, this, pBrep, m_pCurve); }
    }

  // when preExisting Curve exists
  if(m_pCurve) 
    {
      if ( bDeleteOldCurve )
        { delete m_pCurve ; m_pCurve = NULL ; }
      else
        {
          // Do not leave a retained curve pointing at an edge that no longer
          // uses it. Preserve an owner already assigned by a transfer caller.
          if ( m_pCurve->GetOwner() == this )
            { m_pCurve->SetOwner(NULL); }
          if ( bDbgWarnLeaks )
            { SM_DBG_WARN(_T("SmEdge::SetCurve - possible memory leak, replaced NonNUll m_pCurve value without deleting old m_pCurve")) ; }
        }
    }

  // update the value
  m_pCurve = pCurve ;

  // side effect: Add to spatial cache.
  if ( bCallNotify )
    {
      if ( pBrep != NULL )
        { pBrep->Notify(SM_NO_ADD_TO_BREP, this, pBrep, m_pCurve); }
    }

  // gwc: future experiment try the following line here
  // to try:  if(pCurve) { pCurve->SetOwner(this) ; }
   
} // end SmEdge::SetCurve

/*********************************************************
PURPOSE: Fix too-large tolerances by editing the Curve and 
  Edge->Curve shapes to lie more closely on their 
  connected surfaces near the intersection point.
  
NOTES: 
  returns with no changes if the intersection points fail
  to drop to any connected surfaces.     

METHOD ---
  1. Check CurvePoint/EdgePoint 
           CurvePoint/Surfaces
           EdgePoint/Surfaces  gaps and quit when gaps are near zero.

  2. a. if intersection point represents a surf/surf/surf intersection.
          get SSSIPoint and change curve and edge to interpolate SSSIPoint.
     b. else if edge or curve has nearZero tolerances, move intersection
          to surface/curve intersection and change other curve or edge
          to interpolate the intersection point. 
          Check new point and if tolerances are still large,
          reintersect edges and start over.
     c. else both edge and curve have nonZero tolerances, find points
        on connected surfaces for current point intersections,
        change curve and edge to interpolate those points,
        reintersect the curves, and start over

**********************************************************/
SmStatus SmEdge::RefineGeometry
  (SmMarkType eMarkType,    // in : uses without incrementing eMarkType value.
                            //      skip marked edges and mark all others.
                            //      oneof SM_MT_NOMARK = don't check marks before refining geometry
                            //            SM_MT_MARK   = skip marked edges and mark all others
                            //            SM_MT_MARK2
                            //            SM_MT_MARKIO
   SmBoolean &bMadeChange)  // out: TRUE = replaced edgeCurve with new surf/surf intersection
                            //             and updated tolerance.
                            //      FALSE= no changes made because it could not find a good surf/surf intersection
{
  // init output
  bMadeChange = FALSE ;

  // manage edge's mark
  if(eMarkType != SM_MT_NOMARK)
    {
      // no work - checking marks and edge is already marked
      if(IsMarked(eMarkType))
        { return(SM_SUCCESS) ; }
      else // mark this edge
        { Mark(eMarkType) ; }
   } // end managing marks

  // locals
  SmCurve        * pCurve           = GetCurve() ;
  SmVertex       * pStartVertex     = GetStartVertex() ;
  SmVertex       * pEndVertex       = GetOtherVertex(pStartVertex) ;
  SmBoolean        bMadeStartChange = FALSE ;
  SmBoolean        bMadeEndChange   = FALSE ;
  SmBoolean        bFoundCurve      = FALSE;
  double           dMaxGap3d        = 0.0 ;
  SmCurve        * pBestCurve       = NULL ;
  SmFace         * pFace1           = NULL, * pFace2=NULL ;
  SmBSplineCurve * pSurface1UVCurve = NULL, * pSurface2UVCurve=NULL ;
  SmPoint3d        sBestStartVertexPos ; 
  SmPoint3d        sBestEndVertexPos ;   

  // ensure start vertex is refined
  if(pStartVertex)                  
    { SER(pStartVertex->RefineGeometry(eMarkType, bMadeStartChange)) ; }
  
  // ensure end vertex is refined
  if(   pEndVertex
     && pEndVertex != pStartVertex) 
    { SER(pEndVertex->RefineGeometry(eMarkType, bMadeEndChange)) ; }

  // see if Edge->Curve can be defined from its connected faces by a surf/surf intersection
  SmStatus sRtn = CalcEdgeFormFromFaces(bFoundCurve, dMaxGap3d, pBestCurve,
                                        sBestStartVertexPos, 
                                        sBestEndVertexPos,   
                                        &pFace1, &pSurface1UVCurve,
                                        &pFace2, &pSurface2UVCurve) ;

  // when a new valid intersection curve was built - use it
  if(sRtn == SM_SUCCESS && bFoundCurve)
    {
      // inform the public - upcoming change
      Notify(SM_NO_PRE_EDIT, this, SM_NO_GET_BREP(this), NULL) ;

      // Set Edge->Curve = new pBestCurve
      SetCurve(pBestCurve) ; // does not free old curve but does delete current this->UVTrimCurve
      pBestCurve->SetOwner( this );

      // find tolerances
      SM_OLDTOL_LINE SmZoneTol3d sZoneTol3d = SmTol::GetZoneTol3d(GetBrep()) ;                    // default for this edge
      SM_OLDTOL_LINE SetTolerance(smos_Max((double)sZoneTol3d, dMaxGap3d)) ;

      SetInterval (pBestCurve->GetNaturalInterval()) ;
      SetTrimCurve(pFace1, pSurface1UVCurve, dMaxGap3d) ; // deletes old UVTrimCurve
      SetTrimCurve(pFace2, pSurface2UVCurve, dMaxGap3d) ; // deletes old UVTrimCurve

      // inform the public - change done
      Notify(SM_NO_POST_EDIT, this, SM_NO_GET_BREP(this), NULL) ;

      // set output
      bMadeChange = TRUE ;

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
      if(bDebugMe)
        {
          SmBrep    * pBrep = GetBrep() ;
          SmPoint3d   sStartPt1, sEndPt1 ;
          SmPoint3d   sStartPt2, sEndPt2 ;
          SmSurface * pSurface1 = pFace1 ? pFace1->GetSurface() : NULL ;
          SmSurface * pSurface2 = pFace2 ? pFace2->GetSurface() : NULL ;
          SmPoint2d   sStartUV1, sEndUV1 ;
          SmPoint2d   sStartUV2, sEndUV2 ;
          SmExtent1d  sIvl      = GetInterval() ;
          SmVector3d  sPVStart[2], sPVEnd[2] ;

          pSurface1->EvaluatePoint(sStartUV1, sStartPt1) ;
          pSurface1->EvaluatePoint(sEndUV1,   sEndPt1) ;
          pSurface2->EvaluatePoint(sStartUV2, sStartPt2) ;
          pSurface2->EvaluatePoint(sEndUV2,   sEndPt2) ;
          pCurve->Evaluate(sIvl.Evaluate(0.0), 1, TRUE, sPVStart) ; 
          pCurve->Evaluate(sIvl.Evaluate(1.0), 1, TRUE, sPVEnd) ; 
      
          // draw Brep and face/surface combinations
          smgfx_Erase() ;
          smgfx_SetLook(1,2, 0,0,1) ; if(pBrep)     pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
          smgfx_SetLook(1,2, 0,1,1) ; if(pSurface1) pSurface1->DrawUV() ; sm_GraphicsLoop() ;
          smgfx_SetLook(1,2, 0,0,0) ; if(pFace1)    pFace1->Draw(SM_DM_CROSSHATCH) ; sm_GraphicsLoop() ;
          smgfx_SetLook(1,2, 1,0,1) ; if(pSurface2) pSurface2->DrawUV() ; sm_GraphicsLoop() ;
          smgfx_SetLook(1,2, 0,0,0) ; if(pFace2)    pFace2->Draw(SM_DM_CROSSHATCH) ; sm_GraphicsLoop() ;

          // draw new edge, old curve with end tangents
          smgfx_SetLook(3,4, 1,0,1) ; this->Draw() ; sm_GraphicsLoop() ;  // new curve
          smgfx_SetLook(3,4, 0,1,1) ; if(pCurve) pCurve->Draw(&sIvl) ; sm_GraphicsLoop() ; // old curve
          smgfx_SetLook(5,6, 1,.5,0); sPVStart[1].Draw(&sPVStart[0]) ; sm_GraphicsLoop() ;
          smgfx_SetLook(5,6, 1,0,.5); sPVEnd[1].Draw(&sPVEnd[0]) ; sm_GraphicsLoop() ;

          // draw Vertex projected to surface points
          smgfx_SetLook(7,8, 1,0,0) ; sStartPt1.Draw() ; sm_GraphicsLoop() ;
          smgfx_SetLook(7,8, 1,0,0) ; sEndPt1.Draw() ;   sm_GraphicsLoop() ;
          smgfx_SetLook(7,8, 0,1,0) ; sStartPt2.Draw() ; sm_GraphicsLoop() ;
          smgfx_SetLook(7,8, 0,1,0) ; sEndPt2.Draw() ;   sm_GraphicsLoop() ;
          sm_GraphicsLoop() ;
        }
#endif // SM_DEBUG_CODE

      // clean up - delete Edge's original Curve
      if(pCurve) { delete pCurve ; pCurve = NULL ; }

    } // end a new valid intersection curve was built - use it check

  // all done
  return(SM_SUCCESS) ;

} // end SmEdge::RefineGeometry

/*********************************************************
PURPOSE: Calculate this Edge's EdgeForm as the intersection
         found between two faces to which this edge is connected 

NOTES: 
  moves the edge->vertex position by
  refining the vertice's current position down to
  an intersection curve when the 
  current Vertex position is not within ApproxTol3d
  of the intersecting face->surfaces.
**********************************************************/
SmStatus SmEdge::CalcEdgeFormFromFaces
 (SmBoolean        & rbFoundFaceEdgeForm, // out: TRUE = Edge's EdgeForm defined by Face/Face intersection, rpBestCurve found.
  double           & rdMaxGap3d,          // out: when rbFoundFaceEdgeForm == TRUE, max distance between rpBestCurve and Face->Surfaces.
  SmCurve         *& rpBestCurve,         // out: when rbFoundFaceEdgeForm == TRUE, the approximate Face/Face XSect Curve
  SmPoint3d        & rBestStartVertexPos, // out: when rbFoundFaceEdgeForm == TRUE, StartVertexPos refined to Face/Face XSect Curve
  SmPoint3d        & rBestEndVertexPos,   // out: when rbFoundFaceEdgeForm == TRUE, EndVertexPos   refined to Face/Face XSect Curve
  SmFace          ** pOptFace1,           // out: when rbFoundFaceEdgeForm == TRUE, Face1 of 2 used for XSect, NULL to ignore, default:[NULL]
  SmBSplineCurve  ** pOptUVTrimCurve1,    // out: when rbFoundFaceEdgeForm == TRUE, UVTrimCurve on Face1->Surface of new XSectCurve, NULL to ignore, default:[NULL]
  SmFace          ** pOptFace2,           // out: when rbFoundFaceEdgeForm == TRUE, Face2 of 2 used for XSect, NULL to ignore, default:[NULL]
  SmBSplineCurve  ** pOptUVTrimCurve2)    // out: when rbFoundFaceEdgeForm == TRUE, UVTrimCurve on Face2->Surface of new XSectCurve, NULL to ignore, default:[NULL]
const 
{
  // method
  //  1. build bounding box to help narrow search for intersections - make it slightly bigger than needed
  //  2. gather edgeuses
  //    2a. Cull edgeuse->m_tEdgeuseType == SmShell_TYPE
  //  3. iter every edgeuse looking for a non-tangent sector using SmEdgeuse::IsTangentSector()
  //  4. intersect edge sector surface pairs, restricted to the bounding box of step 1
  //     and restricted to vertex locations and current edge directions with
  //     SmSurface::PointBasedSurfaceIntersect()
  //  5. edit edge - replace edge, interval, and tolerance

  // init output
  rbFoundFaceEdgeForm = FALSE ;
  rdMaxGap3d          = 0.0 ;
  rpBestCurve         = NULL ; 
  if(pOptFace1)        { *pOptFace1        = NULL ; }
  if(pOptUVTrimCurve1) { *pOptUVTrimCurve1 = NULL ; }
  if(pOptFace2)        { *pOptFace2        = NULL ; }
  if(pOptUVTrimCurve2) { *pOptUVTrimCurve2 = NULL ; }
  
  // locals                          
  ULONG ii ;
  double       dDeviation = 0.0;
  SmCurve    * pCurve         = GetCurve() ;
  SmVector3d   sPVStart[2],     sPVEnd[2] ;
  SmVertex   * pStartVertex   = GetStartVertex() ;
  SmVertex   * pEndVertex     = GetOtherVertex(pStartVertex) ;
  SmPoint3d    sStartVertexPt = pStartVertex->GetPoint() ;
  SmPoint3d    sEndVertexPt   = pEndVertex->GetPoint() ;

  SmExtent1d   sIvl           = GetInterval() ;
  SmExtent3d   sBBox ;
  SmTArray<SmEdgeuse *> sEdgeuses ; 
  GetEdgeuses(sEdgeuses) ;

  // Get edge EndPoint Tangents
  pCurve->Evaluate(sIvl.Evaluate(0.0), 1, TRUE, sPVStart) ; sPVStart[1].Unitize() ;
  pCurve->Evaluate(sIvl.Evaluate(1.0), 1, TRUE, sPVEnd) ;   sPVEnd[1].Unitize() ;
  
  // find tolerances
  // SmZoneTol3d   sZoneTol3d   = SmTol::GetZoneTol3d(GetBrep()) ;      // default for this edge
  SmXSectTol3d  sXSectTol3d  = SmTol::GetXSectTol3d(GetBrep()) ;     // default for surf/surf intersections
  SmApproxTol3d sApproxTol3d = SmTol::GetApproxTol3d(GetBrep()) ;    // default for all objects

  // Check for Parallel Faces
  double      dAngTolDeg          = SM_ANG_TOL_DEG ;
  double      dMaxParallelFaceGap = 0.0 ; 
  double      dMinDihedralAngDeg  = SM_BIG_DOUBLE ;
  SmFace    * pFace1              = NULL ;  
  SmFace    * pFace2              = NULL ;  
  SmPoint2d   sUV1 ;       
  SmPoint2d   sUV2 ;  

  // get Parallel Face MaxGap3d (if any)
  SmBoolean bParallelFaces = HasParallelFaces
                              (&dAngTolDeg,          // in : optional max angle deg between parallel vectors, 
                                                     //      NULL=SM_ANG_TOL_DEG, default:[NULL]
                               &dMinDihedralAngDeg,  // out: Min Dihedral AngDeg seen between Faces
                               &dMaxParallelFaceGap, // out: Max gap between parallel face pairs
                               &pFace1,              // out: when Rtn == TRUE, optional face1 of MaxParallelFace/Face Gap pair
                               &pFace2,              // out: when Rtn == TRUE, optional face2 of MaxParallelFace/Face Gap pair
                               &sUV1,                // out: when Rtn == TRUE, optional face1 UVPt of MaxParallelFace/Face Gap pair
                               &sUV2) ;              // out: when Rtn == TRUE, optional face2 UVPt of MaxParallelFace/Face Gap pair

  // no work - parallel faces with significant offsets
  if(   bParallelFaces 
     && dMaxParallelFaceGap > sApproxTol3d)
    {
      // can't find a new Edge through intersections with small gaps to all faces
      rbFoundFaceEdgeForm = FALSE ;

      // all done
      return(SM_SUCCESS) ;

    } // end edge has parallel faces with large offset check

  // arrive here when the Edge is not connected to parallel faces with significant offsets

  // build a tolerant target bounding box slightly bigger than current curve and edge endPoints 
  pCurve->CalculateBoundingBox(sIvl, &sBBox) ;
  sBBox.AddPoint3d(sStartVertexPt) ;
  sBBox.AddPoint3d(sEndVertexPt) ;
  sBBox.ExpandAbsolute(5.0 * sXSectTol3d) ;

  // for every edgeuse - search for surface pair to intersect - look for nonTangent sectors
  for(ii=0;ii<sEdgeuses.GetSize();ii++)
    {
      SmEdgeuse *pEdgeuse = sEdgeuses[ii] ;

      // skip Edgeuse shells
      if(pEdgeuse->IsShellEdgeuse())
        { continue ; }

      // Every edge/face connection has two edgeuses - skip duplicate Opposite oriented edgeuses
      if(pEdgeuse->GetOrientation() == SM_OT_OPPOSITE)
        { continue ; }

      // skip tangent and cusp sectors
      if(   pEdgeuse->IsTangentSector(2.0, FALSE, FALSE) // FALSE=tangency check,   FALSE=don't make UVTrimCurves use 3D calcs if required
         || pEdgeuse->IsTangentSector(2.0, TRUE, FALSE)) // TRUE =coincidence check,FALSE=don't make UVTrimCurves use 3D calcs if required
        { continue ; }

      // arrive here when current edgeuse marks a sector which can be reintersected without tangency

      // locals for PointBasedSurfaceIntersection() call
      const SmContext * pContext  = GetContext() ;

      // set edgeuse1 = the edgeuse with SM_OT_SAME
      SmEdgeuse       * pEdgeuse1 = pEdgeuse ;
      SmEdgeuse       * pEdgeuse2 = pEdgeuse->GetRadial() ;
      SM_ASSERT(   pEdgeuse1->GetOrientation() == SM_OT_SAME
                && pEdgeuse2->GetOrientation() == SM_OT_OPPOSITE) ;

      pFace1    = pEdgeuse1->GetFace() ;
      pFace2    = pEdgeuse2->GetFace() ;
      SmSurface       * pSurface1 = pFace1->GetSurface() ;
      SmSurface       * pSurface2 = pFace2->GetSurface() ;


      // drop vertexPoints into SurfaceUV Points
      SmVertexuse * pVertexuseStart1 = pEdgeuse1->GetVertexuse() ;
      SmVertexuse * pVertexuseEnd1   = pEdgeuse1->GetMate()->GetVertexuse() ;
      SmVertexuse * pVertexuseEnd2   = pEdgeuse2->GetVertexuse() ;  // note: pVU2->Vertices swapped to line up with Start1 and End1
      SmVertexuse * pVertexuseStart2 = pEdgeuse2->GetMate()->GetVertexuse() ;
      SmPoint2d   sStartUV1, sEndUV1 ;
      SmPoint2d   sStartUV2, sEndUV2 ;

      // First surface:
      SmStatus sRtnS1 = pVertexuseStart1->ComputeUVPoint(sStartUV1,FALSE) ; // FALSE = don't make UVTrimCurve, if missing use GlobalSolve
      SmStatus sRtnE1 = pVertexuseEnd1  ->ComputeUVPoint(sEndUV1,  FALSE) ; // FALSE = don't make UVTrimCurve, if missing use GlobalSolve

      // skip cases where the vertex points failed to drop to the respective surfaces
      if(   sRtnS1 != SM_SUCCESS
         || sRtnE1 != SM_SUCCESS)
        { continue ; }

      // Check for lamina case: single face.
      if ( sEdgeuses.GetSize() == 2 )
        {
          SM_ASSERT( pFace1 == pFace2 );
          double dMaxDist = 0.0, dMaxDev = 0.0;
          SmApproxTol3d dTol = this->GetTolerance() * 10;  // Be generous
          SmExtent2d sDomain = pFace1->GetUVDomain();
          SmTArray< SmBSplineCurve* > sUVCurves;
          SmStatus eStat = pSurface1->DropCurve( *pContext, sDomain, *pCurve, sIvl, dTol,
              dMaxDist, dMaxDev, sUVCurves );

          if ( eStat != SM_SUCCESS || sUVCurves.GetSize() == 0 )
            { return SM_SUCCESS; }

          // Just work from the first one.
          // If it's a seam, then both uv curves will produce the same 3d geometry.
          // If it's not a seam, then its domain will be shorter, and we'll skip it.
          SmBSplineCurve *pUVCurve = sUVCurves[0];

          // Check that we got the whole curve.
          SmExtent1d sUVCurveDomain = pUVCurve->GetNaturalInterval();
          if ( ! sIvl.AreEqual( sUVCurveDomain, SM_EFF_ZERO ) )
            { return SM_SUCCESS; }

          // Create iso curve if possible.
          SmSurfParamType eWhichDir;
          double dParamValue;
          SmBoolean bIsIsoCurve = SmTrimmingTools::IsIsoCurve(
                                    pUVCurve,
                                    pSurface1,
                                    2.0 * pFace1->GetTolerance(),
                                    eWhichDir,
                                    dParamValue );

          SmBSplineCurve *pNew3dCurve = NULL;

          if ( bIsIsoCurve )
            {
              eStat = pSurface1->CreateIsoParametricCurve( *pContext,
                                      eWhichDir,
                                      dParamValue,
                                      sApproxTol3d, 
                                      pNew3dCurve );
            }

          if ( pNew3dCurve == NULL )
            {
              eStat = pSurface1->LiftCurve( *pContext,
                          sDomain,
                          *pUVCurve,
                          sUVCurveDomain,
                          sApproxTol3d,
                          dMaxDist,
                          pNew3dCurve,
                          TRUE ); // True == bSkipContinuityChecks
            }

          if ( eStat != SM_SUCCESS || pNew3dCurve == NULL )
            { return SM_SUCCESS; }

          // set output
          rbFoundFaceEdgeForm = TRUE ;
          rdMaxGap3d          = dMaxDist ;
          rpBestCurve         = pNew3dCurve ;
          pSurface1->EvaluatePoint( sStartUV1,rBestStartVertexPos );
          pSurface1->EvaluatePoint( sEndUV1,  rBestEndVertexPos   );
  
          if ( pOptFace1 )        { *pOptFace1        = pFace1;   }
          if ( pOptUVTrimCurve1 ) { *pOptUVTrimCurve1 = pUVCurve; }

          // all done
          return(SM_SUCCESS) ;

        } // end if lamina

      // Get uv params on the other surface.
      SmStatus sRtnS2 = pVertexuseStart2->ComputeUVPoint(sStartUV2,FALSE) ; // FALSE = don't make UVTrimCurve, if missing use GlobalSolve
      SmStatus sRtnE2 = pVertexuseEnd2  ->ComputeUVPoint(sEndUV2,  FALSE) ; // FALSE = don't make UVTrimCurve, if missing use GlobalSolve

      // skip cases where the vertex points failed to drop to the respective surfaces
      if(   sRtnS2 != SM_SUCCESS
         || sRtnE2 != SM_SUCCESS)
        { continue ; }

      // Get SurfEndPt-SurfEndPt Gap sizes
      SmPoint3d sStart3d1, sEnd3d1 ;       
      SmPoint3d sStart3d2, sEnd3d2 ;       
                                                
      // Map UVPoints to 3dPoints and get SurfPt-SurfPt gap sizes
      pSurface1->EvaluatePoint(sStartUV1, sStart3d1) ; 
      pSurface1->EvaluatePoint(sEndUV1,   sEnd3d1  ) ; 
      pSurface2->EvaluatePoint(sStartUV2, sStart3d2) ; 
      pSurface2->EvaluatePoint(sEndUV2,   sEnd3d2  ) ;

      // SurfPt-SurfPt gap sizes
      double dStartGap3d = (sStart3d2 - sStart3d1).Length() ;
      double dEndGap3d   = (sEnd3d2   - sEnd3d1  ).Length() ;

      // when needed - try to refine Start/End Vertex dropped UVPoints to Surf/Surf intersection points
      if(   dStartGap3d > sApproxTol3d
         || dEndGap3d   > sApproxTol3d)
        {
          // locals for refine Start XSectPt to surf/surf/plane XSect, plane:[VertexPt, CrvTangent] 
          SmBoolean  bFoundAnswer ;
          SmPoint2d  sUVs[2] ;
          SmPoint3d  sRefineStart3d1, sRefineStart3d2 ;
          SmPoint3d  sRefineEnd3d1,   sRefineEnd3d2 ;
          SmExtent2d sUVDomain1 = pSurface1->GetNaturalUVDomain() ;
          SmExtent2d sUVDomain2 = pSurface2->GetNaturalUVDomain() ;
          SmSolution sSol ;
          
            { // Refine Start XSect Surf UVPts
              
              // refine Start XSect Pt to surf/surf/plane XSect; plane:[VertexPt, CrvTangent]
              SmStatus sRtn = pSurface1->LocalPlaneSurfaceIntersect
                                          (sUVDomain1,       // in : this surface domain of interest
                                           *pSurface2,       // in : other surface
                                           sUVDomain2,       // in : other surface domain of interest
                                           sApproxTol3d,     // in : Max allowed distance between found Surface intersection points
                                           sStartUV1,        // in : this Surface initial UV guess
                                           sStartUV2,        // in : other Surface initial UV guess
                                           sStartVertexPt,   // in : origin of Plane(origin, normal)
                                           sPVStart[1],      // in : normal of Plane(origin, normal)
                                           bFoundAnswer,     // out: TRUE = found a solution
                                           sSol) ;           // out: when rbFoundAnser==TRUE, the solution
                                            
              // RefinePoint failures are usually parallel offset faces which can't be healed by reintersection - quit
              if(sRtn != SM_SUCCESS || bFoundAnswer == FALSE)
                { break ; }

              // Clamp sol to UVDomains and set output
              sUVs[0] = sUVDomain1.ClampPoint2d(SmPoint2d(sSol.m_vStart[0],sSol.m_vStart[1]));
              sUVs[1] = sUVDomain2.ClampPoint2d(SmPoint2d(sSol.m_vStart[2],sSol.m_vStart[3]));
          
              // save XSectPoint UVs for start edge
              sStartUV1 = sUVs[0] ;
              sStartUV2 = sUVs[1] ;

              // Map UVPoints to 3dPoints and get OrigPt-RefinePt move sizes
              pSurface1->EvaluatePoint(sStartUV1, sRefineStart3d1) ; 
              pSurface2->EvaluatePoint(sStartUV2, sRefineStart3d2) ; 

              double dStartMove3d1 = (sRefineStart3d1 - sStart3d1).Length() ;
              double dStartMove3d2 = (sRefineStart3d2 - sStart3d2).Length() ;

              // when moves are bigger than original gap size - (typically nearly parallel offset faces) - quit
              if(   dStartMove3d1 > 2.0*dStartGap3d    // 2.0 to allow some slop 
                 || dStartMove3d2 > 2.0*dStartGap3d)
                { break ; }
            } // end scope to refine Start XSect UVPts

            { // Refine End XSect Surf UVPts
              
              // refine End XSect Pt to surf/surf/plane XSect; plane:[VertexPt, CrvTangent]
              SmStatus sRtn = pSurface1->LocalPlaneSurfaceIntersect
                                          (sUVDomain1,     // in : this surface domain of interest
                                           *pSurface2,     // in : other surface
                                           sUVDomain2,     // in : other surface domain of interest
                                           sApproxTol3d,   // in : Max allowed distance between found Surface intersection points
                                           sEndUV1,        // in : this Surface initial UV guess
                                           sEndUV2,        // in : other Surface initial UV guess
                                           sEndVertexPt,   // in : origin of Plane(origin, normal)
                                           sPVEnd[1],      // in : normal of Plane(origin, normal)
                                           bFoundAnswer,   // out: TRUE = found a solution
                                           sSol) ;         // out: when rbFoundAnser==TRUE, the solution
                                            
              // RefinePoint failures are usually parallel offset faces which can't be healed by reintersection - quit
              if(sRtn != SM_SUCCESS || bFoundAnswer == FALSE)
                { break ; }

              // Clamp sol to UVDomains and set output
              sUVs[0] = sUVDomain1.ClampPoint2d(SmPoint2d(sSol.m_vEnd[0],sSol.m_vEnd[1]));
              sUVs[1] = sUVDomain2.ClampPoint2d(SmPoint2d(sSol.m_vEnd[2],sSol.m_vEnd[3]));
          
              // save XSectPoint UVs for start edge
              sEndUV1 = sUVs[0] ;
              sEndUV2 = sUVs[1] ;

              // Map UVPoints to 3dPoints and get OrigPt-RefinePt move sizes
              pSurface1->EvaluatePoint(sEndUV1, sRefineEnd3d1) ; 
              pSurface2->EvaluatePoint(sEndUV2, sRefineEnd3d2) ; 

              double dEndMove3d1 = (sRefineEnd3d1 - sEnd3d1).Length() ;
              double dEndMove3d2 = (sRefineEnd3d2 - sEnd3d2).Length() ;

              // when moves are bigger than original gap size - (typically nearly parallel offset faces) - quit
              if(   dEndMove3d1 > 2.0*dEndGap3d    // 2.0 to allow some slop 
                 || dEndMove3d2 > 2.0*dEndGap3d)
                { break ; }

            } // end scope to refine End XSect UVPts
        } // end refine Start and End Drop Point positions to surf/surf XSect
                                            
      // place UV Points into arrays         
      SmTArray<SmPoint2d> sUVPoints1, sUVPoints2 ;
      sUVPoints1.Add(sStartUV1) ;
      sUVPoints1.Add(sEndUV1) ;
      sUVPoints2.Add(sStartUV2) ;
      sUVPoints2.Add(sEndUV2) ;

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
      if(bDebugMe)
        {
          // check start point gaps
          SmPoint3d sStartPt1, sEndPt1 ;
          SmPoint3d sStartPt2, sEndPt2 ;
          pSurface1->EvaluatePoint(sStartUV1, sStartPt1) ;
          pSurface1->EvaluatePoint(sEndUV1,   sEndPt1) ;
          pSurface2->EvaluatePoint(sStartUV2, sStartPt2) ;
          pSurface2->EvaluatePoint(sEndUV2,   sEndPt2) ;

          double dGapStart = sStartPt1.DistanceBetween(sStartPt2) ;
          double dGapEnd   = sEndPt1.DistanceBetween(sEndPt2) ;

          SM_ASSERT( SM_IS_ZERO_TO_TOL(dGapStart, sXSectTol3d)) ;
          SM_ASSERT( SM_IS_ZERO_TO_TOL(dGapEnd,   sXSectTol3d)) ;

          SmBrep *pBrep = GetBrep() ;
      
          // draw Brep and face/surface combinations
          smgfx_Erase() ;
          smgfx_SetLook(3,4, 0,0,1) ; if(pBrep)     pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
          smgfx_SetLook(1,2, 0,1,1) ; if(pSurface1) pSurface1->DrawUV() ; sm_GraphicsLoop() ;
          smgfx_SetLook(1,2, 0,1,1) ; if(pSurface1) pSurface1->DrawParams() ; sm_GraphicsLoop() ;
          smgfx_SetLook(1,2, 0,0,0) ; if(pFace1)    pFace1->Draw(SM_DM_CROSSHATCH) ; sm_GraphicsLoop() ;
          smgfx_SetLook(1,2, 1,0,1) ; if(pSurface2) pSurface2->DrawUV() ; sm_GraphicsLoop() ;
          smgfx_SetLook(1,2, 1,0,1) ; if(pSurface2) pSurface2->DrawParams() ; sm_GraphicsLoop() ;
          smgfx_SetLook(1,2, 0,0,0) ; if(pFace2)    pFace2->Draw(SM_DM_CROSSHATCH) ; sm_GraphicsLoop() ;

          // draw edge with end tangents
          smgfx_SetLook(4,5, 1, 0, 1) ; this->Draw() ; sm_GraphicsLoop() ;
          smgfx_SetLook(6,7, 1, 1, 0) ; sPVStart[1].Draw(&sPVStart[0]) ; sm_GraphicsLoop() ;
          smgfx_SetLook(1,2, 0, 1, 0) ; sPVStart[1].DrawPlane(sStartVertexPt) ; sm_GraphicsLoop() ;
          smgfx_SetLook(5,6, 1,.65,0); sPVEnd[1].Draw(&sPVEnd[0]) ; sm_GraphicsLoop() ;
          smgfx_SetLook(1,2, 1,.65,0) ; sPVEnd[1].DrawPlane(sEndVertexPt  ) ; sm_GraphicsLoop() ;

          // draw Vertex projected to surface points
          smgfx_SetLook(8,9, 1,0,0) ; sStartPt1.Draw() ; sm_GraphicsLoop() ;
          smgfx_SetLook(8,9, 1,0,0) ; sEndPt1.Draw() ;   sm_GraphicsLoop() ;
          smgfx_SetLook(10,11, 0,1,0) ; sStartPt2.Draw() ; sm_GraphicsLoop() ;
          smgfx_SetLook(10,11, 0,1,0) ; sEndPt2.Draw() ;   sm_GraphicsLoop() ;
          sm_GraphicsLoop() ;
        }
#endif // SM_DEBUG_CODE

      // New intersection output objects
      SmBSplineCurve   * p3DCurve         = NULL ;        
      SmBSplineCurve   * pSurface1UVCurve = NULL ;
      SmBSplineCurve   * pSurface2UVCurve = NULL ;
      SmTsectCurveType   eCurveType ;      
      
      // Compute new intersection // Version that takes 2d uv parameter points:
      //   the algorithm fails when the sUVPoints1 and sUVPoints2 pairs are not within sApproxTol3d of one another
      SmStatus sRtn = pSurface1->PointBasedSurfaceIntersect
        (*pContext,     pFace1->GetUVDomain(),  // in : Context  // in : surf1 intersection domain limit 
         *pSurface2,    pFace2->GetUVDomain(),  // in : surf2    // in : surf2 intersection domain limit
          sUVPoints1,   sUVPoints2,             // in : Surf1 and Surf2 UV Points on intersection
          TRUE,         TRUE,                   // in : Allow extensions before and after start and last UVPts
          &sPVStart[1], &sPVEnd[1],             // in : pOptStartDirection, pOptEndDirection
          &sApproxTol3d,                        // in : pdOptApproxTol3d, NULL=1/1000 of surface size  
          NULL,                                 // in : pdOptAngTolRad, NULL=30 degrees
          p3DCurve,                 // out: resulting intersection - can be NULL for failure to find xSect
          pSurface1UVCurve,         // out: Surf1 UVTrimCurve                          
          pSurface2UVCurve,         // out: Surf2 UVTrimCurve                          
          eCurveType,               // out: type of intersection on intersection curve 
          dDeviation) ;             // out: max distance between rp3DCurve and surfaces

      SmObjDelete sClean3DCurve(p3DCurve) ;            
      SmObjDelete sClean1UVCurve(pSurface1UVCurve) ;     
      SmObjDelete sClean2UVCurve(pSurface2UVCurve) ;     

      SM_ASSERT(sRtn == SM_SUCCESS && p3DCurve != NULL) ;

      // skip failed intersections
      if(p3DCurve == NULL) 
        { continue ; }

      // skip intersection curves out of tolerance
      if(dDeviation > sXSectTol3d)
        { continue ; }

      // arrive here after building a new Curve shape for the Edge - save results
      sClean3DCurve.Clear() ;
      
#ifdef SM_DEBUG_CODE
      if(bDebugMe)
        {
          SM_DUMP_AND_ASSERT_VALID(p3DCurve) ;

          // check start point gaps
          SmPoint3d sCurveStartPV[2], sCurveEndPV[2] ;
          SmExtent1d sCurveIvl = p3DCurve->GetNaturalInterval() ;

          p3DCurve->Evaluate(sCurveIvl.Evaluate(0.0), 1, TRUE, sCurveStartPV) ;
          p3DCurve->Evaluate(sCurveIvl.Evaluate(1.0), 1, TRUE, sCurveEndPV) ;

          SmBoolean bSuccessStart1, bSuccessEnd1, bSuccessStart2, bSuccessEnd2 ;
          double    dGapStart1, dGapEnd1, dGapStart2, dGapEnd2 ;
          SmBoolean bIsMulti;
          SmPoint2d sStartDropUV1, sEndDropUV1 ;
          SmPoint2d sStartDropUV2, sEndDropUV2 ;
          SmPoint3d sStartPt1, sEndPt1 ;
          SmPoint3d sStartPt2, sEndPt2 ;

          pSurface1->DropPoint(sCurveStartPV[0], pSurface1->GetNaturalUVDomain(), &sStartUV1, bSuccessStart1,
                               sStartDropUV1, dGapStart1, bIsMulti);
          pSurface1->DropPoint(
              sCurveEndPV[0], pSurface1->GetNaturalUVDomain(), &sEndUV1, bSuccessEnd1, sEndDropUV1, dGapEnd1, bIsMulti);

          pSurface2->DropPoint(sCurveStartPV[0], pSurface2->GetNaturalUVDomain(), &sStartUV2, bSuccessStart2,
                               sStartDropUV2, dGapStart2, bIsMulti);
          pSurface2->DropPoint(
              sCurveEndPV[0], pSurface2->GetNaturalUVDomain(), &sEndUV2, bSuccessEnd2, sEndDropUV2, dGapEnd2, bIsMulti);

          pSurface1->EvaluatePoint(sStartDropUV1, sStartPt1) ;
          pSurface1->EvaluatePoint(sEndDropUV1,   sEndPt1) ;
          pSurface2->EvaluatePoint(sStartDropUV2, sStartPt2) ;
          pSurface2->EvaluatePoint(sEndDropUV2,   sEndPt2) ;

          double dGapStart = sStartPt1.DistanceBetween(sStartPt2) ;
          double dGapEnd   = sEndPt1.DistanceBetween(sEndPt2) ;

          SM_ASSERT( SM_IS_ZERO_TO_TOL(dGapStart, sXSectTol3d)) ;
          SM_ASSERT( SM_IS_ZERO_TO_TOL(dGapEnd,   sXSectTol3d)) ;

          SmBrep *pBrep = GetBrep() ;
      
          // draw Brep and face/surface combinations
          smgfx_Erase() ;
          smgfx_SetLook(1,2, 0,0,1) ; if(pBrep)     pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
          smgfx_SetLook(1,2, 0,1,1) ; if(pSurface1) pSurface1->DrawUV() ; sm_GraphicsLoop() ;
          smgfx_SetLook(1,2, 0,0,0) ; if(pFace1)    pFace1->Draw(SM_DM_CROSSHATCH) ; sm_GraphicsLoop() ;
          smgfx_SetLook(1,2, 1,0,1) ; if(pSurface2) pSurface2->DrawUV() ; sm_GraphicsLoop() ;
          smgfx_SetLook(1,2, 0,0,0) ; if(pFace2)    pFace2->Draw(SM_DM_CROSSHATCH) ; sm_GraphicsLoop() ;

          // draw edge with end tangents
          smgfx_SetLook(3,4, 1,0,1) ; if(p3DCurve) p3DCurve->Draw() ; sm_GraphicsLoop() ;
          smgfx_SetLook(5,6, 1,.5,0); sCurveStartPV[1].Draw(&sCurveStartPV[0]) ; sm_GraphicsLoop() ;
          smgfx_SetLook(5,6, 1,0,.5); sCurveEndPV[1].Draw(&sCurveEndPV[0]) ; sm_GraphicsLoop() ;

          // draw Vertex projected to surface points
          smgfx_SetLook(7,8, 1,0,0) ; sStartPt1.Draw() ; sm_GraphicsLoop() ;
          smgfx_SetLook(7,8, 1,0,0) ; sEndPt1.Draw() ;   sm_GraphicsLoop() ;
          smgfx_SetLook(9,10, 0,1,0) ; sStartPt2.Draw() ; sm_GraphicsLoop() ;
          smgfx_SetLook(9,10, 0,1,0) ; sEndPt2.Draw() ;   sm_GraphicsLoop() ;
          sm_GraphicsLoop() ;
        }
#endif // SM_DEBUG_CODE

      // set output
      rbFoundFaceEdgeForm = TRUE ;
      rdMaxGap3d          = dDeviation ;
      rpBestCurve         = p3DCurve ;
      pSurface1->EvaluatePoint(sStartUV1,rBestStartVertexPos) ;
      pSurface1->EvaluatePoint(sEndUV1,  rBestEndVertexPos) ;
  
      if(pOptFace1)        { *pOptFace1        = pFace1 ; }
      if(pOptUVTrimCurve1) { *pOptUVTrimCurve1 = pSurface1UVCurve ; 
                              sClean1UVCurve.Clear() ;
                           }
      if(pOptFace2)        { *pOptFace2        = pFace2 ; }
      if(pOptUVTrimCurve2) { *pOptUVTrimCurve2 = pSurface2UVCurve ; 
                              sClean2UVCurve.Clear() ;
                           }

       // all done
       return(SM_SUCCESS) ;

    } // end iter every edgeuse looking for a pair of surfaces to intersect

  // arrive here when no appropriate geometry was found to reintersect 
  return(SM_SUCCESS) ;

} // end SmEdge::CalcEdgeFormFromFaces

/*******************************************************************//**
PURPOSE: check for and compute max Parallel Face<->Edge<->Face gap 

NOTES: rtn TRUE when Faces parallel anywhere along the Edge
       when Rtn == TRUE sets output rMaxParallelFaceGap = largest parallel Face/Face Gap seen
       when Rtn == FALSE sets output rMaxParallelFaceGap = 0.0 ;
***********************************************************************/
SmBoolean SmEdge::CalcMaxParallelFaceGap
 (SmFace    & rFace1,                 // in : face1 of MaxParallelFace/Face Gap pair
  SmFace    & rFace2,                 // in : face2 of MaxParallelFace/Face Gap pair
  double    & rMinDihedralAngDeg,     // out: Min Dihedral AngDeg seen between Faces
  double    & rMaxParallelFaceGap,    // out: Max gap between parallel face pairs
  double    * pOptAngTolDeg,          // in : optional max angle deg between parallel vectors, 
                                      //      NULL=SM_ANG_TOL_DEG, default:[NULL]
  SmPoint2d * pOptUV1,                // out: when Rtn == TRUE, optional face1 UVPt of MaxParallelFace/Face Gap pair
  SmPoint2d * pOptUV2)                // out: when Rtn == TRUE, optional face2 UVPt of MaxParallelFace/Face Gap pair
 const
{
  // init return and output
  SmBoolean bRtn = FALSE ;
  rMinDihedralAngDeg  = SM_BIG_DOUBLE ;
  rMaxParallelFaceGap = 0.0 ; 
  if(pOptUV1) { pOptUV1->SetUninitialized() ; }
  if(pOptUV2) { pOptUV2->SetUninitialized() ; }

  // locals
  SmEdgeuse *pEdgeuse1 = GetEdgeuseOfFace(&rFace1) ; 
  SmEdgeuse *pEdgeuse2 = GetEdgeuseOfFace(&rFace2) ;
  double     dAngTolDeg = pOptAngTolDeg ? *pOptAngTolDeg : SM_ANG_TOL_DEG ;
  double     dAngRad = 0.0, dAngDeg = 0.0;

  // no work - no EdgeFace Gap
  if(   pEdgeuse1 == NULL || !pEdgeuse1->HasEdgeFaceGap() 
     || pEdgeuse2 == NULL || !pEdgeuse2->HasEdgeFaceGap())
    { return( FALSE ) ; }

  // check state
  SM_ASSERT_MSG(   rFace1.GetSurface() != NULL
                && rFace2.GetSurface() != NULL
                && GetCurve() != NULL,
                _T("SmEdgeuse::CalcMaxParallelFaceGap: Edge not connected to a Curve and two surfaces for given faces")) ;

  // Locals
#ifdef SM_DEBUG_CODE
  ULONG lMaxIndx = 0;
#endif

  SmCurve         * pEdgeCurve = GetCurve() ;
  SmSurface       * pSurface1  = rFace1.GetSurface() ;
  SmSurface       * pSurface2  = rFace2.GetSurface() ;
  SmPoint3d         sNorm1, sNorm2 ;
  const SmContext * pContext   =   GetContext()        ? GetContext()
                                 : rFace1.GetContext() ? rFace1.GetContext()
                                 : rFace2.GetContext() ;

  // gwc: avoid infinite loop - don't call SmTol::GetXSectTol3d(pEdge,pFace) which comes back to this function.
  // gwc: Tol, a temp value, can be any number, only used for sEdgeFaceGF construction then deleted.
  SmXSectTol3d        sXSectTol3d  = SmTol::GetXSectTol3d(pContext) ;
                   
  // construct gap functions
  ULONG               lSampleCnt = 20 ;
  SmExtent1d          sInterval  = GetInterval();
  SmCrvSrfGapFunction sEdgeFace1GF( sXSectTol3d, pEdgeCurve, sInterval, pSurface1, lSampleCnt) ; 
  SmCrvSrfGapFunction sEdgeFace2GF( sXSectTol3d, pEdgeCurve, sInterval, pSurface2, lSampleCnt) ; 

  // get sample arrays - forces sEdgeFaceGF lazy evaluations
  SmTArray<SmGapSample> &rSampleSet1 = sEdgeFace1GF.GetSampleSet() ; SM_ASSERT(lSampleCnt == rSampleSet1.GetSize()) ;
  SmTArray<SmGapSample> &rSampleSet2 = sEdgeFace2GF.GetSampleSet() ; SM_ASSERT(lSampleCnt == rSampleSet2.GetSize()) ;

  AER_MSG( lSampleCnt == rSampleSet1.GetSize() && lSampleCnt == rSampleSet2.GetSize(),
           _T( "Couldn't build gap sample sets with expected sample count" ) );

  // for every Sample pair - check for Parallel faces and save max Parallel Face/Face Gap3d
  for(ULONG ii=0;ii<lSampleCnt;ii++)
    {
      SmPoint2d sUV1 = rSampleSet1[ii].GetOtherParam() ;
      SmPoint3d sPt1 = rSampleSet1[ii].GetOtherPos() ;
      pSurface1->EvaluateNormal(sUV1, TRUE, TRUE, sNorm1) ;

      SmPoint2d sUV2 = rSampleSet2[ii].GetOtherParam() ;
      SmPoint3d sPt2 = rSampleSet2[ii].GetOtherPos() ;
      pSurface2->EvaluateNormal(sUV2, TRUE, TRUE, sNorm2) ;

      // dihedral angle between faces at sample point
      sNorm1.AngleBetween(sNorm2, dAngRad) ;
      dAngDeg = SM_RAD2DEG(dAngRad) ;

      if(rMinDihedralAngDeg > dAngDeg)
        { rMinDihedralAngDeg = dAngDeg ; }

      // when Face Normals are parallel
      if(sNorm1.IsParallelTo(sNorm2, dAngTolDeg))
        {
          bRtn = TRUE ;

          // Save largest parallel Face/Face gap seen
          double dGap3d = sPt1.DistanceBetween(sPt2) ;

          if(rMaxParallelFaceGap <= dGap3d)
            { 
              rMaxParallelFaceGap = dGap3d ; 
#ifdef SM_DEBUG_CODE
              lMaxIndx = ii;
#endif
              if(pOptUV1) { *pOptUV1 = sUV1 ; }
              if(pOptUV2) { *pOptUV2 = sUV2 ; }
            }
        } // end Parallel Faces check
    } // done iterating all Face/Face GapSample pairs

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  // draw 
  if(bDebugMe)
    {
      sEdgeFace1GF.Dump() ;
      sEdgeFace2GF.Dump() ;
      
      SmBrep *pBrep = GetBrep() ;
      SmPoint3d sCurvePt = rSampleSet1[lMaxIndx].GetThisPos() ;
      SmPoint3d sSurf1Pt = rSampleSet1[lMaxIndx].GetOtherPos() ;
      SmPoint3d sSurf2Pt = rSampleSet2[lMaxIndx].GetOtherPos() ;
      SmPoint2d sUV1     = rSampleSet1[lMaxIndx].GetOtherParam() ;
      SmPoint2d sUV2     = rSampleSet2[lMaxIndx].GetOtherParam() ;
      pSurface1->EvaluateNormal(sUV1, TRUE, TRUE, sNorm1) ;
      pSurface2->EvaluateNormal(sUV2, TRUE, TRUE, sNorm2) ;

                      
      smgfx_Erase() ;
      smgfx_SetLook(1,2,  0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(5,7,  0,0,1) ; sCurvePt.Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook(7,10, 1,0,0) ; sSurf1Pt.Draw() ; sNorm1.Draw(&sSurf1Pt) ; sm_GraphicsLoop() ;
      smgfx_SetLook(7,12, 0,1,0) ; sSurf2Pt.Draw() ; sNorm2.Draw(&sSurf2Pt) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2,  0,0,0) ; rFace1.Draw(SM_DM_CROSSHATCH) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2,  0,0,0) ; rFace2.Draw(SM_DM_CROSSHATCH) ; sm_GraphicsLoop() ;
      smgfx_SetLook(3,4,  1,0,1) ; this->Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook(3,4,  1,0,1) ; this->DrawParams() ; sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE
            
  // all done - return TRUE = found parallel GapSamples, FALSE=didn't
  return(bRtn) ;
  
} // end SmEdge::CalcMaxParallelFaceGap

/*******************************************************************//**
PURPOSE: For a manifold edge in a manifold brep, returns exterior angle 
         between faces, in radians

NOTES: If the Edge is not manifold, angle returned without warranty.
       If the Brep is not manifold, then the obtuse angle is returned. 
       In case of the above, return status is SM_ERR_WARNING

       Advanced users can use SmEdgeuse::CalcSectorAngle for precise results.
***********************************************************************/
SmStatus SmEdge::CalcOuterAngle ( 
  double    dParam,     ///< [in] :Edge parameter at which angle is calculated <br>
  double & rdAngle      ///< [out]:Outer angle at dParam in radians            <br>
) const
{
    rdAngle = SM_UNDEF_DOUBLE;
    SmEdgeuse *pPrimaryEU = GetPrimaryEdgeuse();
    SmEdgeuse *pMateEU = pPrimaryEU->GetMate();
    SmEdgeuse *pEU;
    SmStatus   eRtn = SM_SUCCESS;

    SmBoolean bPrimaryEUIsInfinite = pPrimaryEU->GetShell()->GetRegion()->IsInfiniteRegion();
    SmBoolean bMateEUIsInfinite    = pMateEU->GetShell()->GetRegion()->IsInfiniteRegion();

    // Want the Edgeuse on the InfiniteRegion side of the face
    if ( bPrimaryEUIsInfinite == bMateEUIsInfinite )
    { 
        eRtn = SM_ERR_WARNING; 
        pEU  = pPrimaryEU;
    }
    else if ( bPrimaryEUIsInfinite )
    { pEU = pPrimaryEU; }
    else
    { pEU = pMateEU; }

    // Calculate the sector angle through the infinite region
    SER( pEU->CalcSectorAngle( dParam, rdAngle ) );

    // In case of error, return the obtuse angle
    if ( eRtn == SM_ERR_WARNING && rdAngle < SM_PI )
    { rdAngle = SM_2PI - rdAngle; }

    return eRtn;

} // end SmEdge::CalcOuterAngle

/*********************************************************
PURPOSE: Set m_vInteval according to dropping our vertices to our curve.
   To be used if the curve parameterization changes.
**********************************************************/
SmStatus SmEdge::UpdateDomain()
{
  SmExtent1d sThisDomain = this->GetInterval();
  SmExtent1d sNewDomain  = sThisDomain;

  // use whole curve domain: could be outside of our domain.
  SmExtent1d sCrvDomain = m_pCurve->GetNaturalInterval();
  SmBoolean bSuccess;
  double dNewParam0, dNewParam1, dDist;
  SmStatus eStat;
  SmVertex *pV0, *pV1;

  this->GetVertices( pV0, pV1 );
  double dGuessParam = sThisDomain.GetMin(); // Should be very close to this.
  SmXSectTol3d sTol = SmTol::GetXSectTol3d( SmTol::GetZoneTol3d(pV0), SmTol::GetZoneTol3d( m_pCurve ) );

  eStat = m_pCurve->DropPoint( sCrvDomain,       // in : target curve allowed domain
                               pV0->GetPoint(),  // in : Point to drop to curve
                               NULL,             // in : For Snapping, NULL=NoSnap, Vec pointing to CrvInside from desired EndPt for snapping.
                                                 //      when DropPt is within dDistTol in cpVecToCrvInside dir of EndPt, Snap to Ivl EndPt.
                                                 //      Vec handles ambiguities and makes sure that curves are not just touching at the ends.
                               sTol,             // in : Used for SnapDist, and SM_SO_INTERSECT sXSectTol3d distance.
                                                 //      sXSectTol3d = dDistTol ;      SM_SO_INTERSECT 
                                                 //      sXSectTol3d = SM_BIG_DOUBLE ; SM_SO_MINIMIZE, SM_SO_NORMALIZE
                                                 //      sSnapTol3d  = dDistTol ;      SM_SO_MINIMIZE, SM_SO_NORMALIZE, SM_SO_INTERSECT    
                               &dGuessParam,     // in : If given, call cheap LocalPointSolve() else call expensive GlobalPointSolve()
                               bSuccess,         // out: TRUE = found a drop point
                               dNewParam0,       // out: found drop curve param
                               dDist) ;          // out: found drop distance
                                                 // in : SM_SO_MINIMIZE = allow nonNormal drops near endPoints
                                                 //      SM_SO_INTERSECT= like Minimize but point must be within dDistTol
                                                 //      SM_SO_NORMALIZE= exclude nonNormal drops near endPoints
                                                 //      default:[SM_SO_MINIMIZE] to preserve original behavior

  if ( eStat == SM_SUCCESS && bSuccess )
    {
      SM_ASSERT( dDist <= sTol );
      sNewDomain.SetMinMax( dNewParam0, sNewDomain.GetMax() );
    }
  else
    { WARN( _T("Bad Vertex drop to Edge curve\n") ); }

  dGuessParam = sThisDomain.GetMax();
  sTol = SmTol::GetXSectTol3d( SmTol::GetZoneTol3d(pV1), SmTol::GetZoneTol3d( m_pCurve ) );

  eStat = m_pCurve->DropPoint(sCrvDomain,      // in : target curve allowed domain
                              pV1->GetPoint(), // in : Point to drop to curve
                              NULL,            // in : For Snapping, NULL=NoSnap, Vec pointing to CrvInside from desired EndPt for snapping.
                                               //      when DropPt is within dDistTol in cpVecToCrvInside dir of EndPt, Snap to Ivl EndPt.
                                               //      Vec handles ambiguities and makes sure that curves are not just touching at the ends.
                              sTol,            // in : Used for SnapDist, and SM_SO_INTERSECT sXSectTol3d distance.
                                               //      sXSectTol3d = dDistTol ;      SM_SO_INTERSECT 
                                               //      sXSectTol3d = SM_BIG_DOUBLE ; SM_SO_MINIMIZE, SM_SO_NORMALIZE
                                               //      sSnapTol3d  = dDistTol ;      SM_SO_MINIMIZE, SM_SO_NORMALIZE, SM_SO_INTERSECT    
                              &dGuessParam,    // in : If given, call cheap LocalPointSolve() else call expensive GlobalPointSolve()
                              bSuccess,        // out: TRUE = found a drop point
                              dNewParam1,      // out: found drop curve param
                              dDist) ;         // out: found drop distance
                                               // in : SM_SO_MINIMIZE = allow nonNormal drops near endPoints
                                               //      SM_SO_INTERSECT= like Minimize but point must be within dDistTol
                                               //      SM_SO_NORMALIZE= exclude nonNormal drops near endPoints
                                               //      default:[SM_SO_MINIMIZE] to preserve original behavior

  if ( eStat == SM_SUCCESS && bSuccess )
    {
      SM_ASSERT( dDist <= sTol );
      sNewDomain.SetMinMax( sNewDomain.GetMin(), dNewParam1 );
    }
  else
    { WARN( _T("Bad Vertex drop to Edge curve\n") ); }

  if ( !sNewDomain.AreEqual( sThisDomain ) )
  { RemoveUVTrimCurves(); }

  this->SetInterval( sNewDomain );

  // all done
  return(SM_SUCCESS) ;

} // end SmEdge::UpdateDomain

/*******************************************************************//**
PURPOSE: Receive notification of things happening to the object and
take appropriate actions.  

  NOTES: For example cleaning up the cache of an
object which is being deleted or edited.  Also clean up any attributes
and relations specific to this class which are not handled automatically
by construtors.
***********************************************************************/
void SmEdge::Notify                    // expected calls: caller->Notify(Event, pData1, pData2, pData2)                                   
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
                                       // SM_NO_DESTRUCTION          | DelObj      | DelObj   |  NULL                 |  NULL                     // 
{
  switch (eNotifyOperation) 
    {
      case SM_NO_PRE_EDIT              : break ;
                                       
      case SM_NO_CHANGE_GEOMETRY       : {                        
                                           SmBrep *pBrep = ( pData2 != NULL ) ? SM_CAST_PTR( SmBrep, pData2 ) : GetBrep();
                                           if ( pBrep != NULL )
                                             {
                                               pBrep->Notify( SM_NO_RM_FROM_BREP, pData3, pBrep, this->GetCurve() );
                                               pBrep->Notify( SM_NO_ADD_TO_BREP , pData1, pBrep, this->GetCurve() );
                                             }                    
                                         }                        
      case SM_NO_POST_EDIT             : 
      case SM_NO_SPLIT_IN_BREP         : 
      case SM_NO_MERGE_IN_BREP         : // pass the notify call along to all Vertexuses, Edgeuses, and Edgeuse->Vertexuses
                                         // so that those can clean up their cached gaps
                                         {
                                           ULONG ii ;
                                           SM_PTR_ARRAY(sVertexuses, SmVertexuse, 32) ;
                                           SM_PTR_ARRAY(sEdgeuses,   SmEdgeuse, 32) ;
                                           GetEdgeuses(sEdgeuses) ;
                                           GetVertexuses(sVertexuses) ;
                                           for(ii=0;ii<sEdgeuses.GetSize();ii++)
                                             { sEdgeuses[ii]->Notify(eNotifyOperation, pData1, pData2, pData3) ; }
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

} // end SmEdge::Notify

/*******************************************************************//**
PURPOSE: Set orientation of edgeuses belonging to the edge.

NOTES: Assumes edgeuses are listed in radial order
    and assigns eOrientation to the 1st edgeuse and 
        the opposite of eOrienation to its mate 
    and so on for all edgeuse/mate pairs.
***********************************************************************/
void SmEdge::SetOrientation
  (SmOrientType eOrientation)
{
  SmEdgeuse *pData[32];
  SmTArray<SmEdgeuse*> sEdgeuses(32,pData);

  // for every edgeuse/mate pair
  GetEdgeuses(sEdgeuses);
  for (ULONG i=1; i<sEdgeuses.GetSize(); i+=2) 
    {
      SmEdgeuse *pEU1 = (SmEdgeuse*)sEdgeuses[i-1];
      SmEdgeuse *pEU2 = (SmEdgeuse*)sEdgeuses[i];

      // assign eOrietnation to 1st edgeuse
      pEU1->SetOrientation(eOrientation);

      // and opposite orientation to its mate
      switch(eOrientation)
        {
          case SM_OT_SAME     : pEU2->SetOrientation(SM_OT_OPPOSITE); break ;
          case SM_OT_OPPOSITE : pEU2->SetOrientation(SM_OT_SAME);     break ;
          default             : pEU2->SetOrientation(SM_OT_UNKNOWN);  break ;
        }
    } // end iter every edgeuse/mate pair

} // end SmEdge::SetOrientation

/*******************************************************************//**
PURPOSE: Reverse Orientation of Edge keeping EU data consistent.

NOTES: Assumes edgeuses are ordered and oriented for current CurveParameterization.
       Reverses Curve Parameterization,
       Negates Edgeuse orientations
       Deletes uv trim curves.
         Note, we could try reversing them, but domain intervals can get messed up
         on some analytic curves.
***********************************************************************/
void SmEdge::ReverseOrientation()
{
  // locals
  SmExtent1d sNewIvl, sOldIvl = m_vInterval;

  // reverse Curve Parameterization
  if(m_pCurve) 
    { m_pCurve->ReverseParameterization( sOldIvl, sNewIvl ); }

  m_vInterval = sNewIvl;

  // invert EU orientations, and any uv trim curves.
  SmEdgeuse * pStart = (SmEdgeuse*)this->m_pList ;
  SmEdgeuse * pEU    = pStart ;
  do
    {
      // negate Edgeuse orientation
      SmOrientType eOrient =   pEU->GetOrientation() == SM_OT_SAME ? SM_OT_OPPOSITE
                             : pEU->GetOrientation() == SM_OT_OPPOSITE ? SM_OT_SAME
                             : SM_OT_UNKNOWN ; 
      pEU->SetOrientation(eOrient) ; 

      pEU->SetUVTrimCurve( NULL, 0.0, TRUE ) ;  // TRUE = delete existing UVTrimCurve

      // Proceed to next in list.
      pEU = (SmEdgeuse*)pEU->GetNext() ; 

    } while ( pEU != pStart );

  // all done

} // end SmEdge::ReverseOrientation  

/*******************************************************************//**
PURPOSE: Given a sequence of Edges joined end-to-end, make all orientations
   in the same direction as the sequence.

NOTES: Assumes edgeuses are ordered end to end.
***********************************************************************/
SmStatus SmEdge::OrientEdgeSequence( SmTArray< SmEdge* > & rEdgeList )
{
  ULONG lNumEdges = rEdgeList.GetSize(); 
  if ( lNumEdges <2 )
    { return SM_SUCCESS; }

  // To order the first Edge, we have to look at the second Edge.
  SmEdge *pEdge0 = rEdgeList[0];
  SmEdge *pEdge1 = rEdgeList[1];

#ifdef SM_DEBUG_CODE
  SmBoolean bDebugMe=FALSE;
  if ( bDebugMe ) {
      if ( FALSE )
        { smgfx_Erase(); }
      sm_GraphicsLoop();
      smgfx_SetLook( 3,5, 1,0,0 ); pEdge0->DrawParams(); sm_GraphicsLoop();
      smgfx_SetLook( 3,5, 0,1,0 ); pEdge1->DrawParams(); sm_GraphicsLoop();
      sm_GraphicsLoop();
  }
#endif // SM_DEBUG_CODE

  SmVertex *pV00, *pV01, *pV10, *pV11;
  pV00 = pEdge0->GetStartVertex();
  pV01 = pEdge0->GetEndVertex();
  pV10 = pEdge1->GetStartVertex();
  pV11 = pEdge1->GetEndVertex();

  if ( pV00 == pV10  ||  pV00 == pV11 )
    {
      // Start of first matches start or end of the second:
      // Reverse the first -- unless it's a closed two-edge sequence.
      if ( lNumEdges == 2 )
        {
          if ( pV00 == pV11  &&  pV01 == pV10 )
            { return SM_SUCCESS; } // already in order.

          if ( pV00 == pV10  &&  pV01 == pV11 )
            {
              pEdge1->ReverseOrientation();
              return SM_SUCCESS;
            }
        }

      // Not a closed 2-Edge sequence.
      pEdge0->ReverseOrientation();
      pV00 = pEdge0->GetStartVertex();
      pV01 = pEdge0->GetEndVertex();
    }

  // Now the first edge should be oriented correctly.
  // Check the second, since we already have it.

  if ( pV10 == pV01 )
    {}  // it's in order
  else if ( pV11 == pV01 )
    {
      pEdge1->ReverseOrientation();
      pV10 = pEdge1->GetStartVertex();
      pV11 = pEdge1->GetEndVertex();
    }
  else
    { return SM_ERR; }

  // Now the rest of them.
  for ( ULONG ii=2; ii<lNumEdges; ii++ )
    {
      pEdge0 = rEdgeList[ ii-1 ];
      pEdge1 = rEdgeList[ ii   ];
      pV01 = pEdge0->GetEndVertex();
      pV10 = pEdge1->GetStartVertex();
      pV11 = pEdge1->GetEndVertex();

#ifdef SM_DEBUG_CODE
      if ( bDebugMe ) {
          if ( FALSE )
            { smgfx_Erase(); }
          smgfx_SetLook( 3,5, 1,0,0 ); pEdge0->DrawParams(); sm_GraphicsLoop();
          smgfx_SetLook( 3,5, 0,1,0 ); pEdge1->DrawParams(); sm_GraphicsLoop();
          sm_GraphicsLoop();
      }
#endif // SM_DEBUG_CODE

      if ( pV01 == pV10 )
        {}  // it's in order
      else if ( pV01 == pV11 )
      {
          pEdge1->ReverseOrientation();
          pV10 = pEdge1->GetStartVertex();
          pV11 = pEdge1->GetEndVertex();
      }
      else
        { return SM_ERR; }

    }  // end for the rest of the filleted edges.

  return SM_SUCCESS;

} // end SmEdge::OrientEdgeSequence

/*******************************************************************//**
PURPOSE: Swap the order of two input Edgeuses on the Edge's radial list
of Edgeuses.  Used solely as a supporting function for SmLoop::FlipLoopOrientation()

RETURNS: SM_ERR when Edgeuse1 and Edgeuse2 are not a mated pair on the same Edge

NOTE:  This didn't really work out - the Edgeuse radial list has to be oriented 
so that the members alternate directions as [ ... SAME OPPOSITE SAME ... ]
starting on either SAME or OPPOSITE.  The Edgeuse radial list has to be organized
in mated pairs, i.e. [1st Mated Pair, 2nd MatedPair, ... Nth MatedPair].

A function like this one may be needed in the future - but for now
this one is not going to be use.
***********************************************************************/
//  SmStatus SmEdge::SwapEdgeuseOrder
//   (SmEdgeuse & rEdgeuse1,   // in : Edgeuse of this Edge to swap on the Edge->RadialMateEdgeuseList
//    SmEdgeuse & rEdgeuse2)   // in : Edgeuse of this Edge to swap on the Edge->RadialMateEdgeuseList
//  {
//    // locals
//    SmEdge * pEdge1 = rEdgeuse1.GetEdge() ;
//    SmEdge * pEdge2 = rEdgeuse2.GetEdge() ;
//  
//    // check input
//    SM_ASSERT_MSG(   pEdge1 == this 
//                  && pEdge2 == this
//                  && rEdgeuse1.GetMate() == &rEdgeuse2
//                  && rEdgeuse2.GetMate() == &rEdgeuse1,
//                  _T("SmEdge::SwapEdgeuseOrder: input Edgeuses are not a mated Pair on the Edge->RadialMateEdgeuseList")) ; 
//  
//    if(   pEdge1 != this 
//       || pEdge2 != this
//       || rEdgeuse1.GetMate() != &rEdgeuse2
//       || rEdgeuse2.GetMate() != &rEdgeuse1)
//      { return SM_ERR ; }
//  
//    // locals
//    ULONG ii ; 
//    ULONG nIndx1, nIndx2 ; 
//    SmTArray<SmEdgeuse *> sEdgeuses ;
//    
//    // get edgeuses in their current order (depends on GetEdgeuses to return edgeuses in order)
//    GetEdgeuses(sEdgeuses) ; 
//  
//    // find the indices of the input Edgeuses
//    SmBoolean bFind1 = sEdgeuses.FindElement(&rEdgeuse1, nIndx1) ;
//    SmBoolean bFind2 = sEdgeuses.FindElement(&rEdgeuse2, nIndx2) ;
//  
//  #ifdef SM_DEBUG_CODE
//  SmBoolean bDebugMe = FALSE ;
//  static ULONG lCount = 1 ; lCount++ ;
//  static ULONG lDebugCount = 0 ;
//    if(bDebugMe || lDebugCount == lCount)
//      {
//        SM_DUMP_AND_ASSERT_VALID(this) ;
//        ULONG di ; 
//        SmBrep * pBrep = GetBrep() ;
//  
//        smgfx_Erase() ;
//        smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
//        smgfx_SetLook(2,3, 1,0,0) ; if(this)  this->Draw() ; sm_GraphicsLoop() ;
//        smgfx_SetLook(5,6, 0,1,0) ; rEdgeuse1.Draw() ; sm_GraphicsLoop() ;
//        smgfx_SetLook(5,6, 0,1,1) ; rEdgeuse2.Draw() ; sm_GraphicsLoop() ; 
//        smgfx_SetLook(3,4, 1,0,1) ; for(di=0;di<sEdgeuses.GetSize();di++) { if(sEdgeuses[di]) sEdgeuses[di]->Draw(.5) ; } sm_GraphicsLoop() ; 
//        sm_GraphicsLoop() ; 
//      }
//  #endif // SM_DEBUG_CODE
//  
//    // swap the Edgeuses in sEdgeuses array
//    SmEdgeuse *pTmp = sEdgeuses[nIndx1] ;
//    sEdgeuses[nIndx1] = sEdgeuses[nIndx2] ;
//    sEdgeuses[nIndx2] = pTmp ; 
//  
//    // reset the m_pList pointer (really only needed when nIndx1 or nIndx2 == 0)
//    m_pList = sEdgeuses[0] ; 
//    SmOrientType ePrimOrient = sEdgeuses[0]->GetOrientation() ; 
//  
//    // fix up all the Edgeuse Next/Last pointers
//    ULONG lSize = sEdgeuses.GetSize() ;
//    for(ii=0;ii<lSize;ii++)
//      {
//        // note: if(m_eOrientation == OwningEdge->PrimaryEdgeUse->m_eorientation)
//        //            {mate = m_pNext ; radial = m_pLast ; }
//        //       else {mate = m_pLast ; radial = m_pNext ; }
//        if(ePrimOrient == sEdgeuses[ii]->GetOrientation())
//          {
//            sEdgeuses[ii]->m_pNext = sEdgeuses[(ii+1)%lSize] ;
//            sEdgeuses[ii]->m_pLast = sEdgeuses[(ii+lSize-1)%lSize] ; 
//          }
//        else
//          {
//            sEdgeuses[ii]->m_pNext = sEdgeuses[(ii+lSize-1)%lSize] ;
//            sEdgeuses[ii]->m_pLast = sEdgeuses[(ii+1)%lSize] ; 
//          }
//      } // end iter every Edgeuse setting Next and Last Pointers based on orientation
//  
//  #ifdef SM_DEBUG_CODE
//    // after all is done, rEdgeuse1 and rEdgeuse2 should still be a mated pair
//    SM_ASSERT_MSG(   rEdgeuse1.GetMate() == &rEdgeuse2
//                  && rEdgeuse2.GetMate() == &rEdgeuse1,
//                  _T("SmEdge::SwapEdgeuseOrder: after edit, Edgeuses are no longer a mated Pair on the Edge->RadialMateEdgeuseList")) ; 
//    if(bDebugMe || lDebugCount == lCount)
//      {
//        SM_DUMP_AND_ASSERT_VALID(this) ;
//        ULONG di ;
//        SmBrep * pBrep = GetBrep() ;
//  
//        smgfx_Erase() ;
//        smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
//        smgfx_SetLook(2,3, 1,0,0) ; if(this)  this->Draw() ; sm_GraphicsLoop() ;
//        smgfx_SetLook(5,6, 0,1,0) ; rEdgeuse1.Draw() ; sm_GraphicsLoop() ;
//        smgfx_SetLook(5,6, 0,1,1) ; rEdgeuse2.Draw() ; sm_GraphicsLoop() ; 
//        smgfx_SetLook(3,4, 1,0,1) ; for(di=0;di<sEdgeuses.GetSize();di++) { if(sEdgeuses[di]) sEdgeuses[di]->Draw(.5) ; } sm_GraphicsLoop() ; 
//        sm_GraphicsLoop() ; 
//      }
//  #endif // SM_DEBUG_CODE
//  
//    // all done
//    return(SM_SUCCESS) ;
//    
//  } // end SmEdge::SwapEdgeuseOrder

/*******************************************************************//**
PURPOSE: Get the faces which are bounded by this edge.

NOTES: The resulting list contains a unique set of faces
   which surround this edge if any.
***********************************************************************/
void SmEdge::GetFaces
 (SmTArray<SmFace*> & rFaces,           // out: list of all faces attached to edge
  ULONG             * pOptAttributeId)  // in : only include objects containing an attribute with this id
                                        //      NULL to ignore, default:[NULL] 
 const
{
  // init output
  rFaces.ReSet();

  // locals
  SmEdgeuse *pData[32];
  SmTArray<SmEdgeuse*> sEdgeuses(32,pData);

  // for every loop edgeuse
  GetEdgeuses(sEdgeuses);
  for (ULONG i=0; i<sEdgeuses.GetSize(); i++) 
    {
      SmEdgeuse *pEU = (SmEdgeuse*)sEdgeuses[i];
      if (pEU->IsLoopEdgeuse()) 
        {
          // see if face is already in output
          SmFace    *pFace  = pEU->GetFace();
          // Skip dangling loop-typed edgeuses (SmLoopuse_TYPE but m_pSorLU==NULL,
          // so GetFace()==NULL). Edge splitting during tessellation can transiently
          // produce these; emitting NULL here would crash callers that dereference
          // the returned faces (e.g. MakeVertexSplitEdge's face-notify loop).
          if (pFace == NULL) { continue ; }
          SmBoolean  bFound = FALSE;
          for (ULONG j=0; j<rFaces.GetSize(); j++) 
            {
              if (pFace == rFaces[j]) 
                {
                  bFound = TRUE; 
                  break;
                }
            }

          // add unique faces to output
          if(   !bFound
             && (   pOptAttributeId == NULL 
                 || pFace->FindAttribute(*pOptAttributeId) != NULL))
            {
              rFaces.Add(pFace);
            }
        }
    } // end iter every face

} // end SmEdge::GetFaces

/*******************************************************************//**
PURPOSE: Get the Shells which are connected to this edge.

NOTES: The resulting list contains a unique set of shells
   which surround this edge if any.
***********************************************************************/
void SmEdge::GetShells
 (SmTArray<SmShell*> & rShells,         // out: list of all Shells attached to edge
  ULONG             * pOptAttributeId)  // in : only include objects containing an attribute with this id
                                        //      NULL to ignore, default:[NULL] 
 const
{
  // init output
  rShells.ReSet();

  // locals
  SmEdgeuse *pData[32];
  SmTArray<SmEdgeuse*> sEdgeuses(32,pData);

  // for every loop edgeuse
  GetEdgeuses(sEdgeuses);
  for (ULONG i=0; i<sEdgeuses.GetSize(); i++) 
    {
      SmEdgeuse * pEU    = (SmEdgeuse*)sEdgeuses[i];
      SmShell   * pShell = pEU->GetShell() ; 

      // add unique faces to output
      if(   pOptAttributeId == NULL 
         || pShell->FindAttribute(*pOptAttributeId) != NULL)
        {
          rShells.AddUnique(pShell) ;
        } // end matching AttributeId check
    } // end iter every Edgeuse
} // end SmEdge::GetShells

/*******************************************************************//**
PURPOSE: Get the Regions which are connected to this edge.

NOTES: The resulting list contains a unique set of Regions
   which surround this edge if any.
***********************************************************************/
void SmEdge::GetRegions
 (SmTArray<SmRegion*> & rRegions,         // out: list of all Regions attached to edge
  ULONG             * pOptAttributeId)  // in : only include objects containing an attribute with this id
                                        //      NULL to ignore, default:[NULL] 
 const
{
  // init output
  rRegions.ReSet();

  // locals
  SmEdgeuse *pData[32];
  SmTArray<SmEdgeuse*> sEdgeuses(32,pData);

  // for every loop edgeuse
  GetEdgeuses(sEdgeuses);
  for (ULONG i=0; i<sEdgeuses.GetSize(); i++) 
    {
      SmEdgeuse * pEU     = (SmEdgeuse*)sEdgeuses[i] ;
      SmRegion  * pRegion = pEU->GetShell()->GetRegion() ; 

      // add unique faces to output
      if(   pOptAttributeId == NULL 
         || pRegion->FindAttribute(*pOptAttributeId) != NULL)
        {
          rRegions.AddUnique(pRegion) ;
        } // end matching AttributeId check
    } // end iter every Edgeuse
} // end SmEdge::GetRegions

/*******************************************************************//**
PURPOSE: Get Max Vertex/Edge or Edge/Face Gap

NOTES: 
***********************************************************************/
const SmGap * SmEdge::GetMaxGap3d
 (SmGapArray * pOptGapArray, // out: optional List of all Vertex/Edge and Edge/Face Gap3ds, NULL to ignore, default:[NULL]
  SmTol3d    * pOptTol3d)    // in : NotNULL        = only load Gaps larger than OptTol3d into OptGapArray.
                             //      default:[NULL] = load all Gaps into OptGapArray
 const
{ 
  // init output
  if(pOptGapArray) { pOptGapArray->ReSet() ; }

  // locals
  SmGapArray sThisGapArray, * pThisGapArray = pOptGapArray ? &sThisGapArray : NULL ;

  // UpDim gaps
  SmGap * pMaxUpDimGap3d   = (SmGap *)GetMaxUpDimGap3d(pThisGapArray, pOptTol3d) ;        
  
  // build optional array
  if(pOptGapArray) { pOptGapArray->Append(*pThisGapArray) ; }

  // DownDim gaps
  SmGap * pMaxDownDimGap3d = (SmGap *)GetMaxDownDimGap3d(pThisGapArray, pOptTol3d) ; 

  // build optional array
  if(pOptGapArray) { pOptGapArray->Append(*pThisGapArray) ; }

  // set output
  SmGap* pRtnGap =  (*pMaxUpDimGap3d > *pMaxDownDimGap3d)   
                   ?  pMaxUpDimGap3d 
                   :  pMaxDownDimGap3d ;
                 
  // all done
  return(pRtnGap) ;

} // end SmEdge::GetMaxGap3d

/*******************************************************************//**
PURPOSE: Get the vertices of an edge.

NOTES: When there are two vertices - adds start vertex 1st
***********************************************************************/
void SmEdge::GetVertices
 (SmTArray<SmVertex*> & rVertices,        // out: list of found topology objects
  ULONG               * pOptAttributeId)  // in : only include objects containing an attribute with this id
                                          //      NULL to ignore, default:[NULL] 
 const
{
  rVertices.ReSet();
  SmVertex *pV1=NULL, *pV2=NULL ;

  SmEdgeuse* pPrimaryEU = GetPrimaryEdgeuse();
  if(pPrimaryEU == NULL)
    return;

  if(pPrimaryEU->GetOrientation() == SM_OT_SAME)
       { pV1 = GetPrimaryEdgeuse()->GetVertexuse()->GetVertex();
         pV2 = GetPrimaryEdgeuse()->GetMate()->GetVertexuse()->GetVertex();
       }
  else { pV2 = GetPrimaryEdgeuse()->GetVertexuse()->GetVertex();
         pV1 = GetPrimaryEdgeuse()->GetMate()->GetVertexuse()->GetVertex();
       }

  if(   pOptAttributeId == NULL 
     || pV1->FindAttribute(*pOptAttributeId) != NULL)
    { rVertices.Add(pV1); }

  if(   (pV2 != NULL && pV2 != pV1) 
     && (   pOptAttributeId == NULL 
         || pV2->FindAttribute(*pOptAttributeId) != NULL))
    { rVertices.Add(pV2); }

} // end SmEdge::GetVertices

/*******************************************************************//**
PURPOSE: Get the vertex uses of an edge.

NOTES: adds start vertex 1st
***********************************************************************/
void SmEdge::GetVertexuses
(SmTArray<SmVertexuse*> & rVertexuses,      // out: list of objects
 ULONG                  * pOptAttributeId)  // in : only include objects containing an attribute with this id
                                            //      NULL to ignore, default:[NULL]
const 
{
  // init output
  rVertexuses.ReSet();

  // locals
  SmVertexuse *pVu1=NULL, *pVu2=NULL ;

  // when Edge is connected to Vertexuses
  if(GetList() != NULL && GetPrimaryEdgeuse())
    {
      if(GetPrimaryEdgeuse()->GetOrientation() == SM_OT_SAME) 
        {
          pVu1 = GetPrimaryEdgeuse()->GetVertexuse() ;
          pVu2 = GetPrimaryEdgeuse()->GetMate()->GetVertexuse() ;
        } 
      else 
        {
          pVu2 = GetPrimaryEdgeuse()->GetVertexuse() ;
          pVu1 = GetPrimaryEdgeuse()->GetMate()->GetVertexuse() ;
        }

      if(   (pOptAttributeId == NULL)
         || (pVu1->FindAttribute(*pOptAttributeId))) 
        { rVertexuses.Add(pVu1) ; }

      if(   (pOptAttributeId == NULL)
         || (pVu2->FindAttribute(*pOptAttributeId))) 
        { rVertexuses.Add(pVu2) ; }
    } // end connected to vertexuses check

} // end SmEdge::GetVertexuses

/*******************************************************************//**
PURPOSE: Return the Faces ordered around the Edge, starting with
  the Face of the primary Edgeuse.

NOTES: The first face will be that of the primary edgeuse.
***********************************************************************/
SmStatus SmEdge::GetFacesOrdered( SmTArray<SmFace*> & rFaces )
  const
{
  rFaces.ReSet();

  NER( this );

  SmEdgeuse *pStartEU = this->GetPrimaryEdgeuse();

  SmFace *pThisFace = pStartEU->GetFace();
  rFaces.Add( pThisFace );

  SmEdgeuse *pThisEU  = pStartEU->GetMate()->GetRadial();

  do
  {
      pThisFace = pThisEU->GetFace();
      rFaces.Add( pThisFace );

      pThisEU = pThisEU->GetMate()->GetRadial();

  } while ( pThisEU != pStartEU );

  return SM_SUCCESS;

} // end SmEdge::GetFacesOrdered

/*******************************************************************//**
PURPOSE: Get the edgeuses of an edge.

NOTES:
***********************************************************************/
SmStatus SmEdge::GetEdgeuses
(SmTArray<SmEdgeuse*> & rEdgeuses,        ///< [out]: list of objects
 ULONG                * pOptAttributeId,  ///< [in] : only include objects containing an attribute with this id,
                                          ///<        NULL to ignore, default:[NULL]
 const SmTopology     * pOptConnectedTo)  ///< [in] : only include objects connected to this Topo,
                                          ///<        NULL to ignore, default:[NULL]
const
{ 
  ULONG lCount = 0;
  rEdgeuses.SetSize(m_lListSize);
  rEdgeuses.ReSet();

  // for nonNULL m_pList pointers
  if(m_pList != NULL)
    {
      // add all m_pList->m_pNext elements to rEdgeuses
      SmEdgeuse *pElem = (SmEdgeuse*)m_pList;

      // keep walking circular List back to head element
      if(   pOptAttributeId == NULL
         && pOptConnectedTo == NULL)
        {
          // add each elem to rEdgeuses
          do{ 
              // add elem and step to Next object
              rEdgeuses.Add(pElem);
              pElem = (SmEdgeuse*)pElem->m_pNext;

              // check m_lListSize
              lCount ++;
              if(lCount > m_lListSize)
                { SER(SM_ERR) ; }

            } while (pElem && (pElem != m_pList));

          SM_ASSERT(m_lListSize == rEdgeuses.GetSize());
        } // end no attribute filtering branch
      else if(pOptAttributeId != NULL)
        { // add each elem with desired attribute to rEdgeuses
          do{ 
              if(   pElem->FindAttribute(*pOptAttributeId)
                 && (pOptConnectedTo == NULL || pElem->IsConnectedTo(pOptConnectedTo))) 
                { rEdgeuses.Add(pElem) ; }
              
              pElem = (SmEdgeuse*)pElem->m_pNext;

            } while (pElem && (pElem != m_pList));

        } // end only attribute filtering branch
      else 
        { // pOptAttributeId == NULL, pOptConnectedTo != NULL
          // add each elem that connects to the target
          do{ 
              if(pElem->IsConnectedTo(pOptConnectedTo))
                { rEdgeuses.Add(pElem) ; }
              
              pElem = (SmEdgeuse*)pElem->m_pNext;

            } while (pElem && (pElem != m_pList));

        } // end only target filtering branch
    } // end m_pList != NULL check

  // all done
  return SM_SUCCESS;

} // end SmEdge::GetEdgeuses

SmStatus SmEdge::GetEdgeuses(std::unordered_set<SmEdgeuse*>& rEdgeuses) const
{
    ULONG lCount = 0;

    // for nonNULL m_pList pointers
    if (m_pList != NULL)
    {
        // add all m_pList->m_pNext elements to rEdgeuses
        SmEdgeuse* pElem = (SmEdgeuse*)m_pList;

        // keep walking circular List back to head element
        // add each elem to rEdgeuses
        do
        {
            // add elem and step to Next object
            rEdgeuses.insert(pElem);
            pElem = (SmEdgeuse*)pElem->m_pNext;

            // check m_lListSize
            lCount++;
            if (lCount > m_lListSize)
            {
                SER(SM_ERR);
            }

        } while (pElem && (pElem != m_pList));
    } // end m_pList != NULL check

    // all done
    return SM_SUCCESS;
}

/*******************************************************************//**
PURPOSE: Get the number of times this edge bounds this face.  

NOTES: 
    Typically the result is 0 or 1 
    except for seam edges or edges inside of a face where it is 2.  
    It is not impossible but is very unlikely to get more than 2.
***********************************************************************/
ULONG SmEdge::GetNumberFaceOccurances // rtn: number of times edge bounds this face
  (const SmFace * pFace)              // in : target face
 const
{
  // init return value
  ULONG lCount = 0;

  // locals
  SmEdgeuse *pData[32];
  SmTArray<SmEdgeuse*> sEdgeuses(32,pData);

  // get upwardFaceuse
  SmFaceuse *pFU = pFace->GetUpwardFaceuse();

  // for every Edge-Edgeuse
  GetEdgeuses(sEdgeuses);
  for (ULONG i=0; i<sEdgeuses.GetSize(); i++) 
    {
      SmEdgeuse *pEU = (SmEdgeuse*)sEdgeuses[i];

      // count all edgeuses connected to the UpwardFaceuse
      if (pEU->GetFaceuse() == pFU) 
        {
          lCount ++;
        }
    }

  // all done
  return lCount;

} // end SmEdge::GetNumberFaceOccurances

/*************************************************************
PURPOSE:  return edge's largest Edge/Face gap
             or NULL for no gaps(not connected)
NOTES: Consistent Tolerant Model uses Edge/Face gaps
         but does not use the Edge/FaceTrimCurve gaps
         because problem databases often can't build valid UVTrimCurves
**************************************************************/
const SmGap * SmEdge::GetMaxUpDimGap3d
 (SmGapArray * pOptGapArray, // out: optional List of all Vertex/Edge and Edge/Face Gap3ds, NULL to ignore, default:[NULL]
  SmTol3d    * pOptTol3d)    // in : NotNULL        = only load Gaps larger than OptTol3d into OptGapArray.
                             //      default:[NULL] = load all Gaps into OptGapArray
 const
{
  // init output
  if(pOptGapArray) { pOptGapArray->ReSet() ; }

  // locals
  ULONG ii ;
  SmGap * pMaxGap = NULL ;
  SM_PTR_ARRAY(sEdgeuses, SmEdgeuse, 16) ;                                  
  GetEdgeuses(sEdgeuses) ;

  // for every edgeuse
  for(ii=0;ii<sEdgeuses.GetSize();ii++)
    {
      // Max Edge/Face gap
      SmGap * pThisMaxEdgeFaceGap = sEdgeuses[ii]->GetMaxEdgeFaceGap() ;

      if(pThisMaxEdgeFaceGap)
        {
          // build Gap List
          if(pOptGapArray && (pOptTol3d == NULL || pThisMaxEdgeFaceGap->GetLength() > pOptTol3d->val))
            { pOptGapArray->Add(pThisMaxEdgeFaceGap) ; }

          // set output
          if(pMaxGap == NULL || *pMaxGap < *pThisMaxEdgeFaceGap) 
            { pMaxGap = pThisMaxEdgeFaceGap ; }

        }  // end MaxEdgeFaceGap existence check
    } // end iter every edgeuse

  // all done
  return(pMaxGap) ;

} // end SmEdge::GetMaxUpDimGap3d

/*************************************************************
PURPOSE:  return edge's largest Vertex/Edge gap
             or NULL for no gaps(not connected)
NOTES:
**************************************************************/
const SmGap * SmEdge::GetMaxDownDimGap3d  
 (SmGapArray * pOptGapArray, // out: optional List of all Vertex/Edge and Edge/Face Gap3ds, NULL to ignore, default:[NULL]
  SmTol3d    * pOptTol3d)    // in : NotNULL        = only load Gaps larger than OptTol3d into OptGapArray.
                             //      default:[NULL] = load all Gaps into OptGapArray
 const
{
  // init output
  if(pOptGapArray) { pOptGapArray->ReSet() ; }

  // locals
  SmGap * pMaxGap = NULL ;
  SM_PTR_ARRAY(sVertexuses, SmVertexuse, 16) ;                                  
  GetVertexuses(sVertexuses) ;

  // when the Edge is connected to vertices - return largest Vertex/Edge gap
  if( sVertexuses.GetSize() > 0 )
    {
      // Vertex/Edge Gaps
      SmGap * pVEGap0 = sVertexuses[0]->GetVertexEdgeGap() ;    
      SmGap * pVEGap1 = sVertexuses[1]->GetVertexEdgeGap() ;  
      
      // build Gap List
      if(pOptGapArray)
        {  
          if(pVEGap0 && (pOptTol3d == NULL || pVEGap0->GetLength() > pOptTol3d->val)) { pOptGapArray->Add(pVEGap0) ; }
          if(pVEGap1 && (pOptTol3d == NULL || pVEGap1->GetLength() > pOptTol3d->val)) { pOptGapArray->Add(pVEGap1) ; }
        }

      // set output
      if(pVEGap0 && pVEGap1)
        { pMaxGap = ( *pVEGap0 > *pVEGap1 ) ? pVEGap0 : pVEGap1 ; }
    }

  // all done
  return(pMaxGap) ;

} // end SmEdge::GetMaxDownDimGap3d

/*******************************************************************//**
PURPOSE: Add a pair of oriented edgeuses into the list of edgeuses
   contained by the edge.  

NOTES: 1. the edgeuses alternate in their orientations.
       2. All these edgeuses have to be owned by the same edge
***********************************************************************/
SmStatus SmEdge::AddOrientedEUPair
  (SmEdgeuse *pEU1,        // in : 1st edgeuse of mated pair being added
   SmEdgeuse *pEU2,        // in : 2nd edgeuse of mated pair being added
   SmEdgeuse *pEUSector)   // in : Specifies radial sector in which
                           //      to insert the pair.
     
{
  // set pInsertAfter with edgeuse to precede the mated input pair
  SmEdgeuse *pInsertAfter = NULL;
  if (IsLamina()) 
    {
      SmEdgeuse *pPrimaryEU = GetPrimaryEdgeuse();
      pInsertAfter = pPrimaryEU->GetMate();
    }
  else // insert mated pair between a pair of radial mates - not between a pair of side mates
    {
      SmEdgeuse *pEUNext = (SmEdgeuse*)pEUSector->m_pNext;
      if (pEUNext == pEUSector->GetRadial()) 
        {
          pInsertAfter = pEUSector;
        }
      else 
        {
          pInsertAfter = pEUSector->GetRadial();
        }
    }

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  // draw 
  if(bDebugMe)
    {
      SmEdge *pEdge = pInsertAfter->GetEdge() ;
      SmBrep *pBrep = pEdge ? pEdge->GetBrep() : NULL ;

      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(FALSE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(3,4, 0,1,1) ; if(pEdge) pEdge->Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook(3,4, 1,0,0) ; pInsertAfter->Draw() ; sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

  // insert mated pair after InsertAfter so that orientations of edge-Edgeuses oscillate
  if (pInsertAfter->GetOrientation() == pEU1->GetOrientation()) 
    {
      // EU1 and pInsertAfter are the same orient, put EU2 next to it
      InsertAfter(pEU1,pInsertAfter);  // make list {pInsertAfter, pEU1}
      InsertAfter(pEU2,pInsertAfter);  // make list {pInsertAfter, pEU2, pEU1}
    }
  else 
    {  
      // EU1 and pInsertAfter are opposite EU1, put EU1 next to it
      InsertAfter(pEU2,pInsertAfter);  // make list {pInsertAfter, pEU2}
      InsertAfter(pEU1,pInsertAfter);  // make list {pInsertAfter, pEU1, pEU2}
    }

  // all done
  return SM_SUCCESS;

} // end SmEdge::AddOrientedEUPair

/*******************************************************************//**
PURPOSE: Get the vertex belonging to the edge's primary edgeuse. 
    - when primaryEdgeuse->Orientation == SM_OT_SAME
           this vertex marks the edge->curve's start.
      else this vertex marks the edge->curve's end.

NOTES: 
***********************************************************************/
SmVertex * SmEdge::GetVertex() const 
{
    return GetPrimaryEdgeuse()->GetVertexuse()->GetVertex(); 
 
} // end SmEdge::GetVertex

/*******************************************************************//**
PURPOSE: Get the edge->vertex marking the edge->curve start.

NOTES: 
***********************************************************************/
SmVertex * SmEdge::GetStartVertex() const 
{ 
  return( (GetPrimaryEdgeuse()->GetOrientation() == SM_OT_SAME)
         ? GetPrimaryEdgeuse()->GetVertexuse()->GetVertex()
         : GetPrimaryEdgeuse()->GetMate()->GetVertexuse()->GetVertex()) ;

} // end SmEdge::GetStartVertex

/*******************************************************************//**
PURPOSE: Get the edge->vertex marking the edge->curve end.  

NOTES: 
***********************************************************************/
SmVertex * SmEdge::GetEndVertex() const 
{ 
  return(  (GetPrimaryEdgeuse()->GetOrientation() == SM_OT_SAME)
         ? GetPrimaryEdgeuse()->GetMate()->GetVertexuse()->GetVertex()
         : GetPrimaryEdgeuse()->GetVertexuse()->GetVertex() ) ;

} // end GetOtherVertex

/*******************************************************************//**
PURPOSE: Get the vertex at the opposite end of the edge relative to
   the input vertex.  

NOTES: Note that it may be the same vertex as input in
   cases where edge is a self-loop.
   The mate is the corresponding oppositely oriented edge on the other
    side of the face

***********************************************************************/
SmVertex * SmEdge::GetOtherVertex
  (const SmVertex * pVertex) 
 const 
{ 
	SmVertex* pV1 = nullptr;
	SmVertex* pV2 = nullptr;
  if (GetPrimaryEdgeuse())
  { 
      SmEdgeuse* pEdgeuse = GetPrimaryEdgeuse();
      if (pEdgeuse->GetVertexuse())
      {
          pV1 = GetPrimaryEdgeuse()->GetVertexuse()->GetVertex();
      }
      if (pEdgeuse->GetMate())
      {
          SmEdgeuse* pMate = pEdgeuse->GetMate();
          if (pMate->GetVertexuse())
          {
              pV2 = GetPrimaryEdgeuse()->GetMate()->GetVertexuse()->GetVertex();
          }
      }
  }

  if ( pV1 == NULL || pV2 == NULL )
  { 
      SE( SM_ERR ); 
      return NULL;
  }

  if      (pV1 == pVertex) { return pV2; }   
  else if (pV2 == pVertex) { return pV1; }
  SE(SM_ERR);
  return NULL;

} // end GetOtherVertex

/*******************************************************************//**
PURPOSE: Get Edge start and end vertices

NOTES: 
***********************************************************************/
void SmEdge::GetVertices
 (SmVertex *& rpStartVertex,    // out: start vertex
  SmVertex *& rpEndVertex)      // out: end vertex
 const  
{
  if(GetPrimaryEdgeuse()->GetOrientation() == SM_OT_SAME)
    { 
      rpStartVertex = GetPrimaryEdgeuse()->GetVertexuse()->GetVertex() ;
      rpEndVertex   = GetPrimaryEdgeuse()->GetMate()->GetVertexuse()->GetVertex() ;
    }
  else
    {
      rpEndVertex   = GetPrimaryEdgeuse()->GetVertexuse()->GetVertex() ;
      rpStartVertex = GetPrimaryEdgeuse()->GetMate()->GetVertexuse()->GetVertex() ;
    }

} // end SmEdge::GetVertices

/*******************************************************************//**
PURPOSE: Insert one end of the edge into a vertex on a face - not for public use.   

NOTES: 
    The input Vertexuse must be on the primary side of a face.  
    The input Vertexuse must be either a single loop vertexuse 
    or an Edgeuse vertex where the Vertexuse belongs to the 
    sector's CCWEdge on the face where the input Edge is being inserted. 
    
    This is not a public method and should only be used
    by some of the topological operators.  This method may leave things
    in a somewhat invalid state if the vertex is a loop vertexuse.
***********************************************************************/
SmStatus SmEdge::InsertManifoldEdgeEnd
  (SmVertexuse * pVertexuse,      // in : TgtVertexuse to connect to Edge
   SmBoolean     bIsStartVertex)  // in : TRUE = connect Vertexuse as Edge->Starting Vertexuse
                                  //      FALSE= connect Vertexuse as Edge->End Vertexuse
{
  // locals
  SM_PTR_ARRAY(sEdgeuses, SmEdgeuse, 16) ; 
  GetEdgeuses(sEdgeuses);

  // check state: ThisEdge expected to be a manifold edge with 4 Edgeuses
  if (sEdgeuses.GetSize() != 4) SER(SM_ERR_INVALID_INPUT);

  // Named Edgeuses
  SmEdgeuse *pEUTop1, *pEUTop2;
  SmEdgeuse *pEUBot1, *pEUBot2;

  // Edges are arranged in order with edgeuse pairs on different sides of the same Face.
  //   sEdgeuses[0] and sEdgeuses[1] are a mated pair
  //   sEdgeuses[2] and sEdgeuses[4] are a mated pair
  //
  //   sEdgeuses[0] and sEdgeuses[2] have the same directions
  //   sEdgeuses[1] and sEdgeuses[4] share the opposite direction
  if (bIsStartVertex) { pEUTop1 = sEdgeuses[0];
                        pEUBot1 = sEdgeuses[1];
                        pEUBot2 = sEdgeuses[2];
                        pEUTop2 = sEdgeuses[3];
                      }
  else                { pEUTop2 = sEdgeuses[0];
                        pEUBot2 = sEdgeuses[1];
                        pEUBot1 = sEdgeuses[2];
                        pEUTop1 = sEdgeuses[3];
                      }

  // locals (same oriented EUs starting at pVertexuse->Vertex from different sides of the Edge)
  SmVertexuse * pVUTop = pEUTop1->GetVertexuse();
  SmVertexuse * pVUBot = pEUBot2->GetVertexuse();

  // TgtVertex
  SmVertex *pV = pVertexuse->GetVertex();

  // Insert new start vertexuses into TgtVertex
  pV->PostInsert(pVUTop);
  pV->PostInsert(pVUBot);

  // When connecting to a LoopVertex - make a Strut edge
  if (pVertexuse->IsLoopVertexuse()) 
    {
      // Now Link up Edgeuses around strut end
      pEUTop1->m_pCW  = pEUTop2 ; pEUTop2->m_pCCW = pEUTop1 ;
      pEUBot1->m_pCCW = pEUBot2 ; pEUBot2->m_pCW  = pEUBot1 ;

    } // end connecting to a LoopVertex branch

  else // connecting to a EdgeuseVertex (ShellVertex not possible - TgtVertex is on a Face) 
    {
      // This one is somewhat more complex in that we have to do some
      // work to insert the new edge into the loop.
      SmEdgeuse *pOldEUTop1 = pVertexuse->GetEdgeuse(); // OldVertex ThisEdgeuse
      SmEdgeuse *pOldEUTop2 = pOldEUTop1->m_pCW;        // OldVertex PrevEdgeuse
      SmEdgeuse *pOldEUBot1 = pOldEUTop1->GetMate();    
      SmEdgeuse *pOldEUBot2 = pOldEUTop2->GetMate();    

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
      if(bDebugMe) 
        {
          smgfx_Erase();
          smgfx_SetLook(1,2, 0,0,0) ; pOldEUTop1->Draw(); sm_GraphicsLoop();
          smgfx_SetLook(2,3, 0,0,1) ; pOldEUBot1->Draw(); sm_GraphicsLoop();
          smgfx_SetLook(3,4, 0,1,0) ; pOldEUTop2->Draw(); sm_GraphicsLoop();
          smgfx_SetLook(3,4, 0,1,1) ; pOldEUBot2->Draw(); sm_GraphicsLoop();
          sm_GraphicsLoop();
        }
#endif // SM_DEBUG_CODE

      // insert NewEdgeuse into TgtVertex CW/CCW doubly linked list
      pOldEUTop1->m_pCW  = pEUTop2; 
      pOldEUTop2->m_pCCW = pEUTop1; 

      pOldEUBot1->m_pCCW = pEUBot2; 
      pOldEUBot2->m_pCW  = pEUBot1; 

      pEUTop2->m_pCCW = pOldEUTop1;
      pEUTop1->m_pCW  = pOldEUTop2;

      pEUBot2->m_pCW  = pOldEUBot1;
      pEUBot1->m_pCCW = pOldEUBot2;
  } // end connecting to a EdgeuseVertex branch
   
  // all done     
  return SM_SUCCESS;       

} // end SmEdge::InsertManifoldEdgeEnd

/*******************************************************************//**
PURPOSE: Return TRUE if the edge is a lamina edge.  

NOTES: Lamina edges
    are edges which have two edgeuses and only one face.   In other words
    they are on the edge of a face.
***********************************************************************/
SmBoolean SmEdge::IsLamina
  () 
 const
{
    SmEdgeuse *pData[32];
    SmTArray<SmEdgeuse*> sEdgeuses(32,pData);
    GetEdgeuses(sEdgeuses);
    if (sEdgeuses.GetSize() != 2) return FALSE;
    SmEdgeuse *pEU = sEdgeuses[0];
    if (!pEU->IsLoopEdgeuse()) return FALSE;
    return TRUE;

} // end SmEdge::IsLamina

/*******************************************************************//**
PURPOSE: Return TRUE if the edge is a manifold edge.  

NOTES: Manifold edges
    are edges which have four edgeuses one or two faces.   
***********************************************************************/
SmBoolean SmEdge::IsManifold
  () 
 const
{
  SmEdgeuse *pData[32];
  SmTArray<SmEdgeuse*> sEdgeuses(32,pData);
  GetEdgeuses(sEdgeuses);

  // manifold edges connect to two sides of two faces through 4 edgeuses
  if (sEdgeuses.GetSize() != 4) 
    { return FALSE ; }

  // Edgeuse types must be LoopEdgeuses
  SmEdgeuse *pEU = sEdgeuses[0];
  if (!pEU->IsLoopEdgeuse()) 
    { return FALSE ; }

  // arrive here for manifold edges
  return TRUE;

} // end SmEdge::IsManifold


/*******************************************************************//**
PURPOSE: Return TRUE if the edge is a Spine edge.  

NOTES: Spine edges are edges which have 6 or more edgeuses and 
   3 or more connections to faces.   
***********************************************************************/
SmBoolean SmEdge::IsSpine
  () 
 const
{
    if ( GetEdgeuseCount() < 6 ) { return FALSE; }
    SmEdgeuse *pEU = SM_CAST_PTR( SmEdgeuse, &(m_pList[0]) );
    if ( !pEU->IsLoopEdgeuse() ) { return FALSE; }
    return TRUE;

} // end SmEdge::IsSpine

/*******************************************************************//**
PURPOSE: Return TRUE if the edge is a strut edge.  

NOTES: Strut edges are connected to one or more vertices which are
   connected only to this edge.   
***********************************************************************/
SmBoolean SmEdge::IsStrut
  ()
 const
{
  SmTArray<SmVertex*> sVertices ;
  SmTArray<SmEdge*> sEdges ;

  GetVertices(sVertices) ;

  // low work - not enough vertices
  if(sVertices.GetSize() < 2) { return(TRUE); }

  // check every vertex
  ULONG ii, lNumVerts = sVertices.GetSize();
  for(ii=0; ii<lNumVerts; ii++)
    {
      // Edge is a strut when any vertex is connected to just one edge
      sVertices[ii]->GetEdges(sEdges) ;
      if(sEdges.GetSize() == 1) { return(TRUE); }

    } // end iter every vertex

  // else - no vertices were connected to just one edge
  return(FALSE) ;

} // end SmEdge::IsStrut

/*******************************************************************//**
PURPOSE: Return TRUE if the edge is a seam for any surface connected to it.  

NOTES: Seam edges connect two different ends of a single face
    to make it closed.
***********************************************************************/
SmBoolean SmEdge::IsSeam
  () 
 const
{
  // get all faces connected to this edge
  SmTArray<SmFace*> sFaces ;
  GetFaces(sFaces) ;

  // for every face
  ULONG ii, lNumFaces = sFaces.GetSize();
  for(ii=0; ii<lNumFaces; ii++)
    {
      // when edge is a seam for any face->Surface - the edge is a seam
      if(   sFaces[ii]
         && sFaces[ii]->GetSurface()
         && IsSeam(*(sFaces[ii]->GetSurface()))) return(TRUE) ;

    } // end iter every face

  // else - no edge/Face->Surface pairs were a seam
  return FALSE;

} // end SmEdge::IsSeam

/*******************************************************************//**
PURPOSE: Return TRUE if the edge is a seam edge for the given Surface.  

NOTES: Seam edges
    are edges which have four edgeuses onto the same face and
    are on a closed surface and have different parameter space
    curves.
***********************************************************************/
SmBoolean SmEdge::IsSeam
  (const SmSurface & crSurface) 
 const
{
    double dEdgeTol = GetTolerance();

    // surface must be closed - GWC: this filter is more expensive to run than the following check, eliminate it?
    if (! (   crSurface.IsClosed(crSurface.GetNaturalUVDomain(),SM_SP_U,&dEdgeTol) 
           || crSurface.IsClosed(crSurface.GetNaturalUVDomain(),SM_SP_V,&dEdgeTol)) ) 
      {
        return FALSE;
      }

    // locals
    ULONG lCount = 0;
    SmEdgeuse *apEUOfF[2] = { NULL, NULL };
    SmEdgeuse *pData[32];
    SmTArray<SmEdgeuse*> sEdgeuses(32,pData);
    GetEdgeuses(sEdgeuses);

    // for everyother edgeuse on the edge's edgeuse list
    for (ULONG i=0; i<sEdgeuses.GetSize(); i+=2) 
      {
        SmEdgeuse * pEU = sEdgeuses[i];
        SmFace    * pF  = pEU->GetFace();

        // save and count those connected to the TargetSurface
        if (pF->GetSurface() == &crSurface) 
          {
            // save the first two edgeuses 
            if (lCount == 0) apEUOfF[lCount] = pEU;
            if (lCount == 1) apEUOfF[lCount] = pEU;

            // when there are more than two return FALSE // GWC:When does this happen?
            if (lCount == 2) return FALSE;
            lCount ++;
          }
      }

    // each UVTrimCurve must be in a different part of param space
    // check this by cheking the UV distances between midpoints

    // must have 2 edgeuses to common face
    if (lCount == 2) 
      {
        SmStatus eStat;
        // eval UV mid-Point value for both Edgeuses
        SmPoint3d sUV1, sUV2;
        eStat = apEUOfF[0]->NormalizedEvaluate( 0.5, TRUE, sUV1 );    // TRUE = UV Eval, FALSE = 3d Eval
          if ( eStat != SM_SUCCESS ) { return FALSE; }
        eStat = apEUOfF[1]->NormalizedEvaluate( 0.5, TRUE, sUV2 );    // TRUE = UV Eval, FALSE = 3d Eval
          if ( eStat != SM_SUCCESS ) { return FALSE; }

        // when mid-points are distinct
        if (sUV1.DistanceBetween(sUV2) > SM_EFF_ZERO_SQ) 
          {
            // it is a seam curve
            return TRUE;
          }
      }

    // else it is not a seam curve
    return FALSE;

} // end SmEdge::IsSeam

/*******************************************************************//**
PURPOSE: SmSurface::CoverOtherCoincidentSurface for two faces: see if the
         surface of one face can cover the part of the other surface
         that the other face uses.

NOTES:
  Without optional domains, CoverOtherCoincidentSurface samples the natural
  domain of the covered surface. A face may use only a small part of its
  surface - an imported cylinder is often a full 360 degree surface under a
  half-cylinder face, and a working copy made to build a face on can be much
  larger than the face. When the covered surface's domain runs across the covering
  surface's seam or outside its sweep, the cover fails although the two
  faces are coincident and the edge between them is topological.
  Use the stored face domains and return their mapped union for the merge
  caller to preserve. Derive bounds from trim curves only if the stored
  domains fail to establish coverage in either direction.
  Delegate surface selection and domain mapping to CoverOtherCoincidentSurface.
***********************************************************************/
static SmStatus sm_CoverOtherCoincidentFace
  (SmFace     * pFace0,          // in : first face
   SmFace     * pFace1,          // in : second face
   double       dTol,            // in : max separation allowed
   double     & rdMaxDist,       // out: max distance found between covering and covered surfaces
   SmSurface *& rpCoveringSurf,  // out: covering surface, NULL if not possible
   SmExtent2d & rMergedDomain,   // out: union in the covering surface's final NURBS parameters
   SmBoolean    bRefineDomains = FALSE) // in : one fallback attempt using trim-derived bounds
{
  // init output
  rpCoveringSurf = NULL;
  rdMaxDist = SM_BIG_DOUBLE;

  // Just for clearer naming: it's all symmetrical between the two faces.
  SmSurface* pSurf0 = pFace0->GetSurface();
  SmSurface* pSurf1 = pFace1->GetSurface();

  // Use the stored conservative face domains, without generating all UV trims.
  SmExtent2d sDomain0 = pFace0->GetUVDomain();
  SmExtent2d sDomain1 = pFace1->GetUVDomain();

  if (bRefineDomains)
    {
      // Trim creation failure can still return SM_SUCCESS with the stored
      // face domain clipped to the surface domain. An error or degenerate
      // result leaves the stored domain unchanged; there is no automatic
      // fallback to the full natural surface domain here.
      SmExtent2d sTrimDomain;
      if (pFace0->CalculateUVDomainFromUVTrimCurves(sTrimDomain, TRUE) == SM_SUCCESS
          && !sTrimDomain.IsDegenerate())
        { sDomain0 = sTrimDomain; }
      if (pFace1->CalculateUVDomainFromUVTrimCurves(sTrimDomain, TRUE) == SM_SUCCESS
          && !sTrimDomain.IsDegenerate())
        { sDomain1 = sTrimDomain; }
    }

  SmStatus eStat = pSurf0->CoverOtherCoincidentSurface(pSurf1, dTol, rdMaxDist,
                                                       rpCoveringSurf, &sDomain0,
                                                       &sDomain1, &rMergedDomain);
  if (eStat != SM_SUCCESS || rpCoveringSurf != NULL)
    { return eStat; }

  // A valid face domain may still include unused portions of the surface.
  // Allow these faces to merge, but pay for all trims
  // only after the inexpensive stored-domain attempts have failed.
  if (!bRefineDomains)
    { return sm_CoverOtherCoincidentFace(pFace0, pFace1, dTol, rdMaxDist,
                                         rpCoveringSurf, rMergedDomain, TRUE); }

  return SM_SUCCESS;

} // end sm_CoverOtherCoincidentFace

/*******************************************************************//**
PURPOSE: Return TRUE if the edge is a topological edge.

NOTES: 
  Topological edges are those that could be removed without
  changing the shape of the model.  Topological edges are those that bound
  two faces that are, or could be, on the same surface.  When an edge is
  removed, the surface of one of its faces is expanded to cover the other
  surface -- the expanded surface must be coincident with the other surface
  over the domain of the other.  

IMPLEMENTATION:
  A couple of inexpensives checks are made to eliminate most candidates.
  If those tests are passed, expensive tests are run on the surfaces
    when one can be made expanded to be coincident to the other.
***********************************************************************/
SmBoolean SmEdge::IsTopological
  (SmSurface *& rpSurfToKeep,       // out: Surface to keep if 'this' gets deleted to merge two faces into one
                                    //      rpSurfaceToKeep domain may be expanded to 'cover' the other surface
   double     & rdDistBetween,      // out: max dist found between covered and covering surfaces
   double       dApproxTol3d_Gain,   // in : Multiplier applied to XSectTol3d, default:[1.0]
                                    //      argument added to support previous behavior.
   SmExtent2d * pOptMergedFaceDomain) // out: optional merged domain in the retained surface's final NURBS parameters
const
{
  // init output
  rpSurfToKeep  = NULL ;
  rdDistBetween = SM_BIG_DOUBLE ;

  // locals
  SM_PTR_ARRAY(sEUs, SmEdgeuse, 16) ; // SmTArray<SmEdgeuse*> sEUs ;
  GetEdgeuses( sEUs ) ; // expect 4 EUs because this->Edge is manifold
   
  // cheap test 1 - only Manifold edges can be "topological" - not calling IsManifold just to save a little time
  if(   sEUs.GetSize() != 4 
     || !sEUs[0]->IsLoopEdgeuse()) 
    { return FALSE ; }

  // next: cheap test 2 - Only Tangent edges can be "topological"
  //       cheap test 3 - Only coincident edges can be "topological"

  // 1st edgeuse locals
  SmSurface * paSurf[2] ; 
  SmExtent2d  saUVDomain[2] ;
  SmEdgeuse * pEU1   = sEUs[0] ;
  SmFace    * pFace1 = pEU1->GetFace() ;
  paSurf[0]          = pFace1->GetSurface() ;
  saUVDomain[0]      = paSurf[0]->GetNaturalUVDomain() ; 

  // 3rd edgeuse locals = same oriented edgeuse on different face to edgeuse[0]
  SmEdgeuse * pEU2   = sEUs[2];
  SmFace    * pFace2 = pEU2->GetFace();
  paSurf[1]          = pFace2->GetSurface();
  saUVDomain[1]      = paSurf[1]->GetNaturalUVDomain() ; 

  // locals
  ULONG ii, jj, j1 ;
  SmStatus eStat;
  SmPoint3d sUV3d ;
  const int lNumSamples = 3 ;
  double daSamples[lNumSamples] = { 0.37, 0.57, 0.82 } ;
  SmPoint2d  saUV[2][lNumSamples] ;
  SmVector3d saNorm1[lNumSamples], saNorm2[lNumSamples] ;

  // normalized sample values
  for(ii=0;ii<lNumSamples;ii++)
   {
      // For first sample - find the associated surface UV values and surface normals
      eStat = pEU1->NormalizedEvaluate( daSamples[ii], TRUE, sUV3d );  // TRUE = UV Eval, FALSE = 3d Eval
      if ( eStat != SM_SUCCESS )
        { return FALSE; }

      saUV[0][ii].Set(sUV3d.x, sUV3d.y) ;
      eStat = paSurf[0]->EvaluateNormal( saUV[0][ii], FALSE, FALSE, saNorm1[ii] );
      if ( eStat != SM_SUCCESS )
        { return FALSE; }

      eStat = pEU2->NormalizedEvaluate( daSamples[ii], TRUE, sUV3d );  // TRUE = UV Eval, FALSE = 3d Eval
      if ( eStat != SM_SUCCESS )
        { return FALSE; }

      saUV[1][ii].Set(sUV3d.x, sUV3d.y) ;
      eStat = paSurf[1]->EvaluateNormal( saUV[1][ii], FALSE, FALSE, saNorm2[ii] );
      if ( eStat != SM_SUCCESS )
        { return FALSE; }

      // cheap test 2: if the surfaces don't share the same normal (are not g1) - this edge is not topological
      if(smos_Fabs(saNorm1[ii].Dot(saNorm2[ii])) < 1.0 - SM_EFF_ZERO_SQRT )
        {
          return(FALSE) ;
        }

      // cheap test 3: coincidence check - gaps are within tol
      double dGapTol = SmTol::GetXSectTol3d(pFace1, pFace2) ; 
      
      SmPoint3d sPt1, sPt2;
      paSurf[0]->EvaluatePoint( saUV[0][ii], sPt1 );
      paSurf[1]->EvaluatePoint( saUV[1][ii], sPt2 );
      if ( sPt1.DistanceBetween( sPt2 ) > dGapTol )
        {
          return(FALSE) ;
        }

    } // end iter every Sample point - checking for tangency and coincidence

  // edges on seams can't be "topological" because they are needed to mark the seam
  //   Next: Cheap test 4 - eliminate candidate edges running along seams

  // Prepare for OnSeam check - see if surfaces are closed (you'd think periodic here - but checking for 'closed' culls more candidate cases) 
  SmBoolean bIsClosedU[2], bIsClosedV[2] ;
  SmExtentPointType ePtTypeU, ePtTypeV;
  bIsClosedU[0] = paSurf[0]->IsClosed(saUVDomain[0], SM_SP_U) ; // SM_SP_U [check Pos[Umin,v] == Pos[Umax,v]
  bIsClosedU[1] = paSurf[1]->IsClosed(saUVDomain[1], SM_SP_U) ; 
  bIsClosedV[0] = paSurf[0]->IsClosed(saUVDomain[0], SM_SP_V) ; // SM_SP_V [check Pos[u,Vmin] == Pos[u,Vmax]
  bIsClosedV[1] = paSurf[1]->IsClosed(saUVDomain[1], SM_SP_V) ;
  
  // when either surface is closed in either or both directions - check for edges that run along closed boundaries
  if(   bIsClosedU[0]
     || bIsClosedU[1]
     || bIsClosedV[0]
     || bIsClosedV[1])
    {   
      // for both surfaces - see if edge runs along a closed boundary
      for(ii=0;ii<2;ii++)
        {
          // skip testing the same surface twice
          if(ii==1 && paSurf[0] == paSurf[1])
            { continue ; }

          SmBoolean bOnSeam = TRUE ; 
          SmBoolean bIsoU   = TRUE ;
          SmBoolean bIsoV   = TRUE ;
      
          // pick a tolerance
          SmScaledZero sScaledZero = 10000.0 * SmTol::GetScaledZero(saUV[ii][0], saUV[ii][lNumSamples-1]) ; 

          // for every sample point pair - see if the curve is Iso
          for(jj=0,j1=1;j1<lNumSamples;jj++,j1++)
            {

              if(!SM_ARE_SAME_TO_TOL(saUV[ii][jj].x, saUV[ii][j1].x, sScaledZero)) { bIsoU = FALSE ; }
              if(!SM_ARE_SAME_TO_TOL(saUV[ii][jj].y, saUV[ii][j1].y, sScaledZero)) { bIsoV = FALSE ; }

              // nonIsoCurves can't be on seams
              if(!bIsoU && !bIsoV)
                { bOnSeam = FALSE ; break ; }
            
            } // end iter every sample point looking for IsoCurves

          // skip further processing when Curve is not on a seam
          if(bOnSeam == FALSE)
            { continue ; }

          // when the curve is Iso in a nonClosed direction - it's not on a seam (only oneof bIsoU or bIsoV can be TRUE)
          if(   (bIsoU && !bIsClosedU[ii]) 
             || (bIsoV && !bIsClosedV[ii]))
            { 
              bOnSeam = FALSE ; 
              continue ; 
            }

          // when IsoCurve is not on UVDomain boundary - it's not on a seam
          saUVDomain[ii].ClassifyPoint2d(saUV[ii][1], ePtTypeU, ePtTypeV, sScaledZero.val, &sScaledZero.val) ;
          if(bIsoU && (ePtTypeU != SM_EP_START && ePtTypeU != SM_EP_END)) { bOnSeam = FALSE ; continue ; }
          if(bIsoV && (ePtTypeV != SM_EP_START && ePtTypeV != SM_EP_END)) { bOnSeam = FALSE ; continue ; }
          
          // arrive here when UVTrimCurve runs along a surface seam
          // cheap test 4: can't delete edges that run along a seam on any surface - report them as nonTopological
          if(bOnSeam)
            { return(FALSE) ; }

        } // end iter every surface - checking for edges that run along closed boundaries
    } // end surfaces are closed check

  // arrive here when edge might be topological 
  // next - see if either surface can be expanded to be coincident with the other

  // when edge is connected to same surface on both sides - it's topological
  if(paSurf[0] == paSurf[1])
    {
      // set output
      rpSurfToKeep = paSurf[0] ;
      rdDistBetween = 0.0 ; 
      if (pOptMergedFaceDomain != NULL)
        { pFace1->GetUVDomain().Union(pFace2->GetUVDomain(), *pOptMergedFaceDomain); }
    }
  else // edge is connected to 2 different surfaces 
    {
      // edge is topological if surfaces are coincident 
      // and one surface can be contained in the other surface (possibly expanded).

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ; 
      if (bDebugMe) // draw inputs
        { 
          SmBrep *pBrep = GetBrep() ;
          this->Dump() ;
                  
          smgfx_Erase();
          smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE); sm_GraphicsLoop() ;
          smgfx_SetLook(3,4, 1,0,0) ; Draw(); sm_GraphicsLoop();
          smgfx_SetLook(2,3, 0,1,1) ; paSurf[0]->DrawUV() ; sm_GraphicsLoop() ;
          smgfx_SetLook(4,5, 0,0,1) ; paSurf[1]->DrawUV(5,5) ; sm_GraphicsLoop() ;
          sm_GraphicsLoop();
        }
#endif // SM_DEBUG_CODE

      // Get tolerance.
      // (Note: in SmMerge::BooleanPostProcess(), tolerance handling is different.)
      SmApproxTol3d sApproxTol3d = dApproxTol3d_Gain * smos_Min(SmTol::GetApproxTol3d(pFace1), SmTol::GetApproxTol3d(pFace2)) ; 

      // Do this only for Analytic surfaces, because they are extendable.  [cbi660]
      if ( ! ( paSurf[0]->IsAnalytic() || paSurf[0]->IsPlanar( sApproxTol3d ) ) )
        { return FALSE; }
      if ( ! ( paSurf[1]->IsAnalytic() || paSurf[1]->IsPlanar( sApproxTol3d ) ) )
        { return FALSE; }

      // If we have planes use a much tighter coincidence check tolerance
      // for cases where there are small polygons.
      if(paSurf[0]->IsKindOf(SmPlane_TYPE)) 
        { sApproxTol3d = sApproxTol3d / 10000.0; }

      // See if this surface can 'cover' the part of the pOther surface that its face uses
      //  or if the pOther surface can 'cover' the part of this surface that its face uses.
      //  (The natural domains are too much: an edge between two faces on one cylinder whose
      //   two surfaces have different seams or sweeps would never be topological.)
      SmExtent2d sMergedDomain;
      eStat = sm_CoverOtherCoincidentFace(pFace1,
                                          pFace2,
                                          sApproxTol3d,
                                          rdDistBetween,
                                          rpSurfToKeep,
                                          sMergedDomain);

      // note: Side Effect = pSurfToKeep Domain may have been increased

      // when surfaces are not coincident or can't cover the one with the other - then edge is not topological
      if ( rpSurfToKeep == NULL || eStat != SM_SUCCESS )
            { return FALSE ; }  // Surfaces not coincident or one couldn't cover the other.

      if (pOptMergedFaceDomain != NULL)
        { *pOptMergedFaceDomain = sMergedDomain; }

    } // end Edge connects two different surfaces check

  // arrive here when edge is topological - outputs have been set
  return(TRUE) ;

} // end SmEdge::IsTopological

/*******************************************************************//**
PURPOSE: Return TRUE when stored m_sZoneTol3d value == SmTol::GetZoneTol3d(this,0.0,TRUE)

NOTES: This is for making old style tolerances consistent.
  In old style tolerancing, the value (now renamed to m_sZoneTol3d)
  is the ZoneTol3d value for this Edge.  In new style tolerancing,
  it's an optional over ride value for the SmContext::ZoneTol3d value.

  So, this function always returns TRUE when compiled with SM_USE_NEWTOL
  else returns TRUE when m_sZoneTol3d == SmTol::GetZoneTol3d(this,0.0,TRUE)
***********************************************************************/
SmBoolean SmEdge::IsZoneTol3dConsistent
  (SmZoneTol3d * pOptZoneTol3d) // out: computed SmTol::GetZoneTol3d value for this vertex, NULL to ignore
                                //      default:[NULL] 
 const
{
#ifdef SM_USE_NEWTOL
  SM_NEWTOL_LINE return(TRUE) ;
#endif // SM_USE_NEWTOL

  SmZoneTol3d sThisTol3d = GetTolerance() ;
  SmZoneTol3d sZoneTol3d = SmTol::GetZoneTol3d(this) ;

  SmBoolean   bRtn       = SM_ARE_SAME(sZoneTol3d, sThisTol3d) ;
  // SmBoolean   bRtn       = TRUE ; // SM_ARE_SAME(sZoneTol3d, sThisTol3d) ;

  // set output
  if(pOptZoneTol3d) { *pOptZoneTol3d = sZoneTol3d ; }

  // all done
  return(bRtn) ;
  
} // end SmEdge::IsZoneTol3dConsistent()

/*******************************************************************//**
PURPOSE: returns max Face<->Edge<->Face gap of all Edge->Faces 
         that are parallel to one another within SM_ANG_TOL_DEG 
         of one another


NOTES: Uses cached Edge/Edge Gaps stored on the Edge->Edgeuses
       and updates those as necessary.

       When bForceCalc == TRUE, all Edge/Edge gaps have been made 
            current when this routine exits.
***********************************************************************/
SmBoolean SmEdge::HasParallelFaces
 (double    * pOptAngTolDeg,          // in : optional max angle deg between parallel vectors, 
                                      //      NULL=SM_ANG_TOL_DEG, default:[NULL]
  double    * pOptMinDihedralAngDeg,  // out: Min Dihedral AngDeg seen between Faces
  double    * pOptMaxParallelFaceGap, // out: when Rtn == TRUE, optional Max gap between parallel face pairs
  SmFace   ** pOptFace1,              // out: when Rtn == TRUE, optional face1 of MaxParallelFace/Face Gap pair
  SmFace   ** pOptFace2,              // out: when Rtn == TRUE, optional face2 of MaxParallelFace/Face Gap pair
  SmPoint2d * pOptUV1,                // out: when Rtn == TRUE, optional face1 UVPt of MaxParallelFace/Face Gap pair
  SmPoint2d * pOptUV2)                // out: when Rtn == TRUE, optional face2 UVPt of MaxParallelFace/Face Gap pair
 const
{
  // init return and outputs
  SmBoolean bRtn = FALSE ; 
  if(pOptMinDihedralAngDeg)  { *pOptMinDihedralAngDeg  = SM_BIG_DOUBLE ; }
  if(pOptMaxParallelFaceGap) { *pOptMaxParallelFaceGap = 0.0 ; }
  if(pOptFace1)              { *pOptFace1 = NULL ; }
  if(pOptFace2)              { *pOptFace2 = NULL ; }
  if(pOptUV1)                { pOptUV1->SetUninitialized() ; }
  if(pOptUV2)                { pOptUV2->SetUninitialized() ; }

  // locals
  ULONG ii, jj ; 
  double            dMaxGap3d, dMaxParallelFaceGap = 0.0, dMinDihedralAngDeg ;
  SmPoint2d         sUV1 ;
  SmPoint2d         sUV2 ;
  SmTArray<SmFace*> sFaces ;
  GetFaces(sFaces) ;

  // For every face pair
  for(ii=0;ii<sFaces.GetSize()-1;ii++)
    {
      SmFace * pFace1 = sFaces[ii] ; 

      for(jj=ii+1;jj<sFaces.GetSize();jj++)
        {
          SmFace * pFace2 = sFaces[jj] ;

          // Look for parallem faces
          SmBoolean bParallel = CalcMaxParallelFaceGap(*pFace1,
                                                       *pFace2,
                                                       dMinDihedralAngDeg,
                                                       dMaxGap3d,
                                                       pOptAngTolDeg,
                                                       &sUV1,
                                                       &sUV2) ;

          if(pOptMinDihedralAngDeg && *pOptMinDihedralAngDeg > dMinDihedralAngDeg)  
            { *pOptMinDihedralAngDeg  = dMinDihedralAngDeg ; }

          // when this face pair is parallel
          if(bParallel)
            {
              bRtn = TRUE ;

              // and this is the largest gap seen
              if(dMaxParallelFaceGap < dMaxGap3d)
                {
                  // set outputs
                  if(pOptMaxParallelFaceGap) { *pOptMaxParallelFaceGap = dMaxGap3d ; }
                  if(pOptFace1)              { *pOptFace1 = pFace1 ; }
                  if(pOptFace2)              { *pOptFace2 = pFace2 ; }
                  if(pOptUV1)                { *pOptUV1   = sUV1 ; }
                  if(pOptUV2)                { *pOptUV2   = sUV2 ; }
                } // end ParallelFace Gap3d seen check
            } // end Parallel Face Pair check
        } // end iter jj, every Face pair
    } // end iter ii, every Face pair

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  // draw 
  if(bDebugMe)
    {
      SmBrep    * pBrep     = GetBrep() ;
      SmSurface * pSurface1 = (bRtn && pOptFace1) ? (*pOptFace1)->GetSurface() : NULL ;
      SmSurface * pSurface2 = (bRtn && pOptFace2) ? (*pOptFace2)->GetSurface() : NULL ;
      SmPoint3d   sPt1,   sPt2 ;
      SmVector3d  sNorm1, sNorm2 ; 

      if(bRtn && pOptUV1) { pSurface1->EvaluatePoint(*pOptUV1, sPt1) ; 
                            pSurface1->EvaluateNormal(*pOptUV1, TRUE, TRUE, sNorm1) ; 
                          }
      if(bRtn && pOptUV2) { pSurface2->EvaluatePoint(*pOptUV2, sPt2) ; 
                            pSurface2->EvaluateNormal(*pOptUV2, TRUE, TRUE, sNorm2) ; 
                          }

      smgfx_Erase() ;
      smgfx_SetLook(1,2,  0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(5,7,  0,0,1) ; Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook(5,7,  0,0,1) ; DrawParams() ; sm_GraphicsLoop() ;
      smgfx_SetLook(7,10, 1,0,0) ; if(bRtn && pOptUV1) { sPt1.Draw() ; sNorm1.Draw(&sPt1) ; } sm_GraphicsLoop() ;
      smgfx_SetLook(7,12, 0,1,0) ; if(bRtn && pOptUV2) { sPt2.Draw() ; sNorm2.Draw(&sPt2) ; } sm_GraphicsLoop() ;
      smgfx_SetLook(1,2,  0,0,0) ; if(bRtn && pOptFace1) (*pOptFace1)->Draw(SM_DM_CROSSHATCH) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2,  0,0,0) ; if(bRtn && pOptFace2) (*pOptFace2)->Draw(SM_DM_CROSSHATCH) ; sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

  // all done - return TRUE = Any Parallel Face Gap Sample points, FALSE=NONE
  return(bRtn) ;

} // end SmEdge::HasParallelFaces

/*******************************************************************//**
PURPOSE: returns TRUE when all Edge/Face gaps are less than
         XSectTol3d(Context)

NOTES: Uses cached Edge/Face Gaps stored on the Edge->Edgeuses
       and updates those as necessary.

       When bForceCalc == TRUE, all Edge/Face gaps have been made 
            current when this routine exits.
***********************************************************************/
SmBoolean SmEdge::IsWithinXSectTol3dOfFaces
 (double  * pOptMaxGap3d,   // out: optional max Edge/Face gap3d found
  SmFace ** pOptMaxGapFace, // out: optional associated Face for found max Edge/Face gap3d
  SmBoolean bForceCalc)     // in : TRUE = force gap evaluation, FALSE=use cache if available
 const                      //      default:[FALSE]
{
  // return value
  SmBoolean bRtn = TRUE ;

  // init output
  if(pOptMaxGap3d)   { *pOptMaxGap3d = 0.0 ; }
  if(pOptMaxGapFace) { *pOptMaxGapFace = NULL ; }

  // locals
  ULONG ii ;
  SmTArray<SmEdgeuse*> sEdgeuses ;
  SmXSectTol3d         sXSectTol3d = SmTol::GetXSectTol3d(GetContext()) ;
  GetEdgeuses(sEdgeuses) ;

  // when asked - clear Edge/Face gap cache to force gap calculations for every Edgeuse
  if(bForceCalc)
    {
      for(ii=0;ii<sEdgeuses.GetSize();ii++)
        {
          sEdgeuses[ii]->m_sMaxEdgeFaceGap3d.ReSet() ;
        }
    } // end need to clear current Edge/Face gap cache check

  // for every Edgeuse
  for(ii=0;ii<sEdgeuses.GetSize();ii++)
    {
      SmEdgeuse *pEdgeuse = sEdgeuses[ii] ;

      // look for Edge/Face gaps that exceed XSectTol3d
      if(pEdgeuse->HasEdgeFaceGap())
        {
          // Side effect: Calc Edge/Face gap on 1st fetch
          SmEdgeFaceGap * pMaxEdgeFaceGap = pEdgeuse->GetMaxEdgeFaceGap(FALSE)  ; // FALSE=calc gap for 1st vertexuse/Face call, fetch for subsequent calls
          double          dGap3d          = pMaxEdgeFaceGap ? pMaxEdgeFaceGap->m_dGap3d : 0.0 ; 

          // check for gaps that exceed XSectTol3d
          bRtn &= dGap3d < sXSectTol3d ;

          // set output
          if(pOptMaxGap3d && *pOptMaxGap3d < dGap3d)
            {
              *pOptMaxGap3d = dGap3d ;
              if(pOptMaxGapFace) { *pOptMaxGapFace = pMaxEdgeFaceGap->GetFace() ; }
            }
        }
    } // end iter very Edgeuse

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  if(bDebugMe)
    {
      ULONG di ;
      SmTArray<SmFace*> sFaces ;
      SmBrep *pBrep = GetBrep() ;
      GetFaces(sFaces) ;

      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(7,9, 1,0,0) ; Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook(3,4, 0,1,1) ; for(di=0;di<sFaces.GetSize();di++) 
                                    { sFaces[di]->DrawUV(8,8,FALSE,NULL,TRUE) ; sm_GraphicsLoop() ;}
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE  

  // all done
  return(bRtn) ;

} // end SmEdge::IsWithinXSectTol3dOfFaces

/*******************************************************************//**
PURPOSE: Return TRUE when Edge->Curve is within XSectTol3d of all
         connected Edge->Faces

NOTES: 
***********************************************************************/
SmBoolean SmEdge::IsOnFaceEdgeForm  
 (double * pdOptMaxGap3d)   // out: Max Gap3d between Edge->Curve and all connected Faces              
 const 
{
  // init output
  if(pdOptMaxGap3d)
    { *pdOptMaxGap3d = 0.0 ; }

  // locals
  ULONG ii ;
  SmBoolean         bRtn        = TRUE ;
  double            dMaxGap3d   = 0.0 ;
  SmTArray<SmFace*> sFaces ;
  GetFaces(sFaces) ;

  // for every face - check for Max Edge/Face gaps
  for(ii=0;ii<sFaces.GetSize();ii++)
    {
      SmEdgeuse * pEdgeuse = GetEdgeuseOfFace(sFaces[ii]) ;
      // When Edge is connected to Face through an Edgeuse
      if(pEdgeuse)
        {              
          SmFace * pFace    = pEdgeuse->GetFace() ;

          // gap free Edge/Face XSectTol3d
          SmXSectTol3d sXSectTol3d = SmTol::GetXSectTol3d(this, pFace) ;

          // Find the Max Edge/Face gap
          SmEdgeFaceGap * pMaxEdgeFaceGap = pEdgeuse->GetMaxEdgeFaceGap() ;

          if(pMaxEdgeFaceGap)
            {
          // Save the largest MaxEdgeFaceGap
              if(dMaxGap3d < *pMaxEdgeFaceGap)
                { dMaxGap3d = *pMaxEdgeFaceGap ; }

          // update the output
              if(*pMaxEdgeFaceGap > sXSectTol3d) 
            { bRtn = FALSE ; }
            } // end pMaxEdgeFaceGap existence check
        } // end Edgeuse existence check 
    } // end iter every face looking for MaxGap3d

  // set output
  if(pdOptMaxGap3d)
    { *pdOptMaxGap3d = dMaxGap3d ; }

  // all done
  return(bRtn) ;

} // end SmEdge::IsOnFaceEdgeForm   

/*******************************************************************//**
PURPOSE: Return TRUE the edge has 4 edgeuses connected to the target face. 
  When pFace==NULL, return TRUE when the edge has 4 edguses to any face. 

NOTES: 1. Embedded = an Edge embedded within a single face, e.g. a crack.
       2. Seams are not considered embedded. 
       3. An embedded edge can connect to other faces.  Use input pFace
          to query about a specific face
***********************************************************************/
SmBoolean SmEdge::IsEmbedded
 (const SmFace  * pOptFace, // in : Target face for search, NULL = any face attached to Edge, default:[NULL]
  SmBoolean bFaceHasSeam    // in : default:[TRUE] = Face known to have SeamEdge - do Expensive IsSeam() check
                            //      FALSE          = Face known to have no SeamEdge - skip IsSeam() check
 ) const
{
  ULONG ii, jj ;

  // search pOptFace or all faces connected to this edge when pOptFace == NULL
  SmTArray<SmFace*> sFaces ;
  if(pOptFace == NULL) { GetFaces(sFaces) ; }
  else                 { sFaces.Add((SmFace*)pOptFace) ; }

  SmTArray<SmEdgeuse*> sEdgeuses ;
  GetEdgeuses(sEdgeuses) ;

  // for every face
  for(ii=0;ii<sFaces.GetSize();ii++)
    {
      SmFace *pTgtFace = sFaces[ii] ;

      // count the edgeuses that connect to this face
      ULONG lCnt = 0 ;

      // for every edgeuse
      for(jj=0;jj<sEdgeuses.GetSize();jj++)
        {
          SmEdgeuse *pEdgeuse = sEdgeuses[jj] ;

          // count face uses
          if(pEdgeuse->GetFace() == pTgtFace) { lCnt++ ; }
        }

      // when we found a face being embedded
      if((lCnt == 4) && (bFaceHasSeam == FALSE || !this->IsSeam() ))
        { return(TRUE) ; }

    } // end iter every face

  // else - edge is not embedded
  return FALSE;

} // end SmEdge::IsEmbedded

/*******************************************************************//**
PURPOSE: Return TRUE if the Edge is manifold and its two Faces
    are tangent to each other along the Edge.

NOTES: 
***********************************************************************/
SmBoolean SmEdge::IsTangentEdge
 (double    dTangencyTolDeg,
  SmBoolean bCoincidenceTest)   // Test for coincidence instead
 const                          //   of tangency between faces.
{
  if ( ! this->IsManifold() )
    { return FALSE; }

  SmEdgeuse *pPrimEU = GetPrimaryEdgeuse();

  return pPrimEU->IsTangentSector( dTangencyTolDeg, bCoincidenceTest );

} // end IsTangentEdge

/*******************************************************************//**
PURPOSE: Return TRUE if the Edge is manifold and its two Faces
    are G2 to each other along the Edge.

NOTES: 
***********************************************************************/
SmBoolean SmEdge::IsG2Edge() const
{
  // must be manifold
  if (!this->IsManifold() )
    { return FALSE; }

  // get sector continuity for primary edgeuse
  SmEdgeuse       * pPrimEU     = GetPrimaryEdgeuse();
  SmContinuityType  eContinuity = pPrimEU->GetSectorContinuity() ;

  // return G2 boolean
  return(SM_IS_G2(eContinuity)) ; 

} // end SmEdge::IsG2Edge

/*******************************************************************//**
PURPOSE: Return TRUE if the Edge is manifold and its two Faces
    are G3 to each other along the Edge.

NOTES: 
***********************************************************************/
SmBoolean SmEdge::IsG3Edge() const
{
  // must be manifold
  if (!this->IsManifold() )
    { return FALSE; }

  // get sector continuity for primary edgeuse
  SmEdgeuse       * pPrimEU    = GetPrimaryEdgeuse();
  SmContinuityType eContinuity = pPrimEU->GetSectorContinuity() ;

  // return G3 boolean
  return(SM_IS_G3(eContinuity)) ; 

} // end SmEdge::IsG3Edge

/*******************************************************************//**
PURPOSE: Determine if this edge is part of a tolerant corner where 
  more than 3 faces come together not at a point.

NOTES: 
***********************************************************************/
SmBoolean SmEdge::IsComplexCornerMember
  (SmTArray<SmTopology*> *pOptComplexCornerElements, // out: optional list of vertices and edges making up the corner
                                                     //      NULL to ignore, default:[NULL].
   SmTArray<SmFace*>     *pOptComplexCornerFaces)    // out: optional list of faces meeting at this corner
                                                     //      NULL to ignore, default:[NULL].
  const
{
  // init output 
  if(pOptComplexCornerElements) pOptComplexCornerElements->ReSet() ;
  if(pOptComplexCornerFaces)    pOptComplexCornerFaces->ReSet() ;

  // locals
  SmTArray<SmTopology*> sComplexCornerElements ;
  SmTArray<SmFace*>     sComplexCornerFaces ;
     
  // pass the call along
  SmBoolean bRtn = GetVertex()->IsComplexCornerMember(&sComplexCornerElements,
                                                      &sComplexCornerFaces) ; 

  // see if this edge is part of any found complex corner topology
  ULONG lIndex ;
  if(sComplexCornerElements.FindElement((SmTopology *)this, lIndex))
    {
      // set output
      if(pOptComplexCornerElements) pOptComplexCornerElements->Append(sComplexCornerElements) ;
      if(pOptComplexCornerFaces)    pOptComplexCornerFaces->Append(sComplexCornerFaces) ;
      bRtn = TRUE ;
    }

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  if(bDebugMe)
    {
      // draw Brep(blue), ComplexMembers(orange), edge(red)
      ULONG jj ;
      SmBrep *pBrep = GetBrep() ;
      double dcnt   = (double)sComplexCornerFaces.GetSize() + 1 ;

      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(3,4, 1,0,0) ; Draw() ; sm_GraphicsLoop() ;
      for(jj=0;jj<sComplexCornerElements.GetSize();jj++)
        { SmTopology *pObject = sComplexCornerElements[jj] ;
          if(pObject->IsKindOf(SmEdge_TYPE)) 
               { smgfx_SetLook(2,3, 1,0,.5); ((SmEdge  *)pObject)->Draw() ; sm_GraphicsLoop() ; }
          else { smgfx_SetLook(4,5, 1,.5,0); ((SmVertex*)pObject)->Draw() ; sm_GraphicsLoop() ; }
        }
      for(jj=0;jj<sComplexCornerFaces.GetSize();jj++)
        { smgfx_SetLook(1,2, 0, (dcnt-jj-1)/(dcnt), 1.0 - (dcnt-jj-1)/(dcnt)) ;
          sComplexCornerFaces[jj]->Draw(SM_DM_CROSSHATCH, 4,4) ; sm_GraphicsLoop() ; }  
    }
#endif // SM_DEBUG_CODE

  // all done
  return(bRtn) ;

} // end SmEdge::IsComplexCornerMember

/*******************************************************************//**
PURPOSE: Return TRUE if Edge is connected to Vertex, else return FALSE.

NOTES: 
***********************************************************************/
SmBoolean SmEdge::IsConnectedToVertex
  (const SmVertex *cpVertex)           // in : Target Vertex
  const
{
  SmVertex *pStartVertex = GetStartVertex() ;
  SmVertex *pOtherVertex = GetOtherVertex(pStartVertex) ;

  return(   cpVertex != NULL
         && (   cpVertex == pStartVertex
             || cpVertex == pOtherVertex)) ;

} // end SmEdge::IsConnectedToVertex

/*******************************************************************//**
PURPOSE: Return TRUE if Edge is connected to Face, else return FALSE.

NOTES: 
***********************************************************************/
SmBoolean SmEdge::IsConnectedToFace
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

} // end SmEdge::IsConnectedToFace

/*******************************************************************//**
PURPOSE: Return True when Edge is connected to given Target Topology object

NOTES: 1. the input Target cpConnectTgt may be NULL, or of type,
             SmVertex,
             SmEdge,
             SmLoop,
             SmFace,
             SmShell,
             SmRegion
        2. returns TRUE for NULL and for any other unsupported Topology TYPE
***********************************************************************/
SmBoolean SmEdge::IsConnectedTo
  ( const SmTopology *cpConnectTgt )    // in : target Topology
 const
{
  // no work - no Topo
  if(cpConnectTgt == NULL) 
    { return TRUE ; }

  // switch on cpConnectTgt type
  switch(cpConnectTgt->GetType())
    {
      case SmVertexuse_TYPE: { return(   ((SmVertexuse *)cpConnectTgt)->GetEdgeuse()
                                      && this == ((SmVertexuse *)cpConnectTgt)->GetEdgeuse()->GetEdge()) ; 
                             } break ;
      case SmVertex_TYPE   : { SmTArray<SmVertex *> sVertices ;
                               GetVertices(sVertices) ; 
                               return( sVertices.IsIn((SmVertex *)cpConnectTgt) ) ;
                             } break ;
      case SmEdge_TYPE     : { return( this == (SmEdge *)cpConnectTgt ) ; 
                             } break ;
      case SmEdgeuse_TYPE  : { return( this == ((SmEdgeuse *)cpConnectTgt)->GetEdge() ) ;
                             } break ;
      case SmFace_TYPE     : { SmTArray<SmFace *> sFaces ;
                               GetFaces(sFaces) ; 
                               return( sFaces.IsIn((SmFace *)cpConnectTgt) ) ;
                             } break ;
      case SmFaceuse_TYPE  : { SmTArray<SmFace *> sFaces ; // an Edge is always connected to both Face Faceuses
                               GetFaces(sFaces) ; 
                               return( sFaces.IsIn(((SmFaceuse *)cpConnectTgt)->GetFace()) ) ;
                             } break ;
      case SmLoop_TYPE     : { SmTArray<SmEdge *> sEdges ;
                               ((SmLoop *)cpConnectTgt)->GetEdges(sEdges) ; 
                               return( sEdges.IsIn((SmEdge *)this) ) ;
                             } break ;
      case SmLoopuse_TYPE  : { SmTArray<SmEdge *> sEdges ; 
                               ((SmLoopuse *)cpConnectTgt)->GetEdges(sEdges) ; 
                               return( sEdges.IsIn((SmEdge *)this) ) ;
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

} // end SmEdge::IsConnectedTo

/*******************************************************************//**
PURPOSE: Remove all Cached data holding Edge->Curve Param values

NOTES: Removes
         1. all Edge->Edgeuse->UVTrimCurves
         2. all Edge->Edgeuse->CCWEdgeGaps
         3. all Edge->Vertexuse->VertexEdgeGap
***********************************************************************/
SmStatus SmEdge::ClearUVCaches()
{
  // notify the edge of upcoming change
  Notify(SM_NO_PRE_EDIT, this, SM_NO_GET_BREP(this), NULL) ;

  // locals
  ULONG ii ;
// Remove Composites
//  SmCEdge *pCEdge = SM_CAST_NONNULL_PTR( SmCEdge, this ) ;
//
//  // When edge is composite
//  if ( pCEdge != NULL )
//    {
//      SmTArray< SmEdge* > sEdges;
//      pCEdge->GetEdges( sEdges );
//
//      // for every composite member - remove UVTrimCurves
//      for ( ii=0; ii<sEdges.GetSize(); ii++ )
//        { SER( sEdges[ii]->RemoveUVTrimCurves() ); }
//
//      // all done
//      return SM_SUCCESS ;
//
//    } // end composite edge check

  // edge locals
  SmTArray<SmEdgeuse*> sEdgeuses(256) ;
  SmTArray<SmVertexuse*> sVertexuses(256) ;
  GetEdgeuses(sEdgeuses) ;
  GetVertexuses(sVertexuses) ;

  // for every Edgeuse
  for(ii=0; ii<sEdgeuses.GetSize(); ii++) 
    {
      // remove the edgeuse's UVTrimCurve by setting the UVTrimCurve to NULL 
      SmEdgeuse *pEU = sEdgeuses[ii];
      pEU->SetUVTrimCurve(NULL, 0.0, TRUE) ; // TRUE = delete existing UVTrimCurve
    
      // clear the edgeuse's MaxEdgeFaceGap3d
      pEU->InitEdgeCCWEdgeGap() ;
    }

  // for every upwardFaceuse->Vertexuse
  for (ii=0; ii<sVertexuses.GetSize(); ii++)
    {
      // clear the vertexuse's VertexFaceGap3d
      SmVertexuse *pVU = sVertexuses[ii];
      pVU->InitVertexEdgeGap() ;
    }

  // all done
  return SM_SUCCESS;

} // end SmEdge::ClearUVCaches()

/*******************************************************************//**
PURPOSE: Force Edge->Curve type to be SmBSplineCurve by
  replacing the current curve with an SmSplineCurve approximation when
  appropriate.

NOTES: When Edge->curve is replaced, the old curve and all edge->UVTrimCurves are deleted. 
       When Edge->Curve is already a kindof SmBSplineCurve 
       takes no action and returns SM_SUCCESS.
***********************************************************************/
SmStatus SmEdge::ApproximateWithBSpline
 (double   dApproxTol3d,    // in : maximum allowed distance between approx and original curves
  double & dAchievedTol3d)  // out: actual max distance between approx and original curves
{
  // init output
  dAchievedTol3d = 0.0 ;

  // no work - no curve or curve is a kindof SmBSplineCurve
  if(   GetCurve() == NULL
     || GetCurve()->IsKindOf(SmBSplineCurve_TYPE))
    {
      // all done
      return(SM_SUCCESS) ;
    }

  // locals
  ULONG ii ;
  SmStatus sRtn = SM_SUCCESS ;
  const SmContext     * cpContext = GetContext() ;
  SmCurve             * pCurve    = GetCurve() ;
  SmBSplineCurve      * pNewBSplineCurve = NULL ;
  SmTArray<SmVertex *>  sVertices ;
  // SmCEdge             * pCEdge = GetCompositeEdgeOwner() ;
  SmBrep              * pBrep  = GetBrep() ;
  double                dTParam ;
  SmTemporaryChangeValue<SmBoolean> sEdit( pBrep->m_bEditingEnabled, TRUE );

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
#endif // SM_DEBUG_CODE

  // Offset Curves
  if(pCurve->IsKindOf(SmOffsetCurve_TYPE)) 
    {
      SmOffsetCurve * pOffsetCurve  = ((SmOffsetCurve*)pCurve) ;
      const SmCurve * pBaseCurve    = pOffsetCurve->GetBaseCurve() ;
      double          dOffsetDist   = pOffsetCurve->GetOffsetDistance() ;
      SmVector3d      sOffsetNormal = pOffsetCurve->GetOffsetNormal() ;

      if(pBaseCurve->IsKindOf(SmBSplineCurve_TYPE))
        {
          SmBSplineCurve *pBSplineCurve = ((SmBSplineCurve *)pBaseCurve) ;
          pBSplineCurve->CreateSimpleOffset(*cpContext,        // in : context for new object construction
                                            dApproxTol3d,      // in : Distance between curve created and theoretical
                                                               //      exact offset (1.0e-2 to 1.0e-6) are typical
                                                               //      values used for curves of size 1.0.
                                            sOffsetNormal,     // in : Normal of the reference plane used for offseting.
                                            dOffsetDist,       // in : If this is a positive value the offset will lie to
                                                               //      the right hand side of the curve as viewed from the
                                                               //      positive side of the offset plane.  A negative offset
                                                               //      value produces a curve on the left hand side of the
                                                               //      base curve.
                                            pNewBSplineCurve,  // out: If there is no valid offset, this value will be NULL.
                                                               //      Otherwise, it will be offset curve.
                                            dAchievedTol3d) ;  // out: Max error of offset approximation.
                                                               // in : TRUE = approxCurve(param) within 3DApproxTol of ThisCurve(param) for all param values
                                                               //      FALSE= ApproxCurve(param) within 3d ApproxTol of ThisCurve(AnyParam)
                                                               //      default:[FALSE], FALSE produces lower control point count curves for slightly more cost.
         } // end BSplineCurve BaseCurve type
     } // end OffsetCurve check

  // low work - curve shape and orientation can be handled by special cases
  if(pNewBSplineCurve == NULL)
    {
      sRtn = pCurve->MakeExactBSplineIfPossible(pNewBSplineCurve) ; 
    }

  // when Approximating a BSPlineCurve to current shape is required
  if(pNewBSplineCurve == NULL)
    {
      // gather Edge (or CEdge) vertices
      // ReplaceCurve will not edit CEdge curve. It removes this Edge from CEdge, so don't constrain approximation based on CEdge vertices
      // Also, CEdges are not guaranteed to have Vertices. [B665]
      //if(pCEdge) { pCEdge->GetVertices(sVertices) ;}
      //else       { GetVertices(sVertices) ; }
      GetVertices( sVertices );

      // presize locals
      ULONG ip = 0;
      ULONG lPtCnt  = sVertices.GetSize() ;

      // declare presized ApproximateCurveWithTrimPoints() input arrays
      SmTArray<SmPoint3d> s3DTrimPoints    (lPtCnt,  NULL, lPtCnt) ;  // in : optional array of 3d Point constraints, NULL to ignore, default:[NULL]
      SmTArray<double>    sTParamTrimPoints(lPtCnt,  NULL, lPtCnt) ;  // in : associated curve UVPoints made close to each 3DTrimPoint, NULL to ignore, default:[NULL] 

      // for every vertex - add vertex xyz, uv, values to ApproximateCurveWithTrimPoints() input arrays
      for(ii=0;ii<lPtCnt;ii++)
        {
          SmVertex *pVertex = sVertices[ii] ;

          // Get Vertexuse between Edge and Vertex
          SmVertexuse *pVertexuse = pVertex->GetVertexuseOfEdge(this) ;
          if(pVertexuse == NULL)
            { continue ; }
      
          // Get TPoint for vertexuse  
          sRtn = pVertexuse->ComputeTPoint(dTParam) ;
          if(sRtn != SM_SUCCESS)
            { continue ; }

          // set values
          s3DTrimPoints.SetAt(ii, pVertex->GetPoint() ) ;
          sTParamTrimPoints.SetAt(ii, dTParam ) ; 

          // increment the smp array indices
          ip++ ;

        } // end iter ii, every vertex bulding NLib input sample arrays

      // set Point array final sizes
      if(ip != lPtCnt)
        {
          s3DTrimPoints.SetSize(ip) ;
          sTParamTrimPoints.SetSize(ip) ;
        }

      // Build the approximation curve
      pCurve->ApproximateConstrainedCurve
                        (*cpContext,           // in : context for new object construction
                          dApproxTol3d,        // in : maximum allowed distance between Approx and original curves
                          GetInterval(),       // in : curve interval to approximate
                          dAchievedTol3d,      // out: actual max distance between approx and original curves     
                          pNewBSplineCurve,    // out: The new curve when successful, else NULL.
                                               //      expected to be NULL on input - otherwise can cause a memory leak
                         &s3DTrimPoints,       // in : optional array of 3d Point constraints, NULL to ignore, default:[NULL]
                         &sTParamTrimPoints) ; // in : associated curve UVPoints made close to each 3DTrimPoint, NULL to ignore, default:[NULL] 
    } // end couldn't convert to an exact BSpline check

  // when ApproximateCurveWithTrimPoints worked
  if(sRtn == SM_SUCCESS && pNewBSplineCurve != NULL)
    {
      // swap the edge's curve - delete appropriate objects (old curve and UVTrimCurves)
      pBrep->ReplaceCurve(this, pNewBSplineCurve) ;
    }
  else // ApproximateCurveWithTrimPoints failed
    {
      if(pNewBSplineCurve) { delete pNewBSplineCurve ; pNewBSplineCurve = NULL ; }
    }

#ifdef SM_DEBUG_CODE
  // draw 
  if(bDebugMe)
    {
      SmEdge *pEdge = this ;
      pBrep = pEdge ? pEdge->GetBrep() : NULL ;
      SmExtent1d sIvl = GetInterval() ;

      SM_DUMP_AND_ASSERT_VALID(pBrep) ;
      SM_DUMP_AND_ASSERT_VALID(pNewBSplineCurve) ;
      SM_DUMP_AND_ASSERT_VALID(this) ;

      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(3,4, 0,1,1) ; if(pNewBSplineCurve) pNewBSplineCurve->Draw(&sIvl,TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(3,4, 0,1,1) ; if(pNewBSplineCurve) pNewBSplineCurve->DrawParams() ; sm_GraphicsLoop() ;
      smgfx_SetLook(2,3, 0,0,0) ; if(pEdge) pEdge->Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook(2,3, 0,0,0) ; if(pEdge) pEdge->DrawParams() ; sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

  // all done 
  return(SM_SUCCESS) ;

} // end SmEdge::ApproximateWithBSpline

/*******************************************************************//**
PURPOSE: Set the suggested offset-size for this edge and possibly
    its bounding vertices.

NOTES: If the bUpdateOnlyIfLarger flag is TRUE then we will
     use the given tolerance only if it is larger than the existing one.
     If FALSE then we just change the value of the tolerance.
***********************************************************************/
#ifdef SM_USE_OLDTOL
void SmEdge::SetTolerance
 (SmZoneTol3d sNewZoneTol3d,        // in : Desired New ZoneTol3d Value
  SmBoolean   bUpdateOnlyIfLarger,  // in : default:[TRUE] = set tolerance only if sZoneTol3d > m_sZoneTol3d
                                    //      FALSE          = always set tolerance (allow shrinking TolValues)
  SmBoolean   bCascadeToBndries)   // in : default:[TRUE] = Chg VtxTol to be >= EdgeTol (previous behavior)
                                    //      FALSE          = never change a VtxTol
{
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;

  SmBrep *pBrep = GetBrep();
  // when tolerances get too large - inform the public
  if (   sNewZoneTol3d > m_sZoneTol3d
      && pBrep != NULL
      && sNewZoneTol3d > pBrep->GetTolerance()*3.0) 
    {
      if ( bDebugMe) // place for a breakpoint to check by hand
        {
          TCHAR sBuff[SM_TBLOCK_SIZE], sBuffForFile[SM_TBLOCK_SIZE];
          smos_sprintf(sBuff,       _T("Tolerance of Edge = %16.16lf - Getting Rather Large\n"),
                     (double)sNewZoneTol3d);
          smos_sprintf(sBuffForFile,_T("Tolerance of Edge = %16.16lf - Getting Rather Large\n"),
                     (double)sNewZoneTol3d);
          WARN2(sBuff,sBuffForFile);
        }
    } // end large tolerance check

#endif // SM_DEBUG_CODE

  // set tolerances - when asked skip shrinking tolerances
  if(   sNewZoneTol3d <= m_sZoneTol3d
     && bUpdateOnlyIfLarger == TRUE)
    { return ; }

  // set tolerance
  m_sZoneTol3d = sNewZoneTol3d ;

  // when asked - make sure Edge->VertexTols at least as big as thie EdgeTol 
  if(bCascadeToBndries)
    {
      SmVertex *pV  = GetVertex();
      SmVertex *pOV = GetOtherVertex(pV);

      if ( pV && pV->GetTolerance() < sNewZoneTol3d) 
        { pV->SetTolerance(sNewZoneTol3d) ; }

      if ( pOV && pOV->GetTolerance() < sNewZoneTol3d) 
        { pOV->SetTolerance(sNewZoneTol3d) ; }
    }
} // end SmEdge::SetTolerance
#endif // SM_USE_OLDTOL

/*******************************************************************//**
PURPOSE: Signal Errors for edge related errors.

NOTES:  --> Can change database!
    --> But only vertex tolerance,
        or domain of uv curves in Edgeuses
        (and uv curves are routinely created and destroyed during normal operation).
    1. Check that the 3D curve does not have sharp reversals at ends
    2. Curve must have an owner which is this Edge or a CEdge that contains this edge.
    3. check distances from curve-startPt to start vertex and curve-endPt to end vertex.
       If > vertex tolerances,
       --> Update Vertex->Tolerance to endGaps
    4. Check edge interval is contained by curve interval 
    5. Check UVTrimCurve->Intervals = edge->Interval
       --> Scale UVTrimCurve natural interval = edge's stored interval.
    6. Check that the 3D and 2D curves do not have sharp reversals at ends
***********************************************************************/
SmStatus SmEdge::ValidateGeometry
  (SmBoolean bFixTolerances)        // in : TRUE = Don't report tolerance errors
                                    //             adjust tolerances to gap sized
                                    //      FALSE= Report tolerance errors - don't fix them
 const
{
  // local
  const SmEdge *pE = this ;

  // get SAME oriented edgeuse
  SmEdgeuse *pEU =  (GetPrimaryEdgeuse()->GetOrientation() == SM_OT_SAME) 
                   ? GetPrimaryEdgeuse()
                   : GetPrimaryEdgeuse()->GetMate() ;

  // get curve, start and end vertices for this edge
  SmVertex *pStartV = pEU->GetVertexuse()->GetVertex();
  SmVertex *pEndV   = GetOtherVertex(pStartV);
  SmCurve  *pCurve  = GetCurve();

  // Test to see if 3D curve has reversed direction at ends
  SmValidityCheckType  eCheckFailed;
  if (!pCurve->PassesValidityCheck(SM_VC_REVERSE_DIRECTION, eCheckFailed))
      SE(SM_ERR_WARNING);

// Remove Composites
//   // Curve must belong to this Edge or a CompositeEdge containing this edge
  // Curve must belong to this Edge
  const SmObject *pCurveOwner = pCurve->GetOwner() ;
  if(pCurveOwner == NULL) 
    { SER(SM_ERR); }
// Remove Composites - added 2 lines
  if(!pCurveOwner->IsKindOf(SmEdge_TYPE)) { SER(SM_ERR); }
  if( pCurveOwner != pE) { SER(SM_ERR); }

// Remove Composites
// if(!pCurveOwner->IsKindOf(SmEdge_TYPE)  && !pCurveOwner->IsKindOf(SmCEdge_TYPE)) { SER(SM_ERR); }
//  if(pCurveOwner->IsKindOf(SmCEdge_TYPE))
//    {
//      if ( !((SmCEdge *)pCurveOwner)->IsEdgeInCEdge(pE) )
//        { SER(SM_ERR); }
//    }
//  else
//    {
//      if ( pCurveOwner != pE || !pCurveOwner->IsKindOf(SmEdge_TYPE) )
//        { SER(SM_ERR); }
//    }

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
  if (bDebugMe) 
    {
      SmBrep * pBrep = GetBrep() ;

      smgfx_Erase();
      smgfx_SetLook(1,0, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop();
      smgfx_SetLook(3,4, 0,1,1) ; pStartV->Draw(); sm_GraphicsLoop();
      smgfx_SetLook(3,4, 1,0,1) ; pEndV->Draw(); sm_GraphicsLoop();
      smgfx_SetLook(2,3, 1,0,0) ; Draw(); sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

  // get curve vals for Edge interval startPt and EndPt.
  SmPoint3d sStPt, sEndPt;
  SER(pCurve->EvaluatePoint(GetInterval().GetMin(),sStPt));
  SER(pCurve->EvaluatePoint(GetInterval().GetMax(),sEndPt));

  // check dist between curve startPt and start-vertex
  double dStartDist = sStPt.DistanceBetween(pStartV->GetPoint());
  if (dStartDist > pStartV->GetTolerance() + pCurve->GetEdge()->GetTolerance()) 
    {
      if (bFixTolerances && dStartDist < pStartV->GetTolerance() * 100.0) 
        {
          SM_OLDTOL_LINE pStartV->SetTolerance(dStartDist * 2.0) ;
        }
      else 
        {
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe1 = FALSE;
          if (bDebugMe1) 
            {
              smgfx_Erase();
              smgfx_SetLook(2,4, 1,0,0) ; pStartV->Draw();  sm_GraphicsLoop() ;
              smgfx_SetLook(2,4, 1,1,0) ; sStPt.Draw();  sm_GraphicsLoop() ;
              smgfx_SetLook(2,4, 0,1,1) ; Draw(); sm_GraphicsLoop() ;
              sm_GraphicsLoop();
            }
#endif // SM_DEBUG_CODE
          SER(SM_ERR);
        }
    } // end Bad CurveStartPoint - StartVertex gap size check

  // check dist between curve endPt and end-vertex 
  double dEndDist = sEndPt.DistanceBetween(pEndV->GetPoint());
  if (dEndDist > pEndV->GetTolerance() + pCurve->GetEdge()->GetTolerance()) 
    {
      if (bFixTolerances && dEndDist < pEndV->GetTolerance() * 100.0) 
        {
          SM_OLDTOL_LINE pEndV->SetTolerance(dEndDist * 2.0) ;
        }
      else 
        {
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe2 = FALSE;
  if (bDebugMe2) 
    {
      smgfx_Erase();
      smgfx_SetLook(2,4, 1,0,0) ; pEndV->Draw();  sm_GraphicsLoop() ;
      smgfx_SetLook(2,4, 1,1,0) ; sEndPt.Draw();  sm_GraphicsLoop() ;
      smgfx_SetLook(2,4, 0,1,1) ; Draw(); sm_GraphicsLoop() ;
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE
          SER(SM_ERR);
        }
    } // end bad endCurvePoint - endVertex gap size check

  // Check to make sure we have valid intervals.
  SmExtent1d sNaturalInterval = pCurve->GetNaturalInterval();
  
  // First make sure Edge interval is contained by curve's Natural interval
  if (!pE->GetInterval().IsContainedBy(sNaturalInterval, SM_EFF_ZERO_PARAM)) 
  { 
      SmExtent1d sEIvl( pE->GetInterval() );
      TCHAR sBuff[SM_TBLOCK_SIZE];
      smos_sprintf(sBuff,_T("SmEdge::ValidateGeometry(): Edge interval not contained by curve's Natural interval to SM_EFF_ZERO_PARAM\n Edge  Ivl [%16.16lf, %16.16lf] \n Curve Ivl [%16.16lf, %16.16lf] \n"), \
                  sEIvl.GetMin(), sEIvl.GetMax(), sNaturalInterval.GetMin(), sNaturalInterval.GetMax() );
      SE_MSG(SM_ERR, sBuff);
  }

  // Check that the parameters of the geometry match up
  SmEdgeuse * sEUData[64];
  SmTArray<SmEdgeuse*> sEUs(64,sEUData);
  GetEdgeuses(sEUs);

  double dMaxUVDist = -2;
  for (ULONG i=0; i<sEUs.GetSize(); i++) 
    {
      pEU = sEUs[i];
      if (pEU->m_pUVTrimCurve) 
        {
          SmExtent1d sUVIvl = pEU->m_pUVTrimCurve->GetNaturalInterval();
          SmBoolean bEditedParameterization = FALSE ;
          if (!SM_ARE_SAME(sUVIvl.GetMin(),m_vInterval.GetMin())) 
            {
              MSG(_T("Possible Accuracy problems in Edge Trim Intervals"));
              if ( bFixTolerances )
                {
                  pEU->m_pUVTrimCurve->EditParameterization(m_vInterval);
                  bEditedParameterization = TRUE ;
                }
            }
          if (!SM_ARE_SAME(sUVIvl.GetMax(),m_vInterval.GetMax())) 
            {
              MSG(_T("Possible Accuracy problems in Edge Trim Intervals"));
              if ( bFixTolerances )
                {
                  pEU->m_pUVTrimCurve->EditParameterization(m_vInterval);
                  bEditedParameterization = TRUE ;
                }
            }
          // EditParameterization replaces the natural interval. The distance
          // check below must use that interval, not the one captured above.
          if (bEditedParameterization)
            { sUVIvl = pEU->m_pUVTrimCurve->GetNaturalInterval() ; }

          static constexpr SmBoolean bDoUVCurveCheck = TRUE;
          if ( bDoUVCurveCheck )
            {
              // Check accuracy of uv-curve
              SmBSplineCurve *pUVCrv = pEU->m_pUVTrimCurve;
              SmSurface *pSrf = pEU->GetFace()->GetSurface();
              SmCrvOnSurf sCrvOnSurf( *pUVCrv, *pSrf, NULL, 0, GetContext() );

              double dUVDist = -1;
              this->GetCurve()->CurveMaxDistanceBetween( m_vInterval, sCrvOnSurf,
                                                         sUVIvl.GetMin(),
                                                         sUVIvl.GetMax(),
                                                         51, NULL, dUVDist );

#ifdef SM_DEBUG_CODE
              // draw 
              if(bDebugMe)
                {
                  SmBrep  *pBrep  = this->GetBrep();
                  SmFace  *pFace  = (pSrf && pSrf->GetFace()) ? (SmFace *)pSrf->GetFace() : NULL ;
                  pCurve = this->GetCurve() ;
      
                  smgfx_Erase() ;
                  smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
                  smgfx_SetLook(1,2, 0,1,1) ; if(pSrf) pSrf->DrawUV(7,7) ; sm_GraphicsLoop() ;
                  smgfx_SetLook(1,2, 0,1,1) ; if(pSrf) pSrf->DrawPolygon() ; sm_GraphicsLoop() ;
                  smgfx_SetLook(1,2, 0,0,0) ; if(pFace) pFace->Draw(SM_DM_CROSSHATCH) ; sm_GraphicsLoop() ;
                  smgfx_SetLook(3,4, 1,0,0) ; if(pCurve) pCurve->Draw() ; sm_GraphicsLoop() ;
                  smgfx_SetLook(3,4, 0,0,1) ; sCrvOnSurf.Draw() ; sm_GraphicsLoop() ;
                  smgfx_SetLook( 5, 6, 1, 0, 1 ); this->Draw(); sm_GraphicsLoop();
                  sm_GraphicsLoop() ;
                }
#endif // SM_DEBUG_CODE
                    
              if ( dUVDist > this->GetTolerance() ) 
                {
                  TCHAR sBuff[SM_TBLOCK_SIZE];
                  smos_sprintf(sBuff,_T("SmEdge::ValidateGeometry(): UVTrimCurve To Edge Max Dist[%16.16lf] greater than Edgeuse->Tolerance[%16.16lf]\n"),dUVDist, (double)this->GetTolerance());
                  SE_MSG(SM_ERR_WARNING, sBuff);
                }
              if ( dUVDist > dMaxUVDist ) 
                {
                  dMaxUVDist = dUVDist;
                }
            } // end check accuracy of uv-curve.

          // check UV curve does not reverse direction at ends - side effect - reversed curves are fixed in this call
          if (!pEU->m_pUVTrimCurve->PassesValidityCheck(SM_VC_REVERSE_DIRECTION, eCheckFailed))
            {
#ifdef SM_DEBUG_CODE
              // draw 
              if(bDebugMe)
                {
                  pEU->m_pUVTrimCurve->Dump() ;
                }
#endif // SM_DEBUG_CODE
              SE(SM_ERR_WARNING);
            }
        } // end target edgeuse has a UVTrimCurve check
    } // end iter every edgeuse attached to this SmEdge

  return SM_SUCCESS;
} // end SmEdge::ValidateGeometry

/*******************************************************************//**
PURPOSE: Output edge graphics as a polyline, with optional knots Points
            optional controlPoint polygon, and optional neighbor 
            faceuses, edgeuses, and vertexuses. 

NOTES: 
NOTES: 
   Switch on crDisp bits to output different images as

   always                      // Output tessellation polyLine
                               
   if(m_bDrawKnots)            // Draw edge knotPoints (place surface xhatch lines on knot boundaries)
   if(m_bDrawPolygon)          // Draw curve->ControlPolygon
   if(m_bDrawControlPoints)    // Draw curve->ControlPoints
   if(m_bDrawCurvature)        // Draw curvature comb
   if(m_bDrawSpeed)            // Draw Edge Speed combs (tines = curvature direction with 1stDeriv magnitude)
   if(m_bDrawDerivatives)      // Draw 1st Derivative comb
   if(m_bDrawParameterization) // Draw Edge->Curve with sequence of sample points increasing in size 
                               //  and changing color from green to blue to show edge parameterization
                               //  and add sequence of edgeuses also changing
                               //  in size and color to show sector sequence
   if(m_bDrawNeighbors)        //  When drawing a vertex, edge, or face, also draw topology
                               //                topology objects connected to them.
   if(m_bDrawMicro)            //  When drawing a Vertex/Edge, Vertex/Face, or Edge/Face connection
                               //                also draw micro topology graphics.
   if(m_bDrawHighCountCurves)  // Draw high control point count curves in special color and thickness

***********************************************************************/
SmStatus SmEdge::OutputGraphics
 (const SmDisplayParameters & crDisp,     // in : graphic display parameters
  SmGfxArraySet             * pOptGfxSet) // i/o: used for OpenGL ES 2.0. When given, accumulate GfxVertexArrays not GL calls.
                                          //      NULL to ignore. default:[NULL]
 const
{
#ifdef SM_GFX_OUTPUT_CODE

  // locals
  ULONG ii ;
  SmCurve    *pCurve   = GetCurve();
  SmExtent1d  sEdgeIvl = GetInterval();

  // pass the call along to the curve
  pCurve->OutputGraphics(sEdgeIvl, crDisp, NULL, NULL, pOptGfxSet) ;

  // When asked draw Neighbor data, output sequence of connected edegeuses (removed: and their faces) 
  if(crDisp.m_bDrawNeighbors)
    {
      SmTArray<SmEdgeuse *>sEdgeuses ;
      SmTArray<SmFace *>sDoneFaces ;
      GetEdgeuses(sEdgeuses) ;
      ULONG lCnt;
      
      lCnt = sEdgeuses.GetSize() ;

      // save current color and PointSize
      SmVector3d sColor     = smgfx_GetOutputColor(pOptGfxSet) ;
      double     dPointSize = smgfx_OutputPointSize(1.0, pOptGfxSet) ;
      SmColorRuleType sRuleNumber = SM_CR_NEIGHBORMATE ;
      SmVector3d      sRuleColor ;

      for(ii=0;ii<lCnt;ii++)
        {
          SmEdgeuse * pEdgeuse = sEdgeuses[ii] ;
          SmFaceuse * pFaceuse = pEdgeuse ? pEdgeuse->GetFaceuse() : NULL ;
          SmFace    * pFace    = pFaceuse ? pFaceuse->GetFace() : NULL ;

          // first time we see a Face - toggle color
          if(!sDoneFaces.IsIn(pFace)) { if(sRuleNumber == SM_CR_NEIGHBOR)
                                             { sRuleColor  = smgfx_GetRuleColor(this, SM_CR_NEIGHBORMATE) ; 
                                               sRuleNumber = SM_CR_NEIGHBORMATE ;
                                             }
                                        else { sRuleColor  = smgfx_GetRuleColor(this, SM_CR_NEIGHBOR) ; 
                                               sRuleNumber = SM_CR_NEIGHBOR ;
                                             }
                                      }            

          // draw Edgeuse and Faceuse
          if(pEdgeuse)                   { smgfx_SetColor(sRuleColor, pOptGfxSet) ; pEdgeuse->Draw(1.0, FALSE, pOptGfxSet) ; }   // FALSE = don't draw UVTrimCurves
          if(   pFace
             && !sDoneFaces.IsIn(pFace)) { smgfx_SetColor(sRuleColor, pOptGfxSet) ; pFace->DrawUV() ; 
                                           smgfx_SetColor(sRuleColor, pOptGfxSet) ; pFace->Draw(SM_DM_NORMALS) ;
                                           sDoneFaces.Add(pFace) ;  
                                         }
        } // end iter every edgeuse

      // restore current color and PointSize
      smgfx_SetColor(sColor, pOptGfxSet) ;
      smgfx_OutputPointSize(dPointSize, pOptGfxSet) ;

    } // end Draw m_bDrawNeighbors check

  // When asked draw Micro
  if(crDisp.m_bDrawMicro)
    {
      // locals
      SmTArray<SmEdgeuse*> sEdgeuses ;
      GetEdgeuses(sEdgeuses) ;
      double        dMicroEdgeParam =   crDisp.m_dMicroEdgeParam != SM_BIG_DOUBLE
                                      ? crDisp.m_dMicroEdgeParam
                                      : sEdgeIvl.Evaluate(0.5) ;
      ULONG         lTotalCount     = crDisp.m_lMicroEdgeSampleCount ;
      ULONG         lSameSizeCount  = 5 ;
      SmApproxTol3d sApproxTol3d    = SmTol::GetApproxTol3d(this) ;
      double        dStepInc        = 2.0 ;
      double        dMinStep        = SmTol::MapTo1d(sApproxTol3d,     // in : 3d Tol value to map to Param Space
                                                     dMicroEdgeParam,  // in : point on Curve
                                                     *pCurve) ;        // in : Curve mapping ParamSpace to 3dSpace
      double        dMaxStep        = sEdgeIvl.GetLength() / (double)lTotalCount ;
      if(dMinStep > dMaxStep) { dMinStep = dMaxStep ; }
      double        dStep, dEndParam ;

      // when there are multiple edgeuses
      if(sEdgeuses.GetSize() > 1)
        {
          // for every other Edgeuse (don't draw mates twice)
          for(ii=0;ii<sEdgeuses.GetSize();ii+=2)
            {
              SmEdgeuse * pEdgeuse = sEdgeuses[ii] ;

              // pass the call along to the Edgeuse
              pEdgeuse->DrawMicro(&dMicroEdgeParam, pOptGfxSet) ;

            } // end iter every other Edgeuse
        } // end Not a wire branch
      else // Just 1 Edgeuse branch - just MicroDraw the Edge
        {
          // locals
          SmCurve * pCrv = GetCurve() ;
          double    dS ;

          // Micro Sample Locals
          SmTArray<SmPoint3d> sEdgePoints, sEndEdgePoints, *pTgtPoints ;

          // begin Build Micro Samples scope
            {
              // locals
              SmPoint3d sEdgePoint ;

              // for two passes - 0 = [MicroEdgeParam IvlStart], 1 = [MicroEdgeParam IvlEnd] 
              for(ii=0;ii<2;ii++)
                {
                  ULONG jj = 0 ;

                  if(ii==0) { dS         = dMicroEdgeParam ;
                              pTgtPoints = &sEdgePoints ;
                              dStep      = -dMinStep ; 
                              dEndParam  =  sEdgeIvl.GetMin() ;
                            }
                  else      { dS         = sEdgeIvl.SnapValue(dMicroEdgeParam + dMinStep, dMinStep) ;
                              pTgtPoints = &sEndEdgePoints ; 
                              dStep      =  dMinStep ; 
                              dEndParam  =  sEdgeIvl.GetMax() ;
                            }

                  // sample the Curve in micro steps = ApproxTol3d
                  while(sEdgeIvl.ContainsValue(dS))
                    {
                      // Curve evaluate EdgePt, DropSurfPt, and DropSurfBiNormDir 
                      pCrv->EvaluatePoint(dS,               // in : value within Edge->m_vInterval.
                                          sEdgePoint) ;     // out: 3D pt on edge
 
                      // load gfx arrays
                      pTgtPoints->Add(sEdgePoint) ;

                      // exit case
                      if(dS == dEndParam)
                        { break ; }

                      // increment dS - start with dMinStep sized increments for lSameSizeCount samples
                      //                then increase step size every lSameSizeCount by dStepInc multiple
                      //                until step size == dMaxStep
                      jj = (jj+1)%lSameSizeCount ;
                      if(jj==0 && smos_Fabs(dStep) < dMaxStep) { dStep *= dStepInc ; 
                                                                 if(smos_Fabs(dStep) > dMaxStep)
                                                                   { dStep = smos_Sgn(dStep) * dMaxStep ; }
                                                               }
                      dS = sEdgeIvl.SnapValue(dS + dStep, dStep) ; 

                    } // end while sampling the Edge
                } // end iter 2 passes - 0 = [MicroEdgeParam IvlStart], 1 = [MicroEdgeParam IvlEnd]

              // combine the two points sets into one, reversing the StartEdgePoints
              sEdgePoints.ReverseArray(0,sEdgePoints.GetSize()) ;
              sEdgePoints.Append(sEndEdgePoints) ;

            } // end Build Micro Samples scope
    
          // Set MicroEdge color = red and LineWidth >= 2.0
          smgfx_OutputColor(1.0, 0.0, 0.0, pOptGfxSet) ;
          if(smgfx_GetOutputLineWidth(pOptGfxSet) < 2.0) 
            { smgfx_OutputLineWidth(2.0, pOptGfxSet) ; }

          // Draw the MicroEdge 
          smgfx_DrawPolyline((double *)sEdgePoints.GetDataArray(), sEdgePoints.GetSize(), pOptGfxSet) ;

        } // end a wire branch

    } // end Draw m_bDrawMicro check

#else
  SM_REF2(crDisp, pOptGfxSet);
#endif // SM_GFX_OUTPUT_CODE

  return SM_SUCCESS;

} // end SmEdge::OutputGraphics

/*************************************************************
PURPOSE: Add transformed edge, displayed as a polyline,
         to new displayList added to global displayList array.

NOTES: This is expensive because the Edge->Curve
         is copied, translated, displayed, and freed. 
**************************************************************/
SmDisplayList * SmEdge::DrawWTransform
  (const SmAxis2Placement & crTransform) 
 const
{
  SmDisplayList *pRtn = NULL ;

#ifdef SM_GFX_CODE

  // no work - no curve
  if (!m_pCurve) return(NULL) ;

  // Get global SmGraphicsExtern.cpp:s_Disp display parameters
  const SmDisplayParameters &rDisp = smgfx_RefGlobalDisplayParameters() ;

  // transform a temporary copy of the edge->Curve
  SmBSplineCurve *pNewCurve = new (*GetContext()) SmBSplineCurve(*SM_CAST_PTR(SmBSplineCurve,m_pCurve));
  SmObjDelete sCleanup(pNewCurve);
  pNewCurve->Transform(crTransform);

  // open new displayList (unless one is already open)
  SmVector3d sColor = smgfx_GetOutputColor() ;
  smgfx_Open(smgfx_GetRuleColor(this));

  // add transformed curve graphics
  pNewCurve->OutputGraphics(m_vInterval, rDisp, NULL) ;

  // end display list
  smgfx_OutputColor(sColor) ;
  pRtn = smgfx_Close() ;

#else
  SM_REF1(crTransform);
#endif
  return(pRtn) ;

} // end SmEdge::DrawWTransform

/*************************************************************
PURPOSE: Add edge, drawn as a polyline, to new DisplayList 
            added to global display list array.

NOTES:
**************************************************************/
SmDisplayList * SmEdge::Draw
 (SmGfxArraySet * pOptGfxSet)      // i/o: used for OpenGL ES 2.0. When given, accumulate GfxVertexArrays not GL calls.
                                   //      NULL to ignore. default:[NULL]
 const
{
  SmDisplayList *pRtn = NULL ;

#ifdef SM_GFX_CODE

  // no work - no curve
  if (!m_pCurve) return(NULL);
  
  // Get global SmGraphicsExtern.cpp:s_Disp display parameters
  const SmDisplayParameters &rDisp = smgfx_RefGlobalDisplayParameters() ;

  // start new displayList (unless one is already open)
  SmVector3d sColor = smgfx_GetOutputColor(pOptGfxSet) ;
  smgfx_Open(smgfx_GetRuleColor(this), NULL, NULL, FALSE, pOptGfxSet);

  // output the graphics
  OutputGraphics(rDisp, pOptGfxSet) ;

  // end display list
  smgfx_OutputColor(sColor, pOptGfxSet) ;
  pRtn = smgfx_Close(pOptGfxSet) ;

#else
  SM_REF1(pOptGfxSet);
#endif // SM_GFX_CODE
  return(pRtn) ;

} // end SmEdge::Draw

/*************************************************************
PURPOSE: Add edge, drawn as a polyline, to new DisplayList 
            and Draw all edgeuses in order increasing in size and
            ranging in color from blue to green to show the 
            ordering of curve sectors.

NOTES:
**************************************************************/
SmDisplayList * SmEdge::DrawNeighbors
 (SmGfxArraySet * pOptGfxSet)      // i/o: used for OpenGL ES 2.0. When given, accumulate GfxVertexArrays not GL calls.
                                   //      NULL to ignore. default:[NULL]
 const
{
  SmDisplayList *pRtn = NULL ;

#ifdef SM_GFX_CODE

  // no work - no curve
  if (!m_pCurve) return(NULL);
  
  // Get global SmGraphicsExtern.cpp:s_Disp display parameters
  SmDisplayParameters sDisp ;
  smgfx_GetGlobalDisplayParameters(sDisp) ;

  // override display values
  sDisp.m_bDrawParameterization = TRUE ;
  sDisp.m_bDrawNeighbors        = TRUE ;
    
  // start new displayList (unless one is already open)
  SmVector3d sColor = smgfx_GetOutputColor() ;
  smgfx_Open(smgfx_GetRuleColor(this), NULL, NULL, FALSE, pOptGfxSet);

  // output the graphics
  OutputGraphics(sDisp, pOptGfxSet) ;

  // end display list
  smgfx_OutputColor(sColor, pOptGfxSet) ;
  pRtn = smgfx_Close(pOptGfxSet) ;

#else
  SM_REF1(pOptGfxSet);
#endif
  return(pRtn) ;

} // end SmEdge::DrawNeighbors

/*************************************************************
PURPOSE: draw micro view of geometry connecting to this edge

NOTES:
**************************************************************/
SmDisplayList * SmEdge::DrawMicro
 (double        * pOptMicroParam,  // in : optional Edge param to be center of Micro Drawing
  SmGfxArraySet * pOptGfxSet)      // i/o: used for OpenGL ES 2.0. When given, accumulate GfxVertexArrays not GL calls.
                                   //      NULL to ignore. default:[NULL]
 const
{
  SmDisplayList *pRtn = NULL ;

#ifdef SM_GFX_CODE

  // no work - no curve
  if (!m_pCurve) return(NULL);

  // Get global SmGraphicsExtern.cpp:s_Disp display parameters
  SmDisplayParameters sDisp ;
  smgfx_GetGlobalDisplayParameters(sDisp) ;

  // override display values
  sDisp.m_bDrawMicro = TRUE ;
  if(pOptMicroParam && GetInterval().ContainsValue(*pOptMicroParam))
    {
      sDisp.m_dMicroEdgeParam = *pOptMicroParam ; 
    }
    
  // start new displayList (unless one is already open)
  SmVector3d sColor     = smgfx_GetOutputColor() ;
  double     dLineWidth = smgfx_GetOutputLineWidth(pOptGfxSet) ;
  smgfx_Open(smgfx_GetRuleColor(this), NULL, NULL, FALSE, pOptGfxSet);

  // output the graphics
  OutputGraphics(sDisp, pOptGfxSet) ;

  // end display list
  smgfx_OutputColor(sColor, pOptGfxSet) ;
  smgfx_OutputLineWidth(dLineWidth, pOptGfxSet) ;
  sDisp.m_dMicroEdgeParam = SM_BIG_DOUBLE ;
  pRtn = smgfx_Close(pOptGfxSet) ;

#else
  SM_REF2(pOptMicroParam, pOptGfxSet);
#endif
  return(pRtn) ;

} // end SmEdge::DrawMicro

/*******************************************************************//**
PURPOSE: A generic routine to test if an edge is filletable.
         The returned edgeuse->RadialMate is connected to the second face of the fillet.
NOTES:
   Returns Edgeuse for the edge, if filletable, else Null.
   If this Edge is Lamina, not filletable, return Null.
   If this Edge is tangent (to within 2 degrees), not filletable, return Null.
   If this Edge separates two Regions (e.g., manifold),
     return an Edgeuse that does not point into the infinite region,
     and has SM_OT_SAME orientation.

RETURN:
  edgeuse specifying a concave sector for shell geometries and
  edgeuse specifying a sector in an interior region for solid geometries
***********************************************************************/
SmEdgeuse * SmEdge::GetBlendEdgeuse() const
{
  if(IsLamina())
    return NULL; // skip all lamina edges

  SmEdgeuse *pEU0 = GetPrimaryEdgeuse();
  SmEdgeuse *pEU1 = pEU0->GetMate();
  SmBoolean bIsShellFilleting = FALSE;

  // get an edgeuse with SM_OT_SAME orientation (from an interior region when working on a solid)
  if(pEU0->GetShell()->GetRegion() == pEU1->GetShell()->GetRegion())
  {
    bIsShellFilleting = TRUE;
    if(pEU0->GetOrientation() == SM_OT_OPPOSITE)
    {
      pEU0 = pEU0->GetMate();
    }  // shell filleting

    if(pEU0 == NULL)
    {
      return NULL;
    }
  } // end working on a shell branch
  else
  {
    // primary and mate edgeuses are in different regions
    SmShell *pShell = pEU0->GetShell();
    SmRegion * pInfiniteReg = pShell->GetBrep()->GetInfiniteRegion();

    // get edgeuse for interior region
    if(pShell->GetRegion() == pInfiniteReg)
    {
      pEU0 = pEU0->GetMate();
    }

    if(pEU0->GetOrientation() == SM_OT_OPPOSITE)
    {
      pEU0 = pEU0->GetRadial();
    }

    // check state - edgeuse should be part of interior region with SM_OT_SAME orientation
    if(!pEU0 || pEU0->GetShell()->GetRegion() == pInfiniteReg || pEU0->GetOrientation() == SM_OT_OPPOSITE)
    {
      return NULL;
    }
  } // working on a solid branch

    // get radial mate to selected edgeuse
    // pEU0 and peU1 now bound the sector to be filleted
  if(pEU0 == NULL)
  {
    return NULL;
  }

  pEU1 = pEU0->GetRadial();

  // Test to determine if edge bounds two tangential faces
  // test for smooth edge - don't fillet edges within 2 degrees of tangent
  double dTestParam = this->GetInterval().Evaluate( 0.34 );
  SmPoint3d sPnt, sPnt2;

  // get normalized normal and binormals from the two edgeuses at a common point
  SmVector3d sBinVec2, sBinVec, sFaceuseNormal, sFaceuseNormal2;
  SE( pEU0->EvaluateBinormal( dTestParam, FALSE, sPnt, sBinVec, NULL, &sFaceuseNormal ) );
  SE( pEU1->EvaluateBinormal( dTestParam, FALSE, sPnt2, sBinVec2, NULL, &sFaceuseNormal2 ) );
  SE( sFaceuseNormal.Unitize() );
  SE( sFaceuseNormal2.Unitize() );
  SE( sBinVec.Unitize() );
  SE( sBinVec2.Unitize() );

  // set max nonTangent angle to 88 degrees
  double dMaxDotValue = smos_Cosine( 88.0*SM_PI / 180.0 );

  // when 1st edgeuse surface normal is perpendicular to 2nd edgeuse binormal or 
  //      1st edgeuse binormal       is perpendicular to 2nd edgeuse surface normal
  // the faces are tangent and can't be filleted
  double dDot1 = sBinVec2.Dot( sFaceuseNormal );
  if(smos_Fabs( dDot1 ) < dMaxDotValue)
  {
    return NULL; // Tangent: unable to fillet
  }

  double dDot2 = sBinVec.Dot( sFaceuseNormal2 );
  if(smos_Fabs( dDot2 ) < dMaxDotValue)
  {
    return NULL; // Tangent: unable to fillet
  }

  // Same region on both sides
  // when filleting a shell (not a solid) - get a sector which is not convex
  if(bIsShellFilleting)
  {
    double dStepoffDistance = 0.001;
    SE( pEU0->EvaluateBinormalStepOff( dTestParam, dStepoffDistance, sPnt, sBinVec, NULL, &sFaceuseNormal ) );
    SE( pEU1->EvaluateBinormalStepOff( dTestParam, dStepoffDistance, sPnt2, sBinVec2, NULL, &sFaceuseNormal2 ) );
    SmVector3d sVec = sPnt2 - sPnt;
    SE( sVec.Unitize() );
    SE( sFaceuseNormal.Unitize() );
    dDot1 = sVec.Dot( sFaceuseNormal );
    sVec = -sVec;

    SE( sFaceuseNormal2.Unitize() );

    // when dot products are both negative, sector is convex 
    // switch to sector on other side of pEU0
    dDot2 = sVec.Dot( sFaceuseNormal2 );
    if(dDot1 < 0.0 && dDot2 < 0.0)
    {
      pEU0 = pEU0->GetMate();
      if(pEU0->GetOrientation() == SM_OT_OPPOSITE)
      {
        pEU0 = pEU0->GetRadial();
      }
      pEU1 = pEU0->GetRadial();
    } // end need to switch sectors check
  } // end when filleting a shell check

#ifdef SM_DEBUG_CODE
  SmBoolean bDebugMe = FALSE;
  if(bDebugMe)
  {
    pEU0->Draw();
    pEU1->Draw();
    sFaceuseNormal.Draw( &sPnt );
    sFaceuseNormal2.Draw( &sPnt2 );
    sm_GraphicsLoop();
  }
#endif

  return pEU0;

} // end SmEdge::GetBlendEdgeuse

/*******************************************************************//**
PURPOSE: Compute the total size of the memory used by the SmEdge,
            its curve, all its SmEdgeuses, and their attributes.

NOTES:
***********************************************************************/
ULONG SmEdge::GetMemoryUsed       // rtn: smaller size of actually used memory in bytes
  (ULONG    & rlMemoryAllocated,  // out: bigger size of all allocated memory in bytes
   SmMarkType eMarkType)          // in : uses without increment eMarkType value
  const
{
  // in case this method is called directly - get a mark for attribute memory usage
  SmNewMarkAndLock sMarkLock ;
  if(eMarkType == SM_MT_NOMARK)
    {
      eMarkType = sMarkLock.SetContext((SmContext *)GetContext()) ;
    }

  // init output and return
  rlMemoryAllocated = 0 ;
  ULONG lUsed       = 0 ;

  // locals
  ULONG ii, lThisAllocated ;

  // this + attribute memory
  lUsed             +=   sizeof(*this) 
                       + this->GetAttributeMemoryUsed(lThisAllocated, 
                                                      eMarkType) ;  // note: uses without increment eMarkType value
  rlMemoryAllocated +=   sizeof(*this) + lThisAllocated;

  // curve memory
  lUsed             += m_pCurve->GetMemoryUsed(lThisAllocated, 
                                               eMarkType) ;  // note: uses without increment eMarkType value
  rlMemoryAllocated += lThisAllocated ;
  lUsed             += m_pCurve->GetAttributeMemoryUsed(lThisAllocated, 
                                                        eMarkType) ;  // note: uses without increment eMarkType value
  rlMemoryAllocated += lThisAllocated ;

  // for every Edgeuse - add in object and attribute memory
  SmEdgeuse *pEU, *pEdgeuse = (SmEdgeuse *)m_pList ;
  for(pEU  = pEdgeuse,   ii=0; 
      pEU != pEdgeuse || ii==0; 
      pEU  = (SmEdgeuse*)pEU->m_pNext, ii++)
    {
      // get edgeuse and its attribute memory
      lUsed             +=   sizeof(*pEU)
                           + pEU->GetAttributeMemoryUsed(lThisAllocated, 
                                                         eMarkType) ;  // note: uses without increment eMarkType value

      rlMemoryAllocated +=   sizeof(*pEU) 
                           + lThisAllocated ;

      // when Edgeuse has a UVTrimCurve - add in its memory
      SmBSplineCurve *pUVTrimCurve = pEU->GetUVTrimCurvePointer() ;
      if(pUVTrimCurve != NULL)
        {
          lUsed             += pUVTrimCurve->GetMemoryUsed(lThisAllocated, 
                                                           eMarkType) ;  // note: uses without increment eMarkType value
          rlMemoryAllocated += lThisAllocated ;
          lUsed             += pUVTrimCurve->GetAttributeMemoryUsed(lThisAllocated, 
                                                                    eMarkType) ;  // note: uses without increment eMarkType value
          rlMemoryAllocated += lThisAllocated ;

        } // end pUVTrimCurve existence check
    }

  // all done
  return(lUsed) ;

} // end SmEdge::GetMemoryUsed

/*******************************************************************//**
PURPOSE: AssertValid() Test reporting static labels
***********************************************************************/
SmAssertReportLabel sAssertEdge_list[] =
{
  /*  0 */ {SM_AT_POINTER,     _T("Bad Curve"),           _T("m_pCurve is NULL") },
  /*  1 */ {SM_AT_POINTER,     _T("Bad Context"),         _T("Edge and m_pCurve have different contexts") },
  /*  2 */ {SM_AT_DOMAIN,      _T("Bad Domain"),          _T("Edge m_vInterval not contained in m_pCurve->NaturalInterval") },
  /*  3 */ {SM_AT_POINTER,     _T("Bad Primary EU"),      _T("Edge m_pList (its primary EU) is NULL") },
  /*  4 */ {SM_AT_NESTED_TEST, _T("Bad SubTopology"),     _T("An Edge SubTopology Vertex failed its AssertValid check") },

  /*  5 */ /* obsolete */ {SM_AT_GEOMETRIC, _T("Bad Edge Tol"), _T("Edge Tolerance less than Brep Tol") },
  /*  6 */ /* obsolete */ {SM_AT_GEOMETRIC, _T("Bad Edge Tol"), _T("Edge Tolerance greater than 1000 times Brep Tol") },

  /*  7 */ {SM_AT_GEOMETRIC,   _T("Bad Edge Length"),     _T("Edge Length less than Edge tolerance") },

  /*  8 */ {SM_AT_TOPOLOGICAL, _T("Bad Edgeuse TYPE"),    _T("Edgeuse of Manifold or Spine Edge is not TYPE SmLoopuse_TYPE") },

  /*  9 */ {SM_AT_ANGLE,       _T("Bad Sector Ang Sequence"),  _T("An Edge->Edgeuse/RadialEdgeuse pair of radial angles do not increase monotonically") },
  /* 10 */ {SM_AT_ANGLE,       _T("Bad Radial AngleDeg"),      _T("An Edge->Edgeuse/PrimaryEdgeuse sector AngleDeg not computed from 0.0 to 360.0") },
  /* 11 */ {SM_AT_TOPOLOGICAL, _T("Bad Vertex Ptrs"),          _T("Not all Edge->Edgeuse->Vertexuse->Vertices map to the same two vertices") },

  /* 12 */ {SM_AT_GEOMETRIC,   _T("Bad Edge Tol"),        _T("stored Vertex ZoneTol not equal the SmTol::Consistent Tolerance Model value") },
  /* 13 */ {SM_AT_GEOMETRIC,   _T("Bad Edge loc"),        _T("Edge not within XSectTol3d of all Face->Surfaces") },

  /* 14 */ { SM_AT_DIRECTION,   _T( "Bad Edgeuse Orientations" ), _T( "Edgeuses do not follow same-opposite-same-opposite ordering" ) },

  /* 15 */ {SM_AT_POINTER,     _T("Edge Vertex Cnt"),   _T("Closed edges must have one vertex, open edges 2") },
  /* 16 */ {SM_AT_GEOMETRIC,   _T("Edge/Vertex Gap"),   _T("start edge/vertex gap must be less than edge + vertex tolerance") },
  /* 17 */ {SM_AT_GEOMETRIC,   _T("Edge/Vertex Gap"),   _T("end edge/vertex gap must be less than edge + vertex tolerance") }
} ; // end sAssertEdge_list[] 

/*******************************************************************//**
AssertRule Predicate names and return values:  
  predicate = SmBoolean sm_AssertTestClassNameRule#() ; Rtn: TRUE=OK,    FALSE=Bad, 
***********************************************************************/

/*******************************************************************//**
PURPOSE  : SmEdge AssertRule 14 - 
  The Edge->Edgeuses should follow Same-Opposite-Same-Opposite order

  predicate = return TRUE when Edgeuses follow expected order
             
  action    = 
***********************************************************************/
SmBoolean sm_AssertTestEdge14 // rtn: TRUE = okay, FALSE = problem
( const SmEdge * pEdge )      // in : test target
{
    SmBoolean bRtn = TRUE;

    // locals
    SmEdgeuse *pEdgeuse = pEdge->GetPrimaryEdgeuse(); 
    if ( pEdgeuse == NULL )
    { return TRUE; }

    SmEdgeuse *pThisEdgeuse = pEdgeuse;
    SmEdgeuse *pMateEdgeuse = pThisEdgeuse->GetMate();
    SmEdgeuse *pMateRadialEdgeuse = pMateEdgeuse->GetRadial();

    while ( TRUE )
    {
        if (    pThisEdgeuse == NULL 
             || pMateEdgeuse == NULL
             || pMateRadialEdgeuse == NULL )
        { break; }

        bRtn &= pThisEdgeuse->GetOrientation() != pMateEdgeuse->GetOrientation();
        bRtn &= pMateEdgeuse->GetOrientation() != pMateRadialEdgeuse->GetOrientation();

        // Get next set of EUs to compare
        pThisEdgeuse = pMateRadialEdgeuse;
        pMateEdgeuse = pThisEdgeuse->GetMate();
        pMateRadialEdgeuse = pMateEdgeuse->GetRadial();

        if ( ( pThisEdgeuse == pEdgeuse ) || !bRtn )
        { break; }
    } 

    return bRtn;

} // end sm_AssertTestEdge14

/*******************************************************************//**
PURPOSE  : SmEdge AssertRule 13 - 
  When Edge is connected to two faces that intersect,
  Edge should be placed tightly on the edge->faces' surf/surf intersection point.

  predicate = return TRUE when Edge is not connected to 2 distinct faces or
                          when Edge->m_pCurve is within SmTol::GetApproxTol3d(pEdge)
                          of the surf/surf intersection curve.
             
  action    = when Edge is connected to 2 distinct faces
              move Edge->m_pCurve to surf/surf intersection,
              otherwise don't move the Edge->m_pCurve 
***********************************************************************/
SmBoolean sm_AssertTestEdge13         // rtn: TRUE = okay, FALSE = problem
 (const SmEdge * pEdge,               // in : test target
  double       * pdOptMaxGap3d=NULL)  // out: Max Gap3d between Edge->Curve and all connected Faces
{ 
  // bRtn == FALSE when Vertex Point is more than ScaledZero from best corner pos defined by connected faces
  SmBoolean bRtn = pEdge->IsOnFaceEdgeForm(pdOptMaxGap3d) ;

  // all done
  return( bRtn ) ;
    
} // end sm_AssertTestEdge13

// obsolete
// /*******************************************************************//**
// PURPOSE  : SmEdge AssertRule 13 = Place Edge on surf/surf XSect
// ***********************************************************************/
// SmBoolean sm_AssertHealEdge13
//  (SmEdge         * pEdge, 
//   SmAssertReport & rAReport,  
//   SmAssertArray  * pAList) 
// { 
//   // remember Heal has run on this AssertReport
//   rAReport.m_eAssertType = SM_AT_HEALER ;
// 
//   // locals
//   double            dMaxEdgeFaceGap3d, dNewMaxGap3d ;   // Edge->m_pCurve to Face->Surf MaxGap3d
//   const SmContext * cpContext   = pEdge->GetContext() ;
// 
//   // Edge/Face XSectTol3d without gaps
//   SmXSectTol3d  sXSectTol3d  = SmTol::GetXSectTol3d(pEdge, pEdge) ; 
//   SmApproxTol3d sApproxTol3d = SmTol::GetApproxTol3d(cpContext) ; 
// 
//   // check test - other fix functions may have already fixed this one
//   rAReport.m_bOK = sm_AssertTestEdge13(pEdge, &dMaxEdgeFaceGap3d) ;
//   if(TRUE == rAReport.m_bOK)
//     { return(rAReport.m_bOK) ; }
// 
//   // fix broken stored ZoneTol3d value - by replacing it with the SmTol::GetZoneTol3d() value.
//   if(dMaxEdgeFaceGap3d > sXSectTol3d)
//     { 
//       // locals
//       SmBoolean        bFoundFaceEdgeForm ;  // TRUE = Edge faces define an XSecting EdgeForm
//       SmCurve        * pXSectCurve  = NULL ; // Surf/Surf intersection point
//       SmFace         * pFace1=NULL,        * pFace2=NULL ;
//       SmBSplineCurve * pUVTrimCurve1=NULL, * pUVTrimCurve2=NULL ;
//       SmPoint3d        sBestStartVertexPos,  sBestEndVertexPos ;  
// 
//       // try to calc a better intersection curve
//       pEdge->CalcEdgeFormFromFaces(bFoundFaceEdgeForm,
//                                    dNewMaxGap3d,
//                                    pXSectCurve,
//                                    sBestStartVertexPos,
//                                    sBestEndVertexPos, 
//                                    &pFace1,
//                                    &pUVTrimCurve1,
//                                    &pFace2, 
//                                    &pUVTrimCurve2) ;
// 
//       // when possible - fix broken Edge by replacing the Edge->Curve
//       if(bFoundFaceEdgeForm)
//         {
//           // do no harm
//           if(dNewMaxGap3d < dMaxEdgeFaceGap3d)
//             {
//               SmCurve  * pOldCurve = pEdge->GetCurve() ;
//               SmExtent1d sIvl      = pEdge->GetInterval() ;
// 
//               // set XSectCurve->Ivl = origCurve->Ivl
//               pXSectCurve->EditParameterization(sIvl) ;
// 
//               // replace Edge->Curve
//               pEdge->Replace3DCurve(pXSectCurve) ;
// 
//               // Good Citizenship - log all Object changes with pAList here: 
//               //                      pAList->LogReplaceObject(), 
//               //                      pAList->LogSplitObject(),  
//               //                      pAList->LogMergeObject(),  
//               //                      pAList->LogDeleteObject()
//               pAList->LogReplaceObject(pOldCurve, pXSectCurve) ;
// 
//               // replace Edge->Edgeuse(Face1)->UVTrimCurve 
//               if(pFace1 && pUVTrimCurve1)
//                 {
//                   pUVTrimCurve1->EditParameterization(sIvl) ;
// 
//                   SmEdgeuse * pEdgeuse     = pEdge->GetEdgeuseOfFace(pFace1) ;
//                   if(pEdgeuse)
//                   {
//                     SmCurve   * pUVTrimCurve = pEdgeuse->GetUVTrimCurve();
//                     pEdgeuse->SetUVTrimCurve( pUVTrimCurve1, dNewMaxGap3d, TRUE ); // TRUE = delete existing UVTrimCurve
// 
//                     pAList->LogReplaceObject( pUVTrimCurve, pUVTrimCurve1 );
//                   }
//                 }                   
// 
//               // replace Edge->Edgeuse(Face1)->UVTrimCurve 
//               if(pFace2 && pUVTrimCurve2)
//                 {
//                   pUVTrimCurve2->EditParameterization(sIvl) ;
//                   
//                   SmEdgeuse * pEdgeuse = pEdge->GetEdgeuseOfFace(pFace2) ;
//                   if(pEdgeuse)
//                   {
//                     SmCurve   * pUVTrimCurve = pEdgeuse->GetUVTrimCurve();
//                     pEdgeuse->SetUVTrimCurve( pUVTrimCurve2, dNewMaxGap3d, TRUE ); // TRUE = delete existing UVTrimCurve
// 
//                     pAList->LogReplaceObject( pUVTrimCurve, pUVTrimCurve2 );
//                   }
//                 }
// 
//               // no need to check the change - we know that this problem is fixed
//               // rAReport.m_bOK = sm_AssertTestEdge13(pEdge, &dMaxGap3d) ;
//               rAReport.m_bOK = TRUE ;
// 
//             } // end dMaxGap3d okay check
// 
//           // Update Edge vertex positions when current gaps exceed sApproxTol3d
//           SmVertex * pStartVertex, * pEndVertex ;
//           pEdge->GetVertices(pStartVertex, pEndVertex) ;
//           double dStartGap = sBestStartVertexPos.DistanceBetween(pStartVertex->GetPoint()) ;
//           double dEndGap   = sBestEndVertexPos.DistanceBetween(pEndVertex->GetPoint()) ; 
// 
//           if(dStartGap > sApproxTol3d) { pStartVertex->SetPoint(sBestStartVertexPos) ; }
//           if(dEndGap   > sApproxTol3d) { pEndVertex->SetPoint(sBestEndVertexPos) ; }
// 
//         } // end Edge->Faces XSect to create an EdgeForm check
//     } // end Edge/Face MaxGap3d is bigger than ApproxTol3d check                   
// 
//   // all done
//   if(rAReport.m_bOK) { rAReport.m_pHealMessage = _T("Bad SmEdge::Curve too far from faces replaced with attached Surf/Surf intersection curve.") ; }
//   else               { rAReport.m_pHealMessage = _T("Bad SmEdge::Curve too far from faces could not be fixed - Edge not attached to 2 intersecting faces") ; }
//   return(rAReport.m_bOK) ;
// 
// } // end sm_AssertHealEdge13
// end obsolete

/*******************************************************************//**
PURPOSE  : SmEdge AssertRule 12 - 
  Edge's stored ZoneTol must equal SmTol::GetZoneTol3d() Consistent Tolerance Model value

  predicate = if(SM_USE_NEWTOL) return TRUE
              else return SM_ARE_SAME(m_sZoneTol3d, SmTol::GetZoneTol3d(this))
             
  action    = if(SM_USE_NEWTOL) no action
              else if m_sZoneTol3d not equal SmTol::GetZoneTol3d(this) 
                      set m_sZoneTol3d = SmTol::GetZoneTol3d(this)  
***********************************************************************/
SmBoolean sm_AssertTestEdge12        // rtn: TRUE = okay, FALSE = problem
 (const SmEdge * pEdge,              // in : test target
  SmZoneTol3d  * pOptZoneTol3d=NULL) // out: opt SmTol::GetZoneTol3d(pEdge) value, NULL to ignore 
{ 
  SmBoolean bRtn = pEdge->IsZoneTol3dConsistent(pOptZoneTol3d) ;

  return( bRtn ) ;
    
} // end sm_AssertTestEdge12

// obsolete
// /*******************************************************************//**
// PURPOSE  : SmEdge AssertRule 12 - Edge's stored ZoneTol must equal 
//            SmTol::GetZoneTol3d() Consistent Tolerance Model value
// ***********************************************************************/
// SmBoolean sm_AssertHealEdge12
//  (SmEdge         * pEdge,     // in :
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
//   rAReport.m_bOK = sm_AssertTestEdge12(pEdge, &sZoneTol3d) ;
//   if(TRUE == rAReport.m_bOK)
//     { return(rAReport.m_bOK) ; }
// 
//   // fix broken stored ZoneTol3d value - by replacing it with the SmTol::GetZoneTol3d() value.
//   pEdge->SetTolerance(sZoneTol3d, FALSE) ;  // FALSE = always update m_sZoneTol3d
// 
//   // no need to check the change - we know that this problem is fixed
//   // rAReport.m_bOK = sm_AssertTestEdge0(pEdge, &sZoneTol3d) ;
//   rAReport.m_bOK = TRUE ;
// 
//   // Good Citizenship - log all Object changes with pAList here:
//   //                      pAList->LogReplaceObject(), 
//   //                      pAList->LogSplitObject(),  
//   //                      pAList->LogMergeObject(),  
//   //                      pAList->LogDeleteObject().
//   // no object changes to log
// 
//   // all done
//   if(rAReport.m_bOK) { rAReport.m_pHealMessage = _T("InConsistent SmEdge::ZoneTol3d value reset to SmTol::GetZoneTol3d(pEdge) value.") ; }
//   else               { rAReport.m_pHealMessage = _T("InConsistent SmEdge::ZoneTol3d value could not be fixed - Something went wrong, report as bug") ; }
//   return(rAReport.m_bOK) ;
// 
// } // end sm_AssertHealEdge12
// end obsolete

/*******************************************************************//**
PURPOSE: AssertTest Rule 2 - Edge m_vInterval is contained in m_pCurve->NaturalInterval
NOTES:
   predicate = Is pEdge->Interval completely contained within pEdge->GetCurve->NaturalInterval
   action    = Drop Edge->Vertex->Points to Curve and build new interval
                - When Drops fail, No Fix Possible
***********************************************************************/
SmBoolean sm_AssertTestEdge2(SmEdge *pEdge) // TRUE = okay, FALSE = problem
 { SmExtent1d sEdgeIvl  = pEdge->GetInterval() ;
   SmExtent1d sCurveIvl = pEdge->GetCurve()->GetNaturalInterval() ; 
   return( sEdgeIvl.IsContainedBy(sCurveIvl) ) ; 
 }

// obsolete
// /*******************************************************************//**
// PURPOSE: AssertHeal Rule 2 - Edge m_vInterval is contained in m_pCurve->NaturalInterval
// NOTES:
//    predicate = Is pEdge->Interval completely contained within pEdge->GetCurve->NaturalInterval
//    action    = Drop Edge->Vertex->Points to Curve and build new interval
//                 - When Drops fail, No Fix Possible
// ***********************************************************************/
// SmBoolean sm_AssertHealEdge2(SmEdge *pEdge) 
// { // if(Edge->Vertices Drop to Curve) {reset Curve interval } else {return FALSE ; }
//   if(sm_AssertTestEdge2(pEdge))
//     { 
//       // locals
//       SmExtent1d   sIvl         = pEdge->GetCurve()->GetNaturalInterval() ; 
//       SmVertex   * pStartVertex = pEdge->GetStartVertex() ;
//       SmVertex   * pEndVertex   = pEdge->GetOtherVertex(pStartVertex) ; 
//       SmPoint3d    sStartPt     = pStartVertex->GetPoint() ;
//       SmPoint3d    sEndPt       = pEndVertex->GetPoint() ;
//       SmXSectTol3d sXSectTol3d1 = SmTol::GetXSectTol3d(pStartVertex, pEdge) ;
//       SmXSectTol3d sXSectTol3d2 = SmTol::GetXSectTol3d(pEndVertex, pEdge) ;
//       SmBoolean    bSuccess1, bSuccess2 ;
//       double       dParam1 = 0.0,   dParam2 = 0.0,  dDropDist3d1 = 0.0, dDropDist3d2 = 0.0;
// 
//       // Drop pEdge->StartVertex->Point to pEdge->Curve           
//       SmStatus     sRtn1        = pEdge->GetCurve()->DropPoint
//                                     (sIvl,             // in : target curve allowed domain
//                                      sStartPt,         // in : Point to drop to curve
//                                      NULL,             // in : For Snapping, NULL=NoSnap, Vec pointing to CrvInside from desired EndPt for snapping.
//                                                        //      when DropPt is within dDistTol in cpVecToCrvInside dir of EndPt, Snap to Ivl EndPt.                                                       
//                                                        //      Vec handles ambiguities and makes sure that curves are not just touching at the ends.
//                                      sXSectTol3d1,     // in : Used for SnapDist, and SM_SO_INTERSECT sXSectTol3d distance.
//                                                        //      sXSectTol3d = dDistTol ;      SM_SO_INTERSECT 
//                                                        //      sXSectTol3d = SM_BIG_DOUBLE ; SM_SO_MINIMIZE, SM_SO_NORMALIZE
//                                                        //      sSnapTol3d  = dDistTol ;      SM_SO_MINIMIZE, SM_SO_NORMALIZE, SM_SO_INTERSECT    
//                                      NULL,             // in : If given, call cheap LocalPointSolve() else call expensive GlobalPointSolve()
//                                      bSuccess1,        // out: TRUE = found a drop point
//                                      dParam1,          // out: found drop curve param
//                                      dDropDist3d1,     // out: found drop distance
//                                      SM_SO_MINIMIZE) ; // in : SM_SO_MINIMIZE = allow nonNormal drops near endPoints
//                                                        //      SM_SO_INTERSECT= like Minimize but point must be within dDistTol
//                                                        //      SM_SO_NORMALIZE= exclude nonNormal drops near endPoints
//                                                        //      default:[SM_SO_MINIMIZE] to preserve original behavior
//                                                        
//       // Drop pEdge->EndVertex->Point to pEdge->Curve           
//       SmStatus     sRtn2        = pEdge->GetCurve()->DropPoint
//                                     (sIvl,             // in : target curve allowed domain
//                                      sEndPt,           // in : Point to drop to curve
//                                      NULL,             // in : For Snapping, NULL=NoSnap, Vec pointing to CrvInside from desired EndPt for snapping.
//                                                        //      when DropPt is within dDistTol in cpVecToCrvInside dir of EndPt, Snap to Ivl EndPt.                                                    
//                                                        //      Vec handles ambiguities and makes sure that curves are not just touching at the ends.
//                                      sXSectTol3d2,     // in : Used for SnapDist, and SM_SO_INTERSECT sXSectTol3d distance.
//                                                        //      sXSectTol3d = dDistTol ;      SM_SO_INTERSECT 
//                                                        //      sXSectTol3d = SM_BIG_DOUBLE ; SM_SO_MINIMIZE, SM_SO_NORMALIZE
//                                                        //      sSnapTol3d  = dDistTol ;      SM_SO_MINIMIZE, SM_SO_NORMALIZE, SM_SO_INTERSECT    
//                                      NULL,             // in : If given, call cheap LocalPointSolve() else call expensive GlobalPointSolve()
//                                      bSuccess2,        // out: TRUE = found a drop point
//                                      dParam2,          // out: found drop curve param
//                                      dDropDist3d2,     // out: found drop distance
//                                      SM_SO_MINIMIZE) ; // in : SM_SO_MINIMIZE = allow nonNormal drops near endPoints
//                                                        //      SM_SO_INTERSECT= like Minimize but point must be within dDistTol
//                                                        //      SM_SO_NORMALIZE= exclude nonNormal drops near endPoints
//                                                        //      default:[SM_SO_MINIMIZE] to preserve original behavior
//                                                        
//       // error exit - unsuccessful drops
//       if(   sRtn1 != SM_SUCCESS || bSuccess1 != TRUE
//          || sRtn2 != SM_SUCCESS || bSuccess2 != TRUE)
//         { return FALSE ; }
// 
//       // arrive here - ready to create and save new Edge->Interval
//       if(dParam1 > dParam2) { // invert the Curve and all UVTrimCurve orientations
//                               pEdge->ReverseOrientation() ; 
//                               // map oldParams to inverted Param Range
//                               dParam1 = sIvl.GetMax() - dParam1 + sIvl.GetMin() ;
//                               dParam2 = sIvl.GetMax() - dParam2 + sIvl.GetMin() ;
//                               SM_ASSERT(dParam1 <= dParam2) ;
//                             }
//        // make and save new Edge Interval
//        SmExtent1d sNewIvl(dParam1, dParam2) ;
//        pEdge->SetInterval(sNewIvl) ; 
//     } // end Still broken check
// 
//   return TRUE ;  
// } // end sm_AssertHealEdge2
// end obsolete

/*******************************************************************//**
PURPOSE: AssertTest Rule 1 - SmEdge and its SmCurve need to share common contexts 
NOTES:
  predicate = Is pEdge->GetContext equal to pEdge->GetCurve->GetContext
  action    = pEdge->SetCurve = pEdge->GetCurve->CopyCurve(pEdge->GetContext) 
***********************************************************************/
SmBoolean sm_AssertTestEdge1(SmEdge *pEdge)  // TRUE = okay, FALSE = problem
{ return ( (pEdge->GetCurve() == NULL) || (pEdge->GetContext() == pEdge->GetCurve()->GetContext() ) ) ; }

// obsolete
// /*******************************************************************//**
// PURPOSE: AssertHeal Rule 1 - SmEdge and the objects it attaches to need to share common contexts 
// NOTES:
//   predicate = Is pEdge->GetContext equal to pEdge->GetCurve->GetContext
//   action    = pEdge->SetCurve = pEdge->GetCurve->CopyCurve(pEdge->GetContext) 
// ***********************************************************************/
// SmBoolean sm_AssertHealEdge1(SmEdge *pEdge) 
// {  // create and store a Curve copy - delete the old context 
//    if(sm_AssertTestEdge1(pEdge))
//      { SmCurve * pCurveCopy = new (*pEdge->GetContext()) SmCurve(*pEdge->GetCurve()) ; 
//        pEdge->SetCurve(pCurveCopy,  // in : New Curve for Edge
//                        TRUE,        // in : TRUE = delete Old Curve, FALSE=don't, 
//                        TRUE,        // in : TRUE = Signal DBGWarn when saving new curve over old, FALSE=don't, 
//                        FALSE) ;     // in : TRUE = SideEffect: delete all UVTrimCurves, FALSE=don't, 
//      }                             
//    return TRUE ;  
//  } // end sm_AssertHealEdge1
// end obsolete

/*******************************************************************//**
PURPOSE:  AssertTest Rule 0 - An edge must have a Curve 
NOTES:
  predicate = Is pEdge->m_pCurve not equal to NULL
  action    = if(   Edge connected to 2 or more Faces
                 && XSectEdge = XSect(Face1->Surf, Face2->Surf))
                   { pEdge->SetCurve(XSectEdge) ; }
              else { no Fix Possible }
***********************************************************************/
SmBoolean sm_AssertTestEdge0(SmEdge *pEdge) { return( pEdge->GetCurve() != NULL)  ; }

// obsolete
// /*******************************************************************//**
// PURPOSE:  AssertHeal Rule 0 - An edge must have a Curve 
// NOTES:
//   predicate = Is pEdge->m_pCurve not equal to NULL
//   action    = if(   Edge connected to 2 or more Faces
//                  && XSectEdge = XSect(Face1->Surf, Face2->Surf))
//                    { pEdge->SetCurve(XSectEdge) ; }
//               else { no Fix Possible }
// ***********************************************************************/
// SmBoolean sm_AssertHealEdge0(SmEdge *pEdge) 
// { if(sm_AssertTestEdge0(pEdge))
//    { 
//      // locals
//      SmTArray<SmFace *> sFaces ;
//      pEdge->GetFaces(sFaces) ;
// 
//      // when 2 faces exist
//      if(sFaces.GetSize() >= 2)
//        {
//          // locals            
//          SmSurface     * pSurface1           = sFaces[0]->GetSurface() ;
//          SmSurface     * pSurface2           = sFaces[1]->GetSurface() ;
//          const SmBoolean bUseSurfaceEdges[2] = { TRUE, TRUE } ; 
//          SmApproxTol3d   sApproxTol3d        = smos_Min(SmTol::GetApproxTol3d(sFaces[0]), SmTol::GetApproxTol3d(sFaces[1])) ;
// 
//          // when two surfaces exist
//          if(pSurface1 && pSurface2)
//            {
//              // locals
//              SmTArray<SmCurve*>         s3DCurves ;         
//              SmTArray<SmCurve*>         sSurface1UVCurves ;  
//              SmTArray<SmCurve*>         sSurface2UVCurves ;  
//              SmTArray<SmTsectCurveType> sCurveTypes ;
//              SmTArray<double>           sDeviations ;
// 
//              // get XSectCurve = XSect(Surf,Surf) ;
//              pSurface1->GlobalSurfaceIntersect
//                (*pEdge->GetContext(),                        // in : Context for new object construction
//                 pSurface1->GetNaturalUVDomain(),             // in : this Surface intersection limits
//                 *pSurface2,                                  // in : target 2nd intersecting surface
//                 pSurface2->GetNaturalUVDomain(),             // in : 2nd Surface intersection limits
//                 bUseSurfaceEdges,                            // in : TRUE = Find intersection curve start points by intersecting
//                                                              //             the edges of one surface with the other.
//                                                              //      Normally both are TRUE unless you know
//                                                              //      that the edges of one surface do not intersect
//                                                              //      the other surface.  It is a slight optimization
//                                                              //      to set the flag to FALSE
//                 &sApproxTol3d,                               // in : If not given it uses 1/1000 of surface size
//                                                              //      approximation tolerance
//                 NULL,                                        // in : If not given it uses 30 degrees
//                 &s3DCurves,                                  // out: 3D curves produced by intersection
//                 &sSurface1UVCurves,                          // out: UV curves on this surface produced by intersection
//                 &sSurface2UVCurves,                          // out: UV curves on crOtherSurface produced by intersection
//                 &sCurveTypes,                                // out: What type of curve is produced -
//                                                              //      oneof: SM_TC_TOUCHING   - single point intersection (surf norms parallel)
//                                                              //             SM_TC_CROSSING   - curve intersection (surf norms not parallel)
//                                                              //             SM_TC_TANGENT    - curve intersection (surf norms parallel)
//                                                              //             SM_TC_COINCIDENT - curve intersection (surf norms parallel & cross-tangents equal)
//                                                              //             SM_TC_NEAR_TANGENT    - curve has small angle of intersection
//                                                              //             SM_TC_REGION_BOUNDARY - curve bounds region within which the surfs are coincident
//                 &sDeviations) ;                              // out: produced Curve Deviations from each UVCurve to true xSect, NULL to ignore
//                   
//               // when intersections exist
//               if(s3DCurves.GetSize() > 0)
//                 {
//                   // the following is too simple - when given more than one intersection curve
//                   // SmCurve * pCurve = s3DCurves[0] ;
// 
//                   // Drop
// // begin obsolete
// // #ifdef SM_DEBUG_CODE
// //                   ULONG GWC_NOT__DONE_IMPLEMENTING_sm_AssertHealEdge0_YET GWC_LINE ; 
// // #endif // SM_DEBUG_CODE
// // end obsolete
// 
//                 }
//             } // end two surfaces existence check
//         } // end two faces existence check
//     } // end sm_AssertTestEdge0 still fails check
// 
//   // all done    
//   return FALSE ; 
// 
// } // end sm_AssertHealEdge0
// end obsolete

/*******************************************************************//**
PURPOSE:
     
RETURNS ---  TRUE  = OK
             FALSE = Problem
***********************************************************************/
SmBoolean SmEdge::AssertValid
 (SmAssertArray    * pAList,        // i/o: Accumulating list of failed Asserts, NULL to ignore, default:[NULL] 
  SmAssertTestLevel  eTestLevel,    // in : SM_LEVEL_0=fewest tests, SM_LEVEL_1, SM_LEVEL_2=all tests, 
                                    //      SM_LEVEL_GIVEN = run tests in order requested in pTestRequests 
                                    //      default:[SM_LEVEL_0] 
  SmAssertWalking    eWalkTree,     // in : SM_WALK = run AssertValid on any topology graph descendants, SM_NO_WALK=don't, default:[SM_WALK] 
  SmTArray<ULONG>  * pTestRequests) // in : when eTestLevel == SM_LEVEL_GIVEN, run these tests in this order, default:[NULL]
 const
{
   SM_REF1(pTestRequests);

  // init rtn value
  SmBoolean bRtn  = TRUE ;
  
  // call the base class AssertValid
  bRtn &= (  (eTestLevel != SM_LEVEL_GIVEN)
           ? SmOwningTopology::AssertValid(pAList, eTestLevel, SM_NO_WALK) 
           : TRUE ) ; 

  /*  0 */ // Edge has a curve
  bRtn &= SM_ASSERT_BOOLEAN_REPORT(0, SM_LEVEL_0, (m_pCurve != NULL ), _T("") ) ;

  // SmEdge and the objects it attaches to need to share common contexts
  if(m_pCurve) 
    {
      
      /*  1 */ // Edge and Curve both have the same context  
      bRtn &= SM_ASSERT_BOOLEAN_REPORT(1, SM_LEVEL_0, (GetContext() == m_pCurve->GetContext() ), _T("") ) ;

      /*  2 */ // Edge's interval is contained within the curve's Natural interval
      bRtn &= SM_ASSERT_BOOLEAN_REPORT(2, SM_LEVEL_0, (m_vInterval.IsContainedBy(m_pCurve->GetNaturalInterval(), SM_EFF_ZERO) ), _T("") ) ;
    }

  // Check Edge->Curve
  if(m_pCurve) { bRtn &= m_pCurve->AssertValid(pAList, eTestLevel, SM_NO_WALK) ; }

  // Check all connected Edgeuses
  SmTArray<SmEdgeuse *> sEdgeuses ;
  GetEdgeuses(sEdgeuses) ;
  ULONG ii, lNumEUs = sEdgeuses.GetSize();;

// Remove Composites  - moved check up from below
  /*  3 */ // Edges must have uses
      bRtn &= SM_ASSERT_BOOLEAN_REPORT(3, SM_LEVEL_0, ( lNumEUs > 0 ), _T("") ) ; 

// Remove Composites
//  // Check that there is an Edgeuse: error if not.
//  if ( this->GetType() != SmCEdge_TYPE )
//    { 
//      /*  3 */ // When Edge is not a CEdge it must have uses
//      bRtn &= SM_ASSERT_BOOLEAN_REPORT(3, SM_LEVEL_0, ( lNumEUs > 0 ), _T("") ) ; 
//    }

  // check every Edge->Edgeuse
  for ( ii=0; ii<lNumEUs; ii++)
    {
      if(sEdgeuses[ii]) { bRtn &= sEdgeuses[ii]->AssertValid(pAList, eTestLevel, SM_NO_WALK) ; }
    }

  // size locals
  // double  dBrepTol     = GetBrep() ? static_cast<double>(GetBrep()->GetTolerance()) : 0.0 ;
  double  dEdgeTol     = GetTolerance() ;
  double  dCurveLength = m_pCurve->ApproximateLength(m_vInterval, 5) ; 

  /* obsolete */ /*  5 */ // Edge Tolerance less than Brep Tol
  // bRtn &= SM_ASSERT_VALUE_REPORT(5, SM_LEVEL_0, dEdgeTol >= dBrepTol - SM_EFF_ZERO, SM_EFF_ZERO, dEdgeTol - dBrepTol, _T("")) ;

  /* obsolete */ /*  6 */ // Edge Tolerance greater than 1000 times Brep Tol
  // bRtn &= SM_ASSERT_VALUE_REPORT(6, SM_LEVEL_0, dEdgeTol <= 1000 * dBrepTol, 1000 * dBrepTol, dEdgeTol, _T("")) ;

  /*  7 */ // Edge Length must be larger than its tolerance
  bRtn &= SM_ASSERT_VALUE_REPORT(7, SM_LEVEL_0, dCurveLength >= dEdgeTol - SM_EFF_ZERO, dEdgeTol, dCurveLength - dEdgeTol, _T("")) ;

  /* 12 */ // Edge's stored ZoneTol must equal the SmTol::Consistent Tolerance Model value
#ifdef SM_USE_OLDTOL
#ifdef SM_NMTLIB_7166
  SM_OLDTOL_LINE bRtn &= SM_ASSERT_BOOLEAN_REPORT(12, SM_LEVEL_2, (sm_AssertTestEdge12(this) == TRUE), _T("")) ;
#endif // SM_NMTLIB_7166
#endif // SM_USE_OLDTOL

  /* 13 */ // Edge->Curve must be within SmTol::GetXSectTol3d(Edge, GenSurface) of all Face->Surfaces
  double       dMaxGap     = 0.0 ; 
  SmXSectTol3d sXSectTol3d = SmTol::GetXSectTol3d(this, this) ;
  bRtn &= SM_ASSERT_VALUE_REPORT(13, SM_LEVEL_2, (sm_AssertTestEdge13(this, &dMaxGap) == TRUE), sXSectTol3d, dMaxGap, _T("")) ;

  /* 14 */ // Edge->Edgeuse orientations must alternate, SAME-OPPO-SAME-OPPO
  bRtn &= SM_ASSERT_BOOLEAN_REPORT( 14, SM_LEVEL_0, sm_AssertTestEdge14( this ), _T("") );

  // Vertex cardinality and endpoint gaps (moved from SmFace 8-10 so a manifold
  // edge is checked once, not once per incident face).
  {
    SM_PTR_ARRAY(sVerts, SmVertex, 2) ;
    GetVertices( sVerts );
    ULONG lNumVerts = sVerts.GetSize();

    /* 15 */ // closed edges can only have one vert, open edges 2
    bRtn &= SM_ASSERT_BOOLEAN_REPORT(15, SM_LEVEL_0, ( lNumVerts == 1 || lNumVerts == 2 ), _T("") ) ;

    if ( ( lNumVerts == 1 || lNumVerts == 2 ) && m_pCurve != NULL )
      {
        SmVertex *pVtx = GetStartVertex();
        if ( pVtx != NULL )
          {
            SmPoint3d sVtxPt = pVtx->GetPoint();
            SmPoint3d sEdgePt ;
            m_pCurve->EvaluatePoint( m_vInterval.GetMin(), sEdgePt );
            double dDist = sVtxPt.DistanceBetween( sEdgePt );
            double dVtxTol = pVtx->GetTolerance();
            /* 16 */ // start edge/vertex gap must be less than edge + vertex tolerance
            bRtn &= SM_ASSERT_BOOLEAN_REPORT(16, SM_LEVEL_0, (dDist <= dEdgeTol+dVtxTol), _T("")) ;

            pVtx = GetOtherVertex( pVtx );
            if ( pVtx != NULL )
              {
                sVtxPt = pVtx->GetPoint();
                m_pCurve->EvaluatePoint( m_vInterval.GetMax(), sEdgePt );
                dDist = sVtxPt.DistanceBetween( sEdgePt );
                dVtxTol = pVtx->GetTolerance();
                /* 17 */ // end edge/vertex gap must be less than edge + vertex tolerance
                bRtn &= SM_ASSERT_BOOLEAN_REPORT(17, SM_LEVEL_0, (dDist <= dEdgeTol+dVtxTol), _T("") ) ;
              }
          }
      }
  }

  // test Manifold and Spine edges
  if(IsManifold() || IsSpine())
    {
      // Check consistency of contained Edgeuses
      const SmEdgeuse *pTargetEdgeuse = GetPrimaryEdgeuse() ;
      SmEdgeuse       *pRadialEdgeuse = pTargetEdgeuse->GetRadial() ;

      // iter locals
      SM_OBJ_ARRAY(sAngsFromPrimaryDeg, double, 16) ;
      SM_PTR_ARRAY(sVertices, SmVertex, 2) ; 
      ULONG             lIndx ;
      SmExtent1d        sExtent = GetInterval() ;
      const SmEdgeuse * pPrimaryEdgeuse = NULL ;
      SmPoint3d         sTargetPnt, sTargetBiVec, sTargetFaceNormal, sTargetTangent ;
      SmPoint3d         sRadialPnt, sRadialBiVec, sRadialFaceNormal, sRadialTangent ;
      SmPoint3d         sPrimaryBiVec, sPrimaryTangent ;
      SmExtent1d        sCircIvl(0.0, 360.0) ;
      GetVertices(sVertices) ;

      // Must watch out for zero sectors: coincident faces.
      SmBoolean bHaveNonzeroAngle = FALSE;

      // for every edgeuse-radial partner set, get angles from this edgeuse biVec in ascending order
      while(pTargetEdgeuse != pPrimaryEdgeuse)
        {
          /*  8 */ // Edgeuses of Manifold and Spine edges must be of Type SmLoopuse_TYPE
          bRtn &= SM_ASSERT_BOOLEAN_REPORT(8, SM_LEVEL_0, pTargetEdgeuse->m_tEdgeuseType == SmLoopuse_TYPE, _T("")) ;
          bRtn &= SM_ASSERT_BOOLEAN_REPORT(8, SM_LEVEL_0, pRadialEdgeuse->m_tEdgeuseType == SmLoopuse_TYPE, _T("")) ;

          // for SmLoopuse_TYPE edgeuses make sure of radial ordering
          if(   pTargetEdgeuse->m_tEdgeuseType == SmLoopuse_TYPE
             && pRadialEdgeuse->m_tEdgeuseType == SmLoopuse_TYPE)
            {
              // get BinormalEvaluations
              pTargetEdgeuse->EvaluateBinormal(sExtent.Evaluate(.456), TRUE, sTargetPnt, sTargetBiVec, &sTargetTangent, &sTargetFaceNormal) ;
              pRadialEdgeuse->EvaluateBinormal(sExtent.Evaluate(.456), TRUE, sRadialPnt, sRadialBiVec, &sRadialTangent, &sRadialFaceNormal) ;

              // 1st iteration save variables
              if(pPrimaryEdgeuse == NULL) { pPrimaryEdgeuse = pTargetEdgeuse ;
                                            sPrimaryBiVec   = sTargetBiVec ;
                                            sPrimaryTangent = sTargetTangent ;
                                          }

              // Primary BiVec to current Radial BiVec Angle
              double dAngFromPrimaryRad = 0.0;
              sPrimaryTangent.CCWAngleBetween(sPrimaryBiVec, sRadialBiVec, dAngFromPrimaryRad) ;
              double dAngFromPrimaryDeg = SM_RAD2DEG(dAngFromPrimaryRad) ;

              if ( dAngFromPrimaryDeg < -SM_EFF_ZERO )                             { dAngFromPrimaryDeg += 360.0; }
              if ( bHaveNonzeroAngle && dAngFromPrimaryDeg <=  0.0 + SM_EFF_ZERO ) { dAngFromPrimaryDeg += 360.0 ; }
              if (!bHaveNonzeroAngle && dAngFromPrimaryDeg > 360.0 + SM_EFF_ZERO ) { dAngFromPrimaryDeg -= 360.0;  
                                                                                     bHaveNonzeroAngle = ( smos_Fabs( dAngFromPrimaryDeg ) > SM_EFF_ZERO );
                                                                                   }

              // After the first sector
              if ( sAngsFromPrimaryDeg.GetSize() > 0 )
                {
                  // after the first sector map 0.0 degrees to 360.0 degrees assuming the next 0.0 degrees is the mate side to the primary edgeuse
                  if(   SM_IS_ZERO(dAngFromPrimaryDeg)
                     && pPrimaryEdgeuse == pRadialEdgeuse->GetMate() ) 
                    { dAngFromPrimaryDeg = 360.0; }

                  /*  9 */ // check for increasing angles between ordered RadialEUs and PrimaryEdgeuse
                  bRtn &= SM_ASSERT_VALUE_REPORT( 9, SM_LEVEL_0, (dAngFromPrimaryDeg > (sAngsFromPrimaryDeg.GetLast() - SM_EFF_ZERO)), SM_EFF_ZERO, dAngFromPrimaryDeg - sAngsFromPrimaryDeg.GetLast(), _T("") ) ;

                  /* 10 */ // check angle between every RadialEU and the PrimaryEdgeuse is within [0 360]
                  bRtn &= SM_ASSERT_VALUE_REPORT(10, SM_LEVEL_0, (sCircIvl.ContainsValue( dAngFromPrimaryDeg, SM_EFF_ZERO )), SM_EFF_ZERO, SM_UNDEF_DOUBLE, _T("") ) ;

                  /* 11 */ // check for Bad Vertex Ptrs, Not all Edge->Edgeuse->Vertexuse->Vertices map to the same two vertices
                  bRtn &= SM_ASSERT_BOOLEAN_REPORT(11, SM_LEVEL_0, (TRUE == sVertices.FindElement(  pTargetEdgeuse->GetVertexuse()
                                                                                                  ? pTargetEdgeuse->GetVertexuse()->GetVertex()
                                                                                                  : NULL,
                                                                                                  lIndx) ), 
                                                                   _T("")) ;
                  bRtn &= SM_ASSERT_BOOLEAN_REPORT(11, SM_LEVEL_0, (TRUE == sVertices.FindElement(  pRadialEdgeuse->GetVertexuse()
                                                                                                  ? pRadialEdgeuse->GetVertexuse()->GetVertex()
                                                                                                  : NULL,
                                                                                                  lIndx) ), 
                                                                   _T("")) ;
                }
                                          
              sAngsFromPrimaryDeg.Add(dAngFromPrimaryDeg) ;

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe1 = FALSE ;
              // draw 
              if(bDebugMe1)
                {
                  this->Dump() ; 

                  SmBrep  *pBrep       = this->GetBrep();
                  SmFace  *pTargetFace = pTargetEdgeuse ? pTargetEdgeuse->GetFace() : NULL ;
                  SmFace  *pRadialFace = pRadialEdgeuse ? pRadialEdgeuse->GetFace() : NULL ;

                  sAngsFromPrimaryDeg.Dump() ;

                  smgfx_Erase() ;
                  smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
                  smgfx_SetLook(3,4, 0,0,0) ; this->Draw() ; sm_GraphicsLoop() ;
                  smgfx_SetLook(1,2, 0,1,0) ; if(pTargetFace) pTargetFace->Draw(SM_DM_CROSSHATCH,5,5) ; sm_GraphicsLoop() ;
                  smgfx_SetLook(1,2, 0,1,1) ; if(pRadialFace) pRadialFace->Draw(SM_DM_CROSSHATCH,5,5) ; sm_GraphicsLoop() ;

                  smgfx_SetLook(3,4, 0,1,0) ; if(pTargetEdgeuse) pTargetEdgeuse->Draw() ; sm_GraphicsLoop() ;
                  smgfx_SetLook(3,4, 0,1,1) ; if(pRadialEdgeuse) pRadialEdgeuse->Draw() ; sm_GraphicsLoop() ;

                  smgfx_SetLook(5,6, 1,0,0) ; sTargetPnt.Draw() ; sm_GraphicsLoop() ;
                  smgfx_SetLook(2,3, 1,0,0) ; sTargetFaceNormal.Draw(&sTargetPnt) ; sm_GraphicsLoop() ;
                  smgfx_SetLook(7,8, 0,0,1) ; sRadialPnt.Draw() ; sm_GraphicsLoop() ;
                  smgfx_SetLook(2,3, 0,0,1) ; sRadialFaceNormal.Draw(&sRadialPnt) ; sm_GraphicsLoop() ;

                  sm_GraphicsLoop() ;
                }
#endif // SM_DEBUG_CODE

              // get ready for next iteration
              pTargetEdgeuse = pRadialEdgeuse->GetMate() ;
              pRadialEdgeuse = pTargetEdgeuse->GetRadial() ;
            
            } // end iter every edgeuse/radial mate 
        } // end Edge is manifold or spine check
       
    } // end SmLoopuse_TYPE radial edge ordering of SmFaceuses check

  // when asked - AssertValid for a topology graph traversal
  if(eWalkTree == SM_WALK)
    {
      /*  4 */ // Edge nested topology objects must also be valid
      bRtn &= SmBrep::AssertSubTopology( (SmTopology &)*this, pAList, eTestLevel) ;
      // bRtn &= SM_ASSERT_BOOLEAN_REPORT(4, SM_LEVEL_0, (SmBrep::AssertSubTopology( (SmTopology &)*this, pAList, eTestLevel)), _T("")) ;

    } // end asked to walk the tree check

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  if(bDebugMe)
    {
      this->Dump(FALSE) ;
    }
#endif // SM_DEBUG_CODE
  // all done
  // SM_ASSERT(bRtn) ;
  return(bRtn) ;

} // end SmEdge::AssertValid

// obsolete
// /*******************************************************************//**
// PURPOSE: Fix Assert Report failures reported by an AssertValid call
// 
// RETURNS ---  TRUE  = OK
//              FALSE = Problem
// ***********************************************************************/
// SmBoolean SmEdge::AssertHeal
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
//       case 12 : // Edge's stored ZoneTol must equal the SmTol::Consistent Tolerance Model value
//                 bRtn = sm_AssertHealEdge12(this, rAReport, pAList); break;
// 
//       case 13 : // Edge's stored ZoneTol must equal the SmTol::Consistent Tolerance Model value
//                 bRtn = sm_AssertHealEdge13(this, rAReport, pAList); break;
// 
//       case 99 : { // set case number appropriately - run fix code here
//                   // if fix works set rAReport.m_bOK = TRUE ; 
//                 }
//                 break ;
// 
//       default: rAReport.m_eAssertType  = SM_AT_NO_HEAL_YET ;  
//                rAReport.m_pHealMessage = _T("SmEdge::AssertHeal fix not yet supported") ;  
//     }
// 
//   // all done
//   return(bRtn) ;
// 
// } // end SmEdge::AssertHeal
// end obsolete

/*************************************************************
PURPOSE:  Get this edge number within brep

NOTES:
       
**************************************************************/
ULONG SmEdge::GetEdgeNumberInBrep() const
{
  ULONG lEdgeNum = 0;

  SmBrep *pBrep = GetBrep();
  if(pBrep != NULL)
  {
    SmTArray<SmEdge*> sAllEdges;
    pBrep->GetEdges( sAllEdges );
    for(ULONG i = 0; i < sAllEdges.GetSize(); i++)
    {
      if(sAllEdges[i] == this)
      {
        lEdgeNum = i;
        break;
      }
    }

  }

  return(lEdgeNum);

} // end SmEdge::GetEdgeNumberInBrep

/*******************************************************************//**
PURPOSE: Get an edgeuse with the SAME orientation from an edge.

NOTES:
***********************************************************************/
SmEdgeuse* SmEdge::GetSameOrientedEdgeuse() const
{
  return(  (GetPrimaryEdgeuse()->GetOrientation() == SM_OT_SAME)
         ? GetPrimaryEdgeuse()
         : GetPrimaryEdgeuse()->GetMate()) ;

} // end SmEdge::GetSameOrientedEdgeuse

/*******************************************************************//**
PURPOSE:
NOTES:
***********************************************************************/
SmBoolean SmEdge::IsKindOf( SM_TYPE t ) const
{
  return ((SmEdge_TYPE == t) ? TRUE : SmOwningTopology::IsKindOf( (t) ));
}

/*******************************************************************//**
PURPOSE:   Dump preceded by a one line message

NOTES:
***********************************************************************/
void SmEdge::Dump
  (TCHAR * message) 
 const
{
     TCHAR sBuff[SM_TBLOCK_SIZE];
     smos_sprintf(sBuff,_T("\n%s "), message);
     smos_WriteBuffer(sBuff);
     this->Dump();

} // end SmEdge::Dump

/*******************************************************************//**
PURPOSE:  Dump preceded by an integer (ULONG)

NOTES:
***********************************************************************/
void SmEdge::Dump
  (ULONG i) 
 const
{
     TCHAR sBuff[SM_TBLOCK_SIZE];
     smos_sprintf(sBuff,_T("\n%ld "),i);
     smos_WriteBuffer(sBuff);
     this->Dump(FALSE);

} // end SmEdge::Dump

/*************************************************************
PURPOSE:

NOTES:
**************************************************************/
void SmEdge::Dump
  (void) 
 const
{
  this->Dump(FALSE); // FALSE = do NonAbbrev dump
}

/*************************************************************
PURPOSE:

NOTES:
       bAbbrev = TRUE - abbreviated- just print edge number
**************************************************************/
void SmEdge::Dump
  (SmBoolean bAbbrev) 
 const
{
  TCHAR sBuff[16*SM_TBLOCK_SIZE], sBuffForFile[16*SM_TBLOCK_SIZE], sTolLabel[SM_TBLOCK_SIZE], sSmallTopoLabel[SM_TBLOCK_SIZE] = {}, sBrepTolLabel[SM_TBLOCK_SIZE] = {}, sBrepTolVal[SM_TBLOCK_SIZE] = {};
  ULONG i;

  smos_WriteBuffer(_T("\nBegin SmEdge::Dump()"));

  // removed sBuffForFile: it was causing nothing but garbage
  // to be written to the file.

  // write edge number = EdgePointerValue
#ifdef SM_USE_NEWTOL
  smos_sprintf(sTolLabel,       _T("Max Edge/Face Gap:")) ;
  smos_sprintf(sSmallTopoLabel, _T("SmallTopology:")) ;
  //smos_sprintf(sBrepTolLabel,   _T("")) ;
  //smos_sprintf(sBrepTolVal, _T("") ;

  SmBoolean bIsSmall  = IsSmallTopology() ;
  double    dGapOrTol = GetMaxGap3d() ;
#else // SM_USE_OLDTOL
  smos_sprintf(sTolLabel,       _T("%.1024ss"), _T("ZoneTol3d:")) ;
  //smos_sprintf(sSmallTopoLabel, _T("")) ;
  smos_sprintf(sBrepTolLabel,   _T("%.1024ss"), _T("BrepTol:")) ;
  smos_sprintf(sBrepTolVal, _T("%5.7lf"), (double)GetBrep()->GetTolerance()) ;

  SmBoolean bIsSmall  = UNSURE ;    // UNSURE = don't show a small topology report
  double    dGapOrTol = (double)GetTolerance() ;

#endif // SM_USE_OLDTOL
  ULONG lEdgeNum = this->GetEdgeNumberInBrep();

  // header
  smos_sprintf(sBuff,        _T("\n  Brep->Edge[%lu]:[0x%p] Ivl:[%6.16lf %6.16lf] %.128s[%5.7lf] %.128s[%.128s] EdgeuseCnt&1st[%lu 0x%p] %.128s%.128s "), 
             lEdgeNum, this, 
             m_vInterval.GetMin(), m_vInterval.GetMax(), 
             sTolLabel, dGapOrTol,
             sBrepTolLabel, sBrepTolVal,
             m_lListSize, m_pList, 
             sSmallTopoLabel, bIsSmall == TRUE ? _T("TRUE") : bIsSmall == FALSE ? _T("FALSE") : _T("")) ;
  smos_sprintf(sBuffForFile, _T("\n  Brep->Edge[%lu]:[%.128s] Ivl:[%6.16lf %6.16lf] %.128s[%5.7lf] %.128s[%.128s] EdgeuseCnt&1st[%lu %.128s] %.128s%.128s "), 
             lEdgeNum, _T("NotNULL"),
             m_vInterval.GetMin(), m_vInterval.GetMax(),  
             sTolLabel, dGapOrTol,
             sBrepTolLabel, sBrepTolVal,
             m_lListSize, m_pList ? _T("NotNULL") : _T("NULL"), 
             sSmallTopoLabel, bIsSmall == TRUE ? _T("TRUE") : bIsSmall == FALSE ? _T("FALSE") : _T("")) ;
  smos_WriteBuffer(sBuff, sBuffForFile);

  // type and props
// Remove Composites
//  smos_sprintf( sBuff, _T("\n    Properties :[%s, %s, %s, %s, %s, %s] "),  IsWire()     ? _T("Wire Edge")
  smos_sprintf( sBuff, _T("\n    Properties :[%s, %s, %s, %s, %s] "),  IsWire()     ? _T("Wire Edge")
                                                                     : IsLamina()   ? _T("Lamina Edge")
                                                                     : IsManifold() ? _T("Manifold Edge")
                                                                     : IsSpine()    ? _T("Spine Edge")
                                                                     :                _T("Unknown Edge"), 
// Remove Composites
//             IsCompositeEdge()  ? _T("Composite") : _T("NotComposite"),
             IsClosed()         ? _T("Closed")    : _T("Open"),
             IsSeam()           ? _T("Seam")      : _T("NotSeam"),
             IsTangentEdge(1.0) ? _T("Tangent")   : _T("NotTangent"),
             IsStrut()          ? _T("Strut (EndVert(s) connect only to ThisEdge)") : _T("NotStrut")) ;
  smos_WriteBuffer(sBuff);

  // end abbreviated report
  if (bAbbrev) 
    {
      smos_WriteBuffer(_T("\nEnd SmEdge::Dump()\n"));
      return;  // abbreviated just returns edge number
    }
  
  // Edge->StartVertex, EndVertex
  SmTArray<SmVertex*> sVertices;
  GetVertices(sVertices);
  ULONG uCount = sVertices.GetSize();

  smos_sprintf( sBuff, _T("\n    %2lu Vertices:"), uCount );
  smos_WriteBuffer(sBuff);

  // for all verts
  for(i=0;i<uCount;i++)
    {
      ULONG lVtxNum = sVertices[i]->GetVertexNumberInBrep();
      smos_sprintf( sBuff, _T("%s Brep->Vtx[%3lu]:[0x%p]\n"), i==0 ? _T("") : _T("                "), lVtxNum, sVertices[i] );
      smos_sprintf( sBuffForFile, _T("%s Brep->Vtx[%3lu]:[%s]\n"), i==0 ? _T("") : _T("                "), lVtxNum, sVertices[i] ? _T("NotNULL") : _T("NULL"));
      smos_WriteBuffer( sBuff, sBuffForFile);
    }

  // Edge->Faces
  SmTArray<SmFace*> sFaces;
  GetFaces( sFaces );
  uCount = sFaces.GetSize();

  smos_sprintf( sBuff, _T("    %2lu Faces   :"), uCount );
  smos_WriteBuffer(sBuff);

  for(i=0;i<uCount;i++)
    {
      ULONG lFaceNum = sFaces[i]->GetFaceNumberInBrep();
      smos_sprintf( sBuff,        _T( "%s Brep->Face[%3lu]:[0x%p]\n"), i==0 ? _T("") : _T("               "), lFaceNum, sFaces[i]) ;
      smos_sprintf( sBuffForFile, _T( "%s Brep->Face[%3lu]:[%s]\n"),   i==0 ? _T("") : _T("               "), lFaceNum, sFaces[i] ? _T("NotNULL") : _T("NULL"));
      smos_WriteBuffer( sBuff, sBuffForFile);
    }

  // Edge->Edgeuses
  SmTArray<SmEdgeuse*> sEdgeuses;
  GetEdgeuses( sEdgeuses );
  uCount = sEdgeuses.GetSize();

  smos_sprintf( sBuff, _T("    %2lu Edgeuses: "), uCount );
  smos_WriteBuffer(sBuff) ;

  for(i=0;i<uCount;i++)
    { 
      SmEdgeuse    * pEdgeuse     = sEdgeuses[i] ;
      SmEdge       * pEdge        = pEdgeuse->GetEdge() ;
      SmOrientType   eOrientation = pEdgeuse->GetOrientation() ;
      SmFaceuse    * pFaceuse     = pEdgeuse->GetFaceuse() ;
      SmFace       * pFace        = pEdgeuse->GetFace() ;
      SmShell      * pShell       = pEdgeuse->GetShell() ;
      smos_sprintf( sBuff, _T("%sEU[%2lu]:[0x%p] Orient:[%s] EU->Edge:[0x%p]"),
                                i==0 ? _T("") : _T("                 "), 
                                i, sEdgeuses[i], 
                                eOrientation == SM_OT_SAME ? _T("SAME    ") : _T("OPPOSITE"),
                                pEdge) ;
      smos_sprintf( sBuffForFile, _T("%sEU[%2lu]:[%s] Orient:[%s] EU->Edge:[%s]"),
                                i==0 ? _T("") : _T("                 "), 
                                i, sEdgeuses[i] ? _T("NotNULL") : _T("NULL"), 
                                eOrientation == SM_OT_SAME ? _T("SAME    ") : _T("OPPOSITE"),
                                pEdge ? _T("NotNULL") : _T("NULL")) ;
      smos_WriteBuffer( sBuff, sBuffForFile);

      if(pFaceuse) { smos_sprintf(sBuff, _T(" EU->Faceuse:[0x%p]->Face:[0x%p%s]->Shell[0x%p%s]\n"),
                                pFaceuse, 
                                pFace, ((i/2)%2==0) ? _T("^") : _T("*"), 
                                pShell, (((i+1)/2)%2==0) ? _T("^") : _T("*")) ;
                     smos_sprintf(sBuffForFile, _T(" EU->Faceuse:[%s]->Face:[%s%s]->Shell[%s%s]\n"),
                                pFaceuse ? _T("NotNULL") : _T("NULL"), 
                                pFace    ? _T("NotNULL") : _T("NULL"), ((i/2)%2==0) ? _T("^") : _T("*"),
                                pShell   ? _T("NotNULL") : _T("NULL"), (((i+1)/2)%2==0) ? _T("^") : _T("*")) ;
                   }
      else         { smos_sprintf(sBuff, _T(" Edgeuse->Shell[0x%p] = WireEdge\n"), pShell) ;
                     smos_sprintf(sBuffForFile, _T(" Edgeuse->Shell[0x%p] = WireEdge\n"), pShell ? _T("NotNULL") : _T("NULL")) ;
                   }
      smos_WriteBuffer( sBuff, sBuffForFile );
    }  // end iter every edgeuse

  // Edge->Curve Dump  
  if ( m_pCurve )
    {
      smos_WriteBuffer(_T("Begin Edge->Curve Dump() "));
      m_pCurve->Dump(TRUE) ;  // print abrreviated data on curve
    }
  else
    {
      smos_sprintf( sBuff, _T("%s"), _T("  Begin Edge->Curve Dump()\n    Edge->Curve:[NULL]\n") );
      smos_WriteBuffer( sBuff );
    }
  smos_WriteBuffer(_T("End SmEdge::Dump()\n"));

  // extra line
  smos_WriteBuffer(_T("\n"));

} // end SmEdge::Dump

/*******************************************************************//**
PURPOSE:  Pretty print pointer values for this Edge showing how
             it connects to its neighbor Loop/Shell and Vertex
             as appropriate in the topology graph.

NOTES: Only good for debugging because pointer values don't
                stay constant from run to run.
***********************************************************************/
void SmEdge::DumpTopology
  (ULONG lWalkDepth) // in : 0 = no walking, 1 = curve and vertices,  ... 99 = walk to bottom
 const
{
  // locals
  ULONG        ii ;
  TCHAR        sBuff[SM_TBLOCK_SIZE], sBuffForFile[SM_TBLOCK_SIZE] = {};
  SmCurve     *pCurve   = m_pCurve ;
// Remove Composites
//  SmEdge      *pOwner   = (SmEdge*)pCurve->GetEdge() ;
  SmEdgeuse   *pEdgeuse = (SmEdgeuse*) m_pList;

  // count the number of loopuses directly connected to this Edge
  SmTArray<SmEdgeuse*>   sEdgeuses;  GetEdgeuses(sEdgeuses) ;
  SmTArray<SmVertexuse*> sVertexuses ;

  SM_ASSERT(sEdgeuses.GetSize() == m_lListSize) ;

  // output Shell/Loop-Edge connections - check back pointers
  smos_sprintf(sBuff,_T("  %s Edge[0x%p] (%s, %s, %s) has %ld edgeuses\n"), 
               IsWire()     ? _T("Wire")
             : IsLamina()   ? _T("Lamina")
             : IsManifold() ? _T("Manifold")
             :                _T("NonManifold"),
             this,
               IsClosed() ? _T("Closed") : _T("NotClosed"),
               IsSeam() ?   _T("Seam")   : _T("NotSeam"),
               IsStrut() ?  _T("Strut")  : _T("NotStrut"),
             m_lListSize) ;
  smos_WriteBuffer(sBuff, sBuffForFile);
  
// Remove Composites - moved print line up from below
  // output curve connection - check back pointers
  smos_sprintf(sBuff,_T("    Edge[0x%p] -> Curve  [0x%p] \n"), this, pCurve) ;
      smos_WriteBuffer(sBuff, sBuffForFile);

// Remove Composites
//  // output curve connection - check back pointers
//  if(pOwner != this)
//    {
//      SM_ASSERT(pOwner->IsKindOf(SmCEdge_TYPE)) ;
//      smos_sprintf(sBuff,_T("    Edge[0x%p] -> CEdge  [0x%p] -> Curve[0x%p]\n"), 
//                 this, pOwner, pCurve) ;
//      smos_WriteBuffer(sBuff, sBuffForFile);
//    }
//  else
//    {
//      smos_sprintf(sBuff,_T("    Edge[0x%p] -> Curve  [0x%p] \n"), 
//                 this, pCurve) ;
//      smos_WriteBuffer(sBuff, sBuffForFile);
//    }

  // output owned topology objects 

  // Upward Loopuses
  if(m_lListSize > 0)
    {
      // output every owned Edgeuse and its Loopuse/Shell and Vertexuse connections
      for(ii=0;ii<m_lListSize;ii++,pEdgeuse=(SmEdgeuse *)pEdgeuse->m_pNext)
        {
          // check for pointer consistency as possible
          SM_ASSERT(pEdgeuse->GetNext()->GetLast() == pEdgeuse) ;
          SM_ASSERT(   pEdgeuse->GetCCWEdgeuse() == NULL
                    || pEdgeuse->GetCCWEdgeuse()->GetCWEdgeuse() == pEdgeuse) ;
          SM_ASSERT(   pEdgeuse->GetCWEdgeuse() == NULL
                    || pEdgeuse->GetCWEdgeuse()->GetCCWEdgeuse() == pEdgeuse) ;
          SM_ASSERT(pEdgeuse->GetVertexuse()->GetEdgeuse()    == pEdgeuse) ;
          SM_ASSERT((SmEdge *)pEdgeuse->GetOwner() == this) ;
          if(pEdgeuse->m_tEdgeuseType == SmShell_TYPE)
            {
              if(pEdgeuse->m_pUVTrimCurve != NULL)
                {
                  smos_sprintf(sBuff,_T("    Wire Edge[0x%p] -> Edgeuse[0x%p] (Orient:[%s] - UVTrimCurve[0x%p]) -> Shell[0x%p] \n"), 
                             this,
                             pEdgeuse,
                               pEdgeuse->m_eOrientation == SM_OT_SAME ?     _T("  SAME  ")
                             : pEdgeuse->m_eOrientation == SM_OT_OPPOSITE ? _T("OPPOSITE")
                             :                                              _T("UNKNOWN"),
                             pEdgeuse->m_pUVTrimCurve, 
                             pEdgeuse->GetShell()) ;
                  smos_WriteBuffer(sBuff, sBuffForFile);
                }
              else
                {
                  smos_sprintf(sBuff,_T("    Wire Edge[0x%p] -> Edgeuse[0x%p] (Orient:[%s] - No UVTrimCurve) -> Shell[0x%p] \n"), 
                             this,
                             pEdgeuse,
                               pEdgeuse->m_eOrientation == SM_OT_SAME ?     _T("  SAME  ")
                             : pEdgeuse->m_eOrientation == SM_OT_OPPOSITE ? _T("OPPOSITE")
                             :                                              _T("UNKNOWN"),
                             pEdgeuse->GetShell()) ;
                  smos_WriteBuffer(sBuff, sBuffForFile);

                }                          
              smos_sprintf(sBuff,_T("                             Edgeuse[0x%p] -> Vertexuse[0x%p] -> Vertex[0x%p]\n"), 
                         pEdgeuse, 
                         pEdgeuse->GetVertexuse(),
                         pEdgeuse->GetVertexuse()->GetVertex()) ;
              smos_WriteBuffer(sBuff, sBuffForFile);
            }
          else
            {
              SM_ASSERT(pEdgeuse->m_tEdgeuseType == SmLoopuse_TYPE) ;
              if(pEdgeuse->m_pUVTrimCurve != NULL)
                {
                  smos_sprintf(sBuff,_T("    Edge[0x%p] -> Edgeuse[0x%p] (Orient:[%s] - UVTrimCurve[0x%p]) -> Loopuse[0x%p] -> Faceuse[0x%p] -> Face[0x%p]\n"), 
                             this,
                             pEdgeuse, 
                               pEdgeuse->m_eOrientation == SM_OT_SAME ?     _T("  SAME  ")
                             : pEdgeuse->m_eOrientation == SM_OT_OPPOSITE ? _T("OPPOSITE")
                             :                                              _T("UNKNOWN"),
                             pEdgeuse->m_pUVTrimCurve,
                             pEdgeuse->GetLoopuse(),
                             pEdgeuse->GetLoopuse()->GetFaceuse(),
                             pEdgeuse->GetLoopuse()->GetFaceuse()->GetFace()) ;
                  smos_WriteBuffer(sBuff, sBuffForFile);
                }
              else
                {
                  smos_sprintf(sBuff,_T("    Edge[0x%p] -> Edgeuse[0x%p] (Orient:[%s] - No UVTrimCurve) -> Loopuse[0x%p] -> Faceuse[0x%p] -> Face[0x%p]\n"), 
                             this,
                             pEdgeuse, 
                               pEdgeuse->m_eOrientation == SM_OT_SAME ?     _T("  SAME  ")
                             : pEdgeuse->m_eOrientation == SM_OT_OPPOSITE ? _T("OPPOSITE")
                             :                                              _T("UNKNOWN"),
                             pEdgeuse->GetLoopuse(),
                             pEdgeuse->GetLoopuse()->GetFaceuse(),
                             pEdgeuse->GetLoopuse()->GetFaceuse()->GetFace()) ;
                  smos_WriteBuffer(sBuff, sBuffForFile);

                }                     
              smos_sprintf(sBuff,_T("                        Edgeuse[0x%p] -> Vertexuse[0x%p] -> Vertex[0x%p]\n"), 
                         pEdgeuse, 
                         pEdgeuse->GetVertexuse(),
                         pEdgeuse->GetVertexuse()->GetVertex()) ;
              smos_WriteBuffer(sBuff, sBuffForFile);
            }

        } // end iter every Loopuse
    } // end m_lListSize > 0 check

  if(lWalkDepth > 0)
    {
      if(pCurve) pCurve->DumpTopology(lWalkDepth == 99 ? lWalkDepth : lWalkDepth - 1) ;

      SmTArray<SmVertex*> sVertices ;
      GetVertices(sVertices) ;
      for(ii=0;ii<sVertices.GetSize();ii++)
        {
          SmVertex *pVertex = sVertices[ii] ;
          if(pVertex) pVertex->DumpTopology(lWalkDepth == 99 ? lWalkDepth : lWalkDepth - 1) ;
        }
    }

} // end SmEdge::DumpTopology
