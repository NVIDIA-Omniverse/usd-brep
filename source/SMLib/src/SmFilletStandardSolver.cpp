// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************//**
* FILE NAME --- SmFilletStandardSolver.cpp
* PURPOSE: Source code file for those standard SmFilletSolver objects.
*   e.g. SmConstantRadiusFS, SmConstantDistanceFS, SmVariableRadiusFS &
*        SmSurfaceSurfaceFS
**********************************************************************/

#include "StdAfx.h"

#ifndef __SMFILLETSTANDARDSOLVER_H__
#include <SmFilletStandardSolver.h>
#endif

#ifndef __SMFILLETEXECUTIVE_H__
#include <SmFilletExecutive.h>
#endif

#ifndef __SMFILLETINTERSECTOR_H__
#include <SmFilletIntersector.h>
#endif

#include <SmBSplineCurve.h>
#include <SmGraphicsExtern.h>
#include <SmGeomUtility.h>
#include <SmSurfOfExtrusion.h>

/*******************************************************************//**
PURPOSE: Constructor for Constant Radius Fillet object that allows
    different offsets for corresponding surfaces.

NOTES:
***********************************************************************/
SmConstantRadiusFS::SmConstantRadiusFS
  (const SmContext & crContext,
   double dThisApproxTol3d,
   double dAngleTolerance,
   double dTangencyTolerance,
   double dOffsetRadiusSurface1,
   double dOffsetRadiusSurface2,
   const SmSurface & crSurface1,
   const SmSurface & crSurface2,
   SmBoolean bOrientationSurface1,
   SmBoolean bOrientationSurface2)
 : SmFilletSolver(crContext,
                  dThisApproxTol3d,
                  dAngleTolerance,
                  dTangencyTolerance)
{
    m_dFilletRadii[0] = dOffsetRadiusSurface1;
    m_dFilletRadii[1] = dOffsetRadiusSurface2;

    m_dOrientations[0] = 1.0;
    if (bOrientationSurface1 == FALSE) m_dOrientations[0] = - 1.0;
    m_dOrientations[1] = 1.0;
    if (bOrientationSurface2 == FALSE) m_dOrientations[1] = - 1.0;

    m_pSurfaces[0] = new (crContext) SmOffsetSurface( m_dFilletRadii[0]*m_dOrientations[0],
                                                      SM_CONST_CAST(SmSurface &,crSurface1));
    m_pSurfaces[1] = new (crContext) SmOffsetSurface( m_dFilletRadii[1]*m_dOrientations[1],
                                                      SM_CONST_CAST(SmSurface &,crSurface2));
    m_pSurfaces[0]->SetAllowExtension(TRUE);
    m_pSurfaces[1]->SetAllowExtension(TRUE);

    FinishConstruction();  // Take care of other things in the base class.

} // end SmConstantRadiusFS::SmConstantRadiusFS constructor

/*******************************************************************//**
PURPOSE: Constructor for Constant Radius Fillet object.

NOTES:
***********************************************************************/
SmConstantRadiusFS::SmConstantRadiusFS
  (const SmContext & crContext,
   double            dThisApproxTol3d,
   double            dAngleTolerance,
   double            dTangencyTolerance,
   double            dFilletRadius,
   SmEdgeuse       * pEdgeuse,
   double          * pdOptSecondRadius)
  : SmFilletSolver(crContext,
                   dThisApproxTol3d,
                   dAngleTolerance,
                   dTangencyTolerance)
{
    if (!pEdgeuse) SE(SM_ERR);
    m_dFilletRadii[0] = dFilletRadius;
    m_dFilletRadii[1] = dFilletRadius;
    if (pdOptSecondRadius) {
        m_dFilletRadii[1] = *pdOptSecondRadius;
    }
    m_pEdgeuses[0] = pEdgeuse;
    m_pEdgeuses[1] = pEdgeuse->GetRadial();

    // Create offset surfaces of appropriate radius in appropriate direction.

    SmPoint3d sPnt, sPnt2;
    SmVector3d sBinVec2, sBinVec, sFaceuseNormal, sFaceuseNormal2;
    SmEdge *pEdge = m_pEdgeuses[0]->GetEdge();
    SE(m_pEdgeuses[0]->EvaluateBinormal(pEdge->GetInterval().Evaluate(0.5),
        FALSE,sPnt,sBinVec,NULL,&sFaceuseNormal));
    SE(m_pEdgeuses[1]->EvaluateBinormal(pEdge->GetInterval().Evaluate(0.5),
        FALSE,sPnt2,sBinVec2,NULL,&sFaceuseNormal2));

    if (m_pEdgeuses[0]->GetLoopuse()->GetFaceuse()->GetOrientation() == SM_OT_OPPOSITE) {
        sFaceuseNormal = -sFaceuseNormal;
    }
    if (m_pEdgeuses[1]->GetLoopuse()->GetFaceuse()->GetOrientation() == SM_OT_OPPOSITE) {
        sFaceuseNormal2 = -sFaceuseNormal2;
    }

    m_dOrientations[0] = 1.0;
    m_dOrientations[1] = 1.0;
    double dDot1 = sBinVec2.Dot(sFaceuseNormal);
    if (smos_Fabs(dDot1) < SM_EFF_ZERO_SQRT)
    {
        // Surfaces are tangent at midpoint: can't fillet this.
        m_pEdgeuses[0] = NULL;
        m_pEdgeuses[1] = NULL;
        SE(SM_ERR);
        return;
    }
    if (dDot1 < 0.0) {
        m_dOrientations[0] = -1.0;
    }

    double dDot2 = sBinVec.Dot(sFaceuseNormal2);
    if (smos_Fabs(dDot2) < SM_EFF_ZERO_SQRT) {
        // Tangent surfaces.
        m_pEdgeuses[0] = NULL;
        m_pEdgeuses[1] = NULL;
        SE(SM_ERR);
        return;
    }
    if (dDot2 < 0.0) {
        m_dOrientations[1] = -1.0;
    }

    SmSurface *pSurface =
        m_pEdgeuses[0]->GetLoopuse()->GetFaceuse()->GetFace()->GetSurface();
    SmSurface *pSurface2 =
        m_pEdgeuses[1]->GetLoopuse()->GetFaceuse()->GetFace()->GetSurface();
    m_pSurfaces[0] = new (crContext) SmOffsetSurface(
        m_dFilletRadii[0]*m_dOrientations[0],*pSurface);
    m_pSurfaces[1] = new (crContext) SmOffsetSurface(
        m_dFilletRadii[1]*m_dOrientations[1],*pSurface2);
    m_pSurfaces[0]->SetAllowExtension( TRUE );
    m_pSurfaces[1]->SetAllowExtension( TRUE );

    FinishConstruction();  // Take care of other things in the base class.

} // end SmConstantRadiusFS::SmConstantRadiusFS constructor


/*******************************************************************//**
PURPOSE: Destructor for Constant Radius Fillet Surface.

NOTES:
***********************************************************************/
SmConstantRadiusFS::~SmConstantRadiusFS
  ()
{
} // end SmConstantRadiusFS::~SmConstantRadiusFS destructor


/*******************************************************************//**
PURPOSE: Set the offset values to correspond to what they should be
     at this intersection point.

NOTES:
***********************************************************************/
SmStatus SmConstantRadiusFS::SetupOffsetValues
  (const SmTsectPnt &,      // in : not used in this function
   double * pOffsetDist)    // out: Optional fillet radius, NULL to ignore
{
  // get offset distance
  if (pOffsetDist) *pOffsetDist = m_dFilletRadii[0];

  // set offset surface offset distance - Orientation is +/- 1
  GetSurface(0)->SetOffsetDistance(m_dFilletRadii[0]*m_dOrientations[0]);
  GetSurface(1)->SetOffsetDistance(m_dFilletRadii[1]*m_dOrientations[1]);

  return SM_SUCCESS;

} // end SmConstantRadiusFS::SetupOffsetValues


/*******************************************************************//**
PURPOSE: Load the current Jacobian matrix in an incremental way
    and update the parameters.

NOTES:
  With: crX = [u1 v1 u2 v2]
         F  = OffsetSurf1.Position(u1, v1)
         G  = OffsetSurf2.Position(u2, v2)

  Sets:

  rF[]                 = done elsewhere
  rF[rlNumEquations+0] = (F-G).x
  rF[rlNumEquations+1] = (F-G).y
  rF[rlNumEquations+2] = (F-G).z

  pOptJacobian[][]                 = done elsewhere
  pOptJacobian[rlNumEquations+0][] = [ duf.X      dvf.X       -dug.X       -dvg.X ]
  pOptJacobian[rlNumEquations+1][] = [ duf.Y      duf.Y       -dug.Y       -dvg.Y ]
  pOptJacobian[rlNumEquations+2][] = [ duf.Z      duf.Z       -dug.Z       -dvg.Z ]

***********************************************************************/
SmStatus SmConstantRadiusFS::LoadJacobian
  (ULONG                  & rlNumEquations,       // i/o: number of equations to skip when setting
                                                  //      outputs rF and pOptJacobian
   ULONG                  & rlNumParameters,      // i/o: Gets incremented in an odd way to
                                                  //      to point to the input UVPnts.
   ULONG                    lRailIndex,           // in : Specifies rail index of first surface.
   SmSurface             *& rpSurface1,           // in : used only to map which crX[] uvPnt to surface1
   ULONG                  & rlSurf1Offset,        // i/o: index of rpSurface1 uvPnt in crX - ignored if rpSurface1 == NULL
   SmSurface             *& rpSurface2,           // in : used only to map which crX[] uvPnt to surface2
   ULONG                  & rlSurf2Offset,        // i/o: index of rpSurface2 uvPnt in crX - ignored if rpSurface2 == NULL
   const SmTArray<double> & crX,                  // in : current solution, [u1 v1 u2 v2 radius]
   SmTArray<double>       & rF,                   // out: sets rF[rlNumEquations] to rF[rlNumEquations+3] values
   SmMatrix               * pOptJacobian,         // out: sets pOptJacobian[rlNumEquations][] to pOptJacobian[rlNumEquations+3][] rows
   SmBoolean              & rbFoundAnswer)        // out: TRUE = all rF values less than scaled m_dConversionTol
{
  // init output
  rbFoundAnswer = FALSE;

  // Always need to set the offset surface offset distances prior to solving
  GetSurface(0)->SetOffsetDistance(m_dFilletRadii[0]*m_dOrientations[0]);
  GetSurface(1)->SetOffsetDistance(m_dFilletRadii[1]*m_dOrientations[1]);

  // get lSurf1Off index into crX array for lSurf1 parameters
  ULONG lSurf1Off = rlNumParameters;
  rlNumParameters = rlNumParameters + 2;
  if (   lRailIndex == 0
      && (   rpSurface1 != NULL
          && ((SmOffsetSurface*)rpSurface1)->GetBaseSurface() == GetSurface(0)->GetBaseSurface()))
    {
      lSurf1Off = rlSurf1Offset;
      rlNumParameters = rlNumParameters - 2;
    }
  else if (lRailIndex == 1 && (rpSurface2 != NULL &&
      ((SmOffsetSurface*)rpSurface2)->GetBaseSurface() ==
      GetSurface(0)->GetBaseSurface()))
    {
      lSurf1Off = rlSurf2Offset;
      rlNumParameters = rlNumParameters - 2;
    }

  // get offsetSurface1 F Point values for given crX values
  SmPoint3d sF;
  SmVector3d sDUF, sDVF;
  SmPoint2d sUV(crX[lSurf1Off],crX[lSurf1Off+1]);
  SER(GetSurface(0)->Evaluate1stDerivatives(sUV,TRUE,TRUE,sF,sDUF,sDVF));

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
  if (bDebugMe) {
      sm_GraphicsLoop();
      smgfx_SetLook(1,2, 0,1,0); GetSurface(0)->DrawAt(sUV,1); sm_GraphicsLoop();
      sm_GraphicsLoop();
  }
#endif

  // get lSurf2Off index into crX array for lSurf2 parameters
  ULONG lSurf2Off = rlNumParameters;
  rlNumParameters = rlNumParameters + 2;
  if (lRailIndex == 1 && (rpSurface1 != NULL &&
      ((SmOffsetSurface*)rpSurface1)->GetBaseSurface() ==
      GetSurface(1)->GetBaseSurface())) {
      lSurf2Off = rlSurf1Offset;
      rlNumParameters = rlNumParameters - 2;
  }
  if (lRailIndex == 0 && (rpSurface2 != NULL &&
      ((SmOffsetSurface*)rpSurface2)->GetBaseSurface() ==
      GetSurface(1)->GetBaseSurface())) {
      lSurf2Off = rlSurf2Offset;
      rlNumParameters = rlNumParameters - 2;
  }

  // set rlSurfffset as needed
  if (lRailIndex == 0 && rpSurface1 == NULL) { rlSurf1Offset = lSurf1Off; }
  if (lRailIndex == 0 && rpSurface2 == NULL) { rlSurf2Offset = lSurf2Off; }
  if (lRailIndex == 1 && rpSurface1 == NULL) { rlSurf2Offset = lSurf1Off; }
  if (lRailIndex == 1 && rpSurface2 == NULL) { rlSurf1Offset = lSurf2Off; }

  // get offsetSurface2 G Point values for given crX values
  SmPoint3d sG;
  SmVector3d sDUG, sDVG;
  SmPoint2d sUV2(crX[lSurf2Off],crX[lSurf2Off+1]);
  SER(GetSurface(1)->Evaluate1stDerivatives(sUV2,TRUE,TRUE,sG,sDUG,sDVG));
#ifdef SM_DEBUG_CODE
  if (bDebugMe) {
      sm_GraphicsLoop();
      smgfx_SetColor(1,0,0);
      GetSurface(1)->DrawAt(sUV2,1);
      sm_GraphicsLoop();
  }
#endif

  // check surface tangent magnitudes
  double sDUFLenSq = sDUF.LengthSquared();
  double sDVFLenSq = sDVF.LengthSquared();

  double sDUGLenSq = sDUG.LengthSquared();
  double sDVGLenSq = sDVG.LengthSquared();

  // failure: zero length tangents
  if (   sDUFLenSq < SM_EFF_ZERO_SQ
      || sDVFLenSq < SM_EFF_ZERO_SQ
      || sDUGLenSq < SM_EFF_ZERO_SQ
      || sDVGLenSq < SM_EFF_ZERO_SQ)
    { return SM_ERR; }

  // set sDiff1 = distance between offset points
  SmVector3d sDiff1 = sF - sG;

#ifdef SM_DEBUG_CODE
  if (bDebugMe) {
      sm_GraphicsLoop();
      smgfx_SetLook(1,2, 1,0,0); sDiff1.Draw(&sG); sm_GraphicsLoop();
      sm_GraphicsLoop();
  }
#endif

  // Intersection tries to minimize the following six equations.
  // It is essentially intersecting the three surfaces.
  // (Three surfaces: each Fillet has two, one in common.)
  // fun[i+0] = (f-g) . X     - X component of vector f-g
  // fun[i+1] = (f-g) . Y     - Y component of vector f-g
  // fun[i+2] = (f-g) . Z     - Z component of vector f-g
  // Next 3 similar for (f-h).
  //
  // Where f is surface1, g is surface2 and h is surface3
  //
  // It produces the following Jacobian
  //  | duf.X      dvf.X       -dug.X       -dvg.X |
  //  | duf.Y      duf.Y       -dug.Y       -dvg.Y |
  //  | duf.Z      duf.Z       -dug.Z       -dvg.Z |

  if (pOptJacobian)
    {
      ULONG i = rlNumEquations;
      (*pOptJacobian)[i][lSurf1Off]   =   sDUF.x;
      (*pOptJacobian)[i][lSurf1Off+1] =   sDVF.x;
      (*pOptJacobian)[i][lSurf2Off]   = - sDUG.x;
      (*pOptJacobian)[i][lSurf2Off+1] = - sDVG.x;

      i++; // Move to next equation;
      (*pOptJacobian)[i][lSurf1Off]   =   sDUF.y;
      (*pOptJacobian)[i][lSurf1Off+1] =   sDVF.y;
      (*pOptJacobian)[i][lSurf2Off]   = - sDUG.y;
      (*pOptJacobian)[i][lSurf2Off+1] = - sDVG.y;

      i++;
      (*pOptJacobian)[i][lSurf1Off]   =   sDUF.z;
      (*pOptJacobian)[i][lSurf1Off+1] =   sDVF.z;
      (*pOptJacobian)[i][lSurf2Off]   = - sDUG.z;
      (*pOptJacobian)[i][lSurf2Off+1] = - sDVG.z;
    }

  // Compute function values
  rF[rlNumEquations++] = sDiff1.x;
  rF[rlNumEquations++] = sDiff1.y;
  rF[rlNumEquations++] = sDiff1.z;

  // See if we have converged
  double dScaledTol = GetConversionTol() * (1.0 + sF.GetMaxDimension());
  if (   smos_Fabs(sDiff1.x) < dScaledTol
      && smos_Fabs(sDiff1.y) < dScaledTol
      && smos_Fabs(sDiff1.z) < dScaledTol)
    {
      rbFoundAnswer = TRUE;
    }

#ifdef SM_DEBUG_CODE
  if (bDebugMe) {
      if (pOptJacobian) pOptJacobian->Dump();
      smos_WriteBuffer(_T(" F --- \n"));
      SM_DUMP_TARRAY( rF );
  }
#endif

  return SM_SUCCESS;

} // end SmConstantRadiusFS::LoadJacobian


/*******************************************************************//**
PURPOSE: Get the number of variables the Jacobian for the constant
     radius fillet contains.

NOTES:
***********************************************************************/
ULONG SmConstantRadiusFS::GetJacobianSize
  ()
 const
{
    return 4;

} // end SmConstantRadiusFS::GetJacobianSize

/*******************************************************************//**
PURPOSE: Load some initial values for the solver.

NOTES: Given the current input guesses on one surface,
  find a good guess on other surface for upcoming solve.
  Append the results to rIntervals (u- and v-domains of other surface),
  rPeriodicities (in u and v of other surface), and rGuessT (u and v
  guess values on the other surface, for the offset point determined
  by the input surface crSrf1Offset and the input uv values located
  at lSurface1Index in rGuessT).

***********************************************************************/
SmStatus SmConstantRadiusFS::LoadInitialValues
  (ULONG               & rbDoSurf2Calcs,  // in : if 0, don't have to calculate Surf values.
   ULONG                 lRailIndex,      // in : target rail index, 0 or 1
   const SmSurface     & crSrf1Offset,    // in : offset srf corresponding to lRailIndex
   ULONG                 lSurface1Index,  // in : index into rGuessT of srf 1's UV values
   SmSurface          *& rpSrf2Offset,    // out: pointer to other offset surface.
   ULONG               & rlSurface2Index, // out: index into rGuessT of srf2's UV values
   SmExtentNd          & rIntervals,      // i/o: u and v domains of srf2 appended
   SmTArray<SmBoolean> & rPeriodicities,  // i/o: u and v periodicities of srf2 appended.
   SmTArray<double>    & rGuessT)         // i/o: u and v guesses of srf2 appended
{
  // No work - second surface info already calculated.
  if( rbDoSurf2Calcs == 0 )
    { return SM_SUCCESS; }

  // get Other OffsetSurface
  ULONG lOtherRailIndex = 1 - lRailIndex;
  rpSrf2Offset          = this->GetSurface( lOtherRailIndex );

  // Locals
#ifdef SM_DEBUG_CODE
  SmEdgeuse       * pEdgeuse        = this->GetEdgeuse( lRailIndex );
#endif

  SmEdgeuse       * pOtherEdgeuse   = this->GetEdgeuse( lOtherRailIndex );
  const SmSurface * pSrf2Base       = ((SmOffsetSurface*)rpSrf2Offset)->GetBaseSurface();

  SmPoint3d         sSrf2GuessUV;   // This is what we'll be returning, in rGuessT.
  SmBoolean         bFoundSrf2UVGuess = FALSE;

  SmBoolean bTmp0 = FALSE ; 
  SmBSplineSurface * pBSplineSurface0 = SM_CAST_PTR(SmBSplineSurface, pSrf2Base) ; 
  SmTemporaryChangeValue<SmBoolean> sClean0(pBSplineSurface0 ? pBSplineSurface0->GetOutOfBoundsEnabled() : bTmp0, TRUE) ;

  SmSolution        sSData[16];
  SmSolutionArray   sSolutions(16,sSData);

  // Get given 3d guess point
  SmPoint3d sSrf1GuessPt3D;
  SmPoint2d sUV( rGuessT[lSurface1Index], rGuessT[lSurface1Index+1] );
  SER( crSrf1Offset.EvaluatePoint( sUV, sSrf1GuessPt3D ) );

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
static ULONG lCount      = 1 ; lCount++ ;
static ULONG lDebugCount = 0 ;
  // Draw sSrf1GuessPt3D and see if it is on the offset Surface or not
  if (bDebugMe || lDebugCount == lCount)
    {
      SmFace    * pFace =  (SmFace *)crSrf1Offset.GetFace()  ? (SmFace *)crSrf1Offset.GetFace() 
                         : rpSrf2Offset                      ? (SmFace *)rpSrf2Offset->GetFace() 
                                                             : NULL ;
      SmEdge    * pEdge =  pEdgeuse      && pEdgeuse->GetEdge()      ? pEdgeuse->GetEdge() 
                         : pOtherEdgeuse && pOtherEdgeuse->GetEdge() ? pOtherEdgeuse->GetEdge() 
                                                                     : NULL ; 
      SmBrep    * pBrep =  pFace ? pFace->GetBrep() 
                         : pEdge ? pEdge->GetBrep() 
                                 : NULL ;

      smgfx_Erase();
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,1) ; crSrf1Offset.DrawUV() ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 1,1,0) ; rpSrf2Offset->DrawUV() ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,0,0) ; if(pFace) pFace->Draw(SM_DM_CROSSHATCH); sm_GraphicsLoop() ;
      smgfx_SetLook(3,4, 1,0,0) ; if(pEdgeuse) pEdgeuse->Draw(); sm_GraphicsLoop() ;
      smgfx_SetLook(3,4, 1,0,1) ; if(pOtherEdgeuse) pOtherEdgeuse->Draw(); sm_GraphicsLoop() ;
      smgfx_SetLook(5,7, 0,1,0) ; sSrf1GuessPt3D.Draw(); sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE


  // In the tangent-rollover case, the proper uv guess points are set in the TsectPnts
  // of the FilletVertexuses.  They might be on the other side of a seam.

  SmFilletGeom *pFG = this->GetFilletGeomForSurfaces();
  if ( pFG != NULL  &&  pFG->GetFilletGeomType() == SM_FG_TANGENT_ROLLOVER )
  {

#ifdef SM_DEBUG_CODE
SmBoolean bDebugFVUs=FALSE;
      if ( bDebugFVUs )
        { pFG->DumpFilletVUs(); }
#endif

      SmFilletEdge *pRail = pFG->GetRail( lRailIndex );
      SmTArray< SmEdgeuse * > sAllEUs;
      pRail->GetEdgeuses( sAllEUs );
      for ( ULONG ii=0; ii< sAllEUs.GetSize(); ii++ )
      {
          SmEdgeuse         *pEU  = sAllEUs[ ii ];
          SmVertexuse       *pVU  = pEU->GetVertexuse();
          SmFilletVertexuse *pFVU = SM_CAST_PTR( SmFilletVertexuse, pVU );
          if ( pFVU == NULL ) { continue; }
          SmVertex *pVtx = pFVU->GetVertex();
          SmFilletVertex *pFVtx = SM_CAST_PTR( SmFilletVertex, pVtx );
          if ( pFVtx == NULL ) { continue; }
          if ( pFVtx->GetFilletVertexType() != SM_FV_RAIL_X_EDGEUSE ) { continue; }

          // Here we have a FilletVertexuse from the stored FilletGeomForSurfaces.
          // Its uv should be set, and we should use it for srf2 guess.
          SmTsectPnt & rTsectPnt = pFVU->GetTsectPnt();
          SmIntersectionPointType eIntType = rTsectPnt.PointType();
          if ( eIntType == SM_IP_UNDEFINED || eIntType == SM_IP_UNKNOWN ) { continue; }
          sSrf2GuessUV = rTsectPnt.UVPos( 1-lRailIndex );

          bFoundSrf2UVGuess = TRUE;
          break;
      }

  } // end if our FilletGeomForSurfaces was set, and type TangentRollover.

  if ( ! bFoundSrf2UVGuess )
  {
      // get OtherSurface guess uv point

      // If we have edgeuses - use them.
      // If not - drop the GuessPoint to the other surface.

      // But first: if we are rolling on a different face (e.g., doing a rollover),
      // then this is the wrong Edgeuse, and we don't have an appropriate one to use.
      // [Fillet 408, 409]
      if ( pOtherEdgeuse != NULL )
      {
          SmFace *pF = pOtherEdgeuse->GetFace();
          if ( pF != NULL && pF->GetSurface() != pSrf2Base )
           { pOtherEdgeuse = NULL; }
      }

      if(   pOtherEdgeuse != NULL
         && pOtherEdgeuse->GetUVTrimCurve() )
      {
          // Drop pt to the filleted edge, in 3D
          SmCurve    *p3DFilletEdgeCurve = pOtherEdgeuse->GetEdge()->GetCurve();
          SmExtent1d  sIvl               = pOtherEdgeuse->GetEdge()->GetInterval();
          SER( p3DFilletEdgeCurve->GlobalPointSolve(sIvl, SM_SO_MINIMIZE,
                                                    sSrf1GuessPt3D,
                                                    m_dThisApproxTol3d, NULL, NULL,
                                                    SM_SR_SINGLE, sSolutions ));
          if ( sSolutions.GetSize() != 1 )
            { SER(SM_ERR); }

          // get OtherFilletEdge Parameter
          double dFilletEdgeT = sSolutions[0].m_vStart[0];

          // If the curve is closed, and the solution is within tol of an
          // end point, snap the solution to whichever end (start or finish)
          // such that the direction that moves towards the interior of the curve
          // also moves generally towards the interior of the surface.

          if ( p3DFilletEdgeCurve->IsClosed( sIvl ) )
          {
              SmPoint3d s3DPnt;
              SER( p3DFilletEdgeCurve->EvaluatePoint( dFilletEdgeT, s3DPnt ));
              SmVector3d sInwardVector;
              SER( crSrf1Offset.ComputeInwardVector( sUV, sInwardVector ));
              double dSnapTol = 0.001 * p3DFilletEdgeCurve->ApproximateLength( sIvl, 9 );
              double dDist, dParam;
              SmBoolean bSuccess;


              SER(p3DFilletEdgeCurve->DropPoint(sIvl,           // in : target curve allowed domain
                                                s3DPnt,         // in : Point to drop to curve
                                                &sInwardVector, // in : For Snapping, NULL=NoSnap, Vec pointing to CrvInside from desired EndPt for snapping.
                                                                //      when DropPt is within dDistTol in cpVecToCrvInside dir of EndPt, Snap to Ivl EndPt.
                                                                //      Vec handles ambiguities and makes sure that curves are not just touching at the ends.
                                                dSnapTol,       // in : Used for SnapDist in SM_SO_MINIZE case, which this is.
                                                NULL,           // in : If given, call cheap LocalPointSolve() else call expensive GlobalPointSolve()
                                                bSuccess,       // out: TRUE = found a drop point
                                                dParam,         // out: found drop curve param
                                                dDist)) ;       // out: found drop distance
                                                                // in : SM_SO_MINIMIZE = allow nonNormal drops near endPoints
                                                                //      SM_SO_INTERSECT= like Minimize but point must be within dDistTol
                                                                //      SM_SO_NORMALIZE= exclude nonNormal drops near endPoints
                                                                //      default:[SM_SO_MINIMIZE] to preserve original behavior
              if ( bSuccess )
                { dFilletEdgeT = dParam; }

          } // end if closed FilletEdgeCurve

          SmBSplineCurve *pOtherUVCurve = pOtherEdgeuse->GetUVTrimCurve();
          SER( pOtherUVCurve->EvaluatePoint( dFilletEdgeT, sSrf2GuessUV ));
          bFoundSrf2UVGuess = TRUE;

#ifdef SM_DEBUG_CODE
          // Draw sSrf1GuessPt3D and see if it is on the offset Surface or not
          if (bDebugMe)
          {
              SmPoint2d sUVDebug(sSrf2GuessUV.x, sSrf2GuessUV.y) ;
              SmFace    * pFace = (SmFace *)crSrf1Offset.GetFace() ;
              SmBrep    * pBrep = pFace ? pFace->GetBrep() : NULL ;
              SmPoint3d sEdgePoint, sBaseSurfPoint, sOffsetSurfPoint ;
              p3DFilletEdgeCurve->EvaluatePoint(dFilletEdgeT, sEdgePoint) ;
              pSrf2Base         ->EvaluatePoint( sUVDebug, sBaseSurfPoint) ;
              rpSrf2Offset      ->EvaluatePoint( sUVDebug, sOffsetSurfPoint) ;

              smgfx_Erase();
              smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
              smgfx_SetLook(1,2, 0,1,1) ; crSrf1Offset.DrawUV() ; sm_GraphicsLoop() ;
              smgfx_SetLook(1,2, 1,1,0) ; rpSrf2Offset->DrawUV() ; sm_GraphicsLoop() ;
              smgfx_SetLook(1,2, 0,0,0) ; if(pFace) pFace->Draw(SM_DM_CROSSHATCH); sm_GraphicsLoop() ;
              smgfx_SetLook(3,4, 1,0,0) ; if(pEdgeuse) pEdgeuse->Draw(); sm_GraphicsLoop() ;
              smgfx_SetLook(3,4, 1,0,1) ; if(pOtherEdgeuse) pOtherEdgeuse->Draw(); sm_GraphicsLoop() ;
              smgfx_SetLook(5,6, 0,1,0) ; sSrf1GuessPt3D.Draw(); sm_GraphicsLoop() ;
              smgfx_SetLook(6,7, 0,0,1) ; sEdgePoint.Draw(); sm_GraphicsLoop();
              smgfx_SetLook(7,8, 1,0,1) ; sBaseSurfPoint.Draw(); sm_GraphicsLoop();
              smgfx_SetLook(8,9, 1,0,0) ; sOffsetSurfPoint.Draw(); sm_GraphicsLoop();
              sm_GraphicsLoop() ;
      }
#endif // SM_DEBUG_CODE

      } // end given OtherFilletEdge check

      // If we found a uv, check that it's good enough.
      // Check how it looks on the other base surface.
      // If we are near a singularity then use global point solve.

      if ( bFoundSrf2UVGuess )
      {
          // Check parametric degeneracies:
          // (1) 1st derivs lining up,

          SmVector3d sDU, sDV;
          SmVector3d sMat[2][2];
          SmPoint2d sUVPnt( sSrf2GuessUV.x, sSrf2GuessUV.y );
          SER( pSrf2Base->Evaluate( sUVPnt, 1,1,TRUE,TRUE,TRUE, sMat[0] ));
          sDU = sMat[1][0];
          sDV = sMat[0][1];
          double dDeg;
          SER( sDU.AngleBetween( sDV, dDeg ));
          dDeg = SM_RAD2DEG( dDeg );
          if ( dDeg < 30 || dDeg > 150.0 )
          {
              bFoundSrf2UVGuess = FALSE;
          }

          // (2) one deriv going to zero.
          if (   sDU.GetMaxDimension() > 10000.0 * sDV.GetMaxDimension()
              || sDV.GetMaxDimension() > 10000.0 * sDU.GetMaxDimension() )
          {
              bFoundSrf2UVGuess = FALSE;
          }
      } // end found a UVGuessPoint from OtherEdgeuse check

      // If we didn't get a good enough uv on surface2, try a global solve
      if ( !bFoundSrf2UVGuess )
      {
          // Note: should we extend the domain (SetupOffsetExtension())?
          SER( pSrf2Base->GlobalPointSolve(pSrf2Base->GetNaturalUVDomain(),
                                           SM_SO_MINIMIZE, sSrf1GuessPt3D,
                                           SM_EFF_ZERO_SQ, NULL,
                                           SM_SR_SINGLE, sSolutions ));

          if ( sSolutions.GetSize() != 1 )
                { SER(SM_ERR); }

          bFoundSrf2UVGuess = TRUE;
          sSrf2GuessUV.x = sSolutions[0].m_vStart[0];
          sSrf2GuessUV.y = sSolutions[0].m_vStart[1];

      } // end need to use globalPointSolve to find OtherSurface UVGuessPoint check

      // Before we add the rGuessT values, drop point to other offset surface
      // to get a better guess
      SmBoolean  bFound;
      SmSolution sSol;
      SmPoint2d  sUVGuess( sSrf2GuessUV.x, sSrf2GuessUV.y );

      SmExtent2d sDomain = pSrf2Base->GetNaturalUVDomain();
      SetupOffsetExtension( pSrf2Base, sDomain ); // expands sDomain

      SER( rpSrf2Offset->LocalPointSolve(sDomain, SM_SO_MINIMIZE,
                                     sSrf1GuessPt3D, sUVGuess,
                                     bFound, sSol ));
      // when local solve worked
      if ( bFound )
      { // save solution
          double dDist = sSol.m_vStart.m_dSolutionValue;
          if ( dDist < (  smos_Fabs( m_dFilletRadii[0] )
                        + smos_Fabs( m_dFilletRadii[1] ) ) )
          {
              sSrf2GuessUV.x = sSol.m_vStart[0];
              sSrf2GuessUV.y = sSol.m_vStart[1];
          }
      }
      else // use global solve
      {
          // Note: should we extend the domain (use sDomain)?
          SER( pSrf2Base->GlobalPointSolve(pSrf2Base->GetNaturalUVDomain(),
                                           SM_SO_MINIMIZE, sSrf1GuessPt3D,
                                           SM_EFF_ZERO_SQ, NULL,
                                           SM_SR_SINGLE, sSolutions));

          if ( sSolutions.GetSize() == 1 )
          {
              sSrf2GuessUV.x = sSolutions[0].m_vStart[0];
              sSrf2GuessUV.y = sSolutions[0].m_vStart[1];
          }
      } // end drop GuessPoint to offsetSurface branches

  } // end if not bFoundSrf2UVGuess


  // Start setting the return values:
  // the next two entries in rIntervals and rPeriodicities.

  // Note: we're expanding rIntervals, but we also want to expand
  // sDomain, to allow the surface drop to converge slightly outside
  // the domain if that's where it is.
  // [BD; 25 Oct 05; 050718]

  SmExtent2d sDomain = pSrf2Base->GetNaturalUVDomain();
  SetupOffsetExtension( pSrf2Base, sDomain ); // expands sDomain

  rlSurface2Index = rGuessT.GetSize();
  rIntervals[ rlSurface2Index   ] = sDomain.GetUInterval();
  rIntervals[ rlSurface2Index+1 ] = sDomain.GetVInterval();

  rPeriodicities.Add(FALSE);
  rPeriodicities.Add(FALSE);

  // set output
  rGuessT.Add( sSrf2GuessUV.x );
  rGuessT.Add( sSrf2GuessUV.y );

#ifdef SM_DEBUG_CODE
  // Draw sSrf1GuessPt3D and see if it is on the offset Surface or not
  if (bDebugMe)
  {
      SmFace    * pFace = (SmFace *)crSrf1Offset.GetFace() ;
      SmBrep    * pBrep = pFace ? pFace->GetBrep() : NULL ;
      SmPoint2d sUVPnt(sSrf2GuessUV.x,sSrf2GuessUV.y);
      SmPoint3d s3DPnt;
      rpSrf2Offset->EvaluatePoint(sUVPnt,s3DPnt);

      smgfx_Erase();
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,1) ; crSrf1Offset.DrawUV() ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 1,1,0) ; rpSrf2Offset->DrawUV() ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,0,0) ; if(pFace) pFace->Draw(SM_DM_CROSSHATCH); sm_GraphicsLoop() ;
      smgfx_SetLook(3,4, 1,0,0) ; if(pEdgeuse) pEdgeuse->Draw(); sm_GraphicsLoop() ;
      smgfx_SetLook(3,4, 1,0,1) ; if(pOtherEdgeuse) pOtherEdgeuse->Draw(); sm_GraphicsLoop() ;
      smgfx_SetLook(5,7, 0,1,0) ; sSrf1GuessPt3D.Draw(); sm_GraphicsLoop() ;
      smgfx_SetLook(6,8, 1,0,0) ; s3DPnt.Draw(); sm_GraphicsLoop();
      sm_GraphicsLoop() ;
  }
#endif

  // all done
  return SM_SUCCESS;

} // end SmConstantRadiusFS::LoadInitialValues

/*******************************************************************//**
PURPOSE: Utility methods.
NOTES:
***********************************************************************/
SmBoolean SmConstantRadiusFS::IsKindOf( SM_TYPE t ) const
{
  return ((SmConstantRadiusFS_TYPE == t) ? TRUE : SmFilletSolver::IsKindOf( (t) ));
}
/*******************************************************************//**
PURPOSE: Dump a SmConstantRadiusFS.

NOTES: Unimplemented.
***********************************************************************/
void SmConstantRadiusFS::Dump() const {}


/*******************************************************************//**
PURPOSE: Constructor for SmConstantRadiusAssistedFS object where
    the fillet will require the help from SmConstantRadiusFS object

NOTES: The input pEdgeuse corresponds to the surface whose offset
    distance is dFilletRadius1.
***********************************************************************/
SmConstantRadiusAssistedFS::SmConstantRadiusAssistedFS
  (const SmContext & crContext,
   double dThisApproxTol3d,
   double dAngleTolerance,
   double dTangencyTolerance,
   double dFilletRadius1,
   double dFilletRadius2,
   SmEdgeuse *pEdgeuse)
  : SmConstantRadiusFS(crContext,
                       dThisApproxTol3d,
                       dAngleTolerance,
                       dTangencyTolerance,
                       dFilletRadius1,
                       pEdgeuse,
                       &dFilletRadius2)
{

} // end SmConstantRadiusAssistedFS::SmConstantRadiusAssistedFS


/*******************************************************************//**
PURPOSE: Compute the fillet values at given distance from existing point
***********************************************************************/
class SmGuessSphereSolveENFO : public SmFilletSphereSolveENFO
{
public:
    SmGuessSphereSolveENFO(const SmPoint3d & crSphereCenter,
                           double dSphereRadius,
                           SmFilletSolver & crFilletSolver)
      :SmFilletSphereSolveENFO(crSphereCenter,dSphereRadius,crFilletSolver) {}
    virtual SmStatus Evaluate(const SmTArray<double> & crX,             // in : x of Ax=F
                              SmTArray<double>       & rF,              // out: F of Ax=F function values,
                              SmMatrix               * pOptJacobian,    // out: Partial derivatives of the functions.
                              SmBoolean              & rbFoundAnswer);  // out: Not always used, when used
                                                                        //      TRUE = converged (F members are within tolerance of 0.0
                                                                        //      FALSE= Not Used or Not Converged
} ; // end class SmGuessSphereSolveENFO


/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
SmStatus SmGuessSphereSolveENFO::Evaluate
  (const SmTArray<double> & crX,           // in : x of Ax=F
   SmTArray<double>       & rF,            // out: F of Ax=F function values,
   SmMatrix               * pOptJacobian,  // out: Partial derivatives of the functions.
   SmBoolean              & rbFoundAnswer) // out: TRUE =
                                           //      FALSE=

{
    rbFoundAnswer = FALSE;

    if (pOptJacobian) {
        for (ULONG ll=0; ll<pOptJacobian->GetNumRows(); ll++) {
            for (ULONG mm=0; mm<pOptJacobian->GetNumColumns(); mm++) {
                (*pOptJacobian)[ll][mm] = 0.0;
            }
            rF[ll] = 0.0;
        }
    }

    ULONG lNumEquations = 1;
    ULONG lNumParameters = 2;
    SmSurface *pSurface = m_crFilletSolver.GetSurface(0);
    ULONG lSurf1Offset = 0;
    SmSurface * pSurface2 = NULL;
    ULONG lSurf2Offset = 0;
    SmBoolean bFoundAnswer;
    ULONG lRailIndex = 0;

    SmConstantRadiusFS * cpFS = (SmConstantRadiusFS*)&m_crFilletSolver;
    if (cpFS->SmConstantRadiusFS::LoadJacobian(lNumEquations, lNumParameters,
        lRailIndex, pSurface, lSurf1Offset, pSurface2, lSurf2Offset,
        crX, rF, pOptJacobian, bFoundAnswer) != SM_SUCCESS) {
        return SM_ERR;
    }

    SmPoint2d sUV(crX[0],crX[1]);
    SmPoint3d sF;
    SmVector3d sDUF, sDVF;
    SER(m_crFilletSolver.GetSurface(0)->Evaluate1stDerivatives(sUV,TRUE,TRUE,sF,sDUF,sDVF));

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe2 = FALSE;
    if (bDebugMe2) {
        sm_GraphicsLoop();
        smgfx_SetColor(1,0,0);
        m_crFilletSolver.GetSurface(0)->DrawAt(sUV,1);
        smgfx_SetColor(0,0,1);
        SmPoint2d sUV2(crX[lSurf2Offset],crX[lSurf2Offset+1]);
        m_crFilletSolver.GetSurface(1)->DrawAt(sUV2,1);
        sm_GraphicsLoop();
    }
#endif
    // Let's try using the plane instead of the sphere equation
    // f0 = A*x + B*y + C*z + D
    // || A*du.x + B*du.y + C*du.z    A*dv.x + B*dv.y + C*dv.z

    if (m_bPlaneAdjusts) {
        SmVector3d sNorm1 = sDUF * sDVF;
        if (sNorm1.LengthSquared() < SM_EFF_ZERO_SQ) {
            SER(m_crFilletSolver.GetSurface(0)->EvaluateNormal(sUV,TRUE,TRUE,sNorm1));
        }
        SER(sNorm1.Unitize());

        SmPoint2d sUV2(crX[lSurf2Offset],crX[lSurf2Offset+1]);
        SmPoint3d sG;
        SmVector3d sDUG, sDVG;
        SER(m_crFilletSolver.GetSurface(1)->Evaluate1stDerivatives(sUV2,TRUE,TRUE,sG,sDUG,sDVG));

        SmVector3d sNorm2 = sDUG * sDVG;
        if (sNorm2.LengthSquared() < SM_EFF_ZERO_SQ) {
            SER(m_crFilletSolver.GetSurface(1)->EvaluateNormal(sUV2,TRUE,TRUE,sNorm2));
        }
        SER(sNorm2.Unitize());

        SmPoint3d sPlaneOrig = (sF + sG) / 2.0;
        SmVector3d sPlaneABC = sNorm1 * sNorm2;
        if (sPlaneABC.Dot(m_vPlaneABC) < 0.0) { sPlaneABC = - sPlaneABC; }
        m_vPlaneABC = sPlaneABC;
        m_dPlaneD = - m_vPlaneABC.Dot(sPlaneOrig);
     }

    rF[0] = m_vPlaneABC.Dot(sF) + m_dPlaneD;

    if (pOptJacobian) {
        (*pOptJacobian)[0][0] = m_vPlaneABC.Dot(sDUF);
        (*pOptJacobian)[0][1] = m_vPlaneABC.Dot(sDVF);
    }

    // Jacobian answer looks good now check our answer to see if we return
    // TRUE for rbFoundAnswer
    if (bFoundAnswer) {
        double dScaledTol = m_crFilletSolver.GetConversionTol() * (1.0 + sF.GetMaxDimension());
        if (smos_Fabs(rF[0]) < dScaledTol) {
            rbFoundAnswer = TRUE;
        }
    }

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
    if (bDebugMe) {
        SmPoint3d sPnt;
        SER(pSurface->EvaluatePoint(sUV,sPnt));
        smgfx_SetColor(1,0,0);
        sPnt.Draw();
        sm_GraphicsLoop();
        if (pOptJacobian) {
            pOptJacobian->Dump();
            SmMatrix sTmpJ(6,6);
            SmTArray<double> sTmpX(crX);
            for (ULONG ll=0; ll<sTmpJ.GetNumRows(); ll++) {
                for (ULONG mm=0; mm<sTmpJ.GetNumColumns(); mm++) {
                    sTmpJ[ll][mm] = 0.0;
                }
            }
            SER(ComputeJacobian(sTmpX,rF,sTmpJ));
            sTmpJ.Dump();
            smos_WriteBuffer(_T(" F --- \n"));
            SM_DUMP_TARRAY( rF );
        }
    }
#endif

    return SM_SUCCESS;
} // end SmGuessSphereSolveENFO::Evaluate

/*******************************************************************//**
PURPOSE: Solve fillet point on a given plane

NOTES:
***********************************************************************/
SmStatus SmConstantRadiusAssistedFS::GuessPointOnPlaneSolve
  (const SmPoint3d & crPlaneOrig,
   const SmVector3d & crPlaneNormal,
   const SmExtent2d & crUVDomain1,
   const SmExtent2d & crUVDomain2,
   const SmVector2d & rUV1,
   const SmVector2d & rUV2,
   SmBoolean & rbFoundSolution,
   SmTsectPnt & rTsectPnt)
{
    // Now Set up Solver
    ULONG lJacobianSize = GetJacobianSize();

    SmExtentNd sIntervals(lJacobianSize);

    sIntervals[0] = SmExtent1d(crUVDomain1.GetMin().x,crUVDomain1.GetMax().x);
    sIntervals[1] = SmExtent1d(crUVDomain1.GetMin().y,crUVDomain1.GetMax().y);
    sIntervals[2] = SmExtent1d(crUVDomain2.GetMin().x,crUVDomain2.GetMax().x);
    sIntervals[3] = SmExtent1d(crUVDomain2.GetMin().y,crUVDomain2.GetMax().y);

    SmBoolean alPerData[16];
    SmTArray<SmBoolean> sPeriodicities(16,alPerData);

    sPeriodicities.Add(FALSE);
    sPeriodicities.Add(FALSE);
    sPeriodicities.Add(FALSE);
    sPeriodicities.Add(FALSE);

    double adGuessData[16];
    SmTArray<double> sGuessT(16,adGuessData);
    sGuessT.Add(rUV1.x);
    sGuessT.Add(rUV1.y);
    sGuessT.Add(rUV2.x);
    sGuessT.Add(rUV2.y);

    //ULONG lNumVariables = 4; // Here is how many we have loaded so far
    ULONG lSurf1Index = 0; // UV values of Surface1
    ULONG lSurf2Index = 2; // UV values of Surface2
    //ULONG lRailIndex = 0;

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
    if (bDebugMe) {
        smgfx_Erase();
        smgfx_SetColor(1,0,0);
        crPlaneNormal.Draw(&crPlaneOrig);
        smgfx_SetColor(0,0,1);
        GetSurface(0)->DrawUV(2,2);
        GetSurface(1)->DrawUV(2,2);
        sm_GraphicsLoop();
    }
#endif

    double sdData[16];
    SmTArray<double> sSolutionVector(16,sdData);

    double dPlaneD = - crPlaneOrig.Dot(crPlaneNormal);
    SmGuessSphereSolveENFO sEvalFun(crPlaneNormal,dPlaneD,*this);

    SmLocalSolveNd sLS(sEvalFun,          // in : Define Eqns to set to Zero (defines the DOF COUNT)
                       &sIntervals,       // in : NULL or sized:[N], N = this rEvalFun DOF count
                                          //      bounds on problem parameters.
                                          //      Set m_cpIntervals[i].SetUnbounded() for ivl whose prob params don't get clamped
                       &sPeriodicities) ; // in : NULL or sized:[N], N = this rEvalFun DOF count
                                          //      ivl[i] == TRUE = Interval[i] is periodic and values get wrapped
                                          //      ivl[i] == FALSE= Interval[i] is NOT periodic and values get clamped

    sLS.SetDesiredAccuracy(SM_EFF_ZERO_SQ);
    sLS.SetBoundaryHandler(SM_BH_TOTAL_BOUNDARY_HITS,8);
    sLS.SetMaximumIterations(30);

    // Solve it
    SER(sLS.SolveIt(sGuessT,m_dThisApproxTol3d,rbFoundSolution,sSolutionVector));

    // Try it again and allow the plane to adjust as convergance happens
    if (!rbFoundSolution) {
        sEvalFun.SetPlaneAdjusts(TRUE);
        SER(sLS.SolveIt(sGuessT,m_dThisApproxTol3d,rbFoundSolution,sSolutionVector));
    }

    if (rbFoundSolution) {
        rTsectPnt.UVPos(0).x = sSolutionVector[lSurf1Index];
        rTsectPnt.UVPos(0).y = sSolutionVector[lSurf1Index+1];
        rTsectPnt.UVPos(1).x = sSolutionVector[lSurf2Index];
        rTsectPnt.UVPos(1).y = sSolutionVector[lSurf2Index+1];
        for (ULONG i=4; i<sSolutionVector.GetSize(); i++) {
            rTsectPnt.m_adUserDoubles[i-4] = sSolutionVector[i];
        }
    }

    return SM_SUCCESS;

} // end SmConstantRadiusAssistedFS::GuessPointOnPlaneSolve


/*******************************************************************//**
PURPOSE: Find fillet point on a given plane.

NOTES: Finds and stores surface uvPnt values such that
       BaseSurface(uvPnt)   = point on rails
       OffsetSurface(uvPnt) = point on fillet center curve (offsetSurface intersection curve)
                              and on specified plane

       Also finds and stores Surface position, tangent, and normal values
       at the solution as well as the center-curve position.

METHOD ---
  Given a point on and a plane perpendicular to the
     fillet offset-surface intersection-curve,
  Find the two intersection points between that plane and the
     fillet rail curves.

  The intersection points are found satisfying the particular equation
  set implemented within the virtual SmFilletSolver::LoadJacobian() methods
  for each SmFilletSolver derived type.
***********************************************************************/
SmStatus SmConstantRadiusAssistedFS::PointOnPlaneSolve
  (const SmPoint3d  & crPlaneOrig,         // in : pt on offset-surface xsect curve
   const SmVector3d & crPlaneNormal,       // in : tang at that offset-surface xsect curve
   const SmExtent2d & crUVDomain1,         // in : fillet-surf1 UVdomain
   const SmExtent2d & crUVDomain2,         // in : fillet-surf2 UVdomain
   const SmVector2d & rUV1,                // in : fillet-surf1 UV guess value
   const SmVector2d & rUV2,                // NotUsed: in : fillet-surf2 UV guess value
   SmBoolean        & rbFoundSolution,     // out: TRUE = next rTsectPnt found with
                                           //             SmLocalSolveNd:SmFilletSphereSolveENFO::SolveIt().
                                           //      FALSE= no next point found.
   SmTsectPnt       & rTsectPnt)           // out: gets UV values for both surfaces
                                           //        BaseSurface(uvPnt)   = points on rails
                                           //        OffsetSurface(uvPnt) = point on fillet center curve
{
  SM_REF1(rUV2) ;
  // Now Set up Solver
  ULONG lJacobianSize = GetJacobianSize();

  SmExtentNd sIntervals(lJacobianSize);

  sIntervals[0] = SmExtent1d(crUVDomain1.GetMin().x,crUVDomain1.GetMax().x);
  sIntervals[1] = SmExtent1d(crUVDomain1.GetMin().y,crUVDomain1.GetMax().y);
  sIntervals[2] = SmExtent1d(crUVDomain2.GetMin().x,crUVDomain2.GetMax().x);
  sIntervals[3] = SmExtent1d(crUVDomain2.GetMin().y,crUVDomain2.GetMax().y);

  SmBoolean alPerData[16];
  SmTArray<SmBoolean> sPeriodicities(16,alPerData);

  sPeriodicities.Add(FALSE);
  sPeriodicities.Add(FALSE);

  double adGuessData[16];
  SmTArray<double> sGuessT(16,adGuessData);
  sGuessT.Add(rUV1.x);
  sGuessT.Add(rUV1.y);

  SmSurface *pSurface1 = GetSurface(0);
  SmSurface *pSurface2 = NULL;
  ULONG lDoSurf2Calcs  = 1; // Do surface2 calculations.
  ULONG lSurf1Index    = 0;   // UV values of Surface1
  ULONG lSurf2Index;
  ULONG lRailIndex     = 0;

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
  if (bDebugMe) {
      smgfx_Erase();
      smgfx_SetColor(1,0,0);
      crPlaneNormal.Draw(&crPlaneOrig);
      smgfx_SetColor(0,0,1);
      GetSurface(0)->DrawUV(2,2);
      GetSurface(1)->DrawUV(2,2);
      sm_GraphicsLoop();
  }
#endif

  // Load the initial values
  // given current inputs - place a good guess for upcoming solve in sGuessT
  SER(LoadInitialValues(lDoSurf2Calcs,lRailIndex,
      *pSurface1, lSurf1Index, pSurface2, lSurf2Index,
      sIntervals, sPeriodicities, sGuessT));

  double sdData[16];
  SmTArray<double> sSolutionVector(16,sdData);

  // get plane equation for given PlanePt and PlaneNormal
  double dPlaneD = - crPlaneOrig.Dot(crPlaneNormal);

  // allocate NewtonRaphson evaluator - contains equation definitions being solved
  SmFilletSphereSolveENFO sEvalFun(crPlaneNormal,dPlaneD,*this);

  // allocate generic NewtonRaphson solver for this evaluator
  SmLocalSolveNd sLS(sEvalFun,          // in : Define Eqns to set to Zero (defines the DOF COUNT)
                     &sIntervals,       // in : NULL or sized:[N], N = this rEvalFun DOF count
                                        //      bounds on problem parameters.
                                        //      Set m_cpIntervals[i].SetUnbounded() for ivl whose prob params don't get clamped
                     &sPeriodicities) ; // in : NULL or sized:[N], N = this rEvalFun DOF count
                                        //      ivl[i] == TRUE = Interval[i] is periodic and values get wrapped
                                        //      ivl[i] == FALSE= Interval[i] is NOT periodic and values get clamped

  // set solve parameters
  sLS.SetDesiredAccuracy(SM_EFF_ZERO_SQ);
  sLS.SetBoundaryHandler(SM_BH_TOTAL_BOUNDARY_HITS,8);
  sLS.SetMaximumIterations(30);

  // Solve equations with NewtonRaphson iteration
  SER(sLS.SolveIt(sGuessT,m_dThisApproxTol3d,rbFoundSolution,sSolutionVector));

  // Try it again and allow the plane to adjust as convergance happens
  if (!rbFoundSolution) {
      sEvalFun.SetPlaneAdjusts(TRUE);
      SER(sLS.SolveIt(sGuessT,m_dThisApproxTol3d,rbFoundSolution,sSolutionVector));
  }

  // when a solution point was found
  if (rbFoundSolution)
    {
      rTsectPnt.UVPos(0).x = sSolutionVector[lSurf1Index];
      rTsectPnt.UVPos(0).y = sSolutionVector[lSurf1Index+1];
      rTsectPnt.UVPos(1).x = sSolutionVector[lSurf2Index];
      rTsectPnt.UVPos(1).y = sSolutionVector[lSurf2Index+1];
      for (ULONG i=4; i<sSolutionVector.GetSize(); i++) {
          rTsectPnt.m_adUserDoubles[i-4] = sSolutionVector[i];
      }

      // for both surfaces - get and store solution point surface values
      for (ULONG lSrf=0; lSrf<=1; lSrf++)
        {
          // get and store the surface point values
          SmPoint2d sUV = rTsectPnt.UVPos(lSrf);
          SER(GetSurface(lSrf)->Evaluate1stDerivatives(sUV,TRUE,TRUE,
              rTsectPnt.SrfPos(lSrf),
              rTsectPnt.SrfDu(lSrf),
              rTsectPnt.SrfDv(lSrf)));

          // Store a unitized surface normal.
          SmVector3d sNorm =   rTsectPnt.SrfDu(lSrf)
                             * rTsectPnt.SrfDv(lSrf);
          if (sNorm.LengthSquared() < SM_EFF_ZERO_SQ) {
              SER(GetSurface(lSrf)->EvaluateNormal(sUV,TRUE,TRUE,
                  sNorm));
              if (sNorm.LengthSquared() < SM_EFF_ZERO_SQ) {
                  SER(SM_ERR);
              }
          }
          sNorm.Unitize();
          rTsectPnt.SrfNorm(lSrf) = sNorm;

        } // end iter both surfaces

      // Now compute the point and vectors for the 3D curve
      // Point is average of surface points
      rTsectPnt.CrvPos() = rTsectPnt.SrfPos(0);
    }

  return SM_SUCCESS;

} // end SmConstantRadiusAssistedFS::PointOnPlaneSolve

/*******************************************************************//**
PURPOSE: Utility methods.

NOTES:
***********************************************************************/
SmBoolean SmConstantRadiusAssistedFS::IsKindOf( SM_TYPE t ) const
{
  return ((SmConstantRadiusAssistedFS_TYPE == t) ? TRUE : SmConstantRadiusFS::IsKindOf( (t) ));
}
/*******************************************************************//**
PURPOSE: Dump a SmConstantRadiusAssistedFS.

NOTES: Unimplemented.
***********************************************************************/
void SmConstantRadiusAssistedFS::Dump() const {}


/*******************************************************************//**
PURPOSE: Constructor for Constant Distance Fillet object.

NOTES:
***********************************************************************/
SmConstantDistanceFS::SmConstantDistanceFS
  (const SmContext & crContext,
   double            dThisApproxTol3d,
   double            dAngleTolerance,
   double            dTangencyTolerance,
   double            dDistance, // Euclidian distance between the two rails
   SmEdgeuse       * pEdgeuse)
 : SmFilletSolver(crContext,
                  dThisApproxTol3d,
                  dAngleTolerance,
                  dTangencyTolerance),
   m_dDistance(dDistance)
{
    SM_ASSERT(pEdgeuse != NULL);
    m_pEdgeuses[0] = pEdgeuse;
    m_pEdgeuses[1] = pEdgeuse->GetRadial();

    // Create offset surfaces of appropriate radius in appropriate
    // direction.  Note that we may need to adjust this if the two
    // surfaces go to tangency and then back down in the opposite direction.
    // For now assume that the orientations at the center of the edge
    // work for the rest of the edge.

    SmPoint3d sPnt, sPnt2;
    SmVector3d sBinVec2, sBinVec, sFaceuseNormal, sFaceuseNormal2;
    SmEdge *pEdge = m_pEdgeuses[0]->GetEdge();
    SE(m_pEdgeuses[0]->EvaluateBinormal(pEdge->GetInterval().Evaluate(0.5),
        FALSE,sPnt,sBinVec,NULL,&sFaceuseNormal));
    SE(m_pEdgeuses[1]->EvaluateBinormal(pEdge->GetInterval().Evaluate(0.5),
        FALSE,sPnt2,sBinVec2,NULL,&sFaceuseNormal2));

    if (m_pEdgeuses[0]->GetLoopuse()->GetFaceuse()->GetOrientation() == SM_OT_OPPOSITE) {
        sFaceuseNormal = -sFaceuseNormal;
    }
    if (m_pEdgeuses[1]->GetLoopuse()->GetFaceuse()->GetOrientation() == SM_OT_OPPOSITE) {
        sFaceuseNormal2 = -sFaceuseNormal2;
    }


    m_dOrientations[0] = 1.0;
    m_dOrientations[1] = 1.0;
    double dDot1 = sBinVec2.Dot(sFaceuseNormal);
    if (smos_Fabs(dDot1) < SM_EFF_ZERO_SQRT) {
        m_pEdgeuses[0] = NULL;
        m_pEdgeuses[1] = NULL;
        SE(SM_ERR);
        return;
    }
    if (dDot1 < 0.0) {
        m_dOrientations[0] = -1.0;
    }
    double dDot2 = sBinVec.Dot(sFaceuseNormal2);
    if (smos_Fabs(dDot2) < SM_EFF_ZERO_SQRT) {
        m_pEdgeuses[0] = NULL;
        m_pEdgeuses[1] = NULL;
        SE(SM_ERR);
        return;
    }
    if (dDot2 < 0.0) {
        m_dOrientations[1] = -1.0;
    }

    SmSurface *pSurface = m_pEdgeuses[0]->GetLoopuse()->GetFaceuse()->GetFace()->GetSurface();
    SmSurface *pSurface2 = m_pEdgeuses[1]->GetLoopuse()->GetFaceuse()->GetFace()->GetSurface();
    double dApproxRadius = m_dDistance * smos_Sqrt(2.0) / 2.0;
    m_pSurfaces[0] = new (crContext) SmOffsetSurface(dApproxRadius*m_dOrientations[0],*pSurface);
    m_pSurfaces[1] = new (crContext) SmOffsetSurface(dApproxRadius*m_dOrientations[1],*pSurface2);
    m_pSurfaces[0]->SetAllowExtension(TRUE);
    m_pSurfaces[1]->SetAllowExtension(TRUE);

    FinishConstruction();  // Take care of other things in the base class.

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
    if (bDebugMe) {
        sm_GraphicsLoop();
        smgfx_SetColor(0,0,0);
        pSurface->DrawUV(4,8);
        sm_GraphicsLoop();
        smgfx_SetColor(0,0,1);
        pSurface2->DrawUV(9,18);
        sm_GraphicsLoop();
    }
#endif


} // end SmConstantDistanceFS::SmConstantDistanceFS


/*******************************************************************//**
PURPOSE: Destructor for Constant Radius Distance fillet solver.

NOTES:
***********************************************************************/
SmConstantDistanceFS::~SmConstantDistanceFS
  ()
{

} // end SmConstantDistanceFS::~SmConstantDistanceFS destructor


/*******************************************************************//**
PURPOSE: Set the offset values to correspond to what they should be
     at this intersection point.

NOTES:
***********************************************************************/
SmStatus SmConstantDistanceFS::SetupOffsetValues
  (const SmTsectPnt & rTsectPnt,    // in : each TsectPnt stores its own radius to preserve constant distance Fillets
   double * pOffsetDist)            // out: optional fillet radius at this point
{
  // get fillet radius at this point
  double dRadius = rTsectPnt.m_adUserDoubles[0];

  // save fillet radius and set OffsetSurface Offset distances
  if (pOffsetDist) *pOffsetDist = dRadius;
  GetSurface(0)->SetOffsetDistance(dRadius*m_dOrientations[0]);
  GetSurface(1)->SetOffsetDistance(dRadius*m_dOrientations[1]);

  return SM_SUCCESS;

} // end SmConstantDistanceFS::SetupOffsetValues

/*******************************************************************//**
PURPOSE: Load the current Jacobian matrix in an incremental way
    and update the parameters.

NOTES: The Jacobian of this one uses a ball with a variable
    radius and the distance between the points equal to the length.

  With: crX = [u1 v1 u2 v2 Radius]
         F  = BaseSurf1.Position(u1, v1)
         G  = BaseSurf2.Position(u2, v2)
        nF  = BaseSurf1.UnitSurfaceNormal(u1, v1)
        nG  = BaseSurf2.UnitSurfaceNormal(u2, v2)
         R  = Radius
         L  = m_dDistance, specified distance between rail curves
  Sets:

  rF[]                 = done elsewhere
  rF[rlNumEquations+0] = (F-G+R*nF-R*nG).x
  rF[rlNumEquations+1] = (F-G+R*nF-R*nG).y
  rF[rlNumEquations+2] = (F-G+R*nF-R*nG).z
  rF[rlNumEquations+3] = (F-G)*(F-G) - L*L

  pOptJacobian[][]                 = done elsewhere
  pOptJacobian[rlNumEquations+0][] = [ duf.X+R(d|Nf|/du).X   dvf.X+R(d|Nf|/dv).X   -dug.X-R(d|Ng|/du).X   -dvg.X-R(d|Ng|/dv).X   Nf.x-Ng.x ]
  pOptJacobian[rlNumEquations+1][] = [ duf.Y+R(d|Nf|/du).Y   dvf.Y+R(d|Nf|/dv).Y   -dug.Y-R(d|Ng|/du).Y   -dvg.Y-R(d|Ng|/dv).Y   Nf.y-Ng.y ]
  pOptJacobian[rlNumEquations+2][] = [ duf.Z+R(d|Nf|/du).Z   dvf.Y+R(d|Nf|/dv).Z   -dug.Y-R(d|Ng|/du).Z   -dvg.Y-R(d|Ng|/dv).Z   Nf.z-Ng.z ]
  pOptJacobian[rlNumEquations+3][] = [ 2*(f-g)*duf           2*(f-g)*dvf           -2*(f-g)*dug           -2*(f-g)*dvg           0.0       ]

***********************************************************************/
SmStatus SmConstantDistanceFS::LoadJacobian
  (ULONG                  & rlNumEquations,       // i/o: number of equations to skip when setting
                                                  //      outputs rF and pOptJacobian
   ULONG                  & rlNumParameters,      // i/o: Gets incremented in an odd way to
                                                  //      to point to input UVPnts and Radius at appropriate times.
   ULONG                    lRailIndex,           // in : Specifies rail index of first surface.
   SmSurface             *& rpSurface1,           // in : used only to map which crX[] uvPnt to surface1
   ULONG                  & rlSurf1Offset,        // in : index of rpSurface1 uvPnt in crX - ignored if rpSurface1 == NULL
   SmSurface             *& rpSurface2,           // in : used only to map which crX[] uvPnt to surface2
   ULONG                  & rlSurf2Offset,        // in : index of rpSurface2 uvPnt in crX - ignored if rpSurface2 == NULL
   const SmTArray<double> & crX,                  // in : current solution, [u1 v1 u2 v2 radius]
   SmTArray<double>       & rF,                   // out: sets rF[rlNumEquations] to rF[rlNumEquations+3] values
   SmMatrix               * pOptJacobian,         // out: sets pOptJacobian[rlNumEquations][] to pOptJacobian[rlNumEquations+3][] rows
   SmBoolean              & rbFoundAnswer)        // out: TRUE = all rF values less than scaled m_dConversionTol
{
  // init output
  rbFoundAnswer = FALSE;

  // get lSurf1Off index into crX array for lSurf1 parameters
  ULONG lSurf1Off = rlNumParameters;
  if (   lRailIndex == 0
      && (   rpSurface1 != NULL
          && ((SmOffsetSurface*)rpSurface1)->GetBaseSurface() == GetSurface(0)->GetBaseSurface()))
    {
      lSurf1Off = rlSurf1Offset;
    }
  else if (   lRailIndex == 1
           && (   rpSurface2 != NULL
               && ((SmOffsetSurface*)rpSurface2)->GetBaseSurface() == GetSurface(0)->GetBaseSurface()))
    {
      lSurf1Off = rlSurf2Offset;
    }
  else
    {
      rlNumParameters = rlNumParameters + 2;
    }

  // get lSurf2Off index into crX array for lSurf2 parameters
  ULONG lSurf2Off = rlNumParameters;
  if (   lRailIndex == 1
      && (   rpSurface1 != NULL
          && ((SmOffsetSurface*)rpSurface1)->GetBaseSurface() == GetSurface(1)->GetBaseSurface()))
    {
      lSurf2Off = rlSurf1Offset;
    }
  else if (   lRailIndex == 0
           && (   rpSurface2 != NULL
               && ((SmOffsetSurface*)rpSurface2)->GetBaseSurface() == GetSurface(1)->GetBaseSurface()))
    {
      lSurf2Off = rlSurf2Offset;
    }
  else
    {
      rlNumParameters = rlNumParameters + 2;
    }

  // get baseSurface1 F point values for given crX values
  SmPoint3d  sF ;
  SmVector3d sDUF, sDVF, sDUVF, sDUUF, sDVVF ;
  SmPoint2d  sUV(crX[lSurf1Off],crX[lSurf1Off+1]) ;
  GetSurface(0)->SetOffsetDistance(0.0) ;
  SER(GetSurface(0)->Evaluate2ndDerivatives(sUV,TRUE,TRUE,sF,
                                             sDUF,sDVF,sDUVF,sDUUF,sDVVF));
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
  if (bDebugMe) {
      sm_GraphicsLoop();
      smgfx_SetColor(0,1,0);
      GetSurface(0)->DrawAt(sUV,1);
      sm_GraphicsLoop();
  }
#endif

  // get baseSurface2 G point values for given crX values
  SmPoint3d sG;
  SmVector3d sDUG,sDVG,sDUVG,sDUUG,sDVVG;
  SmPoint2d sUV2(crX[lSurf2Off],crX[lSurf2Off+1]);
  GetSurface(1)->SetOffsetDistance(0.0);
  SER(GetSurface(1)->Evaluate2ndDerivatives(sUV2,TRUE,TRUE,sG,
                                              sDUG,sDVG,sDUVG,sDUUG,sDVVG));
#ifdef SM_DEBUG_CODE
  if (bDebugMe) {
      sm_GraphicsLoop();
      smgfx_SetColor(1,0,0);
      GetSurface(1)->DrawAt(sUV2,1);
      sm_GraphicsLoop();
  }
#endif

  // get current radius value
  double dRadius = crX[rlNumParameters];
  rlNumParameters ++;

  // check surface tangent magnitudes
  double sDUFLenSq = sDUF.LengthSquared();
  double sDVFLenSq = sDVF.LengthSquared();

  double sDUGLenSq = sDUG.LengthSquared();
  double sDVGLenSq = sDVG.LengthSquared();

  // failure: zero length tangents
  if (   sDUFLenSq < SM_EFF_ZERO_SQ
      || sDVFLenSq < SM_EFF_ZERO_SQ
      || sDUGLenSq < SM_EFF_ZERO_SQ
      || sDVGLenSq < SM_EFF_ZERO_SQ)
    { return SM_ERR; }

  // get baseSurface SurfaceNormals at current solution points
  SmVector3d sNF = sDUF * sDVF; // Normal on base surface #1
  SmVector3d sNG = sDUG * sDVG; // Normal on base surface #2

  // failure: zero length SurfaceNormal
  if (   sNF.LengthSquared() < SM_EFF_ZERO_SQ
      || sNG.LengthSquared() < SM_EFF_ZERO_SQ)
    { SER(SM_ERR); }

  // get derivatives of SurfaceNormals
  SmVector3d sNFdu = sDUF.DerivativeOfCrossProduct(sDVF,sDUUF,sDUVF);
  SmVector3d sNFdv = sDUF.DerivativeOfCrossProduct(sDVF,sDUVF,sDVVF);
  SmVector3d sNGdu = sDUG.DerivativeOfCrossProduct(sDVG,sDUUG,sDUVG);
  SmVector3d sNGdv = sDUG.DerivativeOfCrossProduct(sDVG,sDUVG,sDVVG);

  // normalize SurfaceNormal derivatives
  SmVector3d sUnitNFdu = m_dOrientations[0]*sNF.UnitizedDerivative(sNFdu);
  SmVector3d sUnitNFdv = m_dOrientations[0]*sNF.UnitizedDerivative(sNFdv);
  SmVector3d sUnitNGdu = m_dOrientations[1]*sNG.UnitizedDerivative(sNGdu);
  SmVector3d sUnitNGdv = m_dOrientations[1]*sNG.UnitizedDerivative(sNGdv);

  // orient and normalize SurfaceNormal vectors
  sNF = sNF*m_dOrientations[0]; SER(sNF.Unitize());
  sNG = sNG*m_dOrientations[1]; SER(sNG.Unitize());

  // set sDiff1 = vector between current rail points (desire sDiff1.Length == m_dDistance
  SmVector3d sDiff1 = sF - sG;

  // set sDiff2 = vector between points on the fillet center curve (desire sDiff2.Length == 0.0)
  SmVector3d sDiff2 = sDiff1 + dRadius*(sNF-sNG);

#ifdef SM_DEBUG_CODE
  if (bDebugMe) {
      sm_GraphicsLoop();
      smgfx_SetColor(0,0,1);
      SmPoint3d sPF = sF + dRadius*sNF;
      sPF.Draw();
      sm_GraphicsLoop();
      smgfx_SetColor(0,1,1);
      SmPoint3d sPG = sG + dRadius*sNG;
      sPG.Draw();
      sm_GraphicsLoop();
      smgfx_SetColor(1,0,0);
      sDiff1.Draw(&sPG);
      sm_GraphicsLoop();
  }
#endif

  // Intersection simply tries to zero the following four equations.
  // Let L be the distance between points.
  //
  // fun[i]   = (f-g) + R(Nf-Ng) . X     - X component
  // fun[i+1] = (f-g) + R(Nf-Ng) . Y     - Y component
  // fun[i+2] = (f-g) + R(Nf-Ng) . Z     - Z component
  // fun[i+3] = (f-g)(f-g) - L*L  - Distance between points is given distance
  //
  // Where f is surface1 original, g is surface2 original
  // and R is offset distance(or radius),
  // Nf is the oriented unitized normal of f,
  // Ng is the oriented unitized normal of g.
  //
  // It produces the following Jacobian with variables U, V, U2, V2, R
  //  | duf.X+R(d|Nf|/du).X   dvf.X+R(d|Nf|/dv).X   -dug.X-R(d|Ng|/du).X   -dvg.X-R(d|Ng|/dv).X   Nf.x-Ng.x |
  //  | duf.Y+R(d|Nf|/du).Y   dvf.Y+R(d|Nf|/dv).Y   -dug.Y-R(d|Ng|/du).Y   -dvg.Y-R(d|Ng|/dv).Y   Nf.y-Ng.y |
  //  | duf.Z+R(d|Nf|/du).Z   dvf.Y+R(d|Nf|/dv).Z   -dug.Y-R(d|Ng|/du).Z   -dvg.Y-R(d|Ng|/dv).Z   Nf.z-Ng.z |
  //  | 2*(f-g)*duf           2*(f-g)*dvf           -2*(f-g)*dug           -2*(f-g)*dvg           0.0       |
  //

  if (pOptJacobian) {
      ULONG i = rlNumEquations;
      ULONG lLastColumn = smos_Max(lSurf2Off,lSurf1Off) + 2;

      (*pOptJacobian)[i][lSurf1Off]   =   sDUF.x + dRadius*sUnitNFdu.x;
      (*pOptJacobian)[i][lSurf1Off+1] =   sDVF.x + dRadius*sUnitNFdv.x;
      (*pOptJacobian)[i][lSurf2Off]   = - sDUG.x - dRadius*sUnitNGdu.x;
      (*pOptJacobian)[i][lSurf2Off+1] = - sDVG.x - dRadius*sUnitNGdv.x;
      (*pOptJacobian)[i][lLastColumn] =   sNF.x  - sNG.x;


      i++; // Move to next equation;
      (*pOptJacobian)[i][lSurf1Off]   =   sDUF.y + dRadius*sUnitNFdu.y;
      (*pOptJacobian)[i][lSurf1Off+1] =   sDVF.y + dRadius*sUnitNFdv.y;
      (*pOptJacobian)[i][lSurf2Off]   = - sDUG.y - dRadius*sUnitNGdu.y;
      (*pOptJacobian)[i][lSurf2Off+1] = - sDVG.y - dRadius*sUnitNGdv.y;
      (*pOptJacobian)[i][lLastColumn] =   sNF.y  - sNG.y;

      i++;
      (*pOptJacobian)[i][lSurf1Off]   =   sDUF.z + dRadius*sUnitNFdu.z;
      (*pOptJacobian)[i][lSurf1Off+1] =   sDVF.z + dRadius*sUnitNFdv.z;
      (*pOptJacobian)[i][lSurf2Off]   = - sDUG.z - dRadius*sUnitNGdu.z;
      (*pOptJacobian)[i][lSurf2Off+1] = - sDVG.z - dRadius*sUnitNGdv.z;
      (*pOptJacobian)[i][lLastColumn] =   sNF.z  - sNG.z;

      i++;
      (*pOptJacobian)[i][lSurf1Off]   =  2.0 * sDiff1.Dot(sDUF);
      (*pOptJacobian)[i][lSurf1Off+1] =  2.0 * sDiff1.Dot(sDVF);
      (*pOptJacobian)[i][lSurf2Off]   = -2.0 * sDiff1.Dot(sDUG);
      (*pOptJacobian)[i][lSurf2Off+1] = -2.0 * sDiff1.Dot(sDVG);
      (*pOptJacobian)[i][lLastColumn] =  0.0;
  }

  // Compute function values - distance vector between offset points on fillet center curve
  //                           distance between points on rail curves
  rF[rlNumEquations++] = sDiff2.x;
  rF[rlNumEquations++] = sDiff2.y;
  rF[rlNumEquations++] = sDiff2.z;
  double dValue        = sDiff1.Dot(sDiff1) - m_dDistance*m_dDistance;
  rF[rlNumEquations++] = dValue;

  // failure: railPoints more than 10*m_Distance apart
  if (smos_Sqrt(smos_Fabs(dValue)) > 10.0 * m_dDistance)
    { return SM_ERR;  } // No convergance just skip out of here

  // set offset surface distances = current radius solution
  GetSurface(0)->SetOffsetDistance(dRadius*m_dOrientations[0]);
  GetSurface(1)->SetOffsetDistance(dRadius*m_dOrientations[1]);

  // See if we have converged
  double dScaledTol = GetConversionTol() * (1.0 + sF.GetMaxDimension());
  if (   smos_Fabs(sDiff2.x) < dScaledTol
      && smos_Fabs(sDiff2.y) < dScaledTol
      && smos_Fabs(sDiff2.z) < dScaledTol
      && smos_Fabs(dValue)   < dScaledTol)
    {
      //smos_Fabs(rF[rlNumEquations-1]) < dScaledTol)
      //smos_Fabs(sDiff1.Length()-m_dDistance) < dScaledTol)
      rbFoundAnswer = TRUE;
    }

#ifdef SM_DEBUG_CODE
  if (bDebugMe) {
      if (pOptJacobian) pOptJacobian->Dump();
      smos_WriteBuffer(_T(" F --- \n"));
      SM_DUMP_TARRAY( rF );
  }
#endif

  // all done
  return SM_SUCCESS;

} // end SmConstantDistanceFS::LoadJacobian

/*******************************************************************//**
PURPOSE: Get the number of variables the Jacobian for the constant
     radius fillet contains.

NOTES:
***********************************************************************/
ULONG SmConstantDistanceFS::GetJacobianSize
  ()
 const
{
    return 5;

} // end SmConstantDistanceFS::GetJacobianSize

/*******************************************************************//**
PURPOSE: Load the initial values for SmConstantDistanceFS.

NOTES:
***********************************************************************/
SmStatus SmConstantDistanceFS::LoadInitialValues
  (ULONG               & rbDoSurf2Calcs,     // in :
   ULONG                 lRailIndex,         // in :
   const SmSurface     & crSurface1,         // in :
   ULONG                 lSurface1Index,     // in :
   SmSurface          *& rpSurface2,         // out:
   ULONG               & rlSurface2Index,    // out:
   SmExtentNd          & rIntervals,         // out:
   SmTArray<SmBoolean> & rPeriodicities,     // out:
   SmTArray<double>    & rGuessT)            // i/o:
{
  if ( rbDoSurf2Calcs != 0 )
    {
      // Now find the UV guess on the other Rail's surface
      SmPoint3d sPnt;
      SmPoint2d sUV(rGuessT[lSurface1Index],rGuessT[lSurface1Index+1]);
      SER(crSurface1.EvaluatePoint(sUV,sPnt));
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
      if (bDebugMe) {
          smgfx_SetColor(0,0,1);
          sPnt.Draw();
          sm_GraphicsLoop();
          smgfx_SetColor(1,0,0);
          SmPoint3d s3DPnt;
          SER(((SmOffsetSurface&)crSurface1).GetBaseSurface()->EvaluatePoint(sUV,s3DPnt));
          s3DPnt.Draw();
          sm_GraphicsLoop();
      }
#endif

      ULONG lOtherRailIndex = 1 - lRailIndex;
      rpSurface2            = GetSurface(lOtherRailIndex);
      SmPoint3d s2DPnt;

      SmSolution      sSData[16];
      SmSolutionArray sSolutions(16,sSData);

      const SmSurface *pSrf2Base = ((SmOffsetSurface*)rpSurface2)->GetBaseSurface();

      SmBoolean bTmp0 = FALSE ; 
      SmBSplineSurface * pBSplineSurface0 = SM_CAST_PTR(SmBSplineSurface, pSrf2Base) ; 
      SmTemporaryChangeValue<SmBoolean> sClean0(pBSplineSurface0 ? pBSplineSurface0->GetOutOfBoundsEnabled() : bTmp0, TRUE) ;

      // If we have edgeuses then use them to compute a point on the other surfaces
      // If not then drop the point to the other surface.
      SmBoolean bFoundUV = FALSE;
      SmEdgeuse *pOtherEU = m_pEdgeuses[lOtherRailIndex];
      if ( pOtherEU != NULL && pOtherEU->GetUVTrimCurve() == NULL )
        { pOtherEU = NULL; }

      // But: if we are rolling on a different face (e.g., doing a rollover),
      // then this is the wrong Edgeuse, and we don't have an appropriate one to use.
      if ( pOtherEU != NULL )
        {
          SmFace *pF = pOtherEU->GetFace();
          if ( pF != NULL && pF->GetSurface() != pSrf2Base )
            { pOtherEU = NULL; }
        }

      if ( pOtherEU != NULL ) 
        {
          SmCurve    *p3DFilletEdgeCurve = pOtherEU->GetEdge()->GetCurve();
          SmExtent1d  sIvl               = pOtherEU->GetEdge()->GetInterval();
          SER(p3DFilletEdgeCurve->GlobalPointSolve(sIvl,         SM_SO_MINIMIZE,
                                                   sPnt,         m_dThisApproxTol3d,
                                                   NULL,         NULL,
                                                   SM_SR_SINGLE, sSolutions));
          if (sSolutions.GetSize() != 1) SER(SM_ERR);
          double dFilletEdgeT = sSolutions[0].m_vStart[0];

          // If the curve is closed, and the solution is within tol of an
          // end point, snap the solution to whichever end (start or finish)
          // such that the direction that moves towards the interior of the
          // curve also moves generally towards the interior of the surface.

          if (p3DFilletEdgeCurve->IsClosed(sIvl))
            {
              SmPoint3d s3DPnt;
              SER( p3DFilletEdgeCurve->EvaluatePoint( dFilletEdgeT, s3DPnt ));
              SmVector3d sInwardVector;
              SER( crSurface1.ComputeInwardVector( sUV, sInwardVector ));
              double dDist, dParam;
              SmBoolean bSuccess;
              SER(p3DFilletEdgeCurve->DropPoint(sIvl,                 // in : target curve allowed domain
                                                s3DPnt,               // in : Point to drop to curve
                                                &sInwardVector,       // in : For Snapping, NULL=NoSnap, Vec pointing to CrvInside from desired EndPt for snapping.
                                                                      //      when DropPt is within dDistTol in cpVecToCrvInside dir of EndPt, Snap to Ivl EndPt.
                                                                      //      Vec handles ambiguities and makes sure that curves are not just touching at the ends.
                                                m_dThisApproxTol3d,   // in : Used for SnapDist, and SM_SO_INTERSECT sXSectTol3d distance.
                                                                      //      sXSectTol3d = dDistTol ;      SM_SO_INTERSECT 
                                                                      //      sXSectTol3d = SM_BIG_DOUBLE ; SM_SO_MINIMIZE, SM_SO_NORMALIZE
                                                                      //      sSnapTol3d  = dDistTol ;      SM_SO_MINIMIZE, SM_SO_NORMALIZE, SM_SO_INTERSECT    
                                                NULL,                 // in : If given, call cheap LocalPointSolve() else call expensive GlobalPointSolve()
                                                bSuccess,             // out: TRUE = found a drop point
                                                dParam,               // out: found drop curve param
                                                dDist)) ;             // out: found drop distance
                                                                      // in : SM_SO_MINIMIZE = allow nonNormal drops near endPoints
                                                                      //      SM_SO_INTERSECT= like Minimize but point must be within dDistTol
                                                                      //      SM_SO_NORMALIZE= exclude nonNormal drops near endPoints
                                                                      //      default:[SM_SO_MINIMIZE] to preserve original behavior
              if ( bSuccess )
                  dFilletEdgeT = dParam;
            }

          // Now determine edgeuses, surfaces and orientations for current
          // iteration of the solver.
          SmBSplineCurve *pOtherUVCurve = pOtherEU->GetUVTrimCurve();
          SER(pOtherUVCurve->EvaluatePoint(dFilletEdgeT,s2DPnt));
          bFoundUV = TRUE;
          SmSolution sSol;
          SmBoolean bFoundAnswer;
          SmPoint2d sGuess(s2DPnt.x,s2DPnt.y);
          SER(rpSurface2->LocalPointSolve(rpSurface2->GetNaturalUVDomain(),
              SM_SO_MINIMIZE,sPnt,sGuess,bFoundAnswer,sSol));

          if (bFoundAnswer)
            {
              double dDist = sSol.m_vStart.m_dSolutionValue;
              if (dDist < 2.0*m_dDistance)
                {
                  s2DPnt.x = sSol.m_vStart[0];
                  s2DPnt.y = sSol.m_vStart[1];
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe1 = FALSE;
                  if (bDebugMe1) {
                      SmPoint2d sUVTest(s2DPnt.x,s2DPnt.y);
                      SmPoint3d s3DPnt;
                      SER(pSrf2Base->EvaluatePoint(sUVTest,s3DPnt));
                      smgfx_SetColor(0,1,0);
                      s3DPnt.Draw();
                      sm_GraphicsLoop();
                      pSrf2Base->DrawUV(1,1);
                      sm_GraphicsLoop();
                  }
#endif
                }
            }
        }

      if (bFoundUV)
        {
          SmVector3d sDU, sDV;
          SmVector3d sMat[2][2];
          SmPoint2d sUVPnt(s2DPnt.x,s2DPnt.y);
          SER(pSrf2Base->Evaluate(sUVPnt,1,1,TRUE,TRUE,TRUE,sMat[0]));
          sDU = sMat[1][0];
          sDV = sMat[0][1];
          double dDeg;
          SER(sDU.AngleBetween(sDV,dDeg));
          dDeg = SM_RAD2DEG(dDeg);
          if (dDeg < 30 || dDeg > 150.0) {
              bFoundUV = FALSE;
          }
          // If we are near a singularity then use global point solve
          if (sDU.GetMaxDimension() > 10000.0 * sDV.GetMaxDimension() ||
              sDV.GetMaxDimension() > 10000.0 * sDU.GetMaxDimension()) {
              bFoundUV = FALSE;
          }
        }

      if (!bFoundUV)
        {
          SER(rpSurface2->GlobalPointSolve(rpSurface2->GetNaturalUVDomain(),
              SM_SO_MINIMIZE,sPnt,SM_EFF_ZERO_SQ,
              NULL,SM_SR_SINGLE,sSolutions));
          if (sSolutions.GetSize() != 1) SER(SM_ERR);

          s2DPnt.x = sSolutions[0].m_vStart[0];
          s2DPnt.y = sSolutions[0].m_vStart[1];
        }

      SmExtent2d sDomain = GetSurface(lOtherRailIndex)->GetBaseSurface()->GetNaturalUVDomain();
      SmVector2d sSize = sDomain.GetSize();
      sSize = m_dSurfaceExtensionFactor * sSize;

      rIntervals[rGuessT.GetSize()] = SmExtent1d(sDomain.GetMin().x-sSize.x,sDomain.GetMax().x+sSize.x);
      rIntervals[rGuessT.GetSize()+1] = SmExtent1d(sDomain.GetMin().y-sSize.y,sDomain.GetMax().y+sSize.y);

      rPeriodicities.Add(FALSE);
      rPeriodicities.Add(FALSE);

      rlSurface2Index = rGuessT.GetSize();

      rGuessT.Add(s2DPnt.x);
      rGuessT.Add(s2DPnt.y);
  }

  // Last of all load the distance
  rIntervals[rlSurface2Index+2] = SmExtent1d(0.0,m_dDistance*10.0);
  rPeriodicities.Add(FALSE);
  double dApproxRadius          = m_dDistance * smos_Sqrt(2.0) / 2.0;
  GetSurface(0)->SetOffsetDistance(dApproxRadius*m_dOrientations[0]);
  GetSurface(1)->SetOffsetDistance(dApproxRadius*m_dOrientations[1]);

  rGuessT.Add(dApproxRadius);

  return SM_SUCCESS;

} // end SmConstantDistanceFS::LoadInitialValues

/*******************************************************************//**
PURPOSE: Utility methods.

NOTES:
***********************************************************************/
SmBoolean SmConstantDistanceFS::IsKindOf( SM_TYPE t ) const
{
  return ((SmConstantDistanceFS_TYPE == t) ? TRUE : SmFilletSolver::IsKindOf( (t) ));
}
/*******************************************************************//**
PURPOSE: Dump a SmConstantDistanceFS.

NOTES: Unimplemented.
***********************************************************************/
void SmConstantDistanceFS::Dump() const {}


/*******************************************************************//**
PURPOSE: Constructor for Variable Radius Fillet object.

NOTES:
***********************************************************************/
SmVariableRadiusFS::SmVariableRadiusFS
  (const SmContext & crContext,
   double            dThisApproxTol3d,
   double            dAngleTolerance,
   double            dTangencyTolerance,
   double         /* dDistance */,       // Euclidian distance between the two rails
   SmEdgeuse       * pEdgeuse,
   SmFilletLaw     & rFilletLaw,
   SmBoolean         bLawOrientation)
 : SmCurveBasedFS(crContext, dThisApproxTol3d, dAngleTolerance, dTangencyTolerance),
   m_rLaw(&rFilletLaw),
   m_bLawOrientation(bLawOrientation)
{
    SM_ASSERT(pEdgeuse != NULL);
    m_pEdgeuses[0] = pEdgeuse;
    m_pEdgeuses[1] = pEdgeuse->GetRadial();

    SmEdge *pEdge = pEdgeuse->GetEdge();
    m_vLawInterval = pEdge->GetInterval();

    m_pOriginal = pEdge->GetCurve();

    if (m_pOriginal->GetDegree() > 1) {
        double dAccuracy = dThisApproxTol3d*10.0;
        double dEdgeArcLength = 0.0;
        SE(m_pOriginal->Length(m_vLawInterval,dAccuracy,dEdgeArcLength));
        rFilletLaw.SetEdgeArcLength(dEdgeArcLength);
    }

    // Create offset surfaces of appropriate radius in appropriate
    // direction.  Note that we may need to adjust this if the two
    // surfaces go to tangency and then back down in the opposite direction.
    // For now assume that the orientations at the center of the edge
    // work for the rest of the edge.

    SmPoint3d sPnt, sPnt2;
    SmVector3d sBinVec2, sBinVec, sFaceuseNormal, sFaceuseNormal2;

    SE(m_pEdgeuses[0]->EvaluateBinormal(pEdge->GetInterval().Evaluate(0.5),
        FALSE,sPnt,sBinVec,NULL,&sFaceuseNormal));
    SE(m_pEdgeuses[1]->EvaluateBinormal(pEdge->GetInterval().Evaluate(0.5),
        FALSE,sPnt2,sBinVec2,NULL,&sFaceuseNormal2));

    if (m_pEdgeuses[0]->GetLoopuse()->GetFaceuse()->GetOrientation() == SM_OT_OPPOSITE) {
        sFaceuseNormal = -sFaceuseNormal;
    }
    if (m_pEdgeuses[1]->GetLoopuse()->GetFaceuse()->GetOrientation() == SM_OT_OPPOSITE) {
        sFaceuseNormal2 = -sFaceuseNormal2;
    }


    m_dOrientations[0] = 1.0;
    m_dOrientations[1] = 1.0;
    double dDot1 = sBinVec2.Dot(sFaceuseNormal);
    if (smos_Fabs(dDot1) < SM_EFF_ZERO_SQRT) {
        m_pEdgeuses[0] = NULL;
        m_pEdgeuses[1] = NULL;
        return;
    }
    if (dDot1 < 0.0) {
        m_dOrientations[0] = -1.0;
    }
    double dDot2 = sBinVec.Dot(sFaceuseNormal2);
    if (smos_Fabs(dDot2) < SM_EFF_ZERO_SQRT) {
        m_pEdgeuses[0] = NULL;
        m_pEdgeuses[1] = NULL;
        SE(SM_ERR);
        return;
    }
    if (dDot2 < 0.0) {
        m_dOrientations[1] = -1.0;
    }

    double dVals[3];
    SE(rFilletLaw.Evaluate(m_vLawInterval.Evaluate(0.5),m_vLawInterval,m_bLawOrientation,dVals));
    double dApproxRadius = dVals[0];

    SmSurface *pSurface = m_pEdgeuses[0]->GetLoopuse()->GetFaceuse()->GetFace()->GetSurface();
    SmSurface *pSurface2 = m_pEdgeuses[1]->GetLoopuse()->GetFaceuse()->GetFace()->GetSurface();
    m_pSurfaces[0] = new (crContext) SmOffsetSurface(dApproxRadius*m_dOrientations[0],*pSurface);
    m_pSurfaces[1] = new (crContext) SmOffsetSurface(dApproxRadius*m_dOrientations[1],*pSurface2);
    m_pSurfaces[0]->SetAllowExtension(TRUE);
    m_pSurfaces[1]->SetAllowExtension(TRUE);

    FinishConstruction();  // Take care of other things in the base class.

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
    if (bDebugMe) {
        sm_GraphicsLoop();
        smgfx_SetColor(0,0,0);
        pSurface->DrawUV(4,8);
        sm_GraphicsLoop();
        smgfx_SetColor(0,0,1);
        pSurface2->DrawUV(9,18);
        sm_GraphicsLoop();
    }
#endif


} // end SmVariableRadiusFS::SmVariableRadiusFS constructor

/*******************************************************************//**
PURPOSE: Constructor for Variable Radius Fillet object.

NOTES:
***********************************************************************/
SmVariableRadiusFS::SmVariableRadiusFS
  (const SmContext & crContext,
   double            dThisApproxTol3d,
   double            dAngleTolerance,
   double            dTangencyTolerance,
   SmSurface       * pSurface,
   SmSurface       * pSurface2,
   double            dOffsetOrientation,
   double            dOffsetOrientation2,
   SmCurve         * pCenterCurve,
   SmFilletLaw     & rFilletLaw,
   SmBoolean         bLawOrientation)
 : SmCurveBasedFS(crContext,
                  dThisApproxTol3d,
                  dAngleTolerance,
                  dTangencyTolerance),
   m_rLaw(&rFilletLaw),
   m_bLawOrientation(bLawOrientation)
{
  m_pOriginal    = pCenterCurve;
  m_pEdgeuses[0] = NULL;
  m_pEdgeuses[1] = NULL;

  m_vLawInterval = m_pOriginal->GetNaturalInterval();

  if (m_pOriginal->GetDegree() > 1) 
    {
      double dAccuracy = dThisApproxTol3d*10.0;
      double dEdgeArcLength = 0.0;
      SE(m_pOriginal->Length(m_vLawInterval,dAccuracy,dEdgeArcLength));
      rFilletLaw.SetEdgeArcLength(dEdgeArcLength);
    }

  // Create offset surfaces of appropriate radius in appropriate
  // direction.
  m_dOrientations[0] = dOffsetOrientation;
  m_dOrientations[1] = dOffsetOrientation2;

  double dVals[3];
  SE(rFilletLaw.Evaluate(m_vLawInterval.Evaluate(0.5),m_vLawInterval,m_bLawOrientation,dVals));
  double dApproxRadius = dVals[0];

  m_pSurfaces[0] = new (crContext) SmOffsetSurface(dApproxRadius*m_dOrientations[0],*pSurface);
  m_pSurfaces[1] = new (crContext) SmOffsetSurface(dApproxRadius*m_dOrientations[1],*pSurface2);
  m_pSurfaces[0]->SetAllowExtension(TRUE);
  m_pSurfaces[1]->SetAllowExtension(TRUE);
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
  if (bDebugMe) 
    {
      sm_GraphicsLoop();
      smgfx_SetLook(1,2, 0,0,0); pSurface->DrawUV(4,8); sm_GraphicsLoop();
      smgfx_SetLook(1,2, 0,0,1); pSurface2->DrawUV(9,18); sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

} // end SmVariableRadiusFS::SmVariableRadiusFS constructor

/*******************************************************************//**
PURPOSE: Destructor for Variable Radius Distance fillet solver.

NOTES:
***********************************************************************/
SmVariableRadiusFS::~SmVariableRadiusFS
  ()
{

} // end SmVariableRadiusFS::~SmVariableRadiusFS destructor


/*******************************************************************//**
PURPOSE: Compute fillet radius at a given edge parameter.

NOTES:
***********************************************************************/
double SmVariableRadiusFS::GetFilletRadius
  (double dParam)
{
    double dValues[3];
    SER(m_rLaw->Evaluate(dParam,m_vLawInterval,m_bLawOrientation,dValues));
    return dValues[0];

} // end SmVariableRadiusFS::GetFilletRadius


/*******************************************************************//**
PURPOSE: Recreate the fillet surface (and fillet edges, if needed)
    Typically, this routine can be used to refine the fillet surfaces
    such that it will interpolate all the rail ends. (For example, when
    SmFilletGeom were split, it is often desirable to have the fillet
    surfaces pass through their rail ends) Another use of it is to
    compute the fillet surface patch for each SmFilletGeom when roll-over
    occurred.

NOTES: User may want to specify 'optional' pMarchDir if the
    geometry hasn't yet been computed
***********************************************************************/
SmStatus SmVariableRadiusFS::ReCalcFilletGeom
 (SmBoolean    bIsAnalyticFillet,  // in : 
  SmVector3d * pOptMarchDir,       // NotUsed: in : 
  SmCurve    * pOptRefCurve)       // NotUsed: in : 
{
  SM_REF2(bIsAnalyticFillet, pOptMarchDir) ;
  const SmContext & crContext = GetCreationContext();
    m_bExtendBefore = FALSE;
    m_bExtendAfter = FALSE;
    SmFilletGeom * pFilletGeom = m_vFilletGeoms[0];

    SmCurve * pCurve = NULL;
    SmBoolean b3DTest = TRUE;
    SmObjDelete sDelete;
//    if (m_eType == SM_FG_CORNER_FILLET)
    if (1) {
        pCurve = pOptRefCurve; NER(pCurve);
    }
    // else 
    // {
    //    // if (m_eType != SM_FG_TANGENT_ROLLOVER)
    //    //     b3DTest = FALSE;
    // 
    //     SmEdge * pEdge = GetEdgeuse(0)->GetEdge();
    //     pCurve = pEdge->GetCurve(); NER(pCurve);
    // }

    SmExtent1d sIvl = pCurve->GetNaturalInterval();
    SmBoolean bIsClosedSolverEdge = FALSE;
#if 0
    if (0 &&//m_eType != SM_FG_CORNER_FILLET && pCurve->IsClosed(sIvl)) 
    {
        bIsClosedSolverEdge = TRUE;
    }
#endif

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe0=FALSE;
    if (bDebugMe0) {
        smgfx_SetColor(0,1,0);
        pCurve->DrawWDeriv(sIvl);
        sm_GraphicsLoop();
    }
#endif
    SmOffsetSurface * pSurf1 = GetSurface(0);
    SmOffsetSurface * pSurf2 = GetSurface(1);
    double dTol = m_dThisApproxTol3d;
    //double dAngTol = GetThisAngTolRad();
    double dScale = sIvl.GetLength();


    // Sort the SmTsectPnt's of each rail-end, the rails should
    // go in the same 'direction' as the solver edge
    // First, determine if the sorting is in 2D or 3D.

    SmTsectPnt* sP1Data[64];
    SmTArray<SmTsectPnt*> sTsectPnts(64,sP1Data);
    SmTArray<double> sProjParams;
    for (ULONG i=0; i<2; i++) {
        if (!b3DTest) {
            SmEdgeuse * pEUOnSurf = GetEdgeuse(i); NER(pEUOnSurf);
            pCurve = pEUOnSurf->GetUVTrimCurve(); NER(pCurve);
        }

        SmTArray<SmFilletEdge *> sAllRails;
        sAllRails.Add(pFilletGeom->GetRail(i));
        if (i==0) {
//            sAllRails.Append(pFilletGeom->m_vOtherRails1);
        }
        else {
//            sAllRails.Append(pFilletGeom->m_vOtherRails2);
        }
        for (ULONG kk=0; kk<sAllRails.GetSize(); kk++) {
            SmFilletEdge * pRail= sAllRails[kk]; NER(pRail);
            SmTArray<SmEdgeuse*> sFilEUs;
            pRail->GetEdgeuses(sFilEUs);
            if (sFilEUs.GetSize() != 2) SER(SM_ERR);
            for (ULONG jjj=0; jjj<2; jjj++) {
                SmEdgeuse * pEU = sFilEUs[jjj];
                SmFilletVertexuse * pVU = (SmFilletVertexuse*)pEU->GetVertexuse();
                SmFilletVertex * pFV = (SmFilletVertex*)pVU->GetVertex();
                if (pFV->GetStatus() == SM_FIL_UNPROCESSED ||
                    pFV->GetFilletVertexType() == SM_FV_MATE ||
                    pFV->GetFilletVertexType() == SM_FV_RAIL_X_EXTENDED_EDGEUSE) {
                    continue;
                }
                SmTsectPnt & rTsectPnt = pVU->GetTsectPnt();
                SmPoint3d sPnt;
                if (!b3DTest) { // Do a 2D test
                    sPnt = SmPoint3d(rTsectPnt.UVPos(i));
                    if (sPnt.x == -SM_BIG_DOUBLE || sPnt.y == -SM_BIG_DOUBLE) {
                        continue;
                    }
                }
                else { // Do a 3D test
                    if (rTsectPnt.m_ePointType == SM_IP_UNKNOWN) {
//                        SmPoint2d sUV = rTsectPnt.UVPos(0);
//                        SER(SetupOffsetValues(rTsectPnt));
//                        SER(pSurf1->EvaluatePoint(sUV,rTsectPnt.CrvPos()));
                    }
                    sPnt = rTsectPnt.CrvPos();
                }

                // Drop 3D point on surface
                SmSolution sSData[8];
                SmSolutionArray sSolutions(8,sSData);
                SER(pCurve->GlobalPointSolve(sIvl,SM_SO_MINIMIZE,
                    sPnt,dTol,NULL,NULL,SM_SR_SINGLE,sSolutions));
                if (sSolutions.GetSize() != 1) SER(SM_ERR);
                double dT = sSolutions[0].m_vStart[0];
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe1=FALSE;
                if (bDebugMe1) {
                    smgfx_SetColor(0,1,0);
                    SmPoint3d sP1, sP2;
                    SmPoint2d sUV = rTsectPnt.UVPos(0);
                    GetSurface(0)->GetBaseSurface()->EvaluatePoint(sUV,sP1);
                    smgfx_SetPointSize(6);
                    sP1.Draw();
                    if (FALSE) {
                        GetSurface(0)->DrawUV(1,1);
                    }
                    sm_GraphicsLoop();
                    SmPoint2d sUV2 = rTsectPnt.UVPos(1);
                    GetSurface(1)->GetBaseSurface()->EvaluatePoint(sUV2,sP2);
                    smgfx_SetColor(1,0,0);
                    sP2.Draw();
                    if (FALSE) {
                        GetSurface(1)->DrawUV(1,1);
                    }
                    sm_GraphicsLoop();
                    rTsectPnt.CrvPos().Draw();
                    sm_GraphicsLoop();
                }
#endif
                //if (sIvl.IsValueOnBoundary(dT) && !b3DTest) {
                //    SmPoint3d sPV[2];
                //    SER(pUVCurveOnSurf->Evaluate(dT,1,1,sPV));
                //    double dAdjustment;
                //    SER(smgu_LineClosestPoint(sPV[0],sPV[1],sUVPnt,dAdjustment));
                //    dT += dAdjustment;
                //}
                if (b3DTest && sIvl.IsValueOnBoundary(dT) && bIsClosedSolverEdge) {
                    // Do this logical test if dT is on boundary of a
                    // closed edge. (Please noted that the rail is already
                    // in the same direction as the solver)
                    if (pEU->GetOrientation() == SM_OT_OPPOSITE) {
                        dT = sIvl.GetMax();
                    }
                }

                ULONG lIndexOfInsert = sProjParams.GetSize();
                SmBoolean bDoInsertion = TRUE;
                for (ULONG k=0; k<sProjParams.GetSize(); k++) {
                    if (smos_Fabs(dT-sProjParams[k]) <
                        SM_EFF_ZERO_SQRT*dScale) {
                        bDoInsertion = FALSE;
                        break;
                    }
                    else if (dT < sProjParams[k]) {
                        lIndexOfInsert = k;
                        break;
                    }
                }
                if (bDoInsertion) {
                    sProjParams.InsertAt(lIndexOfInsert, dT);
                    //rTsectPnt.m_dCurveParameter = -SM_BIG_DOUBLE;
                    sTsectPnts.InsertAt(lIndexOfInsert, &rTsectPnt);
                }
            } // for-jjj
        } // for-kk
    } // for-i

    SmExtent2d sDomain1 = pSurf1->GetNaturalUVDomain();
    SmExtent2d sDomain2 = pSurf2->GetNaturalUVDomain();
    //        if (m_eType != SM_FG_CORNER_FILLET) {
    SetupOffsetExtension(pSurf1,sDomain1);
    SetupOffsetExtension(pSurf2,sDomain2);
    //        }
    // Setup FilletIntersector
    SmFilletIntersector sFI(*pSurf1,sDomain1,
                            *pSurf2,sDomain2,
                            *this);


    SmBSplineCurve *p3DCurve = NULL ;
    SmTsectCurveType eCurveType;
    double dDeviation = 0.0;
    SmBSplineCurve *pSurfaceUVCurves[2];
//    SER(sFI.DoPointIntersection(crContext,sTsectPnts,m_bExtendBefore,
//        m_bExtendAfter,pOptMarchDir,NULL,&dTol,&dAngTol,p3DCurve,pSurfaceUVCurves[0],
//        pSurfaceUVCurves[1], eCurveType, dDeviation));
    SmBoolean bUseNormalAveraging = TRUE;
    SmBoolean bUniformSteps = TRUE;
    SER(sFI.DoLawIntersection(crContext,sTsectPnts,
                               *m_rLaw,
                               m_bLawOrientation,
                               *m_pOriginal,
                               m_vLawInterval,
                               m_dOrientations,
                               bUseNormalAveraging,
                               bUniformSteps,
                               SM_CAST_APPROXTOL3D_PTR(&m_dThisApproxTol3d),
                               &m_dThisAngTolRad,
                               p3DCurve,
                               pSurfaceUVCurves[0],
                               pSurfaceUVCurves[1],
                               eCurveType,
                               dDeviation));
    for (ULONG ii=1; ii<sTsectPnts.GetSize(); ii++) {
        SmTsectPnt * pTSP = sTsectPnts[ii];
        if (pTSP->m_dCurveParameter < 0.0) {
            return SM_ERR;
        }
    }


#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe=FALSE;
    if (bDebugMe) {
        smgfx_SetColor(1,0,0);
        pFilletGeom->GetFilletSurface()->DrawUV(1,1);
        sm_GraphicsLoop();
    }
#endif

    return SM_SUCCESS;

} // end SmVariableRadiusFS::ReCalcFilletGeom

/*******************************************************************//**
PURPOSE: Check whether the radius can change.

NOTES:
  This must return False if it's actually constant.  [090511]
***********************************************************************/
SmBoolean SmVariableRadiusFS::OffsetRadiiCanChange
  ()
{
  SmBoolean bRet = TRUE;
  if ( m_pFSG->GetFilletSurfaceGeneratorType() == SM_FSG_CIRCULAR
      && m_rLaw->IsLinearLaw() == TRUE )
    {
      SmLinearFilletLaw *pLinearLaw = SM_CAST_PTR( SmLinearFilletLaw, m_rLaw );
      if ( pLinearLaw != NULL )
        {
          if ( smos_Fabs(  pLinearLaw->GetStartRadius()
                         - pLinearLaw->GetEndRadius() ) < SM_EFF_ZERO )
            { bRet = FALSE; }
        }
    }

  return bRet;

} // end SmVariableRadiusFS::OffsetRadiiCanChange

/*******************************************************************//**
PURPOSE: Set extension flags to TRUE for each end if there is not
    a blend being done at the corner.

NOTES:
***********************************************************************/
void SmVariableRadiusFS::SetExtensionFlags
  ()
{
    SmFilletSolver::SetExtensionFlags();

    // In order to help out computing fillet radius outside the curve boundary
    // when the filleted edge is not linear. Let's extend the edge.
    SmBSplineCurve *pBSC = SM_CAST_PTR(SmBSplineCurve,m_pOriginal);
    if (pBSC) {
        double dAnalyticTol = 1.0e-8;
        SmPoint3d sLinePoint;
        SmVector3d sLineVector;
        if (   m_rLaw->GetExtendedEdgeCurve() == NULL
            && m_pOriginal->GetDegree() > 1
            && !pBSC->IsLine(5,dAnalyticTol,sLinePoint,sLineVector))
        {
            // Should come in here only once.
            SmExtent1d sIvl = m_pOriginal->GetNaturalInterval();
            SmBSplineCurve *pExtended = NULL;
            SmBoolean bPreciseExtension = TRUE;  // Otherwise it adds another length of sIvl.
            double dNewT = sIvl.GetMin() - sIvl.GetLength();
            static constexpr SmContinuityType eExtCont = SM_CT_CINFINITY;
            SE(pBSC->CreateExtendedCurve( m_crContext,
                dNewT, eExtCont, pExtended, bPreciseExtension ));
            pBSC = pExtended;
            SmObjDelete sDelObj;
            if (pExtended) {
                sDelObj.SetObj(pExtended);
            }
            dNewT = sIvl.GetMax() + sIvl.GetLength();
            SE(pBSC->CreateExtendedCurve( m_crContext,
                dNewT, eExtCont, pExtended, bPreciseExtension ));
            m_rLaw->SetExtendedEdgeCurve(pExtended);

            // Set ours as well.
            //  No, this gets deleted in m_rLaw's destructor.
            //  If we want to set this we must make a copy.
            //m_pExtended = pExtended;

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
            if (bDebugMe) {
                smgfx_SetLook( 3,5, 0,1,1 ); if( m_pOriginal ) { m_pOriginal->DrawParams(); sm_GraphicsLoop(); }
                smgfx_SetLook( 4,6, 1,0,0 ); if(   pExtended ) {   pExtended->DrawParams(); sm_GraphicsLoop(); }
                sm_GraphicsLoop();
            }
#endif // SM_DEBUG_CODE

        } // end if we want to extend the Edge curve.
    }

} // end SmVariableRadiusFS::SetExtensionFlags

/*******************************************************************//**
PURPOSE: Set the offset values to correspond to what they should be
     at this intersection point.

NOTES:
***********************************************************************/
SmStatus SmVariableRadiusFS::SetupOffsetValues
  (const SmTsectPnt & rTsectPnt,     // in : different TsectPnts get different radius fillets
   double * pdOffsetDist)            // out: optional fillet radius at this point, NULL to ignore
{
  // compute the radius value at this intersection point
  double dValues[3];
  m_rLaw->Evaluate(rTsectPnt.m_adUserDoubles[0], // in : target parameter - range:[crCurveInterval.Min,Max]
                  m_vLawInterval,                // in : crCurveInterval = interval defining range of fillet edge
                  m_bLawOrientation,             // in : bReverseOrientation: TRUE = parameters run from interval end to interval start
                  dValues);                      // out: dValues[0] = fillet-radius at dParameter value
                                                 //      dValues[1] = 1st derivative: d(fillet-radius)/d(Parameter)
                                                 //      dValues[2] = 2nd derivative: d2(fillet-radius)/d(Parameter)**2

  // save fillet radius and set OffsetSurface offset distances
  if (pdOffsetDist) *pdOffsetDist = dValues[0];
  GetSurface(0)->SetOffsetDistance(dValues[0]*m_dOrientations[0]);
  GetSurface(1)->SetOffsetDistance(dValues[0]*m_dOrientations[1]);

  return SM_SUCCESS;

} // end SmVariableRadiusFS::SetupOffsetValues

/*******************************************************************//**
PURPOSE: Load the current Jacobian matrix in an incremental way
    and update the parameters.

NOTES:
  // Intersection simply tries to zero the following 4 equations.

  // fun[i+0] = (f-g) . X     - X component of vector f-g
  // fun[i+1] = (f-g) . Y     - Y component of vector f-g
  // fun[i+2] = (f-g) . Z     - Z component of vector f-g
  // fun[i+3] = C'(t) * (C(t) - f)
  //
  // Where f is surface1 offset, g is surface2 offset and
  // where f2 is surface1 original and g2 is surface2 original
  // And C(t) is the filleted edge curve: offset point is in the plane.
  //
  // It produces the following Jacobian with variables U, V, U2, V2, T
  //  | duf.X      dvf.X       -dug.X       -dvg.X      L'N.x-L'N2.x |
  //  | duf.Y      dvf.Y       -dug.Y       -dvg.Y      L'N.y-L'N2.y |
  //  | duf.Z      dvf.Z       -dug.Z       -dvg.Z      L'N.z-L'N2.z |
  //  | C'.-duf    C'.-dvf      0.0          0.0   C''C + C'C' - C''f - C'L'N |
  //
  //  N is the unitized normal of f2 and N2 is unitized normal of g2


  With: crX   = [u1 v1 u2 v2 t]
         F    = OffsetSurf1.Position(u1, v1) for variableRadius = variableRadiusLaw(t)
         G    = OffsetSurf2.Position(u2, v2) for variableRadius = variableRadiusLaw(t)
         t    = BaseCurveParameter - typically a parameter point on the edge being filleted
        C(t)  = BaseCurve.Position
        C'(t) = BaseCurve.Tangent

  Sets:

  rF[]                 = done elsewhere
  rF[rlNumEquations+0] = (F-G) . X     - X component of vector F-G
  rF[rlNumEquations+1] = (F-G) . Y     - Y component of vector F-G
  rF[rlNumEquations+2] = (F-G) . Z     - Z component of vector F-G
  rF[rlNumEquations+3] = C'(t) * (C(t) - F)

  pOptJacobian[][]                 = done elsewhere
  pOptJacobian[rlNumEquations+0][] = [ duf.X      dvf.X       -dug.X       -dvg.X      L'N.x-L'N2.x          ]
  pOptJacobian[rlNumEquations+1][] = [ duf.Y      dvf.Y       -dug.Y       -dvg.Y      L'N.y-L'N2.y          ]
  pOptJacobian[rlNumEquations+2][] = [ duf.Z      dvf.Z       -dug.Z       -dvg.Z      L'N.z-L'N2.z          ]
  pOptJacobian[rlNumEquations+3][] = [ C'.-duf    C'.-dvf      0.0          0.0   C''C + C'C' - C''f - C'L'N ]

***********************************************************************/
SmStatus SmVariableRadiusFS::LoadJacobian
  (ULONG                  & rlNumEquations,       // i/o: number of equations to skip when setting
                                                  //      outputs rF and pOptJacobian
   ULONG                  & rlNumParameters,      // i/o: Gets incremented in an odd way to
                                                  //      to point to the input UVPnts.
   ULONG                    lRailIndex,           // in : Specifies rail index of first surface.
   SmSurface             *& rpSurface1,           // in : used only to map which crX[] uvPnt to surface1
   ULONG                  & rlSurf1Offset,        // i/o: index of rpSurface1 uvPnt in crX - ignored if rpSurface1 == NULL
   SmSurface             *& rpSurface2,           // in : used only to map which crX[] uvPnt to surface2
   ULONG                  & rlSurf2Offset,        // i/o: index of rpSurface2 uvPnt in crX - ignored if rpSurface2 == NULL
   const SmTArray<double> & crX,                  // in : current solution, [u1 v1 u2 v2 radius]
   SmTArray<double>       & rF,                   // out: sets rF[rlNumEquations] to rF[rlNumEquations+3] values
   SmMatrix               * pOptJacobian,         // out: sets pOptJacobian[rlNumEquations][] to pOptJacobian[rlNumEquations+3][] rows
   SmBoolean              & rbFoundAnswer)        // out: TRUE = all rF values less than scaled m_dConversionTol
{
  // init outputs
  rbFoundAnswer = FALSE;

  // get lSurf1Off index into crX array for lSurf1 parameters
  ULONG lSurf1Off = rlNumParameters;
  rlNumParameters = rlNumParameters + 2;
  if (lRailIndex == 0 && (rpSurface1 != NULL &&
      ((SmOffsetSurface*)rpSurface1)->GetBaseSurface() ==
      ((SmOffsetSurface*)GetSurface(0))->GetBaseSurface())) {
      lSurf1Off = rlSurf1Offset;
      rlNumParameters = rlNumParameters - 2;
  }
  if (lRailIndex == 1 && (rpSurface2 != NULL &&
      ((SmOffsetSurface*)rpSurface2)->GetBaseSurface() ==
      ((SmOffsetSurface*)GetSurface(0))->GetBaseSurface())) {
      lSurf1Off = rlSurf2Offset;
      rlNumParameters = rlNumParameters - 2;
  }

  // get lSurf2Off index into crX array for lSurf2 parameters
  ULONG lSurf2Off = rlNumParameters;
  rlNumParameters = rlNumParameters + 2;
  if (lRailIndex == 1 && (rpSurface1 != NULL &&
      ((SmOffsetSurface*)rpSurface1)->GetBaseSurface() ==
      ((SmOffsetSurface*)GetSurface(1))->GetBaseSurface())) {
      lSurf2Off = rlSurf1Offset;
      rlNumParameters = rlNumParameters - 2;
  }
  if (lRailIndex == 0 && (rpSurface2 != NULL &&
      ((SmOffsetSurface*)rpSurface2)->GetBaseSurface() ==
      ((SmOffsetSurface*)GetSurface(1))->GetBaseSurface())) {
      lSurf2Off = rlSurf2Offset;
      rlNumParameters = rlNumParameters - 2;
  }

  // get variableRadiusLaw Parameter
  double dT = crX[rlNumParameters];
  rlNumParameters ++;

  // evaluate radius its derivatives values
  double dValues[3];
  SER(m_rLaw->Evaluate(dT,m_vLawInterval,m_bLawOrientation,dValues));
  double dOffsetDistance = dValues[0];
  double dLp = dValues[1];

  // get OffsetSurface1 F point values for given crX values
  SmPoint3d sF;
  SmVector3d sDUF, sDVF;
  SmPoint2d sUV(crX[lSurf1Off],crX[lSurf1Off+1]);
  GetSurface(0)->SetOffsetDistance(dOffsetDistance*m_dOrientations[0]);
  SER(GetSurface(0)->Evaluate1stDerivatives(sUV,TRUE,TRUE,sF,sDUF,sDVF));
  SmVector3d sN = sDUF * sDVF;
  SER(sN.Unitize());
  sN = sN * m_dOrientations[0];

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
  if (bDebugMe) {
      smgfx_Erase();
      sm_GraphicsLoop();
      smgfx_SetColor(0,1,0);
      GetSurface(0)->DrawAt(sUV,1);
      sm_GraphicsLoop();
  }
#endif

  // get OffsetSurface2 G point values for given crX values
  SmPoint3d sG;
  SmVector3d sDUG, sDVG;
  SmPoint2d sUV2(crX[lSurf2Off],crX[lSurf2Off+1]);
  GetSurface(1)->SetOffsetDistance(dOffsetDistance*m_dOrientations[1]);
  SER(GetSurface(1)->Evaluate1stDerivatives(sUV2,TRUE,TRUE,sG,sDUG,sDVG));
  SmVector3d sN2 = sDUG * sDVG;
  SER(sN2.Unitize());
  sN2 = sN2 * m_dOrientations[1];

#ifdef SM_DEBUG_CODE
  if (bDebugMe) {
      sm_GraphicsLoop();
      smgfx_SetColor(1,0,0);
      GetSurface(1)->DrawAt(sUV2,1);
      sm_GraphicsLoop();
  }
#endif

  // check surface tangent magnitudes
  double sDUFLenSq = sDUF.LengthSquared();
  double sDVFLenSq = sDVF.LengthSquared();

  double sDUGLenSq = sDUG.LengthSquared();
  double sDVGLenSq = sDVG.LengthSquared();

  // failure: zero length tangents
  if (   sDUFLenSq < SM_EFF_ZERO_SQ
      || sDVFLenSq < SM_EFF_ZERO_SQ
      || sDUGLenSq < SM_EFF_ZERO_SQ
      || sDVGLenSq < SM_EFF_ZERO_SQ)
    { return SM_ERR; }

  // set sDiff1 = vector between offsetSurface points
  SmVector3d sDiff1 = sF - sG;

#ifdef SM_DEBUG_CODE
  if (bDebugMe) {
      sm_GraphicsLoop();
      smgfx_SetColor(1,0,0);
      sDiff1.Draw(&sG);
      sm_GraphicsLoop();
  }
#endif

  ULONG lLastColumn = rlNumParameters - 1;

  // set sPV = BaseCurve[pos, tang, 2nd derivative] for dT value from crX
  SmVector3d sPV[3];
  SER(EvaluateBaseCurve(dT,2,sPV));

#ifdef SM_DEBUG_CODE
  if (bDebugMe) {
      sm_GraphicsLoop();
      smgfx_SetColor(0,0,1);
      sPV[0].Draw();
      smgfx_SetColor(0,1,1);
      sPV[1].Draw(&sPV[0]);
      sm_GraphicsLoop();
  }

SmBoolean bDebugMe2 = FALSE;
  if (bDebugMe2) {
//        smgfx_Erase();
      sm_GraphicsLoop();
      smgfx_SetColor(1,0,0);
      sPV[0].Draw();
      m_pOriginal->DrawWDeriv(m_pOriginal->GetNaturalInterval(),0);
      sm_GraphicsLoop();
      if (m_pExtended) m_pExtended->DrawWDeriv(m_pExtended->GetNaturalInterval(),0);
      sm_GraphicsLoop();
      smgfx_SetColor(0,0,1);
      GetSurface(0)->SetOffsetDistance(0.0);
      GetSurface(0)->DrawAt(sUV,0);
      sm_GraphicsLoop();
      smgfx_SetColor(1,0,1);
      GetSurface(1)->SetOffsetDistance(0.0);
      GetSurface(1)->DrawAt(sUV2,0);
      sm_GraphicsLoop();
  }
#endif


  // Intersection simply tries to zero the following 4 equations.
  // It is essentially intersecting the three surfaces by intersecting
  // tangent planes.
  // fun[i+0] = (f-g) . X     - X component of vector f-g
  // fun[i+1] = (f-g) . Y     - Y component of vector f-g
  // fun[i+2] = (f-g) . Z     - Z component of vector f-g
  // fun[i+3] = C'(t) * (C(t) - f)
  //
  // Where f is surface1 offset, g is surface2 offset and
  // where f2 is surface1 original and g2 is surface2 original
  //
  // It produces the following Jacobian with variables U, V, U2, V2, T
  //  | duf.X      dvf.X       -dug.X       -dvg.X      L'N.x-L'N2.x |
  //  | duf.Y      dvf.Y       -dug.Y       -dvg.Y      L'N.y-L'N2.y |
  //  | duf.Z      dvf.Z       -dug.Z       -dvg.Z      L'N.z-L'N2.z |
  //  | C'.-duf    C'.-dvf      0.0          0.0   C''C + C'C' - C''f - C'L'N |
  //
  //  N is the unitized normal of f2 and N2 is unitized normal of g2


  if (pOptJacobian)
    {
      ULONG i = rlNumEquations;

      (*pOptJacobian)[i][lSurf1Off]   =   sDUF.x;
      (*pOptJacobian)[i][lSurf1Off+1] =   sDVF.x;
      (*pOptJacobian)[i][lSurf2Off]   = - sDUG.x;
      (*pOptJacobian)[i][lSurf2Off+1] = - sDVG.x;
      (*pOptJacobian)[i][lLastColumn] =   dLp*(sN.x - sN2.x);


      i++; // Move to next equation;
      (*pOptJacobian)[i][lSurf1Off]   =   sDUF.y;
      (*pOptJacobian)[i][lSurf1Off+1] =   sDVF.y;
      (*pOptJacobian)[i][lSurf2Off]   = - sDUG.y;
      (*pOptJacobian)[i][lSurf2Off+1] = - sDVG.y;
      (*pOptJacobian)[i][lLastColumn] =   dLp*(sN.y - sN2.y);

      i++;
      (*pOptJacobian)[i][lSurf1Off]   =   sDUF.z;
      (*pOptJacobian)[i][lSurf1Off+1] =   sDVF.z;
      (*pOptJacobian)[i][lSurf2Off]   = - sDUG.z;
      (*pOptJacobian)[i][lSurf2Off+1] = - sDVG.z;
      (*pOptJacobian)[i][lLastColumn] =   dLp*(sN.z - sN2.z);

      i++;
      (*pOptJacobian)[i][lSurf1Off]   = - sPV[1].Dot(sDUF);
      (*pOptJacobian)[i][lSurf1Off+1] = - sPV[1].Dot(sDVF);
      (*pOptJacobian)[i][lSurf2Off]   =   0.0;
      (*pOptJacobian)[i][lSurf2Off+1] =   0.0;
      (*pOptJacobian)[i][lLastColumn] =   sPV[2].Dot(sPV[0]) + sPV[1].Dot(sPV[1])
                                        - sPV[2].Dot(sF)     - sPV[1].Dot(sN)*dLp;
    }

  // Compute function values
  rF[rlNumEquations++] = sDiff1.x;
  rF[rlNumEquations++] = sDiff1.y;
  rF[rlNumEquations++] = sDiff1.z;
  SmVector3d sDiff2    = sPV[0] - sF;
  rF[rlNumEquations++] = sPV[1].Dot(sDiff2);

  // See if we have converged
  double dScaledTol = GetConversionTol() * (1.0 + sF.GetMaxDimension());
  if (smos_Fabs(sDiff1.x) < dScaledTol &&
      smos_Fabs(sDiff1.y) < dScaledTol &&
      smos_Fabs(sDiff1.z) < dScaledTol &&
      smos_Fabs(rF[rlNumEquations-1]) < dScaledTol) {
      rbFoundAnswer = TRUE;
  }

#ifdef SM_DEBUG_CODE
  if (bDebugMe) {
      if (pOptJacobian) pOptJacobian->Dump();
      smos_WriteBuffer(_T(" F --- \n"));
      SM_DUMP_TARRAY( rF );
  }
#endif

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe3 = FALSE;
  if ( bDebugMe3 ) {
      SmOffsetSurface *pOffSrf1 = this->GetSurface(   lRailIndex );
      SmOffsetSurface *pOffSrf2 = this->GetSurface( 1-lRailIndex );
      const SmSurface *pBaseSrf1 = pOffSrf1->GetBaseSurface();
      const SmSurface *pBaseSrf2 = pOffSrf2->GetBaseSurface();
      SmEdge *pE = this->GetEdgeuse(0)->GetEdge();

      if ( FALSE ) {
          smgfx_Erase();

          SmObject *pOwnerObj = pBaseSrf1->GetOwner();
          SmFace *pFace = SM_CAST_PTR( SmFace, pOwnerObj );
          SmBrep *pBrep = ( pFace == NULL ) ? NULL : pFace->GetBrep();

          if ( pBrep ) {
              smgfx_SetLook( 1,2, 0,0,0 ); pBrep->Draw(TRUE); sm_GraphicsLoop();
              sm_GraphicsLoop();
          }

          smgfx_SetLook( 3,5, 1,0,1 ); if ( pE ) { pE->Draw(); } sm_GraphicsLoop();
          sm_GraphicsLoop();
      }

      SmPoint2d sUVDebug( crX[1], crX[2] );
      SmPoint3d sBasePnt1, sOffPnt1;
      SER(pBaseSrf1->EvaluatePoint( sUVDebug, sBasePnt1 ));
      SER(pOffSrf1 ->EvaluatePoint( sUVDebug, sOffPnt1  ));

      sm_GraphicsLoop();
      smgfx_SetLook(1,4, 0,0,1); sBasePnt1.DrawPointToPoint( sOffPnt1 ); sm_GraphicsLoop();
      sm_GraphicsLoop();

      sUVDebug.Set( crX[3], crX[4] );
      SmPoint3d sBasePnt2, sOffPnt2;
      SER(pBaseSrf2->EvaluatePoint( sUVDebug, sBasePnt2 ));
      SER(pOffSrf2 ->EvaluatePoint( sUVDebug, sOffPnt2  ));

      smgfx_SetLook(1,4, 0,1,1); sBasePnt2.DrawPointToPoint( sOffPnt2 ); sm_GraphicsLoop();
      sm_GraphicsLoop();

      if ( pE ) {
          SmCurve *pCrv = pE->GetCurve();
          smgfx_SetLook( 2,3, 1,0,0 ); pCrv->DrawAt( crX[5], 1 ); sm_GraphicsLoop();
          sm_GraphicsLoop();

          // SmPoint3d sEdgePt;
          // pCrv->EvaluatePoint( crX[5], sEdgePt );
          SmVector3d sPtVt[3];
          this->EvaluateBaseCurve( crX[5], 2, sPtVt );

          SmPoint3d sMid( ( sOffPnt1 + sOffPnt2 ) / 2 );

          smgfx_SetLook(1,4, 1,0,0); sPtVt[0].DrawPointToPoint( sMid ); sm_GraphicsLoop();
          sm_GraphicsLoop();
      }
  }
#endif

  return SM_SUCCESS;

} // end SmVariableRadiusFS::LoadJacobian

/*******************************************************************//**
PURPOSE: Get the number of variables the Jacobian for the constant
     radius fillet contains.

NOTES:
***********************************************************************/
ULONG SmVariableRadiusFS::GetJacobianSize
  ()
 const
{
    return 5;

} // end SmVariableRadiusFS::GetJacobianSize

/*******************************************************************//**
PURPOSE: Load the initial values for the solver.

NOTES:
***********************************************************************/
SmStatus SmVariableRadiusFS::LoadInitialValues
  (ULONG & rbDoSurf2Calcs,
   ULONG lRailIndex,
   const SmSurface & crSurface1,
   ULONG lSurface1Index,
   SmSurface *& pSurface2,
   ULONG & rlSurface2Index,
   SmExtentNd & rIntervals,
   SmTArray<SmBoolean> & rPeriodicities,
   SmTArray<double> & rGuessT)
{
    // Now find the UV guess on the other Rail's surface
    SmPoint3d sPnt;
    SmPoint2d sUV(rGuessT[lSurface1Index],rGuessT[lSurface1Index+1]);
    SER(crSurface1.EvaluatePoint(sUV,sPnt));
    // Use the extended curve if it's there.
    const SmCurve *p3DFilletEdgeCurve = ( m_pExtended ) ? m_pExtended : m_pOriginal;

    SmSolution sSData[16];
    SmSolutionArray sSolutions(16,sSData);

    // Use the curve's whole domain.
    SmExtent1d sIvl ( p3DFilletEdgeCurve->GetNaturalInterval() );

    SER( p3DFilletEdgeCurve->GlobalPointSolve( sIvl, SM_SO_MINIMIZE, sPnt,
        m_dThisApproxTol3d, NULL, NULL, SM_SR_ALL, sSolutions ));

    if ( sSolutions.GetSize() == 2 )  // [B391]
    {
        // Not handled yet: case of single closed fillet, where m_vLawInterval == sIvl.

        // Check whether one solution is closer to our interval.
        double dT0 = sSolutions[0].m_vStart[0];
        double dT1 = sSolutions[1].m_vStart[0];

        double dDist0 = m_vLawInterval.DistanceFrom( dT0 );
        double dDist1 = m_vLawInterval.DistanceFrom( dT1 );

        if ( dDist0 < dDist1 / 2.0 )
          { sSolutions.RemoveAt( 1 ); }
        else if ( dDist1 < dDist0 / 2.0 )
          { sSolutions.RemoveAt( 0 ); }
        else
          {
            // See whether one has a better solution value.
            double dVal0 = sSolutions[0].m_vStart.m_dSolutionValue;
            double dVal1 = sSolutions[1].m_vStart.m_dSolutionValue;
            if ( dVal0 < dVal1 * 0.9 )
              { sSolutions.RemoveAt( 1 ); }
            else if ( dVal1 < dVal0 * 0.9 )
              { sSolutions.RemoveAt( 0 ); }
            else
              {
                SE_MSG(SM_ERR, _T("Questionble Point Drop in LoadInitialValues(): Please investigate."));
                sSolutions.RemoveAt( 1 ); // This preserves previous behavior.
              }
          }
    } // end if two solutions


    if (sSolutions.GetSize() != 1)
      { SER(SM_ERR); }

    // Now determine edgeuses, surfaces and orientations for current iteration of the solver.
    double dFilletEdgeT = sSolutions[0].m_vStart[0];

    // If start point is beyond end of curve in one direction or the
    // other then use the parameter on the linearly extended curve.
    if ( ! sIvl.ContainsValue( dFilletEdgeT, -SM_EFF_ZERO ) ) // Neg tol: return False for on boundary
    {
        SmPoint3d sPV[2];
        SER(p3DFilletEdgeCurve->Evaluate( dFilletEdgeT, 1, TRUE, sPV ));
        double dParamExtension;
        SER(smgu_LineClosestPoint( sPV[0], sPV[1], sPnt, dParamExtension ));
        dFilletEdgeT = dFilletEdgeT + dParamExtension;
    }

    SmTsectPnt sTsectPnt;
    sTsectPnt.m_adUserDoubles[0] = dFilletEdgeT; // SetupOffsetValues() looks for this here.
    double dRadius;
    SER( SetupOffsetValues( sTsectPnt, &dRadius ));
    SER( crSurface1.EvaluatePoint( sUV, sPnt ));

    // Load surface 2 if neccesary
    if ( rbDoSurf2Calcs != 0 )
      {
        ULONG lOtherRailIndex = 1 - lRailIndex;
        SmExtent2d sDomain = GetSurface(1-lRailIndex)->GetBaseSurface()->GetNaturalUVDomain();
        SmVector2d sSize = sDomain.GetSize();
        sSize = m_dSurfaceExtensionFactor * sSize;
        SmExtent2d sExtendedDomain(SmPoint2d(sDomain.GetMin().x-sSize.x,sDomain.GetMin().y-sSize.y),
            SmPoint2d(sDomain.GetMax().x+sSize.x, sDomain.GetMax().y+sSize.y) );

        pSurface2 = GetSurface(1-lRailIndex);

        SmPoint3d s2DPnt;

        // If we have edgeuses then use them to compute a point on the other surfaces
        // If not then drop the point to the other surface.
        SmEdgeuse *pOtherEU = m_pEdgeuses[lOtherRailIndex];
        if ( pOtherEU != NULL && pOtherEU->GetUVTrimCurve() == NULL )
          { pOtherEU = NULL; }

        // But: if we are rolling on a different face (e.g., doing a rollover),
        // then this is the wrong Edgeuse, and we don't have an appropriate one to use.
        if ( pOtherEU != NULL )
          {
            SmFace *pF = pOtherEU->GetFace();
            const SmSurface *pSrf2Base = ((SmOffsetSurface*)pSurface2)->GetBaseSurface();
            if ( pF != NULL && pF->GetSurface() != pSrf2Base )
              { pOtherEU = NULL; }
          }

        if ( pOtherEU != NULL)
          {
            SmBSplineCurve *pOtherUVCurve = m_pEdgeuses[lOtherRailIndex]->GetUVTrimCurve();
            SER(pOtherUVCurve->EvaluatePoint(sSolutions[0].m_vStart[0],s2DPnt));
            // Before we add it drop point to other surface to get a better guess
            SmBoolean bFound;
            SmSolution sSol;
            SmPoint2d sUVGuess(s2DPnt.x,s2DPnt.y);

            SER(pSurface2->LocalPointSolve(sExtendedDomain,SM_SO_MINIMIZE,sPnt,sUVGuess,bFound,sSol));
            if (bFound) {
                double dDist = sSol.m_vStart.m_dSolutionValue;
                if (dDist < 1.1*dRadius) {
                    s2DPnt.x = sSol.m_vStart[0];
                    s2DPnt.y = sSol.m_vStart[1];
                }
            }
        }
        else
        {
          // Do global solve
          SmSolution sSol[16];
          SmSolutionArray sSolutionArray( 16, sSol );
          SER( pSurface2->GlobalPointSolve( sExtendedDomain, SM_SO_MINIMIZE, sPnt, SM_EFF_ZERO_SQ, NULL, SM_SR_SINGLE, sSolutionArray ) );
          if(sSolutionArray.GetSize() == 1)
          {
            s2DPnt.x = sSolutionArray[0].m_vStart[0];
            s2DPnt.y = sSolutionArray[0].m_vStart[1];
          }
        }

        rIntervals[rGuessT.GetSize()] = SmExtent1d(sDomain.GetMin().x-sSize.x,sDomain.GetMax().x+sSize.x);
        rIntervals[rGuessT.GetSize()+1] = SmExtent1d(sDomain.GetMin().y-sSize.y,sDomain.GetMax().y+sSize.y);

        rPeriodicities.Add(FALSE);
        rPeriodicities.Add(FALSE);
        rlSurface2Index = rGuessT.GetSize();
        rGuessT.Add(s2DPnt.x);
        rGuessT.Add(s2DPnt.y);

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
        if (bDebugMe) {
            sm_GraphicsLoop();
            sPnt.Draw();
            sm_GraphicsLoop();
            p3DFilletEdgeCurve->DrawAt(dFilletEdgeT,2);
            sm_GraphicsLoop();
            SmPoint2d sUVPt(s2DPnt.x,s2DPnt.y);
            pSurface2->DrawAt( sUVPt,2);
            sm_GraphicsLoop();
            GetSurface(lRailIndex)->DrawUV(4,8);
            sm_GraphicsLoop();
            smgfx_SetColor(0,0,1);
            GetSurface(1-lRailIndex)->DrawUV(9,18);
            sm_GraphicsLoop();
        }
#endif

    }

    // Now load the edge parameter data.
    // Make Edge extension factor go out further than surface extension (5x)
    double dEdgeExtension = 5.0 * m_dSurfaceExtensionFactor * (sIvl.GetMax() - sIvl.GetMin());
    rIntervals[rlSurface2Index+2] = SmExtent1d(sIvl.GetMin() - dEdgeExtension,
                                                 sIvl.GetMax() + dEdgeExtension);

    rPeriodicities.Add(FALSE);

    rGuessT.Add(dFilletEdgeT);


    return SM_SUCCESS;

} // end SmVariableRadiusFS::LoadInitialValues

/*******************************************************************//**
PURPOSE:
NOTES:
***********************************************************************/
SmBoolean SmVariableRadiusFS::IsKindOf( SM_TYPE t ) const
{
  return ((SmVariableRadiusFS_TYPE == t) ? TRUE : SmCurveBasedFS::IsKindOf( (t) ));
}

/*******************************************************************//**
PURPOSE: Dump a SmVariableRadiusFS.

NOTES: Unimplemented.
***********************************************************************/
void SmVariableRadiusFS::Dump() const {}


/*******************************************************************//**
PURPOSE: Evaluate either the original base curve or an extension of it.

NOTES: Typically the base curve is either an edge or an extension
    of the edge.  In the future it might be worth while to reapproximate
    the base curve to get smoother fillets.
***********************************************************************/
SmStatus SmCurveBasedFS::EvaluateBaseCurve
  (double dParameter,                  // in : target parameter
   ULONG lNumDerivatives,              // in : number of derivatives to output
   SmVector3d aPointAndDerivatives[])  // out: array of [pos, 1st deriv, 2nd deriv, ...]
                                       //      sized:[1+lNumDerivatives]
{
  // pick curve to use
  const SmCurve   *pCurve =  (m_pExtended)
                            ? m_pExtended
                            : m_pOriginal;

  // get curve interval
  SmExtent1d sIvl = pCurve->GetNaturalInterval();

  // when parameter is not within interval
  if (!sIvl.ContainsValue(dParameter))
    {
      // pick parameter space tolerance
      double dScaledTol = SM_EFF_ZERO * (1.0 + smos_Fabs(sIvl.GetMin()) + smos_Fabs(sIvl.GetMax()));

      // snap to begin point
      if (   dParameter < sIvl.GetMin()
          && dParameter + dScaledTol > sIvl.GetMin())
        {
          dParameter = sIvl.GetMin();
        }

      // snap to end point
      if (   dParameter > sIvl.GetMax()
          && dParameter - dScaledTol < sIvl.GetMax())
        {
          dParameter = sIvl.GetMax();
        }

      // when parameter is still outside interval
      if (!sIvl.ContainsValue(dParameter))
        {
          // Works better to extend the original: extending the extension
          // results in progressively smaller extensions.  [B551]
          pCurve = m_pOriginal;

          // when curve is a BSpline
          if (pCurve->IsKindOf(SmBSplineCurve_TYPE))
            {
              // create and store an extended curve
              SmBSplineCurve *pBSC = SM_CAST_PTR(SmBSplineCurve,pCurve); NER(pBSC);
              SmBSplineCurve *pExtended = NULL ;
              SmBoolean bPreciseExtension = TRUE;  // Otherwise it adds another length of sIvl.
              // But extend farther than requested, so that we won't have to do this too often.
              // This doubles the extension distance:
              double dExtendParam = ( dParameter<sIvl.GetMin() )
                          ? dParameter - ( sIvl.GetMin() - dParameter )
                          : dParameter + ( dParameter - sIvl.GetMax() );
              // But never do a really tiny extension: it might get snapped to the end,
              // and therefore not extended at all.  [Fillet 308]
              double dLen = sIvl.GetLength();
              if ( dExtendParam < sIvl.GetMin() )
              {
                  if ( dExtendParam > sIvl.GetMin() - dLen / 10 )
                    {  dExtendParam = sIvl.GetMin() - dLen / 10; }
              }
              else
              {
                  if ( dExtendParam < sIvl.GetMax() + dLen / 10 )
                    {  dExtendParam = sIvl.GetMax() + dLen / 10; }
              }
              SER(pBSC->CreateExtendedCurve( *pCurve->GetContext(), dExtendParam, SM_CT_G1,
                                              pExtended, bPreciseExtension ));
              if (m_pExtended) { delete m_pExtended; m_pExtended = NULL ; }
              m_pExtended = pExtended;
              pCurve      = pExtended;
            }
          else // Unable to extend non BSpline curves
            { SER(SM_ERR);
            }
        } // end parameter is still not within interval check after snapping endPoints
    } // end parameter is not within interval check

  // evaluate the curve
  SER(pCurve->Evaluate(dParameter,lNumDerivatives,TRUE,aPointAndDerivatives));

  // all done
  return SM_SUCCESS;

} // end SmCurveBasedFS::EvaluateBaseCurve


/*******************************************************************//**
PURPOSE: Destructor for Curve Based fillet solver.  It deletes any
     extended curves left.

NOTES: It assumes that it does not own the original curve.
***********************************************************************/
SmCurveBasedFS::~SmCurveBasedFS
  ()
{
    if (m_pExtended) { delete m_pExtended; m_pExtended = NULL ; }

} // end SmCurveBasedFS::~SmCurveBasedFS destructor

/*******************************************************************//**
PURPOSE: Utility methods.

NOTES:
***********************************************************************/
SmBoolean SmCurveBasedFS::IsKindOf( SM_TYPE t ) const
{
  return ((SmCurveBasedFS_TYPE == t) ? TRUE : SmFilletSolver::IsKindOf( (t) ));
}
/*******************************************************************//**
PURPOSE: Dump a SmCurveBasedFS.

NOTES: Unimplemented.
***********************************************************************/
void SmCurveBasedFS::Dump() const {}



/*******************************************************************//**
PURPOSE: Create a surface with a linear cross section.  This
    is simply a ruled surface or in some special cases a plane or a cone.

NOTES:
***********************************************************************/
SmStatus SmLinearCrossSectionFSG::CreateSurface
 (SmFilletSolver              & rFilletSolver,       // in : 
  SmBSplineCurve              * pCenterLine,         // NotUsed: in : 
  SmBSplineCurve              * pRail1Curve,         // in : 
  SmBSplineCurve              * pRail2Curve,         // in : 
  const SmTArray<SmTsectPnt*> & crFilletPoints,      // NotUsed: in : 
  SmSurface                   * pSurf1,              // NotUsed: in : 
  SmBSplineCurve              * pUV1,                // NotUsed: in : 
  SmSurface                   * pSurf2,              // NotUsed: in : 
  SmBSplineCurve              * pUV2,                // NotUsed: in : 
  SmBSplineSurface           *& rpFilletSurface)     // in : 
{
  SM_REF2(pCenterLine, crFilletPoints) ;
  SM_REF4(pSurf1, pUV1, pSurf2, pUV2) ; 
  SER(SmBSplineSurface::CreateRuledSurface(rFilletSolver.GetCreationContext(),
                                           *pRail1Curve,
                                           *pRail2Curve,
                                           SM_SP_V,
                                           rpFilletSurface));
  return SM_SUCCESS;

} // end SmLinearCrossSectionFSG::CreateSurface

/*******************************************************************//**
PURPOSE: Create a fillet surface with a circular cross section.  This
    is a lofted surface or in some special cases a cylinder or a torus.

NOTES:
***********************************************************************/
SmStatus SmCircularCrossSectionFSG::CreateSurface
  (SmFilletSolver & rFilletSolver,
   SmBSplineCurve * pCenterLine,
   SmBSplineCurve * pRail1Curve,
   SmBSplineCurve * pRail2Curve,
   const SmTArray<SmTsectPnt*> & /*crFilletPoints*/,
   SmSurface *pSurf1,
   SmBSplineCurve *pUV1,
   SmSurface *pSurf2,
   SmBSplineCurve *pUV2,
   SmBSplineSurface *& rpFilletSurface)
{
    double dThisApproxTol3d = rFilletSolver.GetThisApproxTol3d();

    SmBoolean bNeedG1Continuity = FALSE;
    if (rFilletSolver.GetSolverType() == SM_FS_CONST_RADIUS) {
        //bNeedG1Continuity = TRUE; // fix it later
    }
    SER(CreateSurfaceFromCurves(rFilletSolver.GetCreationContext(),
        *pCenterLine,*pRail1Curve,*pRail2Curve,dThisApproxTol3d,
        bNeedG1Continuity,pSurf1,pUV1,pSurf2,pUV2,rpFilletSurface));

    return SM_SUCCESS;

} // end SmCircularCrossSectionFSG::CreateSurface


/*******************************************************************//**
PURPOSE: Create a surface from a centerline curve and two rail curves.
    Users can specify whether G1 condition are required on both ends.
    Typically, in the cases of constant-radius filleting, it is desirable
    to have the cross-boundary derivatives(or tangent-fields) on both ends
    of the fillets be normal to the cross-section planes.

NOTES: All curves should be parameterized in the same direction
     with the same knot vector.

METHOD ---
   1. Generate CrossSections for every center/rail curve knot value.
      Every crossSection curve must have the same knot vector.
      a. When railPoints are on a circle centered on centerPoint,
         crossSection is a cricular arc.
      b. When railPoints are not on circle,
         crossSection is an interpolating curve made either by
         modifying the circle endPoints or by building an interpolating curve.
      c. When railPoints are coincident,
         cross section is a degenerate curve
   2. Make the new Surface
      a. When center, rail1, and rail2 curves are parallel lines,
         NewSurface = Ruled Surface from Start CrossSection to End CrossSection
      b. When center, rail1, and rail2 crves are arcs,
         NewSurface = Surface of Rotation
      c. Else
         NewSurface = skin surface that interpolates all cross sections
   3. When bNeedG1Continuity == TRUE and NewSurface is a skinSurface,
         The skin surface end CrossTangents are computed as an interpolation of the
         rail1 and rail2 endTangent values.

***********************************************************************/
SmStatus SmCircularCrossSectionFSG::CreateSurfaceFromCurves
  (const SmContext   & crContext,          // in : context for new object construction
   SmBSplineCurve    & rCenterLine,        // in : Loci of centerPoints
   SmBSplineCurve    & rRail1Curve,        // in : 1st NewSurface Boundary
   SmBSplineCurve    & rRail2Curve,        // in : 2nd NewSurface Boundary
   double              dThisApproxTol3d,   // in : max allowed dist between NewSurface and idealized surface
   SmBoolean           bNeedG1Continuity,  // in : TRUE =
                                           //      FALSE=
   SmSurface         *,                    // in :
   SmBSplineCurve    *,                    // in :
   SmSurface         *,                    // in :
   SmBSplineCurve    *,                    // in :
   SmBSplineSurface *& rpFilletSurface)    // out: New Surface
{

  // Init outputs
  rpFilletSurface = NULL;

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe=FALSE;
  if (bDebugMe)
    {
      SmEdge *pCenterEdge = (SmEdge *)rCenterLine.GetEdge() ;
      SmEdge *pRailEdge1  = (SmEdge *)rRail1Curve.GetEdge() ;
      SmEdge *pRailEdge2  = (SmEdge *)rRail2Curve.GetEdge() ;

      SmBrep *pBrep1 = pCenterEdge ? pCenterEdge->GetBrep() : NULL ;
      SmBrep *pBrep2 = pRailEdge1  ? pRailEdge1->GetBrep()  : NULL ;
      SmBrep *pBrep3 = pRailEdge2  ? pRailEdge2->GetBrep()  : NULL ;

      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep1) pBrep1->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,0) ; if(pBrep2 && pBrep2 != pBrep1) pBrep2->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 1,1,0) ; if(pBrep3 && pBrep3 != pBrep2 && pBrep3 != pBrep1) pBrep3->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(3,4, 1,0,0) ; rCenterLine.Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook(3,5, 0,1,0) ; rRail1Curve.Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook(3,6, 0,0,1) ; rRail2Curve.Draw() ; sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;
    }
#endif

  // locals
  SmTArray<double> sBreaks;
  SmTArray<double> sKnots, sKnots2;
  SER(rRail1Curve.GetKnots(sKnots));
  SER(rRail2Curve.GetKnots(sKnots2));

  // make sure all 3 curves have compatible knot vectors
  if (sKnots2.GetSize() != sKnots.GetSize())
    {
      SmTArray<SmBSplineCurve*> sRails;
      sRails.Add(&rRail1Curve);
      sRails.Add(&rRail2Curve);
      sRails.Add(&rCenterLine);
      SER(SmBSplineCurve::SyncronizeKnotsOfCurves(sRails,SM_EFF_ZERO_SQRT));
    }

  // local array for cross section curves and parameters
  SmTArray<SmBSplineCurve*> sCrossSections(sKnots.GetSize()*2);
  SmObjsDelete<SmBSplineCurve*> sCleanCrossSections(&sCrossSections);
  SmTArray<double> sParametersOfCrossSections;

  // Surface creation requires 4 or more cross sections so add
  // some extra knots if needed.

  // insert a mid knot for 2 knot curves
  if (sKnots.GetSize() == 2)
    {
      double dStep = (sKnots[1]-sKnots[0])/3.0;
      double dNew1 = sKnots[0] + dStep;
      double dNew2 = dNew1 + dStep;
      rRail1Curve.InsertOneKnot(dNew1,1);
      rRail2Curve.InsertOneKnot(dNew1,1);
      rRail1Curve.InsertOneKnot(dNew2,1);
      rRail2Curve.InsertOneKnot(dNew2,1);
      rRail1Curve.GetKnots(sKnots);
    }

  ULONG nKnotsOld = sKnots.GetSize();

  // insert a mid interval knot into largest span
  while( sKnots.GetSize() < 4 ) {

      ULONG lMaxSpan = 0;
      double dMaxSpanValue = 0.0;
      for (ULONG k=1; k<sKnots.GetSize(); k++)
        {
          double dSpan = sKnots[k]-sKnots[k-1];
          if (dSpan > dMaxSpanValue)
            {
              lMaxSpan = k;
              dMaxSpanValue = dSpan;
            }
        }

      double dMid = (sKnots[lMaxSpan] + sKnots[lMaxSpan-1])/2.0;
      rRail1Curve.InsertOneKnot(dMid,1);
      rRail2Curve.InsertOneKnot(dMid,1);
      rRail1Curve.GetKnots(sKnots);

      ULONG nKnotsNew = sKnots.GetSize();

      // if no knots were added then bail to avoid endless loop
      if( nKnotsNew == nKnotsOld )
        { return( SM_ERR ); }

      nKnotsOld = nKnotsNew;

    } // end while inserting knots into curves to make at least 4

  // when center, rail1, and rail2 curves are parallel lines - NewSurf = RuledSurface
  // when center, rail1, and rail2 curves are arcs           - NewSurf = SurfaceOfRevolution
  // else                                                    - NewSurf = Skinned Surface
  SmBoolean bSweep   = FALSE;
  SmBoolean bRevolve = FALSE;

  // see if newSurf == RuledSurface
  double    dAnalyticTol = 1.0e-8;
  SmPoint3d  sLinePoint;
  SmVector3d sLineVector;
  SmBoolean bIsDeg1Line =    rCenterLine.GetDegree() == 1
                          && rCenterLine.IsLine(11,dAnalyticTol,sLinePoint,sLineVector);
  if (bIsDeg1Line)
    {
      bSweep = TRUE;
      SmPoint3d  sLinePoint2;
      SmVector3d sLineVector2;
      SmBoolean bIsLineR1 = rRail1Curve.IsLine(11,dAnalyticTol,sLinePoint2,sLineVector2);
      double dAngle;
      if ( bIsLineR1 )
        {
          SER(sLineVector2.AngleBetween(sLineVector,dAngle));
          if (dAngle > SM_EFF_ZERO_SQRT)
            {
              bSweep = FALSE;
            }
        }
      SmBoolean bIsLineR2 = rRail2Curve.IsLine(11,dAnalyticTol,sLinePoint2,sLineVector2);
      if ( bIsLineR2 )
        {
          SER(sLineVector2.AngleBetween(sLineVector,dAngle));
          if (dAngle > SM_EFF_ZERO_SQRT)
            {
              bSweep = FALSE;
            }
        }
    } // end NewSurf == SurfaceOfRevolution check

  // see if NewSurf == SurfaceOfRevolution
  double dArcRadius, dStartAng, dEndAng;
  SmAxis2Placement sRefFrame;
  SmBoolean bIsArc = rCenterLine.IsArc(11, dAnalyticTol,
                                       sRefFrame, dArcRadius,
                                       dStartAng, dEndAng);
  if (bIsArc)
    {
      //cbi this is never hit in prog_test.
      bRevolve = TRUE;
      double dRadius2, dStartAng2, dEndAng2;
      double dRadius3, dStartAng3, dEndAng3;
      SmAxis2Placement sRefFrame2, sRefFrame3;
      SmBoolean bIsArc2 = rRail1Curve.IsArc(11, dAnalyticTol, sRefFrame2,
                                             dRadius2, dStartAng2, dEndAng2);
      if ( !bIsArc2 )
        { bRevolve = FALSE; }
      else if ( smos_Fabs( dEndAng2 - dEndAng ) * dRadius2 > dThisApproxTol3d )
        { bRevolve = FALSE; }
      else if ( ! sRefFrame2.AreCoaxial( sRefFrame, dThisApproxTol3d, dRadius2 ) )
        { bRevolve = FALSE; }

      if ( bRevolve )
      {
          SmBoolean bIsArc3 = rRail2Curve.IsArc(11, dAnalyticTol, sRefFrame3,
                                                 dRadius3, dStartAng3, dEndAng3);
          if ( !bIsArc3 )
            { bRevolve = FALSE; }
          else if ( smos_Fabs( dEndAng3 - dEndAng ) * dRadius3 > dThisApproxTol3d )
            { bRevolve = FALSE; }
          else if ( ! sRefFrame3.AreCoaxial( sRefFrame, dThisApproxTol3d, dRadius3 ) )
            { bRevolve = FALSE; }
          // Also check against 1s Rail curve.
          else if ( smos_Fabs( dEndAng3 - dEndAng2 ) * dRadius3 > dThisApproxTol3d )
            { bRevolve = FALSE; }
          else if ( ! sRefFrame3.AreCoaxial( sRefFrame2, dThisApproxTol3d, dRadius3 ) )
            { bRevolve = FALSE; }
      }
    }

  // track max/min number of knots in cross-section curves
  ULONG lMaxNumUniqueKnots = 0;
  ULONG lMinNumUniqueKnots = 1000000;

  ULONG ii, jj, kkk;

  // for two passes - 1st pass allocate curves
  //                  2nd pass make all curve knot vectors compatible
  for ( kkk=0; kkk<2; kkk++ )
    {
      // for every pRail1Curve->knot
      for ( ii=1; ii<sKnots.GetSize(); ii++ )
        {
          // skip to end cross section curve for sweeps and revolves
          if(   ii > 1
             && (   bSweep
                 || bRevolve))
            { ii = sKnots.GetSize() - 1 ; }

          // locals - center, rail1, and rail2 curve sample points
          SmVector3d sCentPV[2];
          SmPoint3d sP1, sP2;

          // for lNumBetween count locations between knot points
          // ok for lNumBetween == 0
          ULONG lNumBetween = 0;
          for (jj=0; jj<=lNumBetween+1; jj++)
            {
              double dT =   (jj==0) ? sKnots[ii-1]
                          : (jj==lNumBetween+1) ? sKnots[ii]
                          : (sKnots[ii-1] + jj * (sKnots[ii]-sKnots[ii-1]) / (lNumBetween+1.0)) ;

              if (jj==0 && ii != 1) { continue; } // Only do first point first time through
              // Compute the knot index

              ULONG lKnotIndex = (jj==0 && ii == 1)
                                 ? 0
                                 : ii;

              // get center, rail1, rail2 points for dT
              SER(rCenterLine.Evaluate(dT,1,TRUE,sCentPV));
              SER(rRail1Curve.EvaluatePoint(dT,sP1));
              SER(rRail2Curve.EvaluatePoint(dT,sP2));

              // get sample Point gaps
              SmVector3d sV1 = sP1 - sCentPV[0];
              SmVector3d sV2 = sP2 - sCentPV[0];
              SmBSplineCurve * pArc = NULL;

#ifdef SM_DEBUG_CODE
              if (bDebugMe)
                {
                  sm_GraphicsLoop();
                  smgfx_SetLook(1,5, 1,0,1); sCentPV[0].Draw(); sm_GraphicsLoop();
                  smgfx_SetLook(1,5, 0,0,1); sP1.Draw(); sm_GraphicsLoop();
                  smgfx_SetLook(1,5, 0,1,1); sP2.Draw(); sm_GraphicsLoop();
                  smgfx_SetLook(1,5, 1,0,0); sCentPV[0].DrawPointToPoint( sP1 ); sm_GraphicsLoop();
                  smgfx_SetLook(1,5, 0,0,1); sCentPV[0].DrawPointToPoint( sP2 ); sm_GraphicsLoop();
                  sm_GraphicsLoop();
                }
#endif

              // when railCurve samplePoints are close together - make a degenerate cross section curve
              if (sP1.DistanceBetween(sP2) < dThisApproxTol3d)
                {
                  // On the 1st pass
                  if (kkk==0)
                    {
                      // Create Create Degenerate Point curve for each dT value on Rail1
                      SER(SmBSplineCurve::CreatePointCurve(crContext,sP1,pArc));  /* parameterized from 0 to 1 */

                      // track max/min cross section unique knot count
                      ULONG lNumCtrlPts  = pArc->GetNumberOfUniqueKnots();
                      lMaxNumUniqueKnots = smos_Max(lMaxNumUniqueKnots,lNumCtrlPts);
                      lMinNumUniqueKnots = smos_Min(lMinNumUniqueKnots,lNumCtrlPts);

                      // accumulate cross section curve and parameter value
                      sParametersOfCrossSections.Add(dT);
                      sCrossSections.Add(pArc);
                    }
                  else // on the 2nd pass
                    {
                      // Insert knots into degenerate Curve until UniqueKnotCount = MaxNumUniqueKnots
                      pArc               = sCrossSections[lKnotIndex];
                      ULONG lNumKnots    = lMaxNumUniqueKnots;
                      ULONG lNumNewKnots = lNumKnots - 2;
                      if (lNumNewKnots == 0) continue;
                      SmTArray<double> sNewKnots;
                      ULONG mm;
                      for (mm=0; mm<lNumNewKnots; mm++)
                        {
                          double dParam = (mm+1.0) / (lNumNewKnots+1.0);
                          sNewKnots.Add(pArc->GetNaturalInterval().Evaluate(dParam));
                        }
                      SER(pArc->InsertKnots(sNewKnots));
                    }
                  continue;
                } // end sameLength gapVector check

              // else the the railCurve points are far appart.
              // build and add a circular arc to the cross section array.
              double dRadius1 = sV1.Length();
              double dRadius2 = sV2.Length();

              // Check for case of zero radius on interior of surface
              // We do allow it on the ends of the surface - however it
              // may not be allowed in the creation - we'll see.
              if (   dRadius1  < SM_EFF_ZERO || dRadius2 < SM_EFF_ZERO)
                {
                  // When sample point with zero radius is an internal point
                  if (ii > 0 && ii < sKnots.GetSize()-1)
                       { // signal an error
                         SER_MSG(SM_ERR, _T("Zero radius cross-section curve")) ;
                       }
                  else { SER_MSG(SM_ERR, _T("Zero radius cross-section curve")) ; // Someday we will handle this at the ends
                       }
                } // end zero radius check

              // else if the radii differences are large
              else if (smos_Fabs(dRadius1 - dRadius2) > dRadius1 / 10.0)
                {
                  // skip adding a cross section for this parameter value
                  SM_DBG_WARN(_T("skipped adding a circular cross-section to a fillet surface because \n") 
                              _T("  railCurve points are not on a circle centered on the centerCurve point") ) ;

                  // But put something into the arrays (on 1st pass), to keep the indexing correct. [B663]
                  if ( kkk==0 )
                  {
                      sParametersOfCrossSections.Add( dT );
                      sCrossSections.Add( NULL );
                  }

                  continue;
                }

              // get the CCW angle from sV1 to sV2
              double dAngle;
              SER(sV1.Unitize());
              SER(sV2.Unitize());
              SmVector3d sZVec = sV1 * sV2;
              SER(sZVec.Unitize());
              SER(sZVec.CCWAngleBetween(sV1,sV2,dAngle));

              // quit for zero and 180 angles
              if (   smos_Fabs(dAngle) < SM_EFF_ZERO_SQRT          // Degenerate case here
                  || smos_Fabs(dAngle) > SM_PI - SM_EFF_ZERO_SQRT) // Nearly 180 degree case here
                { SER_MSG(SM_ERR, _T("Tangent or Degenerate Angle for cross-section curve")); }

              // negate normal for neg angles and recompute CCW angle
              if (dAngle < 0.0) { sZVec = - sZVec; }
              SER(sZVec.CCWAngleBetween(sV1,sV2,dAngle));
              if (dAngle < SM_EFF_ZERO) { SER(SM_ERR); }  // Can't happen ??

              // make unitNormal coordinate system
              SmVector3d sYVec = sZVec * sV1;
              SmVector3d sXVec = sYVec * sZVec;

              // when asked - make complementary circle
              if (m_bComplement)
                {
                  sYVec = - sYVec;
                  dAngle = 2.0 * SM_PI - dAngle;
                }

              // make coordinate system object
              SER(sXVec.Unitize());
              SER(sYVec.Unitize());
              SmAxis2Placement sPlacement;
              SER(sPlacement.SetCanonical(sCentPV[0],sXVec,sYVec));

              // when approximating arcs
              if (m_bApproximateArcs)
                {
                  // on the 1st pass - make an arc, track the min/max number of Unique knots
                  if (kkk==0)
                    {
                      SER(SmBSplineCurve::ApproximateArc(crContext,
                                                         3, NULL, sPlacement,
                                                         dRadius1, 0.0,
                                                         dAngle*180.0/SM_PI,
                                                         m_dTolPercentRadius,
                                                         pArc));

                      // track max/min number of unique knots
                      ULONG lNumKnots = pArc->GetNumberOfUniqueKnots();
                      lMaxNumUniqueKnots = smos_Max( lMaxNumUniqueKnots, lNumKnots );
                      lMinNumUniqueKnots = smos_Min( lMinNumUniqueKnots, lNumKnots );
                    }
                  // on 2nd pass with equal gapVector radii
                  else if (smos_Fabs(dRadius1 - dRadius2) < dThisApproxTol3d)
                    {
                      // get current arc and uniqueKnot count
                      pArc = sCrossSections[lKnotIndex];
                      if ( pArc == NULL )
                        { continue; }

                      ULONG lNumUniqueKnots = pArc->GetNumberOfUniqueKnots();

                      // when needed - replace pArc with a new Arc with appropriate number of knots
                      if (lNumUniqueKnots != lMaxNumUniqueKnots)
                        {
                          ULONG lMaxCtrlPts = lMaxNumUniqueKnots + 2;
                          SER(SmBSplineCurve::ApproximateArc(crContext,
                                                             3, &lMaxCtrlPts, sPlacement,
                                                             dRadius1, 0.0,
                                                             dAngle*180.0/SM_PI,
                                                             m_dTolPercentRadius,
                                                             pArc));
                          SM_ASSERT(sCrossSections[lKnotIndex] != NULL) ; delete sCrossSections[lKnotIndex] ; sCrossSections[lKnotIndex] = NULL ;
                          sCrossSections[lKnotIndex] = pArc;
                        }
                    }

                  else // on 2nd pass with unequal gapVector radii
                    { // Do need to fix knot vector of blend
                      pArc               = sCrossSections[lKnotIndex];
                      if ( pArc == NULL )
                        { continue; }
                      ULONG lNumKnots    = lMaxNumUniqueKnots;
                      ULONG lNumNewKnots = lNumKnots - 2;
                      if (lNumNewKnots == 0) continue;
                      SmTArray<double> sNewKnots;
                      ULONG mm;
                      for (mm=0; mm<lNumNewKnots; mm++)
                        {
                          double dParam = (mm+1.0) / (lNumNewKnots+1.0);
                          sNewKnots.Add(pArc->GetNaturalInterval().Evaluate(dParam));
                        }
                      SER(pArc->InsertKnots(sNewKnots));
                      continue;
                    }
                } // end m_bApproximateArcs == TRUE check
              else // not approximating arcs
                {
                  // 1st pass build a circular segment
                  if (kkk == 0)
                    { SER(SmBSplineCurve::CreateCircleSegment(crContext, 3, sPlacement,
                                                              dRadius1, 0.0,
                                                              dAngle*180.0/SM_PI,
                                                              SM_CO_QUADRATIC,
                                                              pArc));
                    }
                }

#ifdef SM_DEBUG_CODE
              if (bDebugMe)
                {
                  sm_GraphicsLoop();
                  smgfx_SetLook(2,3, 0,1,1); pArc->DrawWDeriv(pArc->GetNaturalInterval(),0); sm_GraphicsLoop();
                  sm_GraphicsLoop();
                }
#endif
              // on 1st pass - See if we have a non-circle case
              if (kkk == 0 && smos_Fabs(dRadius1 - dRadius2) > 1.0e-8)
                {
                  // Tweak arc or create a blend curve using railPoints and current pArc tangents
                  // to interpolate the railCurve sample points
                  SmTArray<SmPoint3d> sPoints;
                  SmTArray<SmVector3d> sVectors;
                  SmPoint3d sPV[2];

                  // add railCurve1Pos/pArcStartTangent to point/vector arrays
                  SER(pArc->Evaluate(pArc->GetNaturalInterval().GetMin(),1,TRUE,sPV));
                  sPoints.Add(sP1);
                  sVectors.Add(sPV[1]);

                  // add railCurve2Pos/pArcEndTangent to point/vector arrays
                  SER(pArc->Evaluate(pArc->GetNaturalInterval().GetMax(),1,TRUE,sPV));
                  sPoints.Add(sP2);
                  sVectors.Add(sPV[1]);

                  // Here either tweak the arc a little by moving the ends or
                  // create an interpolating curve.
                  if (smos_Fabs(dRadius1 - dRadius2) < dThisApproxTol3d*4.0)
                    {
                      // move the arc endPoints to lie directly on the rail curves
                      // gwc: this step is the construction technique for this function
                      //      it is not an attempt to close an arbitrary gap - don't output debug checks
                      SER(pArc->EditEndPoint(sP1, TRUE,  NULL, SM_EE_CONSTRUCTED_END_POINT, FALSE));
                      SER(pArc->EditEndPoint(sP2, FALSE, NULL, SM_EE_CONSTRUCTED_END_POINT, FALSE));
                    }
                  else // replace the circular arc with an interpolating curve
                    {
                      SM_ASSERT(pArc != NULL) ; delete pArc ; pArc = NULL ;
                      SER(SmBSplineCurve::FairBlendCurveDerivatives(sPoints,sVectors,NULL));
                      SER(SmBSplineCurve::CreateInterpolatingCurve(crContext,SM_CP_UNIFORM,3,
                          3,sPoints,sVectors,NULL,TRUE,pArc));
                    }
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe4 = FALSE;
                  if (bDebugMe4)
                    {
                      //                    smgfx_Erase();
                      smgfx_SetColor(0,0,1);
                      pArc->DrawWDeriv(pArc->GetNaturalInterval(),0);
                      sm_GraphicsLoop();
                      smgfx_SetColor(1,0,0);
                      pArc->DrawPolygon();
                      sm_GraphicsLoop();
                    }
#endif
                } // end need to patch up cross-sections to interpolate rail curves check


              // When asked - mirror the cross section
              if (m_bMirror)
                {
                  SmAxis2Placement sMirror;
                  SmVector3d sMirrorXAxis = sP2 - sP1;
                  SER(sMirrorXAxis.Unitize());
                  SER(sZVec.Unitize());
                  SER(sMirror.SetCanonical(sP1,sMirrorXAxis,sZVec));
                  SmCurve *pMirror = NULL;
                  SER(pArc->CreateMirrorCurve(crContext,sMirror,pMirror));
                  NER(pMirror);
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe2 = FALSE;
                  if (bDebugMe2)
                    {
                      smgfx_Erase();
                      smgfx_SetColor(0,0,1);
                      pArc->DrawWDeriv(pArc->GetNaturalInterval(),0);
                      sm_GraphicsLoop();
                      smgfx_SetColor(1,0,0);
                      smgfx_SetLineWidth(4.0);
                      pMirror->DrawWDeriv(pMirror->GetNaturalInterval(),0);
                      smgfx_SetLineWidth(2.0);
                      sm_GraphicsLoop();
                    }
#endif

                  SM_ASSERT(pArc != NULL) ; delete pArc ; pArc = NULL ;
                  pArc = SM_CAST_PTR(SmBSplineCurve,pMirror); NER(pArc);
                  if (kkk == 1)
                    {
                      sCrossSections[lKnotIndex] = pArc;
                    }

                } // end mirror cross section curve check

              // accumulate the cross section curves
              if (kkk == 0)
                {
                  sParametersOfCrossSections.Add(dT);
                  sCrossSections.Add(pArc);
                }
            } // end iter every between knot cross section curve (currently none)
        } // end iter every pRail1Curve->Knot

      // if (! (m_bApproximateArcs && lMaxNumUniqueKnots != lMinNumUniqueKnots))
      if (   m_bApproximateArcs == FALSE
          || lMaxNumUniqueKnots == lMinNumUniqueKnots)
        {
          // only need 1 pass
          break;
        }
    } // end iter 2 times

  // arrive here when all cross section curves and their params are stored in
  // sCrossSections and sParametersOfCrossSections

  // In case we put blanks in:  (Can't use CompressZeros() on <double> array.)
  for ( ii=0; ii<sCrossSections.GetSize(); ii++ )
  {
      if ( sCrossSections[ii] == NULL )
      {
          sCrossSections.RemoveAt( ii );
          sParametersOfCrossSections.RemoveAt( ii );
          ii--;
      }
  }



  SmBSplineSurface *pFilletSurface = NULL;

  // when surface can be constructed as a ruled surface (linear center and rail curves)
  if (bSweep)
    {
      if ( sCrossSections.GetSize() < 2 )  // [B448]
        { SER( SM_ERR ); }

      SmBSplineCurve *pStart = sCrossSections[0];
      SmBSplineCurve *pEnd   = sCrossSections.GetLast();
      SER(SmBSplineSurface::CreateRuledSurface(crContext, *pStart, *pEnd,
                                               SM_SP_U,pFilletSurface));
      SmExtent1d sIvl = rRail1Curve.GetNaturalInterval();
      SmExtent2d sDomain(SmPoint2d(sIvl.GetMin(),0.0),
                         SmPoint2d(sIvl.GetMax(),1.0));
      SER(pFilletSurface->Reparameterize(sDomain));

    } // end ruled surface case
  else if (bRevolve) // when surface can be constructed as a surfaceOfRevolution
    {
      if ( sCrossSections.GetSize() < 1 )  // [B448]
        { SER( SM_ERR ); }

      SmBSplineCurve *pGenCurve = sCrossSections[0];
      SER(SmBSplineSurface::CreateSurfOfRevolution(crContext, pGenCurve,
                                                   sRefFrame.GetOriginRef(),
                                                   sRefFrame.GetZAxis(),
                                                   dEndAng,
                                                   pFilletSurface));
      SmExtent1d sIvl = rRail1Curve.GetNaturalInterval();
      SmExtent2d sDomain(SmPoint2d(sIvl.GetMin(),0.0),
                         SmPoint2d(sIvl.GetMax(),1.0));
      SER(pFilletSurface->Reparameterize(sDomain));
    }
  else // build a skin surface to interpolate all crossSection curves and option end crossTangent values
    {
      if ( sCrossSections.GetSize() < 2 )  // [B448]
        { SER( SM_ERR ); }

      SmBSplineSurface * pDerivSurfs[2] = { NULL, NULL };

      // when asked for G1 continuity
      if (bNeedG1Continuity)
        {
          SmExtent1d sIvl = rCenterLine.GetNaturalInterval();

          // for NewSurface start and end boundaries
          for (kkk=0; kkk<2; kkk++)
            {
              // We need to provide cross-boundary derivative surface
              // for skinned-surface creation.

              // Create rulled surface using end cross-section
              // curve and the tangent vectors of rails
              SmBSplineSurface * pSurf     = NULL;
              SmBSplineCurve   * pNewCurve = NULL;
              SmBSplineCurve   * pGenCurve = sCrossSections[0];
              double dT = sIvl.GetMin();
              if (kkk == 1)
                {
                  pGenCurve = sCrossSections.GetLast();
                  dT = sIvl.GetMax();
                }

              // get railCurve endTangents
              SmVector3d sPVSt[2], sPVEnd[2];
              SER(rRail1Curve.Evaluate(dT,1,TRUE,sPVSt));
              // gwc:replaced one line to interpolate rail2Curve rather than CenterLine
              SER(rRail2Curve.Evaluate(dT,1,TRUE,sPVEnd));
              // SER(rCenterLine.Evaluate(dT,1,TRUE,sPVEnd));

              // make a curve that sweeps from rail1 to rail2 endTangent values
              SER(pGenCurve->CreateByScaleTransRot(crContext,sPVSt[1],sPVEnd[1],
                                                   0,pNewCurve));
              SmObjDelete sCleanNewCrv(pNewCurve);

              // make a Ruled Surface from the crossTangent Curve
              SER(SmBSplineSurface::CreateRuledSurface(crContext,*pGenCurve,*pNewCurve,SM_SP_U,pSurf));
#ifdef SM_DEBUG_CODE
//                    pSurf->Dump();
#endif
              SmExtent1d sCrvIvl = pGenCurve->GetNaturalInterval();
              SmPoint2d  sUVMin  = SmPoint2d(sCrvIvl.GetMin(),0.0);
              SmPoint2d  sUVMax  = SmPoint2d(sCrvIvl.GetMax(),1.0);
              pSurf->AdjustSTEPUVDomain(SmExtent2d(sUVMin,sUVMax));
#ifdef SM_DEBUG_CODE
//                    pSurf->Dump();
#endif
              // save the crossTangent surface
              pDerivSurfs[kkk] = pSurf;

            } // end iter NewSurface start/ends to build crossTangent surfaces

          // add a new knot in the NewSurface start/end intervals
          // to allow the newSurface to interpolate the computed crossTangent values
          SmTArray<double> sKnts;
          rRail1Curve.GetKnots(sKnts);

          double dStartKnotMid = (sKnts[0] + sKnts[1]) / 2.0;
          rRail1Curve.InsertOneKnot(dStartKnotMid,1);
          rRail2Curve.InsertOneKnot(dStartKnotMid,1);

          double dEndKnotMid = (sKnts[sKnts.GetSize()-2] + sKnts.GetLast()) / 2.0;
          rRail1Curve.InsertOneKnot(dEndKnotMid,1);
          rRail2Curve.InsertOneKnot(dEndKnotMid,1);

        } // end need G1 continuity check

      // Build the Skinned Surface to interpolate crossSections and optional end Cross Tangents
      SER(SmBSplineSurface::CreateSkinnedSurface(crContext,
                                                sCrossSections,
                                                FALSE, SM_SP_U, dThisApproxTol3d,
                                                &rRail1Curve, &rRail2Curve,
                                                FALSE, &sParametersOfCrossSections,
                                                pDerivSurfs,
                                                pFilletSurface));
      // clean up the crossTangent surfaces
      for (kkk=0; kkk<2; kkk++)
        {
          if (pDerivSurfs[kkk])  { delete pDerivSurfs[kkk]; pDerivSurfs[kkk] = NULL ; }
        }


      // We have a case where the rail curves start off in the same direction,
      // because the surfaces are not only tangent at one end but also both locally
      // flat.  This means that the end control points of the first interior row of
      // control points will be identical.  However, the algorithm above creates
      // internal points on that first internal row that are not the same.  We fix this
      // by making them all the same.  This means that all of the u-derivatives at the
      // singularity will be parallel: that's legal and doesn't cause problems. [B597]
      ULONG lSing;  // Looking for UMIN or UMAX, or both.
      pFilletSurface->GetSingularities( NULL, NULL, &lSing );
      if ( lSing & SM_SS_UMIN )
        {
          // ULONG lNumU = pFilletSurface->GetNumberControlPoints( SM_SP_U );
          ULONG lNumV = pFilletSurface->GetNumberControlPoints( SM_SP_V );
          SmPoint3d sPt1, sPt2;
          double dW1, dW2;
          SmZoneTol3d dTol = SmTol::GetZoneTol3d( pFilletSurface );
          ULONG lIdxU = 1;  // First interior row.
          pFilletSurface->GetControlPoint( SM_CP_EUCLIDIAN_RATIONAL, lIdxU,       0, sPt1, dW1 );
          pFilletSurface->GetControlPoint( SM_CP_EUCLIDIAN_RATIONAL, lIdxU, lNumV-1, sPt2, dW2 );
          if ( sPt2.CloserThan( dTol, sPt1 ) )
            {
              sPt1 = ( sPt1 + sPt2 ) / 2;  // Make sure they're exactly the same.
              dW1  = ( dW1  + dW2  ) / 2;
              for ( ii=0; ii<lNumV; ii++ )
                { pFilletSurface->SetControlPoint( SM_CP_EUCLIDIAN_RATIONAL, lIdxU, ii, sPt1, dW1 ); }
            }
        }
      if ( lSing & SM_SS_UMAX )
        {
          ULONG lNumU = pFilletSurface->GetNumberControlPoints( SM_SP_U );
          ULONG lNumV = pFilletSurface->GetNumberControlPoints( SM_SP_V );
          SmPoint3d sPt1, sPt2;
          double dW1, dW2;
          SmZoneTol3d dTol = SmTol::GetZoneTol3d( pFilletSurface );
          ULONG lIdxU = lNumU-2;  // First interior row.
          pFilletSurface->GetControlPoint( SM_CP_EUCLIDIAN_RATIONAL, lIdxU,       0, sPt1, dW1 );
          pFilletSurface->GetControlPoint( SM_CP_EUCLIDIAN_RATIONAL, lIdxU, lNumV-1, sPt2, dW2 );
          if ( sPt2.CloserThan( dTol, sPt1 ) )
            {
              sPt1 = ( sPt1 + sPt2 ) / 2;  // Make sure they're exactly the same.
              dW1  = ( dW1  + dW2  ) / 2;
              for ( ii=0; ii<lNumV; ii++ )
                { pFilletSurface->SetControlPoint( SM_CP_EUCLIDIAN_RATIONAL, lIdxU, ii, sPt1, dW1 ); }
            }
        }

    } // end need to build a skin surface branch

  // set output
  rpFilletSurface = pFilletSurface;

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe2=FALSE;
  if (bDebugMe2)
    {
      smgfx_Erase();
      smgfx_SetLook(1,2, 1,0,0);
      rRail1Curve.DrawWDeriv(rRail1Curve.GetNaturalInterval()); sm_GraphicsLoop();
      rRail2Curve.DrawWDeriv(rRail2Curve.GetNaturalInterval()); sm_GraphicsLoop();
      smgfx_SetLook(1,2, 0,0,1);
      rCenterLine.DrawWDeriv(rCenterLine.GetNaturalInterval()); sm_GraphicsLoop();
      sm_GraphicsLoop();
      pFilletSurface->Dump();
      smgfx_SetLook(1,2, 0,1,1); pFilletSurface->DrawUV(4,4); sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
#endif

  // all done
  return SM_SUCCESS;

} // end SmCircularCrossSectionFSG::CreateSurfaceFromCurves

/*******************************************************************//**
PURPOSE: Create a fillet surface with a blend curve cross section.  This
    is a lofted surface.

NOTES:
***********************************************************************/
SmStatus SmBlendCurveCrossSectionFSG::CreateSurface
  (SmFilletSolver & rFilletSolver,
   SmBSplineCurve * pCenterLine,
   SmBSplineCurve * pRail1Curve,
   SmBSplineCurve * pRail2Curve,
   const SmTArray<SmTsectPnt*> & /*crFilletPoints*/,
   SmSurface *pSurf1,
   SmBSplineCurve *pUV1,
   SmSurface *pSurf2,
   SmBSplineCurve *pUV2,
   SmBSplineSurface *& rpFilletSurface)
{
    double dThisApproxTol3d = rFilletSolver.GetThisApproxTol3d();

    SmBoolean bNeedG1Continuity = FALSE;
    if (rFilletSolver.GetSolverType() == SM_FS_CONST_RADIUS) {
        bNeedG1Continuity = TRUE;
    }
    SER(CreateSurfaceFromCurves(rFilletSolver.GetCreationContext(),
        *pCenterLine,*pRail1Curve,*pRail2Curve,dThisApproxTol3d,
        bNeedG1Continuity,pSurf1,pUV1,pSurf2,pUV2,rpFilletSurface));

    return SM_SUCCESS;

} // end SmBlendCurveCrossSectionFSG::CreateSurface



/*******************************************************************//**
PURPOSE: Create a surface from a centerline curve and two rail curves.
    Users can specify whether G1 condition are required on both ends.
    Typically, in the cases of constant-radius filleting, it is desirable
    to have the cross-boundary derivatives(or tangent-fields) on both ends
    of the fillets be normal to the cross-section planes.

NOTES: All curves should be parameterized in the same direction
     with the same knot vector.
***********************************************************************/
SmStatus SmBlendCurveCrossSectionFSG::CreateSurfaceFromCurves
  (const SmContext & crContext,
   SmBSplineCurve & rCenterLine,
   SmBSplineCurve & rRail1Curve,
   SmBSplineCurve & rRail2Curve,
   double dThisApproxTol3d,
   SmBoolean bNeedG1Continuity,
   SmSurface *pSurf1,
   SmBSplineCurve *pUV1,
   SmSurface *pSurf2,
   SmBSplineCurve *pUV2,
   SmBSplineSurface *& rpFilletSurface)
{

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe=FALSE;
    if (bDebugMe) {
        smgfx_SetColor(0,0,1);
        rCenterLine.DrawWDeriv(rCenterLine.GetNaturalInterval());
        rCenterLine.Dump();
        sm_GraphicsLoop();
        smgfx_SetColor(1,0,0);
        rRail1Curve.DrawWDeriv(rRail1Curve.GetNaturalInterval());
        rRail1Curve.Dump();
        sm_GraphicsLoop();
        smgfx_SetColor(0,1,0);
        rRail2Curve.DrawWDeriv(rRail2Curve.GetNaturalInterval());
        rRail2Curve.Dump();
        sm_GraphicsLoop();
    }
#endif

    SmTArray<double> sBreaks;
    SmTArray<double> sKnots;
    SER(rRail1Curve.GetKnots(sKnots));
    SmTArray<double> sKnots2;
    SER(rRail2Curve.GetKnots(sKnots2));
    if (sKnots2.GetSize() != sKnots.GetSize()) SER(SM_ERR); // Curves not compatible

    SmTArray<SmBSplineCurve*> sCrossSections(sKnots.GetSize()*2);
    SmObjsDelete<SmBSplineCurve*> sCleanCrossSections(&sCrossSections);
    SmTArray<double> sParametersOfCrossSections;

    // Surface creation requires 4 or more cross sections so add
    // some extra knots if needed.
    if (sKnots.GetSize() == 2) {
        double dStep = (sKnots[1]-sKnots[0])/3.0;
        double dNew1 = sKnots[0] + dStep;
        double dNew2 = dNew1 + dStep;
        rRail1Curve.InsertOneKnot(dNew1,1);
        rRail2Curve.InsertOneKnot(dNew1,1);
        rRail1Curve.InsertOneKnot(dNew2,1);
        rRail2Curve.InsertOneKnot(dNew2,1);
        rRail1Curve.GetKnots(sKnots);
    }
    while (sKnots.GetSize() < 4) {
        ULONG lMaxSpan = 0;
        double dMaxSpanValue = 0.0;
        for (ULONG k=1; k<sKnots.GetSize(); k++) {
            double dSpan = sKnots[k]-sKnots[k-1];
            if (dSpan > dMaxSpanValue) {
                lMaxSpan = k;
                dMaxSpanValue = dSpan;
            }
        }

        double dMid = (sKnots[lMaxSpan] + sKnots[lMaxSpan-1])/2.0;
        rRail1Curve.InsertOneKnot(dMid,1);
        rRail2Curve.InsertOneKnot(dMid,1);
        rRail1Curve.GetKnots(sKnots);
    }

    //ULONG lMaxNumUniqueKnots = 0;
    //ULONG lMinNumUniqueKnots = 1000000;

    for (ULONG i=1; i<sKnots.GetSize(); i++) {
        SmVector3d sCentPV[2];
        SmPoint3d sP1, sP2;
        ULONG lNumBetween = 0;
        for (ULONG j=0; j<=lNumBetween+1; j++) {
            double dT = sKnots[i-1] + j * (sKnots[i]-sKnots[i-1]) / (lNumBetween+1.0);
            if (j==0) dT = sKnots[i-1];
            if (j==lNumBetween+1) dT = sKnots[i];
            if (j==0 && i != 1) continue; // Only do first point first time through
            // Compute the knot index

            //ULONG lKnotIndex = i;
            //if (j==0 && i == 1) {
            //    lKnotIndex = 0;
            //}

            SER(rCenterLine.Evaluate(dT,1,TRUE,sCentPV));
            SER(rRail1Curve.EvaluatePoint(dT,sP1));
            SER(rRail2Curve.EvaluatePoint(dT,sP2));
            SmVector3d sV1 = sP1 - sCentPV[0];
            SmVector3d sV2 = sP2 - sCentPV[0];
            SmBSplineCurve * pArc = NULL;
            if (sP1.DistanceBetween(sP2) < dThisApproxTol3d) {
                    // Create a point curve.
                SER(SmBSplineCurve::CreatePointCurve(crContext,sP1,pArc));  /* parameterized from 0 to 1 */
                sParametersOfCrossSections.Add(dT);
                sCrossSections.Add(pArc);
                continue;
            }

            double dRadius = sV1.Length();
            double dRadius2 = sV2.Length();

            // Check for case of zero radius on interior of surface
            // We do allow it on the ends of the surface - however it
            // may not be allowed in the creation - we'll see.
            if (dRadius < SM_EFF_ZERO || dRadius2 < SM_EFF_ZERO) {
                if (i > 0 && i < sKnots.GetSize()-1) {
                    SER(SM_ERR);
                }
                else SER(SM_ERR); // Someday we will handle this at the ends
            }

            double dAngle;
            SER(sV1.Unitize());
            SER(sV2.Unitize());
            SmVector3d sZVec = sV1 * sV2;
            SER(sZVec.Unitize());
            SER(sZVec.CCWAngleBetween(sV1,sV2,dAngle));
            if (smos_Fabs(dAngle) < SM_EFF_ZERO_SQRT) {
                SER(SM_ERR);  // Degenerate case here
            }
            if (smos_Fabs(dAngle) > SM_PI - SM_EFF_ZERO_SQRT) {
                SER(SM_ERR); // Nearly 180 degree case here
            }

            if (dAngle < 0.0) {
                sZVec = - sZVec;
            }
            SER(sZVec.CCWAngleBetween(sV1,sV2,dAngle));
            if (dAngle < SM_EFF_ZERO) SER(SM_ERR); // Can't happen ??

            SmAxis2Placement sPlacement;
            SmVector3d sYVec = sZVec * sV1;
            SmVector3d sXVec = sYVec * sZVec;

            SER(sXVec.Unitize());
            SER(sYVec.Unitize());
            SER(sPlacement.SetCanonical(sCentPV[0],sXVec,sYVec));

            SER(SmBSplineCurve::ApproximateArc(crContext,
                3,NULL,sPlacement,dRadius,0.0,dAngle*180.0/SM_PI,dThisApproxTol3d*100.0,pArc));

#ifdef SM_DEBUG_CODE
            if (bDebugMe) {
                sm_GraphicsLoop();
                smgfx_SetColor(0,1,1);
                pArc->DrawWDeriv(pArc->GetNaturalInterval(),0);
                sm_GraphicsLoop();
            }
#endif

//            double dFactor = 1.0;
//            if (m_lContinuity == 2) {
//                dFactor = 1.0;
//            }
//            if (m_lContinuity == 3) {
//                dFactor = 1.2;
//            }
            SmPoint3d sPV[2];
            SER(pArc->Evaluate(pArc->GetNaturalInterval().GetMin(),1,TRUE,sPV));
            SmTArray<SmPoint3d> sPoints;
            SmTArray<SmVector3d> sVectors;
            sPoints.Add(sP1);
            SER(sPV[1].Unitize());
            sVectors.Add(sPV[1]);
            SmVector3d s3DTan1(sPV[1]);
            SER(pArc->Evaluate(pArc->GetNaturalInterval().GetMax(),1,TRUE,sPV));
            SER(sPV[1].Unitize());
            sPoints.Add(sP2);
            sVectors.Add(sPV[1]);
            SmVector3d s3DTan2(sPV[1]);
            SER(s3DTan2.Unitize());
            s3DTan2 = s3DTan2 * s3DTan1.Length();

            // Create a blend curve instead using tangent vectors of the curve
            SmTArray<SmVector3d> * pHigherOrderVecs = NULL;
            SmTArray<SmVector3d> sHigherOrderVecs;
            if (m_lContinuity > 1 && pUV1 && pUV2) {
                SmPoint3d sUVPnt1, sUVPnt2;
                SER(pUV1->EvaluatePoint(dT,sUVPnt1));
                SER(pUV2->EvaluatePoint(dT,sUVPnt2));
                SmPoint2d sUV1(sUVPnt1.x,sUVPnt1.y);
                SmPoint2d sUV2(sUVPnt2.x,sUVPnt2.y);
                SmVector2d sUVTan1, sUVTan2;
                SER(pSurf1->DropVectors(sUV1,TRUE,TRUE,1,&s3DTan1,&sUVTan1));
//                SER(sUVTan1.Unitize());
                SER(pSurf2->DropVectors(sUV2,TRUE,TRUE,1,&s3DTan2,&sUVTan2));
//                SER(sUVTan2.Unitize());
                SmPoint3d sPnt;
                SmVector3d sDU, sDV, sDUV, sDUU, sDVV, sDUUU, sDVVV;
                if (m_lContinuity == 2) {
                    SmVector3d sG2Vec;
                    SER(pSurf1->ComputeHigherOrderDerivs(sUV1,sUVTan1,sG2Vec));
                    SmPoint3d sPnt1;
                    SER(pSurf1->EvaluatePoint(sUV1,sPnt1));
                    sHigherOrderVecs.Add(sG2Vec);

                    SmVector3d sG2Vec2;
                    SER(pSurf2->ComputeHigherOrderDerivs(sUV2,sUVTan2,sG2Vec2));
                     sHigherOrderVecs.Add(sG2Vec2);

                    SER(pSurf2->EvaluatePoint(sUV2,sPnt));
//                    SER(pSurf2->Evaluate2ndDerivatives(sUV2,TRUE,TRUE,sPnt,sDU,sDV,sDUV,sDUU,sDVV));
//                    SmVector3d sG2Vec2 = sDUU * sUVTan2.x + sDVV * sUVTan2.y;
//                    sHigherOrderVecs.Add(sG2Vec2);
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe7 = FALSE;
            if (bDebugMe7) {
                smgfx_Erase();
                smgfx_SetColor(0,0,1);
                pArc->Draw();
                smgfx_SetColor(0,0,0);
                sVectors[0].Draw(&sPnt1);
                sHigherOrderVecs[0].Draw(&sPnt1);
                sm_GraphicsLoop();
                sHigherOrderVecs[1].Draw(&sPnt);
                sVectors[1].Draw(&sPnt);
                pSurf1->DrawUV(4,4);
                pSurf1->DrawAt(sUV1,2);
                smgfx_SetColor(1,0,0);
                sm_GraphicsLoop();
                pSurf2->DrawUV(10,10);
                pSurf2->DrawAt(sUV2,2);
                smgfx_SetColor(1,0,0);
                pArc->DrawPolygon();
                sm_GraphicsLoop();
            }
#endif
                }
                if (m_lContinuity == 3) {
                    SmVector3d sG2Vec,sG3Vec;
                    SER(pSurf1->ComputeHigherOrderDerivs(sUV1,sUVTan1,sG2Vec,&sG3Vec));
                    SmPoint3d sPnt1;
                    SER(pSurf1->EvaluatePoint(sUV1,sPnt1));
                    sHigherOrderVecs.Add(sG2Vec);

                    SmVector3d sG2Vec2, sG3Vec2;
                    SER(pSurf2->ComputeHigherOrderDerivs(sUV2,sUVTan2,sG2Vec2,&sG3Vec2));
                     sHigherOrderVecs.Add(sG2Vec2);

                    SER(pSurf2->EvaluatePoint(sUV2,sPnt));
//                    sHigherOrderVecs.Add(sG2Vec2);

                    sHigherOrderVecs.Add(sG3Vec);
                    sHigherOrderVecs.Add(sG3Vec2);
                }
                pHigherOrderVecs = &sHigherOrderVecs;
            }

            SM_ASSERT(pArc != NULL) ; delete pArc ; pArc = NULL ;
            SER(SmBSplineCurve::FairBlendCurveDerivatives(sPoints,sVectors,pHigherOrderVecs));
            sVectors[0] = sVectors[0]*m_dBlendScale;
            sVectors[1] = sVectors[1]*m_dBlendScale;
            ULONG lDegree = 3;
            if (pHigherOrderVecs) {
                if (pHigherOrderVecs->GetSize() == 2) {
                    sHigherOrderVecs[0] = sHigherOrderVecs[0]*m_dBlendScale*m_dBlendScale;
                    sHigherOrderVecs[1] = sHigherOrderVecs[1]*m_dBlendScale*m_dBlendScale;
                    lDegree = 5;
                }
                if (pHigherOrderVecs->GetSize() == 4) {
                    sHigherOrderVecs[0] = sHigherOrderVecs[0]*m_dBlendScale*m_dBlendScale;
                    sHigherOrderVecs[1] = sHigherOrderVecs[1]*m_dBlendScale*m_dBlendScale;
                    sHigherOrderVecs[2] = sHigherOrderVecs[2]*m_dBlendScale*m_dBlendScale*m_dBlendScale;
                    sHigherOrderVecs[3] = sHigherOrderVecs[3]*m_dBlendScale*m_dBlendScale*m_dBlendScale;
                    lDegree = 7;
                }
            }
            SER(SmBSplineCurve::CreateInterpolatingCurve(crContext,SM_CP_UNIFORM,3,
                lDegree,sPoints,sVectors,pHigherOrderVecs,FALSE,pArc));

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe4 = FALSE;
            if (bDebugMe4) {
                //                    smgfx_Erase();
                pArc->Dump();
                smgfx_SetColor(0,0,1);
                pArc->DrawWDeriv(pArc->GetNaturalInterval(),0);
                sm_GraphicsLoop();
                smgfx_SetColor(1,0,0);
                pArc->DrawPolygon();
                sm_GraphicsLoop();
            }
#endif
            sParametersOfCrossSections.Add(dT);
            sCrossSections.Add(pArc);
        } // for j
    } // for i


    double dAnalyticTol = 1.0e-8;
    SmPoint3d sLinePoint;
    SmVector3d sLineVector;
    SmBoolean bIsLine = rCenterLine.IsLine(11,dAnalyticTol,sLinePoint,sLineVector);
    if (rCenterLine.GetDegree() != 1 && bIsLine) {
        bIsLine = FALSE;
    }

    SmBoolean bSweep = FALSE;
    if (bIsLine) {
        bSweep = TRUE;
        SmPoint3d sLinePoint2;
        SmVector3d sLineVector2;
        SmBoolean bIsLineR1 = rRail1Curve.IsLine(11,dAnalyticTol,sLinePoint2,sLineVector2);
        double dAngle;
        if ( bIsLineR1 )
          {
            SER(sLineVector2.AngleBetween(sLineVector,dAngle));
            if (dAngle > SM_EFF_ZERO_SQRT) {
                bSweep = FALSE;
            }
        }
        SmBoolean bIsLineR2 = rRail2Curve.IsLine(11,dAnalyticTol,sLinePoint2,sLineVector2);
        if ( bIsLineR2 )
          {
            SER(sLineVector2.AngleBetween(sLineVector,dAngle));
            if (dAngle > SM_EFF_ZERO_SQRT) {
                bSweep = FALSE;
            }
        }
    }

    SmBSplineSurface *pFilletSurface = NULL;

    if (bSweep) {
        SmBSplineCurve *pStart = sCrossSections[0];
        SmBSplineCurve *pEnd = sCrossSections.GetLast();
        SER(SmBSplineSurface::CreateRuledSurface(crContext,*pStart,*pEnd,
            SM_SP_U,pFilletSurface));
        SmExtent1d sIvl = rRail1Curve.GetNaturalInterval();
        SmExtent2d sDomain(SmPoint2d(sIvl.GetMin(),0.0),
            SmPoint2d(sIvl.GetMax(),1.0));
        SER(pFilletSurface->Reparameterize(sDomain));
    }
    else {
        double dRadius,dStartAng,dEndAng;
        SmAxis2Placement sRefFrame;
        SmBoolean bIsArc = rCenterLine.IsArc(11,dAnalyticTol,
            sRefFrame,dRadius,dStartAng,dEndAng);
        SmBoolean bRevolve = FALSE;
        if (bIsArc) {
            bRevolve = TRUE;
            double dRadius2, dStartAng2, dEndAng2;
            SmAxis2Placement sRefFrame2;
            SmBoolean bIsArc2 = rRail1Curve.IsArc(11,dAnalyticTol,sRefFrame2,
                dRadius2,dStartAng2,dEndAng2);
            if (!bIsArc2) {
                bRevolve = FALSE;
            }
            bIsArc2 = rRail2Curve.IsArc(11,dAnalyticTol,sRefFrame2,
                dRadius2,dStartAng2,dEndAng2);
            if (!bIsArc2) {
                bRevolve = FALSE;
            }
        }
        if (bRevolve) {
            SmBSplineCurve *pGenCurve = sCrossSections[0];
            SER(SmBSplineSurface::CreateSurfOfRevolution(crContext,pGenCurve,
                sRefFrame.GetOriginRef(),sRefFrame.GetZAxis(),dEndAng,pFilletSurface));
            SmExtent1d sIvl = rRail1Curve.GetNaturalInterval();
            SmExtent2d sDomain(SmPoint2d(sIvl.GetMin(),0.0),
                SmPoint2d(sIvl.GetMax(),1.0));
            SER(pFilletSurface->Reparameterize(sDomain));
        }
        else {
            SmBSplineSurface * pDerivSurfs[2] = { NULL, NULL };
            if (bNeedG1Continuity) {
                SmExtent1d sIvl = rCenterLine.GetNaturalInterval();
                for (ULONG i=0; i<2; i++) {
                    // We need to provide cross-boundary derivative surface
                    // for skinned-surface creation.
                    // Create rulled surface using end cross-section
                    // curve and the tangent vectors of rails
                    SmBSplineSurface * pSurf = NULL;
                    SmBSplineCurve *pNewCurve = NULL;
                    SmBSplineCurve * pGenCurve = sCrossSections[0];
                    double dT = sIvl.GetMin();
                    if (i == 1) {
                        pGenCurve = sCrossSections.GetLast();
                        dT = sIvl.GetMax();
                    }
                    SmVector3d sPVSt[2], sPVEnd[2];
                    SER(rRail1Curve.Evaluate(dT,1,TRUE,sPVSt));
                    SER(rCenterLine.Evaluate(dT,1,TRUE,sPVEnd));
                    SER(pGenCurve->CreateByScaleTransRot(crContext,sPVSt[1],sPVEnd[1],
                        0,pNewCurve));
                    SmObjDelete sCleanNewCrv(pNewCurve);
                    SER(SmBSplineSurface::CreateRuledSurface(crContext,*pGenCurve,*pNewCurve,SM_SP_U,pSurf));
#ifdef SM_DEBUG_CODE
//                    pSurf->Dump();
#endif
                    SmExtent1d sCrvIvl = pGenCurve->GetNaturalInterval();
                    SmPoint2d sUVMin = SmPoint2d(sCrvIvl.GetMin(),0.0);
                    SmPoint2d sUVMax = SmPoint2d(sCrvIvl.GetMax(),1.0);
                    pSurf->AdjustSTEPUVDomain(SmExtent2d(sUVMin,sUVMax));
#ifdef SM_DEBUG_CODE
//                    pSurf->Dump();
#endif
                    pDerivSurfs[i] = pSurf;
                }
                SmTArray<double> sKnts;
                rRail1Curve.GetKnots(sKnts);
                double dStartKnotMid = (sKnts[0] + sKnts[1]) / 2.0;
                rRail1Curve.InsertOneKnot(dStartKnotMid,1);
                rRail2Curve.InsertOneKnot(dStartKnotMid,1);
                double dEndKnotMid = (sKnts[sKnts.GetSize()-2] + sKnts.GetLast()) / 2.0;
                rRail1Curve.InsertOneKnot(dEndKnotMid,1);
                rRail2Curve.InsertOneKnot(dEndKnotMid,1);
            }
            if (rRail1Curve.IsClosed(rRail1Curve.GetNaturalInterval()) &&
                rRail2Curve.IsClosed(rRail2Curve.GetNaturalInterval()) ) {
                SmCurve *pCopyFirst = NULL ;
                SER(sCrossSections[0]->Copy(crContext,pCopyFirst));
                SM_ASSERT(sCrossSections.GetLast() != NULL) ; delete sCrossSections.GetLast() ;
                sCrossSections.RemoveLast();
                sCrossSections.Add((SmBSplineCurve*)pCopyFirst);
            }
            SER(SmBSplineSurface::CreateSkinnedSurface(crContext,
                sCrossSections,TRUE,SM_SP_U,dThisApproxTol3d,
                &rRail1Curve,&rRail2Curve,FALSE,&sParametersOfCrossSections,
                pDerivSurfs,pFilletSurface));
            for (ULONG ii=0; ii<2; ii++) {
                if (pDerivSurfs[ii]) { delete pDerivSurfs[ii]; pDerivSurfs[ii] = NULL ; }
            }
        }
    }

    rpFilletSurface = pFilletSurface;

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe2=FALSE;
    if (bDebugMe2) {
        smgfx_Erase();
        smgfx_SetColor(1,0,0);
        rRail1Curve.DrawWDeriv(rRail1Curve.GetNaturalInterval());
        rRail2Curve.DrawWDeriv(rRail2Curve.GetNaturalInterval());
        smgfx_SetColor(0,0,1);
        rCenterLine.DrawWDeriv(rCenterLine.GetNaturalInterval());
        sm_GraphicsLoop();
        pFilletSurface->DrawUV(4,4);
        pFilletSurface->Dump();
        sm_GraphicsLoop();
        pSurf1->DrawUV(4,4);
        pSurf2->DrawUV(3,3);
    }
#endif

    return SM_SUCCESS;

} // end SmBlendCurveCrossSectionFSG::CreateSurfaceFromCurves


/*******************************************************************//**
PURPOSE: Constructor for the surface surface fillet solver.

NOTES:
***********************************************************************/
SmSurfaceSurfaceFS::SmSurfaceSurfaceFS
  (const SmContext & crContext,
   double dThisApproxTol3d,
   double dAngleTolerance,
   double dTangencyTolerance,
   double dOffsetRadiusSurface1,
   double dOffsetRadiusSurface2,
   const SmSurface & crSurface1,
   const SmSurface & crSurface2,
   SmBoolean bOrientationSurface1,
   SmBoolean bOrientationSurface2,
   const SmPoint2d & crUVGuessSurface1,
   const SmPoint2d & crUVGuessSurface2)
  : SmConstantRadiusFS(crContext,
                       dThisApproxTol3d,
                       dAngleTolerance,
                       dTangencyTolerance,
                       dOffsetRadiusSurface1,
                       dOffsetRadiusSurface2,
                       crSurface1, 
                       crSurface2,
                       bOrientationSurface1, 
                       bOrientationSurface2),
    m_eTrimType(SM_BT_MINIMAL)

{
  m_vUVGuesses[0] = crUVGuessSurface1;
  m_vUVGuesses[1] = crUVGuessSurface2;


} // end SmSurfaceSurfaceFS::SmSurfaceSurfaceFS constructor

/*******************************************************************//**
PURPOSE: Destructor for the surface surface fillet solver.

NOTES: Offset Surfaces are deleted by SmConstantRadiusFS destructor.
***********************************************************************/
SmSurfaceSurfaceFS::~SmSurfaceSurfaceFS
  ()
{

} // end SmSurfaceSurfaceFS::~SmSurfaceSurfaceFS

// Not referenced anywhere
/*******************************************************************//**
PURPOSE: Make a rail edge for the surface surface fillet solver.

NOTES:
**********************************************************************
static SmStatus MakeRail(SmFilletBrep * pPseudoBrep,
                         SmFilletEdge *& rpNewEdge)
{
    SmFilletEdge * pRail = new (pPseudoBrep) SmFilletEdge(pPseudoBrep);
    NER(pRail); SmObjDelete sClean1(pRail);
    pRail->SetFilletEdgeType(SM_FE_RAIL);
    SmFilletEdgeuse *pNewEU1 = new (pPseudoBrep) SmFilletEdgeuse(pPseudoBrep);
    NER(pNewEU1); SmObjDelete sClean2(pNewEU1);
    SmFilletEdgeuse *pNewEU2 = new (pPseudoBrep) SmFilletEdgeuse(pPseudoBrep);
    NER(pNewEU2); SmObjDelete sClean3(pNewEU2);
    SmFilletVertexuse *pNewVU1 = new (pPseudoBrep) SmFilletVertexuse();
    NER(pNewVU1); SmObjDelete sClean4(pNewVU1);
    pNewVU1->SetProperty(pNewEU1);
    SmFilletVertexuse *pNewVU2 = new (pPseudoBrep) SmFilletVertexuse();
    NER(pNewVU2); SmObjDelete sClean5(pNewVU2);
    SmFilletVertex * pStartVert = new (pPseudoBrep) SmFilletVertex();
    NER(pStartVert); SmObjDelete sClean6(pStartVert);
    SmFilletVertex * pEndVert = new (pPseudoBrep) SmFilletVertex();
    NER(pEndVert); SmObjDelete sClean7(pEndVert);
     // Put VU1 into start vertex list and VU2 into end vertex
    SER(pStartVert->PostInsert(pNewVU1));
    SER(pEndVert->PostInsert(pNewVU2));
    pNewVU2->SetProperty(pNewEU2);
    SER(pRail->PreInsert(pNewEU1));
    SER(pRail->PostInsert(pNewEU2));
    pNewEU1->SetFilletVertexuse(pNewVU1);
    pNewEU2->SetFilletVertexuse(pNewVU2);
    // Clear cleanup objects
    sClean1.Clear(); sClean2.Clear(); sClean3.Clear();
    sClean4.Clear(); sClean5.Clear(); sClean6.Clear();
    sClean7.Clear();

    rpNewEdge = pRail;

    return SM_SUCCESS;
}
*/

/*******************************************************************//**
PURPOSE: Compute the geometry of a surface surface fillet solver by
    rolling a ball along the surfaces (intersecting the offset surfaces)
    and then trimming the results by the trim boundary of the face.

NOTES:
***********************************************************************/
SmStatus SmSurfaceSurfaceFS::CalcFilletGeom()
{
  SmSurface * pSurf1 = GetSurface(0);
  SmSurface * pSurf2 = GetSurface(1);
  SmExtent2d sDomain1 = pSurf1->GetNaturalUVDomain();
  SmExtent2d sDomain2 = pSurf2->GetNaturalUVDomain();

  // Need to solve for start point using guess point
  SmTsectPnt sCurrPnt;
  sCurrPnt.UVPos(0)   = m_vUVGuesses[0];
  sCurrPnt.UVDeriv(0) = SmPoint2d(0.0, 0.0);
  sCurrPnt.UVPos(1)   = m_vUVGuesses[1];
  sCurrPnt.UVDeriv(1) = SmPoint2d(0.0, 0.0);

  // Compute a plane that should intersect fillet edge and use it to solve
  SmPoint3d sPnt1, sPnt2;
  SER(pSurf1->EvaluatePoint(m_vUVGuesses[0],sPnt1));
  SER(pSurf2->EvaluatePoint(m_vUVGuesses[1],sPnt2));
  sCurrPnt.CrvPos() = (sPnt1+sPnt2)/2.0;

  SmVector3d sNorm1, sNorm2;
  SER(pSurf1->EvaluateNormal(m_vUVGuesses[0],TRUE,TRUE,sNorm1));
  SER(pSurf2->EvaluateNormal(m_vUVGuesses[1],TRUE,TRUE,sNorm2));
  sCurrPnt.CrvDeriv() = sNorm1 * sNorm2;

  // Bad guess points.  They must be points where the surfaces tangent planes are not locally parallel
  if (sCurrPnt.CrvDeriv().LengthSquared() < SM_EFF_ZERO_SQ) 
    { return SM_ERR; }

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
  if (bDebugMe) 
    {
      smgfx_SetLook(2,4, 0,0,1); sNorm1.Draw(&sPnt1); sm_GraphicsLoop();
      smgfx_SetLook(2,4, 1,0,0); sNorm2.Draw(&sPnt2); sm_GraphicsLoop();
      smgfx_SetLook(1,2, 0,1,1); pSurf1->DrawUV(4,4); sm_GraphicsLoop();
      smgfx_SetLook(1,2, 1,0,1); pSurf2->DrawUV(6,6); sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

  double     dStep = 0.0001;
  SmBoolean  bFoundGoodPoint, bClipped, bHitBoundary;
  SmTsectPnt sNextPnt;
  SER(StepSolve(sCurrPnt,
                dStep,
                sDomain1,
                sDomain2,
                bFoundGoodPoint, 
                bClipped, 
                bHitBoundary, 
                sNextPnt));

  // If we did not find a good point, try again, then quit.
  if (!bFoundGoodPoint) 
    {
      SmBoolean  bFoundAnswer;
      SmSolution sSolution;
      SmPoint3d  sPnt3, sPnt4;

      for (ULONG ii=0; ii<3; ii++) 
        {
          SER(pSurf2->EvaluatePoint(sCurrPnt.UVPos(1),sPnt4));
          SER(pSurf1->LocalPointSolve(sDomain1,
                                      SM_SO_MINIMIZE,
                                      sPnt4,
                                      sCurrPnt.UVPos(0), 
                                      bFoundAnswer, 
                                      sSolution));
          if (bFoundAnswer) 
            {
              sCurrPnt.UVPos(0) = SmPoint2d(sSolution.m_vStart[0],sSolution.m_vStart[1]);
            }

          SER(pSurf1->EvaluatePoint(sCurrPnt.UVPos(0),sPnt3));
          SER(pSurf2->LocalPointSolve(sDomain2,
                                      SM_SO_MINIMIZE,
                                      sPnt3,
                                      sCurrPnt.UVPos(1), 
                                      bFoundAnswer, 
                                      sSolution));
          if (bFoundAnswer) 
            {
              sCurrPnt.UVPos(1) = SmPoint2d(sSolution.m_vStart[0],sSolution.m_vStart[1]);
            }
        }
      SER(StepSolve(sCurrPnt,
                    dStep,
                    sDomain1,
                    sDomain2,
                    bFoundGoodPoint, 
                    bClipped, 
                    bHitBoundary, 
                    sNextPnt));
      if (!bFoundGoodPoint) 
        {
          return SM_SUCCESS;
        }
    }

  SmTsectPnt * sP1Data[4];
  SmTArray<SmTsectPnt*> sTsectPnts(4,sP1Data);

  // Add start point
  sTsectPnts.Add(&sNextPnt);

  SetupOffsetExtension(pSurf1,sDomain1);
  SetupOffsetExtension(pSurf2,sDomain2);

  if (m_vFilletGeoms.GetSize() == 0) 
    { return SM_SUCCESS; }

  SM_ASSERT(m_vFilletGeoms.GetSize() == 1);
  SmFilletGeom * pFilletGeom = m_vFilletGeoms[0];
  SmFilletBrep * pBrep       = m_pExecutive->GetPseudoBrep();

  // Create Rails
  SmFace * pFace[2];
  pFace[0] = (SmFace*)((SmOffsetSurface*)pSurf1)->GetBaseSurface()->GetFace(); NER(pFace[0]);
  pFace[1] = (SmFace*)((SmOffsetSurface*)pSurf2)->GetBaseSurface()->GetFace(); NER(pFace[1]);

  for (ULONG ii=0; ii<2; ii++) 
    {
      SmFilletEdge * pRail = NULL;
      SER(pFilletGeom->MakeRailEdge(pBrep, ii, pRail));
      pRail->SetOriginalFace(pFace[ii]);
    }

  // Setup FilletIntersector
  SmFilletIntersector sFI(*pSurf1,
                           sDomain1,
                          *pSurf2,
                           sDomain2,
                          *this);

  SmBSplineCurve * p3DCurve = NULL ;
  SmBSplineCurve * pSurfaceUVCurve[2];
  SmTsectCurveType eCurveType;
  double           dDeviation;
  double           dAppTol = GetThisApproxTol3d();
  double           dAngTol = GetThisAngTolRad();

  // Trace the spine and rail curves and create fillet surafce.
  // This puts the results into the FilletGeom in 'this' FilletSolver
  // (and not into the output arguemnts).
  SER(sFI.DoPointIntersection(m_crContext,
                              sTsectPnts, 
                              TRUE, TRUE, NULL, NULL, 
                              SM_CAST_APPROXTOL3D_PTR(&dAppTol), 
                              &dAngTol, 
                              p3DCurve,
                              pSurfaceUVCurve[0], 
                              pSurfaceUVCurve[1], 
                              eCurveType, 
                              dDeviation));

  // Here we need to go backward through the process and derive
  // rail intersection points.
  SmSurface * pFilletSurf = pFilletGeom->GetFilletSurface(); NER(pFilletSurf);
  SmExtent2d  sUVDomain   = pFilletSurf->GetNaturalUVDomain();
  SmPoint2d   sUVMax = sUVDomain.GetMax();
  SmPoint2d   sUVMin = sUVDomain.GetMin();
  double      dV[2];// 'V' parameter of each rail(iso-curve)
  dV[0] = sUVDomain.GetMin().y;
  dV[1] = sUVDomain.GetMax().y;

  // Classify the rail curves relative to the faces.
  for (ULONG ii=0; ii<2; ii++) 
    {
      SmFilletEdge    * pRail   = pFilletGeom->GetRail(ii);
      SmFilletEdgeuse * pPrimEU = (SmFilletEdgeuse*)pRail->GetPrimaryEdgeuse();
      SmFilletEdgeuse * pMateEU = (SmFilletEdgeuse*)pPrimEU->GetMate();

      // Create a new CurveClass object and install it into pRail.
      SmCurve               * pCurve      = pRail->GetCurve();
      SmCurveClassification * pCurveClass = new(m_crContext) SmCurveClassification(pCurve, 
                                                                                   pCurve->GetNaturalInterval(),
                                                                                   pMateEU->GetUVTrimCurve(),
                                                                                   GetThisApproxTol3d());
      pRail->SetCurveClass(pCurveClass);

      // Classify rail curve relative to its owning face.
      SER(pRail->GetOriginalFace()->CurveOnClassify(TRUE,         // in : TRUE =Do expensive PtClassification for Curves not XSecting any Face->Bndrys
                                                    *pCurveClass, // i/o: contains curve to classify, accumulates intervals as they are found.
                                                    TRUE));       // in : TRUE=increase Edge tols for new verts found to be slightly too far from their surfaces

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe2 = FALSE;
      if (bDebugMe2) 
        {
          pCurveClass->Dump();
          pCurveClass->Draw();
          sm_GraphicsLoop();
        }
#endif // SM_DEBUG_CODE

      // Create UV-curve on fillet for this rail: isoparameter.
      SmPoint3d sStartUV( sUVMin.x, dV[ii], 0 );
      SmPoint3d sEndUV  ( sUVMax.x, dV[ii], 0 );

      SmBSplineCurve *pUVCurve = NULL ;
      SER(SmBSplineCurve::CreateLineSegment(m_crContext,
                                            2,
                                            sStartUV,
                                            sEndUV,
                                            pUVCurve));
      NER(pUVCurve);

      SmExtent1d sIvl(sUVMin.x, sUVMax.x);
      SER(pUVCurve->EditParameterization(sIvl));
      pPrimEU->SetUVCurve(pUVCurve);
    }

  // Use the classifications to rebuild the surface and
  // set up the trimming information.
  SER( SplitAndTrimFilletSurface( 0, m_eTrimType ));

  // all done
  return SM_SUCCESS;

} // end SmSurfaceSurfaceFS::CalcFilletGeom

/*******************************************************************//**
PURPOSE: Determine the amount an offset surface can be extended for
         evaluation purposes during the fillet process.

NOTES: Same as base class version except zero extension for periodic surfaces. [B566]
***********************************************************************/
void SmSurfaceSurfaceFS::SetupOffsetExtension
  (const SmSurface * pSurf,    // in : target surface
   SmExtent2d      & rDomain)  // i/o: target surface domain of interest
{
  // get length of extensions
  SmVector2d sSize = m_dSurfaceExtensionFactor * rDomain.GetSize();

  // Do not extend if periodic.
  double dPeriodicExtensionFactor = 0.0;

  if (pSurf->IsPeriodic(rDomain,SM_SP_U))
    {
      //if (!m_bExtendBefore && !m_bExtendAfter)
      sSize.x *= dPeriodicExtensionFactor;
    }
  if (pSurf->IsPeriodic(rDomain,SM_SP_V))
    {
      //if (!m_bExtendBefore && !m_bExtendAfter)
      sSize.y *= dPeriodicExtensionFactor;
    }

  // process the min domain boundaries
  SmSurfParamType eSingDir;
  if (!pSurf->IsSingularity(rDomain.GetMin(),eSingDir))
    {
      // when surface is not singular - extend the min domain boundary
      rDomain.AddPoint2d(rDomain.GetMin() - sSize);
    }
  else if (eSingDir == SM_SP_U)
    {
      // singular in U direction only extend the u direction
      SmPoint2d sNewMin  = rDomain.GetMin();
      sNewMin.x         -= sSize.x;
      rDomain.AddPoint2d(sNewMin);
    }
  else
    { // eSingDir == SM_SP_V
      // singular in V direction only extend the v direction
      SmPoint2d sNewMin = rDomain.GetMin();
      sNewMin.y -= sSize.y;
      rDomain.AddPoint2d(sNewMin);
    }

  // process the max domain boundaries
  if (!pSurf->IsSingularity(rDomain.GetMax(),eSingDir))
    {
      // when surface is not singular - extend the max domain boundary
      rDomain.AddPoint2d(rDomain.GetMax() + sSize);
    }
  else if (eSingDir == SM_SP_U)
    {
      // singular in U direction only extend the u direction
      SmPoint2d sNewMax  = rDomain.GetMax();
      sNewMax.x         += sSize.x;
      rDomain.AddPoint2d(sNewMax);
    }
  else
    { // eSingDir == SM_SP_V
      // singular in V direction only extend the v direction
      SmPoint2d sNewMax = rDomain.GetMax();
      sNewMax.y += sSize.y;
      rDomain.AddPoint2d(sNewMax);
    }

} // end void SmSurfaceSurfaceFS::SetupOffsetExtension

/*******************************************************************//**
PURPOSE: Utility methods.

NOTES:
***********************************************************************/
SmBoolean SmSurfaceSurfaceFS::IsKindOf( SM_TYPE t ) const
{
  return ((SmSurfaceSurfaceFS_TYPE == t) ? TRUE : SmConstantRadiusFS::IsKindOf( (t) ));
}
/*******************************************************************//**
PURPOSE: Dump a SmSurfaceSurfaceFS.

NOTES: Unimplemented.
***********************************************************************/
void SmSurfaceSurfaceFS::Dump() const {}
