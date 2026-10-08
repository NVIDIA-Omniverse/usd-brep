// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************
FILE NAME: SmFillets.cpp

PURPOSE: 
    Contains popular high level "C" type functions for filletting

GENERAL NOTES: 
    High level functions may assume some input parameters 
    for ease of use. For maximum flexibility, related functions 
    can be found in SmPrimitiveCreation
**********************************************************************/

#include "StdAfx.h"

#include "SmApiFillets.h"
#include "SmApiGeneral.h"
#include <SmApiTypes.h>
#include "SmFilletExecutive.h"
#include "SmFilletStandardSolver.h"

#if SM_DEBUG_CODE
static const SmVector3d s_kBlue(0, 0, 1);
#endif

/*******************************************************************//**
PURPOSE ---   

NOTES --- 

***********************************************************************/
/*******************************************************************//**
PURPOSE --- Fillet all the edges of a brep with a linear cross section.  

USAGE NOTES --- 

***********************************************************************/
SmApiStatus SmApiChamferFillet
(
    SmBrep*  pBrep,                   ///< [in ]: Pointer to brep
    double  dDistance                ///< [in ]: Distance of linear fillet
)
{
#if SM_DEBUG_CODE
  SmBoolean bDebugMe = FALSE;
  if(bDebugMe)
  {
    SmApiDraw( pBrep, NULL, 1, 1 );
  }
#endif

  const SmContext* pContext = SmApiGetOrCreateContext();

  // Creates circular cross section fillet
  SmFilletSolver * pFS = NULL;
  SmLinearCrossSectionFSG sFSGLinear;

  // Define self-intersection handler for edge fillets
  double dStepBackFactor = 1.0;
  SmMakeSurfaceBlendSIH sSIH( dStepBackFactor );

  SmFilletExecutive sFilExec( *pContext, pBrep );
  sFilExec.SetSelfIntersectionHandler( &sSIH );
  sFilExec.SetDoGlobalMerge( TRUE );

  SmTArray<SmEdge*> sEdges;
  pBrep->GetEdges( sEdges );

  for(ULONG i = 0; i < sEdges.GetSize(); i++)
  {
    SmEdge * pE = sEdges[i];

    SmEdgeuse *pEU = pE->GetBlendEdgeuse();
    if(pEU == NULL)
      continue;

    pFS = new (*pContext) SmConstantDistanceFS( *pContext, pBrep->GetTolerance(), 30.0*SM_PI / 180.0, 2.0*SM_PI / 180.0, dDistance, pEU );

    pFS->SetFilletSurfaceGenerator( &sFSGLinear );

    sFilExec.LoadFilletSolver( pFS );
  }

  SER( sFilExec.CreateFilletCorners() );

  SmTArray<SmFilletCorner*> & rCorners = sFilExec.GetFilletCorners();
  for(ULONG j = 0; j < rCorners.GetSize(); j++)
  {
    rCorners[j]->SetBevel( TRUE );
  }

  SER( sFilExec.DoFilleting() );

#ifdef SM_ASSERT_VALID
  SM_ASSERT_VALID( pBrep );
#endif

#if SM_DEBUG_CODE
  if(bDebugMe)
  {
    SmApiDraw( pBrep, &s_kBlue, 1, 0 );
  }
#endif


  return(SM_SUCCESS);

} // End SmChamferFillet


/*******************************************************************//**
PURPOSE --- Fillet all the edges of a brep with circular cross section.  

USAGE NOTES --- 

***********************************************************************/
SmApiStatus SmApiCircularFillet
(
    SmBrep*  pBrep,                   ///< [in ]: Pointer to brep
    double  dRadius                  ///< [in ]: Radius of circular fillet
)
{
#if SM_DEBUG_CODE
  SmBoolean bDebugMe = FALSE;
  if(bDebugMe)
  {
    SmApiDraw( pBrep, NULL, 1, 1 );
  }
#endif

  const SmContext* pContext = SmApiGetOrCreateContext();
  double dTol = pBrep->GetTolerance();

  // Creates circular cross section fillet
  SmCircularCrossSectionFSG sFSGCircular( FALSE, 1.0e-3 );

  // Define self-intersection handler for edge fillets
  double dStepBackFactor = 1.0;
  SmMakeSurfaceBlendSIH sSIH( dStepBackFactor );

  SmFilletExecutive sFilExec( *pContext, pBrep );
  sFilExec.SetSelfIntersectionHandler( &sSIH );
  sFilExec.SetDoGlobalMerge( TRUE );

  SmTArray<SmEdge*> sEdges;
  pBrep->GetEdges( sEdges );

  for(ULONG i = 0; i < sEdges.GetSize(); i++)
  {
    SmEdge * pE = sEdges[i];

    SmEdgeuse *pEU = pE->GetBlendEdgeuse();
    if(pEU == NULL)
      continue;

    SmConstantRadiusFS * pFS = new(*pContext) SmConstantRadiusFS( *pContext, dTol, 30.0*SM_PI / 180.0, 2.0*SM_PI / 180.0, dRadius, pEU );

    pFS->SetFilletSurfaceGenerator( &sFSGCircular );

    sFilExec.LoadFilletSolver( pFS );
  }

  SER( sFilExec.CreateFilletCorners() );

  SER( sFilExec.DoFilleting() );

  // This operation requires stitching for some reason
  // DoFilleting should be examined so that it results in a valid brep
  pBrep->StitchAndOrient();

#ifdef SM_CHECK_VALIDITY
  SM_ASSERT_VALID( pBrep );
#endif

#if SM_DEBUG_CODE
  if(bDebugMe)
  {
    SmApiDraw( pBrep, &s_kBlue, 1, 0 );
  }
#endif


  return(SM_SUCCESS);

} // End SmCircularFillet

/*******************************************************************//**
PURPOSE ---   

USAGE NOTES --- 

***********************************************************************/
SmApiStatus SmApiRemoveFillet
(
    SmBrep*  pBrep                         ///< [in/out]: Pointer to brep                                             <br>
)
{

    if( pBrep == NULL )
        return SM_ERR_INVALID_INPUT;

    SmTArray<SmFace*> sFaces;
    pBrep->GetFaces( sFaces );

    SmTArray<SmFace*> sFilletFaces;
    for( ULONG ii = 0; ii < sFaces.GetSize(); ii++ )
    {
        SmBoolean bIsFillet = FALSE;
        ULONG lCrossSection = 0, lSolverType = 0;
        SmSurfParamType eRailDir = SM_SP_U;
        double dRadius = 0, dDist = 0;
        SmTArray<SmEdge*> sRailMin, sRailMax;

        SmStatus idStat = sFaces[ii]->IdentifyFillet(
            SM_ZONE_TOL_3D, 5.0, FALSE, FALSE,
            bIsFillet, lCrossSection, lSolverType, eRailDir,
            dRadius, dDist, sRailMin, sRailMax );

        if( idStat == SM_SUCCESS && bIsFillet )
            sFilletFaces.Add( sFaces[ii] );
    }

    if( sFilletFaces.GetSize() > 0 )
    {
        SmTemporaryChangeValue<SmBoolean> sEnableEdit( pBrep->m_bEditingEnabled, TRUE );

        ULONG nRemoved = 0;
        for( ULONG ii = 0; ii < sFilletFaces.GetSize(); ii++ )
        {
            SmStatus delStat = pBrep->DeleteFilletFace( sFilletFaces[ii] );
            if( delStat == SM_SUCCESS )
                nRemoved++;
        }

        if( nRemoved > 0 )
        {
            SmStatus stitchStat = pBrep->StitchAndOrient();
            if( stitchStat != SM_SUCCESS )
                return stitchStat;
        }
    }

    return SM_SUCCESS;
}
