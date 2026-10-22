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
#include "SmFeatureExecutive.h"

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

  if( !pBrep || !SmFilletExecutive::IsRadiusSupported( dDistance, pBrep->GetTolerance() ) )
    return SM_ERR_INVALID_INPUT;

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

  if( !pBrep || !SmFilletExecutive::IsRadiusSupported( dRadius, pBrep->GetTolerance() ) )
    return SM_ERR_INVALID_INPUT;

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

// TRUE when pFace is a fillet (its rail edges are tangent to the faces it joins).
static SmBoolean sm_IsFilletFace(SmFace* pFace)
{
    SmBoolean bIsFillet = FALSE;
    ULONG lCrossSection = 0, lSolverType = 0;
    SmSurfParamType eRailDir = SM_SP_U;
    double dRadius = 0, dDist = 0;
    SmTArray<SmEdge*> sRailMin, sRailMax;
    const SmStatus idStat = pFace->IdentifyFillet(SM_ZONE_TOL_3D, 5.0, TRUE, FALSE,  // require tangent rails
                                                  bIsFillet, lCrossSection, lSolverType, eRailDir,
                                                  dRadius, dDist, sRailMin, sRailMax);
    return idStat == SM_SUCCESS && bIsFillet;
}

// The faces sharing an edge with pFace.
static void sm_NeighbourFaces(SmFace* pFace, SmTArray<SmFace*>& rNeighbours)
{
    rNeighbours.ReSet();
    SmTArray<SmEdge*> sEdges;
    pFace->GetEdges(sEdges);
    for (ULONG ii = 0; ii < sEdges.GetSize(); ii++)
    {
        SmTArray<SmFace*> sEdgeFaces;
        sEdges[ii]->GetFaces(sEdgeFaces);
        for (ULONG jj = 0; jj < sEdgeFaces.GetSize(); jj++)
        {
            if (sEdgeFaces[jj] != pFace)
                rNeighbours.AddUnique(sEdgeFaces[jj]);
        }
    }
}

// Remove fillets that meet at rounded corners, one feature per connected group with its corner faces
// (non-planar faces joining only fillets). Works on a copy; pBrep is replaced only if the result is
// valid, and restored from a copy if the replacement fails.
static SmStatus sm_DefeatureFillets(SmBrep* pBrep)
{
    const SmContext& crContext = *pBrep->GetContext();
    SmBrep* pWork = new (crContext) SmBrep(*pBrep, NULL, NULL);
    SmObjDelete sCleanWork(pWork);

    SmTArray<SmFace*> sFaces, sFeatureFaces;
    pWork->GetFaces(sFaces);
    for (ULONG ii = 0; ii < sFaces.GetSize(); ii++)
    {
        if (sm_IsFilletFace(sFaces[ii]))
            sFeatureFaces.Add(sFaces[ii]);
    }
    for (ULONG ii = 0; ii < sFaces.GetSize(); ii++)
    {
        if (sFeatureFaces.IsIn(sFaces[ii]) || sFaces[ii]->GetSurface()->IsPlanar(SM_ZONE_TOL_3D))
            continue;
        SmTArray<SmFace*> sNeighbours;
        sm_NeighbourFaces(sFaces[ii], sNeighbours);
        // A corner joins three or more fillets; a boss side between two is not one.
        SmBoolean bCorner = sNeighbours.GetSize() >= 3;
        for (ULONG jj = 0; jj < sNeighbours.GetSize() && bCorner; jj++)
            bCorner = sFeatureFaces.IsIn(sNeighbours[jj]);
        if (bCorner)
            sFeatureFaces.Add(sFaces[ii]);
    }
    if (sFeatureFaces.GetSize() == 0)
        return SM_ERR;

    // One feature per connected group.
    SmFeatureExecutive sExecutive(crContext, *pWork);
    SmTArray<SmFeature*> sFeatures;
    SmTArray<SmFace*> sGrouped;
    for (ULONG ii = 0; ii < sFeatureFaces.GetSize(); ii++)
    {
        if (sGrouped.IsIn(sFeatureFaces[ii]))
            continue;
        SmTArray<SmFace*> sGroup;
        sGroup.Add(sFeatureFaces[ii]);
        sGrouped.Add(sFeatureFaces[ii]);
        for (ULONG jj = 0; jj < sGroup.GetSize(); jj++)
        {
            SmTArray<SmFace*> sNeighbours;
            sm_NeighbourFaces(sGroup[jj], sNeighbours);
            for (ULONG kk = 0; kk < sNeighbours.GetSize(); kk++)
            {
                if (sFeatureFaces.IsIn(sNeighbours[kk]) && !sGrouped.IsIn(sNeighbours[kk]))
                {
                    sGroup.Add(sNeighbours[kk]);
                    sGrouped.Add(sNeighbours[kk]);
                }
            }
        }
        SmFeature* pFeature = sExecutive.AddFeature(sGroup, SM_RB_EXTEND_SURFACE, SM_RB_FROM_OTHER_BREP);
        if (pFeature == NULL)
            return SM_ERR;
        sFeatures.Add(pFeature);
    }

    // DoDefeature rebuilds pWork in place; the feature breps it returns are ours to delete.
    SmBrep* pBase = NULL;
    SmTArray<SmBrep*> sFeatureBreps;
    SmObjsDelete<SmBrep*> sCleanFeatures(&sFeatureBreps);
    SER(sExecutive.DoDefeature(pBase, &sFeatureBreps));
    for (ULONG ii = 0; ii < sFeatures.GetSize(); ii++)
    {
        if (!sFeatures[ii]->GetBaseWasApplied())
            return SM_ERR;
    }

    // Rejoin the faces the extended surfaces split.
    {
        SmTemporaryChangeValue<SmBoolean> sEnableEdit(pWork->m_bEditingEnabled, TRUE);
        SER(pWork->RemoveTopologicalEdgesAndVertices());
    }
    if (!pWork->AssertValid(NULL, SM_LEVEL_2) || (pBrep->IsManifoldSolid() && !pWork->IsManifoldSolid()))
        return SM_ERR;

    // Keep a copy to restore pBrep (best effort) if replacing it fails.
    SmBrep* pBackup = new (crContext) SmBrep(*pBrep, NULL, NULL);
    SmObjDelete sCleanBackup(pBackup);

    SmTemporaryChangeValue<SmBoolean> sEnableEdit(pBrep->m_bEditingEnabled, TRUE);
    SmStatus eReplaceStat = pBrep->RemoveAllTopology();
    if (eReplaceStat == SM_SUCCESS)
        eReplaceStat = pBrep->MergeBrep(*pWork);
    if (eReplaceStat != SM_SUCCESS && pBrep->RemoveAllTopology() == SM_SUCCESS)
        pBrep->MergeBrep(*pBackup);
    return eReplaceStat;
}

/*******************************************************************//**
PURPOSE --- Remove the Brep's fillet faces (faces with tangent rail edges).

USAGE NOTES --- Fillets meeting at rounded corners are removed with their
    corner faces. Returns SM_ERR if a fillet can't be removed; fillets already
    removed stay removed.
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
        if( sm_IsFilletFace( sFaces[ii] ) )
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
        // Rounded-corner fillets remain: defeature them.
        if( nRemoved < sFilletFaces.GetSize() )
            return sm_DefeatureFillets( pBrep );
    }

    return SM_SUCCESS;
}
