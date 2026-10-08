// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*******************************************************************//**
* FILE NAME --- SmFeatureExecutive.cpp
* PURPOSE: Implementation of the SmFeatureExecutive class.
**********************************************************************/

#include "StdAfx.h"

#include "SmFeatureExecutive.h"
#include <SmBrep.h>
#include <SmFace.h>
#include <SmEdge.h>
#include <SmLoop.h>
#include <SmTopologyTraverser.h>
#include <SmMerge.h>
#include <SmGraphicsOutput.h>
#include <SmBSplineSurface.h>


/*******************************************************************//**
PURPOSE: Remove excess faces from patches being built

NOTES: Helper function for building patches
       Delete the faces we don't want in the patch
        A) Delete faces that don't use any edge->curve from crLoopEdges
        B) Delete faces that overlap with rSourceBrep

***********************************************************************/
SmStatus sm_TrimPatches
(
    SmBrep & rPatchBrep,
    const SmTArray<SmEdge*> & crLoopEdges,
    const SmTArray<SmFace*> & crFaces
)
{
    // Locals
    ULONG ii, jj, kk;
    const SmContext * cpContext = rPatchBrep.GetContext();
    SmTArray<SmFace*> sFacesToDelete, sPatchFaces;
    SmZoneTol3d dTol = SmTol::GetZoneTol3d( &rPatchBrep );
    rPatchBrep.GetFaces( sPatchFaces );
    SmTemporaryChangeValue<SmBoolean> sSaveBrep( rPatchBrep.m_bEditingEnabled, TRUE );

    for ( ii = 0; ii < sPatchFaces.GetSize(); ii++ )
    {
        // Loop locals
        SmBoolean bDeleteFace = TRUE;
        SmFace*   pPFace = sPatchFaces[ii];
        SmTArray<SmEdge*> sEdges;
        pPFace->GetEdges( sEdges );

        // Delete faces unless they share an edge with rLoopEdges
        for ( jj = 0; jj < sEdges.GetSize() && bDeleteFace; jj++ )
        {
            SmEdge   *pEdge = sEdges[jj];
            SmCurve  *pCurve = pEdge->GetCurve();
            SmPoint3d sCrvPt;
            pCurve->EvaluatePoint( pEdge->GetInterval().GetMid(), sCrvPt );

            for ( kk = 0; kk < crLoopEdges.GetSize() && bDeleteFace; kk++ )
            {
                SmBoolean bOnCurve = FALSE;
                double    dParam, dDist;
                SmEdge   *pCopyEdge = crLoopEdges[kk];
                SmCurve  *pOrigCurve = pCopyEdge->GetCurve();

                pOrigCurve->DropPoint( pCopyEdge->GetInterval(), sCrvPt, NULL, dTol, NULL, bOnCurve, dParam, dDist, SM_SO_INTERSECT );
                if ( bOnCurve )
                {
                    bDeleteFace = FALSE;
                }
            } // end for each edge->curve in rLoopEdges
        } // end for each edge->curve on this patch face

        if ( bDeleteFace )
        {
            sFacesToDelete.Add( pPFace );
            continue;
        }

        // Check for overlap with faces we've already done
        // After MergeAddedTopology there will be no partial overlap, only full or partial containment
        // So we can classify one point from each surface to test for containment

        // Locals for face in patch
        SmTArray<SmPoint2d> sPUVPts;
        SmTArray<SmPoint3d> sPPts;
        SmPoint2d           sPUVPt;
        SmPoint3d           sPPt;

        // Get one point in this patch face
        if ( SM_SUCCESS == pPFace->GetPointsInFace( 1, sPUVPts, sPPts ) )
        {
            sPPt = sPPts[0];
            sPUVPt = sPUVPts[0];
        }
        else
        {
            sFacesToDelete.Add( pPFace );
            continue;
        }

        // Compare with every face that should be done already
        for ( jj = 0; jj < crFaces.GetSize(); jj++ )
        {
            // Classify point from patch against face in pBrep 
            SmFace               * pThisFace = crFaces[jj];
            SmPointClassification  sPtCC1( dTol, cpContext );
            pThisFace->Point3DClassify( sPPt, dTol, FALSE, sPtCC1, &sPUVPt );

            // If there is containment, delete face
            if ( sPtCC1.GetPointClass() == SM_PC_FACE )
            {
                bDeleteFace = TRUE;
                break;
            }

            // Classify point from face in pBrep against patch face
            SmPoint2d  sThisUVPt;
            SmPoint3d  sThisPt;
            SmPointClassification sPtCC2( dTol, cpContext );

            // Get a point in the face in pBrep
            if ( SM_SUCCESS == pThisFace->GetPointsInFace( 1, sPUVPts, sPPts ) )
            {
                sThisPt = sPPts[0];
                sThisUVPt = sPUVPts[0];
            }
            else
            { continue; }

            pPFace->Point3DClassify( sThisPt, dTol, FALSE, sPtCC2, &sThisUVPt );

            // If there is containment, delete face
            if ( sPtCC2.GetPointClass() == SM_PC_FACE )
            {
                bDeleteFace = TRUE;
                break;
            }
        }

        if ( bDeleteFace )
        { sFacesToDelete.Add( pPFace ); }
    } // end edge->curve check for each patch face

    // Remove the faces
    rPatchBrep.RemoveFaces( sFacesToDelete );

    return SM_SUCCESS;

} // end sm_TrimPatches

/*******************************************************************//**
PURPOSE: 

NOTES:
***********************************************************************/
SmFeatureLoop::~SmFeatureLoop()
{
    SmTArray<SmBrep*> sPatches;
    GetUniquePatches( sPatches );
    SmObjsDelete<SmBrep*> sClean( &sPatches );

} // end SmFeatureLoop::~SmFeatureLoop()

/*******************************************************************//**
PURPOSE: Get the patches. If Base and Feature use same patch, it's only 
          returned once.

NOTES:
***********************************************************************/
void SmFeatureLoop::GetUniquePatches( SmTArray<SmBrep*> & rPatches ) ///< [out]:
{
        rPatches.ReSet();

        // If the same patch was used for both, return once
        if ( m_pBasePatch == m_pFeatPatch )
        {
            rPatches.Add( m_pBasePatch );
            return;
        }

        // Else return the individual patches
        if ( m_pBasePatch != NULL )
        { rPatches.Add( m_pBasePatch ); }
        if(m_pFeatPatch != NULL)
        { rPatches.Add( m_pFeatPatch ); }

        return;
} // end SmFeatureLoop::GetUniquePatches

/*******************************************************************//**
PURPOSE: 

NOTES:
***********************************************************************/
ULONG SmFeatureLoop::GetIndexInFeature() const
{
    ULONG lIdx = 0;
    m_pFeature->m_sFeatureLoops.FindElement( SM_CONST_CAST( SmFeatureLoop*, this ), lIdx );

    return lIdx;
} // end SmFeatureLoop::GetIndexInFeature

/*******************************************************************//**
PURPOSE: 

NOTES:
***********************************************************************/
void SmFeatureLoop::Dump() const
{
    // Pass call along
    Dump( SM_BD_BASE_ONLY );

} // end SmFeatureLoop::Dump

/*******************************************************************//**
PURPOSE: 

NOTES:
***********************************************************************/
void SmFeatureLoop::Dump(SmBrepDumpType eDumpType) const
{
    TCHAR sBuff[SM_TBLOCK_SIZE];

    // header
    ULONG lIdx;
    lIdx = GetIndexInFeature();
    smos_sprintf( sBuff, _T( "\n\nDump of SmFeatureLoop      [ %4lu ] " ), lIdx );
    smos_WriteBuffer( sBuff );
    
    smos_sprintf( sBuff, _T( "\nBase    SmRebuildBrepType: [ %4d ] " ), m_eBaseRBType );
    smos_WriteBuffer( sBuff );
    smos_sprintf( sBuff, _T( "\nFeature SmRebuildBrepType: [ %4d ] " ), m_eFeatRBType );
    smos_WriteBuffer( sBuff );

    smos_sprintf( sBuff, _T( "\nBase    BaseExtDist:       [ %f ] " ), m_dBaseExtDist );
    smos_WriteBuffer( sBuff );
    smos_sprintf( sBuff, _T( "\nFeature FeatureExtDist:    [ %f ] " ), m_dFeatExtDist );
    smos_WriteBuffer( sBuff );

    if ( m_pBasePatch )
    {
        smos_sprintf( sBuff, _T("%s"), _T("\nDumping m_pBasePatch:") );
        smos_WriteBuffer( sBuff );
        m_pBasePatch->Dump(eDumpType);
    }
    else
    {
        smos_sprintf( sBuff, _T("%s"), _T("\nNo m_pBasePatch built.") );
        smos_WriteBuffer( sBuff );
    }

    if (m_pFeatPatch )
    {
        smos_sprintf( sBuff, _T("%s"), _T("\nDumping m_pFeaturePatch:") );
        smos_WriteBuffer( sBuff );
        m_pFeatPatch->Dump(eDumpType);
    }
    else
    {
        smos_sprintf( sBuff, _T("%s"), _T("\nNo m_pFeaturePatch built.") );
        smos_WriteBuffer( sBuff );
    }
} // end SmFeatureLoop::Dump

/*******************************************************************//**
PURPOSE: Draw the Edges on the Feature and the Patch 
         if it exists

NOTES:
***********************************************************************/
SmDisplayList * SmFeatureLoop::Draw
(
    SmBoolean bAddToUIPickList, ///< [in] :
    SmGfxArraySet * pOptGfxSet  ///< [in] : draw parameters from smgfx_RefGlobalDisplayParameters()
) const
{
    // Pass call along 
    if ( m_pFeature->m_bIsPartitioned )
    { return ( DrawFeature( bAddToUIPickList, pOptGfxSet ) ); }
    else
    { return ( DrawBase( bAddToUIPickList, pOptGfxSet ) ); }

} // end SmFeatureLoop::Draw

/*******************************************************************//**
PURPOSE: Draw the FeatureLoop Edges on the base and the BasePatch if it exists

NOTES:
***********************************************************************/
SmDisplayList * SmFeatureLoop::DrawBase
(
    SmBoolean bAddToUIPickList, ///< [in] :
    SmGfxArraySet * pOptGfxSet  ///< [in] : draw parameters from smgfx_RefGlobalDisplayParameters()
) const
{
    SmDisplayList *pRtn = NULL;

#ifdef SM_GFX_CODE
    ULONG ii;

    // start new displayList (unless one is already open)
    SmVector3d sColor = smgfx_GetOutputColor( pOptGfxSet );
    smgfx_Open( sColor, NULL, NULL, FALSE, pOptGfxSet );

    smgfx_SetLook( 1, 2, 0, 0, 0 );
    if ( m_pBasePatch ) m_pBasePatch->Draw( bAddToUIPickList, pOptGfxSet );

    smgfx_SetLook( 3, 4, 1, 0, 0 ); 
    for ( ii =0; ii < m_sBaseEdges.GetSize() ; ii++ )
    { m_sBaseEdges[ii]->Draw( pOptGfxSet ); }

    smgfx_OutputColor( sColor, pOptGfxSet );
    pRtn = smgfx_Close( pOptGfxSet );

#else
  SM_REF2(bAddToUIPickList, pOptGfxSet);
#endif // SM_GFX_CODE
    return ( pRtn );

} // end SmFeatureLoop::DrawBase

/*******************************************************************//**
PURPOSE: Draw the FeatureLoop Edges on the Feature and the FeaturePatch 
         if it exists

NOTES:
***********************************************************************/
SmDisplayList * SmFeatureLoop::DrawFeature
(
    SmBoolean bAddToUIPickList, ///< [in] :
    SmGfxArraySet * pOptGfxSet  ///< [in] : draw parameters from smgfx_RefGlobalDisplayParameters()
) const
{
    SmDisplayList *pRtn = NULL;

#ifdef SM_GFX_CODE
    ULONG ii;

    // start new displayList (unless one is already open)
    SmVector3d sColor = smgfx_GetOutputColor( pOptGfxSet );
    smgfx_Open( sColor, NULL, NULL, FALSE, pOptGfxSet );

    // Draw feature patch
    smgfx_SetLook( 1, 2, 0, 0, 0 );
    if ( m_pFeatPatch ) m_pFeatPatch->Draw( bAddToUIPickList, pOptGfxSet );

    // Draw boundary edges of feature
    smgfx_SetLook( 3, 4, 1, 0, 0 );
    for ( ii =0; ii < m_sFeatEdges.GetSize() ; ii++ )
    { m_sFeatEdges[ii]->Draw( pOptGfxSet ); }

    smgfx_OutputColor( sColor, pOptGfxSet );
    pRtn = smgfx_Close( pOptGfxSet );

#else
  SM_REF2(bAddToUIPickList, pOptGfxSet);
#endif //SM_GFX_CODE

    return ( pRtn );

} // end SmFeatureLoop::DrawFeature

/*******************************************************************//**
PURPOSE: 

NOTES:
***********************************************************************/
SmFeature::SmFeature( SmFeatureExecutive * pFExec)
  :
  m_pFBrep( NULL ),
  m_pFeatureExec( pFExec ),
  m_bIsPartitioned( FALSE ),
  m_bDefeatureReady( FALSE ),
  m_bBaseRebuilt( FALSE ),
  m_bFeatRebuilt( FALSE ),
  m_bBaseWasApplied( FALSE ),
  m_bFeatWasApplied( FALSE ),
  m_bCanRestore( FALSE ),
  m_bJoinedPatches( FALSE ),
  m_bUseBPatchForF( FALSE )
{
    m_pBBrep = m_pFeatureExec->m_pBaseBrep;
} // end SmFeature::SmFeature

/*******************************************************************//**
PURPOSE: 

NOTES:
***********************************************************************/
SmFeature::~SmFeature()
{
    SmObjsDelete<SmFeatureLoop*> sClean( &m_sFeatureLoops );
} // end SmFeature::~SmFeature

/*******************************************************************//**
PURPOSE: Draw the Faces and FeatureLoops of the Feature. 

NOTES: If the Feature hasn't been partitioned the geometry is in the
        Base brep. If the Feature has been partitioned the geometry
        is stand-alone.
***********************************************************************/
SmDisplayList * SmFeature::Draw
(
    SmBoolean bAddToUIPickList, ///< [in] :
    SmGfxArraySet * pOptGfxSet  ///< [in] : draw parameters from smgfx_RefGlobalDisplayParameters()
) const
{
    SmDisplayList *pRtn = NULL;

#ifdef SM_GFX_CODE
    ULONG ii;

    // start new displayList (unless one is already open)
    SmVector3d sColor = smgfx_GetOutputColor( pOptGfxSet );
    smgfx_Open( sColor, NULL, NULL, FALSE, pOptGfxSet );

    for ( ii = 0; ii < m_sFaces.GetSize(); ii++ )
    { m_sFaces[ii]->Draw( pOptGfxSet ); }

    if ( m_bIsPartitioned )
    {
        for ( ii = 0; ii < m_sFeatureLoops.GetSize(); ii++ )
        { m_sFeatureLoops[ii]->DrawFeature( bAddToUIPickList, pOptGfxSet ); }
    }
    else
    {
        for ( ii = 0; ii < m_sFeatureLoops.GetSize(); ii++ )
        { m_sFeatureLoops[ii]->DrawBase( bAddToUIPickList, pOptGfxSet ); }
    }

    smgfx_OutputColor( sColor, pOptGfxSet );
    pRtn = smgfx_Close( pOptGfxSet );

#else
  SM_REF2(bAddToUIPickList, pOptGfxSet);
#endif // SM_GFX_CODE
    return( pRtn );

} // end SmFeature::Draw

/*******************************************************************//**
PURPOSE: 

NOTES:
***********************************************************************/
void SmFeature::Dump() const
{
    // pass the call along
    Dump( SM_BD_BASE_ONLY );

    return;
} // end SmFeature::Dump()

/*******************************************************************//**
PURPOSE: 

NOTES:
***********************************************************************/
void SmFeature::Dump(SmBrepDumpType eBDType) const
{
    TCHAR sBuff[SM_TBLOCK_SIZE];
    ULONG lIdx;

    // header
    lIdx = GetIndexInFeatureExec();
    smos_sprintf( sBuff, _T( "\n\nDump of SmFeature [ %4lu ]" ), lIdx );
    smos_WriteBuffer( sBuff );

    // Report State
    smos_sprintf( sBuff, _T( "\nNumber of Faces in Feature: %lu" ), m_sFaces.GetSize() );
    smos_WriteBuffer( sBuff );

    smos_sprintf( sBuff, _T( "\nNumber of Loops in Feature: %lu" ), m_sFeatureLoops.GetSize() );
    smos_WriteBuffer( sBuff );

    smos_sprintf( sBuff, _T( "\nm_bIsPartitioned  == %u" ), m_bIsPartitioned );
    smos_WriteBuffer( sBuff );

    smos_sprintf( sBuff, _T( "\nm_bDefeatureReady == %u" ), m_bDefeatureReady );
    smos_WriteBuffer( sBuff );

    smos_sprintf( sBuff, _T( "\nm_bBaseRebuilt    == %u" ), m_bBaseRebuilt );
    smos_WriteBuffer( sBuff );

    smos_sprintf( sBuff, _T( "\nm_bFeatRebuilt    == %u" ), m_bFeatRebuilt );
    smos_WriteBuffer( sBuff );

    smos_sprintf( sBuff, _T( "\nm_bBaseWasApplied == %u" ), m_bBaseWasApplied );
    smos_WriteBuffer( sBuff );

    smos_sprintf( sBuff, _T( "\nm_bFeatWasApplied == %u" ), m_bFeatWasApplied );
    smos_WriteBuffer( sBuff );

    smos_sprintf( sBuff, _T( "\nm_bCanRestore     == %u" ), m_bCanRestore );
    smos_WriteBuffer( sBuff );

    smos_sprintf( sBuff, _T( "%s" ) , _T("\nDumping Loops:") );
    smos_WriteBuffer( sBuff );

    ULONG ii;
    for ( ii = 0; ii < m_sFeatureLoops.GetSize(); ii++ )
    { m_sFeatureLoops[ii]->Dump(eBDType); }

    return;
} // end SmFeature::Dump()

/*******************************************************************//**
PURPOSE: Add an SmFeatureLoop to an SmFeature

NOTES:
***********************************************************************/
void SmFeature::AddFeatureLoop
(
    SmFeatureLoop * pFLoop
)
{
    pFLoop->m_pFeature = this;
    m_sFeatureLoops.Add( pFLoop );

} // end SmFeature::AddFeatureLoop

/*******************************************************************//**
PURPOSE: 

NOTES:
***********************************************************************/
ULONG SmFeature::GetIndexInFeatureExec() const
{
    ULONG lIdx = 0;
    m_pFeatureExec->m_sFeatures.FindElement( SM_CONST_CAST( SmFeature*, this ), lIdx );

    return lIdx;
} // end SmFeature::GetIndexInFeatureExec

/*******************************************************************//**
PURPOSE: Set all FLoops base extension distances to dExtDistance

NOTES:
***********************************************************************/
void SmFeature::SetBaseExtDist
(
    double dExtDistance
)
{
    // Locals
    SmTArray<SmFeatureLoop*> sFLoops;
    GetFeatureLoops( sFLoops );
    ULONG ii, lNumFLoops = sFLoops.GetSize();

    for ( ii = 0; ii < lNumFLoops; ii++ )
    { sFLoops[ii]->SetBaseExtDistance( dExtDistance ); }

} // end SmFeature::SetBaseExtDist

/*******************************************************************//**
PURPOSE: Set FLoops base extension distances to dExtDistance

NOTES: Must have m_sFeatureLoops.GetSize() == dExtDistances.GetSize()
***********************************************************************/
SmStatus SmFeature::SetBaseExtDist
(
    SmTArray<double> dExtDistances
)
{
    // Locals
    SmTArray<SmFeatureLoop*> sFLoops;
    GetFeatureLoops( sFLoops );
    ULONG ii, lNumFLoops = sFLoops.GetSize();

    if ( dExtDistances.GetSize() != lNumFLoops )
    { SER_MSG( SM_ERR_INVALID_INPUT, _T( "Need a 1-1 correspondance for FeatureLoops and dExtDistances" ) ); }

    for ( ii = 0; ii < lNumFLoops; ii++ )
    { sFLoops[ii]->SetBaseExtDistance( dExtDistances[ii] ); }

    return SM_SUCCESS;

} // end SmFeature::SetBaseExtDist

/*******************************************************************//**
PURPOSE: Set all FLoops feature extension distances to dExtDistance

NOTES:
***********************************************************************/
void SmFeature::SetFeatureExtDist
(
    double dExtDistance
)
{
    // Locals
    SmTArray<SmFeatureLoop*> sFLoops;
    GetFeatureLoops( sFLoops );
    ULONG ii, lNumFLoops = sFLoops.GetSize();

    for ( ii = 0; ii < lNumFLoops; ii++ )
    { sFLoops[ii]->SetFeatureExtDistance( dExtDistance ); }

} // end SmFeature::SetFeatureExtDist

/*******************************************************************//**
PURPOSE: Set FLoops feature extension distances to dExtDistance

NOTES: Must have m_sFeatureLoops.GetSize() == dExtDistances.GetSize()
***********************************************************************/
SmStatus SmFeature::SetFeatureExtDist
(
    SmTArray<double> dExtDistances
)
{
    // Locals
    SmTArray<SmFeatureLoop*> sFLoops;
    GetFeatureLoops( sFLoops );
    ULONG ii, lNumFLoops = sFLoops.GetSize();

    if ( dExtDistances.GetSize() != lNumFLoops )
    { SER_MSG( SM_ERR_INVALID_INPUT, _T( "Need a 1-1 correspondance for FeatureLoops and dExtDistances" ) ); }

    for ( ii = 0; ii < lNumFLoops; ii++ )
    { sFLoops[ii]->SetFeatureExtDistance( dExtDistances[ii] ); }

    return SM_SUCCESS;

} // end SmFeature::SetFeatureExtDist

/*******************************************************************//**
PURPOSE: Set all FLoops base rebuild types to eRBType

NOTES:
***********************************************************************/
void SmFeature::SetBaseRBType
(
    SmRebuildBrepType eRBType
)
{
    // Locals
    SmTArray<SmFeatureLoop*> sFLoops;
    GetFeatureLoops( sFLoops );
    ULONG ii, lNumFLoops = sFLoops.GetSize();

    for ( ii = 0; ii < lNumFLoops; ii++ )
    { sFLoops[ii]->SetBaseRBType( eRBType ); }

} // end SmFeature::SetBaseRBType

/*******************************************************************//**
PURPOSE: Set all FLoops base rebuild types to eRBType

NOTES: Must have m_sFeatureLoops.GetSize() == sRBTypes.GetSize()
***********************************************************************/
SmStatus SmFeature::SetBaseRBType
(
    SmTArray<SmRebuildBrepType> sRBTypes
)
{
    // Locals
    SmTArray<SmFeatureLoop*> sFLoops;
    GetFeatureLoops( sFLoops );
    ULONG ii, lNumFLoops = sFLoops.GetSize();

    if ( sRBTypes.GetSize() != lNumFLoops )
    { SER_MSG( SM_ERR_INVALID_INPUT, _T( "Need a 1-1 correspondance for FeatureLoops and sRBTypes" ) ); }

    for ( ii = 0; ii < lNumFLoops; ii++ )
    { sFLoops[ii]->SetBaseRBType( sRBTypes[ii] ); }

    return SM_SUCCESS;

} // end SmFeature::SetBaseRBTypes

/*******************************************************************//**
PURPOSE: Set all FLoops feature rebuild types to eRBType

NOTES:
***********************************************************************/
void SmFeature::SetFeatureRBType
(
    SmRebuildBrepType eRBType
)
{
    // Locals
    SmTArray<SmFeatureLoop*> sFLoops;
    GetFeatureLoops( sFLoops );
    ULONG ii, lNumFLoops = sFLoops.GetSize();

    for ( ii = 0; ii < lNumFLoops; ii++ )
    { sFLoops[ii]->SetFeatRBType( eRBType ); }

} // end SmFeature::SetFeatureRBType

/*******************************************************************//**
PURPOSE: Set all FLoops feature rebuild types to eRBType

NOTES: Must have m_sFeatureLoops.GetSize() == sRBTypes.GetSize()
***********************************************************************/
SmStatus SmFeature::SetFeatureRBType
(
    SmTArray<SmRebuildBrepType> sRBTypes
)
{
    // Locals
    SmTArray<SmFeatureLoop*> sFLoops;
    GetFeatureLoops( sFLoops );
    ULONG ii, lNumFLoops = sFLoops.GetSize();

    if ( sRBTypes.GetSize() != lNumFLoops )
    { SER_MSG( SM_ERR_INVALID_INPUT, _T( "Need a 1-1 correspondance for FeatureLoops and sRBTypes" ) ); }

    for ( ii = 0; ii < lNumFLoops; ii++ )
    { sFLoops[ii]->SetFeatRBType( sRBTypes[ii] ); }

    return SM_SUCCESS;

} // end SmFeature::SetFeatureRBTypes

/*******************************************************************//**
PURPOSE: Topologocal traversal to populate m_sFaces with faces and 
    check feature validity (i.e. that the brep is bisected)

NOTES:
***********************************************************************/
SmStatus SmFeature::CollectFaces
( SmFace* pFace )
{
    // Locals
    ULONG ii, jj;
    SmTArray<SmFeatureLoop*> sFLoops = m_sFeatureLoops;
    ULONG lNumLoops = sFLoops.GetSize();

#ifdef SM_DEBUG_CODE
    SmBoolean bDebugMe = FALSE;
    ULONG di, dj;
    if ( bDebugMe )
    {
        smgfx_Erase();
        smgfx_SetLook( 1.0, 1.0, 0, 0, 0 );
        m_pBBrep->Draw(); sm_GraphicsLoop();

        smgfx_SetLook( 3.0, 4.0, 1, 0, 0 );
        for ( di = 0; di < sFLoops.GetSize(); di++ )
        {
            SmFeatureLoop* pFLoop = sFLoops[di];
            SmTArray<SmEdge*> sLoopEdges = pFLoop->m_sBaseEdges;
            for ( dj = 0; dj < sLoopEdges.GetSize(); dj++ )
            {
                sLoopEdges[dj]->Draw(); sm_GraphicsLoop(); 
                sm_GraphicsLoop();
            }
        }
        sm_GraphicsLoop();

        pFace->Draw(); sm_GraphicsLoop();
        sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

    // Mark the loop edges
    SmMarkType eMarkType = m_pFeatureExec->m_eMarkType; 
    for ( ii = 0; ii < lNumLoops; ii++ )
    {
        SmFeatureLoop*    pFLoop = sFLoops[ii];
        SmTArray<SmEdge*> sLoopEdges = pFLoop->m_sBaseEdges;
        for ( jj = 0; jj < sLoopEdges.GetSize(); jj++ )
        {
            sLoopEdges[jj]->Mark(eMarkType);
        }
    }

    // Collect all of the faces that make up the feature
    SmTopologyTraverser sTopoTrav;
    SmTArray<SmFace*> sFaces;
    SER( sTopoTrav.CollectFaces( pFace, sFaces, eMarkType ) );
    SetFeatureFaces( sFaces );

    // Check that the loops provided bisect the BRep.
    for ( ii = 0; ii < lNumLoops; ii++ )
    {
        SmFeatureLoop*    pFLoop = sFLoops[ii];
        SmTArray<SmEdge*> sThisLoopEdges = pFLoop->m_sBaseEdges;
        for ( jj = 0; jj < sThisLoopEdges.GetSize(); jj++ )
        {
            SmEdge *pEdge = sThisLoopEdges[jj];
            SmTArray<SmFace*> sEdgeFaces;
            pEdge->GetFaces( sEdgeFaces );
            AERN_MSG( sEdgeFaces.GetSize() == 2, SM_ERR_ASSERT_FAILURE, _T( "All loop edges must have exactly 2 faces" ) );
            SmFace *pFace0 = sEdgeFaces[0], *pFace1 = sEdgeFaces[1];
            SmBoolean bIsBisection = ( pFace0->IsMarked( eMarkType ) && !pFace1->IsMarked( eMarkType ) ) \
                || ( !pFace0->IsMarked( eMarkType ) && pFace1->IsMarked( eMarkType ) );
            AERN_MSG( bIsBisection, SM_ERR_ASSERT_FAILURE, _T( "Loop does not bisect the brep." ) )
        }
    }

    m_bDefeatureReady = TRUE;

    return SM_SUCCESS;

} // end SmFeature::CollectFaces

/*******************************************************************//**
PURPOSE: Find loop edges on the Feature Brep that correspond to loop 
          edges on the Base

NOTES: 
***********************************************************************/
SmStatus SmFeature::CollectFeatureLoopEdges ()
{
    // Locals
    ULONG ii, jj, kk, lNumEdges = 0;
    SmTArray<SmEdge*> sFeatureEdges;
    m_pFBrep->GetEdges( sFeatureEdges );
    
    for ( ii = 0; ii < m_sFeatureLoops.GetSize(); ii++ )
    {
        SmFeatureLoop*            pFLoop   = m_sFeatureLoops[ii];
        const SmTArray<SmEdge*> &crBLEdges = pFLoop->m_sBaseEdges;
        SmTArray<SmEdge*>       & rFLEdges = pFLoop->m_sFeatEdges;
        
        for ( jj = 0; jj < crBLEdges.GetSize(); jj++ )
        {
            SmEdge  *pBaseEdge  = crBLEdges[jj];
            SmCurve *pBaseCurve = pBaseEdge->GetCurve();

            for ( kk = 0; kk < sFeatureEdges.GetSize(); kk++ )
            {
                SmEdge  *pFeatEdge  = sFeatureEdges[kk];
                SmCurve *pFeatCurve = pFeatEdge->GetCurve();

                if ( *pBaseCurve == *pFeatCurve )
                {
                    rFLEdges.Add( pFeatEdge );
                    sFeatureEdges.RemoveAt( kk );

                    lNumEdges++;
                    break;
                }
            }
        }
        AERN( rFLEdges.GetSize() == crBLEdges.GetSize() , SM_ERR_ASSERT_FAILURE);
    }

    return SM_SUCCESS;
} // end SmFeature::CollectFeatureLoopEdges()

/*******************************************************************//**
PURPOSE: Partition the feature from the base brep

NOTES: Topology from BaseBrep is copied to FeatureBrep
***********************************************************************/
SmStatus SmFeature::PartitionFeature()
{
    // No work, already partitioned
    if ( m_bIsPartitioned )
    { return SM_SUCCESS; }

    AERN_MSG( m_bDefeatureReady, SM_ERR_ASSERT_FAILURE, _T( "Cannot partition Feature from Base" ) );

    SmBrep *&prFeature = m_pFBrep;
    SmTArray<SmFace*> &rFaces = m_sFaces;

    prFeature = new ( m_pFeatureExec->m_crContext ) SmBrep();

    SmTemporaryChangeValue<SmBoolean> saveBaseBrep( m_pBBrep->m_bEditingEnabled, TRUE );
    SmTemporaryChangeValue<SmBoolean> saveFeatBrep( ( prFeature )->m_bEditingEnabled, TRUE );

    // Copy faces from BaseBrep to FeatureBrep
    SER( m_pBBrep->CopyFaces( rFaces, prFeature ) );

    // Collect the loop edges on the Feature Brep
    SE( CollectFeatureLoopEdges() );

    // Remove the faces from BaseBrep
    SER( m_pBBrep->RemoveFaces( rFaces ) );

    // Update rFeature Faces
    rFaces.ReSet();
    prFeature->GetFaces( rFaces );

    m_bIsPartitioned = TRUE;
    m_bCanRestore = TRUE;

    return SM_SUCCESS;
} // end SmFeature::PartitionFeature

/*******************************************************************//**
PURPOSE: Build patches for each FeatureLoop for the base and feature breps 

NOTES: 
***********************************************************************/
SmStatus SmFeature::BuildPatches ()
{
    // Locals 
    ULONG ii, lNumLoops;
    m_bBaseRebuilt = TRUE;
    m_bFeatRebuilt = TRUE;

    // Feature must have been partitioned
    AERN_MSG( m_bIsPartitioned, SM_ERR_ASSERT_FAILURE, _T( "Trying to build patch for feature that wasn't partitioned. Continuing to next feature." ) );

    SmTArray<SmFeatureLoop*> sFLoops;  
    GetFeatureLoops(sFLoops);
    lNumLoops = sFLoops.GetSize();

    // Rebuild for the base
    for ( ii = 0; ii < lNumLoops; ii++ )
    {
        SmFeatureLoop* pFLoop = sFLoops[ii];
        SmBrep *pBasePatch = pFLoop->m_pBasePatch;

        // Skip loops that already have patches
        SmStatus eStat = SM_SUCCESS;
        if ( pBasePatch == NULL )
        { eStat = BuildPatch( TRUE, *pFLoop ); }
        if ( eStat != SM_SUCCESS )
        {
            m_bBaseRebuilt = FALSE;
            pFLoop->m_bUseBPatch = FALSE;
        }
        else
        { pFLoop->m_bUseBPatch = TRUE; }
    }

    // Try joining loop patches (fillets run through here, since 2 loops make 1 patch)
    if ( !m_bBaseRebuilt && lNumLoops > 1 )
    { JoinPatches( TRUE ); }

    // Check if we are using the base patch for the feature
    m_bUseBPatchForF = TRUE;
    for ( ii = 0; ii < lNumLoops && m_bUseBPatchForF; ii++ )
    {
        SmFeatureLoop* pFLoop = sFLoops[ii];
        if ( pFLoop->m_eFeatRBType == SM_RB_FROM_OTHER_BREP )
        { continue; }
        else
        { m_bUseBPatchForF = FALSE; }
    }

    // 
    if ( m_bJoinedPatches && m_bUseBPatchForF )
    {
        SmFeatureLoop* pFLoop = sFLoops[0];
        SmTArray<SmEdge*> sEdges;

        // Get all edges from feature
        for ( ii = 0; ii < lNumLoops; ii++ )
        { sEdges.Append( sFLoops[ii]->m_sFeatEdges ); }

        // Use merged patch if it will work for feature
        if ( m_pFeatureExec->ValidatePatch( *m_pFBrep, *pFLoop->m_pBasePatch, sEdges ) == SM_SUCCESS )
        { pFLoop->m_pFeatPatch = pFLoop->m_pBasePatch; }
        else
        { SE_MSG( SM_ERR, _T( "Patch does not fill Feature void manifold-ly" ) ); }
    }
    else
    {
        // Rebuild for the feature
        for ( ii = 0; ii < lNumLoops; ii++ )
        {
            SmFeatureLoop* pFLoop = sFLoops[ii];
            SmBrep *pFeatPatch = pFLoop->m_pFeatPatch;

            // Skip loops that already have patches
            SmStatus eStat = SM_SUCCESS;
            if ( pFeatPatch == NULL )
            { eStat = BuildPatch( FALSE, *pFLoop ); }
            if ( eStat != SM_SUCCESS )
            {
                m_bFeatRebuilt = FALSE;
                pFLoop->m_bUseFPatch = FALSE;
            }
            else
            { pFLoop->m_bUseFPatch = TRUE; }
        }

        if ( !m_bFeatRebuilt && lNumLoops > 1 )
        { JoinPatches( FALSE ); }
    }

    return SM_SUCCESS;
} // end SmFeature::BuildPatches

/*******************************************************************//**
PURPOSE: Build a patch for a FeatureLoop on for either the base or feature brep

NOTES: 
***********************************************************************/
SmStatus SmFeature::BuildPatch
(
    SmBoolean bBaseBrep,
    SmFeatureLoop & rFLoop
)
{
    // Locals
    ULONG ii;
    SmStatus eStat;
    SmFeatureExecutive *pFExec = m_pFeatureExec;
    SmBrep *& prFBrep = m_pFBrep;
    SmTArray<SmEdge*> sBaseEdges = rFLoop.m_sBaseEdges;
    SmTArray<SmEdge*> sFeatureEdges = rFLoop.m_sFeatEdges;
    double dBaseExtDist    = rFLoop.m_dBaseExtDist;
    double dFeatureExtDist = rFLoop.m_dFeatExtDist;

    // set the rebuild method
    SmRebuildBrepType eRBBaseType = rFLoop.m_eBaseRBType;
    SmRebuildBrepType eRBFeatType = rFLoop.m_eFeatRBType;
    SmRebuildBrepType eRebuildType = ( bBaseBrep ) ? eRBBaseType : eRBFeatType;

    // Confirm we have bisected the brep
    AERN_MSG( m_bIsPartitioned, SM_ERR_ASSERT_FAILURE, _T("Cannot rebuild the brep before partitioning") );

    // Get the pointer to the brep we are operating on
    SmBrep  * pTargetBrep      = (bBaseBrep) ? m_pBBrep : prFBrep;

    SmBoolean bBaseIsSource    =  bBaseBrep  ^ ( eRebuildType == SM_RB_FROM_OTHER_BREP );
    SmBrep  * pSourceBrep      = (bBaseIsSource) ? m_pBBrep     : prFBrep;
    double    dExtDist         = (bBaseIsSource) ? dBaseExtDist : dFeatureExtDist;
    SmTArray<SmEdge*> sOpEdges = (bBaseIsSource) ? sBaseEdges   : sFeatureEdges;

    // Create the new set of objects to use in rebuild
    SmBrep *pPatch = NULL;
    //SmBrep *pTBrep = new ( pFExec->m_crContext ) SmBrep( *pTargetBrep );
    //SmTemporaryChangeValue<SmBoolean> saveTBrep( pTBrep->m_bEditingEnabled, TRUE );
    SmObjDelete sCleanPBrep;  //sCleanTBrep( pTBrep )

    SmBrep *& prBasePatch    = rFLoop.m_pBasePatch;
    SmBrep *& prFeaturePatch = rFLoop.m_pFeatPatch;

    // No work
    if ( eRebuildType == SM_RB_NONE )
    {
        // Save empty patch
        pPatch = new ( pFExec->m_crContext ) SmBrep();
        SmBrep *&prSavePatchB = ( bBaseBrep ) ? prBasePatch : prFeaturePatch;
        prSavePatchB = pPatch;

        return SM_SUCCESS;
    }

    // Translate from the LoopEdges to the edges in the SourceBrep
    SmTArray<ULONG> sEdgeIDs;
    for ( ii = 0 ; ii < sOpEdges.GetSize() ; ii++ )
    {
        ULONG lIndex = sOpEdges[ii]->GetEdgeNumberInBrep();
        sEdgeIDs.Add( lIndex );
    }

    SmTArray<SmEdge*> sEdges, sAllEdges;
    pSourceBrep->GetEdges( sAllEdges );
    for ( ii = 0 ; ii < sEdgeIDs.GetSize() ; ii++ )
    { sEdges.Add( sAllEdges[sEdgeIDs[ii]] ); }

    // Have we already built and cached this patch?
    SmBoolean bExtRebuild = ( eRebuildType == SM_RB_FROM_OTHER_BREP || eRebuildType == SM_RB_EXTEND_SURFACE );
    SmBoolean bUseBPatch  =  bBaseIsSource && prBasePatch    != NULL && bExtRebuild;
    SmBoolean bUseFPatch  = !bBaseIsSource && prFeaturePatch != NULL && bExtRebuild;

    // If the patch we want has already been built.
    if ( ( bUseBPatch || bUseFPatch ) && m_bUseBPatchForF )
    {
        pPatch = bUseBPatch ? prBasePatch : prFeaturePatch;
    }
    else if ( ( bUseBPatch || bUseFPatch ) && !m_bUseBPatchForF )
    {
        SmBrep * pPatchToCopy = bUseBPatch ? prBasePatch : prFeaturePatch;
        pPatch = new ( pFExec->m_crContext ) SmBrep( *pPatchToCopy );
    }
    else if ( bExtRebuild && rFLoop.m_bTriedExtSurf )
    {   // We already tried this and it didn't work.
        SER_MSG( SM_ERR, _T( "Trying to rebuild with extended surfaces, known to fail. Skipping" ) );
    }
    else  // Attempt to build the patch
    {
        rFLoop.m_bTriedExtSurf |= bExtRebuild;
        SmZoneTol3d dTol = SmTol::GetZoneTol3d( pTargetBrep );
        SmTArray<SmFace*> & rFaces = rFLoop.m_sBSourceFaces;
        pPatch = new ( pFExec->m_crContext ) SmBrep();
        pPatch->SetTolerance( dTol );

        SmTemporaryChangeValue<SmBoolean> savePBrep( pPatch->m_bEditingEnabled, TRUE );
        sCleanPBrep.SetObj( pPatch );

        SER_MSG( pFExec->RebuildBrep( pPatch, sEdges, eRebuildType, dExtDist, pSourceBrep, rFaces ), _T("Could not build patch for void") );
    }

    // Check that the patch successfully fills the void
    SE_MSG( eStat = pFExec->ValidatePatch( *pTargetBrep, *pPatch, sEdges ), _T("Patch does not fill void manifold-ly. Will try merging patches if available.") );
    
    // Save patch
    SmBrep *&prSavePatchB = ( bBaseBrep ) ? prBasePatch : prFeaturePatch;
    prSavePatchB = pPatch;
    sCleanPBrep.Clear();

#ifdef SM_DEBUG_CODE
    SmBoolean bDebugMe = FALSE;
    if ( bDebugMe )
    {
        smgfx_Erase();
        pPatch->Draw( TRUE ); sm_GraphicsLoop();
        sm_GraphicsLoop();

        smgfx_Erase();
        pTargetBrep->Draw( TRUE ); sm_GraphicsLoop();
        sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

    return eStat;

} // end SmFeature::BuildPatch


/*******************************************************************//**
PURPOSE: 

NOTES: 
***********************************************************************/
SmStatus SmFeature::ApplyPatches ()
{
    // Locals 
    ULONG ii, lNumLoops;
    
    SmTArray<SmFeatureLoop*> sFLoops;
    GetFeatureLoops( sFLoops );

    lNumLoops = sFLoops.GetSize();

    if ( m_bBaseRebuilt && !m_bBaseWasApplied )
    {
        for ( ii = 0; ii < lNumLoops; ii++ )
        {
            SmFeatureLoop* pFLoop = sFLoops[ii];
            SmBrep *pBasePatch = pFLoop->m_pBasePatch;
            if ( pBasePatch == NULL || !pFLoop->m_bUseBPatch)
            { continue; }

            SmTArray<SmFace*> sFaces;
            pBasePatch->GetFaces( sFaces );
            if ( sFaces.GetSize() > 0 )
            { SE( pBasePatch->CopyFaces( sFaces, m_pBBrep ) ); }
        }

        m_pBBrep->StitchAndOrient();
        m_bCanRestore = FALSE;
        m_bBaseWasApplied = TRUE;
    }

    if ( m_bFeatRebuilt && !m_bFeatWasApplied )
    {
        SmBrep *pFeatBrep = m_pFBrep;

        for ( ii = 0; ii < lNumLoops; ii++ )
        {
            SmFeatureLoop* pFLoop = sFLoops[ii];
            SmBrep *pFeatPatch = pFLoop->m_pFeatPatch;
            if (pFeatPatch == NULL || !pFLoop->m_bUseFPatch)
            { continue; }

            SmTArray<SmFace*> sFaces;
            pFeatPatch->GetFaces( sFaces );
            if ( sFaces.GetSize() > 0 )
            { SE( pFeatPatch->CopyFaces( sFaces, pFeatBrep ) ); }
        }

        pFeatBrep->StitchAndOrient();
        m_bCanRestore = FALSE;
        m_bFeatWasApplied = TRUE;
    }

    return SM_SUCCESS;

} // end SmFeature::ApplyPatches

/*******************************************************************//**
PURPOSE: Used after DoDefeature to restore features that were 
    partitioned but not rebuilt

NOTES: Features that use SmRebuildBrepType SM_RB_NONE are considered
    to have been rebuilt

    Deletes the feature brep.
***********************************************************************/
SmStatus SmFeature::RestoreFeature ()
{
    if ( m_bCanRestore )
    {
        SmBrep * pFBrep = m_pFBrep;

        SmTArray<SmFace*> sFaces;
        pFBrep->GetFaces( sFaces );
        SE( pFBrep->CopyFaces( sFaces, m_pBBrep ) );

        m_pBBrep->StitchAndOrient();

        delete pFBrep; m_pFBrep = NULL;

        m_bIsPartitioned = FALSE;
        m_bDefeatureReady = FALSE;
        m_bCanRestore = FALSE;
        m_bBaseWasApplied = TRUE;
        m_bFeatWasApplied = TRUE;

        return SM_SUCCESS;
    }

    return SM_ERR;

} //end SmFeature::RestoreFeature()

/*******************************************************************//**
PURPOSE: Join the invalid patches to make a single patch

NOTES: In some cases, multiple SmFeatureLoops make a single patch,
    e.g. as in fillets.
***********************************************************************/
SmStatus SmFeature::JoinPatches
( SmBoolean bBaseLoops ) ///< [in] : TRUE  = Join loops on base     <br>
                         ///< [in] : FALSE = Join loops on feature  <br>
{
    // Locals
    ULONG ii;
    SmFeatureLoop * pFLoop = m_sFeatureLoops[0];
    SmTArray<SmEdge*> sLoopEdges;
    SmTArray<SmFace*> sFaces;
    const SmContext * cpContext = m_pBBrep->GetContext();
    SmBrep * pPatch = new(*cpContext) SmBrep();

    for ( ii = 0; ii < m_sFeatureLoops.GetSize(); ii++ )
    {
        // Loop locals
        SmStatus        eStat;
        SmFeatureLoop * pThisFLoop = m_sFeatureLoops[ii];
        SmBrep        * pThisPatch = bBaseLoops ? pThisFLoop->m_pBasePatch : pThisFLoop->m_pFeatPatch;
        SmBoolean     & rbUsePatch = bBaseLoops ? pThisFLoop->m_bUseBPatch : pThisFLoop->m_bUseFPatch;
        rbUsePatch = FALSE;

        // Skip missing patches
        if ( pThisPatch == NULL )
        { continue; }
        SmZoneTol3d dTol = SmTol::GetZoneTol3d( pThisPatch );

        // Merge the invalid patches together
        SmMerge sMerge( *cpContext, pPatch, pThisPatch, dTol );
        SE( eStat = sMerge.PiecewiseMerge( NULL, FALSE, TRUE, pPatch ) );
        if ( eStat != SM_SUCCESS )
        { continue; }

        // Save edges
        SmTArray<SmEdge*> sEdges = bBaseLoops ? pThisFLoop->m_sBaseEdges : pThisFLoop->m_sFeatEdges;
        sLoopEdges.Append( sEdges );
        sFaces.AppendUnique( pThisFLoop->m_sBSourceFaces );
    }

    sm_TrimPatches( *pPatch, sLoopEdges, sFaces );

    // Get the pointer to the brep we are operating on
    SmBrep *pTBrep = (bBaseLoops) ? m_pBBrep : m_pFBrep;
    SmFeatureExecutive *pFExec = m_pFeatureExec;
    SmStatus eStat = SM_ERR;

    // Check that the patch successfully fills the void, save if good result
    SE_MSG( eStat = pFExec->ValidatePatch( *pTBrep, *pPatch, sLoopEdges ), _T("Patch does not fill void manifold-ly") );
    if ( eStat == SM_SUCCESS )
    {
        SmBrep   *& prResult   = bBaseLoops ? pFLoop->m_pBasePatch : pFLoop->m_pFeatPatch;
        SmBoolean & rbRebuilt  = bBaseLoops ? m_bBaseRebuilt       : m_bFeatRebuilt;
        SmBoolean & rbUsePatch = bBaseLoops ? pFLoop->m_bUseBPatch : pFLoop->m_bUseFPatch;
        SmObjDelete sClean( prResult );
        sClean.DeleteObj();

        prResult   = pPatch;
        rbRebuilt  = TRUE;
        rbUsePatch = TRUE;
        m_bJoinedPatches = TRUE;
    }

    return eStat;

} //end SmFeature::JoinLoops()

/*******************************************************************//**
PURPOSE: Construct the SmFeatureExecutive object

NOTES:
  The constructor verifies inputs. 
***********************************************************************/
SmFeatureExecutive::SmFeatureExecutive( const SmContext & crContext,
                                              SmBrep    & rOriginalBrep )
    :
    m_crContext( crContext ),
    m_pBaseBrep( &rOriginalBrep )
{
    // increment and lock an unlocked mark
    SmNewMarkAndLock sMarkLock( &m_crContext, SM_MT_ALLMARKS ) ; // increment and lock any unlocked mark
    m_eMarkType = sMarkLock.GetMarkType() ;

} // end SmFeatureExecutive::SmFeatureExecutive

/*******************************************************************//**
PURPOSE: 

NOTES:
***********************************************************************/
SmFeatureExecutive::~SmFeatureExecutive()
{
    SmObjsDelete<SmFeature*> sClean( &m_sFeatures );

} // end SmFeatureExecutive::~SmFeatureExecutive

/*******************************************************************//**
PURPOSE: Get a brep of each feature

NOTES: Entry is null if the Feature hasn't been partitioned
***********************************************************************/
void SmFeatureExecutive::GetFeatureBreps
(
    SmTArray<SmBrep*> & rFBreps  ///< [out]:
)
{
    // prepare for output
    rFBreps.ReSet();

    // locals
    ULONG ii;

    // collect the feature breps
    for ( ii = 0; ii < m_sFeatures.GetSize(); ii++ )
    { rFBreps.Add( m_sFeatures[ii]->m_pFBrep ); }

} // end SmFeatureExecutive::GetFeatureBreps

/*******************************************************************//**
PURPOSE: Set value for every SmFeatureLoop 

NOTES:
***********************************************************************/
void SmFeatureExecutive::SetBaseExtDist
(
    double dExtDistance
)
{
    // locals
    ULONG ii;

    // collect the feature breps
    for ( ii = 0; ii < m_sFeatures.GetSize(); ii++ )
    { m_sFeatures[ii]->SetBaseExtDist( dExtDistance ); }

} // end SmFeatureExecutive::SetBaseExtDist

/*******************************************************************//**
PURPOSE: Set value for every SmFeatureLoop 

NOTES:
***********************************************************************/
void SmFeatureExecutive::SetFeatureExtDist
(
    double dExtDistance
)
{
    // locals
    ULONG ii;

    // collect the feature breps
    for ( ii = 0; ii < m_sFeatures.GetSize(); ii++ )
    { m_sFeatures[ii]->SetFeatureExtDist( dExtDistance ); }

} // end SmFeatureExecutive::SetFeatureExtDist

/*******************************************************************//**
PURPOSE: Set value for every SmFeatureLoop 

NOTES:
***********************************************************************/
void SmFeatureExecutive::SetBaseRBType
(
    SmRebuildBrepType eRBType
)
{
    // locals
    ULONG ii;

    // collect the feature breps
    for ( ii = 0; ii < m_sFeatures.GetSize(); ii++ )
    { m_sFeatures[ii]->SetBaseRBType( eRBType ); }

} // end SmFeatureExecutive::SetBaseRBType

/*******************************************************************//**
PURPOSE: Set value for every SmFeatureLoop 

NOTES:
***********************************************************************/
void SmFeatureExecutive::SetFeatureRBType
(
    SmRebuildBrepType eRBType
)
{
    // locals
    ULONG ii;

    // collect the feature breps
    for ( ii = 0; ii < m_sFeatures.GetSize(); ii++ )
    { m_sFeatures[ii]->SetFeatureRBType( eRBType ); }

} // end SmFeatureExutive::SetFeatureRBType

/*******************************************************************//**
PURPOSE: 
  
NOTES:
***********************************************************************/
SmFeature* SmFeatureExecutive::AddFeatureInternal
(
    SmFace * pFace,
    SmTArray<SmFeatureLoop*> & rFLoops
)
{
    // Locals
    ULONG ii;
    ULONG lNumLoops = rFLoops.GetSize();

    // Create the feature and add the loops
    SmFeature* pFeature = new SmFeature(this);
    for ( ii = 0; ii < lNumLoops; ii++ )
    { pFeature->AddFeatureLoop( rFLoops[ii] ); }

    SmStatus stat = pFeature->CollectFaces( pFace );
    if(stat != SM_SUCCESS)
    {
      SE_MSG( SM_ERR, _T( "Feature edges do not partition the brep" ) );
      return NULL;
    }

    m_sFeatures.Add( pFeature );

    return pFeature;

} // end SmFeatureExecutive::AddFeatureInternal

/*******************************************************************//**
PURPOSE: Advanced AddFeature function: Add a feature where each 
     SmFeatureLoop has been created by the caller
  
NOTES: For memory management, each FeatureLoop is copied. The copy is
    used by the Feature and will be deleted when the FeatureExecutive 
    is deleted. The FeatureLoops input by the caller are the callers 
    responsibility to delete.
***********************************************************************/
SmFeature* SmFeatureExecutive::AddFeature
(
    SmFace * pFace,
    SmTArray<SmFeatureLoop*> & rFLoops
)
{
    // Locals
    ULONG ii;
    ULONG lNumLoops = rFLoops.GetSize();
    SmTArray<SmFeatureLoop*> sFLoops;

    for ( ii = 0; ii < lNumLoops; ii++ )
    {
        SmFeatureLoop * pFLoop = new SmFeatureLoop( *rFLoops[ii] );
        sFLoops.Add( pFLoop );
    }

    return AddFeatureInternal( pFace, sFLoops );

} // end SmFeatureExecutive::AddFeature

/*******************************************************************//**
PURPOSE: Add a feature to the SmFeatureExecutive by boundary edges
  
NOTES: 
***********************************************************************/
SmFeature* SmFeatureExecutive::AddFeature
(
    SmFace * pFace,
    SmTArray<SmEdge*> & rLoopEdges,
    SmRebuildBrepType   eOptBaseRBType,
    SmRebuildBrepType   eOptFeatureRBType
)
{
    // Locals
    ULONG ii, jj;
    SmTArray<SmFeatureLoop*>    sFLoops;
    SmTArray<SmRebuildBrepType> sBaseRBType, sFeatRBType;
    SmObjsDelete<SmFeatureLoop*> sCleanFLoops( &sFLoops );

    // Use topology traverser to sort edges into loops
    SmTArray<ULONG> sChainStart;
    SmTArray<SmEdge*> sLoopEdges( rLoopEdges );
    SmStatus eStat;
    eStat = SmTopologyTraverser::FixEdgeChainList( sLoopEdges, sChainStart );

    // If successfully sorted, create the proper SmFeatureLoops
    if ( eStat == SM_SUCCESS )
    {
        ULONG lNumChains = sChainStart.GetSize();

        for ( ii = 0; ii < lNumChains; ii++ )
        {
            SmTArray<SmEdge*> sFLoopEdges;
            ULONG lNumEdges = ( ii + 1 == lNumChains ) ? sLoopEdges.GetSize() : sChainStart[ii + 1];

            for ( jj = sChainStart[ii]; jj < lNumEdges; jj++ )
            { sFLoopEdges.Add( sLoopEdges[jj] ); }

            SmFeatureLoop *pFLoop = new SmFeatureLoop( sFLoopEdges,
                                                       eOptBaseRBType,
                                                       eOptFeatureRBType );
            sFLoops.Add( pFLoop );
        }
    }
    else
    {
        // Treat edge loops as a single SmFeatureLoop
        SE_MSG( SM_ERR, _T( "Unable to sort edges into chains. Continuing." ) );
        SmFeatureLoop *pFLoop = new SmFeatureLoop( rLoopEdges,
                                                   eOptBaseRBType,
                                                   eOptFeatureRBType );
        sFLoops.Add( pFLoop );
    }

    // Pass the call along and return
    return AddFeature( pFace, sFLoops );

} // end SmFeatureExecutive::AddFeature

/*******************************************************************//**
PURPOSE:  Add a feature to the SmFeatureExecutive by list of faces
  
NOTES: 
***********************************************************************/
SmFeature* SmFeatureExecutive::AddFeature
(
    SmTArray<SmFace*> & sFaces,
    SmRebuildBrepType   eOptBaseRBType,
    SmRebuildBrepType   eOptFeatureRBType
)
{
    if ( sFaces.GetSize() == 0 )
    {
        SE_MSG( SM_ERR, _T( "Adding a feature requires at least one face" ) );
        return NULL;
    }

    // Get all the edges from sFaces
    SmTArray<SmEdge*> sFeatureEdges, sInteriorEdges, sBoundaryEdges;
    for ( ULONG ii = 0; ii < sFaces.GetSize(); ii++ )
    {
        SmTArray<SmEdge*> sTheseEdges;
        sFaces[ii]->GetEdges( sTheseEdges );
        sFeatureEdges.AppendUnique( sTheseEdges );
    } // End for each face in sFaces

    // Determine which feature edges are on the boundary (will make SmFeatureLoops)
    for ( ULONG ii = 0; ii < sFeatureEdges.GetSize(); ii++ )
    {
        SmTArray<SmFace*> sEdgeFaces, sCommonFaces;
        SmEdge *pEdge = sFeatureEdges[ii];
        pEdge->GetFaces( sEdgeFaces );
        sFaces.FindCommonElements( sEdgeFaces, sCommonFaces );
        // Each boundary edge must be manifold, with one face in and one face out of feature
        if ( sCommonFaces.GetSize() == 1 && sEdgeFaces.GetSize() == 2)
        {
            sBoundaryEdges.Add( pEdge );
        }
    } // end for each edge in the feature

    // Use first face, since known to exist
    SmFace *pFace = sFaces[0];

    // Pass the call along and return
    return AddFeature( pFace, sBoundaryEdges, eOptBaseRBType, eOptFeatureRBType );

} // end SmFeatureExecutive::AddFeature

/*******************************************************************//**
PURPOSE:  Replace the Features with simpler geometry

NOTES: For each feature call the squence of methods:
    1) SmFeature::PartitionFeature()
    2) SmFeature::BuildPatches()
    3) SmFeature::ApplyPatches()
***********************************************************************/
SmStatus SmFeatureExecutive::DoDefeature
(
    SmBrep           *& prBaseBrep,     ///< [out]: Rebuilt base brep. 
    SmTArray<SmBrep*> * pFeatureBreps   ///< [out]: Rebuilt feature brep.
)
{
    // Prepare output
    prBaseBrep = NULL;
    if ( pFeatureBreps ) { pFeatureBreps->ReSet(); }

    // locals 
    ULONG ii, lNumFeatures = m_sFeatures.GetSize();
    SmTArray<SmBrep*> sFBreps;

    for ( ii = 0; ii < lNumFeatures; ii++ )
    {
        SmFeature* pFeature = m_sFeatures[ii];

        pFeature->PartitionFeature();
        pFeature->BuildPatches(); 
        pFeature->ApplyPatches();

        sFBreps.Add( pFeature->m_pFBrep );
    }

    // Set output
    prBaseBrep = m_pBaseBrep;
    if ( pFeatureBreps )
    { pFeatureBreps->Append( sFBreps ); }

    return SM_SUCCESS;

} // end SmFeatureExecutive::DoDefeature

/*******************************************************************//**
PURPOSE: Used after DoDefeature to restore all features that were 
    partitioned but not rebuilt

NOTES: Features that use SmRebuildBrepType SM_RB_NONE are considered
    to have been rebuilt and thus will not be added back to the Base
***********************************************************************/
SmStatus SmFeatureExecutive::RestoreFeatures()
{
    // locals
    ULONG ii;
    SmTArray<SmFeature*> sFeature = m_sFeatures;

    for ( ii = 0; ii < sFeature.GetSize(); ii++ )
    { 
        SmFeature *pFeature = sFeature[ii];
        if ( pFeature->RestoreFeature() == SM_SUCCESS )
        {
            m_sFeatures.RemoveAt( ii );
            delete pFeature; pFeature = NULL;
        }
    }

    return SM_SUCCESS;

} //end SmFeatureExecutive::RestoreFeatures()

/*******************************************************************//**
PURPOSE: Replace a single feature with a simpler geometry

NOTES:
  rLoopEdges must form closed loops of manifold edges. The edges need 
  not be ordered. These loops define the boundary of the feature.

  If the rebuild method fails, the bisected brep is returned in its
  bisected parts.
***********************************************************************/
SmStatus SmFeatureExecutive::DefeatureAndRebuild
( 
  const SmContext   & crContext,           // in :
  SmBrep           *& prOriginalBrep,      // i/o: Original brep to be defeatured
  SmFace            & rFeatureFace,        // in : A face on the feature section of rOriginalBrep, gets deleted
  SmTArray<SmEdge*> & rLoopEdges,          // in : A closed loop of manifold edges on rOriginalBrep
  SmBrep           *& prFeatureBrep,       // out: Feature brep
  SmRebuildBrepType   eRBBaseType,         // in : Rebuild method used with BaseBrep
  SmRebuildBrepType   eRBFeatureType       // in : Rebuild method used with FeatureBrep
)
{
    // Create Executive
    SmFeatureExecutive sDeExec( crContext, *prOriginalBrep );

    // Set the feature
    SmFeature * pFeature = sDeExec.AddFeature( &rFeatureFace, rLoopEdges, eRBBaseType, eRBFeatureType );
    AERN_MSG( pFeature != NULL, SM_ERR_ASSERT_FAILURE, _T( "Feature not built" ) );

    // Separate the feature from the base
    SmTArray<SmBrep*> sFeatures;
    SER( sDeExec.DoDefeature(prOriginalBrep, &sFeatures) );

    if ( sFeatures.GetSize() > 0 )
    { prFeatureBrep = sFeatures[0]; }

    return(SM_SUCCESS);

} // end SmFeatureExecutive::DefeatureAndRebuild

/*******************************************************************//**
PURPOSE: Replace a single feature with a simpler geometry

NOTES: The feature faces must all be topologically connected
    
  If the rebuild method fails, the partitioned brep is returned in its
  bisected parts.
***********************************************************************/
SmStatus SmFeatureExecutive::DefeatureAndRebuild
( 
  const SmContext   & crContext,           // in :
  SmBrep           *& prOriginalBrep,      // i/o: Original brep to be defeatured
  SmTArray<SmFace*> & rFeatureFaces,       // in : The contiguous faces of the feature
  SmBrep           *& prFeatureBrep,       // out: Feature brep
  SmRebuildBrepType   eRBBaseType,         // in : Rebuild method used with BaseBrep
  SmRebuildBrepType   eRBFeatureType       // in : Rebuild method used with FeatureBrep
)
{
    // Create Executive
    SmFeatureExecutive sDeExec( crContext, *prOriginalBrep );

    // Set the feature
    SmFeature * pFeature = sDeExec.AddFeature( rFeatureFaces, eRBBaseType, eRBFeatureType );   
    AERN_MSG( pFeature != NULL, SM_ERR_ASSERT_FAILURE, _T( "Feature not built" ) );
    
    // Separate the feature from the base
    SmTArray<SmBrep*> sFeatures;
    SER( sDeExec.DoDefeature(prOriginalBrep, &sFeatures) );

    if ( sFeatures.GetSize() > 0 )
    { prFeatureBrep = sFeatures[0]; }

    return(SM_SUCCESS);

} // end SmFeatureExecutive::DefeatureAndRebuild
/*******************************************************************//**
PURPOSE: Check that the constructed patch will work to fill void

NOTES: This function checks two states to determine quality.
    1) Are the only lamina edges along rLoopEdges?
        Failing this test says the patch is either too small 
        (surfaces not extended enough) or that the extended
        surfaces didn't get trimmed properly
    2) Does the patch intersect the brep only along edges and vertices?
        If there are Face intersections, adding the patch to the brep
        would be a non-manifold result.
***********************************************************************/
SmStatus SmFeatureExecutive::ValidatePatch
(
    const SmBrep            & rTargetBrep,
    const SmBrep            & rPatchBrep,
    const SmTArray<SmEdge*> & rLoopEdges
)
{
    // Locals
    ULONG ii, jj, kk;
    SmTArray<SmFace*> sFaces, sPatchFaces;
    rPatchBrep.GetFaces( sPatchFaces );
    SmBoolean bGoodPatch = TRUE;
    SmZoneTol3d dTol = SmTol::GetZoneTol3d( &rTargetBrep );

    /* Test patch for quality. Need
        0) Existence
        1) no unexpected lamina edges (patch too small)
        2) no internal intersections with pBrep
    */

    // Quality test 0: Existence check
    bGoodPatch = ( sPatchFaces.GetSize() > 0 ) ? TRUE : FALSE;
    AERN_MSG( bGoodPatch, SM_ERR_ASSERT_FAILURE, _T( "No Surfaces available to connect Loop Edges" ) );

    // Quality test 1: Check that each face is only lamina along rLoopEdges
    for ( ii = 0; ii < sPatchFaces.GetSize() && bGoodPatch; ii++ )
    {
        // Loop locals
        SmFace* pFace = sPatchFaces[ii];
        SmTArray<SmEdge*> sEdges;
        pFace->GetEdges( sEdges );

        // for each edge on this face
        for ( jj = 0; jj < sEdges.GetSize(); jj++ )
        {
            SmEdge* pEdge = sEdges[jj];
            if ( pEdge->IsLamina() )
            {
                SmBoolean bGoodEdge = FALSE;
                SmCurve* pCurve = pEdge->GetCurve();
                SmPoint3d sCrvPt; 
                pCurve->EvaluatePoint( pEdge->GetInterval().GetMid(), sCrvPt );
                // for each curve in rLoopEdges

                for ( kk = 0; kk < rLoopEdges.GetSize(); kk++ )
                {
                    SmBoolean bOnCurve = FALSE;
                    double    dParam, dDist;
                    SmEdge   *pCopyEdge = rLoopEdges[kk];
                    SmCurve  *pOrigCurve = pCopyEdge->GetCurve();

                    pOrigCurve->DropPoint( pCopyEdge->GetInterval(), sCrvPt, NULL, dTol, NULL, bOnCurve, dParam, dDist, SM_SO_INTERSECT );
                    if ( bOnCurve )
                    {
                        bGoodEdge = TRUE;
                        break;
                    }
                } // end for each edge->curve in rLoopEdges

                if ( !bGoodEdge )
                {
                    bGoodPatch = FALSE;
                    break;
                }
            } // end for lamina edges
        } // end for each edge on face
    } // end for each patch face

    AERN_MSG( bGoodPatch, SM_ERR_ASSERT_FAILURE, _T( "Patch has unexpected lamina edges. Could be too small or trimmed incorrectly." ) );

    // Get bounding boxes of Faces in sFaces
    rTargetBrep.GetFaces( sFaces );
    SmTArray<SmExtent3d> sBBoxes;
    sBBoxes.SetSize( sFaces.GetSize() );
    for ( ii = 0; ii < sFaces.GetSize(); ii++ )
    {
        SmExtent3d *pBBox = &sBBoxes[ii];
        SmFace    *pFace = sFaces[ii];
        pFace->CalculateBoundingBox( *pBBox );
    }

    // Quality test 2: Check that the patch isn't intersecting pBrep internally
    for ( ii = 0; ii < sPatchFaces.GetSize(); ii++ )
    {
        SmFace* pPatchFace = sPatchFaces[ii];
        SmExtent3d sPatchFaceBBox;
        pPatchFace->CalculateBoundingBox( sPatchFaceBBox );

        for ( jj = 0; jj < sFaces.GetSize(); jj++ )
        {
            SmFace* pFace = sFaces[jj];

            // Quick check via bounding boxes
            SmExtent3d sFaceBBox = sBBoxes[jj];
            if ( sPatchFaceBBox.AreDisjoint(sFaceBBox) )
            { continue; }

            SmTArray<SmCurve*> s3DCurves, sCC3DCrvs, sCCUVCrvs;
            SmTArray<SmCurveClassification*> sCurveClasses;
            SmObjsDelete<SmCurve*> sClean3D( &s3DCurves ), sCleanCC3D(&sCC3DCrvs), sCleanCCUV(&sCCUVCrvs);
            SmObjsDelete<SmCurveClassification*> sCleanCC( &sCurveClasses );

            // intersect the faces (Method shouldn't force ApproxTol3d)
            SER( pPatchFace->FaceIntersect( m_crContext, pFace, (SmApproxTol3d) dTol, 20.0, s3DCurves, NULL, NULL, NULL, &sCurveClasses, NULL, &sCC3DCrvs, &sCCUVCrvs ) );

            // Check the midpoint of each curve classification 
            for ( kk = 0; kk < sCurveClasses.GetSize(); kk++ )
            {
                SmCurveClassification *pCurveClass = sCurveClasses[kk];
                SmPointClassification sPtClass( dTol, &m_crContext );

                double dParam = pCurveClass->GetCurve()->GetNaturalInterval().GetMid();

                pCurveClass->FindPointClass( dParam, dTol, sPtClass );

                // If the curve is internal to the face, then the new face is a bad result
                AERN_MSG( sPtClass.GetClassType() != SM_PC_FACE, SM_ERR_ASSERT_FAILURE, _T( "The patch intersects the existing Brep" ) );

            } // end for each curve classification
        } // end for each face in pBrep
    } // end for each face in pPatchBrep

    return SM_SUCCESS;
} // end SmFeatureExecutive::ValidatePatch()
  
/*******************************************************************//**
PURPOSE: Attempt to fill hole in pBrep along LoopEdges with planar cap

NOTES: Helper function for RebuildBrep
***********************************************************************/
SmStatus SmFeatureExecutive::PlanarCapRebuild
( 
  SmBrep                  * pPatchBrep,///< [i/o]: brep built to patch the SmFeatureLoop
  const SmTArray<SmEdge*> & rLoopEdges ///< [in] : Loop edges corresponding to the SourceBrep
)
{
    // Locals
    ULONG ii;
    SmFace               * pNewFace = NULL;
    SmZoneTol3d            dTol = pPatchBrep->GetTolerance();
    SmTArray<SmCurve*>     sCurves;
    SmObjsDelete<SmCurve*> sClean1( &sCurves );
    pPatchBrep->SetTolerance( dTol );

    // Get copy of curves for new planar face from loop edges
    for ( ii = 0; ii < rLoopEdges.GetSize(); ii++ )
    {
        SmCurve* pCurve = NULL;
        rLoopEdges[ii]->GetCurve()->Copy( m_crContext, pCurve );
        sCurves.Add( pCurve );
    }
    SmTArray<ULONG> sChainStart;
    SmCurve::FixChainCurvesList( sCurves, sChainStart, dTol, FALSE );

    for ( ii = 0; ii < sChainStart.GetSize(); ii++ )
    {
        SmTArray<SmCurve*> sLoop;
        SmObjsDelete<SmCurve*> sClean2( &sLoop );
        ULONG sIndexStart = sChainStart[ii];
        ULONG sIndexEnd = 0;
        if ( ii < sChainStart.GetSize() - 1 )
        { sIndexEnd = sChainStart[ii + 1]; }
        else
        { sIndexEnd = sCurves.GetSize(); }

        do
        {
            sLoop.Add( sCurves[sIndexStart] );
            sCurves[sIndexStart] = NULL;
            sIndexStart++;
        } while ( sIndexStart < sIndexEnd );

        // Create planar face
        SER_MSG( pPatchBrep->CreatePlanarFaceWith3DCurves( NULL, sLoop, dTol, pNewFace ), _T( "Couldn't make a planar cap. Loop edges non-planar?" ) );

        sClean2.Clear();
    }

    return SM_SUCCESS;

} // end SmFeatureExecutive::PlanarCapRebuild

/*******************************************************************//**
PURPOSE: Attempt to fill hole in pBrep along LoopEdges from extended surfaces

NOTES: Helper function for RebuildBrep
***********************************************************************/
SmStatus SmFeatureExecutive::ExtSurfaceRebuild
 (SmBrep                  & rPatchBrep,   ///< [i/o]: brep built to patch the SmFeatureLoop
  SmBrep                  & rSourceBrep,  ///< [in] : Geometry base for SM_RB_EXTEND_SURFACE
  const SmTArray<SmEdge*> & crLoopEdges,  ///< [in] : Loop edges corresponding to the SourceBrep
  double                    dExtDist,     ///< [in] : Extension distance for SM_RB_EXTEND_SURFACE method
  SmTArray<SmFace*>       & rFaces)       ///< NotUsed: [out]: Faces used to construct the patch
{
    SM_REF1(rFaces) ;
    // Locals
    ULONG ii, jj;
    SmZoneTol3d dTol = rSourceBrep.GetTolerance();
    SmTArray<SmFace*> sFaces;
    SmBoolean bTestMergeAtEnd = FALSE;
    SmBrep *pTempPatch = new ( m_crContext ) SmBrep();
    SmObjDelete sCleanResult( pTempPatch );

    // Get all faces that use rLoopEdges
    for ( ii = 0; ii < crLoopEdges.GetSize(); ii++ )
    {
        SmEdge* pEdge = crLoopEdges[ii];
        SmTArray<SmFace*> sLocalFaces;
        pEdge->GetFaces( sLocalFaces );
        sFaces.AppendUnique( sLocalFaces );
    } // end for each edge in crLoopEdges

#ifdef SM_DEBUG_CODE
    SmBoolean bDebugMe = FALSE;
    if ( bDebugMe )
    {
        // Examine surfaces available for rebuild
        smgfx_Erase();
        for ( ULONG di = 0; di < sFaces.GetSize(); di++ )
        {
            SmSurface* pSurf = sFaces[di]->GetSurface();
            pSurf->Draw(); sm_GraphicsLoop();
            sFaces[di]->Draw(); sm_GraphicsLoop();
        }
        sm_GraphicsLoop();
    }
#endif  // SM_DEBUG_CODE

    // Set the distance to extend surfaces
    double dExt;
    if ( dExtDist < 0.0 )
    {
        // Estimate void size to set surface extensions.
        SmExtent3d sVoidBBox;
        for ( ii = 0; ii < crLoopEdges.GetSize(); ii++ )
        {
            SmExtent3d sThisBBox;
            SmEdge* pEdge = crLoopEdges[ii];
            pEdge->CalculateBoundingBox( &sThisBBox );

            sVoidBBox.Union( sThisBBox, sVoidBBox );
        }
        dExt = 2.*sVoidBBox.GetMaxLength();
    }
    else
    { dExt = dExtDist; }

    /* For each pFace along crLoopEdges:
        1) Create a patch face from each pFace
            A) If crLoopEdges are part of pFace inner loop, then invert face 
            B) If not part of inner loop, then:
                1) extend the surface
                2) Make pNewFace from the extended surface
                3) Difference the original face from the extended surface
        4) Copy pNewFace into pTempPatch
    */

    for ( ii = 0; ii < sFaces.GetSize(); ii++ )
    {
        // loop locals
        SmFace    *pFace = sFaces[ii];
        SmSurface *pNewSurf = NULL;

        // make a brep for the new face
        SmBrep* pFaceBrep = new ( m_crContext ) SmBrep();
        pFaceBrep->SetTolerance( dTol );
        pFaceBrep->m_bEditingEnabled = TRUE;

        // Check if this is part of inner loop
        SmTArray<SmEdge*> sFOEdges, sCommonEs;
        pFace->GetEdges( sFOEdges, NULL, TRUE );
        sFOEdges.FindCommonElements( crLoopEdges, sCommonEs );

        // if it is part of an inner loop
        // After Boolean restructure, we don't need this special case for inner loops, but it's more efficient
        if ( sCommonEs.GetSize() == 0 )
        {
            // Locals
            SmTArray<SmFace*> sFaceToCopy, sNewFaces;
            SmTArray<SmEdge*> sFEdges;
            SmTArray<SmLoop*> sLoops, sLoopsCopy;
            SmLoop           *pThisLoop = NULL;

            // Get an edge on this face that is part of crLoopEdges
            pFace->GetEdges( sFEdges );
            sFEdges.FindCommonElements( crLoopEdges, sCommonEs );
            SmEdge *pE = sCommonEs[0];  // must exist since face is along crLoopEdges

            // Get the loops of the face to copy
            pFace->GetLoops( sLoops );

            // For the face to copy, remove all loops except the inner loop that shares an edge w/ crLoopEdges
            // Make that one loop the outer loop by flipping its orientation
            for ( jj = 0; jj < sLoops.GetSize(); jj++ )
            {
                SmLoop *pLoop = sLoops[jj];
                ULONG   lIdx;
                SmTArray<SmEdge*> sLEdges;
                pLoop->GetEdges( sLEdges );
                if ( sLEdges.FindElement( pE, lIdx ) )
                { 
                    pLoop->FlipLoopOrientation();
                    pThisLoop = pLoop;
                }
                else
                { pFace->RemoveLoop( pLoop ); }
            }

            // Place a copy of this face into pFaceBrep
            sFaceToCopy.Add( pFace );
            rSourceBrep.CopyFaces( sFaceToCopy, pFaceBrep, &sNewFaces );

            // Put the face back together for subsequent testing
            pThisLoop->FlipLoopOrientation();
            pFace->RemoveLoop( pThisLoop );
            for ( jj = 0; jj < sLoops.GetSize(); jj++ )
            {
                SmLoop *pLoop = sLoops[jj];
                SmTArray<SmEdge*> sLEdges;
                pFace->InsertLoop( pLoop ); 
            }

        } // end if it's an inner loop
        else
        {
            // Extend the surface
            pFace->GetSurface()->CreateExtendedSurface( m_crContext, dExt, SM_CT_G1, pNewSurf );
            SmExtent2d sSrfIvl = pNewSurf->GetNaturalUVDomain();

            // Make a face from pSurface
            SmFace *pNewFace = NULL;
            pFaceBrep->CreateFaceFromSurface( pNewSurf,   // in : target face
                                              sSrfIvl,     // in : desired face Nurb domain
                                              pNewFace );  // out: New Face connected to this Brep

            // Make the outer loop of pFace into an inner loop of pNewFace
            SmBrep *pRemoveBrep = new( m_crContext ) SmBrep();
            SmTArray<SmFace*> sLocalFaces, sNewFaces;

            sLocalFaces.Add( pFace );
            rSourceBrep.CopyFaces( sLocalFaces, pRemoveBrep, &sNewFaces );

            SmMerge sMerge( m_crContext, pFaceBrep, pRemoveBrep );
            sMerge.ManifoldBoolean( SM_BO_EXCLUSIVE_OR, pFaceBrep );

        } // End if crLoopEdges are not part of inner loop

        // Arrive here after surface has been extended and made into pNewFace less pFace.
        // Next, copy and stitch the new face(s) in pPatchBrep.
        // Note: multiple faces possible when m_dExtDist = 0.

#if 0
        // Below are 2 other methods for possible ways to merge new faces into the brep
        // The most robust method is the one in use, but keep these 2 around for testing
        if (0)
        {
            // Stitch face locals
            SmObjDelete sFaceBrepClean( pFaceBrep );
            ULONG       lNumEdgesStitched;
            double      dMaxVertGap, dMaxEdgeGap, dMinUnstitchedVertGap, dMinUnstitchedEdgeGap;
            SmTArray<SmFace*>   sMergeFaces;
            SmTArray<SmEdge*>   sMergeEdges;
            SmTArray<SmVertex*> sMergeVerts;

            SmBoolean m_bMakingManifoldSolid = FALSE;
            SmBoolean m_bFastEdgeCompare = TRUE;
            SmBoolean m_bDoRegionNesting = FALSE;
            SmBoolean m_bIgnoreProblems = TRUE;

            // Copy faces from pFaceBrep to pPatchBrep
            SmTArray<SmFace*> sTheseFaces;
            pFaceBrep->GetFaces( sTheseFaces );
            pFaceBrep->CopyFaces( sTheseFaces, &rPatchBrep, &sMergeFaces );

            // SmMerge exec to stitch pFaceBrep into rPatchBrep
            SmBrep* pDummyBrep = NULL;
            SmMerge sPatchMerge( m_crContext, &rPatchBrep, pDummyBrep, dTol );

            // Collect new topology to stitch into pPatchBrep
            for ( jj = 0; jj < sMergeFaces.GetSize(); jj++ )
            {
                SmFace* pMergeFace = sMergeFaces[jj];
                SmTArray<SmEdge*>   sTempEs;
                SmTArray<SmVertex*> sTempVs;

                pMergeFace->GetEdges( sTempEs );
                pMergeFace->GetVertices( sTempVs );

                sMergeEdges.AppendUnique( sTempEs );
                sMergeVerts.AppendUnique( sTempVs );
            }

            sPatchMerge.MergeAddedTopology( &sMergeFaces, &sMergeEdges, &sMergeVerts,
                                        lNumEdgesStitched,
                                        dMaxVertGap,  // in/out
                                        dMaxEdgeGap,
                                        dMinUnstitchedVertGap,
                                        dMinUnstitchedEdgeGap,
                                        m_bMakingManifoldSolid,
                                        m_bFastEdgeCompare,
                                        m_bDoRegionNesting,
                                        m_bIgnoreProblems );

        } // end MergeAddedTopology to merge new geometry
        else if (0)
        {
            SmBrep *pResult; 
            SmMerge sPatchMerge( m_crContext, &rPatchBrep, pFaceBrep, dTol );
            sPatchMerge.ManifoldBoolean( SM_BO_MERGE, pResult );
            // pResult == &rPatchBrep
        } // end Test block for ManifoldBoolean
        else
#endif
        {
            // Copy faces from pFaceBrep to pTempPatch
            SmObjDelete sFaceBrepClean( pFaceBrep );
            SmTArray<SmFace*> sTheseFaces;
            pFaceBrep->GetFaces( sTheseFaces );
            SE( pFaceBrep->CopyFaces( sTheseFaces, pTempPatch, NULL, TRUE ) );

            bTestMergeAtEnd = TRUE;
        }


#ifdef  SM_DEBUG_CODE
        if ( bDebugMe )
        {
            SmTArray<SmFace*> sFacesToDraw;
            //pFaceBrep->GetFaces( sFacesToDraw );
            //smgfx_Erase();
            //for ( ULONG di = 0; di < sFacesToDraw.GetSize(); di++ )
            //{
            //    SmFace *pFace = sFacesToDraw[di];
            //    smgfx_ChangeColor(di != 0);
            //    pFace->Draw(SM_DM_CROSSHATCH); sm_GraphicsLoop();
            //}
            //sm_GraphicsLoop();

            pTempPatch->GetFaces( sFacesToDraw );
            smgfx_Erase();
            for ( ULONG di = 0; di < sFacesToDraw.GetSize(); di++ )
            {
                SmFace *pFaceToDraw = sFacesToDraw[di];
                smgfx_ChangeColor(di != 0);
                pFaceToDraw->Draw(SM_DM_CROSSHATCH); sm_GraphicsLoop();
            }
            sm_GraphicsLoop();
        }
#endif
    } // end making a face in pPatchBrep from each surface

    if ( bTestMergeAtEnd )
    {
        SmBrep *pDummy;
        SmMerge sPatchMerge( m_crContext, &rPatchBrep, pTempPatch, dTol );

        sPatchMerge.PiecewiseMerge( NULL, FALSE, TRUE, pDummy );
    }

    // Delete the faces we don't want in the patch
    //  A) Delete faces that don't use any edge->curve from crLoopEdges
    //  B) Delete faces that overlap with rSourceBrep
    sm_TrimPatches( rPatchBrep, crLoopEdges, sFaces );

    // Stitch and orient locals
    rPatchBrep.StitchAndOrient();

    // Extended surfaces are likely much larger than the faces, so fix this.
    rPatchBrep.ShrinkGeometry();

#ifdef SM_DEBUG_CODE
    if ( bDebugMe )
    {
        smgfx_Erase();
        rPatchBrep.Draw( TRUE ); sm_GraphicsLoop();
        sm_GraphicsLoop();
    }
#endif //SM_DEBUG_CODE

    return SM_SUCCESS;
} // SmFeatureExecutive::ExtSurfaceRebuild

/*******************************************************************//**
PURPOSE: Attempt to fill hole in pBrep along LoopEdges from extended surfaces
Helper function for RebuildBrep

NOTES: In Development. Does not work.
***********************************************************************/
SmStatus SmFeatureExecutive::NSidedPatchRebuild
(
  SmBrep            & rPatchBrep,
  SmBrep            & rSourceBrep,
  SmTArray<SmEdge*> & rLoopEdges
)
{
    // Try CreateNSidedPatch without derivative-curves. Guessing that derivative curves will
    // be automatically filled with binormals. That's what I'm hoping for.
    // knife edge showing up in new surfaces. Cured by defining normals?

    // Locals
    ULONG ii;
    SmTArray<SmBSplineCurve*> sLoopBCurves, sDerivCrvs;
    SmTArray<SmSurface*>      sNewSurfaces;
    SmZoneTol3d dTol = rSourceBrep.GetTolerance();
    rPatchBrep.SetTolerance( dTol );

    // Get non-rational BSplineCurves from edges, may change curve shape
    for ( ii = 0; ii < rLoopEdges.GetSize(); ii++ )
    {
        SmBSplineCurve* pCurve = rLoopEdges[ii]->CreateTrimmedNURBSCurve( m_crContext );
        NER( pCurve );

        pCurve->MakeNonRational();
        AERN_MSG( !pCurve->IsRational(), SM_ERR_ASSERT_FAILURE,
                 _T( "N-sided patch requires non-rational curves. Since making curves non-rational "
                     "can move them, this function not support in this case" ) );

        sLoopBCurves.Add( pCurve );
        sDerivCrvs.Add( NULL );
    }

#ifdef SM_DEBUG_CODE
    SmBoolean bDebugMe = FALSE;
    if ( bDebugMe )
    {
        smgfx_Erase();
        smgfx_SetLook( 3, 3, 1, 0, 0 );
        for ( ULONG di = 0; di < sLoopBCurves.GetSize(); di++ )
        {
            sLoopBCurves[di]->Draw(); sm_GraphicsLoop();
            sm_GraphicsLoop();
        }
        smgfx_SetLook( 1, 1, 0, 0, 0 );
        rSourceBrep.Draw(); sm_GraphicsLoop();
        sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

    // Create patch
    SmBSplineSurface::CreateNSidedPatch( m_crContext,
                                         sLoopBCurves,
                                         sDerivCrvs,
                                         1e-5,
                                         sNewSurfaces );

    // Make new surfaces from new patches
    for ( ii = 0; ii < sNewSurfaces.GetSize(); ii++ )
    {
        SmSurface* pNewSurface = sNewSurfaces[ii];
        SmFace*    pNewFace = NULL;
        rPatchBrep.CreateFaceFromSurface( pNewSurface, pNewSurface->GetNaturalUVDomain(), pNewFace );
    }

    return SM_ERR;

} // end SmFeatureExecutive::NSidedPatchRebuild

/*******************************************************************//**
PURPOSE: Redirect the brep to the right rebuild method

NOTES:
***********************************************************************/
SmStatus SmFeatureExecutive::RebuildBrep
( 
    SmBrep           *& prPatchBrep,    ///< [out]: new brep build to patch the SmFeatureLoop
    SmTArray<SmEdge*> & rLoopEdges,     ///< [in] : Loop edges corresponding to the SourceBrep
    SmRebuildBrepType   eRebuildType,   ///< [in] : Rebuild method employed
    double              dExtDist,       ///< [in] : Extension distance for SM_RB_EXTEND_SURFACE method
    SmBrep            * pSourceBrep,    ///< [in] : Geometry base for SM_RB_EXTEND_SURFACE
    SmTArray<SmFace*> & rFaces          ///< [out]: Faces used to construct the patch
)    
{
  /* A few methods for possible development
      1) CreateNSidedPatch (planar (don't use) and non-planar)
        STARTED (on hold, due to need for non-rational curves)
      2) CreateTaperExtrude (all offset curves are calculated from a single normal. This strategy would have to be modified)
      3) ExtendAndTrimEdges from OffsetExecutive (extend faces, rather than surfaces)
   */

    // If rebuilding from surface, use the InnerLoop method if possible
    if ( eRebuildType == SM_RB_FROM_OTHER_BREP )
    { eRebuildType = SM_RB_EXTEND_SURFACE; }

    switch ( eRebuildType )
    {
    //case SM_RB_NSIDED_PATCH:
    //{ return NSidedPatchRebuild( pSourceBrep, pPatchBrep, rLoopEdges ); }
    case SM_RB_PLANAR_CAP:
    { return PlanarCapRebuild( prPatchBrep, rLoopEdges ); }
    case SM_RB_EXTEND_SURFACE:
    { return ExtSurfaceRebuild( *prPatchBrep, *pSourceBrep, rLoopEdges, dExtDist, rFaces ); }
    case SM_RB_NONE:
    { return SM_SUCCESS; }

    default: return SM_ERR;
    }

    // if ( 0 )
    // { return NSidedPatchRebuild( *prPatchBrep, *pSourceBrep, rLoopEdges ); }

} // end SmFeatureExecutive::RebuildBrep

/*******************************************************************//**
PURPOSE: 

NOTES:
***********************************************************************/
void SmFeatureExecutive::Dump() const
{
    // pass the call along
    Dump( SM_BD_BASE_ONLY );
    return;
} // end SmFeatureExecutive::Dump()

/*******************************************************************//**
PURPOSE: 

NOTES:
***********************************************************************/
void SmFeatureExecutive::Dump(SmBrepDumpType eBDType) const
{
    ULONG ii, lNumFeatures = m_sFeatures.GetSize();
    TCHAR sBuff[SM_TBLOCK_SIZE];

    // Header
    smos_sprintf( sBuff, _T( "%s" ), _T("\n\nDump of SmFeatureExecutive ") );
    smos_WriteBuffer( sBuff );

    smos_sprintf( sBuff, _T( "\n\nNumber of Features [ %4lu] " ), lNumFeatures );
    smos_WriteBuffer( sBuff );

    // Dump the base
    smos_sprintf( sBuff, _T( "%s" ) , _T("\n\nDump of m_pBaseBrep: ")  );
    smos_WriteBuffer( sBuff );
    m_pBaseBrep->Dump( eBDType );

    // Dump the features
    smos_sprintf( sBuff, _T( "%s" ) , _T("\n\nDump of SmFeatures: ") );
    smos_WriteBuffer( sBuff );

    for (ii =0; ii < lNumFeatures; ii++ )
    { m_sFeatures[ii]->Dump(eBDType); }

    return;
} // end SmFeatureExecutive::Dump()

#if 0

// Add the following as the last entry to SmRebuildBrepType
//  SM_RB_MAX = SM_RB_NSIDED_PATCH, // Internal. Keep set to last entry, w/contiguous entries. used by Rebuild*BrepAll methods

/*******************************************************************//**
PURPOSE: Attempt to fill the hole in BaseBrep created by bisection with
  all available methods. This method does not edit m_pBaseBrep.

NOTES: Only call this after DoDefeature
  The results are in the order of the definition of SmRebuildBrepType:
  1) SM_RB_NONE
  2) SM_RB_PLANAR_CAP
  3) SM_RB_FROM_SURFACE
  4) SM_RB_INNER_LOOP
  5) SM_RB_NSIDED_PATCH

  If method ii fails, then rBreps[ii] == NULL.
***********************************************************************/
SmStatus SmFeatureExecutive::RebuildBrepAll
( SmBoolean           bBaseBrep, // in : TRUE == rebuild the base brep
                                //      FALSE == rebuild the feature brep
  SmTArray<SmBrep*> & rBreps )    // out: Rebuilt breps, rBreps[ii] == NULL if method ii fails
{
  // Validate readiness
    AERN( m_bIsPartitioned , SM_ERR_ASSERT_FAILURE);

    // locals
    ULONG lNumMethods = SM_RB_MAX + 1; // Number of methods to try
    ULONG ii, jj;
    SmTArray<ULONG> sEdgeIDs;

    SmBrep* pOpBrep = (bBaseBrep) ? m_pBaseBrep : m_pFeatureBrep;
    SmTArray<SmEdge*> sOpEdges = (bBaseBrep) ? m_sBaseLoopEdges : m_sFeatureLoopEdges;
    SmBoolean bOpIsInnerLoop = (bBaseBrep) ? m_bBaseIsInnerLoop : m_bFeatureIsInnerLoop;
    SmBoolean bOpIsOuterLoop = (bBaseBrep) ? m_bBaseIsOuterLoop : m_bFeatureIsOuterLoop;

    // prepare for output
    rBreps.SetSize( lNumMethods );

    for ( ii = 0 ; ii < sOpEdges.GetSize() ; ii++ )
    {
        ULONG lIndex = sOpEdges[ii]->GetEdgeNumberInBrep();
        sEdgeIDs.Add( lIndex );
    }

  // Rebuild base brep with each method
    for ( ii = 0 ; ii < lNumMethods ; ii++ )
    {
      // Set the rebuild type for this iteration
        SmRebuildBrepType eRebuildType = SmRebuildBrepType( ii );

        // prepare the (possibly temporary) brep for rebuilding
        SmBrep *pBrep = new (m_crContext) SmBrep( *pOpBrep );
        SmObjDelete sCleanBrep( pBrep );
        SmTemporaryChangeValue<SmBoolean> saveBaseBrep( pBrep->m_bEditingEnabled, TRUE );

        // Collect the loop edges in the new brep
        SmTArray<SmEdge*> sEdges;
        for ( jj = 0 ; jj < sEdgeIDs.GetSize() ; jj++ )
        {
            SmTArray<SmEdge*> sAllEdges;
            pBrep->GetEdges( sAllEdges );
            sEdges.Add( sAllEdges[sEdgeIDs[jj]] );
        }

#if SM_DEBUG_CODE
        SmBoolean bDebugMe = FALSE;
        if ( bDebugMe )
        {
            smgfx_Erase();
            pBrep->Draw(); sm_GraphicsLoop();
            sm_GraphicsLoop();
        }
#endif // SM_DEBUG_CODE

        if ( SM_SUCCESS == RebuildBrep( pBrep, bOpIsInnerLoop, bOpIsOuterLoop, sEdges, eRebuildType ) )
        {
            rBreps[ii] = pBrep;
            sCleanBrep.Clear();
        }
        else
        { rBreps[ii] = NULL; }
    } // end for each method

    return SM_SUCCESS;
} // end SmFeatureExecutive::RebuildBaseBrepAll

#endif
