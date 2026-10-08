// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*___*/
/**********************************************************************
* FILE NAME --- smfeature_test.cpp
* PURPOSE --- Source code file for testing defeaturing and rebuilding breps
*
**********************************************************************/
/*___*/

#include "StdAfx.h"

#include "smFeatureTest.h"
#include "SmAssertArray.h"

/*******************************************************
PURPOSE --- Test rebuilding fillets

USAGE NOTES ---
********************************************************/
SmStatus my_test_rebuild_fillets()
{
    SmStatus bRtn = SM_SUCCESS;
    ULONG lTestCase = 0;
    SmContext sContext;

    while(TRUE)
    {
        SmTArray<SmEdge*> sAllEdges, sLoopEdges;
        SmTArray<SmFace*> sFaces, sFeatureFaces;
        SmTArray<ULONG>   sEdgeIDs;
        SmTArray<ULONG>   sFaceIDs;
        ULONG lBF, lBV, lBE, lFF, lFV, lFE, lBR = 2, lBS = 2, lFR = 2, lFS = 2;
        const TCHAR *pFileName;
        TCHAR sBuff[SM_TBLOCK_SIZE];

        // Read the file for defeature
        switch ( lTestCase )
        {
        case 0:
        {
            // Part
            pFileName = _T( "../../TestFiles/pt_TestFiles/Defeature/defillet.smb" ) ;
            // Feature face
            sFaceIDs.Add(8);
            // Edges surrounding feature
            sEdgeIDs.Add( 14 ); sEdgeIDs.Add( 16 ); 
            // base result
            lBF = 10, lBE = 18, lBV = 12;
            // base result
            lFF = 3, lFE = 5, lFV = 3;
            break;
        }
        default:
            return bRtn;
        }

        SM_SPRINTF( sBuff, _T( "\n\nEntered my_test_rebuild_fillets case: %ld"), lTestCase );
        MYPRINTF( sBuff );

        SmBrep *pOrigBrep = new( sContext ) SmBrep();
        pOrigBrep->ReadFromFile( sContext, pFileName );

        // Get feature faces and edges
        pOrigBrep->GetEdges( sAllEdges );
        pOrigBrep->GetFaces( sFaces );
        for ( ULONG ii = 0; ii < sFaceIDs.GetSize(); ii++ )
        { sFeatureFaces.Add(sFaces[sFaceIDs[ii]]); }
        sLoopEdges.ReSet();

        for ( ULONG ii = 0; ii < sEdgeIDs.GetSize(); ii++ )
        {
            SmEdge* pEdge = sAllEdges[sEdgeIDs[ii]];
            sLoopEdges.Add( pEdge );
        }

        SmFeatureExecutive sDeExec( sContext, *pOrigBrep );
        sDeExec.AddFeature(sFeatureFaces[0], sLoopEdges );

        // SM_RB_EXTEND_SURFACE= builds from extended surfaces
        SmBrep *pBaseBrep = NULL, *pFeatureBrep = NULL;
        SmTArray<SmBrep*> sFeatureBreps;

        SE( sDeExec.DoDefeature( pBaseBrep, &sFeatureBreps ));

        if(sFeatureBreps.GetSize() > 0)
        { pFeatureBrep = sFeatureBreps[0]; }
        
        // Validate base
        if ( pBaseBrep != NULL )
        { 
            SmStatus eStat;
            SE( eStat = pBaseBrep->ValidateCounts( lBF, lBE, lBV, 0, 0, lBE, lBR, lBS ) );
            bRtn = ( eStat == SM_SUCCESS ) ? bRtn : SM_ERR;
            SM_DUMP( pBaseBrep );
        }
        else 
        { 
            SE_MSG( SM_ERR, _T( "Failed to rebuild base brep" ) );
            bRtn = SM_ERR; 
        }

        // Validate Feature
        if ( pFeatureBrep != NULL )
        {
            SmStatus eStat;
            SE( eStat = pFeatureBrep->ValidateCounts( lFF, lFE, lFV, 0, 0, lFE, lFR, lFS ) );
            bRtn = ( eStat == SM_SUCCESS ) ? bRtn : SM_ERR;
            SM_DUMP( pFeatureBrep );
        }
        else
        { 
            SE_MSG( SM_ERR, _T( "Failed to rebuild feature brep" ) );
            bRtn = SM_ERR; 
        }

        SmObjDelete sCleanB( pBaseBrep );
        SmObjsDelete<SmBrep*> sCleanF( &sFeatureBreps );

        lTestCase++;

#ifdef SM_DEBUG_CODE
        SmBoolean bDebugMe = FALSE;
        if ( smGet_DoGraphics() && bDebugMe )
        {
            smgfx_Erase();
            if (pOrigBrep) { pOrigBrep->Draw(TRUE); sm_GraphicsLoop(); }
            sm_GraphicsLoop();

            smgfx_Erase();
            if (pBaseBrep) { pBaseBrep->Draw(); sm_GraphicsLoop(); }
            sm_GraphicsLoop();

            smgfx_Erase();
            if (pFeatureBrep) { pFeatureBrep->Draw(); sm_GraphicsLoop(); }
            sm_GraphicsLoop();
        }
#endif
    }
} //end my_test_multi_feature
/*******************************************************
PURPOSE --- Test defining SmFeatures by SmTArray<SmFace*>

USAGE NOTES ---
********************************************************/
SmStatus my_test_add_by_faces()
{
    SmStatus bRtn = SM_SUCCESS;
    ULONG lTestCase = 0;
    SmContext sContext;

    while(TRUE)
    {
        SmTArray<SmFace*> sFaces;
        SmTArray<SmFace*> sFeatureFaces1, sFeatureFaces2, sFeatureFaces3;
        SmTArray<ULONG> sFeatureIDs1, sFeatureIDs2, sFeatureIDs3;
        SmRebuildBrepType eBaseRB, eFeatRB;
        ULONG lBF, lBV, lBE, lFF, lFV, lFE, lBR = 2, lBS = 2, lFR = 2, lFS = 2;
        const TCHAR *pFileName;
        TCHAR sBuff[SM_TBLOCK_SIZE];

        // Read the file for defeature
        switch ( lTestCase )
        {
        case 0:
        {
            // Part
            pFileName = _T( "../../TestFiles/pt_TestFiles/Defeature/extrusion.smb" );
            // cylinder
            //lFaceID = 10;
            sFeatureIDs1.Add( 10 );

            // rectangle
            // face id 9
            sFeatureIDs2.Add( 6 );
            sFeatureIDs2.Add( 7 );
            sFeatureIDs2.Add( 8 );
            sFeatureIDs2.Add( 9 );

            // Corner 
            // face id 4
            sFeatureIDs3.Add( 4 );

            // base result
            lBF = 13, lBE = 27, lBV = 20;
            // base result
            lFF = 3, lFE = 3, lFV = 2;

            eBaseRB = SM_RB_EXTEND_SURFACE;
            eFeatRB = SM_RB_FROM_OTHER_BREP;
            break;
        }
        default:
            return bRtn;
        }

        SM_SPRINTF( sBuff, _T( "\n\nEntered my_test_multi_feature case: %ld"), lTestCase );
        MYPRINTF( sBuff );

        SmBrep *pOrigBrep = new( sContext ) SmBrep();
        pOrigBrep->ReadFromFile( sContext, pFileName );

        // Get feature faces and edges
        pOrigBrep->GetFaces( sFaces );
        for ( ULONG ii = 0; ii < sFeatureIDs1.GetSize(); ii++ )
        { sFeatureFaces1.Add( sFaces[sFeatureIDs1[ii]] ); }
        for ( ULONG ii = 0; ii < sFeatureIDs2.GetSize(); ii++ )
        { sFeatureFaces2.Add( sFaces[sFeatureIDs2[ii]] ); }
        for ( ULONG ii = 0; ii < sFeatureIDs3.GetSize(); ii++ )
        { sFeatureFaces3.Add( sFaces[sFeatureIDs3[ii]] ); }

        SmFeatureExecutive sDeExec( sContext, *pOrigBrep );
        sDeExec.AddFeature( sFeatureFaces1, eBaseRB, eFeatRB );
        sDeExec.AddFeature( sFeatureFaces2, eBaseRB, eFeatRB );
        sDeExec.AddFeature( sFeatureFaces3, eBaseRB, eFeatRB );

        // SM_RB_EXTEND_SURFACE= builds from extended surfaces
        SmBrep *pBaseBrep = NULL, *pFeatureBrep = NULL;
        SmTArray<SmBrep*> sFeatureBreps;

        SE( sDeExec.DoDefeature( pBaseBrep, &sFeatureBreps ) );

        if(sFeatureBreps.GetSize() > 0)
        { pFeatureBrep = sFeatureBreps[0]; }
        
        // Validate base
        if ( pBaseBrep != NULL )
        { 
            SmStatus eStat;
            SE( eStat = pBaseBrep->ValidateCounts( lBF, lBE, lBV, 0, 0, lBE, lBR, lBS ) );
            bRtn = ( eStat == SM_SUCCESS ) ? bRtn : SM_ERR;
            SM_DUMP( pBaseBrep );
        }
        else 
        { 
            SE_MSG( SM_ERR, _T( "Failed to rebuild base brep" ) );
            bRtn = SM_ERR; 
        }

        // Validate Feature
        if ( pFeatureBrep != NULL )
        {
            SmStatus eStat;
            SE( eStat = pFeatureBrep->ValidateCounts( lFF, lFE, lFV, 0, 0, lFE, lFR, lFS ) );
            bRtn = ( eStat == SM_SUCCESS ) ? bRtn : SM_ERR;
            SM_DUMP( pFeatureBrep );
        }
        else
        { 
            SE_MSG( SM_ERR, _T( "Failed to rebuild feature brep" ) );
            bRtn = SM_ERR; 
        }

        SmObjDelete sCleanB( pBaseBrep );
        SmObjsDelete<SmBrep*> sCleanF( &sFeatureBreps );

        lTestCase++;

#ifdef SM_DEBUG_CODE
        SmBoolean bDebugMe = FALSE;
        if ( smGet_DoGraphics() && bDebugMe )
        {
            smgfx_Erase();
            if (pOrigBrep) { pOrigBrep->Draw(TRUE); sm_GraphicsLoop(); }
            sm_GraphicsLoop();

            smgfx_Erase();
            if (pBaseBrep) { pBaseBrep->Draw(); sm_GraphicsLoop(); }
            sm_GraphicsLoop();

            smgfx_Erase();
            if (pFeatureBrep) { pFeatureBrep->Draw(); sm_GraphicsLoop(); }
            sm_GraphicsLoop();
        }
#endif
    }
} //end my_test_multi_feature

/*******************************************************
PURPOSE --- Test defeaturing and rebuilding multiple features 

USAGE NOTES ---
********************************************************/
SmStatus my_test_multi_feature()
{
    SmStatus bRtn = SM_SUCCESS;
    ULONG lTestCase = 0;
    SmContext sContext;

    while(TRUE)
    {
        SmTArray<SmEdge*> sAllEdges, sLoopEdges;
        SmTArray<SmFace*> sFaces, sFeatureFaces;
        SmTArray<ULONG>   sFeatureIDs1, sFeatureIDs2, sFeatureIDs3;
        SmTArray<ULONG>   sEdgeIDs;
        SmTArray<ULONG>   sFaceIDs;
        SmRebuildBrepType eBaseRB, eFeatRB;
        ULONG lBF, lBV, lBE, lFF, lFV, lFE, lBR = 2, lBS = 2, lFR = 2, lFS = 2;
        const TCHAR *pFileName;
        TCHAR sBuff[SM_TBLOCK_SIZE];

        // Read the file for defeature
        switch ( lTestCase )
        {
        case 0:
        {
            // Part
            pFileName = _T( "../../TestFiles/pt_TestFiles/Defeature/extrusion.smb" );
            // cylinder
            //lFaceID = 10;
            sFaceIDs.Add( 10 );
            sFeatureIDs1.Add( 8 );
            sFeatureIDs1.Add( 25 );
            //sEdgeIDs.Add( 8 ); sEdgeIDs.Add( 25 );

            // rectangle
            // face id 9
            sFaceIDs.Add( 9 );
            sFeatureIDs2.Add( 4 ); sFeatureIDs2.Add( 5 ); sFeatureIDs2.Add( 6 ); sFeatureIDs2.Add( 7 );
            sFeatureIDs2.Add( 17 ); sFeatureIDs2.Add( 19 ); sFeatureIDs2.Add( 21 ); sFeatureIDs2.Add( 23 );

            // Corner 
            // face id 4
            sFaceIDs.Add( 4 );
            sFeatureIDs3.Add( 2 ); sFeatureIDs3.Add( 12 ); sFeatureIDs3.Add( 13 ); sFeatureIDs3.Add( 14 );
            // base result
            lBF = 13, lBE = 27, lBV = 20;
            // base result
            lFF = 3, lFE = 3, lFV = 2;

            eBaseRB = SM_RB_EXTEND_SURFACE;
            eFeatRB = SM_RB_FROM_OTHER_BREP;
            break;
        }
        default:
            return bRtn;
        }

        SM_SPRINTF( sBuff, _T( "\n\nEntered my_test_multi_feature case: %ld"), lTestCase );
        MYPRINTF( sBuff );

        SmBrep *pOrigBrep = new( sContext ) SmBrep();
        pOrigBrep->ReadFromFile( sContext, pFileName );

        // Get feature faces and edges
        pOrigBrep->GetEdges( sAllEdges );
        pOrigBrep->GetFaces( sFaces );
        for ( ULONG ii = 0; ii < sFaceIDs.GetSize(); ii++ )
        { sFeatureFaces.Add(sFaces[sFaceIDs[ii]]); }
        sLoopEdges.ReSet();

        // Create list of edges for the features
        SmTArray<SmEdge*> sFeatureEdges1, sFeatureEdges2, sFeatureEdges3;
        for ( ULONG ii = 0; ii < sFeatureIDs1.GetSize(); ii++ )
        { sFeatureEdges1.Add( sAllEdges[sFeatureIDs1[ii]] ); }
        for ( ULONG ii = 0; ii < sFeatureIDs2.GetSize(); ii++ )
        { sFeatureEdges2.Add( sAllEdges[sFeatureIDs2[ii]] ); }
        for ( ULONG ii = 0; ii < sFeatureIDs3.GetSize(); ii++ )
        { sFeatureEdges3.Add( sAllEdges[sFeatureIDs3[ii]] ); }

        // Add each feature to the executive
        SmFeatureExecutive sDeExec( sContext, *pOrigBrep );
        sDeExec.AddFeature( sFeatureFaces[0], sFeatureEdges1, eBaseRB, eFeatRB );
        sDeExec.AddFeature( sFeatureFaces[1], sFeatureEdges2, eBaseRB, eFeatRB );
        sDeExec.AddFeature( sFeatureFaces[2], sFeatureEdges3, eBaseRB, eFeatRB );

        // SM_RB_EXTEND_SURFACE= builds from extended surfaces
        SmBrep *pBaseBrep = NULL, *pFeatureBrep = NULL;
        SmTArray<SmBrep*> sFeatureBreps;

        SE( sDeExec.DoDefeature( pBaseBrep, &sFeatureBreps ) );

        if(sFeatureBreps.GetSize() > 0)
        { pFeatureBrep = sFeatureBreps[0]; }
        
        // Validate base
        if ( pBaseBrep != NULL )
        { 
            SmStatus eStat;
            SE( eStat = pBaseBrep->ValidateCounts( lBF, lBE, lBV, 0, 0, lBE, lBR, lBS ) );
            bRtn = ( eStat == SM_SUCCESS ) ? bRtn : SM_ERR;
            SM_DUMP( pBaseBrep );
        }
        else 
        { 
            SE_MSG( SM_ERR, _T( "Failed to rebuild base brep" ) );
            bRtn = SM_ERR; 
        }

        // Validate Feature
        if ( pFeatureBrep != NULL )
        {
            SmStatus eStat;
            SE( eStat = pFeatureBrep->ValidateCounts( lFF, lFE, lFV, 0, 0, lFE, lFR, lFS ) );
            bRtn = ( eStat == SM_SUCCESS ) ? bRtn : SM_ERR;
            SM_DUMP( pFeatureBrep );
        }
        else
        { 
            SE_MSG( SM_ERR, _T( "Failed to rebuild feature brep" ) );
            bRtn = SM_ERR; 
        }

        SmObjDelete sCleanB( pBaseBrep );
        SmObjsDelete<SmBrep*> sCleanF( &sFeatureBreps );

        lTestCase++;

#ifdef SM_DEBUG_CODE
        SmBoolean bDebugMe = FALSE;
        if ( smGet_DoGraphics() && bDebugMe )
        {
            smgfx_Erase();
            if (pOrigBrep) { pOrigBrep->Draw(TRUE); sm_GraphicsLoop(); }
            sm_GraphicsLoop();

            smgfx_Erase();
            if (pBaseBrep) { pBaseBrep->Draw(); sm_GraphicsLoop(); }
            sm_GraphicsLoop();

            smgfx_Erase();
            if ( pFeatureBrep ) { pFeatureBrep->Draw(); sm_GraphicsLoop(); }
            sm_GraphicsLoop();
        }
#endif
    }
} //end my_test_multi_feature

/*******************************************************
PURPOSE --- Test defeaturing and rebuilding multiple features 

USAGE NOTES ---
********************************************************/
SmStatus my_test_multi_feature_adv()
{
    SmStatus bRtn = SM_SUCCESS;
    ULONG lTestCase = 0;
    SmContext sContext;

    while(TRUE)
    {
        SmTArray<SmEdge*> sAllEdges;
        SmTArray<SmFace*> sFaces, sFeatureFaces;
        SmTArray<ULONG> sFeatureIDs00, sFeatureIDs01, sFeatureIDs10, sFeatureIDs11, sFeatureIDs20;
        SmTArray<ULONG>   sEdgeIDs;
        SmTArray<ULONG>   sFaceIDs;
        ULONG lBF, lBV, lBE, lFF, lFV, lFE, lBR = 2, lBS = 2, lFR = 2, lFS = 2;
        const TCHAR *pFileName;
        TCHAR sBuff[SM_TBLOCK_SIZE];

        // Read the file for defeature
        switch ( lTestCase )
        {
        case 0:
        {
            // Part
            pFileName = _T( "../../TestFiles/pt_TestFiles/Defeature/extrusion.smb" );
            // cylinder
            //lFaceID = 10;
            sFaceIDs.Add( 10 );
            sFeatureIDs00.Add( 8 );
            sFeatureIDs01.Add( 25 );
            //sEdgeIDs.Add( 8 ); sEdgeIDs.Add( 25 );

            // rectangle
            // face id 9
            sFaceIDs.Add( 9 );
            sFeatureIDs10.Add( 4 ); sFeatureIDs10.Add( 5 ); sFeatureIDs10.Add( 6 ); sFeatureIDs10.Add( 7 );
            sFeatureIDs11.Add( 17 ); sFeatureIDs11.Add( 19 ); sFeatureIDs11.Add( 21 ); sFeatureIDs11.Add( 23 );

            // Corner 
            // face id 4
            sFaceIDs.Add( 4 );
            sFeatureIDs20.Add( 2 ); sFeatureIDs20.Add( 12 ); sFeatureIDs20.Add( 13 ); sFeatureIDs20.Add( 14 );
            // base result
            lBF = 13, lBE = 27, lBV = 20;
            // base result
            lFF = 3, lFE = 3, lFV = 2;

            break;
        }
        default:
            return bRtn;
        }

        SM_SPRINTF( sBuff, _T( "\n\nEntered my_test_multi_feature_adv case: %ld"), lTestCase );
        MYPRINTF( sBuff );

        SmBrep *pOrigBrep = new( sContext ) SmBrep();
        pOrigBrep->ReadFromFile( sContext, pFileName );

        // Get feature faces and edges
        pOrigBrep->GetEdges( sAllEdges );
        pOrigBrep->GetFaces( sFaces );

        for ( ULONG ii = 0; ii < sFaceIDs.GetSize(); ii++ )
        { sFeatureFaces.Add(sFaces[sFaceIDs[ii]]); }

        SmTArray<SmEdge*> sFeatureEdges00, sFeatureEdges01, sFeatureEdges10, sFeatureEdges11, sFeatureEdges20;
       
        // Convert from Edge indices to Edge ptrs
        for ( ULONG ii = 0; ii < sFeatureIDs00.GetSize(); ii++ )
        { sFeatureEdges00.Add( sAllEdges[sFeatureIDs00[ii]] ); }
        for ( ULONG ii = 0; ii < sFeatureIDs01.GetSize(); ii++ )
        { sFeatureEdges01.Add( sAllEdges[sFeatureIDs01[ii]] ); }
        for ( ULONG ii = 0; ii < sFeatureIDs10.GetSize(); ii++ )
        { sFeatureEdges10.Add( sAllEdges[sFeatureIDs10[ii]] ); }
        for ( ULONG ii = 0; ii < sFeatureIDs11.GetSize(); ii++ )
        { sFeatureEdges11.Add( sAllEdges[sFeatureIDs11[ii]] ); }
        for ( ULONG ii = 0; ii < sFeatureIDs20.GetSize(); ii++ )
        { sFeatureEdges20.Add( sAllEdges[sFeatureIDs20[ii]] ); }

        // Make feature loops from the lists of edges and intended rebuild method
        SmTArray<SmFeatureLoop*> sFLoops0, sFLoops1, sFLoops2;
        SmObjsDelete<SmFeatureLoop*> sClean0( &sFLoops0 ), sClean1( &sFLoops1 ), sClean2( &sFLoops2 );
        SmFeatureLoop *pFLoop00 = new SmFeatureLoop( sFeatureEdges00, SM_RB_EXTEND_SURFACE, SM_RB_FROM_OTHER_BREP );
        SmFeatureLoop *pFLoop01 = new SmFeatureLoop( sFeatureEdges01, SM_RB_EXTEND_SURFACE, SM_RB_FROM_OTHER_BREP );
        SmFeatureLoop *pFLoop10 = new SmFeatureLoop( sFeatureEdges10, SM_RB_EXTEND_SURFACE, SM_RB_FROM_OTHER_BREP );
        SmFeatureLoop *pFLoop11 = new SmFeatureLoop( sFeatureEdges11, SM_RB_EXTEND_SURFACE, SM_RB_FROM_OTHER_BREP );
        SmFeatureLoop *pFLoop20 = new SmFeatureLoop( sFeatureEdges20, SM_RB_EXTEND_SURFACE, SM_RB_FROM_OTHER_BREP );
        sFLoops0.Add( pFLoop00 );
        sFLoops0.Add( pFLoop01 );
        sFLoops1.Add( pFLoop10 );
        sFLoops1.Add( pFLoop11 );
        sFLoops2.Add( pFLoop20 );

        // Create the executive and add the features defined by the feature loops and a face
        SmFeatureExecutive sDeExec( sContext, *pOrigBrep );
        sDeExec.AddFeature( sFeatureFaces[0], sFLoops0 );
        sDeExec.AddFeature( sFeatureFaces[1], sFLoops1 );
        sDeExec.AddFeature( sFeatureFaces[2], sFLoops2 );

        // SM_RB_EXTEND_SURFACE= builds from extended surfaces
        SmBrep *pBaseBrep = NULL, *pFeatureBrep = NULL;
        SmTArray<SmBrep*> sFeatureBreps;

        SE( sDeExec.DoDefeature( pBaseBrep, &sFeatureBreps ) );

        if(sFeatureBreps.GetSize() > 0)
        { pFeatureBrep = sFeatureBreps[0]; }
        
        // Validate base
        if ( pBaseBrep != NULL )
        { 
            SmStatus eStat;
            SE( eStat = pBaseBrep->ValidateCounts( lBF, lBE, lBV, 0, 0, lBE, lBR, lBS ) );
            bRtn = ( eStat == SM_SUCCESS ) ? bRtn : SM_ERR;
            SM_DUMP( pBaseBrep );
        }
        else 
        { 
            SE_MSG( SM_ERR, _T( "Failed to rebuild base brep" ) );
            bRtn = SM_ERR; 
        }

        // Validate Feature
        if ( pFeatureBrep != NULL )
        {
            SmStatus eStat;
            SE( eStat = pFeatureBrep->ValidateCounts( lFF, lFE, lFV, 0, 0, lFE, lFR, lFS ) );
            bRtn = ( eStat == SM_SUCCESS ) ? bRtn : SM_ERR;
            SM_DUMP( pFeatureBrep );
        }
        else
        { 
            SE_MSG( SM_ERR, _T( "Failed to rebuild feature brep" ) );
            bRtn = SM_ERR; 
        }

        SmObjDelete sCleanB( pBaseBrep );
        SmObjsDelete<SmBrep*> sCleanF( &sFeatureBreps );

        lTestCase++;

#ifdef SM_DEBUG_CODE
        SmBoolean bDebugMe = FALSE;
        if ( smGet_DoGraphics() && bDebugMe )
        {
            smgfx_Erase();
            if (pOrigBrep) { pOrigBrep->Draw(TRUE); sm_GraphicsLoop(); }
            sm_GraphicsLoop();

            smgfx_Erase();
            if (pBaseBrep) { pBaseBrep->Draw(); sm_GraphicsLoop(); }
            sm_GraphicsLoop();

            smgfx_Erase();
            if (pFeatureBrep) { pFeatureBrep->Draw(); sm_GraphicsLoop(); }
            sm_GraphicsLoop();
        }
#endif
    }
} //end my_test_multi_feature_adv

/*******************************************************
PURPOSE --- Test defeaturing and rebuilding multiple features 

USAGE NOTES ---
********************************************************/
SmStatus my_test_multi_rebuilds()
{
    SmStatus bRtn = SM_SUCCESS;
    ULONG lTestCase = 0;
    SmContext sContext;

    while(TRUE)
    {
        SmTArray<SmEdge*> sAllEdges, sLoopEdges;
        SmTArray<SmFace*> sFaces, sFeatureFaces;
        SmTArray<ULONG> sFeatureIDs1, sFeatureIDs2, sFeatureIDs3;
        SmTArray<ULONG>   sEdgeIDs;
        SmTArray<ULONG>   sFaceIDs;
        SmRebuildBrepType eBaseRB, eFeatRB;
        ULONG lBF = 0, lBV = 0, lBE = 0, lFF = 0, lFV = 0, lFE = 0, lBR = 2, lBS = 2, lFR = 2, lFS = 2, lBLam = 0, lBMan = 0;
        const TCHAR *pFileName;
        TCHAR sBuff[SM_TBLOCK_SIZE];

        // Read the file for defeature
        switch ( lTestCase )
        {
        case 0:
        case 1:
        case 2:
        {
            // Part
            pFileName = _T( "../../TestFiles/pt_TestFiles/Defeature/extrusion.smb" );
            // cylinder
            //lFaceID = 10;
            sFaceIDs.Add( 10 );
            sFeatureIDs1.Add( 8 );
            sFeatureIDs1.Add( 25 );
            //sEdgeIDs.Add( 8 ); sEdgeIDs.Add( 25 );

            // rectangle
            // face id 9
            sFaceIDs.Add( 9 );
            sFeatureIDs2.Add( 4 ); sFeatureIDs2.Add( 5 ); sFeatureIDs2.Add( 6 ); sFeatureIDs2.Add( 7 );
            sFeatureIDs2.Add( 17 ); sFeatureIDs2.Add( 19 ); sFeatureIDs2.Add( 21 ); sFeatureIDs2.Add( 23 );

            // Corner 
            // face id 4
            sFaceIDs.Add( 4 );
            sFeatureIDs3.Add( 2 ); sFeatureIDs3.Add( 12 ); sFeatureIDs3.Add( 13 ); sFeatureIDs3.Add( 14 );

            eBaseRB = SM_RB_PLANAR_CAP;
            eFeatRB = SM_RB_PLANAR_CAP;
            break;
        }
        default:
            return bRtn;
        }

        SM_SPRINTF( sBuff, _T( "\n\nEntered my_test_multi_rebuild case: %ld"), lTestCase );
        MYPRINTF( sBuff );

        SmBrep *pOrigBrep = new( sContext ) SmBrep();
        pOrigBrep->ReadFromFile( sContext, pFileName );

        // Get feature faces and edges
        pOrigBrep->GetEdges( sAllEdges );
        pOrigBrep->GetFaces( sFaces );
        for ( ULONG ii = 0; ii < sFaceIDs.GetSize(); ii++ )
        { sFeatureFaces.Add(sFaces[sFaceIDs[ii]]); }
        sLoopEdges.ReSet();

        SmTArray<SmEdge*> sFeatureEdges1, sFeatureEdges2, sFeatureEdges3;
        for ( ULONG ii = 0; ii < sFeatureIDs1.GetSize(); ii++ )
        { sFeatureEdges1.Add( sAllEdges[sFeatureIDs1[ii]] ); }
        for ( ULONG ii = 0; ii < sFeatureIDs2.GetSize(); ii++ )
        { sFeatureEdges2.Add( sAllEdges[sFeatureIDs2[ii]] ); }
        for ( ULONG ii = 0; ii < sFeatureIDs3.GetSize(); ii++ )
        { sFeatureEdges3.Add( sAllEdges[sFeatureIDs3[ii]] ); }

        SmFeatureExecutive sDeExec( sContext, *pOrigBrep );
        sDeExec.AddFeature( sFeatureFaces[0], sFeatureEdges1, eBaseRB, eFeatRB );
        sDeExec.AddFeature( sFeatureFaces[1], sFeatureEdges2, eBaseRB, eFeatRB );
        sDeExec.AddFeature( sFeatureFaces[2], sFeatureEdges3, eBaseRB, eFeatRB );

        SmBrep *pBaseBrep = NULL, *pFeatureBrep = NULL;
        SmTArray<SmBrep*> sFeatureBreps;

        SE( sDeExec.DoDefeature( pBaseBrep, &sFeatureBreps ) );

        switch ( lTestCase )
        {
        case 0:
        {
            sDeExec.SetBaseRBType( SM_RB_EXTEND_SURFACE );
            sDeExec.SetFeatureRBType( SM_RB_FROM_OTHER_BREP );
            SE( sDeExec.DoDefeature( pBaseBrep, &sFeatureBreps ) );

            // base result
            lBF = 13, lBE = 27, lBV = 20; lBMan = 27;
            // base result
            lFF = 3, lFE = 3, lFV = 2;

            break;
        }
        case 1:
        {
            sDeExec.SetBaseRBType( SM_RB_NONE );
            sDeExec.SetFeatureRBType( SM_RB_NONE );
            SE( sDeExec.DoDefeature( pBaseBrep, &sFeatureBreps ) );

            // base result
            lBF = 9, lBE = 22, lBV = 18; lBLam = 4; lBMan = 18; lBR = 1; lBS = 1;
            // Feature result
            lFF = 3, lFE = 3, lFV = 2;

            break;
        }
        case 2:
        {
            sDeExec.RestoreFeatures();
            sDeExec.GetFeatureBreps(sFeatureBreps);

            // base result
            lBF = 10, lBE = 22, lBV = 18; lBMan = 22;
            // base result
            lFF = 3, lFE = 3, lFV = 2;
            break;
        }
        default:
            break;
        }

        if(sFeatureBreps.GetSize() > 0)
        { pFeatureBrep = sFeatureBreps[0]; }
        
        // Validate base
        if ( pBaseBrep != NULL )
        { 
            SmStatus eStat;
            SE( eStat = pBaseBrep->ValidateCounts( lBF, lBE, lBV, 0, lBLam, lBMan, lBR, lBS ) );
            bRtn = ( eStat == SM_SUCCESS ) ? bRtn : SM_ERR;
            SM_DUMP( pBaseBrep );
        }
        else 
        { 
            SE_MSG( SM_ERR, _T( "Failed to rebuild base brep" ) );
            bRtn = SM_ERR; 
        }

        // Validate Feature
        if ( pFeatureBrep != NULL )
        {
            SmStatus eStat;
            SE( eStat = pFeatureBrep->ValidateCounts( lFF, lFE, lFV, 0, 0, lFE, lFR, lFS ) );
            bRtn = ( eStat == SM_SUCCESS ) ? bRtn : SM_ERR;
            SM_DUMP( pFeatureBrep );
        }
        else
        { 
            SE_MSG( SM_ERR, _T( "Failed to rebuild feature brep" ) );
            bRtn = SM_ERR; 
        }

        SmObjDelete sCleanB( pBaseBrep );
        SmObjsDelete<SmBrep*> sCleanF( &sFeatureBreps );

        lTestCase++;

#ifdef SM_DEBUG_CODE
        SmBoolean bDebugMe = FALSE;
        if ( smGet_DoGraphics() && bDebugMe )
        {
            smgfx_Erase();
            if (pOrigBrep) { pOrigBrep->Draw(TRUE); sm_GraphicsLoop(); }
            sm_GraphicsLoop();

            smgfx_Erase();
            if (pBaseBrep) { pBaseBrep->Draw(); sm_GraphicsLoop(); }
            sm_GraphicsLoop();

            smgfx_Erase();
            if (pFeatureBrep) { pFeatureBrep->Draw(); sm_GraphicsLoop(); }
            sm_GraphicsLoop();
        }
#endif
    }
} //end my_test_multi_feature

/*******************************************************
PURPOSE --- Test defeaturing and SM_EXTEND_SURFACE rebuild 
    methods on features (as opposed to fillets)

USAGE NOTES ---
********************************************************/
SmStatus my_test_extended_surface()
{
    SmStatus bRtn = SM_SUCCESS;
    ULONG lTestCase = 0;
    SmContext sContext;

    while(TRUE)
    {
        SmTArray<SmEdge*> sAllEdges, sLoopEdges;
        SmTArray<SmFace*> sFaces, sFeatureFaces;
        SmTArray<ULONG>   sEdgeIDs;
        SmTArray<ULONG>   sFaceIDs;
        ULONG lBF, lBV, lBE, lFF, lFV, lFE, lBR = 2, lBS = 2, lFR = 2, lFS = 2;
        const TCHAR *pFileName;
        TCHAR sBuff[SM_TBLOCK_SIZE];

        // Read the file for defeature
        switch ( lTestCase )
        {
        case 0:
        {
            // Part
            pFileName = _T( "../../TestFiles/pt_TestFiles/Defeature/inset_cube.smb" ) ;
            // Feature face
            sFaceIDs.Add(7);
            // Edges surrounding feature
            sEdgeIDs.Add( 11 ); sEdgeIDs.Add( 12 ); sEdgeIDs.Add( 14 ); sEdgeIDs.Add( 15 ); sEdgeIDs.Add( 16 ); sEdgeIDs.Add( 17 );
            // base result
            lBF = 9, lBE = 21, lBV = 14;
            // base result
            lFF = 6, lFE = 12, lFV = 8;
            break;
        }
        case 1:
        {
            // Part
            pFileName = _T( "../../TestFiles/pt_TestFiles/Defeature/inset_cube_1x1fillet.smb" );
            // Feature face
            sFaceIDs.Add(7);
            // Edges surrounding feature
            sEdgeIDs.Add( 13 );  sEdgeIDs.Add( 15 ); sEdgeIDs.Add( 16 ); sEdgeIDs.Add( 17 ); sEdgeIDs.Add( 18 ); sEdgeIDs.Add( 19 ); sEdgeIDs.Add( 20 );
            // base result
            lBF = 11, lBE = 26, lBV = 17;
            // base result
            lFF = 7, lFE = 15, lFV = 10;
            break;
        }
        case 2:
        {
            // Part
            pFileName = _T( "../../TestFiles/pt_TestFiles/Defeature/inset_cube_1x2fillet.smb" );
            // Feature face
            sFaceIDs.Add(8);
            // Edges surrounding feature
            sEdgeIDs.Add( 18 ); sEdgeIDs.Add( 19 ); sEdgeIDs.Add( 20 ); sEdgeIDs.Add( 21 ); sEdgeIDs.Add( 22 ); sEdgeIDs.Add( 23 );
            sEdgeIDs.Add( 14 ); sEdgeIDs.Add( 16 );
            // base result
            lBF = 13, lBE = 30, lBV = 19;
            // base result
            lFF = 8, lFE = 17, lFV = 11;
            break;
        }
        case 3:
        {
            // Part
            pFileName = _T( "../../TestFiles/pt_TestFiles/Defeature/inset_cube_1x3fillet.smb" );
            // Feature face
            sFaceIDs.Add(11);
            // Edges surrounding feature
            sEdgeIDs.Add( 18 ); sEdgeIDs.Add( 19 ); sEdgeIDs.Add( 20 ); sEdgeIDs.Add( 22 ); sEdgeIDs.Add( 23 );
            sEdgeIDs.Add( 24 ); sEdgeIDs.Add( 16 ); sEdgeIDs.Add( 25 ); sEdgeIDs.Add( 26 );
            // base result
            lBF = 15, lBE = 36, lBV = 23;
            // base result
            lFF = 9, lFE = 21, lFV = 14;
            break;
        }
        case 4:
        {
            // Part
            pFileName = _T( "../../TestFiles/pt_TestFiles/Defeature/inset_cube_3x1fillet.smb" );
            // Feature face
            sFaceIDs.Add(11);
            // Edges surrounding feature
            sEdgeIDs.Add( 16 ); sEdgeIDs.Add( 17 ); sEdgeIDs.Add( 18 ); sEdgeIDs.Add( 20 );
            sEdgeIDs.Add( 21 ); sEdgeIDs.Add( 23 ); sEdgeIDs.Add( 24 ); sEdgeIDs.Add( 25 ); sEdgeIDs.Add( 26 );
            // base result
            lBF = 15, lBE = 36, lBV = 23;
            // base result
            lFF = 9, lFE = 21, lFV = 14;
            break;
        }
        case 5:
        {
            // Part
            pFileName = _T( "../../TestFiles/pt_TestFiles/Defeature/flathead_screw_top.smb" );
            // Feature face
            sFaceIDs.Add(4);
            // Edges surrounding feature
            sEdgeIDs.Add( 4 ); sEdgeIDs.Add( 5 ); ; sEdgeIDs.Add( 6 ); sEdgeIDs.Add( 7 ); sEdgeIDs.Add( 8 );
            sEdgeIDs.Add( 2 ); sEdgeIDs.Add( 10 ); sEdgeIDs.Add( 11 ); sEdgeIDs.Add( 12 );
            // base result
            lBF = 8, lBE = 17, lBV = 11;
            // base result
            lFF = 7, lFE = 15, lFV = 10;
            break;
        }
        case 6:
        {
            // Part
            pFileName = _T( "../../TestFiles/pt_TestFiles/Defeature/brep1.smb" );
            // Feature Face
            sFaceIDs.Add(13);
            // Edges surrounding feature
            sEdgeIDs.Add( 26 ); sEdgeIDs.Add( 25 ); sEdgeIDs.Add( 24 ); sEdgeIDs.Add( 23 ); sEdgeIDs.Add( 28 );
            sEdgeIDs.Add( 16 ); sEdgeIDs.Add( 15 ); sEdgeIDs.Add( 14 ); sEdgeIDs.Add( 13 ); sEdgeIDs.Add( 36 );
            // base result
            lBF = 12, lBE = 32, lBV = 22;
            // base result
            lFF = 10, lFE = 24, lFV = 16;
            break;
        }
        case 7:
        {
            // Part
            pFileName = _T( "../../TestFiles/pt_TestFiles/Defeature/LShape_I.smb" );
            // Feature Face
            sFaceIDs.Add(3);
            // Edges surrounding feature
            sEdgeIDs.Add( 1 ); sEdgeIDs.Add( 2 ); sEdgeIDs.Add( 8 ); sEdgeIDs.Add( 13 ); sEdgeIDs.Add( 21 );
            sEdgeIDs.Add( 30 ); sEdgeIDs.Add( 31 ); sEdgeIDs.Add( 32 ); sEdgeIDs.Add( 37 ); sEdgeIDs.Add( 59 );
            // base result
            lBF = 25, lBE = 62, lBV = 40;
            // base result
            lFF = 12, lFE = 28, lFV = 18;

            break;
        }
        case 8:
        {
            // Part
            pFileName = _T( "../../TestFiles/pt_TestFiles/Defeature/LShape_II.smb" );
            // Feature Face
            sFaceIDs.Add(8);
            // Edges surrounding feature
            sEdgeIDs.Add( 1 ); sEdgeIDs.Add( 2 ); sEdgeIDs.Add( 3 ); sEdgeIDs.Add( 8 );
            sEdgeIDs.Add( 13 ); sEdgeIDs.Add( 17 ); sEdgeIDs.Add( 33 ); sEdgeIDs.Add( 37 );
            // base result
            lBF = 22, lBE = 55, lBV = 36;
            // base result
            lFF = 15, lFE = 33, lFV = 20;

            break;
        }
        case 9:
        {
            // Part
            pFileName = _T( "../../TestFiles/pt_TestFiles/Defeature/Platte.smb" );
            // Feature Face
            sFaceIDs.Add(80);
            // Edges surrounding feature
            sEdgeIDs.Add( 57 ); sEdgeIDs.Add( 58 ); sEdgeIDs.Add( 123 ); sEdgeIDs.Add( 124 ); sEdgeIDs.Add( 125 );
            sEdgeIDs.Add( 126 ); sEdgeIDs.Add( 216 ); sEdgeIDs.Add( 217 ); sEdgeIDs.Add( 218 ); sEdgeIDs.Add( 219 );
            // base result
            lBF = 115, lBE = 312, lBV = 210, lBR = 6, lBS = 10;
            // base result
            lFF = 11, lFE = 27, lFV = 18;

            break;
        }
        default:
            return bRtn;
        }

        SM_SPRINTF( sBuff, _T( "\n\nEntered my_test_extended_surface case: %ld"), lTestCase );
        MYPRINTF( sBuff );

        SmBrep *pOrigBrep = new( sContext ) SmBrep();
        pOrigBrep->ReadFromFile( sContext, pFileName );

        // Get feature faces and edges
        pOrigBrep->GetEdges( sAllEdges );
        pOrigBrep->GetFaces( sFaces );
        for ( ULONG ii = 0; ii < sFaceIDs.GetSize(); ii++ )
        { sFeatureFaces.Add(sFaces[sFaceIDs[ii]]); }
        sLoopEdges.ReSet();

        for ( ULONG ii = 0; ii < sEdgeIDs.GetSize(); ii++ )
        {
            SmEdge* pEdge = sAllEdges[sEdgeIDs[ii]];
            sLoopEdges.Add( pEdge );
        }

        SmFeatureExecutive sDeExec( sContext, *pOrigBrep );
        sDeExec.AddFeature(sFeatureFaces[0], sLoopEdges );

        // SM_RB_EXTEND_SURFACE= builds from extended surfaces
        SmBrep *pBaseBrep = NULL, *pFeatureBrep = NULL;
        SmTArray<SmBrep*> sFeatureBreps;

        SE( sDeExec.DoDefeature( pBaseBrep, &sFeatureBreps ));

        if(sFeatureBreps.GetSize() > 0)
        { pFeatureBrep = sFeatureBreps[0]; }
        
        // Validate base
        if ( pBaseBrep != NULL )
        { 
            SmStatus eStat;
            SE( eStat = pBaseBrep->ValidateCounts( lBF, lBE, lBV, 0, 0, lBE, lBR, lBS ) );
            bRtn = ( eStat == SM_SUCCESS ) ? bRtn : SM_ERR;
            SM_DUMP( pBaseBrep );
        }
        else 
        { 
            SE_MSG( SM_ERR, _T( "Failed to rebuild base brep" ) );
            bRtn = SM_ERR; 
        }

        // Validate Feature
        if ( pFeatureBrep != NULL )
        {
            SmStatus eStat;
            SE( eStat = pFeatureBrep->ValidateCounts( lFF, lFE, lFV, 0, 0, lFE, lFR, lFS ) );
            bRtn = ( eStat == SM_SUCCESS ) ? bRtn : SM_ERR;
            SM_DUMP( pFeatureBrep );
        }
        else
        { 
            SE_MSG( SM_ERR, _T( "Failed to rebuild feature brep" ) );
            bRtn = SM_ERR; 
        }

        SmObjDelete sCleanB( pBaseBrep );
        SmObjsDelete<SmBrep*> sCleanF( &sFeatureBreps );

        lTestCase++;

#ifdef SM_DEBUG_CODE
        SmBoolean bDebugMe = FALSE;
        if ( smGet_DoGraphics() && bDebugMe )
        {
            smgfx_Erase();
            if (pOrigBrep) { pOrigBrep->Draw(TRUE); sm_GraphicsLoop(); }
            sm_GraphicsLoop();

            smgfx_Erase();
            if (pBaseBrep) { pBaseBrep->Draw(); sm_GraphicsLoop(); }
            sm_GraphicsLoop();

            smgfx_Erase();
            if (pFeatureBrep) { pFeatureBrep->Draw(); sm_GraphicsLoop(); }
            sm_GraphicsLoop();
        }
#endif
    }
} //end my_test_defeature

/*******************************************************
PURPOSE --- 

USAGE NOTES ---
********************************************************/
PT_EXPORT SmStatus my_test_feature
( )   // in : one of
{
    MYPRINTF( _T( "\nEntered my_test_feature" ) );
    SmStatus eRtn = SM_SUCCESS;
    SmStatus eStat;

    SE_MSG( eStat = my_test_extended_surface(), _T( "my_test_extended_surface FAILS" ) );
    eRtn = ( eStat == SM_SUCCESS ) ? eRtn : SM_ERR;

    SE_MSG( eStat = my_test_multi_feature(), _T( "my_test_multi_feature FAILS" ) );
    eRtn = ( eStat == SM_SUCCESS ) ? eRtn : SM_ERR;

    SE_MSG( eStat = my_test_multi_feature_adv(), _T( "my_test_multi_feature_adv FAILS" ) );
    eRtn = ( eStat == SM_SUCCESS ) ? eRtn : SM_ERR;

    SE_MSG( eStat = my_test_multi_rebuilds(), _T( "my_test_multi_rebuilds FAILS" ) );
    eRtn = ( eStat == SM_SUCCESS ) ? eRtn : SM_ERR;

    SE_MSG( eStat = my_test_add_by_faces(), _T( "my_test_add_by_faces FAILS" ) );
    eRtn = ( eStat == SM_SUCCESS ) ? eRtn : SM_ERR;
        
    SE_MSG( eStat = my_test_rebuild_fillets(), _T( "my_test_rebuild_fillets() FAILS" ) );
    eRtn = ( eStat == SM_SUCCESS ) ? eRtn : SM_ERR;

    SER_MSG( eRtn, _T( "******** FAILURE IN [my_test_feature] ********" ) );
    return eRtn;
}
