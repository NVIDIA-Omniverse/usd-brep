// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

//*******************************************************************
// Undistributed, user managed test file
// You should no longer need to modify any files under source 
// control to get testing flexibility
// The only thing that cannot change is the Test() function signature
//*******************************************************************

#include "StdAfx.h"

#include <cstdio>

#include <SmApiGeneral.h>
#include <SmApiImportExport.h>
#include <SmApiPrimitives.h>
#include <SmBrep.h>
#include <SmVector3d.h>

static SmStatus TestSmBrepFileRoundTrip
(
    const TCHAR * pFileName,
    const char  * pRemoveFileName,
    bool          bAscii
)
{
    SmVector3d sOrigin( 0.0, 0.0, 0.0 );
    SmBrep* pSource = NULL;
    SmStatus eStat = SmApiCreateBox( sOrigin, 5.0, 10.0, 20.0, pSource );
    if( eStat != SM_SUCCESS )
        return eStat;
    SmObjDelete sSourceCleanup( pSource );

    eStat = SmApiWriteBrepToFile( pSource, pFileName, bAscii );
    if( eStat != SM_SUCCESS )
        return eStat;

    SmBrep* pLoaded = NULL;
    eStat = SmApiReadBrepFromFile( pFileName, bAscii, false, pLoaded );
    std::remove( pRemoveFileName );
    if( eStat != SM_SUCCESS )
        return eStat;
    SmObjDelete sLoadedCleanup( pLoaded );

    SmTArray<SmFace*> sSourceFaces, sLoadedFaces;
    SmTArray<SmEdge*> sSourceEdges, sLoadedEdges;
    SmTArray<SmVertex*> sSourceVertices, sLoadedVertices;
    pSource->GetFaces( sSourceFaces );
    pLoaded->GetFaces( sLoadedFaces );
    pSource->GetEdges( sSourceEdges );
    pLoaded->GetEdges( sLoadedEdges );
    pSource->GetVertices( sSourceVertices );
    pLoaded->GetVertices( sLoadedVertices );

    if( sSourceFaces.GetSize() != sLoadedFaces.GetSize() ||
        sSourceEdges.GetSize() != sLoadedEdges.GetSize() ||
        sSourceVertices.GetSize() != sLoadedVertices.GetSize() )
        return SM_ERR;

    return SM_SUCCESS;
}

SmStatus TestSmImportExport()
{
    SmApiCreateContext();

    SmStatus eStat = TestSmBrepFileRoundTrip(
        _T("sm_api_native_io_ascii.smb"),
        "sm_api_native_io_ascii.smb",
        true );
    if( eStat != SM_SUCCESS )
        return eStat;

    return TestSmBrepFileRoundTrip(
        _T("sm_api_native_io_binary.smb"),
        "sm_api_native_io_binary.smb",
        false );
}
