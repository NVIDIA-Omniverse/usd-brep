// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "StdAfx.h"

#include "SmApiImportExport.h"

#include <SmBrep.h>
#include <SmBrepData.h>
#include <SmContext.h>

SmApiStatus SmApiReadBrepFromFile
(
    const TCHAR * pFileName,
    SmBoolean     bAscii,
    SmBoolean     bRebuildUVTrimCurves,
    SmBrep      *& rpBrep
)
{
    rpBrep = NULL;
    if( pFileName == NULL )
        return SM_ERR_INVALID_INPUT;

    SmContext* pContext = SmApiGetOrCreateContext();
    SmBrep* pBrep = new (*pContext) SmBrep();
    if( pBrep == NULL )
        return SM_ERR_OUT_OF_MEMORY;

    SmObjDelete sCleanup( pBrep );
    SmStatus eStat = pBrep->ReadFromFile(
        *pContext,
        pFileName,
        bAscii ? SM_ASCII : SM_BINARY,
        bRebuildUVTrimCurves ? TRUE : FALSE );
    if( eStat != SM_SUCCESS )
        return eStat;

    sCleanup.Clear();
    rpBrep = pBrep;
    return SM_SUCCESS;
}

SmApiStatus SmApiWriteBrepToFile
(
    const SmBrep * pBrep,
    const TCHAR  * pFileName,
    SmBoolean      bAscii
)
{
    if( pBrep == NULL || pFileName == NULL )
        return SM_ERR_INVALID_INPUT;

    return pBrep->WriteToFile(
        pFileName,
        bAscii ? SM_ASCII : SM_BINARY,
        TRUE,
        FALSE,
        0.0 );
}

SmApiStatus SmApiReadPartFromFile
(
    const TCHAR          * pFileName,
    SmBoolean              bAscii,
    SmTArray<SmCurve*>   & rCurves,
    SmTArray<SmSurface*> & rSurfaces,
    SmTArray<long>       & rBooleanTreeNodes,
    SmTArray<SmBrep*>    & rBreps
)
{
    if( pFileName == NULL )
        return SM_ERR_INVALID_INPUT;
    if( rCurves.GetSize() != 0 ||
        rSurfaces.GetSize() != 0 ||
        rBooleanTreeNodes.GetSize() != 0 ||
        rBreps.GetSize() != 0 )
        return SM_ERR_INVALID_INPUT;

    SmTArray<SmCurve*> sCurves;
    SmTArray<SmSurface*> sSurfaces;
    SmTArray<long> sBooleanTreeNodes;
    SmTArray<SmBrep*> sBreps;
    SmObjsDelete<SmCurve*> sCurveCleanup( &sCurves );
    SmObjsDelete<SmSurface*> sSurfaceCleanup( &sSurfaces );
    SmObjsDelete<SmBrep*> sBrepCleanup( &sBreps );

    SmStatus eStat = SmBrepData::ReadPartFromFile(
        *SmApiGetOrCreateContext(),
        pFileName,
        sCurves,
        sSurfaces,
        sBooleanTreeNodes,
        sBreps,
        bAscii ? SM_ASCII : SM_BINARY );
    if( eStat != SM_SUCCESS )
        return eStat;

    if( rCurves.Append( sCurves ) != 0 ||
        rSurfaces.Append( sSurfaces ) != 0 ||
        rBooleanTreeNodes.Append( sBooleanTreeNodes ) != 0 ||
        rBreps.Append( sBreps ) != 0 )
    {
        rCurves.ReSet();
        rSurfaces.ReSet();
        rBooleanTreeNodes.ReSet();
        rBreps.ReSet();
        return SM_ERR_OUT_OF_MEMORY;
    }
    sCurveCleanup.Clear();
    sSurfaceCleanup.Clear();
    sBrepCleanup.Clear();
    return SM_SUCCESS;
}
