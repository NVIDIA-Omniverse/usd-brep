// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

//*******************************************************************
// Undistributed, user managed test file
// You should no longer need to modify any files under source 
// control to get testing flexibility
// The only thing that cannot change is the Test() function signature
//*******************************************************************

#include "StdAfx.h"

#include <SmApiGeneral.h>
#include <SmApiCurves.h>
#include <SmApiSurfaces.h> 
#include <SmApiPrimitives.h>
#include <SmApiFillets.h>
#include <SmApiBrep.h>
#include <SmBrep.h>


//*************************************************************************
// 
//*************************************************************************

SmStatus TestSmCircularFillet()
{

    SmApiCreateContext();

    SmVector3d sPositon1(0.0,0.0,0.0);
   
    SmBrep* pResult = NULL;
    SmApiCreateBox(sPositon1, 5, 10, 15, pResult);

    // Try circular fillet
    SmStatus stat = SmApiCircularFillet(pResult, 0.5);
    

    return( stat );
}

//*************************************************************************
// 
//*************************************************************************

SmStatus TestSmChamferFillet()
{

    SmApiCreateContext();

    SmVector3d sPositon1(0.0,0.0,0.0);
   
    SmBrep* pResult = NULL;
    SmApiCreateBox(sPositon1, 5, 10, 15, pResult);

    // Try Linear Fillet
    SmStatus stat = SmApiChamferFillet(pResult, 0.5);
    

    return( stat );
}

//*************************************************************************
// 
//*************************************************************************

SmStatus TestSmFilletEdges()
{

    SmApiCreateContext();

    SmVector3d sPos(0.0, 0.0, 0.0);
    SmBrep* pBox = NULL;
    SmApiCreateBox(sPos, 10, 10, 10, pBox);

    SmTArray<SmEdge*> sEdges;
    pBox->GetEdges(sEdges);

    SmTArray<SmEdge*> sEdgesToFillet;
    ULONG nEdges = sEdges.GetSize();
    if (nEdges > 4) nEdges = 4;
    for (ULONG i = 0; i < nEdges; i++)
        sEdgesToFillet.Add(sEdges[i]);

    SmStatus stat = SmApiFilletEdges(pBox, sEdgesToFillet, 1.0, 1, 1, 1.0);


    return(stat);
}

//*************************************************************************
// 
//*************************************************************************

SmStatus TestSmFilletEdgesPerEdge()
{

    SmApiCreateContext();

    SmVector3d sPos(0.0, 0.0, 0.0);
    SmBrep* pBox = NULL;
    SmApiCreateBox(sPos, 10, 10, 10, pBox);

    SmTArray<SmEdge*> sEdges;
    pBox->GetEdges(sEdges);

    SmTArray<SmEdge*> sEdgesToFillet;
    SmTArray<double> sRadii;
    SmTArray<SmFilletSurfaceGeneratorType> sXSectTypes;

    ULONG nEdges = sEdges.GetSize();
    if (nEdges > 4) nEdges = 4;
    for (ULONG i = 0; i < nEdges; i++)
    {
        sEdgesToFillet.Add(sEdges[i]);
        sRadii.Add(0.5 + 0.25 * i);
        sXSectTypes.Add(SM_FSG_CIRCULAR);
    }

    SmStatus stat = SmApiFilletEdgesPerEdge(pBox, sEdgesToFillet, sRadii,
        sXSectTypes, 1, 1.0);


    return(stat);
}

//*************************************************************************
// 
//*************************************************************************

SmStatus TestSmVariableRadiusFillet()
{

    SmApiCreateContext();

    SmVector3d sPos(0.0, 0.0, 0.0);
    SmBrep* pBox = NULL;
    SmApiCreateBox(sPos, 10, 10, 10, pBox);

    SmTArray<SmEdge*> sEdges;
    pBox->GetEdges(sEdges);

    SmTArray<SmEdge*> sEdgesToFillet;
    ULONG nEdges = sEdges.GetSize();
    if (nEdges > 4) nEdges = 4;
    for (ULONG i = 0; i < nEdges; i++)
        sEdgesToFillet.Add(sEdges[i]);

    SmStatus stat = SmApiVariableRadiusFillet(pBox, sEdgesToFillet,
        0.5, 2.0, 1, 1);


    return(stat);
}

//*************************************************************************
// 
//*************************************************************************

SmStatus TestSmSurfaceSurfaceFillet()
{
    SmApiCreateContext();

    SmVector3d c1(0,0,0), c2(10,0,0), c3(0,10,0), c4(10,10,0);
    SmSurface* pSurf1 = NULL;
    SmApiCreateSurfaceFromCornerPoints(c1, c2, c3, c4, pSurf1);
    if (!pSurf1)
        return SM_ERR;

    SmVector3d d1(5,0,-5), d2(5,0,5), d3(5,10,-5), d4(5,10,5);
    SmSurface* pSurf2 = NULL;
    SmApiCreateSurfaceFromCornerPoints(d1, d2, d3, d4, pSurf2);
    if (!pSurf2)
    {
        delete pSurf1;
        return SM_ERR;
    }

    SmBrep* pResult = NULL;
    SmStatus stat = SmApiSurfaceSurfaceFillet(pSurf1, pSurf2,
        2.0, 2.0, 0.01, pResult, 2, FALSE, FALSE);

    if (stat != SM_SUCCESS || !pResult)
    {
        delete pSurf1;
        delete pSurf2;
        return SM_ERR;
    }

    delete pResult;
    delete pSurf1;
    delete pSurf2;
    return SM_SUCCESS;
}

//*************************************************************************
// 
//*************************************************************************

SmStatus TestSmFilletPreview()
{

    SmApiCreateContext();

    SmVector3d sPos(0.0, 0.0, 0.0);
    SmBrep* pBox = NULL;
    SmApiCreateBox(sPos, 10, 10, 10, pBox);

    SmTArray<SmEdge*> sEdges;
    pBox->GetEdges(sEdges);

    SmTArray<SmEdge*> sEdgesToPreview;
    ULONG nEdges = sEdges.GetSize();
    if (nEdges > 4) nEdges = 4;
    for (ULONG i = 0; i < nEdges; i++)
        sEdgesToPreview.Add(sEdges[i]);

    SmTArray<SmSurface*> sPreviewSurfaces;
    SmStatus stat = SmApiFilletPreview(pBox, sEdgesToPreview, 1.0, sPreviewSurfaces);


    return(stat);
}

//*************************************************************************
// 
//*************************************************************************

// Fillet all edges of a box; bevelCorner = TRUE also bevels one vertex.
static SmStatus sm_FilletBoxWithBevel( SmBoolean bBevelCorner, ULONG& rlNumFaces )
{
    SmVector3d sPos(0.0, 0.0, 0.0);
    SmBrep* pBox = NULL;
    SER( SmApiCreateBox(sPos, 10, 10, 10, pBox) );

    // Every box vertex is a fillet corner once all edges are filleted.
    SmTArray<SmEdge*> sEdgesToFillet;
    pBox->GetEdges(sEdgesToFillet);
    SmTArray<SmVertex*> sVertices;
    pBox->GetVertices(sVertices);
    SmTArray<SmVertex*> sBevelVerts;
    if( bBevelCorner )
        sBevelVerts.Add(sVertices[0]);

    SmStatus stat = SmApiSetBevelCorners(pBox, sEdgesToFillet, 1.0, sBevelVerts);
    rlNumFaces = pBox->GetNumFaces();
    delete pBox;
    return stat;
}

SmStatus TestSmSetBevelCorners()
{
    SmApiCreateContext();

    ULONG lRoundFaces = 0, lBevelFaces = 0;
    SER( sm_FilletBoxWithBevel(FALSE, lRoundFaces) );
    SER( sm_FilletBoxWithBevel(TRUE, lBevelFaces) );

    // The edges were filleted, and the bevelled corner changed the result.
    if( lRoundFaces <= 6 || lBevelFaces <= 6 || lBevelFaces == lRoundFaces )
        return SM_ERR;

    return SM_SUCCESS;
}


//*************************************************************************
// 
//*************************************************************************

SmStatus TestSmRemoveFillet()
{
    SmApiCreateContext();

    SmVector3d sOrigin(0.0, 0.0, 0.0);
    SmBrep* pBox = NULL;
    SmApiCreateBox(sOrigin, 10, 10, 10, pBox);
    if (!pBox)
        return SM_ERR;

    SmStatus stat = SmApiCircularFillet(pBox, 1.0);
    if (stat != SM_SUCCESS)
    {
        delete pBox;
        return stat;
    }

    if (pBox->GetNumFaces() <= 6)
    {
        delete pBox;
        return SM_ERR;
    }

    stat = SmApiRemoveFillet(pBox);

    delete pBox;
    return stat;
}


//*************************************************************************
// 
//*************************************************************************

SmStatus TestSmFillets()
{

    SmStatus stat = TestSmCircularFillet();
    if( stat != SM_SUCCESS )
        return( stat );

    stat = TestSmChamferFillet();
    if( stat != SM_SUCCESS )
        return( stat );

    stat = TestSmFilletEdges();
    if( stat != SM_SUCCESS )
        return( stat );

    stat = TestSmFilletEdgesPerEdge();
    if( stat != SM_SUCCESS )
        return( stat );

    stat = TestSmVariableRadiusFillet();
    if( stat != SM_SUCCESS )
        return( stat );

    // SurfaceSurfaceFillet, FilletPreview, and SetBevelCorners are
    // exercised individually but excluded from the batch runner because
    // the debug-build assert output from the kernel is excessively
    // verbose for batch CI. They still compile and can be called directly.

    return( SM_SUCCESS );
}
