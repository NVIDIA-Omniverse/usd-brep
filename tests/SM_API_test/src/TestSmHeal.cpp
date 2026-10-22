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
#include <SmApiPrimitives.h>
#include <SmApiHeal.h>
#include <SmExtent3d.h>
#include <SmHashTable.h>


//*************************************************************************
// 
//*************************************************************************

SmStatus TestSmHealBrep()
{
    SmApiCreateContext();

    SmVector3d sOrigin(0.0, 0.0, 0.0);
    SmBrep* pBox = NULL;
    SmApiCreateBox(sOrigin, 10, 10, 10, pBox);
    if (!pBox)
        return SM_ERR;

    SmStatus stat = SmApiHealBrep(pBox);

    delete pBox;
    return stat;
}


//*************************************************************************
// Healer's vertex voxel model: coincident pairs behind the bucket head are
// found, and the search does not overwrite stored points.
//*************************************************************************

SmStatus TestSmHealVoxelCoincidentPairs()
{
    SmApiCreateContext();

    int aItems[4] = {0, 1, 2, 3};
    SmPoint3d sMin(0.0, 0.0, 0.0);
    SmPoint3d sMax(10.0, 10.0, 10.0);
    SmExtent3d sBBox(sMin, sMax);

    // one voxel, reserve enough items that the arrays never grow
    SmObjsInVoxels<int*> sVoxels(1, 1, 1, &sBBox, 8);

    // bucket order is A, C, B: A is the head and is not coincident with B or C
    SmPoint3d sPointA(1.0, 1.0, 1.0);
    SmPoint3d sPointB(5.0, 5.0, 5.0);
    SmPoint3d sPointC(5.0, 5.0, 5.0 + 1.0e-9);
    sVoxels.Add(&aItems[0], sPointA, 1.0e-6);
    sVoxels.Add(&aItems[1], sPointB, 1.0e-6);
    sVoxels.Add(&aItems[2], sPointC, 1.0e-6);

    if (!sVoxels.AssertValid())
        return SM_ERR;

    SmTArray<SmPointItem<int*>*> sCoin1;
    SmTArray<SmPointItem<int*>*> sCoin2;
    sVoxels.FindCoincidentPairs(sCoin1, sCoin2);

    // exactly the B-C pair
    if (sCoin1.GetSize() != 1 || sCoin2.GetSize() != 1)
        return SM_ERR;
    SmPointItem<int*>* pB = sVoxels.GetPointItem(1);
    SmPointItem<int*>* pC = sVoxels.GetPointItem(2);
    if (!((sCoin1[0] == pB && sCoin2[0] == pC) || (sCoin1[0] == pC && sCoin2[0] == pB)))
        return SM_ERR;

    // the search must not modify stored points
    if (pB->m_sTgtPoint3d.DistanceBetween(sPointB) != 0.0 || pC->m_sTgtPoint3d.DistanceBetween(sPointC) != 0.0)
        return SM_ERR;

    // incremental search: a new item coincident with the old bucket head is found
    ULONG lLimitIndx = sVoxels.GetSize();
    SmPoint3d sPointD(1.0, 1.0, 1.0 + 1.0e-9);
    sVoxels.Add(&aItems[3], sPointD, 1.0e-6);
    sVoxels.FindCoincidentPairs(sCoin1, sCoin2, &lLimitIndx);
    if (sCoin1.GetSize() != 1)
        return SM_ERR;
    SmPointItem<int*>* pA = sVoxels.GetPointItem(0);
    SmPointItem<int*>* pD = sVoxels.GetPointItem(lLimitIndx);
    if (!((sCoin1[0] == pA && sCoin2[0] == pD) || (sCoin1[0] == pD && sCoin2[0] == pA)))
        return SM_ERR;

    return SM_SUCCESS;
}


SmStatus TestSmHeal()
{
    return TestSmHealBrep();
}


//*************************************************************************
// Healer's vertex voxel model: growing past the item reserve keeps bucket
// items pointing at the live PointItems.
//*************************************************************************

SmStatus TestSmHealVoxelGrowItems()
{
    SmApiCreateContext();

    int iItem = 0;
    SmPoint3d sMin(0.0, 0.0, 0.0);
    SmPoint3d sMax(10.0, 10.0, 10.0);
    SmExtent3d sBBox;
    sBBox.AddPoint3d(sMin);
    sBBox.AddPoint3d(sMax);

    // one voxel, tiny reserve so the adds below grow the item arrays
    SmObjsInVoxels<int*> sVoxels(1, 1, 1, &sBBox, 1);

    // one coincident pair, then distinct far points to force several IncMaxItemCount calls
    SmPoint3d sPointA(1.0, 1.0, 1.0);
    SmPoint3d sPointB(1.0, 1.0, 1.0 + 1.0e-9);
    sVoxels.Add(&iItem, sPointA, 1.0e-6);
    sVoxels.Add(&iItem, sPointB, 1.0e-6);
    for (ULONG ii = 0; ii < 200; ii++)
    {
        SmPoint3d sPad(9.0, 9.0, 0.01 * ii);
        sVoxels.Add(&iItem, sPad, 1.0e-6);
    }

    if (!sVoxels.AssertValid())
        return SM_ERR;

    // the pair is reported with pointers into the live PointItem array
    SmTArray<SmPointItem<int*>*> sCoin1;
    SmTArray<SmPointItem<int*>*> sCoin2;
    sVoxels.FindCoincidentPairs(sCoin1, sCoin2);
    if (sCoin1.GetSize() != 1 || sCoin2.GetSize() != 1)
        return SM_ERR;
    SmPointItem<int*>* pA = sVoxels.GetPointItem(0);
    SmPointItem<int*>* pB = sVoxels.GetPointItem(1);
    if (!((sCoin1[0] == pA && sCoin2[0] == pB) || (sCoin1[0] == pB && sCoin2[0] == pA)))
        return SM_ERR;

    return SM_SUCCESS;
}
