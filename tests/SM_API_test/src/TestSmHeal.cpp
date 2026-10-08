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


SmStatus TestSmHeal()
{
    return TestSmHealBrep();
}
