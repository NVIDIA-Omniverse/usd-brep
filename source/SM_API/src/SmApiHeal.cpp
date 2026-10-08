// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************
FILE NAME: SmHeal.cpp

PURPOSE: 
   Contains popular high level "C" type functions that heal.

GENERAL NOTES: 
    High level functions may assume some input parameters 
    for ease of use. For maximum flexibility, related functions 
    can be found in SMLib, etc
**********************************************************************/

#include "StdAfx.h"

#include "SmApiHeal.h"
#include <SmBrep.h>

/*******************************************************************//**
PURPOSE --- Heal a brep to fix common topological/geometric defects

NOTES ---   Runs the full healer sequence (SM_HO_ALL).

***********************************************************************/
SmApiStatus SmApiHealBrep
(
    SmBrep*  pBrep                         ///< [in/out]: Pointer to brep                                             <br>
)
{

    if( pBrep == NULL )
        return SM_ERR_INVALID_INPUT;

    SmStatus stat = pBrep->HealBrep( SM_HO_ALL );

    return stat;
}

