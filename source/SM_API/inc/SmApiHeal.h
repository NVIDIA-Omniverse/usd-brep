// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************
FILE NAME: SmHeal.h

PURPOSE: 
    Contains popular, high level "C" type functions that heal.

GENERAL NOTES: 
    High level functions may assume some input parameters 
    for ease of use. For maximum flexibility, related functions 
    can be found in SmPrimitiveCreation
**********************************************************************/


#ifndef __SmApiHeal_H__
#define __SmApiHeal_H__

#ifndef __SmApiTypes_H__
#include <SmApiTypes.h>
#endif

class SmBrep;

SMAPI_EXPORT SmApiStatus SmApiHealBrep( 
    SmBrep*  pBrep                         ///< [in/out]: Pointer to brep                                             <br>
);

#endif
