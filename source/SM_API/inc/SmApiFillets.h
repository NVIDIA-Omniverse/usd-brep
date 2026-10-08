// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************
FILE NAME: SmFillets.h

PURPOSE: 
    Contains popular, high level "C" type functions for filleting.

GENERAL NOTES: 
    High level functions may assume some input parameters 
    for ease of use. For maximum flexibility, related functions 
    can be found in SmPrimitiveCreation
**********************************************************************/


#ifndef __SmApiFillets_H__
#define __SmApiFillets_H__

#ifndef __SmApiTypes_H__
#include <SmApiTypes.h>
#endif

SMAPI_EXPORT SmApiStatus SmApiChamferFillet(                                           
    SmBrep*  pBrep,                         ///< [in/out]: Pointer to brep                                              <br>
    double  dRadius                        ///< [in ]: Chamfer width across the chamfer face, not the setback along each face <br>
);

SMAPI_EXPORT SmApiStatus SmApiCircularFillet( 
    SmBrep*  pBrep,                         ///< [in/out]: Pointer to brep                                             <br>
    double  dRadius                        ///< [in ]: Radius of circular fillet                                      <br>
);

SMAPI_EXPORT SmApiStatus SmApiRemoveFillet( 
    SmBrep*  pBrep                         ///< [in/out]: Pointer to brep                                             <br>
);

#endif
