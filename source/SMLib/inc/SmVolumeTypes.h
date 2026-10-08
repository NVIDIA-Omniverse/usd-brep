// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*******************************************************************//**
* FILE NAME --- SmSurfTypes.h
* PURPOSE: Declaration of surface and volume types.
**********************************************************************/

#ifndef __SMVOLUME_TYPES_H__
#define __SMVOLUME_TYPES_H__

#ifndef __SMOS_TYPES_H__
#include <SmTypes.h>
#endif

#ifndef __SMTARRAY_H__
#include <SmTArray.h>
#endif

/*******************************************************************//**
PURPOSE: If there are any - this is where enums used by the
    SmVolume class hierarchy will be declared
    on a surface

NOTES: 
***********************************************************************/
//      enum SmExampleEnumType 
//      {
//        SM_EE_UNKNOWN,    // Value not yet set
//      
//        SM_EE_OPT1,       // option 1
//        SM_EE_OPT2,       // option 2 
//      
//      } ;

// predeclarations of all SmVolume classes
class SmVolume ;
class SmBSplineVolume ;
class SmTransform ;
class SmBendVolume ;
class SmUnbendVolume ;

// type declarations of all SmVolume classes                                    
#define SmVolume_TYPE               (VOL_BASE_TYPE  +  1)
#define SmBSplineVolume_TYPE        (VOL_BASE_TYPE  +  4)
#define SmBendVolume_TYPE           (VOL_BASE_TYPE  +  6)
#define SmTransform_TYPE            (VOL_BASE_TYPE  +  8)
#define SmUnbendVolume_TYPE         (VOL_BASE_TYPE  + 10)
#define SmTwistVolume_TYPE          (VOL_BASE_TYPE  + 11)
#define SmBendNoStrain_TYPE         (VOL_BASE_TYPE  + 12)    // GWC: this one is not released yet
#define SmVolInVolume_TYPE          (VOL_BASE_TYPE  + 14)    // GWC: this one is obsolete

#endif // __SMVOLUME_TYPES_H__

