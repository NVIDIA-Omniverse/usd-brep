// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/******************************************************************//**
* FILE NAME --- SmNurbs.h
* PURPOSE: Header file for nurbs functions found in gw_nurbs.cpp
**********************************************************************/

#ifndef __Sm_Nurbs_H__
#define __Sm_Nurbs_H__

/* expose a small amount of the NLib interface */
typedef struct  nl_point     NL_POINT ;
typedef struct  cpoint       NL_CPOINT ; 
typedef struct  cpolygon     NL_CPOLYGON ;
typedef struct  cnet         NL_CNET ;
typedef struct  epolygon     NL_EPOLYGON ;
typedef struct  knotvector   NL_KNOTVECTOR ;
typedef struct  curve        NL_CURVE ;
typedef struct  surface      NL_SURFACE ;
typedef struct  interval     NL_INTERVAL ;
typedef struct  stacks       NL_STACKS ;                                
typedef struct  volume       NL_VOLUME ;    

/* Define new data types */
typedef  long        gw_INDEX;    /* Index variable    */
typedef  short       gw_FLAG;     /* Flag variable     */
typedef  double      gw_REAL;     /* Real variable     */

/* make synonyms for NLib datatypes */
typedef  NL_CPOINT      gw_CPOINT;
typedef  short       gw_DEGREE;
typedef  NL_CPOLYGON    gw_CPOLYGON;
typedef  NL_CNET        gw_CNET;
typedef  NL_EPOLYGON    gw_EPOLYGON ;
typedef  gw_REAL     gw_PARAMETER;
typedef  NL_KNOTVECTOR  gw_KNOTVECTOR;
typedef  NL_CURVE       gw_CURVE;
typedef  NL_SURFACE     gw_SURFACE;
typedef  NL_SURFACE     gw_surface;
typedef  NL_INTERVAL    gw_INTERVAL;
typedef  NL_STACKS      gw_STACKS;
typedef  NL_VOLUME      gw_VOLUME;

#endif // !__Sm_Nurbs_H__


