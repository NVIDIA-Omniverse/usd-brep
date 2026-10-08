// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/****************************************************************************************/
/* FrameBasic.h: Basic Function Declarations for NL_CPOLYGON, NL_CNET, NL_CMESH objects */
/****************************************************************************************/

#ifndef _FRAMEBASIC_H
#define _FRAMEBASIC_H

/******************************/
/* NL_CPOLYGON Utility routines  */
/******************************/

GW_EXPORT NL_CPOLYGON *N_AllocCPolygon( NL_STACKS * );
GW_EXPORT NL_CPOLYGON *N_AllocCPolygonAndArray( NL_INDEX, NL_STACKS * );
GW_EXPORT NL_VOID N_FreeCPolygon( NL_CPOLYGON *, NL_STACKS * );
GW_EXPORT NL_VOID N_CPolygonGetCPts( NL_CPOLYGON *, NL_INDEX *, NL_CPOINT ** );
GW_EXPORT NL_VOID N_CPolygonFromCPts( NL_CPOLYGON *, NL_CPOINT *, NL_INDEX );
GW_EXPORT NL_FLAG N_CPolygonFromCPtCoords( NL_CPOLYGON *, NL_REAL *, NL_REAL *, NL_REAL *, NL_REAL *, NL_INDEX, NL_STACKS * );
GW_EXPORT NL_EPOLYGON *N_AllocPtPolygonStruct( NL_STACKS * );
GW_EXPORT NL_EPOLYGON *N_AllocPtPolygon( NL_INDEX, NL_STACKS * );
GW_EXPORT NL_EPOLYGON ** N_Alloc1dPtPolygon( NL_INDEX, NL_INDEX, NL_STACKS * );
GW_EXPORT NL_EPOLYGON ** N_Alloc1dPtPolygonPtrs( NL_INDEX, NL_STACKS * );

/******************************/
/* NL_EPOLYGON Utility routines  */
/******************************/

GW_EXPORT NL_FLAG N_IsectLinePolygon( NL_EPOLYGON *, NL_POINT, NL_POINT, NL_INDEX, NL_POINT *, NL_INDEX *, NL_FLAG * );
GW_EXPORT NL_VOID N_PolygonGetBBox( NL_EPOLYGON *, NL_MINMAXBOX * );
GW_EXPORT NL_FLAG N_PolygonGetClosestLegIndex( NL_EPOLYGON *, NL_POINT, NL_POINT *, NL_INDEX *, NL_REAL *, NL_FLAG * );
GW_EXPORT NL_VOID N_EPolygonFromPts( NL_EPOLYGON *, NL_INDEX, NL_POINT * );
GW_EXPORT NL_VOID N_EPolygonGetPts( NL_EPOLYGON *, NL_INDEX *, NL_POINT ** );
GW_EXPORT NL_FLAG N_PtsAreInPolygon( NL_EPOLYGON *, NL_POINT **, NL_INDEX, NL_INDEX, NL_BOOLEAN ** );
GW_EXPORT NL_FLAG N_PtsAreInPolygonGravityField( NL_EPOLYGON *, NL_POINT **, NL_INDEX, NL_INDEX, NL_REAL, NL_REAL ** );
GW_EXPORT NL_BOOLEAN N_EPolygonIsClosed( NL_EPOLYGON * );
GW_EXPORT NL_BOOLEAN N_PtIsContainedByEPolygon( NL_EPOLYGON *, NL_POINT );
GW_EXPORT NL_REAL N_PolygonGetArea( NL_EPOLYGON * );
GW_EXPORT NL_VOID N_PolygonSetIndex( NL_EPOLYGON *, NL_INDEX );
GW_EXPORT NL_FLAG N_PolygonOffset( NL_EPOLYGON *, NL_REAL, NL_EPOLYGON * );

/*******************************/
/* NL_CNET Utility routines       */
/*******************************/

GW_EXPORT NL_CNET *N_AllocCNet( NL_STACKS * );
GW_EXPORT NL_CNET *N_AllocCNetAndArrays( NL_INDEX, NL_INDEX, NL_STACKS * );
GW_EXPORT NL_VOID N_FreeCNet( NL_CNET *, NL_STACKS * );
GW_EXPORT NL_VOID N_CNetFromCPts( NL_CNET *, NL_CPOINT **, NL_INDEX, NL_INDEX );
GW_EXPORT NL_FLAG N_CNetFromCPtCoords( NL_CNET *, NL_REAL **, NL_REAL **, NL_REAL **, NL_REAL **, NL_INDEX, NL_INDEX, NL_STACKS * );
GW_EXPORT NL_VOID N_CNetGetCPts( NL_CNET *, NL_INDEX *, NL_INDEX *, NL_CPOINT *** );

/*******************************/
/* NL_ENET Utility routines       */
/*******************************/

GW_EXPORT NL_VOID N_ENetFromPts( NL_ENET *, NL_INDEX, NL_INDEX, NL_POINT ** );
GW_EXPORT NL_VOID N_ENetGetBBox( NL_ENET *, NL_MINMAXBOX * );
GW_EXPORT NL_VOID N_ENetGetPts( NL_ENET *, NL_INDEX *, NL_INDEX *, NL_POINT *** );
GW_EXPORT NL_BOOLEAN N_ENetIsClosed( NL_ENET *, NL_FLAG );
GW_EXPORT NL_FLAG N_NetGetClosestLegIndex( NL_ENET *, NL_POINT, NL_POINT *, NL_INDEX *, NL_INDEX *, NL_FLAG *, NL_REAL *, NL_FLAG * );

#endif /* _FRAMEBASIC_H */
