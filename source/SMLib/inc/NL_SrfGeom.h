// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/***************************************************************************************/
/* SrfGeom.h: Geometry Processing Function Declarations that act on NL_SURFACE objects */
/***************************************************************************************/

#ifndef _SRFGEOM_H
#define _SRFGEOM_H

GW_EXPORT NL_FLAG N_SrfGetClosestPt( NL_SURFACE *, NL_POINT, NL_PARAMETER, NL_PARAMETER, NL_REAL, NL_REAL, 
                                     NL_PARAMETER *, NL_PARAMETER *, NL_POINT * );
GW_EXPORT NL_FLAG N_GetClosestPtOnSrf( NL_SURFACE *, NL_POINT, NL_PARAMETER, NL_PARAMETER, NL_REAL, NL_REAL, 
                                       NL_PARAMETER *, NL_PARAMETER *, NL_POINT *, NL_POINT ** );
GW_EXPORT NL_FLAG N_InvertTangentSrfCrv( NL_SURFACE *, NL_VECTOR, NL_PARAMETER, NL_PARAMETER, NL_VECTOR * );
GW_EXPORT NL_FLAG N_ConvertNurbsToPowerBasis( NL_SURFACE *, NL_FLAG, NL_SURFACE ****, NL_INDEX *, NL_INDEX *, NL_STACKS * );
GW_EXPORT NL_FLAG N_ConvertPiecesToNurbs( NL_SURFACE ***, NL_INDEX, NL_INDEX, NL_SURFACE *, NL_STACKS * );
GW_EXPORT NL_FLAG N_SrfGetAverageLen( NL_SURFACE *, NL_REAL *, NL_REAL *, NL_REAL * );
GW_EXPORT NL_FLAG N_ApproxSrfWithQuadSrf( NL_SURFACE *, NL_REAL, NL_FLAG, NL_FLAG, NL_POINT ***, NL_REAL **, 
                                          NL_REAL **, NL_INDEX *, NL_INDEX *, NL_STACKS * );

GW_EXPORT NL_FLAG N_SrfReparmRat( NL_SURFACE *, NL_REAL, NL_REAL, NL_REAL, NL_REAL, NL_FLAG, NL_SURFACE *, NL_STACKS * );
GW_EXPORT NL_FLAG N_SrfReparamArcLength( NL_SURFACE *, NL_REAL, NL_SURFACE *, NL_STACKS * );
GW_EXPORT NL_FLAG N_SrfProjectPts( NL_SURFACE *, NL_POINT *, NL_INDEX, NL_FLAG, NL_FLAG, NL_REAL, NL_POINT **, 
                                   NL_REAL **, NL_REAL **, NL_INDEX *, NL_STACKS * );

#endif /* _SRFGEOM_H */
