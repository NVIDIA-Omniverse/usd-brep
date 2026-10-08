// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*************************************************************************************/
/* SrfCommon.h: Common Surface Function Declarations that act on NL_SURFACE objects  */
/*************************************************************************************/

#ifndef _SRFCOMMON_H
#define _SRFCOMMON_H

GW_EXPORT NL_FLAG N_CreateSrfCornerPts( NL_POINT, NL_POINT, NL_POINT, NL_POINT, NL_SURFACE *, NL_STACKS * );
GW_EXPORT NL_FLAG N_CreateSrfExtrudeCrv( NL_CURVE *, NL_VECTOR, NL_REAL, NL_FLAG, NL_SURFACE *, NL_STACKS * );
GW_EXPORT NL_FLAG N_CreateCylCone( NL_POINT, NL_VECTOR, NL_VECTOR, NL_REAL, NL_REAL, NL_REAL, NL_REAL, NL_REAL, NL_FLAG, NL_FLAG, NL_SURFACE *, NL_STACKS * );
GW_EXPORT NL_FLAG N_CreateEllipticalCylCone( NL_POINT, NL_VECTOR, NL_VECTOR, NL_REAL, NL_REAL, NL_REAL, NL_REAL, NL_REAL, NL_REAL, NL_REAL, NL_FLAG, NL_FLAG, NL_SURFACE *, NL_STACKS * );
GW_EXPORT NL_FLAG N_CreateRuledSrf( NL_CURVE *, NL_CURVE *, NL_FLAG, NL_SURFACE *, NL_STACKS *, NL_STACKS * );
GW_EXPORT NL_FLAG N_CreateRuledSrfFromBoundaryCrvs( NL_CURVE *, NL_CURVE *, NL_FLAG, NL_SURFACE *, NL_STACKS * );
GW_EXPORT NL_FLAG N_CreateRevolvedSrf( NL_CURVE *, NL_POINT, NL_VECTOR, NL_REAL, NL_FLAG, NL_SURFACE *, NL_STACKS * );
GW_EXPORT NL_FLAG N_CreateSphere( NL_POINT, NL_REAL, NL_REAL, NL_REAL, NL_REAL, NL_FLAG, NL_FLAG, NL_SURFACE *, NL_STACKS * );
GW_EXPORT NL_FLAG N_CreateTorus( NL_POINT, NL_VECTOR, NL_POINT, NL_REAL, NL_REAL, NL_REAL, NL_REAL, NL_FLAG, NL_FLAG, NL_SURFACE *, NL_STACKS * );
GW_EXPORT NL_FLAG N_CreateParaboloid( NL_REAL, NL_REAL, NL_REAL, NL_REAL, NL_FLAG, NL_SURFACE *, NL_STACKS * );
GW_EXPORT NL_FLAG N_CreateEllipsoid( NL_REAL, NL_REAL, NL_REAL, NL_REAL, NL_REAL, NL_REAL, NL_FLAG, NL_FLAG, NL_SURFACE *, NL_STACKS * );
GW_EXPORT NL_FLAG N_CreateHyperboloid( NL_REAL, NL_REAL, NL_REAL, NL_REAL, NL_REAL, NL_FLAG, NL_SURFACE *, NL_STACKS * );
GW_EXPORT NL_FLAG N_CreateRuledSrfBetweenCrvAndPt( NL_CURVE *, NL_POINT, NL_FLAG, NL_SURFACE *, NL_STACKS *, NL_STACKS * );

#endif /* _SRFCOMMON_H */
