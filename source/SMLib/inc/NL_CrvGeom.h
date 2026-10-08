// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************************/
/* CrvGeom.h: Geometry Processing Function Declarations that act on NL_CURVE objects */
/**********************************************************************************/

#ifndef _CRVGEOM_H
#define _CRVGEOM_H

/* Geometry processing */

GW_EXPORT NL_FLAG N_CrvClosestPt( NL_CURVE *, NL_POINT, NL_PARAMETER, NL_REAL, NL_REAL, NL_PARAMETER *, NL_POINT * );
GW_EXPORT NL_FLAG N_CrvNurbsToPiecewise( NL_CURVE *, NL_FLAG, NL_CURVE ***, NL_INDEX *, NL_STACKS * );
GW_EXPORT NL_FLAG N_CrvPiecewiseToNurbs( NL_CURVE **, NL_INDEX, NL_CURVE *, NL_STACKS * );
GW_EXPORT NL_FLAG N_CrvArcLength( NL_CURVE *, NL_PARAMETER, NL_PARAMETER, NL_REAL, NL_FLAG, NL_REAL * );
GW_EXPORT NL_FLAG N_CrvClosestPtMultiple( NL_CURVE *, NL_POINT, NL_REAL, NL_REAL, NL_REAL, NL_GCPTEMP *, NL_PARAMETER *, NL_POINT *, NL_FLAG *, NL_STACKS * );
GW_EXPORT NL_FLAG N_ApproxCrvWithPolyline( NL_CURVE *, NL_REAL, NL_FLAG, NL_FLAG, NL_POINT **, NL_REAL **, NL_INDEX *, NL_STACKS * );
GW_EXPORT NL_FLAG N_CrvPercentageAlongPt( NL_CURVE *, NL_REAL, NL_REAL, NL_REAL, NL_FLAG, NL_REAL *, NL_REAL *, NL_POINT * );
GW_EXPORT NL_FLAG N_CrvProjectPts( NL_CURVE *, NL_POINT *, NL_INDEX, NL_REAL, NL_POINT **, NL_REAL **, NL_STACKS * );

#endif /* _CRVGEOM_H */
