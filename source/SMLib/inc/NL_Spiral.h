// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************/
/* SPIRAL.H: Function prototypes for curve and surface spiral         */
/**********************************************************************/

/* Check for inclusion */

#ifndef _SPIRAL_H
#define _SPIRAL_H

GW_EXPORT NL_FLAG N_CrvApproxSpiral( NL_REAL, NL_REAL, NL_REAL, NL_REAL, NL_INDEX, NL_INDEX *, NL_REAL *, NL_CURVE *, NL_STACKS * );

GW_EXPORT NL_FLAG N_CreateSpiralSrf( NL_CURVE *, NL_REAL, NL_REAL, NL_REAL, NL_REAL, NL_INDEX, NL_INDEX *, NL_REAL *, NL_SURFACE *, NL_STACKS * );

GW_EXPORT NL_FLAG N_CreateSpiralSrfTaper( NL_CURVE *Crv, NL_REAL height, NL_REAL RadiusTaper, NL_REAL NumTurns, NL_INDEX RorL, NL_INDEX *numPoints, NL_REAL *tol, NL_SURFACE *Srf, NL_STACKS *S );

#endif /* _SPIRAL_H */
