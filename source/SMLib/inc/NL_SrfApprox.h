// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************/
/* SrfApprox.h: Surface Fitting related funcitons                        */
/**********************************************************************/

#ifndef _SRFAPPROX_H
#define _SRFAPPROX_H

GW_EXPORT NL_FLAG N_ApproxConeWithSrf( NL_POINT, NL_VECTOR, NL_VECTOR, NL_REAL, NL_REAL, NL_REAL, NL_REAL, NL_REAL, 
                                      NL_DEGREE, NL_INDEX, NL_REAL, NL_SURFACE *, NL_STACKS * );

GW_EXPORT NL_FLAG N_ApproxNurbsWithNonRatSrf( NL_SURFACE *, NL_REAL, NL_DEGREE, NL_DEGREE, NL_FLAG, NL_SURFACE *, NL_STACKS * );

GW_EXPORT NL_FLAG N_ApproxRevolvedSrfWithSrf( NL_CURVE *, NL_POINT, NL_VECTOR, NL_REAL, NL_DEGREE, NL_INDEX, 
                                             NL_REAL, NL_SURFACE *, NL_STACKS * );

GW_EXPORT NL_FLAG N_ApproxSubSrfWithSrf( NL_CURVE **, NL_CURVE **, NL_CURVE **, NL_CURVE **, NL_SURFACE *, 
                                        NL_SURFACE *, NL_SFUN *, NL_SFUN *, NL_STACKS * );

GW_EXPORT NL_FLAG N_ApproxNormalSrfWithSrf( NL_SURFACE *, NL_REAL, NL_DEGREE, NL_DEGREE, NL_FLAG, NL_SURFACE *, NL_STACKS * );

GW_EXPORT NL_FLAG N_ApproxSphereWithSrf( NL_POINT, NL_REAL, NL_REAL, NL_REAL, NL_REAL, NL_DEGREE, NL_DEGREE, 
                                        NL_INDEX, NL_INDEX, NL_REAL, NL_SURFACE *, NL_STACKS * );

GW_EXPORT NL_FLAG N_ApproxTorusWithSrf( NL_POINT, NL_VECTOR, NL_POINT, NL_REAL, NL_REAL, NL_REAL, NL_REAL, NL_DEGREE, 
                                       NL_DEGREE, NL_INDEX, NL_INDEX, NL_REAL, NL_SURFACE *, NL_STACKS * );

#endif /* _SRFAPPROX_H */
