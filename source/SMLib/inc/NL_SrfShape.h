// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************/
/* Srfhape.h: Surface Shape related funcitons                         */
/**********************************************************************/

#ifndef _SRFSHAPE_H
#define _SRFSHAPE_H

GW_EXPORT NL_FLAG N_SrfShapeModifyCPts( NL_SURFACE *, NL_INDEX *, NL_REAL *, NL_INDEX, NL_PARAMETER, NL_PARAMETER, 
                                        NL_VECTOR, NL_REAL, NL_REAL *, NL_FLAG );

GW_EXPORT NL_FLAG N_SrfShapeModifyWeight( NL_SURFACE *, NL_INDEX, NL_INDEX, NL_PARAMETER, NL_PARAMETER, NL_REAL, 
                                          NL_REAL *, NL_REAL *, NL_REAL *, NL_FLAG );

GW_EXPORT NL_FLAG N_SrfShapeRemoveKnots( NL_SURFACE *, NL_REAL, NL_INDEX, NL_INDEX, NL_INDEX, NL_INDEX, NL_STACKS * );

GW_EXPORT NL_FLAG N_SrfShapeCreateRatBasis( NL_REAL, NL_DEGREE, NL_DEGREE, NL_SFUN *, NL_FLAG, NL_STACKS * );

GW_EXPORT NL_FLAG N_SrfShapeRegionWarp( NL_SURFACE *, NL_EPOLYGON *, NL_INDEX, NL_INDEX, NL_INDEX, NL_INDEX, NL_INDEX, 
                                        NL_INDEX, NL_SURFACE *, NL_VECTOR, NL_SFUN *, NL_REAL, NL_REAL, NL_FLAG, NL_FLAG, 
                                        NL_REAL **, NL_REAL **, NL_INDEX *, NL_INDEX *, NL_INDEX *, NL_INDEX *, 
                                        NL_BOOLEAN ***, NL_SURFACE *, NL_STACKS *, NL_STACKS * );

GW_EXPORT NL_FLAG N_SrfShapePolylineWarp( NL_SURFACE *, NL_EPOLYGON *, NL_INDEX, NL_INDEX, NL_CURVE *, NL_VECTOR, 
                                          NL_CFUN *, NL_REAL, NL_REAL, NL_REAL, NL_FLAG, NL_FLAG, NL_REAL ***, NL_INDEX *, 
                                          NL_INDEX *, NL_INDEX *, NL_INDEX *, NL_SURFACE *, NL_STACKS *, NL_STACKS * );

GW_EXPORT NL_FLAG N_SrfShapeFlatten( NL_SURFACE *, NL_EPOLYGON *, NL_INDEX, NL_INDEX, NL_INDEX, NL_INDEX, NL_INDEX, 
                                     NL_INDEX, NL_PLANE *, NL_VECTOR, NL_REAL, NL_FLAG, NL_FLAG, NL_FLAG, NL_SURFACE *, 
                                     NL_STACKS *, NL_STACKS * );

GW_EXPORT NL_FLAG N_SrfShapeAxialBend( NL_SURFACE *, NL_PARAMETER, NL_PARAMETER, NL_PARAMETER, NL_INDEX, NL_INDEX, 
                                       NL_REAL, NL_REAL, NL_REAL, NL_REAL, NL_FLAG, NL_FLAG, NL_FLAG, NL_REAL ***, 
                                       NL_INDEX *, NL_INDEX *, NL_SURFACE *, NL_STACKS *, NL_STACKS * );

GW_EXPORT NL_FLAG N_SrfShapeCentralBend( NL_SURFACE *, NL_EPOLYGON *, NL_PARAMETER, NL_PARAMETER, NL_INDEX, NL_INDEX, 
                                         NL_REAL, NL_REAL, NL_REAL, NL_REAL, NL_FLAG, NL_FLAG, NL_FLAG, NL_REAL **, 
                                         NL_REAL **, NL_BOOLEAN ***, NL_SURFACE *, NL_STACKS *, NL_STACKS * );

GW_EXPORT NL_FLAG N_SrfShapeConstraints( NL_SURFACE *, NL_PARAMETER *, NL_PARAMETER *, NL_INDEX, NL_INDEX, NL_INDEX *, NL_INDEX *, 
                                         NL_INDEX, NL_INDEX *, NL_INDEX *, NL_VECTOR **, NL_INDEX **, NL_INDEX **, NL_INDEX *, 
                                         NL_FLAG, NL_RMATRIX *, NL_INDEX **, NL_FLAG *, NL_STACKS * );

GW_EXPORT NL_FLAG N_SrfShapeAxialDeform( NL_SURFACE *, NL_CFUN *, NL_REAL, NL_FLAG, NL_FLAG, NL_FLAG );

GW_EXPORT NL_FLAG N_SrfShapeApproxPts( NL_SURFACE *, NL_POINT *, NL_INDEX, NL_INDEX, NL_INDEX, NL_INDEX, NL_INDEX, 
                                       NL_REAL, NL_SURFACE *, NL_FLAG *, NL_STACKS * );

GW_EXPORT NL_FLAG N_SrfShapeDerivConstraints( NL_SURFACE *, NL_PARAMETER *, NL_PARAMETER *, NL_INDEX, NL_INDEX, 
                                              NL_INDEX *, NL_INDEX *, NL_INDEX, NL_INDEX *, NL_INDEX *, NL_VECTOR **, 
                                              NL_INDEX **, NL_INDEX **, NL_INDEX *, NL_FLAG, NL_RMATRIX *, NL_INDEX **, NL_STACKS * );

GW_EXPORT NL_FLAG N_SrfShapeInterp( NL_SURFACE *, NL_POINT *, NL_INDEX, NL_INDEX, NL_INDEX, NL_INDEX, NL_INDEX, NL_FLAG, NL_SURFACE *, NL_STACKS * );

#endif /* _SRFSHAPE_H */
