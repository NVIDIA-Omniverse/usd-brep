// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************/
/* Tessellate.h: Surface Shape related funcitons                      */
/**********************************************************************/

#ifndef _TESSELLATE_H
#define _TESSELLATE_H

GW_EXPORT NL_FLAG N_TessTrimmedSrf( NL_SURFACE *, NL_CURVE ***, NL_CURVE ****, NL_INDEX, NL_INDEX *, NL_INDEX *, NL_INDEX **, 
                                    NL_REAL, NL_REAL, NL_REAL, NL_PARAMETER **, NL_PARAMETER **, NL_INDEX *, NL_INDEX ***, NL_INDEX **, NL_STACKS * );

GW_EXPORT NL_FLAG N_TessGetTriangleVertices( NL_INDEX **, NL_INDEX, NL_INDEX **, NL_INDEX *, NL_STACKS * );

GW_EXPORT NL_FLAG N_TessGetTriangleEdges( NL_INDEX **, NL_INDEX *, NL_INDEX, NL_INDEX **, NL_INDEX **, NL_INDEX *, NL_STACKS * );

GW_EXPORT NL_FLAG N_TessGetTriangles( NL_INDEX **, NL_INDEX *, NL_INDEX, NL_INDEX **, NL_INDEX **, NL_INDEX **, NL_INDEX *, NL_STACKS * );

GW_EXPORT NL_FLAG N_TessSrfArea( NL_SURFACE *, NL_REAL *, NL_REAL *, NL_REAL * );

GW_EXPORT NL_FLAG N_TessPtsAdjPt( NL_INDEX **, NL_INDEX *, NL_INDEX, NL_INDEX, NL_INDEX **, NL_INDEX *, NL_STACKS * );

GW_EXPORT NL_FLAG N_TessEdgesFromPt( NL_INDEX **, NL_INDEX *, NL_INDEX, NL_INDEX, NL_INDEX **, NL_INDEX **, NL_INDEX *, NL_STACKS * );

GW_EXPORT NL_FLAG N_TessTrianglesAdjPt( NL_INDEX **, NL_INDEX *, NL_INDEX, NL_INDEX, NL_INDEX **, NL_INDEX **, 
                                     NL_INDEX **, NL_INDEX *, NL_STACKS * );

GW_EXPORT NL_FLAG N_TessIsOnEdge( NL_INDEX **, NL_INDEX *, NL_INDEX, NL_INDEX, NL_INDEX, NL_INDEX **, NL_INDEX **, 
                               NL_INDEX **, NL_INDEX *, NL_STACKS * );

GW_EXPORT NL_FLAG N_TessTrianglesAdjTriangle( NL_INDEX **, NL_INDEX *, NL_INDEX, NL_INDEX, NL_INDEX, NL_INDEX, 
                                     NL_INDEX **, NL_INDEX **, NL_INDEX **, NL_INDEX *, NL_STACKS * );

#endif /* _TESSELLATE_H */
