// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/********************************************************************************/
/* FrameAdv.h: Advanced Function Declarations for NL_CPOLYGON, NL_CNET, NL_CMESH objects */
/*********************************************************************************/

#ifndef _FRAMEADV_H
#define _FRAMEADV_H

/*******************************/
/* Advanced NL_CMESH functions    */
/*******************************/

GW_EXPORT NL_CMESH *N_AllocCMesh( NL_STACKS *S );
GW_EXPORT NL_CMESH *N_AllocCMeshAndArrays( NL_INDEX m, NL_INDEX n, NL_INDEX o, NL_STACKS *S );
GW_EXPORT NL_VOID N_CMeshFromCPts( NL_CMESH *mesh, NL_CPOINT *** Pw, NL_INDEX n, NL_INDEX m, NL_INDEX o );
GW_EXPORT NL_FLAG N_CMeshFromCPtCoords( NL_CMESH *mesh, NL_REAL *** wx, NL_REAL *** wy, NL_REAL *** wz, NL_REAL *** w, NL_INDEX m, NL_INDEX n, NL_INDEX o, NL_STACKS *S );
GW_EXPORT NL_VOID N_CMeshGetCPts( NL_CMESH *mesh, NL_INDEX *m, NL_INDEX *n, NL_INDEX *o, NL_CPOINT **** Pw );
GW_EXPORT NL_VOID N_FreeCMesh( NL_CMESH *mesh, NL_STACKS *S );

/*******************************/
/* Advanced NL_EMESH functions    */
/*******************************/

GW_EXPORT NL_VOID N_EMeshFromPts( NL_EMESH *msh, NL_INDEX m, NL_INDEX n, NL_INDEX o, NL_POINT *** P );
GW_EXPORT NL_VOID N_EMeshGetPts( NL_EMESH *msh, NL_INDEX *m, NL_INDEX *n, NL_INDEX *o, NL_POINT **** P );
GW_EXPORT NL_VOID N_EMeshGetBBox( NL_EMESH *msh, NL_MINMAXBOX *box );

#endif /* _FRAMEADV_H */
