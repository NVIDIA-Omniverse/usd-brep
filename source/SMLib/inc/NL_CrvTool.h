// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************/
/* CrvBsc.h: Tool Function Declarations that act on NL_CURVE objects     */
/**********************************************************************/

#ifndef _CRVTOOL_H
#define _CRVTOOL_H

/* Tools */

GW_EXPORT NL_FLAG N_CrvInsertKnot( NL_CURVE *, NL_PARAMETER, NL_INDEX, NL_CURVE *, NL_STACKS *, NL_STACKS * );
GW_EXPORT NL_FLAG N_CrvSplit( NL_CURVE *, NL_PARAMETER, NL_CURVE *, NL_CURVE *, NL_STACKS * );
GW_EXPORT NL_FLAG N_CrvSplitAtInteriorLines( NL_CURVE *, NL_CURVE ***, NL_INDEX *, NL_STACKS * );
GW_EXPORT NL_FLAG N_CrvSplitIntoLinesAndArcs( NL_CURVE *, NL_REAL, NL_CURVE ***, NL_INDEX *, NL_STACKS * );
GW_EXPORT NL_FLAG N_CrvInverseKnotInsert( NL_CURVE *, NL_POINT, NL_CURVE *, NL_STACKS *, NL_STACKS * );
GW_EXPORT NL_FLAG N_CrvDecomposeBez( NL_CURVE *, NL_CURVE ***, NL_INDEX *, NL_STACKS * );
GW_EXPORT NL_FLAG N_CrvDecomposeContinuity( NL_CURVE *, NL_CURVE ***, NL_INDEX *, NL_STACKS * );
GW_EXPORT NL_FLAG N_CrvDecomposeAtG1Continuity( NL_CURVE *, NL_CURVE ***, NL_INDEX *, NL_STACKS * );
GW_EXPORT NL_FLAG N_CrvRefine( NL_CURVE *, NL_KNOTVECTOR *, NL_CURVE *, NL_STACKS *, NL_STACKS * );
GW_EXPORT NL_FLAG N_CrvExtractCrvSeg( NL_CURVE *, NL_PARAMETER, NL_PARAMETER, NL_CURVE *, NL_STACKS *, NL_STACKS * );
GW_EXPORT NL_FLAG N_CrvRemoveKnot( NL_CURVE *, NL_PARAMETER, NL_INDEX, NL_REAL, NL_INDEX *, NL_CURVE *, NL_STACKS * );
GW_EXPORT NL_FLAG N_CrvRemoveDuplicateCPts( NL_CURVE *, NL_REAL tol );
GW_EXPORT NL_FLAG N_CrvRemoveKnotMaxErr( NL_CURVE *, NL_INDEX, NL_INDEX, NL_REAL * );
GW_EXPORT NL_FLAG N_CrvRemoveKnots( NL_CURVE *, NL_REAL, NL_CURVE *, NL_STACKS * );
GW_EXPORT NL_FLAG N_CrvRemoveKnotsParams( NL_CURVE *, NL_PARAMETER *, NL_INDEX, NL_FLAG *, NL_REAL, NL_CURVE *, NL_STACKS * );
GW_EXPORT NL_FLAG N_CrvElevateDegree( NL_CURVE *, NL_INDEX, NL_CURVE *, NL_STACKS *, NL_STACKS * );
GW_EXPORT NL_FLAG N_CrvReduceDegreeOnce( NL_CURVE *, NL_REAL, NL_FLAG *, NL_CURVE *, NL_REAL *, NL_STACKS *, NL_STACKS * );
GW_EXPORT NL_FLAG N_CrvReduceDegree( NL_CURVE *, NL_REAL, NL_CURVE *, NL_STACKS *, NL_STACKS * );
GW_EXPORT NL_FLAG N_CrvsMakeCompatible( NL_CURVE **, NL_INDEX, NL_STACKS * );
GW_EXPORT NL_FLAG N_CrvsMakeCompatibleKnotTol( NL_CURVE **, NL_INDEX, NL_REAL, NL_STACKS * );
GW_EXPORT NL_FLAG N_CrvRemoveDegenSegs( NL_CURVE *, NL_REAL, NL_INDEX *, NL_CURVE *, NL_STACKS *, NL_STACKS * );
GW_EXPORT NL_FLAG N_CrvExtractSegClosed( NL_CURVE *, NL_PARAMETER, NL_PARAMETER, NL_REAL, NL_CURVE *, NL_PARAMETER *, NL_STACKS *, NL_STACKS * );
GW_EXPORT NL_FLAG N_CrvsMakeCompatibleAdjKnots( NL_CURVE **, NL_INDEX, NL_REAL, NL_STACKS * );
GW_EXPORT NL_FLAG N_tooCrvCleanSpans( NL_CURVE *, int *, NL_STACKS * );

#endif /* _CRVTOOL_H */
