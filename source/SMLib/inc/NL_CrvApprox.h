// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************/
/* CrvApprox.h: Surface Fitting related funcitons                     */
/**********************************************************************/

#ifndef _CRVAPPROX_H
#define _CRVAPPROX_H

GW_EXPORT NL_FLAG N_ApproxCrvWithArcs( NL_CURVE *, NL_REAL, NL_CURVE *, NL_STACKS * );

GW_EXPORT NL_FLAG N_ApproxContinuousCrvWithArcs( NL_CURVE *, NL_REAL, NL_CURVE *, NL_STACKS * );

GW_EXPORT NL_FLAG N_CrvArePtsWithinTol( NL_CPOINT *, NL_INDEX, NL_POINT *, NL_INDEX, NL_REAL, NL_FLAG * );

GW_EXPORT NL_FLAG N_CrvIsPolygonWithinTol( NL_CPOINT *, NL_INDEX, NL_POINT *, NL_INDEX, NL_REAL, NL_FLAG * );

GW_EXPORT NL_FLAG N_ApproxCircArcWithCrv( NL_POINT, NL_VECTOR, NL_VECTOR, NL_REAL, NL_REAL, NL_REAL, NL_POINT *, NL_POINT *, NL_DEGREE, NL_REAL, NL_FLAG, NL_CURVE *, NL_STACKS * );

GW_EXPORT NL_FLAG  N_ApproxCrvOnSrfWithCrv( NL_CURVE *, NL_SURFACE *, NL_DEGREE , NL_FLAG , NL_FLAG,
                             NL_FLAG (*)(NL_PARAMETER,NL_PARAMETER *), NL_REAL , NL_REAL ,
                             NL_CURVE *, NL_STACKS * );

GW_EXPORT NL_FLAG N_ApproxArcWithCrv( NL_POINT, NL_VECTOR, NL_VECTOR, NL_REAL r, NL_REAL, NL_REAL, NL_DEGREE, NL_INDEX, NL_INDEX, NL_REAL, NL_FLAG, NL_CURVE *, NL_STACKS * );

GW_EXPORT NL_FLAG N_CalcNumCPtsToApproxArc( NL_REAL, NL_REAL, NL_DEGREE, NL_FLAG, NL_INDEX *, NL_SFUN *, NL_STACKS * );

GW_EXPORT NL_FLAG N_ApproxCircArcWithCrvData( NL_POINT, NL_VECTOR, NL_VECTOR, NL_REAL, NL_REAL, NL_REAL, NL_POINT, NL_POINT, NL_DEGREE, NL_INDEX, NL_CURVE *, NL_STACKS * );

GW_EXPORT NL_FLAG N_ApproxNurbsWithNonRatCrv( NL_CURVE *, NL_REAL, NL_DEGREE, NL_FLAG, NL_FLAG, NL_CURVE *, NL_STACKS * );

GW_EXPORT NL_FLAG N_ApproxNurbsWithCrvKnots( NL_CURVE *, NL_REAL, NL_DEGREE, NL_FLAG, NL_FLAG, NL_FLAG, NL_KNOTVECTOR **, NL_CURVE *, NL_STACKS *, NL_STACKS * );

GW_EXPORT NL_FLAG  N_ApproxG1CrvWithCrv( NL_CURVE *, NL_DEGREE , NL_FLAG, NL_FLAG ,
                             NL_FLAG (*)(NL_PARAMETER,NL_PARAMETER *),
                             NL_REAL , NL_REAL , NL_REAL , NL_CURVE *, NL_STACKS * );

GW_EXPORT NL_FLAG  N_GetPtsForCrvApprox( NL_CURVE *, NL_FLAG (*)(NL_PARAMETER,NL_POINT *), NL_FLAG ,
                             NL_REAL , NL_REAL ,
                             NL_POINT **, NL_INDEX *, NL_PARAMETER **, NL_PARAMETER **, NL_STACKS * );

GW_EXPORT NL_FLAG  N_ApproxProcCrvWithCrv( NL_FLAG (*)(NL_PARAMETER,NL_POINT *), NL_PARAMETER , NL_PARAMETER ,
                             NL_VECTOR *,  NL_VECTOR *, NL_DEGREE , NL_FLAG ,
                             NL_FLAG (*)(NL_PARAMETER,NL_PARAMETER *), NL_REAL , NL_REAL ,
                             NL_REAL , NL_CURVE *, NL_STACKS * );

GW_EXPORT NL_FLAG  N_GetPtsForCrvApproxProc( NL_FLAG (*)(NL_PARAMETER,NL_POINT *), NL_PARAMETER , NL_PARAMETER ,
                             NL_DEGREE , NL_FLAG,  NL_REAL , NL_REAL , NL_POINT **, NL_INDEX *, NL_PARAMETER **,
                             NL_PARAMETER **,   NL_STACKS * );

GW_EXPORT NL_FLAG  N_ApproxProcCrvWithCrvFit( NL_POINT *, NL_INDEX , NL_FLAG (*)(NL_PARAMETER,NL_POINT *), NL_VECTOR *,
                             NL_VECTOR *, NL_PARAMETER *,  NL_FLAG , NL_PARAMETER *,
                             NL_FLAG (*)(NL_PARAMETER,NL_PARAMETER *), NL_DEGREE , NL_REAL , NL_REAL , NL_REAL ,
                             NL_CURVE *, NL_STACKS * );

GW_EXPORT NL_FLAG  N_ApproxCrvToPts( NL_POINT *, NL_INDEX, NL_INDEX *, NL_PARAMETER *, NL_INDEX, NL_REAL *, NL_INDEX, NL_INDEX, NL_CURVE *, NL_STACKS * );

GW_EXPORT NL_FLAG  N_CalcKnotVectorFromPtArrays( NL_EPOLYGON **, NL_INDEX, NL_INDEX, NL_DEGREE, NL_FLAG, NL_INDEX, NL_INDEX *, NL_REAL *, NL_KNOTVECTOR *, NL_STACKS * );


#endif /* _CRVAPPROX_H */
