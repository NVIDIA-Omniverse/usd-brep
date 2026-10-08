// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*****************************************************************************/
/* CrvPoly.h: Function Declarations that act on NL_CURVE objects as polynomials */
/*****************************************************************************/

#ifndef _CRVPOLY_H
#define _CRVPOLY_H

GW_EXPORT NL_FLAG N_CrvPowerBasisEvalPt( NL_CURVE *, NL_PARAMETER, NL_POINT * );
GW_EXPORT NL_FLAG N_CrvPowerBasisReparam( NL_CURVE *, NL_PARAMETER, NL_PARAMETER, NL_CURVE *, NL_STACKS *, NL_STACKS * );
GW_EXPORT NL_FLAG N_CrvPowerBasisEvalDerivs( NL_CURVE *, NL_PARAMETER, NL_INDEX, NL_POINT * );

#endif /* _CRVPOLY_H */
