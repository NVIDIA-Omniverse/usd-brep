// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*******************************************************************************/
/* SrfPoly.h: Function Declarations that act on NL_SURFACE objects as Polynomials */
/*******************************************************************************/

#ifndef _SRFPOLY_H
#define _SRFPOLY_H

GW_EXPORT NL_FLAG N_SrfPowerBasisEvalPt( NL_SURFACE *, NL_PARAMETER, NL_PARAMETER, NL_POINT * );
GW_EXPORT NL_FLAG N_SrfPowerBasisReparam( NL_SURFACE *, NL_PARAMETER, NL_PARAMETER, NL_PARAMETER, 
                                          NL_PARAMETER, NL_SURFACE *, NL_STACKS *, NL_STACKS * );
GW_EXPORT NL_FLAG N_SrfPowerBasisEvalDerivs( NL_SURFACE *, NL_PARAMETER, NL_PARAMETER, NL_FLAG, 
                                             NL_INDEX, NL_INDEX, NL_POINT ** );

#endif /* _SRFPOLY_H */
