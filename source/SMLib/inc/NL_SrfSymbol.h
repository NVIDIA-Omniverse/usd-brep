// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/****************************************************************************************/
/* SrfSymbol.h: Symbolic Operators Function Declarations that act on NL_SURFACE objects */
/****************************************************************************************/

#ifndef _SRFSYMBOL_H
#define _SRFSYMBOL_H

GW_EXPORT NL_FLAG N_SrfMaxFirstDeriv( NL_SURFACE *, NL_FLAG, NL_POINT *, NL_REAL * );
GW_EXPORT NL_FLAG N_SrfMax2ndDeriv( NL_SURFACE *, NL_FLAG, NL_POINT *, NL_REAL * );
GW_EXPORT NL_FLAG N_DotProductTwoSrfs( NL_SURFACE *, NL_SURFACE *, NL_SFUN *, NL_SFUN *, NL_STACKS * );
GW_EXPORT NL_FLAG N_CrossProductTwoSrfs( NL_SURFACE *, NL_SURFACE *, NL_SURFACE *, NL_STACKS * );
GW_EXPORT NL_FLAG N_SumDiffTwoSrfs( NL_SURFACE *, NL_SURFACE *, NL_FLAG, NL_SURFACE *, NL_STACKS * );
GW_EXPORT NL_VOID N_ConstantMultiplySrf( NL_REAL, NL_SURFACE * );
GW_EXPORT NL_FLAG N_CombineTwoSrfs( NL_REAL, NL_SURFACE *, NL_REAL, NL_SURFACE *, NL_FLAG, NL_SURFACE *, NL_STACKS * );
GW_EXPORT NL_FLAG N_SumDiffSrfVector( NL_SURFACE *, NL_VECTOR, NL_FLAG );
GW_EXPORT NL_FLAG N_FirstDerivSrfNonRatSrf( NL_SURFACE *, NL_INDEX, NL_INDEX, NL_SURFACE *, NL_STACKS * );
GW_EXPORT NL_FLAG N_FirstDerivSrfRatSrf( NL_SURFACE *, NL_FLAG, NL_SURFACE *, NL_STACKS * );
GW_EXPORT NL_FLAG N_KDerivSrf( NL_SURFACE *, NL_INDEX, NL_INDEX, NL_SURFACE *, NL_STACKS * );
GW_EXPORT NL_FLAG N_AllDerivSrfNonRatSrf( NL_SURFACE *, NL_INDEX, NL_INDEX, NL_SURFACE ****, NL_STACKS * );
GW_EXPORT NL_FLAG N_AllDerivSrfNurbsSrf( NL_SURFACE *, NL_INDEX, NL_INDEX, NL_SURFACE ****, NL_STACKS * );
GW_EXPORT NL_FLAG N_NormalSrfNurbsSrf_UU( NL_SURFACE *, NL_SURFACE *, NL_STACKS * );
GW_EXPORT NL_FLAG N_SecondDerivSrfRatSrf( NL_SURFACE *, NL_SURFACE *, NL_STACKS * );
GW_EXPORT NL_FLAG N_MixedPartialDerivSrfRatSrf_UV( NL_SURFACE *, NL_SURFACE *, NL_STACKS * );
GW_EXPORT NL_FLAG N_SecondDerivSrfRatSrf_VV( NL_SURFACE *, NL_SURFACE *, NL_STACKS * );
GW_EXPORT NL_FLAG N_CrossBoundDerivCrvNonRatSrf( NL_SURFACE *, NL_FLAG, NL_CURVE *, NL_STACKS * );
GW_EXPORT NL_FLAG N_CrossBoundDerivCrvNurbsSrf( NL_SURFACE *, NL_FLAG, NL_CURVE *, NL_STACKS * );
GW_EXPORT NL_FLAG N_MaxDiffTwoSrfs( NL_SURFACE *, NL_SURFACE *, NL_REAL * );
GW_EXPORT NL_FLAG N_SrfMaxChangeMovingKnot( NL_SURFACE *, NL_INDEX, NL_INDEX, NL_REAL, NL_REAL, NL_FLAG, NL_REAL * );
GW_EXPORT NL_FLAG N_DerivSrfNonRatSrfKnot( NL_SURFACE *, NL_INDEX, NL_FLAG, NL_FLAG, NL_SURFACE *, NL_STACKS * );
GW_EXPORT NL_FLAG N_DerivSrfRatSrfKnot( NL_SURFACE *, NL_INDEX, NL_FLAG, NL_FLAG, NL_SURFACE *, NL_STACKS * );

#endif /* _SRFSYMBOL_H */
