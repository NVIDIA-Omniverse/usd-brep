// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************/
/* CrvOffset.h: Surface Fitting related funcitons                        */
/**********************************************************************/

#ifndef _CRVOFFSET_H
#define _CRVOFFSET_H

GW_EXPORT NL_FLAG N_CrvOffsetFuncVariableDir( NL_CURVE *, NL_CFUN *, NL_CURVE *, NL_DEGREE, NL_FLAG, NL_REAL, NL_CURVE *, NL_STACKS * );

GW_EXPORT NL_FLAG N_CrvOffsetFuncConstantDir( NL_CURVE *, NL_CFUN *, NL_VECTOR, NL_DEGREE, NL_FLAG, NL_REAL, NL_CURVE *, NL_STACKS * );

GW_EXPORT NL_FLAG  N_CrvOffsetApprox( NL_CURVE *, NL_VECTOR , NL_REAL , NL_DEGREE , NL_FLAG, NL_FLAG ,
                             NL_FLAG (*)(NL_PARAMETER,NL_PARAMETER *), NL_REAL , NL_REAL , NL_REAL ,
                             NL_CURVE *, NL_STACKS * );

GW_EXPORT NL_FLAG N_CrvOffsetPtSampling( NL_CURVE *, NL_VECTOR, NL_REAL, NL_DEGREE, NL_FLAG, NL_FLAG, NL_REAL, NL_CURVE *, NL_STACKS * );
GW_EXPORT NL_FLAG N_SrfOffset( NL_SURFACE *, NL_REAL, NL_DEGREE, NL_DEGREE, NL_FLAG, NL_REAL, NL_REAL, NL_SURFACE *, NL_STACKS * );

GW_EXPORT NL_FLAG  N_CrvOffset( NL_CURVE *, NL_VECTOR , NL_REAL , NL_DEGREE , NL_FLAG ,
                             NL_FLAG , NL_FLAG (*)(NL_PARAMETER,NL_PARAMETER *), NL_REAL , NL_REAL , NL_REAL ,
                             NL_CURVE *, NL_STACKS * );

GW_EXPORT NL_FLAG N_SrfOffsetFunc( NL_SURFACE *, NL_SFUN *, NL_SURFACE *, NL_DEGREE, NL_DEGREE, NL_FLAG, NL_REAL, NL_SURFACE *, NL_STACKS * );

#endif /* _CRVOFFSET_H */
