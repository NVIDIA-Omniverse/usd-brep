// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*****************************************************************************/
/* NLI_math.h: NLib Internal Inline Only Functions                           */
/*****************************************************************************/

#include <NL_Globals.h>

#ifndef _NLI_MATH_H
#    define _NL_MATH_H

/***********************/
/* Inlined functions   */
/* Not for export      */
/***********************/

/*******************************************************************/ /**
 PURPOSE: Inline/specialized version of N_CPtToPtEuclid, utilized in
 to facilitate specialization for  NOZ and NOW.

 NOTES: 
 ***********************************************************************/
NL_INLINE NL_VOID NI_CPtToPtEuclid_specialized
 (NL_BOOLEAN noz,
  NL_BOOLEAN now, 
  NL_CPOINT Pw, 
  NL_POINT* P)
{
    if (!now)
    {
        NL_REAL reciprocalW = 1.0 / Pw.w;
        P->x = Pw.x * reciprocalW;
        P->y = Pw.y * reciprocalW;

        if (!noz)
            P->z = Pw.z * reciprocalW;
        else
            P->z = 0.0;
    }
    else
    {
        P->x = Pw.x;
        P->y = Pw.y;

        if (!noz)
            P->z = Pw.z;
        else
            P->z = 0.0;
    }
} /* end NI_CPtToPtEuclid_specalized */

/*******************************************************************/ /**
 PURPOSE: Inline/specialized version of VectorBlendCPt, utilized in
  to facilitate specialization for  NOZ and NOW.

 NOTES: 
 ***********************************************************************/
NL_INLINE NL_VOID NI_VectorBlendCPt_specialized
 (NL_BOOLEAN noz,       // in : TRUE = CPT is 2d not using Z coord, FALSE=CPT is 3d
  NL_BOOLEAN now,       // in : TRUE = CPT is not rational not using w coord, FALSE=CPT is rational
  NL_REAL    alpha,     /* in : alpha of Bw = Bw + alpha*Aw */
  const NL_CPOINT&  Aw, /* in : Aw    of Bw = Bw + alpha*Aw */
  NL_CPOINT *Bw )       /* out: Bw    of Bw = Bw + alpha*Aw */
{
    Bw->x = Bw->x + alpha * Aw.x;
    Bw->y = Bw->y + alpha * Aw.y;

    if( !noz )
        Bw->z = Bw->z + alpha * Aw.z;
    else
        Bw->z = NL_NOZ;

    if( !now )
        Bw->w = Bw->w + alpha * Aw.w;
    else
        Bw->w = NL_NOW;
} /* end NI_VectorBlendCPt_specialized */
#endif
