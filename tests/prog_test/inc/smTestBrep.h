// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// Shared analytic test fixtures and their insertion checks.

#ifndef __SMTESTBREP_H__
#define __SMTESTBREP_H__

#include <prog_test.h>
#include <SmCurveClass.h>

// Each fixture owns its brep; callers release it with SmObjDelete.
struct SmTorusVertexLoopTestCase
{
  SmBrep               *m_pBrep = NULL;
  SmFace               *m_pFace = NULL;
  ULONG                 m_lNumPoints = 0;
  SmVertex             *m_pVertices[5] = {NULL,NULL,NULL,NULL,NULL};
  SmCurveClassification m_sU,m_sV;
  SmSurfParamType        m_eCrossed = SM_SP_NEITHER;
  SmSurfParamType        m_eMissing = SM_SP_NEITHER;
  SmSurfParamType        m_eNear = SM_SP_NEITHER;
  ULONG                 m_lExpectedEdges = 0,m_lExpectedVertices = 0;
};

SmBrep * sm_make_xy_sheet(const SmContext & crContext, double dSize);
SmBoolean sm_HasAssertReport(const SmAssertArray & crAsserts, ULONG lTestIndex, const void * pOwner);

SmStatus sm_MakeLoopOrientationCase
 (const SmContext &crContext, // in : allocation context
  ULONG            lCase,     // in : original knot-sampling case number
  SmBrep         *&rpBrep,    // out: caller-owned brep
  SmFace         *&rpFace,    // out: face with the sampled boundary
  SmLoop         *&rpLoop);   // out: loop whose orientation is measured

SmStatus sm_MakeTorusVertexLoopCase
 (SmContext                &rContext, // i/o: allocation context and Boolean state
  ULONG                     lCase,    // in : original vertex-loop case number
  SmTorusVertexLoopTestCase &rCase);  // out: caller-owned brep and expected seam topology

SmStatus sm_InsertTorusVertexLoopSeams
 (SmContext                &rContext, // i/o: allocation context and Boolean state
  ULONG                     lCase,    // in : direct-insertion case number (0 through 19)
  SmTorusVertexLoopTestCase &rCase);  // i/o: fixture and expected seam topology

#endif // __SMTESTBREP_H__
