// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

//*************************************************************************
// brep_distance: exercise SmApiBrepDistance (surface gap, wall clearance for
// containment) directly.
//   Own translation unit -- see the note in TestSmBrepDistance registration.
//*************************************************************************

#include <StdAfx.h>
#include <SmBrep.h>
#include <SmApiGeneral.h>
#include <SmApiPrimitives.h>
#include <SmApiQueries.h>

#include "SM_API_test.h"

static SmBoolean sm_ClearApproxEqual(double dA, double dB, double dAbsTol)
{
    double dDiff = dA - dB;
    if (dDiff < 0.0) dDiff = -dDiff;
    return dDiff <= dAbsTol ? TRUE : FALSE;
}

SmStatus TestSmBrepDistance()
{
    SmApiCreateContext();

    SmVector3d sOrigin(0.0, 0.0, 0.0);
    SmVector3d sPosB(4.0, 0.0, 0.0);
    SmVector3d sPosTouch(1.0, 0.0, 0.0);
    SmVector3d sPosCross(8.0, 3.0, 3.0);
    SmVector3d sPosInside(3.0, 3.0, 3.0);

    // Disjoint: 3-unit gap.
    SmBrep* pA = NULL;   SER(SmApiCreateBox(sOrigin, 1.0, 1.0, 1.0, pA));
    SmBrep* pB = NULL;   SER(SmApiCreateBox(sPosB, 1.0, 1.0, 1.0, pB));
    if (pA == NULL || pB == NULL) { delete pA; delete pB; return SM_ERR; }
    double d = -1.0;
    if (SmApiBrepDistance(pA, pB, d) != SM_SUCCESS || !sm_ClearApproxEqual(d, 3.0, 1.0e-4))
        { delete pA; delete pB; return SM_ERR; }

    // Symmetric.
    double dRev = -1.0;
    if (SmApiBrepDistance(pB, pA, dRev) != SM_SUCCESS || !sm_ClearApproxEqual(d, dRev, 1.0e-9))
        { delete pA; delete pB; return SM_ERR; }

    // Touching -> 0.
    SmBrep* pTouch = NULL;  SER(SmApiCreateBox(sPosTouch, 1.0, 1.0, 1.0, pTouch));
    double dT = -1.0;
    if (SmApiBrepDistance(pA, pTouch, dT) != SM_SUCCESS || !sm_ClearApproxEqual(dT, 0.0, 1.0e-4))
        { delete pA; delete pB; delete pTouch; return SM_ERR; }

    // Interpenetrating -> 0; fully enclosed -> 3.0, the wall clearance to the
    // nearest cavity wall (pInside [3,5]^3 inside pBig [0,10]^3).
    SmBrep* pBig = NULL;    SER(SmApiCreateBox(sOrigin, 10.0, 10.0, 10.0, pBig));
    SmBrep* pCross = NULL;  SER(SmApiCreateBox(sPosCross, 4.0, 4.0, 4.0, pCross));
    SmBrep* pInside = NULL; SER(SmApiCreateBox(sPosInside, 2.0, 2.0, 2.0, pInside));
    double dC = -1.0, dI = -1.0;
    if (SmApiBrepDistance(pBig, pCross, dC) != SM_SUCCESS || !sm_ClearApproxEqual(dC, 0.0, 1.0e-4) ||
        SmApiBrepDistance(pBig, pInside, dI) != SM_SUCCESS || !sm_ClearApproxEqual(dI, 3.0, 1.0e-4))
        { delete pA; delete pB; delete pTouch; delete pBig; delete pCross; delete pInside; return SM_ERR; }

    // NULL inputs -> error, not crash.
    double dNull = 0.0;
    if (SmApiBrepDistance(NULL, pA, dNull) == SM_SUCCESS ||
        SmApiBrepDistance(pA, NULL, dNull) == SM_SUCCESS)
        { delete pA; delete pB; delete pTouch; delete pBig; delete pCross; delete pInside; return SM_ERR; }

    delete pA; delete pB; delete pTouch; delete pBig; delete pCross; delete pInside;
    return SM_SUCCESS;
}
