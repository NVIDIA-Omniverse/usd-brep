// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************
FILE NAME: TestSmErrorCodes.cpp

PURPOSE:
    SM_API reports a caller mistake (a NULL required argument, too few
    items, mismatched counts, a zero-length axis) as SM_ERR_INVALID_INPUT,
    not the generic SM_ERR. One representative call per source file and
    kind of mistake.
**********************************************************************/

#include "StdAfx.h"

#include <SmApiBrep.h>
#include <SmApiCurves.h>
#include <SmApiFillets.h>
#include <SmApiGeneral.h>
#include <SmApiHeal.h>
#include <SmApiImportExport.h>
#include <SmApiIntersectors.h>
#include <SmApiPolygons.h>
#include <SmApiPrimitives.h>
#include <SmApiQueries.h>
#include <SmApiSurfaces.h>
#include <SmMessages.h>
#include <SmLine.h>
#include <SmPlane.h>
#include <SmSolutionArray.h>
#include <cstdio>

#include "SM_API_test.h"

namespace
{
// Controlled kernel outcomes keep these wrapper tests independent of solver
// heuristics. The curve itself has a valid domain and evaluable line geometry.
class SmQueryStatusLine : public SmLine
{
public:
    SmQueryStatusLine( SmStatus eSolveStatus, SmBoolean bEmpty, SmBoolean bFailEvaluation )
        : SmLine(SmPoint3d(0, 0, 0), SmPoint3d(1, 0, 0), 3, SmApiGetOrCreateContext()),
          m_eSolveStatus(eSolveStatus), m_bEmpty(bEmpty), m_bFailEvaluation(bFailEvaluation)
    {
    }

    SmStatus GlobalPointSolve( const SmExtent1d&, SmSolverOperationType,
                              const SmPoint3d&, double, const double*, const SmVector3d*,
                              SmSolutionRequestedType eRequested, SmSolutionArray& rSolutions ) const override
    {
        if( m_eSolveStatus != SM_SUCCESS || m_bEmpty )
            return m_eSolveStatus;
        SmSolution sSolution;
        sSolution.m_eSolutionType = SM_ST_SINGLE_VALUE;
        sSolution.m_lNumObjects = 1;
        sSolution.m_apObjects[0] = const_cast<SmQueryStatusLine*>(this);
        sSolution.m_lNumVariables = 1;
        sSolution.m_vStart.m_dSolutionValue = 1.0;
        if( eRequested == SM_SR_ALL )
        {
            sSolution.m_vStart[0] = 0.25;
            rSolutions.Add(sSolution);
        }
        sSolution.m_vStart[0] = 0.75;
        rSolutions.Add(sSolution);
        return SM_SUCCESS;
    }

    SmStatus EvaluatePoint( double dParameter, SmPoint3d& rPoint ) const override
    {
        if( m_bFailEvaluation && dParameter > 0.5 )
            return SM_ERR_NOT_WITHIN_TOLERANCE;
        return SmLine::EvaluatePoint(dParameter, rPoint);
    }

private:
    SmStatus m_eSolveStatus;
    SmBoolean m_bEmpty;
    SmBoolean m_bFailEvaluation;
};

// Supply deterministic point/overlap solutions, so a late evaluation failure
// can be tested without depending on numerical solver behavior.
class SmIntersectionStatusPlane : public SmPlane
{
public:
    SmIntersectionStatusPlane(SmBoolean bEmpty, SmBoolean bRanges)
        : SmPlane(SmPoint3d(0, 0, 0), SmVector3d(1, 0, 0), SmVector3d(0, 1, 0),
                  SmVector2d(1, 1), SmExtent2d(SmPoint2d(0, 0), SmPoint2d(1, 1)),
                  SmApiGetOrCreateContext()),
          m_bEmpty(bEmpty), m_bRanges(bRanges)
    {
    }

    SmStatus GlobalCurveIntersect(const SmExtent2d&, const SmCurve&, const SmExtent1d&,
                                 double, SmSolutionArray& rSolutions) const override
    {
        if( m_bEmpty )
            return SM_SUCCESS;
        for( ULONG ii = 0; ii < 2; ++ii )
        {
            SmSolution sSolution;
            sSolution.m_eSolutionType = m_bRanges ? SM_ST_RANGE_OF_VALUES : SM_ST_SINGLE_VALUE;
            sSolution.m_lNumVariables = 3;
            sSolution.m_vStart[0] = ii == 0 ? 0.25 : 0.75;
            sSolution.m_vStart[1] = sSolution.m_vStart[0];
            sSolution.m_vStart[2] = 0.5;
            sSolution.m_vEnd[0] = ii == 0 ? 0.4 : 0.9;
            sSolution.m_vEnd[1] = sSolution.m_vEnd[0];
            sSolution.m_vEnd[2] = 0.5;
            rSolutions.Add(sSolution);
        }
        return SM_SUCCESS;
    }

private:
    SmBoolean m_bEmpty;
    SmBoolean m_bRanges;
};

class SmIntersectionStatusLine : public SmLine
{
public:
    explicit SmIntersectionStatusLine(double dFailureParameter)
        : SmLine(SmPoint3d(0, 0.5, 0), SmPoint3d(1, 0.5, 0), 3, SmApiGetOrCreateContext()),
          m_dFailureParameter(dFailureParameter)
    {
    }

    SmStatus EvaluatePoint(double dParameter, SmPoint3d& rPoint) const override
    {
        if( dParameter >= m_dFailureParameter )
            return SM_ERR_NOT_WITHIN_TOLERANCE;
        return SmLine::EvaluatePoint(dParameter, rPoint);
    }

private:
    double m_dFailureParameter;
};

SmStatus TestIntersectionOutputStatus()
{
    SmStatus result = SM_SUCCESS;
    // Empty solve; points; late point failure; overlaps; late overlap start
    // failure; late overlap end failure. Overlap endpoint checks are used only
    // by the Brep wrapper's duplicate filtering.
    for( ULONG lCase = 0; lCase < 6; ++lCase )
    {
        const SmBoolean bRanges = lCase >= 3;
        SmIntersectionStatusLine sCurve(lCase == 2 || lCase == 4 ? 0.5 : (lCase == 5 ? 0.8 : 2.0));
        SmBrep* pBrep = new (*SmApiGetOrCreateContext()) SmBrep();
        SmObjDelete sBrepCleanup(pBrep);
        SmIntersectionStatusPlane* pPlane = new (*SmApiGetOrCreateContext())
            SmIntersectionStatusPlane(lCase == 0, bRanges);
        SmObjDelete sPlaneCleanup(pPlane);
        SmFace* pFace = NULL;
        SER( pBrep->CreateFaceFromSurface(pPlane, pPlane->GetNaturalUVDomain(), pFace) );
        sPlaneCleanup.Clear(); // The face now owns the surface.

        for( ULONG lWrapper = 0; lWrapper < 3; ++lWrapper )
        {
            if( lCase >= 4 && lWrapper != 2 )
                continue;
            SmLine* pSentinel = NULL;
            SER( SmApiCreateLineSegment(SmPoint3d(10, 0, 0), SmPoint3d(11, 0, 0), pSentinel) );
            SmTArray<SmCurve*> sCurves;
            sCurves.Add(pSentinel);
            SmObjsDelete<SmCurve*> sCurvesCleanup(&sCurves);
            SmTArray<SmPoint3d> sPoints;
            const SmPoint3d sSentinelPoint(10, 20, 30);
            sPoints.Add(sSentinelPoint);

            SmStatus stat;
            if( lWrapper == 0 )
                stat = SmApiIntersectCurveSurface(&sCurve, pPlane, sCurves, sPoints);
            else if( lWrapper == 1 )
                stat = SmApiIntersectCurveFace(&sCurve, pFace, sCurves, sPoints);
            else
                stat = SmApiIntersectCurveBrep(&sCurve, pBrep, sCurves, sPoints);

            const SmStatus eExpected = lCase == 0 ? SM_ERR :
                (lCase == 2 || lCase >= 4 ? SM_ERR_NOT_WITHIN_TOLERANCE : SM_SUCCESS);
            const ULONG lExpectedCurves = 1 + (eExpected == SM_SUCCESS && bRanges ? 2 : 0);
            const ULONG lExpectedPoints = 1 + (eExpected == SM_SUCCESS && !bRanges ? 2 : 0);
            if( stat != eExpected || sCurves.GetSize() != lExpectedCurves ||
                sPoints.GetSize() != lExpectedPoints || sCurves[0] != pSentinel ||
                sPoints[0].DistanceBetween(sSentinelPoint) != 0.0 )
            {
                fprintf(stderr, "Intersection case %lu wrapper %lu: status %ld expected %ld, curves %lu points %lu\n",
                        (unsigned long)lCase, (unsigned long)lWrapper, (long)stat, (long)eExpected,
                        (unsigned long)sCurves.GetSize(), (unsigned long)sPoints.GetSize());
                result = SM_ERR;
            }
        }
    }
    return result;
}
}

SmStatus TestSmInvalidInputCodes()
{
    SmApiCreateContext();

    SmStatus result = SM_SUCCESS;
    auto expect = [&result](SmApiStatus stat, const char* call, SmApiStatus expected = SM_ERR_INVALID_INPUT)
    {
        if (stat != expected)
        {
            fprintf(stderr, "%s returned %ld, expected %ld\n", call, (long)stat, (long)expected);
            result = SM_ERR;
        }
    };

    SmBrep* pBrep = NULL;
    SmSurface* pSurface = NULL;
    SmPolyBrep* pPolyBrep = NULL;
    SmVector3d sOrigin(0.0, 0.0, 0.0);
    SmPoint3d sPoint, sOther;
    SmVector3d sTangent;
    double dDistance = 0.0;
    SmTArray<SmEdge*> sEdges;
    SmTArray<SmFace*> sFaces;
    SmTArray<SmCurve*> sCurves;
    SmTArray<SmBSplineCurve*> sNoCurves;

    // NULL required arguments
    expect(SmApiStitch(NULL), "SmApiStitch");
    expect(SmApiTurnToNurbs(NULL), "SmApiTurnToNurbs");
    expect(SmApiHealBrep(NULL), "SmApiHealBrep");
    expect(SmApiRemoveFillet(NULL), "SmApiRemoveFillet");
    expect(SmApiFilletEdges(NULL, sEdges, 1.0, 0, 0, 0.0), "SmApiFilletEdges");
    expect(SmApiCreatePipeSweep(1.0, NULL, FALSE, pBrep), "SmApiCreatePipeSweep");
    expect(SmApiRemoveCurveKnots(NULL), "SmApiRemoveCurveKnots");
    expect(SmApiLiftUVCurveFromSrf(NULL, NULL, NULL), "SmApiLiftUVCurveFromSrf");
    expect(SmApiIntersectSurfaces(NULL, NULL, NULL), "SmApiIntersectSurfaces");
    expect(SmApiIntersectBreps(NULL, NULL, sCurves, NULL), "SmApiIntersectBreps");
    expect(SmApiPolyBrepCopy(NULL, pPolyBrep), "SmApiPolyBrepCopy");
    expect(SmApiGetFaces(NULL, sFaces), "SmApiGetFaces");
    expect(SmApiBrepBoundingBox(NULL, TRUE, sPoint, sOther), "SmApiBrepBoundingBox");
    expect(SmApiBrepClosestPoint(NULL, sPoint, sOther, dDistance), "SmApiBrepClosestPoint");
    expect(SmApiEdgeTangent(NULL, 0.0, sTangent), "SmApiEdgeTangent");
    expect(SmApiBrepCopy(NULL, pBrep), "SmApiBrepCopy");
    expect(SmApiReadBrepFromFile(NULL, FALSE, FALSE, pBrep), "SmApiReadBrepFromFile");

    // Too few items, bad sizes and mismatched counts
    expect(SmApiMakeCurvesCompatible(sNoCurves), "SmApiMakeCurvesCompatible");
    expect(SmApiOrderCurves(sNoCurves), "SmApiOrderCurves");
    expect(SmApiCreateSkin(sNoCurves, pSurface), "SmApiCreateSkin");
    expect(SmApiCreatePyramid(sOrigin, 0.0, 1.0, pBrep), "SmApiCreatePyramid");
    SmTArray<SmPoint3d> sThreePoints;
    sThreePoints.Add(SmPoint3d(0, 0, 0));
    sThreePoints.Add(SmPoint3d(1, 0, 0));
    sThreePoints.Add(SmPoint3d(0, 1, 0));
    expect(SmApiCreateSurface(sThreePoints, 2, 2, pSurface), "SmApiCreateSurface");

    // A zero-length rotation axis
    SmBrep* pBox = NULL;
    if (SmApiCreateBox(sOrigin, 1.0, 1.0, 1.0, pBox) != SM_SUCCESS || pBox == NULL)
    {
        return SM_ERR;
    }
    SmVector3d sZeroAxis(0.0, 0.0, 0.0);
    expect(SmApiRotate(pBox, sOrigin, sZeroAxis, 30.0), "SmApiRotate");
    delete pBox;

    // Preserve specific kernel errors, but keep the established SM_ERR result
    // for a successful solve with no solution. Also check the evaluation step
    // after a successful solve, not just the solver's status.
    const SmApiStatus aExpected[] = { SM_ERR_NOT_CONVERGING, SM_ERR, SM_ERR_NOT_WITHIN_TOLERANCE, SM_SUCCESS };
    for( ULONG ii = 0; ii < 4; ++ii )
    {
        SmQueryStatusLine sCurve(ii == 0 ? SM_ERR_NOT_CONVERGING : SM_SUCCESS, ii == 1, ii == 2);
        double dParameter = 0.0;
        expect(SmApiCurveClosestPoint(&sCurve, sOrigin, sPoint, dParameter, dDistance),
               "SmApiCurveClosestPoint kernel outcome", aExpected[ii]);

        SmTArray<SmPoint3d> sPoints;
        SmTArray<double> sParameters, sDistances;
        expect(SmApiCurveClosestPointAll(&sCurve, sOrigin, sPoints, sParameters, sDistances),
               "SmApiCurveClosestPointAll kernel outcome", aExpected[ii]);
        if( sPoints.GetSize() != sParameters.GetSize() || sPoints.GetSize() != sDistances.GetSize() )
            result = SM_ERR;
        // A failed evaluation must never append a stale/unevaluated point.
        const ULONG lExpectedPoints = ii < 2 ? 0 : (ii == 2 ? 1 : 2);
        if( sPoints.GetSize() != lExpectedPoints )
            result = SM_ERR;
    }

    if( TestIntersectionOutputStatus() != SM_SUCCESS )
        result = SM_ERR;
    return result;
}
