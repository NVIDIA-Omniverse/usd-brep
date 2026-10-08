// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "OcctGeometryConvert.h"

#include <pxr/base/gf/vec2d.h>
#include <pxr/base/gf/vec3d.h>

#include <algorithm>
#include <cmath>
#include <vector>

namespace occt
{
namespace
{

pxr::GfVec3d ToVec(const SPnt3& rPnt)
{
    return pxr::GfVec3d(rPnt.dX, rPnt.dY, rPnt.dZ);
}

pxr::GfVec3d ToVec(const SDir3& rDir)
{
    return pxr::GfVec3d(rDir.dX, rDir.dY, rDir.dZ);
}

pxr::GfVec2d ToVec(const SPnt2& rPnt)
{
    return pxr::GfVec2d(rPnt.dX, rPnt.dY);
}

pxr::GfVec2d ToVec(const SDir2& rDir)
{
    return pxr::GfVec2d(rDir.dX, rDir.dY);
}

pxr::GfVec3d Place(const pxr::GfMatrix4d& rXf, const SPnt3& rPnt)
{
    return rXf.Transform(ToVec(rPnt));
}

pxr::GfVec3d Orient(const pxr::GfMatrix4d& rXf, const SDir3& rDir)
{
    pxr::GfVec3d v = rXf.TransformDir(ToVec(rDir));
    const double dLen = v.GetLength();
    return (dLen > 0.0) ? (v / dLen) : v;
}

// Build a GfMatrix4d (USD row-vector convention: v' = v * M) from an OCCT 3x4 affine
// out_r = sum_c dMatrix[r][c]*in_c + dMatrix[r][3].
pxr::GfMatrix4d FromOcctMatrix(const double dM[3][4])
{
    pxr::GfMatrix4d sResult(1.0);
    for (int r = 0; r < 3; ++r)
    {
        for (int c = 0; c < 3; ++c)
        {
            sResult[c][r] = dM[r][c]; // transpose linear part for row-vector convention
        }
        sResult[3][r] = dM[r][3]; // translation in the last row
    }
    return sResult;
}

// Number of extra (wrapped) poles a periodic direction contributes when unrolled to the
// equivalent unclamped (non-periodic) B-spline. This is OCCT's M1 = (degree + 1) - mult[first].
// Returns 0 for a non-periodic direction.
int PeriodicExtraPoles(const std::vector<int>& rMults, int iDegree, bool bPeriodic)
{
    if (!bPeriodic || rMults.empty())
    {
        return 0;
    }
    const int iExtra = (iDegree + 1) - rMults.front();
    return iExtra > 0 ? iExtra : 0;
}

// Build the flat knot vector USD expects (length == vertexCount + order). For a clamped
// (non-periodic) B-spline this just expands the (knot, multiplicity) pairs. For a periodic
// B-spline this reproduces OCCT's BSplCLib::KnotSequence: the expanded knots sit in the middle
// and M1 = (degree+1) - mult[first] extra knots are mirrored (period-shifted) onto each end, so
// the result is the geometrically identical unclamped representation over one period.
pxr::VtArray<double> BuildFlatKnots(const std::vector<double>& rKnots, const std::vector<int>& rMults, int iDegree, bool bPeriodic)
{
    pxr::VtArray<double> sFlat;
    const size_t uiK = std::min(rKnots.size(), rMults.size());
    if (uiK == 0)
    {
        return sFlat;
    }

    int iSumMults = 0;
    for (size_t i = 0; i < uiK; ++i)
    {
        iSumMults += rMults[i] > 0 ? rMults[i] : 0;
    }

    if (!bPeriodic)
    {
        sFlat.reserve(static_cast<size_t>(iSumMults));
        for (size_t i = 0; i < uiK; ++i)
        {
            const int iMult = rMults[i] > 0 ? rMults[i] : 0;
            for (int m = 0; m < iMult; ++m)
            {
                sFlat.push_back(rKnots[i]);
            }
        }
        return sFlat;
    }

    const int iM1 = PeriodicExtraPoles(rMults, iDegree, true);
    const int iLen = iSumMults + 2 * iM1;
    sFlat.assign(static_cast<size_t>(iLen), 0.0);

    // Middle section: the expanded distinct knots, offset by M1.
    int iIdx = iM1;
    for (size_t i = 0; i < uiK; ++i)
    {
        const int iMult = rMults[i] > 0 ? rMults[i] : 0;
        for (int m = 0; m < iMult; ++m)
        {
            sFlat[static_cast<size_t>(iIdx++)] = rKnots[i];
        }
    }

    const double dPeriod = rKnots[uiK - 1] - rKnots[0];

    // Leading section: walk backwards from the second-to-last distinct knot, shifted by -period.
    {
        int m = 1;
        int j = static_cast<int>(uiK) - 2;
        for (int i = iM1 - 1; i >= 0; --i)
        {
            // Clamp the read against malformed multiplicities driving j out of range (well-formed
            // OCCT data keeps j in [0, uiK-1] for every iteration, so this is a no-op there).
            const size_t uiJ = static_cast<size_t>(j < 0 ? 0 : j);
            sFlat[static_cast<size_t>(i)] = rKnots[uiJ] - dPeriod;
            ++m;
            if (j >= 0 && m > rMults[static_cast<size_t>(j)])
            {
                --j;
                m = 1;
            }
        }
    }

    // Trailing section: walk forwards from the second distinct knot, shifted by +period.
    {
        int m = 1;
        int j = 1;
        for (int i = iIdx; i < iLen; ++i)
        {
            // Clamp the read against malformed multiplicities driving j past the last knot (well-formed
            // OCCT data keeps j in [0, uiK-1] for every iteration, so this is a no-op there).
            const size_t uiJ = static_cast<size_t>(j >= static_cast<int>(uiK) ? static_cast<int>(uiK) - 1 : j);
            sFlat[static_cast<size_t>(i)] = rKnots[uiJ] + dPeriod;
            ++m;
            if (j < static_cast<int>(uiK) && m > rMults[static_cast<size_t>(j)])
            {
                ++j;
                m = 1;
            }
        }
    }

    return sFlat;
}

// Evaluate a (possibly rational/periodic) B-spline curve at parameter t in local space via de Boor.
// Poles/weights are unrolled for periodic curves to match BuildFlatKnots. Returns the front pole on
// any inconsistency. Used for domain sampling, so robustness matters more than micro-accuracy.
pxr::GfVec3d EvalBSplineCurveLocal(const SCurve3d& rCurve, double t)
{
    const int iP = rCurve.iDegree;
    const int iN0 = static_cast<int>(rCurve.sPoles.size());
    if (iP < 1 || iN0 < 1)
    {
        return iN0 > 0 ? ToVec(rCurve.sPoles.front()) : ToVec(rCurve.sLocation);
    }

    const int iExtra = PeriodicExtraPoles(rCurve.iMults, iP, rCurve.bPeriodic);
    const int iN = iN0 + iExtra;
    const bool bHaveW = rCurve.bRational && static_cast<int>(rCurve.dWeights.size()) == iN0;

    std::vector<pxr::GfVec3d> vP(static_cast<size_t>(iN));
    std::vector<double> vW(static_cast<size_t>(iN));
    for (int i = 0; i < iN; ++i)
    {
        const int iSrc = (i < iN0) ? i : (i - iN0);
        vP[static_cast<size_t>(i)] = ToVec(rCurve.sPoles[static_cast<size_t>(iSrc)]);
        vW[static_cast<size_t>(i)] = bHaveW ? rCurve.dWeights[static_cast<size_t>(iSrc)] : 1.0;
    }

    // Bezier curves carry no explicit knots; synthesize the clamped [0..0, 1..1] vector.
    pxr::VtArray<double> vU;
    if (rCurve.dKnots.empty() && iN0 == iP + 1)
    {
        for (int i = 0; i < iP + 1; ++i)
        {
            vU.push_back(0.0);
        }
        for (int i = 0; i < iP + 1; ++i)
        {
            vU.push_back(1.0);
        }
    }
    else
    {
        vU = BuildFlatKnots(rCurve.dKnots, rCurve.iMults, iP, rCurve.bPeriodic);
    }
    if (static_cast<int>(vU.size()) != iN + iP + 1)
    {
        return vP.front();
    }

    // Clamp t to the valid evaluation domain [U[p], U[n]].
    const double dLo = vU[static_cast<size_t>(iP)];
    const double dHi = vU[static_cast<size_t>(iN)];
    if (t < dLo)
    {
        t = dLo;
    }
    if (t > dHi)
    {
        t = dHi;
    }

    // Find knot span k in [p, n-1] with U[k] <= t <= U[k+1] (NURBS Book A2.1).
    int iK;
    if (t >= vU[static_cast<size_t>(iN)])
    {
        iK = iN - 1;
    }
    else if (t <= vU[static_cast<size_t>(iP)])
    {
        iK = iP;
    }
    else
    {
        int iLow = iP;
        int iHigh = iN;
        iK = (iLow + iHigh) / 2;
        while (t < vU[static_cast<size_t>(iK)] || t >= vU[static_cast<size_t>(iK + 1)])
        {
            if (t < vU[static_cast<size_t>(iK)])
            {
                iHigh = iK;
            }
            else
            {
                iLow = iK;
            }
            iK = (iLow + iHigh) / 2;
        }
    }

    // de Boor on homogeneous coordinates.
    std::vector<pxr::GfVec3d> vD(static_cast<size_t>(iP + 1));
    std::vector<double> vDW(static_cast<size_t>(iP + 1));
    for (int j = 0; j <= iP; ++j)
    {
        const int idx = iK - iP + j;
        const double w = vW[static_cast<size_t>(idx)];
        vD[static_cast<size_t>(j)] = vP[static_cast<size_t>(idx)] * w;
        vDW[static_cast<size_t>(j)] = w;
    }
    for (int r = 1; r <= iP; ++r)
    {
        for (int j = iP; j >= r; --j)
        {
            const int idx = iK - iP + j;
            const double dDen = vU[static_cast<size_t>(idx + iP - r + 1)] - vU[static_cast<size_t>(idx)];
            const double a = (std::fabs(dDen) > 1e-12) ? (t - vU[static_cast<size_t>(idx)]) / dDen : 0.0;
            vD[static_cast<size_t>(j)] = (1.0 - a) * vD[static_cast<size_t>(j - 1)] + a * vD[static_cast<size_t>(j)];
            vDW[static_cast<size_t>(j)] = (1.0 - a) * vDW[static_cast<size_t>(j - 1)] + a * vDW[static_cast<size_t>(j)];
        }
    }
    pxr::GfVec3d sPt = vD[static_cast<size_t>(iP)];
    if (std::fabs(vDW[static_cast<size_t>(iP)]) > 1e-12)
    {
        sPt /= vDW[static_cast<size_t>(iP)];
    }
    return sPt;
}

// A NURBS curve in local (pre-transform) space: the building block for lowering swept surfaces.
struct SNurbCurveLocal
{
    int iDegree = 1;
    std::vector<pxr::GfVec3d> vPoles;
    std::vector<double> vWeights;
    std::vector<double> vKnots; // flat
    double dParamLo = 0.0;
    double dParamHi = 1.0;
};

// Lower a 3D curve to a local-space NURBS. Supports Line (degree 1), Bezier, and BSpline (periodic
// unrolled). pUHint bounds a Line's parameter range when present (a Line is unbounded; the face's
// U extent supplies the segment). Returns false for unsupported basis curve types.
bool LowerCurveToNurbLocal(const SCurve3d& rCurve, const double* pULo, const double* pUHi, SNurbCurveLocal& rOut)
{
    const SCurve3d* pCurve = &rCurve;
    double dTrimLo = 0.0;
    double dTrimHi = 0.0;
    bool bHaveTrim = false;
    while (pCurve && (pCurve->eType == ECurveType::Trimmed || pCurve->eType == ECurveType::Offset) && pCurve->pBasis)
    {
        if (pCurve->eType == ECurveType::Trimmed)
        {
            dTrimLo = pCurve->dFirst;
            dTrimHi = pCurve->dLast;
            bHaveTrim = true;
        }
        pCurve = pCurve->pBasis.get();
    }
    if (!pCurve)
    {
        return false;
    }

    switch (pCurve->eType)
    {
        case ECurveType::Line:
        {
            double dLo = bHaveTrim ? dTrimLo : 0.0;
            double dHi = bHaveTrim ? dTrimHi : 1.0;
            if (pULo)
            {
                dLo = *pULo;
            }
            if (pUHi)
            {
                dHi = *pUHi;
            }
            if (!(dHi > dLo))
            {
                dLo = 0.0;
                dHi = 1.0;
            }
            const pxr::GfVec3d sLoc = ToVec(pCurve->sLocation);
            const pxr::GfVec3d sDir = ToVec(pCurve->sXAxis);
            rOut.iDegree = 1;
            rOut.vPoles = { sLoc + dLo * sDir, sLoc + dHi * sDir };
            rOut.vWeights = { 1.0, 1.0 };
            rOut.vKnots = { dLo, dLo, dHi, dHi };
            rOut.dParamLo = dLo;
            rOut.dParamHi = dHi;
            return true;
        }
        case ECurveType::Bezier:
        case ECurveType::BSpline:
        {
            const int iP = pCurve->iDegree;
            const int iN0 = static_cast<int>(pCurve->sPoles.size());
            if (iP < 1 || iN0 < 1)
            {
                return false;
            }
            const int iExtra = PeriodicExtraPoles(pCurve->iMults, iP, pCurve->bPeriodic);
            const int iN = iN0 + iExtra;
            const bool bHaveW = pCurve->bRational && static_cast<int>(pCurve->dWeights.size()) == iN0;
            rOut.iDegree = iP;
            rOut.vPoles.clear();
            rOut.vWeights.clear();
            for (int i = 0; i < iN; ++i)
            {
                const int iSrc = (i < iN0) ? i : (i - iN0);
                rOut.vPoles.push_back(ToVec(pCurve->sPoles[static_cast<size_t>(iSrc)]));
                rOut.vWeights.push_back(bHaveW ? pCurve->dWeights[static_cast<size_t>(iSrc)] : 1.0);
            }
            pxr::VtArray<double> vU;
            if (pCurve->dKnots.empty() && iN0 == iP + 1)
            {
                for (int i = 0; i < iP + 1; ++i)
                {
                    vU.push_back(0.0);
                }
                for (int i = 0; i < iP + 1; ++i)
                {
                    vU.push_back(1.0);
                }
            }
            else
            {
                vU = BuildFlatKnots(pCurve->dKnots, pCurve->iMults, iP, pCurve->bPeriodic);
            }
            if (static_cast<int>(vU.size()) != iN + iP + 1)
            {
                return false;
            }
            rOut.vKnots.assign(vU.begin(), vU.end());
            rOut.dParamLo = vU[static_cast<size_t>(iP)];
            rOut.dParamHi = vU[static_cast<size_t>(iN)];
            return true;
        }
        default:
            return false;
    }
}

// Rotate a vector that is already perpendicular to the unit axis sAxis by dAngle (right-hand rule).
pxr::GfVec3d RotatePerp(const pxr::GfVec3d& rRadial, const pxr::GfVec3d& rAxis, double dAngle)
{
    return std::cos(dAngle) * rRadial + std::sin(dAngle) * pxr::GfCross(rAxis, rRadial);
}

// Emit a clamped (rational) quadratic Bezier as a 3-pole degree-2 NURBS curve over [t0, t1].
void EmitClampedQuadratic(
    const pxr::GfVec3d& rP0,
    const pxr::GfVec3d& rP1,
    const pxr::GfVec3d& rP2,
    double dW0,
    double dW1,
    double dW2,
    double t0,
    double t1,
    StagedCurveData& rOut
)
{
    rOut.eCurveType = StagedCurveType::BSplineCurve;
    rOut.uiNurbOrder = 3;
    rOut.uiNurbVertexCount = 3;
    rOut.vNurbControlVertices.clear();
    rOut.vNurbControlVertices.push_back(rP0);
    rOut.vNurbControlVertices.push_back(rP1);
    rOut.vNurbControlVertices.push_back(rP2);
    rOut.vNurbWeights.clear();
    rOut.vNurbWeights.push_back(dW0);
    rOut.vNurbWeights.push_back(dW1);
    rOut.vNurbWeights.push_back(dW2);
    rOut.vNurbKnots.clear();
    rOut.vNurbKnots.push_back(t0);
    rOut.vNurbKnots.push_back(t0);
    rOut.vNurbKnots.push_back(t0);
    rOut.vNurbKnots.push_back(t1);
    rOut.vNurbKnots.push_back(t1);
    rOut.vNurbKnots.push_back(t1);
}

// Parabola P(t) = O + (t^2/4F) X + t Y is exactly quadratic, so a non-rational quadratic Bezier over
// [t0, t1] is exact. The Bezier middle pole comes from the affine relation at the midpoint.
bool LowerParabolaToQuadratic(const SCurve3d& rC, const pxr::GfMatrix4d& rXf, double t0, double t1, StagedCurveData& rOut)
{
    if (!(t1 > t0) || rC.dFocal == 0.0)
    {
        return false;
    }
    const pxr::GfVec3d sO = ToVec(rC.sLocation);
    const pxr::GfVec3d sX = ToVec(rC.sXAxis);
    const pxr::GfVec3d sY = ToVec(rC.sYAxis);
    const double d4F = 4.0 * rC.dFocal;
    auto fEval = [&](double t)
    {
        return sO + (t * t / d4F) * sX + t * sY;
    };
    const pxr::GfVec3d sL0 = fEval(t0);
    const pxr::GfVec3d sL2 = fEval(t1);
    const pxr::GfVec3d sLm = fEval(0.5 * (t0 + t1));
    const pxr::GfVec3d sL1 = 2.0 * sLm - 0.5 * sL0 - 0.5 * sL2; // affine (coeffs sum to 1)
    EmitClampedQuadratic(rXf.Transform(sL0), rXf.Transform(sL1), rXf.Transform(sL2), 1.0, 1.0, 1.0, t0, t1, rOut);
    return true;
}

// Hyperbola P(t) = O + rMaj cosh(t) X + rMin sinh(t) Y. A conic arc is an exact rational quadratic
// Bezier: the middle pole is the intersection of the end tangents and its weight comes from the
// barycentric coordinates of an interior (shoulder) point. Computed in the planar (X,Y) frame.
bool LowerHyperbolaToQuadratic(const SCurve3d& rC, const pxr::GfMatrix4d& rXf, double t0, double t1, StagedCurveData& rOut)
{
    if (!(t1 > t0))
    {
        return false;
    }
    const double rA = rC.dRadius;
    const double rB = rC.dMinorRadius;
    auto fx = [&](double t)
    {
        return rA * std::cosh(t);
    };
    auto fy = [&](double t)
    {
        return rB * std::sinh(t);
    };
    auto dfx = [&](double t)
    {
        return rA * std::sinh(t);
    };
    auto dfy = [&](double t)
    {
        return rB * std::cosh(t);
    };

    const double x0 = fx(t0), y0 = fy(t0), x2 = fx(t1), y2 = fy(t1);
    const double tx0 = dfx(t0), ty0 = dfy(t0), tx2 = dfx(t1), ty2 = dfy(t1);

    // Intersection of the two end tangents: P0 + s*T0 = P2 + u*T2.
    const double dDet = -tx0 * ty2 + tx2 * ty0;
    if (std::fabs(dDet) < 1e-12)
    {
        return false;
    }
    const double s = ((x2 - x0) * (-ty2) - (-tx2) * (y2 - y0)) / dDet;
    const double p1x = x0 + s * tx0;
    const double p1y = y0 + s * ty0;

    // Shoulder point and its barycentric coordinates in triangle (P0, P1, P2).
    const double tm = 0.5 * (t0 + t1);
    const double qx = fx(tm), qy = fy(tm);
    const double m00 = x0 - x2, m01 = p1x - x2;
    const double m10 = y0 - y2, m11 = p1y - y2;
    const double dBary = m00 * m11 - m01 * m10;
    if (std::fabs(dBary) < 1e-12)
    {
        return false;
    }
    const double a = ((qx - x2) * m11 - m01 * (qy - y2)) / dBary;
    const double b = (m00 * (qy - y2) - (qx - x2) * m10) / dBary;
    const double c = 1.0 - a - b;
    if (a * c <= 1e-12)
    {
        return false;
    }
    const double dW1 = b / (2.0 * std::sqrt(a * c));

    const pxr::GfVec3d sO = ToVec(rC.sLocation);
    const pxr::GfVec3d sX = ToVec(rC.sXAxis);
    const pxr::GfVec3d sY = ToVec(rC.sYAxis);
    auto fMap = [&](double px, double py)
    {
        return sO + px * sX + py * sY;
    };
    EmitClampedQuadratic(rXf.Transform(fMap(x0, y0)), rXf.Transform(fMap(p1x, p1y)), rXf.Transform(fMap(x2, y2)), 1.0, dW1, 1.0, t0, t1, rOut);
    return true;
}

struct SNurbCurve2d
{
    int iDegree = 1;
    std::vector<pxr::GfVec3d> vHomogeneousPoles; // (u*w, v*w, w)
    std::vector<double> vKnots;
};

bool NearlyEqualKnot(double a, double b)
{
    return std::abs(a - b) <= 1.0e-10 * std::max({ 1.0, std::abs(a), std::abs(b) });
}

bool IsValidNurbCurve2d(const SNurbCurve2d& rCurve)
{
    const int iCount = static_cast<int>(rCurve.vHomogeneousPoles.size());
    if (rCurve.iDegree < 1 || iCount <= rCurve.iDegree || rCurve.vKnots.size() != static_cast<size_t>(iCount + rCurve.iDegree + 1))
    {
        return false;
    }
    for (size_t ii = 0; ii < rCurve.vKnots.size(); ++ii)
    {
        if (!std::isfinite(rCurve.vKnots[ii]) || (ii > 0 && rCurve.vKnots[ii] < rCurve.vKnots[ii - 1]))
        {
            return false;
        }
    }
    for (const pxr::GfVec3d& rPole : rCurve.vHomogeneousPoles)
    {
        if (!std::isfinite(rPole[0]) || !std::isfinite(rPole[1]) || !std::isfinite(rPole[2]) || rPole[2] <= 0.0)
        {
            return false;
        }
    }
    return true;
}

int FindNurbSpan(const SNurbCurve2d& rCurve, double dParameter)
{
    const int p = rCurve.iDegree;
    const int n = static_cast<int>(rCurve.vHomogeneousPoles.size()) - 1;
    if (dParameter >= rCurve.vKnots[static_cast<size_t>(n + 1)] || NearlyEqualKnot(dParameter, rCurve.vKnots[static_cast<size_t>(n + 1)]))
    {
        return n;
    }
    if (dParameter <= rCurve.vKnots[static_cast<size_t>(p)] || NearlyEqualKnot(dParameter, rCurve.vKnots[static_cast<size_t>(p)]))
    {
        return p;
    }

    int iLow = p;
    int iHigh = n + 1;
    int iMid = (iLow + iHigh) / 2;
    while (dParameter < rCurve.vKnots[static_cast<size_t>(iMid)] || dParameter >= rCurve.vKnots[static_cast<size_t>(iMid + 1)])
    {
        if (dParameter < rCurve.vKnots[static_cast<size_t>(iMid)])
        {
            iHigh = iMid;
        }
        else
        {
            iLow = iMid;
        }
        iMid = (iLow + iHigh) / 2;
    }
    return iMid;
}

int KnotMultiplicity(const std::vector<double>& rKnots, double dParameter)
{
    return static_cast<int>(std::count_if(
        rKnots.begin(),
        rKnots.end(),
        [dParameter](double dKnot)
        {
            return NearlyEqualKnot(dKnot, dParameter);
        }
    ));
}

bool InsertNurbKnotOnce(SNurbCurve2d& rCurve, double dParameter)
{
    const int p = rCurve.iDegree;
    const int n = static_cast<int>(rCurve.vHomogeneousPoles.size()) - 1;
    const int m = n + p + 1;
    const int k = FindNurbSpan(rCurve, dParameter);
    const int s = KnotMultiplicity(rCurve.vKnots, dParameter);
    if (s >= p)
    {
        return false;
    }

    std::vector<pxr::GfVec3d> vPoles(static_cast<size_t>(n + 2));
    std::vector<double> vKnots(static_cast<size_t>(m + 2));
    for (int ii = 0; ii <= k; ++ii)
    {
        vKnots[static_cast<size_t>(ii)] = rCurve.vKnots[static_cast<size_t>(ii)];
    }
    vKnots[static_cast<size_t>(k + 1)] = dParameter;
    for (int ii = k + 1; ii <= m; ++ii)
    {
        vKnots[static_cast<size_t>(ii + 1)] = rCurve.vKnots[static_cast<size_t>(ii)];
    }

    for (int ii = 0; ii <= k - p; ++ii)
    {
        vPoles[static_cast<size_t>(ii)] = rCurve.vHomogeneousPoles[static_cast<size_t>(ii)];
    }
    for (int ii = k - s; ii <= n; ++ii)
    {
        vPoles[static_cast<size_t>(ii + 1)] = rCurve.vHomogeneousPoles[static_cast<size_t>(ii)];
    }
    for (int ii = k - p + 1; ii <= k - s; ++ii)
    {
        const double dDen = rCurve.vKnots[static_cast<size_t>(ii + p)] - rCurve.vKnots[static_cast<size_t>(ii)];
        if (std::abs(dDen) <= 1.0e-15)
        {
            return false;
        }
        const double dAlpha = (dParameter - rCurve.vKnots[static_cast<size_t>(ii)]) / dDen;
        vPoles[static_cast<size_t>(ii)] = (1.0 - dAlpha) * rCurve.vHomogeneousPoles[static_cast<size_t>(ii - 1)] +
                                          dAlpha * rCurve.vHomogeneousPoles[static_cast<size_t>(ii)];
    }

    rCurve.vHomogeneousPoles = std::move(vPoles);
    rCurve.vKnots = std::move(vKnots);
    return true;
}

bool SplitNurbCurve2d(const SNurbCurve2d& rInput, double dParameter, SNurbCurve2d& rLeft, SNurbCurve2d& rRight)
{
    rLeft = SNurbCurve2d{};
    rRight = SNurbCurve2d{};
    if (!IsValidNurbCurve2d(rInput))
    {
        return false;
    }

    SNurbCurve2d sWork = rInput;
    const int p = sWork.iDegree;
    const int iCount = static_cast<int>(sWork.vHomogeneousPoles.size());
    const double dLo = sWork.vKnots[static_cast<size_t>(p)];
    const double dHi = sWork.vKnots[static_cast<size_t>(iCount)];
    if (dParameter <= dLo || dParameter >= dHi || NearlyEqualKnot(dParameter, dLo) || NearlyEqualKnot(dParameter, dHi))
    {
        return false;
    }

    int s = KnotMultiplicity(sWork.vKnots, dParameter);
    if (s > p)
    {
        return false;
    }
    while (s < p)
    {
        if (!InsertNurbKnotOnce(sWork, dParameter))
        {
            return false;
        }
        ++s;
    }

    const auto sFirst = std::find_if(
        sWork.vKnots.begin(),
        sWork.vKnots.end(),
        [dParameter](double dKnot)
        {
            return NearlyEqualKnot(dKnot, dParameter);
        }
    );
    if (sFirst == sWork.vKnots.end())
    {
        return false;
    }
    const size_t uiFirst = static_cast<size_t>(std::distance(sWork.vKnots.begin(), sFirst));
    if (uiFirst == 0 || uiFirst + static_cast<size_t>(p) > sWork.vKnots.size() || uiFirst > sWork.vHomogeneousPoles.size())
    {
        return false;
    }

    rLeft.iDegree = p;
    rLeft.vHomogeneousPoles.assign(sWork.vHomogeneousPoles.begin(), sWork.vHomogeneousPoles.begin() + uiFirst);
    rLeft.vKnots.assign(sWork.vKnots.begin(), sWork.vKnots.begin() + uiFirst + static_cast<size_t>(p));
    rLeft.vKnots.push_back(dParameter);

    rRight.iDegree = p;
    rRight.vHomogeneousPoles.assign(sWork.vHomogeneousPoles.begin() + uiFirst - 1, sWork.vHomogeneousPoles.end());
    rRight.vKnots.push_back(dParameter);
    rRight.vKnots.insert(rRight.vKnots.end(), sWork.vKnots.begin() + uiFirst, sWork.vKnots.end());
    return IsValidNurbCurve2d(rLeft) && IsValidNurbCurve2d(rRight);
}

bool TrimNurbCurve2d(SNurbCurve2d& rCurve, double dFirst, double dLast)
{
    if (!IsValidNurbCurve2d(rCurve) || !std::isfinite(dFirst) || !std::isfinite(dLast) || dLast <= dFirst)
    {
        return false;
    }

    const int p = rCurve.iDegree;
    const int iCount = static_cast<int>(rCurve.vHomogeneousPoles.size());
    const double dDomainLo = rCurve.vKnots[static_cast<size_t>(p)];
    const double dDomainHi = rCurve.vKnots[static_cast<size_t>(iCount)];
    if ((dFirst < dDomainLo && !NearlyEqualKnot(dFirst, dDomainLo)) || (dLast > dDomainHi && !NearlyEqualKnot(dLast, dDomainHi)))
    {
        return false;
    }
    if (NearlyEqualKnot(dFirst, dDomainLo))
    {
        dFirst = dDomainLo;
    }
    if (NearlyEqualKnot(dLast, dDomainHi))
    {
        dLast = dDomainHi;
    }

    if (dLast < dDomainHi)
    {
        SNurbCurve2d sLeft;
        SNurbCurve2d sRight;
        if (!SplitNurbCurve2d(rCurve, dLast, sLeft, sRight))
        {
            return false;
        }
        rCurve = std::move(sLeft);
    }
    if (dFirst > dDomainLo)
    {
        SNurbCurve2d sLeft;
        SNurbCurve2d sRight;
        if (!SplitNurbCurve2d(rCurve, dFirst, sLeft, sRight))
        {
            return false;
        }
        rCurve = std::move(sRight);
    }
    return IsValidNurbCurve2d(rCurve);
}

void AppendHomogeneousPole(SNurbCurve2d& rCurve, const pxr::GfVec2d& rPoint, double dWeight)
{
    rCurve.vHomogeneousPoles.emplace_back(rPoint[0] * dWeight, rPoint[1] * dWeight, dWeight);
}

bool BuildFreeformNurbCurve2d(const SCurve2d& rCurve, double dFirst, double dLast, SNurbCurve2d& rOut)
{
    const int p = rCurve.iDegree;
    const int iCount = static_cast<int>(rCurve.sPoles.size());
    if (p < 1 || iCount <= p)
    {
        return false;
    }
    const bool bHaveWeights = rCurve.bRational && static_cast<int>(rCurve.dWeights.size()) == iCount;
    rOut = SNurbCurve2d{};
    rOut.iDegree = p;

    if (rCurve.eType == ECurveType::Bezier)
    {
        if (iCount != p + 1)
        {
            return false;
        }
        for (int ii = 0; ii < iCount; ++ii)
        {
            AppendHomogeneousPole(rOut, ToVec(rCurve.sPoles[static_cast<size_t>(ii)]), bHaveWeights ? rCurve.dWeights[static_cast<size_t>(ii)] : 1.0);
        }
        rOut.vKnots.insert(rOut.vKnots.end(), static_cast<size_t>(p + 1), 0.0);
        rOut.vKnots.insert(rOut.vKnots.end(), static_cast<size_t>(p + 1), 1.0);
    }
    else if (rCurve.eType == ECurveType::BSpline)
    {
        const int iExtra = PeriodicExtraPoles(rCurve.iMults, p, rCurve.bPeriodic);
        for (int ii = 0; ii < iCount + iExtra; ++ii)
        {
            const int iSource = ii < iCount ? ii : ii - iCount;
            AppendHomogeneousPole(
                rOut,
                ToVec(rCurve.sPoles[static_cast<size_t>(iSource)]),
                bHaveWeights ? rCurve.dWeights[static_cast<size_t>(iSource)] : 1.0
            );
        }
        const pxr::VtArray<double> vKnots = BuildFlatKnots(rCurve.dKnots, rCurve.iMults, p, rCurve.bPeriodic);
        rOut.vKnots.assign(vKnots.begin(), vKnots.end());
    }
    else
    {
        return false;
    }

    return TrimNurbCurve2d(rOut, dFirst, dLast);
}

bool BuildAngularNurbCurve2d(const SCurve2d& rCurve, double dFirst, double dLast, SNurbCurve2d& rOut)
{
    const double dSpan = dLast - dFirst;
    if (!std::isfinite(dFirst) || !std::isfinite(dLast) || dSpan <= 1.0e-12)
    {
        return false;
    }
    const int iSegmentCount = std::max(1, static_cast<int>(std::ceil(dSpan / (0.5 * M_PI) - 1.0e-10)));
    if (iSegmentCount > 4096)
    {
        return false;
    }
    const double dSegmentSpan = dSpan / static_cast<double>(iSegmentCount);
    const double dMidWeight = std::cos(0.5 * dSegmentSpan);
    if (dMidWeight <= 1.0e-12)
    {
        return false;
    }

    rOut = SNurbCurve2d{};
    rOut.iDegree = 2;
    const double dXRadius = rCurve.dRadius;
    const double dYRadius = rCurve.eType == ECurveType::Circle ? rCurve.dRadius : rCurve.dMinorRadius;
    const pxr::GfVec2d sCenter = ToVec(rCurve.sLocation);
    const pxr::GfVec2d sX = ToVec(rCurve.sXAxis);
    const pxr::GfVec2d sY = ToVec(rCurve.sYAxis);
    for (int ii = 0; ii <= 2 * iSegmentCount; ++ii)
    {
        const bool bMidpoint = (ii % 2) != 0;
        const double dAngle = dFirst + (0.5 * static_cast<double>(ii)) * dSegmentSpan;
        const double dWeight = bMidpoint ? dMidWeight : 1.0;
        const double dScale = bMidpoint ? (1.0 / dMidWeight) : 1.0;
        const pxr::GfVec2d sPoint = sCenter + dScale * (dXRadius * std::cos(dAngle) * sX + dYRadius * std::sin(dAngle) * sY);
        AppendHomogeneousPole(rOut, sPoint, dWeight);
    }

    rOut.vKnots = { dFirst, dFirst, dFirst };
    for (int ii = 1; ii < iSegmentCount; ++ii)
    {
        const double dKnot = dFirst + static_cast<double>(ii) * dSegmentSpan;
        rOut.vKnots.push_back(dKnot);
        rOut.vKnots.push_back(dKnot);
    }
    rOut.vKnots.push_back(dLast);
    rOut.vKnots.push_back(dLast);
    rOut.vKnots.push_back(dLast);
    return IsValidNurbCurve2d(rOut);
}

void EmitQuadraticNurbCurve2d(
    const pxr::GfVec2d& rP0,
    const pxr::GfVec2d& rP1,
    const pxr::GfVec2d& rP2,
    double dW1,
    double dFirst,
    double dLast,
    SNurbCurve2d& rOut
)
{
    rOut = SNurbCurve2d{};
    rOut.iDegree = 2;
    AppendHomogeneousPole(rOut, rP0, 1.0);
    AppendHomogeneousPole(rOut, rP1, dW1);
    AppendHomogeneousPole(rOut, rP2, 1.0);
    rOut.vKnots = { dFirst, dFirst, dFirst, dLast, dLast, dLast };
}

bool BuildParabolaNurbCurve2d(const SCurve2d& rCurve, double dFirst, double dLast, SNurbCurve2d& rOut)
{
    if (dLast <= dFirst || rCurve.dFocal == 0.0)
    {
        return false;
    }
    const pxr::GfVec2d sOrigin = ToVec(rCurve.sLocation);
    const pxr::GfVec2d sX = ToVec(rCurve.sXAxis);
    const pxr::GfVec2d sY = ToVec(rCurve.sYAxis);
    const auto fEval = [&](double t)
    {
        return sOrigin + (t * t / (4.0 * rCurve.dFocal)) * sX + t * sY;
    };
    const pxr::GfVec2d sP0 = fEval(dFirst);
    const pxr::GfVec2d sP2 = fEval(dLast);
    const pxr::GfVec2d sMid = fEval(0.5 * (dFirst + dLast));
    EmitQuadraticNurbCurve2d(sP0, 2.0 * sMid - 0.5 * sP0 - 0.5 * sP2, sP2, 1.0, dFirst, dLast, rOut);
    return true;
}

bool BuildHyperbolaNurbCurve2d(const SCurve2d& rCurve, double dFirst, double dLast, SNurbCurve2d& rOut)
{
    if (dLast <= dFirst)
    {
        return false;
    }
    const double a = rCurve.dRadius;
    const double b = rCurve.dMinorRadius;
    const auto fx = [a](double t)
    {
        return a * std::cosh(t);
    };
    const auto fy = [b](double t)
    {
        return b * std::sinh(t);
    };
    const auto dfx = [a](double t)
    {
        return a * std::sinh(t);
    };
    const auto dfy = [b](double t)
    {
        return b * std::cosh(t);
    };

    const double x0 = fx(dFirst), y0 = fy(dFirst), x2 = fx(dLast), y2 = fy(dLast);
    const double tx0 = dfx(dFirst), ty0 = dfy(dFirst), tx2 = dfx(dLast), ty2 = dfy(dLast);
    const double dDet = -tx0 * ty2 + tx2 * ty0;
    if (std::abs(dDet) < 1.0e-12)
    {
        return false;
    }
    const double s = ((x2 - x0) * (-ty2) - (-tx2) * (y2 - y0)) / dDet;
    const double p1x = x0 + s * tx0;
    const double p1y = y0 + s * ty0;

    const double dMid = 0.5 * (dFirst + dLast);
    const double qx = fx(dMid), qy = fy(dMid);
    const double m00 = x0 - x2, m01 = p1x - x2;
    const double m10 = y0 - y2, m11 = p1y - y2;
    const double dBary = m00 * m11 - m01 * m10;
    if (std::abs(dBary) < 1.0e-12)
    {
        return false;
    }
    const double dBaryA = ((qx - x2) * m11 - m01 * (qy - y2)) / dBary;
    const double dBaryB = (m00 * (qy - y2) - (qx - x2) * m10) / dBary;
    const double dBaryC = 1.0 - dBaryA - dBaryB;
    if (dBaryA * dBaryC <= 1.0e-12)
    {
        return false;
    }
    const double dWeight = dBaryB / (2.0 * std::sqrt(dBaryA * dBaryC));
    if (dWeight <= 0.0)
    {
        return false;
    }

    const pxr::GfVec2d sOrigin = ToVec(rCurve.sLocation);
    const pxr::GfVec2d sX = ToVec(rCurve.sXAxis);
    const pxr::GfVec2d sY = ToVec(rCurve.sYAxis);
    const auto fMap = [&](double x, double y)
    {
        return sOrigin + x * sX + y * sY;
    };
    EmitQuadraticNurbCurve2d(fMap(x0, y0), fMap(p1x, p1y), fMap(x2, y2), dWeight, dFirst, dLast, rOut);
    return true;
}

bool CopyNurbCurve2dToStaging(const SNurbCurve2d& rCurve, const pxr::GfVec2d& rScale, const pxr::GfVec2d& rTranslation, StagedCurveData2d& rOut)
{
    if (!IsValidNurbCurve2d(rCurve))
    {
        return false;
    }
    rOut = StagedCurveData2d{};
    rOut.uiNurbOrder = static_cast<uint32_t>(rCurve.iDegree + 1);
    rOut.uiNurbVertexCount = static_cast<uint32_t>(rCurve.vHomogeneousPoles.size());
    rOut.vNurbKnots.assign(rCurve.vKnots.begin(), rCurve.vKnots.end());
    rOut.vNurbControlVertices.reserve(rCurve.vHomogeneousPoles.size());
    rOut.vNurbWeights.reserve(rCurve.vHomogeneousPoles.size());
    for (const pxr::GfVec3d& rPole : rCurve.vHomogeneousPoles)
    {
        const double dWeight = rPole[2];
        const pxr::GfVec2d sPoint(rPole[0] / dWeight, rPole[1] / dWeight);
        const pxr::GfVec2d sMapped(sPoint[0] * rScale[0] + rTranslation[0], sPoint[1] * rScale[1] + rTranslation[1]);
        if (!std::isfinite(sMapped[0]) || !std::isfinite(sMapped[1]))
        {
            return false;
        }
        rOut.vNurbControlVertices.push_back(sMapped);
        rOut.vNurbWeights.push_back(dWeight);
    }
    rOut.bValid = true;
    return true;
}

} // namespace

pxr::GfMatrix4d ResolveLocation(const SOcctModel& rModel, int iLocationIndex)
{
    if (iLocationIndex <= 0 || static_cast<size_t>(iLocationIndex) > rModel.sLocations.size())
    {
        return pxr::GfMatrix4d(1.0);
    }

    const SLocation& rLoc = rModel.sLocations[static_cast<size_t>(iLocationIndex - 1)];
    if (rLoc.bElementary)
    {
        return FromOcctMatrix(rLoc.dMatrix);
    }

    // Composite: product of referenced datum powers. OCCT reads the term list (in file order
    // t1..tN) as L = D(tN)^pN * ... * D(t1)^p1 with each new term prepended on the LEFT (see
    // TopTools_LocationSet::Read: `L = L1.Powered(p) * L`). That is a column-vector product. In
    // USD's row-vector convention (point' = point * M) the equivalent matrix is the transpose,
    // which reverses the operand order to U(t1)^p1 * U(t2)^p2 * ... * U(tN)^pN, i.e. file order
    // appended on the RIGHT. Each ResolveLocation term is already in row-vector form.
    pxr::GfMatrix4d sResult(1.0);
    for (const SLocationTerm& rTerm : rLoc.sTerms)
    {
        pxr::GfMatrix4d sTerm = ResolveLocation(rModel, rTerm.iLocationIndex);
        int iPower = rTerm.iPower;
        if (iPower < 0)
        {
            sTerm = sTerm.GetInverse();
            iPower = -iPower;
        }
        pxr::GfMatrix4d sAccum(1.0);
        for (int p = 0; p < iPower; ++p)
        {
            sAccum = sAccum * sTerm;
        }
        sResult = sResult * sAccum;
    }
    return sResult;
}

bool ConvertCurve3d(const SCurve3d& rCurve, const pxr::GfMatrix4d& rTransform, StagedCurveData& rOut, const pxr::GfRange1d* pInterval)
{
    // Bounding interval for unbounded analytic curves (parabola/hyperbola): prefer the edge interval,
    // else a Trimmed wrapper's range.
    double dLo = 0.0, dHi = 0.0;
    bool bRange = false;
    if (pInterval && pInterval->GetMax() > pInterval->GetMin())
    {
        dLo = pInterval->GetMin();
        dHi = pInterval->GetMax();
        bRange = true;
    }

    // Resolve Trimmed/Offset down to the underlying basis curve (range/offset handled elsewhere).
    const SCurve3d* pCurve = &rCurve;
    while (pCurve && (pCurve->eType == ECurveType::Trimmed || pCurve->eType == ECurveType::Offset))
    {
        if (pCurve->eType == ECurveType::Trimmed && !bRange && pCurve->dLast > pCurve->dFirst)
        {
            dLo = pCurve->dFirst;
            dHi = pCurve->dLast;
            bRange = true;
        }
        pCurve = pCurve->pBasis.get();
    }
    if (!pCurve)
    {
        return false;
    }

    switch (pCurve->eType)
    {
        case ECurveType::Line:
            rOut.eCurveType = StagedCurveType::Line;
            rOut.sOrigin = Place(rTransform, pCurve->sLocation);
            rOut.sDirection = Orient(rTransform, pCurve->sXAxis);
            return true;

        case ECurveType::Circle:
            rOut.eCurveType = StagedCurveType::Circle;
            rOut.sCenter = Place(rTransform, pCurve->sLocation);
            rOut.sAxis = Orient(rTransform, pCurve->sAxis);
            rOut.sRefDirection = Orient(rTransform, pCurve->sXAxis);
            rOut.dRadius = pCurve->dRadius;
            return true;

        case ECurveType::Ellipse:
            rOut.eCurveType = StagedCurveType::Ellipse;
            rOut.sCenter = Place(rTransform, pCurve->sLocation);
            rOut.sAxis = Orient(rTransform, pCurve->sAxis);
            rOut.sRefDirection = Orient(rTransform, pCurve->sXAxis);
            rOut.dXRadius = pCurve->dRadius;
            rOut.dYRadius = pCurve->dMinorRadius;
            return true;

        case ECurveType::BSpline:
        {
            rOut.eCurveType = StagedCurveType::BSplineCurve;
            const int iN = static_cast<int>(pCurve->sPoles.size());
            const int iExtra = PeriodicExtraPoles(pCurve->iMults, pCurve->iDegree, pCurve->bPeriodic);
            const int iNewN = iN + iExtra;
            const bool bHaveWeights = pCurve->bRational && pCurve->dWeights.size() == pCurve->sPoles.size();

            rOut.uiNurbOrder = static_cast<uint32_t>(pCurve->iDegree + 1);
            rOut.uiNurbVertexCount = static_cast<uint32_t>(iNewN);
            rOut.vNurbControlVertices.clear();
            rOut.vNurbWeights.clear();
            // For a periodic curve the poles wrap modulo iN, so the appended poles are the first
            // iExtra poles. This is the unclamped form that is geometrically identical to the
            // periodic curve over its valid parameter range.
            for (int i = 0; i < iNewN; ++i)
            {
                const int iSrc = (i < iN) ? i : (i - iN);
                rOut.vNurbControlVertices.push_back(Place(rTransform, pCurve->sPoles[static_cast<size_t>(iSrc)]));
                rOut.vNurbWeights.push_back(bHaveWeights ? pCurve->dWeights[static_cast<size_t>(iSrc)] : 1.0);
            }
            rOut.vNurbKnots = BuildFlatKnots(pCurve->dKnots, pCurve->iMults, pCurve->iDegree, pCurve->bPeriodic);
            return true;
        }

        case ECurveType::Bezier:
        {
            const int iDeg = pCurve->iDegree;
            const int iN = static_cast<int>(pCurve->sPoles.size());
            if (iDeg < 1 || iN != iDeg + 1)
            {
                return false;
            }
            const bool bHaveWeights = pCurve->bRational && static_cast<int>(pCurve->dWeights.size()) == iN;
            rOut.eCurveType = StagedCurveType::BSplineCurve;
            rOut.uiNurbOrder = static_cast<uint32_t>(iDeg + 1);
            rOut.uiNurbVertexCount = static_cast<uint32_t>(iN);
            rOut.vNurbControlVertices.clear();
            rOut.vNurbWeights.clear();
            for (int i = 0; i < iN; ++i)
            {
                rOut.vNurbControlVertices.push_back(Place(rTransform, pCurve->sPoles[static_cast<size_t>(i)]));
                rOut.vNurbWeights.push_back(bHaveWeights ? pCurve->dWeights[static_cast<size_t>(i)] : 1.0);
            }
            rOut.vNurbKnots.clear();
            for (int i = 0; i < iDeg + 1; ++i)
            {
                rOut.vNurbKnots.push_back(0.0);
            }
            for (int i = 0; i < iDeg + 1; ++i)
            {
                rOut.vNurbKnots.push_back(1.0);
            }
            return true;
        }

        case ECurveType::Parabola:
            return bRange && LowerParabolaToQuadratic(*pCurve, rTransform, dLo, dHi, rOut);

        case ECurveType::Hyperbola:
            return bRange && LowerHyperbolaToQuadratic(*pCurve, rTransform, dLo, dHi, rOut);

        default:
            return false;
    }
}

bool ConvertCurve2dToNurb(
    const SCurve2d& rCurve,
    const pxr::GfRange1d& rInterval,
    const pxr::GfVec2d& rScale,
    const pxr::GfVec2d& rTranslation,
    StagedCurveData2d& rOut
)
{
    rOut = StagedCurveData2d{};
    const double dFirst = rInterval.GetMin();
    const double dLast = rInterval.GetMax();
    if (!std::isfinite(dFirst) || !std::isfinite(dLast) || dLast <= dFirst || !std::isfinite(rScale[0]) || !std::isfinite(rScale[1]) ||
        !std::isfinite(rTranslation[0]) || !std::isfinite(rTranslation[1]))
    {
        return false;
    }

    const SCurve2d* pCurve = &rCurve;
    while (pCurve && (pCurve->eType == ECurveType::Trimmed || pCurve->eType == ECurveType::Offset))
    {
        if (pCurve->eType == ECurveType::Offset && std::abs(pCurve->dOffset) > 1.0e-14)
        {
            return false;
        }
        pCurve = pCurve->pBasis.get();
    }
    if (!pCurve)
    {
        return false;
    }

    SNurbCurve2d sNurb;
    switch (pCurve->eType)
    {
        case ECurveType::Line:
        {
            sNurb.iDegree = 1;
            const pxr::GfVec2d sOrigin = ToVec(pCurve->sLocation);
            const pxr::GfVec2d sDirection = ToVec(pCurve->sXAxis);
            AppendHomogeneousPole(sNurb, sOrigin + dFirst * sDirection, 1.0);
            AppendHomogeneousPole(sNurb, sOrigin + dLast * sDirection, 1.0);
            sNurb.vKnots = { dFirst, dFirst, dLast, dLast };
            break;
        }
        case ECurveType::Circle:
        case ECurveType::Ellipse:
            if (!BuildAngularNurbCurve2d(*pCurve, dFirst, dLast, sNurb))
            {
                return false;
            }
            break;
        case ECurveType::Parabola:
            if (!BuildParabolaNurbCurve2d(*pCurve, dFirst, dLast, sNurb))
            {
                return false;
            }
            break;
        case ECurveType::Hyperbola:
            if (!BuildHyperbolaNurbCurve2d(*pCurve, dFirst, dLast, sNurb))
            {
                return false;
            }
            break;
        case ECurveType::Bezier:
        case ECurveType::BSpline:
            if (!BuildFreeformNurbCurve2d(*pCurve, dFirst, dLast, sNurb))
            {
                return false;
            }
            break;
        default:
            return false;
    }
    return CopyNurbCurve2dToStaging(sNurb, rScale, rTranslation, rOut);
}

bool ConvertSurface(
    const SSurface& rSurface,
    const pxr::GfMatrix4d& rTransform,
    StagedSurfaceData& rOut,
    const pxr::GfRange2d* pUVHint,
    pxr::GfRange2d* pSweptDomainOut
)
{
    // Resolve RectangularTrimmed/Offset down to the underlying basis surface.
    const SSurface* pSurface = &rSurface;
    while (pSurface && (pSurface->eType == ESurfaceType::RectangularTrimmed || pSurface->eType == ESurfaceType::Offset))
    {
        pSurface = pSurface->pBasisSurface.get();
    }
    if (!pSurface)
    {
        return false;
    }

    switch (pSurface->eType)
    {
        case ESurfaceType::Plane:
            rOut.eSurfaceType = StagedSurfaceType::Plane;
            rOut.sOrigin = Place(rTransform, pSurface->sLocation);
            rOut.sAxis = Orient(rTransform, pSurface->sAxis);
            rOut.sRefDirection = Orient(rTransform, pSurface->sXAxis);
            return true;

        case ESurfaceType::Cylinder:
            rOut.eSurfaceType = StagedSurfaceType::Cylinder;
            rOut.sOrigin = Place(rTransform, pSurface->sLocation);
            rOut.sAxis = Orient(rTransform, pSurface->sAxis);
            rOut.sRefDirection = Orient(rTransform, pSurface->sXAxis);
            rOut.dRadius = pSurface->dRadius;
            return true;

        case ESurfaceType::Cone:
            rOut.eSurfaceType = StagedSurfaceType::Cone;
            rOut.sOrigin = Place(rTransform, pSurface->sLocation);
            rOut.sRefDirection = Orient(rTransform, pSurface->sXAxis);
            rOut.dRadius = pSurface->dRadius;
            // USD requires a half-angle in (0, pi/2). OCCT may store a negative half-angle (radius
            // shrinking along +axis); flipping the axis and negating the angle is the same surface.
            if (pSurface->dSemiAngle < 0.0)
            {
                rOut.sAxis = -Orient(rTransform, pSurface->sAxis);
                rOut.dSemiAngle = -pSurface->dSemiAngle;
            }
            else
            {
                rOut.sAxis = Orient(rTransform, pSurface->sAxis);
                rOut.dSemiAngle = pSurface->dSemiAngle;
            }
            return true;

        case ESurfaceType::Sphere:
            rOut.eSurfaceType = StagedSurfaceType::Sphere;
            rOut.sCenter = Place(rTransform, pSurface->sLocation);
            rOut.sAxis = Orient(rTransform, pSurface->sAxis);
            rOut.sRefDirection = Orient(rTransform, pSurface->sXAxis);
            rOut.dRadius = pSurface->dRadius;
            return true;

        case ESurfaceType::Torus:
            rOut.eSurfaceType = StagedSurfaceType::Torus;
            rOut.sOrigin = Place(rTransform, pSurface->sLocation);
            rOut.sAxis = Orient(rTransform, pSurface->sAxis);
            rOut.sRefDirection = Orient(rTransform, pSurface->sXAxis);
            rOut.dMajorRadius = pSurface->dRadius;
            rOut.dMinorRadius = pSurface->dMinorRadius;
            return true;

        case ESurfaceType::BSpline:
        {
            rOut.eSurfaceType = StagedSurfaceType::BSplineSurface;

            const int iNbU = pSurface->iNbUPoles;
            const int iNbV = pSurface->iNbVPoles;
            const int iExtraU = PeriodicExtraPoles(pSurface->iUMults, pSurface->iUDegree, pSurface->bUPeriodic);
            const int iExtraV = PeriodicExtraPoles(pSurface->iVMults, pSurface->iVDegree, pSurface->bVPeriodic);
            const int iNewU = iNbU + iExtraU;
            const int iNewV = iNbV + iExtraV;
            const bool bRational = pSurface->bURational || pSurface->bVRational;
            const bool bHaveWeights = bRational && pSurface->dWeights.size() == pSurface->sPoles.size() &&
                                      static_cast<int>(pSurface->sPoles.size()) == iNbU * iNbV;

            rOut.uiNurbUOrder = static_cast<uint32_t>(pSurface->iUDegree + 1);
            rOut.uiNurbVOrder = static_cast<uint32_t>(pSurface->iVDegree + 1);
            rOut.uiNurbUVertexCount = static_cast<uint32_t>(iNewU);
            rOut.uiNurbVVertexCount = static_cast<uint32_t>(iNewV);
            rOut.vNurbControlVertices.clear();
            rOut.vNurbWeights.clear();

            // Poles are stored U-major (index = iu*nbV + iv). Periodic directions wrap modulo the
            // original pole count, so the appended rows/columns repeat the leading poles. Emitting
            // the grid U-major preserves the layout the staging layer expects.
            const bool bValidGrid = (static_cast<int>(pSurface->sPoles.size()) == iNbU * iNbV) && iNbU > 0 && iNbV > 0;
            if (bValidGrid)
            {
                for (int iu = 0; iu < iNewU; ++iu)
                {
                    const int iSrcU = (iu < iNbU) ? iu : (iu - iNbU);
                    for (int iv = 0; iv < iNewV; ++iv)
                    {
                        const int iSrcV = (iv < iNbV) ? iv : (iv - iNbV);
                        const size_t uiSrc = static_cast<size_t>(iSrcU) * static_cast<size_t>(iNbV) + static_cast<size_t>(iSrcV);
                        rOut.vNurbControlVertices.push_back(Place(rTransform, pSurface->sPoles[uiSrc]));
                        rOut.vNurbWeights.push_back(bHaveWeights ? pSurface->dWeights[uiSrc] : 1.0);
                    }
                }
            }
            else
            {
                // Fallback: emit as-is (no unrolling) if the pole grid is inconsistent.
                for (const SPnt3& rPole : pSurface->sPoles)
                {
                    rOut.vNurbControlVertices.push_back(Place(rTransform, rPole));
                }
                rOut.vNurbWeights.assign(pSurface->sPoles.size(), 1.0);
                rOut.uiNurbUVertexCount = static_cast<uint32_t>(iNbU);
                rOut.uiNurbVVertexCount = static_cast<uint32_t>(iNbV);
            }

            rOut.vNurbUKnots = BuildFlatKnots(pSurface->dUKnots, pSurface->iUMults, pSurface->iUDegree, pSurface->bUPeriodic);
            rOut.vNurbVKnots = BuildFlatKnots(pSurface->dVKnots, pSurface->iVMults, pSurface->iVDegree, pSurface->bVPeriodic);
            return true;
        }

        case ESurfaceType::Bezier:
        {
            // A Bezier surface is a single-segment clamped NURBS surface.
            const int iNbU = pSurface->iNbUPoles;
            const int iNbV = pSurface->iNbVPoles;
            const int iDU = pSurface->iUDegree;
            const int iDV = pSurface->iVDegree;
            if (iNbU != iDU + 1 || iNbV != iDV + 1 || static_cast<int>(pSurface->sPoles.size()) != iNbU * iNbV)
            {
                return false;
            }
            const bool bHaveWeights = (pSurface->bURational || pSurface->bVRational) && static_cast<int>(pSurface->dWeights.size()) == iNbU * iNbV;
            rOut.eSurfaceType = StagedSurfaceType::BSplineSurface;
            rOut.uiNurbUOrder = static_cast<uint32_t>(iDU + 1);
            rOut.uiNurbVOrder = static_cast<uint32_t>(iDV + 1);
            rOut.uiNurbUVertexCount = static_cast<uint32_t>(iNbU);
            rOut.uiNurbVVertexCount = static_cast<uint32_t>(iNbV);
            rOut.vNurbControlVertices.clear();
            rOut.vNurbWeights.clear();
            // Poles stored U-major (index = iu*nbV + iv) -- emit in the same order.
            for (size_t i = 0; i < pSurface->sPoles.size(); ++i)
            {
                rOut.vNurbControlVertices.push_back(Place(rTransform, pSurface->sPoles[i]));
                rOut.vNurbWeights.push_back(bHaveWeights ? pSurface->dWeights[i] : 1.0);
            }
            rOut.vNurbUKnots.clear();
            for (int i = 0; i < iDU + 1; ++i)
            {
                rOut.vNurbUKnots.push_back(0.0);
            }
            for (int i = 0; i < iDU + 1; ++i)
            {
                rOut.vNurbUKnots.push_back(1.0);
            }
            rOut.vNurbVKnots.clear();
            for (int i = 0; i < iDV + 1; ++i)
            {
                rOut.vNurbVKnots.push_back(0.0);
            }
            for (int i = 0; i < iDV + 1; ++i)
            {
                rOut.vNurbVKnots.push_back(1.0);
            }
            return true;
        }

        case ESurfaceType::LinearExtrusion:
        {
            // S(u,v) = C(u) + v * dir. U is the basis curve's NURBS; V is exactly linear (degree 1),
            // so two V layers at the face's V bounds reproduce the surface exactly.
            if (!pSurface->pBasisCurve)
            {
                return false;
            }
            double dULo = 0.0, dUHi = 0.0;
            const bool bHaveU = pUVHint && !pUVHint->IsEmpty();
            if (bHaveU)
            {
                dULo = pUVHint->GetMin()[0];
                dUHi = pUVHint->GetMax()[0];
            }
            SNurbCurveLocal sBasis;
            if (!LowerCurveToNurbLocal(*pSurface->pBasisCurve, bHaveU ? &dULo : nullptr, bHaveU ? &dUHi : nullptr, sBasis))
            {
                return false;
            }

            double dV0 = 0.0, dV1 = 1.0;
            if (pUVHint && !pUVHint->IsEmpty() && pUVHint->GetMax()[1] > pUVHint->GetMin()[1])
            {
                dV0 = pUVHint->GetMin()[1];
                dV1 = pUVHint->GetMax()[1];
            }
            const pxr::GfVec3d sDir = ToVec(pSurface->sDirection);
            const int iNbU = static_cast<int>(sBasis.vPoles.size());

            rOut.eSurfaceType = StagedSurfaceType::BSplineSurface;
            rOut.uiNurbUOrder = static_cast<uint32_t>(sBasis.iDegree + 1);
            rOut.uiNurbVOrder = 2;
            rOut.uiNurbUVertexCount = static_cast<uint32_t>(iNbU);
            rOut.uiNurbVVertexCount = 2;
            rOut.vNurbControlVertices.clear();
            rOut.vNurbWeights.clear();
            for (int iu = 0; iu < iNbU; ++iu)
            {
                const pxr::GfVec3d sP = sBasis.vPoles[static_cast<size_t>(iu)];
                rOut.vNurbControlVertices.push_back(rTransform.Transform(sP + dV0 * sDir));
                rOut.vNurbControlVertices.push_back(rTransform.Transform(sP + dV1 * sDir));
                rOut.vNurbWeights.push_back(sBasis.vWeights[static_cast<size_t>(iu)]);
                rOut.vNurbWeights.push_back(sBasis.vWeights[static_cast<size_t>(iu)]);
            }
            rOut.vNurbUKnots.assign(sBasis.vKnots.begin(), sBasis.vKnots.end());
            rOut.vNurbVKnots = { dV0, dV0, dV1, dV1 };
            if (pSweptDomainOut)
            {
                *pSweptDomainOut = pxr::GfRange2d(pxr::GfVec2d(sBasis.dParamLo, dV0), pxr::GfVec2d(sBasis.dParamHi, dV1));
            }
            return true;
        }

        case ESurfaceType::Revolution:
        {
            // OCCT convention: S(u,v) = rotate(C(v), u) about the axis -- U is the revolution angle,
            // V is the basis (meridian) parameter. U is a circular arc represented as a rational
            // quadratic (degree 2) so the swept shape is exact; V is the basis NURBS. The arc
            // parameter is not linear in angle, but no UV pcurves are emitted, so only the geometry
            // and the natural param box (used as the face range) matter.
            if (!pSurface->pBasisCurve)
            {
                return false;
            }
            SNurbCurveLocal sBasis;
            if (!LowerCurveToNurbLocal(*pSurface->pBasisCurve, nullptr, nullptr, sBasis))
            {
                return false;
            }

            double dA0 = 0.0, dA1 = 2.0 * M_PI;
            if (pUVHint && !pUVHint->IsEmpty() && pUVHint->GetMax()[0] > pUVHint->GetMin()[0])
            {
                dA0 = pUVHint->GetMin()[0];
                dA1 = pUVHint->GetMax()[0];
            }
            const double dSpan = dA1 - dA0;
            const int iNSeg = std::max(1, static_cast<int>(std::ceil(dSpan / (M_PI / 2.0) - 1e-9)));
            const double dSeg = dSpan / iNSeg;
            const double dHalfW = std::cos(dSeg / 2.0);
            const int iNbU = 2 * iNSeg + 1; // arc poles (angle)
            const int iNbV = static_cast<int>(sBasis.vPoles.size()); // basis poles (meridian)

            const pxr::GfVec3d sAxisP = ToVec(pSurface->sLocation);
            pxr::GfVec3d sAxisD = ToVec(pSurface->sDirection);
            sAxisD.Normalize();

            // Precompute, per basis pole, its projection onto the axis and the radial offset.
            std::vector<pxr::GfVec3d> vFoot(static_cast<size_t>(iNbV));
            std::vector<pxr::GfVec3d> vRadial(static_cast<size_t>(iNbV));
            for (int iv = 0; iv < iNbV; ++iv)
            {
                const pxr::GfVec3d sP = sBasis.vPoles[static_cast<size_t>(iv)];
                vFoot[static_cast<size_t>(iv)] = sAxisP + pxr::GfDot(sP - sAxisP, sAxisD) * sAxisD;
                vRadial[static_cast<size_t>(iv)] = sP - vFoot[static_cast<size_t>(iv)];
            }

            // Angle and weight for each U (arc) pole index: even = on-curve (angle a0+s*seg, weight 1),
            // odd = tangent-intersection control pole (mid angle, weight cos(seg/2), radial scaled 1/w).
            std::vector<double> vAng(static_cast<size_t>(iNbU));
            std::vector<double> vArcW(static_cast<size_t>(iNbU));
            std::vector<double> vScale(static_cast<size_t>(iNbU));
            for (int iu = 0; iu < iNbU; ++iu)
            {
                const int s = iu / 2;
                if (iu % 2 == 0)
                {
                    vAng[static_cast<size_t>(iu)] = dA0 + s * dSeg;
                    vArcW[static_cast<size_t>(iu)] = 1.0;
                    vScale[static_cast<size_t>(iu)] = 1.0;
                }
                else
                {
                    vAng[static_cast<size_t>(iu)] = dA0 + (s + 0.5) * dSeg;
                    vArcW[static_cast<size_t>(iu)] = dHalfW;
                    vScale[static_cast<size_t>(iu)] = 1.0 / dHalfW;
                }
            }

            rOut.eSurfaceType = StagedSurfaceType::BSplineSurface;
            rOut.uiNurbUOrder = 3;
            rOut.uiNurbVOrder = static_cast<uint32_t>(sBasis.iDegree + 1);
            rOut.uiNurbUVertexCount = static_cast<uint32_t>(iNbU);
            rOut.uiNurbVVertexCount = static_cast<uint32_t>(iNbV);
            rOut.vNurbControlVertices.clear();
            rOut.vNurbWeights.clear();
            // Pole grid U-major (index = iu*nbV + iv): outer = arc (angle), inner = meridian.
            for (int iu = 0; iu < iNbU; ++iu)
            {
                for (int iv = 0; iv < iNbV; ++iv)
                {
                    const pxr::GfVec3d sPole = vFoot[static_cast<size_t>(iv)] +
                                               RotatePerp(vRadial[static_cast<size_t>(iv)], sAxisD, vAng[static_cast<size_t>(iu)]) *
                                                   vScale[static_cast<size_t>(iu)];
                    rOut.vNurbControlVertices.push_back(rTransform.Transform(sPole));
                    rOut.vNurbWeights.push_back(vArcW[static_cast<size_t>(iu)] * sBasis.vWeights[static_cast<size_t>(iv)]);
                }
            }

            // U (arc) knots: clamped degree 2 at angle boundaries; V knots = basis curve knots.
            rOut.vNurbUKnots.clear();
            rOut.vNurbUKnots.push_back(dA0);
            rOut.vNurbUKnots.push_back(dA0);
            rOut.vNurbUKnots.push_back(dA0);
            for (int s = 1; s < iNSeg; ++s)
            {
                const double dK = dA0 + s * dSeg;
                rOut.vNurbUKnots.push_back(dK);
                rOut.vNurbUKnots.push_back(dK);
            }
            rOut.vNurbUKnots.push_back(dA1);
            rOut.vNurbUKnots.push_back(dA1);
            rOut.vNurbUKnots.push_back(dA1);
            rOut.vNurbVKnots.assign(sBasis.vKnots.begin(), sBasis.vKnots.end());
            if (pSweptDomainOut)
            {
                *pSweptDomainOut = pxr::GfRange2d(pxr::GfVec2d(dA0, sBasis.dParamLo), pxr::GfVec2d(dA1, sBasis.dParamHi));
            }
            return true;
        }

        default:
            return false;
    }
}

pxr::GfVec3d EvalCurve3dPoint(const SCurve3d& rCurve, const pxr::GfMatrix4d& rTransform, double t)
{
    const SCurve3d* pCurve = &rCurve;
    while (pCurve && (pCurve->eType == ECurveType::Trimmed || pCurve->eType == ECurveType::Offset) && pCurve->pBasis)
    {
        pCurve = pCurve->pBasis.get();
    }
    if (!pCurve)
    {
        return rTransform.Transform(pxr::GfVec3d(0.0, 0.0, 0.0));
    }

    switch (pCurve->eType)
    {
        case ECurveType::Line:
        {
            const pxr::GfVec3d sLocal = ToVec(pCurve->sLocation) + t * ToVec(pCurve->sXAxis);
            return rTransform.Transform(sLocal);
        }
        case ECurveType::Circle:
        case ECurveType::Ellipse:
        {
            const pxr::GfVec3d sCenter = ToVec(pCurve->sLocation);
            const pxr::GfVec3d sXDir = ToVec(pCurve->sXAxis);
            const pxr::GfVec3d sYDir = pxr::GfCross(ToVec(pCurve->sAxis), sXDir);
            const double dRx = pCurve->dRadius;
            const double dRy = (pCurve->eType == ECurveType::Ellipse) ? pCurve->dMinorRadius : pCurve->dRadius;
            const pxr::GfVec3d sLocal = sCenter + (dRx * std::cos(t)) * sXDir + (dRy * std::sin(t)) * sYDir;
            return rTransform.Transform(sLocal);
        }
        case ECurveType::Parabola:
        {
            const pxr::GfVec3d sO = ToVec(pCurve->sLocation);
            const pxr::GfVec3d sX = ToVec(pCurve->sXAxis);
            const pxr::GfVec3d sY = ToVec(pCurve->sYAxis);
            const double d4F = 4.0 * pCurve->dFocal;
            const pxr::GfVec3d sLocal = (d4F != 0.0) ? (sO + (t * t / d4F) * sX + t * sY) : sO;
            return rTransform.Transform(sLocal);
        }
        case ECurveType::Hyperbola:
        {
            const pxr::GfVec3d sO = ToVec(pCurve->sLocation);
            const pxr::GfVec3d sX = ToVec(pCurve->sXAxis);
            const pxr::GfVec3d sY = ToVec(pCurve->sYAxis);
            const pxr::GfVec3d sLocal = sO + (pCurve->dRadius * std::cosh(t)) * sX + (pCurve->dMinorRadius * std::sinh(t)) * sY;
            return rTransform.Transform(sLocal);
        }
        case ECurveType::Bezier:
        case ECurveType::BSpline:
        {
            return rTransform.Transform(EvalBSplineCurveLocal(*pCurve, t));
        }
        default:
        {
            if (!pCurve->sPoles.empty())
            {
                return rTransform.Transform(ToVec(pCurve->sPoles.front()));
            }
            return rTransform.Transform(ToVec(pCurve->sLocation));
        }
    }
}

} // namespace occt
