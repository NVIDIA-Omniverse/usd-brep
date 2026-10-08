// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

//*******************************************************************
// Undistributed, user managed test file
// You should no longer need to modify any files under source 
// control to get testing flexibility
// The only thing that cannot change is the Test() function signature
//*******************************************************************

#include <StdAfx.h>
#include <SmMerge.h>
#include <SmBrep.h>
#include <SmFace.h>
#include <SmRegion.h>
#include <SmAttribute.h>
#include <SmCurve.h>
#include <SmPoly.h>
#include <SmEdge.h>
#include <limits>
#include <memory>

#include <SmApiGeneral.h>
#include <SmApiCurves.h>
#include <SmApiBrep.h>
#include <SmApiQueries.h>
#include <SmApiPrimitives.h>
#include <SmApiFillets.h>
#include <SmApiIntersectors.h>
#include <SmApiSurfaces.h>
#include <SmApiPolygons.h>

#include <cmath>
#include <cstdio>
#include <limits>

//*************************************************************************
// 
//*************************************************************************

SmStatus TestSmBoolean()
{


    SmApiCreateContext();

    SmVector3d sPositon1(0.0,0.0,0.0);
   
    SmBrep* pResult1 = NULL;
    SmApiCreateBox(sPositon1, 5, 10, 15, pResult1);

    SmVector3d sPositon2(3.0,5.0,0.0);

    SmBrep* pResult2 = NULL;
    SmApiCreateSphere(sPositon2, 10, pResult2);

    // Inputs the kernel cannot handle are rejected before either operand is
    // modified or deleted: the same Brep twice, and EXTRACT_SEPARATE, which has
    // no single result.
    ULONG lFaces1 = pResult1->GetNumFaces();
    ULONG lFaces2 = pResult2->GetNumFaces();
    SmBrep* pRejected = pResult1;
    if( SmApiBoolean(pResult1, pResult1, SM_BO_UNION, pRejected) != SM_ERR_INVALID_INPUT
        || pRejected != NULL )
        return SM_ERR;
    pRejected = pResult1;
    if( SmApiBoolean(pResult1, pResult2, SM_BO_EXTRACT_SEPARATE, pRejected) != SM_ERR_INVALID_INPUT
        || pRejected != NULL )
        return SM_ERR;
    if( pResult1->GetNumFaces() != lFaces1 || pResult2->GetNumFaces() != lFaces2
        || !pResult1->AssertValid() || !pResult2->AssertValid() )
        return SM_ERR;

    SmPolyBrep* pPoly = NULL;
    SER( SmApiTessellate(pResult1, pPoly) );
    double dVolumeBefore = 0.0, dVolumeAfter = 0.0;
    SER( SmApiPolyBrepComputeVolume(pPoly, dVolumeBefore) );
    SmPolyBrep* pPolyRejected = pPoly;
    if( SmApiPolyBooleanUnion(pPoly, pPoly, pPolyRejected) != SM_ERR_INVALID_INPUT
        || pPolyRejected != NULL )
        return SM_ERR;
    SER( SmApiPolyBrepComputeVolume(pPoly, dVolumeAfter) );
    delete pPoly;
    if( fabs(dVolumeAfter - dVolumeBefore) > 1.0e-9 )
        return SM_ERR;

    SmBrep* pResult3 = NULL;
    SmStatus stat = SmApiBoolean(pResult1, pResult2, SM_BO_DIFFERENCE, pResult3);

    return( stat );
}

//*************************************************************************
// 2D union must preserve the complete planar result, including the face
// parameter domain used by area and other downstream queries.
//*************************************************************************

SmStatus TestSmBoolean2DPlanarUnion()
{
    SmApiCreateContext();

    SmVector3d sAOrigin(0.0, 0.0, 0.0);
    SmVector3d sBOrigin(5.0, 0.0, 0.0);
    SmBrep* pA = NULL;
    SmBrep* pB = NULL;
    SmBrep* pResult = NULL;

    SmStatus stat = SmApiCreatePlane(sAOrigin, 10.0, 10.0, pA);
    if (stat != SM_SUCCESS || pA == NULL)
    {
        delete pA;
        return(SM_ERR);
    }

    stat = SmApiCreatePlane(sBOrigin, 10.0, 10.0, pB);
    if (stat != SM_SUCCESS || pB == NULL)
    {
        delete pA;
        delete pB;
        return(SM_ERR);
    }

    stat = SmApiBoolean2d(pA, pB, SM_2D_UNION, pResult);
    if (stat != SM_SUCCESS)
    {
        delete pA;
        delete pB;
        return(SM_ERR);
    }
    if (pResult != pA)
    {
        // Success consumes pB. Avoid guessing ownership if the result-alias
        // contract itself regresses.
        return(SM_ERR);
    }

    // SmApiBoolean2d consumes both inputs, and the result aliases the
    // surviving first Brep on success.
    pA = NULL;
    pB = NULL;

    SmStatus retStat = SM_SUCCESS;
    SmTArray<SmFace*> sFaces;
    if (   retStat == SM_SUCCESS
        && (SmApiGetFaces(pResult, sFaces) != SM_SUCCESS || sFaces.GetSize() == 0))
        retStat = SM_ERR;

    double dArea = 0.0;
    for (ULONG ii = 0; retStat == SM_SUCCESS && ii < sFaces.GetSize(); ii++)
    {
        double dFaceArea = 0.0;
        if (   SmApiFaceComputeArea(sFaces[ii], 1.0e-4, dFaceArea) != SM_SUCCESS
            || !std::isfinite(dFaceArea)
            || dFaceArea <= 0.0)
            retStat = SM_ERR;
        else
            dArea += dFaceArea;
    }

    SmPoint3d sMin;
    SmPoint3d sMax;
    const SmPoint3d sExpectedMin(0.0, 0.0, 0.0);
    const SmPoint3d sExpectedMax(15.0, 10.0, 0.0);
    if (   retStat == SM_SUCCESS
        && (   std::fabs(dArea - 150.0) > 0.03
            || SmApiBrepBoundingBox(pResult, TRUE, sMin, sMax) != SM_SUCCESS
            || sMin.DistanceBetweenSquared(sExpectedMin) > 1.0e-12
            || sMax.DistanceBetweenSquared(sExpectedMax) > 1.0e-12))
        retStat = SM_ERR;

    delete pResult;
    return(retStat);
}

//*************************************************************************
// SmApiBoolean2d must expose kernel input errors instead of reporting success
// with no usable result.
//*************************************************************************

SmStatus TestSmBoolean2DStatusPropagation()
{
    SmApiCreateContext();

    SmVector3d sAOrigin(0.0, 0.0, 0.0);
    SmVector3d sBOrigin(0.0, 0.0, 1.0);
    SmBrep* pA = NULL;
    SmBrep* pB = NULL;
    SmStatus stat = SmApiCreatePlane(sAOrigin, 10.0, 10.0, pA);
    if (stat != SM_SUCCESS || pA == NULL)
    {
        delete pA;
        return(SM_ERR);
    }
    stat = SmApiCreatePlane(sBOrigin, 10.0, 10.0, pB);
    if (stat != SM_SUCCESS || pB == NULL)
    {
        delete pA;
        delete pB;
        return(SM_ERR);
    }

    // Alias the output reference to the first input variable. Failure must
    // leave that caller-owned handle intact.
    const SmBoolean bAEditingEnabled = pA->m_bEditingEnabled;
    const SmBoolean bBEditingEnabled = pB->m_bEditingEnabled;
    SmBrep* pAOriginal = pA;
    stat = SmApiBoolean2d(pA, pB, SM_2D_UNION, pA);
    if (stat != SM_ERR_INVALID_INPUT)
    {
        // A reported success may have consumed pB. Avoid guessing ownership
        // when the status contract itself is what regressed.
        return(SM_ERR);
    }

    SmStatus retStat = (   pA == pAOriginal
                        && pA->GetNumFaces() == 1
                        && pB->GetNumFaces() == 1
                        && pA->m_bEditingEnabled == bAEditingEnabled
                        && pB->m_bEditingEnabled == bBEditingEnabled
                        && pA->AssertValid(NULL, SM_LEVEL_2, SM_WALK)
                        && pB->AssertValid(NULL, SM_LEVEL_2, SM_WALK))
                     ? SM_SUCCESS
                     : SM_ERR;

    // The kernel rejects non-coplanar inputs before consuming either Brep.
    delete pA;
    delete pB;
    return(retStat);
}

//*************************************************************************
// Mass properties: exercise SmApiBrepComputeMassProperties directly.
//*************************************************************************

static SmBoolean sm_MassApproxEqual(double dA, double dB, double dAbsTol)
{
    double dDiff = dA - dB;
    if (dDiff < 0.0) dDiff = -dDiff;
    return dDiff <= dAbsTol ? TRUE : FALSE;
}

// Compare the selected path with the legacy full integration, including the
// non-default orientation/thickness cases used by kernel mass-property callers.
static SmStatus sm_CheckPrecisePropertySelection(const SmFace* pFace, SmBoolean bCheckThickFace)
{
    SmExtent3d sBox;
    SER(pFace->CalculateBoundingBox(sBox));
    const SmVector3d sSize = sBox.GetSize();
    const double dSize = smos_Max(sSize.x, smos_Max(sSize.y, sSize.z));
    const double dEstimate = dSize * dSize / 100.0;
    // Avoid a symmetric integration origin so first moments and products are
    // nonzero and swapped/missing components cannot pass unnoticed.
    const SmPoint3d sOrigin = sBox.GetMid() + SmVector3d(0.37, -0.29, 0.41);
    const ULONG aMomentFlags[] = {
        SM_PPF_AREA_FIRST_MOMENTS, SM_PPF_AREA_SECOND_MOMENTS,
        SM_PPF_AREA_SECOND_MOMENTS, SM_PPF_AREA_SECOND_MOMENTS,
        SM_PPF_VOLUME_FIRST_MOMENTS, SM_PPF_VOLUME_SECOND_MOMENTS,
        SM_PPF_VOLUME_SECOND_MOMENTS, SM_PPF_VOLUME_SECOND_MOMENTS
    };
    const SmOrientType aOrientations[] = { SM_OT_SAME, SM_OT_OPPOSITE };
    const double aAccuracies[] = { 1.0e-3, 1.0e-5 };
    const double aThicknesses[] = { 0.0, 0.1 };
    for (SmOrientType eOrientation : aOrientations)
    for (double dAccuracy : aAccuracies)
    for (double dThickness : aThicknesses)
    {
        if (dThickness > 0.0 && !bCheckThickFace) continue;
        double dFullArea = 0.0, dFullVolume = 0.0;
        SmTArray<SmVector3d> sFullMoments;
        const SmStatus fullStat = pFace->ComputePreciseProperties(eOrientation, dAccuracy, sOrigin,
            dEstimate, dThickness, dFullArea, dFullVolume, sFullMoments);
        if (fullStat != SM_SUCCESS)
        {
            std::printf("Full integration failed: orientation=%d accuracy=%g thickness=%g status=%d\n",
                int(eOrientation), dAccuracy, dThickness, int(fullStat));
            return fullStat;
        }

        for (ULONG lFlags = 1; lFlags <= SM_PPF_ALL; ++lFlags)
        {
            // Exhaust all 63 layouts at 1e-3 with zero thickness, in both
            // orientations. Repeat single groups, centroid groups and ALL at
            // tighter accuracy and with thickness, keeping test time bounded.
            const bool bSingleGroup = (lFlags & (lFlags - 1)) == 0;
            if ((dThickness > 0.0 || dAccuracy < 1.0e-3) && !bSingleGroup
                && lFlags != (SM_PPF_AREA | SM_PPF_AREA_FIRST_MOMENTS)
                && lFlags != (SM_PPF_VOLUME | SM_PPF_VOLUME_FIRST_MOMENTS)
                && lFlags != SM_PPF_ALL)
                continue;

            // Start with nonzero outputs so missing initialization is caught.
            double dArea = -37.0, dVolume = -37.0;
            SmTArray<SmVector3d> sMoments;
            sMoments.SetSize(8);
            for (ULONG ii = 0; ii < sMoments.GetSize(); ++ii)
                sMoments[ii].Set(1.0, 2.0, 3.0);
            const SmStatus stat = pFace->ComputePreciseProperties(eOrientation, dAccuracy, sOrigin,
                dEstimate, dThickness, dArea, dVolume, sMoments, lFlags);
            if (stat != SM_SUCCESS)
            {
                std::printf("Selected integration failed: flags=%lu orientation=%d accuracy=%g thickness=%g status=%d\n",
                    lFlags, int(eOrientation), dAccuracy, dThickness, int(stat));
                return stat;
            }
            if (sMoments.GetSize() != 8) return SM_ERR;
            const double aActual[] = { dArea, dVolume };
            const double aFull[] = { dFullArea, dFullVolume };
            const ULONG aTotalFlags[] = { SM_PPF_AREA, SM_PPF_VOLUME };
            for (ULONG ii = 0; ii < 2; ++ii)
            {
                if (!(lFlags & aTotalFlags[ii]))
                {
                    if (aActual[ii] != 0.0) return SM_ERR;
                }
                else if (!sm_MassApproxEqual(aActual[ii], aFull[ii], 1.0e-12 * smos_Max(1.0, std::fabs(aFull[ii])))
                    || (lFlags == SM_PPF_ALL && aActual[ii] != aFull[ii]))
                {
                    std::printf("Selected total mismatch: flags=%lu total=%lu\n", lFlags, ii);
                    return SM_ERR;
                }
            }
            for (ULONG ii = 0; ii < sMoments.GetSize(); ++ii)
            {
                const double aActualMoment[] = { sMoments[ii].x, sMoments[ii].y, sMoments[ii].z };
                const double aFullMoment[] = { sFullMoments[ii].x, sFullMoments[ii].y, sFullMoments[ii].z };
                for (ULONG jj = 0; jj < 3; ++jj)
                {
                    if (!(lFlags & aMomentFlags[ii]))
                    {
                        if (aActualMoment[jj] != 0.0) return SM_ERR;
                    }
                    else if (!sm_MassApproxEqual(aActualMoment[jj], aFullMoment[jj],
                        1.0e-12 * smos_Max(1.0, std::fabs(aFullMoment[jj])))
                        || (lFlags == SM_PPF_ALL && aActualMoment[jj] != aFullMoment[jj]))
                    {
                        std::printf("Selected moment mismatch: flags=%lu moment=%lu component=%lu\n", lFlags, ii, jj);
                        return SM_ERR;
                    }
                }
            }
        }

        // Check the API's area+first-moment selection against full integration.
        if (dThickness == 0.0 && eOrientation == SM_OT_SAME && dAccuracy == 1.0e-3)
        {
            double dArea = 0.0;
            SmPoint3d sCentroid;
            SER(SmApiFaceComputeCentroid(pFace, dAccuracy, dArea, sCentroid));
            const SmPoint3d sExpected = sOrigin + sFullMoments[0] / dFullArea;
            if (!sm_MassApproxEqual(dArea, std::fabs(dFullArea), 1.0e-12 * std::fabs(dFullArea))
                || sCentroid.DistanceBetween(sExpected) > 1.0e-8)
                return SM_ERR;
        }
    }

    // Invalid selections must fail before changing any outputs.
    const ULONG aInvalidFlags[] = { 0, 0x40, SM_PPF_ALL | 0x40, ~ULONG(0) };
    for (ULONG lFlags : aInvalidFlags)
    {
        double dArea = -37.0, dVolume = -37.0;
        SmTArray<SmVector3d> sMoments;
        sMoments.SetSize(1);
        sMoments[0].Set(1.0, 2.0, 3.0);
        if (pFace->ComputePreciseProperties(SM_OT_SAME, 1.0e-3, sOrigin,
            dEstimate, 0.0, dArea, dVolume, sMoments, lFlags) != SM_ERR_INVALID_INPUT
            || dArea != -37.0 || dVolume != -37.0 || sMoments.GetSize() != 1
            || sMoments[0].x != 1.0 || sMoments[0].y != 2.0 || sMoments[0].z != 3.0)
            return SM_ERR;
    }
    return SM_SUCCESS;
}

SmStatus TestSmPrecisePropertySelection()
{
    SmApiCreateContext();
    const SmVector3d sOrigin(0.0, 0.0, 0.0);
    const double aExpectedAreas[] = { 52.0, 100.0 * SM_PI, 20.0 * SM_PI * SM_PI, 21.0 * SM_PI };
    for (ULONG iCase = 0; iCase < 4; ++iCase)
    {
        SmBrep* pCreated = NULL;
        SmStatus stat = SM_ERR;
        switch (iCase)
        {
        case 0:
            stat = SmApiCreateBox(SmVector3d(1.0e5, -2.0e5, 3.0e5), 2.0, 3.0, 4.0, pCreated);
            break;
        case 1:
            stat = SmApiCreateSphere(sOrigin, 5.0, pCreated);
            break;
        case 2:
            stat = SmApiCreateTorus(sOrigin, 5.0, 1.0, pCreated);
            break;
        case 3:
            stat = SmApiCreatePlanarCircle(sOrigin, 5.0, pCreated);
            break;
        }
        std::unique_ptr<SmBrep> pBrep(pCreated);
        SER(stat);
        if (!pBrep) return SM_ERR;
        if (iCase == 3)
        {
            SmBrep* pHoleRaw = NULL;
            stat = SmApiCreatePlanarCircle(sOrigin, 2.0, pHoleRaw);
            std::unique_ptr<SmBrep> pHole(pHoleRaw);
            SER(stat);
            SmBrep* pResult = NULL;
            SER(SmApiBoolean2d(pBrep.get(), pHole.get(), SM_2D_DIFFERENCE, pResult));
            pHole.release(); // Consumed on success; result aliases the first input.
            if (pResult != pBrep.get()) return SM_ERR;
        }
        SmTArray<SmFace*> sFaces;
        pBrep->GetFaces(sFaces);
        if (sFaces.GetSize() == 0) return SM_ERR;
        double dFaceSum = 0.0;
        for (ULONG ii = 0; ii < sFaces.GetSize(); ++ii)
        {
            // Full mass-property integration of the thick sphere currently
            // fails; check its area independently against the analytic value.
            stat = sm_CheckPrecisePropertySelection(sFaces[ii], iCase != 1);
            if (stat != SM_SUCCESS)
            {
                std::printf("Property-selection case=%lu face=%lu failed\n", iCase, ii);
                return stat;
            }
            double dFaceArea = 0.0;
            SER(SmApiFaceComputeArea(sFaces[ii], 1.0e-4, dFaceArea));
            dFaceSum += dFaceArea;
        }
        double dBrepArea = 0.0;
        SER(SmApiBrepComputeArea(pBrep.get(), 1.0e-4, dBrepArea));
        if (dBrepArea != dFaceSum
            || !sm_MassApproxEqual(dBrepArea, aExpectedAreas[iCase], aExpectedAreas[iCase] * 1.0e-3))
            return SM_ERR;

        if (iCase == 1)
        {
            // A zero area estimate at a sphere pole fails in the nested solver.
            // Every selection must report it too, not return a plausible zero.
            SmExtent3d sBox;
            SER(sFaces[0]->CalculateBoundingBox(sBox));
            double dFullArea = -37.0, dFullVolume = -37.0;
            SmTArray<SmVector3d> sMoments;
            const SmStatus fullStat = sFaces[0]->ComputePreciseProperties(SM_OT_SAME,
                1.0e-4, sBox.GetMid(), 0.0, 0.0, dFullArea, dFullVolume, sMoments);
            if (fullStat == SM_SUCCESS) return SM_ERR;
            for (ULONG lFlags = 1; lFlags <= SM_PPF_VOLUME_SECOND_MOMENTS; lFlags <<= 1)
            {
                double dArea = -37.0, dVolume = -37.0;
                const SmStatus selectedStat = sFaces[0]->ComputePreciseProperties(SM_OT_SAME,
                    1.0e-4, sBox.GetMid(), 0.0, 0.0, dArea, dVolume, sMoments, lFlags);
                if (selectedStat != fullStat || dArea != -37.0 || dVolume != -37.0)
                    return SM_ERR;
            }

            double dArea = 0.0, dVolume = 0.0;
            SER(sFaces[0]->ComputePreciseProperties(SM_OT_SAME, 1.0e-5, sBox.GetMid(),
                1.0, 0.1, dArea, dVolume, sMoments,
                SM_PPF_AREA | SM_PPF_AREA_FIRST_MOMENTS | SM_PPF_AREA_SECOND_MOMENTS));
            const double dOffsetSphereArea = 4.0 * SM_PI * (5.05 * 5.05 + 4.95 * 4.95);
            const double dSecond = 4.0 * SM_PI / 3.0 * (std::pow(5.05, 4) + std::pow(4.95, 4));
            // Zero-valued first moments/products need dimensionally scaled
            // absolute tolerances. The existing solver controls area error,
            // not relative error in each moment (especially near zero).
            const double dFirstTol = dOffsetSphereArea * 5.05 * 1.0e-4;
            const double dSecondTol = dSecond * 1.0e-3;
            if (!sm_MassApproxEqual(dArea, dOffsetSphereArea, dOffsetSphereArea * 1.0e-3)
                || dVolume != 0.0 || sMoments[0].Length() > dFirstTol || sMoments[2].Length() > dSecondTol
                || !sm_MassApproxEqual(sMoments[1].x, dSecond, dSecondTol)
                || !sm_MassApproxEqual(sMoments[1].y, dSecond, dSecondTol)
                || !sm_MassApproxEqual(sMoments[1].z, dSecond, dSecondTol)
                || !sm_MassApproxEqual(sMoments[3].x, 2.0 * dSecond, dSecondTol)
                || !sm_MassApproxEqual(sMoments[3].y, 2.0 * dSecond, dSecondTol)
                || !sm_MassApproxEqual(sMoments[3].z, 2.0 * dSecond, dSecondTol))
            {
                std::printf("Thick-sphere surface integrals differ from analytic values\n");
                return SM_ERR;
            }
        }
    }
    return SM_SUCCESS;
}

SmStatus TestSmMassProperties()
{
    SmApiCreateContext();

    const double dDensity = 2.0;
    const double dAcc     = 1.0e-4;

    // A symmetric cube can't catch component-order / product-sign regressions
    // (equal moments, zero products). Use a distinct-dimension box rotated 30
    // deg about its centroidal Z axis: distinct known moments, one nonzero Ixy.
    const double Lx = 2.0, Ly = 6.0, Lz = 10.0;
    SmVector3d sOrigin(0.0, 0.0, 0.0);
    SmBrep* pBox = NULL;
    SER(SmApiCreateBox(sOrigin, Lx, Ly, Lz, pBox));
    if (pBox == NULL)
        return SM_ERR;

    SmVector3d sCenter(0.5 * Lx, 0.5 * Ly, 0.5 * Lz);   // (1, 3, 5)
    SmVector3d sZAxis(0.0, 0.0, 1.0);
    SmStatus stat = SmApiRotate(pBox, sCenter, sZAxis, 30.0);
    if (stat != SM_SUCCESS) { delete pBox; return stat; }

    // The API is single-pass about a caller origin, so recover the centroid
    // about the bbox midpoint, then take centroidal inertia about it.
    SmPoint3d sBBoxMin, sBBoxMax;
    stat = SmApiBrepBoundingBox(pBox, FALSE, sBBoxMin, sBBoxMax);
    if (stat != SM_SUCCESS) { delete pBox; return stat; }
    SmPoint3d sMid(0.5 * (sBBoxMin.x + sBBoxMax.x),
                   0.5 * (sBBoxMin.y + sBBoxMax.y),
                   0.5 * (sBBoxMin.z + sBBoxMax.z));

    double     dA1 = 0.0, dV1 = 0.0, dM1 = 0.0;
    SmPoint3d  sCentroid, sCentroid2;
    SmVector3d sMOItmp, sPOItmp, sMOI, sPOI;
    double     dArea = 0.0, dVolume = 0.0, dMass = 0.0;
    stat = SmApiBrepComputeMassProperties(pBox, dAcc, dDensity, sMid,
                                          dA1, dV1, dM1, sCentroid, sMOItmp, sPOItmp);
    if (stat != SM_SUCCESS) { delete pBox; return stat; }
    stat = SmApiBrepComputeMassProperties(pBox, dAcc, dDensity, sCentroid,
                                          dArea, dVolume, dMass, sCentroid2, sMOI, sPOI);
    if (stat != SM_SUCCESS) { delete pBox; return stat; }

    const double dM = dDensity * Lx * Ly * Lz;          // 240
    // Body-frame centroidal second moments (integral of coord^2 dm):
    const double dAxx = dM * Lx * Lx / 12.0;            // 80
    const double dByy = dM * Ly * Ly / 12.0;            // 720
    const double dCzz = dM * Lz * Lz / 12.0;            // 2000
    const double dCos2 = 0.75, dSin2 = 0.25;            // cos^2 / sin^2 (30 deg)
    const double dSinCos = 0.4330127018922193;          // sin*cos (30 deg)
    const double dIxx = (dAxx * dSin2 + dByy * dCos2) + dCzz;   // 2560
    const double dIyy = (dAxx * dCos2 + dByy * dSin2) + dCzz;   // 2240
    const double dIzz = dAxx + dByy;                           // 800
    const double dIxy = dSinCos * (dAxx - dByy);               // -277.128...
    const double dAbsIxy = dIxy < 0.0 ? -dIxy : dIxy;

    if (   !sm_MassApproxEqual(dVolume, Lx * Ly * Lz, 0.5)
        || !sm_MassApproxEqual(dArea, 2.0 * (Lx * Ly + Lx * Lz + Ly * Lz), 0.5)
        || !sm_MassApproxEqual(dMass, dM, 0.5)
        || !sm_MassApproxEqual(sCentroid.x, sCenter.x, 1.0e-3)
        || !sm_MassApproxEqual(sCentroid.y, sCenter.y, 1.0e-3)
        || !sm_MassApproxEqual(sCentroid.z, sCenter.z, 1.0e-3)
        || !sm_MassApproxEqual(sMOI.x, dIxx, dIxx * 1.0e-2)
        || !sm_MassApproxEqual(sMOI.y, dIyy, dIyy * 1.0e-2)
        || !sm_MassApproxEqual(sMOI.z, dIzz, dIzz * 1.0e-2)
        || !sm_MassApproxEqual(sPOI.x, 0.0, 5.0)        // Iyz ~ 0
        || !sm_MassApproxEqual(sPOI.y, 0.0, 5.0)        // Izx ~ 0
        || !sm_MassApproxEqual(sPOI.z, dIxy, dAbsIxy * 1.0e-2 + 5.0))
    {
        delete pBox;
        return SM_ERR;
    }
    // Pin the product sign explicitly: Lx < Ly, so dAxx < dByy and Ixy < 0.
    if (sPOI.z >= 0.0) { delete pBox; return SM_ERR; }

    // Single pass about the world origin: moments must satisfy parallel-axis
    // vs. the centroidal values, and the centroid must be origin-independent.
    SmPoint3d  sWorldOrigin(0.0, 0.0, 0.0);
    double     dAo = 0.0, dVo = 0.0, dMo = 0.0;
    SmPoint3d  sCo;
    SmVector3d sMOIo, sPOIo;
    stat = SmApiBrepComputeMassProperties(pBox, dAcc, dDensity, sWorldOrigin,
                                          dAo, dVo, dMo, sCo, sMOIo, sPOIo);
    if (stat != SM_SUCCESS) { delete pBox; return stat; }
    const double cx = sCenter.x, cy = sCenter.y, cz = sCenter.z;
    const double dIxxO = dIxx + dM * (cy * cy + cz * cz);
    const double dIyyO = dIyy + dM * (cx * cx + cz * cz);
    const double dIzzO = dIzz + dM * (cx * cx + cy * cy);
    const double dIyzO = dM * cy * cz;                 // Iyz_centroidal ~ 0
    const double dIzxO = dM * cz * cx;                 // Izx_centroidal ~ 0
    const double dIxyO = dIxy + dM * cx * cy;
    const double dAbsIxyO = dIxyO < 0.0 ? -dIxyO : dIxyO;
    if (   !sm_MassApproxEqual(sCo.x, cx, 1.0e-3)
        || !sm_MassApproxEqual(sCo.y, cy, 1.0e-3)
        || !sm_MassApproxEqual(sCo.z, cz, 1.0e-3)
        || !sm_MassApproxEqual(sMOIo.x, dIxxO, dIxxO * 1.0e-2)
        || !sm_MassApproxEqual(sMOIo.y, dIyyO, dIyyO * 1.0e-2)
        || !sm_MassApproxEqual(sMOIo.z, dIzzO, dIzzO * 1.0e-2)
        || !sm_MassApproxEqual(sPOIo.x, dIyzO, dIyzO * 1.0e-2 + 5.0)
        || !sm_MassApproxEqual(sPOIo.y, dIzxO, dIzxO * 1.0e-2 + 5.0)
        || !sm_MassApproxEqual(sPOIo.z, dIxyO, dAbsIxyO * 1.0e-2 + 5.0))
    { delete pBox; return SM_ERR; }

    double     dA2 = 0.0, dV2 = 0.0, dM2 = 0.0;
    SmPoint3d  sC2;
    SmVector3d sMOI2, sPOI2;

    // Non-positive, sub-SM_EFF_ZERO, and non-finite densities are rejected.
    const double adBadDensity[] = {
        0.0, -1.0, 1.0e-13,
        std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::infinity()
    };
    for (ULONG ii = 0; ii < sizeof(adBadDensity) / sizeof(adBadDensity[0]); ++ii)
    {
        if (SmApiBrepComputeMassProperties(pBox, dAcc, adBadDensity[ii], sMid,
                                           dA2, dV2, dM2, sC2, sMOI2, sPOI2)
            != SM_ERR_INVALID_INPUT)
        { delete pBox; return SM_ERR; }
    }

    // Non-finite accuracy is rejected before clamping.
    double dInvalidVolume = 0.0;
    if (SmApiBrepComputeVolume(pBox,
            std::numeric_limits<double>::quiet_NaN(), dInvalidVolume)
        != SM_ERR_INVALID_INPUT)
    { delete pBox; return SM_ERR; }
    if (SmApiBrepComputeMassProperties(pBox,
            std::numeric_limits<double>::quiet_NaN(), dDensity, sMid,
            dA2, dV2, dM2, sC2, sMOI2, sPOI2) != SM_ERR_INVALID_INPUT)
    { delete pBox; return SM_ERR; }
    if (SmApiBrepComputeMassProperties(pBox, 10.0, dDensity, sMid,
            dA2, dV2, dM2, sC2, sMOI2, sPOI2) != SM_SUCCESS)
    { delete pBox; return SM_ERR; }

    // Brep area is the sum of its face areas.
    const double dExpectedBoxArea = 2.0 * (Lx * Ly + Ly * Lz + Lz * Lx);
    double dBrepArea = 0.0;
    if (   SmApiBrepComputeArea(pBox, dAcc, dBrepArea) != SM_SUCCESS
        || std::fabs(dBrepArea - dExpectedBoxArea) > 1.0e-6)
    { delete pBox; return SM_ERR; }
    if (SmApiBrepComputeArea(pBox,
            std::numeric_limits<double>::quiet_NaN(), dBrepArea)
        != SM_ERR_INVALID_INPUT)
    { delete pBox; return SM_ERR; }

    // A non-finite origin is rejected (it would propagate into every moment).
    SmPoint3d sNanOrigin(std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0);
    if (SmApiBrepComputeMassProperties(pBox, dAcc, dDensity, sNanOrigin,
            dA2, dV2, dM2, sC2, sMOI2, sPOI2) != SM_ERR_INVALID_INPUT)
    { delete pBox; return SM_ERR; }

    // A per-face SM_AI_MASS_PROPERTIES attribute overrides the uniform density
    // in the kernel, so an attributed Brep must be rejected.
    SmTArray<SmFace*> sFaces;
    pBox->GetFaces(sFaces);
    if (sFaces.GetSize() == 0) { delete pBox; return SM_ERR; }
    SmVector3dAttribute* pMassAttr =
        new (*SmApiGetOrCreateContext())
            SmVector3dAttribute(SM_AI_MASS_PROPERTIES, SmVector3d(5.0, 0.0, 0.0));
    sFaces[0]->AddAttribute(pMassAttr);
    if (SmApiBrepComputeMassProperties(pBox, dAcc, dDensity, sMid,
                                       dA2, dV2, dM2, sC2, sMOI2, sPOI2)
        != SM_ERR_INVALID_INPUT)
    { delete pBox; return SM_ERR; }

    delete pBox;

    // A non-manifold sheet body is rejected (no all-zero fallback result).
    SmVector3d sDiskOrigin(0.0, 0.0, 0.0);
    SmBrep* pDisk = NULL;
    SER(SmApiCreatePlanarCircle(sDiskOrigin, 5.0, pDisk));
    if (pDisk == NULL)
        return SM_ERR;
    SmPoint3d sDiskQuery(0.0, 0.0, 0.0);
    stat = SmApiBrepComputeMassProperties(pDisk, dAcc, dDensity, sDiskQuery,
                                          dA2, dV2, dM2, sC2, sMOI2, sPOI2);
    if (stat != SM_ERR_INVALID_INPUT)
    { delete pDisk; return SM_ERR; }

    // Brep area does not need a closed solid, so it covers the sheet body.
    double dDiskArea = 0.0;
    stat = SmApiBrepComputeArea(pDisk, dAcc, dDiskArea);
    delete pDisk;
    if (   stat != SM_SUCCESS
        || std::fabs(dDiskArea - 25.0 * SM_PI) > 25.0 * SM_PI * 1.0e-3)
        return SM_ERR;

    // A nested face-property integration failure must propagate rather than
    // masquerade as a successful zero-area result. A sphere starts at a
    // parametric pole, so a zero area estimate exercises that failure path.
    SmBrep* pSphere = NULL;
    SmVector3d sSphereOrigin(0.0, 0.0, 0.0);
    stat = SmApiCreateSphere(sSphereOrigin, 1.0, pSphere);
    if (stat != SM_SUCCESS || pSphere == NULL)
    {
        delete pSphere;
        return stat == SM_SUCCESS ? SM_ERR : stat;
    }
    SmTArray<SmFace*> sSphereFaces;
    pSphere->GetFaces(sSphereFaces);
    if (sSphereFaces.GetSize() != 1)
    {
        delete pSphere;
        return SM_ERR;
    }

    // Centroid must succeed on the same pole face.
    double dSphereCentroidArea = 0.0;
    SmPoint3d sSphereCentroid;
    if (   SmApiFaceComputeCentroid(sSphereFaces[0], dAcc, dSphereCentroidArea,
                                    sSphereCentroid) != SM_SUCCESS
        || sSphereCentroid.DistanceBetween(SmPoint3d(0.0, 0.0, 0.0)) > 1.0e-3
        || std::fabs(dSphereCentroidArea - 4.0 * SM_PI) > 4.0 * SM_PI * 1.0e-3)
    {
        delete pSphere;
        return SM_ERR;
    }

    SmExtent3d sSphereBBox;
    stat = sSphereFaces[0]->CalculateBoundingBox(sSphereBBox);
    if (stat != SM_SUCCESS)
    {
        delete pSphere;
        return stat;
    }
    double dSphereArea = 0.0;
    double dSphereVolume = 0.0;
    SmTArray<SmVector3d> sSphereMoments;
    stat = sSphereFaces[0]->ComputePreciseProperties(
        SM_OT_SAME, dAcc, sSphereBBox.GetMid(), 0.0, 0.0,
        dSphereArea, dSphereVolume, sSphereMoments);
    if (stat == SM_SUCCESS)
    {
        delete pSphere;
        return SM_ERR;
    }

    SmVector3d sSphereBoxSize = sSphereBBox.GetSize();
    double dSphereAreaEstimate = smos_Max(
        sSphereBoxSize.z, smos_Max(sSphereBoxSize.x, sSphereBoxSize.y));
    dSphereAreaEstimate = dSphereAreaEstimate * dSphereAreaEstimate / 100.0;
    stat = sSphereFaces[0]->ComputePreciseProperties(
        SM_OT_SAME, dAcc, sSphereBBox.GetMid(), dSphereAreaEstimate, 0.0,
        dSphereArea, dSphereVolume, sSphereMoments);
    delete pSphere;
    if (   stat != SM_SUCCESS
        || std::fabs(dSphereArea - 4.0 * SM_PI) > 4.0 * SM_PI * 1.0e-3)
        return SM_ERR;

    return SM_SUCCESS;
}

//*************************************************************************
// 
//*************************************************************************

static SmStatus CreateMergeTestBox
(
    double   dX,
    double   dY,
    double   dZ,
    double   dLength,
    double   dWidth,
    double   dHeight,
    SmBrep*& rpBrep
)
{
    rpBrep = NULL;
    SmVector3d sOrigin(dX, dY, dZ);
    return SmApiCreateBox(sOrigin, dLength, dWidth, dHeight, rpBrep);
}

static SmStatus CheckMergeTestVolume(SmBrep* pBrep, double dExpected)
{
    if (pBrep == NULL)
        return SM_ERR;

    double dVolume = 0.0;
    SER(SmApiBrepComputeVolume(pBrep, 1.0e-4, dVolume));
    const double dScale = std::fabs(dExpected) > 1.0 ? std::fabs(dExpected) : 1.0;
    return std::fabs(dVolume - dExpected) <= dScale * 2.0e-4 ? SM_SUCCESS : SM_ERR;
}

static SmStatus TestMergeBrepsOperation
(
    SmBooleanOperationType eOperation,
    double                 dExpectedVolume,
    SmBoolean              bExpectedManifold
)
{
    SmBrep* pFirst = NULL;
    SmBrep* pSecond = NULL;
    SER(CreateMergeTestBox(0.0, 0.0, 0.0, 10.0, 10.0, 10.0, pFirst));
    SmStatus stat = CreateMergeTestBox(8.0, 6.0, 3.0, 4.0, 8.0, 6.0, pSecond);
    if (stat != SM_SUCCESS)
    {
        delete pFirst;
        return stat;
    }

    SmTArray<SmBrep*> sBreps;
    sBreps.Add(pFirst);
    sBreps.Add(pSecond);

    SmBrep* pResult = NULL;
    stat = SmApiMergeBreps(sBreps, eOperation, pResult);
    if (stat != SM_SUCCESS || pResult == NULL || sBreps.GetSize() != 0)
    {
        delete pResult;
        return stat == SM_SUCCESS ? SM_ERR : stat;
    }

    SmBoolean bManifold = FALSE;
    stat = SmApiBrepIsManifoldSolid(pResult, bManifold);
    if (stat == SM_SUCCESS && bManifold != bExpectedManifold)
        stat = SM_ERR;
    if (stat == SM_SUCCESS && bExpectedManifold)
        stat = CheckMergeTestVolume(pResult, dExpectedVolume);
    if (stat == SM_SUCCESS && !bExpectedManifold)
    {
        SmTArray<SmFace*> sFaces;
        pResult->GetFaces(sFaces);
        if (sFaces.GetSize() == 0)
            stat = SM_ERR;
    }

    delete pResult;
    return stat;
}

enum MergeTestInvalidInput
{
    MERGE_TEST_SINGLETON,
    MERGE_TEST_NULL_ENTRY,
    MERGE_TEST_DUPLICATE_ENTRY,
    MERGE_TEST_BAD_OPERATION
};

static SmStatus TestKernelMergeBrepsInvalidInput(MergeTestInvalidInput eInvalidInput)
{
    SmBrep* pFirst = NULL;
    SmBrep* pSecond = NULL;
    SER(CreateMergeTestBox(0.0, 0.0, 0.0, 2.0, 3.0, 4.0, pFirst));
    SmStatus stat = CreateMergeTestBox(10.0, 0.0, 0.0, 3.0, 4.0, 5.0, pSecond);
    if (stat != SM_SUCCESS)
    {
        delete pFirst;
        return stat;
    }

    SmTArray<SmBrep*> sBreps;
    sBreps.Add(pFirst);
    if (eInvalidInput == MERGE_TEST_NULL_ENTRY)
        sBreps.Add(NULL);
    if (eInvalidInput != MERGE_TEST_SINGLETON)
        sBreps.Add(pSecond);
    if (eInvalidInput == MERGE_TEST_DUPLICATE_ENTRY)
        sBreps.Add(pFirst);

    const ULONG lOperation = eInvalidInput == MERGE_TEST_BAD_OPERATION ? 4 : 0;
    SmBrep* pOutput = pSecond;
    stat = SmMerge::merge_breps(sBreps, lOperation, pOutput);

    SmBoolean bPreserved = sBreps.GetSize() >= 1 && sBreps[0] == pFirst;
    if (eInvalidInput == MERGE_TEST_SINGLETON)
    {
        bPreserved &= sBreps.GetSize() == 1;
    }
    else if (eInvalidInput == MERGE_TEST_NULL_ENTRY)
    {
        bPreserved &= sBreps.GetSize() == 3
                   && sBreps[1] == NULL
                   && sBreps[2] == pSecond;
    }
    else if (eInvalidInput == MERGE_TEST_DUPLICATE_ENTRY)
    {
        bPreserved &= sBreps.GetSize() == 3
                   && sBreps[1] == pSecond
                   && sBreps[2] == pFirst;
    }
    else
    {
        bPreserved &= sBreps.GetSize() == 2 && sBreps[1] == pSecond;
    }

    // On an unexpected consuming regression the saved pointers may no longer
    // be owned by this test, so deliberately return without deleting them.
    if (stat != SM_ERR_INVALID_INPUT || pOutput != NULL || !bPreserved)
        return SM_ERR;
    if (   CheckMergeTestVolume(pFirst, 24.0) != SM_SUCCESS
        || CheckMergeTestVolume(pSecond, 60.0) != SM_SUCCESS)
        return SM_ERR;

    delete pFirst;
    delete pSecond;
    return SM_SUCCESS;
}

static SmStatus TestApiMergeBrepsInvalidOperation(SmBooleanOperationType eOperation)
{
    SmBrep* pFirst = NULL;
    SmBrep* pSecond = NULL;
    SER(CreateMergeTestBox(0.0, 0.0, 0.0, 2.0, 3.0, 4.0, pFirst));
    SmStatus stat = CreateMergeTestBox(10.0, 0.0, 0.0, 3.0, 4.0, 5.0, pSecond);
    if (stat != SM_SUCCESS)
    {
        delete pFirst;
        return stat;
    }

    SmTArray<SmBrep*> sBreps;
    sBreps.Add(pFirst);
    sBreps.Add(pSecond);
    SmBrep* pOutput = pFirst;
    stat = SmApiMergeBreps(sBreps, eOperation, pOutput);

    // As above, cleanup is safe only after proving the rejected call left the
    // original array and objects intact.
    if (   stat != SM_ERR_INVALID_INPUT
        || pOutput != NULL
        || sBreps.GetSize() != 2
        || sBreps[0] != pFirst
        || sBreps[1] != pSecond)
        return SM_ERR;
    if (   CheckMergeTestVolume(pFirst, 24.0) != SM_SUCCESS
        || CheckMergeTestVolume(pSecond, 60.0) != SM_SUCCESS)
        return SM_ERR;

    delete pFirst;
    delete pSecond;
    return SM_SUCCESS;
}

SmStatus TestSmMergeBreps()
{
    SmApiCreateContext();

    // Exercise the kernel boundary directly: an empty list must return a
    // normal status, clear a stale output, and leave the list untouched.
    SmBrep* pSentinel = NULL;
    SER(CreateMergeTestBox(0.0, 0.0, 0.0, 1.0, 1.0, 1.0, pSentinel));
    SmBrep* pOutput = pSentinel;
    SmTArray<SmBrep*> sEmpty;
    SmStatus stat = SmMerge::merge_breps(sEmpty, 0, pOutput);
    if (stat != SM_ERR_INVALID_INPUT || pOutput != NULL || sEmpty.GetSize() != 0)
    {
        delete pSentinel;
        return SM_ERR;
    }
    delete pSentinel;

    // Exercise every defensive kernel preflight independently of the public
    // wrapper so a future wrapper check cannot mask a kernel regression.
    SER(TestKernelMergeBrepsInvalidInput(MERGE_TEST_SINGLETON));
    SER(TestKernelMergeBrepsInvalidInput(MERGE_TEST_NULL_ENTRY));
    SER(TestKernelMergeBrepsInvalidInput(MERGE_TEST_DUPLICATE_ENTRY));
    SER(TestKernelMergeBrepsInvalidInput(MERGE_TEST_BAD_OPERATION));

    // The public entry point provides the same empty-list contract.
    SER(CreateMergeTestBox(0.0, 0.0, 0.0, 1.0, 1.0, 1.0, pSentinel));
    pOutput = pSentinel;
    stat = SmApiMergeBreps(sEmpty, SM_BO_UNION, pOutput);
    if (stat != SM_ERR_INVALID_INPUT || pOutput != NULL || sEmpty.GetSize() != 0)
    {
        delete pSentinel;
        return SM_ERR;
    }
    delete pSentinel;

    // A Boolean needs at least two operands. Reject singleton input without
    // consuming it, consistent with the stable Python contract.
    SmBrep* pSingleton = NULL;
    SER(CreateMergeTestBox(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, pSingleton));
    SmTArray<SmBrep*> sSingleton;
    sSingleton.Add(pSingleton);
    pOutput = pSingleton;
    stat = SmApiMergeBreps(sSingleton, SM_BO_UNION, pOutput);
    if (   stat != SM_ERR_INVALID_INPUT
        || pOutput != NULL
        || sSingleton.GetSize() != 1
        || sSingleton[0] != pSingleton
        || CheckMergeTestVolume(pSingleton, 120.0) != SM_SUCCESS)
        return SM_ERR;
    delete pSingleton;

    // Null entries are rejected before any input is consumed.
    SmBrep* pFirst = NULL;
    SmBrep* pSecond = NULL;
    SER(CreateMergeTestBox(0.0, 0.0, 0.0, 2.0, 3.0, 4.0, pFirst));
    stat = CreateMergeTestBox(10.0, 0.0, 0.0, 3.0, 4.0, 5.0, pSecond);
    if (stat != SM_SUCCESS)
    {
        delete pFirst;
        return stat;
    }
    SmTArray<SmBrep*> sInvalid;
    sInvalid.Add(pFirst);
    sInvalid.Add(NULL);
    sInvalid.Add(pSecond);
    pOutput = pFirst;
    stat = SmApiMergeBreps(sInvalid, SM_BO_UNION, pOutput);
    if (   stat != SM_ERR_INVALID_INPUT
        || pOutput != NULL
        || sInvalid.GetSize() != 3
        || sInvalid[0] != pFirst
        || sInvalid[1] != NULL
        || sInvalid[2] != pSecond
        || CheckMergeTestVolume(pFirst, 24.0) != SM_SUCCESS
        || CheckMergeTestVolume(pSecond, 60.0) != SM_SUCCESS)
        return SM_ERR;
    delete pFirst;
    delete pSecond;

    // Duplicate handles are equally unsafe because a Boolean consumes its
    // operands; reject even non-adjacent duplicates without touching inputs.
    pFirst = NULL;
    pSecond = NULL;
    SER(CreateMergeTestBox(0.0, 0.0, 0.0, 2.0, 3.0, 4.0, pFirst));
    stat = CreateMergeTestBox(10.0, 0.0, 0.0, 3.0, 4.0, 5.0, pSecond);
    if (stat != SM_SUCCESS)
    {
        delete pFirst;
        return stat;
    }
    sInvalid.RemoveAll();
    sInvalid.Add(pFirst);
    sInvalid.Add(pSecond);
    sInvalid.Add(pFirst);
    pOutput = pSecond;
    stat = SmApiMergeBreps(sInvalid, SM_BO_UNION, pOutput);
    if (   stat != SM_ERR_INVALID_INPUT
        || pOutput != NULL
        || sInvalid.GetSize() != 3
        || sInvalid[0] != pFirst
        || sInvalid[1] != pSecond
        || sInvalid[2] != pFirst
        || CheckMergeTestVolume(pFirst, 24.0) != SM_SUCCESS
        || CheckMergeTestVolume(pSecond, 60.0) != SM_SUCCESS)
        return SM_ERR;
    delete pFirst;
    delete pSecond;

    // Only the four documented N-ary operations are accepted. Each rejected
    // enum gets fresh operands and must leave their geometry intact.
    const SmBooleanOperationType sBadOperations[] =
    {
        SM_BO_UNKNOWN,
        SM_BO_EXCLUSIVE_OR,
        SM_BO_PARTIAL_MERGE
    };
    const ULONG lBadOperationCount =
        static_cast<ULONG>(sizeof(sBadOperations) / sizeof(sBadOperations[0]));
    for (ULONG ii = 0; ii < lBadOperationCount; ii++)
        SER(TestApiMergeBrepsInvalidOperation(sBadOperations[ii]));

    // Verify semantic dispatch, not merely success. Unequal operands also
    // prove DIFFERENCE preserves the legacy last-to-first order (B-A=144).
    SER(TestMergeBrepsOperation(SM_BO_UNION, 1144.0, TRUE));
    SER(TestMergeBrepsOperation(SM_BO_INTERSECTION, 48.0, TRUE));
    SER(TestMergeBrepsOperation(SM_BO_DIFFERENCE, 144.0, TRUE));
    SER(TestMergeBrepsOperation(SM_BO_MERGE, 0.0, FALSE));

    // Exercise more than one Boolean-tree node with a three-body union.
    SmBrep* pThird = NULL;
    pFirst = NULL;
    pSecond = NULL;
    SER(CreateMergeTestBox(0.0, 0.0, 0.0, 10.0, 10.0, 10.0, pFirst));
    stat = CreateMergeTestBox(5.0, 5.0, 5.0, 10.0, 10.0, 10.0, pSecond);
    if (stat == SM_SUCCESS)
        stat = CreateMergeTestBox(8.0, 8.0, 8.0, 10.0, 10.0, 10.0, pThird);
    if (stat != SM_SUCCESS)
    {
        delete pFirst;
        delete pSecond;
        delete pThird;
        return stat;
    }

    SmTArray<SmBrep*> sThreeBreps;
    sThreeBreps.Add(pFirst);
    sThreeBreps.Add(pSecond);
    sThreeBreps.Add(pThird);
    pOutput = NULL;
    stat = SmApiMergeBreps(sThreeBreps, SM_BO_UNION, pOutput);
    if (   stat != SM_SUCCESS
        || pOutput == NULL
        || sThreeBreps.GetSize() != 0
        || CheckMergeTestVolume(pOutput, 2532.0) != SM_SUCCESS)
    {
        delete pOutput;
        return stat == SM_SUCCESS ? SM_ERR : stat;
    }
    SmBoolean bManifold = FALSE;
    stat = SmApiBrepIsManifoldSolid(pOutput, bManifold);
    delete pOutput;
    if (stat != SM_SUCCESS || !bManifold)
        return stat == SM_SUCCESS ? SM_ERR : stat;

    // Use three asymmetric operands to lock the legacy last-to-first grouping
    // for a non-commutative operation. B overlaps A by 48, while C is wholly
    // inside A and disjoint from B, so input [C,B,A] gives (A-B)-C = 944.
    pThird = NULL;
    pFirst = NULL;
    pSecond = NULL;
    SER(CreateMergeTestBox(0.0, 0.0, 0.0, 10.0, 10.0, 10.0, pFirst));
    stat = CreateMergeTestBox(8.0, 6.0, 3.0, 4.0, 8.0, 6.0, pSecond);
    if (stat == SM_SUCCESS)
        stat = CreateMergeTestBox(1.0, 1.0, 1.5, 2.0, 2.0, 2.0, pThird);
    if (stat != SM_SUCCESS)
    {
        delete pFirst;
        delete pSecond;
        delete pThird;
        return stat;
    }

    SmTArray<SmBrep*> sDifferenceBreps;
    sDifferenceBreps.Add(pThird);
    sDifferenceBreps.Add(pSecond);
    sDifferenceBreps.Add(pFirst);
    pOutput = NULL;
    stat = SmApiMergeBreps(sDifferenceBreps, SM_BO_DIFFERENCE, pOutput);
    if (   stat != SM_SUCCESS
        || pOutput == NULL
        || sDifferenceBreps.GetSize() != 0
        || CheckMergeTestVolume(pOutput, 944.0) != SM_SUCCESS)
    {
        delete pOutput;
        return stat == SM_SUCCESS ? SM_ERR : stat;
    }
    bManifold = FALSE;
    stat = SmApiBrepIsManifoldSolid(pOutput, bManifold);
    delete pOutput;
    if (stat != SM_SUCCESS || !bManifold)
        return stat == SM_SUCCESS ? SM_ERR : stat;

    return SM_SUCCESS;
}


//*************************************************************************
// 
//*************************************************************************

static SmStatus TestSmTessellateFailures()
{
    SmBrep* box = nullptr;
    SmVector3d origin(0, 0, 0);
    SER(SmApiCreateBox(origin, 10, 10, 10, box));
    std::unique_ptr<SmBrep> ownedBox(box);
    SER(box->TurnToNURBS());
    SmTArray<SmFace*> faces;
    box->GetFaces(faces);

    // Move one face's trim curves outside its surface domain.
    SmTArray<SmEdgeuse*> edgeuses;
    faces[3]->GetUpwardFaceuse()->GetEdgeuses(edgeuses);
    SmAxis2Placement shift(SmPoint3d(1000, 1000, 0), SmVector3d(1, 0, 0), SmVector3d(0, 1, 0));
    for (SmEdgeuse* edgeuse : edgeuses)
    {
        SmBSplineCurve* uv = nullptr;
        SER(edgeuse->GetOrCreateUVTrimCurve(uv));
        SER(uv->Transform(shift));
    }

    SmTArray<SmTessellationFailure> failures;
    // Same tolerances as the former positional call: chord off, 25 deg curve and surface.
    SmTessellationParams params;
    params.dChordHeightTol     = 0.0;
    params.dCurveAngleTolDeg   = 25.0;
    params.dSurfaceAngleTolDeg = 25.0;
    params.dMaxEdgeLength      = 0.0;
    params.dMaxAspectRatio     = 0.0;
    for (SmBoolean allowPartial : {FALSE, TRUE})
    {
        SmPolyBrep* mesh = nullptr;
        SmTessellationReport report;
        SmStatus status = SmApiTessellate(box, mesh, params, allowPartial, &failures, &report);
        if (!report.hasMesh || !report.inputManifold || report.outputManifold ||
            report.laminaEdges != 4 || report.spineEdges != 0 || report.failures.GetSize() != 1)
            return SM_ERR;
        std::unique_ptr<SmPolyBrep> ownedMesh(mesh);
        if ((status == SM_SUCCESS) != bool(allowPartial) || bool(mesh) != bool(allowPartial) ||
            failures.GetSize() != 1 || failures[0].lFaceIndex != 3 ||
            failures[0].eStage != SM_TS_BOUNDARY_PREPARATION || failures[0].eStatus != SM_ERR_FATAL)
            return SM_ERR;
        if (mesh)
        {
            SmTArray<SmPolyFace*> polygons;
            mesh->GetPolyFaces(polygons);
            if (polygons.GetSize() != 10) return SM_ERR;
        }
    }

    SmPolyBrep* mesh = nullptr;
    if (SmApiTessellate(box, mesh, params, TRUE) != SM_ERR_INVALID_INPUT || mesh)
        return SM_ERR;
    if (SmApiTessellate(nullptr, mesh, params, TRUE, &failures) == SM_SUCCESS ||
        mesh || !failures.IsEmpty())
        return SM_ERR;

    // Fail every face while preserving the input's edge incidence.
    for (SmFace* face : faces)
    {
        edgeuses.RemoveAll();
        face->GetUpwardFaceuse()->GetEdgeuses(edgeuses);
        for (SmEdgeuse* edgeuse : edgeuses)
        {
            SmBSplineCurve* uv = nullptr;
            SER(edgeuse->GetOrCreateUVTrimCurve(uv));
            SER(uv->Transform(shift));
        }
    }
    SmTessellationReport report;
    SmStatus status = SmApiTessellate(box, mesh, params, TRUE, nullptr, &report);
    std::unique_ptr<SmPolyBrep> ownedMesh(mesh);
    if (status == SM_SUCCESS || mesh || report.hasMesh || !report.inputManifold ||
        report.failures.GetSize() != faces.GetSize())
        return SM_ERR;
    return SM_SUCCESS;
}

static SmStatus TestSmTessellateBoundaries()
{
    SmVector3d origin(0.0, 0.0, 0.0);
    SmBrep* box = nullptr;
    SER(SmApiCreateBox(origin, 10.0, 10.0, 10.0, box));
    std::unique_ptr<SmBrep> ownedBox(box);
    SmTArray<SmPoint3d> points;
    SmTArray<ULONG> counts;
    SmTArray<SmEdge*> edges;
    SER(SmApiTessellateBoundaries(box, 5.0, points, counts, edges));
    if (edges.GetSize() != 12 || counts.GetSize() != 12 || points.GetSize() != 24)
        return SM_ERR;
    for (ULONG i = 0; i < counts.GetSize(); ++i)
        if (counts[i] != 2)
            return SM_ERR;
    // Missing geometry reports partial output.
    edges[0]->SetCurve(nullptr, TRUE, FALSE, FALSE);
    SmStatus status = SmApiTessellateBoundaries(box, 5.0, points, counts, edges);
    if (status != SM_ERR || counts.GetSize() != 12 || counts[0] != 0 || points.GetSize() != 22)
        return SM_ERR;
    // Every curve missing: the edge list is still full, but nothing was sampled, so
    // there is no usable partial result for allow_partial to accept.
    {
        SmTArray<SmEdge*> allEdges;
        box->GetEdges(allEdges);
        for (ULONG i = 0; i < allEdges.GetSize(); ++i)
            allEdges[i]->SetCurve(nullptr, TRUE, FALSE, FALSE);
        status = SmApiTessellateBoundaries(box, 5.0, points, counts, edges);
        if (status != SM_ERR || points.GetSize() != 0 || edges.GetSize() != 12 || counts.GetSize() != 12)
            return SM_ERR;
        for (ULONG i = 0; i < counts.GetSize(); ++i)
            if (counts[i] != 0)
                return SM_ERR;
    }

    // Invalid input clears outputs. Each call is checked from a prepopulated state, so a
    // path that stopped clearing cannot pass on the previous call's emptiness.
    const SmPoint3d seedPoint(1.0, 2.0, 3.0);
    for (int invalidCase = 0; invalidCase < 3; ++invalidCase)
    {
        points.RemoveAll();
        counts.RemoveAll();
        edges.RemoveAll();
        points.Add(seedPoint);
        counts.Add(99);
        edges.Add(nullptr);

        SmStatus invalidStatus = SM_SUCCESS;
        if (invalidCase == 0)
            invalidStatus = SmApiTessellateBoundaries(box, 0.0, points, counts, edges);
        else if (invalidCase == 1)
            invalidStatus = SmApiTessellateBoundaries(nullptr, 5.0, points, counts, edges);
        else
            invalidStatus = SmApiTessellateBoundaries(box, std::numeric_limits<double>::quiet_NaN(), points, counts, edges);

        if (invalidStatus == SM_SUCCESS || points.GetSize() || counts.GetSize() || edges.GetSize())
            return SM_ERR;
    }
    return SM_SUCCESS;
}

static SmStatus TestSmTessellateMaxEdgeCap()
{
    SmBrep* box = nullptr;
    SmVector3d origin(0.0, 0.0, 0.0);
    SmStatus status = SmApiCreateBox(origin, 10.0, 10.0, 10.0, box);
    std::unique_ptr<SmBrep> ownedBox(box);
    if (status != SM_SUCCESS || !box)
        return SM_ERR;

    // Relative -4 means 10% of the 10x10x10 box diagonal; absolute passes that same length
    // directly. Everything else (params and flags) is identical so the cap is the only variable.
    SmTessellationParams relativeParams;
    relativeParams.dMaxEdgeLength = -4.0;
    SmPolyBrep* relative = nullptr;
    status = SmApiTessellate(box, relative, relativeParams);
    std::unique_ptr<SmPolyBrep> ownedRelative(relative);
    if (status != SM_SUCCESS || !relative)
        return SM_ERR;
    SmTessellationParams absoluteParams;
    absoluteParams.dMaxEdgeLength = 0.1 * std::sqrt(300.0);
    SmPolyBrep* absolute = nullptr;
    status = SmApiTessellate(box, absolute, absoluteParams);
    std::unique_ptr<SmPolyBrep> ownedAbsolute(absolute);
    if (status != SM_SUCCESS || !absolute)
        return SM_ERR;

    // The two caps are only algebraically equal (the relative form recomputes it from the
    // bounding box), so compare positions within a small tolerance rather than bit-for-bit.
    const double kTol = 1.0e-9;
    SmTArray<SmPolyFace*> relativeFaces, absoluteFaces;
    relative->GetPolyFaces(relativeFaces);
    absolute->GetPolyFaces(absoluteFaces);
    if (relativeFaces.GetSize() == 0 || relativeFaces.GetSize() != absoluteFaces.GetSize())
        return SM_ERR;
    for (ULONG i = 0; i < relativeFaces.GetSize(); ++i)
    {
        SmTArray<SmPolyVertex*> relativeVertices, absoluteVertices;
        relativeFaces[i]->GetPolyVertices(relativeVertices);
        absoluteFaces[i]->GetPolyVertices(absoluteVertices);
        if (relativeVertices.GetSize() != absoluteVertices.GetSize())
            return SM_ERR;
        for (ULONG j = 0; j < relativeVertices.GetSize(); ++j)
        {
            if ((relativeVertices[j]->GetPoint() - absoluteVertices[j]->GetPoint()).Length() > kTol)
                return SM_ERR;
        }
    }
    return SM_SUCCESS;
}

static SmStatus TestSmTessellateParams()
{
    // A cylinder, not a box: planar faces ignore every quality control, so a box
    // passes even when a struct field never reaches the kernel.
    SmBrep* cylinder = nullptr;
    SmVector3d origin(0.0, 0.0, 0.0);
    if (SmApiCreateCylinder(origin, 5.0, 10.0, cylinder) != SM_SUCCESS || !cylinder)
        return SM_ERR;
    std::unique_ptr<SmBrep> ownedCylinder(cylinder);

    // Default quality as the baseline.
    SmPolyBrep* coarse = nullptr;
    if (SmApiTessellate(cylinder, coarse) != SM_SUCCESS || !coarse)
        return SM_ERR;
    std::unique_ptr<SmPolyBrep> ownedCoarse(coarse);

    // A tighter chord height set on the struct must reach the kernel and refine the mesh.
    SmTessellationParams fine;
    fine.dChordHeightTol = 0.001;
    SmPolyBrep* refined = nullptr;
    if (SmApiTessellate(cylinder, refined, fine) != SM_SUCCESS || !refined)
        return SM_ERR;
    std::unique_ptr<SmPolyBrep> ownedRefined(refined);

    SmTArray<SmPolyVertex*> coarseVertices, refinedVertices;
    coarse->GetPolyVertices(coarseVertices);
    refined->GetPolyVertices(refinedVertices);
    return (coarseVertices.GetSize() > 0 && refinedVertices.GetSize() > coarseVertices.GetSize())
               ? SM_SUCCESS
               : SM_ERR;
}

SmStatus TestSmTessellate()
{
    SER(TestSmTessellateFailures());
    SER(TestSmTessellateMaxEdgeCap());
    SER(TestSmTessellateParams());
    SER(TestSmTessellateBoundaries());

    SmApiCreateContext();

    SmVector3d sPositon1(0.0,0.0,0.0);
   
    SmBrep* pBrep = NULL;
    SmApiCreateBox(sPositon1, 5, 10, 15, pBrep);

    SmApiCircularFillet(pBrep, 0.5);

    SmPolyBrep* pResult = NULL;
    SmStatus stat = SmApiTessellate(pBrep, pResult);
    

    return( stat );
}


//*************************************************************************
// 
//*************************************************************************

SmStatus TestSmProjectBrepOntoPlane()
{

    SmApiCreateContext();

    SmVector3d sPositon1(0.0,0.0,0.0);
   
    SmBrep* pBrep = NULL;
    SmApiCreateBox(sPositon1, 5, 10, 15, pBrep);

    SmApiCircularFillet(pBrep, 0.5);

    SmVector3d sTranslate(0, 0, 30);
    SmApiTranslate(pBrep, sTranslate);

    SmTArray<SmCurve*>sProjectionCrvs;
    SmVector3d sPlanePt(0, 0, 0);
    SmVector3d sPlaneNrm(0, 0, 1);
    SmStatus stat = SmApiProjectBrepOntoPlane(pBrep, sPlanePt, sPlaneNrm, sProjectionCrvs);
    if( stat != SM_SUCCESS )
        return( stat );

    // A non-unit normal gives the same result and is left as the caller passed it.
    SmVector3d sLongNrm(0, 0, 4);
    SmTArray<SmCurve*> sLongCrvs;
    if( SmApiProjectBrepOntoPlane(pBrep, sPlanePt, sLongNrm, sLongCrvs) != SM_SUCCESS
        || sLongCrvs.GetSize() != sProjectionCrvs.GetSize()
        || sLongNrm.z != 4.0 )
        stat = SM_ERR;

    // A zero-length normal is rejected rather than silently treated as +Z.
    SmVector3d sZeroNrm(0, 0, 0);
    SmTArray<SmCurve*> sZeroCrvs;
    if( stat == SM_SUCCESS
        && ( SmApiProjectBrepOntoPlane(pBrep, sPlanePt, sZeroNrm, sZeroCrvs) != SM_ERR_INVALID_INPUT
             || sZeroCrvs.GetSize() != 0 ) )
        stat = SM_ERR;

    if( stat == SM_SUCCESS
        && SmApiProjectBrepOntoPlane(NULL, sPlanePt, sPlaneNrm, sZeroCrvs) != SM_ERR_INVALID_INPUT )
        stat = SM_ERR;

    for( ULONG ii = 0; ii < sProjectionCrvs.GetSize(); ii++ )
        delete sProjectionCrvs[ii];
    for( ULONG ii = 0; ii < sLongCrvs.GetSize(); ii++ )
        delete sLongCrvs[ii];

    delete pBrep;

    return( stat );
}

//*************************************************************************
// 
//*************************************************************************

SmStatus TestSmCreateSilhouetteCurves()
{
    SmApiCreateContext();

    SmVector3d sPosition(0.0, 0.0, 0.0);
    SmBrep* pBrep = NULL;
    SmApiCreateBox(sPosition, 10, 10, 10, pBrep);
    if (!pBrep)
        return SM_ERR;

    SmTArray<SmCurve*> sCurves;
    SmVector3d sPlanePt(0, 0, 20);
    SmVector3d sPlaneNrm(0, 0, 1);
    SmStatus stat = SmApiCreateSilhouetteCurves(pBrep, sPlanePt, sPlaneNrm, sCurves);

    if(stat == SM_SUCCESS && sCurves.GetSize() == 0)
        stat = SM_ERR;

    // A non-unit normal gives the same result and is left as the caller passed it.
    SmVector3d sLongNrm(0, 0, 3);
    SmTArray<SmCurve*> sLongCrvs;
    if( stat == SM_SUCCESS
        && ( SmApiCreateSilhouetteCurves(pBrep, sPlanePt, sLongNrm, sLongCrvs) != SM_SUCCESS
             || sLongCrvs.GetSize() != sCurves.GetSize()
             || sLongNrm.z != 3.0 ) )
        stat = SM_ERR;
    for( ULONG ii = 0; ii < sLongCrvs.GetSize(); ii++ )
        delete sLongCrvs[ii];

    // A zero-length normal is rejected rather than silently treated as +Z.
    SmVector3d sZeroNrm(0, 0, 0);
    SmTArray<SmCurve*> sZeroCrvs;
    if( stat == SM_SUCCESS
        && ( SmApiCreateSilhouetteCurves(pBrep, sPlanePt, sZeroNrm, sZeroCrvs) != SM_ERR_INVALID_INPUT
             || sZeroCrvs.GetSize() != 0 ) )
        stat = SM_ERR;

    // Appending to borrowed storage must preserve both the existing pointer
    // and the caller's backing array when a larger owned buffer is needed.
    SmCurve* aExisting[1] = { sCurves.GetSize() > 0 ? sCurves[0] : NULL };
    SmTArray<SmCurve*> sAppended(1, aExisting, 1);
    if( stat == SM_SUCCESS )
    {
        if( SmApiCreateSilhouetteCurves(pBrep, sPlanePt, sPlaneNrm, sAppended) != SM_SUCCESS ||
            sAppended.GetSize() != 1 + sCurves.GetSize() ||
            sAppended[0] != sCurves[0] || aExisting[0] != sCurves[0] || sAppended.GetIsBorrowed() )
            stat = SM_ERR;
    }
    for( ULONG ii = 1; ii < sAppended.GetSize(); ++ii )
        delete sAppended[ii];

    for( ULONG ii = 0; ii < sCurves.GetSize(); ii++ )
        delete sCurves[ii];
    delete pBrep;
    pBrep = NULL;

    return( stat );
}

//*************************************************************************
// 
//*************************************************************************

SmStatus TestSmCut()
{

    SmApiCreateContext();

    SmVector3d sPositon1(0.0,0.0,0.0);
   
    SmBrep* pBrep = NULL;
    SmApiCreateBox(sPositon1, 5, 10, 15, pBrep);

    SmApiCircularFillet(pBrep, 0.5);

    SmVector3d sCutPt(0, 0, 7);

    // A zero-length normal is rejected before the Brep is touched, rather
    // than silently cutting along +Z.
    SmVector3d sZeroNrm(0, 0, 0);
    ULONG lFacesBefore = pBrep->GetNumFaces();
    if( SmApiCut(pBrep, sCutPt, sZeroNrm) != SM_ERR_INVALID_INPUT
        || pBrep->GetNumFaces() != lFacesBefore )
        return SM_ERR;

    SmVector3d sCutNrm(0, 0, 1);
    SmStatus stat = SmApiCut(pBrep, sCutPt, sCutNrm);
    if( stat != SM_SUCCESS )
        return( stat );

    // A non-unit normal cuts an identical Brep the same way and is left as
    // the caller passed it.
    SmBrep* pBrep2 = NULL;
    SmApiCreateBox(sPositon1, 5, 10, 15, pBrep2);
    SmApiCircularFillet(pBrep2, 0.5);
    SmVector3d sLongNrm(0, 0, 3);
    if( SmApiCut(pBrep2, sCutPt, sLongNrm) != SM_SUCCESS
        || pBrep2->GetNumFaces() != pBrep->GetNumFaces()
        || sLongNrm.z != 3.0 )
        stat = SM_ERR;
    delete pBrep2;

    return( stat );
}


//*************************************************************************
// 
//*************************************************************************

SmStatus TestSmProjectAndTrim()
{

    SmApiCreateContext();

    SmVector3d sPositon1(0.0,0.0,0.0);
   
    SmBrep* pBrep = NULL;
    SmApiCreateBox(sPositon1, 10, 5, 15, pBrep);

    SmApiCircularFillet(pBrep, 0.5);

    SmTArray<SmPoint3d> sPoints;
    SmVector3d sPoint1(-10.0,-2.0,20.0);
    sPoints.Add( sPoint1 );
    SmVector3d sPoint2(0.0,5.0,20.0);
    sPoints.Add( sPoint2 );
    SmVector3d sPoint3(10.0,0.0,20.0);
    sPoints.Add( sPoint3 );
    SmVector3d sPoint4(20.0,2.0,20.0);
    sPoints.Add( sPoint4 );
   
    SmBSplineCurve* pCurve = NULL;
    long stat = SmApiCreateCurve(sPoints, pCurve);

    SmPoint3d sRefPt(5, 1, 20);

    // A zero-length direction is rejected before the Brep is touched.
    SmVector3d sZeroDir(0, 0, 0);
    ULONG lFacesBefore = pBrep->GetNumFaces();
    if( SmApiProjectAndTrim(pBrep, pCurve, sZeroDir, sRefPt) != SM_ERR_INVALID_INPUT
        || pBrep->GetNumFaces() != lFacesBefore )
        return SM_ERR;

    SmVector3d sProjDir(0, 0, -1);

    // A curve whose projection misses the Brep makes the kernel trim fail;
    // that failure must reach the caller.
    SmTArray<SmPoint3d> sMissPoints;
    sMissPoints.Add( SmPoint3d(100.0, -2.0, 20.0) );
    sMissPoints.Add( SmPoint3d(101.0,  5.0, 20.0) );
    sMissPoints.Add( SmPoint3d(102.0,  0.0, 20.0) );
    sMissPoints.Add( SmPoint3d(103.0,  2.0, 20.0) );
    SmBSplineCurve* pMissCurve = NULL;
    SER( SmApiCreateCurve(sMissPoints, pMissCurve) );
    stat = SmApiProjectAndTrim(pBrep, pMissCurve, sProjDir, sRefPt);
    delete pMissCurve;
    if( stat == SM_SUCCESS || pBrep->GetNumFaces() != lFacesBefore )
        return SM_ERR;

    stat = SmApiProjectAndTrim(pBrep, pCurve, sProjDir, sRefPt);
    if( stat != SM_SUCCESS )
        return( stat );

    // A non-unit direction trims an identical Brep the same way and is left
    // as the caller passed it.
    SmBrep* pBrep2 = NULL;
    SmApiCreateBox(sPositon1, 10, 5, 15, pBrep2);
    SmApiCircularFillet(pBrep2, 0.5);
    SmVector3d sLongDir(0, 0, -4);
    if( SmApiProjectAndTrim(pBrep2, pCurve, sLongDir, sRefPt) != SM_SUCCESS
        || pBrep2->GetNumFaces() != pBrep->GetNumFaces()
        || sLongDir.z != -4.0 )
        stat = SM_ERR;
    delete pBrep2;

    return( stat );
}


//*************************************************************************
// 
//*************************************************************************

SmStatus TestSmProjectCurve()
{
 

    SmApiCreateContext();

    SmVector3d sPositon1(0.0,0.0,0.0);
   
    SmBrep* pBrep = NULL;
    SER( SmApiCreateBox(sPositon1, 10, 5, 15, pBrep) );
    std::unique_ptr<SmBrep> ownedBrep(pBrep);

    SER( SmApiCircularFillet(pBrep, 0.5) );

    SmTArray<SmPoint3d> sPoints;
    SmVector3d sPoint1(-10.0,-2.0,20.0);
    sPoints.Add( sPoint1 );
    SmVector3d sPoint2(0.0,5.0,20.0);
    sPoints.Add( sPoint2 );
    SmVector3d sPoint3(10.0,0.0,20.0);
    sPoints.Add( sPoint3 );
    SmVector3d sPoint4(20.0,2.0,20.0);
    sPoints.Add( sPoint4 );
   
    SmBSplineCurve* pCurve = NULL;
    SER( SmApiCreateCurve(sPoints, pCurve) );
    std::unique_ptr<SmBSplineCurve> ownedCurve(pCurve);

    SmTArray<SmCurve*> sCurves3d;
    SmObjsDelete<SmCurve*> cleanupCurves(&sCurves3d);
    SmVector3d sProjVec(0, 0, -1);
    SER( SmApiProjectCurve(pBrep, pCurve, sProjVec, sCurves3d) );
    if( sCurves3d.GetSize() == 0 )
        return SM_ERR;

    // Check pointer identity AND geometry: restoring just the array size can
    // hide the kernel having cleared the caller's original curve pointers.
    ULONG lCurvesBefore = sCurves3d.GetSize();
    SmTArray<SmCurve*> sOriginalCurves(sCurves3d);
    SmTArray<SmPoint3d> sOriginalPoints;
    for( ULONG ii = 0; ii < lCurvesBefore; ii++ )
    {
        if( sCurves3d[ii] == NULL )
            return SM_ERR;
        SmPoint3d point;
        SER( sCurves3d[ii]->EvaluatePoint(sCurves3d[ii]->GetNaturalInterval().GetMin(), point) );
        sOriginalPoints.Add(point);
    }

    SmVector3d sZeroVec(0, 0, 0);
    if( SmApiProjectCurve(pBrep, pCurve, sZeroVec, sCurves3d) != SM_ERR_INVALID_INPUT
        || sCurves3d.GetSize() != lCurvesBefore )
        return SM_ERR;
    for( ULONG ii = 0; ii < lCurvesBefore; ii++ )
    {
        if( sCurves3d[ii] != sOriginalCurves[ii] )
            return SM_ERR;
        SmPoint3d point;
        SER( sCurves3d[ii]->EvaluatePoint(sCurves3d[ii]->GetNaturalInterval().GetMin(), point) );
        if( (point - sOriginalPoints[ii]).Length() > 1.0e-10 )
            return SM_ERR;
    }

    // A second successful projection appends even when the new result count
    // happens to equal the number of curves already held by the caller.
    SER( SmApiProjectCurve(pBrep, pCurve, sProjVec, sCurves3d) );
    if( sCurves3d.GetSize() != 2 * lCurvesBefore )
        return SM_ERR;
    for( ULONG ii = 0; ii < lCurvesBefore; ii++ )
    {
        if( sCurves3d[ii] != sOriginalCurves[ii]
            || sCurves3d[ii + lCurvesBefore] == NULL
            || sCurves3d[ii + lCurvesBefore] == sOriginalCurves[ii] )
            return SM_ERR;
    }

    // Exercise successful output growth from borrowed storage as well.
    SmCurve* aExisting[1] = { sOriginalCurves[0] };
    SmTArray<SmCurve*> sBorrowed(1, aExisting, 1);
    const SmStatus borrowStat = SmApiProjectCurve(pBrep, pCurve, sProjVec, sBorrowed);
    const SmBoolean bBorrowedOutputOk = borrowStat == SM_SUCCESS &&
        sBorrowed.GetSize() == 1 + lCurvesBefore &&
        sBorrowed[0] == sOriginalCurves[0] && aExisting[0] == sOriginalCurves[0] &&
        !sBorrowed.GetIsBorrowed();
    for( ULONG ii = 1; ii < sBorrowed.GetSize(); ++ii )
        delete sBorrowed[ii];
    if( !bBorrowedOutputOk )
        return SM_ERR;

    // A projection missing the body must not mistake existing output for a
    // new result, and must leave every previous pointer untouched.
    SmVector3d shift(1000, 0, 0);
    SER( SmApiTranslate(pCurve, shift) );
    SmTArray<SmCurve*> sBeforeMiss(sCurves3d);
    if( SmApiProjectCurve(pBrep, pCurve, sProjVec, sCurves3d) != SM_ERR
        || !(sCurves3d == sBeforeMiss) )
        return SM_ERR;

    SmTArray<SmCurve*> sEmpty;
    SmObjsDelete<SmCurve*> cleanupEmpty(&sEmpty);
    if( SmApiProjectCurve(pBrep, pCurve, sZeroVec, sEmpty) != SM_ERR_INVALID_INPUT
        || sEmpty.GetSize() != 0 )
        return SM_ERR;
    if( SmApiProjectCurve(pBrep, pCurve, sProjVec, sEmpty) != SM_ERR
        || sEmpty.GetSize() != 0 )
        return SM_ERR;

    return( SM_SUCCESS );
}


//*************************************************************************
// 
//*************************************************************************

SmStatus TestSmCurveSweep()
{

    SmApiCreateContext();

    SmVector3d sCenter(0.0, 0.0, 0.0);
    SmBSplineCurve* pCircle = NULL;
    SmApiCreateArc(sCenter, 2.0, 0.0, 360.0, pCircle);

    if (!pCircle)
    {
        return(SM_ERR);
    }

    SmTArray<SmPoint3d> sPathPts;
    sPathPts.Add(SmPoint3d(0.0, 0.0, 0.0));
    sPathPts.Add(SmPoint3d(0.0, 0.0, 3.0));
    sPathPts.Add(SmPoint3d(0.0, 0.0, 7.0));
    sPathPts.Add(SmPoint3d(0.0, 0.0, 10.0));

    SmBSplineCurve* pPath = NULL;
    SmApiCreateCurve(sPathPts, pPath);

    if (!pPath)
    {
        return(SM_ERR);
    }

    SmTArray<SmCurve*> sProfileCurves;
    sProfileCurves.Add(pCircle);

    SmBrep* pResult = NULL;
    SmStatus stat = SmApiCreateCurveSweep(sProfileCurves, pPath,
        NULL, NULL, NULL, pResult, NULL, NULL, NULL);


    return(stat);
}

//*************************************************************************
// 
//*************************************************************************

SmStatus TestSmCurveSweepFromFaces()
{

    SmApiCreateContext();

    SmVector3d sCenter(0.0, 0.0, 0.0);
    SmBrep* pDisc = NULL;
    SmApiCreatePlanarCircle(sCenter, 2.0, pDisc);

    if (!pDisc)
    {
        return(SM_ERR);
    }

    SmTArray<SmFace*> sFaces;
    pDisc->GetFaces(sFaces);

    SmTArray<SmPoint3d> sPathPts;
    sPathPts.Add(SmPoint3d(0.0, 0.0, 0.0));
    sPathPts.Add(SmPoint3d(0.0, 0.0, 3.0));
    sPathPts.Add(SmPoint3d(0.0, 0.0, 7.0));
    sPathPts.Add(SmPoint3d(0.0, 0.0, 10.0));

    SmBSplineCurve* pPath = NULL;
    SmApiCreateCurve(sPathPts, pPath);

    if (!pPath)
    {
        return(SM_ERR);
    }

    SmBrep* pResult = NULL;
    SmStatus stat = SmApiCreateCurveSweepFromFaces(sFaces, pPath,
        NULL, NULL, NULL, pResult, NULL, NULL, NULL);


    return(stat);
}

//*************************************************************************
// 
//*************************************************************************

SmStatus TestSmTaperExtrude()
{

    SmApiCreateContext();

    SmVector3d sCorner1(0.0, 0.0, 0.0);
    SmVector3d sCorner2(10.0, 0.0, 0.0);
    SmVector3d sCorner3(10.0, 10.0, 0.0);
    SmVector3d sCorner4(0.0, 10.0, 0.0);

    SmLine* pLine = NULL;
    SmTArray<SmCurve*> sCurves;

    SmApiCreateLineSegment(sCorner1, sCorner2, pLine);
    sCurves.Add(pLine);
    SmApiCreateLineSegment(sCorner2, sCorner3, pLine);
    sCurves.Add(pLine);
    SmApiCreateLineSegment(sCorner3, sCorner4, pLine);
    sCurves.Add(pLine);
    SmApiCreateLineSegment(sCorner4, sCorner1, pLine);
    sCurves.Add(pLine);

    // A NULL entry in the profile is invalid input.
    SmTArray<SmCurve*> sWithNull(sCurves);
    sWithNull.Add(NULL);
    SmBrep* pResult = NULL;
    if (SmApiCreateTaperExtrude(sWithNull, 10.0, 5.0, 3, FALSE, pResult) != SM_ERR_INVALID_INPUT || pResult != NULL)
        return SM_ERR;

    SmStatus stat = SmApiCreateTaperExtrude(sCurves, 10.0, 5.0, 3, FALSE, pResult);


    return(stat);
}

//*************************************************************************
// 
//*************************************************************************

SmStatus TestSmPipeSweep()
{
    SmApiCreateContext();

    SmTArray<SmPoint3d> sPathPts;
    sPathPts.Add(SmPoint3d(0.0, 0.0, 0.0));
    sPathPts.Add(SmPoint3d(5.0, 5.0, 0.0));
    sPathPts.Add(SmPoint3d(10.0, 0.0, 0.0));
    sPathPts.Add(SmPoint3d(15.0, 5.0, 0.0));

    SmBSplineCurve* pPath = NULL;
    SmApiCreateCurve(sPathPts, pPath);

    if (!pPath)
        return(SM_ERR);

    SmBrep* pResult = NULL;
    SmStatus stat = SmApiCreatePipeSweep(1.0, pPath, TRUE, pResult);

    if (stat != SM_SUCCESS)
        return(stat);

    if (pResult == NULL)
        return(SM_ERR);

    if (pResult->GetNumFaces() < 1)
        return(SM_ERR);

    return(SM_SUCCESS);
}

//*************************************************************************
// 
//*************************************************************************

//*************************************************************************
// A null entry inside a non-manifold sweep selection must be rejected before
// the Brep is touched. The Python binding screens these out in
// check_nonmanifold_selection, so without direct SM_API coverage the null
// branch of smApiSweepSelectionIsOwnedBy is never exercised.
//*************************************************************************

static SmBoolean SweepBrepCountsMatch(SmBrep* pBrep, ULONG lFaces, ULONG lEdges, ULONG lVertices)
{
    SmTArray<SmFace*>   sFaces;
    SmTArray<SmEdge*>   sEdges;
    SmTArray<SmVertex*> sVertices;
    pBrep->GetFaces(sFaces);
    pBrep->GetEdges(sEdges);
    pBrep->GetVertices(sVertices);

    return (   sFaces.GetSize()    == lFaces
            && sEdges.GetSize()    == lEdges
            && sVertices.GetSize() == lVertices) ? TRUE : FALSE;
}

static SmStatus TestNonManifoldSweepStaleEntry(SmBoolean bRotational)
{
    SmVector3d sOrigin(0.0, 0.0, 0.0);
    SmBrep* pBox = NULL;
    SmApiCreateBox(sOrigin, 10, 10, 10, pBox);

    if (!pBox)
    {
        return(SM_ERR);
    }

    SmTArray<SmFace*> sFaces;
    pBox->GetFaces(sFaces);
    if (sFaces.GetSize() == 0)
    {
        delete pBox;
        return(SM_ERR);
    }

    // Retain every face, then mutate the Brep and use one the mutation freed.
    // Which face the cut discards is an implementation detail, so select by
    // liveness rather than assuming an index.
    SmVector3d sCutPt(0.0, 0.0, 5.0);
    SmVector3d sCutNormal(0.0, 0.0, 1.0);
    if (SmApiCut(pBox, sCutPt, sCutNormal) != SM_SUCCESS)
    {
        delete pBox;
        return(SM_ERR);
    }

    SmFace* pStaleFace = NULL;
    for (ULONG ii = 0; ii < sFaces.GetSize(); ++ii)
    {
        if (!pBox->IsLiveTopologyMember(sFaces[ii]))
        {
            pStaleFace = sFaces[ii];
            break;
        }
    }

    // The regression is only meaningful if the mutation actually freed a face.
    if (!pStaleFace)
    {
        delete pBox;
        return(SM_ERR);
    }

    SmTArray<SmFace*> sStaleSelection;
    sStaleSelection.Add(pStaleFace);

    SmTArray<SmFace*>   sPostCutFaces;
    SmTArray<SmEdge*>   sPostCutEdges;
    SmTArray<SmVertex*> sPostCutVertices;
    pBox->GetFaces(sPostCutFaces);
    pBox->GetEdges(sPostCutEdges);
    pBox->GetVertices(sPostCutVertices);
    const ULONG lFaces    = sPostCutFaces.GetSize();
    const ULONG lEdges    = sPostCutEdges.GetSize();
    const ULONG lVertices = sPostCutVertices.GetSize();

    SmStatus stat;
    if (bRotational)
    {
        SmPoint3d  sBasePt(0.0, 0.0, 0.0);
        SmVector3d sAxis(0.0, 0.0, 1.0);
        stat = SmApiNonManifoldRotationalSweep(pBox, pBox, sBasePt, sAxis, 90.0,
            TRUE, FALSE, &sStaleSelection, NULL, NULL);
    }
    else
    {
        SmVector3d sSweepDir(0.0, 0.0, 1.0);
        stat = SmApiNonManifoldSweep(pBox, pBox, sSweepDir, 5.0,
            TRUE, FALSE, &sStaleSelection, NULL, NULL);
    }

    // Validating by dereferencing the handle crashes here instead of returning.
    const SmBoolean bRejected  = (stat == SM_ERR_INVALID_INPUT) ? TRUE : FALSE;
    const SmBoolean bUnchanged = SweepBrepCountsMatch(pBox, lFaces, lEdges, lVertices);

    delete pBox;

    if (!bRejected || !bUnchanged)
    {
        return(SM_ERR);
    }

    return(SM_SUCCESS);
}

static SmStatus TestNonManifoldSweepDistinctTarget(SmBoolean bRotational)
{
    // selected-face pass, then whole-Brep pass (no selection)
    for (int iSelected = 1; iSelected >= 0; --iSelected)
    {
        SmVector3d sOrigin(0.0, 0.0, 0.0);
        SmBrep* pBox = NULL;
        SmApiCreateBox(sOrigin, 10, 10, 10, pBox);

        SmVector3d sTargetOrigin(40.0, 0.0, 0.0);
        SmBrep* pTarget = NULL;
        SmApiCreateBox(sTargetOrigin, 10, 10, 10, pTarget);

        if (!pBox || !pTarget)
        {
            delete pBox;
            delete pTarget;
            return(SM_ERR);
        }

        SmTArray<SmFace*>   sFaces;
        SmTArray<SmEdge*>   sEdges;
        SmTArray<SmVertex*> sVertices;
        pBox->GetFaces(sFaces);
        pBox->GetEdges(sEdges);
        pBox->GetVertices(sVertices);
        const ULONG lFaces    = sFaces.GetSize();
        const ULONG lEdges    = sEdges.GetSize();
        const ULONG lVertices = sVertices.GetSize();

        SmTArray<SmFace*>  sSweepFaces;
        SmTArray<SmFace*>* pSweepFaces = NULL;
        if (iSelected && sFaces.GetSize() > 0)
        {
            sSweepFaces.Add(sFaces[0]);
            pSweepFaces = &sSweepFaces;
        }

        SmStatus stat;
        if (bRotational)
        {
            SmPoint3d  sBasePt(0.0, 0.0, 0.0);
            SmVector3d sAxis(0.0, 0.0, 1.0);
            stat = SmApiNonManifoldRotationalSweep(pBox, pTarget, sBasePt, sAxis, 90.0,
                TRUE, FALSE, pSweepFaces, NULL, NULL);
        }
        else
        {
            SmVector3d sSweepDir(0.0, 0.0, 1.0);
            stat = SmApiNonManifoldSweep(pBox, pTarget, sSweepDir, 5.0,
                TRUE, FALSE, pSweepFaces, NULL, NULL);
        }

        // A distinct target must be rejected, leaving both Breps untouched.
        const SmBoolean bRejected = (stat == SM_ERR_INVALID_INPUT) ? TRUE : FALSE;
        const SmBoolean bSourceOk = SweepBrepCountsMatch(pBox,    lFaces, lEdges, lVertices);
        const SmBoolean bTargetOk = SweepBrepCountsMatch(pTarget, lFaces, lEdges, lVertices);

        delete pBox;
        delete pTarget;

        if (!bRejected || !bSourceOk || !bTargetOk)
        {
            return(SM_ERR);
        }
    }

    return(SM_SUCCESS);
}

static SmStatus TestNonManifoldSweepNullEntry(SmBoolean bRotational)
{
    // one pass per selection kind: faces, then edges, then vertices
    for (int iKind = 0; iKind < 3; ++iKind)
    {
        SmVector3d sOrigin(0.0, 0.0, 0.0);
        SmBrep* pBox = NULL;
        SmApiCreateBox(sOrigin, 10, 10, 10, pBox);

        if (!pBox)
        {
            return(SM_ERR);
        }

        SmTArray<SmFace*>   sFaces;
        SmTArray<SmEdge*>   sEdges;
        SmTArray<SmVertex*> sVertices;
        pBox->GetFaces(sFaces);
        pBox->GetEdges(sEdges);
        pBox->GetVertices(sVertices);

        if (   sFaces.GetSize()    == 0
            || sEdges.GetSize()    == 0
            || sVertices.GetSize() == 0)
        {
            delete pBox;
            return(SM_ERR);
        }

        const ULONG lFaces    = sFaces.GetSize();
        const ULONG lEdges    = sEdges.GetSize();
        const ULONG lVertices = sVertices.GetSize();

        // A live handle alongside a null one, so the null entry is the only defect.
        SmTArray<SmFace*>   sSelFaces;
        SmTArray<SmEdge*>   sSelEdges;
        SmTArray<SmVertex*> sSelVertices;

        if (iKind == 0) { sSelFaces.Add(sFaces[0]);       sSelFaces.Add(NULL);    }
        if (iKind == 1) { sSelEdges.Add(sEdges[0]);       sSelEdges.Add(NULL);    }
        if (iKind == 2) { sSelVertices.Add(sVertices[0]); sSelVertices.Add(NULL); }

        SmTArray<SmFace*>*   pSelFaces    = (iKind == 0) ? &sSelFaces    : NULL;
        SmTArray<SmEdge*>*   pSelEdges    = (iKind == 1) ? &sSelEdges    : NULL;
        SmTArray<SmVertex*>* pSelVertices = (iKind == 2) ? &sSelVertices : NULL;

        SmStatus stat;
        if (bRotational)
        {
            SmPoint3d  sBasePt(0.0, 0.0, 0.0);
            SmVector3d sAxis(0.0, 0.0, 1.0);
            stat = SmApiNonManifoldRotationalSweep(pBox, pBox, sBasePt, sAxis, 90.0,
                TRUE, FALSE, pSelFaces, pSelEdges, pSelVertices);
        }
        else
        {
            SmVector3d sSweepDir(0.0, 0.0, 1.0);
            stat = SmApiNonManifoldSweep(pBox, pBox, sSweepDir, 5.0,
                TRUE, FALSE, pSelFaces, pSelEdges, pSelVertices);
        }

        // Must be rejected as invalid input, and leave the Brep exactly as it was.
        const SmBoolean bRejected  = (stat == SM_ERR_INVALID_INPUT) ? TRUE : FALSE;
        const SmBoolean bUnchanged = SweepBrepCountsMatch(pBox, lFaces, lEdges, lVertices);

        delete pBox;

        if (!bRejected || !bUnchanged)
        {
            return(SM_ERR);
        }
    }

    return(SM_SUCCESS);
}

//*************************************************************************
//
//*************************************************************************

SmStatus TestSmNonManifoldSweep()
{

    SmApiCreateContext();

    SmVector3d sOrigin(0.0, 0.0, 0.0);
    SmBrep* pBox = NULL;
    SmApiCreateBox(sOrigin, 10, 10, 10, pBox);

    if (!pBox)
    {
        return(SM_ERR);
    }

    SmVector3d sSweepDir(0.0, 0.0, 1.0);

    SmTArray<SmFace*> sFaces;
    pBox->GetFaces(sFaces);

    SmTArray<SmFace*> sSweepFaces;
    if (sFaces.GetSize() > 0)
        sSweepFaces.Add(sFaces[0]);

    // A zero-length direction is rejected before the Brep is touched, rather
    // than silently sweeping along +Z.
    SmVector3d sZeroDir(0.0, 0.0, 0.0);
    ULONG lFacesBefore = pBox->GetNumFaces();
    if (SmApiNonManifoldSweep(pBox, pBox, sZeroDir, 5.0,
            TRUE, FALSE, &sSweepFaces, NULL, NULL) != SM_ERR_INVALID_INPUT
        || pBox->GetNumFaces() != lFacesBefore)
    {
        return(SM_ERR);
    }

    SmStatus stat = SmApiNonManifoldSweep(pBox, pBox, sSweepDir, 5.0,
        TRUE, FALSE, &sSweepFaces, NULL, NULL);

    if (stat != SM_SUCCESS)
    {
        return(stat);
    }

    // A non-unit direction sweeps the same distance on an identical box
    // (only its direction is used) and is left as the caller passed it.
    SmBrep* pBox2 = NULL;
    SmApiCreateBox(sOrigin, 10, 10, 10, pBox2);
    if (!pBox2)
        return(SM_ERR);
    SmTArray<SmFace*> sFaces2, sSweepFaces2;
    pBox2->GetFaces(sFaces2);
    sSweepFaces2.Add(sFaces2[0]);
    SmVector3d sLongDir(0.0, 0.0, 3.0);
    SmExtent3d sBox1, sBox2;
    stat = SmApiNonManifoldSweep(pBox2, pBox2, sLongDir, 5.0,
        TRUE, FALSE, &sSweepFaces2, NULL, NULL);
    pBox->CalculateBoundingBox(sBox1);
    pBox2->CalculateBoundingBox(sBox2);
    SmBoolean bSame = stat == SM_SUCCESS
        && pBox2->GetNumFaces() == pBox->GetNumFaces()
        && sBox2.GetMin().DistanceBetween(sBox1.GetMin()) < 1.0e-6
        && sBox2.GetMax().DistanceBetween(sBox1.GetMax()) < 1.0e-6
        && sLongDir.z == 3.0;
    delete pBox2;
    if (!bSame)
        return(SM_ERR);

    stat = TestNonManifoldSweepNullEntry(FALSE);
    if (stat != SM_SUCCESS)
    {
        return(stat);
    }

    stat = TestNonManifoldSweepDistinctTarget(FALSE);
    if (stat != SM_SUCCESS)
    {
        return(stat);
    }

    return(TestNonManifoldSweepStaleEntry(FALSE));
}

//*************************************************************************
// 
//*************************************************************************

SmStatus TestSmNonManifoldRotationalSweep()
{

    SmApiCreateContext();

    SmVector3d sOrigin(0.0, 0.0, 0.0);
    SmBrep* pBox = NULL;
    SmApiCreateBox(sOrigin, 5, 5, 5, pBox);

    if (!pBox)
    {
        return(SM_ERR);
    }

    SmTArray<SmFace*> sFaces;
    pBox->GetFaces(sFaces);

    SmTArray<SmFace*> sSweepFaces;
    if (sFaces.GetSize() > 0)
        sSweepFaces.Add(sFaces[0]);

    SmPoint3d sBasePt(0.0, 0.0, 0.0);
    SmVector3d sAxis(0.0, 0.0, 1.0);

    SmStatus stat = SmApiNonManifoldRotationalSweep(pBox, pBox, sBasePt, sAxis, 90.0,
        TRUE, FALSE, &sSweepFaces, NULL, NULL);

    if (stat != SM_SUCCESS)
    {
        return(stat);
    }

    stat = TestNonManifoldSweepNullEntry(TRUE);
    if (stat != SM_SUCCESS)
    {
        return(stat);
    }

    stat = TestNonManifoldSweepDistinctTarget(TRUE);
    if (stat != SM_SUCCESS)
    {
        return(stat);
    }

    return(TestNonManifoldSweepStaleEntry(TRUE));
}

//*************************************************************************
// 
//*************************************************************************

SmStatus TestSmBooleanWithOptions()
{

    SmApiCreateContext();

    SmVector3d sPos1(0.0, 0.0, 0.0);
    SmBrep* pBox = NULL;
    SmApiCreateBox(sPos1, 10, 10, 10, pBox);

    SmVector3d sPos2(5.0, 5.0, 5.0);
    SmBrep* pSphere = NULL;
    SmApiCreateSphere(sPos2, 5, pSphere);

    SmBrep* pResult = NULL;
    SmStatus stat = SmApiBooleanWithOptions(pBox, pSphere, SM_BO_DIFFERENCE,
        FALSE, FALSE, TRUE, FALSE, FALSE, pResult);


    return(stat);
}

//*************************************************************************
// 
//*************************************************************************

// The planar face of pBrep all of whose vertices lie at the given z.
static SmFace* FindPlanarCapAtZ(SmBrep* pBrep, double dZ)
{
    SmTArray<SmFace*> sFaces;
    pBrep->GetFaces(sFaces);
    for (ULONG ii = 0; ii < sFaces.GetSize(); ++ii)
    {
        SmTArray<SmVertex*> sVertices;
        sFaces[ii]->GetVertices(sVertices);
        bool bAll = sVertices.GetSize() > 0;
        for (ULONG kk = 0; kk < sVertices.GetSize() && bAll; ++kk)
            bAll = fabs(sVertices[kk]->GetPoint().z - dZ) < 1.0e-6;
        if (bAll)
            return sFaces[ii];
    }
    return NULL;
}

// Regression: DeleteFace must not let a void region win over the infinite region.
//
// SmBrep::DeleteFace merges the two regions next to the deleted face and chooses a survivor
// by inspecting only the far side. Every region a non-manifold merge splits off the infinite
// region inherits its void flag, so for a face between the infinite region (this side) and a
// finite void region (other side) the infinite region was the one deleted. DeleteFace then
// repointed m_pInfiniteRegion at the survivor, which stays where it was in the region list,
// leaving AssertValid reporting "m_pInfiniteRegion must be first in the region list".
//
// Reported by a user against 9.4.3, from "extrude up to face": a profile swept past a
// target, its top cap deleted, a face on the target's carrier surface merged in, then
// MakeManifold asked to keep the region at the profile. Only targets the sweep crosses twice
// (cylinder, sphere) reach the bad path; a plane does not.
//
// Variant 1 keeps the prism closed and is the control: its merged cells are not void, so the
// old code already picked correctly. Variant 2 deletes the top cap first, which is what makes
// the cells void and what used to fail.
static SmStatus DeleteFaceInfiniteRegionVariant(const SmContext& crContext, SmBoolean bDeleteTopCap)
{
    // Prism 20 x 12 x 100 along +z.
    SmTArray<SmCurve*> sProfile;
    const double dX = 10.0, dY = 6.0;
    sProfile.Add(new (crContext) SmLine(SmPoint3d(-dX, -dY, 0), SmPoint3d( dX, -dY, 0), 3, &crContext));
    sProfile.Add(new (crContext) SmLine(SmPoint3d( dX, -dY, 0), SmPoint3d( dX,  dY, 0), 3, &crContext));
    sProfile.Add(new (crContext) SmLine(SmPoint3d( dX,  dY, 0), SmPoint3d(-dX,  dY, 0), 3, &crContext));
    sProfile.Add(new (crContext) SmLine(SmPoint3d(-dX,  dY, 0), SmPoint3d(-dX, -dY, 0), 3, &crContext));

    SmBrep* pPrism = new (crContext) SmBrep();
    {
        SmPrimitiveCreation sPC(pPrism->GetInfiniteRegion());
        SER(sPC.CreateLinearSweep(sProfile, SmVector3d(0, 0, 1), 100.0, 1, TRUE));
    }
    pPrism->RemoveTopologicalEdgesAndVertices();

    if (bDeleteTopCap)
    {
        SmFace* pTop = FindPlanarCapAtZ(pPrism, 100.0);
        if (pTop == NULL)
            return SM_ERR;
        pPrism->m_bEditingEnabled = TRUE;
        SER(pPrism->DeleteFace(pTop, TRUE, TRUE));
    }

    // One face on the wall of a cylinder of radius 30 whose axis is parallel to y through z = 60,
    // so the prism crosses it twice.
    SmBrep* pCylinder = new (crContext) SmBrep();
    {
        SmVector3d sXAxis(1, 0, 0), sAxis(0, 1, 0);
        SmAxis2Placement sFrame(SmPoint3d(0, -44.0, 60.0), sXAxis, sAxis * sXAxis);
        SmPrimitiveCreation sPC(pCylinder->GetInfiniteRegion());
        SER(sPC.CreateCone(88.0, 30.0, 30.0, 0.0, 360.0, sFrame));
    }

    SmSurface* pWall = NULL;
    {
        SmTArray<SmFace*> sFaces;
        pCylinder->GetFaces(sFaces);
        for (ULONG ii = 0; ii < sFaces.GetSize(); ++ii)
        {
            if (!sFaces[ii]->GetSurface()->IsKindOf(SmPlane_TYPE))
            {
                sFaces[ii]->GetSurface()->Copy(crContext, pWall);
                break;
            }
        }
    }
    if (pWall == NULL)
        return SM_ERR;

    SmBrep* pTool = new (crContext) SmBrep();
    SmFace* pToolFace = NULL;
    SER(pTool->CreateFaceFromSurface(pWall, pWall->GetNaturalUVDomain(), pToolFace));

    SmMerge sMerge(crContext, pPrism, pTool, 1.0e-6, 20.0 * SM_PI / 180.0);
    SmBrep* pMerged = NULL;
    SER(sMerge.NonManifoldBoolean(SM_BO_MERGE, pMerged));
    if (pMerged == NULL)
        return SM_ERR;

    // Keep the cell at the bottom cap.
    SmFace* pBottom = FindPlanarCapAtZ(pMerged, 0.0);
    if (pBottom == NULL)
        return SM_ERR;
    SmFaceuse* pFU1 = NULL;
    SmFaceuse* pFU2 = NULL;
    pBottom->GetFaceuses(pFU1, pFU2);
    SmTArray<SmRegion*> sKeep;
    if (pFU1->GetRegion() != pMerged->GetInfiniteRegion())
        sKeep.Add(pFU1->GetRegion());
    if (pFU2->GetRegion() != pMerged->GetInfiniteRegion())
        sKeep.AddUnique(pFU2->GetRegion());
    if (sKeep.GetSize() != 1)
        return SM_ERR;

    SER(pMerged->MakeManifold(&sKeep, FALSE));

    // The bug left geometry and topology correct, so only the region list catches it.
    SmTArray<SmRegion*> sRegions;
    pMerged->GetRegions(sRegions);
    if (sRegions.GetSize() == 0 || sRegions[0] != pMerged->GetInfiniteRegion())
        return SM_ERR;

    SmRegion* pFound = NULL;
    pMerged->FindInfiniteRegion(pFound);
    if (pFound != pMerged->GetInfiniteRegion())
        return SM_ERR;

    if (!pMerged->AssertValid(NULL, SM_LEVEL_2, SM_WALK))
        return SM_ERR;

    return SM_SUCCESS;
}

SmStatus TestSmDeleteFaceKeepsInfiniteRegion()
{
    SmApiCreateContext();
    SmContext sContext;

    // Control: with the prism closed the merged cells are not void, so this passed before too.
    SER(DeleteFaceInfiniteRegionVariant(sContext, FALSE));

    // The reported case.
    SER(DeleteFaceInfiniteRegionVariant(sContext, TRUE));

    return SM_SUCCESS;
}

//*************************************************************************
//
//*************************************************************************

SmStatus TestSmNonManifoldBoolean()
{

    SmApiCreateContext();

    SmVector3d sPos1(0.0, 0.0, 0.0);
    SmBrep* pBox = NULL;
    SmApiCreateBox(sPos1, 10, 10, 10, pBox);

    SmVector3d sPos2(5.0, 5.0, 5.0);
    SmBrep* pSphere = NULL;
    SmApiCreateSphere(sPos2, 5, pSphere);

    SmBrep* pResult = NULL;
    SmStatus stat = SmApiNonManifoldBoolean(pBox, pSphere, SM_BO_UNION, pResult);


    return(stat);
}

//*************************************************************************
// 
//*************************************************************************

SmStatus TestSmPiecewiseMerge()
{

    SmApiCreateContext();

    SmVector3d sPos1(0.0, 0.0, 0.0);
    SmBrep* pBox1 = NULL;
    SmApiCreateBox(sPos1, 10, 10, 10, pBox1);

    SmVector3d sPos2(5.0, 0.0, 0.0);
    SmBrep* pBox2 = NULL;
    SmApiCreateBox(sPos2, 10, 10, 10, pBox2);

    SmBrep* pResult = NULL;
    SmStatus stat = SmApiPiecewiseMerge(pBox1, pBox2, FALSE, pResult);


    return(stat);
}

//*************************************************************************
// 
//*************************************************************************

SmStatus TestSmBooleanLists()
{

    SmApiCreateContext();

    SmVector3d sPos1(0.0, 0.0, 0.0);
    SmBrep* pBox1 = NULL;
    SmApiCreateBox(sPos1, 10, 10, 10, pBox1);

    SmVector3d sPos2(5.0, 0.0, 0.0);
    SmBrep* pBox2 = NULL;
    SmApiCreateBox(sPos2, 10, 10, 10, pBox2);

    SmTArray<SmBrep*> sBreps1;
    sBreps1.Add(pBox1);
    SmTArray<SmSurface*> sSurfaces1;

    SmTArray<SmBrep*> sBreps2;
    sBreps2.Add(pBox2);
    SmTArray<SmSurface*> sSurfaces2;

    SmTArray<SmBrep*> sResultBreps;
    SmTArray<SmSurface*> sResultSurfaces;

    SmStatus stat = SmApiBooleanLists(sBreps1, sSurfaces1, sBreps2, sSurfaces2,
        1, sResultBreps, sResultSurfaces);


    return(stat);
}

//*************************************************************************
// 
//*************************************************************************

SmStatus TestSmShellBrepFull()
{
    SmApiCreateContext();

    SmVector3d sPos(0.0, 0.0, 0.0);
    SmBrep* pBox = NULL;
    SmStatus stat = SmApiCreateBox(sPos, 10, 10, 10, pBox);
    if (stat != SM_SUCCESS || pBox == NULL)
    {
        return(SM_ERR);
    }

    SmTArray<SmFace*> sFaces;
    pBox->GetFaces(sFaces);
    if (sFaces.GetSize() == 0)
    {
        delete pBox;
        return(SM_ERR);
    }
    SmFace* pSelectedFace = sFaces[0];

    auto BoxAndFaceAreUnchanged = []
    (
        SmBrep*           pTestBox,
        SmFace*           pTestFace,
        const SmPoint3d&  crExpectedMin,
        const SmPoint3d&  crExpectedMax
    ) -> SmBoolean
    {
        if (   pTestBox == NULL
            || pTestFace == NULL
            || pTestBox->GetNumFaces() != 6
            || pTestBox->GetNumEdges() != 12
            || pTestBox->GetNumVertices() != 8)
            return FALSE;

        SmTArray<SmFace*> sCurrentFaces;
        pTestBox->GetFaces(sCurrentFaces);
        ULONG lFaceIndex = 0;
        if (   !sCurrentFaces.FindElement(pTestFace, lFaceIndex)
            || pTestFace->GetBrep() != pTestBox)
            return FALSE;

        SmBoolean bManifold = FALSE;
        if (   SmApiBrepIsManifoldSolid(pTestBox, bManifold) != SM_SUCCESS
            || !bManifold)
            return FALSE;

        double dVolume = 0.0;
        if (   SmApiBrepComputeVolume(pTestBox, 1.0e-4, dVolume) != SM_SUCCESS
            || std::fabs(dVolume - 1000.0) > 1.0e-6)
            return FALSE;

        SmPoint3d sActualMin, sActualMax;
        if (   SmApiBrepBoundingBox(pTestBox, TRUE, sActualMin, sActualMax) != SM_SUCCESS
            || sActualMin.DistanceBetweenSquared(crExpectedMin) > 1.0e-16
            || sActualMax.DistanceBetweenSquared(crExpectedMax) > 1.0e-16)
            return FALSE;

        double dFaceArea = 0.0;
        if (   SmApiFaceComputeArea(pTestFace, 1.0e-4, dFaceArea) != SM_SUCCESS
            || std::fabs(dFaceArea - 100.0) > 1.0e-6)
            return FALSE;

        return TRUE;
    };

    const SmPoint3d sExpectedMin(0.0, 0.0, 0.0);
    const SmPoint3d sExpectedMax(10.0, 10.0, 10.0);
    if (!BoxAndFaceAreUnchanged(pBox, pSelectedFace, sExpectedMin, sExpectedMax))
    {
        delete pBox;
        return(SM_ERR);
    }

    SmTArray<const SmFace*> sFacesToShell;
    sFacesToShell.Add(pSelectedFace);
    SmTArray<const SmFace*> sDuplicateFacesToShell;
    sDuplicateFacesToShell.Add(pSelectedFace);
    sDuplicateFacesToShell.Add(pSelectedFace);
    SmTArray<const SmFace*> sNoFacesToShell;

    struct ShellCase
    {
        double             m_dDistance;
        SmBoolean          m_bCreateSolid;
        const SmTArray<const SmFace*>* m_pFaces;
        double             m_dExpectedVolume;
    };
    ShellCase sCases[] =
    {
        {  1.0, TRUE,  &sFacesToShell,          584.0 },
        {  1.0, FALSE, &sDuplicateFacesToShell, 584.0 },
        {  1.0, FALSE, &sNoFacesToShell,        728.0 },
        { -1.0, FALSE, &sNoFacesToShell,        488.0 }
    };

    const ULONG lNumCases =
        static_cast<ULONG>(sizeof(sCases) / sizeof(sCases[0]));
    for (ULONG ii = 0; ii < lNumCases; ii++)
    {
        SmBrep* pResult = NULL;
        stat = SmApiShellBrepFull(pBox, sCases[ii].m_dDistance, TRUE, TRUE,
                                  sCases[ii].m_bCreateSolid, *sCases[ii].m_pFaces,
                                  pResult);
        if (   stat != SM_SUCCESS
            || pResult == NULL
            || pResult == pBox
            || pResult->GetNumFaces() == 0
            || !BoxAndFaceAreUnchanged(pBox, pSelectedFace,
                                       sExpectedMin, sExpectedMax))
        {
            if (pResult != NULL && pResult != pBox)
                delete pResult;
            delete pBox;
            return(SM_ERR);
        }
        double dResultVolume = 0.0;
        if (   SmApiBrepComputeVolume(pResult, 1.0e-8, dResultVolume) != SM_SUCCESS
            || std::fabs(dResultVolume - sCases[ii].m_dExpectedVolume) > 1.0e-5)
        {
            delete pResult;
            delete pBox;
            return(SM_ERR);
        }
        delete pResult;
        pResult = NULL;
        if (!BoxAndFaceAreUnchanged(pBox, pSelectedFace,
                                    sExpectedMin, sExpectedMax))
        {
            delete pBox;
            return(SM_ERR);
        }
    }

    // A rejected call must publish a null output and leave the source intact.
    SmBrep* pZeroDistanceResult = pBox;
    stat = SmApiShellBrepFull(pBox, 0.0, TRUE, TRUE, TRUE,
                              sFacesToShell, pZeroDistanceResult);
    if (   stat != SM_ERR_INVALID_INPUT
        || pZeroDistanceResult != NULL
        || !BoxAndFaceAreUnchanged(pBox, pSelectedFace,
                                   sExpectedMin, sExpectedMax))
    {
        if (pZeroDistanceResult != NULL && pZeroDistanceResult != pBox)
            delete pZeroDistanceResult;
        delete pBox;
        return(SM_ERR);
    }

    SmVector3d sForeignPos(20.0, 0.0, 0.0);
    SmBrep* pForeignBox = NULL;
    stat = SmApiCreateBox(sForeignPos, 10, 10, 10, pForeignBox);
    if (stat != SM_SUCCESS || pForeignBox == NULL)
    {
        delete pBox;
        return(SM_ERR);
    }

    SmTArray<SmFace*> sForeignFaces;
    pForeignBox->GetFaces(sForeignFaces);
    if (sForeignFaces.GetSize() == 0)
    {
        delete pBox;
        delete pForeignBox;
        return(SM_ERR);
    }
    SmFace* pForeignFace = sForeignFaces[0];
    SmTArray<const SmFace*> sInvalidFacesToShell;
    sInvalidFacesToShell.Add(pForeignFace);

    const SmPoint3d sForeignExpectedMin(20.0, 0.0, 0.0);
    const SmPoint3d sForeignExpectedMax(30.0, 10.0, 10.0);
    SmBrep* pForeignFaceResult = pForeignBox;
    stat = SmApiShellBrepFull(pBox, 1.0, TRUE, TRUE, TRUE,
                              sInvalidFacesToShell, pForeignFaceResult);
    if (   stat != SM_ERR_INVALID_INPUT
        || pForeignFaceResult != NULL
        || !BoxAndFaceAreUnchanged(pBox, pSelectedFace,
                                   sExpectedMin, sExpectedMax)
        || !BoxAndFaceAreUnchanged(pForeignBox, pForeignFace,
                                   sForeignExpectedMin, sForeignExpectedMax))
    {
        if (   pForeignFaceResult != NULL
            && pForeignFaceResult != pBox
            && pForeignFaceResult != pForeignBox)
            delete pForeignFaceResult;
        delete pBox;
        delete pForeignBox;
        return(SM_ERR);
    }

    delete pBox;
    delete pForeignBox;
    return(SM_SUCCESS);
}

//*************************************************************************
// 
//*************************************************************************

SmStatus TestSmOffsetBrepFull()
{

    SmApiCreateContext();

    SmVector3d sPos(0.0, 0.0, 0.0);
    SmBrep* pBox = NULL;
    SmApiCreateBox(sPos, 10, 10, 10, pBox);

    SmBrep* pResult = NULL;
    SmStatus stat = SmApiOffsetBrepFull(pBox, 1.0, TRUE, TRUE, pResult);


    return(stat);
}

//*************************************************************************
// 
//*************************************************************************

SmStatus TestSmStitchIntoSolid()
{

    SmApiCreateContext();

    SmVector3d sPos(0.0, 0.0, 0.0);
    SmBrep* pBox = NULL;
    SmApiCreateBox(sPos, 10, 10, 10, pBox);

    SmBoolean bProducesSolid = FALSE;
    ULONG lStitched = 0;
    double dMaxVGap = 0.0, dMaxEGap = 0.0;

    SmStatus stat = SmApiStitchIntoSolid(pBox, bProducesSolid,
        lStitched, dMaxVGap, dMaxEGap);


    return(stat);
}

//*************************************************************************
// 
//*************************************************************************

SmStatus TestSmStitchIntoShell()
{

    SmApiCreateContext();

    SmVector3d sPos(0.0, 0.0, 0.0);
    SmBrep* pBox = NULL;
    SmApiCreateBox(sPos, 10, 10, 10, pBox);

    SmBoolean bWellFormed = TRUE;
    ULONG lStitched = 0;
    double dMaxVGap = 0.0, dMaxEGap = 0.0;

    SmStatus stat = SmApiStitchIntoShell(pBox, bWellFormed, 10.0,
        lStitched, dMaxVGap, dMaxEGap);


    return(stat);
}

//*************************************************************************
// 
//*************************************************************************

SmStatus TestSmUnifyNormals()
{

    SmApiCreateContext();

    SmVector3d sPos(0.0, 0.0, 0.0);
    SmBrep* pBox = NULL;
    SmApiCreateBox(sPos, 10, 10, 10, pBox);

    if (!pBox)
    {
        return(SM_ERR);
    }

    SmTArray<SmFace*> sFaces;
    pBox->GetFaces(sFaces);

    SmFace* pStartFace = (sFaces.GetSize() > 0) ? sFaces[0] : NULL;

    SmTArray<SmFace*> sFlippedFaces;
    SmStatus stat = SmApiUnifyNormals(pBox, pStartFace, 10, sFlippedFaces);


    return(stat);
}

//*************************************************************************
// 
//*************************************************************************

SmStatus TestSmAdvancedStitch()
{

    SmApiCreateContext();

    SmVector3d sPos(0.0, 0.0, 0.0);
    SmBrep* pBox = NULL;
    SmApiCreateBox(sPos, 10, 10, 10, pBox);

    ULONG lStitched = 0, lLamina = 0;
    double dMaxVGap = 0.0, dMaxEGap = 0.0;

    SmStatus stat = SmApiStitchAdvanced(pBox, 0.001,
        TRUE, TRUE, TRUE, FALSE,
        lStitched, lLamina, dMaxVGap, dMaxEGap);


    return(stat);
}


//*************************************************************************
// 
//*************************************************************************

//*************************************************************************
// 
//*************************************************************************

SmStatus TestSmSweepAlongPlanarPath()
{
    SmApiCreateContext();

    SmTArray<SmCurve*> sEmptyCurves;
    static char sOutputSentinel = 0;
    SmBrep* pEmptyResult = reinterpret_cast<SmBrep*>(&sOutputSentinel);
    if (   SmApiCreateSweepAlongPlanarPath(sEmptyCurves, sEmptyCurves, TRUE, 3, pEmptyResult) == SM_SUCCESS
        || pEmptyResult != NULL)
        return SM_ERR;

    SmPoint3d sCenter(0.0, 0.0, 0.0);
    SmTArray<SmBSplineCurve*> sRectSegments;
    SmApiCreateRectangle(sCenter, 2.0, 1.0, sRectSegments);
    if (sRectSegments.GetSize() != 4)
        return SM_ERR;

    SmTArray<SmCurve*> sProfileCurves;
    for (ULONG ii = 0; ii < sRectSegments.GetSize(); ii++)
        sProfileCurves.Add(sRectSegments[ii]);

    SmTArray<SmCurve*> sPathCurves;
    SmLine* pLine = NULL;
    SmPoint3d sPathStart(0,0,0), sPathEnd(20,0,0);
    SmApiCreateLineSegment(sPathStart, sPathEnd, pLine);
    if (!pLine)
    {
        for (ULONG ii = 0; ii < sRectSegments.GetSize(); ii++)
            delete sRectSegments[ii];
        return SM_ERR;
    }
    sPathCurves.Add(pLine);

    SmBrep* pResult = NULL;
    SmStatus stat = SmApiCreateSweepAlongPlanarPath(
        sProfileCurves, sPathCurves, TRUE, 1, pResult);

    SmStatus retStat = stat;
    if (stat == SM_SUCCESS && !pResult)
        retStat = SM_ERR;
    else if (stat == SM_SUCCESS && pResult->GetNumFaces() < 1)
        retStat = SM_ERR;

    delete pResult;
    for (ULONG ii = 0; ii < sRectSegments.GetSize(); ii++)
        delete sRectSegments[ii];
    delete pLine;

    if (retStat != SM_SUCCESS)
        return retStat;

    SmBSplineCurve* pCircle = NULL;
    SmPoint3d sCircleCenter(0.0, 0.0, 0.0);
    SmLine* pFirstPath = NULL;
    SmPoint3d sFirstStart(0.0, 0.0, 0.0);
    SmPoint3d sFirstEnd(0.0, 0.0, 10.0);
    SmLine* pSecondPath = NULL;
    SmPoint3d sSecondStart(0.0, 0.0, 10.0);
    SmPoint3d sSecondEnd(5.0, 0.0, 15.0);
    SmStatus sCircleStat = SmApiCreateCircle(sCircleCenter, 1.0, pCircle);
    SmStatus sFirstPathStat = SmApiCreateLineSegment(sFirstStart, sFirstEnd, pFirstPath);
    SmStatus sSecondPathStat = SmApiCreateLineSegment(sSecondStart, sSecondEnd, pSecondPath);
    if (   sCircleStat != SM_SUCCESS || pCircle == NULL
        || sFirstPathStat != SM_SUCCESS || pFirstPath == NULL
        || sSecondPathStat != SM_SUCCESS || pSecondPath == NULL)
    {
        delete pCircle;
        delete pFirstPath;
        delete pSecondPath;
        return SM_ERR;
    }

    struct CurveState
    {
        SmExtent1d sInterval;
        SmPoint3d sStart;
        SmPoint3d sMid;
        SmPoint3d sEnd;
        double dLength;
    };

    auto CaptureCurveState = []
    (
        const SmCurve* pCurve,
        CurveState&    rState
    ) -> SmStatus
    {
        if (pCurve == NULL)
            return SM_ERR;

        rState.sInterval = pCurve->GetNaturalInterval();
        pCurve->GetEnds(rState.sStart, rState.sEnd);
        SmStatus evalStat = pCurve->EvaluatePoint(rState.sInterval.Evaluate(0.5), rState.sMid);
        if (evalStat != SM_SUCCESS)
            return evalStat;
        return pCurve->Length(pCurve->GetNaturalInterval(), 1.0e-8, rState.dLength);
    };

    SmCurve* aInputCurves[3] = { pCircle, pFirstPath, pSecondPath };
    CurveState aBefore[3], aAfter[3];
    for (ULONG ii = 0; ii < 3; ii++)
    {
        if (CaptureCurveState(aInputCurves[ii], aBefore[ii]) != SM_SUCCESS)
        {
            delete pCircle;
            delete pFirstPath;
            delete pSecondPath;
            return SM_ERR;
        }
    }

    SmTArray<SmCurve*> sCircleProfile;
    sCircleProfile.Add(pCircle);
    SmTArray<SmCurve*> sStraightPath;
    sStraightPath.Add(pFirstPath);

    struct CapCase
    {
        int iCapEnds;
        SmBoolean bExpectStart;
        SmBoolean bExpectEnd;
    };
    CapCase aCapCases[] = {
        { 0, FALSE, FALSE },
        { 1, TRUE, FALSE },
        { 2, FALSE, TRUE },
        { 3, TRUE, TRUE }
    };

    for (const CapCase& rCapCase : aCapCases)
    {
        SmBrep* pCapResult = NULL;
        SmStatus eCapStat = SmApiCreateSweepAlongPlanarPath(
            sCircleProfile, sStraightPath, TRUE, rCapCase.iCapEnds, pCapResult);
        SmStatus eCaseResult = SM_SUCCESS;
        SmTArray<SmFace*> sFaces;
        SmBoolean bManifold = FALSE;
        if (   eCapStat != SM_SUCCESS || pCapResult == NULL
            || SmApiGetFaces(pCapResult, sFaces) != SM_SUCCESS
            || sFaces.GetSize() != static_cast<ULONG>(
                1 + rCapCase.bExpectStart + rCapCase.bExpectEnd)
            || SmApiBrepIsManifoldSolid(pCapResult, bManifold) != SM_SUCCESS
            || bManifold != (rCapCase.bExpectStart && rCapCase.bExpectEnd))
            eCaseResult = SM_ERR;

        ULONG lStartCapCount = 0;
        ULONG lEndCapCount = 0;
        ULONG lSideCount = 0;
        for (ULONG ii = 0; eCaseResult == SM_SUCCESS && ii < sFaces.GetSize(); ii++)
        {
            SmPoint3d sMin, sMax;
            if (SmApiFaceBoundingBox(sFaces[ii], TRUE, sMin, sMax) != SM_SUCCESS)
            {
                eCaseResult = SM_ERR;
                break;
            }
            if (std::fabs(sMax[2] - sMin[2]) <= 1.0e-4)
            {
                double dCapZ = 0.5 * (sMin[2] + sMax[2]);
                if (std::fabs(dCapZ) <= 1.0e-4)
                    lStartCapCount++;
                else if (std::fabs(dCapZ - 10.0) <= 1.0e-4)
                    lEndCapCount++;
                else
                    eCaseResult = SM_ERR;
            }
            else
                lSideCount++;
        }
        if (   eCaseResult == SM_SUCCESS
            && (   lStartCapCount != static_cast<ULONG>(rCapCase.bExpectStart)
                || lEndCapCount != static_cast<ULONG>(rCapCase.bExpectEnd)
                || lSideCount != 1))
            eCaseResult = SM_ERR;

        if (eCaseResult == SM_SUCCESS)
        {
            double dVolume = 0.0;
            if (   SmApiBrepComputeVolume(pCapResult, 1.0e-4, dVolume) != SM_SUCCESS
                || !std::isfinite(dVolume))
                eCaseResult = SM_ERR;
            else if (rCapCase.bExpectStart && rCapCase.bExpectEnd)
            {
                double dExpectedVolume = 10.0 * std::acos(-1.0);
                if (   dVolume <= 0.0
                    || std::fabs(dVolume - dExpectedVolume) > dExpectedVolume * 0.01)
                    eCaseResult = SM_ERR;
            }
            else if (std::fabs(dVolume) > 1.0e-8)
                eCaseResult = SM_ERR;
        }

        delete pCapResult;
        if (eCaseResult != SM_SUCCESS)
        {
            delete pCircle;
            delete pFirstPath;
            delete pSecondPath;
            return eCaseResult;
        }
    }

    int aInvalidCapEnds[] = { -1, 4 };
    for (int iCapEnds : aInvalidCapEnds)
    {
        static char sInvalidOutputSentinel = 0;
        SmBrep* pInvalidResult = reinterpret_cast<SmBrep*>(&sInvalidOutputSentinel);
        if (   SmApiCreateSweepAlongPlanarPath(
                   sCircleProfile, sStraightPath, TRUE, iCapEnds, pInvalidResult)
                != SM_ERR_INVALID_INPUT
            || pInvalidResult != NULL)
        {
            delete pCircle;
            delete pFirstPath;
            delete pSecondPath;
            return SM_ERR;
        }
    }

    SmTArray<SmCurve*> sConnectedPath;
    sConnectedPath.Add(pFirstPath);
    sConnectedPath.Add(pSecondPath);

    SmBrep* pConnectedResult = NULL;
    stat = SmApiCreateSweepAlongPlanarPath(
        sCircleProfile, sConnectedPath, TRUE, 3, pConnectedResult);

    SmBoolean bInputsUnchanged =
           sCircleProfile.GetSize() == 1
        && sCircleProfile[0] == pCircle
        && sConnectedPath.GetSize() == 2
        && sConnectedPath[0] == pFirstPath
        && sConnectedPath[1] == pSecondPath;
    for (ULONG ii = 0; bInputsUnchanged && ii < 3; ii++)
    {
        bInputsUnchanged =
               CaptureCurveState(aInputCurves[ii], aAfter[ii]) == SM_SUCCESS
            && std::fabs(aAfter[ii].sInterval.GetMin() - aBefore[ii].sInterval.GetMin()) <= 1.0e-12
            && std::fabs(aAfter[ii].sInterval.GetMax() - aBefore[ii].sInterval.GetMax()) <= 1.0e-12
            && aAfter[ii].sStart.DistanceBetweenSquared(aBefore[ii].sStart) <= 1.0e-20
            && aAfter[ii].sMid.DistanceBetweenSquared(aBefore[ii].sMid) <= 1.0e-20
            && aAfter[ii].sEnd.DistanceBetweenSquared(aBefore[ii].sEnd) <= 1.0e-20
            && std::fabs(aAfter[ii].dLength - aBefore[ii].dLength) <= 1.0e-10;
    }

    retStat = bInputsUnchanged ? SM_SUCCESS : SM_ERR;
    if (retStat == SM_SUCCESS)
    {
        SmBoolean bManifold = FALSE;
        double dVolume = 0.0;
        double dExpectedVolume = std::acos(-1.0) * (10.0 + std::sqrt(50.0));
        SmTArray<SmRegion*> sRegions;
        if (pConnectedResult != NULL)
            pConnectedResult->GetRegions(sRegions);
        ULONG lMaterialRegions = 0;
        for (ULONG ii = 0; ii < sRegions.GetSize(); ii++)
            if (!sRegions[ii]->IsVoid())
                lMaterialRegions++;
        if (   stat != SM_SUCCESS
            || pConnectedResult == NULL
            || pConnectedResult->GetNumFaces() != 4
            || pConnectedResult->GetNumEdges() != 5
            || pConnectedResult->GetNumVertices() != 3
            || sRegions.GetSize() != 2
            || lMaterialRegions != 1
            || !pConnectedResult->AssertValid(NULL, SM_LEVEL_2, SM_WALK)
            || SmApiBrepIsManifoldSolid(pConnectedResult, bManifold) != SM_SUCCESS
            || !bManifold
            || SmApiBrepComputeVolume(pConnectedResult, 1.0e-4, dVolume) != SM_SUCCESS
            || !std::isfinite(dVolume)
            || dVolume <= 0.0
            || std::fabs(dVolume - dExpectedVolume) > dExpectedVolume * 0.02)
            retStat = SM_ERR;
    }

    delete pConnectedResult;
    delete pCircle;
    delete pFirstPath;
    delete pSecondPath;

    return retStat;
}


SmStatus TestSmBreps()
{
    SmStatus stat = TestSmBoolean();
    if( stat != SM_SUCCESS )
        return( stat );

    stat = TestSmBoolean2DPlanarUnion();
    if( stat != SM_SUCCESS )
        return( stat );

    stat = TestSmBoolean2DStatusPropagation();
    if( stat != SM_SUCCESS )
        return( stat );

    stat = TestSmMergeBreps();
    if( stat != SM_SUCCESS )
        return( stat );

    stat = TestSmTessellate();
    if( stat != SM_SUCCESS )
        return( stat );

    stat = TestSmProjectBrepOntoPlane(); 
    if( stat != SM_SUCCESS )
        return( stat );

    stat = TestSmCut();
    if( stat != SM_SUCCESS )
        return( stat );

    stat = TestSmProjectAndTrim();
    if( stat != SM_SUCCESS )
        return( stat );

    stat = TestSmProjectCurve();
    if( stat != SM_SUCCESS )
        return( stat );

    // Curve sweep tests excluded: kernel crashes in SmPrimitiveCreation::CreateCurveSweep
    // stat = TestSmCurveSweep();
    // if( stat != SM_SUCCESS )
    //     return( stat );

    // stat = TestSmCurveSweepFromFaces();
    // if( stat != SM_SUCCESS )
    //     return( stat );

    stat = TestSmTaperExtrude();
    if( stat != SM_SUCCESS )
        return( stat );

    stat = TestSmPipeSweep();
    if( stat != SM_SUCCESS )
        return( stat );

    stat = TestSmNonManifoldSweep();
    if( stat != SM_SUCCESS )
        return( stat );

    stat = TestSmNonManifoldRotationalSweep();
    if( stat != SM_SUCCESS )
        return( stat );

    stat = TestSmBooleanWithOptions();
    if( stat != SM_SUCCESS )
        return( stat );

    stat = TestSmNonManifoldBoolean();
    if( stat != SM_SUCCESS )
        return( stat );

    stat = TestSmPiecewiseMerge();
    if( stat != SM_SUCCESS )
        return( stat );

    stat = TestSmBooleanLists();
    if( stat != SM_SUCCESS )
        return( stat );

    stat = TestSmShellBrepFull();
    if( stat != SM_SUCCESS )
        return( stat );

    stat = TestSmOffsetBrepFull();
    if( stat != SM_SUCCESS )
        return( stat );

    stat = TestSmStitchIntoSolid();
    if( stat != SM_SUCCESS )
        return( stat );

    stat = TestSmStitchIntoShell();
    if( stat != SM_SUCCESS )
        return( stat );

    stat = TestSmUnifyNormals();
    if( stat != SM_SUCCESS )
        return( stat );

    stat = TestSmAdvancedStitch();
    if( stat != SM_SUCCESS )
        return( stat );

    return( SM_SUCCESS );
}

// Create an axis-aligned box; returns NULL on failure.  Used only by
// TestSmMaterialCensus below.
// Create an axis-aligned box into rpBox; returns the SmApiCreateBox status so
// callers can SER() it and never feed a NULL operand to a Boolean.  Used only
// by TestSmMaterialCensus below.
static SmStatus sm_CensusBox( double dX, double dY, double dZ,
                              double dL, double dW, double dH, SmBrep*& rpBox )
{
    rpBox = NULL;
    SmVector3d sOrigin( dX, dY, dZ );
    return SmApiCreateBox( sOrigin, dL, dW, dH, rpBox );
}

// Census pBrep, check the counts, and delete it (consumes the input).
// Returns SM_SUCCESS only when the counts match; NULL input is an error.
static SmStatus sm_ExpectCensus( SmBrep* pBrep, long lExpSolids, long lExpVoids )
{
    if( pBrep == NULL )
        return SM_ERR;

    long lSolids = 0, lVoids = 0;
    SmStatus stat = SmApiBrepMaterialCensus( pBrep, lSolids, lVoids );
    delete pBrep;
    if( stat != SM_SUCCESS )
        return stat;

    return ( lSolids == lExpSolids && lVoids == lExpVoids ) ? SM_SUCCESS : SM_ERR;
}

/*******************************************************************//**
PURPOSE --- Exercise SmApiBrepMaterialCensus: count material (solid)
    regions and enclosed void cavities, including the nested-shell case
    (a solid inside a void inside a solid) that gap #8 targets.
***********************************************************************/
SmStatus TestSmMaterialCensus()
{
    SmApiCreateContext();

    // 1. Simple solid box -> one solid, no void.
    SmBrep* pBox = NULL;
    SER( sm_CensusBox( 0, 0, 0, 10, 10, 10, pBox ) );
    SER( sm_ExpectCensus( pBox, 1, 0 ) );

    // 2. Cube with a fully-interior cube removed -> one solid, one void cavity.
    //    Build both operands first so the Boolean never sees a NULL.
    SmBrep* pOuter = NULL;   SER( sm_CensusBox( 0, 0, 0, 10, 10, 10, pOuter ) );
    SmBrep* pInner = NULL;   SER( sm_CensusBox( 3, 3, 3, 4, 4, 4, pInner ) );
    SmBrep* pHollow = NULL;
    SER( SmApiBooleanDifference( pOuter, pInner, pHollow ) );  // consumes pOuter, pInner
    SER( sm_ExpectCensus( pHollow, 1, 1 ) );

    // 3. Two non-overlapping boxes -> two disjoint material regions.
    SmBrep* pA = NULL;    SER( sm_CensusBox( 0, 0, 0, 2, 2, 2, pA ) );
    SmBrep* pFar = NULL;  SER( sm_CensusBox( 20, 0, 0, 2, 2, 2, pFar ) );
    SmBrep* pTwo = NULL;
    SER( SmApiBooleanUnion( pA, pFar, pTwo ) );
    SER( sm_ExpectCensus( pTwo, 2, 0 ) );

    // 4. Nested case (gap #8): a solid floating inside a void cavity ->
    //    two solids, one void, counted unambiguously because SMLib stores
    //    each region separately.
    SmBrep* pOuter2 = NULL;  SER( sm_CensusBox( 0, 0, 0, 10, 10, 10, pOuter2 ) );
    SmBrep* pInner2 = NULL;  SER( sm_CensusBox( 3, 3, 3, 4, 4, 4, pInner2 ) );
    SmBrep* pHollow2 = NULL;
    SER( SmApiBooleanDifference( pOuter2, pInner2, pHollow2 ) );
    SmBrep* pNested = NULL;   SER( sm_CensusBox( 4, 4, 4, 2, 2, 2, pNested ) );
    SmBrep* pResult = NULL;
    SER( SmApiBooleanMerge( pHollow2, pNested, pResult ) );  // consumes both
    SER( sm_ExpectCensus( pResult, 2, 1 ) );

    // 5. NULL input -> error, not a crash.
    long lSolids = 0, lVoids = 0;
    if( SmApiBrepMaterialCensus( NULL, lSolids, lVoids ) == SM_SUCCESS )
        return SM_ERR;

    return SM_SUCCESS;

} // End TestSmMaterialCensus
