// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************//**
* FILE NAME --- SmSurfaceTracer.cpp
* PURPOSE: This file contains surface intersector methods.
**********************************************************************/

#include "StdAfx.h"

#include <SmDirectedSilhouette.h>
#include <SmBSplineSurface.h>
#include <SmBSplineCurve.h>
#include <SmSolutionArray.h>
#include <SmGraphicsExtern.h>
#include <SmHermiteCurve.h>
#include <SmSurfaceCache.h>
#include <SmCurveCache.h>
#include <SmGeomUtility.h>

static void InsertDataToSortedArray(SmTArray<double> & arr, double value);

/*******************************************************************//**
PURPOSE: Initialize all of the values of the span.  Compute what is
    needed.

NOTES: 
***********************************************************************/
SmStatus SmDSSpan::Init(SmDirectedSilhouette * pDirectedSilhouette,
                        const SmBSplineCurve * pCurve,
                        const SmExtent1d & rInterval)
{
    m_pDirectedSilhouette = pDirectedSilhouette;
    m_pCurve = pCurve;
    m_sInterval = rInterval;
    double dMin, dMax;
    SER(CurveParameterToLeftRight(m_sInterval.GetMin(),dMin));
    SER(CurveParameterToLeftRight(m_sInterval.GetMax(),dMax));
    if (dMin < dMax) {
        m_bDirection = TRUE;
        m_sLRInterval.SetMinMax(dMin,dMax);
    }
    else {
        m_bDirection = FALSE;
        m_sLRInterval.SetMinMax(dMax,dMin);
    }
    return SM_SUCCESS;
}

/*******************************************************************//**
PURPOSE:  Given a left right parameter compute the curve parameter 
    by doing an intersection with a plane locally.
      

NOTES: 
***********************************************************************/
SmStatus SmDSSpan::LeftRightToCurveParameter(double dLeftRightParameter, 
                                             double & rdCurveParameter) const
{
    double dD = - dLeftRightParameter;  // Makes origin always [0,0,0]
    SmSolution sSolution;
    SmBoolean bFoundAnswer; 

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
        if (bDebugMe) {
            smgfx_Erase();
            smgfx_SetColor(0,0,1);
            m_pCurve->DrawWDeriv(m_sInterval,0);
            SmPoint3d sPnt = m_pDirectedSilhouette->m_vLeftRightVector * dLeftRightParameter;
            smgfx_SetColor(1,0,0);
            sPnt.Draw();
            m_pDirectedSilhouette->m_vLeftRightVector.Draw(&sPnt);
            sm_GraphicsLoop();
        }
#endif


    SER(m_pCurve->LocalPropertyAnalysis(m_sInterval,SM_CP_PLANE_INTERSECTION,
        m_sInterval.Evaluate(0.5),&dD,&m_pDirectedSilhouette->m_vLeftRightVector,
        bFoundAnswer,sSolution));
    if (!bFoundAnswer) {
        SER(m_pCurve->LocalPropertyAnalysis(m_sInterval,SM_CP_PLANE_INTERSECTION,
            m_sInterval.Evaluate(0.1),&dD,&m_pDirectedSilhouette->m_vLeftRightVector,
            bFoundAnswer,sSolution));
        if (!bFoundAnswer) {
            SER(m_pCurve->LocalPropertyAnalysis(m_sInterval,SM_CP_PLANE_INTERSECTION,
                m_sInterval.Evaluate(0.9),&dD,&m_pDirectedSilhouette->m_vLeftRightVector,
                bFoundAnswer,sSolution));
            if (!bFoundAnswer) {
                // Can not find intersection just use the closest end point
                SmPoint3d sPnt;
                SER(m_pCurve->EvaluatePoint(m_sInterval.GetMin(),sPnt));
                double dStartDist = m_pDirectedSilhouette->m_vLeftRightVector.Dot(sPnt);
                SER(m_pCurve->EvaluatePoint(m_sInterval.GetMax(),sPnt));
                double dEndDist = m_pDirectedSilhouette->m_vLeftRightVector.Dot(sPnt);
                if (smos_Fabs(dStartDist-dLeftRightParameter) < smos_Fabs(dEndDist-dLeftRightParameter)) {
                    rdCurveParameter = m_sInterval.GetMin();
                }
                else { rdCurveParameter = m_sInterval.GetMax(); }
                return SM_SUCCESS;
            }
        }
    }
    rdCurveParameter = sSolution.m_vStart[0];
    return SM_SUCCESS;
}

/*******************************************************************//**
PURPOSE: Compute the local coordinate relative to the projection
   plane which is located at the origin.  The LeftRightVector corresponds
   to the X axis and the ViewingDirection vector corresponds to the
   Y axis.

NOTES: 
***********************************************************************/
SmStatus SmDSSpan::EvaluateProjection(double dCurveParameter,
                                      SmPoint2d & rProjPlanePoint) const
{
    SmPoint3d sPnt;
    SER(m_pCurve->EvaluatePoint(dCurveParameter,sPnt));
    rProjPlanePoint.x = m_pDirectedSilhouette->m_vLeftRightVector.Dot(sPnt);
    rProjPlanePoint.y = m_pDirectedSilhouette->m_vViewingDirection.Dot(sPnt);
    return SM_SUCCESS;
}

/*******************************************************************//**
PURPOSE: Compute the parameter along the LeftRightVector of the 
    point on the span.

NOTES: 
***********************************************************************/
SmStatus SmDSSpan::CurveParameterToLeftRight(double dCurveParameter,
                                             double & rdLeftRightParameter) const
{
    SmPoint2d sPnt;
    SER(EvaluateProjection(dCurveParameter,sPnt));
    rdLeftRightParameter = sPnt.x;
    return SM_SUCCESS;
}

/*******************************************************************//**
PURPOSE: Get the LeftRight parameter corresponding to the requested 
   end.  If TRUE then return the MAX or Right most, if FALSE return the
   leftmost.

NOTES: 
***********************************************************************/
double SmDSSpan::GetLRParameter(SmBoolean bDoRightEnd) const
{
    if (bDoRightEnd) return m_sLRInterval.GetMax();
    return m_sLRInterval.GetMin();
}


/*******************************************************************//**
PURPOSE: Destructor for directed silhouette.

NOTES: 
***********************************************************************/
SmDirectedSilhouette::~SmDirectedSilhouette() 
{
    if (m_pStartSpans) { delete m_pStartSpans; m_pStartSpans = NULL ; }
    if (m_pEndSpans)   { delete m_pEndSpans;   m_pEndSpans   = NULL ; }
    if (m_pFrontSpans) { delete m_pFrontSpans; m_pFrontSpans = NULL ; }
}

/*******************************************************************//**
PURPOSE: Constructor for directed silhouette.

NOTES: 
***********************************************************************/
SmDirectedSilhouette::SmDirectedSilhouette(const SmContext & crContext,
                                           const SmVector3d & crPlaneNormal,
                                           const SmVector3d & crViewingDirection)
    : m_crContext(crContext)
{
    // Make sure vectors are perpendicular
    SM_ASSERT(smos_Fabs(crPlaneNormal.Dot(crViewingDirection)) < SM_EFF_ZERO);
    SM_ASSERT(crPlaneNormal.LengthSquared() > SM_EFF_ZERO_SQ);
    SM_ASSERT(crViewingDirection.LengthSquared() > SM_EFF_ZERO_SQ);
    m_vPlaneNormal = crPlaneNormal;
    m_vViewingDirection = crViewingDirection;
    SE(m_vPlaneNormal.Unitize());
    SE(m_vViewingDirection.Unitize());
    m_vLeftRightVector = m_vViewingDirection * m_vPlaneNormal;
    SE(m_vLeftRightVector.Unitize());
    m_pStartSpans = new (m_crContext) SmTArray<SmDSSpan*>(m_crContext);
    m_pEndSpans = new (m_crContext) SmTArray<SmDSSpan*>(m_crContext);
    m_pFrontSpans = new (m_crContext) SmTArray<SmDSSpan*>(m_crContext);
}

/*******************************************************************//**
PURPOSE: Create the spans and sort them into start and end lists.

NOTES: 
***********************************************************************/
SmStatus SmDirectedSilhouette::CreateSpans(const SmTArray<SmBSplineCurve*> & crCurves)
{
    ULONG i, j, k;
    ULONG lCount = crCurves.GetSize();
    SmSolutionArray sSolutions;
    SmBSplineCurve * pBSC1 = NULL;
    SmBSplineCurve * pBSC2 = NULL;

    SmTArray<double> * pParameterArrays = new SmTArray<double>[lCount];
    SmTypedArrayDelete< SmTArray<double> > sCleanArray(pParameterArrays) ;

    // Intersect each curve with all other curves
    for (i=0; i<lCount; i++) {
        pBSC1 = crCurves[i];
        SmExtent1d sCrvIvl = pBSC1->GetNaturalInterval();
        InsertDataToSortedArray(pParameterArrays[i], sCrvIvl.GetMin());
        InsertDataToSortedArray(pParameterArrays[i], sCrvIvl.GetMax());

        // Find where the curve's tangents are perpendicular to the LeftRightVector
        SER(pBSC1->GlobalPropertyAnalysis(sCrvIvl,
                SM_CP_PERPENDICULAR_TO_VECTOR, NULL, &m_vLeftRightVector, 
                m_d3DTolerance, sSolutions));

        for (k=0; k<sSolutions.GetSize(); k++) {
            SmSolution & rSol = sSolutions[k];
            InsertDataToSortedArray(pParameterArrays[i], rSol.m_vStart[0]);
        }

        for (j=i+1; j<lCount; j++) {
            pBSC2 = crCurves[j];
     
            SER(pBSC1->GlobalCurveSolve(sCrvIvl,
                                       *pBSC2,
                                        pBSC2->GetNaturalInterval(),
                                        SM_SO_PROJECTED_INTERSECT, 
                                        SM_ZONE_TOL_3D,
                                        NULL, 
                                       &m_vPlaneNormal,
                                        SM_SR_ALL,
                                        sSolutions));

            // If found intersection, insert results to parameter arrays. Those
            // parameters are the places where curves need to be splitted
            for (k=0; k<sSolutions.GetSize(); k++) {
                SmSolution & rSol = sSolutions[k];
                InsertDataToSortedArray(pParameterArrays[i], rSol.m_vStart[0]);
                InsertDataToSortedArray(pParameterArrays[j], rSol.m_vStart[1]);
            }
        }
    }

    m_vSpanMgr.Initialize(ALIGN_SIZE(sizeof(SmDSSpan)),2 * lCount) ;
    m_pStartSpans->SetDataSize(2*lCount);
    m_pEndSpans->SetDataSize(2*lCount);
    // Now, split each curve according to the parameters array
    SmExtent1d sIvl;
    for (i=0; i<lCount; i++) {
        pBSC1 = crCurves[i];
        for (j=1; j<pParameterArrays[i].GetSize(); j++) {
            // Check for very small intervals
            sIvl.SetMinMax(pParameterArrays[i][j-1],
                           pParameterArrays[i][j]); 
            if (sIvl.GetMax() - sIvl.GetMin() < SM_EFF_ZERO_SQRT)
                continue;

            SmDSSpan * pSpan = (SmDSSpan*)m_vSpanMgr.GetNewElement();
            SER(pSpan->Init(this,pBSC1,sIvl));
            SER(InsertSpanIntoList(pSpan,FALSE,*m_pStartSpans));
            SER(InsertSpanIntoList(pSpan,TRUE,*m_pEndSpans));
        }
    }

  // GWC: avoid memory leaks replaced following with SmTypedArrayDelete<TYPE> style of delete
  // delete [] pParameterArrays; pParameterArrays = NULL ;

    return SM_SUCCESS;
}

/*******************************************************************//**
PURPOSE: Insert a value into a sorted double-array (static function)

NOTES: 
***********************************************************************/

void InsertDataToSortedArray(SmTArray<double> & arr, double value)
{
    ULONG lInsertPlace = arr.GetSize();
    for (ULONG i=0; i<arr.GetSize(); i++) {
        if (value < arr[i]) {
            lInsertPlace = i;
            break;
        }
    }
    arr.InsertAt(lInsertPlace,value,1);
}


/*******************************************************************//**
PURPOSE: Insert the span into the list relative to one of its ends.

NOTES: 
***********************************************************************/
SmStatus SmDirectedSilhouette::InsertSpanIntoList(SmDSSpan * pSpan,
                                                  SmBoolean bDoingRightEnd,
                                                  SmTArray<SmDSSpan*> & rSortedSpans)
{
    double dLRInsert = pSpan->GetLRParameter(bDoingRightEnd);
    ULONG lInsertPlace = rSortedSpans.GetSize();
    for (ULONG i=0; i<rSortedSpans.GetSize(); i++) {
        SmDSSpan *pTest = rSortedSpans[i];
        double dLRTest = pTest->GetLRParameter(bDoingRightEnd);
        if (dLRInsert < dLRTest) {
            lInsertPlace = i;
            break;
        }
    }
    rSortedSpans.InsertAt(lInsertPlace,pSpan,1);
    return SM_SUCCESS;
}

/*******************************************************************//**
PURPOSE: Find the spans which are frontmost in the list.

NOTES: 
***********************************************************************/
SmStatus SmDirectedSilhouette::FindFrontmostSpans()
{ 
    SmTArray<SmDSSpan*> sActiveSpans;
    SmDSSpan * pCurrFront = NULL;
    double dCurrFrontStartLRParam = - SM_BIG_DOUBLE;
    double dCurrLRParam = - SM_BIG_DOUBLE;

    SmTArray<SmDSSpan*> sStartSpans;
    SmTArray<SmDSSpan*> sEndSpans;

    while (m_pStartSpans->GetSize() != 0 || m_pEndSpans->GetSize() != 0) {

        // Update the active span list
        double dStartLR = SM_BIG_DOUBLE;
        double dEndLR = SM_BIG_DOUBLE;
        SER(GetNextStartSpans(sStartSpans,dStartLR));
        SER(GetNextEndSpans(sEndSpans,dEndLR));

//        if (GetNextTsectPoint.... < dStartLR and dEndLR) {
//            set current LR value and see which is in front
//      else 

        if (smos_Fabs(dStartLR-dEndLR) < m_d3DTolerance || 
            dStartLR < dEndLR) {
            dCurrLRParam = dStartLR;
            m_pStartSpans->RemoveAt(0,sStartSpans.GetSize());
            sActiveSpans.Append(sStartSpans);
            // If have current front span - intersect it with the new curves coming in.
            // SER(FindSpanCrossings(pCurrFront,dCurrFrontStartLRParam,sStartSpans));
        }
        if (smos_Fabs(dStartLR-dEndLR) < m_d3DTolerance ||
            dEndLR < dStartLR) {
            dCurrLRParam = dEndLR;
            m_pEndSpans->RemoveAt(0,sEndSpans.GetSize());
            // Remove end spans from active elements.
            for (ULONG i=0; i<sEndSpans.GetSize(); i++) {
                ULONG lFoundIndex;
                if (sActiveSpans.FindElement(sEndSpans[i],lFoundIndex)) {
                    sActiveSpans.RemoveAt(lFoundIndex,1);
                }
                else { SER(SM_ERR); }
            }
        }

        // We have an active span which is updated.
        // Now find the frontmost span at the current parameter plus a little
        // to resolve ambiguities at the end points.
        SmDSSpan *pNewFront = NULL;
        double dFrontValue = SM_BIG_DOUBLE;
        double dTestParam = dCurrLRParam + m_d3DTolerance * 10.0;
        for (ULONG i=0; i<sActiveSpans.GetSize(); i++) {
            SmDSSpan * pTest = sActiveSpans[i];
            double dCurveParam;
            SER(pTest->LeftRightToCurveParameter(dTestParam,dCurveParam));
            SmPoint2d sProjPnt;
            SER(pTest->EvaluateProjection(dCurveParam,sProjPnt));
            if (sProjPnt.y < dFrontValue) {
                dFrontValue = sProjPnt.y;
                pNewFront = pTest;
            }
        }

        if (pCurrFront == NULL || pNewFront == pCurrFront) {
            // Do nothing just continue on - no changes required
            if (pCurrFront == NULL) {
                // Here we are making a new front span - intersect with active list.
                pCurrFront = pNewFront;
                dCurrFrontStartLRParam = dCurrLRParam;
                // sCrossings.ReSet();
                // SER(FindSpanCrossings(pCurrFront,dCurrFrontStartLRParam,sActiveList));
            }
            continue;
        }

        // If made it here we have a change in the front span 
        // -- Flush old span and start new one.
        SER(FlushFrontSpan(pCurrFront,dCurrFrontStartLRParam,dCurrLRParam));

        // Start new front span.
        pCurrFront = pNewFront;
        dCurrFrontStartLRParam = dCurrLRParam;

        // Here intersect with active list.  Only Keep intersections which are
        // less then a certain value.
        // sCrossings.ReSet();
        // SER(FindSpanCrossings(pCurrFront,dCurrFrontStartLRParam,sActiveList));
    }

    return SM_SUCCESS;
}


/*******************************************************************//**
PURPOSE: This method is the public interface to get the work done.

NOTES: 
***********************************************************************/
SmStatus SmDirectedSilhouette::FindVisibleSegments(const SmTArray<SmBSplineCurve*> & crCurves,
                                                   double d3DTolerance)
{
    SmTArray<SmDSSpan*> sAllSpans;
    m_d3DTolerance = d3DTolerance;
    SER(CreateSpans(crCurves));//, sAllSpans));
//    SER(ProjectIntersectAllSpans(sAllSpans));
    SER(FindFrontmostSpans());
    return SM_SUCCESS;
}


/*******************************************************************//**
PURPOSE: 

NOTES: 
***********************************************************************/
SmStatus SmDirectedSilhouette::ExtractVisibleSegments(SmTArray<SmBSplineCurve*> & rVisibleSegments) const
{
    for (ULONG i=0; i<m_pFrontSpans->GetSize(); i++) {
        SmDSSpan *pSpan = (*m_pFrontSpans)[i];
        NER(pSpan);
        SmBSplineCurve *pCurve = new (m_crContext) SmBSplineCurve(*pSpan->m_pCurve);
        NER(pCurve);
        double dScaledTol = SM_EFF_ZERO * (1.0 + smos_Fabs(pSpan->m_sInterval.GetMin()) + 
            smos_Fabs(pSpan->m_sInterval.GetMax()));
        // Skip very small segments
        if (pSpan->m_sInterval.GetLength() < dScaledTol) {
            continue;
        }
        SER(pCurve->Trim(pSpan->m_sInterval)); // may snap sIvl by tol to existing knots
        rVisibleSegments.Add(pCurve);
    }
    return SM_SUCCESS;
}

/*******************************************************************//**
PURPOSE: 

NOTES: 
***********************************************************************/
SmStatus SmDirectedSilhouette::GetNextStartSpans(SmTArray<SmDSSpan*> & rNextStartSpans,
                                                 double & rdSpanStartValue) const
{
    rNextStartSpans.ReSet();
    if (m_pStartSpans->GetSize() == 0) return SM_SUCCESS;
    SmDSSpan *pSpan = (*m_pStartSpans)[0];
    rNextStartSpans.Add(pSpan);
    rdSpanStartValue = pSpan->GetLRParameter(FALSE); // Gets left parameter
    for (ULONG i=1; i<m_pStartSpans->GetSize(); i++) {
        SmDSSpan *pTestSpan = (*m_pStartSpans)[i];
        double dTestVal = pTestSpan->GetLRParameter(FALSE);
        if (smos_Fabs(dTestVal-rdSpanStartValue) > m_d3DTolerance) {
            return SM_SUCCESS;
        }
        rNextStartSpans.Add(pTestSpan);
    }
    return SM_SUCCESS;
}



/*******************************************************************//**
PURPOSE: 

NOTES: 
***********************************************************************/
SmStatus SmDirectedSilhouette::GetNextEndSpans(SmTArray<SmDSSpan*> & rNextEndSpans,
                                               double & rdSpanEndValue) const
{
    rNextEndSpans.ReSet();
    if (m_pEndSpans->GetSize() == 0) return SM_SUCCESS;
    SmDSSpan *pSpan = (*m_pEndSpans)[0];
    rNextEndSpans.Add(pSpan);
    rdSpanEndValue = pSpan->GetLRParameter(TRUE); // Gets right parameter
    for (ULONG i=1; i<m_pEndSpans->GetSize(); i++) {
        SmDSSpan *pTestSpan = (*m_pEndSpans)[i];
        double dTestVal = pTestSpan->GetLRParameter(TRUE);
        if (smos_Fabs(dTestVal-rdSpanEndValue) > m_d3DTolerance) {
            return SM_SUCCESS;
        }
        rNextEndSpans.Add(pTestSpan);
    }
    return SM_SUCCESS;
}

/*******************************************************************//**
PURPOSE: 

NOTES: 
***********************************************************************/
SmStatus SmDirectedSilhouette::FlushFrontSpan(const SmDSSpan * pSpanToFlush,
                                              double dStartLRParam, 
                                              double dEndLRParam)
{
    double dParamMin, dParamMax;
    SER(pSpanToFlush->LeftRightToCurveParameter(dStartLRParam,dParamMin));
    SER(pSpanToFlush->LeftRightToCurveParameter(dEndLRParam,dParamMax));
    SmExtent1d sIvl(dParamMin);
    sIvl.AddValue(dParamMax);
    SmDSSpan *pSpan = (SmDSSpan*)m_vSpanMgr.GetNewElement();
    SER(pSpan->Init(this,pSpanToFlush->m_pCurve,sIvl));
    m_pFrontSpans->Add(pSpan);
    return SM_SUCCESS;
}



