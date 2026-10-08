// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************//**
* FILE NAME --- SmCubicBezierSurface.cpp 
* PURPOSE: Implementation of SmCubicBezierSurface methods.
**********************************************************************/

#include "StdAfx.h"
#include <SmCubicBezierSurface.h>
#include <SmGeomUtility.h>

/*******************************************************************//**
PURPOSE: Compute the axis alligned and non-axis alligned bounding 
     box   

NOTES: At least one of the outputs must be non-NULL.
***********************************************************************/
SmStatus SmCubicBezierSurface::CalculateBoundingBox
  (SmExtent3d * pNormalBox,  // out: Axis alligned box
   SmPseudoBox * pPseudoBox, // out: Non-axis aligned box
   SmPolarBox        * pPolarBox,                    // out: Surface normal vector field bounding box
   SmBoolean           bExpandPosBoxesByZoneTol3d)   // in : TRUE = returned Normal & Pseudo BBoxes = BBox->ExpandAbsoluate(ZoneTol3d)
                                                     //    : FALSE= returned Normal & Pseudo BBoxes = BBox with no expansion 
                                                     //      default:[FALSE] = previous behavior
  const
{ 
    if (pPolarBox != NULL) SER(SM_ERR); // Not handled yet
    SM_ASSERT(pNormalBox != NULL || pPseudoBox != NULL || pPolarBox != NULL);
    if (pPseudoBox) {
        SmVector3d sV1 = (m_vP[0][0] - m_vP[1][0]) + (m_vP[0][1] - m_vP[1][1]);
        SmVector3d sV2 = (m_vP[0][0] - m_vP[0][1]) + (m_vP[1][0] - m_vP[1][1]);
        double sV1LS = sV1.LengthSquared();
        // Note that we will just use the standard basis vectors if the Surface is
        // closed.  If the mid point lies on same line as the start and end point
        // then we will get two arbitrary vectors for Basis2 and Basis3 otherwise V2 will
        // determine the direction for Basis2 and orthogonal direction for Basis3
        if (sV1LS > SM_EFF_ZERO_SQ) {  
            SmVector3d sBasis1, sBasis2, sBasis3;
            sV1.MakeUnitOrthoVectors(&sV2,sBasis1,sBasis2,sBasis3);
            *pPseudoBox = SmPseudoBox(); // Initialize it
            pPseudoBox->SetBasis(sBasis1,sBasis2,sBasis3);
        } 
        else {
            *pPseudoBox = SmPseudoBox(); // Initialize it
        }
    }

    SmPoint3d sPoint;
    SmPoint3d sLast;
    for (long i=0; i<4; i++) {
        for (long j=0; j<4; j++) {
            SmPoint3d & rPnt = m_vP[i][j];
            if (pNormalBox) {
                if (i==0 && j==0) *pNormalBox = SmExtent3d(rPnt);
                else pNormalBox->AddPoint3d(rPnt);
            }
            if (pPseudoBox) pPseudoBox->AddPoint3d(rPnt);
        }
    }

    // When asked, expand the normal box by the zone tolerance
    if(bExpandPosBoxesByZoneTol3d && pNormalBox)
      { pNormalBox->ExpandAbsolute( SM_ZONE_TOL_3D); }

    // When asked, expand the pPseudoBox box by the zone tolerance
    if(bExpandPosBoxesByZoneTol3d && pPseudoBox)
      { pPseudoBox->ExpandAbsolute( SM_ZONE_TOL_3D); }

    return SM_SUCCESS;

} // end SmCubicBezierSurface::CalculateBoundingBox

/*******************************************************************//**
PURPOSE: Split a cubic Bezier surface given a split direction and 
     two split points and derivatives (scaled) from the surface.

NOTES:  This assumes that the surface U and V which gives the
     points and derivatives align up with the Bezier U and V directions.
     Also note that the derivatives have been multiplied by the U V domain
     size of the original surface.  The Bezier is assumed to always be
     parameterized from 0 to 1.  
***********************************************************************/
SmStatus SmCubicBezierSurface::Split
  (SmSurfParamType eSurfParam, // SM_SP_U splits colunms
   SmPoint3d sSplitPnt,        // Split point on the minimum row or column which
                               // corresponds to 1/2 of the parameter domain of the original domain which
                               // created this bezier.
   SmVector3d sDU,
   SmVector3d sDV,
   SmVector3d sDUV,
   SmPoint3d sSplitPnt2,
   SmVector3d sDU2,
   SmVector3d sDV2,
   SmVector3d sDUV2,
   SmCubicBezierSurface & rSurf1,
   SmCubicBezierSurface & rSurf2) 
  const
{
    // First try a split
    if (eSurfParam == SM_SP_U) { // Split Colunms
        SmBoolean bSame1 = FALSE;
        if (&rSurf1 == this) { bSame1 = TRUE; }
        SmBoolean bSame2 = FALSE;
        if (&rSurf2 == this) { bSame2 = TRUE; }
        // Copy first and last rows and scale 
        for (ULONG i=0; i<4; i++) {
            // Column 1
            if (!bSame1) { rSurf1.m_vP[i][0] = m_vP[i][0]; }
            // Column 2
            rSurf1.m_vP[i][1] = (m_vP[i][0] + m_vP[i][1]) / 2.0;
            // Column 4
            if (!bSame2) { rSurf2.m_vP[i][3] = m_vP[i][3]; }
            // Column 3
            rSurf2.m_vP[i][2] = (m_vP[i][2] + m_vP[i][3]) / 2.0;
        }
        // Now compute the remaining columns from the input points and derivs
        rSurf1.m_vP[0][3] = rSurf2.m_vP[0][0] = sSplitPnt;
        SmVector3d sDVVec = sDV/3.0;
        rSurf1.m_vP[1][3] = rSurf2.m_vP[1][0] = sSplitPnt + sDVVec;

        SmVector3d sDUVec = sDU/3.0;
        rSurf1.m_vP[0][2] = sSplitPnt - sDUVec;
        rSurf2.m_vP[0][1] = sSplitPnt + sDUVec;

        SmVector3d sDUVVec = sDUV/9.0;
        rSurf1.m_vP[1][2] = rSurf1.m_vP[0][2] + sDVVec - sDUVVec;
        rSurf2.m_vP[1][1] = rSurf2.m_vP[0][1] + sDVVec + sDUVVec;

        // Now do top half 
        rSurf1.m_vP[3][3] = rSurf2.m_vP[3][0] = sSplitPnt2;
        SmVector3d sDVVec2 = sDV2/3.0;
        rSurf1.m_vP[2][3] = rSurf2.m_vP[2][0] = sSplitPnt2 - sDVVec2;

        SmVector3d sDUVec2 = sDU2/3.0;
        rSurf1.m_vP[3][2] = sSplitPnt2 - sDUVec2;
        rSurf2.m_vP[3][1] = sSplitPnt2 + sDUVec2;

        SmVector3d sDUVVec2 = sDUV2/9.0;
        rSurf1.m_vP[2][2] = rSurf1.m_vP[3][2] - sDVVec2 + sDUVVec2;
        rSurf2.m_vP[2][1] = rSurf2.m_vP[3][1] - sDVVec2 - sDUVVec2;
    }

    if (eSurfParam == SM_SP_V) { // Split Rows
        SmBoolean bSame1 = FALSE;
        if (&rSurf1 == this) { bSame1 = TRUE; }
        SmBoolean bSame2 = FALSE;
        if (&rSurf2 == this) { bSame2 = TRUE; }
        // Copy first and last rows and scale 
        for (ULONG i=0; i<4; i++) {
            // Column 1
            if (!bSame1) { rSurf1.m_vP[0][i] = m_vP[0][i]; }
            // Column 2
            rSurf1.m_vP[1][i] = (m_vP[0][i] + m_vP[1][i]) / 2.0;
            // Column 4
            if (!bSame2) { rSurf2.m_vP[3][i] = m_vP[3][i]; }
            // Column 3
            rSurf2.m_vP[2][i] = (m_vP[2][i] + m_vP[3][i]) / 2.0;
        }
        // Now compute the remaining columns from the input points and derivs
        SmVector3d sDVVec = sDV/3.0;
        SmVector3d sDUVec = sDU/3.0;
        SmVector3d sDUVVec = sDUV/9.0;

        rSurf1.m_vP[3][0] = rSurf2.m_vP[0][0] = sSplitPnt;
        rSurf1.m_vP[3][1] = rSurf2.m_vP[0][1] = sSplitPnt + sDUVec;

        rSurf1.m_vP[2][0] = sSplitPnt - sDVVec;
        rSurf2.m_vP[1][0] = sSplitPnt + sDVVec;

        rSurf1.m_vP[2][1] = rSurf1.m_vP[2][0] + sDUVec - sDUVVec;
        rSurf2.m_vP[1][1] = rSurf2.m_vP[1][0] + sDUVec + sDUVVec;

        // Now do top half 
        SmVector3d sDVVec2 = sDV2/3.0;
        SmVector3d sDUVec2 = sDU2/3.0;
        SmVector3d sDUVVec2 = sDUV2/9.0;
        rSurf1.m_vP[3][3] = rSurf2.m_vP[0][3] = sSplitPnt2;
        rSurf1.m_vP[3][2] = rSurf2.m_vP[0][2] = sSplitPnt2 - sDUVec2;

        rSurf1.m_vP[2][3] = sSplitPnt2 - sDVVec2;
        rSurf2.m_vP[1][3] = sSplitPnt2 + sDVVec2;

        rSurf1.m_vP[2][2] = rSurf1.m_vP[2][3] - sDUVec2 + sDUVVec2;
        rSurf2.m_vP[1][2] = rSurf2.m_vP[1][3] - sDUVec2 - sDUVVec2;
    }

    return SM_SUCCESS;

} // end SmCubicBezierSurface::Split

/*******************************************************************//**
PURPOSE: Build a bilinear Bezier surface.

NOTES: 
***********************************************************************/
SmStatus SmCubicBezierSurface::BuildBilinear
  (SmPoint3d sP00,
   SmPoint3d sP10,
   SmPoint3d sP01,
   SmPoint3d sP11)
{
    m_vP[0][0] = sP00;
    m_vP[0][3] = sP10;
    m_vP[3][0] = sP01;
    m_vP[3][3] = sP11;
    // Bottom
    SmVector3d sV = sP10 - sP00;
    m_vP[0][1] = sP00 + sV/3.0;
    m_vP[0][2] = sP00 + 2.0 * sV / 3.0;

    // Left 
    sV = sP01 - sP00;
    m_vP[1][0] = sP00 + sV/3.0;
    m_vP[2][0] = sP00 + 2.0 * sV / 3.0;

    // Right 
    sV = sP11 - sP10;
    m_vP[1][3] = m_vP[0][3] + sV /  3.0;
    m_vP[2][3] = m_vP[0][3] + 2.0 * sV /  3.0;

    // Top 
    sV = sP11 - sP01;
    m_vP[3][1] = m_vP[3][0] + sV /  3.0;
    m_vP[3][2] = m_vP[3][0] + 2.0 * sV /  3.0;

    // Middle of second row
    sV = m_vP[1][3] - m_vP[1][0];
    m_vP[1][1] = m_vP[1][0] + sV / 3.0;
    m_vP[1][2] = m_vP[1][0] + 2.0 * sV / 3.0;

    // Middle of second row
    sV = m_vP[2][3] - m_vP[2][0];
    m_vP[2][1] = m_vP[2][0] + sV / 3.0;
    m_vP[2][2] = m_vP[2][0] + 2.0 * sV / 3.0;

    return SM_SUCCESS;

} // end SmCubicBezierSurface::BuildBilinear

/*******************************************************************//**
PURPOSE: Compute the chord heights in the U and V direction. 

NOTES: Note that the chord height is only relative to an
    individual cross section of the control net.  An estimate of 
    the actual chord height can be obtained by adding these two
    chord heights.
***********************************************************************/
SmStatus SmCubicBezierSurface::ComputeNetConstants
  (double * pdUChordHeight,          // out: 
   double * pdVChordHeight,          // out: 
   double * pdUAngleDeg,             // out: 
   double * pdVAngleDeg)             // out: 
  const
{
  // init output
  if (pdUChordHeight) *pdUChordHeight = 0.0;
  if (pdVChordHeight) *pdVChordHeight = 0.0;
  if (pdUAngleDeg)    *pdUAngleDeg    = 0.0;
  if (pdVAngleDeg)    *pdVAngleDeg    = 0.0;

  // locals
  double dMaxUAngle = 0.0;
  double dMaxVAngle = 0.0;

  SmPoint3d  sLinePnt, sTryPoint, sCurr, sNext, sPntOnLine, sLastPoint ;
  SmVector3d sLineVec, sCHVec, sCurrVec ;
  
  // pdVChordHeight and pdVAngleDeg
  {
    // First compute for all U rows - compute V ch and V ang
    // along each row.
    for (long j=0; j<4; j++) 
      {
        sLinePnt = m_vP[j][0];

        if (pdVChordHeight) 
          {
            sLineVec = m_vP[j][3] - sLinePnt;
            ULONG lTryIndex = 2;
            while (sLineVec.LengthSquared() < SM_EFF_ZERO_SQ) 
              {
                // If first and last point coincide try a middle point
                sTryPoint = m_vP[j][lTryIndex];
                sLineVec = sTryPoint - sLinePnt;
                if (lTryIndex == 1) 
                  {
                    // singular point just continue on
                    break;
                  }
                lTryIndex--;
              }
            if (sLineVec.LengthSquared() < SM_EFF_ZERO_SQ) continue; // singular point
          }
        double     dTotalAngle = 0.0;
        SmVector3d sLastVec(0,0,0);
        // Hold U constant and loop accross V points
        sCurr = sLinePnt;
        for (long i=1; i<4; i++) 
          {
            sNext = m_vP[j][i];
            double dLineParam;
            if (pdVChordHeight) 
              {
                SER(smgu_LineClosestPoint(sLinePnt,sLineVec,sNext,dLineParam));
                sPntOnLine = sLinePnt + (sLineVec * dLineParam);
                sCHVec     = sNext - sPntOnLine;
                double dCHSq = sCHVec.LengthSquared();
                if (dCHSq > *pdUChordHeight) *pdUChordHeight = dCHSq;
              }
            if (pdVAngleDeg) 
              {
                sCurrVec = sNext - sCurr;
                double dDist = sCurrVec.Length();
                double dTol  = SM_EFF_ZERO_SQRT * 100.0 * (1.0 + sNext.GetMaxDimension());
                if (dDist < dTol) continue;
                sCurrVec = sCurrVec / dDist;
                if (sLastVec.LengthSquared() > SM_EFF_ZERO_SQ) 
                  {
                    double dCosAngle = sCurrVec.Dot(sLastVec);
                    if (dCosAngle > 1.0) dCosAngle = 1.0;
                    if (dCosAngle < -1.0) dCosAngle = -1.0;
                    dTotalAngle += smos_ArcCosine(dCosAngle);

                  }
                sLastVec = sCurrVec ;
              }
            sCurr = sNext ;
          }
        if (dTotalAngle > dMaxUAngle) dMaxUAngle = dTotalAngle;
      } // Compute constants for each U row
  }
  
  // // pdUChordHeight and pdUAngleDeg
  {
    // First compute for all V columns - compute U ch and U ang
    // along each column
    for (long j=0; j<4; j++) 
      {
        sLinePnt = m_vP[0][j];
      
        if (pdUChordHeight) 
          {
            sLastPoint = m_vP[3][j];
            sLineVec = sLastPoint - sLinePnt;
            ULONG lTryIndex = 2;
            while (sLineVec.LengthSquared() < SM_EFF_ZERO_SQ) 
              {
                // If first and last point coincide try a middle point
                sTryPoint = m_vP[lTryIndex][j]; 
                sLineVec = sTryPoint - sLinePnt;
                if (lTryIndex == 1) 
                  {
                    // singular point just continue on
                    break;
                  }
                lTryIndex--;
              }
            if (sLineVec.LengthSquared() < SM_EFF_ZERO_SQ) continue; // singular point
          }
        
        double dTotalAngle = 0.0;
        SmVector3d sLastVec(0,0,0);
        // Hold U constant and loop accross V points
        sCurr = m_vP[0][j];
        for (long i=1; i<4; i++) 
          {
            sNext = m_vP[i][j];
            if (pdUChordHeight) 
              {
                double dLineParam;
                SER(smgu_LineClosestPoint(sLinePnt,sLineVec,sNext,dLineParam));
                sPntOnLine   = sLinePnt + (sLineVec * dLineParam);
                sCHVec       = sNext - sPntOnLine;
                double dCHSq = sCHVec.LengthSquared();
                if (dCHSq > *pdVChordHeight) *pdVChordHeight = dCHSq;
              }
            if (pdUAngleDeg) 
              {
                sCurrVec     = sNext - sCurr;
                double dTol  = SM_EFF_ZERO_SQRT * 100.0 * (1.0 + sNext.GetMaxDimension());
                double dDist = sCurrVec.Length();
                if (dDist < dTol) continue;
                sCurrVec = sCurrVec / dDist;
                if (sLastVec.LengthSquared() > SM_EFF_ZERO_SQ) 
                  {
                    double dCosAngle = sCurrVec.Dot(sLastVec);
                    if (dCosAngle > 1.0) dCosAngle = 1.0;
                    if (dCosAngle < -1.0) dCosAngle = -1.0;
                    dTotalAngle += smos_ArcCosine(dCosAngle);
                    // smos_ArcCosine(dCosAngle);
                  }
                sLastVec = sCurrVec ;
              }
            sCurr = sNext ;
          }
        if (dTotalAngle > dMaxVAngle) dMaxVAngle = dTotalAngle;
      } // Compute constants for each U row
  }

  // set output
  if (pdUAngleDeg)    *pdUAngleDeg    = dMaxUAngle * 180.0 / SM_PI;
  if (pdVAngleDeg)    *pdVAngleDeg    = dMaxVAngle * 180.0 / SM_PI;
  if (pdUChordHeight) *pdUChordHeight = smos_Sqrt(*pdUChordHeight);
  if (pdVChordHeight) *pdVChordHeight = smos_Sqrt(*pdVChordHeight);
  
  // all done
  return SM_SUCCESS;

} // end SmCubicBezierSurface::ComputeNetConstants

/*******************************************************************//**
PURPOSE: 

NOTES: 
***********************************************************************/
void SmCubicBezierSurface::Dump(void) const
{
    smos_WriteBuffer(_T("SmCubicBezierDump\n"));
    for (ULONG i=0; i<4; i++) {
        for (ULONG j=0; j<4; j++) {
            m_vP[i][j].Dump();  smos_WriteBuffer(_T("  "));
        }
        smos_WriteBuffer(_T("\n"));
    }

} // end SmCubicBezierSurface::Dump

/*******************************************************************//**
PURPOSE: 

NOTES: 
***********************************************************************/
void SmCubicBezierSurface::Draw() const
{
    for (ULONG i=0; i<4; i++) {
        for (ULONG j=0; j<4; j++) {
            m_vP[i][j].Draw();
            if (i > 0) {
                SmVector3d sVec = m_vP[i][j] - m_vP[i-1][j];
                sVec.Draw(&m_vP[i-1][j]);
            }
            if (j > 0) {
                SmVector3d sVec = m_vP[i][j] - m_vP[i][j-1];
                sVec.Draw(&m_vP[i][j-1]);
            }
        }
    }

} // end SmCubicBezierSurface::Draw
