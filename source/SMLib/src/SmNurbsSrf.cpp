// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*******************************************************************//**
* FILE NAME --- SmNurbsSrf.cpp
* PURPOSE: Local copies of GW nurbs functions.
**********************************************************************/

#include "StdAfx.h"

#include <SmNurbsSrf.h>
#include <SmBSplineSurface.h>
#include <SmFace.h>
#include <SmBrep.h>
#include <nurbs.h>
#include <SmGeomUtility.h>


static SM_THREAD_LOCAL TCHAR *rname = NULL;

/*******************************************************************//**
PURPOSE: Find the surface controlPoints nearest/farthest from
            the given control points.

NOTES:  Checks the distance to all points but only stores
   the best lNumToFind results.

ASSUMES -- 1. the output arrays are allocated on input to size:[lNumToFind].
           2. lNumToFind is <= 10.
***********************************************************************/
SmStatus sm_FindNetExtrema
  (const gw_SURFACE * pSur,                  // in : Target Surface
   const SmPoint3d  & rTestPoint,            // in : Target Point
   ULONG              lNumToFind,            // in : Number of controlPoints to find and
                                             //      place in output arrays
   double           * adMinDistSq,           // out: array of TargetPoint/ControlPoint distances sorted by distance
                                             //      sized:[lNumToFind]
   ULONG              alMinUVIndex[10][2],   // out: associated controlPoint [iu, iv] indices for every adMinDistSq entry
   double           * adMaxDistSq,           // out: array of TargetPoint/ControlPoint distances sorted by distance
                                             //      sized:[lNumToFind]
   ULONG              alMaxUVIndex[10][2],   // out: associated controlPoint [iu, iv] indices for every adMaxDistSq entry
   ULONG            & rlNumMinFound,         // out: number of MinDist ControlPoints found [rlNumMinFound <= lNumToFind]
   ULONG            & rlNumMaxFound)         // out: number of MaxDist ControlPoints found [rlNumMinFound <= lNumToFind]
{
  // check input
  SM_ASSERT_MSG(lNumToFind <= 10, _T("sm_FindNetExtrema() Bad Input Value Alert")) ;

  // init output
  rlNumMinFound = 0;
  rlNumMaxFound = 0;

  // locals
  gw_CPOINT **Pw = pSur->net->Pw;

  // for every Surface controlPoint
  ULONG ii, jj;
  long iu, iv;
  for (iu = 0; iu <= pSur->net->n; iu++)
    {
      for (iv = 0; iv <= pSur->net->m; iv++)
        {
          // get TestPoint/ControlPoint[iu,iv] gap
          SmPoint3d sPnt;
          TO_EUCLID(Pw[iu][iv],sPnt);
          SmVector3d sDiffVec = rTestPoint - sPnt;
          double dDistSq = sDiffVec.LengthSquared();

          SmBoolean bInserted = FALSE;

          // for every sorted minDistance ControlPoint
          for (ii=0; ii<rlNumMinFound; ii++)
            {
              // when gap is smaller than current value
              if (dDistSq < adMinDistSq[ii])
                {
                  // insert this ControlPoint into the min arrays output
                  bInserted = TRUE;

                  // make a hole in the saved min arrays
                  for (jj=rlNumMinFound; jj>ii; jj--)
                    {
                      adMinDistSq [jj]    = adMinDistSq[jj-1];
                      alMinUVIndex[jj][0] = alMinUVIndex[jj-1][0];
                      alMinUVIndex[jj][1] = alMinUVIndex[jj-1][1];
                    }

                  // fill the min arrays hole with this controlPoint parameters
                  adMinDistSq [ii]    = dDistSq;
                  alMinUVIndex[ii][0] = iu;
                  alMinUVIndex[ii][1] = iv;

                  // increment the min array count up to lNumToFind
                  rlNumMinFound = smos_Min(rlNumMinFound+1,lNumToFind);
                  break;
                }
            } // end iter every sorted min array value

          // if the point is not yet inserted and there is room in the min Arrays
          if (!bInserted && rlNumMinFound < lNumToFind)
            {
              // add the point to the min arrays
              adMinDistSq[rlNumMinFound]     = dDistSq;
              alMinUVIndex[rlNumMinFound][0] = iu;
              alMinUVIndex[rlNumMinFound][1] = iv;

              // increment the min array count up to lNumToFind
              rlNumMinFound = smos_Min(rlNumMinFound+1,lNumToFind);
            }

          bInserted = FALSE;

          // for every sorted maxDistance ControlPoint
          for (ii=0; ii<rlNumMaxFound; ii++)
            {
              // when gap is larger than current value
              if (dDistSq > adMaxDistSq[ii])
                {
                  // insert this ControlPoint into the max arrays output
                  bInserted = TRUE;

                  // make a hole in the saved max arrays
                  for (jj=rlNumMaxFound; jj>ii; jj--)
                    {
                      adMaxDistSq[jj] = adMaxDistSq[jj-1];
                      alMaxUVIndex[jj][0] = alMaxUVIndex[jj-1][0];
                      alMaxUVIndex[jj][1] = alMaxUVIndex[jj-1][1];
                    }

                  // fill the max arrays hole with this controlPoint parameters
                  adMaxDistSq[ii] = dDistSq;
                  alMaxUVIndex[ii][0] = iu;
                  alMaxUVIndex[ii][1] = iv;

                  // increment the max array count up to lNumToFind
                  rlNumMaxFound = smos_Min(rlNumMaxFound+1,lNumToFind);
                  break;
                }
            } // end iter every sorted max array value

          // if the point is not yet inserted and there is room in the max Arrays
          if (!bInserted && rlNumMaxFound < lNumToFind)
            {
              // add the point to the max arrays
              adMaxDistSq[rlNumMaxFound]     = dDistSq;
              alMaxUVIndex[rlNumMaxFound][0] = iu;
              alMaxUVIndex[rlNumMaxFound][1] = iv;

              // increment the max array count up to lNumToFind
              rlNumMaxFound = smos_Min(rlNumMaxFound+1,lNumToFind);
            }

        } // end iter every V controlPoint index
    } // end iter every U controlPoint index

  // all done
  return SM_SUCCESS;

} // end sm_FindNetExtrema

/*******************************************************************//**
PURPOSE: Compute the chord height, bounding box, Pseudo Box, angular
    tolerances, and corners for a subset of the control polygon.

NOTES:
***********************************************************************/
SmStatus sm_ComputePartialNetConstants
  (const         gw_SURFACE *cpSur,          // in :
   ULONG         lU0Span,                    // in :
   ULONG         lV0Span,                    // in :
   ULONG         lU1Span,                    // in :
   ULONG         lV1Span,                    // in :
   SmExtent3d  * pBoundingBox,               // out:
   SmPseudoBox * pPseudoBox,                 // out:
   SmPoint3d   & rU0V0,                      // out:
   SmPoint3d   & rU1V0,                      // out:
   SmPoint3d   & rU0V1,                      // out:
   SmPoint3d   & rU1V1,                      // out:
   double      * pdUChordHeight,             // out:
   double      * pdVChordHeight,             // out:
   double      * pdUAngleDeg,                // out:
   double      * pdVAngleDeg)                // out:
{
    if (lU0Span > lU1Span) {
        SER(SM_ERR);
    }
    if (lV0Span > lV1Span) {
        SER(SM_ERR);
    }

    gw_CNET   *pNet = cpSur->net;
    gw_CPOINT **Pw  = pNet->Pw;
    double dMaxUAngle = 0.0;
    double dMaxVAngle = 0.0;
    if (pdUChordHeight) { *pdUChordHeight = 0.0; }
    if (pdVChordHeight) { *pdVChordHeight = 0.0; }
    if (pdUAngleDeg   ) { *pdUAngleDeg = 0.0; }
    if (pdVAngleDeg   ) { *pdVAngleDeg = 0.0; }

    ULONG lDegU = cpSur->p ;
    ULONG lDegV = cpSur->q ;

    // Get the range of ControlPoints with nonzero
    // basis functions over the given range of spans.
    ULONG lU0CPoint = lU0Span - lDegU;
    ULONG lU1CPoint = lU1Span;
    ULONG lV0CPoint = lV0Span - lDegV;
    ULONG lV1CPoint = lV1Span;

    ULONG ii, jj;

    if (pBoundingBox) { pBoundingBox->Init(); }

    {  // First compute corner points and set up bounding boxes
        TO_EUCLID(Pw[lU0CPoint][lV0CPoint],rU0V0);
        TO_EUCLID(Pw[lU1CPoint][lV0CPoint],rU1V0);
        TO_EUCLID(Pw[lU0CPoint][lV1CPoint],rU0V1);
        TO_EUCLID(Pw[lU1CPoint][lV1CPoint],rU1V1);
        if (pPseudoBox) {
            SmVector3d sV1 = (rU0V0 - rU1V0) + (rU0V1 - rU1V1);
            SmVector3d sV2 = (rU0V0 - rU0V1) + (rU1V0 - rU1V1);
            double sV1LS = sV1.LengthSquared();
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
    }

    {
        // First compute for all U rows - compute V ch and V ang
        // along each row.
        for (jj=lU0CPoint; jj<=lU1CPoint; jj++) {
            SmPoint3d sLinePnt;
            SmVector3d sLineVec;
            if (pdVChordHeight) {
                TO_EUCLID(Pw[jj][lV0CPoint],sLinePnt);
                SmPoint3d sLastPoint; TO_EUCLID(Pw[jj][lV1CPoint],sLastPoint);
                sLineVec = sLastPoint - sLinePnt;
                ULONG lTryIndex = lV1CPoint-1;
                while (sLineVec.LengthSquared() < SM_EFF_ZERO_SQ) {
                    // If first and last point coincide try a middle point
                    SmPoint3d sTryPoint; TO_EUCLID(Pw[jj][lTryIndex],sTryPoint);
                    sLineVec = sTryPoint - sLinePnt;
                    if (lTryIndex == 0) {
                        // singular point just continue on
                        break;
                    }
                    lTryIndex--;
                }
                if (sLineVec.LengthSquared() < SM_EFF_ZERO_SQ) { continue; } // singular point
            }
            double dTotalAngle = 0.0;
            SmVector3d sLastVec(0,0,0);
            // Hold U constant and loop accross V points
            SmPoint3d sCurr;
            TO_EUCLID(Pw[jj][lV0CPoint], sCurr );

            if ( pBoundingBox ) { pBoundingBox->AddPoint3d(sCurr); }
            if ( pPseudoBox   ) { pPseudoBox->AddPoint3d(sCurr); }

            for (ii=lV0CPoint+1; ii<=lV1CPoint; ii++) {
                SmPoint3d sNext; TO_EUCLID(Pw[jj][ii],sNext);
                if ( pBoundingBox ) { pBoundingBox->AddPoint3d(sNext); }
                if ( pPseudoBox   ) { pPseudoBox  ->AddPoint3d(sNext); }
                double dLineParam;
                if (pdVChordHeight) {
                    SER(smgu_LineClosestPoint(sLinePnt,sLineVec,sNext,dLineParam));
                    SmPoint3d sPntOnLine = sLinePnt + (sLineVec * dLineParam);
                    SmVector3d sCHVec = sNext - sPntOnLine;
                    double dCHSq = sCHVec.LengthSquared();
                    if (dCHSq > *pdVChordHeight) { *pdVChordHeight = dCHSq; }
                }
                if (pdVAngleDeg) {
                    SmVector3d sCurrVec = sNext - sCurr;
                    double dDist = sCurrVec.Length();
                    double dTol = SM_EFF_ZERO_SQRT * 100.0 * (1.0 + sNext.GetMaxDimension());
                    if (dDist < dTol) { break; }
                    sCurrVec = sCurrVec / dDist;
                    if (sLastVec.LengthSquared() > SM_EFF_ZERO_SQ) {
                        double dCosAngle = sCurrVec.Dot(sLastVec);
                        if (dCosAngle > 1.0) dCosAngle = 1.0;
                        if (dCosAngle < -1.0) dCosAngle = -1.0;
                        dTotalAngle += smos_ArcCosine(dCosAngle);

                    }
                    sLastVec = sCurrVec;
                }
                sCurr = sNext;
            }
            if (dTotalAngle > dMaxVAngle) dMaxVAngle = dTotalAngle;
        } // Compute constants for each U row
    }

    {
        // First compute for all V columns - compute U ch and U ang
        // along each column
        for (jj=lV0CPoint; jj<=lV1CPoint; jj++) {
            SmPoint3d sLinePnt;
            SmVector3d sLineVec;
            if (pdUChordHeight) {
                TO_EUCLID(Pw[lU0CPoint][jj],sLinePnt);
                SmPoint3d sLastPoint; TO_EUCLID(Pw[lU1CPoint][jj],sLastPoint);
                sLineVec = sLastPoint - sLinePnt;
                ULONG lTryIndex = lU1CPoint-1;
                while (sLineVec.LengthSquared() < SM_EFF_ZERO_SQ) {
                    // If first and last point coincide try a middle point
                    SmPoint3d sTryPoint;
                    TO_EUCLID(Pw[lTryIndex][jj],sTryPoint);
                    sLineVec = sTryPoint - sLinePnt;
                    if (lTryIndex == 0) {
                        // singular point just continue on
                        break;
                    }
                    lTryIndex--;
                }
                if (sLineVec.LengthSquared() < SM_EFF_ZERO_SQ) { continue; } // singular point
            }

            double dTotalAngle = 0.0;
            SmVector3d sLastVec(0,0,0);
            // Hold U constant and loop accross V points
            SmPoint3d sCurr; TO_EUCLID(Pw[lU0CPoint][jj],sCurr);
            for (ii=lU0CPoint+1; ii<=lU1CPoint; ii++) {
                SmPoint3d sNext; TO_EUCLID(Pw[ii][jj],sNext);
                if (pdUChordHeight) {
                    double dLineParam;
                    SER(smgu_LineClosestPoint(sLinePnt,sLineVec,sNext,dLineParam));
                    SmPoint3d sPntOnLine = sLinePnt + (sLineVec * dLineParam);
                    SmVector3d sCHVec = sNext - sPntOnLine;
                    double dCHSq = sCHVec.LengthSquared();
                    if (dCHSq > *pdUChordHeight) { *pdUChordHeight = dCHSq; }
                }
                if (pdUAngleDeg) {
                    SmVector3d sCurrVec = sNext - sCurr;
                    double dTol = SM_EFF_ZERO_SQRT * 100.0 * (1.0 + sNext.GetMaxDimension());
                    double dDist = sCurrVec.Length();
                    if (dDist < dTol) { break; }
                    sCurrVec = sCurrVec / dDist;
                    if (sLastVec.LengthSquared() > SM_EFF_ZERO_SQ) {
                        double dCosAngle = sCurrVec.Dot(sLastVec);
                        if ( dCosAngle >  1.0 ) { dCosAngle =  1.0; }
                        if ( dCosAngle < -1.0 ) { dCosAngle = -1.0; }
                        dTotalAngle += smos_ArcCosine(dCosAngle);
                        // smos_ArcCosine(dCosAngle);
                    }
                    sLastVec = sCurrVec;
                }
                sCurr = sNext;
            }
            if (dTotalAngle > dMaxUAngle) { dMaxUAngle = dTotalAngle; }
        } // Compute constants for each U row
    }

    if ( pdUAngleDeg    ) { *pdUAngleDeg    = dMaxUAngle * 180.0 / SM_PI; }
    if ( pdVAngleDeg    ) { *pdVAngleDeg    = dMaxVAngle * 180.0 / SM_PI; }
    if ( pdUChordHeight ) { *pdUChordHeight = smos_Sqrt(*pdUChordHeight); }
    if ( pdVChordHeight ) { *pdVChordHeight = smos_Sqrt(*pdVChordHeight); }

    return SM_SUCCESS;
}

/*******************************************************************//**
PURPOSE: Compute the approximate chord heights and turning angles
    in the U, V, and diagonal directions.

NOTES: when pSurface != NULL, compute values from Surface evaluations
       else compute values from Control Net properties.

    Note that the chord height is only relative to an
    individual cross section of the control net.  An estimate of
    the actual chord height can be obtained by adding these two
    chord heights.

    After checking U and V directions a pseudo diagonal direction
    of the surface is checked because this function can underestimate
    chordheight for things like ruled surfaces made from a pair of
    boundary lines when only using U and V cross sections.
***********************************************************************/
SmStatus sm_ComputeNetConstants
  (const SmSurface  * pSurface,         // in : optional surface expected to be nonNULL for non SmBSplineSurface types
   const SmExtent2d * pUVDomain,        // in : only used when pSurface != NULL
   const gw_SURFACE * cpSur,            // in : Shape being tested - always used
   SmZoneTol3d      & rZoneTol3d,       // in : Distance between distinct 3d points
   double           * pdUChordHeight,   // out: max Udir controlPoint dist from polygon BasePlane dist, NULL to ignore
   double           * pdVChordHeight,   // out: max Vdir controlPoint dist from polygon BasePlane dist, NULL to ignore
   double           * pdUAngleDeg,      // out: max Udir polygon endTangent angle, NULL to ignore
   double           * pdVAngleDeg,      // out: max Vdir polygon endTangent angle, NULL to ignore
   double           * pdUVChordHeight)  // out: max UV controlPoint dist from polygon BasePlane dist, NULL to ignore
{
  // init optional output
  if ( pdUChordHeight  ) { *pdUChordHeight  = 0.0; }
  if ( pdVChordHeight  ) { *pdVChordHeight  = 0.0; }
  if ( pdUAngleDeg     ) { *pdUAngleDeg     = 0.0; }
  if ( pdVAngleDeg     ) { *pdVAngleDeg     = 0.0; }
  if ( pdUVChordHeight ) { *pdUVChordHeight = 0.0; }

  // When given a surface with a sub-domain
  // calculate values directly from surface evaluations.
  if (pSurface)
    {
      // pass the call along
      return( sm_ComputeSurfConstants( pSurface, * pUVDomain, 
                                       pdUChordHeight, 
                                       pdVChordHeight, 
                                       pdUAngleDeg, 
                                       pdVAngleDeg, 
                                       pdUVChordHeight )) ;
    } // end given pSurface check

  // arrive here when BSplineSurface cpSur Net properties are to be evaluated
   
  // locals
  gw_SURFACE *pSur       = (gw_SURFACE *)cpSur ;
  gw_CNET    *pNet       = cpSur->net;
  gw_CPOINT **Pw         = pNet->Pw;
  double      dMaxUAngRad = 0.0;
  double      dMaxVAngRad = 0.0;
  SmPoint3d  sLinePnt, sLastPoint, sCurr, sNext, sTryPoint, sPntOnLine;
  SmVector3d sLineVec, sCurrVec, sCHVec;

  // save time - quit when turning angle exceeds a huge limit
  SmBoolean     bOverLimit   = FALSE;
  // gwc replaced: static double dLimitAngRad = 4/5 * SM_PI ; // quit for any turning angle nearing 180 deg - caused prog_test regression
  static constexpr double dLimitAngRad = 10.0;         // more than 1-1/2 times around.

  long ii, jj;

  // VDir ChordHeight and VDir Angle from control polygon
  {
    // First compute for all U rows
    //   - compute V chordheight and V angle along each row.
    for (jj=0; jj<=pNet->n; jj++)
      {
        // For ChordHeight, get a base segment.
        if ( pdVChordHeight )
          {
            // get a base segment from the first to last control point on this net row
            TO_EUCLID(Pw[jj][0],      sLinePnt);
            TO_EUCLID(Pw[jj][pNet->m],sLastPoint);
            sLineVec = sLastPoint - sLinePnt;

            ULONG lTryIndex = pNet->m-1;
            while (sLineVec.LengthSquared() < SM_EFF_ZERO_SQ)
              {
                // If first and last point coincide try a middle point
                TO_EUCLID(Pw[jj][lTryIndex],sTryPoint);
                sLineVec = sTryPoint - sLinePnt ;

                if (lTryIndex == 0)
                  {
                    // singular point just continue on
                    break;
                  }
                lTryIndex--;
              }
            if (sLineVec.LengthSquared() < SM_EFF_ZERO_SQ) { continue; } // singular point
          } // end need to get sLinePnt and sLineVec

        double     dTotalAngRad = 0.0;
        SmVector3d sLastVec(0,0,0);

        // Hold U constant and loop across V points
        TO_EUCLID(Pw[jj][0],sCurr);
        for (ii=1; ii<=pNet->m; ii++)
          {
            TO_EUCLID(Pw[jj][ii],sNext);

            // V Chord height.
            double dLineParam;
            if (pdVChordHeight)
              {
                SER(smgu_LineClosestPoint(sLinePnt,sLineVec,sNext,dLineParam));
                sPntOnLine   = sLinePnt + (sLineVec * dLineParam);
                sCHVec       = sNext - sPntOnLine;
                double dCHSq = sCHVec.LengthSquared();
                if (dCHSq > *pdVChordHeight) { *pdVChordHeight = dCHSq; }
              }

            // V Angle.
            if (pdVAngleDeg)
              {
                // Check for angle getting really big: sometimes the control net
                // is not suitable for these calculations. [B99]
                if ( dTotalAngRad > dLimitAngRad )
                  {
                    if ( !bOverLimit )
                      {
                        bOverLimit = TRUE;
                        dTotalAngRad *= ( pNet->m / ii );  // rough approximation - we quit before integrating the entire turning angle.
                                                           // this approximates the turning angle integrated over the whole surface from the 
                                                           // current partial integral result.
                        // Quit, unless we're doing chord height as well.
                        if ( pdVChordHeight == NULL )
                          { break; }
                      }
                    continue;
                  }

                sCurrVec     = sNext - sCurr;
                double dDist = sCurrVec.Length();
                //double dTol  = SM_EFF_ZERO ;   // GWC: was SM_EFF_ZERO_SQRT * 100.0 * (1.0 + sNext.GetMaxDimension());

                // JGU: Skip control points within ZoneTol
                if (dDist < rZoneTol3d) 
                  { continue; }                        
                sCurrVec = sCurrVec / dDist;
                if (sLastVec.LengthSquared() > SM_EFF_ZERO_SQ)
                  {
                    double dCosine = sCurrVec.Dot(sLastVec);
                    if (dCosine >  1.0) { dCosine =  1.0; }
                    if (dCosine < -1.0) { dCosine = -1.0; }
                    dTotalAngRad += smos_ArcCosine(dCosine);

                  }
                sLastVec = sCurrVec ;
              }
            sCurr = sNext ;

          } // end for all V points

        if (dTotalAngRad > dMaxVAngRad) { dMaxVAngRad = dTotalAngRad; }

        if ( bOverLimit && pdVChordHeight == NULL )
          { break; }

      } // Compute constants for each U row

  } // end VDir ChordHeight and VDir Angle


  bOverLimit = FALSE;

  // Next compute for all V columns - compute U ch and U ang
  // UDir ChordHeight and UDir Angle from control polygon
  {
    // along each column
    for (jj=0; jj<=pNet->m; jj++)
      {
        // For ChordHeight, get a base segment.
        if (pdUChordHeight)
          {
            // get a base segment from the first to last control point on this net row
            TO_EUCLID(Pw[0][jj],sLinePnt);
            TO_EUCLID(Pw[pNet->n][jj],sLastPoint);
            sLineVec = sLastPoint - sLinePnt;
            ULONG lTryIndex = pNet->n-1;
            while (sLineVec.LengthSquared() < SM_EFF_ZERO_SQ)
              {
                // If first and last point coincide try a middle point
                TO_EUCLID(Pw[lTryIndex][jj],sTryPoint);
                sLineVec = sTryPoint - sLinePnt;
                if (lTryIndex == 0)
                  {
                    // singular point just continue on
                    break;
                  }
                lTryIndex--;
              }
            if (sLineVec.LengthSquared() < SM_EFF_ZERO_SQ) { continue; } // singular point
          }

        double dTotalAngle = 0.0;
        SmVector3d sLastVec(0,0,0);

        TO_EUCLID(Pw[0][jj],sCurr);
        for (ii=1; ii<=pNet->n; ii++)
          {
            TO_EUCLID(Pw[ii][jj],sNext);

            // U Chord height.
            if (pdUChordHeight)
              {
                double dLineParam;
                SER(smgu_LineClosestPoint(sLinePnt,sLineVec,sNext,dLineParam));
                sPntOnLine   = sLinePnt + (sLineVec * dLineParam);
                sCHVec       = sNext - sPntOnLine;
                double dCHSq = sCHVec.LengthSquared();
                if (dCHSq > *pdUChordHeight) { *pdUChordHeight = dCHSq; }
              }

            // U Angle.
            if (pdUAngleDeg)
              {
                // Check for angle getting really big: sometimes the control net
                // is not suitable for these calculations. [B99]
                if ( dTotalAngle > dLimitAngRad )
                {
                    if ( !bOverLimit )
                    {
                        bOverLimit = TRUE;
                        dTotalAngle *= ( pNet->n / ii );  // rough approximation
                        // Quit, unless we're doing chord height as well.
                        if ( pdUChordHeight == NULL )
                          { break; }
                    }
                    continue;
                }

                sCurrVec     = sNext - sCurr;
                //double dTol  = SM_EFF_ZERO ;   // GWC: was SM_EFF_ZERO_SQRT * 100.0 * (1.0 + sNext.GetMaxDimension());
                double dDist = sCurrVec.Length();

                // JGU: Skip control points within ZoneTol.
                if (dDist < rZoneTol3d) 
                  { continue; }                 // gwc: why skip short vectors - what about small surfaces?
                                                // JGU: Short vectors == coincident control points
                sCurrVec = sCurrVec / dDist;
                if (sLastVec.LengthSquared() > SM_EFF_ZERO_SQ)
                  {
                    double dCosAngle = sCurrVec.Dot(sLastVec);
                    if (dCosAngle >  1.0) { dCosAngle =  1.0; }
                    if (dCosAngle < -1.0) { dCosAngle = -1.0; }
                    dTotalAngle += smos_ArcCosine(dCosAngle);
                    // smos_ArcCosine(dCosAngle);
                  }
                sLastVec = sCurrVec;
              }
            sCurr = sNext;
          }
        if ( dTotalAngle > dMaxUAngRad ) { dMaxUAngRad = dTotalAngle; }

        if ( bOverLimit && pdUChordHeight == NULL )
          { break; }

      } // Compute constants for each U row
  } // end UDir ChordHeight and UDir Angle

  // UVDir ChordHeight from sample gw_SURFACE surface evaluations
  if (pdUVChordHeight)
    {
      double us, ue, vs, ve ;
      N_SrfGetParameterBounds(pSur, &us, &ue, &vs, &ve) ;
      SmExtent1d sUDomain(us, ue) ;
      SmExtent1d sVDomain(vs, ve) ;

      // test two diagonals running from [0, 0] to [1, 1]
      //                        and from [0, 1] to [1, 0]
      for (jj=0; jj<2; jj++)
        {
          double dS = jj==0 ? -.25 : +.25 ;
          double dV = (double)jj ;
          double dU = 0.0 ;
          long itr=0;

          // get a base segment from the first to last corner on this net diag row
          N_SrfEvalPt(pSur, sUDomain.Evaluate(0.0), sVDomain.Evaluate(dV), NL_LEFT, NL_LEFT, (NL_POINT *)&sLinePnt) ;
          N_SrfEvalPt(pSur, sUDomain.Evaluate(1.0), sVDomain.Evaluate(1.0-dV), NL_LEFT, NL_LEFT, (NL_POINT *)&sLastPoint) ;

          sLineVec = sLastPoint - sLinePnt;
          for(itr=0;itr<=3&&sLineVec.LengthSquared() < SM_EFF_ZERO_SQ;itr++)
            {
              // If first and last point coincide try a middle point
              dU  = 1.0   - .25 * itr ;
              dV  = (1-jj) +  dS * itr ;
              N_SrfEvalPt(pSur, sUDomain.Evaluate(dU), sVDomain.Evaluate(dV), NL_LEFT, NL_LEFT, (NL_POINT *)&sTryPoint) ;
              sLineVec = sTryPoint - sLinePnt;
            }

          // skip singular curves
          if (itr == 4) { continue; }

          //      double dTotalAngle = 0.0;
          //      SmVector3d sLastVec(0,0,0) ;

          // loop accross diagonal points
          dV = (double)jj ;
          dU = 0.0 ;
          //      N_SrfEvalPt(pSur, sUDomain.Evaluate(dU), sVDomain.Evaluate(dV), NL_LEFT, NL_LEFT, (NL_POINT *)&sCurr) ;

          for (itr=1; itr<=3; itr++)
            {
              dU = 0.0 + .25 * itr ;
              dV =  jj -  dS * itr ;

              N_SrfEvalPt(pSur, sUDomain.Evaluate(dU), sVDomain.Evaluate(dV), NL_LEFT, NL_LEFT, (NL_POINT *)&sNext) ;

              double dLineParam;
              SER(smgu_LineClosestPoint(sLinePnt,sLineVec,sNext,dLineParam));
              sPntOnLine   = sLinePnt + (sLineVec * dLineParam);
              sCHVec       = sNext - sPntOnLine;
              double dCHSq = sCHVec.LengthSquared();
              if ( dCHSq > *pdUVChordHeight ) { *pdUVChordHeight = dCHSq; }

              // not checking angles on diagonal walks
              //      if(pdUVAngleDeg)
              //        {
              //          sCurrVec = sNext - sCurr;
              //          double dTol = SM_EFF_ZERO ;   // GWC: was SM_EFF_ZERO_SQRT * 100.0 * (1.0 + sNext.GetMaxDimension());
              //          double dDist = sCurrVec.Length();
              //          if (dDist < dTol) { break; }
              //          sCurrVec = sCurrVec / dDist;
              //          if (itr > 1)
              //            {
              //              double dCosAngle = sCurrVec.Dot(sLastVec);
              //              if ( dCosAngle >  1.0 ) { dCosAngle =  1.0; }
              //              if ( dCosAngle < -1.0 ) { dCosAngle = -1.0; }
              //              dTotalAngle += smos_ArcCosine(dCosAngle);
              //              // smos_ArcCosine(dCosAngle);
              //            }
              //          sLastVec = sCurrVec;
              //        }
              //      sCurr = sNext;
            } // end iter diagonal point samples

          //      if (dTotalAngle > dMaxUAngRad) { dMaxUAngRad = dTotalAngle; }
        } // end iter both diaganols
    } // end pdUVChordHeight existence check

  // UVDir ChordHeight from control polygon
  {
    // UV ChordHeight
    if (pdUVChordHeight)
      {
        ULONG cnt = smos_Min(pNet->m, pNet->n) ;

        // test two diagonals running from [0, 0]       to [cnt, cnt]
        //                        and from [0, pNet->m] to [cnt, pNet->m-cnt]
        for(jj=0;jj<2;jj++)
          {
            ULONG iii=0, jjj, je ;
            int dj = 0;
            if(jj==0) { jjj = 0 ;
                        je  = cnt ;
                        dj  = 1 ;
                      }
            else      { jjj =  pNet->m ;
                        je  =  pNet->m-cnt ;
                        dj  = -1 ;
                      }

            // get a base segment from the first to last control point on this net diag row
            TO_EUCLID(Pw[iii] [jjj], sLinePnt);
            TO_EUCLID(Pw[cnt][je], sLastPoint);
            sLineVec.x = sLastPoint.x - sLinePnt.x;
            sLineVec.y = sLastPoint.y - sLinePnt.y;
            sLineVec.z = sLastPoint.z - sLinePnt.z;
            ULONG itr ;
            for(itr = 1;
                itr<cnt && sLineVec.LengthSquared() < SM_EFF_ZERO_SQ;
                itr++)
              {
                // If first and last point coincide try a middle point
                TO_EUCLID(Pw[cnt-itr][je-dj*itr],sTryPoint);
                sLineVec.x = sTryPoint.x - sLinePnt.x;
                sLineVec.y = sTryPoint.y - sLinePnt.y;
                sLineVec.z = sTryPoint.z - sLinePnt.z;
              }
            // skip singular diagonal curves
            if(itr == cnt)
              { continue ; }

            //      double     dTotalAngle = 0.0;
            //      SmVector3d sLastVec(0,0,0);

            // loop across diagonal points
            TO_EUCLID(Pw[iii][jjj],sCurr);
            for (itr=1; itr<=cnt; itr++)
              {
               TO_EUCLID(Pw[itr][jjj+dj*itr],sNext);

               double dLineParam;
               SER(smgu_LineClosestPoint(sLinePnt,sLineVec,sNext,dLineParam));
               sPntOnLine   = sLinePnt + (sLineVec * dLineParam);
               sCHVec       = sNext - sPntOnLine;
               double dCHSq = sCHVec.LengthSquared();
               if ( dCHSq > *pdUVChordHeight ) { *pdUVChordHeight = dCHSq; }

               // not checking angles on diagonal walks
               //      if (pdUVAngleDeg)
               //        {
               //          sCurrVec        = sNext - sCurr;
               //          double dDist    = sCurrVec.Length();
               //          double dTol     = SM_EFF_ZERO ;   // GWC: was SM_EFF_ZERO_SQRT * 100.0 * (1.0 + sNext.GetMaxDimension());
               //          if (dDist < dTol) continue;
               //          sCurrVec = sCurrVec / dDist;
               //          if (sLastVec.LengthSquared() > SM_EFF_ZERO_SQ)
               //            {
               //              double dCosAngle = sCurrVec.Dot(sLastVec);
               //              if (dCosAngle > 1.0) dCosAngle = 1.0;
               //              if (dCosAngle < -1.0) dCosAngle = -1.0;
               //              dTotalAngle += smos_ArcCosine(dCosAngle);
               //
               //            }
               //          sLastVec = sCurrVec ;
               //        }
                sCurr = sNext ;
              } // end iter every diagonal control point
            //      if (dTotalAngle > dMaxVAngRad) dMaxVAngRad = dTotalAngle;
          } // end iter diagonal rows
      } // end UVDir ChordHeight test check
  } // end UVDir ChordHeight from control polygon block

  // set output
  if ( pdUAngleDeg     ) { *pdUAngleDeg     = SM_RAD2DEG(dMaxUAngRad) ; }
  if ( pdVAngleDeg     ) { *pdVAngleDeg     = SM_RAD2DEG(dMaxVAngRad) ; }
  if ( pdUChordHeight  ) { *pdUChordHeight  = smos_Sqrt(*pdUChordHeight) ; }
  if ( pdVChordHeight  ) { *pdVChordHeight  = smos_Sqrt(*pdVChordHeight) ; }
  if ( pdUVChordHeight ) { *pdUVChordHeight = smos_Sqrt(*pdUVChordHeight); }

  // all done
  return SM_SUCCESS;

} // end sm_ComputeNetConstants

/*******************************************************************//**
PURPOSE: Compute the approximate chord heights and turning angles
    in the U, V, and diagonal directions using surface evaluations, not control net.

NOTES: Note that the chord height is only relative to an
    individual cross section of the control net.  An estimate of
    the actual chord height can be obtained by adding these two
    chord heights.

    After checking U and V directions a pseudo diagonal direction
    of the surface is checked because this function can underestimate
    chordheight for things like ruled surfaces made from a pair of
    boundary lines when only using U and V cross sections.
***********************************************************************/
SmStatus sm_ComputeSurfConstants
  (const SmSurface  * pSurface,         // in : optional surface expected to be nonNULL for non SmBSplineSurface types
   const SmExtent2d & crUVDomain,       // in : only used when pSurface != NULL
   double           * pdUChordHeight,   // out: max Udir controlPoint dist from polygon BasePlane dist, NULL to ignore
   double           * pdVChordHeight,   // out: max Vdir controlPoint dist from polygon BasePlane dist, NULL to ignore
   double           * pdUAngleDeg,      // out: max Udir polygon endTangent angle, NULL to ignore
   double           * pdVAngleDeg,      // out: max Vdir polygon endTangent angle, NULL to ignore
   double           * pdUVChordHeight)  // out: max UV controlPoint dist from polygon BasePlane dist, NULL to ignore
{
  NER( pSurface );

  // init optional output
  if ( pdUChordHeight  ) { *pdUChordHeight  = 0.0; }
  if ( pdVChordHeight  ) { *pdVChordHeight  = 0.0; }
  if ( pdUAngleDeg     ) { *pdUAngleDeg     = 0.0; }
  if ( pdVAngleDeg     ) { *pdVAngleDeg     = 0.0; }
  if ( pdUVChordHeight ) { *pdUVChordHeight = 0.0; }

  // locals
  double     dMaxUAngRad = 0.0;
  double     dMaxVAngRad = 0.0;
  SmPoint3d  sLinePnt, sLastPoint, sCurr, sNext, sTryPoint, sPntOnLine;
  SmVector3d sLineVec, sCurrVec, sCHVec;

  long ii, jj;

  // VDir ChordHeight and VDir Angle - from sample surface evaluations.
  // First compute for all U rows - compute V ch and V ang along each row.
  // Hold U constant and loop accross V.
  for (jj=0; jj<=4; jj++)
    {
      double dU = jj/4.0;

      // Chord height:
      if (pdVChordHeight)
        {
          SER( pSurface->EvaluatePoint( crUVDomain.Evaluate(dU,0.0), sLinePnt ));
          SER( pSurface->EvaluatePoint( crUVDomain.Evaluate(dU,1.0), sLastPoint ));
          sLineVec = sLastPoint - sLinePnt;
          ULONG lTryIndex = 3;
          while (sLineVec.LengthSquared() < SM_EFF_ZERO_SQ)
            {
              // If first and last point coincide try a middle point
              double dV = lTryIndex / 4.0;
              SER( pSurface->EvaluatePoint( crUVDomain.Evaluate(dU,dV), sTryPoint ));
              sLineVec = sTryPoint - sLinePnt;
              if (lTryIndex == 0)
                {
                  // singular point just continue on
                  break;
                }
              lTryIndex--;
            }
          if (sLineVec.LengthSquared() < SM_EFF_ZERO_SQ)
            { continue; } // singular point
        }

      // Angle:
      double dTotalAngRad = 0.0;
      SmVector3d sLastVec(0,0,0);
      SER( pSurface->EvaluatePoint( crUVDomain.Evaluate(dU,0.0), sCurr ));
      for (ii=1; ii<=4; ii++)
        {
          double dV = ii / 4.0;
          SER( pSurface->EvaluatePoint( crUVDomain.Evaluate(dU,dV), sNext ));
          double dLineParam;

          if (pdVChordHeight)
            {
              SER(smgu_LineClosestPoint(sLinePnt,sLineVec,sNext,dLineParam));
              sPntOnLine   = sLinePnt + (sLineVec * dLineParam);
              sCHVec       = sNext - sPntOnLine;
              double dCHSq = sCHVec.LengthSquared();
              if ( dCHSq > *pdVChordHeight ) { *pdVChordHeight = dCHSq; }
            }

          if (pdVAngleDeg)
            {
              sCurrVec     = sNext - sCurr;
              double dDist = sCurrVec.Length();
              double dTol  = SM_EFF_ZERO ;   // GWC: was SM_EFF_ZERO_SQRT * 100.0 * (1.0 + sNext.GetMaxDimension());
              if ( dDist < dTol ) 
                { break; }   // avoid divide by zero
              sCurrVec = sCurrVec / dDist;
              if (sLastVec.LengthSquared() > SM_EFF_ZERO_SQ)
                {
                  double dCosine = sCurrVec.Dot(sLastVec);
                  if ( dCosine >  1.0 ) { dCosine =  1.0; }
                  if ( dCosine < -1.0 ) { dCosine = -1.0; }
                  dTotalAngRad += smos_ArcCosine(dCosine);
                  // smos_ArcCosine(dCosine);
                }
              sLastVec = sCurrVec;
            }
          sCurr = sNext;
        }
      if (dTotalAngRad > dMaxVAngRad) { dMaxVAngRad = dTotalAngRad; }
    } // Compute constants for each U row

  // UDir ChordHeight and UDir Angle.
  // First compute for all V columns - compute U ch and U ang
  // along each column
  for (jj=0; jj<=4; jj++)
    {
      double dV = jj/4.0;

      // Chord height:
      if (pdUChordHeight)
        {
          SER(pSurface->EvaluatePoint(crUVDomain.Evaluate(0.0,dV),sLinePnt));
          SER(pSurface->EvaluatePoint(crUVDomain.Evaluate(1.0,dV),sLastPoint));
          sLineVec = sLastPoint - sLinePnt;
          ULONG lTryIndex = 3;
          while (sLineVec.LengthSquared() < SM_EFF_ZERO_SQ)
            {
              // If first and last point coincide try a middle point
              double dU = lTryIndex / 4.0;
              SER(pSurface->EvaluatePoint(crUVDomain.Evaluate(dU,dV),sTryPoint));
              sLineVec = sTryPoint - sLinePnt;
              if (lTryIndex == 0)
                {
                  // singular point just continue on
                  break;
                }
              lTryIndex--;
            }
          if (sLineVec.LengthSquared() < SM_EFF_ZERO_SQ) { continue; } // singular point
        }

      // Angle:
      double dTotalAngRad = 0.0;
      SmVector3d sLastVec(0,0,0) ;
      // Hold U constant and loop accross V points
      SER(pSurface->EvaluatePoint(crUVDomain.Evaluate(0.0,dV),sCurr));
      for (ii=1; ii<=4; ii++)
        {
          double dU = ii / 4.0;
          SER(pSurface->EvaluatePoint(crUVDomain.Evaluate(dU,dV),sNext));

          if (pdUChordHeight)
            {
              double dLineParam;
              SER(smgu_LineClosestPoint(sLinePnt,sLineVec,sNext,dLineParam));
              sPntOnLine   = sLinePnt + (sLineVec * dLineParam);
              sCHVec       = sNext - sPntOnLine;
              double dCHSq = sCHVec.LengthSquared();
              if ( dCHSq > *pdUChordHeight ) { *pdUChordHeight = dCHSq; }
            }

          if (pdUAngleDeg)
            {
              sCurrVec = sNext - sCurr;
              double dTol = SM_EFF_ZERO ;   // GWC: was SM_EFF_ZERO_SQRT * 100.0 * (1.0 + sNext.GetMaxDimension());
              double dDist = sCurrVec.Length();
              if ( dDist < dTol ) 
                { break; }  // GWC: why this skipping of short speeds?  Doesn't the angle apply to small surfaces?
              sCurrVec = sCurrVec / dDist;
              if (ii > 1)
                {
                  double dCosine = sCurrVec.Dot(sLastVec);
                  if ( dCosine >  1.0 ) { dCosine =  1.0; }
                  if ( dCosine < -1.0 ) { dCosine = -1.0; }
                  dTotalAngRad += smos_ArcCosine(dCosine);
                  // smos_ArcCosine(dCosine);
                }
              sLastVec = sCurrVec;
            }
          sCurr = sNext;
        }
      if ( dTotalAngRad > dMaxUAngRad ) { dMaxUAngRad = dTotalAngRad; }
    } // Compute constants for each U row

  // Diagonal ChordHeight.
  if ( pdUVChordHeight )
    {
        // test two diagonals running from [0, 0] to [1, 1]
        //                        and from [0, 1] to [1, 0]
      for (jj=0; jj<2; jj++)
        {
          double dS = jj==0 ? -.25 : +.25 ;
          double dV = (double)jj ;
          double dU = 0.0 ;
          long itr=0;

          // get a base segment from the first to last control point on this net diag row
          SER(pSurface->EvaluatePoint(crUVDomain.Evaluate(0.0,dV),sLinePnt));
          SER(pSurface->EvaluatePoint(crUVDomain.Evaluate(1.0,1-dV),sLastPoint));
          sLineVec = sLastPoint - sLinePnt;
          for(itr=0;itr<=3&&sLineVec.LengthSquared() < SM_EFF_ZERO_SQ;itr++)
            {
              // If first and last point coincide try a middle point
              dU  = 1.0    - .25 * itr ;
              dV  = (1-jj) +  dS * itr ;
              SER(pSurface->EvaluatePoint(crUVDomain.Evaluate(dU,dV),sTryPoint));
              sLineVec = sTryPoint - sLinePnt;
            }

          // skip singular curves
          if (itr == 4) { continue; }

          // Not checking angles on diagonal walks.
          // double dTotalAngRad = 0.0;
          // SmVector3d sLastVec(0,0,0);

          // Loop across diagonal.
          dV = (double)jj;
          dU = 0.0 ;
          SER( pSurface->EvaluatePoint( crUVDomain.Evaluate(dU,dV), sCurr ));
          for ( itr=1; itr<=3; itr++ )
            {
              dU = 0.0 + .25 * itr ;
              dV = jj  -  dS * itr ;

              SER(pSurface->EvaluatePoint(crUVDomain.Evaluate(dU,dV),sNext));

              double dLineParam;
              SER(smgu_LineClosestPoint(sLinePnt,sLineVec,sNext,dLineParam));
              sPntOnLine   = sLinePnt + (sLineVec * dLineParam);
              sCHVec       = sNext - sPntOnLine;
              double dCHSq = sCHVec.LengthSquared();
              if ( dCHSq > *pdUVChordHeight ) { *pdUVChordHeight = dCHSq; }

              // Not checking angles on diagonal walks. // GWC: why not?
              //      if(pdUVAngleDeg)
              //        {
              //          sCurrVec = sNext - sCurr;
              //          double dTol = SM_EFF_ZERO ;   // GWC: was SM_EFF_ZERO_SQRT * 100.0 * (1.0 + sNext.GetMaxDimension());
              //          double dDist = sCurrVec.Length();
              //          if ( dDist < dTol ) 
              //             { break; }       // GWC: why this skipping of short speeds?  Doesn't the angle apply to small surfaces?
              //          sCurrVec = sCurrVec / dDist;
              //          if (itr > 1)
              //            {
              //              double dCosAngle = sCurrVec.Dot(sLastVec);
              //              if ( dCosAngle >  1.0 ) { dCosAngle =  1.0; }
              //              if ( dCosAngle < -1.0 ) { dCosAngle = -1.0; }
              //              dTotalAngRad += smos_ArcCosine(dCosAngle);
              //              // smos_ArcCosine(dCosAngle);
              //            }
              //          sLastVec = sCurrVec;
              //        }
              sCurr = sNext;
            } // end iter diagonal point samples

          //      if ( dTotalAngRad > dMaxUAngRad ) { dMaxUAngRad = dTotalAngRad; }
        } // end iter both diaganols
    } // end pSurface && pdUVChordHeight existence check

  // set output
  if ( pdUAngleDeg     ) { *pdUAngleDeg     = SM_RAD2DEG(dMaxUAngRad) ; }
  if ( pdVAngleDeg     ) { *pdVAngleDeg     = SM_RAD2DEG(dMaxVAngRad) ; }
  if ( pdUChordHeight  ) { *pdUChordHeight  = smos_Sqrt(*pdUChordHeight) ; }
  if ( pdVChordHeight  ) { *pdVChordHeight  = smos_Sqrt(*pdVChordHeight) ; }
  if ( pdUVChordHeight ) { *pdUVChordHeight = smos_Sqrt(*pdUVChordHeight); }

  // all done
  return SM_SUCCESS;

} // end sm_ComputeSurfConstants

/*******************************************************************//**
PURPOSE: Copy the data for a nurbs surface.

NOTES: pTo is allocated prior to this call and is same size as
    cpFrom.
***********************************************************************/
SmStatus sm_CopyNurbSurface(const gw_SURFACE *cpFrom, gw_SURFACE *pTo)
{
  sm_InitNurbSurfaceMemory(pTo,cpFrom->net->n,
                              cpFrom->net->m,
                              cpFrom->p,cpFrom->q,
                              cpFrom->knu->m,
                              cpFrom->knv->m);

  // okay to use smos_MemCpy on base (double) and static class (gw_CPOINT) objects.
  SER(smos_MemCpy(pTo->knu->U,
              cpFrom->knu->U,
              sizeof(gw_REAL)*(pTo->knu->m + 1), sizeof(gw_REAL)*(pTo->knu->m + 1)));

  SER(smos_MemCpy(pTo->knv->U,
              cpFrom->knv->U,
              sizeof(gw_REAL)*(pTo->knv->m + 1), sizeof(gw_REAL)*(pTo->knv->m + 1)));

  SER(smos_MemCpy(pTo->net->Pw[0],
              cpFrom->net->Pw[0],
              sizeof(gw_CPOINT) * (pTo->net->n + 1) * (pTo->net->m + 1), sizeof(gw_CPOINT) * (pTo->net->n + 1) * (pTo->net->m + 1)));

  return SM_SUCCESS;
} // end sm_CopyNurbSurface

/*******************************************************************//**
PURPOSE: convenience routine for calling sm_ComputeNurbSurfaceSize(args)

NOTES:
***********************************************************************/
ULONG sm_ComputeNurbSurfaceSize
  (const gw_SURFACE *cpSurface)      // in : surface to be examined
{
  // pass the call along
  return( cpSurface ? sm_ComputeNurbSurfaceSize(cpSurface->net->n,
                                                   cpSurface->net->m,
                                                   cpSurface->knu->m,
                                                   cpSurface->knv->m)
                    : 0 ) ;
} // end sm_ComputeNurbSurfaceSize

/*******************************************************************//**
PURPOSE: Given the highest UV knots indices and the highest
    UV ControlPoint indices, compute the size in bytes
    of the resulting nurb surface.

NOTES:
***********************************************************************/
ULONG sm_ComputeNurbSurfaceSize
  (gw_INDEX lUCPointHighestIndex,  // in : max U ControlPoint index value = U ControlPoint_count - 1
   gw_INDEX lVCPointHighestIndex,  // in : max V ControlPoint index value = V ControlPoint_count - 1
   gw_INDEX lUKnotsHighestIndex,   // in : max U knot index value = U knot_count - 1
   gw_INDEX lVKnotsHighestIndex)   // in : max V knot index value = V knot_count - 1
{
  // This routine computes the size of a single piece of memory to
  // contain the nurb Surface.  The following order is used
  // to map the memory to the Surface structure:
  //    gw_SURFACE
  //    gw_CNET
  //    KNOTVECTORU
  //    KNOTVECTORV
  //    <array of double for knots U>
  //    <array of double for knots V>
  //    <array of pointers to gw_CPOINT*>
  //    <array of double*4 for CPOINTS>
  ULONG lTotalSize =
    (  ALIGN_SIZE(sizeof(gw_SURFACE))
     + ALIGN_SIZE(sizeof(gw_CNET))
     + ALIGN_SIZE(sizeof(gw_KNOTVECTOR))
     + ALIGN_SIZE(sizeof(gw_KNOTVECTOR))
     + ALIGN_SIZE(sizeof(gw_REAL) * (lUKnotsHighestIndex+1))
     + ALIGN_SIZE(sizeof(gw_REAL) * (lVKnotsHighestIndex+1))
     + ALIGN_SIZE(sizeof(gw_CPOINT**) * (lUCPointHighestIndex+1))
     + ALIGN_SIZE(sizeof(gw_CPOINT) * (lUCPointHighestIndex+1)
                                    * (lVCPointHighestIndex+1))) ;

  return lTotalSize;

} // end sm_ComputeNurbSurfaceSize


/*******************************************************************//**
PURPOSE: Given a pointer to a proper sized block of memory and the critical
    surface size information, initialize the memory for the Nurbs Surface.

NOTES:

  Internal Surface pointers are set.
  Internal Surface size values are set.
  ControlPoint and knot values are not touched.

  Pointers to surface memory blocks are stored to mimick
  the data structures built into NLibs.  These pointers point to locations
  within the single contiguous block being built in pSurfaceMemory.

***********************************************************************/
void sm_InitNurbSurfaceMemory
  (gw_SURFACE* pSurfaceMemory,      // in : pointer to surface memory block to init,
                                    //      sized:[sm_ComputeNurbSurfaceSize()]
   gw_INDEX  lUCPointHighestIndex,  // in : max U ControlPoint index value = U ControlPoint_count - 1
   gw_INDEX  lVCPointHighestIndex,  // in : max V ControlPoint index value = V ControlPoint_count - 1
   gw_DEGREE lUDegree,              // in : U dir degree
   gw_DEGREE lVDegree,              // in : V dir degree
   gw_INDEX  lUKnotsHighestIndex,   // in : max U knot index value = U knot_count - 1
   gw_INDEX  lVKnotsHighestIndex)   // in : max V knot index value = V knot_count - 1
{
  gw_CNET *pNet            = pSurfaceMemory->net = (gw_CNET*)(((char*)pSurfaceMemory)  + ALIGN_SIZE(sizeof(gw_SURFACE)));
  gw_KNOTVECTOR *pUKnotVec = pSurfaceMemory->knu = (gw_KNOTVECTOR*)(((char*)pNet)      + ALIGN_SIZE(sizeof(gw_CNET)));
  gw_KNOTVECTOR *pVKnotVec = pSurfaceMemory->knv = (gw_KNOTVECTOR*)(((char*)pUKnotVec) + ALIGN_SIZE(sizeof(gw_KNOTVECTOR)));
  pUKnotVec->U                                   = (gw_REAL*)(((char*)pVKnotVec)       + ALIGN_SIZE(sizeof(gw_KNOTVECTOR)));
  pVKnotVec->U                                   = (gw_REAL*)(((char*)pUKnotVec->U)    + ALIGN_SIZE(sizeof(gw_REAL) * (lUKnotsHighestIndex+1)));
  gw_CPOINT **pPw          = pNet->Pw            = (gw_CPOINT**)(((char*)pVKnotVec->U) + ALIGN_SIZE(sizeof(gw_REAL) * (lVKnotsHighestIndex+1)));
  gw_CPOINT *pCP                                 = (gw_CPOINT*)(((char*)pPw)           + ALIGN_SIZE(sizeof(gw_CPOINT**) * (lUCPointHighestIndex+1)));

  pSurfaceMemory->p = lUDegree;
  pSurfaceMemory->q = lVDegree;
  pUKnotVec->m      = lUKnotsHighestIndex;
  pVKnotVec->m      = lVKnotsHighestIndex;
  pNet->n           = lUCPointHighestIndex;
  pNet->m           = lVCPointHighestIndex;

  // Initialize pointers to gw_CPOINT arrays.
  gw_INDEX k, l = 0;
  for( k=0; k<=pNet->n; k++ )
    {
      pNet->Pw[k] = &pCP[l];
      l           = l+pNet->m+1;
    }

  return;

} // end sm_InitNurbSurfaceMemory

  /*******************************************************************//**
PURPOSE: Given useful surface information, allocate the space required
    for a set of nurb surfaces as a single big block.  Return a pointer
    to the block and set the output array of surfaces to point to each
    individual surface.

NOTES: Internal counts, degrees and pointers of the gw_SURFACE
    are initialized for each surface.
***********************************************************************/
char *sm_AllocateBlockOfNurbSurfaces
  (gw_INDEX lNumberOfSurfaces,
   gw_INDEX  lUCPointHighestIndex,
   gw_INDEX  lVCPointHighestIndex,
   gw_DEGREE lUDegree,
   gw_DEGREE lVDegree,
   gw_INDEX  lUKnotsHighestIndex,
   gw_INDEX  lVKnotsHighestIndex,
   SmTArray<void*> & rSurfaces)
{
    ULONG lTotalSize = sm_ComputeNurbSurfaceSize(lUCPointHighestIndex,
        lVCPointHighestIndex, lUKnotsHighestIndex, lVKnotsHighestIndex);

    // okay to use smos_Calloc on static class (gw_CURVE, gw_CPOLYGON, gw_KNOTVECTOR,.. ) objects.
    char* pMemBlock = (char*) smos_Calloc(lNumberOfSurfaces*lTotalSize, 1);
    NERN(pMemBlock);

    // Now load data from input surfaces
    rSurfaces.SetSize(lNumberOfSurfaces);
    long ii;
    for ( ii=0; ii<lNumberOfSurfaces; ii++ )
      {
        gw_SURFACE *pNewSur = (gw_SURFACE*)&pMemBlock[ii*lTotalSize];
        rSurfaces[ii] = SM_REINTERPRET_CAST(SmObject*,pNewSur);
        sm_InitNurbSurfaceMemory(pNewSur,lUCPointHighestIndex,
            lVCPointHighestIndex,lUDegree,lVDegree,lUKnotsHighestIndex,
            lVKnotsHighestIndex);
      }

    return pMemBlock;

} // end sm_AllocateBlockOfNurbSurfaces

/*******************************************************************//**
PURPOSE: Given critical surface information allocate and init the space
    required for a single nurb surface as a single block of memory.

NOTES:
***********************************************************************/
gw_SURFACE * sm_AllocateNurbSurface
  (gw_INDEX  lUCPointHighestIndex,  // in : max U ControlPoint index value = U ControlPoint_count - 1
   gw_INDEX  lVCPointHighestIndex,  // in : max V ControlPoint index value = V ControlPoint_count - 1
   gw_DEGREE lUDegree,              // in : U dir degree
   gw_DEGREE lVDegree,              // in : V dir degree
   gw_INDEX  lUKnotsHighestIndex,   // in : max U knot index value = U knot_count - 1
   gw_INDEX  lVKnotsHighestIndex)   // in : max V knot index value = V knot_count - 1
{
  // get block size for surface
  ULONG lTotalSize = sm_ComputeNurbSurfaceSize(lUCPointHighestIndex,
                                               lVCPointHighestIndex,
                                               lUKnotsHighestIndex,
                                               lVKnotsHighestIndex);

  // allocate surface memory block
  // okay to use smos_Calloc on static class (gw_CURVE, gw_CPOLYGON, gw_KNOTVECTOR,.. ) objects.
  gw_SURFACE *pNewSur = (gw_SURFACE*) smos_Calloc(lTotalSize, 1);
  NERN(pNewSur);

  // init surface memory block pointers and size params - ControlPoint and knot values are not touched
  sm_InitNurbSurfaceMemory(pNewSur,
                           lUCPointHighestIndex,
                           lVCPointHighestIndex,
                           lUDegree, lVDegree,
                           lUKnotsHighestIndex,
                           lVKnotsHighestIndex);
  // all done
  return pNewSur;

} // end sm_AllocateNurbSurface

/*******************************************************************//**
PURPOSE: Allocate a nurb surface and copy the data from the source
    surface.  The returned new surface is a single block of memory.
    The source could be either a single block or one of the
    segmented things we get from NLib.

NOTES: returns NULL if input target surface has no polygon net,
   (the pointer, cpSrcSur->net == NULL)
***********************************************************************/
gw_SURFACE * sm_AllocateAndCopyNurbSurface
  (const gw_SURFACE * cpSrcSur)    // in : target surface to copy
{
  // check input: has to have a net
  gw_CNET *pNet = cpSrcSur ? cpSrcSur->net : NULL;
  if (!pNet)
    { return (NULL); }

  // surface locals
  gw_KNOTVECTOR *pUKnotVec = cpSrcSur->knu;
  gw_KNOTVECTOR *pVKnotVec = cpSrcSur->knv;

  // allocate single block Surface with internal pointers and size params set
  gw_SURFACE *pNewSur = sm_AllocateNurbSurface(pNet->n,
                                               pNet->m,
                                               cpSrcSur->p,
                                               cpSrcSur->q,
                                               pUKnotVec->m,
                                               pVKnotVec->m);

  // copy knot values from target surface to new surface.
  // okay to use smos_MemCpy on base (double) objects.
  SE(smos_MemCpy(pNewSur->knu->U,cpSrcSur->knu->U,sizeof(gw_REAL) * (pNewSur->knu->m + 1), sizeof(gw_REAL) * (pNewSur->knu->m + 1)));
  SE(smos_MemCpy(pNewSur->knv->U,cpSrcSur->knv->U,sizeof(gw_REAL) * (pNewSur->knv->m + 1),sizeof(gw_REAL) * (pNewSur->knv->m + 1)));

  // copy control point values from target surface to new surface one row at a time
  long ii;
  for (ii=0; ii<=pNet->n; ii++)
    {
      // okay to use smos_MemCpy on static class (gw_CPOINT) objects.
      SE(smos_MemCpy(pNewSur->net->Pw[ii],
                  cpSrcSur->net->Pw[ii],
                  sizeof(gw_CPOINT) * (pNewSur->net->m + 1), sizeof(gw_CPOINT) * (pNewSur->net->m + 1)) );
    }

  // all done
  return pNewSur;

} // end sm_AllocateAndCopyNurbSurface


/* ---------------------------------------------------------------------


   DESCRIPTION:

     Given a  surface object, this  routine  maps the  control net to
     Euclidean space. Memory for control points is allocated locally,
     however, the control  net  data  type  is  declared  (allocated)
     in the calling routine. A typical calling example is:

       gw_INDEX    ku, lu, kv, lv;
       gw_SURFACE  sur;
       ENET     ntl;
       NL_STACKS   S;
       ...
       (define surface);
       ...
       N_SrfGetENet(&sur,ku,lu,kv,lv,&ntl,&S);

     Since  the  declarations 'gw_SURFACE sur' and 'ENET ntl' define the
     data types and allocate memory, only the pointers are passed in.


   ACCESS:

     sur         , input  ,  NURBS surface
     ku,lu,kv,lv , input  ,  Start and end  indexes in uv-directions.
                             Only the  control points Pw[ku][kv],...,
                             Pw[lu][lv]  are  mapped.  The  Euclidean
                             points   are   stored   in  P[0][0],...,
                             P[lu-ku][lv-kv].
     ntl         , output ,  Point net
     S           , input  ,  ntl's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NERR


   EXTERNAL REFERENCES:

     N_AllocPt2dArray - Allocate memory for 2-D point array
     N_ErrSet - Set error flag
     N_CPtToPtEuclid - Map point to Euclidean space
     N_SrfGetCPts - Get control net info

   ------------------------------------------------------------------ */

gw_FLAG sm_SurfaceGetENet( gw_SURFACE *sur, gw_INDEX ku, gw_INDEX lu, gw_INDEX kv, gw_INDEX lv, NL_ENET *ntl)
{
  gw_INDEX   i, j, n, m;
  gw_CPOINT  **Pw, *Pw2;
  NL_POINT   **P, *P2;

  /* Get local notation */

  n = sur->net->n;
  m = sur->net->m;
  Pw = sur->net->Pw;
//  N_SrfGetCPts(sur,&n,&m,&Pw);

  /* Check indexes */
  if( ku GT lu  OR  ku LT 0  OR  lu GT n )
  {
    N_ErrSet(NL_IND_ERR,rname);
    return(1);
  }

  if( kv GT lv  OR  kv LT 0  OR  lv GT m )
  {
    N_ErrSet(NL_IND_ERR,rname);
    return(1);
  }

  /* Map control points */
  P = ntl->P;
  if( P EQ NULL )  return(1);

#if 0
  for( i=ku; i<=lu; i++ )
  {
    Pw2 = Pw[i];
    P2 = P[i-ku];
    for( j=kv; j<=lv; j++ )
    {
      TO_EUCLID(Pw2[j],P2[j-kv]);
    }
  }
#endif

  for( i=ku; i<=lu; i++ )
  {
    Pw2 = &Pw[i][kv];
    P2 = P[i-ku];
    for( j=kv; j<=lv; j++ )
    {
      TO_EUCLID(Pw2[0],P2[0]);
      Pw2++; P2++;
    }
  }

  /* Build point net structure */
  ntl->n = lu-ku;
  ntl->m = lv-kv;

  /* Exit */
  return(0);

} // end sm_SurfaceGetENet


/* ---------------------------------------------------------------------


   DESCRIPTION:

     This  tools routine decomposes a  NURBS  surface  into its Bezier
     constituents  without  using  knot   refinement.  Each  piece  is
     represented as a NURBS surface even though the patches are Bezier
     surfaces.

     MEMORY TO STORE THE OUTPUT SURFACES USED TO BE ALLOCATED INSIDE
     THE ROUTINE. The routine has been modified to use an array of
     previously allocated structures. A typical calling example is:

       gw_SURFACE surP, ***surQ;
       gw_INDEX   i, j, k, l;
       gw_CPOINT  **Pw;
       NL_STACKS  SQ;
       ...
       (define surP);
       ...
       N_SrfDecomposeToBez(&surP,&surQ,&k,&l,&SQ);
       ...
       Pw = surQ[i][j]->net->Pw;
       ...

     surQ[i][j],  0<=i<=k,  0<=j<=l, is  a  pointer  to  the  (i,j)-th
     surface.


   ACCESS:

     surP  , input  ,  NURBS surface to be decomposed
     surQ  , output ,  2-D array of Bezier surfaces
     kk,ll , output ,  Highest indexes in surQ
     SQ    , input  ,  surQ's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NERR


   EXTERNAL REFERENCES:

     N_InitNurbs - Start NURBS
     N_BasisGetSpanCount - Find number of non-zero knot spans
     N_SrfGetCPtsDegreesAndKnots - Get surface components
     N_AllocReal1dArray - Allocate memory for real array
     N_AllocCPt2dArray - Allocate memory for a 2-D array of control points
     N_Alloc2dArraySrfPtrsParameters - Allocate memory for a 2-D array of surfaces
     N_CopyCPt - Initialize control point
     N_Combine2CPts - Combination of control points
     N_EndNurbs - End nurbs

   ------------------------------------------------------------------ */

/* Note - modified to use an array of previously existing surfaces */

gw_FLAG  sm_DecomposeSrf(const gw_SURFACE *surP, SmTArray<gw_SURFACE *> & rBeziers )

{

  gw_FLAG        error = NL_NO;

  gw_INDEX       i, j, k, l, n, m, r, s, ru, su, rv, sv, nsu, nsv,
                 mlu, mlv, isu, ieu, isv, iev, iq, jq, save;

  gw_DEGREE      p, q;

  gw_REAL        *UP, *VP, *UQ, *VQ, *uals, *vals, *omus, *omvs, num = 0.0;

  gw_KNOTVECTOR  *knu, *knv;

  gw_CPOINT      **Pw, **Qw, **NQw, **Bw, **NBw, **tmp;

  gw_SURFACE     ***surA;

  NL_STACKS      SL;


  /* Start NURBS environment */
  N_InitNurbs(&SL);

  /* Get local notation */
  knu = surP->knu;
  knv = surP->knv;

  N_SrfGetCPtsDegreesAndKnots((gw_SURFACE *)surP,&n,&m,&Pw,&p,&q,&r,&s,&UP,&VP);

  /* Allocate memory */
  N_BasisGetSpanCount(knu,p,&nsu);
  N_BasisGetSpanCount(knv,q,&nsv);

  if ( nsu < 1 || nsv < 1 )
    { NL_QUIT; }

//  surA = N_Alloc2dArraySrfPtrsParameters(p,q,p,q,2*p+1,2*q+1,nsu-1,nsv-1,SQ);
  surA = (gw_SURFACE***)N_AllocArraySrfPtrs(nsu-1,&SL);
  if( surA EQ NULL )  NL_QUIT;
  for (i=0; i<nsu; i++) {
      surA[i] = (gw_SURFACE**)&rBeziers[i*nsv];
  }

  NQw = N_AllocCPt2dArray(p,m,&SL);
  if( NQw EQ NULL )  NL_QUIT;

  Bw = N_AllocCPt2dArray(p,m,&SL);
  if( Bw EQ NULL )  NL_QUIT;

  NBw = N_AllocCPt2dArray(p,m,&SL);
  if( NBw EQ NULL )  NL_QUIT;

  uals = N_AllocReal1dArray(p,&SL);
  if( uals EQ NULL )  NL_QUIT;

  omus = N_AllocReal1dArray(p,&SL);
  if( omus EQ NULL )  NL_QUIT;

  vals = N_AllocReal1dArray(q,&SL);
  if( vals EQ NULL )  NL_QUIT;

  omvs = N_AllocReal1dArray(q,&SL);
  if( omvs EQ NULL )  NL_QUIT;

  /* Initialize for u-directional decomposition */
  isu = p;  ieu = p+1;  iq = -1;

  for( i=0; i<=p; i++ )
  {
    for( l=0; l<=m; l++ )
    {
      N_CopyCPt(Pw[i][l],&Bw[i][l]);
    }
  }


  /* Decompose in u-direction into Bezier strips */


  while( ieu LT r )
  {
    iq = iq+1;

    /* Get knot multiplicity */

    i=ieu;
    while( ieu LT r  AND  UP[ieu] EQ UP[ieu+1] )  ieu++;
    mlu = ieu-i+1;
    ru  = p-mlu;
    if (ru < 0.0) { ru = 0; } // GAC added fix for bug

    /* Insert the knot */

    if( mlu LT p )
    {
      num = UP[ieu]-UP[isu];
      for( i=p; i>mlu; i-- )
      {
        uals[i-mlu-1] = num/(UP[isu+i]-UP[isu]);
        omus[i-mlu-1] = 1.0-uals[i-mlu-1];
      }

      for( i=1; i<=ru; i++ )
      {
        su   = mlu+i;
        save = ru-i;
        for( j=p; j>=su; j-- )
        {
          for( l=0; l<=m; l++ )
          {
            N_Combine2CPts(uals[j-su],Bw[j][l],omus[j-su],Bw[j-1][l],&Bw[j][l]);
          }
        }
        if( ieu LT r )
        {
          for( l=0; l<=m; l++ )
          {
            N_CopyCPt(Bw[p][l],&NBw[save][l]);
          }
        }
      }
    }

    /* Bezier strip completed. Initialize for */
    /* v-directional decomposition            */

    // This avoids a crash when knot multiplicity is greater than degree [B262]
    if( iq >= nsu ) 
        NL_QUIT;


    isv = q;  iev = q+1;  jq = -1;

    Qw = surA[iq][0]->net->Pw;

    for( j=0; j<=q; j++ )
    {
      for( k=0; k<=p; k++ )
      {
        N_CopyCPt(Bw[k][j],&Qw[k][j]);
      }
    }

    /* Decompose in v-direction into Bezier patches */

    while( iev LT s )
    {
      jq = jq+1;
      Qw = surA[iq][jq]->net->Pw;
      UQ = surA[iq][jq]->knu->U;
      VQ = surA[iq][jq]->knv->U;
      if( jq LT nsv-1 )
      {
        NQw = surA[iq][jq+1]->net->Pw;
      }

      /* Get knot multiplicity */

      i=iev;
      while( iev LT s  AND  VP[iev] EQ VP[iev+1] )  iev++;
      mlv = iev-i+1;
      rv  = q-mlv;
      if (rv < 0.0) { rv = 0; } // GAC added fix for bug

      /* Insert the knot */

      if( mlv LT q )
      {
        num = VP[iev]-VP[isv];
        for( i=q; i>mlv; i-- )
        {
          vals[i-mlv-1] = num/(VP[isv+i]-VP[isv]);
          omvs[i-mlv-1] = 1.0-vals[i-mlv-1];
        }

        for( i=1; i<=rv; i++ )
        {
          sv   = mlv+i;
          save = rv-i;
          for( j=q; j>=sv; j-- )
          {
            for( k=0; k<=p; k++ )
            {
              N_Combine2CPts(vals[j-sv],Qw[k][j],omvs[j-sv],Qw[k][j-1],&Qw[k][j]);
            }
          }
          if( iev LT s )
          {
            for( k=0; k<=p; k++ )
            {
              N_CopyCPt(Qw[k][q],&NQw[k][save]);
            }
          }
        }
      }

      /* Get knot vectors */

      for( i=0; i<=p; i++ )
      {
        UQ[i] = UP[isu];  UQ[i+p+1] = UP[ieu];
      }

      for( j=0; j<=q; j++ )
      {
        VQ[j] = VP[isv];  VQ[j+q+1] = VP[iev];
      }

      /* Patch completed - prepare for next Bezier patch */

      if( iev LT s )
      {
        for( i=rv; i<=q; i++ )
        {
          for( k=0; k<=p; k++ )
          {
            N_CopyCPt(Bw[k][iev-q+i],&NQw[k][i]);
          }
        }
      }
      isv = iev;  iev = iev+1;
    } /* End while for Bezier patches */

    /* Bezier strip decomposed - prepare for next strip */

    if( ieu LT r )
    {
      for( i=ru; i<=p; i++ )
      {
        for( l=0; l<=m; l++ )
        {
          N_CopyCPt(Pw[ieu-p+i][l],&NBw[i][l]);
        }
      }
    }
    isu = ieu;  ieu = ieu+1;
    tmp = Bw;   Bw  = NBw;    NBw = tmp;
  } /* End while for Bezier strips */

  /* End NURBS and Exit */


  EXIT:

  N_EndNurbs(&SL);

  return(error);

} // end sm_DecomposeSrf

/* ---------------------------------------------------------------------

   DESCRIPTION:

     This  tools  routine  splits a NURBS  surface into two  surfaces
     either in u- or in v-direction. The split must be at an interior
     parameter value. If the output surface is initialized  to  NULL,
     memory to store new  control  points and  knots is  allocated. A
     typical calling example is:

       gw_SURFACE    sur, surL, surR;
       gw_PARAMETER  t;
       NL_STACKS     SG;
       ...
       (define sur, get t);
       ...
       N_SrfSplit(&sur,t,UDIR,&surL,&surR,&SG);



   ACCESS:

     sur  , input  ,  NURBS surface to be split
     t    , input  ,  Parameter where surface is to be split
     dir  , input  ,  Flag:
                        UDIR: Split in u-direction
                        VDIR: Split in v-direction
     surL , output ,  Left half of surface defined  over [U[0],t]  or
                      [V[0],t]
     surR , output ,  Right half of surface defined over [t,U[r]]  or
                      [t,V[s]]

   RETURN CODES:

     0 : No error
     1 : Error saved in NERR


   EXTERNAL REFERENCES:

   ------------------------------------------------------------------ */
gw_FLAG  sm_SplitSrf
 (gw_SURFACE      * sur,          // in : tgt surface
  gw_PARAMETER      dSplitParam,  // in : split param
  gw_FLAG           dir,          // in : SplitDir: NL_UDIR=Split KnotVectorU, use - SM_SURFPARAM_TO_NLDIR(eSurfParam)
                                  //                NL_VDir=Split KnotVectorV        to convert SmSurfParamType to NL_DIR types
  gw_SURFACE     *& surL,         // out: Split surface result, Ivl=[MinParam, TgtParam]
  gw_SURFACE     *& surR)         // out: Split surface result, Ivl=[TgtParam, MaxParam]
{
    gw_FLAG        error = NL_NO;
    gw_INDEX       n, m, r, s, spu, mlu, spv, mlv;
    gw_INDEX       nl, ml, nr, mr, rl, sl, rr, sr;
    gw_DEGREE      p, q;
    gw_REAL        *U, *V;
    gw_KNOTVECTOR  *knu, *knv;
    gw_CPOINT      **Pw;

    if ((surL == NULL) || surR == NULL)
    {
        knu = sur->knu;
        knv = sur->knv;

        N_SrfGetCPtsDegreesAndKnots(sur, &n, &m, &Pw, &p, &q, &r, &s, &U, &V);

        /* Check parameters and compute highest indexes */

        switch (dir)
        {
            case NL_UDIR :

                error = N_KnotVectorIsEndParam(knu, dSplitParam, rname);
                if (error EQ NL_YES)
                  { NL_OUT; }

                error = N_BasisFindSpanAndMult(knu, p, dSplitParam, NL_LEFT, &spu, &mlu);
                if (error EQ NL_YES)
                  { NL_OUT; }
                nl = spu - mlu;
                ml = m;
                rl = nl + p + 1;
                sl = s;
                nr = n + p - spu;
                mr = m;
                rr = nr + p + 1;
                sr = s;
                break;

            case NL_VDIR :

                error = N_KnotVectorIsEndParam(knv, dSplitParam, rname);
                if (error EQ NL_YES)
                  { NL_OUT; }
                error = N_BasisFindSpanAndMult(knv, q, dSplitParam, NL_LEFT, &spv, &mlv);
                if (error EQ NL_YES)
                  { NL_OUT; }
                nl = n;
                ml = spv - mlv;
                rl = r;
                sl = ml + q + 1;
                nr = n;
                mr = m + q - spv;
                rr = r;
                sr = mr + q + 1;
                break;

            default :
                error = NL_YES;
                NL_OUT;
        }


        /* See if memory is needed */
        if (surL == NULL)
        {
            surL = sm_AllocateNurbSurface(nl, ml, p, q, rl, sl);
        }

        if (surR == NULL)
        {
            surR = sm_AllocateNurbSurface(nr, mr, p, q, rr, sr);
        }
    }

    error = N_SrfSplit(sur,         // in : TgtSurface
                       dSplitParam, // in : Tgt SplitParam
                       dir,         // in : SplitDir: NL_UDIR=Split KnotVectorU, use - SM_SURFPARAM_TO_NLDIR(eSurfParam)
                                    //                NL_VDir=Split KnotVectorV        to convert SmSurfParamType to NL_DIR types
                       surL,        // out: Split surface result, Ivl=[MinParam, TgtParam]
                       surR,        // out: Split surface result, Ivl=[TgtParam, MaxParam]
                       NULL) ;      // in : stack for new object memory

EXIT:
    return (error);

} // end sm_SplitSrf

/*******************************************************************//**
PURPOSE: return true when gw_Surface's bounding box's largest side is less than tol.

NOTES: tolerance set to Max(dScaledZero,d3DTol)
***********************************************************************/   
SmBoolean sm_IsNSrfDegenerate
 (const gw_SURFACE *pSur,         // in : target representation to check
  double            d3DTol        // in : min distance between distinct points, default:[SM_EFF_ZERO]
 )                            
{
  // For speed - build bounding box directly from NLib representation
  NL_MINMAXBOX sNLBox ;
  N_SrfGetBBox((gw_SURFACE *)pSur, &sNLBox) ;
  SmExtent3d sBox(sNLBox.xl, sNLBox.yb, sNLBox.zn,
                  sNLBox.xr, sNLBox.yt, sNLBox.zf) ;
  SM_ASSERT(!sBox.HasNegativeVolume()) ;
 
  // select a tolerance - scale it to the 1st control point location
  double dScaledZero = 1000.0 * SM_EFF_ZERO * (1.0 + sBox.GetMaxDimension());
  double dTol        = smos_Max(d3DTol, dScaledZero);

 // test the bbox for point sized
  SmBoolean bDegenerate = sBox.IsPointSized(dTol) ;

  // all done
  return(bDegenerate) ;

} // end sm_IsNSrfDegenerate


/*******************************************************************//**
PURPOSE: Return true when any of the polyNet col or row
  vertices has a repeated pair of end ControlPoints.  

NOTES: Repeated control points are allowed on singular 
  boundary curves and are not reported as a duplicate pair.

***********************************************************************/
SmBoolean sm_HasRepeatedEndControlPoints
  (double dTol,                      // in : min distance between distinct points, default:[SM_ZONE_TOL_3D=1.0e-5]
                                     //      when less than or equal to 0.0 reset to SM_ZONE_TOL_3D
   const gw_SURFACE *pSur,           // in : target representation
   SmBSplineSurface *pSurface)       // in : only used for debug, default:[NULL]
{
  // no work - no BSplineSurface representation
  if(pSur == NULL)
    { return FALSE ; }

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  if(bDebugMe)
    {
      Dump_NSrf(pSur, false) ;
    }
#endif // SM_DEBUG_CODE


  // locals
  ULONG ii, jj, jMax = 0, kk, jRowLength = 0 ;
  NL_INDEX i0 = 0, i1 = 0, i2 = 0, id = 0 ;
  NL_INDEX j0 = 0, j1 = 0, j2 = 0, jd = 0 ;
  NL_INDEX iN = pSur->net->n ; // max 1st index in PW
  NL_INDEX iM = pSur->net->m ; // max 2nd indes in PW

  // pick tolerance
  double dTolerance = dTol > 0.0 ? dTol : SM_ZONE_TOL_3D ;

  // rules: 1. singularity: a boundary edge of the control             
  //           polyhedron may be degenerate.                          
  //        2. Any nonDegenerate boundary edge or any internal        
  //           row or col SubCurve of the control polyhedral that     
  //           has its end control points repeated will be          
  //           reported.                                              

  // for every edge of the surface polyhedron
  for(ii=0;ii<4;ii++)
    { // +--------+-----------------+------+------------------+--------------------+
      // |   ii   |  walk edge      | jMax | i0   i1   i2  id |  j1   j1   j2  jd  |
      // +--------+-----------------+------+------------------+--------------------+
      // | ii = 0 | walk i, j = 0   |  iN  |  0    0    0   1 |   0    1    2   0  |
      // | ii = 1 | walk i, j = iM  |  iN  |  0    0    0   1 |  iM  iM-1 iM-2  0  |
      // | ii = 2 | walk j, i = 0   |  iM  |  0    1    2   0 |   0    0    0   1  |
      // | ii = 3 | walk j, i = iN  |  iM  |  iN iN-1 iN-2  0 |   0    0    0   1  |
      // +--------+-----------------+------+------------------+--------------------+
      switch(ii)
        {
          // set iterators and samplers to walk the various 4 edges of the polyhedron
          case 0 : { jMax = iN ;   jRowLength = iM ;
                     id = 1 ;      jd = 0 ;
                     i0 = 0 ;      j0 = 0 ;
                     i1 = 0 ;      j1 = 1 ;
                     i2 = 0 ;      j2 = 2 ;
                   } break ;
          case 1 : { jMax = iN ;   jRowLength = iM ;
                     id = 1 ;      jd = 0 ;
                     i0 = 0 ;      j0 = iM ;
                     i1 = 0 ;      j1 = iM-1 ;
                     i2 = 0 ;      j2 = iM-2 ;
                   } break ;
          case 2 : { jMax = iM ;   jRowLength = iN ;
                     id = 0 ;      jd = 1 ;
                     i0 = 0 ;      j0 = 0 ;
                     i1 = 1 ;      j1 = 0 ;
                     i2 = 2 ;      j2 = 0 ;
                   } break ;
          case 3 : { jMax = iM ;   jRowLength = iN ;
                     id = 0 ;      jd = 1 ;
                     i0 = iN ;     j0 = 0 ;
                     i1 = iN-1 ;   j1 = 0 ;
                     i2 = iN-2 ;   j2 = 0 ;
                   } break ;
        } // end switch to set up iterators and samplers

      // skip walking sequence of net curves when curves have less than 3 control points
      if(jRowLength < 2)
        { continue ; }

      // walk every net curve that start/ends on this boundary - increment all the indices
      for(jj=0; jj<=jMax; jj++, 
                          i0+=id, i1+=id, i2+=id,
                          j0+=jd, j1+=jd, j2+=jd)
        {
          // ignore repeated control points on singular boundaries - they are supposed to be repeated
          SmBoolean bDegenerate = FALSE ;
          if(jj == 0 || jj == jMax)
            {
              // check net curve for degeneracy
              SmExtent3d sBBox ;

              // to walk the boundary curve in question - use the same indices trick as before
              //      +---------------+--------------------+
              //      |ii == (0 || 1) |  ki = jj   kid = 0 |
              //      |               |  kj = 0    kjd = 1 |
              //      +---------------+--------------------+
              //      |ii == (2 || 3) |  ki = 0    kid = 1 |
              //      |               |  kj = jj   kjd = 0 |
              //      +---------------+--------------------+

              // set up indices to walk the boundary curve in question
              NL_INDEX ki, kid, kj, kjd ;
              if(ii == 0 || ii == 1 ) { ki = jj ; kid = 0 ;
                                        kj = 0 ;  kjd = 1 ;
                                      }
              else                    { ki = 0  ; kid = 1 ;
                                        kj = jj ; kjd = 0 ;
                                      }
              // for every control point in this surface net curve
              for(kk=0;kk<=jRowLength;kk++,
                                      ki+=kid,
                                      kj+=kjd)
                {
                  // ensure points in euclidean space
                  SmPoint3d sPt ; 
                  TO_EUCLID(pSur->net->Pw[ki][kj], sPt) ;

                  // add points to the bounding box
                  sBBox.AddPoint3d(sPt) ;

                  // most bounding boxes will be nonDegenerate - make one stab at being quick
                  if( kk==1 )
                    {
                      // check size of box
                      // dScaledZero = SM_EFF_ZERO * (1.0 + sBBox.GetMaxDimension()) ;
                      bDegenerate = sBBox.IsPointSized(dTolerance) ;

                      if(bDegenerate == FALSE)
                        {
                          // don't need to look any further
                          break ; 
                        } 
                    } // end quicky check to skip obviously nonDegenerate curves
                } // end bounding box size

              // make sure the bbox hasn't grown since the 1st check on its size
              if(bDegenerate)
                {
                  // dScaledZero = SM_EFF_ZERO * (1.0 + sBBox.GetMaxDimension()) ;
                  bDegenerate = sBBox.IsPointSized(dTolerance) ;
                }
             
            } // end is this a boundary netCurve check and setting bDegenerate

          // skip singular boundary edges
          if(bDegenerate)
            { continue ; }

          // check ControlPoint pair coincidence
          double dCPtDist ;

          // scale tolerance
          //double dScaledZero = SM_EFF_ZERO * (1.0 + ((SmVector3d *)(&pSur->net->Pw[i0][j0]))->GetMaxDimension()) ; 

          // get end CPt pair distance (works for homogeneous and euclidean points)
          N_DistCptCpt( pSur->net->Pw[i0][j0], pSur->net->Pw[i1][j1],  &dCPtDist) ;

          // remember if it's a repeat
          SmBoolean bRepeat = SM_IS_ZERO_TO_TOL(dCPtDist, dTolerance) ;

          // when a repeated control point pair was found
          if(bRepeat)
            {
#ifdef SM_DEBUG_CODE
              // draw before
              if(bDebugMe)
                {
                  if(pSurface) pSurface->Dump() ;
                  else         Dump_NSrf(pSur, false) ;

                  SmFace *pFace = pSurface ? (SmFace *)pSurface->GetFace() : NULL ;
                  SmBrep *pBrep = pFace ? pFace->GetBrep() : NULL ;

                  smgfx_Erase() ;
                  smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
                  smgfx_SetLook(1,2, 0,1,1) ; if(pSurface) pSurface->DrawUV(8,8,FALSE,NULL,TRUE) ; sm_GraphicsLoop() ;
                  smgfx_SetLook(2,3, 0,0,0) ; if(pFace) pFace->Draw() ; sm_GraphicsLoop() ;
                  sm_GraphicsLoop() ;
                }
#else
                SM_REF1(pSurface);
#endif // SM_DEBUG_CODE

              // all done 
              return(TRUE) ;

            } // end found a repeated pair of control points check

        } // end iter every ControlPoint triple on this edge of the ControlNet
    } // end iter every edge of the ControlNet

  // all done
  return(FALSE) ;

} // end sm_HasRepeatedEndControlPoints

/*******************************************************************//**
PURPOSE: Return true when the knot multiplicity is incorrect

NOTES: At ends, the knot multiplicity greater than deg + 1 is not allowed.
       At interior, the knot multiplicity greater than degree is not allowed
       Requires future "healing" function to repair

***********************************************************************/
SmBoolean sm_HasKnotMultiplicityGreaterThanDegree
 ( const gw_SURFACE *pSur,        // in : target representation
   SmBSplineSurface *pSurface)    // NotUsed: in : only used for debug, default:[NULL]
{
    SM_REF1(pSurface) ;
    // no work - no BSplineSurface representation
    if( pSur == NULL )
        return FALSE;


#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  if(bDebugMe)
    {
      Dump_NSrf(pSur, false) ;
    }
#endif // SM_DEBUG_CODE

    ULONG uDeg = (ULONG)pSur->p;

    SmTArray<double> uKnots;
    SmTArray<ULONG> uKnotMult;
    SER(sm_GetKnots( pSur->knu, uKnots, &uKnotMult, NULL) );

    ULONG ii ;
    for( ii = 0; ii < uKnotMult.GetSize(); ii++ ) {
        if( ii == 0 || ii == uKnotMult.GetSize() - 1 ) {
            if( uKnotMult[ii] > uDeg + 1 )
                return( TRUE );
        }
        else {
            if( uKnotMult[ii] > uDeg  )
                return( TRUE );
        }
    }

    ULONG vDeg = (ULONG)pSur->q;
  
    SmTArray<double> vKnots;
    SmTArray<ULONG> vKnotMult;
    SER(sm_GetKnots(pSur->knv, vKnots, &vKnotMult, NULL) );

    for( ii = 0; ii < vKnotMult.GetSize(); ii++ ) {
        if( ii == 0 || ii == vKnotMult.GetSize() - 1 ) {
            if( vKnotMult[ii] > vDeg + 1 )
                return( TRUE );
        }
        else {
            if( vKnotMult[ii] > vDeg  )
                return( TRUE );
        }
    }

  // all done
  return(FALSE) ;

} // end sm_HasKnotMultiplicityGreaterThanDegree


/*******************************************************************//**
PURPOSE: Pretty Print gw_SURFACE struct

USAGE NOTES---- Move to SmGraphicsNLib.cpp
***********************************************************************/
void Dump_NSrf
 (const gw_SURFACE * pSur,    // in : target representation
  SmBoolean          bAbbrev) // in : TRUE = decimate CPt and Knot reports, default:[TRUE]
{
  TCHAR sBuff[SM_TBLOCK_SIZE], sBuffForFile[SM_TBLOCK_SIZE] ;

  // gw_SURFACE ptr
  smos_sprintf(sBuff,_T("\nBEGIN Dump m_pNurb(gw_SURFACE) = 0x%p "),pSur);
  smos_sprintf(sBuffForFile,_T("\nBEGIN Dump m_pNurb(gw_SURFACE) = %s "),pSur ? _T("notNULL") : _T("NULL"));
  smos_WriteBuffer(sBuff, sBuffForFile);

  // no work - pSur is NULL
  if ( pSur == NULL ) return ;

  // locals
  gw_CNET  *pNet = pSur->net;  
  gw_FLAG   lrat = ( N_IsSrfRat((gw_SURFACE *)pSur) ) ? 1 : 0 ;

  // ControlPoint Counts, Degrees, RationalFlag
  smos_sprintf(sBuff,_T("\n   # CP_u = %ld, # CP_v = %ld, Deg_u = %d, Deg_v = %d %s"), 
             pNet->n+1, pNet->m+1, pSur->p, pSur->q, lrat ? _T("Rational") : _T("NotRational"));
  smos_WriteBuffer(sBuff);

  // output every requested controlPoint
  ULONG inc_n = bAbbrev ? pNet->n : 1;
  ULONG inc_m = bAbbrev ? pNet->m : 1;

  // Control Point label
  SmBoolean bIsRational = pNet->Pw[0][0].w == NL_NOW ? FALSE : TRUE ;
  if(!bIsRational) { smos_sprintf(sBuff,_T("\n  ControlPoint [CartX, CartY, CartZ]  #Total= %lu"), (pNet->n+1)*(pNet->m+1)) ; }
  else             { smos_sprintf(sBuff,_T("\n  ControlPoint [W*X, W*Y, W*Z, W]  #Total= %lu"), (pNet->n+1)*(pNet->m+1)) ; }
  smos_WriteBuffer(sBuff) ;

  for (gw_INDEX ii=0; ii<=pNet->n; ii+= inc_n) 
    {
      for (gw_INDEX j=0; j<=pNet->m; j+= inc_m) 
        {
          // output PointIndex, x, y, z, optional w
          smos_sprintf(sBuff,_T("\n  [%ld][%ld] = %16.16lf %16.16lf %16.16lf"),
                              ii,j, 
                              pNet->Pw[ii][j].x,
                              pNet->Pw[ii][j].y,
                              pNet->Pw[ii][j].z);
          smos_WriteBuffer(sBuff);
          if (pNet->Pw[ii][j].w != NL_NOW) { smos_sprintf(sBuff,_T(" %16.16lf"),pNet->Pw[ii][j].w);
                                             smos_WriteBuffer(sBuff);
                                           }
        }
      smos_WriteBuffer(_T("\n"));
    } // end iter every controlPoint

  // For rational Control Points also output Control Points in euclidean coordinates
  if(bIsRational) 
    {
      smos_sprintf(sBuff,_T("\n  Euclidean ControlPoints[CartX, CartY, CartZ]  #Total= %lu"), (pNet->n+1)*(pNet->m+1)) ; 
      smos_WriteBuffer(sBuff) ;
      for (gw_INDEX ii=0; ii<=pNet->n; ii+= inc_n) 
        {
          for (gw_INDEX j=0; j<=pNet->m; j+= inc_m) 
            {
              SmPoint3d sPnt;
              TO_EUCLID(pNet->Pw[ii][j], sPnt);

              // output PointIndex, x, y, z
              smos_sprintf(sBuff,_T("\n  [%ld][%ld] = %16.16lf %16.16lf %16.16lf"),
                                  ii,j, 
                                  sPnt.x,
                                  sPnt.y,
                                  sPnt.z);
              smos_WriteBuffer(sBuff);
            }
          smos_WriteBuffer(_T("\n"));
        } // end iter every controlPoint
    } // end ControlPoints are rational check

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  if(bDebugMe)
    {
      // For pattern seraching - print Control Point label by colums
      smos_WriteBuffer(_T("Print ControlPoints a 2nd time by Column\n"));
      SmBoolean bIsRat = pNet->Pw[0][0].w == NL_NOW ? FALSE : TRUE ;
      if(!bIsRat) { smos_sprintf(sBuff,_T("\n  ControlPoint [CartX, CartY, CartZ]  #Total= %lu"), (pNet->n+1)*(pNet->m+1)) ; }
      else             { smos_sprintf(sBuff,_T("\n  ControlPoint [W*X, W*Y, W*Z, W]  #Total= %lu"), (pNet->n+1)*(pNet->m+1)) ; }
      smos_WriteBuffer(sBuff) ;

      for (gw_INDEX j=0; j<=pNet->m; j+= inc_m) 
        {
          for (gw_INDEX ii=0; ii<=pNet->n; ii+= inc_n) 
            {
              //      SmPoint3d sPnt;
              //      TO_EUCLID(pNet->Pw[ii][j],sPnt);
              //      smos_sprintf(sBuff,_T("\n     [%ld][%ld] = "),ii,j);
              //      smos_WriteBuffer(sBuff);
              //      sPnt.Dump(bAbbrev);

              // output PointIndex, x, y, z, optional w
              smos_sprintf(sBuff,_T("\n  [%ld][%ld] = %16.16lf %16.16lf %16.16lf"),
                                  ii,j, 
                                  pNet->Pw[ii][j].x,
                                  pNet->Pw[ii][j].y,
                                  pNet->Pw[ii][j].z);
              smos_WriteBuffer(sBuff);
              if (pNet->Pw[ii][j].w != NL_NOW) { smos_sprintf(sBuff,_T(" %16.16lf"),pNet->Pw[ii][j].w);
                                                 smos_WriteBuffer(sBuff);
                                               }
            }
          smos_WriteBuffer(_T("\n"));
        } // end iter every controlPoint
    }
#endif // SM_DEBUG_CODE

  // output u and v knots
  SmTArray<double> sUKnots, sVKnots;
  SmTArray<ULONG>  sUMult,  sVMult;

  SE(sm_GetKnots(pSur->knu, sUKnots, &sUMult));
  SE(sm_GetKnots(pSur->knv, sVKnots, &sVMult));

  ULONG inc_ir = bAbbrev ? sUKnots.GetSize()-1 : 1 ;
  ULONG inc_is = bAbbrev ? sVKnots.GetSize()-1 : 1 ;

  smos_sprintf(sBuff,_T("\n   # Unique U Knots = %ld, # Unique V Knots = %ld "), 
             sUKnots.GetSize(), sVKnots.GetSize());
  smos_WriteBuffer(sBuff);

  // for every UKnot
  smos_WriteBuffer(_T("\n   KnotU [value, multiplicity]"));
  inc_n = bAbbrev ? sUKnots.GetSize()-1 : 1;
  for (ULONG i=0; i<sUKnots.GetSize(); i+= inc_ir) 
    {
      smos_sprintf(sBuff,_T("\n     %16.16lf, %ld"), sUKnots[i],sUMult[i]);
      smos_WriteBuffer(sBuff);
    }

  // output v knots
  smos_WriteBuffer(_T("\n   KnotV [value, multiplicity]"));
  inc_m = bAbbrev ? sVKnots.GetSize()-1 : 1;
  for (ULONG j=0; j<sVKnots.GetSize(); j+= inc_is) 
    {
      smos_sprintf(sBuff,_T("\n     %16.16lf, %ld"),sVKnots[j],sVMult[j]);
      smos_WriteBuffer(sBuff);
    }
  smos_WriteBuffer(_T("\n"));

  // isDegenerate
  SmBoolean bDegenerate     = sm_IsNSrfDegenerate(pSur, SM_ZONE_TOL_3D) ;
  SmBoolean bHasRepeatCpts  = sm_HasRepeatedEndControlPoints(SM_ZONE_TOL_3D, pSur) ;
  SmBoolean bHasRepeatKnots = false; //sm_HasKnotMultiplicityGreaterThanDegree(pSur) ;

  // for Degenerate surfaces
  if(bDegenerate)
    {
      NL_INDEX lCPtCount = (pNet->n+1) * (pNet->m+1) ;
      smos_sprintf(sBuff,_T("  NOTICE: DegenerateSurface - All %ld Control Points are coincident.\n"), 
                 lCPtCount) ;
      smos_WriteBuffer(sBuff);

      smos_sprintf(sBuff,_T("%s") , "          Not best practice: consider replacing with a point.\n" );
      smos_WriteBuffer(sBuff);
    } // end bDegenerate check

  // for surfaces with repeated end control points within the control net
  if(bHasRepeatCpts)
    {
      // report any endPair ControlPoint coincidence
      smos_WriteBuffer(_T("  WARNING: BSplineSurface ControlNet has a row or col with repeated end ControlPoints - not an error, but not good practice.\n")) ;
    }

  // for surfaces with repeated end control points within the control net
  if(bHasRepeatKnots)
    {
      // report incorrect knot multiplicity
      smos_WriteBuffer(_T("  WARNING: BSplineSurface U or V Knot multiplicity greater than degree.\n")) ;
    }

  // gw_SURFACE ptr
  smos_sprintf(sBuff,_T("\nEND Dump m_pNurb(gw_SURFACE) = 0x%p "),pSur);
  smos_sprintf(sBuffForFile,_T("\nEND Dump m_pNurb(gw_SURFACE) = %s "),pSur ? _T("notNULL") : _T("NULL"));
  smos_WriteBuffer(sBuff, sBuffForFile);

} // end Dump_NSrf
