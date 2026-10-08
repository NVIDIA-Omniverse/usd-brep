// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*******************************************************************//**
* FILE NAME --- SmNurbsVol.cpp
* PURPOSE: Local copies of GW nurbs functions.
**********************************************************************/

#include "StdAfx.h"

#include <SmNurbsVol.h>
#include <nurbs.h>
#include <SmGeomUtility.h>


/*******************************************************************//**
PURPOSE: Find the volume controlPoints nearest/farthest from 
            the given control points.

NOTES:  Checks the distance to all points but only stores
   the best lNumToFind results.

ASSUMES -- 1. the output arrays are allocated on input to size:[lNumToFind].
           2. lNumToFind is <= 10.
***********************************************************************/
SmStatus sm_FindMeshExtrema
  (const gw_VOLUME * pVol,                   // in : Target Volume
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
  SM_ASSERT_MSG(lNumToFind <= 10, _T("sm_FindMeshExtrema() Bad Input Value Alert")) ;

  // init output
  rlNumMinFound = 0;
  rlNumMaxFound = 0;

  // locals
  gw_CPOINT ***Pw = pVol->mesh->Pw;

  // for every Volume controlPoint
  for (long iu = 0; iu <= pVol->mesh->m; iu++) 
    {
      for (long iv = 0; iv <= pVol->mesh->n; iv++) 
        {
          for (long sm = 0; sm <= pVol->mesh->o; sm++)
            {
              // get TestPoint/ControlPoint[iu,iv] gap
              SmPoint3d sPnt; 
              TO_EUCLID(Pw[iu][iv][sm],sPnt);
              SmVector3d sDiffVec = rTestPoint - sPnt;
              double dDistSq = sDiffVec.LengthSquared();

              SmBoolean bInserted = FALSE;

              // for every sorted minDistance ControlPoint 
              for (ULONG i=0; i<rlNumMinFound; i++) 
                {
                  // when gap is smaller than current value
                  if (dDistSq < adMinDistSq[i]) 
                    {
                      // insert this ControlPoint into the min arrays output
                      bInserted = TRUE;
                  
                      // make a hole in the saved min arrays
                      for (ULONG j=rlNumMinFound; j>i; j--) 
                        {
                          adMinDistSq[j]     = adMinDistSq[j-1];
                          alMinUVIndex[j][0] = alMinUVIndex[j-1][0];
                          alMinUVIndex[j][1] = alMinUVIndex[j-1][1];
                        }

                      // fill the min arrays hole with this controlPoint parameters 
                      adMinDistSq[i]     = dDistSq;
                      alMinUVIndex[i][0] = iu;
                      alMinUVIndex[i][1] = iv;

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
              for (ULONG ii=0; ii<rlNumMaxFound; ii++) 
                {
                  // when gap is larger than current value
                  if (dDistSq > adMaxDistSq[ii]) 
                    {
                      // insert this ControlPoint into the max arrays output
                      bInserted = TRUE;

                      // make a hole in the saved max arrays
                      for (ULONG j=rlNumMaxFound; j>ii; j--) 
                        {
                          adMaxDistSq[j] = adMaxDistSq[j-1];
                          alMaxUVIndex[j][0] = alMaxUVIndex[j-1][0];
                          alMaxUVIndex[j][1] = alMaxUVIndex[j-1][1];
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
            } // end iter every W controlPoint index
        } // end iter every V controlPoint index
    } // end iter every U controlPoint index

  // all done
  return SM_SUCCESS;

} // end sm_FindMeshExtrema


/*******************************************************************//**
PURPOSE: Compute the chord height, bounding box, Pseudo Box, angular
    tolerances, and corners for a subset of the control polygon.  

NOTES: 
***********************************************************************/
SmStatus sm_ComputePartialMeshConstants
  (const gw_VOLUME *cpVol,        // in : Volume to query
   ULONG lU0Span,                 // in : Min U Span to query
   ULONG lV0Span,                 // in : Min V Span to query
   ULONG lW0Span,                 // in : Min W Span to query
   ULONG lU1Span,                 // in : Max U Span to query
   ULONG lV1Span,                 // in : Max V Span to query
   ULONG lW1Span,                 // in : Max W Span to query
   SmExtent3d  * pBoundingBox,    // out: Bounding Box
   SmPseudoBox * pPseudoBox,      // out: Pseudo Box 
   SmPoint3d & rU0V0W0,           // out: specified span corner point
   SmPoint3d & rU1V0W0,           // out: specified span corner point
   SmPoint3d & rU0V1W0,           // out: specified span corner point
   SmPoint3d & rU1V1W0,           // out: specified span corner point
   SmPoint3d & rU0V0W1,           // out: specified span corner point
   SmPoint3d & rU1V0W1,           // out: specified span corner point
   SmPoint3d & rU0V1W1,           // out: specified span corner point
   SmPoint3d & rU1V1W1,           // out: specified span corner point
   double * pdUChordHeight,       // out: Max U Chord height along any single row of constant vw CPoints
   double * pdVChordHeight,       // out: Max V Chord height along any single row of constant uw CPoints
   double * pdWChordHeight,       // out: Max W Chord height along any single row of constant uv CPoints
   double * pdUAngleDeg,          // out: Max U tangAngle Change along any single row of constant vw CPoints
   double * pdVAngleDeg,          // out: Max V tangAngle Change along any single row of constant uw CPoints
   double * pdWAngleDeg)          // out: Max W tangAngle Change along any single row of constant uv CPoints
{
  // check for input errors
  if(lU0Span >= lU1Span) SER(SM_ERR);
  if(lV0Span >= lV1Span) SER(SM_ERR);
  if(lW0Span >= lW1Span) SER(SM_ERR);
  
  // locals
  ULONG j, k ;
  NL_CMESH  *pMesh = cpVol->mesh;
  gw_CPOINT ***Pw  = pMesh->Pw;
  double dMaxUAngle = 0.0;
  double dMaxVAngle = 0.0;
  double dMaxWAngle = 0.0;
  if (pdUChordHeight) *pdUChordHeight = 0.0;
  if (pdVChordHeight) *pdVChordHeight = 0.0;
  if (pdVChordHeight) *pdWChordHeight = 0.0;
  if (pdUAngleDeg) *pdUAngleDeg = 0.0;
  if (pdVAngleDeg) *pdVAngleDeg = 0.0;
  if (pdWAngleDeg) *pdVAngleDeg = 0.0;

  ULONG lDegU = cpVol->p ;
  ULONG lDegV = cpVol->q ;
  ULONG lDegW = cpVol->r ;

  // Get the range of ControlPoints with nonzero
  // basis functions over the given range of spans.                          
  ULONG lU0CPoint = lU0Span - lDegU  ;
  ULONG lU1CPoint = lU1Span ;
  ULONG lV0CPoint = lV0Span - lDegV  ;
  ULONG lV1CPoint = lV1Span ;
  ULONG lW0CPoint = lW0Span - lDegW  ;
  ULONG lW1CPoint = lW1Span ;
                                                                                    
  // initialize bounding box when present                                              
  if (pBoundingBox) pBoundingBox->Init();                                              
                                                                                       
  {  TO_EUCLID(Pw[lU0CPoint][lV0CPoint][lW0CPoint],rU0V0W0);
     TO_EUCLID(Pw[lU1CPoint][lV0CPoint][lW0CPoint],rU1V0W0);
     TO_EUCLID(Pw[lU0CPoint][lV1CPoint][lW0CPoint],rU0V1W0);
     TO_EUCLID(Pw[lU1CPoint][lV1CPoint][lW0CPoint],rU1V1W0);
     TO_EUCLID(Pw[lU0CPoint][lV0CPoint][lW1CPoint],rU0V0W1);
     TO_EUCLID(Pw[lU1CPoint][lV0CPoint][lW1CPoint],rU1V0W1);
     TO_EUCLID(Pw[lU0CPoint][lV1CPoint][lW1CPoint],rU0V1W1);
     TO_EUCLID(Pw[lU1CPoint][lV1CPoint][lW1CPoint],rU1V1W1);
     if (pPseudoBox) 
       {
         SmVector3d sV1 = (rU0V0W0 - rU1V0W0) + (rU0V1W0 - rU1V1W0);
         SmVector3d sV2 = (rU0V0W0 - rU0V1W0) + (rU1V0W0 - rU1V1W0);
         double sV1LS = sV1.LengthSquared();
         if (sV1LS > SM_EFF_ZERO_SQ) 
           {  
             SmVector3d sBasis1, sBasis2, sBasis3;
             sV1.MakeUnitOrthoVectors(&sV2,sBasis1,sBasis2,sBasis3);
             pPseudoBox->Init() ; // init intervals - leave basis vectors alone
             pPseudoBox->SetBasis(sBasis1,sBasis2,sBasis3);
           } 
         else 
           {
             pPseudoBox->Init() ; // init intervals - leave basis vectors alone
           }
       }
  }

  {
    // First compute for all V/W columns - compute U ch and U ang
    // along each column
    for(j=lV0CPoint; j<=lV1CPoint; j++)
    {
      for(k=lW0CPoint; k<=lW1CPoint; k++)
      {
        SmPoint3d  sLinePnt;
        SmVector3d sLineVec;
        if (pdUChordHeight) 
          {
            TO_EUCLID(Pw[lU0CPoint][j][k],sLinePnt);
            SmPoint3d sLastPoint; TO_EUCLID(Pw[lU1CPoint][j][k],sLastPoint);
            sLineVec = sLastPoint - sLinePnt;
            ULONG lTryIndex = lU1CPoint-1;
            while (sLineVec.LengthSquared() < SM_EFF_ZERO_SQ) 
              {
                // If first and last point coincide try a middle point
                SmPoint3d sTryPoint; 
                TO_EUCLID(Pw[lTryIndex][j][k],sTryPoint);  
                sLineVec = sTryPoint - sLinePnt;
                if (lTryIndex == 0) 
                  {
                    // singular point just continue on
                    break;
                  }
                lTryIndex--;
              }
            if (sLineVec.LengthSquared() < SM_EFF_ZERO_SQ) continue; // singular point
          } // end need to get UChordHeight Check
        
        double     dTotalAngle = 0.0;
        SmVector3d sDiffVector(0,0,0);
        SmVector3d sLastVec(0,0,0);

        // Hold U constant and loop accross V/W points
        SmPoint3d sCurr; TO_EUCLID(Pw[lU0CPoint][j][k],sCurr);
        for (ULONG i=lU0CPoint+1; i<=lU1CPoint; i++) 
          {
            SmPoint3d sNext; TO_EUCLID(Pw[i][j][k],sNext);
            if (pdUChordHeight) 
              {
                double dLineParam;
                SER(smgu_LineClosestPoint(sLinePnt,sLineVec,sNext,dLineParam));
                SmPoint3d  sPntOnLine = sLinePnt + (sLineVec * dLineParam);
                SmVector3d sCHVec     = sNext - sPntOnLine;
                double dCHSq = sCHVec.LengthSquared();
                if (dCHSq > *pdUChordHeight) *pdUChordHeight = dCHSq;
              }
            if (pdUAngleDeg) 
              {
                SmVector3d sCurrVec = sNext - sCurr;
                double     dTol     = SM_EFF_ZERO_SQRT * 100.0 * (1.0 + sNext.GetMaxDimension());
                double     dDist    = sCurrVec.Length();
                if (dDist < dTol) break;
                sCurrVec = sCurrVec / dDist;
                if (sLastVec.LengthSquared() > SM_EFF_ZERO_SQ) 
                  {
                    double dCosAngle = sCurrVec.Dot(sLastVec);
                    if (dCosAngle >  1.0) dCosAngle =  1.0;
                    if (dCosAngle < -1.0) dCosAngle = -1.0;
                    dTotalAngle += smos_ArcCosine(dCosAngle);
                    // smos_ArcCosine(dCosAngle);
                  }
                sLastVec = sCurrVec;
              }
            sCurr = sNext;
          }
        if (dTotalAngle > dMaxUAngle) dMaxUAngle = dTotalAngle;
      } // Compute constants for each U row
    }
  } // end compute for all V/W columns - compute U ch and U ang

  {
    // First compute for all U/W rows - compute V ch and V ang
    // along each row.
    for(j=lU0CPoint; j<=lU1CPoint; j++) 
    {
      for(k=lW0CPoint; k<=lW1CPoint; k++)
      {
        SmPoint3d sLinePnt;
        SmVector3d sLineVec;
        if (pdVChordHeight) 
          {
            TO_EUCLID(Pw[j][lV0CPoint][k],sLinePnt);
            SmPoint3d sLastPoint; TO_EUCLID(Pw[j][lV1CPoint][k],sLastPoint);
            sLineVec = sLastPoint - sLinePnt;
            ULONG lTryIndex = lV1CPoint-1;
            while (sLineVec.LengthSquared() < SM_EFF_ZERO_SQ) 
              {
                // If first and last point coincide try a middle point
                SmPoint3d sTryPoint; TO_EUCLID(Pw[j][lTryIndex][k],sTryPoint);  
                sLineVec = sTryPoint - sLinePnt;
                if (lTryIndex == 0) 
                  {
                    // singular point just continue on
                    break;
                  }
                lTryIndex--;
              }
            if (sLineVec.LengthSquared() < SM_EFF_ZERO_SQ) continue; // singular point
          }

        double     dTotalAngle = 0.0;
        SmVector3d sDiffVector(0,0,0);
        SmVector3d sLastVec(0,0,0);

        // Hold U/W constant and loop accross V points
        SmPoint3d sCurr; TO_EUCLID(Pw[j][lV0CPoint][k],sCurr);
        if (pBoundingBox) pBoundingBox->AddPoint3d(sCurr);
        if (pPseudoBox)   pPseudoBox->AddPoint3d(sCurr);

        for (ULONG i=lV0CPoint+1; i<=lV1CPoint; i++) 
          {
            SmPoint3d sNext; TO_EUCLID(Pw[j][i][k],sNext);
            if (pBoundingBox) pBoundingBox->AddPoint3d(sNext);
            if (pPseudoBox)   pPseudoBox->AddPoint3d(sNext);
            double dLineParam;
            if (pdVChordHeight) 
              {
                SER(smgu_LineClosestPoint(sLinePnt,sLineVec,sNext,dLineParam));
                SmPoint3d  sPntOnLine = sLinePnt + (sLineVec * dLineParam);
                SmVector3d sCHVec     = sNext - sPntOnLine;
                double     dCHSq      = sCHVec.LengthSquared();
                if (dCHSq > *pdVChordHeight) *pdVChordHeight = dCHSq;
              }

            if (pdVAngleDeg) 
              {
                SmVector3d sCurrVec = sNext - sCurr;
                double     dDist    = sCurrVec.Length();
                double     dTol     = SM_EFF_ZERO_SQRT * 100.0 * (1.0 + sNext.GetMaxDimension());
                if (dDist < dTol) break;
                sCurrVec = sCurrVec / dDist;
                if (sLastVec.LengthSquared() > SM_EFF_ZERO_SQ) 
                  {
                    double dCosAngle = sCurrVec.Dot(sLastVec);
                    if (dCosAngle >  1.0) dCosAngle = 1.0;
                    if (dCosAngle < -1.0) dCosAngle = -1.0;
                    dTotalAngle += smos_ArcCosine(dCosAngle);

                  }
                sLastVec = sCurrVec;
              }
            sCurr = sNext;
          }
        if (dTotalAngle > dMaxVAngle) dMaxVAngle = dTotalAngle;
      } // Compute constants for each V row
    }
  } // end compute for all U/W rows - compute V ch and V ang

  {
    // First compute for all U/V rows - compute W ch and W ang
    // along each row.
    for(j=lU0CPoint; j<=lU1CPoint; j++) 
    {
      for(k=lV0CPoint; k<=lV1CPoint; k++)
      {
        SmPoint3d  sLinePnt;
        SmVector3d sLineVec;
        if (pdWChordHeight) 
          {
            TO_EUCLID(Pw[j][k][lW0CPoint],sLinePnt);
            SmPoint3d sLastPoint; TO_EUCLID(Pw[j][k][lW1CPoint],sLastPoint);
            sLineVec = sLastPoint - sLinePnt;
            ULONG lTryIndex = lW1CPoint-1;
            while (sLineVec.LengthSquared() < SM_EFF_ZERO_SQ) 
              {
                // If first and last point coincide try a middle point
                SmPoint3d sTryPoint; TO_EUCLID(Pw[j][k][lTryIndex],sTryPoint);  
                sLineVec = sTryPoint - sLinePnt;
                if (lTryIndex == 0) 
                  {
                    // singular point just continue on
                    break;
                  }
                lTryIndex--;
              }
            if (sLineVec.LengthSquared() < SM_EFF_ZERO_SQ) continue; // singular point
          }

        double     dTotalAngle = 0.0;
        SmVector3d sDiffVector(0,0,0);
        SmVector3d sLastVec(0,0,0);

        // Hold U/W constant and loop accross V points
        SmPoint3d sCurr; TO_EUCLID(Pw[j][k][lW0CPoint],sCurr);
        if (pBoundingBox) pBoundingBox->AddPoint3d(sCurr);
        if (pPseudoBox)   pPseudoBox->AddPoint3d(sCurr);

        for (ULONG i=lW0CPoint+1; i<=lW1CPoint; i++) 
          {
            SmPoint3d sNext; TO_EUCLID(Pw[j][k][i],sNext);
            if (pBoundingBox) pBoundingBox->AddPoint3d(sNext);
            if (pPseudoBox)   pPseudoBox->AddPoint3d(sNext);
            double dLineParam;
            if (pdWChordHeight) 
              {
                SER(smgu_LineClosestPoint(sLinePnt,sLineVec,sNext,dLineParam));
                SmPoint3d  sPntOnLine = sLinePnt + (sLineVec * dLineParam);
                SmVector3d sCHVec     = sNext - sPntOnLine;
                double     dCHSq      = sCHVec.LengthSquared();
                if (dCHSq > *pdWChordHeight) *pdWChordHeight = dCHSq;
              }

            if (pdWAngleDeg) 
              {
                SmVector3d sCurrVec = sNext - sCurr;
                double     dDist    = sCurrVec.Length();
                double     dTol     = SM_EFF_ZERO_SQRT * 100.0 * (1.0 + sNext.GetMaxDimension());
                if (dDist < dTol) break;
                sCurrVec = sCurrVec / dDist;
                if (sLastVec.LengthSquared() > SM_EFF_ZERO_SQ) 
                  {
                    double dCosAngle = sCurrVec.Dot(sLastVec);
                    if (dCosAngle >  1.0) dCosAngle = 1.0;
                    if (dCosAngle < -1.0) dCosAngle = -1.0;
                    dTotalAngle += smos_ArcCosine(dCosAngle);

                  }
                sLastVec = sCurrVec;
              }
            sCurr = sNext;
          }
        if (dTotalAngle > dMaxVAngle) dMaxVAngle = dTotalAngle;
      } // Compute constants for each W row
    }
  } // end compute for all U/W rows - compute W ch and W ang
  
  if (pdUAngleDeg) *pdUAngleDeg = dMaxUAngle * 180.0 / SM_PI;
  if (pdVAngleDeg) *pdVAngleDeg = dMaxVAngle * 180.0 / SM_PI;
  if (pdWAngleDeg) *pdWAngleDeg = dMaxWAngle * 180.0 / SM_PI;
  if (pdUChordHeight) *pdUChordHeight = smos_Sqrt(*pdUChordHeight);
  if (pdVChordHeight) *pdVChordHeight = smos_Sqrt(*pdVChordHeight);
  if (pdWChordHeight) *pdWChordHeight = smos_Sqrt(*pdWChordHeight);
  
  // all done
  return SM_SUCCESS;

} // end sm_ComputePartialMeshConstants

/*******************************************************************//**
PURPOSE: Compute the chord heights in the U and V direction. 

NOTES: Note that the chord height is only relative to an
    individual cross section of the control mesh.  An estimate of 
    the actual chord height can be obtained by adding these two
    chord heights.
***********************************************************************/
SmStatus sm_ComputeMeshConstants
  (const SmVolume   * pVolume,          // in : 
   const SmExtent3d * pUVWDomain,       // in : 
   const gw_VOLUME  * cpVol,            // in : 
   double           * pdUChordHeight,   // out: 
   double           * pdVChordHeight,   // out: 
   double           * pdWChordHeight,   // out: 
   double           * pdUAngleDeg,      // out: 
   double           * pdVAngleDeg,      // out: 
   double           * pdWAngleDeg)      // out: 
{
  long j, k ;
  NL_CMESH  *pMesh = cpVol->mesh;
  gw_CPOINT ***Pw  = pMesh->Pw;
  double dMaxUAngle = 0.0;
  double dMaxVAngle = 0.0;
  double dMaxWAngle = 0.0;
  if (pdUChordHeight) *pdUChordHeight = 0.0;
  if (pdVChordHeight) *pdVChordHeight = 0.0;
  if (pdWChordHeight) *pdWChordHeight = 0.0;
  if (pdUAngleDeg) *pdUAngleDeg = 0.0;
  if (pdVAngleDeg) *pdVAngleDeg = 0.0;
  if (pdWAngleDeg) *pdWAngleDeg = 0.0;

  {
    // First compute for all VW columns - compute U ch and U ang
    // along each column
    for(j=0; j<=pMesh->n; j++) 
    {
      for(k=0;k<=pMesh->o; k++)
      {
        SmPoint3d  sLinePnt;
        SmVector3d sLineVec;
        if (pdUChordHeight) 
          {
            TO_EUCLID(Pw[0][j][k],sLinePnt);
            SmPoint3d sLastPoint; TO_EUCLID(Pw[pMesh->m][j][k],sLastPoint);
            sLineVec = sLastPoint - sLinePnt;
            ULONG lTryIndex = pMesh->m-1;
            while (sLineVec.LengthSquared() < SM_EFF_ZERO_SQ) 
              {
                // If first and last point coincide try a middle point
                SmPoint3d sTryPoint; 
                TO_EUCLID(Pw[lTryIndex][j][k],sTryPoint);  
                sLineVec = sTryPoint - sLinePnt;
                if (lTryIndex == 0) 
                  {
                    // singular point just continue on
                    break;
                  }
                lTryIndex--;
              }
            if (sLineVec.LengthSquared() < SM_EFF_ZERO_SQ) continue; // singular point
          }
        
        double     dTotalAngle = 0.0;
        SmVector3d sDiffVector(0,0,0);
        SmVector3d sLastVec(0,0,0);
        // Hold U constant and loop accross V points
        SmPoint3d sCurr; TO_EUCLID(Pw[0][j][k],sCurr);
        for (long i=1; i<=pMesh->m; i++) 
          {
            SmPoint3d sNext; TO_EUCLID(Pw[i][j][k],sNext);
            if (pdUChordHeight) 
              {
                double dLineParam;
                SER(smgu_LineClosestPoint(sLinePnt,sLineVec,sNext,dLineParam));
                SmPoint3d sPntOnLine = sLinePnt + (sLineVec * dLineParam);
                SmVector3d sCHVec = sNext - sPntOnLine;
                double dCHSq = sCHVec.LengthSquared();
                if (dCHSq > *pdUChordHeight) *pdUChordHeight = dCHSq;
              }
            if (pdUAngleDeg) 
              {
                SmVector3d sCurrVec = sNext - sCurr;
                double dTol = SM_EFF_ZERO_SQRT * 100.0 * (1.0 + sNext.GetMaxDimension());
                double dDist = sCurrVec.Length();
                if (dDist < dTol) continue;
                sCurrVec = sCurrVec / dDist;
                if (sLastVec.LengthSquared() > SM_EFF_ZERO_SQ) 
                  {
                    double dCosAngle = sCurrVec.Dot(sLastVec);
                    if (dCosAngle >  1.0) dCosAngle = 1.0;
                    if (dCosAngle < -1.0) dCosAngle = -1.0;
                    dTotalAngle += smos_ArcCosine(dCosAngle);
                    // smos_ArcCosine(dCosAngle);
                  }
                sLastVec.x = sCurrVec.x;
                sLastVec.y = sCurrVec.y;
                sLastVec.z = sCurrVec.z;
              }
            sCurr.x = sNext.x;
            sCurr.y = sNext.y;
            sCurr.z = sNext.z;
          }
        if (dTotalAngle > dMaxUAngle) dMaxUAngle = dTotalAngle;
      } // Compute constants for each U row
    }
  } // end compute for all VW columns - compute U ch and U ang

  {
    // First compute for all UW rows - compute V ch and V ang
    // along each row.
    for (j=0; j<=pMesh->m; j++) 
    {
      for(k=0; k<=pMesh->o; k++)
      {
        SmPoint3d sLinePnt;
        SmVector3d sLineVec;
        if (pdVChordHeight) 
          {
            TO_EUCLID(Pw[j][0][k],sLinePnt);
            SmPoint3d sLastPoint; TO_EUCLID(Pw[j][pMesh->n][k],sLastPoint);
            sLineVec.x = sLastPoint.x - sLinePnt.x;
            sLineVec.y = sLastPoint.y - sLinePnt.y;
            sLineVec.z = sLastPoint.z - sLinePnt.z;
            ULONG lTryIndex = pMesh->n-1;
            while (sLineVec.LengthSquared() < SM_EFF_ZERO_SQ) 
              {
                // If first and last point coincide try a middle point
                SmPoint3d sTryPoint; TO_EUCLID(Pw[j][lTryIndex][k],sTryPoint);  
                sLineVec.x = sTryPoint.x - sLinePnt.x;
                sLineVec.y = sTryPoint.y - sLinePnt.y;
                sLineVec.z = sTryPoint.z - sLinePnt.z;
                if (lTryIndex == 0) 
                  {
                    // singular point just continue on
                    break;
                  }
                lTryIndex--;
              }
            if (sLineVec.LengthSquared() < SM_EFF_ZERO_SQ) continue; // singular point
          }
        double     dTotalAngle = 0.0;
        SmVector3d sDiffVector(0,0,0);
        SmVector3d sLastVec(0,0,0);
        // Hold UW constant and loop accross V points
        SmPoint3d sCurr; TO_EUCLID(Pw[j][0][k],sCurr);
        for (long i=1; i<=pMesh->n; i++) 
          {
            SmPoint3d sNext; TO_EUCLID(Pw[j][i][k],sNext);
            double dLineParam;
            if (pdVChordHeight) 
              {
                SER(smgu_LineClosestPoint(sLinePnt,sLineVec,sNext,dLineParam));
                SmPoint3d  sPntOnLine = sLinePnt + (sLineVec * dLineParam);
                SmVector3d sCHVec     = sNext - sPntOnLine;
                double     dCHSq      = sCHVec.LengthSquared();
                if (dCHSq > *pdVChordHeight) *pdVChordHeight = dCHSq;
              }
            if (pdVAngleDeg) 
              {
                SmVector3d sCurrVec = sNext - sCurr;
                double     dDist    = sCurrVec.Length();
                double     dTol     = SM_EFF_ZERO_SQRT * 100.0 * (1.0 + sNext.GetMaxDimension());
                if (dDist < dTol) continue;
                sCurrVec = sCurrVec / dDist;
                if (sLastVec.LengthSquared() > SM_EFF_ZERO_SQ) 
                  {
                    double dCosAngle = sCurrVec.Dot(sLastVec);
                    if (dCosAngle >  1.0) dCosAngle = 1.0;
                    if (dCosAngle < -1.0) dCosAngle = -1.0;
                    dTotalAngle += smos_ArcCosine(dCosAngle);

                  }
                sLastVec.x = sCurrVec.x;
                sLastVec.y = sCurrVec.y;
                sLastVec.z = sCurrVec.z;
              }
            sCurr.x = sNext.x;
            sCurr.y = sNext.y;
            sCurr.z = sNext.z;
          }
        if (dTotalAngle > dMaxVAngle) dMaxVAngle = dTotalAngle;
      } // Compute constants for each V row
    }
  } // end for all UW rows - compute V ch and V ang
  
  {
    // First compute for all UW rows - compute V ch and V ang
    // along each row.
    for (j=0; j<=pMesh->m; j++) 
    {
      for(k=0; k<=pMesh->n; k++)
      {
        SmPoint3d sLinePnt;
        SmVector3d sLineVec;
        if (pdWChordHeight) 
          {
            TO_EUCLID(Pw[j][k][0],sLinePnt);
            SmPoint3d sLastPoint; TO_EUCLID(Pw[j][k][pMesh->n],sLastPoint);
            sLineVec.x = sLastPoint.x - sLinePnt.x;
            sLineVec.y = sLastPoint.y - sLinePnt.y;
            sLineVec.z = sLastPoint.z - sLinePnt.z;
            ULONG lTryIndex = pMesh->n-1;
            while (sLineVec.LengthSquared() < SM_EFF_ZERO_SQ) 
              {
                // If first and last point coincide try a middle point
                SmPoint3d sTryPoint; TO_EUCLID(Pw[j][k][lTryIndex],sTryPoint);  
                sLineVec.x = sTryPoint.x - sLinePnt.x;
                sLineVec.y = sTryPoint.y - sLinePnt.y;
                sLineVec.z = sTryPoint.z - sLinePnt.z;
                if (lTryIndex == 0) 
                  {
                    // singular point just continue on
                    break;
                  }
                lTryIndex--;
              }
            if (sLineVec.LengthSquared() < SM_EFF_ZERO_SQ) continue; // singular point
          }
        double     dTotalAngle = 0.0;
        SmVector3d sDiffVector(0,0,0);
        SmVector3d sLastVec(0,0,0);
        // Hold UV constant and loop accross W points
        SmPoint3d sCurr; TO_EUCLID(Pw[j][k][0],sCurr);
        for (long i=1; i<=pMesh->o; i++) 
          {
            SmPoint3d sNext; TO_EUCLID(Pw[j][k][i],sNext);
            double dLineParam;
            if (pdWChordHeight) 
              {
                SER(smgu_LineClosestPoint(sLinePnt,sLineVec,sNext,dLineParam));
                SmPoint3d  sPntOnLine = sLinePnt + (sLineVec * dLineParam);
                SmVector3d sCHVec     = sNext - sPntOnLine;
                double     dCHSq      = sCHVec.LengthSquared();
                if (dCHSq > *pdWChordHeight) *pdWChordHeight = dCHSq;
              }
            if (pdWAngleDeg) 
              {
                SmVector3d sCurrVec = sNext - sCurr;
                double     dDist    = sCurrVec.Length();
                double     dTol     = SM_EFF_ZERO_SQRT * 100.0 * (1.0 + sNext.GetMaxDimension());
                if (dDist < dTol) continue;
                sCurrVec = sCurrVec / dDist;
                if (sLastVec.LengthSquared() > SM_EFF_ZERO_SQ) 
                  {
                    double dCosAngle = sCurrVec.Dot(sLastVec);
                    if (dCosAngle >  1.0) dCosAngle = 1.0;
                    if (dCosAngle < -1.0) dCosAngle = -1.0;
                    dTotalAngle += smos_ArcCosine(dCosAngle);

                  }
                sLastVec.x = sCurrVec.x;
                sLastVec.y = sCurrVec.y;
                sLastVec.z = sCurrVec.z;
              }
            sCurr.x = sNext.x;
            sCurr.y = sNext.y;
            sCurr.z = sNext.z;
          }
        if (dTotalAngle > dMaxVAngle) dMaxVAngle = dTotalAngle;
      } // Compute constants for each W row
    }
  } // end for all UV rows - compute W ch and W ang
  
  if (pVolume) 
    {
      // First compute for all VW columns - compute U ch and U ang
      // along each column
      for (j=0; j<=4; j++) 
      {
        for(k=0; k<=4; k++)
        {
          SmPoint3d  sLinePnt;
          SmVector3d sLineVec;
          double dV = (double)j/4.0;
          double dW = (double)j/4.0;
          if (pdUChordHeight) 
            {
              SER(pVolume->EvaluatePoint(pUVWDomain->Evaluate(0.0,dV,dW),sLinePnt));
              SmPoint3d sLastPoint; 
              SER(pVolume->EvaluatePoint(pUVWDomain->Evaluate(1.0,dV,dW),sLastPoint));
              sLineVec = sLastPoint - sLinePnt;
              ULONG lTryIndex = 3;
              while (sLineVec.LengthSquared() < SM_EFF_ZERO_SQ) 
                {
                  // If first and last point coincide try a middle point
                  SmPoint3d sTryPoint; 
                  double dU = lTryIndex / 4.0;
                  SER(pVolume->EvaluatePoint(pUVWDomain->Evaluate(dU,dV,dW),sTryPoint));
                  sLineVec = sTryPoint - sLinePnt;
                  if (lTryIndex == 0) 
                    {
                      // singular point just continue on
                      break;
                    }
                  lTryIndex--;
                }
              if (sLineVec.LengthSquared() < SM_EFF_ZERO_SQ) continue; // singular point
            }
          
          double     dTotalAngle = 0.0;
          SmVector3d sDiffVector(0,0,0);
          SmVector3d sLastVec;
          // Hold U constant and loop accross V points
          SmPoint3d sCurr;
          SER(pVolume->EvaluatePoint(pUVWDomain->Evaluate(0.0,dV,dW),sCurr));
          for (long i=1; i<=4; i++) 
            {
              SmPoint3d sNext; 
              double dU = i / 4.0;
              SER(pVolume->EvaluatePoint(pUVWDomain->Evaluate(dU,dV,dW),sNext));
              if (pdUChordHeight) 
                {
                  double dLineParam;
                  SER(smgu_LineClosestPoint(sLinePnt,sLineVec,sNext,dLineParam));
                  SmPoint3d  sPntOnLine = sLinePnt + (sLineVec * dLineParam);
                  SmVector3d sCHVec     = sNext - sPntOnLine;
                  double     dCHSq      = sCHVec.LengthSquared();
                  if (dCHSq > *pdUChordHeight) *pdUChordHeight = dCHSq;
                }
              if (pdUAngleDeg) 
                {
                  SmVector3d sCurrVec = sNext - sCurr;
                  double dTol  = SM_EFF_ZERO_SQRT * 100.0 * (1.0 + sNext.GetMaxDimension());
                  double dDist = sCurrVec.Length();
                  if (dDist < dTol) break;
                  sCurrVec = sCurrVec / dDist;
                  if (i > 1) 
                    {
                      double dCosAngle = sCurrVec.Dot(sLastVec);
                      if (dCosAngle >  1.0) dCosAngle = 1.0;
                      if (dCosAngle < -1.0) dCosAngle = -1.0;
                      dTotalAngle += smos_ArcCosine(dCosAngle);
                      // smos_ArcCosine(dCosAngle);
                    }
                  sLastVec = sCurrVec;
                }
              sCurr = sNext;
            }
          if (dTotalAngle > dMaxUAngle) dMaxUAngle = dTotalAngle;
        } // Compute constants for each U row
      }
    } // end pVolume existence check

  if (pVolume) 
    {
      NER(pUVWDomain);
      // First compute for all UW rows - compute V ch and V ang
      // along each row.
      for (j=0; j<=4; j++) 
      {
        for(k=0; k<=4; k++)
        {
          SmPoint3d sLinePnt;
          SmVector3d sLineVec;
          double dU = (double)j/4.0;
          double dW = (double)k/4.0;
          if (pdVChordHeight) 
            {
              SER(pVolume->EvaluatePoint(pUVWDomain->Evaluate(dU,0.0,dW),sLinePnt));
              SmPoint3d sLastPoint; 
              SER(pVolume->EvaluatePoint(pUVWDomain->Evaluate(dU,1.0,dW),sLastPoint));
              sLineVec = sLastPoint - sLinePnt;
              ULONG lTryIndex = 3;
              while (sLineVec.LengthSquared() < SM_EFF_ZERO_SQ) 
                {
                  // If first and last point coincide try a middle point
                  double dV = (double)lTryIndex / 4.0;
                  SmPoint3d sTryPoint; 
                  SER(pVolume->EvaluatePoint(pUVWDomain->Evaluate(dU,dV,dW),sTryPoint));
                  sLineVec = sTryPoint - sLinePnt;
                  if (lTryIndex == 0) 
                    {
                      // singular point just continue on
                      break;
                    }
                  lTryIndex--;
                }
              if (sLineVec.LengthSquared() < SM_EFF_ZERO_SQ) continue; // singular point
            }
          double     dTotalAngle = 0.0;
          SmVector3d sDiffVector(0,0,0);
          SmVector3d sLastVec(0,0,0);
          // Hold U constant and loop accross V points
          SmPoint3d sCurr;  
          SER(pVolume->EvaluatePoint(pUVWDomain->Evaluate(dU,0.0,dW),sCurr));
          for (long i=1; i<=4; i++) 
            {
              SmPoint3d sNext; 
              double dV = (double)i / 4.0;
              SER(pVolume->EvaluatePoint(pUVWDomain->Evaluate(dU,dV,dW),sNext));
              double dLineParam;
              if (pdVChordHeight) 
                {
                  SER(smgu_LineClosestPoint(sLinePnt,sLineVec,sNext,dLineParam));
                  SmPoint3d  sPntOnLine = sLinePnt + (sLineVec * dLineParam);
                  SmVector3d sCHVec     = sNext - sPntOnLine;
                  double     dCHSq      = sCHVec.LengthSquared();
                  if (dCHSq > *pdVChordHeight) *pdVChordHeight = dCHSq;
                }
              if (pdVAngleDeg) 
                {
                  SmVector3d sCurrVec = sNext - sCurr;
                  double     dDist    = sCurrVec.Length();
                  double     dTol     = SM_EFF_ZERO_SQRT * 100.0 * (1.0 + sNext.GetMaxDimension());
                  if (dDist < dTol) break;
                  sCurrVec = sCurrVec / dDist;
                  if (sLastVec.LengthSquared() > SM_EFF_ZERO_SQ) 
                    {
                      double dCosAngle = sCurrVec.Dot(sLastVec);
                      if (dCosAngle >  1.0) dCosAngle = 1.0;
                      if (dCosAngle < -1.0) dCosAngle = -1.0;
                      dTotalAngle += smos_ArcCosine(dCosAngle);
                      // smos_ArcCosine(dCosAngle);
                    }
                  sLastVec = sCurrVec;
                }
              sCurr = sNext;
            }
          if (dTotalAngle > dMaxVAngle) dMaxVAngle = dTotalAngle;
        } // Compute constants for each U row
      } // end for all UW rows - compute V ch and V ang
    } // end pVolume existence check
  
  if (pVolume) 
    {
      NER(pUVWDomain);
      // First compute for all UV rows - compute W ch and W ang
      // along each row.
      for (j=0; j<=4; j++) 
      {
        for(k=0; k<=4; k++)
        {
          SmPoint3d  sLinePnt;
          SmVector3d sLineVec;
          double dU = (double)j/4.0;
          double dV = (double)k/4.0;
          if (pdWChordHeight) 
            {
              SER(pVolume->EvaluatePoint(pUVWDomain->Evaluate(dU,dV,0.0),sLinePnt));
              SmPoint3d sLastPoint; 
              SER(pVolume->EvaluatePoint(pUVWDomain->Evaluate(dU,dV,1.0),sLastPoint));
              sLineVec = sLastPoint - sLinePnt;
              ULONG lTryIndex = 3;
              while (sLineVec.LengthSquared() < SM_EFF_ZERO_SQ) 
                {
                  // If first and last point coincide try a middle point
                  double dW = (double)lTryIndex / 4.0;
                  SmPoint3d sTryPoint; 
                  SER(pVolume->EvaluatePoint(pUVWDomain->Evaluate(dU,dV,dW),sTryPoint));
                  sLineVec = sTryPoint - sLinePnt;
                  if (lTryIndex == 0) 
                    {
                      // singular point just continue on
                      break;
                    }
                  lTryIndex--;
                }
              if (sLineVec.LengthSquared() < SM_EFF_ZERO_SQ) continue; // singular point
            }
          double     dTotalAngle = 0.0;
          SmVector3d sDiffVector(0,0,0);
          SmVector3d sLastVec(0,0,0);
          // Hold U constant and loop accross V points
          SmPoint3d sCurr;  
          SER(pVolume->EvaluatePoint(pUVWDomain->Evaluate(dU,dV,0.0),sCurr));
          for (long i=1; i<=4; i++) 
            {
              SmPoint3d sNext; 
              double dW = (double)i / 4.0;
              SER(pVolume->EvaluatePoint(pUVWDomain->Evaluate(dU,dV,dW),sNext));
              double dLineParam;
              if (pdWChordHeight) 
                {
                  SER(smgu_LineClosestPoint(sLinePnt,sLineVec,sNext,dLineParam));
                  SmPoint3d  sPntOnLine = sLinePnt + (sLineVec * dLineParam);
                  SmVector3d sCHVec     = sNext - sPntOnLine;
                  double     dCHSq      = sCHVec.LengthSquared();
                  if (dCHSq > *pdWChordHeight) *pdWChordHeight = dCHSq;
                }
              if (pdWAngleDeg) 
                {
                  SmVector3d sCurrVec = sNext - sCurr;
                  double     dDist    = sCurrVec.Length();
                  double     dTol     = SM_EFF_ZERO_SQRT * 100.0 * (1.0 + sNext.GetMaxDimension());
                  if (dDist < dTol) break;
                  sCurrVec = sCurrVec / dDist;
                  if (sLastVec.LengthSquared() > SM_EFF_ZERO_SQ) 
                    {
                      double dCosAngle = sCurrVec.Dot(sLastVec);
                      if (dCosAngle >  1.0) dCosAngle = 1.0;
                      if (dCosAngle < -1.0) dCosAngle = -1.0;
                      dTotalAngle += smos_ArcCosine(dCosAngle);
                      // smos_ArcCosine(dCosAngle);
                    }
                  sLastVec = sCurrVec;
                }
              sCurr = sNext;
            }
          if (dTotalAngle > dMaxVAngle) dMaxVAngle = dTotalAngle;
        } // Compute constants for each U row
      } // end for all UW rows - compute V ch and V ang
    } // end pVolume existence check
  
  
  if (pdUAngleDeg) *pdUAngleDeg = dMaxUAngle * 180.0 / SM_PI;
  if (pdVAngleDeg) *pdVAngleDeg = dMaxVAngle * 180.0 / SM_PI;
  if (pdWAngleDeg) *pdWAngleDeg = dMaxWAngle * 180.0 / SM_PI;
  if (pdUChordHeight) *pdUChordHeight = smos_Sqrt(*pdUChordHeight);
  if (pdVChordHeight) *pdVChordHeight = smos_Sqrt(*pdVChordHeight);
  if (pdWChordHeight) *pdWChordHeight = smos_Sqrt(*pdWChordHeight);
  
  return SM_SUCCESS;

} // end sm_ComputeMeshConstants

/*******************************************************************//**
PURPOSE: Copy the data for a nurbs volume.

NOTES: pTo is allocated prior to this call and is same size as
    cpFrom.
***********************************************************************/
SmStatus sm_CopyNurbVolume
  (const gw_VOLUME * cpFrom, 
   gw_VOLUME       * pTo)
{
  // allocate contiguous memory block
  sm_InitNurbVolumeMemory(pTo,
                             cpFrom->mesh->m,cpFrom->mesh->n,cpFrom->mesh->o,
                             cpFrom->p,cpFrom->q,cpFrom->r,
                             cpFrom->knu->m,cpFrom->knv->m,cpFrom->knw->m);

  // copy knot vectors - okay to use smos_MemCpy on base (double) objects.
  SER(smos_MemCpy(pTo->knu->U, cpFrom->knu->U, sizeof(gw_REAL) * (pTo->knu->m + 1), sizeof(gw_REAL) * (pTo->knu->m + 1)));
  SER(smos_MemCpy(pTo->knv->U, cpFrom->knv->U, sizeof(gw_REAL) * (pTo->knv->m + 1), sizeof(gw_REAL) * (pTo->knv->m + 1)));
  SER(smos_MemCpy(pTo->knw->U, cpFrom->knw->U, sizeof(gw_REAL) * (pTo->knw->m + 1), sizeof(gw_REAL) * (pTo->knw->m + 1)));

  // copy control points - okay to use smos_MemCpy on static class (gw_CPOINT) objects.
  SER(smos_MemCpy(pTo->mesh->Pw[0],
              cpFrom->mesh->Pw[0],
              sizeof(gw_CPOINT) * (pTo->mesh->m + 1) 
                                * (pTo->mesh->n + 1) 
                                * (pTo->mesh->o + 1),sizeof(gw_CPOINT) * (pTo->mesh->m + 1) 
                                * (pTo->mesh->n + 1) 
                                * (pTo->mesh->o + 1)));
  // all done
  return SM_SUCCESS;

} // end sm_CopyNurbVolume

/*******************************************************************//**
PURPOSE: convenience routine for calling sm_ComputeNurbVolumeSize(args)

NOTES:
***********************************************************************/
ULONG sm_ComputeNurbVolumeSize
  (const gw_VOLUME *cpVolume)      // in : surface to be examined
{
  // pass the call along
  return( cpVolume ? sm_ComputeNurbVolumeSize(cpVolume->mesh->m,
                                              cpVolume->mesh->n,
                                              cpVolume->mesh->o,
                                              cpVolume->knu->m,
                                              cpVolume->knv->m,
                                              cpVolume->knw->m)
                    : 0 ) ;
} // end sm_ComputeNurbVolumeSize

/*******************************************************************//**
PURPOSE: Given the highest UV knots indices and the highest
    UV ControlPoint indices, compute the size of the resulting nurb volume.

NOTES: 
***********************************************************************/
ULONG sm_ComputeNurbVolumeSize
  (gw_INDEX lUCPointHighestIndex,  // in : max U ControlPoint index value = U ControlPoint_count - 1
   gw_INDEX lVCPointHighestIndex,  // in : max V ControlPoint index value = V ControlPoint_count - 1
   gw_INDEX lWCPointHighestIndex,  // in : max W ControlPoint index value = V ControlPoint_count - 1
   gw_INDEX lUKnotsHighestIndex,   // in : max U knot index value = U knot_count - 1                                           
   gw_INDEX lVKnotsHighestIndex,   // in : max V knot index value = V knot_count - 1                                           
   gw_INDEX lWKnotsHighestIndex)   // in : max W knot index value = W knot_count - 1                                           
{
  // This routine computes the size of a single piece of memory to
  // contain the nurb Volume.  The following order is used
  // to map the memory to the Volume structure:
  //    gw_VOLUME
  //    NL_CMESH
  //    KNOTVECTORU
  //    KNOTVECTORV
  //    <array of double for knots U>
  //    <array of double for knots V>
  //    <array of double for knots W>
  //    <array of pointers to gw_CPOINT**>
  //    <array of pointers to gw_CPOINT*>
  //    <array of double*4 for CPOINTS>
  ULONG lTotalSize = 
    (  ALIGN_SIZE(sizeof(gw_VOLUME)) 
     + ALIGN_SIZE(sizeof( NL_CMESH ))
     + ALIGN_SIZE(sizeof(gw_KNOTVECTOR)) 
     + ALIGN_SIZE(sizeof(gw_KNOTVECTOR))    
     + ALIGN_SIZE(sizeof(gw_KNOTVECTOR))    
     + ALIGN_SIZE(sizeof(gw_REAL) * (lUKnotsHighestIndex+1))   
     + ALIGN_SIZE(sizeof(gw_REAL) * (lVKnotsHighestIndex+1))   
     + ALIGN_SIZE(sizeof(gw_REAL) * (lWKnotsHighestIndex+1))   
     + ALIGN_SIZE(sizeof(gw_CPOINT**) * (lUCPointHighestIndex+1))  
     + ALIGN_SIZE(sizeof(gw_CPOINT*) * (lUCPointHighestIndex+1)
                                     * (lVCPointHighestIndex+1)) 
     + ALIGN_SIZE(sizeof(gw_CPOINT) * (lUCPointHighestIndex+1) 
                                    * (lVCPointHighestIndex+1)
                                    * (lWCPointHighestIndex+1))) ;
  // all done
  return lTotalSize;    

} // end sm_ComputeNurbVolumeSize

/*******************************************************************//**
PURPOSE: Given a pointer to a proper sized block of memory and the critical
    volume size information, initialize the memory for the Nurbs Volume.

NOTES: 

  Internal Volume pointers are set.
  Internal Volume size values are set.
  ControlPoint and knot values are not touched.

  Pointers to volume memory blocks are stored to mimick
  the data structures built into NLibs.  These pointers point to locations
  within the single contiguous block being built in pVolumeMemory.

***********************************************************************/
void sm_InitNurbVolumeMemory
  (gw_VOLUME* pVolumeMemory,        // in : pointer to volume memory block to init, 
                                    //      sized:[sm_ComputeNurbVolumeSize()]
   gw_INDEX  lUCPointHighestIndex,  // in : max U ControlPoint index value = U ControlPoint_count - 1
   gw_INDEX  lVCPointHighestIndex,  // in : max V ControlPoint index value = V ControlPoint_count - 1
   gw_INDEX  lWCPointHighestIndex,  // in : max W ControlPoint index value = V ControlPoint_count - 1
   gw_DEGREE lUDegree,              // in : U dir degree                                             
   gw_DEGREE lVDegree,              // in : V dir degree                                             
   gw_DEGREE lWDegree,              // in : W dir degree                                             
   gw_INDEX  lUKnotsHighestIndex,   // in : max U knot index value = U knot_count - 1                
   gw_INDEX  lVKnotsHighestIndex,   // in : max V knot index value = V knot_count - 1                
   gw_INDEX  lWKnotsHighestIndex)   // in : max W knot index value = W knot_count - 1                
{ 
  // set internal memory pointers
  NL_CMESH      * pMesh     = pVolumeMemory->mesh = (NL_CMESH*)     (((char*)pVolumeMemory) + ALIGN_SIZE(sizeof(gw_VOLUME)));
  gw_KNOTVECTOR * pUKnotVec = pVolumeMemory->knu  = (gw_KNOTVECTOR*)(((char*)pMesh)         + ALIGN_SIZE(sizeof( NL_CMESH )));
  gw_KNOTVECTOR * pVKnotVec = pVolumeMemory->knv  = (gw_KNOTVECTOR*)(((char*)pUKnotVec)     + ALIGN_SIZE(sizeof(gw_KNOTVECTOR)));
  gw_KNOTVECTOR * pWKnotVec = pVolumeMemory->knw  = (gw_KNOTVECTOR*)(((char*)pVKnotVec)     + ALIGN_SIZE(sizeof(gw_KNOTVECTOR)));
  pUKnotVec->U                                    = (gw_REAL*)      (((char*)pWKnotVec)     + ALIGN_SIZE(sizeof(gw_KNOTVECTOR)));
  pVKnotVec->U                                    = (gw_REAL*)      (((char*)pUKnotVec->U)  + ALIGN_SIZE(sizeof(gw_REAL) * (lUKnotsHighestIndex+1)));
  pWKnotVec->U                                    = (gw_REAL*)      (((char*)pVKnotVec->U)  + ALIGN_SIZE(sizeof(gw_REAL) * (lVKnotsHighestIndex+1)));
  gw_CPOINT     ***pPw      = pMesh->Pw           = (gw_CPOINT***)  (((char*)pWKnotVec->U)  + ALIGN_SIZE(sizeof(gw_REAL) * (lWKnotsHighestIndex+1)));
  gw_CPOINT     ** pQw                            = (gw_CPOINT**)   (((char*)pPw)           + ALIGN_SIZE(sizeof(gw_CPOINT**) * (lUCPointHighestIndex+1)));
  gw_CPOINT     *  pCP                            = (gw_CPOINT*)    (((char*)pQw)           + ALIGN_SIZE(sizeof(gw_CPOINT*)  * (lUCPointHighestIndex+1)
                                                                                                                             * (lVCPointHighestIndex+1)));                
  // set internal size parameter values
  pVolumeMemory->p = lUDegree;
  pVolumeMemory->q = lVDegree;
  pVolumeMemory->r = lWDegree;
  pUKnotVec->m     = lUKnotsHighestIndex;
  pVKnotVec->m     = lVKnotsHighestIndex;
  pWKnotVec->m     = lWKnotsHighestIndex;
  pMesh->m         = lUCPointHighestIndex;
  pMesh->n         = lVCPointHighestIndex;
  pMesh->o         = lWCPointHighestIndex;

  // Initialize pointers to gw_CPOINT arrays.
  gw_INDEX i, j, jcnt ;
  gw_INDEX l  = 0;
  gw_INDEX ll = 0;
  for(i=0,jcnt=0; i<=pMesh->m; i++ )  
    {  
      pMesh->Pw[i] = &pQw[l];  
      l            = l + pMesh->n + 1;
      for(j=0; j<=pMesh->n; j++,jcnt++)
        {
          pQw[jcnt] = &pCP[ll];
          ll     = ll + pMesh->o + 1;
        }  
    }

  // all done
  return;

} // end sm_InitNurbVolumeMemory

/*******************************************************************//**
PURPOSE: Given useful volume information, allocate the space required
    for a set of nurb volumes as a single big block.  Return a pointer
    to the block and set the output array of volumes to point to each
    individual volume.

NOTES: Internal counts, degrees and pointers of the gw_VOLUME 
    are initialized for each volume.
***********************************************************************/
char *sm_AllocateBlockOfNurbVolumes
  (gw_INDEX         lNumberOfVolumes,     // in : 
   gw_INDEX         lUCPointHighestIndex, // in : 
   gw_INDEX         lVCPointHighestIndex, // in : 
   gw_INDEX         lWCPointHighestIndex, // in : 
   gw_DEGREE        lUDegree,             // in : 
   gw_DEGREE        lVDegree,             // in : 
   gw_DEGREE        lWDegree,             // in : 
   gw_INDEX         lUKnotsHighestIndex,  // in : 
   gw_INDEX         lVKnotsHighestIndex,  // in : 
   gw_INDEX         lWKnotsHighestIndex,  // in : 
   SmTArray<void*>  & rVolumes)           // out: 
{
  // get size of 1 volume
  ULONG lTotalSize = sm_ComputeNurbVolumeSize
                   (lUCPointHighestIndex,
                    lVCPointHighestIndex, 
                    lWCPointHighestIndex, 
                    lUKnotsHighestIndex, 
                    lVKnotsHighestIndex,
                    lWKnotsHighestIndex);

  // allocate memory for all volumes
  // okay to use smos_Calloc on static class (gw_CURVE, gw_CPOLYGON, gw_KNOTVECTOR,.. ) objects.
  char* pMemBlock = (char*) smos_Calloc(lNumberOfVolumes*lTotalSize, 1);
  NERN(pMemBlock);
  
  // Now load data from input volumes
  rVolumes.SetSize(lNumberOfVolumes);
  for (long i=0; i<lNumberOfVolumes; i++) 
    {
      gw_VOLUME *pNewVol = (gw_VOLUME*)&pMemBlock[i*lTotalSize];
      rVolumes[i] = SM_REINTERPRET_CAST(SmObject*,pNewVol);
      sm_InitNurbVolumeMemory(pNewVol,
                                 lUCPointHighestIndex,
                                 lVCPointHighestIndex,
                                 lWCPointHighestIndex,
                                 lUDegree,lVDegree,lWDegree,
                                 lUKnotsHighestIndex,
                                 lVKnotsHighestIndex,
                                 lWKnotsHighestIndex);
    }

  // all done
  return pMemBlock;

} // end sm_AllocateBlockOfNurbVolumes

/*******************************************************************//**
PURPOSE: Given critical volume information allocate and init the space 
    required for a single nurb volume as a single block of memory.

NOTES: 
***********************************************************************/
gw_VOLUME * sm_AllocateNurbVolume
  (gw_INDEX  lUCPointHighestIndex,  // in : max U ControlPoint index value = U ControlPoint_count - 1 
   gw_INDEX  lVCPointHighestIndex,  // in : max V ControlPoint index value = V ControlPoint_count - 1 
   gw_INDEX  lWCPointHighestIndex,  // in : max W ControlPoint index value = W ControlPoint_count - 1 
   gw_DEGREE lUDegree,              // in : U dir degree
   gw_DEGREE lVDegree,              // in : V dir degree
   gw_DEGREE lWDegree,              // in : W dir degree
   gw_INDEX  lUKnotsHighestIndex,   // in : max U knot index value = U knot_count - 1
   gw_INDEX  lVKnotsHighestIndex,   // in : max V knot index value = V knot_count - 1
   gw_INDEX  lWKnotsHighestIndex)   // in : max W knot index value = W knot_count - 1
{ 
  // get block size for volume
  ULONG lTotalSize = sm_ComputeNurbVolumeSize(lUCPointHighestIndex, 
                                                 lVCPointHighestIndex, 
                                                 lWCPointHighestIndex, 
                                                 lUKnotsHighestIndex, 
                                                 lVKnotsHighestIndex,
                                                 lWKnotsHighestIndex);

  // allocate volume memory block
  // okay to use smos_Calloc on static class (gw_CURVE, gw_CPOLYGON, gw_KNOTVECTOR,.. ) objects.
  gw_VOLUME *pNewVol = (gw_VOLUME*) smos_Calloc(lTotalSize, 1);
  NERN(pNewVol);
  
  // init volume memory block pointers and size params - ControlPoint and knot values are not touched
  sm_InitNurbVolumeMemory(pNewVol,
                             lUCPointHighestIndex, 
                             lVCPointHighestIndex,
                             lWCPointHighestIndex,
                             lUDegree, lVDegree, lWDegree,
                             lUKnotsHighestIndex, 
                             lVKnotsHighestIndex,
                             lWKnotsHighestIndex);

  // all done
  return pNewVol;

} // end sm_AllocateNurbVolume

/*******************************************************************//**
PURPOSE: Allocate a nurb volume and copy the data from the source
    volume.  The returned new volume is a single block of memory.  
    The source could be either a single block or one of the 
    segmented things we get from NLib.

NOTES: returns NULL if input target volume has no polygon mesh,
   (the pointer, cpSrcVol->mesh == NULL)
***********************************************************************/
gw_VOLUME * sm_AllocateAndCopyNurbVolume
  (const gw_VOLUME *cpSrcVol)    // in : target volume to copy 
{ 
  // check input: has to have a mesh
  NL_CMESH *pMesh = cpSrcVol ? cpSrcVol->mesh : NULL ;
  if (!pMesh) 
    { return (NULL); }

  // volume locals
  gw_KNOTVECTOR *pUKnotVec = cpSrcVol->knu;
  gw_KNOTVECTOR *pVKnotVec = cpSrcVol->knv;
  gw_KNOTVECTOR *pWKnotVec = cpSrcVol->knw;

  // allocate single block Volume with internal pointers and size params set
  gw_VOLUME *pNewVol = sm_AllocateNurbVolume(pMesh->m, pMesh->n, pMesh->o,
                                               cpSrcVol->p, cpSrcVol->q, cpSrcVol->r,
                                               pUKnotVec->m, pVKnotVec->m, pWKnotVec->m) ;

  // copy knot values from target volume to new volume
  // okay to use smos_MemCpy on base (double) objects.
  SE(smos_MemCpy(pNewVol->knu->U,cpSrcVol->knu->U,sizeof(gw_REAL) * (pNewVol->knu->m + 1), sizeof(gw_REAL) * (pNewVol->knu->m + 1)));
  SE(smos_MemCpy(pNewVol->knv->U,cpSrcVol->knv->U,sizeof(gw_REAL) * (pNewVol->knv->m + 1), sizeof(gw_REAL) * (pNewVol->knv->m + 1)));
  SE(smos_MemCpy(pNewVol->knw->U,cpSrcVol->knw->U,sizeof(gw_REAL) * (pNewVol->knw->m + 1), sizeof(gw_REAL) * (pNewVol->knw->m + 1)));

  // copy control point values from target volume to new volume one row at a time
#ifdef SM_DEBUG_CODE
  //NL_CPOINT *pLastNew = pNewVol->mesh->Pw[0][0]  ;
  //NL_CPOINT *pLastSrc = cpSrcVol->mesh->Pw[0][0]  ;
#endif

  long i,j ;
  for(i=0; i<=pMesh->m; i++) 
  {
    for(j=0; j<=pMesh->n; j++) 
    {
      // okay to use smos_MemCpy on static class (gw_CPOINT) objects.
      SE(smos_MemCpy(pNewVol->mesh->Pw[i][j],
                  cpSrcVol->mesh->Pw[i][j],
                  sizeof(gw_CPOINT) * (pNewVol->mesh->o + 1), sizeof(gw_CPOINT) * (pNewVol->mesh->o + 1)));
#ifdef SM_DEBUG_CODE
      //pLastNew = pNewVol->mesh->Pw[i][j]  ;
      //pLastSrc = cpSrcVol->mesh->Pw[i][j]  ;
#endif
    }
  }

  // all done
  return pNewVol;

} // end sm_AllocateAndCopyNurbVolume

// USE NLIB N_VolumeGetEMesh
