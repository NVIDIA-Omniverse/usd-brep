// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*___*/
/**********************************************************************
* FILE NAME --- smcurv_test.cpp
* PURPOSE --- Test functions for various objects.
*
**********************************************************************/
/*___*/ 

#include "StdAfx.h"

#include <prog_test.h>
#include <smCurveTest.h>
#include <SmPlane.h>
#include <SmCircle.h>
#include <SmConic.h>
#include <SmParabola.h>
#include <SmHyperbola.h>
#include <SmCompositeCurve.h>
#include <SmPolynomial.h>
#include <SmLine.h>
#include <SmCubicBezierSurface.h>
#include <SmMatrix.h>
#include <SmGeomUtility.h>

/***********************************************************************
PURPOSE ---

USAGE NOTES ---
***********************************************************************/
void my_draw_2crv_sol
  (SmCurve *pCurve1, 
   SmCurve *pCurve2, 
   SmSolutionArray & rSolutions,
   SmVector3d *pOptProjVec=NULL)    // for graphics only - NULL to ignore
{
#ifdef SM_GFX_CODE
  if (smGet_DoGraphics())
    {
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
      if(bDebugMe)
        {
          pCurve1->Dump() ;
          pCurve2->Dump() ;
          rSolutions.Dump() ;
        }
#endif // SM_DEBUG_CODE
      ULONG ii, lThisSampleCnt, lSampleCnt=50 ;
      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0,0,1); pCurve1->Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook(2,3, 0,1,0); pCurve2->Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook(3,4, 0,1,0); pCurve1->DrawCrvGapFunction(*pCurve2, lSampleCnt) ; sm_GraphicsLoop() ;
      for(ii=0;ii<rSolutions.GetSize();ii++)
        { SmSolution &rSol = rSolutions[ii] ;
          SmExtent1d sIvl, sNaturalIvl = pCurve1->GetNaturalInterval() ;
          if(rSol.m_eSolutionType == SM_ST_RANGE_OF_VALUES)
            { sIvl.SetMinMax(rSol.m_vStart.m_adParameters[0],
                             rSol.m_vEnd.m_adParameters[0]) ;
              lThisSampleCnt = 10 * lSampleCnt ;
              if(pOptProjVec)
                {
                  SmPoint3d sPoint1, sPoint2 ;
                  pCurve1->EvaluatePoint(rSol.m_vStart.m_adParameters[0], sPoint1) ;
                  pCurve1->EvaluatePoint(rSol.m_vEnd.m_adParameters[0], sPoint2) ;
                  smgfx_SetLook(1,2, 0,0,0) ; (*pOptProjVec*3.0).Draw(&sPoint1) ; sm_GraphicsLoop() ;
                  smgfx_SetLook(1,2, 0,0,0) ; (*pOptProjVec*3.0).Draw(&sPoint2) ; sm_GraphicsLoop() ;
                }
            }
          else // SM_ST_SINGLE_VALUE
            {
              sIvl.SetMinMax(rSol.m_vStart.m_adParameters[0],
                             rSol.m_vStart.m_adParameters[0]) ;
              lThisSampleCnt = lSampleCnt ;
              if(pOptProjVec)
                {
                  SmPoint3d sPoint1 ;
                  pCurve1->EvaluatePoint(rSol.m_vStart.m_adParameters[0], sPoint1) ;
                  smgfx_SetLook(1,2, 0,0,0) ; (*pOptProjVec*3.0).Draw(&sPoint1) ; sm_GraphicsLoop() ;
                }
            }
          sIvl.ExpandAbsolute(sNaturalIvl.GetLength()/100.0) ;
          sIvl.Intersect(sNaturalIvl, sIvl) ;
          pCurve1->DrawCrvGapFunction(*pCurve2, lThisSampleCnt, &sIvl) ; sm_GraphicsLoop() ;
        }
      smgfx_SetLook(9,10, 1,0,0); rSolutions.Draw(); sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;
    }
#else
  SM_REF4(pCurve1, pCurve2, rSolutions, pOptProjVec);
#endif
    
} // end my_draw_2crv_sol


/***********************************************************************
PURPOSE --- Dump solution array, print solution values, 

USAGE NOTES ---
***********************************************************************/
void my_draw_crv_pnt_sol
  (SmCurve             * pCurve,         // in : 
   SmPoint3d           * pPnt,           // in : 
   SmSolutionArray     & rSolutions,     // in : 
   SmTArray<SmPoint3d> * pCombPoints,    // in : optional Comb tine base points
   SmTArray<SmPoint3d> * pCombVecs)      // in : optional Comb tine vectors
{
  ULONG i ;

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  if(bDebugMe)
    {
      rSolutions.Dump() ;
    }
#endif // SM_DEBUG_CODE

  // print solutions
  for(i = 0; i < rSolutions.GetSize(); i++)
   {
     TCHAR sBuff[SM_TBLOCK_SIZE];
     SM_SPRINTF(sBuff, _T("Value = %16.16lf\n"), rSolutions[i].m_vStart.m_dSolutionValue);
     smos_WriteBuffer(sBuff);
   }
 
#ifdef SM_GFX_CODE
  if (smGet_DoGraphics())
    {
      smgfx_SetLook(1,2, 0,1,0) ; if(pCurve) pCurve->Draw() ;
      smgfx_SetLook(2,5, 1,0,0) ; for(i = 0; i < rSolutions.GetSize(); i++)
                                    {
                                      SmPoint3d    sP1, sP2;
                                      if(pPnt) sP1 = *pPnt;
                                      pCurve->EvaluatePoint(rSolutions[i].m_vStart[0], sP2);
                                      sP2.Draw();
                                      if(pPnt) { sP1.Draw();
                                                 SmVector3d sVec = sP1 - sP2;
                                                 sVec.Draw(&sP2);
                                               }
                                    }
      // property comb
      if(pCombPoints && pCombVecs)
        {
          smgfx_SetLook(1,3, 0,1,1) ; 
          ULONG ii ;
          for(ii=0;ii<pCombPoints->GetSize();ii++) 
            {
              pCombPoints->GetAt(ii).Draw() ; 
              pCombVecs->GetAt(ii).Draw(&pCombPoints->GetAt(ii)) ; 
            }
        }
      smgfx_SetLook(1,2, 0,0,0) ; sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;
  } // end if smGet_DoGraphics()

#else
  SM_REF4(pCurve, pPnt, pCombPoints, pCombVecs);
#endif // SM_GFX_CODE
} // end my_draw_crv_pnt_sol

/***********************************************************************
PURPOSE ---  Build a circle in the xy plane starting and ending at
             3:00 o'clock, the (dRadius,0,0) point.

USAGE NOTES ---
***********************************************************************/
PT_EXPORT SmBSplineCurve * my_create_circle
(const SmContext   & crContext,  // in : 
 double              dRadius,    // in : 
 const SmPoint3d   & rCenter,    // in : 
 SmNurbCircleParam   eParam,     // in : oneof:  SM_CO_QUADRATIC,    // degree 2
                                 //              SM_CO_QUINTIC       // degree 5
 double              dRed,       // in : 
 double              dGreen,     // in : 
 double              dBlue,      // in : 
 SmBoolean           b2D)        // in : TRUE=output 2d curve, FALSE=output 3d curve, default:[FALSE]
{
  SmAxis2Placement sA2P;
  sA2P.SetCanonical(rCenter, SmVector3d(1, 0, 0), SmVector3d(0, 1, 0));
  SmBSplineCurve *pCir = NULL;
  SmBSplineCurve::CreateCircleSegment(crContext, 3, sA2P, dRadius, 0.0, 360.0, eParam, pCir);
#ifdef SM_GFX_CODE
  if (smGet_DoGraphics())
    {
      smgfx_SetLook(1,2, dRed, dGreen, dBlue); pCir->DrawWDeriv(pCir->GetNaturalInterval(), 0);
    }
#else
  SM_REF3(dRed, dGreen, dBlue);
#endif

  if(b2D) { pCir->ConvertTo2D() ; }

  return pCir;

} // end my_create_circle

/***********************************************************************
PURPOSE ---

USAGE NOTES ---
***********************************************************************/
PT_EXPORT SmBSplineCurve * my_create_ellipse
(const SmContext & crContext, 
 double            dRadius1, 
 double            dRadius2, 
 const SmPoint3d & rCenter, 
 SmNurbCircleParam eParam,
 double            dRed, 
 double            dGreen, 
 double            dBlue,
 SmBoolean         b2D)        // in : TRUE=output 2d curve, FALSE=output 3d curve, default:[FALSE]
{
  SmAxis2Placement sA2P;
  sA2P.SetCanonical(rCenter, SmVector3d(1, 0, 0), SmVector3d(0, 1, 0));
  SmBSplineCurve *pEll = NULL;
  SmBSplineCurve::CreateEllipseSegment(crContext, 3, sA2P, dRadius1, dRadius2, 0.0, 360.0, eParam, pEll);
#ifdef SM_GFX_CODE
  if (smGet_DoGraphics())
    {
      smgfx_SetColor(dRed, dGreen, dBlue);
      pEll->Draw();
    }
#else
  SM_REF3(dRed, dGreen, dBlue);
#endif

  if(b2D) { pEll->ConvertTo2D() ; }

  return pEll;

} // end my_create_ellipse

/***********************************************************************
PURPOSE ---  Build a BSPline curve for given ControlPoints - 
             when smGet_DoGraphics() == TRUE,
             add its image to the current graphics state.

USAGE NOTES ---  NaturalInterval = [0,1]
                 KnotVec         = [evanly spaced, FullyMult ends, single mult internal]

***********************************************************************/
PT_EXPORT SmBSplineCurve * my_create_nurb
 (const SmContext       & crContext,  // in : context for new geometry construction
  const SmPoint3d       & rTranslate, // in : translation applied to points prior to curve construction
  SmTArray < SmPoint3d> & rPoints,    // in : list of control point positions
  ULONG                   lDegree,    // in : desired degree of BSpline
  double                  dRed,       // in : red   of curve color - added to current image
  double                  dGreen,     // in : green of curve color   when smGet_DoGraphics() == TRUE
  double                  dBlue,      // in : blue  of curve color
  SmBoolean               b2D)        // in : TRUE=output 2d curve, FALSE=output 3d curve, default:[FALSE]
{
  // locals
  ULONG              lNumKnot = rPoints.GetSize() - lDegree + 1;
  SmTArray < ULONG>  sKnotMult(lNumKnot);
  SmTArray < double> sKnots(lNumKnot);

  // min knot value = 0.0, fully multiple
  sKnotMult.Add(lDegree + 1);
  sKnots.Add(0.0);

  // for every internal
  for (ULONG i = 1; i < lNumKnot - 1; i++)
    {
      // add evenly spaced single knots
      sKnots.Add(((double)i)/lNumKnot);
      sKnotMult.Add(1);
    } // end iter every internal knot

  // max knot value = 1.0, full multiple
  sKnotMult.Add(lDegree + 1);
  sKnots.Add(1.0);
  
  // create a list of tranlated points
  SmBSplineCurve *pNurb = NULL;
  SmTArray < SmPoint3d> sPoints;
  sPoints.SetSize(rPoints.GetSize());
  for (ULONG j = 0; j < rPoints.GetSize(); j++)
    {
      sPoints[j] = rPoints[j] + rTranslate;
    }

  // pass the knots and translated points along to BSpline Constructor
  SmBSplineCurve::CreateCanonical(crContext, 
                                  3, 
                                  lDegree, 
                                  sPoints, 
                                  SM_CF_UNSPECIFIED,
                                  sKnotMult, 
                                  sKnots, 
                                  SM_KT_UNSPECIFIED,
                                   NULL, NULL, 
                                  pNurb);
#ifdef SM_GFX_CODE
  if (smGet_DoGraphics())
    {
      smgfx_SetColor(dRed, dGreen, dBlue); pNurb->DrawWDeriv(pNurb->GetNaturalInterval(), 0);
    }
#else
  SM_REF3(dRed, dGreen, dBlue);
#endif

  if(b2D) { pNurb->ConvertTo2D() ; }


  // all done
  return pNurb;
  
} // end my_create_nurb

/***********************************************************************
PURPOSE ---

USAGE NOTES ---
***********************************************************************/
PT_EXPORT SmBSplineCurve * my_create_line
(const SmContext & crContext, 
 const SmPoint3d & rStartPt, 
 const SmPoint3d & rEndPt, 
 double            dRed, 
 double            dGreen, 
 double            dBlue,
 SmBoolean         b2D)        // in : TRUE=output 2d curve, FALSE=output 3d curve, default:[FALSE]

{
  SmBSplineCurve *pLine = NULL;
  SmBSplineCurve::CreateLineSegment(crContext, 
                                    3,
                                    rStartPt, 
                                    rEndPt, 
                                    pLine);
#ifdef SM_GFX_CODE
  if (smGet_DoGraphics())
    {
      smgfx_SetColor(dRed, dGreen, dBlue);
      pLine->DrawWDeriv(pLine->GetNaturalInterval(), 0);
    }
#else
  SM_REF3(dRed, dGreen, dBlue);
#endif

  if(b2D) { pLine->ConvertTo2D() ; }

  return pLine;
} // end my_create_line

/***********************************************************************
PURPOSE ---  Intersect two curves to a tolerance of SM_ZONE_TOL_3D.

USAGE NOTES ---
***********************************************************************/
PT_EXPORT SmStatus my_test_GlobalCC_intersect
  (SmCurve & rC1,     // in : intersecting curve 1
   SmCurve & rC2,     // in : intersecting curve 2
   ULONG & rlCount)   // out: number of solutions found
{
    SmSolutionArray sSolutions;
    SmExtent1d sIvl1 = rC1.GetNaturalInterval();
    SmExtent1d sIvl2 = rC2.GetNaturalInterval();
    // Following two lines used to test partial curve stuff
    //    sIvl1.SetMinMax(sIvl1.Evaluate(0.5),sIvl1.Evaluate(1.0));
    //    sIvl2.SetMinMax(sIvl2.Evaluate(0.25),sIvl2.Evaluate(0.75));
    
    if (FALSE)
    {
        // Test curve curve solve.
        
        //        double d = 0.2082;
        //        SER(rC1.GlobalCurveSolve(sIvl1,rC2,sIvl2,SM_SO_MINIMIZE,
        //            SM_ZONE_TOL_3D/10.0, NULL ,NULL, SM_SR_ALL, sSolutions));
    }
    else 
    {
        SER(rC1.GlobalCurveIntersect(sIvl1,
                                     rC2, sIvl2,
                                     SM_ZONE_TOL_3D, sSolutions));
    }

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  // draw 
  if(bDebugMe)
    {
      sSolutions.Dump() ;
    }
#endif // SM_DEBUG_CODE
    
#ifdef SM_DEBUG_CODE
  // draw 
  if(bDebugMe)
    {
      sSolutions.Dump() ;
      ULONG ii, lThisSampleCnt, lSampleCnt = 50 ;

      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0,0,0); rC1.Draw(); sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,0,1); rC2.Draw(); sm_GraphicsLoop() ;
      smgfx_SetLook(2,3, 0,1,0); rC1.DrawCrvGapFunction(rC2, lSampleCnt) ; sm_GraphicsLoop() ;
      for(ii=0;ii<sSolutions.GetSize();ii++)
        { SmSolution &rSol = sSolutions[ii] ;
          SmExtent1d sIvl, sNaturalIvl = rC1.GetNaturalInterval() ;
          if(rSol.m_eSolutionType == SM_ST_RANGE_OF_VALUES)
            { sIvl.SetMinMax(rSol.m_vStart.m_adParameters[0],
                             rSol.m_vEnd.m_adParameters[0]) ;
              lThisSampleCnt = 10 * lSampleCnt ;
            }
          else
            {
              sIvl.SetMinMax(rSol.m_vStart.m_adParameters[0],
                             rSol.m_vStart.m_adParameters[0]) ;
              lThisSampleCnt = lSampleCnt ;
            }
          sIvl.ExpandAbsolute(sNaturalIvl.GetLength()/100.0) ;
          sIvl.Intersect(sNaturalIvl, sIvl) ;
          rC1.DrawCrvGapFunction(rC2, lThisSampleCnt, &sIvl) ; sm_GraphicsLoop() ;
        }
      smgfx_SetLook(9,10, 1,0,0); sSolutions.Draw(); sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;

    }
#endif // SM_DEBUG_CODE && SM_GFX_CODE

#ifdef SM_GFX_CODE
  if (smGet_DoGraphics()) {
      smgfx_Erase();
      smgfx_SetLook(1, 2, 0, 0, 0); rC1.Draw();
      smgfx_SetLook(3, 4, 0, 0, 1); rC2.Draw();
      smgfx_SetLook(5, 6, 1, 0, 0); sSolutions.Draw(); sm_GraphicsLoop();
      sm_GraphicsLoop();
  }
#endif // SM_GFX_CODE

    rlCount = 0;
    for (ULONG kk = 0; kk < sSolutions.GetSize(); kk++)
    {
        if (sSolutions[kk].m_eSolutionType == SM_ST_SINGLE_VALUE)
        {
            rlCount++;
        }
        else 
        { // Must be coincident segment
            rlCount++;
        }
    }
    return SM_SUCCESS;
} // end my_test_GlobalCC_intersect


/***********************************************************************
PURPOSE ---

USAGE NOTES ---
***********************************************************************/
SmStatus my_test_drop_point
(const SmBSplineCurve & crCurve, 
   const SmExtent1d & crInterval,
   ULONG lNumPoints)
{
    ULONG lStartEval = smcurv_GetCurveEvalCount();
    SM_REF1(lStartEval) ;
    SmSolutionArray sSolutions;
    for (ULONG i = 0; i < lNumPoints; i++)
    {
        SmPoint3d sTestPnt;
        double dT = crInterval.Evaluate(((double)i)/(lNumPoints - 1));
        crCurve.EvaluatePoint(dT, sTestPnt);
        
        SER(crCurve.GlobalPointSolve(crCurve.GetNaturalInterval(),
            SM_SO_INTERSECT, sTestPnt, SM_ZONE_TOL_3D, NULL, NULL,
            SM_SR_ALL, sSolutions));
        
        if (sSolutions.GetSize() == 0)
            SER(SM_ERR);
        
#ifdef SM_GFX_CODE
        // if (smGet_DoGraphics())
        // {
        //     for (ULONG kk = 0; kk < sSolutions.GetSize(); kk++)
        //     {
        //         SmPoint3d sFndPnt;
        //         SER(crCurve.EvaluatePoint(sSolutions[kk].m_vStart[0], sFndPnt));
        //         sFndPnt.Draw();
        //     }
        // }
#endif 
    } // for
    
    return SM_SUCCESS;
} // end my_test_drop_point


/***********************************************************************
PURPOSE ---

USAGE NOTES ---
***********************************************************************/
SmStatus my_test_point_extrema
(const SmBSplineCurve & crCurve, 
   const SmExtent1d & crInterval,
   SmSolverOperationType eSolverOperation,
   long lNumX,
   long lNumY,
   long lNumZ)
{
    SmExtent3d sBBox;
    SmSolutionArray sSolutions;
    
    SER(crCurve.CalculateBoundingBox(crInterval, &sBBox, NULL));
    double dFindDistance =(sBBox.GetMax().DistanceBetween(sBBox.GetMin()))/10.0;
    for (long i = 0; i < lNumX; i++)
    {
        double dX =(sBBox.GetMax().x - sBBox.GetMin().x) /(lNumX - 1 + SM_EFF_ZERO);
        if (dX < SM_EFF_ZERO)
            i = lNumX;
        if (lNumX < 2)
            dX = 0.0;
        for (long j = 0; j < lNumY; j++)
        {
            double dY =(sBBox.GetMax().y - sBBox.GetMin().y) /(lNumY - 1 + SM_EFF_ZERO);
            if (dY < SM_EFF_ZERO)
                j = lNumY;
            if (lNumY < 2)
                dY = 0.0;
            for (long k = 0; k < lNumZ; k++)
            {
                double dZ =(sBBox.GetMax().z - sBBox.GetMin().z) /(lNumZ - 1 + SM_EFF_ZERO);
                if (dZ < SM_EFF_ZERO)
                    k = lNumZ;
                if (lNumZ < 2)
                    dZ = 0.0;
                SmPoint3d sClPt(sBBox.GetMin().x + i * dX,
                    sBBox.GetMin().y + j * dY,
                    sBBox.GetMin().z + k * dZ);
                
                SmExtent1d sIvl1 = crCurve.GetNaturalInterval();
                // Following line used to test partial curve stuff
                //                sIvl1.SetMinMax(sIvl1.Evaluate(0.5),sIvl1.Evaluate(1.0));
                SER(crCurve.GlobalPointSolve(sIvl1,
                    eSolverOperation, sClPt, SM_ZONE_TOL_3D, &dFindDistance, NULL,
                    SM_SR_ALL, sSolutions));
                
#ifdef SM_GFX_CODE
                if (smGet_DoGraphics())
                {
                    for (ULONG kk = 0; kk < sSolutions.GetSize(); kk++)
                    {
                        SmPoint3d sFndPnt;
                        SER(crCurve.EvaluatePoint(sSolutions[kk].m_vStart[0], sFndPnt));
                        SmVector3d sTo = sFndPnt - sClPt;

                        if(kk==0) { smgfx_Erase() ;
                                    smgfx_SetLook(1,2, 0,0,1) ; crCurve.Draw(&sIvl1) ; sm_GraphicsLoop() ;
                                    smgfx_SetLook(3,4, 1,0,0) ; sClPt.Draw() ; sm_GraphicsLoop() ;
                                  }
                        smgfx_SetLook(3,4, 0,1,0) ; sFndPnt.Draw() ; sm_GraphicsLoop() ;
                        smgfx_SetLook(1,2, 0,0,0) ; sTo.Draw(&sClPt) ; sm_GraphicsLoop() ;
                        sm_GraphicsLoop() ;
                    }
                }
#endif
            }
        }
    }
    
    return SM_SUCCESS;
} // end my_test_point_extrema

// This static routine is never referenced anywhere
/*
static SmStatus my_test_CC_extrema(SmBSplineCurve & rC1, SmBSplineCurve & rC2,
                                   SmSolverOperationType eDistExtr) 
                               {
                               SmExtent3d sBBox;
                               SER(rC1.CalculateBoundingBox(rC1.GetNaturalInterval(), &sBBox, NULL));
                               double dFindDistance =(sBBox.GetMax().DistanceBetween(sBBox.GetMin()))/2.0;
                               ULONG lNumFails = 0;
                               for (ULONG i = 0; i <= 10; i++)
                               {
                               double dGuess1 = i/10.0;
                               for (ULONG j = 0; j <= 10; j++)
                               {
                               double dGuess2 = j/10.0;
                               SmSolution sSolution;
                               SmBoolean bFoundAnswer;
                               if (rC1.LocalCurveSolve(rC1.GetNaturalInterval(),
                               rC2, rC2.GetNaturalInterval(),
                               eDistExtr, SM_ZONE_TOL_3D, &dFindDistance, NULL, dGuess1, dGuess2, 
                               bFoundAnswer, sSolution) != SM_SUCCESS)
                               {
                               lNumFails ++;
                               continue;
                               }
                               
                                 #ifdef SM_GFX_CODE
                                 if (smGet_DoGraphics() && bFoundAnswer)
                                 {
                                 smgfx_SetColor(0.5, 0.5, 0);
                                 SmPoint3d sP1, sP2;
                                 rC1.EvaluatePoint(sSolution.m_vStart[0], sP1);
                                 rC2.EvaluatePoint(sSolution.m_vStart[1], sP2);
                                 sP1.Draw();
                                 sP2.Draw();
                                 SmVector3d sVec = sP1 - sP2;
                                 sVec.Draw(&sP2);
                                 smgfx_SetColor(0, 0, 0);
                                 }
                                 #endif
                                 }
                                 }
                                 TCHAR sBuff[256];
                                 SM_SPRINTF(sBuff, _T("Number CC closest Failures = %ld\n"), lNumFails);
                                 smos_WriteBuffer(sBuff);
                                 return SM_SUCCESS;
                                 }
*/

/***********************************************************************
PURPOSE ---

USAGE NOTES ---
***********************************************************************/
static SmStatus my_test_GlobalCC_extrema
  (SmBSplineCurve & rC1,                // in : 
   SmBSplineCurve & rC2,                // in : 
   SmSolverOperationType eDistExtr,     // in : 
   ULONG & rlNumFound)                  // out: 
{
  // for SM_SO_AT_DISTANCE - select pdDist = BBOxSize/4.0
  SmExtent3d sBBox;
  SER(rC1.CalculateBoundingBox(rC1.GetNaturalInterval(), &sBBox, NULL));

  // size of bounding box
  double dFindDistance = (sBBox.GetMax().DistanceBetween(sBBox.GetMin()));
  double dScFindDist = dFindDistance/4.0;
  double * pdDist = NULL;
  if (eDistExtr == SM_SO_AT_DISTANCE)
    {
      pdDist = &dScFindDist;
    }
  
  rlNumFound = 0;
  ULONG lEnd = 1;
   
  for (ULONG i = 0; i < lEnd; i++)
    {
      SmSolutionArray sSolutions;
      SmExtent1d sIvl1 = rC1.GetNaturalInterval();
      SmExtent1d sIvl2 = rC2.GetNaturalInterval();
      // Following two lines used to test partial curve stuff
      //        sIvl1.SetMinMax(sIvl1.Evaluate(0.5),sIvl1.Evaluate(1.0));
      //        sIvl2.SetMinMax(sIvl2.Evaluate(0.25),sIvl2.Evaluate(0.75));
      SER(rC1.GlobalCurveSolve(sIvl1,
                               rC2, sIvl2,
                               eDistExtr, SM_ZONE_TOL_3D, pdDist, NULL,
                               SM_SR_ALL, sSolutions));
      
      rlNumFound += sSolutions.GetSize();
      
#ifdef SM_GFX_CODE
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = 0 ;
      if(bDebugMe)
        {
          sSolutions.Dump() ;
        }
#endif // SM_DEBUG_CODE

      if (smGet_DoGraphics())
        {
          for (ULONG kk = 0; kk < sSolutions.GetSize(); kk++)
            {
              SmSolution sSol = sSolutions[kk];

              SmPoint3d sP1, sP2;
              rC1.EvaluatePoint(sSol.m_vStart[0], sP1);
              rC2.EvaluatePoint(sSol.m_vStart[1], sP2);

              if(kk==0) { smgfx_Erase() ;
                          smgfx_SetLook(1,2, 0,1,0) ; rC1.Draw() ; sm_GraphicsLoop() ;
                          smgfx_SetLook(1,2, 0,0,1) ; rC2.Draw() ; sm_GraphicsLoop() ;
                        }
              smgfx_SetLook(1,2, 0.2, 0, 0.3); sP1.DrawPointToPoint(sP2); sm_GraphicsLoop() ;
              sm_GraphicsLoop() ;
            }
        }
#endif
    } // for

  return SM_SUCCESS;
} // end my_test_GlobalCC_extrema

// This is never used or referenced anywhere
/*
static SmStatus my_test_CC_intersect(SmBSplineCurve & rC1, SmBSplineCurve & rC2)
{
ULONG lNumFails = 0;
for (ULONG i = 0; i <= 10; i++)
{
double dGuess1 = i/10.0;
for (ULONG j = 0; j <= 10; j++)
{
double dGuess2 = j/10.0;
double dParam1, dParam2, dDeviation;
SmBoolean bFoundTsect;
if (rC1.LocalCurveIntersect(rC1.GetNaturalInterval(),
rC2, rC2.GetNaturalInterval(),
SM_ZONE_TOL_3D, dGuess1, dGuess2,
bFoundTsect, dParam1, dParam2, dDeviation) != SM_SUCCESS)
{
lNumFails ++;
continue;
}

#ifdef SM_GFX_CODE
if (smGet_DoGraphics())
{
smgfx_SetColor(1.0, 0.0, 0);
SmPoint3d sP1, sP2;
rC1.EvaluatePoint(dParam1, sP1);
rC2.EvaluatePoint(dParam2, sP2);
sP1.Draw();
sP2.Draw();
SmVector3d sTo = sP1 - sP2;
if (sTo.LengthSquared() > SM_EFF_ZERO)
sTo.Draw(&sP2);
}
#endif
}
}
TCHAR sBuff[256];
SM_SPRINTF(sBuff, _T("Number CC intersect Failures = %ld\n"), lNumFails);
smos_WriteBuffer(sBuff);
return SM_SUCCESS;
}
*/

/***********************************************************************
PURPOSE ---

USAGE NOTES ---
***********************************************************************/
PT_EXPORT SmStatus my_test_object(void)
{
  SmContext sContext;

  SmObject sObj1;  //##test empty c'tor
  sObj1.SetContext( &sContext );  //##test SetContext()

  SmObject * pObj2 = new( sContext ) SmObject;  //##test 'new' with context
//  SmObject * pObj3 = new( ) SmObject;  // syntax error ')'
//  SmObject * pObj4 = new SmObject;  // cannot access private member

  const SmContext *pContext1 = sObj1.GetContext();  //##test GetContext()
  const SmContext *pContext2 = pObj2->GetContext();

  if ( pContext1 != pContext2 ) { SER(SM_ERR); }

  // Dump methods:
  ULONG lDumpNum = 78;
  sObj1.Dump();
  sObj1.Dump( _T(" Test SmObject::Dump() with string:") );
  sObj1.Dump( FALSE );
  sObj1.Dump( lDumpNum );

  delete pObj2; //cbi: ( pContext2 ): won't compile with any arguments anywhere

  return SM_SUCCESS;

} // end my_test_object

/***********************************************************************
PURPOSE ---

USAGE NOTES ---
***********************************************************************/
PT_EXPORT SmStatus my_test_pseudobox()
{
    SmAxis2Placement sAP;
    sAP.RotateAboutAxis(SM_PI/4.0, SmVector3d(1.0, 0.0, 0.0));
    sAP.RotateAboutAxis(SM_PI/4.0, SmVector3d(0.0, 1.0, 0.0));
    
    SmPseudoBox sPB;
    sPB.SetBasis(sAP.GetXAxis(), sAP.GetYAxis(), sAP.GetZAxis());
#ifdef SM_GFX_CODE
    if (smGet_DoGraphics())
    {
        smgfx_SetColor(1.0, 0, 0);
        smgfx_Open(smgfx_GetRuleColor());
    }
#endif
    sPB.AddPoint3d(SmPoint3d(0.3, 0.2, 0.1));
#ifdef SM_GFX_CODE
    if (smGet_DoGraphics())
        smgfx_DrawPoint(0.3, 0.2, 0.1);
#endif
    sPB.AddPoint3d(SmPoint3d(0.2, 0.3, 0.4));
#ifdef SM_GFX_CODE
    if (smGet_DoGraphics())
        smgfx_DrawPoint(0.2, 0.3, 0.4);
#endif
    sPB.AddPoint3d(SmPoint3d(0.25, 0.4, 0.3));
#ifdef SM_GFX_CODE
    if (smGet_DoGraphics())
    {
        smgfx_DrawPoint(0.25, 0.4, 0.3);
        smgfx_SetColor(0, 0, 0);
        smgfx_Close();
    }
#endif
    
#ifdef SM_GFX_CODE
    if (smGet_DoGraphics())
        sPB.Draw();
#endif
    double dDist = sPB.DistanceToPoint(SmPoint3d(0.25, 0.3, 0.2));
    if (dDist > SM_EFF_ZERO)
        SER(SM_ERR);
    dDist = sPB.DistanceToPoint(SmPoint3d(6.0, 3.0, 0.5));
    if (dDist < SM_EFF_ZERO)
        SER(SM_ERR);
    
    SmPseudoBox sPB2;
    sPB2.AddPoint3d(SmPoint3d(0.8, 0.2, 0.3));
    sPB2.AddPoint3d(SmPoint3d(0.9, 0.3, 0.4));
#ifdef SM_GFX_CODE
    if (smGet_DoGraphics())
        sPB2.Draw();
#endif
    if (!sPB.AreDisjoint(sPB2))
        SER(SM_ERR);
    sPB2.AddPoint3d(SmPoint3d(0.25, 0.4, 0.3));
    if (sPB.AreDisjoint(sPB2))
        SER(SM_ERR);
    
    return SM_SUCCESS;
} // end my_test_pseudobox

/***********************************************************************
PURPOSE --- Unit tests for classes SmSolution and SmSolutionArray

USAGE NOTES ---
   These tests are quite comprehensive in that they call pretty much
   every method, but they are not all functionally exhaustive, testing
   all behaviors, they just test that the methods are being called
   correctly.  They were added for the Java project, for which we
   need a wrapper for every method.
***********************************************************************/

// Two static helpers for my_test_solution_array:
//
// which: 1 creates curve from (0,0,0) to (8,4,0), 2 makes (8,4,0) to 16,8,0).
static SmBSplineCurve * create_curve( const SmContext & crContext, int which )
{
  SmBSplineCurve *pCrv = NULL;
  SmTArray< SmPoint3d > sPoints;
  if ( which == 1 ) {
      sPoints.Add( SmPoint3d(  0, 0, 0 ));
      sPoints.Add( SmPoint3d(  4, 0, 0 ));
      sPoints.Add( SmPoint3d(  4, 4, 0 ));
      sPoints.Add( SmPoint3d(  8, 4, 0 ));
  } else {
      sPoints.Add( SmPoint3d(  8, 4, 0 ));
      sPoints.Add( SmPoint3d( 12, 4, 0 ));
      sPoints.Add( SmPoint3d( 12, 8, 0 ));
      sPoints.Add( SmPoint3d( 16, 8, 0 ));
  }

  SmInterpolationType ePz = SM_IT_CHORDLENGTH;  // enum: in SmCurveTypes.h

  SmBSplineCurve::InterpolatePoints( crContext, sPoints, NULL, 3, NULL, NULL, FALSE, ePz, pCrv );

  return pCrv;
}
static SmStatus create_solution_array(
        const SmContext & crContext,
        int which,    // in: 1 or 2.  1 creates sol pt (4,2,0), 2 makes (7,5,0).
        SmBSplineCurve *& pCrv,        // out: the object in the Solution.
        SmSolutionArray & solArr )
{
  pCrv = create_curve( crContext, which );
  SmExtent1d sDomain = pCrv->GetNaturalInterval();
  SmPoint3d sPt;
  double dParam = sDomain.GetMid();
  pCrv->EvaluatePoint( dParam, sPt );

  SmSolverOperationType   eOpType  = SM_SO_MINIMIZE;
  SmSolutionRequestedType eSolType = SM_SR_ALL;

  pCrv->GlobalPointSolve(sDomain, eOpType, sPt, 0.001, NULL, NULL, eSolType, solArr );

  if ( solArr.GetSize() != 1 )
    { SER( SM_ERR ); }

  return SM_SUCCESS;
}


PT_EXPORT SmStatus my_test_solution(void)
{

  // Scope to separate SmSolution and SmSolutionArray
{

//##test constructor
  // Scope to invoke destructor:
  {
    SmSolution sol1;

    if ( sol1.m_eSolutionType != SM_ST_UNKNOWN )
      { SER( SM_ERR ); }

//## test destructor:  leave scope.
  }

  // Create and populate an SmSolutionArray.
  SmContext sContext;
  SmBSplineCurve *pCrv = NULL;
  SmSolutionArray solArr;
  SmStatus eStat = create_solution_array( sContext, 1, pCrv, solArr );
  if ( eStat != SM_SUCCESS )
    { SER( SM_ERR ); }

//## test operator=()
  SmSolution sol1 = solArr[0];
  SmSolution sol2;

  sol2 = sol1;
  if ( sol2.m_eSolutionType != SM_ST_SINGLE_VALUE )
    { SER( SM_ERR ); }

//##test ClassifySolution
  // Equal solutions, should return flag 0.
  long flag = sol1.ClassifySolutionEnd( sol2, TRUE );
  if ( flag != 0 )
    { SER( SM_ERR ); }

//##test GetIndex
  ULONG long1, long2;
  eStat = sol1.GetIndex( pCrv, TRUE, long1, long2 );
  if ( eStat != SM_SUCCESS )
    { SER( SM_ERR ); }
  if ( long1 != 0 )
    { SER( SM_ERR ); }
  if ( long2 != 0 )
    { SER( SM_ERR ); }



//##test GetPoint( index ), with both default arguments
  SmPoint3d sParams, sPoint;
  double dTol = 1.0e-8;
  eStat = sol1.GetPoint( 0, sPoint, &sParams, TRUE );
  if ( eStat != SM_SUCCESS )
    { SER( SM_ERR ); }
  if ( fabs( sParams.x - 0.5 ) > dTol )
    { SER( SM_ERR ); }

//##test GetPoint( index ), with one default argument
  eStat = sol1.GetPoint( 0, sPoint, &sParams );
  if ( eStat != SM_SUCCESS )
    { SER( SM_ERR ); }
  if ( fabs( sParams.x - 0.5 ) > dTol )
    { SER( SM_ERR ); }

//##test GetPoint( index ), with no default arguments
  eStat = sol1.GetPoint( 0, sPoint );
  if ( eStat != SM_SUCCESS )
    { SER( SM_ERR ); }
  if ( fabs( sPoint.x - 4.0 ) > dTol )
    { SER( SM_ERR ); }



//##test GetPoint( object ), with both default arguments
  eStat = sol1.GetPoint( pCrv, TRUE, sPoint, &sParams, TRUE );
  if ( eStat != SM_SUCCESS )
    { SER( SM_ERR ); }
  if ( fabs( sParams.x - 0.5 ) > dTol )
    { SER( SM_ERR ); }

//##test GetPoint( object ), with one default argument
  eStat = sol1.GetPoint( pCrv, TRUE, sPoint, &sParams );
  if ( eStat != SM_SUCCESS )
    { SER( SM_ERR ); }
  if ( fabs( sParams.x - 0.5 ) > dTol )
    { SER( SM_ERR ); }

//##test GetPoint( object ), with no default arguments
  eStat = sol1.GetPoint( pCrv, TRUE, sPoint );
  if ( eStat != SM_SUCCESS )
    { SER( SM_ERR ); }
  if ( fabs( sParams.x - 0.5 ) > dTol )
    { SER( SM_ERR ); }


//##test GetObjectIndexForParameterIndex
  eStat = sol1.GetObjectIndexForParameterIndex( 0, long1, long2 );
  if ( long1 != 0 )
    { SER( SM_ERR ); }
  if ( long2 != 0 )
    { SER( SM_ERR ); }

//##test GetParameterInterval
  SmExtent1d sExtent;
  eStat = sol1.GetParameterInterval( 0, sExtent );
  if ( sExtent.GetMin() != 0.0 )
    { SER( SM_ERR ); }
  if ( sExtent.GetMax() != 1.0 )
    { SER( SM_ERR ); }

//##test GetObjectTolerance
  dTol = sol1.GetObjectTolerance( 0 );
  // With the new tolerance model, even geometry can have tol.
  // In  that case, it should be >= its Context's tol.
  SmObject *pObj = sol1.m_apObjects[0];
  if ( pObj != NULL && pObj->GetContext() != NULL )
  {
      if ( dTol < SmTol::GetZoneTol3d( ( pObj->GetContext() )) )
        { SER( SM_ERR ); }
  }
  else if ( dTol != 0.0 ) // 0 otherwise
    { SER( SM_ERR ); }

//##test IsEqual
  SmBoolean bOk = sol1.IsEqual( sol2 );
  if ( ! bOk )
    { SER( SM_ERR ); }

//##test Draw
#ifdef SM_GFX_CODE
  if(smGet_DoGraphics()){
      sol1.Draw();  // can test only with graphics
    }
#endif

//##test Dump
  sol1.Dump();

  delete pCrv; pCrv=NULL;

} // End scope for SmSolution.

{ // Start scope for SmSolutionArray

//##test constructor
  // Scope to invoke destructor:
  {
    SmSolutionArray sol1;

    if ( sol1.GetSize() != 0 )
      { SER( SM_ERR ); }

//## test destructor:  leave scope.
  }

  // Create and populate an SmSolutionArray.
  SmContext sContext;
  SmTArray<SmBSplineCurve*> sCurves;
  SmObjsDelete<SmBSplineCurve*> sCleanC( &sCurves );
  SmBSplineCurve *pCrv = NULL;
  SmSolutionArray solArr;
  SmStatus eStat = create_solution_array( sContext, 1, pCrv, solArr );
  sCurves.Add( pCrv );
  if ( eStat != SM_SUCCESS )
    { SER( SM_ERR ); }

//##test GetSize
  ULONG long1 = solArr.GetSize();
  if ( long1 != 1 )
    { SER( SM_ERR ); }

//##test GetDataSize
  long1 = solArr.GetDataSize();
  if ( long1 != 4 ) // The initial capacity is 4.
    { SER( SM_ERR ); }

//##test SetSize
  solArr.SetSize( 3 );
  long1 = solArr.GetSize();
  if ( long1 != 3 )
    { SER( SM_ERR ); }

//##test SetDataSize
  solArr.SetDataSize( 20 );
  long1 = solArr.GetDataSize();
  if ( long1 != 20 )
    { SER( SM_ERR ); }

//##test SetSizeValue
  solArr.SetSizeValue( 5 );
  long1 = solArr.GetSize();
  if ( long1 != 5 )
    { SER( SM_ERR ); }

//##test ReSet
  solArr.ReSet();
  long1 = solArr.GetSize();
  if ( long1 != 0 )
    { SER( SM_ERR ); }

//##test RemoveAll
  // Need a new SolutionArray
  create_solution_array( sContext, 1, pCrv, solArr );
  sCurves.Add( pCrv );
  solArr.RemoveAll();
  long1 = solArr.GetSize();
  if ( long1 != 0 )
    { SER( SM_ERR ); }

//##test GetAt
  // Need a new SolutionArray
  create_solution_array( sContext, 1, pCrv, solArr );
  sCurves.Add( pCrv );
  SmSolution sol1 = solArr.GetAt( 0 );

  SmPoint3d sPoint;
  double dTol = 1.0e-8;
  eStat = sol1.GetPoint( 0, sPoint );
  if ( eStat != SM_SUCCESS )
    { SER( SM_ERR ); }
  if ( fabs( sPoint.x - 4.0 ) > dTol )
    { SER( SM_ERR ); }

//##test SetAt
  solArr.SetSize( 3 );
  solArr.SetAt( 1, sol1 );

  sol1 = solArr.GetAt( 1 );
  sPoint.SetUninitialized();
  eStat = sol1.GetPoint( 0, sPoint );
  if ( eStat != SM_SUCCESS )
    { SER( SM_ERR ); }
  if ( fabs( sPoint.x - 4.0 ) > dTol )
    { SER( SM_ERR ); }

//##test GetDataArray
  // NOT USED: This is just a holdover from SmTArray, and is being removed.
  // solArr.GetDataArray()

//##test operator[]
  solArr.SetSize( 3 );
  solArr.SetAt( 2, sol1 );
  SmSolution sol2 = solArr[ 2 ];
  sPoint.SetUninitialized();
  eStat = sol2.GetPoint( 0, sPoint );
  if ( fabs( sPoint.x - 4.0 ) > dTol )
    { SER( SM_ERR ); }

//##test Add
  // reinitialize solArr
  solArr.ReSet();
  create_solution_array( sContext, 1, pCrv, solArr );
  sCurves.Add( pCrv );
  if ( solArr.GetSize() != 1 )
    { SER( SM_ERR ); }
  sol1 = solArr.GetAt( 0 );

  solArr.Add( sol1 );

  if ( solArr.GetSize() != 2 )
    { SER( SM_ERR ); }

//##test Append
  SmSolutionArray solArr2;
  create_solution_array( sContext, 1, pCrv, solArr2 );
  sCurves.Add( pCrv );

  // solArr has size 2.
  solArr.Append( solArr2 );

  if ( solArr.GetSize() != 3 )
    { SER( SM_ERR ); }

//##test AddSortedSolution
  // It won't add a duplicate solution, so get a different one.
  solArr2.ReSet();
  create_solution_array( sContext, 2, pCrv, solArr2 );
  sCurves.Add( pCrv );
  sol2 = solArr2.GetAt( 0 );
  solArr.AddSortedSolution( sol2, SM_SK_BY_FIRST_PARAMETER );

  if ( solArr.GetSize() != 4 ) // solArr was size 3.
    { SER( SM_ERR ); }

//##test Copy
  solArr2.Copy( solArr );
  if ( solArr2.GetSize() != 4 ) // solArr was size 4.
    { SER( SM_ERR ); }

//##test InsertAt( Solution ), with default argument
  // solArr has size 3.
  solArr.InsertAt( 1, sol1, 3 );  // insert 3 times at position 1.

  if ( solArr.GetSize() != 7 ) // solArr was size 4.
    { SER( SM_ERR ); }

//##test InsertAt( Solution ), no default argument
  // solArr has size 6.
  solArr.InsertAt( 2, sol1 );  // insert once (default) at position 2.

  if ( solArr.GetSize() != 8 ) // solArr was size 7.
    { SER( SM_ERR ); }

//##test InsertAt( SolutionArray )
  // solArr has size 8.
  solArr2.Copy( solArr );
  solArr.InsertAt( 3, solArr2 );

  if ( solArr.GetSize() != 16 ) // solArr was size 8.
    { SER( SM_ERR ); }

//##test RemoveAt, with default argument
  solArr.RemoveAt( 3, 6 );

  if ( solArr.GetSize() != 10 ) // solArr was size 16.
    { SER( SM_ERR ); }

//##test RemoveAt, no default argument
  solArr.RemoveAt( 4 );

  if ( solArr.GetSize() != 9 ) // solArr was size 10.
    { SER( SM_ERR ); }

//##test swap.
  // For this we need two different SmSolutions, to check.
  solArr.ReSet();
  solArr2.ReSet();
  create_solution_array( sContext, 1, pCrv, solArr );
  sCurves.Add( pCrv );
  create_solution_array( sContext, 2, pCrv, solArr2 );
  sCurves.Add( pCrv );
  sol1 = solArr2.GetAt( 0 );
  solArr.Add( sol1 );
  // Now solArr has two different Solutions.
  if ( solArr.GetSize() != 2 )
    { SER( SM_ERR ); }
  solArr.Swap( 0, 1 );

  sPoint.SetUninitialized();
  sol1 = solArr.GetAt( 0 );
  eStat = sol1.GetPoint( 0, sPoint );
  if ( fabs( sPoint.x - 12.0 ) > dTol )
    { SER( SM_ERR ); }

//##tests AssertValid
  SmBoolean bOk = solArr.AssertValid();
  if ( ! bOk )
    { SER( SM_ERR ); }

} // End scope for SmSolutionArray.

  return SM_SUCCESS;
}

/***********************************************************************
PURPOSE ---

USAGE NOTES ---
***********************************************************************/
PT_EXPORT SmStatus my_test_array(void)
{
    SmObject obj1, obj2, obj3;
    //    SmContext sContext(pPool);
    SmContext sContext;
    SmTArray<void*> *pPoolArr = new(sContext) SmTArray < void*>(sContext);
    pPoolArr->Add(&obj1);
    pPoolArr->Add(&obj2);
    pPoolArr->Add(&obj3);
    SM_ASSERT(pPoolArr != NULL);
    delete pPoolArr;
    pPoolArr = NULL;
    
    SmTArray<void*> *pArray = new(sContext) SmTArray < void*>(sContext);
    for (long i = 0; i < 100; i++)
    {
        pArray->Add(pArray);    
    }
    
    if (pArray->GetSize() != 100)
        SER(SM_ERR);
    
    pArray->SetSize(20);
    pArray->SetDataSize(64);
    if (pArray->GetSize() != 20)
        SER(SM_ERR);
    if (pArray->GetDataSize() != 64)
        SER(SM_ERR);
    pArray->SetDataSize(10);
    if (pArray->GetSize() != 10)
        SER(SM_ERR);
    if (pArray->GetDataSize() != 10)
        SER(SM_ERR);
    pArray->SetDataSize(20);
    if (pArray->GetSize() != 10)
        SER(SM_ERR);
    if (pArray->GetDataSize() != 20)
        SER(SM_ERR);
    
    
    
    pArray->ReSet();
    if (pArray->GetSize() != 0)
        SER(SM_ERR);
    if (pArray->GetDataSize() != 20)
        SER(SM_ERR);
    
    pArray->Add(pArray);
    pArray->RemoveAll();
    if (pArray->GetSize() != 0)
        SER(SM_ERR);
    if (pArray->GetDataSize() != 0)
        SER(SM_ERR);
    
    SmTArray < void*> sArray;
    sArray.Add(NULL);
    sArray.Add(NULL);
    pArray->Add(pArray);
    pArray->Add(pArray);
    pArray->InsertAt(1, &sArray, 2);
    pArray->InsertAt(3, &sArray);
    pArray->RemoveAt(4, 1);
    ULONG nFoundIndex;
    if (!pArray->FindElement(&sArray, nFoundIndex))
    {
        SER(SM_ERR);
    }
    if (pArray->AddUnique(&sArray))
    {
        SER(SM_ERR_UNKNOWN);
    }
    SmTArray < void*> sArray2;
    if (!pArray->AddUnique(&sArray2))
    {
        SER(SM_ERR_UNKNOWN);
    }
    
    SmTArray<void*> *pArray2 = new(sContext) SmTArray < void*>(sContext);
    pArray2->Add(pArray2);
    
    pArray->Append(*pArray2);
    pArray->InsertAt(2, pArray2);
    
    SM_ASSERT(pArray != NULL);
    delete pArray;
    pArray = NULL;
    SM_ASSERT(pArray2 != NULL);
    delete pArray2;
    pArray2 = NULL;


  //##test c'tor with 3 args (no defaults):
  double pData[10];
  pData[0] = 17;
  SmTArray< double > da1( 10, pData, 1 );
  SmTArray< double > *pda1 = new ( &sContext ) SmTArray< double> ( 10, pData, 1 );
  delete pda1; pda1 = NULL;

  if ( da1.GetSize()     !=  1 ) { SER( SM_ERR ); }
  if ( da1.GetDataSize()  < 10 ) { SER( SM_ERR ); }
  if ( da1[0] != 17.0 ) { SER( SM_ERR ); }

  //##test c'tor with 2 args, 1 default:
  double pData2[10];
  pData2[1] = 21.0;
  SmTArray< double > da2( 10, pData2 );
  if ( da2.GetSize()     >  0 ) { SER( SM_ERR ); }
  if ( da2.GetDataSize() < 10 ) { SER( SM_ERR ); }
  if ( da2.GetDataArray()[1] != 21.0 )         { SER( SM_ERR ); }

  //##test c'tor with 1 arg, 2 defaults:
  SmTArray< double > da3( 15 );
  if ( da3.GetDataSize() < 15 ) { SER( SM_ERR ); }

  //##test c'tor with 0 args, all defaults:
  SmTArray< double > da4;
  if ( da4.GetSize() != 0 ) { SER( SM_ERR ); }

  //##test c'tor that takes SmContext:
  SmTArray<double> da5(sContext);

  //##test Add() : need this for other tests.
  da2.Add( 6.0 );
  da2.Add( 12.0 );
  if ( da2.GetSize() != 2 ) { SER( SM_ERR ); }
  if ( da2[1] != 12.0 )     { SER( SM_ERR ); }

  //##test copy c'tor
  // initial values (above):  SmTArray< double > da2( 10, pData2 );
  SmTArray<double> da6( da2 );
  if ( da6.GetSize()     !=  2 ) { SER( SM_ERR ); }
  if ( da6[1] != 12.0 )          { SER( SM_ERR ); }

  //##test assignment operator=
  // initial values (above):  SmTArray< double > da2( 10, pData2 );
  da4 = da2;
  if ( da4.GetSize()   != 2 ) { SER( SM_ERR ); }
  if ( da4[1] != 12.0 )       { SER( SM_ERR ); }

  //##test SetAll()
  da4.SetAll( 76.0 );
  if ( da4.GetSize()   != 2 ) { SER( SM_ERR ); }
  if ( da4[0] != 76.0 )       { SER( SM_ERR ); }
  if ( da4[1] != 76.0 )       { SER( SM_ERR ); }

  //## test SetSize()
  da4.SetSize( 4 );
  if ( da4.GetSize() != 4 ) { SER( SM_ERR ); }
  if ( da4[1] != 76.0 )       { SER( SM_ERR ); }

  //## test SetDataSize()
  da4.SetSize( 20 );
  if ( da4.GetSize() != 20 ) { SER( SM_ERR ); }
  if ( da4[1] != 76.0 )      { SER( SM_ERR ); }
  if ( da4[2] !=  0.0 )      { SER( SM_ERR ); }

  //## test ReSet()
  da4.ReSet();
  if ( da4.GetSize() != 0 ) { SER( SM_ERR ); }
  double *pData3 = da4.GetDataArray();        //## test GetDataArray()
  if ( pData3 == NULL )  { SER( SM_ERR ); }  // should not set to Null

  //## test RemoveAll() and GetDataArray()
  da4.RemoveAll();
  if ( da4.GetSize() != 0 ) { SER( SM_ERR ); }
  pData3 = da4.GetDataArray();
  if ( pData3 != NULL )  { SER( SM_ERR ); }  // should set to Null

  //## test SetArray()
  da4.SetArray( 10, pData, 4 );
  if ( da4.GetSize()      !=  4 )    { SER( SM_ERR ); }
  if ( da4.GetDataSize()  != 10 )    { SER( SM_ERR ); }
  if ( da4.GetDataArray() != pData ) { SER( SM_ERR ); }

  //##test SetIsBorrowed()
  // Note, this could cause either memory leaks or double deletes.
  da4.SetIsBorrowed( TRUE );  // could cause memory leak.
  // No way to test this.

  //## test GetMemoryUsed()
  ULONG lMemAlloc;
  ULONG lMemUsed = da4.GetMemoryUsed( lMemAlloc );
  // da4 is borrowed here, so GetMemoryUsed() reports just the object itself, which varies with pointer and ULONG width.
  if ( lMemUsed  != sizeof( da4 ) ) { SER( SM_ERR ); }
  if ( lMemAlloc != sizeof( da4 ) ) { SER( SM_ERR ); }

  //## test FindElement()
  // Set it up first.
  da4.ReSet();
  da4.Add( 10.0 );
  da4.Add( 11.0 );
  da4.Add( 12.0 );
  da4.Add( 12.0 );
  da4.Add( 12.0 );

  SmBoolean bOk;

  ULONG lIndex;
  bOk = da4.FindElement( 11.0, lIndex );
  if ( ! bOk )       { SER( SM_ERR ); }
  if ( lIndex != 1 ) { SER( SM_ERR ); }

  //## test FindElements()
  SmTArray< ULONG > sIndices;
  da4.FindElements( 12.0, sIndices );
  if ( sIndices.GetSize() != 3 ) { SER( SM_ERR ); }
  if ( sIndices[0] != 2 )        { SER( SM_ERR ); }
  if ( sIndices[1] != 3 )        { SER( SM_ERR ); }
  if ( sIndices[2] != 4 )        { SER( SM_ERR ); }

  //## test SetAt() and GetAt()
  da4.SetAt( 3, 27.0 );
  double dVal = da4.GetAt( 3 );
  if ( dVal != 27.0 ) { SER( SM_ERR ); }

  //## test both operator[]'s: non-const for setting and const for getting
  da4[2] = 112.0;
  dVal = da4[2];
  if ( dVal != 112.0 ) { SER( SM_ERR ); }

  //## test GetLast();
  da4.Add( 14.0 );
  dVal = da4.GetLast();
  if ( dVal != 14.0 ) { SER( SM_ERR ); }

  //Set up for next tests.
  da4.ReSet();
  da4.Add( 10.0 );
  da4.Add( 11.0 );
  da4.Add( 12.0 );
  da4.Add( 13.0 );
  da4.Add( 14.0 );

  da5.ReSet();
  da5.Add( 10.0 );
  da5.Add( 12.0 );
  da5.Add( 14.0 );
  da5.Add( 16.0 );
  da5.Add( 18.0 );

  //## test FindCommonElements()
  da4.FindCommonElements( da5, da2 );  // da2 is output
  if ( da2.GetSize() != 3 ) { SER( SM_ERR ); }

  //## test FindUniqueElements()
  da4.FindUniqueElements( da5, da2 );  // da2 is output
  if ( da2.GetSize() != 2 ) { SER( SM_ERR ); }

  //## test RemoveElements()
  da4.RemoveElements( da5, da2 );  // da2 is output
  if ( da2.GetSize() != 2 ) { SER( SM_ERR ); }

  // Set up for next tests.
  da4.ReSet();
  da4.Add( 10.0 );
  da4.Add( 13.0 );
  da4.Add( 11.0 );
  da4.Add( 12.0 );
  da4.Add( 13.0 );
  da4.Add( 14.0 );
  da4.Add( 13.0 );

  //## test GetDuplicates()
  da4.GetDuplicates( da2 );  // da2 is output
  if ( da2.GetSize() != 1 ) { SER( SM_ERR ); }
  if ( da2[0] != 13.0     ) { SER( SM_ERR ); }

  //## test RemoveDuplicates()
  da4.RemoveDuplicates();  // da2 is output
  if ( da4.GetSize() != 5 ) { SER( SM_ERR ); }

  // Set up for next tests.
  da4.ReSet();
  da4.Add( 10.0 );
  da4.Add( 11.0 );
  da4.Add( 12.0 );
  da4.Add( 13.0 );

  //## test AddUnique()
  da4.AddUnique( 14.0 );
  if ( da4.GetSize() != 5 ) { SER( SM_ERR ); }
  da4.AddUnique( 14.0 );
  if ( da4.GetSize() != 5 ) { SER( SM_ERR ); }

  //## test Push() and Pop()
  da4.Push( 21.0 );
  if ( da4.GetSize() != 6 ) { SER( SM_ERR ); }
  da4.Pop( dVal );
  if ( da4.GetSize() != 5 ) { SER( SM_ERR ); }
  if ( dVal != 21.0 ) { SER( SM_ERR ); }

  //cbi next: Append().


    return SM_SUCCESS;
} // end my_test_array

/***********************************************************************
PURPOSE ---

USAGE NOTES ---
***********************************************************************/
PT_EXPORT SmStatus my_test_map( void )
{
  SmContext sContext;
  SmObject obj1, obj2, obj3, obj4;
  SmMapPtrToPtr<SmObject, SmObject> *pPPMap = new(sContext) SmMapPtrToPtr<SmObject, SmObject>;
  SmObjDelete sCU( pPPMap );

  if(!pPPMap->IsEmpty())
    SER( SM_ERR );

  pPPMap->Insert( &obj1, &obj2 );
  if(pPPMap->IsEmpty())
    SER( SM_ERR );

  if(pPPMap->Count() != 1)
    SER( SM_ERR );

  pPPMap->Insert( &obj2, &obj3 );
  pPPMap->Insert( &obj3, &obj4 );
  pPPMap->Insert( &obj4, &obj1 );
  if(pPPMap->Count() != 4)
    SER( SM_ERR );

  void *pPtr = nullptr;
  if(pPPMap->Contains( &obj1 ))
  {
    pPtr = (*pPPMap)[&obj1];
    if(pPtr != &obj2)
      SER( SM_ERR );
  }
  else { SER( SM_ERR ); }

  if(pPtr != pPPMap->At( &obj1 ))
    SER( SM_ERR );

#ifdef SM_DEFINED_HASH_ORDER
  static_assert(false, "not implemented");
#endif
  SmTArray<SmObject*> sKeys, sVals;
  ULONG lCount = 0;
  pPPMap->GetAllKeyValuePairs( sKeys, sVals );
  for(ULONG ii = 0; ii < sKeys.GetSize(); ii++)
  {
    SmObject *key = sKeys[ii];
    SmObject *val = sVals[ii];
    if (pPPMap->At(key) != val)
      SER(SM_ERR);

    lCount++;
  }
   
  if(lCount != 4)
    SER( SM_ERR );

  pPPMap->Remove( &obj1 );

  return SM_SUCCESS;

} // end my_test_map


/***********************************************************************
PURPOSE ---

USAGE NOTES ---
***********************************************************************/
PT_EXPORT SmStatus my_test_polynomial(void)
{
    { // Test quadratic equation solver
        double aS[2], aC[3];
        ULONG lNS;
        
        aC[0] = 2;
        aC[1] = 3;
        aC[2] = 1;
        SmPolynomial::SolveQuadraticEqn(aC, SM_EFF_ZERO, lNS, aS);
        SM_ASSERT(lNS == 2);
        // Should produce -1 and -2
        SM_ASSERT(smos_Fabs(aS[0] + aS[1] + 3) < SM_EFF_ZERO);
        
        aC[0] = 2;
        aC[1] = 7;
        aC[2] = 6;
        SmPolynomial::SolveQuadraticEqn(aC, SM_EFF_ZERO, lNS, aS);
        // Should produce -1/2 and -2/3
        SM_ASSERT(lNS == 2);
        
        aC[0] = 1;
        aC[1] = 2;
        aC[2] = 1;
        SmPolynomial::SolveQuadraticEqn(aC, SM_EFF_ZERO, lNS, aS);
        SM_ASSERT(lNS == 1);
        // Should produce -1
        SM_ASSERT(smos_Fabs(aS[0] + 1) < SM_EFF_ZERO);
        
        aC[0] = 1;
        aC[1] = -1;
        aC[2] = 1;
        SmPolynomial::SolveQuadraticEqn(aC, SM_EFF_ZERO, lNS, aS);
        // Should not produce any real answers
        SM_ASSERT(lNS == 0);
    }
    
    { // Test cubic equation solver
        double aS[3], aC[4];
        ULONG lNS;
        
        aC[0] = 6;
        aC[1] = 11;
        aC[2] = 6;
        aC[3] = 1;
        SmPolynomial::SolveCubicEqn(aC, SM_EFF_ZERO, lNS, aS);
        // Should produce -1 and -2 and -3
        SM_ASSERT(lNS == 3);
        
        aC[0] = 1;
        aC[1] = 3;
        aC[2] = 3;
        aC[3] = 1;
        SmPolynomial::SolveCubicEqn(aC, SM_EFF_ZERO, lNS, aS);
        // Should produce -1  - a triple root
        SM_ASSERT(lNS == 1);
        
        aC[0] = 2;
        aC[1] = 5;
        aC[2] = 4;
        aC[3] = 1;
        SmPolynomial::SolveCubicEqn(aC, SM_EFF_ZERO, lNS, aS);
        // Should produce -1 and -2 with a double root at -2
        SM_ASSERT(lNS == 2);
        
        aC[0] = 1;
        aC[1] = 0;
        aC[2] = 0;
        aC[3] = 1;
        SmPolynomial::SolveCubicEqn(aC, SM_EFF_ZERO, lNS, aS);
        // Should produce -1 with two imaginary roots
        SM_ASSERT(lNS == 1);
    }
    
    { // Test quintic equation solver
        double aS[4], aC[5];
        ULONG lNS;
        
        aC[0] = -24;
        aC[1] = -38;
        aC[2] = -13;
        aC[3] = 2;
        aC[4] = 1;
        SmPolynomial::SolveQuarticEqn(aC, SM_EFF_ZERO, lNS, aS);
        // Should produce -1 and -2 and -3 and +4
        SM_ASSERT(lNS == 4);
        
        aC[0] = 1;
        aC[1] = 4;
        aC[2] = 6;
        aC[3] = 4;
        aC[4] = 1;
        SmPolynomial::SolveQuarticEqn(aC, SM_EFF_ZERO, lNS, aS);
        // Should produce -1  - a quadrupal root
        SM_ASSERT(lNS == 1);
        
        aC[0] = 2;
        aC[1] = 7;
        aC[2] = 9;
        aC[3] = 5;
        aC[4] = 1;
        SmPolynomial::SolveQuarticEqn(aC, SM_EFF_ZERO, lNS, aS);
        // Should produce -1 and -2 with a double roots at -2 and -1
        SM_ASSERT(lNS == 2);
        
        aC[0] = 1;
        aC[1] = 0;
        aC[2] = -2;
        aC[3] = 0;
        aC[4] = 1;
        SmPolynomial::SolveQuarticEqn(aC, SM_EFF_ZERO, lNS, aS);
        // Should produce -1 and 1 both double roots
        SM_ASSERT(lNS == 2);
    }
    
    
    
    return SM_SUCCESS;
} // end my_test_polynomial

/***********************************************************************
PURPOSE --- Unit tests for class SmVector2d

USAGE NOTES ---
   These tests are quite comprehensive in that they call pretty much
   every method, but they are not all functionally exhaustive, testing
   all behaviors.  Most were added for the Java project, for which we
   need a wrapper for every method.
***********************************************************************/
PT_EXPORT SmStatus my_test_vector2d()
{
  // Scope so that we can see its destructor:
  {
      SmVector2d v2(0.5, 1.2);                    //##test: constructor(double,double)
      if ( smos_Fabs( v2.x - 0.5 ) > SM_EFF_ZERO ) { SER(SM_ERR); }
      if ( smos_Fabs( v2.y - 1.2 ) > SM_EFF_ZERO ) { SER(SM_ERR); }

      v2.SetUninitialized();                      //##test: SetUninitialized()
      if ( v2.IsInitialized() ) { SER(SM_ERR); }  //##test: Isinitialized()

  } //##test: destructor call here.

  double adVec[2] = {1.0, 2.0};
  SmVector2d v3(adVec);                           //##test: constructor( double[] )

  if (smos_Fabs(v3.LengthSquared() - 5.0) > SM_EFF_ZERO)  //##test: LengthSquared()
    { SER(SM_ERR); }
  SmVector2d v4;                                  //##test: construct(void)
  v4.Set( 3, 4 );  //##test: Set()
  if (smos_Fabs(v4.Length() - 5.0) > SM_EFF_ZERO) //##test: Length()
    { SER(SM_ERR); }

  SmVector2d v5 = v4;     //##test: assignment operator=()
  v4 = - v4;              //##test: Unary minus: operator -(const SmVector2d& v)
  SmVector2d v6( v5 );    //##test: copy constructor

  if ( smos_Fabs( v6.x-v5.x ) > SM_EFF_ZERO ) { SER(SM_ERR); }
  if ( smos_Fabs( v6.y-v5.y ) > SM_EFF_ZERO ) { SER(SM_ERR); }

  v6 += v4;               //##test: operator+=()
  if ( smos_Fabs( v6.x ) > SM_EFF_ZERO ) { SER(SM_ERR); }
  if ( smos_Fabs( v6.y ) > SM_EFF_ZERO ) { SER(SM_ERR); }

  SER(v3.Unitize());      //##test: Unitize()
  if (smos_Fabs(v3.Length() - 1.0) > SM_EFF_ZERO)
    { SER(SM_ERR); }

  if (smos_Fabs(v3.Dot(v3) - 1.0) > SM_EFF_ZERO)  //##test: Dot()
    { SER(SM_ERR); }

  SmVector2d v8(5, 1);
  SmVector2d v9(1, 4);
  if (smos_Fabs(v8.DistanceBetween(v9) - 5.0) > SM_EFF_ZERO)  //##test: DistanceBetween()
    { SER(SM_ERR); }

  double dTest;
  if ( v8.AngleBetween( v9, dTest ) != SM_SUCCESS )     //##test: AngleBetween()
    { SER(SM_ERR); }
  if ( smos_Fabs( dTest - 1.1284221038181517 ) > SM_EFF_ZERO ) { SER(SM_ERR); }

  if ( v9.CCWAngleBetween( v8, dTest ) != SM_SUCCESS )  //##test: CCWAngleBetween()
    { SER(SM_ERR); }
  if ( smos_Fabs( dTest - 5.1547632033614343 ) > SM_EFF_ZERO ) { SER(SM_ERR); }

  dTest = v9.Cross( v8 );  //##test: Cross()
  if ( smos_Fabs( dTest + 19.0               ) > SM_EFF_ZERO ) { SER(SM_ERR); }

  dTest = v8.DistanceBetweenSquared( v4 );  //##test: DistanceBetweenSquared()
  if ( smos_Fabs( dTest - 89.0               ) > SM_EFF_ZERO ) { SER(SM_ERR); }

  if ( v8.CloserThan( 1.0, v9 ) ) { SER(SM_ERR); }  //##test: CloserThan()

  dTest = v8.GetMinDimension();  //##test: GetMinDimension()
  if ( smos_Fabs( dTest - 1.0 ) > SM_EFF_ZERO ) { SER(SM_ERR); }

  dTest = v8.GetMaxDimension();  //##test: GetMaxDimension()
  if ( smos_Fabs( dTest - 5.0 ) > SM_EFF_ZERO ) { SER(SM_ERR); }


  SmVector2d v10 = - v8;

  SmVector2d v11 = v10 + v8;  //##test:  operator+()
  if ( smos_Fabs( v11.x ) > SM_EFF_ZERO ) { SER(SM_ERR); }
  if ( smos_Fabs( v11.y ) > SM_EFF_ZERO ) { SER(SM_ERR); }

  v11 = v8 - v8;             //##test:  operator-()
  if ( smos_Fabs( v11.x ) > SM_EFF_ZERO ) { SER(SM_ERR); }
  if ( smos_Fabs( v11.y ) > SM_EFF_ZERO ) { SER(SM_ERR); }

  v11 = v8 * 2.0;            //##test:  operator*()
  if ( smos_Fabs( v11.x - v8.x*2.0 ) > SM_EFF_ZERO ) { SER(SM_ERR); }
  if ( smos_Fabs( v11.y - v8.y*2.0 ) > SM_EFF_ZERO ) { SER(SM_ERR); }

  v11 *= 3.0;                //##test:  operator*=()
  if ( smos_Fabs( v11.x - v8.x*6.0 ) > SM_EFF_ZERO ) { SER(SM_ERR); }
  if ( smos_Fabs( v11.y - v8.y*6.0 ) > SM_EFF_ZERO ) { SER(SM_ERR); }

  SmVector2d v12 = v8 / 2.0;            //##test:  operator/()
  if ( smos_Fabs( v12.x - v8.x/2.0 ) > SM_EFF_ZERO ) { SER(SM_ERR); }
  if ( smos_Fabs( v12.y - v8.y/2.0 ) > SM_EFF_ZERO ) { SER(SM_ERR); }

  v11 /= 2.0;                //##test:  operator/=()
  if ( smos_Fabs( v11.x - v8.x*3.0 ) > SM_EFF_ZERO ) { SER(SM_ERR); }
  if ( smos_Fabs( v11.y - v8.y*3.0 ) > SM_EFF_ZERO ) { SER(SM_ERR); }

  SmVector2d v17 = v8.Multiply( v8 );  //##test: Multiply()
  if ( smos_Fabs( v17.x - 25.0 ) > SM_EFF_ZERO ) { SER(SM_ERR); }
  if ( smos_Fabs( v17.y -  1.0 ) > SM_EFF_ZERO ) { SER(SM_ERR); }

  SER( v11.Divide( v8, v17 ) );        //##test: Divide()
  if ( smos_Fabs( v17.x - 3.0 ) > SM_EFF_ZERO ) { SER(SM_ERR); }
  if ( smos_Fabs( v17.y - 3.0 ) > SM_EFF_ZERO ) { SER(SM_ERR); }

  dTest = v8[0];    //##test: indexing: operator[], const
  if ( smos_Fabs( dTest - 5.0 ) > SM_EFF_ZERO ) { SER(SM_ERR); }

  v11[1] = 6.0;     //##test: indexing: operator[], non-const, for assignment
  if ( smos_Fabs( v11.y - 6.0 ) > SM_EFF_ZERO ) { SER(SM_ERR); }

  v6 = v5.Rotate( SM_PI / 2.0 );  //##test: Rotate()
  if ( smos_Fabs( v6.x + 4.0 ) > SM_EFF_ZERO ) { SER(SM_ERR); }
  if ( smos_Fabs( v6.y - 3.0 ) > SM_EFF_ZERO ) { SER(SM_ERR); }

  int iInside = v3.IsInTriangle( v4, v8, v9 );  //##test: IsInTriangle()
  if ( iInside != 1 ) { SER(SM_ERR); }

  ULONG lDumpNum = 77;
  v8.Dump();                                           //##test: Dump(void)
  v5.Dump( _T("\n  Vector2d: Dump with message\n") );  //##test: Dump(char *)
  v6.Dump( lDumpNum );                                 //##test: Dump(ULONG)
  v4.GetTypeString();                                  //##test: GetTypeString()

  // Read() and Write() are not working.
  // FILE *pFile = SM_FOPEN( _T("vector2d_test"), _T("w") );
  // v17.Write( pFile );  //##test: Write()
  // v3 .Read ( pFile );  //##test: Read()
  // if ( smos_Fabs( v3.x - v17.x ) > SM_EFF_ZERO ) { SER(SM_ERR); }
  // if ( smos_Fabs( v3.y - v17.y ) > SM_EFF_ZERO ) { SER(SM_ERR); }


#ifdef SM_GFX_CODE
  if (smGet_DoGraphics())
        v9.Draw();
#endif

    return SM_SUCCESS;

} // end my_test_vector2d

/***********************************************************************
PURPOSE --- Unit tests for class SmVector3d

USAGE NOTES ---
   These tests are quite comprehensive in that they call pretty much
   every method, but they are not all functionally exhaustive, testing
   all behaviors.  Most were added for the Java project, for which we
   need a wrapper for every method.
***********************************************************************/
PT_EXPORT SmStatus my_test_vector3d()
{
    // Scope so that we can see its destructor:
    {
        SmVector3d v2(0.5, 1.2, 2.2);  //##test: constructor(double,double)
        if (smos_Fabs(v2.x - 0.5) > SM_EFF_ZERO) { SER(SM_ERR); }
        if (smos_Fabs(v2.y - 1.2) > SM_EFF_ZERO) { SER(SM_ERR); }
        if (smos_Fabs(v2.z - 2.2) > SM_EFF_ZERO) { SER(SM_ERR); }

        v2.SetUninitialized();                      //##test: SetUninitialized()
        if (v2.IsInitialized()) { SER(SM_ERR); }  //##test: Isinitialized()

    } //##test: destructor call here.

    double adVec[3] = { 1.0, 2.0, 2.0 };
    SmVector3d v3(adVec);              //##test: constructor( double[] )

    //Values: v3( 1, 2, 2 )
    double dTest;
    dTest = v3.LengthSquared();        //##test: LengthSquared()
    if (smos_Fabs(dTest - 9.0) > SM_EFF_ZERO) { SER(SM_ERR); }

    SmVector3d v4;                     //##test: construct(void)
    v4.Set(3, 4, 0);                 //##test: Set()
    dTest = v4.Length();   //##test: Length()
    if (smos_Fabs(dTest - 5.0) > SM_EFF_ZERO) { SER(SM_ERR); }

    //Values: v4( 3, 4, 0 )
    SmVector3d v5 = v4;     //##test: assignment operator=()
    v4 = -v4;              //##test: Unary minus: operator -(const SmVector3d& v)

    //Values: v5( 3, 4, 0 )
    SmVector3d v6(v5);    //##test: copy constructor

    if (smos_Fabs(v6.x - v5.x) > SM_EFF_ZERO) { SER(SM_ERR); }
    if (smos_Fabs(v6.y - v5.y) > SM_EFF_ZERO) { SER(SM_ERR); }
    if (smos_Fabs(v6.z - v5.z) > SM_EFF_ZERO) { SER(SM_ERR); }

    //Values: v6( 3, 4, 0 ),  v4( -3, -4, 0 )
    v6 += v4;               //##test: operator+=()
    if (smos_Fabs(v6.x) > SM_EFF_ZERO) { SER(SM_ERR); }
    if (smos_Fabs(v6.y) > SM_EFF_ZERO) { SER(SM_ERR); }
    if (smos_Fabs(v6.z) > SM_EFF_ZERO) { SER(SM_ERR); }

    //Values: v6( 0, 0, 0 ),  v4( -3, -4, 0 )
    v6 -= v4;               //##test: operator-=()
    if (smos_Fabs(v6.x - 3.0) > SM_EFF_ZERO) { SER(SM_ERR); }
    if (smos_Fabs(v6.y - 4.0) > SM_EFF_ZERO) { SER(SM_ERR); }
    if (smos_Fabs(v6.z) > SM_EFF_ZERO) { SER(SM_ERR); }

    //Values: v3( 1, 2, 2 )
    SER(v3.Unitize());      //##test: Unitize()
    if (smos_Fabs(v3.Length() - 1.0) > SM_EFF_ZERO)
    {
        SER(SM_ERR);
    }

    if (smos_Fabs(v3.Dot(v3) - 1.0) > SM_EFF_ZERO)  //##test: Dot()
    {
        SER(SM_ERR);
    }

    SmVector3d v8(5, 1, 2);
    SmVector3d v9(1, 3, 6);
    if (smos_Fabs(v8.DistanceBetween(v9) - 6.0) > SM_EFF_ZERO)  //##test: DistanceBetween()
    {
        SER(SM_ERR);
    }

    //Values: v8( 5, 1, 2 ), v9( 1, 3, 6 )
    if (v8.AngleBetween(v9, dTest) != SM_SUCCESS)     //##test: AngleBetween()
    {
        SER(SM_ERR);
    }
    if (smos_Fabs(dTest - 1.0022805307621478) > SM_EFF_ZERO) { SER(SM_ERR); }

    //Values: v8( 5, 1, 2 ), v9( 1, 3, 6 )
    v5.Set(0, 0, 1);
    if (v5.CCWAngleBetween(v8, v9, dTest) != SM_SUCCESS)  //##test: CCWAngleBetween()
    {
        SER(SM_ERR);
    }
    if (smos_Fabs(dTest - 1.0516502125483738) > SM_EFF_ZERO) { SER(SM_ERR); }

    // Not in SmVector3d:
    //  dTest = v9.Cross( v8 );  //##test: Cross()
    //  if ( smos_Fabs( dTest + 19.0               ) > SM_EFF_ZERO ) { SER(SM_ERR); }

      //Values: v8( 5, 1, 2 ), v9( 1, 3, 6 )
    dTest = v8.DistanceBetweenSquared(v4);  //##test: DistanceBetweenSquared()
    if (smos_Fabs(dTest - 93.0) > SM_EFF_ZERO) { SER(SM_ERR); }

    if (v8.CloserThan(1.0, v9)) { SER(SM_ERR); }  //##test: CloserThan()

    dTest = v8.GetMinDimension();  //##test: GetMinDimension()
    if (smos_Fabs(dTest - 1.0) > SM_EFF_ZERO) { SER(SM_ERR); }

    dTest = v8.GetMaxDimension();  //##test: GetMaxDimension()
    if (smos_Fabs(dTest - 5.0) > SM_EFF_ZERO) { SER(SM_ERR); }


    SmVector3d v10(-5, -1, -2);  // v10 = -v8

    //Values: v8( 5, 1, 2 )
    SmVector3d v11 = v10 + v8;  //##test:  operator+()
    if (smos_Fabs(v11.x) > SM_EFF_ZERO) { SER(SM_ERR); }
    if (smos_Fabs(v11.y) > SM_EFF_ZERO) { SER(SM_ERR); }
    if (smos_Fabs(v11.z) > SM_EFF_ZERO) { SER(SM_ERR); }

    //Values: v8( 5, 1, 2 )
    v11 = v8 - v8;             //##test:  operator-()   (binary)
    if (smos_Fabs(v11.x) > SM_EFF_ZERO) { SER(SM_ERR); }
    if (smos_Fabs(v11.y) > SM_EFF_ZERO) { SER(SM_ERR); }
    if (smos_Fabs(v11.z) > SM_EFF_ZERO) { SER(SM_ERR); }

    //Values: v8( 5, 1, 2 )
    v11 = v8 * 2.0;            //##test:  operator*()
    if (smos_Fabs(v11.x - v8.x*2.0) > SM_EFF_ZERO) { SER(SM_ERR); }
    if (smos_Fabs(v11.y - v8.y*2.0) > SM_EFF_ZERO) { SER(SM_ERR); }
    if (smos_Fabs(v11.z - v8.z*2.0) > SM_EFF_ZERO) { SER(SM_ERR); }

    //Values: v8( 5, 1, 2 )
    v11 *= 3.0;                //##test:  operator*=()
    if (smos_Fabs(v11.x - v8.x*6.0) > SM_EFF_ZERO) { SER(SM_ERR); }
    if (smos_Fabs(v11.y - v8.y*6.0) > SM_EFF_ZERO) { SER(SM_ERR); }
    if (smos_Fabs(v11.z - v8.z*6.0) > SM_EFF_ZERO) { SER(SM_ERR); }

    //Values: v8( 5, 1, 2 )
    SmVector3d v12 = v8 / 2.0;            //##test:  operator/()
    if (smos_Fabs(v12.x - v8.x / 2.0) > SM_EFF_ZERO) { SER(SM_ERR); }
    if (smos_Fabs(v12.y - v8.y / 2.0) > SM_EFF_ZERO) { SER(SM_ERR); }
    if (smos_Fabs(v12.z - v8.z / 2.0) > SM_EFF_ZERO) { SER(SM_ERR); }

    //Values: v11( 30, 6, 12 )
    v11 /= 2.0;                //##test:  operator/=()
    if (smos_Fabs(v11.x - 15.0) > SM_EFF_ZERO) { SER(SM_ERR); }
    if (smos_Fabs(v11.y - 3.0) > SM_EFF_ZERO) { SER(SM_ERR); }
    if (smos_Fabs(v11.z - 6.0) > SM_EFF_ZERO) { SER(SM_ERR); }

    //Values: v8( 5, 1, 2 ),  v9( 1, 3, 6 )
    SmVector3d v17 = v8.Multiply(v9);  //##test: Multiply()
    if (smos_Fabs(v17.x - 5.0) > SM_EFF_ZERO) { SER(SM_ERR); }
    if (smos_Fabs(v17.y - 3.0) > SM_EFF_ZERO) { SER(SM_ERR); }
    if (smos_Fabs(v17.z - 12.0) > SM_EFF_ZERO) { SER(SM_ERR); }

    //Values: v11( 15, 3, 6 ), v8( 5, 1, 2 )
    SER(v11.Divide(v8, v17));        //##test: Divide()
    if (smos_Fabs(v17.x - 3.0) > SM_EFF_ZERO) { SER(SM_ERR); }
    if (smos_Fabs(v17.y - 3.0) > SM_EFF_ZERO) { SER(SM_ERR); }
    if (smos_Fabs(v17.z - 3.0) > SM_EFF_ZERO) { SER(SM_ERR); }

    //Values: v8( 5, 1, 2 )
    dTest = v8[0];    //##test: indexing: operator[], const
    if (smos_Fabs(dTest - 5.0) > SM_EFF_ZERO) { SER(SM_ERR); }

    //Values: v11( 15, 3, 6 )
    v11[1] = 7.0;     //##test: indexing: operator[], non-const, for assignment
    if (smos_Fabs(v11.y - 7.0) > SM_EFF_ZERO) { SER(SM_ERR); }

    // Not in SmVector3d:
    //  // v5( 3, 4, 0 )
    //  v6 = v5.Rotate( SM_PI / 2.0 );  //##test: Rotate()
    //  if ( smos_Fabs( v6.x + 4.0 ) > SM_EFF_ZERO ) { SER(SM_ERR); }
    //  if ( smos_Fabs( v6.y - 3.0 ) > SM_EFF_ZERO ) { SER(SM_ERR); }
    //  if ( smos_Fabs( v6.z - 0.0 ) > SM_EFF_ZERO ) { SER(SM_ERR); }

      //Values: v4( -3, -4, 0 ),  v8( 5, 1, 2 ),  v9( 1, 3, 6 ),
    v3.Set(2, 1, 3);
    int iInside = v3.IsInTriangle(v4, v8, v9);  //##test: IsInTriangle()
    if (iInside != 1) { SER(SM_ERR); }

    ULONG lDumpNum = 77;
    v8.Dump();                                         //##test: Dump(void)
    v5.Dump(_T("\n  Vector3d: Dump with message\n"));  //##test: Dump(char *)
    v6.Dump(lDumpNum);                                 //##test: Dump(ULONG)
    v4.GetTypeString();                                //##test: GetTypeString()

    // Not doint Write().
    // FILE *pFile = SM_FOPEN( _T("vector2d_test"), _T("w") );
    // v17.Write( pFile );  //##test: Write()


#ifdef SM_GFX_CODE
    if (smGet_DoGraphics())
    {
        v9.Draw();
    }
#endif

    ////////////////   Methods that aren't in SmVector2d    /////////////

    SmVector3d tmpVec0(0, 0, 1);
    SmVector3d tmpVec1(0, 1, 0);

    SmVector3d v21(tmpVec0, tmpVec1);   //##test c'tor with 2 points
    if (smos_Fabs(v21.x) > SM_EFF_ZERO) { SER(SM_ERR); }
    if (smos_Fabs(v21.y - 1.0) > SM_EFF_ZERO) { SER(SM_ERR); }
    if (smos_Fabs(v21.z + 1.0) > SM_EFF_ZERO) { SER(SM_ERR); }

    SmVector2d tmpVec2d(1, 0);
    SmVector3d v22(tmpVec2d);  //##test c'tor with SmVector2d
    if (smos_Fabs(v22.x - 1.0) > SM_EFF_ZERO) { SER(SM_ERR); }
    if (smos_Fabs(v22.y) > SM_EFF_ZERO) { SER(SM_ERR); }
    if (smos_Fabs(v22.z) > SM_EFF_ZERO) { SER(SM_ERR); }

    SmBoolean bBool = v21.IsUndef();
    if (bBool) { SER(SM_ERR); }  // should be not undef.

    tmpVec0.Set(1, 1, 1);
    tmpVec1.Set(3, 3, 3);
    SmPoint3d pt1(0, 0, 0);
    SmPoint3d pt2(1, 1, 1);

    bBool = tmpVec0.IsColinearWith(tmpVec1, pt1, pt2);
    if (!bBool) { SER(SM_ERR); }

    double dDouble = 0.000001;
    bBool = tmpVec0.IsParallelTo(tmpVec1, dDouble);
    if (!bBool) { SER(SM_ERR); }

    tmpVec0.Set(1, 1, 1);
    tmpVec1.Set(-3, 2, 1);
    bBool = tmpVec0.IsPerpendicularTo(tmpVec1, dDouble);
    if (!bBool) { SER(SM_ERR); }

    v21.Set(0, 2, 1);
    bBool = tmpVec0.IsInsideSector(tmpVec1, v21, v22);
    if (!bBool) { SER(SM_ERR); }


    dDouble = tmpVec0.TripleProduct(v21, v22);
    if (smos_Fabs(dDouble + 1.0) > SM_EFF_ZERO) { SER(SM_ERR); }

    tmpVec0.Set(0, 1, 0);
    tmpVec1.Set(0, 2, 1);
    SmVector3d v23;
    SmStatus eStat = tmpVec0.MakeUnitOrthoVectors(
        &tmpVec1, v21, v22, v23);
    if (eStat != SM_SUCCESS) { SER(SM_ERR); }
    if (smos_Fabs(v21.x) > SM_EFF_ZERO) { SER(SM_ERR); }
    if (smos_Fabs(v21.y - 1.0) > SM_EFF_ZERO) { SER(SM_ERR); }
    if (smos_Fabs(v21.z) > SM_EFF_ZERO) { SER(SM_ERR); }
    if (smos_Fabs(v22.x) > SM_EFF_ZERO) { SER(SM_ERR); }
    if (smos_Fabs(v22.y) > SM_EFF_ZERO) { SER(SM_ERR); }
    if (smos_Fabs(v22.z - 1.0) > SM_EFF_ZERO) { SER(SM_ERR); }
    if (smos_Fabs(v23.x - 1.0) > SM_EFF_ZERO) { SER(SM_ERR); }
    if (smos_Fabs(v23.y) > SM_EFF_ZERO) { SER(SM_ERR); }
    if (smos_Fabs(v23.z) > SM_EFF_ZERO) { SER(SM_ERR); }

    tmpVec1.Unitize(); // plane normal must be a unit vector.
    v21 = tmpVec0.ProjectToPlane(tmpVec1);
    if (smos_Fabs(v21.x) > SM_EFF_ZERO) { SER(SM_ERR); }
    if (smos_Fabs(v21.y - 0.2) > SM_EFF_ZERO) { SER(SM_ERR); }
    if (smos_Fabs(v21.z + 0.4) > SM_EFF_ZERO) { SER(SM_ERR); }

    v21 = tmpVec0.ProjectPointToPlane(v22, tmpVec1);
    if (smos_Fabs(v21.x) > SM_EFF_ZERO) { SER(SM_ERR); }
    if (smos_Fabs(v21.y - 0.6) > SM_EFF_ZERO) { SER(SM_ERR); }
    if (smos_Fabs(v21.z + 0.2) > SM_EFF_ZERO) { SER(SM_ERR); }

    dDouble = 1.5707963267948966;   //  PI/2
    tmpVec1.Set(0, 0, 1);
    v21 = tmpVec0.RotateVecAboutAxis(tmpVec1, dDouble);
    if (smos_Fabs(v21.x + 1.0) > SM_EFF_ZERO) { SER(SM_ERR); }
    if (smos_Fabs(v21.y) > SM_EFF_ZERO) { SER(SM_ERR); }
    if (smos_Fabs(v21.z) > SM_EFF_ZERO) { SER(SM_ERR); }

    v22.Set(1, 0, 0);
    v21 = tmpVec0.RotatePtAboutLine(v22, tmpVec1, dDouble);
    if (smos_Fabs(v21.x) > SM_EFF_ZERO) { SER(SM_ERR); }
    if (smos_Fabs(v21.y + 1.0) > SM_EFF_ZERO) { SER(SM_ERR); }
    if (smos_Fabs(v21.z) > SM_EFF_ZERO) { SER(SM_ERR); }

    v21 = tmpVec0.NormalInPlane(tmpVec1);
    if (smos_Fabs(v21.x) > SM_EFF_ZERO) { SER(SM_ERR); }
    if (smos_Fabs(v21.y) > SM_EFF_ZERO) { SER(SM_ERR); }
    if (smos_Fabs(v21.z - 1.0) > SM_EFF_ZERO) { SER(SM_ERR); }

    v21 = tmpVec0.UnitizedDerivative(tmpVec1);
    if (smos_Fabs(v21.x) > SM_EFF_ZERO) { SER(SM_ERR); }
    if (smos_Fabs(v21.y) > SM_EFF_ZERO) { SER(SM_ERR); }
    if (smos_Fabs(v21.z - 1.0) > SM_EFF_ZERO) { SER(SM_ERR); }

    SmVector3d tmpVec2(0, 0, 0);
    eStat = tmpVec0.UnitizedDerivative2(tmpVec1, tmpVec2, v21, v22);
    if (eStat != SM_SUCCESS) { SER(SM_ERR); }
    if (smos_Fabs(v21.x) > SM_EFF_ZERO) { SER(SM_ERR); }
    if (smos_Fabs(v21.y) > SM_EFF_ZERO) { SER(SM_ERR); }
    if (smos_Fabs(v21.z - 1.0) > SM_EFF_ZERO) { SER(SM_ERR); }
    if (smos_Fabs(v22.x) > SM_EFF_ZERO) { SER(SM_ERR); }
    if (smos_Fabs(v22.y + 1.0) > SM_EFF_ZERO) { SER(SM_ERR); }
    if (smos_Fabs(v22.z) > SM_EFF_ZERO) { SER(SM_ERR); }

    SmVector3d tmpVec3(0, 0, 0);
    eStat = tmpVec0.UnitizedDerivative3(tmpVec1, tmpVec2, tmpVec3, v21, v22, v23);
    if (eStat != SM_SUCCESS) { SER(SM_ERR); }
    if (smos_Fabs(v21.x) > SM_EFF_ZERO) { SER(SM_ERR); }
    if (smos_Fabs(v21.y) > SM_EFF_ZERO) { SER(SM_ERR); }
    if (smos_Fabs(v21.z - 1.0) > SM_EFF_ZERO) { SER(SM_ERR); }
    if (smos_Fabs(v22.x) > SM_EFF_ZERO) { SER(SM_ERR); }
    if (smos_Fabs(v22.y + 1.0) > SM_EFF_ZERO) { SER(SM_ERR); }
    if (smos_Fabs(v22.z) > SM_EFF_ZERO) { SER(SM_ERR); }
    if (smos_Fabs(v23.x) > SM_EFF_ZERO) { SER(SM_ERR); }
    if (smos_Fabs(v23.y) > SM_EFF_ZERO) { SER(SM_ERR); }
    if (smos_Fabs(v23.z + 3.0) > SM_EFF_ZERO) { SER(SM_ERR); }

    tmpVec2.Set(-1, 0, 0);
    tmpVec3.Set(0, 0, 0);
    v21 = tmpVec0.DerivativeOfCrossProduct(tmpVec1, tmpVec2, tmpVec3);
    if (smos_Fabs(v21.x) > SM_EFF_ZERO) { SER(SM_ERR); }
    if (smos_Fabs(v21.y - 1.0) > SM_EFF_ZERO) { SER(SM_ERR); }
    if (smos_Fabs(v21.z) > SM_EFF_ZERO) { SER(SM_ERR); }

    v21.Set(3, -5, 1);
    dDouble = v21.SumFabs();
    if (smos_Fabs(dDouble - 9.0) > SM_EFF_ZERO) { SER(SM_ERR); }

    tmpVec0.Set(0, 3, 6);
    tmpVec1.Set(5, 2, 9);
    eStat = tmpVec0.SolveTwoLinearEquations(tmpVec1, tmpVec2d);
    if (smos_Fabs(tmpVec2d.x - 1.0) > SM_EFF_ZERO) { SER(SM_ERR); }
    if (smos_Fabs(tmpVec2d.y - 2.0) > SM_EFF_ZERO) { SER(SM_ERR); }

    // test BaryCentric(): requires SmTArray.
    SmTArray< SmVector3d > sVecArray;
    SmTArray< double     > sWtsArray;
    sVecArray.Add(tmpVec0);
    sVecArray.Add(tmpVec1);
    sVecArray.Add(tmpVec2);
    eStat = v21.BaryCentric(sVecArray, sWtsArray);
    SER(eStat);
    if (sWtsArray.GetSize() != 3) { SER(SM_ERR); }
    if (smos_Fabs(sWtsArray[0] - 0.37955232524702731) > SM_EFF_ZERO) { SER(SM_ERR); }
    if (smos_Fabs(sWtsArray[1] - 0.46691726514122972) > SM_EFF_ZERO) { SER(SM_ERR); }
    if (smos_Fabs(sWtsArray[2] - 0.15353040961174297) > SM_EFF_ZERO) { SER(SM_ERR); }

    v21.Set(2, 3, 1);
    v22.Set(4, 1, 5);
    v21.Swap(v22);
    if (smos_Fabs(v21.x - 4.0) > SM_EFF_ZERO) { SER(SM_ERR); }
    if (smos_Fabs(v21.y - 1.0) > SM_EFF_ZERO) { SER(SM_ERR); }
    if (smos_Fabs(v21.z - 5.0) > SM_EFF_ZERO) { SER(SM_ERR); }
    if (smos_Fabs(v22.x - 2.0) > SM_EFF_ZERO) { SER(SM_ERR); }
    if (smos_Fabs(v22.y - 3.0) > SM_EFF_ZERO) { SER(SM_ERR); }
    if (smos_Fabs(v22.z - 1.0) > SM_EFF_ZERO) { SER(SM_ERR); }

    v23 = v21 * v22;  //## test cross product operator
    if (smos_Fabs(v23.x + 14.0) > SM_EFF_ZERO) { SER(SM_ERR); }
    if (smos_Fabs(v23.y - 6.0) > SM_EFF_ZERO) { SER(SM_ERR); }
    if (smos_Fabs(v23.z - 10.0) > SM_EFF_ZERO) { SER(SM_ERR); }

    tmpVec0.Set(3, 6, 5);
    tmpVec1.Set(3, 6, 5);
    bBool = (tmpVec0 == tmpVec1);   //##test equality operator
    if (!bBool) { SER(SM_ERR); }

    tmpVec1.Set(3, 7, 5);
    bBool = (tmpVec0 != tmpVec1);   //##test inequality operator
    if (!bBool) { SER(SM_ERR); }

    tmpVec0.Dump(FALSE);            //##test Dump( SmBoolean )

    //SmStatus Write(FILE *)  const;

#ifdef SM_GFX_CODE
    if (smGet_DoGraphics()) {
        SmContext sContext;
        tmpVec0.DrawPointToPoint(tmpVec1, &sContext);  //##test DrawPointToPoint with default arg
        tmpVec0.DrawPointToPoint(tmpVec1);             //##test DrawPointToPoint without default arg
        tmpVec0.DrawPlane(tmpVec1, &sContext);  //##test DrawPlane with default arg
        tmpVec0.DrawPlane(tmpVec1);             //##test DrawPlane without default arg
    }
#endif


  return SM_SUCCESS;

} // end my_test_vector3d

/***********************************************************************
PURPOSE ---

USAGE NOTES ---
***********************************************************************/
PT_EXPORT SmStatus my_test_axis2placement()
{
    {
        SmAxis2Placement sAP;
        SmPoint3d sOrigin(0, 0, 0);
        SmVector3d sXAxis(1, 0, 0);
        SmVector3d sYAxis(0, 1, 0);
        SmVector3d sZAxis(0, 0, 1);
        SER(sAP.SetCanonical(sOrigin, sXAxis, sYAxis));
        sAP.RotateAboutAxis(20.0*SM_PI/180.0, sZAxis);
        sAP.Dump();
        SmContext sContext;
        SmBSplineCurve *pLine = my_create_line(sContext, SmPoint3d(0, 0, 0),
            SmPoint3d(1, 0, 0), 1, 0, 0);
        SmObjDelete sClean( pLine );
        SER(pLine->Transform(sAP));
        pLine->Dump();
#ifdef SM_GFX_CODE
        if (smGet_DoGraphics())
        {
            pLine->Draw();
        }
#endif
    }
    {
        SmAxis2Placement sAP;
        SmPoint3d sOrigin(1, 2, 3);
        SmVector3d sXAxis(0, 1, 0);
        SmVector3d sYAxis(0, 0, 1);
        SER(sAP.SetCanonical(sOrigin, sXAxis, sYAxis));
        SmPoint3d sOrigin2;
        SmVector3d sXAxis2, sYAxis2;
        sAP.GetCanonical(sOrigin2, sXAxis2, sYAxis2);
        
        sAP.Translate(sOrigin);
        sAP.RotateAboutAxis(SM_PI, sXAxis2);
        SmPoint3d sNewOrig = sAP.GetOrigin();
        SmVector3d sZAxis = sAP.GetZAxis();
        double dXAng, dYAng, dZAng;
        sAP.DecomposeToAngles(dXAng, dYAng, dZAng);
        
        SmAxis2Placement sAPInv;
        sAP.Invert(sAPInv);
        SmPoint3d sPnt(0, 0, 0);
        sAP.TransformPoint(sPnt, sPnt);
        sAPInv.TransformPoint(sPnt, sPnt);
        if (sPnt.Dot(sPnt) > SM_EFF_ZERO)
            SER(SM_ERR);
        SmVector3d sVec(1, 0, 0);
        sAP.TransformVector(sVec, sVec);
        sAPInv.TransformVector(sVec, sVec);
        if (smos_Fabs(sVec.x - 1.0) > SM_EFF_ZERO)
            SER(SM_ERR);
        SmAxis2Placement sIdentity;
        sAP.TransformAxis2Placement(sAPInv, sIdentity);
        
#ifdef SM_GFX_CODE
        if (smGet_DoGraphics())
            sAP.Draw();
#endif
        return SM_SUCCESS;
    }
} // end my_test_axis2placement

/***********************************************************************
PURPOSE --- Unit tests for class SmExtent1d

USAGE NOTES ---
   These tests are quite comprehensive in that they call pretty much
   every method, but they are not all functionally exhaustive, testing
   all behaviors.  Most were added for the Java project, for which we
   need a wrapper for every method.
***********************************************************************/
PT_EXPORT SmStatus my_test_extent1d(void)
{
    SmExtent1d e1(1, 2);
    if (e1.GetMin() != 1.0 || e1.GetMax() != 2.0)
      { SER(SM_ERR); }
    SmExtent1d e2 = e1;
    if (e2.GetMin() != 1.0 || e2.GetMax() != 2.0)
      { SER(SM_ERR); }

  SmBoolean bOk;

    if ( ! ( e2 == e1 ) )
      { SER(SM_ERR); }

    SmExtent1d e77 ( e1 );
    if (e77.GetMin() != 1.0 || e77.GetMax() != 2.0)
      { SER(SM_ERR); }
    if ( ! ( e77 == e1 ) )
      { SER(SM_ERR); }

    e77.Dump();
    smos_WriteBuffer( _T("\n"), _T("\n") );

    bOk = e77.AssertValid();
    if ( ! bOk )
      { SER(SM_ERR); }

    bOk = e77.AssertDefined();
    if ( ! bOk )
      { SER(SM_ERR); }

    SM_TYPE lType = e77.GetType();
    if ( lType != SmExtent1d_TYPE )
      { SER(SM_ERR); }

    e77.Init();
    bOk = e77.IsInit();
    if ( ! bOk )
      { SER(SM_ERR); }

    bOk = e77.AssertDefined();
    if ( ! bOk ) // should still be 'defined'.
      { SER(SM_ERR); }


    SmExtent1d e3(4);
    if (e3.GetMin() != 4.0 || e3.GetMax() != 4.0)
      { SER(SM_ERR); }
    SmExtent1d e4;
    e4.AddValue(6);
    e4.AddValue(5);
    e4.AddValue(7);
    if (e4.GetMin() != 5.0 || e4.GetMax() != 7.0)
      { SER(SM_ERR); }
    SmExtent1d e5;
    e5.SetMinMax(8, 10);
    if (e5.GetMin() != 8.0 || e5.GetMax() != 10.0)
      { SER(SM_ERR); }
    if (! e4.AreDisjoint(e5))
      { SER(SM_ERR); }
    if (smos_Fabs(e4.MinimumDistance(e5) - 1.0) > SM_EFF_ZERO)
      { SER(SM_ERR); }


    if (smos_Fabs(e4.MaximumDistance(e5) - 5.0) > SM_EFF_ZERO)
      { SER(SM_ERR); }


    SmExtent1d sUnion;
    e4.Union(e5, sUnion);
    e5.AddValue(7);
    if (e4.AreDisjoint(e5))
      { SER(SM_ERR); }
    e5.AddValue(5);
    if (e4.AreDisjoint(e5))
      { SER(SM_ERR); }
    e5.AddValue(2);
    if (e4.AreDisjoint(e5))
      { SER(SM_ERR); }
    SmExtent1d sIntersect;
      { SER(e4.Intersect(e5, sIntersect)); }
    

  // Union was created before e5 was expanded.
  bOk = ( smos_Fabs( sUnion.GetMin() - e4.GetMin() )) < SM_EFF_ZERO;
  if ( ! bOk )
    { SER(SM_ERR); }
  bOk = ( smos_Fabs( sUnion.GetMax() - e5.GetMax() )) < SM_EFF_ZERO;
  if ( ! bOk )
    { SER(SM_ERR); }

  // Intersect was created after e5 was expanded.
  bOk = ( smos_Fabs( sIntersect.GetMin() - e4.GetMin() ) < SM_EFF_ZERO);
  if ( ! bOk )
    { SER(SM_ERR); }
  bOk = ( smos_Fabs( sIntersect.GetMax() - e4.GetMax() ) < SM_EFF_ZERO);
  if ( ! bOk )
    { SER(SM_ERR); }

  double dVal = e4.GetMid();
  if ( smos_Fabs( dVal - 6.0 ) > SM_EFF_ZERO )
    { SER(SM_ERR); }
  dVal = e4.GetLength();
  if ( smos_Fabs( dVal - 2.0 ) > SM_EFF_ZERO )
    { SER(SM_ERR); }
  dVal = e4.GetMaxDimension();
  if ( smos_Fabs( dVal - 7.0 ) > SM_EFF_ZERO )
    { SER(SM_ERR); }
  dVal = e4.MaximumDistance( e5 );
  if ( smos_Fabs( dVal - 8.0 ) > SM_EFF_ZERO )
    { SER(SM_ERR); }

  double dExpFactor = 0.1;
  SmExtent1d e6 = e4;
  SmExtent1d & e6Ref = e6.ExpandAbsolute( dExpFactor );
  SmExtent1d e7( e4.GetMin() - dExpFactor, e4.GetMax() + dExpFactor );
  bOk = ( e7 == e6Ref );
  if ( ! bOk )
    { SER(SM_ERR); }

  SmExtent1d  e8 = e5;
  SmExtent1d &e8Ref = e8.ExpandRelative( dExpFactor );
  double dExpRel = e5.GetLength() * dExpFactor;
  SmExtent1d e9( e5.GetMin() - dExpRel, e5.GetMax() + dExpRel);
  bOk = e9.AreEqual( e8Ref, SM_EFF_ZERO );
  if ( ! bOk )
    { SER(SM_ERR); }

  bOk = e8.AreEqual( e5 );  // default arg
  if ( bOk )  // S/B False
    { SER(SM_ERR); }

  bOk = e8.AreEqual( e9.GetMin(), e9.GetMax(), SM_EFF_ZERO );
  if ( ! bOk )
    { SER(SM_ERR); }

  bOk = e8.AreEqual( e5.GetMin(), e5.GetMax() ); // default arg
  if ( bOk )  // Should be False
    { SER(SM_ERR); }

  double dTest = e4.GetMin() - 1.0;
  dVal = e4.ClampValue( dTest );
  if ( smos_Fabs( dVal - e4.GetMin() ) > SM_EFF_ZERO )
    { SER(SM_ERR); }

  dTest = 0.35;
  dVal = e4.Evaluate( dTest );
  double dCheck = e4.GetMin() + dTest * e4.GetLength();
  if ( smos_Fabs( dCheck - dVal ) > SM_EFF_ZERO )
    { SER(SM_ERR); }

  SmStatus eStat = e4.Inversion( dVal, dCheck );
  SER( eStat );
  if ( smos_Fabs( dCheck - dTest ) > SM_EFF_ZERO )
    { SER(SM_ERR); }

  bOk = e4.GetTLeftEval( dVal );
  if ( ! bOk )
    { SER(SM_ERR); }

  dVal = e4.Evaluate( 0.65 );
  bOk = e4.GetTLeftEval( dVal );
  if ( bOk )
    { SER(SM_ERR); }

  bOk = e4.IsContainedBy( e6, SM_EFF_ZERO );
  if ( ! bOk )
    { SER(SM_ERR); }

  bOk = e5.IsContainedBy( e6 ); // default arg
  if ( bOk )
    { SER(SM_ERR); }
  
  dTest = 0.35;
  dVal = e4.Evaluate( dTest );
  bOk = e4.ContainsValue( dVal, SM_EFF_ZERO ); // default arg
  if ( ! bOk )
    { SER(SM_ERR); }

  bOk = e4.ContainsValue( dVal ); // default arg
  if ( ! bOk )
    { SER(SM_ERR); }

  bOk = e4.IsValueOnBoundary( dVal, SM_EFF_ZERO );
  if ( bOk )
    { SER(SM_ERR); }

  dTest = e4.GetMin();
  bOk = e4.IsValueOnBoundary( dTest );  // default arg
  if ( ! bOk )
    { SER(SM_ERR); }

//---

  SmExtentPointType eType = SM_EP_UNKNOWN;

  eType = e4.ClassifyPoint( 6.0, 0.000001 );
  if ( eType != SM_EP_INSIDE )
    { SER(SM_ERR); }

  eType = e4.ClassifyPoint( 3.0 );  // default arg
  if ( eType != SM_EP_OUTSIDE )
    { SER(SM_ERR); }

  bOk = e4.HasNegativeLength();
  if ( bOk )
    { SER(SM_ERR); }

  dCheck = e4.PeriodicWrap( 9.5 );
  if ( fabs( dCheck - 5.5 ) > SM_EFF_ZERO )
    { SER(SM_ERR); }

  dCheck = e4.ClampPeriodicValue( 21.5, 5.0, 0.000001 );
  if ( fabs( dCheck - 6.5 ) > SM_EFF_ZERO )
    { SER(SM_ERR); }

  dCheck = e4.ClampPeriodicValue( 21.5, 4.0 );  // default arg
  if ( fabs( dCheck - 5.5 ) > SM_EFF_ZERO )
    { SER(SM_ERR); }

  SmTArray<SmExtent1d> sExtArray;

  eStat = e4.IntersectPeriodic( e2, 2.0, sExtArray, TRUE );
  if ( eStat != SM_SUCCESS )
    { SER(SM_ERR); }
  if ( sExtArray.GetSize() != 1 )
    { SER(SM_ERR); }
  SmExtent1d sIntExt = sExtArray[0];
  dCheck = sIntExt.GetMin();
  if ( fabs( dCheck - 5.0 ) > SM_EFF_ZERO )
    { SER(SM_ERR); }
  dCheck = sIntExt.GetMax();
  if ( fabs( dCheck - 6.0 ) > SM_EFF_ZERO )
    { SER(SM_ERR); }

  eStat = e4.IntersectPeriodic( e2, 5.0, sExtArray ); // default arg
  if ( eStat != SM_SUCCESS )
    { SER(SM_ERR); }
  if ( sExtArray.GetSize() != 1 )
    { SER(SM_ERR); }
  sIntExt = sExtArray[0];
  dCheck = sIntExt.GetMin();
  if ( fabs( dCheck - 6.0 ) > SM_EFF_ZERO )
    { SER(SM_ERR); }
  dCheck = sIntExt.GetMax();
  if ( fabs( dCheck - 7.0 ) > SM_EFF_ZERO )
    { SER(SM_ERR); }

  bOk = e4.IsClosed( 2.0 );  // 2.0 is 'period'.
  if ( ! bOk )
    { SER(SM_ERR); }

  bOk = e4.AreDisjointPeriodic( e2, 4.0, 0.000001 );
  if ( bOk )
    { SER(SM_ERR); }
  bOk = e4.AreDisjointPeriodic( e2, 10.0 );  // default arg
  if ( ! bOk )
    { SER(SM_ERR); }

  double dMin, dMax;

  bOk = e4.ContainsPeriodicValue( 29.5, 4.0, &dCheck, &dMin, &dMax, 0.000001 );
  if ( ! bOk )
    { SER(SM_ERR); }
  if ( fabs( dCheck - 5.5 ) > SM_EFF_ZERO )
    { SER(SM_ERR); }
  if ( fabs( dMin   - 0.5 ) > SM_EFF_ZERO )
    { SER(SM_ERR); }
  if ( fabs( dMax   - 1.5 ) > SM_EFF_ZERO )
    { SER(SM_ERR); }

  bOk = e4.ContainsPeriodicValue( 29.5, 4.0, &dCheck, &dMin, &dMax );
  if ( ! bOk )
    { SER(SM_ERR); }
  if ( fabs( dCheck - 5.5 ) > SM_EFF_ZERO )
    { SER(SM_ERR); }
  if ( fabs( dMin   - 0.5 ) > SM_EFF_ZERO )
    { SER(SM_ERR); }
  if ( fabs( dMax   - 1.5 ) > SM_EFF_ZERO )
    { SER(SM_ERR); }

  bOk = e4.ContainsPeriodicValue( 29.5, 4.0, &dCheck, &dMin );
  if ( ! bOk )
    { SER(SM_ERR); }
  if ( fabs( dCheck - 5.5 ) > SM_EFF_ZERO )
    { SER(SM_ERR); }
  if ( fabs( dMin   - 0.5 ) > SM_EFF_ZERO )
    { SER(SM_ERR); }

  bOk = e4.ContainsPeriodicValue( 29.5, 4.0, &dCheck );
  if ( ! bOk )
    { SER(SM_ERR); }
  if ( fabs( dCheck - 5.5 ) > SM_EFF_ZERO )
    { SER(SM_ERR); }

  bOk = e4.ContainsPeriodicValue( 29.5, 5.0 );
  if ( bOk )
    { SER(SM_ERR); }

  bOk = e4.IsValueOnPeriodicBoundary( 365.5, 360, 0.000001, &dCheck );
  if ( bOk )
    { SER(SM_ERR); }  // S/B False
  if ( fabs( dCheck - 0.5 ) > 0.000001 )
    { SER(SM_ERR); }

  bOk = e4.IsValueOnPeriodicBoundary( 365.5, 360, 0.000001 );
  if ( bOk )
    { SER(SM_ERR); }  // S/B False

  bOk = e4.IsValueOnPeriodicBoundary( 365, 360 );  // default args
  if ( ! bOk )
    { SER(SM_ERR); }


    return SM_SUCCESS;
} // end my_test_extent1d



/***********************************************************************
PURPOSE ---

USAGE NOTES ---
***********************************************************************/
PT_EXPORT SmStatus my_test_extent2d()
{
//##test c'tor: 2-point
  SmVector2d v1(  1, 2 );
  SmVector2d v2(  3, 4 );
  SmExtent2d e1( v1, v2 );

//##test GetMin : e1( (1,2), (3,4) )
  SmVector2d v3;
  v3 = e1.GetMin();
  if ( v3.x != 1.0 || v3.y != 2.0 ) { SER(SM_ERR); }

//##test GetMax : e1( (1,2), (3,4) )
  v3 = e1.GetMax();
  if ( v3.x != 3.0 || v3.y != 4.0 ) { SER(SM_ERR); }

//##test c'tor: default : #define SM_BIG_DOUBLE 1.0e20  (SmMath.h)
  SmExtent2d e2;
  //      v3 = e2.GetMin();
  //      if ( v3.x != SM_BIG_DOUBLE || v3.y != SM_BIG_DOUBLE ) { SER(SM_ERR); }

//##test operator=() : e1( (1,2), (3,4) )
  e2 = e1;
  v3 = e2.GetMin();
  if ( v3.x != 1.0 || v3.y != 2.0 ) { SER(SM_ERR); }

//##test c'tor: 4 doubles
  SmExtent2d e3( 11, 12, 13, 14 );
  v3 = e3.GetMin();
  if ( v3.x != 11.0 || v3.y != 12.0 ) { SER(SM_ERR); }
  v3 = e3.GetMax();
  if ( v3.x != 13.0 || v3.y != 14.0 ) { SER(SM_ERR); }

//##test c'tor: 1-point : v1(1,2)
  SmExtent2d e4( v1 );
  v3 = e4.GetMin();
  if ( v3.x != 1.0 || v3.y != 2.0 ) { SER(SM_ERR); }
  v3 = e4.GetMax();
  if ( v3.x != 1.0 || v3.y != 2.0 ) { SER(SM_ERR); }

  // Scope to test destructor:
  {
//##test c'tor: copy ctor( SmExtent2d ) : e1( (1,2), (3,4) )
      SmExtent2d e5( e1 );
      v3 = e5.GetMin();
      if ( v3.x != 1.0 || v3.y != 2.0 ) { SER(SM_ERR); }

//##test c'tor: copy from SmExtent3d( SmExtent3d )
      SmExtent3d sExt3d( 1, 2, 3, 4, 5, 6 );
      SmExtent2d e6( sExt3d );
      v3 = e6.GetMax();
      if ( v3.x != 4.0 || v3.y != 5.0 ) { SER(SM_ERR); }

//##test destructor: exit scope:
  }

//##test Init()
  e2.Init();
  //      v3 = e2.GetMin();
  //      if ( v3.x !=  SM_BIG_DOUBLE || v3.y !=  SM_BIG_DOUBLE ) { SER(SM_ERR); }
  //      v3 = e2.GetMax();
  //      if ( v3.x != -SM_BIG_DOUBLE || v3.y != -SM_BIG_DOUBLE ) { SER(SM_ERR); }

//##test IsInit() : 
  SmBoolean bFlag = e2.IsInit();
  if ( ! bFlag ) { SER(SM_ERR); }

//##test AssertDefined() : doesn't matter what's in e2.
  bFlag = e2.AssertDefined();
  if ( ! bFlag ) { SER(SM_ERR); }

//##test AddPoint2d() : e1( (1,2), (3,4) )
  e2 = e1;
  v3.Set( 5, 6 );
  e2.AddPoint2d( v3 );
  v3 = e2.GetMax();
  if ( v3.x != 5.0 || v3.y != 6.0 ) { SER(SM_ERR); }

//##test SetMinMax( 4 doubles )
  e2.SetMinMax( 1, 2, 3, 4 );
  v3 = e2.GetMin();
  if ( v3.x != 1.0 || v3.y != 2.0 ) { SER(SM_ERR); }

//##test SetMinMax( 2 pts ) : v1(1,2), v2(3,4)
  e2.SetMinMax( v1, v2 );
  v3 = e2.GetMin();
  if ( v3.x != 1.0 || v3.y != 2.0 ) { SER(SM_ERR); }
  
//##test AreDisjoint() with default argument: e1( (1,2), (3,4) )
  SmExtent2d e6(  -3, -2,  0, 1 );
  bFlag = e1.AreDisjoint( e6, 2.0 );
  if (   bFlag ) { SER(SM_ERR); }   // Not disjoint at this tolerance.
  
//##test AreDisjoint() without default argument: e1( 1, 2, 3, 4 ),  e6( -3, -2,  0, 1 );
  bFlag = e1.AreDisjoint( e6 );
  if ( ! bFlag ) { SER(SM_ERR); }   // Disjoint at default tolerance.

//##test MinimumDistance() :  e1( (1,2), (3,4) )
  e6.SetMinMax ( -4, -2, -3, -1 );
  double dTest = e1.MinimumDistance( e6 );
  if ( dTest != 5.0 ) { SER(SM_ERR); }

//##test MaximumDistance() :  e1( (1,2), (3,4) ), e6( (-4,-2), (-3,-1) )
  dTest = e1.MaximumDistance( e6 );
  if ( dTest * dTest != 85.0 ) { SER(SM_ERR); }

//##test MinimumDistanceSquared() :  e1( (1,2), (3,4) ), e6( (-4,-2), (-3,-1) )
  dTest = e1.MinimumDistanceSquared( e6 );
  if ( dTest != 25.0 ) { SER(SM_ERR); }

//##test MaximumDistanceSquared() :  e1( (1,2), (3,4) ), e6( (-4,-2), (-3,-1) )
  dTest = e1.MaximumDistanceSquared( e6 );
  if ( dTest != 85.0 ) { SER(SM_ERR); }

//##test Union() :  e1( (1,2), (3,4) ), e6( (-4,-2), (-3,-1) )
  e1.Union( e6, e3 );
  v3 = e3.GetMin();
  if ( v3.x != -4.0 || v3.y != -2.0 ) { SER(SM_ERR); }
  v3 = e3.GetMax();
  if ( v3.x !=  3.0 || v3.y !=  4.0 ) { SER(SM_ERR); }
  
//##test Intersect() :  e1( (1,2), (3,4) )
  e3.SetMinMax( 0, 0, 4, 3 );
  SmStatus eStat = e1.Intersect( e3, e4 );
  SER( eStat );
  v3 = e4.GetMax();
  if ( v3.x != 3.0 || v3.y != 3.0 ) { SER(SM_ERR); }

//##test SetUInterval() :  e2( (1,2), (3,4) )
  SmExtent1d sExt1d( 7, 8 );
  e2.SetUInterval( sExt1d );
  v3 = e2.GetMin();
  if ( v3.x != 7.0 ) { SER(SM_ERR); }
  v3 = e2.GetMax();
  if ( v3.x != 8.0 ) { SER(SM_ERR); }

//##test SetVInterval() :  e2( (7,2), (8,4) )
  sExt1d.SetMinMax( 4, 5 );
  e2.SetVInterval( sExt1d );
  v3 = e2.GetMin();
  if ( v3.y != 4.0 ) { SER(SM_ERR); }
  v3 = e2.GetMax();
  if ( v3.y != 5.0 ) { SER(SM_ERR); }

//##test SetUMin()      :  e2( (7,4), (8,5) )
  e2.SetUMin( 2 );
  v3 = e2.GetMin();
  if ( v3.x != 2.0 ) { SER(SM_ERR); }

//##test SetUMax()      :  e2( (2,4), (8,5) )
  e2.SetUMax( 5 );
  v3 = e2.GetMax();
  if ( v3.x != 5.0 ) { SER(SM_ERR); }

//##test SetVMin()      :  e2( (2,4), (5,5) )
  e2.SetVMin( 3 );
  v3 = e2.GetMin();
  if ( v3.y != 3.0 ) { SER(SM_ERR); }

//##test SetVMax()      :  e2( (2,3), (5,5) )
  e2.SetVMax( 6 );
  v3 = e2.GetMax();
  if ( v3.y != 6.0 ) { SER(SM_ERR); }

//##test Transpose()    :  e2( (2,3), (5,6) )
  e2.Transpose();
  v3 = e2.GetMin();
  if ( v3.x != 3.0 || v3.y != 2.0 ) { SER(SM_ERR); }

//##test ExpandAbsolute() : e2( (3,2), (6,5) )
  e2.ExpandAbsolute( 1.0 );
  v3 = e2.GetMin();
  if ( v3.x != 2.0 || v3.y != 1.0 ) { SER(SM_ERR); }

//##test ExpandRelative() : e2( (2,1), (7,6) )
  e2.ExpandRelative( 1.0 );
  v3 = e2.GetMin();
  if ( v3.x != -3.0 || v3.y != -4.0 ) { SER(SM_ERR); }

//##test GetUMin() : e2( (-3,-4), (12,11) )
  dTest = e2.GetUMin();
  if ( dTest != -3.0 ) { SER(SM_ERR); }

//##test GetUMax() : e2( (-3,-4), (12,11) )
  dTest = e2.GetUMax();
  if ( dTest != 12.0 ) { SER(SM_ERR); }

//##test GetVMin() : e2( (-3,-4), (12,11) )
  dTest = e2.GetVMin();
  if ( dTest != -4.0 ) { SER(SM_ERR); }

//##test GetVMax() : e2( (-3,-4), (12,11) )
  dTest = e2.GetVMax();
  if ( dTest != 11.0 ) { SER(SM_ERR); }

//##test GetSize() : e2( (-3,-4), (12,11) )
  v3 = e2.GetSize();
  if ( v3.x != 15.0 || v3.y != 15.0 ) { SER(SM_ERR); }

//##test GetUInterval() : e2( (-3,-4), (12,11) )
  sExt1d = e2.GetUInterval();
  if ( sExt1d.GetMin() != -3.0 || sExt1d.GetMax() != 12.0 ) { SER(SM_ERR); }

//##test GetVInterval() : e2( (-3,-4), (12,11) )
  sExt1d = e2.GetVInterval();
  if ( sExt1d.GetMin() != -4.0 || sExt1d.GetMax() != 11.0 ) { SER(SM_ERR); }

//##test XLength() : e2( (-3,-4), (12,11) )
  dTest = e2.XLength();
  if ( dTest != 15.0 ) { SER(SM_ERR); }

//##test YLength() : e2( (-3,-4), (12,11) )
  dTest = e2.YLength();
  if ( dTest != 15.0 ) { SER(SM_ERR); }

//##test GetMaxDimension() : e2( (-3,-4), (12,11) )
  dTest = e2.GetMaxDimension();
  if ( dTest != 12.0 ) { SER(SM_ERR); }

//##test AreEqual() with default argument: e1( 1, 2, 3, 4 )
  e2.SetMinMax( 1, 2.1, 3, 4 );
  bFlag = e1.AreEqual( e2, 0.2 );
  if ( ! bFlag ) { SER(SM_ERR); }   // Equal at this tolerance.

//##test AreEqual() without default argument: e1( 1, 2, 3, 4 ),  e2( 1, 2.1, 3, 4 )
  bFlag = e1.AreEqual( e2 );
  if (   bFlag ) { SER(SM_ERR); }   // Not equal at default tolerance.

//##test operator==() : e1( 1, 2, 3, 4 )
  e2 = e1;
  bFlag = (e1 == e2);
  if ( ! bFlag ) { SER(SM_ERR); }

//##test IsDegenerate( ) with default argument:
  e2.SetMinMax( 4, 5, 4.1, 5.1 );
  bFlag = e2.IsDegenerate( 0.2 );
  if ( ! bFlag ) { SER(SM_ERR); }   // Degenerate at this tolerance.

//##test IsDegenerate() without default argument:   e2( 4, 5, 4.1, 5.1 )
  bFlag = e2.IsDegenerate();
  if (   bFlag ) { SER(SM_ERR); }   // Not degenerate at default tolerance.

//##test HasNegativeArea()
  e2.SetMinMax( 0, 0, 0, 0 );
  e2.ExpandAbsolute( -0.1 );
  bFlag = e2.HasNegativeArea();
  if ( ! bFlag ) { SER(SM_ERR); }

//##test IsContainedBy() with default argument: e1( (1,2), (3,4) )
  e2 = e1;
  e2.ExpandAbsolute( 0.01 );
  bFlag = e2.IsContainedBy( e1, 0.1 );
  if ( ! bFlag ) { SER(SM_ERR); } // is contained at this tolerance.

//##test IsContainedBy() without default argument: e1( (1,2), (3,4) )
  e2 = e1;
  e2.ExpandAbsolute( 0.01 );
  bFlag = e2.IsContainedBy( e1 );
  if (   bFlag ) { SER(SM_ERR); } // not contained at default tolerance.

//##test ContainsPoint2d() with default argument: e1( 1, 2, 3, 4 )
  v3.Set( 0.9, 3 );
  bFlag = e1.ContainsPoint2d( v3, 0.2 );
  if ( ! bFlag ) { SER(SM_ERR); }   // is contained at this tolerance.

//##test ContainsPoint2d() without default argument: e1( 1, 2, 3, 4 ), v3( 0.9, 3 );
  bFlag = e1.ContainsPoint2d( v3 );
  if (   bFlag ) { SER(SM_ERR); } // not contained at default tolerance.

//##test ClassifyPoint2d() with default argument: e1( (1,2), (3,4) )
  v3.Set( 1.1, 2.1 );
  SmExtentPointType eTypeU, eTypeV;
  e2.ClassifyPoint2d( v3, eTypeU, eTypeV, 0.2 );
  if ( eTypeU != SM_EP_START ) { SER(SM_ERR); } // at start, at this tol
  if ( eTypeV != SM_EP_START ) { SER(SM_ERR); }

//##test ClassifyPoint2d() without default argument: e1( (1,2), (3,4) )
  v3.Set( 1.1, 2.1 );
  e2.ClassifyPoint2d( v3, eTypeU, eTypeV );
  if ( eTypeU != SM_EP_INSIDE ) { SER(SM_ERR); } // inside, at default tol
  if ( eTypeV != SM_EP_INSIDE ) { SER(SM_ERR); }

//##test IsTouchingOnePointAt2DCorner() with default argument : e1( (1,2), (3,4) )
  e2.SetMinMax( 3.01, 4.01, 8, 9 );
  bFlag = e1.IsTouchingOnePointAt2DCorner( e2, 0.1 );
  if ( ! bFlag ) { SER(SM_ERR); } // is touching at this tol

//##test IsTouchingOnePointAt2DCorner() without default argument : e1( (1,2), (3,4) )
  e2.SetMinMax( 3.01, 4.01, 8, 9 );
  bFlag = e1.IsTouchingOnePointAt2DCorner( e2 );
  if (   bFlag ) { SER(SM_ERR); } // not touching at default tol

//##test IsPoint2dOnBoundary() with two default arguments: e1( (1,2), (3,4) )
  v3.Set( 1.1, 2.1 );
  SmVector2d v4;
  bFlag = e1.IsPoint2dOnBoundary( v3, 0.2, &v4 );
  if ( ! bFlag ) { SER(SM_ERR); } // is on boundary, at this tol
  if ( v4.x != 1.0 || v4.y != 1.0 ) { SER(SM_ERR); }

//##test IsPoint2dOnBoundary() with one default argument: e1( (1,2), (3,4) ),  v3( 1.1, 2.1 );
  bFlag = e1.IsPoint2dOnBoundary( v3, 0.2 );
  if ( ! bFlag ) { SER(SM_ERR); } // is on boundary, at this tol

//##test IsPoint2dOnBoundary() with no default arguments: e1( (1,2), (3,4) ),  v3( 1.1, 2.1 );
  bFlag = e1.IsPoint2dOnBoundary( v3 );
  if (   bFlag ) { SER(SM_ERR); } // not on boundary, at default tol

//##test ContainsPeriodicPoint2d() with two default arguments: e1( (1,2), (3,4) )
  v3.Set( 8.9, 3.0 );
  bFlag = e1.ContainsPeriodicPoint2d( v3, 4, 0, 0.2, &v4 );
  if ( ! bFlag ) { SER(SM_ERR); } // does contain it, at this tol
  if ( v4.x != 1.0 || v4.y != 3.0 ) { SER(SM_ERR); }

//##test ContainsPeriodicPoint2d() with one default argument: e1( (1,2), (3,4) ), v3( 8.9, 3.0 );
  bFlag = e1.ContainsPeriodicPoint2d( v3, 4, 0, 0.2 );
  if ( ! bFlag ) { SER(SM_ERR); } // does contain it, at this tol

//##test ContainsPeriodicPoint2d() with no default arguments: e1( (1,2), (3,4) ), v3( 8.9, 3.0 );
  bFlag = e1.ContainsPeriodicPoint2d( v3, 4, 0 );
  if (   bFlag ) { SER(SM_ERR); } // does not contain it, at default tol

//##test AreDisjointPeriodic() : e1( (1,2), (3,4) )
  e2.SetMinMax( 10, 0, 12, 4 );
  bFlag = e1.AreDisjointPeriodic( e2, 4, 0 );
  if (   bFlag ) { SER(SM_ERR); }

//##test IntersectPeriodic() with default argument: e1( (1,2), (3,4) )
  e2.SetMinMax( 2, 0, 6, 2 );
  SmTArray< SmExtent2d > sExtArray;
  eStat = e2.IntersectPeriodic( e1, 4, 0, sExtArray, TRUE );
  SER( eStat );
  if ( sExtArray.GetSize() != 2 ) { SER(SM_ERR); }
  v3 = sExtArray[0].GetMin();
  if ( v3.x != 2.0 || v3.y != 2.0 ) { SER(SM_ERR); }
  v3 = sExtArray[0].GetMax();
  if ( v3.x != 3.0 || v3.y != 2.0 ) { SER(SM_ERR); }

//##test IntersectPeriodic() without default argument: e1( (1,2), (3,4) )
  eStat = e2.IntersectPeriodic( e1, 4, 0, sExtArray );
  SER( eStat );
  if ( sExtArray.GetSize() != 1 ) { SER(SM_ERR); }

//##test Evaluate() : e1( (1,2), (3,4) )
  v3 = e1.Evaluate( 0.5, 0.5 );
  if ( v3.x != 2.0 || v3.y != 3.0 ) { SER(SM_ERR); }

//##test Inversion() with default arg: e1( (1,2), (3,4) ), v3( 2, 3 )
  eStat = e1.Inversion( v3, v4, 0.1 );
  SER( eStat );
  if ( v4.x != 0.5 || v4.y != 0.5 ) { SER(SM_ERR); }

//##test Inversion() without default arg: e1( (1,2), (3,4) ), v3( 2, 3 )
  eStat = e1.Inversion( v3, v4 );
  SER( eStat );
  if ( v4.x != 0.5 || v4.y != 0.5 ) { SER(SM_ERR); }

//##test ClampPoint2d() : e1( (1,2), (3,4) )
  v4.Set( 0, 0 );
  v3 = e1.ClampPoint2d( v4 );
  if ( v3.x != 1.0 || v3.y != 2.0 ) { SER(SM_ERR); }

//##test ClipLine2d() : e1( (1,2), (3,4) )
  v3.Set( 2, 3 );
  v4.Set( 0, 1 );
  dTest = e1.ClipLine2d( v3, v4, 4.0 );
  if ( dTest != 1.0 ) { SER(SM_ERR); }

//##test IntersectWithInfiniteLine() with default arg: e1( (1,2), (3,4) )
  v3.Set( 0, 3.1 );
  v4.Set( 1, 1 );
  dTest = 0.2;
  eStat = e1.IntersectWithInfiniteLine( v3, v4, bFlag, sExt1d, &dTest );
  if ( ! bFlag ) { SER(SM_ERR); } // should intersect at this tol.
  if ( fabs( sExt1d.GetMin() - 0.9 ) > SM_EFF_ZERO ) { SER(SM_ERR); } // (has noise)
  if ( sExt1d.GetMax() != 1.0 ) { SER(SM_ERR); }

//##test IntersectWithInfiniteLine() without default arg: e1( (1,2), (3,4) ), v3( 0, 3.1 ), v4( 1, 1 )
  eStat = e1.IntersectWithInfiniteLine( v3, v4, bFlag, sExt1d );
  if (   bFlag ) { SER(SM_ERR); } // should not intersect at default tol.

//##test GetULeftEval() : e1( (1,2), (3,4) )
  dTest = 2.5;
  bFlag = e1.GetULeftEval( dTest );
  if (   bFlag ) { SER(SM_ERR); }

//##test GetVLeftEval() : e1( (1,2), (3,4) ), dTest( 2.5 )
  bFlag = e1.GetVLeftEval( dTest );
  if ( ! bFlag ) { SER(SM_ERR); }

//##test Point2dMapToDomain() : e1( (1,2), (3,4) )
  e2.SetMinMax( 4, 0, 10, 8 );
  v3.Set( 2, 3 );
  eStat = e1.Point2dMapToDomain( v3, e2, FALSE, v4 );
  SER( eStat );
  if ( v4.x != 7.0 || v4.y != 4.0 ) { SER(SM_ERR); }

  // Utilities
  e1.Dump();

#ifdef SM_GFX_CODE
  if (smGet_DoGraphics())
      e1.Draw();
#endif

  bFlag = e1.AssertValid();
  if ( ! bFlag ) { SER(SM_ERR); }

  SM_TYPE eType = e1.GetType();   // SmExtent2d_TYPE == 12090 (SmCoreTypes.h)
  if ( eType != SmExtent2d_TYPE ) { SER(SM_ERR); }

  eType = e1.GetClassType();
  if ( eType != SmExtent2d_TYPE ) { SER(SM_ERR); }

  const TCHAR *cTypeString = e1.GetTypeString();
  if ( smos_WStrCmp( cTypeString, _T("SmExtent2d_TYPE") ) ) { SER(SM_ERR); }

  const TCHAR *cTypeString2 = e1.GetClassTypeString();
  if ( smos_WStrCmp( cTypeString2, _T("SmExtent2d_TYPE") ) ) { SER(SM_ERR); }

  return SM_SUCCESS;
} // end my_test_extent2d

/***********************************************************************
PURPOSE ---

USAGE NOTES ---
***********************************************************************/
PT_EXPORT SmStatus my_test_extent3d()
{
    SmPoint3d v1(1, 1, 1);
    SmPoint3d v2(2, 2, 1);
    SmExtent3d e1(v1, v2);
    SmExtent3d e2 = e1;
    SmExtent3d e3(v2);
    SmExtent3d e4;
    SmPoint3d v3(6, 6, 6);
    SmPoint3d v4(5, 5, 5);
    SmPoint3d v5(7, 7, 7);
    e4.AddPoint3d(v3);
    e4.AddPoint3d(v4);
    e4.AddPoint3d(v5);
    
    SmExtent3d e5;
    e5.SetMinMax(v4, v5);
    SmPoint3d v7 = e5.GetMin();
    SmPoint3d v8 = e5.GetMax();
    
    SmExtent3d e6(-v5, - v4);
    if (! e5.AreDisjoint(e6))
        SER(SM_ERR);

    //double dist = e4.MinimumDistance(e5);

    SmExtent3d sUnion;
    e5.Union(e6, sUnion);
    
    SmExtent3d sIntersect;
    SmExtent3d e7(v4, v5);
    SER(e4.Intersect(e7, sIntersect));
    
#ifdef SM_GFX_CODE
    if (smGet_DoGraphics())
        sUnion.Draw();
#endif

    return SM_SUCCESS;
} // end my_test_extent3d

/***********************************************************************
PURPOSE ---

USAGE NOTES ---
***********************************************************************/
PT_EXPORT SmStatus my_test_matrix(void)
{
    {
        SmMatrix sM(2, 2);
        sM.SetAt(0, 0, 1.0);
        sM.SetAt(0, 1, 1.0);
        sM.SetAt(1, 0, 2.0);
        sM.SetAt(1, 1, 2.0);
        SmTArray < double> sR, sS;
        sR.Add(4.0);
        sR.Add(6.0);
        sM.Dump();
        // the following should fail
        if (sM.SolveLinearSystem(sR, sS) != SM_ERR_WARNING)
        {
            SER(SM_ERR);  
        }
        sM.SetAt(0, 0, 1.0);
        sM.SetAt(0, 1, 2.0);
        sM.SetAt(1, 0, 2.0);
        sM.SetAt(1, 1, 2.0);
        sM.Dump();
        SER(sM.SolveLinearSystem(sR, sS));
        SM_DUMP_TARRAY(sS) ; // sS.Dump();
        if (! SM_ARE_SAME(sS.GetAt(0), 2.0))
            SER(SM_ERR);
        if (! SM_ARE_SAME(sS.GetAt(1), 1.0))
            SER(SM_ERR);
    }
    
    {
        SmMatrix sM(3, 3);
        sM.SetAt(0, 0, 1.0);
        sM.SetAt(0, 1, 1.0);
        sM.SetAt(0, 2, 2.0);
        sM.SetAt(1, 0, 2.0);
        sM.SetAt(1, 1, 4.0);
        sM.SetAt(1, 2, - 3.0);
        sM.SetAt(2, 0, 3.0);
        sM.SetAt(2, 1, 6.0);
        sM.SetAt(2, 2, - 5.0);
        SmTArray < double> sR, sS;
        sR.Add(9.0);
        sR.Add(1.0);
        sR.Add(0.0);
        sM.Dump();
        SER(sM.SolveLinearSystem(sR, sS));
        SM_DUMP_TARRAY(sS) ; // sS.Dump();
        if (! SM_ARE_SAME(sS.GetAt(0), 1.0))
            SER(SM_ERR);
        if (! SM_ARE_SAME(sS.GetAt(1), 2.0))
            SER(SM_ERR);
        if (! SM_ARE_SAME(sS.GetAt(2), 3.0))
            SER(SM_ERR);
    }
    
    return SM_SUCCESS;
} // end my_test_matrix


/***********************************************************************
PURPOSE ---

USAGE NOTES ---
***********************************************************************/
PT_EXPORT SmStatus my_test_cci_self(void)
{
  SmContext sContext;
    
    
  if (TRUE)
    {
#ifdef SM_GFX_CODE
      smgfx_SetColor(0, 0, 0);
      smgfx_SetLineWidth(3.0);
#endif // SM_GFX_CODE

      SmTArray < SmPoint3d> sPnts1;
      sPnts1.Add(SmPoint3d(0, 0.1, 0));            
      sPnts1.Add(SmPoint3d(1, 0, 0));    
      sPnts1.Add(SmPoint3d(1.5, 2, 0.0));
      sPnts1.Add(SmPoint3d(1.8, - 1.3, 0.0));    
      sPnts1.Add(SmPoint3d(2.0, 0.0, 0.0));    
      sPnts1.Add(SmPoint3d(2.3, 0.1, 0));
      sPnts1.Add(SmPoint3d(2.8, 0.5, 0));
      SmBSplineCurve *pLine1 = my_create_line(sContext, SmPoint3d(-4, 0.1, 0), SmPoint3d(-3, - 1, 0), 0, 0, 0.0);
      SmBSplineCurve *pLine2 = my_create_line(sContext, SmPoint3d(-3, - 1, 0), SmPoint3d(-3, 0.1, 0), 0, 0, 0.0);
      SmBSplineCurve *pNurb1 = my_create_nurb(sContext, SmPoint3d(-3, 0, 0), sPnts1, 3, 0, 0, 0);
      SmBSplineCurve *pLine3 = my_create_line(sContext, SmPoint3d(-0.2, 0.5, 0.0), SmPoint3d(-2.0, 2.0, 0.0), 0, 0, 0);
      SmBSplineCurve *pLine4 = my_create_line(sContext, SmPoint3d(-2.0, 2.0, 0.0), SmPoint3d(-4.0, 0.1, 0.0), 0, 0, 0);

      SmTArray < SmCurve*> sCurves;
      //    SmObjsDelete<SmBSplineCurve*> sCleanCurves(&sCurves);
        
      sCurves.Add(pLine1);
      sCurves.Add(pLine2);
      sCurves.Add(pNurb1);
      sCurves.Add(pLine3);
      sCurves.Add(pLine4);

      SmCompositeCurve *pComposite = new(sContext) SmCompositeCurve(3, sCurves, TRUE);
      SmObjDelete sClean(pComposite);
        
      SmBSplineCurve *pCir3 = my_create_circle(sContext, 0.3, SmPoint3d(-2.3, 1.0, 0), SM_CO_QUADRATIC, 0, 0, 0);
      SmTArray < SmCurve*> sHoles;
      sHoles.Add(pCir3);
      //    SmObjsDelete<SmCurve*> sCleanHoles(&sHoles);
        
      SmCompositeCurve *pHoleCC = new(sContext) SmCompositeCurve(3, sHoles, TRUE);
      SmObjDelete sClean2(pHoleCC);
        
      SmBSplineCurve *pLine21 = my_create_line(sContext, SmPoint3d(-4.5, - 2, 0), SmPoint3d(1, - 2, 0), 0, 0, 0.0);
      SmBSplineCurve *pLine22 = my_create_line(sContext, SmPoint3d(1, - 2, 0), SmPoint3d(1, 3, 0), 0, 0, 0.0);
      SmBSplineCurve *pLine23 = my_create_line(sContext, SmPoint3d(1, 3, 0.0), SmPoint3d(-4.5, 3.0, 0.0), 0, 0, 0);
      SmBSplineCurve *pLine24 = my_create_line(sContext, SmPoint3d(-4.5, 3.0, 0.0), SmPoint3d(-4.5, - 2, 0.0), 0, 0, 0);
      SmTArray < SmCurve*> sCurves2;
      //    SmObjsDelete<SmCurve*> sCleanCurves2(&sCurves2);
      sCurves2.Add(pLine21);
      sCurves2.Add(pLine22);
      sCurves2.Add(pLine23);
      sCurves2.Add(pLine24);
        
      SmCompositeCurve *pOuter = new(sContext) SmCompositeCurve(3, sCurves2, TRUE);
      SmObjDelete sClean3(pOuter);
        
      double dTol = 0.001;
        
      SmTArray < ULONG> sOrigSides;
      sOrigSides.Add(1);
      sOrigSides.Add(2);
      sOrigSides.Add(1);
        
        
      SmTArray < SmCompositeCurve*> sOrigComposites;
      sOrigComposites.Add(pOuter);
      sOrigComposites.Add(pComposite);
      sOrigComposites.Add(pHoleCC);
        
      SmVector3d sVec(0, 0, 1);
        
      for (ULONG i = 0; i < 20; i++)
        {
          SmTArray < SmBSplineCurve*> sTrimmedOffsets;

// ULONG GWC_GET_BETTER_VALUES_FOR_TOLERANCE_IN_FOLLOWING_CALL GWC_LINE ;

          SER(SmCompositeCurve::CreateOffsetsOfManyCurves
                (sContext,             // in : context for new object construction
                 sOrigComposites,      // in : Array of composite curves to offset
                 sOrigSides,           // in : Corresponding direction of each composite
                                       //      curve to offset.  1-LEFT, 2-RIGHT, 3-BOTH
                 0.001,                // in : Minimum distance at which offset curve end-gaps are filled with corner curves
                 0.0025,               // in : Tolerance to which BSpline Approximations to exact offset curves are built
                 sVec,                 // in : Defines, along with the curve's parameter direction,
                                       //      the right and left hand offset directions.
                 SM_OC_FILLET_CORNER,  // in : SM_OC_LINEAR_EXTENSION: corner = 2 lines from given ends to common linear extension xsect point.
                                       //      SM_OC_FILLET_CORNER: corner = fillet arc centered on crVertexPoint running to given end points
                                       //      SM_OC_LINEAR_CHAMFER: corner = line between endpoints (result is actually within offset distance so bTrimResults must = false).
                 TRUE,                 // in : TRUE = concave raw offsets are intersected and trimmed back to common intersection points
                                       //      FALSE= skip trim step
                 0.1 + 0.1*i,          // in : offset distance (a negative value negates the offset direction)
                 sTrimmedOffsets)) ;   // out: Resulting offset curves will have attribute attached
                                       //      describing origination of curve
                                       // in : TRUE = if BSplineCurve just copy it (preserves CrvParams)
                                       //      FALSE= approximate BSplineCurves (changes CrvParams), NonBSplineCrvs always Approximated
                                       //      default:[FALSE]
                                       // out: optional MaxGap for each output approximation
#ifdef SM_GFX_CODE
          if (smGet_DoGraphics()) {
              ULONG jj;
              smgfx_Erase();
              smgfx_SetLook(1, 2, 0, 0, 1); for (jj = 0;jj < sOrigComposites.GetSize();jj++)
              {
                  sOrigComposites[jj]->Draw(); sm_GraphicsLoop();
              }
              smgfx_SetLook(1, 2, 0, 1, 0); for (jj = 0;jj < sTrimmedOffsets.GetSize();jj++)
              {
                  sTrimmedOffsets[jj]->Draw(); sm_GraphicsLoop();
              }
              sm_GraphicsLoop();
          }
#endif // SM_GFX_CODE
            
          SmTArray < SmCompositeCurve*> sNewComposites;
          SmObjsDelete < SmCompositeCurve*> sCleanNC(&sNewComposites);
          SER(SmCompositeCurve::BuildCompositesFromCurves
                     (sContext, 
                      sTrimmedOffsets, 
                      FALSE,
                      dTol, dTol, dTol, dTol, 2*dTol, dTol, 
                      sNewComposites));
#ifdef SM_GFX_CODE
          if (smGet_DoGraphics()) {
              smgfx_SetLook(3, 4, 1, 0, 0); for (ULONG jj = 0;jj < sNewComposites.GetSize();jj++)
              {
                  sNewComposites[jj]->Draw(); sm_GraphicsLoop();
              }
              sm_GraphicsLoop();
          }
#endif // SM_GFX_CODE

        }

      return SM_SUCCESS;
    }

  return SM_SUCCESS ;

} // end my_test_cci_self




/***********************************************************************
PURPOSE ---

USAGE NOTES ---
***********************************************************************/
PT_EXPORT SmStatus my_test_cci_tangent(void)
{
    //    SmContext sContext(pPool);
    SmContext sContext;
    
    { // For testing tangencies
        ULONG lNumFound;
        
        
        SmBSplineCurve *pLine = my_create_line(sContext, SmPoint3d(1, 2, 0), SmPoint3d(3, 2, 0), 0.5, 0.5, 0.0);
        SmObjDelete sCU1(pLine);
        SmBSplineCurve *pCir3 = my_create_circle(sContext, 0.5, SmPoint3d(1.5, 2.5, 0), SM_CO_QUADRATIC, 0, 1, 1);
        SmObjDelete sCU2(pCir3);
        SmBSplineCurve *pCir4 = my_create_circle(sContext, 0.5, SmPoint3d(2.5, 2.5, 0), SM_CO_QUINTIC, 1, 0, 1);
        SmObjDelete sCU3(pCir4);
        
        // Line circle tangency
        SER(my_test_GlobalCC_intersect(*pLine, *pCir3, lNumFound));
        SM_ASSERT(lNumFound == 1);
        
        // Circle line tangency
        SER(my_test_GlobalCC_intersect(*pCir4, *pLine, lNumFound));
        SM_ASSERT(lNumFound == 1);
        
        SmBSplineCurve *pEll1 = my_create_ellipse(sContext, 1.0, 0.5, SmPoint3d(-2, 2, 0), SM_CO_QUINTIC, 1, 0, 1);
        SmObjDelete sCU4(pEll1);
        SmBSplineCurve *pCir1 = my_create_circle(sContext, 0.5, SmPoint3d(-2, 2, 0), SM_CO_QUINTIC, 1, 1, 0);
        SmObjDelete sCU5(pCir1);
        SmBSplineCurve *pCir2 = my_create_circle(sContext, 1.0, SmPoint3d(-2, 2, 0), SM_CO_QUADRATIC, 0, 1, 1);
        SmObjDelete sCU6(pCir2);
        
        // Tangent circle inside of ellipse to ellipse
#ifdef SM_GFX_CODE
        SmBoolean bDebugMe = FALSE;
        if (bDebugMe)
        {
            pEll1->DrawWDeriv(1);
            pCir1->DrawWDeriv(1);
        }
#endif
        SER(my_test_GlobalCC_intersect(*pCir1, *pEll1, lNumFound));
        SM_ASSERT(lNumFound == 2);
        
        // Tangent circle outside of ellipse
        SER(my_test_GlobalCC_intersect(*pCir2, *pEll1, lNumFound));
        SM_ASSERT(lNumFound == 5);  // GWC: good case - one exact xsect is on a pair of seams (4 solutiosn)
                                    //      and one solution in the MIdCurves
        
        SmTArray < SmPoint3d> sPnts1;
        sPnts1.Add(SmPoint3d(0, 0.1, 0));
        sPnts1.Add(SmPoint3d(0.2, 0.1, 0));
        sPnts1.Add(SmPoint3d(0.4, 0.2, 0.0));
        sPnts1.Add(SmPoint3d(0.6, - 0.3, 0.0));
        sPnts1.Add(SmPoint3d(1.0, 0.0, 0.0));
        sPnts1.Add(SmPoint3d(1.1, 0.1, 0));
        sPnts1.Add(SmPoint3d(1.0, 0.5, 0));
        SmBSplineCurve *pNurb1 = my_create_nurb(sContext, SmPoint3d(-0.5, 2, 0), sPnts1, 3, 1, 0.5, 0);
        SmObjDelete sCU7(pNurb1);
        SmTArray < SmPoint3d> sPnts2;
        sPnts2.Add(SmPoint3d(0, 0.1, 0));
        sPnts2.Add(SmPoint3d(0.2, 0.1, 0));
        sPnts2.Add(SmPoint3d(0.4, 0.3, 0));
        sPnts2.Add(SmPoint3d(0.6, - 0.4, 0));
        sPnts2.Add(SmPoint3d(1.0, 0.0, 0));
        sPnts2.Add(SmPoint3d(1.1, 0.1, 0));
        sPnts2.Add(SmPoint3d(1.0, 0.5, 0));
        SmBSplineCurve *pNurb2 = my_create_nurb(sContext, SmPoint3d(-0.5, 2, 0), sPnts2, 3, 1, 0.5, 0);
        SmObjDelete sCU8(pNurb2);
        
        // Nurbs tangent at ends with 2 intermediate intersections
        //
        // Note: These two curves appear to be coincident over a substantial
        // interval near their end; you can't tell from the graphics.
        // I analyzed them and they are not, they approach monotonically.
        // They get within SM_ZONE_TOL_3D (the tol used in the test) at about parameters
        // 0.9531.  So the proper behavior is to return only the exact point
        // intersection at the end (param 1.0).  [7/20/2011]

        SER(my_test_GlobalCC_intersect(*pNurb1, *pNurb2, lNumFound));
        SM_ASSERT(lNumFound == 4);  // GWC: 4 is the correct solution. Currently getting a closely
                                    //      placed solution pair at one end rather than a single
                                    //      solution. [3/1/2012]
        // BD Right, it's an extraneous solution at t=0.999393926 (on both curves),
        // with a function value 2.8e-11, where all other function values are ~ 1e-16.
        // (Found because of the way the caching works.)
        // There's a similar one near the start of the curves that's closer to the
        // beginning, 0.00001055.  That one is flagged as identical in SmGlobalSolver::
        // IdenticalSolutions(), which uses domain length / 10000, or 0.0001 as the
        // criterion for parameters being the same.  That value happens to call
        // 0.00001055 and 0.0 the same, but not 0.099939 and 1.0.  [10/6/2016]
    }
    
    
    //    pPool->Free();
    //    delete pPool; pPool = NULL ;
    
    return SM_SUCCESS;
} // end my_test_cci_tangent


/***********************************************************************
PURPOSE ---

USAGE NOTES ---  lCount == -1 run all cases else 
***********************************************************************/
PT_EXPORT SmStatus my_test_cci_analy()
{
    SmContext sContext;
    int lCount = -1;    // run all tests

    double dTol = SM_ZONE_TOL_3D ;
    if (lCount != -1)
    {
        lCount++; 
    }
    
    if (lCount ==1 || lCount == -1)
    {
        SmBSplineCurve *pLine = my_create_line(sContext, SmPoint3d(0, 0, 0), SmPoint3d(2.5, 0, 0), 0.5, 0.5, 0.0);
        SmObjDelete sCleanUp(pLine);
        SmBSplineCurve *pCir3 = my_create_circle(sContext, 0.5, SmPoint3d(0, 0, 0), SM_CO_QUADRATIC, 0, 1, 1);
        SmObjDelete sCleanUp1(pCir3);
        
        SmExtent1d sTrmIvl(0.03, 1.0);
        SER(pCir3->Trim(sTrmIvl));
        
        SmSolutionArray sSolutions;
        SmExtent1d sIvl = pLine->GetNaturalInterval();
        SmPoint3d sTestPoint(-0.5, 0.15, 0.0);
#ifdef SM_DEBUG_CODE
        clock_t start, finish;
        start = clock();
#endif
        for (ULONG i = 0; i < 100000; i++)
        {
            SER(pCir3->GlobalPointSolve(sIvl,
                                        SM_SO_MINIMIZE, sTestPoint, SM_ZONE_TOL_3D, NULL, NULL,
                                        SM_SR_ALL, sSolutions));
        }
#ifdef SM_DEBUG_CODE
        finish = clock();
        sm_PrintTime(_T("100000 GlobalPointSolve - minimize calls "), start, finish);
#endif
#ifdef SM_GFX_CODE
        if (smGet_DoGraphics()) {
            smgfx_SetLook(1, 2, 0, 0, 1); pCir3->Draw(&sIvl);
            smgfx_SetLook(3, 4, 1, 0, 1); sTestPoint.Draw();
            smgfx_SetLook(5, 6, 1, 0, 0); sSolutions.Draw();
            sm_GraphicsLoop();
        }
#endif
        
        if(lCount != -1) return SM_SUCCESS;
    }
    
    
    if (lCount ==2  || lCount == -1)
    {
        // Test Circle/Circle intersection out of plane - 30 degrees
        
        ULONG lNumFound;
        double x  = -2.0*SM_ZONE_TOL_3D; 
        double dx = SM_ZONE_TOL_3D/10.0;
        for (ULONG i = 0; i < 40; i++)
          {
            // gap = x = -.00002 + i * .000001 ;
            SmBSplineCurve *pCir2 = my_create_circle(sContext, 0.5, SmPoint3d(-0.5 + x, 0.0, 0), SM_CO_QUADRATIC, 0, 1, 1);
            SmObjDelete sCU1(pCir2);
            SmBSplineCurve *pCir3 = my_create_circle(sContext, 1.0, SmPoint3d(0, 0, 0), SM_CO_QUADRATIC, 0, 1, 1);
            SmObjDelete sCU2(pCir3);
            SmAxis2Placement sRot;
            sRot.RotateAboutAxis(30, SmVector3d(1, 0, 0));
            pCir2->Transform(sRot);
            
            // Near end of closed curve
            // IntersectTol = 1.0e5
            //   should get intersections when gap <= 2 * IntersectTol
            SER(my_test_GlobalCC_intersect(*pCir2, *pCir3, lNumFound)); 
                                                              
            if (x < -(dTol + SM_EFF_ZERO*(1.0+dTol) ) )
              {
                SM_ASSERT(lNumFound == 0); 
              }
            else if (x <= dTol + SM_EFF_ZERO*(1.0+dTol))
              {
                SM_ASSERT(lNumFound == 1); 
              }
            else {SM_ASSERT(lNumFound == 0);}
            x = x + dx;
          }
        if(lCount != -1) return SM_SUCCESS;
    }    
    if (lCount ==3 || lCount == -1)
    {
        // Test Circle/Circle intersection - near a tangency with 2 duplicate points 
        
        ULONG lNumFound;
        double x = -2.0*SM_ZONE_TOL_3D;;
        double dx = SM_ZONE_TOL_3D/10.0;
        for (ULONG i = 0; i < 40; i++)
        {
            SmBSplineCurve *pCir2 = my_create_circle(sContext, 0.5, SmPoint3d(0.5 + x, 0.0, 0), SM_CO_QUADRATIC, 0, 1, 1);
            SmObjDelete sCU1(pCir2);
            SmBSplineCurve *pCir3 = my_create_circle(sContext, 1.0, SmPoint3d(0, 0, 0), SM_CO_QUADRATIC, 0, 1, 1);
            SmObjDelete sCU2(pCir3);
            
            // Near end of closed curve
            SER(my_test_GlobalCC_intersect(*pCir2, *pCir3, lNumFound));
            if (x < -0.999*SM_ZONE_TOL_3D)    // gwc: these are working tolerances but not good ones
            {                     //      they have to map back to the definition of two point intersections
                SM_ASSERT(lNumFound == 0); 
            }
            else if (x <= 1.0e-9)
            {
                SM_ASSERT(lNumFound == 4); // gwc: 4 solutions should be a bug. They are giving the exact two
            }                              //      solutions and the two ends.  The current rule is that
            else {SM_ASSERT(lNumFound == 2);} //   for closed curves near-miss intersections are not given when
            x = x + dx;                       //   exact intersections are available.
        }
        if(lCount != -1) return SM_SUCCESS;
    }    
    
    
    if (lCount == 4 || lCount == -1)
    {
        // Test Circle/Circle intersection - near a tangency with zero duplicate points 
        
        ULONG lNumFound;
        double x = -2.0*SM_ZONE_TOL_3D;;
        double dx = SM_ZONE_TOL_3D/10.0;
        for (ULONG i = 0; i < 40; i++)
        {
            SmBSplineCurve *pCir2 = my_create_circle(sContext, 0.5, SmPoint3d(0.0, 0.5 + x, 0), SM_CO_QUADRATIC, 0, 1, 1);
            SmObjDelete sCU1(pCir2);
            SmBSplineCurve *pCir3 = my_create_circle(sContext, 1.0, SmPoint3d(0, 0, 0), SM_CO_QUADRATIC, 0, 1, 1);
            SmObjDelete sCU2(pCir3);
            
            // Near end of closed curve
            SER(my_test_GlobalCC_intersect(*pCir2, *pCir3, lNumFound));
            if (x < -0.999*SM_ZONE_TOL_3D)         // gwc: these are working tolerances but not good ones
                                       //      they have to map back to the definition of two point intersections
            {
                SM_ASSERT(lNumFound == 0); 
            }
            else if (x <= 1.0e-9)
            {
                SM_ASSERT(lNumFound == 1); 
            }
            else {SM_ASSERT(lNumFound == 2);}
            x = x + dx;
        }
        if(lCount != -1) return SM_SUCCESS;
    }    
    
    if (lCount == 5 || lCount == -1)
    {
        // Test Circle/Circle intersection - near a tangency with zero duplicate points - do
        // a shifting of the curve along the tangency.
        ULONG lNumFound;
        
        double x = -2.0*SM_ZONE_TOL_3D;;
        double dx = SM_ZONE_TOL_3D/10.0;
        for (ULONG i = 0; i < 40; i++)
        {
            SmBSplineCurve *pCir2 = my_create_circle(sContext, 0.5, SmPoint3d(-x, 0.5, 0), SM_CO_QUADRATIC, 0, 1, 1);
            SmObjDelete sCU1(pCir2);
            SmBSplineCurve *pCir3 = my_create_circle(sContext, 1.0, SmPoint3d(0, 0, 0), SM_CO_QUADRATIC, 0, 1, 1);
            SmObjDelete sCU2(pCir3);
            
            // Near end of closed curve
            SER(my_test_GlobalCC_intersect(*pCir2, *pCir3, lNumFound));
            if (smos_Fabs(x) < 1.1*SM_ZONE_TOL_3D/10.0)
            {
                SM_ASSERT(lNumFound == 1); 
            }
            else {SM_ASSERT(lNumFound == 2);}
            x = x + dx;
        }
        if(lCount != -1) return SM_SUCCESS;
    }    
    
    if (lCount == 6 || lCount == -1)
    {
        // Test Circle/Circle intersection - near a tangency with one duplicate point
        ULONG lNumFound;
        
        double x = -2.0*SM_ZONE_TOL_3D;;
        double dx = SM_ZONE_TOL_3D/10.0;
        for (ULONG i = 0; i < 40; i++)
        {
            SmBSplineCurve *pCir2 = my_create_circle(sContext, 0.5, SmPoint3d(-1.5 + x, 0, 0), SM_CO_QUADRATIC, 0, 1, 1);
            SmObjDelete sCU1(pCir2);
            SmBSplineCurve *pCir3 = my_create_circle(sContext, 1.0, SmPoint3d(0, 0, 0), SM_CO_QUADRATIC, 0, 1, 1);
            SmObjDelete sCU2(pCir3);
            
            // Near end of closed curve
            SER(my_test_GlobalCC_intersect(*pCir2, *pCir3, lNumFound));
            if (x < -SM_ZONE_TOL_3D)
            {
                SM_ASSERT(lNumFound == 0); 
            }
            else if (x < 0.0)
            {
                SM_ASSERT(lNumFound == 2); 
            }
            else {SM_ASSERT(lNumFound == 2);}
            x = x + dx;
        }
        if(lCount != -1) return SM_SUCCESS;
    }    
    
    if (lCount == 7 || lCount == -1)
    {
        // Test Line Circle intersection where line crossing plane at 15 degrees near curve
        ULONG lNumFound;
        
        double x = -2.0*SM_ZONE_TOL_3D;;
        double dx = SM_ZONE_TOL_3D/10.0;
        for (ULONG i = 0; i < 40; i++)
        {
            SmBSplineCurve *pLine = my_create_line(sContext, SmPoint3d(0.0 + x, 0.0, 0.05 + x), SmPoint3d(-2.0 + x, 0.0, - 0.05 + x), 0.5, 0.5, 0.0);
            SmObjDelete sCU1(pLine);
            SmBSplineCurve *pCir3 = my_create_circle(sContext, 1.0, SmPoint3d(0, 0, 0), SM_CO_QUADRATIC, 0, 1, 1);
            SmObjDelete sCU2(pCir3);
            
            // Near end of closed curve
            SER(my_test_GlobalCC_intersect(*pLine, *pCir3, lNumFound));
            if (x < -1.0999*SM_ZONE_TOL_3D)
            {
                SM_ASSERT(lNumFound == 0); 
            }
            else if (x > 1.09999*SM_ZONE_TOL_3D)
            {
                SM_ASSERT(lNumFound == 0); 
            }
            else {SM_ASSERT(lNumFound == 1);}
            x = x + dx;
        }
        if(lCount != -1) return SM_SUCCESS;
    }    
    if (lCount == 8 || lCount == -1)
    {
        // Test Line Circle intersection where line crossing plane at 15 degrees near curve
        ULONG lNumFound;
        
        double x = -2.0*SM_ZONE_TOL_3D;;
        double dx = SM_ZONE_TOL_3D/10.0;
        for (ULONG i = 0; i < 40; i++)
        {
            SmBSplineCurve *pLine = my_create_line(sContext, SmPoint3d(0.0 + x, 0.0, 0.1 + x), SmPoint3d(-2.0 + x, 0.0, - 0.1 + x), 0.5, 0.5, 0.0);
            SmObjDelete sCU1(pLine);
            SmBSplineCurve *pCir3 = my_create_circle(sContext, 1.0, SmPoint3d(0, 0, 0), SM_CO_QUADRATIC, 0, 1, 1);
            SmObjDelete sCU2(pCir3);
            
            // Near end of closed curve
            SER(my_test_GlobalCC_intersect(*pLine, *pCir3, lNumFound));
            if (x < -1.1999*SM_ZONE_TOL_3D)
            {
                SM_ASSERT(lNumFound == 0); 
            }
            else if (x > 1.19999*SM_ZONE_TOL_3D)
            {
                SM_ASSERT(lNumFound == 0); 
            }
            else {SM_ASSERT(lNumFound == 1);}
            x = x + dx;
        }
        if(lCount != -1) return SM_SUCCESS;
    }    
    
    if (lCount == 9 || lCount == -1)
    {
        // Test Line Circle intersection where line crossing plane at 45 degrees near curve
        ULONG lNumFound;
        
        double x = -2.0*SM_ZONE_TOL_3D;;
        double dx = SM_ZONE_TOL_3D/10.0;
        for (ULONG i = 0; i < 40; i++)
        {
            SmBSplineCurve *pLine = my_create_line(sContext, SmPoint3d(0.0 + x, 0.0, 1.0), SmPoint3d(-2.0 + x, 0.0, - 1.0), 0.5, 0.5, 0.0);
            SmObjDelete sCU1(pLine);
            SmBSplineCurve *pCir3 = my_create_circle(sContext, 1.0, SmPoint3d(0, 0, 0), SM_CO_QUADRATIC, 0, 1, 1);
            SmObjDelete sCU2(pCir3);
            
            // Near end of closed curve
            SER(my_test_GlobalCC_intersect(*pLine, *pCir3, lNumFound));
            if (x < -1.4001*SM_ZONE_TOL_3D)
            {
                SM_ASSERT(lNumFound == 0); 
            }
            else if (x > 1.4001*SM_ZONE_TOL_3D)
            {
                SM_ASSERT(lNumFound == 0); 
            }
            else {SM_ASSERT(lNumFound == 1);}
            x = x + dx;
        }
        if(lCount != -1) return SM_SUCCESS;
    }    
    
    if (lCount == 10 || lCount == -1)
    {
        // Test Line Circle intersection where line is just near touching circle at one end - out of plane
        ULONG lNumFound;
        
        double x = -1.5*SM_ZONE_TOL_3D;;
        double dx = SM_ZONE_TOL_3D/10.0;
        for (ULONG i = 0; i < 30; i++)
        {
            SmBSplineCurve *pLine = my_create_line(sContext, SmPoint3d(-1.0 + x, 0.0, 0.0), SmPoint3d(-2.0, 0.0, - 2.0), 0.5, 0.5, 0.0);
            SmObjDelete sCU1(pLine);
            SmBSplineCurve *pCir3 = my_create_circle(sContext, 1.0, SmPoint3d(0, 0, 0), SM_CO_QUADRATIC, 0, 1, 1);
            SmObjDelete sCU2(pCir3);
            
            // Near end of closed curve
            SER(my_test_GlobalCC_intersect(*pLine, *pCir3, lNumFound));
            if (x < -1.001*SM_ZONE_TOL_3D)
            {
                SM_ASSERT(lNumFound == 0); 
            }
            else if (x > 1.1999*SM_ZONE_TOL_3D)
            {
                SM_ASSERT(lNumFound == 0); 
            }
            else {SM_ASSERT(lNumFound == 1);}
            x = x + dx;
        }
        if(lCount != -1) return SM_SUCCESS;
    }    
    
    if (lCount == 11 || lCount == -1)
    {
        // Test Line Circle intersection where line is just near touching circle at one end - in plane
        ULONG lNumFound;
        
        double x = -1.5*SM_ZONE_TOL_3D;;
        double dx = SM_ZONE_TOL_3D/10.0;
        for (ULONG i = 0; i < 30; i++)
        {
            SmBSplineCurve *pLine = my_create_line(sContext, SmPoint3d(-1.0 + x, 0.0, 0.0), SmPoint3d(-2.0, 0.0, 0.0), 0.5, 0.5, 0.0);
            SmObjDelete sCU1(pLine);
            SmBSplineCurve *pCir3 = my_create_circle(sContext, 1.0, SmPoint3d(0, 0, 0), SM_CO_QUADRATIC, 0, 1, 1);
            SmObjDelete sCU2(pCir3);
            
            // Near end of closed curve
            SER(my_test_GlobalCC_intersect(*pLine, *pCir3, lNumFound));
            if (x < -1.001*SM_ZONE_TOL_3D)
            {
                SM_ASSERT(lNumFound == 0); 
            }
            else {SM_ASSERT(lNumFound == 1);}
            x = x + dx;
        }
        if(lCount != -1) return SM_SUCCESS;
    }    
    if (lCount == 12 || lCount == -1)
    {
        // Test Line Circle intersection where line is perpendicular to the circle
        ULONG lNumFound;
        
        double x = -1.5*SM_ZONE_TOL_3D;;
        double dx = SM_ZONE_TOL_3D/10.0;
        for (ULONG i = 0; i < 30; i++)
        {
            SmBSplineCurve *pLine = my_create_line(sContext, SmPoint3d(-1.0 + x, 0.0, - 1.0), SmPoint3d(-1.0 + x, 0.0, 1.0), 0.5, 0.5, 0.0);
            SmObjDelete sCU1(pLine);
            SmBSplineCurve *pCir3 = my_create_circle(sContext, 1.0, SmPoint3d(0, 0, 0), SM_CO_QUADRATIC, 0, 1, 1);
            SmObjDelete sCU2(pCir3);
            
            // Near end of closed curve
            double dTolerance = SM_ZONE_TOL_3D ;
            SER(my_test_GlobalCC_intersect(*pLine, *pCir3, lNumFound));
            if (x < -(dTolerance +SM_EFF_ZERO))
            {
                SM_ASSERT(lNumFound == 0); 
            }
            else if (x > ( dTolerance +SM_EFF_ZERO))
            {
                SM_ASSERT(lNumFound == 0); 
            }
            else {SM_ASSERT(lNumFound == 1);}
            x = x + dx;
        }
        if(lCount != -1) return SM_SUCCESS;
    }   
    
    if (lCount == 13 || lCount == -1)
    {
        // Test Line Circle intersection where line is tangent to circle
        ULONG lNumFound;
        
        double x = -SM_ZONE_TOL_3D;;
        double dx = SM_ZONE_TOL_3D/10.0;
        for (ULONG i = 0; i < 20; i++)
        {
            SmBSplineCurve *pLine = my_create_line(sContext, SmPoint3d(-1.0 + x, - 1.0, 0), SmPoint3d(-1.0 + x, 1.0, 0), 0.5, 0.5, 0.0);
            SmObjDelete sCU1(pLine);
            SmBSplineCurve *pCir3 = my_create_circle(sContext, 1.0, SmPoint3d(0, 0, 0), SM_CO_QUADRATIC, 0, 1, 1);
            SmObjDelete sCU2(pCir3);
            
            // Near end of closed curve
            SER(my_test_GlobalCC_intersect(*pLine, *pCir3, lNumFound));
            if (x <= SM_EFF_ZERO)
            {
                SM_ASSERT(lNumFound == 1); 
            }
            else {SM_ASSERT(lNumFound == 2);}
            x = x + dx;
        }
        if(lCount != -1) return SM_SUCCESS;
    }    
    
    if (lCount == 14 || lCount == -1)
    {
        // For testing near knots and near end of closed curves
        ULONG lNumFound;
        
        double y = 0;
        double dy = SM_ZONE_TOL_3D/10.0;
        for (ULONG i = 0; i < 20; i++)
        {
            SmBSplineCurve *pLine = my_create_line(sContext, SmPoint3d(0, y, 0), SmPoint3d(2.5, y, 0), 0.5, 0.5, 0.0);
            SmObjDelete sCU1(pLine);
            SmBSplineCurve *pCir3 = my_create_circle(sContext, 1.0, SmPoint3d(0, 0, 0), SM_CO_QUADRATIC, 0, 1, 1);
            SmObjDelete sCU2(pCir3);
            SmBSplineCurve *pCir4 = my_create_circle(sContext, 1.0, SmPoint3d(2.5, 0, 0), SM_CO_QUADRATIC, 1, 0, 1);
            SmObjDelete sCU3(pCir4);
            
            // Near end of closed curve
            SER(my_test_GlobalCC_intersect(*pLine, *pCir3, lNumFound));
            if ( y <= SM_ZONE_TOL_3D * 0.9999999 )
            {
                if (lNumFound != 3 && lNumFound != 2)
                    SE(SM_ERR);
            }
            else if ( y > SM_ZONE_TOL_3D * 1.0000001 )
            {
                if (lNumFound != 1)
                    SE(SM_ERR);
            }
            // else: if y is 'exactly' SM_ZONE_TOL3D, the result can depend on noise.  [161003]

            y += dy;
            
            // Near knot
            SER(my_test_GlobalCC_intersect(*pCir4, *pLine, lNumFound));
            SM_ASSERT(lNumFound == 1);
        }    
        
        if(lCount != -1) return SM_SUCCESS;
    } 
    lCount = 0;
    
    
    //    pPool->Free();
    //    delete pPool; pPool = NULL ;
    
    return SM_SUCCESS;
} // end my_test_cci_analy


/***********************************************************************
PURPOSE ---

USAGE NOTES ---
***********************************************************************/
PT_EXPORT SmStatus my_test_cci_coincidence(void)
{
    //    SmContext sContext(pPool);
    SmContext sContext;
    
    { // For testing coincidence
        ULONG lNumFound;
        
        SmTArray < SmPoint3d> sPnts1;
        sPnts1.Add(SmPoint3d(0, 0.1, 0));
        sPnts1.Add(SmPoint3d(0.2, 0.1, 0));
        sPnts1.Add(SmPoint3d(0.4, 0.2, 0.0));
        sPnts1.Add(SmPoint3d(0.6, - 0.3, 0.0));
        sPnts1.Add(SmPoint3d(1.0, 0.0, 0.0));
        sPnts1.Add(SmPoint3d(1.1, 0.2, 0));
        sPnts1.Add(SmPoint3d(1.0, 0.5, 0));
        SmBSplineCurve *pNurb1 = my_create_nurb(sContext, SmPoint3d(0, 0, 0), sPnts1, 3, 1, 0.5, 0);
        SmObjDelete sCU1(pNurb1);
        SmTArray < SmPoint3d> sPnts2;
        sPnts2.Add(SmPoint3d(0, 0.1, 0.0));
        sPnts2.Add(SmPoint3d(0.2, 0.1, 0.0));
        sPnts2.Add(SmPoint3d(0.4, 0.2, 0.0));
        sPnts2.Add(SmPoint3d(0.6, - 0.3, 0.0));
        sPnts2.Add(SmPoint3d(1.0, 0.2, 0.0));
        sPnts2.Add(SmPoint3d(1.1, - 0.1, 0.0));
        sPnts2.Add(SmPoint3d(1.0, 0.3, 0.0));
        SmBSplineCurve *pNurb2 = my_create_nurb(sContext, SmPoint3d(0, 0, 0), sPnts2, 3, 1, 0.5, 0);
        SmObjDelete sCU2(pNurb2);
        
        // Test for nurb coincidence and intersection
        SER(my_test_GlobalCC_intersect(*pNurb1, *pNurb2, lNumFound));
        SM_ASSERT(lNumFound == 3);
        
        SmBSplineCurve *pLine = my_create_line(sContext, SmPoint3d(-2, 0, 0), SmPoint3d(-1, 0, 0), 0.5, 0.5, 0.0);
        SmObjDelete sCU3(pLine);
        SmBSplineCurve *pLine2 = my_create_line(sContext, SmPoint3d(-1.5, 0, 0), SmPoint3d(-0.5, 0, 0), 0.5, 0.5, 0.0);
        SmObjDelete sCU4(pLine2);
        SmBSplineCurve *pCir3 = my_create_circle(sContext, 0.499999, SmPoint3d(1.5, - 1.5, 0), SM_CO_QUADRATIC, 0, 1, 1);
        SmObjDelete sCU5(pCir3);
        SmBSplineCurve *pCir4 = my_create_circle(sContext, 0.500005, SmPoint3d(1.5, - 1.5, 0), SM_CO_QUINTIC, 1, 0, 1);
        SmObjDelete sCU6(pCir4);
        
        // Test line coincidence
        SER(my_test_GlobalCC_intersect(*pLine, *pLine2, lNumFound));
        SM_ASSERT(lNumFound == 1);
        
        // Test for standard circle coincidence
        SER(my_test_GlobalCC_intersect(*pCir4, *pCir3, lNumFound));
        SM_ASSERT(lNumFound == 1);   // gwc: used to be 3 but should really be 1
        
        SmBSplineCurve *pCir1 = my_create_circle(sContext, 0.50000, SmPoint3d(2, 0, 0), SM_CO_QUINTIC, 1, 1, 0);
        SmObjDelete sCU7(pCir1);
        SmBSplineCurve *pCir2 = my_create_circle(sContext, 0.50000, SmPoint3d(2, 0, 0), SM_CO_QUADRATIC, 0, 1, 1);
        SmObjDelete sCU8(pCir2);
        
        // Test for circle coincidence with non matching radii
        SER(my_test_GlobalCC_intersect(*pCir1, *pCir2, lNumFound));
        SM_ASSERT(lNumFound == 1);   // gwc: used to be 3 but should really be 1
        
        SmBSplineCurve *pCir5 = my_create_circle(sContext, 0.50000, SmPoint3d(0, 0, 0), SM_CO_QUINTIC, 1, 1, 0);
        SmObjDelete sCU9(pCir5);
        SmBSplineCurve *pCir6 = my_create_circle(sContext, 0.50000, SmPoint3d(0, 0, 0), SM_CO_QUADRATIC, 0, 1, 1);
        SmObjDelete sCU10(pCir6);
        
        SmAxis2Placement sA2P;
        sA2P.SetCanonical(SmPoint3d(0, 0, 0), SmVector3d(0, 1, 0), SmVector3d(-1, 0, 0));
        
        pCir6->Transform(sA2P);
        // Test for circle coincidence with non coincident start/end points
        SER(my_test_GlobalCC_intersect(*pCir5, *pCir6, lNumFound));
        SM_ASSERT(lNumFound == 2);
        
        sA2P.SetCanonical(SmPoint3d(0, 0, 0), SmVector3d(1, 0, 0), SmVector3d(0, - 1, 0));
        SmBSplineCurve *pCir7 = my_create_circle(sContext, 0.50000, SmPoint3d(0, 0, 0), SM_CO_QUADRATIC, 0, 1, 1);
        SmObjDelete sCU11(pCir7);
        pCir7->Transform(sA2P);
        // Test for circle coincidence with coincident start/end points but opposite directions
        SER(my_test_GlobalCC_intersect(*pCir5, *pCir7, lNumFound));
        SM_ASSERT(lNumFound == 1);   // gwc: used to be 3 but should really be 1
        
        sA2P.SetCanonical(SmPoint3d(0, 0, 0), SmVector3d(0, 1, 0), SmVector3d(1, 0, 0));
        SmBSplineCurve *pCir8 = my_create_circle(sContext, 0.50000, SmPoint3d(0, 0, 0), SM_CO_QUADRATIC, 0, 1, 1);
        SmObjDelete sCU12(pCir8);
        pCir8->Transform(sA2P);
        // Test for circle coincidence with non- coincident start/end points and opposite directions
        SER(my_test_GlobalCC_intersect(*pCir5, *pCir8, lNumFound));
        SM_ASSERT(lNumFound == 2);
    }
    
    
    
    //    pPool->Free();
    //    delete pPool; pPool = NULL ;
    
    return SM_SUCCESS;
} // end my_test_cci_coincidence








/***********************************************************************
PURPOSE ---

USAGE NOTES ---
***********************************************************************/
PT_EXPORT SmStatus my_test_cci_tol(void)
{
    //    SmContext sContext(pPool);
    SmContext sContext;
    
    { // For testing intersections close to tolerance away or overlapping
        ULONG lNumFound;
        
        SmBSplineCurve *pLine = my_create_line(sContext, SmPoint3d(1, - 2, 0), SmPoint3d(3, - 2, 0), 0.5, 0.5, 0.0);
        SmObjDelete sCU1(pLine);
        SmBSplineCurve *pCir3 = my_create_circle(sContext, 0.499999, SmPoint3d(1.5, - 1.5, 0), SM_CO_QUINTIC, 0, 1, 1);
        SmObjDelete sCU2(pCir3);
        SmBSplineCurve *pCir4 = my_create_circle(sContext, 0.500005, SmPoint3d(2.5, - 1.5, 0), SM_CO_QUINTIC, 1, 0, 1);
        SmObjDelete sCU3(pCir4);
        
        // Test for slightly overlapping tangency
        SER(my_test_GlobalCC_intersect(*pCir4, *pLine, lNumFound));
        SM_ASSERT(lNumFound == 2); // used to be 1, gwc: modified near-tangent intersections to count 2-pt solutions when spaced more than tol apart
        
        // Test for near tangency - no real intersections
        SER(my_test_GlobalCC_intersect(*pLine, *pCir3, lNumFound));
        SM_ASSERT(lNumFound == 1);
        
        
        // The following two tests cover near tangency where no real intersections
        // exist and where a slight overlap exists
        SmBSplineCurve *pEll1 = my_create_ellipse(sContext, 1.0, 0.5, SmPoint3d(-2, - 2, 0), SM_CO_QUINTIC, 1, 0, 1);
        SmObjDelete sCU4(pEll1);
        SmBSplineCurve *pCir1 = my_create_circle(sContext, 0.499999, SmPoint3d(-2, - 2, 0), SM_CO_QUINTIC, 1, 1, 0);
        SmObjDelete sCU5(pCir1);
        SmBSplineCurve *pCir2 = my_create_circle(sContext, 0.999999, SmPoint3d(-2, - 2, 0), SM_CO_QUINTIC, 0, 1, 1);
        SmObjDelete sCU6(pCir2);
        
        // No real intersection - circle inscribed in an ellipse
        SER(my_test_GlobalCC_intersect(*pCir1, *pEll1, lNumFound));
        SM_ASSERT(lNumFound == 2);
        
        // Slight overlap
        SER(my_test_GlobalCC_intersect(*pCir2, *pEll1, lNumFound));
        SM_ASSERT(lNumFound == 8);   // gwc: was 7, used to combine 3 intersection points into one coincident region.
                                     //      now we keep those point solutions.  Changed to 9. [090315]
                                     // GWC: broken case  [120301]
                                     //      1 - at a double intersection it's keeping the
                                     //          mid-point where the curves are tangent to one another
                                     //          rather than returning just 2 solutions it returns 3.
                                     //      2 - for solutions near the seam it's returning exact solutions
                                     //          and extra end-point near-miss solutions.  It's not clear
                                     //          due to symmetry if that's a closed extra solution or
                                     //          if that's a mid-point on a two point extra solution
                                     //          like broken case 1 above.
                                     // bd: Now fixed.  (I didn't do it, just improvements.) [161003]
                                     //   It now returns 8.  4 are 'good' solutions (tight, actual
                                     //   intersections), 2 on the right (across the seams) and 2 on
                                     //   the left (just above and below the midpoints).  The other 4
                                     //   are tolerance-sized solutions for the end points, at the
                                     //   seam: 0/0, 0/1, 1/0, 1/1.  Changed to 8.
        SmTArray < SmPoint3d> sPnts1;
        sPnts1.Add(SmPoint3d(0, 0.1, 0));
        sPnts1.Add(SmPoint3d(0.2, 0.1, 0));
        sPnts1.Add(SmPoint3d(0.4, 0.2, 0.0));
        sPnts1.Add(SmPoint3d(0.6, - 0.3, 0.0));
        sPnts1.Add(SmPoint3d(1.0, 0.0, 0.0));
        sPnts1.Add(SmPoint3d(1.1, 0.1, 0));
        sPnts1.Add(SmPoint3d(1.0, 0.5, 0));
        SmBSplineCurve *pNurb1 = my_create_nurb(sContext, SmPoint3d(-0.5, - 2, 0), sPnts1, 3, 1, 0.5, 0);
        SmObjDelete sCU7(pNurb1);
        SmTArray < SmPoint3d> sPnts2;
        sPnts2.Add(SmPoint3d(0, 0.0, 0.000005));
        sPnts2.Add(SmPoint3d(0.2, 0.2, 0.000005));
        sPnts2.Add(SmPoint3d(0.4, 0.3, 0.000005));
        sPnts2.Add(SmPoint3d(0.6, - 0.4, 0.000005));
        sPnts2.Add(SmPoint3d(1.0, 0.0, 0.000005));
        sPnts2.Add(SmPoint3d(1.1, 0.1, 0.000005));
        sPnts2.Add(SmPoint3d(1.0, 0.3, 0.000005));
        SmBSplineCurve *pNurb2 = my_create_nurb(sContext, SmPoint3d(-0.5, - 2, 0), sPnts2, 3, 1, 0.5, 0);
        SmObjDelete sCU8(pNurb2);
        
        SER(my_test_GlobalCC_intersect(*pNurb1, *pNurb2, lNumFound));  // GWC: this case is good.
        SM_ASSERT(lNumFound == 3);
    }
    
    
    
    
    
    //    pPool->Free();
    //    delete pPool; pPool = NULL ;
    
    return SM_SUCCESS;
} // end my_test_cci_tol








/***********************************************************************
PURPOSE ---

USAGE NOTES ---
***********************************************************************/
PT_EXPORT SmStatus my_test_cci_nurb( void )
{
  SmContext sContext;

  {
    SmTArray < SmPoint3d> sPnts1;
    sPnts1.Add( SmPoint3d( -1, 0.5, 0 ) );
    sPnts1.Add( SmPoint3d( 0, 0.5, 0 ) );
    sPnts1.Add( SmPoint3d( 1, 1, 0 ) );
    sPnts1.Add( SmPoint3d( 0, 1.5, 0 ) );
    sPnts1.Add( SmPoint3d( -1, 2, 0 ) );
    sPnts1.Add( SmPoint3d( 1, 4.5, 0 ) );
    sPnts1.Add( SmPoint3d( 0, 3.5, 0 ) );
    sPnts1.Add( SmPoint3d( -1, 4, 0 ) );
    sPnts1.Add( SmPoint3d( 1, 5, 0 ) );
    sPnts1.Add( SmPoint3d( -1, 6, 0 ) );
    sPnts1.Add( SmPoint3d( 0, 6.5, 0 ) );
    sPnts1.Add( SmPoint3d( 1, 7, 0 ) );
    sPnts1.Add( SmPoint3d( 0, 7.5, 0 ) );
    sPnts1.Add( SmPoint3d( -1, 8, 0 ) );
    sPnts1.Add( SmPoint3d( 0, 8.5, 0 ) );
    sPnts1.Add( SmPoint3d( 1, 9, 0 ) );
    sPnts1.Add( SmPoint3d( -1, 9.5, 0 ) );
    SmBSplineCurve *pNurb1 = my_create_nurb( sContext, SmPoint3d( 10, -5, 0 ), sPnts1, 3, 0, 0, 0 );
    SmObjDelete sCU1( pNurb1 );

    SmTArray < SmPoint3d> sPnts2;
    sPnts2.Add( SmPoint3d( 1, 0.5, 0 ) );
    sPnts2.Add( SmPoint3d( 0, 0.5, 0 ) );
    sPnts2.Add( SmPoint3d( -1, 1, 0 ) );
    sPnts2.Add( SmPoint3d( 0, 1.5, 0 ) );
    sPnts2.Add( SmPoint3d( 1, 2, 0 ) );
    sPnts2.Add( SmPoint3d( -1, 4.5, 0 ) );
    sPnts2.Add( SmPoint3d( 0, 3.5, 0 ) );
    sPnts2.Add( SmPoint3d( 1, 4, 0 ) );
    sPnts2.Add( SmPoint3d( -1, 5, 0 ) );
    sPnts2.Add( SmPoint3d( 1, 6, 0 ) );
    sPnts2.Add( SmPoint3d( 0, 6.5, 0 ) );
    sPnts2.Add( SmPoint3d( -1, 7, 0 ) );
    sPnts2.Add( SmPoint3d( 0, 7.5, 0 ) );
    sPnts2.Add( SmPoint3d( 1, 8, 0 ) );
    sPnts2.Add( SmPoint3d( 0, 8.5, 0 ) );
    sPnts2.Add( SmPoint3d( -1, 9, 0 ) );
    sPnts2.Add( SmPoint3d( 1, 9.5, 0 ) );
    SmBSplineCurve *pNurb2 = my_create_nurb( sContext, SmPoint3d( 10, -5, 0 ), sPnts2, 3, 1, 0.5, 0 );
    SmObjDelete sCU2( pNurb2 );

    ULONG nCount;
    SER( my_test_GlobalCC_intersect( *pNurb1, *pNurb2, nCount ) );
    SM_ASSERT( nCount == 16 );

    pNurb1 = my_create_nurb( sContext, SmPoint3d( 6, -5, 0 ), sPnts1, 3, 0, 0, 0 );
    SmObjDelete sCU3( pNurb1 );
    pNurb2 = my_create_nurb( sContext, SmPoint3d( 7, -5, 0 ), sPnts2, 3, 1, 0.5, 0 );
    SmObjDelete sCU4( pNurb2 );
    SER( my_test_GlobalCC_intersect( *pNurb1, *pNurb2, nCount ) );
    SM_ASSERT( nCount == 8 );

    pNurb1 = my_create_nurb( sContext, SmPoint3d( 13, -4, 0 ), sPnts1, 3, 0, 0, 0 );
    SmObjDelete sCU5( pNurb1 );
    pNurb2 = my_create_nurb( sContext, SmPoint3d( 13, -5, 0 ), sPnts2, 3, 1, 0.5, 0 );
    SmObjDelete sCU6( pNurb2 );
    SER( my_test_GlobalCC_intersect( *pNurb1, *pNurb2, nCount ) );
    SM_ASSERT( nCount == 8 );  // Coincidence case
                             // GWC: Broken case - missing point intersections and
                             //                    and missing regions of partial coincidence
  }

  {  // Make a real good nurb test for the closest point stuff
    SmTArray < SmPoint3d> sPnts1;
    sPnts1.Add( SmPoint3d( -1, 0.5, 0 ) );
    sPnts1.Add( SmPoint3d( 0, 0.5, 0 ) );
    sPnts1.Add( SmPoint3d( 1, 1, 0 ) );
    sPnts1.Add( SmPoint3d( 0, 1.5, 0 ) );
    sPnts1.Add( SmPoint3d( -1, 2, 0 ) );
    sPnts1.Add( SmPoint3d( 0, 2.5, 0 ) );
    sPnts1.Add( SmPoint3d( 1, 3, 0 ) );
    sPnts1.Add( SmPoint3d( 0, 3.5, 0 ) );
    sPnts1.Add( SmPoint3d( -1, 4, 0 ) );
    sPnts1.Add( SmPoint3d( 0, 4.5, 0 ) );
    sPnts1.Add( SmPoint3d( 1, 5, 0 ) );
    sPnts1.Add( SmPoint3d( 0, 5.5, 0 ) );
    sPnts1.Add( SmPoint3d( -1, 6, 0 ) );
    sPnts1.Add( SmPoint3d( 0, 6.5, 0 ) );
    sPnts1.Add( SmPoint3d( 1, 7, 0 ) );
    sPnts1.Add( SmPoint3d( 0, 7.5, 0 ) );
    sPnts1.Add( SmPoint3d( -1, 8, 0 ) );
    sPnts1.Add( SmPoint3d( 0, 8.5, 0 ) );
    sPnts1.Add( SmPoint3d( 1, 9, 0 ) );
    sPnts1.Add( SmPoint3d( 0, 9.5, 0 ) );
    sPnts1.Add( SmPoint3d( -1, 9.5, 0 ) );
    SmBSplineCurve *pNurb1 = my_create_nurb( sContext, SmPoint3d( 0, -5, 0 ), sPnts1, 3, 0, 0, 0 );
    SmObjDelete sCU1( pNurb1 );

    SmTArray < SmPoint3d> sPnts2;
    sPnts2.Add( SmPoint3d( 1, 0.5, 0 ) );
    sPnts2.Add( SmPoint3d( 0, 0.5, 0 ) );
    sPnts2.Add( SmPoint3d( -1, 1, 0 ) );
    sPnts2.Add( SmPoint3d( 0, 1.5, 0 ) );
    sPnts2.Add( SmPoint3d( 1, 2, 0 ) );
    sPnts2.Add( SmPoint3d( 0, 2.5, 0 ) );
    sPnts2.Add( SmPoint3d( -1, 3, 0 ) );
    sPnts2.Add( SmPoint3d( 0, 3.5, 0 ) );
    sPnts2.Add( SmPoint3d( 1, 4, 0 ) );
    sPnts2.Add( SmPoint3d( 0, 4.5, 0 ) );
    sPnts2.Add( SmPoint3d( -1, 5, 0 ) );
    sPnts2.Add( SmPoint3d( 0, 5.5, 0 ) );
    sPnts2.Add( SmPoint3d( 1, 6, 0 ) );
    sPnts2.Add( SmPoint3d( 0, 6.5, 0 ) );
    sPnts2.Add( SmPoint3d( -1, 7, 0 ) );
    sPnts2.Add( SmPoint3d( 0, 7.5, 0 ) );
    sPnts2.Add( SmPoint3d( 1, 8, 0 ) );
    sPnts2.Add( SmPoint3d( 0, 8.5, 0 ) );
    sPnts2.Add( SmPoint3d( -1, 9, 0 ) );
    sPnts2.Add( SmPoint3d( 0, 9.5, 0 ) );
    sPnts2.Add( SmPoint3d( 1, 9.5, 0 ) );
    SmBSplineCurve *pNurb2 = my_create_nurb( sContext, SmPoint3d( 0, -5, 0 ), sPnts2, 3, 1, 0.5, 0 );
    SmObjDelete sCU2( pNurb2 );

    ULONG nCount;
    SER( my_test_GlobalCC_intersect( *pNurb1, *pNurb2, nCount ) );
    SM_ASSERT( nCount == 10 );  // gwc: this case is good

    pNurb1 = my_create_nurb( sContext, SmPoint3d( -4, -5, 0 ), sPnts1, 3, 0, 0, 0 );
    SmObjDelete sCU3( pNurb1 );
    pNurb2 = my_create_nurb( sContext, SmPoint3d( -3, -5, 0 ), sPnts2, 3, 1, 0.5, 0 );
    SmObjDelete sCU4( pNurb2 );
    SER( my_test_GlobalCC_intersect( *pNurb1, *pNurb2, nCount ) );
    SM_ASSERT( nCount == 10 ); // GWC: this case is good

    pNurb1 = my_create_nurb( sContext, SmPoint3d( 3, -4, 0 ), sPnts1, 3, 0, 0, 0 );
    SmObjDelete sCU5( pNurb1 );
    pNurb2 = my_create_nurb( sContext, SmPoint3d( 3, -5, 0 ), sPnts2, 3, 1, 0.5, 0 );
    SmObjDelete sCU6( pNurb2 );
    SER( my_test_GlobalCC_intersect( *pNurb1, *pNurb2, nCount ) );
    SM_ASSERT( nCount == 2 );  // Coincidence case
                             // GWC: This case is broken - it does not
                             //      get the full partial-coincident region.
                             //      We need to define exactly the end of a partial
                             //      coincident region before fixing this problem.
                             // I think the right solution will end up being a single coincident region.
                             //   for now just disable the check.
    // BD: I have analyzed the previous pair of curves quite closely.
    // They have a large coincident region, and then one small region
    // where they approach within tolerance but don't quite touch.
    // At parameter pair:     they:
    // ( 0.1025695, 0.207833) approach within tol (which is SM_ZONE_TOL_3D).
    // ( 0.105262,  0.210525) become 'truly' coincident: dist essentially 0.
    // ( 0.684203,  0.789466 ) diverge from 'true' coincidence.
    // ( 0.687668,  0.792932 ) diverge farther than tol.
    // The near-miss is within tol from about 0.7347 to .7368
    // (on pNurb1), with a min at about .7356.
    // So the result should be two solutions: one range sol,
    // and probably a point sol for the near-miss.  [7/20/2011]

  }


  return SM_SUCCESS;
} // end my_test_cci_nurb






/***********************************************************************
PURPOSE ---

USAGE NOTES ---
***********************************************************************/
PT_EXPORT SmStatus my_test_cc_nurb(void)
{
    SmContext sContext;
    
    {  // Make a real good nurb test for the closest point stuff
        SmTArray < SmPoint3d> sPnts1;
        sPnts1.Add(SmPoint3d(-1, 0.5, 0));            
        sPnts1.Add(SmPoint3d(0, 0.5, 0));
        sPnts1.Add(SmPoint3d(1, 1, 0));
        sPnts1.Add(SmPoint3d(0, 1.5, 0));
        sPnts1.Add(SmPoint3d(-1, 2, 0));
        sPnts1.Add(SmPoint3d(0, 2.5, 0));
        sPnts1.Add(SmPoint3d(1, 3, 0));
        sPnts1.Add(SmPoint3d(0, 3.5, 0));
        sPnts1.Add(SmPoint3d(-1, 4, 0));
        sPnts1.Add(SmPoint3d(0, 4.5, 0));
        sPnts1.Add(SmPoint3d(1, 5, 0));
        sPnts1.Add(SmPoint3d(0, 5.5, 0));
        sPnts1.Add(SmPoint3d(-1, 6, 0));
        sPnts1.Add(SmPoint3d(0, 6.5, 0));
        sPnts1.Add(SmPoint3d(1, 7, 0));
        sPnts1.Add(SmPoint3d(0, 7.5, 0));
        sPnts1.Add(SmPoint3d(-1, 8, 0));
        sPnts1.Add(SmPoint3d(0, 8.5, 0));
        sPnts1.Add(SmPoint3d(1, 9, 0));
        sPnts1.Add(SmPoint3d(0, 9.5, 0));
        sPnts1.Add(SmPoint3d(-1, 9.5, 0));
        SmBSplineCurve *pNurb1 = my_create_nurb(sContext, SmPoint3d(2, - 5, 0), sPnts1, 3, 0, 0, 0);
        SmObjDelete sCU1(pNurb1);
        
        SmTArray < SmPoint3d> sPnts2;
        sPnts2.Add(SmPoint3d(1, 0.5, 0));            
        sPnts2.Add(SmPoint3d(0, 0.5, 0));
        sPnts2.Add(SmPoint3d(-1, 1, 0));
        sPnts2.Add(SmPoint3d(0, 1.5, 0));
        sPnts2.Add(SmPoint3d(1, 2, 0));
        sPnts2.Add(SmPoint3d(0, 2.5, 0));
        sPnts2.Add(SmPoint3d(-1, 3, 0));
        sPnts2.Add(SmPoint3d(0, 3.5, 0));
        sPnts2.Add(SmPoint3d(1, 4, 0));
        sPnts2.Add(SmPoint3d(0, 4.5, 0));
        sPnts2.Add(SmPoint3d(-1, 5, 0));
        sPnts2.Add(SmPoint3d(0, 5.5, 0));
        sPnts2.Add(SmPoint3d(1, 6, 0));
        sPnts2.Add(SmPoint3d(0, 6.5, 0));
        sPnts2.Add(SmPoint3d(-1, 7, 0));
        sPnts2.Add(SmPoint3d(0, 7.5, 0));
        sPnts2.Add(SmPoint3d(1, 8, 0));
        sPnts2.Add(SmPoint3d(0, 8.5, 0));
        sPnts2.Add(SmPoint3d(-1, 9, 0));
        sPnts2.Add(SmPoint3d(0, 9.5, 0));
        sPnts2.Add(SmPoint3d(1, 9.5, 0));
        SmBSplineCurve *pNurb2 = my_create_nurb(sContext, SmPoint3d(-2, - 5, 0), sPnts2, 3, 1, 0.5, 0);
        SmObjDelete sCU2(pNurb2);
        
        ULONG lNumFound;
        SER(my_test_GlobalCC_extrema(*pNurb1, *pNurb2, SM_SO_MINIMIZE, lNumFound));
        SM_ASSERT(lNumFound == 2);
        SER(my_test_GlobalCC_extrema(*pNurb1, *pNurb2, SM_SO_MAXIMIZE, lNumFound));
        SM_ASSERT(lNumFound == 2);
        
        pNurb1 = my_create_nurb(sContext, SmPoint3d(-9, - 5, 0), sPnts1, 3, 0, 0, 0);
        SmObjDelete sCU3(pNurb1);
        pNurb2 = my_create_nurb(sContext, SmPoint3d(-5, - 5, 0), sPnts2, 3, 1, 0.5, 0);
        SmObjDelete sCU4(pNurb2);
        SER(my_test_GlobalCC_extrema(*pNurb1, *pNurb2, SM_SO_MINIMIZE, lNumFound));
        SM_ASSERT(lNumFound == 3);
        SER(my_test_GlobalCC_extrema(*pNurb1, *pNurb2, SM_SO_MAXIMIZE, lNumFound));
        SM_ASSERT(lNumFound == 2);
        
        pNurb1 = my_create_nurb(sContext, SmPoint3d(9, - 5, 0), sPnts1, 3, 0, 0, 0);
        SmObjDelete sCU5(pNurb1);
        pNurb2 = my_create_nurb(sContext, SmPoint3d(5, - 5, 0), sPnts2, 3, 1, 0.5, 0);
        SmObjDelete sCU6(pNurb2);
        SER(my_test_GlobalCC_extrema(*pNurb1, *pNurb2, SM_SO_NORMALIZE, lNumFound));
        SM_ASSERT(lNumFound == 81);
        
        pNurb1 = my_create_nurb(sContext, SmPoint3d(12, - 5, 0), sPnts1, 3, 0, 0, 0);
        SmObjDelete sCU7(pNurb1);
        pNurb2 = my_create_line(sContext, SmPoint3d(14, - 5, 0), SmPoint3d(14, 5, 0), 1, 0.5, 0);
        SmObjDelete sCU8(pNurb2);
        SER(my_test_GlobalCC_extrema(*pNurb1, *pNurb2, SM_SO_AT_DISTANCE, lNumFound));
        SM_ASSERT(lNumFound == 10);
    }   
    
    return SM_SUCCESS;
} // end my_test_cc_nurb

/***********************************************************************
PURPOSE ---

USAGE NOTES ---
***********************************************************************/
PT_EXPORT SmStatus my_test_cc_analy()
{
  int lCount = -1;

  // if lCount is -1 then do all tests, else increment test
  if((lCount + 1) != 0)
    lCount++;

  if(lCount == 1 || lCount < 0)
  {
    SmCubicBezierSurface sSurf1, sSurf2;
    sSurf1.BuildBilinear( SmPoint3d( 0, 0, 0 ), SmPoint3d( 2, 0, 0 ),
        SmPoint3d( 0, 1, 0 ), SmPoint3d( 2, 1, 0 ) );
#ifdef SM_GFX_CODE 
    if(smGet_DoGraphics())
    {
      smgfx_Erase();
      smgfx_SetLook( 1, 2, 0, 0, 1 ); sSurf1.Draw(); sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
#endif
    sSurf1.Split( SM_SP_U, SmPoint3d( 1, 0, 0 ), SmVector3d( 1, 0, 0 ), SmVector3d( 0, 1, 0 ), SmVector3d( 0, 0, 0 ),
        SmPoint3d( 1, 1, 0 ), SmVector3d( 1, 0, 0 ), SmVector3d( 0, 1, 0 ), SmVector3d( 0, 0, 0 ),
        sSurf1, sSurf2 );
    //        sSurf1.Split(SM_SP_V,SmPoint3d(0,0.5,0),SmVector3d(2,0,0),SmVector3d(0,0.5,0),SmVector3d(0,0,0),
    //                             SmPoint3d(2,0.5,0),SmVector3d(2,0,0),SmVector3d(0,0.5,0),SmVector3d(0,0,0),
    //                            sSurf1,sSurf2);
#ifdef SM_GFX_CODE 
    if(smGet_DoGraphics())
    {
      smgfx_SetLook( 3, 4, 1, 0, 1 ); sSurf1.Draw(); sm_GraphicsLoop();
      smgfx_SetLook( 3, 4, 0, 1, 0 ); sSurf2.Draw(); sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
#endif
  } // 1

  //    SmContext sContext(pPool);
  SmContext sContext;


  SmTArray < SmPoint3d> sPnts;
  sPnts.Add( SmPoint3d( 0, 0.1, 0 ) );
  sPnts.Add( SmPoint3d( 0.2, 0, 0 ) );
  sPnts.Add( SmPoint3d( 0.4, 0.2, -0.1 ) );
  sPnts.Add( SmPoint3d( 0.6, -0.3, 0.2 ) );
  sPnts.Add( SmPoint3d( 1.0, 0.0, 0.1 ) );
  sPnts.Add( SmPoint3d( 1.1, 0.1, 0 ) );
  sPnts.Add( SmPoint3d( 1.0, 0.5, 0 ) );
  SmBSplineCurve *pNewBSC = my_create_nurb( sContext, SmPoint3d( 0, 0, 0 ),
      sPnts, 3, 1, 0.5, 0 );
  SmObjDelete sCU( pNewBSC );

  if(lCount == 2 || lCount < 0)
  {
    // Test mirror of curve
    SmAxis2Placement sMirror;
    sMirror.SetCanonical( SmPoint3d( 0, 0.5, 0 ), SmVector3d( 1, 0, 0 ),
        SmVector3d( 0, 0, 1 ) );
    SmCurve *pBSC;
    SER( pNewBSC->CreateMirrorCurve( sContext, sMirror, pBSC ) );
    SmObjDelete sClean( pBSC );
#ifdef SM_GFX_CODE
    if(smGet_DoGraphics())
    {
      smgfx_Erase();
      smgfx_SetLook( 1, 2, 0, 0, 1 ); pNewBSC->DrawWDeriv( pNewBSC->GetNaturalInterval(), 0 ); sm_GraphicsLoop();
      smgfx_SetLook( 1, 2, 0, 1, 0 ); pBSC->DrawWDeriv( pBSC->GetNaturalInterval(), 0 ); sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
#endif
  } // 2

  if(lCount == 3 || lCount < 0)
  {
    // For testing curve/curve closest point, farthest point and either
    SmAxis2Placement sA2P;
    sA2P.SetCanonical( SmPoint3d( 1, -3, 0 ), SmVector3d( 1, 0, 0 ), SmVector3d( 0, 1, 0 ) );
    SmBSplineCurve *pCir = NULL;
    SER( SmBSplineCurve::CreateCircleSegment( sContext, 3, sA2P, 1.0, 0.0, 360.0,
         SM_CO_QUADRATIC, pCir ) );
    SmObjDelete sCU1( pCir );
#ifdef SM_GFX_CODE
    if(smGet_DoGraphics())
    {
      smgfx_Erase();
      smgfx_SetLook( 1, 2, 0, 0, 1 ); pCir->Draw(); sm_GraphicsLoop();
    }
#endif

    SmBSplineCurve *pEll = NULL;
    sA2P.SetCanonical( SmPoint3d( -4, -3, 0 ), SmVector3d( 1, 0, 0 ), SmVector3d( 0, 1, 0 ) );
    SER( SmBSplineCurve::CreateEllipseSegment( sContext, 3, sA2P, 1.5,
         0.3, 0.0, 360.0, SM_CO_QUINTIC, pEll ) );
    SmObjDelete sCU2( pEll );
#ifdef SM_GFX_CODE
    if(smGet_DoGraphics())
    {
      smgfx_SetLook( 1, 2, 0, 1, 0 ); pEll->Draw(); sm_GraphicsLoop();
    }
#endif

    sA2P.SetCanonical( SmPoint3d( 0, 1, 0 ), SmVector3d( 1, 0, 0 ), SmVector3d( 0, 1, 0 ) );
    SmBSplineCurve *pCir2 = NULL;
    SER( SmBSplineCurve::CreateCircleSegment( sContext, 3, sA2P, 2.0, 0.0, 360.0, SM_CO_QUINTIC, pCir2 ) );
    SmObjDelete sCU3( pCir2 );
#ifdef SM_GFX_CODE
    if(smGet_DoGraphics())
    {
      smgfx_SetLook( 1, 2, 1, 0, 0 ); pCir2->Draw(); sm_GraphicsLoop();
    }
#endif

    sA2P.SetCanonical( SmPoint3d( 0.3, 0.4, 0 ), SmVector3d( 1, 0, 0 ), SmVector3d( 0, 1, 0 ) );
    SmBSplineCurve *pCir3 = NULL;
    SER( SmBSplineCurve::CreateCircleSegment( sContext, 3, sA2P, 1.0, 0.0, 360.0, SM_CO_QUINTIC, pCir3 ) );
    SmObjDelete sCU4( pCir3 );
#ifdef SM_GFX_CODE
    if(smGet_DoGraphics())
    {
      smgfx_SetLook( 1, 2, 1, 0, 1 ); pCir3->Draw(); sm_GraphicsLoop();
    }
#endif

    sA2P.SetCanonical( SmPoint3d( -2.5, -2.3, 0 ), SmVector3d( 0, 1, 0 ), SmVector3d( 0, 0, 1 ) );
    SmBSplineCurve *pCir4 = NULL;
    SER( SmBSplineCurve::CreateCircleSegment( sContext, 3, sA2P, 1.0, 0.0, 360.0, SM_CO_QUADRATIC, pCir4 ) );
    SmObjDelete sCU5( pCir4 );
#ifdef SM_GFX_CODE
    if(smGet_DoGraphics())
    {
      smgfx_SetLook( 1, 2, 1, .5, 0 ); pCir4->Draw(); sm_GraphicsLoop();
    }
#endif


    ULONG lNumFound;
    SER( my_test_GlobalCC_extrema( *pCir, *pEll, SM_SO_MINIMIZE, lNumFound ) );
    SM_ASSERT( lNumFound == 2 ); // gwc: min maps to closed curve so solutions include param=0 and param=1

    SER( my_test_GlobalCC_extrema( *pCir4, *pEll, SM_SO_MINIMIZE, lNumFound ) );
    SM_ASSERT( lNumFound == 1 );

#ifdef SM_GFX_CODE
    if(smGet_DoGraphics())
    {
      pNewBSC->Draw();
    }
#endif
    SER( my_test_GlobalCC_extrema( *pCir2, *pNewBSC, SM_SO_MINIMIZE, lNumFound ) );
    SM_ASSERT( lNumFound == 1 );

    SER( my_test_GlobalCC_extrema( *pCir2, *pCir3, SM_SO_MINIMIZE, lNumFound ) );
    SM_ASSERT( lNumFound == 1 );
  } // 3

  if(lCount == 4 || lCount < 0)
  {
    // For testing closest point, farthest point and either
    SmBSplineCurve *pLine = NULL;
    SER( SmBSplineCurve::CreateLineSegment( sContext, 3, SmPoint3d( 0.0, 0.0, 0 ),
         SmPoint3d( 1.0, 0.0, 0.0 ), pLine ) );
    SmObjDelete sCU1( pLine );
#ifdef SM_GFX_CODE
    if(smGet_DoGraphics())
    {
      smgfx_Erase();
      smgfx_SetLook( 1, 2, 0, 0, 0 ); pLine->Draw(); sm_GraphicsLoop();
    }
#endif
    SER( my_test_point_extrema( *pLine, pLine->GetNaturalInterval(),
         SM_SO_AT_DISTANCE, 1, 1, 1 ) );   // 1 x 1 x 1 tests

#ifdef SM_GFX_CODE
    if(smGet_DoGraphics())
    {
      smgfx_SetLook( 1, 2, 0, 0, 1 ); pNewBSC->Draw(); sm_GraphicsLoop();
    }
#endif
    SER( my_test_point_extrema( *pNewBSC, pNewBSC->GetNaturalInterval(),
         SM_SO_AT_DISTANCE, 10, 10, 10 ) );  // 10 x 10 x 10 tests
    SER( my_test_drop_point( *pNewBSC, pNewBSC->GetNaturalInterval(), 1000 ) );

    SmAxis2Placement sA2P;
    sA2P.SetCanonical( SmPoint3d( 0, 3, 0 ), SmVector3d( 1, 0, 0 ), SmVector3d( 0, 1, 0 ) );
    SmBSplineCurve *pCir = NULL;
    SER( SmBSplineCurve::CreateCircleSegment( sContext, 3, sA2P, 1.0, 0.0,
         360.0, SM_CO_QUADRATIC, pCir ) );
    SmObjDelete sCU2( pCir );
#ifdef SM_GFX_CODE
    if(smGet_DoGraphics())
    {
      smgfx_SetLook( 1, 2, 0, 1, 0 ); pCir->Draw(); sm_GraphicsLoop();
    }
#endif
    SER( my_test_point_extrema( *pCir, pNewBSC->GetNaturalInterval(), SM_SO_AT_DISTANCE, 10, 10, 10 ) );
    SER( my_test_drop_point( *pCir, pNewBSC->GetNaturalInterval(), 1000 ) );

    sA2P.SetCanonical( SmPoint3d( 0, -3, 0 ), SmVector3d( 1, 0, 0 ), SmVector3d( 0, 1, 0 ) );
    SmBSplineCurve *pEll = NULL;
    SER( SmBSplineCurve::CreateEllipseSegment( sContext, 3, sA2P, 1.5, 0.3, 0.0, 360.0, SM_CO_QUINTIC, pEll ) );
    SmObjDelete sCU3( pEll );
#ifdef SM_GFX_CODE
    if(smGet_DoGraphics())
    {
      smgfx_SetLook( 1, 2, 1, 0, 0 ); pEll->Draw(); sm_GraphicsLoop();
      pEll->DrawPolygon();
    }
#endif
    SER( my_test_point_extrema( *pEll, pNewBSC->GetNaturalInterval(), SM_SO_AT_DISTANCE, 10, 10, 10 ) );
    SER( my_test_drop_point( *pEll, pNewBSC->GetNaturalInterval(), 1000 ) );
  } // 4

  if(lCount == 5 || lCount < 0)
  {  // Test AT_Distance
    SmBSplineCurve *pLine = NULL;
    SER( SmBSplineCurve::CreateLineSegment( sContext, 3,
         SmPoint3d( -0.5, 2.0, 0 ), SmPoint3d( -0.5, 8.0, 0.0 ), pLine ) );
    SmObjDelete sCU1( pLine );
#ifdef SM_GFX_CODE
    if(smGet_DoGraphics())
    {
      smgfx_Erase();
      smgfx_SetLook( 1, 2, 0, 0, 1 ); pLine->Draw(); sm_GraphicsLoop();
    }
#endif

    SmAxis2Placement sA2P;
    sA2P.SetCanonical( SmPoint3d( 0, 5, 0 ), SmVector3d( 1, 0, 0 ), SmVector3d( 0, 1, 0 ) );
    SmBSplineCurve *pCir = NULL;
    SER( SmBSplineCurve::CreateCircleSegment( sContext, 3, sA2P, 1.0, 0.0, 360.0, SM_CO_QUADRATIC, pCir ) );
    SmObjDelete sCU2( pCir );
#ifdef SM_GFX_CODE
    if(smGet_DoGraphics())
    {
      smgfx_SetLook( 1, 2, 0, 1, 0 ); pCir->Draw(); sm_GraphicsLoop();
    }
#endif
    SmBSplineCurve *pCir2 = NULL;
    sA2P.SetCanonical( SmPoint3d( 1, -1, 0 ), SmVector3d( 1, 0, 0 ), SmVector3d( 0, 1, 0 ) );
    SER( SmBSplineCurve::CreateCircleSegment( sContext, 3, sA2P, 1.0, 0.0, 360.0, SM_CO_QUINTIC, pCir2 ) );
    SmObjDelete sCU3( pCir2 );
#ifdef SM_GFX_CODE
    if(smGet_DoGraphics())
    {
      smgfx_SetLook( 1, 2, 0, 1, 0 ); pCir2->Draw(); sm_GraphicsLoop();
    }
#endif
    SmBSplineCurve *pLine2 = NULL;
    SER( SmBSplineCurve::CreateLineSegment( sContext, 3,
         SmPoint3d( 1.5, 2.0, 0 ), SmPoint3d( 1.5, 8.0, 0.0 ), pLine2 ) );
    SmObjDelete sCU4( pLine2 );
#ifdef SM_GFX_CODE
    if(smGet_DoGraphics())
    {
      smgfx_SetLook( 1, 2, 1, 0, 0 ); pLine2->Draw(); sm_GraphicsLoop();
    }
#endif
    ULONG lNumFound;
    SER( my_test_GlobalCC_extrema( *pCir, *pLine2, SM_SO_AT_DISTANCE, lNumFound ) );
    SM_ASSERT( lNumFound == 2 );
    SER( my_test_GlobalCC_extrema( *pCir, *pLine, SM_SO_AT_DISTANCE, lNumFound ) );
    SM_ASSERT( lNumFound == 2 ); // GWC: this test returns the boundary points where
#ifdef SM_GFX_CODE                 //      the circle transitions from being further
    if(smGet_DoGraphics())         //      to closer than the internally derived distance,
    {                          //      e.g. it marks the Local Neighborhood ends within distance of the line.
      smgfx_SetLook( 1, 2, 1, 0, 0 ); pNewBSC->Draw(); sm_GraphicsLoop();
    }
#endif
    SER( my_test_GlobalCC_extrema( *pCir2, *pNewBSC, SM_SO_AT_DISTANCE, lNumFound ) );
    SM_ASSERT( lNumFound == 0 );  // gwc: This test marks the boundaries of the local Neighborhood.
    lCount = 0;                 //      Turns out that a radius 1 circle that intersects
  } // 5                          //      pNewBSC is completely within the test's distance of 2.236,
                                  //      meaning the local Neighborhood is the entire curve and the
                                  //      local neighborhood has no boundaries.    
  return SM_SUCCESS;
} // end my_test_cc_analy


/***********************************************************************
PURPOSE ---

USAGE NOTES ---
***********************************************************************/
PT_EXPORT SmStatus my_test_methods()
{
    my_test_polynomial();
    
    SmContext sContext;
    
    {
        SmCurve *pBSC16;
        SmBSplineCurve *pCir3 = my_create_circle(sContext, 1.0, SmPoint3d(0, 0, 0), SM_CO_QUADRATIC, 0, 1, 1);
        SmObjDelete sClean(pCir3);
        SER(pCir3->CreatePlaneProjection(sContext, SM_PT_PARALLEL, SmPoint3d(0, 0, 0),
            SmVector3d(1, 0, 0), SmVector3d(1, 0, 0), pBSC16));
        SmObjDelete sCU3(pBSC16);
        SmBSplineCurve *pApprox = NULL;
        double dAchTol;
        SmTArray < double> sBreaks;
        pBSC16->GetKnots(sBreaks);
        SER(pBSC16->ApproximateCurve(sContext, sBreaks, 0.00001, dAchTol, pApprox));
        SmObjDelete sCleanA( pApprox );
#ifdef SM_GFX_CODE
        if (smGet_DoGraphics())
        {
            smgfx_SetColor(1, 0, 0);
            pCir3->Dump();
            pBSC16->Dump();
            pApprox->Dump();
            pBSC16->Draw();
            smgfx_SetColor(1, 0, 1);
        }
#endif
    }
    
    
    
    SmTArray < SmPoint3d> sPnts;
    sPnts.Add(SmPoint3d(0, 0.1, 0));
    sPnts.Add(SmPoint3d(0.2, 0, 0));
    sPnts.Add(SmPoint3d(0.4, 0.2, - 0.1));
    sPnts.Add(SmPoint3d(0.6, - 0.3, 0.2));
    sPnts.Add(SmPoint3d(1.0, 0.0, 0.1));
    sPnts.Add(SmPoint3d(1.1, 0.1, 0));
    sPnts.Add(SmPoint3d(1.0, 0.5, 0));
    SmBSplineCurve *pNewBSC = my_create_nurb(sContext, SmPoint3d(0, 0, 0), sPnts, 3, 1, 0.5, 0);
    SmObjDelete sCU(pNewBSC);
    
    {
        if (pNewBSC->IsRational())
            SER(SM_ERR);
        SmBSplineCurve *pCir3 = my_create_circle(sContext, 1.0, SmPoint3d(1.5, 2.5, 0), SM_CO_QUADRATIC, 0, 1, 1);
        SmObjDelete sClean(pCir3);
        if (!pCir3->IsRational())
            SER(SM_ERR);

        // ULONG lNumKnots1 = pCir3->GetNumberNaturalKnots();

        double dArray[100];
        SER(pCir3->GetKnotsExpert(2, 5, dArray));
        SER(pCir3->GetControlPointsExpert(SM_CP_NON_RATIONAL, 0, 4, 3, dArray));
        SER(pCir3->GetControlPointsExpert(SM_CP_EUCLIDIAN_RATIONAL, 0, 4, 4, dArray));
        SER(pCir3->GetControlPointsExpert(SM_CP_HOMOGENEOUS_RATIONAL, 0, 4, 4, dArray));
        double dKnots[100]; 
        double dPoints[300];
        ULONG lNumKnots = pCir3->GetNumberNaturalKnots();
        ULONG lNumCpts = pCir3->GetNumberControlPoints();
        SER(pCir3->GetKnotsExpert(0, lNumKnots - 1, dKnots));
        SER(pCir3->GetControlPointsExpert(SM_CP_HOMOGENEOUS_RATIONAL, 0, lNumCpts - 1, 4, dPoints));
        SmBSplineCurve sTest;
        sTest.SetContext(&sContext);
        SER(sTest.SetExpert(3, 2, SM_CF_UNSPECIFIED,
            SM_EK_CLAMPPED, lNumKnots, dKnots,
            SM_CP_HOMOGENEOUS_RATIONAL,
            4, dPoints));
        pCir3->Dump();
        sTest.Dump();
    }
    
    {
        double dKnots[100]; 
        double dPoints[300];
        ULONG lNumKnots = pNewBSC->GetNumberNaturalKnots();
        ULONG lNumCpts = pNewBSC->GetNumberControlPoints();
        SER(pNewBSC->GetKnotsExpert(0, lNumKnots - 1, dKnots));
        SER(pNewBSC->GetControlPointsExpert(SM_CP_NON_RATIONAL, 0, lNumCpts - 1, 4, dPoints));
        SmBSplineCurve sTest;
        sTest.SetContext(&sContext);
        SER(sTest.SetExpert(3, pNewBSC->GetDegree(), SM_CF_UNSPECIFIED,
            SM_EK_CLAMPPED, lNumKnots, dKnots,
            SM_CP_NON_RATIONAL,
            4, dPoints));
        pNewBSC->Dump();
        sTest.Dump();
    }
    
    {  // Test length
        double dLength;
        SmBSplineCurve *pCir3 = my_create_circle(sContext, 1.0, SmPoint3d(1.5, 2.5, 0), SM_CO_QUADRATIC, 0, 1, 1);
        SmObjDelete sCU1(pCir3);
        SER(pCir3->Length(pCir3->GetNaturalInterval(), SM_EFF_ZERO*1000, dLength));
        if (smos_Fabs(dLength - SM_PI*2.0) > SM_EFF_ZERO*1000)
            SER(SM_ERR);
        // Test Length
        SmBSplineCurve *pLine = my_create_line(sContext, SmPoint3d(1, 2, 0), SmPoint3d(3, 2, 0), 0.5, 0.5, 0.0);
        SmObjDelete sCU2(pLine);
        SER(pLine->Length(pLine->GetNaturalInterval(), SM_EFF_ZERO*1000, dLength));
        if (smos_Fabs(dLength - 2.0) > SM_EFF_ZERO)
            SER(SM_ERR);
    }
    
    
    return SM_SUCCESS;
} // end my_test_methods



/***********************************************************************
PURPOSE ---

USAGE NOTES ---
***********************************************************************/
SmStatus my_test_analytic_curve(SmBSplineCurve * pCurve, SmExtent1d & rIvlSTEP)
{
    SER(pCurve->AdjustSTEPInterval(rIvlSTEP));
    
#ifdef SM_GFX_CODE
    if (smGet_DoGraphics())
    {
        smgfx_Erase();
        smgfx_SetLook(1,2, 1,0,0); pCurve->Draw(); sm_GraphicsLoop();
        sm_GraphicsLoop();
    }
#endif
    
    
    SmExtent1d sNurbIvl = pCurve->GetNaturalInterval();
    double dStep = sNurbIvl.GetLength()/10.0;
    for (ULONG i = 0; i <= 10; i++)
    {
        double dParam = i*dStep + sNurbIvl.GetMin();
        if (i == 10)
            dParam = sNurbIvl.GetMax();
        SmPoint3d sPnt;
        SER(pCurve->EvaluatePoint(dParam, sPnt));
        SmSolutionArray sSolutions;
        double dTolerance = SM_ZONE_TOL_3D;
        SER(pCurve->GlobalPointSolveSTEP(rIvlSTEP, SM_SO_INTERSECT,
            sPnt, dTolerance, NULL, NULL, SM_SR_ALL, sSolutions));
        for (ULONG j = 0; j < sSolutions.GetSize(); j++)
        {
            double dAnalParam = sSolutions[j].m_vStart[0];
            SmPoint3d sAnalPnt;
            SER(pCurve->EvaluateSTEP(dAnalParam, 0, TRUE, &sAnalPnt));
            double dDist = sAnalPnt.DistanceBetween(sPnt);
            if (dDist > dTolerance)
            {
                SER(SM_ERR);
            }
        }
    }
    
    pCurve->Dump();
    pCurve->ReverseParameterization(sNurbIvl, sNurbIvl);
    pCurve->Dump();
    
#ifdef SM_GFX_CODE
    if (smGet_DoGraphics())
    {
        smgfx_Erase();
        smgfx_SetLook(1,2, 1,0,0); pCurve->Draw(); sm_GraphicsLoop();
        sm_GraphicsLoop();
    }
#endif
    return SM_SUCCESS;
} // end my_test_analytic_curve


/***********************************************************************
PURPOSE ---

USAGE NOTES ---
***********************************************************************/
PT_EXPORT SmStatus my_test_analytic_curves()
{
    SmContext sContext;
    SmBSplineCurve * pCurve = NULL;
    
    if (TRUE)
    {
        // Line Tests
        SmPoint3d sLinePoint(1.0, 0.0, 0.0);
        SmVector3d sLineVector(0.0, 0.0, 1.0);
        SmLine * pLine = NULL;
        SmLine::CreateCanonical(sContext, sLinePoint, sLineVector, pLine);
        pCurve = pLine;
        SmObjDelete sCU(pCurve);
        SmExtent1d sIvlSTEP(-1.0, 2.0);
        SER(my_test_analytic_curve(pCurve, sIvlSTEP));
    }
    
    if (TRUE)
    {
        // Circle Tests
        SmPoint3d sOrigin(1.0, 0.0, 0.0);
        SmVector3d sX(1.0, 0.0, 0.0);
        SmVector3d sY(0.0, 0.0, 1.0);
        SmAxis2Placement sPos;
        sPos.SetCanonical(sOrigin, sX, sY);
        SmCircle * pCircle = NULL;
        SmCircle::CreateCanonical(sContext, sPos, 0.5, pCircle);
        pCurve = pCircle;
        SmObjDelete sCU(pCurve);
        SmExtent1d sIvlSTEP(0.0, 90.0);
        SER(my_test_analytic_curve(pCurve, sIvlSTEP));
    }
    
    if (TRUE)
    {
        // Ellipse Tests
        SmPoint3d sOrigin(1.0, 0.0, 0.0);
        SmVector3d sX(1.0, 0.0, 0.0);
        SmVector3d sY(0.0, 0.0, 1.0);
        SmAxis2Placement sPos;
        sPos.SetCanonical(sOrigin, sX, sY);
        SmEllipse * pEllipse = NULL;
        SmEllipse::CreateCanonical(sContext, sPos, 1.0, 0.5, pEllipse);
        pCurve = pEllipse;
        SmObjDelete sCU(pCurve);
        SmExtent1d sIvlSTEP(0.0, 270.0);
        SER(my_test_analytic_curve(pCurve, sIvlSTEP));
    }
    
    if (TRUE)
    {
        // Parabola Tests
        SmPoint3d sOrigin(0.0, 0.0, 0.0);
        SmVector3d sX(-1.0, 0.0, 0.0);
        SmVector3d sY(0.0, 0.0, 1.0);
        SmAxis2Placement sPos;
        sPos.SetCanonical(sOrigin, sX, sY);
        SmParabola * pParabola = NULL;
        SmParabola::CreateCanonical(sContext, sPos, 0.5, pParabola);
        pCurve = pParabola;
        SmObjDelete sCU(pCurve);
        SmExtent1d sIvlSTEP(-1.0, 2.0);
        SER(my_test_analytic_curve(pCurve, sIvlSTEP));
    }
    
    if (TRUE)
    {
        // Hyperbola Tests
        SmPoint3d sOrigin(0.0, 0.0, 0.0);
        SmVector3d sX(-1.0, 0.0, 0.0);
        SmVector3d sY(0.0, 0.0, 1.0);
        SmAxis2Placement sPos;
        sPos.SetCanonical(sOrigin, sX, sY);
        SmHyperbola * pHyperbola = NULL;
        SmHyperbola::CreateCanonical(sContext, sPos, 1.0, 1.0, pHyperbola);
        pCurve = pHyperbola;
        SmObjDelete sCU(pCurve);
        SmExtent1d sIvlSTEP(-1.0, 2.0);
        SER(my_test_analytic_curve(pCurve, sIvlSTEP));
    }
    
    return SM_SUCCESS;
} // end my_test_analytic_curves




/***********************************************************************
PURPOSE ---

USAGE NOTES ---
***********************************************************************/
PT_EXPORT SmStatus my_test_creation(void)
{
    SER(my_test_analytic_curves());
    //    SmContext sContext(pPool);
    SmContext sContext;
    
    SmTArray < SmPoint3d> sPnts;
    sPnts.Add(SmPoint3d(0, 0.1, 0));
    sPnts.Add(SmPoint3d(0.2, 0, 0));
    sPnts.Add(SmPoint3d(0.4, 0.2, - 0.1));
    sPnts.Add(SmPoint3d(0.6, - 0.3, 0.2));
    sPnts.Add(SmPoint3d(1.0, 0.0, 0.1));
    sPnts.Add(SmPoint3d(1.1, 0.1, 0));
    sPnts.Add(SmPoint3d(1.0, 0.5, 0));
    
    {
        SmBSplineCurve *pBSC17 = NULL;
        SmBSplineCurve *pBSC16 = NULL;
        SER(SmBSplineCurve::CreateLineSegment(sContext, 3, SmPoint3d(0.2, 0.2, 0), SmPoint3d(0.5, 0.1, 1.0), pBSC17));
        SER(SmBSplineCurve::CreateLineSegment(sContext, 2, SmPoint3d(0.2, 0.2, 0), SmPoint3d(0.5, 0.1, 1.0), pBSC16));
        SmObjDelete sCU2(pBSC17);
        SmObjDelete sCU3(pBSC16);
    }
    
    
    if (FALSE)
    {
        SmAxis2Placement sA2P;
        sA2P.SetCanonical(SmPoint3d(0, 2, 2), SmVector3d(0, 1, 0), SmVector3d(0, 0, 1));
        SmBSplineCurve *pBSC16 = NULL;
        SER(SmBSplineCurve::CreateEllipseSegment(sContext, 3, sA2P, 1.5, 0.3, 0.0, 360.0, SM_CO_QUINTIC, pBSC16));
        SmAxis2Placement sA2P2;
        sA2P2.SetCanonical(SmPoint3d(1, 0, 0), SmVector3d(0, 1, 0), SmVector3d(0, 0, 1));
        SmBSplineCurve *pBSC17 = NULL;
        SER(SmBSplineCurve::CreateEllipseSegment(sContext, 3, sA2P2, 0.3, 1.5, 0.0, 180.0, SM_CO_QUADRATIC, pBSC17));
        SmObjDelete sCU2(pBSC17);
        SmObjDelete sCU3(pBSC16);
    }
    
    if (TRUE)
    {
        SmAxis2Placement sA2P;
        sA2P.SetCanonical(SmPoint3d(2, 2, 2), SmVector3d(0, 1, 0), SmVector3d(0, 0, 1));
        SmBSplineCurve *pBSC16 = NULL;
        SER(SmBSplineCurve::CreateCircleSegment(sContext, 3, sA2P, 1.5, 0.0, 360.0, SM_CO_QUINTIC, pBSC16));
        SmAxis2Placement sA2P2;
        sA2P2.SetCanonical(SmPoint3d(0, 0, 0), SmVector3d(0, 1, 0), SmVector3d(0, 0, 1));
        SmBSplineCurve *pBSC17 = NULL;
        SER(SmBSplineCurve::CreateCircleSegment(sContext, 3, sA2P2, 1.5, 0.0, 90.0, SM_CO_QUADRATIC, pBSC17));
        SmObjDelete sCU2(pBSC17);
        SmObjDelete sCU3(pBSC16);
        {
            ULONG dim;
            ULONG degree;
            SmTArray < SmPoint3d> pts(100);
            SmBSplineCurveForm f;
            SmTArray < ULONG> mult(100);
            SmTArray < double> knots(100);
            SmKnotType type;
            SmTArray < double> weights(100);
            
            pBSC17->GetCanonical(dim, degree, pts, f, mult, knots, type, weights);
            
            pBSC17->Dump();
            SmBSplineCurve *pArc = NULL;
            
            SmBSplineCurve::CreateCanonical(sContext,
                3,                  // 3 dimensional
                2,                  // quadratic NURBS
                pts,
                SM_CF_UNSPECIFIED,  // no specific form
                mult,
                knots,
                SM_KT_UNSPECIFIED,  // knot type
                // could perhaps use Bezier type?
                &weights,
                0,                  // untrimmed
                pArc);
            SmObjDelete sCU99(pArc);
            
            pArc->Dump();
        }            
    }
    
    if (TRUE)
    {
        SmTArray < SmPoint3d> sCPnts;
        sCPnts.Add(SmPoint3d(0, 0, 0));
        sCPnts.Add(SmPoint3d(0, 0.2, 0));
        sCPnts.Add(SmPoint3d(0, 0.4, 0.2));
        sCPnts.Add(SmPoint3d(0.2, 0.6, - 0.3));
        sCPnts.Add(SmPoint3d(0, 0.8, 0));
        sCPnts.Add(SmPoint3d(0, 0.9, 0.2));
        sCPnts.Add(SmPoint3d(0, 1.0, 0.5));
        
        SmTArray < ULONG> sKnotMult;
        sKnotMult.Add(4);
        sKnotMult.Add(1);
        sKnotMult.Add(1);
        sKnotMult.Add(1);
        sKnotMult.Add(4);
        
        SmTArray < double> sKnots;
        sKnots.Add(0.0);
        sKnots.Add(0.3);
        sKnots.Add(0.6);
        sKnots.Add(0.8);
        sKnots.Add(1.0);
        
        SmTArray < double> sWeights;
        sWeights.Add(1.5);
        sWeights.Add(0.5);
        sWeights.Add(1.5);
        sWeights.Add(0.5);
        sWeights.Add(1.5);
        sWeights.Add(0.5);
        sWeights.Add(1.5);
        
        SmBSplineCurve *pBSC12 = NULL, *pBSC13 = NULL;
        SER(SmBSplineCurve::CreateCanonical(sContext, 3, 3, sCPnts, SM_CF_UNSPECIFIED,
            sKnotMult, sKnots, SM_KT_UNSPECIFIED, &sWeights, NULL, pBSC12));
        SmObjDelete sCU4(pBSC12);
        pBSC12->Dump();
        
        ULONG lDim, lDeg;
        SmBSplineCurveForm eBSplineCurveForm;
        SmKnotType eKnotType;
        SER(pBSC12->GetCanonical(lDim, lDeg, sCPnts, eBSplineCurveForm,
            sKnotMult, sKnots, eKnotType, sWeights));
        SM_DUMP_TARRAY(sCPnts) ; // sCPnts.Dump();
        SM_DUMP_TARRAY(sKnotMult) ; // sKnotMult.Dump();
        SM_DUMP_TARRAY(sKnots) ; // sKnots.Dump();
        SM_DUMP_TARRAY(sWeights) ; // sWeights.Dump();
        SER(SmBSplineCurve::CreateCanonical(sContext, 3, 3, sCPnts, SM_CF_UNSPECIFIED,
            sKnotMult, sKnots, SM_KT_UNSPECIFIED, &sWeights, NULL, pBSC13));
        SmObjDelete sCU5(pBSC13);
    }
    
    return SM_SUCCESS;
} // end my_test_creation








/***********************************************************************
PURPOSE ---

USAGE NOTES ---
***********************************************************************/
PT_EXPORT SmStatus my_test_cp_nurb(void)
{
    //    SmContext sContext(pPool);
    SmContext sContext;
    
    {
        SmTArray < SmPoint3d> sPnts1;
        sPnts1.Add(SmPoint3d(-1, 0.5, 0));            
        sPnts1.Add(SmPoint3d(0, 0.5, 0));
        sPnts1.Add(SmPoint3d(1, 1, 0));
        sPnts1.Add(SmPoint3d(0, 1.5, 0));
        sPnts1.Add(SmPoint3d(-1, 2, 0));
        sPnts1.Add(SmPoint3d(0, 2.5, 0));
        sPnts1.Add(SmPoint3d(1, 3, 0));
        sPnts1.Add(SmPoint3d(0, 3.5, 0));
        sPnts1.Add(SmPoint3d(-1, 4, 0));
        sPnts1.Add(SmPoint3d(0, 4.5, 0));
        sPnts1.Add(SmPoint3d(1, 5, 0));
        sPnts1.Add(SmPoint3d(0, 5.5, 0));
        sPnts1.Add(SmPoint3d(-1, 6, 0));
        sPnts1.Add(SmPoint3d(0, 6.5, 0));
        sPnts1.Add(SmPoint3d(1, 7, 0));
        sPnts1.Add(SmPoint3d(0, 7.5, 0));
        sPnts1.Add(SmPoint3d(-1, 8, 0));
        sPnts1.Add(SmPoint3d(0, 8.5, 0));
        sPnts1.Add(SmPoint3d(1, 9, 0));
        sPnts1.Add(SmPoint3d(0, 9.5, 0));
        sPnts1.Add(SmPoint3d(-1, 9.5, 0));
        SmBSplineCurve *pNurb1 = my_create_nurb(sContext, SmPoint3d(-8, - 5, 0), sPnts1, 3, 1.0, 0.5, 0);
        SmObjDelete sCleanUp1(pNurb1);
#ifdef SM_GFX_CODE
        smgfx_SetColor(0.7, 0.3, 0.2);
#endif
        
        SmAxis2Placement sA2P;
        sA2P.SetCanonical(SmPoint3d(0, 7, - 5), SmVector3d(1, 0, 0), SmVector3d(0, 0, 1));
        
        SER(my_test_point_extrema(*pNurb1, pNurb1->GetNaturalInterval(), SM_SO_MINIMIZE, 5, 19, 1));
        pNurb1->Transform(sA2P);
#ifdef SM_GFX_CODE
        if( smGet_DoGraphics()){
            smgfx_SetColor(1.0, 0.5, 0.0);
            pNurb1->Draw();
          }
#endif
        SER(my_test_point_extrema(*pNurb1, pNurb1->GetNaturalInterval(), SM_SO_MINIMIZE, 5, 1, 19));
        
        pNurb1 = my_create_nurb(sContext, SmPoint3d(-4, - 5, 0), sPnts1, 3, 1.0, 0.5, 0);
        SmObjDelete sCleanUp2(pNurb1);
#ifdef SM_GFX_CODE
        smgfx_SetColor(0.7, 0.3, 0.2);
#endif
        SER(my_test_point_extrema(*pNurb1, pNurb1->GetNaturalInterval(), SM_SO_NORMALIZE, 5, 5, 1));
        SmVector3d sScale(1, 0.5, 1);
        pNurb1->Transform(sA2P, &sScale);
#ifdef SM_GFX_CODE
        if( smGet_DoGraphics()){
        smgfx_SetColor(1.0, 0.5, 0.0);
        pNurb1->Draw();
          }
#endif
        SER(my_test_point_extrema(*pNurb1, pNurb1->GetNaturalInterval(), SM_SO_NORMALIZE, 5, 1, 5));
        
        pNurb1 = my_create_nurb(sContext, SmPoint3d(0, - 5, 0), sPnts1, 3, 1.0, 0.5, 0);
        SmObjDelete sCleanUp3(pNurb1);
#ifdef SM_GFX_CODE
        smgfx_SetColor(0.7, 0.3, 0.2);
#endif
        SER(my_test_point_extrema(*pNurb1, pNurb1->GetNaturalInterval(), SM_SO_MAXIMIZE, 5, 5, 1));
        pNurb1->Transform(sA2P);
#ifdef SM_GFX_CODE
        if( smGet_DoGraphics()){
        smgfx_SetColor(1.0, 0.5, 0.0);
        pNurb1->Draw();
          }
#endif
        SER(my_test_point_extrema(*pNurb1, pNurb1->GetNaturalInterval(), SM_SO_MAXIMIZE, 5, 1, 5));
        
        pNurb1 = my_create_nurb(sContext, SmPoint3d(4, - 5, 0), sPnts1, 3, 1.0, 0.5, 0);
        SmObjDelete sCleanUp4(pNurb1);
#ifdef SM_GFX_CODE
        smgfx_SetColor(0.7, 0.3, 0.2);
#endif
        SER(my_test_point_extrema(*pNurb1, pNurb1->GetNaturalInterval(), SM_SO_AT_DISTANCE, 5, 5, 1));
        pNurb1->Transform(sA2P);
#ifdef SM_GFX_CODE
        if( smGet_DoGraphics()){
        smgfx_SetColor(1.0, 0.5, 0.0);
        pNurb1->Draw();
          }
#endif
        SER(my_test_point_extrema(*pNurb1, pNurb1->GetNaturalInterval(), SM_SO_AT_DISTANCE, 5, 1, 5));
    }
    
    
    //    pPool->Free();
    //    delete pPool; pPool = NULL ;
    
    return SM_SUCCESS;
} // end my_test_cp_nurb






/***********************************************************************
PURPOSE ---

USAGE NOTES ---
***********************************************************************/
PT_EXPORT SmStatus my_test_cci_projection(void)
{
    SmContext sContext;
    
    if (FALSE)
    {
        SmTArray < SmPoint3d> sPnts1;
        sPnts1.Add(SmPoint3d(-1, 0.5, 0.6));            
        sPnts1.Add(SmPoint3d(0, 0.5, 0.5));
        sPnts1.Add(SmPoint3d(1, 1, 0.4));
        sPnts1.Add(SmPoint3d(0, 1.5, 0.3));
        sPnts1.Add(SmPoint3d(-1, 2, 0.2));
        sPnts1.Add(SmPoint3d(1, 4.5, 0.1));
        sPnts1.Add(SmPoint3d(0, 3.5, 0.0));
        sPnts1.Add(SmPoint3d(-1, 4, 0.-1));
        sPnts1.Add(SmPoint3d(1, 5, 0.-0.2));
        sPnts1.Add(SmPoint3d(-1, 6, - 0.3));
        sPnts1.Add(SmPoint3d(0, 6.5, - 0.4));
        sPnts1.Add(SmPoint3d(1, 7, - 0.5));
        sPnts1.Add(SmPoint3d(0, 7.5, - 0.6));
        sPnts1.Add(SmPoint3d(-1, 8, - 0.5));
        sPnts1.Add(SmPoint3d(0, 8.5, - 0.4));
        sPnts1.Add(SmPoint3d(1, 9, - 0.3));
        sPnts1.Add(SmPoint3d(-1, 9.5, - 0.2));
        SmBSplineCurve *pNurb1 = my_create_nurb(sContext, SmPoint3d(10, - 5, 0), sPnts1, 3, 0, 0, 0);
        SmObjDelete sCU1(pNurb1);
        SmTArray < SmPoint3d> sPnts2;
        sPnts2.Add(SmPoint3d(1, 0.5, 0));            
        sPnts2.Add(SmPoint3d(0, 0.5, 0));
        sPnts2.Add(SmPoint3d(-1, 1, 0));
        sPnts2.Add(SmPoint3d(0, 1.5, 0));
        sPnts2.Add(SmPoint3d(1, 2, 0));
        sPnts2.Add(SmPoint3d(-1, 4.5, 0));
        sPnts2.Add(SmPoint3d(0, 3.5, 0));
        sPnts2.Add(SmPoint3d(1, 4, 0));
        sPnts2.Add(SmPoint3d(-1, 5, 0));
        sPnts2.Add(SmPoint3d(1, 6, 0));
        sPnts2.Add(SmPoint3d(0, 6.5, 0));
        sPnts2.Add(SmPoint3d(-1, 7, 0));
        sPnts2.Add(SmPoint3d(0, 7.5, 0));
        sPnts2.Add(SmPoint3d(1, 8, 0));
        sPnts2.Add(SmPoint3d(0, 8.5, 0));
        sPnts2.Add(SmPoint3d(-1, 9, 0));
        sPnts2.Add(SmPoint3d(1, 9.5, 0));
        SmBSplineCurve *pNurb2 = my_create_nurb(sContext, SmPoint3d(10, - 5, 0), sPnts2, 3, 1, 0.5, 0);
        SmObjDelete sCU2(pNurb2);
        //    SmBSplineCurve *pLine = my_create_line(sContext,SmPoint3d(0,1.0,1.5),SmPoint3d(0,1.0,-1.5),0.0,0.0,0.0);
        //    SmBSplineCurve *pLine = my_create_line(sContext,SmPoint3d(0,-1.0,0.5),SmPoint3d(0,1.5,0),0.0,0.0,0.0);
        //    SmBSplineCurve *pLine = my_create_line(sContext,SmPoint3d(0,-1.0,0.5),SmPoint3d(0,1.0,0),0.0,0.0,0.0);
        //    SmBSplineCurve *pLine = my_create_line(sContext,SmPoint3d(0,-1.0,0.75),SmPoint3d(0,1.0,-0.5),0.0,0.0,0.0);
        //    SmObjDelete sCU2(pLine);
        
        SmVector3d sDirNorm[3];
        sDirNorm[0] = SmVector3d(0, 0, 1);
        //    sDirNorm[1] = SmVector3d(0,0,1);
        //    sDirNorm[1] = SmVector3d(1,0,0);
        sDirNorm[1] = SmVector3d(0, 1, 0);
        //    sDirNorm[1] = SmVector3d(0,0,1);
        
        SmSolutionArray sSolutions;
        
        SER(pNurb1->GlobalCurveSolve(pNurb1->GetNaturalInterval(),
            *pNurb1, pNurb1->GetNaturalInterval(),
            SM_SO_PROJECTED_INTERSECT, SM_ZONE_TOL_3D,
            NULL, sDirNorm,
            SM_SR_ALL,
            sSolutions));
        //    SER(pNurb1->GlobalCurveSolve(pNurb1->GetNaturalInterval(),
        //        *pNurb2,pNurb2->GetNaturalInterval(),
        //        SM_SO_PROJECTED_INTERSECT, SM_ZONE_TOL_3D,
        //        NULL, sDirNorm,
        //        SM_SR_ALL,
        //        sSolutions));
        //    SER(pNurb1->GlobalCurveSolve(pNurb1->GetNaturalInterval(),
        //        *pLine,pLine->GetNaturalInterval(),
        //        SM_SO_PROJECTED_INTERSECT, SM_ZONE_TOL_3D,
        //        NULL, sDirNorm,
        //        SM_SR_ALL,
        //       sSolutions));
        
        my_draw_2crv_sol(pNurb1, pNurb1, sSolutions);
    }
    
    if (TRUE)
    {
        SmBSplineCurve *pLine = my_create_line(sContext, SmPoint3d(0, 1.0, 1.5), SmPoint3d(0, 1.0, - 1.5), 0.0, 0.0, 0.0);
        //    SmBSplineCurve *pLine = my_create_line(sContext,SmPoint3d(0,-1.0,0.5),SmPoint3d(0,1.5,0),0.0,0.0,0.0);
        //    SmBSplineCurve *pLine = my_create_line(sContext,SmPoint3d(0,-1.0,0.5),SmPoint3d(0,1.0,0),0.0,0.0,0.0);
        //    SmBSplineCurve *pLine = my_create_line(sContext,SmPoint3d(0,-1.0,0.75),SmPoint3d(0,1.0,-0.5),0.0,0.0,0.0);
        SmObjDelete sCU1(pLine);
        //    SmBSplineCurve *pCir = my_create_circle(sContext,0.5,SmPoint3d(0.0,0.0,0.0),SM_CO_QUADRATIC,0,1,1);
        SmBSplineCurve *pCir = my_create_circle(sContext, 0.5, SmPoint3d(0.0, 0.5, 0.0), SM_CO_QUADRATIC, 0, 1, 1);
        SmObjDelete sCU2(pCir);
        
        SmVector3d sDirNorm[3];
        //    sDirNorm[1] = SmVector3d(0.5,0,1);
        sDirNorm[1] = SmVector3d(0, 0, 1);
        //    sDirNorm[1] = SmVector3d(1,0,0);
        sDirNorm[0] = SmVector3d(0, 0, 1);
        
        // line tangent is same direction as project vector
        SmSolutionArray sSolutions;
        SER(pCir->GlobalCurveSolve(pCir->GetNaturalInterval(),
            *pLine, pLine->GetNaturalInterval(),
            SM_SO_PROJECTED_INTERSECT, SM_ZONE_TOL_3D,
            NULL, sDirNorm,
            SM_SR_ALL,
            sSolutions));
        // currently this gets two solutions - one at an actual intersection
        // and one at the line endPoint.  The problem is that this entire line
        // projects to that one intersection point.  What's the reight answer here?
        // It could be the actual point intersection, or it could be that the
        // entire line interval projects to the intersection point - an odd kind of interval solution.
        // But the solution should not be it's current two point solution.  That
        // behavior is very arbitrary.
        my_draw_2crv_sol(pCir, pLine, sSolutions, &sDirNorm[0]);
    }
    return SM_SUCCESS;
} // end my_test_cci_projection



/***********************************************************************
PURPOSE ---

USAGE NOTES ---
***********************************************************************/
PT_EXPORT SmStatus my_test_cp_analy()
{
    //      EndParm();
    //      return SM_SUCCESS;
    
    //    SmContext sContext(pPool);
    SmContext sContext;
    SmTArray<SmPoint3d> sCombPoints, sCombVecs ;
    
    //    if (TRUE) {
    //        SmBSplineCurve *pBSC = NULL ;
    //        SER(SmBSplineCurve::ReadFromFile(sContext,
    //            "ravi_crv.dat",pBSC));
    //
    //        SmTArray<SmPoint3d> sPnts;
    //        sPnts.Add(SmPoint3d(-40.660,  18.000, 78.000  ));
    //        sPnts.Add(SmPoint3d(-25.282,  18.000, 68.250 ));
    //        sPnts.Add(SmPoint3d(-12.015,  18.000, 58.500  ));
    //        sPnts.Add(SmPoint3d(-0.895,  18.000, 48.750  ));
    //        sPnts.Add(SmPoint3d(8.188,  18.000, 39.000  ));
    //        sPnts.Add(SmPoint3d(15.225,  18.000, 29.250  ));
    //        sPnts.Add(SmPoint3d(20.297 , 18.000, 19.500 ));
    //        sPnts.Add(SmPoint3d(23.387,  18.000, 9.750  ));
    //        sPnts.Add(SmPoint3d(24.557,  18.000, 0.000  ));
    //        sPnts.Add(SmPoint3d(23.934,  18.000, -9.750  ));
    //        sPnts.Add(SmPoint3d(21.722,  18.000, -19.500  ));
    //        sPnts.Add(SmPoint3d(17.908,  18.000, -29.250  ));
    //        sPnts.Add(SmPoint3d(12.529,  18.000, -39.000  ));
    //        sPnts.Add(SmPoint3d(5.570,  18.000, -48.750  ));
    //        sPnts.Add(SmPoint3d(-2.939,  18.000, -58.500  ));
    //        sPnts.Add(SmPoint3d(-13.011,  18.000, -68.250  ));
    //
    //        sm_GraphicsLoop();
    //        smgfx_SetColor(0,0,0);
    //        pBSC->DrawWDeriv(pBSC->GetNaturalInterval(),0);
    //        
    //        SmVector3d sDropVector(1.0,0.0,0.0);
    //
    //        for (ULONG k=0; k<sPnts.GetSize(); k++) {
    //            double dD = - sPnts[k].Dot(sDropVector);
    //            
    //            SmSolutionArray sSolutions;
    //            
    //            pBSC->GlobalPropertyAnalysis(pBSC->GetNaturalInterval(),
    //                SM_CP_PLANE_INTERSECTION,
    //                &dD,
    //                &sDropVector,
    //                SM_ZONE_TOL_3D,
    //                sSolutions, &sCombPoints, &sCombVecs);        
    //            
    //            smos_WriteBuffer("\n\nInput Point - ");
    //            sPnts[k].Dump();
    //            
    //            for (ULONG i=0; i<sSolutions.GetSize(); i++) {
    //                SmSolution & rSol = sSolutions[i];
    //                double dT = rSol.m_vStart[0];
    //                smgfx_SetColor(1,0,0);
    //                pBSC->DrawAt(dT,0);
    //                smgfx_SetColor(0,0,1);
    //                SmPoint3d sPnt;
    //                pBSC->EvaluatePoint(dT,sPnt);
    //                sm_GraphicsLoop();
    //                sPnts[k].Draw();
    //                sDropVector.Draw(&sPnts[k]);
    //                smos_WriteBuffer("\n    Dropped Point - ");
    //                sPnt.Dump();
    //            }
    //            
    //        }
    //        return SM_SUCCESS;
    //
    //    }
    
    if (TRUE)
    {
        SmBSplineCurve *pLine = my_create_circle(sContext, 0.5, SmPoint3d(2.0, 0.2, 0.0), SM_CO_QUADRATIC, 0, 1, 1);
        SmObjDelete sCU1(pLine);
        SmBSplineCurve *pCir = my_create_circle(sContext, 0.5, SmPoint3d(0.0, 0.0, 0.0), SM_CO_QUADRATIC, 0, 1, 1);
        SmObjDelete sCU2(pCir);
        
        SmAxis2Placement sRot;
        sRot.RotateAboutAxis(SM_DEG2RAD(20.0), SmVector3d(1, 0, 0));
        SER(pCir->Transform(sRot));
#ifdef SM_GFX_CODE
        if (smGet_DoGraphics()) {
            smgfx_SetColor(0, 0, 0);
            pLine->DrawWDeriv(pLine->GetNaturalInterval());
            smgfx_SetColor(0, 0, 1);
            pCir->DrawWDeriv(pCir->GetNaturalInterval());
        }
#endif
        
        
        SmVector3d sDirNorm[3];
        sDirNorm[0] = SmVector3d(1.0, 0.0, 0.0);
        sDirNorm[0].x = 1.0;
        sDirNorm[0].y = 0.0;
        sDirNorm[0].z = 0.0;
        
        SmSolutionArray sSolutions;
        SER(pCir->GlobalCurveSolve(pCir->GetNaturalInterval(),
            *pLine, pLine->GetNaturalInterval(),
            SM_SO_3D_SIGNED_DIRECTED_MINIMIZE, SM_ZONE_TOL_3D,
            NULL, sDirNorm,
            SM_SR_ALL,
            sSolutions));
        
        SM_ASSERT(sSolutions.GetSize() == 2);
        my_draw_2crv_sol(pCir, pLine, sSolutions);
    }
    
    if (TRUE)
    {
        SmBSplineCurve *pLine = my_create_circle(sContext, 0.5, SmPoint3d(0.0, 2.6, 0.0), SM_CO_QUADRATIC, 0, 1, 1);
        SmObjDelete sCU1(pLine);
        SmBSplineCurve *pCir = my_create_circle(sContext, 0.5, SmPoint3d(2.0, 2.4, 0.0), SM_CO_QUADRATIC, 0, 1, 1);
        SmObjDelete sCU2(pCir);
        
        SmVector3d sDirNorm[3];
        sDirNorm[0] = SmVector3d(1, 0, 0);
        sDirNorm[0].x = 1.0;
        sDirNorm[0].y = 0.0;
        sDirNorm[0].z = 0.0;
        
        
        SmSolutionArray sSolutions;
        SER(pCir->GlobalCurveSolve(pCir->GetNaturalInterval(),
            *pLine, pLine->GetNaturalInterval(),
            SM_SO_3D_SIGNED_DIRECTED_MINIMIZE, SM_ZONE_TOL_3D,
            NULL, sDirNorm,
            SM_SR_ALL,
            sSolutions));
        
        SM_ASSERT(sSolutions.GetSize() == 1);
        my_draw_2crv_sol(pCir, pLine, sSolutions);
    }
    
    if (TRUE)
    {
        SmBSplineCurve *pLine = my_create_circle(sContext, 0.5, SmPoint3d(2.0, 0.6, 0.0), SM_CO_QUADRATIC, 0, 1, 1);
        SmObjDelete sCU1(pLine);
        SmBSplineCurve *pCir = my_create_circle(sContext, 0.5, SmPoint3d(0.0, 0.4, 0.0), SM_CO_QUADRATIC, 0, 1, 1);
        SmObjDelete sCU2(pCir);
        
        SmVector3d sDirNorm[3];
        sDirNorm[0] = SmVector3d(1, 0, 0);
        sDirNorm[0].x = 1.0;
        sDirNorm[0].y = 0.0;
        sDirNorm[0].z = 0.0;
        
        
        SmSolutionArray sSolutions;
        SER(pCir->GlobalCurveSolve(pCir->GetNaturalInterval(),
            *pLine, pLine->GetNaturalInterval(),
            SM_SO_3D_SIGNED_DIRECTED_MINIMIZE, SM_ZONE_TOL_3D,
            NULL, sDirNorm,
            SM_SR_ALL,
            sSolutions));
        
        SM_ASSERT(sSolutions.GetSize() == 1);
        my_draw_2crv_sol(pCir, pLine, sSolutions);
    }
    
    if (FALSE)
    {
        SmTArray < SmBSplineCurve*> arrCurves;
        SmTArray < SmCompositeCurve*> arrCompositeCurves;
        SmBSplineCurve * pCurve = NULL;
        
        SER(SmBSplineCurve::CreateLineSegment(sContext,
            3, SmPoint3d(1, 5, 0), SmPoint3d(1, 8, 0), pCurve));
        arrCurves.Add(pCurve);
        
        SER(SmBSplineCurve::CreateLineSegment(sContext,
            3, SmPoint3d(4.1, 7.9, 0), SmPoint3d(4.1, 5.1, 0), pCurve));
        arrCurves.Add(pCurve);
        
        SER(SmBSplineCurve::CreateLineSegment(sContext,
            3, SmPoint3d(1, 8, 0), SmPoint3d(4, 8, 0), pCurve));
        arrCurves.Add(pCurve);
        
        SER(SmBSplineCurve::CreateLineSegment(sContext,
            3, SmPoint3d(4, 5, 0), SmPoint3d(1, 5, 0), pCurve));
        arrCurves.Add(pCurve);
        
        SmCompositeCurve::BuildCompositesFromCurves(sContext,
            arrCurves,
            FALSE, // SmBoolean bMakeCurvesHomogeneous,
            0.0, // double dThisApproxTol3d,
            0.00001, // double dSamePointTolerance,
            0.0, // double dDistanceToAverage,
            0.0, // double dDistanceToExtendTrim,
            0.0, // double dDistanceToCreateLine,
            0.0, // double dDistanceToCreateBlend,
            arrCompositeCurves);
    }
    return SM_SUCCESS;
    
#if 0 // unreachable code
    if (TRUE)
    {
        //    SmBSplineCurve *pLine = my_create_circle(sContext,0.5,SmPoint3d(0.0,-1.2,1.0),SM_CO_QUADRATIC,0,1,1);
        //    SmObjDelete sCU1(pLine);
        SmBSplineCurve *pCir = my_create_circle(sContext, 0.5, SmPoint3d(-2, 1, 0.5), SM_CO_QUADRATIC, 0, 1, 1);
        SmObjDelete sCU2(pCir);
        
        SmVector3d sDirNorm[3];
        sDirNorm[0] = SmVector3d(0, 0, 0);
        sDirNorm[1] = SmVector3d(0, 0, 1);
        sDirNorm[2] = SmVector3d(1, 0, 0);
        
        SmSolutionArray sSolutions;
        SER(pCir->GlobalPropertyAnalysis(pCir->GetNaturalInterval(),
            SM_CP_MINIMIZE_CCW_ANGLE, NULL, sDirNorm, SM_ZONE_TOL_3D,
            sSolutions, &sCombPoints, &sCombVecs));
        
        SM_ASSERT(sSolutions.GetSize() == 1);
        my_draw_crv_pnt_sol(pCir, &sDirNorm[0], sSolutions, &sCombPoints, &sCombVecs);
        
        sDirNorm[0] = SmVector3d(0, 1.4, 0);
        SER(pCir->GlobalPropertyAnalysis(pCir->GetNaturalInterval(),
            SM_CP_MINIMIZE_CCW_ANGLE, NULL, sDirNorm, SM_ZONE_TOL_3D,
            sSolutions, &sCombPoints, &sCombVecs));
        
        SM_ASSERT(sSolutions.GetSize() == 1);
        my_draw_crv_pnt_sol(pCir, &sDirNorm[0], sSolutions, &sCombPoints, &sCombVecs);
        
        sDirNorm[0] = SmVector3d(0, 3.0, 0);
        SER(pCir->GlobalPropertyAnalysis(pCir->GetNaturalInterval(),
            SM_CP_MINIMIZE_CCW_ANGLE, NULL, sDirNorm, SM_ZONE_TOL_3D,
            sSolutions, &sCombPoints, &sCombVecs));
        
        SM_ASSERT(sSolutions.GetSize() == 1);
        my_draw_crv_pnt_sol(pCir, &sDirNorm[0], sSolutions, &sCombPoints, &sCombVecs);
        
        sDirNorm[0] = SmVector3d(-3, 3.0, 0);
        SER(pCir->GlobalPropertyAnalysis(pCir->GetNaturalInterval(),
            SM_CP_MINIMIZE_CCW_ANGLE, NULL, sDirNorm, SM_ZONE_TOL_3D,
            sSolutions, &sCombPoints, &sCombVecs));
        
        SM_ASSERT(sSolutions.GetSize() == 1);
        my_draw_crv_pnt_sol(pCir, &sDirNorm[0], sSolutions, &sCombPoints, &sCombVecs);
        
        sDirNorm[0] = SmVector3d(-3, 1.2, 0);
        SER(pCir->GlobalPropertyAnalysis(pCir->GetNaturalInterval(),
            SM_CP_MINIMIZE_CCW_ANGLE, NULL, sDirNorm, SM_ZONE_TOL_3D,
            sSolutions, &sCombPoints, &sCombVecs));
        
        SM_ASSERT(sSolutions.GetSize() == 1);
        my_draw_crv_pnt_sol(pCir, &sDirNorm[0], sSolutions, &sCombPoints, &sCombVecs);
        
        sDirNorm[0] = SmVector3d(-3, 0, 0);
        SER(pCir->GlobalPropertyAnalysis(pCir->GetNaturalInterval(),
            SM_CP_MINIMIZE_CCW_ANGLE, NULL, sDirNorm, SM_ZONE_TOL_3D,
            sSolutions, &sCombPoints, &sCombVecs));
        
        SM_ASSERT(sSolutions.GetSize() == 1);
        my_draw_crv_pnt_sol(pCir, &sDirNorm[0], sSolutions, &sCombPoints, &sCombVecs);
    }
    
    
    if (TRUE)
    {
        //    SmBSplineCurve *pLine = my_create_circle(sContext,0.5,SmPoint3d(0.0,-1.2,1.0),SM_CO_QUADRATIC,0,1,1);
        //    SmObjDelete sCU1(pLine);
        SmBSplineCurve *pCir = my_create_circle(sContext, 0.5, SmPoint3d(-2, 1, 0.5), SM_CO_QUADRATIC, 0, 1, 1);
        SmObjDelete sCU2(pCir);
        
        SmVector3d sDirNorm[3];
        sDirNorm[0] = SmVector3d(0, 0, 0);
        sDirNorm[1] = SmVector3d(0, 0, 1);
        sDirNorm[2] = SmVector3d(1, 0, 0);
        
        SmSolutionArray sSolutions;
        SER(pCir->GlobalPropertyAnalysis(pCir->GetNaturalInterval(),
            SM_CP_MINIMIZE_DIRECTED_ANGLE, NULL, sDirNorm, SM_ZONE_TOL_3D,
            sSolutions, &sCombPoints, &sCombVecs));
        
        SM_ASSERT(sSolutions.GetSize() == 1);
        my_draw_crv_pnt_sol(pCir, &sDirNorm[0], sSolutions, &sCombPoints, &sCombVecs);
        
        sDirNorm[0] = SmVector3d(0, 1.4, 0);
        SER(pCir->GlobalPropertyAnalysis(pCir->GetNaturalInterval(),
            SM_CP_MINIMIZE_DIRECTED_ANGLE, NULL, sDirNorm, SM_ZONE_TOL_3D,
            sSolutions, &sCombPoints, &sCombVecs));
        
        SM_ASSERT(sSolutions.GetSize() == 1);
        my_draw_crv_pnt_sol(pCir, &sDirNorm[0], sSolutions, &sCombPoints, &sCombVecs);
        
        sDirNorm[0] = SmVector3d(0, 3.0, 0);
        SER(pCir->GlobalPropertyAnalysis(pCir->GetNaturalInterval(),
            SM_CP_MINIMIZE_DIRECTED_ANGLE, NULL, sDirNorm, SM_ZONE_TOL_3D,
            sSolutions, &sCombPoints, &sCombVecs));
        
        SM_ASSERT(sSolutions.GetSize() == 1);
        my_draw_crv_pnt_sol(pCir, &sDirNorm[0], sSolutions, &sCombPoints, &sCombVecs);
        
        sDirNorm[0] = SmVector3d(-3, 3.0, 0);
        SER(pCir->GlobalPropertyAnalysis(pCir->GetNaturalInterval(),
            SM_CP_MINIMIZE_DIRECTED_ANGLE, NULL, sDirNorm, SM_ZONE_TOL_3D,
            sSolutions, &sCombPoints, &sCombVecs));
        
        SM_ASSERT(sSolutions.GetSize() == 1);
        my_draw_crv_pnt_sol(pCir, &sDirNorm[0], sSolutions, &sCombPoints, &sCombVecs);
        
        sDirNorm[0] = SmVector3d(-3, 1.2, 0);
        SER(pCir->GlobalPropertyAnalysis(pCir->GetNaturalInterval(),
            SM_CP_MINIMIZE_DIRECTED_ANGLE, NULL, sDirNorm, SM_ZONE_TOL_3D,
            sSolutions, &sCombPoints, &sCombVecs));
        
        SM_ASSERT(sSolutions.GetSize() == 1);
        my_draw_crv_pnt_sol(pCir, &sDirNorm[0], sSolutions, &sCombPoints, &sCombVecs);
        
        sDirNorm[0] = SmVector3d(-3, 0, 0);
        SER(pCir->GlobalPropertyAnalysis(pCir->GetNaturalInterval(),
            SM_CP_MINIMIZE_DIRECTED_ANGLE, NULL, sDirNorm, SM_ZONE_TOL_3D,
            sSolutions, &sCombPoints, &sCombVecs));
        
        SM_ASSERT(sSolutions.GetSize() == 1);
        my_draw_crv_pnt_sol(pCir, &sDirNorm[0], sSolutions, &sCombPoints, &sCombVecs);
    }
    
    
    if (TRUE)
    {
        SmBSplineCurve *pCir = my_create_line(sContext, SmPoint3d(-2, 0.3, 0.2), SmPoint3d(-2, 2.0, 0), 0.0, 0.0, 0.0);
        SmObjDelete sCU1(pCir);
        
        //    SmBSplineCurve *pCir = my_create_circle(sContext,0.5,SmPoint3d(1,1,0.5),SM_CO_QUADRATIC,0,1,1);
        //    SmObjDelete sCU2(pCir);
        
        SmVector3d sDirNorm[3];
        sDirNorm[0] = SmVector3d(0, 0, 0);
        sDirNorm[1] = SmVector3d(0, 0, 1);
        sDirNorm[2] = SmVector3d(1, 0, 0);
        
        SmSolutionArray sSolutions;
        SER(pCir->GlobalPropertyAnalysis(pCir->GetNaturalInterval(),
            SM_CP_MINIMIZE_DIRECTED_ANGLE, NULL, sDirNorm, SM_ZONE_TOL_3D,
            sSolutions, &sCombPoints, &sCombVecs));
        
        SM_ASSERT(sSolutions.GetSize() == 1);
        my_draw_crv_pnt_sol(pCir, &sDirNorm[0], sSolutions, &sCombPoints, &sCombVecs);
        
        sDirNorm[0] = SmVector3d(0, 1.4, 0);
        SER(pCir->GlobalPropertyAnalysis(pCir->GetNaturalInterval(),
            SM_CP_MINIMIZE_DIRECTED_ANGLE, NULL, sDirNorm, SM_ZONE_TOL_3D,
            sSolutions, &sCombPoints, &sCombVecs));
        
        SM_ASSERT(sSolutions.GetSize() == 1);
        my_draw_crv_pnt_sol(pCir, &sDirNorm[0], sSolutions, &sCombPoints, &sCombVecs);
        
        sDirNorm[0] = SmVector3d(0, 3.0, 0);
        SER(pCir->GlobalPropertyAnalysis(pCir->GetNaturalInterval(),
            SM_CP_MINIMIZE_DIRECTED_ANGLE, NULL, sDirNorm, SM_ZONE_TOL_3D,
            sSolutions, &sCombPoints, &sCombVecs));
        
        SM_ASSERT(sSolutions.GetSize() == 1);
        my_draw_crv_pnt_sol(pCir, &sDirNorm[0], sSolutions, &sCombPoints, &sCombVecs);
        
        sDirNorm[0] = SmVector3d(-3, 3.0, 0);
        SER(pCir->GlobalPropertyAnalysis(pCir->GetNaturalInterval(),
            SM_CP_MINIMIZE_DIRECTED_ANGLE, NULL, sDirNorm, SM_ZONE_TOL_3D,
            sSolutions, &sCombPoints, &sCombVecs));
        
        SM_ASSERT(sSolutions.GetSize() == 1);
        my_draw_crv_pnt_sol(pCir, &sDirNorm[0], sSolutions, &sCombPoints, &sCombVecs);
        
        sDirNorm[0] = SmVector3d(-3, 1.2, 0);
        SER(pCir->GlobalPropertyAnalysis(pCir->GetNaturalInterval(),
            SM_CP_MINIMIZE_DIRECTED_ANGLE, NULL, sDirNorm, SM_ZONE_TOL_3D,
            sSolutions, &sCombPoints, &sCombVecs));
        
        SM_ASSERT(sSolutions.GetSize() == 1);
        my_draw_crv_pnt_sol(pCir, &sDirNorm[0], sSolutions, &sCombPoints, &sCombVecs);
        
        sDirNorm[0] = SmVector3d(-3, 0, 0);
        SER(pCir->GlobalPropertyAnalysis(pCir->GetNaturalInterval(),
            SM_CP_MINIMIZE_DIRECTED_ANGLE, NULL, sDirNorm, SM_ZONE_TOL_3D,
            sSolutions, &sCombPoints, &sCombVecs));
        
        SM_ASSERT(sSolutions.GetSize() == 1);
        my_draw_crv_pnt_sol(pCir, &sDirNorm[0], sSolutions, &sCombPoints, &sCombVecs);
    }
    
    
    
    if (TRUE)
    {
        //    SmBSplineCurve *pLine = my_create_circle(sContext,0.5,SmPoint3d(0.0,-1.2,1.0),SM_CO_QUADRATIC,0,1,1);
        //    SmObjDelete sCU1(pLine);
        SmBSplineCurve *pCir = my_create_circle(sContext, 0.5, SmPoint3d(-1, 1, 0.5), SM_CO_QUADRATIC, 0, 1, 1);
        SmObjDelete sCU2(pCir);
        
        SmVector3d sDirNorm[3];
        sDirNorm[0] = SmVector3d(0, 0, 0);
        sDirNorm[1] = SmVector3d(0, 0, 1);
        sDirNorm[2] = SmVector3d(1, 0, 0);
        
        SmSolutionArray sSolutions;
        SER(pCir->GlobalPropertyAnalysis(pCir->GetNaturalInterval(),
            SM_CP_MINIMIZE_DIRECTED_ANGLE, NULL, sDirNorm, SM_ZONE_TOL_3D,
            sSolutions, &sCombPoints, &sCombVecs));
        
        SM_ASSERT(sSolutions.GetSize() == 1);
        my_draw_crv_pnt_sol(pCir, &sDirNorm[0], sSolutions, &sCombPoints, &sCombVecs);
    }
    
    if (TRUE)
    {
        //    SmBSplineCurve *pLine = my_create_circle(sContext,0.5,SmPoint3d(0.0,-1.2,1.0),SM_CO_QUADRATIC,0,1,1);
        //    SmObjDelete sCU1(pLine);
        SmBSplineCurve *pCir = my_create_circle(sContext, 0.5, SmPoint3d(-1, 1, 0.5), SM_CO_QUADRATIC, 0, 1, 1);
        SmObjDelete sCU2(pCir);
        
        SmVector3d sDirNorm[3];
        sDirNorm[0] = SmVector3d(0, 0, 0);
        sDirNorm[1] = SmVector3d(0, 0, 1);
        sDirNorm[2] = SmVector3d(1, 0, 0);
        
        SmSolutionArray sSolutions;
        SER(pCir->GlobalPropertyAnalysis(pCir->GetNaturalInterval(),
            SM_CP_MINIMIZE_DIRECTED_ANGLE, NULL, sDirNorm, SM_ZONE_TOL_3D,
            sSolutions, &sCombPoints, &sCombVecs));
        
        SM_ASSERT(sSolutions.GetSize() == 1);
        my_draw_crv_pnt_sol(pCir, &sDirNorm[0], sSolutions, &sCombPoints, &sCombVecs);
    }
    
    if (TRUE)
    {
        //    SmBSplineCurve *pLine = my_create_circle(sContext,0.5,SmPoint3d(0.0,-1.2,1.0),SM_CO_QUADRATIC,0,1,1);
        //    SmObjDelete sCU1(pLine);
        SmBSplineCurve *pCir = my_create_circle(sContext, 0.5, SmPoint3d(-1, 1, 0.5), SM_CO_QUADRATIC, 0, 1, 1);
        SmObjDelete sCU2(pCir);
        
        SmVector3d sDirNorm[3];
        sDirNorm[0] = SmVector3d(0, 0, 0);
        sDirNorm[1] = SmVector3d(0, 0, 1);
        sDirNorm[2] = SmVector3d(1, 0, 0);
        
        SmSolutionArray sSolutions;
        SER(pCir->GlobalPropertyAnalysis(pCir->GetNaturalInterval(),
            SM_CP_MINIMIZE_DIRECTED_ANGLE, NULL, sDirNorm, SM_ZONE_TOL_3D,
            sSolutions, &sCombPoints, &sCombVecs));
        
        SM_ASSERT(sSolutions.GetSize() == 1);
        my_draw_crv_pnt_sol(pCir, &sDirNorm[0], sSolutions, &sCombPoints, &sCombVecs);
    }
    
    
    if (TRUE)
    {
        //    SmBSplineCurve *pLine = my_create_circle(sContext,0.5,SmPoint3d(0.0,-1.2,1.0),SM_CO_QUADRATIC,0,1,1);
        //    SmObjDelete sCU1(pLine);
        SmBSplineCurve *pCir = my_create_circle(sContext, 0.5, SmPoint3d(0, 0, 0), SM_CO_QUADRATIC, 0, 1, 1);
        SmObjDelete sCU2(pCir);
        SmAxis2Placement sA2P;
        SmVector3d sX(1, 0.2, 0.9);
        sX.Unitize();
        SmVector3d sY(-0.2, 1, - 0.1);
        sY = sX * sY * sX;
        sY.Unitize();
        sA2P.SetCanonical(SmVector3d(0, 0, 0), sX, sY);
        pCir->Transform(sA2P);
#ifdef SM_GFX_CODE
        smgfx_SetColor(1, 0, 0);
        if( smGet_DoGraphics()){
        pCir->DrawWDeriv(pCir->GetNaturalInterval(), 0);
          }
#endif
        
        SmVector3d sDirNorm[3];
        sDirNorm[0] = SmVector3d(0, 0, 1);
        sDirNorm[1] = SmVector3d(1, 0, 0);
        
        SmSolutionArray sSolutions;
        SER(pCir->GlobalPropertyAnalysis(pCir->GetNaturalInterval(),
            SM_CP_PERPENDICULAR_TO_VECTOR, NULL, sDirNorm, SM_ZONE_TOL_3D,
            sSolutions, &sCombPoints, &sCombVecs));
        
        SM_ASSERT(sSolutions.GetSize() == 2);
        my_draw_crv_pnt_sol(pCir, NULL, sSolutions, &sCombPoints, &sCombVecs);
        
        
        for (ULONG i = 0; i < sSolutions.GetSize(); i++)
        {
            SmSolution & rSol = sSolutions[i];
#ifdef SM_GFX_CODE
            if (smGet_DoGraphics())
            {
                smgfx_SetColor(1.0, 0.0, 0);
                SmPoint3d sP1, sP2;
                pCir->EvaluatePoint(rSol.m_vStart[0], sP2);
                sP2.Draw();
            }
#endif
        }
        
    
    }
    
    
    
    if (TRUE)
    {
        //    SmBSplineCurve *pLine = my_create_circle(sContext,0.5,SmPoint3d(0.0,-1.2,1.0),SM_CO_QUADRATIC,0,1,1);
        //    SmObjDelete sCU1(pLine);
        SmBSplineCurve *pCir = my_create_circle(sContext, 0.5, SmPoint3d(0, 0, 0), SM_CO_QUADRATIC, 0, 1, 1);
        SmObjDelete sCU2(pCir);
        SmAxis2Placement sA2P;
        SmVector3d sX(1, 0.2, 0.9);
        sX.Unitize();
        SmVector3d sY(-0.2, 1, - 0.1);
        sY = sX * sY * sX;
        sY.Unitize();
        sA2P.SetCanonical(SmVector3d(0, 0, 0), sX, sY);
        pCir->Transform(sA2P);
#ifdef SM_GFX_CODE
        if (smGet_DoGraphics()){
        smgfx_SetColor(1, 0, 0);
        pCir->DrawWDeriv(pCir->GetNaturalInterval(), 0);
          }
#endif
        
        SmVector3d sDirNorm[3];
        sDirNorm[0] = SmVector3d(0, 0, 1);
        sDirNorm[1] = SmVector3d(2, - 2, 0);
        
        SmSolutionArray sSolutions;
        SER(pCir->GlobalPropertyAnalysis(pCir->GetNaturalInterval(),
            SM_CP_PROJECTED_POINT_MINIMIZE, NULL, sDirNorm, SM_ZONE_TOL_3D,
            sSolutions, &sCombPoints, &sCombVecs));
        
        SM_ASSERT(sSolutions.GetSize() == 1);
        my_draw_crv_pnt_sol(pCir, NULL, sSolutions, &sCombPoints, &sCombVecs);
    }
    
    if (TRUE)
    {
        //    SmBSplineCurve *pLine = my_create_circle(sContext,0.5,SmPoint3d(0.0,-1.2,1.0),SM_CO_QUADRATIC,0,1,1);
        //    SmObjDelete sCU1(pLine);
        SmBSplineCurve *pCir = my_create_circle(sContext, 0.5, SmPoint3d(0, 0, 0), SM_CO_QUADRATIC, 0, 1, 1);
        SmObjDelete sCU2(pCir);
        SmAxis2Placement sA2P;
        SmVector3d sX(1, 0.2, 0.9);
        sX.Unitize();
        SmVector3d sY(-0.2, 1, - 0.1);
        sY = sX * sY * sX;
        sY.Unitize();
        sA2P.SetCanonical(SmVector3d(0, 0, 0), sX, sY);
        pCir->Transform(sA2P);
#ifdef SM_GFX_CODE
        if(smGet_DoGraphics()){
            smgfx_SetColor(1, 0, 0);
            pCir->DrawWDeriv(pCir->GetNaturalInterval(), 0);
          }
#endif
        
        
        SmVector3d sDirNorm[3];
        sDirNorm[0] = SmVector3d(0, 0, 1);
        sDirNorm[1] = SmVector3d(2, 2, 0.5);
        
        SmSolutionArray sSolutions;
        SER(pCir->GlobalPropertyAnalysis(pCir->GetNaturalInterval(),
            SM_CP_PROJECTED_POINT_MAXIMIZE, NULL, sDirNorm, SM_ZONE_TOL_3D,
            sSolutions, &sCombPoints, &sCombVecs));
        
        SM_ASSERT(sSolutions.GetSize() == 1);
        my_draw_crv_pnt_sol(pCir, NULL, sSolutions, &sCombPoints, &sCombVecs);
    }
    
    
    if (TRUE)
    {
        SmBSplineCurve *pCir = my_create_line(sContext, SmPoint3d(-2, 0.3, 0.2), SmPoint3d(-2, 2.0, 0), 0.0, 0.0, 0.0);
        SmObjDelete sCU1(pCir);
        
        //    SmBSplineCurve *pCir = my_create_circle(sContext,0.5,SmPoint3d(1,1,0.5),SM_CO_QUADRATIC,0,1,1);
        //    SmObjDelete sCU2(pCir);
        
        SmVector3d sDirNorm[3];
        sDirNorm[0] = SmVector3d(0, 0, 0);
        sDirNorm[1] = SmVector3d(0, 0, 1);
        sDirNorm[2] = SmVector3d(0, 1, 0);
        
        SmSolutionArray sSolutions;
        SER(pCir->GlobalPropertyAnalysis(pCir->GetNaturalInterval(),
            SM_CP_MINIMIZE_ANGLE_TO_PLANE, NULL, sDirNorm, SM_ZONE_TOL_3D,
            sSolutions, &sCombPoints, &sCombVecs));
        
        SM_ASSERT(sSolutions.GetSize() == 1);
        my_draw_crv_pnt_sol(pCir, NULL, sSolutions, &sCombPoints, &sCombVecs);
    }
    
    if (TRUE)
    {
        //    SmBSplineCurve *pLine = my_create_circle(sContext,0.5,SmPoint3d(0.0,-1.2,1.0),SM_CO_QUADRATIC,0,1,1);
        //    SmObjDelete sCU1(pLine);
        SmBSplineCurve *pCir = my_create_circle(sContext, 0.5, SmPoint3d(-1, - 1, 0.5), SM_CO_QUADRATIC, 0, 1, 1);
        SmObjDelete sCU2(pCir);
        
        SmVector3d sDirNorm[3];
        sDirNorm[0] = SmVector3d(0, 0, 0);
        sDirNorm[1] = SmVector3d(0, 0, 1);
        sDirNorm[2] = SmVector3d(0, 1, 0);
        
        SmSolutionArray sSolutions;
        SER(pCir->GlobalPropertyAnalysis(pCir->GetNaturalInterval(),
            SM_CP_MINIMIZE_ANGLE_TO_PLANE, NULL, sDirNorm, SM_ZONE_TOL_3D,
            sSolutions, &sCombPoints, &sCombVecs));
        
        SM_ASSERT(sSolutions.GetSize() == 1);
        my_draw_crv_pnt_sol(pCir, NULL, sSolutions, &sCombPoints, &sCombVecs);
    }
    
    if (TRUE)
    {
        //    SmBSplineCurve *pLine = my_create_circle(sContext,0.5,SmPoint3d(0.0,-1.2,1.0),SM_CO_QUADRATIC,0,1,1);
        //    SmObjDelete sCU1(pLine);
        SmBSplineCurve *pCir = my_create_circle(sContext, 0.5, SmPoint3d(-1, 1, 0.5), SM_CO_QUADRATIC, 0, 1, 1);
        SmObjDelete sCU2(pCir);
        
        SmVector3d sDirNorm[3];
        sDirNorm[0] = SmVector3d(0, 0, 0);
        sDirNorm[1] = SmVector3d(0, 0, 1);
        sDirNorm[2] = SmVector3d(0, 1, 0);
        
        SmSolutionArray sSolutions;
        SER(pCir->GlobalPropertyAnalysis(pCir->GetNaturalInterval(),
            SM_CP_MINIMIZE_ANGLE_TO_PLANE, NULL, sDirNorm, SM_ZONE_TOL_3D,
            sSolutions, &sCombPoints, &sCombVecs));
        
        SM_ASSERT(sSolutions.GetSize() == 1);
        my_draw_crv_pnt_sol(pCir, NULL, sSolutions, &sCombPoints, &sCombVecs);
    }
    
    if (TRUE)
    {
        //    SmBSplineCurve *pLine = my_create_circle(sContext,0.5,SmPoint3d(0.0,-1.2,1.0),SM_CO_QUADRATIC,0,1,1);
        //    SmObjDelete sCU1(pLine);
        SmBSplineCurve *pCir = my_create_circle(sContext, 0.5, SmPoint3d(-1, 1, 0.5), SM_CO_QUADRATIC, 0, 1, 1);
        SmObjDelete sCU2(pCir);
        
        SmVector3d sDirNorm[3];
        sDirNorm[0] = SmVector3d(0, 0, 0);
        sDirNorm[1] = SmVector3d(0, 0, 1);
        sDirNorm[2] = SmVector3d(0, 1, 0);
        
        SmSolutionArray sSolutions;
        SER(pCir->GlobalPropertyAnalysis(pCir->GetNaturalInterval(),
            SM_CP_MINIMIZE_ANGLE_TO_PLANE, NULL, sDirNorm, SM_ZONE_TOL_3D,
            sSolutions, &sCombPoints, &sCombVecs));
        
        SM_ASSERT(sSolutions.GetSize() == 1);
        my_draw_crv_pnt_sol(pCir, NULL, sSolutions, &sCombPoints, &sCombVecs);
    }
    
    if (TRUE)
    {
        //    SmBSplineCurve *pLine = my_create_circle(sContext,0.5,SmPoint3d(0.0,-1.2,1.0),SM_CO_QUADRATIC,0,1,1);
        //    SmObjDelete sCU1(pLine);
        SmBSplineCurve *pCir = my_create_circle(sContext, 0.5, SmPoint3d(-1, 1, 0.5), SM_CO_QUADRATIC, 0, 1, 1);
        SmObjDelete sCU2(pCir);
        
        SmVector3d sDirNorm[3];
        sDirNorm[0] = SmVector3d(0, 0, 0);
        sDirNorm[1] = SmVector3d(0, 0, 1);
        sDirNorm[2] = SmVector3d(0, 1, 0);
        
        SmSolutionArray sSolutions;
        SER(pCir->GlobalPropertyAnalysis(pCir->GetNaturalInterval(),
            SM_CP_MINIMIZE_ANGLE_TO_PLANE, NULL, sDirNorm, SM_ZONE_TOL_3D,
            sSolutions, &sCombPoints, &sCombVecs));
        
        SM_ASSERT(sSolutions.GetSize() == 1);
        my_draw_crv_pnt_sol(pCir, NULL, sSolutions, &sCombPoints, &sCombVecs);
    }
    
    
    
    if (TRUE)
    {
        SmConic sConic(SM_CT_HYPERBOLA, 2.0, 1.0);
        SmConic sConic2(SM_CT_HYPERBOLA, 1, 2);
        sConic.Transform2D(10.0*SM_PI/180.0, SmPoint2d(-0.4, - 0.3));
        sConic.Transform2D(-85.0*SM_PI/180.0, SmPoint2d(0, 0));
#ifdef SM_GFX_CODE
        if( smGet_DoGraphics()){
        sConic.DrawWDeriv(sConic.GetNaturalInterval(), 0);
        sConic2.DrawWDeriv(sConic2.GetNaturalInterval(), 0);
          }
#endif
        SmSolutionArray sSols;
        SER(sConic.IntersectConic(sConic.GetNaturalInterval(), sConic2,
            sConic2.GetNaturalInterval(), SM_ZONE_TOL_3D, sSols));
        SM_ASSERT(sSols.GetSize() == 1);
        for (ULONG j = 0; j < sSols.GetSize(); j++)
        {
            SmPoint3d sPnt;
            double dT = sSols[j].m_vStart[0];
            sConic.EvaluatePoint(dT, sPnt);
            sPnt.Dump();
#ifdef SM_GFX_CODE
        if( smGet_DoGraphics()){
            smgfx_SetColor(1.0, 0, 0);
            sPnt.Draw();
          }
#endif
            smos_WriteBuffer(_T("\n"));
        }
    }
    
    
    if (TRUE)
    {
        SmConic sConic2(SM_CT_ELLIPSE, 2.0, 1.0);
        SmConic sConic(SM_CT_HYPERBOLA, 1.0, 2.0);
        sConic2.Transform2D(30.0*SM_PI/180.0, SmPoint2d(-0.3, - 0.4));
#ifdef SM_GFX_CODE
        if( smGet_DoGraphics()){
        sConic.DrawWDeriv(sConic.GetNaturalInterval(), 0);
        sConic2.DrawWDeriv(sConic2.GetNaturalInterval(), 0);
          }
#endif // SM_GFX_CODE
        SmSolutionArray sSols;
        SER(sConic.IntersectConic(sConic.GetNaturalInterval(), sConic2,
            sConic2.GetNaturalInterval(), SM_ZONE_TOL_3D, sSols));
        SM_ASSERT(sSols.GetSize() == 2);
        for (ULONG j = 0; j < sSols.GetSize(); j++)
        {
            SmPoint3d sPnt;
            double dT = sSols[j].m_vStart[0];
            sConic.EvaluatePoint(dT, sPnt);
            sPnt.Dump();
#ifdef SM_GFX_CODE
        if( smGet_DoGraphics()){
            smgfx_SetColor(1.0, 0, 0);
            sPnt.Draw();
          }
#endif
            smos_WriteBuffer(_T("\n"));
        }
    }
    
    if (TRUE)
    {
        SmConic sConic(SM_CT_HYPERBOLA, 2.0, 1.0);
        SmConic sConic2(SM_CT_LINE, 0.5, 0.0);
        sConic.Transform2D(10.0*SM_PI/180.0, SmPoint2d(-0.4, - 0.3));
        sConic2.Transform2D(12.0*SM_PI/180.0, SmPoint2d(0.4, 0.3));
#ifdef SM_GFX_CODE
        if( smGet_DoGraphics()){
        sConic.DrawWDeriv(sConic.GetNaturalInterval(), 0);
        sConic2.DrawWDeriv(sConic2.GetNaturalInterval(), 0);
          }
#endif // SM_GFX_CODE

        SmSolutionArray sSols;
        SER(sConic.IntersectConic(sConic.GetNaturalInterval(), sConic2,
            sConic2.GetNaturalInterval(), SM_ZONE_TOL_3D, sSols));
        SM_ASSERT(sSols.GetSize() == 1);
        for (ULONG j = 0; j < sSols.GetSize(); j++)
        {
            SmPoint3d sPnt;
            double dT = sSols[j].m_vStart[0];
            sConic.EvaluatePoint(dT, sPnt);
            sPnt.Dump();
#ifdef SM_GFX_CODE
            if( smGet_DoGraphics()){
            smgfx_SetColor(1.0, 0, 0);
            sPnt.Draw();
              }
#endif
            smos_WriteBuffer(_T("\n"));
        }
    }
    
    if (TRUE)
    {
        SmConic sConic(SM_CT_LINE, 2.0, 0.0);
        SmConic sConic2(SM_CT_LINE, 0.5, 0.0);
        sConic.Transform2D(30.0*SM_PI/180.0, SmPoint2d(0.3, 0.6));
        sConic2.Transform2D(-20.0*SM_PI/180.0, SmPoint2d(-0.3, 0.2));
#ifdef SM_GFX_CODE
        if( smGet_DoGraphics()){
        sConic.DrawWDeriv(sConic.GetNaturalInterval(), 0);
        sConic2.DrawWDeriv(sConic2.GetNaturalInterval(), 0);
          }
#endif
        SmSolutionArray sSols;
        SER(sConic.IntersectConic(sConic.GetNaturalInterval(), sConic2,
            sConic2.GetNaturalInterval(), SM_ZONE_TOL_3D, sSols));
        SM_ASSERT(sSols.GetSize() == 1);
        for (ULONG j = 0; j < sSols.GetSize(); j++)
        {
            SmPoint3d sPnt;
            double dT = sSols[j].m_vStart[0];
            sConic.EvaluatePoint(dT, sPnt);
            sPnt.Dump();
#ifdef SM_GFX_CODE
            if( smGet_DoGraphics()) {
            smgfx_SetColor(1.0, 0, 0);
            sPnt.Draw();
              }
#endif
            smos_WriteBuffer(_T("\n"));
        }
    }
    
    if (TRUE)
    {
        SmConic sConic(SM_CT_LINE, 2.0, 0.0);
        SmConic sConic2(SM_CT_LINE, 0.5, 0.0);
        sConic2.Transform2D(10.0*SM_PI/180.0, SmPoint2d(0.3, 0.6));
#ifdef SM_GFX_CODE
            if( smGet_DoGraphics()) {
        sConic.DrawWDeriv(sConic.GetNaturalInterval(), 0);
        sConic2.DrawWDeriv(sConic2.GetNaturalInterval(), 0);
              }
#endif
        SmSolutionArray sSols;
        SER(sConic.IntersectConic(sConic.GetNaturalInterval(), sConic2,
            sConic2.GetNaturalInterval(), SM_ZONE_TOL_3D, sSols));
        SM_ASSERT(sSols.GetSize() == 1);
        for (ULONG j = 0; j < sSols.GetSize(); j++)
        {
            SmPoint3d sPnt;
            double dT = sSols[j].m_vStart[0];
            sConic.EvaluatePoint(dT, sPnt);
            sPnt.Dump();
#ifdef SM_GFX_CODE
            if( smGet_DoGraphics()) {
            smgfx_SetColor(1.0, 0, 0);
            sPnt.Draw();
              }
#endif
            smos_WriteBuffer(_T("\n"));
        }
    }
    
    
    if (TRUE)
    {
        SmConic sConic(SM_CT_ELLIPSE, 2.0, 1.0);
        SmConic sConic2(SM_CT_LINE, 0.5, 0.0);
        sConic.Transform2D(30.0*SM_PI/180.0, SmPoint2d(0.0, 0.0));
        sConic2.Transform2D(-15.0*SM_PI/180.0, SmPoint2d(0.0, 0.0));
#ifdef SM_GFX_CODE
            if( smGet_DoGraphics()) {
        sConic.DrawWDeriv(sConic.GetNaturalInterval(), 0);
        sConic2.DrawWDeriv(sConic2.GetNaturalInterval(), 0);
              }
#endif
        SmSolutionArray sSols;
        SER(sConic.IntersectConic(sConic.GetNaturalInterval(), sConic2,
            sConic2.GetNaturalInterval(), SM_ZONE_TOL_3D, sSols));
        SM_ASSERT(sSols.GetSize() == 2);
        for (ULONG j = 0; j < sSols.GetSize(); j++)
        {
            SmPoint3d sPnt;
            double dT = sSols[j].m_vStart[0];
            sConic.EvaluatePoint(dT, sPnt);
            sPnt.Dump();
#ifdef SM_GFX_CODE
            if( smGet_DoGraphics()) {
            smgfx_SetColor(1.0, 0, 0);
            sPnt.Draw();
              }
#endif
            smos_WriteBuffer(_T("\n"));
        }
    }
    
    
    if (TRUE)
    {
        SmConic sConic2(SM_CT_ELLIPSE, 2.0, 1.0);
        SmConic sConic(SM_CT_LINE, 0.5, 0.0);
        sConic2.Transform2D(30.0*SM_PI/180.0, SmPoint2d(0.3, 0.1));
#ifdef SM_GFX_CODE
            if( smGet_DoGraphics()) {
        sConic.DrawWDeriv(sConic.GetNaturalInterval(), 0);
        sConic2.DrawWDeriv(sConic2.GetNaturalInterval(), 0);
              }
#endif
        SmSolutionArray sSols;
        SER(sConic.IntersectConic(sConic.GetNaturalInterval(), sConic2,
            sConic2.GetNaturalInterval(), SM_ZONE_TOL_3D, sSols));
        SM_ASSERT(sSols.GetSize() == 2);
        for (ULONG j = 0; j < sSols.GetSize(); j++)
        {
            SmPoint3d sPnt;
            double dT = sSols[j].m_vStart[0];
            sConic.EvaluatePoint(dT, sPnt);
            sPnt.Dump();
#ifdef SM_GFX_CODE
            if( smGet_DoGraphics()) {
            smgfx_SetColor(1.0, 0, 0);
            sPnt.Draw();
              }
#endif
            smos_WriteBuffer(_T("\n"));
        }
    }
    
    
    if (TRUE)
    {
        SmConic sConic(SM_CT_ELLIPSE, 2.0, 1.0);
        SmConic sConic2(SM_CT_ELLIPSE, 0.5, 1.0 + SM_EFF_ZERO*10000.0);
        //    sConic.Transform2D(30.0*SM_PI/180.0,SmPoint2d(0.4,0.5));
#ifdef SM_GFX_CODE
            if( smGet_DoGraphics()) {
        sConic.DrawWDeriv(sConic.GetNaturalInterval(), 0);
        sConic2.DrawWDeriv(sConic2.GetNaturalInterval(), 0);
              }
#endif
        SmSolutionArray sSols;
        SER(sConic.IntersectConic(sConic.GetNaturalInterval(), sConic2,
            sConic2.GetNaturalInterval(), SM_ZONE_TOL_3D, sSols));
        SM_ASSERT(sSols.GetSize() == 4);
        for (ULONG j = 0; j < sSols.GetSize(); j++)
        {
            SmPoint3d sPnt;
            double dT = sSols[j].m_vStart[0];
            sConic.EvaluatePoint(dT, sPnt);
            sPnt.Dump();
#ifdef SM_GFX_CODE
            if( smGet_DoGraphics()) {
            smgfx_SetColor(1.0, 0, 0);
            sPnt.Draw();
              }
#endif
            smos_WriteBuffer(_T("\n"));
        }
    }
    
    
    if (TRUE)
    {
        SmConic sConic(SM_CT_ELLIPSE, 2.0, 1.0);
        SmConic sConic2(SM_CT_ELLIPSE, 2.0, 4.0);
        //    sConic.Transform2D(30.0*SM_PI/180.0,SmPoint2d(0.4,0.5));
#ifdef SM_GFX_CODE
            if( smGet_DoGraphics()) {
        sConic.DrawWDeriv(sConic.GetNaturalInterval(), 0);
        sConic2.DrawWDeriv(sConic2.GetNaturalInterval(), 0);
              }
#endif
        SmSolutionArray sSols;
        SER(sConic.IntersectConic(sConic.GetNaturalInterval(), sConic2,
            sConic2.GetNaturalInterval(), SM_ZONE_TOL_3D, sSols));
        SM_ASSERT(sSols.GetSize() == 2);
        for (ULONG j = 0; j < sSols.GetSize(); j++)
        {
            SmPoint3d sPnt;
            double dT = sSols[j].m_vStart[0];
            sConic.EvaluatePoint(dT, sPnt);
            sPnt.Dump();
#ifdef SM_GFX_CODE
            if( smGet_DoGraphics()) {
            smgfx_SetColor(1.0, 0, 0);
            sPnt.Draw();
              }
#endif
            smos_WriteBuffer(_T("\n"));
        }
    }
    
    if (FALSE)
    {
        // Note that there is a bug when the first conic has a transform
        // and rotation
        SmConic sConic(SM_CT_ELLIPSE, 2.0, 1.0);
        SmConic sConic2(SM_CT_ELLIPSE, 1.0, 2.0);
        sConic.Transform2D(30.0*SM_PI/180.0, SmPoint2d(0.4, 0.5));
#ifdef SM_GFX_CODE
            if( smGet_DoGraphics()) {
        sConic.DrawWDeriv(sConic.GetNaturalInterval(), 0);
        sConic2.DrawWDeriv(sConic2.GetNaturalInterval(), 0);
              }
#endif
        SmSolutionArray sSols;
        SER(sConic.IntersectConic(sConic.GetNaturalInterval(), sConic2,
            sConic2.GetNaturalInterval(), SM_ZONE_TOL_3D, sSols));
        SM_ASSERT(sSols.GetSize() == 4);
        for (ULONG j = 0; j < sSols.GetSize(); j++)
        {
            SmPoint3d sPnt;
            double dT = sSols[j].m_vStart[0];
            sConic.EvaluatePoint(dT, sPnt);
            sPnt.Dump();
#ifdef SM_GFX_CODE
            if( smGet_DoGraphics()) {
            smgfx_SetColor(1.0, 0, 0);
            sPnt.Draw();
              }
#endif
            smos_WriteBuffer(_T("\n"));
        }
    }
    
    if (TRUE)
    {
        SmConic sConic(SM_CT_ELLIPSE, 2.0, 1.0);
        SmConic sConic2(SM_CT_PARABOLA, 1.0, 2.0);
        sConic2.Transform2D(30.0*SM_PI/180.0, SmPoint2d(0.4, 0.5));
#ifdef SM_GFX_CODE
            if( smGet_DoGraphics()) {
        sConic.DrawWDeriv(sConic.GetNaturalInterval(), 0);
        sConic2.DrawWDeriv(sConic2.GetNaturalInterval(), 0);
              }
#endif
        SmSolutionArray sSols;
        SER(sConic.IntersectConic(sConic.GetNaturalInterval(), sConic2,
            sConic2.GetNaturalInterval(), SM_ZONE_TOL_3D, sSols));
        SM_ASSERT(sSols.GetSize() == 2);
        for (ULONG j = 0; j < sSols.GetSize(); j++)
        {
            SmPoint3d sPnt;
            double dT = sSols[j].m_vStart[0];
            sConic.EvaluatePoint(dT, sPnt);
            sPnt.Dump();
#ifdef SM_GFX_CODE
            if( smGet_DoGraphics()) {
            smgfx_SetColor(1.0, 0, 0);
            sPnt.Draw();
              }
#endif
            smos_WriteBuffer(_T("\n"));
        }
    }
    
    if (TRUE)
    {
        SmConic sConic(SM_CT_ELLIPSE, 2.0, 1.0);
        SmConic sConic2(SM_CT_HYPERBOLA, 1.0, 2.0);
        sConic2.Transform2D(30.0*SM_PI/180.0, SmPoint2d(-0.3, - 0.4));
#ifdef SM_GFX_CODE
            if( smGet_DoGraphics()) {
        sConic.DrawWDeriv(sConic.GetNaturalInterval(), 0);
        sConic2.DrawWDeriv(sConic2.GetNaturalInterval(), 0);
              }
#endif
        SmSolutionArray sSols;
        SER(sConic.IntersectConic(sConic.GetNaturalInterval(), sConic2,
            sConic2.GetNaturalInterval(), SM_ZONE_TOL_3D, sSols));
        SM_ASSERT(sSols.GetSize() == 2);
        for (ULONG j = 0; j < sSols.GetSize(); j++)
        {
            SmPoint3d sPnt;
            double dT = sSols[j].m_vStart[0];
            sConic.EvaluatePoint(dT, sPnt);
            sPnt.Dump();
#ifdef SM_GFX_CODE
            if( smGet_DoGraphics()) {
            smgfx_SetColor(1.0, 0, 0);
            sPnt.Draw();
              }
#endif
            smos_WriteBuffer(_T("\n"));
        }
    }
    
    if (TRUE)
    {
        SmConic sConic(SM_CT_ELLIPSE, 2.0, 1.0);
        SmConic sConic2(SM_CT_ELLIPSE, 1.0, 2.0);
        sConic2.Transform2D(30.0*SM_PI/180.0, SmPoint2d(0.3, 0.5));
#ifdef SM_GFX_CODE
            if( smGet_DoGraphics()) {
        sConic.DrawWDeriv(sConic.GetNaturalInterval(), 0);
        sConic2.DrawWDeriv(sConic2.GetNaturalInterval(), 0);
              }
#endif
        SmSolutionArray sSols;
        SER(sConic.IntersectConic(sConic.GetNaturalInterval(), sConic2,
            sConic2.GetNaturalInterval(), SM_ZONE_TOL_3D, sSols));
        SM_ASSERT(sSols.GetSize() == 4);
        for (ULONG j = 0; j < sSols.GetSize(); j++)
        {
            SmPoint3d sPnt;
            double dT = sSols[j].m_vStart[0];
            sConic.EvaluatePoint(dT, sPnt);
            sPnt.Dump();
#ifdef SM_GFX_CODE
            if( smGet_DoGraphics()) {
            smgfx_SetColor(1.0, 0, 0);
            sPnt.Draw();
              }
#endif
            smos_WriteBuffer(_T("\n"));
        }
    }
    
    if (TRUE)
    {
        SmConic sConic(SM_CT_ELLIPSE, 2.0, 1.0);
        SmConic sConic2(SM_CT_ELLIPSE, 1.0, 2.0);
        sConic2.Transform2D(30.0*SM_PI/180.0, SmPoint2d(0, 0));
#ifdef SM_GFX_CODE
            if( smGet_DoGraphics()) {
        sConic.DrawWDeriv(sConic.GetNaturalInterval(), 0);
        sConic2.DrawWDeriv(sConic2.GetNaturalInterval(), 0);
              }
#endif
        SmSolutionArray sSols;
        SER(sConic.IntersectConic(sConic.GetNaturalInterval(), sConic2,
            sConic2.GetNaturalInterval(), SM_ZONE_TOL_3D, sSols));
        SM_ASSERT(sSols.GetSize() == 4);
        for (ULONG j = 0; j < sSols.GetSize(); j++)
        {
            SmPoint3d sPnt;
            double dT = sSols[j].m_vStart[0];
            sConic.EvaluatePoint(dT, sPnt);
            sPnt.Dump();
#ifdef SM_GFX_CODE
            if( smGet_DoGraphics()) {
            smgfx_SetColor(1.0, 0, 0);
            sPnt.Draw();
              }
#endif
            smos_WriteBuffer(_T("\n"));
        }
    }
    
    if (TRUE)
    {
        SmConic sConic(SM_CT_ELLIPSE, 2.0, 1.0);
        SmConic sConic2(SM_CT_ELLIPSE, 1.0, 2.0);
#ifdef SM_GFX_CODE
            if( smGet_DoGraphics()) {
        sConic.DrawWDeriv(sConic.GetNaturalInterval(), 0);
        sConic2.DrawWDeriv(sConic2.GetNaturalInterval(), 0);
              }
#endif
        SmSolutionArray sSols;
        SER(sConic.IntersectConic(sConic.GetNaturalInterval(), sConic2,
            sConic2.GetNaturalInterval(), SM_ZONE_TOL_3D, sSols));
        SM_ASSERT(sSols.GetSize() == 4);
        for (ULONG j = 0; j < sSols.GetSize(); j++)
        {
            SmPoint3d sPnt;
            double dT = sSols[j].m_vStart[0];
            sConic.EvaluatePoint(dT, sPnt);
            sPnt.Dump();
#ifdef SM_GFX_CODE
            if( smGet_DoGraphics()) {
            smgfx_SetColor(1.0, 0, 0);
            sPnt.Draw();
              }
#endif
            smos_WriteBuffer(_T("\n"));
        }
    }
    
    if (TRUE)
    {
        SmBSplineCurve *pCir3 = my_create_circle(sContext, 0.5, SmPoint3d(0, 0, 0), SM_CO_QUADRATIC, 0, 1, 1);
        SmObjDelete sCleanUp1(pCir3);
        SmBSplineCurve *pCir4 = my_create_circle(sContext, 0.5, SmPoint3d(2, 2, 0), SM_CO_QUINTIC, 1, 0, 1);
        SmObjDelete sCleanUp2(pCir4);
        SmSolutionArray sSolutions;
        SER(pCir3->GlobalPointSolve(pCir3->GetNaturalInterval(),
            SM_SO_MINIMIZE, SmPoint3d(0, 0, 0), SM_ZONE_TOL_3D, NULL, NULL,
            SM_SR_ALL, sSolutions));
        sSolutions.Dump();
    }
    
    
    
    //    pPool->Free();
    //    delete pPool; pPool = NULL ;
    
    return SM_SUCCESS;

#endif // unreachable code

} // end my_test_cp_analy



/***********************************************************************
PURPOSE ---

USAGE NOTES ---
***********************************************************************/
PT_EXPORT SmStatus my_test_time(void)
{
    //    SmContext sContext(pPool);
    SmContext sContext;
    
    
    SmTArray < SmPoint3d> sPnts;
    sPnts.Add(SmPoint3d(0, 0.1, 0));
    sPnts.Add(SmPoint3d(0.2, 0, 0));
    sPnts.Add(SmPoint3d(0.4, 0.2, - 0.1));
    sPnts.Add(SmPoint3d(0.6, - 0.3, 0.2));
    sPnts.Add(SmPoint3d(1.0, 0.0, 0.1));
    sPnts.Add(SmPoint3d(1.1, 0.1, 0));
    sPnts.Add(SmPoint3d(1.0, 0.5, 0));
    SmBSplineCurve *pNewBSC = my_create_nurb(sContext, SmPoint3d(0, 0, 0), sPnts, 3, 1, 0.5, 0);
    SmObjDelete sCU1(pNewBSC);
    
    
    
    
    
    
    
    
    
    
    
    
    
    if (FALSE)
    { 
#ifdef SM_DEBUG_CODE
        clock_t start, finish;
        start = clock();
#endif
        for (long i = 0; i < 10000; i++)
        {
            SmPoint3d sClPt(0.5, 0.2, 1.0);
            double dGuess = 0.5;
            SmBoolean bFoundAnswer;
            SmSolution sSolution;
            SER(pNewBSC->LocalPointSolve(pNewBSC->GetNaturalInterval(),
                SM_SO_MINIMIZE, sClPt, NULL, NULL, NULL, dGuess, bFoundAnswer, sSolution));
        }
#ifdef SM_DEBUG_CODE
        finish = clock();
        sm_PrintTime(_T("10k Closest Point calls "), start, finish);
#endif
    }
    
    if (FALSE)
    {
#ifdef SM_DEBUG_CODE
        clock_t start, finish;
        start = clock();
#endif
        SmPoint3d sPnt;
        for (long i = 0; i < 10000; i++)
        {
            pNewBSC->EvaluatePoint(0.354, sPnt);
        }
#ifdef SM_DEBUG_CODE
        finish = clock();
        sm_PrintTime(_T("10k Point evaluations "), start, finish);
#endif
        
#ifdef SM_DEBUG_CODE
        start = clock();
#endif
        SmPoint3d sPV[2];
        for (ULONG ii = 0; ii < 3000; ii++)
        {
            pNewBSC->Evaluate(0.354, 1, TRUE, sPV);
        }
#ifdef SM_DEBUG_CODE
        finish = clock();
        sm_PrintTime(_T("3k Point & 1st deriv evaluations "), start, finish);
#endif
        
#ifdef SM_DEBUG_CODE
        start = clock();
#endif
        SmPoint3d sPVV[2];
        for (ULONG iii = 0; iii < 3000; iii++)
        {
            pNewBSC->Evaluate(0.354, 2, TRUE, sPVV);
        }
#ifdef SM_DEBUG_CODE
        finish = clock();
        sm_PrintTime(_T("3k Point & 1st & 2nd deriv evaluations "), start, finish);
#endif
    }
    
    
    
    
    
    
    //    pPool->Free();
    //    delete pPool; pPool = NULL ;
    
    return SM_SUCCESS;
} // end my_test_time


/***********************************************************************
PURPOSE --- Test SmCurve interface

USAGE NOTES ---
   This was written for the Java project: we need tests for all methods
   that are to be wrapped.  This is not testing the functionality of all
   the methods, only that they are being called correctly.  The list is:

   InterpolatePoints
   CreateInterpolatingCurve
   CreateCircleSegment      <- requires SmAxis2Placement
   JoinWith              1 def arg
   CreateByJoining
   WriteToFile           2 def args
   IsRational
   GetDegree
   GetControlPointsPointer
   GetControlPoint
   GetNumberNaturalKnots
   GetKnots              1 def arg
   GetGwNurbPointer
   Length
   Evaluate              1 def arg
   EvaluatePoint
   DropPoint
   Trim                  1 def arg
   EditParameterization  1 def arg
   ReverseParameterization
   CreateExtendedCurve   1 def arg
   GetNaturalInterval
   CutWithPlane
   ScaleKnotVector
   SetControlPoint
   AssertValid
   GlobalPointSolve   <- requires SmSolution

***********************************************************************/
static SmStatus my_test_curve_interface()
{
  SmContext sContext;

// By default, don't actually write out files when testing WriteToFile.
// Reset this flag (in the code here or in the debugger) to enable.
static SmBoolean sbDoWriteTests = FALSE;


  //##test  InterpolatePoints:
  SmBSplineCurve *pCurve1 = NULL;
  SmTArray< SmPoint3d > sPoints;
  sPoints.Add( SmPoint3d( 0, 0, 0 ));
  sPoints.Add( SmPoint3d( 4, 0, 0 ));
  sPoints.Add( SmPoint3d( 4, 4, 0 ));
  sPoints.Add( SmPoint3d( 8, 4, 0 ));

  SmInterpolationType ePz = SM_IT_CHORDLENGTH;  // enum: in SmCurveTypes.h

  SmStatus eStat = SmBSplineCurve::InterpolatePoints( sContext,
                       sPoints,
                       NULL,  // Optional parameters
                       3,     // Degree
                       NULL,  // Optional start tangent
                       NULL,  // Optional end tangent
                       FALSE, // Closed?
                       ePz,
                       pCurve1 );

  SER( eStat );
  if ( pCurve1 == NULL )
    { SER( SM_ERR ); }

  // How to test: for the Java interface, WriteToFile().
  // (Draw and Dump are not requested.)

  //##test WriteToFile:
  if ( sbDoWriteTests ) { 
      eStat = pCurve1->WriteToFile(_T("../prog_test/OutputFiles/curve1.smc"), FALSE, TRUE ); 
  }
  SER( eStat );

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe=FALSE;
if ( bDebugMe ) {
  if ( FALSE )
    { smgfx_Erase(); }
  smgfx_SetLook( 1,2, 0,0,1 ); pCurve1->Draw(); sm_GraphicsLoop();
  sm_GraphicsLoop();
}
#endif

  //##test CreateInterpolatingCurve
  SmBSplineCurve * pCurve2 = NULL;
  // Create this so that it will join with pCurve1.
  sPoints.ReSet();
  sPoints.Add( SmPoint3d(  8, 4, 0 ));
  sPoints.Add( SmPoint3d( 12, 4, 0 ));
  sPoints.Add( SmPoint3d( 12, 8, 0 ));
  sPoints.Add( SmPoint3d( 16, 8, 0 ));

  SmTArray<SmVector3d> sVectorsToInterpolate;
  SmCurveParameterizationType ePz2 = SM_CP_CHORDLENGTH;  // enum: in SmCurveTypes.h

  eStat = SmBSplineCurve::CreateInterpolatingCurve( sContext,
     ePz2,
     3,  // Dimension,
     3,  // Degree,
     sPoints,
     sVectorsToInterpolate, // empty.
     NULL, // cpMoreVectorsToInterpolate,
     TRUE, // bIgnoreVectorMagnitude,
     pCurve2 );

  SER( eStat );
  if ( pCurve2 == NULL )
    { SER( SM_ERR ); }

  if ( sbDoWriteTests ) { 
      eStat = pCurve2->WriteToFile(_T("../prog_test/OutputFiles/curve2.smc"), FALSE, TRUE ); 
  }

#ifdef SM_DEBUG_CODE
if ( bDebugMe ) {
  if ( FALSE )
    { smgfx_Erase(); }
  smgfx_SetLook( 1,2, 0,1,0 ); pCurve2->Draw(); sm_GraphicsLoop();
  sm_GraphicsLoop();
}
#endif


  //##test JoinWith (no default args)
  // JoinWith takes an optional Tolerance argument.
  // We should be able to make it succeed with our tolerance, and fail at the default.
  // Unfortunately we can't, because JoinWith calls CreateByJoining, which doesn't
  // take a tolerance argument, it figures its own, which is .001 in this case.
  // So the curves have to be within that to ever succeed.
  // If JoinWith is not passed a tolerance, it figures a tol of .026, which
  // won't change its behavior in any case.
  // So, join twice.
  eStat = pCurve1->JoinWith( 1, pCurve2, 0 );
  SER( eStat );

  if ( sbDoWriteTests ) { 
      eStat = pCurve1->WriteToFile(_T("../prog_test/OutputFiles/curveJoin1.smc"), FALSE, TRUE );
  }

#ifdef SM_DEBUG_CODE
if ( bDebugMe ) {
  if ( FALSE )
    { smgfx_Erase(); }
  smgfx_SetLook( 2,4, 0,1,1 ); pCurve1->Draw(); sm_GraphicsLoop();
  sm_GraphicsLoop();
}
#endif


  //##test JoinWith (one default arg)
  // Have to make another curve for the other JoinWith Call.
  sPoints.ReSet();
  sPoints.Add( SmPoint3d( 16,  8, 0 ));
  sPoints.Add( SmPoint3d( 20,  8, 0 ));
  sPoints.Add( SmPoint3d( 20, 16, 0 ));
  sPoints.Add( SmPoint3d( 24, 16, 0 ));

  SmBSplineCurve * pCurve3 = NULL;
  eStat = SmBSplineCurve::CreateInterpolatingCurve( sContext,
     ePz2,
     3,  // Dimension,
     3,  // Degree,
     sPoints,
     sVectorsToInterpolate, // empty.
     NULL, // cpMoreVectorsToInterpolate,
     TRUE, // bIgnoreVectorMagnitude,
     pCurve3 );

  SER( eStat );
  if ( pCurve2 == NULL )
    { SER( SM_ERR ); }

  double dTol = 0.01;
  eStat = pCurve1->JoinWith( 1, pCurve3, 0, &dTol );
  SER( eStat );

  if ( sbDoWriteTests ) { 
      eStat = pCurve1->WriteToFile(_T("../prog_test/OutputFiles/curveJoin2.smc"), FALSE, TRUE ); 
  }

#ifdef SM_DEBUG_CODE
if ( bDebugMe ) {
  if ( FALSE )
    { smgfx_Erase(); }
  smgfx_SetLook( 1,2, 1,0,1 ); pCurve1->Draw(); sm_GraphicsLoop();
  sm_GraphicsLoop();
}
#endif


  //##test CreateByJoining
  // For CreateByJoining, we need three curves.
  // Let's get rid of what we have and start over.
  delete pCurve1; pCurve1 = NULL;
  delete pCurve2; pCurve2 = NULL;
  delete pCurve3; pCurve3 = NULL;

  sPoints.ReSet();
  sPoints.Add( SmPoint3d(  0, 0, 0 ));
  sPoints.Add( SmPoint3d(  4, 0, 0 ));
  sPoints.Add( SmPoint3d(  4, 4, 0 ));
  sPoints.Add( SmPoint3d(  8, 4, 0 ));

  eStat = SmBSplineCurve::InterpolatePoints( sContext, sPoints,
          NULL, 3, NULL, NULL, FALSE, ePz, pCurve1 );

  SER( eStat );
  if ( pCurve1 == NULL )
    { SER( SM_ERR ); }

  sPoints.ReSet();
  sPoints.Add( SmPoint3d(  8, 4, 0 ));
  sPoints.Add( SmPoint3d( 12, 4, 0 ));
  sPoints.Add( SmPoint3d( 12, 8, 0 ));
  sPoints.Add( SmPoint3d( 16, 8, 0 ));

  eStat = SmBSplineCurve::InterpolatePoints( sContext, sPoints,
          NULL, 3, NULL, NULL, FALSE, ePz, pCurve2 );

  SER( eStat );
  if ( pCurve1 == NULL )
    { SER( SM_ERR ); }

  sPoints.ReSet();
  sPoints.Add( SmPoint3d( 16,  8, 0 ));
  sPoints.Add( SmPoint3d( 20,  8, 0 ));
  sPoints.Add( SmPoint3d( 20, 12, 0 ));
  sPoints.Add( SmPoint3d( 24, 12, 0 ));

  eStat = SmBSplineCurve::InterpolatePoints( sContext, sPoints,
          NULL, 3, NULL, NULL, FALSE, ePz, pCurve3 );

  SER( eStat );
  if ( pCurve1 == NULL )
    { SER( SM_ERR ); }

  SmTArray<SmBSplineCurve*> sBSpCrvs;
  sBSpCrvs.Add( pCurve1 );
  sBSpCrvs.Add( pCurve2 );
  sBSpCrvs.Add( pCurve3 );
  SmBSplineCurve *pCurve4 = NULL;

  eStat = SmBSplineCurve::CreateByJoining( sContext, sBSpCrvs, NULL, pCurve4 );

  SER( eStat );
  if ( pCurve4 == NULL )
    { SER( SM_ERR ); }

  if ( sbDoWriteTests ) { 
      eStat = pCurve4->WriteToFile(_T("../prog_test/OutputFiles/curveJoin3.smc"), FALSE, TRUE ); 
  }

#ifdef SM_DEBUG_CODE
if ( bDebugMe ) {
  if ( FALSE )
    { smgfx_Erase(); }
  smgfx_SetLook( 1,2, 1,0,0 ); pCurve4->Draw(); sm_GraphicsLoop();
  sm_GraphicsLoop();
}
#endif


  //##test IsRational
  SmBoolean bFlag = pCurve4->IsRational();
  if ( bFlag == TRUE )
    { SER( SM_ERR ); }

  //##test GetDegree
  ULONG lNum = pCurve4->GetDegree();
  if ( lNum != 3 )
    { SER( SM_ERR ); }

  //##test GetControlPointsPointer
  double *pdPtsPtr = NULL;
  eStat = pCurve4->GetControlPointsPointer( lNum, pdPtsPtr );

  SER( eStat );
  if ( lNum != 10 )
    { SER( SM_ERR ); }
  if ( pdPtsPtr == NULL )
    { SER( SM_ERR ); }
  if ( pdPtsPtr[12] != 8.0 )
    { SER( SM_ERR ); }

  //##test GetControlPoint
  SmControlPointFormType eCPType = SM_CP_NON_RATIONAL;  // enum: in SmCurveTypes.h
  SmPoint3d sCtrlPt;
  double dWt;
  eStat = pCurve4->GetControlPoint( eCPType, 3, sCtrlPt, dWt );
  SER( eStat );
  if ( sCtrlPt.x != 8.0 )
    { SER( SM_ERR ); }
  if ( dWt != 1.0 )
    { SER( SM_ERR ); }

  //##test GetNumberNaturalKnots
  lNum = pCurve4->GetNumberNaturalKnots();
  if ( lNum != 14 )
    { SER( SM_ERR ); }

  //##test GetKnots ( no default args )
  SmTArray< double > sKnots;
  eStat = pCurve4->GetKnots( sKnots );
  SER( eStat );
  if ( sKnots.GetSize() != 4 )
    { SER( SM_ERR ); }
  if ( sKnots[1] != 1.0 )
    { SER( SM_ERR ); }

  SmTArray< ULONG  > sMults;
  eStat = pCurve4->GetKnots( sKnots, &sMults );
  SER( eStat );
  if ( sMults.GetSize() != 4 )
    { SER( SM_ERR ); }
  if ( sMults[1] != 3 )
    { SER( SM_ERR ); }



  //##test Evaluate, no default arg
  dTol = 1.0e-12;  // Need a tolerance for Evaluate.
  SmVector3d sPtVec[2];

  eStat = pCurve4->Evaluate( 0.5, 1, FALSE, sPtVec );
  SER( eStat );
  if ( fabs( sPtVec[1].y - 13 ) > dTol )
    { SER( SM_ERR ); }

  //##test Evaluate, one default arg
  eStat = pCurve4->Evaluate( 0.5, 1, FALSE, sPtVec, TRUE );
  SER( eStat );
  if ( fabs( sPtVec[1].y - 13 ) > dTol )
    { SER( SM_ERR ); }

  //##test EvaluatePoint
  SmPoint3d sPoint;
  eStat = pCurve4->EvaluatePoint( 0.5, sPoint );
  SER( eStat );
  if ( fabs( sPoint.y - 2 ) > dTol )
    { SER( SM_ERR ); }

  //##test Trim, no default arg
  SmExtent1d sExtent( 0.0, 2.0 );
  eStat = pCurve4->Trim( sExtent );
  SER( eStat );
  lNum = pCurve4->GetNumberNaturalKnots();
  if ( lNum != 11 )
    { SER( SM_ERR ); }

  //##test Trim, one default arg
  sExtent.SetMinMax( 1.0, 2.0 );
  eStat = pCurve4->Trim( sExtent, FALSE );
  SER( eStat );
  lNum = pCurve4->GetNumberNaturalKnots();
  if ( lNum != 8 )
    { SER( SM_ERR ); }

  //##test EditParameterization, no default args
  sExtent.SetMinMax( 0.0, 1.0 );
  eStat = pCurve4->EditParameterization( sExtent );
  SER( eStat );
  eStat = pCurve4->EvaluatePoint( 0.5, sPoint );
  if ( fabs( sPoint.y - 6 ) > dTol )
    { SER( SM_ERR ); }

  //##test EditParameterization, one default arg
  sExtent.SetMinMax( 2.0, 4.0 );
  eStat = pCurve4->EditParameterization( sExtent, FALSE );
  SER( eStat );
  eStat = pCurve4->EvaluatePoint( 3.0, sPoint );
  if ( fabs( sPoint.y - 6 ) > dTol )
    { SER( SM_ERR ); }

  //##test ReverseParameterization
  SmExtent1d sExtent2;
  eStat = pCurve4->ReverseParameterization( sExtent, sExtent2 );
  SER( eStat );
  eStat = pCurve4->EvaluatePoint( 0.0, sPoint );
  if ( fabs( sPoint.y - 8 ) > dTol )
    { SER( SM_ERR ); }


  //## test CreateExtendedCurve( parameter ), no default arg
  delete pCurve3; pCurve3 = NULL;
  eStat = pCurve1->CreateExtendedCurve( sContext, 1.5, SM_CT_CINFINITY, pCurve3 );
  SER( eStat );
  if ( pCurve3 == NULL )
    { SER( SM_ERR ); }

  pCurve3->EvaluatePoint( 2.5, sPoint );
  if ( fabs( sPoint.y + 260 ) > dTol )
    { SER( SM_ERR ); }

#ifdef SM_DEBUG_CODE
if ( bDebugMe ) {
  pCurve3->Dump();
  if ( FALSE )
    { smgfx_Erase(); }
  smgfx_SetLook( 4,6, 0,0,1 ); pCurve3->Draw(); sm_GraphicsLoop();
  sm_GraphicsLoop();
}
#endif

  //## test CreateExtendedCurve( parameter ), one default arg
  delete pCurve3; pCurve3 = NULL;
  eStat = pCurve1->CreateExtendedCurve( sContext, 1.5, SM_CT_CINFINITY, pCurve3, TRUE );
  SER( eStat );
  if ( pCurve3 == NULL )
    { SER( SM_ERR ); }

  pCurve3->EvaluatePoint( 1.5, sPoint );
  if ( fabs( sPoint.y + 21 ) > dTol )
    { SER( SM_ERR ); }

#ifdef SM_DEBUG_CODE
if ( bDebugMe ) {
  pCurve3->Dump();
  if ( FALSE )
    { smgfx_Erase(); }
  smgfx_SetLook( 5,7, 1,0,0 ); pCurve3->Draw(); sm_GraphicsLoop();
  sm_GraphicsLoop();
}
#endif

  //## test CreateExtendedCurve( distance )
  delete pCurve3; pCurve3 = NULL;
  int iEnd = 2;
  eStat = pCurve1->CreateExtendedCurve( sContext, 25.0, iEnd, SM_CT_CINFINITY, pCurve3 );
  SER( eStat );
  if ( pCurve3 == NULL )
    { SER( SM_ERR ); }

  pCurve3->EvaluatePoint( 1.25, sPoint );
  if ( fabs( sPoint.y + 3.4375 ) > dTol )
    { SER( SM_ERR ); }

#ifdef SM_DEBUG_CODE
if ( bDebugMe ) {
  pCurve3->Dump();
  if ( FALSE )
    { smgfx_Erase(); }
  smgfx_SetLook( 5,7, 1,0,0 ); pCurve3->Draw(); sm_GraphicsLoop();
  sm_GraphicsLoop();
}
#endif


  //##test GetNaturalInterval
  sExtent = pCurve3->GetNaturalInterval();
  if ( sExtent.GetMin() != 0.0 )
    { SER( SM_ERR ); }
  if ( smos_Fabs( sExtent.GetMax() - 1.3746575285553713 ) > dTol )
    { SER( SM_ERR ); }


  //##test ScaleKnotVector
  eStat = pCurve1->ScaleKnotVector( 5, 7 );
  SER( eStat );
  pCurve1->EvaluatePoint( 6.0, sPoint );
  if ( fabs( sPoint.y - 2 ) > dTol )
    { SER( SM_ERR ); }


  //##test SetControlPoint
  eCPType = SM_CP_NON_RATIONAL;  // enum SmControlPointFormType: defined above
  eStat = pCurve1->SetControlPoint( eCPType, 1, sPoint, 1.0 );
  SER( eStat );
  pCurve1->EvaluatePoint( 6.0, sPoint );
  if ( fabs( sPoint.y - 4.5 ) > dTol )
    { SER( SM_ERR ); }

#ifdef SM_DEBUG_CODE
if ( bDebugMe ) {
  pCurve1->Dump();
  if ( FALSE )
    { smgfx_Erase(); }
  smgfx_SetLook( 3,5, 1,0,0 ); pCurve1->Draw(); sm_GraphicsLoop();
  sm_GraphicsLoop();
}
#endif


  //##test AssertValid
  bFlag = pCurve1->AssertValid();
  if ( ! bFlag )   // Not much else to test...
    { SER( SM_ERR ); }


  //////////
// Implemented in SmCurve.cpp:

  //##test Length
  sExtent = pCurve1->GetNaturalInterval();
  double dLen;
  dTol = 0.000001;
  eStat = pCurve1->Length( sExtent, dTol, dLen );
  if ( fabs( dLen - 11.888509121634552 ) > dTol )
    { SER( SM_ERR ); }


  //##test DropPoint
  // sPoint( 2.25, 4.5, 0.0 )
  sExtent = pCurve1->GetNaturalInterval();
  double dParam, dDist;
  eStat = pCurve1->DropPoint(sExtent,  // in : target curve allowed domain
                             sPoint,   // in : Point to drop to curve
                             NULL,     // in : For Snapping, NULL=NoSnap, Vec pointing to CrvInside from desired EndPt for snapping.
                                       //      when DropPt is within dDistTol in cpVecToCrvInside dir of EndPt, Snap to Ivl EndPt.
                                       //      Vec handles ambiguities and makes sure that curves are not just touching at the ends.
                             0.001,    // in : Used for SnapDist, and SM_SO_INTERSECT sXSectTol3d distance.
                                       //      sXSectTol3d = dDistTol ;      SM_SO_INTERSECT 
                                       //      sXSectTol3d = SM_BIG_DOUBLE ; SM_SO_MINIMIZE, SM_SO_NORMALIZE
                                       //      sSnapTol3d  = dDistTol ;      SM_SO_MINIMIZE, SM_SO_NORMALIZE, SM_SO_INTERSECT    
                             NULL,     // in : If given, call cheap LocalPointSolve() else call expensive GlobalPointSolve()
                             bFlag,    // out: TRUE = found a drop point
                             dParam,   // out: found drop curve param
                             dDist) ;  // out: found drop distance
                                       // in : SM_SO_MINIMIZE = allow nonNormal drops near endPoints
                                       //      SM_SO_INTERSECT= like Minimize but point must be within dDistTol
                                       //      SM_SO_NORMALIZE= exclude nonNormal drops near endPoints
                                       //      default:[SM_SO_MINIMIZE] to preserve original behavior

  dTol = 1.0e-12;
  SER( eStat );
  if ( ! bFlag )
    { SER( SM_ERR ); }
  if ( fabs( dParam - 6 ) > dTol )
    { SER( SM_ERR ); }
  if ( fabs( dDist ) > dTol )
    { SER( SM_ERR ); }


  //##test CutWithPlane
  SmTArray< SmCurve* > sCrvs;
  SmVector3d sNorm( 1, 0, 0 );
  eStat = pCurve1->CutWithPlane( sContext, sPoint, sNorm, sCrvs );
  SmObjsDelete<SmCurve*> sCleanC( &sCrvs );
  SER( eStat );
  if ( sCrvs.GetSize() != 1 )
    { SER( SM_ERR ); }


  //##test GlobalPointSolve  also, requires SmSolutionArray

  ///////////////
// Not yet: need other classes first:
  // CreateCircleSegment  requires SmAxis2Placement
  // GlobalPointSolve     requires SmSolutionArray
  // GetGwNurbPointer     requires gw_CURVE

  // Clean up.
  delete pCurve1; pCurve1 = NULL;
  delete pCurve2; pCurve2 = NULL;
  delete pCurve3; pCurve3 = NULL;
  delete pCurve4; pCurve4 = NULL;


  return SM_SUCCESS;
}


/***********************************************************************
PURPOSE ---

USAGE NOTES ---
***********************************************************************/
PT_EXPORT SmStatus my_test_one_curve_prop
  (SmCurve          * pNurb1,            // in : 
   const SmVector3d & crDelta,           // in : 
   ULONG            & rlTotalFound)      // out: 
{
  rlTotalFound = 0;
  
  SmTArray<SmPoint3d> sCombPoints, sCombVecs ;
  SmAxis2Placement sA2P;
  sA2P.SetCanonical(crDelta, SmVector3d(1, 0, 0), SmVector3d(0, 1, 0));
  
#ifdef SM_GFX_CODE
  if (smGet_DoGraphics()) {
      smgfx_Erase();
      smgfx_SetLook(1, 2, 0, 0, 1); pNurb1->Draw(); sm_GraphicsLoop();
      sm_GraphicsLoop();
  }
#endif

  if (TRUE)
    {
      // Plane Classification
      SmSolutionArray sSolutions;
      SmVector3d sPlaneNorm(1.0, 0.0, 0.0);
      sPlaneNorm.Unitize();
      double dD = 0;
      for (ULONG i = 0; i < 1; i++)
        {
          // Find segments of a curve on the positive side of plane
          // equation.  The plane is defined by a normal vector (unitized) and
          // and the plane equation D value input as a value.  The plane equation
          // is Ax + By + Cz + D = 0.  Where A,B,C is a unitized plane normal and
          // D the negative value of any point on the plane doted with the normal.
          SER(pNurb1->GlobalPropertyAnalysis(pNurb1->GetNaturalInterval(),
                                             SM_CP_PLANE_CLASSIFY, 
                                             &dD, 
                                             &sPlaneNorm, 
                                             SM_ZONE_TOL_3D, 
                                             sSolutions, 
                                             &sCombPoints, 
                                             &sCombVecs));
          // call 0, sSolutions.GetSize() == 5 ; GWC: curve has been translated - now expect sSolutions.GetSize() == 1
          // call 1, sSolutions.GetSize() == 1
        }
      if (smGet_DoGraphics())
        {
          for (ULONG kk = 0; kk < sSolutions.GetSize(); kk++)
            {
              rlTotalFound++;
#ifdef SM_GFX_CODE
              if(kk == 0) pNurb1->Dump() ;

              SmPoint3d sStartPnt;
              SmPoint3d sEndPnt;
              SER(pNurb1->EvaluatePoint(sSolutions[kk].m_vStart[0], sStartPnt));
              SER(pNurb1->EvaluatePoint(sSolutions[kk].m_vEnd[0],   sEndPnt));
              SmVector3d sVec = sStartPnt - sEndPnt;
              SmPoint2d   sScale(1.0, 1.0) ;
              SmExtent2d  sAnalUV(0,0,10,10) ;
              SmVector3d  sX, sY, sZ ;
              sPlaneNorm.MakeUnitOrthoVectors(NULL,sZ,sX,sY);
              SmPlane sPlane(sPlaneNorm * -dD, sX, sY, sScale, sAnalUV, pNurb1->GetContext()) ;

              smgfx_SetLook(1,3, 1,0,0) ; sStartPnt.Draw(); sm_GraphicsLoop() ;
              smgfx_SetLook(1,3, 1,0,0) ; sEndPnt.Draw(); sm_GraphicsLoop() ;
              smgfx_SetLook(1,3, 0,1,0) ; sVec.Draw(&sEndPnt); sm_GraphicsLoop() ;
              smgfx_SetLook(1,2, 0,0,1) ; if(kk==0) sPlane.Draw() ; sm_GraphicsLoop() ;
              sm_GraphicsLoop() ;
#endif
            }
          my_draw_crv_pnt_sol(pNurb1, NULL, sSolutions, &sCombPoints, &sCombVecs);

        }
    }
  
  pNurb1->Transform(sA2P);
#ifdef SM_GFX_CODE
  if (smGet_DoGraphics())
        {
            smgfx_SetLook(1,2, 0,0,1) ; pNurb1->Draw() ; sm_GraphicsLoop() ;
            sm_GraphicsLoop() ;
        }            
#endif
  
  if (TRUE)
    {
      // Plane Classification
      SmSolutionArray sSolutions;
      SmVector3d sPlaneNorm(-1.0, 0.0, 0.0);
      sPlaneNorm.Unitize();
      double dD = 2.5;
      for (ULONG i = 0; i < 1; i++)
        {
          // Find segments of a curve on the positive side of plane
          // equation.  The plane is defined by a normal vector (unitized) and
          // and the plane equation D value input as a value.  The plane equation
          // is Ax + By + Cz + D = 0.  Where A,B,C is a unitized plane normal and
          // D the negative value of any point on the plane doted with the normal.
          SER(pNurb1->GlobalPropertyAnalysis(pNurb1->GetNaturalInterval(),
                                             SM_CP_PLANE_CLASSIFY, 
                                             &dD, 
                                             &sPlaneNorm, 
                                             SM_ZONE_TOL_3D, 
                                             sSolutions, 
                                             &sCombPoints, 
                                             &sCombVecs));
          // call 0, sSolutions.GetSize() == 6 ; with the translated curve now expect sSolutions.GetSize() == 0
          // call 1, sSolutions.GetSize() == 0
        }
      if (smGet_DoGraphics())
        {
          for (ULONG kk = 0; kk < sSolutions.GetSize(); kk++)
            {
              rlTotalFound++;
#ifdef SM_GFX_CODE
              SmPoint3d sStartPnt;
              SmPoint3d sEndPnt;
              SER(pNurb1->EvaluatePoint(sSolutions[kk].m_vStart[0], sStartPnt));
              SER(pNurb1->EvaluatePoint(sSolutions[kk].m_vEnd[0],   sEndPnt));
              SmVector3d sVec = sStartPnt - sEndPnt;
              SmPoint2d   sScale(1.0, 1.0) ;
              SmExtent2d  sAnalUV(0,0,10,10) ;
              SmVector3d  sX, sY, sZ ;
              sPlaneNorm.MakeUnitOrthoVectors(NULL,sZ,sX,sY);
              SmPlane sPlane(sPlaneNorm * -dD, sX, sY, sScale, sAnalUV, pNurb1->GetContext()) ;

              smgfx_SetLook(1,3, 1,0,0) ; sStartPnt.Draw(); sm_GraphicsLoop() ;
              smgfx_SetLook(1,3, 1,0,0) ; sEndPnt.Draw(); sm_GraphicsLoop() ;
              smgfx_SetLook(1,3, 0,1,0) ; sVec.Draw(&sEndPnt); sm_GraphicsLoop() ;
              smgfx_SetLook(1,2, 0,0,1) ; if(kk==0) sPlane.Draw() ; sm_GraphicsLoop() ;
              sm_GraphicsLoop() ;
#endif
            }
          my_draw_crv_pnt_sol(pNurb1, NULL, sSolutions, &sCombPoints, &sCombVecs);
        }
    }
  
  pNurb1->Transform(sA2P);
#ifdef SM_GFX_CODE
      if (smGet_DoGraphics())
        {
          smgfx_SetLook(1,2, 0,0,1) ; pNurb1->Draw() ; sm_GraphicsLoop() ;
          sm_GraphicsLoop() ; 
        }           
#endif
  
  if (TRUE)
    {
      // Plane Intersection
      SmSolutionArray sSolutions;
      SmVector3d sPlaneNorm(1.0, 0.0, 0.0);
      sPlaneNorm.Unitize();
      double dD = -5.0;
      for (ULONG i = 0; i < 1; i++)
        {
          // Find points of intersection with plane equation
          // The plane is defined by a normal vector (unitized) and
          // and the plane equation D value input as a value
          SER(pNurb1->GlobalPropertyAnalysis(pNurb1->GetNaturalInterval(),
              SM_CP_PLANE_INTERSECTION, &dD, &sPlaneNorm, SM_ZONE_TOL_3D, sSolutions, &sCombPoints, &sCombVecs));
          // call 0, sSolutions.GetSize() == 10
          // call 1, sSolutions.GetSize() == 0
        }
      if (smGet_DoGraphics())
        {
          for (ULONG kk = 0; kk < sSolutions.GetSize(); kk++)
            {
              rlTotalFound++;
#ifdef SM_GFX_CODE
              SmPoint3d sFndPnt;
              SER(pNurb1->EvaluatePoint(sSolutions[kk].m_vStart[0], sFndPnt));
              SmPoint2d   sScale(1.0, 1.0) ;
              SmExtent2d  sAnalUV(0,0,10,10) ;
              SmVector3d  sX, sY, sZ ;
              sPlaneNorm.MakeUnitOrthoVectors(NULL,sZ,sX,sY);
              SmPlane sPlane(sPlaneNorm * -dD, sX, sY, sScale, sAnalUV, pNurb1->GetContext()) ;

              smgfx_SetLook(1,3, 1,0,0) ; sFndPnt.Draw(); sm_GraphicsLoop() ;
              smgfx_SetLook(1,2, 0,0,1) ; if(kk==0) sPlane.Draw() ; sm_GraphicsLoop() ;
              sm_GraphicsLoop() ;
#endif
            }
          my_draw_crv_pnt_sol(pNurb1, NULL, sSolutions, &sCombPoints, &sCombVecs);
        }
    }
  
  pNurb1->Transform(sA2P);
#ifdef SM_GFX_CODE
  if (smGet_DoGraphics()) {
      smgfx_SetLook(1, 2, 0, 0, 1); pNurb1->Draw(); sm_GraphicsLoop();
      sm_GraphicsLoop();
  }
#endif
  
  if (TRUE)
    {
      // Silhouette to vector test
      SmSolutionArray sSolutions;
      SmVector3d sVec(1.0, 1.0, 0.0);
      sVec.Unitize();
      // Find parallel projection silhouette points relative to a view vector - 
      //      requires vector input
      SER(pNurb1->GlobalPropertyAnalysis(pNurb1->GetNaturalInterval(),
          SM_CP_SILHOUETTE_VECTOR, NULL, &sVec, SM_ZONE_TOL_3D, sSolutions, &sCombPoints, &sCombVecs));
          // call 0, sSolutions.GetSize() == 7
          // call 1, sSolutions.GetSize() == 13 (gwc: this looks wrong to me - needs check)
      if (smGet_DoGraphics())
        {
          for (ULONG kk = 0; kk < sSolutions.GetSize(); kk++)
            {
              rlTotalFound++;
#ifdef SM_GFX_CODE
              SmPoint3d sFndPnt;
              SER(pNurb1->EvaluatePoint(sSolutions[kk].m_vStart[0], sFndPnt));

              smgfx_SetLook(1,3, 1,0,0) ; sFndPnt.Draw(); sm_GraphicsLoop() ;
              smgfx_SetLook(1,2, 0,0,1) ; sVec.Draw(&sFndPnt) ; sm_GraphicsLoop() ;
              sm_GraphicsLoop() ;
#endif
            }
          my_draw_crv_pnt_sol(pNurb1, NULL, sSolutions, &sCombPoints, &sCombVecs);
        }
    }
  
  
  pNurb1->Transform(sA2P);
#ifdef SM_GFX_CODE
  if (smGet_DoGraphics()) {
      smgfx_SetLook(1, 2, 0, 0, 1); pNurb1->Draw(); sm_GraphicsLoop();
      sm_GraphicsLoop();
  }
#endif

  if (TRUE)
    {
      // Silhouette point test
      SmSolutionArray sSolutions;
      SmVector3d sPnt(3.3, 0.0, 0.0);
      // Find perspective projection silhouette points relative to eye point -
      //      requires vector input
      SER(pNurb1->GlobalPropertyAnalysis(pNurb1->GetNaturalInterval(),
          SM_CP_SILHOUETTE_POINT, NULL, &sPnt, SM_ZONE_TOL_3D, sSolutions, &sCombPoints, &sCombVecs));
          // call 0, sSolutions.GetSize() == 6
          // call 1, sSolutions.GetSize() == 8  (gwc:this might need to be 7 - needs check)
      if (smGet_DoGraphics())
        {
          for (ULONG kk = 0; kk < sSolutions.GetSize(); kk++)
            {
              rlTotalFound++;
#ifdef SM_GFX_CODE
              SmPoint3d sFndPnt;
              SER(pNurb1->EvaluatePoint(sSolutions[kk].m_vStart[0], sFndPnt));
              SmVector3d sVec = sFndPnt - sPnt ;

              smgfx_SetLook(1,3, 1,0,0) ; sFndPnt.Draw(); sm_GraphicsLoop() ;
              smgfx_SetLook(1,2, 0,0,1) ; sPnt.Draw() ; sm_GraphicsLoop() ;
              smgfx_SetLook(1,2, 0,1,0) ; sVec.Draw(&sPnt) ; sm_GraphicsLoop() ;
              sm_GraphicsLoop() ;
#endif
            }
          my_draw_crv_pnt_sol(pNurb1, NULL, sSolutions, &sCombPoints, &sCombVecs);
        }
    }
  
  pNurb1->Transform(sA2P);
#ifdef SM_GFX_CODE
  if (smGet_DoGraphics()) {
      smgfx_SetLook(1, 2, 0, 0, 1); pNurb1->Draw(); sm_GraphicsLoop();
      sm_GraphicsLoop();
  }
#endif
  
  if (TRUE)
    {
      // Parallel to vector test
      SmSolutionArray sSolutions;
      SmVector3d sVec(1.0, 1.0, 0.0);
      sVec.Unitize();
      // Find points where tangent parallel to vector -
      //      requires vector input
      SER(pNurb1->GlobalPropertyAnalysis(pNurb1->GetNaturalInterval(),
          SM_CP_PARALLEL_TO_VECTOR, NULL, &sVec, SM_ZONE_TOL_3D, sSolutions, &sCombPoints, &sCombVecs));
          // call 0, sSolutions.GetSize() == 7
          // call 1, sSolutions.GetSize() == 0
      if (smGet_DoGraphics())
        {
          for (ULONG kk = 0; kk < sSolutions.GetSize(); kk++)
            {
              rlTotalFound++;
#ifdef SM_GFX_CODE
              SmPoint3d sFndPnt;
              SER(pNurb1->EvaluatePoint(sSolutions[kk].m_vStart[0], sFndPnt));

              smgfx_SetLook(1,3, 1,0,0) ; sFndPnt.Draw(); sm_GraphicsLoop() ;
              smgfx_SetLook(1,2, 0,0,1) ; sVec.Draw(&sFndPnt) ; sm_GraphicsLoop() ;
              sm_GraphicsLoop() ;
#endif
            }
          my_draw_crv_pnt_sol(pNurb1, NULL, sSolutions, &sCombPoints, &sCombVecs);
        }
    }
  
  pNurb1->Transform(sA2P);
#ifdef SM_GFX_CODE
  if (smGet_DoGraphics()) {
      smgfx_SetLook(1, 2, 0, 0, 1); pNurb1->Draw(); sm_GraphicsLoop();
      sm_GraphicsLoop();
  }
#endif
  if (TRUE)
    {
      // Test finding of inflection points
      SmSolutionArray sSolutions;
      // Find points where second derivative vanishes
      SER(pNurb1->GlobalPropertyAnalysis(pNurb1->GetNaturalInterval(),
          SM_CP_INFLECTION_POINTS, NULL, NULL, SM_ZONE_TOL_3D, sSolutions, &sCombPoints, &sCombVecs));
          // call 0, sSolutions.GetSize() == 8
          // call 1, sSolutions.GetSize() == 0 (not a planar curve)
      if (smGet_DoGraphics())
        {
          for (ULONG kk = 0; kk < sSolutions.GetSize(); kk++)
            {
              rlTotalFound++;
#ifdef SM_GFX_CODE
              SmPoint3d sFndPnt;
              SER(pNurb1->EvaluatePoint(sSolutions[kk].m_vStart[0], sFndPnt));

              smgfx_SetLook(1,3, 1,0,0) ; sFndPnt.Draw(); sm_GraphicsLoop() ;
              sm_GraphicsLoop() ;
#endif
            }
          my_draw_crv_pnt_sol(pNurb1, NULL, sSolutions, &sCombPoints, &sCombVecs);
        }
    }
  
  pNurb1->Transform(sA2P);
#ifdef SM_GFX_CODE
  if (smGet_DoGraphics()) {
      smgfx_SetLook(1, 2, 0, 0, 1); pNurb1->Draw(); sm_GraphicsLoop();
      sm_GraphicsLoop();
  }
#endif
  
  if (TRUE)
    {
      // Min tangent
      SmSolutionArray sSolutions;
      // Find minimum first derivative points - find all minima within toler 
      //   of the smallest first deriviate value (just one.)
      SER(pNurb1->GlobalPropertyAnalysis(pNurb1->GetNaturalInterval(),
          SM_CP_MINIMIZE_FIRST_DERIVATIVE, NULL, NULL, SM_ZONE_TOL_3D, sSolutions, &sCombPoints, &sCombVecs));
          // call 0, sSolutions.GetSize() == 1
          // call 1, sSolutions.GetSize() == 1
      if (smGet_DoGraphics())
        {
          for (ULONG kk = 0; kk < sSolutions.GetSize(); kk++)
            {
              rlTotalFound++;
#ifdef SM_GFX_CODE
              SmPoint3d sFndPnt;
              SER(pNurb1->EvaluatePoint(sSolutions[kk].m_vStart[0], sFndPnt));

              smgfx_SetLook(1,3, 1,0,0) ; sFndPnt.Draw(); sm_GraphicsLoop() ;
              sm_GraphicsLoop() ;
#endif
            }
          my_draw_crv_pnt_sol(pNurb1, NULL, sSolutions, &sCombPoints, &sCombVecs);
        }
    }
  if (TRUE)
    {
      // Max tangent
      SmSolutionArray sSolutions;
      // Find maximum first derivative points
      SER(pNurb1->GlobalPropertyAnalysis(pNurb1->GetNaturalInterval(),
          SM_CP_MAXIMIZE_FIRST_DERIVATIVE, NULL, NULL, SM_ZONE_TOL_3D, sSolutions, &sCombPoints, &sCombVecs));
          // call 0, sSolutions.GetSize() == 1
          // call 1, sSolutions.GetSize() == 1
      if (smGet_DoGraphics())
        {
          for (ULONG kk = 0; kk < sSolutions.GetSize(); kk++)
            {
              rlTotalFound++;
#ifdef SM_GFX_CODE
              SmPoint3d sFndPnt;
              SER(pNurb1->EvaluatePoint(sSolutions[kk].m_vStart[0], sFndPnt));

              smgfx_SetLook(1,3, 1,0,0) ; sFndPnt.Draw(); sm_GraphicsLoop() ;
              sm_GraphicsLoop() ;
#endif
            }
          my_draw_crv_pnt_sol(pNurb1, NULL, sSolutions, &sCombPoints, &sCombVecs);
        }
    }
  
  pNurb1->Transform(sA2P);
#ifdef SM_GFX_CODE
  if (smGet_DoGraphics()) {
      smgfx_SetLook(1, 2, 0, 0, 1); pNurb1->Draw(); sm_GraphicsLoop();
      sm_GraphicsLoop();
  }
#endif
  
  if (TRUE)
    {
      // Min radius of curvature
      SmSolutionArray sSolutions;
      // Find minimum radius of curvature points
      SER(pNurb1->GlobalPropertyAnalysis(pNurb1->GetNaturalInterval(),
          SM_CP_MINIMIZE_RADIUS_OF_CURVATURE, NULL, NULL, 1.0e-9, sSolutions, &sCombPoints, &sCombVecs));
          // call 0, sSolutions.GetSize() == 1
          // call 1, sSolutions.GetSize() == 1
      if (smGet_DoGraphics())
        {
          for (ULONG kk = 0; kk < sSolutions.GetSize(); kk++)
            {
              rlTotalFound++;
#ifdef SM_GFX_CODE
              SmPoint3d sFndPnt;
              SER(pNurb1->EvaluatePoint(sSolutions[kk].m_vStart[0], sFndPnt));

              smgfx_SetLook(1,3, 1,0,0) ; sFndPnt.Draw(); sm_GraphicsLoop() ;
              sm_GraphicsLoop() ;
#endif
            }
          my_draw_crv_pnt_sol(pNurb1, NULL, sSolutions, &sCombPoints, &sCombVecs);
        }
    }
  if (TRUE)
    {
      // Max radius of curvature
      SmSolutionArray sSolutions;
      // Find maximum radius of curvature points
      SER(pNurb1->GlobalPropertyAnalysis(pNurb1->GetNaturalInterval(),
          SM_CP_MAXIMIZE_RADIUS_OF_CURVATURE, NULL, NULL, SM_ZONE_TOL_3D, sSolutions, &sCombPoints, &sCombVecs));
          // call 0, sSolutions.GetSize() == 8  (gwc: why isn't this the same number as the number of points of inflection (8) on the planar curve?)
          // call 1, sSolutions.GetSize() == 1
      //    
      if (smGet_DoGraphics())
        {
          for (ULONG kk = 0; kk < sSolutions.GetSize(); kk++)
            {
              rlTotalFound++;
#ifdef SM_GFX_CODE
              SmPoint3d sFndPnt;
              SER(pNurb1->EvaluatePoint(sSolutions[kk].m_vStart[0], sFndPnt));

              smgfx_SetLook(1,3, 1,0,0) ; sFndPnt.Draw(); sm_GraphicsLoop() ;
              sm_GraphicsLoop() ;
#endif
            }
          my_draw_crv_pnt_sol(pNurb1, NULL, sSolutions, &sCombPoints, &sCombVecs);
        }
    }
  
  pNurb1->Transform(sA2P);
#ifdef SM_GFX_CODE
  if (smGet_DoGraphics()) {
      smgfx_SetLook(1, 2, 0, 0, 1); pNurb1->Draw(); sm_GraphicsLoop();
      sm_GraphicsLoop();
  }
#endif

  if (TRUE)
    {
      // Test finding specific first deriv length
      SmSolutionArray sSolutions;
      double dDerivLength = 15.8;
      // Find points where first derivative is of given length - 
      //      requires value input
      SER(pNurb1->GlobalPropertyAnalysis(pNurb1->GetNaturalInterval(),
          SM_CP_FIRST_DERIVATIVE_LENGTH, &dDerivLength, NULL, SM_ZONE_TOL_3D, sSolutions, &sCombPoints, &sCombVecs));
          // call 0, sSolutions.GetSize() == 16  (gwc: this is not verified)
          // call 1, sSolutions.GetSize() == 16  (gwc: this is not verified)
      if (smGet_DoGraphics())
        {
          for (ULONG kk = 0; kk < sSolutions.GetSize(); kk++)
            {
              rlTotalFound++;
#ifdef SM_GFX_CODE
              SmPoint3d sFndPnt[2];
               
              SER(pNurb1->Evaluate(sSolutions[kk].m_vStart[0], 1, FALSE, &sFndPnt[0]));

              smgfx_SetLook(1,3, 1,0,0) ; sFndPnt[0].Draw(); sm_GraphicsLoop() ;
              smgfx_SetLook(1,2, 0,0,1) ; sFndPnt[1].Draw(&sFndPnt[0]) ; sm_GraphicsLoop() ;
              sm_GraphicsLoop() ;
#endif
            }
          my_draw_crv_pnt_sol(pNurb1, NULL, sSolutions, &sCombPoints, &sCombVecs);
        }
    }
  
  pNurb1->Transform(sA2P);
#ifdef SM_GFX_CODE
  if (smGet_DoGraphics()) {
      smgfx_SetLook(1, 2, 0, 0, 1); pNurb1->Draw(); sm_GraphicsLoop();
      sm_GraphicsLoop();
  }
#endif

  if (TRUE)
    {
      // Test finding of specific radius of curvature
      SmSolutionArray sSolutions;
      double dRadiusOfCurvature = 0.8;
      // Find points where radius of curvature equal to given value - 
      //      requires value input
      SER(pNurb1->GlobalPropertyAnalysis(pNurb1->GetNaturalInterval(),
          SM_CP_RADIUS_OF_CURVATURE, &dRadiusOfCurvature, NULL, SM_ZONE_TOL_3D, sSolutions, &sCombPoints, &sCombVecs));
          // call 0, sSolutions.GetSize() == 18  (gwc: this is not verified)
          // call 1, sSolutions.GetSize() == 20  (gwc: this is not verified)
      if (smGet_DoGraphics())
        {
          for (ULONG kk = 0; kk < sSolutions.GetSize(); kk++)
            {
              rlTotalFound++;
#ifdef SM_GFX_CODE
              SmPoint3d sFndPnt[2];
               
              SER(pNurb1->Evaluate(sSolutions[kk].m_vStart[0], 1, FALSE, &sFndPnt[0]));
              
              smgfx_SetLook(1,3, 1,0,0) ; sFndPnt[0].Draw(); sm_GraphicsLoop() ;
              smgfx_SetLook(1,2, 0,0,1) ; sFndPnt[1].Draw(&sFndPnt[0]) ; sm_GraphicsLoop() ;
              sm_GraphicsLoop() ;
#endif
            }
          my_draw_crv_pnt_sol(pNurb1, NULL, sSolutions, &sCombPoints, &sCombVecs);
        }
    }
  
  pNurb1->Transform(sA2P);
#ifdef SM_GFX_CODE
  if (smGet_DoGraphics()) {
      smgfx_SetLook(1, 2, 0, 0, 1); pNurb1->Draw(); sm_GraphicsLoop();
      sm_GraphicsLoop();
  }
#endif
  
  if (TRUE)
    {
      // Perpendicular to X axis
      SmSolutionArray sSolutions;
      SmVector3d sVec(1.0, 0.0, 0.0) ;
      // Find points where X value of tangent becomes zero
      SER(pNurb1->GlobalPropertyAnalysis(pNurb1->GetNaturalInterval(),
          SM_CP_X_NORMAL, NULL, NULL, SM_ZONE_TOL_3D, sSolutions, &sCombPoints, &sCombVecs));
          // call 0, sSolutions.GetSize() == 9
          // call 1, sSolutions.GetSize() == 9
      if (smGet_DoGraphics())
        {
          for (ULONG kk = 0; kk < sSolutions.GetSize(); kk++)
            {
              rlTotalFound++;
#ifdef SM_GFX_CODE
              SmPoint3d sFndPnt[2];
              SER(pNurb1->Evaluate(sSolutions[kk].m_vStart[0], 1, FALSE, &sFndPnt[0]));

              smgfx_SetLook(1,3, 1,0,0) ; sFndPnt[0].Draw(); sm_GraphicsLoop() ;
              smgfx_SetLook(1,2, 0,0,1) ; sFndPnt[1].Draw(&sFndPnt[0]) ; sm_GraphicsLoop() ;
              smgfx_SetLook(1,2, 0,0,1) ; sVec.Draw(&sFndPnt[0]) ; sm_GraphicsLoop() ;
              sm_GraphicsLoop() ;
#endif
            }
          my_draw_crv_pnt_sol(pNurb1, NULL, sSolutions, &sCombPoints, &sCombVecs);
        }
    }
  
  pNurb1->Transform(sA2P);
#ifdef SM_GFX_CODE
  if (smGet_DoGraphics()) {
      smgfx_SetLook(1, 2, 0, 0, 1); pNurb1->Draw(); sm_GraphicsLoop();
      sm_GraphicsLoop();
  }
#endif

  if (TRUE)
    {
      // Perpendicular to Y axis
      SmSolutionArray sSolutions;
      SmVector3d sVec(0.0, 1.0, 0.0) ;
      // Find points where Y value of tangent becomes zero
      SER(pNurb1->GlobalPropertyAnalysis(pNurb1->GetNaturalInterval(),
          SM_CP_Y_NORMAL, NULL, NULL, SM_ZONE_TOL_3D, sSolutions, &sCombPoints, &sCombVecs));
          // call 0, sSolutions.GetSize() == 3
          // call 1, sSolutions.GetSize() == 3
      if (smGet_DoGraphics())
        {
          for (ULONG kk = 0; kk < sSolutions.GetSize(); kk++)
            {
              rlTotalFound++;
#ifdef SM_GFX_CODE
              SmPoint3d sFndPnt[2];
              SER(pNurb1->Evaluate(sSolutions[kk].m_vStart[0], 1, FALSE, &sFndPnt[0]));
              
              smgfx_SetLook(1,3, 1,0,0) ; sFndPnt[0].Draw(); sm_GraphicsLoop() ;
              smgfx_SetLook(1,2, 0,0,1) ; sFndPnt[1].Draw(&sFndPnt[0]) ; sm_GraphicsLoop() ;
              smgfx_SetLook(1,2, 0,0,1) ; sVec.Draw(&sFndPnt[0]) ; sm_GraphicsLoop() ;
              sm_GraphicsLoop() ;
#endif
            }
          my_draw_crv_pnt_sol(pNurb1, NULL, sSolutions, &sCombPoints, &sCombVecs);
        }
    }
  
  if (TRUE)
    {
      // Perpendicular to Z axis
      SmSolutionArray sSolutions;
      SmVector3d sVec(0.0, 0.0, 1.0) ;
      // Find points where Z value of tangent becomes zero
      SER(pNurb1->GlobalPropertyAnalysis(pNurb1->GetNaturalInterval(),
          SM_CP_Z_NORMAL, NULL, NULL, SM_ZONE_TOL_3D, sSolutions, &sCombPoints, &sCombVecs));
          // call 0, sSolutions.GetSize() == 0
          // call 1, sSolutions.GetSize() == 8
      if (smGet_DoGraphics())
        {
          for (ULONG kk = 0; kk < sSolutions.GetSize(); kk++)
            {
              rlTotalFound++;
#ifdef SM_GFX_CODE
              SmPoint3d sFndPnt[2];
              SER(pNurb1->Evaluate(sSolutions[kk].m_vStart[0], 1, FALSE, &sFndPnt[0]));

              smgfx_SetLook(1,3, 1,0,0) ; sFndPnt[0].Draw(); sm_GraphicsLoop() ;
              smgfx_SetLook(1,2, 0,0,1) ; sFndPnt[1].Draw(&sFndPnt[0]) ; sm_GraphicsLoop() ;
              smgfx_SetLook(1,2, 0,0,1) ; sVec.Draw(&sFndPnt[0]) ; sm_GraphicsLoop() ;
              sm_GraphicsLoop() ;
#endif
            }
          my_draw_crv_pnt_sol(pNurb1, NULL, sSolutions, &sCombPoints, &sCombVecs);
        }
    }
  
  pNurb1->Transform(sA2P);
#ifdef SM_GFX_CODE
  if (smGet_DoGraphics()) {
      smgfx_SetLook(1, 2, 0, 0, 1); pNurb1->Draw(); sm_GraphicsLoop();
      sm_GraphicsLoop();
  }
#endif
  
  if (TRUE)
    {
      // Perpendicular to X, Y or Z axis
      SmSolutionArray sSolutions;
      SmVector3d sVec1(1.0, 0.0, 0.0) ;
      SmVector3d sVec2(0.0, 1.0, 0.0) ;
      SmVector3d sVec3(0.0, 0.0, 1.0) ;
      // Find points where X,Y or Z value of tangent becomes zero
      SER(pNurb1->GlobalPropertyAnalysis(pNurb1->GetNaturalInterval(),
          SM_CP_XYZ_NORMAL, NULL, NULL, SM_ZONE_TOL_3D, sSolutions, &sCombPoints, &sCombVecs));
          // call 0, sSolutions.GetSize() == 12
          // call 1, sSolutions.GetSize() == 20
      if (smGet_DoGraphics())
        {
          for (ULONG kk = 0; kk < sSolutions.GetSize(); kk++)
            {
              rlTotalFound++;
#ifdef SM_GFX_CODE
              SmPoint3d sFndPnt[2];
              SER(pNurb1->Evaluate(sSolutions[kk].m_vStart[0], 1, FALSE, &sFndPnt[0]));

              smgfx_SetLook(1,3, 1,0,0) ; sFndPnt[0].Draw(); sm_GraphicsLoop() ;
              smgfx_SetLook(1,2, 0,0,1) ; sFndPnt[1].Draw(&sFndPnt[0]) ; sm_GraphicsLoop() ;
              smgfx_SetLook(1,2, 0,0,1) ; sVec1.Draw(&sFndPnt[0]) ; sm_GraphicsLoop() ;
              smgfx_SetLook(1,2, 0,0,1) ; sVec2.Draw(&sFndPnt[0]) ; sm_GraphicsLoop() ;
              smgfx_SetLook(1,2, 0,0,1) ; sVec3.Draw(&sFndPnt[0]) ; sm_GraphicsLoop() ;
              sm_GraphicsLoop() ;
#endif
            }
          my_draw_crv_pnt_sol(pNurb1, NULL, sSolutions, &sCombPoints, &sCombVecs);
        }
    }
  
  pNurb1->Transform(sA2P);
#ifdef SM_GFX_CODE
  if (smGet_DoGraphics()) {
      smgfx_SetLook(1, 2, 0, 0, 1); pNurb1->Draw(); sm_GraphicsLoop();
      sm_GraphicsLoop();
  }
#endif
  
  if (TRUE)
    {
      // Tangent to a circle that's tangent to a given point.
      SmSolutionArray sSolutions;
      SmPoint3d sCirclePtTan[2];
      sCirclePtTan[0].Set( 40, 0, 0 );
      sCirclePtTan[1].Set(  1, 0, 0 );
      sCirclePtTan[1].Unitize();

      // Find points where some circle can be tangent to both.
      // In 2d, there should be 6.  (It's not meant to do 3d.)
      SER(pNurb1->GlobalPropertyAnalysis(pNurb1->GetNaturalInterval(),
          SM_CP_TANGENT_TO_CIRCLE, NULL, sCirclePtTan, SM_ZONE_TOL_3D, sSolutions, &sCombPoints, &sCombVecs));
          // call 0, sSolutions.GetSize() ==  6
          // call 1, sSolutions.GetSize() == 11
      if (smGet_DoGraphics())
        {
          for (ULONG kk = 0; kk < sSolutions.GetSize(); kk++)
            {
              rlTotalFound++;
#ifdef SM_GFX_CODE
              SmPoint3d sFndPnt[2];
              SER(pNurb1->Evaluate(sSolutions[kk].m_vStart[0], 1, FALSE, &sFndPnt[0]));

              smgfx_SetLook(1,3, 1,0,0) ; sFndPnt[0].Draw(); sm_GraphicsLoop() ;

              SmVector3d sPlaneNorm = sCirclePtTan[1] * sFndPnt[1];
              double dMag = sPlaneNorm.Length();
              if ( dMag > SM_EFF_ZERO_SQRT ) {
                  sPlaneNorm /= dMag;
                  SmVector3d sCirNorm = sPlaneNorm * sCirclePtTan[1];
                  SmVector3d sCrvNorm = sPlaneNorm * sFndPnt[1];
                  double dT0, dT1;
                  if ( smgu_LineLineClosestPoint( sCirclePtTan[0], sCirNorm,
                          sFndPnt[0], sCrvNorm, dT0, dT1 ) == SM_SUCCESS )
                  {
                  
                      SmPoint3d sCent0 = sCirclePtTan[0] + dT0 * sCirNorm;
                      SmPoint3d sCent1 = sFndPnt     [0] + dT1 * sCrvNorm;
                      // Note: this is also called in 3d.  It's a 2d function,
                      // and in 3d, these could be different (although in 2d
                      // they should be the same):
                      // SM_ASSERT( sCent0.CloserThan( SM_EFF_ZERO_SQRT, sCent1 ));
                      SmCircle * pCircle = NULL;
                      double dRad = sCent1.DistanceBetween( sFndPnt[0] );
                      // Everything has to be orthonormal for SmAxis2 c'tor.
                      SmVector3d sX, sY, sZ;
                      sCirclePtTan[1].MakeUnitOrthoVectors( &sFndPnt[1], sX, sY, sZ );
                      SmAxis2Placement sOrigin( sCent1, sX, sY );
                      SmCircle::CreateCanonical( *( pNurb1->GetContext() ),
                          sOrigin, dRad, pCircle );
                      smgfx_SetLook( 1,1, 1,0,1 ); pCircle->Draw(); sm_GraphicsLoop();
                      sm_GraphicsLoop();
                      delete pCircle;
                  }
              }
              sm_GraphicsLoop() ;
#endif
            }
          my_draw_crv_pnt_sol(pNurb1, NULL, sSolutions, &sCombPoints, &sCombVecs);
        }
    }

  // all done  
  return SM_SUCCESS;

} // end my_test_one_curve_prop

/***********************************************************************
PURPOSE ---

USAGE NOTES ---
***********************************************************************/
PT_EXPORT SmStatus my_test_curve_properties(void)
{
    my_test_creation();
    //    SmContext sContext(pPool);
    SmContext sContext;
    SmTArray<SmPoint3d> sCombPoints, sCombVecs ;

    if (TRUE)
    {
        // Test tangency situation for circle 2 pt case
        double dD = -0.99999;
        SmVector3d sPlaneNorm(0, 1, 0);
        SmSolutionArray sSolutions;
        SmBSplineCurve *pCir = my_create_circle(sContext, 1.0, SmPoint3d(0, 0, 0), SM_CO_QUINTIC, 0, 1, 1);
        SmObjDelete sCleanup(pCir);
        SER(pCir->GlobalPropertyAnalysis(pCir->GetNaturalInterval(),
            SM_CP_PLANE_INTERSECTION, &dD, &sPlaneNorm, SM_ZONE_TOL_3D, 
            sSolutions,   // out: all solutions
            &sCombPoints, // out: check comb(to view property being zeroed): curve sample points (to view property being zeroed)                                  
            &sCombVecs)); // out: check comb(to view property being zeroed): tines - lengths proportional to property being zeroed.
        my_draw_crv_pnt_sol(pCir, NULL, sSolutions, &sCombPoints, &sCombVecs);
        SM_ASSERT(sSolutions.GetSize() == 2) ;
    }

    if (TRUE)
    {
        // Test a 2D nurb
        SmTArray < SmPoint3d> sPnts1;
        sPnts1.Add(SmPoint3d(-1, 0.5, 0 ));
        sPnts1.Add(SmPoint3d( 0, 0.5, 0 ));
        sPnts1.Add(SmPoint3d( 1, 1.0, 0 ));
        sPnts1.Add(SmPoint3d( 0, 1.5, 0 ));
        sPnts1.Add(SmPoint3d(-1, 2.0, 0 ));
        sPnts1.Add(SmPoint3d( 1, 4.5, 0 ));
        sPnts1.Add(SmPoint3d( 0, 3.5, 0 ));
        sPnts1.Add(SmPoint3d(-1, 4.0, 0 ));
        sPnts1.Add(SmPoint3d( 1, 5.0, 0 ));
        sPnts1.Add(SmPoint3d(-1, 6.0, 0 ));
        sPnts1.Add(SmPoint3d( 0, 6.5, 0 ));
        sPnts1.Add(SmPoint3d( 1, 7.0, 0 ));
        sPnts1.Add(SmPoint3d( 0, 7.5, 0 ));
        sPnts1.Add(SmPoint3d(-1, 8.0, 0 ));
        sPnts1.Add(SmPoint3d( 0, 8.5, 0 ));
        sPnts1.Add(SmPoint3d( 1, 9.0, 0 ));
        sPnts1.Add(SmPoint3d(-1, 9.5, 0 ));
        SmBSplineCurve *pNurb1 = my_create_nurb(sContext, SmPoint3d(0, - 5, 0), sPnts1, 3, 1.0, 0.5, 0);
        SmObjDelete sCleanup2(pNurb1);

#ifdef SM_DEBUG_CODE
        static SmBoolean MyDebug = FALSE;
        if (MyDebug)
        {
            smgfx_ZoomOut();
            pNurb1->Draw();
        }
#endif
        ULONG lTotalFound;
        my_test_one_curve_prop(pNurb1, SmVector3d(2.5, 0, 0), lTotalFound);
        SM_ASSERT(lTotalFound == 124);  // bd: I reviewed these quite thoroughly
                                        // recently, and 118 should be correct.   (3/23/09)
                                        // Then add 6 for new tan-to-circle test. (3/25/09)
    }

    if (TRUE)
    {
        // Test a 3D NUrb
        SmTArray < SmPoint3d> sPnts1;
        sPnts1.Add(SmPoint3d(-1, 0.5,  0.0 ));
        sPnts1.Add(SmPoint3d( 0, 0.5,  0.5 ));
        sPnts1.Add(SmPoint3d( 1, 1.0,  1.0 ));
        sPnts1.Add(SmPoint3d( 0, 1.5,  0.0 ));
        sPnts1.Add(SmPoint3d(-1, 2.0, -0.5 ));
        sPnts1.Add(SmPoint3d( 1, 4.5, -1.2 ));
        sPnts1.Add(SmPoint3d( 0, 3.5, -0.3 ));
        sPnts1.Add(SmPoint3d(-1, 4.0,  0.8 ));
        sPnts1.Add(SmPoint3d( 1, 5.0,  0.0 ));
        sPnts1.Add(SmPoint3d(-1, 6.0,  0.4 ));
        sPnts1.Add(SmPoint3d( 0, 6.5, -0.3 ));
        sPnts1.Add(SmPoint3d( 1, 7.0,  1.0 ));
        sPnts1.Add(SmPoint3d( 0, 7.5, -1.0 ));
        sPnts1.Add(SmPoint3d(-1, 8.0, -2.0 ));
        sPnts1.Add(SmPoint3d( 0, 8.5,  0.0 ));
        sPnts1.Add(SmPoint3d( 1, 9.0,  0.0 ));
        sPnts1.Add(SmPoint3d(-1, 9.5,  2.0 ));
        SmBSplineCurve *pNurb1 = my_create_nurb(sContext, SmPoint3d(4, - 5, 0), sPnts1, 3, 1.0, 0.5, 0);
        SmObjDelete sCleanup3(pNurb1);

#ifdef SM_GFX_CODE
        if (smGet_DoGraphics()) {
            pNurb1->Draw(); sm_GraphicsLoop();
        }
#endif
        ULONG lTotalFound;
        my_test_one_curve_prop(pNurb1, SmVector3d(2.5, 0, 0), lTotalFound);
        SM_ASSERT(lTotalFound == 113); // gwc: I reviewed this 3/14/09 (lTotalFound == 102) and can't find
                                       //      any glaring errors.
                                       // gwc: 10/15/10 plus 11 more for final new case (lTotalFound == 113)
                                       //      It finds sets of points where tangent and curvature values go through target values
                                       //        I did not review those cases to make sure that the solution set is complete.
                                       // Then add 11 for new tan-to-circle test. (3/25/09)
    }
    //    pPool->Free();
    //    delete pPool; pPool = NULL ;

    return SM_SUCCESS;
} // end my_test_curve_properties


/***********************************************************************
PURPOSE ---

USAGE NOTES ---
***********************************************************************/
PT_EXPORT SmStatus my_test_bsplinecurve(void)
{
    // First test the interface
    SE( my_test_curve_interface() );

    SER(my_test_curve_properties());
    
    SmContext sContext;
    
    
    SmTArray < SmPoint3d> sPnts;
    sPnts.Add(SmPoint3d(0, 0.1, 0));
    sPnts.Add(SmPoint3d(0.2, 0, 0));
    sPnts.Add(SmPoint3d(0.4, 0.2, - 0.1));
    sPnts.Add(SmPoint3d(0.6, - 0.3, 0.2));
    sPnts.Add(SmPoint3d(1.0, 0.0, 0.1));
    sPnts.Add(SmPoint3d(1.1, 0.1, 0));
    sPnts.Add(SmPoint3d(1.0, 0.5, 0));
    SmBSplineCurve *pNewBSC = my_create_nurb(sContext, SmPoint3d(0, 0, 0), sPnts, 3, 1, 0.5, 0);
    SmObjDelete sClean( pNewBSC );
    
#ifdef SM_GFX_CODE
    if (smGet_DoGraphics()) {
        pNewBSC->Draw(); sm_GraphicsLoop();
    }
#endif
    
    //    pPool->Free();
    //    delete pPool; pPool = NULL ;
    
    return SM_SUCCESS;
} // end my_test_bsplinecurve

/***********************************************************************
PURPOSE ---

USAGE NOTES ---
***********************************************************************/
PT_EXPORT SmStatus my_test_crv_angle_min()
{
    return SM_SUCCESS;
} // end my_test_crv_angle_min

/***********************************************************************
PURPOSE ---

USAGE NOTES ---
***********************************************************************/
PT_EXPORT SmStatus my_test_crv_directed_max()    
{
    SmContext sContext;
    
    if (TRUE)
    {
        SmTArray < SmPoint3d> sPnts1;
        sPnts1.Add(SmPoint3d(99.8664793682965382, 139.3514588568065733, 0.0000000000000000));            
        sPnts1.Add(SmPoint3d(305.5340100220061004, - 64.3607694381224604, 0.0000000000000000));
        sPnts1.Add(SmPoint3d(148.6970000000000312, 136.3246519931062437, 0.0000000000000000));
        SmBSplineCurve *pCir = my_create_nurb(sContext, SmPoint3d(0, 0, 0), sPnts1, 2, 0.0, 0.0, 0);
        SmObjDelete sCleanUp1(pCir);
        
        SmTArray < SmPoint3d> sPnts2;
        sPnts2.Add(SmPoint3d(135.3130000000000166, 209.5363552051969691, 0.0000000000000000));            
        sPnts2.Add(SmPoint3d(204.4563635326836675, 297.0223818265180853, 0.0000000000000000));
        sPnts2.Add(SmPoint3d(291.1623890210322543, 69.7446403153885228, 0.0000000000000000));
        sPnts2.Add(SmPoint3d(123.0024762346569105, 173.2553458007064648, 0.0000000000000000));
        SmBSplineCurve *pLine = my_create_nurb(sContext, SmPoint3d(0, 0, 0), sPnts2, 3, 0.0, 0.0, 0);
        SmObjDelete sCleanUp2(pLine);
        
        SmAxis2Placement sA2P;
        SmVector3d sX(1, 0.2, 0.9);
        sX.Unitize();
        SmVector3d sY(-0.2, 1, - 0.1);
        sY = sX * sY * sX;
        sY.Unitize();
        sA2P.SetCanonical(SmVector3d(0, 0, 0), sX, sY);
        //    pCir->Transform(sA2P);
        //    smgfx_SetColor(1,0,0);
        //    pCir->DrawWDeriv(pCir->GetNaturalInterval(),0);
        
        
        SmVector3d sDirNorm[3];
        sDirNorm[0] = SmVector3d(0, 0, 1);
        sDirNorm[1] = SmVector3d(0, 1, 0);
        
        SmSolutionArray sSolutions;
        SER(pCir->GlobalCurveSolve(pCir->GetNaturalInterval(),
            *pLine, pLine->GetNaturalInterval(),
            SM_SO_DIRECTED_MAXIMIZE, SM_ZONE_TOL_3D,
            NULL, sDirNorm,
            SM_SR_ALL,
            sSolutions));
        
        SM_ASSERT(sSolutions.GetSize() == 1);
        my_draw_2crv_sol(pCir, pLine, sSolutions);
    }
    
    if (TRUE)
    {
        //    SmBSplineCurve *pLine = my_create_line(sContext,SmPoint3d(-2,-2,0),SmPoint3d(-2,2,0),0.0,0.0,0.0);
        //    SmObjDelete sCU1(pLine);
        
        SmBSplineCurve *pLine = my_create_circle(sContext, 0.5, SmPoint3d(0.0, - 1.2, 1.0), SM_CO_QUADRATIC, 0, 1, 1);
        SmObjDelete sCU1(pLine);
        SmBSplineCurve *pCir = my_create_circle(sContext, 0.5, SmPoint3d(0, 0, 0), SM_CO_QUADRATIC, 0, 1, 1);
        SmObjDelete sCU2(pCir);
        
        SmVector3d sDirNorm[2];
        sDirNorm[0] = SmVector3d(0, 0, 1);
        sDirNorm[1] = SmVector3d(1, 3, 0);
        
        SmSolutionArray sSolutions;
        SER(pLine->GlobalCurveSolve(pLine->GetNaturalInterval(),
            *pCir, pCir->GetNaturalInterval(),
            SM_SO_DIRECTED_MAXIMIZE, SM_ZONE_TOL_3D, NULL, sDirNorm, SM_SR_ALL,
            sSolutions));
        
        SM_ASSERT(sSolutions.GetSize() == 1);
        my_draw_2crv_sol(pLine, pCir, sSolutions);
    }
    
    return SM_SUCCESS;
} // end my_test_crv_directed_max

/***********************************************************************
PURPOSE ---

USAGE NOTES ---
***********************************************************************/
PT_EXPORT SmStatus my_test_crv_directed_min()    
{
    SmContext sContext;
    
    
    if (TRUE)
    {
        SmTArray < SmPoint3d> sPnts1;
        sPnts1.Add(SmPoint3d(99.8664793682965382, 139.3514588568065733, 0.0000000000000000));            
        sPnts1.Add(SmPoint3d(305.5340100220061004, - 64.3607694381224604, 0.0000000000000000));
        sPnts1.Add(SmPoint3d(148.6970000000000312, 136.3246519931062437, 0.0000000000000000));
        SmBSplineCurve *pCir = my_create_nurb(sContext, SmPoint3d(0, 0, 0), sPnts1, 2, 0.0, 0.0, 0);
        SmObjDelete sCleanUp1(pCir);
        
        SmTArray < SmPoint3d> sPnts2;
        sPnts2.Add(SmPoint3d(135.3130000000000166, 209.5363552051969691, 0.0000000000000000));            
        sPnts2.Add(SmPoint3d(204.4563635326836675, 297.0223818265180853, 0.0000000000000000));
        sPnts2.Add(SmPoint3d(291.1623890210322543, 69.7446403153885228, 0.0000000000000000));
        sPnts2.Add(SmPoint3d(123.0024762346569105, 173.2553458007064648, 0.0000000000000000));
        SmBSplineCurve *pLine = my_create_nurb(sContext, SmPoint3d(0, 0, 0), sPnts2, 3, 0.0, 0.0, 0);
        SmObjDelete sCleanUp2(pLine);
        
        SmAxis2Placement sA2P;
        SmVector3d sX(1, 0.2, 0.9);
        sX.Unitize();
        SmVector3d sY(-0.2, 1, - 0.1);
        sY = sX * sY * sX;
        sY.Unitize();
        sA2P.SetCanonical(SmVector3d(0, 0, 0), sX, sY);
        //    pCir->Transform(sA2P);
        //    smgfx_SetColor(1,0,0);
        //    pCir->DrawWDeriv(pCir->GetNaturalInterval(),0);
        
        
        SmVector3d sDirNorm[3];
        sDirNorm[0] = SmVector3d(0, 0, 1);
        sDirNorm[1] = SmVector3d(0, 1, 0);
        
        SmSolutionArray sSolutions;
        SER(pCir->GlobalCurveSolve(pCir->GetNaturalInterval(),
            *pLine, pLine->GetNaturalInterval(),
            SM_SO_DIRECTED_MINIMIZE, SM_ZONE_TOL_3D,
            NULL, sDirNorm,
            SM_SR_ALL,
            sSolutions));
        
        SM_ASSERT(sSolutions.GetSize() == 1);
        my_draw_2crv_sol(pCir, pLine, sSolutions);
    }
    return SM_SUCCESS;
    
#if 0 
    if (TRUE)
    {
        SmBSplineCurve *pLine = my_create_line(sContext, SmPoint3d(-2, 0.2, 0.3), SmPoint3d(-2, 0.2, - 0.3), 0.0, 0.0, 0.0);
        SmObjDelete sCU1(pLine);
        
        //    SmBSplineCurve *pLine = my_create_circle(sContext,0.5,SmPoint3d(0.0,-1.2,1.0),SM_CO_QUADRATIC,0,1,1);
        //    SmObjDelete sCU1(pLine);
        SmBSplineCurve *pCir = my_create_circle(sContext, 0.5, SmPoint3d(0, 0, 0), SM_CO_QUADRATIC, 0, 1, 1);
        SmObjDelete sCU2(pCir);
        
        SmVector3d sDirNorm[2];
        sDirNorm[0] = SmVector3d(0, 0, 1);
        sDirNorm[1] = SmVector3d(1, 0, 0);
        
        SmSolutionArray sSolutions;
        SER(pLine->GlobalCurveSolve(pLine->GetNaturalInterval(),
            *pCir, pCir->GetNaturalInterval(),
            SM_SO_DIRECTED_MINIMIZE, SM_ZONE_TOL_3D, NULL, sDirNorm, SM_SR_ALL,
            sSolutions));
        
        SM_ASSERT(sSolutions.GetSize() == 2);
        my_draw_2crv_sol(pLine, pCir, sSolutions);
    }
    
    if (TRUE)
    {
        //    SmBSplineCurve *pLine = my_create_line(sContext,SmPoint3d(-2,-2,0),SmPoint3d(-2,2,0),0.0,0.0,0.0);
        //    SmObjDelete sCU1(pLine);
        
        SmBSplineCurve *pLine = my_create_circle(sContext, 0.5, SmPoint3d(0.0, - 1.2, 0), SM_CO_QUADRATIC, 0, 1, 1);
        SmObjDelete sCU1(pLine);
        SmBSplineCurve *pCir = my_create_circle(sContext, 0.5, SmPoint3d(0, 0, 0), SM_CO_QUADRATIC, 0, 1, 1);
        SmObjDelete sCU2(pCir);
        
        SmVector3d sDirNorm[2];
        sDirNorm[0] = SmVector3d(0, 0, 1);
        sDirNorm[1] = SmVector3d(1, 3, 0);
        
        SmSolutionArray sSolutions;
        SER(pLine->GlobalCurveSolve(pLine->GetNaturalInterval(),
            *pCir, pCir->GetNaturalInterval(),
            SM_SO_DIRECTED_MINIMIZE, SM_ZONE_TOL_3D, NULL, sDirNorm, SM_SR_ALL,
            sSolutions));
        
        SM_ASSERT(sSolutions.GetSize() == 1);
        my_draw_2crv_sol(pLine, pCir, sSolutions);
    }
    
    if (TRUE)
    {
        // FORD
        SmTArray < SmPoint3d> sPnts1;
        sPnts1.Add(SmPoint3d(-37.9543531295402019, 165.5771033883733878, 0.0000000000000000));            
        sPnts1.Add(SmPoint3d(-4.3674041451923902, 303.3005867526989618, 0.0000000000000000));
        sPnts1.Add(SmPoint3d(62.1974470317728887, 145.8029999999999973, 0.0000000000000000));
        
        SmTArray < SmPoint3d> sPnts2;
        sPnts2.Add(SmPoint3d(-54.0231676959287626, 219.9939999999999998, 0.0000000000000000));            
        sPnts2.Add(SmPoint3d(14.0788088965479208, 113.8559514297990347, 0.0000000000000000));
        sPnts2.Add(SmPoint3d(62.1974470317728887, 248.3328843002657322, 0.0000000000000000));
        
        SmBSplineCurve *pLine = my_create_nurb(sContext, SmPoint3d(0, 0, 0), sPnts1, 2, 0.0, 0.0, 0);
        SmObjDelete sCleanUp1(pLine);
        SmBSplineCurve *pCir = my_create_nurb(sContext, SmPoint3d(0, 0, 0), sPnts2, 2, 0.0, 0.0, 0);
        SmObjDelete sCleanUp2(pCir);
        
        SmVector3d sDirNorm[3];
        sDirNorm[0] = SmVector3d(0, 0, 1);
        sDirNorm[1] = SmVector3d(0, 1, 0);
        
        SmSolutionArray sSolutions;
        SER(pCir->GlobalCurveSolve(pCir->GetNaturalInterval(),
            *pLine, pLine->GetNaturalInterval(),
            SM_SO_DIRECTED_MINIMIZE, SM_ZONE_TOL_3D,
            NULL, sDirNorm,
            SM_SR_ALL,
            sSolutions));
        
        SM_ASSERT(sSolutions.GetSize() == 1);
        my_draw_2crv_sol(pCir, pLine, sSolutions);
    }
    
    return SM_SUCCESS;
#endif // unreachable code

} // end my_test_crv_directed_min

/***********************************************************************
PURPOSE ---

USAGE NOTES ---
***********************************************************************/
PT_EXPORT SmStatus my_test_crv_projected_max()    
{
    SmContext sContext;
    
    return SM_SUCCESS;
} // end my_test_crv_projected_max

/***********************************************************************
PURPOSE ---

USAGE NOTES ---
***********************************************************************/
PT_EXPORT SmStatus my_test_crv_projected_min()    
{
    SmContext sContext;
    
    if (TRUE)
    {
        SmTArray < SmPoint3d> sPnts1;
        sPnts1.Add(SmPoint3d(289.7255043107365395, - 239.7477614133240422, - 122.2882013167549076));            
        sPnts1.Add(SmPoint3d(277.8708884790188449, - 251.6023772450417653, - 93.0322316736942554));            
        sPnts1.Add(SmPoint3d(306.7514880042527921, - 222.7217777198077613, - 92.3420624412881210));            
        sPnts1.Add(SmPoint3d(318.0770598211955189, - 211.3962059028650629, - 164.4933655470701126));            
        sPnts1.Add(SmPoint3d(321.1817003977853346, - 208.2915653262752471, - 31.4939960752873915));            
        sPnts1.Add(SmPoint3d(334.3774797641247005, - 195.0957859599358528, - 130.0000000000000000));            
        SmBSplineCurve *pCir = my_create_nurb(sContext, SmPoint3d(0, 0, 0), sPnts1, 2, 0.0, 0.0, 0);
        SmObjDelete sCleanUp1(pCir);
        
        SmTArray < SmPoint3d> sPnts2;
        sPnts2.Add(SmPoint3d(299.7255043107365395, - 229.7477614133240991, - 12.2882013167549129));            
        sPnts2.Add(SmPoint3d(282.9605458247985439, - 229.7477614133240991, 16.9677683263057411));            
        sPnts2.Add(SmPoint3d(323.8038813628504045, - 229.7477614133240991, 17.6579375587118825));            
        sPnts2.Add(SmPoint3d(339.8206586280012402, - 229.7477614133240991, - 54.4933655470701126));            
        sPnts2.Add(SmPoint3d(344.2112834377084027, - 229.7477614133240991, 78.5060039247126156));            
        sPnts2.Add(SmPoint3d(362.8729335836686118, - 229.7477614133240991, - 20.0000000000000036));            
        SmBSplineCurve *pLine = my_create_nurb(sContext, SmPoint3d(0, 0, 0), sPnts2, 3, 0.0, 0.0, 0);
        SmObjDelete sCleanUp2(pLine);
        
        SmAxis2Placement sA2P;
        SmVector3d sX(1, 0.2, 0.9);
        sX.Unitize();
        SmVector3d sY(-0.2, 1, - 0.1);
        sY = sX * sY * sX;
        sY.Unitize();
        sA2P.SetCanonical(SmVector3d(0, 0, 0), sX, sY);
        //    pCir->Transform(sA2P);
        //    smgfx_SetColor(1,0,0);
        //    pCir->DrawWDeriv(pCir->GetNaturalInterval(),0);
        
        
        SmVector3d sDirNorm[3];
        sDirNorm[0] = SmVector3d(0, 0, 1);
        //    sDirNorm[1] = SmVector3d(0,0,1);
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  // draw 
        if(bDebugMe)
          {
            SmPoint3d sBasePt ;
            pCir->EvaluatePoint(.5, sBasePt) ;

            smgfx_Erase() ;
            smgfx_SetLook(1,2, 0,0,1) ; if(pCir) pCir->Draw(NULL, TRUE) ; sm_GraphicsLoop() ;
            smgfx_SetLook(1,2, 0,1,1) ; if(pLine) pLine->Draw(NULL, TRUE) ; sm_GraphicsLoop() ;
            smgfx_SetLook(3,4, 0,0,0) ; (30*sDirNorm[0]).Draw(&sBasePt) ; sm_GraphicsLoop() ;
            sm_GraphicsLoop() ;
          }       

#endif // SM_DEBUG_CODE

        SmSolutionArray sSolutions;
        SER(pCir->GlobalCurveSolve(pCir->GetNaturalInterval(),
                                  *pLine, 
                                   pLine->GetNaturalInterval(),
                                   SM_SO_PROJECTED_MINIMIZE, 
                                   SM_ZONE_TOL_3D,
                                   NULL, 
                                   sDirNorm,
                                   SM_SR_ALL,
                                   sSolutions));
        
#ifdef SM_DEBUG_CODE
  // draw 
        if(bDebugMe)
          {
            sSolutions.Dump() ;
#ifdef SM_GFX_CODE
            SmPoint3d sBasePt ;
            pCir->EvaluatePoint(.5, sBasePt) ;

            SmPoint3d sPtObj1, sPtObj2 ;
            if(sSolutions.GetSize() > 0)
              {
                sSolutions[0].GetPoint(0, sPtObj1) ;
                sSolutions[0].GetPoint(1, sPtObj2) ; 
              }
            smgfx_Erase() ;
            smgfx_SetLook(1,2, 0,0,1) ; if(pCir) pCir->Draw(NULL, TRUE) ; sm_GraphicsLoop() ;
            smgfx_SetLook(1,2, 0,1,1) ; if(pLine) pLine->Draw(NULL, TRUE) ; sm_GraphicsLoop() ;
            smgfx_SetLook(3,4, 0,0,0) ; (30*sDirNorm[0]).Draw(&sBasePt) ; sm_GraphicsLoop() ;
            smgfx_SetLook(2,3, 1,0,0) ; sSolutions.Draw() ; sm_GraphicsLoop() ;
            smgfx_SetLook(1,4, 0,0,0) ; if(sSolutions.GetSize() > 0) { sPtObj1.DrawPointToPoint(sPtObj2) ; } sm_GraphicsLoop() ;
            sm_GraphicsLoop() ;
#endif
          }
#endif // SM_DEBUG_CODE

        SM_ASSERT(sSolutions.GetSize() == 2);
        my_draw_2crv_sol(pCir, pLine, sSolutions);
    }
    
    if (TRUE)
    {
        SmBSplineCurve *pCir = my_create_circle(sContext, 0.5, SmPoint3d(0, 0, 0), SM_CO_QUADRATIC, 0, 1, 1);
        SmObjDelete sCU2(pCir);
        SmBSplineCurve *pLine = my_create_circle(sContext, 0.5, SmPoint3d(0.5, - 1.5, 1.0), SM_CO_QUADRATIC, 0, 1, 1);
        SmObjDelete sCU1(pLine);
        SmAxis2Placement sA2P;
        SmVector3d sX(1, 0.2, 0.9);
        sX.Unitize();
        SmVector3d sY(-0.2, 1, - 0.1);
        sY = sX * sY * sX;
        sY.Unitize();
        sA2P.SetCanonical(SmVector3d(0, 0, 0), sX, sY);
        pCir->Transform(sA2P);
#ifdef SM_GFX_CODE
        if(smGet_DoGraphics()){
        smgfx_SetColor(1, 0, 0);
        pCir->DrawWDeriv(pCir->GetNaturalInterval(), 0);
          }
#endif
        
        
        SmVector3d sDirNorm[3];
        sDirNorm[0] = SmVector3d(0, 0, 1);
        
        SmSolutionArray sSolutions;
        SER(pCir->GlobalCurveSolve(pCir->GetNaturalInterval(),
            *pLine, pLine->GetNaturalInterval(),
            SM_SO_PROJECTED_MINIMIZE, SM_ZONE_TOL_3D,
            NULL, sDirNorm,
            SM_SR_ALL,
            sSolutions));
        
        SM_ASSERT(sSolutions.GetSize() == 1);
        my_draw_2crv_sol(pCir, pLine, sSolutions);
    }
    
    return SM_SUCCESS;
} // end my_test_crv_projected_min

/***********************************************************************
PURPOSE ---

USAGE NOTES ---
***********************************************************************/
PT_EXPORT SmStatus my_test_crv_projected_tangency()    
{
    SmContext sContext;
    
    if (TRUE)
    {
        SmBSplineCurve *pCir = my_create_circle(sContext, 0.5, SmPoint3d(0, 0, 0), SM_CO_QUADRATIC, 0, 1, 1);
        SmObjDelete sCU2(pCir);
        SmBSplineCurve *pLine = my_create_circle(sContext, 0.5, SmPoint3d(0.5, - 1.5, 1.0), SM_CO_QUADRATIC, 0, 1, 1);
        SmObjDelete sCU1(pLine);
        SmAxis2Placement sA2P;
        SmVector3d sX(1, 0.2, 0.9);
        sX.Unitize();
        SmVector3d sY(-0.2, 1, - 0.1);
        sY = sX * sY * sX;
        sY.Unitize();
        sA2P.SetCanonical(SmVector3d(0, 0, 0), sX, sY);
        pCir->Transform(sA2P);
        if(smGet_DoGraphics()){
#ifdef SM_GFX_CODE
        smgfx_SetColor(1, 0, 0);
#endif
        pCir->DrawWDeriv(pCir->GetNaturalInterval(), 0);
          }
        
        
        SmVector3d sDirNorm[3];
        sDirNorm[0] = SmVector3d(0, 0, 1);
        
        SmSolutionArray sSolutions;
        SER(pCir->GlobalCurveSolve(pCir->GetNaturalInterval(),
            *pLine, pLine->GetNaturalInterval(),
            SM_SO_PROJECTED_TANGENCY, SM_ZONE_TOL_3D,
            NULL, sDirNorm,
            SM_SR_ALL,
            sSolutions));
        
        SM_ASSERT(sSolutions.GetSize() == 4);
        my_draw_2crv_sol(pCir, pLine, sSolutions);
    }
    
    return SM_SUCCESS;
} // end my_test_crv_projected_tangency

/***********************************************************************
PURPOSE ---

USAGE NOTES ---
***********************************************************************/
PT_EXPORT SmStatus my_test_crv_signed_angle_min()        
{
    SmContext sContext;
    
    if (TRUE)
    {
        SmBSplineCurve *pCir = my_create_line(sContext, SmPoint3d(-2, 0.5, 0.5), SmPoint3d(-2, 1.5, 0), 0.0, 0.0, 0.0);
        SmObjDelete sCU2(pCir);
        SmBSplineCurve *pLine = my_create_circle(sContext, 0.5, SmPoint3d(-4.0, 3.2, 1.0), SM_CO_QUADRATIC, 0, 1, 1);
        SmObjDelete sCU1(pLine);
        
        SmVector3d sDirNorm[3];
        sDirNorm[0] = SmVector3d(0, 0, 1);
        sDirNorm[1] = SmVector3d(1, 0, 0);
        
        SmSolutionArray sSolutions;
        SER(pCir->GlobalCurveSolve(pCir->GetNaturalInterval(),
            *pLine, pLine->GetNaturalInterval(),
            SM_SO_SIGNED_ANGLE_MINIMIZE, SM_ZONE_TOL_3D,
            NULL, sDirNorm,
            SM_SR_ALL,
            sSolutions));
        
        SM_ASSERT(sSolutions.GetSize() == 1);
        my_draw_2crv_sol(pCir, pLine, sSolutions);
    }
    
    if (TRUE)
    {
        SmBSplineCurve *pCir = my_create_circle(sContext, 0.8, SmPoint3d(-2, 1, 0.5), SM_CO_QUADRATIC, 0, 1, 1);
        SmObjDelete sCU2(pCir);
        SmBSplineCurve *pLine = my_create_circle(sContext, 0.5, SmPoint3d(0.0, 1.2, 1.0), SM_CO_QUADRATIC, 0, 1, 1);
        SmObjDelete sCU1(pLine);
        
        SmVector3d sDirNorm[3];
        sDirNorm[0] = SmVector3d(0, 0, 1);
        sDirNorm[1] = SmVector3d(1, 0, 0);
        
        SmSolutionArray sSolutions;
        SER(pCir->GlobalCurveSolve(pCir->GetNaturalInterval(),
            *pLine, pLine->GetNaturalInterval(),
            SM_SO_SIGNED_ANGLE_MINIMIZE, SM_ZONE_TOL_3D,
            NULL, sDirNorm,
            SM_SR_ALL,
            sSolutions));
        
        SM_ASSERT(sSolutions.GetSize() == 1);
        my_draw_2crv_sol(pCir, pLine, sSolutions);
    }
    
    if (TRUE)
    {
        SmBSplineCurve *pCir = my_create_line(sContext, SmPoint3d(-2, 0.5, 0.5), SmPoint3d(-2, 1.5, 0), 0.0, 0.0, 0.0);
        SmObjDelete sCU2(pCir);
        SmBSplineCurve *pLine = my_create_circle(sContext, 0.5, SmPoint3d(0.0, - 1.2, 1.0), SM_CO_QUADRATIC, 0, 1, 1);
        SmObjDelete sCU1(pLine);
        
        SmVector3d sDirNorm[3];
        sDirNorm[0] = SmVector3d(0, 0, 1);
        sDirNorm[1] = SmVector3d(1, 0, 0);
        
        SmSolutionArray sSolutions;
        SER(pCir->GlobalCurveSolve(pCir->GetNaturalInterval(),
            *pLine, pLine->GetNaturalInterval(),
            SM_SO_SIGNED_ANGLE_MINIMIZE, SM_ZONE_TOL_3D,
            NULL, sDirNorm,
            SM_SR_ALL,
            sSolutions));
        
        SM_ASSERT(sSolutions.GetSize() == 1);
        my_draw_2crv_sol(pCir, pLine, sSolutions);
    }
    
    if (TRUE)
    {
        SmBSplineCurve *pCir = my_create_line(sContext, SmPoint3d(-2, 0.5, 0.5), SmPoint3d(-2, 1.5, 0), 0.0, 0.0, 0.0);
        SmObjDelete sCU2(pCir);
        SmBSplineCurve *pLine = my_create_circle(sContext, 0.5, SmPoint3d(0.0, 3.2, 1.0), SM_CO_QUADRATIC, 0, 1, 1);
        SmObjDelete sCU1(pLine);
        
        SmVector3d sDirNorm[3];
        sDirNorm[0] = SmVector3d(0, 0, 1);
        sDirNorm[1] = SmVector3d(1, 0, 0);
        
        SmSolutionArray sSolutions;
        SER(pCir->GlobalCurveSolve(pCir->GetNaturalInterval(),
            *pLine, pLine->GetNaturalInterval(),
            SM_SO_SIGNED_ANGLE_MINIMIZE, SM_ZONE_TOL_3D,
            NULL, sDirNorm,
            SM_SR_ALL,
            sSolutions));
        
        SM_ASSERT(sSolutions.GetSize() == 1);
        my_draw_2crv_sol(pCir, pLine, sSolutions);
    }
    
    if (TRUE)
    {
        SmBSplineCurve *pCir = my_create_line(sContext, SmPoint3d(-2, 0.5, 0.5), SmPoint3d(-2, 1.5, 0), 0.0, 0.0, 0.0);
        SmObjDelete sCU2(pCir);
        SmBSplineCurve *pLine = my_create_circle(sContext, 0.5, SmPoint3d(-4.0, - 1.2, 1.0), SM_CO_QUADRATIC, 0, 1, 1);
        SmObjDelete sCU1(pLine);
        
        SmVector3d sDirNorm[3];
        sDirNorm[0] = SmVector3d(0, 0, 1);
        sDirNorm[1] = SmVector3d(1, 0, 0);
        
        SmSolutionArray sSolutions;
        SER(pCir->GlobalCurveSolve(pCir->GetNaturalInterval(),
            *pLine, pLine->GetNaturalInterval(),
            SM_SO_SIGNED_ANGLE_MINIMIZE, SM_ZONE_TOL_3D,
            NULL, sDirNorm,
            SM_SR_ALL,
            sSolutions));
        
        SM_ASSERT(sSolutions.GetSize() == 1);
        my_draw_2crv_sol(pCir, pLine, sSolutions);
    }
    
    if (TRUE)
    {
        SmBSplineCurve *pCir = my_create_line(sContext, SmPoint3d(-2, 0.5, 0.5), SmPoint3d(-2, 1.5, 0), 0.0, 0.0, 0.0);
        SmObjDelete sCU2(pCir);
        SmBSplineCurve *pLine = my_create_circle(sContext, 0.5, SmPoint3d(-4.0, 1.2, 1.0), SM_CO_QUADRATIC, 0, 1, 1);
        SmObjDelete sCU1(pLine);
        
        SmVector3d sDirNorm[3];
        sDirNorm[0] = SmVector3d(0, 0, 1);
        sDirNorm[1] = SmVector3d(1, 0, 0);
        
        SmSolutionArray sSolutions;
        SER(pCir->GlobalCurveSolve(pCir->GetNaturalInterval(),
                                   *pLine, pLine->GetNaturalInterval(),
                                   SM_SO_SIGNED_ANGLE_MINIMIZE, SM_ZONE_TOL_3D,
                                   NULL, sDirNorm,
                                   SM_SR_ALL,
                                   sSolutions));
        
        SM_ASSERT(sSolutions.GetSize() == 2); // gwc: used to be 1 - but the sol happens at the closed curve join - 2 seems reasonable
        my_draw_2crv_sol(pCir, pLine, sSolutions);
    }
    
    if (TRUE)
    {
        SmBSplineCurve *pCir = my_create_circle(sContext, 0.8, SmPoint3d(-2, 1, 0.5), SM_CO_QUADRATIC, 0, 1, 1);
        SmObjDelete sCU2(pCir);
        SmBSplineCurve *pLine = my_create_circle(sContext, 0.5, SmPoint3d(0.0, - 1.2, 1.0), SM_CO_QUADRATIC, 0, 1, 1);
        SmObjDelete sCU1(pLine);
        
        SmVector3d sDirNorm[3];
        sDirNorm[0] = SmVector3d(0, 0, 1);
        sDirNorm[1] = SmVector3d(1, 0, 0);
        
        SmSolutionArray sSolutions;
        SER(pCir->GlobalCurveSolve(pCir->GetNaturalInterval(),
            *pLine, pLine->GetNaturalInterval(),
            SM_SO_SIGNED_ANGLE_MINIMIZE, SM_ZONE_TOL_3D,
            NULL, sDirNorm,
            SM_SR_ALL,
            sSolutions));
        
        SM_ASSERT(sSolutions.GetSize() == 1);
        my_draw_2crv_sol(pCir, pLine, sSolutions);
    }
    
    if (TRUE)
    {
        SmBSplineCurve *pCir = my_create_circle(sContext, 0.8, SmPoint3d(-2, 1, 0.5), SM_CO_QUADRATIC, 0, 1, 1);
        SmObjDelete sCU2(pCir);
        SmBSplineCurve *pLine = my_create_circle(sContext, 0.5, SmPoint3d(0.0, 3.2, 1.0), SM_CO_QUADRATIC, 0, 1, 1);
        SmObjDelete sCU1(pLine);
        
        SmVector3d sDirNorm[3];
        sDirNorm[0] = SmVector3d(0, 0, 1);
        sDirNorm[1] = SmVector3d(1, 0, 0);
        
        SmSolutionArray sSolutions;
        SER(pCir->GlobalCurveSolve(pCir->GetNaturalInterval(),
            *pLine, pLine->GetNaturalInterval(),
            SM_SO_SIGNED_ANGLE_MINIMIZE, SM_ZONE_TOL_3D,
            NULL, sDirNorm,
            SM_SR_ALL,
            sSolutions));
        
        SM_ASSERT(sSolutions.GetSize() == 1);
        my_draw_2crv_sol(pCir, pLine, sSolutions);
    }
    
    if (TRUE)
    {
        SmBSplineCurve *pCir = my_create_circle(sContext, 0.8, SmPoint3d(-2, 1, 0.5), SM_CO_QUADRATIC, 0, 1, 1);
        SmObjDelete sCU2(pCir);
        SmBSplineCurve *pLine = my_create_circle(sContext, 0.5, SmPoint3d(-4.0, - 1.2, 1.0), SM_CO_QUADRATIC, 0, 1, 1);
        SmObjDelete sCU1(pLine);
        
        SmVector3d sDirNorm[3];
        sDirNorm[0] = SmVector3d(0, 0, 1);
        sDirNorm[1] = SmVector3d(1, 0, 0);
        
        SmSolutionArray sSolutions;
        SER(pCir->GlobalCurveSolve(pCir->GetNaturalInterval(),
            *pLine, pLine->GetNaturalInterval(),
            SM_SO_SIGNED_ANGLE_MINIMIZE, SM_ZONE_TOL_3D,
            NULL, sDirNorm,
            SM_SR_ALL,
            sSolutions));
        
        SM_ASSERT(sSolutions.GetSize() == 1);
        my_draw_2crv_sol(pCir, pLine, sSolutions);
    }
    
    if (TRUE)
    {
        SmBSplineCurve *pCir = my_create_circle(sContext, 0.8, SmPoint3d(-2, 1, 0.5), SM_CO_QUADRATIC, 0, 1, 1);
        SmObjDelete sCU2(pCir);
        SmBSplineCurve *pLine = my_create_circle(sContext, 0.5, SmPoint3d(-4.0, 1.2, 1.0), SM_CO_QUADRATIC, 0, 1, 1);
        SmObjDelete sCU1(pLine);
        
        SmVector3d sDirNorm[3];
        sDirNorm[0] = SmVector3d(0, 0, 1);
        sDirNorm[1] = SmVector3d(1, 0, 0);
        
        SmSolutionArray sSolutions;
        SER(pCir->GlobalCurveSolve(pCir->GetNaturalInterval(),
            *pLine, pLine->GetNaturalInterval(),
            SM_SO_SIGNED_ANGLE_MINIMIZE, SM_ZONE_TOL_3D,
            NULL, sDirNorm,
            SM_SR_ALL,
            sSolutions));
        
        SM_ASSERT(sSolutions.GetSize() == 1);
        my_draw_2crv_sol(pCir, pLine, sSolutions);
    }
    
    if (TRUE)
    {
        SmBSplineCurve *pCir = my_create_circle(sContext, 0.8, SmPoint3d(-2, 1, 0.5), SM_CO_QUADRATIC, 0, 1, 1);
        SmObjDelete sCU2(pCir);
        SmBSplineCurve *pLine = my_create_circle(sContext, 0.5, SmPoint3d(-4.0, 3.2, 1.0), SM_CO_QUADRATIC, 0, 1, 1);
        SmObjDelete sCU1(pLine);
        
        SmVector3d sDirNorm[3];
        sDirNorm[0] = SmVector3d(0, 0, 1);
        sDirNorm[1] = SmVector3d(1, 0, 0);
        
        SmSolutionArray sSolutions;
        SER(pCir->GlobalCurveSolve(pCir->GetNaturalInterval(),
            *pLine, pLine->GetNaturalInterval(),
            SM_SO_SIGNED_ANGLE_MINIMIZE, SM_ZONE_TOL_3D,
            NULL, sDirNorm,
            SM_SR_ALL,
            sSolutions));
        
        SM_ASSERT(sSolutions.GetSize() == 1);
        my_draw_2crv_sol(pCir, pLine, sSolutions);
    }
    
    return SM_SUCCESS;
} // end my_test_crv_signed_angle_min

/***********************************************************************
PURPOSE ---

USAGE NOTES ---
***********************************************************************/
PT_EXPORT SmStatus my_test_crv_signed_directed_min()    
{
    SmContext sContext;
    
    if (TRUE)
    {
        SmTArray < SmPoint3d> sPnts1;
        sPnts1.Add(SmPoint3d(99.8664793682965382, 139.3514588568065733, 0.0000000000000000));            
        sPnts1.Add(SmPoint3d(305.5340100220061004, - 64.3607694381224604, 0.0000000000000000));
        sPnts1.Add(SmPoint3d(148.6970000000000312, 136.3246519931062437, 0.0000000000000000));
        SmBSplineCurve *pCir = my_create_nurb(sContext, SmPoint3d(0, 0, 0), sPnts1, 2, 0.0, 0.0, 0);
        SmObjDelete sCleanUp1(pCir);
        
        SmTArray < SmPoint3d> sPnts2;
        sPnts2.Add(SmPoint3d(135.3130000000000166, 209.5363552051969691, 0.0000000000000000));            
        sPnts2.Add(SmPoint3d(204.4563635326836675, 297.0223818265180853, 0.0000000000000000));
        sPnts2.Add(SmPoint3d(291.1623890210322543, 69.7446403153885228, 0.0000000000000000));
        sPnts2.Add(SmPoint3d(123.0024762346569105, 173.2553458007064648, 0.0000000000000000));
        SmBSplineCurve *pLine = my_create_nurb(sContext, SmPoint3d(0, 0, 0), sPnts2, 3, 0.0, 0.0, 0);
        SmObjDelete sCleanUp2(pLine);
        
        SmAxis2Placement sA2P;
        SmVector3d sX(1, 0.2, 0.9);
        sX.Unitize();
        SmVector3d sY(-0.2, 1, - 0.1);
        sY = sX * sY * sX;
        sY.Unitize();
        sA2P.SetCanonical(SmVector3d(0, 0, 0), sX, sY);
        //    pCir->Transform(sA2P);
        //    smgfx_SetColor(1,0,0);
        //    pCir->DrawWDeriv(pCir->GetNaturalInterval(),0);
        
        
        SmVector3d sDirNorm[3];
        sDirNorm[0] = SmVector3d(0, 0, 1);
        sDirNorm[1] = SmVector3d(0, 1, 0);
        
        SmSolutionArray sSolutions;
        SER(pCir->GlobalCurveSolve(pCir->GetNaturalInterval(),
            *pLine, pLine->GetNaturalInterval(),
            SM_SO_SIGNED_DIRECTED_MINIMIZE, SM_ZONE_TOL_3D,
            NULL, sDirNorm,
            SM_SR_ALL,
            sSolutions));
        
        SM_ASSERT(sSolutions.GetSize() == 1);
        my_draw_2crv_sol(pCir, pLine, sSolutions);
    }
    
    if (TRUE)
    {
        // FORD
        SmTArray < SmPoint3d> sPnts1;
        sPnts1.Add(SmPoint3d(-37.9543531295402019, 165.5771033883733878, 0.0000000000000000));            
        sPnts1.Add(SmPoint3d(-4.3674041451923902, 303.3005867526989618, 0.0000000000000000));
        sPnts1.Add(SmPoint3d(62.1974470317728887, 145.8029999999999973, 0.0000000000000000));
        
        SmTArray < SmPoint3d> sPnts2;
        sPnts2.Add(SmPoint3d(-54.0231676959287626, 219.9939999999999998, 0.0000000000000000));            
        sPnts2.Add(SmPoint3d(14.0788088965479208, 113.8559514297990347, 0.0000000000000000));
        sPnts2.Add(SmPoint3d(62.1974470317728887, 248.3328843002657322, 0.0000000000000000));
        
        SmBSplineCurve *pLine = my_create_nurb(sContext, SmPoint3d(0, 0, 0), sPnts1, 2, 0.0, 0.0, 0);
        SmObjDelete sCleanUp1(pLine);
        SmBSplineCurve *pCir = my_create_nurb(sContext, SmPoint3d(0, 0, 0), sPnts2, 2, 0.0, 0.0, 0);
        SmObjDelete sCleanUp2(pCir);
        
        SmVector3d sDirNorm[3];
        sDirNorm[0] = SmVector3d(0, 0, 1);
        sDirNorm[1] = SmVector3d(0, 1, 0);
        
        SmSolutionArray sSolutions;
        SER(pCir->GlobalCurveSolve(pCir->GetNaturalInterval(),
            *pLine, pLine->GetNaturalInterval(),
            SM_SO_SIGNED_DIRECTED_MINIMIZE, SM_ZONE_TOL_3D,
            NULL, sDirNorm,
            SM_SR_ALL,
            sSolutions));
        
        SM_ASSERT(sSolutions.GetSize() == 1);
        my_draw_2crv_sol(pCir, pLine, sSolutions);
    }
    
    return SM_SUCCESS;
} // end my_test_crv_signed_directed_min

/***********************************************************************
PURPOSE ---

USAGE NOTES ---
***********************************************************************/
PT_EXPORT SmStatus my_test_signed_pivot_min(void)
{
    SmContext sContext;
    
    if (TRUE)    // circ/line test
    {
        SmBSplineCurve *pCir1 = my_create_circle(sContext, 0.5, SmPoint3d(3, 1.5, 0), SM_CO_QUADRATIC, 0, 1, 1);
        SmObjDelete sCleanUp1(pCir1);
        SmPoint3d sStart(0, 0, 0);
        SmPoint3d sEnd(5, 0, 0);
        SmBSplineCurve *pLine = my_create_line(sContext, sStart, sEnd, 0.5, 0.5, 0.5);
        SmObjDelete sCleanUp2(pLine);
        
        SmVector3d sDirNorm[3];
        sDirNorm[0] = SmVector3d(0, 0, 1);// Normal
        sDirNorm[1] = SmVector3d(-3, 0, 0);// Pivot
        sDirNorm[2] = SmVector3d(1, 0, 0);// Side
        
        SmSolutionArray sSolutions;
        SER(pCir1->GlobalCurveSolve(pCir1->GetNaturalInterval(),
            *pLine, pLine->GetNaturalInterval(),
            SM_SO_SIGNED_PIVOT_MINIMIZE, SM_ZONE_TOL_3D,
            NULL, sDirNorm,
            SM_SR_ALL,
            sSolutions));
        
        SM_ASSERT(sSolutions.GetSize() == 1);         // gwc:used to be 2, but I can't figure out what the desired
        my_draw_2crv_sol(pCir1, pLine, sSolutions);  //     solution is, so one or two both look good to me.
    }                                                 //     What do you think?
    if (TRUE) // line/circ test - same test as the last - swap input arguments
    {
        SmBSplineCurve *pCir1 = my_create_circle(sContext, 0.5, SmPoint3d(3, 1.5, 0), SM_CO_QUADRATIC, 0, 1, 1);
        SmObjDelete sCleanUp1(pCir1);
        SmPoint3d sStart(0, 0, 0);
        SmPoint3d sEnd(5, 0, 0);
        SmBSplineCurve *pLine = my_create_line(sContext, sStart, sEnd, 0.5, 0.5, 0.5);
        SmObjDelete sCleanUp2(pLine);
        
        SmVector3d sDirNorm[3];
        sDirNorm[0] = SmVector3d(0, 0, 1);// Normal
        sDirNorm[1] = SmVector3d(-3, 0, 0);// Pivot
        sDirNorm[2] = SmVector3d(1, 0, 0);// Side
        
        SmSolutionArray sSolutions;
        SER(pLine->GlobalCurveSolve(pLine->GetNaturalInterval(),
            *pCir1, pCir1->GetNaturalInterval(),
            SM_SO_SIGNED_PIVOT_MINIMIZE, SM_ZONE_TOL_3D,
            NULL, sDirNorm,
            SM_SR_ALL,
            sSolutions));
        
        SM_ASSERT(sSolutions.GetSize() == 1);         // gwc:used to be 2, but I can't figure out what the desired
        my_draw_2crv_sol(pLine, pCir1, sSolutions);   //     solution is, so one or two both look good to me.     
    }                                                 //     What do you think?                                   
    if (TRUE)  // circ1/circ2 test
    {
        SmBSplineCurve *pCir1 = my_create_circle(sContext, 0.5, SmPoint3d(3, 1.5, 0), SM_CO_QUADRATIC, 0, 1, 1);
        SmObjDelete sCleanUp1(pCir1);
        SmBSplineCurve *pCir2 = my_create_circle(sContext, 0.5, SmPoint3d(2.9, - 1.5, 0), SM_CO_QUADRATIC, 0, 1, 1);
        SmObjDelete sCleanUp2(pCir2);
        
        SmVector3d sDirNorm[3];
        sDirNorm[0] = SmVector3d(0, 0, 1);// Normal
        sDirNorm[1] = SmVector3d(-3, 0, 0);// Pivot
        sDirNorm[2] = SmVector3d(1, 0, 0);// Side
        
        SmSolutionArray sSolutions;
        SER(pCir2->GlobalCurveSolve(pCir2->GetNaturalInterval(),
            *pCir1, pCir1->GetNaturalInterval(),
            SM_SO_SIGNED_PIVOT_MINIMIZE, SM_ZONE_TOL_3D,
            NULL, sDirNorm,
            SM_SR_ALL,
            sSolutions));
        
        SM_ASSERT(sSolutions.GetSize() == 1);
        my_draw_2crv_sol(pCir2, pCir1, sSolutions);
    }
    if (TRUE) // circ2/circ1 test - same as the last bug interchanged input arguments
    {
        SmBSplineCurve *pCir1 = my_create_circle(sContext, 0.5, SmPoint3d(3, 1.5, 0), SM_CO_QUADRATIC, 0, 1, 1);
        SmObjDelete sCleanUp1(pCir1);
        SmBSplineCurve *pCir2 = my_create_circle(sContext, 0.5, SmPoint3d(2.9, - 1.5, 0), SM_CO_QUADRATIC, 0, 1, 1);
        SmObjDelete sCleanUp2(pCir2);
        
        SmVector3d sDirNorm[3];
        sDirNorm[0] = SmVector3d(0, 0, 1);// Normal
        sDirNorm[1] = SmVector3d(-3, 0, 0);// Pivot
        sDirNorm[2] = SmVector3d(1, 0, 0);// Side
        
        SmSolutionArray sSolutions;
        SER(pCir1->GlobalCurveSolve(pCir1->GetNaturalInterval(),
            *pCir2, pCir2->GetNaturalInterval(),
            SM_SO_SIGNED_PIVOT_MINIMIZE, SM_ZONE_TOL_3D,
            NULL, sDirNorm,
            SM_SR_ALL,
            sSolutions));
        
        SM_ASSERT(sSolutions.GetSize() == 1);
        my_draw_2crv_sol(pCir1, pCir2, sSolutions);
    }
    if (TRUE)
    {
        SmBSplineCurve *pCir1 = my_create_circle(sContext, 0.5, SmPoint3d(3, 1.5, 0), SM_CO_QUADRATIC, 0, 1, 1);
        SmObjDelete sCleanUp1(pCir1);
        SmPoint3d sStart(3, 0, 0);
        SmPoint3d sEnd(5, - 2, 0);
        SmBSplineCurve *pLine = my_create_line(sContext, sStart, sEnd, 0.5, 0.5, 0.5);
        SmObjDelete sCleanUp2(pLine);
        
        SmVector3d sDirNorm[3];
        sDirNorm[0] = SmVector3d(0, 0, 1);// Normal
        sDirNorm[1] = SmVector3d(-3, 0, 0);// Pivot
        sDirNorm[2] = SmVector3d(1, 0, 0);// Side
        
        SmSolutionArray sSolutions;
        SER(pLine->GlobalCurveSolve(pLine->GetNaturalInterval(),
            *pCir1, pCir1->GetNaturalInterval(),
            SM_SO_SIGNED_PIVOT_MINIMIZE, SM_ZONE_TOL_3D,
            NULL, sDirNorm,
            SM_SR_ALL,
            sSolutions));
        
        SM_ASSERT(sSolutions.GetSize() == 1);
        my_draw_2crv_sol(pLine, pCir1, sSolutions);
    }
    if (TRUE)
    {
        SmBSplineCurve *pCir1 = my_create_circle(sContext, 0.5, SmPoint3d(3, 1.5, 0), SM_CO_QUADRATIC, 0, 1, 1);
        SmObjDelete sCleanUp1(pCir1);
        SmPoint3d sStart(3, 3, 0);
        SmPoint3d sEnd(2, 4, 0);
        SmBSplineCurve *pLine = my_create_line(sContext, sStart, sEnd, 0.5, 0.5, 0.5);
        SmObjDelete sCleanUp2(pLine);
        
        SmVector3d sDirNorm[3];
        sDirNorm[0] = SmVector3d(0, 0, 1);// Normal
        sDirNorm[1] = SmVector3d(-3, 0, 0);// Pivot
        sDirNorm[2] = SmVector3d(1, 0, 0);// Side
        
        SmSolutionArray sSolutions;
        SER(pLine->GlobalCurveSolve(pLine->GetNaturalInterval(),
            *pCir1, pCir1->GetNaturalInterval(),
            SM_SO_SIGNED_PIVOT_MINIMIZE, SM_ZONE_TOL_3D,
            NULL, sDirNorm,
            SM_SR_ALL,
            sSolutions));
        
        SM_ASSERT(sSolutions.GetSize() == 1);
        my_draw_2crv_sol(pLine, pCir1, sSolutions);
    }
    
    return SM_SUCCESS;
} // end my_test_signed_pivot_min


/***********************************************************************
PURPOSE ---

USAGE NOTES ---
***********************************************************************/
PT_EXPORT SmStatus my_test_bsplinecurve_nlib(void)
{
    SmContext sContext;
    
    SmTArray < SmPoint3d> sPnts1;
    sPnts1.Add(SmPoint3d(-1, 0.5, 0.6));            
    sPnts1.Add(SmPoint3d( 0, 0.5,  0.5));
    sPnts1.Add(SmPoint3d( 1, 1,    0.4));
    sPnts1.Add(SmPoint3d( 0, 1.5,  0.3));
    sPnts1.Add(SmPoint3d(-1, 2, 0.2));
    sPnts1.Add(SmPoint3d( 1, 4.5,  0.1));
    sPnts1.Add(SmPoint3d( 0, 3.5,  0.0));
    sPnts1.Add(SmPoint3d(-1, 4,   -0.1));
    sPnts1.Add(SmPoint3d( 1, 5,   -0.2));
    sPnts1.Add(SmPoint3d(-1, 6,   -0.3));
    sPnts1.Add(SmPoint3d( 0, 6.5, -0.4));
    sPnts1.Add(SmPoint3d( 1, 7,   -0.5));
    sPnts1.Add(SmPoint3d( 0, 7.5, -0.6));
    sPnts1.Add(SmPoint3d(-1, 8,   -0.5));
    sPnts1.Add(SmPoint3d( 0, 8.5, -0.4));
    sPnts1.Add(SmPoint3d( 1, 9,   -0.3));
    sPnts1.Add(SmPoint3d(-1, 9.5, -0.2));

    if (TRUE)
    {
        SmBSplineCurve *pNewCrv = NULL;
        double dTol = SM_ZONE_TOL_3D;
        SER(SmBSplineCurve::ApproximatePoints(sContext,// const SmContext & crContext,
            sPnts1,  // const SmTArray<SmPoint3d> & crPoints,
            3,       // ULONG lDegree,
            NULL,    // SmVector3d * pOptStartTangent,
            NULL,    // SmVector3d * pOptEndTangent,
            FALSE,   // SmBoolean bIsClosedCurve,
            &dTol,   // double * pdOptTolerance,
            pNewCrv));// SmBSplineCurve *& rpNewCurve
        
        SmObjDelete sDelCrv1(pNewCrv);
#ifdef SM_GFX_CODE
        if (smGet_DoGraphics()) {
            smgfx_Erase();
            smgfx_SetLook(1, 2, 0, 0, 1); pNewCrv->DrawWDeriv(pNewCrv->GetNaturalInterval(), 0);
            smgfx_SetLook(3, 4, 1, 0, 0); for (ULONG ii = 0;ii < sPnts1.GetSize();ii++) { sPnts1[ii].Draw(); }
            sm_GraphicsLoop();
        }
#endif
    }
    if (TRUE)
    {
        SmBSplineCurve *pNewCrv = NULL;
        // double dTol = SM_ZONE_TOL_3D;
        SER(SmBSplineCurve::C1NonRationalCubicInterpolate(
            sContext, // const SmContext & crContext,
            sPnts1,   // const SmTArray<SmPoint3d> & crPoints,
            pNewCrv));// SmBSplineCurve *& rpNewCurve
        
        SmObjDelete sDelCrv1(pNewCrv);
#ifdef SM_GFX_CODE
        if (smGet_DoGraphics()) {
            smgfx_SetLook(1, 2, 0, 1, 0); pNewCrv->DrawWDeriv(pNewCrv->GetNaturalInterval(), 0);
            sm_GraphicsLoop();
        }
#endif
    }
    if (TRUE)
    {
        SmBSplineCurve *pNurb1 = my_create_nurb(sContext, SmPoint3d(5, -2.5, 0), sPnts1, 3, 0, 0, 0);
        SmObjDelete sCU1(pNurb1);
        SER(pNurb1->InsertOneKnot(0.3456789,// double dNewKnot,
                                  2));      // ULONG lNumKnotInsertions
#ifdef SM_GFX_CODE
        if (smGet_DoGraphics()) {
            smgfx_SetLook(1, 2, 0, 1, 1); pNurb1->DrawWDeriv(pNurb1->GetNaturalInterval(), 0);
            smgfx_SetLook(3, 4, 1, 0, 0); for (ULONG ii = 0;ii < sPnts1.GetSize();ii++) { (SmPoint3d(5, -2.5, 0) + sPnts1[ii]).Draw(); }
            sm_GraphicsLoop();
        }
#endif
    }
    if (TRUE)
    {
        SmBSplineCurve *pNewCrv = NULL;
        // double dTol = SM_ZONE_TOL_3D;
        SER(SmBSplineCurve::InterpolatePoints(sContext,// const SmContext & crContext,
            sPnts1,  // const SmTArray<SmPoint3d> & crPoints,
            NULL,    // const SmTArray<double> * cpOptParams,
            3,       // ULONG lDegree,
            NULL,    // SmVector3d * pOptStartTangent,
            NULL,    // SmVector3d * pOptEndTangent,
            FALSE,   // SmBoolean bIsClosedCurve,
            SM_IT_CHORDLENGTH,// SmInterpolationType eParameterization,
            pNewCrv));// SmBSplineCurve *& rpNewCurve
        
        SmObjDelete sDelCrv1(pNewCrv);
#ifdef SM_GFX_CODE
        if (smGet_DoGraphics()) {
            smgfx_SetLook(2, 3, 1, 0, 1); pNewCrv->DrawWDeriv(pNewCrv->GetNaturalInterval(), 0);
            sm_GraphicsLoop();
        }
#endif
    }
    if (TRUE)
    {
        SmVector3d sTrans(10, - 5, 0) ; 
        SmBSplineCurve *pNurb1 = my_create_nurb(sContext, sTrans, sPnts1, 3, 0, 0, 0);
        SmObjDelete sCU1(pNurb1);
        SmBSplineCurve sNurbCpy(*pNurb1) ;

        SmPoint3d sNurbStart = sPnts1[0] + sTrans ; 
        SmPoint3d sNurbEnd   = sPnts1.GetLast() + sTrans ;

        double     dCircRadius = 2.0 ;
        SmPoint3d  sCircCenter(-1, 0.5, 0.6) ; sCircCenter += sTrans ;
        SmPoint3d  sCircStart (-1, 2.5, 0.6) ; sCircStart  += sTrans ;
        SmPoint3d  sCircEnd   (-3, 0.5, 0.6) ; sCircEnd    += sTrans ;
        SmPoint3d  sCircStartTan(-1, 0, 0) ;
        SmPoint3d  sCircEndTan  (0, -1, 0) ;
        SmExtent1d sCircAnalDomain(90, 180) ;
        SmPoint3d  sX(1,0,0) ;
        SmPoint3d  sY(0,1,0) ;

        SmPoint3d  sLineStart = sPnts1[0] + sTrans ;
        SmPoint3d  sLineEnd   = sPnts1.GetLast() + sTrans ;
        SmPoint3d  sLineStartTan(sLineEnd - sLineStart) ;
        SmPoint3d  sLineEndTan  (sLineEnd - sLineStart) ;
        
        SmLine   sLine(sLineStart, sLineEnd, 3, &sContext) ;
        SmLine   sLineCpy(sLine) ;
        SmCircle sCircle(sCircCenter, sX, sY, sCircAnalDomain, dCircRadius, 3, &sContext) ;
        SmCircle sCircleCpy(sCircle) ;
    

        SmPoint3d  sDelStart(.1,.5,.1), sDelStartTan(0, .1, .1) ;
        SmPoint3d  sDelEnd  (.1,.5,.1), sDelEndTan  (.1, 0, .1) ;
        SmCurve * pNewLineCurve = NULL ;
        SmCurve * pNewCircleCurve = NULL ;

        SmPoint3d sNewBSplineStartPos[5] { sNurbStart + sDelStart,
                                           sNurbStart + sDelStart,
                                           sPnts1[0], // not used
                                           sPnts1[0], // not used
                                           sNurbStart,       
                                         } ;
        SmPoint3d sNewBSplineStartTan[5] { sPnts1[0], // not used
                                           sPnts1[1] - sPnts1[0] + SmPoint3d(.4, .4, .4),
                                           sPnts1[0], // not used
                                           sPnts1[0], // not used
                                           sPnts1[1] - sPnts1[0],
                                         } ;
        SmPoint3d sNewBSplineEndPos[5] { sPnts1[0],   // not used
                                         sPnts1[0],   // not used
                                         sNurbEnd + sDelEnd,
                                         sNurbEnd + sDelEnd,
                                         sNurbEnd,
                                       } ;
        SmPoint3d sNewBSplineEndTan[5] { sPnts1[0],   // not used
                                         sPnts1[0],   // not used
                                         sPnts1[0],   // not used
                                         sPnts1.GetLast() - sPnts1[sPnts1.GetSize()-2] + SmPoint3d(.4, .4, .4),
                                         sPnts1.GetLast() - sPnts1[sPnts1.GetSize()-2],
                                       } ;

        SmPoint3d sNewLineStartPos[5] { sLineStart + sDelStart,
                                        sLineStart + sDelStart,
                                        sLineStart, // not used
                                        sLineStart, // not used
                                        sLineStart,       
                                      } ;
        SmPoint3d sNewLineStartTan[5] { sLineStart, // not used
                                        sLineStartTan + sDelStartTan,
                                        sLineStart, // not used
                                        sLineStart, // not used
                                        sLineStartTan,
                                      } ;
        SmPoint3d sNewLineEndPos[5] { sLineStart,   // not used
                                      sLineStart,   // not used
                                      sLineEnd + sDelEnd,
                                      sLineEnd + sDelEnd,
                                      sLineEnd,
                                    } ;
        SmPoint3d sNewLineEndTan[5] { sLineStart,   // not used
                                      sLineStart,   // not used
                                      sLineStart,   // not used
                                      sLineEndTan + sDelEndTan,
                                      sLineEndTan,
                                    } ;

        SmPoint3d sNewCircStartPos[5] { sCircStart + sDelStart,
                                        sCircStart + sDelStart,
                                        sCircStart, // not used
                                        sCircStart, // not used
                                        sCircStart,       
                                      } ;
        SmPoint3d sNewCircStartTan[5] { sCircStart, // not used
                                        sCircStartTan + sDelStartTan,
                                        sCircStart, // not used
                                        sCircStart, // not used
                                        sCircStartTan,
                                      } ;
        SmPoint3d sNewCircEndPos[5] { sCircStart,   // not used
                                      sCircStart,   // not used
                                      sCircEnd + sDelEnd,
                                      sCircEnd + sDelEnd,
                                      sCircEnd,
                                    } ;
        SmPoint3d sNewCircEndTan[5] { sCircStart,   // not used
                                      sCircStart,   // not used
                                      sCircStart,   // not used
                                      sCircEndTan + sDelEndTan,
                                      sCircEndTan,
                                    } ;
        for(ULONG ii=0;ii<5;ii++)
          {
            pNurb1->DeformToEndPointTargets((ii < 2 ) || (ii == 4) ? &sNewBSplineStartPos[ii] : NULL,       // in :  NotNULL = Curve StartPt new Tgt position 
                                                                                                            //       NULL    = leave as is
                                            (ii >= 2) || (ii == 4) ? &sNewBSplineEndPos[ii]   : NULL ,      // in :  NotNULL = Curve EndPt new Tgt position
                                                                                                            //       NULL    = leave as is
                                            NULL,                                                           // out: Ptr to new curve when editing shape changes this curve's type
                                                                                                            //      Not used when this curve can be edited in place without changing its type.
                                                                                                            //      default:[NULL]. Returns Err if not given when its needed
                                            (ii == 1) || (ii == 4) ? &sNewBSplineStartTan[ii] : NULL,       // in : optional Start 1stDir tangent direction  
                                            (ii == 1) || (ii == 4) ? SM_IV_SPECIFIED   : SM_IV_SAME,        // in : oneof SM_IV_SPECIFIED    : set Start1stDir = pTgtStartTan dir
                                                                                                            //            SM_IV_SAME         : set Start1stDir = Init Start1stDir
                                                                                                            //            SM_IV_UNCONSTRAINED: set Start1stDir = Unspecified
                                                                                                            //      default:[SM_IV_SAME]
                                            (ii == 3) || (ii == 4) ? &sNewBSplineEndTan[ii]   : NULL,       // in : optional End 1stDir tangent direction
                                            (ii == 3) || (ii == 4) ? SM_IV_SPECIFIED   : SM_IV_SAME) ;      // in : oneof SM_IV_SPECIFIED    : set Start1stDir = pTgtStartTan dir
                                                                                                            //            SM_IV_SAME         : set End1stDir = Init End1stDir
                                                                                                            //            SM_IV_UNCONSTRAINED: set End1stDir = Unspecified
                                                                                                            //      default:[SM_IV_SAME]
                                      
            sLine.DeformToEndPointTargets  ((ii < 2 ) || (ii == 4) ? &sNewLineStartPos[ii] : NULL,              // in :  NotNULL = Curve StartPt new Tgt position 
                                                                                                                //       NULL    = leave as is
                                            (ii >= 2) || (ii == 4) ? &sNewLineEndPos[ii]   : NULL ,             // in :  NotNULL = Curve EndPt new Tgt position
                                                                                                                //       NULL    = leave as is
                                           &pNewLineCurve,                                                      // out: Ptr to new curve when editing shape changes this curve's type
                                                                                                                //      Not used when this curve can be edited in place without changing its type.
                                                                                                                //      default:[NULL]. Returns Err if not given when its needed
                                            (ii == 1) || (ii == 4) ? &sNewLineStartTan[ii] : NULL,              // in : optional Start 1stDir tangent direction  
                                            (ii == 1) || (ii == 4) ? SM_IV_SPECIFIED   : SM_IV_UNCONSTRAINED,   // in : oneof SM_IV_SPECIFIED    : set Start1stDir = pTgtStartTan dir
                                                                                                                //            SM_IV_SAME         : set Start1stDir = Init Start1stDir
                                                                                                                //            SM_IV_UNCONSTRAINED: set Start1stDir = Unspecified
                                                                                                                //      default:[SM_IV_SAME]
                                            (ii == 3) || (ii == 4) ? &sNewLineEndTan[ii]   : NULL,              // in : optional End 1stDir tangent direction
                                            (ii == 3) || (ii == 4) ? SM_IV_SPECIFIED   : SM_IV_UNCONSTRAINED) ; // in : oneof SM_IV_SPECIFIED    : set Start1stDir = pTgtStartTan dir
                                                                                                                //            SM_IV_SAME         : set End1stDir = Init End1stDir
                                                                                                                //            SM_IV_UNCONSTRAINED: set End1stDir = Unspecified
                                                                                                                //      default:[SM_IV_SAME]
                                     
            sCircle.DeformToEndPointTargets((ii < 2 ) || (ii == 4) ? &sNewCircStartPos[ii] : NULL,              // in :  NotNULL = Curve StartPt new Tgt position 
                                                                                                                //       NULL    = leave as is
                                            (ii >= 2) || (ii == 4) ? &sNewCircEndPos[ii]   : NULL ,             // in :  NotNULL = Curve EndPt new Tgt position
                                                                                                                //       NULL    = leave as is
                                           &pNewCircleCurve,                                                    // out: Ptr to new curve when editing shape changes this curve's type
                                                                                                                //      Not used when this curve can be edited in place without changing its type.
                                                                                                                //      default:[NULL]. Returns Err if not given when its needed
                                            (ii == 1) || (ii == 4) ? &sNewCircStartTan[ii] : NULL,              // in : optional Start 1stDir tangent direction  
                                            (ii == 1) || (ii == 4) ? SM_IV_SPECIFIED   : SM_IV_UNCONSTRAINED,   // in : oneof SM_IV_SPECIFIED    : set Start1stDir = pTgtStartTan dir
                                                                                                                //            SM_IV_SAME         : set Start1stDir = Init Start1stDir
                                                                                                                //            SM_IV_UNCONSTRAINED: set Start1stDir = Unspecified
                                                                                                                //      default:[SM_IV_SAME]
                                            (ii == 3) || (ii == 4) ? &sNewCircEndTan[ii]   : NULL,              // in : optional End 1stDir tangent direction 
                                            (ii == 3) || (ii == 4) ? SM_IV_SPECIFIED   : SM_IV_UNCONSTRAINED) ; // in : oneof SM_IV_SPECIFIED    : set Start1stDir = pTgtStartTan dir
                                                                                                                //            SM_IV_SAME         : set End1stDir = Init End1stDir
                                                                                                                //            SM_IV_UNCONSTRAINED: set End1stDir = Unspecified
                                                                                                                //      default:[SM_IV_SAME]
#ifdef SM_GFX_CODE
SmBoolean bDebugMe = FALSE ;
            if (smGet_DoGraphics() && bDebugMe)
              {
                smos_WriteBuffer(_T("\nBegin Nurb InCurve Dump\n")) ;                         SM_DUMP_AND_ASSERT_VALID(&sNurbCpy) ;
                smos_WriteBuffer(_T("\nEnd Nurb InCurve Dump\nBegin Nurb OutCurve Dump\n")) ; SM_DUMP_AND_ASSERT_VALID(pNurb1) ;
                smos_WriteBuffer(_T("\nEnd Nurb OutCurve Dump\n")) ;
                
                smos_WriteBuffer(_T("\nBegin Line InCurve Dump\n")) ;                         SM_DUMP_AND_ASSERT_VALID(&sLineCpy) ;
                smos_WriteBuffer(_T("\nEnd Line InCurve Dump\nBegin Line OutCurve Dump\n")) ; SM_DUMP_AND_ASSERT_VALID(&sLine) ;
                smos_WriteBuffer(_T("\nEnd Line OutCurve Dump\n")) ;
                
                smos_WriteBuffer(_T("\nBegin Circle InCurve Dump\n")) ;                           SM_DUMP_AND_ASSERT_VALID(&sCircleCpy) ;
                smos_WriteBuffer(_T("\nEnd Circle InCurve Dump\nBegin Circle OutCurve Dump\n")) ; SM_DUMP_AND_ASSERT_VALID(&sCircle) ;
                smos_WriteBuffer(_T("\nEnd Circle OutCurve Dump\n")) ;
                
                // draw brep, edge and this SmCurve
                smgfx_Erase() ;
                smgfx_SetLook(4,8, 0,0,1)   ; sNurbCpy.DrawWithKnots() ; sm_GraphicsLoop() ;
                smgfx_SetLook(1,2, 1,.5,.5) ; sNurbCpy.DrawCurvature() ; sm_GraphicsLoop() ;
                smgfx_SetLook(1,2, 1,0,0)   ; sNurbCpy.DrawPolygon() ; sm_GraphicsLoop() ;

                smgfx_SetLook(3,6, 1,0,0)   ; if(pNurb1) pNurb1->DrawWithKnots() ; sm_GraphicsLoop() ;
                smgfx_SetLook(1,2, 1,.5,.5) ; if(pNurb1) pNurb1->DrawCurvature() ; sm_GraphicsLoop() ;
                smgfx_SetLook(1,2, 1,0,1)   ; if(pNurb1) pNurb1->DrawPolygon() ; sm_GraphicsLoop() ;

                smgfx_SetLook(4,8, 0,0,1)   ; sLineCpy.DrawWithKnots() ; sm_GraphicsLoop() ;
                smgfx_SetLook(3,6, 1,0,0)   ; if(pNewLineCurve) { pNewLineCurve->DrawWithKnots() ; sm_GraphicsLoop() ; }
                                              else              { sLine.DrawWithKnots() ;          sm_GraphicsLoop() ; }

                smgfx_SetLook(4,8, 0,0,1)   ; sCircleCpy.DrawWithKnots() ; sm_GraphicsLoop() ;
                smgfx_SetLook(1,2, 1,.5,.5) ; sCircleCpy.DrawCurvature() ; sm_GraphicsLoop() ;
                smgfx_SetLook(1,2, 1,0,0)   ; sCircleCpy.DrawPolygon() ; sm_GraphicsLoop() ;

                smgfx_SetLook(3,6, 1,0,0)   ; if(pNewCircleCurve) pNewCircleCurve->DrawWithKnots() ; sm_GraphicsLoop() ;
                smgfx_SetLook(1,2, 1,.5,.5) ; if(pNewCircleCurve) pNewCircleCurve->DrawCurvature() ; sm_GraphicsLoop() ;
                smgfx_SetLook(1,2, 1,0,1)   ; if(pNewCircleCurve) pNewCircleCurve->DrawPolygon() ; sm_GraphicsLoop() ;

                sm_GraphicsLoop() ;
              }
#endif // end SM_GFX_CODE

            // clean up
            if(pNewLineCurve)   { delete pNewLineCurve ;   pNewLineCurve = NULL ; }
            if(pNewCircleCurve) { delete pNewCircleCurve ; pNewCircleCurve = NULL ; }
          } // end iter DeformToEndPointTargets cases
    }

    if (TRUE)
    {
        SmBSplineCurve *pNewCrv = NULL;
        // double dTol = SM_ZONE_TOL_3D;
        SER(SmBSplineCurve::PiecewiseArcInterpolate
              (sContext,          // const SmContext & crContext,
               sPnts1,            // const SmTArray<SmPoint3d> & crPoints,
               SM_IT_CHORDLENGTH, // SmInterpolationType eParameterization,
               pNewCrv));         // SmBSplineCurve *& rpNewCurve
        
        SmObjDelete sDelCrv1(pNewCrv);
#ifdef SM_GFX_CODE
        if (smGet_DoGraphics()) {
            smgfx_Erase();
            smgfx_SetLook(1, 2, 0, .5, 1); pNewCrv->DrawWDeriv(pNewCrv->GetNaturalInterval(), 0);
            smgfx_SetLook(3, 4, 1, 0, 0); for (ULONG ii = 0;ii < sPnts1.GetSize();ii++) { sPnts1[ii].Draw(); }
            sm_GraphicsLoop();
        }
#endif
    }
    if (TRUE)
    {
        SmBSplineCurve *pNurb1 = my_create_nurb(sContext, SmPoint3d(-5, 2.5, 0), sPnts1, 3, 0, 0, 0);
        SmObjDelete sCU1(pNurb1);
        SmTArray < double> sKnots;
        pNurb1->GetKnots(sKnots);
        SmTArray < double> sNewKnots;
        sNewKnots.Add(0.25);
        sNewKnots.Add(0.35);
        sNewKnots.Add(0.45);
        sNewKnots.Add(0.55);
        sNewKnots.Add(0.65);
        sNewKnots.Add(0.75);
        SER(pNurb1->RefineCurve(sNewKnots));// const SmTArray<double> & crNewKnots
#ifdef SM_GFX_CODE
        if (smGet_DoGraphics()) {
            smgfx_SetLook(1, 2, 1, .2, .5); pNurb1->DrawWDeriv(pNurb1->GetNaturalInterval(), 0);
            smgfx_SetLook(3, 4, 1, 0, 0); for (ULONG ii = 0;ii < sPnts1.GetSize();ii++) { (SmPoint3d(-5, 2.5, 0) + sPnts1[ii]).Draw(); }
            sm_GraphicsLoop();
        }
#endif
    }
    if (TRUE)
    {
        SmBSplineCurve *pNurb1 = my_create_nurb(sContext, SmPoint3d(-8, 4, 0), sPnts1, 3, 0, 0, 0);
        SmObjDelete sCU1(pNurb1);
        double dTol = SM_ZONE_TOL_3D;
        SER(pNurb1->RemoveKnots(dTol, // double dThisApproxTol3d,
            TRUE, // SmBoolean bConstrainEndDeriv,
            1));  // ULONG lHighestDerivConstraint
        
#ifdef SM_GFX_CODE
        if (smGet_DoGraphics()) {
            smgfx_SetLook(1, 2, .2, .4, 1); pNurb1->DrawWDeriv(pNurb1->GetNaturalInterval(), 0);
            smgfx_SetLook(3, 4, 1, 0, 0); for (ULONG ii = 0;ii < sPnts1.GetSize();ii++) { (SmPoint3d(-8, 4, 0) + sPnts1[ii]).Draw(); }
            sm_GraphicsLoop();
        }
#endif
    }
    if (TRUE)
    {
        SmBSplineCurve *pNurb1 = my_create_nurb(sContext, SmPoint3d(10, - 5, 0), sPnts1, 3, 0, 0, 0);
        SmObjDelete sCU1(pNurb1);
        double dTol = SM_ZONE_TOL_3D;
        double dKnot = 0.01;
        ULONG  lNumKnotsRemoved;
        SER(pNurb1->RemoveOneKnot
               (dKnot,               // double dKnot,
                1,                   // ULONG lNumKnotsRemoval,
                dTol,                // double dThisApproxTol3d,
                lNumKnotsRemoved));  // ULONG & rlNumKnotsRemoved;
#ifdef SM_GFX_CODE
        if (smGet_DoGraphics()) {
            smgfx_SetLook(1, 2, .2, 1, .4); pNurb1->DrawWDeriv(pNurb1->GetNaturalInterval(), 0);
            smgfx_SetLook(3, 4, 1, 0, 0); for (ULONG ii = 0;ii < sPnts1.GetSize();ii++) { (SmPoint3d(10, -5, 0) + sPnts1[ii]).Draw(); }
            sm_GraphicsLoop();
        }
#endif
    }
    return SM_SUCCESS;
} // end my_test_bsplinecurve_nlib

