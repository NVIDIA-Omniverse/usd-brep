// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0


/**********************************************************************//**
* FILE NAME --- SmTess.cpp
* PURPOSE: Implementation of Brep-Based Tessellation Methods.
**********************************************************************/

#include "StdAfx.h"

#include <SmGraphicsExtern.h>
#include <SmBSplineCurve.h>
#include <SmBSplineSurface.h>
#include <SmOffsetCurve.h>
#include <SmOffsetSurface.h>
#include <SmTess.h>
#include <SmCurveCache.h>
#include <SmHermiteCurve.h>
#include <SmTrimmingTools.h>
#include <SmNurbsSrf.h>
#include <SmGeomUtility.h>
#include <SmVertex.h>
#include <SmEdge.h>
#include <SmBrep.h>
#include <SmTopologyTraverser.h>
#include <SmGraphicsOutput.h>
#include <SmPlane.h>  // for Draw() methods
#include <SmAssertArray.h>

#include <atomic>
#include <unordered_set>
#include <iterator>

#ifdef SM_USE_TBB
#include "tbb/parallel_for.h"
#endif

// #define TIMER_CODE 1

/*******************************************************************//**
PURPOSE: Class SmUVTessCallback

NOTES: 
***********************************************************************/
class SmUVTessCallback : public SmTessCallback
{
protected:
    SmSurface       * m_pSurface;   // Surface being tessellated
    SmTessSrfCache  * m_pSC;        // Surface Cache for this surface
    SmFace          * m_pFace;      // Owning Face with possible trim boundaries
    SmTess          * m_pTess;      // Tess object managing the tessellation of this face/Surface

public:
    SmTArray<double>  m_vUSplits;   // Surface tessellation split points found by pFace->GetSurface()->FindTessellationSplits(
    SmTArray<double>  m_vVSplits;   // Surface tessellation split points found by pFace->GetSurface()->FindTessellationSplits(

public:
    // constructor
    SmUVTessCallback(SmSurface      * pInSurface,
                     SmTessSrfCache * pInSC,
                     SmFace         * pInFace,
                     SmTess         * pInTess)          : m_pSurface(pInSurface),
                                                          m_pSC     (pInSC),
                                                          m_pFace   (pInFace),
                                                          m_pTess   (pInTess)
                                                        { }
    // destructor
   ~SmUVTessCallback()                                  { m_pSurface = NULL ;
                                                          m_pSC      = NULL ;
                                                          m_pFace    = NULL ;
                                                          m_pTess    = NULL ;
                                                        }

    // simple access
    virtual SmPoint3d Get3DPoint   (const SmPoint3d & crInPoint) const { return SmTess::Get3DPoint(m_pSurface,crInPoint); }
    virtual double    Get3DLength  (const SmPoint3d & crStart,
                                    const SmPoint3d & crEnd)     const { return Get3DPoint(crStart).DistanceBetween(Get3DPoint(crEnd)); }
    virtual SmPoint3d GetNormal    (SmPolyFace *pPFace)          const { SM_REF1(pPFace) ; return SmVector3d(0,0,1); }
    SmPoint2d         GetQuadSizeAt(const SmPoint3d & crPoint)   const ;

    virtual ULONG     GetInitialAge(SmPolyEdge *pEdge)           const { ULONG lStartAge = 0;
                                                                         SmEdgeuse *pEU2 = m_pTess->GetPolyEdgeEU(pEdge);
                                                                         if (pEU2 && pEU2->GetEdge()->IsSeam(*m_pSurface))
                                                                           { lStartAge = 40; } // Seams have high age.
                                                                         return lStartAge;
                                                                       }
    virtual SmPoint3d PointAt3DDistInUV(const SmPoint3d  & crMid,
                                        const SmVector3d & crBin,
                                        double        dQuadSize) const;


    SmPoint3d         FindPointNear(const SmPoint3d & crTest,
                                    ULONG           & rlUIndex,
                                    ULONG           & rlVIndex)  const ;

    virtual SmBoolean IsEdgeShortEnough(const SmPoint3d & crStartPt,
                                        const SmPoint3d & crEndPt,
                                        double dRatio)           const { ULONG lNumSubdivisions;
                                                                         SmBoolean bRet = FALSE;
                                                                         SER(m_pTess->CheckAgainstSurfaceCache(m_pSC,
                                                                                                               crStartPt,
                                                                                                               crEndPt,
                                                                                                               dRatio,
                                                                                                               FALSE,
                                                                                                               lNumSubdivisions));
                                                                         if (lNumSubdivisions == 0) { bRet = TRUE; }
                                                                         return bRet;
                                                                       }

    virtual SmStatus  Tweak(const SmPoint3d & crStartPt,         // in :
                            const SmPoint3d & crEndPt,           // in :
                            const SmPoint3d & crOtherStartPt,    // in :
                            const SmPoint3d & crOtherEndPt,      // NotUsed: in :
                            SmTessGridPoint & rNewEndPoint);     // out:

    virtual SmStatus  TweakQuad(const SmPoint3d       & crStartPt,
                                const SmTessGridPoint & crStartTop,
                                const SmPoint3d       & crEndPt,
                                const SmTessGridPoint & crEndTop,
                                SmPoint3d             & rNewStartTop,
                                SmPoint3d             & rNewEndTop);

    double            QuadScore(const SmPoint3d & crStartPt,             // in :
                                const SmPoint3d & crStartTop,            // in :
                                const SmPoint3d & crEndPt,               // in :
                                const SmPoint3d & crEndTop,              // in :
                                double            dOrigCircumf) const ;  // NotUsed: in :

    virtual SmDisplayList * Draw(void);

} ; // end class SmUVTessCallback

/*******************************************************************//**
PURPOSE:

NOTES: 
***********************************************************************/
SmDisplayList * SmUVTessCallback::Draw()
{
  SmDisplayList *pRtn = NULL ;

#ifdef SM_GFX_CODE

  smgfx_Open(smgfx_GetRuleColor());

  // draw UV locations storec in m_vUSplits and m_vVSplits.
  for (ULONG i=0; i<m_vUSplits.GetSize(); i++)
    {
      for (ULONG j=0; j<m_vVSplits.GetSize(); j++)
        {
          SmPoint3d sPnt(m_vUSplits[i], m_vVSplits[j],0);
          sPnt.Draw();
        }
    }

  pRtn = smgfx_Close() ;

#endif // end SM_GFX_CODE

  // all done
  return(pRtn) ;

} // end SmUVTessCallback::Draw

/*******************************************************************//**
PURPOSE:

NOTES: 
***********************************************************************/
SmPoint2d SmUVTessCallback::GetQuadSizeAt
  (const SmPoint3d & crPoint)
 const
{
  // locals
  ULONG lU, lV;
  SmPoint3d sPnt = FindPointNear(crPoint,lU,lV);
  SmPoint2d sSize;

  //
  if (lU > 1) { sSize.x = m_vUSplits[lU]   - m_vUSplits[lU-1]; }
  else        { sSize.x = m_vUSplits[lU+1] - m_vUSplits[lU]; }

  //
  if (lV > 1) { sSize.y = m_vVSplits[lV]   - m_vVSplits[lV-1]; }
  else        { sSize.y = m_vVSplits[lV+1] - m_vVSplits[lV]; }

  // all done
  return sSize;

} // end SmUVTessCallback::GetQuadSizeAt

/*******************************************************************//**
PURPOSE:

NOTES: 
***********************************************************************/
double SmUVTessCallback::QuadScore
  (const SmPoint3d & crStartPt,       // in :
   const SmPoint3d & crStartTop,      // in :
   const SmPoint3d & crEndPt,         // in :
   const SmPoint3d & crEndTop,        // in :
   double            dOrigCircumf)    // NotUsed: in :
 const
{
  SM_REF1(dOrigCircumf) ; 
  // FIrst measurement is quad circumference
//    double dSizeCircumf = crSizeVec.x*2.0 + crSizeVec.y*2.0;

  SmVector2d sQuadSize = GetQuadSizeAt(crStartPt);

  SmVector3d sVec1 = crEndPt    - crStartPt;
  SmVector3d sVec2 = crEndTop   - crEndPt;
  SmVector3d sVec3 = crStartTop - crEndTop;
  SmVector3d sVec4 = crStartPt  - crStartTop;

#ifdef SM_DEBUG_CODE
  SmBoolean bDebugMe = FALSE;
  if (bDebugMe)
    {
      smgfx_Erase();
      smgfx_SetColor(1,0,0); crStartPt.Draw(); sm_GraphicsLoop();
      smgfx_SetColor(1,0,0); sVec1.Draw(&crStartPt); sm_GraphicsLoop();

      smgfx_SetColor(1,0,1); crEndPt.Draw(); sm_GraphicsLoop();
      smgfx_SetColor(1,0,1); sVec2.Draw(&crEndPt); sm_GraphicsLoop();

      smgfx_SetColor(0,0,1); crEndTop.Draw(); sm_GraphicsLoop();
      smgfx_SetColor(0,0,1); sVec3.Draw(&crEndTop); sm_GraphicsLoop();

      smgfx_SetColor(0,0,0); crStartTop.Draw(); sm_GraphicsLoop();
      smgfx_SetColor(0,0,0); sVec4.Draw(&crStartTop); sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

  double dLeng1 = sVec1.Length();
  double dLeng2 = sVec2.Length();
  double dLeng3 = sVec3.Length();
  double dLeng4 = sVec4.Length();

//    double dRatio2 = smos_Fabs(dLeng2-dLeng4)/smos_Max(dLeng2,dLeng4);
//    double dRatio3 = smos_Max(dLeng2,dLeng4) /dLeng1;

//    double dQuadCircumf = sQuadSize.x*2.0 + sQuadSize.y*2.0;
  double dScoreCir = 0.0;
  // smos_Fabs(dQuadCircumf-(0.5*dOrigCircumf)) / dOrigCircumf;  // Better the smaller
  // But not too small - try to pull it to 1/2 of the original circumference
//    dScoreCir *= 3.0; // Add a stronger weighting factor to this to help
  // smaller things more.  We want to prevent skipping over stuff.

  // This will try to make edge sizes the same
  ULONG  lNumE           = 4;
//    double dWeight = 1.0;
  double dBadEdge        = 0.8;
  double dBadEdgePenalty = 5.0;

  if (dLeng3 == 0.0) lNumE = 3;
  if (smos_Fabs(sVec1.x) > smos_Fabs(sVec1.y))
    {
      double dEdgeScore = smos_Fabs(smos_Fabs(sVec1.x) - sQuadSize.x)/sQuadSize.x;
      if (dEdgeScore > dBadEdge) dEdgeScore = dEdgeScore * dBadEdgePenalty;
      dScoreCir += dEdgeScore;
    }
  else
    {
      double dEdgeScore = smos_Fabs(smos_Fabs(sVec1.y) - sQuadSize.y)/sQuadSize.y;
      if (dEdgeScore > dBadEdge) dEdgeScore = dEdgeScore * dBadEdgePenalty;
      dScoreCir += dEdgeScore;
    }

  if (smos_Fabs(sVec2.x) > smos_Fabs(sVec2.y))
    {
      double dEdgeScore = smos_Fabs(smos_Fabs(sVec2.x) - sQuadSize.x)/sQuadSize.x;
      if (dEdgeScore > dBadEdge) dEdgeScore = dEdgeScore * dBadEdgePenalty;
      dScoreCir += dEdgeScore;
    }
  else
    {
      double dEdgeScore = smos_Fabs(smos_Fabs(sVec2.y) - sQuadSize.y)/sQuadSize.y;
      if (dEdgeScore > dBadEdge) dEdgeScore = dEdgeScore * dBadEdgePenalty;
      dScoreCir += dEdgeScore;
    }

  if (dLeng3 != 0.0)
    {
      if (smos_Fabs(sVec3.x) > smos_Fabs(sVec3.y))
        {
          double dEdgeScore = smos_Fabs(smos_Fabs(sVec3.x) - sQuadSize.x)/sQuadSize.x;
          if (dEdgeScore > dBadEdge) dEdgeScore = dEdgeScore * dBadEdgePenalty;
          dScoreCir += dEdgeScore;
        }
      else
        {
          double dEdgeScore = smos_Fabs(smos_Fabs(sVec3.y) - sQuadSize.y)/sQuadSize.y;
          if (dEdgeScore > dBadEdge) dEdgeScore = dEdgeScore * dBadEdgePenalty;
          dScoreCir += dEdgeScore;
        }
    }

  if (smos_Fabs(sVec4.x) > smos_Fabs(sVec4.y))
    {
      double dEdgeScore = smos_Fabs(smos_Fabs(sVec4.x) - sQuadSize.x)/sQuadSize.x;
      if (dEdgeScore > dBadEdge) dEdgeScore = dEdgeScore * dBadEdgePenalty;
      dScoreCir += dEdgeScore;
    }
  else
    {
      double dEdgeScore = smos_Fabs(smos_Fabs(sVec4.y) - sQuadSize.y)/sQuadSize.y;
      if (dEdgeScore > dBadEdge) dEdgeScore = dEdgeScore * dBadEdgePenalty;
      dScoreCir += dEdgeScore;
    }


  if (dLeng1 == 0.0) dScoreCir += 100.0;
  if (dLeng2 == 0.0) dScoreCir += 100.0;
  if (dLeng4 == 0.0) dScoreCir += 100.0;
  if (dLeng3 == 0.0) dScoreCir += 3.0;  // Weight Triangles a little heavier

  if (sVec2.x != 0.0 && sVec2.y != 0.0) { dScoreCir += 0.5; }
  if (sVec3.x != 0.0 && sVec3.y != 0.0) { dScoreCir += 0.5; }
  if (sVec4.x != 0.0 && sVec4.y != 0.0) { dScoreCir += 0.5; }

  // Now weight up when an edge gets too long
//    double dSizeLeng = crSizeVec.Length();
//    if (dLeng1 > dSizeLeng) dScoreCir += 1.0;
//    if (dLeng2 > dSizeLeng) dScoreCir += 1.0;
//    if (dLeng3 > dSizeLeng) dScoreCir += 1.0;
//    if (dLeng4 > dSizeLeng) dScoreCir += 1.0;

  // Second measurement is convexity.
  if (sVec1.LengthSquared() > SM_EFF_ZERO_SQ) sVec1.Unitize();
  if (sVec2.LengthSquared() > SM_EFF_ZERO_SQ) sVec2.Unitize();
  if (sVec3.LengthSquared() > SM_EFF_ZERO_SQ) sVec3.Unitize();
  if (sVec4.LengthSquared() > SM_EFF_ZERO_SQ) sVec4.Unitize();

  double dAngleTotal = 0.0;

  if (lNumE == 4)
    {
      double dAngle1 = smos_Fabs(sVec1.Dot(sVec2));
      double dAngle2 = smos_Fabs(sVec2.Dot(sVec3));
      double dAngle3 = smos_Fabs(sVec3.Dot(sVec4));
      double dAngle4 = smos_Fabs(sVec4.Dot(sVec1));
      if (dAngle1 > 0.5) dAngleTotal += dAngle1;
      if (dAngle2 > 0.5) dAngleTotal += dAngle2;
      if (dAngle3 > 0.5) dAngleTotal += dAngle3;
      if (dAngle4 > 0.5) dAngleTotal += dAngle4;
      dAngleTotal += dAngle1 + dAngle2 + dAngle3 + dAngle4;
    }
  if (lNumE == 3)
    {
      // Try to snap to 60 degrees for triangles
      double dAngle1 = smos_Fabs(0.5 - smos_Fabs(sVec1.Dot(sVec2)));
      double dAngle2 = smos_Fabs(0.5 - smos_Fabs(sVec2.Dot(sVec4)));
      double dAngle3 = smos_Fabs(0.5 - smos_Fabs(sVec4.Dot(sVec1)));
      if (dAngle1 > 0.5) dAngleTotal += dAngle1;
      if (dAngle2 > 0.5) dAngleTotal += dAngle2;
      if (dAngle3 > 0.5) dAngleTotal += dAngle3;
      dAngleTotal += dAngle1 + dAngle2 + dAngle3;
    }

  return dAngleTotal + dScoreCir;

} // end SmUVTessCallback::QuadScore

/*******************************************************************//**
PURPOSE:

NOTES: 
***********************************************************************/
SmPoint3d SmUVTessCallback::FindPointNear
  (const SmPoint3d & crTest,    // in :
   ULONG           & rlUIndex,  // out:
   ULONG           & rlVIndex)  // out:
 const
{
  // lcoals
  SmPoint3d sCurrEnd = crTest;
  double    dMinU;

    {
      ULONG low     = 0;
      ULONG high    = m_vUSplits.GetSize()-1;
      ULONG mid     = (low+high)/2;
      ULONG lastmid = mid;

      while(   sCurrEnd.x  < m_vUSplits[mid]
            || sCurrEnd.x >= m_vUSplits[mid] )
        {
          if( sCurrEnd.x < m_vUSplits[mid] ) { high = mid; }
          else                               { low  = mid; }
          mid = (low+high)/2;

          if (mid == lastmid)
            break;
          lastmid = mid;
        }

      dMinU    = m_vUSplits[mid];
      rlUIndex = mid;
    }

  double dMinV;
    {
      ULONG low     = 0;
      ULONG high    = m_vVSplits.GetSize()-1;
      ULONG mid     = (low+high)/2;
      ULONG lastmid = mid;

      while(   sCurrEnd.y < m_vVSplits[mid]
            || sCurrEnd.y >= m_vVSplits[mid] )
        {
          if( sCurrEnd.y < m_vVSplits[mid] ) { high = mid; }
          else                               { low  = mid; }
          mid = (low+high)/2;

          if (mid == lastmid)
            break;
          lastmid = mid;
        }

      dMinV    = m_vVSplits[mid];
      rlVIndex = mid;
    }

  return SmPoint3d(dMinU, dMinV, crTest.z);

} // end SmUVTessCallback::FindPointNear

/*******************************************************************//**
PURPOSE:

NOTES: 
***********************************************************************/
SmStatus SmUVTessCallback::TweakQuad
  (const SmPoint3d & crStartPt,
   const SmTessGridPoint & crStartTop,
   const SmPoint3d & crEndPt,
   const SmTessGridPoint & crEndTop,
   SmPoint3d & rNewStartTop,
   SmPoint3d & rNewEndTop)
{
    rNewStartTop = crStartTop.m_vPnt;
    rNewEndTop = crEndTop.m_vPnt;

    if (crStartTop.m_pPolyVertex && crEndTop.m_pPolyVertex)
      {
        return SM_SUCCESS;
      }


    // crStartTop and crEndTop are not one node away in the grid

#ifdef SM_DEBUG_CODE
      SmBoolean bDebugMe = FALSE;
    if (bDebugMe)
      {
        smgfx_Erase();
        smgfx_SetColor(1,0,0);
        crStartPt.Draw();
        sm_GraphicsLoop();
        smgfx_SetColor(0,0,0);
        crStartTop.m_vPnt.Draw();
        sm_GraphicsLoop();
        smgfx_SetColor(1,0,1);
        crEndPt.Draw();
        sm_GraphicsLoop();
        smgfx_SetColor(0,0,1);
        crEndTop.m_vPnt.Draw();
        sm_GraphicsLoop();
      }
#endif

    ULONG lUMin = smos_Min(crStartTop.m_lU,crEndTop.m_lU);
    ULONG lUMax = smos_Max(crStartTop.m_lU,crEndTop.m_lU);
    ULONG lVMin = smos_Min(crStartTop.m_lV,crEndTop.m_lV);
    ULONG lVMax = smos_Max(crStartTop.m_lV,crEndTop.m_lV);
    if (lUMin > 0) lUMin --;
    if (lUMax < m_vUSplits.GetSize()-1) lUMax ++;
    if (lVMin > 0) lVMin --;
    if (lVMax < m_vVSplits.GetSize()-1) lVMax ++;

    ULONG lUSt, lVSt;
    // Expand search radius using mid point of edge
    FindPointNear((crStartPt+crEndPt)/2.0,lUSt,lVSt);
    lUMin = smos_Min(lUMin,lUSt);
    lUMax = smos_Max(lUMax,lUSt);
    lVMin = smos_Min(lVMin,lVSt);
    lVMax = smos_Max(lVMax,lVSt);
//    FindPointNear(crEndPt,lUSt,lVSt);
//    lUMin = smos_Min(lUMin,lUSt);
//    lUMax = smos_Max(lUMax,lUSt);
//    lVMin = smos_Min(lVMin,lVSt);
//    lUMax = smos_Max(lVMax,lVSt);
    SmPoint3d sMinCorner(m_vUSplits[lUMin],m_vVSplits[lVMin],0);
    SmPoint3d sMaxCorner(m_vUSplits[lUMax],m_vVSplits[lVMax],0);
    SmVector3d sCornerVec = sMaxCorner - sMinCorner;
    double dOrigCircumf = sCornerVec.x*2.0 + sCornerVec.y*2.0;


    // Find Binormal of original
    SmVector3d sBase = crEndPt - crStartPt;
    SmVector3d sBinormal(-sBase.y,sBase.x,0.0);

    // Basically we test every horizontal or vertical combination
    // of adjacent grid elements.
    double dMinScore = QuadScore(crStartPt,crStartTop.m_vPnt,crEndPt,crEndTop.m_vPnt,dOrigCircumf);

    for (ULONG lU=lUMin; lU<=lUMax; lU++)
      {
        for (ULONG lV=lVMin; lV<=lVMax; lV++)
          {
            SmPoint3d sTest(m_vUSplits[lU],m_vVSplits[lV],0);
            SmVector3d sSideTest = sTest - crStartPt;
            if (sSideTest.Dot(sBinormal) < 0) continue;
            if (crStartTop.m_pPolyVertex)
              {
                ULONG lDelta =   smos_Labs(((long)lU-(long)crStartTop.m_lU))
                               + smos_Labs(((long)lV-(long)crStartTop.m_lV));
                if (lDelta > 2) continue;
                double dScore1 = QuadScore(crStartPt,crStartTop.m_vPnt,crEndPt,sTest,dOrigCircumf);
                if (dScore1 < dMinScore)
                  {
                    dMinScore = dScore1;
                    rNewEndTop = sTest;
                  }
                continue;
              }
            if (crEndTop.m_pPolyVertex)
              {
                ULONG lDelta = smos_Labs((long)(lU-crEndTop.m_lU)) + smos_Labs((long)(lV-crEndTop.m_lV));
                if (lDelta > 2) continue;
                double dScore1 = QuadScore(crStartPt,sTest,crEndPt,crEndTop.m_vPnt,dOrigCircumf);
                if (dScore1 < dMinScore)
                  {
                    dMinScore = dScore1;
                    rNewStartTop = sTest;
                  }
                continue;
              }
            if (lU < lUMax)
              {
                SmPoint3d sTest2(m_vUSplits[lU+1],m_vVSplits[lV],0);
                SmVector3d sSideTest2 = sTest2 - crStartPt;
                if (sSideTest2.Dot(sBinormal) > 0)
                  {
                    double dScore1 = QuadScore(crStartPt,sTest,crEndPt,sTest2,dOrigCircumf);
                    if (dScore1 < dMinScore)
                      {
                        dMinScore = dScore1;
                        rNewStartTop = sTest;
                        rNewEndTop = sTest2;
                      }
                    double dScore2 = QuadScore(crStartPt,sTest2,crEndPt,sTest,dOrigCircumf);
                    if (dScore2 < dMinScore)
                      {
                        dMinScore = dScore2;
                        rNewStartTop = sTest2;
                        rNewEndTop = sTest;
                      }
                  }
              }
            if (lV < lVMax)
              {
                SmPoint3d sTest2(m_vUSplits[lU],m_vVSplits[lV+1],0);
                SmVector3d sSideTest2 = sTest2 - crStartPt;
                if (sSideTest2.Dot(sBinormal) > 0)
                  {
                    double dScore1 = QuadScore(crStartPt,sTest,crEndPt,sTest2,dOrigCircumf);
                    if (dScore1 < dMinScore)
                      {
                        dMinScore = dScore1;
                        rNewStartTop = sTest;
                        rNewEndTop = sTest2;
                      }
                    double dScore2 = QuadScore(crStartPt,sTest2,crEndPt,sTest,dOrigCircumf);
                    if (dScore2 < dMinScore)
                      {
                        dMinScore = dScore2;
                        rNewStartTop = sTest2;
                        rNewEndTop = sTest;
                      }
                  }
              }
          }
      }

    return SM_SUCCESS;

} // end SmUVTessCallback::TweakQuad

/*******************************************************************//**
PURPOSE:

NOTES: 
***********************************************************************/
SmStatus SmUVTessCallback::Tweak
  (const SmPoint3d & crStartPt,         // in :
   const SmPoint3d & crEndPt,           // in :
   const SmPoint3d & crOtherStartPt,    // NotUsed: in :
   const SmPoint3d & crOtherEndPt,      // in :
   SmTessGridPoint & rNewEndPoint)      // out:
{
  SM_REF1(crOtherStartPt) ; 
    rNewEndPoint.m_vPnt = crEndPt; // Default is to keep end point same
    rNewEndPoint.m_bPointOnGrid = FALSE;

#ifdef SM_DEBUG_CODE
    SmBoolean bDebugMe = FALSE;
    if (bDebugMe)
      {
        crStartPt.Dump();
        crEndPt.Dump();
        m_vUSplits.Dump();
        m_vVSplits.Dump();
        smgfx_Erase();
        smgfx_SetColor(1,0,0);
        crStartPt.Draw();
        sm_GraphicsLoop();
        smgfx_SetColor(1,0,1);
        crEndPt.Draw();
        sm_GraphicsLoop();
        smgfx_SetColor(0,0,0);
        Draw();
        sm_GraphicsLoop();
      }
#endif

    // Let's do the triangle test to see if we have two aligned
    if (SM_ARE_SAME(crStartPt.x,crOtherEndPt.x) ||
        SM_ARE_SAME(crStartPt.y,crOtherEndPt.y) )
          {
        ULONG lUIndex, lVIndex;
        SmPoint3d sMid = (crStartPt + crOtherEndPt) / 2.0;
        SmPoint3d sTest = FindPointNear(sMid,lUIndex,lVIndex);
        if (SM_ARE_SAME(crStartPt.x,sTest.x) ||
            SM_ARE_SAME(crStartPt.y,sTest.y) )
              {
            if (sTest.DistanceBetweenSquared(crStartPt) > SM_EFF_ZERO_SQ &&
                sTest.DistanceBetweenSquared(crOtherEndPt) > SM_EFF_ZERO_SQ)
                  {
                rNewEndPoint.m_lU = lUIndex;
                rNewEndPoint.m_lV = lVIndex;
                rNewEndPoint.m_vPnt = sTest;
                rNewEndPoint.m_bPointOnGrid = TRUE;
                return SM_SUCCESS;
              }
            rNewEndPoint.m_lU = lUIndex;
            rNewEndPoint.m_lV = lVIndex;
            rNewEndPoint.m_vPnt = crOtherEndPt;
            rNewEndPoint.m_bPointOnGrid = TRUE;
            return SM_SUCCESS;
          }
      }

    SmPoint3d sCurrEnd = crEndPt;
    SmVector3d sVec = sCurrEnd - crStartPt;
//    double dOrigLength = sVec.Length();

    for (ULONG j=0; j<5; j++)
      {

        ULONG lUIndex, lVIndex;
        SmPoint3d sTestEnd = FindPointNear(sCurrEnd,lUIndex,lVIndex);

        SmVector3d sNewVec = sTestEnd - crStartPt;
        double dNewLength = sNewVec.Length();
        if (dNewLength > SM_EFF_ZERO)
          {
            rNewEndPoint.m_lU = lUIndex;
            rNewEndPoint.m_lV = lVIndex;
            sCurrEnd = sTestEnd;
            break;
          }
        sCurrEnd = crStartPt + (j+2.0) * (crEndPt-crStartPt);
      }


    if (sCurrEnd.DistanceBetweenSquared(crStartPt) < SM_EFF_ZERO_SQ)
      {
        return SM_SUCCESS;
      }

    SmVector3d sNewVec = sCurrEnd - crStartPt;
    double dDot = sVec.Dot(sNewVec);
    if (dDot < 0.0)
      {
        return SM_SUCCESS;
      }

#ifdef SM_DEBUG_CODE
    SmBoolean bDebugMe2 = FALSE;
    if (bDebugMe2)
      {
        crStartPt.Dump();
        crEndPt.Dump();
        sCurrEnd.Dump();
        m_vUSplits.Dump();
        m_vVSplits.Dump();
        smgfx_Erase();
        smgfx_SetColor(1,0,0);
        sCurrEnd.Draw();
        sm_GraphicsLoop();
        smgfx_SetColor(1,1,0);
        crStartPt.Draw();
        sm_GraphicsLoop();
        smgfx_SetColor(1,0,1);
        crEndPt.Draw();
        sm_GraphicsLoop();
        smgfx_SetColor(0,0,0);
        Draw();
        sm_GraphicsLoop();
      }
#endif
    rNewEndPoint.m_bPointOnGrid = TRUE;
    rNewEndPoint.m_vPnt = sCurrEnd;

    return SM_SUCCESS;

} // end SmUVTessCallback::Tweak

/*******************************************************************//**
PURPOSE:

NOTES: 
***********************************************************************/
SmPoint3d SmUVTessCallback::PointAt3DDistInUV
  (const SmPoint3d  & crMid,
   const SmVector3d & crBin,
   double             dQuadSize)
 const
{
  // locals
  double    dStep  = dQuadSize;
  SmPoint3d s3DMid = Get3DPoint(crMid);
  SmPoint3d sOffset;

  //
  for (ULONG jjj=0; jjj<5; jjj++)
    {
      // Compute a scaled version of the 2D vector of length dAverageEdgeLength in 3D
      SmVector3d sScaledBinVec = m_pTess->ScaleUVVecTo3DLength(m_pSurface,
                                                               crMid,
                                                               dStep,
                                                               crBin);
      sOffset             = crMid + sScaledBinVec;
      SmPoint3d s3DOffset = Get3DPoint(sOffset);
      double dEdgeLength  = s3DMid.DistanceBetween(s3DOffset);

      if (smos_Fabs(dEdgeLength-dQuadSize) < 0.01 * dQuadSize)
        { break; }

      dStep = dStep * dQuadSize/dEdgeLength;
    }

  // all done
  return sOffset;

} // end SmUVTessCallback::PointAt3DDistInUV


/*******************************************************************//**
PURPOSE: Static helper routine: remove any PolyEdges in the given loop
   from the given list.

NOTES: 
***********************************************************************/
static void sm_RemovePolyEdgesFromArray( SmPolyLoop *pPolyLoop, SmTArray< SmPolyEdge* > &rEdgeList )
{
  if ( rEdgeList.GetSize() < 1 ) { return; }

  ULONG ii, lIndex;
  SmPolyEdge *pPE;
  SM_PTR_ARRAY(sPolyEdges, SmPolyEdge, 32);
  pPolyLoop->GetPolyEdges( sPolyEdges );

  for ( ii=0; ii<sPolyEdges.GetSize(); ii++ )
  {
      pPE = sPolyEdges[ii];
      if( pPE && rEdgeList.FindElement( pPE, lIndex ))
      {
          rEdgeList.RemoveAt( lIndex, 1 );
      }
  }

  return;

} // end static sm_RemovePolyEdgesFromArray

/*******************************************************************//**
PURPOSE: Make a copy of this attribute in a virtual way such that
    a general call can be made to copy all attributes.

NOTES: 
***********************************************************************/
SmAttribute * SmPolyToEUVUAttr::MakeCopy
  (const SmContext & crContext)
 const
{
    SmPolyToEUVUAttr *pRet = new (crContext) SmPolyToEUVUAttr(m_lAttributeID,m_eBehavior);
    pRet->m_pEdgeuse = m_pEdgeuse;
    pRet->m_pVertexuse = m_pVertexuse;
    return pRet;

} // end SmPolyToEUVUAttr::MakeCopy

/*******************************************************************//**
PURPOSE: This is the constructor for the tessellation object.

NOTES: 
***********************************************************************/
SmTess::SmTess
  (const SmContext     & crContext,
   SmCurveTessDriver   & rCurveTess,
   SmSurfaceTessDriver & rSurfaceTess)
 : // m_lTriangulateFaceLevel(0),
   m_crContext            (crContext),
   m_bAdvancingFront      (FALSE),
   m_pTessBrep            (NULL),
   m_rCurveTess           (rCurveTess),
   m_rSurfaceTess         (rSurfaceTess),
   m_vCache               (crContext,SM_RC_ONE_TO_ONE),
   m_p3DPolyBrep          (NULL),
  //  m_pBrepToPolyBrepMap   (NULL),
   m_bCheckLicense        (FALSE),
   m_eTessAlgorithm       (SM_TA_FASTER_TESSELLATION)
{

} // end SmTess::SmTess constructor

/*******************************************************************//**
PURPOSE: Destructor for SmTess object.

NOTES: 
***********************************************************************/
SmTess::~SmTess()
{
    SmTArray<SmTessSrfCache*> sCaches;
    this->m_vCache.GetAllSeconds(sCaches);

    for (ULONG i=0; i<sCaches.GetSize(); i++)
      {
        SM_ASSERT(sCaches[i] != NULL) ; delete sCaches[i]; sCaches[i] = NULL ;
      }

    if (m_p3DPolyBrep)        { delete m_p3DPolyBrep;        m_p3DPolyBrep        = NULL ; }
    if (m_pTessBrep)          { delete m_pTessBrep;          m_pTessBrep          = NULL ; }
  //  if (m_pBrepToPolyBrepMap) { delete m_pBrepToPolyBrepMap; m_pBrepToPolyBrepMap = NULL ; }

} // end SmTess::~SmTess destructor

/*******************************************************************//**
PURPOSE: Set the smoothing data.

NOTES: 
***********************************************************************/
void SmTess::SetSmoothingData
  (const SmSmoothingData & crSmoothingData)
{
    m_vSmoothingData = crSmoothingData;

} // end SmTess::SetSmoothingData

/*******************************************************************//**
PURPOSE: Set the Edgeuse corresponding to this poly edge ask for
         attribute propagation from OriginalEdgeuse->Edge to PolyEdge

NOTES: 
***********************************************************************/
void SmTess::SetPolyEdgeEU
  (SmPolyEdge *pPolyEdge,          // in : Tgt PolyEdge
   SmEdgeuse  *pOriginalEdgeuse)   // in : BrepEdgeuse origin of PolyEdge tessellation
{
  if (pPolyEdge == NULL)
    {
      SE(SM_ERR);
      return;
    }

  pPolyEdge->SetOriginalEdgeuse(pOriginalEdgeuse,TRUE) ;
  // GWC: okay, I changed this
  // GWC: this is awful code - we are typecasting to hide an SmEdgeuse ptr in a SmPolyVertAuxDat slot
  // pPolyEdge->m_pAuxData = (SmPolyVertAuxData*)pEdgeuse;

//    SmPolyToEUVUAttr *pAttr = (SmPolyToEUVUAttr*)pPolyEdge->FindAttribute(SM_AI_POLY_TO_EUVU);
//    if (pAttr == NULL) {
//        pAttr = new (*pPolyEdge->GetContext()) SmPolyToEUVUAttr(SM_AI_POLY_TO_EUVU, SM_AB_COPY);
//        pPolyEdge->AddAttribute(pAttr);
//    }
//    pAttr->m_pEdgeuse = pEdgeuse;

} // end SmTess::SetPolyEdgeEU

/*******************************************************************//**
PURPOSE: Set the Vertexuse corresponding to this poly edge.

NOTES: 
***********************************************************************/
void SmTess::SetPolyEdgeVU
  (SmPolyEdge  *pPolyEdge,
   SmVertexuse *pVertexuse)
{
  if (pPolyEdge == NULL)
    {
      SE(SM_ERR);
      return;
    }
  SmPolyToEUVUAttr *pAttr = (SmPolyToEUVUAttr*)pPolyEdge->FindAttribute(SM_AI_POLY_TO_EUVU);
  if (pAttr == NULL)
    {
      pAttr = new (*pPolyEdge->GetContext()) SmPolyToEUVUAttr(SM_AI_POLY_TO_EUVU, SM_AB_COPY);
      pPolyEdge->AddAttribute(pAttr);
    }
  pAttr->m_pVertexuse = pVertexuse;

} // end SmTess::SetPolyEdgeVU

/*******************************************************************//**
PURPOSE: Get the edgeuse of a poly edge.

NOTES: 
***********************************************************************/
SmEdgeuse * SmTess::GetPolyEdgeEU
  (const SmPolyEdge *pPolyEdge)
 const
{
  if (pPolyEdge == NULL)
    {
      SE(SM_ERR);
      return NULL;
    }

  return pPolyEdge->GetEdgeuse() ;

  // return (SmEdgeuse*)pPolyEdge->m_pAuxData;
  // gwc: this looks like bad coding practice.
  //      We should not be casting a SmPolyVertAuxData type to a SmEdgeuse*
  //      Imagine the problems this causes for persistence.  That
  //      Operation won't know what data to write or read.

//    SmPolyToEUVUAttr *pAttr = (SmPolyToEUVUAttr*)pPolyEdge->FindAttribute(SM_AI_POLY_TO_EUVU);
//    if (!pAttr) {
//        return NULL;
//    }
//    return pAttr->m_pEdgeuse;

} // end SmTess::GetPolyEdgeEU

/*******************************************************************//**
PURPOSE: Get the vertexuse of a poly edge.

NOTES: 
***********************************************************************/
SmVertexuse * SmTess::GetPolyEdgeVU
  (const SmPolyEdge *pPolyEdge)
 const
{
    if (pPolyEdge == NULL)
      {
        SE(SM_ERR);
        return NULL;
      }
    SmPolyToEUVUAttr *pAttr = (SmPolyToEUVUAttr*)pPolyEdge->FindAttribute(SM_AI_POLY_TO_EUVU);
    if (!pAttr)
      {
        return NULL;
      }
    return pAttr->m_pVertexuse;

} // end SmTess::GetPolyEdgeVU

/*******************************************************************//**
PURPOSE: This object is an attribute which will contain EU and VU
    pointers back to the Tessellation Brep.

NOTES: 
***********************************************************************/
class SmPolyEdgeToVertexUseAttr : public SmAttribute
{
  friend class SmTess;
protected:
  SmVertexuse *m_pVertexuse;

public:
  SmPolyEdgeToVertexUseAttr(ULONG lAttributeID,
      SmAttributeBehaviorType eBehavior = SM_AB_COPY)
    : SmAttribute(lAttributeID,eBehavior),
      m_pVertexuse(NULL) {}

  virtual ~SmPolyEdgeToVertexUseAttr() {}

  virtual SmAttribute * MakeCopy(const SmContext & crContext) const
  {
      SmPolyEdgeToVertexUseAttr *pRet = new (crContext) SmPolyEdgeToVertexUseAttr (m_lAttributeID,m_eBehavior);
      pRet->m_pVertexuse = m_pVertexuse;
      return pRet;
  }

  // get memory used and allocated
  virtual ULONG GetMemoryUsed(ULONG &rlMemoryAllocated) const
     { ULONG lAllocated, lUsed = m_vUsers.GetMemoryUsed(lAllocated) ;
       rlMemoryAllocated = sizeof(this) + lAllocated - sizeof(SmTArray<SmAObject*>) ;
       return(             sizeof(this) + lUsed      - sizeof(SmTArray<SmAObject*>)) ;
     }

} ; // end class SmPolyEdgeToVertexUseAttr

/*******************************************************************//**
PURPOSE: Set the Vertexuse corresponding to this poly edge.

NOTES: 
***********************************************************************/
void SmTess::SetPolyEdgeVertexuse
  (SmPolyEdge  *pPolyEdge,
   SmVertexuse *pVertexuse)
{
  if (pPolyEdge == NULL)
    {
      SE(SM_ERR);
      return;
    }
  SmPolyEdgeToVertexUseAttr *pAttr = (SmPolyEdgeToVertexUseAttr*)pPolyEdge->FindAttribute(SM_AI_TESSELATION_POLY_EDGE_VERTEX_USE);
  if (pAttr == NULL)
    {
      pAttr = new (*pPolyEdge->GetContext()) SmPolyEdgeToVertexUseAttr(SM_AI_TESSELATION_POLY_EDGE_VERTEX_USE, SM_AB_COPY);
      pPolyEdge->AddAttribute(pAttr);
    }
  pAttr->m_pVertexuse = pVertexuse;

} // end SmTess::SetPolyEdgeVU

/*******************************************************************//**
PURPOSE: Get the vertexuse of a poly edge.

NOTES: 
***********************************************************************/
SmVertexuse * SmTess::GetPolyEdgeVertexuse
  (const SmPolyEdge *pPolyEdge)
 const
{
    if (pPolyEdge == NULL)
      {
        SE(SM_ERR);
        return NULL;
      }
    SmPolyEdgeToVertexUseAttr *pAttr = (SmPolyEdgeToVertexUseAttr*)pPolyEdge->FindAttribute(SM_AI_TESSELATION_POLY_EDGE_VERTEX_USE);
    if (!pAttr)
      {
        return NULL;
      }
    return pAttr->m_pVertexuse;

} // end SmTess::GetPolyEdgeVU

/*******************************************************************//**
PURPOSE:

NOTES: 
***********************************************************************/
static double sm_3dStepEstimate
  (double dRadiusOfCurvature,
   double dChordHeightTolerance,
   double dAngleTolerance)
{
  double d3DStepSize = SM_BIG_DOUBLE;

  if (dChordHeightTolerance > SM_EFF_ZERO)
    {
      double dRMinusCh = dRadiusOfCurvature - dChordHeightTolerance;
      double d3DCHStepSize = 2.0 * smos_Sqrt( smos_Fabs((dRadiusOfCurvature * dRadiusOfCurvature) -
          (dRMinusCh*dRMinusCh)) );
      d3DStepSize = smos_Min(d3DStepSize,d3DCHStepSize);
    }
  if (dAngleTolerance > SM_EFF_ZERO)
    {
      double d3DCHStepSize = 2.0 * smos_Sine(dAngleTolerance/2.0) * dRadiusOfCurvature;
      if (d3DCHStepSize > 0.0)
        {
          d3DStepSize = smos_Min(d3DStepSize,d3DCHStepSize);
        }
    }
  return d3DStepSize;

} // end sm_3dStepEstimate

/*******************************************************************//**
PURPOSE:

NOTES: 
***********************************************************************/
static double sm_EstimateStep
  (const SmExtent1d & crInterval,             // in :
   SmPoint3d          aPVV[3],                // in :
   double             dChordHeightTolerance,  // in :
   double             dAngleTolerance)        // in :
{
  double d2ndDerivLength = aPVV[2].Length();
  if (SM_IS_ZERO(d2ndDerivLength))
    {
      return crInterval.GetMax() - crInterval.GetMin();
    }
  double dRadiusOfCurvature = 1.0 / aPVV[2].Length();

  // Compute 3D step size based on tolerances
  double d3DStepSize = sm_3dStepEstimate(dRadiusOfCurvature,
                                           dChordHeightTolerance,
                                           dAngleTolerance*SM_PI/180.0);

  // Get parameter estimate by multiplying by tangent length
  double dEstParamStepSize = d3DStepSize * aPVV[1].Length();

  return dEstParamStepSize;  // Step sizes seem a little too large

} // end sm_EstimateStep

#if 0
/*******************************************************************//**
PURPOSE:

NOTES: UNUSED
***********************************************************************/
static void sm_Output
  (SmPoint3d aPVV[3],
   double dParameter,
   SmTArray<double> * pParameters,
   SmTArray<SmPoint3d> * pPoints)
{
    if (pParameters) pParameters->Add(dParameter);
    if (pPoints) pPoints->Add(aPVV[0]);

} // end sm_Output
#endif

/*******************************************************************//**
PURPOSE: Initialize tessellation parameters - first step size,
   minimum and maximum steps which can be taken.

NOTES:
***********************************************************************/
SmStatus SmCurveTessDriver::Initialize
  (const SmCurve    & crCurve,                  // NotUsed: in : target curve
   const SmExtent1d & crInterval,               // in : tessellation interval
   SmVector3d         aInitialPntAndDerivs[3],  // in : Three derivatives at crInterval.GetMin()
   double           & rdFirstStepEstimate,      // out:
   double           & rdMinStepSize,            // out:
   double           & rdMaxStepSize)            // out:

{
  SM_REF1(crCurve) ; 
  // get estimated step size based on single point position, tangent, and curvature sampling.
  rdFirstStepEstimate = sm_EstimateStep(crInterval,
                                         aInitialPntAndDerivs,
                                         m_dChordHeight,
                                         m_dAngTolDeg);

  // max step - limited by Minimum Number of segments
  rdMaxStepSize = (crInterval.GetMax()-crInterval.GetMin());
  if (m_lMinSegNumber > 1) rdMaxStepSize /= m_lMinSegNumber;

  // estimated step - limited to max step size
  rdFirstStepEstimate = smos_Min(rdFirstStepEstimate,rdMaxStepSize);

  // min step - set by Minimum Parametric Ratio
  rdMinStepSize = m_dMinParamRatio * ( crInterval.GetMax() - crInterval.GetMin() );

  // all done
  return SM_SUCCESS;

} // end SmCurveTessDriver::Initialize

/*******************************************************************//**
PURPOSE: Test the current step to see if it satisfies the criteria
    of the tessellation.  It will make a recommendation to accept the
    current step, increase the step or decrease the step size.

NOTES: 
***********************************************************************/
SmStatus SmCurveTessDriver::TestStep
 (double                     dCurrentParameter, // NotUsed: in :
  double                     dParamStepSize,    // in :
  SmVector3d                 sCurPVV[3],        // in :
  SmVector3d                 sNextPVV[3],       // in :
  SmTessStepTestResultType & reResult)          // out:
{
  SM_REF1(dCurrentParameter) ; 
  reResult = SM_ST_INCREASE_STEP;
  // First do the distance checks because they are the easiest.
  double dDist = 0.0;
  if (m_dMaxDist3dBetweenPts > SM_EFF_ZERO)
    {
      dDist = sCurPVV[0].DistanceBetween(sNextPVV[0]);
      if (dDist > m_dMaxDist3dBetweenPts)
        {
          reResult = SM_ST_CUT_STEP;
          return SM_SUCCESS;
        }
      if (dDist < m_dMaxDist3dBetweenPts / 1.13)
        {
          reResult = SM_ST_INCREASE_STEP;
        }
      else
        {
          reResult = SM_ST_ACCEPT_STEP;
        }
    }

  // Use a temporary hermite approximation to this segment to
  // compute some of the value of the tolerance.
  SmHermiteCurve sHerm(sCurPVV[0],sCurPVV[1]*dParamStepSize,
                       sNextPVV[0],sNextPVV[1]*dParamStepSize);
  sHerm.SetContext(NULL);

  if (m_dChordHeight > SM_EFF_ZERO)
    {
      double dChordHeight = sHerm.ComputeChordHeight();
      if (dChordHeight > m_dChordHeight)
        {
          reResult = SM_ST_CUT_STEP;
          return SM_SUCCESS;
        }
      if (dChordHeight < m_dChordHeight / 2.0 && reResult != SM_ST_ACCEPT_STEP)
        {
          reResult = SM_ST_INCREASE_STEP;
        }
      else
        {
          reResult = SM_ST_ACCEPT_STEP;
        }
    }

  if (m_dAngTolDeg > SM_EFF_ZERO)
    {
      double dAngleRad;
      if (sCurPVV[1].AngleBetween(sNextPVV[1],dAngleRad) != SM_SUCCESS)
        {
          reResult = SM_ST_ACCEPT_STEP;
          return SM_SUCCESS;
        }
      double dAngleSpan = dAngleRad * 180.0 / SM_PI;
      if (dAngleSpan > m_dAngTolDeg)
        {
          reResult = SM_ST_CUT_STEP;
          return SM_SUCCESS;
        }
      if (dAngleSpan < m_dAngTolDeg / 2.0 && reResult != SM_ST_ACCEPT_STEP)
        {
          reResult = SM_ST_INCREASE_STEP;
        }
      else
        {
          reResult = SM_ST_ACCEPT_STEP;
        }
    }


  // Now test tangent length vs distance and delta t to see if everything
  // is normal.  Othewise we'll cut the step.
  if (dDist < SM_EFF_ZERO) dDist = sCurPVV[0].DistanceBetween(sNextPVV[0]);
  double dRatio = (sCurPVV[1].Length() * dParamStepSize) / dDist;

  if (smos_Fabs(dRatio) < 0.5 || smos_Fabs(dRatio) > 2.0)
    {
      reResult = SM_ST_CUT_STEP;
      return SM_SUCCESS;
    }

  return SM_SUCCESS;

} // end SmCurveTessDriver::TestStep

/*******************************************************************//**
PURPOSE: Tessellate a curve using the Curve Tessellation Driver.

NOTES: The controlling logic of this function is fairly complex.
   There are three different tessellation schemes that may be used.

   Most curves will be tessellated by SmCurve::TessellatebyBisection().
   However it is possible that they can be tessellated by SmCurveCache::Tessellate()
   or SmBSplineCurve::EquallySpacedPoints().

    if(m_bEvalBasedTessellation == FALSE)
      {
        SmCurve::TessellateByBisection() - Generate a set of curve segments
          that pass a set of geometry tests: not too long,
                                             chord height tolerance,
                                             end tangents don't exceed angle tol.
          Segments that fail are iteratively split in half.
          Segments that succeed are extended until they fail, then the last largest
          good segment is added to the output.
      }

    if(   TessellateByBisection failed
       && m_bEvalBasedTessellation == FALSE
       && Curve is a BSpline )
      {
        SmCurveCache::Tessellate() - decompose each BSpline into its natural spans.
           Iteratively split those spans in half until each segment passes
           the geometry tests.  No lengthening of segments is tried.
      }
    if(   Curve is Degenerate)
      {
        return single tessellation point.
      }
    if(   Curve is a BSpline
       && m_dMaxDist3dBetweenPts > 0.0)
      {
        SmBSplineCurve::EquallySpacedPoints()
          find a set of equally spaced points along the curve in Image Space.
      }
    try SmCurve::TessellateByBisection() one more time and quit.
      }

   Note that this is an extremely good although somewhat slow
   tessellator.  You have to be extra careful when dealing with
   evaluator based things.  It still may miss spikes but then
   every one will.

***********************************************************************/
SmStatus SmCurveTessDriver::TessellateCurve
  (const SmCurve       & crCurve,           // in : target curve
   const SmExtent1d    & crInterval,        // in : target interval
   SmTArray<double>    * pParameters,       // out: tessellation parameters
   SmTArray<SmPoint3d> * pPoints)           // out: associated tessellation points, includes curve endPoints
{
  // check input
  if (pParameters == NULL && pPoints == NULL)
    { SER(SM_ERR_INVALID_INPUT); }

  // init output
  if (pParameters) pParameters->ReSet();
  if (pPoints)     pPoints->ReSet();

#ifdef SM_DEBUG_CODE
  SmBoolean bDebugMe = FALSE;
  if (bDebugMe)
    {
      SM_ASSERT_VALID(&crCurve) ;
    }
#endif // SM_DEBUG_CODE

  // Put first into output and last onto stacks
  // locals
  SmPoint3d sCurPVV[3];
//   long nEval = 1;

  // When not using evaluator based tessellation - try tessellation by bisection
  if (!m_bEvalBasedTessellation)
    {
      // find set of largest segments that pass all geometry tests
      //    - not too long
      //    - within tol of chord height
      //    - angle between end tangents within angle tolerance
      if (crCurve.TessellateByBisection(crInterval,
                                        m_dChordHeight,
                                        m_dAngTolDeg,
                                        m_dMaxDist3dBetweenPts,
                                        pParameters,
                                        pPoints) == SM_SUCCESS)
        {
          return SM_SUCCESS;
        }
    }

  // when not using evaluator based tessellation and bisection failed
  //  - try Curve Cache tessellation
  // Modified to appease Linux compiler
  // Do we really need SM_CAST_NONNULL_PTR? ICurveCache constructor works with SmCurve. Why does it have to ba an SmBSplineCurve?
  // Original Code 
  //if (   !m_bEvalBasedTessellation && SM_CAST_NONNULL_PTR(SmBSplineCurve,&crCurve))
  if (!m_bEvalBasedTessellation )
  {
      SmCurveCache sCurveCache(crCurve,
                               crInterval,
                               FALSE,
                               m_dChordHeight,
                               m_dAngTolDeg,
                               0.0,
                               m_lMinSegNumber,
                               TRUE,
                               m_dMaxDist3dBetweenPts,
                               m_dMinParamRatio );

      SER(sCurveCache.Tessellate());

      SER(sCurveCache.GetTessellation(pParameters,pPoints));

      return SM_SUCCESS;
    }

  // Implement a generic curve tessellator which utilizes
  // evaluator based techniques

  // exit case - degenerate curves
  if (crCurve.IsDegenerate())
    {
      if (pParameters) pParameters->Add(crInterval.GetMin());
      if (pPoints) pPoints->Add(sCurPVV[0]);
      return SM_SUCCESS;
    }

  // exit case - NLib BSpline - make equally spaced tessellation
  SmBSplineCurve *pBSC = SM_CAST_NONNULL_PTR(SmBSplineCurve,&crCurve);
  if(   pBSC
     && m_dMaxDist3dBetweenPts > 0.0)
    {
      double dLength = crCurve.ApproximateLength(crInterval,30);
      double dNum    = dLength / m_dMaxDist3dBetweenPts;
      ULONG  lNum    = (ULONG)(dNum + 1.0);
      if (lNum < 2) lNum = 2;
      SER(pBSC->EquallySpacedPoints(crInterval.GetMin(),
                                    crInterval.GetMax(),
                                    lNum,
                                    dLength/1000.0,
                                    pPoints,
                                    pParameters));
      return SM_SUCCESS;
    }

  // find set of largest segments that pass all geometry tests
  //    - not too long
  //    - within tol of chord height
  //    - angle between end tangents within angle tolerance
  if (crCurve.TessellateByBisection(crInterval,
                                    m_dChordHeight,
                                    m_dAngTolDeg,
                                    m_dMaxDist3dBetweenPts,
                                    pParameters,
                                    pPoints) == SM_SUCCESS)
    {
      return SM_SUCCESS;
    }

  // all done
  SE(SM_ERR);
  return SM_ERR;

} // end SmCurveTessDriver::TessellateCurve

/*******************************************************************//**
PURPOSE: Helper function for SmTess::MakePolygon that adds a PolyEdge
         to fill UV PolyEdge End<->Start UVPosition Gaps between
         PolyEdges being added to a PolyLoop that exceed tolerance.

NOTES: when input gap is tighter than tolerance does nothing.

       when filler PolyEdge is added
         1. Adds NewPolyEdge to rNewPolyEdges list
         2. adds association (PrevEndVertex, NewPolyVertex) to sUsedVertices mapping
         3. sets PrevEndUV     = StartUV
                 PrevEndVertex = StartVertex
            so that calling algorithm can still use the
            PrevEndUV and PrevEndVertex values as the last values
            added to the PolyLoop being built.
***********************************************************************/
SmStatus sms_FillGapUVWithPolyEdge
 (SmTess                                * pTess,           // in : SmTess object for this face tessellation
  SmEdgeuse                             * pEdgeuse,        // in : Edgeuse spaned by [sStartUV sEndUV]
  SmPoint3d                             & rPrevEndUV,      // i/o: Last UVPoint in PolyLoop (to connect with rStartUV)
                                                           //      set = rStartUV if filler PolyEdge is added
  SmPoint3d                             & rStartUV,        // in : Tgt  UVPoint to Add next to PolyLoop (to connect with PrevEndUV)
  SmPoint3d                             & rEndUV,          // in : Next UVPoint to Add to PolyLoop after rStartUV
  SmPoint2d                             & rUVTol,          // in : min allowed U and V dist for a gap that needs filling
  SmPolyLoop                            * pPolyLoop,       // in : PolyLoop to receive new PolyVerts and PolyEdges
  double                                  dTolerance,      // in : tolerance to assign new PolyVerts
  SmVertex                             *& rpPrevEndVertex, // i/o: Vertex that maps to rPrevEndUV
                                                           //      set = pStartVertex if filler PolyEdge is added
  SmVertex                              * pStartVertex,    // in : Vertex that maps to rStartUV
  SmTArray<SmPolyEdge*>                 & rNewPolyEdges,   // out: accumulating List of created PolyEdges
  SmMapPtrToPtr<SmVertex, SmPolyVertex> & rUsedVertices)   // i/o: accumulating list of (Vertex,PolyVertex) maps for all created PolyVertices
{
  // locals
  SmVector3d   sGapUV      = rStartUV - rPrevEndUV ;
  SmPolyBrep * pPolyBrepUV = pPolyLoop->GetPolyBrep() ;

  // no work - no gap
  if(   smos_Fabs( sGapUV.x ) < rUVTol.x
     && smos_Fabs( sGapUV.y ) < rUVTol.y )
    { return( SM_SUCCESS ) ; }

  // when PrevSegmentEnd to ThisSegmentStart exceeds tol - add filler PolyEdge when appropriate

  // Special Cases that cause gaps:
  //   Large gap      - Two edges don't meet at one 3d point and the
  //                    3d points project to different UVPoints
  //   Vertex at Pole - one 3d point maps to 2 different UVPoints
  //                    depending on the tangents of the edges
  //                    that come into the PoleVertex.

  // First check to make sure we don't overlap back onto ourselves.
  // The check is if the vector from Start to End doubles back
  // on that from Prev to Start.  We check whether Prev point
  // drops to the segment from Start to End, interior to the
  // segments, and we also check whether End point drops to
  // the segment from Prev to Start, interior to it.
  // Also, we again check u and v components separately, because
  // the u and v domains can be scaled so differently.
  SmBoolean  bAddSeg = TRUE;
    {
      SmVector3d sVecPrev( rPrevEndUV - rStartUV );
      SmVector3d sVecNext( rEndUV  - rStartUV );
      double     dLenSq  = sVecNext.Dot( sVecNext );

      // Get the 2d gap from Prev point to the segment from Start to End.
      double dT = ( dLenSq > SM_EFF_ZERO )
                 ? (sVecPrev.Dot( sVecNext ) ) / dLenSq
                 : 0 ; // (default in case sVecNext is zero)

      // When prev is before the beginning, it's ok.
      // Check the perpendicular gap - if that's small don't add an Edge. gwc???: why this skip?
      // If prev is beyond the end, then we'll check how end projects to Prev vec (next).
      if(  dT > -SM_EFF_ZERO
        && dT < 1.0 + SM_EFF_ZERO )
        {
          sGapUV = sVecPrev - dT * sVecNext; // perp from PrevPt to Start-End segment.
          if(   smos_Fabs( sGapUV.x ) < rUVTol.x
             && smos_Fabs( sGapUV.y ) < rUVTol.y)
            { bAddSeg = FALSE; }
        }

     // when still adding the segment - skip case where perp from EndPt to Prev-Start segment is small
     if ( bAddSeg == TRUE )
        {
          // arrive here when Prev is beyond the end or
          // Prev is not beyond the end and the perp gap exceeds tolerance

          // Get the 2d gap from Next point to the segment from Start to Prev.
          dT = 0; // (default in case sVecPrev is zero)
          dLenSq = sVecPrev.Dot( sVecPrev );
          if ( dLenSq > SM_EFF_ZERO )
            { dT = ( sVecPrev.Dot( sVecNext ) ) / dLenSq; }
          // if end is before the beginning, it's ok.
          // if end is beyond prev, then we just checked it.
          if(   dT > -SM_EFF_ZERO
             && dT < 1.0+SM_EFF_ZERO )
            {
              sGapUV = sVecNext - dT * sVecPrev; // perp from EndPt to Prev-Start segment.
              if(    smos_Fabs( sGapUV.x ) < rUVTol.x
                 &&  smos_Fabs( sGapUV.y ) < rUVTol.y )
                { bAddSeg = FALSE; }
            }
        }
    } // end set bAddSeg value scope

  // when gap is large enough - add extra PolyVertex (for poles) and extra PolyEdge (for Poles and big Gaps)
  if ( bAddSeg == TRUE )
    {
      SmPolyEdge   * pNewPolyEdge ;
      SmPolyVertex * pNewStartPolyVertex, * pNewEndPolyVertex ;

      // Ok, we have a gap between PrevEndUV and StartUV and it doesn't double back
      //     on itself. Add a new filler PolyEdge between PrevEndUV and StartUV.
      // normally for every PrevEdgeuse->rPrevEndUV <-> Edgeuse->rStartUV connection,
      //          1 PolyVertex is added at PrevEdgeuse->rPrevEndUV
      //          1 PolyEdge between rPrevEndUV and next Joint
      // But with a large UV gap add
      //          1 PolyVertex at PrevEdgeuse->rPrevEndUV,     // made with this AddPolyEdge() call
      //          1 PolyVertex at Edgeuse->rStartUV, and       // made with subsequent AddPolyEdge() call
      //          1 PolyEdge between rPrevEndUV and rStartUV   // made with this AddPolyEdge() call
      //          1 PolyEdge between rStartUV and next Joint   // made with subsequent AddPolyEdge() call
      SER(pPolyLoop->AddPolyEdge
           (dTolerance,           // in : min dist between distinct points
            rPrevEndUV,           // in : Line start position
            rStartUV,             // in : Line end position
            NULL,                 // in : when m_pLastEndPolyVertex NotNULL (set on last call through pOptEndPolyVertex),
                                  //           m_pLastEndPolyVertex is Start PolyVertex for PolyLoop->PolyEdge
                                  //      else: pOptStartPolyVertex NotNULL = Start PolyVertex for PolyLoop->PolyEdge,
                                  //                                NULL    = create New PolyVertex for 1stPolyEdge
            NULL,                 // in : NotNULL = stored in m_pLastEndPolyVertex to be
                                  //                Start PolyVertex for next AddPolyEdge() call.
            pPolyBrepUV,          // in : provides context for new obj construction and
                                  //      accumulates new PolyVertices on its m_pVertexListHead list
            pNewPolyEdge,         // out: new edge, stitched to radial partners when pOptStartPolyVertex and pOptEndPolyVertex are NotNULL
            &pNewStartPolyVertex, // out: when given, set to Start PolyVertex for NewEdge (always NonNULL),
                                  //      default:[NULL]
            &pNewEndPolyVertex,   // out: when given, set to End   PolyVertex for NewEdge (NULL except when pOptEndPolyVertex != NULL),
                                  //      default:[NULL]
            rpPrevEndVertex,      // in : Opt PtrVal stored in NewStartPolyVertex::>OriginalVertex,
                                  //      default:[NULL]
            pStartVertex) );      // in : Opt PtrVal stored in NewEndPolyVertex::OriginalVertex
                                  //      only used when pOptEndPolyVertex != NULL,
                                  //      default:[NULL]
      rNewPolyEdges.Add(pNewPolyEdge);

      // gwc: we need to record a SmVertex BackPtr with each PolyVertex
      //      so that later when multiple standalone PolyBreps each containing
      //      one PolyFace are combined into a common PolyBrep model
      //      we can match vertices that share multiple faces to prevent
      //      making duplicate coincident PolyVertices for one SmVertex.

      // remember last PolyVertex mapped from this PrevEndVertex - needed for complexVertex check
      rUsedVertices.Insert(rpPrevEndVertex, pNewStartPolyVertex) ;

      // graphisoft: remember last PolyVertex mapped from this PrevEndVertex - needed for complexVertex check
      pTess->SetPolyEdgeVertexuse (pNewPolyEdge, pEdgeuse->GetVertexuse ());

      // increment rPrevEndUV so subsequent AddPolyEdge call creates NewPolyVertex at rStartUV
      rPrevEndUV      = rStartUV;
      rpPrevEndVertex = pStartVertex ;
    } // end Add Filler PolyEdge check

  // all done
  return(SM_SUCCESS) ;

} // end sms_FillGapUVWithPolyEdge

/*******************************************************************//**
PURPOSE: Make a PolyBrep (with 1 PolyFace) that approximates
         pFace->pLoop->pEdge->UVTrimCurves with UVSpace PolyEdges.

NOTES: 
 1.  1 (or more) PolyLoop is created for each pFace->Loop

 2.  initially, 1 PolyEdge is created for each pFace->UVTrimCurve.
  2a.  Make sure the Face->Edges have been split to a small enough size
       prior to this call.

 3.  PolyEdges are subdivided at all PolyEdge/SurfaceCache SubdivisionBoundary
     intersections or until subdivision size is about smallest surface subdivision node size.
  3a.  m_bAdvancingFront == FALSE, only subdivide PolyEdges created to fill
                                   gaps between loop edges (typically along surface singularities)
  3b.  m_bAdvancingFront == TRUE,  subdivide all PolyEdges created

 4. cases for mapping Loops to PolyLoop(s)
   normal case : map Edgeuses and Vertices to PolyEdges and PolyVertices.
                                     +-----+       +-----+
                                      \   /   ==>   \   /
                                       \ /           \ /
                                        +             +
                      UV Edgeuses and Vertices       PolyEdges and PolyVertices

   normal case: Cylinder  <----->              +------+             +------+
                           \   /               |      |             |      |
            circular top    \ /                |      |             |      |      In 3Space the vertices are
                             +  Shared         |      |             |      |      shared, but not in UVSpace
            Seam = pair of   "  Vertex         |      |             |      |      So, 1 3dVertex can map
            linear edgeuses  "          ==>    |      |     ==>     |      |      to any number of PolyVertices
                             +  Shared         |      |             |      |
                            / \  Vertex        |      |             |      |
            circular bot   /   \               |      |             |      |
                          <----->              +------+             +------+
                         3D Space       UV Edgeuses and Vertexuse    PolyEdges and PolyVertices

   normal case: Sphere                            missing
                                            singular segments         +======+
       Singularity                             { +      + }           |      |  In 3Space the vertices are
                at top->   <+>   Shared          |      |             |      |  shared, and there are no edgeuses
                            "    Vertex          |      |             |      |  along the singular boundaries.
       seam = pair of       "                    |      |             |      |  So, 1 3dVertex can map to
       linear edgeuses->    "             ==>    |      |     ==>     |      |  any number of PolyVertices and
                            "                    |      |             |      |  PolyEdges have to be added across the
       singularity          "    Shared          |      |             |      |  missing segment boundaries
                at bot->   <+>   Vertex        { +      + }           +======+
                        3D Space         UV Edgeuses and Vertices    PolyEdges and PolyVertices

   special case: Gap in Loop not at a pole  (an illegal face but common enough input problem to deal with)
                                     +---------+                    +--------+
                                      \       /       ==>            \      /
                                       \     /                        \    /
                                        +   + Tol exceeding gap        +==+   Tol exceeding gap
                           UV Edgeuses and Vertices              PolyEdges and PolyVertices


   normal case: Vertex at a pole   +-----+                 +--------+
                                    \   /        ==>        \      /
                                     \ /                     \    /
                                      + <-Pole              { +==+ } <-Pole
                         UV Edgeuses and Vertices       PolyEdges and PolyVertices

                 When a vertex is at a pole it will typically yield multiple
                 PolyVertices per vertex with an 'extra' PolyEdge between
                 each pair of PolyVertices generated.  The simplest
                 case will have 1 PoleVertex => 2 PolyVertices + 1 'extra' PolyEdge.

   In all cases a walk-the-loop-edgeuses-in-sequence 
   building PolyVertices and PolyEdges, and splitting
   loops when needed looks like it can be made to map
   Loops to PolyLoops.

   Special case handling for Complex Vertices has been removed. See original comments in code.
   Removing topological edges has also been elimated, as a topological Edge in one face may
   be a, e.g., spine edge.  [B672]

Assume Brep PolyLoops do not have coincident vertices then:

{
BeginPolyLoop
For Every Edgeuse
  { Get PrevEndUVPoint, PrevEndOrigVertex
    Get StartUVPoint,   StartOrigVertex
    Get EndUVPoint,     EndOrigVertex

    if(Gap(PrevEndUVPoint, StartUVPoint) > tol)
      { Fill Gap ; // its possible to add a PolyEdge to complex Pole loops that won't be needed in the end here
        PrevEndUVPoint    = StartUVPoint ;
        PrevEndOrigVertex = StartOrigVertex ;
        map(PrevEndPolyVertex, PrevEndOrigVertex) pair
      }

    PolyLoop->AddPolyEdge(PrevEndUVPoint, // not StartUVPoint - remove less than tol sized gaps
                          PrevEndOrigVertex,
                          EndUVPoint,
      {  PrevEndPolyVertex = MakePolyVertex(PrevEndOrigVertex) ;
         NewPolyEdge = MakePolyEdge StartingAt(PrevEndPolyVertex) ;
         PolyLoop->AddEdge(NewPolyEdge)
      }

    // check for complex vertex - PolyVert exists for EndOrigVertex split loop
    if(   NotLastEdgeuse
       && OldEndPolyVert = MapFrom(EndOrigVertex)
       && (   OldEndPolyVert.UVPoint == EndUVPoint
           || EndOrigVertex->IsSingular(OldEndPolyVert.UVPoint == EndUVPoint Gap Direction))
      {
         if(Gap(EndUVPoint, OldEndPolyVert.UVPoint) > tol)
           { Fill Gap ; // its possible to add a PolyEdge to complex Pole loops that won't be needed in the end here
             PrevEndUVPoint    = EndUVPoint ;
             PrevEndOrigVertex = EndOrigVertex ;
           }

         pPolyLoop->SplitPolyLoop(at OldEndPolyVert, pNewClosedPolyLoop)

         if(Last edge on OldLoop is a FillEdge on Singularity)
           {
             Remove Fill Edge
             PrevEndPolyVertex = pPolyLoop->GetSize > 0 ? OldStartPolyVertex : GetLastEndVertex;
             PrevEndUVPoint    = pPolyLoop->GetSize > 0 ? OldStartUVPoint    : GetLastEndUVPoint:
           }
         else
           {
             PrevEndPolyVertex = OldEndPolyVertex ;
             PrevEndUVPoint    = OldEndUVPoint ;
           }
         PrevEndOrigVertex = EndOrigVertex ;
      }

    map(PrevEndPolyVertex, PrevEndOrigVertex) pair
  }
FinishPolyLoop
}

***********************************************************************/
SmStatus SmTess::MakePolygon
  (SmFace      * pFace,        // in : face to approximate as a polygon
   double        dTolerance,   // in : 3d tol used for creating Poly topo
   SmPolyBrep *& rpPolyBrep,   // out: UV PolyBrep containing
                               //        1 SmPolyFace per simple outer pFace->Loop
                               //            (complex pFace->Loops are broken into simple Loops yielding a few PolyFaces)
                               //        1 inner SmPolyLoop for each inner pFace->Loop,
                               //        1 SmPolyEdge for each pFace->Loop->Edge,
                               //        1 SmPolyVertex for each pFace->Loop->Vertex.
                               //        No Inner PolyVerts or PolyEdges at this time.
                               //        PolyLoops not broken up into triangles at this time.
   SmPolyFace *& rpPolyFace)   // out: UV PolyFace polygon approximation to input pFace
{
#ifdef SM_DEBUG_CODE
    SmBoolean bDebugMe = FALSE;
#endif // SM_DEBUG_CODE

  // locals
  ULONG ii, jj;
  SM_PTR_ARRAY(sLoopuses,      SmLoopuse,  16);   // SmTArray<SmLoopuse*>
  SM_PTR_ARRAY(sEdgeuses,      SmEdgeuse,  64);   // SmTArray<SmEdgeuse*>
  SM_PTR_ARRAY(sNewPolyEdges,  SmPolyEdge, 64);   // When m_bAdvancingFront == FALSE
                                                  //        sNewPolyEdges contains only edges created to fill gaps between loop edgeuses.
                                                  //      m_bAdvancingFront == TRUE
                                                  //        sNewPolyEdges contains all edges added to SmPolyFace
                                                  // for every sNewPolyEdges Edge - see if it needs to be split at Cache node boundaries
  SM_PTR_ARRAY(sEdges,         SmEdge,     64) ;  // SmTArray<SmEdge*>
  SM_PTR_ARRAY(sRepeatedEdges, SmEdge,     64) ;  // SmTArray<SmEdge*> Edges with multiple Edgeuses in a given Loopuse
  SM_PTR_ARRAY(sPolyLoops,   SmPolyLoop,   16) ;
  SmMapPtrToPtr<SmVertex, SmPolyVertex> sUsedVertices ;
  SmMapTypeToType<SmEdge*, SmPolyEdge*> sUsedEdges    ;    // map [SmEdge   SmPolyEdge]   pairs for RepeatedEdges
  SmContext         * pContext = new SmContext ; //  Each pPolyBrepUV gets its own Context, to be able to use Marks in parallel code
  SmTessSrfCache    * pSC      = m_vCache.GetSecond(pFace) ; NER(pSC);
  SmSurface         * pSurface = pFace->GetSurface();
  SmFaceuse         * pUpFU    = pFace->GetUpwardFaceuse();
  SmExtent2d          sExpandedDomain = pSurface->GetNaturalUVDomain().ExpandRelative(2);
  pUpFU->GetLoopuses( sLoopuses );
  pFace->GetEdges   ( sEdges ) ;

  // output: PolyBrepUV to hold 1 PolyFace,
  //                            1 (or more) PolyLoop for each SmLoop,
  //                            1 PolyEdge for each SmEdgeuse,
  //                            1 (or more) PolyVertex for each SmVertex
  SmPolyBrep * pPolyBrepUV = new ( *pContext ) SmPolyBrep(pFace->GetBrep()->GetTolerance());
  SmObjDelete sCleanBrep(pPolyBrepUV);
  pPolyBrepUV->SetOwnsContext(TRUE) ;
  pPolyBrepUV->SetOKBackPtrs(TRUE) ;   // the original Brep will last as long as the cache

  // output: PolyFaceUV
  SmVector3d   sZ(0,0,1) ;
  SmPolyFace * pPolyFaceUV = new (pPolyBrepUV) SmPolyFace(dTolerance, pPolyBrepUV, NULL, NULL, &sZ);
  pPolyFaceUV->SetOriginalFace(pFace, TRUE) ; // TRUE = copy pFace->Attributes onto pPolyFaceUV

  // for every loopuse - build 1 (or more) PolyLoop in PolyFaceUV
  for (ii=0; ii<sLoopuses.GetSize(); ii++)
    {
      SmLoopuse * pLoopuse = sLoopuses[ii];
      SmBoolean   bSplitPolyLoop = FALSE ; // When a loop is split at a complex vertex - have to recompute LoopOrientation

      // EdgeLoop branch
      if(pLoopuse->IsEdgeLoopuse())
        {
          pLoopuse->GetEdgeuses(sEdgeuses);

#ifdef SM_DEBUG_CODE
          if (bDebugMe)
            {
              pLoopuse->Dump() ;

              TCHAR sBuff[SM_TBLOCK_SIZE] ;
              smos_sprintf(sBuff, _T("\nSmTess::MakePolygon: EdgeuseCnt:[%ld]"),sEdgeuses.GetSize());
              smos_WriteBuffer(sBuff) ;

              for(ULONG di=0;di<sEdgeuses.GetSize();di++)
                {
                  SmEdgeuse * pDbgEdgeuse     = sEdgeuses[di] ;
                  SmVertex  * pDbgStartVertex = pDbgEdgeuse->GetVertexuse()->GetVertex() ;
                  SmVertex  * pDbgEndVertex   = pDbgEdgeuse->GetEdge()->GetOtherVertex(pDbgStartVertex) ;
                  SmPoint3d   sDbgStartUV, sDbgEndUV ;
                  SER(pDbgEdgeuse->NormalizedEvaluate( 0.0, TRUE, sDbgStartUV )); // TRUE = UV Eval, FALSE = 3d Eval
                  SER(pDbgEdgeuse->NormalizedEvaluate( 1.0, TRUE, sDbgEndUV ));   // TRUE = UV Eval, FALSE = 3d Eval

                  // Start
                  smos_sprintf(sBuff, _T("\n [%3ld] Edgeuse:[0x%p], StartVertex:[0x%p], StartPoint:"),di,pDbgEdgeuse,pDbgStartVertex);
                  smos_WriteBuffer(sBuff);
                  sDbgStartUV.Dump() ;
                  pDbgStartVertex->GetPoint().Dump() ;

                  smos_sprintf(sBuff, _T("\n                                     EndVertex  :[0x%p], EndPoint  :"),pDbgEndVertex);
                  smos_WriteBuffer(sBuff);
                  sDbgEndUV.Dump() ;
                  pDbgEndVertex->GetPoint().Dump() ;
                }
            }
#endif // SM_DEBUG_CODE

          // locals
          SmPolyLoop   * pPolyLoop           = NULL ;  // ptr to PolyLoop to be built from this Loopuse
          SmEdgeuse    * pPrevEdgeuse        = NULL ;

          SmVertex     * pPrevStartVertex    = NULL ;
          SmVertex     * pPrevEndVertex      = NULL ;
          SmVertex     * pStartVertex        = NULL ;
          SmVertex     * pEndVertex          = NULL ;

          SmPolyVertex * pNewStartPolyVertex = NULL ;
          SmPolyVertex * pNewEndPolyVertex   = NULL ;

          SmPoint3d      sPrevEndUV ;
          SmPoint3d      sStartUV ;
          SmPoint3d      sEndUV ;
          SmPoint2d      sStartUV2d;
          SmPoint2d      sEndUV2d;
          SmPoint2d      sPrevEndUV2d;


          ULONG          lNumEdgeuses = sEdgeuses.GetSize();
          ULONG          lIndex ;

          // skip problem loops with fewer than 3 edges
          if (sEdgeuses.GetSize() < 3)
            { continue; }

          // Pick DistTolUV tolerance: sUVTol:[dDistTolU, dDistTolV]
          SmVector2d sUVTol ;
            {
              // We need 2d tolerance, not the 3d tolerance that is passed in.
              // But, we don't just want to use a 2d verison of the passed-in
              // 3d tol, because of the way this is used:
              // These happen at poles and across seams: the loops in the SmBrep representation are
              // not necessarily closed in uv space, they can jump right across poles.
              // So, we will just use a fraction of the domain size.
              // This is rather arbitrary, but it captures the meaning:
              // looking for big gaps such as what happens at poles.
              // [B231]

              // Also, work with u and v separately: for poles it will be one or
              // the other, and domains can be very lopsided (e.g., [0,1] and [0,360]).
              // Also, the surface's domain is more meaningful here than the Face's domain.
              sUVTol = pSurface->GetNaturalUVDomain().GetSize();

              // Use 1/1000 of the domain size.  (Note, 1/100 is too big [B241].)
              sUVTol /= 1000.0;

            } // end Get Tolerance scope

          // init empty PolyLoop
          SER(pPolyFaceUV->StartPolyEdgeLoop(pPolyLoop));   // Construct empty PolyLoop
          pPolyLoop->m_eOrientation =   (ii > 0)        // The first outer loop orientation = SM_OT_SAME,
                                      ? SM_OT_OPPOSITE  // subsequent inner loop orienations are SM_OT_OPPOSITE
                                      : SM_OT_SAME ;

          // Find Edges that are used multiple times in this loop (e.g. struts)
          sRepeatedEdges.SetSize( lNumEdgeuses );
          for ( jj = 0; jj < lNumEdgeuses; ++jj )
          { sRepeatedEdges[jj] = sEdgeuses[jj]->GetEdge(); }
          sRepeatedEdges.GetDuplicates(sRepeatedEdges);

          // init 'Prev' locals to last Edgeuse for first iteration
          pPrevEdgeuse     = sEdgeuses.GetLast() ;
          SmBSplineCurve* pPrevUVTrim = pPrevEdgeuse->GetUVTrimCurve();
          pPrevStartVertex = pPrevEdgeuse->GetVertexuse()->GetVertex() ;
          pPrevEndVertex   = pPrevEdgeuse->GetEdge()->GetOtherVertex(pPrevStartVertex) ;
          // GetCCWEdgeuse() is NULL for a wire edgeuse (SmEdgeuse.h); a wire edge
          // in this loop is not representable as a polygon, so fail the face
          // (SM_ERR_FATAL, handled by Phase1SetupBrep).
          SmEdgeuse* pPrevCCWEdgeuse = pPrevEdgeuse->GetCCWEdgeuse();
          AERN_MSG(pPrevCCWEdgeuse != NULL, SM_ERR_FATAL,
                   _T("SmTess::MakePolygon: loop edgeuse has no CCW neighbor (wire edge in loop)"));
          sPrevEndUV = pPrevCCWEdgeuse->GetVertexuse()->GetUVPoint();

          SM_ASSERT(pPrevEndVertex == pPrevCCWEdgeuse->GetVertexuse()->GetVertex());

          if (pPrevUVTrim || sPrevEndUV.x == SM_BIG_DOUBLE)
          {
              SER(pPrevEdgeuse->NormalizedEvaluate(1.0, TRUE, sPrevEndUV));// TRUE = UV Eval, FALSE = 3d Eval
          }

          // init iter locals as if iter had been done on LastEdgeuse
          pEndVertex     = pPrevEndVertex ;
          sEndUV         = sPrevEndUV ;

          // for every loop->Edgeuse insert a PolyEdge into the SmPolyFace
          for (jj=0; jj<lNumEdgeuses; jj++)
            {
              SmEdgeuse  * pEdgeuse     = sEdgeuses[jj];
              SmPolyEdge * pNewPolyEdge = NULL ;
              SmEdge     * pEdge        = pEdgeuse->GetEdge();
              SmBSplineCurve* pUVTrim = pEdgeuse->GetUVTrimCurve();

              //SmZoneTol3d  sZoneTol3d   = SmTol::GetZoneTol3d( pEdge );

              // remember last iter locals
              pPrevEndVertex  = pEndVertex ;
              sPrevEndUV      = sEndUV ;

              // this iter locals
              pStartVertex      = pEdgeuse->GetVertexuse()->GetVertex() ;
              pEndVertex        = pEdge->GetOtherVertex(pStartVertex) ;
              sStartUV2d = pEdgeuse->GetVertexuse()->GetUVPoint();
              // Same wire-edgeuse guard as above.
              SmEdgeuse* pCCWEdgeuse = pEdgeuse->GetCCWEdgeuse();
              AERN_MSG(pCCWEdgeuse != NULL, SM_ERR_FATAL,
                       _T("SmTess::MakePolygon: loop edgeuse has no CCW neighbor (wire edge in loop)"));
              sEndUV2d = pCCWEdgeuse->GetVertexuse()->GetUVPoint();

              sStartUV.x = sStartUV2d.x;
              sStartUV.y = sStartUV2d.y;
              sStartUV.z = 0.0;

              sEndUV.x = sEndUV2d.x;
              sEndUV.y = sEndUV2d.y;
              sEndUV.z = 0.0;

              SM_ASSERT(pEndVertex == pCCWEdgeuse->GetVertexuse()->GetVertex());

              if (pUVTrim || sStartUV.x == SM_BIG_DOUBLE ||
                  sEndUV.x == SM_BIG_DOUBLE)
              {
                  SER(pEdgeuse->NormalizedEvaluate(0.0, TRUE, sStartUV)); // TRUE = UV Eval, FALSE = 3d Eval
                  SER(pEdgeuse->NormalizedEvaluate(1.0, TRUE, sEndUV)); // TRUE = UV Eval, FALSE = 3d Eval
              }

#ifdef SM_DEBUG_CODE
              if (bDebugMe)
                {
                  SM_ASSERT_VALID( pPolyLoop );
                  SM_DUMP( pPolyBrepUV );

                  SmVector3d sVec = sEndUV-sStartUV;

                  if (ii==0 && jj==0)
                    { smgfx_Erase(); }
                  smgfx_SetLook( 2,4, 0,0,0); sStartUV. Draw(); sm_GraphicsLoop();
                  smgfx_SetLook( 2,4, 0,0,1); sEndUV.Draw(); sm_GraphicsLoop();
                  //smgfx_SetLook( 2,4, 0,1,0); sVec.Draw(&sStartUV); sm_GraphicsLoop();
                  smgfx_SetLook( 3,5, 1,0,0); sPrevEndUV.Draw(); sm_GraphicsLoop();
                  smgfx_SetLook( 1, 2, 0, 0, 0 ); pPolyFaceUV->Draw(); sm_GraphicsLoop();
                  sm_GraphicsLoop();
                }
#endif // SM_DEBUG_CODE

              // This check for UV's near the surface doesn't fire in prog_test.
              //  In the associated wire file, 1 in every 1000 surfaces fails to
              //  tessellate because of the following AER.
              //  When time permits, it would behoove us to check those failing tessellations.
              //  In the cases we have checked, the UVTrimCurves provided by Alias are a couple
              //  orders of magnitude outside the domain of the associated surfaces.
#ifdef SM_DEBUG_CODE
              if (   !sExpandedDomain.ContainsPoint2d(sStartUV.Get2dXY())
                  || !sExpandedDomain.ContainsPoint2d(sEndUV.Get2dXY()))
              {
                  TCHAR sBuff[SM_MAXSIZE];
                  SmExtent2d sDomain = pSurface->GetNaturalUVDomain();
                  SM_SPRINTF(sBuff, _T("\n UV domain: [%f, %f] X [%f, %f], StartUV: (%f, %f), EndUV: (%f, %f)"),
                                sDomain.GetMin().x, sDomain.GetMax().x,
                                sDomain.GetMin().y, sDomain.GetMax().y,
                                sStartUV.x, sStartUV.y,
                                sEndUV.x, sEndUV.y);
                  SM_DBG_WARN(sBuff);
              }
#endif
              AERN_MSG(sExpandedDomain.ContainsPoint2d(sStartUV.Get2dXY()) && sExpandedDomain.ContainsPoint2d(sEndUV.Get2dXY()), SM_ERR_FATAL, _T("Edgeuse->UVPoint far outside surface domain."));

              // 1. if sPrevEndUV to sStartUV Gap exceeds tol- add filler PolyEdge(sPrevEndUV, sStartUV)
              //                                               and set sPrevEndUV = sStartUV (for next step)
              sms_FillGapUVWithPolyEdge
                 ( this,                   // in : SmTess object for this face tessellation
                   pEdgeuse,               // in : Edgeuse spaned by [sStartUV sEndUV]
                   sPrevEndUV,             // i/o: Last UVPoint in PolyLoop (to connect with sStartUV)
                                           //      set = sStartUV if filler PolyEdge is added
                   sStartUV,               // in : Tgt  UVPoint to Add next to PolyLoop (to connect with PrevEndUV)
                   sEndUV,                 // in : Next UVPoint to Add to PolyLoop after sStartUV
                   sUVTol,                 // in : min allowed U and V dist for a gap that needs filling
                   pPolyLoop,              // in : PolyLoop to receive new PolyVerts and PolyEdges
                   dTolerance,             // in : tolerance to assign new PolyVerts
                   //sZoneTol3d,           // in : tolerance to assign new PolyVerts
                   pPrevEndVertex,         // i/o: Vertex that maps to sPrevEndUV
                                           //      set = pStartVertex if filler PolyEdge is added
                   pStartVertex,           // in : Vertex that maps to sStartUV
                   sNewPolyEdges,          // out: accumulating List of Gap-Filling created PolyEdges to be checked for further subdivison later
                   sUsedVertices) ;        // i/o: accumulating list of (Vertex,PolyVertex) maps for all created PolyVertices

              // 2. add PolyEdge for this pEdgeuse
              //     from PrevEndUV to EndUV instead of from StartUV to EndUV.
              //     Using PrevEndUV eliminates any tolerance sized gaps between polyedges within the polyloop.
              if(SM_SUCCESS == pPolyLoop->AddPolyEdge
                  (dTolerance,           // in : min dist between distinct points
                  //(sZoneTol3d,           // in : min dist between distinct points
                   sPrevEndUV,           // in : Line start position
                   sEndUV,               // in : Line end position
                   NULL,                 // in : pOptStartPolyVertex: when m_pLastEndPolyVertex NotNULL (set on last call to pOptEndPolyVertex),
                                         //           m_pLastEndPolyVertex is Start PolyVertex for PolyLoop->PolyEdge
                                         //      else: pOptStartPolyVertex NotNULL = Start PolyVertex for PolyLoop->PolyEdge,
                                         //                                NULL    = create New PolyVertex for 1stPolyEdge
                   NULL,                 // in : pOptEndPolyVertex: NotNULL = stored in m_pLastEndPolyVertex to be
                                         //                Start PolyVertex for next AddPolyEdge() call.
                   pPolyBrepUV,          // in : provides context for new obj construction and
                                         //      accumulates new PolyVertices on its m_pVertexListHead list
                   pNewPolyEdge,         // out: new edge, stitched to radial partners when pOptStartPolyVertex and pOptEndPolyVertex are NotNULL
                   &pNewStartPolyVertex, // out: when given, set to Start PolyVertex for NewEdge (always NonNULL),
                                         //      default:[NULL]
                   &pNewEndPolyVertex,   // out: when given, set to End   PolyVertex for NewEdge (NULL except when pOptEndPolyVertex != NULL),
                                         //      default:[NULL]
                   pPrevEndVertex,       // in : Opt PtrVal stored in NewStartPolyVertex::>OriginalVertex,
                                         //      default:[NULL]
                   pEndVertex))          // in : Opt PtrVal stored in NewEndPolyVertex::OriginalVertex
                                         //      only used when pOptEndPolyVertex != NULL,
                                         //      default:[NULL]
                {
                  // when m_bAdvancingFront == TRUE
                  if (m_bAdvancingFront)
                    {
                      // also add PolyEdges replicating Edgeuses to the list of PolyEdges to be checked for further subdivision
                      sNewPolyEdges.Add(pNewPolyEdge);
                    }

                  // remember last PolyVertex mapped from this PrevEndVertex - needed for complexVertex check
                  sUsedVertices.Insert(pPrevEndVertex, pNewStartPolyVertex) ;

                  // remember the NewPolyEdge source Edgeuse in the NewPolyEdge->m_pEdgeuse value
                  SetPolyEdgeEU(pNewPolyEdge,pEdgeuse);

                  // remember the NewPolyEdge source Edge when we expect to see this edge again
                  if ( sRepeatedEdges.FindElement( pEdge, lIndex ) )
                    { sUsedEdges.AddUnique( pEdge, pNewPolyEdge ); }
                }

              // Obsolete. Do not split loops at ComplexVertices. 
               // special case: complex loops ('8') yield multiple polyLoops (all outer) per Loop.
               //
               //                   +-----+                  +-----+
               //                    \   /        ==>         \   /
               //                     \ /                      \ /
               //                      + <-Complex Vertex     { + } <-Complex Vertex
               //                     / \                     { + }   maps to multiple
               //                    /   \                     / \    PolyVertices - all coincident.
               //                   +-----+                   /   \   Make one PolyVertex and
               //            UV Edgeuses and Vertices        +-----+  use it multiple times.
               //
               //                 A complex loop is detected by coming across the same Vertex
               //                 more than once while walking the Loop.  note: All Edges
               //                 that share a common vertex are combined into a single loop,
               //                 so every time a walk of a Loop hits the same vertex at a common UV Point
               //                 will always be a complex loop.  Use this fact to code for
               //                 complex loops.  In the simple case, if a vertex is hit a second
               //                 time, close the current loop, and continue with the loop
               //                 being built before that. If no loop was being built - start
               //                 a new one.  This is easy to implement with a LastInFirstOut loop
               //                 stack for management.
               //
               //   special case: A complex loop that decomposes to a loop within another loop
               //                +---+   +---+            +---+   +---+    output two PolyLoops
               //                |    \ /    |    ==>     |    \ /    |       (1 outer and 1 inner loop)
               //                | in  +  <------Complex  |   { + }   |  <-Complex Vertex
               //                |    / \ in |   Vertex   |   { + }   |    maps to multiple
               //                |   /out\   |out         | in / \    |    PolyVertices - all coincident.
               //             out|  +-----+  |            |   /out\   |    Make one PolyVertex and
               //                |     in    |            |  +-----+  |    use it multiple times.
               //                +-----------+            +-----------+
               //         UV Edgeuses and Vertices     PolyEdges and PolyVertices
               //                 When multiple PolyLoops are made from one Loop
               //                 containment of one loop within another will have to be detected.
               //
               //   special case: A complex loop that decomposes to a loop within another loop connected by a pair of edgeuses
               //                +---+   +---+            +---+   +---+
               //                |    \ /    |    ==>     |    \ /    |
               //                | in  +  <------Complex  |     +     |  <-Complex Vertex
               //                |     "     |   Vertex   |           |      maps to one PolyVertex
               //                |     " <----- Edge used |       <--------topological edge does not add any PolyEdges
               //                |     "     |    twice   |           |
               //                |     +  <------Complex  |           |  <-Complex Vertex
               //                |    / \ in |   Vertex   |     +     |    maps to one PolyVertex
               //                |   /out\   |out         | in / \    |
               //             out|  +-----+  |            |   /out\   |
               //                |     in    |            |  +-----+  |
               //                +-----------+            +-----------+
               //         UV Edgeuses and Vertices     PolyEdges and PolyVertices
               //                 When multiple PolyLoops are made from one Loop
               //                 containment of one loop within another will have to be detected.
               //                 The topological Edge will generate a degenerate polyLoop that gets
               //                   detected after construction and then deleted.  Once that PolyLoop
               //                   is deleted this is basically the same case as A complex loop that
               //                   decomposes to a loop within another loop.
               //
               //   special case: compounded complex loops of the form: are handled by the same
               //                  +-----+   +-----+    +-----+     +-----+    FIFO loop stack management algorithm!
               //                   \    |   |    /      \    |     |    /
               //                    \    \ /    /        \    \   /    /
               //                     +----+----+   ==>    +----+ +----+
               //                         / \ Complex            +  maps to
               //                        /   \  Vertex          / \  multiple PolyVertices - These are all
               //                       +-----+                /   \  coincident - create one PolyVertex
               //              UV Edgeuses and Vertices       +-----+         and use it once per simple PolyLoop
               //
               //   special case: complex Pole Loops - the Pole vertex is a complex vertex within a complex loop.
               //                                      (a legal face - worth supporting)
               //
               //       +-----+             6 possible missing segs                       This is handled by the FIFO loop
               //        \   /                only 2 correct ones                           management stack and splitting.
               //         \ /                     { +  + +  + }            { +==+ +==+ } <-Pole can map
               //         <+> <-Pole      ==>      /   | |   \       ==>    /   | |   \    to 4 PolyVertices
               //         / \    Shared           /    | |    \            /    | |    \       The PolyVertices are coincident wherever
               //        /   \    Vertex         +-----+ +-----+          +-----+ +-----+      more than one edge approaches the pole
               //       +-----+               UV Edgeuses and Vertices                         from the same direction. (it's possible)
               //      3D Space                                                            1. break complex loop into set of simple loops
               //                                                                          2. Add PolyEdges across missing segment boundaries,
               //    This is a hard case to detect because when                            3. check for coincidence UVPoints and make only
               //       looks a lot like the normal cylinder and                              one PolyVertex that gets used multiple times
               //       sphere cases when walking the sequence of                             for every found coincident pair.
               //       edgeuses arrives at the Shared Vertex Pole.
               //
               //     This is a hard case to distinguish from the          In all cases a walk-the-loop-edgeuses-in-sequence
               //     cylinder and sphere cases. All three cases           building PolyVertices and PolyEdges, and splitting
               //     map single vertices to multiple PolyVertices.        loops when needed looks like it can be made to map
               //     For cylinders there are no missing                   Loops to PolyLoops.
               //     segements and every edgeuse start (or end)
               //     maps to a different polyVertex. For spheres
               //     segments are missing and have to be filled
               //     with added PolyLines whose Poly vertex locations
               //     are determined from existing Edgeuse start
               //     and end UVPositions.  There are no coincident
               //     PolyVertices to be shared. For Complex poles
               //     some missing segments are filled with
               //     added PolyEdges while others are split
               //     points for creating independent loops.

               //Special case: Undetected Complex loop
             
               /*                +-----+                                          */
               /*                 \   /        ==> Not yet handled                */
               /*                  \ /                                            */
               /*                   X <-Complex location where edges intersect    */
               /*                  / \  but is missing a vertex to mark the spot. */
               /*                 /   \                                           */
               /*                +-----+                                          */
               /*         UV Edgeuses and Vertices                                */

//              // 3. for internal edges - check for ComplexVertex - they spawn additional PolyLoops
//              if(jj < sEdgeuses.GetSize() - 1)
//                {
//                  // A ComplexVertex is a vertex that gets used more than once in a single loop which
//                  // happens for loops which come back upon themselves as in a figure eight, that's a
//                  // problem in UVSpace but not always in 3d Space.
//                  // Loops for Cylinders, Cones, and Tori are complex in 3d space (where the
//                  // loop runs back to and then along the seam) but not in UV space where
//                  // the loop is the natural boundary of the UVDomain, i.e. a rectangle.
//                  // Loops for an annulus made on plane could be represented by an outer and inner loop pair,
//                  // or by a single loop when those two loops are connected by an Edge and a pair of Edgeuses.
//                  // A simple annulus trimmed from a plane with just one loop will have two complex vertices
//                  // connected by a pair of edgeuses in UV space which should be replicated with a pair of PolyVertices
//                  // and a pair of PolyEdges.
//
//                  // Fetch the last PolyVertex mapped from this EndVertex (notNULL only for ComplexVertices).
//                  //  Because sUsedVertices only stores the last PolyVertex associated with pEndVertex
//                  //  that will be the start of the PolyEdge that begins this portion of PolyLoop that
//                  //  forms a closed loop at this ComplexVertex and not the EndPolyVertex of the previous
//                  //  portion of the PolyLoop.  (When ComplexVertices are at poles
//                  //  one Vertex can map to a set of PolyVertices with different UV values otherwise
//                  //  ComplexVertices all map to just one PolyVertex and this issue is moot.)
//                  SmPolyVertex  * pOldStartPolyVertex  = (SmPolyVertex *)sUsedVertices.GetValueAt(pEndVertex) ;
//                  SmBoolean       bCheckComplexVertex  = FALSE ;
//
//                  // a Vertex can be complex when it's used multiple times in a single loop
//                  if ( pOldStartPolyVertex != NULL && pOldStartPolyVertex->GetPoint().IsInitialized() )
//                    {
//                      SmVector3d sOldStart    = pOldStartPolyVertex->GetPoint() ;
//                      SmVector3d sOldToNewGap = sOldStart - sEndUV ;
//
//                      // a Vertex is complex when its multiple uses happen at a single UV point
//                      if(smos_Fabs(sOldToNewGap.x) < SM_EFF_ZERO && smos_Fabs(sOldToNewGap.y) < SM_EFF_ZERO)
//                        {
//                          bCheckComplexVertex = TRUE ;
//                        }
//
//                      // a vertex is also complex when its multiple uses happen along a singular boundary of a surface
//                      // but not across a seam
//                      else if(smos_Fabs(sOldToNewGap.x) < SM_EFF_ZERO || smos_Fabs(sOldToNewGap.y) < SM_EFF_ZERO)
//                        {
//                          // Surface natural boundaries
//                          SmExtent2d sUVDomain = pSurface->GetNaturalUVDomain() ;
//
//                          // classify the UVPoints
//                          SmPoint2d sOldStartUV(sOldStart.x, sOldStart.y) ;
//                          SmPoint2d sEndUV2d(sEndUV.x, sEndUV.y) ;
//                          SmExtentPointType eTypeU1, eTypeV1, eTmp ;
//                          SmExtentPointType eTypeU2, eTypeV2 ;
//                          sUVDomain.ClassifyPoint2d(sOldStartUV, eTypeU1, eTmp, sUVTol.x) ;   // eU1 using Udir Tol
//                          sUVDomain.ClassifyPoint2d(sOldStartUV, eTmp, eTypeV1, sUVTol.y) ;   // eV1 using Vdir Tol
//                          sUVDomain.ClassifyPoint2d(sEndUV2d, eTypeU2, eTmp, sUVTol.x) ;  // eU2 using Udir Tol
//                          sUVDomain.ClassifyPoint2d(sEndUV2d, eTmp, eTypeV2, sUVTol.y) ;  // eV2 using Vdir Tol
//
//                          // check for Points across a seam pair - they are not a complex vertex
//                          if(   (eTypeU1 == SM_EP_START && eTypeU2 == SM_EP_END)
//                             || (eTypeU1 == SM_EP_END   && eTypeU2 == SM_EP_START)
//                             || (eTypeV1 == SM_EP_START && eTypeV2 == SM_EP_END)
//                             || (eTypeV1 == SM_EP_END   && eTypeV2 == SM_EP_START))
//                            {
//                              bCheckComplexVertex  = FALSE ;
//                            }
//                        }
//                    } // end computing bCheckComplexVertex value
//
//                  // when EndVertex is a ComplexVertex - split off a closed PolyLoop and continue building the original PolyLoop
//                  if(bCheckComplexVertex)
//                    {
//#ifdef SM_DEBUG_CODE
//                      if(bDebugMe)
//                        { SM_ASSERT_VALID(pPolyLoop) ; }
//#endif // SM_DEBUG_CODE
//                      // locals
//                      SmPoint3d    sOldStartUV   = pOldStartPolyVertex->GetPoint() ;
//                      SM_PTR_ARRAY(sPolyEdges, SmPolyEdge, 32) ;
//                      pPolyLoop->GetPolyEdges(sPolyEdges) ;
//
//                      // find split PolyEdge
//                      for(kk=0;kk<sPolyEdges.GetSize();kk++)
//                        {
//                          SmPolyEdge   *pThisPolyEdge        = sPolyEdges[kk] ;
//                          SmPolyVertex *pThisStartPolyVertex = pThisPolyEdge->GetStartPolyVertex() ;
//
//                          // when this is the split point - check for sEndUV-sOldStartUV gap and spawn a NewClosedPolyLoop by splitting original PolyLoop at Complex Vertex
//                          if(pOldStartPolyVertex == pThisStartPolyVertex)
//                            {
//                              SmPoint3d      sOldEndUV        = pThisPolyEdge->GetOtherPolyVertex(pOldStartPolyVertex)->GetPoint() ;
//                              SmPolyEdge   * pOldPrevPolyEdge = pThisPolyEdge->GetCWPolyEdge() ;
//                              SmPolyVertex * pOldPrevStart    = pOldPrevPolyEdge->GetStartPolyVertex() ;
//                              SmPolyVertex * pOldPrevEnd      = pOldPrevPolyEdge->GetOtherPolyVertex(pOldPrevStart) ;
//                              SmPolyLoop   * pNewClosedLoop   = NULL ;
//
//                              // if needed fill sEndUV to sOldStartUV gap - add filler PolyEdge(sEndUV, sOldStartUV)
//                              //   this gap could happen for ComplexVertices at Poles
//                              //   and for edgeuses that have ends that don't come together at a single point
//                              sms_FillGapUVWithPolyEdge
//                                ( this,               // in : SmTess object for this face tessellation
//                                  pEdgeuse,           // in : Edgeuse spaned by [sStartUV sEndUV]
//                                  sEndUV,             // i/o: Last UVPoint in PolyLoop (to connect with sStartUV)
//                                                      //      set = sStartUV if filler PolyEdge is added
//                                  sOldStartUV,        // in : Tgt  UVPoint to Add next to PolyLoop (to connect with PrevEndUV)
//                                  sOldEndUV,          // in : Next UVPoint to Add to PolyLoop after sStartUV
//                                  sUVTol,             // in : min allowed U and V dist for a gap that needs filling
//                                  pPolyLoop,          // in : PolyLoop to receive new PolyVerts and PolyEdges
//                                  dTolerance,         // in : tolerance to assign new PolyVerts
//                                  pEndVertex,         // i/o: Vertex that maps to sPrevEndUV
//                                                      //      set = pStartVertex if filler PolyEdge is added
//                                  pEndVertex,         // in : Vertex that maps to sStartUV
//                                  sNewPolyEdges,      // out: accumulating List of Gap-Filling created PolyEdges to be checked for further subdivison later
//                                  sUsedVertices) ;    // i/o: accumulating list of (Vertex,PolyVertex) maps for all created PolyVertices
//
//                              // Split PolyLoop, Set pPolyLoop from FirstPolyEdge to pOldPrevPolyEdge (= PolyEdge before pThisPolyEdge.)
//                              //                 Set pPolyLoop->m_pLastEndPolyVertex = pThisPolyEdge->GetStartPolyVertex()
//                              //                 Set pNewClosedLoop from pThisPolyEdge to LastPolyEdge
//                              //                 Set pNewClosedLoop->m_bClosed = TRUE because those PolyLoops were built closed.
//                              pPolyLoop->SplitPolyLoop(pThisPolyEdge,    // in : First PolyEdge to place in second child
//                                                       pNewClosedLoop) ; // in : New PolyLoop, NULL on input
//                              SM_ASSERT(pNewClosedLoop->IsClosed()) ;
//                              bSplitPolyLoop = TRUE ; // remember to run ComputeLoopOrientation on pPolyLoop when it's completed
//
//#ifdef SM_DEBUG_CODE
//                             if(bDebugMe)
//                               { SM_ASSERT_VALID(pPolyLoop) ;
//                                 SM_ASSERT_VALID(pNewClosedLoop) ;
//                               }
//#endif // SM_DEBUG_CODE
//                              // skip problem loops with fewer than 3 edges
//                              if (pNewClosedLoop->GetSize() < 3)
//                                {
//                                  // remove and delete NewClosedLoop. Delete NewClosedLoop PolyEdges and no longer used PolyVertices
//                                  // First: if sNewPolyEdges contains any PolyEdges in pNewClosedLoop,
//                                  // remove them from the array -- they will be stale.  [B444]
//                                  sm_RemovePolyEdgesFromArray( pNewClosedLoop, sNewPolyEdges );
//
//                                  pPolyFaceUV->RemovePolyLoop(pNewClosedLoop,          // in : PolyLoop to delete
//                                                                pPolyLoop->GetSize() > 0
//                                                              ? pThisStartPolyVertex
//                                                              : NULL) ;  // in : Save this PolyVertex - it's the pPolyLoop::m_pLastEndPolyVertex
//                                  pNewClosedLoop = NULL ;
//                                }
//
//                              // when NewClosedLoop was made by split - compute loop orientation - child loop orients can vary arbitratily from parent loop orientation
//                              if(pNewClosedLoop)
//                                {
//                                  // loop state locals
//                                  SmOrientType eLoopOrient ;
//                                  double       dLoopArea ;
//
//                                  // compute loop orientation
//                                  pNewClosedLoop->ComputeLoopOrientation(sZ, eLoopOrient, dLoopArea) ;
//                                  pNewClosedLoop->SetOrientation(eLoopOrient) ;
//
//                                } // end NewClosedLoop existence check
//
//                              // for ComplexVertices at Poles the last PolyEdge in PolyLoop
//                              // can be a filler PolyEdge that is no longer needed filling a gap between
//                              // what is being mapped as two different PolyLoops.
//                              // When pOldPrevPolyEdge (= PolyEdge before pThisPolyEdge.) is on a surface singularity - remove it
//
//                              // when PolyLoop still has PolyEdges
//                              if(pPolyLoop->GetSize() > 0)
//                                {
//                                  // pOldPrevPolyEdge span
//                                  SmVector3d sOldPrevVec = pOldPrevEnd->GetPoint() - pOldPrevStart->GetPoint() ;
//
//                                  // pOldPrevPolyEdge Start Singularity
//                                  SmPoint2d       sUVPnt(pOldPrevStart->GetPoint().x, pOldPrevStart->GetPoint().y) ;
//                                  SmSurfParamType eOldPrevStartSDir ;
//                                  pSurface->IsSingularity(sUVPnt, eOldPrevStartSDir, dTolerance) ;
//
//                                  // Surface natural boundaries
//                                  SmExtent2d sUVDomain = pSurface->GetNaturalUVDomain() ;
//
//                                  // classify the UVPoints
//                                  SmPoint2d sOldPrevStartUV(pOldPrevStart->GetPoint().x, pOldPrevStart->GetPoint().y) ;
//                                  SmPoint2d sOldPrevEndUV  (pOldPrevEnd->GetPoint().x,   pOldPrevEnd->GetPoint().y) ;
//                                  SmExtentPointType eTypeU1, eTypeV1, eTmp ;
//                                  SmExtentPointType eTypeU2, eTypeV2 ;
//                                  sUVDomain.ClassifyPoint2d(sOldPrevStartUV, eTypeU1, eTmp, sUVTol.x) ;   // eU1 using Udir Tol
//                                  sUVDomain.ClassifyPoint2d(sOldPrevStartUV, eTmp, eTypeV1, sUVTol.y) ;   // eV1 using Vdir Tol
//                                  sUVDomain.ClassifyPoint2d(sOldPrevEndUV, eTypeU2, eTmp, sUVTol.x) ;  // eU2 using Udir Tol
//                                  sUVDomain.ClassifyPoint2d(sOldPrevEndUV, eTmp, eTypeV2, sUVTol.y) ;  // eV2 using Vdir Tol
//
//                                  // check for Points sharing a singularity edge
//                                  if(   (eTypeU1 == eTypeU2 && eOldPrevStartSDir == SM_SP_V)
//                                     || (eTypeV1 == eTypeV2 && eOldPrevStartSDir == SM_SP_U))
//                                    {
//                                      // remove LastPolyEdge from pPolyLoop
//                                      SM_ASSERT(pPolyLoop->GetLastPolyEdge() == pOldPrevPolyEdge) ;
//                                      pPolyLoop->RemoveLastPolyEdge() ;
//
//                                      // remove pOldPrevPolyEdge from sNewPolyEdges list when present
//                                      ULONG lIndex ;
//                                      if(pOldPrevPolyEdge && sNewPolyEdges.FindElement(pOldPrevPolyEdge, lIndex))
//                                        {
//                                          sNewPolyEdges.RemoveAt(lIndex, 1) ;
//                                        }
//                                    }
//                                } // end PolyLoop has PolyEdges check
//
//                              // next: set iter variables for next iteration
//
//                              // branch on existence of pSplitEndPolyVertex for no and some PolyEdges remain in PolyLoop cases
//                              SmPolyVertex * pSplitEndPolyVertex = pPolyLoop->GetLastEndPolyVertex() ;
//
//                              // pSplitEndPolyVertex will be NULL when the complex Vertex was the 1st vertex and all PolyEdges are in NewClosedLoop
//                              if(pSplitEndPolyVertex == NULL)
//                                {
//                                  // init 'Prev' locals to last Edgeuse for first iteration
//                                  pPrevEdgeuse     = sEdgeuses.GetLast() ;
//                                  pPrevStartVertex = pPrevEdgeuse->GetVertexuse()->GetVertex() ;
//                                  pPrevEndVertex   = pPrevEdgeuse->GetEdge()->GetOtherVertex(pPrevStartVertex) ;
//                                  SER(pPrevEdgeuse->NormalizedEvaluate( 1.0, TRUE, sPrevEndUV ));  // TRUE = UV Eval, FALSE = 3d Eval
//
//                                  // init iter locals as if iter had been done on LastEdgeuse
//                                  pEndVertex     = pPrevEndVertex ;
//                                  sEndUV         = sPrevEndUV ;
//                                } // end no PolyEdges remain in pPolyLoop branch
//                              else // some PolyEdges remain in pPolyLoop branch
//                                {
//                                  // init iter locals as if iter had been done on LastPolyEdge remaining in PolyLoop
//                                  pEndVertex = pSplitEndPolyVertex->GetVertex() ;
//                                  sEndUV     = pSplitEndPolyVertex->GetPoint() ;
//                                }
//
//                              // quit searching after finding the SplitEdge
//                              break ;
//                            } // end if ThisPolyEdge is the SplitPolyEdge check
//                        } // end iter PolyEdges seeking SplitPolyEdge
//                    } // end EndVertex is a ComplexVertex check
//                } // end internal edge check
            } // end iter every loop->Edgeuse adding PolyEdges to PolyLoop

#ifdef SM_DEBUG_CODE
          if(bDebugMe)
            {
              SM_ASSERT_VALID(pPolyLoop) ;
            }
#endif // SM_DEBUG_CODE

          // Glue Vertices and Edges for Edges that are radials
          // Have to wait until all PolyGeom exists, otherwise the PolyVertices aren't in place, so the Edges are wrong
          for ( jj = 0; jj < sRepeatedEdges.GetSize() ; ++jj )
          {
              SmEdge                * pEdge = sRepeatedEdges[jj];
              SmTArray<SmPolyEdge*>   sPolyEdges, sDegenPolyEdges;
              SmPolyEdge            * pThisPolyEdge         = NULL ;
              SmPolyEdge            * pOtherPolyEdge        = NULL ;
              sUsedEdges.GetValuesFor( pEdge, sPolyEdges );

              // Each edge should have had at most 2 Edgeuses in pFace. Every Edge with 2 EUs should have created 2 PolyEdges
              if ( sPolyEdges.GetSize() != 2 )
              {
                  SM_ASSERT( sPolyEdges.GetSize() == 2 );
                  continue;
              }

              // PolyEdges run head-to-tail. Glue PolyEdges.
              pThisPolyEdge  = sPolyEdges[0];
              pOtherPolyEdge = sPolyEdges[1];
              if (    pThisPolyEdge->GetStartPoint() == pOtherPolyEdge->GetEndPoint()
                   && pThisPolyEdge->GetEndPoint()   == pOtherPolyEdge->GetStartPoint() )
              { pPolyBrepUV->GlueEdges( pThisPolyEdge, pOtherPolyEdge, SM_OT_OPPOSITE, sDegenPolyEdges ); }
          } // End for each repeated Edge, glue new PolyEdges

#ifdef SM_DEBUG_CODE
          if(bDebugMe)
            {
              SM_ASSERT_VALID(pPolyLoop) ;
            }
#endif // SM_DEBUG_CODE

          // finish off the current PolyLoop
          SER( pPolyLoop->FinishPolyEdgeLoop(pPolyBrepUV) );

          // skip problem loops with fewer than 3 edges
          if (pPolyLoop->GetSize() < 3)
            {
              // remove and delete NewClosedLoop. Delete NewClosedLoop PolyEdges and no longer used PolyVertices
              // First: if sNewPolyEdges contains any PolyEdges in pNewClosedLoop,
              // remove them from the array -- they will be stale.  [B444]
              sm_RemovePolyEdgesFromArray( pPolyLoop, sNewPolyEdges );

              pPolyFaceUV->RemovePolyLoop(pPolyLoop,    // in : PolyLoop to delete
                                          NULL) ;       // in : No PolyVertex to save
              pPolyLoop = NULL ;
            }

          // when pPolyLoop was made by split - compute loop orientation - child loop orients can vary arbitrarily from parent loop orientation
          if(pPolyLoop && bSplitPolyLoop)
            {
               // loop state locals
               SmOrientType eLoopOrient ;
               double       dLoopArea ;

               pPolyLoop->ComputeLoopOrientation(sZ, eLoopOrient, dLoopArea) ;
               pPolyLoop->SetOrientation(eLoopOrient) ;

            } // end Split PolyLoop check

#ifdef SM_DEBUG_CODE
          if (bDebugMe)
            { SM_ASSERT_VALID(pPolyLoop) ;  // check that every PolyVertex has a SmVertex back pointer
            }
#endif // SM_DEBUG_CODE

        } // end EdgeLoop branch
      else // VertexLoop branch
        {
          // locals
          SmPolyEdge    * pNewPolyEdge = NULL;
          SmPolyLoop    * pNewPolyLoop = NULL;
          SmPolyVertex  * pNewPolyVertex = NULL;
          SmSolution      aSData[16];
          SmSolutionArray sSolutions(16,aSData);

          // vertex 3DPoint
          SmVertexuse   * pVertexuse    = pLoopuse->GetVertexuse();         NER(pVertexuse);
          SmVertex      * pVertex       = pVertexuse->GetVertex();          NER(pVertex);
          SmPoint3d       sPnt          = pVertexuse->GetVertex()->GetPoint();

          // Drop Vertex 3DPoint to Surface to find closes Surface UVPoint
          SER(pSurface->GlobalPointSolve(pFace->GetUVDomain(),
                                         SM_SO_MINIMIZE,
                                         sPnt,
                                         pVertexuse->GetVertex()->GetTolerance(),
                                         NULL,
                                         SM_SR_SINGLE,
                                         sSolutions));

          // skip problem vertices that don't project to surface
          if (sSolutions.GetSize() < 1)
            { continue; }

          // Surface UVPoint
          SmPoint3d sUVPnt(sSolutions[0].m_vStart[0],
                           sSolutions[0].m_vStart[1],
                           0.0);

          // insert vertexLoop into SmPolyFace - assume VertexLoops are not coincident with any other vertices
          SER(pPolyFaceUV->AddSinglePolyVertexLoop(sUVPnt,
                                                   dTolerance,
                                                   pNewPolyVertex,
                                                   pNewPolyEdge,
                                                   pNewPolyLoop));

          // remember the SmVertex->PolyVertex mapping
          pNewPolyVertex->SetOriginalVertex(pVertex, TRUE) ; // TRUE = propagate attributes if possible

          // graphisoft - remember the SmVertex->PolyVertex mapping
          SetPolyEdgeVertexuse (pNewPolyEdge, pVertexuse);

        } // end VertexLoop branch

    } // end iter every loopuse - building 1 (or more) PolyLoop in PolyFaceUV

  // next: ensure the PolyLoop order - 1st = outer loop, rest = inner loops


  // arrive here when a PolyLoop has been added to SmPolyFace under construction

  // make sure 1st PolyLoop is orient==SM_OT_SAME and the rest are SM_OT_OPPOSITE
  pPolyFaceUV->OrderPolyLoops() ;

  // When m_bAdvancingFront == FALSE
  //        sNewPolyEdges contains only edges created to fill gaps between loop edgeuses.
  //      m_bAdvancingFront == TRUE
  //        sNewPolyEdges contains all edges added to SmPolyFace

  // for every sNewPolyEdges Edge - see if it needs to be split at Cache node boundaries
  ULONG lNumNew = sNewPolyEdges.GetSize();
  SmTArray<SmPolyEdge*> sNewPolyEdgesFromSubdivision;
  for(jj=0; jj<lNumNew; jj++)
    {
      SmPolyEdge *pNewEdge = sNewPolyEdges[jj];
      SmPoint3d   sStPt    = pNewEdge->GetStartPoint();
      SmPoint3d   sEndPt   = pNewEdge->GetEndPoint();

      // project St and End pts into 3Space
      SmPoint2d sStUV(sStPt.x,  sStPt.y) ;
      SmPoint2d sEndUV(sEndPt.x, sEndPt.y) ;
      SmPoint3d sSt3d, sEnd3d ;
      pSurface->EvaluatePoint(sStUV,  sSt3d) ;
      pSurface->EvaluatePoint(sEndUV, sEnd3d) ;

      // skip splitting NewEdges degenerate in 3d (typically built along Surface singularity boundaries)
      if(sEnd3d.CloserThan(dTolerance, sSt3d))
        { continue ; }

      //  // alternative skip idea for NewEdge on Pole - (but doesn't catch degenerate edges made in other ways)
      //  SmSurfParamType eSingDir ;
      //  SmPoint2d   sMidPt((sStPt.x+sEndPt.x)/2.0, (sStPt.y+sEndPt.y)/2.0) ;
      //  SmBoolean bOnPole = pSurface->IsSingularity(sMidPt, eSingDir, dTolerance) ;
      //  if(bOnPole)
      //    { continue ; }

      // when the polyEdge is vertical or horizontal
      if(   SM_ARE_SAME(sEndPt.x,sStPt.x)
         || SM_ARE_SAME(sEndPt.y,sStPt.y))
        {
          // get PolyEdge's Line geometry
          SmExtent1d sIvl(0.0,1.0);
          SmVector3d sLinePnt, sLineVector;
          pNewEdge->GetLine(sLinePnt,sLineVector);

          // mark all Line cache subdivision tree boundary intersections
          SM_OBJ_ARRAY(sTSplits, double, 256) ; // SmTArray<double>
          if (m_bAdvancingFront)
            {
              // Only do this for Pole edges which are filling in if we
              // are using advancing front.
              if (GetPolyEdgeEU(pNewEdge) == NULL)
                {
                   // Find an array of T splits for an Iso-parametric line
                   // corresponding to crossings of the cache.
                   SER(FindIsoLineSplits(pNewEdge,pSC,sLinePnt,sLineVector,sIvl,sTSplits));
                }
            }
          else
            {
              SER(FindIsoLineSplits(pNewEdge,pSC,sLinePnt,sLineVector,sIvl,sTSplits));
            }

          // Split the polyedge at each Line/CacheSubdivisionTreeNodeBoundary intersection
          SmPolyEdge *pSplitEdge = pNewEdge;
          for (ULONG kkk=1; kkk+1<sTSplits.GetSize(); kkk++)
            {
              SmPolyEdge   *pTmpNewEdge = NULL;
              SmPolyVertex *pNewVertex = NULL;
              SmPoint3d sSplitPnt = sLinePnt + sTSplits[kkk] * sLineVector;
              SER(pSplitEdge->GetPolyFace()->MakeVertexSplitPolyEdge(pSplitEdge,   // in : PolyEdge to split
                                                                     sSplitPnt,    // in : Point split location
                                                                     pTmpNewEdge,  // out: new PolyEdge (and new radial partners)
                                                                     pNewVertex)); // out: new PolyVertex
              if (!pTmpNewEdge)
                  continue;
              if (pTmpNewEdge->GetStartPoint().DistanceBetween(sSplitPnt) <
                  pTmpNewEdge->GetTolerance())
                {
                  pSplitEdge = pTmpNewEdge;
                }
            } // end iter all PolyEdge Split Points
        } // end horizontal or vertical edge branch
      else // PolyEdge is not horizontal or vertical
        {
          // See if edge needs to be subdivided
          ULONG lNumSubdivisions;
          SER(CheckAgainstSurfaceCache(pSC,sStPt,sEndPt,0.9,FALSE,lNumSubdivisions, TRUE));
          // split edge into subdivisions
          if (lNumSubdivisions > 0 && ! m_bAdvancingFront)
            {
              // split into equal sized segments - works on manifold and lamina PolyEdges
                SER(SubdivideManifoldEdge(
                    pNewEdge, lNumSubdivisions, NULL, FALSE, sNewPolyEdgesFromSubdivision));
            }
        }  // end not a horizontal or a vertical edge branch
    } // end iter every new PolyEdge

  // set output
  sCleanBrep.Clear();
  rpPolyBrep = pPolyBrepUV;
  rpPolyFace = pPolyFaceUV;

#ifdef SM_DEBUG_CODE
  if (bDebugMe)
    {
      SM_DUMP_AND_ASSERT_VALID(pFace) ;
      SM_DUMP_AND_ASSERT_VALID(rpPolyFace) ;
      SM_DUMP_AND_ASSERT_VALID(rpPolyBrep) ;

      smgfx_Erase();
      smgfx_SetLook(1,2, 0,0,0); if(pPolyBrepUV) pPolyBrepUV->Draw(TRUE); sm_GraphicsLoop();
      smgfx_SetLook(1,2, 0,0,0); pFace->DrawUVCurves(FALSE); sm_GraphicsLoop();
      smgfx_SetLook(1,2, 0,0,0); pSC->DrawSubdivision2D(FALSE); sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
#endif

  // all done
  return SM_SUCCESS;

} // end SmTess::MakePolygon

/*******************************************************************//**
PURPOSE:

NOTES: 
***********************************************************************/
static SmStatus sm_InsertSortedBreaks
  (SmTArray<double> & rDoubleArray,
   double dNewDouble)
{
  for (ULONG i=0; i<rDoubleArray.GetSize(); i++)
    {
      if (SM_ARE_SAME(dNewDouble,rDoubleArray[i])) return SM_SUCCESS;
      if (dNewDouble < rDoubleArray[i])
        {
          rDoubleArray.InsertAt(i,dNewDouble,1);
          return SM_SUCCESS;
        }
    }
  rDoubleArray.Add(dNewDouble);
  return SM_SUCCESS;

} // end sm_InsertSortedBreaks

/*******************************************************************//**
PURPOSE: Subdivide isoparameter-curve Edges to the surface tessellation cache.

NOTES: This splits pEdgeToSubdivide when its UVTrimCurve is an isoParamCurve
       otherwise has no effect.  New Edges and Vertices are inserted into
       Brep, m_pTessBrep.

SIDE EFFECTS ---
   The given edge is split up -- vertices inserted -- in m_pTessBrep.
***********************************************************************/
SmStatus SmTess::IsoEdgeSubdivide
  (SmEdge *pEdgeToSubdivide)
{
  if ( pEdgeToSubdivide->IsWire() )
    { return SM_SUCCESS; }

  // locals
  ULONG ii ;
  SM_PTR_ARRAY(sEdgeuses, SmEdgeuse, 64) ; // SmTArray<SmEdgeuse *>
  SM_OBJ_ARRAY(sTSplits,  double,    64) ; // SmTArray<double>

  pEdgeToSubdivide->GetEdgeuses( sEdgeuses );
  ULONG lNumEUs = sEdgeuses.GetSize();

  // for every other edgeuse (skip mates) - break after parsing the 1st EU successfully
  for ( ii=0; ii<lNumEUs; ii+=2 )
    {
      SmEdgeuse      * pEU      = sEdgeuses[ii];
      SmBSplineCurve * pUVCurve = NULL ;
      SmVector3d       sPV[2], sLineVec;
      SmPoint3d        sLinePnt;

      // Get UVTrimCurve locals
      SER( pEU->GetOrCreateUVTrimCurve( pUVCurve ));
      if ( pUVCurve == NULL )
        { continue; }
      SmExtent1d sIvl = pUVCurve->GetNaturalInterval();
      SER( pUVCurve->Evaluate( sIvl.Evaluate( 0.3456 ), 1, TRUE, sPV ));

      // skip non Iso oriented UVTrimCurves (check one tangent value)
      if (! (SM_IS_ZERO(sPV[1].x) || SM_IS_ZERO(sPV[1].y)))
        { continue; }

      // skip nonLinear UVTrimCurves
      double dTol = SM_EFF_ZERO * (1.0 + sPV[0].GetMaxDimension()) ;
      if (!pUVCurve->IsLine(13,dTol,sLinePnt,sLineVec))
        { continue ; }

      // Split line at X or Y parameters of cache elements
      SmFace         * pFace = pEU->GetFace();
      SmTessSrfCache * pSC   = m_vCache.GetSecond(pFace); NER(pSC);

      SER( FindIsoLineSplits( NULL, pSC, sLinePnt, sLineVec, sIvl, sTSplits ));
      break; // For now only allow one splitting

    } // end iter ii, every Edge->EU looking for IsoParameter UVTrimCurve

  // Now split the edge at the given sTSplits parameters
  if ( sTSplits.GetSize() > 1 )
    {
      SER( ClampAndSplitEdge( pEdgeToSubdivide, sTSplits ));
    }

    // all done
  return SM_SUCCESS;

} // end SmTess::IsoEdgeSubdivide

/*******************************************************************//**
PURPOSE: Find an array of T splits for an Iso-parametric line
    corresponding to crossings of the cache subdivision boundaries.

NOTES: rTSplits will not be reset by this method because
    it is an incremental.
***********************************************************************/
SmStatus SmTess::FindIsoLineSplits
 (SmPolyEdge      * pEdge,           // NotUsed: in :
  SmTessSrfCache  * pSurfaceCache,   // in :
  const SmPoint3d & crLinePoint,     // in : The line point and vector define the entire
                                     //      UV curve to be split.  The interval is just
                                     //      the mapping between the 0-1 parameterization
                                     //      of the line and the actual parameterization
                                     //      of the UV curve.
  const SmVector3d & crLineVector,   // in :
  const SmExtent1d & crLineInterval, // in :
  SmTArray<double> & rTSplits)       // out:
 const
{
  SM_REF1(pEdge) ; 
  SmPoint2d sLineStart(crLinePoint.x,crLineVector.y);
  SmPoint2d sLineEnd(crLinePoint.x+crLineVector.x,crLinePoint.y+crLineVector.y);
  SER(sm_InsertSortedBreaks(rTSplits,crLineInterval.GetMin()));
  SER(sm_InsertSortedBreaks(rTSplits,crLineInterval.GetMax()));
  if (this->m_bAdvancingFront)
    {
      double dData[64];
      SmTArray<double> sSplits(64,dData);
      SmFace *pFace = pSurfaceCache->GetFace();

      // when PolyLine is more vertical
      if (smos_Fabs(crLineVector.x) < smos_Fabs(crLineVector.y))
        {
          SER(pFace->GetSurface()->FindTessellationSplits(SM_SP_U,
                                                          pFace->GetUVDomain(),
                                                          0.0,
                                                          0.0,
                                                          pSurfaceCache->m_dMaximumSideLength3D,
                                                          sSplits));
          for (ULONG i=0; i<sSplits.GetSize(); i++)
            {
              SmPoint3d sSplitPnt(crLinePoint.x,sSplits[i],0.0);
              double dParam;
              smgu_LineClosestPoint(crLinePoint,crLineVector,sSplitPnt,dParam);
              if (dParam > 0.01 && dParam < 0.99)
                {
                  SER(sm_InsertSortedBreaks(rTSplits,crLineInterval.Evaluate(dParam)));
                }
            }
        } // end PolyLine is more vertical branch
      else // end PolyLine is more horizontal branch
        {
          SER(pFace->GetSurface()->FindTessellationSplits(SM_SP_V,
                                                          pFace->GetUVDomain(),
                                                          0.0,
                                                          0.0,
                                                          pSurfaceCache->m_dMaximumSideLength3D,
                                                          sSplits));
          for (ULONG i=0; i<sSplits.GetSize(); i++)
            {
              SmPoint3d sSplitPnt(sSplits[i],crLinePoint.y,0.0);
              double dParam;
              smgu_LineClosestPoint(crLinePoint,crLineVector,sSplitPnt,dParam);
              if (dParam > 0.01 && dParam < 0.99)
                {
                  SER(sm_InsertSortedBreaks(rTSplits,crLineInterval.Evaluate(dParam)));
                }
            }
        } // end PolyLine is more horizontal branch

    } // end m_bAdvancingFront == TRUE branch
  else // m_bAdvancingFront == FALSE branch
    {
      SM_PTR_ARRAY(sNodes, SmTreeNode,256 ) ; // SmTArray<SmTreeNode*>
      SmExtent2d sUVBox(crLinePoint);
      sUVBox.AddPoint2d(sLineEnd);
      sUVBox.ExpandAbsolute(SM_EFF_ZERO_SQRT);
      pSurfaceCache->FindUVNodes(sUVBox,sNodes);
      for (ULONG j=0; j<sNodes.GetSize(); j++)
        {
          SmTreeNode    *pNode = sNodes[j];
          SmBezierAux2d *pAux  = (SmBezierAux2d*)pNode->m_pData;
          double dTSplit;

          if ( SM_IS_ZERO( crLineVector.x ))
            {
              double dY = pAux->m_sUVDomain.GetMin().y;
              if ( fabs( crLineVector.y ) < 1.0e-8 )
                { return SM_ERR;   }
              dTSplit = (dY - crLinePoint.y) / crLineVector.y;
            }
          else
            {
              double dX = pAux->m_sUVDomain.GetMin().x;
              dTSplit = (dX - crLinePoint.x) / crLineVector.x;
            }
          // Now Convert T to parameter space of curve
          // and insert into a sorted list of doubles.
          // Don't insert breaks that are not within the curve domain.
          // MakeVertexSplitPolyEdge() will create new edges that
          // extend beyond the original. [B157]
          if ( dTSplit > 0.01 && dTSplit < 0.99 )
            {
              double dCrvT = crLineInterval.Evaluate( dTSplit );
              SER( sm_InsertSortedBreaks( rTSplits, dCrvT ));
            }
        }
    }

  // all done
  return SM_SUCCESS;

} // end SmTess::FindIsoLineSplits

/*******************************************************************//**
PURPOSE: Clamp and split pEdge at given rParameters.

NOTES: 
   'Clamp' means to skip SplitParams too close to pEdge->EndVertices.
   This routine always skips the first and last params in the list.

   Each edge split
     1. Adds 1 new SmVertex and 1 new Edge per Split to m_pTessBrep
     2. removes all Marks, from pOldEdge and pNewEdge.

SIDE EFFECTS ---
   pEdge       gets split (repeatedly) and reused as one of the final split children.
   m_pTessBrep gets new SmVertices and SmEdges.
***********************************************************************/
SmStatus SmTess::ClampAndSplitEdge
  (SmEdge           * pEdge,         // in: edge to be split
   SmTArray<double> & rParameters)   // in: where to split edge
{
#ifdef SM_DEBUG_CODE
  SmBoolean bDebugMe = FALSE;
  if (bDebugMe)
    {
      pEdge->GetCurve()->Dump();
      rParameters.Dump();
    }
#endif

  // check input
  SM_ASSERT(rParameters.GetSize() > 0 );

  // Next clamp end points of tessellation to 3D Vertices

  // clamp start vertex: remove SplitParams too close to start vertex.
  SmCurve  * pCurve       = pEdge->GetCurve(); NER(pCurve);
  SmVertex * pV           = pEdge->GetStartVertex();
  SmPoint3d  sVPnt        = pV->GetPoint();
  double     dVTol        = pV->GetTolerance();
  SmXSectTol3d sObjXSectTol3d = SmTol::GetXSectTol3d(dVTol, dVTol);
  ULONG      lNumToRemove = 1; // Always remove the first one.
  ULONG ii, lNumParams    = rParameters.GetSize();

  // for Every internal SplitParam - Count Params too close to sVPnt to use
  for (ii=1; ii+1< lNumParams; ii++)  // note: can't say lNumParams-1
    {
      SmPoint3d sEPnt;
      SER(pCurve->EvaluatePoint(rParameters[ii],sEPnt));
      if (sEPnt.DistanceBetween(sVPnt) < sObjXSectTol3d)
        {
          lNumToRemove ++;
        }
      else
        {
          break;
        }
    } // end iter every internal SplitParam

  // when okay - remove params too close to SVPnt
  if (rParameters.GetSize() > lNumToRemove)
    {
      rParameters.RemoveAt(0,lNumToRemove);
    }

  // Clamp end vertex - remove parameters too close to end vertex.
  lNumToRemove    = 1;
  SmVertex *pEndV = pEdge->GetOtherVertex(pV);
  sVPnt           = pEndV->GetPoint();
  dVTol           = pEndV->GetTolerance();
  lNumParams      = rParameters.GetSize();

  sObjXSectTol3d = SmTol::GetXSectTol3d(dVTol, dVTol);

  // for every internal param - find SplitParams too close to end vertex.
  for (ii=1; ii+1<lNumParams; ii++)  // note: can't say lNumParams-1
    {
      ULONG lSize = rParameters.GetSize();
      SmPoint3d sEPnt;
      SER(pCurve->EvaluatePoint(rParameters[lSize-1-ii],sEPnt));
      if (sEPnt.DistanceBetween(sVPnt) < sObjXSectTol3d)
        {
          lNumToRemove ++;
        }
      else
        {
          break;
        }
    } // end iter SplitParams looking for params too close to end vertex

    // when okay - remove SplitParams too close to end vertex
  if (rParameters.GetSize() >= lNumToRemove)
    {
      rParameters.RemoveAt(rParameters.GetSize()-lNumToRemove,lNumToRemove);
    }

  lNumParams = rParameters.GetSize();

  // For every remaining SplitParam
  for (ii=0; ii<lNumParams; ii++)
    {
      SmEdge   * pNewEdge1 = NULL, * pNewEdge2 = NULL;
      SmVertex * pNewVertex = NULL;

      // Split pEdge in m_pTessBrep at SplitParam[ii]
      if(SM_SUCCESS == m_pTessBrep->MakeVertexSplitEdge(pEdge,
                                                        rParameters[ii],
                                                        pNewEdge1,
                                                        pNewEdge2,
                                                        pNewVertex) )
        {
          pEdge =   (pNewEdge2->GetInterval().GetMax() > pNewEdge1->GetInterval().GetMax())
                  ? pNewEdge2
                  : pNewEdge1 ;
        }
    } // end iter every SplitParam - splitting pEdge in m_pTessBrep

  // all done
  return SM_SUCCESS;

} // end SmTess::ClampAndSplitEdge

/*******************************************************************//**
PURPOSE: Split pEdge in m_pTessBrep at found tessellation SplitParams

NOTES: 1. SplitParams for pEdge->Curve are found with m_rCurveTess.TessellateCurve()
          using m_rCurveTess control parameters:
            SmBoolean m_bEvalBasedTessellation; // FALSE = SmCurve::TessellateByBisection()
                                                //          if that fails - tessellate by SmCurveCache spatial decomposition
                                                // TRUE  = SmBSplineCurve::EquallySpacedPoints()
                                                // default:[FALSE]

            ULONG  m_lMinSegNumber;             // min number of segments in tess polygon,     0=ignore
            double m_dChordHeight;              // max distance between geom and tess segment, 0=ignore
            double m_dAngTolDeg;                // max angle between tess tangents,            0=ignore
            double m_dMaxDist3dBetweenPts;      // max distance between tess pts,              0=ignore
            double m_dMinParamRatio;            // Limits the smallness of the stepsize

       2. m_pTessBrep gets 1 new SmVertex and 1 new SmEdge per split when
          pEdge is split (repeatedly) by ClampAndSplitEdge(SplitParams).

***********************************************************************/
SmStatus SmTess::TessellateEdge
  (SmEdge *pEdgeToTessellate)  // in : edge to tessellate
{
  // check input
  NER(pEdgeToTessellate);

#ifdef SM_DEBUG_CODE
  SmBoolean bDebugMe = FALSE;
  std::atomic<ULONG> lCount(1);
  lCount++;
  std::atomic<ULONG> lDebugCount (0);
  if (bDebugMe || lCount == lDebugCount)
    {
      SM_ASSERT_VALID(pEdgeToTessellate) ;
    }
#endif // SM_DEBUG_CODE

  // locals
  SM_OBJ_ARRAY(sParameters, double, 256) ; // SmTArray<double>
  SmEdge    * pEdge  = pEdgeToTessellate;
  SmExtent1d  sIvl   = pEdge->GetInterval();
  SmCurve   * pCurve = pEdge->GetCurve(); NER(pCurve);

  // set AdvancingFront state in m_rCurveTess
  if (this->m_bAdvancingFront)
    {
      m_rCurveTess.m_bEvalBasedTessellation = TRUE;
    }

  // get sParameters list for curve tesselation
  SER(m_rCurveTess.TessellateCurve(*pCurve,               // in : target curve
                                    pEdge->GetInterval(), // in : target interval
                                   &sParameters));        // out: tessellation parameters, includes curve endPoints

#ifdef SM_DEBUG_CODE
  if (bDebugMe || lCount == lDebugCount)
    {
      sParameters.Dump();
      smgfx_SetColor(1,0,0);
      for (ULONG mmm=0; mmm<sParameters.GetSize(); mmm++)
        {
          pCurve->DrawAt(sParameters[mmm],0);
        }
      sm_GraphicsLoop();
      smgfx_SetColor(0,0,0);
      pEdgeToTessellate->Draw();
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

  // Clamp and split pEdge at given sParameters adding
  // 1 new SmVertex and 1 New SmEdge per Split to m_pTessBrep
  SER(ClampAndSplitEdge(pEdgeToTessellate,sParameters));

  // all done
  return SM_SUCCESS;

} // end SmTess::TessellateEdge

/*******************************************************************//**
PURPOSE: Create SmTessSrfCache cache for pFace->Surface
         and relate the pair <pFace, pSurfaceTessCache> in m_vCache

NOTES: 
***********************************************************************/
SmStatus SmTess::CreateFaceTessCache(SmFace * pFace)
{

#ifdef SM_DEBUG_CODE
  SmBoolean bDebugMe2 = FALSE;
  if (bDebugMe2)
    {
      SmSurface *pSurface = pFace->GetSurface(); 
      pSurface->Dump() ;

      smgfx_Erase();
      smgfx_SetLook(1,2, 0,0,0) ; pFace->Draw( SM_DM_WIREFRAME ); sm_GraphicsLoop();
      smgfx_SetLook(1,2, 0,1,1) ; pSurface->DrawUV(4,4); sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
#endif

  // First subdivide segments to be no greater than 110% the size
  // of the grid elements that contain it.

  // allocate the TessSrfCache
  SmTessSrfCache* pSC = new (this->m_crContext) SmTessSrfCache
      (pFace,                                                     // in : ptr to Face with TrimSurface to tessellate
       NULL,                                                      // in : PolyBrep tessellation of the TrimmedSurface (the polygons)
      &this->m_rSurfaceTess,                                      // in : contains: m_vViewVector, m_dSilhouetteChordHeight, m_dSilhouetteAngleToleranceDeg
       this->m_rSurfaceTess.m_dChordHeight,                       // in : max allowed control-Point dist to patch basePlane,       0.0 = ignore
       this->m_rSurfaceTess.m_dAngTolDeg*SM_PI/180.0, // in : max allowed controlPolygon endTangent angles (radians),  0.0 = ignore
       this->m_rSurfaceTess.m_dMaxEdgeLength3D,                   // in : max allowed basePolygon side length in u or v direction, 0.0 = ignore
       this->m_rSurfaceTess.m_dMinEdgeLength3D,                   // in : polygon side length stopping size for subdivision,       0.0 = ignore
       this->m_rSurfaceTess.m_dMinEdgeLengthRatioUV,              // in : Subdivison stops once node get smaller than this size,   0.0 = ignore
       this->m_rSurfaceTess.m_dMaxAspectRatio);                   // in : max allowed element basepolygon aspect ratio,            0.0 = ignore
  pSC->SetTessellationAlgorithm(this->m_eTessAlgorithm);
  NER(pSC);
  SmObjDelete sCleanup(pSC);

  // build the surface subdivision - these subdivisions set the final tessellation
  SER(pSC->BuildTree());

  ULONG lMem;
  pSC->GetMemoryUsed(lMem);
  //smos_WriteLong(_T("Memory Used in Cache = "),lMem);

  sCleanup.Clear();
  m_vCache.RelatePair(pFace,pSC);

  return SM_SUCCESS;

} // end SmTess::CreateFaceTessCache

/*******************************************************************//**
PURPOSE:

NOTES:  
***********************************************************************/
SmStatus SmTess::CreateFaceTessCache(SmFace* pFace, SmTessSrfCache*& pSC)
{

#ifdef SM_DEBUG_CODE
    SmBoolean bDebugMe2 = FALSE;
    if (bDebugMe2)
    {
        SmSurface* pSurface = pFace->GetSurface();
        pSurface->Dump();

        smgfx_Erase();
        smgfx_SetLook(1, 2, 0, 0, 0);
        pFace->Draw(SM_DM_WIREFRAME);
        sm_GraphicsLoop();
        smgfx_SetLook(1, 2, 0, 1, 1);
        pSurface->DrawUV(4, 4);
        sm_GraphicsLoop();
        sm_GraphicsLoop();
    }
#endif

    // First subdivide segments to be no greater than 110% the size
    // of the grid elements that contain it.

    // allocate the TessSrfCache
    pSC = new (this->m_crContext) SmTessSrfCache(pFace, // in : ptr to Face with TrimSurface to tessellate
                                                 NULL, // in : PolyBrep tessellation of the TrimmedSurface (the
                                                       // polygons)
                                                 &this->m_rSurfaceTess, // in : contains: m_vViewVector,
                                                                        // m_dSilhouetteChordHeight,
                                                                        // m_dSilhouetteAngleToleranceDeg
                                                 this->m_rSurfaceTess.m_dChordHeight, // in : max allowed control-Point
                                                                                      // dist to patch basePlane, 0.0 =
                                                                                      // ignore
                                                 this->m_rSurfaceTess.m_dAngTolDeg * SM_PI / 180.0, // in : max allowed
                                                                                                    // controlPolygon
                                                                                                    // endTangent angles
                                                                                                    // (radians),  0.0 =
                                                                                                    // ignore
                                                 this->m_rSurfaceTess.m_dMaxEdgeLength3D, // in : max allowed
                                                                                          // basePolygon side length in
                                                                                          // u or v direction, 0.0 =
                                                                                          // ignore
                                                 this->m_rSurfaceTess.m_dMinEdgeLength3D, // in : polygon side length
                                                                                          // stopping size for
                                                                                          // subdivision,       0.0 =
                                                                                          // ignore
                                                 this->m_rSurfaceTess.m_dMinEdgeLengthRatioUV, // in : Subdivison stops
                                                                                               // once node get smaller
                                                                                               // than this size,   0.0
                                                                                               // = ignore
                                                 this->m_rSurfaceTess.m_dMaxAspectRatio); // in : max allowed element
                                                                                          // basepolygon aspect ratio,
                                                                                          // 0.0 = ignore
    pSC->SetTessellationAlgorithm(this->m_eTessAlgorithm);
    NER(pSC);
    SmObjDelete sCleanup(pSC);

    // build the surface subdivision - these subdivisions set the final tessellation
    SER(pSC->BuildTree());

    //    ULONG lMem = pSC->GetMemoryUsed();
    //    smos_WriteLong(_T("Memory Used in Cache = "),lMem);

    sCleanup.Clear();
    // m_vCache.RelatePair(pFace, pSC);

    return SM_SUCCESS;

} // end SmTess::CreateFaceTessCache


/*******************************************************************//**
PURPOSE: Split all pFace->Edges in m_pTessBrep until their UVSize is
  smaller than the UV size of the SmTessSrfCache subdivision tree leaf nodes
  that they intersect.

NOTES: 
 Edges are evenly split until their lengths are about the size of a subdivision node.
 Edge splits are not placed exactly on the Subdivison boundary.
 That makes good sense. The Edges end up tessellated to a reasonable size
 but the arbitrary subdivision tree boundaries are not propagated to the curve
 tessellation.

TODO:
   If needed, this method could do more split checking as follows:
       1) Check for intersection of line segments in UV - refine when necessary
       2) Check for inner loops being inside of polygon of outer loops

***********************************************************************/
SmStatus SmTess::SplitFaceEdgesToUVSize
  (SmFace * pFace)     // in : target pFace in m_pTessBrep being tessellated
{
  // locals
  SM_PTR_ARRAY(sEdgeuses, SmEdgeuse, 64) ; // SmTArray<SmEdgeuse*>
  SmTessSrfCache      * pSC = m_vCache.GetSecond(pFace); NER(pSC);

  // get pFace->Edgeuses
  SmFaceuse           * pUpperFU = pFace->GetUpwardFaceuse();
  pUpperFU->GetEdgeuses(sEdgeuses);

#ifdef SM_DEBUG_CODE
  SmBoolean bDebugMe = FALSE;
  if (bDebugMe)
    {
      smgfx_Erase();
      pSC->DrawSubdivision2D(TRUE);
      for (ULONG di=0; di<sEdgeuses.GetSize(); di++)
        {
          SmEdge *pE = sEdgeuses[di]->GetEdge();
          pE->Dump();
          pE->Draw();
          SmBSplineCurve *pUVCurve = NULL ;
          sEdgeuses[di]->GetOrCreateUVTrimCurve(pUVCurve);
          if ( pUVCurve )
            {
              smgfx_ChangeColor(di != 0);
              pUVCurve->DrawAt(pUVCurve->GetNaturalInterval().GetMin(),0);
              pUVCurve->Draw();
              pUVCurve->Dump();
            }
          sm_GraphicsLoop();
        }
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

 // Use the sEdgeuses array as a stack.
 // We want to use DropPoint to populate the UV for each Vertexuse
 // This doesn't always work, so we generate the UV values with NormalizedEvaluate
 // when either DropPoint fails or when it generates more than 1 possible solution

#define SM_SUBDIVISION_RATIO  1.1
  SmPoint2d sStart, sEnd;

  SmExtent2d sFaceDomain = pFace->GetUVDomain();
  SmSurface* pSurf = pFace->GetSurface();

  double sGap = 0.0;
  SmBoolean bIsMulti = FALSE;
  SmSolution sSol;
  SmSolution sData[4];
  SmSolutionArray sSolutions(4, sData);

  // If the first pass, we populate the UV values
  for (ULONG ii = 0; ii < sEdgeuses.GetSize(); ii++)
  {
      SmEdgeuse* pEU = sEdgeuses[ii];
      SmBSplineCurve* pUVTrim;
      pUVTrim = pEU->GetUVTrimCurve();
      SmPoint2d sUVPoint = { SM_BIG_DOUBLE, SM_BIG_DOUBLE };
      SmPoint3d sStartUVPoint3d;

      // If we already have a UV trim curve, just use that
      if (pUVTrim)
      {
          if (SM_SUCCESS != pEU->NormalizedEvaluate(0.0, TRUE, sStartUVPoint3d))
          {
              sEdgeuses.RemoveAt(ii);
              ii--;
              continue;
          }
  
          sUVPoint.x = sStartUVPoint3d.x;
          sUVPoint.y = sStartUVPoint3d.y;
      }
      // Otherwise, we try to to use DropPoint to get the UV values
      else
      {
          SmBoolean bSuccessStart = TRUE;
          SmPoint3d s3dStart = pEU->GetVertexuse()->GetVertex()->GetPoint();
          // Note that we are bailing on DropPoint if it has more than one possible soluition
          pSurf->DropPoint(
              s3dStart, sFaceDomain, NULL, bSuccessStart, sUVPoint, sGap, bIsMulti, SM_SO_MINIMIZE, NULL, &sSolutions);

          // If DropPoint fails or the solution is multivalued, we resort to (implicit) UV trim curve construction
          if (!bSuccessStart || bIsMulti)
          {
              if (SM_SUCCESS != pEU->NormalizedEvaluate(0.0, TRUE, sStartUVPoint3d))
              {
                  sEdgeuses.RemoveAt(ii);
                  ii--;
                  continue;
              }
              sUVPoint.x = sStartUVPoint3d.x;
              sUVPoint.y = sStartUVPoint3d.y;
          }
                
      }

      // In either case, we have the UV values for the vertexuse, so set it
      pEU->GetVertexuse()->SetUVPoint(sUVPoint);

#ifdef SM_DEBUG_CODE
      SmPoint3d sEdgePt, sSurfacePt;
      pSurf->EvaluatePoint(sUVPoint, sSurfacePt);
      pEU->GetEdge()->GetCurve()->EvaluatePoint(pEU->GetEdgeParam(0.0), sEdgePt);

      SmZoneTol3d sTol3d = SmTol::GetZoneTol3d(pEU->GetEdge());
      if (sEdgePt.DistanceBetweenSquared(sSurfacePt) > sTol3d*sTol3d*10000 )
      {
          TCHAR sBuff[SM_MAXSIZE];
          smos_sprintf(
              sBuff,
              _T("Vertexuse UVPoint maps to location further than 100*tol from Edge->Curve End point. Tol: %f. Dist: %f."),
              sTol3d, sEdgePt.DistanceBetween(sSurfacePt));
          SM_ASSERT_MSG( FALSE , sBuff);
      }
#endif

  }

  // split every face->edgeuse repeatedly until each result is smaller than
  // the SmTessSrfCache leaf nodes that they intersect

  while (sEdgeuses.GetSize() > 0)
  {
      SmEdgeuse * pEU = sEdgeuses.GetLast();
      sEdgeuses.RemoveLast();

      SmBSplineCurve * pUVTrim;
      
      pUVTrim = pEU->GetUVTrimCurve();

      // It is not always the case that the UV coordinates of 3d colocated vertexuses is the same.
      // When DropPoint returns multiple values, we built a trim curve above, so we can use the existence
      // of a trim curve to filter out such degeneracies.

      if (pUVTrim)
      {
          SmPoint3d sStart3d;
          SmPoint3d sEnd3d;
          if (SM_SUCCESS != pEU->NormalizedEvaluate(0.0, TRUE, sStart3d)) // TRUE = UV Eval, FALSE = 3d Eval
          {
              continue;
          }
          if (SM_SUCCESS != pEU->NormalizedEvaluate(1.0, TRUE, sEnd3d)) // TRUE = UV Eval, FALSE = 3d Eval
          {
              continue;
          }
          sStart.x = sStart3d.x;
          sStart.y = sStart3d.y;
          sEnd.x = sEnd3d.x;
          sEnd.y = sEnd3d.y;

      }
      else
      {
          sStart = pEU->GetVertexuse()->GetUVPoint();
          sEnd = pEU->GetCCWEdgeuse()->GetVertexuse()->GetUVPoint();
      }

      SM_ASSERT(sStart.x != SM_BIG_DOUBLE);
      SM_ASSERT(sEnd.x != SM_BIG_DOUBLE);


#ifdef SM_DEBUG_CODE
      if (bDebugMe)
        {
          SM_ASSERT_VALID(pEU) ;
          SmBSplineCurve *pUVTrimCurve = pEU->GetUVTrimCurve() ; // (Don't create anything in a Debug block.)
          SM_ASSERT_VALID(pUVTrimCurve) ;
        }
#endif // SM_DEBUG_CODE

      // Note: we pass FALSE for every edge: says it's not an interior edge.
      //  But currently it doesn't really use it anyway, only if it needs a 3d check.
      ULONG lNumSubdivisions;
      SER( CheckAgainstSurfaceCache( pSC,                  // in : Cache with subdivision tree to interogate
                                     sStart,               // in : start of UV segment to review
                                     sEnd,                 // in : end of UV segment to review
                                     SM_SUBDIVISION_RATIO, // in : allowed variation in size before segment needs division
                                     FALSE,                // in : TRUE = interior edge - don't check 3d lengths
                                                           //      FALSE= exterior edge - do check 3d lengths
                                     lNumSubdivisions,     // out: number of times Start/End UV segment needs to
                                                           //      to be divided so that each divisions size is about
                                                           //      the same as the smallest cache subdivision tree
                                                           //      leaf node that intersects the segments UVBounding box.
                                     TRUE)) ;              // in : for optimization only: TRUE = return after finding the first split
                                                           //      FALSE = Check all nodes for max Split, quit when splitCnt > 100.
      // when segment needs to be split
      if(   lNumSubdivisions > 0
         && !this->m_bAdvancingFront)
      {
          SmEdge * pEdge = pEU->GetEdge();
          SmBrep * pBrep = pEdge->GetBrep();
          if (!pBrep) SER(SM_ERR);

          // split the edge at its UV mid-point
          SmEdge   * pNewEdge1 = NULL, * pNewEdge2 = NULL;
          SmVertex * pNewVertex = NULL;
          if (SM_SUCCESS != pBrep->MakeVertexSplitEdge(pEdge,
                                                       pEdge->GetInterval().Evaluate(0.5),
                                                       pNewEdge1,
                                                       pNewEdge2,
                                                       pNewVertex))
            {
              // Too small to subdivide in 3D - perhaps we should
              // figure out how to do it in 2D later. This is probably
              // one of those near pole edges.
              continue;
            }

          // arrive here when pEdge and pEU have been reused to
          // point to the pNewEdge1 child of the split

          // Find the Edgeuse connected to pNewEdge2
          SmEdgeuse * pEU2 = pEU->GetCCWEdgeuse();
          SmBoolean bCCW = TRUE;

          if( pEU2->GetEdge() != pNewEdge2)
            {
              pEU2 = pEU->GetCWEdgeuse();
              bCCW = FALSE;
              if (pEU2->GetEdge() != pNewEdge2)
                { SER(SM_ERR); }
            }

          SmBoolean bSuccessMid = TRUE;

          // Set sUVGuess
          SmPoint2d sUVMid;
          SmPoint3d sUVMid3d;
          SmPoint2d sUVGuess = (sStart + sEnd) / 2.; 

          // We now need to fill in the UV values for the new vertexuse
          // Same idea as above, if there is a UV trim curve, use that.
          // Otherwise try to use DropPoint. If we can't use DropPoint,
          // build the trim curve.

          // If the new edge (pEdge2) is after the old one (pEdge), we..
          if (bCCW)
          {
              SM_ASSERT(pEU2->GetVertexuse()->GetVertex() == pNewVertex);
              SM_ASSERT(pEU2->GetCWEdgeuse()->GetVertexuse()->GetUVPoint() == sStart);

              pUVTrim = pEU2->GetUVTrimCurve();
              if (pUVTrim)
              {
                  if (SM_SUCCESS != pEU2->NormalizedEvaluate(0.0, TRUE, sUVMid3d))
                  {
                      continue;
                  }

                  sUVMid.x = sUVMid3d.x;
                  sUVMid.y = sUVMid3d.y;
              }
              else
              {
                  SmPoint3d s3dStart = pNewVertex->GetPoint();

                  pSurf->DropPoint(s3dStart, sFaceDomain, &sUVGuess, bSuccessMid, sUVMid, sGap, bIsMulti, SM_SO_MINIMIZE,
                                   NULL, &sSolutions);

                  if (!bSuccessMid || bIsMulti)
                  {
                      if (SM_SUCCESS != pEU2->NormalizedEvaluate(0.0, TRUE, sUVMid3d))
                      {
                          continue;
                      }

                      sUVMid.x = sUVMid3d.x;
                      sUVMid.y = sUVMid3d.y;
                  }
              }
              pEU2->GetVertexuse()->SetUVPoint(sUVMid);
          }
          // Otherwise, the new edge (pEdge2) is before the old one (pEdge), so we..
          else
          {
              SM_ASSERT(pEU->GetVertexuse()->GetVertex() == pNewVertex);
              SM_ASSERT(pEU->GetCWEdgeuse()->GetVertexuse()->GetUVPoint() == sStart);

              pUVTrim = pEU->GetUVTrimCurve();

              if (pUVTrim)
              {
                  if (SM_SUCCESS != pEU->NormalizedEvaluate(0.0, TRUE, sUVMid3d))
                  {
                      continue;
                  }

                  sUVMid.x = sUVMid3d.x;
                  sUVMid.y = sUVMid3d.y;
              }
              else
              {
                  SmPoint3d s3dStart = pNewVertex->GetPoint();
                  pSurf->DropPoint(s3dStart, sFaceDomain, &sUVGuess, bSuccessMid, sUVMid, sGap, bIsMulti, SM_SO_MINIMIZE,
                                   NULL, &sSolutions );

                  if (!bSuccessMid || bIsMulti)
                  {
                      if (SM_SUCCESS != pEU->NormalizedEvaluate(0.0, TRUE, sUVMid3d))
                      {
                          continue;
                      }

                      sUVMid.x = sUVMid3d.x;
                      sUVMid.y = sUVMid3d.y;
                  }
              }
              pEU->GetVertexuse()->SetUVPoint(sUVMid);
          }
          // place split Edgeuse children back into queue for further checking
          sEdgeuses.Add(pEU);
          sEdgeuses.Add(pEU2);
      }  // end need to split edge check
  } // end while edgeuses remain to be checked for size

  // all done
  return SM_SUCCESS;

} // end SmTess::SplitFaceEdgesToUVSize

/*******************************************************************//**
PURPOSE: Determine how many times a linear segment between
  two uvPoints needs to be divided so that each division is
  about the size of the smallest SmTessSrfCache subdivision tree
  leaf node that intersects the bounding box of the segment.

NOTES: 
  1. All segments are checked against the UV size of all SmTrimSrfCache
     subdivision leaf nodes that intersect the UVBounding box of the
     UV Segment.
  2. Exterior segments which are to be divided are checked
     for their 3d length and if they are too small are not divided.
  3. We stop checking at rlNumberSubdivisions == 100: that's the max we'll return.

***********************************************************************/
SmStatus SmTess::CheckAgainstSurfaceCache
  (SmTessSrfCache  * pTessCache,            // in : Cache with subdivision tree to interogate
   const SmPoint3d & crStartPoint,          // in : start of UV segment to review
   const SmPoint3d & crEndPoint,            // in : end of UV segment to review
   double            dSubdivisionRatio,     // in : allowed size variation before segment needs split
   SmBoolean         bIsInteriorEdge,       // in : TRUE = interior edge - don't check 3d lengths
                                            //      FALSE= exterior edge - do check 3d lengths
   ULONG           & rlNumberSubdivisions,  // out: number of splits Start/End UV segment needs
                                            //      so segment sizes are about the smallest xsecting
                                            //      cache subdivision tree leaf node size.
   SmBoolean         bFirstSplitOnly)       // in : for optimization only: TRUE = return after finding the first split
                                            //      FALSE = Check all nodes for max Split, quit when splitCnt > 100.
                                            //      default:[FALSE]
 const
{
#ifdef SM_DEBUG_CODE
  SmBoolean bDebugMe = FALSE;
  if (bDebugMe)
    {
      smgfx_Erase();
      smgfx_SetLook(1,4, 1,0,0); crStartPoint.Draw(); sm_GraphicsLoop();
      smgfx_SetLook(1,4, 0,1,0); crEndPoint.Draw();   sm_GraphicsLoop();
      smgfx_SetLook(1,2, 0,0,0);
      pTessCache->DrawSubdivision2D(FALSE);
      sm_GraphicsLoop();
    }
#endif

  rlNumberSubdivisions = 0;

  SM_PTR_ARRAY(sNodes, SmTreeNode, 256) ; // SmTArray<SmTreeNode*>

  // make a UVBox about 1% smaller than the UVBox containing input Start/End pts
  SmVector3d sVec  = crEndPoint - crStartPoint;
  SmPoint3d  sPnt1 = crStartPoint + 0.01 * sVec;
  SmPoint3d  sPnt2 = crStartPoint + 0.99 * sVec;
  SmPoint2d  sUVStart(sPnt1.x,sPnt1.y);
  SmPoint2d  sUVEnd(sPnt2.x,sPnt2.y);
  SmExtent2d sSegBox(sUVStart);
  sSegBox.AddPoint2d(sUVEnd);
  SmVector2d sUVSegSize = sSegBox.GetSize();

  // get all Tess subdivision tree leaf nodes that intersect segment bbox
  //SER(pTessCache->FindUVNodesOfClass(sSegBox,NULL,sNodes));

  // locals
  SmTree* pTree = pTessCache->GetTree();
  SmTreeNode* pTop = pTree->GetTopNode();
  SM_PTR_ARRAY(sStack, SmTreeNode, 100); // SmTArray<SmTreeNode *>

  // init the stack with Tree->TopNode
  sStack.Add(pTop);

  // while nodes are on the stack
  while (sStack.GetSize() > 0)
  {
      SmTreeNode* pNode = (SmTreeNode*)sStack.GetLast();
      sStack.RemoveLast();

      SM_ASSERT(pNode->m_eAuxDataType == SM_AD_AUX_DATA || pNode->m_eAuxDataType == SM_AD_BEZIER_SURFACE ||
                pNode->m_eAuxDataType == SM_AD_BEZIER_SURFACE_HEAD);

      SmBezierAux2d* pAux = (SmBezierAux2d*)pNode->m_pData;

      // skip nodes whose UVDomains don't intersect
      if (pAux->m_sUVDomain.AreDisjoint(sSegBox))
      {
          continue;
      }

      // Arrive here when UVDomains intersect
      if (pAux->m_bIsTessOnly == TRUE)
      {
          continue;
      }

      // Put leaf nodes in output or children on the stack
      if (pNode->m_pChild1 != NULL)
      { // Non-leaf node
          SmTreeNode* pNodeC2 = pNode->m_pChild2;
          SmBezierAux2d* pAuxC2 = (SmBezierAux2d*)pNodeC2->m_pData;
          if (pAuxC2 -> m_bIsTessOnly == TRUE)
          {
              sNodes.Add(pNode);
              continue;
          }

          sStack.Add(pNode->m_pChild2);
          sStack.Add(pNode->m_pChild1);
      }
      else
      { // Leaf node
          sNodes.Add(pNode);
      }
  } // While nodes are in stack

  // for every intersecting tess leaf node
  for (ULONG i=0; i<sNodes.GetSize(); i++)
    {
      SmTreeNode *pNode = sNodes[i];
      SmBezierAux2d *pAux      = (SmBezierAux2d*)pNode->m_pData;

      // skip nodes that are tessellation mesh only
      //if (pAux -> m_bIsTessOnly == TRUE)
      //{
      //    continue;
      //}
      //else if (pNode->m_pChild1 != NULL)
      //{
      //    SmTreeNode* pNodeChild1 = pNode->m_pChild1;
      //    SmBezierAux2d* pAuxChild1 = (SmBezierAux2d*)pNodeChild1->m_pData;
      //    if (pAuxChild1->m_bIsTessOnly == FALSE)
      //    {
      //        continue;
      //    }
      //}

      SmVector2d     sNodeSize = pAux->m_sUVDomain.GetSize();

      // when segment is larger than leaf node size in x diretion
      if (sUVSegSize.x > sNodeSize.x * dSubdivisionRatio)
        {
          // compute number of divisions needed to make segment about smaller than node.
          ULONG lNewSub = (ULONG)(sUVSegSize.x / (sNodeSize.x * dSubdivisionRatio));
          if (lNewSub > rlNumberSubdivisions) { rlNumberSubdivisions = lNewSub; }
          if (   bFirstSplitOnly
              || rlNumberSubdivisions >= 100) { break; }
        }

      // when segment is larger than leaf node size in y diretion
      if (sUVSegSize.y > sNodeSize.y * dSubdivisionRatio)
        {
          // compute number of divisions needed to make segment about smaller than node.
          ULONG lNewSub = ULONG(sUVSegSize.y / (sNodeSize.y * dSubdivisionRatio));
          if (lNewSub > rlNumberSubdivisions) { rlNumberSubdivisions = lNewSub; }
          if (   bFirstSplitOnly
              || rlNumberSubdivisions >= 100 ) { break; }
        }
    }

  // Check things in 3D just to make sure that we are large enough to subdivide
  // because it is possible that this is an edge which is near a pole of a surface.

  // when segment should be divided and its an exterior edge
  if(   rlNumberSubdivisions > 0
     && !bIsInteriorEdge)
    {
      // locals
      SmPoint3d         sStart3D, sEnd3D, sMid3D;
      SmPoint2d         sUVMid = (sUVStart + sUVEnd) / 2.0;
      const SmSurface * cpSurf = pTessCache->m_cpSurface;

      // get segment 3d size
      SER(cpSurf->EvaluatePoint(sUVStart,sStart3D));
      SER(cpSurf->EvaluatePoint(sUVEnd,sEnd3D));
      double dDistStartEnd = sStart3D.DistanceBetween(sEnd3D);

      // when segment is very small
      if (dDistStartEnd < m_pTessBrep->GetTolerance() * 20)
        {
          // get segment midpoint
          SER(cpSurf->EvaluatePoint(sUVMid,sMid3D));

          // watch out for closed curves
          // get distance from start to mid to end points
          double dDist = sStart3D.DistanceBetween(sMid3D) + sMid3D.DistanceBetween(sEnd3D);

          // gwc: removed minimum size check - don't subdivide degenerate 3d edges - that leads to confusing 3d Tessellations
          //  if (dDist > m_pTessBrep->GetTolerance() / 10.0)
          //    {

          // and midSegment is small - probably near a pole - don't subdivide
          if (dDist < m_pTessBrep->GetTolerance() * 20.0)
            {
              rlNumberSubdivisions = 0;
            }

          //  }
        }
    }

  // This is a temp fix to avoid huge values '5650554'
  // It is suppressing the symptoms of another problem
  if (rlNumberSubdivisions > 100)
    { rlNumberSubdivisions = 100; } // RCLxx

  return SM_SUCCESS;

} // end SmTess::CheckAgainstSurfaceCache

/*******************************************************************//**
PURPOSE: Determine if the UV polygons of the face have intersecting edges.
            If so, split at intersections.

NOTES: Works in 2d.
***********************************************************************/
SmStatus SmTess::CheckPolygonIntersections
  (SmPolyFace * pPolygon,           // in : PolyFace to check
   SmBoolean  & rbRefinementDone)   // out: TRUE = PolyFace modified when intersecting edges are found.
{
  rbRefinementDone = FALSE;

#ifdef SM_DEBUG_CODE
  SmBoolean bDebugMe = FALSE;
  if (bDebugMe)
    {
      smgfx_Erase();
      smgfx_SetLook( 1,2, 0,0,0 ); pPolygon->Draw(); sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
#endif

  SM_PTR_ARRAY(sEdges, SmPolyEdge, 256) ; // SmTArray<SmPolyEdge*>
  SmBoolean bHaveIntersections = TRUE;

  // Loop twice (mmm).
  ULONG mmm;
  for ( mmm=0; mmm<2; mmm++ )
    {
      // If no intersections on the first loop, no need to do the second.
      if ( !bHaveIntersections )
        { break; }

      bHaveIntersections = FALSE;
      SmBoolean bSplitSegment = TRUE;
      ULONG     lNumSplits    = 0;
      pPolygon->GetPolyEdges(sEdges);
      ULONG     lOriginalEdgeCount = sEdges.GetSize();
      SM_PTR_ARRAY(sEdges2,SmPolyEdge,256);

      // Keep going until we split no more segments.
      while ( bSplitSegment && lNumSplits < 100 )
        {
          bSplitSegment = FALSE;
          lNumSplits ++;
          pPolygon->GetPolyEdges( sEdges );
          if ( sEdges.GetSize() > lOriginalEdgeCount + 20 )
            { return SM_ERR; }

          // If fewer than 100 edges, just use them all,
          // otherwise use SD.
          SmBoolean bUseSD = FALSE;
          SmTree *pEdgeTree = NULL;
          SmObjDelete sCleanTree;
          if ( sEdges.GetSize() < 100 )
            {
              sEdges2.ReSet();
              sEdges2.Append(sEdges);
            }
          else
            {
              bUseSD = TRUE;
              SmExtent3d sFaceBBox;
              pPolygon->CalculateBoundingBox( sFaceBBox );
              sFaceBBox.ExpandRelative(0.01);
              ULONG lTreeInitialize = smos_Max( sEdges.GetSize()/10, 100 );
              pEdgeTree = new (m_crContext) SmTree( sFaceBBox, lTreeInitialize, lTreeInitialize/10 );
              NER( pEdgeTree );
              sCleanTree.SetObj( pEdgeTree );
              ULONG ie;
              for ( ie=0; ie<sEdges.GetSize(); ie++ )
                {
                  SmPolyEdge *pE = sEdges[ie];
                  SmExtent3d sEdgeBBox( pE->GetStartPoint() );
                  sEdgeBBox.AddPoint3d( pE->GetEndPoint() );
                  sEdgeBBox.ExpandRelative( 0.01 );
                  SER( pEdgeTree->AddToSpatialTree( sEdgeBBox, pE ));
                }
            }

          // For every edge -- or until we split a segment --
          // loop over sEdges2 checking for intersections.
          // sEdges2 is the same list as sEdges if there are fewer
          // than 100 edges, otherwise we use a proximity test
          // to get a subset of nearby edges.

          // Outer loop over all sEdges:
          ULONG i, lNumEdges = sEdges.GetSize();
          for ( i=1; i<lNumEdges && !bSplitSegment; i++ )
            {
              SmPolyEdge * pSeg1 = sEdges[i-1];
              SmEdgeuse *pEU1 = GetPolyEdgeEU(pSeg1);
              if ( pEU1 == NULL && GetPolyEdgeVU( pSeg1 ) == NULL )
                { continue; } // Fake curve added for poles of surfaces

              // Don't intersect with immediate neighbors.
              // These are checked in the inner loop.
              SmPolyEdge *pCCW = pSeg1->GetCCWPolyEdge();
              SmPolyEdge *pCW  = pSeg1->GetCWPolyEdge();

              SmPoint2d sStart2d( pSeg1->GetStartPoint().x, pSeg1->GetStartPoint().y );
              SmPoint2d sEnd2d  ( pSeg1->GetEndPoint().x,   pSeg1->GetEndPoint().y   );
              SmExtent2d sBBox1( sStart2d );
              sBBox1.AddPoint2d( sEnd2d );

              // If sEdges2 is the same as sEdges, we start at sEdges[i].
              ULONG lStart = i;
              if ( bUseSD )
                {
                  lStart = 0;
                  SmExtent3d sSeg1BBox(pSeg1->GetStartPoint());
                  sSeg1BBox.AddPoint3d(pSeg1->GetEndPoint());

                  // Temp workaround to appease Linux gcc compiler
                  // Original code with typecast has warning: dereferencing type-punned pointer will break strict-aliasing rules [-Wstrict-aliasing]
                  //SER(pEdgeTree->GetObjectsInBox(sSeg1BBox,(SmTArray<SmObject*>&)sEdges2));

                  SmTArray<SmObject*> sObjects;
                  SER(pEdgeTree->GetObjectsInBox(sSeg1BBox, sObjects));
                  sEdges2.ReSet();
                  for (ULONG ii=0;ii<sObjects.GetSize();ii++) {sEdges2.Add((SmPolyEdge*)sObjects[ii]);}

                }

#ifdef SM_DEBUG_CODE
              SmBoolean bDebugMe3 = FALSE;
              if (bDebugMe3)
                {
                  smgfx_SetLook( 2,2, 0,1,0 ); pSeg1->Draw(); sm_GraphicsLoop();
                  sm_GraphicsLoop();
                }
#endif

              // Inner loop over sEdges2 -- but quit if we split a segment.
              ULONG k, lNumEdges2 = sEdges2.GetSize();
              for ( k=lStart; k<lNumEdges2 && !bSplitSegment; k++ )
                {
                  SmPolyEdge * pSeg2 = sEdges2[k];

                  // Don't intersect with immediate neighbors, or with itself.
                  if ( pSeg2 == pSeg1 || pSeg2 == pCCW || pSeg2 == pCW )
                    { continue; }

                  SmEdgeuse *pEU2 = GetPolyEdgeEU(pSeg2);
                  if (pEU1 == pEU2) continue;
                  if (pEU2 == NULL && GetPolyEdgeVU(pSeg2) == NULL)
                    { continue; } // Fake curve added for poles of surfaces

                  if ( pEU1 == NULL && pEU2 == NULL )
                    {
                      SER(SM_ERR); // intersections with single vertex loops
                      // not yet implemented
                    }
                  if (   pEU1->GetCCWEdgeuse() == pEU2
                      || pEU1->GetCWEdgeuse()  == pEU2 )
                    { continue; }

                  // Do a quick boxing test prior to intersection
                  SmPoint2d sStart2( pSeg2->GetStartPoint().x, pSeg2->GetStartPoint().y );
                  SmPoint2d sEnd2  ( pSeg2->GetEndPoint().x,   pSeg2->GetEndPoint().y   );
                  SmExtent2d sBBox2( sStart2 );
                  sBBox2.AddPoint2d( sEnd2 );
                  sBBox2.ExpandAbsolute( SM_EFF_ZERO_SQRT );
                  if ( sBBox2.AreDisjoint( sBBox1 ))
                    { continue; }

#ifdef SM_DEBUG_CODE
                  SmBoolean bDebugMe2 = FALSE;
                  if (bDebugMe2)
                    {
                      smgfx_Erase();
                      smgfx_SetLook( 1,2, 0,0,0 ); pSeg1->Draw(); sm_GraphicsLoop();
                      smgfx_SetLook( 1,2, 0,0,1 ); pSeg2->Draw(); sm_GraphicsLoop();
                      sm_GraphicsLoop();
                    }
                  if (bDebugMe3)
                    {
                      smgfx_SetLook( 1,2, 1,0,0 ); pSeg2->Draw(); sm_GraphicsLoop();
                      sm_GraphicsLoop();
                    }
#endif // SM_DEBUG_CODE

                  // Ok, do the intersection.
                  ULONG lNumInt;
                  double aThisParams[2], aOtherParams[2], aDeviations[2];
                  double dUVTol = SM_EFF_ZERO * (1.0 + pSeg2->GetStartPoint().GetMaxDimension()
                                                     + pSeg2->GetEndPoint().GetMaxDimension());
                  SER( pSeg1->IntersectSegment( pSeg2->GetStartPoint(),
                                                pSeg2->GetEndPoint(), dUVTol, lNumInt,
                                                aThisParams, aOtherParams, aDeviations ));
                  if ( lNumInt == 0 )
                    { continue; }

                  // Check for intersections at ends - just skip
                  // Note, this used to be dUVTol * 10.0, but that's way too tight;
                  // it's only for display.   [bd, 090607]
                  // This might even be tighter than we need:
                  double dParamTol = 0.01;
                  if ( aThisParams[0] < dParamTol || aThisParams[0] > 1.0-dParamTol )
                    {
                      if ( aOtherParams[0] < dParamTol || aOtherParams[0] > 1.0-dParamTol )
                        {
                          continue;
                        }
                    }

                  // To fix the situation here we can project points from
                  // each segment to the other segment.
                  if ( mmm == 0 )
                    {
                      SmBoolean bSplit;
                      SER(TrySplitSegment( pSeg2, pSeg1->GetStartPoint(), bSplit ));
                      if (bSplit) { bSplitSegment = TRUE; }
                      SER(TrySplitSegment( pSeg2, pSeg1->GetEndPoint(),   bSplit ));
                      if (bSplit) { bSplitSegment = TRUE; }
                      SER(TrySplitSegment( pSeg1, pSeg2->GetStartPoint(), bSplit ));
                      if (bSplit) { bSplitSegment = TRUE; }
                      SER(TrySplitSegment( pSeg1, pSeg2->GetEndPoint(),   bSplit ));
                      if (bSplit) { bSplitSegment = TRUE; }
                      if ( bSplitSegment )
                        {
                          rbRefinementDone = TRUE;
                          break;
                        }
                    }

                  // If we made it to here than we need to make sure that we
                  // did not have a real intersection. Because we were unable
                  // to drop points and do any work.

                  // Check whether the intersection is a good one, to a tight tol.
                  SmBoolean bFoundGoodInt = FALSE;
                  SmExtent1d sPGIvl( dParamTol, 1.0 - dParamTol ); // (See comment above about dParamTol.)
                  ULONG j;
                  for ( j=0; j<lNumInt; j++ )
                    {
                      SmPoint3d sUVPnt;
                      if ( !sPGIvl.ContainsValue( aOtherParams[j] )) continue;
                      if ( !sPGIvl.ContainsValue( aThisParams[j]  )) continue;

                      // Neither param is near an end.
                      SER( pSeg1->EvaluatePoint( aThisParams[j], sUVPnt ));
                      SmPoint3d sUVPnt2;
                      SER( pSeg2->EvaluatePoint( aOtherParams[j], sUVPnt2 ));
                      double dTol = SM_EFF_ZERO * (1.0 + sUVPnt.GetMaxDimension());
                      double dDistSq = sUVPnt.DistanceBetweenSquared( sUVPnt2 );
                      if ( dDistSq < dTol*dTol )
                        {
                          bFoundGoodInt = TRUE;
                          bHaveIntersections = TRUE;
                        }
#ifdef SM_DEBUG_CODE
                      if (bDebugMe)
                        {
                          SmPoint3d sUVPoint;
                          SER(pSeg1->EvaluatePoint( aThisParams[j], sUVPoint ));
                          smgfx_SetLook( 3,5, 1,0,0 ); sUVPoint.Draw(); sm_GraphicsLoop();
                          sm_GraphicsLoop();
                        }
#endif
                    }  // end for each intersection

                  if ( bFoundGoodInt && mmm == 1 )
                    {
                      // The thing to try here is to remove one or both ends of each
                      // segment that intersected and see if that solves the problems.
                      // It could be one of those cases where the curves cross just
                      // before getting to the vertex and we inserted a new vertex
                      // or two in that bad area.
                      SM_PTR_ARRAY(sBadTopoVerts1, SmPolyVertex, 16) ; // SmTArray<SmPolyVertex*>
                      SM_PTR_ARRAY(sBadTopoVerts2, SmPolyVertex, 16) ; // SmTArray<SmPolyVertex*>
                      SER( FindInsideOutTopoVertices( pSeg1, pSeg2, sBadTopoVerts1, sBadTopoVerts2 ));

                      // First take care of end condition where two curves overlap
                      // just before getting to the real vertex.  This is the most
                      // common bad case.
                      if ( sBadTopoVerts1.GetSize() > 0 || sBadTopoVerts2.GetSize() > 0 )
                        {
#ifdef SM_DEBUG_CODE
                          if (bDebugMe)
                            {
                              smgfx_Erase();
                              smgfx_SetColor(0,0,0);
                              pPolygon->Draw();
                              sm_GraphicsLoop();
                            }
#endif
                          rbRefinementDone = TRUE; // Refinement this time means
                                                   // deletion of things.
                          bSplitSegment    = TRUE; // Not really split but same effect

                          ULONG jj;
                          for ( jj=0; jj<sBadTopoVerts1.GetSize(); jj++ )
                            { SER( RemoveTopologicalVertex( sBadTopoVerts1[jj] )); }
                          for ( jj=0; jj<sBadTopoVerts2.GetSize(); jj++ )
                            { SER( RemoveTopologicalVertex( sBadTopoVerts2[jj] )); }
#ifdef SM_DEBUG_CODE
                          if (bDebugMe)
                            {
                              smgfx_Erase();
                              smgfx_SetColor(0,0,0);
                              pPolygon->Draw();
                              sm_GraphicsLoop();
                            }
#endif
                          break;
                        }
                      else
                        {
                          // Big problems here because we are unable to resolve UV intersections.
                          // It is likely that the UV curves cross each other.
                          return SM_ERR;
                        }

                    } // end if bFoundGoodInt and mmm==1.
                } // end inner loop, over sEdges2
            } // end outer loop, over sEdges

          if ( lNumSplits >= 100 )
            {
              return SM_ERR;
            }
        } // While
    } // end Loop twice (mmm)

#ifdef SM_DEBUG_CODE
  if (bDebugMe)
    {
      smgfx_Erase();
      smgfx_SetLook( 1,2, 0,0,0 ); pPolygon->Draw(); sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
#endif

  return SM_SUCCESS;

} // end SmTess::CheckPolygonIntersections

/*******************************************************************//**
PURPOSE: Remove a topological vertex in the polygon and the real
    topology.

NOTES: 
***********************************************************************/
SmStatus SmTess::RemoveTopologicalVertex(SmPolyVertex *pVertexToRemove)
{
  SmPolyEdge *pPEdge = pVertexToRemove->GetFirstPolyEdge(); NER(pPEdge);
  SmPolyBrep *pPBrep = pPEdge->GetPolyBrep();

  SmEdgeuse *pEdgeuse = GetPolyEdgeEU(pPEdge); NER(pEdgeuse);
  SmEdge *pEdgeToDelete = pEdgeuse->GetEdge(); NER(pEdgeToDelete);

  SmVertexuse *pVUToRemove = GetPolyEdgeVU(pPEdge); NER(pVUToRemove);

  SmVertex *pVToRemove = pVUToRemove->GetVertex(); NER(pVToRemove);

  SmBrep *pBrep = pVToRemove->GetBrep();

  SM_PTR_ARRAY(sRemEdges, SmPolyEdge, 16) ; // SmTArray<SmPolyEdge*>

  if (pVToRemove->IsTopologicalVertex())
    {
      SER(pPBrep->DeleteTopologicalVertex(pVertexToRemove,sRemEdges));
      SER(pBrep->DeleteTopologicalVertex(pVToRemove,pEdgeToDelete));
    }

  return SM_SUCCESS;

} // end SmTess::RemoveTopologicalVertex

/*******************************************************************//**
PURPOSE: Check Polygon Loop nesting.

NOTES: 
***********************************************************************/
SmStatus SmTess::CheckPolygonNesting
  (SmPolyFace * pPolygon,              // in : Polyface to check
   SmBoolean  & rbLoopRefinementDone)  // out: TRUE = Edges were split when Points in one inner loop were       
                                       //      found to be outside the outer loop or inside one of the other inner loops.
{
  // init output
  rbLoopRefinementDone = FALSE;

  // locals
  ULONG ii, jj ;
  SM_PTR_ARRAY(sPolyLoops, SmPolyLoop, 256) ; // SmTArray<SmPolyLoop*>
  pPolygon->GetPolyLoops(sPolyLoops);

  // no work - no loops
  if (sPolyLoops.GetSize() == 0)
    { return SM_SUCCESS; }

  // Grab a point of each inner loop and test for it to be
  // inside of outer loop and outside of all inner loops.
  for (ii=1; ii<sPolyLoops.GetSize(); ii++)
    {
      SmPolyLoop * pLoop = sPolyLoops[ii];
      SmPolyEdge * pEdge = pLoop->GetFirstPolyEdge();
#ifdef SM_DEBUG_CODE
      SmBoolean bDebugMe = FALSE;
      if (bDebugMe)
        {
          smgfx_Erase();
          smgfx_SetLook(3,4, 1,0,0) ; pEdge->Draw(); sm_GraphicsLoop();
          smgfx_SetLook(1,2, 0,0,0); pPolygon->Draw(); sm_GraphicsLoop();
          sm_GraphicsLoop();
        }
#endif // SM_DEBUG_CODE

      // for all loops
      for (jj=0; jj<sPolyLoops.GetSize(); jj++)
        {
          // skip testing PolyLoop against itself
          if (jj == ii)
            { continue; }

          //
          SmPolyContainmentType ePCType;
          SER( sPolyLoops[jj]->ContainsPolyLoop( pLoop, ePCType, pPolygon->GetTolerance() ));
          SmBoolean bInside =   ePCType == SM_PCT_INSIDE
                             || ePCType == SM_PCT_ON_BOUNDARY;

          // okay when outer PolyLoop[jj] contains 1st PolyLoop[ii]->PolyEdge
          //      or   inner PolyLoop[jj] does not contain 1st PolyLoop[ii]->PolyEdge
          if(   ( bInside && jj==0)
             || (!bInside && jj!=0))
            { continue; }

          // Here we have incorrect nesting of things see if we
          // can fix it by splitting the closest edge of the loop.
          SmBoolean bSplitMade;
          SER(SplitSegmentOfLoop(sPolyLoops[jj],pEdge->GetStartPoint(),bSplitMade));
          if (!bSplitMade)
            {
              SER(SplitSegmentOfLoop(sPolyLoops[jj],pEdge->GetEndPoint(),bSplitMade));
              if (!bSplitMade) SER(SM_ERR); // Problem here because we are
              // unable to split a segment.  Perhaps a correction could be
              // to find the closest edge in the polygon and then split that.
            }
          rbLoopRefinementDone = TRUE;
          return SM_SUCCESS;
        }
    }

  return SM_SUCCESS;

} // end SmTess::CheckPolygonNesting

/*******************************************************************//**
PURPOSE: Given two segments which intersect on their interior, find
    the vertices which are on the wrong side.  This will only work for
    topological vertices.

NOTES: This work is done in the parameter space of the surface.
***********************************************************************/
SmStatus SmTess::FindInsideOutTopoVertices
  (const SmPolyEdge *cpSeg1,
   const SmPolyEdge *cpSeg2,
   SmTArray<SmPolyVertex*> & rSeg1BadVerts,
   SmTArray<SmPolyVertex*> & rSeg2BadVerts)
  const
{
  rSeg1BadVerts.ReSet();
  rSeg2BadVerts.ReSet();

  SmPoint3d sSeg1Start = cpSeg1->GetStartPoint();
  SmPoint3d sSeg1End = cpSeg1->GetEndPoint();

  SmPoint3d sSeg2Start = cpSeg2->GetStartPoint();
  SmPoint3d sSeg2End = cpSeg2->GetEndPoint();

  ULONG lSide;
  SER(smgu_SegmentPoint2DSide(sSeg1Start,sSeg1End,sSeg2Start,lSide));
  if (lSide == 2)
    {
      SmPolyVertex *pPV = cpSeg2->GetStartPolyVertex();
      SmVertexuse *pVU = GetPolyEdgeVU(cpSeg2);
      if (pVU)
        {
          SmVertex *pV = pVU->GetVertex();
          if (pV && pV->IsTopologicalVertex())
            {
              rSeg2BadVerts.Add(pPV);
            }
        }
    }
  SER(smgu_SegmentPoint2DSide(sSeg1Start,sSeg1End,sSeg2End,lSide));
  if (lSide == 2)
    {
      SmPolyVertex *pPV = cpSeg2->GetEndPolyVertex();
      SmPolyEdge *pCCW = cpSeg2->GetCCWPolyEdge();
      SmVertexuse *pVU = GetPolyEdgeVU(pCCW);
      if (pVU)
        {
          SmVertex *pV = pVU->GetVertex();
          if (pV && pV->IsTopologicalVertex())
            {
              rSeg2BadVerts.Add(pPV);
            }
        }
    }

  SER(smgu_SegmentPoint2DSide(sSeg2Start,sSeg2End,sSeg1Start,lSide));
  if (lSide == 2)
    {
      SmPolyVertex *pPV = cpSeg1->GetStartPolyVertex();
      SmVertexuse *pVU = GetPolyEdgeVU(cpSeg1);
      if (pVU)
        {
          SmVertex *pV = pVU->GetVertex();
          if (pV && pV->IsTopologicalVertex())
            {
              rSeg1BadVerts.Add(pPV);
            }
        }
    }

  SER(smgu_SegmentPoint2DSide(sSeg2Start,sSeg2End,sSeg1End,lSide));
  if (lSide == 2)
    {
      SmPolyVertex *pPV = cpSeg1->GetEndPolyVertex();
      SmPolyEdge *pCCW = cpSeg1->GetCCWPolyEdge();
      SmVertexuse *pVU = GetPolyEdgeVU(pCCW);
      if (pVU)
        {
          SmVertex *pV = pVU->GetVertex();
          if (pV && pV->IsTopologicalVertex())
            {
              rSeg1BadVerts.Add(pPV);
            }
        }
    }

  return SM_SUCCESS;

} // end SmTess::FindInsideOutTopoVertices

/*******************************************************************//**
PURPOSE: Create and store in pSC->mTS_pPolyBrep an SmPolyBrep containing
         a piecewise linear approximations of the Loops and Edges in pFace.

NOTES: 
  0. where pSC = SmTessSrfCache for input pFace - the pFace/pSC pairing is stored in SmRelation, m_vCache
  1. creates pSC->mTS_pPolyBrep and adds to that
       1 SmPolyFace per simple outer pFace->Loop
           (complex pFace->Loops are broken into simple Loops yielding a few PolyFaces)
       1 inner SmPolyLoop for each inner pFace->Loop,
       1 SmPolyEdge for each pFace->Loop->Edge,
       1 SmPolyVertex for each pFace->Loop->Vertex.
       No Inner PolyVerts or PolyEdges at this time.
       PolyLoops not broken up into triangles at this time.

  2. sets pSC->mTS_pPolyBrep->m_bOKBackPtrs == TRUE. The intent
       is while tessellating those ptrs are current. After Tessellation
       they may be stale.

  3. Splits pFace->Edges to Fix problems that may be caused by
     using linear polylines to approximate the curves of pFace.
      a. Split all polyedges wherever they intersect one another.
      b. Split loop->polyedges if points from one inner loop end up
         outside the outer loop or inside one of the other inner loops.

  4. Sets rbRefinementDone == TRUE if any of the polyEdges were split to fix problems.

SIDE EFFECTS ---
  pSC->mTS_pPolyBrep = set to point at newly allocated PolyBrep
                       containing one PolyFace and one PolyLoop
                       per pFace->Loop and one PolyEdge per pFace->Edge.
***********************************************************************/
SmStatus SmTess::MakePolyBrepFromFaceEdges
 (SmFace    * pFace,              // in : target face
  SmBoolean & rbRefinementDone)   // out: FALSE= PSC->MTS_pPolyBrep built without any further subdivision of pFace->Edges
                                  //      TRUE = pFace->Edges were subdivided to resolve problems caused by
                                  //             linearization.
{
  // init output
  rbRefinementDone = FALSE;

  // locals
  SmFaceuse      * pUpperFU = pFace->GetUpwardFaceuse();
  SmTessSrfCache * pSC      = m_vCache.GetSecond(pFace); NER(pSC);

  // This tolerance value is used when creating new Poly topology.
  // The tolerance of the face is appropriate for that.  [B231]
  // (Later:) However, what appears to be appropriate doesn't always work. [B233]
  //double dTolerance = SM_EFF_ZERO_SQRT;  // pFace->GetTolerance();
  double dTolerance = pFace->GetTolerance(); // JLMCC review this change from FS

  // delete any old SmTessSrfCache PolyBrep
  if (pSC->mTS_pPolyBrep)
    {
      dTolerance = pSC->mTS_pPolyBrep->GetTolerance();
      delete pSC->mTS_pPolyBrep; pSC->mTS_pPolyBrep = NULL;
    }

  // get all face upward edgeuses
  SM_PTR_ARRAY(sEdgeuses, SmEdgeuse, 32) ; // SmTArray<SmEdgeuse*>
  pUpperFU->GetEdgeuses(sEdgeuses);

  // Make a PolyBrep linear model approximation of nonLinear pFace
  // by approximating every pFace->pLoop->Edge with a Line
  // (assumes all pFace->Edges have already been subdivided short enough to be nearly linear)
  SmPolyBrep * pPolyBrep = NULL ;
  SmPolyFace * pPolyFace = NULL ;
  SmStatus eStat = SM_SUCCESS;
  eStat = MakePolygon(pFace,      // in : face to approximate as a polygon
                      dTolerance, // in : 3d tol used for creating Poly topo
                      pPolyBrep,  // out: UV PolyBrep containing
                                  //        1 SmPolyFace per simple outer pFace->Loop
                                  //            (complex pFace->Loops are broken into simple Loops yielding a few PolyFaces)
                                  //        1 inner SmPolyLoop for each inner pFace->Loop,
                                  //        1 SmPolyEdge for each pFace->Loop->Edge,
                                  //        1 SmPolyVertex for each pFace->Loop->Vertex.
                                  //        No Inner PolyVerts or PolyEdges at this time.
                                  //        PolyLoops not broken up into triangles at this time.
                      pPolyFace); // out: UV PolyFace polygon approximation to input pFace
  if(SM_SUCCESS != eStat)
    {
      return eStat;
    }
  SmObjDelete sCleanPolyBrep(pPolyBrep);

  // while in the tess sequence - mark all mTS_pPolyBrep backpointers as current
  pPolyBrep->SetOKBackPtrs(TRUE) ;

#ifdef SM_DEBUG_CODE
  SmBoolean bDebugMe = FALSE;
  if (bDebugMe)
    {
      pPolyBrep->Dump() ;
      SM_ASSERT_VALID(pPolyFace) ;

      smgfx_Erase();
      smgfx_SetLook(1,2, 0,0,0) ; if(pPolyBrep) pPolyBrep->Draw(TRUE); sm_GraphicsLoop();
      smgfx_SetLook(3,4, 1,0,0) ; if(pFace) pFace->Draw(); sm_GraphicsLoop();
      smgfx_SetLook(1,2, 1,0,0) ; if(pFace) pFace->DrawUVCurves(FALSE); sm_GraphicsLoop();
      smgfx_SetLook(2,3, 0,1,0) ; if(pSC)   pSC->DrawSubdivision2D(FALSE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(3,4, 0,0,1) ; pPolyFace->Draw(); sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

  // Split any polyEdges which intersect one another.
  // This is possible because SmFace curves are being approximated by SmPolyEdgeslines.
  SmBoolean bIntersectionRefinementDone = FALSE;
  if (CheckPolygonIntersections(pPolyFace,bIntersectionRefinementDone) != SM_SUCCESS)
    {
      // return SM_ERR;    [ bd, 090607 ]
      MSG(_T("SmTess::MakePolyBrepFromFaceEdges - ERROR from CheckPolygonIntersections()"));
    }

  // Check that a point from within each inner loop is contained
  // within the outer loop and is not contained within any other inner loop.
  // If any problems are found try to fix them by additional edge splits
  //  on the assumption that a truer approximation of the SmFace curves
  //  will be less problematic.
  SmBoolean bLoopRefinementDone = FALSE;
  if (CheckPolygonNesting(pPolyFace,bLoopRefinementDone) != SM_SUCCESS)
    {
      // return SM_ERR;    [ bd, 090607 ]
      MSG(_T("SmTess::MakePolyBrepFromFaceEdges - ERROR from CheckPolygonNesting()"));
    }

  // remember if any refinement fixes of the approximating polygon were required
  if(   bIntersectionRefinementDone
     || bLoopRefinementDone)
    { rbRefinementDone = TRUE; }

#ifdef SM_DEBUG_CODE
  if (bDebugMe)
    {
      pPolyBrep->Dump() ;

      smgfx_Erase();
      smgfx_SetLook(1,2, 0,0,0) ; pFace->Draw(); sm_GraphicsLoop();
      smgfx_SetLook(3,4, 0,0,1) ; pPolyFace->Draw(); sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

  // make PolyBrep permanent and saved within SmTessSrfCache
  sCleanPolyBrep.Clear();
  pSC->mTS_pPolyBrep = pPolyBrep;

  // all done
  return SM_SUCCESS;

} // end SmTess::MakePolyBrepFromFaceEdges

/*******************************************************************//**
PURPOSE: Create a PolyFace in the given Brep that goes around
         the Node->LoopDomain, CCW from the upper left corner.

NOTES: Adds a SmPolyRegion->SmPolyShell->SmPolyFace->SmPolyLoop->SmPolyEdge[n]
       structure to this SmPolyBrep. Where n = numper of TreeVertices
       surrounding the given pNode
***********************************************************************/
// GWCTreeVertexTemp
//  static SmStatus sm_MakePolygonFromNode
//   (SmPolyBrep       * pPolyBrep,      // in : PolyBrep to receive new PolyEdges
//    SmTreeNode       * pNode,          // in : Node whose LoopVertices mark the perimeter of the new PolyLoop
//    SmPolyShell     *& rpTgtPolyShell, // i/o: NULL    = allocate a New PolyShell to contain rpNewPolyFace
//                                       //      NotNULL = use to contain rpNewPolyFace
//    SmPolyFace      *& rpNewPolyFace)  // out: New PolyFace created by this call
//  {
//    // locals
//    ULONG ii ;
//    SmTree                 * pTree      = pNode->m_pTree ;
//    double                   dDistTol3d = pPolyBrep->GetTolerance() ;
//    SmTArray<SmPolyVertex *> sPolyVertices ;
//    SmTArray<ULONG>          sVertexLoop ;
//    pNode->GetVertexLoop(SM_BIG_ULONG, sVertexLoop) ;
//
//    // for every TreeVertex index - accumulate sPolyVertices array with got/built PolyVertices
//    for(ii=0;ii<sVertexLoop.GetSize();ii++)
//      {
//        ULONG indx = sVertexLoop[ii] ;
//
//        // fetch PolyVertex for this TreeVertex
//        SmPolyVertex *pPolyVertex = pTree->GetTreeVertexPolyVertex(indx) ;
//
//        // When No PolyVertex
//        if(pPolyVertex == NULL)
//          {
//            // make a PolyVertex
//            pPolyVertex = new( pPolyBrep ) SmPolyVertex( pTree->GetTreeVertexUVPoint(indx), dDistTol3d );
//
//            // note: normally new PolyVertices are included in pPolyBrep->m_pVertexListHead list
//            //       with a call to pPolyBrep->m_pVertexListHead->PostInsert(pPolyVertex);
//            //       In this case all these vertices are sent to CreatePolyFace() which passes
//            //       them to SmPolyLoop::AddPolyEdge() where they get inserted into the VertexListHead.
//            //       So, skip the call here.
//
//            // remember TreeVertex<->PolyVertex mapping
//            pTree->SetTreeVertexPolyVertex(indx, pPolyVertex) ;
//          }
//
//        // accumulate the PolyVertex List
//        sPolyVertices.Add(pPolyVertex) ;
//
//      } // end iter every TreeVertex building the sPolyVertices list
//
//    // Build a PolyBrep->PolyFace from sPolyVertices
//    SmStatus sRtn = pPolyBrep->CreatePolyFace(NULL, rpTgtPolyShell, sPolyVertices, rpNewPolyFace);
//
//    // remember the PolyShell
//    rpTgtPolyShell = rpNewPolyFace ? (SmPolyShell *)rpNewPolyFace->GetPolyShell() : NULL ;
//
//    // all done
//    return sRtn;
//
//  } // end sm_MakePolygonFromNode

/*******************************************************************//**
PURPOSE: Build a loop from a lamina boundary edges.  Connect new edges
   to lamina edges so they will not be lamina edges any more.  New edges
   have opposite orientation as old edges.

NOTES: returned rpNewLoop is not yet contained within a PolyFace

   rPolyEdgesOfLoop is built by TraceLaminaLoop() in reverse order.
***********************************************************************/
SmStatus SmTess::BuildLaminaLoop
 (SmTArray<SmPolyEdge*> & rPolyEdgesOfLoop,  // i/o: Input edges of loops - replaced by new edges
  SmPolyLoop           *& rpNewPolyLoop)     // out: NewLoop
 const
{
  // check state - must have edges
  if (rPolyEdgesOfLoop.GetSize() == 0)
    { SER(SM_ERR); }

  // locals
  ULONG ii,    lNumEdges = rPolyEdgesOfLoop.GetSize();
  SmPolyEdge * pPolyEdge = rPolyEdgesOfLoop[0];
  SmPolyBrep * pPolyBrep = pPolyEdge->GetPolyBrep();
  SM_PTR_ARRAY(sNewEdges, SmPolyEdge, 256) ; // SmTArray<SmPolyEdge*>

#ifdef SM_DEBUG_CODE
  SmBoolean bDebugMe = FALSE;
  if (bDebugMe)
    {
      TCHAR sBuff[SM_TBLOCK_SIZE] ;
      smos_sprintf(sBuff, _T("\nSmTess::BuildLaminaLoop: rPolyEdgesOfLoop:[%ld]"),rPolyEdgesOfLoop.GetSize());
      smos_WriteBuffer(sBuff) ;

      for(ULONG di=lNumEdges-1;di!=lNumEdges;di=((di+lNumEdges)-1)%lNumEdges)
        {
          SmPolyEdge    * pDbgPolyEdge        = rPolyEdgesOfLoop[di] ;
          SmPolyVertex  * pDbgStartPolyVertex = pDbgPolyEdge->GetStartPolyVertex() ;
          SmPolyVertex  * pDbgEndPolyVertex   = pDbgPolyEdge->GetEndPolyVertex() ;

          // Start
          smos_sprintf(sBuff, _T("\n [%3ld] PolyEdge:[0x%p], StartPolyVertex:[0x%p], StartPoint:"),di,pDbgPolyEdge,pDbgStartPolyVertex);
          smos_WriteBuffer(sBuff);
          pDbgStartPolyVertex->GetPoint().Dump() ;

          smos_sprintf(sBuff, _T("\n                                       EndPolyVertex  :[0x%p], EndPoint  :"),pDbgEndPolyVertex);
          smos_WriteBuffer(sBuff);
          pDbgEndPolyVertex->GetPoint().Dump() ;
        }
    }
#endif // SM_DEBUG_CODE

  // make new Loop - not connected to any PolyFace
  // gwc: don't call pFace->StartEdgePolyLoop() which also creates NewLoop but attached to pFace
  SmPolyLoop * pNewPolyLoop = new (pPolyBrep) SmPolyLoop(NULL) ;

  // for every input PolyEdge - Add a PolyEdge to pNewPolyLoop
  for (ii=0; ii<lNumEdges; ii++)
    {
      SmPolyEdge   * pOldPolyEdge     = rPolyEdgesOfLoop[ii];
      SmPoint3d      sStartPt         = pOldPolyEdge->GetStartPoint();
      SmPoint3d      sEndPt           = pOldPolyEdge->GetEndPoint();
      SmPolyVertex * pStartPolyVertex = pOldPolyEdge->GetStartPolyVertex();
      SmPolyVertex * pEndPolyVertex   = pOldPolyEdge->GetEndPolyVertex();
      SmPolyEdge   * pNewPolyEdge;

      // PolyEdges are in reverse order - reverse the direction of each PolyEdge with this call
      SER(pNewPolyLoop->AddPolyEdge
            (pOldPolyEdge->GetTolerance(), // in : min dist between distinct points
             sEndPt,                       // in : Line start position
             sStartPt,                     // in : Line end position
             pEndPolyVertex,               // in : when m_pLastEndPolyVertex NotNULL (set on last call through pOptEndPolyVertex),
                                           //           m_pLastEndPolyVertex is Start PolyVertex for PolyLoop->PolyEdge
                                           //      else: pOptStartPolyVertex NotNULL = Start PolyVertex for PolyLoop->PolyEdge,
                                           //                                NULL    = create New PolyVertex for 1stPolyEdge
             pStartPolyVertex,             // in : NotNULL = stored in m_pLastEndPolyVertex to be
                                           //                Start PolyVertex for next AddPolyEdge() call.
             pPolyBrep,                    // in : provides context for new obj construction and
                                           //      accumulates new PolyVertices on its m_pVertexListHead list
             pNewPolyEdge));               // out: new edge, stitched to radial partners when pOptStartPolyVertex and pOptEndPolyVertex are NotNULL

      sNewEdges.Add(pNewPolyEdge);

    } // end iter every PolyEdge building pNewPolyLoop

  // finish pNewPolyLoop
  pNewPolyLoop->FinishPolyEdgeLoop(pPolyBrep);

  // gwc: GLUE called moved inside of AddPolyEdge call
  //
  //  SM_PTR_ARRAY(sEdgesBetween,SmPolyEdge,16);
  //  lNumEdges = rPolyEdgesOfLoop.GetSize();
  //  for (ii=0; ii<lNumEdges; ii++)
  //    {
  //      // Glue (make radial partners) rPolyEdgesOfLoop[ii] and sNewEdges[ii] without deleting any PolyEdge,
  //      //   glue endPVerts deleting pOtherEdge->PolyVerts
  //      SER(pPolyBrep->GlueEdges(rPolyEdgesOfLoop[ii], // in : Target PolyEdge1
  //                               sNewEdges[ii],        // in : Target PolyEdge2
  //                               SM_OT_OPPOSITE,       // in : oneof SM_OT_SAME, SM_OT_OPPOSITE
  //                               sEdgesBetween));      // out: PolyEdges between glued vertices (made zero length by gluing)
  //                                                     // out: PolyVerts deleted when PolyVerts were glued (stale pointers), NULL to ignore, default:[NULL]
  //                                                     // out: associated surviving PolyVerts when PolyVerts were glued, NULL to ignore, default:[NULL]
  //      rPolyEdgesOfLoop[ii] = sNewEdges[ii];
  //    }

  // set output
  rpNewPolyLoop = pNewPolyLoop;

#ifdef SM_DEBUG_CODE
  if (bDebugMe)
    {
      SmPolyFace     * pPolyFace = pNewPolyLoop->GetPolyFace() ;
      SmFace         * pFace     = pPolyFace ? pPolyFace->GetOriginalFace() : NULL ;
      SmTessSrfCache * pSC       = pFace ? m_vCache.GetSecond(pFace) : NULL ;

      pPolyBrep->Dump() ;
      SM_ASSERT_VALID(pNewPolyLoop) ; // PolyLoop is not yet attached to a PolyFace - expect some errors

      smgfx_Erase();
      smgfx_SetLook(1,2, 0,0,0) ; if(pPolyBrep) pPolyBrep->Draw(TRUE); sm_GraphicsLoop();
      smgfx_SetLook(3,4, 1,0,0) ; if(pNewPolyLoop) pNewPolyLoop->Draw(); sm_GraphicsLoop();
      smgfx_SetLook(2,3, 0,1,0) ; if(pSC)   pSC->DrawSubdivision2D(FALSE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(3,4, 0,0,1) ; if(pPolyFace) pPolyFace->Draw(); sm_GraphicsLoop(); // pPolyFace is always going to be NULL
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

  // all done
  return SM_SUCCESS;

} // end SmTess::BuildLaminaLoop

/*******************************************************************//**
PURPOSE: Trace a loop of lamina PolyEdges starting at the given lamina
    PolyEdge.  Note that this method assumes that all polygons have the same
    relative orientation.

NOTES: We will trace the loop in reverse order so construction
       will be easier.

  increments an unlocked mark value
***********************************************************************/
SmStatus SmTess::TraceLaminaLoop
 (SmPolyEdge            * pStartPolyEdge,   // in :
  SmTessSrfCache        * pTessCache,       // in :
  SmTArray<SmPolyEdge*> & rPolyLoopEdges)   // out:
{
  // error - StartEdge is not lamina
  if ( !pStartPolyEdge->IsLamina() )
    { SER(SM_ERR); }

  // locals
  SM_PTR_ARRAY(sPolyVertexEdges, SmPolyEdge, 64) ; // SmTArray<SmPolyEdge *>
  SM_PTR_ARRAY(sNextPolyEdges,   SmPolyEdge, 64) ; // SmTArray<SmPolyEdge *>

#ifdef SM_DEBUG_CODE
  SmBoolean bDebugMe = FALSE;
  if (bDebugMe)
    {
      smgfx_Erase();
      smgfx_SetLook(1,2, 0,0,0); pStartPolyEdge->GetPolyBrep()->Draw(); sm_GraphicsLoop();
      smgfx_SetLook(2,2, 1,0,0); pStartPolyEdge->Draw(); sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

  // increment and lock an unlocked mark
  SmNewMarkAndLock sMarkLock( pStartPolyEdge->GetContext(), SM_MT_ALLMARKS) ; // increment and lock any unlocked mark
  SmMarkType eMarkType = sMarkLock.GetMarkType() ;

  // Do this lamina tracing two times.  The first time we do subdivisions
  // to insert vertices of equivalent frequency as neighboring quads.
  // The second time we collect things again.
  for ( ULONG ii=0; ii<2; ii++ )
    {
      sMarkLock.NewMark() ;
      pStartPolyEdge->Mark(eMarkType);
      rPolyLoopEdges.ReSet();
      rPolyLoopEdges.Add(pStartPolyEdge);

      SmBoolean bDone = FALSE;

      // This while loop goes until we trace an entire loop.
      while ( !bDone )
        {
          // Trace in reverse order: start with last.
          SmPolyEdge   * pCurrPolyEdge = rPolyLoopEdges.GetLast();
          SmPolyVertex * pStartPolyV   = pCurrPolyEdge->GetStartPolyVertex();

          // For each edge incident on this edge's start vertex,
          // collect all lamina edges, into sNextPolyEdges.

          pStartPolyV->GetPolyEdges( sPolyVertexEdges );
          sNextPolyEdges.ReSet();
          ULONG lNumVEdges = sPolyVertexEdges.GetSize();
          for (ULONG jj=0; jj<lNumVEdges; jj++)
            {
              SmPolyEdge *pPolyEdge = sPolyVertexEdges[jj];
              if ( !pPolyEdge->IsLamina() )     { continue; }
              if ( pPolyEdge == pCurrPolyEdge ) { continue; }
              sNextPolyEdges.Add( pPolyEdge );
#ifdef SM_DEBUG_CODE
              SmBoolean bDebugMe2 = FALSE;
              if (bDebugMe2)
                {
                  smgfx_Erase();
                  smgfx_ChangeColor(jj != 0);
                  pCurrPolyEdge->Draw();
                  sm_GraphicsLoop();
                  smgfx_ChangeColor();
                  pPolyEdge->Draw();
                  sm_GraphicsLoop();
                }
#endif
            } // end loop collecting lamina edges at this vertex

          // sNextPolyEdges now contains all lamina edges at this vertex,
          // other than pCurrPolyEdge.

          if (sNextPolyEdges.GetSize() == 0)
            {
              bDone = TRUE;
              continue;
            }

          SmPolyEdge *pNextPolyE = sNextPolyEdges[0];

          // If there is more than one candidate edge at this vertex,
          // Find the closest clockwise edge to the current one to make
          // the turn in ambiguous cases.
          if ( sNextPolyEdges.GetSize() > 1 )
            {
              SmVector3d sBin     = pCurrPolyEdge->ComputeBinormal();
              SmVector3d sEdgeVec =  pCurrPolyEdge->GetEndPoint()
                                   - pCurrPolyEdge->GetStartPoint();
              // Note: if sEdgeVec and sBin are both small, then sNorm
              // will be really tiny, and could bomb out here, even if
              // sEdgeVec and sBin are big enough on their own.
              // So unitize them before taking their product.  [B138]
              SER( sEdgeVec.Unitize() );
              SER( sBin    .Unitize() );

              SmVector3d sNorm = sEdgeVec * sBin;
              SER( sNorm.Unitize() );

              double dMinAngle = SM_BIG_DOUBLE;
              for (ULONG jj=0; jj<sNextPolyEdges.GetSize(); jj++)
                {
                  SmPolyEdge *pTest = sNextPolyEdges[jj];
                  SmVector3d sTestVec;
                  if (pTest->GetStartPolyVertex() == pStartPolyV)
                    {
                      sTestVec = pTest->GetEndPoint()-pTest->GetStartPoint();
                    }
                  else
                    {
                      sTestVec = pTest->GetStartPoint()-pTest->GetEndPoint();
                    }
                  double dTestAng;
                  SER( sNorm.CCWAngleBetween( sEdgeVec, sTestVec, dTestAng ));
                  // Make it positive
                  if (dTestAng < 0.0)
                    {
                      dTestAng += SM_PI*2.0;
                    }
                  // Now turn it from CCW to CW angle
                  dTestAng = 2.0*SM_PI - dTestAng;

                  if ( dTestAng < dMinAngle)
                    {
                      dMinAngle = dTestAng;
                      pNextPolyE = pTest;
                    }
                } // end loop over all lamina edges
            } // end if more than one lamina edge

          // Got next edge.  Have we already been here?
          if ( pNextPolyE->IsMarked(eMarkType) )
            {
              break;
            }

          // Mark it and add to list.
          pNextPolyE->Mark(eMarkType);
          rPolyLoopEdges.Add( pNextPolyE );

#ifdef SM_DEBUG_CODE
          if (bDebugMe)
            {
              smgfx_Erase();
              for (ULONG kkk=0; kkk<rPolyLoopEdges.GetSize(); kkk++)
                {
                  rPolyLoopEdges[kkk]->Draw();
                  sm_GraphicsLoop();
                }
            }
#endif // SM_DEBUG_CODE
      } // end while: done tracing a loop.

      // The first time through, if lamina edges are longer than
      // neighboring quads, subdivide them to match neighboring quads.

      // locals
      SmTArray<SmPolyEdge*> sNewPolyEdgesFromSubdivision;

      if ( ii==0 )
        {
          for ( ULONG jj=0; jj<rPolyLoopEdges.GetSize(); jj++ )
            {
              SmPolyEdge *pPolyEdge = rPolyLoopEdges[jj];
              ULONG lNumSubdivisions;

              SER( CheckAgainstSurfaceCache( pTessCache,
                                             pPolyEdge->GetStartPoint(),
                                             pPolyEdge->GetEndPoint(),
                                             1.001,
                                             TRUE,
                                             lNumSubdivisions ) );
              if ( lNumSubdivisions > 0 )
                {
                  if ( lNumSubdivisions > 50 ) lNumSubdivisions = 50;
                  SER(SubdivideManifoldEdge(
                      pPolyEdge, lNumSubdivisions, NULL, FALSE, sNewPolyEdgesFromSubdivision));
                }
            } // end loop on all edges of loop checking for subdivisions
        }
    } // end two-time trace loop.

#ifdef SM_DEBUG_CODE
  SmBoolean bDebugMe3 = FALSE;
  if (bDebugMe3)
    {
      for (ULONG di=0; di<rPolyLoopEdges.GetSize(); di++)
        {
          if (di==0) smgfx_Erase();
          smgfx_ChangeColor(di != 0);
          rPolyLoopEdges[di]->Draw();
          sm_GraphicsLoop();
        }
    }
#endif // SM_DEBUG_CODE

  // all done
  return SM_SUCCESS;

} // end SmTess::TraceLaminaLoop

/*******************************************************************//**
PURPOSE: assign values to pAux->m_CornerNode for all tree nodes

NOTES: Graphisoft
***********************************************************************/
void  InitializeTreeCorners
 (SmTree* pTree)
{
  SmTreeNode     *pTop    = pTree->GetTopNode();
  SM_PTR_ARRAY(sStack, SmTreeNode, 100) ;  // SmTArray<SmTreeNode *>

  // init the stack with Tree->TopNode
  sStack.Add(pTop);

  SmBoolean childrenProcessed = FALSE;

  // while nodes are on the stack
  while (sStack.GetSize() > 0)
    {
      SmTreeNode *pNode = sStack.GetLast();
      sStack.RemoveLast();

      if (pNode == NULL)
        {
          childrenProcessed = TRUE;
          continue;
        }

      SmBezierAux2d *pAux = (SmBezierAux2d*)pNode->m_pData;

      // Put leaf nodes in output or children on the stack
      if (pNode->m_pChild1 != NULL) // non-leaf node
        {
          SM_ASSERT (pNode->m_pChild2 != NULL);
          if (childrenProcessed)
            {
              SmBezierAux2d *pAuxC1 = (SmBezierAux2d*)pNode->m_pChild1->m_pData;
              SmBezierAux2d *pAuxC2 = (SmBezierAux2d*)pNode->m_pChild2->m_pData;
              pAux->m_hasInsidePart = pAuxC1->m_hasInsidePart || pAuxC2->m_hasInsidePart;

              if (pAux->m_eSplitDir == SM_SP_U)
                {
                  pAux->m_CornerNode [0] = pAuxC1->m_CornerNode [0];
                  pAux->m_CornerNode [1] = pAuxC2->m_CornerNode [1];
                  pAux->m_CornerNode [2] = pAuxC2->m_CornerNode [2];
                  pAux->m_CornerNode [3] = pAuxC1->m_CornerNode [3];
                  SM_ASSERT (pAuxC1->GetUVDomain ().GetUMax () == pAuxC2->GetUVDomain ().GetUMin ());
                  SM_ASSERT (pAuxC1->GetUVDomain ().GetVMin () == pAuxC2->GetUVDomain ().GetVMin ());
                  SM_ASSERT (pAuxC1->GetUVDomain ().GetVMax () == pAuxC2->GetUVDomain ().GetVMax ());
                }
              else
                {
                  pAux->m_CornerNode [0] = pAuxC1->m_CornerNode [0];
                  pAux->m_CornerNode [1] = pAuxC1->m_CornerNode [1];
                  pAux->m_CornerNode [2] = pAuxC2->m_CornerNode [2];
                  pAux->m_CornerNode [3] = pAuxC2->m_CornerNode [3];
                  SM_ASSERT (pAuxC1->GetUVDomain ().GetUMin () == pAuxC2->GetUVDomain ().GetUMin ());
                  SM_ASSERT (pAuxC1->GetUVDomain ().GetUMax () == pAuxC2->GetUVDomain ().GetUMax ());
                  SM_ASSERT (pAuxC1->GetUVDomain ().GetVMax () == pAuxC2->GetUVDomain ().GetVMin ());
                }
            }
          else
            {
              sStack.Add (pNode);
              sStack.Add (NULL);
              sStack.Add (pNode->m_pChild2);
              sStack.Add (pNode->m_pChild1);

              pAux->m_hasInsidePart = FALSE;
            }
        }
      else // Leaf node
        {
          pAux->m_hasInsidePart = pAux->m_eNodeClass == SM_NC_INSIDE;
          pAux->m_CornerNode [0] = pAux->m_CornerNode [1] = pAux->m_CornerNode [2] = pAux->m_CornerNode [3] = pNode;
        }

      childrenProcessed = FALSE;
    } // end while nodes are in stack

} // end Graphisoft InitializeTreeCorners

/*******************************************************************//**
PURPOSE: Construct quads for the interior surface subdivisiopn nodes of a face. This
is called recursively by CreatePolygonsFromInsidePartsOfTree.

NOTES: Given a tree node, the UV boundary of which is already approximated
by a loop of 4 PolyEdges, this method constructs a bisecting PolyEdge to approximate
the boundaries of the children of the node.

 * When necessary, the method will split
   one or two of the parent's bounding PolyEdges.
 * Skips nodes with no children or which do not have a child which is interior
   to the boundary of the face.
***********************************************************************/
static SmBoolean PrepareTreeNodeData
 (SmTreeNode       * pNode,                           // in : Node for which a bisecting
                                                      //      PolyEdge is to be constructed
  SmPolyEdge       * const cornerEdge[4],             // in : Loop of bounding PolyEdges for UV domain of pNode.
  double             dTolerance,                      // NotUsed: in :
  SmFace           * pOriginalFace,                   // in : IwFace corresponding to the surface for this IwTreeNode -> IwTree.
  SmPolyEdge       * cornerEdgeC1[4],                 // out: Bounding loop of PolyEdges for one child node.
  SmPolyEdge       * cornerEdgeC2[4],                 // out: Bounding loop of PolyEdges for other child node.
  SmPolyEdge       * minMaxSplitEdgeRadialPair [2])   // out: Newly constructed PolyEdge and radial mate for use in bisecting UV domain of
                                                      //      pNode.
{

  SM_REF1(dTolerance) ;

  // Get aux data for node.
  SmBezierAux2d* pAux = (SmBezierAux2d*)pNode->m_pData;

  // We are only interested in building PolyEdges to bound children for nodes which have a child
  // on the interior of the face. If that is not this node, leave.
  if (!pAux->m_hasInsidePart)
    {
      cornerEdge[0]->GetPolyBrep ()->DeletePolyFace (cornerEdge[0]->GetPolyFace ());
      return FALSE;
    }
  else
    {
      if (pNode->m_pChild1 == NULL)
        {
          // nothing to do, keep the polygon
          return FALSE;
        }
    }


  // initialize output
  minMaxSplitEdgeRadialPair [0] = NULL;
  minMaxSplitEdgeRadialPair [1] = NULL;

  const SmBoolean dirU = pAux->m_eSplitDir == SM_SP_U;
  // get split direction for the children of this node


  double SmPoint3d::*valueInDir = dirU ? &SmPoint3d::x : &SmPoint3d::y;
  // function pointer for relavent split direction


  SmExtent1d (SmExtent2d::*GetIntervalInDir) (void) const = dirU ? &SmExtent2d::GetUInterval : &SmExtent2d::GetVInterval;
  // function pointer for getting interval to which the new edge will be orthogonal.

  const SmExtent1d intervalInDir = (pAux->GetUVDomain ().*GetIntervalInDir) ();
  // get the interval, this will contain starting PolyVertex for new PolyEdge.

  const double splitValue = (((SmBezierAux2d*)pNode->m_pChild1->m_pData)->GetUVDomain ().*GetIntervalInDir) ().GetMax ();
  // natural split location is the boundary between children on above interval. get it.

  SM_ASSERT (splitValue > intervalInDir.GetMin () && splitValue < intervalInDir.GetMax ());
  // we are going to allow snapping to existing PolyVertices, these values will be used in bounding the snap
  const double childLength = (((SmBezierAux2d*)pNode->m_pChild1->m_pData)->GetUVDomain().*GetIntervalInDir)().GetLength();
  const double childLengthQuarterSQ = (childLength * childLength) * (0.0625);

  // split location has to be on the interval
  SM_ASSERT (splitValue > intervalInDir.GetMin () && splitValue < intervalInDir.GetMax ());


  SmPolyEdge * minMaxEdge[2]     = {NULL, NULL}; // split point on bottom/top or left/right edges
  SmPolyEdge * minMaxPrevEdge[2] = {NULL, NULL}; // previous point on bottom/top or left/right edges
  // we are going to look for IwPolyEdge -> GetStartPoints() to which we will snap the split.
  // minMaxEdge will hold our candidates.

  // Only checking on the opposing sides, relative to the interval orthogonal
  // to split direction for children. Will use startCorner to start in the appropriate corner,
  // and to jump to the opposite side.
  ULONG startCorner = dirU ? 0 : 1;
  for (ULONG minMax = 0; minMax < 2; ++minMax, startCorner += 2)
    {
      const ULONG nextCorner = (startCorner + 1) % 4;

      if (minMax == 0)
        {
          SM_ASSERT (cornerEdge [startCorner]->GetStartPoint ().*valueInDir < splitValue);
          SM_ASSERT (cornerEdge [nextCorner ]->GetStartPoint ().*valueInDir > splitValue);
        }
      else
        {
          SM_ASSERT (cornerEdge [nextCorner ]->GetStartPoint ().*valueInDir < splitValue);
          SM_ASSERT (cornerEdge [startCorner]->GetStartPoint ().*valueInDir > splitValue);
        }

      minMaxPrevEdge [minMax] = cornerEdge [startCorner];

      double minDistSQ = 0;
      // find the closest vertex to splitValue in the in the edge chain
      for (SmPolyEdge* edge = cornerEdge [startCorner]->GetCCWPolyEdge (); edge != cornerEdge [nextCorner]; edge = edge->GetCCWPolyEdge ())
        {
          const double val = edge->GetStartPoint ().*valueInDir;

          if (minMax == 0) { if (val < splitValue) { minMaxPrevEdge [minMax] = edge; } }
          else             { if (val > splitValue) { minMaxPrevEdge [minMax] = edge; } }

          // we throw away candidate split locations (aka PolyVertex which is start of PolyEdge)
          // if they don't exist (NULL),  if they are not closer than another candidate, or
          // if they are farther than 1/4 of the width of the child from boundary between children.
          const double distSQ = (val - splitValue) * (val - splitValue);
          if ((minMaxEdge[minMax] == NULL || minDistSQ > distSQ) && (distSQ < childLengthQuarterSQ))
            {
              minMaxEdge [minMax] = edge;
              minDistSQ = distSQ;
            }
        }

      // if not within tolerance don't use the found vertex. Update tol for [b675]
      SmPoint3d point;
      if (dirU) { point =   SmPoint3d (splitValue,
      // fill in the rest of the split location, given splitValue.
                                       minMax == 0 ? pAux->GetUVDomain ().GetVMin ()
                                                   : pAux->GetUVDomain ().GetVMax (),
                                       0.0);
                }
      else      { point =   SmPoint3d (minMax == 0 ? pAux->GetUVDomain ().GetUMax ()
                                                   : pAux->GetUVDomain ().GetUMin (),
                                       splitValue,
                                       0.0);
                }

      //if ( minMaxEdge[minMax] ) // Found a Vertex to use
      //{
      //    IwXSectTol3d sStartXSectTol = IwTol::GetXSectTol3d( minMaxEdge[minMax], minMaxEdge[minMax]->GetStartPolyVertex() );
      //    IwXSectTol3d sEndXSectTol = IwTol::GetXSectTol3d( minMaxEdge[minMax], minMaxEdge[minMax]->GetEndPolyVertex() );
      //    if(    !( ( minMaxEdge[minMax]->GetStartPoint() - point ).Length() < sStartXSectTol )//     not within tol of PolyEdge start
      //        && !( ( minMaxEdge[minMax]->GetEndPoint()   - point ).Length() < sEndXSectTol ) )// And not within tol of PolyEdge End
      //      { minMaxEdge[minMax] = NULL; }
      //}

      // we found a good IwPolyEdge -> GetStartPoint() to snap to, make sure this doesn't move us too close to a neighbor.
      if (minMaxEdge [minMax] != NULL)
        {

          const double val = minMaxEdge [minMax]->GetStartPoint ().*valueInDir;

          // We want to use "val" instead of "splitVal".
          // Ensure that the introduced inaccuracy won't move vertices too close to their neighbors: No vertex can move closer to its neighbor than 1/4th of the original distance.
          // (This also avoids reversing the order of the subdivision vertices.)

          SmBezierAux2d* pAuxC1;
          SmBezierAux2d* pAuxC2;

          if(minMax == 0) { pAuxC1 = ((SmBezierAux2d*)((SmBezierAux2d*)pNode->m_pChild1->m_pData)->m_CornerNode[nextCorner] ->m_pData);
                            pAuxC2 = ((SmBezierAux2d*)((SmBezierAux2d*)pNode->m_pChild2->m_pData)->m_CornerNode[startCorner]->m_pData);
                          }
          else            { pAuxC1 = ((SmBezierAux2d*)((SmBezierAux2d*)pNode->m_pChild1->m_pData)->m_CornerNode[startCorner]->m_pData);
                            pAuxC2 = ((SmBezierAux2d*)((SmBezierAux2d*)pNode->m_pChild2->m_pData)->m_CornerNode[nextCorner] ->m_pData);
                          }

          const double prevSplit = (pAuxC1->GetUVDomain ().*GetIntervalInDir) ().GetMin ();
          const double nextSplit = (pAuxC2->GetUVDomain ().*GetIntervalInDir) ().GetMax ();

          // prevSplit and nextSplit are the closest values to splitValue where further splits might be added later

          if (val < (prevSplit + 3.0 * splitValue) / 4.0 || (nextSplit + 3.0 * splitValue) / 4.0 < val)
            {
              // don't use a vertex that is closer to another split
              minMaxEdge [minMax] = NULL;
            }
        }

      // we are going to split.
      if (minMaxEdge [minMax] == NULL)
        {
          // vertex not found -> split edge
          SmPolyVertex* newVertex = NULL;
          minMaxPrevEdge[minMax]->GetPolyFace()->MakeVertexSplitPolyEdge(minMaxPrevEdge[minMax], // in : PolyEdge to split
                                                                         point,                  // in : Point split location
                                                                         minMaxEdge[minMax],     // out: new PolyEdge (and new radial partners)
                                                                         newVertex);             // out: new PolyVertex
          //
          if (!minMaxPrevEdge [minMax]->IsLamina ())
            {
              minMaxSplitEdgeRadialPair [minMax] = minMaxPrevEdge [minMax]->GetRadial ();
              SM_ASSERT (minMaxSplitEdgeRadialPair [minMax]->GetStartPolyVertex () == newVertex);
            }

          SM_ASSERT (newVertex != NULL);
        }
    }

  // build bisecting PolyEdge and radial mate.

  //JLMCC: as far as I can tell, the below code is doing nothing.
  //SmTArray<SmPolyEdge*> sEdges0;
  //minMaxEdge[0]->GetStartPolyVertex()->GetPolyEdges(sEdges0);
  //SmTArray<SmPolyEdge*> sEdges1;
  //minMaxEdge[1]->GetStartPolyVertex()->GetPolyEdges(sEdges1);
  //SmTArray<SmPolyEdge*> sEdgesCommon;
  //sEdges0.FindCommonElements(sEdges1, sEdgesCommon);

  SmPolyEdge * newEdge = NULL;
  SmPolyLoop * newLoop = NULL;
  SmPolyFace * newFace = NULL;

  minMaxEdge[0]->GetPolyFace()->MakeManifoldEdge(minMaxEdge [0],
                                                 minMaxEdge [1],
                                                 newEdge,
                                                 newLoop,
                                                 newFace);
  newFace->SetOriginalFace (pOriginalFace, TRUE) ; // TRUE = copy pOriginalFace->Attributes onto newFace
  SM_ASSERT (newEdge != NULL);
  SM_ASSERT (newEdge->GetRadial () != NULL);
  SM_ASSERT (newEdge->GetRadial ()->GetRadial () == newEdge);
  SM_ASSERT (newLoop != NULL);
  SM_ASSERT (newFace != NULL);

  // fill in PolyEdges for loops around children.
  if (dirU)
    {
      cornerEdgeC1[0] = cornerEdge [0];
      cornerEdgeC1[1] = newEdge;
      cornerEdgeC1[2] = minMaxEdge [1];
      cornerEdgeC1[3] = cornerEdge [3];

      cornerEdgeC2[0] = minMaxEdge [0];
      cornerEdgeC2[1] = cornerEdge [1];
      cornerEdgeC2[2] = cornerEdge [2];
      cornerEdgeC2[3] = newEdge->GetRadial ();
    }
  else
    {
      cornerEdgeC1[0] = cornerEdge [0];
      cornerEdgeC1[1] = cornerEdge [1];
      cornerEdgeC1[2] = newEdge;
      cornerEdgeC1[3] = minMaxEdge [1];

      cornerEdgeC2[0] = newEdge->GetRadial ();
      cornerEdgeC2[1] = minMaxEdge [0];
      cornerEdgeC2[2] = cornerEdge [2];
      cornerEdgeC2[3] = cornerEdge [3];
    }

  return TRUE;

} // end Graphisoft PrepareTreeNodeData

/*******************************************************************//**
PURPOSE:

NOTES: Graphisoft
***********************************************************************/
static void RemoveUnneccessaryTopologicalVertices
 (SmPolyEdge * const minMaxSplitEdgeRadialPair [2])
{
    for (ULONG minMax = 0; minMax < 2; ++minMax)
      {
        if(   minMaxSplitEdgeRadialPair[minMax] != NULL
           && minMaxSplitEdgeRadialPair[minMax]->IsLamina ()
           && minMaxSplitEdgeRadialPair[minMax]->GetCWPolyEdge ()->IsLamina ())
        {
            // both polygons at the new split vertex have been deleted -> the topological vertex between them can be removed
            SM_PTR_ARRAY(removedEdges, SmPolyEdge, 2);
            minMaxSplitEdgeRadialPair[minMax]->GetPolyBrep ()->DeleteTopologicalVertex (minMaxSplitEdgeRadialPair[minMax]->GetStartPolyVertex (), removedEdges);
            SM_ASSERT (removedEdges.GetSize () == 1 && removedEdges [0] == minMaxSplitEdgeRadialPair[minMax]);
        }
    }

} // end Graphisoft RemoveUnneccessaryTopologicalVertices

/*******************************************************************//**
PURPOSE:

NOTES: Graphisoft
***********************************************************************/
#if defined (PROCESS_TREE_RECURSIVE)
static void ProcessTreeNodeRecursive
 (SmTreeNode * pNode,
  SmPolyEdge * const cornerEdge[4],
  double       dTolerance,
  SmFace     * pOriginalFace)
{
    SmPolyEdge* cornerEdgeC1 [4];
    SmPolyEdge* cornerEdgeC2 [4];
    SmPolyEdge* minMaxSplitEdgeRadialPair [2];
    if (PrepareTreeNodeData (pNode,
                             cornerEdge,
                             dTolerance,
                             pOriginalFace,
                             cornerEdgeC1,
                             cornerEdgeC2,
                             minMaxSplitEdgeRadialPair))
      {
        ProcessTreeNodeRecursive (pNode->m_pChild1, cornerEdgeC1, dTolerance, pOriginalFace);
        ProcessTreeNodeRecursive (pNode->m_pChild2, cornerEdgeC2, dTolerance, pOriginalFace);

        RemoveUnneccessaryTopologicalVertices (minMaxSplitEdgeRadialPair);
      }

} // end Graphisoft ProcessTreeNodeRecursive
#endif // if defined (PROCESS_TREE_RECURSIVE)

/*******************************************************************//**
PURPOSE:

NOTES: Graphisoft
***********************************************************************/
static void ProcessTree
 (SmTreeNode                  * pNode,
  const SmTArray<SmPolyEdge*> & polyEdges,
  double                        dTolerance,
  SmFace                      * pOriginalFace)
{

  SM_OBJ_ARRAY (sStack, StackData, 16);

  sStack.Push (StackData (pNode, polyEdges.GetDataArray ()));

  while (sStack.GetSize () > 0)
    {
      const StackData stackData = sStack.GetLast();
      sStack.RemoveLast();

      if (stackData.m_pNode != NULL)
        {
          SmPolyEdge* cornerEdgeC1 [4];
          SmPolyEdge* cornerEdgeC2 [4];
          SmPolyEdge* minMaxSplitEdgeRadialPair [2];

          if (PrepareTreeNodeData (stackData.m_pNode,
                                   stackData.m_cornerEdge,
                                   dTolerance,
                                   pOriginalFace,
                                   cornerEdgeC1,
                                   cornerEdgeC2,
                                   minMaxSplitEdgeRadialPair))
            {
              sStack.Push (StackData (minMaxSplitEdgeRadialPair));
              sStack.Push (StackData (stackData.m_pNode->m_pChild2, cornerEdgeC2));
              sStack.Push (StackData (stackData.m_pNode->m_pChild1, cornerEdgeC1));
            }
        }
      else
        {
          RemoveUnneccessaryTopologicalVertices (stackData.m_minMaxSplitEdgeRadialPair);
        }
    } // end while

} // end Graphisoft ProcessTree

/*******************************************************************//**
PURPOSE:

NOTES: Graphisoft
***********************************************************************/
static void CreatePolygonsFromInsidePartsOfTree
 (SmTree     * pTree,
  SmPolyBrep * pPolyBrep,
  double       dTolerance,
  SmFace     * pOriginalFace)
{
  InitializeTreeCorners (pTree);

  SmTreeNode    * pTop = pTree->GetTopNode();
  if (pTop == NULL) { return; }
  SmBezierAux2d * pAux = (SmBezierAux2d*)pTop->m_pData;

  SmAxis2Placement sPosition;
  sPosition.Translate(pAux->GetUVDomain ().GetMin());
  SmVector2d sUVSize = pAux->GetUVDomain ().GetSize();

  SmPolyFace    * newFace;

  // Add 4 PolyEdges and associated PolyVertices to pPolyBrep
  SM_PTR_ARRAY(polyEdges, SmPolyEdge, 4);
  pPolyBrep->CreateRectangle(pPolyBrep->GetTolerance(),
                             sUVSize.x,
                             sUVSize.y,
                             sPosition,
                             newFace,
                             & polyEdges);

  newFace->SetOriginalFace (pOriginalFace, TRUE) ; // TRUE = copy pOriginalFace->Attributes onto newFace

#if defined (PROCESS_TREE_RECURSIVE)
  // for simpler debugging
  ProcessTreeNodeRecursive (pTop, polyEdges.GetDataArray (), dTolerance, pOriginalFace);
#else
  ProcessTree (pTop, polyEdges, dTolerance, pOriginalFace);
#endif
} // end Graphisoft CreatePolygonsFromInsidePartsOfTree

/*******************************************************************//**
PURPOSE: Triangulate 1 many sided pFace into many 3 sided PolyFaces by repeated subdivision.

NOTES: 1. add PolyEdges to outline and divide all 'INSIDE' Surface Subdivision quads.
       2. add PolyEdges to triangulate between 'INSIDE' and boundary PolyEdges.
 - Add new SmPolyEdges between pSC->mTS_pPolyBrep existing PolyVertices.
 - If needed, Subdivided new SmPolyEdges to meet tessellation parameters.
***********************************************************************/
SmStatus SmTess::AddQuadBoundaries
 (SmFace *pFace)
{
  // locals
  SmTessSrfCache * pSC             = m_vCache.GetSecond(pFace); NER(pSC);
  SmPolyBrep     * pPolyBrep       = pSC->mTS_pPolyBrep; NER(pPolyBrep);
  SmPolyFace     * pOuterPolyFace  = pPolyBrep->GetFirstPolyFace();
  SmPolyShell    * pOuterPolyShell = (SmPolyShell *)pOuterPolyFace->GetPolyShell() ;

  // local data arrays
  SM_PTR_ARRAY(sPolyEdges,      SmPolyEdge, 256) ;  // SmTArray<SmPolyEdge *>
  SM_PTR_ARRAY(sInnerPolyLoops, SmPolyLoop,  32) ;  // SmTArray<SmPolyLoop *>
  SM_PTR_ARRAY(sNodes,          SmTreeNode, 256) ;  // SmTArray<SmTreeNode *>
  SM_PTR_ARRAY(sOuterPolyFaces, SmPolyFace,  32) ;  // SmTArray<SmPolyFace *>
  SM_PTR_ARRAY(sPolyLoopEdges,  SmPolyEdge, 256) ;  // SmTArray<SmPolyEdge *>
  SM_PTR_ARRAY(sPolyFaces,      SmPolyFace, 256) ;  // SmTArray<SmPolyFace *>
  SM_PTR_ARRAY(sNewPolyFaces,   SmPolyFace, 256) ;  // SmTArray<SmPolyFace *>

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
std::atomic<ULONG> lCount(1) ; lCount++ ;
std::atomic<ULONG> lDebugCount(0);
  if (bDebugMe || lCount == lDebugCount)
    {
      SmBoolean bOK_pSC = pSC->AssertValid() ;                       if(!bOK_pSC)            { SM_ASSERT_VALID(pSC) ;            }
      SmBoolean bOK_pPolyBrep = pPolyBrep->AssertValid() ;           if(!bOK_pPolyBrep)      { SM_ASSERT_VALID(pPolyBrep) ;      }
      SmBoolean bOK_pOuterPolyFace = pOuterPolyFace->AssertValid() ; if(!bOK_pOuterPolyFace) { SM_ASSERT_VALID(pOuterPolyFace) ; }
      pSC->Dump() ;
      pPolyBrep->Dump() ;
      pOuterPolyFace->Dump() ;

      smgfx_Erase();
      smgfx_SetLook(2,1, 1,0,0); pFace->Draw(SM_DM_WIREFRAME); sm_GraphicsLoop();
      smgfx_SetLook(1,1, 0,0,0); pFace->GetSurface()->DrawUV(5,5); sm_GraphicsLoop();
      smgfx_SetLook(1,3, .4,.4,.4); pSC->DrawSubdivision3D(); sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

  // Create polygon mesh from the m_pTessBrep->face->Surface->SmTessSrfCache in and on nodes

  // Fetch all PolyLoops in pOuterPolyFace - expect only 1 for each pFace->Loop at this time
  pOuterPolyFace->GetPolyLoops(sInnerPolyLoops);  // temporarily sInnerPolyLoops[0] is outer loop - will be removed later
  if (sInnerPolyLoops.GetSize() == 0)
    { return SM_SUCCESS; }

#ifdef SM_DEBUG_CODE
  if (bDebugMe)
    {
      smgfx_Erase();
      for (ULONG kk=0; kk<sInnerPolyLoops.GetSize(); kk++)
        {
          SmPolyLoop *pInnerLoop     = sInnerPolyLoops[kk];
          SmPolyEdge *pInnerLoopEdge = pInnerLoop->GetFirstPolyEdge();
          SmPoint3d   sLoopPoint     = pInnerLoopEdge->GetStartPoint();

          smgfx_SetLook(1,4, 0,0,0); pInnerLoop->Draw( TRUE, FALSE );     sm_GraphicsLoop();
          smgfx_SetLook(1,4, 0,0,0); pInnerLoop->Draw( TRUE, TRUE  );     sm_GraphicsLoop();
          smgfx_SetLook(2,4, 1,0,0); pInnerLoopEdge->Draw( TRUE, FALSE ); sm_GraphicsLoop();
          smgfx_SetLook(2,4, 1,0,0); pInnerLoopEdge->Draw( TRUE, TRUE  ); sm_GraphicsLoop();
          smgfx_SetLook(1,3, .4,.4,.4); pSC->DrawSubdivision2D(FALSE) ;   sm_GraphicsLoop() ;
          sm_GraphicsLoop();
        }
    }
#endif // SM_DEBUG_CODE

  // find all SmTessSrfCache TreeNodes of type SM_NC_INSIDE the pFace->UVDomain
  SmNodeClassType eNodeClass = SM_NC_INSIDE;
  SER( pSC->FindUVNodesOfClass( pFace->GetUVDomain(), &eNodeClass, sNodes ));

  // when not using AdvancingFront
  if ( !m_bAdvancingFront )
    {
       CreatePolygonsFromInsidePartsOfTree (pSC->GetTree (), pPolyBrep, pPolyBrep->GetTolerance (), pFace);
    }

  // obsolete
  //  // when not using AdvancingFront
  //  if ( !m_bAdvancingFront )
  //    {
  //      // For every node that's INSIDE, create an SmPolyFace in the Brep that
  //      // goes counterclockwise around the node.  So we'll have a gridded
  //      // uv domain, with INSIDE grids having a face, others empty.
  //      SmPolyFace  * pPolyFace  = NULL ;
  //
  //      // SmExtent2d   sUVDomain  = pFace->GetUVDomain();
  //      // double       dSize      = sUVDomain.GetSize().Length();
  //      // sUVDomain.ExpandAbsolute( dSize / 100.0 );
  //      // SmPoint3d    sMin( sUVDomain.GetMin().x, sUVDomain.GetMin().y, -dSize );
  //      // SmPoint3d    sMax( sUVDomain.GetMax().x, sUVDomain.GetMax().y, dSize/2.345 );
  //      // SmExtent3d   sBBox( sUVDomain.GetMin(), sUVDomain.GetMax() );
  //      // ULONG        lTreeInitialize = smos_Max( sNodes.GetSize()/10, 100 );
  //      // SmTree     * pVertTree = new (m_crContext) SmTree(sBBox, lTreeInitialize, lTreeInitialize/10 );
  //      // SmObjDelete  sClean(pVertTree);
  //
  //      // clean any old TreeVertex <-> PolyVertex maps
  //      pSC->GetTree()->ClearAllPolyVertices() ;
  //
  //      // Clear node marks
  //      pSC->GetTree()->ClearNodeMarks() ;
  //
  //      // pick a 3d Tolerance from the input pFace
  //      double dDistTol3d = pFace->GetTolerance() ;
  //
  //      // Add a PolyEdge bounded PolyFace to PolyBrep for each pFace->Surface->Subdivision inside Node
  //      for(ULONG ii=0; ii<sNodes.GetSize(); ii++ )
  //        {
  //          SmTreeNode    * pNode = sNodes[ii];
  //          SmBezierAux2d * pAux  = (SmBezierAux2d*)pNode->m_pData;
  //
  //          // skip already processed Nodes
  //          if(pAux->m_bIsMarked == TRUE)
  //            { continue ; }
  //
  //          // when nodes are too small
  //          if(pNode->GetBoundingBox().GetMaxDimension() < dDistTol3d * 10.0)
  //            {
  //              // locals
  //              SmTreeNode    *pParent    = pNode->m_pParent ;
  //              SmBezierAux2d *pParentAux = (SmBezierAux2d*)pParent->m_pData ;
  //
  //              // init to skip this node
  //              pNode = NULL ;
  //              pAux  = NULL ;
  //
  //              // while there is an unmarked ancestor
  //              while(pParent && pParentAux->m_bIsMarked == FALSE)
  //                {
  //                  // and the ancestor is in the face
  //                  if( ((SmBezierAux2d*)pParent->m_pData)->m_eNodeClass == SM_NC_INSIDE)
  //                    {
  //                      // and the ancestor is large enough
  //                      if(pParent->GetBoundingBox().GetMaxDimension() >= dDistTol3d * 10.0)
  //                        {
  //                          // Mark the parent and its descendants
  //                          ((SmBezierAux2d*)pParent->m_pData)->m_bIsMarked = TRUE ;
  //                          pParent->PropagateToChildren() ;
  //
  //                          // Make a quad of the ancestor
  //                          pNode = pParent ;
  //                          pAux  = pParentAux ;
  //
  //                          // done with search
  //                          pParent    = NULL ;
  //                          pParentAux = NULL ;
  //                        }
  //                      else
  //                        {
  //                          // try the grandparent
  //                          pParent    = pParent->m_pParent ;
  //                          pParentAux = pParent ? (SmBezierAux2d*)pParent->m_pData : NULL ;
  //                        }
  //                    }
  //                  else // ancestor is not in the face - quit the search
  //                    {
  //                      pParent    = NULL ;
  //                      pParentAux = NULL ;
  //                    }
  //                } // end while looking for a large enough ancestor
  //            } // end check to replace small nodes with larger ones
  //
#ifdef SM_DEBUG_CODE
  //          if(pNode->GetBoundingBox().GetMaxDimension() > dDistTol3d * 10.0)
  //            {
  //              // gwc: this is being hit. In the one case I've investigated so far,
  //              //      a PolyLoop is being built with
  //              //      SmPolyLoop::AddPolyEdge
  //              //        SmPolyBrep::StitchEdgeTopological
  //              //          SmPolyBrep::GlueEdges
  //              //            SmPolyBrep::OrderRadialEdges
  //              //              SmPolyFace::GetNormal
  //              //      we get here because 1. Added edge is coincident with an existing edge - which calls Stitch and then Glue
  //              //                          2. Glue finds out that more than two PolyEdges are being glued and calls Order.
  //              //                          3. Order calls GetNormal on a PolyFace with a partially completed Loop - thus the error.
  //              //       fixes? The caller of AddPolyEdge knows all the edges to be used to build the new PolyFace - that
  //              //                caller could figure out this normal.  How do I get that normal all the way down to this level?
  //              //                The AddPolyEdge is a user interface method - so don't add a new argument value to it
  //              //                Adding and setting a member value somewhere would require user level code to be rewritten.
  //              //       Fix: All the fixes have to impact the way a user uses the system.  Since Spline edges are rare
  //              //            this won't commonly come up. I'm going to add an optional Normal vector argument to the SmPolyFace
  //              //            constructor.  To avoid this bug, users should specify the FaceNormal through that constructor.
  //              //      However - this is supposed to be a manifold model!  So the initial error that sets this up
  //              //                is that coincident edges are being built and then stitched together into spline edges rather than
  //              //                manifold edges.  I don't know where that's happening yet.
  //              WARN(_T(" SmTess::AddQuadBoundaries: Found an inside Node too small (tol=pFace->GetTolerance() * 10.0) for a SmPolyBrep")) ;
  //            }
#endif // SM_DEBUG_CODE
  //
  //          // add 4 sided SmPolyFace for large enough Node to pPolyBrep
  //          if(pNode)
  //            {
  //              // gwc: I added this next size check but it may have a problem
  //              //      It seems possible to skip an inside node surrounded by other
  //              //      usable inside nodes to create a hole in the final set of
  //              //      polyFaces covering this face.  However, above
  //              //      I combined small inside nodes with their siblings and ancestors
  //              //      until some node ancestor is large enough to pass this test.
  //              //      The only small nodes that make it here should either be
  //              //      the root node for the whole tree, or attached to a sibling
  //              //      that is not inside.  Therefore it should never happen that
  //              //      we skip a small node comppletely surrounded by valid larger nodes.
  //              if(pNode->GetBoundingBox().GetMaxDimension() > dDistTol3d * 10.0)
  //              SER( sm_MakePolygonFromNode( pPolyBrep,
  //                                           pNode,
  //                                           pOuterPolyShell,
  //                                           pPolyFace));
  //              // remember the pFace <-> pPolyFace relationship
  //              pPolyFace->m_pFace                        = pFace;
  //              ((SmBezierAux2d*)pNode->m_pData)->m_pFace = pPolyFace;
  //
  //            } // end Node was large enough to add check
  //
  //        } // end iter every INSIDE TreeNode
  //    } // end Not using Advancing front check

  // GWC: removed need for Stitch Call - sm_MakePolygonFromNode now builds connected PolyBreps
  //
  //  // Stitch those polygons together.  This will make all interior edges
  //  // nonlamina, and leave all boundary edges lamina.
  //  ULONG lNumStitched, lNumLamina;
  //  SER( pPolyBrep->Stitch( pPolyBrep->GetTolerance(),
  //                       TRUE,
  //                       TRUE,
  //                       TRUE,
  //                       lNumStitched,
  //                       lNumLamina ));
  //

#ifdef SM_DEBUG_CODE
  SmBoolean bDebugMe3 = FALSE;
  if (bDebugMe3)
    {
      SM_ASSERT_VALID(pSC) ;
      SM_ASSERT_VALID(pPolyBrep) ;
      SM_ASSERT_VALID(pSC->mTS_pPolyBrep) ;

      if ( FALSE )
        { smgfx_Erase(); }
      smgfx_SetLook(1,2, 1,0,0); pPolyBrep->GetFirstPolyFace()->Draw( TRUE, FALSE );      sm_GraphicsLoop();
      smgfx_SetLook(1,2, 1,0,0); pPolyBrep->GetFirstPolyFace()->Draw( TRUE, TRUE  );      sm_GraphicsLoop();
      smgfx_SetLook(1,2, 1,0,0); pPolyBrep->GetFirstPolyFace()->DrawDebug( TRUE, FALSE ); sm_GraphicsLoop();
      smgfx_SetLook(1,2, 1,0,0); pPolyBrep->GetFirstPolyFace()->DrawDebug( TRUE, TRUE  ); sm_GraphicsLoop();

      smgfx_SetLook(2,3, 1,0,0); if(pFace) pFace->DrawUV(); sm_GraphicsLoop();                 // Face being tessellated
      smgfx_SetLook(2,3, 1,0,0); if(pFace) pFace->DrawUVCurves(FALSE); sm_GraphicsLoop();      // Face being tessellated
      smgfx_SetLook(1,2, 0,0,0); if(m_pTessBrep) m_pTessBrep->Draw(TRUE) ; sm_GraphicsLoop();  // 3d Brep being tessellated
      smgfx_SetLook(1,2, 0,0,0); if(pPolyBrep) pPolyBrep->Draw( TRUE, FALSE ); sm_GraphicsLoop(); // 2d Brep of Face tessellation
      smgfx_SetLook(1,2, 0,0,0); if(pPolyBrep) pPolyBrep->Draw( TRUE, TRUE  ); sm_GraphicsLoop(); // 3d Brep of Face tessellation
      smgfx_SetLook(2,1, 0,1,0); if(pSC) pSC->DrawSubdivision2D(TRUE); sm_GraphicsLoop();      // 2d SrfTessCache of Face being tessellated

      SmTArray<SmPolyLoop*> aLoops;
      SmTArray<SmPolyFace*> aFaces;
      pPolyBrep->GetPolyFaces( aFaces );
      if ( FALSE )
        { smgfx_Erase(); }
      smgfx_SetLook(2,4, 0,0,1);
      for ( ULONG jj=0; jj< aFaces.GetSize(); jj++ )
        {
          SmPolyFace *pPF = aFaces[jj];
          pPF->GetPolyLoops( aLoops );
          for ( ULONG kk=0; kk< aLoops.GetSize(); kk++ )
            {
              (aLoops[kk])->Draw( TRUE, FALSE ); sm_GraphicsLoop();
              (aLoops[kk])->Draw( TRUE, TRUE  ); sm_GraphicsLoop();
            }
          SmTArray<SmPolyEdge*> sEdges;
          pPF->GetPolyEdges( sEdges );
          for (ULONG ijk=0; ijk<sEdges.GetSize(); ijk++) {
               smgfx_SetLook(1,2, 0,0,1); sEdges[ijk]->Draw  (); sm_GraphicsLoop();
               smgfx_SetLook(1,2, 0,0,1); sEdges[ijk]->Draw3D(); sm_GraphicsLoop();
          }
          sm_GraphicsLoop();
        }
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

  /* Arrive here when PolyFaces of pSC->mTS_pPolyBrep looks something like the following:
  
          +-------+      (Every Closed polygon has its own PolyFace and PolyLoop)
         /         \                         
        /   +---+   +
       +    |   |   |
       |    |   |   +--+ <=== outer loop already in pSC->mTS_pPolyBrep
       |    +---+---+   \     built by Phase1SetupBrep, PolyVertices map to m_pTessBrep->Vertices
       +    |   |   |    \                
       |    |   |   |     +--------+
       |    |   |   |              |
       |    +---+---+---+---+---+  |    note: Subdivision Nodes that intersect the outerBoundary
       |    |   |   |   |   |   |  +          of the polygon are not inserted as Quad boundaries
       |    |   +---+   |   |   |  |          leaving a large annular area to be broken into
       |    |   |   |   |   |   |  |          triangles at a later time.
       |    +---+---+---+---+---+  |
       |    |   |   |   |   |      |
       |    |   +---+---+   |      +
       +    |   |   |   |   |<====/=== Inner PolyFaces Added here by AddQuadBoundaries() for each SubdivisionTree->TreeNode
        \   +---+---+---+---+    +       PolyVertices map to SmTree::TreeVertices
         \                       |
          +----------------------+
   */ 

   // note: some cases will have no INSIDE subdivision Tree Nodes in which case
   //       pSC->mTS_pPolyBrep will have only 1 PolyFace containing PolyLoops for
   //       each pFace->Loop at this time.

  // Now create loops corresponding to the lamina boundaries of the INSIDE nodes
  // and create new polygons and loops for the lamina edges.

  // Remove outer loop from sInnerPolyLoops list
  sInnerPolyLoops.RemoveAt(0,1);

  // Remove the inner loops from the pOuterPolyFace Doubly Linked List - they're saved in sInnerPolyLoops list
  for (ULONG ii=0; ii<sInnerPolyLoops.GetSize(); ii++)
    {
      pOuterPolyFace->Remove(sInnerPolyLoops[ii]);
    }

  // build OuterPolyFaces list
  sOuterPolyFaces.Add( pOuterPolyFace );

  // Get PolyBrep->PolyEdges
  pPolyBrep->GetPolyEdges( sPolyEdges );

  // Trace boundaries: loops of lamina edges.
  ULONG lNumEdges = sPolyEdges.GetSize();
  for ( ULONG ii=0; ii<lNumEdges; ii++ )
    {
      SmPolyEdge *pPolyEdge = sPolyEdges[ii];

      // when PolyEdge is an internal face lamina PolyEdge
      if(   pPolyEdge->IsLamina()
         && pPolyEdge->GetPolyFace() != pOuterPolyFace )
        {
          SmPolyLoop *pPolyLoop;

          // walk sequence of lamina edges to find a bounding loop for pPolyFace
          SER( TraceLaminaLoop( pPolyEdge, pSC, sPolyLoopEdges ));  // note: increments an unlocked mark value

          // Build a PolyLoop not yet in a PolyFace
          //   (with new PolyEdges that are RadialPartners to the old PolyEdges)
          SER( BuildLaminaLoop( sPolyLoopEdges, pPolyLoop ));

#ifdef SM_DEBUG_CODE
          SmBoolean bDebugMe1 = FALSE;
          if (bDebugMe1)
            {
              smgfx_SetLook(3,2, 1,0,1); pPolyLoop->Draw(); sm_GraphicsLoop();
            }
#endif // SM_DEBUG_CODE

          // Orientation: since all of the original little square faces
          // went CCW, then outer loops will go CCW and inner loops will be CW.
          SmOrientType eOrient;
          SmVector3d   sNormal( 0,0,1 );
          double       dArea;
          SER( pPolyLoop->ComputeLoopOrientation( sNormal, eOrient, dArea ));
          pPolyLoop->m_eOrientation = eOrient;

          // When NewPolyLoop is CCW - it needs to be the outerLoop of a new PolyFace
          if ( eOrient == SM_OT_SAME )
            {
              // Traced a new outer loop. Create a new Face for it.
              SmVector3d sZ(0,0,1) ;
              SmPolyFace *pNewFace = new (pPolyBrep) SmPolyFace( pPolyBrep->GetTolerance(),
                                                                 pPolyBrep,
                                                                 NULL,
                                                                 pOuterPolyShell,
                                                                 &sZ );

              // set pNewFace->m_pOriginalFace value and propagate attributes if possible
              pNewFace->SetOriginalFace(pOuterPolyFace->m_pOriginalFace, TRUE) ;

              // Insert the New PolyLoop into the New PolyFace
              pNewFace->PostInsert( pPolyLoop );
              sOuterPolyFaces.Add( pNewFace );
            }
          else // traced an inner loop - add PolyLoop to InnerPolyLoops list
            {
              sInnerPolyLoops.Add( pPolyLoop );  // Traced an inner loop. Not yet contained in a PolyFace
            }
        } // end if this edge is Lamina
    } // end loop on all edges

  // Now place all inner loops into correct outer loop.
  // The tricky part is that outer loops could be nested
  // to unknown levels and that inner loops may be inside
  // more than one (nested) outer loop.
  //
  // For each inner loop, check it against all outer loops
  // other than the first one.  (It's already in the first one.)

  ULONG lNumInner = sInnerPolyLoops.GetSize();
  for (ULONG ii=0; ii<lNumInner; ii++)
    {
      SmPolyLoop          * pInnerPolyLoop     = sInnerPolyLoops[ii];
      SmPolyFace          * pContainerPolyFace = NULL;
      SmPolyContainmentType ePCType;

      // All outer loops after the first one:
      ULONG lNumOuter = sOuterPolyFaces.GetSize();
      for (ULONG jj=1; jj<lNumOuter; jj++ )
        {
          SmPolyFace* pPolyFace = sOuterPolyFaces[jj];
          SmPolyLoop *pOuterPolyLoop = pPolyFace->GetOuterPolyLoop();

          // Determine if pInnerPolyLoop is inside pOuterPolyLoop
          SER( pOuterPolyLoop->ContainsPolyLoop( pInnerPolyLoop,
                                                 ePCType,
                                                 pPolyFace->GetTolerance() ) );

          // when pInnerPolyLoop is INSIDE or on the OuterBoundary // gwc: inner loops should not be on the Boundary
          if ( ePCType == SM_PCT_INSIDE || ePCType == SM_PCT_ON_BOUNDARY )
            {
              if ( pContainerPolyFace == NULL )
                {
                  // This is the first containing face that we've found
                  // for this inner loop.
                  pContainerPolyFace = pPolyFace;
                }
              else // this innerLoop is inside more than 1 OuterLoop
                {
                  // We've already found a container face for this inner loop,
                  // so the outer loops must be nested.
                  // Check which is the inside the other.
                  SmPolyLoop *pTestFaceLoop = pContainerPolyFace->GetOuterPolyLoop();

                  SER( pOuterPolyLoop->ContainsPolyLoop( pTestFaceLoop,
                                                         ePCType,
                                                         pPolyFace->GetTolerance() ) );

                  // If current container face is not inside of outer face
                  // then the outer face must be inside - replace container face
                  if ( ePCType != SM_PCT_INSIDE && ePCType != SM_PCT_ON_BOUNDARY )
                    {
                      pContainerPolyFace = pPolyFace;
                    }
                } // end if innerLoop is inside more than 1 outerLoop check
            } // end if inner loop is inside this OuterLoop check
        } // end iter OuterLoops looking for container of current innerLoop

      // when innerLoop is only contained in the outer most Outerloop - pContainerPolyFace == NULL
      if ( pContainerPolyFace == NULL )
        {
          // didn't find an enclosing outer loop - innerLoop is contained by pOuterPolyFace
          pContainerPolyFace = pOuterPolyFace;
        }

      // Add this inner loop to the appropriate face.
      // PostInsert, because the outer loop must be first.
      pContainerPolyFace->PostInsert( pInnerPolyLoop );

    } // end loop on all inner loops

#ifdef SM_DEBUG_CODE
  SmBoolean bDebugMe5 = FALSE;
  if (bDebugMe5)
    {
      for (ULONG ii=0; ii<sOuterPolyFaces.GetSize(); ii++)
        {
          smgfx_Erase();
          sOuterPolyFaces[ii]->Draw( TRUE, FALSE );
          sOuterPolyFaces[ii]->Draw( TRUE, TRUE  );
          sm_GraphicsLoop();
        }
    }
#endif // SM_DEBUG_CODE

  // Arrive here when: PolyFace polygons have been created to cover the entire UV domain of the surface.
  //                   Every PolyLoop is contained in a PolyFace.  All PolyFaces are in the same PolyShell.
  //                   No duplicate PolyVertices and all coincident PolyEdges are radial partners of one another.
  // Next: Triangulate all PolyFaces.
  //
  //       Simple Quad Nodes map      Complex Quad Nodes (more than just corner Polyvertices) map
  //        from +---+ to +---+        from +-+ to +-+
  //             |   |    |\  |             | |    |\|
  //             |   |    | \ |             | +    | +
  //             |   |    |  \|             | |    |/|
  //             +---+    +---+             +-+    +-+
  //
  //       Gap between the outer loop and the inner boundary of the Subdivision Cache node PolyFaces maps
  //  from  +-------+                          to  +-------+
  /*       /         \                            / \ `,  / \                        */
  //      /   +---+   +                          / , +---+---+
  //     +    |   |   |                         +    |   | \ |
  //     |    |   |   +--+                      |\   |   | , +--+
  /*     |    |   +---+   \                     | \  |   +---+.' \                   */
  /*     +    |       |    \                    +  \ |       |\ , \                  */
  //     |    |       |     +--------+          | , \|       | \ ' +--------+
  //     |    +       |              |          |  ' +       |  \ / \ ' , / |
  //     |    |       +---+---+---+  |          |   /|       +---+---+---+ ,|
  //     |    |                   |  +          |  / |                   |  +
  //     |    |                   |  |          | /  |                   | /|
  //     |    |                   |  |          |/   |                   |/ |
  //     +    +               +---+  |          +----+               +---+  |
  //     |    |               |      |          |  / |               |  / \ |
  //     |    |               |      +          | /  |               |      +
  //     +    |               |     /           + ,  |               |/ '  /
  //      \   +---+---+---+---+    +             \ ' +---+---+---+---+----+
  //       \                       |              \ /  /  /   /   /    \  |
  //        +----------------------+               +----------------------+
  //
  //     outer loop already in pSC->mTS_pPolyBrep
  //     built by Phase1SetupBrep, PolyVertices map to m_pTessBrep->Vertices
  //
  //     Boundary of Inner Faces Added here by AddQuadBoundaries() for each SubdivisionTree->TreeNode
  //         PolyVertices map to SmTree::TreeVertices

  // locals
  SmStatus eStat;
  SmStatus eRetStatus = SM_SUCCESS;
  pPolyBrep->GetPolyFaces( sPolyFaces );
  ULONG lNumFaces = sPolyFaces.GetSize();

  // for every PolyFace - triangulate it
  for(ULONG ii=0;ii<lNumFaces;ii++)
    {
      SmPolyFace *pPolyFace = sPolyFaces[ii];
      // m_lTriangulateFaceLevel = 0;
      if ( m_bAdvancingFront )
        {
          // triangulate Face with AdvancingFront (AF) - calls TessellateWithQuads
          eStat = TriangulateFaceAF( pPolyFace,
                                     pFace,
                                     pSC->m_dMaximumSideLength3D,
                                     sNewPolyFaces );
        }
      else // triangulate face without AdvancingFront (AF)
        {
          eStat = TriangulateFace( pPolyFace );
        }

      // remember when any Face triangulation failed
      if(eStat != SM_SUCCESS)
        { eRetStatus = SM_ERR; }

#ifdef SM_DEBUG_CODE
      if (bDebugMe)
        {
          SM_ASSERT_VALID(pPolyBrep) ;

          if ( FALSE )
            { smgfx_Erase(); }
          smgfx_SetLook( 1,2, 0,0,0 ); pPolyBrep->Draw(); sm_GraphicsLoop();
          sm_GraphicsLoop();
        }
#endif // SM_DEBUG_CODE

    } // end iter every PolyFace triangulating it

  // Here is where we should do some work to move vertices around
  // to make better looking polygons.
  if ( !m_bAdvancingFront ) // Advancing front already smoothed.
    {
      SER( SmoothPolygons( pPolyBrep ));
    }

#ifdef SM_DEBUG_CODE
  if (bDebugMe)
    {
      SM_ASSERT_VALID(pPolyBrep) ;

      if ( FALSE )
        { smgfx_Erase(); }
      smgfx_SetLook( 1,2, 0,0,0 ); pPolyBrep->Draw( FALSE ); sm_GraphicsLoop();
      smgfx_SetLook( 1,2, 0,0,0 ); pPolyBrep->Draw( TRUE  ); sm_GraphicsLoop();
      sm_GraphicsLoop();

      SmTArray< SmPolyFace* > sPFs;
      pPolyBrep->GetPolyFaces( sPFs );
      for ( ULONG jj=0; jj<sPFs.GetSize(); jj++ )
      {
          SmPolyFace *pPF = sPFs[jj];

          SmTArray< SmPolyEdge* > sEdges;
          pPF->GetPolyEdges( sEdges );
          for (ULONG jk=0; jk<sEdges.GetSize(); jk++) {
               smgfx_SetLook(1,2, 0,0,1); sEdges[jk]->Draw  (); sm_GraphicsLoop();
               smgfx_SetLook(1,2, 0,0,1); sEdges[jk]->Draw3D(); sm_GraphicsLoop();
          }
      }
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

  // all done
  return eRetStatus;

} // end SmTess::AddQuadBoundaries

/*******************************************************************//**
PURPOSE: After Phase1SetupBrep() has run once and
         After Phase2CreateFacePolygons() has run once for every m_pTessBrep->Face,
  Output all m_pTessBrep->Face[ii]->Surface (SmTessSrfCache)pSC->mTS_pPolyBrep
           pPolyFaces as polygons through rPolygonOutput class

NOTES: 
 1.
 2. rPolygonOutput.m_eOutputType = control OutputPolygon() output behavior, oneof:
         SM_PO_CREATE_POLYBREP,              // Output creates a 3-D SmPolyBrep in SmTess::m_p3DPolyBrep
         SM_PO_TRIANGLES,                    // Output only triangles.
         SM_PO_QUADRALATERALS,               // Output quadralaterals when two adjacent
                                             //   coplanar polygons are found that border with their longest edge.
                                             //   Otherwise output triangles when these conditions are not meet.
         SM_PO_TRIANGLE_STRIPS,              // Output triangle strips (see OpenGL) when possible (see m_lMinFanTriangles)
         SM_PO_TRIANGLE_FANS,                // Output triangle fans (see OpenGL) when possible.
         SM_PO_STRIPS_OR_FANS,               // Output strips when possible and then fans next.
         SM_PO_TRIANGLE_MESH,                // Output a mesh of triangles (indexed set of arrays) for each face.
         SM_PO_ALL_POLYGON_EDGES,            // Output only the polygon edges
         SM_PO_ALL_NON_PLANAR_EDGES,         // Output only non-coplanar edges of a polygon
         SM_PO_BOUNDARY_EDGES,               // Output edges that correspond to trimmed surface boundaries
         SM_PO_BOUNDARY_AND_SILHOUETTE_EDGES // Output boundary edges and silhouette edges of polygons.
***********************************************************************/
SmStatus SmTess::OutputPolygons
  (SmPolygonOutputCallback & rPolygonOutput,  // in : Chooses how and where to output the polygons
   SmGfxArraySet           * pOptGfxSet)
{
  // locals
  ULONG ii, jj ;
  SmTopologyTraverser sTraverser ;
  SM_PTR_ARRAY(sAllFaces,       SmFace, 512) ; // SmTArray<SmFace*>
  SM_PTR_ARRAY(sConnectedFaces, SmFace, 512) ; // SmTArray<SmFace*>
  m_pTessBrep->GetFaces(sAllFaces);

  // when there are faces to output
  if(sAllFaces.GetSize() > 0)
    {

      // Clear old PolyVertex back pointers
      m_pTessBrep->ClearAllPolyVertices() ;

      // pick, lock, and call NewMark() on any unmarked Mark
      SM_ASSERT_BREAK(sAllFaces[0] != NULL) ;
      SmNewMarkAndLock sMarkLock((SmContext *)sAllFaces[0]->GetContext()) ;
      SmMarkType       eMarkType = sMarkLock.GetMarkType() ;

      // for every m_pTessBrep->Face - call OutputFacePolygons
      for(ii=0; ii<sAllFaces.GetSize(); ii++)
        {
          SmFace * pFace = sAllFaces[ii] ;

          // skip already processed faces
          if(pFace->IsMarked(eMarkType))
            { continue ; }

          // collect and mark all faces connected to pFace through shared edges and vertices
          sTraverser.CollectFaces(pFace,           // in : Seed face (gets marked)
                                  sConnectedFaces, // out: List of connected faces (Get marked)
                                  eMarkType) ;     // in : specify mark for target objects (not incremented)
          SmPolyShell * pPolyShell = NULL ;

          // for all connected faces
          for(jj=0;jj<sConnectedFaces.GetSize();jj++)
            {

              // output sets of PolyFaces all owned by PolyShell made for the first PolyFace
              SER(OutputFacePolygons
                    (sConnectedFaces[jj], // in : target face
                     rPolygonOutput,      // in : Chooses how and where to output the polygons
                     &pPolyShell,         // in : only used when rPolygonOutput.GetOutputType() == SM_PO_CREATE_POLYBREP
                                          //        ppOptPolyShell NULL    = every new SmPolyFace gets a new SmPolyShell and SmPolyRegion
                                          //       *ppOptPolyShell NotNULL = owner of any new SmPolyFaces constructed
                                          //       *ppOptPolyShell NULL    = create and save ptr to new PolyShell for New SmPolyFaces of pFace
                                          //      NULL to ignore, default:[NULL]
                     pOptGfxSet));        // i/o: only used when rPolygonOutput.GetOutputType() != SM_PO_CREATE_POLYBREP
                                          //      used for OpenGL ES 2.0. When given, accumulate GfxVertexArrays not GL calls.
                                          //      NULL to ignore. default:[NULL]
                                          // note: increments an unlocked mark type
#ifdef SM_DEBUG_CODE
              SmBoolean bDebugMe = FALSE;
              if(bDebugMe)
                {
                  ULONG di ;
                  SmTessSrfCache * pSC           = m_vCache.GetSecond(pFace); // fetch the tessellated surface cache for this face from the stored SmRelation
                  SmPolyBrep     * pFPB_PolyBrep = (pSC) ? pSC->mTS_pPolyBrep : NULL ;
                  SM_ASSERT_VALID(m_p3DPolyBrep) ;
                  SM_ASSERT_VALID(pFPB_PolyBrep) ;

                  SM_PTR_ARRAY(sPolyVertices3d,  SmPolyVertex, 256) ; // SmTArray<SmPolyVertex *>
                  SM_PTR_ARRAY(sPolyVertices2d,  SmPolyVertex, 256) ; // SmTArray<SmPolyVertex *>

                  // get all PolyBrep PolyVertices
                  if(m_p3DPolyBrep)
                    { m_p3DPolyBrep->GetPolyVertices(sPolyVertices3d); }
                  ULONG lNumPolyVertices3d = sPolyVertices3d.GetSize() ;

                  if(pFPB_PolyBrep)
                    { pFPB_PolyBrep->GetPolyVertices(sPolyVertices2d); }
                  ULONG lNumPolyVertices2d = sPolyVertices2d.GetSize() ;

                  TCHAR sBuff[SM_TBLOCK_SIZE];
                  smos_sprintf(sBuff,   _T("\nSmTess::OutputPolygons - m_p3DPolyBrep PolyVertCnt = %ld"), lNumPolyVertices3d) ;
                  smos_WriteBuffer(sBuff) ;

                  for(di=0; di<lNumPolyVertices3d; di++)
                    {
                      SmPolyVertex * pPolyVertex3d = sPolyVertices3d[di];
                      SmVertex     * pVertex       = pPolyVertex3d->GetVertex() ;
                      SmPolyVertex * pBackPtr      = pVertex ? pVertex->GetPolyVertex() : NULL ;

                      smos_sprintf(sBuff, _T("\n[%3ld]: PolyVertex3d:[0x%p],                              PolyPos:"), di, pPolyVertex3d) ;
                      smos_WriteBuffer(sBuff) ;
                      if(pPolyVertex3d) { pPolyVertex3d->GetPoint().Dump() ; }
                      smos_sprintf(sBuff, _T("\n       BackPtr     :[0x%p], Vertex:[0x%p], VertPos:"), pBackPtr, pVertex) ;
                      smos_WriteBuffer(sBuff) ;
                      if(pVertex) pVertex->GetPoint().Dump() ;
                    } // end iter every FacePolyBrep->PolyVertex

                  smos_sprintf(sBuff,   _T("\nSmTess::OutputPolygons - pFPB_PolyBrep PolyVertCnt = %ld"), lNumPolyVertices2d) ;
                  smos_WriteBuffer(sBuff) ;

                  for(di=0; di<lNumPolyVertices2d; di++)
                    {
                      SmPolyVertex * pPolyVertex2d = sPolyVertices2d[di];
                      SmVertex     * pVertex       = pPolyVertex2d->GetVertex() ;
                      SmPolyVertex * pBackPtr      = pVertex ? pVertex->GetPolyVertex() : NULL ;

                      smos_sprintf(sBuff, _T("\n[%3ld]: PolyVertex2d:[0x%p],                              PolyPos:"), di, pPolyVertex2d) ;
                      smos_WriteBuffer(sBuff) ;
                      if(pPolyVertex2d) { pPolyVertex2d->GetPoint().Dump() ; }
                      smos_sprintf(sBuff, _T("\n       BackPtr     :[0x%p], Vertex:[0x%p], VertPos:"), pBackPtr, pVertex) ;
                      smos_WriteBuffer(sBuff) ;
                      if(pVertex) pVertex->GetPoint().Dump() ;
                    } // end iter every FacePolyBrep->PolyVertex
                }
#endif // SM_DEBUG_CODE

            } // end iter all faces connected to sAllFaces[ii]
        } // end iter sAllFaces[ii]
    } // end faces to output check

  // give the caller a chance to post process - default:[take no actions]
  SER(rPolygonOutput.CompletedPolygonOutput());

  // During tessellation sTess.m_p3DPolyBrep back ptrs are current.
  // After tessellation back ptrs can become stale if target pBrep is deleted.
  if(m_p3DPolyBrep) { m_p3DPolyBrep->SetOKBackPtrs(FALSE) ; }

  // all done
  return SM_SUCCESS;

} // end SmTess::OutputPolygons

/*******************************************************************//**
PURPOSE: Is this a PolyEdge which splits a quadralateral in UV?

NOTES: return TRUE if the two PolyFaces connected to PolyEdge form
       a rectangular axis-aligned UVDomain.
***********************************************************************/
SmBoolean SmTess::IsQuadSplittingEdge
 (SmPolyEdge  * pPolyEdge,     // in : target PolyEdge
  SmPolyEdge *& rpBottomEdge,  // out:
  SmPolyEdge *& rpRightEdge,   // out:
  SmPolyEdge *& rpTopEdge,     // out:
  SmPolyEdge *& rpLeftEdge)    // out:
 const
{
  // init output
  rpBottomEdge = NULL;
  rpRightEdge  = NULL;
  rpTopEdge    = NULL;
  rpLeftEdge   = NULL;

  // no work - edge is not manifold
  if (!pPolyEdge->IsManifold())
    { return FALSE; }

  // PolyFace->PolyEdge locals
  SmPoint3d    sStart   = pPolyEdge->GetStartPoint();
  SmPoint3d    sEnd     = pPolyEdge->GetEndPoint();

  SmPolyEdge * pCWEdge  = pPolyEdge->GetCWNonDegeneratePolyEdge();
  SmPolyEdge * pCCWEdge = pPolyEdge->GetCCWNonDegeneratePolyEdge();
  // SmPolyEdge * pCWEdge  = pPolyEdge->GetCWPolyEdge();
  // SmPolyEdge * pCCWEdge = pPolyEdge->GetCCWPolyEdge();

  // No Work - PolyEdge is not part of a triangle
  if (pCWEdge->GetCWNonDegeneratePolyEdge() != pCCWEdge)
    { return FALSE; }

  // Neighbor PolyFace PolyEdge locals
  SmPolyEdge * pRadial     = pPolyEdge->GetRadial();
  SmPolyEdge * pRadCWEdge  = pRadial->GetCWNonDegeneratePolyEdge();
  SmPolyEdge * pRadCCWEdge = pRadial->GetCCWNonDegeneratePolyEdge();
  // SmPolyEdge * pRadCWEdge  = pRadial->GetCWPolyEdge();
  // SmPolyEdge * pRadCCWEdge = pRadial->GetCCWPolyEdge();

  // No work - Neighbor PolyFace is not a triangle
  // if (pRadCWEdge->GetCWPolyEdge() != pRadCCWEdge)
  if (pRadCWEdge->GetCWNonDegeneratePolyEdge() != pRadCCWEdge)
    { return FALSE; }

  // geometry locals
  double    dTol      = 50.0 * SM_EFF_ZERO * (1.0 + sStart.GetMaxDimension() + sEnd.GetMaxDimension());
  double    dTolSq    = dTol * dTol;
  SmPoint3d sCWPnt    = pCWEdge->GetStartPoint();
  SmPoint3d sRadCWPnt = pRadCWEdge->GetStartPoint();

  // BBox of target PolyEdge
  SmExtent3d sBBox(sStart);
  sBBox.AddPoint3d(sEnd);

  // BBox of possible quad other diagonal
  SmExtent3d sBBox2(sCWPnt);
  sBBox2.AddPoint3d(sRadCWPnt);

  // test if two neighbors make a rectangular quad to tol
  if (!sBBox.ContainsPoint3d(sCWPnt,dTol))    { return FALSE; }
  if (!sBBox.ContainsPoint3d(sRadCWPnt,dTol)) { return FALSE; }
  if (!sBBox2.ContainsPoint3d(sStart,dTol))   { return FALSE; }
  if (!sBBox2.ContainsPoint3d(sEnd,dTol))     { return FALSE; }

  // Here we definitely have a rectangular domain.  Just figure out
  // what edges go where.

  SM_PTR_ARRAY(sPolyEdges, SmPolyEdge, 32) ; // SmTArray<SmPolyEdge*>
  sPolyEdges.Add(pCWEdge);
  sPolyEdges.Add(pCCWEdge);
  sPolyEdges.Add(pRadCWEdge);
  sPolyEdges.Add(pRadCCWEdge);

  // 4 domain corner points
  SmPoint3d sMinUV    = sBBox.GetMin();
  SmPoint3d sMaxUV    = sBBox.GetMax();
  SmPoint3d sMaxUMinV = sBBox.Evaluate(1.0,0.0,0.0);
  SmPoint3d sMinUMaxV = sBBox.Evaluate(0.0,1.0,0.0);

  // For all 4 PolyEdges - figure out which one starts in which Quad corner
  for (ULONG i=0; i<sPolyEdges.GetSize(); i++)
    {
      SmPolyEdge *pE = sPolyEdges[i];
      SmPoint3d sESt = pE->GetStartPoint();
      if      (sESt.DistanceBetweenSquared(sMinUV)    < dTolSq) { rpBottomEdge = pE; }
      else if (sESt.DistanceBetweenSquared(sMaxUV)    < dTolSq) { rpTopEdge    = pE; }
      else if (sESt.DistanceBetweenSquared(sMaxUMinV) < dTolSq) { rpRightEdge  = pE; }
      else if (sESt.DistanceBetweenSquared(sMinUMaxV) < dTolSq) { rpLeftEdge   = pE; }
      else
        {
          return FALSE;
        }
    }

  // Make sure all are set.
  if(   rpBottomEdge == NULL
     || rpTopEdge    == NULL
     || rpRightEdge  == NULL
     || rpLeftEdge   == NULL)
    { return FALSE; }

  // all done - this is an axis aligned Quad
  return TRUE;

} // end SmTess::IsQuadSplittingEdge

/*******************************************************************//**
PURPOSE: Trace polygons that correspond to square parametetric domains.

NOTES: For right now we will always trace upward.
***********************************************************************/
SmStatus SmTess::TraceQuads
 (SmSurfParamType           eDirection,          // in : SM_SP_U = hold V constant, move in positive U dir.
                                                 //      SM_SP_V = hold U constant, move in positive V direction.
  SmPolyFace              * pStartFace,          // in :
  SmBoolean              /* bForce3DPlanarity */,
  SmTArray<SmPolyFace*>   & rTracedFaces,        // out:
  SmTArray<SmPolyVertex*> & rMinVerts,           // out:
  SmTArray<SmPolyVertex*> & rMaxVerts,           // out:
  SmMarkType                eMarkType)           // in : checks without incrementing or assigning eMarkTypeValue
{
  // init output
  rTracedFaces.ReSet();
  rMinVerts.ReSet();
  rMaxVerts.ReSet();

  // locals
  SM_PTR_ARRAY(sPolyEdges, SmPolyEdge, 64) ; // SmTArray<SmPolyEdge*>
  SmPolyFace * pCurrFace  = pStartFace;
  SmPolyEdge * pFirstLeft = NULL;
  SmPolyEdge * pFirstBot  = NULL;

#ifdef SM_DEBUG_CODE
  SmBoolean bDebugMe = FALSE;
  if (bDebugMe)
    {
      SmPolyBrep * pPolyBrep = pStartFace->GetPolyBrep() ;

      smgfx_Erase();
      smgfx_SetColor(1,0,0); pStartFace->Draw(); sm_GraphicsLoop();
      smgfx_SetColor(0,0,0); if(pPolyBrep) pPolyBrep->Draw(); sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

  // for two iterations
  for (ULONG lDirection=0; lDirection<2; lDirection ++)
    {
      // on second pass - set up for upcoming while loop
      if (lDirection == 1)
        {
          if (rTracedFaces.GetSize() == 0) break;
          rMinVerts.ReverseArray(0,rMinVerts.GetSize());
          rMaxVerts.ReverseArray(0,rMaxVerts.GetSize());

          if (eDirection == SM_SP_U) { SmPolyEdge *pLeftRad = pFirstLeft->GetRadial();
                                       if (!pLeftRad) break;
                                       pCurrFace = pLeftRad->GetPolyFace();
                                     }
          else                       { SmPolyEdge *pBotRad = pFirstBot->GetRadial();
                                       if (!pBotRad) break;
                                       pCurrFace = pBotRad->GetPolyFace();
                                     }
        } // end lDirection == 1 check

      SmBoolean bDone = FALSE;

      // while still finding Triangle pairs that form axis aligned rectangual quads
      while (!bDone)
        {
          bDone = TRUE;

          // quit when CurrFace is already processed
          if (pCurrFace->IsMarked(eMarkType))
            { break; }

          pCurrFace->GetPolyEdges(sPolyEdges);

          double dLongestLength = -1.0;
          ULONG lLongestEdge = 0;

          // for every PolyFace->PolyEdge - find the longest
          for (ULONG jj=0; jj<sPolyEdges.GetSize(); jj++)
            {
              SmPolyEdge *pPolyEdge = sPolyEdges[jj];

              // when PolyEdge is Manifold
              if (pPolyEdge->IsManifold())
                {
                  SmPolyVertex * pStartV  = pPolyEdge->GetStartPolyVertex();
                  SmPolyVertex * pEndV    = pPolyEdge->GetOtherPolyVertex(pStartV);
                  SmVector3d     sEdgeVec = pEndV->GetPoint() - pStartV->GetPoint();
                  double dEdgeLengthSq = sEdgeVec.LengthSquared();
                  if (dEdgeLengthSq > dLongestLength)
                    {
                      dLongestLength = dEdgeLengthSq;
                      lLongestEdge   = jj;
                    }
                } // end is Manifold check
            } // end iter every PolyEdge seeking the longest

          // skip degenerate PolyFaces
          if (dLongestLength < 0.0)
            { break; }

          // Longest PolyEdge and neighbors
          SmPolyEdge *pLongestEdge = sPolyEdges[lLongestEdge];
          SmPolyEdge *pRightEdge, *pBottomEdge, *pTopEdge, *pLeftEdge;

          // if pLongestEdge does not divide two PolyFaces which form an axis aligned UVDomain rectangle
          if(FALSE == IsQuadSplittingEdge(pLongestEdge,
                                          pBottomEdge,
                                          pRightEdge,
                                          pTopEdge,
                                          pLeftEdge))
            {
              // no more quads found - quit
              bDone = TRUE;
            }
          else // pLongestEdge does divide a pair of PolyFaces that form an axis aligned rectangular domain
            {
              if (pCurrFace == pStartFace)
                {
                  pFirstLeft = pLeftEdge;
                  pFirstBot  = pBottomEdge;
                }
              SmPolyEdge * pRad     = pLongestEdge->GetRadial();
              SmPolyFace * pRadFace = pRad->GetPolyFace();

              // skip already processed RadFaces
              if (pRadFace->IsMarked(eMarkType))
                { break; }

              rTracedFaces.Add(pCurrFace);
              rTracedFaces.Add(pRadFace);
              if (eDirection == SM_SP_U)
                {
                  // add first verts
                  if (rMinVerts.GetSize() == 0) { rMinVerts.Add(pBottomEdge->GetStartPolyVertex());
                                                  rMaxVerts.Add(pLeftEdge->GetStartPolyVertex());
                                                }
                  // add second vertex
                  if (lDirection == 1)          { rMinVerts.Add(pBottomEdge->GetStartPolyVertex());
                                                  rMaxVerts.Add(pLeftEdge->GetStartPolyVertex());
                                                }
                  else                          { rMinVerts.Add(pRightEdge->GetStartPolyVertex());
                                                  rMaxVerts.Add(pTopEdge->GetStartPolyVertex());
                                                }

                  // get next iter pCurrFace
                  if (lDirection == 0) { SmPolyEdge *pRightRad = pRightEdge->GetRadial();
                                         if (pRightRad == NULL) break;
                                         pCurrFace = pRightRad->GetPolyFace();
                                       }
                  else                 { SmPolyEdge *pLeftRad = pLeftEdge->GetRadial();
                                         if (pLeftRad == NULL) break;
                                         pCurrFace = pLeftRad->GetPolyFace();
                                       }
                }
              else // (eDirection == SM_SP_V)
                {
                  // add first verts
                  if (rMinVerts.GetSize() == 0) { rMinVerts.Add(pBottomEdge->GetStartPolyVertex());
                                                  rMaxVerts.Add(pRightEdge->GetStartPolyVertex());
                                                }
                  // add second vertex
                  if (lDirection == 1)          { rMinVerts.Add(pBottomEdge->GetStartPolyVertex());
                                                  rMaxVerts.Add(pRightEdge->GetStartPolyVertex());
                                                }
                  else                          { rMinVerts.Add(pLeftEdge->GetStartPolyVertex());
                                                  rMaxVerts.Add(pTopEdge->GetStartPolyVertex());
                                                }
                  // get next iter pCurrFace
                  if (lDirection == 0) { SmPolyEdge *pTopRad = pTopEdge->GetRadial();
                                         if (pTopRad == NULL) break;
                                         pCurrFace = pTopRad->GetPolyFace();
                                       }
                  else                 { SmPolyEdge *pBotRad = pBottomEdge->GetRadial();
                                         if (pBotRad == NULL) break;
                                         pCurrFace = pBotRad->GetPolyFace();
                                       }
                } // end (eDirection == SM_SP_V) branch

                // arrive here after finding an axis aligned Quad - look for another one
              bDone = FALSE;

            } // end pLongestEdge divides two PolyFaces which form an axis aligned UVDomain rectangle check

        } // end while still finding quad elements loop
    } // end iter lDirection:[0,1]

  // order output
  if (rMinVerts.GetSize() > 0)
    {
      rMinVerts.ReverseArray(0,rMinVerts.GetSize());
      rMaxVerts.ReverseArray(0,rMaxVerts.GetSize());
    }

  // all done
  return SM_SUCCESS;

} // end SmTess::TraceQuads

/*******************************************************************//**
PURPOSE: Get the fan defined that uses pPEdge

NOTES: 
***********************************************************************/
static SmStatus sm_GetFan
 (SmPolyEdge            * pPolyEdge,  // in :
  SmTArray<SmPolyEdge*> & rFanEdges,  // out:
  SmMarkType              eMarkType)  // in : uses without incrementing or assigning eMarkType value
{
  // init output
  rFanEdges.ReSet();

  // Now Traverse polygons on clockwise side until
  // we hit the end of available fan polygons.
  // by either hitting the edge or a marked polygon.
  SmPolyEdge *pStartEdge = pPolyEdge;

  // while still traversing a sequence of triangles
  while (TRUE)
    {
      // done when we hit a NonManifold Edge
      if (!pStartEdge->IsManifold())
        { break; }

      // neighbor locals
      SmPolyEdge * pRadEdge = pStartEdge->GetRadial(); NER(pRadEdge);
      SmPolyFace * pRadFace = pRadEdge->GetPolyFace();

      // done when we hit a Marked Face
      if (pRadFace->IsMarked(eMarkType))
        { break; }

      // Move to next side
      pStartEdge = pRadEdge->GetCCWNonDegeneratePolyEdge();
      // used to be: pStartEdge = pRadEdge->GetCCWPolyEdge();

      // done when we come back to the start
      if (pStartEdge == pPolyEdge)
        { break; }

    } // end while still traversing a sequence of Triangles

  // Now Traverse polygons clockwise and pick up edges along
  // the fan starting at the start edge.
  rFanEdges.Add(pStartEdge);
  rFanEdges.Add(pStartEdge->GetCCWNonDegeneratePolyEdge());
  // rFanEdges.Add(pStartEdge->GetCCWPolyEdge());
  while (TRUE)
    {
      SmPolyEdge *pCW = pStartEdge->GetCWNonDegeneratePolyEdge();
      // SmPolyEdge *pCW = pStartEdge->GetCWPolyEdge();

      rFanEdges.Add(pCW);

      // done when we hit a NonManifold Edge
      if (!pCW->IsManifold())
       { break; }

      // neighbor locals
      SmPolyEdge * pRadial = pCW->GetRadial(); NER(pRadial);
      SmPolyFace * pRadFace = pRadial->GetPolyFace();

      // done when we hit a Marked PolyFace
      if (pRadFace->IsMarked(eMarkType))
        { break; }

      // done when we hit a nonTriangular PolyFace
      if (!pRadFace->IsTriangle())
        { break; }

      pStartEdge = pRadial;

      // done when we come back to the start
      if (pStartEdge == rFanEdges[0])
        { break; }

    } // end while still traversing a sequence of Triangles

  // all done
  return SM_SUCCESS;

} // end sm_GetFan

/*******************************************************************//**
PURPOSE: Tessellate Output - copy one PFace->Surface's
  (SmTessSrfCache)pSC->mTS_pPolyBrep->pPolyTopology objs to m_p3DPolyBrep.
  m_pTessBrep         = not modified.
  m_vCache            = not modified
  pSC->mTS_pPolyBrep  = not modified
  m_p3DPolyBrep       = linear approx of m_pTessBrep with one SmPolyEdge
                        for every m_pTessBrep->Edge and many SmPolyFaces
                        for every m_pTessBrep->Face.
                        m_p3DPolyBrep->m_bOKBackPtrs == TRUE.  During tessellation
                        back ptrs are current. After, back ptrs can go stale

NOTES: good for aggregating several pFace tessellations into one SmPolyBrep
  called by OutputFacePolygons(pFace, rPolygonOutput) when
  rPolygonOutput->m_eOutputType == SM_PO_CREATE_POLYBREP

  1. This may create degenerate polygons and the results
     will not be stitched together.  Use the stitching to remove degenerate
     polygons and glue edges together to make a solid.
  2. bReverse: reverses polygons of faces whose Upward faceuses are NOT conected to the infinite region.
               This is a reasonbale behavior for displaying outer shells of manifold objects
               but may be too restrictive for displaying inner sides of outer shells, or inner shells.
               If this is a problem, ask SMS support to include a bReverse input argument giving
               control of the feature to the caller.

***********************************************************************/
SmStatus SmTess::OutputToPolyBrep
 (SmFace                     * pFace,            // in : target face
  SmPolygonOutputCallback    & rPolygonOutput,   // NotUsed: in : Chooses how and where to output the polygons
  SmPolyShell               ** ppOptPolyShell,   // in : NULL to ignore, default:[NULL], else
                                                 //       ppOptPolyShell NULL    = every SmPolyFace gets its own SmPolyShell and SmPolyRegion
                                                 //      *ppOptPolyShell NotNULL = owner of any new SmPolyFaces constructed
                                                 //      *ppOptPolyShell NULL    = create and save ptr to new PolyShell for New SmPolyFaces of pFace
  const SmTArray<SmPoint3d>  & cr3DPoints,       // in : 3dPoint list
  const SmTArray<SmPoint2d>  & crUVPoints,       // in : associated UVPoints
  const SmTArray<SmVector3d> & cr3DNormals,      // in : associated 3dNormals
  const SmTArray<SmVertex*>  & crBrepVertices)   // in : associated BrepVertex or NULL=none
{
  SM_REF1(rPolygonOutput) ; 
  // local references
  SmTessSrfCache * pSC           = m_vCache.GetSecond(pFace);
  SmPolyBrep     * pFPB_PolyBrep = (pSC != NULL) ? pSC->mTS_pPolyBrep : NULL ;
  SmFaceuse      * pFU           = pFace->GetUpwardFaceuse();
  double           dTolerance    = pFace->GetBrep()->GetTolerance();
  SmBoolean        bReverse      = (pFU->GetShell()->GetRegion() != pFace->GetBrep()->GetInfiniteRegion() ) ? TRUE : FALSE;

  // increment and lock an unlocked mark
  SmNewMarkAndLock sMarkLock( pFace->GetContext(), SM_MT_ALLMARKS) ; // increment and lock any unlocked mark
  SmMarkType eMarkType = sMarkLock.GetMarkType() ;

  // gwc: obsolete
  //  init an empty m_pBrepToPolyBrepMap, mapping from orig Brep to PolyBrep elements
  //  if (m_pBrepToPolyBrepMap == NULL)
  //    { m_pBrepToPolyBrepMap = new (m_crContext) SmMapPtrToPtr(1000,&m_crContext); }
  //  else
  // GWC: why reinitialize this - don't we want to accumulate all the vertex maps from all the faces?
  //    { m_pBrepToPolyBrepMap->RemoveAll(); }

  // ensure 3DPolyBrep exists - use orig Brep tol which may be too big.
  //  1. m_p3DPolyBrep only is created on first call.
  //     subsequent calls continue to add polygons to the same m_p3DPolyBrep
  //  2. m_p3DPolyBrep->m_bOKBackPtrs == TRUE. During Tessellation back ptrs are current.
  //     After Tessellation, back ptrs can become stale.

  if (m_p3DPolyBrep == NULL)
    { m_p3DPolyBrep = new (m_crContext) SmPolyBrep(dTolerance);
      m_p3DPolyBrep->SetOKBackPtrs(TRUE) ;  // the original Brep will last as long as the cache
    }                                       //  backPts SmPolyFace::m_pFace, SmPolyEdge::m_pEdgeuse, SmPolyVertex::m_pVertex are valid

  // no work - no polygons for this face
  if (!pSC)
    { return SM_SUCCESS; }

  // no work - no FacePolyBrep
  NER(pFPB_PolyBrep);

  // local data
  ULONG ii, jj, kk;
  SM_PTR_ARRAY(sFPB_PolyLoopEdges, SmPolyEdge,       256) ; // SmTArray<SmPolyEdge *>
  SM_PTR_ARRAY(sFPB_PolyFaceLoops, SmPolyLoop,       256) ; // SmTArray<SmPolyLoop *>
  SM_PTR_ARRAY(sFPB_PolyFaces,     SmPolyFace,       256) ; // SmTArray<SmPolyFace *>
  SM_PTR_ARRAY(sFPB_PolyVertices,  SmPolyVertex,     256);  // SmTArray<SmPolyVertex *>
  SM_PTR_ARRAY(s3DPolyVertices,    SmPolyVertex,     256);  // SMTArray<SmPolyVertex *>
  SM_PTR_ARRAY(sDelFaces,          SmPolyFace,        16);  // SmTArray<SmPolyFace *>
  SM_PTR_ARRAY(sDelLoops,          SmPolyLoop,        16);  // SmTArray<SmPolyFace *>
  SM_PTR_ARRAY(sNewVertices,       SmPolyVertex,     256);  // SmTArray<SmPolyVertex *>
  SM_PTR_ARRAY(sNewAuxData,        SmPolyVertAuxData,256);  // SmTArray<SmPolyVertAuxData *>

  sNewVertices.SetSize(crBrepVertices.GetSize());
  sNewAuxData.SetSize (crBrepVertices.GetSize());

  // get all FacePolyBrep PolyFaces and PolyVertices
  pFPB_PolyBrep->GetPolyFaces(sFPB_PolyFaces);
  pFPB_PolyBrep->GetPolyVertices(sFPB_PolyVertices);
  ULONG lNumAllVerts = sFPB_PolyVertices.GetSize();
  ULONG lNumFaces    = sFPB_PolyFaces.GetSize();

  // for every FacePolyBrep PolyVertex - create(sets OrigVertex) or get a PolyVertex in m_p3DPolyBrep and an associated SmPolyVertAuxData
  for(ii=0; ii<lNumAllVerts; ii++)
    {
      SmPolyVertex      * pFPB_PolyVertex = sFPB_PolyVertices[ii];
      SmPolyVertex      * pPolyVertex3D;
      SmPolyVertAuxData * pNewAux;
      SER(GetOrCreatePolyVertex(pFPB_PolyVertex,  // in : PolyVertex to map to m_p3DPolyBrep PolyVertices
                                cr3DPoints,       // in : assoc 3dPoint    in cr3DPoints    [pVertexIn2D->m_lIndexValue]
                                crUVPoints,       // in : assoc UVPoint    in crUVPoints    [pVertexIn2D->m_lIndexValue]
                                cr3DNormals,      // in : assoc 3dNormal   in cr3DNormals   [pVertexIn2D->m_lIndexValue]
                                crBrepVertices,   // in : assoc BrepVertex in crBrepVertices[pVertexIn2D->m_lIndexValue]
                                pPolyVertex3D,    // out: PolyVertex mapped to pVertexIn2D->BrepVertex in m_pBrepToPolyBrepMap
                                pNewAux));        // out: new associated Aux Data (UVPnt & Normal - to be attached to appropriate SmPolyEdge)
      sNewVertices[pFPB_PolyVertex->GetIndexValue()] = pPolyVertex3D ;
      sNewAuxData [pFPB_PolyVertex->GetIndexValue()] = pNewAux;

      // PolyVertex may have been marked on prior call to OutputToPolyBrep. Clear mark for use below.
      // After marks have been improved, we can just iterate mark instead unmarking.
      pPolyVertex3D->UnMark(SM_MT_ALLMARKS);
    } // end iter every FacePolyBrep->PolyVertex

#ifdef SM_DEBUG_CODE
  SmBoolean bDebugMe = FALSE;
  if(bDebugMe)
    {
      TCHAR sBuff[SM_TBLOCK_SIZE];
      smos_sprintf(sBuff,   _T("\nSmTess::OutputToPolyBrep - number of PolyVertices = %ld"), lNumAllVerts) ;
      smos_WriteBuffer(sBuff) ;

      for(ii=0; ii<lNumAllVerts; ii++)
        {
          SmPolyVertex      * pFPB_PolyVertex = sFPB_PolyVertices[ii];
          SmPolyVertex      * pNewVertex      = sNewVertices[pFPB_PolyVertex->GetIndexValue()] ;
          SmPoint3d           sNewPoint       = pNewVertex->GetPoint() ;
          SmVertex          * pVertex         = pFPB_PolyVertex->GetVertex() ;
          SmPolyVertex      * pBackPtr        = pVertex ? pVertex->GetPolyVertex() : NULL ;
          ULONG               lIndx           = pFPB_PolyVertex->GetIndexValue() ;

          smos_sprintf(sBuff, _T("\n[%3ld]: FPB_PolyVertex:[0x%p], Idx:[%3ld], New_PolyVertex:[0x%p], Vertex:[0x%p], NewPos :"),
                           ii, pFPB_PolyVertex, lIndx, pNewVertex, pVertex) ;
          smos_WriteBuffer(sBuff) ;
          sNewPoint.Dump() ;
          smos_sprintf(sBuff, _T("\n                                                       BackPtr       :[0x%p]                               VertPos:"),
                           pBackPtr) ;
          smos_WriteBuffer(sBuff) ;
          if(pVertex) { pVertex->GetPoint().Dump() ; }
        } // end iter every FacePolyBrep->PolyVertex
    }
#endif // SM_DEBUG_CODE

  // for every FacePolyBrep->PolyFace - create a new SmPolyFace in m_p3DPolyBrep
  for (ii=0; ii<lNumFaces; ii++)
    {
      SmPolyFace *pFPB_PolyFace  = sFPB_PolyFaces[ii];
      SmVector3d  sNorm          = pFPB_PolyFace->GetNormal3d() ;
      double      dNormSize      = sNorm.Length() ;
      SmPolyLoop *pFPB_OuterLoop = pFPB_PolyFace->GetOuterPolyLoop() ;
      if ( pFPB_OuterLoop == NULL )  // [B436]
        {
          WARN(_T(" SmTess::OutputToPolyBrep: Null OuterPolyLoop found in a PolyFace" ) );
          continue;
        }

      pFPB_OuterLoop->GetPolyEdges(sFPB_PolyLoopEdges);
      ULONG       lNumEdges      = sFPB_PolyLoopEdges.GetSize();
      ULONG       lEdge3DCnt     = 0 ;

      // count FPB OuterLoop distinct PolyEdge3D objects
      SM_ASSERT(pFPB_OuterLoop->IsClosed()) ;
      for(jj=0;jj<lNumEdges;jj++)
        {
          SmPolyEdge   * pFPB_PolyEdge = sFPB_PolyLoopEdges[jj] ;
          SmPolyEdge   * pFPB_NextEdge = sFPB_PolyLoopEdges[(jj+1)%lNumEdges] ;

          ULONG          lIndx         = pFPB_PolyEdge->GetStartPolyVertex()->GetIndexValue() ;
          ULONG          lNext         = pFPB_NextEdge->GetStartPolyVertex()->GetIndexValue() ;

          SmPolyVertex * pPolyVertex3D = sNewVertices[lIndx] ;
          SmPolyVertex * pNextVertex3D = sNewVertices[lNext] ;

          SmPoint3d      sPolyPoint3d  = pPolyVertex3D->GetPoint() ;
          SmPoint3d      sNextPoint3d  = pNextVertex3D->GetPoint() ;

          // Skip zero length PolyEdges in 3d - happens legally when UV tessellations map through a Surface Singularity.
          if(   pPolyVertex3D == pNextVertex3D
             || sPolyPoint3d.DistanceBetween(sNextPoint3d) < dTolerance/100.0)
            { continue ; }

          // else count the edge
          lEdge3DCnt++ ;

        } // end iter every FPB OuterLoop PolyEdge gathering distinct PolyVertex3d objects

      // skip PolyFaces whose outerloops map to 2 or fewer distince PolyEdge3Ds
      if(lEdge3DCnt <= 2)
        { continue ; }

      // check to see if the PolyFace outer loop has 3 or more distinct PolyVertex3d


      // make a PolyFace for each pFPB_PolyFace to add m_p3DPolyBrep
      SmPolyFace *pNewPFace = new (m_p3DPolyBrep) SmPolyFace(dTolerance,
                                                             m_p3DPolyBrep,
                                                             NULL,
                                                             ppOptPolyShell ? *ppOptPolyShell : NULL,
                                                             dNormSize < 1.1 ? &sNorm : NULL);
      if(ppOptPolyShell && *ppOptPolyShell == NULL)
        { *ppOptPolyShell = pNewPFace->GetPolyShell() ; }

      // propagate PolyFace data as needed
      pNewPFace->SetOriginalFace(pFPB_PolyFace->GetOriginalFace(), TRUE) ; // TRUE = copy pOriginalFace->Attributes onto pNewPFace

      // get FacePolyBrep->PolyFace->PolyLoops
      pFPB_PolyFace->GetPolyLoops(sFPB_PolyFaceLoops);
      ULONG lEdgesCreated = 0;
      ULONG lNumLoops = sFPB_PolyFaceLoops.GetSize();

      // for every FacePolyBrep->PolyFace->PolyLoop - build a Loop in pNewPFace
      for (jj=0; jj<lNumLoops; jj++)
        {
          SmPolyLoop *pNewPLoop = NULL ;
          SER(pNewPFace->StartPolyEdgeLoop(pNewPLoop)) ;

          pNewPLoop->m_eOrientation = (jj > 0) ? SM_OT_OPPOSITE : SM_OT_SAME ;

          // FPB_PolyLoop->Edges
          sFPB_PolyFaceLoops[jj]->GetPolyEdges(sFPB_PolyLoopEdges);

          // when asked - reverse the FPB_PolyLoop edge order
          if (bReverse)
            {
              sFPB_PolyLoopEdges.ReverseArray(0,sFPB_PolyLoopEdges.GetSize());
            }

          // for every nonDegenerate FPB_PolyLoop->Edge - Build a PolyLoop3D->Edge
          lNumEdges = sFPB_PolyLoopEdges.GetSize();
          for (kk=0; kk<lNumEdges; kk++)
            {
              SmPolyEdge   * pFPB_PolyEdge    = sFPB_PolyLoopEdges[kk];
              SmPolyVertex * pFPB_StartVertex = pFPB_PolyEdge->GetStartPolyVertex();
              SmPolyVertex * pFPB_EndVertex   = pFPB_PolyEdge->GetEndPolyVertex();

              if (bReverse)
                {
                  SmPolyVertex *pTmp = pFPB_StartVertex;
                  pFPB_StartVertex   = pFPB_EndVertex;
                  pFPB_EndVertex     = pTmp;
                }

              // check for zero length 2D edges in existing pFPB_PolyBrep MappedSmTessSrfCache
              if (pFPB_StartVertex == pFPB_EndVertex)
                {
                  SE(SM_ERR);
                }

              // FPB_PolyEdge->PolyVertices3D
              SmPolyEdge   * pNewPEdge;
              SmPolyVertex * pNewStart = sNewVertices[pFPB_StartVertex->GetIndexValue()];
              SmPolyVertex * pNewEnd   = sNewVertices[pFPB_EndVertex->GetIndexValue()];

              // Add surface normals to NewStart and NewEnd. Only need 1 per Original Face
              if (!pNewStart->IsMarked(eMarkType))
              {
                  pNewStart->AddFaceToList(pNewPFace);
                  pNewStart->AddFaceNormal(cr3DNormals[pFPB_StartVertex->GetIndexValue()]);
                  pNewStart->Mark(eMarkType);
              }
              if (!pNewEnd->IsMarked(eMarkType))
              {
                  pNewEnd->AddFaceToList(pNewPFace);
                  pNewEnd->AddFaceNormal(cr3DNormals[pFPB_EndVertex->GetIndexValue()]);
                  pNewEnd->Mark(eMarkType);
              }

              // skip degenerate 3D NewPolyEdges with StartPVertex == EndPVertex - legally happens when PolyLoop runs through a Surface singularity pole
              if (pNewStart == pNewEnd)
                {
                  // SM_ASSERT_MSG(pNewStart != pNewEnd, _T("OutputToPolyBrep: Bad PEdge with StartPVertex == EndPVertex")) ;
                  continue;
                }

              // FPB_PolyEdge->PolyVertices3D->Point3D
              SmPoint3d sP1 = pNewStart->GetPoint();
              SmPoint3d sP2 = pNewEnd->GetPoint();

              // gwc: we could move this upcoming short PolyEdge block to AddPolyEdge()
              //      so that all PolyLoops are made to skip short edges in the same manner.

              // Skip zero length PolyEdges in 3d - happens legally when UV tessellations map through a Surface Singularity.
              if (sP1.DistanceBetween(sP2) < dTolerance/100.0)
                {
#ifdef SM_DEBUG_CODE
                  SmPoint2d sUV1 = crUVPoints[pFPB_StartVertex->GetIndexValue()];
                  SmPoint2d sUV2 = crUVPoints[pFPB_EndVertex->GetIndexValue()];
                  SM_ASSERT_MSG(!sUV1.CloserThan(SM_EFF_ZERO, sUV2), _T("OutputToPolyBrep found Coincident UVPoints PolyVertices")) ;
                  //SmSurface *pSurf = pFace ? pFace->GetSurface() : NULL ;
                  //SmSurfParamType eSingularDir1, eSingularDir2 ;
                  //SmBoolean bSingular1 = pSurf->IsSingularity(sUV1, eSingularDir1, dTolerance/100.0) ;
                  //SmBoolean bSingular2 = pSurf->IsSingularity(sUV2, eSingularDir2, dTolerance/100.0) ;
              //    SM_ASSERT_MSG(bSingular1 && bSingular2, _T("Warning: OutputToPolyBrep found Coincident 3dPoints not on a SurfSingularity - probably too small triangles"));
#endif // SM_DEBUG_CODE

                  // tell the PolyLoop to merge its last PolyEdge with this short one by
                  // setting the PolyLoop's LastEndPoint and LastPolyVertex to this short edge's last data
                  pNewPLoop->SetLastEndPoint(sP2) ;
                  pNewPLoop->SetLastEndPolyVertex(pNewEnd) ;
                  continue;
                }

              SmTArray<SmPolyVertex *> sDeletedVerts;
              SmTArray<SmPolyVertex *> sSurvivingVerts;

              // Add Edge to pNewFace->pNewPLoop
              SER(pNewPLoop->AddPolyEdge(dTolerance,            // in : min dist between distinct points
                                         sP1,                   // in : Line start position
                                         sP2,                   // in : Line end position
                                         pNewStart,             // in : when m_pLastEndPolyVertex NotNULL (set on last call through pOptEndPolyVertex),
                                                                //           m_pLastEndPolyVertex is Start PolyVertex for PolyLoop->PolyEdge
                                                                //      else: pOptStartPolyVertex NotNULL = Start PolyVertex for PolyLoop->PolyEdge,
                                                                //                                NULL    = create New PolyVertex for 1stPolyEdge
                                         pNewEnd,               // in : NotNULL = stored in m_pLastEndPolyVertex to be
                                                                //                Start PolyVertex for next AddPolyEdge() call.
                                         m_p3DPolyBrep,         // in : provides context for new obj construction and
                                                                //      accumulates new PolyVertices on its m_pVertexListHead list
                                         pNewPEdge,             // out: new edge, stitched to radial partners when pOptStartPolyVertex and pOptEndPolyVertex are NotNULL
                                         NULL,
                                         NULL,
                                         NULL,
                                         NULL,
                                         &sDeletedVerts,        // out: vertices deleted when gluing, to clean up stale pointers.
                                         &sSurvivingVerts));    // out: vertices surviving after gluing, to clean up stale pointers.

              // Addressing potential stale pointers from gluing:
              for (ULONG ll = 0; ll < sDeletedVerts.GetSize(); ll++) // for each deleted polyvertex
              {
                  SmTArray<ULONG> sIndices;
                  if (sNewVertices.FindElements(sDeletedVerts[ll], sIndices)) // find all instances in the array
                  {
                      for (ULONG mm = 0; mm < sIndices.GetSize(); mm++)
                      {
                          sNewVertices.SetAt(sIndices[mm], sSurvivingVerts[ll]); // and replace with the corresponding surviving polyvertex
                      } // end replacing
                  } // end finding all deleted polyverts in the list of new vertices
              } // end each deleted polyvertex

              // Set pNewEdge->pAuxData - this increments the pAuxData use count
              pNewPEdge->SetAuxData(sNewAuxData[pFPB_StartVertex->GetIndexValue()]);

              // propagate PolyFace data as needed
              pNewPEdge->SetOriginalEdgeuse(pFPB_PolyEdge->GetOriginalEdgeuse(), TRUE) ; // TRUE = copy pOriginalFace->Attributes onto pNewPFace

#ifdef SM_DEBUG_CODE
              SM_ASSERT_BREAK(pNewPEdge->GetStartPolyVertex() != NULL);
#endif

              lEdgesCreated ++;
            } // end loop on all Edges in this Loop

          // finish pNewPLoop by making sure its closed and the first and last pPolyEdges share a common PolyVertex
          SmStatus eStat = pNewPLoop->FinishPolyEdgeLoop();

          if ( eStat != SM_SUCCESS )
          {
              sDelLoops.Add(pNewPLoop);
          }

        } // end iter all Loops in this Face

      // remember PolyFace constructions for degenerate polygons - they get deleted later on.
      if (lEdgesCreated < 3)
        {
          // GWC: Checking for degenerate polygons is needed here - just awkward.
          //      When a UV triangle is made with an edge running along a surface singularity the above code
          //      will notice that the edge's 3dLength is zero and will output that as a two PolyEdge degenerate polygon.
          //      When working with cones this happens often.
          //      Seems to gwc that we should just skip creating the face here and save the trouble of removing it later on.
          //      But this seems to work so leave the code as is.
          sDelFaces.Add(pNewPFace);
        }

#ifdef SM_DEBUG_CODE
      if (bDebugMe)
        {
          SM_PTR_ARRAY(sLoopuses, SmLoopuse, 16) ;
          SmSurface *pSurface = pFace->GetSurface() ;
          pFace->GetUpwardLoopuses(sLoopuses) ;
          SmLoopuse *pLoopuse = sLoopuses[0] ;

          SM_ASSERT_VALID(pSurface) ;

          smgfx_Erase();
          smgfx_SetLook(1,2) ;        if(m_p3DPolyBrep) m_p3DPolyBrep->Draw(); sm_GraphicsLoop();
          smgfx_SetLook(1,2, 0,1,1) ; if(pSurface) pSurface->DrawUV(8,8, FALSE, NULL, TRUE) ; sm_GraphicsLoop() ;
          smgfx_SetLook(1,2, 0,1,1) ; if(pSurface) pSurface->DrawPolygon() ; sm_GraphicsLoop() ;
          smgfx_SetLook(5,6, 1,0,0) ; if(pSurface) pSurface->DrawSeams() ; sm_GraphicsLoop() ;
          smgfx_SetLook(5,6, 1,0,0) ; if(pSurface) pSurface->DrawPoles() ; sm_GraphicsLoop() ;
          smgfx_SetLook(2,3, 0,0,1) ; if(pLoopuse) pLoopuse->Draw(FALSE) ; sm_GraphicsLoop() ;
          sm_GraphicsLoop();

          smgfx_SetLook(1,2, 0,0,0) ; if(pSC) pSC->DrawSubdivision2D() ; sm_GraphicsLoop() ;
          smgfx_SetLook(1,2, 0,0,0) ; if(pSC) pSC->DrawTessellation2D() ; sm_GraphicsLoop() ;

          smgfx_SetLook(1,2, 0,0,0) ; if(pSC) pSC->DrawSubdivision3D() ; sm_GraphicsLoop() ;
          smgfx_SetLook(1,2, 0,0,0) ; if(pSC) pSC->DrawTessellation3D() ; sm_GraphicsLoop() ;
          sm_GraphicsLoop();
        }
#endif // SM_DEBUG_CODE

      if(ppOptPolyShell) { *ppOptPolyShell = pNewPFace->GetPolyShell() ; }

    } // end iter ii, every FacePolyBrep->PolyFace

  // Collect this face's standalone PolyVertices in list order; earlier faces leave none, so no
  // need to scan all of m_p3DPolyBrep (quadratic). FindElement skips glued duplicates.
  for(ii=0; ii<lNumAllVerts; ii++)
    {
      SmPolyVertex *p3DPolyVertex = sNewVertices[sFPB_PolyVertices[ii]->GetIndexValue()];
      ULONG         lFoundIndex ;
      if (   p3DPolyVertex->m_pPolyEdgeList == NULL
          && !s3DPolyVertices.FindElement(p3DPolyVertex, lFoundIndex))
        {
          s3DPolyVertices.Add(p3DPolyVertex);
        }
    }

  ULONG lNumDelLoops = sDelLoops.GetSize();
  for (ii = 0; ii < lNumDelLoops; ii++)
  {
      m_p3DPolyBrep->DeletePolyLoop(sDelLoops[ii]);
  }

  // Delete the faces that were allocated but didn't get enough edges.
  ULONG lNumDelFaces = sDelFaces.GetSize();
  for(ii=0; ii<lNumDelFaces; ii++)
    {
      SER(m_p3DPolyBrep->DeletePolyFace(sDelFaces[ii]));
    }

  // delete standalone vertices
  ULONG lNumPVerts = s3DPolyVertices.GetSize();
  for(ii=0; ii<lNumPVerts; ii++)
    {
      SER(m_p3DPolyBrep->DeleteStandalonePolyVertex(s3DPolyVertices[ii]));
    }

  // delete all New AuxData objects that didn't get used
  ULONG lNumNewAux = sNewAuxData.GetSize();
  for(ii=0; ii<lNumNewAux; ii++)
    {
      sNewAuxData[ii] = sNewAuxData[ii]->DeleteIfNoUsers();
    }

  // all done
  return SM_SUCCESS;

} // end SmTess::OutputToPolyBrep

/*******************************************************************//**
PURPOSE: Create or Get m_p3DPolyBrep->PolyVertex for input pVertexIn2D

NOTES:  1. returns a PolyVertex3d from m_p3DPolyBrep
             PolyVertex3d =  pVertexIn2D->GetVertex()->GetPolyVertex3d()
                           ? pVertexIn2D->GetVertex()->GetPolyVertex3d()
                           : a new PolyVertex3d inserted into m_p3DPolyBrep

        2. if(pVertexIn2D->GetVertex()) stores PolyVertex3d in
           pVertexIn2D->GetVertex()->SetPolyVertex3d(PolyVertex3d)

        3. return an associated new SmPolyVertAuxData object
             whether PolyVertex is new or already exists.

        4. pVertexIn2D is assumed to have a valid IndexValue used to access
             info within the input PolyVertex data arrays.
***********************************************************************/
SmStatus SmTess::GetOrCreatePolyVertex
 (const SmPolyVertex         * pVertexIn2D,         // in : PolyVertex to map to m_p3DPolyBrep PolyVertices
  const SmTArray<SmPoint3d>  & cr3DPoints,          // in : assoc 3dPoint    in cr3DPoints    [pVertexIn2D->m_lIndexValue]
  const SmTArray<SmPoint2d>  & crUVPoints,          // in : assoc UVPoint    in crUVPoints    [pVertexIn2D->m_lIndexValue]
  const SmTArray<SmVector3d> & cr3DNormals,         // in : assoc 3dNormal   in cr3DNormals   [pVertexIn2D->m_lIndexValue]
  const SmTArray<SmVertex*>  & crBrepVertices,      // NotUsed: in : assoc BrepVertex in crBrepVertices[pVertexIn2D->m_lIndexValue]
  SmPolyVertex              *& rpNew3DVertex,       // out: PolyVertex mapped to pVertexIn2D->BrepVertex in m_pBrepToPolyBrepMap
  SmPolyVertAuxData         *& rpNewAuxData)        // out: associated Aux Data (UVPnt & Normal - to be attached to appropriate SmPolyEdge)
{
  SM_REF1(crBrepVertices) ; 
  // locals
  const SmContext * cpContext   = m_p3DPolyBrep->GetContext();
  ULONG             lIndex      = pVertexIn2D->GetIndexValue() ;
  SmVertex        * pOrigVertex = pVertexIn2D->GetVertex() ;
  SmPolyVertex    * pVertexIn3D = pOrigVertex ? pOrigVertex->GetPolyVertex() : NULL ;

  // get/create rpNew3DVertex
  if(pVertexIn3D) { rpNew3DVertex = pVertexIn3D ; }
  else            { rpNew3DVertex = new (m_p3DPolyBrep) SmPolyVertex(cr3DPoints[lIndex],
                                                                      m_p3DPolyBrep->GetTolerance());
                    // add NewPolyV to m_p3DPolyBrep Vertex list
                    m_p3DPolyBrep->m_pPolyVertexListHead->PostInsert(rpNew3DVertex);

                    // remember the PolyVertex <=> Vertex association
                    if(pOrigVertex)
                      {
                        // gwc: one Vertex maps to many PolyVertices in 2d space.
                        //      That Vertex maps to one PolyVertex in 3d space.
                        //      If all is well, we set pOrigVertex->m_pPolyVertex3D just one time
                        SM_ASSERT_MSG(pOrigVertex->GetPolyVertex() == NULL, _T("SmTess::GetOrCreatePolyVertex - Setting Vertex->3DPolyVertex when current value is not NULL")) ;
                        pOrigVertex->SetPolyVertex(rpNew3DVertex) ;
                        rpNew3DVertex->SetOriginalVertex(pOrigVertex, TRUE) ; // and propagate attributes if possible
                      }
                  }

  // Now create the Auxilary data and store UV and Normal values
  rpNewAuxData            = new (*cpContext) SmPolyVertAuxData(); NER(rpNewAuxData);
  rpNewAuxData->m_vNormal = cr3DNormals[lIndex];
  rpNewAuxData->m_vUV     = crUVPoints [lIndex];

  // all done
  return SM_SUCCESS;


// OBSOLETE
//
//
//    // input VertexIn2D IndexValue - must be valid
//    ULONG lIndex = pVertexIn2D->GetIndexValue();
//    SM_ASSERT_BREAK(lIndex != SM_UNDEF_ULONG) ;
//
//    // locals
//    const SmContext * cpContext  = m_p3DPolyBrep->GetContext();
//    SmVertex        * pBrepV     = crBrepVertices[lIndex];
//    SmPolyVertex    * pNewPolyV  = NULL;
//
//    // seek existing PolyVertex for target BrepVertex
//    if (pBrepV != NULL)
//      {
//        pNewPolyV = (SmPolyVertex*)m_pBrepToPolyBrepMap->GetValueAt(pBrepV);
//      }
//
//    // when mapped PolyVertex does NOT exist
//    if (pNewPolyV == NULL)
//      {
//        // Create Vertex which is the 3D point
//        pNewPolyV = new (m_p3DPolyBrep) SmPolyVertex(cr3DPoints[lIndex],
//                                                     m_p3DPolyBrep->GetTolerance());
//        NER(pNewPolyV);
//
//        // add NewPolyV to m_p3DPolyBrep Vertex list
//        m_p3DPolyBrep->m_pVertexListHead->PostInsert(pNewPolyV);
//
//        // when pBrepV exists - map it to pNewPolyV
//        if (pBrepV)
//          {
//            m_pBrepToPolyBrepMap->SetAt(pBrepV,pNewPolyV);
//          }
//      } // end need to create pNewPolyV check
//
//    // Now create the Auxilary data and store UV and Normal values
//    SmPolyVertAuxData *pAux = new (*cpContext) SmPolyVertAuxData(); NER(pAux);
//    pAux->m_vNormal = cr3DNormals[lIndex];
//    pAux->m_vUV     = crUVPoints[lIndex];
//
//    // set output
//    rpNew3DVertex = pNewPolyV;
//    rpNewAuxData  = pAux;
//
//    // all done
//    return SM_SUCCESS;
//
} // end SmTess::GetOrCreatePolyVertex

/*******************************************************************//**
PURPOSE: OutputFacePolygons() helper function to
     change Surface Normals for very small PolyFaces with bad Surface Normals

NOTES: Small PolyFaces are likely to happen near difficult surface
  locations (near a pole on a surface with some bad control points [B367])

  When the PolyFace is small compared to the face
    return SM_ERR when PolyFace is so small or too much of a sliver face to compute
                   a PolyFace normal directly from the rPoints array.
    else
      when the angles between the Normal vectors at the PolyVertices
      and the PolyFace Normal are large,
         replace Vertex Normals with PolyFace normal

    rerturn SM_SUCCESS.

    This is no longer used because it is believed that the reason it was
    written is no longer applicable. [B267] and [B267] do not seem related.
    prog_test however does hit this code. That test case actually performed
    better without this code.
***********************************************************************/
SmStatus sms_ReviewPolyFaceNormals
 (SmExtent3d           & rFaceBBox, // in : BBox for SmFace being tessellated (used to determine when this PolyFace is small)
  SmTArray<SmPoint3d>  & rPoints,   // in : PolyFace Vertex positions in CCW order
  SmTArray<SmVector3d> & rNormals)  // in : associated PolyFace Vertex normals
{
  // locals
  ULONG ii ;
  ULONG lNumPts = rPoints.GetSize() ;
  SM_ASSERT(rNormals.GetSize() == lNumPts) ;

  // PolyFace BBox
  SmExtent3d sPolyBBox ;
  for(ii=0;ii<lNumPts;ii++)
    { sPolyBBox.AddPoint3d(rPoints[ii]) ; }

  // BBox sizes
  SmVector3d sPolySize = sPolyBBox.GetSize() ;
  SmVector3d sFaceSize = rFaceBBox.GetSize() ;

  // When PolyFace is small compared to Face in any dimension - small and sliver polygons
  if(   sPolySize.x < sFaceSize.x/12.0    // <== magic number!
     || sPolySize.y < sFaceSize.y/12.0
     || sPolySize.z < sFaceSize.z/12.0)
    {
      // check the changes in PolyVertex normals
      // For very small PolyFaces, expect small changes at normals unless
      //  the surface is poorly defined at this location.
      //  ex: near a pole with badly positioned control points [B267]

      // locals
      SmVector3d sNorm(0,0,0), sThisNorm ;
      double     dMinNormCosAng = 1.0 ;        // 1 = parallel, 0 = perp, -1 = anti-parallel
      double     dLimitNormCosAng = .9063078 ; // <== Magic Number: .9396926 = smos_Cos(SM_DEG2RAD(20))
                                               //                   .9063078 = smos_Cos(SM_DEG2RAD(25))
                                               //                   .8660254 = smos_Cos(SM_DEG2RAD(30))

      // find best guess for PolyFace normal
      for(ii=1;ii<lNumPts-1;ii++)
        {
          // Best guess for PolyFace Normal based on PolyEdge CrossProducts at PolyFace Vertices
          sThisNorm = (rPoints[ii+1] - rPoints[ii]) * (rPoints[ii-1] - rPoints[ii]) ;
          if(sNorm.GetMaxDimension() < sThisNorm.GetMaxDimension())
            {
              sNorm = sThisNorm ;
            }
        } // end iter every Point computing PolyFaceNormal

      // unitize the normals
      if(SM_SUCCESS != sNorm.Unitize())
        {
          return(SM_ERR) ;   // sErr when sNormSq < SM_EFF_ZERO_SQ (noise for very small PolyFace)
                           // too small to fix
        }

      // largest deviation between neighbor PolyVertices and Polygon Normal
      for(ii=0;ii<lNumPts && dMinNormCosAng > dLimitNormCosAng;ii++)
        {
          // Max Angle between normals at neighbor PolyVerts
          double dDotNeighbors = rNormals[ii].Dot(rNormals[(ii+1)%lNumPts]) ;
          double dDotNorm      = rNormals[ii].Dot(sNorm) ;
          if(dDotNeighbors < dMinNormCosAng) { dMinNormCosAng = dDotNeighbors ; }
          if(dDotNorm      < dMinNormCosAng) { dMinNormCosAng = dDotNorm ; }

        } // end iter every Point computing NormalAngs

      // when largest angle between PolyVertex normals is large
      if(dMinNormCosAng <= dLimitNormCosAng)  // <== Magic Number: .9396926 = smos_Cos(SM_DEG2RAD(20))
        {                                     //                   .9063078 = smos_Cos(SM_DEG2RAD(25))
                                              //                   .8660254 = smos_Cos(SM_DEG2RAD(30))

          // reset PolyVertex normals
          for(ii=0;ii<lNumPts;ii++)
            { rNormals[ii] = sNorm ; }

        } // end large deviation in PolyVertex normal direction check
    } // end PolyFace is small compared to Face check

  // all done
  return(SM_SUCCESS) ; // PolyFace was large enough to compute a PolyFace normal

} // end sms_ReviewPolyFaceNormals

/*******************************************************************//**
PURPOSE: Output polygons for this face through the polygon output
         callback class.

NOTES: 0. increments an unlocked mark value
       1. The Polygons of pFace->Surface->TessSrfCache->mTS_pPolyBrep
          are output in various styles depending on the
          rPolygonOutput.GetOutputType() value.
            SM_PO_CREATE_POLYBREP,              // Output creates a 3-D SmPolyBrep in SmTess::m_p3DPolyBrep
            SM_PO_TRIANGLES,                    // Output only triangles.
            SM_PO_QUADRALATERALS,               // Output quadralaterals when two adjacent
                                                //   coplanar polygons are found that border with their longest edge.
                                                //   Otherwise output triangles when these conditions are not meet.
            SM_PO_TRIANGLE_STRIPS,              // Output triangle strips (see OpenGL) when possible (see m_lMinFanTriangles)
            SM_PO_TRIANGLE_FANS,                // Output triangle fans (see OpenGL) when possible.
            SM_PO_STRIPS_OR_FANS,               // Output strips when possible and then fans next.
            SM_PO_TRIANGLE_MESH,                // Output a mesh of triangles (indexed set of arrays) for each face.
            SM_PO_ALL_POLYGON_EDGES,            // Output only the polygon edges
            SM_PO_ALL_NON_PLANAR_EDGES,         // Output only non-coplanar edges of a polygon
            SM_PO_BOUNDARY_EDGES,               // Output edges that correspond to trimmed surface boundaries
            SM_PO_BOUNDARY_AND_SILHOUETTE_EDGES // Output boundary edges and silhouette edges of polygons.

       2. auto face reverse: Currently, face polygons are output
          reversed (display backsides) whenever the face's upward
          faceuse is NOT connected to the infinite shell.

          This works well for showing the outsides of manifold objects.
          However, this may not be the appropriate behavior for showing
          internal shells or the inside of outer shells.
          If this is not the behavior you want, inform
          SMS support and an input argument can be added to give
          the caller direct control of the reverse feature.

       3. when outputting to SmTess::m_p3DPolyBrep, after last OutputFacePolygons call,
          call m_p3DPolyBrep->SetOKBackPtrs(FALSE) ;

Example:
  The following order is used:
    1) Triangle Strips
    2) Triangle Fans
    3) Rectangles
    4) Triangles

  increments unlocked mark value
***********************************************************************/
SmStatus SmTess::OutputFacePolygons
 (SmFace                  * pFace,          // in : target face
  SmPolygonOutputCallback & rPolygonOutput, // in : Chooses how and where to output the polygons
  SmPolyShell            ** ppOptPolyShell, // in : only used when rPolygonOutput.GetOutputType() == SM_PO_CREATE_POLYBREP
                                            //        ppOptPolyShell NULL    = every new SmPolyFace gets a new SmPolyShell and SmPolyRegion
                                            //       *ppOptPolyShell NotNULL = owner of any new SmPolyFaces constructed
                                            //       *ppOptPolyShell NULL    = create and save ptr to new PolyShell for New SmPolyFaces of pFace
                                            //      NULL to ignore, default:[NULL]
  SmGfxArraySet           * pOptGfxSet)     // i/o: only used when rPolygonOutput.GetOutputType() != SM_PO_CREATE_POLYBREP
                                            //      used for OpenGL ES 2.0. When given, accumulate GfxVertexArrays not GL calls.
                                            //      NULL to ignore. default:[NULL]
                                            // note: increments an unlocked mark type
{
#ifdef SM_DEBUG_CODE
  // debug locals
    SmBoolean bDebugMe = FALSE;
  // TCHAR sBuff[SM_TBLOCK_SIZE];
#endif // SM_DEBUG_CODE

  // low work - only output a pFace wireframe
  if(   rPolygonOutput.GetOutputType() == SM_PO_ALL_POLYGON_EDGES
     || rPolygonOutput.GetOutputType() == SM_PO_BOUNDARY_EDGES
     || rPolygonOutput.GetOutputType() == SM_PO_BOUNDARY_AND_SILHOUETTE_EDGES
     || rPolygonOutput.GetOutputType() == SM_PO_ALL_NON_PLANAR_EDGES)
    {
      SER(OutputFaceWireframe(pFace, rPolygonOutput, pOptGfxSet)); // note: increments unlocked mark value
      return SM_SUCCESS;
    }

  // locals
  SmStatus         eStat         = SM_SUCCESS ;
  SmTessSrfCache * pSC           = m_vCache.GetSecond(pFace); // fetch the tessellated surface cache for this face from the stored SmRelation
  SmPolyBrep     * pFPB_PolyBrep = pSC ? pSC->mTS_pPolyBrep : NULL ;

  // no work - no polygons for this face
  if ( !pSC )
    { return SM_SUCCESS; }

  // no work error - SmTessSrfCache PolyBrep for this face is missing
  NER(pFPB_PolyBrep);

  // increment and lock an unlocked mark
  SmNewMarkAndLock sLockMark(pFPB_PolyBrep->GetContext()) ;
  SmMarkType       eMarkType = sLockMark.GetMarkType() ;

  // locals
  SmSurface      * pSurf           = pFace->GetSurface();
  SmFaceuse      * pFU             = pFace->GetUpwardFaceuse();
  SmBoolean        bReverse        = (pFU->GetShell()->GetRegion() != pFace->GetBrep()->GetInfiniteRegion() ) ? TRUE : FALSE;
  ULONG            lIndexCnt       = 0 ;
  //SmExtent3d       sFaceBBox ;
  //pFace->CalculateBoundingBox(sFaceBBox) ;

  // local arrays for outputting polygons
  SM_PTR_ARRAY(sFPB_PolyFaces,     SmPolyFace,   256) ; // SmTArray<SmPolyFace *>
  SM_PTR_ARRAY(sFPB_PolyVertices,  SmPolyVertex, 256) ; // SmTArray<SmPolyVertex *>
  SM_PTR_ARRAY(sFPB_PolyFaceEdges, SmPolyEdge,     4) ; // SmTArray<SmPolyEdge *>

  // init PolyVertex data arrays
  SM_OBJ_ARRAY(sPointsUV,     SmPoint2d,  64) ; // SmTArray<SmPoint2d>  global Vertex Data Arrays, indexed:[pPV->GetIndexValue()]
  SM_OBJ_ARRAY(sPoints3D,     SmPoint3d,  64) ; // SmTArray<SmPoint3d>  global Vertex Data Arrays, indexed:[pPV->GetIndexValue()]
  SM_OBJ_ARRAY(sNormals3D,    SmVector3d, 64) ; // SmTArray<SmVector3d> global Vertex Data Arrays, indexed:[pPV->GetIndexValue()]
  SM_PTR_ARRAY(sBrepVertices, SmVertex,   64) ; // SmTArray<SmVertex *> global Vertex Data Arrays, indexed:[pPV->GetIndexValue()]

  SM_OBJ_ARRAY(sUVPoints,     SmPoint2d,  32) ; // SmTArray<SmPoint2d>  - for indiviudal polygons
  SM_OBJ_ARRAY(sPoints,       SmPoint3d,  32) ; // SmTArray<SmPoint3d>  - for indiviudal polygons
  SM_OBJ_ARRAY(sNormals,      SmVector3d, 32) ; // SmTArray<SmVector3d> - for indiviudal polygons

  // get all PolyBrep PolyFaces
  pFPB_PolyBrep->GetPolyFaces(sFPB_PolyFaces);

  // get all PolyBrep PolyVertices
  pFPB_PolyBrep->GetPolyVertices(sFPB_PolyVertices);  // some might be topological vertices on singular Edges
  ULONG lNumPolyVertices = sFPB_PolyVertices.GetSize() ;

  // size the PolyVert data arrays - for performance
  // PolyBrep->PolyVertex arrays - these sizes are known
  sPointsUV.    SetSize(lNumPolyVertices) ;
  sPoints3D.    SetSize(lNumPolyVertices) ;
  sNormals3D.   SetSize(lNumPolyVertices) ;
  sBrepVertices.SetSize(lNumPolyVertices) ;

#ifdef SM_32K_LOCAL_DATA_LIMIT
  if(lNumPolyVertices > 64)
    {
      ULONG lMinSize = smos_Max(lNumPolyVertices/4, 32);
      ULONG lNewSize = smos_Min(lMinSize,256) ;
      // PolyFace->PolyVertex arrays - these sizes are guesses
      sUVPoints.SetDataSize(lNewSize) ; // leave sizes at zero. Fill with array.Add(value)
      sPoints.  SetDataSize(lNewSize) ;
      sNormals. SetDataSize(lNewSize) ;
    }
#else // no SM_32K_LOCAL_DATA_LIMIT
  if(lNumPolyVertices > 64)
    {
      ULONG lNewSize = smos_Min(256, lNumPolyVertices) ;
      // PolyFace->PolyVertex arrays - these sizes are guesses
      sUVPoints.SetDataSize(lNewSize) ; // leave sizes at zero. Fill with array.Add(value)
      sPoints.  SetDataSize(lNewSize) ;
      sNormals. SetDataSize(lNewSize) ;
    }
#endif // no SM_32K_LOCAL_DATA_LIMIT

  // 1. Gather PolyVertData(PointUV, Point3D, Normal3D, BrepVertex)
  //     order the array[WithEdgeuse, Without] - watch out for PolyVerts on Surface degeneracies

  // for every FBP_PolyVertex - init IndexValue assignments for PolyVertices with edgeuse
  for(ULONG ii=0;ii<lNumPolyVertices;ii++)
    {
      SmPolyVertex *pPolyVertex = sFPB_PolyVertices[ii] ;

      // first half of IndexValue assignments for WithEdgeuse PolyVertices
      if(pPolyVertex->GetOriginalEdgeuse()) { pPolyVertex->SetIndexValue(lIndexCnt) ;
                                              lIndexCnt++ ;
                                            }
      else                                  { pPolyVertex->UnInitIndexValue() ; }
    } // end iter PolyVertices assigning indexValues to WithEdgeuse PolyVertices

  // These quantities will be saved in the PolyVertex data arrays - should be a struct
  SmVertex  * pOrigVertex = NULL;
  SmPoint3d   sUV, sPnt;
  SmPoint2d   sUVPnt;
  SmVector3d  sNorm(0,0,0);
  SmEdgeuse * pEdgeuse ;

  // for every sFPB_PolyVertex - finish IndexValue assignments, cache PolyVertex data
  for(ULONG ii=0;ii<lNumPolyVertices;ii++)
    {
      SmPolyVertex *pPolyVertex = sFPB_PolyVertices[ii] ;

      // finish index assignments
      if(!pPolyVertex->IsIndexValueInit()) { pPolyVertex->SetIndexValue(lIndexCnt) ;
                                             lIndexCnt++ ;
                                             pEdgeuse = NULL ;
                                           }
      else                                 { pEdgeuse = pPolyVertex->GetEdgeuse() ; }

      // get PolyVertex UVPnt
      sUV = pPolyVertex->GetPoint();
      sUVPnt.Set( sUV.x, sUV.y );

      // get pOrigVertex and sPnt
      if(pEdgeuse) { // with    EU
                     pOrigVertex = pEdgeuse->GetVertexuse()->GetVertex();
                     sPnt        = pOrigVertex->GetPoint();
                   }
      else         { // without EU
                     if(SM_SUCCESS != pSurf->EvaluatePoint(sUVPnt, sPnt))
                       { continue; } // GWC???: what's the right behavior for a failed EvaluatePoint calculation
                     pOrigVertex = NULL ;
                   }

      // when asked - generate mesh point normals
      if (rPolygonOutput.GetGenerateNormals())
        {
          eStat = pSurf->EvaluateNormal(sUVPnt,TRUE,TRUE,sNorm) ;
          if ( eStat != SM_SUCCESS )
            { SE(eStat); continue; } // GWC???: what's the right behavior for a failed normal calculation

          // when reversing - negate the normal
          if(bReverse)
             { sNorm = -sNorm; }

        } // end asked to generate mesh point normal check

      // save PolyVertex Data in PolyVertex Data Arrays
      ULONG lIndexValue = pPolyVertex->GetIndexValue() ;
      sPoints3D    .SetAt(lIndexValue, sPnt  );       // from Brep Vertex (exterior edges) or from BrepSurfaceEval(sUVPnt) (interior edges)
      sPointsUV    .SetAt(lIndexValue, sUVPnt);       // from face tessSrfCache PolyBrep Edge start value
      sNormals3D   .SetAt(lIndexValue, sNorm );       // from surface evaluation
      sBrepVertices.SetAt(lIndexValue, pOrigVertex ); // from Brep Vertex when available or NULL

    } // end iter every PolyVertex - finishing IndexValue assignments, caching PolyVertex data

#ifdef SM_DEBUG_CODE
  // check all PolyVertices are Indexed
  if(bDebugMe)
    {
      SmBoolean bAllPolyVerticesIndexed = TRUE ;
      ULONG     lBadPVCount     = 0 ;
      SM_PTR_ARRAY(sDebugPolyVertices, SmPolyVertex, 1024) ;
      pFPB_PolyBrep->GetPolyVertices(sDebugPolyVertices) ;
      for(ULONG ii=0;ii<sDebugPolyVertices.GetSize();ii++)
        {
          if(!sDebugPolyVertices[ii]->IsIndexValueInit()) { lBadPVCount += 1 ;
                                                          }
          bAllPolyVerticesIndexed &= sDebugPolyVertices[ii]->IsIndexValueInit() ;
        }
      SM_ASSERT_BREAK(bAllPolyVerticesIndexed) ;
    }
#endif // SM_DEBUG_CODE

  // arrive here with built PolyVertex data lists: sPoints3D, sPointsUV, sNormals3D, and sBrepVertices
  //  each data access for PolyVert is Array[PolyVert->m_lIndexValue]

  // when asked - Add pFace->Surface (SmTessSrfCache)pSC->mTS_pPolyBrep->pPolyTopology objs to m_p3DPolyBrep
  if (rPolygonOutput.GetOutputType() == SM_PO_CREATE_POLYBREP)
    {
      // Tessellate Output - copy PFace->Surface's (SmTessSrfCache)pSC->mTS_pPolyBrep->pPolyTopology objs to m_p3DPolyBrep.
      //   m_p3DPolyBrep->m_bOKBackPtrs == TRUE. During tessellation back ptrs are current.
      //   After, back ptrs may become stale
      SER(OutputToPolyBrep(pFace,
                           rPolygonOutput,
                           ppOptPolyShell,
                           sPoints3D,
                           sPointsUV,
                           sNormals3D,
                           sBrepVertices));

#ifdef SM_DEBUG_CODE
      SmBoolean bDebugMe2 = FALSE;
      if (bDebugMe2)
        {
          smgfx_Erase();
          m_p3DPolyBrep->Draw();
          sm_GraphicsLoop();
        }
#endif // SM_DEBUG_CODE
      return SM_SUCCESS;

    } // end if SM_PO_CREATE_POLYBREP check

  // if you want quads, that should then be the first option to test RCLxx
  // Look for 4 sided polygons which are coplanar.
  if (rPolygonOutput.GetOutputType() == SM_PO_QUADRALATERALS)
    {
      // for every PolyFace in the pFace->Surface->SmTessSrfCache->mTS_pPolyBrep
      for (ULONG ii=0; ii<sFPB_PolyFaces.GetSize(); ii++)
        {
          SmPolyFace *pPolyFace = sFPB_PolyFaces[ii];

          // skip processed faces
          if (pPolyFace->IsMarked(eMarkType))
            { continue; }

          // PolyFace->PolyLoop NonDegenarate PolyEdges in CCW order
          //   From here on, get CCW members from sequences in this array which automatically skips over Degenerate PolyEdges
          //   rather than using the GetCCWPolyEdge() and GetCWPolyEdge() methods.
          pPolyFace->GetNonDegeneratePolyEdges(sFPB_PolyFaceEdges);
          ULONG lNumPolyEdges = sFPB_PolyFaceEdges.GetSize() ;
          // gwc: this used to be
          // pPolyFace->GetPolyEdges(sFPB_PolyFaceEdges);

          // skip not Triangle nor Quad polygons
          if(   lNumPolyEdges != 3
             && lNumPolyEdges != 4)
               { continue; }

          // init vertex data arrays used just for this PolyFace
          sPoints.ReSet();
          sNormals.ReSet();
          sUVPoints.ReSet();

          // when polyFace is a triangle
          if (lNumPolyEdges == 3)
            {
              double dShortestLength = SM_BIG_DOUBLE;
              ULONG  lLongestEdge    = 0;
              double dLongestLength  = 0.0;

              // Find the longest edge of this face and see if there is
              // an adjacent polygon with the same normal.
              for (ULONG j=0; j<sFPB_PolyFaceEdges.GetSize(); j++)
                {
                  SmPolyEdge   * pPEdge        = sFPB_PolyFaceEdges[j];
                  SmPolyVertex * pStartV       = pPEdge->GetStartPolyVertex();
                  SmPolyVertex * pEndV         = pPEdge->GetOtherPolyVertex(pStartV);
                  SmVector3d     sEdgeVec      =   sPoints3D[ pEndV  ->GetIndexValue() ]
                                                 - sPoints3D[ pStartV->GetIndexValue() ];
                  double         dEdgeLengthSq = sEdgeVec.LengthSquared();

                  // save longest triangle side
                  if (dEdgeLengthSq > dLongestLength)
                    {
                      dLongestLength = dEdgeLengthSq;
                      lLongestEdge = j;
                    }

                  // save shortest triangle side
                  if (dEdgeLengthSq < dShortestLength)
                    {
                      dShortestLength = dEdgeLengthSq;
                    }
                } // end iter triangle sides - saving longest and shortest edge lengths

              // Skip polygons with zero length edges
              if (dShortestLength < SM_EFF_ZERO_SQ)
                { continue; }

              SmPolyEdge *pLongEdge     = sFPB_PolyFaceEdges[lLongestEdge];
              SmPolyEdge *pLongEdgeCCW1 = sFPB_PolyFaceEdges[(lLongestEdge+1)%lNumPolyEdges];
              SmPolyEdge *pLongEdgeCCW2 = sFPB_PolyFaceEdges[(lLongestEdge+2)%lNumPolyEdges];

              // skip lamina edges
              if (pLongEdge->IsLamina())
                 { continue; }

              // Put the 3 points into the arrays in such a way
              // that we can add the 4th point.
                {
                  // SmPolyEdge   * pCCW   = pLongEdge->GetCCWPolyEdge();
                  // SmPolyVertex * pHeadV = pCCW->GetStartPolyVertex();
                  SmPolyVertex * pHeadV = pLongEdgeCCW1->GetStartPolyVertex();

                  // 1st vertex data
                  sNormals. Add(sNormals3D[pHeadV->GetIndexValue()]);
                  sPoints.  Add(sPoints3D [pHeadV->GetIndexValue()]);
                  sUVPoints.Add(sPointsUV [pHeadV->GetIndexValue()]);

                  // 2nd vertex data
                  // pCCW   = pCCW->GetCCWPolyEdge();
                  // pHeadV = pCCW->GetStartPolyVertex();
                  pHeadV = pLongEdgeCCW2->GetStartPolyVertex();

                  sNormals. Add(sNormals3D[pHeadV->GetIndexValue()]);
                  sPoints.  Add(sPoints3D [pHeadV->GetIndexValue()]);
                  sUVPoints.Add(sPointsUV [pHeadV->GetIndexValue()]);

                  // 3rd vertex
                  // pCCW   = pCCW->GetCCWPolyEdge();
                  // pHeadV = pCCW->GetStartPolyVertex();
                  pHeadV = pLongEdge->GetStartPolyVertex();

                  sNormals. Add(sNormals3D[pHeadV->GetIndexValue()]);
                  sPoints.  Add(sPoints3D [pHeadV->GetIndexValue()]);
                  sUVPoints.Add(sPointsUV [pHeadV->GetIndexValue()]);
                }

              // if neighborFace to longest edge is coPlanar to current triangle
              // treat the 2 faces as a single Quad element
                {
                  SmPolyEdge   * pRadialEdge = pLongEdge->GetRadial();
                  // SmPolyEdge   * pCWEdge     = pRadialEdge->GetCWPolyEdge();
                  SmPolyEdge   * pCWEdge     = pRadialEdge->GetCWNonDegeneratePolyEdge();
                  SmPolyVertex * pCWStartV   = pCWEdge->GetStartPolyVertex();
                  SmPoint3d      sCWPoint    = sPoints3D[pCWStartV->GetIndexValue()];
                  SmVector3d     sV1         = sPoints[1] - sPoints[0];
                  SmVector3d     sV2         = sPoints[2] - sPoints[0];
                  SmVector3d     sPlaneNorm  = sV1 * sV2;

                  // skip degenerate triangles
                  if (sPlaneNorm.LengthSquared() < SM_EFF_ZERO_SQ)
                    { continue; }

                  // This test is far too demanding, does not make enough quads RCLxx
                  // double dTol = SM_EFF_ZERO * (1.0 + sCWPoint.GetMaxDimension());
                  double dTol = 0.001 * (1.0 + sCWPoint.GetMaxDimension());

                  SmPoint3d sProjPnt;
                  SER(smgu_PointProjectToPlane(sCWPoint,sPoints[0],sPlaneNorm,sProjPnt));

                  // skip when far point of last triangle is too far from current plane
                  double    dDistSq = sProjPnt.DistanceBetweenSquared(sCWPoint);
                  if (dDistSq > dTol*dTol) continue;

                  // skip when sCWPoint is too close to sPoints[0]
                  dDistSq = sCWPoint.DistanceBetweenSquared(sPoints[0]);
                  if (dDistSq < dTol*dTol) continue;

                  // skip when sCWPoint is too close to sPoints[2].
                  dDistSq = sCWPoint.DistanceBetweenSquared(sPoints[2]);
                  if (dDistSq < dTol*dTol) continue;

                  // If we made it here sCWStartV works as the 4th point of a planar quad with existing triangle
                  pPolyFace->Mark(eMarkType);

                  SmPolyFace * pOtherFace = pCWEdge->GetPolyFace();
                  pOtherFace->Mark(eMarkType);

                  // 4th vertex
                  sNormals.Add(sNormals3D[pCWStartV->GetIndexValue()]);
                  sPoints.Add(sCWPoint);
                  sUVPoints.Add(sPointsUV[pCWStartV->GetIndexValue()]);
                } // end longest edge NeighborFace makes a 2 triangle coplanar Quad check

            } // end PolyFace triangle check
          else // Four sided polygon - build sNormals, sPoints, sUVPoints arrays
            {
              pPolyFace->Mark(eMarkType);

              // 1st vertex
              // SmPolyEdge   * pCCW = sFPB_PolyFaceEdges[0]->GetCCWPolyEdge();
              SmPolyEdge   * pCCW = sFPB_PolyFaceEdges[1] ;
              SmPolyVertex * pHeadV = pCCW->GetStartPolyVertex();
              sNormals. Add(sNormals3D[pHeadV->GetIndexValue()]);
              sPoints.  Add(sPoints3D [pHeadV->GetIndexValue()]);
              sUVPoints.Add(sPointsUV [pHeadV->GetIndexValue()]);

              // 2nd vertex
              // pCCW   = pCCW->GetCCWPolyEdge();
              pCCW   = sFPB_PolyFaceEdges[2] ;
              pHeadV = pCCW->GetStartPolyVertex();
              sNormals. Add(sNormals3D[pHeadV->GetIndexValue()]);
              sPoints.  Add(sPoints3D [pHeadV->GetIndexValue()]);
              sUVPoints.Add(sPointsUV [pHeadV->GetIndexValue()]);

              // 3rd vertex
              // pCCW   = pCCW->GetCCWPolyEdge();
              pCCW   = sFPB_PolyFaceEdges[3] ;
              pHeadV = pCCW->GetStartPolyVertex();
              sNormals. Add(sNormals3D[pHeadV->GetIndexValue()]);
              sPoints.  Add(sPoints3D [pHeadV->GetIndexValue()]);
              sUVPoints.Add(sPointsUV [pHeadV->GetIndexValue()]);

              // 4th vertex
              // pCCW   = pCCW->GetCCWPolyEdge();
              pCCW   = sFPB_PolyFaceEdges[0] ;
              pHeadV = pCCW->GetStartPolyVertex();
              sNormals. Add(sNormals3D[pHeadV->GetIndexValue()]);
              sPoints.  Add(sPoints3D [pHeadV->GetIndexValue()]);
              sUVPoints.Add(sPointsUV [pHeadV->GetIndexValue()]);
            } // end quad branch case - building sNormals, sPoints, sUVPoints arrays

          if (bReverse)
            {
              sPoints.ReverseArray(0,sPoints.GetSize());
              sNormals.ReverseArray(0,sNormals.GetSize());
              sUVPoints.ReverseArray(0,sUVPoints.GetSize());
            }

          // Reset Normals for very small PolyFaces with large changes in PolyVertexNormals
          //  These Polygons are likely near difficult surface points with large errors
          //  in the computed surface normals [B267]

          // See comment in sms_ReviewPolyFaceNormals header.

          // if(SM_SUCCESS != sms_ReviewPolyFaceNormals(sFaceBBox, sPoints, sNormals))
          //  {
              // skip PolyFaces too small to fix
          //    continue ;
          //  }

          // Output Quadralateral - massage polygon data depending on draw parameters and
          //                        pass the tuned polygons on to be output to the graphics engine
          SER(rPolygonOutput.OutputPolygon(1,                        // 1 = 4-Sided Planar Polygon
                                           sPoints.GetSize(),
                                           sPoints.GetDataArray(),
                                           sNormals.GetDataArray(),
                                           sUVPoints.GetDataArray(),
                                           pFace->GetSurface(),
                                           pFace,
                                           pOptGfxSet));
        } // end iter sFPB_PolyFaces array
    } // end if OutputType == SM_PO_QUADRALATERALS check

  // output as triangle mesh
  if (rPolygonOutput.GetOutputType() == SM_PO_TRIANGLE_MESH)
    {
      // Output all remaining unmarked faces as triangles
      SM_OBJ_ARRAY(sPolygonVertexCounts,   ULONG, 512) ; // SmTArray<ULONG> = number of vertices in each polygon
      SM_OBJ_ARRAY(sPolygonVertexIndicies, ULONG, 512) ; // SmTArray<ULONG> = accumulated list of global vertex indices - PolyVertex->m_lIndexValue
      SM_OBJ_ARRAY(sInitialIndicies,       ULONG,  32) ; // SmTArray<ULONG> = one polygon's list of global vertex indices

      // for every PolyFace
      for (ULONG ii=0; ii<sFPB_PolyFaces.GetSize(); ii++)
        {
          SmPolyFace *pPFace = sFPB_PolyFaces[ii];

          // skipped processed faces - mark the others
          if (pPFace->IsMarked(eMarkType))
            { continue; }
          pFace->Mark(eMarkType);

          // get this PolyFace->PolyEdges
          pPFace->GetNonDegeneratePolyEdges(sFPB_PolyFaceEdges);
          // used to be: pPFace->GetPolyEdges(sFPB_PolyFaceEdges);

          // skip degenerate polygons
          if (sFPB_PolyFaceEdges.GetSize() < 3)
            { continue; }

          // skip not triangle polygons
          if (sFPB_PolyFaceEdges.GetSize() > 3)
            { continue; }

          // TODO - something to skip bad triangle - too small, sliver, or mismatched vertex normals
          //  GWC: switching from GetPolyEdge to GetNonDegeneratePolyEdges catches the case of too small and sliver triangles

          // init the next polygon output
          sInitialIndicies.ReSet();
          sPolygonVertexCounts.Add(3);

          // for every polygon vertex - load sInitialArrays with global Index value pPolyVertex->m_lIndexValue
          for (ULONG j=0; j<3; j++)
            {
              SmPolyEdge   * pPEdge  = sFPB_PolyFaceEdges[j];
              SmPolyVertex * pStartV = pPEdge->GetStartPolyVertex();
              sInitialIndicies.Add(pStartV->GetIndexValue());
            }

          // when asked - reverse
          if (bReverse)
            {
              sInitialIndicies.ReverseArray(0,sInitialIndicies.GetSize());
            }

          // accumulate vertex index array
          sPolygonVertexIndicies.Append(sInitialIndicies);

        } // end iter ii, every sFPB_PolyFace

      // Output all Triangles - one triangle at a time
      SER(rPolygonOutput.OutputMesh(sPolygonVertexCounts,
                                    sPolygonVertexIndicies,
                                    sPoints3D,
                                    sNormals3D,
                                    sPointsUV,
                                    pFace->GetSurface(),
                                    pFace,
                                    pOptGfxSet));
      return SM_SUCCESS;

    }  // end output as SM_PO_TRIANGLE_MESH check

  // Now look for Triangle Strips - where polygons lie along a strip
  // with top and bottom alternating.
  if(   rPolygonOutput.GetOutputType() == SM_PO_TRIANGLE_STRIPS
     || rPolygonOutput.GetOutputType() == SM_PO_STRIPS_OR_FANS)
    {
      SM_PTR_ARRAY(sUTracedFaces, SmPolyFace,   256) ; // SmTArray<SmPolyFace *>
      SM_PTR_ARRAY(sVTracedFaces, SmPolyFace,   256) ; // SmTArray<SmPolyFace *>
      SM_PTR_ARRAY(sBotVerts,     SmPolyVertex, 256) ; // SmTArray<SmPolyVertex *>
      SM_PTR_ARRAY(sTopVerts,     SmPolyVertex, 256) ; // SmTArray<SmPolyVertex *>
      SM_PTR_ARRAY(sLeftVerts,    SmPolyVertex, 256) ; // SmTArray<SmPolyVertex *>
      SM_PTR_ARRAY(sRightVerts,   SmPolyVertex, 256) ; // SmTArray<SmPolyVertex *>

      // for 2 tries
      for (ULONG lTries = 0; lTries < 2; lTries ++)
        {
          // for every PolyFace
          for (ULONG ii=0; ii<sFPB_PolyFaces.GetSize(); ii++)
            {
              SmPolyFace *pPFace = sFPB_PolyFaces[ii];

              // skip processed faces
              if (pPFace->IsMarked(eMarkType))
                { continue; }

              // Get PolyFace->PolyEdges
              pPFace->GetNonDegeneratePolyEdges(sFPB_PolyFaceEdges);
              // used to be: pPFace->GetPolyEdges(sFPB_PolyFaceEdges);

              // Skip not triangle polygons
              if (sFPB_PolyFaceEdges.GetSize() != 3)
                { continue; }

              // init polygon data arrays
              sPoints.ReSet();
              sNormals.ReSet();
              sUVPoints.ReSet();

              // seek a lamina edge in the current triangle
              SmPolyEdge *pLamina     = NULL;

              if (lTries == 0)
                {
                  for (ULONG jj=0; jj<sFPB_PolyFaceEdges.GetSize(); jj++)
                    {
                      SmPolyEdge *pPEdge = sFPB_PolyFaceEdges[jj];
                      if (pPEdge->IsLamina())
                        {
                          pLamina     = pPEdge;
                        }
                    }
                }

              // The first time around only check lamina edges.
              if (lTries == 0 && !pLamina)
                { continue; }

              // look for sequence of Triangles that form a sequence of axis aligned quad elements
              SER(TraceQuads(SM_SP_U, pPFace, FALSE, sUTracedFaces, sTopVerts,  sBotVerts,   eMarkType));  // note: checks without incrementing or assigning eMarkType value
              SER(TraceQuads(SM_SP_V, pPFace, FALSE, sVTracedFaces, sLeftVerts, sRightVerts, eMarkType));  // note: checks without incrementing or assigning eMarkType value

              // skip degenerate strips
              if (sTopVerts.GetSize() < 3 && sLeftVerts.GetSize() < 3)
                { continue; }

              // When UTraced faces is bigger than VTraced faces
              if (sUTracedFaces.GetSize() > sVTracedFaces.GetSize())
                {
                  // mark the UTraced faces as processed
                  for (ULONG jj=0; jj<sUTracedFaces.GetSize(); jj++)
                    {
                      sUTracedFaces[jj]->Mark(eMarkType);
                    }

                  // move the traced strip vertics to the local vertex data arrays
                  for (ULONG jj=0; jj<sTopVerts.GetSize(); jj++)
                    {
                      SmPolyVertex *pTopV = sTopVerts[jj];
                      SmPolyVertex *pBotV = sBotVerts[jj];

                      if (bReverse)
                        {
                          sNormals. Add(sNormals3D[pTopV->GetIndexValue()]);
                          sPoints.  Add(sPoints3D [pTopV->GetIndexValue()]);
                          sUVPoints.Add(sPointsUV [pTopV->GetIndexValue()]);

                          sNormals. Add(sNormals3D[pBotV->GetIndexValue()]);
                          sPoints.  Add(sPoints3D [pBotV->GetIndexValue()]);
                          sUVPoints.Add(sPointsUV [pBotV->GetIndexValue()]);
                        }
                      else
                        {
                          sNormals. Add(sNormals3D[pBotV->GetIndexValue()]);
                          sPoints.  Add(sPoints3D [pBotV->GetIndexValue()]);
                          sUVPoints.Add(sPointsUV [pBotV->GetIndexValue()]);

                          sNormals. Add(sNormals3D[pTopV->GetIndexValue()]);
                          sPoints.  Add(sPoints3D [pTopV->GetIndexValue()]);
                          sUVPoints.Add(sPointsUV[ pTopV->GetIndexValue()]);
                        }
                    }
                }
              else // When VTraced faces is bigger than UTraced faces branch
                {
                  // mark all VTraced faces as processed
                  for (ULONG jj=0; jj<sVTracedFaces.GetSize(); jj++)
                    {
                      sVTracedFaces[jj]->Mark(eMarkType);
                    }

                  // move the traced strip vertex data into the compressed vertex data arrays
                  for (ULONG jj=0; jj<sLeftVerts.GetSize(); jj++)
                    {
                      SmPolyVertex *pLeftV  = sLeftVerts[jj];
                      SmPolyVertex *pRightV = sRightVerts[jj];

                      if (bReverse)
                        {
                          sNormals. Add(sNormals3D[pRightV->GetIndexValue()]);
                          sPoints.  Add(sPoints3D [pRightV->GetIndexValue()]);
                          sUVPoints.Add(sPointsUV [pRightV->GetIndexValue()]);

                          sNormals. Add(sNormals3D[pLeftV->GetIndexValue()]);
                          sPoints.  Add(sPoints3D [pLeftV->GetIndexValue()]);
                          sUVPoints.Add(sPointsUV [pLeftV->GetIndexValue()]);
                        }
                      else
                        {
                          sNormals. Add(sNormals3D[pLeftV->GetIndexValue()]);
                          sPoints.  Add(sPoints3D [pLeftV->GetIndexValue()]);
                          sUVPoints.Add(sPointsUV [pLeftV->GetIndexValue()]);

                          sNormals. Add(sNormals3D[pRightV->GetIndexValue()]);
                          sPoints.  Add(sPoints3D [pRightV->GetIndexValue()]);
                          sUVPoints.Add(sPointsUV [pRightV->GetIndexValue()]);
                        }
                    }
                } // end VTraced faces is bigger than UTraced faces branch

              // TODO - something to skip bad triangle - too small, sliver, or mismatched vertex normals
              // GWC: switching from GetPolyEdges to GetNonDegeneratePolyEdges() should eliminate
              //      several potential degenerate triangle cases

              // Output Triangle Strip
              SER(rPolygonOutput.OutputPolygon(4,
                                               sPoints.GetSize(),
                                               sPoints.GetDataArray(),
                                               sNormals.GetDataArray(),
                                               sUVPoints.GetDataArray(),
                                               pFace->GetSurface(),
                                               pFace,
                                               pOptGfxSet));
            } // end iter every PolyFace
        } // end iter 2 lTries
    } // end SM_PO_TRIANGLE_STRIPS || SM_PO_STRIPS_OR_FANS) check

  // when asked - Look for Triangle Fans - where a bunch of triangles come into
  // a single vertex.
  if(   rPolygonOutput.GetOutputType() == SM_PO_TRIANGLE_FANS
     || rPolygonOutput.GetOutputType() == SM_PO_STRIPS_OR_FANS)
    {
      SM_PTR_ARRAY(sFanEdges, SmPolyEdge, 64) ; // SmTArray<SmPolyEdge *>

      ULONG lMinTriangles = rPolygonOutput.GetMinFanTriangles();

      // for every PolyFace
      for (ULONG ii=0; ii<sFPB_PolyFaces.GetSize(); ii++)
        {
          SmPolyFace *pPFace        = sFPB_PolyFaces[ii];
          ULONG       lBestEdge     = 0;
          ULONG       lMostFanEdges = 0;

          // skip processed faces
          if (pPFace->IsMarked(eMarkType))
            { continue; }

          // get PolyFace->PolyEdges
          pPFace->GetNonDegeneratePolyEdges(sFPB_PolyFaceEdges);
          // used to be: pPFace->GetPolyEdges(sFPB_PolyFaceEdges);

          // skip not triangle polygons
          if (sFPB_PolyFaceEdges.GetSize() != 3)
            { continue; }

          // for every PolyEdge
          for (ULONG jj=0; jj<sFPB_PolyFaceEdges.GetSize(); jj++)
            {
              SmPolyEdge *pPEdge = sFPB_PolyFaceEdges[jj];

              // Get the fan
              SER(sm_GetFan(pPEdge,sFanEdges,eMarkType));  // note: uses without incrementing or assigning eMarkType value

              // save the largest fan
              if (sFanEdges.GetSize() > lMostFanEdges)
                {
                  lBestEdge     = jj;
                  lMostFanEdges = sFanEdges.GetSize();
                }
            } // end iter every PolyEdge

          // when the largest fan is big enough to bother
          if (lMostFanEdges > lMinTriangles + 1)
            {
              // regen the fan - yuk
              SER(sm_GetFan(sFPB_PolyFaceEdges[lBestEdge],sFanEdges,eMarkType)); // note: uses without incrementing or assigning eMarkType value

              // init local vertex data arrays
              sPoints.ReSet();
              sNormals.ReSet();
              sUVPoints.ReSet();

              // For every Fan Edge
              for (ULONG kk=0; kk<sFanEdges.GetSize(); kk++)
                {
                  SmPolyEdge *pEdge = sFanEdges[kk];

                  SmPolyFace *pEdgeFace = pEdge->GetPolyFace();

                  // mark the face as processed
                  pEdgeFace->Mark(eMarkType);

                  // build the local polygon vertex data arrays
                  SmPolyVertex *pStartV = pEdge->GetStartPolyVertex();
                  sNormals. Add(sNormals3D[pStartV->GetIndexValue()]);
                  sPoints.  Add(sPoints3D [pStartV->GetIndexValue()]);
                  sUVPoints.Add(sPointsUV [pStartV->GetIndexValue()]);
                } // end iter every fan edge

              if (bReverse)
                {
                  sPoints.ReverseArray(1,sPoints.GetSize());
                  sNormals.ReverseArray(1,sNormals.GetSize());
                  sUVPoints.ReverseArray(1,sUVPoints.GetSize());
                }

              // TODO - something to skip bad triangle - too small, sliver, or mismatched vertex normals
              // gwc: switching from GetPolyEdges to GetNonDegeneratePolyEdges() should catch many degenerate triangle cases

              // Output Triangle Fans
              SER(rPolygonOutput.OutputPolygon(3,
                                               sPoints.GetSize(),
                                               sPoints.GetDataArray(),
                                               sNormals.GetDataArray(),
                                               sUVPoints.GetDataArray(),
                                               pFace->GetSurface(),
                                               pFace,
                                               pOptGfxSet));

            } // end largest fan is big enough to bother check
        } // end iter ii, every polyface
    } // end SM_PO_TRIANGLE_FANS || SM_PO_STRIPS_OR_FANS case

  // Output all remaining unmarked faces as triangles
  ULONG lNumFaces = sFPB_PolyFaces.GetSize();

  // for every PolyFace
  for (ULONG ii=0; ii<lNumFaces; ii++)
    {
      SmPolyFace *pPFace = sFPB_PolyFaces[ii];

      // skip processed faces - mark the others
      if (pPFace->IsMarked(eMarkType))
        { continue; }
      pFace->Mark(eMarkType);

      // get PolyFace->PolyEdges
      pPFace->GetNonDegeneratePolyEdges(sFPB_PolyFaceEdges);
      // used to be: pPFace->GetPolyEdges(sFPB_PolyFaceEdges);

      // init local polygon vertex data arrays
      sPoints.ReSet();
      sNormals.ReSet();
      sUVPoints.ReSet();

      // skip degenerate polygons
      if (sFPB_PolyFaceEdges.GetSize() < 3)
        { continue; }

      // for every PolyFace->PolyEdge
      ULONG lNumEdges = sFPB_PolyFaceEdges.GetSize();
      for (ULONG jj=0; jj<lNumEdges; jj++)
        {
          SmPolyEdge   *pPEdge  = sFPB_PolyFaceEdges[jj];
          SmPolyVertex *pStartV = pPEdge->GetStartPolyVertex();

          sNormals. Add(sNormals3D[pStartV->GetIndexValue()]);
          sPoints.  Add(sPoints3D [pStartV->GetIndexValue()]);
          sUVPoints.Add(sPointsUV [pStartV->GetIndexValue()]);
        }

      if (bReverse)
        {
          sPoints.  ReverseArray(0, sPoints.GetSize());
          sNormals. ReverseArray(0, sNormals.GetSize());
          sUVPoints.ReverseArray(0, sUVPoints.GetSize());
        }

      // lType: 0 = Triangle, 1 = Quad
      ULONG lType = sPoints.GetSize() == 3 ? 0 : 1 ;

      // gwc: switching from GetPolyEdges() to GetNonDegeneratePolyEdges() eliminates this next section
      //  // skip polygons with degenerate edges
      //  double dScaledZero = SM_EFF_ZERO * ( 1.0 + sPoints[2].GetMaxDimension() );
      //  // Edge 0-1
      //  if ( sPoints[0].DistanceBetween( sPoints[1] ) < dScaledZero )
      //    { continue; }
      //  // Edge 1-2
      //  if ( sPoints[1].DistanceBetween( sPoints[2] ) < dScaledZero )
      //    { continue; }
      //
      //  // triangles
      //  if( lType == 0 )
      //    {
      //      // Edge 2-0
      //      if ( sPoints[2].DistanceBetween( sPoints[0] ) < dScaledZero )
      //        { continue; }
      //    }
      //  else // quads
      //    {
      //      // Edge 2-3
      //      if ( sPoints[2].DistanceBetween( sPoints[3] ) < dScaledZero )
      //        { continue; }
      //      // Edge 3-0
      //      if ( sPoints[3].DistanceBetween( sPoints[0] ) < dScaledZero )
      //        { continue; }
      //    } // end branches checking for degenerate edges within polygons

      //  Reset Normals for very small PolyFaces with large changes in PolyVertexNormals
      //  These Polygons are likely near difficult surface points with large errors
      //  in the computed surface normals [B267]
      
      // See comment in sms_ReviewPolyFaceNormals header.
      // if(SM_SUCCESS != sms_ReviewPolyFaceNormals(sFaceBBox, sPoints, sNormals))
      //  {
          // skip triangles too small to fix
      //    continue ;
      //  }

      // Output polygon, either triangle or quad
      SE(rPolygonOutput.OutputPolygon(lType,
                                      sPoints.GetSize(),
                                      sPoints.GetDataArray(),
                                      sNormals.GetDataArray(),
                                      sUVPoints.GetDataArray(),
                                      pFace->GetSurface(),
                                      pFace,
                                      pOptGfxSet));

    } // end iter ii, all PolyFaces - outputting all unprocessed PolyFaces

  // all done
  return SM_SUCCESS;

} // end SmTess::OutputFacePolygons

/*******************************************************************//**
PURPOSE: Compute the Normal to a 3 sided Polygon

NOTES: 
***********************************************************************/
static SmStatus sm_ComputePolygonNormal
 (SmPolyFace                * pPolyFace,      // in : PolyFace to Query
  const SmTArray<SmPoint3d> & cr3DPoints,     // in : PolyVertex 3dPositions - access: cr3DPoints[PolyVertex->GetIndexValue()]
  SmVector3d                & rPolygonNormal) // out: Normal for PolyFace
{
  SM_PTR_ARRAY(sPolyEdges, SmPolyEdge, 16) ; // SmTArray<SmPolyEdge*>

  // get PolyFace->PolyEdges
  pPolyFace->GetNonDegeneratePolyEdges(sPolyEdges) ;
  // pPolyFace->GetPolyEdges(sPolyEdges);

  // skip nonTriangular PolyFaces
  if(sPolyEdges.GetSize() != 3)
    { SER(SM_ERR) ;  }

  // locals
  SmPolyVertex * pV1 = sPolyEdges[0]->GetStartPolyVertex();
  SmPolyVertex * pV2 = sPolyEdges[0]->GetEndPolyVertex();
  SM_ASSERT_MSG(   sPolyEdges[1] == sPolyEdges[0]->GetCCWPolyEdge()
                || SmTol::IsDegenerate(sPolyEdges[0]->GetCCWPolyEdge()),
                _T("sm_ComputePolygonNormal: assumption about sPolyEdge array order is wrong - this is a bug")) ;
  SmPolyVertex * pV3 = sPolyEdges[1]->GetCCWPolyEdge()->GetEndPolyVertex();

  SmPoint3d      sP1 = cr3DPoints[pV1->GetIndexValue()];
  SmPoint3d      sP2 = cr3DPoints[pV2->GetIndexValue()];
  SmPoint3d      sP3 = cr3DPoints[pV3->GetIndexValue()];

  SmVector3d     sV1 = sP2 - sP1;
  SmVector3d     sV2 = sP3 - sP2;

  SmScaledZero sScaledZero = SmTol::GetScaledZero(sP1, sP2, sP3) ;
  // double dScale = SM_EFF_ZERO * (1.0 + sP1.GetMaxDimension() + sP2.GetMaxDimension() + sP3.GetMaxDimension());
  double dScaledZeroSq = sScaledZero * sScaledZero ;

  // degenerate edge checks
  if (sV1.LengthSquared() < dScaledZeroSq) { return SM_ERR; }
  if (sV2.LengthSquared() < dScaledZeroSq) { return SM_ERR; }

  // PolyFace Normal = EdgeVec cross product
  SmVector3d sCross = sV1 * sV2;

  // degenerate PolyFace check
  if (sCross.LengthSquared() < dScaledZeroSq) { return SM_ERR; }
  // if (sCross.LengthSquared() < SM_EFF_ZERO_SQ) { return SM_ERR; }

  // unitize the normal
  SER(sCross.Unitize());

  // set output
  rPolygonNormal = sCross;

  // all done
  return SM_SUCCESS;

} // end sm_ComputePolygonNormal

/*******************************************************************//**
PURPOSE: Get the normal of the PolyFace Neighbor connected to the
         CWPolyEdge edge of pPolyEdge->StartVertex

NOTES: 1. If the targeted edge is lamina (has no neighbor face)
          tries the next CWPolyEdge, and if that one is also
          lamina return SM_ERR without setting rPolygonNormal
***********************************************************************/
static SmStatus sm_GetOppositeSidePolyNormal
 (SmPolyEdge                * pPolyEdge,      // in : Source PolyEdge
  SmPolyFace                * pPolyFace,      // NotUsed: in : pPolyFace
  const SmTArray<SmPoint3d> & cr3DPoints,     // in : Array of PolyVertex 3dPositions access: cr3DPoints[PolyVertex->GetIndexValue()]
  SmVector3d                & rPolygonNormal) // out: Normal for neighbor PolyFace
{
  SM_REF1(pPolyFace) ; 
  // get target Next PolyEdge
  SmPolyEdge * pNextPolyEdge = pPolyEdge->GetCCWPolyEdge();

  // When target PolyEdge is lamina - seek another neighbor
  if (pNextPolyEdge->IsLamina()) { pNextPolyEdge = pNextPolyEdge->GetCCWPolyEdge(); }
  if (pNextPolyEdge->IsLamina()) { return SM_ERR; }

  // when PolyEdge has a neighbor PolyFace
  SmPolyEdge *pRadial  = pNextPolyEdge->GetRadial();
  SmPolyFace *pRadFace = pRadial->GetPolyFace();

  // pass that along to get the neighbor PolyFace Normal
  return sm_ComputePolygonNormal(pRadFace, cr3DPoints, rPolygonNormal);

} // end sm_GetOppositeSidePolyNormal

/*******************************************************************//**
PURPOSE: Output polygon edges for this face through the polygon output
         callback class.

NOTES: 
Example:
Output Types include:
   1) All Polygon Edges
   2) Boundary Edges
   3) Boundary Edges and Silhouette Edges

 increments unlocked mark value
***********************************************************************/
SmStatus SmTess::OutputFaceWireframe
 (SmFace                  * pFace,           // in : Face with SmTessSrfCache to output
  SmPolygonOutputCallback & rPolygonOutput,  // in : Chooses how and where to output the polygons
  SmGfxArraySet           * pOptGfxSet)      // i/o: used for OpenGL ES 2.0. When given, accumulate GfxVertexArrays not GL calls.
{
  // no work - not a valid request
  if (   rPolygonOutput.GetOutputType() != SM_PO_ALL_POLYGON_EDGES
      && rPolygonOutput.GetOutputType() != SM_PO_BOUNDARY_EDGES
      && rPolygonOutput.GetOutputType() != SM_PO_BOUNDARY_AND_SILHOUETTE_EDGES
      && rPolygonOutput.GetOutputType() != SM_PO_ALL_NON_PLANAR_EDGES)
    {
      SER(SM_ERR);
    }

  // locals
  ULONG ii, jj, kk ;
  //SmFaceuse      * pFU       = pFace->GetUpwardFaceuse();
  SmSurface      * pSurf     = pFace->GetSurface();
  SmTessSrfCache * pSC       = m_vCache.GetSecond(pFace);
  SmPolyBrep     * pPolyBrep = pSC ? pSC->mTS_pPolyBrep : NULL ;

  // no work - pFace has no TrimSrfCache with a m_TSPolyBrep with polygons
  NER(pSC) ;
  NER(pPolyBrep) ;

  // SmBoolean bReverse = FALSE;
  // if (pFU->GetShell()->GetRegion() != pFace->GetBrep()->GetInfiniteRegion() )
  //   {
  //     bReverse = TRUE;
  //   }

  // local Arrays
  SmPoint2d  sUVPnt;
  SmPoint3d  sPnt;
  SmVector3d sNorm(0,0,0);
  SM_PTR_ARRAY(sPolyFaces,     SmPolyFace, 256) ; // SmTArray<SmPolyFace *>
  SM_PTR_ARRAY(sPolyFaceEdges, SmPolyEdge,  16) ; // SmTArray<SmPolyEdge *>
#ifdef SM_32K_LOCAL_DATA_LIMIT
  SM_OBJ_ARRAY(sPoints3D,          SmPoint3d, 64) ; // SmTArray<SmPoint3d>
  SM_OBJ_ARRAY(sEdgeVertexIndices, ULONG,     64) ; // SmTArray<ULONG>
#else
  SM_OBJ_ARRAY(sPoints3D,          SmPoint3d, 1024) ; // PolyVertex 3dPoints - access: sPoints3d[pPolyV->GetIndexValue()]
  SM_OBJ_ARRAY(sEdgeVertexIndices, ULONG,     1024) ; // SmTArray<ULONG>
#endif

  // Get PolyBrep->PolyFaces
  pPolyBrep->GetPolyFaces(sPolyFaces);

#ifdef SM_DEBUG_CODE
  SmBoolean bDebugMe4 = FALSE;
  if (bDebugMe4)
    {
      smgfx_Erase();
      smgfx_SetColor(0,0,0);
      pPolyBrep->Draw();
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

  // increment and lock an unlocked mark
  SmNewMarkAndLock sMarkLock( pPolyBrep->GetContext(), SM_MT_ALLMARKS) ; // increment and lock any unlocked mark
  SmMarkType eMarkType = sMarkLock.GetMarkType() ;

  // compute and save PolyVertex->3DPoints in sPoints3D, access: sPoints3D[PolyVert->GetIndexValue]

  // First iter  = take care of all exterior face->PolyEdges (those marked with a SmEdgeUse)
  // Second iter = take care of remaining interior face->PolyEdges
  for (ii=0; ii<2; ii++)
    {
      for (jj=0; jj<sPolyFaces.GetSize(); jj++)
        {
          SmPolyFace *pPFace = sPolyFaces[jj];
          pPFace->GetPolyEdges(sPolyFaceEdges);

          // GWC: since marks on PolyVertices are being used - this culling (not present in the next iteration) isn't needed and may cause conflicts
          //   // skip degenerate PolyFaces
          //   if (sPolyFaceEdges.GetSize() < 3)
          //     {
          //       continue;
          //     }

          // for every PolyFace->PolyEdge
          for (kk=0; kk<sPolyFaceEdges.GetSize(); kk++)
            {
              SmPolyEdge   * pPolyFaceEdge = sPolyFaceEdges[kk];
              SmPolyVertex * pStartV       = pPolyFaceEdge->GetStartPolyVertex();

              // when pStartV is unmarked
              if (!pStartV->IsMarked(eMarkType))
                {
                  if (GetPolyEdgeEU(pPolyFaceEdge))
                    {
                      if (ii==1) { SER(SM_ERR); }
                      SmVertex *pVSt = GetPolyEdgeEU(pPolyFaceEdge)->GetVertexuse()->GetVertex();
                      sPnt = pVSt->GetPoint();
                    }
                  else
                    {
                      if (ii==0) continue;
                      sUVPnt = pPolyFaceEdge->GetStartPoint();
                      SER(pSurf->EvaluatePoint(sUVPnt,sPnt));
                    }

                  // Save information about this point in the head
                  pStartV->SetIndexValue(sPoints3D.GetSize());
                  pStartV->Mark(eMarkType);
                  sPoints3D.Add(sPnt);
                }
            } // end iter kk, for ever PolyFace->PolyEdge
        } // end iter jj, for every PolyFace
    } // end iter ii, for two passes setting sPoints3D point values and PolyVertex->IndexValues

  // for every PolyFace - Output PolyFaces as edge lists using marks to prevent listing edges multiple times
  for (ii=0; ii<sPolyFaces.GetSize(); ii++)
    {
      SmPolyFace *pPFace = sPolyFaces[ii];

      // skipped already processed PolyFaces
      if (pPFace->IsMarked(eMarkType))
        { continue; }

      pFace->Mark(eMarkType);

      // get NonDegenerate PolyEdges - needed for quads with singular edges acting as triangles
      // used to be: pPFace->GetPolyEdges(sPolyFaceEdges);
      pPFace->GetNonDegeneratePolyEdges(sPolyFaceEdges);

      // for every PolyFace->PolyEdge
      for (jj=0; jj<sPolyFaceEdges.GetSize(); jj++)
        {
          SmPolyEdge *pPolyFaceEdge = sPolyFaceEdges[jj];

          // skip already processed PolyFace->PolyEdges
          if (pPolyFaceEdge->IsMarked(eMarkType))
            { continue; }

          pPolyFaceEdge->Mark(eMarkType);

          // SmBoolean bBoundaryEdge = FALSE;
          // if (GetPolyEdgeEU(pPolyFaceEdge) != NULL)
          //   {
          //     bBoundaryEdge = TRUE;
          //   }

          // when PolyFace->Edge is Manifold - check for cases where PolyEdge does not get drawn
          if( pPolyFaceEdge->IsManifold() )
            {
              pPolyFaceEdge->GetRadial()->Mark(eMarkType);

              // when some PolyEdges don't get drawn
              if (   rPolygonOutput.GetOutputType() == SM_PO_BOUNDARY_AND_SILHOUETTE_EDGES
                  || rPolygonOutput.GetOutputType() == SM_PO_ALL_NON_PLANAR_EDGES)
                {
                  SmVector3d sNorm1, sNorm2;
                  if (sm_ComputePolygonNormal(pPFace,sPoints3D,sNorm1) != SM_SUCCESS)
                    {
                      // try and get Normal from neighbor PolyFace when this PolyFace is a zero size polygons
                      if (sm_GetOppositeSidePolyNormal(pPolyFaceEdge,pPFace,sPoints3D,sNorm1) != SM_SUCCESS)
                        {
                          continue;
                        }
                    }

                  SmPolyFace *pOtherPFace = pPolyFaceEdge->GetRadial()->GetPolyFace();
                  if (sm_ComputePolygonNormal(pOtherPFace,sPoints3D,sNorm2) != SM_SUCCESS)
                    {
                      // try and get Normal from neighbor PolyFace when this PolyFace is a zero size polygons
                      if (sm_GetOppositeSidePolyNormal(pPolyFaceEdge->GetRadial(),pOtherPFace,sPoints3D,sNorm2) != SM_SUCCESS)
                        {
                          continue;
                        }
                    }

                  // when asked - don't draw coplanar edges
                  if (rPolygonOutput.GetOutputType() == SM_PO_ALL_NON_PLANAR_EDGES)
                    {
                      if (sNorm1.CloserThan(SM_EFF_ZERO, sNorm2))
                        {
                          continue;  // Don't draw coplanar edges
                        }
                    } // end need to cull for coplanar PolyEdge check: SM_PO_ALL_NON_PLANAR_EDGES

                  else // when asked - don't draw PolyEdge connected to PolyFaces oriented in the same viewing direction
                       //              i.e. only draw PolyEdges connected to PolyFaces whose normals point in opposite viewing directions
                       // SM_PO_BOUNDARY_AND_SILHOUETTE_EDGES
                    {
                      SmPoint3d  sPointOnEdge = sPoints3D[pPolyFaceEdge->GetStartPolyVertex()->GetIndexValue()];
                      SmVector3d sViewVector  = rPolygonOutput.GetViewDirection(sPointOnEdge);
                      if (sViewVector.Dot(sNorm1) * sViewVector.Dot(sNorm2) > -SM_EFF_ZERO)
                        {
                          // Not a silhouette just skip it
                          continue;
                        }
                    } // end need to cull PolyEdges connected to PolyFaces oriented in the same viewing direction check: SM_PO_BOUNDARY_AND_SILHOUETTE_EDGES
                } // end when some PolyEdges don't get drawn check:  SM_PO_BOUNDARY_AND_SILHOUETTE_EDGES
                  //                                                 SM_PO_ALL_NON_PLANAR_EDGES)

              // don't draw Manifold edges for case SM_PO_BOUNDARY_EDGES
              if (rPolygonOutput.GetOutputType() == SM_PO_BOUNDARY_EDGES)
                {
                  continue;
                }

            } // end PolyFace->PolyEdge is manifold check - check for cases where ManifoldEdge does not get drawn

          // Arrive here when PolyEdge is added to output

          // Add Edge PolyVertex end pair to sEdgeVertexIndices
          SmPolyVertex *pStartV = pPolyFaceEdge->GetStartPolyVertex();
          sEdgeVertexIndices.Add(pStartV->GetIndexValue());

          SmPolyVertex *pEndV = pPolyFaceEdge->GetEndPolyVertex();
          sEdgeVertexIndices.Add(pEndV->GetIndexValue());

        } //end iter jj, every PolyFace->PolyEdge
    } // end iter ii, every PolyFace

  // Output the Point3D and EdgeVertexIndex arrays.
  SER(rPolygonOutput.OutputFaceWireframe(pFace,
                                         sEdgeVertexIndices,
                                         sPoints3D,
                                         pOptGfxSet));

  // all done
  return SM_SUCCESS;

} // end SmTess::OutputFaceWireframe

/*******************************************************************//**
PURPOSE:

NOTES:  
***********************************************************************/
static SmStatus sm_AddEdgeToVertexTree
 (SmPolyEdge * pNewEdge,
  SmTree     * pVertexTree)
{
  SmPolyVertex *pVert = pNewEdge->GetStartPolyVertex();
  SmExtent3d sVBBox(pVert->GetPoint());
  sVBBox.ExpandAbsolute(pVert->GetTolerance());
  SER(pVertexTree->AddToSpatialTree(sVBBox,pNewEdge));
  return SM_SUCCESS;

} // end sm_AddEdgeToVertexTree

/*******************************************************************//**
PURPOSE:

NOTES:  
***********************************************************************/
static SmBoolean IsCollinear (double angleDeg)
{
    constexpr double angleEps = 0.0011; // something bigger than in IsConvexCorner
    return smos_Fabs (angleDeg) < angleEps || smos_Fabs (180 - angleDeg) < angleEps || smos_Fabs (360 - angleDeg) < angleEps;
}

/*******************************************************************//**
PURPOSE: For Debug only - pretty print corner scores and histories
         from SmTess::TriangulateFace()

NOTES: score ==  (bInsideVertices) ? BAD_SCORE
                :  ((!IsConvex2D || !IsConvex3D) ? BAD_SCORE : 0)
                 + (1.0-dWt3d)*2d AngleDeg
                 + (  dWt3d  )*3d AngleDeg
                 + (bSingleLoopV ? 360 : 0)
                 + 1000 * lNumSplitsToBeLinear
***********************************************************************/
#define SM_SCORE_NOT_CONVEX2d    (1 << 0)  // BAD_SCORE
#define SM_SCORE_NOT_CONVEX3d    (1 << 1)  // BAD_SCORE
#define SM_SCORE_SINGLE_VERTEX   (1 << 2)  // += 360.0
#define SM_SCORE_ANGLES          (1 << 3)  // += (ULONG)( (1.0-dWt3d)*dAngDeg2d + dWt3d*dAngDeg3d )
#define SM_SCORE_SUBDIVISIONS    (1 << 4)  // += lNumSubdivisionsNeeded * 1000
#define SM_SCORE_INSIDE_VERTICES (1 << 5)  // BAD_SCORE
#define SM_SCORE_RESET_1         (1 << 6)  // score reset to 0
#define SM_SCORE_RESET_2         (1 << 7)  // score reset to 0

#define SM_SCORE_ITER_INC        (1 << 8)  // must be bigger than all other score values

/*******************************************************************//**
PURPOSE:

NOTES:  
***********************************************************************/
void sm_DumpScoreHistory
 (ULONG                   lNumIter,    // in : Iteration number
  ULONG                   lBestIndx,   // NotUsed: in : index of BestScore corner
  SmTArray<SmPolyEdge *> &rPolyEdges,  // in : array of all scored PolyEdges
  SmTArray<ULONG>        &rScores,     // in : Scores[rPolyEdges[ii].GetIndexValue()] for rPolyEdges[ii]
  SmTArray<ULONG>        &rHistory,    // in : History[rPolyEdges[ii].GetIndexValue()] for rPolyEdges[ii]
  SmTArray<SmBoolean>    &rIterChange, // in : IterChange[rPolyEdges[ii].GetIndexValue()] == TRUE for scores that change this iter.
  SmBoolean               bDumpAll)    // in : TRUE = dump all score histories, FALSE=Only Dump changed score histories.
{
  SM_REF1(lBestIndx) ; 
  TCHAR sBuff[SM_TBLOCK_SIZE] ;
  ULONG ii, jj ;
  ULONG lIterChangeCount = 0 ;
  constexpr ULONG BAD_SCORE = 999999 ;

  for(ii=0;ii<rIterChange.GetSize();ii++)
    {
      if(rIterChange[ii]) { lIterChangeCount++ ; }
    }

  // header
  smos_sprintf(sBuff, _T("\nTriangulateFace iter[%3lu] Score histories, CornerCount:[%3lu], ChangedScores:[%3lu], * = Changed Corner"), lNumIter,
                                                                                                               rPolyEdges.GetSize(),
                                                                                                               lIterChangeCount);
  smos_WriteBuffer(sBuff) ;

  // for every PolyEdge
  for(ii=0;ii<rPolyEdges.GetSize();ii++)
    {
      ULONG lIndx    = rPolyEdges[ii]->GetIndexValue() ;
      ULONG lScore   = rScores[lIndx] ;
      ULONG lHistory = rHistory[lIndx] ;

      // when asked skip unchanged scores
      if(!bDumpAll && rIterChange[ii] == FALSE)
        { continue ; }

      // this PolyEdge->StartVertex label and raw score
      smos_sprintf(sBuff, _T("\n  Corner[%3lu%s], Score[%7lu]: "), lIndx,
                                                               rIterChange[ii] ? _T("*") : _T(" "),
                                                               lScore) ;
      smos_WriteBuffer(sBuff) ;

      // Parse Score
      TCHAR  sAngString[64] ;
      ULONG  lNumSubdivisions = 0 ;
      ULONG  lIterOffset      = (1 << 0) ;
      if(lScore == BAD_SCORE) { smos_snprintf(sAngString, 64, _T("%s"), _T(" - ")) ; }
      else                    { if(lScore > BAD_SCORE) { lScore -= BAD_SCORE ; }
                                if(lScore > 999) { lNumSubdivisions = lScore / 1000 ;
                                                   lScore -= lNumSubdivisions * 1000 ;
                                                 }
                                if(lScore >= 360) { lScore -= 360 ; }
                                smos_snprintf(sAngString, 64, _T("%3lu"), lScore) ;
                              }

      // score ==  (bInsideVertices) ? BAD_SCORE
      //          :  ((!IsConvex2D || !IsConvex3D) ? BAD_SCORE : 0)
      //           + (1.0-dWt3d)*2d AngleDeg
      //           + (  dWt3d  )*3d AngleDeg
      //           + (bSingleLoopV ? 360 : 0)
      //           + 1000 * lNumSplitsToBeLinear

      // for the experiment - each score might have 3 iterations
      for(jj=0;jj<3;jj++)
        {
          if(lHistory & (SM_SCORE_NOT_CONVEX2d    * lIterOffset)) { smos_WriteBuffer(_T("Not Convex2d,     ")) ; }
          if(lHistory & (SM_SCORE_NOT_CONVEX3d    * lIterOffset)) { smos_WriteBuffer(_T("Not Convex3d,     ")) ; }
          if(lHistory & (SM_SCORE_SINGLE_VERTEX   * lIterOffset)) { smos_WriteBuffer(_T("Single Vertex,    ")) ; }
          if(lHistory & (SM_SCORE_ANGLES          * lIterOffset)) { smos_sprintf(sBuff,_T("AngDegScored[%s], "), sAngString) ;       smos_WriteBuffer(sBuff) ; }
          if(lHistory & (SM_SCORE_SUBDIVISIONS    * lIterOffset)) { smos_sprintf(sBuff,_T("Subdivisons[%3lu], "), lNumSubdivisions) ; smos_WriteBuffer(sBuff) ; }
          if(lHistory & (SM_SCORE_INSIDE_VERTICES * lIterOffset)) { smos_WriteBuffer(_T("Inside Verts,     ")) ; }

          if((jj == 0) && (lHistory & SM_SCORE_RESET_1)) { smos_WriteBuffer(_T("Score Reset 1,    ")) ; }
          if((jj == 1) && (lHistory & SM_SCORE_RESET_2)) { smos_WriteBuffer(_T("Score Reset 2,    ")) ; }

          lIterOffset *= SM_SCORE_ITER_INC ;
       } // end iter experiment cycles
    } // end iter every PolyEdge
} // end sm_DumpScoreHistory

/*******************************************************************//**
PURPOSE: Triangulate a general pPolyFace polygon by inserting a sequence
  of new PolyEdges into the PolyFace->PolyBrep until the original
  PolyFace polygon is split into a set of triangular PolyFace polygons.

 NOTES: 
  1. Does not add new PolyVertices but does record how many subdvisions
  it thinks each new PolyEdge needs to be the right size for a good
  tessellation of the existing face.  It's left to the caller
  to use the subdivision output information to add new PolyVertices
  to the tessellation.

  2. Exception that adds a new PolyVertex.
  When adding a PolyEdge that connects to a PolyVertex on a singular
  PolyEdge, will split the singular PolyEdge, adding a new PolyVertex, if
  the split is within the domain of the singular PolyEdge and makes the
  new PolyEdge an IsoParameter line.  In which case the PolyFace
  made by the split will have 4 PolyEdges, just 3 of which are
  NonDegenerate.

METHOD:
Here is how this works:
    0) Combine interior loops using minimum distance function
    1) Look for a convex corner 'V0' in the outer loop.
    2) Validate that the two adjacent corners can have an edge
       using the surface cache to check the distance from 'VCW' to 'VCCW'
    3) Find any other vertices in triangle V0, VCW, VCCW
         a) If none found make triangle and split face.
         b) Find the vertex VBest which is closest to the angle of
            the edge to VCCW and closest in distance to V0 in case of
            ties relative to the angle.
         c) Make an edge from V0 to VBest.  If it splits the face
            then call TriangulateFace recursively.  If it doesn't
            just continue on.

  NOTE-To improve quality measure edge length relative to quad size
  and find best corner to cut for optimal tessellation.
***********************************************************************/
SmStatus SmTess::TriangulateSingleFace
 (SmTArray<SmPolyFace*> & rPolyFaceStack,        // i/o: Top member gets triangulated,
                                                 //      MakeManifoldEdge() generated children get added to stack.
  SmTArray<SmPolyEdge*> & rPolyEdgesToSubdivide, // i/o: new poly edges which should subdivided
  SmTArray<ULONG>       & rPolyEdgeSubdivisions, // i/o: number of subdivisions for members of rPolyEdgesToSubdivide
  SmTree               *& rpVertexTree,          // i/o: Spatial tree of PolyFace PolyVertices.
                                                 //      New PolyVertices are added.
  SmNewMarkAndLock      & rMarkLock)             // in : increments MarkLock mark value when rpVertexTree is NULL on input
{
  SM_ASSERT_MSG(rPolyFaceStack.GetSize () > 0, _T("SmTess::TriangulateSingleFace given unexpectedy Empty PolyFaceStack"));

  // Get target PolyFace = top of rPolyFaceStack
  SmPolyFace * pPolyFace = NULL;
  rPolyFaceStack.Pop (pPolyFace);

#ifdef SM_DEBUG_CODE
  SmBoolean bDebugMe = FALSE; 
  std::atomic<ULONG> lCount(1);
  lCount++;
  std::atomic<ULONG> lDebugCount(0);
  if (bDebugMe || lDebugCount == lCount)
    {
      SmTArray<SmPolyEdge*> sPolyEdges;
      pPolyFace->GetPolyEdges(sPolyEdges);

      smgfx_Erase();
      smgfx_SetLook(1,2, 0,0,0); pPolyFace->Draw( TRUE, FALSE ); sm_GraphicsLoop();
      smgfx_SetLook(1,2, 0,0,0); pPolyFace->Draw( TRUE, TRUE  ); sm_GraphicsLoop();
      sm_GraphicsLoop();

      smgfx_Erase();
      smgfx_SetLook(1,2, 0,0,0); pPolyFace->m_pOriginalFace->Draw(SM_DM_WIREFRAME); sm_GraphicsLoop();
      for (ULONG i=0; i<sPolyEdges.GetSize(); i++)
        {
           smgfx_SetLook(1,2, 0,0,1); sPolyEdges[i]->Draw  (); sm_GraphicsLoop();
           smgfx_SetLook(1,2, 0,0,1); sPolyEdges[i]->Draw3D(); sm_GraphicsLoop();
        }
      sm_GraphicsLoop();
  }
#endif // SM_DEBUG_CODE

  // locals
  ULONG ii, jj ;
  SmFace         * pOriginalFace = pPolyFace->GetOriginalFace();
  SmTessSrfCache * pTessCache    = m_vCache.GetSecond( pOriginalFace );

  // local array data
  SM_PTR_ARRAY(sPolyLoops,            SmPolyLoop,   64) ; // SmTArray<SmPolyLoop *>
  SM_PTR_ARRAY(sPolyFaceEdges,        SmPolyEdge,  256) ; // SmTArray<SmPolyEdge *>
  SM_PTR_ARRAY(sInsidePolyVerts,      SmPolyEdge,   64) ; // SmTArray<SmPolyEdge *>
  SM_PTR_ARRAY(sInsidePolyVerts_3d,   SmPolyEdge,   64) ; // SmTArray<SmPolyEdge *>
  SM_PTR_ARRAY(sSingleVertices,       SmPolyVertex, 16) ; // SmTArray<SmPolyVertex *>

  SM_OBJ_ARRAY(sScores,             ULONG,      512) ; // SmTArray<ULONG>
  SM_OBJ_ARRAY(sNumSubdivisions,    ULONG,      512) ; // SmTArray<ULONG>
  SM_OBJ_ARRAY(sChordDist,          double,     512) ; // SmTArray<double>

#ifdef SM_DEBUG_CODE
  ULONG lIterOffset = (1 << 0) ;
  ULONG lIterInc    = SM_SCORE_ITER_INC ;  SM_REF1(lIterInc) ;
  SM_OBJ_ARRAY(sHistory,     ULONG,      512) ; // SmTArray<ULONG>
  SM_OBJ_ARRAY(sIterChanged, SmBoolean,  512) ; // SmTArray<ULONG>
#endif // SM_DEBUG_CODE

  // get PolyFace->PolyLoops
  pPolyFace->GetPolyLoops(sPolyLoops);

  // If the face has more than one loop, connect them all into a single loop.
  // Starting with the outer loop (which is the first one, sPolyLoops[0]),
  // connect an inner loop to it, by joining two vertices.
  // Connect the closest pair of vertices.

  // Begin scope - Connect Multiple loops
    {
      // locals
      SmPolyEdge * pNewPolyEdge  = NULL ;
      SmPolyLoop * pNewPolyLoop  = NULL ;
      SmPolyFace * pNewPolyFace  = NULL ;

      SmTArray<SmPolyEdge*> sNonDegenPolyEdges;
      pPolyFace->GetNonDegeneratePolyEdges(sNonDegenPolyEdges);

      // while pPolyFace has inner loops
      while ( sPolyLoops.GetSize() > 1 )
        {
          // Find smallest loop to connect to others.
          // Starting with the smaller loops first helps a little in the computation when there are lots of loops.
          SmPolyLoop * pInnerLoop = NULL;
          ULONG        lMinEdges  = SM_BIG_ULONG;
          ULONG lNumLoops = sPolyLoops.GetSize();
          for ( ii=1; ii<lNumLoops; ii++ )
            {
              if (sPolyLoops[ii]->GetSize() < lMinEdges)
                {
                  pInnerLoop = sPolyLoops[ii];
                  lMinEdges = pInnerLoop->GetSize();
                }
            } // end iter PolyLoops finding smallest loop

          // find PolyFace->OtherPolyLoopEdge with start-pt closest to any input pPolyLoop->PolyEdges
          SmPolyEdge * pPolyLoopEdge = NULL ;
          SmPolyEdge * pPolyFaceEdge = NULL;
          SmStatus eStat = pPolyFace->FindClosestPointToLoop(pInnerLoop, // in : PolyLoop to check
                                                             NULL, // in : NotNull = Find closest point to just this
                                                                   // PolyLoopEdge,
                                                                   //      NULL    = Find closest point to any
                                                                   //      PolyLoopEdge
                                                             pPolyLoopEdge, // out: PolyLoopEdge closest to
                                                                            // NeighborLoopEdge
                                                             pPolyFaceEdge, // out: NeighborLoop->PolyEdge with start-pt
                                                                            // closest to PolyLoopEdge
                                                             &sNonDegenPolyEdges);

          if ( eStat != SM_SUCCESS ) // [B434]
          {
              ULONG lIndex;
              if ( sPolyLoops.FindElement( pInnerLoop, lIndex ) )
                {
                  pPolyFace->RemovePolyLoop(pInnerLoop, NULL);
                  sPolyLoops.RemoveAt(lIndex);
                  // Refresh the list of non-degen poly edges if we'll need it again
                  if (sPolyLoops.GetSize() > 1) 
                  {
                      pPolyFace->GetNonDegeneratePolyEdges(sNonDegenPolyEdges);
                  }
                }
              else
                { SER_MSG( SM_ERR, _T("SmTess::TriangulateFace() error: FindClosestPointToLoop failed - a bug") ); }
              continue;
          }

#ifdef SM_DEBUG_CODE
          SmBoolean bDebugMe0 = FALSE;
          if (bDebugMe0)
            {
              smgfx_Erase();
              smgfx_SetLook(1,1, 0,0,0); pPolyFace->Draw();  sm_GraphicsLoop();
              smgfx_SetLook(4,1, 0,1,0); pPolyFaceEdge->Draw();  sm_GraphicsLoop();
              smgfx_SetLook(2,1, 0,0,1); pInnerLoop->Draw(); sm_GraphicsLoop();
              smgfx_SetLook(4,1, 1,0,0); pPolyLoopEdge->Draw();  sm_GraphicsLoop();
              sm_GraphicsLoop();
            }
#endif // SM_DEBUG_CODE

          // error case - no closest PolyEdge pair found
          if ( pPolyLoopEdge == NULL || pPolyFaceEdge == NULL )
            { SER_MSG( SM_ERR, _T("SmTess::TriangulateFace() error: FindClosestPointToLoop failed - a bug") ); }

          // If either edge is a single-vertex edge, record it.
          if(pPolyLoopEdge->GetStartPolyVertex()->IsSingleVertexLoop())
            { sSingleVertices.Add(pPolyLoopEdge->GetStartPolyVertex()) ; }

          if(pPolyFaceEdge->GetStartPolyVertex()->IsSingleVertexLoop())
            { sSingleVertices.Add(pPolyFaceEdge->GetStartPolyVertex()) ; }

          // Now connect the loops with a manifold PolyEdge pair between their closest vertices.
          SER( pPolyFace->GetPolyBrep()->MakeEdgeInFace
                   (pPolyFace,                           // in : target PolyFace
                    pPolyLoopEdge->GetStartPolyVertex(), // in : Start of New manifold PolyEdge pair
                    pPolyFaceEdge->GetStartPolyVertex(), // in : End of New manifold PolyEdge pair
                    pPolyFace->GetTolerance(),           // in : NewEdge assigned tolerance
                    pNewPolyEdge,                        // out: one of the PolyEdges of the new manifold PolyEdge pair.
                                                         //      The PolyEdge pair can be any mix of new or reused PolyEdges.
                    pNewPolyLoop,                        // out: Created when NewEdge splits an existing PolyLoop (vertices start connected to same PolyLoop)
                    pNewPolyFace ) );                    // out: Created when PolyLoop is split to contain one of the split children.
                                                         //      pFace contains the other split child.

          // Refresh the Loops list - should have one less loop
          pPolyFace->GetPolyLoops( sPolyLoops );

          // does pNewPolyEdge 3dShape need subdivision so segment sizes are about surface subdivsion node size
          ULONG lNumSubdivisionsNeeded;
          SER( CheckAgainstSurfaceCache( pTessCache,                    // in : Cache with subdivision tree to interogate
                                         pNewPolyEdge->GetStartPoint(), // in : start of UV segment to review
                                         pNewPolyEdge->GetEndPoint(),   // in : end of UV segment to review
                                         1.1,                           // in : allowed size variation before segment needs splits
                                         TRUE,                          // in : TRUE = interior edge - don't check 3d lengths
                                                                        //      FALSE= exterior edge - do check 3d lengths
                                         lNumSubdivisionsNeeded ));     // out: number of splits Start/End UV segment needs
                                                                        //      so segment sizes are about the smallest xsecting
                                                                        //      cache subdivision tree leaf node size.
          // when pNewPolyEdge subdivison is needed

          if ( lNumSubdivisionsNeeded > 0 )
          {
              SmTArray<SmPolyEdge*> sNewPolyEdgesFromSubdivision;

              // split edge
              SER( SubdivideManifoldEdge( pNewPolyEdge,           // in : PolyEdge to split
                                          lNumSubdivisionsNeeded, // in : number of splits required
                                          rpVertexTree,         // in : spatial tree of <PolyEdge,PolyEdge->StartPt position> pairs
                                          TRUE,
                                          sNewPolyEdgesFromSubdivision));
              sNonDegenPolyEdges.Append(sNewPolyEdgesFromSubdivision);
          }
          else
          {
              sNonDegenPolyEdges.Add(pNewPolyEdge);
          }

        } // end while pPolyFace has inner loops

#ifdef SM_DEBUG_CODE
        SmTArray<SmPolyLoop*> dPolyLoops;
        pPolyFace->GetPolyLoops(dPolyLoops);
        if (dPolyLoops.GetSize() != 1)
        {
            SM_ASSERT_MSG(dPolyLoops.GetSize() > 1, _T("PolyFace has multiple PolyLoops when it should have 1."));
            SM_ASSERT_MSG(dPolyLoops.GetSize() == 0, _T("PolyFace has 0 PolyLoops"));
        }
#endif
    } // End Scope - to pre-process PolyLoops into a single PolyLoop

  // Now all loops are connected into a single one.

#ifdef SM_DEBUG_CODE
  if (bDebugMe || lDebugCount == lCount)
    {
      SmFace     * pFace        = pPolyFace->GetOriginalFace() ;
      SmPolyBrep * pTS_PolyBrep = GetTessBrepOfFace(pFace) ;
      SM_ASSERT_VALID(pTS_PolyBrep) ;
      SM_ASSERT_VALID(pPolyFace) ;
      SM_PTR_ARRAY(sPolyEdges, SmPolyEdge, 32) ; // SmTArray<SmPolyEdge*>
      pPolyFace->GetPolyEdges(sPolyEdges);

      smgfx_Erase();
      smgfx_SetLook(1,2, 0,0,0); pPolyFace->Draw( TRUE, FALSE ); sm_GraphicsLoop();
      smgfx_SetLook(1,2, 0,0,0); pPolyFace->Draw( TRUE, TRUE  ); sm_GraphicsLoop();
      sm_GraphicsLoop();

      if ( FALSE )
        { smgfx_Erase(); }
      smgfx_SetLook(1,2, 0,0,0); pPolyFace->m_pOriginalFace->Draw(SM_DM_WIREFRAME); sm_GraphicsLoop();
      smgfx_SetLook(4,6, 0,0,1);
      for (ULONG i=0; i<sPolyEdges.GetSize(); i++)
        {
          sPolyEdges[i]->Draw  (); sm_GraphicsLoop();
          sPolyEdges[i]->Draw3D(); sm_GraphicsLoop();
          sm_GraphicsLoop();
        }
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

  // get PolyFace->PolyEdges

  // pPolyFace->GetNonDegeneratePolyEdges(sPolyFaceEdges);
  // used to be: pPolyFace->GetPolyEdges(sPolyFaceEdges);

  // performance improvement in corner cutting topology updating
  // requires that there are no adjacent degenerate PolyEdges.
  // Previously, these were just passed over in tessellation
  // Now, we cull them to allow for performance improvements
  // to topology updating.

  // TODO: go back and simplify code now there are not degenerate edges
  //       in the array sPolyFaceEdges.

  SmTArray<SmPolyEdge*> sDegenPolyEdges;
  pPolyFace->GetDegeneratePolyEdges(sDegenPolyEdges);

  for (ULONG kk = 0; kk < sDegenPolyEdges.GetSize(); kk++)
  {
      SmPolyEdge* pPolyEdge = sDegenPolyEdges[kk];

      // Squeeze the degen Edge if it's manifold and part of a loop
      // These requirements are due to limitations of SqueezeEdge
      if (   pPolyEdge            != pPolyEdge->GetRadial()
          && pPolyEdge->GetNext() != pPolyEdge->GetRadial())
      {
          SmPolyVertex* pStartPolyVertex = pPolyEdge->GetStartPolyVertex();
          pPolyEdge->GetPolyBrep()->SqueezeEdge(pPolyEdge, pStartPolyVertex);
      }
      else
      {
          SM_DBG_MSG(TRUE, _T("Unexpected degenerate PolyEdge. Investigate."));
      }

      delete pPolyEdge;
      pPolyEdge = NULL;
  }

  // get PolyFace->PolyEdges

  pPolyFace->GetPolyEdges(sPolyFaceEdges);
  // used to be: pPolyFace->GetPolyEdges(sPolyFaceEdges);

  // case 1 - triangle or degenerate polygon - no work
  if ( sPolyFaceEdges.GetSize() <= 3 )
    { return SM_SUCCESS; }

  // Begin Scope - case 2 - Four-sided convex polygon - split along shortest mid-edge
  //   GWC: this case does not split added manifold edge pairs to tessellation params.
  //        That's a good thing because these quads come from the leaf nodes of the
  //        surface subdivision tree and won't need further subdivision.
    {
      // if four-sided convex PolyFace - split PolyFace into two triangles and return.
      if ( sPolyFaceEdges.GetSize() == 4 )
        {
          // locals
          SmVector3d sUp( 0,0,1 );
          double     dAngDeg = 0.0, dAngDeg3d = 0.0;
          double     dRatio = 0.0;

          // If convex polygon between NonDegenerate Neighbors
          if (   sPolyFaceEdges[0]->IsConvexCorner(sUp,dAngDeg,dRatio,dAngDeg3d,sPolyFaceEdges[3])   // TRUE = inside dAngleDeg at StartVert in PolyEdge->PolyLoop < 179.999
              && sPolyFaceEdges[1]->IsConvexCorner(sUp,dAngDeg,dRatio,dAngDeg3d,sPolyFaceEdges[0])   // TRUE = inside dAngleDeg at StartVert in PolyEdge->PolyLoop < 179.999
              && sPolyFaceEdges[2]->IsConvexCorner(sUp,dAngDeg,dRatio,dAngDeg3d,sPolyFaceEdges[1])   // TRUE = inside dAngleDeg at StartVert in PolyEdge->PolyLoop < 179.999
              && sPolyFaceEdges[3]->IsConvexCorner(sUp,dAngDeg,dRatio,dAngDeg3d,sPolyFaceEdges[2]) ) // TRUE = inside dAngleDeg at StartVert in PolyEdge->PolyLoop < 179.999
            {
              SmPolyEdge   * pNewPolyEdge  = NULL ;
              SmPolyLoop   * pNewPolyLoop  = NULL ;
              SmPolyFace   * pNewPolyFace  = NULL ;
              SmPolyEdge   * pNewSingularEdge   = NULL ;
              SmPolyVertex * pNewSingularVertex = NULL ;

              // get polygon diagonal lengths
              double dDist02 = sPolyFaceEdges[0]->GetStartPoint().DistanceBetweenSquared(
                               sPolyFaceEdges[2]->GetStartPoint() );
              double dDist13 = sPolyFaceEdges[1]->GetStartPoint().DistanceBetweenSquared(
                               sPolyFaceEdges[3]->GetStartPoint() );

              // Split polygon with shorter diagonal: creates better triangles (less skinny)
              if ( dDist02 < dDist13 )
                {
                  SER(pPolyFace->MakeManifoldEdge(sPolyFaceEdges[0], sPolyFaceEdges[2], pNewPolyEdge, pNewPolyLoop,
                                                  pNewPolyFace,
                                                  FALSE,           // in : Depends on whether there are degenerate edges. 
                                                                                          //      If there are no singular edges, no need to do extra checking for changes around singular edges.
                                                  &pNewSingularEdge,      // out: New SingularEdge or NULL when no Singular
                                                                                          // Edge found needing splitting
                                                                                          //      (a radial mateEdge running from End to
                                                                                          //      Start is also made)
                                                  &pNewSingularVertex,  // out: New SingularVertex or NULL when no
                                                                                          // Singular Edge found needing splitting
                                                   sPolyFaceEdges[1]));

                }
              else
                {
                  SER(pPolyFace->MakeManifoldEdge(sPolyFaceEdges[1],
                                                  sPolyFaceEdges[3],
                                                  pNewPolyEdge,
                                                  pNewPolyLoop,
                                                  pNewPolyFace,
                                                  FALSE,             // in : Depends on whether there are degenerate edges.
                                                                                            //      If there are no singular edges, no need to do extra checking for changes around singular edges.
                                                  &pNewSingularEdge,        // out: New SingularEdge or NULL when no Singular Edge found needing splitting
                                                                                            //      (a radial mateEdge running from End to Start is also made)
                                                  &pNewSingularVertex,    // out: New SingularVertex or NULL when no Singular Edge found needing splitting
                                                  sPolyFaceEdges[2]));
                }
              return SM_SUCCESS;

            } // end convex-polygon check
        } // end four-sided polygon check
    } // End Scope - special case 2: four-sided convex polygons

  // arrive here when polygon is not degenerate, a triangle, or a convex quad



  #ifdef SM_USE_TESS_SPLITFACE
  {
      // BEGIN JLMCC EDIT: Splitting the face to improve aspect ratio. Doing this a few times should decrease the
      // ability for long fans to persist as the face itself will have a smaller aspect ratio. Note that the aspect
      // definition in FindSplitLine is not intuitive.

      // locals
      SmBoolean bSuccess = FALSE;
      ULONG lLineStartIndex;
      ULONG lLineEndIndex;
      double dMaxAspectRatio = 5;
      double dLimitAspectRatio = NULL;

      // polyFace normal
      SmVector3d sFaceNormal = pPolyFace->GetNormal(FALSE, // in : FALSE = return m_vNormal when available, TRUE =
                                                           // always recompute m_vNormal
                                                    TRUE); // in : TRUE = increase polyEdge and polyVertex tolerances
                                                           // when needed

      // find best SplitLine (maximizes child loop aspect ratios)
      SmPolyFace::FindSplitLine(sPolyFaceEdges, // in : ordered target edges forming a polygon to be split
                                sFaceNormal, // in : plane normal of the target loop
                                &dLimitAspectRatio, // in : NULL = no min limit of output aspect ratios
                                NULL, // in : NULL = reject splits with child loops that touch or cross the splitPlane
                                bSuccess, // out: TRUE = found a split line that satisfies split constraints,
                                          // FALSE=didn't
                                dMaxAspectRatio, // out: Aspect ratio for given split, else -SM_BIG_DOUBLE
                                lLineStartIndex, // out: SplitLine Start =
                                                 // rLoopEdges[rlLineStartIndex]->GetStartPoint()
                                lLineEndIndex); // out: SplitLine End   =
                                                // rLoopEdges[rlLineStartIndex]->GetStartPoint()
      /*
      // when no splitLine was found (probably a nonConvex polygon)
      if (!bSuccess)
      {
          // try splitting while allowing for child loops that cross the splitPlane
          SmBoolean bUseLooseSideCheck = TRUE;
          SER(SmPolyFace::FindSplitLine(sPolyFaceEdges, // in : ordered target edges forming a polygon to be split
                                        sFaceNormal, // in : plane normal of the target loop
                                        &dLimitAspectRatio, // in : NULL = no min limit of output aspect ratios
                                        &bUseLooseSideCheck, // in : TRUE = only reject splits with child loops that
                                                             // touch the split line
                                        bSuccess, // out: TRUE = found a split line that satisfies split constraints,
                                                  // FALSE=didn't
                                        dMaxAspectRatio, // out: Aspect ratio for given split, else -SM_BIG_DOUBLE
                                        lLineStartIndex, // out: SplitLine Start =
                                                         // rLoopEdges[rlLineStartIndex]->GetStartPoint()
                                        lLineEndIndex)); // out: SplitLine End   =
                                                         // rLoopEdges[rlLineStartIndex]->GetStartPoint()

      } // end attempted split recovery branch
      */
      // if we found a good split line, split the face and add the new faces to the stack
      if (bSuccess)
      {
          // MakeManifoldEdge arguments
          SmPolyEdge* pNewEdge = NULL;
          SmPolyLoop* pNewLoop = NULL;
          SmPolyFace* pNewPolyFace = NULL;
          SmPolyEdge* pStartVertexEdge = sPolyFaceEdges[lLineStartIndex];
          SmPolyEdge* pEndVertexEdge = sPolyFaceEdges[lLineEndIndex];
          SmBoolean bDoSingularEdges = TRUE;
          SmPolyEdge* pNewSingularEdge = NULL;
          SmPolyVertex* pNewSingularVertex = NULL;

          // Now, split the face by the split line
          SER(pPolyFace->MakeManifoldEdge(pStartVertexEdge, // in : start of new edge =
                                                            // pStartVertexEdge->GetStartPolyVertex()
                                          pEndVertexEdge, // in : end of new edge   =
                                                          // pEndVertexEdge->GetStartPolyVertex()
                                          pNewEdge, // out: Edge going from Start to End (a radial mateEdge running
                                                    // from End to Start is also made)
                                          pNewLoop, // out: New PolyLoop or NULL when no new polyLoop was made
                                          pNewPolyFace, // out: New PolyFace or NULL when no new polyFace was made
                                          bDoSingularEdges, // in : TRUE = Split Singular Edge and connect to new
                                                            // PolyVertex if that can make NewEdge an IsoParamCurve
                                          &pNewSingularEdge, // out: New SingularEdge (and radial Mate) or NULL when
                                                             // no Singular Edge found needing splitting
                                          &pNewSingularVertex)); // out: New SingularVertex or NULL when no Singular
                                                                 // Edge found needing splitting

#ifdef SM_DEBUG_CODE
          SmBoolean bDebugMe = FALSE;
          if (bDebugMe)
          {
              SM_DUMP_AND_ASSERT_VALID(pPolyFace);
              SM_DUMP_AND_ASSERT_VALID(pNewPolyFace);
              // SM_DUMP_AND_ASSERT_VALID(GetPolyBrep());

              smgfx_Erase();
              smgfx_SetLook(1, 2, 1, 0, 0);
              pNewEdge->Draw();
              sm_GraphicsLoop();
              smgfx_SetLook(1, 2, 0, 0, 1);
              pPolyFace->Draw();
              sm_GraphicsLoop();
              smgfx_SetLook(1, 2, 0, 1, 1);
              pNewPolyFace->Draw();
              sm_GraphicsLoop();
              sm_GraphicsLoop();
          }
#endif // SM_DEBUG_CODE

          ULONG lNumSubdivisionsNeeded;
          SER(CheckAgainstSurfaceCache(pTessCache, // in : Cache with subdivision tree to interogate
                                       pNewEdge->GetStartPolyVertex()->GetPoint(), // in : start of UV segment
                                                                                   // to review
                                       pNewEdge->GetEndPolyVertex()->GetPoint(),   // in : end of UV segment to
                                                                                   // review
                                       1.1,  // in : allowed size variation before segment needs split
                                       TRUE, // in : TRUE = interior edge - don't check 3d lengths
                                             //      FALSE= exterior edge - do check 3d lengths
                                       lNumSubdivisionsNeeded)); // out: number of splits Start/End UV segment needs
                                                                 //      so segment sizes are about the smallest
                                                                 //      xsecting cache subdivision tree leaf node
                                                                 //      size.
          // remember which NewPolyEdges (and all their radial partners) need to be subdivided
          if (lNumSubdivisionsNeeded > 0)
          {
              if (lNumSubdivisionsNeeded > 20)
                  lNumSubdivisionsNeeded = 20;
              rPolyEdgeSubdivisions.Add(lNumSubdivisionsNeeded);
              rPolyEdgesToSubdivide.Add(pNewEdge);
          }

          // If added manifold PolyEdge pair splits face, call TriangluateFace on each child recursively
          if (pNewPolyFace != NULL)
          {
              rPolyFaceStack.Push(pPolyFace);
              rPolyFaceStack.Push(pNewPolyFace);
              return SM_SUCCESS;
          }
      }
  }
  // END JLMCC EDIT.
  #endif

  // upcoming - split general polygon into a set of triangles
  // next     - but first build a Spatial Tree of all PolyFace Vertices.  Tree will
  //            be used to find Vertices that interact with potential new PolyEdges cheaply.

  // locals
  SmBoolean   bDone     = FALSE;
  SmVector3d  sNormal(0,0,1);
  ULONG       lNumEdges = sPolyFaceEdges.GetSize(); // gwc: currently only of NonDegenerate PolyEdges

  // Begin Scope - Prepare for Triangulation iteration:
  // When needed - make rpVertexTree; init PolyEdge Index values and PolyVertex Score arrays
    {
      if ( !rpVertexTree )
        {
          // PolyFace bounding box
          SmExtent3d  sFaceBox;
          SER( pPolyFace->CalculateBoundingBox( sFaceBox ));

          // vertex spatial tree
          rpVertexTree = new (m_crContext) SmTree(sFaceBox); NER(rpVertexTree);

          // increment the locked mark value every time a new VertexTree is started
          rMarkLock.NewMark();    // gwc: used in upcoming FindInsideVertices call

          // Build the vertex tree - faster queries - many times.
          // gwc: currently only of NonDegenerate PolyEdges
          for ( ii=0; ii < sPolyFaceEdges.GetSize(); ii++ )
            {
              SmPolyEdge   * pPolyEdge = sPolyFaceEdges[ii];
              SmPolyVertex * pPolyVert = pPolyEdge->GetStartPolyVertex();
              if (pPolyVert)
              {
                  SmExtent3d sVBBox(pPolyVert->GetPoint());
                  sVBBox.ExpandAbsolute(pPolyVert->GetTolerance());

                  // load tree - Object = pPOlyEdge, BBox = PolyEdge->StartPt->Pos
                  SER(rpVertexTree->AddToSpatialTree(sVBBox, pPolyEdge));
              }

            } // end iter ii, every PolyFace->PolyEdge load VertexTree
        } // end need to build VertexTree check

      // Init PolyEdge-IndexValues. For PolyVertexData, sScores and sChordDist [pPolyFaceEdge->GetIndexValue()] array access
      // note: If we set the index values for only the edges in sPolyFaceEdges,
      // then SingularEdges don't get an Index value - okay if we never ask for it,
      // but it does happen.  [B456]
      // Note, this might not be good for the algorithm.  If we don't set Degenerate edges
      // to zero, this case will crash.  Perhaps we should run it that way to see
      // how it can happen.
      // SmTArray< SmPolyEdge* > sAllPolyEdges;
      // pPolyFace->GetPolyEdges( sAllPolyEdges );
      // ULONG lIdx=0;
      // for(ii=0; ii<sAllPolyEdges.GetSize(); ii++ )
      //   {
      //     SmPolyEdge *pPolyFaceEdge = sAllPolyEdges[ii];
      //     if ( SmTol::IsDegenerate(pPolyFaceEdge) )
      //       { pPolyFaceEdge->SetIndexValue( 0 ); }
      //     else
      //       { pPolyFaceEdge->SetIndexValue( lIdx++ ); }
      //   }

      // size and init the PolyVertexData arrays, indexed:[pPolyFaceEdge->GetIndexValue()]
      sScores           .SetSize( lNumEdges ) ; //sScores   .SetAll(0) ;
      sNumSubdivisions  .SetSize( lNumEdges ) ; //sScores   .SetAll(0) ;
      sChordDist        .SetSize( lNumEdges ) ; //sChordDist.SetAll(0) ;
#ifdef SM_DEBUG_CODE
      sHistory          .SetSize( lNumEdges ) ; //sHistory    .SetAll(0) ;
      sIterChanged      .SetSize( lNumEdges ) ; //sIterChanged.SetAll(0) ;
#endif // SM_DEBUG_CODE

    } // End Scope - Prepare for Triangulation iteration, build Vertex Spatial tree, assign PolyEdges Index Values

  // arrive here when:
  //    PolyFace polygon has at least 5 edges or is a nonConvex Quad.
  //    rpVertexTree = SmTree loaded with <pPolyFaceEdge, pPolyFaceEdge->StartPt->pos> pairs
  //    next - enter a while where each iter splits 1 triangle off polygon until polygon itself becomes a triangle

  // large value to label corners not suitable for splitting
  constexpr ULONG BAD_SCORE = 999999;
  ULONG        lNumIter  = 0 ;

  struct ScoreCalculationParameters { SmBoolean bAnglesIn3d;
                                      SmBoolean bInsideVertsIn3d;
                                      SmBoolean bAllowColinear;
                                      double    dWt3d;
                                    } ;

  // SmBoolean sbCalcAnglesIn3d       = FALSE;  // however: [B179 B181 B186] // [B157]
  // SmBoolean sbFindInsideVertsIn3d  = TRUE;   //          [B128]

  // We combine 2d and 3d angle measures.  [B128 157 179 181 186 346 364]
  // 0.5 gives equal weight to 2d and 3d angles.

  // Setup Experiment to make corner Score a weighted sum of 2d and 3d angles - used to only be 2d angles
  ScoreCalculationParameters sScoreParameters[] =
    {
//    {TRUE,  TRUE,  FALSE, 0.5},    // balanced
//    {TRUE,  FALSE, FALSE, 0.1},    // mostly 2d
//    {TRUE,  TRUE,  FALSE, 0.9},    // mostly 3d
//    {TRUE,  FALSE, FALSE, 0.0},    // only 2d with 3d inclusion test
      {FALSE, FALSE, FALSE, 0.0},    // only 2d
      {FALSE, FALSE, TRUE,  0.0}     // only 2d + 3d colinear
    } ;

  const ULONG lNumParameters = ((sizeof(sScoreParameters))/(sizeof(sScoreParameters[0]))) ;
  ULONG       lActParameters = 0;

  // End Setup Experiment to make corner Score a weighted sum of 2d and 3d angles - used to only be 2d angles

  // next - While Polygon has more than 3 edges
  //        Split off a triangle from Polygon at "best" polygon vertex corner
  //          by Split across corner OtherVertex Pts to form triangles.
  //        Each split creates one triangle and reduces this polygon's edge count by 1
  //        Best Vertex score is the vertex with the lowest score:
  //
  //        score = !IsConvexIn2d() || !IsConvexIn3d() ? BAD_SCORE : 0
  //        score += WasVertexSingleLoopV() ? 360.0 : 0.0 ;   // broken - only works for last SingleLoopV added to combined PolyLoop
  //        score += (dAngDeg2d + dAngDeg3d) / 2.0 ;
  //        score += 1000 * NumNeededSubdivisions for NewEdge ;
  //
  //        score = InsideVertices2d() || InsideVertices3d()  BAD_SCORE : 0


  // while PolyFace has more than 3 PolyEdges
  //   each iter finds best corner and checks the triangle [best corner's two PolyEdges, and Edge between best corner's neighbor vertices]
  //    when triangle contains no PolyVerts - Add manifold PolyEdge pair between neighbor vertices splitting polygon into triangle, and smaller polygon
  //    when triangle contains PolyVerts - Add manifold PolyEdge pair between best corner and contained vertex, may or may not split polygon.
  while (!bDone)
    {
label_TryAgain:
      lNumIter++ ;

#ifdef SM_DEBUG_CODE
      sIterChanged.SetAll(0) ;
#endif // SM_DEBUG_CODE


      // used to be: pPolyFace->GetPolyEdges(sPolyFaceEdges);
      lNumEdges = sPolyFaceEdges.GetSize();

      //           (PolyEdges are in CCW order)
      //           To get CW and CCW NonDegenerate Neighbor PolyEdges (skipping degenerate neighbors)
      //              CWPolyFaceEdge  = sPolyFaceEdges[((ii+lNumEdges)-1)%lNumEdges]
      //              TgtPolyFaceEdge = sPolyFaceEdges[ii]
      //              CCWPolyFaceEdge = sPolyFaceEdges[ii+1]
      //           To get CW and CCW immediate neighbors even when they happen to be Degenerate
      //              CWPolyFaceEdge  = TgtPolyFaceEdges->GetCWPolyEdge()
      //              TgtPolyFaceEdge = sPolyFaceEdges[ii]
      //              CCWPolyFaceEdge = TgtPolyFaceEdges->GetCCWPolyEdge()

      // exit - PolyFace has become a triangle or is degenerate
      if ( lNumEdges <= 3 )
        { break; }

      // locals for upcoming next SplitPolyEdge optimal EndPts seach
      ULONG  lBestScoreEdge = 0;
      ULONG  lBestScore     = BAD_SCORE;
      double dBestChordDist = SM_BIG_DOUBLE;

#ifdef SM_DEBUG_CODE
      SmBoolean bDebugMe0 = FALSE;
      constexpr ULONG lDebugBreak = BAD_SCORE; // (just something big)
      if ( lNumEdges == lDebugBreak )
        { dBestChordDist += 1.0; }  // (just for a breakpoint: won't change value)

      if (bDebugMe0)
        {
          smgfx_Erase();
          smgfx_SetLook(1,2, 0,0,0); pPolyFace->Draw( TRUE, FALSE ); sm_GraphicsLoop();
          smgfx_SetLook(1,2, 0,0,0); pPolyFace->Draw( TRUE, TRUE  ); sm_GraphicsLoop();
          smgfx_SetLook(1,2, 0,0,0); pPolyFace->m_pOriginalFace->Draw(SM_DM_WIREFRAME); sm_GraphicsLoop();
          sm_GraphicsLoop();

          smgfx_SetLook(2,4, 0,0,1); for ( ii=0; ii<lNumEdges; ii++ )
                                       { sPolyFaceEdges[ii]->Draw()  ; sm_GraphicsLoop();
                                         sPolyFaceEdges[ii]->Draw3D(); sm_GraphicsLoop();
                                         sm_GraphicsLoop();
                                       }
          sm_GraphicsLoop();
      }
#endif // SM_DEBUG_CODE

      SmPolyEdge * pInsideVertexEdge = NULL;

      // Begin Scope - To Find Best Corner Score
      {
          SmBoolean bRescored = FALSE;

          // for every PolyEdge->StartPt - score neighbor PolyVertex pairs to find next SplitPolyEdge optimal EndPts
          for (ii = 0; ii < lNumEdges; ii++)
          {
              // NonDegenerate PrevVertex and NextVertex to this Vertex in current PolyLoop locals
              SmPolyEdge* pV0Edge = sPolyFaceEdges[ii];
              SmPolyEdge* pVertexClosestToV0VCCW = NULL;
              SmPolyEdge* pVertexClosestToV0VCCW_3d = NULL;
              SmPolyEdge* pVCW_Edge = NULL;
              SmPolyEdge* pVCCW_Edge = NULL;

#ifdef SM_DEBUG_CODE
              SmBoolean bDebugMe5 = FALSE;
              SmBoolean bDebugMe5a = FALSE;
              if (bDebugMe5)
              {
                  if (bDebugMe5a)
                  {
                      smgfx_Erase();
                      smgfx_SetLook(2, 4, 0, 1, 0);
                      pPolyFace->Draw(TRUE, TRUE);
                      sm_GraphicsLoop();
                      smgfx_SetLook(3, 5, 0, 1, 0);
                      pPolyFace->m_pOriginalFace->Draw();
                      sm_GraphicsLoop();
                      sm_GraphicsLoop();
                  }
                  smgfx_SetLook(6, 8, 0, 1, 1);
                  pV0Edge->Draw();
                  sm_GraphicsLoop();
                  smgfx_SetLook(6, 8, 0, 1, 1);
                  pVCW_Edge->Draw();
                  sm_GraphicsLoop();
                  sm_GraphicsLoop();

                  smgfx_SetLook(6, 8, 0, 1, 1);
                  pV0Edge->Draw3D();
                  sm_GraphicsLoop();
                  smgfx_SetLook(6, 8, 0, 1, 1);
                  pVCW_Edge->Draw3D();
                  sm_GraphicsLoop();
                  sm_GraphicsLoop();
              }
#endif // SM_DEBUG_CODE

              // when PolyFaceEdge->Start Neighbor Vertex pair has no Score
              if (sScores[ii] == 0)
              {
                  bRescored = TRUE;

                  //if (pV0Edge->IsDegenerate())
                  //{
                  //    sScores[ii] = BAD_SCORE;
                  //    continue;
                  //}

                  pVCW_Edge = sPolyFaceEdges[((ii + lNumEdges) - 1) % lNumEdges];
                  pVCCW_Edge = sPolyFaceEdges[(ii + 1) % lNumEdges];
                  SM_ASSERT_MSG(pVCW_Edge == pV0Edge->GetCWPolyEdge() || SmTol::IsDegenerate(pV0Edge->GetCWPolyEdge()),
                                _T("TriangulateSingleFace:: Bug in Array order assumption"));
                  SM_ASSERT_MSG(pVCCW_Edge == pV0Edge->GetCCWPolyEdge() || SmTol::IsDegenerate(pV0Edge->GetCCWPolyEdge()),
                                _T("TriangulateSingleFace:: Bug in Array order assumption"));
#ifdef SM_DEBUG_CODE
                  SmBoolean bDebugEdge = FALSE;
                  if (bDebugEdge)
                  {
                      if (!(pVCW_Edge == pV0Edge->GetCWPolyEdge() || SmTol::IsDegenerate(pV0Edge->GetCWPolyEdge())))
                      {

                          ULONG lVCW_index;
                          ULONG lVCW_real;
                          SmBoolean bDegen = SmTol::IsDegenerate(pV0Edge->GetCWPolyEdge());
                          SM_REF1(bDegen);
                          sPolyFaceEdges.FindElement(pVCW_Edge, lVCW_index);
                          sPolyFaceEdges.FindElement(pV0Edge->GetCCWPolyEdge(), lVCW_real);
                      }
                      if (!(pVCCW_Edge == pV0Edge->GetCCWPolyEdge() || SmTol::IsDegenerate(pV0Edge->GetCCWPolyEdge())))
                      {

                          ULONG lVCCW_index;
                          ULONG lVCCW_real;
                          SmBoolean bDegen = SmTol::IsDegenerate(pV0Edge->GetCCWPolyEdge());
                          SM_REF1(bDegen);
                          sPolyFaceEdges.FindElement(pVCCW_Edge, lVCCW_index);
                          sPolyFaceEdges.FindElement(pV0Edge->GetCCWPolyEdge(), lVCCW_real);
                      }
                  }
#endif
                  // SmPolyEdge   *pVCW_Edge    = pV0Edge  ->GetCWPolyEdge();    // gwc: removed to skip over degenerate
                  // PolyEdges SmPolyEdge   *pVCCW_Edge   = pV0Edge  ->GetCCWPolyEdge();   // gwc: removed to skip over
                  // degenerate PolyEdges

                  SmPolyVertex* pVCW_Vertex = pVCW_Edge->GetStartPolyVertex();
                  SmPolyVertex* pVCCW_Vertex = pV0Edge->GetEndPolyVertex(); // gwc: not always the same as
                                                                            // pVCCW_Edge->GetStartPolyVertex() with
                                                                            // singular edges

                  ULONG lScore = 0;
#ifdef SM_DEBUG_CODE
                  ULONG lHistory = 0;
#endif // SM_DEBUG_CODE
                  double dAngDeg2d = 0.0;
                  double dAngDeg3d = 0.0;
                  double dChordDistUV = 0.0;

                  // get dAngDeg2d and if it's not convex in 2d we want a bad score
                  if (!pV0Edge->IsConvexCorner(sNormal, dAngDeg2d, dChordDistUV, dAngDeg3d, pVCW_Edge))
                  {
                      if (!sScoreParameters[lActParameters].bAllowColinear || !IsCollinear(dAngDeg2d))
                      {
                          lScore = BAD_SCORE;
#ifdef SM_DEBUG_CODE
                          lHistory += SM_SCORE_NOT_CONVEX2d * lIterOffset;
#endif // SM_DEBUG_CODE
                      }
                  }

                  // get dAngDeg3d and if it's not convex in 3d we definitely want a bad score
                  if (sScoreParameters[lActParameters].bAnglesIn3d && dAngDeg3d > 179.999)
                  {
                      if (!sScoreParameters[lActParameters].bAllowColinear || !IsCollinear(dAngDeg3d))
                      {
                          lScore = BAD_SCORE;
#ifdef SM_DEBUG_CODE
                          lHistory += SM_SCORE_NOT_CONVEX3d * lIterOffset;
#endif // SM_DEBUG_CODE
                      }
                  }

                  // when current corner was a SingleLoopV before being connected into the loop system, gwc: I don't see
                  // the wisdom in this choice
                  if (sSingleVertices.IsIn(pV0Edge->GetStartPolyVertex()))
                  {
                      // Weight this one up a lot (so its less likely to be used)
                      // and we should be able to tessellate around a vertex all of the way.
                      dAngDeg2d += 360.0;
#ifdef SM_DEBUG_CODE
                      lHistory += SM_SCORE_SINGLE_VERTEX * lIterOffset;
#endif // SM_DEBUG_CODE
                  }

                  // Combine 2d and 3d angles according to dWt3d.
                  // Smaller angle better score.
                  lScore += (ULONG)((1.0 - sScoreParameters[lActParameters].dWt3d) * smos_Ceil(dAngDeg2d) +
                                    sScoreParameters[lActParameters].dWt3d * smos_Ceil(dAngDeg3d));

#ifdef SM_USE_TESS_DEGREE
                  {
                      // BEGIN JLMCC EDIT: Add vertex connectivity to score. Higher score means less likely to use,
                      // this should help avoid fans.

                      SmTArray<SmPolyEdge*> sPolyEdgesVCW_Vertex;
                      SmTArray<SmPolyEdge*> sPolyEdgesVCCW_Vertex;

                      pVCW_Vertex->GetPolyEdgesOfFace(pPolyFace, sPolyEdgesVCW_Vertex);
                      pVCCW_Vertex->GetPolyEdgesOfFace(pPolyFace, sPolyEdgesVCCW_Vertex);

                      ULONG lNumEdgesAdjacent = sPolyEdgesVCW_Vertex.GetSize();
                      lNumEdgesAdjacent += sPolyEdgesVCCW_Vertex.GetSize();

                      lScore += 30 * lNumEdgesAdjacent;
                  }
                  // END JLMCC EDIT
#endif
#ifdef SM_DEBUG_CODE
                  lHistory += SM_SCORE_ANGLES * lIterOffset;
#endif // SM_DEBUG_CODE

                  // Subdivision produces higher score (less likely to be used)
                  ULONG lNumSubdivisionsNeeded;
                  SER(CheckAgainstSurfaceCache(pTessCache, // in : Cache with subdivision tree to interogate
                                               pVCCW_Vertex->GetPoint(), // in : start of UV segment to review
                                               pVCW_Vertex->GetPoint(), // in : end of UV segment to review
                                               1.1, // in : allowed size variation before segment needs split
                                               TRUE, // in : TRUE = interior edge - don't check 3d lengths
                                                     //      FALSE= exterior edge - do check 3d lengths
                                               lNumSubdivisionsNeeded)); // out: number of splits Start/End UV segment
                                                                         // needs
                                                                         //      so segment sizes are about the smallest
                                                                         //      xsecting cache subdivision tree leaf
                                                                         //      node size.

                  // weight subdivision so it's more of a deterent than the angle value
                  // GWC:  Is it possible to pick a concave corner due to this?  Not likely a concave corner is set to
                  // 999,999

                  if (lNumSubdivisionsNeeded > 0)
                  {
                      lScore += lNumSubdivisionsNeeded * 1000;
                      sNumSubdivisions[ii] = lNumSubdivisionsNeeded;
#ifdef SM_DEBUG_CODE
                      lHistory += SM_SCORE_SUBDIVISIONS * lIterOffset;
#endif // SM_DEBUG_CODE
                  }

                  // See if proposed triangle contains any other PolyFace vertices
                  //  cbi Possible optimization: need only one inside vtx, not all of them. gwc: agreed.
                  //  gwc: this needs to be extended to check for all bad cases.
                  //        o. Proposed PolyFaces have opposite normals
                  //             - ex: concave quad is split across wrong pair of end points
                  //        o. Proposed PolyFaces Intersect one another
                  //             - ex: remote vertex is within proposed triangle which implies new triangle
                  //                   boundary intersects old polygon edges
                  //             - note: in UV space if orig Polygon does not self intersect
                  //                     can not propose a new triangle that intersects old polygon
                  //                     edges without new triangle containing old polygon vertices.
                  //        o. Proposed PolyEdges have chord heights within tolerance
                  //           - internal case: chord heights measured along a surface normal
                  //           - boundary case when tessellation edge walks outside surface domain:
                  //              chord height measured to closest boundary point (not along a normal)
                  //
                  SER(pPolyFace->FindInsideVertices(pV0Edge, // in : V0 of Tri[P0=V0->StartPt, P1=V1->StartPt,
                                                             // P2=V2->StartPt]
                                                    pVCCW_Edge, // in : V1 of Tri[P0=V0->StartPt, P1=V1->StartPt,
                                                                // P2=V2->StartPt]
                                                    pVCW_Edge, // in : V2 of Tri[P0=V0->StartPt, P1=V1->StartPt,
                                                               // P2=V2->StartPt]
                                                    rpVertexTree, // in : vertex spatial tree, can narrow search if
                                                                  // present
                                                    sInsidePolyVerts, // out: PolyEdges with inside StartPts in
                                                                      // Tri[P0,P1,P2]
                                                    pVertexClosestToV0VCCW, // out: PolyEdge  with inside StartPt
                                                                            // closest to PEdge[P0,P1], NULL for none
                                                    /* GWC needs work */ sInsidePolyVerts_3d, // out: PolyEdges with
                                                                                              // inside StartPts
                                                                                              // Tri[pOrigSurf(P0,P1,P2)]
                                                    /* GWC needs work */ pVertexClosestToV0VCCW_3d, // out: PolyEdge
                                                                                                    // with inside
                                                                                                    // StartPt closest
                                                                                                    // to
                                                                                                    // PEdge[pOrigSurf(P0,P1)],
                                                                                                    // NULL for none
                                                    sScoreParameters[lActParameters].bInsideVertsIn3d, // in : TRUE=Do
                                                                                                       // 3D tests, load
                                                                                                       // xx_3d outputs,
                                                                                                       // FALSE=Skip 3D
                                                                                                       // tests
                                                    rMarkLock.GetMarkType())); // in : checks without incrementing
                                                                               // eMarkType Value
                  // When any vertices are inside proposed triangle: bad score.
                  if (sInsidePolyVerts.GetSize() > 0 || sInsidePolyVerts_3d.GetSize() > 0)
                  {
                      lScore = BAD_SCORE;
#ifdef SM_DEBUG_CODE
                      lHistory += SM_SCORE_INSIDE_VERTICES * lIterOffset;
#endif // SM_DEBUG_CODE
                  }

                  // save scores in PolyFaceVertexData arrays
                  sScores[ii] = lScore; //  (bInsideVertices) ? BAD_SCORE
                                        // :  ((!IsConvex2D || !IsConvex3D) ? BAD_SCORE : 0)
                                        //  + (1.0-dWt3d)*2d AngleDeg
                                        //  + (  dWt3d  )*3d AngleDeg
                                        //  + (bSingleLoopV ? 360 : 0)
                                        //  + 1000 * lNumSplitsToBeLinear

                  sChordDist[ii] = dChordDistUV; // Dist:[PrevVertPt, NextVertPt] of this corner vertex in PolyLoop
#ifdef SM_DEBUG_CODE
                  sHistory[ii] = lHistory;
                  sIterChanged.SetAt(ii, TRUE);
#endif // SM_DEBUG_CODE

              } // end need PolyFaceEdge->Start Neighbor Vertex pair score check

              // Save best score - ties go to smallest UV space dBestChordDist.  gwc: ties should go to be best
              // base/chord dist ratios
              if ((sScores[ii] < lBestScore) || (sScores[ii] == lBestScore && sChordDist[ii] < dBestChordDist))
              {
                  lBestScore = sScores[ii]; //  (bInsideVertices) ? BAD_SCORE
                                            // :  ((!IsConvex2D || !IsConvex3D) ? BAD_SCORE : 0)
                                            //  + (1.0-dWt3d)*2d AngleDeg
                                            //  + (  dWt3d  )*3d AngleDeg
                                            //  + (bSingleLoopV ? 360 : 0)
                                            //  + 1000 * lNumSplitsToBeLinear
                  lBestScoreEdge = ii;
                  dBestChordDist = sChordDist[ii];
                  if (!bRescored)
                  {
                      pVCW_Edge = sPolyFaceEdges[((ii + lNumEdges) - 1) % lNumEdges];
                      pVCCW_Edge = sPolyFaceEdges[(ii + 1) % lNumEdges];

                      SER(pPolyFace->FindInsideVertices(
                          pV0Edge, // in : V0 of Tri[P0=V0->StartPt, P1=V1->StartPt,
                                   // P2=V2->StartPt]
                          pVCCW_Edge, // in : V1 of Tri[P0=V0->StartPt, P1=V1->StartPt,
                                      // P2=V2->StartPt]
                          pVCW_Edge, // in : V2 of Tri[P0=V0->StartPt, P1=V1->StartPt,
                                     // P2=V2->StartPt]
                          rpVertexTree, // in : vertex spatial tree, can narrow search if
                                        // present
                          sInsidePolyVerts, // out: PolyEdges with inside StartPts in
                                            // Tri[P0,P1,P2]
                          pVertexClosestToV0VCCW, // out: PolyEdge  with inside StartPt
                                                  // closest to PEdge[P0,P1], NULL for none
                          /* GWC needs work */ sInsidePolyVerts_3d, // out: PolyEdges with
                                                                    // inside StartPts
                                                                    // Tri[pOrigSurf(P0,P1,P2)]
                          /* GWC needs work */ pVertexClosestToV0VCCW_3d, // out: PolyEdge
                                                                          // with inside
                                                                          // StartPt closest
                                                                          // to
                                                                          // PEdge[pOrigSurf(P0,P1)],
                                                                          // NULL for none
                          sScoreParameters[lActParameters].bInsideVertsIn3d, // in : TRUE=Do
                                                                             // 3D tests, load
                                                                             // xx_3d outputs,
                                                                             // FALSE=Skip 3D
                                                                             // tests
                          rMarkLock.GetMarkType())); // in : checks without incrementing
                                                     // eMarkType Value

                      
                  }
                  pInsideVertexEdge = pVertexClosestToV0VCCW ? pVertexClosestToV0VCCW : pVertexClosestToV0VCCW_3d;
              }
          } // end iter ii, every PolyEdge->StartPt - scoring neighbor PolyVertex pairs to find next SplitPolyEdge
            // optimal EndPts
      
          // score == (bInsideVertices) ? BAD_SCORE
          //           :  ((!IsConvex2D || !IsConvex3D) ? BAD_SCORE : 0)
          //            + (1.0-dWt3d)*2d AngleDeg
          //            + (  dWt3d  )*3d AngleDeg
          //            + (bSingleLoopV ? 360 : 0)
          //            + 1000 * lNumSplitsToBeLinear

          // Begin Experiment - when best score is BAD_SCORE
          //                     try recalculating scores with different 2d/3d angle weights.
          //   gwc: Experiment does not look like a help because a best score of at least BAD_SCORE
          //        only comes when all corners are !convex or contain other vertices or proposed new PolyEdge
          //        needs too many subdivisions.  Changing the weights on adding 2d and 3d angles to score
          //        won't change those problems.
          //   GWC: Looking at current score formula, best score >= BAD_SCORE can happen when
          //         1. all corners contain vertices (is this physically possible? - I don't think so)
          //         2. all other corners are !convex. (a convex "hole" would have all !convex corners,
          //                                           but the holes have been connected to the outerloop
          //                                           and there should be no more holes, so unless
          //                                           a poor choice was made for a previous triangle, holes
          //                                           should not lead to this condition.)
          //         3. The NumSplitsToBeLinear is greater than 1000 (when BAD_SCORE == 999999)
          //                                      (gwc: can this happen? perhaps on a degenerate surface)
          if ( lBestScore >= BAD_SCORE )
            {
              MSG(_T("TriangulateFace: no good polygon split, all corners are !convex or contain OtherVerts, or SplitEdge needs at least 1000 subdivisions. A problem."));

#ifdef SM_DEBUG_CODE
              SmBoolean bDebugMe7 = FALSE;
              // Inspect bad ones.
              if ( bDebugMe7 )
                {

                  sm_DumpScoreHistory(lNumIter, lBestScoreEdge, sPolyFaceEdges, sScores, sHistory, sIterChanged, FALSE) ; // FALSE= only Dump changed scores
                  sm_DumpScoreHistory(lNumIter, lBestScoreEdge, sPolyFaceEdges, sScores, sHistory, sIterChanged, TRUE) ;  // TRUE = dump all scores
                  SM_OBJ_ARRAY(sPoints2d, SmPoint3d, 16) ;
                  SM_OBJ_ARRAY(sPoints3d, SmPoint3d, 16) ;
                  pPolyFace->GetOuterPolyLoop()->GetPoints(sPoints2d) ;
                  sPoints3d.SetSize(sPoints2d.GetSize());
                  if(pPolyFace->GetOKBackPtrs() && pOriginalFace->GetSurface())
                    {
                      for(ii=0;ii<sPoints2d.GetSize();ii++)
                        {
                          SmPoint2d sUV(sPoints2d[ii].x, sPoints2d[ii].y) ;
                          pOriginalFace->GetSurface()->EvaluatePoint(sUV, sPoints3d[ii]) ;
                        }
                    }

                  // expect all scores to be large - we need to understand why
                  sm_DumpScoreHistory(lNumIter, lBestScoreEdge, sPolyFaceEdges, sScores, sHistory, sIterChanged, FALSE) ; // FALSE= only Dump changed scores
                  sm_DumpScoreHistory(lNumIter, lBestScoreEdge, sPolyFaceEdges, sScores, sHistory, sIterChanged, TRUE) ;  // TRUE = dump all scores
                  sPoints2d.Dump() ;
                  sPoints3d.Dump() ;

                  // locals
                  SmFace       * pOrigFace = pPolyFace->GetOKBackPtrs() ? pPolyFace->GetOriginalFace() : NULL ;
                  SmBrep       * pOrigBrep = pOrigFace ? pOrigFace->GetBrep() : NULL ;
                  //SmSurface    * pOrigSurf = pOrigFace ? pOrigFace->GetSurface() : NULL ;

                  SmPolyEdge   * pV0Edge      = sPolyFaceEdges[lBestScoreEdge];
                  SmPolyEdge   * pVCW_Edge    = pV0Edge  ->GetCWPolyEdge();
                  SmPolyEdge   * pVCCW_Edge   = pV0Edge  ->GetCCWPolyEdge();
                  SmPolyVertex * pV0Vertex    = pV0Edge  ->GetStartPolyVertex();
                  SmPolyVertex * pVCW_Vertex  = pVCW_Edge ->GetStartPolyVertex();
                  SmPolyVertex * pVCCW_Vertex = pVCCW_Edge->GetStartPolyVertex();

                  smgfx_Erase();
                  // Draw Face and PolyFace
                  smgfx_SetLook(2,4, 0,1,1); if(pOrigBrep) pOrigBrep->Draw(TRUE); sm_GraphicsLoop();
                  smgfx_SetLook(1,2, 0,1,1); if(pOrigFace) pOrigFace->DrawUV(8,8,FALSE,NULL,TRUE); sm_GraphicsLoop();
                  smgfx_SetLook(4,6, 0,0,0); if(pPolyFace) pPolyFace->Draw(FALSE, TRUE); sm_GraphicsLoop() ;
                  sm_GraphicsLoop();

                  // Draw Best Corner edges
                  smgfx_SetLook(6,8, 1,0,0);  if(pVCW_Edge   ) pVCW_Edge    ->Draw(FALSE, TRUE) ; sm_GraphicsLoop() ;
                  smgfx_SetLook(6,8, 0,1,0);  if(pV0Edge     ) pV0Edge      ->Draw(FALSE, TRUE) ; sm_GraphicsLoop() ;
                  smgfx_SetLook(6,8, 0,0,1);  if(pVCCW_Edge  ) pVCCW_Edge   ->Draw(FALSE, TRUE) ; sm_GraphicsLoop() ;

                  // Draw Best Corner vertices
                  smgfx_SetLook(3,10, 1,0,0);  if(pVCW_Vertex ) pVCW_Vertex  ->Draw(FALSE, TRUE) ; sm_GraphicsLoop() ;
                  smgfx_SetLook(3,10, 0,1,0);  if(pV0Vertex )   pV0Vertex    ->Draw(FALSE, TRUE) ; sm_GraphicsLoop() ;
                  smgfx_SetLook(3,10, 0,0,1);  if(pVCCW_Vertex) pVCCW_Vertex ->Draw(FALSE, TRUE) ; sm_GraphicsLoop() ;
                  sm_GraphicsLoop();

                  // Draw all vertices
                  smgfx_SetLook(5,13, 1,0,1);  for(ii=0;ii<sPolyFaceEdges.GetSize();ii++)
                                                 { if(sPolyFaceEdges[ii]) sPolyFaceEdges[ii]->GetStartPolyVertex()->Draw(FALSE,TRUE); sm_GraphicsLoop() ; }
                  smgfx_SetLook(5,16, 0,1,0);  for(ii=0;ii<sPoints3d.GetSize();ii++)
                                                 { sPoints3d[ii].Draw(); sm_GraphicsLoop() ; }
                  sm_GraphicsLoop();
                }
#endif // SM_DEBUG_CODE

              if ( lBestScore == BAD_SCORE )
                {
                  // Reset all scores so they get recalculated with different 2d/3d weight.
                  ULONG lNumScores = sScores.GetSize();
                  for ( jj = 0; jj < lNumScores; jj++ )
                    { sScores[jj] = 0; }

                  // try recomputing scores using different 2d/3d angle weights
                  if ( lActParameters < lNumParameters - 1 )
                    {
                      ++lActParameters;
                      goto label_TryAgain;    // <=== HEADS UP - A GOTO JUMP
                    }
                  else
                    { break; } // already tried that trick.
                } // end if best score == BAD_SCORE
            } // end if best score >= BAD_SCORE - just here for debugging
      } // End Scope - To Find Best Corner Score

      // arrive here when we're done with lScore, and inside the while PolygonEdgeCount > 3 loop.
      //   lBestScoreEdge    = sPolyFaceEdges index of PolyEdge whose start Vertex labels the corner to be turned into a triangle
      //                       by adding a manifold PolyEdge pair between its neighbor vertices.
      //   sPolyFaceEdges    = Array of all PolyEdges still in the PolyLoop being triangulated
      //   sScores           = Array with a score value for every PolyEdge still in the PolyLoop being triangulated
      //   sHistory          = only in SM_DEBUG_CODE mode - the history of all choices that make up sScores values.
      //   sNumSubdivisions  = the number of subdivisions required for the edge.
      //   pInsideVertexEdge = NULL    = NoVertex in proposed Triangle; Add manifold PolyEdge pair between best corner's neighbor vertices.
      //                       NotNULL = pInsideVertexEdge->StartVertex is in proposed triangle;
      //                                 Add manifold PolyEdge pair between best corner and this start vertex.

      // next: add a manifold PolyEdge pair either to split off a triangle from current Polygon or connect best corner to an inside vertex

      // common locals for both pInsideVertexEdge == NULL and pInsideVertexEdge != NULL branches
      SmPolyEdge * pBestEdge = sPolyFaceEdges[lBestScoreEdge];

      SmPolyEdge   * pNewPolyEdge  = NULL ;
      SmPolyLoop   * pNewPolyLoop  = NULL ;
      SmPolyFace   * pNewPolyFace  = NULL ;
      SmPolyEdge   * pNewSingularEdge = NULL ;
      SmPolyVertex * pNewSingularVertex = NULL ;

#ifdef SM_DEBUG_CODE
      SmBoolean bDebugMe8 = FALSE;
      SmTArray<SmPolyLoop*> sDBGPolyLoops ;
      pPolyFace->GetPolyLoops(sDBGPolyLoops) ;

      if (sDBGPolyLoops.GetSize() > 0)
      {
            SmPolyLoop* pPolyLoop = sDBGPolyLoops[0];

            if (bDebugMe8)
            {
                sm_DumpScoreHistory(lNumIter, lBestScoreEdge, sPolyFaceEdges, sScores, sHistory, sIterChanged,
                                    FALSE); // FALSE= only Dump changed scores
                sm_DumpScoreHistory(
                    lNumIter, lBestScoreEdge, sPolyFaceEdges, sScores, sHistory, sIterChanged, TRUE); // TRUE = dump all
                                                                                                      // scores

                SmFace* pOrig = pPolyFace->GetOKBackPtrs() ? pPolyFace->GetOriginalFace() : NULL;
                SmSurface* pSurface = pOrig ? pOrig->GetSurface() : NULL;
                SmSurface* pCopySurface = NULL;
                SmBSplineSurface* pBSplineSurface = NULL;
                SmObjDelete sClean;
                if (pSurface)
                {
                    pSurface->Copy(*pSurface->GetContext(), pCopySurface);
                    sClean.SetObj(pCopySurface);
                    pBSplineSurface = SM_CAST_PTR(SmBSplineSurface, pCopySurface);
                    if (pBSplineSurface)
                    {
                      SmPoint3d sPoint;
                      double dWt;
                      pBSplineSurface->GetControlPoint(SM_CP_NON_RATIONAL, 0, 0, sPoint, dWt);
                      sPoint = -sPoint;
                      SmAxis2Placement sTranslate;
                      sTranslate.Translate(sPoint);
                      pBSplineSurface->Transform(sTranslate);
                      SM_ASSERT_VALID(pBSplineSurface);
                    }
                }
                SM_ASSERT_VALID(pSurface);
            }

            SmBoolean bDebugMe8a = FALSE;
            if (bDebugMe8a)
            {
                SmFace* pOrig = pPolyFace->GetOKBackPtrs() ? pPolyFace->GetOriginalFace() : NULL;
                SmSurface* pSurface = pOrig ? pOrig->GetSurface() : NULL;

                smgfx_Erase();
                smgfx_SetLook(1, 2, 0, 0, 0);
                pPolyFace->Draw(TRUE, FALSE);
                sm_GraphicsLoop();
                smgfx_SetLook(1, 2, 0, 0, 0);
                pPolyFace->Draw(TRUE, TRUE);
                sm_GraphicsLoop();
                smgfx_SetLook(1, 2, 0, 0, 1);
                if (pPolyLoop)
                    pPolyLoop->Draw(TRUE, FALSE);
                sm_GraphicsLoop();
                smgfx_SetLook(1, 2, 0, 0, 1);
                if (pPolyLoop)
                    pPolyLoop->Draw(FALSE, TRUE);
                sm_GraphicsLoop();
                smgfx_SetLook(1, 2, 0, 0, 0);
                if (pOrig)
                    pOrig->Draw(SM_DM_WIREFRAME);
                sm_GraphicsLoop();
                if (FALSE)
                {
                    smgfx_SetLook(1, 2, 0, 0, 0);
                    if (pSurface)
                      pSurface->Draw();
                    sm_GraphicsLoop();
                    smgfx_SetLook(1, 2, 0, 0, 0);
                    if (pSurface)
                      pSurface->DrawUV();
                    sm_GraphicsLoop();
                    smgfx_SetLook(1, 2, 0, 0, 0);
                    if (pSurface)
                      pSurface->DrawPolygon();
                    sm_GraphicsLoop();
                    smgfx_SetLook(1, 2, 0, 0, 0);
                    if (pSurface)
                      pSurface->DrawControlPoints();
                    sm_GraphicsLoop();
                }

                smgfx_SetLook(6, 10, 1, 0, 1);
                pBestEdge->Draw();
                sm_GraphicsLoop();
                smgfx_SetLook(6, 10, 1, 0, 1);
                pBestEdge->Draw3D();
                sm_GraphicsLoop();
                sm_GraphicsLoop();
            }
      }
#endif // SM_DEBUG_CODE

      // when pInsideVertexEdge == NULL, Add manifold PolyEdge pair between best corner's neighbor vertices
      // splitting polygon into a triangle child and a polygon child (with one fewer polyedges)
      if ( pInsideVertexEdge == NULL )
      {
          // score == (bInsideVertices) ? BAD_SCORE
          //           :  ((!IsConvex2D || !IsConvex3D) ? BAD_SCORE : 0)
          //            + (1.0-dWt3d)*2d AngleDeg
          //            + (  dWt3d  )*3d AngleDeg
          //            + (bSingleLoopV ? 360 : 0)
          //            + 1000 * lNumSplitsToBeLinear

          ULONG lScore = sScores[lBestScoreEdge ];
          if ( lScore > 99999 ) { lScore = lScore % 100000; } // remove a BAD_SCORE value that might have been added to lScore
          ULONG lNumSubdivisionsNeeded = sNumSubdivisions[lBestScoreEdge]; 

          // cbi 456  flip comments (1st 4 for last 2):
          // Get NonDegen PolyEdges whose StartVertices along with the BestEdge->StartVertex make the triangle about to be split off from polygon
          SmPolyEdge * pVCWEdge      = sPolyFaceEdges[((lBestScoreEdge+lNumEdges)-1)%lNumEdges] ;
          SmPolyEdge * pVCCWEdge     = sPolyFaceEdges[(lBestScoreEdge+1)%lNumEdges] ;
          SM_ASSERT(pBestEdge->GetCWPolyEdge() == pVCWEdge   || SmTol::IsDegenerate(pBestEdge->GetCWPolyEdge())) ;
          SM_ASSERT(pBestEdge->GetCCWPolyEdge() == pVCCWEdge || SmTol::IsDegenerate(pBestEdge->GetCCWPolyEdge())) ;
          //SmPolyEdge * pVCWEdge      = pBestEdge->GetCWPolyEdge();
          //SmPolyEdge * pVCCWEdge     = pBestEdge->GetCCWPolyEdge();

          // Mark BestEdge (gwc: where are marks being used? In the vertex tree, unclear if this is necessary though.).

          pBestEdge->Mark(rMarkLock.GetMarkType());

          // Create a manifold PolyEdge pair between a pair of given PolyEdge->StartVertices in a PolyFace.
          SER( pPolyFace->MakeManifoldEdge(pVCWEdge,               // in : start of new edge = pStartVertexEdge->GetStartPolyVertex()
                                           pVCCWEdge,              // in : end of new edge   = pEndVertexEdge->GetStartPolyVertex()
                                           pNewPolyEdge,           // out: Edge going from Start to End (a radial mateEdge running from End to Start is also made)
                                           pNewPolyLoop,           // out: New PolyLoop or NULL when no new polyLoop was made
                                           pNewPolyFace,           // out: New PolyFace or NULL when no new polyFace was made
                                           FALSE, // in : TRUE = Split Singular Edge and connect to new
                                                          // PolyVertex if that can make NewEdge an IsoParamCurve
                                           &pNewSingularEdge,      // out: New SingularEdge or NULL when no Singular Edge found needing splitting
                                                                   //      (a radial mateEdge running from End to Start is also made)
                                           &pNewSingularVertex,   // out: New SingularVertex or NULL when no Singular Edge found needing splitting
                                           pBestEdge));

          // gwc: when MakeManifoldEdge changes the inserted PolyEdge connecting to a Singular PolyEdge
          //      then recompute lNumSubdivisionsNeeded here.

          // clear scores for BestEdge and its neighbors
          // first, we will let the new polyedge take the place of pBestEdge, which is going to
          // be split off into a triangle.
          
          sScores[(lBestScoreEdge) % lNumEdges] = 0; // Need to recompute these, 0 is the init value
          sNumSubdivisions[(lBestScoreEdge) % lNumEdges] = 0;
          sChordDist[(lBestScoreEdge) % lNumEdges] = 0.0;
          sPolyFaceEdges[(lBestScoreEdge) % lNumEdges] = pNewPolyEdge;

          // reset the score for the ccw edge

          sScores[(lBestScoreEdge+1) % lNumEdges] = 0; // Need to recompute these, 0 is the init value
          sNumSubdivisions[(lBestScoreEdge+1) % lNumEdges] = 0;
          sChordDist[(lBestScoreEdge+1) % lNumEdges] = 0.0;

          // and the cw edge from pNewPolyEdge

          sScores[((lBestScoreEdge + lNumEdges) - 2) % lNumEdges] = 0; // Need to recompute these, 0 is the initvalue
          sNumSubdivisions[((lBestScoreEdge + lNumEdges) - 2) % lNumEdges] = 0;
          sChordDist[((lBestScoreEdge + lNumEdges) - 2) % lNumEdges] = 0.0;

          // remove old cw edge, it is in the new triangle with the radial of pNewPolyEdge and pBestEdge

          sScores.RemoveAt(((lBestScoreEdge + lNumEdges) - 1) % lNumEdges);
          sNumSubdivisions.RemoveAt(((lBestScoreEdge + lNumEdges) - 1) % lNumEdges);
          sChordDist.RemoveAt(((lBestScoreEdge + lNumEdges) - 1) % lNumEdges);
          sPolyFaceEdges.RemoveAt(((lBestScoreEdge + lNumEdges) - 1) % lNumEdges);

          // check if we need to subdivide...

          if (pNewSingularEdge != NULL)
          {
              SER(CheckAgainstSurfaceCache(pTessCache, // in : Cache with subdivision tree to interogate
                                           pNewPolyEdge->GetStartPolyVertex()->GetPoint(), // in : start of UV
                                                                                           // segment to review
                                           pNewPolyEdge->GetEndPolyVertex()->GetPoint(), // in : end of UV segment to
                                                                                         // review
                                           1.1, // in : allowed size variation before segment needs split
                                           TRUE, // in : TRUE = interior edge - don't check 3d lengths
                                                 //      FALSE= exterior edge - do check 3d lengths
                                           lNumSubdivisionsNeeded)); // out: number of splits Start/End UV segment
                                                                     // needs
                                                                     //      so segment sizes are about the smallest
                                                                     //      xsecting cache subdivision tree leaf
                                                                     //      node size.


              // remember which NewPolyEdges need to be subdivided
              if (lNumSubdivisionsNeeded > 0)
              {
                  if (lNumSubdivisionsNeeded > 20)
                  {
                      lNumSubdivisionsNeeded = 20;
                  }
                  rPolyEdgeSubdivisions.Add(lNumSubdivisionsNeeded);
                  rPolyEdgesToSubdivide.Add(pNewPolyEdge);
              }
          }
#ifdef SM_DEBUG_CODE
          // keep history and sIterChanged for two new entries
          sHistory.Add(0) ; sHistory.Add(0) ;
          sIterChanged.Add(1) ; sIterChanged.Add(1) ;
#endif // SM_DEBUG_CODE

          // Add <PolyEdge, PolyEdge->StartPt->Pos> pairs to VertexTree for NewPolyEdge and RadialPartner
          SER( sm_AddEdgeToVertexTree( pNewPolyEdge, rpVertexTree ));
          SER( sm_AddEdgeToVertexTree( pNewPolyEdge->GetRadial(), rpVertexTree ));
      } // end pInsideVertexEdge == NULL, Split off triangle with added manifold PolyEdge pair branch
      else  // pInsideVertexEdge != NULL - add manifold PolyEdge pair between best corner and vertex within the proposed triangle
      {
          // Make an edge between the vertices of pInsideVertexEdge and pBestEdge.

          // Create a manifold PolyEdge pair between a pair of given PolyEdge->StartVertices in a PolyFace.
          SER( pPolyFace->MakeManifoldEdge
                            (pBestEdge,              // in : start of new edge = pStartVertexEdge->GetStartPolyVertex()
                             pInsideVertexEdge,      // in : end of new edge   = pEndVertexEdge->GetStartPolyVertex()
                             pNewPolyEdge,           // out: Edge going from Start to End (a radial mateEdge running from End to Start is also made)
                             pNewPolyLoop,           // out: New PolyLoop or NULL when no new polyLoop was made
                             pNewPolyFace,           // out: New PolyFace or NULL when no new polyFace was made
                             FALSE,                   // in : Depends on whether there are degenerate edges. If not, don't do the extra checking.
                             &pNewSingularEdge,      // out: New SingularEdge or NULL when no Singular Edge found needing splitting
                                                     //      (a radial mateEdge running from End to Start is also made)
                             &pNewSingularVertex)) ; // out: New SingularVertex or NULL when no Singular Edge found needing splitting

          // see if this New manifold PolyEdge pair needs to be subdivided.
          // GWC check needed:Added this check here was previously using
          //      the lNumSubDivisionsNeeded computed from the rejected edge
          //      For a corrupted surface this may be causing lots of triangles - check that
          ULONG lNumSubdivisionsNeeded;
          SER(CheckAgainstSurfaceCache(pTessCache, // in : Cache with subdivision tree to interogate
                                       pBestEdge->GetStartPolyVertex()->GetPoint(), // in : start of UV segment to
                                                                                    // review
                                       pInsideVertexEdge->GetStartPolyVertex()->GetPoint(), // in : end of UV segment to
                                                                                            // review
                                       1.1, // in : allowed size variation before segment needs split
                                       TRUE, // in : TRUE = interior edge - don't check 3d lengths
                                             //      FALSE= exterior edge - do check 3d lengths
                                       lNumSubdivisionsNeeded)); // out: number of splits Start/End UV segment needs
                                                                 //      so segment sizes are about the smallest
                                                                 //      xsecting cache subdivision tree leaf node size.
          // remember which NewPolyEdges (and all their radial partners) need to be subdivided
          // 
          // Add <PolyEdge, PolyEdge->StartPt->Pos> pairs to VertexTree for NewPolyEdge and RadialPartner
          SER(sm_AddEdgeToVertexTree(pNewPolyEdge, rpVertexTree));
          SER(sm_AddEdgeToVertexTree(pNewPolyEdge->GetRadial(), rpVertexTree));

          if (lNumSubdivisionsNeeded > 0)
          {
              if (lNumSubdivisionsNeeded > 20)
              {
                  lNumSubdivisionsNeeded = 20;
              }
              rPolyEdgeSubdivisions.Add(lNumSubdivisionsNeeded);
              rPolyEdgesToSubdivide.Add(pNewPolyEdge);
          }

          // clear scores for edges
          ULONG lInsideVertexEdgeIndex = 0;
          sPolyFaceEdges.FindElement(pInsideVertexEdge, lInsideVertexEdgeIndex);
          
          // If added manifold PolyEdge pair splits face, call TriangluateFace on each child recursively
          if (pNewPolyFace != NULL)
          {
              rPolyFaceStack.Push(pNewPolyFace);
              rPolyFaceStack.Push(pPolyFace);
              return SM_SUCCESS;
          }

          // we probably just left, if not reset necessary scores (adjacent to the two edges
          // we just interacted with).

          sScores[lBestScoreEdge] = 0; // Need to recompute these
          sScores[((lBestScoreEdge + lNumEdges) - 1) % (lNumEdges)] = 0;
          sScores[(lBestScoreEdge + 1) % (lNumEdges)] = 0;
          sScores[((lInsideVertexEdgeIndex + lNumEdges) - 1) % (lNumEdges)] = 0;
          sScores[(lInsideVertexEdgeIndex + 1) % (lNumEdges)] = 0;
          sScores[lInsideVertexEdgeIndex] = 0;

          sChordDist[lBestScoreEdge] = 0; // Need to recompute these
          sChordDist[((lBestScoreEdge + lNumEdges) - 1) % (lNumEdges)] = 0;
          sChordDist[(lBestScoreEdge + 1) % (lNumEdges)] = 0;
          sChordDist[((lInsideVertexEdgeIndex + lNumEdges) - 1) % (lNumEdges)] = 0;
          sChordDist[(lInsideVertexEdgeIndex + 1) % (lNumEdges)] = 0;
          sChordDist[lInsideVertexEdgeIndex] = 0;

          sNumSubdivisions[lBestScoreEdge] = 0; // Need to recompute these
          sNumSubdivisions[((lBestScoreEdge + lNumEdges) - 1) % (lNumEdges)] = 0;
          sNumSubdivisions[(lBestScoreEdge + 1) % (lNumEdges)] = 0;
          sNumSubdivisions[((lInsideVertexEdgeIndex + lNumEdges) - 1) % (lNumEdges)] = 0;
          sNumSubdivisions[(lInsideVertexEdgeIndex + 1) % (lNumEdges)] = 0;
          sNumSubdivisions[lInsideVertexEdgeIndex] = 0;

          // add the two newly minted polyedges to the arrays, in location next to pBestEdge.
          
          sScores.InsertAt(((lBestScoreEdge) ) % (lNumEdges), 0, 1); // Need to recompute these, 0 is the uninit value
          sNumSubdivisions.InsertAt(((lBestScoreEdge) ) % (lNumEdges), 0, 1);
          sChordDist.InsertAt(((lBestScoreEdge)) % (lNumEdges), 0.0, 1);
          sPolyFaceEdges.InsertAt(((lBestScoreEdge)) % (lNumEdges), pNewPolyEdge->GetRadial(), 1);

          sScores.InsertAt(((lBestScoreEdge)) % (lNumEdges), 0, 1); // Need to recompute these, 0 is the uninit value
          sNumSubdivisions.InsertAt(((lBestScoreEdge)) % (lNumEdges), 0, 1);
          sChordDist.InsertAt(((lBestScoreEdge)) % (lNumEdges), 0.0, 1);
          sPolyFaceEdges.InsertAt(((lBestScoreEdge)) % (lNumEdges), pNewPolyEdge, 1);

#ifdef SM_DEBUG_CODE
          // keep history and sIterChanged for two new entries
          sHistory.Add(0) ; sHistory.Add(0) ;
          sIterChanged.Add(1) ; sIterChanged.Add(1) ;
#endif // SM_DEBUG_CODE
      } // end pInsideVertexEdge != NULL branch (can't make a triangle by adding a single edge without crossing another edge branch)

#ifdef SM_DEBUG_CODE
        SmBoolean bDebugMe9 = FALSE;
        SmBoolean bDebugMe9a = FALSE;
      if (bDebugMe9)
        {
          sm_DumpScoreHistory(lNumIter, lBestScoreEdge, sPolyFaceEdges, sScores, sHistory, sIterChanged, FALSE) ; // FALSE= only Dump changed scores
          sm_DumpScoreHistory(lNumIter, lBestScoreEdge, sPolyFaceEdges, sScores, sHistory, sIterChanged, TRUE) ;  // TRUE = dump all scores
        }

      if ( bDebugMe9a )
        {
          SmFace *pOrigFace = pPolyFace->GetOKBackPtrs() ? pPolyFace->GetOriginalFace() : NULL ;

          smgfx_Erase();
          smgfx_SetLook(2,4, 0,1,0); pPolyFace->Draw(TRUE,TRUE); sm_GraphicsLoop();
          smgfx_SetLook(1,2, 0,0,0); if(pOrigFace) pOrigFace->Draw(SM_DM_WIREFRAME); sm_GraphicsLoop();
          sm_GraphicsLoop();

          smgfx_SetLook(4,6, 1,0,0); pNewPolyEdge->Draw()  ; sm_GraphicsLoop();
          smgfx_SetLook(4,6, 1,0,0); pNewPolyEdge->Draw3D(); sm_GraphicsLoop();
          sm_GraphicsLoop();
        }
#endif // SM_DEBUG_CODE

  }  // end while PolyFace has more than 3 PolyEdges

  // arrive here when done dividing PolyFace into a triangular set of PolyFaces

  return SM_SUCCESS;

} // end SmTess::TriangulateSingleFace

/*******************************************************************//**
PURPOSE: Triangulate a general pPolyFace polygon by inserting a sequence
  of new PolyEdges into the PolyFace->PolyBrep until the original
  PolyFace polygon is split into a set of triangular PolyFace polygons.

NOTES: 
***********************************************************************/
SmStatus SmTess::TriangulateFace
    (SmPolyFace * pPolyFace)       // in : PolyFace to split into a family of triangle PolyFaces
                                   //      New PolyVertices are added.
{
  //   // Increment recursion level.
  //   SmTemporaryChangeValue<ULONG> sChange(m_lTriangulateFaceLevel, m_lTriangulateFaceLevel+1);
  //
  //   // limit recursion level
  //   if (m_lTriangulateFaceLevel > 15)
  //     {
  //       SER(SM_ERR);
  //     }

  // init return value
  SmStatus retval = SM_SUCCESS;

  // temporary VertexTree object
  SmTree    * pVertexTree = NULL;
  SmObjDelete sClean(pVertexTree) ;
  struct TreeGuard { SmTree *& tree;
                     TreeGuard (SmTree*& tree) : tree (tree) { }
                    ~TreeGuard () { delete tree; tree = NULL; }
                   } treeGuard (pVertexTree) ;

  // increment and lock an unlocked mark
  SmNewMarkAndLock sMarkLock( pPolyFace->GetContext(), SM_MT_ALLMARKS) ; // increment and lock any unlocked mark

  // stack locals
  SM_PTR_ARRAY(sPolyFaceStack,        SmPolyFace, 64); // Top member to be triangulated, MakeManifoldEdge children get added to stack
  SM_PTR_ARRAY(sPolyFaceStackNewTree, SmPolyFace, 64); // Faces connected to PolyEdges further subdivided are added to share a common VertexTree when triangulated
  SM_PTR_ARRAY(sPolyEdgesToSubdivide, SmPolyEdge, 64); // PolyEdges to further subdivide from TriangulateSingleFace() call
  SM_OBJ_ARRAY(sPolyEdgeSubdivisions, ULONG,      64); // number of subdivisions for each PolyEdge to further subdivide

  // locals
  SmFace     * pOriginalFace = pPolyFace->GetOriginalFace();
  //SmExtent3d   sOriginalFaceBBox ;
  //SmExtent2d   sOriginalUVDomain ;
  //if(pOriginalFace)
  //  {
  //    pOriginalFace->CalculateBoundingBox(sOriginalFaceBBox) ;
  //    sOriginalUVDomain = pOriginalFace->GetSurface()->GetNaturalUVDomain() ;
  //  }
  //double dOrigUVDomainMaxLength = sOriginalUVDomain.GetMaxLength() ;
  //double dOrigFaceBBoxMaxLength = sOriginalFaceBBox.GetMaxLength() ;

  // init PolyFaces to triangulate stack
  sPolyFaceStack.Push (pPolyFace);

  // while the face stack contains faces (it grows with faces connected to edges added that need further subdivision.)
  while(   sPolyFaceStack.GetSize() != 0
        || sPolyFaceStackNewTree.GetSize() != 0
        || sPolyEdgesToSubdivide.GetSize() != 0)
    {
      // after all PolyFaces have been triangulated using the current set of PolyVertices
      if (sPolyFaceStack.GetSize () == 0)
        {
          // Next - Add new PolyVertices by spliting all PolyEdges marked for further
          //        subdivision by last iter TriangulateSingleFace() call.

          // locals
          int    ii ;
          int    lNumSubdiv = (int)sPolyEdgesToSubdivide.GetSize();

          SM_OBJ_ARRAY(sPolyEdgeProps, SmPolyEdgeProp, 64) ;
          sPolyEdgeProps.SetSize(lNumSubdiv) ;

          // delete the current VertexTree - so it can be rebuilt with the New PolyVertices
          delete pVertexTree;
          pVertexTree = NULL;

          // for every PolyEdge needing subdivision - get PolyEdge properties, checked later to skip requested subdivisions
          for(ii=0; ii<lNumSubdiv; ii++) // use ints here because of upcoming ii subtraction
            {
              SmPolyEdge *pPE1 = sPolyEdgesToSubdivide[ii];

              // notes: o. Get properties of all PolyFaces prior to Subdividing PolyEdges while all PolyFaces are still triangles
              //        o. subdivision candidate PolyEdges are expected to be lamina or manifold

              // Get Aspect Ratios of PolyFaces connected to subdivision candidate PolyEdge
              sPolyEdgeProps[ii].m_dAspectRatio1 = 1.0 ;
              sPolyEdgeProps[ii].m_dAspectRatio2 = 1.0 ;
              SER( pPE1->ComputeAspectRatio( sPolyEdgeProps[ii].m_dAspectRatio1 ) );  // note: ComputeAspectRatio always returns SM_SUCCESS
              if ( pPE1->GetRadial() )
                {
                  SER( pPE1->GetRadial()->ComputeAspectRatio( sPolyEdgeProps[ii].m_dAspectRatio2 ) );
                }

              // Get 2d length of candidate subdivision pieces
              sPolyEdgeProps[ii].m_dPELengthUV = pPE1->Length() / sPolyEdgeSubdivisions[ii] ;

              // Get 3d length of candidate subdvision pieces
              sPolyEdgeProps[ii].m_dPELength3d = SM_BIG_DOUBLE ;
              if(pOriginalFace)
                {
                  SmPoint3d  sStart3d, sEnd3d ;

                  SmVector3d sStartUV = pPE1->GetStartPoint() ;
                  SmVector2d sStart2d(sStartUV.x, sStartUV.y) ;

                  SmVector3d sEndUV   = pPE1->GetEndPoint() ;
                  SmVector2d sEnd2d(sEndUV.x, sEndUV.y) ;

                  pOriginalFace->GetSurface()->EvaluatePoint( sStart2d, sStart3d) ;
                  pOriginalFace->GetSurface()->EvaluatePoint( sEnd2d, sEnd3d) ;

                  sPolyEdgeProps[ii].m_dPELength3d = sStart3d.DistanceBetween(sEnd3d) / sPolyEdgeSubdivisions[ii] ;
                }
            } // end iter every PolyEdge needing subdivision getting properties

          // for every PolyEdge needing subdivision - subdivide it - turning triangle PolyFaces into higher order polygons
          for(ii=0; ii<lNumSubdiv; ii++) // need ints because of upcoming ii subtraction
            {
              
#ifdef SM_DEBUG_CODE
              SmBoolean bDebugMe10 = FALSE;
              SmPolyEdge* pPE1 = sPolyEdgesToSubdivide[ii];

              if (bDebugMe10)
                {
                  SmFace     * pFace        = pPolyFace->GetOriginalFace() ;
                  SmPolyBrep * pTS_PolyBrep = GetTessBrepOfFace(pFace) ;
                  SM_ASSERT_VALID(pTS_PolyBrep) ;

                  //SmExtent3d sFaceBBox ;

                  smgfx_Erase() ;
                  smgfx_SetLook(1,2, 0,0,1); pTS_PolyBrep->Draw(TRUE, TRUE, TRUE) ; sm_GraphicsLoop() ;
                  smgfx_SetLook(2,4, 1,0,0); pPE1->Draw()  ; sm_GraphicsLoop();
                  smgfx_SetLook(2,4, 1,0,0); pPE1->Draw3D(); sm_GraphicsLoop();
                  sm_GraphicsLoop();
                }
#endif // SM_DEBUG_CODE

              // skip subdividision when PolyEdge is too small or attached to a tiny aspect ratio PolyFace
              SmBoolean bSkip = FALSE ;
              //if(sPolyEdgeProps[ii].m_dPELengthUV < 0.01 * dOrigUVDomainMaxLength)        { bSkip = TRUE ; } // 100 to 1 <== magic number could be tuned
              //if(sPolyEdgeProps[ii].m_dPELength3d < 0.01 * dOrigFaceBBoxMaxLength)        { bSkip = TRUE ; }
              if(pOriginalFace && sPolyEdgeProps[ii].m_dPELength3d < 10.0 * pOriginalFace->GetTolerance()) { bSkip = TRUE ; }
              if(sPolyEdgeProps[ii].m_dAspectRatio1 < 0.02)                               { bSkip = TRUE ; } // 50 to 1 <== magic number could be tuned
              if(sPolyEdgeProps[ii].m_dAspectRatio2 < 0.02)                               { bSkip = TRUE ; }
              if(bSkip)
                {
                  // remove the PolyEdge from the PolyEdge list - to help the next loop
                  sPolyEdgesToSubdivide.RemoveAt(ii, 1) ;
                  sPolyEdgeSubdivisions.RemoveAt(ii, 1) ;
                  sPolyEdgeProps.RemoveAt(ii, 1) ;

                  // decrement the loop iter and Cnt variables
                  ii-- ;
                  lNumSubdiv-- ;

                  // move on to the next new PolyEdge
                  continue ;

                } // end Small PolyEdge connected to extreme aspect ratio PolyFace check

              // Split PolyEdge (and all its radial partners) the number of requested times
              SmTArray<SmPolyEdge*> sNewPolyEdgesFromSubdivision;
              SER( SubdivideManifoldEdge(sPolyEdgesToSubdivide[ii], // in : PolyEdge to split
                                         sPolyEdgeSubdivisions[ii], // in : number of splits required
                                         pVertexTree,
                                         FALSE,
                                         sNewPolyEdgesFromSubdivision)); // in : spatial tree of
                                                                                               // <PolyEdge,PolyEdge->StartPt
                                                                                               // position> pairs

#ifdef SM_DEBUG_CODE
              if (bDebugMe10)
                {
                  SmFace     * pFace        = pPolyFace->GetOriginalFace() ;
                  SmPolyBrep * pTS_PolyBrep = GetTessBrepOfFace(pFace) ;
                  SM_ASSERT_VALID(pTS_PolyBrep) ;

                  smgfx_Erase() ;
                  smgfx_SetLook(1,2, 0,0,1); pTS_PolyBrep->Draw(TRUE, TRUE, TRUE) ; sm_GraphicsLoop() ;
                  smgfx_SetLook(2,4, 1,0,0); pPE1->Draw()  ; sm_GraphicsLoop();
                  smgfx_SetLook(2,4, 1,0,0); pPE1->Draw3D(); sm_GraphicsLoop();
                  SmPolyEdge *pCCW = pPE1->GetCCWPolyEdge();
                  smgfx_SetLook(2,4, 1,0,0); pCCW->Draw()  ; sm_GraphicsLoop();
                  smgfx_SetLook(2,4, 1,0,0); pCCW->Draw3D(); sm_GraphicsLoop();

                  sm_GraphicsLoop();
                }
#endif // SM_DEBUG_CODE

            } // end iter ii subdividing every PolyEdge needing subdivision

          // And now place faces connected to recently-subdivided edges onto Faces to triangulate stack.
          lNumSubdiv = sPolyEdgesToSubdivide.GetSize();
          for ( ii=0; ii<lNumSubdiv; ii++ )
            {
              SmPolyEdge * pPolyEdge = sPolyEdgesToSubdivide[ii];
              pPolyFace = pPolyEdge->GetPolyFace();

              // Stack PolyFace connected to target PolyEdge
              sPolyFaceStackNewTree.Push( pPolyFace );

              // Stack PolyFace connected to target PolyEdge's Radial partner
              if ( pPolyEdge->IsManifold() )
                {
                  SmPolyEdge *pRadial  = pPolyEdge->m_pNextRadialE;
                  SmPolyFace *pRadFace = pRadial->GetPolyFace();
                  sPolyFaceStackNewTree.Push( pRadFace );
                }
            } // end iter ii, placing faces connected to recently-subdivided edges onto faces to triangulate stack

          // exit while loop - when no new faces remain to be triangulated
          if (sPolyFaceStackNewTree.GetSize () == 0) //
            {
              break;
            }

          // refresh iteration locals for next iteration
          sPolyEdgesToSubdivide.SetSize (0);
          sPolyEdgeSubdivisions.SetSize (0);

          // Init PolyFaceStack with top of NewTree Stack member
          SmPolyFace* pActFace = NULL;
          sPolyFaceStackNewTree.Pop (pActFace);
          sPolyFaceStack.Push (pActFace);

        } // end after all PolyFaces have been triangulated using the current set of PolyVertices check

      // break top sPolyFaceStack PolyFace into a set of triangle PolyFaces by adding PolyEdges between existing PolyVertices
      SmStatus stat = TriangulateSingleFace(sPolyFaceStack,        // i/o: Top member gets triangulated,
                                                                   //      MakeManifoldEdge() generated children get added to stack.
                                            sPolyEdgesToSubdivide, // i/o: new poly edges which should subdivided
                                            sPolyEdgeSubdivisions, // i/o: number of subdivisions for members of rPolyEdgesToSubdivide
                                            pVertexTree,           // i/o: Spatial tree of PolyFace PolyVertices.
                                                                   //      New PolyVertices are added.
                                            sMarkLock) ;           // in : increments MarkLock mark value when rpVertexTree is NULL on input
      if (stat != SM_SUCCESS) { SE (stat);
                                retval = stat;
                              }
    } // end while PolyFaces remain to be triangulated

  return retval;

} // end SmTess::TriangulateFace

/*******************************************************************//**
PURPOSE: Get the 3D point of this UV point on the surface if there
   is one.  If not just return itself.

NOTES: 
***********************************************************************/
SmPoint3d SmTess::Get3DPoint
 (const SmSurface * cpSurface,
  const SmPoint3d & crUVPoint)
{
  if (!cpSurface) return crUVPoint;
  SmPoint2d sUV(crUVPoint.x,crUVPoint.y);
  SmPoint3d s3DPnt;
  SE(cpSurface->EvaluatePoint(sUV,s3DPnt));
  return s3DPnt;

} // end SmTess::Get3DPoint

/*******************************************************************//**
PURPOSE: Scale a UV vector to the length it needs to be to move
    the given 3d distance.  If the surface does not exist just use
    the 2D length.

NOTES: 
***********************************************************************/
SmVector3d SmTess::ScaleUVVecTo3DLength
 (const SmSurface  * cpSurface,
  const SmPoint3d  & crUVPoint,
  double             d3DLength,
  const SmVector3d & crUVVector)
{
  SmVector3d sRet;

  if (!cpSurface)
    {
      sRet = crUVVector;
      SE(sRet.Unitize());
      sRet = sRet * d3DLength;

      return sRet;
    }

  SmVector3d sDU, sDV, sPnt;

  SmPoint2d sUV(crUVPoint.x,crUVPoint.y);
  SE(cpSurface->Evaluate1stDerivatives(sUV,TRUE,TRUE,sPnt,sDU,sDV));
  sRet = crUVVector;
  SE(sRet.Unitize());

  SmVector3d s3DVec       = sDU * sRet.x + sDV * sRet.y;
  double     d3DVecLength = s3DVec.Length();

  // all done - set return
  sRet = sRet * (d3DLength / d3DVecLength);
  return sRet;

} // end SmTess::ScaleUVVecTo3DLength

/*******************************************************************//**
PURPOSE: Triangulate a polygonal face by Advancing Front.

NOTES: 
Example:
Here is how this works:
    0) Combine interior loops using minimum distance function
    1) Look for a convex corner 'V0' in the outer loop.
    2) Validate that the two adjacent corners can have an edge
       using the surface cache to check the distance from 'VCW' to 'VCCW'
    3) Find any other vertices in triangle V0, VCW, VCCW
         a) If none found make triangle and split face.
         b) Find the vertex VBest which is closest to the angle of
            the edge to VCCW and closest in distance to V0 in case of
            ties relative to the angle.
         c) Make an edge from V0 to VBest.  If it splits the face
            then call TriangulateFace recursively.  If it doesn't
            just continue on.

  NOTE-To improve quality measure edge length relative to quad size
  and find best corner to cut for optimal tessellation.
***********************************************************************/
SmStatus SmTess::TriangulateFaceAF
 (SmPolyFace            * pPolyFace, // in :
  SmFace                * pFace,     // in :
  double                  dQuadSize, // in :
  SmTArray<SmPolyFace*> & rNewFaces) // out:
{
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
std::atomic<ULONG> lCount(1);
lCount++;
std::atomic<ULONG> lDebugCount(0);
  if (bDebugMe || lDebugCount == lCount)
    {
      smgfx_Erase();
      smgfx_SetColor(0,0,0); pPolyFace->Draw(SM_DM_WIREFRAME); sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

  // locals
  SM_PTR_ARRAY(sNewInteriorFaces,SmPolyFace,256);
  SmTessSrfCache * pSC = m_vCache.GetSecond(pFace); NER(pSC);
  SmUVTessCallback sCB(pFace->GetSurface(),pSC,pFace,this);

  // find split parameters that make a good tessellation of a constant V IsoParamCurve
  SER(pFace->GetSurface()->FindTessellationSplits(SM_SP_V,
                                                  pFace->GetUVDomain(),
                                                  0.0,
                                                  0.0,
                                                  pSC->m_dMaximumSideLength3D,
                                                  sCB.m_vUSplits));


  // find split parameters that make a good tessellation of a constant U IsoParamCurve
  SER(pFace->GetSurface()->FindTessellationSplits(SM_SP_U,
                                                  pFace->GetUVDomain(),
                                                  0.0,
                                                  0.0,
                                                  pSC->m_dMaximumSideLength3D,
                                                  sCB.m_vVSplits));

#ifdef SM_DEBUG_CODE
  SmBoolean bDebugMe2 = FALSE;
  if (bDebugMe2)
    {
      sCB.m_vUSplits.Dump();
      sCB.m_vVSplits.Dump();
      SmPoint3d sPnt;

      smgfx_Erase();
      pFace->GetSurface()->DrawUV(5,5);
      for (ULONG j=0; j<sCB.m_vUSplits.GetSize(); j++)
        { for (ULONG k=0; k<sCB.m_vVSplits.GetSize(); k++)
            { SmPoint2d sUV(sCB.m_vUSplits[j],sCB.m_vVSplits[k]);
              pFace->GetSurface()->EvaluatePoint(sUV,sPnt);
              smgfx_SetColor(1,0,0);
              sPnt.Draw();
            }
          sm_GraphicsLoop();
        }
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

    //
    SER(pPolyFace->TessellateWithQuads(dQuadSize,
                                       sCB,
                                       rNewFaces,
                                       sNewInteriorFaces));

    // SmTess::SmoothPolygons is a different method than SmPolyBrep::SmoothPolygons. gwc:why?
    SER(pPolyFace->GetPolyBrep()->SmoothPolygons(sNewInteriorFaces,
                                                 m_vSmoothingData.m_lSmoothingPasses,
                                                 m_vSmoothingData.m_dSmoothingStepSize));  // note: increments unlocked mark value

    // all done
    return SM_SUCCESS;

} // end SmTess::TriangulateFaceAF

/*******************************************************************//**
PURPOSE: Triangulate a polygonal face by Advancing Front.

NOTES: 
Example:
Here is how this works:
    0) Combine interior loops using minimum distance function
    1) Look for a convex corner 'V0' in the outer loop.
    2) Validate that the two adjacent corners can have an edge
       using the surface cache to check the distance from 'VCW' to 'VCCW'
    3) Find any other vertices in triangle V0, VCW, VCCW
         a) If none found make triangle and split face.
         b) Find the vertex VBest which is closest to the angle of
            the edge to VCCW and closest in distance to V0 in case of
            ties relative to the angle.
         c) Make an edge from V0 to VBest.  If it splits the face
            then call TriangulateFace recursively.  If it doesn't
            just continue on.

  NOTE-To improve quality measure edge length relative to quad size
  and find best corner to cut for optimal tessellation.
***********************************************************************/
#if 0
SmStatus SmTess::TriangulateFaceAF
  (SmPolyFace * pPolyFace)
{
#ifdef SM_DEBUG_CODE
SmBoolean  bDebugMe = FALSE;
std::atomic<ULONG> lCount(1) ; lCount++ ;
std::atomic<ULONG> lDebugCount(0) ;
    if (bDebugMe || lDebugCount == lCount) {
        smgfx_Erase();
        sm_GraphicsLoop();
        smgfx_SetColor(0,0,0);
        pPolyFace->Draw();
        sm_GraphicsLoop();
    }
#endif

    pPolyFace->NewMark(SM_MT_MARK2);  // no need to fix obsolete mark call in obsolete method

    SmPolyEdge *pNewEdge;
    SmPolyLoop *pNewLoop;
    SmPolyFace *pNewFace;

#ifdef SM_DEBUG_CODE
    std::atomic <SmBoolean>  bDebugMe3 = FALSE;
    if (bDebugMe3) {
        smgfx_Erase();
        sm_GraphicsLoop();
        smgfx_SetColor(0,0,0);
        pPolyFace->Draw();
        sm_GraphicsLoop();
    }
#endif

    SM_PTR_ARRAY(sIntersectionEdges,SmPolyEdge,32);
    SmPoint3d sPData[64];
    SmTArray<SmPoint3d> sIntersectionPoints(64,sPData);

    SmPolyEdge *sEData[256];
    SmTArray<SmPolyEdge*> sEdges(256,sEData);
    pPolyFace->GetPolyEdges(sEdges);

    SmBoolean bFourSidedConvex = FALSE;

    if (sEdges.GetSize() < 3) {
        return SM_SUCCESS;
    }

    SmPolyEdge *sEData2[256];
    SmTArray<SmPolyEdge*> sInsideVertices(256,sEData2);


    SmBoolean bDone = FALSE;

    SmExtent3d sFaceBox;
    SER(pPolyFace->CalculateBoundingBox(sFaceBox));

    SmVector3d sNormal(0,0,1);

    ULONG sLData2[512];
    SmTArray<ULONG> sScores(512,sLData2);
    ULONG sLData3[512];
    SmTArray<ULONG> sAge(512,sLData3);
    SmBoolean sBData[512];
    SmTArray<SmBoolean> sEligable(512,sBData);
    double sD2Data[515];
    SmTArray<double> sNodeSpacing(512,sD2Data);
    SmTArray<SmPoint3d> sEdgePoints;

    // Set up auxilary data indexed by GetIndexValue() on the PolyEdge
    pPolyFace->GetPolyEdges(sEdges);
    double dMaxEdge = 0.0;
    double dTotalEdgeLength = 0.0;
    ULONG lOriginalNumEdges = sEdges.GetSize();
    for (ULONG mm=0; mm<sEdges.GetSize(); mm++) {
        SmPolyEdge *pE = sEdges[mm];
        pE->SetIndexValue(mm);
        sScores.Add(0);
        SmPolyEdge *pECW = pE->GetCWPolyEdge();
        // Compute the node spacing as the average of the lenghts of surrounding
        // edges.
        double dEdgeLength = pE->GetStartPoint().DistanceBetween(pE->GetEndPoint());
        if (dEdgeLength > dMaxEdge) {
            dMaxEdge = dEdgeLength;
        }
        dTotalEdgeLength += dEdgeLength;
        double dNodeSpace = (dEdgeLength +
            pECW->GetStartPoint().DistanceBetween(pECW->GetEndPoint()) ) / 2.0;
        sNodeSpacing.Add(dNodeSpace);
        sEdgePoints.Add(pE->GetStartPoint());
        sEligable.Add(TRUE); // Make everything eligable the first time.
        sAge.Add(0); // Everything starts with zero age.
    }
    double dAverageEdgeLength = dTotalEdgeLength / sEdges.GetSize();

#define BAD_SCORE 999999
    SM_PTR_ARRAY(sFaceStack,SmPolyFace,32);
    sFaceStack.Add(pPolyFace);

    while (sFaceStack.GetSize() > 0) {
        SmPolyFace *pFace = sFaceStack.GetLast();
        sFaceStack.RemoveLast();

    while (!bDone) {
        pFace->GetPolyEdges(sEdges);
        if (sEdges.GetSize() == 3) {
            break;
        }

        // Find the optimal edge to cut by looking at their scores
        ULONG lBestScoreEdge;
        ULONG lBestScore = BAD_SCORE;
        double dBestChordDist = SM_BIG_DOUBLE;
        SmPolyEdge *pInsideVertexEdge = NULL;
        for (ULONG i=0; i<sEdges.GetSize(); i++) {
            SmPolyEdge *pV0Edge = sEdges[i];
            if (!sEligable[pV0Edge->GetIndexValue()]) {
                continue;
            }
            SmPolyEdge *pVCWEdge = pV0Edge->GetCWPolyEdge();
            SmPolyEdge *pVCCWEdge = pV0Edge->GetCCWPolyEdge();
            SmPolyVertex *pVCW = pVCWEdge->GetStartPolyVertex();
            SmPolyVertex *pVCCW = pVCCWEdge->GetStartPolyVertex();
            // Compute edge scores.
            SmPolyEdge *pVertexClosestToV0VCCW = NULL;
            if (TRUE || sScores[pV0Edge->GetIndexValue()] == 0) {
#ifdef SM_DEBUG_CODE
std::atomic <SmBoolean>  bDebugMe5 = FALSE;
        if (bDebugMe5) {
            smgfx_Erase();
            smgfx_SetColor(0,0,1);
            pV0Edge->Draw();
            sm_GraphicsLoop();
            smgfx_SetColor(1,0,0);
            pVCWEdge->Draw();
            sm_GraphicsLoop();
            smgfx_SetColor(0,0,0);
            pFace->Draw();
            sm_GraphicsLoop();
        }
#endif
                ULONG lScore = 0;
                double dAngDeg;
                double dChordDist;
                pV0Edge->IsConvexCorner(sNormal,dAngDeg,dChordDist);

                lScore += (ULONG)(dAngDeg); // Smaller angle better score.
                // Higher ratio higher score

                // Add edge length ratio to score
                double dEdgeLeng = pV0Edge->GetStartPoint().DistanceBetween(pV0Edge->GetEndPoint());
                lScore += (ULONG) (360.0 * dEdgeLeng/dMaxEdge);

                // Add the age.
                lScore += (ULONG) (360.0 * sAge[pV0Edge->GetIndexValue()]);

                sScores[pV0Edge->GetIndexValue()] = lScore;
            }
            if (sScores[pV0Edge->GetIndexValue()] < lBestScore) {
                lBestScore = sScores[pV0Edge->GetIndexValue()];
                lBestScoreEdge = i;
                pInsideVertexEdge = pVertexClosestToV0VCCW;
            }
        }

        if (lBestScore == BAD_SCORE) {
            SM_PTR_ARRAY(sNewFaces,SmPolyFace,32);
            SER(pFace->TriangulatePlanar(sNewFaces));  // note: increments unlocked mark value
            continue;
        }

        SmPolyEdge *pBestEdge = sEdges[lBestScoreEdge];


        // Here we have a potential canidate to try.  Let's try to compute the
        // new point for a triangle.
        SmVector3d sBin = pBestEdge->ComputeBinormal();
        SmPoint3d sMidPoint;
        SER(pBestEdge->EvaluatePoint(0.5,sMidPoint));
        sBin.Unitize();
        double dEdgeLength = pBestEdge->GetStartPoint().DistanceBetween(pBestEdge->GetEndPoint());
        SmPoint3d sCenterPoint = sMidPoint + sBin * (dEdgeLength/2.0); // More or less the center of the triangle.
        double dSumationTop = 0.0;
        double dSumationBot = 0.0;
        for (ULONG ie=0; ie<lOriginalNumEdges; ie++) {
            double dDist = sCenterPoint.DistanceBetween(sEdgePoints[ie]);
            double dTmp = sNodeSpacing[ie] / dDist;
            dSumationBot = dSumationBot + dTmp;
            dSumationTop = dSumationTop + dTmp * sNodeSpacing[ie];
        }
        double dNewEdgeLength = dSumationTop/dSumationBot;
        if (dEdgeLength > dAverageEdgeLength * 1.5) {
            dEdgeLength = dAverageEdgeLength * 1.5;
        }
        if (dEdgeLength < dAverageEdgeLength/1.5) {
            dEdgeLength = dAverageEdgeLength/1.5;
        }
        if (dNewEdgeLength < dAverageEdgeLength/1.5) {
            dNewEdgeLength = dAverageEdgeLength/1.5;
        }
        if (dNewEdgeLength > dAverageEdgeLength * 1.5) {
            dNewEdgeLength = dAverageEdgeLength * 1.5;
        }
        if (dNewEdgeLength < dEdgeLength/1.1) {
            dNewEdgeLength = dEdgeLength/1.1;
        }
        if (dNewEdgeLength > dEdgeLength*1.1) {
            dNewEdgeLength = dEdgeLength*1.1;
        }
//        if (dNewEdgeLength > 2.0 * dEdgeLength) {
//            dNewEdgeLength = 2.0 * dEdgeLength;
 //       }

        double dHeight = smos_Sqrt(dNewEdgeLength*dNewEdgeLength - (dEdgeLength/2.0)*(dEdgeLength/2.0));
        SmPoint3d sIdealPoint = sMidPoint + sBin * dHeight;

        // Now check to see if there are any points within sIdealPoint using a
        // radius of say 2/3 height.
        SmPolyEdge *pClosestVertexEdge;
        SmPolyVertex *pClosestVertex = NULL;
        SER(pFace->ClosestVertex(sIdealPoint,dNewEdgeLength/1.3,pClosestVertexEdge));
        if (pClosestVertexEdge) {
            pClosestVertex = pClosestVertexEdge->GetStartPolyVertex();
            sIdealPoint = pClosestVertex->GetPoint();
        }

        // See if there are any vertices closer to the base edge.
        SmPoint3d sStart = pBestEdge->GetStartPoint();
        SmPoint3d sEnd = pBestEdge->GetEndPoint();
        SmPolyEdge *pClosestVEdge;
        SER(pFace->FindVerticesInTriangle
                     (sStart,              // in : P0 = (!bWorkIn3D) ? crP0 : OrigFace->Srf(crP0.x,crP0.y)
                      sEnd,                // in : P1 = (!bWorkIn3D) ? crP1 : OrigFace->Srf(crP1.x,crP1.y)
                      sIdealPoint,         // in : P2 = (!bWorkIn3D) ? crP2 : OrigFace->Srf(crP2.x,crP2.y)
                      NULL,                // in : vertex spatial tree, can narrow search if present
                      sInsideVertices,     // out: list of all PolyFace->PolyVerts in tri[P0,P1,P2]
                      pClosestVEdge));     // out: closest PolyVert to PolyEdge[P0,P1]
                                           // in : TRUE  = input PTs are UVPnts projected through OrigFace->Surface
                                           //       GWC?Bug: if TRUE: StraightEdge Planar triangle assumptions not valid
                                           //      FALSE = inputs are 3d pts
                                           //      default:[FALSE]

        if (sInsideVertices.GetSize() > 0) {
            pClosestVertex = pClosestVEdge->GetStartPolyVertex();
            sIdealPoint = pClosestVertex->GetPoint();
        }

#ifdef SM_DEBUG_CODE
std::atomic <SmBoolean>  bDebugMe2 = FALSE;
        if (bDebugMe2) {
            smgfx_Erase();
            smgfx_SetColor(0,0,1);
            pBestEdge->Draw();
            sm_GraphicsLoop();
            smgfx_SetColor(1,0,0);
            sIdealPoint.Draw();
            sm_GraphicsLoop();
            smgfx_SetColor(0,0,0);
            pFace->Draw();
            sm_GraphicsLoop();
        }
#endif

        // Now Check to see if these two segments produce any intersections.
        SER(pFace->IntersectSegment(sStart,sIdealPoint,
            pFace->GetTolerance(),sIntersectionPoints,sIntersectionEdges));
        if (sIntersectionPoints.GetSize() > 0.0) {
            sEligable[pBestEdge->GetIndexValue()] = FALSE;
            continue;
        }
        SER(pFace->IntersectSegment(sEnd,sIdealPoint,
            pFace->GetTolerance(),sIntersectionPoints,sIntersectionEdges));
        if (sIntersectionPoints.GetSize() > 0.0) {
            sEligable[pBestEdge->GetIndexValue()] = FALSE;
            continue;
        }


        if (!pClosestVertex) {
            SmPolyVertex *pNewV;
            SmPolyEdge *pNewE;
            SmPolyLoop *pNewL;
            SER(pFace->AddSingleVertexPolyLoop(sIdealPoint,
                                               pFace->GetTolerance(),
                                               NULL,       // GWC for now - ignore duplicating existing PolyVertices
                                               pNewV,pNewE,pNewL));
            NER(pNewV);
            pClosestVertex = pNewV;
        }

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
std::atomic<ULONG> lCount(1) ; lCount++ ;
std::atomic<ULONG> lDebugCount(0) ;
        if (bDebugMe || lCount == lDebugCount) {
            smgfx_Erase();
            smgfx_SetColor(0,0,1);
            pBestEdge->Draw();
            sm_GraphicsLoop();
            smgfx_SetColor(1,0,0);
            pClosestVertex->GetPoint().Draw();
            sm_GraphicsLoop();
            smgfx_SetColor(0,0,0);
            pFace->Draw();
            sm_GraphicsLoop();
            SmTArray<SmPolyFace*> sTmpFaces;
            pFace->GetPolyBrep()->GetPolyFaces(sTmpFaces);
            if (lCount % 20 == 0) {
            for (ULONG ifa=0; ifa<sTmpFaces.GetSize(); ifa++) {
                sTmpFaces[ifa]->Draw();
            }
            }
            sm_GraphicsLoop();
        }
#endif

        sTouchedVertices.ReSet()

        SmPolyEdge *pExistingPolyEdge1, *pExistingPolyEdge2;
        SER(pClosestVertex->FindPolyEdgeBetween(pBestEdge->GetStartPolyVertex(),pExistingPolyEdge1));
        SER(pClosestVertex->FindPolyEdgeBetween(pBestEdge->GetEndPolyVertex(),pExistingPolyEdge2));
        if (!pExistingPolyEdge1) {
            sTouchedVertices.Add(pClosestVertex);
            sTouchedVertices.Add(pBestEdge->GetStartPolyVertex());
            SER(pFace->GetPolyBrep()->MakeEdgeInFace(pFace,pClosestVertex,pBestEdge->GetStartPolyVertex(),
                pFace->GetTolerance(),pNewEdge,pNewLoop,pNewFace)); NER(pNewEdge);
            pNewEdge->GetRadial()->SetIndexValue(sScores.GetSize());
            SmPolyEdge *pECW = pNewEdge->GetRadial()->GetCWPolyEdge();
            double dEdgeLength = pNewEdge->GetStartPoint().DistanceBetween(pNewEdge->GetEndPoint());
            double dNodeSpace = (dEdgeLength +
                pECW->GetStartPoint().DistanceBetween(pECW->GetEndPoint()) ) / 2.0;
            sNodeSpacing.Add(dNodeSpace);
            sScores.Add(0);
            sEligable.Add(TRUE);
            sAge.Add(sAge[pBestEdge->GetIndexValue()]+1);
            if (pNewFace) {
                sFaceStack.Add(pNewFace);
                sFaceStack.Add(pFace);
                for (ULONG kkk=0; kkk<sEligable.GetSize(); kkk++) { sEligable[kkk] = TRUE; }
                continue;
            }
        }
        if (!pExistingPolyEdge2) {
            sTouchedVertices.Add(pClosestVertex);
            sTouchedVertices.Add(pBestEdge->GetEndPolyVertex());
            SER(pFace->GetPolyBrep()->MakeEdgeInFace(pFace,pClosestVertex,pBestEdge->GetEndPolyVertex(),
                pFace->GetTolerance(),pNewEdge,pNewLoop,pNewFace)); NER(pNewEdge);
            if (pNewFace) {
                sFaceStack.Add(pNewFace);
            }
            pNewEdge->GetRadial()->SetIndexValue(sScores.GetSize());
            SmPolyEdge *pECW = pNewEdge->GetRadial()->GetCWPolyEdge();
            double dEdgeLength = pNewEdge->GetStartPoint().DistanceBetween(pNewEdge->GetEndPoint());
            double dNodeSpace = (dEdgeLength +
                pECW->GetStartPoint().DistanceBetween(pECW->GetEndPoint()) ) / 2.0;
            sNodeSpacing.Add(dNodeSpace);
            sScores.Add(0);
            sEligable.Add(TRUE);
            sAge.Add(sAge[pBestEdge->GetIndexValue()]+1);
        }
        for (ULONG kkk=0; kkk<sEligable.GetSize(); kkk++) { sEligable[kkk] = TRUE; }
    }
    }
    return SM_SUCCESS;

} // end SmTess::TriangulateFaceAF

#endif // 0

/*******************************************************************//**
PURPOSE: Subdivide the PolyEdge (and all its radial partners)
         into several equal sized segments and update the vertex tree.

NOTES: Works on either manifold or lamina edges.
***********************************************************************/
SmStatus SmTess::SubdivideManifoldEdge
 (SmPolyEdge * pPolyEdgeToSplit,         // in : PolyEdge to split
  ULONG        lNumberSplits,            // in : number of splits required
  SmTree     * pVertexTree,              // in : spatial tree of <PolyEdge,PolyEdge->StartPt position> pairs
  SmBoolean    bReturnPolyEdges,         // in : flag for sending out the newly constructed PolyEdges.
  SmTArray<SmPolyEdge*> & sNewPolyEdges) // out: newly constructed PolyEdges, including the polyedge passed in.
{
  // locals
  ULONG ii, jj ;
  SmBoolean bIsLamina = pPolyEdgeToSplit->IsLamina();
  SM_PTR_ARRAY(sPolyEdges, SmPolyEdge, 32) ;  // SmTArray<SmPolyEdge *>

  // While edges remain to be split
  sPolyEdges.Add(pPolyEdgeToSplit);
  while (sPolyEdges.GetSize() <= lNumberSplits)
    {
      // for every this iter PolyEdge
      ULONG lOrigSize = sPolyEdges.GetSize();
      for (ii=0; ii<lOrigSize; ii++)
        {
          SmPolyEdge * pPolyEdge          = sPolyEdges[ii];
          SmZoneTol3d  sPolyEdgeZoneTol3d = SmTol::GetZoneTol3d(pPolyEdge) ;
          SmPoint3d    sMid      = (pPolyEdge->GetStartPoint() + pPolyEdge->GetEndPoint()) / 2.0;
          SmZoneTol3d  sSrcZoneTol3d = SmTol::GetZoneTol3d(pPolyEdge) ;

//cbi TEMPORARY:
//cbi These both fix the bug, but #1 adds 40% to the tests,
//cbi and #2 hangs there.

std::atomic<int> cbi(0);
          // locals
          SmPolyFace * pPolyFace = pPolyEdgeToSplit->GetPolyFace();
          SmFace     * pOrigFace = pPolyFace->GetOKBackPtrs() ? pPolyFace->GetOriginalFace() : NULL ;

          // when PolyFace backPtrs are valid - refine sMid point to OrigFace->Edge->DropPt when needed
          if(pOrigFace)
            {
              if ( cbi==1 )
                {
                  // Have to check that sMid is in the 3d face. [B258]
                  SmPoint2d sMidUV( sMid.x, sMid.y );
    //cbi make this a subroutine: ( pPolyFace, pMidUV )
                  SmPointClassification sPC(sPolyEdgeZoneTol3d, &m_crContext) ;

                  // classify the PolyEdge mid Point against the original Face
                  pOrigFace->PointClassify( sMidUV, sSrcZoneTol3d, TRUE, FALSE, sPC );

                  // When MidPoint classifies to UNKNOWN - drop sMidPoint to closest OrigFace Edge
                  if ( sPC.GetPointClass() == SM_PC_UNKNOWN )
                    {
                      // Drop the 3d point to the 3d edge.
                      // Unfortunately, we don't have that edge directly (because PolyEdges
                      // don't in general correspond to edges of the original face).
                      // We'll drop the point to each Edge, using a local solve with
                      // the Edge's midpoint as a guess.
                      // First get the 3d point.
                      SmSurface  * pSurf = pOrigFace->GetSurface();
                      SmPoint3d    sMid3d;
                      pSurf->EvaluatePoint( sMidUV, sMid3d );

                      SM_PTR_ARRAY(sEdges, SmEdge, 32) ; // SmTArray<SmEdge*>
                      pOrigFace->GetEdges( sEdges );
                      double dTol = 2 * pOrigFace->GetTolerance();
                      ULONG  lNumEdges = sEdges.GetSize();


                      for ( jj=0; jj<lNumEdges; jj++ )
                        {
                          SmEdge    * pE     = sEdges[jj];
                          SmExtent1d  sDom   = pE->GetInterval();
                          double      dGuess = sDom.GetMid();
                          SmBoolean   bSuccess;
                          SmBoolean   bIsMulti;
                          double      dT, dDist;
                          SmCurve   * pCurve = pE->GetCurve();
                          pCurve->DropPoint(sDom,       // in : target curve allowed domain
                                            sMid3d,     // in : Point to drop to curve
                                            NULL,       // in : For Snapping, NULL=NoSnap, Vec pointing to CrvInside from desired EndPt for snapping.
                                                        //      when DropPt is within dDistTol in cpVecToCrvInside dir of EndPt, Snap to Ivl EndPt.
                                                        //      Vec handles ambiguities and makes sure that curves are not just touching at the ends.
                                            dTol,       // in : Used for SnapDist, and SM_SO_INTERSECT sXSectTol3d distance.
                                                        //      sXSectTol3d = dDistTol ;      SM_SO_INTERSECT 
                                                        //      sXSectTol3d = SM_BIG_DOUBLE ; SM_SO_MINIMIZE, SM_SO_NORMALIZE
                                                        //      sSnapTol3d  = dDistTol ;      SM_SO_MINIMIZE, SM_SO_NORMALIZE, SM_SO_INTERSECT    
                                            &dGuess,    // in : If given, call cheap LocalPointSolve() else call expensive GlobalPointSolve()
                                            bSuccess,   // out: TRUE = found a drop point
                                            dT,         // out: found drop curve param
                                            dDist) ;    // out: found drop distance
                                                        // in : SM_SO_MINIMIZE = allow nonNormal drops near endPoints
                                                        //      SM_SO_INTERSECT= like Minimize but point must be within dDistTol
                                                        //      SM_SO_NORMALIZE= exclude nonNormal drops near endPoints
                                                        //      default:[SM_SO_MINIMIZE] to preserve original behavior
                          if (   bSuccess
                              && sDom.ContainsValue( dT, -SM_EFF_ZERO_SQRT )
                             )
                            {
                              double dEdgeLen = pCurve->ApproximateLength( sDom, 5 );
                              if ( dDist < 0.25 * dEdgeLen )
                                {
                                  // Set our uv midpoint to agree with the position on the edge.
                                  pCurve->EvaluatePoint( dT, sMid3d );
                                  SmExtent2d sFaceDom = pOrigFace->GetUVDomain();
                                  SmPoint2d sNewUV;
                                  pSurf->DropPoint( sMid3d, sFaceDom, &sMidUV, bSuccess, sNewUV, dDist, bIsMulti );
                                  if ( bSuccess )
                                    {
                                      sMid.Set( sNewUV.x, sNewUV.y, 0 );
                                      break;
                                    } // end successful droppoint check
                                } // end ( dDist < 0.25 * dEdgeLen ) check
                            } // end if ContainsValue check
                        } // end iter jj, every sEdges
                    } // end MidPoint classifies to UNKNOWN check - so drop sMidPoint to closest OrigFace Edge check
                } // end ( cbi==1 ) branch
              else if ( cbi==2 )
                {  //cbi.
                  // Let's just see how expensive it is to get the uv of the 3d midpoint.
                  SmPoint2d    sMidUV( sMid.x, sMid.y );
                  SmSurface  * pSurf     = pOrigFace ? pOrigFace->GetSurface() : NULL ;
                  SmPoint3d    sP0       = pPolyEdge->GetEndPoint();
                  SmPoint3d    sP1       = pPolyEdge->GetStartPoint();
                  SmPoint2d    sP02d( sP0.x, sP0.y );
                  SmPoint2d    sP12d( sP1.x, sP1.y );

                  // This won't work at singularities: if you do two different ones with
                  // the same off-singularity point, you'll get the same 3d midpoint,
                  // and hence the same uv dropped point.
                  SmSurfParamType eDirUV;
                  if (   ! pSurf->IsSingularity( sP02d, eDirUV )
                      && ! pSurf->IsSingularity( sP12d, eDirUV ) )
                    {
                      SmPoint3d sP03d, sP13d;
                      pSurf->EvaluatePoint( sP02d, sP03d );
                      pSurf->EvaluatePoint( sP12d, sP13d );
                      SmPoint3d sMid3d = ( sP03d + sP13d ) / 2;
                      SmExtent2d sFaceDom = pOrigFace->GetUVDomain();
                      SmBoolean  bSuccess;
                      SmBoolean  bIsMulti;
                      double     dDist = 0.0;
                      SmPoint2d  sNewUV;

                      pSurf->DropPoint( sMid3d, sFaceDom, &sMidUV, bSuccess, sNewUV, dDist, bIsMulti );

      #ifdef SM_DEBUG_CODE
                      SmBoolean bDebugMe14 = FALSE;
                      if (bDebugMe14)
                        {
                          if ( FALSE )
                            {
                              smgfx_Erase();
                              smgfx_SetLook( 1,2, 0,0,0 ); pPolyFace->Draw(SM_DM_WIREFRAME); sm_GraphicsLoop();
                              smgfx_SetLook( 1,2, 0,0,0 ); pOrigFace->Draw(SM_DM_WIREFRAME); sm_GraphicsLoop();
                              sm_GraphicsLoop();
                            }
                          smgfx_SetLook(2,4, 0,1,1 ); pPolyEdge->Draw(); sm_GraphicsLoop();
                          smgfx_SetLook(4,6, 0,0,1 ); sP0.Draw(); sm_GraphicsLoop();
                          smgfx_SetLook(4,6, 0,0,1 ); sP1.Draw(); sm_GraphicsLoop();
                          smgfx_SetLook(4,6, 0,1,0 ); sMid.Draw(); sm_GraphicsLoop();
                          smgfx_SetLook(4,6, 1,0,0 ); sNewUV.Draw(); sm_GraphicsLoop();
                          sm_GraphicsLoop();

                          smgfx_SetLook(2,4, 0,1,1 ); pPolyEdge->Draw3D(); sm_GraphicsLoop();
                          smgfx_SetLook(4,6, 0,0,1 ); sP03d.Draw(); sm_GraphicsLoop();
                          smgfx_SetLook(4,6, 0,0,1 ); sP13d.Draw(); sm_GraphicsLoop();
                          smgfx_SetLook(4,6, 0,1,0 ); sMid3d.Draw(); sm_GraphicsLoop();
                          SmPoint3d sNew3d;
                          pSurf->EvaluatePoint( sNewUV, sNew3d );
                          smgfx_SetLook(4,6, 1,0,0 ); sNew3d.Draw(); sm_GraphicsLoop();
                          sm_GraphicsLoop();
                        }
      #endif // SM_DEBUG_CODE
                      if ( bSuccess )
                        {
                          sMid.Set( sNewUV.x, sNewUV.y, 0 );
                        } //end successful DropPoint check
                    } // end not UVPoints are not on surface singularity check
                } // end ( cbi==2 ) branch
            } // end PolyFace->GetOKBackPtrs = TRUE check - refine sMid point to OrigFace->Edge->DropPt when needed
          // end cbi temporary section.

          SmPolyEdge   * pRadial = pPolyEdge->m_pNextRadialE;
          SmPolyEdge   * pNewEdge = NULL;
          SmPolyVertex * pNewVertex = NULL;
          SER(pPolyEdge->GetPolyFace()->MakeVertexSplitPolyEdge(pPolyEdge,    // in : PolyEdge to split
                                                                sMid,         // in : Point split location
                                                                pNewEdge,     // out: new PolyEdge
                                                                pNewVertex)); // out: new PolyVertex
          if (pNewEdge == NULL)
            { continue; }
          if (pVertexTree) { SER(sm_AddEdgeToVertexTree(pNewEdge,pVertexTree)); }

          sPolyEdges.Add(pNewEdge);
          if (!bIsLamina)
            {
              SmPolyEdge *pOtherNewEdge = pNewEdge->m_pNextRadialE;
              if (pOtherNewEdge == pRadial)
                {
                  pOtherNewEdge = pPolyEdgeToSplit->m_pNextRadialE;
                }
              if (pVertexTree) { SER(sm_AddEdgeToVertexTree(pOtherNewEdge,pVertexTree)); }
            }
        } // end iter i, every this while iter PolyEdge
    } // end while edges remain to be split

  // all done
  if (bReturnPolyEdges)
  {
      sNewPolyEdges = sPolyEdges;
  }
  return SM_SUCCESS;

} // end SmTess::SubdivideManifoldEdge

/*******************************************************************//**
PURPOSE: Tessellate Phase 1 - tessellate Brep->Edges preparing for
         water tight face tessellations in Phase 2.
  m_pTessBrep       = orig target Brep modified with edge splits.
  m_vCache          = RelatePairs<pFace,pSC> pSC = SmTessSrfCache for pFace->Surface
  pSC->mTS_PolyBrep = m_pTessBrep linear approximation with:
                        1 SmPolyFace per simple outer pFace->Loop (complex pFace->Loops are broken into simple Loops yielding a few PolyFaces)
                        1 inner SmPolyLoop for each inner pFace->Loop,
                        1 SmPolyEdge for each pFace->Loop->Edge,
                        1 SmPolyVertex for each pFace->Loop->Vertex.
  m_p3DPolyBrep     = NULL.
NOTES: 
  Store m_pTessBrep.
  Split m_pTessBrep->Edges to lengths suitable for linear approximation.
  Build pSC->mTS_PolyBrep for each pFace containing one SmPolyFace
             connected to 1 SmPolyLoop per Face->Loop and 1 SmPolyEdge per Loop->Edge.
SIDE EFFECTS:
  1. m_pTessBrep = pBrepUsedForTessellation;
  2. Create SmTessSrfCache pSC for every Face
       using control params in m_rSurfaceTess,
       store pSC as m_vCache.RelatePair(pFace,pSC);
  3. modify m_pTessBrep:
     For every m_pTessBrep->Edge
       a. If(IsoCurve) Tessellate (split) at pSC Surface BreakPoints,
       b. Tessellate (split) all Edges to m_rCurveTess control params,
       c. Further Tessellate Edges until every Edge length is about pSC Node Size,
       d. Refine (further split) if needed to m_pTessBrep->Edges to eliminate
          polyline/polyline xsects that occur due to approximating
          m_pTessBrep->Curves with mTS_PolyBrep->Lines.
  4. create mTS_PolyBrep for each pFace
       a piecewise linear approximation of all m_pTessBrep->pFace->Loops and Edges.
       mTS_PolyBrep->m_bOKBackPtrs == TRUE.  During Tessellation back ptrs are current.
       After Tessellation, back ptrs can go stale.

NOTES: 1. If you wish to tessellate everything at once and then
          get the polygons out use 'DoTessellation'.

       2. If you wish to generate the polygons for one face at a time you can
          use the following procedure:

          1) SmTess::Phase1SetUpBrep -- Set up the Brep for tessellation
          2) For Each Face in the Brep do the following:
               a) SmTess::Phase2CreateFacePolygons -- creates polygons for each face
               b) SmTess::OutputFacePolygons -- sends polygons back to your program
               c) SmTess::Phase3DeleteFacePolygons -- deletes the polygons for the face

          Here is what the code might look like:

              SmTArray<SmFace*> sFaces;
              pBrepCopy->GetFaces(sFaces);
              SER(sTess.Phase1SetupBrep(pBrepCopy,bFailedFaces));  // note: increments an unlocked mark value
              for (ULONG ii=0; ii<sFaces.GetSize(); ii++)
                {
                  if (sTess.Phase2CreateFacePolygons(sFaces[ii]) == SM_SUCCESS)
                    {
                      sTess.OutputFacePolygons(sFaces[ii],sPolyOutput);
                    }
                  sTess.Phase3DeleteFacePolygons(sFaces[ii]);
                }

  increments an unlocked mark value
***********************************************************************/
SmStatus SmTess::Phase1SetupBrep
 (SmBrep            * pBrepUsedForTessellation,      // in : Brep to tessellate. Brep is saved in SmTess, then modified
                                                     //      with split edges, and then deleted when SmTess is deleted.
                                                     //      Typically this is a copy of the Brep to tessellate.
  SmBoolean         & rbSomeFacesFailedToTessellate, // out: TRUE = a problem face was encountered
  SmTArray<SmFace*> * pOptFacesToTessellate,         // in : Only Set up to tessellate these faces for now.
  SmTArray<SmStatus>* pFaceStatuses)
{
#ifdef TIMER_CODE
#ifdef SM_DEBUG_CODE
  clock_t start = clock();
#endif
#endif
  if (pFaceStatuses) pFaceStatuses->SetSize(0);

  // check input
  NER(pBrepUsedForTessellation);

  // init output
  rbSomeFacesFailedToTessellate = FALSE;

  // place tgt brep into the target SmTess member slot
  m_pTessBrep = pBrepUsedForTessellation;

  // prepare that brep for changes
  m_pTessBrep->m_bEditingEnabled = TRUE;
// Remove Composites
// m_pTessBrep->m_bMakeComposites = TRUE;

  // prepare SmVertex::m_pPolyVertex back pointers (prevent using stale values)
  m_pTessBrep->ClearAllPolyVertices() ;

  m_pTessBrep->AddAttribute(new(m_pTessBrep->GetContext()) SmAttribute(SM_AI_TESSELLATING));
  
  // local SmTArrays
  ULONG ii, jj ;
  SM_PTR_ARRAY(sFaces,    SmFace, 256) ; // SmTArray<SmFace*>
  SM_PTR_ARRAY(sEdges,    SmEdge, 256) ; // SmTArray<SmEdge *> = All m_pTessBrep edges to tessellate
  SM_PTR_ARRAY(sFEdges,   SmEdge, 256) ; // SmTArray<SmEdge *>
  SM_PTR_ARRAY(sEFaces,   SmFace, 256) ; // SmTArray<SmFace *>
  SM_PTR_ARRAY(sAdjFaces, SmFace, 256) ; // SmTArray<SmFace *>

  // Begin Scope: get faces to tessellate (pOptFacesToTessellate == NULL, get all faces and edges)
    {
      if (pOptFacesToTessellate == NULL)
        { m_pTessBrep->GetFaces(sFaces);
          m_pTessBrep->GetEdges(sEdges);
        }
      else // tessellate the faces on pOptFacesToTessellate list
        { sFaces.Append(*pOptFacesToTessellate);

          // increment and lock an unlocked mark
          SmNewMarkAndLock sMarkLock( m_pTessBrep->GetContext(), SM_MT_ALLMARKS) ; // increment and lock any unlocked mark
          SmMarkType eMarkType = sMarkLock.GetMarkType() ;

          //long lProductCode = 1;

          // Get all m_pTessBrep->Face->Edges to tessellate - use marks to get them just once
          for (ii=0; ii<sFaces.GetSize(); ii++)
            {
              SmFace *pF = sFaces[ii];
              pF->Mark(eMarkType);
              pF->GetEdges(sFEdges);
              for (jj=0; jj<sFEdges.GetSize(); jj++)
                {
                  SmEdge *pE = sFEdges[jj];
                  if (pE && !pE->IsMarked(eMarkType))
                    {
                      pE->Mark(eMarkType);
                      sEdges.Add(pE);
                    }
                }  // end iter every edge
            } // end iter every face - accumulating edges

          // Add Adjacent faces to the face list as well - use marks to avoid duplications
          for (ii=0; ii<sEdges.GetSize(); ii++)
            {
              SmEdge *pE = sEdges[ii];
              pE->GetFaces(sEFaces);
              for (jj=0; jj<sEFaces.GetSize(); jj++)
                {
                  SmFace *pEF = sEFaces[jj];
                  if (!pEF->IsMarked(eMarkType))
                    {
                      pEF->Mark(eMarkType);
                      sFaces.Add(pEF);
                    }
                }
            } // end iter every edge looking for neighbor faces
        } // end user specified list of Faces and Edges to tessellate branch
    } // End Scope: get faces to tessellate (pOptFacesToTessellate == NULL, get all faces and edges)

  SmTArray<SmStatus> sLocalStatuses;
  SmTArray<SmStatus>& sStatuses = pFaceStatuses ? *pFaceStatuses : sLocalStatuses;
  sStatuses.SetSize(sFaces.GetSize());
  for (ii = 0; ii < sStatuses.GetSize(); ++ii) sStatuses[ii] = SM_SUCCESS;
  auto recordFailure = [&](ULONG index, SmStatus status)
  {
    if (sStatuses[index] == SM_SUCCESS) sStatuses[index] = status;
    rbSomeFacesFailedToTessellate = TRUE;
  };

  // Arrive here when: sFaces = list of all faces to tessellate
  //                   sEdges = list of all edges to tessellate
  //   If pOptFacesToTessellate == NULL, that's the whole Brep,
  //   else sFaces = OptFaces + adjacent faces and sEdges = all their edges.

  // Next, For all sFaces create pTessSrfCache cache for Face->Surface,
  //       and relate the pair <pFace, pTessSrfCache(pFace->Surface)> in m_vCache
  //       Also count the various types of surfaces.

  // locals
  SM_OBJ_ARRAY(sRectangularTrim, SmBoolean, 512) ; // SmTArray<SmBoolean>
  std::atomic<ULONG> lNumRev(0);
  std::atomic<ULONG> lNumExt(0);
  std::atomic<ULONG> lNumPlane(0);
  std::atomic<ULONG> lNumFailedCache(0);
  SmStatus eStat;
  std::atomic<ULONG> lRectangularTrimCount(0);  // number of faces without trimming UVTrimCurve boundaries

  // GWC_NEEDS_WORK PARALLEL_OPPORTUNITY_HERE GWC_LINE ;
  // gwc: parallel opportunity here.
  //      It may be possible to parallize the following loop on faces to build Surface Tessellation Caches.
  //      The only obvious change required is to move the RelatePair(pFace,pSC) global memory management from the
  //      CreateFaceTessCache(pFace) call up to this level to happen after this loop by doing something like
  //      SmTArray<SmTessSrfCache *> pSCs ; 
  //      pSC->SetSize(sFaces.GetSize()) ;
  //      SmTessSrfCache *pSC = pSCs[ii] ; 
  //      change eStat = CreateFaceTessCache( pFace ); to eStat = CreateFaceTessCache(pFace, pSC) ;
  //    
  //      Then in a 2nd loop executed after all the parallel threads are done building all the pSCs
  //      for (ii=0; ii<sFaces.GetSize(); ii++)
  //        {
  //          SmFace *pFace = sFaces[ii] ;
  //          SmTessSrfCache *pSC = pSCs[ii] ;
  //          m_vCache.RelatePair(pFace,pSC) ;
  //        } 

  #ifdef SM_USE_TBB
  SmTArray<SmTessSrfCache*> pSCs;
  pSCs.SetSize(sFaces.GetSize());
  sRectangularTrim.SetSize(sFaces.GetSize());
  static tbb::task_arena sArena;
  sArena.execute(
      [&]()
  {
      tbb::parallel_for((ULONG)0, sFaces.GetSize(), [&](ULONG ii)
      {
            SmFace   * pFace            = sFaces[ii];
            SmBoolean  bRectangularTrim = FALSE;
            SmBoolean  bNaturalTrim     = FALSE;
            SmExtent2d sUVDomain = pFace->GetUVDomain();

            SmStatus eStat = pFace->FindIfRectangularTrim(sUVDomain, bRectangularTrim, bNaturalTrim) ;

            if (eStat != SM_SUCCESS)
            {
                bRectangularTrim = FALSE ;
                bNaturalTrim     = FALSE ;
            }
            pFace->SetIsRectangularTrim(bRectangularTrim);
            if (bRectangularTrim == TRUE) lRectangularTrimCount++;
            sRectangularTrim[ii] = bRectangularTrim;
            SmTessSrfCache*& pSC = pSCs[ii];
            eStat = CreateFaceTessCache(pFace, pSC);

            if (eStat != SM_SUCCESS)
            {
                sStatuses[ii] = eStat;
                lNumFailedCache++;
                sFaces[ii] = NULL;
            }

            if (pFace->GetSurface()->IsKindOf(SmSurfOfRevolution_TYPE))
            {
                lNumRev++;
            }

            if (pFace->GetSurface()->IsKindOf(SmSurfOfExtrusion_TYPE))
            {
                lNumExt++;
            }

            if (pFace->GetSurface()->IsKindOf(SmPlane_TYPE))
            {
                lNumPlane++;
            }
      });
  });

  for (ii = 0; ii < sFaces.GetSize(); ii++)
  {
      SmFace* pFace = sFaces[ii];
      SmTessSrfCache* pSC = pSCs[ii];
      if (pFace) m_vCache.RelatePair(pFace, pSC);
  }
  #else
  // for every face - build its surface tessellation cache
  for (ii=0; ii<sFaces.GetSize(); ii++)
    {
      SmFace *pFace = sFaces[ii];

   //if (this->m_bAdvancingFront) // - build sRectangularTrim array
   //{
      SmBoolean  bRectangularTrim = FALSE ;
      SmBoolean  bNaturalTrim     = FALSE ;
      SmExtent2d sUVDomain        = pFace->GetUVDomain() ;

      eStat = pFace->FindIfRectangularTrim(sUVDomain, bRectangularTrim, bNaturalTrim) ;
      if ( eStat != SM_SUCCESS )
        { 
          bRectangularTrim = FALSE ;
          bNaturalTrim     = FALSE ; // If it were, then it wouldn't have had a problem.
        } 
      pFace->SetIsRectangularTrim(bRectangularTrim);
      if(bRectangularTrim == TRUE) { lRectangularTrimCount++ ; }
      sRectangularTrim.Add(bRectangularTrim);
   //}

      // build SmTessSrfCache pSC containing surface subdivision tree for pFace->Surface
      //    using control params in m_rSurfaceTess:
      //      m_vViewVector,
      //      m_dSilhouetteChordHeight,
      //      m_dSilhouetteAngleToleranceDeg
      //      m_dChordHeight,                 // in : max allowed control-Point dist to patch basePlane,       0.0 = ignore
      //      m_dAngTolDeg                    // in : max allowed controlPolygon endTangent angles (radians),  0.0 = ignore
      //        *SM_PI/180.0,
      //      m_dMaxEdgeLength3D,             // in : max allowed basePolygon side length in u or v direction, 0.0 = ignore
      //      m_dMinEdgeLength3D,             // in : polygon side length stopping size for subdivision,       0.0 = ignore
      //      m_dMinEdgeLengthRatioUV,        // in : Subdivison stops once node get smaller than this size,   0.0 = ignore
      //      m_dMaxAspectRatio);             // in : max allowed element basepolygon aspect ratio,            0.0 = ignore
      // and store in m_vCache.RelatePair(pFace,pSC).
      // m_pTessBrep is not modified.
      eStat = CreateFaceTessCache( pFace );

      if ( eStat != SM_SUCCESS )
        {
          recordFailure(ii, eStat);
          lNumFailedCache ++;
          sFaces[ii] = NULL;
        }

#ifdef SM_DEBUG_CODE
      SmBoolean bDebugMe = FALSE;
      if (bDebugMe)
        {
          SmTessSrfCache *pSC = m_vCache.GetSecond(pFace);
          SM_ASSERT_VALID(pSC) ;
          SM_ASSERT_VALID(pFace) ;
          SM_ASSERT_VALID(pSC->mTS_pPolyBrep) ;

          if ( FALSE )
            { smgfx_Erase(); }
          smgfx_SetLook(2,3, 1,0,0); if(pFace) pFace->DrawUV(); sm_GraphicsLoop();                // Face being tessellated
          smgfx_SetLook(2,3, 1,0,0); if(pFace) pFace->DrawUVCurves(FALSE); sm_GraphicsLoop();     // Face being tessellated
          SmSurface *pSurf = (pFace) ? pFace->GetSurface() : NULL;
          smgfx_SetLook(1,2, 0,1,1); if(pSurf) pSurf->DrawUV(3,2); sm_GraphicsLoop();             // Surface of Face
          smgfx_SetLook(1,2, 0,0,0); if(m_pTessBrep) m_pTessBrep->Draw(TRUE) ; sm_GraphicsLoop(); // 3d Brep being tessellated
          smgfx_SetLook(2,1, 0,1,0); if(pSC) pSC->DrawSubdivision2D(); sm_GraphicsLoop();         // 2d SrfTessCache of Face being tessellated
          smgfx_SetLook(1,2, 0,0,0); if(pSC) pSC->DrawTessellation2D(); sm_GraphicsLoop();        // 2d Brep of Face tessellation
          smgfx_SetLook(2,1, 0,1,0); if(pSC) pSC->DrawSubdivision3D(); sm_GraphicsLoop();         // 2d SrfTessCache of Face being tessellated
          smgfx_SetLook(1,2, 0,0,0); if(pSC) pSC->DrawTessellation3D(); sm_GraphicsLoop();        // 2d Brep of Face tessellation
          sm_GraphicsLoop();
        }
#endif // SM_DEBUG_CODE

      // count the plane, revolved, and extruded surfaces seen
      if (pFace->GetSurface()->IsKindOf(SmSurfOfRevolution_TYPE))
        { lNumRev ++; }

      if (pFace->GetSurface()->IsKindOf(SmSurfOfExtrusion_TYPE))
        { lNumExt ++; }

      if (pFace->GetSurface()->IsKindOf(SmPlane_TYPE))
        { lNumPlane ++; }

    } // end iter all faces building face tessellation caches
  #endif

  // set output - remember any face tessellation failures
  if ( lNumFailedCache > 0 )
    { rbSomeFacesFailedToTessellate = TRUE; }

#ifdef TIMER_CODE
#ifdef SM_DEBUG_CODE
  clock_t finish = clock();
  sm_PrintTime(_T("Time for TessCache Creation in Phase1 of SetupBrep"),start,finish);
  start = clock();
#endif
#endif

#ifdef SM_DEBUG_CODE
  SmBoolean bDebugMe1 = FALSE;
  if (bDebugMe1)
    {
      smos_WriteLong(_T("Number Rectangular Trimmed Faces = "),lRectangularTrimCount);
      smos_WriteLong(_T("Total Faces = "),sFaces.GetSize());
      smos_WriteLong(_T("Number Revolutions = "),lNumRev);
      smos_WriteLong(_T("Number Extrusion = "),lNumExt);
      smos_WriteLong(_T("Number Planes = "),lNumPlane);
      smos_WriteLong(_T("Number Failed Cache construction = "),lNumFailedCache);
    }
#endif

  // next Split m_pTessBrep->Edges
  //    first at iso-curve parameters and
  //    then using curve tessellation parameters.
  // Note, if an edge is split, its Mark is removed.

  // This creates SmEdges.
  SmTemporaryChangeValue <SmBoolean> sTempChangeDB( ((SmContext*) &m_crContext)->GetDoingBooleanRef(), TRUE );

  ULONG lNumEdges;
  if ( TRUE )  // (!m_bAdvancingFront)
    {
      m_pTessBrep->GetEdges( sEdges );
      lNumEdges = sEdges.GetSize();

      #ifdef SM_USE_TBB
      {
          std::unordered_set<SmEdgeuse*> sEdgeuses;
          for (SmEdge* pEdge : sEdges)
          {
              pEdge->GetEdgeuses(sEdgeuses);
          }

          static tbb::task_arena sArenaSame;
          sArenaSame.execute(
              [&]()
              {
                  tbb::parallel_for((size_t)0, sEdgeuses.size(),
                                    [&sEdgeuses](size_t i)
                                    {
                                        auto it = sEdgeuses.begin(); // JLMCC get rid of the auto, it is iterator from unordered_set
                                        std::advance(it, i);
                                        SmEdgeuse * pEU = *it;
                                        if (pEU->GetOrientation() == SM_OT_OPPOSITE)
                                        {
                                            return;
                                        }
                                        SmBSplineCurve* pUVCurve = NULL;
                                        pEU->GetOrCreateUVTrimCurve(pUVCurve);
                                    });
              });
          static tbb::task_arena sArenaOpp;
          sArenaOpp.execute(
              [&]()
              {
                  tbb::parallel_for((size_t)0, sEdgeuses.size(),
                                    [&sEdgeuses](size_t i)
                                    {
                                        auto it = sEdgeuses.begin(); // JLMCC get rid of the auto, it is iterator from
                                                                     // unordered_set
                                        std::advance(it, i);
                                        SmEdgeuse* pEU = *it;
                                        if (pEU->GetOrientation() == SM_OT_SAME)
                                        {
                                            return;
                                        }
                                        SmBSplineCurve* pUVCurve = NULL;
                                        pEU->GetOrCreateUVTrimCurve(pUVCurve);
                                    });
              });
      }
      #endif

      // for every edge - split IsoParameterCurves at Surface breakPoints (improves approx curve/surface accuracy)
      for ( ii=0; ii<lNumEdges; ii++ )
        {
          SmEdge *pEdge = sEdges[ii];

#ifdef SM_DEBUG_CODE
          SmBoolean bDebugMe12 = FALSE;
          if (bDebugMe12)
            {
              if ( ii == 0 )
                { smgfx_Erase(); }
              smgfx_SetLook(1,2, 0,0,1 ); pEdge->Draw(); sm_GraphicsLoop();
              sm_GraphicsLoop();
            }
#endif

          // when edge maps to an iso UVTrimCurve for some face
          // then split that edge at all the surface splits
          //  so that edge splits and face splits are compatible
          IsoEdgeSubdivide(pEdge);

#ifdef SM_DEBUG_CODE
          if (bDebugMe12)
            {
              smgfx_SetLook(2,4, 1,0,0 ); pEdge->Draw(); sm_GraphicsLoop();
              sm_GraphicsLoop();
            }
#endif
        } // end iter every edge splitting those with iso UVTrimCurves
    } // end if TRUE

#ifdef TIMER_CODE
#ifdef SM_DEBUG_CODE
  finish = clock();
  sm_PrintTime(_T("Time for Iso Edge Subdivide in Phase1 of SetupBrep"),start,finish);
  start = clock();
#endif // SM_DEBUG_CODE
#endif // TIMER_CODE

  m_pTessBrep->GetEdges(sEdges);
  lNumEdges = sEdges.GetSize();

  // for every m_pTessBrep->Edge - tessellate (split) to m_rCurveTess params
  for ( ii=0; ii<lNumEdges; ii++ )
    {
      SmEdge *pEdge = sEdges[ii];

#ifdef SM_DEBUG_CODE
      SmBoolean bDebugMe14 = FALSE;
      if (bDebugMe14)
        {
          if ( ii == 0 )
            { smgfx_Erase(); }
          smgfx_SetLook(1,2, 0,0,1 ); pEdge->Draw(); sm_GraphicsLoop();
          sm_GraphicsLoop();
        }
#endif

      // Tessellate pEdge (split pEdge in m_pTessBrep at found tessellation SplitParams)
      //   using m_rCurveTess control params:
      //     SmBoolean m_bEvalBasedTessellation; // FALSE = SmCurve::TessellateByBisection()
      //                                         //          if that fails - tessellate by SmCurveCache spatial decomposition
      //                                         // TRUE  = SmBSplineCurve::EquallySpacedPoints()
      //                                         // default:[FALSE]
      //
      //     ULONG  m_lMinSegNumber;             // min number of segments in tess polygon,     0=ignore
      //     double m_dChordHeight;              // max distance between geom and tess segment, 0=ignore
      //     double m_dAngTolDeg;                // max angle between tess tangents,            0=ignore
      //     double m_dMaxDist3dBetweenPts;      // max distance between tess pts,              0=ignore
      //     double m_dMinParamRatio;            // Limits the smallness of the stepsize
      //
      // m_pTessBrep gets 1 new SmVertex and 1 new SmEdge per split when
      // pEdge is split (repeatedly) by ClampAndSplitEdge(SplitParams).
      SER( TessellateEdge( pEdge ));

#ifdef SM_DEBUG_CODE
      if (bDebugMe14)
        {
          smgfx_SetLook(2,4, 1,0,0 ); pEdge->Draw(); sm_GraphicsLoop();
          sm_GraphicsLoop();
        }
#endif

    } // end loop calling TessellateEdge() on each edge 

#ifdef SM_DEBUG_CODE
  if ( bDebugMe1 )
  {
      SM_DUMP_AND_ASSERT_VALID( m_pTessBrep );
  }
#endif // end SM_DEBUG_CODE

#ifdef TIMER_CODE
#ifdef SM_DEBUG_CODE
  finish = clock();
  sm_PrintTime(_T("Time for Edge Tessellate in Phase1 of SetupBrep"),start,finish);
  start = clock();
#endif
#endif

  // Next, call SplitFaceEdgesToUVSize() on each face.
  // This splits all Face->Edges to be as small as
  // the SmTrimSrfCache leaf nodes that contain them.

  ULONG lNumFaces = sFaces.GetSize();
  for(ii=0; ii<lNumFaces; ii++ )
    {
      // locals
      SmFace         * pFace = sFaces[ii];
      SmTessSrfCache * pSC   = m_vCache.GetSecond( pFace );

      // skip pFaces with data problems - GWC: these continues are really bugs, they shouldn't be hit
      if ( pFace == NULL )
        { continue; }
      if (!pSC)
        { continue; }

      // evenly split pFace->Edges in m_pTessBrep so UVLengths are about surface subdivision node size.
      eStat = SplitFaceEdgesToUVSize( pFace );

      // If that fails, try recreating the Face's uv trim curves,
      // and try SplitFaceEdgesToUVSize() again.

      if ( eStat != SM_SUCCESS )
        {
          // clean UVTrimCurves
          SER(pFace->RemoveUVTrimCurves());
          double dTmp;

          // recreate the UVTrimCurves
          eStat = pFace->CreateUVTrimCurves(TRUE,              // in : TRUE = adjust Edge and Vertex tolerances to include measured gaps
                                            NULL,              // in : optional array of UVTrimCurves to assign to Face->Edgeuses
                                            NULL,              // in : associated orientations. If cpOptUVCurves notNULL, then
                                                               //      cpOptOrientations must be notNULL.
                                            dTmp, dTmp, dTmp,
                                            dTmp, dTmp, dTmp);
          if ( eStat != SM_SUCCESS )
            {
              // When CreateUVTrimCurves() fails - the tessellation failed
              recordFailure(ii, eStat);
              SER(Phase3DeleteFacePolygons( pFace ));
              continue;
            }

          // then try splitting edges a 2nd time
          eStat = SplitFaceEdgesToUVSize( pFace );
          if ( eStat != SM_SUCCESS )
            {
              // When 2nd try SplitFaceEdgesToUVSize() fails - the tessellation failed
              recordFailure(ii, eStat);
              SER(Phase3DeleteFacePolygons( pFace ));
              continue;
            }
        } // end if SplitFaceEdgesToUVSize() failed

    } // end iter every face splitting edges to be as small as their SmTrimSrfCache leaf nodes

#ifdef TIMER_CODE
#ifdef SM_DEBUG_CODE
  finish = clock();
  sm_PrintTime(_T("Time for Create Face UV Segments in Phase1 of SetupBrep"),start,finish);
  start = clock();
#endif // SM_DEBUG_CODE
#endif // TIMER_CODE

  // arrive here after all pFace->pEdges in m_pTessBrep have been subdivided
  // to tessellation length, i.e. each Edge piece is well approximated by a line.

  // Next for every pFace, build a PolyBrep piecewise linear approximation of pFace->Loops and Edges
  //    stored in pSC->mTS_PolyBrep containing:
  //       1 SmPolyFace per simple outer pFace->Loop (complex pFace->Loops are broken into simple Loops yielding a few PolyFaces)
  //       1 inner SmPolyLoop for each inner pFace->Loop,
  //       1 SmPolyEdge for each pFace->Loop->Edge,
  //       1 SmPolyVertex for each pFace->Loop->Vertex.
  //    This assumes all pFace->Edges have already been split to tessellation length.
  //    pEdges are refined (subdivided) whenever approximating an Edge->Curve
  //    with a PolyEdge->line runs into problems.
  //     e.g. PoyEdge->Lines intersect one another when Edge->Curves don't

  // iter locals
  ULONG bDone = FALSE;
  ULONG lNumTries = 0;

  //#ifdef SM_USE_TBB
  //{
  //    m_pTessBrep->CreateUVTrimCurvesParallel();
  //}
  //#endif

  // iter every face making MakePolyBrepFromFaceEdges() calls until pEdges are no longer being refined.
  while (!bDone && lNumTries < 100)
    {
      lNumTries ++;
      bDone = TRUE;

      // for every Face
      for(ii=0; ii<sFaces.GetSize(); ii++)
        {
          SmFace         * pFace = sFaces[ii];
          SmTessSrfCache * pSC   = m_vCache.GetSecond(pFace);
          SmBoolean        bRefinementDone;

          // GWC: pFace is not expected to be NULL - hitting this continue is a bug
          if ( pFace == NULL )
            { continue; }

          // GWC: pSC is not expected to be NULL - hitting this continue is a bug
          if (!pSC)
            { continue; }

#ifdef SM_DEBUG_CODE
            SmBoolean bRefresh = FALSE;
            SmBoolean bDebugMe7 = FALSE;
          if ( bDebugMe7 )
            {
              pFace->Dump();
              pFace->AssertValid();
              if ( bRefresh )
                {
                  smgfx_Erase();
                  smgfx_SetLook( 1,2, 0,0,0 ); pBrepUsedForTessellation->Draw(TRUE); sm_GraphicsLoop();
                  sm_GraphicsLoop();
                }
              smgfx_SetLook( 2,4, 0,1,1 ); pFace->DrawUV(3,3); sm_GraphicsLoop();
              sm_GraphicsLoop();
              smgfx_SetLook( 4,6, 1,0,0 ); pFace->Draw( SM_DM_WIREFRAME ); sm_GraphicsLoop();
              sm_GraphicsLoop();
            }
#endif // SM_DEBUG_CODE

          // Create and store in pSC->mTS_pPolyBrep an SmPolyBrep
          // linear approximation of the Loops and Edges in pFace containing:
          //   1 SmPolyFace per simple outer pFace->Loop
          //       (complex pFace->Loops are broken into simple Loops yielding a few PolyFaces)
          //   1 inner SmPolyLoop for each inner pFace->Loop,
          //   1 SmPolyEdge for each pFace->Loop->Edge,
          //   1 SmPolyVertex for each pFace->Loop->Vertex.
          //   No Inner PolyVerts or PolyEdges at this time.
          //   PolyLoops not broken up into triangles at this time.
          //
          // Sets pSC->mTSpPolyBrep->m_bOKBackPtrs == TRUE.
          //     While tessellating, back ptrs are current. After tessellating, ptrs may go stale.
          eStat = MakePolyBrepFromFaceEdges( pFace,             // in : target face
                                             bRefinementDone ); // out: FALSE= PSC->MTS_pPolyBrep built without any further subdivision of pFace->Edges
                                                                //      TRUE = pFace->Edges were subdivided to resolve problems caused by
                                                                //             linearization.
          if (eStat == SM_ERR_FATAL)
           {
             recordFailure(ii, eStat);
             SER(Phase3DeleteFacePolygons(pFace));
             continue;
           }

#ifdef SM_DEBUG_CODE
// GWC_NOTE CHANGE_NEXT_LINE_TO_TRUE_FOR_TESSELLATE_DEBUG GWC_LINE ;
          SmBoolean bDebugMe8 = FALSE;
          if ( bDebugMe8 )
            {
              TCHAR sBuff[SM_TBLOCK_SIZE];

              SmTessSrfCache * pTSC = m_vCache.GetSecond(pFace);
              SmPolyBrep     * pTS_pPolyBrep = pTSC ? pTSC->mTS_pPolyBrep : NULL ;
              SM_PTR_ARRAY(sPolyFaces, SmPolyFace, 512) ;
              if(pTS_pPolyBrep) pTS_pPolyBrep->GetPolyFaces(sPolyFaces) ;

              smos_sprintf(sBuff, _T("\nSmTess::Phase1SetupBrep Face [%3ld of %ld]: TriangleCount=%ld, running AssertValid(pTSC->mTS_pPolyBrep=0x%p) after MakePolyBrepFromFaceEdges()"),
                                ii, sFaces.GetSize(), sPolyFaces.GetSize(), pTS_pPolyBrep);
              smos_WriteBuffer(sBuff) ;
              SM_ASSERT_VALID(pTS_pPolyBrep) ;

              smos_sprintf(sBuff, _T("\n        End Face [%3ld] AssertValid(pTSC->mTS_pPolyBrep=0x%p)"), ii, pTS_pPolyBrep);
              smos_WriteBuffer(sBuff) ;

              pTS_pPolyBrep->DumpPolyFaces   (1, TRUE) ; // TRUE = only dump objs with attributes
              pTS_pPolyBrep->DumpPolyEdges   (1, TRUE) ; // TRUE = only dump objs with attributes
              pTS_pPolyBrep->DumpPolyVertices(1, TRUE) ; // TRUE = only dump objs with attributes

            }
#endif // SM_DEBUG_CODE

          // If it failed, recreate Face's uv curves and try again.
          if ( eStat != SM_SUCCESS )
            {
              double dTmp;

              // Clean and rebuildUVTrimCurves
              SER(pFace->RemoveUVTrimCurves());
              if(SM_SUCCESS != (eStat = pFace->CreateUVTrimCurves(TRUE,            // in : TRUE = adjust Edge and Vertex tolerances to include measured gaps
                                                         NULL,            // in : optional array of UVTrimCurves to assign to Face->Edgeuses
                                                         NULL,            // in : associated orientations. If cpOptUVCurves notNULL, then
                                                                          //      cpOptOrientations must be notNULL.
                                                         dTmp,dTmp,dTmp,
                                                         dTmp,dTmp,dTmp)))
                {
                  recordFailure(ii, eStat);
                  SER(Phase3DeleteFacePolygons(pFace));
                  continue;
                }

              // Split all pFace->Edges in m_pTessBrep until their UVSize is
              // smaller than the UV size of the SmTessSrfCache subdivision tree leaf nodes
              // that they intersect.
              if ((eStat = SplitFaceEdgesToUVSize(pFace)) != SM_SUCCESS)
                {
                  recordFailure(ii, eStat);
                  SER(Phase3DeleteFacePolygons(pFace));
                  continue;
                }

              // Try again with new UVTrimCurves.
              //   Create and store in pSC->mTS_pPolyBrep an SmPolyBrep
              //   linear approximation of the Loops and Edges in pFace containing:
              //     1 SmPolyFace per simple outer pFace->Loop
              //         (complex pFace->Loops are broken into simple Loops yielding a few PolyFaces)
              //     1 inner SmPolyLoop for each inner pFace->Loop,
              //     1 SmPolyEdge for each pFace->Loop->Edge,
              //     1 SmPolyVertex for each pFace->Loop->Vertex.
              //     No Inner PolyVerts or PolyEdges at this time.
              //     PolyLoops not broken up into triangles at this time.
              //
              //   Sets pSC->mTSpPolyBrep->m_bOKBackPtrs == TRUE.
              //       While tessellating back ptrs are current After tessellating, ptrs may go stale.
              if(SM_SUCCESS != (eStat = MakePolyBrepFromFaceEdges(pFace,            // in : target face
                                                         bRefinementDone))) // out: FALSE= PSC->MTS_pPolyBrep built without any further subdivision of pFace->Edges
                                                                           //      TRUE = pFace->Edges were subdivided to resolve problems caused by
                                                                           //             linearization.
                {
                  recordFailure(ii, eStat);
                  SER(Phase3DeleteFacePolygons(pFace));
                  continue;
                }
            } // end if MakePolyBrepFromFaceEdges() failed

          // when bRefinementDone == TRUE - prepare for next iteration
          if (bRefinementDone)
            {
              // after a large number of tries
              if (lNumTries == 99)
                {
                  // The tessellation failed -
                  // Tessellate Phase 3 - Clean up temp tessellation data structures for pFace
                  //   m_pTessBrep        = not modified.
                  //   m_vCache           = remove RelatePair<pFace,pSC> and delete SmTessSrfCache pSC
                  //   pSC->mTS_pPolyBrep = deleted
                  //   m_p3DPolyBrep      = not modified.
                  recordFailure(ii, SM_ERR);
                  SER(Phase3DeleteFacePolygons(pFace));
                }

              // Set bDone == FALSE to force all pFaces to be retessellated.
              //   GWC: this could be more efficient - we need only to call
              //        MakePolyBrepFromFaceEdges() on all the faces attached to
              //        an Brep->Edge that was split.  Instead of refacetting all
              //        faces we should accumulate of list of out of date
              //        faces and retessellate just those.
              bDone = FALSE;
            }

        } // end iter every face
    }  // end while refinements are still being done

#ifdef TIMER_CODE
#ifdef SM_DEBUG_CODE
  finish = clock();
  sm_PrintTime(_T("Time for Create PolyBrep of Face in Phase1 of SetupBrep"),start,finish);
  start = clock();
#endif
#endif

  // when using AdvancingFront
  if (this->m_bAdvancingFront)
    {
     // for every face - remember if the face is naturally trimmed
      for (ii=0; ii<sFaces.GetSize(); ii++)
        {
          sFaces[ii]->SetIsRectangularTrim(sRectangularTrim[ii]);
        }
    }

  // all done
  return SM_SUCCESS;

} // end SmTess::Phase1SetupBrep

/*******************************************************************//**
PURPOSE: Convenience routine that calls
  Phase1SetUpBrep()               = Split m_TessBrep Edges into good linear approximations
  then Phase2CreateFacePolygons() = Build one Tessellation per Face stored in pSC->mTS_PolyBrep .
     (where pSC = SmTessSrfCache for each Face in m_TessBrep)

  After this call use SmTess::OutputPolygons() to
    1.    Use polygons to build a single tessellated PolyBrep Model of m_TessBrep in m_p3DPolyBrep,
    2. or Stream polygons out to current draw stream (as a sequence of OpenGL calls),
    3. or Accumulate polygon into a SmGFxArraySet for later rapid rendering.

  m_pTessBrep   = pBrepUsedForTessellation after Edge splits
  m_p3DPolyBrep = Not built in this call. But can be built by SmTess::OutputPolygons()
                  to be the linear approx of m_pTessBrep with one SmPolyEdge
                  for every m_pTessBrep->Edge and many SmPolyFaces
                  for every m_pTessBrep->Face

NOTES: 1. convenience function for calling Phase1SetupBrep once
          followed by one Phase2CreateFacePolygons call per pBrep->pFace.

       2. m_pTessBrep edges are subdivided (split) until each segement
          is well approximated by a line and meet the tessellation
          control specifications in m_rCurveTess.

       3. m_pTessBrep Surface subdivision surface caches are built to
          the specifications in m_rSurfaceTess and then used
          to create SmPolySurface meshes that are stored
          in pSC->mTS_PolyBrep

      4. Suggested input parameters might include dChordHeight + dMaxEdgeLength3D
         where dMaxEdgeLength3D is related to the size of the object as suggested below

         SmExtent3d sBBox;
         pBrepUsedForTessellation->CalculateBoundingBox( sBBox, TRUE );
         double dSize = sBBox.GetSize().Length();
         static constexpr double sdEdgeLengthRatio = 0.40;
         this->m_rSurfaceTess.m_dMaxEdgeLength3D = dSize * sdEdgeLengthRatio;

***********************************************************************/
SmStatus SmTess::DoTessellation
 (SmBrep    * pBrepUsedForTessellation,      // in : Brep to tessellate. Brep is saved in SmTess, then modified
                                             //      with split edges, and then deleted when SmTess is deleted.
                                             //      Typically this is a copy of the Brep to tessellate.
  SmBoolean & rbSomeFacesFailedToTessellate) // out: TRUE = some faces failed to tessellate
{
#ifdef SM_DEBUG_CODE
    SmBoolean bDebugMe = FALSE;
  TCHAR sBuff[SM_TBLOCK_SIZE];
  if ( bDebugMe )
    {
      if ( FALSE )
        { smgfx_Erase(); }
      smgfx_SetLook( 1,2, 0,0,1 ); pBrepUsedForTessellation->Draw( TRUE ); sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

  SmTemporaryChangeValue <SmBoolean> sTempChangeDB( ((SmContext*) &m_crContext)->GetDoingBooleanRef(), TRUE );

  // Lock the bounding box while this is in scope (not a problem).
  //SmLockBrepBox lockit (pBrepUsedForTessellation);

  // The issue of MaxEdgeLength3D:
  // The tessellation algorithm gives much better results when MaxEdgeLength3D is specified.  [B418]
  // Forcing MaxEdgeLength3D might cause slowness or other issues. [B563]
  // It is not a good idea to force modification of input parameters

  // This can make a huge difference when there are small faces on large surfaces [B563]
  pBrepUsedForTessellation->ShrinkGeometry();

  // Tessellate Phase 1 - tessellate all Brep->Edges preparing for water tight face tessellations in Phase 2.
  //   m_pTessBrep       = orig target Brep modified with edge splits.
  //   m_vCache          = RelatePairs<pFace,pSC> pSC = SmTessSrfCache for pFace->Surface
  //   pSC->mTS_PolyBrep = m_pTessBrep linear approximation for each m_pTessBrep->PFace
  //                       with 1 PolyLoop per Loop, and 1 PolyEdge per Edge.
  //   m_p3DPolyBrep     = NULL.
  SER(Phase1SetupBrep(pBrepUsedForTessellation, rbSomeFacesFailedToTessellate));  // note: increments an unlocked mark value

  // Now Tessellate each face
  SM_PTR_ARRAY(sFaces, SmFace, 256) ; // SmTArray<SmFace*>
  m_pTessBrep->GetFaces( sFaces );
  ULONG kk = 0;
  ULONG lNumFaces = sFaces.GetSize();

  // Now create tessellation surface caches for each face

//  // Allocate an SmGfxArraySet for each face.
//  SmTArray< SmGfxArraySet* > sGfxSets;
//  sGfxSets.SetSize( lNumFaces );
//  for ( kk=0; kk<lNumFaces; kk++ )
//    {
//      sGfxSets[kk] = new SmGfxArraySet( sSaveColor.x, sSaveColor.y, sSaveColor.z );
//    }
//  SmObjsDelete<SmGfxArraySet*> sDelGfxSets( &sGfxSets );

  int ii=0;
  int iNumF = lNumFaces;

  for (ii=0; ii<iNumF; ii++)
    {
      kk = ii;
      SmFace *pFace = sFaces[kk];

      // Tessellate Phase 2 - for 1 pFace: add compatible Face->Surface tessellation to existing pSC->mTS_pPolyBrep Face->Edge->Curve tessellations.
      //   m_pTessBrep         = not modified.
      //   m_vCache            = pSC->TreeNodes of RelatePair<pFace,pSC> is loaded with pFace->UVTrimCurves
      //                         and labeled in/out/on pFace.
      //   pSC->mTS_pPolyBrep  = gets additional new SmPolyEdges and SmPolyVertices
      //   m_p3DPolyBrep       = NULL ;
      if (Phase2CreateFacePolygons( pFace ) != SM_SUCCESS)
        {
          rbSomeFacesFailedToTessellate = TRUE;
        }

#ifdef SM_DEBUG_CODE
// GWC_NOTE CHANGE_NEXT_LINE_TO_TRUE__FOR_TESSELLATE_DEBUG GWC_LINE ;
        SmBoolean bDebugMe8 = FALSE;
        SmBoolean bDebugPar = FALSE;
      if ( bDebugMe8 )
        {
          SmTessSrfCache * pSC = m_vCache.GetSecond(sFaces[kk]);
          SmPolyBrep     * pTS_pPolyBrep = pSC ? pSC->mTS_pPolyBrep : NULL ;
          SM_PTR_ARRAY(sPolyFaces, SmPolyFace, 512) ;
          if(pTS_pPolyBrep) pTS_pPolyBrep->GetPolyFaces(sPolyFaces) ;

          smos_sprintf(sBuff, _T("\nBegin SmTess::DoTessellation AssertValid(pSC->mTS_pPolyBrep=0x%p), after Phase2CreateFacePolygons() on Face [%3ld of %ld]: TriangleCount=%ld"),
                            pTS_pPolyBrep, kk+1, sFaces.GetSize(), sPolyFaces.GetSize());
          smos_WriteBuffer(sBuff) ;
          SM_ASSERT_VALID(pTS_pPolyBrep) ;
          smos_sprintf(sBuff, _T("\nEnd   SmTess::DoTessellation AssertValid(pSC->mTS_pPolyBrep=0x%p), after Phase2CreateFacePolygons() on Face [%3ld] "), pTS_pPolyBrep, kk+1);
          smos_WriteBuffer(sBuff) ;
        }

        if ( bDebugPar )
          {
            smos_sprintf(sBuff, _T("%ld A: Face %p tessellating:\n"), kk, pFace );
            smos_WriteBuffer(sBuff);
          }
#endif // SM_DEBUG_CODE

    } // end iter every face - finishing off UVTessellation pSC->mTS_pPolyBrep

  return SM_SUCCESS;

} // end SmTess::DoTessellation

/*******************************************************************//**
PURPOSE: Tessellate Phase 2 - Add compatible Face->Surface tessellation
         to existing pSC->mTS_pPolyBrep Face->Edge->Curve tessellations.
  m_pTessBrep         = not modified.
  m_vCache            = pSC->TreeNodes of RelatePair<pFace,pSC> is loaded with pFace->UVTrimCurves
                        and labeled in/out/on pFace.
  pSC->mTS_pPolyBrep  = gets additional new SmPolyEdges and SmPolyVertices
  m_p3DPolyBrep       = NULL ;

NOTES: 1. This method requires Phase1SetupBrep() to run first, which
             subdivides p_TessBrep edges to lengths well approximated by lines,
             creates the pSC, pFace->Surface SmTessSrfCache,
                     (stored in: m_vCache->RelatePair<pFace, SmTessSrfCache pSC>)
                     the pSC->mTS_pPolyBrep,
                     and all the SmPolyEdges and SmPolyVertices of the
                        pFace->Edge tessellations.

       2. This call imbeds the m_pTessBrep->Edge->UVTrimCurves into the
          pSC SmTessSrfCache->TreeNodes and labels each TreeNode
          as in/on/boundary with respect to PFace

       3. Then PolyEdges are added to pSC->mTS_pPolyBrep to outline every inside
          quad pFace->Suface->TessSrfCache node.  Those quads are further
          split into triangles.

SIDE EFFECTS: pSC->mTS_pPolyBrep = gets new SmPolyEdges and SmPolyVertices
              m_vCache           = pSC->TreeNodes of RelatePairs<pFace,pSC> is
                                   loaded with pFace->UVTrimCurvesa and
                                   labeled in/out/on pFace.
***********************************************************************/
SmStatus SmTess::Phase2CreateFacePolygons
 (SmFace *pFace)
{
  // locals
  SmTessSrfCache *pSC = m_vCache.GetSecond(pFace);

  // no work - pFace has no related SmTessSrfCache
  if (!pSC)
    { return SM_ERR; }

  // Build TessSrfCache Tree, Imbed UVTrimCurves, Label TreeNodes Inside/Outside/OnBoundary.
  //  Phase 1 already built the TessSrfCache, so that step will be skipped.
  //  But the Imbedding of UVTrimCurves and Labeling of TreeNodes as In/Out/On needs to be done.
  SER(pSC->BuildTree());

  // Triangulate the interior of pSC->mTS_pPolyBrep which currently contains just
  // one (or more) PolyLoop(s) of PolyEdges all contained by just one PolyFace.
  //    1. Add Quad PolyFaces for every INSIDE pFace->Surface->SrfTessCache SubdivisionTree TreeNode
  //    2. Place outer lamina Edge boundary of the QTreeNode Quad PolyFaces
  //         into an inner PolyLoop of the current PolyFace.
  //         (Handle multiple PolyLoop cases.)
  //    3. Now that entire pFace->UVDomain is covered by a set of PolyFaces,
  //       decompose every quad and high PolyEdge count PolyFace into sets of triangular PolyFaces.
  // pSC->mTS_pPolyBrep changes from one PolyFace to one PolyFace for every tessellation triangle.
  if (AddQuadBoundaries(pFace) != SM_SUCCESS)
    {
      // When AddQuadBoundaries fail - clean up with Phase 3
      SER(Phase3DeleteFacePolygons(pFace));
      return SM_ERR;
    }

#ifdef SM_DEBUG_CODE
    SmBoolean bDebugMe = FALSE;
  SM_PTR_ARRAY(sDbgPolyFaces, SmPolyFace, 256) ;
  pSC->mTS_pPolyBrep->GetPolyFaces(sDbgPolyFaces) ;

  // large PolyFace count tessellations are likely to have problems
  if(sDbgPolyFaces.GetSize() > 512)
    {
      if (bDebugMe)
        {
          SM_ASSERT_VALID(pSC) ;
          SM_ASSERT_VALID(pFace) ;
          SM_ASSERT_VALID(pSC->mTS_pPolyBrep) ;

          if ( FALSE )
            { smgfx_Erase(); }
          smgfx_SetLook(2,3, 1,0,0); if(pFace) pFace->DrawUV(8,8,FALSE,NULL,TRUE); sm_GraphicsLoop(); // Face isoParam lines in 3d
          smgfx_SetLook(2,3, 1,0,0); if(pFace) pFace->DrawUVCurves(FALSE); sm_GraphicsLoop();         // Face UVCurves in 3d & on z=0 plane
          smgfx_SetLook(1,2, 0,0,0); if(m_pTessBrep) m_pTessBrep->Draw(TRUE) ; sm_GraphicsLoop();     // Brep tessellation in 3d
          smgfx_SetLook(2,1, 0,1,0); if(pSC) pSC->DrawSubdivision2D(); sm_GraphicsLoop();        // Face subdivision in 2d on z=0 plane
          smgfx_SetLook(2,1, 0,1,0); if(pSC) pSC->DrawSubdivision3D(FALSE); sm_GraphicsLoop();   // Face subdivision in 3d
          smgfx_SetLook(1,2, 0,0,0); if(pSC) pSC->DrawTessellation2D(); sm_GraphicsLoop();  // Face tessellation Brep in 2d on z=0 plane
          smgfx_SetLook(1,2, 0,0,0); if(pSC) pSC->DrawTessellation3D(); sm_GraphicsLoop();  // Face tessellation Brep in 3d
          smgfx_SetLook(1,2, 1,0,0); if(pFace) pFace->DrawSubdivisions() ; sm_GraphicsLoop();
          sm_GraphicsLoop();

          ULONG ii, jj ;
          SmTArray<SmPolyLoop*> aLoops;
          SmTArray<SmPolyFace*> aFaces;
          pSC->mTS_pPolyBrep->GetPolyFaces( aFaces );
          smgfx_SetLook(2,4, 0,0,1);
          for ( ii=0; ii< aFaces.GetSize(); ii++ )
            {
              SmPolyFace *pPF = aFaces[ii];
              pPF->GetPolyLoops( aLoops );
              for ( jj=0; jj< aLoops.GetSize(); jj++ )
                {
                  (aLoops[jj])->Draw(); sm_GraphicsLoop();
                }
              SmTArray<SmPolyEdge*> sPolyEdges;
              pPF->GetPolyEdges(sPolyEdges);
              for (ULONG ijk=0; ijk<sPolyEdges.GetSize(); ijk++)
                {
                   smgfx_SetLook(1,2, 0,0,1); sPolyEdges[ijk]->Draw  (); sm_GraphicsLoop();
                   smgfx_SetLook(1,2, 0,0,1); sPolyEdges[ijk]->Draw3D(); sm_GraphicsLoop();
                }
              sm_GraphicsLoop();
            }
          sm_GraphicsLoop();
        }
    } // end PolyFace count is large check
#endif // SM_DEBUG_CODE

  // all done
  return SM_SUCCESS;

} // end SmTess::Phase2CreateFacePolygons

/*******************************************************************//**
PURPOSE: Final phase of tessellation - clean up pFace's SmTessSrfCache.

NOTES: Remove <pFace,pTessSrfCache(pFace->Surface)> pair from m_vCache
       table and delete temp pSC = pTessSrfCache object.
***********************************************************************/
SmStatus SmTess::Phase3DeleteFacePolygons
 (SmFace *pFace)
{
  SmTessSrfCache *pSC = m_vCache.GetSecond(pFace);
  if (pSC)
    {
      m_vCache.RemovePair(pFace, pSC);
      delete pSC; pSC = NULL ;
    }

  // all done

  return SM_SUCCESS;

} // end SmTess::Phase3DeleteFacePolygons

/*******************************************************************//**
PURPOSE: Try to split a segment using a projection point. Skips
         splitting segments near their endPoints.

NOTES: 1. Splits both the given SmPolyEdge and its associated SmPolyEdge->EdgeUse->Edge
          as a pair keeping the association between the Edges of the Brep Model
          one to one with the PolyEdges of the PolyBrep Model.

METHOD: 1. Get pPEdgeToSplit->Edgeuse->Edge->UVTrimCurve
        2. Drop PointToSplit to UVTrimCurve
        3. skip splitting segments near endPoints
        4. Split Edge at DropPoint with MakeVertexSplitEdge()
        5. Split PolyEdge at Edge->UVTrimCurve->SplitPoint with MakeVertexSplitPolyEdge()
        6. Set PolyEdge->Edgeuse back pointers
***********************************************************************/
SmStatus SmTess::TrySplitSegment
 (SmPolyEdge      * pPEdgeToSplit,     // in : UV PolyEdge to split
  const SmPoint3d & crPointToSplit,    // in : target UV Split Point
  SmBoolean       & rbSegmentIsSplit)  // out: TRUE = pPEdgeToSplit and pPEdgeToSplit->Edgeuse->Edge where split
                                       //      FALSE= no Edges or PolyEdges were split
{
  // init output
  rbSegmentIsSplit = FALSE;

  // locals
  SmSolution       aSData[16];
  SmSolutionArray  sSolutions(16,aSData);
  SmPoint3d        sSplitPnt(crPointToSplit);
  SmBSplineCurve * pUVCurve = NULL ;
  SmEdge         * pNewEdge1 = NULL, * pNewEdge2 = NULL;
  SmVertex       * pNewVertex = NULL;

  // pEdgeToSplit must be associated with a Brep Edgeuse with a UVTrimCurve and an Edge
  SmEdgeuse * pPEU  = GetPolyEdgeEU(pPEdgeToSplit); NER(pPEU) ;
  SmEdge    * pEdge = pPEU->GetEdge();              NER(pEdge) ;
  SmBrep    * pBrep = pEdge->GetBrep();             NER(pBrep) ;
  SER( pPEU->GetOrCreateUVTrimCurve( pUVCurve ));
  NER( pUVCurve );
  SmExtent1d sIvl = pUVCurve->GetNaturalInterval();

  // drop SplitPnt down to UVTrimCurve
  SER(pUVCurve->GlobalPointSolve(pUVCurve->GetNaturalInterval(),
                                 SM_SO_MINIMIZE,
                                 sSplitPnt,
                                 SM_EFF_ZERO,
                                 NULL,
                                 NULL,
                                 SM_SR_SINGLE,
                                 sSolutions));

  // exit - when SplitPnt did not drop to UVTrimCurve
  if (sSolutions.GetSize() == 0)
    { return SM_SUCCESS; }

  // exit - don't split segment when SplitPoint is near segment endPoint
  double dParam = sSolutions[0].m_vStart[0];
  double dMin   = sIvl.Evaluate(0.0001);
  double dMax   = sIvl.Evaluate(0.9999);
  if (dParam < dMin || dParam > dMax)
    { return SM_SUCCESS; }

  // arrive here after all checks passed and time to split pSeg->Edgeuse->Edge

  // split pEdgeToSplit->Edgeuse->Edge
  if (pBrep->MakeVertexSplitEdge(pEdge,dParam,pNewEdge1,pNewEdge2,pNewVertex) != SM_SUCCESS)
    {
      rbSegmentIsSplit = FALSE;
      return SM_SUCCESS;
    }

  // Get Edgeuse param for split point - watch out for orientation
  SmEdgeuse * pEU2             = pPEU->GetCCWEdgeuse();
  double      dNormalizedParam = 1.0;
  if (pEU2->GetEdge() != pNewEdge2)
    {
      dNormalizedParam = 0.0;
      pEU2 = pPEU->GetCWEdgeuse();
      if (pEU2->GetEdge() != pNewEdge2)
        { SER(SM_ERR); }
    }

  // evaluate Edgeuse at Split point
  SmPoint3d sPnt;
  SER(pPEU->NormalizedEvaluate(dNormalizedParam,TRUE,sPnt));    // TRUE = UV Eval, FALSE = 3d Eval

  // locals
  SmPolyFace   * pPolyFace = pPEdgeToSplit->GetPolyFace();
  SmPolyEdge   * pNewPEdge = NULL;
  SmPolyVertex * pNewPVertex = NULL;

  // propagate the Edgeuse split into the PolyEdge
  SER(pPolyFace->MakeVertexSplitPolyEdge(pPEdgeToSplit, // in : PolyEdge to split
                                         sPnt,          // in : Point split location
                                         pNewPEdge,     // out: new PolyEdge (and new radial partners)
                                         pNewPVertex)); // out: new PolyVertex

  // exit - when PolyEdge failed to split
  if(pNewPEdge == NULL)
    { return SM_SUCCESS; }

  // propagate Split Edgeuse pointers to Split PolyEdges
  if (dNormalizedParam > 0.5) { SetPolyEdgeEU(pNewPEdge,     pEU2);
                             // SetPolyEdgeEU(pPEdgeToSplit, pPEU);  // GWC: already set this way
                              }
  else                        { SetPolyEdgeEU(pNewPEdge,     pPEU);
                                SetPolyEdgeEU(pPEdgeToSplit, pEU2);
                              }

  // set output
  rbSegmentIsSplit = TRUE;

  // all done
  return SM_SUCCESS;

} // end SmTess::TrySplitSegment

/*******************************************************************//**
PURPOSE: Specialized property evaluator for SmTess::SmoothPolygons()

NOTES: 1. Given the list of Edges attached to an implied central Vertex
          compute, the min, max, and total areas of the implied triangles
          attached to the implied central vertex, and an area weighted
          centroid of all the vertex positions of all the triangles
          attached to the given list of edges.
       2. Assumes operations are in 2D and that all are triangles
***********************************************************************/
SmStatus sm_ComputeProperties
 (const SmPoint3d             & crReferencePoint, // in : a nearby central point (commonly the pos of the implied central vertex)
  const SmTArray<SmPolyEdge*> & crPolyEdges,      // in : List of Edges attached to some PolyVertex
                                                  //      assumed ordered:[StartAt, EndAt, StartAtPartner, EndAt, ...]
  SmPoint3d                   & rCentroid,        // out: Centroid = area weighted avg of all vertices attached to crPolyEdges
  double                      & rdArea,           // out: Sum of all triangle areas attached to implied central vertex
  double                      & rdMinArea,        // out: Min of all areas of Triangles connected to implied central vertex
  double                      & rdMaxArea)        // out: Max of all areas of Triangles connected to implied central vertex
{
#ifdef SM_DEBUG_CODE
  SmBoolean bDebugMe = FALSE;
  if (bDebugMe)
    {
      smgfx_Erase();
      smgfx_SetColor(1,0,0); crReferencePoint.Draw(); sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
#endif

  // init output
  rCentroid.Set(0,0,0);
  rdArea    = 0.0;
  rdMinArea = SM_BIG_DOUBLE;
  rdMaxArea = - SM_BIG_DOUBLE;

  // locals
  ULONG ii ;

  // for every PolyEdge Pair (assumes an PolyEdge ordering about a central vertex)
  for(ii=1; ii<crPolyEdges.GetSize(); ii++)
    {
      SmPolyEdge *pPrev = crPolyEdges[ii-1];
      SmPolyEdge *pNext = crPolyEdges[ii];
      ii++;
      SmPoint3d sP0 = pPrev->GetEndPoint()   - crReferencePoint;
      SmPoint3d sP1 = pNext->GetStartPoint() - crReferencePoint;

#ifdef SM_DEBUG_CODE
      if (bDebugMe)
        {
          smgfx_SetLook(3,4, 0,0,1); sP0.Draw(); sm_GraphicsLoop();
          smgfx_SetLook(3,4, 0,1,0); sP1.Draw(); sm_GraphicsLoop();
          smgfx_SetLook(1,2, 0,1,0); pPrev->GetPolyFace()->Draw(); sm_GraphicsLoop();
          sm_GraphicsLoop();
        }
#endif
      // GWC: old section - removed for bug fix
      //   double a = sP0.x*sP1.y - sP1.x*sP0.y; // Cross product
      //   rCentroid.x += a * (sP0.x + sP1.x);
      //   rCentroid.y += a * (sP0.y + sP1.y);
      //   double dTriArea = a / 2.0;
      //   rdArea += dTriArea;

      // GWC: new section - needed for bug fix
      double dTriArea = (sP0.x*sP1.y - sP1.x*sP0.y) / 2.0 ;  // Cross product/2.0
      rCentroid.x += dTriArea * (sP0.x + sP1.x);             // Centroid is an area weighted avg of Point positions
      rCentroid.y += dTriArea * (sP0.y + sP1.y);             // Centroid is an area weighted avg of Point positions
      rdArea += dTriArea;

      if (dTriArea < rdMinArea) { rdMinArea = dTriArea; }
      if (dTriArea > rdMaxArea) { rdMaxArea = dTriArea; }
    }

  // GWC: replaced one line
  rCentroid /= rdArea ;
  // rCentroid = rCentroid / (6.0 * rdArea);  // GWC: this looks like a bug - I think the 6 implies there are only 3
                                              //      edges attached to the implied central vertex
#ifdef SM_DEBUG_CODE
  if (bDebugMe)
    {
      smgfx_SetColor(1,0,0); rCentroid.Draw(); sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
#endif

  rCentroid.x = crReferencePoint.x + rCentroid.x;
  rCentroid.y = crReferencePoint.y + rCentroid.y;
  return SM_SUCCESS;

} // end sm_ComputeProperties

/*******************************************************************//**
PURPOSE: Smooth PolyFaces of a PolyBrep by moving all the Manifold
  vertices towards the centroid of verices in all faces attached to this vertex

NOTES: 
  1.) control:
   m_vSmoothingData.m_lSmoothingPasses    = Number of iterations before quitting
   m_bAdvancingFront                      = TRUE = a little costly - smooth every vertex every iteration
                                            FALSE= stop smoothing a vertex location once the
                                                   triangles attached to the vertex pass an Area ratio criteria.
   m_vSmoothingData.m_dMinSmoothingRatio  = when m_bAdvancingFront == FALSE
                                            stop smoothing a vertex location once
                                            its Max/Min attached tri area ratio < m_vSmoothingData.m_dMinSmoothingRatio.
   m_vSmoothingData.m_lSmoothingTechnique = currently has to be equal to 1
   m_vSmoothingData.m_dSmoothingStepSize  = ex: 1.0 = vertices are moved to the centroid each iteration
                                                0.5 = vertices are moved half way to the centroid each iteration
***********************************************************************/
SmStatus SmTess::SmoothPolygons
 (SmPolyBrep *pPolyBrep)         // in : target PolyBrep
{
  // locals
  ULONG ii, jj, kk, lPasses ;
  SM_PTR_ARRAY(sEdges,        SmPolyEdge,   256) ; // SmTArray<SmPolyEdge*>
  SM_PTR_ARRAY(sFaceEdges,    SmPolyEdge,    32) ; // SmTArray<SmPolyEdge*>
  SM_PTR_ARRAY(sVertices,     SmPolyVertex, 256) ; // SmTArray<SmPolyVertex*>
  SM_PTR_ARRAY(sCoinVertices, SmPolyVertex,  32) ; // SmTArray<SmPolyVertex*>
  pPolyBrep->GetPolyVertices(sVertices);

#ifdef SM_DEBUG_CODE
  SmBoolean bDebugMe = FALSE;
  if (bDebugMe)
    {
      smgfx_Erase();
      smgfx_SetColor(0,0,0); pPolyBrep->Draw(); sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

  // iter m_vSmoothingData.lSmoothingPasses number of smoothing passes
  for(lPasses=0; lPasses<m_vSmoothingData.m_lSmoothingPasses; lPasses++)
    {
      // For every vertex - Move vertex towards centroid of vertices in all faces attached to this vertex
      for (ii=0; ii<sVertices.GetSize(); ii++)
        {
          SmPolyVertex *pV = sVertices[ii];

          // skip NonManifold vertices
          if(!pV->IsManifoldVertex())
            { continue; }

          // locals
          pV->GetPolyEdges(sEdges);
          SmBoolean bIsInteriorVertex = TRUE;

          // for every PolyVertex->Edge - see if its attached to an Original Brep Edgeuse
          for (jj=0; jj<sEdges.GetSize(); jj++)
            {
              SmPolyEdge *pE = sEdges[jj];

              // remember when PolyEdge comes from an Original Brep Edge
              if (GetPolyEdgeEU(pE) != NULL)
                {
                  bIsInteriorVertex = FALSE;
                  break;
                }
//                if (m_bAdvancingFront) {
                  // Note that this prevents smoothing of first level
                  // polygons - we don't want to shorten edges to the
                  // boundary.  These are critical ones in smoothing.
//                    if (pE->GetEndPolyVertex()->IsLaminaVertex()) {
//                        bIsInteriorVertex = FALSE;
//                    }
//                    if (pE->GetStartPolyVertex()->IsLaminaVertex()) {
//                        bIsInteriorVertex = FALSE;
//                    }
//                }
            } // end iter every PolyVertex->Edge seeing if its attached to an Original BrepEdgeuse

          // skip vertices attached to Edges representing original Brep Edges
          if (!bIsInteriorVertex)
            { continue; }

          // Note that poly edges come back in pairs.  The first one
          // has pV as its start vertex and the second one has pV
          // as its end vertex.

          SmPoint3d sCentroid;
          double dStartMinArea=0.0, dStartMaxArea, dArea;
          if (m_bAdvancingFront)
            {
              pV->GetStartingPolyEdges(sEdges);

              // locals
              SmPoint3d sTotalPnt(0,0,0);
              ULONG     lTotal = 0;

              // for every PolyVertex->StartingEdge
              for (jj=0; jj<sEdges.GetSize(); jj++)
                {
                  SmPolyFace *pF = sEdges[jj]->GetPolyFace();
                  pF->GetPolyEdges(sFaceEdges);

                  // for every StartingEdge->Face->Edge
                  for (kk=0; kk<sFaceEdges.GetSize(); kk++)
                    {
                      SmPolyEdge *pFE = sFaceEdges[kk];
                      if (pFE->GetStartPolyVertex() != pV)
                        {
                          lTotal++;
                          sTotalPnt = sTotalPnt + pFE->GetStartPoint();
                        }
                    } // end iter every StartingEdge->Face->Edge
                } // end iter every PolyVertex->StartingEdge

              // get the Centroid
              if ( lTotal < 1 ) { lTotal = 1; }
              sCentroid = sTotalPnt/lTotal;

            } // end m_bAdvancingFront == TRUE branch
          else // m_bAdvancingFront == FALSE
            {
              // Compute a minimum and maximum area for each triangle
              // based on the point owned by pV
              SER(sm_ComputeProperties(pV->GetPoint(),  // in : a nearby central point (commonly the pos of the implied central vertex)
                                       sEdges,          // in : List of Edges attached to some PolyVertex
                                                        //      assumed ordered:[StartAt, EndAt, StartAtPartner, EndAt, ...]
                                       sCentroid,       // out: Centroid = area weighted avg of all vertices attached to crPolyEdges
                                       dArea,           // out: Sum of all triangle areas attached to implied central vertex
                                       dStartMinArea,   // out: Min of all areas of Triangles connected to implied central vertex
                                       dStartMaxArea)); // out: Max of all areas of Triangles connected to implied central vertex
              if (dStartMinArea < 0.0)
                { SER(SM_ERR); } // Big problems

              // skip vertices connected to acceptable triangles
              if (dStartMaxArea/dStartMinArea < m_vSmoothingData.m_dMinSmoothingRatio)
                { continue; } // Ratio is acceptable don't bother smooth

            } // end m_bAdvancingFront == FALSE branch

          // Now do averaging to compute new point and check if it
          // improves the ratios.
          SmPoint3d sTargetPoint;
          if (m_vSmoothingData.m_lSmoothingTechnique == 1)
            {  sTargetPoint = sCentroid; }
          else
            { SER(SM_ERR); } // Other smoothing techniques not implemented yet

          SmVector3d sMoveVec    = sTargetPoint - pV->GetPoint();
          SmPoint3d  sDeltaPoint = pV->GetPoint() + m_vSmoothingData.m_dSmoothingStepSize * sMoveVec;

          // when asked - see if we can save some moves
          if (!m_bAdvancingFront)
            {
              double dFinalMinArea,dFinalMaxArea;
              SER(sm_ComputeProperties(sDeltaPoint,     // in : a nearby central point (commonly the pos of the implied central vertex)
                                       sEdges,          // in : List of Edges attached to some PolyVertex
                                                        //      assumed ordered:[StartAt, EndAt, StartAtPartner, EndAt, ...]
                                       sCentroid,       // out: Centroid = area weighted avg of all vertices attached to crPolyEdges
                                       dArea,           // out: Sum of all triangle areas attached to implied central vertex
                                       dFinalMinArea,   // out: Min of all areas of Triangles connected to implied central vertex
                                       dFinalMaxArea)); // out: Max of all areas of Triangles connected to implied central vertex

              // skip vertex moves that don't improve the MinArea size
              if (dFinalMinArea < dStartMinArea)
                { continue; } // Did not see improvement don't touch vertex.

            } // end need to see area improvement check

          pV->SetPoint(sDeltaPoint);

#ifdef SM_DEBUG_CODE
          SmBoolean bDebugMe2 = FALSE;
          if (bDebugMe2)
            {
              smgfx_Erase();
              smgfx_SetColor(1,0,0); pPolyBrep->Draw(); sm_GraphicsLoop();
              sm_GraphicsLoop();
            }
#endif // SM_DEBUG_CODE

        } // end iter every vertex - Move vertex towards centroid of vertices in all faces attached to this vertex

#ifdef SM_DEBUG_CODE
        SmBoolean bDebugMe3 = FALSE;
      if (bDebugMe3)
        {
          smgfx_Erase();
          smgfx_SetColor(0,0,1); pPolyBrep->Draw(); sm_GraphicsLoop();
          sm_GraphicsLoop();
        }
#endif
    } // end iter m_vSmoothingData.lSmoothingPasses number of smoothing passes

  return SM_SUCCESS;

} // end SmTess::SmoothPolygons

/*******************************************************************//**
PURPOSE: Find a segment in the loop to split that is closest to the
   given point.

NOTES: 
***********************************************************************/
SmStatus SmTess::SplitSegmentOfLoop
 (SmPolyLoop      * pLoop,
  const SmPoint3d & crTestPoint,
  SmBoolean       & rbSplitDone)
{
  rbSplitDone = FALSE;

  SM_PTR_ARRAY(sEdges, SmPolyEdge, 256) ; // SmTArray<SmPolyEdge*>
  pLoop->GetPolyEdges(sEdges);

  SmPolyEdge * pEFound = NULL;
  double dMinDist = SM_BIG_DOUBLE;

  ULONG lNumEdges = sEdges.GetSize();
  for (ULONG ii=0; ii<lNumEdges; ii++)
    {
      SmPolyEdge * pSeg = sEdges[ii];
      SmEdgeuse *pEU = GetPolyEdgeEU(pSeg);
      if ( !pEU ) { continue; }
      SmBSplineCurve *pUVCurve = pEU->GetUVTrimCurve();
      if ( !pUVCurve ) { continue; }

      SmSolution aSData[16];
      SmSolutionArray sSolutions(16,aSData);
      SmExtent1d sIvl = pUVCurve->GetNaturalInterval();
      SmPoint3d sSplitPnt(crTestPoint);
      SER(pUVCurve->GlobalPointSolve(pUVCurve->GetNaturalInterval(),SM_SO_MINIMIZE,
          sSplitPnt,SM_EFF_ZERO,NULL,NULL,SM_SR_SINGLE,sSolutions));

      if (sSolutions.GetSize() == 0)
        {
          continue;
        }

      SmSolution & rSol = sSolutions[0];
      if (rSol.m_vStart.m_dSolutionValue < dMinDist)
        {
          pEFound = pSeg;
          dMinDist = rSol.m_vStart.m_dSolutionValue;
        }
    }

  if (pEFound != NULL)
    {
      SER(TrySplitSegment(pEFound,crTestPoint,rbSplitDone));
      return SM_SUCCESS;
    }

  return SM_SUCCESS;

} // end SmTess::SplitSegmentOfLoop

/*******************************************************************//**
PURPOSE: Test the current tree node against the tolerances.

NOTES: 
***********************************************************************/
SmStatus SmTessSrfCache::TestAgainstTolerances
 (SmTreeNode      * pNode,                   // in : node to test
  SmBoolean       & rbNeedsSubdivision,      // out: TRUE = patch fails some tessellation test
  SmSurfParamType & reSubdivisionDirection)  // out: suggested direction to split bad elements SM_SP_U or SM_SP_V            
{
  // Make all the standard surface Tolerance tests
  // m_dChordHeightTolerance;     // max allowed Element ControlPoint to BasePlane3d dist3d,      0.0 = ignore
  // m_dAngTolRad;                // max allowed Element ControlPolygon turning ang3d (radians),  0.0 = ignore
  // m_dAspectRatio3D;            // max allowed Element BasePolygon aspect ratio3d,              0.0 = ignore
  // m_dMaximumSideLength3D;      // max allowed Element BasePolygon side length3d,               0.0 = ignore
  // m_dMinimumSideLength3D;      // min allowed Element BasePolygon side length3d,               0.0 = ignore, 0.0 = ignore
  // m_dMinimumSideLengthRatioUV; // min allowed Element (side lengthUV/origUVDomain.Size) ratio, 0.0 = ignore
  //                              //   note: smallest elements run about 1/2 of m_dMinimumSideLength3D when used.
  //                              //   note: smallest elements run about 1/2 of m_dMinimumSideLengthRatioUV when used.
  SER(SmSurfaceCache::TestAgainstTolerances(pNode,
                                            rbNeedsSubdivision,
                                            reSubdivisionDirection));

  // no more work - element not yet good enough to bother with viewBased Tessellation
  //                or not doing view specific tessellation
  if(   rbNeedsSubdivision
     || m_pSurfaceTessDriver->GetType() != SmViewBasedTessDriver_TYPE)
    { return SM_SUCCESS; }

  // arrive here once node passes all the regular surface tessellation tests

  // next: 1. test node to see if its part of the silhouetted boundary for this
  //          viewing direction = (SmViewBasedTessDriver *)m_pSurfaceTessDriver::m_vViewVector.
  //       2. Subdivide silhouette bounding nodes to tighter tolerances stored in
  //          (SmViewBasedTessDriver *)m_pSurfaceTessDriver

  // locals
  SmTessSrfCache        * pSurfaceCache = this ;
  SmViewBasedTessDriver * pTessDriver   = (SmViewBasedTessDriver *) m_pSurfaceTessDriver ;
  SmBezierAux2d         * pAux          = (SmBezierAux2d*)pNode->m_pData;
  SmExtent2d              sUVDomain     = pAux->m_sUVDomain;
  const SmSurface       * cpSurToTest   = pSurfaceCache->GetSurface();

  // define samples - center and four corners
  SmPoint2d sUVs[5];
  sUVs[0] = sUVDomain.Evaluate(0.5,0.5);
  sUVs[1] = sUVDomain.Evaluate(0.0,0.0);
  sUVs[2] = sUVDomain.Evaluate(0.0,1.0);
  sUVs[3] = sUVDomain.Evaluate(1.0,0.0);
  sUVs[4] = sUVDomain.Evaluate(1.0,1.0);

  // see if node contains part of this viewing direction's silhoutte curve by seeing if
  //   sample point surface normals span the orthogonal to the viewing vector.
  // This seems to be a cheap, but not as conservative, polar box test. Can it mis-classify nodes?
  //   Unlikely since nodes here are mostly planar due to the previous SmSurfaceCache::TestAgainstTolerances check.
  SmBoolean bSilhouette = FALSE;

  // init a direction value to remember which side of the
  // orthogonal to the view vector this node's surface normals are pointing
  double dDirection = 0.0;

  // for every sample
  for (ULONG i=0; i<5; i++)
    {
      // get sample surface normal
      SmVector3d sNorm;
      SER(cpSurToTest->EvaluateNormal(sUVs[i],TRUE,TRUE,sNorm));
      double dDot = sNorm.Dot(pTessDriver->GetViewVector());

      // This node is part of the silhouette
      // when a subsequent sample normal differs in direction from another
      if (dDot * dDirection < 0.0)
        {
          bSilhouette = TRUE;
          break;
        }

      // remember the direction
      dDirection = dDot;
    }

  // no work - node is not part of the silhouette for this viewing angle
  if (!bSilhouette)
    { return SM_SUCCESS; }

  // switch to the tighter silhouette tessellation tolerances
  SmTemporaryChangeValue<double> sChangeCH(m_dChordHeightTolerance, pTessDriver->GetSilhouetteChordHeight());
  SmTemporaryChangeValue<double> sChangeAT(m_dAngTolRad,            pTessDriver->GetSilhouetteAngleTolerance());
  pAux->m_bChordHeightSatisfied = FALSE;
  pAux->m_bAngleTolSatisfied    = FALSE;

  // subdivide this node to the tighter silhouette tolerances
  SER(SmSurfaceCache::TestAgainstTolerances(pNode,
                                            rbNeedsSubdivision,
                                            reSubdivisionDirection));
  // all done
  return SM_SUCCESS;

} // end SmTessSrfCache::TestAgainstTolerances

//      /*******************************************************************//**
//      PURPOSE: View based tessellation where subdivision nodes that
//         lie on the silhouette boundary for the current view vector are
//         tessellated to silhouette tolerances while all other nodes
//         are tessellated to normal tolerances.
//
//      NOTES: 
//      ***********************************************************************/
//      SmStatus SmViewBasedTessDriver::TestAgainstTolerances
//        (SmTessSrfCache  * pSurfaceCache,
//         SmTree          * pTree,
//         SmTreeNode      * pNode,
//         SmBoolean       & rbNeedsSubdivision,
//         SmSurfParamType & reSubdivisionDirection)
//      {
//        // Make all the standard surface Tolerance tests
//        SER(SmSurfaceTessDriver::TestAgainstTolerances(pSurfaceCache,
//                                                       pTree,
//                                                       pNode,
//                                                       rbNeedsSubdivision,
//                                                       reSubdivisionDirection));
//
//        // no work - element not yet good enough for regular tessellation
//        if (rbNeedsSubdivision)
//          { return SM_SUCCESS; }
//
//        // arrive here once node passes all the regular surface tessellation tests
//
//        // locals
//        SmBezierAux2d    *pAux        = (SmBezierAux2d*)pNode->m_pData;
//        SmExtent2d        sUVDomain   = pAux->m_sUVDomain;
//        const SmSurface & crSurToTest = pSurfaceCache->GetSurface();
//
//        // define samples - center and four corners
//        SmPoint2d sUVs[5];
//        sUVs[0] = sUVDomain.Evaluate(0.5,0.5);
//        sUVs[1] = sUVDomain.Evaluate(0.0,0.0);
//        sUVs[2] = sUVDomain.Evaluate(0.0,1.0);
//        sUVs[3] = sUVDomain.Evaluate(1.0,0.0);
//        sUVs[4] = sUVDomain.Evaluate(1.0,1.0);
//
//        // look for a node whose sample point surface normals span the orhtogonal to the viewing vector
//        //   this seems to be either a poor man's substitute for the polar box or
//        //        a tighter way to test polar boxes.  But its not as conservative
//        //        as polar boxes and might mis-classify some nodes
//        SmBoolean bSilhouette = FALSE;
//
//        // init a direction value to remember which side of the
//        // orthogonal to the view vector this node's surface normals are pointing
//        double dDirection = 0.0;
//
//        // for every sample
//        for (ULONG i=0; i<5; i++)
//          {
//            // get sample surface normal
//            SmVector3d sNorm;
//            SER(crSurToTest.EvaluateNormal(sUVs[i],TRUE,TRUE,sNorm));
//            double dDot = sNorm.Dot(m_vViewVector);
//
//            // This node is part of the silhouette
//            // when a subsequent sample normal differes in direction from another
//            if (dDot * dDirection < 0.0)
//              {
//                bSilhouette = TRUE;
//                break;
//              }
//
//            // remember the direction
//            dDirection = dDot;
//          }
//
//        // no work - node is not part of the silhouette for this viewing angle
//        if (!bSilhouette) return SM_SUCCESS;
//
//        // switch to the tighter silhouette tessellation tolerances
//        SmTemporaryChangeValue<double> sChangeCH(m_dChordHeight, m_dSilhouetteChordHeight);
//        SmTemporaryChangeValue<double> sChangeAT(m_dAngTolDeg,m_dSilhouetteAngleToleranceDeg);
//        pAux->m_bChordHeightSatisfied = FALSE;
//        pAux->m_bAngleTolSatisfied    = FALSE;
//
//        // subdivide this node to the tighter silhouette tolerances
//        SER(SmSurfaceTessDriver::TestAgainstTolerances(pSurfaceCache,
//                                                       pTree,
//                                                       pNode,
//                                                       rbNeedsSubdivision,
//                                                       reSubdivisionDirection));
//        // all done
//        return SM_SUCCESS;
//
//      } // end SmViewBasedTessDriver::TestAgainstTolerances


//      /*******************************************************************//**
//      PURPOSE: Test the current tree node against the tolerances in the
//           surface tessellation driver.
//
//      NOTES: 
//      ***********************************************************************/
//      SmStatus SmSurfaceTessDriver::TestAgainstTolerances
//        (SmTessSrfCache  * pSurfaceCache,
//         SmTree          * pTree,
//         SmTreeNode      * pNode,
//         SmBoolean       & rbNeedsSubdivision,
//         SmSurfParamType & reSubdivisionDirection)
//      {
//        // init output
//        rbNeedsSubdivision     = FALSE;
//        reSubdivisionDirection = SM_SP_U;
//
//        // check state - nodes are properly labeled and have not yet been split
//        if(   pNode->m_eAuxDataType != SM_AD_BEZIER_SURFACE
//           && pNode->m_eAuxDataType != SM_AD_AUX_DATA)
//         { SER(SM_ERR) ; }
//
//        if(   pNode->m_pChild1 != NULL)
//          { SER(SM_ERR) ; }
//
//        // locals
//        SmBezierAux2d   *pAux                = (SmBezierAux2d*)pNode->m_pData;
//        SmPseudoBox      sPSBox, *pPSBox     = &sPSBox;
//        gw_SURFACE      *pSur                = NULL;
//
//        double           dChordHeightSquared =   m_dChordHeight
//                                               * m_dChordHeight;
//        double           dAngleTolDeg        =   m_dAngTolDeg;
//        double           dUChordHeight, dVChordHeight;
//        double           dUAngleDeg,    dVAngleDeg;
//        SmBoolean        bForceAngleTest ;
//        SmBoolean        bSubdivide          = FALSE;
//        SmSurfParamType  eSubdivideDirection = SM_SP_U;
//
//        SmBoolean bBothTolerancesSatisfied = pAux->m_bChordHeightSatisfied && pAux->m_bAngleTolSatisfied;
//
//
//        // When node is from SmTessSrfCache::BuildTree for a BSplineSurface
//        //              through a BuildTreeWithSubdivision() call it has
//        //              an m_pData object WITHOUT a bezier gw_SURFACE patch
//        //              and an m_pData object with an mBA_pSurface value
//        if (pNode->m_eAuxDataType == SM_AD_AUX_DATA)
//          {
//            bForceAngleTest      = TRUE;
//            pPSBox               = &sPSBox;
//            pSur                 = pAux->mBA_pSurface->GetGwNurbPointer();
//
//            if (!bBothTolerancesSatisfied)
//              {
//                SER(pAux->mBA_pSurface->CalculateBoundingBox(pAux->m_sUVDomain,
//                                                           &pNode->m_sBBox,
//                                                           &sPSBox));
//              }
//          }
//        else // node is from SmSurfaceCache::BuildTree for any kind of surface
//             //         through a BuildTreeBase() call it has
//             //         an m_pData object WITH a bezier gw_SURFACE patch
//             //         and an m_pData object WITHOUT an mBA_pSurface value.
//          {
//            SM_ASSERT(pNode->m_eAuxDataType == SM_AD_BEZIER_SURFACE)
//            SmBezierPatch *pBezPatch = (SmBezierPatch*)pNode->m_pData;
//
//            bForceAngleTest          = FALSE;
//            pPSBox                   = &pBezPatch->m_sPseudoBox;
//            pSur                     = pBezPatch->GetBezierPtr();
//          }
//
//
//        // When the surface is not a SmBSplineSurface
//        //  - Get a Surface pointer and a subdomain.
//        //  These values force sm_ComputeNetConstants() to
//        //    generate approximate ChordHeight and Turning Angle values
//        //    by sampling the surface.
//        const SmSurface  *pSurToTest = NULL;
//        const SmExtent2d *pUVDomain  = NULL;
//        if (!pSurfaceCache->m_crSurface.IsKindOf(SmBSplineSurface_TYPE))
//          {
//            pSurToTest = &pSurfaceCache->m_crSurface;
//            pUVDomain  = &pAux->m_sUVDomain;
//          }
//
//        // chordheight
//        if (!bBothTolerancesSatisfied)
//          {
//            if (dChordHeightSquared > SM_EFF_ZERO)
//              {
//                SmExtent1d sIvl1, sIvl2, sIvl3;
//                pPSBox->GetIntervals(sIvl1,sIvl2,sIvl3);
//                double dHeight = sIvl3.GetMax() - sIvl3.GetMin();
//                if (dHeight*dHeight > dChordHeightSquared)
//                  {
//                    bSubdivide = TRUE;
//                    // note: pSurToTest is only notNULL for non SmBSPlineSurface objects
//                    SER(pSurfaceCache->ComputeNetConstants(pSurToTest,      // in : notNULL = test surface values to compute outputs
//                                                           pUVDomain,       // in : only used when pSurToTest != NULL, target domain
//                                                           pSur,            // in : Always used - target shape to test
//                                                                            //      when pNode->m_eAuxDataType == SM_AD_AUX_DATA
//                                                                            //        pSUr = SmBezierAux2d mBA_pSurface->gw_SURFACE
//                                                                            //      when pNode->m_eAuxDataType == SM_AD_BEZIER_SURFACE
//                                                                            //        pSur = SmBezierPatch gw_SURFACE
//                                                           &dUChordHeight,  // out:
//                                                           &dVChordHeight,  // out:
//                                                           NULL,            // out:
//                                                           NULL));          // out:
//                    if (dUChordHeight < dVChordHeight)
//                      {
//                        eSubdivideDirection = SM_SP_V;
//                      }
//                  }
//                else
//                  {
//                    pAux->m_bChordHeightSatisfied = TRUE;
//                  }
//              }
//            else
//              {
//                pAux->m_bChordHeightSatisfied = TRUE;
//              } // end ChordHeight
//
//
//            // AngleTol
//            if (dAngleTolDeg < SM_EFF_ZERO)
//              {
//                pAux->m_bAngleTolSatisfied = TRUE;
//              }
//
//            if (   bForceAngleTest                        // only TRUE for nodes from SmTessSrfCache::BuildTree() for SmBSplineSurface objects
//                || (   !bSubdivide
//                    && dAngleTolDeg > SM_EFF_ZERO))
//              {
//                // note: pSurToTest is only notNULL for non SmBSPlineSurface objects
//                SER(pSurfaceCache->ComputeNetConstants(pSurToTest,          // in : notNULL = test surface values to compute outputs
//                                                       pUVDomain,           // in : only used when pSurToTest != NULL, target domain
//                                                       pSur,                // in : Always used - target shape to test
//                                                                            //      when pNode->m_eAuxDataType == SM_AD_AUX_DATA
//                                                                            //        pSUr = SmBezierAux2d mBA_pSurface->gw_SURFACE
//                                                                            //      when pNode->m_eAuxDataType == SM_AD_BEZIER_SURFACE
//                                                                            //        pSur = SmBezierPatch gw_SURFACE
//                                                       NULL,                // out:
//                                                       NULL,                // out:
//                                                       &dUAngleDeg,         // out:
//                                                       &dVAngleDeg));       // out:
//
//                // Optimization for SmTessSrfCache::BuildTree().  When node angle
//                //    goes below m_dStartFastSubdivisionAngleDeg value AND m_eTessellationAlgorithm == SM_TA_FASTER_TESSELLATION
//                //    Then switch to SubdivideByBlock() from SubdivideNode() in SubdivideToTolerances().
//
//                // note: bForceAngleTest is only true for nodes from SmTessSrfCache::BuildTreeWithSubdivision()
//                //                       when ((SmBezierAux2d *)m_pData)->mBA_pSurface != NULL
//                if(   bForceAngleTest
//                   && dUAngleDeg < pSurfaceCache->m_dStartFastSubdivisionAngleDeg
//                   && dVAngleDeg < pSurfaceCache->m_dStartFastSubdivisionAngleDeg)
//                  {
//                    // change SubdivideToTolerances() behavior
//                    //   TRUE = when m_eTessellationAlgorithm == SM_TA_FASTER_TESSELLATION: call SubdivideByBlock()
//                    //          else                                                        call SubdivideNode()
//                    //   FALSE= call SubdivideNode()
//                    pSurfaceCache->m_bSwitchToFastSubdivision = TRUE;
//                  }
//
//                // Subdivide failing patches
//                if(   !bSubdivide
//                   && (   dUAngleDeg > dAngleTolDeg
//                       || dVAngleDeg > dAngleTolDeg))
//                  {
//                    bSubdivide = TRUE;
//                    // Split the direction with the largest angle.
//                    if (dVAngleDeg > dUAngleDeg) eSubdivideDirection = SM_SP_V;
//                  }
//                else
//                  {
//                    pAux->m_bAngleTolSatisfied = TRUE;
//                  }
//              } // end AngleTol
//          } // end bBothTolerancesSatisfied not satisfied check
//
//        // when ChordHeight and AngleTol checks are passed - this is a flat patch
//        //   AspectRatio3D,
//        //   MaximumSideLength3D,
//        //   MinimumSideLength3D
//        if (!bSubdivide)
//          {
//            if(   m_dMaxAspectRatio  != 0.0
//               || m_dMaxEdgeLength3D != 0.0
//               || m_dMinEdgeLength3D != 0.0)
//              {
//                SmPoint3d  sU0V0, sU1V0, sU1V1, sU0V1;
//                SmExtent2d sDomain;
//                SmPoint2d  sAspectRatio, sMaxSideLength3D, sMinSideLength3D;
//
//                // compute patch AspectRatio, MaxSideLength, MinSideLength - from corner positions
//                SER(pSurfaceCache->GetCorners(pSur,pAux->m_sUVDomain,sU0V0,sU1V0,sU1V1,sU0V1,sDomain));
//                SER(pSurfaceCache->ComputeSizeTolerances(sU0V0,sU1V0,sU1V1,sU0V1,
//                                                         sAspectRatio,
//                                                         sMaxSideLength3D,
//                                                         sMinSideLength3D));
//
//                // MaximumSideLength3D
//                if(   m_dMaxEdgeLength3D != 0.0
//                   && (   sMaxSideLength3D.x > m_dMaxEdgeLength3D
//                       || sMaxSideLength3D.y > m_dMaxEdgeLength3D) )
//                  {
//                    bSubdivide = TRUE;
//                  }
//
//                // AspectRatio3D
//                else if(   m_dMaxAspectRatio != 0.0
//                        && (   sAspectRatio.x > m_dMaxAspectRatio
//                            || sAspectRatio.y > m_dMaxAspectRatio))
//                  {
//                    bSubdivide = TRUE;
//                  }
//
//                // divide biggest dimension
//                eSubdivideDirection =   (sMaxSideLength3D.x > sMaxSideLength3D.y)
//                                      ? SM_SP_U
//                                      : SM_SP_V ;
//
//
//                // stop division if element is too small
//                if(   m_dMinEdgeLength3D != 0.0
//                   && sMaxSideLength3D.x     < m_dMinEdgeLength3D
//                   && sMinSideLength3D.y     < m_dMinEdgeLength3D)
//                  {
//                    bSubdivide = FALSE;
//                  }
//              }
//          } // end  AspectRatio3D, MaximumSideLength3D, MinimumSideLength3D
//
//
//        if (!bSubdivide)
//          {
//            return SM_SUCCESS;  // No further subdivision - passed all tests
//          }
//
//        // Perform a test here to make sure that we eventually stop subdivision
//        // For now don't let either node direction get less than 1/1000th of
//        // size of original domain
//        SmVector2d sUVSize   = pSurfaceCache->m_sUVDomain.GetSize();
//        SmVector2d sNodeSize = pAux->m_sUVDomain.GetSize();
//        double     dMinELR   =  (m_dMinEdgeLengthRatioUV != 0.0)
//                               ? m_dMinEdgeLengthRatioUV
//                               : 0.001;
//
//        // USide too small
//        if (sNodeSize.x < sUVSize.x * dMinELR)
//          {
//            if (sNodeSize.y > sUVSize.y / 100.0 && eSubdivideDirection == SM_SP_U)
//              {
//                eSubdivideDirection = SM_SP_V;
//              }
//            else
//              {
//                rbNeedsSubdivision = FALSE;
//                return SM_SUCCESS;
//              }
//          }
//
//        // VSide too small
//        if (sNodeSize.y < sUVSize.y * dMinELR)
//          {
//            if (sNodeSize.x > sUVSize.x / 100.0 && eSubdivideDirection == SM_SP_V)
//              {
//                eSubdivideDirection = SM_SP_U;
//              }
//            else
//              {
//                rbNeedsSubdivision = FALSE;
//                return SM_SUCCESS;
//              }
//          }
//
//        // set output
//        rbNeedsSubdivision     = TRUE;
//        reSubdivisionDirection = eSubdivideDirection;
//
//        // all done
//        return SM_SUCCESS;
//
//
//      } // end SmSurfaceTessDriver::TestAgainstTolerances

/*******************************************************************//**
PURPOSE: Build the tree using subdivision.

NOTES: 
  All TreeNode->m_pData types are SM_AD_AUX_DATA, i.e. they store
    no BezierPatches.
***********************************************************************/
SmStatus SmTessSrfCache::BuildTreeWithSubdivision
 (SmBSplineSurface *pBSS)           // in : BSpline Surface being tessellated
{
  // allocate the decomposition tree stored within the Cache
  m_pTree = new (*GetContext()) SmTree(this);

  // Initialize node block manager
  ULONG lPatchCount = 64;
  ULONG lNumPerBlock = smos_Min(50,(3 * lPatchCount) + 16);
  m_pTree->m_sNodeMgr.Initialize(ALIGN_SIZE(sizeof(SmTreeNode)), lNumPerBlock);

  // initialize parent subdivision tree m_pData objects
  m_sAuxMgr.Initialize(ALIGN_SIZE(sizeof(SmBezierAux2d)), lNumPerBlock) ;

  // to prevent bad recursion back into BuildTree - turn off point testing
  SmTemporaryChangeValue<SmBoolean> sTCV(m_bPointTestEnabled, FALSE);

  // Now that we have things initialized let's start building a tree.

  // allocate 1 top node and its m_pData object
  SmTreeNode     *pTopNode = (SmTreeNode*)m_pTree->m_sNodeMgr.GetNewElement();
  SmBezierAux2d  *pAux     = (SmBezierAux2d*)m_sAuxMgr.GetNewElement();

  m_pTree->m_pTopNode      = pTopNode;

  // set pNode and pAux data
  pTopNode->m_pTree        = m_pTree;
  pTopNode->m_pParent      = NULL;
  pTopNode->m_pChild1      = NULL;
  pTopNode->m_pChild2      = NULL;
  pTopNode->m_pData        = pAux;
  pTopNode->m_eAuxDataType = SM_AD_AUX_DATA;
  pTopNode->m_eGeomType    = SM_NG_DEFAULT;

  pAux->m_pOwningTreeNode       = pTopNode;
  pAux->m_eSplitDir             = SM_SP_U;
  pAux->m_bIsMarked             = FALSE;
  pAux->m_bIsTessOnly           = FALSE;
  pAux->m_eNodeClass            = SM_NC_UNKNOWN;
  pAux->m_pFace                 = NULL;
  pAux->m_sUVDomain             = pBSS->GetNaturalUVDomain();
  pAux->m_vVertexList.Init();
  pAux->m_sEdgeuseList.Init();
  pAux->m_sPolyEdgeList.Init();

// GWCTreeVertexTemp
//  // Create/Connect TreeVertex Loop:[0 1, 2, 3] = 0+---+3
//  //   around sUVDomain for 1st node               |   |
//  //                                              1+---+2
//  SmExtent2d sUVDomain = pBSS->GetNaturalUVDomain();
//  pAux->m_lStartTreeVertexIndx = m_pTree->SetRootTreeVertexLoop(sUVDomain, 0, 1, 2, 3, pTopNode) ;

// GWCTreeVertexTemp
//  #ifdef SM_DEBUG_CODE
//  SmBoolean bDebugMe = FALSE ;
//    if(bDebugMe)
//      {
//        SmBoolean bOK = m_pTree->AssertTreeVertices() ;
//        m_pTree->DumpTreeVertices() ;
//      }
//  #endif // SM_DEBUG_CODE

  // prepare for subdivision - point pAux to a copy of pBSS
  SmBSplineSurface *pBSSCopy = new (*GetContext()) SmBSplineSurface(*pBSS);
  pAux->mBA_pSurface         = pBSSCopy; // Just for the duration of subdivision
  m_sSubdivisionSurfaces.Add(pBSSCopy);

  pAux->m_bChordHeightSatisfied = FALSE;
  pAux->m_bAngleTolSatisfied    = FALSE;

  pBSSCopy->CalculateBoundingBox(pAux->m_sUVDomain, &pTopNode->m_sBBox, NULL, NULL);
  SM_ASSERT_MSG(!pTopNode->m_sBBox.HasNegativeVolume() , _T("TopNode has a negative volume")) ;

  // Subdivide 1st whole surface node into pieces that all meet tessellation requirements.
  // note: SmTessSrfCache::BuildTreeWithSubdivision() differs from SmSurfaceCache::BuildTreeBase()
  //        - SmTessSrfCache::BuildTreeWithSubdivision() calls SubdivideToTolerances()
  //             - with one node encompassing the whole surface
  //             - and pNode->m_eAuxDataType == SM_AD_AUX_DATA       (m_pData = (SmBezierAux2d *))
  //        - SmSurfaceCache::BuildTreeBase() calls SubdivideToTolerances()
  //             - with an array of nodes encompassing pieces of the whole surface
  //             - and pNode->m_eAuxDataType == SM_AD_BEZIER_SURFACE (m_pData = (SmBezierPatch *))
  SER(SubdivideToTolerances(m_pTree,pTopNode,NULL));

  // gwc: the following block could be added - but the values are not used so allow the arrays to be empty in AssertValid
  //  // Set the continuities
  //  //   Note: continuity is in opposite direction as to that which it was queried.
  //  SmContinuityType eMinCont;
  //  ULONG lNodeCount=0;
  //  SmTArray<SmContinuityType> & rUContinuities = *m_pUContinuities;
  //  SmTArray<SmContinuityType> & rVContinuities = *m_pVContinuities;
  //  SER(m_crSurface.CalculateContinuities(SM_SP_U, eMinCont, rUContinuities));
  //  SER(m_crSurface.CalculateContinuities(SM_SP_V, eMinCont, rVContinuities));

  // all done
  return SM_SUCCESS;

} // end SmTessSrfCache::BuildTreeWithSubdivision

/*******************************************************************//**
PURPOSE: Build TessSrfCache Tree, Imbed UVTrimCurves into TreeNodes, Label Nodes Inside/Outside/OnBoundary.
    1. Subdivide the m_pFace->surface using either chord height and/or angular
       tessellation tolerance to produce a piecewise Bezier representation of
       the surface.
    2. Then take the parameter space curves and vertices and put them into the
       leaf TreeNodes of the Subdivision Tree without any further Node subdivision.
    3. Then each TreeNode->m_pData->m_eNodeClass is marked as one of SM_NC_INSIDE
                                                                     SM_NC_OUTSIDE
                                                                     SM_NC_ON_BOUNDARY
NOTES: If one of the SmSurfaceCache stored tolerances is zero then that tolerance is
    not used in the subdivision calculation.  At least one of the SmSurfaceCache
    m_dChordHeightTolerance or m_dAngTolRad values must be NonZero.
***********************************************************************/
SmStatus SmTessSrfCache::BuildTree
 (SmMemBlockMgr *)                 // in : only declared to be a virtual function of SmSurfaceCache::BuildTree
{
  SmFace           *pCacheFace = this->GetFace();
  SmSurface        *pSurface   = pCacheFace->GetSurface();
  SmBSplineSurface *pBSS       = SM_CAST_PTR(SmBSplineSurface,pSurface); // okay if pBSS is NULL.
  SM_ASSERT(pSurface == GetSurface()) ;

#ifdef SM_DEBUG_CODE
  SmBoolean bDebugMe = FALSE;
#endif

  // prevent bad recursion back into BuildTree
  SmTemporaryChangeValue<SmBoolean> sTCV2(m_bFaceContainmentDone, 2);

  // Build the m_pTree subdivision tree of pSurface
      // note: calls either SmTessSrfCache::BuildTreeWithSubdivision() or
  if(!m_pTree)
    {
      //                    SmSurfaceCache::BuildTree()
      //              which differ in the initial set of subdivision nodes.
      //        - BuildTreeWithSubdivision() calls SubdivideToTolerances()
      //              with one node encompassing the whole surface
      //        - BuildTree() calls SubdivideToTolerances()
      //              with an array of nodes encompassing pieces of the whole surface
      if(pBSS)
        {
          // creates one single starter node for entire BSpline Surface -
          // Then subdivides that until pieces pass the tessellation tests
          //    Every TreeNode->m_eAuxDataType == SM_AD_AUX_DATA where m_pData is type SmBezierAux2d.
          SER(BuildTreeWithSubdivision(pBSS)) ;
        }
      else // surface is not a BSplineSurface
        {
          // creates a starter array of approximate degree 3x3 bezier patches -
          //    with a call to sm_CreateApproximateBezierPatchBlock(),
          //      one for every every non-zero surface span up until the span count gets > 5
          //      then build 5 evenly spaced degree 3x3 bezier surface approximations to the real surface.
          // Then subdivides those until pieces pass the tessellation tests.
          //    Every 1st generation and subsequent subdivided TreeNode->m_eAuxDataType == SM_AD_BEZIER_SURFACE
          //       where m_pData is type SmBezierPatch.
          //    Every ancester to the 1st generation TreeNode->m_eAuxDataType == SM_AD_BEZIER_SURFACE_HEAD
          //       where m_pData is type SmBezierAux2d.
          SmSurfaceCache::BuildTree(NULL) ;
        }

    } // end need to build Subdivision Tree branch

  // Phase1 of tessellation creates pSC->mTS_pPolyBrep, a single faced SmPolyBrep
  // for this surface's owner face containing a piecewise
  // linear approximation of connected linear
  // SmPolyEdges for every loop boundary.

  // quit if this surface's owner face has not yet gone through phase1 of tessellation.
  if (!mTS_pPolyBrep)
    {
#ifdef SM_DEBUG_CODE
      if(bDebugMe)
        {
          SM_ASSERT_VALID(m_pTree) ;
        }
#endif // SM_DEBUG_CODE
      return SM_SUCCESS;
    }

  // arrive here when the boundary edges of a Phase1
  // tessellation PolyBrep needs to add its
  // uvTrimCurves to the Subdivision tree

  // Get 1st polyFace and its PolyEdges. Phase1 PolyBreps have only 1 face.
  SmPolyFace *pPolyFace = mTS_pPolyBrep->GetFirstPolyFace(); NER(pPolyFace);
  SM_ASSERT_MSG(pPolyFace->m_pOriginalFace == pCacheFace, _T("gwc: test of SmTessSrfCache::BuildTree face pointer logic")) ;

  SM_PTR_ARRAY(sPolyEdges, SmPolyEdge, 256) ; // SmTArray<SmPolyEdge *>
  pPolyFace->GetPolyEdges(sPolyEdges);

  // init curve list memory manager
  m_sPolyEdgeListMgr.Initialize(ALIGN_SIZE(sizeof(SmPolyEdgeList)),30);

#ifdef SM_DEBUG_CODE
  if (bDebugMe)
    {
      SM_DUMP(m_pTree) ;
      SM_DUMP(this) ;  // all leaf nodes which are on the boundary should have some vertices or edges
      SM_DUMP(pCacheFace);
      SmTArray<SmTreeNode*> sNodes ;
      m_pTree->GetAllTreeNodes(sNodes) ;

      smgfx_Erase();
      smgfx_SetLook(2,1, 1,0,0); pCacheFace->Draw(SM_DM_WIREFRAME); sm_GraphicsLoop();
      smgfx_SetLook(1,1, 0,0,0); pCacheFace->GetSurface()->DrawUV(5,5); sm_GraphicsLoop();
      smgfx_SetLook(1,3, .4,.4,.4); this->DrawSubdivision2D(TRUE); sm_GraphicsLoop();
      smgfx_SetLook(1,3, .4,.4,.4); this->DrawSubdivision3D(); sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

  // Implant every PolyEdge into the subdivision tree - without subdividing any tree nodes
    {

      // prevent undesired recursion to BuildTree - turn off m_bPointTestEnabled
      // This will allow future GetTree() calls to return a Tree with or without
      //   implanted UVTrimCurves
      SmTemporaryChangeValue<SmBoolean> sTCV1(m_bPointTestEnabled, FALSE);

      for (ULONG ii=0; ii<sPolyEdges.GetSize(); ii++)
        {
          SmPolyEdge * pPolyEdge = sPolyEdges[ii];

          // Add pPolyEdge pointer in leaf TreeNode's m_sPolyEdgeList
          // for every leaf TreeNode whose m_sUVDomain is intersected by the segment
          SER(ImplantUVSegment(pPolyEdge));
        }
    } // end scope for implanting every PolyEdge UVTrimCurve into the TreeNodes's m_sPolyEdgeList
    
#ifdef SM_DEBUG_CODE
  if (bDebugMe)
    {
      SM_DUMP_AND_ASSERT_VALID(m_pTree) ;
      SM_DUMP_AND_ASSERT_VALID(this) ;  // all leaf nodes which are on the boundary should have some vertices or edges
      SM_DUMP_AND_ASSERT_VALID(pCacheFace);
      SmTArray<SmTreeNode*> sNodes ;
      m_pTree->GetAllTreeNodes(sNodes) ;

      smgfx_Erase();
      smgfx_SetLook(2,1, 1,0,0); pCacheFace->Draw(SM_DM_WIREFRAME); sm_GraphicsLoop();
      smgfx_SetLook(1,1, 0,0,0); pCacheFace->GetSurface()->DrawUV(5,5); sm_GraphicsLoop();
      smgfx_SetLook(1,3, .4,.4,.4); this->DrawSubdivision2D(TRUE); sm_GraphicsLoop();
      smgfx_SetLook(1,3, .4,.4,.4); this->DrawSubdivision3D(); sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

    // Clean up NURBS and Bezier pointers that become stale when m_sSubdisionSurface and sBezierBlock are freed
    {
        // need to
        //   Free all m_sSubdivisionSurface memory.
        //   Set SmBezierAux2d::mBA_pSurface pointers to source surface.
        //   Set SmBezierPatch::m_pBezier pointers to NULL - they are construction only data.

        // make m_sSubdivisionSurfaces temporary so it deletes itself when leaving this scope.
        SmObjsDelete<SmSurface*> sClean(&m_sSubdivisionSurfaces);

        ULONG lBezCount = m_sBezMgr.GetNumActiveElements();
        ULONG lAuxCount = m_sAuxMgr.GetNumActiveElements();
        for (ULONG ii = 0; ii < lBezCount; ii++)
        {
            SmBezierPatch* pBezPatch = (SmBezierPatch*)m_sBezMgr.GetAt(ii);
            pBezPatch->mBA_pSurface = pSurface;
            SM_ASSERT(pBezPatch->m_pBezier == NULL);
        }
        for (ULONG ii = 0; ii < lAuxCount; ii++)
        {
            SmBezierAux2d* pAux = (SmBezierAux2d*)m_sAuxMgr.GetAt(ii);
            pAux->mBA_pSurface = pSurface;
        }
    }
  m_sSubdivisionSurfaces.ReSet();

  // set Cache state bits
  m_bProcessBoundaryCurves  = FALSE;
  m_bPointTestEnabled       = TRUE;
  m_bHaveTSurfaceCache      = TRUE;

  #ifdef SM_USE_TESS_REFINE_MESH
  {
      /*
      // get all nodes in tree
      SmTArray<SmTreeNode*> sNodesStack;
      m_pTree->GetAllTreeNodes(sNodesStack);
      std::atomic<ULONG> lMaxCacheSize(0);

      // while we have nodes on the stack
      while (sNodesStack.GetSize() > 0)
      {
          // Get the node and the node aux data, remove node from stack
          SmTreeNode* pNode;
          sNodesStack.Pop(pNode);

          SmBezierAux2d* pAux = (SmBezierAux2d*)pNode->m_pData;

          // Test to see if the node is a leaf on the boundary of the face
          if (pNode->m_pChild1 == NULL && pAux->m_eNodeClass == SM_NC_ON_BOUNDARY)
          {
              // Get edges in current node
              SmTArray<SmPolyEdgeList*> sPolyEdgeListArray;
              pAux->m_sPolyEdgeList.GetAllNodes(sPolyEdgeListArray);
              if (sPolyEdgeListArray.GetSize() > lMaxCacheSize)
              {
                  lMaxCacheSize = sPolyEdgeListArray.GetSize();
              }
          }
      }

      //if (lMaxCacheSize <= 8)
      //{
      //    lMaxCacheSize = 8;
      //}
      */
      
     
      SmTemporaryChangeValue<SmBoolean> sTCV3(m_bPointTestEnabled, 0);
      //Parameter choice
      ULONG lEdgeCountThreshold = 6; // probably 4, 3 works better for some surfaces
      //std::atomic<ULONG> lEdgeCountThreshold = 3 + (lMaxCacheSize / 32);
      ULONG lEdgeThresholdGap = 1;
      //std::atomic<ULONG> lEdgeThresholdGap = 1 + (lMaxCacheSize / 32); // probably 1
      double dNeighborThreshold = 4; // probably 2,
      //std::atomic<double> dNeighborThreshold = 2 + (lMaxCacheSize / 8); 
      double dNeighborThresholdGrowth = 4; // probably 4
      //std::atomic<double> dNeighborThresholdGrowth = 4 + (lMaxCacheSize / 16);
      SmBoolean bIgnoreBoundaryNeighbors = FALSE;

      RefineMeshBoundaries(lEdgeCountThreshold,         // in : number of distinct PolyEdges in a 
                                                        // bounddary nodebefore subdivision 
                           lEdgeThresholdGap,           // in : disparity in PolyEdge counts between subdomains
                                                        // before split to quads. Bigger number, more splits
                           dNeighborThreshold,          // in : number of neighbors before splitting neighbor
                                                        // Smaller number, more splits in interior
                           dNeighborThresholdGrowth,    // in : rate of change of splits in neighbors
                                                        // Smaller number, splits extend farther into the face
                           bIgnoreBoundaryNeighbors);   // in : flag to determine whether boundary neighbors are
                                                        // considered for subdivision in neighbors step
  }
  #endif

#ifdef SM_DEBUG_CODE
  if (bDebugMe)
    {
      SM_DUMP_AND_ASSERT_VALID(m_pTree) ;
      SM_DUMP_AND_ASSERT_VALID(this) ;  // all leaf nodes which are on the boundary should have some vertices or edges
      SM_DUMP_AND_ASSERT_VALID(pCacheFace);
      SmTArray<SmTreeNode*> sNodes ;
      m_pTree->GetAllTreeNodes(sNodes) ;

      smgfx_Erase();
      smgfx_SetLook(2,1, 1,0,0); pCacheFace->Draw(SM_DM_WIREFRAME); sm_GraphicsLoop();
      smgfx_SetLook(1,1, 0,0,0); pCacheFace->GetSurface()->DrawUV(5,5); sm_GraphicsLoop();
      smgfx_SetLook(1,3, .4,.4,.4); this->DrawSubdivision2D(TRUE); sm_GraphicsLoop();
      smgfx_SetLook(1,3, .4,.4,.4); this->DrawSubdivision3D(); sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

  // set tree node containment.
  // Mark each TreeNode->m_pData->m_eNodeClass one of SM_NC_INSIDE
  //                                                  SM_NC_OUTSIDE
  //                                                  SM_NC_ON_BOUNDARY
  //
  //   This used to be based upon 2d raycasting but now uses loop containment.
  //   As such it can be called with m_bPointTestEnabled on or off. It must be
  //   called after all face edges have UVTrimCurves.
  SER(MarkFaceContainment());
  sTCV2.Clear() ;
  m_bFaceContainmentDone = TRUE;

#ifdef SM_DEBUG_CODE
  if (bDebugMe)
    {
      SM_DUMP_AND_ASSERT_VALID(m_pTree) ;
      SM_DUMP_AND_ASSERT_VALID(this) ;  // all leaf nodes which are on the boundary should have some vertices or edges
      SM_DUMP_AND_ASSERT_VALID(pCacheFace);
      SmTArray<SmTreeNode*> sNodes ;
      m_pTree->GetAllTreeNodes(sNodes) ;

      smgfx_Erase();
      smgfx_SetLook(2,1, 1,0,0); pCacheFace->Draw(SM_DM_WIREFRAME); sm_GraphicsLoop();
      smgfx_SetLook(1,1, 0,0,0); pCacheFace->GetSurface()->DrawUV(5,5); sm_GraphicsLoop();
      smgfx_SetLook(1,3, .4,.4,.4); this->DrawSubdivision2D(TRUE); sm_GraphicsLoop();
      smgfx_SetLook(1,3, .4,.4,.4); this->DrawSubdivision3D(); sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

  // all done
  return SM_SUCCESS;

} // end SmTessSrfCache::BuildTree

/*******************************************************************/ /**
 PURPOSE: Modifies the surface subdivision tree which drives mesh
    construction to improve tessellation quality. Subdivides boundary nodes
    to better match PolyEdge length. Subdivides nodes adjacent to boundary
    nodes when a sufficient number of neighboring nodes have been subdivided.
    These divisions are propogated into the interior of faces in such as way as
    to taper mesh density to discourage fans.

 NOTES:
   1. Given a boundary leaf which contain more than lEdgeCountThreshold
      polyedges, we subdivide the leaf.
   2. Classify the polyedges in the leaf to interior quads, we use the
      distribution of leaves to determine the split direction. Options
      are split in U, split in V or split in both (to four quads).
      a. If precisely one of the subquads of the node contains no
         PolyEdges OR the approximate distribution of PolyEdges between left 
         and right AND between top and bottom is approximately uniform,
         (absolute value of differences in counts is within
         lEdgeThresholdGap), then we split to subquads (split in U,
         split children in V) .
      b. Else we split orthogonal to the direction of the greater difference
         in PolyEdge distribution. Ex: The difference in PolyEdge counts
         between the left half and the right half is 5, the difference in counts
         between top and bottom is 1. We split in V. This has the effect of
         ensuring that PolyEdge density in boundary nodes decreases strictly
         monotonically with a minimum of splits.
   3. Split nodes, adding boundary children to stack. We label children as m_pData
      -> m_bIsTess = TRUE. This allows for filtering out these nodes when calling
      CheckAgainstSurfaceCache(), since these nodes are not derived from the geometric
      information that method is testing against.
   4. When a boundary node is split, we iterate through the neighbors of the node. We
      are going to potentially split neighbors and then neighbors of neighbors, etc,
      testing each generation of nodes against an increasing threshold for the number
      of adjacent nodes.
      a. For each neighbor, we count the number of adjacent nodes to left/right and
         top/bottom.
         i.   When both counts are greater than dNeighborThreshold, we split
              the node to subquads. (In U, children in V)
         ii.  When one of the left/right or top/bottom counts is greater than
              dNeighborThreshold, we split once. If left/right, then split
              in V, if top/bottom then in U. This ensures that the number of
              adjacent nodes in children decreases strictly monotonically.
      b. Split nodes, add neighbors of children to the stack for the next generation.
         label children as m_pData -> m_bIsTess = TRUE. This allows for
         filtering these ndoes out when calling CheckAgainstSurfaceCache() since
         these nodes are not derived from the geometric information that method is
         testing against.
      c. When we have exhausted the intitial set of neighbors, we increase
         dNeighborThresholdParam by dNeighborThresholdGrowth and start over, quit
         when we do not split any neighbors.

 ***********************************************************************/
SmStatus SmTessSrfCache::RefineMeshBoundaries
  (ULONG        lEdgeCountThreshold,        // in : number of PolyEdges in node
                                            //      before split   
   ULONG        lEdgeThresholdGap,          // in : threshold for gap in PolyEdge distributions
                                            //      between halves of boundary node
   double       dNeighborThreshold,         // in : threshold for subdivision of neighbors
                                            //      to divided boundary nodes               
   double       dNeighborThresholdGrowth,   // in : growth rate for threshold for subdivision 
                                            //      of neighbors to divided boundary nodes 
   SmBoolean    bIgnoreBoundaryNeighbors)   // in : Determines whether we reevaluate for subdivision boundary nodes
                                            //      which are neighbors of subdivided interior nodes.
{
    // get all nodes in tree
    SmTArray<SmTreeNode*> sNodesStack;
    m_pTree->GetAllTreeNodes(sNodesStack);

    // while we have nodes on the stack
    while (sNodesStack.GetSize() > 0)
    {
        // Get the node and the node aux data, remove node from stack
        SmTreeNode* pNode;
        sNodesStack.Pop(pNode);

        SmBezierAux2d* pAux = (SmBezierAux2d*)pNode->m_pData;

        // Test to see if the node is a leaf on the boundary of the face
        if (pNode->m_pChild1 == NULL && pAux->m_eNodeClass == SM_NC_ON_BOUNDARY)
        {
            // Get edges in current node
            SmTArray<SmPolyEdge*> sPolyEdgeArray;
            SmTArray<SmPolyEdgeList*> sPolyEdgeListArrayDirty;
            pAux->m_sPolyEdgeList.GetAllNodes(sPolyEdgeListArrayDirty);

            // remove edges which are either duplicated in the opposite direction, from seams
            // or are degenerate
            while (sPolyEdgeListArrayDirty.GetSize() > 0)
            {
                SmPolyEdgeList* pNodePolyEdgeList;
                sPolyEdgeListArrayDirty.Pop(pNodePolyEdgeList);
                SmPolyEdge* pEdge = pNodePolyEdgeList->m_pPolyEdge;
                SmPolyEdge* pEdgeReverse = pEdge->GetSymmetricPolyEdge();
                if (sPolyEdgeArray.IsIn(pEdge) || sPolyEdgeArray.IsIn(pEdgeReverse) || pEdge->IsDegenerate())
                {
                    continue;
                }
                else if ((pEdge->GetEdgeuse() != NULL) && (pEdge->GetEdgeuse()->GetEdge()->IsSeam()))
                {
                    continue;
                }
                else
                {
                    sPolyEdgeArray.Add(pEdge);
                }
            }

            // If we have too many edges in this node...
            if (sPolyEdgeArray.GetSize() > lEdgeCountThreshold)
            {
                //.. then we are going to split. We will classify edges by
                // subdomains to determine split direction
                SmExtent2d* pNodeUVDomain = &(pAux->m_sUVDomain);

                // build subdomains
                double dNodeMaxUTemp = pNodeUVDomain->GetUMax();
                double dNodeMinUTemp = pNodeUVDomain->GetUMin();
                double dNodeMaxVTemp = pNodeUVDomain->GetVMax();
                double dNodeMinVTemp = pNodeUVDomain->GetVMin();

                SmVector2d sUVSize = m_sUVDomain.GetSize();
                // safe guard against subdividing a node endlessly
                if (smos_Fabs(dNodeMaxUTemp - dNodeMinUTemp) < (sUVSize.x / 1000.0) &&
                    smos_Fabs(dNodeMaxVTemp - dNodeMinVTemp) < (sUVSize.y / 1000.0))
                {
                    SM_ASSERT_MSG(FALSE, _T("Node over subdivided in RefineBoundaryMesh()"));
                    continue;
                }

                // midpoints for constructing subdomains
                double dNodeMidU = (dNodeMinUTemp + dNodeMaxUTemp) / 2.0;
                double dNodeMidV = (dNodeMinVTemp + dNodeMaxVTemp) / 2.0;

                double dNodeMaxU = ((dNodeMaxUTemp - dNodeMidU) * 0.99) + dNodeMidU;
                double dNodeMinU = dNodeMidU - ((dNodeMidU - dNodeMinUTemp) * 0.99);
                double dNodeMaxV = ((dNodeMaxVTemp - dNodeMidV) * 0.99) + dNodeMidV;
                double dNodeMinV = dNodeMidV - ((dNodeMidV - dNodeMinVTemp) * 0.99);

                // set up subdomains
                SmExtent2d sNodeUVDomainTopLeft;
                SmExtent2d sNodeUVDomainTopRight;
                SmExtent2d sNodeUVDomainBottomLeft;
                SmExtent2d sNodeUVDomainBottomRight;

                sNodeUVDomainTopLeft.SetMinMax(dNodeMinU, dNodeMidV, dNodeMidU, dNodeMaxV);
                sNodeUVDomainTopRight.SetMinMax(dNodeMidU, dNodeMidV, dNodeMaxU, dNodeMaxV);
                sNodeUVDomainBottomLeft.SetMinMax(dNodeMinU, dNodeMinV, dNodeMidU, dNodeMidV);
                sNodeUVDomainBottomRight.SetMinMax(dNodeMidU, dNodeMinV, dNodeMaxU, dNodeMidV);

                // We are going classify the curve cache inside the subdomains,
                // this will inform the split direction.

                ULONG lEdgeCountTopLeft = 0;
                ULONG lEdgeCountTopRight = 0;
                ULONG lEdgeCountBottomLeft = 0;
                ULONG lEdgeCountBottomRight = 0;

                // for each edge in the cache for this node...
                for (ULONG nn = 0; nn < sPolyEdgeArray.GetSize(); nn++)
                {
                    SmPolyEdge* pEdge = sPolyEdgeArray[nn];
                    // Get 2d information to use to test intersection
                    SmPoint3d sStartPoint3d = pEdge->GetStartPoint();
                    SmPoint2d sStartPoint;
                    sStartPoint.x = sStartPoint3d.x;
                    sStartPoint.y = sStartPoint3d.y;
                    SmVector3d sVec3d = pEdge->GetVec();
                    SmVector2d sVec;
                    sVec.x = sVec3d.x;
                    sVec.y = sVec3d.y;
                    SmExtent1d sUnit(0., 1.);
                    SmExtent1d sIntersection;

                    // locals for intersection test left
                    SmBoolean bFoundIntervalTopLeft;
                    SmExtent1d rIntervalTopLeft;

                    // Test intersection of edge and domain left
                    sNodeUVDomainTopLeft.IntersectWithInfiniteLine(
                        sStartPoint, sVec, bFoundIntervalTopLeft, rIntervalTopLeft);

                    // If we get an intersection
                    if (bFoundIntervalTopLeft && rIntervalTopLeft.Intersect(sUnit, sIntersection) == SM_SUCCESS)
                    {
                        // increment count
                        lEdgeCountTopLeft++;
                    }
                    // locals for intersection test  right
                    SmBoolean bFoundIntervalTopRight;
                    SmExtent1d rIntervalTopRight;

                    // Test intersection of edge and domain right
                    sNodeUVDomainTopRight.IntersectWithInfiniteLine(
                        sStartPoint, sVec, bFoundIntervalTopRight, rIntervalTopRight);

                    // If we get an intersection
                    if (bFoundIntervalTopRight && rIntervalTopRight.Intersect(sUnit, sIntersection) == SM_SUCCESS)
                    {
                        // increment count
                        lEdgeCountTopRight++;
                    }
                    // locals for intersection test Bottom
                    SmBoolean bFoundIntervalBottomLeft;
                    SmExtent1d rIntervalBottomLeft;

                    // Test intersection of edge and domain bottom
                    sNodeUVDomainBottomLeft.IntersectWithInfiniteLine(
                        sStartPoint, sVec, bFoundIntervalBottomLeft, rIntervalBottomLeft);

                    // If we get an intersection
                    if (bFoundIntervalBottomLeft && rIntervalBottomLeft.Intersect(sUnit, sIntersection) == SM_SUCCESS)
                    {
                        // increment count
                        lEdgeCountBottomLeft++;
                    }
                    // locals for intersection test Top
                    SmBoolean bFoundIntervalBottomRight;
                    SmExtent1d rIntervalBottomRight;

                    // Test intersection of edge and domain bottom right
                    sNodeUVDomainBottomRight.IntersectWithInfiniteLine(
                        sStartPoint, sVec, bFoundIntervalBottomRight, rIntervalBottomRight);

                    // If we get an intersection
                    if (bFoundIntervalBottomRight && rIntervalBottomRight.Intersect(sUnit, sIntersection) == SM_SUCCESS)
                    {
                        // increment count
                        lEdgeCountBottomRight++;
                    } // end classification of edges into subdomains.
                }

                // Arrive here, we know we are going to split the node.
                // 
                // We can either split the node in U or V or both.
                // 
                // We use the classifications of the edges in the cache
                // to determine split direction.

                SmSurfParamType eSubdivideDirection = SM_SP_UNKNOWN;
                SmBoolean bSubdivideToQuads = FALSE;

                // note the these are approximate, since an edge might be incident with more than one
                // subdomain. 
                ULONG lEdgeCountLeftRightDifference =
                    smos_Iabs((lEdgeCountTopLeft + lEdgeCountBottomLeft) - (lEdgeCountTopRight + lEdgeCountBottomRight));
                ULONG lEdgeCountTopBottomDifference =
                    smos_Iabs((lEdgeCountTopLeft + lEdgeCountTopRight) - (lEdgeCountBottomLeft + lEdgeCountBottomRight));

                // for geometric tests for node construction. This follows the behavior used in initial tree construction
                SmExtent2d sDomain;
                SmPoint3d sU0V0, sU1V0, sU1V1, sU0V1;

                // node property locals
                SmPoint2d sAspectRatio, sMaxSideLength3D, sMinSideLength3D;

                // Get Surface 3d corners
                SER(GetCorners(NULL, pAux->m_sUVDomain, sU0V0, sU1V0, sU1V1, sU0V1, sDomain));

                // compute from 3d corner set - cheap
                SER(ComputeSizeTolerances(sU0V0, // in : 3d corner of Surface(u0, v0) of target quad
                                          sU1V0, // in : 3d corner of Surface(u1, v0) of target quad
                                          sU1V1, // in : 3d corner of Surface(u1, v1) of target quad
                                          sU0V1, // in : 3d corner of Surface(u0, v1) of target quad
                                          sAspectRatio, // out: [dMaxU/dMaxV, dMaxV/dMaxU] in 3d, except Max <
                                                        // dScaledZero, then 0.5
                                          sMaxSideLength3D, // out: [dMaxU, dMaxV] in 3d,
                                          sMinSideLength3D)); // out: [dMinU, dMinV] in 3d, except Min < dScaledZero,
                                                              // then Max

                // We want to subdivide to follow the contour of the trim curves. If one of the subdomains it empty,
                // then we are something of a bend in the curve, we want to go to quads to follow the bend. Note
                // that this guarantees that one of the children will not be further subdivided.
                //
                // If we have a somewhat uniform distribution of PolyEdges in the node, resolution of the mesh is too low,
                // we we want to go to quads.
                //
                // Otherwise, we subdivide to reduce the average number of PolyEdges by splitting nodes orthogonal
                // to the direction of the greatest disparity in PolyEdge counts.
                //
                // Example: For the node of the left below, the disparity in top/bottom is |4-4|=0.
                //          The dispartity in left right is |8-0|=8. Left/right is bigger, so we split in V
                // 
                //       *----------------------*            *----------------------*
                //       |   !                  |            |   !                  |
                //       |   !     interior     |            |   !     interior     |
                //       |   !     of face      |            |   !     of face      |
                //       |   !                  |            *___!__________________*
                //       |   !                  |    --->    |   !                  |
                //       |   !                  |            |   !                  |
                //       |    \                 |            |    \                 |
                //       |     \                |            |     \                |
                //       *----------------------*            *----------------------*
                //
                //          Edge density decreases without adding unecessary nodes.

                if (((lEdgeCountTopLeft == 0) && (lEdgeCountTopRight * lEdgeCountBottomLeft * lEdgeCountBottomRight != 0)) ||
                    ((lEdgeCountTopRight == 0) && (lEdgeCountTopLeft * lEdgeCountBottomLeft * lEdgeCountBottomRight != 0)) ||
                    ((lEdgeCountBottomLeft == 0) && (lEdgeCountTopRight * lEdgeCountTopLeft * lEdgeCountBottomRight != 0)) ||
                    ((lEdgeCountBottomRight == 0) && (lEdgeCountTopRight * lEdgeCountBottomLeft * lEdgeCountTopLeft != 0)) ||
                    ((ULONG)smos_Iabs(lEdgeCountLeftRightDifference - lEdgeCountTopBottomDifference) <= lEdgeThresholdGap))
                {
                    bSubdivideToQuads = TRUE;
                }
                else
                {
                    bSubdivideToQuads = FALSE;
                    eSubdivideDirection =
                        (lEdgeCountLeftRightDifference > (lEdgeCountTopBottomDifference + lEdgeThresholdGap)) ?
                            SM_SP_V :
                            SM_SP_U; 
                }

                // setting minimum side length ratio, either parameter or mininum value of 0.001
                double dMinSLR = (m_dMinimumSideLengthRatioUV != 0.0) ? m_dMinimumSideLengthRatioUV : 0.001;

                // USide or VSide too small When being split
                if (((eSubdivideDirection == SM_SP_U || bSubdivideToQuads) && sUVSize.x < (sUVSize.x * dMinSLR)) ||
                    ((eSubdivideDirection == SM_SP_V || bSubdivideToQuads) && sUVSize.y < (sUVSize.y * dMinSLR)))
                {
                    continue;
                }

                // if we can't make the split we want, we stop considering this node
                if (m_dMinimumSideLength3D > 0.0 && ((sMinSideLength3D.x <= m_dMinimumSideLength3D &&
                                                         (eSubdivideDirection == SM_SP_U || bSubdivideToQuads)) ||
                                                     ((sMinSideLength3D.y <= m_dMinimumSideLength3D) &&
                                                         (eSubdivideDirection == SM_SP_V || bSubdivideToQuads))))
                {
                    continue;
                }
                // same, if we can't split, we stop considering this node
                else if (m_dAspectRatio3D > 0.0 &&
                         (sAspectRatio.x > m_dAspectRatio3D || sAspectRatio.y > m_dAspectRatio3D))
                {
                    // bSubdivideToQuads = FALSE;
                    // eSubdivideDirection = (sMaxSideLength3D.x > sMaxSideLength3D.y) ? SM_SP_U : SM_SP_V;
                    continue;
                }

                // Get neighbors before splitting, we will test these for additional subdivisions
                SmTArray<SmTreeNode*> sNodeNeighborArray;
                pNode->GetNeighbors(sNodeNeighborArray);

                std::unordered_set<SmTreeNode*> sNodeNeighborSet;

                for (ULONG ii = 0; ii < sNodeNeighborArray.GetSize(); ii++)
                {
                    sNodeNeighborSet.insert(sNodeNeighborArray[ii]);
                }

                if (bSubdivideToQuads)
                {
                    // now we split, first in U
                    SubdivideNode(pNode, SM_SP_U, FALSE, NULL);

                    // then split first child in V
                    SmTreeNode* pNodeFirstSplitChild1 = pNode->m_pChild1;
                    SubdivideNode(pNodeFirstSplitChild1, SM_SP_V, FALSE, NULL);

                    SmTreeNode* pNodeSecondSplitChild1Child1 = pNodeFirstSplitChild1->m_pChild1;
                    SmTreeNode* pNodeSecondSplitChild1Child2 = pNodeFirstSplitChild1->m_pChild2;

                    // then split second child in V
                    SmTreeNode* pNodeFirstSplitChild2 = pNode->m_pChild2;
                    SubdivideNode(pNodeFirstSplitChild2, SM_SP_V, FALSE, NULL);

                    SmTreeNode* pNodeSecondSplitChild2Child1 = pNodeFirstSplitChild2->m_pChild1;
                    SmTreeNode* pNodeSecondSplitChild2Child2 = pNodeFirstSplitChild2->m_pChild2;

                    // get children aux data
                    SmBezierAux2d* pAuxC1C1 = (SmBezierAux2d*)pNodeSecondSplitChild1Child1->m_pData;
                    SmBezierAux2d* pAuxC1C2 = (SmBezierAux2d*)pNodeSecondSplitChild1Child2->m_pData;
                    SmBezierAux2d* pAuxC2C1 = (SmBezierAux2d*)pNodeSecondSplitChild2Child1->m_pData;
                    SmBezierAux2d* pAuxC2C2 = (SmBezierAux2d*)pNodeSecondSplitChild2Child2->m_pData;

                    // set these nodes as IsTessOnly so they do not influence edge subdivision in the corner
                    // cutting algorihm (impacts CheckSurfaceCache() otehrwise)
                    pAuxC1C1->m_bIsTessOnly = TRUE;
                    pAuxC1C2->m_bIsTessOnly = TRUE;
                    pAuxC2C1->m_bIsTessOnly = TRUE;
                    pAuxC2C2->m_bIsTessOnly = TRUE;

                    // add children to the stack
                    sNodesStack.Add(pNodeSecondSplitChild1Child1);
                    sNodesStack.Add(pNodeSecondSplitChild1Child2);
                    sNodesStack.Add(pNodeSecondSplitChild2Child1);
                    sNodesStack.Add(pNodeSecondSplitChild2Child2);
                }
                // only subdividing once
                else
                {
                    // subdivide node
                    SubdivideNode(pNode, eSubdivideDirection, FALSE, NULL);
                    SmTreeNode* pNodeFirstSplitChild1 = pNode->m_pChild1;
                    SmTreeNode* pNodeFirstSplitChild2 = pNode->m_pChild2;

                    // get and update children aux data, these nodes as TessOnly
                    SmBezierAux2d* pAuxC1 = (SmBezierAux2d*)pNodeFirstSplitChild1->m_pData;
                    SmBezierAux2d* pAuxC2 = (SmBezierAux2d*)pNodeFirstSplitChild2->m_pData;

                    pAuxC1->m_bIsTessOnly = TRUE;
                    pAuxC2->m_bIsTessOnly = TRUE;

                    // add children to the stack
                    sNodesStack.Add(pNodeFirstSplitChild1);
                    sNodesStack.Add(pNodeFirstSplitChild2);
                }

                // Arrive here, we have subdivided a boundary node.
                //
                // Next we want to taper the mesh density into the interior of the face by using subdivisions
                // along the boundary to drive some subdivision in the interior.
                // 
                // If we have too many nodes adjacent to an node neighboring a subdividing boundary node,
                // we increase the density of the mesh by subdividing in such a way as to reduce the number of nodes
                // adjacent to children.
                // 
                // We add children to the stack for the next generation of splits, during which we increase the number of
                // neighbors required to generate a split (stopping runaway subdivision into the interior).

                // local for the neighbor node threshold, we are going to modify it below
                double dNeighborThresholdModified = dNeighborThreshold;

                // while we have neighbors of a subdivided boundary node.
                //while (sNodeNeighborArray.GetSize() > 0)
                while (sNodeNeighborSet.size() > 0)
                {
                    // we are going to do this in "generations" neighbors of the subdivided boundary node, then
                    // neighbors of neighbors, etc. Temp array holds the nodes for the next generation.
                    ///SmTArray<SmTreeNode*> sTempNeighborNodeArray;
                    std::unordered_set<SmTreeNode*> sTempNeighborNodeSet;

                    // For each node in the current array of neighbors
                    //for (ULONG ii = 0; ii < sNodeNeighborArray.GetSize(); ii++)
                    for (std::unordered_set<SmTreeNode*>::iterator iter = sNodeNeighborSet.begin();
                        iter != sNodeNeighborSet.end(); iter++)
                    {
                        // get neighbor node
                        //SmTreeNode* pNodeNeighbor = sNodeNeighborArray[ii];
                        SmTreeNode* pNodeNeighbor = *iter;
                        SmBezierAux2d* pAuxNeighbor = (SmBezierAux2d*)pNodeNeighbor->m_pData;

                        // if, for some reason, this is not a leaf, or we have decided to not consider
                        // boundary nodes as neighbors, ignore the node
                        if (((pAuxNeighbor->m_eNodeClass == SM_NC_ON_BOUNDARY) && bIgnoreBoundaryNeighbors)
                            || pNodeNeighbor -> m_pChild1 != NULL)
                        {
                            continue;
                        }
                        // otherwise, we are going to test for potential subdivision
                        else
                        {
                            // count neighbors above/below this node
                            ULONG lTopBottomLeafCount = 0;
                            SmTArray<SmTreeNode*> sNodeNeighborNeighborsTopArray;
                            pNodeNeighbor->GetTopNeighbors(sNodeNeighborNeighborsTopArray);
                            lTopBottomLeafCount = sNodeNeighborNeighborsTopArray.GetSize();
                            SmTArray<SmTreeNode*> sNodeNeighborNeighborsBotArray;
                            pNodeNeighbor->GetBotNeighbors(sNodeNeighborNeighborsBotArray);
                            lTopBottomLeafCount += sNodeNeighborNeighborsBotArray.GetSize();

                            // count neighbors to left/right of this node
                            ULONG lLeftRightLeafCount = 0;
                            SmTArray<SmTreeNode*> sNodeNeighborNeighborsLeftArray;
                            pNodeNeighbor->GetLeftNeighbors(sNodeNeighborNeighborsLeftArray);
                            lLeftRightLeafCount = sNodeNeighborNeighborsLeftArray.GetSize();
                            SmTArray<SmTreeNode*> sNodeNeighborNeighborsRightArray;
                            pNodeNeighbor->GetRightNeighbors(sNodeNeighborNeighborsRightArray);
                            lLeftRightLeafCount += sNodeNeighborNeighborsRightArray.GetSize();

                            // for subdivision determination
                            SmBoolean bSubdivideToQuadsNeighbor = FALSE;
                            SmBoolean bSubdivideNeighbor = FALSE;
                            SmSurfParamType eSubdivideDirectionNeighbor = SM_SP_UNKNOWN;

                            // if both the top/bottom and left/right are larger than the threshold,
                            // subdivide to the quads
                            if (lLeftRightLeafCount > dNeighborThresholdModified &&
                                lTopBottomLeafCount > dNeighborThresholdModified)
                            {
                                bSubdivideNeighbor = TRUE;
                                bSubdivideToQuadsNeighbor = TRUE;
                            }
                            // else, if the left/right is too big, split in V
                            // this reduces the number of neighbors in children on left/right
                            else if (lLeftRightLeafCount > dNeighborThresholdModified)
                            {
                                bSubdivideNeighbor = TRUE;
                                bSubdivideToQuadsNeighbor = FALSE;
                                eSubdivideDirectionNeighbor = SM_SP_V;
                            }
                            // else, if the top/bottom is too big, split in U
                            // this reduces the number of neighbors in children on top/bottom
                            else if (lTopBottomLeafCount > dNeighborThresholdModified)
                            {
                                bSubdivideNeighbor = TRUE;
                                bSubdivideToQuadsNeighbor = FALSE;
                                eSubdivideDirectionNeighbor = SM_SP_U;
                            }

                            // testing the node for the geometric constraints for subdivision.
                            // this follows the treatment during ndoe subdivision used in initial tree constuction
                            SmExtent2d sDomainNeighbor;
                            SmPoint3d sU0V0Neighbor, sU1V0Neighbor, sU1V1Neighbor, sU0V1Neighbor;

                            // node property locals
                            SmPoint2d sAspectRatioNeighbor, sMaxSideLength3DNeighbor, sMinSideLength3DNeighbor;

                            // Get Surface 3d corners
                            SER(GetCorners(NULL, pAuxNeighbor->m_sUVDomain, sU0V0Neighbor, sU1V0Neighbor, sU1V1Neighbor,
                                           sU0V1Neighbor, sDomainNeighbor));

                            // compute from 3d corner set - cheap
                            SER(ComputeSizeTolerances(sU0V0Neighbor, // in : 3d corner of Surface(u0, v0) of target
                                                                     // quad
                                                      sU1V0Neighbor, // in : 3d corner of Surface(u1, v0) of target
                                                                     // quad
                                                      sU1V1Neighbor, // in : 3d corner of Surface(u1, v1) of target
                                                                     // quad
                                                      sU0V1Neighbor, // in : 3d corner of Surface(u0, v1) of target
                                                                     // quad
                                                      sAspectRatioNeighbor, // out: [dMaxU/dMaxV, dMaxV/dMaxU] in 3d,
                                                                            // except Max <
                                                      // dScaledZero, then 0.5
                                                      sMaxSideLength3DNeighbor, // out: [dMaxU, dMaxV] in 3d,
                                                      sMinSideLength3DNeighbor)); // out: [dMinU, dMinV] in 3d, except
                                                                                  // Min
                                                                                  // <
                            // dScaledZero, then Max

                            SmVector2d sNodeSizeNeighbor = pAuxNeighbor->m_sUVDomain.GetSize();

                            // if we are splitting a node too small in UV, ignore it.
                            // Try to salvage a split to quads if we can.
                            if (((eSubdivideDirectionNeighbor == SM_SP_U || bSubdivideToQuadsNeighbor) &&
                                 sNodeSizeNeighbor.x < (sUVSize.x * dMinSLR)) ||
                                 ((eSubdivideDirectionNeighbor == SM_SP_V || bSubdivideToQuadsNeighbor) &&
                                 sNodeSizeNeighbor.y < (sUVSize.y * dMinSLR)))
                            {
                                if (bSubdivideToQuadsNeighbor && (sNodeSizeNeighbor.x < (sUVSize.x * dMinSLR)) &&
                                                                  !(sNodeSizeNeighbor.y < (sUVSize.y * dMinSLR)))
                                {
                                     bSubdivideToQuadsNeighbor = FALSE;
                                     eSubdivideDirectionNeighbor = SM_SP_V;
                                }
                                else if (bSubdivideToQuadsNeighbor && ((sNodeSizeNeighbor.y < (sUVSize.y * dMinSLR)) &&
                                             !(sNodeSizeNeighbor.x < (sUVSize.x * dMinSLR))))
                                {
                                     bSubdivideToQuadsNeighbor = FALSE;
                                     eSubdivideDirectionNeighbor = SM_SP_U;
                                }
                                else
                                {
                                     continue;
                                }
                            }

                            // if we are spliting a node too small in image space, ignore it.
                            // Try to salvage a split to quads if we can
                            if (m_dMinimumSideLength3D > 0.0 &&
                                ((sMinSideLength3DNeighbor.x <= m_dMinimumSideLength3D &&
                                      (eSubdivideDirectionNeighbor == SM_SP_U || bSubdivideToQuadsNeighbor)) ||
                                  ((sMinSideLength3DNeighbor.y <= m_dMinimumSideLength3D) &&
                                      (eSubdivideDirectionNeighbor == SM_SP_V || bSubdivideToQuadsNeighbor))))
                            {
                                if (bSubdivideToQuadsNeighbor && (sMinSideLength3DNeighbor.y <= m_dMinimumSideLength3D &&
                                        !(sMinSideLength3DNeighbor.x <= m_dMinimumSideLength3D)))
                                {
                                    bSubdivideToQuadsNeighbor = FALSE;
                                    eSubdivideDirectionNeighbor = SM_SP_U;
                                }
                                else if (bSubdivideToQuadsNeighbor && (sMinSideLength3DNeighbor.x <= m_dMinimumSideLength3D &&
                                             !(sMinSideLength3DNeighbor.y <= m_dMinimumSideLength3D)))
                                {
                                    bSubdivideToQuadsNeighbor = FALSE;
                                    eSubdivideDirectionNeighbor = SM_SP_V;
                                }
                                else
                                {
                                    continue;
                                }
                            }
                        

                            // if aspect ratio is bad, split to fix it.
                            if (m_dAspectRatio3D > 0.0 && (sAspectRatioNeighbor.x > m_dAspectRatio3D ||
                                                                sAspectRatioNeighbor.y > m_dAspectRatio3D))
                            {
                                bSubdivideNeighbor = TRUE;
                                bSubdivideToQuads = FALSE;
                                eSubdivideDirection = (sMaxSideLength3D.x > sMaxSideLength3D.y) ? SM_SP_U : SM_SP_V;
                            }

                            // if we are splitting this node
                            if (bSubdivideNeighbor)
                            {
                                // add neighbors of this node ot the array for the next generation
                                SmTArray<SmTreeNode*> sAddNodeNeighborArray;
                                pNodeNeighbor->GetNeighbors(sAddNodeNeighborArray);
                                //sTempNeighborNodeArray.AppendUnique(sAddNodeNeighborArray);
                                for (ULONG ii = 0; ii < sAddNodeNeighborArray.GetSize(); ii++)
                                {
                                    sTempNeighborNodeSet.insert(sAddNodeNeighborArray[ii]);
                                }

                                // if we are spliting to quads
                                if (bSubdivideToQuadsNeighbor)
                                {
                                    // split in V
                                    SubdivideNode(pNodeNeighbor, SM_SP_V, FALSE, NULL);
                                    SmTreeNode* pNodeNeighborChild1 = pNodeNeighbor->m_pChild1;
                                    SmTreeNode* pNodeNeighborChild2 = pNodeNeighbor->m_pChild2;

                                    // split child 1 in U
                                    SubdivideNode(pNodeNeighborChild1, SM_SP_U, FALSE, NULL);
                                    SmTreeNode* pNodeNeighborChild1Child1 = pNodeNeighborChild1->m_pChild1;
                                    SmTreeNode* pNodeNeighborChild1Child2 = pNodeNeighborChild1->m_pChild2;

                                    // get aux data for children of child 1
                                    SmBezierAux2d* pAuxNC1C1 = (SmBezierAux2d*)pNodeNeighborChild1Child1->m_pData;
                                    SmBezierAux2d* pAuxNC1C2 = (SmBezierAux2d*)pNodeNeighborChild1Child2->m_pData;

                                    // set as TessOnly
                                    pAuxNC1C1->m_bIsTessOnly = TRUE;
                                    pAuxNC1C2->m_bIsTessOnly = TRUE;

                                    // add neighbors of these children to the stack of neighbors
                                    //sTempNeighborNodeArray.Add(pNodeNeighborChild1Child1);
                                    //sTempNeighborNodeArray.Add(pNodeNeighborChild1Child2);
                                    sTempNeighborNodeSet.insert(pNodeNeighborChild1Child1);
                                    sTempNeighborNodeSet.insert(pNodeNeighborChild1Child2);

                                    // if a child of child 1 happens to be a boundary node, add it to the stack of boundary nodes
                                    SmBezierAux2d* pAuxNeighborC1C1 = (SmBezierAux2d*)pNodeNeighborChild1Child1->m_pData;
                                    if (pAuxNeighborC1C1->m_eNodeClass == SM_NC_ON_BOUNDARY)
                                    {
                                        sNodesStack.AddUnique(pNodeNeighborChild1Child1);
                                    }
                                    SmBezierAux2d* pAuxNeighborC1C2 = (SmBezierAux2d*)pNodeNeighborChild1Child2->m_pData;
                                    if (pAuxNeighborC1C2->m_eNodeClass == SM_NC_ON_BOUNDARY)
                                    {
                                        sNodesStack.AddUnique(pNodeNeighborChild1Child2);
                                    }

                                    // split child 2 in U
                                    SubdivideNode(pNodeNeighborChild2, SM_SP_U, FALSE, NULL);
                                    SmTreeNode* pNodeNeighborChild2Child1 = pNodeNeighborChild2->m_pChild1;
                                    SmTreeNode* pNodeNeighborChild2Child2 = pNodeNeighborChild2->m_pChild2;

                                    // get aux data for children of child 2
                                    SmBezierAux2d* pAuxNC2C1 = (SmBezierAux2d*)pNodeNeighborChild2Child1->m_pData;
                                    SmBezierAux2d* pAuxNC2C2 = (SmBezierAux2d*)pNodeNeighborChild2Child2->m_pData;

                                    // set as TessOnly
                                    pAuxNC2C1->m_bIsTessOnly = TRUE;
                                    pAuxNC2C2->m_bIsTessOnly = TRUE;

                                    // add these children to the stack of neighbors
                                    //sTempNeighborNodeArray.Add(pNodeNeighborChild2Child1);
                                    //sTempNeighborNodeArray.Add(pNodeNeighborChild2Child2);
                                    sTempNeighborNodeSet.insert(pNodeNeighborChild2Child1);
                                    sTempNeighborNodeSet.insert(pNodeNeighborChild2Child2);

                                    // if a child of child 2 happens to be a boundary node, add it to the stack of boundary nodes
                                    SmBezierAux2d* pAuxNeighborC2C1 = (SmBezierAux2d*)pNodeNeighborChild2Child1->m_pData;
                                    if (pAuxNeighborC2C1->m_eNodeClass == SM_NC_ON_BOUNDARY)
                                    {
                                        sNodesStack.AddUnique(pNodeNeighborChild2Child1);
                                    }
                                    SmBezierAux2d* pAuxNeighborC2C2 = (SmBezierAux2d*)pNodeNeighborChild2Child2->m_pData;
                                    if (pAuxNeighborC2C2->m_eNodeClass == SM_NC_ON_BOUNDARY)
                                    {
                                        sNodesStack.AddUnique(pNodeNeighborChild2Child2);
                                    }
                                }
                                // otherwise we are only spliting once
                                else
                                {
                                    // split node
                                    SubdivideNode(pNodeNeighbor, eSubdivideDirectionNeighbor, FALSE, NULL);
                                    SmTreeNode* pNodeNeighborChild1 = pNodeNeighbor->m_pChild1;
                                    SmTreeNode* pNodeNeighborChild2 = pNodeNeighbor->m_pChild2;

                                    // aux data for children
                                    SmBezierAux2d* pAuxNC1 = (SmBezierAux2d*)pNodeNeighborChild1->m_pData;
                                    SmBezierAux2d* pAuxNC2 = (SmBezierAux2d*)pNodeNeighborChild2->m_pData;

                                    //set as TessOnly
                                    pAuxNC1->m_bIsTessOnly = TRUE;
                                    pAuxNC2->m_bIsTessOnly = TRUE;

                                    // add these children to the stack of neighbors
                                    //sTempNeighborNodeArray.Add(pNodeNeighborChild1);
                                    //sTempNeighborNodeArray.Add(pNodeNeighborChild2);
                                    sTempNeighborNodeSet.insert(pNodeNeighborChild1);
                                    sTempNeighborNodeSet.insert(pNodeNeighborChild2);

                                    // if one of these children happens to be on the boundary, add it to the stack of boundary nodes
                                    SmBezierAux2d* pAuxNeighborChild1 = (SmBezierAux2d*)pNodeNeighborChild1->m_pData;
                                    if (pAuxNeighborChild1->m_eNodeClass == SM_NC_ON_BOUNDARY)
                                    {
                                        sNodesStack.AddUnique(pNodeNeighborChild1);
                                    }
                                    SmBezierAux2d* pAuxNeighborChild2 = (SmBezierAux2d*)pNodeNeighborChild2->m_pData;
                                    if (pAuxNeighborChild2->m_eNodeClass == SM_NC_ON_BOUNDARY)
                                    {
                                        sNodesStack.AddUnique(pNodeNeighborChild2);
                                    }
                                } // end determine which/if split in neighbors
                            } // end neighbor subdivision
                        } // end not boundary neighbors
                    } // end for all neighbors in last generation nodes on stack

                    // Done with this generation, replace last generation of nodes
                    // with the next and increment the threshold for subdivision
                    //sNodeNeighborArray = sTempNeighborNodeArray;
                    sNodeNeighborSet.clear();
                    sNodeNeighborSet = sTempNeighborNodeSet;
                    dNeighborThresholdModified += dNeighborThresholdGrowth;

                } // end while we still have neighbors to process
            } // end splitting boundary nodes
        } // end classification boundary/leaf
    } // end while nodes are on stack
    return SM_SUCCESS;
} // end SmTessSrfCache::RefineMeshBoundaries

/*******************************************************************//**
PURPOSE: Find the list of m_pTree->nodes in the subdivision surface tree
    whose UV domain intersects the input UVDomain and match the
    optional cpTypeToFind value.

NOTES: Only returns leaves
***********************************************************************/
SmStatus SmTessSrfCache::FindUVNodesOfClass
 (const SmExtent2d      & crUVDomain,    // in : culling domain
  const SmNodeClassType * cpTypeToFind,  // in : opt culling nodetype, NULL to ignore
  SmTArray<SmTreeNode*> & rNodes)        // out: List of m_pTree nodes intersecting crUVDomain
 const                                   //      and optionally of type cpTypeToFind
{
  // init output
  rNodes.ReSet();

  // locals
  SmTessSrfCache *pTCache = SM_CONST_CAST(SmTessSrfCache*,this);
  SmTree         *pTree   = pTCache->GetTree();
  SmTreeNode     *pTop    = pTree->GetTopNode();

  SM_PTR_ARRAY(sStack, SmTreeNode, 100) ;  // SmTArray<SmTreeNode *>

  // init the stack with Tree->TopNode
  sStack.Add(pTop);

  // while nodes are on the stack
  while (sStack.GetSize() > 0)
    {
      SmTreeNode *pNode = (SmTreeNode*)sStack.GetLast();
      sStack.RemoveLast();

      SM_ASSERT(   pNode->m_eAuxDataType == SM_AD_AUX_DATA
                || pNode->m_eAuxDataType == SM_AD_BEZIER_SURFACE
                || pNode->m_eAuxDataType == SM_AD_BEZIER_SURFACE_HEAD);

      SmBezierAux2d *pAux = (SmBezierAux2d*)pNode->m_pData;

      // skip nodes whose UVDomains don't intersect
      if (pAux->m_sUVDomain.AreDisjoint(crUVDomain))
        { continue ; }

      // Arrive here when UVDomains intersect

      // Put leaf nodes in output or children on the stack
      if (pNode->m_pChild1 != NULL)
        { // Non-leaf node
          NER(pNode->m_pChild2);
          sStack.Add(pNode->m_pChild2);
          sStack.Add(pNode->m_pChild1);
        }
      else
        { // Leaf node
          // when asked - skip nodes of wrong type
          if ( cpTypeToFind  &&  *cpTypeToFind != pAux->m_eNodeClass )
             { continue;   }
          rNodes.Add(pNode);
        }
    } // While nodes are in stack

  // all done
  return SM_SUCCESS;

} // end SmTessSrfCache::FindUVNodesOfClass

/*******************************************************************//**
PURPOSE:  Add pSegment pointer to TreeNode's m_sPolyEdgeList
         for every leaf TreeNode whose m_sUVDomain is intersected by the segment

NOTES: Yes, UVSegment can be added to multiple TreeNode leaves.
       No, UVSegments are not added to internal TreeNode parents.
***********************************************************************/
SmStatus SmTessSrfCache::ImplantUVSegment
 (SmPolyEdge * pSegment)         // in : target segment
{
  // locals
  SM_PTR_ARRAY(sNodes, SmTreeNode, 256) ; // SmTArray<SmTreeNode*>

  // get segment endpoints
  SmPoint2d  sStartPt(pSegment->GetStartPoint().x,pSegment->GetStartPoint().y );
  SmPoint2d  sEndPt  (pSegment->GetEndPoint().x,  pSegment->GetEndPoint().y );

  // Tolerance: get uv space tols. [B649]. GWC: Limit tolerance to a small fraction of the surface domain.
  // 
  // NOTE: When 3d tolerances projected to uv space get large (near poles or fast changes of parameterization), 
  // the number of SM_NC_ON_BOUNDARY classified SmTessSrfCache subdivision nodes increases leaving fewer and 
  // fewer nodes available to be classified as SM_NC_INSIDE. Only SM_NC_INSIDE nodes add polyBrep vertices to surfaces.  
  // When the tolerance becomes the same size as the surface all subdivision nodes for a face->Surface 
  // get classified to SM_NC_ON_BOUNDARY leaving no nodes to be classified as SM_NC_INSIDE.
  // The final PolyFace only gets vertices on the edges of the Brep being tessellated.  
  // Those PolyFaces do a poor job of approximating the shape of the Face->Surface and end up ignoring the 
  // specified tessellation accuracy parameters because those parameters set the size of the subdivision nodes.
  // A large tolerance here prevents those nodes from being labeled as SM_NC_INSIDE which prevents the resulting
  // polyface from introducing surface vertices to the accuracy scales embedded within the surface subdivision.

  SmTol3d sTol3d = pSegment->GetTolerance();
  const SmSurface *pSurf = m_pFace->GetSurface();
  SmPoint2d sMidUV( ( sStartPt + sEndPt ) / 2.0 );
  SmVector2d sDirUV( 1, 0 );
  SmTol2d sTolU = smos_Min( (double)SmTol::MapTo2d( sTol3d, sMidUV, sDirUV, *pSurf ), m_sUVDomain.XLength() / 100.0) ;
  sDirUV.Set( 0, 1 );
  // original line:  SmTol2d sTolV = SmTol::MapTo2d( sTol3d, sMidUV, sDirUV, *pSurf ) ;
  SmTol2d sTolV = smos_Min( (double)SmTol::MapTo2d( sTol3d, sMidUV, sDirUV, *pSurf ), m_sUVDomain.YLength() / 100.0) ;
  SmVector2d sTolExpansion( sTolU, sTolV );

  // Don't try intersecting the segment with the overall uv domain.
  // UV curves can wander slightly outside of the surface's domain,
  // which can really cause problems when intersecting a short segment
  // of such a curve.  Clamping should be sufficient.  [B67]
  // But we'll give a warning if it's too far out.
  SmExtent2d sExpDomain( m_sUVDomain );
  sExpDomain.ExpandAbsolute( sTolExpansion * 10.0 );
  SM_ASSERT_MSG( sExpDomain.ContainsPoint2d( sStartPt ),
    _T("SmTessSrfCache::ImplantUVSegment Warning: Tessellation segment outside of uv domain") );
  SM_ASSERT_MSG( sExpDomain.ContainsPoint2d( sEndPt ),
    _T("SmTessSrfCache::ImplantUVSegment Warning: Tessellation segment outside of uv domain") );

  sStartPt = m_sUVDomain.ClampPoint2d( sStartPt );
  sEndPt   = m_sUVDomain.ClampPoint2d( sEndPt   );

  SmVector2d sLineVec = sEndPt - sStartPt ;
  SmBoolean bIsDegenerate =    smos_Fabs(sLineVec.x) <= sTolU
                            && smos_Fabs(sLineVec.y) <= sTolV;

  // build trimmed segment bounding box - increased by tolerance
  SmExtent2d sSegUVExtent( sStartPt );
  sSegUVExtent.AddPoint2d( sEndPt );
  sSegUVExtent.ExpandAbsolute( sTolExpansion * 10.0); // gwc original line:  sSegUVExtent.ExpandAbsolute( sTolExpansion * 1000.0 );

  // get tree nodes that intersect segment BBox - returns leaf and non-leaf nodes including nodes that just touch.
  SER( FindUVNodes( sSegUVExtent, sNodes ));

#ifdef SM_DEBUG_CODE
  SmBoolean bDebugMe = FALSE;
  if (bDebugMe)
    {
      smgfx_Erase();
      smgfx_SetColor(1,0,0); pSegment->Draw(); sm_GraphicsLoop();
      smgfx_SetColor(1,0,1); pSegment->GetPolyFace()->Draw(); sm_GraphicsLoop();
      smgfx_SetColor(0,0,0); this->DrawSubdivision2D(TRUE);  sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

  // check state - segment should intersect some subdivision tree nodes
  if (sNodes.GetSize() == 0)
    {
      SER(SM_ERR);
      return SM_SUCCESS;
    }

  // More locals
  SmBoolean bFoundInterval;
  SmExtent1d sTrimIvl;

  // for every subdivision leaf/internal TreeNode within segment bounding box
  for (ULONG i=0; i<sNodes.GetSize(); i++)
    {
      // skip non-leaf node TreeNodes
      SmTreeNode *pNode = (SmTreeNode*)sNodes[i];
      SM_ASSERT(   pNode->m_eAuxDataType == SM_AD_AUX_DATA
                || pNode->m_eAuxDataType == SM_AD_BEZIER_SURFACE
                || pNode->m_eAuxDataType == SM_AD_BEZIER_SURFACE_HEAD);

      // skip non-leaf TreeNodes
      if (pNode->m_pChild1)
        { continue; }

      // node locals
      SmBezierAux2d *pAux    = (SmBezierAux2d*)pNode->m_pData;
      SmExtent2d     sDomain   ( pAux->m_sUVDomain );
      SmExtent2d     sDomainExp( pAux->m_sUVDomain );
      sDomainExp.ExpandAbsolute( sTolExpansion * 10.0 ); // gwc original line: sDomainExp.ExpandAbsolute( sTolExpansion * 1000.0 );  

#ifdef SM_DEBUG_CODE
      SmBoolean bDebugMe2 = FALSE;
      if (bDebugMe2)
        {
          smgfx_SetLook(1,4, 0,0,1); sDomain.Draw(GetContext()); sm_GraphicsLoop();
          sm_GraphicsLoop();
        }
#endif // SM_DEBUG_CODE

      // skip nodes that don't intersect the segment (both endPoints are out of Node->Domain and LineSeg does not XSect Domain)
      if(   !sDomainExp.ContainsPoint2d( sStartPt )
         && !sDomainExp.ContainsPoint2d( sEndPt   ))
        {
          if ( bIsDegenerate )
            { continue; }
          else
            {
              SER(sDomainExp.IntersectWithInfiniteLine(sStartPt,sLineVec,bFoundInterval,sTrimIvl));
              if ( !bFoundInterval ) { continue; }
            }
        }

      // Add segment to this node's m_sPolyEdgeList
      pAux->m_eNodeClass = SM_NC_ON_BOUNDARY;
      SmPolyEdgeList   *pPolyEdgeList = (SmPolyEdgeList*)m_sPolyEdgeListMgr.GetNewElement();
      pPolyEdgeList->m_pPolyEdge      = pSegment;
      // not used: pCL->m_pCurveNode  = NULL;
      pAux->m_sPolyEdgeList.Prepend(pPolyEdgeList);

    } // end iter every treeNode that intersects segment's bounding box

  // all done
  return SM_SUCCESS;

} // end SmTessSrfCache::ImplantUVSegment

/*******************************************************************//**
PURPOSE: Mark the containment of each node as inside/outside or on
    boundary of the face.

NOTES: 
***********************************************************************/
SmStatus SmTessSrfCache::MarkFaceContainment()
{
  // Propagate up ON_BOUNDARY flag and a mark indicating when
  // all subnodes have been classified.
  SmTree     * pTree = GetTree();
  SmTreeNode * pTop  = pTree->GetTopNode();
  SM_PTR_ARRAY(sStack, SmTreeNode, 100) ; // SmTArray<SmTreeNode*>
  sStack.Add(pTop);
  while (sStack.GetSize() > 0)
    {
      SmTreeNode *pNode = sStack.GetLast();
      sStack.RemoveLast();
      SM_ASSERT(   pNode->m_eAuxDataType == SM_AD_AUX_DATA
                || pNode->m_eAuxDataType == SM_AD_BEZIER_SURFACE
                || pNode->m_eAuxDataType == SM_AD_BEZIER_SURFACE_HEAD);
      SmBezierAux2d *pAux = (SmBezierAux2d*)pNode->m_pData;

      // If the node is a leaf, process the flags otherwise
      // put its children on the stack.
      if (pNode->m_pChild1 != NULL)
        {
          // Non-leaf node
          SM_ASSERT(pNode->m_pChild2 != NULL);
          sStack.Add(pNode->m_pChild1);
          sStack.Add(pNode->m_pChild2);
        }
      else // Leaf node
        {
          if (pAux->m_eNodeClass == SM_NC_ON_BOUNDARY)
            {
              pAux->m_bIsMarked = TRUE;
              // Propagate mark up tree if both children are marked
              // Propagate SM_NC_ON_BOUNDARY
              SmTreeNode *pParent = pNode->m_pParent;
              while (pParent != NULL)
                {
                  SmBezierAux2d *pParentAux = (SmBezierAux2d*)pParent->m_pData;
                  SmBezierAux2d *pAux1 = (SmBezierAux2d*)pParent->m_pChild1->m_pData;
                  SmBezierAux2d *pAux2 = (SmBezierAux2d*)pParent->m_pChild2->m_pData;
                  if(   pAux1->m_bIsMarked
                     && pAux2->m_bIsMarked)
                    {
                      pParentAux->m_bIsMarked = TRUE;
                    }
                  pParentAux->m_eNodeClass = SM_NC_ON_BOUNDARY;
                  pParent = pParent->m_pParent;
                }
            }
        }
    } // end while treenodes are on the stack


  SM_PTR_ARRAY(sAdjacentNodes, SmTreeNode, 100) ; // SmTArray<SmTreeNode*>

  sStack.ReSet();
  sStack.Add(pTop);
  SmTreeNode *pCurrNode = FindUnmarkedLeafNode(SM_NC_UNKNOWN,sStack);
  // Find an unmarked and SM_NC_UNKNOWN leaf node
  // If none found then we are done.
  // If found then find adjacent nodes using the bounding box
  // of the CurrNode.  If any of the adjacent nodes have a classification
  // of SM_NC_INSIDE or SM_NC_OUTSIDE then use that to set pCurrNode
  // classification.  Otherwise do a point based classification using rayfire.
  // Then propagate classification to all SM_NC_UNKNOWN nodes in adjacent
  // list.  With any luck we should only have to do one or two rayfires per
  // region being classified.  If we end up doing more then that, then we
  // can propagate to a second level.

  while (pCurrNode != NULL)
    {
      SmBezierAux2d * pAux            = (SmBezierAux2d*)pCurrNode->m_pData;
      SmNodeClassType eFoundNodeClass = SM_NC_UNKNOWN;
      SmFace        * pFoundFace      = NULL;

      // gather all nodes intersecting this node's uv domain.
      // gets leaf and non-leaf nodes and nodes that just touch.
      SER(FindUVNodes(pAux->m_sUVDomain,sAdjacentNodes));

      // for every adjacent node
      for (ULONG i=0; i<sAdjacentNodes.GetSize(); i++)
        {
          SmTreeNode *pAdjNode = sAdjacentNodes[i];

          // skip non-leaf nodes
          if (pAdjNode->m_pChild1 != NULL)
            { continue; }

          // when leaf node has been classified
          SmBezierAux2d *pAdjAux = (SmBezierAux2d*)pAdjNode->m_pData;
          if(   pAdjAux->m_eNodeClass == SM_NC_INSIDE
             || pAdjAux->m_eNodeClass == SM_NC_OUTSIDE)
            {
              // remember the classified node's classification data
              eFoundNodeClass = pAdjAux->m_eNodeClass;
              if (pAdjAux->m_eNodeClass == SM_NC_INSIDE)
                {
                  pFoundFace = (SmFace*)pAdjAux->m_pFace;
                }
              break;
            }
        } // end iter every adjacent node

      // when no neighbor was classified
      if (eFoundNodeClass == SM_NC_UNKNOWN)
        {
          SmBoolean bIsInside;

          // gwc: why use the PolyBrep's 1st polyface
          SmPolyFace *pPFace = mTS_pPolyBrep->GetFirstPolyFace();
          pFoundFace = pPFace->GetOriginalFace() ;

          // classify the node
          SER(pPFace->PointInPolygon(pAux->m_sUVDomain.Evaluate(0.5,0.5),
                                     bIsInside));
          // remember the classification
          if (bIsInside)
            {
              eFoundNodeClass = SM_NC_INSIDE;
              pPFace->SetOriginalFace(pFoundFace, TRUE) ; // TRUE=propagate attributes if possible
            }
          else
            {
              eFoundNodeClass = SM_NC_OUTSIDE;
            }
        }

      // Now we should have a point classification go through list and
      // mark each node in the proper way.

      // for every neighbor
      for (ULONG j=0; j<sAdjacentNodes.GetSize(); j++)
        {
          SmTreeNode *pAdjNode = sAdjacentNodes[j];

          // skip non-leaf nodes
          if (pAdjNode->m_pChild1 != NULL)
            { continue; }

          // for unmarked neighbor leaf nodes
          SmBezierAux2d *pAdjAux = (SmBezierAux2d*)pAdjNode->m_pData;
          if (!pAdjAux->m_bIsMarked)
            {
              // copy the remembered classification data to the node
              pAdjAux->m_bIsMarked  = TRUE;
              pAdjAux->m_eNodeClass = eFoundNodeClass;
              pAdjAux->m_pFace      = pFoundFace;

              // Propagate information up tree
              SmTreeNode *pParent = pAdjNode->m_pParent;
              while (pParent != NULL)
                {
                  SmBezierAux2d *pParentAux = (SmBezierAux2d*)pParent->m_pData;
                  SmBezierAux2d *pAux1      = (SmBezierAux2d*)pParent->m_pChild1->m_pData;
                  SmBezierAux2d *pAux2      = (SmBezierAux2d*)pParent->m_pChild2->m_pData;

                  // when both children are marked - mark the parent
                  if(   pAux1->m_bIsMarked
                     && pAux2->m_bIsMarked)
                    {
                      pParentAux->m_bIsMarked = TRUE;
                    }

                  // when both children have the current in/out classification
                  if(   pAux1->m_eNodeClass == eFoundNodeClass
                     && pAux2->m_eNodeClass == eFoundNodeClass)
                    {
                      // give the common classification to the parent
                      pParentAux->m_eNodeClass = eFoundNodeClass;
                      pParentAux->m_pFace      = pFoundFace;
                    }
                  // move up the tree
                  pParent = pParent->m_pParent;
                } // end while parents
            } // end unmarked neighbor node check
        } // end iter every neighbor

      // find target node for next iteration
      pCurrNode = FindUnmarkedLeafNode(SM_NC_UNKNOWN,sStack);

    } // end while nodes remain to be classified

  // all done
  return SM_SUCCESS;

} // end SmTessSrfCache::MarkFaceContainment

/*******************************************************************//**
PURPOSE: Constructor for SmTessSrfCache.  It allows input of data used
    during tessellation of the cache.  Note that this constructor only
    inputs values.  The actual creation of cache data is done in the
    BuildTree method.

NOTES: All arguments except the curve and interval are optional.
***********************************************************************/
SmTessSrfCache::SmTessSrfCache
 (SmFace              * pFace,                   // in : ptr to Face with TrimSurface to tessellate
  SmPolyBrep          * pPolyBrep,               // in : PolyBrep tessellation of the TrimmedSurface (the polygons)
  SmSurfaceTessDriver * pSurfaceTessDriver,      // in : contains: m_vViewVector, m_dSilhouetteChordHeight, m_dSilhouetteAngleToleranceDeg
  double                dChordHeightTolerance,   // in : max allowed control-Point dist to patch basePlane,       0.0 = ignore
  double                dAngleTolerance,         // in : max allowed controlPolygon endTangent angles (radians),  0.0 = ignore
  double                dMaxEdgeLength3D,        // in : max allowed basePolygon side length in u or v direction, 0.0 = ignore
  double                dMinEdgeLength3D,        // in : polygon side length stopping size for subdivision,       0.0 = ignore
  double                dMinEdgeLengthRatioUV,   // in : Subdivison stops once node get smaller than this size,   0.0 = ignore
  double                dMaxAspectRatio)         // in : max allowed element basepolygon aspect ratio,            0.0 = ignore
: SmTrimSrfCache      (*pFace->GetSurface(),
                        dChordHeightTolerance,
                        dAngleTolerance),
  m_pFace             (pFace),
  mTS_pPolyBrep       (pPolyBrep),
  m_pSurfaceTessDriver(pSurfaceTessDriver)
{
  m_dAspectRatio3D            = dMaxAspectRatio;
  m_dMaximumSideLength3D      = dMaxEdgeLength3D; //gives lots of cache on large planes
  m_dMinimumSideLength3D      = dMinEdgeLength3D;
  m_dMinimumSideLengthRatioUV = dMinEdgeLengthRatioUV;

} // end SmTessSrfCache::SmTessSrfCache constructor

/*******************************************************************//**
PURPOSE: Destructor for the tessellating surface cache.

NOTES: 
***********************************************************************/
SmTessSrfCache::~SmTessSrfCache()
{
  // SM_ASSERT(m_pPolyBrep != NULL) ;
  if(mTS_pPolyBrep) { delete mTS_pPolyBrep ; mTS_pPolyBrep = NULL ; }

} // end SmTessSrfCache::~SmTessSrfCache destructor

/*******************************************************************//**
PURPOSE:
NOTES:
***********************************************************************/
SmBoolean SmTessSrfCache::IsKindOf( SM_TYPE t ) const
{
  return ((SmTessSrfCache_TYPE == t) ? TRUE : SmTrimSrfCache::IsKindOf( (t) ));
}

/*******************************************************************//**
PURPOSE:

NOTES: 
***********************************************************************/
void SmTessSrfCache::Dump() const
{
  TCHAR sBuff[SM_TBLOCK_SIZE], sBuffForFile[SM_TBLOCK_SIZE];

  SmTArray<void*> sNodes;
  m_pTree->m_sNodeMgr.GetActiveElements(sNodes);


  // derived label
  smos_sprintf(sBuff,       _T("\nBEGIN SmTessSrfCache = 0x%p, Face = 0x%p, mTS_pPolyBrep = 0x%p, Number of Nodes = %ld"),
                          this,m_pFace,mTS_pPolyBrep,sNodes.GetSize());
  smos_sprintf(sBuffForFile,_T("\nBEGIN SmTessSrfCache = %s, Face = %s, mTS_pPolyBrep = %s, Number of Nodes = %ld"),
                          _T("notNULL"),
                          m_pFace       ? _T("notNULL") : _T("NULL"),
                          mTS_pPolyBrep ? _T("notNULL") : _T("NULL"),
                          sNodes.GetSize());
  smos_WriteBuffer(sBuff, sBuffForFile);

  // mTS_PolyBrep (Face's UV tessellation)
  if(mTS_pPolyBrep)
    {
      smos_sprintf(sBuff,       _T("\nBEGIN SmTessSrfCache[0x%p]::mTS_pPolyBrep[0x%p] Dump() (the tessellation PolyBrep)"),this,mTS_pPolyBrep);
      smos_sprintf(sBuffForFile,_T("\nBEGIN SmTessSrfCache[%s]::mTS_pPolyBrep[%s] Dump() (Face tessellation PolyBrep)"), _T("notNULL"),
                                                                                                                      mTS_pPolyBrep ? _T("notNULL") : _T("NULL"));
      smos_WriteBuffer(sBuff, sBuffForFile);

      if(mTS_pPolyBrep)
        { mTS_pPolyBrep->Dump() ; }

      smos_sprintf(sBuff,       _T("\nEND SmTessSrfCache[0x%p]::mTS_pPolyBrep[0x%p] Dump() (the tessellation PolyBrep)"),this,mTS_pPolyBrep);
      smos_sprintf(sBuffForFile,_T("\nEND SmTessSrfCache[%s]::mTS_pPolyBrep[%s] Dump() (Face tessellation PolyBrep)"), _T("notNULL"),
                                                                                                                    mTS_pPolyBrep ? _T("notNULL") : _T("NULL"));
      smos_WriteBuffer(sBuff, sBuffForFile);
    }

  // SmTrimSrfCache base class
  smos_sprintf(sBuff,       _T("\nSmTessSrfCache[0x%p]::SmTrimSrfCache BaseClass Dump(): no additional members"),
                          this);
  smos_sprintf(sBuffForFile,_T("\nSmTessSrfCache[%s]::SmTrimSrfCache BaseClass Dump(): no additional members"),
                          _T("notNULL"));
  smos_WriteBuffer(sBuff, sBuffForFile);

  // SmSurfaceCache base class
  smos_sprintf(sBuff,       _T("\nBEGIN SmTessSrfCache[0x%p]::SmTrimSrfCache::SmSurfaceCache BaseClass Dump() "),
                          this);
  smos_sprintf(sBuffForFile,_T("\nBEGIN SmTessSrfCache[%s]::SmTrimSrfCache::SmSurfaceCache BaseClass Dump() "),
                          _T("notNULL"));
  smos_WriteBuffer(sBuff, sBuffForFile);


  this->SmSurfaceCache::Dump() ;


  smos_sprintf(sBuff,     _T("\nEND SmTessSrfCache[0x%p]::SmTrimSrfCache::SmSurfaceCache BaseClass Dump() "),
                          this);
  smos_sprintf(sBuffForFile,_T("\nEND SmTessSrfCache[%s]::SmTrimSrfCache::SmSurfaceCache BaseClass Dump() "),
                          _T("notNULL"));
  smos_WriteBuffer(sBuff, sBuffForFile);

  // all done
  smos_sprintf(sBuff,       _T("\nEND SmTessSrfCache[0x%p], Face = 0x%p, mTS_pPolyBrep = 0x%p, Number of Nodes = %ld"),
                          this, m_pFace, mTS_pPolyBrep, sNodes.GetSize());
  smos_sprintf(sBuffForFile,_T("\nEND SmTessSrfCache[%s], Face = %s, mTS_pPolyBrep = %s, Number of Nodes = %ld"),
                          _T("notNULL"),
                          m_pFace       ? _T("notNULL") : _T("NULL"),
                          mTS_pPolyBrep ? _T("notNULL") : _T("NULL"),
                          sNodes.GetSize());
  smos_WriteBuffer(sBuff, sBuffForFile);

} // end SmTessSrfCache::Dump

/*******************************************************************//**
PURPOSE:

NOTES: 
***********************************************************************/
SmDisplayList * SmTessSrfCache::DrawTessellation2D
 (SmPlane       * pOptOutPlane, // in : Draw on this plane, NULL=draw on z=0 plane, default:[NULL]
  SmGfxArraySet * pOptGfxSet)   // i/o: used for OpenGL ES 2.0. When given, accumulate GfxVertexArrays not GL calls.
                                //      NULL to ignore. default:[NULL]
const
{
  SmDisplayList *pRtn = NULL ;

#ifdef SM_GFX_CODE
  // start new displayList (unless one is already open)
  SmVector3d sColor = smgfx_GetOutputColor(pOptGfxSet) ;
  smgfx_Open(smgfx_GetRuleColor(this), NULL, NULL, FALSE, pOptGfxSet);

  if (mTS_pPolyBrep)
    {
      smgfx_SetColor(0,0,1, pOptGfxSet);
      // Now implant edgeuses and vertexuses into the cache tree
      mTS_pPolyBrep->Draw(FALSE,        // in : TRUE = add PolyBrep to display list so it can be picked, FALSE= don't, default:[FALSE]
                          TRUE,         // in : TRUE = draw pts in NativeSpace, default:[TRUE]
                          FALSE,        // in : TRUE = draw pts after projecting through OrigSurf, default:[FALSE]
                          pOptOutPlane, // in : When bDrawRaw==TRUE, Draw on this plane, NULL=draw on z=0 plane, default:[NULL]
                          pOptGfxSet);  // i/o: used for OpenGL ES 2.0. When given, accumulate GfxVertexArrays not GL calls.
    }

  // end DisplayList
  smgfx_OutputColor(sColor, pOptGfxSet) ;
  pRtn = smgfx_Close(pOptGfxSet) ;

#else
  SM_REF2(pOptOutPlane, pOptGfxSet);
#endif // SM_GFX_CODE
  return(pRtn) ;

} // end SmTessSrfCache::DrawTessellation2D

/*******************************************************************//**
PURPOSE:

NOTES: 
***********************************************************************/
SmDisplayList * SmTessSrfCache::DrawTessellation3D
 (SmGfxArraySet * pOptGfxSet)    // i/o: used for OpenGL ES 2.0. When given, accumulate GfxVertexArrays not GL calls.
                                 //      NULL to ignore. default:[NULL]
 const
{
  SmDisplayList *pRtn = NULL ;

#ifdef SM_GFX_CODE
  // start new displayList (unless one is already open)
  SmVector3d sColor = smgfx_GetOutputColor(pOptGfxSet) ;
  smgfx_Open(smgfx_GetRuleColor(this), NULL, NULL, FALSE, pOptGfxSet);

  if (mTS_pPolyBrep)
    {
      smgfx_SetColor(0,0,1, pOptGfxSet);
      // Now implant edgeuses and vertexuses into the cache tree
      mTS_pPolyBrep->Draw(FALSE,       // in : TRUE = add PolyBrep to display list so it can be picked, FALSE=don't, default:[FALSE]
                          FALSE,       // in : TRUE = draw pts in NativeSpace, FALSE=don't, default:[TRUE]
                          TRUE,        // in : TRUE = draw pts after projecting through OrigSurf, FALSE=don't, default:[FALSE]
                          NULL,        // in : When bDrawRaw==TRUE, Draw on this plane, NULL=draw on z=0 plane, default:[NULL]
                          pOptGfxSet); // i/o: used for OpenGL ES 2.0. When given, accumulate GfxVertexArrays not GL calls.
    }

  // end DisplayList
  smgfx_OutputColor(sColor, pOptGfxSet) ;
  pRtn = smgfx_Close(pOptGfxSet) ;

#else
  SM_REF1(pOptGfxSet);
#endif // SM_GFX_CODE
  return(pRtn) ;

} // end SmTessSrfCache::DrawTessellation3D

