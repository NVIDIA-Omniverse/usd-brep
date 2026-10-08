// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************//**
* FILE NAME --- SmSurfaceIntersector.cpp 
* PURPOSE: This file contains surface intersector methods.
**********************************************************************/

 
#include "StdAfx.h"

#include <SmSurfaceIntersector.h>

#include <SmHermiteCurve.h>
#include <SmSurfaceCache.h>
#include <SmCurveCache.h>
#include <SmCacheMgrSrf.h>
#include <SmGeomUtility.h>
#include <SmConic.h>
#include <SmCrvOnSurf.h>
#include <SmSArray.h>
#include <SmOffsetSurface.h>
#include <SmMatrix.h>
#include <SmGraphicsOutput.h>
#include <SmAssertArray.h>
#include <SmFilletExecutive.h>
#ifdef SM_DEBUG_CODE
#include <SmFilletIntersector.h>
#endif // SM_DEBUG_CODE


// FIle Function Forward Declarations
SmBoolean sm_IsPointUnique
  (SmVector3d          &rPoint,       // in : proposed degenerate point
   SmTList<SmTsectPnt> &rStartPoints, // in : list of remaining start points to search
   SmTArray<SmCurve*>  &r3DCurves,    // in : list of 3DCurves to check
   double              d3DTol) ;      // in : min dist between unique 3d points

/*******************************************************************//**
PURPOSE: Construct a SmTsectPnt object.

NOTES: 
***********************************************************************/
SmTsectPnt::SmTsectPnt
  ( )
 : m_ePointType(SM_IP_UNKNOWN),
   m_dCurveParameter(0.0),
   m_dTangentPlaneAngleRad(0.0),
   m_dDeviation(0.0)
{
  m_eUVParamType[0][0]   = SM_EP_UNKNOWN ;
  m_eUVParamType[0][1]   = SM_EP_UNKNOWN ;
  m_eUVParamType[1][0]   = SM_EP_UNKNOWN ;
  m_eUVParamType[1][1]   = SM_EP_UNKNOWN ;

  m_apUserPointer[0] = m_apUserPointer[1] = NULL;
  for (ULONG i=0; i<SM_MAX_USER_DOUBLES; i++) 
    {
      m_adUserDoubles[i] = 0.0;
    }

  // SmVector3d and SmVector2d components are initialized to SM_UNDEF_DOUBLE
//  for (ULONG j=0; j<2; j++) 
//    {
//      m_v3DCurvePV[j].x = m_v3DCurvePV[j].y = m_v3DCurvePV[j].z = 0.0;
//      m_vUVCurvePV[j][0].x = m_vUVCurvePV[j][0].y = 0.0;
//      m_vUVCurvePV[j][1].x = m_vUVCurvePV[j][1].y = 0.0;
//      for (ULONG m=0; m<3; m++) 
//        {
//          for (ULONG n=0; n<3; n++) 
//            {
//              m_vSurfacePV[j][m][n].x = 0.0;
//              m_vSurfacePV[j][m][n].y = 0.0;
//              m_vSurfacePV[j][m][n].z = 0.0;
//            }
//        }
//    }
} // end SmTsectPnt::SmTsectPnt default constructor

/*******************************************************************//**
PURPOSE: copy constructor

NOTES:
***********************************************************************/
SmTsectPnt::SmTsectPnt
  (const SmTsectPnt &crTsectPnt)        // in : object to copy     
{ 
  m_ePointType            = crTsectPnt.m_ePointType ; 
                          
  m_eUVParamType[0][0]    = crTsectPnt.m_eUVParamType[0][0] ;  
  m_eUVParamType[0][1]    = crTsectPnt.m_eUVParamType[0][1] ;  
  m_eUVParamType[1][0]    = crTsectPnt.m_eUVParamType[1][0] ;  
  m_eUVParamType[1][1]    = crTsectPnt.m_eUVParamType[1][1] ;
                                                           
  m_vUVCurvePV[0][0]       = crTsectPnt.m_vUVCurvePV[0][0] ;     
  m_vUVCurvePV[0][1]       = crTsectPnt.m_vUVCurvePV[0][1] ;     
  m_vUVCurvePV[1][0]       = crTsectPnt.m_vUVCurvePV[1][0] ;     
  m_vUVCurvePV[1][1]       = crTsectPnt.m_vUVCurvePV[1][1] ;  
                          
  m_vSurfacePV[0][0][0]   = crTsectPnt.m_vSurfacePV[0][0][0] ;  
  m_vSurfacePV[0][0][1]   = crTsectPnt.m_vSurfacePV[0][0][1] ;  
  m_vSurfacePV[0][0][2]   = crTsectPnt.m_vSurfacePV[0][0][2] ;  
  m_vSurfacePV[0][1][0]   = crTsectPnt.m_vSurfacePV[0][1][0] ;  
  m_vSurfacePV[0][1][1]   = crTsectPnt.m_vSurfacePV[0][1][1] ;  
  m_vSurfacePV[0][1][2]   = crTsectPnt.m_vSurfacePV[0][1][2] ;  
  m_vSurfacePV[0][2][0]   = crTsectPnt.m_vSurfacePV[0][2][0] ;  
  m_vSurfacePV[0][2][1]   = crTsectPnt.m_vSurfacePV[0][2][1] ;  
  m_vSurfacePV[0][2][2]   = crTsectPnt.m_vSurfacePV[0][2][2] ;
                          
  m_vSurfacePV[1][0][0]   = crTsectPnt.m_vSurfacePV[1][0][0] ;  
  m_vSurfacePV[1][0][1]   = crTsectPnt.m_vSurfacePV[1][0][1] ;  
  m_vSurfacePV[1][0][2]   = crTsectPnt.m_vSurfacePV[1][0][2] ;  
  m_vSurfacePV[1][1][0]   = crTsectPnt.m_vSurfacePV[1][1][0] ;  
  m_vSurfacePV[1][1][1]   = crTsectPnt.m_vSurfacePV[1][1][1] ;  
  m_vSurfacePV[1][1][2]   = crTsectPnt.m_vSurfacePV[1][1][2] ;  
  m_vSurfacePV[1][2][0]   = crTsectPnt.m_vSurfacePV[1][2][0] ;  
  m_vSurfacePV[1][2][1]   = crTsectPnt.m_vSurfacePV[1][2][1] ;  
  m_vSurfacePV[1][2][2]   = crTsectPnt.m_vSurfacePV[1][2][2] ;
                          
  m_v3DCurvePV[0]         = crTsectPnt.m_v3DCurvePV[0] ;       
  m_v3DCurvePV[1]         = crTsectPnt.m_v3DCurvePV[1] ;       
                                                              
  m_dCurveParameter       = crTsectPnt.m_dCurveParameter ;     
  m_dTangentPlaneAngleRad = crTsectPnt.m_dTangentPlaneAngleRad ;
  m_dDeviation            = crTsectPnt.m_dDeviation ;          
  
  for(ULONG ii=0;ii<SM_MAX_USER_DOUBLES;ii++)
    {
      m_adUserDoubles[ii] = crTsectPnt.m_adUserDoubles[ii] ;
    }

  m_apUserPointer[0]      = crTsectPnt.m_apUserPointer[0] ;
  m_apUserPointer[1]      = crTsectPnt.m_apUserPointer[1] ;
    
} // end SmTsectPnt::SmTsectPnt copy constructor

/*******************************************************************//**
PURPOSE: assignment operator

NOTES:

  The [1][2] and [2][1] entries for each surface are unused,
  and are likely uninitialized.
***********************************************************************/
SmTsectPnt & SmTsectPnt::operator=
  (const SmTsectPnt &crTsectPnt)  
{
  if(this == &crTsectPnt) return *this ;

  // init base values
  m_pNext = NULL ; // gwc: check to see if m_pNext should be initialized, copied, or left alone
  m_pPrev = NULL ; // gwc: check to see if m_pPrev should be initialized, copied, or left alone

  // member values
  m_ePointType            = crTsectPnt.m_ePointType ; 
                          
  m_eUVParamType[0][0]    = crTsectPnt.m_eUVParamType[0][0] ;  
  m_eUVParamType[0][1]    = crTsectPnt.m_eUVParamType[0][1] ;  
  m_eUVParamType[1][0]    = crTsectPnt.m_eUVParamType[1][0] ;  
  m_eUVParamType[1][1]    = crTsectPnt.m_eUVParamType[1][1] ;
                                                           
  m_vUVCurvePV[0][0]       = crTsectPnt.m_vUVCurvePV[0][0] ;     
  m_vUVCurvePV[0][1]       = crTsectPnt.m_vUVCurvePV[0][1] ;     
  m_vUVCurvePV[1][0]       = crTsectPnt.m_vUVCurvePV[1][0] ;     
  m_vUVCurvePV[1][1]       = crTsectPnt.m_vUVCurvePV[1][1] ;  
                          
  m_vSurfacePV[0][0][0]   = crTsectPnt.m_vSurfacePV[0][0][0] ;  
  m_vSurfacePV[0][0][1]   = crTsectPnt.m_vSurfacePV[0][0][1] ;  
  m_vSurfacePV[0][0][2]   = crTsectPnt.m_vSurfacePV[0][0][2] ;  
  m_vSurfacePV[0][1][0]   = crTsectPnt.m_vSurfacePV[0][1][0] ;  
  m_vSurfacePV[0][1][1]   = crTsectPnt.m_vSurfacePV[0][1][1] ;  
//m_vSurfacePV[0][1][2]   = crTsectPnt.m_vSurfacePV[0][1][2] ;  
  m_vSurfacePV[0][2][0]   = crTsectPnt.m_vSurfacePV[0][2][0] ;  
//m_vSurfacePV[0][2][1]   = crTsectPnt.m_vSurfacePV[0][2][1] ;  
  m_vSurfacePV[0][2][2]   = crTsectPnt.m_vSurfacePV[0][2][2] ;
                          
  m_vSurfacePV[1][0][0]   = crTsectPnt.m_vSurfacePV[1][0][0] ;  
  m_vSurfacePV[1][0][1]   = crTsectPnt.m_vSurfacePV[1][0][1] ;  
  m_vSurfacePV[1][0][2]   = crTsectPnt.m_vSurfacePV[1][0][2] ;  
  m_vSurfacePV[1][1][0]   = crTsectPnt.m_vSurfacePV[1][1][0] ;  
  m_vSurfacePV[1][1][1]   = crTsectPnt.m_vSurfacePV[1][1][1] ;  
//m_vSurfacePV[1][1][2]   = crTsectPnt.m_vSurfacePV[1][1][2] ;  
  m_vSurfacePV[1][2][0]   = crTsectPnt.m_vSurfacePV[1][2][0] ;  
//m_vSurfacePV[1][2][1]   = crTsectPnt.m_vSurfacePV[1][2][1] ;  
  m_vSurfacePV[1][2][2]   = crTsectPnt.m_vSurfacePV[1][2][2] ;
                          
  m_v3DCurvePV[0]         = crTsectPnt.m_v3DCurvePV[0] ;       
  m_v3DCurvePV[1]         = crTsectPnt.m_v3DCurvePV[1] ;       
                                                              
  m_dCurveParameter       = crTsectPnt.m_dCurveParameter ;     
  m_dTangentPlaneAngleRad = crTsectPnt.m_dTangentPlaneAngleRad ;
  m_dDeviation            = crTsectPnt.m_dDeviation ;          
  
  for(ULONG ii=0;ii<SM_MAX_USER_DOUBLES;ii++)
    {
      m_adUserDoubles[ii] = crTsectPnt.m_adUserDoubles[ii] ;
    }

  m_apUserPointer[0]      = crTsectPnt.m_apUserPointer[0] ;
  m_apUserPointer[1]      = crTsectPnt.m_apUserPointer[1] ;

  return *this ;

} // end ::SmTsectPnt &operator= assignment operator

/*******************************************************************//**
PURPOSE: return TRUE when the TsectPnt lies on either of the
            surface's natural domain boundaries

NOTES:
***********************************************************************/
SmBoolean SmTsectPnt::IsBounded() const
{
  // return TRUE when the TsectPnt projected UVPnts lie on any surface natural domain boundary or is outside
  SmBoolean bRtn =    (m_eUVParamType[0][0] != SM_EP_INSIDE && m_eUVParamType[0][0] != SM_EP_UNKNOWN) 
                   || (m_eUVParamType[0][1] != SM_EP_INSIDE && m_eUVParamType[0][1] != SM_EP_UNKNOWN) 
                   || (m_eUVParamType[1][0] != SM_EP_INSIDE && m_eUVParamType[1][0] != SM_EP_UNKNOWN) 
                   || (m_eUVParamType[1][1] != SM_EP_INSIDE && m_eUVParamType[1][1] != SM_EP_UNKNOWN) ;
  // all done
  return(bRtn) ;

} // end SmTsectPnt::IsBounded

/*******************************************************************//**
PURPOSE: Set SmTsectPnt m_eUVParamType values for Given UVPoints on            
            Given target UVDomains        
NOTES: Assumes 
  1. The input value arrays and the arrays stored in this SmTsectPnt
     are associated, i.e. all index 0 values are for m_cpSurface[0] within 
     the controlling SmSurfaceIntersector and all index 1 values for m_cpSurface[1].
  2. SmTsectPnt::m_vSurfacePV values are set - needed to size tolerances.
       
***********************************************************************/
void SmTsectPnt::ClassifyPointParams  // eff: Set m_eUVParamType values for
  (SmPoint2d         aUVs[2],         // in : Given UVPoints on SmSurfaceIntersector::m_cpSurface[0 and 1]
   const SmSurface * apSurface[2],    // in : The SmSurfaceIntersector::m_cpSurface array
   const SmExtent2d  aUVDomain[2],    // in : Given target UVDomains for SmSurfaceIntersector::m_cpSurface[0 and 1]
   double            dTol3d)          // in : min distance between unique 3d points
                                      //      default:[SM_EFF_ZERO]
{
  // locals
  //      SmPoint3d  sPt0, sPu0, sPv0 ;
  double     dTolU0, dTolV0, dUSize0, dVSize0 ;
  SmExtent1d sUIvl0  = aUVDomain[0].GetUInterval() ;
  SmExtent1d sVIvl0  = aUVDomain[0].GetVInterval() ;
  double     dUBand0 = sUIvl0.GetLength() / 10.0 ;
  double     dVBand0 = sVIvl0.GetLength() / 10.0 ;

  // srf 0
  if(apSurface[0])
    {
      // apSurface[0]->Evaluate1stDerivatives(aUVs[0], TRUE, TRUE, sPt0, sPu0, sPv0) ;
      dUSize0 = SrfDu(0).Length() ; if(dUSize0 < SM_EFF_ZERO) dUSize0 = SM_EFF_ZERO ;
      dVSize0 = SrfDv(0).Length() ; if(dVSize0 < SM_EFF_ZERO) dVSize0 = SM_EFF_ZERO ;
      dTolU0  = smos_Min(dUBand0, dTol3d / smos_Max(0.99, dUSize0)) ;
      dTolV0  = smos_Min(dVBand0, dTol3d / smos_Max(0.99, dVSize0)) ;

      //      apSurface[0]->Evaluate1stDerivatives(aUVs[0], TRUE, TRUE, sPt0, sPu0, sPv0) ;
      //      dUSize0 = sPu0.Length() ; if(dUSize0 < SM_EFF_ZERO) dUSize0 = SM_EFF_ZERO ;
      //      dVSize0 = sPv0.Length() ; if(dVSize0 < SM_EFF_ZERO) dVSize0 = SM_EFF_ZERO ;
      //      dTolU0  = smos_Min(dUBand0, dTol3d / smos_Max(0.99, dUSize0)) ;
      //      dTolV0  = smos_Min(dVBand0, dTol3d / smos_Max(0.99, dVSize0)) ;
    }
  else
    {
      dTolU0 = dUBand0 ;
      dTolV0 = dVBand0 ;
    }
  if(dTolU0 < SM_EFF_ZERO) dTolU0 = SM_EFF_ZERO ;
  if(dTolV0 < SM_EFF_ZERO) dTolV0 = SM_EFF_ZERO ;
  UParamType(0) = sUIvl0.ClassifyPoint(aUVs[0].x, dTolU0) ;
  VParamType(0) = sVIvl0.ClassifyPoint(aUVs[0].y, dTolV0) ;

  // locals
  //      SmPoint3d  sPt1, sPu1, sPv1 ;
  double     dTolU1, dTolV1, dUSize1, dVSize1 ;
  SmExtent1d sUIvl1  = aUVDomain[1].GetUInterval() ;
  SmExtent1d sVIvl1  = aUVDomain[1].GetVInterval() ;
  double     dUBand1 = sUIvl1.GetLength() / 10.0 ;
  double     dVBand1 = sVIvl1.GetLength() / 10.0 ;

  // srf 1
  if(apSurface[1])
    {
      // apSurface[1]->Evaluate1stDerivatives(aUVs[1], TRUE, TRUE, sPt1, sPu1, sPv1) ;
      dUSize1 = SrfDu(1).Length() ; if(dUSize1 < SM_EFF_ZERO) dUSize1 = SM_EFF_ZERO ;
      dVSize1 = SrfDv(1).Length() ; if(dVSize1 < SM_EFF_ZERO) dVSize1 = SM_EFF_ZERO ;
      dTolU1  = smos_Min(dUBand1, dTol3d / smos_Max(0.99, dUSize1)) ;
      dTolV1  = smos_Min(dVBand1, dTol3d / smos_Max(0.99, dVSize1)) ;

      //      apSurface[1]->Evaluate1stDerivatives(aUVs[1], TRUE, TRUE, sPt1, sPu1, sPv1) ;
      //      dUSize1 = sPu1.Length() ; if(dUSize1 < SM_EFF_ZERO) dUSize1 = SM_EFF_ZERO ;
      //      dVSize1 = sPv1.Length() ; if(dVSize1 < SM_EFF_ZERO) dVSize1 = SM_EFF_ZERO ;
      //      dTolU1  = smos_Min(dUBand1, dTol3d / smos_Max(0.99, dUSize1)) ;
      //      dTolV1  = smos_Min(dVBand1, dTol3d / smos_Max(0.99, dVSize1)) ;
    }
  else
    {
      dTolU1 = dUBand1 ;
      dTolV1 = dVBand1 ;
    }
  if(dTolU1 < SM_EFF_ZERO) dTolU1 = SM_EFF_ZERO ;
  if(dTolV1 < SM_EFF_ZERO) dTolV1 = SM_EFF_ZERO ;
  UParamType(1) = sUIvl1.ClassifyPoint(aUVs[1].x, dTolU1) ;
  VParamType(1) = sVIvl1.ClassifyPoint(aUVs[1].y, dTolV1) ;

  //      // srf 0
  //      double dUSize0 = SrfDu(0).Length() ; if(dUSize0 < SM_EFF_ZERO) dUSize0 = SM_EFF_ZERO ; 
  //      double dVSize0 = SrfDv(0).Length() ; if(dVSize0 < SM_EFF_ZERO) dVSize0 = SM_EFF_ZERO ;
  //      double dTolUV0 = dTol3d / smos_3Max(0.1, dUSize0, dVSize0) ;
  //      aUVDomain[0].ClassifyPoint2d(aUVs[0], 
  //                                   UParamType(0), 
  //                                   VParamType(0),
  //                                   dTolUV0) ;
  //      
  //      
  //      // srf 1
  //      double dUSize1 = SrfDu(1).Length() ; if(dUSize1 < SM_EFF_ZERO) dUSize1 = SM_EFF_ZERO ; 
  //      double dVSize1 = SrfDv(1).Length() ; if(dVSize1 < SM_EFF_ZERO) dVSize1 = SM_EFF_ZERO ;
  //      double dTolUV1 = dTol3d / smos_3Max(0.1, dUSize1, dVSize1) ;
  //      aUVDomain[1].ClassifyPoint2d(aUVs[1], 
  //                                   UParamType(1), 
  //                                   VParamType(1),
  //                                   dTolUV1) ;

} // end SmTsectPnt::ClassifyPointParams

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
// obsolete
//      void SmTsectPnt::CopySrfEvalMatrix(
//          ULONG lSrf,            // in: which surface, 0 or 1
//          SmVector3d aMat[3][3]  // out: evaluation matrix
//        )
//      {
//        SM_ASSERT( lSrf <= 1 );
//        if ( lSrf > 1 ) { lSrf = 0; }
//      
//        ULONG i, j;
//      
//        for ( i = 0; i < 3; i++ )
//          {
//            for ( j = 0; j < 3; j++ )
//              {
//                // check to avoid assert:
//                if ( m_vSurfacePV[lSrf][i][j].IsInitialized() )
//                  { 
//                    aMat[i][j] = m_vSurfacePV[lSrf][i][j];
//                  }
//                else
//                  { 
//                    aMat[i][j].SetUninitialized();
//                  }
//              }
//          }
//      }

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
void SmTsectPnt::Dump() const
{
  TCHAR sBuff[SM_TBLOCK_SIZE], sBuffForFile[SM_TBLOCK_SIZE];
  
  smos_sprintf(sBuff,        _T("\nSmTsectPnt 0x%p"), this);
  smos_sprintf(sBuffForFile, _T("%s"),_T("\nSmTsectPnt"));
  smos_WriteBuffer(sBuff, sBuffForFile);

  smos_sprintf(sBuff,        _T("\n  PointType       : %s"), 
               m_ePointType == SM_IP_UNDEFINED     ? _T("SM_IP_UNDEFINED    ")     
             : m_ePointType == SM_IP_UNKNOWN       ? _T("SM_IP_UNKNOWN      ")      
             : m_ePointType == SM_IP_CROSSING      ? _T("SM_IP_CROSSING     ")     
             : m_ePointType == SM_IP_TANGENT_POINT ? _T("SM_IP_TANGENT_POINT")
             : m_ePointType == SM_IP_TANGENT_CURVE ? _T("SM_IP_TANGENT_CURVE")
             : m_ePointType == SM_IP_SINGULARITY   ? _T("SM_IP_SINGULARITY  ")  
             : m_ePointType == SM_IP_COINCIDENCE   ? _T("SM_IP_COINCIDENCE  ")
             : m_ePointType == SM_IP_TOUCHING      ? _T("SM_IP_TOUCHING     ")
             : _T("ERROR")) ;
  smos_WriteBuffer(sBuff);     
  smos_sprintf(sBuff,        _T("\n  SurfO UVPt Type : [UParam =%s, VParam=%s]"), 
               m_eUVParamType[0][0] == SM_EP_START     ? _T("SM_EP_START   ")     
             : m_eUVParamType[0][0] == SM_EP_INSIDE    ? _T("SM_EP_INSIDE  ")       
             : m_eUVParamType[0][0] == SM_EP_END       ? _T("SM_EP_END     ")      
             : m_eUVParamType[0][0] == SM_EP_BOTH      ? _T("SM_EP_BOTH    ") 
             : m_eUVParamType[0][0] == SM_EP_OUTSIDE   ? _T("SM_EP_OUTSIDE") 
             : m_eUVParamType[0][0] == SM_EP_UNKNOWN   ? _T("SM_EP_UNKNOWN")   
             : _T("ERROR"),
               m_eUVParamType[0][1] == SM_EP_START     ? _T("SM_EP_START   ")     
             : m_eUVParamType[0][1] == SM_EP_INSIDE    ? _T("SM_EP_INSIDE  ")      
             : m_eUVParamType[0][1] == SM_EP_END       ? _T("SM_EP_END     ")     
             : m_eUVParamType[0][1] == SM_EP_BOTH      ? _T("SM_EP_BOTH    ") 
             : m_eUVParamType[0][1] == SM_EP_OUTSIDE   ? _T("SM_EP_OUTSIDE")
             : m_eUVParamType[0][1] == SM_EP_UNKNOWN   ? _T("SM_EP_UNKNOWN")  
             : _T("ERROR")
             ) ;
  smos_WriteBuffer(sBuff);
  smos_sprintf(sBuff,        _T("\n  Surf1 UVPt Type : [UParam =%s, VParam=%s]"), 
               m_eUVParamType[1][0] == SM_EP_START     ? _T("SM_EP_START   ")     
             : m_eUVParamType[1][0] == SM_EP_INSIDE    ? _T("SM_EP_INSIDE  ")       
             : m_eUVParamType[1][0] == SM_EP_END       ? _T("SM_EP_END     ")      
             : m_eUVParamType[1][0] == SM_EP_BOTH      ? _T("SM_EP_BOTH    ") 
             : m_eUVParamType[1][0] == SM_EP_OUTSIDE   ? _T("SM_EP_OUTSIDE") 
             : m_eUVParamType[1][0] == SM_EP_UNKNOWN   ? _T("SM_EP_UNKNOWN")   
             : _T("ERROR"),
               m_eUVParamType[1][1] == SM_EP_START     ? _T("SM_EP_START   ")     
             : m_eUVParamType[1][1] == SM_EP_INSIDE    ? _T("SM_EP_INSIDE  ")       
             : m_eUVParamType[1][1] == SM_EP_END       ? _T("SM_EP_END     ")      
             : m_eUVParamType[1][1] == SM_EP_BOTH      ? _T("SM_EP_BOTH    ") 
             : m_eUVParamType[1][1] == SM_EP_OUTSIDE   ? _T("SM_EP_OUTSIDE") 
             : m_eUVParamType[1][1] == SM_EP_UNKNOWN   ? _T("SM_EP_UNKNOWN")   
             : _T("ERROR")
             ) ;
  smos_WriteBuffer(sBuff);

  smos_WriteBuffer(_T("\n  Curve Parameter : ")) ; smos_sprintf(sBuff, _T("%16.16lf"), m_dCurveParameter) ;       smos_WriteBuffer(sBuff) ;
  smos_WriteBuffer(_T("\n  Srf/Srf Ang(rad): ")) ; smos_sprintf(sBuff, _T("%16.16lf"), m_dTangentPlaneAngleRad) ; smos_WriteBuffer(sBuff) ;
  smos_WriteBuffer(_T("\n  Max Dev to Surfs: ")) ; smos_sprintf(sBuff, _T("%16.16lf"), m_dDeviation) ;            smos_WriteBuffer(sBuff) ;
  smos_WriteBuffer(_T("\n")) ;
  smos_WriteBuffer(_T("\n  Surf0 UV Pos    : ")) ; m_vUVCurvePV[0][0].Dump();
  smos_WriteBuffer(_T("\n  Surf0 UV Tan    : ")) ; m_vUVCurvePV[0][1].Dump();
  smos_WriteBuffer(_T("\n  Surf1 UV Pos    : ")) ; m_vUVCurvePV[1][0].Dump();
  smos_WriteBuffer(_T("\n  Surf1 UV Tan    : ")) ; m_vUVCurvePV[1][1].Dump();
  smos_WriteBuffer(_T("\n")) ;
  smos_WriteBuffer(_T("\n  3D Curve XYZ Pos: ")) ; m_v3DCurvePV[0].Dump() ; 
  smos_WriteBuffer(_T("\n  3D Curve XYZ Tan: ")) ; m_v3DCurvePV[1].Dump() ; 
  smos_WriteBuffer(_T("\n")) ;
  smos_WriteBuffer(_T("\n  Surf0 XYZ Pos   : ")) ; m_vSurfacePV[0][0][0].Dump() ; 
  smos_WriteBuffer(_T("\n  Surf0 XYZ Tan U : ")) ; m_vSurfacePV[0][1][0].Dump() ; 
  smos_WriteBuffer(_T("\n  Surf0 XYZ Tan V : ")) ; m_vSurfacePV[0][0][1].Dump() ; 
  smos_WriteBuffer(_T("\n  Surf0 XYZ Twist : ")) ; m_vSurfacePV[0][1][1].Dump() ; 
  smos_WriteBuffer(_T("\n  Surf0 XYZ Crv U : ")) ; m_vSurfacePV[0][2][0].Dump() ; 
  smos_WriteBuffer(_T("\n  Surf0 XYZ Crv V : ")) ; m_vSurfacePV[0][0][2].Dump() ; 
  smos_WriteBuffer(_T("\n  Surf0 XYZ Norm  : ")) ; m_vSurfacePV[0][2][2].Dump() ; 
  smos_WriteBuffer(_T("\n")) ;
  smos_WriteBuffer(_T("\n  Surf1 XYZ Pos   : ")) ; m_vSurfacePV[1][0][0].Dump() ; 
  smos_WriteBuffer(_T("\n  Surf1 XYZ Tan U : ")) ; m_vSurfacePV[1][1][0].Dump() ; 
  smos_WriteBuffer(_T("\n  Surf1 XYZ Tan V : ")) ; m_vSurfacePV[1][0][1].Dump() ; 
  smos_WriteBuffer(_T("\n  Surf1 XYZ Twist : ")) ; m_vSurfacePV[1][1][1].Dump() ; 
  smos_WriteBuffer(_T("\n  Surf1 XYZ Crv U : ")) ; m_vSurfacePV[1][2][0].Dump() ; 
  smos_WriteBuffer(_T("\n  Surf1 XYZ Crv V : ")) ; m_vSurfacePV[1][0][2].Dump() ; 
  smos_WriteBuffer(_T("\n  Surf1 XYZ Norm  : ")) ; m_vSurfacePV[1][2][2].Dump() ; 

} // end SmTsectPnt::Dump

/*******************************************************************//**
PURPOSE: Draw a TsectPnt graphic

NOTES:  the 3d Curve tangent and the two 3d surface tangents 
  are supposed to   be coincident.  These will be drawn in different 
  colors but with increasing line widths so that one can observe that 
  they are coincident.
***********************************************************************/
SmDisplayList * SmTsectPnt::Draw() 
  const
{
  SmDisplayList *pRtn = NULL ;

#ifdef SM_GFX_CODE

  // start new displayList (unless one is already open)
  smgfx_Open();

  // output graphics locals
  SmPoint3d  sPos3d = m_v3DCurvePV[0] ;
  SmVector3d sTan3d = m_v3DCurvePV[1] ;

  SmPoint3d  sPos1  = m_vSurfacePV[0][0][0] ;
  SmVector3d sTan1  =   m_vUVCurvePV[0][1].x * m_vSurfacePV[0][1][0]
                      + m_vUVCurvePV[0][1].y * m_vSurfacePV[0][0][1] ;
  SmVector3d sNorm1 = m_vSurfacePV[0][2][2] ;

  SmPoint3d  sPos2  = m_vSurfacePV[1][0][0] ;
  SmVector3d sTan2  =   m_vUVCurvePV[1][1].x * m_vSurfacePV[1][1][0]
                      + m_vUVCurvePV[1][1].y * m_vSurfacePV[1][0][1] ;
  SmVector3d sNorm2 = m_vSurfacePV[1][2][2] ;

  // draw 3d curve pos and tan in red
  SmVector3d sColor = smgfx_OutputColor(1,0,0) ;
  double     dLineWidth = smgfx_GetOutputLineWidth() ;
  smgfx_OutputPoint(sPos3d.x,  sPos3d.y,  sPos3d.z) ;
  smgfx_OutputLine (sPos3d.x,  sPos3d.y,  sPos3d.z,
                    sPos3d.x + sTan3d.x,  sPos3d.y + sTan3d.y,  sPos3d.z + sTan3d.z) ;

  // draw Surf0 pos, tan, norm in blue
  smgfx_OutputColor(0,0,1) ;
  smgfx_OutputLineWidth(dLineWidth+2) ;

  smgfx_OutputPoint(sPos1.x,  sPos1.y,  sPos1.z) ;
  smgfx_OutputLine (sPos1.x,  sPos1.y,  sPos1.z,
                    sPos1.x + sTan1.x,  sPos1.y + sTan1.y,  sPos1.z + sTan1.z) ;
  smgfx_OutputLine (sPos1.x,  sPos1.y,  sPos1.z,
                    sPos1.x + sNorm1.x, sPos1.y + sNorm1.y, sPos1.z + sNorm1.z) ;

  // draw Surf1 pos, tan, norm in green
  smgfx_OutputColor(0,1,0) ;
  smgfx_OutputLineWidth(dLineWidth+4) ;

  smgfx_OutputPoint(sPos2.x,  sPos2.y,  sPos2.z) ;
  smgfx_OutputLine (sPos2.x,  sPos2.y,  sPos2.z,
                    sPos2.x + sTan2.x,  sPos2.y + sTan2.y,  sPos2.z + sTan2.z) ;
  smgfx_OutputLine (sPos2.x,  sPos2.y,  sPos2.z,
                    sPos2.x + sNorm2.x, sPos2.y + sNorm2.y, sPos2.z + sNorm2.z) ;

  smgfx_OutputColor(sColor) ;
  smgfx_OutputLineWidth(dLineWidth) ;

  // end display list
  pRtn = smgfx_Close() ;

#endif // end SM_GFX_CODE
  return(pRtn) ;

} // end SmTsectPnt::Draw


/*******************************************************************//**
PURPOSE: Construct a surface intersector object.

NOTES: 
  m_dSteppingParallelTolRadians, the parallel-normals tolerance:
    This should be as small as possible, because using the previous tangent
    direction in near-tangent cases causes problems (it makes TestSpanAccuracy()
    report an artifically high error, causing crawling and early stopping),
    so do that only in extreme cases.  Use the cross product of the surface
    normals for the intersection-curve direction as long as its direction is
    meaningful.  Starting with unit vectors, if the cross product retains 3
    or 4 significant digits, the direction should be meaningful.
    So SM_EFF_ZERO makes a reasonable cutoff.

  m_dStartParallelTolRadians is used to classify start intersection points
    and needs to be set much looser than m_dSteppingParallelTolRadians to
    prevent misclassify tangent start points as crossing.  That misclassification
    will cause the stepping direction to walk in the wrong direction.
    An initial step in the wrong direction will cause the general
    surface/surface intersector to fail. 

***********************************************************************/
SmSurfaceIntersector::SmSurfaceIntersector
  (const SmSurface  & crSurface1,          // in : Target Surf 1
   const SmExtent2d & crUVDomain1,         // in : Target Surf 1 Domain
   const SmSurface  & crSurface2,          // in : Target Surf 2
   const SmExtent2d & crUVDomain2,         // in : Target Surf 2 Domain
   SmBoolean  bFromFilletIntersector)      // in : TRUE = from FilletIntersector: use UVDomains as given
                                           //      FALSE= reduce given UVDomains to XSect(UVDomain,Surf->UVNatDom)
                                           //      default:[FALSE]
 : 
   m_cpContext                  (NULL),
// m_cpSurface[2]            - set below,
// m_vUVDomain[2]            - set below,
// m_bClosedU[2]             - set below,
// m_bClosedV[2]             - set below,
// m_bUseSurfaceEdges[2]     - set below,
   m_dThisAngTolRad           (SM_DEG2RAD( 20.0 )),
// m_dThisApproxTol3d     - set below,
   m_dSteppingParallelTolRadians( SM_EFF_ZERO ),     // see discussion above
   m_dStartParallelTolRadians   ( SM_EFF_ZERO_RAD ), // see discussion above
// m_dNodeConverganceTol     - uninit - not being used,
   m_dCurveTraceDirection       (1.0), 
   m_dLastThroughParmeter       (0.0),
   m_dExtensionDistance         (0.0),    // parameter distance we want to go beyond if extension is needed
   m_bDoingExtensionClipping    (FALSE),
   m_bCurveIsClosed             (FALSE), 
   m_bCheckForInteriorCurves    (TRUE),
   m_bDoingSelfIntersection     (&crSurface1 == &crSurface2),
   m_bClippedByThroughPoint     (FALSE), 
   m_bDoBoundaryPointOnCurveTest(TRUE),
// m_vTSPntMgr               - set below,
   m_pStartPoint                (NULL), 
   m_pEndPoint                  (NULL)
// m_vCurvePoints            - init by SmTList<SmTsectPnt> default constructor
// m_vStartPoints            - init by SmTList<SmTsectPnt> default constructor
// m_vThroughPoints          - init by SmTList<SmTsectPnt> default constructor
{
  // locals
  SmExtent2d sSurf1UVDomain = crSurface1.GetNaturalUVDomain() ;
  SmExtent2d sSurf2UVDomain = crSurface2.GetNaturalUVDomain() ;

  // check input - given Surf XSect domains fit within Surface NaturalUVDomains
  //               unless bFromFilletIntersector == TRUE
    {
      // check bFromFilletIntersector == TRUE or given UVDomain1 fits within the Surface1 NaturalUVDomain
      if(bFromFilletIntersector || crUVDomain1.IsContainedBy(sSurf1UVDomain, SM_EFF_ZERO))
        { 
          sSurf1UVDomain = crUVDomain1 ;
        }
      else // input domain not contained by natural domain
        {
          // inform the public
          WARN(_T("SmSurfaceIntersector constructor: Given Surface1->UVDomain not completely contained in Surface1->NaturalUVDomain - changing input domain")) ;
          // change input domain to intersection of input and natural domains
          sSurf1UVDomain.Intersect(crUVDomain1, sSurf1UVDomain) ;

          // when that intersection is degenerate - inform the public
          if(sSurf1UVDomain.IsDegenerate(SM_EFF_ZERO))
            WARN(_T("SmSurfaceIntersector constructor: Worse yet - Intersection of given UVDomain with NaturalUVDomain is degenerate")) ;
        }

      // check bFromFilletIntersector == TRUE or given UVDomain2 fits within the Surface2 NaturalUVDomain
      if(bFromFilletIntersector || crUVDomain2.IsContainedBy(sSurf2UVDomain, SM_EFF_ZERO))
        { 
          sSurf2UVDomain = crUVDomain2 ;
        }
      else // input domain not contained by natural domain
        {
          // inform the public
          WARN(_T("SmSurfaceIntersector constructor: Given Surface2->UVDomain not completely contained in Surface2->NaturalUVDomain - changing input domain")) ;
          // change input domain to intersection of input and natural domains
          sSurf2UVDomain.Intersect(crUVDomain2, sSurf2UVDomain) ;

          // when that intersection is degenerate - inform the public
          if(sSurf2UVDomain.IsDegenerate(SM_EFF_ZERO))
            WARN(_T("SmSurfaceIntersector constructor: Worse yet - Intersection of given UVDomain with NaturalUVDomain is degenerate")) ;
        }
     } // end Check input scope

  m_cpSurface[0]        = &crSurface1;
  m_vUVDomain[0]        = sSurf1UVDomain;
  m_bClosedU [0]        = crSurface1.IsClosed(crUVDomain1, SM_SP_U) ;
  m_bClosedV [0]        = crSurface1.IsClosed(crUVDomain1, SM_SP_V) ;
  m_bUseSurfaceEdges[0] = TRUE;

  m_cpSurface[1]        = &crSurface2; 
  m_vUVDomain[1]        = sSurf2UVDomain;
  m_bClosedU [1]        = crSurface2.IsClosed(crUVDomain2, SM_SP_U) ;
  m_bClosedV [1]        = crSurface2.IsClosed(crUVDomain2, SM_SP_V) ;
  m_bUseSurfaceEdges[1] = TRUE;

  // Compute a reasonable default tolerance for intersection based on the sizes
  // of the surfaces.  Note that this may be overridden during subsequent operations.
  SmPoint3d sPMid, sP1, sP2;
  SE(crSurface1.EvaluatePoint(crUVDomain1.GetMin(),sP1));
  SE(crSurface1.EvaluatePoint(crUVDomain1.Evaluate(0.5,0.5),sPMid));
  SE(crSurface1.EvaluatePoint(crUVDomain1.GetMax(),sP2));
  double dSize          = sP1.DistanceBetween(sPMid) + sPMid.DistanceBetween(sP2);
  m_dThisApproxTol3d = dSize/1000.0;  // accuracy is 1/1000th size of surface

  // init TSectPnt manager
  m_vTSPntMgr.Initialize(ALIGN_SIZE(sizeof(SmTsectPnt)),100) ;

  // global solver members

  m_eSolverOperation        = SM_SO_INTERSECT;     // same as SmGlobalSolver::DefaultConstructor
//m_eOperationCategory      = SM_OPERATION_OTHER;  // from SmGlobalSolver::DefaultConstructor  
  m_bProjectedOperation     = FALSE ;              // same as SmGlobalSolver::DefaultConstructor
  m_eSolutionRequested      = SM_SR_NODES;         // different
  m_lNumTrees               = 2;                   // different
  m_lNumVariables           = 4;                   // different
//m_apTrees[0,1,..,SM_GS_MAX_TREES-1] = NULL;      // from SmGlobalSolver::DefaultConstructor 
  m_d3dTolerance            = 0.0;                 // different
  m_dBestAnswerSoFarSq      = 0.0;                 // same as SmGlobalSolver::DefaultConstructor
  m_dAtDistance             = 0.0;                 // same as SmGlobalSolver::DefaultConstructor
  m_dAtDistanceSq           = 0.0;                 // same as SmGlobalSolver::DefaultConstructor
//m_cpOptVectors            = NULL;                // from SmGlobalSolver::DefaultConstructor 
//m_pSolutions              = NULL;                // from SmGlobalSolver::DefaultConstructor 

  // m_slNumGlobalSolves    - static global values
  // m_slNumNodesVisited    - static global values
  // m_slNumLocalSolves     - static global values
  // m_slNumNewtonSolves    - static global values

} // end SmSurfaceIntersector::SmSurfaceIntersector constructor

/*******************************************************************//**
PURPOSE: 

NOTES: 
***********************************************************************/
static SmTsectCurveType sm_ComputeCurveType
  (const SmSurface * cpSurface1,
   const SmSurface * cpSurface2,
   const SmCurve * cpUVCurve1,
   const SmCurve * cpUVCurve2)
{
    SmTsectCurveType eRet = SM_TC_CROSSING;
    SmPoint3d sP1, sP2;
    SmExtent1d sIvl1 = cpUVCurve1->GetNaturalInterval();
    SmExtent1d sIvl2 = cpUVCurve2->GetNaturalInterval();
    cpUVCurve1->EvaluatePoint(sIvl1.Evaluate(0.5),sP1);
    cpUVCurve2->EvaluatePoint(sIvl2.Evaluate(0.5),sP2);
    SmVector3d sNorm1, sNorm2;
    SmVector2d sUV1(sP1.x,sP1.y), sUV2(sP2.x,sP2.y);
    cpSurface1->EvaluateNormal(sUV1,TRUE,TRUE,sNorm1);
    cpSurface2->EvaluateNormal(sUV2,TRUE,TRUE,sNorm2);
    double dSign = 1.0;
    double dDot = sNorm1.Dot(sNorm2);
    if (dDot < 0.0) { dSign = -1.0; }

    double dLimit = 0.0001; // was SM_EFF_ZERO_SQRT [B162]

    // Test to see if normals coincide
    if (smos_Fabs(dDot) > 1.0 - dLimit)
    {
        // We're at least TAN.
        eRet = SM_TC_TANGENT;
        double dGaussCur1 = 0.0, dNormalCur1 = 0.0, dPrinCur11 = 0.0, dPrinCur12 = 0.0;
        SmVector3d sEFG1, sLMN1, sPrinKV11, sPrinKV12;
        SE(cpSurface1->EvaluateGeometric(sUV1,TRUE,TRUE,
            dGaussCur1, dNormalCur1, dPrinCur11, dPrinCur12,
            sEFG1,sLMN1,sPrinKV11,sPrinKV12));
        double dGaussCur2 = 0.0, dNormalCur2 = 0.0, dPrinCur21 = 0.0, dPrinCur22 = 0.0;
        SmVector3d sEFG2, sLMN2, sPrinKV21, sPrinKV22;
        SE(cpSurface2->EvaluateGeometric(sUV2,TRUE,TRUE,
            dGaussCur2, dNormalCur2, dPrinCur21, dPrinCur22,
            sEFG2,sLMN2,sPrinKV21,sPrinKV22));
        // Now check to see if we have some coincidence occuring

        // First check to see if we have coincident umbilical points
        double dScale = 1.0 + smos_Fabs(dPrinCur11);
        if (smos_Fabs(dPrinCur11-dPrinCur12) < dLimit * dScale) {
            // Surf1 is umbilical.
            // If Surf2 is not umbilical, return TAN.
            if (smos_Fabs(dPrinCur21-dPrinCur22) > dLimit * dScale) { return eRet; }
            // Surf2 is also umbilical.  If radii of curvatures are equal, then COINC.
            if (smos_Fabs(dPrinCur11-dSign*dPrinCur21) > dLimit * dScale) { return eRet; }
            // both umbilical, with the same curvature radii: COINCIDENT.
            eRet = SM_TC_COINCIDENT;
            return eRet;
        }
        // Here, Srf1 is not umbilical. If Srf2 is umbilical, just quit now.
        if (smos_Fabs(dPrinCur21-dPrinCur22) < dLimit * dScale) { return eRet; }

        // Now check to see if we have curvature vectors and magnitudes which are 
        // equivalent.
        if (smos_Fabs(sPrinKV11.Dot(sPrinKV21)) > 1.0 - dLimit) {
            // First prin crv vectors line up.  Compare the appropriate curvatures.
            if (smos_Fabs(dPrinCur11-dSign*dPrinCur21) > dLimit * dScale) { return eRet; }
            double dScale2 = 1.0 + smos_Fabs(dPrinCur12);
            if (smos_Fabs(dPrinCur12-dSign*dPrinCur22) > dLimit * dScale2) { return eRet; }
            eRet = SM_TC_COINCIDENT;
        }
        if (smos_Fabs(sPrinKV11.Dot(sPrinKV22)) > 1.0 - dLimit) {
            // First prin crv vector of Surf1 lines up with 2nd prin crv vector
            // of Surf2.  Compare the appropriate curvatures.
            if (smos_Fabs(dPrinCur11-dSign*dPrinCur22) > dLimit * dScale) { return eRet; }
            double dScale2 = 1.0 + smos_Fabs(dPrinCur12);
            if (smos_Fabs(dPrinCur12-dSign*dPrinCur21) > dLimit * dScale2) { return eRet; }
            eRet = SM_TC_COINCIDENT;
        }
    }
    return eRet;

} // end sm_ComputeCurveType


/*******************************************************************//**
PURPOSE: Make a quick disjointness/grazing test in case (some of) the 
            boundary curves are planar

NOTES: output: 
 SM_BR_GRAZE    one pair of boundary curve bounding planes is
                coincident, and the surfaces are on the opposite 
                sides of this plane
 SM_BR_DISJOINT The surfaces are definitely disjoint: one pair of
                boundary curve planes are parallel, and the surfaces
                are OFF the region enclosed by the parallel planes.
 SM_BR_UNKNOWN  Intersection possible: more work needed
***********************************************************************/
SmStatus 
SmSurfaceIntersector::DoBoundaryPlaneTest
  (SmPatchBoundaryBoundingPlanes& rsBoundingPlanes1Arg,  // in : BoundaryPlanes from SurfaceCache1
   SmPatchBoundaryBoundingPlanes& rsBoundingPlanes2Arg,  // in : BoundaryPlanes from SurfaceCache2
   double d3DTol,                                        // in : max distance between distinct intersection points
   SmExtent3d &rBBox,                                    // in : intersection bounding box limit
   SmTArray<SmCurve*>         & rp3DCurvesArg,           // out: augmented with bndryCrv/bndryCrv xSects for grazind cases
   SmTArray<SmTsectCurveType> & rCurveTypes,             // out: 
   SmTArray<double>           & rDeviations,             // out: 
   SmBoundaryPlaneTestResult  & reResultArg)             // out: oneof SM_BR_GRAZE one pair of boundary curve bounding planes is       
                                                         //                        coincident, and the surfaces are on the opposite    
                                                         //                        sides of this plane                                 
                                                         //            SM_BR_DISJOINT The surfaces are definitely disjoint: one pair of
                                                         //                        boundary curve planes are parallel, and the surfaces
                                                         //                        are OFF the region enclosed by the parallel planes. 
                                                         //            SM_BR_UNKNOWN  Intersection possible: more work needed          
{
  // hard coded tolerances
  double dSize = rBBox.GetSize().Length() ;
  double dAngleTol    = 1e-4; // -hof- ??
  double dDistanceTol = smos_Min(1e-2, d3DTol/dSize)/100.0; // used for plane coincidence -hof- ??
  // Reminder that I am not sure if these numbers are OK for you.
  
  // locals
  const SmSurface        * pSurf1 = m_cpSurface[0];
  const SmSurface        * pSurf2 = m_cpSurface[1];
  const SmBSplineSurface * pBSS1  = SM_CAST_PTR(SmBSplineSurface, pSurf1);
  const SmBSplineSurface * pBSS2  = SM_CAST_PTR(SmBSplineSurface, pSurf2);

  // Control polygons are not available for general surfaces: 
  // some tests within this function are based on them, so bail out if not 
  // a BSplineSurface:
  if (   pBSS1 == NULL 
      || pBSS2 == NULL) 
    {
      reResultArg = SM_BR_UNKNOWN;
      return SM_SUCCESS;
    }

  // for all combinations of boundaries between both surfaces
  for(int ii=0; ii<4; ii++) 
    {
      // locals
      SmBoolean  bPlaneExists1,        bPlaneExists2;
      SmBoolean  bBoundaryDegenerate1, bBoundaryDegenerate2;
      SmPoint3d  sPlaneBasePoint1,     sPlaneBasePoint2;
      SmVector3d sPlaneNormal1,        sPlaneNormal2;
      SmPoint3d *pSamplePoint1=NULL,  *pSamplePoint2 = NULL;
      double     dTolerance1,          dTolerance2;

      // get surf1 ith boundary planarity (and plane values when it is)
      SER(rsBoundingPlanes1Arg.GetNthBoundingPlane(ii,
                                       bPlaneExists1,
                                       bBoundaryDegenerate1,
                                       sPlaneBasePoint1,sPlaneNormal1,pSamplePoint1,
                                       dTolerance1));
      for(int jj=0; jj<4; jj++) 
        {
          // get surf2 jth boundary planarity (and plane values when it is)
          SER(rsBoundingPlanes2Arg.GetNthBoundingPlane(jj, 
                                           bPlaneExists2,
                                           bBoundaryDegenerate2,
                                           sPlaneBasePoint2,sPlaneNormal2,pSamplePoint2,
                                           dTolerance2));
          
          // skip cases that are not planar/planar                                                      
          if(!bPlaneExists1 || !bPlaneExists2)
            { continue ; }
 
          // The plane normals point 'off the material', so to 
          // detect disjointness or grazing here they must be oriented 
          // opposite and parallel to within tolerance:
          double dDotDiscr = sPlaneNormal1.Dot(sPlaneNormal2);
          if(dDotDiscr > -1.0+dAngleTol) // -hof- ??
            { continue ; }
 
          // check for disjoint or overlap cases - all curve sample points must
          //   lie outside the material side of the other curve's
          //   bounding plane. Check sample points on both curves
          //   because planes are only parallel to within tolerance
          //   and two planes can be disjoint at their basePoints while
          //   not disjoint at some of the sample points.
          SmBoolean bDisjoint = TRUE ;
          SmBoolean bOverlap  = FALSE ;
          double dMax1 = sPlaneBasePoint1.GetMaxDimension() ;
          double dMax2 = sPlaneBasePoint2.GetMaxDimension() ;  
          double dTolSum = smos_Max(dDistanceTol/1000.0, dTolerance1 + dTolerance2) * (1.0 + smos_Max(dMax1, dMax2)) ;
          for(ULONG smp=0; smp<SM_PB_SAMPLECOUNT; smp++)
            {
              double dSignedPlaneDist2 = sPlaneNormal2.Dot(pSamplePoint1[smp] - sPlaneBasePoint2) ;
              double dSignedPlaneDist1 = sPlaneNormal1.Dot(pSamplePoint2[smp] - sPlaneBasePoint1) ;

              // check for non Disjoint boundary surfaces (overlapping or coincident)
              if(   dSignedPlaneDist2 <= dTolSum
                 || dSignedPlaneDist1 <= dTolSum)
                { bDisjoint = FALSE ;
                }
              
              // check for Overlapping surfaces
              if(   dSignedPlaneDist1 < -dDistanceTol
                 || dSignedPlaneDist2 < -dDistanceTol)
                {
                  bOverlap = TRUE ;
                  break ;
                }
            } // end iter all boundary curve sample points

          // disjoint Surface Bounding planes - no intersection possible
          if(bDisjoint) { reResultArg = SM_BR_DISJOINT;
                          return SM_SUCCESS;
                        }
          
          // overlapping Surface Bounding planes - go onto next boundary plane pair
          if(bOverlap)  { continue ;
                        }

          // We arrive here for grazing or 'coincident boundary planes' case.
          //   Assume this is the only possible pair of grazing naturalBoundary planes.
          //   Load output with NatrualBoundary Curve/Curve intersections and return. 
          reResultArg = SM_BR_GRAZE;
 
          // build grazing Surface->NaturalBoundaryCurves 
          SmBSplineCurve* pBSC1, *pBSC2;
          SmPoint2d sMin1, sMax1 ; 
          SmPoint2d sMin2, sMax2 ;
          SER(rsBoundingPlanes1Arg.CreateBoundaryCurveForIndex(ii, pBSC1));
          SER(rsBoundingPlanes2Arg.CreateBoundaryCurveForIndex(jj, pBSC2));
          SER(rsBoundingPlanes1Arg.GetBoundaryUVEndPoints(ii, sMin1, sMax1));
          SER(rsBoundingPlanes2Arg.GetBoundaryUVEndPoints(jj, sMin2, sMax2));

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
static ULONG lCount = 1 ; lCount++ ;
static ULONG lDebugCount = 0 ;
          // add drawing of CurveBound Points
          if(bDebugMe || lDebugCount == lCount)
            {
              SM_ASSERT_VALID(pBSC1) ;
              SM_ASSERT_VALID(pBSC2) ;
              SmPoint3d sStart1, sEnd1 ;
              SmPoint3d sStart2, sEnd2 ;

              pSurf1->EvaluatePoint(sMin1, sStart1) ;
              pSurf1->EvaluatePoint(sMax1, sEnd1) ;
              pSurf2->EvaluatePoint(sMin2, sStart2) ;
              pSurf2->EvaluatePoint(sMax2, sEnd2) ;

              SmFace *pFace1 = (SmFace *)m_cpSurface[0]->GetFace() ;
              SmFace *pFace2 = (SmFace *)m_cpSurface[1]->GetFace() ;
              SmBrep *pBrep1 = pFace1 ? pFace1->GetBrep() : NULL ;
              SmBrep *pBrep2 = pFace2 ? pFace2->GetBrep() : NULL ;

              smgfx_Erase() ;
              smgfx_SetLook(1,2, 0,0,1) ; if(pBrep1) pBrep1->Draw(TRUE) ; sm_GraphicsLoop() ;
              smgfx_SetLook(1,2, 0,1,0) ; if(pBrep2) pBrep2->Draw(TRUE) ; sm_GraphicsLoop() ;
              smgfx_SetLook(1,2, 0,1,1) ; m_cpSurface[0]->DrawUV(8,8,FALSE,NULL,pBrep1==NULL) ; sm_GraphicsLoop() ;
              smgfx_SetLook(1,2, 1,1,0) ; m_cpSurface[1]->DrawUV(8,8,FALSE,NULL,pBrep2==NULL) ; sm_GraphicsLoop() ;
              smgfx_SetLook(3,4, 1,0,0) ; sStart1.Draw() ; sEnd1.Draw() ; sm_GraphicsLoop() ;
              smgfx_SetLook(4,5, 1,0,1) ; sStart2.Draw() ; sEnd2.Draw() ; sm_GraphicsLoop() ;
              smgfx_SetLook(2,3, 1,0,0) ; pBSC1->Draw() ; sm_GraphicsLoop() ;
              smgfx_SetLook(2,3, 1,0,1) ; pBSC2->Draw() ; sm_GraphicsLoop() ;
              sm_GraphicsLoop() ;
            }
#endif // SM_DEBUG_CODE


          SmObjDelete sCleanup1(pBSC1);
          SmObjDelete sCleanup2(pBSC2);
 
          // Intersect Curves
          SmSolutionArray sSolutions;
          SER(pBSC1->GlobalCurveIntersect(pBSC1->GetNaturalInterval(),
                                          *pBSC2,
                                          pBSC2->GetNaturalInterval(),
                                          dDistanceTol, sSolutions));
          ULONG nSolutions = sSolutions.GetSize();

          // all done for no intersections
          if (nSolutions == 0) { return SM_SUCCESS; }    

          // for every solution
          for (ULONG kk=0; kk<sSolutions.GetSize(); kk++) 
            {
              // switch on point and interval solutions
              if (sSolutions[kk].m_eSolutionType == SM_ST_SINGLE_VALUE) 
                {
                  // single point intersection - output a degenerate curve
                  SmPoint3d sSolutionPoint;
                  SER(pBSC1->EvaluatePoint(sSolutions[kk].m_vStart[0], 
                                           sSolutionPoint));

                  // make degenerate curve
                  SmBSplineCurve *pDegenerateCurve = NULL ;
                  SER(SmBSplineCurve::CreateDegenerateCurve(*pSurf1->GetContext(),
                                                            3, // dim of result
                                                            sSolutionPoint,
                                                            pDegenerateCurve));
                  NER(pDegenerateCurve);
      
                  // output degenerate curve
                  rp3DCurvesArg.Add(pDegenerateCurve);
                  rCurveTypes.Add(SM_TC_TOUCHING);
                  rDeviations.Add(sSolutions[kk].m_vStart.m_dSolutionValue);
                }
              else // interval intersection - Output a boundary curve trimmed to the intersection interval
                {
                  // get intersection intervals
                  double dStartParam1 = sSolutions[kk].m_vStart[0];
                  double dStartParam2 = sSolutions[kk].m_vStart[1];
                  double dEndParam1   = sSolutions[kk].m_vEnd[0];
                  double dEndParam2   = sSolutions[kk].m_vEnd[1];
                  //SmBoolean bSwap2 ;
                  if(dStartParam2 > dEndParam2) { //bSwap2       = TRUE ;
                                                  double dTmp  = dStartParam2 ;
                                                  dStartParam2 = dEndParam2 ;
                                                  dEndParam2   = dTmp ;
                                                }
                  else                          { //bSwap2       = FALSE ;
                                                }
                  SmExtent1d sTrimIvl1(dStartParam1, dEndParam1);
                  SmExtent1d sTrimIvl2(dStartParam2, dEndParam2);
  
                  // for nonZero intersection intervals
                  SM_ASSERT(sTrimIvl1.GetLength() > SM_EFF_ZERO) ;
                  if (sTrimIvl1.GetLength() > SM_EFF_ZERO) 
                    {
                      // copy and trim the Surf1->NaturalBoundaryCurve
                      NER(pBSC1);
                      SmBSplineCurve* pBSC = new (*pSurf1->GetContext()) SmBSplineCurve(*pBSC1);
                      pBSC->Trim(sTrimIvl1);   // may snap sIvl by tol to existing knots

                      // classify intersection crossing/tangent/coincident 
                      // -cheap check- check interval endpoints only
                      //   we know that these points line up.  To check a midPoint
                      //   we have to drop a point from one curve to the other to find
                      //   tightly paired curve points prior to making the check.

                      // get SurfacePoints for each interval endPoint
                      SmPoint2d s2DStart[2], s2DEnd[2] ;
                      if(SM_ARE_SAME(sMin1.x,sMax1.x)) { s2DStart[0].Set(sMin1.x, sTrimIvl1.GetMin()) ;
                                                         s2DEnd[0].  Set(sMax1.x, sTrimIvl1.GetMax()) ;
                                                       }
                      else                             { s2DStart[0].Set(sTrimIvl1.GetMin(), sMin1.y) ;
                                                         s2DEnd[0].  Set(sTrimIvl1.GetMax(), sMax1.y) ;
                                                       }
                      if(SM_ARE_SAME(sMin2.x,sMax2.x)) { s2DStart[1].Set(sMin2.x, sTrimIvl2.GetMin()) ;
                                                         s2DEnd[1].  Set(sMax2.x, sTrimIvl2.GetMax()) ;
                                                       }
                      else                             { s2DStart[1].Set(sTrimIvl2.GetMin(), sMin2.y) ;
                                                         s2DEnd[1].  Set(sTrimIvl2.GetMax(), sMax2.y) ;
                                                       }

                      // get endPoint SurfaceValues
                      //SmVector3d sStartNorm[2], sEndNorm[2] ;
                      SmTsectPnt sStartTPnt, sEndTPnt ;
                      ComputePointValues(s2DStart, sStartTPnt) ;
                      ComputePointValues(s2DEnd,   sEndTPnt) ;

                      // classify IntersectionCurve from its endPoint intersection classifications
                      SmTsectCurveType eType =   
                         (   sStartTPnt.m_ePointType == SM_IP_COINCIDENCE
                          && sEndTPnt.m_ePointType   == SM_IP_COINCIDENCE)   ? SM_TC_COINCIDENT
                       : (   sStartTPnt.m_ePointType >= SM_IP_TANGENT_CURVE 
                          && sEndTPnt.m_ePointType   >= SM_IP_TANGENT_CURVE) ? SM_TC_TANGENT
                       : SM_TC_CROSSING ;
                      
                      // output the curve
                      rp3DCurvesArg.Add(pBSC);
                      rCurveTypes.Add(eType);
                      rDeviations.Add(smos_Max(sSolutions[kk].m_vStart.m_dSolutionValue,
                                               sSolutions[kk].m_vEnd.m_dSolutionValue) );
                    }
                } // end interval solution branch
            } // end iter every solution

          // all done - the case is grazing and all intersections have been output
          return SM_SUCCESS;     
                 
        } // end iter boundaries of surf2
    } // end iter boundaries of surf1

  // no pair of surface natural boundaries were found to be grazing
  reResultArg = SM_BR_UNKNOWN;
  return SM_SUCCESS;     

} // end SmSurfaceIntersector::DoBoundaryPlaneTest

/*******************************************************************//**
PURPOSE: 
  Find surface/surface intersection startpoints by finding  
  all the intersections of
      the surface[0]->m_vUVDomain[0]->isoParameterBoundaryCurves with surface[1]
  and the surface[1]->m_vUVDomain[1]->isoParameterBoundaryCurves with surface[0]. 

NOTES:
  1. Each intersection Point is tested to see if it lies on one of the
      existing intersection curves input in the r3DCurves array.

      - Intersection points not on an existing intersection 
         curve are added to the m_vCurvePoints array.
      - Intersection points on an existing intersection 
         curve are ignored.

  2. When a boundary curve is found to be coincident with the other surface,
     it is trimmed to the intersection domain and added to the r3DCurves array.
     When a boundary curve intersection is found to be tangent, and not
     coincident it is added to the r3DCurves array as a degenerate curve.

  3. Intersection points that happen to lie on a pair of boundary curves
     are placed last m_vCurvePoints array so they get used first when
     tracing out an intersection.

     Intersection points that lie on just one boundary curve are placed
     first in the m_vCurvePoints array.
***********************************************************************/
SmStatus SmSurfaceIntersector::FindBoundaryStartPoints
  (const SmExtent3d   & crIntersectionBox,     // in : bound on surface/surface intersections
   SmTArray<SmCurve*> & r3DCurves,             // i/o: currently found intersections
                                               //      augmented whenever a boundary curve
                                               //      has a coincident range with the other surface
   SmTArray<SmCurve*> & rSurface1UVCurves,     // out: associated Surface1 UVTrimCurves
   SmTArray<SmCurve*> & rSurface2UVCurves,     // out: associated Surface2 UVTrimCurves
   SmTArray<SmTsectCurveType> & rCurveTypes,   // out: associated Curve Type: oneof
                                               //        SM_TC_TOUCHING
                                               //        SM_TC_CROSSING
                                               //        SM_TC_TANGENT
                                               //        SM_TC_COINCIDENT
                                               //        SM_TC_NEAR_TANGENT
                                               //        SM_TC_REGION_BOUNDARY
   SmTArray<double>   & rDeviations)           // out: associated deviations
{
  // locals
  SmBSplineCurve           *pData[8],  *pUVData[8] ;
  SmTsectPnt               *sData[16], *sDegData[4];
  SmCurve                  *pTargetData[8];
  SmPoint3d                 s3dPointData[4] ;
  SmTArray<SmBSplineCurve*> sIsoCurves(8,pData), sUVTrimCurves(8,pUVData) ;
  SmTArray<SmCurve*>        sTargetCurves(8,pTargetData) ;
  SmTArray<SmExtent1d>      sIntervals ;
  SmTArray<SmTsectPnt*>     sEdgeCrossings(16,sData), sDegeneratePoints(4,sDegData) ;
  SmTArray<SmPoint3d>       s3DPoints(4,s3dPointData) ;
  SmTArray<ULONG>           sSurfIndex ;

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe  = FALSE ;
SmBoolean bDebugMe2 = FALSE ;
static int lCount = 1 ; 
static int lDebugCount = 0 ;
lCount ++ ;
#endif

  // get surface caches - make temporary change to processBoundaryCurves
  SmSurfaceCache *apSC[2];
  apSC[0] = smsurf_GetSurfaceCache(m_cpSurface[0]); NER(apSC[0]);
  apSC[1] = smsurf_GetSurfaceCache(m_cpSurface[1]); NER(apSC[1]);
  
  // Turn on boundary curve processing to force LocalSolve to look for drop points
  //   on boundary curves.
  SmTemporaryChangeValue<SmBoolean> sStack1(apSC[0]->m_bProcessBoundaryCurves,TRUE);
  SmTemporaryChangeValue<SmBoolean> sStack2(apSC[1]->m_bProcessBoundaryCurves,TRUE);

  // for both surfaces - gather and intersect surf NaturalBoundaryCurves with other surface
  //                   - classify results - add points    to m_vCurvePoints
  //                   -                  - add intervals to r3DCurves
  for (ULONG lSrf=0; lSrf<2; lSrf++) 
    {
      // skip surfaces marked as not using Surface edges - set by caller who knows one srf's bndries don't xsect the other srf
      if (!m_bUseSurfaceEdges[lSrf]) 
        { continue ; }

      // init working arrays
      sTargetCurves.ReSet() ;                                        
      sIntervals.ReSet() ;                                           
      
      // Get thisSurface->3DBoundaryCurves 
      //       3                                                                     
      //       |                                 
      //     +----+    (exact for BSplineSurfaces and Planes, else approximate)   
      //     |    |     output sIsoCurves ordered: [0] - minimum const U curve    
      //   0-|    |-2                              [1] - minimum const V curve           
      //     +----+                                [2] - maximum const U curve           
      //       |                                   [3] - maximum const V curve  
      //       1                                                                
      SmBoolean bCurvesAreCached;
      SER(apSC[lSrf]->GetIsoBoundaryCurves
            (m_vUVDomain[lSrf],         // in : target Surface UVDomain                                       
             m_dThisApproxTol3d/3.0,    // in : max dist between returned isoParameter Curves and Surface     
             bCurvesAreCached,          // out: TRUE = returned curves will be deleted with surface cache     
                                        //      FALSE= user should delete output curves after done using them.
             sIsoCurves));              // out: IsoParameter Curves bounding input crUVDomain                 
                                        //      ordered: [0] - minimum const U curve                          
                                        //               [1] - minimum const V curve                          
                                        //               [2] - maximum const U curve                          
                                        //               [3] - maximum const V curve  
                                                                
      // Automatic clean up of non-cached curves
      SmTArray<SmBSplineCurve*> *pCurves = (bCurvesAreCached) ? NULL : &sIsoCurves ;
      SmObjsDelete<SmBSplineCurve*> sCleanupCurves(pCurves);

      // init the processing array sizes 
      ULONG lIsoCount = sIsoCurves.GetSize() ;
      sTargetCurves.SetSize(lIsoCount) ;
      sUVTrimCurves.SetSize(lIsoCount) ;
      sIntervals.   SetSize(lIsoCount) ;

      // add IsoCurves and their intervals to processing arrays
      for(ULONG ii=0;ii<lIsoCount;ii++)               
        {                                       
          sTargetCurves[ii] = sIsoCurves[ii] ;  
          sUVTrimCurves[ii] = NULL ;
          
          // map of isocurves used to show proper interval ranges for each ii value           
          //     3     
          //     |     
          //   +----+   lCoord  == 0 for constant U direction NaturalBoundaryCurves
          //   |    |              1 for constant V direction NaturalBoundaryCurves
          // 0-|    |-2           not used for surface->Face->edge curves
          //   +----+  
          //     |     
          //     1     
          ULONG lCoord = ii % 2 ;

          if(lCoord == 1) 
               { sIntervals[ii].SetMinMax(m_vUVDomain[lSrf].GetMin().x,m_vUVDomain[lSrf].GetMax().x) ; }
          else { sIntervals[ii].SetMinMax(m_vUVDomain[lSrf].GetMin().y,m_vUVDomain[lSrf].GetMax().y) ; }

        } // end iter ii, adding isocurve and its interval to processing arrays

// begin GWC:FACE_EDGES_CHANGE

static constexpr SmBoolean bGWCAddEdgeThroughPoints = FALSE ;

      // when asked and surface has a face - include all Surface->Face->Edge->Curves
      if(   m_cpSurface[lSrf]->GetOwner()
         && bGWCAddEdgeThroughPoints == TRUE)
        {
          SmFace    * pFace   = (SmFace *)m_cpSurface[lSrf]->GetFace() ;
          SmExtent2d  sDomain = m_cpSurface[lSrf]->GetNaturalUVDomain() ;

          SmTArray<SmEdgeuse *> sEdgeuses ;
          pFace->GetUpwardEdgeuses(sEdgeuses) ;

          // for every Surface->Face->Edge
          for(ULONG ii=0;ii<sEdgeuses.GetSize();ii++)
            {
              // edgeuse locals
              SmEdgeuse      * pEdgeuse     = sEdgeuses[ii] ;   
              SmEdge         * pEdge        = pEdgeuse->GetEdge() ;
              SmCurve        * pCurve       = pEdge->GetCurve() ;
              SmExtent1d       sIvl         = pEdge->GetInterval() ;
              SmBSplineCurve * pUVTrimCurve = NULL ;
              pEdgeuse->GetOrCreateUVTrimCurve(pUVTrimCurve) ;

              // UVTrimCurve BBox
              SmExtent3d sBBox ;
              pUVTrimCurve->CalculateBoundingBox(sIvl, &sBBox) ;

              // BBox corner values
              double sUVMinX = sBBox.GetMin().x ;
              double sUVMinY = sBBox.GetMin().y ;
              double sUVMaxX = sBBox.GetMax().x ;
              double sUVMaxY = sBBox.GetMax().y ;

              // Skip edges that are part of a boundary isoParameter Curve
              if(   (   SM_ARE_SAME(sUVMinX, sUVMaxX)
                     && (   SM_ARE_SAME(sUVMinX, sDomain.GetMin().x)
                         || SM_ARE_SAME(sUVMaxX, sDomain.GetMax().x)))
                 || (   SM_ARE_SAME(sUVMinY, sUVMaxY)
                     && (   SM_ARE_SAME(sUVMinY, sDomain.GetMin().y)
                         || SM_ARE_SAME(sUVMaxY, sDomain.GetMax().y))))
                { continue ; }

              // Build processing arrays - Curves, UVTrimCurves, and Intervals
              sTargetCurves.Add(pCurve) ;
              sUVTrimCurves.Add(pUVTrimCurve) ;
              sIntervals.   Add(sIvl) ;
              // old way
              //      sIntervals.SetSize(sIntervals.GetSize()+1) ;
              //      sIntervals[sIntervals.GetSize()-1] = sIvl ;
              
            } // end iter every Surface->Face->Edge
        } // end when asked and Surface Has Face - add face edges check


#ifdef SM_DEBUG_CODE
      // draw SurfaceCurve(red), Breps(blue,green), Surfaces(cyan,yellow), Faces(black)
      if (bDebugMe || lDebugCount == lCount) 
        {
          SmFace *pFace1 = (SmFace *)m_cpSurface[0]->GetFace() ;
          SmFace *pFace2 = (SmFace *)m_cpSurface[1]->GetFace() ;
          SmBrep *pBrep1 = (pFace1) ? pFace1->GetBrep() : NULL ;
          SmBrep *pBrep2 = (pFace2) ? pFace2->GetBrep() : NULL ;

          smgfx_Erase() ;
          smgfx_SetLook(1,2, 0,0,1) ; if(pBrep1) { pBrep1->Draw(TRUE) ; } sm_GraphicsLoop() ;
          smgfx_SetLook(1,2, 0,1,0) ; if(pBrep2) { pBrep2->Draw(TRUE) ; } sm_GraphicsLoop() ;
          smgfx_SetLook(4,6, 1,0,1) ; for(ULONG ii=0;ii < sTargetCurves.GetSize();ii++)
                                        { SmCurve *pTC = sTargetCurves[ii] ;
                                          pTC->Draw(&sIntervals[ii]); sm_GraphicsLoop();
                                        }
          smgfx_SetLook(1,2, 0,1,1) ; m_cpSurface[0]->DrawUV(6,6,FALSE,NULL,pBrep1==NULL) ; sm_GraphicsLoop() ;
          smgfx_SetLook(1,2, 0,0,0) ; if(pFace1) { pFace1->Draw(SM_DM_CROSSHATCH) ; }sm_GraphicsLoop() ;
          smgfx_SetLook(1,2, 1,1,0) ; m_cpSurface[1]->DrawUV(6,6,FALSE,NULL,pBrep2==NULL) ; sm_GraphicsLoop() ;
          smgfx_SetLook(1,2, 0,0,0) ; if(pFace2) { pFace2->Draw(SM_DM_CROSSHATCH) ; }sm_GraphicsLoop() ;
          sm_GraphicsLoop();
        }
#endif // SM_DEBUG_CODE

// end GWC:FACE_EDGES_CHANGE                                                      
      
      // for every TargetCurve - intersect ThisSurface->BoundaryCurves with OtherSurface                                                                           
      //       3     
      //       |     
      //     +----+    for all targetCurves  [0]    = constant min U Boundary Curve
      //     |    |                          [1]    = constant min V Boundary Curve
      //   0-|    |-2                        [2]    = constant max U Boundary Curve
      //     +----+                          [3]    = constant max V Boundary Curve
      //       |                             [4...] = Surface->Face->Edge-Curves (if any)
      //       1     
      for(ULONG lCrv=0;lCrv<sTargetCurves.GetSize();lCrv++)
        {
          // set dNormParam = 0.0 for min NaturalBoundaryCurves
          //     and LSide    1.0 for max NaturalBoundaryCurves
          //                          not used for surface->Face->Edge curves
          // set lCoord     = 0.0 for constant U direction NaturalBoundaryCurves
          //                  1.1 for constant V direction NaturalBoundaryCurves
          //                           not used for surface->Face->edge curves
          double dNormParam = lCrv < 2 ? 0.0 :  1.0 ;
          ULONG  lCoord     = lCrv % 2 ;
          ULONG  lSide      = lCrv < 2 ? 0 : 1 ;
          
          // Select the BoundarySurfaceCurve and interval for this iteration
          SmCurve    *pTgtBSC = sTargetCurves[lCrv]; NER(pTgtBSC);
          SmExtent1d  sBSCIvl = sIntervals[lCrv] ;

          SM_ASSERT(   (lCrv >= 4)
                    || (lCoord == 0 && (   m_vUVDomain[lSrf].GetMin().y == sBSCIvl.GetMin()
                                        && m_vUVDomain[lSrf].GetMax().y == sBSCIvl.GetMax()))
                    || (lCoord == 1 && (   m_vUVDomain[lSrf].GetMin().x == sBSCIvl.GetMin()
                                        && m_vUVDomain[lSrf].GetMax().x == sBSCIvl.GetMax()))) ;

          // locals
          SmSolution aData[20];
          SmSolutionArray sSolutions(20,aData);

#ifdef SM_DEBUG_CODE
          // draw NaturalBoundaryCurve/OtherSurface intersection inputs
          if (bDebugMe || lDebugCount == lCount) 
            {
              pTgtBSC->Dump();
              m_cpSurface[1-lSrf]->Dump();

              SmFace *pFace1 = (SmFace *)m_cpSurface[0]->GetFace() ;
              SmFace *pFace2 = (SmFace *)m_cpSurface[1]->GetFace() ;
              SmBrep *pBrep1 = (pFace1) ? pFace1->GetBrep() : NULL ;
              SmBrep *pBrep2 = (pFace2) ? pFace2->GetBrep() : NULL ;

              smgfx_Erase() ;
              smgfx_SetLook(1,2, 0,0,1) ; if(pBrep1) { pBrep1->Draw(TRUE) ; } sm_GraphicsLoop() ;
              smgfx_SetLook(1,2, 0,1,0) ; if(pBrep2) { pBrep2->Draw(TRUE) ; } sm_GraphicsLoop() ;
              smgfx_SetLook(5,6, 1,0,1) ; pTgtBSC->Draw(&sBSCIvl); sm_GraphicsLoop();
              smgfx_SetLook(1,2, 0,1,1) ; m_cpSurface[0]->DrawUV(6,6,FALSE,NULL,pBrep1==NULL) ; sm_GraphicsLoop() ;
              smgfx_SetLook(1,2, 1,1,0) ; m_cpSurface[1]->DrawUV(6,6,FALSE,NULL,pBrep2==NULL) ; sm_GraphicsLoop() ;
              sm_GraphicsLoop();
            }
#endif
          // skip degenerate TargetCurves
          if (pTgtBSC->IsDegenerate(SM_EFF_ZERO,&sBSCIvl)) 
             { continue; }
              
          // BoundaryCurve BBox + Tol
          SmExtent3d sBBox;
          SER(pTgtBSC->CalculateBoundingBox(sBSCIvl,&sBBox));
          sBBox.ExpandAbsolute(m_dThisApproxTol3d);      // GWC: ApproxTol used as GeomTol here
              
          // when intersection BBox intersects (not disjoint) BoundaryCurve BBox
          if (! sBBox.AreDisjoint(crIntersectionBox)) 
            {
              // temporarily turn off point validation
              SmSurface *pSurf = (SmSurface *)m_cpSurface[1-lSrf] ;
              SmTemporaryChangeValue<SmObject *> sChange(pSurf->m_pOwner, NULL) ;

              // intersect OtherSurface[index=1-lSrf] with ThisSurface[index=lSrf]->BoundaryCurve[lSide]
              SER(m_cpSurface[1-lSrf]->GlobalCurveIntersect( m_vUVDomain[1-lSrf],   // in : Domain of the surface to intersect          
                                                            *pTgtBSC,               // in : Curve to intersect with surface             
                                                             sBSCIvl,               // in : Interval of curve to intersect                
                                                             m_dThisApproxTol3d,    // in : 3D distance tolerance to use in intersection
                                                             sSolutions));          // out: Contains results of intersection            

#ifdef SM_DEBUG_CODE
              // draw NaturalBoundaryCurve/OtherSurface intersection results 
              if(bDebugMe || lDebugCount == lCount)
                {
                  sSolutions.Dump() ;

                  SmFace *pFace1 = (SmFace *)m_cpSurface[0]->GetFace() ;
                  SmFace *pFace2 = (SmFace *)m_cpSurface[1]->GetFace() ;
                  SmBrep *pBrep1 = (pFace1) ? pFace1->GetBrep() : NULL ;
                  SmBrep *pBrep2 = (pFace2) ? pFace2->GetBrep() : NULL ;

                  smgfx_Erase() ;
                  smgfx_SetLook(1,2, 0,0,1) ; if(pBrep1) { sChange.UseOrigValue() ; pBrep1->Draw(TRUE) ; sChange.UseTempValue() ; } sm_GraphicsLoop() ;
                  smgfx_SetLook(1,2, 0,1,0) ; if(pBrep2) { sChange.UseOrigValue() ; pBrep2->Draw(TRUE) ; sChange.UseTempValue() ; } sm_GraphicsLoop() ;
                  smgfx_SetLook(6,8, 1,0,0) ; sSolutions.Draw() ; sm_GraphicsLoop() ;
                  smgfx_SetLook(1,2, 0,1,1) ; m_cpSurface[0]->DrawUV(6,6,FALSE,NULL,pBrep1==NULL) ; sm_GraphicsLoop() ;
                  smgfx_SetLook(1,2, 0,0,0) ; if(pFace1) { sChange.UseOrigValue() ; pFace1->Draw(SM_DM_CROSSHATCH) ; sChange.UseTempValue() ; } sm_GraphicsLoop() ;
                  smgfx_SetLook(1,2, 1,1,0) ; m_cpSurface[1]->DrawUV(6,6,FALSE,NULL,pBrep2==NULL) ; sm_GraphicsLoop() ;
                  smgfx_SetLook(1,2, 0,0,0) ; if(pFace2) { sChange.UseOrigValue() ; pFace2->Draw(SM_DM_CROSSHATCH) ; sChange.UseTempValue() ; } sm_GraphicsLoop() ;
                  smgfx_SetLook(5,6, 1,0,1) ; pTgtBSC->Draw(&sBSCIvl); sm_GraphicsLoop();
                  sm_GraphicsLoop() ;
                }
#endif // SM_DEBUG_CODE

            } // end intersecting BoundingBox check

          // for every intersection solution
          for (ULONG i=0; i<sSolutions.GetSize(); i++) 
            {
              SmSolution & rSol  = sSolutions[i];

              // alloc and init a SmTsectPnt for this solution 
              SmTsectPnt * pTSP  = (SmTsectPnt*)m_vTSPntMgr.GetNewElement();
              pTSP->m_ePointType = SM_IP_UNKNOWN;

              // classify the solution 
              // add point solutions to m_vStartPoints
              // add degenerate touching point solutions and interval solutions to r3DCurves
              
              // For single intersection points
              if (rSol.m_eSolutionType == SM_ST_SINGLE_VALUE) 
                {
                  SmPoint2d sUVs[2] ; // xSect srf UVs for both srfs

                  // set otherSurface UVPoint
                  sUVs[1-lSrf].x = rSol.m_vStart[1];
                  sUVs[1-lSrf].y = rSol.m_vStart[2];

                  // set ThisSurface UVPoint - from Crv/Srf Solution and knowing which isoCurve is current tgt
                  if (lCrv < 4 && lCoord == 0) { // For constant U natural boundary curves
                                                 sUVs[lSrf].x = m_vUVDomain[lSrf].Evaluate(dNormParam,dNormParam).x;
                                                 sUVs[lSrf].y = rSol.m_vStart[0];
                                               }
                  else if(lCrv < 4 )           { // For constant V natural boundary curves
                                                 sUVs[lSrf].x = rSol.m_vStart[0];
                                                 sUVs[lSrf].y = m_vUVDomain[lSrf].Evaluate(dNormParam,dNormParam).y;
                                               }
                  else // only used when bGWCAddEdgeThroughPoints == TRUE
                    { // For Face Edges - get xSect point's ThisSurface->UVPoint 
                    
                      // Project Face->Edge->Curve->Point to Surface UVPoint.                        
                      //   sTargetCurves[lCrv] = Face->Edge->Curve
                      //   sUVTrimCurves[lCrv] = associated Face->Edge->UVTrimCurve for m_cpSurface[lSrf]  
                      //   sIntervals   [lCrv] = interval for both Curve and UVTrimCurve   

                      // Use TargetCurve to get 3DPoint
                      SmPoint3d sCurvePoint ;
                      sTargetCurves[lCrv]->EvaluatePoint(rSol.m_vStart[0], sCurvePoint) ;

                      // Use UVTrimCurve to get UVGuessPoint
                      SmPoint3d sUVGuess ;
                      sUVTrimCurves[lCrv]->EvaluatePoint(rSol.m_vStart[0], sUVGuess) ;

                      // drop CurvePoint onto Surface to find UVPoint
                      SmSolution sSol ;
                      SmBoolean bFoundAnswer ;
                      m_cpSurface[lSrf]->LocalPointSolve(m_vUVDomain[lSrf], 
                                                         SM_SO_MINIMIZE, 
                                                         sCurvePoint, 
                                                         SmPoint2d(sUVGuess.x, sUVGuess.y),
                                                         bFoundAnswer, 
                                                         sSol) ;
                      if(bFoundAnswer == FALSE)
                        {
                          SM_DBG_WARN(_T("GWC:Intersector StartPoint from Edge/Surface xsect failed")) ;
                          SM_DBG_WARN(_T("  if this fires - then new StartPoint extension needs to be reviewed")) ;

                          // skip this solution - we lose a startPoint
                          continue ;
                        }

                      // set UVPoint
                      sUVs[lSrf].x = sSol.m_vStart[0];
                      sUVs[lSrf].y = sSol.m_vStart[1];

                    } // end setting ThisSurface UVPoint for Face->Edge->UVTrimCurve StartPoints branch

                  // GWC:This call is redundant to the same call made in upcoming ComputePointValues() call.
                  //      // Set m_eUVParamType values for each UVpoint with respect to its surface's natural UVdomain
                  //      // This sets pTSP->m_eUVParamType (four values): Start, Inside, End...
                  //      pTSP->ClassifyPointParams(sUVs, m_cpSurface, m_vUVDomain, m_dThisApproxTol3d) ;  // GWC: ApproxTol used as param GeomTol here

                  // when processing a face->Edge->Curve 
                  if(lCrv >= 4) // (only used when bGWCAddEdgeThroughPoints == TRUE)
                    {   
                      // skip surface->Face->Edge/OtherSurface Intersection ThroughPoints on the SurfaceBoundary.
                      // they're already StartPoints.
                      if(   m_vUVDomain[lSrf]  .IsPoint2dOnBoundary(sUVs[lSrf])
                         || m_vUVDomain[1-lSrf].IsPoint2dOnBoundary(sUVs[1-lSrf]))
                        { continue ; }
                    }

                  // skip start points which are on surface singularities
                  SmSurfParamType eSingDir;
                  if (   m_cpSurface[  lSrf]->IsSingularity(sUVs[  lSrf],eSingDir)
                      || m_cpSurface[1-lSrf]->IsSingularity(sUVs[1-lSrf],eSingDir)) 
                    {
                      // The start point is on a surface singularity - just skip it
                      // GWC: what's the logic of this choice?  Intersection curves can easily
                      //      start and/or stop on surface singularity points (which by caveat are
                      //      limited to the boundaries of the surface definition).  These interesection
                      //      curves must be collected elsewhere in the algorithm, but where?
                      //      Perhaps the problem is that assigning a unique value to the singular parameter
                      //      is undoable with the information available in this algorithm
                      //      which causes the upcoming trace algorithm to not work. 
                      continue ;
                    }

                  // Compute xsect point values for given mated UVPoints including:
                  //     xSectPoint type: SM_IP_CROSSING,      - normal crossing intersection point
                  //                      SM_IP_TANGENT_POINT, - surfaces touch at point           
                  //                      SM_IP_TANGENT_CURVE, - point on tangent curve            
                  //                      SM_IP_SINGULARITY,   - point at nexus of 4 or 6 intersect curves     
                  //                      SM_IP_COINCIDENCE    - matching 2nd fundamental forms    
                  //     Surface 3D and UV point and derivative values 
                  //     3D XSectCurve Position and Tangent
                  // Tell ComputePointValues() that this is a starting point computation by setting 
                  //     the 3rd argument, pOptPreviousPnt = NULL

                  if(ComputePointValues(sUVs, *pTSP, NULL) != SM_SUCCESS) 
                    { continue; }

                  SM_ASSERT_BREAK(   (lCoord == 0 && lSide == 0 && pTSP->UParamType(lSrf) == SM_EP_START)
                                  || (lCoord == 0 && lSide == 1 && pTSP->UParamType(lSrf) == SM_EP_END)
                                  || (lCoord == 1 && lSide == 0 && pTSP->VParamType(lSrf) == SM_EP_START)
                                  || (lCoord == 1 && lSide == 1 && pTSP->VParamType(lSrf) == SM_EP_END) ) ;

        // GWC_NEEDS_WORK COULD_MODIFY_HERE_FOR_B254_pTSP_IS_A_SINGULARITY_INTERSECTION_POINT__CPV_could_alloc_and_return_a_2nd_pTSP GWC_LINE ;
        // The returning a 2nd pTSP idea is not good enough (it works only for singular points at the nexus of 4 intersection curves)
        // - I've thought of a case where one singularity intersection point is the nexus for 6 intersection curve start directions.
        //   Consider a cylinder and a torus sized and position such that they share a coincident circle.  Two additional
        //   intersection loops will start and stop at the point on the intersection circle that is inner most to the torus
        //   The intersection circle and the two intersection loops all start and stop at that point, creating a intersection
        //   singularity point at the nexus of 6 different intersection curves. I'm still trying to imagine a case that
        //   puts a singular intersection point at the nexus of 8 intersection directions because there's a comment
        //   saying such a case has been seen.

        // GWC_NEEDS_WORK OR_pTSP_COULD_STORE_A_2nd_Start_Dir GWC_LINE ; 
        // Adding multiple directions to the pTSP data structure idea is not good enough.
        // Mostly, this case is rare and augmenting the data structure of all pTSPs seems like overkill.
        // Additionally, now that we have a case of a singular point nexus with 6 xSect curves, the pTSP extension
        // would have to be more than storing just 2 directions (at least 3 will be needed.)
        // I think adding multiple pTSPs to the StartPoint array is better than changing the SmTsectPnt 
        // class to hold multiple directions. 

        // GWC_NEEDS_WORK OR_we_add_new_method_here_to_deal_explicitly_with_creating_multiple_start_points_for_Sinlgular_XSect_points GWC_LINE ;
        // if a single intersection point can end up being a singular intersection point nexus of multiple intersection
        // curves, then I think we should add one pTSP to the startPoint array for each direction.
        // If we do that, perhaps a new method could go here that generates the multiple pTSPs and adds them to the list
        // having the advantage that the step becomes explicit, stand alone and well documented. 

#ifdef SM_DEBUG_CODE
                  // draw 
                  if(bDebugMe)
                    {
                      sSolutions.Dump() ;
                      pTSP->Dump() ;

                      SmFace *pFace1 = (SmFace *)m_cpSurface[0]->GetFace() ;
                      SmFace *pFace2 = (SmFace *)m_cpSurface[1]->GetFace() ;
                      SmBrep *pBrep1 = (pFace1) ? pFace1->GetBrep() : NULL ;
                      SmBrep *pBrep2 = (pFace2) ? pFace2->GetBrep() : NULL ;

                      smgfx_Erase() ;
                      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep1) { pBrep1->Draw(TRUE) ; } sm_GraphicsLoop() ;
                      smgfx_SetLook(1,2, 0,1,0) ; if(pBrep2) { pBrep2->Draw(TRUE) ; } sm_GraphicsLoop() ;
                      smgfx_SetLook(1,2, 0,1,1) ; m_cpSurface[0]->DrawUV(6,6,FALSE,NULL,pBrep1==NULL) ; sm_GraphicsLoop() ;
                      smgfx_SetLook(1,2, 1,1,0) ; m_cpSurface[1]->DrawUV(6,6,FALSE,NULL,pBrep2==NULL) ; sm_GraphicsLoop() ;
                      smgfx_SetLook(3,4, 1,0,1) ; pTgtBSC->Draw(&sBSCIvl); sm_GraphicsLoop();
                      smgfx_SetLook(5,6, 1,0,1) ; m_cpSurface[0]->DrawAt(sUVs[0]) ; sm_GraphicsLoop() ;
                      smgfx_SetLook(5,6, 1,0,1) ; m_cpSurface[1]->DrawAt(sUVs[1]) ; sm_GraphicsLoop() ;
                      smgfx_SetLook(7,8, 1,0,0) ; sSolutions.Draw() ; sm_GraphicsLoop() ;
                      smgfx_SetLook(5,7, 0,0,0) ; pTSP->Draw() ; sm_GraphicsLoop() ;
                      sm_GraphicsLoop() ;
                    }
#endif // SM_DEBUG_CODE

                  // Check quality of face->edge/srf XSect found ThroughPoints
                  if(lCrv >= 4)  // only used when bGWCAddEdgeThroughPoints == TRUE
                    {
                      // get the surf/surf point gap
                      SmVector3d sGapVector  = pTSP->SrfPos(0) - pTSP->SrfPos(1) ;
                      double     dGapSize    = sGapVector.Length() ;
                      double     dScaledZero = SM_EFF_ZERO * (1.0 + pTSP->SrfPos(0).GetMaxDimension()) ;

                      // when gap is larger than scaled zero
                      if(dGapSize > dScaledZero)
                        {
                          // Move ThroughPoint to nearby surf/surf xSect 
                          SmBoolean bFoundPoint ;
                          SER(RefinePoint(sUVs[0], sUVs[1], pTSP->m_ePointType,
                                          &pTSP->CrvDeriv(), NULL, 
                                          FALSE, bFoundPoint, sUVs)) ; 
                        
                          // don't use this point if it doesn't refine
                          if(bFoundPoint == FALSE)
                            { continue ; }
                                          
                          // update xsect point values
#ifdef SM_DEBUG_CODE
                          SmTsectPnt sTSP0(*pTSP) ;
#endif // SM_DEBUG_CODE
                          // tell ComputePointValues that this is a start point by setting pOptPreviousPnt = NULL
                          if(ComputePointValues(sUVs, *pTSP, NULL) != SM_SUCCESS)
                            { continue ; }

#ifdef SM_DEBUG_CODE
                          SmVector3d sGap2Vector  = pTSP->SrfPos(0) - pTSP->SrfPos(1) ;
                          SmVector3d sJump1Vector = sTSP0.SrfPos(0) - pTSP->SrfPos(0) ;
                          SmVector3d sJump2Vector = sTSP0.SrfPos(1) - pTSP->SrfPos(1) ;
                          double     dGap2Size    = sGap2Vector.Length() ;
                          double     dJump1Size   = sJump1Vector.Length() ;
                          double     dJump2Size   = sJump2Vector.Length() ;
                          double     dScaled2Zero = SM_EFF_ZERO * (1.0 + pTSP->SrfPos(0).GetMaxDimension()) ;
                          SM_ASSERT(dGap2Size < dScaled2Zero) ;
                          SM_ASSERT(dJump1Size < dGapSize * 1.1) ;
                          SM_ASSERT(dJump2Size < dGapSize * 1.1) ;
#endif // SM_DEBUG_CODE
                        } // end gap bigger than dScaledZero check
                    } // end have a throughpoint to verify check (only used when bGWCAddEdgeThroughPoints == TRUE)

// begin GWC:CHANGE_CLASSIFY_UPDATE

                  // compute the isoParamCurve's Position and Tangent
                  SmPoint3d sPT[2] ;
                  pTgtBSC->Evaluate(rSol.m_vStart[0], 1, TRUE, sPT) ;

                  // look for tangent surface/surfaceBoundaryCurve tangent intersections
                  // that are point intersections.
                  SmBoolean bIsDegenerate = FALSE ;
                  if(lCrv < 4)
                    { // Set bIsDegenerate = TRUE: pTSP is a degenerate pt XSect curve
                      //                    FALSE: pTSP is part of XSect curve and should be traced.
                      ClassifyBoundaryStartPointIntersection(*pTSP, bIsDegenerate) ;
                    }

                  // when the IntersectionPoint is degenerate (not part of an intersection curve)
                  if(bIsDegenerate)
                    {  
                      // when point is not already on an intersection curve
                      if( !IsPointOnCurve( pTSP->CrvPos(), r3DCurves ) )
                        {
                          // add point to degeneratePoint list.
                          //  Later a 3d,uv1,uv2 tuple of degenerate curves are built for each unique degenerate
                          //  point and added to the r3DCurves, rSurface1UVCurves, rSurface2UVCurves output arrays. 
                          sDegeneratePoints.Add(pTSP) ;
                          s3DPoints.Add(sPT[0]) ;
                          sSurfIndex.Add(1-lSrf) ;

                          // move on to next solution
                          continue ;
                        } // end not a duplicate point check
                    } // end Is DegenerateIntersectionPoint check

// end GWC:CHANGE_CLASSIFY_UPDATE

                  // if the point is not already on one of the intersection curves
                  // GWC_NEEDS_WORK singular_points_could_have_multiple_directions__This_check_could_be_pos_and_dir_dependent GWC_LINE ;
                  if ( !IsPointOnCurve( pTSP->CrvPos(), r3DCurves ) )
                    {
                      // save point to the sEdgeCrossings, m_vStartPoints, or m_vThroughPoints xsect points array
                      pTSP->m_dDeviation = rSol.m_vStart.m_dSolutionValue;
                      if(lCrv >= 4) 
                        { 
                          AddThroughPoint(*pTSP) ;
                        }
                      else if (m_vUVDomain[1-lSrf].IsPoint2dOnBoundary(sUVs[1-lSrf],SM_EFF_ZERO*100.0))
                        {
                          // later, sEdgeCrossing pts are added last to the m_vStartPoints list
                          // so that they are the first ones traced - this helps to get the longest trace possible
                          // first and can save tracing the xSect curve out in small overlapping parts.
                          sEdgeCrossings.Add(pTSP);
                        }
                      else 
                        {
                          // Add unique TsectPnts to the m_vStartPoints list
                          AddStartPoint(*pTSP);
                        }
                    }
                } // end single point intersection check

              // when intersection is a NaturalBoundaryCurve range of values 
              if (   rSol.m_eSolutionType == SM_ST_RANGE_OF_VALUES
                  && lCrv < 4 ) // skip edge curves
                {
                  // Here we need to add some additional logic to
                  // process coincident curves.  Small curves we can
                  // just use the start points.  Large curves we can
                  // use the curves as intersection curves.  We then
                  // need to classify the curve as either a crossing
                  // intersection, tangent intersection, or surface
                  // coincidence boundary.
                  double dSize    = rSol.m_vEnd[0] - rSol.m_vStart[0];
                  double dIvlSize = sBSCIvl.GetMax() - sBSCIvl.GetMin();

                  // Here we need to apply a little heuristics - which is
                  // a very dangerous thing.  The heuristic is this - 
                  // if the interval returned is greater than 1/100th of
                  // the original we will consider it an intersection
                  // curve.  If it is smaller then we will just consider 
                  // that it is used to get start points.  Note that 
                  // we could apply some distance measurement to aid in
                  // our testing.
                  // Now we have added a check to prevent finding coincident
                  // curves where the differences between the distances is
                  // greater than 0.1
//                        double dDistRatio = smos_Fabs(rSol.m_vStart.m_dSolutionValue -
//                            rSol.m_vEnd.m_dSolutionValue) / 
//                            (rSol.m_vStart.m_dSolutionValue + rSol.m_vEnd.m_dSolutionValue
//                            + SM_EFF_ZERO * 100.00);
                  if (dSize > dIvlSize/100.0 /*&& dDistRation > 0.1*/) 
                    {
                      // prepare for coincident maybe check
                      if (rSol.m_vEnd.m_dSolutionValue < SM_EFF_ZERO) 
                        {
                          rSol.m_vEnd.m_dSolutionValue = SM_EFF_ZERO;
                        }
                      if (rSol.m_vStart.m_dSolutionValue < SM_EFF_ZERO) 
                        {
                          rSol.m_vStart.m_dSolutionValue = SM_EFF_ZERO;
                        }
                      double dSizeRatio = rSol.m_vStart.m_dSolutionValue / rSol.m_vEnd.m_dSolutionValue;

                      // Filter out coincident maybe type cases
                      if (dSizeRatio > 100.00 || dSizeRatio < 0.01) continue; 

                      // get a curve sample point to test against existing xsect curves
                      double dTestParam = (rSol.m_vStart[0] + rSol.m_vEnd[0]) / 2.0;
                      SmPoint3d sTestPnt;
                      SER(pTgtBSC->EvaluatePoint(dTestParam,sTestPnt));

                      // If sample point is on an existing intersection curve - skip this curve.
                      if (IsPointOnCurve(sTestPnt,r3DCurves)) 
                        {
                          continue;
                        }

                      // create output curve trimmed to the intersection interval
                      SmBSplineCurve *pBSCResult = NULL ;
                      if(pTgtBSC->IsKindOf(SmBSplineCurve_TYPE))
                        {
                          pBSCResult = new (*m_cpContext) SmBSplineCurve((SmBSplineCurve &)*pTgtBSC);
                        }
                      else
                        {
                          double dAchievedTolerance ;
                          SmTArray<double> sBreakParameters ;
                          sBreakParameters.Add(rSol.m_vStart[0]) ;
                          sBreakParameters.Add(rSol.m_vEnd[0]) ;
                          pTgtBSC->ApproximateCurve(*m_cpContext, sBreakParameters,
                                                    m_dThisApproxTol3d/3.0,
                                                    dAchievedTolerance, pBSCResult,
                                                    TRUE,    // in : bOptCreateAnalytics      
                                                    FALSE,   // in : bOptMatchParameterization
                                                    FALSE) ; // in : bJustCopyBSplines        

                        }
                          
                      NER(pBSCResult);
                      SmObjDelete sClean3D(pBSCResult);
                      SmExtent1d sTrimIvl(rSol.m_vStart[0],rSol.m_vEnd[0]);
                      SER(pBSCResult->Trim(sTrimIvl));   // may snap sIvl by tol to existing knots
                      sClean3D.Clear();

                      // add the curve to the intersection curve result list
                      r3DCurves.Add(pBSCResult);

                      double dDistToSurf1, dDistToSurf2, dDeviation;

                      // drop curve to Surface0 to build UVTrimCurve
                      SmTArray<SmBSplineCurve*> sUVCurves;
                      SER(m_cpSurface[0]->DropAndTrimCurve(*m_cpContext,
                                                           m_vUVDomain[0],
                                                           *pBSCResult,
                                                           pBSCResult->GetNaturalInterval(),
                                                           m_dThisApproxTol3d,
                                                           dDistToSurf1,
                                                           dDeviation, 
                                                           sUVCurves, 
                                                           TRUE, 
                                                           TRUE ));

                      // drop curve case: built two UVTrimCurves - trim curve on seam of closed surface
                      if (sUVCurves.GetSize() == 2) 
                        {
                          SM_ASSERT(lCrv < 4) ;
                          rSurface1UVCurves.Add(sUVCurves[lSide]);
                          delete sUVCurves[1-lSide]; sUVCurves[1-lSide] = NULL ;
                        }
                      // drop curve case: built one UVTrimCurve
                      else if (sUVCurves.GetSize() == 1) 
                        {
                          rSurface1UVCurves.Add(sUVCurves[0]); 
                        }
                      // drop curve case: default - something is wrong - don't use this curve
                      else 
                        {
                          r3DCurves.RemoveLast();
                          SmObjsDelete<SmBSplineCurve*> sCleanUVCrvs( &sUVCurves );
                          continue;
                        }

                      // Back trim 3D curve if needed.
                      SmBSplineCurve *pUVBSC = SM_CAST_PTR(SmBSplineCurve,rSurface1UVCurves.GetLast());
                      SmExtent1d sUVIvl = pUVBSC->GetNaturalInterval();
                      if (!sTrimIvl.IsContainedBy(sUVIvl)) 
                        {
                          sTrimIvl = sUVIvl;
                          SER(pBSCResult->Trim(sTrimIvl));  // may snap sIvl by tol to existing knots
                        }

                      // drop curve to Surface1 to build UVTrimCurve
                      SER(m_cpSurface[1]->DropAndTrimCurve(*m_cpContext,
                                                           m_vUVDomain[1],
                                                           *pBSCResult,
                                                           pBSCResult->GetNaturalInterval(),
                                                           m_dThisApproxTol3d, 
                                                           dDistToSurf2,
                                                           dDeviation, 
                                                           sUVCurves, 
                                                           TRUE, 
                                                           TRUE ));

                      // drop curve case: built 2 UVTrimCurves - trim curve on boundary of closed surface
                      if (sUVCurves.GetSize() == 2) 
                        {
                          SM_ASSERT(lCrv < 4) ;
                          rSurface2UVCurves.Add(sUVCurves[lSide]); 
                          delete sUVCurves[1-lSide]; sUVCurves[1-lSide] = NULL ;
                        }
                      // drop curve case: built 1 UVTrimCurve
                      else if (sUVCurves.GetSize() == 1) 
                        {
                          rSurface2UVCurves.Add(sUVCurves[0]); 
                        }
                      else // drop curve case: default - something is wrong - don't use this curve
                        {
                          r3DCurves.RemoveLast();
                          SM_ASSERT(rSurface1UVCurves.GetLast() != NULL) ; delete rSurface1UVCurves.GetLast() ; 
                          rSurface1UVCurves.RemoveLast();
                          SmObjsDelete<SmBSplineCurve*> sCleanUVCrvs( &sUVCurves );
                          continue;
                        }

                      // Back trim 3D curve if needed.
                      SmBSplineCurve *pUVBSC2 = SM_CAST_PTR(SmBSplineCurve,rSurface2UVCurves.GetLast());
                      SmExtent1d sUVIvl2 = pUVBSC2->GetNaturalInterval();
                      if (!sTrimIvl.IsContainedBy(sUVIvl2)) 
                        {
                          sTrimIvl = sUVIvl2;
                          SER(pBSCResult->Trim(sTrimIvl)); // may snap sIvl by tol to existing knots
                          SmBSplineCurve *pUVBSC1 = SM_CAST_PTR(SmBSplineCurve,rSurface1UVCurves.GetLast());
                          SER(pUVBSC1->Trim(sTrimIvl));   // may snap sIvl by tol to existing knots
                        }
                      
                      sClean3D.Clear();

                      // If we are not very close to surface then skip this intersection.
                      if (   dDistToSurf1 > m_dThisApproxTol3d 
                          || dDistToSurf2 > m_dThisApproxTol3d ) 
                        {
                          SM_ASSERT(rSurface1UVCurves.GetLast() != NULL) ; delete rSurface1UVCurves.GetLast() ; 
                          rSurface1UVCurves.RemoveLast();
                          SM_ASSERT(rSurface2UVCurves.GetLast() != NULL) ; delete rSurface2UVCurves.GetLast() ;
                          rSurface2UVCurves.RemoveLast();
                          SM_ASSERT(r3DCurves.GetLast() != NULL) ; delete r3DCurves.GetLast() ;
                          r3DCurves.RemoveLast();
                        } // end don't use this curve branch
                      else // use this curve
                        { // set the curve deviation and type
                          rDeviations.Add(smos_Max(dDistToSurf1,dDistToSurf2));
                          SmTsectCurveType eCurveType = sm_ComputeCurveType(
                              m_cpSurface[0],m_cpSurface[1],rSurface1UVCurves.GetLast(),
                              rSurface2UVCurves.GetLast());
                          if (eCurveType == SM_TC_COINCIDENT || eCurveType == SM_TC_REGION_BOUNDARY)
                            {
                              m_bCheckForInteriorCurves = FALSE;
                            }
                          rCurveTypes.Add(eCurveType);

                          // GWC:CLEAN_STARTPOINT_LIST_CHANGE Jan 2005
                          // Remove startPoints on this curve to prevent finding the same curve twice
                          SmTsectPnt *pNextPt ;
                          SmTsectPnt *pCurPt = m_vStartPoints.GetFirstNode();

                          // for every StartPoint
                          for(;pCurPt;pCurPt=pNextPt)
                            {
                              // get next StartPoint
                              pNextPt =  (SmTsectPnt *)(  (pCurPt->m_pNext == m_vStartPoints.GetFirstNode()) 
                                                        ? NULL 
                                                        : pCurPt->m_pNext) ;

                              // when pCurPt is on one of the r3DCurves
                              SM_ASSERT(m_vStartPoints.GetFirstNode() != NULL) ;
                              if ( IsPointOnCurve( pCurPt->CrvPos(), r3DCurves ) )
                                { // remove it
                                  m_vStartPoints.Remove(pCurPt) ;
                                } // end point on curve check
                            } // end iter every start point

                           // for every sEdgeCrossings Point
                           for(int j=0;j<(int)sEdgeCrossings.GetSize();j++)
                             {
                               // when jth Pt is on one of the r3DCurves
                               if ( IsPointOnCurve( sEdgeCrossings[(ULONG)j]->CrvPos(), r3DCurves ))
                                { // remove it
                                  sEdgeCrossings.RemoveAt((ULONG)j) ;
                                  j-- ;
                                } // end point on curve check
                             } // end iter every sEdgeCrossings Point

                           // end GWC:CLEAN_STARTPOINT_LIST_CHANGE

                       } // end use this curve branch
                    } // end large intersection branch

                  else // Small intersection - just use as start points  
                    {
                      // Check for a perpendicular point inside of the segment
                      // denoting a local minima.
                      SmBoolean  bFoundSolution;
                      SmSolution sSolution;
                      double     dT = rSol.m_vStart[0];
                      SmPoint2d  sUV(rSol.m_vStart[1],rSol.m_vStart[2]);
                      SmExtent1d sUVIvl2(rSol.m_vStart[0],rSol.m_vEnd[0]);

                      // Expand the interval by a little
                      sUVIvl2.ExpandAbsolute(dIvlSize*SM_EFF_ZERO_SQRT);
                      sUVIvl2.Intersect(pTgtBSC->GetNaturalInterval(),sUVIvl2);

                      // Swap start and end only when sUV is not on the boundary and
                      // the end has better deviation
                      if (   rSol.m_vStart.m_dSolutionValue > rSol.m_vEnd.m_dSolutionValue
                          && !m_vUVDomain[1-lSrf].IsPoint2dOnBoundary(sUV)) 
                        {
                          rSol.m_vStart = rSol.m_vEnd;
                        }

                      // use closest surface/curve point if found
                      SER(m_cpSurface[1-lSrf]->LocalCurveSolve(m_vUVDomain[1-lSrf],
                                                               *pTgtBSC,
                                                               sUVIvl2,
                                                               SM_SO_NORMALIZE,
                                                               m_dThisApproxTol3d,
                                                               NULL,
                                                               NULL,
                                                               sUV,
                                                               dT,
                                                               bFoundSolution,
                                                               sSolution));
                      if (  bFoundSolution 
                          && sSolution.m_vStart.m_dSolutionValue < rSol.m_vStart.m_dSolutionValue) 
                        {
                          rSol = sSolution;
                        }

                      // get intersection surfaceUVPoints from Curve/Surface xSect Solution
                      SmPoint2d sUVs[2];
                      if (lCrv < 4 && lCoord == 0) 
                        { // Processing constant U curves
                          sUVs[lSrf].x = m_vUVDomain[lSrf].Evaluate(dNormParam,dNormParam).x;
                          sUVs[lSrf].y = rSol.m_vStart[0];
                        }
                      else if(lCrv < 4 )
                        { // Processing constant V curves
                          sUVs[lSrf].y = m_vUVDomain[lSrf].Evaluate(dNormParam,dNormParam).y;
                          sUVs[lSrf].x = rSol.m_vStart[0];
                        }
                      else// project Curve Point to Surface to find UVPoint
                        {
                          // Use TargetCurve to get 3DPoint
                          SmPoint3d sPoint ;
                          sTargetCurves[lCrv]->EvaluatePoint(rSol.m_vStart[0], sPoint) ;

                          // Use UVTrimCurve to get UVGuessPoint
                          SmPoint3d s3dGuess ;
                          sUVTrimCurves[lCrv]->EvaluatePoint(rSol.m_vStart[0], s3dGuess) ;

                          // drop 3dPoint onto Surface to find UVPoint
                          SmSolution sSol ;
                          SmBoolean bFoundAnswer ;
                          m_cpSurface[lSrf]->LocalPointSolve(m_vUVDomain[lSrf], SM_SO_MINIMIZE, 
                                                             sPoint, SmPoint2d(s3dGuess.x,s3dGuess.y),
                                                             bFoundAnswer, sSol) ;
                          if(bFoundAnswer == FALSE)
                            {
                              SM_DBG_WARN(_T("GWC:Intersector StartPoint from Edge/Surface xsect failed")) ;
                              SM_DBG_WARN(_T("  if this fires - then new StartPoint extension needs to be reviewed")) ;

                              // skip this solution - we lose a startPoint
                              continue ;
                            }

                          // set UVPoint
                          sUVs[lSrf].x = sSol.m_vStart[0];
                          sUVs[lSrf].y = sSol.m_vStart[1];
                        } // end Project CurvePoint to Surface to find UVPoint branch

                      sUVs[1-lSrf].x = rSol.m_vStart[1];
                      sUVs[1-lSrf].y = rSol.m_vStart[2];

                      // skip start points which are on surface singularities
                      SmSurfParamType eSingDir;
                      if (   m_cpSurface[0]->IsSingularity(sUVs[0],eSingDir)
                          || m_cpSurface[1]->IsSingularity(sUVs[1],eSingDir)) 
                        {
                          // The start point is a singularity just skip it
                          continue;
                        }
                          
                      // Compute xsect point values including for given mated UVPoints:
                      //     xSectPoint type: SM_IP_CROSSING,      - normal crossing intersection point
                      //                      SM_IP_TANGENT_POINT, - surfaces touch at point           
                      //                      SM_IP_TANGENT_CURVE, - point on tangent curve            
                      //                      SM_IP_SINGULARITY,   - point at nexus of 4 or 6 intersect curves     
                      //                      SM_IP_COINCIDENCE    - matching 2nd fundamental forms    
                      //     Surface 3D and UV point and derivative values 
                      //     3D XSectCurve Position and Tangent
                      // Tell ComputePointValues that this is a start point by setting pOptPreviousPnt = NULL
                      if (ComputePointValues(sUVs, *pTSP, NULL) == SM_SUCCESS) 
                        {
// begin GWC:CHANGE_CLASSIFY_UPDATE
                          // compute the isoParamCurve's Position and Tangent
                          SmPoint3d sPT[2] ;
                          pTgtBSC->Evaluate(rSol.m_vStart[0], 1, TRUE, sPT) ;

                          // look for tangent surface/surfaceBoundaryCurve tangent intersections
                          // that are point intersections.
                          SmBoolean bIsDegenerate = FALSE ;
                          if(lCrv < 4)
                            { ClassifyBoundaryStartPointIntersection(*pTSP, bIsDegenerate) ;
                            }

                          // when the IntersectionPoint is degenerate (not part of an intersection curve)
                          if(bIsDegenerate)
                            {  
                              // when point is not already on an intersection curve
                              if( !IsPointOnCurve( pTSP->CrvPos(), r3DCurves ))
                                {
                                  // add point to degeneratePoint list
                                  sDegeneratePoints.Add(pTSP) ;
                                  s3DPoints.Add(sPT[0]) ;
                                  sSurfIndex.Add(1-lSrf) ;

                                  // move on to next solution
                                  continue ;
                                } // end not a duplicate point check
                            } // end Is DegenerateIntersectionPoint check

// end GWC:CHANGE_CLASSIFY_UPDATE

                          // if the point is not already on one of the intersection curves 
                          if ( !IsPointOnCurve( pTSP->CrvPos(), r3DCurves ))
                            {
                              // save point to the sEdgeCrossings, m_vStartPoints, or m_vThroughPoints xsect points array
                               pTSP->m_dDeviation = rSol.m_vStart.m_dSolutionValue;
                              if(lCrv >= 4)
                                {
                                  AddThroughPoint(*pTSP);
                                }
                              else if(m_vUVDomain[1-lSrf].IsPoint2dOnBoundary(sUVs[1-lSrf],SM_EFF_ZERO*100.0))
                                {
                                  sEdgeCrossings.Add(pTSP);
                                }
                              else 
                                {
                                  // Add unique TsectPnts to the m_vStartPoints list
                                  AddStartPoint(*pTSP);
                                }
                            }
                        }
                  } // end small intersection branch
              } // end SolutionType == SM_ST_RANGE_OF_VALUES check
          } // end iter every Surface/boundaryCurve solution
      } // end iter every Surface->Curve
  } // end iter each surface - lSrf

  // Now add edge crossings so that they are last on the
  // m_vStartPoints list but go first in the intersection process.
  for(ULONG ii=0; ii<sEdgeCrossings.GetSize(); ii++) 
    {
      // Add unique TsectPnts to the m_vStartPoints list
      AddStartPoint(*sEdgeCrossings[ii]);
    }

  // for every degenerate point 
  for(ULONG ii=0; ii<sDegeneratePoints.GetSize(); ii++)
    {
      // let degeneratePoint = Avg(CurvePoint,SurfacePoint)
      SmTsectPnt * pTSP             = sDegeneratePoints[ii] ;
      SmPoint3d    sDegeneratePoint = (s3DPoints[ii] + pTSP->SrfPos(sSurfIndex[ii]))/2.0 ;

      // skip Points already on an intersection curve or start point (due to seams)
      if (IsPointOnCurve(sDegeneratePoint,r3DCurves) || IsPointOnStartPoint(*pTSP, m_vStartPoints) )
        { continue ; }

      // GWC: potential method to avoid creating then removing degenerate solutions needs to be verified
      //      // skip points already on a StartPoint
      //      if (IsPointOnStartPoint(*pTSP, m_vStartPoints))
      //        { continue ; }

      // add degenerate 3d curve with UVTrimCurves to output 

      // Make and add 3D DegenerateCurves
      SmBSplineCurve * pNewBSP = NULL ;
      SER(SmBSplineCurve::CreateDegenerateCurve(*m_cpContext, 3,
                                                sDegeneratePoint,
                                                pNewBSP));
      r3DCurves.Add(pNewBSP);

      // Now for the first UV degenerate curve
      SER(SmBSplineCurve::CreateDegenerateCurve(*m_cpContext, 2,
                                                pTSP->UVPos(0),
                                                pNewBSP));
      rSurface1UVCurves.Add(pNewBSP);

      // Now for the second UV degenerate curve
      SER(SmBSplineCurve::CreateDegenerateCurve(*m_cpContext, 2,
                                                pTSP->UVPos(1),
                                                pNewBSP));
      rSurface2UVCurves.Add(pNewBSP);

      // set curve output dist and type parameters
      double dDist = pTSP->SrfPos(0).DistanceBetween(pTSP->SrfPos(1));
      rDeviations.Add(dDist);
      rCurveTypes.Add(SM_TC_TOUCHING);

    } // end iter every degeneratePoint

#ifdef SM_DEBUG_CODE
  if (bDebugMe2 || bDebugMe || lDebugCount == lCount) 
    {
      SmTArray<SmTsectPnt*> sStartPoints, sThroughPoints;
      m_vStartPoints.GetAllNodes(sStartPoints);
      m_vThroughPoints.GetAllNodes(sThroughPoints);

      // draw Brep(blue,green), Surfaces(cyan,yellow), faces(black), StartPoints(red)
      SmFace *pFace1 = (SmFace *)m_cpSurface[0]->GetFace() ;
      SmFace *pFace2 = (SmFace *)m_cpSurface[1]->GetFace() ;
      SmBrep *pBrep1 = (pFace1) ? pFace1->GetBrep() : NULL ;
      SmBrep *pBrep2 = (pFace2) ? pFace2->GetBrep() : NULL ;

      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep1) { pBrep1->Draw(TRUE) ; } sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,0) ; if(pBrep2) { pBrep2->Draw(TRUE) ; } sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,1) ; m_cpSurface[0]->DrawUV(6,6,FALSE,NULL,pBrep1==NULL) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,0,0) ; if(pFace1) { pFace1->Draw(SM_DM_CROSSHATCH) ; } sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 1,1,0) ; m_cpSurface[1]->DrawUV(6,6,FALSE,NULL,pBrep2==NULL) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,0,0) ; if(pFace2) { pFace2->Draw(SM_DM_CROSSHATCH) ; } sm_GraphicsLoop() ;
      ULONG i ;
      for (i=0; i<sStartPoints.GetSize(); i++) 
        {
          SmTsectPnt *pTSP = sStartPoints[i];
          SmPoint3d sP1, sP2;
          SER(m_cpSurface[0]->EvaluatePoint(pTSP->UVPos(0),sP1));
          SER(m_cpSurface[1]->EvaluatePoint(pTSP->UVPos(1),sP2));

          smos_WriteBuffer(_T("\nStart Point Pair - ")); sP1.Dump();
          smos_WriteBuffer(_T("\n                   ")); sP2.Dump(); 
          
          smgfx_SetLook(3,6, 1,0,0); sP1.Draw(); sm_GraphicsLoop() ;
          smgfx_SetLook(5,8, 0,1,0); sP2.Draw(); sm_GraphicsLoop() ;
        }
      for (i=0; i<sThroughPoints.GetSize(); i++) 
        {
          SmTsectPnt *pTSP = sThroughPoints[i];
          SmPoint3d sP1, sP2;
          SER(m_cpSurface[0]->EvaluatePoint(pTSP->UVPos(0),sP1));
          SER(m_cpSurface[1]->EvaluatePoint(pTSP->UVPos(1),sP2));
          smgfx_SetLook(3,6, 0,0,1); sP1.Draw(); sm_GraphicsLoop() ;
          smgfx_SetLook(5,8, 1,0,0); sP2.Draw(); sm_GraphicsLoop() ;
        }
      for (i=0; i<sDegeneratePoints.GetSize(); i++) 
        {
          SmTsectPnt *pTSP = sDegeneratePoints[i];
          SmPoint3d sP1, sP2;
          SER(m_cpSurface[0]->EvaluatePoint(pTSP->UVPos(0),sP1));
          SER(m_cpSurface[1]->EvaluatePoint(pTSP->UVPos(1),sP2));
          smgfx_SetLook(3,6, 0,1,0); sP1.Draw(); sm_GraphicsLoop() ;
          smgfx_SetLook(5,8, 1,0,0); sP2.Draw(); sm_GraphicsLoop() ;
        }
      sm_GraphicsLoop();
    }
#endif

  return SM_SUCCESS;

} // end SmSurfaceIntersector::FindBoundaryStartPoints

/*******************************************************************//**
PURPOSE: Process the case where the two surfaces have a bounding box
    which have a dimensionality of less than 3 (one or more intervals are
    very small - less than 2x sApproxTol3d).  

NOTES: 
***********************************************************************/
SmStatus SmSurfaceIntersector::GrazingBoxProcess
  (const SmContext    & crContext,                    // in : context for new object construction
   const SmExtent3d   & crIntersectionBBox,           // in : Bounding Box containing intersection
   const SmExtent2d   & crUVDomain1,                  // in : Surface0 UVDomain containing intersection
   const SmExtent2d   & crUVDomain2,                  // in : Surface1 UVDomain containing intersection
   SmBoolean          & rbFurtherWorkNeeded,          // out: TRUE = Send to general Surf/Surf intersector
                                                      //      FALSE= All done - no intersections
   SmTArray<SmCurve*> * p3DCurves,                    // out: 3D intersection curves
   SmTArray<SmCurve*> * pSurface1UVCurves,            // out: associated Surface1 Curve
   SmTArray<SmCurve*> * pSurface2UVCurves,            // out: Associated Surface2 Curve
   SmTArray<SmTsectCurveType> *pCurveTypes,           // out: associated surf/surf intersection type
   SmTArray<double>   * pDeviations)                  // out: intersection deviation
{                                                   
  // init output
  rbFurtherWorkNeeded = TRUE;

  // locals
  double dApproxTol3d = m_dThisApproxTol3d;
  SmVector3d sBBoxSize = crIntersectionBBox.GetSize() ;
  //ULONG lBBoxDim =   (sBBoxSize.x < dApproxTol3d ? 0 : 1)
  //                 + (sBBoxSize.y < dApproxTol3d ? 0 : 1)
  //                 + (sBBoxSize.z < dApproxTol3d ? 0 : 1) ;

  // for all 4 Surface1->UVcorners of the crUVDomain1
  for (ULONG iu1=0; iu1<2; iu1++) 
   for (ULONG iv1=0; iv1<2; iv1++) 
    {
      // skip UVcorner[iu1,uv1] not contained by crUVDomain1
      SmPoint2d sUV1 = m_vUVDomain[0].Evaluate(iu1*1.0,iv1*1.0);
      if (!crUVDomain1.ContainsPoint2d(sUV1)) continue;

      // for all 4 Surface2->UVCorners of the crUVDomain2
      for (ULONG iu2=0; iu2<2; iu2++) 
       for (ULONG iv2=0; iv2<2; iv2++) 
        {
           // skip UVcorner[iu2,iv2] not contained by crUVDomain2
           SmPoint2d sUV2 = m_vUVDomain[1].Evaluate(iu2*1.0,iv2*1.0);
           if (!crUVDomain2.ContainsPoint2d(sUV2)) continue;

           // If reach here we have a point which is inside of the UV
           // domain of the intersection.  Check the 3D points for equality.
           SmVector3d sDU1, sDV1;
           SmVector3d sDU2, sDV2;
           SmPoint3d sPnt1, sPnt2;
           SER(m_cpSurface[0]->Evaluate1stDerivatives(sUV1,TRUE,TRUE,sPnt1,sDU1,sDV1));
           SER(m_cpSurface[1]->Evaluate1stDerivatives(sUV2,TRUE,TRUE,sPnt2,sDU2,sDV2));
           if (iu1 == 1) sDU1 = - sDU1;
           if (iv1 == 1) sDV1 = - sDV1;
           if (iu2 == 1) sDU2 = - sDU2;
           if (iv2 == 1) sDV2 = - sDV2;

           // when 3D Points are within tolerance
           if (sPnt1.DistanceBetween(sPnt2) < dApproxTol3d) 
             {
               // Have coincident points.  Some day we might want to add
               // a degenerate curve right here.  
               // For now just try to eliminate the possiblity of
               // intersection going away from the point.

               // skip singular surfaces
               if (sDU1.LengthSquared() < SM_EFF_ZERO) continue;
               if (sDV1.LengthSquared() < SM_EFF_ZERO) continue;
               if (sDU2.LengthSquared() < SM_EFF_ZERO) continue;
               if (sDV2.LengthSquared() < SM_EFF_ZERO) continue;

               // normalize the tangents
               SER(sDU1.Unitize());
               SER(sDV1.Unitize());
               SER(sDU2.Unitize());
               SER(sDV2.Unitize());

               // get Surface1 Normal
               SmVector3d sNorm1 = sDU1 * sDV1;
               if (sNorm1.LengthSquared() < SM_EFF_ZERO) 
                 { continue;
                 }

               // get sBin = orthogonalized sDV1, in sDU1/sDV1 plane and perp to sDU1
               SmVector3d sBin = sNorm1 * sDU1;

               // project both sDU2 and sDV2 onto Surface1 plane
               SmVector3d sProjDU2 = sNorm1 * sDU2 * sNorm1; // Project to plane
               SmVector3d sProjDV2 = sNorm1 * sDV2 * sNorm1; // Project to plane
               SmBoolean bUInside = FALSE;
               SmBoolean bVInside = FALSE;

               // If both derivative vectors of second are perpendicular to tangent plane
               // of the first surface then just set to FALSE and return because
               // we can't get a good read on it.
               if (   sProjDU2.LengthSquared() < SM_EFF_ZERO_SQ
                   && sProjDV2.LengthSquared() < SM_EFF_ZERO_SQ) 
                 {
                   rbFurtherWorkNeeded = TRUE;
                   return SM_SUCCESS;
                 }

               // See if Surface2 Intersection domain extends into Surface1 Intersection domain
               // by cheking both the DomainCorner Utangent and Vtangent vectors
               // to see if they lie within this Surface1 IntersectionDomain corner sector.
               if (sProjDU2.LengthSquared() > SM_EFF_ZERO_SQ) 
                 {
                   bUInside = sProjDU2.IsInsideSector(sDU1,sBin,sDV1);
                 }
               if (sProjDV2.LengthSquared() > SM_EFF_ZERO_SQ) 
                 {
                   bVInside = sProjDV2.IsInsideSector(sDU1,sBin,sDV1);
                 }

               // when Surface2 IntersectionDomain does not move into Surface1 Intersection domain
               if (bUInside == FALSE && bVInside == FALSE) 
                 {
                   // no further work needed here - check next corners
                   rbFurtherWorkNeeded = FALSE;

                   // this is a point intersection - mark it with a degenerate curve
                   SmBSplineCurve *pNewBSP = NULL ;
                   SER(SmBSplineCurve::CreateDegenerateCurve(crContext, 3, sPnt1, pNewBSP));
                   if(p3DCurves) { p3DCurves->Add(pNewBSP); }

                   // Now for the first UV degenerate curve
                   SER(SmBSplineCurve::CreateDegenerateCurve(crContext, 2, sUV1, pNewBSP));
                   if(pSurface1UVCurves) { pSurface1UVCurves->Add(pNewBSP); }

                   // Now for the second UV degenerate curve
                   SER(SmBSplineCurve::CreateDegenerateCurve(crContext, 2, sUV2, pNewBSP));
                   if(pSurface2UVCurves) { pSurface2UVCurves->Add(pNewBSP); }
              
                   // set curve dist and type parameters
                   double dDist = sPnt1.DistanceBetween(sPnt2);
                   if(pCurveTypes) { pCurveTypes->Add(SM_TC_TOUCHING); }
                   if(pDeviations) { pDeviations->Add(dDist); }
                   return SM_SUCCESS;

                 }
               else 
                 {
                   // further work is needed - set bit and return
                   rbFurtherWorkNeeded = TRUE;
                   return SM_SUCCESS;
                 }
             } // end corner points are coincident check
         } // end iter 4 Surface2->UVDomain corner Points
     } // End iter 4 Surface1->UVDomain corner Points
  return SM_SUCCESS;

} // end SmSurfaceIntersector::GrazingBoxProcess

/*******************************************************************//**
PURPOSE: Step back from a surface pole along the curve 
    and compute good 1st derivatives for the surface.

NOTES: 
***********************************************************************/
SmStatus SmSurfaceIntersector::StepBackFromSingularity
  (SmSurfParamType eSingDir,
   ULONG lSurface,
   ULONG bDirection,      // 0 - step off rPrevPnt
                          // 1 - step off rCurrPnt
   SmTsectPnt & rPrevPnt,
   SmTsectPnt & rCurrPnt,
   double dStepSize)
{
    if (!bDirection) {
        rPrevPnt.CrvDeriv() = rCurrPnt.CrvDeriv();
    }
    else {
        rCurrPnt.CrvDeriv() = rPrevPnt.CrvDeriv();
    }


    SmHermiteCurve sHerm(rPrevPnt.CrvPos(), rPrevPnt.CrvDeriv() * dStepSize,
                         rCurrPnt.CrvPos(), rCurrPnt.CrvDeriv() * dStepSize );
    sHerm.SetContext(NULL);
    SmExtent1d sIvl = sHerm.GetNaturalInterval();
    SmPoint3d sPnt;
    
    double dTVal = 0.01;
    if (bDirection) dTVal = 0.99;
    SER(sHerm.EvaluatePoint(sIvl.Evaluate(dTVal),sPnt));

    SmSolution sSolution;
    SmBoolean bFoundAnswer = FALSE;
    SER(m_cpSurface[lSurface]->LocalPointSolve(m_vUVDomain[lSurface],
                                               SM_SO_MINIMIZE,
                                               sPnt,
                                               rPrevPnt.UVPos(lSurface),
                                               bFoundAnswer,
                                               sSolution));
    if (!bFoundAnswer) {
        SmSolution sData[16];
        SmSolutionArray sSolutions(16,sData);
        SER(m_cpSurface[lSurface]->GlobalPointSolve(m_vUVDomain[lSurface],
                                                    SM_SO_MINIMIZE,sPnt,
                                                    m_dThisApproxTol3d,
                                                    NULL,
                                                    SM_SR_ALL,
                                                    sSolutions));
        if (sSolutions.GetSize() > 0) {
            sSolution = sSolutions[0];
            bFoundAnswer = TRUE;
        }
    }

    if (bFoundAnswer) {
        SmPoint2d sUV(sSolution.m_vStart[0],sSolution.m_vStart[1]);
        SmVector3d sDU, sDV;
        SER(m_cpSurface[lSurface]->Evaluate1stDerivatives(sUV,TRUE,TRUE,sPnt,sDU,sDV));
        if (!bDirection) {
            rPrevPnt.SrfDu(lSurface) = sDU;
            rPrevPnt.SrfDv(lSurface) = sDV;
            if (eSingDir == SM_SP_U) { rPrevPnt.UVPos(lSurface).x= sUV.x; }
            else { rPrevPnt.UVPos(lSurface).y = sUV.y; }
        }
        else {
            rCurrPnt.SrfDu(lSurface) = sDU;
            rCurrPnt.SrfDv(lSurface) = sDV;
            if (eSingDir == SM_SP_U) { rCurrPnt.UVPos(lSurface).x= sUV.x; }
            else { rCurrPnt.UVPos(lSurface).y = sUV.y; }
        }
    }
    return SM_SUCCESS;

} // end SmSurfaceIntersector::StepBackFromSingularity

/*******************************************************************//**
PURPOSE: 
   For an XSectPnt location, specified as a pair of Surface UV points, compute:
         1. Intersection Point Type
         2. the Surface position and derivative values at the UVPoints,                     
         3. the 3DCurve position and Tangent values,                                        
         4. The 3DCurve tangent values projected into the surface UV Domains.               
   If the XSectPnt is near a singular intersection point (where the surfaces 
         are parallel to one another), Refine the XSectPnt location and
         set the output values for the singular intersection point.
                
NOTES: 
  1. Save Surface1 and Surface2 Pos, 
                                1st derivative, 
                                2nd derivative, and 
                                unitized normal 
     for given UV values.
  2. Let Curve 3DPoint = Avg(Surface1(UV1), Surface2(UV2))
  3. If type != SM_IP_SINGULARITY, assume type == SM_IP_CROSSING and
     (GWC: Only the FilletIntersector sets points to SM_IP_SINGULARITY outside of this method.
           For at least the SmSurfaceIntersector and the SmAdvSurfaceIntersector, the type
           will always be SM_IP_UNKNOWN)
      Let 3DPoint.Tang = cross(Surface1_Normal, Surface2_Normal).Unitize()
      3.a If not given a previous point, 
            make sure that points on boundaries get tangent curve directions
            that point inside the surface domains.
      3.b If given a previous point, 
            make sure that PrevPnt.Tang.Dot(cross(Prev.Surface1_Normal, Prev.Surface2_Normal)
            is the same sign as 3DPoint.Tang.Dot(cross(3DPoint.Surface1_Normal, 3DPoint.Surface2_Normal).
            If not negate 3DPoint.Tang. 
  4. When Surface normals are nearly parallel, ( 3DPoint.Tang length == 0, or 
                                                 running along a tangent intersection.)
     Sort intersection singularity cases marking type as
         SM_IP_TANGENT_CURVE   - Point is on tangent curve
         SM_IP_TANGENT_POINT   - Singular tangent point (surfaces touch at this point) 
         SM_IP_COINCIDENCE     - Point on surfaces where second fundamental forms match
         and return
     or make sure that a 3DPoint.Tang value is computed.
  5. Project tang vector into both Surface's UV planes and store
     UVTangent values in rTsectPnt.m_vUVCurvePV[jSrf][1].  These are not
     normalized.
  6. let rTsectPnt.m_dTangentPlaneAngleRad = angle between surface normals
  7. If surface normals are within three degrees search for a nearby singularity point
     where the surfaces are parallel to one another.
       If found, refine this point's location to parallel point 
         and set output values for the refined output locations
       When not found and given a PreviousPnt,
         set thisPoints->3DCurve->Tangent = PreviousPnt->3DCurve->Tangent

***********************************************************************/
SmStatus SmSurfaceIntersector::ComputePointValues
 (SmPoint2d aUVValues[2],         // in : UV values on each surface for point
  SmTsectPnt & rTsectPnt,         // out: TsectPnt 3D and UV pos, tangent, and type values
  SmTsectPnt * pOptPreviousPnt,   // in : Last intersection point on curve being stepped out, NULL to ignore
                                  //      When supplied used to handle singularity cases and
                                  //      insure continuity of trace direction.
                                  //      default:[NULL]
  double * pdStepSize)            // in : distance to step back from singularities to try and find 
                                  //      a nearby neighbor to use to computePointValues, NULL to ignore 
                                  //      default:[NULL]
{
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  // draw Surface0(blue), Surface1(green), XSectPoint(Red)
  if(bDebugMe)
    {
      SmPoint3d sPoint1, sPoint2 ;
      m_cpSurface[0]->EvaluatePoint(aUVValues[0], sPoint1) ;
      m_cpSurface[1]->EvaluatePoint(aUVValues[1], sPoint2) ;
      SmFace *pFace1 = (SmFace *)m_cpSurface[0]->GetFace() ;
      SmFace *pFace2 = (SmFace *)m_cpSurface[1]->GetFace() ;
      SmBrep *pBrep1 = pFace1 ? pFace1->GetBrep() : NULL ;
      SmBrep *pBrep2 = pFace2 ? pFace2->GetBrep() : NULL ;

      smgfx_Erase() ;
      smgfx_SetLook(1,2, 1,1,1) ;     if(pBrep1) { pBrep1->Draw(TRUE) ; } sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, .5,.5,.5) ;  if(pBrep2) { pBrep2->Draw(TRUE) ; } sm_GraphicsLoop() ;
      smgfx_SetLook(2,3,  0,0,1) ;  m_cpSurface[0]->DrawUV(3,3,FALSE,NULL,pBrep1==NULL) ; sm_GraphicsLoop() ;
      smgfx_SetLook(2,3,  0,1,0) ;  m_cpSurface[1]->DrawUV(3,3,FALSE,NULL,pBrep2==NULL) ; sm_GraphicsLoop() ;
      smgfx_SetLook(3,5,  1,0,0) ;  sPoint1.Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook(6,8,  0,1,0) ;  sPoint2.Draw() ; sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

  // 1. For each surface, store UVPoint values and compute and save surface values. 
  //      (pos, 1st & 2nd derivs, and unitized normal.)
  ULONG lSrf ;
  for(lSrf=0; lSrf<2; lSrf++) 
    {
      // store input UV values into output rTSectPnt
      rTsectPnt.UVPos(lSrf)   = aUVValues[lSrf];

      // load Surface eval matrix into output rTsectPnt
      SER(m_cpSurface[lSrf]->Evaluate(aUVValues[lSrf],                    // in : target UV parameter
                                      2,                                  // in : highest U derivative
                                      2,                                  // in : highest V derivative
                                      TRUE,TRUE,TRUE,                     // in : UFromLeft, VFromLeft, OnlyUpperHalf
                                      rTsectPnt.SrfEvalMatrix( lSrf ) )); // out: derivative matrix

      // store a unitized surface normal in rTsectPnt.
      SmVector3d sNorm = rTsectPnt.SrfDu(lSrf) * rTsectPnt.SrfDv(lSrf);
      if (sNorm.LengthSquared() < SM_EFF_ZERO_SQ) 
        {
          // try to evaluate a normal for pole and singularity points
          SER(m_cpSurface[lSrf]->EvaluateNormal(aUVValues[lSrf],TRUE,TRUE,
              sNorm));

          // error case - surface point has no normal
          if (sNorm.LengthSquared() < SM_EFF_ZERO_SQ) 
            {
              SER(SM_ERR);
            }
        } // end zero length normal check

      // save the unitized normal and the UV param value in the TsectPnt
      sNorm.Unitize();
      rTsectPnt.SrfNorm(lSrf) = sNorm;

    } // end iter lSrf, computing surface and surface normal evaluations for both surfaces

  // Classify the UVPoint Params and set m_eUVParamType values
  // This sets pTSP->m_eUVParamType (four values): Start, Inside, End...
  rTsectPnt.ClassifyPointParams(aUVValues, 
                                m_cpSurface, 
                                m_vUVDomain, 
                                m_dThisApproxTol3d) ;
                                          
  // Now compute the 3D curve Position and Deriv Vector
  // 2. CurvePoint = average of SurfacePoints
  rTsectPnt.CrvPos() = (  rTsectPnt.SrfPos(0) + rTsectPnt.SrfPos(1) ) / 2.0;

  // init deviation with max distance between CrvPos and SurfacePoints
  rTsectPnt.m_dDeviation = rTsectPnt.CrvPos().DistanceBetween(rTsectPnt.SrfPos(0)) ;

  // when TsectPoint->Type is not a singularity we need to 
  //   determine its intersection type and 3DCurve_Direction.
  //   (SM_IP_SINGULARITY type TsectPnts have already been   
  //                      processed and have a 3DCurve_Direction.)
  //   ((GWC: Only the FilletIntersector sets points to SM_IP_SINGULARITY outside of this method.
  //          So this rule may be just for the fillet case, but I'm not sure I couldn't track that
  //             just be reading the code.  In any case,
  //          for at least the SmSurfaceIntersector and the SmAdvSurfaceIntersector, the type
  //          will always be SM_IP_UNKNOWN))

  // 3. for Points which are NOT already marked SM_IP_SINGULARITY
  if (rTsectPnt.m_ePointType != SM_IP_SINGULARITY) 
    {
      // assume type == SM_IP_CROSSING and compute xsect curve tangent direction
      rTsectPnt.m_ePointType = SM_IP_CROSSING;
      rTsectPnt.CrvDeriv()   =  rTsectPnt.SrfNorm(0)
                              * rTsectPnt.SrfNorm(1) 
                              * m_dCurveTraceDirection;

      // Compute/Store angle between normal vectors to test SM_IP_CROSSING assumption
      SER(rTsectPnt.SrfNorm(0).AngleBetween(rTsectPnt.SrfNorm(1),
                                            rTsectPnt.m_dTangentPlaneAngleRad));
      if (rTsectPnt.m_dTangentPlaneAngleRad > SM_PI/2.0) 
        {
          rTsectPnt.m_dTangentPlaneAngleRad = SM_PI - rTsectPnt.m_dTangentPlaneAngleRad;
        }
     
      // 3.a gwc: ensure xsect curve tangent direction points into surfaces
      //          when start point happens to fall on surface natural boundaries
      if(pOptPreviousPnt == NULL) // because pOptPreviousPnt == NULL (a starter point)
        {
          // Drop the UVDeriv onto both surfaces for next check
          for(lSrf=0; lSrf<=1; lSrf++)
            {
              // project xSectCurve Tangent vector onto UV planes for both surfaces
              SER(smsurf_DropVectors(rTsectPnt.SrfDu(lSrf),      // in : Surface U_dir tangent
                                     rTsectPnt.SrfDv(lSrf),      // in : Surface V_dir tangent
                                     1,                          // in : number of vectors to drop
                                     &rTsectPnt.CrvDeriv(),      // in : vector to drop
                                     &rTsectPnt.UVDeriv(lSrf))); // out: Resulting UVVector
            }

          // when the starter xsect direction does not go inside a surface when the
          // starter point is on the surface's natural boundary
          if(   (rTsectPnt.UParamType(0) == SM_EP_START && !(rTsectPnt.UVDeriv(0).x >= -SM_EFF_ZERO))
             || (rTsectPnt.UParamType(0) == SM_EP_END   && !(rTsectPnt.UVDeriv(0).x <=  SM_EFF_ZERO)) 
             || (rTsectPnt.VParamType(0) == SM_EP_START && !(rTsectPnt.UVDeriv(0).y >= -SM_EFF_ZERO))
             || (rTsectPnt.VParamType(0) == SM_EP_END   && !(rTsectPnt.UVDeriv(0).y <=  SM_EFF_ZERO))
        
             || (rTsectPnt.UParamType(1) == SM_EP_START && !(rTsectPnt.UVDeriv(1).x >= -SM_EFF_ZERO))
             || (rTsectPnt.UParamType(1) == SM_EP_END   && !(rTsectPnt.UVDeriv(1).x <=  SM_EFF_ZERO)) 
             || (rTsectPnt.VParamType(1) == SM_EP_START && !(rTsectPnt.UVDeriv(1).y >= -SM_EFF_ZERO))
             || (rTsectPnt.VParamType(1) == SM_EP_END   && !(rTsectPnt.UVDeriv(1).y <=  SM_EFF_ZERO)))
            {
              // negate the xsect starter direction 
              rTsectPnt.UVDeriv(0) = - rTsectPnt.UVDeriv(0) ;
              rTsectPnt.UVDeriv(1) = - rTsectPnt.UVDeriv(1) ;
              rTsectPnt.CrvDeriv() = - rTsectPnt.CrvDeriv() ;
            }

          // complain when that does not fix the problem - odds are that the
          // solution after it is trimmed to the surface boundaries
          // is going to be a degenerate point.
          if(   (rTsectPnt.UParamType(0) == SM_EP_START && !(rTsectPnt.UVDeriv(0).x >= -SM_EFF_ZERO))
             || (rTsectPnt.UParamType(0) == SM_EP_END   && !(rTsectPnt.UVDeriv(0).x <=  SM_EFF_ZERO)) 
             || (rTsectPnt.VParamType(0) == SM_EP_START && !(rTsectPnt.UVDeriv(0).y >= -SM_EFF_ZERO))
             || (rTsectPnt.VParamType(0) == SM_EP_END   && !(rTsectPnt.UVDeriv(0).y <=  SM_EFF_ZERO))
        
             || (rTsectPnt.UParamType(1) == SM_EP_START && !(rTsectPnt.UVDeriv(1).x >= -SM_EFF_ZERO))
             || (rTsectPnt.UParamType(1) == SM_EP_END   && !(rTsectPnt.UVDeriv(1).x <=  SM_EFF_ZERO)) 
             || (rTsectPnt.VParamType(1) == SM_EP_START && !(rTsectPnt.UVDeriv(1).y >= -SM_EFF_ZERO))
             || (rTsectPnt.VParamType(1) == SM_EP_END   && !(rTsectPnt.UVDeriv(1).y <=  SM_EFF_ZERO)))
            {
              // ignore this message for startpoints - the degeneracy gets found out later
              if(pOptPreviousPnt != NULL)
                {
                  SM_ASSERT_MSG(0, _T("negating SmTsectPnt XSect direction didn't send it to surface interiors in all directions")) ;
                }
#ifdef SM_DEBUG_CODE
              // draw Surface0(blue), Surface1(green), XSectPoint(Red)
              if(bDebugMe)
                {
                  SmPoint3d sPoint1, sPoint2 ;
                  m_cpSurface[0]->EvaluatePoint(aUVValues[0], sPoint1) ;
                  m_cpSurface[1]->EvaluatePoint(aUVValues[1], sPoint2) ;
                  SmFace *pFace1 = (SmFace *)m_cpSurface[0]->GetFace() ;
                  SmFace *pFace2 = (SmFace *)m_cpSurface[1]->GetFace() ;
                  SmBrep *pBrep1 = pFace1 ? pFace1->GetBrep() : NULL ;
                  SmBrep *pBrep2 = pFace2 ? pFace2->GetBrep() : NULL ;
                  rTsectPnt.Dump() ; 

                  smgfx_Erase() ;
                  smgfx_SetLook(.5,2, 1,1,1) ;     if(pBrep1) { pBrep1->Draw(TRUE) ; } sm_GraphicsLoop() ;
                  smgfx_SetLook(.5,2, .5,.5,.5) ;  if(pBrep2) { pBrep2->Draw(TRUE) ; } sm_GraphicsLoop() ;
                  smgfx_SetLook(1,3,  0,0,1) ;  m_cpSurface[0]->DrawUV(3,3,FALSE,NULL,pBrep1==NULL) ; sm_GraphicsLoop() ;
                  smgfx_SetLook(1,3,  0,1,0) ;  m_cpSurface[1]->DrawUV(3,3,FALSE,NULL,pBrep2==NULL) ; sm_GraphicsLoop() ;
                  smgfx_SetLook(3,5,  1,0,0) ;  sPoint1.Draw() ; sm_GraphicsLoop() ;
                  smgfx_SetLook(6,8,  0,1,0) ;  sPoint2.Draw() ; sm_GraphicsLoop() ;
                  smgfx_SetLook(9,10, 0,0,1) ;  rTsectPnt.Draw() ; sm_GraphicsLoop() ;
                  sm_GraphicsLoop() ;
                }
#endif // SM_DEBUG_CODE
            }

        } // end check starter points ensuring initial xsect direction goes into surface domains
      else // 3.b Previous Point exists
        {
          // ensure trace direction is the same as the last
          SmBoolean bSameDir ;

          // for simple case check consistency with surface normals
          if(pOptPreviousPnt->PointType() == SM_IP_CROSSING)
            {
              SmBoolean bLastSameDir = 0.0 < pOptPreviousPnt->CrvDeriv().Dot(  pOptPreviousPnt->SrfNorm(0)
                                                                             * pOptPreviousPnt->SrfNorm(1)) ;
              SmBoolean bThisSameDir = 0.0 < rTsectPnt.CrvDeriv().Dot(  rTsectPnt.SrfNorm(0)
                                                                      * rTsectPnt.SrfNorm(1)) ;
              bSameDir = (bLastSameDir == bThisSameDir) ;
            }
          else // just check consistency of trace curve directions
            {  
              bSameDir = 0.0 < pOptPreviousPnt->CrvDeriv().Dot(rTsectPnt.CrvDeriv()) ;
            }
      
          // when needed - negate rTsectPnt crv dir    
          if(bSameDir == FALSE)
            {
              //      rTsectPnt.UVDeriv(0) = - rTsectPnt.UVDeriv(0) ;
              //      rTsectPnt.UVDeriv(1) = - rTsectPnt.UVDeriv(1) ;
              rTsectPnt.CrvDeriv() = - rTsectPnt.CrvDeriv() ;

            } // end need to negate crv dir check
        } // end check previous points ensuring next xsect direction is compatible with last

      // 4. when tangent plane angle is very small or
      //    when xsect tangent length is zero we could be at a surface pole
      //    or running into or along a tangent intersection
      if (   rTsectPnt.m_dTangentPlaneAngleRad  < m_dStartParallelTolRadians      // GWCTangentChange
          || rTsectPnt.CrvDeriv().Length()      < m_dStartParallelTolRadians      // GWCTangentChange
          || (   pOptPreviousPnt != NULL
              && pOptPreviousPnt->m_ePointType == SM_IP_TANGENT_CURVE)) 
        {
          SmSurfParamType eSingDir;

          // We're at some sort of problem area where we can't use the
          // cross product of the surface normals to get a step direction.
          //
          // If we're at a singularity on the surface (e.g., a pole),
          // and we have a previous point, use the direction from the
          // previous point, projected to the current tangent plane.
          // Otherwise, check for an intersection singularity, where the
          // surfaces become tangent.

          if(   pOptPreviousPnt != NULL
             && (   m_cpSurface[0]->IsSingularity( aUVValues[0], eSingDir )
                 || m_cpSurface[1]->IsSingularity( aUVValues[1], eSingDir )))
            {
              SmVector3d sLastVec    = pOptPreviousPnt->CrvDeriv();
              SmVector3d sProjVec    =   rTsectPnt.SrfNorm(0) 
                                       * sLastVec 
                                       * rTsectPnt.SrfNorm(0);
              rTsectPnt.CrvDeriv()   = sProjVec;
              rTsectPnt.m_ePointType = SM_IP_TANGENT_CURVE;
            }
          else // we are probably on an intersection singularity
            { 
              // determine the xSectPoint's singularity Type and a 3DCurve_Direction value
              //  type = oneof: SM_IP_TANGENT_POINT  - surfaces touch at point                  
              //                SM_IP_TANGENT_CURVE  - point on tangent curve                   
              //                SM_IP_SINGULARITY    - point at nexus of 4 or 6 intersect curves
              //                SM_IP_COINCIDENCE    - matching 2nd fundamental forms           
              // Set that value in rTsectPnt.
              // This also uses and sets the CrvDeriv member
              // of pOptPreviousPnt, if present.

              if (ComputeSingularityPoint(aUVValues, rTsectPnt, pOptPreviousPnt) != SM_SUCCESS) 
                {
                  return SM_ERR;
                }

              // GWC: tangent and coincidence points don't get traced.
              //      tangent points are point intersections.
              //      coincident points are regions of coincidence and get mapped out
              //       by the coincidence checker.
              if (   rTsectPnt.m_ePointType == SM_IP_TANGENT_POINT 
                  || rTsectPnt.m_ePointType == SM_IP_COINCIDENCE) 
                {
                  return SM_SUCCESS;
                }
            }
        } // end length of cross product of normals is zero check
    } // end m_ePointType != SM_IP_SINGULARITY check  (perhaps a fillet special case - never true for SmSurfaceIntersector)

  // if xSectTangent length is still bad, we have a TangentPoint intersection
  double dTangentLength = rTsectPnt.CrvDeriv().Length();
  if ( dTangentLength < m_dStartParallelTolRadians )    // GWCTangentChange
    {
      SE(SM_ERR);
      rTsectPnt.m_ePointType = SM_IP_TANGENT_POINT;
      return SM_SUCCESS;
    }

  // the xSectCurve Tangent is not zero, so normalize it.
  rTsectPnt.CrvDeriv() /= dTangentLength;

  // Check for the lengths of the first derivatives of the surface.
  // If they are near zero, then check for singularities.
  // Note, doing this requires a previous point and a step size value.
  if (pOptPreviousPnt && pdStepSize) 
    {
      SmSurfParamType eSingDir;
      double dDULenSq1 = rTsectPnt.SrfDu(0).LengthSquared();
      double dDVLenSq1 = rTsectPnt.SrfDv(0).LengthSquared();

      // if we are at a surface1 singular point
      if (   dDULenSq1 < SM_EFF_ZERO_SQRT 
          || dDVLenSq1 < SM_EFF_ZERO_SQRT) 
        {
          if (m_cpSurface[0]->IsSingularity(aUVValues[0],eSingDir)) 
            {
              // Step back along 3D curve a little bit and compute
              // tangent vectors of the surface again.
              SER(StepBackFromSingularity(eSingDir,0,1,*pOptPreviousPnt,rTsectPnt,*pdStepSize));
            }
        }

      // if we are at a surface2 singular point
      double dDULenSq2 = rTsectPnt.SrfDu(1).LengthSquared();
      double dDVLenSq2 = rTsectPnt.SrfDv(1).LengthSquared();
      if (   dDULenSq2 < SM_EFF_ZERO_SQRT 
          || dDVLenSq2 < SM_EFF_ZERO_SQRT) 
        {
          if (m_cpSurface[1]->IsSingularity(aUVValues[1],eSingDir)) 
            {
              // Step back along 3D curve a little bit and compute
              // tangent vectors of the surface again.
              SER(StepBackFromSingularity(eSingDir,1,1,*pOptPreviousPnt,rTsectPnt,*pdStepSize));
            }
        }
    } // end previousPnt and StepSize existence check

  // 5. for both surfaces - set IntersectCurve UV Tangents
  for(lSrf=0; lSrf<=1; lSrf++)
    {
      // 6. project xSectCurve Tangent vector onto UV planes for both surfaces
      SER(smsurf_DropVectors(rTsectPnt.SrfDu(lSrf),      // in : Surface U_dir tangent
                             rTsectPnt.SrfDv(lSrf),      // in : Surface V_dir tangent
                             1,                          // in : number of vectors to drop
                             &rTsectPnt.CrvDeriv(),      // in : vector to drop
                             &rTsectPnt.UVDeriv(lSrf))); // out: Resulting UVVector
    }

  // 6. Compute/Store angle between normal vectors
  SER( rTsectPnt.SrfNorm(0).AngleBetween(rTsectPnt.SrfNorm(1), 
                                         rTsectPnt.m_dTangentPlaneAngleRad));
  if (rTsectPnt.m_dTangentPlaneAngleRad > SM_PI/2.0) 
    {
      rTsectPnt.m_dTangentPlaneAngleRad = SM_PI - rTsectPnt.m_dTangentPlaneAngleRad;
    }

  // 7. If surface normals are within 3.0 degrees - seek a nearby singular point
  //    A. When a nearby point can be found where the surfaces are parallel to one another 
  //         refine the point location to that point and set the output values for that point
  //    B. when no nearby parallel point exists and given a Previous Point
  //         set output 3DCurve->Tangent = PreviousPnt->3DCurve->Tangent 
  if (   pOptPreviousPnt != NULL 
      && rTsectPnt.m_ePointType == SM_IP_CROSSING 
      && rTsectPnt.m_dTangentPlaneAngleRad < SM_DEG2RAD( 3.0 ) )
    {
      SmSolution sSolution;
      SmBoolean bFoundAnswer = FALSE;

      // Look for singularity in a neighborhood twice as big
      // as the current point is from the last point.  
      // This area should be big enough to pick up most points of singularity.

      // skip Previous Points of unknown type 
      //   1st points on a trace are given tempPreviousPnts of UNKNOWN type
      //   which are uninitialized except for its 3DCurve->Tangent value.

      if (    pOptPreviousPnt->m_ePointType != SM_IP_UNKNOWN
          &&  pOptPreviousPnt->m_ePointType != SM_IP_SINGULARITY )    // GWC: why skip singularity points?
        {                                                             //      could have been set by above ComputeSingularityPoint() call.
          // let surface1 domain = area centered on thisPoint and large enough
          // to include PreviousPoint
          SmExtent2d sUVDomain1(pOptPreviousPnt->UVPos(0));
          SmVector2d sBigger1 = rTsectPnt.UVPos(0) - pOptPreviousPnt->UVPos(0); 
          SmPoint2d  sNewPnt1 = rTsectPnt.UVPos(0) + sBigger1;
          sUVDomain1.AddPoint2d(m_vUVDomain[0].ClampPoint2d(sNewPnt1));

          // let surface2 domain = area centered on thisPoint and large enough
          // to include PreviousPoint
          SmExtent2d sUVDomain2(pOptPreviousPnt->UVPos(1));
          SmVector2d sBigger2 = rTsectPnt.UVPos(1) - pOptPreviousPnt->UVPos(1); 
          SmPoint2d  sNewPnt2 = rTsectPnt.UVPos(1) + sBigger2;
          sUVDomain2.AddPoint2d(m_vUVDomain[1].ClampPoint2d(sNewPnt2));

          // seek point where surfaces are parallel to one another
          // within the given domains
          SER( m_cpSurface[0]->LocalSurfaceSolve( sUVDomain1,
                                                 *m_cpSurface[1], 
                                                  sUVDomain2,
                                                  SM_SO_NORMALIZE, 
                                                  SM_EFF_ZERO_SQRT, NULL, NULL,
                                                  sUVDomain1.Evaluate( 0.5, 0.5 ),
                                                  sUVDomain2.Evaluate( 0.5, 0.5 ),
                                                  bFoundAnswer, 
                                                  sSolution )
             );
        } // end pOptPreviousPnt->m_ePointType check

      // when a point where the surfaces are parallel to one another was found near the inputPoint
      if (bFoundAnswer) 
        {
          // set output ComputePointValues = the parallel point solution
          SmPoint2d sUVSFound[2] = { SmPoint2d(sSolution.m_vStart[0],sSolution.m_vStart[1]),
                                     SmPoint2d(sSolution.m_vStart[2],sSolution.m_vStart[3]) 
                                   } ;
          rTsectPnt.m_ePointType = SM_IP_UNKNOWN;

          // get PointValues for found point, The NULL prevents additional recursion
          SER(ComputePointValues(sUVSFound, rTsectPnt, NULL));

          // when given a previousPnt
          if (pOptPreviousPnt) 
            {
              // when new and previous TsectPnt->3DCurveTangents are opposite
              if (pOptPreviousPnt->CrvDeriv().Dot(rTsectPnt.CrvDeriv()) < 0.0) 
                {  
                  //cbi: does this signal crossing a singularity?
                  // invert the new TsectPnt Tangent and recompute associated UVTangents
                  rTsectPnt.CrvDeriv() = - rTsectPnt.CrvDeriv();
                  for (ULONG jSrf=0; jSrf<=1; jSrf++) 
                    {
                      SER(smsurf_DropVectors(rTsectPnt.SrfDu(jSrf),
                                             rTsectPnt.SrfDv(jSrf),
                                             1,
                                             &rTsectPnt.CrvDeriv(),
                                             &rTsectPnt.UVDeriv(jSrf)));
                    } // end iter both surfaces recomputing UVTangent vectors
                } // end opposite tangents check
            }  // end OptPreviousPnt existence check
        } // end found an intersection branch

      // Didn't find a nearby tangent point.  Then, if the surfaces are tangent:
      else if (   rTsectPnt.m_ePointType == SM_IP_CROSSING 
               && rTsectPnt.m_dTangentPlaneAngleRad < m_dStartParallelTolRadians )  // GWCTangentChange
        {
          // take 3DCurve->tangent from PreviousPnt and recompute UVTangents
          // rTsectPnt.m_ePointType = SM_IP_SINGULARITY;
          rTsectPnt.CrvDeriv() = pOptPreviousPnt->CrvDeriv();

          // Need to normalize this: all the other TsectPnts will be unit.
          rTsectPnt.CrvDeriv().Unitize();

          for (ULONG jSrf=0; jSrf<=1; jSrf++) 
            {
              SER(smsurf_DropVectors(rTsectPnt.SrfDu(jSrf), 
                                     rTsectPnt.SrfDv(jSrf),
                                     1,
                                     &rTsectPnt.CrvDeriv(),
                                     &rTsectPnt.UVDeriv(jSrf)));
            }

          return SM_SUCCESS;

        } // end surface normals are within .1 degrees and surf/surf intersection failed to find a new point branch
    } // end surface normals are within 3 degrees of one another check

  // all done
  return SM_SUCCESS;

} // SmSurfaceIntersector::ComputePointValues

/*******************************************************************//**
PURPOSE: Compute the point and derivative values of the singularity point.

NOTES: Note that the basic version of the surface intersector does
    not handle points where the normals of a surface coincide.  It just
    produces an error.
***********************************************************************/
SmStatus SmSurfaceIntersector::ComputeSingularityPoint
  (SmPoint2d    aUVValues[2],      // in : Surface intersection UVPoints 
   SmTsectPnt & rTsectPnt,         // out: SurfacePoint values 
   SmTsectPnt * pOptPreviousPnt)   // in : last successfully classified intersectionCurve Point
{
  SM_REF3(aUVValues, rTsectPnt, pOptPreviousPnt) ;
    // If you do not wish to see any errors when singularity points or
    // tangency points are encountered in the basic intersector just 
    // comment out the following two lines.  If all goes well it should
    // just continue on to other curves without singularities.
    smos_WriteBuffer(_T("Singularity/Tangency Points not handled by basic SSI Module\n"));
    SE(SM_ERR);
    return SM_ERR;

} // end SmSurfaceIntersector::ComputeSingularityPoint

/*******************************************************************//**
PURPOSE: Compute the step size for the next intersection point given
   the current point and the last step size.

NOTES: Note that the step size is a parametric step size

METHOD ---
  if   dOldStepSize == 0 let rdNewStepSize = 100.0 * m_dThisApproxTol3d
  else                   let rdNewStepSize = dOldStepSize ;

NOTE: a previous method based on xsect curve curvature has been abandoned
***********************************************************************/
SmStatus SmSurfaceIntersector::ComputeStepSize
  (SmTsectPnt & rTsectPnt,        // NotUsed: in : last xsect point
   double       dOldStepSize,     // in : last step size
   double     & rdNewStepSize)    // out: next step size
{
  SM_REF1(rTsectPnt) ;
    if (SM_IS_ZERO(dOldStepSize))
      {
        rdNewStepSize = 100.0 * m_dThisApproxTol3d; // default if nothing else goes right
        dOldStepSize  = rdNewStepSize;
      }
    else 
      {
        rdNewStepSize = dOldStepSize;
      }

    // all done
    return SM_SUCCESS;


    // Compute a step size based on curvature of the curve at the 
    // current point.
//    double dTanLengSq = rTsectPnt.CrvDeriv().LengthSquared();
//    SmVector3d sCurvatureVec = rTsectPnt.CrvDeriv * rTsectPnt.Crv2ndDeriv *
//                               rTsectPnt.CrvDeriv / (dTanLengSq * dTanLengSq);
//    double dCurvature = sCurvatureVec.Length();
//    if (SM_IS_ZERO(dCurvature)) return SM_SUCCESS;
//    double dRadOfCurv = 1.0 / dCurvature;
//    double dTanLeng = rTsectPnt.CrvDeriv().Length();
//    if (SM_IS_ZERO(dTanLeng)) {
//        SE(SM_ERR);  
//        return SM_SUCCESS;
//    }
    // Note that the tangent length should always be
    // one to produce curves parameterized by arc length
//    SM_ASSERT(SM_IS_ZERO(dTanLeng-1.0));
    
//    rdNewStepSize = dRadOfCurv * m_dThisAngTolRad / dTanLeng;
//    rdNewStepSize /= 2.0; // Shrink it somewhat to be safe and prevent extra
     // iterations in inner loops
//    if (!SM_IS_ZERO(dOldStepSize)) {
        // If we had an old step size then condition the new step
        // size using it.
//        if (rdNewStepSize > 1.5 * dOldStepSize) {
//            rdNewStepSize = 1.5 * dOldStepSize;
//        }
//        else if (rdNewStepSize < 0.75 * dOldStepSize) {
//            rdNewStepSize = 0.75 * dOldStepSize;
//        }
//        else {
//            rdNewStepSize = dOldStepSize;
//        }
//    }   
//    return SM_SUCCESS;
} // end SmSurfaceIntersector::ComputeStepSize

/*******************************************************************//**
PURPOSE: Compute the next point in the intersection stepping algorithm.
    This algorithm will compute the point based on the step size and return
    a boolean indicating if the step size satisfies all of our stepping 
    criteria.

NOTES: 
 - Return value: if this method returns anything other than SM_SUCCESS,
   then it will not be able to calculate a step of any size.
   This can happen for example if the start points are outside of their
   uv domains, or if the start points are already out of the given
   tolerance of each other.  So the caller need not keep trying smaller steps.
   (that's how TestSpanAccuracy() tells us.)

METHOD ---
  1. Quit if TsectPnt is not in both surfaces.
  2. clip StepSize to ensure next guesses are in both surfaces
     2a. if StepSize is 0.0 return a boundary hit
  3. let next UV = prev UV + StepSize * prev UV_tangent
  4. refine next UV values with surf/surf intersection
      4a. use tol = m_dThisApproxTol3d     for SM_IP_CROSSING points
      4b. use tol = 100*m_dThisApproxTol3d for SM_IP_TANGENT_CURVE and SM_IP_SINGULARITY points
      4c. skip intersection for SM_IP_TANGENT_POINT points
      4d. quit for all other types
  5. set stepSize to equal arc-length of surface1 UVTrimCurve
    5a. Handle special case where stepSize == 0.0
  6. Compute NextTSP point values (3D pos and tang                                                              
                                   2D pos and tang on both surfaces                                             
                                   Surface pos, 1st deriv, 2nd deriv, and normals for both surfaces             
                                   Type = oneof SM_IP_TANGENT_CURVE, SM_IP_TANGENT_POINT, SM_IP_COINCIDENCE, ...
                                   angle between surface normals)
  7. Check validity of nextTSP
     7a. for SM_IP_TANGENT_POINT and SM_IP_TANGENT_POINT points  mark rbFoundGoodPoint = FALSE and quit
     7b. if refined surface1 UV point stepped in wrong direction mark rbFoundGoodPoint = FALSE and quit
     7c. TestSpanAccuracy
          rbFoundGoodPoint == TRUE when
                angleBetween(TsectPnt.3DTangent, rNextTSP.3DTangent)           < m_dThisAngTolRad 
            and max approx_3DCurve/approx_SurfTrimCurve dist (tested at 5 pts) < m_dThisApproxTol3d/2.0  
  8. let NextTSP.m_dCurveParameter =  rPrevTSP.m_dCurveParameter + dStepSize; 
     This gives the xsect curve a Surface1 UVTrimCurve arc-length parameterization. 
  GWC:Modification - switched from Surface1 UVTrimCurve arc-length parameterization to
                     3DCurve arc-length parameterization                                                                                 
***********************************************************************/
SmStatus SmSurfaceIntersector::ComputeNextPoint
  (SmTsectPnt & rPrevTSP,         // in : last computed xSect point positions and tangents
   double       dStepSize,        // i/o: size for next step (reduced when a boundary is hit)
   SmTsectPnt & rNextTSP,         // out: next xsect point
   SmBoolean  & rbFoundGoodPoint, // out: TRUE = Found next point passes angle and dist checks
                                  //      FALSE= doesn't
   double     & rdDeviationFound, // out: max approx_3Dcurve/approx_surfTrimCurve dist       
                                  //        of 5 test points between TsectPnt and NextTSP    
   double     & rdAngleFoundRad,  // out: angleBetween(TsectPnt.3DTangent, NextTSP.3DTangent)
   SmBoolean  & rbClipped,        // out: TRUE=proposed stepSize was reduced to
                                  //       force NextTSP to be in both Surfaces
   SmBoolean  & rbBoundaryHit)    // out: TRUE=TsectPnt is already on a boundary
                                  //       and the NextTSP direction steps off one of the surfaces.
{
  // init output
  rbFoundGoodPoint = FALSE ;
  rbClipped        = FALSE ;
  rbBoundaryHit    = FALSE ;
  
  // check state - both surfaces contain rPrevTSP
  if (   !m_vUVDomain[0].ContainsPoint2d( rPrevTSP.UVPos(0), SM_EFF_ZERO )
      || !m_vUVDomain[1].ContainsPoint2d( rPrevTSP.UVPos(1), SM_EFF_ZERO ))
    { SER(SM_ERR); }

  // if needed: clip stepsize to keep nextTSP in both surfaces
  //   with nextTSP = TsectPnt + StepSize * TsectPntTangent 
  double dUnclippedStepSize = dStepSize;

  dStepSize = m_vUVDomain[0].ClipLine2d(rPrevTSP.UVPos(0), 
                                        rPrevTSP.UVDeriv(0), 
                                        dStepSize);
  dStepSize = m_vUVDomain[1].ClipLine2d(rPrevTSP.UVPos(1), 
                                        rPrevTSP.UVDeriv(1), 
                                        dStepSize);

  // remember clipping when it happens
  rbClipped = ( dStepSize < dUnclippedStepSize - SM_EFF_ZERO * (1 + dUnclippedStepSize) );

  // when StepSize is clipped to zero - return a boundary hit with no next point
  if ( SM_IS_ZERO( dStepSize ) )
    {
      rbFoundGoodPoint = FALSE;
      rbBoundaryHit    = TRUE;
      return SM_SUCCESS;
    }

  // Set NextStep UVs for Surface0 and Surface1
  SmVector2d sGuessUV0 =   rPrevTSP.UVPos(0) 
                         + dStepSize * rPrevTSP.UVDeriv(0);
  SmVector2d sGuessUV1 =   rPrevTSP.UVPos(1) 
                         + dStepSize * rPrevTSP.UVDeriv(1);

  // this safety step should no longer fire since StepSize has already been clipped
  sGuessUV0 = m_vUVDomain[0].ClampPoint2d( sGuessUV0 );
  sGuessUV1 = m_vUVDomain[1].ClampPoint2d( sGuessUV1 );

static constexpr SmBoolean cbiBdryTest = TRUE;

  // Find surf/surf xSectPoint near guess sGuessUV points)
  SmBoolean bFoundAnswer; 
  SmVector2d sNextUVs[2];

  // Note: do not SER, because returning SM_ERR means something different.
  // (See this method's Usage Notes, above.)
  SmStatus eStat = RefinePoint( sGuessUV0, sGuessUV1,
                                rPrevTSP.m_ePointType, 
                                &rPrevTSP.CrvDeriv(),      // cpOptPlaneNormal
                                &rPrevTSP.UVPos(0),        // cpOptPreviousUV
                                cbiBdryTest,               // bDoBoundaryTesting (cbi want this?)
                                bFoundAnswer, sNextUVs);
  if ( eStat != SM_SUCCESS || bFoundAnswer == FALSE )
    {
      rbFoundGoodPoint = FALSE;
      return SM_SUCCESS;
    }

  // compute actual step to guess step
  SmVector2d sGuessStep      = sGuessUV0   - rPrevTSP.UVPos(0);
  SmVector2d sActualStep     = sNextUVs[0] - rPrevTSP.UVPos(0);
  double     dGuessStepSize  = sGuessStep.Length();
  double     dActualStepSize = sActualStep.Length();
  if ( dGuessStepSize > SM_EFF_ZERO * dActualStepSize )
    {
      dStepSize *= ( dActualStepSize / dGuessStepSize );
    }

  // Step size may go to zero again so test it again.
  if (   SM_IS_ZERO(dStepSize) 
      || dActualStepSize < dGuessStepSize / 100.0) 
    {

      // Before bailing out try local intersection again without boundary testing.
      // Sometimes local surface intersect converges to a boundary which is back to
      // the previous point (usually a start point)
      // Note, do not SER.
      eStat = RefinePoint( sGuessUV0, sGuessUV1,
                           rPrevTSP.m_ePointType, 
                           &rPrevTSP.CrvDeriv(), // cpOptPlaneNormal
                           &rPrevTSP.UVPos(0),   // cpOptPreviousUV
                           FALSE,                // bDoBoundaryTesting
                           bFoundAnswer, 
                           sNextUVs );

      if ( eStat != SM_SUCCESS || bFoundAnswer == FALSE )
        {
          rbFoundGoodPoint = FALSE;
          return SM_SUCCESS;
        }

      //      SER(m_cpSurface[0]->LocalSurfaceIntersect
      //                 (m_vUVDomain[0], *m_cpSurface[1], m_vUVDomain[1], 
      //                  m_dThisApproxTol3d, sGuessUV0, sGuessUV1, 
      //                  &rPrevTSP.CrvDeriv(),  // cpOptPlaneNormal  
      //                  NULL,                  // cpOptPreviousUV   
      //                  FALSE,                 // bDoBoundaryTesting
      //                  bFoundAnswer, sSol));

      if (bFoundAnswer) 
        {
          // update StepSize for this refined point
          // sNextUVs[0] = m_vUVDomain[0].ClampPoint2d(
          //    SmPoint2d(sSol.m_vStart[0],sSol.m_vStart[1]));
          // sNextUVs[1] = m_vUVDomain[1].ClampPoint2d(
          //    SmPoint2d(sSol.m_vStart[2],sSol.m_vStart[3]));

          sGuessStep      = sGuessUV0   - rPrevTSP.UVPos(0);
          sActualStep     = sNextUVs[0] - rPrevTSP.UVPos(0);
          dGuessStepSize  = sGuessStep.Length();
          dActualStepSize = sActualStep.Length();
          if ( dGuessStepSize > SM_EFF_ZERO * dActualStepSize )
            {
              dStepSize *= ( dActualStepSize / dGuessStepSize );
            }
        }

      // if this 2nd refined point also ends up with a zero stepSize - quit
      if ( SM_IS_ZERO( dStepSize ) )
        {
          rbFoundGoodPoint = FALSE;
          rbBoundaryHit    = TRUE;
          return SM_SUCCESS;
        }
    } // end dStepSize length check

#ifdef SM_DEBUG_CODE
  // distance moved by refine point:
  // d2DDist0 = sNextUVs[0].DistanceBetween(sGuessUV0) ;       //unused
  // double d2DDist1 = sNextUVs[1].DistanceBetween(sGuessUV1) ;       //unused
#endif

  // set NextTSP values given its surface UV values.
  //   compute: 3D pos and tang
  //            2D pos and tang on both surfaces
  //            Surface pos, 1st deriv, 2nd deriv, and normals for both surfaces
  //            Type = oneof SM_IP_TANGENT_CURVE, SM_IP_TANGENT_POINT, SM_IP_COINCIDENCE, ...
  //            angle between surface normals
  //   Does not set curve parameter.
  // Tell ComputePointValues that this is a next point (not a start point) by setting pOptPreviousPoint != NULL)
  if ( ComputePointValues( sNextUVs, rNextTSP, &rPrevTSP, &dStepSize)
        != SM_SUCCESS) 
    {
      rbBoundaryHit = TRUE; // causes termination of current tracing
      rbFoundGoodPoint = FALSE;
      return SM_SUCCESS;
    }

  // end iteration for tangent and coincident points
  if (   rNextTSP.m_ePointType == SM_IP_TANGENT_POINT 
      || rNextTSP.m_ePointType == SM_IP_COINCIDENCE) 
    {
      rbFoundGoodPoint = FALSE;
      return SM_SUCCESS;
    }  
      
  // Also make sure that the point is going in the same direction
  // There are some cases where the next point could be on the
  // wrong side of the current point.  This is also a violation
  // of our angle tolerance.
  // Note: use SER() here because if it returns failure, that means
  // that the prev points has a zero-length tangent, and therefore
  // we won't be able to get a meaningful step from there.
  double dLineParam;
  SER( smgu_LineClosestPoint(rPrevTSP.UVPos(0),
                             rPrevTSP.UVDeriv(0),
                             rNextTSP.UVPos(0),
                             dLineParam ));
  // when next point lies behind TsectPnt 
  if ( dLineParam < 0.0 )
    {
      // mark NextTSP as SM_IP_CROSSING and quit
      if ( rNextTSP.m_ePointType == SM_IP_SINGULARITY )
        {
          rNextTSP.m_ePointType = SM_IP_CROSSING;
        }
      rbFoundGoodPoint = FALSE;
      return SM_SUCCESS;
    }

  // let NextTSP param = TsectPnt.Param + 3DCurve ArcLength
  // Note, formerly added dStepSize, but that doesn't necessarily
  // reflect the 3d step size.  If it's very different, it will
  // create tangent vectors too big or too small; big tangent
  // vectors can create curves that double back on themselves.
  // So calculate actual 3d distance.  [Fillet regression 2:219]
  double dDistMoved = rNextTSP.CrvPos().DistanceBetween( rPrevTSP.CrvPos() );
  rNextTSP.m_dCurveParameter = rPrevTSP.m_dCurveParameter + dDistMoved;

  // set rbFoundGoodPoint == TRUE 
  //  when TsectPnt.3dTangent                                   > m_dSteppingParallelTolRadians
  //   and angleBetween(TsectPnt.3DTangent, rNextTSP.3DTangent) < m_dThisAngTolRad 
  //   and for 5 points along hermite curves between PrevTSP and NextTSP
  //        :     Dist(Surf0(UVCrv0(T)), Surf1(UvCrv1(T))      < m_dThisApproxTol3d
  //          and Dist(Surf0(UVCrv0(T)), Crv(T))               < m_dThisApproxTol3d
  //          and Dist(Surf1(UVCrv1(T)), Crv(T))               < m_dThisApproxTol3d
  SER(TestSpanAccuracy( rPrevTSP, rNextTSP, NULL, 
                        rbFoundGoodPoint,    // out: passed angle and deviation tests
                        rdDeviationFound,    // out: angleBetween(rPrevTSP.3DTangent, rNextTSP.3DTangent
                        rdAngleFoundRad));   // out: max approx_3DCurve/approx_SurfTrimCurve dist

  return SM_SUCCESS;

} // end SmSurfaceIntersector::ComputeNextPoint


SmStatus SmSurfaceIntersector::RefinePoint
(
  const SmPoint2d   & sUV0,               // in : m_cpSurface[0] guess point 
  const SmPoint2d   & sUV1,               // in : m_cpSurface[1] guess point
  SmIntersectionPointType eIPType,        // in : Expected Type of surf/surf intersection
  const SmVector3d  * cpOptPlaneNormal,   // in : If specified, solution will lie on plane defined
                                          //        Surf1Point/Surf2Point avg and this Normal vector.
                                          //      If bDoBoundaryTesting == TRUE this plane may be moved
                                          //        to a found boundary point. 
                                          //      If this vector is not specified, this method will try
                                          //      to calculate its own planeNormal by crossing the surface normals.
  const SmVector2d   * cpOptPreviousUV,    // in : Previous UV if we are stepping.  We will use this
                                          //      to prevent stepping back to same point during convergance,
                                          //      or jumping across periodic boundaries.
  SmBoolean           bDoBoundaryTesting, // in : TRUE = Look for Surface/SurfaceBoundary intersections 
  SmBoolean         & bFoundAnswer,       // out: TRUE = refinement succeeded
  SmVector2d          sUVs[2]             // out: refined UVPoints clamped to Surface Boundaries
)
{                    
  // init output
  bFoundAnswer = TRUE ;

  // locals
  SmSolution sSol;
  double     dTol   = 0.0 ;
  SmBoolean  bXSect = FALSE ;

  // 5. decide when to run surf/surf xsect with appropriate tol to refine next UV values
  switch(eIPType)
    {
      case SM_IP_TANGENT_CURVE:
      case SM_IP_SINGULARITY  : bXSect = TRUE ;
                                dTol   = m_dThisApproxTol3d*10.00 ;
                                break ;

      case SM_IP_CROSSING     : bXSect = TRUE ;
                                dTol   = m_dThisApproxTol3d ;
                                break ;

      case SM_IP_TANGENT_POINT: bXSect = FALSE ;
                                break ;

      default                 : bFoundAnswer = FALSE;
                                return SM_SUCCESS; 
   }               

  // when refining next point with surf/surf/plane xsects
  //   with plane = [Surf0GuessPoint, Normal = rTsectPnt.CrvDeriv()]
  if(bXSect) 
    { 
      // do local surface/surface intersection from nextTSP to find next point
      SER(m_cpSurface[0]->LocalSurfaceIntersect(m_vUVDomain[0], 
                                               *m_cpSurface[1], 
                                                m_vUVDomain[1], 
                                                dTol, 
                                                sUV0, 
                                                sUV1, 
                                                cpOptPlaneNormal,          
                                                cpOptPreviousUV,           
                                                bDoBoundaryTesting,        
                                                bFoundAnswer, 
                                                sSol));
             
      // when surf/surf xsect failed - quit
      if (!bFoundAnswer) 
        {
          return SM_SUCCESS;
        }
    } // end need to surf/surf xsect check

  // clamp the refined point to the surface boundaries    
  sUVs[0] = m_vUVDomain[0].ClampPoint2d(SmPoint2d(sSol.m_vStart[0],sSol.m_vStart[1]));
  sUVs[1] = m_vUVDomain[1].ClampPoint2d(SmPoint2d(sSol.m_vStart[2],sSol.m_vStart[3]));

#ifdef SM_DEBUG_CODE
static double dMaxJumpU0 = 0.0 ;
static double dMaxJumpU1 = 0.0 ;
static double dMaxJumpV0 = 0.0 ;
static double dMaxJumpV1 = 0.0 ;

static double dMaxLastJumpU0 = 0.0 ;
static double dMaxLastJumpU1 = 0.0 ;
static double dMaxLastJumpV0 = 0.0 ;
static double dMaxLastJumpV1 = 0.0 ;

  // did the refinement jump the next point across a periodic boundary?
  //double dPeriodU0 = .9 * m_vUVDomain[0].XLength() ;
  //double dPeriodU1 = .9 * m_vUVDomain[1].XLength() ;
  //double dPeriodV0 = .9 * m_vUVDomain[0].YLength() ;
  //double dPeriodV1 = .9 * m_vUVDomain[1].YLength() ;

  double dJumpU0 = smos_Max(smos_Fabs(sSol.m_vStart[0] - sUV0.x), smos_Fabs(sUVs[0].x - sUV0.x)) ;
  double dJumpU1 = smos_Max(smos_Fabs(sSol.m_vStart[2] - sUV1.x), smos_Fabs(sUVs[1].x - sUV1.x)) ;
  double dJumpV0 = smos_Max(smos_Fabs(sSol.m_vStart[1] - sUV0.y), smos_Fabs(sUVs[0].y - sUV0.y)) ;
  double dJumpV1 = smos_Max(smos_Fabs(sSol.m_vStart[3] - sUV1.y), smos_Fabs(sUVs[1].y - sUV1.y)) ;

  if(dMaxJumpU0 < dJumpU0) { dMaxJumpU0 = dJumpU0 ; } 
  if(dMaxJumpU1 < dJumpU1) { dMaxJumpU1 = dJumpU1 ; }
  if(dMaxJumpV0 < dJumpV0) { dMaxJumpV0 = dJumpV0 ; }
  if(dMaxJumpV1 < dJumpV1) { dMaxJumpV1 = dJumpV1 ; }

  //SmBoolean bJumpedPeriod     =    (m_bClosedU[0] && dJumpU0 >= dPeriodU0)
  //                              || (m_bClosedU[1] && dJumpU1 >= dPeriodU1)
  //                              || (m_bClosedU[0] && dJumpV0 >= dPeriodV0)
  //                              || (m_bClosedV[1] && dJumpV1 >= dPeriodV1) ;
  //SmBoolean bJumpedLastPeriod = FALSE ;

  if(cpOptPreviousUV)
    { 
      const SmPoint2d *pLastUV0 =   cpOptPreviousUV ;
      const SmPoint2d *pLastUV1 = &(cpOptPreviousUV[2]) ;

      double dLastJumpU0 = smos_Fabs(pLastUV0->x - sUV0.x) ;
      double dLastJumpU1 = smos_Fabs(pLastUV1->x - sUV1.x) ;
      double dLastJumpV0 = smos_Fabs(pLastUV0->y - sUV0.y) ;
      double dLastJumpV1 = smos_Fabs(pLastUV1->y - sUV1.y) ;

      if(dMaxLastJumpU0 < dLastJumpU0) { dMaxLastJumpU0 = dLastJumpU0 ; }
      if(dMaxLastJumpU1 < dLastJumpU1) { dMaxLastJumpU1 = dLastJumpU1 ; }
      if(dMaxLastJumpV0 < dLastJumpV0) { dMaxLastJumpV0 = dLastJumpV0 ; }
      if(dMaxLastJumpV1 < dLastJumpV1) { dMaxLastJumpV1 = dLastJumpV1 ; }

      //bJumpedLastPeriod =    (m_bClosedU[0] && dLastJumpU0 >= dPeriodU0)
      //                    || (m_bClosedU[1] && dLastJumpU1 >= dPeriodU1)
      //                    || (m_bClosedU[0] && dLastJumpV0 >= dPeriodV0)
      //                    || (m_bClosedV[1] && dLastJumpV1 >= dPeriodV1) ;
    }
#endif // SM_DEBUG_CODE

  // all done
  return SM_SUCCESS ;

} // end SmSurfaceIntersector::RefinePoint

/*******************************************************************//**
PURPOSE: Test the accuracy of a given span in terms of the 3D curves
    being on the surface and the satisfaction of angle tolerances.

NOTES: Checks the NextTSP tangent length - 
         - when too short returns SM_ERR, rbFoundGoodPoint=FALSE

       Checks the change in the Curve tangent angle - 
         - when it's > m_dThisAngTolRad, returns SM_SUCCESS, rbFoundGoodPoint=FALSE

       Using the input TsectPnt and NextTSP pos and tangent data, 
       builds 3 hermite xSect curves:
        Curve3d, CurveUV0, and CurveUV1
       
       Then for 5 points along the hermite curves, with varying param T, 
       checks the distance between the points
        Curve3d(T)
        Surface0(CurveUV0(T))
        Surface1(CurveUV1(T))
       When dist > m_dThisApproxTol3d returns SM_SUCCESS and rbFoundGoodPoint=FALSE

       Arrive here - return SM_SUCCESS and rbFoundGoodPoint=TRUE


METHOD ---
  1. Check state: 3DTangent length == 0 return error.
  2. let rdAngleFoundRad = angle between TsectPnt.3DTangent and NextTSP.Tangent
  3. if rdAngleFoundRad > m_dThisAngTolRad
       return Bad Point
  4. Build 3 hermite curves between TsectPnt and NextTSP
     based on given point positions and tangents: 
        one 3D curve and two UV curves
  5. Test 5 interior points for max Curve/Surface distances along hermite curves
  6. if any Curve/Surface dist > m_dThisApproxTol3d/2.0
       return Bad Point

  Bad Points: Change type from SM_IP_SINGULARITY to SM_IP_CROSSING
              set rbFoundGoodPoint == FALSE
***********************************************************************/
SmStatus SmSurfaceIntersector::TestSpanAccuracy
 (SmTsectPnt & rTsectPnt,           // in : last trace point
  SmTsectPnt & rNextTSP,            // in : current trace point
  SmTsectPnt * ,                    // out: pOptMidPoint = NOT USED
  SmBoolean  & rbFoundGoodPoint,    // out: TRUE  = next point passes both angle and dist checks
                                    //      FALSE =    angleFoundRad > m_dThisAngTolRad
                                    //              or max Curve/Surf dist > m_dThisApproxTol3d/2.0 
  double     & rdDeviationFound,    // out: max approx_3Dcurve/approx_surfTrimCurve dist 
                                    //        of 5 test points between TsectPnt and NextTSP
  double     & rdAngleFoundRad)     // out: angleBetween(TsectPnt.3DTangent, NextTSP.3DTangent)
{
    rbFoundGoodPoint = FALSE;

    // check 3D space tangent vector length
    // GWC_NEEDS_WORK An_angular_tol_is_being_compared_to_a_distance_here__probably_an_error GWC_LINE ;
   if ( rNextTSP.CrvDeriv().Length() < m_dSteppingParallelTolRadians )
    {
        SER(SM_ERR);
    }

    // get xsect curve parameter StepSize
    double dStepSize = rNextTSP.m_dCurveParameter - rTsectPnt.m_dCurveParameter;

    // let rdAngleFoundRad = angle between TsectPnt.3DTangent and NextTSP.3DTangent
    SER(rTsectPnt.CrvDeriv().AngleBetween(rNextTSP.CrvDeriv(),rdAngleFoundRad));

    // First do a quick angular test to see if we can reject this span
    if (   rdAngleFoundRad > m_dThisAngTolRad 
        && dStepSize       > SM_EFF_ZERO*100.0) 
      {
          // GWC_NEEDS_WORK Why_would_one_change_the_classification_of_a_rejected_point_here GWC_LINE ;
          if (rNextTSP.m_ePointType == SM_IP_SINGULARITY)
          {
            rNextTSP.m_ePointType = SM_IP_CROSSING;
          }

        // mark the rejected point
        rbFoundGoodPoint = FALSE;
        return SM_SUCCESS;
      }

    // TODO: if the given start point positions (Crv,Srf0,Srf1) 
    //       are already farther apart than the tolerance we use, 
    //       then we don't have to bother settingu up and testing the Hermite curves.
    //       As a corollary to that, if we return failure, the caller
    //       will keep trying shorter steps, with the same start points,
    //       which will never succeed.  This implies that the caller
    //       needs to know what tolerance we're using: it should
    //       probably be passed in.
    //       [bd 24 Feb 06 060105]
             
    // Create two parameter space Hermite curves and test some points (currently 5)
    // on the curves as projected into 3d by their corresponding surfaces.
    SmHermiteCurve sUVCrv0(rTsectPnt.UVPos(0),
                           rTsectPnt.UVDeriv(0)*dStepSize,
                           rNextTSP.UVPos(0),
                           rNextTSP.UVDeriv(0)*dStepSize,
                           2);
    sUVCrv0.SetContext(NULL);

    SmHermiteCurve sUVCrv1(rTsectPnt.UVPos(1),
                           rTsectPnt.UVDeriv(1)*dStepSize,
                           rNextTSP.UVPos(1),
                           rNextTSP.UVDeriv(1)*dStepSize,
                           2);
    sUVCrv1.SetContext(NULL);

    // create a 3D Hermite curve
    SmHermiteCurve s3DCrv(rTsectPnt.CrvPos(), rTsectPnt.CrvDeriv()*dStepSize,
                          rNextTSP. CrvPos(), rNextTSP. CrvDeriv()*dStepSize);
    s3DCrv.SetContext(NULL);

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
    if (bDebugMe) {
        SmSurface *pSurf0 = SM_CONST_CAST(SmSurface*,m_cpSurface[0]);
        SmSurface *pSurf1 = SM_CONST_CAST(SmSurface*,m_cpSurface[1]);
        if ( FALSE ) {
            smgfx_Erase();
            smgfx_SetLook( 1,1, 0,0.7,0.7 ); pSurf0->Draw(); sm_GraphicsLoop();
            smgfx_SetLook( 1,1, 0, 0, 0.7 ); pSurf1->Draw(); sm_GraphicsLoop();
        }
        SmCrvOnSurf sMap1(sUVCrv0,*pSurf0);
//        sMap1.SetContext(NULL);
        smgfx_SetLook(1,1, 0,1,1); sMap1.Draw(); sm_GraphicsLoop();
        SmCrvOnSurf sMap2(sUVCrv1,*pSurf1);
//        sMap2.SetContext(NULL);
        smgfx_SetLook(1,1, 0,0,1); sMap2.Draw(); sm_GraphicsLoop();
        smgfx_SetLook(2,1, 1,0,0); s3DCrv.Draw(); sm_GraphicsLoop();
        sm_GraphicsLoop();
    }
#endif

/*
    Note: the following does make sense, because if two surfaces intersect
          at an angle that's not near perpendicular, then two surface points
          could be within tolerance, but the distance between either of the
          points and the actual surface intersection could be farther than
          tolerance.  However, in practice, this does more harm than good.
          The reason is because of consistency of tolerances: higher-level
          routines have already decided that the given points are within
          tolerance, and if we decide here that they're not, then that
          inconsistency will cause bugs.  [bd; 060105, at various radii]

#define SSI_ANGLE_TOL_ADJUST_TestSpanAccuracy

*/

#ifdef SSI_ANGLE_TOL_ADJUST_TestSpanAccuracy

    // Utilize the angle between the normals to adjust the tolerance.
    // When the surfaces are close to parallel (small angle between
    // their normals), you could have a point on each that are close
    // together, but far from the actual intersection.  In this case,
    // we need tighter tolerance.
    // If we think of the surfaces as planes (which is close over
    // tolerance-sized distances), consider the given tolerance to be
    // the size of the region over which points on two surfaces are
    // within the tolerance we come up with here.  If we multiply the
    // given tolerance by tan( ang/2 ), then points could be within
    // that tighter tolerance over a region as big as tol.
    double dMinTanAngle = smos_Min( rTsectPnt.m_dTangentPlaneAngleRad,
                                     rNextTSP.m_dTangentPlaneAngleRad );
    // (Was 2 * sin(ang/2): that's not really what we want.)
    // double dAdjustedTolerance = 2.0 * m_dThisApproxTol3d
    //                                 * smos_Sine( dMinTanAngle / 2.0 );
    
    double dAdjustedTolerance = m_dThisApproxTol3d
                                    * smos_Tangent( dMinTanAngle / 2.0 );

    // Don't let the adjusted tolerance get to be too much smaller than the
    // given tolerance.  Otherwise we could get nearly infinite numbers of
    // points near tangency and singularity points.
    // [Note: if you change this, check fillet regression 2:227]
    dAdjustedTolerance = smos_Max( dAdjustedTolerance, m_dThisApproxTol3d/16.0 );

    // Don't let points get farther away than the tolerance either.
    // (Note, can't actually happen,
    //  assuming m_dTangentPlaneAngleRad <= 90 degrees.)
    dAdjustedTolerance = smos_Min( m_dThisApproxTol3d, dAdjustedTolerance );

#else  // SSI_ANGLE_TOL_ADJUST_TestSpanAccuracy

    double dAdjustedTolerance = m_dThisApproxTol3d;

#endif  // SSI_ANGLE_TOL_ADJUST_TestSpanAccuracy


    double dTolSq = dAdjustedTolerance * dAdjustedTolerance;
    double dMaxDistSq = 0.0;

    // for every test point - do a distance check
#define NUM_TEST_POINTS 5
    for (ULONG i=0; i<NUM_TEST_POINTS; i++) 
      {
        // get next interior xsect curve param values
        double dParam = (i+1.0) / (NUM_TEST_POINTS+1.0);

        // get UV points from UV curves
        SmPoint3d sUV3d0, sUV3d1;
        SER(sUVCrv0.EvaluatePoint(dParam,sUV3d0));
        SER(sUVCrv1.EvaluatePoint(dParam,sUV3d1));
        SmPoint2d sUV0(sUV3d0.x,sUV3d0.y);
        SmPoint2d sUV1(sUV3d1.x,sUV3d1.y);

        // clamp UV points to surface domains
        sUV0 = m_vUVDomain[0].ClampPoint2d(sUV0);
        sUV1 = m_vUVDomain[1].ClampPoint2d(sUV1);

        // get surface points from UVpoints
        SmPoint3d sPnt0, sPnt1, sCrvPnt;
        SER(m_cpSurface[0]->EvaluatePoint(sUV0,sPnt0));
        SER(m_cpSurface[1]->EvaluatePoint(sUV1,sPnt1));

        // get 3D xyz point from 3D curve
        SER(s3DCrv.EvaluatePoint(dParam,sCrvPnt));

#ifdef SM_DEBUG_CODE
        if (bDebugMe) {
            smgfx_SetLook(1,4, 0,1,1); sPnt0.Draw(); sm_GraphicsLoop();
            smgfx_SetLook(1,6, 0,0,1); sPnt1.Draw(); sm_GraphicsLoop();
            sm_GraphicsLoop();
        }
#endif
        // get distances between 3D points
        double dDistSq   = sPnt0.DistanceBetweenSquared(sPnt1);
        double dDistSqC0 = sPnt0.DistanceBetweenSquared(sCrvPnt);
        double dDistSqC1 = sPnt1.DistanceBetweenSquared(sCrvPnt);

        // get max curve/surface distance
        dDistSq = smos_3Max(dDistSq, dDistSqC0, dDistSqC1) ;

        if ( dDistSq > dMaxDistSq )
          { dMaxDistSq = dDistSq; }

        // when max curve/surface dist is bad
        if (   dDistSq > dTolSq
            && dStepSize > SM_EFF_ZERO*100.0) 
          {
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe2 = FALSE;
            if (bDebugMe2) {
                SmSurface *pSurf0 = SM_CONST_CAST(SmSurface*,m_cpSurface[0]);
                SmSurface *pSurf1 = SM_CONST_CAST(SmSurface*,m_cpSurface[1]);
                if ( FALSE ) {
                    smgfx_Erase();
                    smgfx_SetLook( 1,1, 0,0.7,0.7 ); pSurf0->Draw(); sm_GraphicsLoop();
                    smgfx_SetLook( 1,1, 0, 0, 0.7 ); pSurf1->Draw(); sm_GraphicsLoop();
                }
                SmCrvOnSurf sMap1(sUVCrv0,*pSurf0);
//                sMap1.SetContext(NULL);
                smgfx_SetLook(1,1, 0,1,1); sMap1.Draw(); sm_GraphicsLoop();
                SmCrvOnSurf sMap2(sUVCrv1,*pSurf1);
//                sMap2.SetContext(NULL);
                smgfx_SetLook(1,1, 0,0,1); sMap2.Draw(); sm_GraphicsLoop();
                smgfx_SetLook(2,1, 1,0,0); s3DCrv.Draw(); sm_GraphicsLoop();
                sm_GraphicsLoop();
            }
#endif
            // set next point type, mark point as bad, and quit
            if (rNextTSP.m_ePointType == SM_IP_SINGULARITY) 
              {
                rNextTSP.m_ePointType = SM_IP_CROSSING;
              }
            rbFoundGoodPoint = FALSE;
            rdDeviationFound = smos_Sqrt(dMaxDistSq);

            return SM_SUCCESS;
          } // end max curve/surface dist > dTolSq check

        // save max curve/surface distance 
        if (dDistSq > dMaxDistSq) { dMaxDistSq = dDistSq; }

      } // end iter every test point
    
    // save max tested distance    
    rdDeviationFound = smos_Sqrt(dMaxDistSq);
    rbFoundGoodPoint = TRUE; // passed test

    return SM_SUCCESS;

} // end SmSurfaceIntersector::TestSpanAccuracy

/*******************************************************************//**
PURPOSE: Add a point to the existing curve points.  Note that in the 
     future this routine may try to combine segments to decrease the number
     of points.

NOTES: 
***********************************************************************/
SmStatus SmSurfaceIntersector::AddPointToCurve
 (SmTsectPnt & rNextTSP)  // in : target intersection point
{
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
  if (bDebugMe) 
    {
      rNextTSP.CrvPos().Dump();
      smos_WriteBuffer(_T("\n"));
      sm_GraphicsLoop();
      smgfx_SetColor(1,0,0);
      rNextTSP.CrvPos().Draw();
      sm_GraphicsLoop();
    }
#endif
  m_vCurvePoints.Append(&rNextTSP);
  return SM_SUCCESS;

} // end SmSurfaceIntersector::AddPointToCurve

/*******************************************************************//**
PURPOSE: Add unique TsectPnts to the m_vStartPoints list.

NOTES: A TsectPnt is a duplicate and should not be added to m_sStartPoints when 
           its UVDist to an existing StartPoint is less than SM_EFF_ZERO 
      or   its UVDist to an existing StartPoint is less than SM_EFF_ZERO_SQRT
           and the 3d distance between points is less than m_dThisApproxTol3d.
      or   it's a bounded intersection (a curve passed within tolerance of 
           a natural boundary edge of a surface without actually intersecting it)
           and it's 'near' another exact intersection. (The exact intersection in
           the bounded/exact pair of intersections is saved and the bounded intersection
           is treated as the duplicate.) 
             note: The exact/bounded pair duplicate rule
                   covers the case where a ThisSurf/OtherSurfBoundaryCurve exact intersection
                   happens to be near a ThisSurf/OtherSurfBoundaryCurve bounded intersection (or vice versa)
                   because the OtherSurf boundaries are larger than the ThisSurf boundaries but 
                   happen to lie within tolerance of one another. This happens more often
                   than expected because it's common for users to attempt to help out
                   the intersector by extending the size of the surface a small amount
                   to guarantee that surf/surf intersections are truly crossing.
                   Often times, that extension amount is often of the same size as tolerances.  

                   In this case, keeping both start points 
                   confuses the trace algorithm.  Both SmTsectPnt points
                   in this situation will have pretty much the same intersection direction.
                   If trace happens to pick the bounded intersection before the exact intersect
                   the trial and walk algorithm may step into
                   an exact intersection point somewhere near the exact start.  
                   If that first step happens to be within tolerance
                   of the exact intersection start point, then trace will assume that
                   the trace is done because it has successfully traced a curve from 
                   one start point to another.  At which point it will remove both start points
                   from the start list -- as it was designed to do -- and the actual intersection
                   curve which was supposed to start at the exact SmTsectPnt of the pair
                   in question won't be traced.  That missed large intersection curve
                   may or may not be discovered later when the surface/surface algorithm gets
                   around to seeking interior intersections. The short intersection curve that
                   was actually found does not actually exist on both surfaces.  It represents
                   the extension of the intersection curve past the boundary of one surface
                   to the boundary of the other surface.  That short curve is not
                   meant to be part of the output solution.  However, the trace algorithm
                   does not always check for that situation and it may or may not be
                   added to the output depending on its overall length and whether the
                   system can generate UVTrimCurves on both surfaces for the segment.
                   The strategy adopted to fix all this is to discard the bounded start point
                   when paired with an exact start point.  Unfortunately, some bounded 
                   start points are needed; those start points that don't have a partner 
                   exact intersection start point.  The tricky part of the fix is to figure
                   out when an exact and a bounded start point are paired.  Depending on angles,
                   the locations of the two intersection may be larger than tolerance.
                   So the test for 'near' attempts to figure out when a bounded start 
                   point has a partner exact start point by checking 3d distances, uv distance,
                   and intersection curve orientations.  

                   Yes, this is a very complicated behavior.
                   I'm confident that this is the right fix for this situation.
                   I'm not confident that my predicate for identifying paired start points
                   is accurate enough to handle all cases. That predicate is a heuristic. 
                   [cf. B254 B295]
***********************************************************************/
SmStatus SmSurfaceIntersector::AddStartPoint
 (SmTsectPnt & rStartTSP)             // in : candidate StartPoint to add to m_vStartPoints array
{
  // locals
  SmBoolean bFoundMatch = FALSE;

  // get current StartPoints
  SmTsectPnt *aData[50];
  SmTArray<SmTsectPnt*> sStartPts(50,aData);
  m_vStartPoints.GetAllNodes(sStartPts);

  // Is rStartTSP a bounded intersection
  double    dScaledZero   = SM_EFF_ZERO * 100.0 * (1.0 + rStartTSP.SrfPos(0).GetMaxDimension()) ;
  SmBoolean bBoundedStart =    rStartTSP.m_dDeviation > dScaledZero
                            && (   rStartTSP.UParamType(0) == SM_EP_START
                                || rStartTSP.UParamType(0) == SM_EP_END
                                || rStartTSP.UParamType(0) == SM_EP_BOTH
                                || rStartTSP.VParamType(0) == SM_EP_START
                                || rStartTSP.VParamType(0) == SM_EP_END
                                || rStartTSP.VParamType(0) == SM_EP_BOTH)
                            && (   rStartTSP.UParamType(1) == SM_EP_START
                                || rStartTSP.UParamType(1) == SM_EP_END
                                || rStartTSP.UParamType(1) == SM_EP_BOTH
                                || rStartTSP.VParamType(1) == SM_EP_START
                                || rStartTSP.VParamType(1) == SM_EP_END
                                || rStartTSP.VParamType(1) == SM_EP_BOTH) ;

  // check every StartPoint for a match to input point
  for (ULONG i=0; i<sStartPts.GetSize(); i++) 
    {
      SmTsectPnt *pTSP           = sStartPts[i];
      SmBoolean   bBoundedTarget =    pTSP->m_dDeviation > dScaledZero
                                   && (   pTSP->UParamType(0) == SM_EP_START
                                       || pTSP->UParamType(0) == SM_EP_END
                                       || pTSP->UParamType(0) == SM_EP_BOTH
                                       || pTSP->VParamType(0) == SM_EP_START
                                       || pTSP->VParamType(0) == SM_EP_END
                                       || pTSP->VParamType(0) == SM_EP_BOTH)
                                   && (   pTSP->UParamType(1) == SM_EP_START
                                       || pTSP->UParamType(1) == SM_EP_END
                                       || pTSP->UParamType(1) == SM_EP_BOTH
                                       || pTSP->VParamType(1) == SM_EP_START
                                       || pTSP->VParamType(1) == SM_EP_END
                                       || pTSP->VParamType(1) == SM_EP_BOTH) ;
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
      // draw 
      if(bDebugMe)
        {
          SmFace *pFace1 = m_cpSurface[0] ? (SmFace *)m_cpSurface[0]->GetFace() : NULL ;
          SmFace *pFace2 = m_cpSurface[1] ? (SmFace *)m_cpSurface[1]->GetFace() : NULL ;
          SmBrep *pBrep1 = pFace1 ? pFace1->GetBrep() : NULL ;
          SmBrep *pBrep2 = pFace2 ? pFace2->GetBrep() : NULL ;

          smgfx_Erase() ;
          smgfx_SetLook(1,2, 0,0,1) ; if(pBrep1) pBrep1->Draw(TRUE) ; sm_GraphicsLoop() ;
          smgfx_SetLook(1,2, 0,1,0) ; if(pBrep2) pBrep2->Draw(TRUE) ; sm_GraphicsLoop() ;
          smgfx_SetLook(1,2, 0,1,1) ; if(m_cpSurface[0]) m_cpSurface[0]->DrawUV(8,8,FALSE,NULL,TRUE) ; sm_GraphicsLoop() ;
          smgfx_SetLook(1,2, 0,1,0) ; if(m_cpSurface[1]) m_cpSurface[1]->DrawUV(8,8,FALSE,NULL,TRUE) ; sm_GraphicsLoop() ;
          smgfx_SetLook(1,2, 0,0,0) ; if(pFace1) pFace1->Draw(SM_DM_CROSSHATCH) ; sm_GraphicsLoop() ;
          smgfx_SetLook(1,2, 0,0,0) ; if(pFace2) pFace2->Draw(SM_DM_CROSSHATCH) ; sm_GraphicsLoop() ;
          smgfx_SetLook(3,4, 1,0,0) ; if(rStartTSP.m_dDeviation < pTSP->m_dDeviation) 
                                           { rStartTSP.Draw() ; sm_GraphicsLoop() ; 
                                             pTSP->Draw() ;     sm_GraphicsLoop() ; } 
                                      else { pTSP->Draw() ;     sm_GraphicsLoop() ;  
                                             rStartTSP.Draw() ; sm_GraphicsLoop() ; } 
          sm_GraphicsLoop() ;
        }
#endif // SM_DEBUG_CODE

      // get UVDist to TargetPoint on both surfaces
      double dDistUV1Sq = rStartTSP.UVPos(0).DistanceBetweenSquared(pTSP->UVPos(0));
      double dDistUV2Sq = rStartTSP.UVPos(1).DistanceBetweenSquared(pTSP->UVPos(1));

      // when UVDists for both Surfs are zero
      // cbi probably want a larger tol here [B295]
      if (   dDistUV1Sq < SM_EFF_ZERO_SQ 
          && dDistUV2Sq < SM_EFF_ZERO_SQ) 
        {
          // found a match
          bFoundMatch = TRUE;
          break;
        }

      // Look for Exact/bounded intersection start point pairs
      if(bBoundedStart != bBoundedTarget)
        {
          // let's assume a bounded/exact pair will be 
          //      within 2*m_dThisApproxTol3d in 3d space     (I think this generally helps but can be fooled,
          //                                                      given odd narrow angles, this distance can be made
          //                                                      arbitrarily large - however I believe trace will
          //                                                      turn the bounded intersection into a degenerate point
          //                                                      and won't get into trouble for large distances.)
          // and  within in UVTol in UV space on both surfaces   (I think this generally helps but can be fooled for the
          //                                                      same reasons as the 3d check but is
          //                                                      required to prevent seam start points from being 
          //                                                      treated as pairs)
          // and  have very similar intersection curve start tangent directions (I think this is essential)
          // We might also consider having the points be on the same boundary sides however
          //   I think requiring same boundary between potential StartPoint pairs
          //   might be too restrictive around corners and a potential pairing
          //   could be missed.  If too many pairs are found perhaps this idea could cull the false positives.

          // start 3d points must be close together
          if(   (rStartTSP.SrfPos(0).DistanceBetween(pTSP->SrfPos(0)) < 2.0 * m_dThisApproxTol3d) 
             && (rStartTSP.SrfPos(1).DistanceBetween(pTSP->SrfPos(1)) < 2.0 * m_dThisApproxTol3d) ) 
            {
              // get UV distances
              double dUVTol1 = m_cpSurface[0]->GetNaturalUVDomain().GetMaxDimension() / 10.0 ;
              double dUVTol2 = m_cpSurface[1]->GetNaturalUVDomain().GetMaxDimension() / 10.0 ;

              // UV Points must be close enough
              if(   (rStartTSP.UVPos(0).DistanceBetween(pTSP->UVPos(0)) < dUVTol1) 
                 && (rStartTSP.UVPos(1).DistanceBetween(pTSP->UVPos(1)) < dUVTol2))
                {
                  // get XSectCurve tangent directions and a tangent angle tolerance value
                  double dCurvature0 = 0 ;
                  double dCurvature1 = 0 ;
                  SmVector3d sTangPlaneDir ;
                  smsurf_EvaluateNormalSection(rStartTSP.m_vSurfacePV[0],rStartTSP.UVDeriv(0),sTangPlaneDir, dCurvature0) ;
                  smsurf_EvaluateNormalSection(rStartTSP.m_vSurfacePV[1],rStartTSP.UVDeriv(1),sTangPlaneDir, dCurvature1) ;
                  double dCurvature = smos_Max( smos_Fabs(dCurvature0), smos_Fabs(dCurvature1) ) ;
                  double dCRadius   = SM_IS_ZERO(dCurvature) ? 100 * m_dThisApproxTol3d : smos_Min(1.0/dCurvature,100 * m_dThisApproxTol3d) ;
                  double dAngTolRad = .01 + smos_ArcTangent2(m_dThisApproxTol3d, dCRadius) ;

                  double dAngBetweenRad = 0 ;
                  rStartTSP.CrvDeriv().AngleBetween(pTSP->CrvDeriv(), dAngBetweenRad) ; 

                  // when tangent angels are the same - these are an exact/bounded startPoint pair
                  //   only save the exact intersection and discard the bounded intersection
                  if(dAngBetweenRad < dAngTolRad)
                    {
                      // save the exact intersection
                      if(bBoundedStart)
                        {
                          bFoundMatch = TRUE ; 
                          break ;
                        } // end omit bounded TSP branch
                      else // replace pTSP with rStartTSP
                        {
                          pTSP = &rStartTSP ;
                          bFoundMatch = TRUE ;
                          break ;
                        } // end replace bounded with exact TSP branch
                    } // end is a Bounded/Exact TSp pair check
                } // end UV Points are close enough
            } // end 3d points are close check
        } // end potential bounded/exact intersection pair check

      // GWC: this is an old check and I can't quite figure out what
      //   it's trying to do.  Perhaps it's trying to deal
      //   with the Exact/bounded intersection start point pair problem.
      //   Whatever the case, I've left this in to preserve previous behavior.
      // when UVDists for both Surfs are small
      if (   dDistUV1Sq < SM_EFF_ZERO 
          && dDistUV2Sq < SM_EFF_ZERO) 
        {
          // get TargetPoint 3d position
          SmPoint3d sP1, sP2;
          SER(m_cpSurface[0]->EvaluatePoint(pTSP->UVPos(0),sP1));
          SER(m_cpSurface[1]->EvaluatePoint(pTSP->UVPos(1),sP2));

          // CurrentPoint surf/surf distance is less than tolerance
          if (sP1.DistanceBetween(sP2) < m_dThisApproxTol3d) 
            {
#ifdef SM_DEBUG_CODE
              // GWC: This test might be improved if it were changed to check
              //      the 3d distance from the testPoint to the CurrentPoint.
              //      The following tries this change and outputs a message
              //      when the new check would be different than the current check.
              //      If the following fires then the possible change should be considered.
              if(   rStartTSP.SrfPos(0).DistanceBetween(pTSP->SrfPos(0)) > m_dThisApproxTol3d
                 || rStartTSP.SrfPos(1).DistanceBetween(pTSP->SrfPos(1)) > m_dThisApproxTol3d
                 || rStartTSP.SrfPos(0).DistanceBetween(pTSP->SrfPos(1)) > m_dThisApproxTol3d
                 || rStartTSP.SrfPos(1).DistanceBetween(pTSP->SrfPos(0)) > m_dThisApproxTol3d)
                {
                  SM_DBG_WARN(_T("GWC trial code - AddStartPoint() found a matching startPoint when it might be distinct.")) ;
                  SM_DBG_WARN(_T("    When this fires - check the case and see which match criterion is appropriate.")) ;
                }
#endif
              bFoundMatch = TRUE;
              break;
            }
        } // end UVDists are small check
    } // end iter every StartPoint looking for matchs
  
  // add Point to m_vStartPoints list
  if (!bFoundMatch) 
    {
      // put crossing points in front of tangent, coincident, and singular points 
      if (rStartTSP.m_ePointType == SM_IP_CROSSING) { m_vStartPoints.Append(&rStartTSP); }
      else                                          { m_vStartPoints.Prepend(&rStartTSP); }
                                                    
    }
  return SM_SUCCESS;

}  // end SmSurfaceIntersector::AddStartPoint

/*******************************************************************//**
PURPOSE: Add a point to the through point list.  This method checks
    for duplicates.  The intersector will force curves near these points
    to go through them.

NOTES: 
***********************************************************************/
SmStatus SmSurfaceIntersector::AddThroughPoint
  (SmTsectPnt & rThroughTSP)           // in : Candidate Point
{
  SmBoolean bFoundMatch = FALSE;

  // for every through point
  SmTsectPnt *aData[50];
  SmTArray<SmTsectPnt*> sThroughPts(50,aData);
  m_vThroughPoints.GetAllNodes(sThroughPts);
  for (ULONG i=0; i<sThroughPts.GetSize(); i++) 
    {
      SmTsectPnt *pTSP  = sThroughPts[i];

      // get CandidatePoint/ThroughPoint Surface UVDistances
      double dDistUV1Sq = rThroughTSP.UVPos(0).DistanceBetweenSquared(
                          pTSP->UVPos(0));
      double dDistUV2Sq = rThroughTSP.UVPos(1).DistanceBetweenSquared(
                          pTSP->UVPos(1));

      // when the points are close in UVSpace
      if (   dDistUV1Sq < SM_EFF_ZERO_SQ 
          && dDistUV2Sq < SM_EFF_ZERO_SQ) 
        {
          // the CandidatePoint is a duplicate
          bFoundMatch = TRUE;
          break;
        } // end found a duplicate point
    } // end iter every through point

  // when candidate point is unique
  if (!bFoundMatch) 
    {
      // add it to the throughPoints array
      m_vThroughPoints.Append(&rThroughTSP);
    }

  // all done
  return SM_SUCCESS;

} // end SmSurfaceIntersector::AddThroughPoint

/*******************************************************************//**
PURPOSE: Test given Point to see if it is on an XSectCurve segment
            bounded by two given endPoints and a tolerance

NOTES: No Side effects - all input arguments could be const except
                they're not due to implementation convenience
***********************************************************************/
static SmBoolean sm_IsPointOnCurve     // rtn: TRUE=PointToTest is on curve between Prev and Next points
  (SmTsectPnt & crPointToTest,         // in : TargetPoint
   SmTsectPnt & crPrevTSP,             // in : LastPoint known to be on XSectCurve
   SmTsectPnt & crNextTSP,             // in : NextPoint known to be on XSectCurve
   double       d3DApproxTol)          // in : 
{
  // First adjust the tolerance for the angle between the two surfaces.
  // If they're close to tangent, then the region where points could
  // be considered within tolerance 'spreads out', specifically, by
  // the tangent of the half-angle between them.
  // Note, sometimes some of these values are zero, which
  // presumably means unset.
  // [regression tests: my_shell_demo iters 0,3,6,7; fillet test 2:214]

  // GWC_NEEDS_WORK extend_algorithm_to_check_for_direction_for_Singularity_points GWC_LINE ;

  // get max surfNorm/surfNorm angle at XSectCurve Segment endPoints
  double dAng = smos_Max( crPrevTSP.m_dTangentPlaneAngleRad,
                          crNextTSP.m_dTangentPlaneAngleRad );

  // when the angle is not exactly zero (tangent xSectCurve Segment)
  if ( dAng > 0.0 )
    {
      // increase the tolerance size
      double dTan = smos_Tangent( dAng / 2 );
      if ( dTan < 0.001 ) { dTan = 0.001; }
      d3DApproxTol /= dTan;
    }

  // Get xSect Segment domain interval.
  double dDeltaT = crNextTSP.m_dCurveParameter - crPrevTSP.m_dCurveParameter;

  // Build 3d hermite curve from PrevTSP to NextTSP.
  const SmPoint3d & rP1 = crPrevTSP.CrvPos();
  const SmPoint3d & rP2 = crNextTSP.CrvPos();
  SmHermiteCurve sHerm3d( rP1, crPrevTSP.CrvDeriv()*dDeltaT,
                        rP2, crNextTSP.CrvDeriv()*dDeltaT, 3 );
  sHerm3d.SetContext(NULL);

  // Get rough distance between PrevTSP and NextTSP.
  double dLength = (rP2-rP1).GetMaxDimension();

  // let s3DBox = bounding box for hermite curve
  SmExtent3d s3DBox;
  SmExtent1d sHermiteDomain = sHerm3d.GetNaturalInterval();  // (We know it's [0,1]...)
  sHerm3d.CalculateBoundingBox( sHermiteDomain, &s3DBox );

  // let sPntBox = HalfSize+Tol Sized box centered on the crPointToTest
  // (Note: that can be a pretty big box.)
  SmExtent3d sPntBox( crPointToTest.CrvPos() );
  sPntBox.ExpandAbsolute( d3DApproxTol*2.0 + dLength / 4.0 );

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe2 = FALSE;
  if (bDebugMe2)
    {
      sm_GraphicsLoop();
      smgfx_SetLook(4,6, 1,0,0) ; crPointToTest.CrvPos().Draw(); sm_GraphicsLoop();
      smgfx_SetLook(3,4, 1,0,1) ; sHerm3d.Draw(&sHermiteDomain); sm_GraphicsLoop();
      smgfx_SetLook(1,2, 0,1,0) ; s3DBox.Draw(NULL) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, .5,.5,.5) ; sPntBox.Draw(NULL) ; sm_GraphicsLoop() ;
      sm_GraphicsLoop();
    }
#endif

  // Not on curve if the bounding box tests fails.
  if ( s3DBox.AreDisjoint( sPntBox ) )
    { return FALSE; }


  // Do bounding-box checks in the uv-space of both surfaces:
  // check the uv points against the bounding boxes of Hermite curves.
  ULONG lSurf;
  for ( lSurf=0; lSurf<2; lSurf++ )
    {
      // Make a hermite UVTrimCurve.
      SmHermiteCurve sHermUV( crPrevTSP.UVPos(lSurf),
                              crPrevTSP.UVDeriv(lSurf)*dDeltaT,
                              crNextTSP.UVPos(lSurf),
                              crNextTSP.UVDeriv(lSurf)*dDeltaT,
                              2);
      sHermUV.SetContext( NULL );
      SmExtent1d sUVDomain = sHermUV.GetNaturalInterval();

      // Let sUVBox = UVTrimCurve bounding box expanded by its diagonal length.
      // But: Have to expand u and v separately.  Example:  [B401]
      // u is 0.0 - 0.00402  on domain 0-1
      // v is 19.4 - 21.0 on domain 0-360
      // So u gets expanded by 1.6, which would include a point across the seam.

      SmExtent3d sUVBox;
      sHermUV.CalculateBoundingBox( sUVDomain, &sUVBox );

      // sUVBox.ExpandAbsolute( sUVBox.GetSize().Length() );

      // Have to check for zero-length domains:
      // expanding by their length wouldn't do anything.
      double dDiag = sUVBox.GetSize().Length();  // for checking zero-length domains.

      SmExtent1d sTemp = sUVBox.GetUInterval();
      double dLen = sTemp.GetLength();
      if ( dLen < dDiag / 100 )
        { dLen = dDiag / 100; }
      sTemp.ExpandAbsolute( dLen );
      sUVBox.SetUInterval( sTemp );

      sTemp = sUVBox.GetVInterval();
      dLen = sTemp.GetLength();
      if ( dLen < dDiag / 100 )
        { dLen = dDiag / 100; }
      sTemp.ExpandAbsolute( dLen );
      sUVBox.SetVInterval( sTemp );


      // Let sUV = UVPoint to test.
      SmPoint3d sUV( crPointToTest.UVPos( lSurf ));

      // If point is not contained by the expanded UVBounding Box - return FALSE.
      if ( ! sUVBox.ContainsPoint3d( sUV ))
        { return FALSE; }

    } // end iter both surfaces

  // Leave this next test out.  Its premise is dodgy, and it could never
  // happen anyway: dDistToEnd >= dDist, always.  (dDistToEnd is the
  // hypotenuse of a right triangle, and dDist is a leg.)
//  // Test to see if point is just off the end of the curve.
//  // This line starts at the curve end (next pt) and leaves the curve,
//  // so if the drop param is negative, it's before the end of the curve.
//  double dParam;
//  smgu_LineClosestPoint( rP2,  // current curve end point
//                         crNextTSP.CrvDeriv(),
//                         crPointToTest.CrvPos(),
//                         dParam );
//
//  // When point is beyond endPoint
//  if ( dParam > 0.0 )
//    {
//      // When point is close to the xSectCurve segment chord
//      double dDist;
//      smgu_LinePointDistance( rP2,
//                              crNextTSP.CrvDeriv(),
//                              crPointToTest.CrvPos(),
//                              dDist);
//      if (   dDist < dLength / 10.0 
//          && dDist < d3DApproxTol*20.0 )
//        {
//           // When point is close to xSectCurve segment endPoint
//           double dDistToEnd = crPointToTest.CrvPos().DistanceBetween( rP2 );
//           if ( dDistToEnd < dDist/4.0 )
//             {
//               // found an end point
//               return TRUE;
//             }
//        }
//    } // End point-is-beyond-endPoint check
  
  // Arrive here when Point projects to xSectCurve Segment
  // Find closest point on the Hermite Curve to the Point.
  SmSolution sSol;
  SmBoolean bSuccess;

  // Use midpoint as guess: should be fine for this simple curve.
  double dGuess = sHermiteDomain.Evaluate(0.5);

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
  if (bDebugMe) 
    {
      sm_GraphicsLoop();
      SmExtent1d  sInt = sHerm3d.GetNaturalInterval();
      smgfx_SetLook(4,6, 1,0,0) ; crPointToTest.CrvPos().Draw(); sm_GraphicsLoop();
      smgfx_SetLook(3,4, 1,0,1) ; sHerm3d.Draw(&sInt); sm_GraphicsLoop();
      smgfx_SetLook(1,2, 0,1,0) ; s3DBox.Draw(NULL) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, .5,.5,.5) ; sPntBox.Draw(NULL) ; sm_GraphicsLoop() ;
      sm_GraphicsLoop();
    }
#endif  
  // Find point on hermite curve closest to crPointToTest.
  SER( sHerm3d.LocalPointSolve( sHermiteDomain,
                                SM_SO_MINIMIZE,
                                crPointToTest.CrvPos(), 
                                NULL, NULL, NULL, dGuess,
                                bSuccess, sSol ));

  // If the test point dropped to the interior of the Hermite curve,
  // then we've moved past it.  [B391]
  // Note, we could have used NORMALIZE for the drop, but we want to check
  // the distance from the ends, when it doesn't drop to the interior.
  if ( bSuccess )
    {
      // First, if distance is within tolerance, it's good.
      double dDist = sSol.m_vStart.m_dSolutionValue;
      if ( dDist <= d3DApproxTol )
      {
          // crPointToTest has a direction: check that too.  [B401]
          SmVector3d sPV[2];
          sHerm3d.Evaluate( sSol.m_vStart[0], 1, FALSE, sPV );
          // We actually want one that's pointing back into the curve that
          // we're creating, so look for negative dot product.

          if ( crPointToTest.CrvDeriv().Dot( sPV[1] ) < 0.0 )
            { return TRUE; }
      }

      // Check value on interior.  (Pass negative tol: just excludes end points.)
      double dT = sSol.m_vStart[0];
      if ( sHermiteDomain.ContainsValue( dT, -SM_EFF_ZERO ) ) 
        {
          // But, it could be on a completely different curve,
          // far away, so just give it a more liberal tolerance.  [B395]
          // Actually, it must be very liberal, > 111.  [B493]
          //   static double dFudge = 120.0;
          // Actually, it's worse than that.  [B690]
          // Don't even use tol, use the length of the Hermite curve.

#ifdef SM_DEBUG_CODE
      if ( bDebugMe ) {
          TCHAR sBuff[128];
          smos_sprintf(sBuff, _T("\n   Dist %16.8lf  Tol %16.8lf   Dist/Tol %16.8lf   Len %16.8lf   Dist/Len %16.8lf"), 
            dDist, d3DApproxTol, dDist/d3DApproxTol, dLength, dDist/dLength );
          smos_WriteBuffer( sBuff );
      }
#endif // SM_DEBUG_CODE

          double cbiDistLimit = dLength / 2.0;

          if ( dDist <= cbiDistLimit )
            { return TRUE; }
          else
            { return FALSE; } // (Just a good place for a breakpoint.)

        } // end if point drop is interior to the curve
    } // end if point drop succeeded

  // The point is not on the curve.
  return FALSE;

} // end sm_IsPointOnCurve

/*******************************************************************//**
PURPOSE: Trim a TraceCurve xsect curve segment down to a specified
            trim point.
            
METHOD ---
***********************************************************************/
static SmStatus sm_UpdateTsectPoint
  (SmTsectPnt           * pPrev1,         // in : start point for xsect segment
   SmTsectPnt           & rNextTSP,       // i/o: end   point for xsect segment to be set to trim
   SmTsectPnt           * pNewTSP,        // i/o: trim point to end segment; m_dCurveParameter updated if unset.
   SmSurfaceIntersector & rSI)            // in : intersector object with surface pointers and more
{
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  if(bDebugMe)
    {
      pPrev1->Dump() ; 
      rNextTSP.Dump() ;
      pNewTSP->Dump() ;

      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0,1,1) ; rSI.GetSurface(0)->DrawUV(4,4,FALSE,NULL,TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 1,1,0) ; rSI.GetSurface(1)->DrawUV(4,4,FALSE,NULL,TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(6,8, 1,0,0) ; pPrev1->Draw() ; sm_GraphicsLoop();
      smgfx_SetLook(6,8, 0,1,0) ; rNextTSP.Draw() ; sm_GraphicsLoop();
      smgfx_SetLook(6,8, 0,0,1) ; pNewTSP->Draw() ; sm_GraphicsLoop();
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

  // get pPrev1/rNextTSP 3DCurve arcLength distance
  double dDist = pPrev1->CrvPos().DistanceBetween(rNextTSP.CrvPos());

  // get parameter Step between pPrev1 and rNextTSP
  double dStep = rNextTSP.m_dCurveParameter - pPrev1->m_dCurveParameter;
  
  // get NewTSP UV values
  SmPoint2d sUVs[2];
  sUVs[0] = pNewTSP->UVPos(0);
  sUVs[1] = pNewTSP->UVPos(1);

  // copy rNextTSP user values from pNewTSP
  for (ULONG i=0; i<SM_MAX_USER_DOUBLES; i++) 
    {
      rNextTSP.m_adUserDoubles[i] = pNewTSP->m_adUserDoubles[i];
    }
  rNextTSP.m_apUserPointer[0] = pNewTSP->m_apUserPointer[0];
  rNextTSP.m_apUserPointer[1] = pNewTSP->m_apUserPointer[1];

  // ComputePointValues with pNewTSP UV values and store results in rNextTSP 
  // Tell ComputePointValues that this is a next point (not a start point) by setting pOptPreviousPnt != NULL.
  SER(rSI.ComputePointValues(sUVs, rNextTSP, pPrev1));

  // set parameter step size based on current curve parameterization
  double dNewDist = pPrev1->CrvPos().DistanceBetween(rNextTSP.CrvPos());
  double dNewStep = dNewDist / dDist * dStep ;
  rNextTSP.m_dCurveParameter = pPrev1->m_dCurveParameter + dNewStep;

  // set pNewTSP parameter value so that it knows  
  // if/where it was used on the intersection curve.
  if (pNewTSP->m_dCurveParameter < 0.0) 
    {
      pNewTSP->m_dCurveParameter = rNextTSP.m_dCurveParameter;
    }
#ifdef SM_DEBUG_CODE
  if(bDebugMe)
    {
      pPrev1->Dump() ; 
      rNextTSP.Dump() ;
      pNewTSP->Dump() ;

      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0,1,1) ; rSI.GetSurface(0)->DrawUV(4,4,FALSE,NULL,TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 1,1,0) ; rSI.GetSurface(1)->DrawUV(4,4,FALSE,NULL,TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(6,8, 1,0,0) ; pPrev1->Draw() ; sm_GraphicsLoop();
      smgfx_SetLook(6,8, 0,1,0) ; rNextTSP.Draw() ; sm_GraphicsLoop();
      smgfx_SetLook(6,8, 0,0,1) ; pNewTSP->Draw() ; sm_GraphicsLoop();
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

  return SM_SUCCESS;

}  // end sm_UpdateTsectPoint

/*******************************************************************//**
PURPOSE: Test the current point to see if it is near an existing 
   start point or the start of itself (i.e. curve is closed).

NOTES: 
  Snap to target point when the target point is within tol of the hermite curve between this and the prev SmTSectPnt.
  The list of target points include: 
  1. all m_vThroughPoints 
  2. all m_vStartPoints
  3. m_pStartPoint 
  4. m_pEndPoint (with care for single span curves terminating on a seam.)
  note: (No snapping to intersection singularities - such snapping is already done within the rNextTSP values in ComputePointValues().
  
  Set rbDone == TRUE when curve trace should be stopped 
  because rNextTSP satisfies one of the following conditions:
  1. rNextTSP == m_pStartPoint (start of current trace)
  2. rNextTSP == m_pEndPoint   (desired end of current trace)
  3. rNextTSP is oneof m_pThroughPoints 
  4. rNextTSP m_dCurveParameter is larger than m_dLastThroughParmeter + m_dExtensionDistance when m_bDoingExtensionClipping == TRUE
 *5. rNextTSp is a singluarity point   (* = currently disabled when bStopAtSingularity == FALSE, default:[FALSE])
  6. rNextTSP is on a boundary and heading out of the surface domain.
  
***********************************************************************/
SmStatus SmSurfaceIntersector::TestPoint
 (SmTsectPnt & rNextTSP,   // i/o: point to test - gets replaced by through/end points when appropriate
  SmBoolean  & rbDone)     // out: TRUE = NextTSP is now one of the m_pStartPoint
                           //             or the m_pEndPoint
                           //             or one of the m_pThroughPoints when m_bDoingExtensionClipping == TRUE
                           //             or rNextTSP.m_ePointType == SM_IP_SINGULARITY
                           //             or point is on a domain boundary
                           //                and pushing outward (leaving domain).
{
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
  if (bDebugMe) 
    {
      // draw inputs
      SmFace *pFace1 = (SmFace *)m_cpSurface[0]->GetFace() ;
      SmFace *pFace2 = (SmFace *)m_cpSurface[1]->GetFace() ;
      SmBrep *pBrep1 =   pFace1 ? pFace1->GetBrep() : NULL ;  
      SmBrep *pBrep2 =   pFace2 ? pFace2->GetBrep() : NULL ;  

      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep1) pBrep1->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,0) ; if(pBrep2) pBrep2->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,1) ; m_cpSurface[0]->DrawUV(6,6,FALSE,NULL,pBrep1==NULL) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,0,0) ; if(pFace1) pFace1->Draw(SM_DM_CROSSHATCH) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 1,1,0) ; m_cpSurface[1]->DrawUV(6,6,FALSE,NULL,pBrep2==NULL) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,0,0) ; if(pFace2) pFace2->Draw(SM_DM_CROSSHATCH) ; sm_GraphicsLoop() ;
      smgfx_SetLook(4,6, 1,0,0) ; rNextTSP.Draw() ; sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

  // If we are given through points (stored in this object) check them
  SmTsectPnt *aData[50], *aStartData[20] ;
  SmTArray<SmTsectPnt*> sThroughPts(50,aData), sStartPts(20,aStartData) ;
  m_vThroughPoints.GetAllNodes(sThroughPts);

  // for every through point
  // Note: no need to check all points, they should be in order,
  // so the first in the list must be the one we'll hit.
  ULONG i = 0;
  if ( sThroughPts.GetSize() > 0 )
    {
      SmTsectPnt *pThroughTSP = sThroughPts[i];
      SmTsectPnt *pPrev1      = m_vCurvePoints.GetLastNode();

#ifdef SM_DEBUG_CODE
          if (bDebugMe) 
            {
              smgfx_SetLook(1,2, 0,1,1) ; m_cpSurface[0]->DrawUV(6,6,FALSE,NULL,TRUE) ; sm_GraphicsLoop() ;
              smgfx_SetLook(1,2, 1,1,0) ; m_cpSurface[1]->DrawUV(6,6,FALSE,NULL,TRUE) ; sm_GraphicsLoop() ;
              smgfx_SetLook(4,6, 1,0,0) ; pPrev1->Draw() ; sm_GraphicsLoop() ;
              smgfx_SetLook(4,6, 1,0,1) ; pThroughTSP->Draw() ; sm_GraphicsLoop() ;
              sm_GraphicsLoop() ;
            }
#endif // SM_DEBUG_CODE

      // if through point is on hermite curve from pPrev1 point to rNextTSP point
      if (sm_IsPointOnCurve(*pThroughTSP, 
                            *pPrev1, 
                            rNextTSP, 
                            m_dThisApproxTol3d)) 
        {
          // The tracing has passed this through point. modify the rNextTSP
          // to correspond to the through point and announce that we are done.
          SER( sm_UpdateTsectPoint( pPrev1, rNextTSP, pThroughTSP, *this ));


          // remove the through point from the list - we're done with it
          m_vThroughPoints.Remove(pThroughTSP);

          // mark the last throughParameter distance when m_bDoingExtensionClipping
          if (   m_bDoingExtensionClipping 
              && m_vThroughPoints.GetLastNode() == NULL) 
            {
              m_dLastThroughParmeter = pThroughTSP->m_dCurveParameter;
            }

          // remember that a through point was hit
          m_bClippedByThroughPoint = TRUE;

          // If we are doing normal intersection, the through points are
          // other start points that may finish a curve.
          return SM_SUCCESS;

        } // end through point is on hermite curve from prev to current point
    } // end iter every through point

  // Check against Start Points and remove them if we hit one to finish the curve.
  m_vStartPoints.GetAllNodes(sStartPts);

  // Note that the better points (i.e. those generated by curve/curve intersection
  // are at the end of the list.  Start looking there and work back to the start
  // of the list.
  ULONG ip, lNumStartPts = sStartPts.GetSize();
  for (ip=0; ip<lNumStartPts; ip++) 
    {
      SmTsectPnt *pStartTSP = sStartPts[sStartPts.GetSize()-ip-1];
      SmTsectPnt *pPrev1    = m_vCurvePoints.GetLastNode();

#ifdef SM_DEBUG_CODE
          if (bDebugMe) 
            {
              smgfx_SetLook(1,2, 0,1,1) ; m_cpSurface[0]->DrawUV(6,6,FALSE,NULL,TRUE) ; sm_GraphicsLoop() ;
              smgfx_SetLook(1,2, 1,1,0) ; m_cpSurface[1]->DrawUV(6,6,FALSE,NULL,TRUE) ; sm_GraphicsLoop() ;
              smgfx_SetLook(4,6, 1,0,0) ; pPrev1->Draw() ; sm_GraphicsLoop() ;
              smgfx_SetLook(4,6, 1,0,1) ; pStartTSP->Draw() ; sm_GraphicsLoop() ;
              sm_GraphicsLoop() ;
            }
#endif // SM_DEBUG_CODE
 
      // Skip pPrev1 if it's the same as m_pStartPoint.
      if (pPrev1 == m_pStartPoint) 
        { continue; }

      // Also skip it if it's at the same place as pStartTSP.  [B136]
      // But -- we do want to test against our tol.  [B295]
      // // Note, this is prompted by pPrev1 having been copied from pStartTSP,
      // // in TestPoint(), so we're looking for 'exact', so use tight tolerance.
      double dDist = pPrev1->CrvPos().DistanceBetween( pStartTSP->CrvPos() );
      if ( dDist < m_dThisApproxTol3d )
        {
          // Make sure it's not a seam.
          double dDom1Size = m_vUVDomain[0].GetSize().Length();
          double dDom2Size = m_vUVDomain[1].GetSize().Length();

          double dDist1 = pPrev1->UVPos(0).DistanceBetween( pStartTSP->UVPos(0) );
          double dDist2 = pPrev1->UVPos(1).DistanceBetween( pStartTSP->UVPos(1) );
          if ( dDist1 < dDom1Size / 2.0 && dDist2 < dDom2Size / 2.0 )
            { continue; }
        }

      // if StartTsp is on xsect curve segment - no side effects, just a check
      if (sm_IsPointOnCurve(*pStartTSP, 
                            *pPrev1, 
                            rNextTSP, 
                            m_dThisApproxTol3d)) 
        {
          // Have stepped over or near a given start point.
          // Modify the rNextTSP to correspond to that start point
          SER(sm_UpdateTsectPoint(pPrev1,rNextTSP,pStartTSP,*this));
              
          // Remove the start point we just crossed, no need to start there again.
          // But, singularity points or tangent points could have more than
          // one curve coming into them, so don't remove those. 
          if (   rNextTSP.m_ePointType == SM_IP_CROSSING
              && pStartTSP->m_dTangentPlaneAngleRad > m_dSteppingParallelTolRadians )
            {
              m_vStartPoints.Remove(pStartTSP);
            }

          // In most cases finding this start point will end the intersection
          // curve.   In the case where more than one intersection curve runs 
          // through this point, other start and through points will be used 
          // to find the other intersection curves - so just end the curve.  
          // However, don't end the curve for the special case of a crossing intersection
          // where pStartTSP is on the boundary of a domain when the 
          // intersection is tangent to that boundary and there is room
          // left in the domain to continue tracing the curve.  In that case this
          // 'start' point should be treated as a through point.
          if(pStartTSP->PointType() == SM_IP_CROSSING)
            {
              SmBoolean bUBoundThroughPoint0 = (  (pStartTSP->UParamType(0) == SM_EP_START || pStartTSP->UParamType(0) == SM_EP_END)
                                                && pStartTSP->VParamType(0)  == SM_EP_INSIDE
                                                && SM_IS_ZERO(pStartTSP->UVDeriv(0).x)) ;
              SmBoolean bUBoundThroughPoint1 = (  (pStartTSP->UParamType(1) == SM_EP_START || pStartTSP->UParamType(1) == SM_EP_END)
                                                && pStartTSP->VParamType(1)  == SM_EP_INSIDE
                                                && SM_IS_ZERO(pStartTSP->UVDeriv(1).x)) ;
              SmBoolean bVBoundThroughPoint0 = (  (pStartTSP->VParamType(0) == SM_EP_START || pStartTSP->VParamType(0) == SM_EP_END)
                                                && pStartTSP->UParamType(0)  == SM_EP_INSIDE                                        
                                                && SM_IS_ZERO(pStartTSP->UVDeriv(0).y)) ;                                            
              SmBoolean bVBoundThroughPoint1 = (  (pStartTSP->VParamType(1) == SM_EP_START || pStartTSP->VParamType(1) == SM_EP_END)
                                                && pStartTSP->UParamType(1)  == SM_EP_INSIDE                                        
                                                && SM_IS_ZERO(pStartTSP->UVDeriv(1).y)) ;
              rbDone = (   bUBoundThroughPoint0
                        || bUBoundThroughPoint1
                        || bVBoundThroughPoint0
                        || bVBoundThroughPoint1) ? FALSE : TRUE ;
            }
          else
            { rbDone = TRUE; }

          return SM_SUCCESS;

        } // end StartTsp is on xsect curve segment check
    } // end iter all start points

  // If we have a start point check for a closed curve situation
  if (m_pStartPoint) 
    {
      // Don't check if we are only starting the curve
      SmTsectPnt *pPrev1 = m_vCurvePoints.GetLastNode();
      if (pPrev1 != m_pStartPoint) 
        {
          SmTsectPnt *pPrev2 = m_vCurvePoints.GetPrevNode(pPrev1);
          if (pPrev2 != m_pStartPoint) 
            {
              // if StartTsp is on xsect curve segment
              if (sm_IsPointOnCurve(*m_pStartPoint, 
                                    *pPrev1, 
                                    rNextTSP,
                                    m_dThisApproxTol3d)) 
                {
                  // Have found loop around - modify the rNextTSP to correspond
                  // to the start point and announce that we are done.
                  SER(sm_UpdateTsectPoint(pPrev1, rNextTSP, m_pStartPoint, *this));
                  m_bCurveIsClosed = TRUE;
                  rbDone           = TRUE;
                  return SM_SUCCESS;
                }
            }
        }
    } // end m_pStartPoint existence check

  // If we are given an ending point - check it
  if (m_pEndPoint) 
    {
      SmTsectPnt *pPrev1 = m_vCurvePoints.GetLastNode();

      // This check may cause a failure for situations where we have a start
      // and end node and only a single span between them.  Perhaps we need
      // add an additional check for coincidence.  That has been done.
      if (sm_IsPointOnCurve(*m_pEndPoint, 
                            *pPrev1, 
                            rNextTSP,
                            m_dThisApproxTol3d)) 
        {
          SmBoolean bOnSeam = FALSE;
          if (pPrev1 == m_vCurvePoints.GetFirstNode()) 
            {
              // Have only one node on the list there are two possible cases
              // First is that it is a closed curve on a seam that is just getting started
              // Second is that it is a single span curve
              // Check for the end point to be on the start point of the hermite.
              double dDist1 = pPrev1->CrvPos().DistanceBetween( m_pEndPoint->CrvPos() );
              if (dDist1 < m_dThisApproxTol3d) 
                {
                  bOnSeam = TRUE;
                }
            }
          if (!bOnSeam) 
            {
              // Have found loop around - modify the rNextTSP to correspond
              // to the end point and announce that we are done.
              SER(sm_UpdateTsectPoint(pPrev1, rNextTSP, m_pEndPoint, *this));
              rbDone = TRUE;
              return SM_SUCCESS;
            }
        } // end endPoint on curve check
    } // end m_pEndPoint existence check

  else if (   m_bDoingExtensionClipping 
           && m_dLastThroughParmeter != 0.0) 
    {
      if (rNextTSP.m_dCurveParameter > m_dLastThroughParmeter + m_dExtensionDistance) 
        {
          rbDone = TRUE;
          return SM_SUCCESS;
        }
    }

  // Removed next section to plow right through singularities.
  // Not sure why, but stopping at singularities causes one regression, and doesn't
  // improve any behavior.  [ torus/torus Boolean after
  // 'Testing Booleans with Primitives from Primitive Creation' ]
  
  // stop at singularities 
static SmBoolean bStopAtSingularity = FALSE ;
  if(   bStopAtSingularity    == TRUE
     && rNextTSP.m_ePointType == SM_IP_SINGULARITY )
    {
      // don't stop if previous node was also singular
      // We may have started on a singular point and have
      // not stepped far enough to get off the point

      SmTsectPnt *pPrev1 = m_vCurvePoints.GetLastNode();
      if(  !pPrev1 
         || pPrev1->m_ePointType != SM_IP_SINGULARITY)
        {
          rbDone = TRUE;
          return SM_SUCCESS;
        }
    } // end TraceCurve at Singularity point check

#ifdef SM_DEBUG_CODE
  if (bDebugMe) 
    {
      smos_WriteBuffer(_T("\n***********TestPoint - "));
      rNextTSP.Dump();
    }
#endif

  // Check whether the point is inside of the domain of the surfaces.

  SmBoolean bOnBoundary = FALSE;

  // for both surfaces
  for (ULONG lSrf=0; lSrf<2; lSrf++) 
    {
      // let tol scale to surface domain size
      double dSize = m_vUVDomain[lSrf].GetMaxDimension() ;
      double dTol    = SM_EFF_ZERO      * (1.0 + dSize) * 100.0;
      double dAngTol = SM_EFF_ZERO_SQRT * (1.0 + dSize) * 10.0;

      // check NextTSP to see if its crossing a boundary
      SmPoint2d sBiNorm ;
      if(   m_vUVDomain[lSrf].IsPoint2dOnBoundary(rNextTSP.UVPos(lSrf),
                                                  dTol,
                                                  &sBiNorm)
         && (   (sBiNorm.x != 0.0 && smos_Fabs(rNextTSP.UVDeriv(lSrf).x) > dAngTol)
             || (sBiNorm.y != 0.0 && smos_Fabs(rNextTSP.UVDeriv(lSrf).y) > dAngTol)))   
        {
          bOnBoundary = TRUE ;
        }
//      
//            // check NextTSP to see if its on boundary 1
//            if (smos_Fabs(m_vUVDomain[lSrf].GetMin().x-rNextTSP.UVPos(lSrf).x) <
//                dTol) 
//              {
//                if (smos_Fabs(rNextTSP.UVDeriv(lSrf).x) > dAngTol) 
//                  {
//                    bOnBoundary = TRUE;
//                  }
//              }
//            
//            // check NextTSP to see if its on boundary 2
//            if (smos_Fabs(m_vUVDomain[lSrf].GetMax().x-rNextTSP.UVPos(lSrf).x) <
//                dTol) 
//                  {
//                if (smos_Fabs(rNextTSP.UVDeriv(lSrf).x) > dAngTol) 
//                  {
//                    bOnBoundary = TRUE;
//                  }
//              }
//            
//            // check NextTSP to see if its on boundary 3
//            if (smos_Fabs(m_vUVDomain[lSrf].GetMin().y-rNextTSP.UVPos(lSrf).y) < 
//                dTol) 
//              {
//                if (smos_Fabs(rNextTSP.UVDeriv(lSrf).y) > dAngTol) 
//                  {
//                    bOnBoundary = TRUE;
//                  }
//              }
//            
//            // check NextTSP to see if its on boundary 4
//            if (smos_Fabs(m_vUVDomain[lSrf].GetMax().y-rNextTSP.UVPos(lSrf).y) < 
//                dTol) 
//              {
//                if (smos_Fabs(rNextTSP.UVDeriv(lSrf).y) > dAngTol) 
//                  {
//                    bOnBoundary = TRUE;
//                  }
//              }

      // done when point is on and heading out of any boundary
      if (bOnBoundary) 
        { break ; }
    } // end iter both surfaces - lSrf

  // mark points not crossing boundaries as not done and return
  if (!bOnBoundary) 
    {
      rbDone = FALSE;
      return SM_SUCCESS;
    }

  // else this point is crossing a boundary
  rbDone = TRUE;

  SmTsectPnt * aSData[50];
  SmTArray<SmTsectPnt*> sStartTSP(50,aSData);
  m_vStartPoints.GetAllNodes(sStartTSP);

  // for every start point
  for (ULONG ii=0; ii<sStartTSP.GetSize(); ii++) 
    {
      SmTsectPnt *pTSP = sStartTSP[ii];
      SmBoolean bSame = TRUE;

      // for both surfaces
      for (ULONG k=0; k<=1; k++) 
        {
          // get scaled zero
          double dSize = m_vUVDomain[k].GetMaxDimension() ;
          double dTol = SM_EFF_ZERO * (1.0 + dSize) * 100.0;

          // get UVdist from starpoint to NextTSP
          double dDist = pTSP->UVPos(k).DistanceBetweenSquared
                                                  (rNextTSP.UVPos(k));
          //cbiTol: dDist is squared, dTol is not.  But dTol is arbitrary anyway.
          // remember when startPoint is not the same as NextTSP
          if (dDist > dTol) 
            {
              bSame = FALSE;
              break ;
            }
        } // end iter both surfaces

      // when start point is close enough to NextPoint
      if (bSame) 
        {
          // remove TSP from StartPoints list
          m_vStartPoints.Remove(pTSP);
        }
    } // end iter every start point

  // done unless doing boundary point on curve test
  // GWC_NEEDS_WORK _possible_bug___should_this_be_m_bDoBoundaryPointOnCurveTest_equal_FALSE GWC_LINE ;
  // GWC: because bStopAtSingularity has been set to TRUE the next check does not normally run.
  //      which may be a good thing.  Is the next check on through points equivalent to the
  //      first check on through points that have already been made?
  if (bStopAtSingularity == FALSE)   
    {
      return SM_SUCCESS;
    }

  // Now, do one more test which is to test all through pts & end point
  // to see whether this boundary point is close to any of them and use that
  // instead.

  // add EndPoint to ThroughPoints list
  if(m_pEndPoint) 
    { sThroughPts.Add(m_pEndPoint); }

  // 3D Prev/Next Point locals
  SmVector3d & rNextPnt  = rNextTSP.CrvPos();
  SmVector2d & rNextUV0  = rNextTSP.UVPos(0);
  SmVector2d & rNextUV1  = rNextTSP.UVPos(1);
  SmVector3d & rNextVec  = rNextTSP.CrvDeriv();
  SmTsectPnt * pPrev1    = m_vCurvePoints.GetLastNode();
  double dLastStepDist   = pPrev1->CrvPos().DistanceBetween(rNextPnt);
  double dLastStepUV0    = pPrev1->UVPos(0).DistanceBetween(rNextUV0);
  double dLastStepUV1    = pPrev1->UVPos(1).DistanceBetween(rNextUV1);

  // for every through point
  for (ULONG j=0; j<sThroughPts.GetSize(); j++) 
    {
      SmTsectPnt * pTSP      = sThroughPts[j];
      SmPoint3d  & rTestPnt  = pTSP->CrvPos();
      SmPoint2d  & rTestUV0  = pTSP->UVPos(0) ;
      SmPoint2d  & rTestUV1  = pTSP->UVPos(1) ;
      double       dStepDist = rTestPnt.DistanceBetween(rNextPnt);
      double       dStepUV0  = rTestUV0.DistanceBetween(rNextUV0) ;
      double       dStepUV1  = rTestUV1.DistanceBetween(rNextUV1) ;

      // skip ThroughPoints far from TestPoint or across closed boundaries
      if(   dStepDist > dLastStepDist
         || dStepUV0  > dLastStepUV0
         || dStepUV1  > dLastStepUV1) 
        { continue; } // Too far away

      // Project TestPoint to NextPoint line
      double dT;
      SER(smgu_LineClosestPoint(rNextPnt,rNextVec,rTestPnt,dT));
      SmPoint3d sClosestPnt = rNextPnt + rNextVec*dT;
      double dDist    = rTestPnt.DistanceBetween(sClosestPnt);

      // when TestPoint is near NextPoint line
      if (dDist < 10.0*m_dThisApproxTol3d) 
        {
          // set NextTSP = pTSP
          SER(sm_UpdateTsectPoint(pPrev1, rNextTSP, pTSP, *this));

          // when ThroughPoint is not the endPoint
          if (pTSP != m_pEndPoint) 
            {
              // 
              if (m_vThroughPoints.GetFirstNode() != NULL) 
                {
                  m_vThroughPoints.Remove(pTSP);
                }

              // 
              if (   m_bDoingExtensionClipping 
                  && m_vThroughPoints.GetLastNode() == NULL) 
                {
                  m_dLastThroughParmeter = pTSP->m_dCurveParameter;
                }
              m_bClippedByThroughPoint = TRUE;
            }
        } // when TestPoint is close to NextPoint check
    } // end iter every through point

  // all done
  return SM_SUCCESS;

} // end SmSurfaceIntersector::TestPoint

/*******************************************************************//**
PURPOSE: Trace a curve from the given starting point placing
 a sequence of SmTsectPnts on the m_vCurvePoints list.  

NOTES: 
  Adds a sequence of SmTsectPnts to m_vCurvePoints.

METHOD --- 

  1. Special Case: No points in m_vCurvePoints and start point is Tangent or Coincident.
     return 1 point array in m_vCurvePoints.
  2. if(m_vCurvePoints is empty) Place StartPoint in m_vCurvePoints
  3. Test expected next point for surface containment :
      Surface1_UVPosition[1] = Surface1_UVPosition[0] + 10*EFF_ZERO*Surface1_UVTangent[0]
      Surface2_UVPosition[1] = Surface2_UVPosition[0] + 10*EFF_ZERO*Surface2_UVTangent[0]
     If next points are out of bounds - reverse trace direction - when allowed.
  4. Begin trace iteration
    4a. Get last node
    4b. let dStepSize = last step size if given else 100 * m_dThisApproxTol3d ;
    4c. allocate/init a new SmTsectPnt object
    4d. ComputeNextPoint(last_point, stepSize, next_point,...)
        given a stepSize, 
        find pointClassification (Good/Bad, Clipped, HitBoundary), deviation, and angle
    4e. while(!bFoundGoodPoint): Search for a GoodPoint iteration: 
             decrease the stepSize and retry ComputeNextPoint() until 
                it gets close to zero - then quit trace iteration
             or bFoundGoodPoint == TRUE
    4f. With a GoodPoint without clipping and tight tolerances
        Begin largest step goodPoint search: while(bFoundGoodPoint && !bClipped)
           increase step size
           retry ComputeNextPoint()
             if point is bad - quit searching for larger steps
             if point is reset to last position - probably on a boundary - quit searching
           See if pNextTSP is close to a specified end-point, TestPoint().
             if point is a designated endPoint - AddPointToCurve and return
           if last stepIncrease gave a badPoint
             Step back to last good point - quit searching for larger steps
    4g. if UV or XYZ step size is too small - set bDone = TRUE
        else if 2nd Point and we are retracing an existing xsect curve - set bDone = TRUE
    4h. if bDone == FALSE
           see if the point is near to any of the specified end-points, TestPoint()
           set pNextTSP deviation and
           AddPointToCurve()
    4i. increment point list count

          

SIDE EFFECTS ---
  m_vCurvePoints:
    place sequence of SmTsectPnts on the m_vCurvePoints list 
    marking an xsect curve.  0, 1, or more points are possible.
      Each SmTsectPnt stores
        1. parameter value.
        2. 3D position and tangent.
        3. UV position and tangent on each defining surface.
        4. 3D position, 1st deriv, 2nd deriv, and normals on each defining surface.
        5. angle between surface normals
        6. max deviation between 3D positions
        7. an SmIntersectionPointType, SM_IP_CROSSING, SM_IP_TANGENT_POINT, ...
        8. Some user data, an array[4] of doubles, and two NULL pointers.

   m_dCurveTraceDirection:
     gets negated if the first step from startPoint is out of bounds - when allowed.

   m_bCurveIsClosed:
     initialized to FALSE

***********************************************************************/
SmStatus 
SmSurfaceIntersector::TraceCurve
  (SmTsectPnt         & rStartPoint,      // in : Location to start next xsect curve trace
   SmTArray<SmCurve*> & r3DCurves)        // in : accumulation of all xsect curves used to
                                          //      prevent tracing out an already found intersection.
{
#ifdef SM_DEBUG_CODE
int iTraceDebug = 0; // 1-5: only accepted points;  > 5: all trial points.
SmBoolean bDebugMe = FALSE;
static ULONG lCount = 1 ; lCount ++ ;
static ULONG lDebugCount = 0 ;
  // draw Breps(blue,green), Surfaces(cyan,yellow), faces(black), StartPoint(red)
  if (bDebugMe || lDebugCount == lCount) 
    {
      rStartPoint.Dump();

      SmFace *pFace1 = (SmFace *)m_cpSurface[0]->GetFace() ;
      SmFace *pFace2 = (SmFace *)m_cpSurface[1]->GetFace() ;
      SmBrep *pBrep1 = pFace1 ? pFace1->GetBrep() : NULL ;
      SmBrep *pBrep2 = pFace2 ? pFace2->GetBrep() : NULL ;
      SmPoint3d sPoint ; 
      m_cpSurface[0]->EvaluatePoint(rStartPoint.UVPos(0),sPoint) ;

      smgfx_Erase();
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep1) { pBrep1->Draw(TRUE) ; } sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,0) ; if(pBrep2) { pBrep2->Draw(TRUE) ; } sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,1) ; m_cpSurface[0]->DrawUV(4,4,FALSE,NULL,pBrep1==NULL); sm_GraphicsLoop();
      smgfx_SetLook(1,2, 1,1,0) ; m_cpSurface[1]->DrawUV(4,4,FALSE,NULL,pBrep2==NULL); sm_GraphicsLoop();
      smgfx_SetLook(1,2, 0,0,0) ; if(pFace1) { pFace1->Draw(SM_DM_CROSSHATCH) ; } sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,0,0) ; if(pFace2) { pFace2->Draw(SM_DM_CROSSHATCH) ; } sm_GraphicsLoop() ;
      smgfx_SetLook(6,8, 1,0,0) ; rStartPoint.Draw() ; sm_GraphicsLoop() ;

      smgfx_SetLook(2,4, 1,0,0);
      ULONG jjj, lNumCrvs = r3DCurves.GetSize();
      for (jjj=0; jjj<lNumCrvs; jjj++ )
        {
          r3DCurves[jjj]->Draw(); sm_GraphicsLoop();
        }
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE
  // If we are starting on a tangent point just quit here and
  //   don't try to trace the curve.  It should create a single 
  //   point degenerate curve.
  // If this is the first point in curve do the proper initilization

  // scale tolerances by UV domain size
  double dUVTol_0 = SM_EFF_ZERO * (1.0 + m_vUVDomain[0].GetMaxDimension()) ;
  double dUVTol_1 = SM_EFF_ZERO * (1.0 + m_vUVDomain[1].GetMaxDimension()) ;

  //SmBoolean bForwardTrace = FALSE;

  // when there are no points on the m_vCurvePoints list 
  // pick a bForwardTrace walking direction from the starting point
  if (m_vCurvePoints.GetLastNode() == NULL) 
    {
      //bForwardTrace = TRUE;

      // 1. Special Case: when the given start point is a Tangent or Coincidence point
      if (   rStartPoint.m_ePointType == SM_IP_TANGENT_POINT 
          || rStartPoint.m_ePointType == SM_IP_COINCIDENCE) 
        {
          // when the point is NOT on a previously found xsect curve 
          if (!IsPointOnCurve(rStartPoint.CrvPos(),r3DCurves)) 
            {
              // place rStartPoint on the m_vCurvePoints list making a single point xsect curve
              rStartPoint.m_dCurveParameter = 0.0; // curves start at 0.0
              rStartPoint.m_dDeviation      = 0.0;
              m_vCurvePoints.Append(&rStartPoint);
              m_bCurveIsClosed              = TRUE; // Closed curve finishes the job
            } // end PointNotOnCurve Check

          // return having created a 0 or 1 point count array in m_vCurvePoints
          return SM_SUCCESS;

        } // end startPoint of type SM_IP_TANGENT_POINT or SM_IP_COINCIDENCE check
      
      // 2. when rStartPoint type is not TANGENT_POINT or COINCIDENCE
      //    place the start point on the m_vCurvePoints array
      rStartPoint.m_dCurveParameter = 0.0; // curves start at 0.0
      rStartPoint.m_dDeviation      = 0.0;
      m_vCurvePoints.Append(&rStartPoint);
      
      // 3. Check to see if expected first step is out of bounds -- 
      //    If it is try reversing the direction - when allowed.

      // let nextPointUV = curPosUV + TolSizeStep * curDirUV - for both surfaces 
      SmPoint2d sDelta_0 = rStartPoint.UVPos(0) + (10.0 * SM_EFF_ZERO_SQRT) * rStartPoint.UVDeriv(0);
      SmPoint2d sDelta_1 = rStartPoint.UVPos(1) + (10.0 * SM_EFF_ZERO_SQRT) * rStartPoint.UVDeriv(1);

      // when NextPointUV is out of bounds for either surface - reverse directions when not on a boundary or quit
      if(   !m_vUVDomain[0].ContainsPoint2d(sDelta_0, dUVTol_0)
         || !m_vUVDomain[1].ContainsPoint2d(sDelta_1, dUVTol_1)) 
        {
          // if start point is on a surface parameter bound - output a single point curve
          // (GWC: Switching trace direction should step out of bounds if IsBounded is true)
          // (JGU: Test modified in Rev 9059)
          if ( rStartPoint.IsBounded() && IsPointOnCurve( rStartPoint.CrvPos(), r3DCurves ) )
          {
              // remove the point from the m_vCurvePoints array - no curve to add here
              m_vCurvePoints.Init();
              // return having created 1 or 0 point count array in m_vCurvePoints
              return SM_SUCCESS;
          }
          else // try reversing the trace direction
            {
              m_dCurveTraceDirection *= -1.0;  // Reverse curve tracing direction
              SER(ReverseCurveDirection());    // reverse m_vCurvePoints order (if any)
                                               //   negate tangent vectors, reassign parameters
                                               //   so oldParamRange[0,max] is the same as
                                               //      newParamRange[0,max] (just running in opposite dirs]
            }
        } // end next step is out of Surface1 UV bounds branch
    } // end getlastNode == NULL, need to select trace direction check
  
  // GWC_NEEDS_WORK rStartPoint_should_be_Checked_for_coincidence_with_3dCurves_before_being_traced__allowing_for_singularities_and_direction GWC_LINE ;

  // Arrive here when curve needs to be traced (it's not a single point curve),
  //                  The startPoint is added to the m_vCurvePoints array, and  
  //                  m_dCurveTraceDirection is set to + or - 1.0 
  //                        based on a small stepping out-of-bounds check.

  // Curve is not closed prior to tracing
  m_bCurveIsClosed = FALSE;
  
  // init number of points saved for this curve
  ULONG lPointCount = 0;
  
  // Init trace parameters
  double    dStepSize      = 0.0;
  SmBoolean bDone          = FALSE;
  double    dDeviation     = 0.0;
  SmTsectPnt *pNextTSP     = NULL;
  SmBoolean bHitBoundary;

  // 4. iterate intersection trace - finding and adding points to m_vCurvePoints array
  //     stop when - step out of bounds
  while (!bDone) 
    {
      // 4a. get tracedCurve's last node
      SmTsectPnt *pTSP = m_vCurvePoints.GetLastNode();

      // 4b. let dStepSize = last step size if given, else 100 * m_dThisApproxTol3d ;
      //     step size is used in the surface UV domains to get next UV guess points
      SER(ComputeStepSize(*pTSP,dStepSize,dStepSize));

      // 4c. allocate/init a new SmTsectPnt object
      pNextTSP  = (SmTsectPnt*)m_vTSPntMgr.GetNewElement();
      pNextTSP->m_ePointType = SM_IP_UNKNOWN;
      SmBoolean bFoundGoodPoint;
      double    dAngleFound;
      SmBoolean bClipped;

      // 4d. Compute the next xsect curve point for current dStepSize 
      //        - virtual function (SmFilletIntersector/SmSurfaceIntersector) to 
      //     let next UVs = current UVs + stepSize * current UV_Tangents
      //                (one UV for each surface)
      //          Refine UVs with a surf/surf xsect call
      //          compute NextTSP values at the refined point
      //                (3D pos & tang, UV pos & tang, type, etc.)
      //          pick NextTSP param value = surface1 3DCurve arcLength parameterization
      //          bFoundGoodPoint = TRUE when next point satisifies all tol criteria:
      //            1. no boundaries were hit
      //            2. the point type is not singluar or tangent point  GWC:tangent makes sense, but why not singular?
      //            3. the step size from pTSP to the refined pNextTSP is bigger than 0.0
      //            4. angleBetween(pTSP.3DTangent, pNextTSP.3DTangent) < m_dThisAngTolRad
      //            5. max approx3DCurve/approxUVTrimCurve dist < m_dThisApproxTol3d/2.0
      //               at 5 sample points along hermite Curves defined by the
      //                pTSP and pNextTSP pos and tang values in 3D and UV space.

      // GWC_NEEDS_WORK are_singular_points_really_BadPoints__could_be_bug_275__See_comment_above GWC_LINE ;
      if (SM_SUCCESS != ComputeNextPoint(*pTSP, dStepSize, 
                                         *pNextTSP,  bFoundGoodPoint, 
                                         dDeviation, dAngleFound,
                                         bClipped,   bHitBoundary))
        {
          // ComputeNextPoint() Error return indicates that it can never succeed.
          bFoundGoodPoint = FALSE;
          break ; // break = jump out of while loop and exit, returning SM_SUCCESS ; 
        }

#ifdef SM_DEBUG_CODE
      if ( bDebugMe || iTraceDebug > 5) 
        {
          smgfx_SetLook(6,3, 0,0,1) ; pNextTSP->Draw(); sm_GraphicsLoop();
          sm_GraphicsLoop();
        }
#endif // SM_DEBUG_CODE

      // when the next guess steps out of bounds - stop tracing
      if (bHitBoundary) 
        {
          // and current stepSize is modest
          if (dStepSize < m_dThisApproxTol3d * 100.0)
            {
              // quit iterating curve trace
              break; // break = jump out of while loop and exit, returning SM_SUCCESS ;
            }
        }

      // when a next point was found - verify next point is in surface boundaries
      if (bFoundGoodPoint)
        {
          // verify that NextPoint is contained in both surfaces - could be an assert
          if(!m_vUVDomain[0].ContainsPoint2d(pNextTSP->UVPos(0), dUVTol_0 ) )
            { SER(SM_ERR); }
          if(!m_vUVDomain[1].ContainsPoint2d(pNextTSP->UVPos(1), dUVTol_1 ) )
            { SER(SM_ERR); }
        }
            
      SmBoolean bDecreasedStepSize = FALSE;

      // when the nextPoint is not a good xsect point
      // - try finding a good one by taking smaller and smaller step sizes
      while (!bFoundGoodPoint) 
        {
          // decrease the stepSize until it gets close to zero

          // Note: Could do a bisection here.  I tried it and it got the
          // stepsizes much closer to optimal without much extra work.
          // It caused one fillet regression to succeed (2:200), but caused
          // three others to fail (2:226,228,236), so for now let's leave it
          // as it is.  Maybe someday we could investigate.  [bd 06Jan06]

          dStepSize /= 1.5;
          if (dStepSize < SM_EFF_ZERO*100.0) 
            {
              // We have hit something nasty here if we have enough
              // just quit looking for a good point
              // and check things out down below.
              break ; // break = jump out of while loop and exit, returning SM_SUCCESS ;
            }

          // remember the stepSize was decreased
          bDecreasedStepSize = TRUE;

          // retry ComputeNextPoint with reduced stepSize
          if(   ComputeNextPoint(*pTSP, dStepSize, *pNextTSP, bFoundGoodPoint,
                                 dDeviation, dAngleFound, bClipped, bHitBoundary )
             != SM_SUCCESS)
          {
              bFoundGoodPoint = FALSE;
              break;  // Error return indicates that it can never succeed.
          }

#ifdef SM_DEBUG_CODE
          // Trace debug: backing up.
          if ( bDebugMe || iTraceDebug > 5) 
            {
              smgfx_SetLook(6,3, 0,0,1) ; pNextTSP->Draw(); sm_GraphicsLoop();
              sm_GraphicsLoop();
            }
#endif // SM_DEBUG_CODE

          // when smaller stepSize found a good next point - verify its in both surfaces
          if (bFoundGoodPoint) 
            {
              if ( !m_vUVDomain[0].ContainsPoint2d(pNextTSP->UVPos(0),dUVTol_0))
                { SER(SM_ERR); }
              if ( !m_vUVDomain[1].ContainsPoint2d(pNextTSP->UVPos(1),dUVTol_1))
                { SER(SM_ERR); }
            }

        } // end while decreasing stepSize searching for a goodpoint because big steps are failing
      
      // when a good next point could not be found even with smaller step sizes 
      //  - quit iterating curve trace
      if (!bFoundGoodPoint) 
        { break ; } // break = jump out of while loop and exit, returning SM_SUCCESS ; 
        
      // Found a good point.  Before trying to increase step,
      // check whether we've stepped past a specified through-point or end-point,
      // or there's some other stopping criterion.
      // If we've passed a through-point or end-point (or other terminating
      // condition), TestPoint() will snap pNextTSP to it, thus adjusting
      // its m_dCurveParameter -- that's how we check its results.
      // If TestPoint() decides that this is the end of the curve,
      // it will set bDone to True.

#ifdef SM_DEBUG_CODE
      SmTsectPnt sNextCopyTSP = *pNextTSP ;
#endif // SM_DEBUG_CODE

      double dLastCurveParam = pNextTSP->m_dCurveParameter;

      // Snap NextTSP to nearby m_vThroughPoints, m_vStartPoints, m_pStartPoint, m_pEndPoint
      //   Set bDone == TRUE: when NextTSP was StartPoint, EndPoint, or a ThroughPoints,
      //                        or rNextTSP m_dCurveParameter is larger than m_dLastThroughParmeter + m_dExtensionDistance when m_bDoingExtensionClipping == TRUE 
      //                        or NExtTSP on boundary heading out of domain 
      SER( TestPoint( *pNextTSP, bDone ));  

#ifdef SM_DEBUG_CODE
      if ( bDebugMe ) 
        {
          smgfx_SetLook(6,3, 0,0,1) ; sNextCopyTSP.Draw(); sm_GraphicsLoop();
          smgfx_SetLook(6,3, 1,0,0) ; pNextTSP->Draw(); sm_GraphicsLoop();
          sm_GraphicsLoop();
        }
#endif // SM_DEBUG_CODE

      if (   bDone 
          || pNextTSP->m_dCurveParameter != dLastCurveParam )
        {
          SER( AddPointToCurve( *pNextTSP ));

          if ( bDone ) // Done with this curve.
            { 
              break ; // break = jump out of while loop and exit, returning SM_SUCCESS ;
            }
  
          continue ; // go on to next step.
        }

      // to minimize number of xsect points - try looking for next points with bigger step sizes.

      // When the stepSize was not clipped to stay on the surfaces
      //      and the stepsize was not decreased to find a good next point
      //      and the current deviations are small compared to the limits,
      // then try increasing the step size.
      if (   !bClipped 
          && !bDecreasedStepSize 
          &&  dDeviation  < m_dThisApproxTol3d / 8.0
          &&  dAngleFound < m_dThisAngTolRad    / 4.0 ) 
        {
          // save the lastCurveParam
          dLastCurveParam = pNextTSP->m_dCurveParameter;

          // iterate looking for the largest stepSize increase possible
          while (bFoundGoodPoint && !bClipped) 
            {
              // increase stepSize geometrically
              dStepSize *= 1.5;

              // get NextPoint for given stepSize (set bFoundGoodPoint
              //   == TRUE when NextPoint passes all usability tests)
              // No: [SMS13]  pNextTSP->m_ePointType = pTSP->m_ePointType ;
              if ( ComputeNextPoint(*pTSP, dStepSize, *pNextTSP, bFoundGoodPoint,
                                    dDeviation, dAngleFound, bClipped, bHitBoundary )
                    != SM_SUCCESS )
                {
                  // Error return indicates that it can never succeed.
                  bFoundGoodPoint = FALSE;
                  break ;  // break = end while looking for larger spaced good points
                }

              // quit increasing step sizes after finding a point that is no good
              if (!bFoundGoodPoint) 
                { continue; }

              // Found a good point.

#ifdef SM_DEBUG_CODE
              if ( bDebugMe || iTraceDebug > 5) 
                {
                  smgfx_SetLook(6,3, 0,0,1) ; pNextTSP->Draw(); sm_GraphicsLoop();
                  sm_GraphicsLoop();
                }
#endif // SM_DEBUG_CODE

              // assert goodPoints are in both surfaces
              if ( !m_vUVDomain[0].ContainsPoint2d( pNextTSP->UVPos(0), dUVTol_0) )
                  { SER(SM_ERR); }
              if ( !m_vUVDomain[1].ContainsPoint2d( pNextTSP->UVPos(1), dUVTol_1) )
                  { SER(SM_ERR); }

              // when refined stepSize is zero, mark the point as bad and quit
              // its probably on a boundary
              if (pNextTSP->m_dCurveParameter <= dLastCurveParam + SM_EFF_ZERO) 
                {
                  bFoundGoodPoint = FALSE;
                  break ; // break = end while looking for larger spaced good points
                }

              dLastCurveParam = pNextTSP->m_dCurveParameter;

              // See if the pNextTSP has passed a specified end-point or
              // through-point, or has run into a surface/surface
              // singularity (parallel normals).
#ifdef SM_DEBUG_CODE
              sNextCopyTSP = *pNextTSP ;
#endif // SM_DEBUG_CODE
              SER(TestPoint(*pNextTSP,bDone));

#ifdef SM_DEBUG_CODE
              if ( bDebugMe ) 
                {
                  smgfx_SetLook(6,3, 0,0,1) ; sNextCopyTSP.Draw(); sm_GraphicsLoop();
                  smgfx_SetLook(6,3, 1,0,0) ; pNextTSP->Draw(); sm_GraphicsLoop();
                  sm_GraphicsLoop();
                }
#endif // SM_DEBUG_CODE
              // If it passed a through point, TestPoint() will snap
              // pNextTSP->m_dCurveParameter back to the through point.
              // That's how we know it passed one.

              if (   bDone 
                  || pNextTSP->m_dCurveParameter != dLastCurveParam) 
                {
                  // TestPoint() snapped to something.
                  if (bDone) 
                    {
                      // TestPoint() detected an end condition: done tracing.
                      SER(AddPointToCurve(*pNextTSP));
#ifdef SM_DEBUG_CODE
                      // Trace debug: final point.
                      if ( iTraceDebug > 0 ) 
                        {
                          smgfx_SetLook( 3,6, 0,0,0 ); pNextTSP->Draw(); sm_GraphicsLoop();
                          sm_GraphicsLoop();
                        }

                      // Step Debug:
                      if (bDebugMe) 
                        {
                          SmFace *pFace1 = (SmFace *)m_cpSurface[0]->GetFace() ;
                          SmFace *pFace2 = (SmFace *)m_cpSurface[1]->GetFace() ;
                          SmBrep *pBrep1 = pFace1 ? pFace1->GetBrep() : NULL ;
                          SmBrep *pBrep2 = pFace2 ? pFace2->GetBrep() : NULL ;
                          SmTArray<SmTsectPnt *> sTPnts ;
                          m_vCurvePoints.GetAllNodes(sTPnts) ;

                          smgfx_Erase();
                          smgfx_SetLook(1,2, 0,0,1) ; if(pBrep1) { pBrep1->Draw(TRUE) ; } sm_GraphicsLoop() ;
                          smgfx_SetLook(1,2, 0,1,0) ; if(pBrep2) { pBrep2->Draw(TRUE) ; } sm_GraphicsLoop() ;
                          smgfx_SetLook(1,2, 0,1,1) ; m_cpSurface[0]->DrawUV(4,4,FALSE,NULL,pBrep1==NULL); sm_GraphicsLoop();
                          smgfx_SetLook(1,2, 1,1,0) ; m_cpSurface[1]->DrawUV(4,4,FALSE,NULL,pBrep2==NULL); sm_GraphicsLoop();
                          smgfx_SetLook(1,2, 0,0,0) ; if(pFace1) { pFace1->Draw(SM_DM_CROSSHATCH) ; } sm_GraphicsLoop() ;
                          smgfx_SetLook(1,2, 0,0,0) ; if(pFace2) { pFace2->Draw(SM_DM_CROSSHATCH) ; } sm_GraphicsLoop() ;
                          smgfx_SetLook(6,8, 1,0,0) ; rStartPoint.Draw() ; sm_GraphicsLoop() ;
                          smgfx_SetLook(6,8, 0,0,1) ; pNextTSP->Draw(); sm_GraphicsLoop();
                          ULONG ii, cnt = sTPnts.GetSize() ; 
                          for(ii=0;ii<cnt;ii++)
                            { double dD = (double)ii/(double)(cnt > 1 ? cnt-1 : cnt) ;
                              smgfx_SetLook(3,4, 0,1-dD,dD) ; sTPnts[ii]->Draw() ; sm_GraphicsLoop() ;
                            }
                          sm_GraphicsLoop();

                        }
#endif // SM_DEBUG_CODE
                      return SM_SUCCESS;
                    } // end if bDone: done tracing curve.

                  // TestPoint() snapped to something, but didn't say we're done
                  // tracing this curve.  Break out of increasing-step-size loop.

                  //bFoundGoodPoint = FALSE;
                  break;

                } // end if TestPoint() snapped to something.
            } // end while looking for larger spaced good points

          // if last stepsize increase gave a bad-point
          if ( !bFoundGoodPoint )
            {
              //  step back to last good one
              dStepSize /= 1.5;

              // No: [SMS13]  pNextTSP->m_ePointType = pTSP->m_ePointType ;
              SER(ComputeNextPoint(*pTSP, dStepSize, *pNextTSP, bFoundGoodPoint,
                                   dDeviation, dAngleFound, bClipped, bHitBoundary));

              if (bFoundGoodPoint) 
                {
                  if (!m_vUVDomain[0].ContainsPoint2d(pNextTSP->UVPos(0),dUVTol_0)) 
                    { SER(SM_ERR); }
                  if (!m_vUVDomain[1].ContainsPoint2d(pNextTSP->UVPos(1),dUVTol_1)) 
                    { SER(SM_ERR); }
                } // end goodPoint check
            } // end last increase gave a badPoint check
        } // end try for bigger stepsize check

      // arrive here after setting bFoundGoodPoint and pNextTSP from last step, pTSP and stepsize.
      //   bFoundGoodPoint == TRUE nextPoint has been found after
      //      chances to increase or decrease the stepsize to minimize
      //      total point count and to hit endCurve targets directly.

      // Reset m_bClippedByThroughPoint
      m_bClippedByThroughPoint = FALSE;

#ifdef SM_DEBUG_CODE
      // draw StartPoint(red), NextPoint(Magenta), Breps(blue,green), surfaces(cyan,yellow)
      if (bDebugMe) 
        {
          SmFace *pFace1 = (SmFace *)m_cpSurface[0]->GetFace() ;
          SmFace *pFace2 = (SmFace *)m_cpSurface[1]->GetFace() ;
          SmBrep *pBrep1 = pFace1 ? pFace1->GetBrep() : NULL ;
          SmBrep *pBrep2 = pFace2 ? pFace2->GetBrep() : NULL ;
          SmTArray<SmTsectPnt *> sCurvePoints ;
          m_vCurvePoints.GetAllNodes(sCurvePoints) ;

          m_vCurvePoints.Dump() ;

          smgfx_Erase();
          smgfx_SetLook(1,2, 0,0,1) ; if(pBrep1) { pBrep1->Draw(TRUE) ; } sm_GraphicsLoop() ;
          smgfx_SetLook(1,2, 0,1,0) ; if(pBrep2) { pBrep2->Draw(TRUE) ; } sm_GraphicsLoop() ;
          smgfx_SetLook(1,2, 0,1,1) ; m_cpSurface[0]->DrawUV(4,4,FALSE,NULL,pBrep1==NULL); sm_GraphicsLoop();
          smgfx_SetLook(1,2, 1,1,0) ; m_cpSurface[1]->DrawUV(4,4,FALSE,NULL,pBrep2==NULL); sm_GraphicsLoop();
          smgfx_SetLook(3,4, 1,0,0) ; rStartPoint.Draw(); sm_GraphicsLoop() ;
          smgfx_SetLook(4,6, 0,1,0) ; for(ULONG ii=0;ii<sCurvePoints.GetSize();ii++)
                                         { sCurvePoints[ii]->Draw(); sm_GraphicsLoop() ; }
          smgfx_SetLook(4,6, 1,0,1) ; pNextTSP->Draw(); sm_GraphicsLoop() ;
          sm_GraphicsLoop();
        }
#endif // SM_DEBUG_CODE

      // 4g. If step size gets too small we have a problem just stop when
      // we get real close to the problem.
      // Check both parameter space (dStepSize) and 3d step.
      //dLastValidStep = dStepSize;
      if ( dStepSize < SM_EFF_ZERO*100.0 )
        {
          bDone = TRUE;
          break;  // out of big loop: done tracing.
        }

      // get 3DCurve arcLength and a scaled tolerance
      double dDistMoved = pNextTSP->CrvPos().DistanceBetween(pTSP->CrvPos());
      double dTol = SM_EFF_ZERO * 100.0 * (1.0 + pTSP->CrvPos().GetMaxDimension());

      if ( dDistMoved < dTol )
        { 
          bDone = TRUE; 
          break;  // out of big loop: done tracing.
        }

      // If this is the second point in this trace,
      // check it against previous curves to see if we have
      // already traced this one.
      if (     // bForwardTrace &&    This was causing two coincident curves being returned. [B659]
             lPointCount == 0  // 2nd point
          && (pNextTSP->m_ePointType != SM_IP_SINGULARITY 
          && IsPointOnCurve(pNextTSP->CrvPos(),r3DCurves))) 
        {
          // unclipped points have to be on curves already traced
          if ( !bClipped )
            {
              bDone = TRUE;
              break;  // out of big loop: done tracing.
            }

          // need to test a mid point or something to make sure
          SmHermiteCurve s3DCrv(pTSP->CrvPos(),
                                pTSP->CrvDeriv()*dStepSize,
                                pNextTSP->CrvPos(),
                                pNextTSP->CrvDeriv()*dStepSize);
          s3DCrv.SetContext(NULL);
          SmPoint3d sMidPnt;
          SER(s3DCrv.EvaluatePoint(0.5,sMidPnt));
          if (IsPointOnCurve(sMidPnt,r3DCurves)) 
            {
              bDone = TRUE;
              break;  // out of big loop: done tracing.
            }
        } // end second point is on a previously traced xsect curve check
      
      // Check whether we've stepped past a specified through-point or end-point.
      // If we've passed a through-point or end-point, TestPoint() will
      // snap pNextTSP to it -- thus adjusting its m_dCurveParameter.
      // If passed an end-point, TestPoint() will set bDone to True.

      if ( !bDone )
        {
          double dSaveParam = pNextTSP->m_dCurveParameter;

          SER(TestPoint(*pNextTSP,bDone));

          // when TestPoint moved the pNextTSP
          if ( dSaveParam != pNextTSP->m_dCurveParameter )
            {
              //dLastValidStep = 0.0;
            }

          // save the deviation and add Point to m_vCurvePoints point list
          pNextTSP->m_dDeviation = dDeviation;
          SER( AddPointToCurve( *pNextTSP ));

#ifdef SM_DEBUG_CODE
          // Trace debug: accepted point.
          if ( iTraceDebug > 0 ) 
            {
              smgfx_SetLook( 3,6, 0,0,0 ); pNextTSP->Draw(); sm_GraphicsLoop();
              sm_GraphicsLoop();
            }
#endif // SM_DEBUG_CODE

        } // end not-passed-an-end-point check

      // 4i. increment the m_vCurvePoints point list count
      lPointCount++;

      // warn the public of excessive point counts
      switch(lPointCount)
        {
          case 1000: MSG(_T("Warning More than 1000 points in intersection curve"));
                     break ;
          case 2000: MSG(_T("Warning More than 2000 points in intersection curve"));
                     break ;
          case 5000: MSG(_T("Failing - 5000 Points in Surface Intersection curve"));
                     MSG(_T("The tolerance is too small or a surface is poorly defined"));
                     SER(SM_ERR); // Better get out of here this curve is getting
                     // ridiculus - need to do something about the surface quality
                     // or increase the tolerance.
                     break ;
          default:   break ;
        } // end switch on lPointCount
      
    } // end while iterating trace curve finding and adding points to m_vCurvePoints array

#ifdef SM_DEBUG_CODE
  // Trace debug: final point.
  if ( iTraceDebug > 0 ) 
    {
      smgfx_SetLook( 3,6, 0,0,0 ); pNextTSP->Draw(); sm_GraphicsLoop();
      sm_GraphicsLoop();
    }

  // Step debug: draw Breps(blue,green), Surfaces(cyan,yellow), faces(black),
  //                  StartPoint(red), m_vCurvePoints(green-blue)
  if (bDebugMe || lDebugCount == lCount) 
    {
      SmFace *pFace1 = (SmFace *)m_cpSurface[0]->GetFace() ;
      SmFace *pFace2 = (SmFace *)m_cpSurface[1]->GetFace() ;
      SmBrep *pBrep1 = pFace1 ? pFace1->GetBrep() : NULL ;
      SmBrep *pBrep2 = pFace2 ? pFace2->GetBrep() : NULL ;
      SmTArray<SmTsectPnt *> sTPnts ;
      m_vCurvePoints.GetAllNodes(sTPnts) ;

      smgfx_Erase();
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep1) { pBrep1->Draw(TRUE) ; } sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,0) ; if(pBrep2) { pBrep2->Draw(TRUE) ; } sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,1) ; m_cpSurface[0]->DrawUV(4,4,FALSE,NULL,pBrep1==NULL); sm_GraphicsLoop();
      smgfx_SetLook(1,2, 1,1,0) ; m_cpSurface[1]->DrawUV(4,4,FALSE,NULL,pBrep2==NULL); sm_GraphicsLoop();
      smgfx_SetLook(1,2, 0,0,0) ; if(pFace1) { pFace1->Draw(SM_DM_CROSSHATCH) ; } sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,0,0) ; if(pFace2) { pFace2->Draw(SM_DM_CROSSHATCH) ; } sm_GraphicsLoop() ;
      smgfx_SetLook(6,8, 1,0,0) ; rStartPoint.Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook(6,8, 0,0,1) ; pNextTSP->Draw() ; sm_GraphicsLoop() ;
      ULONG ii, cnt = sTPnts.GetSize() ; 
      for(ii=0;ii<cnt;ii++)
        { double dD = (double)ii/(double)(cnt > 1 ? cnt-1 : cnt) ;
          smgfx_SetLook(3,4, 0,1-dD,dD) ; sTPnts[ii]->Draw() ; sm_GraphicsLoop() ;
        }
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

  // all done
  return SM_SUCCESS;

} // end SmSurfaceIntersector::TraceCurve

/*******************************************************************//**
PURPOSE: Flush out the curve defined in the control point and knots arrays
             and create a B-Spline and put it into the curves array.

NOTES: 
***********************************************************************/
static SmStatus sm_FlushCrv
  (const SmContext & crContext,          // in : context for new SmBSplineCurve
   ULONG lDimension,                     // in : Control Point image dimension
   SmTArray<SmPoint3d> & rCntrlPoly,     // in : Array of Control Points
   SmTArray<double> & rKnots,            // in : Array of Unique Knots
   SmTArray<SmCurve*> & crCurves)        // out: new SmBsplineCurve placed at end of array
{
    // no work - no knots
    if (rKnots.GetSize() == 0) return SM_SUCCESS;

    // make a piecewise C0, degree 3, knot multiplicity array
    // 4 knots on each end, 3 knots for each internal knot
    SmTArray<ULONG> sKnotMult(rKnots.GetSize(),NULL,rKnots.GetSize());
    sKnotMult[0] = 4;
    for (ULONG i=1; i+1<sKnotMult.GetSize(); i++)  // note: can't say sKnotMult.GetSize()-1
      {
        sKnotMult[i] = 3;
      }
    sKnotMult[sKnotMult.GetSize()-1] = 4;

    // make the degree 3 SmBsplineCurve
    SmBSplineCurve *pNewBSP = NULL ;
    ULONG lDegree = 3;
    SER(SmBSplineCurve::CreateCanonical(crContext,lDimension,lDegree,
        rCntrlPoly, SM_CF_UNSPECIFIED, sKnotMult, rKnots, 
        SM_KT_UNSPECIFIED, NULL, NULL, pNewBSP));

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
    if (bDebugMe) {
        sm_GraphicsLoop();
        pNewBSP->DrawPolygon();
        sm_GraphicsLoop();
    }
    if (FALSE) {
        pNewBSP->Dump();
    }
#endif

// Note that the following line will remove extra knots and
// decrease the size of the curve.  This can easily be done
// after drop curve.
// GWC:NOTE No knots are currently removed because all the curves
//          associated with an intersection (1 3DCurve, 2 UVTrimCurves)
//          are returned with the same parameterization.  Unless all 3 curves
//          are reduced in the same fashion the returned curves would
//          have different parameterizations.  So no reduction is done.
//    SER(pNewBSP->RemoveExtraKnots(SM_EFF_ZERO));
#ifdef SM_DEBUG_CODE
    if (FALSE) {
        pNewBSP->Dump();
    }
SmBoolean bDebugMe2 = FALSE;
    if (bDebugMe2) {
        sm_GraphicsLoop();
        pNewBSP->DrawPolygon();
        sm_GraphicsLoop();
    }
#endif

    crCurves.Add(pNewBSP);
    return SM_SUCCESS;

} // end sm_FlushCrv


/*******************************************************************//**
PURPOSE: Flush the curve which currently exists in m_vCurvePoints
    list as 3 output curves, a 3DCurve and 2 UVTrimCurves along with
    a CurveType and Deviation value where deviation is max distance
    from 3DCurve to surfaces.

NOTES:
  There are 3 output curves: 1 3DCurve and 2 UVTrimCurves.
  All output curves are degree 3 non-rational BSplines.
  All output curves are piecewise C0, that is all internal knots 
    have a multiplicity equal to degree.
  All output curves share a common parameterization, that is they have the 
    same knot vectors.

  The outputs accumulate in the output arrays.
***********************************************************************/
SmStatus 
SmSurfaceIntersector::FlushCurve
  (SmTArray<SmCurve*> & r3DCurves,            // out: 3D intersection curve
   SmTArray<SmCurve*> & rSurface1UVCurves,    // out: 1st surface UVTrimCurve
   SmTArray<SmCurve*> & rSurface2UVCurves,    // out: 2nd surface UVTrimCurve
   SmTArray<SmTsectCurveType> & rCurveTypes,  // out: oneof: SM_TC_TOUCHING   = 2 surfs touch at 1 pt     
                                              //             SM_TC_CROSSING   = 2 surfs xsect along curve; surf normals are not parallel      
                                              //             SM_TC_TANGENT    = 2 surfs xsect along curve tangent to one another     
                                              //             SM_TC_COINCIDENT = 2 surfs are contiguous and share normals and cross-tangents along curve   
                                              //             SM_TC_NEAR_TANGENT = curve has small angle of xsect  
                                              //             SM_TC_REGION_BOUNDARY = curve bounds a region within which surfs are coincident
   SmTArray<double> & rDeviations)            // out: max 3DCurve through point deviation from source surfaces 
                                              //      NOTE: the deviation between through points may be larger than this value.
{
  // Place m_vCurvePoints in sPnts array
  SmTsectPnt *aData[130];
  SmTArray<SmTsectPnt*> sPnts(130,aData);
  m_vCurvePoints.GetAllNodes(sPnts);

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  if(bDebugMe)
    {
      m_vCurvePoints.Dump() ;
    }
#endif // SM_DEBUG_CODE
  
  // Clear out m_vCurvePoints point list
  while ( m_vCurvePoints.GetLastNode() )
    {
      m_vCurvePoints.RemoveLast();
    }

  // check no points to flush.
  if (sPnts.GetSize() < 1) 
    {
      return SM_SUCCESS;
    }

  // When intersection is single point, output a degenerate curve
  if ( sPnts.GetSize() == 1 )
    {
      SmTsectPnt *pStart = sPnts[0];

      // gwc:change
      //  old behavior: output degenerate curve for touching cases only
      //  new behavior: output degenerate curves for touching cases
      //                and line intersections that are limited to points
      //                due to intersection bounds.
      //                Take care not to output a duplicate
      //                degenerateCurve - a point that is already
      //                a degenerate curve or as the start/end point
      //                of an intersection curve.
      
  // gwc:removed culling check
  //          // when the point is not tangent - no curve to flush
  //          if (pStart->m_ePointType != SM_IP_TANGENT_POINT) 
  //            {
  //              return SM_SUCCESS;
  //            }
  //          else 
  //            {  // Create a degenerate curve }
  
      if(   pStart->m_ePointType == SM_IP_TANGENT_POINT
         || sm_IsPointUnique(pStart->CrvPos(), 
                             m_vStartPoints, 
                             r3DCurves, 
                             m_dThisApproxTol3d))
        {
          // Create a degenerate curve
          SmBSplineCurve *pNewBSP = NULL ;
          SER(SmBSplineCurve::CreateDegenerateCurve(*m_cpContext, 3,
                                                    pStart->CrvPos(),
                                                    pNewBSP));
          r3DCurves.Add(pNewBSP);

          // Now for the first UV degenerate curve
          SER(SmBSplineCurve::CreateDegenerateCurve(*m_cpContext, 2,
                                                    pStart->UVPos(0),
                                                    pNewBSP));
          rSurface1UVCurves.Add(pNewBSP);

          // Now for the second UV degenerate curve
          SER(SmBSplineCurve::CreateDegenerateCurve(*m_cpContext, 2,
                                                    pStart->UVPos(1),
                                                    pNewBSP));
          rSurface2UVCurves.Add(pNewBSP);
          
          // set curve dist and type parameters
          double dDist = pStart->SrfPos(0).DistanceBetween( pStart->SrfPos(1) );
          rDeviations.Add(dDist);
          rCurveTypes.Add(SM_TC_TOUCHING);
          return SM_SUCCESS;
        } // end single point is tangent or unique check
    } // end just one point to flush branch

  // Eliminate those nasty little curves which
  // start and end and have all points within tolerance of each other
  SmBoolean bTooShort = TRUE;

  // for every xsect point pair
  for (ULONG jj=1; jj<sPnts.GetSize(); jj++) 
    {
      SmTsectPnt *pTSPPrev = sPnts[jj-1];
      SmTsectPnt *pTSPCurr = sPnts[jj];
      SmPoint3d sP1(pTSPPrev->CrvPos());
      SmPoint3d sP2(pTSPCurr->CrvPos());

      // if any sequential point pair separation distance is greater than Tol
      if (!sP1.CloserThan(m_dThisApproxTol3d,sP2)) 
      //if (sP1.DistanceBetween(sP2) > m_dThisApproxTol3d) 
        {
          // keep the curve
          bTooShort = FALSE;
          break;
        }
    } // end measuring every point-pair distance 

  // when all consecutive xsect point pairs are within tol of one another
  if ( bTooShort ) 
    { // don't flush a curve - GWC:COULD output a degenerate curve
      return SM_SUCCESS;
    } // end all point-pair distances less than tol apart check


  // locals - knot, s3dCtrlPts, sUVCtrlPts0, sUVCtrlPts1 arrays 
  SmTArray<double> sKnots;
  SmPoint3d sData3D[100];
  SmTArray<SmPoint3d> s3DCtrlPts(100,sData3D);
  SmPoint3d sDataUV1[100];
  SmTArray<SmPoint3d> sUVCtrlPts0(100,sDataUV1);
  SmPoint3d sDataUV2[100];
  SmTArray<SmPoint3d> sUVCtrlPts1(100,sDataUV2);
  SmTArray<SmPoint3d> *apUVCtrlPts[2];
  apUVCtrlPts[0] = &sUVCtrlPts0;
  apUVCtrlPts[1] = &sUVCtrlPts1;

  // init curve statistics 
  double dMaxDeviation = sPnts[0]->m_dDeviation;
  double dMaxAngle     = sPnts[0]->m_dTangentPlaneAngleRad;
#ifdef SM_DEBUG_CODE
  double dMaxStep2dU[2] ; dMaxStep2dU[0] = dMaxStep2dU[1] = 0.0 ;
  double dMaxStep2dV[2] ; dMaxStep2dV[0] = dMaxStep2dV[1] = 0.0 ;
#endif 
  SmTsectCurveType eCurveType =   (sPnts[0]->m_ePointType == SM_IP_TANGENT_CURVE)
                                ? SM_TC_TANGENT
                                : SM_TC_CROSSING;

  // start knot, 3d, and 2d control point arrays
  sKnots.Add(sPnts[0]->m_dCurveParameter);
  s3DCtrlPts.Add(sPnts[0]->CrvPos());
  apUVCtrlPts[0]->Add(SmPoint3d(sPnts[0]->UVPos(0)));
  apUVCtrlPts[1]->Add(SmPoint3d(sPnts[0]->UVPos(1)));

  // for every SmTsectPnt to flush
  for (ULONG i=1; i<sPnts.GetSize(); i++) 
    {
      SmTsectPnt *pTSPPrev = sPnts[i-1];
      SmTsectPnt *pTSPCurr = sPnts[i];

      // Do a little check here to test to see if the end segment is very small
      // if it is then just eliminate it.
      if (i == sPnts.GetSize() - 2) 
        {
          SmTsectPnt *pTSPFinal = sPnts[i+1];
          double dTDelta        = pTSPCurr->m_dCurveParameter - pTSPPrev->m_dCurveParameter;
          double dTDeltaFinal   = pTSPFinal->m_dCurveParameter - pTSPCurr->m_dCurveParameter;
          // merge short end-segements into their neighbor as
          // this    +-------------+-+             
          // goes to +---------------+
          if (dTDeltaFinal/dTDelta < 0.1) 
            {
              // skip next to last xsect point
              i++;
              pTSPCurr = sPnts[i];
            }
        } // end penultimate segment check

      // Collect maximum tangent plane angle
      if ( pTSPCurr->m_dTangentPlaneAngleRad > dMaxAngle )
        { dMaxAngle = pTSPCurr->m_dTangentPlaneAngleRad; }

      // Collect maximum deviation
      if ( pTSPCurr->m_dDeviation > dMaxDeviation )
         { dMaxDeviation = pTSPCurr->m_dDeviation; }

      // First create the 3D curve if asked for
      // 1. add the curve's next knot
      sKnots.Add( pTSPCurr->m_dCurveParameter );

      // 2. get the segments hermite representation (2 end-pts + 2 end-tangents)
      SmPoint3d sP1( pTSPPrev->CrvPos() );
      SmPoint3d sP4( pTSPCurr->CrvPos() );

      // scale end tangents by the element's parameter length
      double dScale = pTSPCurr->m_dCurveParameter - pTSPPrev->m_dCurveParameter;
      SmVector3d sD1( pTSPPrev->CrvDeriv() * dScale );
      SmVector3d sD2( pTSPCurr->CrvDeriv() * dScale );

      // 3. Convert from cubic Hermite to cubic Bezier:
      // scale the vectors in the Hermite by 1/3 and
      // add or subtract from the first/last vertices.
      SmPoint3d sP2 = sP1 + sD1 / 3.0;
      SmPoint3d sP3 = sP4 - sD2 / 3.0;
      
      // 4. add the points to the 3d curve (don't duplicate segment end-points)
      s3DCtrlPts.Add( sP2 );
      s3DCtrlPts.Add( sP3 );
      s3DCtrlPts.Add( sP4 );

      // for both surfaces - build the uv curves
      for (ULONG lSrf=0; lSrf<2; lSrf++) 
        {
          // get element end positions and end-tangents scaled to element param length
          SmPoint2d  sPt1( pTSPPrev->UVPos(lSrf) );
          SmPoint2d  sPt4( pTSPCurr->UVPos(lSrf) );
          SmVector2d sDrv1( pTSPPrev->UVDeriv(lSrf) * dScale );
          SmVector2d sDrv2( pTSPCurr->UVDeriv(lSrf) * dScale );

          // Convert from cubic Hermite to cubic Bezier:
          SmPoint2d sPt2 = sPt1 + sDrv1 / 3.0;
          SmPoint2d sPt3 = sPt4 - sDrv2 / 3.0;

          // add control points - don't duplicate end points
          apUVCtrlPts[lSrf]->Add(SmPoint3d( sPt2 ));
          apUVCtrlPts[lSrf]->Add(SmPoint3d( sPt3 ));
          apUVCtrlPts[lSrf]->Add(SmPoint3d( sPt4 ));

#ifdef SM_DEBUG_CODE
          // gather step size statistics
          if(dMaxStep2dU[lSrf] < smos_Fabs( sPt1.x - sPt2.x))
            dMaxStep2dU[lSrf] = smos_Fabs( sPt1.x - sPt2.x) ;
          if(dMaxStep2dU[lSrf] < smos_Fabs( sPt1.x - sPt3.x))
            dMaxStep2dU[lSrf] = smos_Fabs( sPt1.x - sPt3.x) ;
          if(dMaxStep2dU[lSrf] < smos_Fabs( sPt1.x - sPt4.x))
            dMaxStep2dU[lSrf] = smos_Fabs( sPt1.x - sPt4.x) ;
          
          if(dMaxStep2dV[lSrf] < smos_Fabs( sPt1.y - sPt2.y))
            dMaxStep2dV[lSrf] = smos_Fabs( sPt1.y - sPt2.y) ;
          if(dMaxStep2dV[lSrf] < smos_Fabs( sPt1.y - sPt3.y))
            dMaxStep2dV[lSrf] = smos_Fabs( sPt1.y - sPt3.y) ;
          if(dMaxStep2dV[lSrf] < smos_Fabs( sPt1.y - sPt4.y))
            dMaxStep2dV[lSrf] = smos_Fabs( sPt1.y - sPt4.y) ;
          
#endif 
        } // end iter both surfaces - lSrf
    } // end iter every xsect point - building piece-wise C0 'bezier' curves

  // build SmBSplineCurves from knot and control point arrays
  SER(sm_FlushCrv(*m_cpContext,3,s3DCtrlPts, sKnots,r3DCurves));
  SER(sm_FlushCrv(*m_cpContext,2,sUVCtrlPts0,sKnots,rSurface1UVCurves));
  SER(sm_FlushCrv(*m_cpContext,2,sUVCtrlPts1,sKnots,rSurface2UVCurves));

  // store the maxDeviation
  rDeviations.Add(dMaxDeviation);

  // check max angle value to set the SM_TC_NEAR_TANGENT value for SM_TC_CROSSING curves
  if (   dMaxAngle < SM_DEG2RAD( 5.0 )
      && eCurveType == SM_TC_CROSSING 
      && s3DCtrlPts.GetSize() > 4) 
   {
      eCurveType = SM_TC_NEAR_TANGENT;
   }

  // store the curve type
  rCurveTypes.Add(eCurveType);

  return SM_SUCCESS;

} // end SmSurfaceIntersector::FlushCurve

/*******************************************************************//**
PURPOSE: Find interior intersection curves.

NOTES: 
  output intersections are added to the output arrays
***********************************************************************/
SmStatus SmSurfaceIntersector::FindInteriorCurves
 (SmTArray<SmCurve*> & r3DCurves,            // out: 3d intersection curve array
  SmTArray<SmCurve*> & rSurface1UVCurves,    // out: associated UVTrimCurves on Surface 1
  SmTArray<SmCurve*> & rSurface2UVCurves,    // out: associated UVTrimCurves on Surface 2
  SmTArray<SmTsectCurveType> & rCurveTypes,  // out: associated CurveType, oneof:
                                             //       SM_TC_TOUCHING,      
                                             //       SM_TC_CROSSING,      
                                             //       SM_TC_TANGENT,       
                                             //       SM_TC_COINCIDENT,    
                                             //       SM_TC_NEAR_TANGENT,  
                                             //       SM_TC_REGION_BOUNDARY
  SmTArray<double>   & rDeviations)          // out: associated max distance between 3DCurve and surfaces
{
  // The basic intersector does not allow self intersection testing.
  // This is only available in SmAdvSurfaceIntersector.
  if (m_bDoingSelfIntersection) 
    {
      SER(SM_ERR);
    }
  // Find node pairs in the tree
#ifdef SM_32K_LOCAL_DATA_LIMIT
  SmSolution aData[50];
  SmSolutionArray sSolutions(50,aData);
#else
  SmSolution aData[200];
  SmSolutionArray sSolutions(200,aData);
#endif
  SER(m_cpSurface[0]->GlobalSurfaceSolve(m_vUVDomain[0],
                                        *m_cpSurface[1],
                                         m_vUVDomain[1],
                                        SM_SO_INTERSECT,
                                        m_dThisApproxTol3d,
                                        NULL, 
                                        NULL,
                                        SM_SR_NODES,
                                        sSolutions));

  // Now for each node pair do a test to see if produces a unique start point
  // Note that our first pass at this is naive and slow.
  for (ULONG i=0; i<sSolutions.GetSize(); i++) 
    {
      SmSolution & rSol   = sSolutions[i];
      SmTreeNode * pNode1 = rSol.m_apNodes[0];
      SmTreeNode * pNode2 = rSol.m_apNodes[1];
      SM_ASSERT(pNode1->m_eAuxDataType == SM_AD_BEZIER_SURFACE);
      SM_ASSERT(pNode2->m_eAuxDataType == SM_AD_BEZIER_SURFACE);
      SmBezierPatch *pBezPatch1 = (SmBezierPatch*)pNode1->m_pData; NER(pBezPatch1);
      SmBezierPatch *pBezPatch2 = (SmBezierPatch*)pNode2->m_pData; NER(pBezPatch2);

      SmExtent2d sUVDomain1 = pBezPatch1->GetUVDomain();
      SmExtent2d sUVDomain2 = pBezPatch2->GetUVDomain();

      // First let's do a fast test to see if the intersection of
      // these two nodes is close to a 3D curve.  If we pass this
      // test then we can go onto a little more stricter test.
      SmExtent3d sIntBBox = pNode1->m_sBBox;
      sIntBBox.ExpandAbsolute(m_dThisApproxTol3d);
      SER(sIntBBox.Intersect(pNode2->m_sBBox,sIntBBox));

      // for every current 3D intersection curve
      SmBoolean bIsNearCurve = FALSE;
      for (ULONG j=0; j<r3DCurves.GetSize(); j++) 
        {
          SmCurve      *pCurve = r3DCurves[j];
          SmCurveCache *pCC    = (SmCurveCache*)SmCacheMgr::GetOrCreateObjectCache(SM_OC_CURVE, pCurve); 
          NER(pCC);
          SmTree *pTree = pCC->GetTree();

          if (pTree->IntersectsBox(sIntBBox)) 
            {
              bIsNearCurve = TRUE;
              break;
            }
        } // end iter searching for nearby existing 3d intersection curves

      if (bIsNearCurve) continue;

      // Get an intersection curve start point and convert it to 3D
      SmSolution sSol;
      SmBoolean bFoundAnswer;
      SER(m_cpSurface[0]->LocalSurfaceIntersect(m_vUVDomain[0],
                                               *m_cpSurface[1],
                                                m_vUVDomain[1],
                                                m_dThisApproxTol3d,
                                                sUVDomain1.Evaluate(0.5,0.5), 
                                                sUVDomain2.Evaluate(0.5,0.5),
                                                NULL,
                                                NULL,
                                                TRUE,
                                                bFoundAnswer, 
                                                sSol));
      if (!bFoundAnswer) continue;
      SmPoint2d sUV(sSol.m_vStart[0],sSol.m_vStart[1]);
      SmPoint3d sSurfPnt;

      SER(m_cpSurface[0]->EvaluatePoint(sUV,sSurfPnt));

      // Now test start point to see if it drops to any of the existing 3D curves
      SmBoolean bPointIsOnCurve = IsPointOnCurve(sSurfPnt,r3DCurves);

      if (bPointIsOnCurve) continue;
          
      // If we made it this far then we have a point which should serve as
      // a good start point.  Load it up and trace it.
      SmPoint2d sUVPts[2];
      sUVPts[0] = sUV;
      sUVPts[1] = SmPoint2d(sSol.m_vStart[2],sSol.m_vStart[3]);
      SmTsectPnt *pStartTSP = (SmTsectPnt*)m_vTSPntMgr.GetNewElement();
      pStartTSP->m_ePointType = SM_IP_UNKNOWN;
      m_dCurveTraceDirection  = 1.0;

      // tell ComputePointValues this is a start point by setting pOptPreviousPnt = NULL
      if (ComputePointValues(sUVPts, *pStartTSP, NULL) != SM_SUCCESS) 
        { continue; }

      // trace the curve
      m_pStartPoint = pStartTSP;
      double dInputTraceDirection = m_dCurveTraceDirection ;
      SER(TraceCurve(*pStartTSP, r3DCurves));

      // trace curve can switch the trace direction when the first step is out of bounds

      // when Start point is not Bounded and currently traced curve is not closed
      if(   !m_bCurveIsClosed
         && !pStartTSP->IsBounded()
         &&  dInputTraceDirection == m_dCurveTraceDirection) // no switch in TraceCurve() of trace direction
        {
          // reverse the existing set of m_vCurvePoints
          m_dCurveTraceDirection *= -1.0;
          SER(ReverseCurveDirection());  // reverse m_vCurvePoints order (if any)
                                         //   negate tangent vectors, reassign parameters
                                         //   so oldParamRange[0,max] is the same as
                                         //      newParamRange[0,max] (just running in opposite dirs]

          // and do more tracing in the opposite direction
          SER(TraceCurve(*pStartTSP,r3DCurves));

          // a cusp will be made if TraceCurve reversed directions on the last call
          SM_ASSERT_MSG(dInputTraceDirection != m_dCurveTraceDirection,
                        _T("SmSurfaceIntersector::FindInteriorCurves: Assumed TraceCurve() would not reverse trace direction - probably building a cusp")) ;
          
          // reverse m_vCurvePoints again to restore original trace direction
          m_dCurveTraceDirection *= 1.0;
          SER(ReverseCurveDirection());  // reverse m_vCurvePoints order (if any)
                                         //   negate tangent vectors, reassign parameters
                                         //   so oldParamRange[0,max] is the same as
                                         //      newParamRange[0,max] (just running in opposite dirs]
        }

      // Add xsectCurve to r2DCurves defined by m_vCurvePoints.  m_vCurvePoints gets cleared.
      SER(FlushCurve(r3DCurves,rSurface1UVCurves,rSurface2UVCurves,rCurveTypes,rDeviations));
      m_pStartPoint = NULL;
    
    } // end iter every node/node global surface solution

  // all done
  return SM_SUCCESS;

} // SmSurfaceIntersector::FindInteriorCurves

/*******************************************************************//**
PURPOSE: This method is the top level interface function for performing
    surface/surface intersection.

NOTES: 
    This method currently only works on surfaces which are 
    at least G1 (smooth) within the corresponding surface domain.  It is 
    possible to intersect surfaces with discontinuities by making multiple
    calls to this method using different surface domains.

***********************************************************************/
SmStatus SmSurfaceIntersector::DoIntersection
  (const SmContext            & crContext,               // in : context for new object construction
   const SmApproxTol3d        * pOptApproxTol3d,         // in : specify dist tol, NULL = use m_dThisApproxTol3d value  
   const double               * pdOptAngTolRad,          // in : specify ang tol,  NULL = use m_dThisAngTolRad    value
   SmTArray<SmCurve*>         * p3DCurves,               // out: 3dCurve intersections
   SmTArray<SmCurve*>         * pSurface1UVCurves,       // out: associated UVTrimCurves on m_cpSurface[0]
   SmTArray<SmCurve*>         * pSurface2UVCurves,       // out: associated UVTrimCurves on m_cpSurface[1]
   SmTArray<SmTsectCurveType> * pCurveTypes,             // out: oneof for each 3dCurve
                                                         //      SM_TC_TOUCHING,       // Curve is a degenerate point which represents a 
                                                         //                            // single point where surfaces touch.
                                                         //      SM_TC_CROSSING,       // Curve represents a crossing intersection where 
                                                         //                            // surface normals are not parallel.
                                                         //      SM_TC_TANGENT,        // Curve represents a tangent curve where surfaces touch
                                                         //                            // along a curve
                                                         //      SM_TC_COINCIDENT,     // Curve represents a point where the two surfaces
                                                         //                            // are coincident.  Usually this curve corresponds to 
                                                         //                            // a boundary curve from one of the surfaces.
                                                         //                            // Along this curve, both surfaces are contiguous,
                                                         //                            // have parallel surface normals, and have a
                                                         //                            // cross-tangent direction in which the surface
                                                         //                            // curvatures are the same.
                                                         //      SM_TC_NEAR_TANGENT,   // Curve has a relatively small angle of intersection
                                                         //      SM_TC_REGION_BOUNDARY // Curve bounds a region, within which the surfaces
                                                         //                            // are coincident
   SmTArray<double>           * pDeviations)             // out: associated max 3DCurve to Surface distance
{
  // init output
  if (p3DCurves)         { p3DCurves->ReSet() ;         }
  if (pCurveTypes)       { pCurveTypes->ReSet() ;       }
  if (pSurface1UVCurves) { pSurface1UVCurves->ReSet() ; }
  if (pSurface2UVCurves) { pSurface2UVCurves->ReSet() ; }
  if (pDeviations)       { pDeviations->ReSet();        }

  // 
  // 
  // 
  if (pOptApproxTol3d) m_dThisApproxTol3d = *pOptApproxTol3d;
  if (pdOptAngTolRad) m_dThisAngTolRad   = *pdOptAngTolRad;

  // get surface caches and their decomposition trees
  SmSurfaceCache *pSC1   = smsurf_GetSurfaceCache(m_cpSurface[0]); NER(pSC1);
  SmSurfaceCache *pSC2   = smsurf_GetSurfaceCache(m_cpSurface[1]); NER(pSC2);
  SmTree         *pTree1 = pSC1->GetTree(); NER(pTree1);
  SmTree         *pTree2 = pSC2->GetTree(); NER(pTree2);
  
  m_apTrees[0] = pTree1;
  m_apTrees[1] = pTree2;

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  // draw Breps(blue and green), surfaces(cyan and yellow), and surface caches(blue and green) 
  if(bDebugMe)
    {
      SM_ASSERT_VALID(pSC1) ;
      SM_ASSERT_VALID(pSC2) ;
      SM_ASSERT_VALID(m_cpSurface[0]) ;
      SM_ASSERT_VALID(m_cpSurface[1]) ;

      SmFace *pFace1 = (SmFace *)m_cpSurface[0]->GetFace() ;
      SmFace *pFace2 = (SmFace *)m_cpSurface[1]->GetFace() ;
      SmBrep *pBrep1 = (pFace1) ? pFace1->GetBrep() : NULL ;
      SmBrep *pBrep2 = (pFace2) ? pFace2->GetBrep() : NULL ;

      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep1) { pBrep1->Draw(TRUE) ; } sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,0) ; if(pBrep2) { pBrep2->Draw(TRUE) ; } sm_GraphicsLoop() ;

      smgfx_SetLook(1,2, 0,1,1) ; m_cpSurface[0]->DrawUV(6,6,FALSE,NULL,pBrep1==NULL) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 1,1,0) ; m_cpSurface[1]->DrawUV(6,6,FALSE,NULL,pBrep2==NULL) ; sm_GraphicsLoop() ;

      smgfx_SetLook(1,2, 0,0,1) ; pSC1->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,0) ; pSC2->Draw(TRUE) ; sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

//  // no work surfaces contain C0 continuities
//  if (pSC1->GetSurfaceContinuity(m_vUVDomain[0]) <= SM_CT_C0) 
//    {
//        SER(SM_ERR_INVALID_INPUT);
//    }
//  if (pSC2->GetSurfaceContinuity(m_vUVDomain[1]) <= SM_CT_C0) 
//    {
//        SER(SM_ERR_INVALID_INPUT);
//    }  
      
  // no work - root treeNode bounding boxes are disjoint - no possible intersections
  SmTreeNode *pRoot1 = pTree1->GetTopNode();
  SmTreeNode *pRoot2 = pTree2->GetTopNode();
  SmExtent3d  sBBox  = pRoot1->m_sBBox;
  sBBox.ExpandAbsolute(m_dThisApproxTol3d);
  if (sBBox.AreDisjoint(pRoot2->m_sBBox)) 
    { return SM_SUCCESS; }

  // get bounding box intersection - the region containing a possible intersection
  sBBox.Intersect(pRoot2->m_sBBox,sBBox);
  sBBox.ExpandAbsolute(m_dThisApproxTol3d);

  // Find node pairs in the tree where intersections might exist
      //   output: 1. Store number of intersecting leafNode pairs based on bounding pseudoBox intersection test.
      //           2. Store union of all intersecting leafNode bounding box intersections.
      //           3. Store union of all intersecting leafNode intervals.  
      //   sSolutions[0].m_vStart[6] = number of leafNode pairs with intersecting bounding boxes
      //   sSolutions[0].m_vStart[0] = BoundingBoxUnion.MinX
      //   sSolutions[0].m_vStart[1] = BoundingBoxUnion.MinY
      //   sSolutions[0].m_vStart[2] = BoundingBoxUnion.MinZ
      //   sSolutions[0].m_vStart[3] = BoundingBoxUnion.MaxX
      //   sSolutions[0].m_vStart[4] = BoundingBoxUnion.MaxY
      //   sSolutions[0].m_vStart[5] = BoundingBoxUnion.MaxZ
      //   sSolutions[0].m_vEnd[0] = surf0.UVDomainUnion.MinX
      //   sSolutions[0].m_vEnd[1] = surf0.UVDomainUnion.MinY
      //   sSolutions[0].m_vEnd[2] = surf0.UVDomainUnion.MaxX
      //   sSolutions[0].m_vEnd[3] = surf0.UVDomainUnion.MaxY
      //   sSolutions[0].m_vEnd[4] = surf1.UVDomainUnion.MinX
      //   sSolutions[0].m_vEnd[5] = surf1.UVDomainUnion.MinY
      //   sSolutions[0].m_vEnd[6] = surf1.UVDomainUnion.MaxX
      //   sSolutions[0].m_vEnd[7] = surf1.UVDomainUnion.MaxY
  SmSolution      aData[10];
  SmSolutionArray sSolutions(10,aData);

  // Find node pairs in the tree where intersections might exist
  SER(SolveIt(SM_SO_INTERSECTION_TEST, // in : specify the solver problem                                                            
              SM_SR_ALL,               // in : one of SM_SR_SINGLE - get best solution                                               
                                       //             SM_SR_ALL    - get all  solutions                                              
                                       //             SM_SR_NODES  - find nodes in tree where solution may exist                     
               m_dThisApproxTol3d,     // in : Size distance to satisfy                                                              
               SM_BIG_DOUBLE,          // in : Best scalar value found to date                                                       
               NULL,                   // in : a vector not required for SM_SO_INTERSECTION_TEST solver:   
               sSolutions));           // out: array of SmSolution objects

  // no work - no intersecting leafNode bounding boxes
  if (sSolutions.GetSize() == 0) 
    { return SM_SUCCESS; }
 
  // get leafNode BBox intersection results
  SmSolution & rSol = sSolutions[0];

  // refine the intersection sBBox size based on leafNode BBox intersections
  sBBox.SetMinMax(SmPoint3d(rSol.m_vStart[0],rSol.m_vStart[1],rSol.m_vStart[2]),
                  SmPoint3d(rSol.m_vStart[3],rSol.m_vStart[4],rSol.m_vStart[5]) );
  
#ifdef SM_DEBUG_CODE
  // draw common bounding boxes(red)
  if(bDebugMe)
    {
      SmFace *pFace1 = (SmFace *)m_cpSurface[0]->GetFace() ;
      SmFace *pFace2 = (SmFace *)m_cpSurface[1]->GetFace() ;
      SmBrep *pBrep1 = (pFace1) ? pFace1->GetBrep() : NULL ;
      SmBrep *pBrep2 = (pFace2) ? pFace2->GetBrep() : NULL ;

      TCHAR sBuff[SM_TBLOCK_SIZE];
      ULONG lXCount = (ULONG)rSol.m_vStart[6] ;
      smos_sprintf(sBuff,_T("\nNumber of intersecting LeafNode Pairs = %ld "), lXCount);
      smos_WriteBuffer(sBuff);
      SmExtent2d sSurf1Domain(SmPoint2d(rSol.m_vEnd[0], rSol.m_vEnd[1]),      
                              SmPoint2d(rSol.m_vEnd[2], rSol.m_vEnd[3])) ;    
      SmExtent2d sSurf2Domain(SmPoint2d(rSol.m_vEnd[4], rSol.m_vEnd[5]),      
                              SmPoint2d(rSol.m_vEnd[6], rSol.m_vEnd[7])) ;    
      
      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep1) { pBrep1->Draw(TRUE) ; } sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,0) ; if(pBrep2) { pBrep2->Draw(TRUE) ; } sm_GraphicsLoop() ;
      smgfx_SetLook(2,3, 1,.5,0); sBBox.Draw(&crContext) ; sm_GraphicsLoop() ;
      smgfx_SetLook(2,3, 1,0,0) ; m_cpSurface[0]->DrawUV(6,6,FALSE,&sSurf1Domain,pBrep1==NULL) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,0,0) ; if(pFace1) { pFace1->Draw(SM_DM_CROSSHATCH) ; }sm_GraphicsLoop() ;
      smgfx_SetLook(2,3, 1,0,1) ; m_cpSurface[1]->DrawUV(6,6,FALSE,&sSurf2Domain,pBrep2==NULL) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,0,0) ; if(pFace2) { pFace2->Draw(SM_DM_CROSSHATCH) ; }sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

  // look for single leafNode bounding box intersection - may be a point
  //   When surfaces only graze each other then do some
  //   processing to help figure out what the intersection is and possibly
  //   eliminate the need for full intersection.
  if (rSol.m_vStart[6] < 2.0) 
    {
      SmVector3d sSize  = sBBox.GetSize();

      // count the number of small dimensions: 3 = a point intersection
      ULONG lSmallCount = 0;
      if (sSize.x < 2.0*m_dThisApproxTol3d) lSmallCount ++;
      if (sSize.y < 2.0*m_dThisApproxTol3d) lSmallCount ++;
      if (sSize.z < 2.0*m_dThisApproxTol3d) lSmallCount ++;

      // when any dimension is smaller than 2*tolerance -
      if (lSmallCount > 0) 
        {
          // make Bboxes for intersection UVDomain
          SmExtent2d sUVDomain1(SmPoint2d(rSol.m_vEnd[0],rSol.m_vEnd[1]),
                                SmPoint2d(rSol.m_vEnd[2],rSol.m_vEnd[3]) );
          SmExtent2d sUVDomain2(SmPoint2d(rSol.m_vEnd[4],rSol.m_vEnd[5]),
                                SmPoint2d(rSol.m_vEnd[6],rSol.m_vEnd[7]) );

          // resolve the intersection
          SmBoolean bMoreWorkRequired;
          SER(GrazingBoxProcess(crContext,sBBox,sUVDomain1,sUVDomain2,
                                bMoreWorkRequired,
                                p3DCurves,pSurface1UVCurves,
                                pSurface2UVCurves,pCurveTypes,pDeviations));

          if (!bMoreWorkRequired) return SM_SUCCESS;

          // GWC: could be a bug if GrazingBoxProcess returns both
          //      a solution and a flag to find more intersections
          // SM_ASSERT(pD3Curves != NULL && p3DCurves->GetSize() == 0) ;
        } // found a small dimension intersection check
    } // end found a single leafNode pair intersection check
     

  // arrive here when we need to do full intersections

  // check out caches to delay cache deletes until they are checked back in
  SmCacheCheckOutIn sCheckIO(pSC1);
  SmCacheCheckOutIn sCheckIO2(pSC2);

  // temporarily turn off point testing and boundary curve testing 

  { // define a scope to force temporary change destructors to run 
    // before surface cache is checked back in and possibly deleted.

  // Turn off point testing so GlobalPointSolve() will keep all point solutions
  //   without classifying the solution point against the trim boundaries.
  // Turn on boundary curve processing to force LocalSolve to look for drop points
  //   on boundary curves.
  SmTemporaryChangeValue<SmBoolean> sStack1(pSC1->m_bPointTestEnabled,FALSE);
  SmTemporaryChangeValue<SmBoolean> sStack2(pSC2->m_bPointTestEnabled,FALSE);
  SmTemporaryChangeValue<SmBoolean> sStack3(pSC1->m_bProcessBoundaryCurves,TRUE);
  SmTemporaryChangeValue<SmBoolean> sStack4(pSC2->m_bProcessBoundaryCurves,TRUE);

  m_cpContext = &crContext;

  // locals
  SmTArray<SmCurve*>     s3DCurves;
  SmObjsDelete<SmCurve*> sCleanup3D(&s3DCurves);
  SmTArray<SmCurve*>     sSurface1UVCurves;
  SmObjsDelete<SmCurve*> sCleanupUV1(&sSurface1UVCurves);
  SmTArray<SmCurve*>     sSurface2UVCurves;
  SmObjsDelete<SmCurve*> sCleanupUV2(&sSurface2UVCurves);
  SmTArray<double>           sDeviations;
  SmTArray<SmTsectCurveType> sCurveTypes;
  
  // look for special case - 
  // two surfaces intersect only along pair of planar boundary curves
  //   possible results:
  //     SM_BR_GRAZE,     // one pair of boundary curve bounding planes is   
  //                      // coincident, and the surfaces are on the opposite
  //                      // sides of this plane - intersection can be found
  //                      // by intersecting boundary curves                             
  //     SM_BR_DISJOINT,  // The surfaces are definitely disjoint            
  //     SM_BR_UNKNOWN    // Intersection possible: more work needed 

#ifdef SM_DEBUG_CODE
  // draw both surface caches with their boundary planes
  if(bDebugMe)
    {
      smgfx_Erase() ;

      // draw surface caches with bounding planes
      smgfx_SetLook(1,2, 0,0,1) ; pSC1->Draw(FALSE, TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(3,4, 0,1,0) ; pSC2->Draw(FALSE, TRUE) ; sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;

      // draw Breps
      smgfx_SetLook(4,5, 0,1,1) ; if(pSC1->GetBrep()) pSC1->GetBrep()->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(5,6, 1,0,1) ; if(pSC2->GetBrep()) pSC2->GetBrep()->Draw(TRUE) ; sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;

    }
#endif // SM_DEBUG_CODE
  
  // check boundary curve bounding planes for possible disjointness or grazing            
  SmBoundaryPlaneTestResult eBoundaryTestResult;
  SER(DoBoundaryPlaneTest
       (pSC1->GetPatchBoundaryBoundingPlanes(),  // in : BoundaryPlanes from SurfaceCache1                                      
        pSC2->GetPatchBoundaryBoundingPlanes(),  // in : BoundaryPlanes from SurfaceCache2                                      
        m_dThisApproxTol3d,                      // in : max distance between distinct intersection points                      
        sBBox,                                   // in : intersection bounding box limit                                        
        s3DCurves,                               // out: augmented with bndryCrv/bndryCrv xSects for grazind cases              
        sCurveTypes,                             // out:                                                                        
        sDeviations,                             // out:                                                                        
        eBoundaryTestResult));                   // out: oneof SM_BR_GRAZE one pair of boundary curve bounding planes is        
                                                 //                        coincident, and the surfaces are on the opposite     
                                                 //                        sides of this plane                                  
                                                 //            SM_BR_DISJOINT The surfaces are definitely disjoint: one pair of 
                                                 //                        boundary curve planes are parallel, and the surfaces 
                                                 //                        are OFF the region enclosed by the parallel planes.  
                                                 //            SM_BR_UNKNOWN  Intersection possible: more work needed           
  
  // set output with any graze intersections
  if (   eBoundaryTestResult == SM_BR_GRAZE 
      && s3DCurves.GetSize() != 0) 
    { 
      if (p3DCurves)   { p3DCurves->Append(s3DCurves); sCleanup3D.Clear(); }
      if (pCurveTypes) { pCurveTypes->Append(sCurveTypes); }
      if (pDeviations) { pDeviations->Append(sDeviations); }
                       
      for (ULONG ii=0; ii<s3DCurves.GetSize(); ii++) 
        { if (pSurface1UVCurves) { pSurface1UVCurves->Add(NULL); }
          if (pSurface2UVCurves) { pSurface2UVCurves->Add(NULL); }
        }
    }

  // Return if the boundary plane test detected disjointness or grazing
  // (a pair of boundary curve bounding planes coincident)
  if (   eBoundaryTestResult == SM_BR_GRAZE
      || eBoundaryTestResult == SM_BR_DISJOINT) 
    { return SM_SUCCESS; 
    }

  // get StartPoints =     Surf1->NaturalBoundaryCurve/Surf2 Intersections
  //                   and Surf1/Surf2->NaturalBoundaryCurve Intersections
  if (!m_bDoingSelfIntersection) 
    { // Find all Boundary isoParameterBoundaryCurve/Surface intersections.  Then
      //   add point           to m_vStartPoints for point intersections bounding intersection curves.
      //   add degenerateCurve to s3DCurves for degenerate intersections.
      //   add regular Curve   to s3DCurves for interval intersections.
      SER(FindBoundaryStartPoints(sBBox,             // in : bound on surface/surface intersections       
                                  s3DCurves,         // i/o: currently found intersections                
                                                     //      augmented whenever a boundary curve          
                                                     //      has a coincident range with the other surface
                                  sSurface1UVCurves, // out: associated Surface1 UVTrimCurves             
                                  sSurface2UVCurves, // out: associated Surface2 UVTrimCurves             
                                  sCurveTypes,       // out: associated Curve Type: oneof                 
                                                     //        SM_TC_TOUCHING                             
                                                     //        SM_TC_CROSSING                             
                                                     //        SM_TC_TANGENT                              
                                                     //        SM_TC_COINCIDENT                           
                                                     //        SM_TC_NEAR_TANGENT                         
                                                     //        SM_TC_REGION_BOUNDARY                      
                                  sDeviations));     // out: associated deviations                        
    }

#ifdef SM_DEBUG_CODE
  TCHAR sBuffer[SM_TBLOCK_SIZE] ;
  // draw Brep1(Blue), Brep2(green), Surf1-Face1(cyan-black), Surf2-Face2(yellow-black)
  //      s3DCurves(from Red to magenta) made from SurfA->Boundary/SurfB degenerate and interval intersections
  //      m_vStartPoints(magenta to black)
  if (bDebugMe) 
    {
      ULONG ii ;
      SmTArray<SmTsectPnt*> sStartPoints ;
      m_vStartPoints.Dump() ;
      m_vStartPoints.GetAllNodes(sStartPoints) ;
      ULONG lCurveCount = s3DCurves.GetSize() ;

      SmFace *pFace1 = (SmFace *)m_cpSurface[0]->GetFace() ;
      SmFace *pFace2 = (SmFace *)m_cpSurface[1]->GetFace() ;
      SmBrep *pBrep1 = (pFace1) ? pFace1->GetBrep() : NULL ;
      SmBrep *pBrep2 = (pFace2) ? pFace2->GetBrep() : NULL ;

      m_cpSurface[0]->Dump() ;
      m_cpSurface[1]->Dump() ;

      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep1) { pBrep1->Draw(TRUE) ; } sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,0) ; if(pBrep2) { pBrep2->Draw(TRUE) ; } sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,1) ; m_cpSurface[0]->DrawUV(3,3,FALSE,NULL,pBrep1==NULL) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,0,0) ; if(pFace1) { pFace1->Draw(SM_DM_CROSSHATCH) ; }sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 1,1,0) ; m_cpSurface[1]->DrawUV(3,3,FALSE,NULL,pBrep2==NULL) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,0,0) ; if(pFace2) { pFace2->Draw(SM_DM_CROSSHATCH) ; }sm_GraphicsLoop() ;
      for (ii=0; ii<lCurveCount; ii++) 
        {
          SmCurve *pCurve = s3DCurves[ii];  
          SmExtent1d sInt =  pCurve->GetNaturalInterval();
          smgfx_SetLook(3+ii, 5+ii, 1, 0, lCurveCount == 1 ? 1.0 : (double)ii/(double)(lCurveCount-1)) ;
          pCurve->Draw(&sInt); sm_GraphicsLoop();
          SM_ASSERT_VALID_NO_STREAM(((SmBSplineCurve *)pCurve)) ;
        }
      for (ii=0; ii<sStartPoints.GetSize(); ii++) 
        {
          SmTsectPnt *pTsectPnt = sStartPoints[ii] ; 
          smos_sprintf(sBuffer, _T("\nsStartPoint[%ld]"), ii); smos_WriteBuffer(sBuffer);
          pTsectPnt->Dump() ;
          smgfx_SetLook(4+lCurveCount, 5+lCurveCount, 1, sStartPoints.GetSize() == 1 ? 1.0 : (double)ii/(double)(sStartPoints.GetSize()-1), 1.0) ;
          pTsectPnt->Draw() ; sm_GraphicsLoop();
        }
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

  // while there are boundaryCurve/Surface xsect startPoints, 
  // use them to trace out intersection curves
  while (m_vStartPoints.GetLastNode() != NULL) 
    {
#ifdef SM_DEBUG_CODE
      // For experiments on m_vStartPoints order
      if (bDebugMe) 
        {
          // move member [iFrom] to slot [iTo], bump all affected members by one
          ULONG iFrom=1, iTo=0, cnt ; // set these in the debugger as desired

          SmTArray<SmTsectPnt*> sStartPoints, sStartPoints2 ;
          m_vStartPoints.GetAllNodes(sStartPoints) ;
          SM_DUMP_TARRAY( sStartPoints );
          cnt = sStartPoints.GetSize() ;

          SmTsectPnt *pFrom = sStartPoints[iFrom] ;  // set ii in debugger 
          SmTsectPnt *pPrev = (SmTsectPnt *)sStartPoints[iFrom < iTo ? iTo : ((iTo+cnt) - 1)%cnt] ;
          m_vStartPoints.Remove(pFrom) ;
          m_vStartPoints.InsertAfter(pFrom, pPrev) ;
          m_vStartPoints.GetAllNodes(sStartPoints2) ;

          sStartPoints.Dump() ;
          sStartPoints2.Dump() ;
         }
#endif // SM_DEBUG_CODE

      // nrxt: Send Last m_vStartPoints TSectPoint to TraceCurve()
      SmTsectPnt *pStartTSP  = m_vStartPoints.GetLastNode(); NER(pStartTSP);
      SmTsectPnt *pTmpTSP    = (SmTsectPnt*)m_vTSPntMgr.GetNewElement();
      m_dCurveTraceDirection = 1.0;
      ULONG lNumCurves       = s3DCurves.GetSize();

      // Remove Last StartPoint from m_vStartPoint array - keep temp copy of StartPoint 
      *pTmpTSP = *pStartTSP;
      m_vStartPoints.RemoveLast();

      // Trace an intersection curve.
      // This places a sequence of SmTsectPnts on our m_vCurvePoints list.
      // Note, it doesn't create curves: s3DCurves is an input-only
      // argument, the list of already-created curves, to avoid repeats.
//cbi: When TraceCurve() is given a pt already on a curve,
// it traces anyway, then checks the 2nd pt against all other crvs,
// finds one, and so returns just one pt, the start pt (which is already
// on another curve).  Then it calls FlushCurve(), which is (currently)
// necessary because it has the side effect of emptying the pt list.
// Then just below, the one pt gets put on the m_vThroughPoints list
// because no int crv was generated.  It's treating it like a singularity
// of some sort.
//cbi.
      double dInputTraceDirection = m_dCurveTraceDirection ;
      if (TraceCurve(*pStartTSP, s3DCurves) == SM_SUCCESS) 
        {
          // special case - StartPoint's UVTangent is tangent to the Surface domain boundary,
          //    trace the curve in both directions to get complete intersection curve.
          // Trace reverse direction when
          //    1. Curve is not yet closed
          //    2. Reverse direction has not yet been traced
          //    3. Surface0 UVTangent is tangent to all Surface0 UVBoundaries that the start point lies upon
          //    4. Surface1 UVTangent is tangent to all Surface1 UVBoundaries that the start point lies upon
          if(   !m_bCurveIsClosed                               // curve is not yet closed
             && dInputTraceDirection == m_dCurveTraceDirection  // have yet to trace the reverse direction
             && (   (pStartTSP->UParamType(0) == SM_EP_INSIDE || SM_IS_ZERO_TO_TOL(pStartTSP->UVDeriv(0).x, SM_EFF_ZERO))
                 && (pStartTSP->VParamType(0) == SM_EP_INSIDE || SM_IS_ZERO_TO_TOL(pStartTSP->UVDeriv(0).y, SM_EFF_ZERO)))
             && (   (pStartTSP->UParamType(1) == SM_EP_INSIDE || SM_IS_ZERO_TO_TOL(pStartTSP->UVDeriv(1).x, SM_EFF_ZERO))
                 && (pStartTSP->VParamType(1) == SM_EP_INSIDE || SM_IS_ZERO_TO_TOL(pStartTSP->UVDeriv(1).y, SM_EFF_ZERO))))
            {
              // trace the curve in the opposite direction
              SER(ReverseCurveDirection()) ;
              m_dCurveTraceDirection *= -1.0 ;
              SER(TraceCurve(*pStartTSP, s3DCurves)) ;

              // a cusp will be made if TraceCurve reversed directions on the last call
              SM_ASSERT_MSG(dInputTraceDirection != m_dCurveTraceDirection,
                            _T("SmSurfaceIntersector::DoIntersection: Assumed TraceCurve() would not reverse trace direction - probably building a cusp")) ;
          
              // restore the curve direction
              m_dCurveTraceDirection *= 1.0;
              SER(ReverseCurveDirection());

            } // end special case need to trace curve in both directions

#ifdef SM_DEBUG_CODE
          // draw Brep1(Blue), Brep2(green), Surf1-Face1(cyan-black), Surf2-Face2(yellow-black)
          //      s3DCurves(from Red to magenta) made from SurfA->Boundary/SurfB degenerate and interval intersections
          //      m_vStartPoints(magenta to black)
          if (bDebugMe) 
            {
              pStartTSP->Dump() ;

              ULONG ii ;
              SmTArray<SmTsectPnt*> sCurvePoints ;
              m_vCurvePoints.GetAllNodes(sCurvePoints) ;
              ULONG lCurveCount = s3DCurves.GetSize() ;

              SmFace *pFace1 = (SmFace *)m_cpSurface[0]->GetFace() ;
              SmFace *pFace2 = (SmFace *)m_cpSurface[1]->GetFace() ;
              SmBrep *pBrep1 = (pFace1) ? pFace1->GetBrep() : NULL ;
              SmBrep *pBrep2 = (pFace2) ? pFace2->GetBrep() : NULL ;

              smgfx_Erase() ;
              smgfx_SetLook(1,2, 0,0,1) ; if(pBrep1) { pBrep1->Draw(TRUE) ; } sm_GraphicsLoop() ;
              smgfx_SetLook(1,2, 0,1,0) ; if(pBrep2) { pBrep2->Draw(TRUE) ; } sm_GraphicsLoop() ;
              smgfx_SetLook(1,2, 0,1,1) ; m_cpSurface[0]->DrawUV(3,3,FALSE,NULL,pBrep1==NULL) ; sm_GraphicsLoop() ;
              smgfx_SetLook(1,2, 0,0,0) ; if(pFace1) { pFace1->Draw(SM_DM_CROSSHATCH) ; }sm_GraphicsLoop() ;
              smgfx_SetLook(1,2, 1,1,0) ; m_cpSurface[1]->DrawUV(3,3,FALSE,NULL,pBrep2==NULL) ; sm_GraphicsLoop() ;
              smgfx_SetLook(1,2, 0,0,0) ; if(pFace2) { pFace2->Draw(SM_DM_CROSSHATCH) ; }sm_GraphicsLoop() ;
              smgfx_SetLook(3,4, 1,0,0) ; pStartTSP->Draw() ; sm_GraphicsLoop() ;
              for (ii=0; ii<lCurveCount; ii++) 
                {
                  SmCurve *pCurve = s3DCurves[ii];  
                  SmExtent1d sInt =  pCurve->GetNaturalInterval();
                  smgfx_SetLook(3+ii, 5+ii, 1, 0, lCurveCount == 1 ? 1.0 : (double)ii/(double)(lCurveCount-1)) ;
                  pCurve->Draw(&sInt); sm_GraphicsLoop();
                  SM_ASSERT_VALID(((SmBSplineCurve *)pCurve)) ;
                }
              for (ii=0; ii<sCurvePoints.GetSize(); ii++) 
                {
                  SmTsectPnt *pTsectPnt = sCurvePoints[ii] ; 
                  pTsectPnt->Dump() ;
                  smgfx_SetLook(4+lCurveCount, 5+lCurveCount, 1, sCurvePoints.GetSize() == 1 ? 1.0 : (double)ii/(double)(sCurvePoints.GetSize()-1), 1.0) ;
                  pTsectPnt->Draw() ; sm_GraphicsLoop();
                }
              sm_GraphicsLoop();
            }
#endif // SM_DEBUG_CODE

          // create and add a 3DCurve and 2 UVTrimCurves with a CurveType and Deviation value
          // from the points in m_vCurvePoints.
          //   deviation = max distance between 3dCurve and surfaces. 
          SER(FlushCurve(s3DCurves, sSurface1UVCurves, sSurface2UVCurves,
                         sCurveTypes, sDeviations));
        }

      // when an intersection curve was not generated from this startPoint
      if (lNumCurves == s3DCurves.GetSize()) 
        {
          // place the xsectPoint in the m_vThroughPoints array
          m_vThroughPoints.Append(pTmpTSP);
        }

    } // end while xsect points in m_vStartPoints array

  // Now that we have found all boundary intersections - look for non boundary
  // intersections.
//    if (m_bCheckForInteriorCurves) {
  SER(FindInteriorCurves(s3DCurves,sSurface1UVCurves,sSurface2UVCurves,
                         sCurveTypes,sDeviations));
//    }

  // remove degenerate Curves whose points lie on nonNegenerate intersections
  for (long ii=0; (ULONG)ii<s3DCurves.GetSize() && s3DCurves.GetSize() > 0; ii++) 
    {
      SmCurve  *pCurve = s3DCurves[(ULONG)ii] ;
      SmPoint3d sCurvePoint ;

      // when curve is degenerate
      if(pCurve->IsDegenerate())
        {
          // and degenerate point is on another curve
          pCurve->EvaluatePoint(0.0, sCurvePoint) ;
          if(IsPointOnCurve(sCurvePoint,s3DCurves,(ULONG *)&ii))
            {
              // remove and delete the Curve and its associated UVCurves
              s3DCurves.RemoveAt((ULONG)ii) ;
              delete pCurve ; pCurve = NULL ;

              SM_ASSERT((ULONG)ii < sSurface1UVCurves.GetSize()) ;
              SM_ASSERT((ULONG)ii < sSurface2UVCurves.GetSize()) ;

              SmCurve *pCurveUV1 = sSurface1UVCurves[(ULONG)ii] ;
              SmCurve *pCurveUV2 = sSurface2UVCurves[(ULONG)ii] ;

              sSurface1UVCurves.RemoveAt((ULONG)ii) ;
              sSurface2UVCurves.RemoveAt((ULONG)ii) ;
              sDeviations.RemoveAt((ULONG)ii) ;
              sCurveTypes.RemoveAt((ULONG)ii) ;

              if(pCurveUV1) { delete pCurveUV1 ; pCurveUV1 = NULL ; }
              if(pCurveUV2) { delete pCurveUV2 ; pCurveUV2 = NULL ; }
              

              // decrement the count and continue
              ii-- ;
            } 
        } // end degenerateCurve check
    } // end iter every s3DCurves[i] looking for degenerate curves to remove

#ifdef SM_DEBUG_CODE
  // draw Brep1(Blue), Brep2(green), Surf1-Face1(cyan-black), Surf2-Face2(yellow-black)
  //      s3DCurves(from Red to magenta) 
  if (bDebugMe) 
    {
      ULONG ii, lCurveCount = s3DCurves.GetSize() ;
      SmFace *pFace1 = (SmFace *)m_cpSurface[0]->GetFace() ;
      SmFace *pFace2 = (SmFace *)m_cpSurface[1]->GetFace() ;
      SmBrep *pBrep1 = (pFace1) ? pFace1->GetBrep() : NULL ;
      SmBrep *pBrep2 = (pFace2) ? pFace2->GetBrep() : NULL ;

      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep1) { pBrep1->Draw(TRUE) ; } sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,0) ; if(pBrep2) { pBrep2->Draw(TRUE) ; } sm_GraphicsLoop() ;

      smgfx_SetLook(1,2, 0,1,1) ; m_cpSurface[0]->DrawUV(6,6,FALSE,NULL,pBrep1==NULL) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,0,0) ; if(pFace1) { pFace1->Draw(SM_DM_CROSSHATCH) ; }sm_GraphicsLoop() ;

      smgfx_SetLook(1,2, 1,1,0) ; m_cpSurface[1]->DrawUV(6,6,FALSE,NULL,pBrep2==NULL) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,0,0) ; if(pFace2) { pFace2->Draw(SM_DM_CROSSHATCH) ; }sm_GraphicsLoop() ;

      for (ii=0; ii<lCurveCount; ii++) 
        {
          SmCurve   * pCurve = s3DCurves[ii];  
          SmExtent1d  sInt   = pCurve->GetNaturalInterval();
          smgfx_SetLook(3+ii, 5+ii, 1, 0, lCurveCount == 1 ? 1.0 : (double)ii/(double)(lCurveCount-1)) ;
          pCurve->Draw(&sInt); sm_GraphicsLoop();
          SM_ASSERT_VALID(((SmBSplineCurve *)pCurve)) ;
        }
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

  // set outputs
  SM_ASSERT(   (p3DCurves         == NULL || p3DCurves->GetSize()         == 0)
            && (pSurface1UVCurves == NULL || pSurface1UVCurves->GetSize() == 0)
            && (pSurface2UVCurves == NULL || pSurface2UVCurves->GetSize() == 0)
            && (pDeviations       == NULL || pDeviations->GetSize()       == 0)
            && (pCurveTypes       == NULL || pCurveTypes->GetSize()       == 0)) ;

  if (p3DCurves)         { p3DCurves->Append(s3DCurves);                 sCleanup3D.Clear(); }
  if (pSurface1UVCurves) { pSurface1UVCurves->Append(sSurface1UVCurves); sCleanupUV1.Clear(); }
  if (pSurface2UVCurves) { pSurface2UVCurves->Append(sSurface2UVCurves); sCleanupUV2.Clear(); }
  if (pDeviations)       { pDeviations->Append(sDeviations); }
  if (pCurveTypes)       { pCurveTypes->Append(sCurveTypes); }

  } // end scope to force temporary change destructors to run 
    // before surface cache is checked back in and possibly deleted.

  // all done
  return SM_SUCCESS;

} // end SmSurfaceIntersector::DoIntersection
                       
/*******************************************************************//**
PURPOSE: This method is the top level interface function for performing
    surface/surface intersection given one or more points.  It will produce
    only a single curve set.  The points will correspond to knots in the
    output curve set.  An optional start direction can be specified make the
    curve start and/or stop on the given start/end point.  It is a little
    complex how to use this function so read the following documentation
    carefully.

NOTES: This method currently only works on surfaces which are 
    at least G1 (smooth) within the corresponding surface domain.  

    The first point is always the start point.  If no start direction is 
    specified the intersector will try to go in both directions from the
    given start point, unless the start point is on a natural boundary of one
    of the surfaces.  In that case, the intersector will only try to go
    in the direction that takes the curve into the interiors of both surfaces.
    
    If two points are specified, both points will lie on the output curve.
    They will bound the output curve if pOptStartDirection is specified.  Note
    that if you give a start direction, you must be able to trace from the
    start point to the end point in this direction.
    
    If more than two points are specified, all points must lie on the curve.
    If the start direction is specified in this case the points must lie 
    in an ordered fashion along the intersection.  
    
    Note that all points will correspond to knots in the output curves.

    If multiple points are given then the extension flags indicate if the
    points are terminators for the intersection curve in the corresponding
    directions.

***********************************************************************/
SmStatus SmSurfaceIntersector::DoPointIntersection
  (const SmContext     & crContext,               // in : context for new object construction
   const SmTArray<SmTsectPnt*> & crTsectPoints,   // in : Points on the intersection 
   SmBoolean             bExtendBeforeStart,      // in : If more than two points
                                                  //      are specified this will allow extension of the 
                                                  //      intersection curve prior to the start point
   SmBoolean             bExtendAfterEnd,         // in : If more than two points
                                                  //      are specified this will allow the extension of the
                                                  //      intersection curve after the last point.
   const SmVector3d    * pOptStartDirection,      // in : Specifies direction to start the intersection marching
   const SmVector3d    * pOptEndDirection,        // in : Specifies direction to end the intersection marching
   const SmApproxTol3d * pOptApproxTol3d,         // in : specify dist tol, NULL = use m_dThisApproxTol3d value
   const double        * pdOptAngTolRad,          // in : specify ang tol,  NULL = use m_dThisAngTolRad    value
   SmBSplineCurve     *& rp3DCurve,               // out: Resulting 3Dcurve - note that this will be NULL
                                                  //      if we are unable to find a curve that satisfies
                                                  //      the input requirements and passes through all given points.
   SmBSplineCurve     *& rpSurface1UVCurve,       // out: Resulting UVTrimCurve on Surface1
   SmBSplineCurve     *& rpSurface2UVCurve,       // out: Resulting UVTrimCurve on Surface2
   SmTsectCurveType    & reCurveType,             // out: specify the kind of intersection curve found
   double              & rdDeviation)             // out: max distance from through points to source surfaces
{
  // init output 
  rp3DCurve              = NULL;
  rpSurface1UVCurve      = NULL;
  rpSurface2UVCurve      = NULL;
  m_dCurveTraceDirection = 1.0;

  // no work - Invalid input - no through points specified
  if (crTsectPoints.GetSize() == 0) 
    { SER(SM_ERR); }

  // Use supplied tolerances when given
  if ( pOptApproxTol3d ) { m_dThisApproxTol3d = *pOptApproxTol3d ; }
  if ( pdOptAngTolRad ) { m_dThisAngTolRad    = *pdOptAngTolRad ; }

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
static ULONG lCount      = 1 ; lCount++ ;
static ULONG lDebugCount = 0 ;
  // draw every given TsectPoint, start direction from 1st point, and surfaces 
  if (bDebugMe || lCount == lDebugCount) 
    {
      const SmSurface * pSurface1 = GetSurface(0) ;
      const SmSurface * pSurface2 = GetSurface(1) ;
      SmFace * pFace1 = (SmFace *)pSurface1->GetFace() ;
      SmFace * pFace2 = (SmFace *)pSurface2->GetFace() ;
      SmBrep * pBrep1 = pFace1 ? pFace1->GetBrep() : NULL ;
      SmBrep * pBrep2 = pFace2 ? pFace2->GetBrep() : NULL ;
      SmPoint3d sPnt1, sPnt2 ;

      smgfx_Erase() ;
      smgfx_SetLook(1,2,  0,0,1); if(pBrep1) pBrep1->Draw(TRUE) ; sm_GraphicsLoop();
      smgfx_SetLook(1,2,  0,1,0); if(pBrep2 && pBrep1 != pBrep2) pBrep2->Draw(TRUE) ; sm_GraphicsLoop();
      for (ULONG ii=0; ii<crTsectPoints.GetSize(); ii++) 
        {
          m_cpSurface[0]->EvaluatePoint(crTsectPoints[ii]->UVPos(0),sPnt1);
          m_cpSurface[1]->EvaluatePoint(crTsectPoints[ii]->UVPos(1),sPnt2);

          smgfx_SetLook(4,10, 1,0,0); sPnt1.Draw(); sm_GraphicsLoop();
          smgfx_SetLook(6,12, 0,1,0); sPnt2.Draw(); sm_GraphicsLoop();
          smgfx_SetLook(4,10, 1,0,1); if(pOptStartDirection && ii==0) pOptStartDirection->Draw(&sPnt1); sm_GraphicsLoop();
          smgfx_SetLook(4,10, 1,0,1); if(pOptEndDirection && ii==crTsectPoints.GetSize()-1) pOptEndDirection->Draw(&sPnt1); sm_GraphicsLoop();
          sm_GraphicsLoop();
        }
      
      smgfx_SetLook(1,4, 0,0,0); m_cpSurface[0]->DrawUV(5,5); sm_GraphicsLoop();
      smgfx_SetLook(1,4, 0,1,1); m_cpSurface[1]->DrawUV(5,5); sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

  // store the context
  m_cpContext = &crContext;

  // output locals - temporary s3DCurves, sSurface1UVCurves, sSurface2UVCurves object arrays
  //                 and sDeviations, sCurveTypes arrays
  ULONG ii ;
  SmTArray<SmCurve*>         s3DCurves;
  SmObjsDelete<SmCurve*>     sCleanup3D(&s3DCurves);
  SmTArray<SmCurve*>         sSurface1UVCurves;
  SmObjsDelete<SmCurve*>     sCleanupUV1(&sSurface1UVCurves);
  SmTArray<SmCurve*>         sSurface2UVCurves;
  SmObjsDelete<SmCurve*>     sCleanupUV2(&sSurface2UVCurves);
  SmTArray<double>           sDeviations;
  SmTArray<SmTsectCurveType> sCurveTypes;

  // init the ThroughPoint and StartPoint arrays  (set m_pListHead = NULL; )
  m_vThroughPoints.Init();
  m_vStartPoints.Init();

  // if given an optional start direction - store it in a prevTSP 3DCurve tangent value
  SmTsectPnt sTempTSP;
  SmTsectPnt *pPrevTSP = NULL;
  if (pOptStartDirection) 
    {
      sTempTSP.m_ePointType = SM_IP_UNKNOWN ;
      sTempTSP.CrvDeriv()   = *pOptStartDirection;
      pPrevTSP              = &sTempTSP; 
    }

  // if given an optional end direction - store it in a prevEndTSP 3DCurve tangent value
  SmTsectPnt sTempEndTSP;
  SmTsectPnt *pPrevEndTSP = NULL;
  if (pOptEndDirection) 
    {
      sTempEndTSP.CrvDeriv() = *pOptEndDirection;
      pPrevEndTSP            = &sTempEndTSP; 
    }

  // Initialize start point and set up next points
  // for every given target through point
  for(ii=0; ii<crTsectPoints.GetSize(); ii++) 
    {
      // get the next through point
      SmTsectPnt *pTSP = crTsectPoints[ii];

      // initialize its values to unknown
      pTSP->m_ePointType      = SM_IP_UNKNOWN;
      pTSP->m_dCurveParameter = -1.0;

      // get the through point's associated UV values on each source surface
      SmPoint2d sUVs[2];
      sUVs[0] = pTSP->UVPos(0);
      sUVs[1] = pTSP->UVPos(1);

      // Now evaluate the geometry in pTSP.
      // For the first point, pass in the given start direction.
      // We do this through pPrevTSP, which is non-null if start dir was given
      if (ii > 0) pPrevTSP = NULL;

      // For the last point, pass in the given end direction.
      if (ii == crTsectPoints.GetSize()-1) pPrevTSP = pPrevEndTSP;

      // get xsect Point's 3D position,
      //                   3D tangent,
      //                   Surface1 UV tangent, and
      //                   Surface2 UV tangent from
      // the xsect Point's Surface UV position values.
      if (ComputePointValues(sUVs, *pTSP, pPrevTSP) != SM_SUCCESS) 
        {
          SER(SM_ERR);
        }        

      // If the first pair of given points are not within tolerance on
      // the two surfaces, then we know that we won't find a solution.
      // After ComputePointValues(), those surface points are available.
      if ( ii == 0 )
        {
          SmPoint3d &rPt0 = pTSP->SrfPos(0);
          SmPoint3d &rPt1 = pTSP->SrfPos(1);
          if ( !rPt0.CloserThan(m_dThisApproxTol3d, rPt1))
          //if ( rPt0.DistanceBetween( rPt1 ) > m_dThisApproxTol3d )
          {
              return SM_ERR; // returning null curves indicates failure.
          }
        }

      // Set up the SmSurfaceIntersector StartPoint, EndPoint,
      // and ThroughPoint arrays for upcoming curve trace
      // with the appropriate fillet SmTsectPt through points.

      // for the first point
      if (ii == 0) 
        { 
          SmTsectPnt *pStartPnt = (SmTsectPnt*)m_vTSPntMgr.GetNewElement();
          *pStartPnt = *pTSP;

          // Add unique TsectPnts to the m_vStartPoints list
          AddStartPoint(*pStartPnt);
          pTSP->m_dCurveParameter = 0.0;
          if (!(   pOptStartDirection 
                && crTsectPoints.GetSize() > 1) ) 
            {
              m_pStartPoint = pStartPnt;
            }
        }

      // for the last point with an Optional StartDirection and without ExtendAfterEnd
      else if (    ii == crTsectPoints.GetSize()-1 
               &&  pOptStartDirection 
               && !bExtendAfterEnd) 
        { 
          // set target endPoint
          m_pEndPoint = pTSP;
        }

      // add all intermediate points to m_vThroughPoints array
      else
        {
          AddThroughPoint(*pTSP);
        }

    }  // end iter every given target through point

  // Now trace each boundary start point
  SmTsectPnt *pTSP = m_vStartPoints.GetLastNode();
  NER(pTSP);
  m_vStartPoints.RemoveLast();
  m_dCurveTraceDirection = 1.0;

  // If we need to trace beyond the end, setup a clipping criteria to
  // avoid tracing too far from the end
  if (m_dExtensionDistance > 0.0) 
    {
      if (bExtendAfterEnd && m_pEndPoint == NULL) 
        {
          m_bDoingExtensionClipping = TRUE;
        }
    }

  // Reverse direction of tracing if indicated by Start Direction
  if (pOptStartDirection && pOptStartDirection->Dot(pTSP->CrvDeriv()) < -SM_EFF_ZERO) 
    {
       pTSP->CrvDeriv() = -pTSP->CrvDeriv();
       pTSP->UVDeriv(0) = -pTSP->UVDeriv(0);
       pTSP->UVDeriv(1) = -pTSP->UVDeriv(1);
       m_dCurveTraceDirection   = -1.0;
    }

  // Save a copy of the final curve trace TsectPnt, we might need it later.
  SmTsectPnt *pFinalTSP = NULL;

  // now trace the curve - build m_vCurvePoints sequence
  double dInputTraceDirection = m_dCurveTraceDirection ;
  if (TraceCurve(*pTSP,s3DCurves) == SM_SUCCESS) 
    {
#ifdef SM_DEBUG_CODE
      if (bDebugMe) 
        {
          m_vCurvePoints.Dump() ;
        }
#endif
      // if asked - trace the curve to extend before the first start point
      if (   bExtendBeforeStart 
          && !m_bCurveIsClosed 
          && m_vCurvePoints.GetLastNode() != NULL
          && dInputTraceDirection == m_dCurveTraceDirection) // TraceCurve did not reverse directions
        {
          SER(ReverseCurveDirection());  // reverse m_vCurvePoints order (if any)
                                         //   negate tangent vectors, reassign parameters
                                         //   so oldParamRange[0,max] is the same as
                                         //      newParamRange[0,max] (just running in opposite dirs]
          m_dCurveTraceDirection *= -1.0;
          SER(TraceCurve(*pTSP,s3DCurves));

        // a cusp will be made if TraceCurve reversed directions on the last call
        SM_ASSERT_MSG(dInputTraceDirection != m_dCurveTraceDirection,
                      _T("SmSurfaceIntersector::DoPointIntersection: Assumed TraceCurve() would not reverse trace direction - probably building a cusp")) ;
        
          SER(ReverseCurveDirection());  // reverse m_vCurvePoints order (if any)
                                         //   negate tangent vectors, reassign parameters
                                         //   so oldParamRange[0,max] is the same as
                                         //      newParamRange[0,max] (just running in opposite dirs]
          m_dCurveTraceDirection *= -1.0;
#ifdef SM_DEBUG_CODE
          if (bDebugMe) 
            {
              m_vCurvePoints.Dump() ;
            }
#endif
        }

      // Save a copy.  (FlushCurve deletes the list.)
      pFinalTSP = m_vCurvePoints.GetLastNode();

      // convert through point and tangents into SmBSplineCurves
      // when SmSurfaceIntersector::FlushCurve() runs,
      //        load 1. s3DCurves          with surf/surf intersection curve(s)
      //             2. sSurface1UVCurves  with m_cpSurface[0] UVTrimCurve(s)
      //             3. sSurface2UVCurves  with m_cpSurface[1] UVTrimCurve(s)
      // when SmFilletIntersector::FlushCurve() runs,
      //        set  1. pFilletGeom->SetFilletSurface               = New Fillet Surface
      //             2. pFilletGeom->m_pCenterLineCurve             = Offset Surface/Surface Intersection Curve
      //             3. pFilletGeom->GetRail(0)->GetPrimaryEdgeuse()->GetMate()->SetUVCurve(pUV1) = new Rail UVCurve 1
      //             4. pFilletGeom->GetRail(0)->SetCurve(pCurve1)  = new Rail 3dCurve 1
      //             5. pFilletGeom->GetRail(1)->GetPrimaryEdgeuse()->GetMate()->SetUVCurve(pUV1) = new Rail UVCurve 1
      //             6. pFilletGeom->GetRail(1)->SetCurve(pCurve1)  = new Rail 3dCurve 1
      //      where target pFilletGeom =   m_pCurrFilletGeom
      //                                 ? m_pCurrFilletGeom
      //                                 : m_rFilletSolver.GetLastFilletGeom();
      SER(FlushCurve(s3DCurves, sSurface1UVCurves, sSurface2UVCurves,
                     sCurveTypes, sDeviations));

    } // end successful TraceCurve Call check

#ifdef SM_DEBUG_CODE
  // draw Breps(blue,green), Surfaces(cyan,yellow), faces(black), StartPoint(red)
  if (bDebugMe || lDebugCount == lCount) 
    {
      SmFace *pFace1 = (SmFace *)m_cpSurface[0]->GetFace() ;
      SmFace *pFace2 = (SmFace *)m_cpSurface[1]->GetFace() ;
      SmBrep *pBrep1 = pFace1 ? pFace1->GetBrep() : NULL ;
      SmBrep *pBrep2 = pFace2 ? pFace2->GetBrep() : NULL ;

      smgfx_Erase();
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep1) { pBrep1->Draw(TRUE) ; } sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,0) ; if(pBrep2) { pBrep2->Draw(TRUE) ; } sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,1) ; m_cpSurface[0]->DrawUV(4,4,FALSE,NULL,pBrep1==NULL); sm_GraphicsLoop();
      smgfx_SetLook(1,2, 1,1,0) ; m_cpSurface[1]->DrawUV(4,4,FALSE,NULL,pBrep2==NULL); sm_GraphicsLoop();
      smgfx_SetLook(1,2, 0,0,0) ; if(pFace1) { pFace1->Draw(SM_DM_CROSSHATCH) ; } sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,0,0) ; if(pFace2) { pFace2->Draw(SM_DM_CROSSHATCH) ; } sm_GraphicsLoop() ;

      if(this->IsKindOf(SmFilletIntersector_TYPE))
        {
          SmFilletIntersector *pFilletIntersector = (SmFilletIntersector *)this ;
          SmFilletGeom        *pCurrFilletGeom    =  pFilletIntersector->GetCurrFilletGeom()
                                                   ? pFilletIntersector->GetCurrFilletGeom()
                                                   : pFilletIntersector->GetFilletSolver()->GetLastFilletGeom();
          // gwc: I think pSideSurface no longer has to be a BBSplineSurface - replace 1 line
          // rm : SmBSplineSurface * pFilletSurface = pCurrFilletGeom->GetFilletSurface() ;                                    
          SM_FILLETSURF_TYPE * pFilletSurface = pCurrFilletGeom->GetFilletSurface() ;                                    
          SmBSplineCurve   * pCenterLine    = pCurrFilletGeom->GetCenterLineCurve() ;
                                            
          SmFilletEdge     * pRailEdge0     = pCurrFilletGeom->GetRail(0) ;                                            
          SmBSplineCurve   * pRailUVCurve0  = pRailEdge0 ? pRailEdge0->GetPrimaryEdgeuse()->GetUVTrimCurve(): NULL ;
               
          SmFilletEdge     * pRailEdge1     = pCurrFilletGeom->GetRail(1) ;
          SmBSplineCurve   * pRailUVCurve1  = pRailEdge1 ? pRailEdge1->GetPrimaryEdgeuse()->GetUVTrimCurve() : NULL ;     
          
          smgfx_SetLook(2,3, 1,0,0) ; if(pCenterLine) pCenterLine->Draw(NULL, TRUE) ; sm_GraphicsLoop() ;
          smgfx_SetLook(2,3, 0,0,1) ; if(pRailEdge0) pRailEdge0->Draw() ; sm_GraphicsLoop() ;
          smgfx_SetLook(2,3, 0,1,0) ; if(pRailEdge1) pRailEdge1->Draw() ; sm_GraphicsLoop() ;
          smgfx_SetLook(1,2, 1,.5,0); if(pFilletSurface) pFilletSurface->DrawUV(8,8,FALSE,NULL,TRUE) ; sm_GraphicsLoop() ;
          smgfx_SetLook(3,4, 0,1,1) ; if(pRailUVCurve0) { SmCrvOnSurf sSurf1Curve( *pRailUVCurve0, (SmSurface &)*m_cpSurface[0]) ;
                                                          sSurf1Curve.Draw(NULL, TRUE) ; sm_GraphicsLoop() ;
                                                        }   
          smgfx_SetLook(3,4, 1,1,0) ; if(pRailUVCurve1) { SmCrvOnSurf sSurf2Curve( *pRailUVCurve1, (SmSurface &)*m_cpSurface[1]) ;      
                                                          sSurf2Curve.Draw(NULL, TRUE) ; sm_GraphicsLoop() ;
                                                        } 
          sm_GraphicsLoop();
        }
      else // SmSurfaceIntersector or SmAdvSurfaceIntersector
        {
          for(ii=0;ii<s3DCurves.GetSize();ii++)
            { 
              smgfx_SetLook(2,3, 1,0,0) ; s3DCurves[ii]->Draw(NULL, TRUE) ; sm_GraphicsLoop() ;
              if(sSurface1UVCurves.GetSize() < ii)
                { SmCrvOnSurf sSurf1Curve(*sSurface1UVCurves[ii], (SmSurface &)*m_cpSurface[0]) ;
                  smgfx_SetLook(2,3, 0,0,1) ; sSurf1Curve.Draw(NULL, TRUE) ; sm_GraphicsLoop() ;
                }
              if(sSurface2UVCurves.GetSize() < ii)
                { SmCrvOnSurf sSurf2Curve(*sSurface2UVCurves[ii], (SmSurface &)*m_cpSurface[1]) ;
                  smgfx_SetLook(2,3, 0,1,0) ; sSurf2Curve.Draw(NULL, TRUE) ; sm_GraphicsLoop() ;
                }
            } // end iter all intersection results
        } // end SmSurfaceIntersector branch
    } // end if bDebugMe check
#endif // SM_DEBUG_CODE

  // set the output values (for SmSurfaceIntersector::FlushCurve() not used by SmFilletIntersector::FlushCurve())
  if (s3DCurves.GetSize() == 1) 
    {
      sCleanup3D.Clear();
      rp3DCurve = SM_CAST_PTR(SmBSplineCurve,s3DCurves[0]);
    }
  if (sSurface1UVCurves.GetSize() == 1) 
    {
      sCleanupUV1.Clear();
      rpSurface1UVCurve = SM_CAST_PTR(SmBSplineCurve,sSurface1UVCurves[0]);
    }
  if (sSurface2UVCurves.GetSize() == 1) 
    {
      sCleanupUV2.Clear();
      rpSurface2UVCurve = SM_CAST_PTR(SmBSplineCurve,sSurface2UVCurves[0]);
    }
  if (sDeviations.GetSize() == 1) 
    {
      rdDeviation = sDeviations[0];
    }
  if (sCurveTypes.GetSize() == 1) 
    {
      reCurveType = sCurveTypes[0];
    }

  // Handle case where a singularity point was hit stopping the curve trace
  // prematurely.  Solution = Find a 2nd XSectCurve starting at the
  // the singularity Point and connect the two pieces together.
  //    (GWC: didn't work to make it through a cusp made by too large an offset on a curved surface)
  // Note: the TsectPnt's in crTsectPoints are initialized with
  // m_dCurveParameter = -1.0.  When the curve trace passes them, it updates
  // that value to the current curve parameter, which is >= 0.0.
  // So if the last TsectPnt's m_dCurveParameter is -1, then the curve trace
  // ended without hitting it.
  //
  // Note: that doesn't necessarily indicate a singularity, it will also
  // happen when a domain boundary is hit.
  // So, do not do this trick if the end of the curve trace is not a singulartiy.
  // (Actually, it's questionable whether we would want to do it
  // even in the case of a singularity.  It's never hit in prog_test.)

  SmBoolean bIsSingular = FALSE;
  SmTsectPnt *pLast             = crTsectPoints.GetLast();
  ULONG       lTotalTsectPoints = crTsectPoints.GetSize();
static       SmBoolean bInsideMe = FALSE;
  if (rp3DCurve && pLast->m_dCurveParameter < 0.0 && !bInsideMe) 
    {
      if ( pFinalTSP != NULL )
       {
          SmPoint2d sUV = pFinalTSP->UVPos(0);
          SmSurfParamType eWhichDir;
          bIsSingular  = m_cpSurface[0]->IsSingularity( sUV, eWhichDir );
          sUV = pFinalTSP->UVPos(1);
          bIsSingular |= m_cpSurface[1]->IsSingularity( sUV, eWhichDir );
        }
    }

  if ( bIsSingular )
    {
      SmTsectPnt *saTsectPnts[8];
      SmTArray<SmTsectPnt*> sTsectPoints(8,saTsectPnts);

      // let pTSP = last entry of the input throughPoint list
      SmTsectPnt *pTSPts = crTsectPoints[lTotalTsectPoints-1];

      // let sEndTSP = current intersection 3DCurve endPoint
      SmTsectPnt sEndTSP;
      SmPoint3d sPnt;
      SmExtent1d sIvl1 = rp3DCurve->GetNaturalInterval();
      SER(sSurface1UVCurves[0]->EvaluatePoint(sIvl1.GetMax(),sPnt));
      sEndTSP.UVPos(0).x = sPnt.x;
      sEndTSP.UVPos(0).y = sPnt.y;
      SER(sSurface2UVCurves[0]->EvaluatePoint(sIvl1.GetMax(),sPnt));
      sEndTSP.UVPos(1).x = sPnt.x;
      sEndTSP.UVPos(1).y = sPnt.y;
      
      // set through points = [sEndTSP, pTSPts]
      sTsectPoints.Add(&sEndTSP);
      sTsectPoints.Add(pTSPts);

      // init data structures for recursive call 
      m_vCurvePoints.Init();
      m_vStartPoints.Init();
      m_vThroughPoints.Init();
      SmBSplineCurve * p3DCurve2        = NULL ;
      SmBSplineCurve * pSurface1UVCurve = NULL ;
      SmBSplineCurve * pSurface2UVCurve = NULL ;
      SmTsectCurveType eCurveType;
      double dDeviation;
      SmTemporaryChangeValue<SmBoolean> sChange(bInsideMe,TRUE);

      // try to get intersection from other end to here
      SER(DoPointIntersection(*m_cpContext, sTsectPoints, 
                              bExtendBeforeStart, bExtendAfterEnd, 
                              pOptStartDirection, pOptEndDirection,
                              pOptApproxTol3d, pdOptAngTolRad, 
                              p3DCurve2, pSurface1UVCurve, pSurface2UVCurve, 
                              eCurveType, dDeviation));
     
      // when 2nd effort DoPointIntersection succeeded
      if (p3DCurve2) 
        {
          // get 1stCurve maxPoint
          SmPoint3d sCrv1Pnt, sCrv2Pnt;
          SER(rp3DCurve->EvaluatePoint(sIvl1.GetMax(),sCrv1Pnt));

          // get 2ndCurve minPoint
          SmExtent1d sIvl2 = p3DCurve2->GetNaturalInterval();
          SER(p3DCurve2->EvaluatePoint(sIvl2.GetMin(),sCrv2Pnt));
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe1 = FALSE;
          if (bDebugMe1) 
            {
              smgfx_SetColor(1,0,0);
              rp3DCurve->DrawWDeriv(sIvl1,0); sm_GraphicsLoop();
              sCrv1Pnt.Draw(); sm_GraphicsLoop();
              smgfx_SetColor(1,0,1);
              p3DCurve2->DrawWDeriv(sIvl2,0); sm_GraphicsLoop();
              sCrv2Pnt.Draw(); sm_GraphicsLoop();
              sm_GraphicsLoop();
            }
#endif
          // when curve endPoints are close enough
          if (sCrv1Pnt.CloserThan(m_dThisApproxTol3d,sCrv2Pnt) ) 
          //if (sCrv1Pnt.DistanceBetween(sCrv2Pnt) < m_dThisApproxTol3d) 
            {
              // Join 2 curves
              SER(rp3DCurve->JoinWith(1,         // in : 1 = join at ThisCurve end
                                      p3DCurve2, // in : OtherCurve
                                      0));       // in : 0 = join at OtherCurve start
              SER(rpSurface1UVCurve->JoinWith(1,pSurface1UVCurve,0));
              SER(rpSurface2UVCurve->JoinWith(1,pSurface2UVCurve,0));
              SmObjDelete sDelete0(p3DCurve2);
              SmObjDelete sDelete1(pSurface1UVCurve);
              SmObjDelete sDelete2(pSurface2UVCurve);
            } // end curves on either side of a singularity point are close enough check
        } // end found an intersection curve starting at singularity point
    } // end stopped prematurely on a singularity point check

  // all done
  return SM_SUCCESS;

} // end SmSurfaceIntersector::DoPointIntersection
                                              
/*******************************************************************//**
PURPOSE: This method reverses the orientation (tangent directions
   and order) of the current intersection curve.  This is used when
   curves need to be traced in both directions from the starting point.

NOTES: modifies m_vCurvePoints array members.
  1. negates tangent vector directions:
           CrvDeriv(), UVDeriv(0), UVDeriv(1);
  2. changes parameterization so that old-end is given parameter = 0
       and all other points are ordered in increasing parameterizations.
       note: old parameter range = [0,max] will be the same as the
             new parameter range = [0,max].
  3. reverses the order of SmTSectPnts appear in the m_vCurvePoints array.
          
***********************************************************************/
SmStatus 
SmSurfaceIntersector::ReverseCurveDirection()
{
  //if (m_bDoingExtensionClipping) {
  //    m_bDoingExtensionClipping = FALSE;
  //}
  SmTsectPnt *pLastPoint = m_vCurvePoints.GetLastNode();
  NER(pLastPoint);
  SmTsectPnt *aData[400];

  // store m_vCurvePoints in local array
  SmTArray<SmTsectPnt*> sPnts(400,aData);
  m_vCurvePoints.GetAllNodes(sPnts);

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
  if (bDebugMe) {
      smos_WriteBuffer(_T("\n\nBefore RCD\n"));
      for (ULONG j=0; j<sPnts.GetSize(); j++) {
          TCHAR sBuff[SM_TBLOCK_SIZE];
          smos_sprintf(sBuff,_T("pnts[%ld] = %16.16lf\n"),j,sPnts[j]->m_dCurveParameter);
          smos_WriteBuffer(sBuff);
      }
  }
#endif

  // clear m_vCurvePoints array
  m_vCurvePoints.Init();

  // for every Point in reverse order
  double dCurrParam = 0.0;
  for (ULONG i=sPnts.GetSize(); i>0; i--) 
    {
      SmTsectPnt *pPnt = sPnts[i-1];

      // negate the tangent directions
      pPnt->CrvDeriv() = - pPnt->CrvDeriv();
      pPnt->UVDeriv(0) = - pPnt->UVDeriv(0);
      pPnt->UVDeriv(1) = - pPnt->UVDeriv(1);

      // change parameters so that old end is parameter = 0
      // and all other points working to old begin is an increasing parameter
      double dDeltaParam = 0.0;
      if (i > 1) 
        {
          dDeltaParam = pPnt->m_dCurveParameter - 
                        sPnts[i-2]->m_dCurveParameter;
        }
      pPnt->m_dCurveParameter = dCurrParam;
      dCurrParam = dCurrParam + dDeltaParam;
      m_vCurvePoints.Append(pPnt);

    } // end iter every point negating tangent vecs, assigning a new
      //  parameter value, and negating the point order in the m_vCurvePoints array 

#ifdef SM_DEBUG_CODE
  if (bDebugMe) 
    {
      smos_WriteBuffer(_T("\n\nAfter RCD\n"));
      for (ULONG j=0; j<sPnts.GetSize(); j++) 
        {
          TCHAR sBuff[SM_TBLOCK_SIZE];
          smos_sprintf(sBuff,_T("pnts[%ld] = %16.16lf\n"),j,sPnts[j]->m_dCurveParameter);
          smos_WriteBuffer(sBuff);
        }
    }
#endif // SM_DEBUG_CODE

  // all done
  return SM_SUCCESS;

} // end SmSurfaceIntersector::ReverseCurveDirection

/*******************************************************************//**
PURPOSE: Determine if a point is on one of the curves that have already
    been generated by intersection.

NOTES: 
***********************************************************************/
SmBoolean SmSurfaceIntersector::IsPointOnCurve
  (const SmPoint3d & crPointToTest,          // in : point to test
   const SmTArray<SmCurve*> & cr3DCurves,    // in : curves to test
   ULONG *pOptSkipIndex)                     // in : skip this curve, NULL to ignore 
   const
{
  // GWC_NEEDS_WORK To_trace_multiple_curves_from_a_singularity_point__Extend_this_to_test_for_singularity_pts_and_directions GWC_LINE ;

  SmBoolean bPointIsOnCurve = FALSE;

  // for every cr3DCurve
  for (ULONG jj=0; jj<cr3DCurves.GetSize(); jj++) 
    {
      // when given an optional skip index - skip it
      if(pOptSkipIndex && jj == *pOptSkipIndex) 
        { continue ; }

      SmCurve *pCurve = cr3DCurves[jj];
      double dCrvParam, dDist;
      SmBoolean bSuccess;

      // if the curve is degenerate (can be treated as a single point)
      if (pCurve->IsDegenerate()) 
        {
          SmExtent1d sIvl = pCurve->GetNaturalInterval();
          SmPoint3d sCrvPnt;
          SER(pCurve->EvaluatePoint(sIvl.GetMin(),sCrvPnt));

          // when test point is within tol*4.0 of curve point
          if (sCrvPnt.DistanceBetween(crPointToTest) < m_dThisApproxTol3d*4.0) 
            {
              // mark the point as on the curve
              bPointIsOnCurve = TRUE;
              break;
            }
        } // end degenerate curve branch
      else 
        { // the curve is non-degerenate
          SmExtent1d sIvl = pCurve->GetNaturalInterval();

          // Find nearest point on curve to test point within 10*tolerance. [B448 B517 B606]
          SER(pCurve->DropPoint(sIvl,                         // in : target curve allowed domain
                                crPointToTest,                // in : Point to drop to curve
                                NULL,                         // in : For Snapping, NULL=NoSnap, Vec pointing to CrvInside from desired EndPt for snapping.
                                                              //      when DropPt is within dDistTol in cpVecToCrvInside dir of EndPt, Snap to Ivl EndPt.
                                                              //      Vec handles ambiguities and makes sure that curves are not just touching at the ends.
                                m_dThisApproxTol3d * 10.0,    // in : Used for SnapDist, and SM_SO_INTERSECT sXSectTol3d distance.
                                                              //      sXSectTol3d = dDistTol ;      SM_SO_INTERSECT 
                                                              //      sXSectTol3d = SM_BIG_DOUBLE ; SM_SO_MINIMIZE, SM_SO_NORMALIZE
                                                              //      sSnapTol3d  = dDistTol ;      SM_SO_MINIMIZE, SM_SO_NORMALIZE, SM_SO_INTERSECT    
                                NULL,                         // in : If given, call cheap LocalPointSolve() else call expensive GlobalPointSolve()
                                bSuccess,                     // out: TRUE = found a drop point
                                dCrvParam,                    // out: found drop curve param
                                dDist,                        // out: found drop distance
                                SM_SO_INTERSECT ));           // in : SM_SO_MINIMIZE = allow nonNormal drops near endPoints
                                                              //      SM_SO_INTERSECT= like Minimize but point must be within dDistTol
                                                              //      SM_SO_NORMALIZE= exclude nonNormal drops near endPoints
                                                              //      default:[SM_SO_MINIMIZE] to preserve original behavior

          // If the point is within tolerance, it's on the curve.
          if (bSuccess && dDist < m_dThisApproxTol3d) 
            {
              bPointIsOnCurve = TRUE;
              break;
            }

          // If within 10x tol, then check it more closely.
          // The question is, is crPointToTest on this intersection curve,
          // or on a separate, nearby one?
          // We'll check the distances to the surface at crPointToTest
          // and at the point on the curve.  If the surfaces diverge in
          // between, then it looks like two separate intersection curves.

          // Also, since we're dealing with a tolerance-sized area, we can
          // assume that the surfaces don't do anything wild in between.

          if ( bSuccess && dDist < m_dThisApproxTol3d * 10 )
            {
              const SmSurface *pSrf1 = m_cpSurface[0];
              const SmSurface *pSrf2 = m_cpSurface[1];
              SmBoolean bOk;
              SmBoolean bIsMulti;
              SmExtent2d sDom1 = pSrf1->GetNaturalUVDomain();
              SmExtent2d sDom2 = pSrf2->GetNaturalUVDomain();
              double dIntPtGap1 = 0.0, dIntPtGap2 = 0.0;

              SmPoint2d sIntPtUV1, sIntPtUV2;
              // (Note, these IntPtGaps are presumably 'zero', but we need the uv's.)
              pSrf1->DropPoint( crPointToTest, sDom1, NULL, bOk, sIntPtUV1, dIntPtGap1, bIsMulti );
              pSrf2->DropPoint( crPointToTest, sDom2, NULL, bOk, sIntPtUV2, dIntPtGap2, bIsMulti );

              SmPoint3d sCrvPt;
              pCurve->EvaluatePoint( dCrvParam, sCrvPt );

              SmPoint2d sCrvPtUV1, sCrvPtUV2;
              double dCrvPtGap1 = 0.0, dCrvPtGap2 = 0.0;
              pSrf1->DropPoint( sCrvPt, sDom1, &sIntPtUV1, bOk, sCrvPtUV1, dCrvPtGap1, bIsMulti );
              pSrf2->DropPoint( sCrvPt, sDom2, &sIntPtUV2, bOk, sCrvPtUV2, dCrvPtGap2, bIsMulti );

              SmPoint3d sMidSrf1, sMidSrf2;

              SmPoint2d sMidPtUV1( ( sIntPtUV1 + sCrvPtUV1 ) / 2 );
              SmPoint2d sMidPtUV2( ( sIntPtUV2 + sCrvPtUV2 ) / 2 );
              pSrf1->EvaluatePoint( sMidPtUV1, sMidSrf1 );
              pSrf2->EvaluatePoint( sMidPtUV2, sMidSrf2 );

              double dGapMid = sMidSrf1.DistanceBetween( sMidSrf2 );

              if ( dGapMid < ( dIntPtGap1+dIntPtGap2 ) + ( dCrvPtGap1+dCrvPtGap2 ) )
                {
                  bPointIsOnCurve = TRUE;
                  break;
                }
            } // end if dDist within 10 x tol
        } // end non-degenerate curve branch
    } // end iter every cr3DCurve

  return bPointIsOnCurve;

} // end SmSurfaceIntersector::IsPointOnCurve

/*******************************************************************//**
PURPOSE: Determine if a point is coincident with one of the existing
    start points.

NOTES: 
***********************************************************************/
SmBoolean SmSurfaceIntersector::IsPointOnStartPoint
  (const SmTsectPnt          & crPointToTest,  // in : point to test
   const SmTList<SmTsectPnt> & crStartPoints,  // in : StartPoints to test
   ULONG                     * pOptSkipIndex)  // in : skip this index, NULL to ignore 
 const
{
  SmBoolean bPointIsOnStartPoint = FALSE;

  SmTsectPnt &rTestPt = (SmTsectPnt &)crPointToTest ;

  const SmPoint3d &crTestPt0 = rTestPt.SrfPos(0) ;
  //      const SmPoint2d &crTestUV0 = rTestPt.UVPos(0) ;

  const SmPoint3d &crTestPt1 = rTestPt.SrfPos(1) ;
  //      const SmPoint2d &crTestUV1 = rTestPt.UVPos(1) ;

  //      double dTolUV0 = m_cpSurface[0]->GetNaturalUVDomain().GetMaxDimension() / 1000.0 ;
  //      double dTolUV1 = m_cpSurface[1]->GetNaturalUVDomain().GetMaxDimension() / 1000.0 ;

  // for every StartPoint
  ULONG jj = 0 ;
  SmTsectPnt * pFirstPoint = crStartPoints.GetFirstNode() ;
  SmTsectPnt * pStartPoint = pFirstPoint ;
  for(pStartPoint=crStartPoints.GetFirstNode(); 
      pStartPoint!=NULL; 
      pStartPoint = (pStartPoint->GetNext() == pFirstPoint ? NULL : (SmTsectPnt *)pStartPoint->GetNext()), jj++)
    {
      // when given an optional skip index - skip it
      if(pOptSkipIndex && jj == *pOptSkipIndex) 
        { continue ; }

      const SmPoint3d &crStartPt0 = pStartPoint->SrfPos(0) ;
      //      const SmPoint2d &crStartUV0 = pStartPoint->UVPos(0) ;

      const SmPoint3d &crStartPt1 = pStartPoint->SrfPos(1) ;
      //      const SmPoint2d &crStartUV1 = pStartPoint->UVPos(1) ;

      // when 3d point is within tol of both Surface Start 3d and UV points
      if(   crStartPt0.DistanceBetweenSquared(crTestPt0) < m_dThisApproxTol3d
         //      && crStartUV0.DistanceBetweenSquared(crTestUV0) < dTolUV0
         && crStartPt1.DistanceBetweenSquared(crTestPt1) < m_dThisApproxTol3d)
         //      && crStartUV1.DistanceBetweenSquared(crTestUV1) < dTolUV1)
        { 
          // the test point is already a start point
          bPointIsOnStartPoint = TRUE ; 
          break ;
        }
    } // end iter every cr3DCurve

  return bPointIsOnStartPoint ;

} // end SmSurfaceIntersector::IsPointOnStartPoint

/*******************************************************************//**
PURPOSE: Compute Surf UV values of Offset Surface/Surface/Plane intersection
         given a guess point on the two surfaces.

NOTES: uses offsets to m_cpSurfaces.
       computing the Surf/Surf/Plane intersection point
         of the two fillet Offset-surfaces finds the
         rail points on the the original fillet surfaces. 
***********************************************************************/
SmStatus SmSurfaceIntersector::EvaluateLawPoint
 (double              dParameter,              // in : filletCurve parameter - used to compute current fillet radius
  const SmPoint2d   & crUV0,                   // in : guess uv point on m_cpSurface[0]
  const SmPoint2d   & crUV1,                   // in : guess uv point on m_cpSurface[1] 
  const SmFilletLaw & crLawCurve,              // in : FilletLaw to compute fillet-radius for every Curve param 
  SmBoolean           bLawOrient,              // in : bReverseOrientation: TRUE = parameters run from interval end to interval start
  const SmExtent1d  & crPointCurveInterval,    // in : crCurveInterval = interval defining range of fillet edge 
  double              dSurfaceOrientations[2], // in : Offset directons for m_cpSurface[0,1], either -1.0 or 1.0 values. 
  const SmPoint3d   & rPlaneOrigin,            // in : Origin of Plane(origin, normal)
  const SmVector3d  & rPlaneNormal,            // in : Normal of Plane(origin, normal)
  double            & ,                        // out: rdCurveParam - not set by this method
  SmPoint2d           sUVs[2])                 // out: Surf Params of m_pSurface[0]/m_pSurface[0]/plane XSect result
{

  // Set up offset values on surfaces.
  double dOffsets[3]; // out: dValues[0] = fillet-radius at dParameter value
                      //      dValues[1] = 1st derivative: d(fillet-radius)/d(Parameter)
                      //      dValues[2] = 2nd derivative: d2(fillet-radius)/d(Parameter)**2

  // get fillet radius and its derivatives at a point along a fillet edge
  SER(crLawCurve.Evaluate(dParameter,           // in : dParameter = target parameter - range:[crCurveInterval.Min,Max]
                          crPointCurveInterval, // in : crCurveInterval = =interval defining range of fillet edge
                          bLawOrient,           // in : bReverseOrientation: TRUE = parameters run from interval end to interval start
                          dOffsets));           // out: dValues[0] = fillet-radius at dParameter value
                                                //      dValues[1] = 1st derivative: d(fillet-radius)/d(Parameter)
                                                //      dValues[2] = 2nd derivative: d2(fillet-radius)/d(Parameter)**2

  // Cast stored m_cpSurfaces to Offset Surfaces
  SmOffsetSurface *pOffSet1 = SM_CONST_CAST(SmOffsetSurface*,
                                            SM_CAST_PTR(SmOffsetSurface,this->m_cpSurface[0]));
  SmOffsetSurface *pOffSet2 = SM_CONST_CAST(SmOffsetSurface*,
                                            SM_CAST_PTR(SmOffsetSurface,this->m_cpSurface[1]));

  // set Offset Surface Offset signed distances
  pOffSet1->SetOffsetDistance(dOffsets[0]*dSurfaceOrientations[0]);
  pOffSet2->SetOffsetDistance(dOffsets[0]*dSurfaceOrientations[1]);
  
  // solver locals  
  SmBoolean  bFoundAnswer;
  SmSolution sSol;

  // do surf/surf/plane intersection
  SER(pOffSet1->LocalPlaneSurfaceIntersect(m_vUVDomain[0],        // in : this surface domain of interest
                                           *pOffSet2,             // in : other surface
                                           m_vUVDomain[1],        // in : other surface domain of interest
                                           m_dThisApproxTol3d,    // in : Max allowed distance between found Surface intersection points
                                           crUV0,                 // in : this Surface initial UV guess
                                           crUV1,                 // in : other Surface initial UV guess
                                           rPlaneOrigin,          // in : origin of Plane(origin, normal)
                                           rPlaneNormal,          // in : normal of Plane(origin, normal)
                                           bFoundAnswer,          // out: TRUE = found a solution
                                           sSol));                // out: when rbFoundAnser==TRUE, the solution
  if (!bFoundAnswer) 
    { SER(SM_ERR); }
  
  // set output
  sUVs[0] = m_vUVDomain[0].ClampPoint2d(SmPoint2d(sSol.m_vStart[0],sSol.m_vStart[1]));
  sUVs[1] = m_vUVDomain[1].ClampPoint2d(SmPoint2d(sSol.m_vStart[2],sSol.m_vStart[3]));

  // all done
  return SM_SUCCESS;

} // end SmSurfaceIntersector::EvaluateLawPoint


/*******************************************************************//**
PURPOSE: compute SmTsectPnt values at the Offset Surf/Surf/Plane intersection
         point for given fillet curve param value

NOTES: 
***********************************************************************/
SmStatus SmSurfaceIntersector::ComputeLawPointValues
 (double              dParameter,              // in : Param for crPointCurve eval
  const SmPoint2d   & crUV0,                   // in : guess uv point on m_cpSurface[0]
  const SmPoint2d   & crUV1,                   // in : guess uv point on m_cpSurface[1]
  double              dDeltaStep,              // in : param increment used to estimate crPointCurve tangent direction
  const SmVector3d    caNormals[2],            // in : when bAverageNormals = TRUE, PlaneNormals at crPointCurve crPointCurveInterval bounds
  SmBoolean           bAverageNormals,         // in : TRUE = PlaneNormals(param) = LinearInterp(Param,Ivl) of caNormals values
                                               //      FALSE= PlaneNormals(param) = crPointCurve(Param) tangents
  const SmFilletLaw & crLawCurve,              // in : FilletLaw to compute fillet-radius for every Curve param
  SmBoolean           bLawOrient,              // in : bReverseOrientation: TRUE = parameters run from interval end to interval start
  const SmCurve     & crPointCurve,            // in : Should be Arc Length Parameterized
  const SmExtent1d  & crPointCurveInterval,    // in : interval defining range of fillet edge 
  double              dSurfaceOrientations[2], // in : Offset directons for m_cpSurface[0,1], either -1.0 or 1.0 values.
  SmBoolean           bSkipComputingUVS,       // in : TRUE = use rTSectPnt, UVPos(0), UVPos(1), and Param values
                                               //      FALSE= refine rTSectPnt values to Surf/Surf/Plane xsect before using
  SmTsectPnt        & rTsectPnt)               // i/o: in : bSkipComputingUVS== TRUE  rTsectPnt.UVPos(0);
                                               //                                     rTsectPnt.UVPos(1);
                                               //                                     rTsectPnt.m_adUserDoubles[0];
                                               //      out: bSkipComputingUVS== FALSE rTsectPnt.UVPos(0);
                                               //                                     rTsectPnt.UVPos(1);
                                               //                                     rTsectPnt.m_adUserDoubles[0];
                                               //           m_ePointType etc.
{
  // for 6 iterations
  for (ULONG i=0; i<6; i++) 
    {
      // First compute the plane and delta plane to be used to intersect
      double     dDeltaParam = dParameter + dDeltaStep;
      SmVector3d sPV[2], sPVDelta[2];
      crPointCurve.Evaluate(dParameter,  1, TRUE, sPV);
      crPointCurve.Evaluate(dDeltaParam, 1, TRUE, sPVDelta);
        
      if (bAverageNormals) 
        {
          double dNormParam;
          crPointCurveInterval.Inversion(dParameter,dNormParam);
          sPV[1] = (1.0-dNormParam)*caNormals[0] + dNormParam*caNormals[1];

          crPointCurveInterval.Inversion(dDeltaParam,dNormParam);
          sPVDelta[1] = (1.0-dNormParam)*caNormals[0] + dNormParam*caNormals[1];       
        }
      
      // unitize weighted input caNormal values  
      SER(sPV[1].Unitize());
      SER(sPVDelta[1].Unitize());
        
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe=FALSE;
      if (bDebugMe) 
        {
          sm_GraphicsLoop();

          smgfx_SetColor(0,1,0); sPV[0].Draw(); sm_GraphicsLoop();
          smgfx_SetColor(0,1,0); sPV[1].Draw(&sPV[0]); sm_GraphicsLoop();
          
          smgfx_SetColor(0,1,0); sPVDelta[0].Draw(); sm_GraphicsLoop();
          smgfx_SetColor(0,1,0); sPVDelta[1].Draw(&sPVDelta[0]); sm_GraphicsLoop();
          smgfx_SetColor(1,0,0); crPointCurve.Draw(); sm_GraphicsLoop();
          smgfx_SetColor(0,0,1); caNormals[0].Draw(&sPV[0]); sm_GraphicsLoop();
          smgfx_SetColor(0,0,1); caNormals[1].Draw(&sPV[0]); sm_GraphicsLoop();
      }
#endif // SM_DEBUG_CODE
        
      // Now do the intersection and get the parameters on the two base surfaces.
      SmPoint2d sUVs[2], sUVDeltas[2];
      double dCurveParam, dCurveDeltaParam;
      if (bSkipComputingUVS) 
        {
          sUVs[0]     = rTsectPnt.UVPos(0);
          sUVs[1]     = rTsectPnt.UVPos(1);
          dCurveParam = rTsectPnt.m_adUserDoubles[0];
        }
      else 
        {
          // Compute Surf UV values of offset Surface/Surface/Plane intersection
          // given a guess point on the two surfaces.
          SER(EvaluateLawPoint(dParameter,           // in : filletCurve parameter - used to compute current fillet radius
                               crUV0,                // in : guess uv point on m_cpSurface[0]
                               crUV1,                // in : guess uv point on m_cpSurface[1] 
                               crLawCurve,           // in : FilletLaw to compute fillet-radius for every Curve param 
                               bLawOrient,           // in : bReverseOrientation: TRUE = parameters run from interval end to interval start
                               crPointCurveInterval, // in : crCurveInterval = interval defining range of fillet edge 
                               dSurfaceOrientations, // in : Offset directons for m_cpSurface[0,1], either -1.0 or 1.0 values.
                               sPV[0],               // in : Origin of Plane(origin, normal)
                               sPV[1],               // in : Normal of Plane(origin, normal)
                               dCurveParam,          // out: rdCurveParam 
                               sUVs));               // out: Surf Params of m_pSurface[0]/m_pSurface[0]/plane XSect result

          rTsectPnt.m_adUserDoubles[0] = dCurveParam;
        }

      // Compute Surf UV values of Surface/Surface/Plane intersection
      // given a guess point on the two surfaces.
      SER(EvaluateLawPoint(dDeltaParam,          // in : filletCurve parameter - used to compute current fillet radius
                           crUV0,                // in : guess uv point on m_cpSurface[0]
                           crUV1,                // in : guess uv point on m_cpSurface[1] 
                           crLawCurve,           // in : FilletLaw to compute fillet-radius for every Curve param 
                           bLawOrient,           // in : bReverseOrientation: TRUE = parameters run from interval end to interval start
                           crPointCurveInterval, // in : crCurveInterval = =interval defining range of fillet edge 
                           dSurfaceOrientations, // in : Offset directons for m_cpSurface[0,1], either -1.0 or 1.0 values.
                           sPVDelta[0],          // in : Origin of Plane(origin, normal)
                           sPVDelta[1],          // in : Normal of Plane(origin, normal)
                           dCurveDeltaParam,     // out: rdCurveParam - not set by this method
                           sUVDeltas));          // out: Surf Params of m_pSurface[0]/m_pSurface[0]/plane XSect result
     
      // arrive here: have Surf/Surf/Plane XSectPt for both dParam and dDeltaParam
      
      // build DeltaParam SmTsectPnt   
      SmTsectPnt sDeltaPnt;
//        rTsectPnt.m_adUserDoubles[0] = dParameter;
      sDeltaPnt.m_dCurveParameter  = rTsectPnt.m_dCurveParameter + dDeltaStep;
      sDeltaPnt.m_adUserDoubles[0] = dCurveDeltaParam;
      sDeltaPnt.m_adUserDoubles[1] = dDeltaParam;
      
      // Get Surf and Crv Pt3d, UVPt, and deriv values for Rail Surf UVPoint values  
      if(SM_SUCCESS == ComputePointValuesDelta(sUVs,       // in : TgtParam      rail Surf UVPt values to evaluate
                                               sUVDeltas,  // in : TgtDeltaParam rail Surf UVPt values to evaluate
                                               rTsectPnt,  // out: container for TgtParam surface and Crv 3dPt, UVPos and derivative values
                                               sDeltaPnt)) // out: container for TgtDeltaParam surface and Crv 3dPt, UVPos and derivative values
        { return SM_SUCCESS; }

      // set up for next iteration with a bigger dDeltaStep
      dDeltaStep *= 10.0;

    } // end iter 6 times

  // all done
  return SM_ERR;

} // end SmSurfaceIntersector::ComputeLawPointValues

/*******************************************************************//**
PURPOSE: 

NOTES: 
***********************************************************************/
SmStatus SmSurfaceIntersector::SplitSpan
 (const SmTsectPnt  * cpCurrConst,
  const SmTsectPnt  * cpNextConst,
  double              dDeltaStep,
  const SmVector3d    caNormals[2],
  SmBoolean           bAverageNormals,
  const SmFilletLaw & crLawCurve,
  SmBoolean           bLawOrient,
  const SmCurve     & crPointCurve,
  const SmExtent1d  & crPointCurveInterval,
  double              dSurfaceOrientations[2],
  SmTsectPnt       *& rpNewTsect)
{
    SmTsectPnt *pCurrTSP = SM_CONST_CAST( SmTsectPnt*, cpCurrConst );
    SmTsectPnt *pNextTSP = SM_CONST_CAST( SmTsectPnt*, cpNextConst );

    SmTsectPnt *pNewTSP = (SmTsectPnt*)m_vTSPntMgr.GetNewElement();
    pNewTSP->m_ePointType = SM_IP_CROSSING;

    // Compute the Plane point and normal from the curve and/or previous normals
   // SmPoint3d sPV[2];
    double dCurrParam = pCurrTSP->m_adUserDoubles[1];
    double dNextParam = pNextTSP->m_adUserDoubles[1];
    double dMidParam = (dCurrParam+dNextParam) / 2.0;
    pNewTSP->m_adUserDoubles[1] = dMidParam;

    SmPoint2d sUV0 = (pCurrTSP->UVPos(0) + pNextTSP->UVPos(0)) / 2.0;
    SmPoint2d sUV1 = (pCurrTSP->UVPos(1) + pNextTSP->UVPos(1)) / 2.0;

    pNewTSP->m_dCurveParameter = (pCurrTSP->m_dCurveParameter + pNextTSP->m_dCurveParameter)/2.0;

    SER(ComputeLawPointValues(dMidParam,            // in : Param for crPointCurve eval
                              sUV0,                 // in : guess uv point on m_cpSurface[0]
                              sUV1,                 // in : guess uv point on m_cpSurface[1]
                              dDeltaStep,           // in : param increment used to estimate crPointCurve tangent direction
                              caNormals,            // in : when bAverageNormals = TRUE, PlaneNormals at crPointCurve crPointCurveInterval bounds
                              bAverageNormals,      // in : TRUE = PlaneNormals(param) = LinearInterp(Param,Ivl) of caNormals values
                                                    //      FALSE= PlaneNormals(param) = crPointCurve(Param) tangents
                              crLawCurve,           // in : FilletLaw to compute fillet-radius for every Curve param
                              bLawOrient,           // in : bReverseOrientation: TRUE = parameters run from interval end to interval start
                              crPointCurve,         // in : Should be Arc Length Parameterized
                              crPointCurveInterval, // in : interval defining range of fillet edge 
                              dSurfaceOrientations, // in : Offset directons for m_cpSurface[0,1], either -1.0 or 1.0 values.
                              FALSE,                // in : TRUE = use rTSectPnt, UVPos(0), UVPos(1), and Param values
                                                    //      FALSE= refine rTSectPnt values to Surf/Surf/Plane xsect before using
                              *pNewTSP));           // i/o: in : bSkipComputingUVS== TRUE  rTsectPnt.UVPos(0);
                                                    //                                     rTsectPnt.UVPos(1);
                                                    //                                     rTsectPnt.m_adUserDoubles[0];
                                                    //      out: bSkipComputingUVS== FALSE rTsectPnt.UVPos(0);
                                                    //                                     rTsectPnt.UVPos(1);
                                                    //                                     rTsectPnt.m_adUserDoubles[0];
                                                    //           m_ePointType etc.

    double dDist = pCurrTSP->CrvPos().DistanceBetween(pNewTSP->CrvPos());
    if (dDist > SM_ZONE_TOL_3D * (dNextParam - dCurrParam)) 
      { pNewTSP->m_dCurveParameter = pCurrTSP->m_dCurveParameter + dDist; }

    rpNewTsect = pNewTSP;

    return SM_SUCCESS;

} // end SmSurfaceIntersector::SplitSpan

/*******************************************************************//**
PURPOSE: This function basically traces out a law curve of offsets
    of a surface given a start and end point with values corresponding 
    to the start and end of the law curve radii.  The primary use for
    this tool is to create variable radius fillets.

NOTES: Assumes that the input surfaces are SmOffsetSurface
***********************************************************************/
SmStatus SmSurfaceIntersector::DoLawIntersection
 (const SmContext             & crContext,                // NotUsed: in : 
  const SmTArray<SmTsectPnt*> & crTsectPoints,            // in : 
  const SmFilletLaw           & crLawCurve,               // in : 
  SmBoolean                     bLawOrient,               // in : 
  const SmCurve               & crPointCurve,             // in : Should be Arc Length Parameterized
  const SmExtent1d            & crPointCurveInterval,     // in : 
  double                        dSurfaceOrientations[2],  // in : Offset directons for m_cpSurface[0,1], either -1.0 or 1.0 values.
  SmBoolean                     bUseNormalPlaneAveraging, // in : 
  SmBoolean                     bUniformSteps,            // NotUsed: in : 
  const SmApproxTol3d         * pOptApproxTol3d,          // in : 
  const double                * pdOptAngTolRad,           // NotUsed: in : 
  SmBSplineCurve              *& rp3DCurve,               // out: Resulting curve - note that this will be NULL
                                                          //      if we are unable to find a curve that satisfies
                                                          //      the input requirements and passes through all given points.
  SmBSplineCurve              *& rpSurface1UVCurve,       // out: 
  SmBSplineCurve              *& rpSurface2UVCurve,       // out: 
  SmTsectCurveType             & reCurveType,             // out: 
  double                       & rdDeviation)             // out: 
{
  SM_REF3(crContext, bUniformSteps, pdOptAngTolRad) ;
  if (crTsectPoints.GetSize() != 2) 
    { SER(SM_ERR); }

  // First let's set up our point curve information.
  double d3DTol = 1.0e-3;
  //double dAngleTolRadians = SM_DEG2RAD( 20 );
  if (pOptApproxTol3d) 
    {
      d3DTol = *pOptApproxTol3d;
    }
  m_dThisApproxTol3d = d3DTol;
        
  double dDeltaStep = 1.0e-8;
 
  // Make our Offset Surfaces
  SmOffsetSurface *pOffSet1 = SM_CONST_CAST(SmOffsetSurface*,
                                            SM_CAST_PTR(SmOffsetSurface,this->m_cpSurface[0]));
  SmOffsetSurface *pOffSet2 = SM_CONST_CAST(SmOffsetSurface*,
                                            SM_CAST_PTR(SmOffsetSurface,this->m_cpSurface[1]));
  NER(pOffSet1);
  NER(pOffSet2);

  // Set up stack 
  SmTsectPnt *pFirst = crTsectPoints[0];
  SmTsectPnt *pLast = crTsectPoints.GetLast();

  SmVector3d sNormals[2];
  sNormals[0] = pFirst->CrvDeriv();
  sNormals[1] = pLast->CrvDeriv();

  pFirst->m_adUserDoubles[0] = pFirst->m_adUserDoubles[1] = crPointCurveInterval.GetMin();
  pLast->m_adUserDoubles[0] = pLast->m_adUserDoubles[1] = crPointCurveInterval.GetMax();

  SER(ComputeLawPointValues(pFirst->m_adUserDoubles[1], // in : Param for crPointCurve eval
                            pFirst->UVPos(0),           // in : guess uv point on m_cpSurface[0]
                            pFirst->UVPos(1),           // in : guess uv point on m_cpSurface[1]
                            dDeltaStep,                 // in : param increment used to estimate crPointCurve tangent direction
                            sNormals,                   // in : when bAverageNormals = TRUE, PlaneNormals at crPointCurve crPointCurveInterval bounds
                            bUseNormalPlaneAveraging,   // in : TRUE = PlaneNormals(param) = LinearInterp(Param,Ivl) of caNormals values
                                                        //      FALSE= PlaneNormals(param) = crPointCurve(Param) tangents
                            crLawCurve,                 // in : FilletLaw to compute fillet-radius for every Curve param
                            bLawOrient,                 // in : bReverseOrientation: TRUE = parameters run from interval end to interval start
                            crPointCurve,               // in : Should be Arc Length Parameterized
                            crPointCurveInterval,       // in : interval defining range of fillet edge 
                            dSurfaceOrientations,       // in : Offset directons for m_cpSurface[0,1], either -1.0 or 1.0 values.
                            FALSE,                      // in : TRUE = use rTSectPnt, UVPos(0), UVPos(1), and Param values
                                                        //      FALSE= refine rTSectPnt values to Surf/Surf/Plane xsect before using
                            *pFirst));                  // i/o: in : bSkipComputingUVS== TRUE  rTsectPnt.UVPos(0);
                                                        //                                     rTsectPnt.UVPos(1);
                                                        //                                     rTsectPnt.m_adUserDoubles[0];
                                                        //      out: bSkipComputingUVS== FALSE rTsectPnt.UVPos(0);
                                                        //                                     rTsectPnt.UVPos(1);
                                                        //                                     rTsectPnt.m_adUserDoubles[0];
                                                        //           m_ePointType etc.

  SER(ComputeLawPointValues(pLast->m_adUserDoubles[1], // in : Param for crPointCurve eval
                            pLast->UVPos(0),           // in : guess uv point on m_cpSurface[0]
                            pLast->UVPos(1),           // in : guess uv point on m_cpSurface[1]
                            -dDeltaStep,               // in : param increment used to estimate crPointCurve tangent direction
                            sNormals,                  // in : when bAverageNormals = TRUE, PlaneNormals at crPointCurve crPointCurveInterval bounds
                            bUseNormalPlaneAveraging,  // in : TRUE = PlaneNormals(param) = LinearInterp(Param,Ivl) of caNormals values
                                                       //      FALSE= PlaneNormals(param) = crPointCurve(Param) tangents
                            crLawCurve,                // in : FilletLaw to compute fillet-radius for every Curve param
                            bLawOrient,                // in : bReverseOrientation: TRUE = parameters run from interval end to interval start
                            crPointCurve,              // in : Should be Arc Length Parameterized
                            crPointCurveInterval,      // in : interval defining range of fillet edge 
                            dSurfaceOrientations,      // in : Offset directons for m_cpSurface[0,1], either -1.0 or 1.0 values.
                            FALSE,                     // in : TRUE = use rTSectPnt, UVPos(0), UVPos(1), and Param values
                                                       //      FALSE= refine rTSectPnt values to Surf/Surf/Plane xsect before using
                            *pLast));                  // i/o: in : bSkipComputingUVS== TRUE  rTsectPnt.UVPos(0);
                                                       //                                     rTsectPnt.UVPos(1);
                                                       //                                     rTsectPnt.m_adUserDoubles[0];
                                                       //      out: bSkipComputingUVS== FALSE rTsectPnt.UVPos(0);
                                                       //                                     rTsectPnt.UVPos(1);
                                                       //                                     rTsectPnt.m_adUserDoubles[0];
                                                       //           m_ePointType etc.

  pFirst->m_dCurveParameter = 0.0;
  pLast->m_dCurveParameter = pFirst->CrvPos().DistanceBetween(pLast->CrvPos());

//    AddPointToCurve(*pFirst);

  SmTArray<SmTsectPnt*> sStack;
  sStack.Add(pLast);
  sStack.Add(pFirst);
  //double dCurveParam = 0.0;

  SmTArray<SmTsectPnt*> sCurvePnts;
  sCurvePnts.Add(pFirst);

  while (sStack.GetSize() > 1) 
    {
      SmTsectPnt * pCurr = sStack.GetLast();
      sStack.RemoveLast();
      SmTsectPnt * pNext = sStack.GetLast();

      // Test this interval aginst tolerances
      double dDevFound, dAngleFoundRad;
      SmBoolean bFoundGoodPoint;

      pNext->m_dCurveParameter = pCurr->m_dCurveParameter + 
                  pCurr->CrvPos().DistanceBetween(pNext->CrvPos());
       
      // Create the New Point and stuff its values.
      SmTsectPnt *pNewTSP = NULL;
      SER(SplitSpan(pCurr,
                    pNext,
                    dDeltaStep,
                    sNormals,
                    bUseNormalPlaneAveraging,
                    crLawCurve,
                    bLawOrient,
                    crPointCurve,
                    crPointCurveInterval,
                    dSurfaceOrientations,
                    pNewTSP));
  
      SER(TestSpanAccuracy(*pCurr,*pNext,pNewTSP,bFoundGoodPoint,dDevFound,dAngleFoundRad));

      if (bFoundGoodPoint) 
        {
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe3=FALSE;
          if (bDebugMe3) 
            {
              SmVector3d sVec = pNext->CrvDeriv();
              sVec = sVec * (pNext->m_dCurveParameter - pCurr->m_dCurveParameter);
              sm_GraphicsLoop();
              pNext->CrvPos().Draw();
              sVec.Draw(&pNext->CrvPos());
              sm_GraphicsLoop();
            }
#endif
          // Here we accept the current point - put it on the list
          sCurvePnts.Add(pNext);
        }

      else 
        {
          // If need to subdivide more create mid point and put it back on
          // stack along with the pCurr.
          
          sStack.Add(pNewTSP);
          sStack.Add(pCurr);
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe=FALSE;
          if (bDebugMe)
           {
              sm_GraphicsLoop();
              pNewTSP->CrvPos().Draw(); sm_GraphicsLoop();
              pNewTSP->CrvDeriv().Draw(&pNewTSP->CrvPos()); sm_GraphicsLoop();
              pCurr->CrvPos().Draw(); sm_GraphicsLoop();
              pCurr->CrvDeriv().Draw(&pCurr->CrvPos()); sm_GraphicsLoop();
              pNext->CrvPos().Draw(); sm_GraphicsLoop();
              pNext->CrvDeriv().Draw(&pNext->CrvPos()); sm_GraphicsLoop();
              sm_GraphicsLoop();
              if (FALSE) 
                {
                  pOffSet1->DrawUV(10,10); sm_GraphicsLoop();
                  pOffSet2->DrawUV(20,20); sm_GraphicsLoop();
                }
            }
#endif // SM_DEBUG_CODE
            
        }
    }

  // Walk back through and see if we need to subdivide more
  double dSplitFactor = 2.7;
  SmBoolean bDone = FALSE;
  while (!bDone) 
    {
      bDone = TRUE;
      for (ULONG ip=1; ip<sCurvePnts.GetSize(); ip++) 
        {
          SmTsectPnt *pCurr = sCurvePnts[ip-1];
          SmTsectPnt *pNext = sCurvePnts[ip];
          SmTsectPnt *pNextNext = NULL;
          SmTsectPnt *pPrev = NULL;
          double dCurrParam = pCurr->m_dCurveParameter;
          double dNextParam = pNext->m_dCurveParameter;
          double dCurrParamRange = dNextParam - dCurrParam;
          // Test pCurr<-->pNext against previous and next next span to see if we
          // need to split it.
          SmBoolean bSplit = FALSE;
          if (ip > 1) 
            {
              pPrev = sCurvePnts[ip-2];
              double dPrevParamRange = dCurrParam - pPrev->m_dCurveParameter;
              if (dCurrParamRange > dPrevParamRange*dSplitFactor) 
                {
                  bSplit = TRUE;
                }
            }
          if (ip+1 < sCurvePnts.GetSize()) 
            {
              pNextNext = sCurvePnts[ip+1];
              double dNextParamRange = pNextNext->m_dCurveParameter - dNextParam;
              if (dCurrParamRange > dNextParamRange*dSplitFactor) 
                {
                  bSplit = TRUE;
                }
            }
          if (bSplit) 
            {
              SmTsectPnt * pNewTsect = NULL;
              SER(SplitSpan(pCurr,
                            pNext,
                            dDeltaStep,
                            sNormals,
                            bUseNormalPlaneAveraging,
                            crLawCurve,
                            bLawOrient,
                            crPointCurve,
                            crPointCurveInterval,
                            dSurfaceOrientations,
                            pNewTsect));
                  sCurvePnts.InsertAt(ip,pNewTsect);
              bDone = FALSE;
            }
        }
    }

  for (ULONG icp=0; icp<sCurvePnts.GetSize(); icp++) 
    {
       AddPointToCurve(*sCurvePnts[icp]);
    }


  SmTArray<SmCurve*> s3DCurves;
  SmTArray<SmCurve*> sSurface1UVCurves;
  SmTArray<SmCurve*> sSurface2UVCurves;
  SmTArray<double> sDeviations;
  SmTArray<SmTsectCurveType> sCurveTypes;
  m_vThroughPoints.Init();
  m_vStartPoints.Init();

  SER(FlushCurve(s3DCurves,sSurface1UVCurves,sSurface2UVCurves,
                 sCurveTypes,sDeviations));

  if (s3DCurves.GetSize() == 1) { rp3DCurve = SM_CAST_PTR(SmBSplineCurve,s3DCurves[0]); }
  if (sSurface1UVCurves.GetSize() == 1) { rpSurface1UVCurve = SM_CAST_PTR(SmBSplineCurve,sSurface1UVCurves[0]); }
  if (sSurface2UVCurves.GetSize() == 1) { rpSurface2UVCurve = SM_CAST_PTR(SmBSplineCurve,sSurface2UVCurves[0]); }
  if (sDeviations.GetSize() == 1) { rdDeviation = sDeviations[0]; }
  if (sCurveTypes.GetSize() == 1) { reCurveType = sCurveTypes[0]; }

  return SM_SUCCESS;

} // end SmSurfaceIntersector::DoLawIntersection

/*******************************************************************//**
PURPOSE: Analyze an intersection point known to lie on a boundary
    to see if it is degenerate (a single point of intersection) or 
    a part of an intersection curve

NOTES: Intersection Points can be degenerate because
  1. They are a tangent surface/surface intersection point that only
     has one point of tangency, or
  2. They are part of an intersection curve that has been trimmed down
     to a point due to surface boundaries like a piece of paper on a table
     touching a coke can.  The infinite cylinder intersects
     the infinite plane in a circle, but the finite plane intersects
     the finite cylinder at just one point.

METHOD ---
  When the XSectPoint is a SM_IP_TANGENT_CURVE or SM_IP_SINGULARITY 
    An intersection Curve exists when
    both XSect Curve UV directions point inside their respective surfaces.

  When the XSectPoint is a SM_IP_COINCIDENCE point
    An intersection Curve exists if either SurfacePoint is not on a Boundary
    or when the boundary sectors overlap.

  When the XSectPoint is a SM_IP_CROSSING point 
    both XSect Curve UV directions must point inside their respective surfaces
    and additional work is needed.
    Classify the Boundary curve against the surface as either
    tangent or crossing. When XSectPoint lies on two or more boundary curves 
    there will be two or more classifications.

  When BoundaryCurve/Surface XSect Type is crossing
    An intersection Curve exists when
    both XSect Curve UV directions point inside their respective surfaces.

  When BoundaryCurve/Surface XSect Type is tangent
    An intersection Curve exists when points along the tangent
    boundary are coincident to or cross over to the other 
    side of the surface.
***********************************************************************/
SmStatus SmSurfaceIntersector::ClassifyBoundaryStartPointIntersection
 (SmTsectPnt & rTSectPnt,        // in : Target Point with UVs, XSectCurvePoint, and SurfacePoint data
  SmBoolean  & bIsDegenerate)    // out: TRUE = intersection point is not part of an intersection curve
                                 //             due to tangent intersections or boundary effects
                                 //      FALSE= Point is part of an intersection curve
{
  // init output
  bIsDegenerate = FALSE ;

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  // draw Surface0(blue), Surface1(green), XSectPoint(Red)
  if(bDebugMe)
    {
      SmFace *pFace = (SmFace *)m_cpSurface[0]->GetFace() ;
      SmBrep *pBrep = pFace ? pFace->GetBrep() : NULL ;

      smgfx_Erase() ;
      smgfx_SetLook(.5,2, 1,1,1) ;  if(pBrep) { pBrep->Draw(TRUE) ; } sm_GraphicsLoop() ;
      smgfx_SetLook(1,3,  0,0,1) ;  m_cpSurface[0]->DrawUV(6,6,FALSE,NULL,pBrep==NULL) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,3,  0,1,0) ;  m_cpSurface[1]->DrawUV(6,6,FALSE,NULL,pBrep==NULL) ; sm_GraphicsLoop() ;
      smgfx_SetLook(3,5,  1,0,0) ;  rTSectPnt.CrvPos().Draw() ; sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

  // TangentPoints are always degenerate
  if(rTSectPnt.m_ePointType == SM_IP_TANGENT_POINT)
    { 
      bIsDegenerate = TRUE ;
      return(SM_SUCCESS) ;
    }

  // Classify small Pos and Neg steps from xSectPoint in XSectCurve direction
  SmPoint2d sPDelta[2] ;  // small positive step for both surfaces
  SmPoint2d sNDelta[2] ;  // small negative step for both surfaces
  SmBoolean bPInside, bNInside ;

  sPDelta[0] = rTSectPnt.UVPos(0) + SM_EFF_ZERO_SQRT * rTSectPnt.UVDeriv(0) ;
  sPDelta[1] = rTSectPnt.UVPos(1) + SM_EFF_ZERO_SQRT * rTSectPnt.UVDeriv(1) ;
  bPInside   = (   m_vUVDomain[0].ContainsPoint2d(sPDelta[0],10.0 * SM_EFF_ZERO)
                && m_vUVDomain[1].ContainsPoint2d(sPDelta[1],10.0 * SM_EFF_ZERO)) ;
  
  sNDelta[0] = rTSectPnt.UVPos(0) - SM_EFF_ZERO_SQRT * rTSectPnt.UVDeriv(0) ;
  sNDelta[1] = rTSectPnt.UVPos(1) - SM_EFF_ZERO_SQRT * rTSectPnt.UVDeriv(1) ;
  bNInside   = (   m_vUVDomain[0].ContainsPoint2d(sNDelta[0],10.0 * SM_EFF_ZERO)
                && m_vUVDomain[1].ContainsPoint2d(sNDelta[1],10.0 * SM_EFF_ZERO)) ;

  // when small pos or neg step from xSectPoint in XSect Curve direction are outside Surf1 || Surf2     
  if (!bPInside && !bNInside)
    {
      // no intersection curve is possible
      bIsDegenerate = TRUE ;
      return(SM_SUCCESS) ;

    } // end is small step in domain check

  // When XSectType is SM_IP_TANGENT_CURVE or SM_IP_SINGULARITY 
  // no more checks are needed there will be an intersection curve.
  //  This conclusion based on testing already done in ComputePointValues().
  if(   rTSectPnt.m_ePointType == SM_IP_TANGENT_CURVE
     || rTSectPnt.m_ePointType == SM_IP_SINGULARITY
     || rTSectPnt.m_ePointType == SM_IP_COINCIDENCE)  
    { 
      bIsDegenerate = FALSE ;
      return(SM_SUCCESS) ;
    }

  // arrive here for additional checks on SM_IP_CROSSING points
  SM_ASSERT(rTSectPnt.m_ePointType == SM_IP_CROSSING) 

  double     dAngleTol  = SM_DEG2RAD(2.0) ;
  SmVector3d sXSectGap  = rTSectPnt.SrfPos(1) - rTSectPnt.SrfPos(0) ;
  SmBoolean  bTangentU[2] ; bTangentU[0] = FALSE ; bTangentU[1] = FALSE ;
  SmBoolean  bTangentV[2] ; bTangentV[0] = FALSE ; bTangentV[1] = FALSE ;
  SmBoolean  bBoundaryU[2] ; bBoundaryU[0] = FALSE ; bBoundaryU[1] = FALSE ;
  SmBoolean  bBoundaryV[2] ; bBoundaryV[0] = FALSE ; bBoundaryV[1] = FALSE ;
  SmVector3d sBiNormU[2] ;
  SmVector3d sBiNormV[2] ;

  // for both SurfacePoints - find and test tangent BoundaryCurve/Surface intersections
  ULONG lSrf ;
  for(lSrf=0;lSrf<2;lSrf++)
    {
      // current surf index = lSrf
      // other   surf index = 1 - lSrf
      SmPoint2d  sUV        = rTSectPnt.UVPos(lSrf) ;
      SmExtent2d sDomain    = m_vUVDomain[lSrf] ;
      SmVector3d sOtherNorm = rTSectPnt.SrfNorm(1-lSrf) ;
      SmVector3d sTangentU  = rTSectPnt.SrfDu(lSrf) ;
      SmVector3d sTangentV  = rTSectPnt.SrfDv(lSrf) ;

      // See if sUV is on a natural boundary tangent to OtherSurface
      if     (SM_ARE_SAME(sDomain.GetMin().x,sUV.x)) { bBoundaryU[lSrf] =  TRUE ;
                                                       bTangentU[lSrf]  =  sOtherNorm.IsPerpendicularTo(sTangentV, dAngleTol) ;
                                                       sBiNormU[lSrf]   =  sTangentU ;
                                                     }
      else if(SM_ARE_SAME(sDomain.GetMax().x,sUV.x)) { bBoundaryU[lSrf] =  TRUE ;
                                                       bTangentU[lSrf]  =  sOtherNorm.IsPerpendicularTo(sTangentV, dAngleTol) ;
                                                       sBiNormU[lSrf]   = -sTangentU ;
                                                     }
      if     (SM_ARE_SAME(sDomain.GetMin().y,sUV.y)) { bBoundaryV[lSrf] =  TRUE ;
                                                       bTangentV[lSrf]  =  sOtherNorm.IsPerpendicularTo(sTangentU, dAngleTol) ;
                                                       sBiNormV[lSrf]   =  sTangentV ;                           
                                                     }                                                                        
      else if(SM_ARE_SAME(sDomain.GetMax().y,sUV.y)) { bBoundaryV[lSrf] =  TRUE ;
                                                       bTangentV[lSrf]  =  sOtherNorm.IsPerpendicularTo(sTangentU, dAngleTol) ;
                                                       sBiNormV[lSrf]   = -sTangentV ;
                                                     }
    } // end iter both surfaces making tangent BoundaryCurve/Surface intersection check

  // When sUV is NOT on a tangent Natural Boundary
  SmBoolean bOnTangentNaturalBoundary =   (bBoundaryU[0] && bTangentU[0])   
                                       || (bBoundaryV[0] && bTangentV[0])
                                       || (bBoundaryU[1] && bTangentU[1])   
                                       || (bBoundaryV[1] && bTangentV[1]) ;
  if(!bOnTangentNaturalBoundary)
    {
      // Intersection Curve will not be degenerate
      bIsDegenerate = FALSE ;
      return(SM_SUCCESS) ;
    }

#ifdef SM_DEBUG_CODE

  // points close to being singular (the two surf normals are nearly parallel)
  // may have all the bTangent flags set, in which case the XSectCurve UVTangent
  // may be in any direction.
  SmBoolean bTangent0U = rTSectPnt.SrfNorm(1).IsPerpendicularTo( rTSectPnt.SrfDu(0), dAngleTol);
  SmBoolean bTangent0V = rTSectPnt.SrfNorm(1).IsPerpendicularTo( rTSectPnt.SrfDv(0), dAngleTol);
  SmBoolean bTangent1U = rTSectPnt.SrfNorm(0).IsPerpendicularTo( rTSectPnt.SrfDu(1), dAngleTol);
  SmBoolean bTangent1V = rTSectPnt.SrfNorm(0).IsPerpendicularTo( rTSectPnt.SrfDv(1), dAngleTol);

  SmBoolean bNearSingularity =    (bTangent0U && bTangent0V)
                               || (bTangent1U && bTangent1V) ;

  // when xSectPoint is near a singularity - all UVDirections should be nearly perp to the surface normals
  SM_ASSERT(   !bNearSingularity
            || (   (bTangent0U && bTangent0V)
                && (bTangent1U && bTangent1V))) ;

  // When a NaturalBoundaryCurve is tangent to the OtherSurface
  // the xSectCurve tangent direction should lie along the Natural boundary.
  SM_ASSERT(   bNearSingularity 
            || (   !bTangentU[0] 
                || (smos_Fabs(rTSectPnt.UVDeriv(0).y) > 100.0 * smos_Fabs(rTSectPnt.UVDeriv(0).x)))) ;
  SM_ASSERT(   bNearSingularity 
            || (   !bTangentV[0] 
                || (smos_Fabs(rTSectPnt.UVDeriv(0).x) > 100.0 * smos_Fabs(rTSectPnt.UVDeriv(0).y)))) ;
  SM_ASSERT(   bNearSingularity 
            || (   !bTangentU[1] 
                || (smos_Fabs(rTSectPnt.UVDeriv(1).y) > 100.0 * smos_Fabs(rTSectPnt.UVDeriv(1).x)))) ;
  SM_ASSERT(   bNearSingularity 
            || (   !bTangentV[1] 
                || (smos_Fabs(rTSectPnt.UVDeriv(1).x) > 100.0 * smos_Fabs(rTSectPnt.UVDeriv(1).y)))) ;

  // only one boundary can be tangent at a time unless we are near a Singularity
  SM_ASSERT(   bNearSingularity 
            || (!bTangentU[0] && !bTangentV[0])
            || ( bTangentU[0] !=  bTangentV[0])) ; 
  SM_ASSERT(   bNearSingularity 
            || (!bTangentU[1] && !bTangentV[1])
            || ( bTangentU[1] !=  bTangentV[1])) ; 

  // draw Surface0(blue), Surface1(green), XSectPoint(Red)
  if(bDebugMe)
    {
      SmFace *pFace = (SmFace *)m_cpSurface[0]->GetFace() ;
      SmBrep *pBrep = pFace ? pFace->GetBrep() : NULL ;
      //double dUVTangent0dX = rTSectPnt.UVDeriv(0).x ;
      //double dUVTangent0dY = rTSectPnt.UVDeriv(0).y ;
      //double dUVTangent1dX = rTSectPnt.UVDeriv(1).x ;
      //double dUVTangent1dY = rTSectPnt.UVDeriv(1).y ;

      smgfx_Erase() ;
      smgfx_SetLook(1,2,  1,1,1)  ;  if(pBrep) { pBrep->Draw(TRUE) ; } sm_GraphicsLoop() ;
      smgfx_SetLook(1,3,  0,0,1)  ;  m_cpSurface[0]->DrawUV(6,6,FALSE,NULL,pBrep==NULL) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,3,  0,1,0)  ;  m_cpSurface[1]->DrawUV(6,6,FALSE,NULL,pBrep==NULL) ; sm_GraphicsLoop() ;
      smgfx_SetLook(3,5,  1,0,0)  ;  rTSectPnt.CrvPos().Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook(3,5,  1,0,0)  ;  rTSectPnt.CrvDeriv().Draw(&rTSectPnt.CrvPos()) ; sm_GraphicsLoop() ;
      smgfx_SetLook(2,3,  0,1,.5) ;  rTSectPnt.SrfDv(0).Draw(&rTSectPnt.SrfPos(0)) ; sm_GraphicsLoop() ;
      smgfx_SetLook(2,3,  0,1,1)  ;  rTSectPnt.SrfDu(0).Draw(&rTSectPnt.SrfPos(0)) ; sm_GraphicsLoop() ;
      smgfx_SetLook(3,4,  1,0,.5) ;  rTSectPnt.SrfDv(1).Draw(&rTSectPnt.SrfPos(1)) ; sm_GraphicsLoop() ;
      smgfx_SetLook(3,4,  1,0,1)  ;  rTSectPnt.SrfDu(1).Draw(&rTSectPnt.SrfPos(1)) ; sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

  // arrive here - test gap function shape to determine if XSectCurve is degenerate
  // A XSectPoint is degenerate when
  //   the gap function on both sides of the XSect point
  //   diverges monotonically from the XSectPoint to either
  //      the end of the domain 
  //   or until the gap size gets to be larger than EFF_ZERO
  // When an XSectPoint happens to be on a boundary corner - only check
  //   for divergence on one side.

  // locals - gap function properties 
  SmPoint3d  sCurvePoint[3], sOSurfPoint[3], sGap[3] ;
  double     sGapLength [3], dSampleDist=0.0 ;
  SmBoolean  sbInside   [3] ;
  double bScaledZero = SM_EFF_ZERO * (1 + rTSectPnt.CrvPos().GetMaxDimension()) ;

  // for both SurfacePoints - find and test tangent BoundaryCurve/Surface intersections
  for(lSrf=0;lSrf<2;lSrf++)
    {
      // When sUV is NOT on a tangent Natural Boundary
      if(!bTangentU[lSrf] && !bTangentV[lSrf])
        {
          // this may have an intersection curve 
          // unless OtherSurface xSect point happens to be a nonIntersecting tangent boundary.
          // So check other surface
          continue ;
        }

      // Sample the NaturalBoundary/OtherSurface gap function along the curveTangentDirection
      SmVector3d sBiNorm, sTangent ;
      SmVector3d sOSurfPV[2][2] ;
      if(bTangentU[lSrf]) { sBiNorm  = sBiNormU[lSrf] ;
                            sTangent = rTSectPnt.SrfDv(lSrf) ;
                          }
      else                { sBiNorm  = sBiNormV[lSrf] ;
                            sTangent = rTSectPnt.SrfDu(lSrf) ;
                          }

      double dStride = m_dThisApproxTol3d/sTangent.Length()/2.0;

      // In practice, this does not have to be so small, and it saves
      // several iterations each time if it is bigger.  [CC_AK300]
      dStride *= 100.0;

      // test Gap Function on both sides of the XSectPoint
      SmBoolean bIsSideDegenerate[2] ;  bIsSideDegenerate[0] = FALSE ; bIsSideDegenerate[1] = FALSE ;
      for(ULONG lSide=0; lSide<2; lSide++)
        {
          // init side classification booleans
          SmBoolean bZero = TRUE ;
          SmBoolean bMono = TRUE ; 
          SmBoolean bDiv  = TRUE ;
          SmBoolean bOpen = TRUE ;

          // skip directions which take us out of the surface domains
          if(   (lSide == 0 && bPInside == FALSE)
             || (lSide == 1 && bNInside == FALSE))
            {
              bIsSideDegenerate[lSide] = TRUE ;
              continue ;
            }
              
          // init the SamplePoint arrays with the xSectPoint     
          sCurvePoint[0] = rTSectPnt.SrfPos(lSrf) ;  
          sOSurfPoint[0] = rTSectPnt.SrfPos(1-lSrf) ;
          sGap[0].Set(0.0, 0.0, 0.0) ;     
          sGapLength[0]  = 0.0 ;                    
          sbInside[0]    = TRUE ;                                

          // walk Gap Function 1st in positive direction then the negative
          double delta = lSide == 0 ? dStride : -dStride ;

          // Note: Possible enhancement: instead of using an iterative
          // step-off method, it would be more efficient to utilize
          // 2nd-order (curvature) quantities at the intersection point.

          // until this side is classified
          SmBoolean bDone = FALSE ;
          ULONG ii;
          for (ii=1;!bDone;ii++,delta *= 5.0)  // (Was 2.0, but bigger works well and saves iters.)
            {
              // get the gapFunction sample array index
              ULONG tgt = ii > 1 ? 2 : 1 ; // gwcgeometric growth change; ii % 3 ;

              // make sure that the classification is done in just
              // a few steps - study any cases that violate the following
              // and make appropriate changes
              // (Changed from 10 to 15 because we no longer do the Monotonic
              // or Diverging tests until the gap is non-zero.)
              SM_ASSERT(ii<15) ;

              // init output
              sbInside[tgt] = FALSE ;

              // get the samplePoint Surface UVValues
              SmPoint2d sUV0, sUV1, sUVDrop ;
              sUV0 = rTSectPnt.UVPos(lSrf)   + delta * rTSectPnt.UVDeriv(lSrf) ;
              sUV1 = rTSectPnt.UVPos(1-lSrf) + delta * rTSectPnt.UVDeriv(1-lSrf) ;

              // When SamplePoint is in both domains
              if(   m_vUVDomain[lSrf]  .ContainsPoint2d(sUV0,10.0 * SM_EFF_ZERO)
                 && m_vUVDomain[1-lSrf].ContainsPoint2d(sUV1,10.0 * SM_EFF_ZERO))
                { 
                  // evaluate the thisSurface naturalBoundary 3d Point
                  m_cpSurface[lSrf]->EvaluatePoint(sUV0, sCurvePoint[tgt]) ;

                  // get originPoint/currentPoint distance
                  dSampleDist = (rTSectPnt.SrfPos(1) - sCurvePoint[tgt]).Length() ;

                  // Find InDomain OtherSurface nearest point to sCurvePoint
                  // note: for small gaps (which is this function's operation point)
                  //       SM_SO_MINIMIZE will snap to boundaries even when
                  //       the project point is just outside the domain
                  SmBoolean bFoundAnswer = FALSE ;
                  SmSolution sSolution ;
                  m_cpSurface[1-lSrf]->LocalPointSolve(m_vUVDomain[1-lSrf], SM_SO_NORMALIZE, 
                                                       sCurvePoint[tgt], sUV1, 
                                                       bFoundAnswer, sSolution) ;
                  // when a solution was found
                  if(bFoundAnswer)
                    {
                      // evaluate the nearest OtherSurface point
                      sUVDrop.Set(sSolution.m_vStart.m_adParameters[0],
                                  sSolution.m_vStart.m_adParameters[1]) ;
                      m_cpSurface[1-lSrf]->Evaluate(sUVDrop, 1, 1, TRUE, TRUE, TRUE, sOSurfPV[0]) ;

                      // get the BoundaryCurve/Surface gap vector
                      //   little trick: Subtract the sXSectGap from every gap SamplePoint 
                      //   to keep tolerance problems from creating 
                      //   oppositely directed gap vectors
                      sOSurfPoint[tgt] = sOSurfPV[0][0] ;
                      sGap[tgt]        =   (sOSurfPoint[tgt] - sCurvePoint[tgt]) 
                                        + ((lSrf == 0) ? -sXSectGap : sXSectGap) ;
                      sGapLength[tgt]  = sGap[tgt].Length() ;
                      sbInside[tgt]    = TRUE ;

#ifdef SM_DEBUG_CODE
                      // draw ThisSurfacePoint and OtherSurfacePoint
                      if(bDebugMe)
                        {
                          smgfx_SetLook(4,6, 1,0,1) ; sCurvePoint[tgt].Draw() ; sm_GraphicsLoop() ;
                          smgfx_SetLook(4,6, 1,1,0) ; sOSurfPoint[tgt].Draw() ; sm_GraphicsLoop() ;
                          sm_GraphicsLoop() ;
                        }
#endif // SM_DEBUG_CODE

                    } // end ThisSurfacePoint dropped to otherSurface check
                } // end small step inside domain check

              // if we've stepped off the domain
              if(sbInside[tgt] == FALSE)
                {
                  bDone = TRUE ;

                  // decide if short line is degenerate or not
                  // this is a heuristic that can be tuned as needed
                  //   1. too short to sample - degenerate
                  //   2. continuously diverging through the last sample point while shorter than 10*approxTol - degnerate
                  //   3. else non degenerate
                  bIsSideDegenerate[lSide] =  (   ii < 3                                                  // too short to sample
                                               || (   bMono && bDiv && sBiNorm.Dot(sGap[((tgt+3)-1)%3]) > 0.0 // diverging short line
                                                   && dSampleDist < m_dThisApproxTol3d*10.0))
                                             ? TRUE
                                             : FALSE ;
                } // end stepped off the domain check

              // try to classify the side after getting two sample points
              else if(ii >= 2)
                {
                  ULONG tgt1 = ((tgt+3)-1) % 3 ;
                  ULONG tgt2 = ((tgt+3)-2) % 3 ;

                  // 1st: is xSectCurve gapSize zero (coincident curves)
                  if(sGapLength[tgt] > bScaledZero)
                   { bZero = FALSE ; }
                  
                  // 2nd: is xSectCurve gap function open -
                  //      we need to make sure that the current gap
                  //      does not represent an internal intersection point.
                  //      To test this to 1st order approximation, 
                  //      Find an intersection point
                  //      closest to the sUV point on the two planes,
                  //      (sUV,biNorm,Tangent) and 
                  //      (DropPoint,OtherSurf->TangentU,OtherSurf->TangentV).
                  //      If that point is off either surface - no internal intersection point 
                  //
                  //      min ((P-C)**2 - Lambda(Dot(BXT,P-C)) where
                  //        P         = unknown point = S + du*Su + dv*Sv
                  //        C         = ThisSurf->sCurvePoint[tgt]
                  //        S, Su, Sv = drop point and 1st derivatives on OtherSurf
                  //        BXT       = cross(ThisSurf->BiNorm, ThisSurf->Tangent)
                  //
                  //      Solve: [ 2*Su*Su  2*Su*Sv  -(BXT)*Su][  du  ]   [ -2(S-C)*Su ]
                  //             [ 2*Su*Sv  2*Sv*Sv  -(BXT)*Sv][  dv  ] = [ -2(S-C)*Sv ]
                  //             [ (BXT)*Su (BXT)*Sv  0       ][lambda] = [-(BXT)*(S-C)]
                  SmVector3d sBXT = sBiNorm * sTangent ;
                  SmVector3d sSC  = sOSurfPV[0][0] - sCurvePoint[tgt] ;
                  double dBData[3], dXData[3] ;
                  SmTArray<double> sB(3,dBData,3) ;
                  SmTArray<double> sX(3,dXData,3) ;
                  SmMatrix sA(3,3) ;
                  sA.SetAt(0, 0,  2*sOSurfPV[1][0].Dot(sOSurfPV[1][0])) ;
                  sA.SetAt(0, 1,  2*sOSurfPV[1][0].Dot(sOSurfPV[0][1])) ;
                  sA.SetAt(0, 2, -sBXT.Dot(sOSurfPV[1][0])) ;

                  sA.SetAt(1, 0,  2*sOSurfPV[0][1].Dot(sOSurfPV[1][0])) ;
                  sA.SetAt(1, 1,  2*sOSurfPV[0][1].Dot(sOSurfPV[0][1])) ;
                  sA.SetAt(1, 2, -sBXT.Dot(sOSurfPV[0][1])) ;

                  sA.SetAt(2, 0,  sBXT.Dot(sOSurfPV[1][0])) ;
                  sA.SetAt(2, 1,  sBXT.Dot(sOSurfPV[0][1])) ;
                  sA.SetAt(2, 2,  0.0) ;

                  sB.SetAt(0, -2*sSC.Dot(sOSurfPV[1][0])) ;
                  sB.SetAt(1, -2*sSC.Dot(sOSurfPV[0][1])) ; 
                  sB.SetAt(2, -sBXT.Dot(sSC)) ;

                  // solve: solver fails when two planes are parallel
                  SmStatus sRtn = sA.SolveLinearSystem(sB, sX) ;
                  if(SM_SUCCESS != sRtn) { // open for nonZero Gaps
                                           bOpen = TRUE ;
                                         }
                  else
                    {
                      // get Point P OtherSurfaceUV and (P-C) 3d values
                      SmPoint2d sPUV(sUVDrop.x + sX[0], 
                                     sUVDrop.y + sX[1]) ;
                      SmVector3d sPC =   sOSurfPV[0][0] 
                                       + sX[0] * sOSurfPV[1][0]
                                       + sX[1] * sOSurfPV[0][1] 
                                       - sCurvePoint[tgt] ;
                      double dPCDotBiNorm = sPC.Dot(sBiNorm) ;

                      // Open when solution point is not in either surface
                      bOpen =  (   !m_vUVDomain[1-lSrf].ContainsPoint2d(sPUV, SM_EFF_ZERO)
                                || dPCDotBiNorm < -SM_EFF_ZERO)
                             ? TRUE
                             : FALSE ;
                                
                    } // end bOpen check based on 1st order intersection pt approximation. 

                  // The next two checks should not be tested until we have
                  // meaningful quantities: when the gap is essentially zero,
                  // they are just random noise.  [CC_AK300]
                  if ( ! bZero )
                    {
                      // 3rd: is xSectCurve gap function monotonic 
                      if(sGap[tgt].Dot(sGap[tgt1]) <= 0.0) 
                        { bMono = FALSE ; }

                      // 4th: is xSectCurve gap function diverging (growing faster than linear pace)
                      if(   sGapLength[tgt] == 0.0  
                         || sGapLength[tgt] < sGapLength[tgt1] + 1.0001 * (sGapLength[tgt1] - sGapLength[tgt2])) 
                        { bDiv = FALSE ; }
                    }

                  // now classify this side of the XSectCurve.
                  // The XSectCurve on this side of the XSectPoint is degenerate when
                  //   has a nonZero OutsideThisSurface Gap that is monotonically diverging
                  if(!bZero && bMono && bDiv && bOpen)
                    {
                      bIsSideDegenerate[lSide] = TRUE ;
                      bDone = TRUE ;
                    }
                  else if(bZero && bMono && bDiv)
                    {
                      // try another point
                      bDone = FALSE ;

                      // move current guess to last guess
                      sCurvePoint[1] = sCurvePoint[2] ;
                      sOSurfPoint[1] = sOSurfPoint[2] ;
                      sGap[1]        = sGap[2] ;      
                      sGapLength[1]  = sGapLength[2] ;
                      sbInside[1]    = sbInside[2] ;   
                    }
                  else // gapFunction is no longer diverging monotonically outsideThisSurface
                    {
                      // this side of the curve can have an intersection
                      bIsSideDegenerate[lSide] = FALSE ;
                      bDone = TRUE ;
                    }

                } // end enough samples to test side check
            } // end while this side is not yet classified
        } // end iter both sides of XSectPoint test GapFunction
      
      // degenerate intersection if both sides of XSectPoint are degenerate
      if(bIsSideDegenerate[0] && bIsSideDegenerate[1])
        {
          bIsDegenerate = TRUE ;
          return(SM_SUCCESS) ;
        }

    } // end iter both surfaces - lSrf

  // arrive here when crossing XSectPoint is NOT a degenerate curve intersection
  // e.g. it is part of an actual intersection curve
  bIsDegenerate = FALSE ;
  return(SM_SUCCESS) ;

} // end SmSurfaceIntersector::ClassifyBoundaryStartPointIntersection 

/*******************************************************************//**
PURPOSE:
NOTES:
***********************************************************************/
SmBoolean SmSurfaceIntersector::IsKindOf( SM_TYPE t ) const
{
  return ((SmSurfaceIntersector_TYPE == t) ? TRUE : SmGlobalSolver::IsKindOf( (t) ));
}

/*******************************************************************//**
PURPOSE: Pretty print   

NOTES: 
***********************************************************************/
void SmSurfaceIntersector::Dump() const
{
  // TCHAR sBuff[SM_TBLOCK_SIZE];

  // start
  smos_WriteBuffer(_T("\nBegin SmSurfaceIntersector::Dump()")) ;

  // dump base
  SmGlobalSolver::Dump() ;

  // pretty print values

  // end
  smos_WriteBuffer(_T("\nEnd SmSurfaceIntersector::Dump()\n")) ;

} // end SmSurfaceIntersector::Dump

/*******************************************************************//**
PURPOSE: See if a point proposed for a degenerate edge is
  duplicated in the m_vStartPoints list or is already used in
  one of the accumulated output points

NOTES:
***********************************************************************/
SmBoolean sm_IsPointUnique
  (SmVector3d          &rPoint,       // in : proposed degenerate point
   SmTList<SmTsectPnt> &rStartPoints, // in : list of remaining start points to search
   SmTArray<SmCurve*>  &r3DCurves,    // in : list of 3DCurves to check
   double              d3DTol)        // in : min dist between unique 3d points
{
  // check m_vStartPoints for duplicates
  SmTsectPnt *pFirst   = NULL ;
  SmTsectPnt *pCurrent = rStartPoints.GetFirstNode() ;
  for(;pCurrent && pCurrent != pFirst;)
    {
      // report duplicate nodes
      if(rPoint.DistanceBetween(pCurrent->CrvPos()) < d3DTol)
        { return(FALSE) ; }

      // set next iteration
      pFirst   = rStartPoints.GetFirstNode() ;
      pCurrent = rStartPoints.GetNextNode(pCurrent) ;

    } // end iter every StartPoint

  // check intersectionCurve start/end points for duplicates
  for(ULONG ii=0;ii<r3DCurves.GetSize();ii++)
    {
      SmCurve *pCurve = r3DCurves[ii] ;
      SmExtent1d sIvl = pCurve->GetNaturalInterval() ;
      SmVector3d sPoint ;

      // report duplicate startPoints
      pCurve->EvaluatePoint(sIvl.GetMin(), sPoint) ;
      if(rPoint.DistanceBetween(sPoint)< d3DTol)
        { return(FALSE) ; }

      // report duplicate endPoints
      pCurve->EvaluatePoint(sIvl.GetMax(), sPoint) ;
      if(rPoint.DistanceBetween(sPoint)< d3DTol)
        { return(FALSE) ; }

    } // end iter every 3DCurve

  // arrive here when point is unique
  return(TRUE) ;

} // end sm_IsPointUnique
