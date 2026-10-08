// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************//**
* FILE NAME --- SmSurfaceDropLib.cpp
* PURPOSE: This file contains much of the functionality for dropping curves to surfaces.
**********************************************************************/

#include "StdAfx.h"

#include <SmGraphicsOutput.h>
#include <SmTArray.h>
#include <SmCurveCache.h>
#include <SmLocalSolve1d.h>
#include <SmLocalSolveNd.h>
#include <SmGeomUtility.h>
#include <SmGlobalSolver.h>
#include <SmCompositeCurve.h>
#include <SmGauss.h>
#include <SmHermiteCurve.h>
#include <SmCrvOnSurf.h>
#include <SmPlane.h>
#include <SmOffsetSurface.h>
#include <SmAssertArray.h>
#include <SmSurfaceDropCurve.h>

#ifdef SM_DEBUG_CODE
#include <SmBrep.h>
#include <SmFace.h>
#include <SmEdge.h>
#endif // SM_DEBUG_CODE

// **********************************************************************
// 
//   Local static routines for DropCurve().
// 
// **********************************************************************

/*******************************************************************//**
PURPOSE: Standardize DropCurve() Debug drawing blocks

NOTES: Asserts and Dumps p3dCurve and p3dSurface
                Draws Brep, Surface, Curve
***********************************************************************/
void my_AssertAndDrawDropCurve
 (const SmCurve    * p3dCurve,
  SmExtent1d       & rInterval,
  const SmSurface  * p3dSurface)
{
  SmEdge *pEdge = (SmEdge *)p3dCurve->GetEdge() ;
  SmFace *pFace = (SmFace *)p3dSurface->GetFace() ;
  SmBrep *pBrep =   pFace ? pFace->GetBrep() : pEdge ? pEdge->GetBrep() : NULL ;
                              
  SM_DUMP_AND_ASSERT2_VALID(p3dSurface) ;
  SM_DUMP_AND_ASSERT2_VALID(p3dCurve) ;

  if(p3dCurve == NULL)
    { smgfx_Erase() ; }
  smgfx_SetLook(1,3, 0,0,1) ; if (pBrep) { pBrep->Draw(TRUE) ; } sm_GraphicsLoop() ;

  smgfx_SetLook(1,2, 0,1,1);     p3dSurface->DrawUV(4,4,FALSE,NULL,TRUE); sm_GraphicsLoop() ;
  smgfx_SetLook(4,5, .7,.7,.7) ; p3dSurface->DrawSeams() ; sm_GraphicsLoop() ;
  smgfx_SetLook(6,7, .7,.7,.7) ; p3dSurface->DrawPoles() ; sm_GraphicsLoop() ;
  smgfx_SetLook(3,4, 1,0,0) ;    p3dSurface->DrawParams() ; sm_GraphicsLoop() ;
  smgfx_SetLook(1,2, 0,0,0) ; if (pFace) { pFace->Draw(SM_DM_CROSSHATCH) ; } sm_GraphicsLoop() ;

  smgfx_SetLook(5,6, 1,0,0) ; p3dCurve->Draw(&rInterval, TRUE) ; sm_GraphicsLoop() ;
  smgfx_SetLook(1,2, 0,1,0) ; p3dCurve->DrawSpeed( -1.0, 35) ; sm_GraphicsLoop() ;
  smgfx_SetLook(7,8, 1,0,0) ; p3dCurve->DrawParams( &rInterval ) ; sm_GraphicsLoop() ;
  sm_GraphicsLoop() ;

} // end my_AssertAndDrawDropCurve

/*******************************************************************//**
PURPOSE: Convenience Class containing all information about a 
  Curve Point dropped to a Surface Point 

NOTES: Helper class for SmSurface::DropCurve
***********************************************************************/
class SmDropPt
{
 public:
  double     m_dT ;                  // Parm of CurvePt dropped to Surface
  SmBoolean  m_bGoodDrop ;           // TRUE = CurvPt within dApprox dist of SurfNormalLine at DropPoint
                                         
  SmPoint3d  m_CrvPD[3] ;            // Crv point, 1stDeriv, and 2ndDeriv for Curve(dT)
  SmPoint2d  m_UVPD[2] ;             // Srf UV Drop Pt and 1stDeriv from GlobalPtSolve(MINIMIZE)
  SmPoint3d  m_3dPD[2] ;             // Srf 3dPt and 1stDeriv for Srf UVPt[0]
  SmVector3d m_SrfNormal ;           // unit Surface Normal vector at DropPoint
  SmBoolean  m_bFromLeftU ;          // TRUE = When DropPt falls on a srf U bndry, Eval from left
  SmBoolean  m_bFromLeftV ;          // TRUE = When DropPt falls on a srf V bndry, Eval from left
  double     m_dDropToSurf ;         // Distance from CurvePt to Srf DropPt
  double     m_dApproxDev ;          // Distance from CurvePt to SrfNormalLine at DropPt
                                     //   Will be near 0.0 when CurvePt is over the Surface
  double     m_dNormalToCrvAngDeg ;  // Crv1stDir to SrfPlane Normal And - should be 90.0
  double     m_dSrfToCrvAngDeg ;     // Crv1stDir dropped to SrfPlane to UV1stDir proj through surface Ang - should be 0.0
  double     m_dSrfToCrvSpeedRatio ; // Crv1stDir dropped to SrfPlane to UV1stDir proj through surface Speed Ratio- should be 1.0
  SmBoolean  m_bOnBoundary ;         // TRUE = DropPt within SM_EFF_ZERO of Domain boundary
  SmBoolean  m_bOnSeam ;             // TRUE = when m_bOnBoundary, TRUE when that boundary is a seam
  SmBoolean  m_bOnPole ;             // TRUE = when m_bOnBoundary, TRUE when that boundary is a pole
  SmBoolean  m_bDegenCurve ;         // TRUE = Curve being dropped is degenerate, i.e a point (often dropped to singularities to complete loops)
                                     //        note: DegenCurves being dropped as part of a degenerate edge is a Brep representation violation
                                     //              being supported here to support reading in problem databases prior to their being healed.
  SmBoolean  m_bLeavingDomain ;      // TRUE = DropCrv through this DropPt is exiting the surface domain at this point
  SmBoolean  m_bEnteringDomain ;     // TRUE = through this DropPt is entering the surface domain at this point
                                     //  leaving/entering Note: bndries are currently considered crossed when the 3d angle 
                                     //        between DropCrv Tangent and the Surface NatTrimBndry measured on the surfance tangent
                                     //        plane is greater than dLimitRad (currently 30 degrees) in SmDropPt::SetProperties(). 
                                     //        It may be that a more subtle definition is required.
  SmStatus   m_eDropDerivRtn ;       // SM_SUCCESS = 1st Deriv dropped with tight tolerances, else didn't

  // default SmDropPt constructor
  SmDropPt(double    dT,
           SmBoolean bDegenCurve)                { SetUninitialized() ;
                                                   m_dT = dT ;
                                                   m_bDegenCurve = bDegenCurve ;
                                                 }

  // gap from CurvePt to Srf DropPt
  SmGapSample GetCrvSrfGap(SmGapSample &rCrvSrfGap) { if(m_bGoodDrop == UNSURE)
                                                        rCrvSrfGap.SetGapDropType(SM_GD_NO_DROP) ;
                                                      else
                                                        { rCrvSrfGap.SetGapDropType( m_bOnBoundary ? SM_GD_BOUNDARY
                                                                                                   : SM_GD_NORMAL) ;       
                                                          rCrvSrfGap.SetThisPos(m_CrvPD[0]) ;
                                                          rCrvSrfGap.SetThisParam(m_dT) ;
                                                          rCrvSrfGap.SetOtherPos(m_3dPD[0]) ;
                                                          rCrvSrfGap.SetOtherParam(m_UVPD[0].x,m_UVPD[0].y) ;
                                                        } 
                                                      return( rCrvSrfGap );
                                                    }

  // SmDropPt assignment operator
  SmDropPt & operator=(const SmDropPt &crDropPt) { if(this == &crDropPt) return *this ;
                                                   m_bGoodDrop           = crDropPt.m_bGoodDrop ;
                                                   m_dT                  = crDropPt.m_dT ;
                                                   m_CrvPD[0]            = crDropPt.m_CrvPD[0] ;
                                                   m_CrvPD[1]            = crDropPt.m_CrvPD[1] ;
                                                   m_CrvPD[2]            = crDropPt.m_CrvPD[2] ;
                                                   m_UVPD[0]             = crDropPt.m_UVPD[0] ;
                                                   m_UVPD[1]             = crDropPt.m_UVPD[1] ;
                                                   m_3dPD[0]             = crDropPt.m_3dPD[0] ;
                                                   m_3dPD[1]             = crDropPt.m_3dPD[1] ;
                                                   m_SrfNormal           = crDropPt.m_SrfNormal ;
                                                   m_bFromLeftU          = crDropPt.m_bFromLeftU ;
                                                   m_bFromLeftV          = crDropPt.m_bFromLeftV ;
                                                   m_dDropToSurf         = crDropPt.m_dDropToSurf ;
                                                   m_dApproxDev          = crDropPt.m_dApproxDev ;
                                                   m_dNormalToCrvAngDeg  = crDropPt.m_dNormalToCrvAngDeg ;
                                                   m_dSrfToCrvAngDeg     = crDropPt.m_dSrfToCrvAngDeg ;   
                                                   m_dSrfToCrvSpeedRatio = crDropPt.m_dSrfToCrvSpeedRatio ;
                                                   m_bOnBoundary         = crDropPt.m_bOnBoundary ;
                                                   m_bOnSeam             = crDropPt.m_bOnSeam ;
                                                   m_bOnPole             = crDropPt.m_bOnPole ;
                                                   m_bDegenCurve         = crDropPt.m_bDegenCurve ;
                                                   m_bLeavingDomain      = crDropPt.m_bLeavingDomain ;
                                                   m_bEnteringDomain     = crDropPt.m_bEnteringDomain ;
                                                   m_eDropDerivRtn       = crDropPt.m_eDropDerivRtn ;
  
                                                   return *this ;
                                                 }
  // default SmDropPt constructor
  void SetUninitialized()                        { m_dT                  = SM_UNDEF_DOUBLE ; 
                                                   m_bGoodDrop           = UNSURE ;
                                                   m_CrvPD[0].SetUninitialized() ; 
                                                   m_CrvPD[1].SetUninitialized() ;
                                                   m_CrvPD[2].SetUninitialized() ; 
                                                   m_UVPD[0].SetUninitialized() ;
                                                   m_UVPD[1].SetUninitialized() ;
                                                   m_3dPD[0].SetUninitialized() ;
                                                   m_3dPD[1].SetUninitialized() ;
                                                   m_SrfNormal.SetUninitialized() ;
                                                   m_bFromLeftU          = UNSURE ; 
                                                   m_bFromLeftV          = UNSURE ; 
                                                   m_dDropToSurf         = SM_UNDEF_DOUBLE ; 
                                                   m_dApproxDev          = SM_UNDEF_DOUBLE ;
                                                   m_dNormalToCrvAngDeg  = SM_UNDEF_DOUBLE ; 
                                                   m_dSrfToCrvAngDeg     = SM_UNDEF_DOUBLE ; 
                                                   m_dSrfToCrvSpeedRatio = SM_UNDEF_DOUBLE ;
                                                   m_bOnBoundary         = UNSURE ; 
                                                   m_bOnSeam             = UNSURE ;
                                                   m_bOnPole             = UNSURE ;
                                                   m_bDegenCurve         = UNSURE ;
                                                   m_bLeavingDomain      = UNSURE ; 
                                                   m_bEnteringDomain     = UNSURE ; 
                                                   m_eDropDerivRtn       = SM_BIG_ULONG ;
                                                 }

  // predicates

  // when CrvTangent drops to Surface within 3 deg and without a large speed change - return TRUE
  SmBoolean IsGoodTangentDrop(SmBoolean bOnPole) { return(  (bOnPole == FALSE) 
                                                          ? (// smos_Fabs(90.0 - m_dNormalToCrvAngDeg) < 3.0
                                                                m_dSrfToCrvAngDeg < 3.0 
                                                             && m_dSrfToCrvSpeedRatio < 2.0
                                                             && m_dSrfToCrvSpeedRatio > 0.5)
                                                          : (// smos_Fabs(90.0 - m_dNormalToCrvAngDeg) < 3.0
                                                                m_dSrfToCrvAngDeg < 15.0 
                                                             && m_dSrfToCrvSpeedRatio < 4.0
                                                             && m_dSrfToCrvSpeedRatio > 0.25)) ;
                                                 } 

  // returns SM_POC_INSIDE       = curve tangent is moving into surface from a boundary      
  //         SM_POC_OUTSIDE      = curve tangent is moving out of surface from a boundary    
  //         SM_POC_ON_BOUNDARY  = curve tangent is moving along boundary from a boundary    
  //         SM_POC_UNKNOWN      =    curve drop UV point is not on a surface domain boundary
  //                               or curve1stDeri or SrfBdry1stDeriv is zero
  SmPointObjectContainmentType IsMovingAlongBoundary // in : m_UVPD[0]  = DropPt SrfUV 2dPoint
                                                     //      m_CrvPD[2] = Curve Point and 1stDeriv
   (const SmSurface  & crSurface,                // in : tgt surface
    const SmExtent2d & crUVDomain,               // in : surface domain of interest
    double             dUVTolX,                  // in : UV Tolerance in the U direction
    double             dUVTolY,                  // in : UV Tolerance in the V direction
    double             dAngLimitDeg=5.0,         // in : max angle between CrvTang and SrfBndryTanget to be moving along boundary        
    SmSurfParamType    eTestBdry=SM_SP_BOTH)     // in : SM_SP_BOTH = test for pts on any bdry  
  const ;                                        //      SM_SP_U    = test for pts on the u min or max bdry (moving along +/- v dir)
                                                 //      SM_SP_V    = test for pts on the v min or max bdry (moving along +/- u dir)
                                                 //      All others treated the same as SM_SP_BOTH
                                                 
  // When Span [ThisDropPt crEnd] drops to Surface within sApproxTol3d - return SM_SUCCESS
  SmStatus TestSpanAccuracy                      // in : m_dT
                                                 //      m_CrvPD
   (const SmSurface  & crSurface,                // in : target surface
    const SmExtent2d & crUVDomain,               // in : surface domain of interest
    const SmCurve    & cr3dCurve,                // in : curve being dropped
    SmDropPt         & rEnd,                     // in : rEnd.m_dT
                                                 // out: rEnd.m_CrvDP
    SmApproxTol3d      sApproxTol3d,             // in : min dist between 3d pts - determines when spans run outside srfBdry
                                                 //                                and CrvPts are not on associated SrfNormals 
    double             dUVTolX,                  // NotUsed: in : UV tol in the u dir
    double             dUVTolY,                  // NotUsed: in : UV tol in the v dir
    SmBoolean        & rbSpanPassesTest,         // out: TRUE = span is good - spans shorter than sIvl/1000.0
                                                 //                            all sample CrvPts within sApproxTol3d of SrfPt Normals 
                                                 //      FALSE= span is bad  - spans that leave srfBdry by more than sApproxTol3d
                                                 //                            sample CrvPt not within sApproxTol3d of SrfPt Normal
    double           & rdMaxDropToSurf,          // out: largest seen SurfacePt to CurvePt distance
    double           & rdMaxDropToSurfParam,     // out: Param value for rdMaxDropToSurf
    double           & rdMaxApproxDev,           // out: largest seen deviation of CurvePt from SrfNormalLine
    SmBoolean          bDoRefinement=TRUE) ;     // NotUsed: in : TRUE = attempt refining span 1stDeriv mags when span fails, default:[TRUE]
                                                 
                                                 
  // side effects                                

  // load an EndSpan DropPt from a StartSpan DropPt and a desired curve T value 
  SmStatus GetNextPoint                          // rtn: (1000) SM_SUCCESS              (1020) SM_ERR_OUTSIDE_OF_DOMAIN
                                                 //      (1001) SM_ERR                  (1021) SM_ERR_NOT_WITHIN_TOLERANCE  
                                                 //      (1032) SM_ERR_BAD_TANGENT_DROP (1030) SM_ERR_BAD_SURFACE_POINT
   (const SmSurface           & crSurface,       // in : main surface
    SmOffsetSurface           & rOffset,         // i/o: offset of main surface, its offset distance is set in this routine
    ULONG                       lSingularities,  // in : orof: SM_SS_NONE, SM_SS_UMIN, SM_SS_VMIN, SM_SS_UMAX, SM_SS_VMAX, SM_SS_UNKNOWN
    const SmTArray<SmPoint3d> * sSrfPolePoints,  // in : PolePoint[4] array, ordered :[UMinPole, VMinPole, UMaxPole, VMaxPole]
                                                 //      NonSingular side values set to
                                                 //      SmPoint3d::SetUninitialized(),
    const SmExtent2d          & crUVDomain,      // in : limiting domain of the surface
    SmSurfParamType             eSrfClosure,     // in : oneof: SM_SP_U, SM_SP_V, SM_SP_BOTH, SM_SP_NEITHER
    const SmCurve             & cr3dCurve,       // in : Curve being dropped
    SmApproxTol3d               sApproxTol3d,    // in : max allowed distance between drop point and surfNormal line at drop point,
    double                      dUVTolX,         // NotUsed: in : UV Tolerance in the u direction
    double                      dUVTolY,         // NotUsed: in : UV Tolerance in the v direction
    double                      dEndT,           // in : Param value of EndSpan Curve point being dropped 
    SmDropPt                  & rEnd) ;          // in : crStart.m_dT
                                                 //      crStart.m_CrvPD, crStart.m_bFromLeftU, crStart.m_bFromLeftV
                                                 //      crStart.m_bDegenCurve
                                                 // out: rEnd.m_dT  
                                                 //      rEnd.m_CrvPD 
                                                 //      rEnd.m_UVPD, rEnd.m_bFromLeftU, rEnd.m_bFromLeftV,
                                                 //      rEnd.m_3dPD
                                                 //      rEnd.m_dDropToSurf,     rEnd.m_dApproxDev
                                                 //      rEnd.m_dNormalToCrvAngDeg,
                                                 //      rEnd.m_dSrfToCrvAngDeg, rEnd.m_dSrfToCrvSpeedRatio
                                                 //      rEnd.m_bOnBoundary,     rEnd.m_bLeavingDomain,  rEnd.m_bEnteringDomain
                                                 //      rEnd.m_eDropDerivRtn    rEnd.m_bOnSeam          rEnd.m_bOnPole
                                                 //      rEnd.m_bDegenCurve
                                                 //         SM_SUCCESS = good drop, 
                                                 //         SM_ERR_NOT_WITHIN_TOLERANCE = CrvPt not with tol of SrfNormalLine at dropPoint
                                                 //                usually caused by dropping a CrvPt not over the Srf to a SrfBoundary
                                                 //         SM_ERR_OUTSIDE_OF_DOMAIN = span leaves the domain or crosses a seam beyond snap tolerance
                                                 //         SM_ERR_BAD_TANGENT_DROP  = CrvTangent dropped to nonParallel SrfTangent
                                                 //                usually caused by dropping a CrvTangent near a srfPole

  // Drop m_CrvPD[1] to m_UVPD[1] and gather all related data
  SmStatus SetProperties                         // rtn:  (1000) SM_SUCCESS  (1032) SM_ERR_BAD_TANGENT_DROP               
                                                 //       (1001) SM_ERR      (1021) SM_ERR_NOT_WITHIN_TOLERANCE               
   (const SmSurface  &crSurface,                 // in : target drop surface
    SmOffsetSurface  &rOffset,                   // i/o: Offset from crSurface, it's offset dist gets modified to minimize tolerances
    ULONG             lSingularities,            // in : orof: SM_SS_NONE, SM_SS_UMIN, SM_SS_VMIN, SM_SS_UMAX, SM_SS_VMAX
    SmApproxTol3d     sApproxTol3d,              // in : min dist between distinct 3d points
    const SmExtent2d &crUVDomain,                // in : Domain of interest for crSurface
    SmBoolean         bAdjustPolePt) ;           // in : TRUE = adjust m_UVPD[0] with FindDegenParamForDirection when m_UVPD[0] is on pole
                                                 //      FALSE= don't
                                                 // in : m_UVPD[0]  = drop pCurve(dT) position to surface 2d point (Solution of GlobalPointSolve for dT) 
                                                 //      m_CrvPD[0] = position pCurve(dT)
                                                 //      m_CrvPD[1] = 1stDeriv pCurve(dT)
                                                 //      m_CrvPD[2] = 2ndDeriv pCurve(dT)
                                                 //      m_bDegenCurve (When TRUE projections of Curve based on NonZero derivatives are left Uninit)
                                                 // out: m_UVPD[1],             Proj of m_CrvPD[1] to Surface
                                                 //      m_bGoodDrop,           TRUE = m_dApproxDev < sApproxTol3d
                                                 //      m_3dPD[0],             Proj of m_UVPD[0] through Surface back to 3d
                                                 //      m_3dPD[1],             Proj of m_UVPD[1] through Surface back to 3d
                                                 //      m_SrfNormal,           SurfaceNormal at DropPoint
                                                 //      m_bFromLeftU,          TRUE=Dropped Srf 1stDeriv starts in pos U direction, FALSE=in neg U direction
                                                 //      m_bFromLeftV,          TRUE=Dropped Srf 1stDeriv starts in pos V direction, FALSE=in neg V direction
                                                 //      m_dDropToSurfDrop                                                         
                                                 //      m_dApproxDev,          deviation of m_CrvPD[0] from SrfNormalLine starting at Surface(sSrfUV[0])
                                                 //      m_dNormalToCrvAngDeg,  angleDeg between m_CrvPD[1] and SurfacePlane Normal, should be 90.0
                                                 //      m_dSrfToCrvAngDeg,     angleDeg between m_CrvPD[1] and SurfaceProjection of sSrfUV[1], should be near 0.0
                                                 //      m_dSrfToCrvSpeedRatio, m_3dPD[1].Length() / m_CrvPD[1], should be near 1.0 
                                                 //      m_bOnBoundary, 
                                                 //      m_bOnSeam,
                                                 //      m_bOnPole,
                                                 //      m_bLeavingDomain, 
                                                 //      m_bEnteringDomain
                                                 //      m_eDropDerivRtn        SM_SUCCESS = good drop,
                                                 //                             SM_ERR_BAD_TANGENT_DROP = CrvTangent drop produced a non Parallel SrfTangent direction
                                                 //                             SM_ERR_NOT_WITHIN_TOLERANCE = Drop not along surface normal
                                                 //                                     usually caused by dropping a CrvPt not over the surface to surface boundary.

  // Correct a seam-side choice only when the 3D endpoint stays within tolerance
  SmStatus CheckSeamJump                         // in : m_CrvPD,
                                                 //      m_UVPD, m_bFromLeftU, m_bFromLeftV
   (const SmSurface  * cpSurface,                // in : Surface to which Curve is being dropped                                       
    SmSurfParamType    eSrfClosure,              // in : one of SM_SP_U, SM_SP_V, SM_SP_NEITHER, SM_SP_BOTH                            
    SmApproxTol3d      sApproxTol3d,             // in : maximum 3D displacement allowed by a seam correction
    SmDropPt         & rEnd)                     // i/o: in : rEnd.m_UVPD[0]
                                                 //      out: seam-side correction only when within sApproxTol3d
   const ;

  // When Start and End DropPts are on Surface boundary - snap Start and End m_UVPD[1] to point along the boundary
  SmStatus FixBoundaryTangents                  // in : m_UVPD[0], m_UVPD[1]
                                                // out: m_UVPD[1]: When on bdry, set m_UVPD[1].param = 0
   (const SmExtent2d & crUVDomain,              // in : Domain range
    SmDropPt         & rEnd,                    // i/o: rEnd.m_UVPD[0]=current dropped point, 
                                                //      rEnd.m_UVPD[1]=current drop direction
                                                //      When on bdry, set rEnd.m_UVPD[1].param = 0
    double             dUVTolX,                 // in : UV tol in the u dir
    double             dUVTolY,                 // in : UV tol in the v dir
    SmApproxTol3d      sApproxTol3d,            // in : not currently used in this function   
    SmBoolean        & rbFixBTChange) ;         // out: TRUE = changed a value, FALSE = didn't

  // Modify Start and Stop 1stDeriv vectors to optimize Approx DropSpan to Actual Curve
  SmStatus RefineSpan1stDerivs                  // in : m_CrvPD,
                                                //      m_UVPD, m_bFromLeftU, m_bFromLeftV
                                                // out: m_UVPD[1] = optimized SpanStart[UV1stDeriv]
   (const SmSurface  & crSurface,               // in : target surface
    SmOffsetSurface  & rOffset,                 // i/o: offset of main surface, its offset distance is set in this routine
    ULONG             lSingularities,           // in : orof: SM_SS_NONE, SM_SS_UMIN, SM_SS_VMIN, SM_SS_UMAX, SM_SS_VMAX
    const SmExtent2d & crUVDomain,              // in : surface domain of interest
    SmSurfParamType    eSrfClosure,             // in : oneof: SM_SP_U, SM_SP_V, SM_SP_BOTH, SM_SP_NEITHER
    const SmCurve    & cr3dCurve,               // in : curve being dropped
    SmApproxTol3d      sApproxTol3d,            // in : max allowed distance between drop point and surfNormal line at drop point,
    double             dUVTolX,                 // in : UV Tolerance in the u direction
    double             dUVTolY,                 // in : UV Tolerance in the v direction
    SmBoolean          bSpanStartGoodDir,       // in : TRUE = preserve input sStart.m_UVPD[1] direction, FALSE = optimize it
    SmDropPt         & rEnd,                    // in : sEnd.m_CrvPD, 
                                                //      sEnd.m_UVPD[0] 
                                                // i/o: sEnd.m_UVPD[1] = SpanEnd[UV1stDeriv]
    SmBoolean          bSpanEndGoodDir,         // in : TRUE = preserve input sStart.m_UVPD[1] direction, FALSE = optimize it
    double            &rdDropToSurf,            // out: Drop distance
    double            &rdApproxDev) ;           // out: deviation of sCrvPD[0] from SrfNormalLine starting at Surface(sSrfUV[0])

  // Find a DropToBoundaryPoint between a DropToSurface and a NoDropToSurfacePoint
  SmStatus IterateToBoundary                    // in : m_dT, m_UVPD, m_bFromLeftU, m_bFromLeftV, m_bGoodDrop
   (const SmSurface & crSurface,                 // in : target surface
    SmOffsetSurface & rOffset,                   // in : target surface offset  
    ULONG             lSingularities,            // in : orof: SM_SS_NONE, SM_SS_UMIN, SM_SS_VMIN, SM_SS_UMAX, SM_SS_VMAX
    SmExtent2d      & rUVDomain,                 // in : Surface domain   
    SmSurfParamType   eSrfClosure,               // in : oneof: SM_SP_U, SM_SP_V, SM_SP_BOTH, SM_SP_NEITHER                 
    const SmCurve   & crCurve,                   // in : Curve being dropped
    SmApproxTol3d     sApproxTol3d,              // in : max allowed distance between drop point and surfNormal line at drop point                                                                   
    double            dUVTolX,                   // in : UV tol in u dir
    double            dUVTolY,                   // in : UV tol in v dir  
    const SmDropPt  & crEnd,                     // in : m_dT, m_UVPD, m_bFromLeftU, m_bFromLeftV, m_bGoodDrop
    SmDropPt        & rDropPt)                   // out: Drop to Boundary DropPt
   const ;


  // utilities
  void            Dump(double dTol, const TCHAR *pOptLabel = NULL) const ;
  SmDisplayList * Draw()                        const ; 

} ; // end class SmDropPt

/*******************************************************************//**
PURPOSE: Pretty Print a Drop Point

NOTES:
***********************************************************************/
#define DUMPVEC(a) if(!(a).IsUndef()) { (a).Dump() ; } else { smos_WriteBuffer(_T(" [UNINITIALIZED]")) ; }
#define DUMPVAL(a) if( (a) != SM_UNDEF_DOUBLE) \
                                  { smos_sprintf(sBuff, _T("[%16.16lf]"),(a)) ; smos_WriteBuffer(sBuff) ; } \
                             else { smos_WriteBuffer(_T(" [UNINITIALIZED]")) ; }  
void SmDropPt::Dump
  (double dTol,
   const TCHAR *pOptLabel) 
 const
{
  TCHAR sBuff[SM_TBLOCK_SIZE] ;

  // Label
  smos_WriteBuffer(_T("\n")) ;
  if(pOptLabel) { smos_WriteBuffer(pOptLabel) ; }
  else          { smos_WriteBuffer(_T("Drop Point: ")) ; }

  // Param and State
  smos_sprintf(sBuff, _T("\n  T [%16.16lf]"),
             m_dT) ;
  smos_WriteBuffer(sBuff) ;
                                                        
  smos_sprintf(sBuff, _T("\n  m_bGoodDrop      [%s], m_bOnBoundary[%s],  bFromLeftU[%s]"),
             m_bGoodDrop   == TRUE ? _T("TRUE ") : m_bGoodDrop   == FALSE ? _T("FALSE") : _T("UNINIT"),
             m_bOnBoundary == TRUE ? _T("TRUE ") : m_bOnBoundary == FALSE ? _T("FALSE") : _T("UNINIT"),
             m_bFromLeftU  == TRUE ? _T("TRUE ") : m_bFromLeftU  == FALSE ? _T("FALSE") : _T("UNINIT")) ; 
  smos_WriteBuffer(sBuff) ;
                                                     
  smos_sprintf(sBuff, _T("\n  m_bEnteringDomain[%s], m_bOnSeam    [%s],  bFromLeftV[%s]"),
             m_bEnteringDomain == TRUE ? _T("TRUE ") : m_bEnteringDomain == FALSE ? _T("FALSE") : _T("UNINIT"),
             m_bOnSeam         == TRUE ? _T("TRUE ") : m_bOnSeam         == FALSE ? _T("FALSE") : _T("UNINIT"),
             m_bFromLeftV      == TRUE ? _T("TRUE ") : m_bFromLeftV      == FALSE ? _T("FALSE") : _T("UNINIT")) ; 
  smos_WriteBuffer(sBuff) ;

  smos_sprintf(sBuff, _T("\n  m_bLeavingDomain [%s], m_bOnPole    [%s],  eDropDerivRtn[%s]"),
             m_bLeavingDomain == TRUE ? _T("TRUE ") : m_bLeavingDomain == FALSE ? _T("FALSE") : _T("UNINIT"),
             m_bOnPole        == TRUE ? _T("TRUE ") : m_bOnPole        == FALSE ? _T("FALSE") : _T("UNINIT"),
             m_eDropDerivRtn == SM_SUCCESS   ? _T("SM_SUCCESS")
           : m_eDropDerivRtn == SM_ERR       ? _T("SM_ERR")
           : m_eDropDerivRtn == SM_BIG_ULONG ? _T("UNINIT")
           : _T("An Error Code") ) ; 
  smos_WriteBuffer(sBuff) ;
                       
  smos_sprintf(sBuff, _T("\n                            m_bDegenCurve[%s]"),
             m_bDegenCurve == TRUE ? _T("TRUE ") : m_bDegenCurve == FALSE ? _T("FALSE") : _T("UNINIT")) ;
   smos_WriteBuffer(sBuff) ;

  if(m_eDropDerivRtn != SM_SUCCESS && m_eDropDerivRtn != SM_ERR && m_eDropDerivRtn != SM_BIG_ULONG)
    { smos_sprintf(sBuff,_T("[%ld]"), m_eDropDerivRtn) ;
      smos_WriteBuffer(sBuff) ;
    }

  // measures
  smos_WriteBuffer(_T("\n  dDropToSurf dist    = ")) ; DUMPVAL(m_dDropToSurf) ;         smos_WriteBuffer(_T(" From CrvPt to SrfDropPt dist: 0.0 for Pts on Surf")) ;
  smos_WriteBuffer(_T("\n  dTol                = ")) ; DUMPVAL(dTol) ;
  smos_WriteBuffer(_T("\n  dApproxDev  dist    = ")) ; DUMPVAL(m_dApproxDev) ;          smos_WriteBuffer(_T(" From CrvPt to SrfNormalLine dist: 0.0 for Drop along SrfNormal - should be less than tol")) ; 
  smos_WriteBuffer(_T("\n  dNormalToCrvAngDeg  = ")) ; DUMPVAL(m_dNormalToCrvAngDeg) ;  smos_WriteBuffer(_T(" Crv1stDir to SrfPlane Normal Ang - should be 90.0")) ;
  smos_WriteBuffer(_T("\n  dSrfToCrvAngDeg     = ")) ; DUMPVAL(m_dSrfToCrvAngDeg) ;     smos_WriteBuffer(_T(" Crv1stDir dropped to SrfPlane to UV1stDir proj through surface Ang - should be 0.0")) ;
  smos_WriteBuffer(_T("\n  dSrfToCrvSpeedRatio = ")) ; DUMPVAL(m_dSrfToCrvSpeedRatio) ; smos_WriteBuffer(_T(" Crv1stDir dropped to SrfPlane to UV1stDir proj through surface Speed Ratio- should be 1.0")) ;

  // CrvPD
  smos_WriteBuffer(_T("\n  m_CrvPD[0] = ")) ; DUMPVEC(m_CrvPD[0]) ;
  smos_WriteBuffer(_T("\n  m_CrvPD[1] = ")) ; DUMPVEC(m_CrvPD[1]) ;
  smos_WriteBuffer(_T("\n  m_CrvPD[2] = ")) ; DUMPVEC(m_CrvPD[2]) ;

  // UVPD
  smos_WriteBuffer(_T("\n  m_UVPD[0]  = ")) ; DUMPVEC(m_UVPD[0]) ; 
  smos_WriteBuffer(_T("\n  m_UVPD[1]  = ")) ; DUMPVEC(m_UVPD[1]) ;

  // 3dPD
  smos_WriteBuffer(_T("\n  m_3dPD[0]  = ")) ; DUMPVEC(m_3dPD[0]) ;
  smos_WriteBuffer(_T("\n  m_3dPD[1]  = ")) ; DUMPVEC(m_3dPD[1]) ;

  // SurfaceNormal
  smos_WriteBuffer(_T("\n  SrfNormal  = ")) ; DUMPVEC(m_SrfNormal) ;
                                          
} // end SmDropPt::Dump
#undef DUMPVEC
#undef DUMPVAL

/*******************************************************************//**
PURPOSE: Draw DropPt

NOTES:  CrvPt        (red)
        Crv1stDir    (red)
        Srf 3dPt     (current color)
        Srf 3d1stDir (current color)
        Srf Normal   (green)
        Drop Vector  (magenta)
***********************************************************************/
SmDisplayList * SmDropPt::Draw() 
 const
{
  SmDisplayList *pRtn = NULL ;

#ifdef SM_GFX_CODE
  // Get global SmGraphicsExtern.cpp:s_Disp display parameters
  SmDisplayParameters sDisp ;
  smgfx_GetGlobalDisplayParameters(sDisp) ;

  // start new displayList (unless one is already open)
  SmVector3d sColor     = smgfx_GetOutputColor() ;
  double     dW = smgfx_GetLineWidth() ;
  double     dS = smgfx_GetPointSize() ;
  smgfx_Open(smgfx_GetRuleColor());

  // CrvPD
  smgfx_SetColor(1,0,0) ; if(   !m_CrvPD[0].IsUndef())  { smgfx_OutputPoint(m_CrvPD[0].x,m_CrvPD[0].y,m_CrvPD[0].z) ; }
  smgfx_SetColor(1,0,0) ; if(   !m_CrvPD[0].IsUndef()   
                             && !m_CrvPD[1].IsUndef())  { smgfx_OutputLine (m_CrvPD[0].x,m_CrvPD[0].y,m_CrvPD[0].z,
                                                                            m_CrvPD[0].x+m_CrvPD[1].x,
                                                                            m_CrvPD[0].y+m_CrvPD[1].y,
                                                                            m_CrvPD[0].z+m_CrvPD[1].z) ;
                                                        }

  // 3dPD
  smgfx_SetLook(dW, dS+2, sColor) ; if(   !m_3dPD[0].IsUndef())   { smgfx_OutputPoint(m_3dPD[0].x,m_3dPD[0].y,m_3dPD[0].z) ; }
  smgfx_SetLook(dW, dS,   sColor) ; if(   !m_3dPD[0].IsUndef()    
                                       && !m_3dPD[1].IsUndef())   { smgfx_OutputLine (m_3dPD[0].x,m_3dPD[0].y,m_3dPD[0].z,
                                                                                      m_3dPD[0].x+m_3dPD[1].x,
                                                                                      m_3dPD[0].y+m_3dPD[1].y,
                                                                                      m_3dPD[0].z+m_3dPD[1].z) ;
                                                                  }
  // SurfaceNormal
  smgfx_SetLook(dW, dS, 0,1,0) ; if(   !m_3dPD[0].IsUndef()    
                                    && !m_SrfNormal.IsUndef())   { smgfx_OutputLine (m_3dPD[0].x,m_3dPD[0].y,m_3dPD[0].z,
                                                                                     m_3dPD[0].x+m_SrfNormal.x,
                                                                                     m_3dPD[0].y+m_SrfNormal.y,
                                                                                     m_3dPD[0].z+m_SrfNormal.z) ;
                                                                 }
  // drop vector
  smgfx_SetLook(dW+1, dS, .5,0,1) ; if(   !m_CrvPD[0].IsUndef()    
                                       && !m_3dPD[0].IsUndef())  { smgfx_OutputLine (m_CrvPD[0].x,m_CrvPD[0].y,m_CrvPD[0].z,
                                                                                     m_3dPD[0].x,m_3dPD[0].y,m_3dPD[0].z) ;
                                                                 }
  // end display list
  pRtn = smgfx_Close() ;
  smgfx_OutputColor(sColor) ;
  smgfx_OutputLineWidth(dW) ;
  smgfx_OutputPointSize(dS) ;

#endif // SM_GFX_CODE
  return(pRtn) ;

} // end SmDropPt::Draw

/*******************************************************************//**
PURPOSE: DropCurve helper function.  
   Determine whether a trace point is moving along the boundary of a surface
   using only the current drop point information (not the last drop point).

NOTES:
  When (    UV2dPoint is on given Domain boundary to tol
        and Crv3dTangent is within 5 degs of SurfaceBdryTangent)
  THEN Curve is 'moving' along boundary and returns TRUE.

  if CurveTangent or SurfaceTangents are zero, returns FALSE.

  returns oneof: SM_POC_INSIDE       = curve tangent is moving into surface from a boundary
                 SM_POC_OUTSIDE      = curve tangent is moving out of surface from a boundary
                 SM_POC_ON_BOUNDARY  = curve tangent is moving along boundary from a boundary
                 SM_POC_UNKNOWN      =    curve drop UV point is not on a surface domain boundary 
                                       or curve1stDeri or SrfBdry1stDeriv is zero
***********************************************************************/
SmPointObjectContainmentType SmDropPt::IsMovingAlongBoundary
 (const SmSurface  & crSurface,             // in : tgt surface
  const SmExtent2d & crUVDomain,            // in : surface domain of interest
  double             dUVTolX,               // in : UV Tolerance in the U direction
  double             dUVTolY,               // in : UV Tolerance in the V direction
  double             dAngLimitDeg,          // in : max angle between CrvTang and SrfBndryTanget to be moving along boundary        
                                            //      default:[5]
  SmSurfParamType    eTestBdry)             // in : SM_SP_BOTH = test for pts on any bdry  
 const                                      //      SM_SP_U    = test for pts on the u min or max bdry (moving along +/- v dir)
                                            //      SM_SP_V    = test for pts on the v min or max bdry (moving along +/- u dir)
                                            //      All others treated the same as SM_SP_BOTH
                                            //      default:[IS_SP_BOTH]
                                            //
                                            // in : m_UVPD[0]  = DropPt SrfUV 2dPoint
                                            //      m_CrvPD[2] = Curve Point and 1stDeriv
{
  // return value
  SmPointObjectContainmentType ePOCType = SM_POC_UNKNOWN ;

  // locals
  SmPoint3d          sSrfPt ;
  SmVector3d         sSu, sSv, sSrfNorm ;
  double             dAngRad ;
  const SmExtent1d & crURange = crUVDomain.GetUInterval();
  const SmExtent1d & crVRange = crUVDomain.GetVInterval();
  SmSurfParamType    eTest    = (eTestBdry != SM_SP_U && eTestBdry != SM_SP_V) ? SM_SP_BOTH : eTestBdry ;

  // Angle limit for 'along': default:[5 degrees]
  double dAngLimitRad = SM_DEG2RAD( dAngLimitDeg );

  // Check running along U-boundary ( in +- v direction).

  // when UVPt is on UMin/UMax boundary                
  if((eTest == SM_SP_U || eTest == SM_SP_BOTH) && crURange.IsValueOnBoundary( this->m_UVPD[0].x, dUVTolX ) )
  // if ( crURange.IsValueOnBoundary( this->m_UVPD[0].x, dUTol ) )
    {
      // which surface bdry, Sv 1stDeriv, and m_SrfNorm
      SmBoolean bEvalLowSide = ( this->m_UVPD[0].x < crURange.GetMid() );
      crSurface.Evaluate1stDerivatives( this->m_UVPD[0], bEvalLowSide, FALSE, sSrfPt, sSu, sSv );
      crSurface.EvaluateNormal( this->m_UVPD[0], bEvalLowSide, FALSE, sSrfNorm) ;

      // when either Crv or SrfBdry is zero length - then not moving along boundary
      if(   sSv.LengthSquared()        < SM_EFF_ZERO_SQ
         || this->m_CrvPD[1].LengthSquared() < SM_EFF_ZERO_SQ)
        { return( SM_POC_UNKNOWN ) ; }
      
      // curveTangent to Surface Sv angle - [-Pi to Pi] returns zero for zero length vectors
      sSrfNorm.CCWAngleBetween( sSv, this->m_CrvPD[1], dAngRad) ;

      // sector classification
      ePOCType =   (   dAngRad > -SM_PI + dAngLimitRad
                    && dAngRad <  0.0   - dAngLimitRad) ? ( bEvalLowSide ? SM_POC_INSIDE : SM_POC_OUTSIDE )
                 : (   dAngRad >  0.0   + dAngLimitRad 
                    && dAngRad <  SM_PI - dAngLimitRad) ? ( bEvalLowSide ? SM_POC_OUTSIDE : SM_POC_INSIDE )
                 : SM_POC_ON_BOUNDARY ;

      // watch out for raindrops (Sv on both sides of the seam classify the CurveTangent as Inside)
      // compare the curve tangent direction to surface normals on both sides of the seam
      if(ePOCType == SM_POC_INSIDE)
        {
          SmPointObjectContainmentType eOtherPOCType = SM_POC_UNKNOWN ;
          
          // get other side seam evaluations
          SmPoint2d  sOtherUV(bEvalLowSide ? crURange.GetMax() : crURange.GetMin(), m_UVPD[0].y) ;
          SmPoint3d  sOtherPt ;
          SmVector3d sOtherSu, sOtherSv, sOtherNorm ;
          double     dOtherAngRad ;
          crSurface.Evaluate1stDerivatives( sOtherUV, !bEvalLowSide, FALSE, sOtherPt, sOtherSu, sOtherSv );
          crSurface.EvaluateNormal( sOtherUV, !bEvalLowSide, FALSE, sOtherNorm) ;

          // curveTangent to Surface Sv angle - [-Pi to Pi] returns zero for zero length vectors
          sOtherNorm.CCWAngleBetween( sOtherSv, this->m_CrvPD[1], dOtherAngRad) ;

          // other sector classification
          eOtherPOCType =   (   dOtherAngRad > -SM_PI + dAngLimitRad
                             && dOtherAngRad <  0.0   - dAngLimitRad) ? ( !bEvalLowSide ? SM_POC_INSIDE : SM_POC_OUTSIDE )
                          : (   dOtherAngRad >  0.0   + dAngLimitRad 
                             && dOtherAngRad <  SM_PI - dAngLimitRad) ? ( !bEvalLowSide ? SM_POC_OUTSIDE : SM_POC_INSIDE )
                          : SM_POC_ON_BOUNDARY ;

          if(eOtherPOCType == SM_POC_INSIDE)
            {
              // let ePOCType = INSIDE when m_dNormalToCrvAngDeg is closer to 90 deg than OtherNormalToCrvAngDeg
              double dOtherNormalToCrvAngRad ;
              sOtherNorm.AngleBetween(this->m_CrvPD[1],dOtherNormalToCrvAngRad) ;
              
              ePOCType = smos_Fabs(m_dNormalToCrvAngDeg - 90.0) < smos_Fabs(SM_RAD2DEG(dOtherNormalToCrvAngRad) - 90.0)
                         ? SM_POC_INSIDE
                         : SM_POC_OUTSIDE ; 

            } // end eOtherPOCType == SM_POC_INSIDE
        } // end ePOCType == SM_POC_INSIDE check

      // all done
      return(ePOCType) ;

      //      // curveTangent to Surface Sv angle - returns zero for zero length vectors
      //      if ( this->m_CrvPD[1].AngleBetween( sSv, dAngRad ) != SM_SUCCESS )
      //        { 
      //          // arrive here when either tangent is degenerate: then not moving along boundary
      //          return FALSE; 
      //        } 
      //      
      //      // when Tangents' angle is less than limit
      //      if(   dAngRad <= dAngLimitRad 
      //         || dAngRad > SM_PI - dAngLimitRad )
      //        { 
      //          return TRUE; 
      //        }

    } // end check along U-boundary

  // Check running along V-boundary ( in +- u direction).
  
  // when UVPt is on VMin/VMax boundary   
  if((eTest == SM_SP_V || eTest == SM_SP_BOTH) && crVRange.IsValueOnBoundary( this->m_UVPD[0].y, dUVTolY ) )
  // if ( crVRange.IsValueOnBoundary( this->m_UVPD[0].y, dVTol ) )
    {
      // which surface bdry, Su 1stDeriv, and m_SrfNorm
      SmBoolean bEvalLowSide = ( this->m_UVPD[0].y < crVRange.GetMid() );
      crSurface.Evaluate1stDerivatives( this->m_UVPD[0], FALSE, bEvalLowSide, sSrfPt, sSu, sSv );
      crSurface.EvaluateNormal( this->m_UVPD[0], FALSE, bEvalLowSide, sSrfNorm) ;

      // curveTangent to Surface Su angle - [-Pi to Pi] returns zero for zero length vectors
      sSrfNorm.CCWAngleBetween( sSu, this->m_CrvPD[1], dAngRad) ;

      // sector classification
      ePOCType =   (   dAngRad > -SM_PI + dAngLimitRad
                    && dAngRad <  0.0   - dAngLimitRad) ? ( bEvalLowSide ? SM_POC_OUTSIDE : SM_POC_INSIDE )
                 : (   dAngRad >  0.0   + dAngLimitRad 
                    && dAngRad <  SM_PI - dAngLimitRad) ? ( bEvalLowSide ? SM_POC_INSIDE : SM_POC_OUTSIDE )
                 : SM_POC_ON_BOUNDARY ;
      
      // watch out for raindrops (Su on both sides of the seam classify the CurveTangent as Inside)
      // compare the curve tangent direction to surface normals on both sides of the seam
      if(ePOCType == SM_POC_INSIDE)
        {
          SmPointObjectContainmentType eOtherPOCType = SM_POC_UNKNOWN ;
          
          // get other side seam evaluations
          SmPoint2d  sOtherUV(m_UVPD[0].x, bEvalLowSide ? crVRange.GetMax() : crVRange.GetMin() ) ;
          SmPoint3d  sOtherPt ;
          SmVector3d sOtherSu, sOtherSv, sOtherNorm ;
          double     dOtherAngRad ;
          crSurface.Evaluate1stDerivatives( sOtherUV, !bEvalLowSide, FALSE, sOtherPt, sOtherSu, sOtherSv );
          crSurface.EvaluateNormal( sOtherUV, !bEvalLowSide, FALSE, sOtherNorm) ;

          // curveTangent to Surface Sv angle - [-Pi to Pi] returns zero for zero length vectors
          sOtherNorm.CCWAngleBetween( sOtherSu, this->m_CrvPD[1], dOtherAngRad) ;

          // other sector classification
          eOtherPOCType =   (   dOtherAngRad > -SM_PI + dAngLimitRad
                             && dOtherAngRad <  0.0   - dAngLimitRad) ? ( !bEvalLowSide ? SM_POC_OUTSIDE : SM_POC_INSIDE )
                          : (   dOtherAngRad >  0.0   + dAngLimitRad 
                             && dOtherAngRad <  SM_PI - dAngLimitRad) ? ( !bEvalLowSide ? SM_POC_INSIDE : SM_POC_OUTSIDE )
                          : SM_POC_ON_BOUNDARY ;

          if(eOtherPOCType == SM_POC_INSIDE)
            {
              // let ePOCType = INSIDE when m_dNormalToCrvAngDeg is closer to 90 deg than OtherNormalToCrvAngDeg
              double dOtherNormalToCrvAngRad ;
              sOtherNorm.AngleBetween(this->m_CrvPD[1],dOtherNormalToCrvAngRad) ;
              
              ePOCType = smos_Fabs(m_dNormalToCrvAngDeg - 90.0) < smos_Fabs(SM_RAD2DEG(dOtherNormalToCrvAngRad) - 90.0)
                         ? SM_POC_INSIDE
                         : SM_POC_OUTSIDE ; 

            } // end eOtherPOCType == SM_POC_INSIDE
        } // end ePOCType == SM_POC_INSIDE check

      // all done
      return(ePOCType) ;

      //      // curveTangent to Surface Su angle
      //      if ( this->m_CrvPD[1].AngleBetween( sSu, dAngRad ) != SM_SUCCESS )
      //        { 
      //          // arrive here when either tangent is degenerate: then not moving along boundary
      //          return FALSE; 
      //        } 
      //      
      //      // when Tangents' angle is less than limit
      //      if ( dAngRad <= dAngLimitRad || dAngRad > SM_PI - dAngLimitRad )
      //        {                    
      //          return TRUE; 
      //        }

    } // end check along U-boundary

  // arrive here when UV point is not on a surface domain boundary
  return SM_POC_UNKNOWN;

} // end SmDropPt::IsMovingAlongBoundary

/*******************************************************************//**
PURPOSE: DropCurve helper function.
   Test whether the surface projection of a hermite cubic dropSpan
   is within tol of the curve being dropped.  Passes spans which
   get less than rIvl/1000.0.

NOTES:
   
   Sample compare the distance between the cr3dCurve and the crSurface(HermiteApprox())
   at 5 interior samples.  When all distances are less than tol the cubic dropSpan is good.

   Short intervals ( SpanIvl.GetLength() <= cr3dCurveIvl.GetLength()/1000.0 ) are
     assumed good to prevent infinite subdivision iteration in problem situations.
***********************************************************************/
SmStatus SmDropPt::TestSpanAccuracy        // rtn: SM_SUCCESS or SER err codes 
  (const SmSurface  & crSurface,           // in : target surface
   const SmExtent2d & crUVDomain,          // in : surface domain of interest
   const SmCurve    & cr3dCurve,           // in : curve being dropped
   SmDropPt         & rEnd,                // in : rEnd.m_dT
                                           // out: rEnd.m_CrvDP
   SmApproxTol3d      sApproxTol3d,        // in : min dist between 3d pts - determines when spans run outside srfBdry
                                           //                                and CrvPts are not on associated SrfNormals 
   double             dUVTolX,             // NotUsed: in : UV tol in the u dir
   double             dUVTolY,             // NotUsed: in : UV tol in the v dir
   SmBoolean        & rbSpanPassesTest,    // out: TRUE = span is good - spans shorter than sIvl/1000.0
                                           //                            all sample CrvPts within sApproxTol3d of SrfPt Normals 
                                           //      FALSE= span is bad  - spans that leave srfBdry by more than sApproxTol3d
                                           //                            sample CrvPt not within sApproxTol3d of SrfPt Normal
   double           & rdMaxDropToSurf,     // out: largest seen SurfacePt to CurvePt distance
   double           & rdMaxDropToSurfParam,// out: Param value for rdMaxDropToSurf
   double           & rdMaxApproxDev,      // out: largest seen deviation of CurvePt from SrfNormalLine
   SmBoolean          bDoRefinement)       // NotUsed: in : TRUE = attempt refining span 1stDeriv mags when span fails, default:[TRUE]
{
  SM_REF3(dUVTolX, dUVTolY, bDoRefinement) ;
  // init output
  rbSpanPassesTest     = TRUE ;
  rdMaxDropToSurf      = 0.0 ;
  rdMaxApproxDev       = 0.0 ;
  rdMaxDropToSurfParam = m_dT ;

  // locals
  const SmDropPt &crStart = *this ;
  SmExtent1d      rIvl(crStart.m_dT, rEnd.m_dT) ;

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  if(bDebugMe)
    {
      crStart.Dump(sApproxTol3d, _T("Start DropPt")) ;
      rEnd.Dump(sApproxTol3d, _T("End DropPt")) ;
    }
#endif // SM_DEBUG_CODE

  // Note: don't do the following, it's no longer necessary
  // and results in needlessly increased tolerances.  [B382]
  // // If StartSpan is moving along a surface bdry - increase tol. 
  // //    (UVPoint within tol of bdry and CurveTanDir within 5 deg of SurfaceTanDir)
  // //   Happens often in practice, and data can be sloppy. [091026]
  // if ( SM_POC_ON_BOUNDARY == crStart.IsMovingAlongBoundary( crSurface, crUVDomain, dUVTolX, dUVTolY ) )
  //   {
  //     sApproxTol3d *= 5.0;
  //   }
  // else // check if EndSpan is moving along a surface bdry
  //   {
  //     // get SpanEnd[CrvPt Crv1stDeriv]
  //     cr3dCurve.Evaluate( rEnd.m_dT, 1, FALSE, rEnd.m_CrvPD, TRUE );  // nonZeroTangent values
  //
  //     // If EndSpan is moving along a surface bdry - increase tol
  //     if ( SM_POC_ON_BOUNDARY == rEnd.IsMovingAlongBoundary( crSurface, crUVDomain, dUVTolX, dUVTolY) )
  //       {
  //         sApproxTol3d *= 5.0;
  //       }
  //   } // end need to increase tol check

  // current parameter step size
  double dDeltaT = rEnd.m_dT - crStart.m_dT;

  // Don't let the parameter step get too small
  if (dDeltaT < rIvl.GetLength()/1000.0)
    {
      rbSpanPassesTest = TRUE;
      return SM_SUCCESS;
    }

  // Cubic Hermite approximation to UVCurve 
  //  (note: scale tangents as if 3DCurve from crStart.m_dT to rEnd.m_dT were scaled from 0 to 1)
  SmHermiteCurve sHermite( crStart.m_UVPD[0],
                           crStart.m_UVPD[1] * dDeltaT, 
                           rEnd.m_UVPD[0],   
                           rEnd.m_UVPD[1] * dDeltaT );
#ifdef SM_DEBUG_CODE
  if(bDebugMe)  // draw curve and sample pts of cr3dCurve and hermite curve on surface
    {
      SM_ASSERT_VALID(&crSurface) ;
      SM_ASSERT_VALID(&cr3dCurve) ;

      SmCrvOnSurf sUVCrvOnSurf(sHermite, (SmSurface &)crSurface) ;
      SmEdge *pEdge = (SmEdge *)cr3dCurve.GetEdge() ;
      SmFace *pFace = (SmFace *)crSurface.GetFace() ;
      SmBrep *pBrep =   pFace ? pFace->GetBrep() : pEdge ? pEdge->GetBrep() : NULL ;
      
      SmPoint3d sCrvMidPV[2] ;
      SmPoint3d sSrfStart[2][2], sSrfMid[2][2], sSrfEnd[2][2] ;
      SmPoint3d sSrfStart1stDeriv, sSrfMid1stDeriv, sSrfEnd1stDeriv ;
      SmPoint3d sHermiteMidPV[2] ;
      SmPoint2d sSrfMidUV[2] ;
      sHermite.Evaluate(.5, 1, TRUE, sHermiteMidPV) ;
      sSrfMidUV[0].Set(sHermiteMidPV[0].x, sHermiteMidPV[0].y) ;
      sSrfMidUV[1].Set(sHermiteMidPV[1].x, sHermiteMidPV[1].y) ;

      cr3dCurve.Evaluate((crStart.m_dT+rEnd.m_dT)/2.0, 1, TRUE, sCrvMidPV) ;
      crSurface.Evaluate(crStart.m_UVPD[0], 1, 1, TRUE, TRUE, TRUE, sSrfStart[0]) ;
      crSurface.Evaluate(sSrfMidUV[0],        1, 1, TRUE, TRUE, TRUE, sSrfMid[0]) ;
      crSurface.Evaluate(rEnd.m_UVPD[0],   1, 1, TRUE, TRUE, TRUE, sSrfEnd[0]) ;

      sSrfStart1stDeriv = crStart.m_UVPD[1].x * sSrfStart[1][0] + crStart.m_UVPD[1].y * sSrfStart[0][1] ;
      sSrfMid1stDeriv   = sSrfMidUV[1].x      * sSrfMid[1][0]   + sSrfMidUV[1].y      * sSrfMid[0][1] ;
      sSrfEnd1stDeriv   = rEnd.m_UVPD[1].x    * sSrfEnd[1][0]   + rEnd.m_UVPD[1].y    * sSrfEnd[0][1] ;
      if(FALSE)
        { smgfx_Erase();
          smgfx_SetLook(1,2, 0,0,1); if (pBrep) { pBrep->Draw(TRUE) ; } sm_GraphicsLoop() ;
          smgfx_SetLook(1,2, 0,1,1); crSurface.DrawUV(4,4); sm_GraphicsLoop();
          smgfx_SetLook(1,2, 0,0,0); if (pFace) { pFace->Draw(SM_DM_CROSSHATCH) ; } sm_GraphicsLoop() ;
          smgfx_SetLook(1,2, 0,1,0); cr3dCurve.Draw( &rIvl, TRUE ); sm_GraphicsLoop();
          smgfx_SetLook(3,4, 1,0,0); cr3dCurve.DrawParams( &rIvl ); sm_GraphicsLoop();
          sm_GraphicsLoop() ;
        }
      smgfx_SetLook(1,2, 1,0,0) ; sUVCrvOnSurf.Draw(NULL, TRUE) ; sm_GraphicsLoop() ;

      smgfx_SetLook(5,6, 0,0,1); crStart.m_CrvPD[0].Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,0,1); crStart.m_CrvPD[1].Draw(&crStart.m_CrvPD[0]) ; sm_GraphicsLoop() ;
      smgfx_SetLook(5,6, 0,1,0); sCrvMidPV[0].Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,0); sCrvMidPV[1].Draw(&sCrvMidPV[0]) ; sm_GraphicsLoop() ;
      smgfx_SetLook(5,6, 1,0,0); rEnd.m_CrvPD[0].Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 1,0,0); rEnd.m_CrvPD[1].Draw(&rEnd.m_CrvPD[0]) ; sm_GraphicsLoop() ;

      smgfx_SetLook(8,9, 0,0,1); sSrfStart[0][0].Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,0,1); sSrfStart1stDeriv.Draw(&sSrfStart[0][0]) ; sm_GraphicsLoop() ;
      smgfx_SetLook(8,9, 0,1,0); sSrfMid[0][0].Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,0); sSrfMid1stDeriv.Draw(&sSrfMid[0][0]) ; sm_GraphicsLoop() ;
      smgfx_SetLook(8,9, 1,0,0); sSrfEnd[0][0].Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 1,0,0); sSrfEnd1stDeriv.Draw(&sSrfEnd[0][0]) ; sm_GraphicsLoop() ;

      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

  // locals
  ULONG ii, lNumTestPoints = 3;
  SmPoint3d sHermPnt, sSurfPnt, sCrvPnt[2] ;
  SmPoint2d sUV ;

  // for every test point
  for(ii=0;ii<lNumTestPoints;ii++)
    {
      // Eval HermiteUVSpanCurve to get next sample UV point
      double dNormalizedT = (ii+1.0) / (lNumTestPoints+1.0);
      SER( sHermite.EvaluatePoint( dNormalizedT, sHermPnt ));
      sUV.Set( sHermPnt.x, sHermPnt.y );

      // If the UVPoint is not in the domain - check 3DDist moved when UVPoint is clampped
      if( ! crUVDomain.ContainsPoint2d( sUV, SM_EFF_ZERO ))
        {
          // Get Surface(Clamp(sUV))/Surface(sUV) dist. 
          SmPoint3d sPtFromUV ;
          SmPoint3d sPtFromClampUV;
          SmPoint2d sClampUV = crUVDomain.ClampPoint2d( sUV ) ;
          crSurface.EvaluatePoint( sUV,      sPtFromUV) ;
          crSurface.EvaluatePoint( sClampUV, sPtFromClampUV ) ;

          // check clamp move distance against sApproxTol3d. Used to be - if (crUVDomain.MinimumDistance(sUV) > SM_EFF_ZERO*100.0)
          if(sPtFromUV.DistanceBetween( sPtFromClampUV ) > sApproxTol3d )     
                                                                             
            {
              // SampleUV = HermiteCurve(dNormalizedT) is out of Surface domain 
              // and Clamp distance is larger than sApproxTol3d - don't pass this span
              rbSpanPassesTest = FALSE;
              return SM_SUCCESS;
            }

          // else clamp dist is small - use the clamp sUV value
          sUV = sClampUV ;
        
        } // end UVPoint = Hermite(dNormalizedT) is out of Surface region check

      // get SpanSmp[CrvPt]
      SER( cr3dCurve.Evaluate( crStart.m_dT + dNormalizedT*dDeltaT, 1, TRUE, sCrvPnt ));

      // test SpanSmp[CrvPnt/SurfaceNormal dist] against sApproxTol3d
      SmBoolean bIsGoodPoint;
      SmStatus eReasonIsBad;
      double dDropToSurf, dApproxDev ;
      SmBoolean bFromLeftU, bFromLeftV ;
      SmStatus eStat = smsurf_IsPointOnNormal
                ( crSurface,              // in : target surface                                                   
                 &crUVDomain,             // in : clamp domain: if present, clamp crUVToTest to this domain.       
                                          //      NULL to ignore                                                   
                  sUV,                    // in : Surface test 2Dpoint                                             
                  sCrvPnt[0],             // in : input   test 3DPoint
                 &sCrvPnt[1],             // in : associated 3d tangent direction 
                  bFromLeftU, bFromLeftV, // out: associated from left eval values                                            
                  sApproxTol3d,           // in : Max dist allowed between unique points                           
                  bIsGoodPoint,           // out: TRUE=TestPoint within tol of SurfaceNormal line, FALSE=not       
                  eReasonIsBad,           // out: SM_ERR_OUTSIDE_OF_DOMAIN    = input UVTestPoint is not within domain or
                                          //                                    input UVTestPoint is on surface natural boundary and
                                          //                                        vector from SurfacePoint to TestPoint runs outside of surface domain
                                          //      SM_ERR_BAD_SURFACE_POINT    = Surface.Evaluate() fails - testing impossible
                                          //      SM_ERR_NOT_WITHIN_TOLERANCE = test run - Point not on normal               
                                          //      SM_ERR_UNKNOWN              = test run - Point is on normal                
                  dDropToSurf,            // out: when rbGoodPoint == TRUE, set to SurfPt/TestPt            
                                          //           rbGoodPoint == FALSE, set to 0.0                            
                  dApproxDev) ;           // out: deviation of CrvPt from SrfNormalLine
      
      // when smsurf_IsPointOnNormal - it's not a good point
      if ( eStat != SM_SUCCESS )
        {
          bIsGoodPoint = FALSE;
        }

      // When any point is bad - whole span fails check
      if( !bIsGoodPoint)
        {
          rbSpanPassesTest = FALSE;
          break ;
        }
      else // - save drop statistics
        {
          if(dDropToSurf > rdMaxDropToSurf) { rdMaxDropToSurf = dDropToSurf ;
                                              rdMaxDropToSurfParam = crStart.m_dT + dNormalizedT*dDeltaT ;
                                            }
          if(dApproxDev > rdMaxApproxDev)   { rdMaxApproxDev = dApproxDev ; }
        }
    } // end iter every test point

#ifdef SM_DEBUG_CODE
  if(bDebugMe)
    {
      crStart.Dump(sApproxTol3d, _T("Start DropPt")) ;
      rEnd.Dump(sApproxTol3d, _T("End DropPt")) ;
    }
#endif // SM_DEBUG_CODE

  // all done
  return SM_SUCCESS;

} // end SmDropPt::TestSpanAccuracy

/*******************************************************************//**
PURPOSE: DropCurve helper function. 
    Find : SpanEnd DropPoint while respecting seams and poles
    Given: Span StartDropPoint and SpanEnd sEnd.m_dT guess value 

NOTES:
  return status:
    SM_SUCCESS                  = output values set
    SM_ERR                      = a called method (LocalPointSolve, GlobalPointSolve, DropVectors, Crv and Srf Evaluates) failed, or
                                  or CurveTangent is perp to Surface at SrfDropPoint.
    SM_ERR_OUTSIDE_OF_DOMAIN    = SpanStartPt drops to boundary and heads out of domain
    SM_ERR_BAD_SURFACE_POINT    = SpanStartPt drops outside of UVDomain or Curve is crossing (not running along) a seam
    SM_ERR_NOT_WITHIN_TOLERANCE = Drop not along surface normal
                                   usually caused by dropping a CrvPt not over the surface to surface boundary.
    SM_ERR_BAD_TANGENT_DROP     = CrvTangent drop produced a non Parallel SrfTangent direction

METHOD ---
  1. To reduce noise - adjust offset distance for rOffset to run rOffset through the CrvSpanStartPt

  2. Get SrfSpanEndUV guess from 2nd order Taylor step.
     2.a If step is too big (near poles) set guess = SrfSpanStartUV
  3. drop CrvSpanEndPt to surface (getting aSrfSpanEndUV[0]) using LocalPointSolve(SM_SO_MINIMIZE)

#ifdef SM_BDRY_CHECK
  4. if dropped span crosses seam - shorten span to seam in sm_CheckPushingBoundary
     4a. adjust rOffset distance so that rOffset runs through the CrvSpanEndPt
     4b. Drop sCrvSpanEnd 1stDeriv to rOffset (getting aSrfSpanEndUV[1] using rOffset.DropVectors()
     4c. temporarily Set eRet = SM_ERR_OUTSIDE_OF_DOMAIN (gets lost before getting to step 5)
#endif // SM_BDRY_CHECK

  5. Test LocalPointSolve DropPoint
     5a. If dropping to pole, set SrfSpanStartUV[0].DegenParameter
     5b. if a seam-side correction is needed, move the endpoint only within sApproxTol3d
         5b1. preserve endpoints that cannot be corrected within tolerance
         5b2. return SM_ERR_OUTSIDE_OF_DOMAIN so the caller can retry another candidate
     5c. check CrvSpanEndPt is within tol of SrfSpanEnd m_SrfNormal with smsurf_IsPointOnNormal
         5c1. When SrfSpanEnd point is no good, set eRet = eReasonIsBad from smsurf_IsPointOnNormal.
 
  6. if test fails 
     6a. Get GlobalPointSolve DropPoint
     6b. rerun all DropPoint tests from last step.
 
  7. If SpanEnd DropPoint passes tests - compute SrfSpanEnd 1stDeriv
     7a. adjust rOffset distance so that rOffset runs through CrvSpanEndPt
     7b. Drop sCrvSpanEnd 1stDeriv to rOffset (getting aSrfSpanEndUV[1]] useing rOffset.DropVectors()
     7c. If dropping to pole, set SrfSpanStartUV[1].DegenCoord to 0.0
***********************************************************************/
SmStatus SmDropPt::GetNextPoint                // rtn: (1000) SM_SUCCESS              (1020) SM_ERR_OUTSIDE_OF_DOMAIN
                                               //      (1001) SM_ERR                  (1021) SM_ERR_NOT_WITHIN_TOLERANCE  
                                               //      (1032) SM_ERR_BAD_TANGENT_DROP (1030) SM_ERR_BAD_SURFACE_POINT
  (const SmSurface           & crSurface,      // in : main surface
   SmOffsetSurface           & rOffset,        // i/o: offset of main surface, its offset distance is set in this routine
   ULONG                       lSingularities, // in : orof: SM_SS_NONE, SM_SS_UMIN, SM_SS_VMIN, SM_SS_UMAX, SM_SS_VMAX
   const SmTArray<SmPoint3d> * sSrfPolePoints, // in : TODO
   const SmExtent2d          & crUVDomain,     // in : limiting domain of the surface
   SmSurfParamType             eSrfClosure,    // in : oneof: SM_SP_U, SM_SP_V, SM_SP_BOTH, SM_SP_NEITHER
   const SmCurve             & cr3dCurve,      // in : Curve being dropped
   SmApproxTol3d               sApproxTol3d,   // in : max allowed distance between drop point and surfNormal line at drop point,
   double                      dUVTolX,        // NotUsed: in : UV Tolerance in the u direction
   double                      dUVTolY,        // NotUsed: in : UV Tolerance in the v direction
   double                      dEndT,          // in : param value of SpanEnd CurvePt being dropped
   SmDropPt                  & rEnd)           // in : crStart.m_dT, crStart.m_CrvPD, crStart.m_bFromLeftU, crStart.m_bFromLeftV
                                               // out: rEnd.m_dT
                                               //      rEnd.m_CrvPD, 
                                               //      rEnd.m_UVPD, rEnd.m_bFromLeftU, rEnd.m_bFromLeftV,
                                               //      rEnd.m_3dPD
                                               //      rEnd.m_dDropToSurf,     rEnd.m_dApproxDev
                                               //      rEnd.m_dNormalToCrvAngDeg
                                               //      rEnd.m_dSrfToCrvAngDeg, rEnd.m_dSrfToCrvSpeedRatio
                                               //      rEnd.m_bOnBoundary,     rEnd.m_bLeavingDomain,  rEnd.m_bEnteringDomain
                                               //      rEnd.m_eDropDerivRtn    rEnd.m_bOnSeam,         rEnd.m_bOnPole
                                               //         SM_SUCCESS = good drop, 
                                               //         SM_ERR_NOT_WITHIN_TOLERANCE = CrvPt not with tol of SrfNormalLine at dropPoint
                                               //                usually caused by dropping a CrvPt not over the Srf to a SrfBoundary
                                               //         SM_ERR_OUTSIDE_OF_DOMAIN = span leaves the domain or crosses a seam beyond snap tolerance
                                               //         SM_ERR_BAD_TANGENT_DROP  = CrvTangent dropped to nonParallel SrfTangent
                                               //                usually caused by dropping a CrvTangent near a srfPole              
{
  SM_REF2(dUVTolX, dUVTolY) ;
  // locals
  SmDropPt &rStart = *this ;

  // init return values
  SmStatus eRet            = SM_ERR ;
  rEnd.SetUninitialized() ;
  rEnd.m_dT                = dEndT ;    
  rEnd.m_bDegenCurve       = rStart.m_bDegenCurve ;

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  if(bDebugMe)
    {
      rStart.Dump(sApproxTol3d, _T("Start DropPt")) ;
      rEnd.Dump(sApproxTol3d, _T("End DropPt")) ;
    }
#endif // SM_DEBUG_CODE

  // UDir tolerance
  SmExtent1d sIntervalU   = crUVDomain.GetUInterval() ;
  SmExtent1d sIntervalV   = crUVDomain.GetVInterval();
  double     dScaledZeroU = SM_EFF_ZERO * (1.0 + sIntervalU.GetLength()) ;
  double     dScaledZeroV = SM_EFF_ZERO * (1.0 + sIntervalV.GetLength()) ;

  // quit when rStart.m_UVPD[0] is not in UVDomain
  if(!crUVDomain.ContainsPoint2d(rStart.m_UVPD[0], smos_Max(dScaledZeroU, dScaledZeroV)))
    {
      return (SM_ERR_BAD_SURFACE_POINT) ;
    }

  // quit when rStart is on boundary and DropCurve is walking out of domain
  if(rStart.m_bOnBoundary && rStart.m_bLeavingDomain)
    {
      return (SM_ERR_OUTSIDE_OF_DOMAIN) ;
    }
 
  // arrive here when Span is not leaving the domain after starting on the UVBdry

  // get StartPoint Surf DU, DV, DUU, DVV, DUV values
  SmVector3d sSrfPt, sSu, sSv, sSuu, sSuv, sSvv;
  crSurface.Evaluate2ndDerivatives( rStart.m_UVPD[0], 
                                    rStart.m_bFromLeftU, rStart.m_bFromLeftV,        
                                    sSrfPt,              
                                    sSu, sSv, sSuv, sSuu, sSvv) ; // NonZeroTangent values

  // LocalPointSolve locals: Find SpanEnd[UV] from local drop of SpanEnd[CrvPt]
  ULONG ii, jj ;
  SmBoolean    bFoundAnswer;
  SmSolution   sSolution;
  SmSolution & rSolution = sSolution;
  SmSolution   sData[4];
  SmSolutionArray sSolutions(4,sData);

  // Get LocalPointSolve GuessSpanEnd[UV] point. 

  // Guess SpanEnd[UV] with quadratic Taylor step 
  //   (linear Taylor gives bad guess near quickly changing SrfDerivs common at poles [090726])
  // using SpanStart[m_dT UV UV1stDeriv UV2ndDeriv] and SpanEnd[m_dT]

  // with:                Ctt                   = Suu*Ut^2 + 2*Suv*Ut*Vt + Svv*Vt^2  +  Su*Utt  +  Sv*Vtt
  // solve for UVtt from: Su*UVtt.x + Sv*UVtt.y = Ctt - [ Suu*Ut^2 + 2*Suv*Ut*Vt + Svv*Vt^2 ]
  // then                 GuessSpanEnd[UV]      = SpanStart[UV] + deltaT*SpanStart[UVt] + 1/2 * deltaT^2*SpanStart[UVtt].

  SmVector2d sUVtt(0,0); // 2nd deriv of uv w.r.t. t (as would be in rStart.m_UVPD[2]).
  double     dUt = rStart.m_UVPD[1].x; // just renaming...
  double     dVt = rStart.m_UVPD[1].y;

  // solve for UVtt from: Su*UVtt.x + Sv*UVtt.y = Ctt - [ Suu*Ut^2 + 2*Suv*Ut*Vt + Svv*Vt^2 ]
  SmVector3d s2ndDerivQuantity = sSuu * dUt*dUt + 2*sSuv*dUt*dVt + sSvv * dVt*dVt;
  SmVector3d sRHS              = rStart.m_CrvPD[2] - s2ndDerivQuantity;
  smgu_VecLinCombTwoVectors( sSu, sSv, sRHS, sUVtt );

  // calc GuessSpanEnd[UV] with Taylor step from SpanStart[UV UV1stDeriv UV2ndDeriv]
  double dDeltaT  = rEnd.m_dT - rStart.m_dT;
  rEnd.m_UVPD[0]  = rStart.m_UVPD[0] + dDeltaT * rStart.m_UVPD[1];  // first-order Taylor
  rEnd.m_UVPD[0] += sUVtt * ( dDeltaT * dDeltaT / 2.0 );  // add second-order Taylor

  // Avoid bad SpanEnd[UV] guesses (common near poles with rapidly changing Surf Deriv values).
  // When next guess is obviously wrong - use SpanStart[UV] to start the local solve,  GWC: [B187]
  SmVector2d sDeltaUV = rEnd.m_UVPD[0] - rStart.m_UVPD[0] ;
  SmVector2d sUVSize  = crUVDomain.GetSize() ;
  if(   smos_Fabs(sDeltaUV.x) > 1.5 * sUVSize.x
     || smos_Fabs(sDeltaUV.y) > 1.5 * sUVSize.y)
    {
      rEnd.m_UVPD[0] = rStart.m_UVPD[0] ;
    }

  // clamp GuessSpanEnd[UV]  // GWC: should Curve/Bndry angle be checked so walking seams can clamp and crossing seams can trim?
  rEnd.m_UVPD[0] = crUVDomain.ClampPoint2d( rEnd.m_UVPD[0] );

  // get SpanEnd[CrvPt Crv1stDeriv]
  SER( cr3dCurve.Evaluate(rEnd.m_dT, 2, FALSE, rEnd.m_CrvPD)) ;    // nonZeroTangent values

#ifdef SM_DEBUG_CODE
  if (bDebugMe)
    {
      cr3dCurve.Dump() ;
      rEnd.Dump(sApproxTol3d, _T("End DropPt")) ;
      if(FALSE)
        { smgfx_Erase() ; }
      sm_GraphicsLoop();
      smgfx_SetLook(1,2, 1,1,0); crSurface.DrawUV(2,2,FALSE,NULL,TRUE);  sm_GraphicsLoop();
      smgfx_SetLook(2,3, 0,1,1); cr3dCurve.Draw(NULL, TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,4, 1,0,0); rStart.m_CrvPD[0].Draw(); rStart.m_CrvPD[1].Draw(&rStart.m_CrvPD[0]); sm_GraphicsLoop();
      smgfx_SetLook(1,5, 0,1,0); rEnd.m_CrvPD[0].Draw(); rEnd.m_CrvPD[1].Draw(&rEnd.m_CrvPD[0]); sm_GraphicsLoop();
      smgfx_SetLook(1,7, 0,0,1); crSurface.DrawAt(rStart.m_UVPD[0],0); sm_GraphicsLoop();
      smgfx_SetLook(1,9, 1,0,1); crSurface.DrawAt(rEnd.m_UVPD[0],0);  sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

  // find SpanEnd[UV] by dropping SpanEnd[CrvPt] to Surface starting iter at GuessSpanEnd[UV].
  //      note: cannot use DropPoint() or DropPointFast(), because they have
  //            different behaviors for seams, out of bounds, etc.
  //      note: changed operation from NORMALIZE to MINIMIZE to guarantee even out-of-bounds
  //            crv pts would always drop to surface pts. [bd 29 Jan 09]
  SER( crSurface.LocalPointSolve( crUVDomain,           // in : surface domain of interest
                                  SM_SO_MINIMIZE,       // in : operation (was SM_SO_NORMALIZE)
                                  rEnd.m_CrvPD[0],      // in : 3dPoint being dropped
                                  rEnd.m_UVPD[0],       // in : guess UVPoint
                                  bFoundAnswer,         // out: TRUE = found answer, else FALSE
                                  sSolution ));         // out: contains the surface point cloest to the 3dPoint

  // MINIMIZE should always find some solution.
  if ( !bFoundAnswer )
    { SER( SM_ERR ); }

  // load SpanEnd[UV] from DropPoint(SpandEnd[CrvPt]) solution
  rEnd.m_UVPD[0].x = sSolution.m_vStart[0];
  rEnd.m_UVPD[0].y = sSolution.m_vStart[1];

#ifdef SM_DEBUG_CODE
  if (bDebugMe)
    {
      SmVector3d sPrevPD[2]; // sPrevPD[0] = cr3dCurve 3dPoint    for param = rStart.m_dT
                             // sPrevPD[1] = cr3dCurve 3dTangent  for param = rStart.m_dT
      SER(cr3dCurve.Evaluate(rStart.m_dT,1,TRUE,sPrevPD));

      if(FALSE)
        { smgfx_Erase() ; }
      sm_GraphicsLoop();
      smgfx_SetLook(2,3, 0,1,1); cr3dCurve.Draw(NULL, TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 1,1,0); crSurface.DrawUV(2,2,FALSE,NULL,TRUE);  sm_GraphicsLoop();
      smgfx_SetLook(1,4, 1,0,0); rStart.Draw(); sm_GraphicsLoop();
      smgfx_SetLook(1,5, 0,1,0); rEnd.Draw(); sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

  // next: clean up and verify SpanEnd[UV]
  //         1. when SpanEnd[UV] is on singularity - set degen param to match span end tangent direction
  //         2. when span crosses a seam, move SpanEnd[UV] back to seam
  //         3. verify that SpanEnd[CrvPt] is within tol of the surface normal coming from crSurface(SpanEnd[UV]).

  // for 2 iterations - find eRet value, oneof: SM_SUCCESS (good point)                                                    
  //                                            SM_ERR_NOT_WITHIN_TOLERANCE (CrvPoint not within tol of SrfPoint m_SrfNormal)
  //                                            SM_ERR_OUTSIDE_OF_DOMAIN (Seam correction is bad )                         
  //  on iter1 - clean up and test current SpanEnd[UV] found with LocalPointSolve:
  //             if that fails - goto iter2
  //  on iter2 - recompute SpanEnd[UV] using GlobalPointSolve - clean up and test again.
  //             if that fails - set eRtn with error
  for(ii=0; ii<2 && eRet!=SM_SUCCESS; ii++)
    {
      // if iter 2: (only run after rejecting LocalPointSolve DropPoint) - try again with GlobalPointSolve DropPoint
      if(ii == 1)   
        {
          //SmSurface *pSurf = SM_CONST_CAST(SmSurface*,&crSurface);

          // temporarily set owner to null to prevent solver from seeing Face edges.
          SmTemporaryChangeValue<SmObject*> sTON( ((SmSurface &)crSurface).m_pOwner, NULL );

          // drop sCrvSpanEndPt to surface with GlobalPointSolve (not LocalPointSolve)
          SER(crSurface.GlobalPointSolve  // eff: search surface for points whose normal vectors aim at the target point  
                      (crUVDomain,        // in : Domain of surface to search for solutions                               
                       SM_SO_MINIMIZE,    // in : oneof: SM_SO_MINIMIZE, SM_SO_MAXIMIZE, SM_SO_NORMALIZE, SM_SO_INTERSECT 
                       rEnd.m_CrvPD[0],   // in : Target point for the solve operation                                    
                       SM_EFF_ZERO,       // in :                                                                         
                       NULL,              // in : Max/Min Drop distance for min/max and normalize operations.             
                                          //      NULL to ignore.                                                         
                       SM_SR_ALL,         // in : SM_SR_SINGLE=get best solution, SM_SR_ALL=get all solutions             
                       sSolutions )) ;    // out: array of problem solutions reported as surface UV parameter values      

          // save best solution (closest to orig parameterSpace estimate into rSolution
          double dSmallDiff = 0.0;
          for(jj=0;jj<sSolutions.GetSize();jj++)
            {
              SmSolution & rTmpSol = sSolutions[jj] ;

              // distance between this solution and orig parameter estimate
              double dDiff =  smos_Fabs( rTmpSol.m_vStart[0] - rEnd.m_UVPD[0].x )
                            + smos_Fabs( rTmpSol.m_vStart[1] - rEnd.m_UVPD[0].y ) ;
              
              // save best solution
              if(   (jj == 0)
                 || (   dDiff < dSmallDiff
                     && rTmpSol.m_vStart.m_dSolutionValue < sApproxTol3d ))
                {
                  dSmallDiff = dDiff;
                  rSolution  = rTmpSol;
                }
            } // end iter every GlobalPointSolve solution - looking for best solution

          // quit when Global Drop was no different than local drop - already have all values
          SmVector2d sGlobalDropUV(rSolution.m_vStart[0], rSolution.m_vStart[1]) ;
          if(rEnd.m_UVPD[0].CloserThan(SM_EFF_ZERO, sGlobalDropUV))
            { break ; }

        } // end 2nd iter check - Try GlobalPointSolve DropPoint solution

      // set SpanEnd[UV] = drop SpanEnd[CrvPt] to Surface (1st with LocalPointSolve soln, 2nd with GlobalPointSolve soln)
      rEnd.m_UVPD[0].Set(rSolution.m_vStart[0], rSolution.m_vStart[1]) ;

      // When SpanEnd[UV] is on a pole - adjust it's singular param value  [061001]
      SmSurfParamType eSingularDirection ;
      ULONG lSingularities2 = sSrfPolePoints != nullptr ? crSurface.IsSingularity(rEnd.m_UVPD[0], lSingularities, sSrfPolePoints, eSingularDirection, sApproxTol3d) : crSurface.IsSingularity(rEnd.m_UVPD[0], eSingularDirection, sApproxTol3d);
      // SM_ASSERT(lSingularities2 == lSingularities);
      if(lSingularities2 != 0)
        {
          crSurface.FindDegenParamForDirection(rEnd.m_UVPD[0],      // i/o: a point on the surface singularity,                                        
                                                                    //      when 2 sols are possible (seam), the sol closest to initial rUV pt is used 
                                               rEnd.m_CrvPD[1],     // in : the 3d direction to match with the NonSingular Surface tangent                  
                                               FALSE,               // in : TRUE  = Dir to match is from the pole heading out, as in starting a curve
                                                                    //      FALSE = Dir to match ends at the pole heading in, as in ending a curve                     
                                               eSingularDirection,  // in : oneof SM_SP_U: Surf(si,v) == Surf(sj,v) where si, sj are any valid u values
                                                                    //            SM_SP_V: Surf(u,si) == Surf(u,sj) where si, sj are any valid v values
                                               0.0001) ;            // in : a loose tolerance; will try for a tight one.                                    
                                               // crUVDomain ) ; // in : SubDomain of Surf to be tested                                 
        }

//cbi: Arrive here with aSrfSpanStartUV = SpanStart[UV]
//                      rEnd.m_UVPD   = guess for localPtSolve
//                      rEnd.m_UVPD   = 1st iter: solution for LocalPtSolve  drop of EndSpan[CrvPt] to Surface
//                                        2nd iter: solution for GlobalPtSolve drop of EndSpan[CrvPt] to Surface
//  on 1st pass, rEnd.m_UVPD should be close to rEnd.m_UVPD and
//     should not have jumped seam but might not be a Normalized solution.
//  on 2nd pass, rEnd.m_UVPD will be a Normalized solution but may have jumped seam.
//
// It's still probably worth doing the GlobalSolve ... well, maybe not anymore?
//    Now that LocalSolve uses MINIMIZE, and finds a good bdry point...
//    But: the GlobalSolve is indeed hit in prog_test.
// If we didn't do GlobalSolve, then wouldn't jump seams.
//
// : rethink this whole loop.

      // Correct an ambiguous seam-side choice only within the 3D tolerance.
      // A larger correction rejects this candidate without moving the endpoint.
      eRet = rStart.CheckSeamJump(&crSurface, eSrfClosure, sApproxTol3d, rEnd);
      if (eRet != SM_SUCCESS)
        { return eRet; }

      // Historical domain-crossing heuristic (unused).
      // obsolete
      //      if(eSrfClosure != SM_SP_NEITHER)
      //        { SmVector2d sSpanStartJumpUV = rEnd.m_UVPD - rStart.m_UVPD[0];
      //          SmVector2d sSpanEndJumpUV   = rEnd.m_UVPD - rEnd.m_UVPD[0];
      //      
      //          // when jump point is more than 2/3 of domain from both span start and end - it's likely the curve has gone out of bounds
      //          SmBoolean bLargeSpanDiffU = (   smos_Fabs( sSpanStartJumpUV.x ) > crUVDomain.XLength() / 1.5     
      //                                       && smos_Fabs( sSpanEndJumpUV.x )   > crUVDomain.XLength() / 1.5 ) ;
      //          SmBoolean bLargeSpanDiffV = (   smos_Fabs( sSpanStartJumpUV.y ) > crUVDomain.YLength() / 1.5   
      //                                       && smos_Fabs( sSpanEndJumpUV.y )   > crUVDomain.YLength() / 1.5 ) ;
      //          SmBoolean bAfterGlobalSolve = (ii == 1) ;
      //          if(   (bLargeSpanDiffU && ( bAfterGlobalSolve || eSrfClosure == SM_SP_U || eSrfClosure == SM_SP_BOTH))
      //             || (bLargeSpanDiffV && ( bAfterGlobalSolve || eSrfClosure == SM_SP_V || eSrfClosure == SM_SP_BOTH)))
      //            {
      //              // GWC: this branch is never hit in prog_test - but sm_CheckSeamJump still makes some suspicious edits.
      //              eRet = SM_ERR_OUTSIDE_OF_DOMAIN;
      //            }
      //        }

      //      // if SpanEnd[UV] looks OK - check SpanEnd[CrvPt] is within tol of SpanEnd[Srfpt] SurfaceNormal
      //      if(eRet == SM_SUCCESS)
      //        {
      //          SmStatus  eReasonIsBad;
      //      
      //          // 3dTol - when endSpan is within 5 deg of moving along domain boundary - increase tolerance.
      //          //         Happens often in practice, and data can be sloppy.  [091026]
      //          //         GWC: why not increase the tolerance for any Span that ends on a boundare?
      //          double dTempTol =  (SM_POC_ON_BOUNDARY == rEnd.IsMovingAlongBoundary( crSurface, crUVDomain, dUVTolX, dUVTolY))
      //                            ? 5.0 * sApproxTol3d
      //                            : sApproxTol3d ;
      //      
      //          // test SpanEnd[CrvPt/SurfaceNormal dist] against dTempTol
      //          SmVector3d sCrvEndFromDir = -rEnd.m_CrvPD[1] ;
      //          SmStatus eStat = smsurf_IsPointOnNormal
      //                    ( crSurface,            // in : target surface                                                   
      //                      NULL,                 // in : clamp domain: if present, clamp crUVToTest to this domain.       
      //                                            //      NULL to ignore                                                   
      //                      rEnd.m_UVPD[0],       // in : Surface test 2Dpoint                                             
      //                      rEnd.m_CrvPD[0],      // in : input   test 3DPoint  
      //                     &sCrvEndFromDir,       // in : associated 3d tangent  
      //                      rEnd.m_bFromLeftU,    // out: TRUE=eval srf u from left at rEnd.m_UVPD[0], else FALSE
      //                      rEnd.m_bFromLeftV,    // out: TRUE=eval srf v from left at rEnd.m_UVPD[0], else FALSE                                        
      //                      dTempTol,             // in : Max dist allowed between unique points                           
      //                      rEnd.m_bGoodDrop,     // out: TRUE=TestPoint within tol of SurfaceNormal line, FALSE=not       
      //                      eReasonIsBad,         // out: SM_ERR_OUTSIDE_OF_DOMAIN    = input UVTestPoint is not within domain or
      //                                            //                                    input UVTestPoint is on surface natural boundary and
      //                                            //                                        vector from SurfacePoint to TestPoint runs outside of surface domain
      //                                            //      SM_ERR_BAD_SURFACE_POINT    = Surface.Evaluate() fails - testing impossible
      //                                            //      SM_ERR_NOT_WITHIN_TOLERANCE = test run - Point not on normal               
      //                                            //      SM_ERR_UNKNOWN              = test run - Point is on normal                
      //                      rEnd.m_dDropToSurf,   // out: when rbGoodPoint == TRUE, set to SurfPt/TestPt dist           
      //                                            //           rbGoodPoint == FALSE, set to 0.0                            
      //                      rEnd.m_dApproxDev) ;  // out: deviation of CrvPt from SrfNormalLine
      //      
      //          rEnd.m_bGoodDrop = (eStat == SM_SUCCESS) ;
      //          if( !rEnd.m_bGoodDrop ) { eRet = eReasonIsBad;  }
      //      
      //        } // end SpanEnd CrvPoint is on SrfPoint m_SrfNormal check

#ifdef SM_DEBUG_CODE
      if(bDebugMe)
        {
          rStart.Dump(sApproxTol3d, _T("Start DropPt")) ;
          rEnd.Dump(sApproxTol3d, _T("End DropPt")) ;
        }
#endif // SM_DEBUG_CODE

      // next: get SpanEnd[OtherValues] from SpanEnd[CrvPD and UV[0]]
      //   method: Negate SpanEnd[Crv1stDeriv] so that vector looks into Span (forces correct FromLeftU and FromLeftV computations)
      //           use SmDropPt::SetProperties() to get SpanEnd[-UV1stDeriv] 
      //           Negate SpanEnd[Crv1stDeriv and UV1stDeriv] to make them positive again. (Hermite cubic definitions) 
      rEnd.m_CrvPD[1] = -rEnd.m_CrvPD[1] ;
      eRet = rEnd.SetProperties(crSurface,      // in : target drop surface                                                                                                              
                                rOffset,        // i/o: Offset from crSurface, its offset dist gets modified to minimize tolerances                                                     
                                lSingularities, // in : orof: SM_SS_NONE, SM_SS_UMIN, SM_SS_VMIN, SM_SS_UMAX, SM_SS_VMAX
                                sApproxTol3d,   // in : min dist between distinct 3d points                                                                                              
                                crUVDomain,     // in : Domain of interest for crSurface                                                                                                 
                                FALSE) ;        // in : don't run FindDegenParamForDirection, that's already been done
                                                // in : m_UVPD[0]  = drop pCurve(dT) position to surface 2d point (Solution of GlobalPointSolve for dT) 
                                                //      m_CrvPD[0] = position pCurve(dT)
                                                //      m_CrvPD[1] = 1stDeriv pCurve(dT)
                                                //      m_CrvPD[2] = 2ndDeriv pCurve(dT)
                                                // out: m_UVPD[1],             Proj of m_CrvPD[1] to Surface
                                                //      m_bGoodDrop,           TRUE = m_dApproxDev < sApproxTol3d
                                                //      m_3dPD[0],             Proj of m_UVPD[0] through Surface back to 3d
                                                //      m_3dPD[1],             Proj of m_UVPD[1] through Surface back to 3d
                                                //      m_SrfNormal,           SurfaceNormal at DropPoint
                                                //      m_bFromLeftU,          TRUE=Dropped Srf 1stDeriv starts in pos U direction, FALSE=in neg U direction
                                                //      m_bFromLeftV,          TRUE=Dropped Srf 1stDeriv starts in pos V direction, FALSE=in neg V direction
                                                //      m_dDropToSurfDrop                                                         
                                                //      m_dApproxDev,          deviation of m_CrvPD[0] from SrfNormalLine starting at Surface(sSrfUV[0])
                                                //      m_dNormalToCrvAngDeg,  angleDeg between m_CrvPD[1] and SrfPlane normal, should be near 90.0
                                                //      m_dSrfToCrvAngDeg,     angleDeg between m_CrvPD[1] and SurfaceProjection of sSrfUV[1], should be near 0.0
                                                //      m_dSrfToCrvSpeedRatio, m_3dPD[1].Length() / m_CrvPD[1], should be near 1.0 
                                                //      m_bOnBoundary, 
                                                //      m_bOnSeam,
                                                //      m_bOnPole,
                                                //      m_bLeavingDomain, 
                                                //      m_bEnteringDomain
                                                //      m_eDropDerivRtn        SM_SUCCESS = good drop,
                                                //                             SM_ERR_BAD_TANGENT_DROP = CrvTangent drop produced a non Parallel SrfTangent direction
                                                //                             SM_ERR_NOT_WITHIN_TOLERANCE = Drop not along surface normal
                                                //                                     usually caused by dropping a CrvPt not over the surface to surface boundary.

       // when Properties are set - negate the tangent direction a second time
       if ( eRet != SM_ERR )  // 1001 = SM_ERR = zero length derivatives (Avoid uninitialized vector complaints.)
                              // 1000 = SM_SUCCESS                  = all values initialized
                              // 1032 = SM_ERR_BAD_TANGENT_DROP     = all values initialized
                              // 1021 = SM_ERR_NOT_WITHIN_TOLERANCE = all values initialized
         {
           rEnd.m_CrvPD[1] = -rEnd.m_CrvPD[1] ;
           rEnd.m_UVPD[1]  = -rEnd.m_UVPD[1] ;
           // gwc: why not rEnd.m_3dPD[1] as well?
           if(rEnd.m_bOnBoundary)
             {
               SM_ASSERT(   rEnd.m_bLeavingDomain != rEnd.m_bEnteringDomain
                         || (rEnd.m_bLeavingDomain == FALSE && rEnd.m_bEnteringDomain == FALSE)) ;

               // negate entering/leaving when leaving or entering domain - leave alone when running along domain 
               if     (rEnd.m_bLeavingDomain)  { SM_ASSERT(rEnd.m_bEnteringDomain == FALSE) ;
                                                 rEnd.m_bLeavingDomain   = FALSE ;
                                                 rEnd.m_bEnteringDomain  = TRUE ;
                                               }
               else if(rEnd.m_bEnteringDomain) { SM_ASSERT(rEnd.m_bLeavingDomain == FALSE) ;
                                                 rEnd.m_bLeavingDomain   = TRUE ;
                                                 rEnd.m_bEnteringDomain  = FALSE ;
                                               }
            } // end need to negate Leaving/Entering domain values
         } // end initialized DropPt values check

    } // end LocalPointSolve/GlobalPointSolve iter to test DropPoint solutions setting eRet value

#ifdef SM_DEBUG_CODE
  if(bDebugMe)
    {
      rStart.Dump(sApproxTol3d, _T("Start DropPt")) ;
      rEnd.Dump(sApproxTol3d, _T("End DropPt")) ;
    }
#endif // SM_DEBUG_CODE

  // all done
  return eRet;

} // end SmDropPt::GetNextPoint

/*******************************************************************//**
PURPOSE: DropCurve helper function.
       Find: all DropPoint properties
       Given: a CurvePoint Dropped to a SurfaceUVPoint

NOTES:
 Make sure the this SmDropPt object's m_UVPD[0], m_CrvPD[0], and m_CrvPD[1]
 are set prior to this call.  All other SmDropPt Object member values are set here.

 When SurfaceUVPoint lands on a surface pole, make sure to call SetProperties() 
 with bAdustPolePt == TRUE or make sure to call FindDegenParamForDirection() 
 to set the SurfaceUVPoint's degenerate parameter value prior to making this call.  

RETURNS --- 
  (1000) SM_SUCCESS                  = when the drop method yields a good approx to Crv1stDeriv
  (1001) SM_ERR                      = When CurveTangent, sCrvPD[1], is perp to surface at the surface drop point, sSrfUV[0]
  (1032) SM_ERR_BAD_TANGENT_DROP     = when approx is bad, commonly due to dropping near a pole
                                       where large 2nd deriviatives make this approximation poor
  (1021) SM_ERR_NOT_WITHIN_TOLERANCE = When CrvPt is not on SrfNormalLine at SrfDropPoint.  Usually due to
                                       dropping a point not over the surface and drop point is pulled
                                       to nearest boundary point.
***********************************************************************/
SmStatus SmDropPt::SetProperties
  (const SmSurface  &crSurface,      // in : target drop surface
   SmOffsetSurface  &rOffset,        // i/o: Offset from crSurface, it's offset dist gets modified to minimize tolerances
   ULONG             lSingularities, // in : orof: SM_SS_NONE, SM_SS_UMIN, SM_SS_VMIN, SM_SS_UMAX, SM_SS_VMAX
   SmApproxTol3d     sApproxTol3d,   // in : min dist between distinct 3d points
   const SmExtent2d &crUVDomain,     // in : Domain of interest for crSurface
   SmBoolean         bAdjustPolePt)  // in : TRUE  = adjust m_UVPD[0] with FindDegenParamForDirection when m_UVPD[0] is on pole
                                     //      FALSE = don't
                                     // in : m_UVPD[0]  = drop pCurve(dT) position to surface 2d point (Solution of GlobalPointSolve for dT) 
                                     //      m_CrvPD[0] = position pCurve(dT),
                                     //      m_CrvPD[1] = 1stDeriv pCurve(dT),
                                     //      m_CrvPD[2] = 2ndDeriv pCurve(dT),
                                     // out: m_UVPD[1],             Proj of m_CrvPD[1] to Surface
                                     //      m_bGoodDrop,           TRUE = m_dApproxDev < sApproxTol3d
                                     //      m_3dPD[0],             Proj of m_UVPD[0] through Surface back to 3d
                                     //      m_3dPD[1],             Proj of m_UVPD[1] through Surface back to 3d
                                     //      m_SrfNormal,           SurfaceNormal at DropPoint
                                     //      m_bFromLeftU,          TRUE=Dropped Srf 1stDeriv starts in pos U direction, FALSE=in neg U direction
                                     //      m_bFromLeftV,          TRUE=Dropped Srf 1stDeriv starts in pos V direction, FALSE=in neg V direction
                                     //      m_dDropToSurfDrop                                                         
                                     //      m_dApproxDev,          deviation of m_CrvPD[0] from SrfNormalLine starting at Surface(sSrfUV[0])
                                     //      m_dNormalToCrvAngDeg,  angleDeg between m_CrvPD[1] and SrfPlane normal, should be near 90.0
                                     //      m_dSrfToCrvAngDeg,     angleDeg between m_CrvPD[1] and SurfaceProjection of sSrfUV[1], should be near 0.0
                                     //      m_dSrfToCrvSpeedRatio, m_3dPD[1].Length() / m_CrvPD[1], should be near 1.0 
                                     //      m_bOnBoundary,
                                     //      m_bOnSeam,
                                     //      m_bOnPole,
                                     //      m_bLeavingDomain,
                                     //      m_bEnteringDomain,
                                     //      m_eDropDerivRtn,       SM_SUCCESS = good drop,
                                     //                             SM_ERR_BAD_TANGENT_DROP = CrvTangent drop produced a non Parallel SrfTangent direction
                                     //                             SM_ERR_NOT_WITHIN_TOLERANCE = Drop not along surface normal
                                     //                                     usually caused by dropping a CrvPt not over the surface to surface boundary.
{
  // locals
  SmPoint3d sSrfDU, sSrfDV ;

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  if(bDebugMe)
    {
      Dump(sApproxTol3d, _T("Target DropPt")) ;
    }
#endif // SM_DEBUG_CODE

  // param is on a pole - change it's UV pos so that the Crv1stDeriv lines up with the sole SrfTangent
  // note: when UVPD[0] is on a surface pole 
  //       FindDegenParamForDirection()
  SmSurfParamType eSingularDirection = SM_SP_U;
  m_bOnPole =    (lSingularities != SM_SS_NONE) 
              && (crSurface.IsSingularity(m_UVPD[0],          // in : Surface UVPoint to test
                                          eSingularDirection, // out: SM_SP_U = crUVToTest is within tol of a degenerate V isoParameter Curve.
                                                              //                where Surf  (crUVToTest.x, VMinOrMax) = const, 
                                                              //                      SurfDU(crUVToTest.x, VMinOrMax) = 0.0
                                                              //      SM_SP_V = crUVToTest is within tol of a degenerate U isoParameter Curve.
                                                              //                where Surf  (UMinOrMax, crUVToTest.y) = const, 
                                                              //                      SurfDV(UMinOrMax, crUVToTest.y) = 0.0 
                                                              //      SM_SP_BOTH = surface is singular in both directions at crUVToTest.
                                                              //      SM_SP_NEITHER = surface is not singular at crUVToTest.
                                          sApproxTol3d)) ;    // in : min 3d distance between distinct points,
                                                              //      default:[SM_EFF_ZERO] 
  if(m_bOnPole  && m_bDegenCurve != TRUE)
    {
      if(bAdjustPolePt)
        {
           // find degenParam value at a singularity that matches the dropCurve UV direction
           crSurface.FindDegenParamForDirection
                             (m_UVPD[0],           // i/o: a point on the surface singularity moved along degenerate UV Boundary.                                        
                                                   //      when 2 sols are possible (seam), the sol closest to initial rUV pt is used 
                              m_CrvPD[1],          // in : the 3d direction to match with the NonSingular Surface tangent              
                              TRUE,                // in : TRUE  = Dir to match is from the pole heading out, as in starting a curve
                                                   //      FALSE = Dir to match ends at the pole heading in, as in ending a curve                 
                              eSingularDirection,  // in : oneof SM_SP_U: Surf(si,v) == Surf(sj,v) where si, sj are any valid u values
                                                   //            SM_SP_V: Surf(u,si) == Surf(u,sj) where si, sj are any valid v values
                              0.0001) ;            // in : a loose tolerance; will try for a tight one.                                
                              // sUVDomain ) ;     // in : SubDomain of Surf to be tested               
        } // end m_CrvPD[1] is NonZero branch
    } // end OnPool is TRUE check

  // Surface pos and Srf U and V Derivs
  SER( crSurface.Evaluate1stDerivatives(  m_UVPD[0], TRUE, TRUE, m_3dPD[0], sSrfDU, sSrfDV )) ;

  // set m_bFromLeftU and m_bFromLeftV 
  m_bFromLeftU = (sSrfDU.Dot(m_CrvPD[1]) >= 0.0) ; // TRUE=Use upper interval, FALSE = lower interval
  m_bFromLeftV = (sSrfDV.Dot(m_CrvPD[1]) >= 0.0) ; // TRUE=Use upper interval, FALSE = lower interval

  // reevaluate when on knot boundaries as needed
  if(m_bFromLeftU != TRUE || m_bFromLeftV != TRUE)
    {
      SER( crSurface.Evaluate1stDerivatives(  m_UVPD[0], m_bFromLeftU, m_bFromLeftV, m_3dPD[0], sSrfDU, sSrfDV )) ;
    }

  // SurfaceNormal at DropPoint
  SER( crSurface.EvaluateNormal( m_UVPD[0], m_bFromLeftU, m_bFromLeftV, m_SrfNormal )) ;
  SmVector3d sStartGap( m_CrvPD[0] - m_3dPD[0] ) ;

#ifdef SM_DEBUG_CODE
  SM_ASSERT_BREAK_MSG(   sSrfDU.IsZero(1000 * SmTol::GetScaledZero()) 
                      || m_SrfNormal.IsPerpendicularTo(sSrfDU), _T("SmDropPt::SetProperties : found a Surface Drop Point where SurfaceNormal is not perp to SurfaceTangent - case needs review")) ;
  SM_ASSERT_BREAK_MSG(   sSrfDV.IsZero(1000 * SmTol::GetScaledZero()) 
                      || m_SrfNormal.IsPerpendicularTo(sSrfDV), _T("SmDropPt::SetProperties : found a Surface Drop Point where SurfaceNormal is not perp to SurfaceTangent - case needs review")) ;
#endif // SM_DEBUG_CODE

  // UDir tolerance
  SmExtent1d   sIntervalU   = crUVDomain.GetUInterval() ;
  SmExtent1d   sIntervalV   = crUVDomain.GetVInterval();
  SmZoneTol3d  sZoneTol3d   = SmTol::GetZoneTol3d(&crSurface) ;
  SmVector2d   sUDir(1,0) ;
  SmVector2d   sVDir(0,1) ; 
  SmTol2d      sZoneTol2d_U = SmTol::MapTo2d(sZoneTol3d, m_UVPD[0], sUDir, crSurface) ;
  SmTol2d      sZoneTol2d_V = SmTol::MapTo2d(sZoneTol3d, m_UVPD[0], sVDir, crSurface) ;
  // SmScaledZero sScaledZeroU = SmTol::GetScaledZero(sIntervalU) ;
  // SmScaledZero sScaledZeroV = SmTol::GetScaledZero(sIntervalV) ;
  double       dLimitRad    = SM_DEG2RAD(30.0) ;
  //      double     dCosineLimit = 0.5 ; // Allow 30 degrees.

  // Classify DropPoint against UVDomain
  //   Note: this criterion is not the stepping for a surface-point solve criteria. 
  //           SurfaceSteppingSolve Criteria: When the direction of the step is 
  //             leaning outside of the surface even a little bit, 
  //             Then the point is presumed off the surface, 
  //         but here we're following a curve that could wander in and out.
  //   cf. [SMS16] [061001]

  m_bOnBoundary     = FALSE ;
  m_bLeavingDomain  = FALSE ;
  m_bEnteringDomain = FALSE ;
  m_bOnSeam         = FALSE ;

  // When on U Boundary - set DropPoint Boundary State, [m_bOnBoundary, m_bOnSeam, m_bLeavingDomain, m_bEnteringDomain] 
  if(   m_UVPD[0].x < sIntervalU.GetMin() + sZoneTol2d_U  // + sScaledZeroU                          
     || m_UVPD[0].x > sIntervalU.GetMax() - sZoneTol2d_U) // - sScaledZeroU
    {
      // remember on boundary
      m_bOnBoundary  = TRUE ;
      m_bOnSeam     |= crSurface.IsOnSeam(m_UVPD[0], &sApproxTol3d) ; 

      // Crv and Srf 1st derivative lengths
      double dSrfLenSq = sSrfDU.LengthSquared();
      double dCrvLenSq = m_CrvPD[1].LengthSquared();

      // when 1st derivs are nonZero (when m_bDegenCurve == TRUE, m_CrvPD[1] is 0.0 and this check should fail)
      if(  !(m_bOnPole && (lSingularities & (SM_SS_VMIN | SM_SS_VMAX))) 
         && (dSrfLenSq * dCrvLenSq > SM_EFF_ZERO_SQ ) )
        {
          // when curve is walking out of the surface by more than AngleLimit (30 degrees) - return SM_ERR
          
          // old check - assume dU and dV are perpendicular then
          //             the dot product of CrvTangent and and dU will be zero.
          //             This can break down when dU and dV are not perpendicular.
          // new check - Going out of the domain when the angle between the 
          //             dV (the vector running along the surface boundary)
          //             and crvTangent is more than about 30 degrees outside of the
          //             domain.
          // old check
          //      // get crv/srf 1st deriv dot product
          //      double dDot = sSrfDU.Dot(m_CrvPD[1] ) / smos_Sqrt( dSrfLenSq * dCrvLenSq );
          //      
          //      // negate dot product for starting boundary
          //      if(m_UVPD[0].x < sIntervalU.GetMin() + dScaledZeroU)
          //        { dDot = -dDot; }
          //      
          //      if      (dDot > dCosineLimit)  { m_bLeavingDomain  = TRUE ; }
          //      else if( dDot < -dCosineLimit) { m_bEnteringDomain = TRUE ; }
          
          // new check
          double dCCWRad ;
          m_SrfNormal.CCWAngleBetween(sSrfDV, m_CrvPD[1], dCCWRad) ; 

          if ( dCCWRad < 0 ) // This returns [-Pi, Pi]. [B260]
            { dCCWRad += 2*SM_PI; }
          
          // when on min boundary (dU points into surf) 
          if(m_UVPD[0].x < sIntervalU.GetMin() + sZoneTol2d_U) // + sScaledZeroU) 
            { 
              m_bLeavingDomain  = SM_IS_BETWEEN(dCCWRad, dLimitRad, SM_PI - dLimitRad) ;
              m_bEnteringDomain = SM_IS_BETWEEN(dCCWRad, SM_PI + dLimitRad, 2.0* SM_PI - dLimitRad) ;
            }
          else // on max boundary (dU points out of surf)
            {
              m_bLeavingDomain  = SM_IS_BETWEEN(dCCWRad, SM_PI + dLimitRad, 2.0* SM_PI - dLimitRad) ;
              m_bEnteringDomain = SM_IS_BETWEEN(dCCWRad, dLimitRad, SM_PI - dLimitRad) ; 
            }
        } // end nonZero 1st Deriv check
    } // end PrevUVPoint is on UMin/UMax boundary - need to set [m_bOnBoundary, m_bOnSeam, m_bLeavingDomain, m_bEnteringDomain] check

  // When On V Boundary - set DropPoint Boundary State, [m_bOnBoundary, m_bOnSeam, m_bLeavingDomain, m_bEnteringDomain]
  if(   (m_UVPD[0].y < sIntervalV.GetMin() + sZoneTol2d_V)   // + sScaledZeroV
     || (m_UVPD[0].y > sIntervalV.GetMax() - sZoneTol2d_V))  // - sScaledZeroV
    {
      // remember on boundary
      m_bOnBoundary  = TRUE ;
      m_bOnSeam     |= crSurface.IsOnSeam(m_UVPD[0],&sApproxTol3d) ; 

      // Crv and Srf 1st derivative lengths
      double dSrfLenSq = sSrfDV.LengthSquared();
      double dCrvLenSq = m_CrvPD[1].LengthSquared();
      
      // when 1st derivs are nonZero (when m_bDegenCurve == TRUE, m_CrvPD[1] is 0.0 and this check should fail)
      if(  !(m_bOnPole && (lSingularities & (SM_SS_UMIN | SM_SS_UMAX))) 
         && (dSrfLenSq * dCrvLenSq > SM_EFF_ZERO_SQ) )
        {
          // when curve is walking out of the surface by more than AngleLimit (30 degrees) - return SM_ERR
          
          // old check - assume dU and dV are perpendicular then
          //             the dot product of CrvTangent and and dU will be zero.
          //             This can break down when dU and dV are not perpendicular.
          // new check - Going out of the domain when the angle between the 
          //             dV (the vector running along the surface boundary)
          //             and crvTangent is more than about 30 degrees outside of the
          //             domain.
          // old check
          //      // get crv/srf 1st deriv dot product
          //      double dDot = sSrfDV.Dot( m_CrvPD[1] ) / smos_Sqrt( dSrfLenSq * dCrvLenSq );
          //      
          //      // negate dot product for starting boundary
          //      if(m_UVPD[0].y < sIntervalV.GetMin() + dScaledZeroV)
          //        { dDot = -dDot; }
          //      
          //      // when curve is walking out of the surface by more than CosineLimit (30 degrees) - return SM_ERR
          //      if     (dDot > dCosineLimit)  { m_bLeavingDomain  = TRUE ; }
          //      else if(dDot < -dCosineLimit) { m_bEnteringDomain = TRUE ; }

          // new check
          double dCCWRad ;
          m_SrfNormal.CCWAngleBetween(sSrfDU, m_CrvPD[1], dCCWRad) ; 

          if ( dCCWRad < 0 ) // This returns [-Pi, Pi]. [B260]
            { dCCWRad += 2*SM_PI; }
          
          // when on min boundary (dV points into surf) 
          if(m_UVPD[0].y < sIntervalV.GetMin() + sZoneTol2d_V)  // sScaledZeroV) 
            { 
              m_bLeavingDomain  = SM_IS_BETWEEN(dCCWRad, SM_PI + dLimitRad, 2.0* SM_PI - dLimitRad) ;
              m_bEnteringDomain = SM_IS_BETWEEN(dCCWRad, dLimitRad, SM_PI - dLimitRad) ;             
            }                     
          else // on max boundary (dV points out of surf)
            {
              m_bLeavingDomain  = SM_IS_BETWEEN(dCCWRad, dLimitRad, SM_PI - dLimitRad) ;             
              m_bEnteringDomain = SM_IS_BETWEEN(dCCWRad, SM_PI + dLimitRad, 2.0* SM_PI - dLimitRad) ;
            }                     

        } // end nonZero 1st Deriv check
    } // end PrevUVPoint is on VMin/VMax boundary - need to set [m_bOnBoundary, m_bOnSeam, m_bLeavingDomain, m_bEnteringDomain] check

  // save the curve/Surface drop distance and deviation (distance from SurfNormalLine to CurvePoint)
  double dDropParam ;
  smgu_LinePointDistance(m_3dPD[0], m_SrfNormal, m_CrvPD[0], m_dApproxDev, &dDropParam) ;
  m_dDropToSurf = dDropParam;      // was:   sStartGap.Length()   [B488]

  // A drop is good if the original point being dropped is within a tolerant shape of the SrfNormalLine 
  //   which is defined to be a cylinder of ApproxTol radius up to one unit from the surface and
  //   then a cone whose radius increases for larger drop distances.
  double dScaledTol = (m_dDropToSurf < 1.0 ? 1.0 : m_dDropToSurf) * sApproxTol3d ;
  m_bGoodDrop       = m_dApproxDev < dScaledTol ;

  // when the Drop is not good and not on the boundary
  if(   m_bOnBoundary == FALSE 
     && m_bGoodDrop   == FALSE) // only possible when GlobalPointSolve fails to work properly
    {
      // note: GlobalPointSolve returns a surface drop point within tol of the actual surface drop point.
      //       The gap between the srfNormalLine from this SrfPoint to the original 3d point being dropped
      //       will vary depending on the curvature of the surface.  For planes that
      //       gap will be equal to the drop tolerance.  When dropping to a concave surface
      //       from a distance less than the radius of curvature, the gap actually gets smaller, and when
      //       dropping to concave surfaces the gap gets larger.  Adjust the tolerance
      //       to reflect the geometry of the surface. For small angles this scaling of the tol
      //       can be approximated from the relation,
      //          ApproxTol/RadiusOfCurvature = MaxGapAtOrigPoint/(OrigPoint-CenterOfCurvature).Length
      //       See if the drop is good allowing for a larger tolerance due to surface curvature.

      // when drop deviation is huge - don't bother with this check
      if(m_dApproxDev < 1000 * dScaledTol)
        {
          // get the surface principle curvatures
          double      sdGaussianCurvature = 0.0;       
          double      sdNormalCurvature = 0.0;
          double      sdPrincipleCurvature1 = 0.0;
          double      sdPrincipleCurvature2 = 0.0;
          SmVector3d  sEFGOfFirstFundForm ;       
          SmVector3d  sLMNOfSecondFundForm ;      
          SmVector3d  sPrincipleCurvatureVector1 ;
          SmVector3d  sPrincipleCurvatureVector2 ;
          crSurface.EvaluateGeometric(m_UVPD[0], m_bFromLeftU, m_bFromLeftV,
                                      sdGaussianCurvature,       
                                      sdNormalCurvature,         
                                      sdPrincipleCurvature1,     
                                      sdPrincipleCurvature2,     
                                      sEFGOfFirstFundForm,       
                                      sLMNOfSecondFundForm,      
                                      sPrincipleCurvatureVector1,
                                      sPrincipleCurvatureVector2) ;

         // center of curvature for both principle curvatures
         SmPoint3d sCenter1 = m_3dPD[0] + m_SrfNormal * sdPrincipleCurvature1 ;
         SmPoint3d sCenter2 = m_3dPD[0] + m_SrfNormal * sdPrincipleCurvature2 ;
         double dPtToCenter1 = (sCenter1 - m_CrvPD[0]).Length() ;
         double dPtToCenter2 = (sCenter2 - m_CrvPD[0]).Length() ;

         // get max scaling due to surface curvature
         double dCurvScaling = smos_3Max(smos_Fabs(sdPrincipleCurvature1) * dPtToCenter1,
                                         smos_Fabs(sdPrincipleCurvature2) * dPtToCenter2,
                                         5.0) ;

         // reevaluate the quality of the drop
          m_bGoodDrop  = m_dApproxDev < dCurvScaling * sApproxTol3d ;
       } // end small enough deviation check to justify a curvature based GoodDrop evaluation

#ifdef SM_DEBUG_CODE
      if(m_bGoodDrop == FALSE)
        {
          // room for a break point - GlobalPointSolve failed to find a good answer, look into this
          // m_bGoodDrop = m_bGoodDrop ;
        }
#endif // SM_DEBUG_CODE

    } // end drop was not good

  // can't drop zero length Crv1stDeriv vectors when m_bDegenCurve is FALSE (Crv1stDerivs should be NonZero)
  double dCrvSpeed = m_CrvPD[1].Length() ;
  if(dCrvSpeed < SM_EFF_ZERO && m_bDegenCurve == FALSE)
    { if(m_bGoodDrop) // inform the public
        { m_bGoodDrop = FALSE ;
          SER_MSG(SM_ERR, _T("DropCurve: attempted to drop zero length CurveTangent to Surface")) ; 
        }
      else // skip informing the public when the drop is already known to be bad
        { return(SM_ERR) ; }
    }

  // when m_bDegenCurve != TRUE
  if(m_bDegenCurve != TRUE)
    {
      // can't drop Crv1stDeriv vectors parallel to m_SrfNormal (degenerate case)
      double dSrfNormCrvTanAngRad ;
      m_SrfNormal.AngleBetween(m_CrvPD[1], dSrfNormCrvTanAngRad) ;
      if(SM_RAD2DEG(dSrfNormCrvTanAngRad) < 3.0)
        { if(m_bGoodDrop)
            { m_bGoodDrop = FALSE ;
              SER_MSG(SM_ERR, _T("DropCurve: attempted to drop CurveTangent parallel to SurfaceNormal")) ; 
            }
          else // skip informing the public when the drop is already known to be bad
            { return(SM_ERR) ; }
        }
    } // end only NonDegenCurve should have NonZero Crv1stDeriv checks

  // Set rOffset.OffsetDist = crv/srf startSpan gap size in m_SrfNormal direction 
  double dOffsetDist = sStartGap.Dot( m_SrfNormal ) ;
  rOffset.SetOffsetDistance( dOffsetDist ) ;

  // Get SrfSpanStart 1stDeriv - drop CrvSpanStart 1stDeriv to OffsetSurface TangentPlane at m_UVPD (For NonDegenCurve)
  if(m_bDegenCurve == FALSE)
    {
      m_UVPD[0] = crUVDomain.ClampPoint2d( m_UVPD[0]) ; // note: perhaps no longer needed. [061001]
      SER( rOffset.DropVectors( m_UVPD[0], m_bFromLeftU, m_bFromLeftV, 1,     
                                &m_CrvPD[1],     // in : Crv Start 1stDeriv              
                                &m_UVPD[1])) ;   // out: rOffset UVTrimCurve 1stDeriv              
    } // end NonDegenCurve check

  // when DropPoint is on a pole - set DropPoint[UV 1stDeriv] Degen Param to 0 - SpanStart[UV] degen param already set
  if(m_bOnPole)
    {
      // make sure eSingularDirection is set
      SM_ASSERT(eSingularDirection != SM_SP_NEITHER) ;

      // Just set undef param to 0: no 'sideways' movement at singularity.
      if ( eSingularDirection == SM_SP_U )
        { m_UVPD[1].x = 0.0; }  // note: m_UVPD[0] pts starting at surface poles have 
      else                               //       already been adjusted by FindDegenParamForDirection()      
        { m_UVPD[1].y = 0.0; }  //       so that this property will be TRUE.
    }

  // arrive here after vector has been dropped to surface (except for m_bDegenCurve == TRUE cases)
  //  report the quality of the drop - compare the surface directions tangent with the curve tangent
  //   1. the Curve1stDeriv, SurfaceDirectionalstDeriv and the SurfaceNormal have to be coplanar
  //   2. the Curve1stDeriv and SurfaceDirectional1stDeriv have to point in the same general direction

  if(m_bDegenCurve != TRUE)
  {
    // SurfaceDirectional1stDeriv - sOffsetPD evaluated for a unitized m_UVPD[1]
    SmPoint3d sOffsetPD[2], sCrvProj1stDeriv;

    if(!m_UVPD[1].IsZero())
    {
      rOffset.EvaluateDirectionalDerivs( m_UVPD[0], m_UVPD[1], 1, sOffsetPD, m_bFromLeftU, m_bFromLeftV );
      double dUVSpeed = m_UVPD[1].Length();
      sOffsetPD[1] = dUVSpeed * sOffsetPD[1];

      // project Crv1stDir to plane perp to SurfaceNormal vector
      sCrvProj1stDeriv = m_CrvPD[1].ProjectToPlane( m_SrfNormal );

      // angle between Crv and SrfNormal 
      double dNormalToCrvAngRad = 0.0;
      m_CrvPD[1].AngleBetween( m_SrfNormal, dNormalToCrvAngRad );
      m_dNormalToCrvAngDeg = SM_RAD2DEG( dNormalToCrvAngRad );

      // angle between Crv and Srf 1stDerivs
      double dOffsetToCrvAngRad = 0.0;
      sCrvProj1stDeriv.AngleBetween( sOffsetPD[1], dOffsetToCrvAngRad );
      m_dSrfToCrvAngDeg = SM_RAD2DEG( dOffsetToCrvAngRad );

      // size ratios
      double dSrfSpeed = sOffsetPD[1].Length();
      double dProjCrvSpeed = sCrvProj1stDeriv.Length();
      m_dSrfToCrvSpeedRatio = (dProjCrvSpeed > SM_EFF_ZERO) ? dSrfSpeed / dProjCrvSpeed
        : (dSrfSpeed < SM_EFF_ZERO) ? 1.0
        : 10000;

      // now project the UV vector to 3d through the surface
      m_3dPD[1] = sSrfDU * m_UVPD[1].x + sSrfDV * m_UVPD[1].y;

    } // end if !isZero vector

    m_eDropDerivRtn = IsGoodTangentDrop( m_bOnPole )
      ? SM_SUCCESS
      : SM_ERR_BAD_TANGENT_DROP;

#ifdef SM_DEBUG_CODE                                                         
    if(bDebugMe)
    { // draw surf at SrfSpanStart - for large offsets also draw rOffset
      smgfx_Erase();
      smgfx_SetLook( 1, 2, 0, 1, 1 ); crSurface.DrawUV( 4, 4, FALSE, NULL, TRUE ); sm_GraphicsLoop();
      smgfx_SetLook( 2, 4, 0, 0, 1 ); crSurface.DrawAt( m_UVPD[0], 1 ); sm_GraphicsLoop();
      smgfx_SetLook( 2, 4, 0, 1, 1 ); if(smos_Fabs( dOffsetDist ) > 0.01) rOffset.DrawAt( m_UVPD[0], 1 ); sm_GraphicsLoop();
      smgfx_SetLook( 2, 3, 0, 1, 0 ); m_CrvPD[1].Draw( &m_CrvPD[0] ); sm_GraphicsLoop();
      smgfx_SetLook( 3, 4, 0, 1, .5 ); sCrvProj1stDeriv.Draw( &sOffsetPD[0] ); sm_GraphicsLoop();
      smgfx_SetLook( 4, 5, 1, 0, 0 ); sOffsetPD[1].Draw( &sOffsetPD[0] ); sm_GraphicsLoop();

      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

  } // end m_bDegenCurve != TRUE check

#ifdef SM_DEBUG_CODE
  if(bDebugMe)
    {
      Dump(sApproxTol3d, _T("Target DropPt")) ;
    }
#endif // SM_DEBUG_CODE
  
  // all done
  return(  (m_bGoodDrop == FALSE) 
         ?  SM_ERR_NOT_WITHIN_TOLERANCE 
         :  m_eDropDerivRtn ) ;

} // end SmDropPt::SetProperties

/*******************************************************************//**
PURPOSE: Correct a dropped endpoint that selected the opposite side of a seam.

         The transverse step comparison identifies a possible seam jump. Snap
         the endpoint only if the proposed correction moves its surface point
         by no more than sApproxTol3d. Larger corrections leave the endpoint
         unchanged and reject this candidate span, allowing DropCurve to try
         another seam-side start instead of accepting a short, invalid UV wrap.

RETURNS: SM_SUCCESS = no detected jump, or corrected within sApproxTol3d.
         SM_ERR_OUTSIDE_OF_DOMAIN = possible seam crossing that cannot be snapped
                                    within tolerance; retry another candidate.
         Other errors indicate a failed surface evaluation.

NOTES: A large UV jump can be a genuine seam crossing or an incorrect seam-side
       choice at the start of the span. It does not establish invalid geometry,
       and this helper does not split curves at seam crossings.
***********************************************************************/
SmStatus SmDropPt::CheckSeamJump             // in : uses: m_UVPD[0], m_bFromLeftU, m_bFromLeftV
                                             //      assumes: other thisDropPt values not yet set. 
 (const SmSurface  * cpSurface,              // in : Surface to which Curve is being dropped                                       
  SmSurfParamType    eSrfClosure,            // in : one of SM_SP_U, SM_SP_V, SM_SP_NEITHER, SM_SP_BOTH                            
  SmApproxTol3d      sApproxTol3d,           // in : maximum 3D displacement allowed by a seam correction
  SmDropPt         & rEnd)                   // i/o: in : rEnd.m_UVPD[0]
                                             //      out: seam-side correction only when within sApproxTol3d
 const
{
  SmStatus sRtn = SM_SUCCESS ; 

  // no work - not a closed surface
  if ( eSrfClosure == SM_SP_NEITHER )  
    { return( sRtn ) ; }

  // locals
  const SmDropPt & crStart   = *this ;
  SmExtent2d       sUVDomain = cpSurface->GetNaturalUVDomain();

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  if(bDebugMe)
    {
      crStart.Dump(sApproxTol3d, _T("Start DropPt")) ;
      rEnd.Dump(sApproxTol3d, _T("End DropPt")) ;
    }
#endif // SM_DEBUG_CODE

  // GWC thoughts: Between any two points in a UVDomain there is just one line segment that
  //   connects them directly (without crossing any seam boundaries).
  //   Once one boundary (or two) is (are) made periodic there become an infinite number
  //   of line segments that connect any two points in a UVDomain.  Usually of
  //   interest are the minimal length segments that just cross a seam once.
  //   For a single periodic boundary there are 2 such curves and for a UVDomain with
  //   two periodic boundaries there are 4 line segments that pass both seams just once
  //   and 4 more line segments that pass just one seam once and the other seam not at all.
  // In this function only two chord line segments are considered at a time,
  // the direct connection and the single seam crossing case moving in the opposite direction.
  //   GWC: I'm not sure this is complete but it probably handles most cases.
  //   GWC: This implementation handles stepping across two torus seam boundaries
  //        by moving the endDropPt back to the corner of the UVDomain.
  //        This only makes sense to me if the curve was dropping to a seam
  //         and was meant to end at the corner and the drop function happened
  //         to step across both seams at the same time.  A very special case.
  // A picture to help think about chords on a periodic domain
  //          A periodic UV Domain repeated to represent the effect of periodic boundaries
  //           @ - start point in the UVDomain
  //           # - end point in the UVDomain repeated to show various seam crossing 
  //                  line segs that connect the two
  //                                      .                          |
  //                                      .        |                 |
  //           |                 |        .        |                 |
  //         --+-----------------+-----------------+-----------------+--
  //           |              #  |              #  |              #  |
  //           |               \ |             /   |                 |
  //           |                \|           /     |                 |   a few possible single period chords shown
  //           |                 \         /       |                 |    (some omitted to keep the picture simpler)
  //        ---+-----------------+\-------/--------+-----------------+----------
  //           |              #~~| \    /    ...#  |     ,,,,,,,,,#  |
  //    . . .  |                 ~~ \ /   ...   ,,,,,,,,,            |  . . .
  //           |                 | ~~@...,,,,,,,   |                 |
  //           |                 |    ^^^          |                 |
  //         --+-----------------+--------^^-------+-----------------+----------------
  //        #  |              #  |           ^^^#  |              #  |              #
  //           |                 |                 |                 |
  //           |                 |                 |                 |
  //           |                 |                 |                 |
  //     ------+-----------------+-----------------+-----------------+-----------------+--
  //        #  |              #  |         .    #  |              #  |              #  |
  //                             |         .       |                 |
  //                                       .                         |

  // GWC: I leave the above picture in place because it shows things that might
  //      be considered by this function in some future life and it took a lot of time
  //      to draw.  Right now I think that this implementation is not for the above case but
  //      rather the single case of an endPt dropping to a seam while having problems
  //      knowing which side of the seam should be targeted.
  //
  // old note: This can be tricky to test for, because steps are not always small.
  // For a small step, a seam jump would be characterized by a parameter
  // step that is very large and in the wrong direction.
  // We'll compare the actual 3d step to what would be expected from
  // the parameter shift and the surface derivative.
  // Also, we will restrict ourselves to fairly large parameter-space steps,
  // because on some surfaces, for small, legitimate steps, these tests
  // can be fooled if the step is sort of along the seam direction,
  // even out in the middle of the surface.

  // Check u-direction when srf is closed in U and span length > 1/2 DomainLength 
  //  GWC: this compares an estimated span step in 3d against
  //        the direct "..." and the seam crossing '~~~' chords shown above.
  SmExtent1d sUDomain = sUVDomain.GetUInterval();
  if(   (   eSrfClosure == SM_SP_U 
         || eSrfClosure == SM_SP_BOTH )
     && smos_Fabs( rEnd.m_UVPD[0].x - crStart.m_UVPD[0].x ) > sUDomain.GetLength() / 2.0)
    {
      // sStart.m_UVPD => sSrfSpanStart3d point and 1stDeriv
      SmPoint3d  sSrfSpanStart3d;
      SmVector3d sSrfStartSu, sSrfStartSv;
      SER(cpSurface->Evaluate1stDerivatives( crStart.m_UVPD[0], crStart.m_bFromLeftU, crStart.m_bFromLeftV, sSrfSpanStart3d, sSrfStartSu, sSrfStartSv ));

      // rEnd.m_UVPD[0] => sSrfSpanEnd3d point
      SmPoint3d sSrfSpanEnd3d;
      SER(cpSurface->EvaluatePoint( rEnd.m_UVPD[0], sSrfSpanEnd3d ));
      SmVector3d sStep3d( sSrfSpanEnd3d - sSrfSpanStart3d );

      // Compare the steps perpendicular to the U seam so motion along Sv
      // cannot mask a reversed U step. Crossing both steps with Sv removes
      // their parallel components and applies the same rotation and scale.
      // This retains the normal component of the actual chord on curved surfaces.
      // The scale cancels in the ratio; a zero Sv gives a zero comparison.
      SmVector3d sAcrossStep3d = sStep3d * sSrfStartSv;
      double dDeltaU = rEnd.m_UVPD[0].x - crStart.m_UVPD[0].x;
      SmVector3d sAcrossExpectedStep3d = dDeltaU * (sSrfStartSu * sSrfStartSv);
      double dNumer = sAcrossStep3d.Dot(sAcrossExpectedStep3d);
      double dDenom = sAcrossStep3d.Dot(sAcrossStep3d);

      // The expected/actual ratio is negative for a reversed seam step.
      // Compare without division so zero transverse motion does not trigger a jump.

      // when span jumps seam - snap back to domain
      //  GWC: shouldn't this find the intersection between the span and the seamCurve?
      //        not if the jump is only intended for UVPoints within tol of the seam boundary
      if ( dNumer < dDenom * -2.0 )
        {
          // A seam-side correction must preserve the dropped point in 3D.
          // Leave interior points alone; the span may genuinely cross the seam.
          SmPoint2d sSnappedUV(rEnd.m_UVPD[0]);
          sSnappedUV.x = rEnd.m_UVPD[0].x > sUDomain.GetMid()
                           ? sUDomain.GetMin() : sUDomain.GetMax();
          SmPoint3d sSnappedPoint;
          SER(cpSurface->EvaluatePoint(sSnappedUV, sSnappedPoint));
          if (sSnappedPoint.DistanceBetween(sSrfSpanEnd3d) <= sApproxTol3d)
            { rEnd.m_UVPD[0] = sSnappedUV; }
          else
            { return SM_ERR_OUTSIDE_OF_DOMAIN; }
        }
    } // end u-direction check.

  // Repeat for v-direction

  // Check v-direction when srf is closed in V and span length > 1/2 DomainLength 
  SmExtent1d sVDomain = sUVDomain.GetVInterval();
  if(   (   eSrfClosure == SM_SP_V 
         || eSrfClosure == SM_SP_BOTH )
     && smos_Fabs( rEnd.m_UVPD[0].y - crStart.m_UVPD[0].y ) > sVDomain.GetLength() / 2.0)
    {
      // sSrfStartUV => sSrfStart3d point and 1stDeriv
      SmPoint3d  sSrfSpanStart3d;
      SmVector3d sSrfStartSu, sSrfStartSv;
      SER(cpSurface->Evaluate1stDerivatives( crStart.m_UVPD[0], crStart.m_bFromLeftU, crStart.m_bFromLeftV, sSrfSpanStart3d, sSrfStartSu, sSrfStartSv ));

      // rEnd.m_UVPD[0] => sSrfSpanEnd3d point
      SmPoint3d sSrfSpanEnd3d;
      SER(cpSurface->EvaluatePoint( rEnd.m_UVPD[0], sSrfSpanEnd3d ));
      SmVector3d sStep3d( sSrfSpanEnd3d - sSrfSpanStart3d );

      // Compare only motion across the V seam, removing the Su direction.
      SmVector3d sAcrossStep3d = sStep3d * sSrfStartSu;
      double dDeltaV = rEnd.m_UVPD[0].y - crStart.m_UVPD[0].y;
      SmVector3d sAcrossExpectedStep3d = dDeltaV * (sSrfStartSv * sSrfStartSu);
      double dNumer = sAcrossStep3d.Dot(sAcrossExpectedStep3d);
      double dDenom = sAcrossStep3d.Dot(sAcrossStep3d);

      // The expected/actual ratio is negative for a reversed seam step.
      // Compare without division so zero transverse motion does not trigger a jump.

      // when span jumps seam - snap back to domain
      //  GWC: shouldn't this find the intersection between the span and the seamCurve?
      if ( dNumer < dDenom * -2.0 )
        {
          // A seam-side correction must preserve the dropped point in 3D.
          // Leave interior points alone; the span may genuinely cross the seam.
          SmPoint2d sSnappedUV(rEnd.m_UVPD[0]);
          sSnappedUV.y = rEnd.m_UVPD[0].y > sVDomain.GetMid()
                           ? sVDomain.GetMin() : sVDomain.GetMax();
          SmPoint3d sSnappedPoint;
          SER(cpSurface->EvaluatePoint(sSnappedUV, sSnappedPoint));
          if (sSnappedPoint.DistanceBetween(sSrfSpanEnd3d) <= sApproxTol3d)
            { rEnd.m_UVPD[0] = sSnappedUV; }
          else
            { return SM_ERR_OUTSIDE_OF_DOMAIN; }
        }
    } // end v-direction check.

#ifdef SM_DEBUG_CODE
  if(bDebugMe)
    {
      crStart.Dump(sApproxTol3d, _T("Start DropPt")) ;
      rEnd.Dump(sApproxTol3d, _T("End DropPt")) ;
    }
#endif // SM_DEBUG_CODE

  // all done
  return( sRtn ) ; 

} // end SmDropPt::CheckSeamJump

/*******************************************************************//**
PURPOSE: DropCurve helper function.
  Adjust DropPoint tangent directions making up for DropCurve 
  algorithm evaluation noise to prevent a curve dropping to a surface 
  natural boundary from weaving on and off of that boundary.

NOTES:
  For SpanStart and SpanEnd drop UVpts that drop to a surface domain boundary, 
  set the associated SpanStart and SpanEnd 1stDeriv vector coordinates to 0.0 
  so that the DropCurve walk algorithm won't wonder off the current boundary.

  If both points are on both a U and a V boundaries, both coordinates of 
  the dropUVTangent vector is set to zero.
   
  This is called before sm_TestSpanAccuracy(), and sm_AddToCurve().

  Occasionally DropCurve is asked to drop very short segments - only
  Adjust Drop Points when the interval base cord is mostly parallel
  to the target domain boundary.
***********************************************************************/
SmStatus SmDropPt::FixBoundaryTangents    // in : m_UVPD[0], m_UVPD[1]
                                          // out: m_UVPD[1]: When on bdry, set m_UVPD[1].param = 0
  (const SmExtent2d & crUVDomain,         // in : Domain range
   SmDropPt         & rEnd,               // i/o: rEnd.m_UVPD[0]=current dropped point, 
                                          //      rEnd.m_UVPD[1]=current drop direction
                                          //      When on bdry, set rEnd.m_UVPD[1].param = 0
   double             dUVTolX,            // in : UV tol in the u dir
   double             dUVTolY,            // in : UV tol in the v dir
   SmApproxTol3d      sApproxTol3d,       // in : not currently used in this function
   SmBoolean        & rbFixBTChange)      // out: TRUE = changed a value, FALSE = didn't
{
  // init output
  rbFixBTChange = FALSE ;

  // locals
  SmDropPt &rStart = *this ;
  double dUStep = smos_Fabs(rEnd.m_UVPD[0].x - rStart.m_UVPD[0].x) ;
  double dVStep = smos_Fabs(rEnd.m_UVPD[0].y - rStart.m_UVPD[0].y) ;

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  if(bDebugMe)
    {
      rStart.Dump(sApproxTol3d, _T("Start DropPt")) ;
      rEnd.Dump(sApproxTol3d, _T("End DropPt")) ;
    }
#else
  SM_REF1(sApproxTol3d);
#endif // SM_DEBUG_CODE

  // SpanStart and SpanEnd DropUVPts on UMin bdry - moving in V dir
  if(   SM_ARE_SAME_TO_TOL(rStart.m_UVPD[0].x,crUVDomain.GetMin().x, dUVTolX) 
     && SM_ARE_SAME_TO_TOL(rEnd.m_UVPD[0].x,  crUVDomain.GetMin().x, dUVTolX)
     && dVStep > dUStep ) 
    {
      // remove any DropUVDir motion in the u direction
      rStart.m_UVPD[1].x = 0.0 ;
      rEnd.m_UVPD[1].x   = 0.0 ;
      rbFixBTChange      = TRUE ;
    }
  
  // SpanStart and SpanEnd DropUVPts on UMax bdry - moving in V dir
  if(   SM_ARE_SAME_TO_TOL(rStart.m_UVPD[0].x,crUVDomain.GetMax().x, dUVTolX) 
     && SM_ARE_SAME_TO_TOL(rEnd.m_UVPD[0].x,  crUVDomain.GetMax().x, dUVTolX)
     && dVStep > dUStep ) 
    {
      // remove any DropUVDir motion in the u direction
      rStart.m_UVPD[1].x = 0.0 ;
      rEnd.m_UVPD[1].x   = 0.0 ;
      rbFixBTChange      = TRUE ;
    }
  
  // SpanStart and SpanEnd DropUVPts on VMin bdry - moving in U dir
  if(   SM_ARE_SAME_TO_TOL(rStart.m_UVPD[0].y,crUVDomain.GetMin().y, dUVTolY)
     && SM_ARE_SAME_TO_TOL(rEnd.m_UVPD[0].y,  crUVDomain.GetMin().y, dUVTolY)
     && dUStep > dVStep ) 
    {
      // remove any DropUVDir motion in the v direction
      rStart.m_UVPD[1].y = 0.0 ;
      rEnd.m_UVPD[1].y   = 0.0 ;
      rbFixBTChange      = TRUE ;
    }
  
  // SpanStart and SpanEnd DropUVPts on VMax bdry - moving in U dir
  if(   SM_ARE_SAME_TO_TOL(rStart.m_UVPD[0].y,crUVDomain.GetMax().y, dUVTolY) 
     && SM_ARE_SAME_TO_TOL(rEnd.m_UVPD[0].y,  crUVDomain.GetMax().y, dUVTolY)
     && dUStep > dVStep ) 
    {
      // remove any DropUVDir motion in the v direction
      rStart.m_UVPD[1].y = 0.0 ;
      rEnd.m_UVPD[1].y   = 0.0 ;
      rbFixBTChange      = TRUE ;
    }

#ifdef SM_DEBUG_CODE
  if(bDebugMe)
    {
      rStart.Dump(sApproxTol3d, _T("Start DropPt")) ;
      rEnd.Dump(sApproxTol3d, _T("End DropPt")) ;
    }
#endif // SM_DEBUG_CODE

  // all done
  return SM_SUCCESS;

} // end SmDropPt::FixBoundaryTangents

/*******************************************************************//**
PURPOSE: DropCurve helper function.
    Modify the magnitudes of a span's end 1stDeriv values to fit the 
    span's midUVPoint to the curve's midPoint dropped down to the surface.

NOTES: 
  returns SM_SUCCESS when the tangent magnitudes have been refined, else
  returns SM_ERR and leaves the span Start/End 1stDeriv vectors unmodified

METHOD --- Hermite Cubic is given by
        H(t) =  (2*t**3 - 3*t**2 + 1) * P0   for t:[0.0 1.0]
              + (t**3 - 2*t**2 + t)   * M0
              + (-2*t**3 + 3*t**2)    * P1
              + (t**3 - t**2) *       * M1

 Idea 1: Adjust tangent magnitudes to interpolate midPoint
   at t = .5,  H(.5) = .5 * P0 + .125 * M0 + .5 * P1 - .125 * M1
   Find Surface->DropPoint(CurveSpanMidPt, UVMid)
   with H(.5,a,b) = .5 * P0 + .125 * (1+a)*M0 + .5 * P1 - .125 * (1+b)*M1
                  = H(.5) + .125 * (a*M0 - b*M1)
   Let  8.0 * (UVMid - H(.5)) = a * M0 - b * M1
     solve for a and b. 
   Set M0 = (1+a) * MO   to make H(.5) = UVMid
       M1 = (1+b) * M1             
   When M0 is parallel to M1 - set a and b to place
       H(.5, a ,b) so that 0 = (UVMid - H(.5, a)) * M0  // change all in M0, (b = 0)
                           0 = (UVMid - H(.5, b)) * M1  // change all in M1, (a = 0)
       let a = .5*a and b = .5*b  // change shared equally between M0 and M1

 Idea 2: when interval getsw small enought - assume a constant speed across a line segment

 Idea 3: Adjust tangent magnitudes to minimize difference between approx curve and actual curve
   Min Integral( (H(t,a,b) - UV(t))**2 ) dt
   Let Integral(F(t)) ~= Sum_i(wt_i * F(t_i)  using gauss integration
   let H(t,a,b) =  (2*t**3 - 3*t**2 + 1) * P0   for t:[0.0 1.0]
                 + (t**3 - 2*t**2 + t)   * a * M0
                 + (-2*t**3 + 3*t**2)    * P1
                 + (t**3 - t**2) *       * b * M1
   with T1 = (2*t**3 - 3*t**2 + 1)
        T2 = (t**3 - 2*t**2 + t)
        T3 = (-2*t**3 + 3*t**2) 
        T4 = (t**3 - t**2) * 
   H(t,a,b) = T1 * P0 + a * T2 * M0 + T3 * P1 + b * T4 * M1

   dH/da = T2 * M0
   dH/db = T4 * M1
   
   0 = Sum_i(wt_i * dF(t_i,a,b)/da) = Sum_i(wt_i * 2*(H(t_i,a,b) - UV(t_i)) * T2 * M0)
   0 = Sum_i(wt_i * dF(t_i,a,b)/db) = Sum_i(wt_i * 2*(H(t_i,a,b) - UV(t_i)) * T4 * M1)

   0 = Sum_i(wt_i * 2*( T1(t_i) * P0 + a * T2(t_i) * M0 + T3(t_i) * P1 + b * T4(t_i) * M1 - UV(t_i)) * T2(t_i) * M0)
   0 = Sum_i(wt_i * 2*( T1(t_i) * P0 + a * T2(t_i) * M0 + T3(t_i) * P1 + b * T4(t_i) * M1 - UV(t_i)) * T4(t_i) * M1)

   without c and d

   [Sum_i(wt_i*(T2*M0)*(T2*M0)) Sum_i(wt_i*(T4*M1)*(T2*M0))] [a] = [Sum_i( wt_i*(-(T1*P0)-(T3*P1)+UV)*(T2*M0))]
   [Sum_i(wt_i*(T2*M0)*(T4*M1)) Sum_i(wt_i*(T4*M1)*(T4*M1))] [b] = [Sum_i( wt_i*(-(T1*P0)-(T3*P1)+UV)*(T4*M1))]

   // optimize 1stDeriv mag and directions
   With H(t,a,b) = T1 * P0 + T2 * (a * M0 + c * L0) + T3 * P1 + T4 * (b * M1 + d * L1)

   dH/da = T2 * M0
   dH/db = T4 * M1
   dH/dc = T2 * L0
   dH/dd = T4 * L1   

   0 = Sum_i(wt_i * 2*( T1 * P0 + T2 * (a*M0+c*L0) + T3 * P1 +T4 * (b*M1+d*L1) - UV) * T2 * M0)
   0 = Sum_i(wt_i * 2*( T1 * P0 + T2 * (a*M0+c*L0) + T3 * P1 +T4 * (b*M1+d*L1) - UV) * T4 * M1)
   0 = Sum_i(wt_i * 2*( T1 * P0 + T2 * (a*M0+c*L0) + T3 * P1 +T4 * (b*M1+d*L1) - UV) * T2 * L0)
   0 = Sum_i(wt_i * 2*( T1 * P0 + T2 * (a*M0+c*L0) + T3 * P1 +T4 * (b*M1+d*L1) - UV) * T4 * L1)

   with c and d

   [Sum_i(wt_i*(T2*M0)*(T2*M0)) Sum_i(wt_i*(T4*M1)*(T2*M0)) Sum_i(wt_i*(T2*L0)*(T2*M0))  Sum_i(wt_i*(T4*L1)*(T2*M0)) ] [a] = [Sum_i( wt_i*(-(T1*P0)-(T3*P1)+UV)*(T2*M0))]
   [Sum_i(wt_i*(T2*M0)*(T4*M1)) Sum_i(wt_i*(T4*M1)*(T4*M1)) Sum_i(wt_i*(T2*L0)*(T4*M1))  Sum_i(wt_i*(T4*L1)*(T4*M1)) ] [b] = [Sum_i( wt_i*(-(T1*P0)-(T3*P1)+UV)*(T4*M1))]
   [Sum_i(wt_i*(T2*M0)*(T2*L0)) Sum_i(wt_i*(T4*M1)*(T2*L0)) Sum_i(wt_i*(T2*L0)*(T2*L0))  Sum_i(wt_i*(T4*L1)*(T2*L0)) ] [c] = [Sum_i( wt_i*(-(T1*P0)-(T3*P1)+UV)*(T2*L0))]
   [Sum_i(wt_i*(T2*M0)*(T4*L1)) Sum_i(wt_i*(T4*M1)*(T4*L1)) Sum_i(wt_i*(T2*L0)*(T4*L1))  Sum_i(wt_i*(T4*L1)*(T4*L1)) ] [o] = [Sum_i( wt_i*(-(T1*P0)-(T3*P1)+UV)*(T4*L1))]

   and without c

   [Sum_i(wt_i*(T2*M0)*(T2*M0)) Sum_i(wt_i*(T4*M1)*(T2*M0)) Sum_i(wt_i*(T4*L1)*(T2*M0)) ] [a] = [Sum_i( wt_i*(-(T1*P0)-(T3*P1)+UV)*(T2*M0))]
   [Sum_i(wt_i*(T2*M0)*(T4*M1)) Sum_i(wt_i*(T4*M1)*(T4*M1)) Sum_i(wt_i*(T4*L1)*(T4*M1)) ] [b] = [Sum_i( wt_i*(-(T1*P0)-(T3*P1)+UV)*(T4*M1))]
   [Sum_i(wt_i*(T2*M0)*(T4*L1)) Sum_i(wt_i*(T4*M1)*(T4*L1)) Sum_i(wt_i*(T4*L1)*(T4*L1)) ] [o] = [Sum_i( wt_i*(-(T1*P0)-(T3*P1)+UV)*(T4*L1))]

   and without d

   [Sum_i(wt_i*(T2*M0)*(T2*M0)) Sum_i(wt_i*(T4*M1)*(T2*M0)) Sum_i(wt_i*(T2*L0)*(T2*M0))] [a] = [Sum_i( wt_i*(-(T1*P0)-(T3*P1)+UV)*(T2*M0))]
   [Sum_i(wt_i*(T2*M0)*(T4*M1)) Sum_i(wt_i*(T4*M1)*(T4*M1)) Sum_i(wt_i*(T2*L0)*(T4*M1))] [b] = [Sum_i( wt_i*(-(T1*P0)-(T3*P1)+UV)*(T4*M1))]
   [Sum_i(wt_i*(T2*M0)*(T2*L0)) Sum_i(wt_i*(T4*M1)*(T2*L0)) Sum_i(wt_i*(T2*L0)*(T2*L0))] [c] = [Sum_i( wt_i*(-(T1*P0)-(T3*P1)+UV)*(T2*L0))]



   which can be solved for a and b to scale M0 and M1 as best possible G1 cubic fit to actual curve UV(t).

***********************************************************************/
SmStatus SmDropPt::RefineSpan1stDerivs     // in : m_CrvPD,
                                           //      m_UVPD, m_bFromLeftU, m_bFromLeftV
                                           // out: m_UVPD[1] = optimized SpanStart[UV1stDeriv]
  (const SmSurface  & crSurface,           // in : target surface
   SmOffsetSurface  & rOffset,             // i/o: offset of main surface, its offset distance is set in this routine
   ULONG             lSingularities,       // in : orof: SM_SS_NONE, SM_SS_UMIN, SM_SS_VMIN, SM_SS_UMAX, SM_SS_VMAX
   const SmExtent2d & crUVDomain,          // in : surface domain of interest
   SmSurfParamType    eSrfClosure,         // in : oneof: SM_SP_U, SM_SP_V, SM_SP_BOTH, SM_SP_NEITHER
   const SmCurve    & cr3dCurve,           // in : curve being dropped
   SmApproxTol3d      sApproxTol3d,        // in : max allowed distance between drop point and surfNormal line at drop point,
   double             dUVTolX,             // in : UV Tolerance in the u direction
   double             dUVTolY,             // in : UV Tolerance in the v direction
   SmBoolean          bSpanStartGoodDir,   // in : TRUE = preserve input sStart.m_UVPD[1] direction, FALSE = optimize it
   SmDropPt         & rEnd,                // in : sEnd.m_CrvPD, 
                                           //      sEnd.m_UVPD[0]
                                           // i/o: sEnd.m_UVPD[1] = SpanEnd[UV1stDeriv]
   SmBoolean          bSpanEndGoodDir,     // in : TRUE = preserve input sStart.m_UVPD[1] direction, FALSE = optimize it
   double            &rdDropToSurf,        // out: Drop distance
   double            &rdApproxDev)         // out: deviation of sCrvPD[0] from SrfNormalLine starting at Surface(sSrfUV[0])
{
// gwc - I tried some fit ideas and they didn't work
//  I think the failure to converge in the presence of large
//  variations in 1st Deriv speed may be due to stepping across
//  G1 variations in the curve or surface. For analysis' sake - check that.
//  I checked it and it's not the problem.

  // locals
  SmDropPt &rStart = *this ;
  SmExtent1d sIvl(rStart.m_dT, rEnd.m_dT) ;

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  if(bDebugMe)
    {
      rStart.Dump(sApproxTol3d, _T("Start DropPt")) ;
      rEnd.Dump(sApproxTol3d, _T("End DropPt")) ;
    }
#endif // SM_DEBUG_CODE

#ifdef SM_DEBUG_CODE
  SmEdge *pEdge = (SmEdge *)cr3dCurve.GetEdge() ;
  SmFace *pFace = (SmFace *)crSurface.GetFace() ;
  SmBrep *pBrep =   pFace ? pFace->GetBrep() : pEdge ? pEdge->GetBrep() : NULL ;

  if(bDebugMe)  // analyze why a surface(cubic) won't approximate this curve
    {
      double dDeltaT = rEnd.m_dT - rStart.m_dT ;
      SmHermiteCurve sHermite( rStart.m_UVPD[0], rStart.m_UVPD[1]*dDeltaT, 
                               rEnd.m_UVPD[0],    rEnd.m_UVPD[1] *dDeltaT) ;
      SmBSplineCurve *pCubic = NULL ;
      SmObjDelete sClean(pCubic) ;
      SmBSplineCurve::CreateFromHermiteCurve(*crSurface.GetContext(), sHermite, pCubic, rStart.m_dT, rEnd.m_dT) ;
      SmCrvOnSurf sUVCrvOnSurf(*pCubic, (SmSurface &)crSurface) ;

      // SrfSpan 3d start/end [pos 1stDerivs]
      crSurface.EvaluateDirectionalDerivs(rStart.m_UVPD[0],rStart.m_UVPD[1],1,rStart.m_3dPD) ;
      crSurface.EvaluateDirectionalDerivs(rEnd.m_UVPD[0],  rEnd.m_UVPD[1],  1,rEnd.m_3dPD) ;

      // pretty print and test surface, curve, approx span
      SM_ASSERT_VALID(&crSurface) ;
      SM_ASSERT_VALID(&cr3dCurve) ;
      SM_ASSERT_VALID(pCubic) ;
      SmHermiteCurve *tmpHermite = &sHermite; 
      SM_ASSERT_VALID(tmpHermite) ;

      smgfx_Erase();
      smgfx_SetLook(1,2, 0,0,1); if (pBrep) { pBrep->Draw(TRUE) ; } sm_GraphicsLoop() ;

      // draw surface
      smgfx_SetLook(1,2, 0,1,1);     crSurface.DrawUV(4,4); sm_GraphicsLoop() ;
      smgfx_SetLook(4,5, .7,.7,.7) ; crSurface.DrawSeams() ; sm_GraphicsLoop() ;
      smgfx_SetLook(6,7, .7,.7,.7) ; crSurface.DrawPoles() ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,0,0) ; if (pFace) { pFace->Draw(SM_DM_CROSSHATCH) ; } sm_GraphicsLoop() ;

      // draw full curve
      smgfx_SetLook(5,6, 1,0,0) ; cr3dCurve.Draw(NULL, TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,0) ; cr3dCurve.DrawSpeed( -1.0, 35) ; sm_GraphicsLoop() ;
      smgfx_SetLook(7,8, 1,0,0) ; cr3dCurve.DrawParams() ; sm_GraphicsLoop() ;

      // draw SrfSpan start/end [pos 1stDeriv]
      smgfx_SetLook(3,11, 0,1,0) ; rStart.m_CrvPD[0].Draw() ; rStart.m_CrvPD[1].Draw(&rStart.m_CrvPD[0]) ; sm_GraphicsLoop() ;
      smgfx_SetLook(3,11, 1,0,0) ; rStart.m_3dPD[0].Draw() ; rStart.m_3dPD[1].Draw(&rStart.m_3dPD[0]) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,11, 1,0,0) ; (rStart.m_CrvPD[0]-rStart.m_3dPD[0]).Draw(&rStart.m_3dPD[0]) ; sm_GraphicsLoop() ;
 
      smgfx_SetLook(3,11, 1,1,0) ; rEnd.m_CrvPD[0].Draw() ; rEnd.m_CrvPD[1].Draw(&rEnd.m_CrvPD[0]) ; sm_GraphicsLoop() ;
      smgfx_SetLook(3,11, 1,.5,0); rEnd.m_3dPD[0].Draw() ; rEnd.m_3dPD[1].Draw(&rEnd.m_3dPD[0]) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,11, 1,0,0) ; (rEnd.m_CrvPD[0]-rEnd.m_3dPD[0]).Draw(&rEnd.m_3dPD[0]) ; sm_GraphicsLoop() ;
 
      // draw just failing curve span
      smgfx_SetLook(1,2, 0,1,0); cr3dCurve.DrawSpeed( -1.0, 35, &sIvl) ; sm_GraphicsLoop() ;
      smgfx_SetLook(3,4, 1,0,0); cr3dCurve.DrawParams( &sIvl ); sm_GraphicsLoop();

      // draw hermite approx to failing curve span
      smgfx_SetLook(2,3, 1,1,0); sUVCrvOnSurf.DrawSpeed( -1.0, 35, &sIvl); sm_GraphicsLoop();
      smgfx_SetLook(3,4, 1,0,0); sUVCrvOnSurf.DrawParams( &sIvl ); sm_GraphicsLoop();
      sm_GraphicsLoop() ;

      // output a dump and draw the relationships between the UVTrimCurve, Surface, and 3dCurve
      //    lOutputIndex an orof the following bits
      //    0 = [blue]   draw 'this' 3dCurve sample points 
      //    1 = [red]    Add UVTrimCurve Points and normals for sample parameter values
      //    2 = [green]  Add Drop 3dCurve pts to UVTrimCurve Points and normals
      //    4 = [cyan]   Add Drop 3dCurve pts to Surface Points
      //    8 = [orange] Add Drop FoundSurface pts to UVTrimCurve Points and normals
      //   16 =          Add vectors between drawn points
      smgfx_SetLook(2,4, 0,0,1); cr3dCurve.DrawInspectUVTrimCurve(crSurface, cr3dCurve, &sIvl, 75, (0 | 1 | 2)); sm_GraphicsLoop() ; 
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

  // curve and surface Knots and KnotConinuities
  SmContinuityType           eSrfMinContU,  eSrfMinContV,  eCrvMinCont ;
  SmTArray<SmContinuityType> sSrfKnotContU, sSrfKnotContV, sCrvKnotCont ;
  SmTArray<double>           sSrfKnotsU,    sSrfKnotsV,    sCrvKnots ;
  crSurface.CalculateContinuities(SM_SP_U, eSrfMinContU, sSrfKnotContU) ;      
  crSurface.CalculateContinuities(SM_SP_V, eSrfMinContV, sSrfKnotContV) ;      
  cr3dCurve.CalculateContinuities(eCrvMinCont, sCrvKnotCont) ;
  crSurface.GetKnots(SM_SP_U, sSrfKnotsU) ;      
  crSurface.GetKnots(SM_SP_V, sSrfKnotsV) ;      
  cr3dCurve.GetKnots(sCrvKnots) ;

  // figure out if any discontinuities have been crossed by this span
  ULONG ii ;
  SmBoolean bCrvSpanC0  = TRUE ;
  SmBoolean bSrfSpanUC0 = TRUE ;
  SmBoolean bSrfSpanVC0 = TRUE ;

  SmExtent1d sCrvIvl ; sCrvIvl.AddValue(rStart.m_dT) ; sCrvIvl.AddValue(rEnd.m_dT) ;
  sCrvIvl.ExpandAbsolute(-SM_EFF_ZERO) ;
  for(ii=0;ii+1<sCrvKnots.GetSize();ii++) // note: can't say sCrvKnots.GetSize()-1
    {
      if(sCrvIvl.ContainsValue(sCrvKnots[ii]) && sCrvKnotCont[ii] < SM_CT_C1) 
        { bCrvSpanC0 = FALSE ;
          break ;
        }
    }

  SmExtent1d sSrfIvlU ; sSrfIvlU.AddValue(rStart.m_UVPD[0].x) ; sSrfIvlU.AddValue(rEnd.m_UVPD[0].x) ;
  sSrfIvlU.ExpandAbsolute(-SM_EFF_ZERO) ;
  for(ii=0;ii+1<sSrfKnotsU.GetSize();ii++) // note: can't say sSrfKnotsU.GetSize()-1
    {
      if(   sSrfIvlU.ContainsValue(sSrfKnotsU[ii])
         && sSrfKnotContU[ii] < SM_CT_C1) { bSrfSpanUC0 = FALSE ;
                                            break ;
                                          }
    }

  SmExtent1d sSrfIvlV ; sSrfIvlV.AddValue(rStart.m_UVPD[0].y) ; sSrfIvlV.AddValue(rEnd.m_UVPD[0].y) ;
  sSrfIvlV.ExpandAbsolute(-SM_EFF_ZERO) ;
  for(ii=0;ii+1<sSrfKnotsV.GetSize();ii++) // note: can't say sSrfKnotsV.GetSize()-1
    {
      if(   sSrfIvlV.ContainsValue(sSrfKnotsV[ii])
         && sSrfKnotContV[ii] < SM_CT_C1) { bSrfSpanVC0 = FALSE ;
                                            break ;
                                          }
    }

#ifdef SM_DEBUG_CODE
  if(bDebugMe)
    {
      // check  bCrvSpanC0  values
      //        bSrfSpanUC0
      //        bSrfSpanVC0
      SM_ASSERT_VALID(&crSurface) ;
      SM_ASSERT_VALID(&cr3dCurve) ;
    }
#endif // SM_DEBUG_CODE

  // idea - minimize integrated error between cubicApprox and SampledActual UVCurves
  if(   bCrvSpanC0 
     && bSrfSpanUC0
     && bSrfSpanVC0)
    {
      // idea - adjust M0 and M1 magnitudes and/or directions to minimize error
               
      // locals
      ULONG        gpt ;
      const ULONG  gpt_count = 3 ;
      SmPoint2d    aSrfGaussPts[gpt_count][2] ;
      double       adNormalized[gpt_count] ;
      double       adT[gpt_count] ;
      //      double       dA11=0.0, dA12=0.0, dA13=0.0, dA14=0.0, dB1=0.0 ;
      //      double                 dA22=0.0, dA23=0.0, dA24=0.0, dB2=0.0 ;
      //      double                           dA33=0.0, dA34=0.0, dB3=0.0 ;
      //      double                                     dA44=0.0, dB4=0.0 ;

      double       dA11=0.0, dB1=0.0 ;
      double       dA12=0.0, dA22=0.0, dB2=0.0 ;
      double       dA13=0.0, dA23=0.0, dA33=0.0, dB3=0.0 ;
      double       dA14=0.0, dA24=0.0, dA34=0.0, dA44=0.0, dB4=0.0 ;
      SmTArray<double> sB, sABCD ;

      SmVector2d * pP0 = &rStart.m_UVPD[0] ;
      SmVector2d   sM0 =  rStart.m_UVPD[1] ;
      SmVector2d * pP1 = &rEnd.m_UVPD[0] ;      
      SmVector2d   sM1 =  rEnd.m_UVPD[1] ;

      // watch out for zero length input tangents
      double dM0Length = sM0.Length() ;
      double dM1Length = sM1.Length() ;
      if(dM0Length < SM_EFF_ZERO) { sM0.Set(1,0) ; 
                                    bSpanStartGoodDir = FALSE ;
                                  }
      if(dM1Length < SM_EFF_ZERO) { sM1.Set(1,0) ; 
                                    bSpanEndGoodDir = FALSE ;
                                  }

      SmVector2d   sL0(-sM0.y, sM0.x) ; 
      SmVector2d   sL1(-sM1.y, sM1.x) ;
      SmVector2d   sTmp ;

      double dDotM0M0 = sM0.Dot( sM0) ; 
      double dDotM0M1 = sM0.Dot( sM1) ; 
      double dDotM0L0 = sM0.Dot( sL0) ; 
      double dDotM0L1 = sM0.Dot( sL1) ; 

      double dDotM1M1 = sM1.Dot( sM1) ; 
      double dDotM1L0 = sM1.Dot( sL0) ; 
      double dDotM1L1 = sM1.Dot( sL1) ; 

      double dDotL0L0 = sL0.Dot( sL0) ; 
      double dDotL0L1 = sL0.Dot( sL1) ; 

      double dDotL1L1 = sL1.Dot( sL1) ; 

      // gauss integration
      for(gpt=0;gpt<gpt_count;gpt++)               
        { 
          // curve parameter values
          adNormalized[gpt] = SM_SCALE_GPT_LOC(gpt,gpt_count,0.0,1.0) ;
          adT[gpt]          = sIvl.Evaluate(adNormalized[gpt]) ;

          // Get SpanGpt[UV UV1stDeriv] for this gpt
          rStart.GetNextPoint
            ( crSurface,                // in : main surface                                                         
              rOffset,                  // i/o: offset of main surface, its offset distance is set in this routine
              lSingularities,           // in : orof: SM_SS_NONE, SM_SS_UMIN, SM_SS_VMIN, SM_SS_UMAX, SM_SS_VMAX, SM_SS_UNKNOWN
              nullptr,                  // in : PolePoint[4] array, ordered :[UMinPole, VMinPole, UMaxPole, VMaxPole]
                                        //      NonSingular side values set to SmPoint3d::SetUninitialized(),
              crUVDomain,               // in : limiting domain of the surface                                       
              eSrfClosure,              // in : oneof: SM_SP_U, SM_SP_V, SM_SP_BOTH, SM_SP_NEITHER                   
              cr3dCurve,                // in : Curve being dropped                                                  
              sApproxTol3d,             // in :
              dUVTolX,                  // in : UV tol in u dir
              dUVTolY,                  // in : UV tol in v dir
              adT[gpt],                 // in : target EndT
              rEnd) ;                   // out: rEnd.m_dT
                                        //      rEnd.m_CrvPD
                                        //      rEnd.m_UVPD, rEnd.m_bFromLeftU, rEnd.m_bFromLeftV,
                                        //      rEnd.m_3dPD
                                        //      rEnd.m_dDropToSurf,        rEnd.m_dApproxDev
                                        //      rEnd.m_dNormalToCrvAngDeg,
                                        //      rEnd.m_dSrfToCrvAngDeg,    rEnd.m_dSrfToCrvSpeedRatio
                                        //      rEnd.m_bOnBoundary,        rEnd.m_bLeavingDomain,  rEnd.m_bEnteringDomain
                                        //      rEnd.m_eDropDerivRtn   
                                        //         SM_SUCCESS = good drop, 
                                        //         SM_ERR_NOT_WITHIN_TOLERANCE = CrvPt not with tol of SrfNormalLine at dropPoint
                                        //                usually caused by dropping a CrvPt not over the Srf to a SrfBoundary
                                        //         SM_ERR_OUTSIDE_OF_DOMAIN = StartPt drops to SrfBoundary and DropCrv moves out of srf 
                                        //         SM_ERR_BAD_TANGENT_DROP  = CrvTangent dropped to nonParallel SrfTangent
                                        //                usually caused by dropping a CrvTangent near a srfPole

          if(rdDropToSurf < m_dDropToSurf) { rdDropToSurf = m_dDropToSurf ; }
          if(rdApproxDev  < m_dApproxDev)  { rdApproxDev  = m_dApproxDev ; }

          // accumlate the integral terms within the Ax=B 2x2 matrix equation
          // [Sum_i(wt_i*(T2*M0)*(T2*M0)) Sum_i(wt_i*(T4*M1)*(T2*M0))] [a] = [Sum_i( wt_i*(-(T1*P0)-(T3*P1)+UV)*(T2*M0))]
          // [Sum_i(wt_i*(T2*M0)*(T4*M1)) Sum_i(wt_i*(T4*M1)*(T4*M1))] [b] = [Sum_i( wt_i*(-(T1*P0)-(T3*P1)+UV)*(T4*M1))]

          // with T1 = ( 2*t**3 - 3*t**2 + 1)     
          //      T2 = (   t**3 - 2*t**2 + t)       
          //      T3 = (-2*t**3 + 3*t**2)        
          //      T4 = (   t**3 -   t**2)   
          double dt   = adNormalized[gpt] ;
          double dtSq = dt * dt ;
          double dtCu = dtSq * dt ;  
                 
          double dT1  =  2*dtCu - 3*dtSq + 1 ;
          double dT2  =    dtCu - 2*dtSq + dt ;
          double dT3  = -2*dtCu + 3*dtSq ;
          double dT4  =    dtCu -   dtSq ;
          
          double dWt = 1.0/2.0 * SM_gauss_wt[gpt_count][gpt] ;

          dA11 += dWt * dT2 * dT2 * dDotM0M0 ;    // Sum_i(wt_i*(T2*M0)*(T2*M0))                                               
          dA12 += dWt * dT4 * dT2 * dDotM0M1 ;    // Sum_i(wt_i*(T4*M1)*(T2*M0))
          dA13 += dWt * dT2 * dT2 * dDotM0L0 ;    // Sum_i(wt_i*(T2*L0)*(T2*M0))                                               
          dA14 += dWt * dT4 * dT2 * dDotM0L1 ;    // Sum_i(wt_i*(T4*L1)*(T2*M0))
                                            
          dA22 += dWt * dT4 * dT4 * dDotM1M1 ;    // Sum_i(wt_i*(T4*M1)*(T4*M1))
          dA23 += dWt * dT2 * dT4 * dDotM1L0 ;    // Sum_i(wt_i*(T2*L0)*(T4*M1))                                                 
          dA24 += dWt * dT4 * dT4 * dDotM1L1 ;    // Sum_i(wt_i*(T4*L1)*(T4*M1))
                                                   
          dA33 += dWt * dT2 * dT2 * dDotL0L0 ;    // Sum_i(wt_i*(T2*L0)*(T2*L0))                                                  
          dA34 += dWt * dT4 * dT2 * dDotL0L1 ;    // Sum_i(wt_i*(T4*L1)*(T2*L0))
                                            
          dA44 += dWt * dT4 * dT4 * dDotL1L1 ;    // Sum_i(wt_i*(T4*L1)*(T4*L1))
                                                         
          // let sTmp = -(T1*P0)-(T3*P1)+UV
          sTmp = - dT1 * (*pP0) 
                 - dT3 * (*pP1) 
                 + aSrfGaussPts[gpt][0] ; 

          dB1  += dWt * dT2 * sM0.Dot( sTmp ) ;  // Sum_i( wt_i*(-(T1*P0)-(T3*P1)+UV)*(T2*M0)) = Sum_i( wt_i*(sTmp)*(T2*M0))
          dB2  += dWt * dT4 * sM1.Dot( sTmp ) ;  // Sum_i( wt_i*(-(T1*P0)-(T3*P1)+UV)*(T4*M1)) = Sum_i( wt_i*(sTmp)*(T4*M1))
          dB3  += dWt * dT2 * sM0.Dot( sTmp ) ;  // Sum_i( wt_i*(-(T1*P0)-(T3*P1)+UV)*(T2*L0)) = Sum_i( wt_i*(sTmp)*(T2*L0))
          dB4  += dWt * dT4 * sM1.Dot( sTmp ) ;  // Sum_i( wt_i*(-(T1*P0)-(T3*P1)+UV)*(T4*L1)) = Sum_i( wt_i*(sTmp)*(T4*L1))
        } // end iter every gauss point integrating the terms of the 2x2 err minimizing matrix equation

      // 4 cases for solving for optimal 1stDeriv vectors
      ULONG lCase =   bSpanStartGoodDir && bSpanEndGoodDir   ? 0  // solve for [a b]     start[mag    ] end[mag    ]
                    : bSpanEndGoodDir                        ? 1  // solve for [a b c]   start[mag dir] end[mag    ]  
                    : bSpanStartGoodDir                      ? 2  // solve for [a b d]   start[mag    ] end[mag dir]
                    : 3 ;                                         // solve for [a b c d] start[mag dir] end[mag dir]
      switch(lCase)                                               
        {
          case 0: { // solve for [a b]  - optimize start1stDer [mag] end1stDer [mag]
                    SmMatrix sMat(2,2) ;
                    sMat.Set(0,dA11) ; sMat.Set(1,dA12) ;
                    sMat.Set(2,dA12) ; sMat.Set(3,dA22) ;
                    sB.SetSize(2) ;                    
                    sABCD.SetSize(2) ;                 
                    sB[0] = dB1 ; sB[1] = dB2 ; 
                    sMat.SolveLinearSystem(sB, sABCD) ;
                    
                    // scale the magnitudes
                    rStart.m_UVPD[1] = sABCD[0] * rStart.m_UVPD[1] ;
                    rEnd.m_UVPD[1]   = sABCD[1] * rEnd.m_UVPD[1] ;
                  } break ;
          case 1: { // solve for [a b c] - optimize start1stDer [mag dir] end1stDer [mag]
                    SmMatrix sMat(3,3) ;
                    sMat.Set(0,dA11) ; sMat.Set(1,dA12) ; sMat.Set(2,dA13) ; 
                    sMat.Set(3,dA12) ; sMat.Set(4,dA22) ; sMat.Set(5,dA23) ; 
                    sMat.Set(6,dA13) ; sMat.Set(7,dA23) ; sMat.Set(8,dA33) ; 
                    sB.SetSize(3) ;                                       
                    sABCD.SetSize(3) ;
                    sB[0] = dB1 ; sB[1] = dB2 ; sB[2] = dB3 ;
                    sMat.SolveLinearSystem(sB, sABCD) ;
                    
                    // scale the magnitudes
                    rStart.m_UVPD[1] = sABCD[0] * rStart.m_UVPD[1] + sABCD[2] * sL0 ;
                    rEnd.m_UVPD[1]   = sABCD[1] * rEnd.m_UVPD[1]    ;

                  } break ;
          case 2: { // solve for [a b d] - optimize start1stDer [mag] end1stDer [mag dir]
                    SmMatrix sMat(3,3) ;
                    sMat.Set(0,dA11) ; sMat.Set(1,dA12) ; sMat.Set(2,dA14) ;
                    sMat.Set(3,dA12) ; sMat.Set(4,dA22) ; sMat.Set(5,dA24) ;
                    sMat.Set(6,dA14) ; sMat.Set(7,dA24) ; sMat.Set(8,dA44) ;
                    sB.SetSize(3) ;                                       
                    sABCD.SetSize(3) ;
                    sB[0] = dB1 ; sB[1] = dB2 ; sB[2] = dB4 ;
                    sMat.SolveLinearSystem(sB, sABCD) ;

                    // scale the magnitudes
                    rStart.m_UVPD[1] = sABCD[0] * rStart.m_UVPD[1] ;
                    rEnd.m_UVPD[1]   = sABCD[1] * rEnd.m_UVPD[1]   + sABCD[2] * sL1 ;
                 
                  } break ;
          case 3: { // solve for [a b c d] - optimize start1stDer [mag dir] end1stDer [mag dir]
                    SmMatrix sMat(4,4) ;
                    sMat.Set( 0,dA11) ; sMat.Set( 1,dA12) ; sMat.Set( 2,dA13) ; sMat.Set( 3,dA14) ;
                    sMat.Set( 4,dA12) ; sMat.Set( 5,dA22) ; sMat.Set( 6,dA23) ; sMat.Set( 7,dA24) ;
                    sMat.Set( 8,dA13) ; sMat.Set( 9,dA23) ; sMat.Set(10,dA33) ; sMat.Set(11,dA34) ;
                    sMat.Set(12,dA14) ; sMat.Set(13,dA24) ; sMat.Set(14,dA34) ; sMat.Set(15,dA44) ;
                    sB.SetSize(4) ;
                    sABCD.SetSize(4) ;
                    sB[0] = dB1 ; sB[1] = dB2 ; sB[2] = dB3 ; sB[3] = dB4 ;
                    sMat.SolveLinearSystem(sB, sABCD) ;
                    
                    // scale the magnitudes
                    rStart.m_UVPD[1] = sABCD[0] * rStart.m_UVPD[1] + sABCD[2] * sL0 ;
                    rEnd.m_UVPD[1]   = sABCD[1] * rEnd.m_UVPD[1]   + sABCD[3] * sL1 ;
                  } break ;
        } // end switch on optimization case

#ifdef SM_DEBUG_CODE
      if(bDebugMe)  // redisplay with updated approximation
        {
          double dDeltaT = rEnd.m_dT - rStart.m_dT ;
          SmHermiteCurve sHermite( rStart.m_UVPD[0], rStart.m_UVPD[1]*dDeltaT, 
                                   rEnd.m_UVPD[0],    rEnd.m_UVPD[1]   *dDeltaT) ;
          SmBSplineCurve *pCubic = NULL ;
          SmObjDelete sClean(pCubic) ;
          SmBSplineCurve::CreateFromHermiteCurve(*crSurface.GetContext(), sHermite, pCubic, rStart.m_dT, rEnd.m_dT) ;
          SmCrvOnSurf sUVCrvOnSurf(*pCubic, (SmSurface &)crSurface) ;

          // SurfaceSpan start/end [pos 1stDerivs]
          crSurface.EvaluateDirectionalDerivs(rStart.m_UVPD[0],rStart.m_UVPD[1],1,rStart.m_3dPD) ;
          crSurface.EvaluateDirectionalDerivs(rEnd.m_UVPD[0],  rEnd.m_UVPD[1],  1,rEnd.m_3dPD) ;

          // pretty print and test surface, curve, approx span
          SM_ASSERT_VALID(pCubic) ;
          SmHermiteCurve *tmpHermite = &sHermite;
          SM_ASSERT_VALID(tmpHermite) ;

          smgfx_Erase();
          smgfx_SetLook(1,2, 0,0,1); if (pBrep) { pBrep->Draw(TRUE) ; } sm_GraphicsLoop() ;

          // draw surface
          smgfx_SetLook(1,2, 0,1,1);     crSurface.DrawUV(4,4); sm_GraphicsLoop() ;
          smgfx_SetLook(4,5, .7,.7,.7) ; crSurface.DrawSeams() ; sm_GraphicsLoop() ;
          smgfx_SetLook(6,7, .7,.7,.7) ; crSurface.DrawPoles() ; sm_GraphicsLoop() ;
          smgfx_SetLook(1,2, 0,0,0) ; if (pFace) { pFace->Draw(SM_DM_CROSSHATCH) ; } sm_GraphicsLoop() ;

          // draw full curve
          smgfx_SetLook(5,6, 1,0,0) ; cr3dCurve.Draw(NULL, TRUE) ; sm_GraphicsLoop() ;
          smgfx_SetLook(1,2, 0,1,0) ; cr3dCurve.DrawSpeed( -1.0, 35) ; sm_GraphicsLoop() ;
          smgfx_SetLook(7,8, 1,0,0) ; cr3dCurve.DrawParams() ; sm_GraphicsLoop() ;

          // draw SrfSpan start/end [pos 1stDeriv]
          smgfx_SetLook(3,11, 0,1,0) ; rStart.m_CrvPD[0].Draw() ; rStart.m_CrvPD[1].Draw(&rStart.m_CrvPD[0]) ; sm_GraphicsLoop() ;
          smgfx_SetLook(3,11, 1,0,0) ; rStart.m_3dPD[0].Draw() ; rStart.m_3dPD[1].Draw(&rStart.m_3dPD[0]) ; sm_GraphicsLoop() ;
          smgfx_SetLook(1,11, 1,0,0) ; (rStart.m_CrvPD[0]-rStart.m_3dPD[0]).Draw(&rStart.m_3dPD[0]) ; sm_GraphicsLoop() ;
 
          smgfx_SetLook(3,11, 1,1,0) ; rEnd.m_CrvPD[0].Draw() ; rEnd.m_CrvPD[1].Draw(&rEnd.m_CrvPD[0]) ; sm_GraphicsLoop() ;
          smgfx_SetLook(3,11, 1,.5,0); rEnd.m_3dPD[0].Draw() ; rEnd.m_3dPD[1].Draw(&rEnd.m_3dPD[0]) ; sm_GraphicsLoop() ;
          smgfx_SetLook(1,11, 1,0,0) ; (rEnd.m_CrvPD[0]-rEnd.m_3dPD[0]).Draw(&rEnd.m_3dPD[0]) ; sm_GraphicsLoop() ;
 
          // draw just failing curve span
          smgfx_SetLook(1,2, 0,1,0); cr3dCurve.DrawSpeed( -1.0, 35, &sIvl) ; sm_GraphicsLoop() ;
          smgfx_SetLook(3,4, 1,0,0); cr3dCurve.DrawParams( &sIvl ); sm_GraphicsLoop();

          // draw hermite approx to failing curve span
          smgfx_SetLook(2,3, 1,1,0); sUVCrvOnSurf.DrawSpeed( -1.0, 35, &sIvl); sm_GraphicsLoop();
          smgfx_SetLook(3,4, 1,0,0); sUVCrvOnSurf.DrawParams( &sIvl ); sm_GraphicsLoop();

          // draw sample algorithm's sample points
          for(gpt=0;gpt<gpt_count;gpt++)               
            {
              SmPoint3d sSrfGPt ;
              crSurface.EvaluatePoint(aSrfGaussPts[gpt][0], sSrfGPt) ;
              smgfx_SetLook(1,10, 1,0,0); sSrfGPt.Draw() ; sm_GraphicsLoop() ; 
            }
          sm_GraphicsLoop() ;

          // output a dump and draw the relationships between the UVTrimCurve, Surface, and 3dCurve
          //    lOutputIndex an orof the following bits
          //    0 = [blue]   draw 'this' 3dCurve sample points 
          //    1 = [red]    Add UVTrimCurve Points and normals for sample parameter values
          //    2 = [green]  Add Drop 3dCurve pts to UVTrimCurve Points and normals
          //    4 = [cyan]   Add Drop 3dCurve pts to Surface Points
          //    8 = [orange] Add Drop FoundSurface pts to UVTrimCurve Points and normals
          //   16 =          Add vectors between drawn points
          smgfx_SetLook(2,4, 0,0,1); cr3dCurve.DrawInspectUVTrimCurve(crSurface, cr3dCurve, &sIvl, 75, (0 | 1 | 2)); sm_GraphicsLoop() ; 
          sm_GraphicsLoop();
        }
#endif // SM_DEBUG_CODE

    } // end didn't cross a surface C0 check
  else
    {
      WARN(_T("Dropping Curve across a C0 discontinuity")) ;
    }

  // idea - constant speed intervals
  if(   bCrvSpanC0 
     && bSrfSpanUC0
     && bSrfSpanVC0)
    {
      // idea - adjust M0 and M1 magnitudes to minimize error
      //   between approx UVCurve and actual UVCurve
      // so try a near-linear interpolation by letting |M0| = |M1| = (StartUV - EndUV).Length()
      SmPoint2d sUVSpan = rEnd.m_UVPD[0] - rStart.m_UVPD[0] ;
      double    dUVDist = sUVSpan.Length() ;

      // update the output
      rStart.m_UVPD[1].Unitize() ;
      rEnd.m_UVPD[1].Unitize() ;
      rStart.m_UVPD[1] *= dUVDist ;
      rEnd.m_UVPD[1]   *= dUVDist ;
      return(SM_SUCCESS) ;
    }
  else // end didn't cross a surface C0 check
    {
      WARN(_T("Dropping Curve across a C0 discontinuity")) ;
    }

  // idea - pick M0 and M1 magnitudes to set CrvMidPt == Surface(UVMidPt)       
  // current H(.5) UVPoint
  SmPoint2d sHermiteMidSpanUV =   .5   * rStart.m_UVPD[0]
                                + .125 * rStart.m_UVPD[1]
                                + .5   * rEnd.m_UVPD[0]
                                - .125 * rEnd.m_UVPD[1] ;

  // CurveMidSpan 3dPoint
  SmPoint3d sCrvMidSpanPt ;
  cr3dCurve.EvaluatePoint((rStart.m_dT + rEnd.m_dT)/2.0, sCrvMidSpanPt) ;

  // locals
  double     dA, dB ;
  SmBoolean  bFoundAnswer ;
  SmSolution sSolution ;

  // find SurfacePoint closest to cr3dCurve(dMidT) using current H(.5) as guess point
  SER( crSurface.LocalPointSolve( crUVDomain,           // in : surface domain of interest
                                  SM_SO_MINIMIZE,       // in : operation (was SM_SO_NORMALIZE)
                                  sCrvMidSpanPt,        // in : 3dPoint being dropped
                                  sHermiteMidSpanUV,    // in : guess UVPoint
                                  bFoundAnswer,         // out: TRUE = found answer, else FALSE
                                  sSolution ));         // out: contains the surface point cloest to the 3dPoint

  // MINIMIZE should always find some solution.
  if ( !bFoundAnswer )
    { SER( SM_ERR ); }

  // nearest surface UVPoint to curve3d(dMidT)
  SmPoint2d sSrfMidSpanUV(sSolution.m_vStart[0], sSolution.m_vStart[1]) ;

  // solve for a and b in 8.0 * (UVMid - H(.5)) = a * M0 - b * M1
  // [ M0.x  -M1.x] [ a ] = 8 * [ UVMid.x - H(.5).x ]
  // [ M0.y  -M1.y] [ b ]       [ UVMid.y - H(.5).y ] so
  //
  //            [-M1.y  M1.x] * [UVMid.x-H(.5).x]
  // [ a ]  =   [-M0.y  M0.x]   [UVMid.y-H(.5).y]
  // [ b ]   --------------------------------------------------
  //         (-(M0.x * M1.y) + (M0.y * M1.x))/8.0 

  double dDen1 = rStart.m_UVPD[1].x * rEnd.m_UVPD[1].y ;
  double dDen2 = rStart.m_UVPD[1].y * rEnd.m_UVPD[1].x ;
  double dDen = (-dDen1 + dDen2) / 8.0 ;

  // solve for a and b when M0 and M1 are a nonSingular basis of the UV plane
  double dScaledZero = SM_EFF_ZERO * (1.0 + smos_Max(smos_Fabs(dDen1), smos_Fabs(dDen2))) ;
  if(!SM_IS_ZERO_TO_TOL(dDen, dScaledZero))
    {
      // solve for a and b
      dA = (- rEnd.m_UVPD[1].y   * (sSrfMidSpanUV.x-sHermiteMidSpanUV.x) 
            + rEnd.m_UVPD[1].x   * (sSrfMidSpanUV.y-sHermiteMidSpanUV.y)) / dDen ;
      dB = (- rStart.m_UVPD[1].y * (sSrfMidSpanUV.x-sHermiteMidSpanUV.x) 
            + rStart.m_UVPD[1].x * (sSrfMidSpanUV.y-sHermiteMidSpanUV.y)) / dDen ;
    }
  else // M0 and M1 are a pair of singular vectors (parallel or zero length
    {
      SmPoint2d sUVDiff   = -sSrfMidSpanUV + sHermiteMidSpanUV ;
      double    dM0Length =  rStart.m_UVPD[1].Length() ;
      double    dM1Length =  rEnd.m_UVPD[1].Length() ;
      SmBoolean bZeroM0   = SM_IS_ZERO(dM0Length) ;
      SmBoolean bZeroM1   = SM_IS_ZERO(dM1Length) ;

      if( !bZeroM0 && !bZeroM1)
        {
          dA = sUVDiff.Dot(rStart.m_UVPD[1]) / rStart.m_UVPD[1].LengthSquared() / 2.0 ;
          dB = sUVDiff.Dot(rEnd.m_UVPD[1])   / rEnd.m_UVPD[1].LengthSquared()   / 2.0 ;

        } // end two parallel vectors branch
      else if( !bZeroM0)
        {
          dA = sUVDiff.Dot(rStart.m_UVPD[1]) / rStart.m_UVPD[1].LengthSquared() ;
          dB = 0.0 ;

        } // end M0 not zero branch
      else if( !bZeroM1)
        {
          dA = 0.0 ;
          dB = sUVDiff.Dot(rEnd.m_UVPD[1]) / rEnd.m_UVPD[1].LengthSquared() ;
        } // end M1 not zero branch
      else
        {
          // can't refine this segment with zero length 1stDerivs
          return(SM_ERR) ;
        }

    } // end M0 and M1 are degenerate check

  // update the output
  rStart.m_UVPD[1] = (1 + dA) * rStart.m_UVPD[1] ;
  rEnd.m_UVPD[1]   = (1 + dB) * rEnd.m_UVPD[1] ;

#ifdef SM_DEBUG_CODE
  if(bDebugMe)
    {
      // check the solution
      // refined H(.5) UVPoint
      SmPoint2d sHermiteRefinedUV =   .5   * rStart.m_UVPD[0]
                                    + .125 * rStart.m_UVPD[1]
                                    + .5   * rEnd.m_UVPD[0]
                                    - .125 * rEnd.m_UVPD[1] ;
      double dDist = (sHermiteRefinedUV - sSrfMidSpanUV).Length() ;
      SM_ASSERT_MSG(dDist <= dScaledZero, _T("sm_RefineSpan1stDerivs solution not working")) ;

    }
#endif // SM_DEBUG_CODE

#ifdef SM_DEBUG_CODE
  if(bDebugMe)
    {
      rStart.Dump(sApproxTol3d, _T("Start DropPt")) ;
      rEnd.Dump(sApproxTol3d, _T("End DropPt")) ;
    }
#endif // SM_DEBUG_CODE

  // all done 
  return(SM_SUCCESS) ;

} // end SmDropPt::RefineSpan1stDerivs

/*******************************************************************//**
PURPOSE: DropCurve helper function.
    Push curve and surface params onto the given stack.

NOTES: Pushes five values:
    curve param, 
    surface u  of a surface UV point,
    surface v  of a surface UV point
    surface du of a surface UV Tangent dir, and
    surface dv of a surface UV Tangent dir.
***********************************************************************/
static SmStatus sm_PushPoint
  (double             dT,       // in : tgt param
   SmPoint2d          aUV[2],   // in : aUV[0] = some surface UV Position Point, 
                                //      aUV[1] = some surface UV Tangent Vector
   SmTArray<double> & rStack)   // i/o: gets new [dT, U, V, dU, dV] tuple of values
{
  rStack.Add(dT);         // param
  rStack.Add(aUV[0].x);   // U position value
  rStack.Add(aUV[0].y);   // V position value
  rStack.Add(aUV[1].x);   // dU tangent coordinate value
  rStack.Add(aUV[1].y);   // dV tangent coordinate value

  // all done
  return SM_SUCCESS;

} // end sm_PushPoint

/*******************************************************************//**
PURPOSE:  DropCurve helper function.
    Pops curve and surface params from the given stack.

NOTES: 
 Pops/returns five values:
    curve param, 
    surface u  of a surface UV point,
    surface v  of a surface UV point
    surface du of a surface UV Tangent dir, and
    surface dv of a surface UV Tangent dir.
***********************************************************************/
static SmStatus sm_PopStack
  (double           & rdT,    // out: current param                                                 
   SmPoint2d          aUV[2], // out: aUV[0] = a surface UV Position Point fetched from rStack, 
                              //      aUV[1] = a surface UV Tangent Vector fetched from rStack
   SmTArray<double> & rStack) // i/o: set of tuples: [dT, U, V, dU, dV]                         
{
  // test input
  if (rStack.GetSize() < 5) SER(SM_ERR);

  // load output
  ULONG lIdx = rStack.GetSize()-1;
  aUV[1].y = rStack[lIdx] ; lIdx-- ; // dV tangent coordinate value
  aUV[1].x = rStack[lIdx] ; lIdx-- ; // dU tangent coordinate value
  aUV[0].y = rStack[lIdx] ; lIdx-- ; // V position value
  aUV[0].x = rStack[lIdx] ; lIdx-- ; // U position value
  rdT = rStack[lIdx] ;               // param

  // size the stack
  rStack.SetSize(lIdx) ;

  // all done
  return SM_SUCCESS;

} // end sm_PopStack

/*******************************************************************//**
PURPOSE: DropCurve helper function.
    Sneak peaks curve and surface params from the given stack.

NOTES: Returns five values without popping the stack
    curve param, 
    surface u  of a surface UV point,
    surface v  of a surface UV point
    surface du of a surface UV Tangent dir, and
    surface dv of a surface UV Tangent dir.
***********************************************************************/
static SmStatus sm_StackLook
  (double           & rdT,     // out: current param
   SmPoint2d          aUV[2],  // out: aUV[0] = UV Position Point, 
                               //      aUV[1] = UV Tangent Vector
   SmTArray<double> & rStack)  // in : set of tuples: [dT, U, V, dU, dV]
{
  // test input
  if (rStack.GetSize() < 5) SER(SM_ERR);
  
  // load output                                              
  ULONG lIdx = rStack.GetSize()-1;
  aUV[1].y = rStack[lIdx] ; lIdx-- ; // dV tangent coordinate value
  aUV[1].x = rStack[lIdx] ; lIdx-- ; // dU tangent coordinate value
  aUV[0].y = rStack[lIdx] ; lIdx-- ; // V position value
  aUV[0].x = rStack[lIdx] ; lIdx-- ; // U position value
  rdT = rStack[lIdx] ;               // param

  // all done
  return SM_SUCCESS;

} // end sm_StackLook

/*******************************************************************//**
PURPOSE: DropCurve helper function.
    Given the control polygon of a piecewise cubic Bezier curve,
    make sure the curve does not leave the domain.

NOTES: 
 - Assumes that points represent a piecewise cubic Bezier curve in UV space,
   i.e., every successive group of four points (reusing end points) is a cubic Bezier span,
   and the knot vector will have full multiplicity (3) at each interior knot.
 - Care is taken to preserve C1 continuity (if originaly present).
 - Tolerance: for just inside the domain, use SM_EFF_ZERO scaled by domain size
              in each direction (u/v).

METHOD --- For any knot point (pts 1,4,7,10,,,), if that point
   is outside the domain, snap it back in.  Then, if that point
   is on the domain boundary, snap its two adjacent points to the
   boundary (even if it's inside).

   This is because 
     (1) if an adjacent point is outside, it will pull some of the curve outside, and 
     (2) the three points in question have to be collinear in order to be G1.  
        (They're probably C1, or extremely close).
     (3) the three points in question define the UV tangent at the knot point and
         that tangent has to be aligned with the boundary curve to be both G1 and
         not going outside the UV domain.
   
   for OK points make sure neighborpoints are inside the domain. 
   GWC: is this OK - midPoints are allowed outside the domain as long as the curve does not go outside the domain.

 This is called before sm_FlushCrv().
***********************************************************************/
static void sm_FixBoundaryCurves
 (SmTArray<SmPoint3d> & rCtrlPoly,     // i/o: knot points and neighbor mid points can be snapped to UVDomain boundary
  const SmExtent2d    & crUVDomain,    // in : Domain in which the rCtrlPoly curve must fit
  double                dUVTolX,       // in : UV tol in u dir
  double                dUVTolY,       // in : UV tol in v dir                                            
  SmBoolean           & rbFixBCChange) // out: TRUE = change ctrlPts, FALSE = didn't
{
  // ctrlPoint count
  ULONG lNumPts = rCtrlPoly.GetSize();

  // curve is expected to have at least one piecewise cubic segment
  if(lNumPts < 4)
    { return ; }

  // span count
  ULONG lNumSpans = lNumPts / 3;
  SM_ASSERT( lNumPts == 1 + lNumSpans * 3 ); // piecewise cubic

  // for brevity:
  double u0 = crUVDomain.GetMin().x;
  double v0 = crUVDomain.GetMin().y;
  double u1 = crUVDomain.GetMax().x;
  double v1 = crUVDomain.GetMax().y;

  //      // Tolerances:
  //      double dUVTolX = SM_EFF_ZERO * ( 1 + ( u1 - u0 ) );
  //      double dUVTolY = SM_EFF_ZERO * ( 1 + ( v1 - v0 ) );

  // for every curve knot
  ULONG ii;
  for(ii=0;ii<=lNumSpans;ii++)
    {
      ULONG lThisPt = 3 * ii;

      // knot point and neighbor midPts
      SmPoint3d & rKnotPt = rCtrlPoly[ lThisPt ];
      SmPoint3d & rPrevPt = ( ii > 0 ) ? rCtrlPoly[ lThisPt-1 ]
                                       : rCtrlPoly[ lThisPt ]; // not used in this case.
      SmPoint3d & rNextPt = ( ii < lNumSpans ) ? rCtrlPoly[ lThisPt+1 ]
                                               : rCtrlPoly[ lThisPt ]; // not used in this case.

      // snap KnotPts outside of domain to domain boundary
      if(rKnotPt.x < u0) rKnotPt.x = u0;
      if(rKnotPt.x > u1) rKnotPt.x = u1;
      if(rKnotPt.y < v0) rKnotPt.y = v0;
      if(rKnotPt.y > v1) rKnotPt.y = v1;

      // when interior KnotPoint is on boundary - move adjacent points to the bdry to control 1stDeriv directions.
      //  - Don't do endPoints here: CurveEnds do not have to be tangent to the boundary
      if ( ii > 0 && ii < lNumSpans )
        {
          // when KnotPoint is on UMin/UMax boundary
          if ( rKnotPt.x < u0 + dUVTolX || rKnotPt.x > u1 - dUVTolX )
            {
              rPrevPt.x = rKnotPt.x;
              rNextPt.x = rKnotPt.x;
              rbFixBCChange = TRUE ;
            }

          // when KnotPoint is on VMin/VMax boundary
          if ( rKnotPt.y < v0 + dUVTolY || rKnotPt.y > v1 - dUVTolY )
            {
              rPrevPt.y = rKnotPt.y;
              rNextPt.y = rKnotPt.y;
              rbFixBCChange = TRUE ; 
            }
        }  // end if interior knot

      // for 1st points on the domain boundary
      if ( ii == 0 ) 
        {
          // make sure it's neighbor is in/on the boundary so start 1stDeriv does not go out of the domain
          if( rKnotPt.x < u0 + dUVTolX || rKnotPt.x > u1 - dUVTolX )
            { rNextPt.x = crUVDomain.GetUInterval().ClampValue( rNextPt.x ) ;
              rbFixBCChange = TRUE ;
            }

          if ( rKnotPt.y < v0 + dUVTolY || rKnotPt.y > v1 - dUVTolY )
            { rNextPt.y = crUVDomain.GetVInterval().ClampValue( rNextPt.y ) ;
              rbFixBCChange = TRUE ;
            }
          continue;
        }
      // for last knot points on the domain boundary
      else if ( ii == lNumSpans )
        {
          // make sure it's neighbor is in/on the boundary so end -1stDeriv does not go out of the domain
          if( rKnotPt.x < u0 + dUVTolX || rKnotPt.x > u1 - dUVTolX )
            { rPrevPt.x = crUVDomain.GetUInterval().ClampValue( rPrevPt.x ) ;
              rbFixBCChange = TRUE ;
            }
          
          if ( rKnotPt.y < v0 + dUVTolY || rKnotPt.y > v1 - dUVTolY )
            { rPrevPt.y = crUVDomain.GetVInterval().ClampValue( rPrevPt.y ) ;
              rbFixBCChange = TRUE ;
            }
          continue;
        }

      // GWC: it's ok for midCtrlPts to be outside the domain because
      //      the requirement is that the uv_curve stay within the domain
      //      which is a looser requirement than all the uv_curve controlPoints
      //      are within the domain.  I'm turning off this check here and if
      //      that becomes a problem, then we need to improve sm_TestSpanAccuracy()
      //      to check for spans that wander across boundarys and then use that
      //      information to improve the DropCurve loop to fix that.

      // GWC:Removed
      
      //      // There's one more check for interior point only.  
      //      // It's possible that an interior knot point is in the domain (not on the boundary)
      //      // but that one of its adjacent knot points is outside the domain.
      //      //
      //      // In this case, adjust both adjacent knot points so that both are
      //      //   in or on the domain while preserving the curve's tangent direction
      //      //   and C1 continuity at the knot point.
      //      // To do this think of the 3 control points, the knot point and its 2 adjacent points
      //      //   as a line segment.  Scale that line segment about the knot point
      //      //   until it's small enough that both adjacent points are either
      //      //   on or in the domain.  
      //      //
      //      // As long as the ratio of the distances between the knot point and 
      //      //   its 2 neighbors stays constant, C1 continuity will be preserved and
      //      // as long as the direction of the 3pt line segment stays constant,
      //      //   the 1stDeriv direction at the knot point will stay constant.
      //      
      //      // Check neighbors of interior knot points.
      //      double dFraction = 1.1;
      //      SmPoint3d sBasePt;
      //      int iWhichPt = 0;  // -1 for prev, +1 for next
      //      
      //      // prevPoint outside UMin/UMax boundary check
      //      if ( rPrevPt.x < u0 || rPrevPt.x > u1 )
      //        {
      //          iWhichPt = -1;
      //          sBasePt = rNextPt;
      //          double limit = ( rPrevPt.x < u0 ) ? u0 : u1;
      //          dFraction = ( limit - sBasePt.x ) / ( rPrevPt.x - sBasePt.x );
      //        }
      //      
      //      // prevPoint outside VMin/VMax boundary check
      //      if ( rPrevPt.y < v0 || rPrevPt.y > v1 )
      //        {
      //          iWhichPt = -1;
      //          sBasePt = rNextPt;
      //          double limit = ( rPrevPt.y < v0 ) ? v0 : v1;
      //          dFraction = ( limit - sBasePt.y ) / ( rPrevPt.y - sBasePt.y );
      //      
      //        }
      //      
      //      // nextPoint outside UMin/UMax boundary check
      //      if ( rNextPt.x < u0 || rNextPt.x > u1 )
      //        {
      //          iWhichPt = 1;
      //          sBasePt = rPrevPt;
      //          double limit = ( rNextPt.x < u0 ) ? u0 : u1;
      //          dFraction = ( limit - sBasePt.x ) / ( rNextPt.x - sBasePt.x );
      //        }
      //      
      //      // nextPoint outside VMin/VMax boundary check
      //      if ( rNextPt.y < v0 || rNextPt.y > v1 )
      //        {
      //          iWhichPt = 1;
      //          sBasePt = rPrevPt;
      //          double limit = ( rNextPt.y < v0 ) ? v0 : v1;
      //          dFraction = ( limit - sBasePt.y ) / ( rNextPt.y - sBasePt.y );
      //        }
      //      
      //      // Check whether we hit any here.
      //      if ( dFraction < 1.0 )
      //        {
      //          // Pull the bad point, and the knot point,
      //          // towards the base point by dFraction.
      //          SmPoint3d & rPtToPull = ( iWhichPt < 0 ) ? rPrevPt : rNextPt;
      //      
      //          rPtToPull = sBasePt + dFraction * ( rPtToPull - sBasePt );
      //          rKnotPt   = sBasePt + dFraction * ( rKnotPt   - sBasePt );
      //        }

    } // end iter every curve knot

} // end sm_FixBoundaryCurves()

/*******************************************************************//**
PURPOSE:  DropCurve helper function.
    DumpCurve helper function to remove duplicate solutions
    from a GlobalPointSolve() sSolutions array.

NOTES: 
***********************************************************************/
static void sm_RemoveDuplicateDropPointSolutions
 (SmSolutionArray &rSolutions,             // i/o: Array to cull
  double           dUVTolX,                // in : min dist between distinct u points
  double           dUVTolY)                // in : min dist between distinct v points
{
  // locals
  ULONG ii, jj ;

  // Cull duplicate StartPoint Drop solutions whose UV values are within tolerance.
  if ( rSolutions.GetSize() > 1 )
    {
      // for every dropPoint Solution - except the first
      for(ii=1;ii<rSolutions.GetSize();ii++)
        {
          SmPoint2d sUV(rSolutions[ii].m_vStart[0], 
                        rSolutions[ii].m_vStart[1] );

          // for every other dropPoint Solution (earlier in the solution array)
          int ji;
          for(ji=(int)(ii-1);ji>=0;ji--)  // start close to ii.
            {
              jj = (ULONG)ji ;

              SmPoint2d sUV2(rSolutions[jj].m_vStart[0],
                             rSolutions[jj].m_vStart[1]);

              // when points are within tolerance
              if (   smos_Fabs(sUV.x-sUV2.x) < dUVTolX
                 && smos_Fabs(sUV.y-sUV2.y) < dUVTolY )
                {
                  // remove sol last from ordered solution array: later sols have larger error vals.
                  rSolutions.RemoveAt(ii);
                  ii--;  // start next iter at the same ii.
                  break;
                }
            } // end iter all other solutions
        } // end iter all solutions
    } // end more than 1 nearest point solution check

} // end sm_RemoveDuplicateDropPointSolutions

/*******************************************************************//**
PURPOSE: DropCurve helper function.
    Add a BSplineCurve created from the input control point 
    and knot arrays to the output crCurves array and ReSet 
    the input arrays. 

NOTES: 
   When rKnots.Size == 0, makes no changes and returns SM_SUCCESS, else
   Creates a piecewise cubic Bezier curve: KnotMults[4 3 3 ... 4]
                                           Knots = rKnots
                                           CPts  = rCtrlPoly

   when successful:  Adds new curve to crCurves array
                     ReSet rCtrlPoly
                     ReSet rKnots
                     Returns SM_SUCCESS ;
   else returns SM_ERR ;
***********************************************************************/
static SmStatus sm_FlushCrv
  (const SmContext           & crContext,     // in : context for new object construction
   const SmSurface           & crSurface,     // NotUsed: in : tgt surface for DropCurve
   const SmExtent2d          & crUVDomain,    // in : domain of interest for this surface
   const SmCurve             & cr3dCurve,     // NotUsed: in : Curve to project onto the surface
   SmTArray<SmPoint3d>       & rCtrlPoly,     // i/o: control points of piecewise cubic Bezier.
   SmTArray<double>          & rKnots,        // i/o: knot values at breakpoints (single)
   double                      dUVTolX,       // in : UV Tolerance in the U direction
   double                      dUVTolY,       // in : UV Tolerance in the V direction
   SmTArray<SmBSplineCurve*> & crCurves,      // out:
   SmBoolean                 & rbFixBCChange) // out: TRUE = sm_FixBoundaryCurves changed pts before flush, FALSE = didn't 
{
  SM_REF2(crSurface, cr3dCurve) ;
  // no work: no knots
  ULONG lNumKnots = rKnots.GetSize() ;
  if (lNumKnots == 0)
    { return SM_SUCCESS ; }

  // locals
  ULONG i ;
  SmTArray<ULONG> sKnotMult( lNumKnots, NULL, lNumKnots ) ;

  // postprocess the curve
  // 1. Make sure all 'knot' controlPoints are in or on the domain
  // 2. Make sure all 'knot' controlPoints (other than the 1st and last) on the boundary
  //      have tangents parallel to the boundary by moving the neighboring 'internal' ctrlPts to the boundary.
  sm_FixBoundaryCurves( rCtrlPoly, crUVDomain, dUVTolX, dUVTolY, rbFixBCChange );

  // post process the curve
  // 1. merge very short intervals with neighbors when possible
  // GWC TOBEWRITTEN: sm_MergeShortIntervals( sCtrlPoly, ...

  // for every knot - set knot multiplicy = [ 4 3 3 ... 4]
  sKnotMult[0]           = 4 ;
  for(i=1;i+1<lNumKnots;i++) { sKnotMult[i] = 3 ; } // note: can't say lNumKnots-1
  sKnotMult[lNumKnots-1] = 4 ;

  // create degree 3 piecewise C1 BSpline
  SmBSplineCurve *pNewBSP    = NULL ;
  ULONG           lDimension = 2 ;
  ULONG           lDegree    = 3 ;
  SmStatus eStat = SmBSplineCurve::CreateCanonical(crContext,
                                                   lDimension,
                                                   lDegree,
                                                   rCtrlPoly, 
                                                   SM_CF_UNSPECIFIED, 
                                                   sKnotMult, 
                                                   rKnots,
                                                   SM_KT_UNSPECIFIED,
                                                   NULL, 
                                                   NULL, 
                                                   pNewBSP) ;
  if ( eStat != SM_SUCCESS ) 
    { SER( eStat ) ; }

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  if(bDebugMe)
    {
      pNewBSP->Dump() ;
    }
#endif // SM_DEBUG_CODE

// Note that the following line would remove extra knots and
// decrease the size of the curve.  We have chosen not to do
// it here ; it can easily be done after drop curve if desired.
//    SER( pNewBSP->RemoveExtraKnots( SM_EFF_ZERO )) ;

#ifdef SM_DEBUG_CODE
  if(bDebugMe) 
    {
      pNewBSP->Dump() ;
    }
#endif // SM_DEBUG_CODE

  // set output
  crCurves.Add( pNewBSP ) ;
  rKnots.ReSet() ;
  rCtrlPoly.ReSet() ;

  // all done
  return SM_SUCCESS ;

} // end sm_FlushCrv

/*******************************************************************//**
PURPOSE: DropCurve helper function.
    Add a Cubic Bezier segment to the growing 
    control polygon in rCtrlPoly and 
    knot vector in rKnots arrays, 
    flushing discontinuous curves and restarting with input span.

NOTES:
 Returns   SM_ERR when rCtrlPoly point count gets to 10,000.
 Publishes a warning when rCtrlPoly point count gets to 2,000.
   
  1. append dEndT_thisSeg to growing rKnots array:
   [StartT_1stSeg, EndT_1stSeg, EndT_2ndSeg . . . EndT_thisSeg]

  2. append cubicBezier mid and end CtrlPts to growing rCtrlPoly array:
   [Beg_1st Mid1_1st Mid2_1st EndPt_1st Mid1_2nd Mid2_2nd EndPt_2nd ... Mid1_this Mid2_this EndPt_this]

  3. when this new segment is not continuous with the growing curve in either
     param or image space  (sStart.m_dT param != rKnots.last or caStartUV[0] != rCtrlPoly.last)
     then growing Curve is output to crCurves with a call to sm_FlushCrv,
       rCtrlPoly and rKnots are reset, and 
       then restarted with the current input span data.

  4. If the span is zero length, the growing curve is output and rCtrlPoly and rKnots is reset
       and left empty.
***********************************************************************/
static SmStatus sm_AddToCurve
  (const SmContext           & crContext,           // in : in case we create a new curve
   const SmSurface           & crSurface,           // in : tgt Surface for DropCurve
   const SmExtent2d          & crUVDomain,          // in : used to clamp UVCtrlPoints to remove noise when walking boundaries
   const SmCurve             & cr3dCurve,           // in : curve being dropped in DropCurve
   const SmDropPt            & crStart,             // in : Span start drop point
   const SmDropPt            & crEnd,               // in : Span end drop point
   SmTArray<SmPoint3d>       & rCtrlPoly,           // i/o: growing UVcurve polygon - add 3 controlpoints (4 for 1st segment),
                                                    //      When input span is discontinuous with last span: reset to this input span   
                                                    //      note: declared as SmPoint3d but used as SmPoint2d to match up with upcoming SmBSplineCurve::CreateCanonical() call.
   SmTArray<double>          & rKnots,              // i/o: growing curve knot vector - add sEnd.m_dT (and sStart.m_dT for 1st segment),
                                                    //      When input span is discontinuous with last span: reset to this input span   
   SmApproxTol3d               sApproxTol3d,        // in : 
   double                      dUVTolX,             // in : UV tol in u dir
   double                      dUVTolY,             // in : UV tol in v dir                                            
   SmTArray<SmBSplineCurve*> & crCurves,            // out: 1 curve added made from input rCtrlPoly and rknots 
                                                    //      when input span is discontinuous with sCtrlPoly or sKnots last span.
   SmBoolean                 & rbFixBCChange)       // out: when curve flushed, TRUE = ctrlPts changed by sm_FixBoundaryCurve, FALSE = not changed
{
  // locals for hermite curve segment
  double     dScale  = crEnd.m_dT - crStart.m_dT;
  SmPoint2d  sP1(crStart.m_UVPD[0]);
  SmVector2d sD1(crStart.m_UVPD[1] * dScale);
  SmPoint2d  sP4(crEnd.m_UVPD[0]);
  SmVector2d sD2(crEnd.m_UVPD[1]   * dScale);

#ifdef SM_DEBUG_CODE
  if(   sD1.LengthSquared() < SM_EFF_ZERO_SQ
     || sD2.LengthSquared() < SM_EFF_ZERO_SQ)
    {
      SM_ASSERT_MSG(   sD1.LengthSquared() > SM_EFF_ZERO_SQ
                    && sD2.LengthSquared() > SM_EFF_ZERO_SQ, _T("DropCurve helper sm_AddToCurve() asked to build Hermite segment with zero length Tangents")) ;
    }
#endif // SM_DEBUG_CODE

  // Bezier segement interior control points are end points +/- 1st derivs divided by degree
  SmPoint2d sP2 = sP1 + sD1 / 3.0;
  SmPoint2d sP3 = sP4 - sD2 / 3.0;

  // when beg/end pt is on a pole - straighten out dog-leg shapes
  // gwc: this has only been needed to fix UVTrimCurves on Surfaces that have bad ControlNets causing dropPoint confusions.
  //      When the surface control Net is fine FindDegenParamForDirection() works well.
  //      We only fix this here to allow some kinds of poorly defined surfaces to be tessellated.
  //      fixing this here may cause other methods that depend on NewtonRaphson solutions to run into trouble.
  SmBoolean bDogLeg =   (crStart.m_bOnPole || crEnd.m_bOnPole) 
                      ? smgu_IsDogLeg(sP1, sP2, sP3, sP4) 
                      : FALSE ;  
  if(bDogLeg)
   {
     // fetch the singularity direction
     SmSurfParamType eSingularDirection ;
     if(crStart.m_bOnPole) { crSurface.IsSingularity(sP1, eSingularDirection, sApproxTol3d) ; }
     else                  { crSurface.IsSingularity(sP4, eSingularDirection, sApproxTol3d) ; }

     // straighten out pole by moving Pt on Singularity in line with End of segment not on singularity
     if(eSingularDirection == SM_SP_U) // U varies, V const along singularity
       { if(crStart.m_bOnPole) { sP1.x = sP4.x ;
                                 sP2.x = sP3.x ;
                               }
         else                  { sP4.x = sP1.x ;
                                 sP3.x = sP2.x ;
                               }
       }
     else // V varies, U const along singularity
       {
         if(crStart.m_bOnPole) { sP1.y = sP4.y ; 
                                 sP2.y = sP3.y ; 
                               }                 
         else                  { sP4.y = sP1.y ; 
                                 sP3.y = sP2.y ; 
                               }                 
       }
   } // end need to straighten out dog-leg at pole check
                                                   
  // when span is zero length or discontinuous with the growing curve arrays,
  // output current controlPoint and knot lists as a curve. 
  if ( rKnots.GetSize() > 0 )
    {
      SmPoint3d s3dP1(sP1);
      if(   (   dScale < SM_EFF_ZERO)  // zero length span
         || (   smos_Fabs( rKnots.GetLast() - crStart.m_dT )  > SM_EFF_ZERO   // next segment discontinuous in param space
             || s3dP1.DistanceBetween( rCtrlPoly.GetLast() ) > SM_EFF_ZERO)) // next segment discontinuous in image space
        {
          // span is notC0 or zeroLength -  create current curve, restart lists, and carry on.
          SER( sm_FlushCrv( crContext, crSurface, crUVDomain, cr3dCurve, rCtrlPoly, rKnots, dUVTolX, dUVTolY, crCurves, rbFixBCChange ));
        }
    }

  // skip zero length spans
  if(dScale < SM_EFF_ZERO)
    { return(SM_SUCCESS) ; }

  // Clamp points into parameter space
  sP1 = crUVDomain.ClampPoint2d(sP1);
//    sP2 = crUVDomain.ClampPoint2d(sP2);  // These can be out of bounds
//    sP3 = crUVDomain.ClampPoint2d(sP3);  //   without the curve going out.
  sP4 = crUVDomain.ClampPoint2d(sP4);

  // when adding 1st segment - add start CtrlPoint and start knot value
  if (rKnots.GetSize() == 0)
    {
      rKnots.Add( crStart.m_dT );
      rCtrlPoly.Add( SmPoint3d( sP1 ));
    }

  // for all segments - add mid and end CtrlPts and end knot value
  rKnots.Add( crEnd.m_dT );
  rCtrlPoly.Add( SmPoint3d( sP2 ));
  rCtrlPoly.Add( SmPoint3d( sP3 ));
  rCtrlPoly.Add( SmPoint3d( sP4 ));

  // inform the public for large drop point counts
  if (rCtrlPoly.GetSize() == 2000) 
    { MSG(_T("More Than 2000 Control Points for Dropped Curve")); }

  // return error for huge drop point counts
  if (rCtrlPoly.GetSize() >= 10000) 
    { return SM_ERR; }

  // all done
  return SM_SUCCESS;

} // end sm_AddToCurve

#if 0 // Unused
/*******************************************************************//**
PURPOSE: DropCurve helper function.
    This is a DropCurve helper function.
    Find curve closest points and intervals 'near' to surface poles while
    ignoring places where the curve intersects the pole.
   

NOTES: These intervals are needed because the DropCurve algorithm,
    based on surface derivative sizes, has a hard time building good UVTrimCurves
    near surface poles although it does a good job for curves that intersect
    poles. These curve intervals define places on the curve
    where UVCurves must be computed without using the surface derivative values. 
***********************************************************************/
static SmStatus sm_GetNearPoleIvls
 (const SmSurface      & crSurface,      // in : Surface being examined
  const SmExtent2d     & crUVDomain,     // in : Surf Domain of interest
  const SmCurve        & crCurve3d,      // in : CurveOnSurf that may run close to pole
  const SmExtent1d     & crInterval,     // in : Curve interval of interest
  SmApproxTol3d          sApproxTol3d,   // in : min distance between distinct 3d points
  ULONG                  lSingularities, // in : list of surface singularities
                                         //      orof: SM_SS_UMIN, SM_SS_VMIN, SM_SS_UMAX, SM_SS_VMAX
  SmTArray<SmPoint3d>  & rSrfPolePoints, // in : Pole locs ordered:[UMin, VMin, UMax, VMax]
                                         //      pts for nonSingular sides set to SmPoint2d::SetUninitialized()
  SmTArray<SmExtent1d> & rCrvPoleIvls,   // out: intervals over which the curve is 'near' the pole
  SmTArray<SmPoint3d>  & rCrvPolePoints, // out: associated closest point to pole for each interval
                                         //      note: assume one closest point to the pole per interval
  SmTArray<double>     & rCrvPoleParams, // out: associated curve parameters for every closest point in sCrvPolePoints
  SmTArray<ULONG>      & rCrvPoleSides,  // out: associated surface side for each pole interval
                                         //      oneof: 0 = SM_SS_UMIN_INDEX, 1 = SM_SS_VMIN_INDEX, 2 = SM_SS_UMAX_INDEX, 3 = SM_SS_VMAX_INDEX
                                         //      can be used directly as indices into rSrfPolePoints
  SmTArray<SmBoolean>  & rCrvPoleHits)   // out: associated closest point classification, TRUE = point hits pole, FALSE = just near
{
  // init output
  rCrvPoleIvls.ReSet() ; 
  rCrvPolePoints.ReSet() ;
  rCrvPoleParams.ReSet() ;
  rCrvPoleSides.ReSet() ;

  // locals
  ULONG ii, jj ;
  SmSolutionArray sSolutions ;
  SmExtent3d      sSurfBox ;
  SmBoolean       bSingular = FALSE ;
  SmPoint3d       sPV[2], sToPole ;
  SmPoint2d       sGuessUV, sDropUV ;
  SmSolution      sSolution ;

  // surface size
  crSurface.CalculateBoundingBox(crUVDomain, &sSurfBox) ;
  double dSurfSize = sSurfBox.GetMaxLength() ;

  // pick near pole distance defining the outer limit of the pole 'neighborhood'
  double dNearPoleDist = .01 * dSurfSize ;

  // pick intersection tolerance - a tight tolerance is needed to prevent any snapping.
  double dXSectTol3d = SM_EFF_ZERO * (1.0 + dSurfSize) ;

  // curve classification container
  SmCurveClassification sCurveClassification(&crCurve3d, crInterval, NULL, dXSectTol3d) ;

  // for every surface side
  for(ii=0;ii<4;ii++)
    {
      // skip NonSingular sides
      bSingular =(   (ii == 0 && lSingularities & SM_SS_UMIN) 
                  || (ii == 1 && lSingularities & SM_SS_VMIN)
                  || (ii == 2 && lSingularities & SM_SS_UMAX)
                  || (ii == 3 && lSingularities & SM_SS_VMAX)) ;
      if(!bSingular)
        { continue ; }

      // find points on curve exactly dNearPoleDist from curve
      crCurve3d.GlobalPropertyAnalysis(crInterval,
                                       SM_CP_DISTANCE_TO_POINT,
                                      &dNearPoleDist, 
                                      &rSrfPolePoints[ii],
                                       dXSectTol3d,
                                       sSolutions) ;
  
      // parse the solutions
      for(jj=0;jj<sSolutions.GetSize();jj++)
        {
          SmSolution &rSolution = sSolutions[jj] ;
          SmSolutionEnd &rStart = rSolution.m_vStart ;

          // only expect point solutions
          if(rSolution.m_eSolutionType != SM_ST_SINGLE_VALUE)
            {
              // if this branch is ever hit - need to add code here - not currently hit in prog_test
              SE_MSG(SM_ERR, _T("Unexpected branch hit - need to add code here NOW!!!")) ;

              // if this happens just add the start and end solutions to the CurveClassification and
              // label the intervening interval as SM_PC_POINT
              continue ;
            }

          // for point solutions - add a PointClassification to the CurveClassification
          if(rSolution.m_eSolutionType == SM_ST_SINGLE_VALUE)
            {
              // locals
              SmBoolean bInsertionMade ;
              double    dCurveParam = rStart.m_adParameters[0] ;

              // the point classification
              SmPointClassification sPointClassification ;
              sPointClassification.SetPointClass(SM_PC_POINT) ;
              sPointClassification.SetDeviation(rStart.m_dSolutionValue) ;
              sPointClassification.SetTolerance3d(dXSectTol3d) ;
              sPointClassification.SetTParam(ii) ; // begin awful hack temporarily using TParam to store lPoleIndex 
              sPointClassification.SetPreSnapParam(dCurveParam) ;

              // Insert the point solution into the CurveClassification
              sCurveClassification.InsertPointClass
                (sPointClassification,     // in : Point to insert or merge                                                  
                 dCurveParam,              // in : associated parameter value                                                
                 SM_BIG_DOUBLE,            // in : When inserting a sequence of pointClass objects                           
                                           //      is used to prevent two objects from both being                            
                                           //      classified to one interval endPoint.                                      
                                           //      Set to SM_BIG_DOUBLE to ignore.                                           
                 bInsertionMade,           // out: TRUE = inserted, FALSE=merged                                             
                 TRUE) ;                   // in : TRUE = force point insertion even when                                    
                                           //             point is close to existing classification points                   
                                           //      FALSE = use internal heuristics to decide when to merge and when to insert
              SM_ASSERT(bInsertionMade == TRUE) ;
            } // end Point Solution branch
        } // end iter every solution
    } // end iter every surface side - looking for CurveIntervals close to SurfacePoles


#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  // draw 
  if(bDebugMe)
    {
      sCurveClassification.Dump() ;

      SmFace *pFace = (SmFace *)crSurface.GetFace() ;
      SmEdge *pEdge = (SmEdge *)crCurve3d.GetEdge() ;
      SmBrep *pBrep =   pFace ? pFace->GetBrep() 
                      : pEdge ? pEdge->GetBrep() 
                      : NULL ;

      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,1) ; crSurface.DrawUV() ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,0,0) ; if(pFace) pFace->Draw(SM_DM_CROSSHATCH) ; sm_GraphicsLoop() ;
      smgfx_SetLook(3,4, 1,0,0) ; crCurve3d.Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook(4,6, 1,0,1) ; if(pEdge) pEdge->Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook(5,8, 0,1,0) ; sCurveClassification.Draw(TRUE) ; sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

  // no work - curve does not wander near pole
  if(   sCurveClassification.GetSize() == 1
     && sCurveClassification[0].m_vStart.GetPointClass() == SM_PC_UNKNOWN
     && sCurveClassification[0].m_vEnd.GetPointClass()   == SM_PC_UNKNOWN)
    {
      return(SM_SUCCESS) ; 
    }

  // classify the intervals
  for(ii=0;ii<sCurveClassification.GetSize();ii++)
    {
      SmCurveInterval       & rCurveInterval  = sCurveClassification[ii] ;
      SmPointClassification & rStart          = rCurveInterval.m_vStart ;
      SmPointClassification & rMid            = rCurveInterval.m_vMid ;
      SmPointClassification & rEnd            = rCurveInterval.m_vEnd ;
      SmExtent1d              sSubIvl         = rCurveInterval.m_vInterval ;
      ULONG                   lStartPoleIndex = (ULONG)rStart.GetTParam() ; // awful hack temporarily using TParam to store lPoleIndex
      ULONG                   lEndPoleIndex   = (ULONG)rEnd.GetTParam() ;   // awful hack temporarily using TParam to store lPoleIndex

      // check state
      SER_MSG(  ((rStart.GetPointClass() == SM_PC_POINT) || (rEnd.GetPointClass() == SM_PC_POINT))
              ? SM_SUCCESS : SM_ERR,
              _T("Something is wrong with the near pole curve classification - debug NOW!!!")) ;
          
      // skip already classified intervals
      if(rMid.GetPointClass() != SM_PC_UNKNOWN)
        { continue ; }

      // when both ends classify to different poles - not near a pole, this is a span moving from pole to pole
      if(   rStart.GetPointClass() == SM_PC_POINT
         && rEnd.GetPointClass()   == SM_PC_POINT
         && lStartPoleIndex != lEndPoleIndex)
        {
          rMid.SetPointClass(SM_PC_NOTHING) ;
          
          // classify next interval
          continue ;
        }

      // cheap classify from start point - check curve tangent direction at start point
      if(rStart.GetPointClass() == SM_PC_POINT)
        {
          crCurve3d.Evaluate(sSubIvl.GetMin(), 1, TRUE, sPV) ; // nonZeroTangents

          // see if curve is entering or leaving a range close to the pole point
          sToPole = rSrfPolePoints[lStartPoleIndex] - sPV[0] ;

          // when angle between tangent and ToPole vec is unambiguous
          if(!sToPole.IsPerpendicularTo(sPV[1], 2.0)) 
            {
              // use the dot product to classify the neighboring intervals
              if(sToPole.Dot(sPV[1]) > 0.0) 
                { 
                  // upper interval is near pole - lower interval is not
                  rMid.SetPointClass(SM_PC_POINT) ;
                  rMid.SetTParam(lStartPoleIndex) ;  // begin awful hack temporarily using TParam to store lPoleIndex
                  if(ii > 0 && sCurveClassification[ii-1].m_vMid.GetPointClass() == SM_PC_UNKNOWN)
                    { 
                      sCurveClassification[ii-1].m_vMid.SetPointClass(SM_PC_NOTHING) ;
                    }
                }
              else
                { 
                  // lower interval is near pole - upper interval is not
                  rMid.SetPointClass(SM_PC_NOTHING) ;
                  if(ii > 0 && sCurveClassification[ii-1].m_vMid.GetPointClass() == SM_PC_UNKNOWN)
                    { 
                      sCurveClassification[ii-1].m_vMid.SetPointClass(SM_PC_POINT) ;
                      sCurveClassification[ii-1].m_vMid.SetTParam(lStartPoleIndex) ; // begin awful hack temporarily using TParam to store lPoleIndex
                    }
                }

              // classify next interval
              continue ;
            } // end start curve tangent direction can classify interval branch
        } // end SubIvl StartPoint is a near pole boundary check

      // arrive here when startPoint does not classify boundary - try to classify ivl with upper point

      // cheap classify from end point - check curve tangent direction at end point
      if(rEnd.GetPointClass() == SM_PC_POINT)
        {
          crCurve3d.Evaluate(sSubIvl.GetMax(), 1, FALSE, sPV) ; // nonZeroTangents

          // see if curve is entering or leaving a range close to the pole point
          sToPole = rSrfPolePoints[lEndPoleIndex] - sPV[0] ;

          // when angle between tangent and ToPole vec is unambiguous
          if(!sToPole.IsPerpendicularTo(sPV[1], 2.0)) 
            {
              // use the dot product to classify the neighboring intervals
              if(sToPole.Dot(sPV[1]) > 0.0) 
                { 
                  // upper interval is near pole - lower interval is not
                  rMid.SetPointClass(SM_PC_NOTHING) ;
                  if(   (ii < sCurveClassification.GetSize() - 1) 
                     && sCurveClassification[ii+1].m_vMid.GetPointClass() == SM_PC_UNKNOWN)
                    { 
                      sCurveClassification[ii+1].m_vMid.SetPointClass(SM_PC_POINT) ;
                      sCurveClassification[ii+1].m_vMid.SetTParam(lEndPoleIndex) ; // begin awful hack temporarily using TParam to store lPoleIndex
                    }
                }
              else
                { 
                  // lower interval is near pole - upper interval is not
                  rMid.SetPointClass(SM_PC_POINT) ;
                  rMid.SetTParam(lEndPoleIndex) ;  // begin awful hack temporarily using TParam to store lPoleIndex
                  if(   (ii < sCurveClassification.GetSize() - 1) 
                     && sCurveClassification[ii+1].m_vMid.GetPointClass() == SM_PC_UNKNOWN)
                    { 
                      sCurveClassification[ii+1].m_vMid.SetPointClass(SM_PC_NOTHING) ;
                    }
                }

              // classify next interval
              continue ;
            } // end start curve tangent direction can classify interval branch
        } // end SubIvl StartPoint is a near pole boundary check

      // arrive here when neither startPoint nor endPoint classify the interval - try the mid point
      crCurve3d.EvaluatePoint(sSubIvl.GetMid(), sPV[0]) ;

      // check distance to pole
      ULONG lPoleIndex = (rStart.GetPointClass() == SM_PC_POINT) ? lStartPoleIndex : lEndPoleIndex ;
      sToPole = rSrfPolePoints[lPoleIndex] - sPV[0] ;
      double dDistToPole = sToPole.Length() ;

      if(dDistToPole < dNearPoleDist - dXSectTol3d)
        {
          rMid.SetPointClass(SM_PC_POINT) ;
          rMid.SetTParam(lPoleIndex) ;  // begin awful hack temporarily using TParam to store lPoleIndex
        }
      else if(dDistToPole > dNearPoleDist + dXSectTol3d)
        {
          rMid.SetPointClass(SM_PC_NOTHING) ;
        }
      else
        {
          // currently not hit in prog_test
          SER_MSG(SM_ERR, _T("Unexpected branch - all points within dXSectTol3d should be already be marked as point classifications")) ;
        }
    } // end iter every interval - getting interval classifications

#ifdef SM_DEBUG_CODE
  if(bDebugMe)
    {
      sCurveClassification.Dump() ;
    }
#endif // SM_DEBUG_CODE 

  // set output
  for(ii=0;ii<sCurveClassification.GetSize();ii++)
    {
      SmCurveInterval       & rCurveInterval  = sCurveClassification[ii] ;
      SmPointClassification & rMid            = rCurveInterval.m_vMid ;
      ULONG                   lPointIndex     = (ULONG)rMid.GetTParam() ; // end of awful hack temporarily using TParam to store lPoleIndex
      SmExtent1d              sSubIvl         = rCurveInterval.m_vInterval ;
      
      // skip not 'near' intervals
      if(rMid.GetPointClass() != SM_PC_POINT)
        { continue ; }

      // Find point in interval closest to PolePoint
      SmBoolean bFoundAnswer ;
      crCurve3d.LocalPointSolve(sSubIvl,                     // in : search interval                             
                                SM_SO_MINIMIZE,              // in : specify specific operation to optimize      
                                rSrfPolePoints[lPointIndex], // in : point specializing this search              
                                NULL,                        // in : Max allowed solution distance        
                                NULL,                        // in : Used only for SM_SO_AT_DISTANCE 
                                NULL,                        // in : Vector direction        
                                sSubIvl.GetMid(),            // in : search starting parameter                   
                                bFoundAnswer,                // out: TRUE=converged,FALSE=didn't                 
                                sSolution) ;                 // out: solution container for solver               
      
      SM_ASSERT_MSG(bFoundAnswer == TRUE, _T("Every Interval must have a closest point")) ;
      SM_ASSERT(sSolution.m_eSolutionType == SM_ST_SINGLE_VALUE) ;

      // when everything is working properly - load the output arrays
      if(bFoundAnswer)
        {
          crCurve3d.EvaluatePoint(sSolution.m_vStart[0], sPV[0]) ;

          // save current output - everything but the PoleHit classification
          rCrvPoleIvls.  Add(sSubIvl) ;
          rCrvPolePoints.Add(sPV[0]) ;
          rCrvPoleParams.Add(sSolution.m_vStart[0]) ;
          rCrvPoleSides. Add(lPointIndex) ;

          // classify the CrvPolePoint as either near or on the pole

          // surface drop point guess
          sGuessUV.Set(  lPointIndex == SM_SS_UMIN ? crUVDomain.GetUMin()
                       : lPointIndex == SM_SS_UMAX ? crUVDomain.GetUMax()
                       : (crUVDomain.GetUMin() + crUVDomain.GetUMax()) / 2.0,

                         lPointIndex == SM_SS_VMIN ? crUVDomain.GetVMin()
                       : lPointIndex == SM_SS_VMAX ? crUVDomain.GetVMax()
                       : (crUVDomain.GetVMin() + crUVDomain.GetVMax()) / 2.0) ;

          // drop CrPolePoint to surface
          SER( crSurface.LocalPointSolve( crUVDomain,           // in : surface domain of interest
                                          SM_SO_MINIMIZE,       // in : operation (always a solution)
                                          sPV[0],               // in : 3dPoint being dropped
                                          sGuessUV,             // in : guess UVPoint
                                          bFoundAnswer,         // out: TRUE = found answer, else FALSE
                                          sSolution ));         // out: contains the surface point cloest to the 3dPoint
          SM_ASSERT(bFoundAnswer == TRUE) ;

          // classify srfDropPoint as singular or not
          sDropUV.Set(sSolution.m_vStart[0], sSolution.m_vStart[1]) ;
          SmSurfParamType eSingularDirection ;
          SmBoolean bPoleHit = crSurface.IsSingularity(sDropUV, eSingularDirection, sApproxTol3d) ;
          SM_ASSERT(   (bPoleHit == FALSE)
                    || (eSingularDirection == SM_SP_U && (lPointIndex == SM_SS_VMIN_INDEX || lPointIndex == SM_SS_VMAX_INDEX))
                    || (eSingularDirection == SM_SP_V && (lPointIndex == SM_SS_UMIN_INDEX || lPointIndex == SM_SS_UMAX_INDEX))) ;

          // save the output
          rCrvPoleHits.  Add(bPoleHit) ;
        }
    } // end iter every interval - setting output

#ifdef SM_DEBUG_CODE
  // draw 
  if(bDebugMe)
    {
      TCHAR sBuff[SM_TBLOCK_SIZE] ;

      sCurveClassification.Dump() ;
      smos_sprintf(sBuff,_T("\nNumber of NearPole Ivls: %ld"), rCrvPoleIvls.GetSize()) ;
      smos_WriteBuffer(sBuff) ;
      for(ii=0;ii<rCrvPoleIvls.GetSize();ii++)
        {
          smos_sprintf(sBuff,_T("\n Interval %ld : "),ii) ; smos_WriteBuffer(sBuff) ; rCrvPoleIvls[ii].Dump() ;
          smos_sprintf(sBuff,_T("%s"),_T("\n   CrvPolePoint : ")) ;    smos_WriteBuffer(sBuff) ; rCrvPolePoints[ii].Dump() ;
          smos_sprintf(sBuff,_T("\n   CrvPoleParam : %16.16lf"),rCrvPoleParams[ii]) ; smos_WriteBuffer(sBuff) ; 
          smos_sprintf(sBuff,_T("\n   CrvPoleSide  : %s"),  rCrvPoleSides[ii] == 0 ? _T("UMin")
                                                        : rCrvPoleSides[ii] == 1 ? _T("VMin")
                                                        : rCrvPoleSides[ii] == 2 ? _T("UMax")
                                                        : rCrvPoleSides[ii] == 3 ? _T("VMax")
                                                        : _T("Error")) ; smos_WriteBuffer(sBuff) ;
         smos_WriteBuffer(_T("\n")) ; 
        }

      SmFace *pFace = (SmFace *)crSurface.GetFace() ;
      SmEdge *pEdge = (SmEdge *)crCurve3d.GetEdge() ;
      SmBrep *pBrep =   pFace ? pFace->GetBrep() 
                      : pEdge ? pEdge->GetBrep() 
                      : NULL ;

      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,1) ; crSurface.DrawUV() ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,0,0) ; if(pFace) pFace->Draw(SM_DM_CROSSHATCH) ; sm_GraphicsLoop() ;
      smgfx_SetLook(3,4, 1,0,0) ; crCurve3d.Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook(4,6, 1,0,1) ; if(pEdge) pEdge->Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook(5,8, 0,1,0) ; sCurveClassification.Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(6,10,1,0,0) ; for(ii=0;ii<rCrvPoleIvls.GetSize();ii++)
                                    { rCrvPolePoints[ii].Draw() ; sm_GraphicsLoop() ; }
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

  // all done 
  return(SM_SUCCESS) ;

} // end sm_GetNearPoleIvls
#endif // 0 = unused

/*******************************************************************//**
PURPOSE: DropCurve helper function.
    This is a DropCurve helper function.
    Use binary search to find the point on a curve that drops to a surface
    Domain boundary.
   
NOTES: 
  returns SM_SUCCESS = Found a boundary point
          SM_ERR     = Never found a point on the surface or 
                        never found a point on the boundary
                          (happened in prog_test due to seam skipping and jumping between
                             different local solutions - my_test_suite_2 iter: 202, a confused case)
***********************************************************************/
SmStatus SmDropPt::IterateToBoundary     // in : m_dT, m_UVPD, m_bFromLeftU, m_bFromLeftV, m_bGoodDrop
 (const SmSurface & crSurface,            // in : target surface
  SmOffsetSurface & rOffset,              // in : target surface offset  
  ULONG             lSingularities,       // in : orof: SM_SS_NONE, SM_SS_UMIN, SM_SS_VMIN, SM_SS_UMAX, SM_SS_VMAX
  SmExtent2d      & rUVDomain,            // in : Surface domain   
  SmSurfParamType   eSrfClosure,          // in : oneof: SM_SP_U, SM_SP_V, SM_SP_BOTH, SM_SP_NEITHER                 
  const SmCurve   & crCurve,              // in : Curve being dropped
  SmApproxTol3d     sApproxTol3d,         // in : max allowed distance between drop point and surfNormal line at drop point                                                                   
  double            dUVTolX,              // in : UV tol in u dir
  double            dUVTolY,              // in : UV tol in v dir  
  const SmDropPt  & crEnd,                // in : m_dT, m_UVPD, m_bFromLeftU, m_bFromLeftV, m_bGoodDrop
  SmDropPt        & rDropPt)              // out: Drop to Boundary DropPt
 const
{
  // init output
  SmStatus sRetStatus = SM_ERR ;

  // locals
  SmDropPt       sThisStart = *this ;
  double         dThisEndT  = crEnd.m_dT ;
  double         dThisT     = crEnd.m_dT ;
  SmDropPt       sThisDropPt(sThisStart) ;
  SmExtent1d     sCrvIvl = crCurve.GetNaturalInterval() ;
  SmVector2d     sIntoDomain ;
  SmBoolean      bHitSurface = FALSE ;

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  if(bDebugMe)
    {
      sThisStart.Dump(sApproxTol3d, _T("Start DropPt")) ;
      crEnd.Dump(sApproxTol3d, _T("End DropPt")) ;
    }
#endif // SM_DEBUG_CODE

  // quit when a boundary point is found or span gets too small
  SmBoolean bDone = (   (   sThisStart.m_bGoodDrop 
                         && sThisStart.m_bOnBoundary
                         && sThisStart.m_bLeavingDomain == TRUE)
                     || SM_IS_ZERO(dThisT - sThisStart.m_dT)) ;

  // Try a binary search to find the crossing m_dT value
  // until we converge onto a boundary point
  while(!bDone)
    {
      // subdivide the problem span - set sEnd.m_dT to current Span midPoint
      dThisT = (sThisStart.m_dT + dThisEndT) / 2.0 ;

      // try mapping rdThisT curve param to Surface sThisSrfUV
      sRetStatus = sThisStart.GetNextPoint
        (crSurface,                // in : main surface                                                       
         rOffset,                  // i/o: offset of main surface, its offset distance is set in this routine 
         lSingularities,           // in : orof: SM_SS_NONE, SM_SS_UMIN, SM_SS_VMIN, SM_SS_UMAX, SM_SS_VMAX
         nullptr,                  // in : TODO
         rUVDomain,                // in : limiting domain of the surface                                     
         eSrfClosure,              // in : oneof: SM_SP_U, SM_SP_V, SM_SP_BOTH, SM_SP_NEITHER                 
         crCurve,                  // in : Curve being dropped                                                
         sApproxTol3d,             // in : max allowed distance between drop point and surfNormal line at drop point                                                                   
         dUVTolX,                  // in : UV tol in u dir
         dUVTolY,                  // in : UV tol in v dir
         dThisT,                   // in : Param value of EndSpan Curve point being dropped                                           
         sThisDropPt) ;            // out: rEnd.m_dT
                                   //      rEnd.m_CrvPD 
                                   //      rEnd.m_UVPD, rEnd.m_bFromLeftU, rEnd.m_bFromLeftV,
                                   //      rEnd.m_3dPD
                                   //      rEnd.m_dDropToSurf,        rEnd.m_dApproxDev
                                   //      rEnd.m_dNormalToCrvAngDeg,
                                   //      rEnd.m_dSrfToCrvAngDeg,    rEnd.m_dSrfToCrvSpeedRatio
                                   //      rEnd.m_bOnBoundary,        rEnd.m_bLeavingDomain,  rEnd.m_bEnteringDomain
                                   //      rEnd.m_eDropDerivRtn   
                                   //         SM_SUCCESS = good drop, 
                                   //         SM_ERR_NOT_WITHIN_TOLERANCE = CrvPt not with tol of SrfNormalLine at dropPoint
                                   //                usually caused by dropping a CrvPt not over the Srf to a SrfBoundary
                                   //         SM_ERR_OUTSIDE_OF_DOMAIN = StartPt drops to SrfBoundary and DropCrv moves out of srf 
                                   //         SM_ERR_BAD_TANGENT_DROP  = CrvTangent dropped to nonParallel SrfTangent
                                   //                usually caused by dropping a CrvTangent near a srfPole 

#ifdef SM_DEBUG_CODE
  if(bDebugMe)
    {
      sThisDropPt.Dump(sApproxTol3d, _T("This DropPt")) ;

      smgfx_Erase() ;
      SmExtent1d sIvl( crCurve.GetNaturalInterval() );
      // draw Brep, Surface (UV, seams, poles, params), Face, Curve (speed, params)
      my_AssertAndDrawDropCurve(&crCurve, sIvl, &crSurface) ;
      smgfx_SetLook(1,11, 1,0,0) ; sThisStart.Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook(2,11, .2,.5,.5) ; sThisDropPt.Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook(3,11, 1,.5,0) ; crEnd.Draw() ; sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE
      // when still out of bounds - try splitting interval again
      if(sThisDropPt.m_bGoodDrop == FALSE)
        { dThisEndT = sThisDropPt.m_dT ; }

      // when in bounds - check to see if we are on a boundary
      if(sThisDropPt.m_bGoodDrop == TRUE)
        {
          // remember when we hit a surface
          bHitSurface = TRUE ;

          // save this eval as the next iter thisStart value
          sThisStart = sThisDropPt ;

          // done when ThisStart is now on a boundary
          bDone = (   sThisStart.m_bGoodDrop      == TRUE
                   && sThisStart.m_bOnBoundary    == TRUE
                   && sThisStart.m_bLeavingDomain == TRUE) ;
        }

      // when done
      if(bDone)
        {
          // Save output
          rDropPt = sThisDropPt ;
        }

      // quit when subdivision interval gets too small
      if(   !bDone 
         && (   SM_IS_ZERO((dThisEndT - sThisStart.m_dT)/2.0)   // tight tol when walking to border
             || (   bHitSurface == FALSE                        // loose tol when looking for any surface hit
                 && this->m_bGoodDrop == FALSE 
                 && crEnd.m_bGoodDrop == FALSE
                 && SM_IS_ZERO_TO_TOL((dThisEndT - sThisStart.m_dT)/2.0, sCrvIvl.GetLength()/150.0))))
        {
          bDone      = TRUE ;
          sRetStatus = SM_ERR ; 
        }

    } // end while binary search for boundary

#ifdef SM_DEBUG_CODE
  if(bDebugMe)
    {
      rDropPt.Dump(sApproxTol3d, _T("This DropPt")) ;
    }
#endif // SM_DEBUG_CODE

  // all done
  return(sRetStatus) ;

} // end SmDropPt::IterateToBoundary

/*******************************************************************//**
PURPOSE: This method creates a parameter space image of a 3D curve
    normal projection to the surface (projection along surface normals).

NOTES:
    Only returns results when the entire curve drops to the surface.
    For curves known to be on the surface use SmSurface::DropAndTrimCurve()
    to get partial curve drop results.

    Curves which cross seams or poles are treated the same as curves
    which walk off the surface, no UVTrimCurve is created and an
    error code is returned.

    Drop a 3D trimming curve to create 1 (or 2) 2D trimming curves.
    Creates 2 trimming curves as a special case when the surface is
    closed and the 3D curve happens to be coincident with the seam.

    The produced UVTrimCurves will be B-Spline curves with
    approximately the same parameterization as the 3D curve.

   The following restrictions apply:
    1) The curve must lie on or very near the surface.
    2) The curve may have C0 discontinuities but the surface within the
           specified domain must be at least G1.  In other words, the
           surface must be smooth inside of crUVDomain.
    3) The 3D curve should not cross seams of closed surfaces
           and should lie inside the UVDomain specified as input.
    4) If a curve lies completely on the seam then two parameter space
           curves will be returned.

  If the 3d curve does cross a domain boundary (#3 above):
  In practice, this happens in either of two cases:
   1. the curve runs along the boundary, but wanders in and out
      due to tolerance-sized noise, or
   2. the curve runs across the boundary: just one crossing.

  In case 1, this will return the whole curve, with some parts
  wandering out of the domain.
  If the boundary is a seam, it will return two curves,
  one on each side of the domain.

  In case 2, The curve will be treated as any curve that is not
  completely contained over the surface and the method will return
  with no results.

  So if two curves are returned, it's always a curve running along a seam.
  
  NOTE: if the curve is near the surface and expected to cross 
  boundaries of the surface, then DropAndTrimCurve() will return 
  a separate uv curve for each piece of the curve, between 
  intersections with the boundaries.
***********************************************************************/
SmStatus SmSurface::DropCurve
 (
  const SmContext           & crContext,           // in : context for new object construction
  const SmExtent2d          & crUVDomain,          // in : domain of interest for this surface
  const SmCurve             & cr3dCurve,           // in : Curve to project onto the surface
  const SmExtent1d          & crInterval,          // in : interval of interest for target curve
  SmApproxTol3d               sApproxTol3d,        // in : sApproxTol3d = max allowed distance between drop point and m_SrfNormal line at drop point
  double                    & rdMaxDropToSurf,     // out: MaxDist(DropPt, 3dCurvePt), found by sampling, may be slightly less than actual max.
  double                    & rdMaxApproxDev,      // out: MaxDist(DropPt->SrfNormalLine, 3dCurvePt), Quality Measure: expected to be close to 0.0
  SmTArray<SmBSplineCurve*> & rUVCurves,           // out: 1 (or 2) curves constructed by projection.
                                                   //      (2 curves for closed surfaces when cr3dCurve is coincident with seam)
  SmBoolean                   bKeepAllDropCurves,  // in : TRUE = return all drop curves that stay over the surface (those that wander off do not drop), 
                                                   //      FALSE= only return drop curves with drop distances less than sApproxTol3d
                                                   //      default:[TRUE]
  SmDropCurveFail           * pOptDropCurveFail,   // out: Optional data container of a DropCurve fail or success
                                                   //      NUll to ignore, default:[NULL]
  SmBoolean                   bCreatingUVTrimCurve // NotUsed: in : TRUE when creating a UVTrimCurve. This forces the UVTrimcurve parameterization to match crInterval
 ) const
{
  SM_REF1(bCreatingUVTrimCurve) ;
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
static ULONG lCount = 0 ; lCount++ ;
#endif // SM_DEBUG_CODE

  // always gather DropCurve data
  SmDropCurveFail sDropCurveFail, * pDropCurveFail = pOptDropCurveFail ? pOptDropCurveFail : &sDropCurveFail ;

  // init output - also make rUVCurves temporary in case of failure - will be set to permanent when successful
  rUVCurves.ReSet() ;
  SmObjsDelete<SmBSplineCurve*> sCleanUVCrvs(&rUVCurves) ;
  rdMaxDropToSurf = 0.0 ;  // 3d drop dist from 3dCurve to surface
  rdMaxApproxDev  = 0.0 ;  // 3d approx dist between approx UVTrimCurve to 'exact' UVDropCurve
  pDropCurveFail->UnInit() ;  

  // locals
  ULONG ii, jj, kk ;
  SmExtent1d       sInterval(crInterval) ;
  SmScaledZero     sCurveScaledZero = SmTol::GetScaledZero(cr3dCurve) ; 
  SmBoolean        bDegenCurve = cr3dCurve.IsDegenerate(sCurveScaledZero, &crInterval) ;
  SmDropPt         sStart(sInterval.GetMin(), bDegenCurve) ;
  SmDropPt         sEnd  (sInterval.GetMax(), bDegenCurve) ;
  double           dDropToSurf, dMaxDropToSurf = 0.0 ;
  double           dApproxDev,  dMaxApproxDev  = 0.0 ;
  // double           dDropToSurfParam, dMaxDropToSurfParam ; 
  SmSolution       sSols[10] ;
  SmSolutionArray  sSolutions(10,sSols) ;
  // SmBoolean        bIsReversed = FALSE ;
  SmCurve        * p3dCurve = (SmCurve *)&cr3dCurve ;

  // watch out for degenerate UVDomains
  SmExtent2d sUVDomain = (   crUVDomain.XLength() >= SM_EFF_ZERO
                          && crUVDomain.YLength() >= SM_EFF_ZERO)
                        ? crUVDomain
                        : GetNaturalUVDomain() ;
  SmBoolean bDegenSurfU            = sUVDomain.XLength() < SM_EFF_ZERO ;
  SmBoolean bDegenSurfV            = sUVDomain.XLength() < SM_EFF_ZERO ;
  SmBoolean bDropDomainIsContained = sUVDomain.IsContainedBy(GetNaturalUVDomain(), SM_EFF_ZERO) ;
  
  // no work - degen surf domain
  if(   bDegenSurfU
     || bDegenSurfV)
    {
      SM_SET1_DROP_CURVE_FAIL(SM_ERR_DEGENERATE_SURFACE, SM_CTC_DEGENSURFACE_FAIL) ;      // 1034
#ifdef SM_DEBUG_CODE  
      if(bDebugMe)
        {
          pDropCurveFail->Dump() ;

          smgfx_Erase() ;
          pDropCurveFail->Draw() ; sm_GraphicsLoop() ;
          sm_GraphicsLoop() ;
        }
#endif // SM_DEBUG_CODE
      SER(SM_ERR) ;
    } // end no work - degen surf domain check

  // check state - given UVDomain must fit within the Surface's Natural UVDomain
  if(bDropDomainIsContained == FALSE)
    {
      SM_SET1_DROP_CURVE_FAIL(SM_ERR_OUTSIDE_OF_DOMAIN, SM_CTC_DROPDOMAIN_UNCONTAINED_FAIL) ;   // 1020
#ifdef SM_DEBUG_CODE  
      if(bDebugMe)
        {
          pDropCurveFail->Dump() ;

          smgfx_Erase() ;
          pDropCurveFail->Draw() ; sm_GraphicsLoop() ;
          sm_GraphicsLoop() ;
        }
#endif // SM_DEBUG_CODE           

      SER_MSG( bDropDomainIsContained ? SM_SUCCESS : SM_ERR,
              _T("SmSurface::DropCurve given crUVDomain that is bigger than Surface->GetNaturalUVDomain()")) ;
    } // end no work - error given UVDomain must fit within the Surface's Natural UVDomain

  // UV Tolerances - proportional to surface 3d/UV mapping size
  // Note, this should use the surface's domain, not the possibly smaller face domain.
  SmExtent2d sSrfNatDomain = GetNaturalUVDomain() ;
  SmVector2d sProjectSize = ApproxDerivativeLengths(sSrfNatDomain) ;
  double     dUVTolX      = sApproxTol3d / (sProjectSize.x + 1.0) ;  
  double     dUVTolY      = sApproxTol3d / (sProjectSize.y + 1.0) ;

  // (Did check this case: ok. [B409])
  //SM_ASSERT_MSG(dUVTolX < .001 * sUVDomain.XLength(), _T("DropCurve U Tolerance calculation is too large - check this case") ) ;
  //SM_ASSERT_MSG(dUVTolY < .001 * sUVDomain.YLength(), _T("DropCurve V Tolerance calculation is too large - check this case") ) ;

  // temporarily enable slight out-of-bounds convergence to do our job until we leave this scope.
  SmBoolean bTmp0 = FALSE ; 
  SmBSplineSurface * pBSplineSurface0 = SM_CAST_NONNULL_PTR(SmBSplineSurface, this) ; 
  SmTemporaryChangeValue<SmBoolean> sClean0(pBSplineSurface0 ? pBSplineSurface0->GetOutOfBoundsEnabled() : bTmp0, TRUE) ;

  // Ensure the surface has a cache
  SmSurfaceCache *pSC = smsurf_GetSurfaceCache(this) ;
  SmCacheCheckOutIn sCheckIO(pSC) ;

  // Turn off point testing so GlobalPointSolve() will keep all point solutions
  //          without classifying the solution point against the trim boundaries.
  // Turn on boundary curve processing to force LocalSolve to look for drop points
  //         on boundary curves.
  SmBoolean bTempBool = TRUE;
  SmTemporaryChangeValue<SmBoolean> sChange1(pSC ? pSC->m_bPointTestEnabled      : bTempBool, FALSE) ;
  SmTemporaryChangeValue<SmBoolean> sChange2(pSC ? pSC->m_bProcessBoundaryCurves : bTempBool, TRUE) ;

  SmTArray<SmPoint3d> sSrfPolePoints;                // PolePoint array, ordered :[UMinPole, VMinPole, UMaxPole, VMaxPole]
  SmTArray<SmPoint3d> sSrfPoleNormals;
  ULONG               lApproxSingularities;
  ULONG               lSingularities = GetSingularities(&sSrfPolePoints, &sSrfPoleNormals, &lApproxSingularities);

  // to reduce tolerances - define an offset surface (set to interpolate curve sample points during sample and walk)
  SmOffsetSurface sOffset(1.0, (SmSurface&)*this, FALSE, lSingularities) ;
  sOffset.SetContext(GetContext()) ;
  SmBoolean bTmp1 = FALSE ; 
  SmBSplineSurface * pBSplineSurface1 = SM_CAST_NONNULL_PTR(SmBSplineSurface, &sOffset) ; 
  SmTemporaryChangeValue<SmBoolean> sClean1(pBSplineSurface1 ? pBSplineSurface1->GetOutOfBoundsEnabled() : bTmp1, TRUE) ;

  // Characterize Surface seams and singularities 
  // Use the surface's own domain.
  SmSurfParamType     eSingularDirection = SM_SP_NEITHER;
  SmBoolean           bClosedU           = IsClosed(sSrfNatDomain, SM_SP_U, SM_CAST_DOUBLE_PTR(&sApproxTol3d)) ;
  SmBoolean           bClosedV           = IsClosed(sSrfNatDomain, SM_SP_V, SM_CAST_DOUBLE_PTR(&sApproxTol3d)) ;
  SmSurfParamType     eSrfClosure        =   bClosedU && bClosedV ? SM_SP_BOTH
                                           : bClosedU             ? SM_SP_U
                                           : bClosedV             ? SM_SP_V
                                           :                        SM_SP_NEITHER ;

#ifdef SM_DEBUG_CODE
static ULONG lDebugCount = 0 ;
  if(bDebugMe)
    {
      if((lApproxSingularities - lSingularities) != 0) // same as (lApproxSingularities & ~lSingularities) != 0)
                                                       // because ApproxSingularities is set for every Singularities but not vice versa
        {
          SM_ASSERT_VALID_AND_DUMP(this) ;
          lSingularities = GetSingularities(&sSrfPolePoints, &sSrfPoleNormals, &lApproxSingularities) ;
        }
    }

  //SmEdge *pEdge = (SmEdge *)p3dCurve->GetEdge() ;
  //SmFace *pFace = (SmFace *)GetFace() ;
  //SmBrep *pBrep =   pFace ? pFace->GetBrep() : pEdge ? pEdge->GetBrep() : NULL ;
  
  // Assert and Dump p3dCurve and Surface, Draw Brep, Surface, Face, Curve 
  if(bDebugMe || lDebugCount == lCount)                                
    {
      smgfx_Erase() ;
      // draw Brep, Surface (UV, seams, poles, params), Face, Curve (speed, params)
      my_AssertAndDrawDropCurve(p3dCurve, sInterval, this) ;
      sm_GraphicsLoop() ;
    }
    
#else
  SM_REF2(sSrfPoleNormals, lApproxSingularities);
#endif // SM_DEBUG_CODE

  // set sStart's Curve Start Point, 1stDeriv, and 2nd Deriv - don't expect failure - could set failure data but no point
  SER(p3dCurve->Evaluate(sStart.m_dT, 2, TRUE, sStart.m_CrvPD, TRUE)) ;

  // note: 1. When CurveStart can drop to the surface
  //          GlobalPointSolve usually returns 1 solution but
  //                        will return 2 solutions for seam points (cylinders),
  //                        will return 4 solutions for seams corners on tori
  //                        and  2 solutions for other torus seam points,
  //                        may  return 1 or more solutions for a pole which is also a seam
  //                        may  return nearly duplicate extra solutions due to tolerances and
  //                             the granularity of the curve caching mechanism: the solution can be
  //                             found in more than one tessellation piece (this problem has mostly been fixed)
  //       2. what we want - one CurveStart dropPoint solution for every curve to be traced
  //               and we want to correct any mistake GlobalDropPoint might make by duplicating a solution
  //               or missing a seam solution.
  //          So, We have to clean up the sSolutions array before using it.
  //   

  // Drop curve StartPoint to surface. 
  //   SM_SO_MINIMIZE = drop along normal vector when possible, else jump to nearby boundaries when possible, else no solutions).
  SmStatus sStatus = GlobalPointSolve                             // eff: search surface for points whose normal vectors aim at the target point
                       (sUVDomain,                                // in : Domain of surface to search for solutions
                        SM_SO_MINIMIZE,                           // in : oneof: SM_SO_MINIMIZE, SM_SO_MAXIMIZE, SM_SO_NORMALIZE, SM_SO_INTERSECT
                        sStart.m_CrvPD[0],                        // in : Target point for the solve operation
                        smos_Max(sApproxTol3d/100.0,SM_EFF_ZERO), // in : Obj ZoneTol3d assoc with Target Point, if none, use: SmTol::GetZoneTol3d(BREP_CONTEXT_OR_NULL)       
                        NULL,                                     // in : Max/Min Drop distance for min/max and normalize operations.
                                                                  //      NULL to ignore.
                        SM_SR_ALL,                                // in : SM_SR_SINGLE=get best solution, SM_SR_ALL=get all solutions
                        sSolutions) ;                             // out: array of problem solutions reported as surface UV parameter values
  // don't expect failure - could set failure data but no point
  SER(sStatus) ;

  // there should always be at least one solution
  SM_ASSERT(sSolutions.GetSize() > 0) ;
  if( sSolutions.GetSize() == 0 )
    { return SM_ERR; }

  // Classify the 1st start DropSolution point, in: m_UVPD[0]=SrfDropUV, m_CrvPD[0,1,2]=position, 1stDer, 2ndDer pCurve(dT),
  sStart.m_UVPD[0].Set(sSolutions[0].m_vStart[0], sSolutions[0].m_vStart[1]) ;
  sStart.SetProperties(*this,            // in : target drop surface                                                            
                        sOffset,         // i/o: Offset from crSurface, it's offset dist gets modified to minimize tolerances   
                        lSingularities,  // in : orof: SM_SS_NONE, SM_SS_UMIN, SM_SS_VMIN, SM_SS_UMAX, SM_SS_VMAX               
                        sApproxTol3d,    // in : min dist between distinct 3d points                                            
                        sUVDomain,       // in : Domain of interest for crSurface                                               
                        TRUE) ;          // in : TRUE = adjust m_UVPD[0] with FindDegenParamForDirection when m_UVPD[0] is on pole 
                        
#ifdef SM_DEBUG_CODE
  if (bDebugMe) 
    { 
      // ususally expect 1 sol, but possibly multiple solutions at seams and poles and due to tolerances
      smos_WriteBuffer(_T("\nUVDomain of Interest   : ")) ; crUVDomain.Dump() ;
      smos_WriteBuffer(_T("Surface NaturalUVDomain: ")) ; sSrfNatDomain.Dump() ;
      sSolutions.Dump() ;
      sStart.Dump(sApproxTol3d, _T("Start Drop Point")) ;

      smgfx_Erase() ;
      // draw Brep, Surface (UV, seams, poles, params), Face, Curve (speed, params)
      my_AssertAndDrawDropCurve(p3dCurve, sInterval, this) ;
      smgfx_SetLook(1,11, 1,0,0) ; sStart.Draw() ; sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

  // when 1st start DropPoint is not over the surface - return
  if(sStart.m_bGoodDrop == FALSE)
    {
      SM_SET2_DROP_CURVE_FAIL(SM_ERR_OUTSIDE_OF_DOMAIN, SM_CTC_FIRSTDROP_UNCONTAINED_FAIL) ;   // 1020
#ifdef SM_DEBUG_CODE  
      if(bDebugMe)
        {
          pDropCurveFail->Dump() ; 

          smgfx_Erase() ;
          pOptDropCurveFail->Draw() ; sm_GraphicsLoop() ;
          sm_GraphicsLoop() ;
        }
#endif // SM_DEBUG_CODE

      return(SM_SUCCESS) ;
    }

// GWC removed this section when backing out the partial curve drop feature:
//      // This next section supports partially dropped curves.  When the StartPoint
//      // does not drop over the surface it tries dropping the end point.  And if
//      // that is over the surface, then traces out a dropCurve from the end to
//      // the point where the curve runs off the surface.
//        // when 1st start DropPoint is not over the surface - see if end DropPoint is
//        if(sStart.m_bGoodDrop == FALSE)
//          {
//            // set sEnd's Curve Start Point, 1stDeriv, and 2nd Deriv
//            SER(p3dCurve->Evaluate(sEnd.m_dT, 2, TRUE, sEnd.m_CrvPD, TRUE)) ;
//            
//            // Drop curve EndPoint to surface. 
//            //   SM_SO_MINIMIZE = drop along normal vector when possible, else jump to nearby boundaries when possible, else no solutions).
//            SmStatus sStatus = GlobalPointSolve(sUVDomain, 
//                                                SM_SO_MINIMIZE,     // snap to boundaries when CrvStart is not over the surface
//                                                sEnd.m_CrvPD[0],
//                                                smos_Max(sApproxTol3d/100.0,SM_EFF_ZERO),        
//                                                NULL, 
//                                                SM_SR_ALL, 
//                                                sSolutions) ;
//      
//            // Classify the 1st end DropSolution point, in: m_UVPD[0]=SrfDropUV, m_CrvPD[0,1,2]=position, 1stDer, 2ndDer pCurve(dT),
//            sEnd.m_UVPD[0].Set(sSolutions[0].m_vStart[0], sSolutions[0].m_vStart[1]) ;
//            sEnd.SetProperties(*this, sOffset, lSingularities, sApproxTol3d, sUVDomain, TRUE) ;
//      
//      #ifdef SM_DEBUG_CODE
//            if (bDebugMe) 
//              { 
//                // ususally expect 1 sol, but possibly multiple solutions at seams and poles and due to tolerances
//                sSolutions.Dump() ;
//                sStart.Dump(sApproxTol3d, _T("Start Drop Point")) ;
//                sEnd.Dump(sApproxTol3d, _T("End Drop Point")) ;
//      
//                smgfx_Erase() ;
//                my_AssertAndDrawDropCurve(p3dCurve, sInterval, this) ;
//                smgfx_SetLook(1,11, 1,0,0) ; sStart.Draw() ; sm_GraphicsLoop() ;
//                smgfx_SetLook(3,11, 1,.5,0) ; sEnd.Draw() ; sm_GraphicsLoop() ;
//                sm_GraphicsLoop() ;
//              }
//      #endif // SM_DEBUG_CODE
//      
//            // quit when end point is off the surface
//            if(sEnd.m_bGoodDrop == FALSE)
//              {
//                return(SM_SUCCESS) ; 
//              }
//      
//            SmExtent1d sOldIvl = p3dCurve->GetNaturalInterval() ;
//            SmExtent1d sNewIvl ;
//      
//            // copy and reverse the curve's parameterization
//            bIsReversed = TRUE ;
//            SmCurve *pCopy = NULL ;
//            p3dCurve->Copy(crContext, pCopy) ;
//            p3dCurve = SM_CAST_PTR(SmBSplineCurve, pCopy) ;
//            if(p3dCurve == NULL)
//              { return(SM_SUCCESS) ; }
//      
//            // map the interval of interest to the new interval
//            // (y-oldMin)/OldLength = (newMax-x)/NewLength
//            p3dCurve->ReverseParameterization(sOldIvl, sNewIvl) ;
//            double dNewMin = sNewIvl.GetMax() - sNewIvl.GetLength() * (sInterval.GetMax() - sOldIvl.GetMin()) / sOldIvl.GetLength() ;
//            double dNewMax = sNewIvl.GetMax() - sNewIvl.GetLength() * (sInterval.GetMin() - sOldIvl.GetMin()) / sOldIvl.GetLength() ;
//            sInterval.SetMinMax(dNewMin, dNewMax) ;
//      
//            // update the sStart and sEnd Drop points
//            sStart.m_dT       = dNewMin ;
//            sStart.m_UVPD[0]  =  sEnd.m_UVPD[0] ;
//            sStart.m_CrvPD[0] =  sEnd.m_CrvPD[0] ;
//            sStart.m_CrvPD[1] = -sEnd.m_CrvPD[1] ;
//            sStart.m_CrvPD[2] =  sEnd.m_CrvPD[2] ;
//            sEnd.m_dT         = dNewMax ;
//      
//            // refresh the start point properties, in: m_UVPD[0]=SrfDropUV, m_CrvPD[0,1,2]=position, 1stDer, 2ndDer pCurve(dT),
//            sStart.SetProperties(*this, sOffset, lSingularities, sApproxTol3d, sUVDomain, TRUE) ;
//      
//            // next run the algorithm in reverse - remember to reverse all the output before exiting
//      
//          } // end sStart point is off the surface check
// GWC: END REMOVED SECTION that supported partially dropped curves

  // GlobalPointSolve used to make multiple nearby solutions due to the caching mechanism's granularity.
  // Although that problem is fixed - take time here to remove dropPoints with duplicate UV Values (to tol).
  sm_RemoveDuplicateDropPointSolutions(sSolutions, dUVTolX, dUVTolY) ;

  // refresh sStart to 1st solution UV loc
  sStart.m_UVPD[0].Set(sSolutions[0].m_vStart[0], sSolutions[0].m_vStart[1]) ;

  // Characterize DropPoint loc: bOnPole
  SmBoolean  bOnPole =    (lSingularities != SM_SS_NONE) 
                       && (IsSingularity( sStart.m_UVPD[0],   // in : Surface UVPoint to test
                                          lSingularities,     // in : SM_SS_NONE or one of: SM_SS_UMIN, SM_SS_VMIN, SM_SS_UMAX, SM_SS_VMAX, SM_SS_UNKNOWN
                                          &sSrfPolePoints,    //      PolePoint[4] array, ordered :[UMinPole, VMinPole, UMaxPole, VMaxPole]
                                                                          //      NonSingular side values set to
                                                                          //      SmPoint3d::SetUninitialized(),
                                          eSingularDirection, // out: SM_SP_U = crUVToTest is within tol of a degenerate V isoParameter Curve.
                                                              //                where Surf  (crUVToTest.x, VMinOrMax) = const,
                                                              //                      SurfDU(crUVToTest.x, VMinOrMax) = 0.0
                                                              //      SM_SP_V = crUVToTest is within tol of a degenerate U isoParameter Curve.
                                                              //                where Surf  (UMinOrMax, crUVToTest.y) = const,
                                                              //                      SurfDV(UMinOrMax, crUVToTest.y) = 0.0
                                                              //      SM_SP_BOTH = surface is singular in both directions at crUVToTest.
                                                              //      SM_SP_NEITHER = surface is not singular at crUVToTest.
                                          sApproxTol3d)) ;

  // For DropToPolePts, finish setting sSolutions[ii].m_vStart UV values
  //  - find degenParam value to match surface nonDegenTangent dir with curveTangent dir
  if(bOnPole)
    {
      if(bDegenCurve)
        {
          // locals

          SmBSplineCurve *pUVTrimCurve = NULL ;
          SmPoint3d sUVStart, sUVEnd ;
          SmPoint2d sUVMin = sSrfNatDomain.GetMin() ;
          SmPoint2d sUVMid = sSrfNatDomain.GetMid() ;
          SmPoint2d sUVMax = sSrfNatDomain.GetMax() ;

          // Orient the UVTrimCurve Start/End Points to place UVSurface 'inside' on the left hand side
          if(eSingularDirection == SM_SP_U || eSingularDirection == SM_SP_BOTH)
            {
              if(sStart.m_UVPD[0].y < sUVMid.y) { sUVStart.Set(sUVMin.x, sUVMin.y, 0.0) ;
                                                  sUVEnd.  Set(sUVMax.x, sUVMin.y, 0.0) ; 
                                                }
              else                              { sUVStart.Set(sUVMax.x, sUVMax.y, 0.0) ;
                                                  sUVEnd.  Set(sUVMin.x, sUVMax.y, 0.0) ; 
                                                }
            }
          else 
            {
              if(sStart.m_UVPD[0].x > sUVMid.x) { sUVStart.Set(sUVMax.x, sUVMin.y, 0.0) ;
                                                  sUVEnd.  Set(sUVMax.x, sUVMax.y, 0.0) ; 
                                                }
              else                              { sUVStart.Set(sUVMin.x, sUVMax.y, 0.0) ;
                                                  sUVEnd.  Set(sUVMin.x, sUVMin.y, 0.0) ; 
                                                }
            }

          // create a UVCurve that spans the singular Edge and return
          SmBSplineCurve::CreateLineSegment(crContext, 2, sUVStart, sUVEnd, pUVTrimCurve) ;
          pUVTrimCurve->EditParameterization(crInterval); 

          // set output
          rUVCurves.Add(pUVTrimCurve) ;
          rdMaxDropToSurf = sStart.m_dDropToSurf ;
          rdMaxApproxDev  = sStart.m_dApproxDev ; 

          // arrive here after successfully building dropCurves - make them permanent
          sCleanUVCrvs.Clear() ;

          // all done
          return(SM_SUCCESS) ;

        } // end Drop DegenCurve to Pole branch
      else
        {
          // make sure eSingularDirection is set
          SM_ASSERT(eSingularDirection != SM_SP_NEITHER) ;

          // for every CurveStart DropPoint
          for(ii=0;ii<sSolutions.GetSize() ;ii++)
            {
              sStart.m_UVPD[0].Set(sSolutions[ii].m_vStart[0], 
                                   sSolutions[ii].m_vStart[1]) ;

              // find degenParam value at a singularity that matches the dropCurve UV direction
              SmStatus eStat = FindDegenParamForDirection
                                (sStart.m_UVPD[0],      // i/o: a point on the surface singularity,                                        
                                                        //      when 2 sols are possible (seam), the sol closest to initial rUV pt is used 
                                 sStart.m_CrvPD[1],     // in : the 3d direction to match with the NonSingular Surface tangent             
                                 TRUE,                  // in : TRUE  = Dir to match is from the pole heading out, as in starting a curve
                                                        //      FALSE = Dir to match ends at the pole heading in, as in ending a curve                
                                 eSingularDirection,    // in : oneof SM_SP_U: Surf(si,v) == Surf(sj,v) where si, sj are any valid u values
                                                        //            SM_SP_V: Surf(u,si) == Surf(u,sj) where si, sj are any valid v values
                                 0.0001) ;              // in : a loose tolerance; will try for a tight one.                               
                                 // sUVDomain ) ;  // in : SubDomain of Surf to be tested               

              // note: one snafu that gets fixed in the next section.
              //   FindDegenParamForDirection is a NR search and as such has problems with multiple solutions.
              //     When the found degenParam is on a seam one wants two answers, one on each side of the seam.
              //     With NR it's always possible (though not probable), no matter the start point for the search
              //     that the algorithm will end up finding the solution far from 
              //     the start point skipping the sol near the start point. In this section, we ignore the
              //     seam problem and just want any solution that is good.  The next section will ensure
              //     that points that drop to seams get a solution for both sides of the seam.

              // when FindDegenParamForDirection works - update the CurveStart dropPt degenParam value
              if ( eStat == SM_SUCCESS )
                {
                  if ( eSingularDirection == SM_SP_U ) { sSolutions[ii].m_vStart[0] = sStart.m_UVPD[0].x; }
                  if ( eSingularDirection == SM_SP_V ) { sSolutions[ii].m_vStart[1] = sStart.m_UVPD[0].y; }
                }
              else // 
                { SM_SET4_DROP_CURVE_FAIL(SM_ERR_BAD_FIND_DEGEN_PARAM, SM_CTC_BAD_FIND_DEGENPARAM_WARNING) ;
                  SM_ASSERT(FALSE) ; 
                }
          } // end Drop NonDegen Curve to Pole branch
        } // end iter every DropPoint solution

      // GlobalPointSolve used to make multiple nearby solutions due to the caching mechanism's granularity.
      // Although that problem is fixed - take time here to remove dropPoints with duplicate UV Values (to tol).
      sm_RemoveDuplicateDropPointSolutions(sSolutions, dUVTolX, dUVTolY) ;

      // refresh sStart to 1st solution UV loc
      sStart.m_UVPD[0].Set(sSolutions[0].m_vStart[0], sSolutions[0].m_vStart[1]) ;

    } // end Dropped point to pole check

#ifdef SM_DEBUG_CODE
  if(bDebugMe) 
    { 
      // arrive here after setting pole degenParam values (OK at this time to have duplicates and miss some seam solutions) 
      smos_WriteBuffer(_T("\nUVDomain of Interest   : ")) ; crUVDomain.Dump() ;
      smos_WriteBuffer(_T("Surface NaturalUVDomain: ")) ; sSrfNatDomain.Dump() ;
      sSolutions.Dump() ;
      // in: m_UVPD[0]=SrfDropUV, m_CrvPD[0,1,2]=position, 1stDer, 2ndDer pCurve(dT)
      sStart.SetProperties(*this, sOffset, lSingularities, sApproxTol3d, sUVDomain, TRUE) ;
      sStart.Dump(sApproxTol3d, _T("Start Drop Point")) ;

      smgfx_Erase() ;
      // draw Brep, Surface (UV, seams, poles, params), Face, Curve (speed, params)
      my_AssertAndDrawDropCurve(p3dCurve, sInterval, this) ;  
      smgfx_SetLook(1,11, 1,0,0) ; sStart.Draw() ; sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

  // calc eOnSeam: (Finish DropPoint characterization) TRUE = point on seam (don't consider curve direction yet)
  // Use surface's own domain for seam checking.
  SmExtentPointType eUType, eVType ;
  sSrfNatDomain.ClassifyPoint2d(sStart.m_UVPD[0], eUType, eVType, dUVTolX, &dUVTolY) ;
  SM_ASSERT_MSG(eUType == SM_EP_START || eUType == SM_EP_INSIDE || eUType == SM_EP_END,_T("SmSurface::DropCurve Warning - StartUVPt not simply in or on boundary domain - may be an uncontained point or may misclassification small case as seam - needs review")) ;
  SM_ASSERT_MSG(eVType == SM_EP_START || eVType == SM_EP_INSIDE || eVType == SM_EP_END,_T("SmSurface::DropCurve Warning - StartUVPt not simply in or on boundary domain - may be an uncontained point or may misclassification small case as seam - needs review")) ;
  SmSurfParamType eOnSeam =   (   (eUType == SM_EP_START || eUType == SM_EP_END) && bClosedU
                               && (eVType == SM_EP_START || eVType == SM_EP_END) && bClosedV) ? SM_SP_BOTH
                            : (   (eUType == SM_EP_START || eUType == SM_EP_END) && bClosedU) ? SM_SP_U
                            : (   (eVType == SM_EP_START || eVType == SM_EP_END) && bClosedV) ? SM_SP_V
                            : SM_SP_NEITHER ;

  // for DropToSeamPts - make a DropPoint on both sides of the seam (don't worry about duplicates) when curve is moving along seam
  //                   - remove a DropPoint when curve starts on a seam and moves into the interior of the surface.
  //                   - after all DropPoint solutions are processed - remove duplicate UVPoint solutions.
  if (eOnSeam != SM_SP_NEITHER)
    {
      // for every original CurveStart DropPoint
      ULONG nOrigSols = sSolutions.GetSize() ;
      for(ii=0;ii<nOrigSols;ii++)
        {
          // make sure sStart(m_UVPD[0], m_CrvPD[1]) are set for IsMovingAlongBoundary call
          sStart.m_UVPD[0].Set(sSolutions[ii].m_vStart[0], 
                               sSolutions[ii].m_vStart[1] ) ;

          // classify this UVpoint
          sUVDomain.ClassifyPoint2d(sStart.m_UVPD[0], eUType, eVType, dUVTolX, &dUVTolY) ;
          SmPointObjectContainmentType ePOCTypeU =   (eOnSeam == SM_SP_U || eOnSeam == SM_SP_BOTH) 
                                                   ? sStart.IsMovingAlongBoundary(*this,      // in : tgt surface
                                                                                  sSrfNatDomain,  // in : surface domain of interest
                                                                                  dUVTolX,    // in : UV Tolerance in the U direction
                                                                                  dUVTolY,    // in : UV Tolerance in the V direction
                                                                                  5.0,        // in : max angle between CrvTang and SrfBndryTanget to be moving along boundary   
                                                                                              //      default:[5]
                                                                                  SM_SP_U)    // in : SM_SP_BOTH = test for pts on any bdry  
                                                                                              //      SM_SP_U    = test for pts on the u min or max bdry (moving along +/- v dir)
                                                                                              //      SM_SP_V    = test for pts on the v min or max bdry (moving along +/- u dir)
                                                                                              //      All others treated the same as SM_SP_BOTH
                                                                                              //      default:[IS_SP_BOTH]
                                                                                              //
                                                                                              // in : m_UVPD[0]  = DropPt SrfUV 2dPoint
                                                                                              //      m_CrvPD[2] = Curve Point and 1stDeriv
                                                   : SM_POC_UNKNOWN ;
          SmPointObjectContainmentType ePOCTypeV =   (eOnSeam == SM_SP_V || eOnSeam == SM_SP_BOTH) 
                                                   ? sStart.IsMovingAlongBoundary(*this,      // in : tgt surface
                                                                                  sSrfNatDomain,  // in : surface domain of interest
                                                                                  dUVTolX,    // in : UV Tolerance in the U direction
                                                                                  dUVTolY,    // in : UV Tolerance in the V direction
                                                                                  5.0,        // in : max angle between CrvTang and SrfBndryTanget to be moving along boundary   
                                                                                              //      default:[5]
                                                                                  SM_SP_V)    // in : SM_SP_BOTH = test for pts on any bdry  
                                                                                              //      SM_SP_U    = test for pts on the u min or max bdry (moving along +/- v dir)
                                                                                              //      SM_SP_V    = test for pts on the v min or max bdry (moving along +/- u dir)
                                                                                              //      All others treated the same as SM_SP_BOTH
                                                                                              //      default:[IS_SP_BOTH]
                                                                                              //
                                                                                              // in : m_UVPD[0]  = DropPt SrfUV 2dPoint
                                                                                              //      m_CrvPD[2] = Curve Point and 1stDeriv
                                                   : SM_POC_UNKNOWN ;

          // when startPt is on wrong side of seam - jump it to other side
          if(ePOCTypeU == SM_POC_OUTSIDE)     { sSolutions[ii].m_vStart.m_adParameters[0] =   eUType == SM_EP_START 
                                                                                            ? sSrfNatDomain.GetMax().x
                                                                                            : sSrfNatDomain.GetMin().x ;
                                              }
          if(ePOCTypeV == SM_POC_OUTSIDE)     { sSolutions[ii].m_vStart.m_adParameters[1] =   eVType == SM_EP_START 
                                                                                            ? sSrfNatDomain.GetMax().y
                                                                                            : sSrfNatDomain.GetMin().y ;
                                              }
          // when classification is unknown - crv or SrfBdry 1stDeriv is zero or uvPoint not on bdry
          if(   ePOCTypeU == SM_POC_UNKNOWN
             && (   eOnSeam == SM_SP_U 
                 || eOnSeam == SM_SP_BOTH))    { // arrive here when crv or SrfBdry 1stDeriv is zero or uvPoint not on bdry.
                                                 // GWC: I'm not sure what should be done here - so for now just duplicate the pt
                                                 //      and plan to review when we get a case that uses this branch 
                                                 //      - not currently hit in prog_test
                                                 SM_DBG_WARN(_T("DropCurve failed to classify a starting U drop point - review this case")) ;
                                                 ePOCTypeU = SM_POC_ON_BOUNDARY ;
                                               }
           if(   ePOCTypeV == SM_POC_UNKNOWN
             && (   eOnSeam == SM_SP_V
                 || eOnSeam == SM_SP_BOTH))    { // arrive here when crv or SrfBdry 1stDeriv is zero or uvPoint not on bdry.
                                                 // GWC: I'm not sure what should be done here - so for now just duplicate the pt
                                                 //      and plan to review when we get a case that uses this branch
                                                 //      - not currently hit in prog_test
                                                 SM_DBG_WARN(_T("DropCurve failed to classify a starting V drop point - review this case")) ;
                                                 ePOCTypeV = SM_POC_ON_BOUNDARY ;
                                               
                                               }
          // when startPt is moving along seam - add a 2nd solution to catch the other side of the seam
          if(ePOCTypeU == SM_POC_ON_BOUNDARY)  { sSolutions.Add( sSolutions[ii] ) ; // copies [ii] into [lastIndex]
                                                 jj = sSolutions.GetSize() - 1 ;   // let jj = lastIndex
                                               
                                                 // set the paired solution u param values to sit on the other side of the seam
                                                 sSolutions[ii].m_vStart.m_adParameters[0] = sSrfNatDomain.GetMin().x;
                                                 sSolutions[jj].m_vStart.m_adParameters[0] = sSrfNatDomain.GetMax().x;
                                               }               
          if(ePOCTypeV == SM_POC_ON_BOUNDARY)  { sSolutions.Add( sSolutions[ii] ) ; // copies [ii] into [lastIndex]
                                                 jj = sSolutions.GetSize() - 1 ;   // let jj = lastIndex
                                               
                                                 // set the paired solution v param values to sit on the other side of the seam
                                                 sSolutions[ii].m_vStart.m_adParameters[1] = sSrfNatDomain.GetMin().y;
                                                 sSolutions[jj].m_vStart.m_adParameters[1] = sSrfNatDomain.GetMax().y;
                                               }               
        } // end iter every orig DropPoint solution copying values across seams

      // Cull DropPoints with duplicate UV values (to tol)
      sm_RemoveDuplicateDropPointSolutions(sSolutions, dUVTolX, dUVTolY) ;
      sStart.m_UVPD[0].Set( sSolutions[0].m_vStart[0], sSolutions[0].m_vStart[1] ) ;

    } // end Dropped point to seam check

#ifdef SM_DEBUG_CODE
  if(bDebugMe) 
    { 
      // arrive here after propagating DropPtToSeam solutions across seams - (OK to have multiple solutions on different sides of seams - no duplicates)
      sSolutions.Dump() ; 

      // fill out and display sStart for 1st Drop Solution, in: m_UVPD[0]=SrfDropUV, m_CrvPD[0,1,2]=position, 1stDer, 2ndDer pCurve(dT)
      sStart.SetProperties(*this, sOffset, lSingularities, sApproxTol3d, sUVDomain, TRUE) ;
      sStart.Dump(sApproxTol3d, _T("Start Drop Point")) ;

      smgfx_Erase() ;
      // draw Brep, Surface (UV, seams, poles, params), Face, Curve (speed, params)
      my_AssertAndDrawDropCurve(p3dCurve, sInterval, this) ;
      smgfx_SetLook(1,11, 1,0,0) ; sStart.Draw() ; sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

  // arrive here after finalizing the sSolutions list
  //  - should have no duplicates
  //  - should have solutions on both sides of seams for DropToSeamPts
  //  - should have valid degenParams for DropToPolePts

#ifdef SM_DEBUG_CODE

  // Signal error - only expect 1 (typical case) or 2 (curves dropping along seam case) start drop points
  if(!(    (eOnSeam == SM_SP_NEITHER  &&  sSolutions.GetSize() == 1)
       || ((   eOnSeam == SM_SP_U 
            || eOnSeam == SM_SP_V
            || eOnSeam == SM_SP_BOTH) && (sSolutions.GetSize() == 1 || sSolutions.GetSize() == 2))) )
    {
      SM_ASSERT_MSG((    (eOnSeam == SM_SP_NEITHER  &&  sSolutions.GetSize() == 1)
                     || ((   eOnSeam == SM_SP_U 
                          || eOnSeam == SM_SP_V
                          || eOnSeam == SM_SP_BOTH) && (sSolutions.GetSize() == 1 || sSolutions.GetSize() == 2))),
                    _T("SmSurface::DropCurve() found unexpected number of dropPoint Solutions for curve Start")) ;
    }

  // check that multiple DropSolutions are all on seams
  if(sSolutions.GetSize() != 1)
    {
      SM_ASSERT(sSolutions.GetSize() == 2 || sSolutions.GetSize() == 4) ;
      if(bDebugMe) 
        { 
          sSolutions.Dump() ;
          
          // fill out and display sStart for 1st Drop Solution, in: m_UVPD[0]=SrfDropUV, m_CrvPD[0,1,2]=position, 1stDer, 2ndDer pCurve(dT)
          sStart.SetProperties(*this, sOffset, lSingularities, sApproxTol3d, sUVDomain, TRUE) ;
          sStart.Dump(sApproxTol3d, _T("Start Drop Point")) ;
          
          // check each soln is on seam
          SmPoint2d sUV1(sSolutions[0].m_vStart[0], sSolutions[0].m_vStart[1]) ;
          SmPoint2d sUV2(sSolutions[1].m_vStart[0], sSolutions[1].m_vStart[1]) ;

          SM_OLDTOL_LINE double dUVTol = SM_EFF_ZERO * 1000.0 * (1 + smos_Max(sUV1.GetMaxDimension(), sUV2.GetMaxDimension())) ;

          SmTol3d dTempTol = (SmTol3d)dUVTol;
          SM_ASSERT_MSG( IsOnSeam(sUV1,&dTempTol), _T("SmSurface::DropCurve() expects point to be on seam that isn't")) ;
          SM_ASSERT_MSG( IsOnSeam(sUV2,&dTempTol), _T("SmSurface::DropCurve() expects point to be on seam that isn't")) ;

          if(sSolutions.GetSize() == 4)
            {
              // check each soln is on seam
              SmPoint2d sUV3(sSolutions[2].m_vStart[0], sSolutions[2].m_vStart[1]) ;
              SmPoint2d sUV4(sSolutions[3].m_vStart[0], sSolutions[3].m_vStart[1]) ;

              dTempTol = (SmTol3d)dUVTol;
              SM_ASSERT_MSG( IsOnSeam(sUV3,&dTempTol), _T("SmSurface::DropCurve() expects point to be on seam that isn't")) ;
              SM_ASSERT_MSG( IsOnSeam(sUV4,&dTempTol), _T("SmSurface::DropCurve() expects point to be on seam that isn't")) ;
            }

        } // end if bDebugMe check solutions on seams
    } // end there are 2 or 4 startCurve drop solutions check
#endif // SM_DEBUG_CODE

  // arrive here after dropping CurveStart to surface and cleaning up the results stored in sSolutions array.
  //   Every dropped StartPoint represents a Curve to be traced.
  //   Usually 1 point but 2 when starting along a seam or 4 when starting along two seams on a torus.
  // next: loop over every CurveStart first trying an isoCurve drop else trying a sample and walk algorithm

  // gwc: this might be a good place to decide if we are or not dropping onto a natural curve boundary
  //      in which case we can tell the upcoming loop whether to snap or not to snap to near-by nat surf boundaries.
  // SmBoolean bSnapToNatBndries = TRUE ;
  if(sStart.m_bOnBoundary)
    {
      // perhaps - we beef up IsMovingAlongBoundary to check for divergence and curvature away from the
      //   the boundary over a few sample points near the start and end points.
      //   Only if both the start and the end point (and perhaps a mid point) all test out
      //   as being on and moving along the same boundary do we mark at this time that the
      //   curve is moving along the boundary.  Otherwise we aren't fooled by a start point that
      //   looks like its going to trace out a nat surf boundary and then end up with a bad drop curve
      //   result as the sample and walk algorithm below is forced to transition from walking along
      //   to walking in or out of the boundary.

      // gwc: need to implement this - the following is just vaporware for now
      //  // something sophisticated here to decide if we are moving along or crossing a boundary
      //  sStart.IsMovingAlongBoundary(*this,      // in : tgt surface
      //                               sUVDomain,  // in : surface natural domain
      //                               dUVTolX,    // in : UV Tolerance in the U direction
      //                               dUVTolY,    // in : UV Tolerance in the V direction
      //                               5.0,        // in : max angle between CrvTang and SrfBndryTanget to be moving along boundary   
      //                                           //      default:[5]
      //                               SM_SP_U) ;  // in : SM_SP_BOTH = test for pts on any bdry  
      //                                           //      SM_SP_U    = test for pts on the u min or max bdry (moving along +/- v dir)
      //                                           //      SM_SP_V    = test for pts on the v min or max bdry (moving along +/- u dir)
      //                                           //      All others treated the same as SM_SP_BOTH
      //                                           //      default:[IS_SP_BOTH]
      //                                           //
      //                                           // in : m_UVPD[0]  = DropPt SrfUV 2dPoint
      //                                           //      m_CrvPD[2] = Curve Point and 1stDeriv
      //  
      //  sEnd.IsMovingAlongBoundary(???)
      //  sMid.IsMovingAlongBoundary(???)
    }

  // loop locals
  SmStatus eRetStatus     = SM_ERR ;
  ULONG    lNumDropPoints = sSolutions.GetSize() ;

  // CurveCache locals: delay computing CurveCache until after trying DropIsoCurve to save time
  SmCurveCache  * pCC        = NULL;
  const SmCurve * pCurve     = NULL ;
  ULONG           lCCSpanCnt = 0 ; 
  // GWC:BEND_REPLACE_ONE_LINE
  //      const SmBSplineCurve * pCurve     = NULL ;

  //      // NearPole locals: delay computing Curve/Pole near approach pts until after trying DropIsoCurve to save time.
  //      SmTArray<SmExtent1d> sCrvPoleIvls ;    // when crSurface has poles : intervals on crCurve3d close to a pole
  //      SmTArray<SmPoint3d>  sCrvPolePoints ;  // for each sCrvPolIvls : closest Point to the pole
  //      SmTArray<double>     sCrvPoleParams ;  // for each sCrvPolIvls : closest point's crCurve3d parameter
  //      SmTArray<ULONG>      sCrvPoleSides ;   // for each sCrvPolIvls : PoleSide: 0=UMin,1=VMin,2=UMax,3=VMax
  //      SmTArray<SmBoolean>  sCrvPoleHits ;    // for each sCrvPolIvls : closest point's classification, TRUE = point hits pole, FALSE = just near
  //      SmBoolean            bNeedPoleIvls = TRUE ;

  // for each successfully dropped startPoint - trace out a UVTrimCurve
  //   GWC note: multiple drop points are now on seams. Not all curves that start
  //             on a seam run along the seam, in which case only 1 output curve will be generated.
  //             For curves that do run along seams, we should just copy the first result and
  //             edit the controlpoint coordinates to jump the traced curve over to the
  //             other side of the seam rather than tracing the same curve two times.
  //             That's left as a future enhancement opportunity.
  for(ii=0;ii<lNumDropPoints;ii++)
    {
      // set sStart[m_dT, m_UVPD[0]]
      sStart.m_dT         = sInterval.GetMin() ;
      sStart.m_UVPD[0].x = sSolutions[ii].m_vStart[0] ;
      sStart.m_UVPD[0].y = sSolutions[ii].m_vStart[1] ;

      // skip start points not within the domain of interest.
      //cbiTol: SmTol2d sUVTol = SmTol::MapTo2d( sApproxTol3d, sStart.m_UVPD[0], *this );
      double sUVTol = SM_EFF_ZERO_PARAM * (1 + smos_Min( sUVDomain.GetUInterval().GetLength(), sUVDomain.GetVInterval().GetLength() ));
      if ( !sUVDomain.ContainsPoint2d( sStart.m_UVPD[0], sUVTol ) )
        { continue ; }

      // try dropping Curve to a Surface isoParameter curve - for cheap - don't expect failures - no point in gathering failure data
      SmBoolean       bSuccess;
      SmBSplineCurve *pIsoCurve = NULL;
      SER( DropIsoCurve(crContext,                // in : context for new object construction                     
                        sUVDomain,                // in : This Surface domain of interest                        
                        *p3dCurve,                // in : Curve to drop                                          
                        sInterval,                // in : Curve interval of interest                             
                        sStart.m_UVPD[0],         // in : Surface UVPoint corresponding to start of3d curve.     
                        sApproxTol3d,             // in : max allowed distance between drop point and surfNormal line at drop point           
                        bSuccess,                 // out: TRUE = dropped a curve                                 
                        sStart.m_dDropToSurf,     // out: max sample distance found between *p3dCurve and Surface
                                                  //      taken at p3dCurve->NumberOfKnot evenly spaced samples  
                        pIsoCurve,                // out: Pointer to newly allocated UVLine curve or             
                                                  //      NULL if no curve were constructed                      
                       &sStart.m_dApproxDev)) ;   // out: Max CurveToDrop smplPoint to DropSurfNormLine dist
             //          &dMaxDropToSurfParam)) ;   // out: opt param value for rdMaxDropToSurf value

      // low work: curve dropped to surface isoParameter curve - done with current start point
      if ( bSuccess )
        {
#ifdef SM_DEBUG_CODE
          if(bDebugMe)
            {
              SM_ASSERT_VALID(pIsoCurve) ;
              SmCrvOnSurf sCrvOnSurf( * pIsoCurve, (SmSurface &)*this) ;
              ULONG lSmpCnt = 55 ; 
              SmCrvCrvGapFunction sFaceTrimCurveCrvGF((SmXSectTol3d)sApproxTol3d, &sCrvOnSurf, (SmExtent1d &)crInterval, p3dCurve, lSmpCnt) ; 
              SmCrvCrvGapFunction sCrvFaceTrimCurveGF((SmXSectTol3d)sApproxTol3d, p3dCurve, (SmExtent1d &)crInterval, &sCrvOnSurf, lSmpCnt) ; 
              SmCrvSrfGapFunction sCrvSrfGF((SmXSectTol3d)sApproxTol3d, &sCrvOnSurf, (SmExtent1d &)crInterval, this, lSmpCnt) ;

              SmGapSample *pMaxFaceTrimCurveCrvGap; sFaceTrimCurveCrvGF.GetMaxGapSample(pMaxFaceTrimCurveCrvGap);
              SmGapSample *pMaxCrvFaceTrimCurveGap; sCrvFaceTrimCurveGF.GetMaxGapSample(pMaxCrvFaceTrimCurveGap);
              SmGapSample *pMaxCrvSrfGap;           sCrvSrfGF.GetMaxGapSample(pMaxCrvSrfGap);
      
              //double dMaxFaceTrimCurveCrvGap = sMaxFaceTrimCurveCrvGap.GetLength() ; 
              //double dMaxCrvFaceTrimCurveGap = sMaxCrvFaceTrimCurveGap.GetLength() ; 
              //double dMaxCrvSrfGap           = sMaxCrvSrfGap.GetLength() ; 
            }
#endif // SM_DEBUG_CODE

          // add to output
          rUVCurves.Add(pIsoCurve) ;
          if(dMaxDropToSurf < sStart.m_dDropToSurf) { dMaxDropToSurf = sStart.m_dDropToSurf ;
                                                      // if(pMaxDropToSurfParam) { *pMaxDropToSurfParam = dMaxDropToSurfParam ; }
                                                    }
          if(dMaxApproxDev  < sStart.m_dApproxDev)  { dMaxApproxDev  = sStart.m_dApproxDev ;  }

          // Go to next iter(ii) dropped UVstart point to build next UVcurve.
          continue ;

        } // end Curve dropped to IsoCurve check

      // arrive here when curve did not drop to isoParameter curve.
      //   Proceed to tracing the curve projection.

      // trace curve locals
      SmPoint3d           sPData[256];
      double              sKData[256];
      SmTArray<SmPoint3d> sCtrlPoly(256,sPData) ;  // output UVcurve control polygon being built
      SmTArray<double>    sKnots(256,sKData) ;     // output UVcurve control knot vector being built

      // pole and seam crossing locals
      SmBoolean  bHaveStartUVTan  = FALSE ;
      SmBoolean  bFixBTChange     = FALSE ;  // TRUE = sm_FixBoundaryTangents moved control points
      SmBoolean  bFixBCChange     = FALSE ;  // TRUE = sm_FixBoundaryCurves moved control points

      // SmBoolean  bLeavingDomain   = FALSE ;
      // double     dCrossingT       = 0.0;

      // GWC: Special near pole handling code was not found necessary - removing for the time being
//            ULONG      lNearPoleIndx    = 0 ;
//            SmBoolean  bIsPoleIvl       = FALSE ;
//            SmBoolean  bIsNextPoleIvl   = FALSE ;
//            SmBoolean  bIsPoleIvlSplit  = FALSE ;
//            ULONG      lPoleIvlCnt      = 0 ;
//            ULONG      iiPole           = 0 ;

      // get curve cache - just once - after DropIsoCurve to prevent extra work
      if(pCC == NULL )
        {
          pCC        = (SmCurveCache*)SmCacheMgr::GetOrCreateObjectCache(SM_OC_CURVE,p3dCurve) ; NER(pCC) ;
          // GWC:BEND_REPLACE_ONE_LINE 
          // pCurve     = (const SmBSplineCurve *)pCC->GetCurve() ;
          pCurve     = (const SmCurve *)pCC->GetCurve() ;
          lCCSpanCnt = pCC->GetSpanCount() ;
          SM_ASSERT(pCurve == p3dCurve) ;
        }

      // GWC: Special near pole handling code was not found necessary - removing for the time being
      // BEGIN REMOVE BLOCK
      //      // get 'near pole' CrvIvls - just once - after DropIsoCurve to prevent extra work
      //      if(   bNeedPoleIvls  == TRUE 
      //         && lSingularities != SM_SS_NONE)
      //        {
      //          // get 'near' pole intervals from solution array
      //          sm_GetNearPoleIvls(*this,      sUVDomain,  // in : tgt surf and domain
      //                              *p3dCurve, sInterval,  // in : CurveOnSurf and interval
      //                              sApproxTol3d,          // in : min dist between distinct 3d points
      //                              lSingularities,        // in : list of surface singularities                                 
      //                                                     //      orof: SM_SS_UMIN, SM_SS_VMIN, SM_SS_UMAX, SM_SS_VMAX          
      //                              sSrfPolePoints,        // in : Pole locs ordered:[UMin, VMin, UMax, VMax]                    
      //                                                     //      pts for nonSingular sides set to SmPoint2d::SetUninitialized()
      //                              sCrvPoleIvls,          // out: intervals over which the curve is 'near' the pole             
      //                              sCrvPolePoints,        // out: associated closest point to pole for each interval            
      //                                                     //      note: assume one closest point to the pole per interval
      //                              sCrvPoleParams,        // out: associated curve parameters for every closest point in sCrvPolePoints       
      //                              sCrvPoleSides,         // out: associated surface side for each pole interval                
      //                                                     //      oneof: 0=SM_SS_UMIN, 1=SM_SS_VMIN, 2=SM_SS_UMAX, 3=SM_SS_VMAX 
      //                              sCrvPoleHits) ;        // out: associated closest point classification, TRUE = point hits pole, FALSE = just near
      //      
      //          // remember that this is done
      //          lPoleIvlCnt   = sCrvPoleIvls.GetSize() ;
      //          bNeedPoleIvls = FALSE ;
//     // GWC_NOTE REMOVE_NEXT_LINE_TO_ENABLE_POLE_HANDLING GWC_LINE GWC_LINE ;
//                iiPole = lPoleIvlCnt ;  // setting iiPole == lPoleIvlCnt says all PoleIvls have been processed - that's how poleIvls get skipped later on.
//      
//      #ifdef SM_DEBUG_CODE
//                if(bDebugMe)
//                  {
//                    smgfx_Erase() ;
//                    // draw Brep, Surface (UV, seams, poles, params), Face, Curve (speed, params)
//                    my_AssertAndDrawDropCurve(p3dCurve, sInterval, this) ;
//                    for(kk=0;kk<sCrvPoleIvls.GetSize();kk++)
//                      { SmPoint3d sSrfSpanStart ; EvaluatePoint(sStart.m_UVPD[0], sSrfSpanStart) ; 
//                        smgfx_SetLook(7,8, 0,0,1) ; p3dCurve->Draw(&sCrvPoleIvls[kk]) ; sm_GraphicsLoop() ;
//                        smgfx_SetLook(10,11, 0,0,1) ; sCrvPolePoints[kk].Draw() ; sm_GraphicsLoop() ;
//                        smgfx_SetLook(1,11, 1,0,0) ;  sSrfSpanStart.Draw() ; (sSrfSpanStart-sCrvPolePoints[kk]).Draw(&sCrvPolePoints[kk]) ; sm_GraphicsLoop() ;
//                      }
//                    sm_GraphicsLoop() ;
//                  }
//      #endif // SM_DEBUG_CODE
      //        } // end Need Pole Ivls check
      // END BLOCK REMOVAL

      // next - Build a piecewise hermite cubic UVCurve approx to the DropCurve UVTrimCurve one
      //         CurveCache decomposition span (e.g. CacheSpan) at a time. 
      //
      // next method - For every CacheSpan
      //   Approx CacheSpanDropCurve with piecewise hermite UVTrimCurve.
      //      Start with single hermite approx of span - subdivide as needed to meet tol test.
      //   Special Handling: Seams
      //      Start Points have already been placed on the proper side of the seam.
      //      When walking along a seam - make sure not to jump seams to other side of surface domain
      //        done in sm_CheckSeamJump
      //   Special Handling: Poles
      //      Pole Intersections -  Split CacheSpans at points of Pole intersections:
      //        Use FindDegenParamForDirection() to pick degen param value based on curve's tangent dir at pole point.
      //        Set degen param Tangent coordinate to zero.
      //      Pole Approaches - Use different span appox technique not based on tangent projections
      //        over Curve regions near poles (e.g. PoleIvls).

      // for every curve cache decomposition span
      for(jj=0;jj<lCCSpanCnt;jj++)
        {
          // State at top of loop:
          //   jj                 index of CurveCacheDecomposition interval being dropped
          //   sStart.m_dT        param of rest of curve still needing to be dropped
          //   pCC                Curve Cache containing curve decomposition intervals
          //   sInterval          Curve interval of interest
          //   bHaveStartUVTan    TRUE = last iteration set sStart values
          //   dMaxDropSq         largest startSpan SrfPt3d/CrvPt distance seen
          //   sCtrlPoly          ordered UV ctrlPoint array for all previously dropped span ivls (stored in SmPoint3d array - z coords ignored)
          //   sKnots             ordered param values for all previously dropped span ivls
          //
          // GWC: near pole special case was not shown to be required
          //      //   sCrvPoleIvls,      Curve intervals near poles
          //      //   sCrvPolePoints     PolePoints, ordered:[UMin, VMin, UMax, VMax]
          //      //   sCrvPoleParams     Curve Param values marking points of closest approach to a pole, one for each sCrvPoleIvls
          //      //   sCrvPoleSides      PoleSide index for each sCrvPoleIvls: 0=UMin, 1=VMin, 2=UMax, 3=VMax
          //      //   sCrvPoleHits       TRUE=Nearest point hits pole, FALSE=just near pole
          
          // - have SpanStart[m_dT, UV] and CacheSpan jj index - Get SpanStart[UV 1stDeriv] and SpanEnd[m_dT, UV, UV 1stDeriv]
          //    to define Span drop approx hermite approx. When that approx passes tol checks, add cubic hermite segments
          //    as control points and param values to the arrays sCtrlPoly and sKnots. Subdivide Span into segments
          //    to meet tols as needed.

          // get span shape and ivl for cache interval
          SmBezierSpan * pBezSpan  = pCC->GetAt(jj) ;
          SmExtent1d     sIvl      = pBezSpan->GetInterval() ;
          SmBoolean      bNULLIvl  = FALSE ;
          // SmBoolean      bNearPole = FALSE ;

          // due to nearPoleIvl handling - may have already dropped all this CacheIvl, go to next iter(ii)
          if(sIvl.GetMax() <= sStart.m_dT)
            { continue ; }
          
          // due to nearPoleIvl or seam handling - may have already dropped part of this CacheIvl
          if(sIvl.GetMin() > sStart.m_dT)
            {  bNULLIvl &= (sIvl.SetMinMax(sStart.m_dT, sIvl.GetMax()) != SM_SUCCESS) ; }

          // GWC: near pole special case not found necessary
          // BEGIN BLOCK REMOVAL
          // when CacheIvl starts at end of or after current PoleIvl - increment the Pole Ivl
          //      if(iiPole < lPoleIvlCnt && sIvl.GetMin() >= sCrvPoleIvls[iiPole].GetMax() - SM_EFF_ZERO)
          //        {
          //          iiPole++ ;
          //        }
          //      
          //      // when remaining curve runs near any poles - modify Span Ivl as needed to step cautiously up to and around poles
          //      if(iiPole < lPoleIvlCnt)
          //        {
          //          // when CacheIvl overlaps the next NearPoleIvl 
          //          if(sIvl.GetMax() > sCrvPoleIvls[iiPole].GetMin())
          //            {
          //              // decrement jj the CacheSpan count - so that CacheSpans split by PoleIvls get a chance to drop their 2nd haves
          //              if(jj > 0) { jj-- ; }
          //      
          //              // When CacheIvl starts before the PoleIvl - only drop CacheSpan segment outside of interval
          //              if(sIvl.GetMin() < sCrvPoleIvls[iiPole].GetMin())
          //                {
          //                  // exception - Let CacheSpans run to Pole XSects where tangent drops still work
          //                  if(   sCrvPoleHits[iiPole] == TRUE   // next PoleIvl XSects the pole
          //                     && sIvl.ContainsValue(sCrvPoleParams[iiPole], SM_EFF_ZERO))
          //                    {
          //                      bNULLIvl &= (sIvl.SetMinMax(sIvl.GetMin(), sCrvPoleParams[iiPole]) != SM_SUCCESS) ; 
          //                    }
          //                  else // split CacheSpan at PoleIvl start
          //                    {
          //                      bNULLIvl &= (sIvl.SetMinMax(sIvl.GetMin(), sCrvPoleIvls[iiPole].GetMin()) != SM_SUCCESS) ;  
          //                    }
          //                }
          //              
          //              // When CacheIvl starts on or in Pole Ivl
          //              else if(sCrvPoleIvls[iiPole].ContainsValue(sIvl.GetMin(), SM_EFF_ZERO))
          //                {
          //                  // drop PoleIvl 
          //                  sIvl = sCrvPoleIvls[iiPole] ;
          //      
          //                  // remember the difference between near pole and pole intersection cases
          //                  bNearPole = (sCrvPoleHits[iiPole] != TRUE) ;
          //                  
          //                  // Start the PoleIvl from the current sStart.m_dT value       
          //                  bNULLIvl &= (sIvl.SetMinMax(sStart.m_dT, sIvl.GetMax()) != SM_SUCCESS) ; 
          //                }
          //            } // end CacheSpan interacts with a NearPoleIvl check
          //        } // end watch for nearPoleIvl check
          // END BLOCK REMOVAL

          // skip spans outside the given curve interval, goto next iter(ii)
          if(bNULLIvl || sInterval.AreDisjoint( sIvl ))
            { continue; }

          // when sIvl is not already within sInterval
          if( !sIvl.IsContainedBy( sInterval, SM_EFF_ZERO ))
            {
              // trim sIvl to fit within sInterval
              SmExtent1d sTempIvl;
              SmStatus sRtn = sIvl.Intersect( sInterval, sTempIvl ) ;

              // skip disjoint spans
              if(sRtn != SM_SUCCESS)
                { // disjoint spans have already been checked - shouldn't come down this failure branch. goto next iter(ii)
                  continue; 
                }  

              // skip degenerate spans, goto next iter(ii)
              //double dMinIvlLen = sIvl.GetLength() / 1000;  //cbiTol  // [B406]
              //if ( sTempIvl.GetLength() < dMinIvlLen )
              if ( p3dCurve->ApproximateLength( sTempIvl, 5 ) < sApproxTol3d )
              { continue; }

              sIvl = sTempIvl;
            }

          SM_ASSERT_MSG(SM_IS_ZERO(sStart.m_dT-sIvl.GetMin()), _T("logic to start iter with sStart.m_dT is broken - fix here")) ;

          // get span endPoints
          sStart.m_dT = sIvl.GetMin() ;
          sEnd.m_dT   = sIvl.GetMax() ;

          // get CrvSpanStart point, 1stDeriv, and 2ndDeriv - don't expect failure - no point in collecting failure data
          SER( pCurve->Evaluate(sStart.m_dT, 2, TRUE, sStart.m_CrvPD)) ;   // nonZeroTangent values

          // GWC nearPole special case handling was not shown to be necessary
          //      // when moving near a pole
          //      if(bNearPole)
          //        {
          //          // do something for NearPole spans
          //      
          //          // set sStart.m_dT, sStart.m_UVPD[0], sStart.m_bFromLeftU, and sStart.m_bFromLeftV for next iteration
          //      
          //          // move onto next segment
          //          continue ;
          //        }

          // Get SpanStart[UV1stDeriv] by dropping SpanStart[Crv1stDeriv] to offset surface running through SpanStart[CrvPt]
          if( !bHaveStartUVTan)
            { sStart.SetProperties(*this,           // in : target drop surface                                                                                                  
                                    sOffset,        // i/o: Offset from crSurface, it's offset dist gets modified to minimize tolerances                                         
                                    lSingularities, // in : orof: SM_SS_NONE, SM_SS_UMIN, SM_SS_VMIN, SM_SS_UMAX, SM_SS_VMAX
                                    sApproxTol3d,   // in : min dist between distinct 3d points                                                                             
                                    sUVDomain,      // in : Domain of interest for crSurface
                                    TRUE) ;         // in : FALSE = don't adjust m_UVPD[0] with a call FindDegenParamForDirection
                                                    // in : m_UVPD[0]  = drop pCurve(dT) position to surface 2d point (Solution of GlobalPointSolve for dT) 
            /* GWC: I think above should be TRUE */ //      m_CrvPD[0] = position pCurve(dT)
                                                    //      m_CrvPD[1] = 1stDeriv pCurve(dT)
                                                    //      m_CrvPD[2] = 2ndDeriv pCurve(dT)
                                                    // out: m_UVPD[1],             Proj of m_CrvPD[1] to Surface
                                                    //      m_bGoodDrop,           TRUE = m_dApproxDev < sApproxTol3d
                                                    //      m_3dPD[1],             Proj of m_UVPD[1] through Surface back to 3d
                                                    //      m_SrfNormal,           SurfaceNormal at DropPoint
                                                    //      m_bFromLeftU,          TRUE=Dropped Srf 1stDeriv starts in pos U direction, FALSE=in neg U direction
                                                    //      m_bFromLeftV,          TRUE=Dropped Srf 1stDeriv starts in pos V direction, FALSE=in neg V direction
                                                    //      m_dDropToSurfDrop                                                         
                                                    //      m_dApproxDev,          deviation of m_CrvPD[0] from SrfNormalLine starting at Surface(sSrfUV[0])
                                                    //      m_dNormalToCrvAngDeg,  angleDeg between m_CrvPD[1] and SrfPlane normal, should be near 90.0
                                                    //      m_dSrfToCrvAngDeg,     angleDeg between m_CrvPD[1] and SurfaceProjection of sSrfUV[1], should be near 0.0
                                                    //      m_dSrfToCrvSpeedRatio, m_3dPD[1].Length() / m_CrvPD[1], should be near 1.0 
                                                    //      m_bOnBoundary, 
                                                    //      m_bLeavingDomain, 
                                                    //      m_bEnteringDomain
                                                    //      m_eDropDerivRtn        SM_SUCCESS = good drop,
                                                    //                             SM_ERR_BAD_TANGENT_DROP = CrvTangent drop produced a non Parallel SrfTangent direction
                                                    //                             SM_ERR_NOT_WITHIN_TOLERANCE = Drop not along surface normal
                                                    //                                     usually caused by dropping a CrvPt not over the surface to surface boundary.

              // save sMaxDropToSurface distance seen
              if( smos_Fabs( sStart.m_dDropToSurf ) > dMaxDropToSurf ) // DropDist is signed.
                { dMaxDropToSurf = smos_Fabs( sStart.m_dDropToSurf );
                  // dMaxDropToSurfParam = sStart.m_dT ;
                }
              if( sStart.m_dApproxDev  > dMaxApproxDev  ) { dMaxApproxDev  = sStart.m_dApproxDev ; }

#ifdef SM_DEBUG_CODE
              if(sStart.m_bGoodDrop == FALSE)
                { // place for a break from iter(jj) back to iter(ii)
                  sStart.m_bGoodDrop = sStart.m_bGoodDrop ;
                }
              if(bDebugMe)
                {
                  sStart.Dump(sApproxTol3d, _T("Start Drop Point")) ;
                  smgfx_Erase() ;
                  // draw Brep, Surface (UV, seams, poles, params), Face, Curve (speed, params)
                  my_AssertAndDrawDropCurve(p3dCurve, sInterval, this) ;
                  smgfx_SetLook(3,11, 0,0,1) ; sStart.Draw() ; sm_GraphicsLoop() ;
                  smgfx_SetLook(2,12, .2,.4,.7) ; DrawUVPolyline(sCtrlPoly, TRUE) ; sm_GraphicsLoop() ;
                  sm_GraphicsLoop() ;
      
                }
#endif // SM_DEBUG_CODE
            } // end need StartUVTan check

          // compute SpanEnd[UV and Properties] by dropping SpanEnd[CrvPt Crv1stDeriv] to surf
          //      correcting for seam jumps (wrong side pt drops) and accounting for drops to poles.
          // inputs: SpanStart[T UV UV1stDeriv UV2ndDeriv] and SpanEnd[T]
          eRetStatus = sStart.GetNextPoint
            (*this,                     // in : main surface                                                         
              sOffset,                  // i/o: offset of main surface, its offset distance is set in this routine
              lSingularities,           // in : orof: SM_SS_NONE, SM_SS_UMIN, SM_SS_VMIN, SM_SS_UMAX, SM_SS_VMAX
              &sSrfPolePoints,          // in : PolePoint[4] array, ordered :[UMinPole, VMinPole, UMaxPole, VMaxPole]
                                        //      NonSingular side values set to
                                        //      SmPoint3d::SetUninitialized(),
              sUVDomain,                // in : limiting domain of the surface                                       
              eSrfClosure,              // in : oneof: SM_SP_U, SM_SP_V, SM_SP_BOTH, SM_SP_NEITHER                   
             *pCurve,                   // in : Curve being dropped                                                  
              sApproxTol3d,             // in : max allowed distance between drop point and surfNormal line at drop point
              dUVTolX,                  // in : UV tol in u dir
              dUVTolY,                  // in : UV tol in v dir                                            
              sEnd.m_dT,                // in : SpanStart Curve parameter
              sEnd) ;                   // out: rEnd.m_dT
                                        //      rEnd.m_CrvPD 
                                        //      rEnd.m_UVPD, rEnd.m_bFromLeftU, rEnd.m_bFromLeftV,
                                        //      rEnd.m_3dPD
                                        //      rEnd.m_dDropToSurf,        rEnd.m_dApproxDev
                                        //      rEnd.m_dNormalToCrvAngDeg
                                        //      rEnd.m_dSrfToCrvAngDeg,    rEnd.m_dSrfToCrvSpeedRatio
                                        //      rEnd.m_bOnBoundary,        rEnd.m_bLeavingDomain,  rEnd.m_bEnteringDomain
                                        //      rEnd.m_eDropDerivRtn   
                                        //         SM_SUCCESS = good drop, 
                                        //         SM_ERR_NOT_WITHIN_TOLERANCE = CrvPt not with tol of SrfNormalLine at dropPoint
                                        //                usually caused by dropping a CrvPt not over the Srf to a SrfBoundary
                                        //         SM_ERR_OUTSIDE_OF_DOMAIN = StartPt drops to SrfBoundary and DropCrv moves out of srf 
                                        //         SM_ERR_BAD_TANGENT_DROP  = CrvTangent dropped to nonParallel SrfTangent
                                        //                usually caused by dropping a CrvTangent near a srfPole                                             
#ifdef SM_DEBUG_CODE
          if(bDebugMe)
            {
              sEnd.Dump(sApproxTol3d, _T("End Drop Point")) ;

              smgfx_Erase() ;
              // draw Brep, Surface (UV, seams, poles, params), Face, Curve (speed, params)
              my_AssertAndDrawDropCurve(p3dCurve, sInterval, this) ;
              smgfx_SetLook(3,11, 0,0,1) ; sStart.Draw() ; sm_GraphicsLoop() ;
              smgfx_SetLook(3,11, 1,.5,0) ; sEnd.Draw() ; sm_GraphicsLoop() ;
              smgfx_SetLook(2,12, .2,.4,.7) ; DrawUVPolyline(sCtrlPoly, TRUE) ; sm_GraphicsLoop() ;
              sm_GraphicsLoop() ;
            }
#endif // SM_DEBUG_CODE

          // save max DropDist and ApproxDeviation
          if(eRetStatus == SM_SUCCESS)
            {
              if( smos_Fabs( sEnd.m_dDropToSurf ) > dMaxDropToSurf ) // DropDist is signed.
                { dMaxDropToSurf = smos_Fabs( sEnd.m_dDropToSurf );
                  // dMaxDropToSurfParam = sEnd.m_dT ;
                }
              if( sEnd.m_dApproxDev  > dMaxApproxDev  ) { dMaxApproxDev  = sEnd.m_dApproxDev ; }
            }

          // GWC: REMOVED PARTIAL DROP FEATURE.  This next block of code
          //      found the CurvePoint that dropped exactly to a Surface Boundary so that
          //      a partial drop Curve could be found and returned.
          //      Keep this code around in case it's desired to split a dropCurve at a 
          //      seam crossing into two output UVCurves.
          // BEGIN REMOVE BLOCK
          //      // Handle sm_GetNextPoint problems - sEnd.m_dT does not drop to the surface - do binary search for new sEnd.m_dT
          //      if(eRetStatus == SM_ERR_NOT_WITHIN_TOLERANCE)  
          //        {
          //          // Arrive here when SpanEnd CrvPt did not drop down a srfNormalLine at the dropPoint
          //          //   expected to be caused by dropping a CrvPt that is not over the surface to a srfBoundary.
          //          // Expect SpanStart CrvPt to have dropped well and head into the surface
          //          if(   sEnd.m_bGoodDrop   != FALSE
          //             || sEnd.m_bOnBoundary != TRUE 
          //             || sStart.m_bGoodDrop != TRUE
          //             || (   sStart.m_bOnBoundary    == TRUE 
          //                 && sStart.m_bLeavingDomain == TRUE))  
          //            {
          //              SM_ASSERT(   sEnd.m_bGoodDrop   == FALSE
          //                        && sEnd.m_bOnBoundary == TRUE 
          //                        && sStart.m_bGoodDrop == TRUE
          //                        && (   sStart.m_bOnBoundary == FALSE 
          //                            || sStart.m_bLeavingDomain == FALSE)) ; 
          //            }
          //      
          //          // do a binary search until the crossing m_dT value can be found.
          //      
          //          // Remember when span crossed a boundary: later
          //          //  1. This curve will be closed ending at the boundary crossing.
          //          //  2. If span crossed a seam bdry, 
          //          //       continue tracing a 2nd curve from other side of boundary.
          //          //     Else, quit tracing from this drop point - move onto next drop point.
          //          // But, to close off this curve, we continue with this loop (the stack and all, below), 
          //          //   to test tolerance (and further subdivide if necessary),
          //          //   to finish adding points to the sCtrlPoly and sKnots arrays, 
          //          //   and finally to flush sCtrlPoly and sKnots to create the dropped curve.
          //      
          //          // Try a binary search to find the crossing m_dT value
          //          SmDropPt  sThis(0.0) ;
          //          SmStatus  sThisRetStatus = eRetStatus ;
          //      
          //          SmBoolean bDone = SM_IS_ZERO(sEnd.m_dT - sStart.m_dT) ;
          //      
          //          if(!bDone)
          //            {
          //              // binary search from start to end for surface boundary point
          //              sThisRetStatus = sStart.IterateToBoundary   // in : m_dT, m_UVPD, m_bFromLeftU, m_bFromLeftV, m_bGoodDrop                    
          //                (*this,                                   // in : target surface                                                           
          //                  sOffset,                                // in : target surface offset                                                    
          //                  lSingularities,                         // in : orof: SM_SS_NONE, SM_SS_UMIN, SM_SS_VMIN, SM_SS_UMAX, SM_SS_VMAX
          //                  sUVDomain,                              // in : Surface domain                                                           
          //                  eSrfClosure,                            // in : oneof: SM_SP_U, SM_SP_V, SM_SP_BOTH, SM_SP_NEITHER                       
          //                 *pCurve,                                 // in : Curve being dropped                                                      
          //                  sApproxTol3d,                           // in : max allowed distance between drop point and surfNormal line at drop point
          //                  dUVTolX,                                // in : UV tol in u dir                                                          
          //                  dUVTolY,                                // in : UV tol in v dir                                                          
          //                  sEnd,                                   // in : m_dT, m_UVPD, m_bFromLeftU, m_bFromLeftV, m_bGoodDrop                    
          //                  sThis) ;                                // out: Drop to Boundary DropPt                                                  
          //                
          //              // when boundary was found - copy ThisSolution into sSrfSpanEnd
          //              if(sThisRetStatus == SM_SUCCESS)
          //                {
          //                  eRetStatus         = sThisRetStatus ;
          //                  sEnd               = sThis ;
          //                  SM_ASSERT(sThis.m_bLeavingDomain  == TRUE) ;
          //                  
          //                  sIvl.SetMinMax(sStart.m_dT, sEnd.m_dT) ;
          //      
          //                  // save sMaxDropToSurface distance seen
          //                  if( dMaxDropToSurf < sThis.m_dDropToSurf )
          //                    {
          //                       dMaxDropToSurf = sThis.m_dDropToSurf ; 
          //                       // dMaxDropToSurfParam = sThis.m_dT ;
          //                    }
          //
          //                  if( dMaxApproxDev  < sThis.m_dApproxDev )
          //                    { dMaxApproxDev  = sThis.m_dApproxDev ;  }
//      #ifdef SM_DEBUG_CODE
//                            if(sEnd.m_bGoodDrop == FALSE)
//                              { // place for a break from iter(jj) back to iter(ii)
//                                sEnd.m_bGoodDrop = sEnd.m_bGoodDrop ;
//                              }
//      #endif // SM_DEBUG_CODE
          //                }
          //              else
          //                {
          //                  eRetStatus = SM_ERR_NOT_CONVERGING ; // remember we tried to fix this case but that didn't work
          //                }
          //            } // end can try looking for endPoint check 
          //        } // end NextGetPoint was off the surface check

          // do not return a curve when DropCurve wanders out of the surface domain (call DropAndTrimCurve for that case)
          if(   (sEnd.m_bOnPole || sEnd.m_bLeavingDomain)     // when leaving the domain or hitting a pole
             && !crInterval.IsValueOnBoundary(sEnd.m_dT))     // in the middle of the curve
            {
              // change the return status to force error handling
              eRetStatus = SM_ERR_OUTSIDE_OF_DOMAIN ;  // 1020
            }

          // Handle sm_GetNextPoint problems - when something went wrong - quit without a drop curve
          if(eRetStatus != SM_SUCCESS)
            {
              // GWC: prog_test arrives here often with
              //          eRetStatus == SM_ERR_OUTSIDE_OF_DOMAIN    (eRetStatus == 1020) SpanStart onBoundary and DropCrv heading out of Domain
              //          eRetStatus == SM_ERR_BAD_TANGENT_DROP     (eRetStatus == 1032) CrvTangents dropped to bad SrfTangents
              //          eRetStatus == SM_ERR                      (eRetStatus == 1001) Some nested eval call failed
              //          eRetStatus == SM_ERR_NOT_CONVERGING       (eRetStatus == 1022) IterateToBoundary failed to find endPoint for a case expected of walking off the end of the domain
              //          eRetStatus == SM_ERR_NOT_WITHIN_TOLERANCE (eRetStatsu == 1021) NextPoint is no longer over the surface -
              //                                                                          sEnd.m_bOnBoundary should be TRUE
              //                                                                          sEnd.m_bLeavingDomain probably TRUE (but FALSE when boundary following curve wanders slowly out of tol)
              //        what do we want to do about that?

#ifdef SM_DEBUG_CODE
              if(bDebugMe)
                {
                  sStart.Dump(sApproxTol3d, _T("Start Drop Point")) ;
                  sEnd.Dump(sApproxTol3d, _T("End Drop Point")) ;
                  sCtrlPoly.Dump() ;

                  smgfx_Erase() ;
                  // draw Brep, Surface (UV, seams, poles, params), Face, Curve (speed, params)
                  my_AssertAndDrawDropCurve(p3dCurve, sInterval, this) ;
                  smgfx_SetLook(3,11, 0,0,1) ; sStart.Draw() ; sm_GraphicsLoop() ;
                  smgfx_SetLook(3,11, 1,.5,0); sEnd.Draw() ; sm_GraphicsLoop() ;
                  smgfx_SetLook(2,12, .2,.4,.7) ; DrawUVPolyline(sCtrlPoly, TRUE) ; sm_GraphicsLoop() ;
                  sm_GraphicsLoop() ;
                }
#endif // SM_DEBUG_CODE

              // 1st fix case: eRetStatus == SM_ERR_OUTSIDE_OF_DOMAIN = 1020
              //   allow drops when the curve extends past the natural surface by less than tolerance.
              //   In this case map the end of the drop curve to the boundary point
              if(   eRetStatus == SM_ERR_OUTSIDE_OF_DOMAIN
                 && sStart.m_bOnBoundary    == TRUE
                 && sStart.m_bLeavingDomain == TRUE
                 && sKnots.GetSize() > 1)
                {
                  // check to see how much curve is left to drop
                  SmExtent1d sEndIvl(sStart.m_dT, sInterval.GetMax()) ;
                  double dApproxLength = p3dCurve->ApproximateLength(sEndIvl, 5) ;

                  // when remaining edge fragment to drop is short
                  if(dApproxLength < sApproxTol3d)
                    {
                      // and if the distance between the edge end to the boundary point is small
                      SmPoint3d sCurveEnd ;
                      p3dCurve->EvaluatePoint(sInterval.GetMax(), sCurveEnd) ;
                      double dEndGapLength = sStart.m_3dPD[0].DistanceBetween(sCurveEnd) ;
                      if(dEndGapLength < sApproxTol3d)
                        {
                          // change the last saved knot value to the end of the curve
                          sKnots.SetAt(sKnots.GetSize()-1, sInterval.GetMax()) ;
                        }

                      // quit the iter(jj) every CacheSpan loop (with curve to flush) and move onto next StartPoint
                      break;
                    } // end found a curve close enough to the end to save check
                } // end walked off the edge of the surface check


              // arrive here when one of several things may be wrong: 
              //  Unless caught by a special case - just clear the curve data and try next start point.
              if(   eRetStatus == SM_ERR_NOT_CONVERGING         // (1022)
                 || eRetStatus == SM_ERR_NOT_WITHIN_TOLERANCE)  // (1021)
                {
#ifdef SM_DEBUG_CODE
                  TCHAR sBuff[SM_TBLOCK_SIZE] ;
#endif // SM_DEBUG_CODE

                  // fix case: when asked to KeepAllDropCurves and the only problem is that 
                  //  the sEnd Point dropped too far along a SrfNormal vector (i.e. it's still over the surface)
                  if(   bKeepAllDropCurves == TRUE
                     && sEnd.m_dApproxDev < sApproxTol3d)
                    {
                      // mark this point as OK and carry on
                      eRetStatus       = SM_SUCCESS ;
                      sEnd.m_bGoodDrop = TRUE ; 

#ifdef SM_DEBUG_CODE
                      // warn the public
                      smos_sprintf(sBuff,_T("DropCurve: Dropped a CurvePoint more than ApproxDist[%16.16lf]: DropDist[%16.16lf], DistFromSrfNorm[%16.16lf]: continuing"),
                                 (double)sApproxTol3d,
                                 sEnd.m_dDropToSurf,
                                 sEnd.m_dApproxDev) ;
                      SM_DBG_WARN(sBuff) ; 
#endif // SM_DEBUG_CODE

                    }
                  else // the sEnd point is not over the surface - warn the public and fail this drop curve
                    {
                      SM_SET3_DROP_CURVE_FAIL(SM_ERR_NOT_WITHIN_TOLERANCE, SM_CTC_KEPT_BEYOND_TOL_DROP) ;   // 1021
#ifdef SM_DEBUG_CODE  
                      if(bDebugMe)
                        {
                          if ( pOptDropCurveFail ) { pOptDropCurveFail->Dump(); }

                          smgfx_Erase() ;
                          pDropCurveFail->Draw() ;  sm_GraphicsLoop() ;
                          sm_GraphicsLoop() ;
                        }

                      smos_sprintf(sBuff,_T("DropCurve: Min3dPtToSurfDist[%16.16lf], Min3dPtToDropUVPtNormLineDist[%16.16lf] more than sApproxTol3d [%16.16lf]\n           No UVCurve being returned. This may be a problem if the caller expected to walk over seams"),
                                 sEnd.m_dDropToSurf,
                                 sEnd.m_dApproxDev,
                                 (double)sApproxTol3d) ;
                      SM_DBG_WARN(sBuff) ; 
#endif // SM_DEBUG_CODE

                    } // end else: m_dApproxDev >= sApproxTol3d

                  // GWC note: If ending up here when crossing a seam is a problem then this method
                  //           needs to be extended to return multiple pieces when crossing a seam.
                  //           That's not too hard to do.  All the code needed for that is almost ready.
                  //           see note below in the bLeavingDomain section.

                } // end if NOT_CONVERGING or NOT_WITHIN_TOLERANCE

#ifdef SM_DEBUG_CODE
              if(   eRetStatus != SM_ERR_NOT_CONVERGING        // (1022)
                 && eRetStatus != SM_ERR_NOT_WITHIN_TOLERANCE  // (1021)
                 && eRetStatus != SM_SUCCESS                   // (1000)
                 && eRetStatus != SM_ERR_OUTSIDE_OF_DOMAIN)     // (1020): an expected domain/seam crossing
                {
                  if( !sEnd.m_bOnPole && !sEnd.m_bLeavingDomain)
                    {
                      SM_DBG_WARN(_T("DropCurve assist sm_GetNextPoint() failed - consider this a bug")) ;
                    }
                }
#endif // SM_DEBUG_CODE

              if(eRetStatus != SM_SUCCESS) // (1000)
                {
                  // when asked - save data for fail report
                  SM_SET3_DROP_CURVE_FAIL(eRetStatus, pDropCurveFail->m_eCreatePathType) ;
                  if(pDropCurveFail->m_eCreatePathType == SM_CTC_UNKNOWN)
                    { pDropCurveFail->m_eCreatePathType = SM_CTC_DROPSTARTPOINT_FAIL ; }

#ifdef SM_DEBUG_CODE  
                  if(bDebugMe)
                    {
                      pDropCurveFail->Dump() ;

                      smgfx_Erase() ;
                      pDropCurveFail->Draw() ; sm_GraphicsLoop() ;
                      sm_GraphicsLoop() ;
                    }
#endif // SM_DEBUG_CODE

                  // clear growing curve CtrlPolygon
                  sCtrlPoly.ReSet() ;
                  sKnots.ReSet() ;

                  // quit the iter(jj) every CacheSpan loop (with no curve to flush) and move onto next StartPoint
                  break;
                } // end if eRetStatus from SmDropPt::GetNextPoint still != SM_SUCCESS after fix chances check

            } // end if eRetStatus from SmDropPt::GetNextPoint != SM_SUCCESS check

#ifdef SM_DEBUG_CODE
          if(sEnd.m_bGoodDrop == FALSE || sStart.m_bGoodDrop == FALSE)
            { // place for a break
              sEnd.m_bGoodDrop = sEnd.m_bGoodDrop ;
            }
#endif // SM_DEBUG_CODE

          // arrive here once we have a good SpanStartPoint and SpanEndPoint 
          //  or when eRetStatus == SM_ERR_OUTSIDE_OF_DOMAIN we have
          //     a good SpanStartPoint and a SpanEndPoint known to be off the surface.
          //      Then need to search for boundary point to get the good sEnd.m_dT SpanEndPoint

          // next: Once eRetStatus == SM_SUCCESS, Drop current SpanIvl from sStart.m_dT to sEnd.m_dT 
          //             to surface as a hermite approx to span's dropped endPts and 1stDerivs.
          //
          //       if dropped hermite approx doesn't pass tol checks or goes out of the domain - 
          //       Using a while loop with a stack, subdivide span into progresively smaller segments  
          //           until each segment's dropped hermite approx passes tol checks and then
          //       Add hermite approx ctrlPts (4 for 1st segment and 3 for each subseqent segment) to sCtrolPoly
          //           and segment params (2 for 1st segment and 1 for each subsequent segment) to sKnots arrays
          //           using sm_AddToCurve().
          double           sData[256*4*2];
          SmTArray<double> sStack(256*4*2,sData) ;
          SmBoolean        bSpanPassesTest;

          // init sStack with 1st tuple:[sEnd.m_dT, U, V, dU, dV], UV = domain point, dUV = domain 1stDeriv
          // (it marks a span that runs from sStart.m_dT to sEnd.m_dT) 
          SM_ASSERT_MSG(   SM_IS_ZERO(sIvl.GetMin() - sStart.m_dT)
                        && SM_IS_ZERO(sIvl.GetMax() - sEnd.m_dT),
                                _T("DropSpan is not maintaining sIvl with sStart.m_dT and sEnd.m_dT values - fix this now")) ;
          SER( sm_PushPoint(sEnd.m_dT, sEnd.m_UVPD, sStack )) ;

          // while step points remain on the stack
          //   In each iter, test Span[sStart.m_dT TopOfStackT] for tol and
          //     for bad span  = subdivide span and push split point to stack
          //     for good span = add span to sCtrlPoly and sKnots and pop last step point from stack
          while ( sStack.GetSize() != 0 )
            {
              // ULONG lMisses = 0;

              // load sEnd.m_dT=param, sEnd.m_UVPD[0]=position point, sEnd.m_UVPD[1]=1stDeriv vec from last entry in sStack
              SER( sm_StackLook(sEnd.m_dT, sEnd.m_UVPD, sStack )) ;
              sIvl.SetMinMax(sStart.m_dT, sEnd.m_dT) ;

              // when span starts and ends on the same domain boundary - 
              //   assume curve between is walking the boundary, set start/end 1stDerivs parallel to boundary 
              SER( sStart.FixBoundaryTangents // in : m_UVPD[0], m_UVPD[1]
                                              // out: m_UVPD[1]: When on bdry, set m_UVPD[1].param = 0
                      (sUVDomain,             // in : Domain range
                       sEnd,                  // i/o: rEnd.m_UVPD[0]=current dropped point, 
                                              //      rEnd.m_UVPD[1]=current drop direction
                                              //      When on bdry, set rEnd.m_UVPD[1].param = 0
                       dUVTolX,               // in : UV tol in the u dir
                       dUVTolY,               // in : UV tol in the v dir
                       sApproxTol3d,          // in : not currently used in this function
                       bFixBTChange )) ;      // out: TRUE = changed a value, FALSE = didn't

#ifdef SM_DEBUG_CODE
              if(bFixBTChange)
                {
                  if(bDebugMe)
                    {
                      smgfx_Erase() ;
                      // draw Brep, Surface (UV, seams, poles, params), Face, Curve (speed, params)
                      my_AssertAndDrawDropCurve(p3dCurve, sInterval, this) ;
                      smgfx_SetLook(3,11, 0,0,1) ; sStart.Draw() ; sm_GraphicsLoop() ;
                      smgfx_SetLook(3,11, 1,.5,0); sEnd.Draw() ; sm_GraphicsLoop() ;
                      smgfx_SetLook(2,12, .2,.4,.7) ; DrawUVPolyline(sCtrlPoly, TRUE) ; sm_GraphicsLoop() ;
                      sm_GraphicsLoop() ;
      
                    } // end if bDebugMe check
                } // end sm_FixBoundaryTangents made a change check
#endif // SM_DEBUG_CODE

              // Check 3dCurve to CurveOnSurf dist represented by cubic Hermite defined by 2 UV positions and tangents
              double dDropToSurfParam ; 
              SER(sStart.TestSpanAccuracy(*this,            // in : target surface                       
                                           sUVDomain,       // in : surface domain of interest           
                                          *pCurve,          // in : curve being dropped 
                                           sEnd,            // in : rEnd.m_dT
                                                            // out: rEnd.m_CrvDP
                                           sApproxTol3d,    // in : min dist between 3d pts                                     
                                           dUVTolX,         // in : UV tol in the u dir
                                           dUVTolY,         // in : UV tol in the v dir
                                           bSpanPassesTest, // out: TRUE = span is good - spans shorter than sIvl/1000.0
                                                            //                            all sample CrvPts within sApproxTol3d of SrfPt Normals 
                                                            //      FALSE= span is bad  - spans that leave srfBdry by more than sApproxTol3d
                                                            //                            sample CrvPt not within sApproxTol3d of SrfPt Normal
                                           dDropToSurf,     // out: SurfacePt to CurvePt distance 
                                           dDropToSurfParam,// out: param value for dDropToSurf
                                           dApproxDev,      // out: Deviation of CurvePt from SrfNormalLine
                                           TRUE)) ;         // in : TRUE = attempt refining span 1stDeriv mags when span fails, FALSE=don't

// GWC: REMOVED BLOCK - This was an idea to refine a span by optimizing its end tangent values.
//       That code works - but the trick was not found to be needed once this function
//       had been cleaned up.  I'm leaving it here in case the idea comes back into favor.
// BEGIN REMOVE BLOCK
//                    // when C1 approx to span fails tol checks - try making a G1 (not C1) span that is within tolerance
//                    if(!bSpanPassesTest)
//                      {
//      #ifdef SM_DEBUG_CODE  // draw and display the problem span geometry
//                        if(bDebugMe) // draw the failing case with many interogation tools including a printed table of gap distances
//                          {
//                            // hermite approximation curve as sUVCrvOnSurf
//                            double          dDeltaT = sIvl.GetLength() ;
//                            SmHermiteCurve  sHermite( sStart.m_UVPD[0], sStart.m_UVPD[1]*dDeltaT, 
//                                                      sEnd.m_UVPD[0],   sEnd.m_UVPD[1]*dDeltaT );
//                            SmBSplineCurve *pCubic  = NULL ;
//                            SmObjDelete     sClean(pCubic) ;
//                            SmBSplineCurve::CreateFromHermiteCurve(*GetContext(), sHermite, pCubic, sStart.m_dT, sEnd.m_dT) ;
//                            SmCrvOnSurf     sUVCrvOnSurf(*pCubic, (SmSurface &)*this) ;
//                        
//                            // pretty print and test surface, curve, approx span
//                            SM_DUMP_AND_ASSERT_VALID(pCubic) ;
//                            SM_DUMP_AND_ASSERT_VALID(&sHermite) ;
//      
//                            smgfx_Erase() ;
//                            // draw Brep, Surface (UV, seams, poles, params), Face, Curve (speed, params)
//                            my_AssertAndDrawDropCurve(p3dCurve, sInterval, this) ;
//                            smgfx_SetLook(3,11, 0,0,1) ; sStart.Draw() ; sm_GraphicsLoop() ;
//                            smgfx_SetLook(3,11, 1,.5,0); sEnd.Draw() ; sm_GraphicsLoop() ;
//                            smgfx_SetLook(2,12, .2,.4,.7) ; DrawUVPolyline(sCtrlPoly, TRUE) ; sm_GraphicsLoop() ;
//                        
//                            // draw just failing curve span
//                            smgfx_SetLook(1,2, 0,1,0); p3dCurve->DrawSpeed( -1.0, 35, &sIvl) ; sm_GraphicsLoop() ;
//                            smgfx_SetLook(3,4, 1,0,0); p3dCurve->DrawParams( &sIvl ); sm_GraphicsLoop();
//      
//                            // draw hermite approx to failing curve span
//                            smgfx_SetLook(2,3, 1,1,0); sUVCrvOnSurf.DrawSpeed( -1.0, 35, &sIvl); sm_GraphicsLoop();
//                            smgfx_SetLook(3,4, 1,0,0); sUVCrvOnSurf.DrawParams( &sIvl ); sm_GraphicsLoop();
//                            sm_GraphicsLoop() ;
//      
//                            // output a dump and draw the relationships between the UVTrimCurve, Surface, and 3dCurve
//                            //    lOutputIndex an orof the following bits
//                            //    0 = [blue]   draw 'this' 3dCurve sample points 
//                            //    1 = [red]    Add UVTrimCurve Points and normals for sample parameter values
//                            //    2 = [green]  Add Drop 3dCurve pts to UVTrimCurve Points and normals
//                            //    4 = [cyan]   Add Drop 3dCurve pts to Surface Points
//                            //    8 = [orange] Add Drop FoundSurface pts to UVTrimCurve Points and normals
//                            //   16 =          Add vectors between drawn points
//                            smgfx_SetLook(2,4, 0,0,1); p3dCurve->DrawInspectUVTrimCurve(*this, *pCubic, &sIvl, 75, (0 | 1 | 2)); sm_GraphicsLoop() ; 
//                            sm_GraphicsLoop();
//      
//                          } // end if bDebugMe check
//      #endif // SM_DEBUG_CODE
//      
//      #ifdef SM_DEBUG_CODE
//      // GWC_NOTE CHANGE_NEXT_CONDITIONAL_TO_bDebugMe_EQ_FALSE_TO_DEBUG_sm_RefineSpan1stDerivs GWC_LINE ;
//                        if(bDebugMe)
//                          {
//                            // Copy Start and End points
//                            SmDropPt sStartCopy = sStart ;
//                            SmDropPt sEndCopy   = sEnd ;
//      
//                            // locals
//                            double dRefinedDropToSurf = 0 ;
//                            double dRefinedApproxDev  = 0 ;
//      
//                            // fit a G1 Span to the Span 
//                            SmStatus sRtn = sStartCopy.RefineSpan1stDerivs
//                              (*this,                                  // in : target surface                                                               
//                                sOffset,                               // i/o: offset of main surface, its offset distance is set in this routine        
//                                lSingularities,                        // in : orof: SM_SS_NONE, SM_SS_UMIN, SM_SS_VMIN, SM_SS_UMAX, SM_SS_VMAX
//                                sUVDomain,                             // in : surface domain of interest                                                   
//                                eSrfClosure,                           // in : oneof: SM_SP_U, SM_SP_V, SM_SP_BOTH, SM_SP_NEITHER                        
//                               *pCurve,                                // in : curve being dropped                                                          
//                                sApproxTol3d,                          // in : max allowed distance between drop point and surfNormal line at drop point,   
//                                dUVTolX,                               // in : UV Tolerance in the u direction                                              
//                                dUVTolY,                               // in : UV Tolerance in the v direction 
//                                sStart.m_eDropDerivRtn == SM_SUCCESS,  // in : TRUE = preserve input aSrfSpanStartUV[1] direction, FALSE = optimize it
//                                sEndCopy,                              // in : sEnd.m_CrvPD, 
//                                                                       //      sEnd.m_UVPD[0] 
//                                                                       // i/o: sEnd.m_UVPD[1] = SpanEnd[UV1stDeriv]
//                                sEnd.m_eDropDerivRtn == SM_SUCCESS,    // in : TRUE = preserve input aSrfSpanStartUV[1] direction, FALSE = optimize it                                      
//                                dDropToSurf,                           // out: Drop distance
//                                dApproxDev) ;                          // out: deviation of sCrvPD[0] from SrfNormalLine starting at Surface(sSrfUV[0])
//      
//                            if(dRefinedDropToSurf < dDropToSurf) { dRefinedDropToSurf = dDropToSurf ; }
//                            if(dRefinedApproxDev  < dApproxDev ) { dRefinedApproxDev  = dApproxDev  ; }
//      
//                            // retest the modified Span
//                            if(sRtn = SM_SUCCESS)
//                              {
//                                SM_ASSERT_MSG(   SM_IS_ZERO(sIvl.GetMin() - sStart.m_dT)
//                                              && SM_IS_ZERO(sIvl.GetMax() - sEnd.m_dT),
//                                              _T("DropSpan is not maintaining sIvl with sStart.m_dT and sEnd.m_dT values - fix this now")) ;
//                                SER(sStartCopy.TestSpanAccuracy
//                                
//                                      (*this,                     // in : target surface                                                             
//                                        sUVDomain,                // in : surface domain of interest                                                 
//                                       *pCurve,                   // in : curve being dropped 
//                                        sEndCopy,                 // in : rEnd.m_dT
//                                                                  // out: rEnd.m_CrvDP                                                       
//                                        sApproxTol3d,             // in : min dist between 3d pts                                                    
//                                        dUVTolX,                  // in : UV tol in the u dir                                                        
//                                        dUVTolY,                  // in : UV tol in the v dir                                                        
//                                        bSpanPassesTest,          // out: TRUE = span is good - spans shorter than sIvl/1000.0                       
//                                                                  //                            all sample CrvPts within sApproxTol3d of SrfPt Normals 
//                                                                  //      FALSE= span is bad  - spans that leave srfBdry by more than sApproxTol3d     
//                                                                  //                            sample CrvPt not within sApproxTol3d of SrfPt Normal        
//                                        dDropToSurf,              // out: SurfacePt to CurvePt distance 
//                                        dApproxDev)) ;            // out: Deviation of CurvePt from SrfNormalLine
//      
//                                if(dRefinedDropToSurf < dDropToSurf) { dRefinedDropToSurf = dDropToSurf ; }
//                                if(dRefinedApproxDev  < dApproxDev ) { dRefinedApproxDev  = dApproxDev  ; }
//      
//                                // if it passes - update the span 1st deriv values (span endPts remain constant)
//                                if(bSpanPassesTest)
//                                  {
//                                    sStart = sStartCopy ;
//                                    sEnd   = sEndCopy ;
//                                    dDropToSurf    = dRefinedDropToSurf ;
//                                    dApproxDev     = dRefinedApproxDev ;
//      
//                                  } // end G1 span passed tol check
//                              } // end RefineSpan1stDeriv return success check
//                          } // if dDebugMe check
//      #endif // SM_DEBUG_CODE
//                      } // end try a G1 Span to save a failing C1 Span approx that did not pass tol check
                              
              // stop subdivision - just accept very short spans
              if(   !bSpanPassesTest                    //     span contains out of tol sample pts
                 && (  smos_Fabs(sEnd.m_dT-sStart.m_dT) // and the interval is very short
                     < sInterval.GetLength()/8000.0))  
                 // && !bLeavingDomain)                    // The bLeavingDomain feature is not currently being used.
                {
                  // pass this short problem interval to stop further subdivision on spans
                  // that just don't converge.  Passing this span builds a 'mostly' valid 
                  // UVTrimCurve with just this small span out of tolerance.
                  bSpanPassesTest = TRUE;
                                         
                  // - GWC: we should review all the prog_test cases that come down this 
                  //        branch and see if DropCurve can be improved to handle them.  
                  //        I've temporarily added a sm_RefineSpan1stDerivs() above to experiment
                  //          with trying to find a G1 span that replaces C1 spans that fail to converge.
                  //        I've also added the call p3dCurve->DrawInspectUVTrimCurve(crSurface, *pCubic, &rIvl, 75, (0 | 1 | 2));
                  //          to dump and draw the gap functions between the UVTrimCurve, surface and 3d curve.  Use
                  //          these for helpful analysis in debug mode.
                  //        1. I know that rapidly changing parameter speed creates spans
                  //           that don't converge with further subdivision.
                  //             I believe something might be done about regularizing the speed changes
                  //             across a span by splitting the span on both curve (already done) and
                  //             surface (not yet done) knot boundaries.  I've checked some of the
                  //             failing cases and they have not been caused by crossing surface knot boundaries.
                  //             and so splitting at surface knots won't be of help.
                  //           I've added the function sm_RefineSpan1stDerivs to have a place to try
                  //             fitting the span with a variety of various different cubics
                  //             a.) fit cubic to interpolate start, stop, and mid points while being G1 with 1st Derivs.
                  //                     (didn't help)
                  //             b.) set cubic to constant speed while interpolating start and stop points while
                  //                      being G1 with 1st Derivs.  (didn't help)
                  //             c.) optimize the square distance between cubic and span
                  //                     Min( Integral( 3dCurve(m_dT) - Surface(UVTrimCubic(m_dT)) dt) over span [sStart.m_dT sEnd.m_dT] 
                  //                      not yet tried
                  //        2. Also it's suspected self-intersecting surfaces may prevent convergence here
                  //             although I've never seen that. 

                } // end if didn't pass, not leaving domain, and small step.

              // if span passes tol check - add dropped hermite approx seg to sCtrlPoly and sKnots and popStack
              if(bSpanPassesTest)  // GWC: section handles the 4 cases
                                   //     case pass and long: addToCurve  (DONE)
                                   //     case pass and short: addToCurve and merge with last span if it is a seam artifact, don't merge if it's a subdivision arrtifact
                                   //           post process out real short steps - remove that attempt from this code
                                   //     case NoPass and long: subdivide and iteration  (DONE)
                                   //     case NoPass and short: not needed - previous test labels these spans as
                                   //              acceptable under the 'get what you can' philosophy of UVTrimCurve generation.               
                {
                  // save max DropDist and ApproxDeviation
                  if( dMaxDropToSurf < dDropToSurf ) { dMaxDropToSurf = dDropToSurf ;
                                                       // dMaxDropToSurfParam = dDropToSurfParam ;
                                                     }
                  if( dMaxApproxDev  < dApproxDev)   { dMaxApproxDev  = dApproxDev ;  }

                  // add a cubicHermite segment to the growing sCtrlPoly and sKnots array
                  //  if thisSpan is not C0 with last, add curve to rUVCurves, restart sCtrlPoly and sKnots with this span
                  SM_ASSERT_MSG(   SM_IS_ZERO(sIvl.GetMin() - sStart.m_dT)
                                && SM_IS_ZERO(sIvl.GetMax() - sEnd.m_dT),
                                _T("DropSpan is not maintaining sIvl with sStart.m_dT and sEnd.m_dT values - fix this now")) ;
                  SmStatus eStat = sm_AddToCurve
                    ( crContext,               // in : in case we create a new curve                                       
                      *this,                   // in : tgt Surface for DropCurve
                      sUVDomain,               // in : used to clamp UVCtrlPoints to remove noise when walking boundaries 
                      *p3dCurve,               // in : curve being dropped in DropCurve
                      sStart,                  // in : Span start drop point 
                      sEnd,                    // in : Span end drop point   
                      sCtrlPoly,               // i/o: growing UVcurve polygon - add 3 controlpoints (4 for 1st segment), 
                                               //      When input span is discontinuous with last span: reset to this input span 
                                               //      z coord values ignored  
                      sKnots,                  // i/o: growing curve knot vector - add sEnd.m_dT (and sStart.m_dT for 1st segment),
                                               //      When input span is discontinuous with last span: reset to this input span   
                      sApproxTol3d,            // in : not used 
                      dUVTolX,                 // in : UV tol in u dir
                      dUVTolY,                 // in : UV tol in v dir                                         
                      rUVCurves,               // out: 1 curve added made from input rCtrlPoly and rknots 
                                               //      when input span is discontinuous with sCtrlPoly or sKnots last span.
                      bFixBCChange) ;          // out: when curve flushed, TRUE = ctrlPts changed by sm_FixBoundaryCurve, FALSE = not changed 
                  
                  // sm_AddToCurve only outputs SM_ERR when sCtrlPoly count exceeds 10,000 points.
                  if ( eStat != SM_SUCCESS )
                    { SE( eStat ) ; } // RCLxx changed from SER

                  // Move End values into Start values for next subdivision (or CacheSpan) iteration - guarantees C1 continuity
                  sStart = sEnd ;
                  sStart.m_bFromLeftU = sEnd.m_bFromLeftU ? FALSE : TRUE ;
                  sStart.m_bFromLeftV = sEnd.m_bFromLeftV ? FALSE : TRUE ;
                  bHaveStartUVTan     = TRUE ;

                  // done with span [sStart.m_dT TopOfStackT] - move to next span by removing top of stack
                  SER( sm_PopStack( sEnd.m_dT, sEnd.m_UVPD, sStack )) ;

                  // algorithm design: exiting this branch is the only exit from the decomposition while loop,
                  //  which guarantees Each Cache iteration begins with a sStart.m_dT and a sStart.m_UVPD[0] value.
                   
#ifdef SM_DEBUG_CODE
                  if(bDebugMe)
                    {
                      smgfx_SetLook(2,4, 0,1,0) ; sOffset.DrawAt(sStart.m_UVPD[0],1) ; sm_GraphicsLoop() ;
                      sm_GraphicsLoop() ;
                    }
#endif // SM_DEBUG_CODE
                   // back to next piece of span in subdivision stack iteration
                   continue ;

                } // end bSpanPassesTest == TRUE branch. Next, move on to next span dropping iteration

              else // bSpanPassesTest == FALSE branch, Crv and SrfProjection over this Span are not within tolerance
                {
                  // seek a dropable split point in span [sStart.m_dT sEnd.m_dT] to make a shorter span to test
                  SmBoolean bFoundGoodOne = FALSE ; // TRUE = found a split point that works in sm_GetNextPoint
                                                    // FALSE= still seeking a dropable split point for current span

                  // Try 6 progressively shorter split points seeking one that can drop in sm_GetNextPoint
                  for(kk=0;kk<6;kk++)
                    {
                      // subdivide the problem span - set sEnd.m_dT to current Span midPoint
                      sEnd.m_dT = (sStart.m_dT + sEnd.m_dT) / 2.0 ;

                      // drop curvePoint(sEnd.m_dT) to find Surface sEnd.m_UVPD and all other sEnd property values
                      eRetStatus = sStart.GetNextPoint
                        (*this,            // in : main surface                                                       
                          sOffset,         // i/o: offset of main surface, its offset distance is set in this routine 
                          lSingularities,  // in : orof: SM_SS_NONE, SM_SS_UMIN, SM_SS_VMIN, SM_SS_UMAX, SM_SS_VMAX, SM_SS_UNKNOWN
                          &sSrfPolePoints, // in : PolePoint[4] array, ordered :[UMinPole, VMinPole, UMaxPole, VMaxPole]
                                           //      NonSingular side values set to
                                           //      SmPoint3d::SetUninitialized(),
                          sUVDomain,       // in : limiting domain of the surface                                     
                          eSrfClosure,     // in : oneof: SM_SP_U, SM_SP_V, SM_SP_BOTH, SM_SP_NEITHER                 
                         *pCurve,          // in : Curve being dropped                                                
                          sApproxTol3d,    // in : max allowed distance between drop point and surfNormal line at drop point                                                                   
                          dUVTolX,         // in : UV tol in u dir
                          dUVTolY,         // in : UV tol in v dir                                            
                          sEnd.m_dT,       // in : param value of SpanEnd CurvePt being dropped                                                
                          sEnd) ;          // out: rEnd.m_dT
                                           //      rEnd.m_CrvPD 
                                           //      rEnd.m_UVPD, rEnd.m_bFromLeftU, rEnd.m_bFromLeftV,
                                           //      rEnd.m_3dPD
                                           //      rEnd.m_dDropToSurf,        rEnd.m_dApproxDev
                                           //      rEnd.m_dNormalToCrvAngDeg
                                           //      rEnd.m_dSrfToCrvAngDeg,    rEnd.m_dSrfToCrvSpeedRatio
                                           //      rEnd.m_bOnBoundary,        rEnd.m_bLeavingDomain,  rEnd.m_bEnteringDomain
                                           //      rEnd.m_eDropDerivRtn   
                                           //         SM_SUCCESS = good drop, 
                                           //         SM_ERR_NOT_WITHIN_TOLERANCE = CrvPt not with tol of SrfNormalLine at dropPoint
                                           //                usually caused by dropping a CrvPt not over the Srf to a SrfBoundary
                                           //         SM_ERR_OUTSIDE_OF_DOMAIN = StartPt drops to SrfBoundary and DropCrv moves out of srf 
                                           //         SM_ERR_BAD_TANGENT_DROP  = CrvTangent dropped to nonParallel SrfTangent
                                           //                usually caused by dropping a CrvTangent near a srfPole                        

                      // done when GetNextPoint can drop current split point 
                      if(eRetStatus == SM_SUCCESS)
                        { 
                          bFoundGoodOne = TRUE ;
                          break ; // exit kk iteration for acceptable break points
                        }
                    }  // end iter(kk) subdividing span until sm_GetNextPoint works

                  // when no dropable split pt(passes sm_GetNextPoint) was found - quit tracing this drop point
                  if ( !bFoundGoodOne )
                    { 
                      // copy accumulated drop points for failure report
                      SM_SET3_DROP_CURVE_FAIL(eRetStatus, SM_CTC_SPLIT_DROPSTEP_FAIL) ; 
#ifdef SM_DEBUG_CODE  
                      if(bDebugMe)
                        {
                        if(pOptDropCurveFail) pOptDropCurveFail->Dump() ;

                          smgfx_Erase() ;
                          pDropCurveFail->Draw() ; sm_GraphicsLoop() ;
                          sm_GraphicsLoop() ;
                        }
#endif // SM_DEBUG_CODE

                      // clear out accumulated drop points
                      sCtrlPoly.ReSet() ; 
                      sKnots.ReSet() ;    

                      // increment jj to terminate iteration on CacheIvls
                      jj = pCC->GetSpanCount() ; // gwc this line should not be needed when breaking out of iter jj

                      // break iter(jj) back to iter(ii) next StartDropPoint
                      break;                    
                    } // end can't find dropable split point check

                  // arrive here when span failed tol test and new sEnd.m_dT split point has been selected

                  // save max DropDist and ApproxDeviation
                  if( dMaxDropToSurf < sEnd.m_dDropToSurf) { dMaxDropToSurf = sEnd.m_dDropToSurf ;
                                                             // dMaxDropToSurfParam = sEnd.m_dT ; 
                                                           }
                  if( dMaxApproxDev  < sEnd.m_dApproxDev)  { dMaxApproxDev  = sEnd.m_dApproxDev ;  }

                  // Push new sEnd.m_dT split param value onto the stack - then loop again on subdivided span [sStart.m_dT, TopOfStackT]
                  SER( sm_PushPoint( sEnd.m_dT, sEnd.m_UVPD, sStack )) ;
#ifdef SM_DEBUG_CODE
                  if(bDebugMe)
                    {
                      smgfx_SetLook(1,2, 0,1,0) ; sOffset.DrawAt(sEnd.m_UVPD[0],1) ; sm_GraphicsLoop() ;
                      sm_GraphicsLoop() ;
                    }
#endif // SM_DEBUG_CODE
                  
                  // don't change sStart.m_dT or sStart.m_UVPD - next subdivision iteration uses those start values 
                  
                  // go to next while stack of EndPts EndPt iteration
                  continue ;
                } // end did not find a good span, so subdivide branch

            } // end while subdividing span until all pieces are small enough to pass tolerance tests
          
          // arrive here after all (or as much as possible) of CacheSpan[Ivl] has been dropped to surface 
          //   adding drop points to sCtrlPoly and sknots. The interval was added as one or 
          //   possibly more segments as needed to make sure each segment passes the tolerance check.
          //   The next iteration sStart object has been updated to the last Span segment sEnd value.
          
          // GWC: this branch of code that outputs two UVCurves for a curve that drops across a seam is not yet implemented.
          //       To get this right will take a little bit of rework.
          //        When GetNextPoint returns sEnd.m_bLeavingDomain == TRUE and
          //                                  sEnd.m_bOnBoundary == TRUE and
          //                                  sEnd.m_UVPD[0] is on Seam boundary, then
          //          Call sEnd.IterateToBoundary to find the curve param value that drops exactly to the seam
          //          Then run some variation of the following to flush the current curve and to restart
          //             the trace on the other side of the seam.  If you choose to do this
          //             make sure to change the comments in the header.
          // Begin Removed Block
          //      // when sm_GetNextPoint split CacheIvl at a SrfDomainBdry - close off current curve, deal with seams
          //      if(bLeavingDomain)
          //        {
          //          // add a UVCurve made from sCtrlPoly control points and sKnots params to rUVCurves array
          //          SER( sm_FlushCrv( crContext, *this, sUVDomain, *p3dCurve, sCtrlPoly, sKnots, dUVTolX, dUVTolY, rUVCurves, bFixBCChange )) ;
          //      
          //          // remember when bdry was crossed near end of curve
          //          double    dScaledZero    = ( sInterval.GetLength() + 1.0 ) * SM_EFF_ZERO_SQRT;
          //          SmBoolean bDoneWithCurve = SM_IS_ZERO_TO_TOL(sInterval.GetMax() - sEnd.m_dT, dScaledZero) ;
          //      
          //          // when a measurable curve fragment remains to be dropped - check to see if crossed boundary is a seam 
          //          if( !bDoneWithCurve )
          //            {
          //              SmExtent1d sSrfDomainU = sUVDomain.GetUInterval() ;
          //              SmExtent1d sSrfDomainV = sUVDomain.GetVInterval() ;
          //              
          //              // When Srf is closed in U and current span ivl ends on bdry - current cacheIvl crossed a U seam
          //              if (   ( eSrfClosure == SM_SP_U || eSrfClosure == SM_SP_BOTH )
          //                  && (   sEnd.m_UVPD[0].x <= sSrfDomainU.GetMin()
          //                      || sEnd.m_UVPD[0].x >= sSrfDomainU.GetMax() ))
          //                {
          //                  // set bLeavingDomain FALSE to force rest of curve to be dropped as 2nd UVCurve
          //                  bLeavingDomain  = FALSE ;
          //                  bHaveStartUVTan = FALSE ;
          //      
          //                  // for next iter - set SrfSpanStartUV.x to other side of U seam
          //                  if ( sEnd.m_UVPD[0].x < sSrfDomainU.GetMid()) { sStart.m_UVPD[0].x = sSrfDomainU.GetMax() ; }
          //                  else                                          { sStart.m_UVPD[0].x = sSrfDomainU.GetMin() ; }
          //                }
          //      
          //              // When Srf is closed in V and current span ivl ends on bdry - current cacheIvl crossed a V seam
          //              if (   ( eSrfClosure == SM_SP_V || eSrfClosure == SM_SP_BOTH )
          //                  && (   sEnd.m_UVPD[0].y <= sSrfDomainV.GetMin()
          //                      || sEnd.m_UVPD[0].y >= sSrfDomainV.GetMax() ))
          //                {
          //                  // set m_bLeavingDomain FALSE to force rest of curve to be dropped as 2nd UVCurve
          //                  bLeavingDomain  = FALSE ;
          //                  bHaveStartUVTan = FALSE ;
          //                  
          //                  // for next iter - set SrfSpanStartUV.y to other side of V seam
          //                  if ( sEnd.m_UVPD[0].y < sSrfDomainV.GetMid() ) { sStart.m_UVPD[0].y = sSrfDomainV.GetMax() ; }
          //                  else                                           { sStart.m_UVPD[0].y = sSrfDomainV.GetMin() ; }
          //                }
          //            } // end if not DoneWithCurve check
          //      
          //          // m_bLeavingDomain still TRUE when rest of Curve does not need to be dropped as 2nd curve
          //          if( bLeavingDomain ) { jj = pCC->GetSpanCount() ; // causes break out of loop over every curve cache decomposition span
          //                                 break ; // exit iter every CacheSpan loop
          //                               }
          //          else                 { if(jj > 0) jj--;         // We still have to finish this Cache span.
          //                               }
          //        } // end if curve left domain check
          // END REMOVED BLOCK


        } // end iter(jj) every curve cache decomposition span

      // arrive here after all curve cacheSpans have been dropped to surface as 
      //   a sequence of piecewise C1 cubic segments (where each segment passes tol checks)
      //   whose ordered controlPts are in sCtrlPoly and ordered Param values are in sKnots

      // add a UVCurve made from sCtrlPoly control points and sKnots params to rUVCurves array
      SER( sm_FlushCrv( crContext, *this, sUVDomain, *p3dCurve, sCtrlPoly, sKnots, dUVTolX, dUVTolY, rUVCurves, bFixBCChange )) ;

     } // end iter(ii) each successfully dropped startPoint

  // exit case - no curves built
  if (rUVCurves.GetSize() < 1)
    {
      // no need to Save failure data - that has already been saved
      return SM_SUCCESS;  // was eRetStatus [bd 091029]
    }

  // save sampled max Deviation value seen during construction (sampled only at segment endPts)
  rdMaxDropToSurf = dMaxDropToSurf ;
  rdMaxApproxDev  = dMaxApproxDev ; 
  // if(pMaxDropToSurfParam) { *pMaxDropToSurfParam = dDropToSurfParam ; }

  // next: calc accurate maxDeviation between Crv and UVCrv projection
  SM_OBJ_ARRAY(sDistances,  double, 16) ;
  SM_OBJ_ARRAY(sDeviations, double, 16) ;
  // SM_OBJ_ARRAY(sParams,  double, 16) ;

  // for every UVTrimCurve - get distance from 3dCurve to UVTrimCurve projection
  for(ii=0;ii<rUVCurves.GetSize() ;ii++)
    {
      SmBSplineCurve *pBSC   = rUVCurves[ii];
      SmExtent1d      sUVIvl = pBSC->GetNaturalInterval() ;
      
      // Following block is obsolete. bIsReversed no longer used
      //// if the algorithm was run on the reverse parameterized curve
      //if(bIsReversed)
      //  {
      //    // reverse these output UVCurves
      //    SmExtent1d sCrvIvl = cr3dCurve.GetNaturalInterval() ;
      //    SmExtent1d sRevIvl ;
      //    pBSC->ReverseParameterization(sUVIvl, sRevIvl) ;

      //    // make sure the UV intervals match up with the input intervals

      //    // map the UVCurve Ivl to the original Curve interval
      //    double dNewMin = crInterval.GetMin() + (sInterval.GetMax() - sUVIvl.GetMax()) / sInterval.GetLength()  * crInterval.GetLength() ;
      //    double dNewMax = crInterval.GetMax() - (sCrvIvl.GetMin() - sInterval.GetMin()) / sInterval.GetLength() * crInterval.GetLength() ;
      //    sUVIvl.SetMinMax(dNewMin, dNewMax) ;

      //    // adjust parameter interval back to input parameter interval range
      //    pBSC->EditParameterization(sUVIvl, FALSE) ;

      //  } // end if the algorithm was run on the reverse parameterized curve

      // Determine 3D deviation from projection
      SmSurface  * pThisSurf = SM_CONST_CAST(SmSurface*,this) ;
      SmCrvOnSurf  sCrvOnSurf( *pBSC, *pThisSurf, NULL, 0 ) ;

      // We need both the distance and the deviation from the normal,
      // so call SmCrvOnSurf::SurfaceCurveMaxDistanceBetween().  [B315]

      double dMaxDropToSurfT, dMaxDropToSurfOtherT ;
      double dMaxDevT, dMaxDevOtherT ;
      sCrvOnSurf.SurfaceCurveMaxDistanceBetween
           (sUVIvl,               // in : interval limit for this curve
           *p3dCurve,             // in : other curve to test
            sUVIvl.GetMin(),      // in : OtherCurve param mapping to ThisCurve Interval.Min value
            sUVIvl.GetMax(),      // in : OtherCurve param mapping to ThisCurve Interval.Max value
            20,                   // in : Min number of samples to take - it measures at least this many points
            sApproxTol3d,         // in : Distance where nearby DropPoint solutions will be considered the same solution
            NULL,                 // in : Max allowed gap, either Dist or Dev.
                                  //      Quit searching once this value is exceeded.
                                  //      NULL to ignore.   Never quit search when NULL.
            dDropToSurf,          // out: Set to signed max distance seen along the surface normal.
            dApproxDev,           // out: Set to max lateral deviation seen.
           &dMaxDropToSurfT,      // out: Curve param for returned MaxDist Found, NULL to ignore.
           &dMaxDropToSurfOtherT, // out: OtherCurve param for returned MaxDist Found, NULL to ignore.
           &dMaxDevT,             // out: Curve param for returned MaxDev Found, NULL to ignore.
           &dMaxDevOtherT) ;      // out: OtherCurve param for returned MaxDev Found, NULL to ignore.
        
      // save sampled MaxDist between 3DCurve and Surf(UVCurve)
      if ( dDropToSurf > rdMaxDropToSurf ) { rdMaxDropToSurf = dDropToSurf; 
                                             // if(pMaxDropToSurfParam) { *pMaxDropToSurfParam = dMaxDropToSurfT ; }
                                           }
      if ( dApproxDev > rdMaxApproxDev ) { rdMaxApproxDev = dApproxDev; }
      sDistances .Add( dDropToSurf ) ;
      sDeviations.Add( dApproxDev  ) ;
 //     sParams.Add( dMaxDropToSurfT) ;

#ifdef SM_DEBUG_CODE
      if(bDebugMe)
        {
          pBSC->Dump() ;

          smgfx_Erase() ;
          smgfx_SetLook(2,3, 1,0,0) ; DrawUVCurve(*pBSC) ; sm_GraphicsLoop() ;
          // draw Brep, Surface (UV, seams, poles, params), Face, Curve (speed, params)
          my_AssertAndDrawDropCurve(p3dCurve, sInterval, this) ;
          sm_GraphicsLoop() ;
        }
#endif // SM_DEBUG_CODE
    } // end iter every built UVTrimCurve - getting max Crv/UVCrv distances

  // arrive here after successfully building dropCurves - make them permanent
  sCleanUVCrvs.Clear() ;

  // when asked - cull curves with drop distances larger than sApproxTol3d (i.e. curves not on the surface)
  // NOTE: This mixes DropDistance with sApproxTol3d, it's a hack to work around a missing dMaxDropDist input argument.
  //       One thing that might be done, is to add dMaxDropDist to the inupt argument list.
  if(   bKeepAllDropCurves == FALSE
     && ( rdMaxDropToSurf > sApproxTol3d  ||  rdMaxApproxDev > sApproxTol3d )
     && rUVCurves.GetSize() > 1)
    {
      SmTArray<SmBSplineCurve*> sBetterCurves;
      dDropToSurf = 0.0;
      dApproxDev  = 0.0;
  
      // save the dropCurves with small drop distances
      for(ii=0; ii<rUVCurves.GetSize() ; ii++)
        {
          if (sDistances[ii] < sApproxTol3d)
            {
              sBetterCurves.Add(rUVCurves[ii]) ;
              rUVCurves[ii] = NULL;
              if(dDropToSurf < sDistances[ii]) { dDropToSurf = sDistances[ii] ; 
                                                 // dDropToSurfParam = sParams[ii] ; 
                                               }
              if(dApproxDev  < sDeviations[ii]) { dApproxDev = sDeviations[ii]; }
            }
        }

      // delete all dropCurves with large drop distances
      for(ii=0; ii<rUVCurves.GetSize() ; ii++)
        {
          if (rUVCurves[ii]) { delete rUVCurves[ii]; rUVCurves[ii] = NULL ; }
        }
  
      // set output to dropCurves with small drop distances
      rdMaxDropToSurf = dDropToSurf ;
      rdMaxApproxDev  = dApproxDev ;
      // if(pMaxDropToSurfParam) { *pMaxDropToSurfParam = dDropToSurfParam ; }
      rUVCurves.ReSet() ;
      rUVCurves.Append(sBetterCurves) ;
  
    } // end need to look for dropCurveurves with small drop distances check

  // all done
  SM_SET4_DROP_CURVE_FAIL(SM_SUCCESS, pDropCurveFail->m_eCreatePathType) ;
  if(pDropCurveFail->m_eCreatePathType == SM_CTC_UNKNOWN)
                    { pDropCurveFail->m_eCreatePathType = SM_CTC_DROPCURVE_OKAY ; }

  return SM_SUCCESS;

} // end SmSurface::DropCurve

/*******************************************************************//**
PURPOSE: This method creates a parameter space image of a 3D curve
    dropped onto a surface using normal projection (projection
    along surface normals).

NOTES: 
    It is specifically designed to drop 3D trimming curves to create 
    2D UVTrimCurves.  One UVTrimCurve (or two when on a seam) will 
    be returned for every section of the 3dcurve that lies over the
    target surface.  This is different than SmSurface::DropCurve which
    only returns a UVTrimCurve (or two when on a seam) when the entire
    curve drops to the surface. 
    
    The 2D parameter space UVTrimCurves produced will be B-Spline curves  
    with approximately the same parameterization as the 3D curve.

  The SMLib verions of Drop has the following restrictions:
    1) The curve can cross the boundary of the surface or the seam.
    2) The surface can have interior discontinuities.
    3) If the curve crosses the surface boundary or interior discontinuity
           edges, the curve must lie on the surface (it is intersected with
           the boundary and discontinuity curves).

  NOTE: This will find only curves that intersect a boundary of the surface.
  For curves that are entirely inside the surface boundaries, use DropCurve().

***********************************************************************/
SmStatus SmSurface::DropAndTrimCurve
 (
   const SmContext      & crContext,            // in : context for newly created geometry
   const SmExtent2d     & crUVDomain,           // in : Domain limits for accepting the dropped curve
   const SmCurve        & cr3dCurve,            // in : Curve to project onto the surface
   const SmExtent1d     & crInterval,           // in : cr3dCurve interval to drop
   SmApproxTol3d          sApproxTol3d,         // in : max allowed drop distance.
                                                //      The projected curve is broken up into more than one piece
                                                //      everytime the projection curve goes outside the surface boundary
                                                //      by more than this amount.  
   double               & rdMaxDropToSurf,      // out: MaxDist(DropPt, 3dCurvePt), found by sampling, may be slightly less than actual max.
   double               & rdMaxApproxDev,       // out: MaxDist(DropPt->SrfNormalLine, 3dCurvePt), Quality Measure: expected to be close to 0.0
   SmTArray<SmBSplineCurve*> & rUVCurves,       // out: Projected UVTrimCurves      
   SmBoolean            bKeepAllDropCurves,     // in : TRUE = keep all drop curves
                                                //      FALSE=Only keep curves when rdMaxDropToSurf <= sApproxTol3d,
                                                //      default:[TRUE]      
   SmBoolean            bOptCurveIsOnSurface,   // in : TRUE=cr3dCurve is known to lie on the Surface
                                                //      default:[FALSE]
   SmDropCurveFail    * pOptDropCurveFail       // out: Optional data container of a DropCurve fail or success
                                                //      NUll to ignore, default:[NULL]
 ) const          
{
  // always gather DropCurve data
  SmDropCurveFail sDropCurveFail, * pDropCurveFail = pOptDropCurveFail ? pOptDropCurveFail : &sDropCurveFail ;

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
static ULONG lCount = 0 ; lCount++ ;
// draw curve over given interval(red), draw surface(blue) ;
  if (bDebugMe) 
    {
      Dump();
      cr3dCurve.Dump();

      smgfx_Erase() ;
      smgfx_SetLook(3,5, 1,0,0) ; cr3dCurve.DrawWDeriv(crInterval,0); sm_GraphicsLoop();
      smgfx_SetLook(1,3, 0,1,1) ; DrawUV(4,4); sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

  // init outputs
  rUVCurves.ReSet();
  rdMaxApproxDev  = 0.0;
  rdMaxDropToSurf = 0.0;
  pDropCurveFail->UnInit() ;

  // locals
  SmTArray<SmCurve*> sUVCurves;
  SmTArray<double> sMaxDropDists;
  SmTArray<double> sDeviations;

  // prepare the SurfaceDropCurve operation object
  SmSurfaceDropCurve sDrop(this,crUVDomain,cr3dCurve,crInterval);
  sDrop.SetCurveIsOnSurface(bOptCurveIsOnSurface);

  // trace out a UVTrimCurve by projecting cr3dCurve points onto Surface
  SER(sDrop.DoTrace
       (crContext,           // in : context for new object construction 
        sApproxTol3d == 0.0  // in : max allowed deviation of TrimCurve from ideal Projection, NULL for m_dThisApproxTol3d
          ? NULL             
          : &sApproxTol3d,   
        NULL,                // in : 
        NULL,                // out: Surface Curves - not always a 3d curve for every UVDropCurve
        &sUVCurves,          // out: UVDropCurves (more than 1 if projected to seam or in/out of boundary)
        &sMaxDropDists,      // out: max 3dCurveSmpPoint to DropSurfPoint dist for each p3DCurves
        &sDeviations)) ;     // out: max 3dCurveSmpPoint to DropSurfNormLine dist for each p3DCurves

  // for every resulting UV curve (more than one if curve projected to seam
  //     or in/out of surface UVDomain boundary)
  // - Check for a too-big drop distance, if bKeepAllDropCurves is False.
  // - Set outputs: rUVCurves, rdMaxDropToSurf, rdMaxApproxDev.

  ULONG ii, lNumUV = sUVCurves.GetSize();
  for ( ii=0; ii<lNumUV; ii++ )
    {
      SmBSplineCurve * pBSC = SM_CAST_PTR( SmBSplineCurve, sUVCurves[ii] );

// No longer have to do this: DoTrace() returns that info now.
//    // Determine 3D deviation from projection
//    SmExtent1d sIvl      = pBSC->GetNaturalInterval();
//    SmSurface *pThisSurf = SM_CONST_CAST(SmSurface*,this);
//    SmCrvOnSurf sCrvOnSurf(*pBSC,*pThisSurf, NULL, 0, &crContext);
//
//    double dDistToCurve;
//    SER(cr3dCurve.CurveMaxDistanceBetween(sIvl, sCrvOnSurf,
//                                          sIvl.GetMin(), sIvl.GetMax(),
//                                          20, NULL, dDistToCurve));

      // skip curves too far from Surface when bKeepAllDropCurves is FALSE
      if(   bKeepAllDropCurves == FALSE 
         && sMaxDropDists[ii] > sApproxTol3d) 
        {
          delete pBSC; pBSC = NULL ; 
          continue; // Don't keep this curve it is too far away
        }

      // set outputs: 
      // 1. UVTrimCurve array
      // 2. max Dist from Curve to 3DMap of Surface(UVTrimCurve)
      // 3. max projection distance from 3dCurve to Surface normal for a ThroughPoint  
      rUVCurves.Add( pBSC );
      rdMaxDropToSurf = smos_Max( rdMaxDropToSurf, sMaxDropDists[ii] );
      rdMaxApproxDev  = smos_Max( rdMaxApproxDev,  sDeviations  [ii] );

#ifdef SM_DEBUG_CODE
      if (bDebugMe &&
          !pBSC->GetNaturalInterval().ContainsValue(crInterval.GetMax()-SM_EFF_ZERO)) 
        {
          cr3dCurve.Dump();
          pBSC->Dump();

          SmSurface *pThisSurf = SM_CONST_CAST(SmSurface*,this);
          SmCrvOnSurf sCrvOnSurf(*pBSC,*pThisSurf, NULL, 0, &crContext);
          smgfx_Erase();
          smgfx_SetLook( 2,4, 0,0,0 ); sCrvOnSurf.Draw(); sm_GraphicsLoop();
          smgfx_SetLook( 2,4, 1,0,0 ); cr3dCurve.DrawWDeriv( crInterval ); sm_GraphicsLoop();
          smgfx_SetLook( 1,0, 0,1,1 ); this->DrawUV(4,4); sm_GraphicsLoop();
          sm_GraphicsLoop();
        }
#endif // SM_DEBUG_CODE

    } // end iter every drop curve solution to set outputs

  // capture the output for debug and heal purposes
  SmExtent2d sUVDomain = GetNaturalUVDomain() ; 
  SM_SET_DROP_AND_TRIM_CURVE_FAIL(SM_SUCCESS, pDropCurveFail->m_eCreatePathType) ;
    if(pDropCurveFail->m_eCreatePathType == SM_CTC_UNKNOWN)
      { pDropCurveFail->m_eCreatePathType = SM_CTC_DROPANDTRIMCURVE_OKAY ; }

  return SM_SUCCESS;

} // end SmSurface::DropAndTrimCurve

/*******************************************************************//**
PURPOSE: When input 3dCurve drops to a surface isoParameter curve
   with the same parameterization over the given Interval, build and 
   return a UVLine isoParameter curve, else don't return a curve.  

NOTES:
   Checks that Curve drops to isoCurve by checking that
     1. startTangent == a Surface U or V 1st Derivative at the Drop point
     2. A sequence of evenly spaced Curve/Surface points along
        the detected SurfaceTangent direction are coincident in that direction.
        (A 3Dcurve with the same shape as the isoParameter Curve but different
         parameterization will fail this check.)
     3. the parametric length of the isoCurve is nonDegenerate.

   when all checks are passed, a line UVTrimCurve is created
   and stored in the output.
***********************************************************************/
SmStatus SmSurface::DropIsoCurve
(
  const SmContext      & crContext,            // in : context for new object construction
  const SmExtent2d     & crUVDomain,           // in : This Surface domain of interest
  const SmCurve        & cr3dCurve,            // in : Curve to drop 
  const SmExtent1d     & crInterval,           // in : Curve interval of interest
  const SmPoint2d      & crStartUVPoint,       // in : Surface UVPoint corresponding to start of3d curve.
  SmApproxTol3d          sApproxTol3d,         // in : max allowed distance between drop point and surfNormal line at drop point 
  SmBoolean            & rbSuccessfulIsoDrop,  // out: TRUE = dropped a curve
  double               & rdMaxDropToSurf,      // out: DropDist = max sample distance found between cr3dCurve and Surface
                                               //      taken at cr3dCurve.NumberOfKnot evenly spaced samples
  SmBSplineCurve      *& rpIsoUVCurve,         // out: Pointer to newly allocated UVLine curve or
                                               //      NULL if no curve were constructed
  double               * pdMaxApproxDev        // out: Max CurveToDrop smplPoint to DropSurfNormLine dist
) const

{
  // check input
  SM_ASSERT_MSG(rpIsoUVCurve == NULL, _T("SmSurface::DropIsoCurve() NonNULL input pointer")) ;

  // init output
  rbSuccessfulIsoDrop = FALSE;
  rdMaxDropToSurf     = 0.0;
  if(pdMaxApproxDev)      { *pdMaxApproxDev = 0.0 ; }

  // locals
  SmVector3d sPV[2]; // sPV[0] = cr3dCurve 3dPoint
                     // sPV[1] = cr3dCurve 3dTangent
  // double dTolSq  = sApproxTol3d*sApproxTol3d; // GWC: CHANGE
  double dOneDeg = 1.0*SM_PI/180.0;   
  
  // Check Curve Tangent == one of Surface 1st parametric derivatives
  SmBoolean bLeftT = crInterval.GetTLeftEval(crInterval.GetMin());
  SER(cr3dCurve.Evaluate(crInterval.GetMin(),1,bLeftT,sPV, TRUE));   // nonZeroTangent values

  // when curve StartTangent == 0.0 look for nonDegenerate tangent near StartPoint
  if (sPV[1].LengthSquared() < SM_EFF_ZERO_SQ) 
    {
      // try for startTangent near StartPoint
      SER(cr3dCurve.Evaluate(crInterval.Evaluate(SM_EFF_ZERO_SQRT),1,bLeftT,sPV));
      if (sPV[1].LengthSquared() < SM_EFF_ZERO_SQ) 
        {
          // try for startTangent further from startPoint
          SER(cr3dCurve.Evaluate(crInterval.Evaluate(1.0e-3),1,bLeftT,sPV));
          if (sPV[1].LengthSquared() < SM_EFF_ZERO_SQ) 
            {
              // could not find a nonZero tangent near startPoint - quit
              return SM_SUCCESS;
            }
        }
    } // end Degenerate curve check

  // arrive here when curve startTangent in sPV[1] is nonZero
  SM_ASSERT(sPV[1].LengthSquared() >= SM_EFF_ZERO_SQ) ;

  // locals
  SmPoint3d  sPnt;
  SmVector3d sDU, sDV;
  SmVector2d sDirVec;
  double     dAngU  = SM_PI/2.0;
  double     dAngV  = SM_PI/2.0;
  SmBoolean  bFound = FALSE;
  double     dScale = 1.0;

  // for 4 possible surface directions
  for (ULONG k=0; k<4; k++) 
    {
      SmBoolean bLeftU = TRUE;
      SmBoolean bLeftV = TRUE;
      if (k==1) { bLeftV = FALSE; }
      if (k==2) { bLeftV = FALSE; 
                  bLeftU = FALSE; }
      if (k==3) { bLeftU = FALSE; }

      // evaluate the Surface 1st derivatives
      SER(Evaluate1stDerivatives(crStartUVPoint, bLeftU, bLeftV, sPnt, sDU, sDV));

      // Project crv tan vectors to srf plane before checking angles.
      // (Previously was not necessary because curves were required to lie in the surface.)
      SmVector3d sNorm = sDU * sDV;
      double dNormLen = sNorm.Length();
      if ( dNormLen > SM_EFF_ZERO )
        { sNorm /= dNormLen; }
      else
        { SER( EvaluateNormal( crStartUVPoint, bLeftU, bLeftV, sNorm )); }
      SmVector3d sCrvTanProj = sPV[1].ProjectToPlane( sNorm );

      // get Curve/Surface tangent angles
      dAngU = SM_PI/2.0;
      dAngV = SM_PI/2.0;
      if(sDU.LengthSquared() > SM_EFF_ZERO_SQ) { SER( sDU.AngleBetween( sCrvTanProj, dAngU )); }
      if(sDV.LengthSquared() > SM_EFF_ZERO_SQ) { SER( sDV.AngleBetween( sCrvTanProj, dAngV )); }

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
      if (bDebugMe) 
        {
          sm_GraphicsLoop();
          smgfx_SetLook(2,2, 1,0,0); cr3dCurve.DrawAt(crInterval.GetMin(),1); sm_GraphicsLoop();
          smgfx_SetLook(1,2, 0,1,0); DrawAt(crStartUVPoint,1);                sm_GraphicsLoop();
          sm_GraphicsLoop();
        }
#endif // SM_DEBUG_CODE
      // Note following effectively checks for derivatives being in the same direction.
      if (dAngU < dOneDeg) 
        {
          SER(Evaluate1stDerivatives(crStartUVPoint, TRUE,TRUE,sPnt,sDU,sDV));
          dScale  = sDU.Length() / sPV[1].Length();
          sDirVec = SmPoint2d(1,0);
          bFound  = TRUE; 
          break;
        }
      else if (dAngU > SM_PI - dOneDeg) 
        {
          SER(Evaluate1stDerivatives(crStartUVPoint, FALSE,FALSE,sPnt,sDU,sDV));
          dScale  = sDU.Length() / sPV[1].Length();
          sDirVec = SmPoint2d(-1,0);
          bFound  = TRUE; 
          break;
        }
      else if (dAngV < dOneDeg) 
        {
          SER(Evaluate1stDerivatives(crStartUVPoint, TRUE,TRUE,sPnt,sDU,sDV));
          dScale  = sDV.Length() / sPV[1].Length();
          sDirVec = SmPoint2d(0,1);
          bFound  = TRUE; 
          break;
        }
      else if (dAngV > SM_PI - dOneDeg) 
        {
          SER(Evaluate1stDerivatives(crStartUVPoint, FALSE,FALSE,sPnt,sDU,sDV));
          dScale  = sDV.Length() / sPV[1].Length();
          sDirVec = SmPoint2d(0,-1);
          bFound  = TRUE; 
          break;
        }
    } // end iter 4 possible surface derivative directions

  // when curveTangent is not parallel to a surface 1st Derivative - quit
  if (!bFound) 
    { return SM_SUCCESS; }
  
  // arrive here when start CurveTangent is parallel to a Surface 1st derivative at the dropPoint
  
  // get Curve knots - trim them to the input interval range
  double dKnots[100];
  SmTArray<double> sKnots(100,dKnots);
  SER(cr3dCurve.GetKnots(sKnots));

  // remove knots below and above the crInterval range
  while(   sKnots.GetSize() > 0
        && sKnots[0] < crInterval.GetMin() - SM_EFF_ZERO_SQRT) 
    { sKnots.RemoveAt(0,1);
    }
  while (   sKnots.GetSize() > 0 
         && sKnots.GetLast() > crInterval.GetMax() + SM_EFF_ZERO_SQRT) 
    { sKnots.RemoveLast();
    }

  // make sure to have a knot at the crInterval Min and Max boundaries
  if (   sKnots.GetSize() == 0 
      || !SM_ARE_SAME(sKnots[0],crInterval.GetMin())) 
    { sKnots.InsertAt(0,crInterval.GetMin(),1);
    }
  if (   sKnots.GetSize() == 1 
      || !SM_ARE_SAME(sKnots.GetLast(),crInterval.GetMax())) 
    { sKnots.Add(crInterval.GetMax());
    }

  // for every Curve span between knots 
  //  - check for curve/surfaceIsoLine coincidence with equal parameterization
  double dIvlSize = crInterval.GetMax() - crInterval.GetMin();
  double dMaxDeviation = 0.0;
  double dMaxDrop      = 0.0;
  double dDropLineDist ;         
  //      double dMaxDiffSq = 0.0;           // GWC : CHANGE
  ULONG  lNumKnots = sKnots.GetSize();
  for (ULONG i=1; i<=lNumKnots; i++) 
    {
      // get Surface UVPoint at even sized param steps along surface isoParameter Curve
      double dStep =   (i < lNumKnots)
                     ? (i/(double)lNumKnots)*dIvlSize
                     : dIvlSize ;
      SmVector2d sUVTest = crStartUVPoint + sDirVec * dStep / dScale;
      sUVTest = crUVDomain.ClampPoint2d(sUVTest);

      // Clamp dParam to CurveInterval to account for tolerances
      double dParam = crInterval.ClampValue(crInterval.GetMin() + dStep);
      
      // get curve/surfaceNormal gap for parameter points
      SmPoint3d sCPnt, sSPnt, sSNrm ;
      double    dLineParam ;
      SER(cr3dCurve.EvaluatePoint(dParam,sCPnt));
      SER(EvaluatePoint (sUVTest,sSPnt));
      SER(EvaluateNormal(sUVTest, TRUE, TRUE, sSNrm));  // unitized normal
      smgu_LinePointDistance(sSPnt, sSNrm, sCPnt, dDropLineDist, &dLineParam) ;  // dLineParam is drop dist because sSNrm is unitLength

      // when gap is too large - quit
      if(dDropLineDist > sApproxTol3d)
        { return SM_SUCCESS; }

      // save the largest gap seen
      // The distance from the point to the surface will be fabs( dLineParam ):
      // that's the param along the unit surface normal.
      if ( dLineParam < 0 ) { dLineParam = -dLineParam; }  // [B296]
      if(dMaxDeviation < dDropLineDist) { dMaxDeviation = dDropLineDist ; }
      if(dMaxDrop      < dLineParam)    { dMaxDrop      = dLineParam ;
                                        }

    } // end iter every curve span looking for evenly spaced coincident curve/surface points

  // get the surface parameter start/end points
  SmPoint3d sUVStart(crStartUVPoint);
  SmPoint3d sUVEnd  = SmPoint3d(crUVDomain.ClampPoint2d(crStartUVPoint + sDirVec*dIvlSize/dScale));

  // skip degenerate parameter space curves - GWC: need a change - short curves could get short trim curves - need to compare 3d curve lengths?
  double    dUVSize = sUVStart.DistanceBetween( sUVEnd );
  if(   dUVSize < SM_EFF_ZERO_SQRT               // short UVTrimCurve
     && (   sDU.LengthSquared() <= SM_EFF_ZERO   // near a surface singulartiy
         || sDV.LengthSquared() <= SM_EFF_ZERO)) // gwc: keep short curves on NonSingular Surfaces - they are just short
    { return SM_SUCCESS; }

  // Passed all tests - make and output a parameter space line.
//cbi Why not create an SmLine?
  rbSuccessfulIsoDrop = TRUE;
  rdMaxDropToSurf     = dMaxDrop ;
  if(pdMaxApproxDev)      { *pdMaxApproxDev      = dMaxDeviation ; }
  SER( SmBSplineCurve::CreateLineSegment( crContext, 2, sUVStart, sUVEnd, rpIsoUVCurve ));
  SER( rpIsoUVCurve->EditParameterization( crInterval ));

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  if(bDebugMe)
    {
      SM_ASSERT_VALID(rpIsoUVCurve) ;
      SmCrvOnSurf sCrvOnSurf( * rpIsoUVCurve, (SmSurface &)*this) ;
      ULONG lSmpCnt = smos_Max(20, lNumKnots * 5) ; 
      SmCrvCrvGapFunction sFaceTrimCurveCrvGF((SmXSectTol3d)sApproxTol3d, &sCrvOnSurf, (SmExtent1d &)crInterval, &cr3dCurve,  lSmpCnt) ; 
      SmCrvCrvGapFunction sCrvFaceTrimCurveGF((SmXSectTol3d)sApproxTol3d, &cr3dCurve,  (SmExtent1d &)crInterval, &sCrvOnSurf, lSmpCnt) ; 
      SmCrvSrfGapFunction sFaceTrimCurveSrfGF((SmXSectTol3d)sApproxTol3d, &sCrvOnSurf, (SmExtent1d &)crInterval, this, lSmpCnt) ;
      SmCrvSrfGapFunction sCrvSrfGF          ((SmXSectTol3d)sApproxTol3d, &cr3dCurve,  (SmExtent1d &)crInterval, this, lSmpCnt) ;

      SmGapSample *pMaxFaceTrimCurveCrvGap; sFaceTrimCurveCrvGF.GetMaxGapSample(pMaxFaceTrimCurveCrvGap);
      SmGapSample *pMaxCrvFaceTrimCurveGap; sCrvFaceTrimCurveGF.GetMaxGapSample(pMaxCrvFaceTrimCurveGap);
      SmGapSample *pMaxFaceTrimCurveSrfGap; sCrvSrfGF.GetMaxGapSample(pMaxFaceTrimCurveSrfGap);
      SmGapSample *pMaxCrvSrfGap;           sCrvSrfGF.GetMaxGapSample(pMaxCrvSrfGap);
      
      //double dMaxFaceTrimCurveCrvGap = sMaxFaceTrimCurveCrvGap.GetLength() ; 
      //double dMaxCrvFaceTrimCurveGap = sMaxCrvFaceTrimCurveGap.GetLength() ; 
      //double dMaxFaceTrimCurveSrfGap = sMaxFaceTrimCurveSrfGap.GetLength() ; 
      //double dMaxCrvSrfGap         = sMaxCrvSrfGap.GetLength() ; 

      smos_WriteBuffer(_T("Begin Dump FaceTrimCurve/Edge->Curve GapFunction\n")) ;
      sFaceTrimCurveCrvGF.Dump() ;
      smos_WriteBuffer(_T("End Dump FaceTrimCurve/Edge->Curve GapFunction\n")) ;
      smos_WriteBuffer(_T("Begin Dump Edge->Curve/FaceTrimCurve GapFunction\n")) ;
      sCrvFaceTrimCurveGF.Dump() ;
      smos_WriteBuffer(_T("Begin Dump Edge->Curve/FaceTrimCurve GapFunction\n")) ;

      SmPoint3d sTrimCurve3d, sEdge3d ;
      ULONG lIndex = 0 ;
      double dTrimCurveParam = sFaceTrimCurveCrvGF.GetGap(lIndex).GetThisParam(0) ;
      double dEdgeParam      = sFaceTrimCurveCrvGF.GetGap(lIndex).GetOtherParam(0);
      sCrvOnSurf.EvaluatePoint(dTrimCurveParam, sTrimCurve3d) ;
      cr3dCurve.EvaluatePoint(dEdgeParam, sEdge3d) ;

      smgfx_Erase() ;
      SmExtent1d sTempIvl = cr3dCurve.GetNaturalInterval();
      // draw Brep, Surface (UV, seams, poles, params), Face, Curve (speed, params)
      my_AssertAndDrawDropCurve(&cr3dCurve, sTempIvl, this) ;

      SmSurface *pNonConstThis = SM_CAST_NONNULL_PTR( SmSurface, this );
      SmCrvOnSurf sCOS( *rpIsoUVCurve, *pNonConstThis );
      smgfx_SetLook( 3,5, 1,0,1 ); sCOS.Draw(); sm_GraphicsLoop();

      smgfx_SetLook(2,3, 1,0,0) ; cr3dCurve.Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 1,0,0) ; sFaceTrimCurveCrvGF.Draw(FALSE, FALSE, TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,0,1) ; sCrvFaceTrimCurveGF.Draw(FALSE, FALSE, TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,0) ; sFaceTrimCurveSrfGF.Draw(FALSE, FALSE, TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 1,0,1) ; sCrvSrfGF.Draw(FALSE, FALSE, TRUE) ; sm_GraphicsLoop() ;        
      sm_GraphicsLoop() ;

    }
#endif // SM_DEBUG_CODE
  return SM_SUCCESS;

} // end SmSurface::DropIsoCurve

// GWC: OBSOLETE idea for having GetNextPoint find span crossing seam points
//      NEW IDEA - no partial curve drops
//      POTENTIAL OTHER IDEA - In DropCurve under the commented out section
//           that checks for if(bLeavingDomain) implement a crossing seam
//           identification and response behavior.
//      In all 3 cases this function will not be used.  I'm obsoleteing it.
//      /*******************************************************************//**
//      PURPOSE: Check to see whether the step from the previous point to the
//        current point is leaving the domain (on boundary, pushing outward),
//        in either u- or v-direction.
//      
//      NOTES: 
//      ***********************************************************************/
//      // note: this function is not being used
//      static SmStatus sm_CheckPushingBoundary
//       (const SmSurface  & crSurface,           // in : target surface being dropped to
//        const SmExtent2d & crUVDomain,          // in : surface domain
//        SmSurfParamType    eWhichDir,           // in : SM_SP_U or SM_SP_V
//        const SmCurve    & cr3dCurve,           // in : target curve being dropped
//        double             sStart.m_dT,         // in : Curve span start param
//        double             sEnd.m_dT,           // in : Curve span end   param
//        const SmPoint3d    caCrvSpanStartPV[2], // in : caCrvSpanStartPV[0] = cr3dCurve 3dPoint
//                                                //      caCrvSpanStartPV[1] = cr3dCurve 3dTangent
//        SmApproxTol3d      sApproxTol3d,        // in : DropPoint tolerance = sApproxTol3d * 5.0
//        SmPoint3d        & rCrvSpanEndPt,       // i/o: EndCrvSpan->3dPoint,               Curve(sEnd.m_dT)
//                                                //        when(rbIsPushing == TRUE) set to Curve(rdCrossingT)
//        SmPoint2d        & rSrfSpanEndUV,       // i/o: EndSrfSpan UVPoint,                DropPoint(Curve(sEnd.m_dT))
//                                                //        when(rbIsPushing == TRUE) set to DropPoint(Curve(rdCrossingT)))
//        SmPoint3d        & rSrfSpanEndPt,       // i/o: EndSrfSpan 3dPoint,                Surface(DropPoint(Curve(sEnd.m_dT)))
//                                                //        when(rbIsPushing == TRUE) set to Surface(DropPoint(Curve(rdCrossingT)))
//        SmBoolean        & rbIsPushing,         // out: TRUE = rSrfSpanEndUV point is pushing boundary, FALSE=not
//        SmBoolean        & rbOnLowBdry,         // out: when(rbIsPushing==TRUE) TRUE = rSrfSpanEndUV on ParamMin Bdry, FALSE = on ParamMax Bdry
//        double           & rdCrossingT,         // out: when(rbIsPushing==TRUE) CurveParam of Curve/SrfBdry intersection 
//        double           & rdDeviation)         // out: when(rbIsPushing==TRUE) Distance in SrfBdryCrossDir direction from rCrvSpanEndPt to rSrfSpanEndPt 
//      {
//        // init output
//        rbIsPushing = FALSE;
//        rbOnLowBdry = FALSE;
//        rdCrossingT = sEnd.m_dT;
//        rdDeviation = 0;
//      
//        // load direction asked for.
//        double     dSrfSpanEndUVParam;
//        SmExtent1d sDimension;
//        if ( eWhichDir == SM_SP_U ) { dSrfSpanEndUVParam = rSrfSpanEndUV.x;
//                                      sDimension         = crUVDomain.GetUInterval();
//                                    }
//        else                        { dSrfSpanEndUVParam = rSrfSpanEndUV.y;
//                                      sDimension         = crUVDomain.GetVInterval();
//                                    }
//      
//        // get tolerance
//        double dScaledZero = SM_EFF_ZERO * ( 1.0 + sDimension.GetLength() ); 
//      
//        // When EndSrfSpan param is within the surface interval and not on boundary
//        if(   dSrfSpanEndUVParam > sDimension.GetMin() + dScaledZero
//           && dSrfSpanEndUVParam < sDimension.GetMax() - dScaledZero )
//          {
//            // Not on or outside the boundary, can't be pushing.
//            return SM_SUCCESS;
//          }
//      
//        // Evaluate locals
//        SmPoint3d  sSrfEndPt;
//        SmVector3d sSrfEndSu, sSrfEndSv;
//        rbOnLowBdry = dSrfSpanEndUVParam < sDimension.GetMid();
//      
//        // Evaluate EndSrfSpan position and 1st derivatives.
//        crSurface.Evaluate1stDerivatives( rSrfSpanEndUV, rbOnLowBdry, TRUE, sSrfEndPt, sSrfEndSu, sSrfEndSv );
//        
//        // locals for cross-boundary and along-boundary derivatives.
//        SmVector3d sCrossDeriv, sBndryDeriv;  
//        if(eWhichDir == SM_SP_U ) { sCrossDeriv = sSrfEndSu;
//                                    sBndryDeriv = sSrfEndSv;
//                                  }
//        else                      { sCrossDeriv = sSrfEndSv;
//                                    sBndryDeriv = sSrfEndSu;
//                                  }
//      
//        // StartCrvSpan and EndSrfSpan 1stDerivative lengths
//        double dSrfLenSq = sCrossDeriv.LengthSquared();
//        double dCrvLenSq = caCrvSpanStartPV[1].LengthSquared();
//      
//        // when StartCrvSpan and EndSrfSpan 1stDerivs are nonZero - quit
//        if ( dSrfLenSq * dCrvLenSq < SM_EFF_ZERO_SQ )
//          {
//            // Can't get good derivs: just don't worry about this one then.
//            return SM_SUCCESS;
//          }
//      
//        // get StartCrvSpan 1stDeriv/EndSrfSpan 1stDeriv angle as: cos(theta) 
//        double dDot = sCrossDeriv.Dot( caCrvSpanStartPV[1] ) / smos_Sqrt( dSrfLenSq * dCrvLenSq );
//      
//        // negate for low boundary
//        if ( rbOnLowBdry )
//          { dDot = -dDot; }
//      
//        // when StartPoint and direction is pushing in or pushing out by less than about 5.7 degrees
//        if(dDot < 0.1) // if(cos(theta) < .1)  // cos(90 - 5.7 degrees) ~= .1, so angles between 85 to 275 degrees
//          {
//            // Not pushing out
//            return SM_SUCCESS;
//          }
//      
//        // arrive here when
//        //   rSrfSpanEndUV       (EndSrfSpan UVPoint) is within tol of or outside surface boundary 
//        //   caCrvSpanStartPV[1] (StartCrvSpan->1stDeriv) points in same dir as sCrossDeriv (EndSrfSpan->CrossTangent)
//        // GWC: why are we comparing the StartCrvSpan->Tangent with the EndSrfSpan->CrossTangent
//      
//        // DropCurve is pushing outside the surface domain
//        rbIsPushing = TRUE;
//      
//        // Find the exact curve projected to surface crossing the surface boundary point.
//        // Original thought: curve/curve intersect with boundary isocurve.
//        // Problem: curve is off the surface, not parallel to it, so the
//        //          closest point to the curve is not directly above the seam.
//        // We want the point on the curve that drops to the seam -- directly above it.  
//        // So: find where the curve crosses the plane of the seam.
//        //
//        // But first, it's common for the guess to be right on.
//        SmBoolean bFound;
//      
//        // get curr Crv/Srf gap and dot that with the cross derivative to get distance to boundary
//        SER( sCrossDeriv.Unitize()) ;
//        SmVector3d sGap( rCrvSpanEndPt - sSrfEndPt );
//        dDot = sGap.Dot( sCrossDeriv );
//        if ( rbOnLowBdry )
//          { dDot = -dDot; }
//      
//        // when length of gap projected in CrossDeriv direction is less than tight tolerance
//        dScaledZero = SM_EFF_ZERO * ( 1.0 + sSrfEndPt.GetMaxDimension() );
//        if(dDot <= 100*dScaledZero)
//          {
//            // rCrvSpanEndPt is right on surface boundary
//            rdDeviation = sGap.Length();
//            bFound      = TRUE;
//          }
//        else // rCrvSpanEndPt is not on surface boundary - find Curve/SrfBdry crossing
//          {
//            // A real crossing, have to solve.
//            //   1. Get CurveParam at Curve/SeamPlane intersection
//            //   2. Get Curve3D Point for CurveParam
//            //   3. Drop Curve3D Point to Surface to SurfaceUV point
//            //      3a.  To save time, Drop Curve3D Point to SurfaceBdry IsoParameter line
//      
//            // Note: if Sv is near zero, then multiplying it twice will be
//            //       really small, so unitize them both first.
//            SER( sBndryDeriv.Unitize() );
//      
//            // normal of plane containing Surface Seam and SurfaceNormal at sSrfSpanEndPt
//            SmVector3d sPlaneNorm = sBndryDeriv * sCrossDeriv * sBndryDeriv;
//            SER( sPlaneNorm.Unitize() );
//      
//            // locals for LocalPropertyAnalysis call
//            SmExtent1d sCrvIvl( sStart.m_dT, sEnd.m_dT );
//            sCrvIvl.ExpandRelative( 0.2 );
//            double     dCurveGuess = sCrvIvl.GetMid();
//            double     dDValue  = - sSrfEndPt.Dot( sPlaneNorm );
//            SmSolution sSol;
//      
//            // intersect curve with plane of seam
//            // GWC: Are all seams planar? (certainly for cylinders, spheres, and cones - but not general BSplines)
//            SER( cr3dCurve.LocalPropertyAnalysis( sCrvIvl,                  // in : Curve interval to search
//                                                  SM_CP_PLANE_INTERSECTION, // in : Curve Property
//                                                  dCurveGuess,              // in : Guess Point = Span Mid Point
//                                                  &dDValue, &sPlaneNorm,    // in : Plane equation as norm = (A B C) and d
//                                                  bFound,                   // out: TRUE = plane crossing found
//                                                  sSol ));                  // out: the solution
//            // if ( !bFound )
//            //   { SER( SM_ERR ); }
//      
//            // curve param at curve/SeamPlane intersection
//            rdCrossingT = sSol.m_vStart[0];
//      
//            // watch for noise at curve start boundaries
//            if ( rdCrossingT < sStart.m_dT )
//              { rdCrossingT = sStart.m_dT; } // can happen at the beginning of curves.
//      
//            // But we also need the surface uv.  We know it's on the
//            // boundary, so it's easier to drop to that.
//            if(bFound)
//              {
//                // Surface boundary isoParameter Curve
//                double dIsoParam = rbOnLowBdry ? sDimension.GetMin() : sDimension.GetMax();
//                SmIsoCurve sIsoCurve( crSurface, eWhichDir, dIsoParam, rbOnLowBdry, FALSE );
//      
//                // locals for Drop CurvePoint to SurfaceIsoParameter Curve call
//                double dGuessT = ( eWhichDir == SM_SP_U ) ? rSrfSpanEndUV.y : rSrfSpanEndUV.x;
//                double dPrmOnBdry;
//                cr3dCurve.EvaluatePoint( rdCrossingT, rCrvSpanEndPt );
//      
//                // Drop CurvePoint to SurfaceIsoParameter Curve
//                sIsoCurve.DropPoint( sIsoCurve.GetNaturalInterval(), 
//                                     rCrvSpanEndPt, 
//                                     NULL,
//                                     5.0*sApproxTol3d,      // when the increase in sApproxTol3d - it's not been used up to this point
//                                     &dGuessT,
//                                     bFound, 
//                                     dPrmOnBdry, 
//                                     rdDeviation );
//                // when CurvePoint dropped to SurfaceIsoParameterCurve
//                if(bFound)
//                  {
//                    // Set output rSrfSpanEndUV 
//                    if ( eWhichDir == SM_SP_U ) {  rSrfSpanEndUV.Set( dIsoParam, dPrmOnBdry );
//                                                }
//                    else                        {  rSrfSpanEndUV.Set( dPrmOnBdry, dIsoParam );
//                                                }
//      
//                    // set output rSrfSpanEndPt
//                    crSurface.EvaluatePoint( rSrfSpanEndUV, rSrfSpanEndPt );
//                  }
//              } // end if bFound == TRUE for curve/SeamPlane intersection check
//      
//          } // end rCrvSpanEndPt is not on surface boundary branch - solve for crossing
//      
//        // all done
//        return SM_SUCCESS;
//      
//      } // end sm_CheckPushingBoundary()
// GWC: END OBSOLETE
