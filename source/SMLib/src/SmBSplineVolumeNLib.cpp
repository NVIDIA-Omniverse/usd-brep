// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************//**
* FILE NAME --- SmBSplineVolumeNLib.cpp 
* PURPOSE: Implementation of methods used exclusively with
* NLib product.  
* Oct-2006 - gwc - initial version
**********************************************************************/

#include "StdAfx.h"

#include <SmConfig.h>

#include <nurbs.h>
#include <SmContext.h>
#include <SmBSplineCurve.h>
#include <SmBSplineSurface.h>
#include <SmBSplineVolume.h>
#include <SmAxis2Placement.h>
#include <SmGraphicsExtern.h>
#include <SmGraphicsOutput.h>
#include <SmNurbsCrv.h>     // GW_MAX_DEGREE
#include <SmAssertArray.h>
#include <SmPlane.h>

// This object automatically calls the initilization and termination of the GW nurb volume stack.
class SmNLibStackHandler 
{
  private:
    NL_STACKS * m_pStacks;
  public:
    SmNLibStackHandler(NL_STACKS * pStacks) { m_pStacks = pStacks; N_InitNurbs(m_pStacks); }
   ~SmNLibStackHandler()                 { N_EndNurbs(m_pStacks); }
} ; // end class SmNLibStackHandler

// prints error and returns status
#define NL_ERR(a) { \
     NL_FLAG nl_ERR = (a); \
     if (nl_ERR == NL_YES) { SER(SM_ERR); } }

#define COPY_XYZ(from,to) (to).x = (from).x; (to).y = (from).y; (to).z = (from).z;

#if 0
/*******************************************************************//**
PURPOSE:

NOTES: UNUSED
***********************************************************************/
static VOLUME * sm_CreateNlibVolume
  (const SmBSplineVolume *pBSV,
   NL_STACKS & SG)
{
    VOLUME *pNewVol = N_AllocVolumeAndArrays((NL_INDEX)pBSV->GetNumberControlPoints(SM_VP_U),
                                             (NL_INDEX)pBSV->GetNumberControlPoints(SM_VP_V),
                                             (NL_INDEX)pBSV->GetNumberControlPoints(SM_VP_W),
                                             (NL_DEGREE)pBSV->GetDegree(SM_VP_U),
                                             (NL_DEGREE)pBSV->GetDegree(SM_VP_V),
                                             (NL_DEGREE)pBSV->GetDegree(SM_VP_W),
                                             (NL_INDEX)pBSV->GetNumberNaturalKnots(SM_VP_U),
                                             (NL_INDEX)pBSV->GetNumberNaturalKnots(SM_VP_V),
                                             (NL_INDEX)pBSV->GetNumberNaturalKnots(SM_VP_W),
                                             &SG);

    VOLUME *pVol = pBSV->GetOrCreateGwNurbPointer();
    if (N_VolumeCopy(pVol,pNewVol,&SG)) 
      {
        SE(SM_ERR);
        return NULL;
      }
    return pNewVol;

} // end sm_CreateNlibVolume


/*******************************************************************//**
PURPOSE: Elevate the degree of a NURB volume in either U, V, or W
 by specified increment (1,2,..) without changing the geometry 
 
NOTES: This method actually elevate the degree only if NLib is
   present.
***********************************************************************/
SmStatus SmBSplineVolume::DegreeElevate
  (SmVolumeParamType eDirectionToElevate,  // in : elevation direction:
                                           //       oneof SM_VP_U, SM_VP_V, or SM_VP_W
   ULONG             lNewDegree)           // in : desired degree in specified direction
{
  // locals
  NL_STACKS SG;
  SmNLibStackHandler sSH(&SG);
  NL_VOLUME vol;
  N_VolumeInitArrays(&vol);
  NL_VOLUME * volP = GetOrCreateGwNurbPointer();
  NER(volP);
  NL_INDEX increm ;

  // pass the call along
  if(eDirectionToElevate == SM_VP_ALL)
    {
      increm = lNewDegree - GetDegree(SM_VP_U);
      NL_ERR(N_VolumeElevateDegree(volP, increm, NL_UDIR, &vol, &SG, &SG));

      increm = lNewDegree - GetDegree(SM_VP_V);
      NL_ERR(N_VolumeElevateDegree(&vol, increm, NL_VDIR, &vol, &SG, &SG));

      increm = lNewDegree - GetDegree(SM_VP_W);
      NL_ERR(N_VolumeElevateDegree(&vol, increm, NL_WDIR, &vol, &SG, &SG));
    }
  else
    {
      NL_FLAG UVWFlag =   eDirectionToElevate == SM_VP_U   ? NL_UDIR
                     : eDirectionToElevate == SM_VP_V   ? NL_VDIR
                     :                                    NL_WDIR ;

      increm = lNewDegree - GetDegree(eDirectionToElevate);
      NL_ERR(N_VolumeElevateDegree(volP, increm, UVWFlag, &vol, &SG, &SG));
    }

  const SmContext * pContext = GetContext();
  NER(pContext);

  // place output vol in temporary pTemp
  SmBSplineVolume *pTemp = new (*pContext) SmBSplineVolume(pContext, (gw_VOLUME *)&vol);
  SmObjDelete sClean(pTemp);

  // swap ptemp and this m_pNurb pointers (deletes the old m_pNurb object)
  gw_VOLUME * pNewVol = pTemp->GetOrCreateGwNurbPointer();
  pTemp->m_pNurb      = volP ;
  m_pNurb             = pNewVol;

  // all done
  return SM_SUCCESS;

} // end SmBSplineVolume::DegreeElevate


// gwc: a function to consider adding later on
//      
//      /*******************************************************************//**
//      PURPOSE: Estimate average length of a volume in u-, v-, and w- directions,
//          and it computes a bound on volume area by taking the product of the
//          maximums of length in u-, v-, and w- directions. 
//      
//      NOTES: This method actually reduces the degree only if NLib is
//         present.
//      ***********************************************************************/
//      SmStatus SmBSplineVolume::GetMeasures
//        (double & rdAverageLengthU,        // out: 
//         double & rdAverageLengthV,        // out: 
//         double & rdAverageLengthW,        // out: 
//         double & rdEstimatedVolumeBound)  // out: 
//      {
//        NL_REAL   A, lu, lv, lw;
//        VOLUME * volP = GetOrCreateGwNurbPointer() ;
//        NER(volP);
//      
//        NL_ERR(N_SrfGetAverageLen(volP,&A,&lu,&lv));
//      
//        rdAverageLengthU = lu;
//        rdAverageLengthV = lv;
//        rdEstimatedVolumeBound = A;
//      
//        return SM_SUCCESS;
//      
//      } // end SmBSplineVolume::GetMeasures



/*******************************************************************//**
PURPOSE: Insert one new  knot into a NURBS volume either in u-, v- or 
    w-direction. 

NOTES: The new knot must be an interior knot and the sum of
    the multiplicities of the old and the new knots must be less than
    or equal to the respective degree. 
    
      This method is only available for users with NLib
    
***********************************************************************/
SmStatus SmBSplineVolume::InsertOneKnot
  (double            dNewKnot,            // in : parameter value to insert 
   ULONG             lNumKnotInsertions,  // in : Number of times to insert the knot
   SmVolumeParamType eVolumeParam)        // in : oneof: SM_VP_U, SM_VP_V, SM_VP_W
{
  // locals
  NL_STACKS SG;
  SmNLibStackHandler sSH(&SG);
  NL_VOLUME vol;
  N_VolumeInitArrays(&vol);

  // set N_VolumeInsertKnot arguments
  NL_FLAG   dir =   eVolumeParam == SM_VP_U ? NL_UDIR
               : eVolumeParam == SM_VP_V ? NL_VDIR
               :                           NL_WDIR ;
  NL_INDEX  nt  = lNumKnotInsertions;
  NL_VOLUME * volP = GetOrCreateGwNurbPointer() ;
  NER(volP);

  // pass the call along
  NL_ERR(N_VolumeInsertKnot(volP,dNewKnot,nt,dir,&vol,&SG,&SG));

  // make temporary SMBSplineVolume
  const SmContext * pContext = GetContext();
  NER(pContext);
  SmBSplineVolume *pTemp = new (*pContext) SmBSplineVolume(pContext, (gw_VOLUME *)&vol);
  SmObjDelete sClean(pTemp);

  // swap m_pNurb pointers (so the old one gets freed with the temporary SmBSplineVolume)
  gw_VOLUME * pNewVol = pTemp->GetOrCreateGwNurbPointer();
  pTemp->m_pNurb      = volP ;
  m_pNurb             = pNewVol;

  // all done
  return SM_SUCCESS;

} // end SmBSplineVolume::InsertOneKnot

#endif

/*******************************************************************//**
PURPOSE: Return TRUE when minimum internal discontinuity is

NOTES: Note given SM_VP_U this compute equivalent continuity to the U knot vector
       and the same for SM_VP_V and SM_VP_W.
***********************************************************************/
SmBoolean SmBSplineVolume::HasDiscontinuitiesSimple
 (SmDiscontinuities3d * pOptDisconts,       // out: optional list of discontinuities and summary data, NULL to ignore,
                                            //      default:[NULL]
  SmBoolean             bCalcGeometric)     // in : TRUE = expensive - use geometric testing to compute actal geometric discontinuities
                                            //      FALSE= cheap - report representational discontinuites
                                            //      default:[TRUE]
 const  
{
  // locals - and optional indirection
  SmDiscontinuities3d  sDisconts ;
  SmDiscontinuities3d *pDisconts = pOptDisconts ? pOptDisconts : &sDisconts ;

  // init output
  pDisconts->Init() ;

  // Calc the continuities in U
  CalculateContinuitiesSimple(SM_VP_U, pDisconts->m_eMinContU, pDisconts->m_sParamsU, pDisconts->m_sContsU, bCalcGeometric) ;

  // Calc the continuities in V
  CalculateContinuitiesSimple(SM_VP_V, pDisconts->m_eMinContV, pDisconts->m_sParamsV, pDisconts->m_sContsV, bCalcGeometric) ;

  // Calc the continuities in W
  CalculateContinuitiesSimple(SM_VP_W, pDisconts->m_eMinContW, pDisconts->m_sParamsW, pDisconts->m_sContsW, bCalcGeometric) ;

  // all done
  return(   pDisconts->m_eMinContU < SM_CT_G1
         || pDisconts->m_eMinContV < SM_CT_G1
         || pDisconts->m_eMinContW < SM_CT_G1) ;

} // end SmBSplineVolume::HasDiscontinuitiesSimple

/*******************************************************************//**
PURPOSE: Return TRUE when Curve does not cross any volume's boundaries or
         internal C1 discontinuities and is contained within the ParamSpace domain

NOTES: For BSplineVolumes, NaturalParam Domain = External boundaries
       Knot vectors and control point positions can creaate internal discontinuities

       For now - just check external boundaries and boundaries created by knots
***********************************************************************/
SmBoolean SmBSplineVolume::IsCurveInParamDomainSimple
 (const SmCurve & crParamCurve)       // in : Tgt ParamSpace Curve to check
 const       
{ 
  // locals
  SmExtent3d  sParamDomain   = GetNaturalParamDomain() ;
  SmPseudoBox sParamPseudoBox(sParamDomain) ; // note: PseudoBox and BBox are equivalent because BBox is orthogonal and axis aligned
  SmExtent1d  sCurveInterval = crParamCurve.GetNaturalInterval() ;
  SmExtent3d  sCurveBBox ;
  SmPseudoBox sCurvePseudoBox ; 

  SM_OBJ_ARRAY(sPlanePoints, SmPoint3d, 6) ;
  SM_OBJ_ARRAY(sPlaneNormals, SmVector3d, 6) ;

  // low work - check BBoxes for gross inside/outside classifications        
  crParamCurve.CalculateBoundingBox(sCurveInterval, &sCurveBBox, &sCurvePseudoBox) ;

  if(sCurveBBox.IsContainedBy(sParamDomain, SM_EFF_ZERO))         { return TRUE ; }
  if(sCurveBBox.AreDisjoint  (sParamDomain, SM_EFF_ZERO))         { return FALSE ; }
  if(sCurvePseudoBox.IsContainedBy(sParamPseudoBox, SM_EFF_ZERO)) { return TRUE ; }
  if(sCurvePseudoBox.AreDisjoint  (sParamPseudoBox))              { return FALSE ; }

  // now check the tight bounding box - a little more expensive
  crParamCurve.CalculateTightBoundingBox(sCurveInterval, &sCurveBBox) ;
  if(sCurveBBox.IsContainedBy(sParamDomain, SM_EFF_ZERO))         { return TRUE ; }
  if(sCurveBBox.AreDisjoint  (sParamDomain, SM_EFF_ZERO))         { return FALSE ; }

  // arrive here when more expensive check is needed

  // get the ParamBBox bounding planes
  sParamDomain.GetPlanes(sPlanePoints, sPlaneNormals) ;

  // is curve coincident, touching, or on positive side of all bounding planes
  SmBoolean bRtn = crParamCurve.IsOnPositiveSideOfPlanes(sPlanePoints, sPlaneNormals) ;

  // all done
  return(bRtn) ; 

 } // end SmBSplineVolume::IsCurveInParamDomainSimple

/*******************************************************************//**
PURPOSE: Return TRUE when Surface does not cross any volume's boundaries or
         internal C1 discontinuities and is contained within the ParamSpace domain

NOTES: For BSplineVolumes, NaturalParam Domain = External boundaries
       Knot vectors and control point positions can creaate internal discontinuities

       For now - just check external boundaries and boundaries created by knots
***********************************************************************/
SmBoolean SmBSplineVolume::IsSurfaceInParamDomainSimple
 (const SmSurface & crParamSurface)   // in : Tgt ParamSpace Surface to check
 const 
{ 
  // locals
  SmExtent3d                 sParamDomain = GetNaturalParamDomain() ;
  SmPseudoBox                sParamPseudoBox(sParamDomain) ;
  SmExtent2d                 sSurfaceUVDomain = crParamSurface.GetNaturalUVDomain() ;
  SmExtent3d                 sSurfaceBBox ;
  SmPseudoBox                sSurfacePseudoBox ;

  // low work - check BBoxes for gross inside/outside classifications        
  crParamSurface.CalculateBoundingBox(sSurfaceUVDomain, &sSurfaceBBox, &sSurfacePseudoBox) ;
  if(sSurfaceBBox.IsContainedBy(sParamDomain, SM_EFF_ZERO))         { return TRUE ; }
  if(sSurfaceBBox.AreDisjoint  (sParamDomain, SM_EFF_ZERO))         { return FALSE ; }
  if(sSurfacePseudoBox.IsContainedBy(sParamPseudoBox, SM_EFF_ZERO)) { return TRUE ; }
  if(sSurfacePseudoBox.AreDisjoint  (sParamPseudoBox))              { return FALSE ; }

  // now check the tight bounding box - a little more expensive
  crParamSurface.CalculateTightBoundingBox(sSurfaceUVDomain, &sSurfaceBBox) ;
  if(sSurfaceBBox.IsContainedBy(sParamDomain, SM_EFF_ZERO))         { return TRUE ; }
  if(sSurfaceBBox.AreDisjoint  (sParamDomain, SM_EFF_ZERO))         { return FALSE ; }

  // arrive here when Surface needs to be checked for intersections with volume discontinuity planes

  // locals
  SM_PTR_ARRAY(sPlanes, SmPlane, 16) ; 

  // Get all discontinuity planes = ParamBBox bounding planes + internal discontinuity planes

  // locals
  ULONG ii, dir ; 
  SmVolumeParamType          eParamType ;
  SmExtent2d                 sPlaneDomain ;
  SmVector3d                 sPlanePoint, sPlaneNormal, sPlaneU, sPlaneV ;
  SmExtent1d                 sUIvl = sParamDomain.GetUInterval() ;
  SmExtent1d                 sVIvl = sParamDomain.GetVInterval() ;
  SmExtent1d                 sWIvl = sParamDomain.GetWInterval() ;
  SmVector3d                 sU(1,0,0), sV(0,1,0), sW(0,0,1) ;
  SmContinuityType           eMinContinuity ;   
  SmTArray<double>           sParams ;          
  SmTArray<SmContinuityType> sConts ;           
  SmContext                  sContext ; 
  SmVector2d                 sUVScale(1,1) ;

  // Get U, V, and W Discontinuities defined by U, V, ane W knot vectors (ignore discontinuities due to CPoint positions)
  for(dir=0;dir<3;dir++)
    {
      // locals and inits
      if     (dir == 0) { eParamType = SM_VP_U ; 
                          sPlaneNormal.Set(1,0,0) ; 
                          sPlaneU = sV ; sPlaneDomain.SetUInterval(sVIvl) ;
                          sPlaneV = sW ; sPlaneDomain.SetVInterval(sWIvl) ;
                        }
      else if(dir == 1) { eParamType = SM_VP_V ; 
                          sPlaneNormal.Set(0,1,0) ; 
                          sPlaneU = sW ; sPlaneDomain.SetUInterval(sWIvl) ; 
                          sPlaneV = sU ; sPlaneDomain.SetVInterval(sUIvl) ; 
                        }
      else              { eParamType = SM_VP_W ; 
                          sPlaneNormal.Set(0,0,1) ; 
                          sPlaneU = sU ; sPlaneDomain.SetUInterval(sUIvl) ; 
                          sPlaneV = sV ; sPlaneDomain.SetVInterval(sVIvl) ; 
                        }

      // Get list of U,V, or W valued discontinuties
      CalculateContinuitiesSimple
       (eParamType,      // in : SM_VP_U, SM_VP_V, or SM_VP_W
        eMinContinuity,  // out: minimum continuity over all interior knots
        sParams,         // out: param values marking discontinuity
        sConts,          // out: assocaited continuity type for each rParams value
                         //      end param continuities = SM_CT_DISCONTINUOUS
        FALSE) ;         // in : TRUE = expensive - use geometric testing to compute actal geometric discontinuities
                         //      FALSE= cheap - report representational discontinuites
                         //      default:[TRUE]

      // for every discontinuity
      ULONG lDiscontCnt = sParams.GetSize() ; 
      for(ii=0;ii<lDiscontCnt;ii++)
        {
          // no work - listed discontinuity is continuous enough
          if(sConts[ii] >= SM_CT_C1_C2)
            { continue ; }

          // locals
          if     (dir == 0)     { sPlanePoint.Set(sParams[ii],0,0) ; }
          else if(dir == 1)     { sPlanePoint.Set(0,sParams[ii],0) ; }
          else                  { sPlanePoint.Set(0,0,sParams[ii]) ; }

          // negate normal on last plane so that it points into the ParamDomain
          if(ii==lDiscontCnt-1) { sPlaneNormal = -sPlaneNormal ; }

          // make and store Trim planes
          SmPlane *pPlane = new (sContext) SmPlane(sPlanePoint,
                                                   sPlaneU,
                                                   sPlaneV,
                                                   sUVScale,
                                                   sPlaneDomain,
                                                   &sContext) ; 

          sPlanes.Add(pPlane) ;

        } // end iter every discontinuity plane
    } // end iter every U,V, and W direction

  // Make pPlanes memory temporary
  SmObjsDelete<SmPlane *> sClean(&sPlanes) ; 

  // arrive here when sPlanes, are built.

  // return value - assume FALSE - update by classifying surface against discontinuity planes
  SmBoolean bRtn = FALSE ; 

  // check for surface crossing discontinuity planes check
  SmBoolean bHasPlaneCrossings = crParamSurface.HasPlaneCrossings(sPlanes) ;

  // when surface is on the inside or the outside of the Volume ParamDomain
  if(bHasPlaneCrossings == FALSE)
    {
      // classify one SurfacePoint 
  
      // Surface MidPoint loc
      SmPoint2d sMidUVPoint    = sSurfaceUVDomain.Evaluate(.5,.5) ;
      SmPoint3d sMidParamPoint ;
      crParamSurface.EvaluatePoint(sMidUVPoint, sMidParamPoint) ;

      // classify MidPoint against Volume ParamDomain
      bRtn = sParamDomain.ContainsPoint3d(sMidParamPoint, SM_EFF_ZERO) ;
    } 

  // all done - return final classification
  return bRtn ;

} // end SmBSplineVolume::IsSurfaceInParamDomainSimple

/*******************************************************************//**
PURPOSE: Compute the continuity of a volume along either the U, V or W directions

NOTES: Note given SM_VP_U this computes equivalent continuity to the U knot vector
       and the same for SM_VP_V and SM_VP_W.
***********************************************************************/
SmStatus SmBSplineVolume::CalculateContinuitiesSimple
 (SmVolumeParamType            eVolumeParam,      // in : SM_VP_U, SM_VP_V, or SM_VP_W
  SmContinuityType           & reMinContinuity,   // out: minimum continuity over all interior knots
  SmTArray<double>           & rParams,           // out: param values marking discontinuity
  SmTArray<SmContinuityType> & rConts,            // out: assocaited continuity type for each rParams value
                                                  //      end param continuities = SM_CT_DISCONTINUOUS
  SmBoolean                    bCalcGeometric)    // in : TRUE = expensive - use geometric testing to compute actal geometric discontinuities
                                                  //      FALSE= cheap - report representational discontinuites
                                                  //      default:[TRUE]
 const
{
  // init output
  reMinContinuity = SM_CT_CINFINITY ;
  rParams.ReSet();
  rConts.ReSet();

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  // draw
  if(bDebugMe)
    {
      SM_DUMP_AND_ASSERT_VALID(this) ;

      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0,1,1) ; this->DrawUVW() ; sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

  // Get unique knot values for eVolumeParam direction - 
  ULONG jj, kk ;
  ULONG            sMData[256] ;
  SmTArray<ULONG>  sMults(256,sMData) ;
  SER(GetKnots(eVolumeParam, rParams, &sMults)) ;

  /// knot count
  ULONG lKnotCnt = rParams.GetSize() ;

  // check state - nonDegenerate knot vectors must have 2 or more knots
  SER((lKnotCnt <= 1) ? SM_ERR : SM_SUCCESS) ;

  // locals
  ULONG lDegree = GetDegree(eVolumeParam) ;

  // low work - no geometric tests
  if(bCalcGeometric == FALSE)
    {
      // set end continuties
      // compute conts at each internal knot
      for(ULONG ii=0; ii<lKnotCnt-1; ii++)
        {
          // size rConts array
          rConts.SetSize(lKnotCnt) ;

          // save representational continuity at knot
          ULONG lCont = (lDegree >= sMults[ii]) ? lDegree - sMults[ii] : 0 ;
          rConts[ii] =   lCont == 0 ? SM_CT_C0
                       : lCont == 1 ? SM_CT_C1
                       : lCont == 2 ? SM_CT_C1_C2
                       : lCont == 3 ? SM_CT_C1_C2_C3
                       :              SM_CT_CINFINITY ;

          // save min continuity
          if(reMinContinuity > rConts[ii]) { reMinContinuity = rConts[ii] ; }

        } // end iter ii, every unique knot value 

      // all done
      return(SM_SUCCESS) ;

    } // end low work - no geometric tests

  // arrive here when asked to test geometric continuity at every knot value
  // method - for every knot value - sample left and right values at an
  //          array of sample points on the associated isoparameter plane
  //          and report lowest sampled continuity.  This is OK because
  //          BSplines are biecewise polynomials and sampling will detect
  //          discontinuities accurately.
  // rParams = array of knot values along the eVolumeParam direction used
  //           to defined the isoParameter values of the tested IsoParameterPlanes 

  // get knots in other directions - those knot spacings determine the point sampling locations

  // GetKnot locals
  double                     adData1[256], adData2[256], aidData[256] ;
  ULONG                      alData[256];
  SmContinuityType           aeData[256];

  SmVolumeParamsType eConstantParams ;
  SmTArray<double>           sKnots1(256,adData1), sKnots2(256,adData2), sIsoKnots(256,aidData) ;
  SmTArray<ULONG>            sKnotMultiplicities(256,alData);
  SmTArray<SmContinuityType> sTmpConts(256,aeData);
  SmContinuityType eTmpMinCont;

  // Get the knots going in the other (u/v/w) directions.
  if     (eVolumeParam == SM_VP_U) { eConstantParams = SM_VPS_VW ;
                                     SER(GetKnots(SM_VP_V, sKnots1)) ;
                                     SER(GetKnots(SM_VP_W, sKnots2)) ;
                                   }                                
  else if(eVolumeParam == SM_VP_V) { eConstantParams = SM_VPS_UW ;  
                                     SER(GetKnots(SM_VP_U, sKnots1)) ;
                                     SER(GetKnots(SM_VP_W, sKnots2)) ;
                                   }                                
  else                             { eConstantParams = SM_VPS_UV ;  
                                     SER(GetKnots(SM_VP_U, sKnots1)) ;
                                     SER(GetKnots(SM_VP_V, sKnots2)) ;
                                   }
  // add midKnot to singleSpan KnotVectors to avoid degenerate computations
  //   for span volumeswith singularities at each end.
  if ( sKnots1.GetSize() == 2 ) { sKnots1.SetSize(3);
                                  sKnots1[2] = sKnots1[1] ;
                                  sKnots1[1] = 0.5*(sKnots1[0]+sKnots1[2]);
                                }

  if ( sKnots2.GetSize() == 2 ) { sKnots2.SetSize(3);
                                  sKnots2[2] = sKnots2[1] ;
                                  sKnots2[1] = 0.5*(sKnots2[0]+sKnots2[2]);
                                }

  // reduce large knot counts to 5 sample points (don't need more)
  if (sKnots1.GetSize() > 5)    { sKnots1[4] = sKnots1.GetLast();
                                  sKnots1[3] = 0.155 * sKnots1[0] + 0.845 * sKnots1[4] ;
                                  sKnots1[2] = 0.5   * sKnots1[0] + 0.5   * sKnots1[4] ;
                                  sKnots1[1] = 0.655 * sKnots1[0] + 0.345 * sKnots1[4] ;
                                  sKnots1.SetSize(5);
                                }

  if (sKnots2.GetSize() > 5)    { sKnots2[4] = sKnots2.GetLast();
                                  sKnots2[3] = 0.155 * sKnots2[0] + 0.845 * sKnots2[4] ; 
                                  sKnots2[2] = 0.5   * sKnots2[0] + 0.5   * sKnots2[4] ; 
                                  sKnots2[1] = 0.655 * sKnots2[0] + 0.345 * sKnots2[4] ; 
                                  sKnots2.SetSize(5);
                                }

  // mark working with the first IsoCurve to classify
  SmBoolean bFirst = TRUE ;

  // next: build an array of IsoCurves in the given test direction and check their continuities at the knots

  // for every knot value to sample
  for(ULONG ii=0;ii<sKnots1.GetSize(); ii++)
    {
      double dParameter1 = sKnots1[ii];

      for(jj=0;jj<sKnots2.GetSize();jj++)
        {
          double dParameter2 = sKnots2[ii];

          // make temp isoParameter curve for this knot sample value
          SmCurve *pIsoCrv = NULL ;
          SER(EvaluateIsoParametricCurveSimple(*GetContext(),eConstantParams,dParameter1,dParameter2,0.0,pIsoCrv));
          pIsoCrv->GetKnots(sIsoKnots) ;
          SmObjDelete sCleanup(pIsoCrv);

#ifdef SM_DEBUG_CODE
      // draw
      if(bDebugMe)
        {
          SM_DUMP_AND_ASSERT_VALID(this) ;
          SM_DUMP_AND_ASSERT_VALID(pIsoCrv) ;

          smgfx_Erase() ;
          smgfx_SetLook(1,2, 0,1,1) ; this->DrawUVW() ; sm_GraphicsLoop() ;
          smgfx_SetLook( 3, 4, 1, 0, 0 ); if(pIsoCrv) { pIsoCrv->Draw(); sm_GraphicsLoop(); }
          sm_GraphicsLoop() ;
        }
#endif // SM_DEBUG_CODE

          // When curve is degenerate - try another knot sample value
          if (pIsoCrv->IsDegenerate())
            {
              // If we have not been able to compute a continuity and we are
              // past the first knot.  Then take a middle value between the knots
              // and try it.
              if ((ii>0 && jj>0) && bFirst)
                {
                  sCleanup.Clear();
                  SM_ASSERT(pIsoCrv != NULL) ; delete pIsoCrv ; pIsoCrv = NULL ;
                  dParameter1 = (sKnots1[ii-1] + sKnots1[ii]) / 2.0;
                  dParameter2 = (sKnots2[ii-1] + sKnots2[ii]) / 2.0;
                  SER(EvaluateIsoParametricCurveSimple(*GetContext(),eConstantParams,dParameter1,dParameter2,0.0,pIsoCrv));
                  sCleanup.SetObj(pIsoCrv);
                }
              if (pIsoCrv->IsDegenerate())
                {
                  continue;
                }
            } // end curve is degenerate check

          // calculate the iso-curve continuities
          SER(pIsoCrv->CalculateContinuities(eTmpMinCont,sTmpConts));

          // if this is the first curve to classify
          if (bFirst)
            {
              bFirst = FALSE ;

              // store computed values
              rConts.ReSet() ;
              rConts.Append(sTmpConts) ; 
              reMinContinuity = eTmpMinCont;
            }
          else // other iso-curves have been classified branch
            {
              // save minimum continuity values
              if(eTmpMinCont < reMinContinuity)
                { reMinContinuity = eTmpMinCont; }

              // for every knot - save min continuity seen
              for(kk=0; kk<sTmpConts.GetSize(); kk++)
                {
                  if (sTmpConts[kk] < rConts[kk])
                    { rConts[kk] = sTmpConts[kk] ; }

                } // end iter every knot value to store min continuity seen

            } // end other iso-curves have been classified branch
        } // end iter jj, every [ii, jj] knot sample, bulding iso-parameter curves
    } // end iter ii

  // when none of the curves classify - something is wrong.
  // For now just set a default continuity output
  if(bFirst)
    {
      // size rConts array
      rConts.SetSize(lKnotCnt) ;

      // init rConts array to [Discontinuous, G1, ... G1, Discontinuous]
      rConts[0]          = SM_CT_DISCONTINUOUS ;
      rConts[lKnotCnt-1] = SM_CT_DISCONTINUOUS ;
      for(ULONG ii=1; ii+1<rParams.GetSize(); ii++) { rConts.SetAt(ii, SM_CT_G1); }

      // inform the public
      SE_MSG(SM_ERR,_T("SmBSplineVolume::CalculateContinuitiesSimple Geometric classification failed - returning default continuity array")) ;
    
    } // end no iso-curves classified check

  // all done
  return SM_SUCCESS;

} // end SmBSplineVolume::CalculateContinuitiesSimple

// gwc: a function to consider adding later on
//      /*******************************************************************//**
//      PURPOSE: Add a set of knots into one direction of a volume.  
//      
//      NOTES: Note that the knot vector
//         should not contain duplicates of existing knots unless you want to
//         increase the multiplicity of the knots.  This means that you should not
//         put the first and last knot values of the original curve into the array.
//      ***********************************************************************/
//      SmStatus SmBSplineVolume::InsertKnots
//        (SmVolumeParamType   eVolumeParam,    // in : oneof: SM_VP_U, SM_VP_V, SM_VP_W
//         SmTArray<double>  & rNewKnots)       // in : array of new knots to add
//      {
//        // locals
//        KNOTVECTOR knt;
//        STACKS SG;
//        SmNLibStackHandler sSH(&SG);
//        VOLUME vol;
//        N_VolumeInitArrays(&vol);
//      
//        NL_FLAG   dir =   eVolumeParam == SM_VP_U ? NL_UDIR
//                     : eVolumeParam == SM_VP_V ? NL_VDIR
//                     :                           NL_WDIR ;
//      
//        VOLUME * volP = GetOrCreateGwNurbPointer();
//        NER(volP);
//      
//        knt.n = (NL_INDEX)rNewKnots.GetSize() - 1;
//        knt.U = (NL_REAL*)rNewKnots.GetDataArray();
//      
//        // pass the call along 
//        // gwc: function needs to be extended and long names
//        //      given to N_CrvRefine, N_SrfInsertKnots, and the new N_toovrf
//        NL_ERR(N_SrfInsertKnots(volP,&knt,dir,&vol,&SG,&SG));
//      
//        const SmContext * pContext = GetContext();
//        NER(pContext);
//        SmBSplineVolume *pTemp = new (*pContext) SmBSplineVolume((gw_VOLUME *)&vol);
//        SmObjDelete sClean(pTemp);
//      
//        gw_VOLUME * pNewVol = pTemp->GetOrCreateGwNurbPointer();
//        pTemp->m_pNurb       = volP ;
//        m_pNurb              = pNewVol;
//      
//      #ifdef SM_DEBUG_CODE
//      SmBoolean bDebugMe = FALSE;
//        if (bDebugMe) 
//          {
//            sm_GraphicsLoop();
//            smgfx_SetColor(0,0,1);
//            DrawUV(1,1);
//            Dump();
//          }
//      #endif
//      
//        return SM_SUCCESS;
//      
//      } // end SmBSplineVolume::InsertKnots


// gwc: a function to consider adding later on
//      /*******************************************************************//**
//      PURPOSE: Insert knots into this curve such that the maximum distance
//          between the points on the curve is dDistance.
//      
//      NOTES: 
//      ***********************************************************************/
//      SmStatus SmBSplineVolume::InsertKnotsByDistance
//        (SmVolumeParamType eVolumeParam,  // in : oneof: SM_VP_U, SM_VP_V, SM_VP_W
//         double dDistance)                // in : Max allowed 3d distance between knots
//      {
//        double dData[256];
//        SmTArray<double> sKnots(256,dData);
//        double dData2[256];
//        SmTArray<double> sNewKnots(256,dData2);
//        SmExtent3d sDomain = GetNaturalParamDomain();
//        GetKnots(eVolumeParam,sKnots);
//        for (ULONG j=1; j<sKnots.GetSize(); j++) 
//          {
//            SmExtent1d sIvl(sKnots[j-1],sKnots[j]);
//            if (sIvl.GetLength() < SM_EFF_ZERO_SQRT) continue;
//            double dKnotDist;
//            SER(CalculateApproxIsoCurveDistance(eVolumeParam,sDomain,sIvl,5,4,dKnotDist));
//            ULONG lNumSubdivides = (ULONG)(dKnotDist / dDistance);
//            for (ULONG k=0; k<lNumSubdivides; k++) 
//              {
//                double dT = (k+1.0) / (lNumSubdivides+1.0);
//                sNewKnots.Add(sIvl.Evaluate(dT));
//              }
//          }
//        SER(InsertKnots(eVolumeParam,sNewKnots));
//        return SM_SUCCESS;
//      
//      } // end SmBSplineVolume::InsertKnotsByDistance

// gwc: a function to consider adding later on
//      /*******************************************************************//**
//      PURPOSE: Refine a NURBS volume with a given knot vector in either
//          u-, v-, or w-direction. 
//      
//      NOTES: It is  assumed that the new knot vector fits
//          into the old  ones.
//          
//            This method is only available for users with NLib
//          
//      ***********************************************************************/
//      SmStatus SmBSplineVolume::RefineVolume
//        (const SmTArray<double> & crNewKnots,     // in : 
//         SmVolumeParamType        eVolumeParam)   // in : oneof: SM_VP_U, SM_VP_V, SM_VP_W
//      {
//        // locals
//        STACKS SG;
//        SmNLibStackHandler sSH(&SG);
//        VOLUME vol;
//        N_VolumeInitArrays(&vol);
//      
//        KNOTVECTOR  *knt;
//        NL_INDEX       n = crNewKnots.GetSize()-1;
//        knt = N_AllocKnotVectorAndArray(n,&SG);
//        NL_REAL *U = knt->U;
//        for (ULONG i=0; i<crNewKnots.GetSize(); i++) 
//          {
//            U[i] = crNewKnots[i];
//          }
//      
//        NL_FLAG   dir =   eVolumeParam == SM_VP_U ? NL_UDIR
//                     : eVolumeParam == SM_VP_V ? NL_VDIR
//                     :                           NL_WDIR ;
//      
//        VOLUME * volP = GetOrCreateGwNurbPointer();
//        NER(volP);
//      
//        NL_ERR(N_SrfInsertKnots(volP,knt,dir,&vol,&SG,&SG));
//      
//        const SmContext * pContext = GetContext();
//        NER(pContext);
//      
//        SmBSplineVolume *pTemp = new (*pContext) SmBSplineVolume((gw_VOLUME *)&vol);
//        SmObjDelete sClean(pTemp);
//      
//        gw_VOLUME * pNewVol = pTemp->GetOrCreateGwNurbPointer();
//        pTemp->m_pNurb       = volP ;
//        m_pNurb              = pNewVol;
//      #ifdef SM_DEBUG_CODE
//      SmBoolean bDebugMe = FALSE;
//        if (bDebugMe) 
//          {
//            sm_GraphicsLoop();
//            smgfx_SetColor(0,0,1);
//            DrawUV(1,1);
//            Dump();
//          }
//      #endif
//      
//        return SM_SUCCESS;
//      
//      } // end SmBSplineVolume::RefineVolume


// gwc: a function to consider adding later on
//      /*******************************************************************//**
//      PURPOSE: Remove all removable knots from a NURBS volume with optional
//          constraints on some of the knots that are not to be removed.
//      
//      NOTES: This method is only available for users with NLib
//          
//      ***********************************************************************/
//      SmStatus SmBSplineVolume::RemoveKnots
//        (double dThisApproxTol3d,
//         SmBoolean bRemoveUKnots,
//         SmBoolean bRemoveVKnots,
//         SmBoolean bRemoveWKnots,
//         const SmTArray<double> * cpOptKeptUKnots,  // u-knots that will NOT be removed
//         const SmTArray<double> * cpOptKeptVKnots,  // u-knots that will NOT be removed
//         const SmTArray<double> * cpOptKeptwKnots)  // u-knots that will NOT be removed
//      {
//        // locals
//        STACKS SG;
//        SmNLibStackHandler sSH(&SG);
//        VOLUME vol;
//        N_VolumeInitArrays(&vol);
//      
//        // set call arguments
//        NL_FLAG dir =   (bRemoveUKnots ? NL_UDIR : 0)
//                   + (bRemoveUKnots ? NL_VDIR : 0)
//                   + (bRemoveUKnots ? NL_WDIR : 0) ;
//        VOLUME * volP = GetOrCreateGwNurbPointer();
//        NER(volP);
//      
//        if (cpOptKeptUKnots || cpOptKeptVKnots || cpOptKeptWKnots) 
//          {
//            NL_PARAMETER  *UK = NULL, *VK = NULL, *WK = NULL;
//            NL_INDEX      mu = 0, mv = 0, mw = 0;
//            if (cpOptKeptUKnots) { ULONG lCount = cpOptKeptUKnots->GetSize();
//                                   mu = lCount-1;
//                                   UK = N_AllocReal1dArray(mu,&SG);
//                                   for (ULONG iii=0; iii<lCount; iii++) 
//                                     {
//                                       UK[iii] = (*cpOptKeptUKnots)[iii];
//                                     }
//                                 }
//            if (cpOptKeptVKnots) { ULONG lCount = cpOptKeptVKnots->GetSize();
//                                   mv = lCount-1;
//                                   VK = N_AllocReal1dArray(mv,&SG);
//                                   for (ULONG jjj=0; jjj<lCount; jjj++) 
//                                     {
//                                       VK[jjj] = (*cpOptKeptVKnots)[jjj];
//                                     }
//                                 }
//            if (cpOptKeptWKnots) { ULONG lCount = cpOptKeptWKnots->GetSize();
//                                   mw = lCount-1;
//                                   WK = N_AllocReal1dArray(mw,&SG);
//                                   for (ULONG kkk=0; kkk<lCount; kkk++) 
//                                     {
//                                       WK[kkk] = (*cpOptKeptWKnots)[kkk];
//                                     }
//                                 }
//            // pass the call along
//            // gwc: extend function to 3d and rename N_SrfRemoveKnotsConstraints and N_toocrk with long names
//            NL_ERR(N_SrfRemoveKnotsConstraints(volP,UK,mu,VK,mv,dThisApproxTol3d,dir,&vol,&SG));
//          }
//        else 
//          {
//            // pass the call along
//            // gwc: extend function to 3d and rename N_SrfRemoveAllKnots and N_CrvRemoveKnots with long names
//            NL_ERR(N_SrfRemoveAllKnots(volP,dThisApproxTol3d,dir,&vol,&SG));
//          }
//      
//        // save result in this SmBSplineVolume
//        const SmContext * pContext = GetContext();
//        NER(pContext);
//        SmBSplineVolume *pTemp = new (*pContext) SmBSplineVolume((gw_VOLUME *)&vol);
//        SmObjDelete sClean(pTemp);
//      
//        gw_VOLUME * pNewVol = pTemp->GetOrCreateGwNurbPointer();
//        pTemp->m_pNurb       = volP ;
//        m_pNurb              = pNewVol;
//      
//      #ifdef SM_DEBUG_CODE
//      SmBoolean bDebugMe = FALSE;
//        if (bDebugMe) 
//          {
//            sm_GraphicsLoop();
//            smgfx_SetColor(0,0,1);
//            DrawUV(1,1);
//            Dump();
//          }
//      #endif
//      
//        // all done
//        return SM_SUCCESS;
//      
//      } // end SmBSplineVolume::RemoveKnots

//      /*******************************************************************//**
//      PURPOSE: Remove one knot multiple times from a NURBS volume either
//          in u-, v- or w-direction.
//      
//      NOTES:  The knot must be an interior  knot.
//      
//          This method is only available for users with NLib
//          
//      ***********************************************************************/
//      SmStatus SmBSplineVolume::RemoveOneKnot
//        (double              dKnot,               // in : 
//         ULONG               lNumKnotsRemoval,    // in : 
//         SmVolumeParamType   eVolumeParam,        // in : oneof: SM_VP_U, SM_VP_V, SM_VP_W
//         double              dThisApproxTol3d,   // in : 
//         ULONG             & rlNumKnotsRemoved)   // out: 
//      {
//        STACKS SG;
//        SmNLibStackHandler sSH(&SG);
//        VOLUME vol;
//        N_VolumeInitArrays(&vol);
//      
//        NL_FLAG   dir =   eVolumeParam == SM_VP_U ? NL_UDIR
//                     : eVolumeParam == SM_VP_V ? NL_VDIR
//                     :                           NL_WDIR ;
//      
//        VOLUME * volP = GetOrCreateGwNurbPointer();
//        NER(volP);
//      
//        NL_INDEX  nt = lNumKnotsRemoval;
//        NL_INDEX  rt;
//      
//        // pass the call along
//        // gwc: a function to consider adding later on
//        //      extend N_SrfRemoveKnotConditional to 3d and rename N_SrfRemoveKnotConditional and N_CrvRemoveKnot to long names.
//        NL_ERR(N_SrfRemoveKnotConditional(volP,dKnot,nt,dThisApproxTol3d,dir,&rt,&vol,&SG));
//        rlNumKnotsRemoved = rt;
//      
//        const SmContext * pContext = GetContext();
//        NER(pContext);
//        SmBSplineVolume *pTemp = new (*pContext) SmBSplineVolume((gw_VOLUME *)&vol);
//        SmObjDelete sClean(pTemp);
//      
//        gw_VOLUME * pNewVol = pTemp->GetOrCreateGwNurbPointer();
//        pTemp->m_pNurb       = volP ;
//        m_pNurb              = pNewVol;
//      
//      #ifdef SM_DEBUG_CODE
//      SmBoolean bDebugMe = FALSE;
//        if (bDebugMe) 
//          {
//            sm_GraphicsLoop();
//            smgfx_SetColor(0,0,1);
//            DrawUV(1,1);
//            Dump();
//          }
//      #endif
//      
//        return SM_SUCCESS;
//      
//      } // end SmBSplineVolume::RemoveOneKnot

/*******************************************************************//**
PURPOSE: Reparameterize the domain of a B-Spline Volume

NOTES: This method is only available for users with NLib
    
***********************************************************************/
SmStatus SmBSplineVolume::Reparameterize
  (const SmExtent3d & crTrimDomain)       // in : new parameter range for BSPlineVolume
{
#ifdef USE_VOLUMES
    Notify(SM_NO_PRE_EDIT, this, SM_NO_GET_OWNER(this), NULL);

    // set N_VolumeReparam arguments
    VOLUME* volP = GetOrCreateGwNurbPointer();
    NER(volP);

    // pass the call along
    N_VolumeReparam(volP, crTrimDomain.GetMin().x,
        crTrimDomain.GetMax().x,
        crTrimDomain.GetMin().y,
        crTrimDomain.GetMax().y,
        crTrimDomain.GetMin().z,
        crTrimDomain.GetMax().z);
    Notify(SM_NO_POST_EDIT, this, SM_NO_GET_OWNER(this), NULL);

    // all done
    SM_DUMP_AND_ASSERT2_VALID(this);
    return SM_SUCCESS;
#else
    SM_REF1(crTrimDomain);
    return SM_ERR;
#endif // USE_VOLUMES

} // end SmBSplineVolume::Reparameterize

// gwc: a function to consider adding later on
//      /*******************************************************************//**
//      PURPOSE: Reparameterize the domain of a B-Spline Volume
//      
//      NOTES: This method is only available for users with NLib
//          
//      ***********************************************************************/
//      SmStatus SmBSplineVolume::ReparametrizeWithArcLength
//        ()
//      {
//        // locals
//        STACKS SG;
//        SmNLibStackHandler sSH(&SG);
//        VOLUME vol;
//        N_VolumeInitArrays(&vol);
//        Notify(SM_NO_PRE_EDIT, this, NULL);
//      
//        // set call arguments
//        VOLUME * volP = GetOrCreateGwNurbPointer();
//        NER(volP);
//        NL_REAL tol = 0.01;  //Relative tolerance 
//      
//        // pass call along
//        // gwc: could extend N_grpsap to 3d and rename N_SrfReparamArcLength and N_SrfReparamMultKnots to long names
//        NL_ERR(N_SrfReparamArcLength(volP,tol,&vol,&SG)); 
//      
//        // save modified volume in this SmBSplineVolume
//        const SmContext * pContext = GetContext();
//        NER(pContext);
//        SmBSplineVolume *pTemp = new (*pContext) SmBSplineVolume((gw_VOLUME *)&vol);
//        SmObjDelete sClean(pTemp);
//      
//        gw_VOLUME * pNewVol = pTemp->GetOrCreateGwNurbPointer();
//        pTemp->m_pNurb       = volP ;
//        m_pNurb              = pNewVol;
//        Notify(SM_NO_POST_EDIT, this, NULL);
//      
//      #ifdef SM_DEBUG_CODE
//      SmBoolean bDebugMe = FALSE;
//        if (bDebugMe) {
//            sm_GraphicsLoop();
//            smgfx_SetColor(0,0,1);
//            DrawUV(1,1);
//            Dump();
//        }
//      #endif
//        
//        // give analytics a chance to update their nested STEP data
//        RebuildSTEPFromNURBParameters() ;
//      
//        // all done
//        SM_DUMP_AND_ASSERT2_VALID(this) ;
//        return SM_SUCCESS;
//      
//      } // end SmBSplineVolume::ReparametrizeWithArcLength

/*******************************************************************//**
PURPOSE: Test volume for degenerate control points and return location

NOTES: This method is only available for users with NLib

METHOD --- search all control Points to see if any share a duplicate location
           within tolerance.  When one is found set Nu, Nv, Nw to the
           index of the first such occurance and return
           SM_ERR, else return SM_SUCCESS.
    
***********************************************************************/
SmStatus SmBSplineVolume::TestDegenerate
  (double dTol,        // in : Min 3d distance between distinct ControlPoints
   ULONG  &rNu,        // out: Index of 1st ControlPoint with a duplicate or 0
   ULONG  &rNv,        // out: Index of 1st ControlPoint with a duplicate or 0
   ULONG  &rNw)        // out: Index of 1st ControlPoint with a duplicate or 0
  const
{
#ifdef USE_VOLUMES
    // locals
    VOLUME* volP = ((SmBSplineVolume*)this)->GetOrCreateGwNurbPointer();
    NER(volP);
    NL_INDEX nu = 0, nv = 0, nw = 0;

    // pass the call along
    NL_FLAG fRtn = N_VolumeHasEqualCPts(volP, dTol, &nu, &nv, &nw);

    // set output
    rNu = nu;
    rNv = nv;
    rNw = nw;

    // all done
    return fRtn == 0 ? SM_SUCCESS : SM_ERR;
#else
    SM_REF4(dTol, rNu, rNv, rNw);
    return SM_ERR;
#endif // USE_VOLUMES

} // end SmBSplineVolume::TestDegenerate

/*******************************************************************//**
PURPOSE: Test Volume corners for (du x dv) . dw = 0.0 (no volume)

NOTES:  Tol is min volume of (du x dv) . dw (not normalised)
     May be a pole, a cusp,  or du->, dv->, or dw-> may be co-linear.
     This method is only available for users with NLib
            
***********************************************************************/
SmStatus SmBSplineVolume::TestCornerVolumes
  (double dTol)     // in : ScaledZero for length of a vector
                    //      degenerate when Volume <= dTol*dTol*dTol
 const
{   
  SmPoint3d sUVW;
  SmPoint3d sPnt;
  SmVector3d sDU, sDV, sDW;
  double S, Tol3;
  SmExtent3d sUVDomain = GetNaturalParamDomain();

  Tol3 = dTol*dTol*dTol;

  // Evaluate derivatives at all uv corners

  // first for W = 0.0
  sUVW = sUVDomain.Evaluate(0.0,0.0,0.0);
  SER(Evaluate1stDerivatives(sUVW,TRUE,TRUE,TRUE,sPnt,sDU,sDV,sDW));
  S = sDU.TripleProduct(sDV, sDW) ;    
  if (S < Tol3) SER(SM_ERR_WARNING);

  sUVW = sUVDomain.Evaluate(0.0,1.0,0.0);    
  SER(Evaluate1stDerivatives(sUVW,TRUE,TRUE,TRUE,sPnt,sDU,sDV,sDW));
  S = sDU.TripleProduct(sDV, sDW) ;    
  if (S < Tol3) SER(SM_ERR_WARNING);

  sUVW = sUVDomain.Evaluate(1.0,1.0,0.0);    
  SER(Evaluate1stDerivatives(sUVW,TRUE,TRUE,TRUE,sPnt,sDU,sDV,sDW));
  S = sDU.TripleProduct(sDV, sDW) ;    
  if (S < Tol3) SER(SM_ERR_WARNING);

  sUVW = sUVDomain.Evaluate(1.0,0.0,0.0);    
  SER(Evaluate1stDerivatives(sUVW,TRUE,TRUE,TRUE,sPnt,sDU,sDV,sDW));
  S = sDU.TripleProduct(sDV, sDW) ;    
  if (S < Tol3) SER(SM_ERR_WARNING);

  // now for W = 1.0
  sUVW = sUVDomain.Evaluate(0.0,0.0,1.0);
  SER(Evaluate1stDerivatives(sUVW,TRUE,TRUE,TRUE,sPnt,sDU,sDV,sDW));
  S = sDU.TripleProduct(sDV, sDW) ;    
  if (S < Tol3) SER(SM_ERR_WARNING);

  sUVW = sUVDomain.Evaluate(0.0,1.0,1.0);    
  SER(Evaluate1stDerivatives(sUVW,TRUE,TRUE,TRUE,sPnt,sDU,sDV,sDW));
  S = sDU.TripleProduct(sDV, sDW) ;    
  if (S < Tol3) SER(SM_ERR_WARNING);

  sUVW = sUVDomain.Evaluate(1.0,1.0,1.0);    
  SER(Evaluate1stDerivatives(sUVW,TRUE,TRUE,TRUE,sPnt,sDU,sDV,sDW));
  S = sDU.TripleProduct(sDV, sDW) ;    
  if (S < Tol3) SER(SM_ERR_WARNING);

  sUVW = sUVDomain.Evaluate(1.0,0.0,1.0);    
  SER(Evaluate1stDerivatives(sUVW,TRUE,TRUE,TRUE,sPnt,sDU,sDV,sDW));
  S = sDU.TripleProduct(sDV, sDW) ;    
  if (S < Tol3) SER(SM_ERR_WARNING);

  // all done
  return SM_SUCCESS;

} // end SmBSplineVolume::TestCornerVolumes

//      // gwc: a function to consider adding later on
//      /*******************************************************************//**
//      PURPOSE: Trim a B-Spline volume to correspond to a subset
//          of the original volume trimmed to a square UV boundary.  
//          
//          The volume's natural UVW Domain is modified to equal the input
//          trim boundary domain.
//      
//      NOTES:
//        1. The volume's natural UVW Domain is modified to be near the input
//           trim boundary domain.
//        2. The trimBoundary may be snapped to existing knot values.
//        3. The input trimBoundary domain is set to equal the volume's
//           new natural domain on exit.
//      
//      METHOD
//        1. Trim boundaries close to existing knots are snapped to 
//           (or away from) existing knot values to prevent the creation of 
//           short (as measured in parameter space) spans.
//        1a. The input rTrimDomain is modified if any trim boundary is snapped.
//      
//        2. A new volume is created from the old volume whose natural domain
//           is equal to the requested trim boundary by inserting knots
//           at the trim values.  Knots and Control Points outside
//           of the requested trim boundary are removed from the new volume.
//        
//        3. The SmBSPlineVolume->m_pNurb pointers of this SmBSplineVolume
//           and the new one are swapped making the trimmed volume the
//           official definition. 
//        
//        4. The new volume with the old m_pNurb pointer is deleted when 
//           exiting the function's scope.  
//         
//        The definition of the NURB volume pointer under this SMBSplineVolume
//        is modified.
//        
//         
//      ***********************************************************************/
//      SmStatus SmBSplineVolume::TrimWithDomain
//        (SmExtent3d & rTrimDomain)       // i/o: requested volume new natural domain
//                                         //      set to modified volume's natural domain.
//                                         //      Input trim values may be snapped
//                                         //      to (or away) from existing knot values
//                                         //      to prevent the creation of short spans.
//      {
//        // get current natural boundary
//        SmExtent3d sNaturalDomain = GetNaturalParamDomain();
//      
//        // check input - trim domain not contained within natural domain (no tolerance allowed)
//        if (!rTrimDomain.IsContainedBy(sNaturalDomain)) 
//          { SER(SM_ERR); }
//      
//        // no work - natural domain is same size as trim domain
//        if (sNaturalDomain.IsContainedBy(rTrimDomain)) 
//          { return SM_SUCCESS; }
//        
//        STACKS     SG;
//        SmNLibStackHandler sSH(&SG);
//      
//        // init NLIb style volume (no internal ControlPoint, Weight, or knot memory)
//        VOLUME sur1;
//        N_VolumeInitArrays(&sur1);
//      
//        // get trim domain corner points
//        NL_REAL ul = rTrimDomain.GetMin().x;
//        NL_REAL ur = rTrimDomain.GetMax().x;
//        NL_REAL vl = rTrimDomain.GetMin().y;
//        NL_REAL vr = rTrimDomain.GetMax().y;
//        NL_REAL wl = rTrimDomain.GetMin().z;
//        NL_REAL wr = rTrimDomain.GetMax().z;
//      
//        // don't build too small spans in u direction. 
//        // snap U trim boundaries to existing U knots or
//        //   away from U knots to prevent making a span less than 10*tolerance
//        {
//          // scale tolerances to domain size
//          double dTolMin = SM_EFF_ZERO_SQRT*(1.0+smos_Fabs(ul));
//          double dTolMax = SM_EFF_ZERO_SQRT*(1.0+smos_Fabs(ur));
//          
//          // get current U knots
//          SmTArray<double> sKnots;
//          GetKnots(SM_VP_U,sKnots);
//      
//          // for every U knot - looking to snap trim boundaries
//          for (ULONG i=0; i<sKnots.GetSize(); i++) 
//            {
//              double dKnot = sKnots[i];
//      
//              // when lower trim boundary is close to knot
//              if (smos_Fabs(dKnot - ul) < dTolMin) 
//                {
//                  // when trim boundary is slightly larger than knot - set trim boundary to knot
//                  if (ul >= dKnot || i == 0) 
//                    {
//                      ul = dKnot;
//                    }
//                  else // space trim boundary more than tol away from knot
//                    { // Set UL to either the previous knot or slightly less than current knot
//                      ul = dKnot - 10.0 * dTolMin;
//                      if (ul < sKnots[i-1]) 
//                        { ul = sKnots[i-1];
//                        }
//                    }
//                } // end this knot close to lower trim boundary check
//      
//              // when upper trim boundary is close to knot
//              if (smos_Fabs(dKnot - ur) < dTolMax) 
//                {
//                  if (ur <= dKnot || i==sKnots.GetSize()-1) 
//                    { ur = dKnot; }
//                  else 
//                    {
//                      ur = dKnot + 10.0 * dTolMax;
//                      if (ur > sKnots[i+1]) 
//                        { ur = sKnots[i+1];
//                        }
//                    }
//                }  // end this knot close to upper trim boundary check
//            } // end iter every U knot
//          
//          // no work - snapped trim boundaries are within tolerance of one another
//          if (smos_Fabs(ur - ul) < dTolMin + dTolMax) 
//            {
//              // Error here because trim points are too close together
//              return SM_ERR_INVALID_INPUT;
//            }
//      
//        } // end filtering new small U span widths
//      
//        // don't build too small spans in V direction. 
//        // snap V trim boundaries to existing V knots or
//        //   away from V knots to prevent making a span less than 10*tolerance
//        {
//          double dTolMin = SM_EFF_ZERO_SQRT*(1.0+smos_Fabs(vl));
//          double dTolMax = SM_EFF_ZERO_SQRT*(1.0+smos_Fabs(vr));
//          
//          // for every V knot - look to snap V trim boundaries
//          SmTArray<double> sKnots;
//          GetKnots(SM_VP_V,sKnots);
//          for (ULONG i=0; i<sKnots.GetSize(); i++) 
//            {
//              double dKnot = sKnots[i];
//      
//              // snap lower V Trim boundaries as appropriate
//              if (smos_Fabs(dKnot - vl) < dTolMin) 
//                {
//                  if (vl >= dKnot || i==0)
//                    { vl = dKnot; }
//                  else 
//                    {
//                      vl = dKnot - 10.0 * dTolMin;
//                      if (vl < sKnots[i-1]) 
//                        {  vl = sKnots[i-1];
//                        }
//                    }
//                } // end lower V Trim boundary near knot value check
//      
//              // snap upper V Trim Boundaries as appropriate
//              if (smos_Fabs(dKnot - vr) < dTolMax) 
//                {
//                  if (vr <= dKnot || i==sKnots.GetSize()-1) 
//                    { vr = dKnot; }
//                  else 
//                    {
//                      vr = dKnot + 10.0 * dTolMax;
//                      if (vr > sKnots[i+1]) 
//                        { vr = sKnots[i+1]; }
//                    }
//                } // end upper V Trim boundary near knot value check
//            } // end iter every V knot - looking to snap trim boundaries
//          
//          // no work - snapped trim boundaries are within tolerance of one another
//          if (smos_Fabs(vr - vl) < dTolMin + dTolMax) 
//            {
//              // Error here because trim points are too close together
//              return SM_ERR_INVALID_INPUT;
//            }
//        }
//      
//          // don't build too small spans in W direction. 
//        // snap W trim boundaries to existing W knots or
//        //   away from W knots to prevent making a span less than 10*tolerance
//        {
//          double dTolMin = SM_EFF_ZERO_SQRT*(1.0+smos_Fabs(wl));
//          double dTolMax = SM_EFF_ZERO_SQRT*(1.0+smos_Fabs(wr));
//          
//          // for every W knot - look to snap W trim boundaries
//          SmTArray<double> sKnots;
//          GetKnots(SM_VP_W,sKnots);
//          for (ULONG i=0; i<sKnots.GetSize(); i++) 
//            {
//              double dKnot = sKnots[i];
//      
//              // snap lower W Trim boundaries as appropriate
//              if (smos_Fabs(dKnot - wl) < dTolMin) 
//                {
//                  if (wl >= dKnot || i==0)
//                    { wl = dKnot; }
//                  else 
//                    {
//                      wl = dKnot - 10.0 * dTolMin;
//                      if (wl < sKnots[i-1]) 
//                        {  wl = sKnots[i-1];
//                        }
//                    }
//                } // end lower W Trim boundary near knot value check
//      
//              // snap upper W Trim Boundaries as appropriate
//              if (smos_Fabs(dKnot - wr) < dTolMax) 
//                {
//                  if (wr <= dKnot || i==sKnots.GetSize()-1) 
//                    { wr = dKnot; }
//                  else 
//                    {
//                      wr = dKnot + 10.0 * dTolMax;
//                      if (wr > sKnots[i+1]) 
//                        { wr = sKnots[i+1]; }
//                    }
//                } // end upper W Trim boundary near knot value check
//            } // end iter every W knot - looking to snap trim boundaries
//          
//          // no work - snapped trim boundaries are within tolerance of one another
//          if (smos_Fabs(wr - wl) < dTolMin + dTolMax) 
//            {
//              // Error here because trim points are too close together
//              return SM_ERR_INVALID_INPUT;
//            }
//        }
//      
//        // place snapped Trim Boundary values into input TrimDomain
//        rTrimDomain.SetMinMax(SmPoint3d(ul,vl,wl),SmPoint3d(ur,vr,wr));
//      
//        // local volume pointer
//        VOLUME * volP = GetOrCreateGwNurbPointer();
//        NER(volP);
//      
//        // extract patch defined by trim boundaries from volP into sur1
//        // sur1 is a new volume. 
//        // gwc: could extend N_SrfExtractPatch to 3d and rename N_SrfExtractPatch and N_CrvExtractCrvSeg to long names
//        NL_ERR(N_SrfExtractPatch(volP,ul,ur,vl,vr,&sur1,&SG,&SG)); 
//      
//        // build SMLib volume from NLib sur1
//        SmBSplineVolume *pTemp = new (*GetContext()) SmBSplineVolume((gw_VOLUME *)&sur1);
//      
//        // delete pTemp volume when exiting scope
//        SmObjDelete sClean(pTemp);
//      
//        // swap the old and new BSPline->m_pNurb volume pointers. 
//        Notify(SM_NO_PRE_EDIT, this, NULL);
//        gw_VOLUME * pNewVol = pTemp->GetOrCreateGwNurbPointer();
//        pTemp->m_pNurb       = volP ;
//        m_pNurb              = pNewVol;
//        Notify(SM_NO_POST_EDIT, this, NULL);
//      
//        // all done
//        return SM_SUCCESS;
//      
//      } // end SmBSplineVolume::TrimWithDomain


// gwc: a function to consider adding later on
//      /*******************************************************************//**
//      PURPOSE: Output polygons for a volume to the given call back function.
//      
//      NOTES: Note that right now it only outputs triangles.
//                  USES NLib trimmed volume tessellator
//      ***********************************************************************/
//      SmStatus SmBSplineVolume::OutputPolygons
//        (double dVolumeChordHeightTolerance,       // in : max distance between polygon and volume
//         double dCurveChordHeightTolerance,        // in : mas distance between polygon edge and volume
//         SmBoolean bReverseNormals,                // in : TRUE = Reverse polygon normals prior to display
//                                                   //      FALSE= don't
//         SmPolygonOutputCallback & rPolygonOutput) // in : Object used to actually output the triangles
//         const
//      {
//        const SmBSplineVolume *pSrf = this;
//        const SmContext *pContext = pSrf->GetContext();
//          
//        CURVE ***cuo;
//        CURVE ****cui;
//        VOLUME *vol;
//        NL_INDEX n = 0;
//        NL_INDEX ho[1];
//        NL_INDEX hi[1];
//        NL_INDEX **hs;
//        NL_REAL epc = dCurveChordHeightTolerance;
//        NL_REAL eps = dVolumeChordHeightTolerance;
//        NL_REAL tol = 1.0e-7;
//        double dOffsetDistance = 1.0;
//      
//        // work with copy of volume since it gets corrupted
//        SmBSplineVolume *pBSV = new (*pContext) SmBSplineVolume(*pSrf);  
//        SmObjDelete sSurfCleanup(pBSV);
//        vol = pBSV->GetOrCreateGwNurbPointer();
//         
//        // optional scale tolerances to size of minmax box
//        if (eps < SM_EFF_ZERO)  //this is what triggers it
//          {
//            SmExtent3d sBBox;
//            pSrf->CalculateBoundingBox(sBBox);
//            double dSize = sBBox.GetSize().Length();
//            eps = dSize/300.0;
//            dOffsetDistance = dSize/100; 
//          }
//      
//        if (epc < eps)  // and for zero epc, use 1/2 volume tol
//          {
//            epc = eps/2.0;
//          }
//      
//      
//        STACKS S; 
//        N_InitNurbs(&S);
//      
//        // Now start setting up what we can
//        hs     = NULL;
//        hi[0]  = -1;
//        cui    = N_AllocArrayTripleCrvPtrs(0,&S);
//        cui[0] = NULL;
//      
//        cuo    = N_AllocArrayRealCrvPtrs(0,&S);
//        ho[0]  = 3;
//        cuo[0] = N_AllocArrayCrvPtrs(3,&S);
//       
//        // Get constant parameter uv curves (lines) from volume
//        NL_PARAMETER ul, ur, vb, vt;
//        N_SrfGetParameterBounds(vol, &ul, &ur, &vb, &vt );
//      
//        // domain corner points
//        NL_POINT  P1;  P1.x = ul; P1.y = vb; P1.z = NL_NOZ;
//        NL_POINT  P2;  P2.x = ul; P2.y = vt; P2.z = NL_NOZ;
//        NL_POINT  P3;  P3.x = ur; P3.y = vt; P3.z = NL_NOZ;
//        NL_POINT  P4;  P4.x = ur; P4.y = vb; P4.z = NL_NOZ;
//      
//        CURVE crv1;
//        N_CrvInitArrays(&crv1);
//        N_CrvLineFrom2Pts(P1, P2, &crv1, &S);
//        cuo[0][0] = &crv1;
//        CURVE crv2;
//        N_CrvInitArrays(&crv2);
//        N_CrvLineFrom2Pts(P2, P3, &crv2, &S);
//        cuo[0][1] = &crv2;
//        CURVE crv3;
//        N_CrvInitArrays(&crv3);
//        N_CrvLineFrom2Pts(P3, P4, &crv3, &S);
//        cuo[0][2] = &crv3;
//        CURVE crv4;
//        N_CrvInitArrays(&crv4);
//        N_CrvLineFrom2Pts(P4, P1, &crv4, &S);
//        cuo[0][3] = &crv4;
//      
//        NL_PARAMETER *u, *v;
//        NL_INDEX m;
//        NL_INDEX **DT, *hd;
//
//        N_TessTrimmedSrf(vol,cuo,cui,n,ho,hi,hs,epc,eps,tol,&u,&v,&m,&DT,&hd,&S) ;
//       
//        // Tessellation worked - get out triangles and dump to graphics
//        NL_INDEX *alf, *bet, *gam;
//        NL_INDEX numtri;
//        if (N_TessGetTriangles(DT,hd,m,&alf,&bet,&gam,&numtri,&S) == NL_YES) 
//          {
//            N_EndNurbs(&S);
//            SER(SM_ERR); // extraction failed
//          }
//        else 
//          {
//            for (long kk=0; kk<=numtri; kk++) 
//              {
//                SmPoint3d sUV1(u[alf[kk]],v[alf[kk]]);
//                SmPoint3d sPts[3];
//                SmPoint3d sUVPts[3];
//                SE(pBSV->EvaluatePoint(sUV1,sPts[0]));
//                sUVPts[0] = sUV1;
//                SmPoint3d sUV2(u[bet[kk]],v[bet[kk]]);
//                SE(pBSV->EvaluatePoint(sUV2,sPts[1]));
//                sUVPts[1] = sUV2;
//                SmPoint3d sUV3(u[gam[kk]],v[gam[kk]]);
//                SE(pBSV->EvaluatePoint(sUV3,sPts[2]));
//                sUVPts[2] = sUV3;
//                SmVector3d sNorms[3];
//                SE(pBSV->EvaluateNormal(sUV1,TRUE,TRUE,sNorms[0]));
//                SE(pBSV->EvaluateNormal(sUV2,TRUE,TRUE,sNorms[1]));
//                SE(pBSV->EvaluateNormal(sUV3,TRUE,TRUE,sNorms[2]));
//                sPts[0] = sPts[0] + dOffsetDistance * sNorms[0];
//                sPts[1] = sPts[1] + dOffsetDistance * sNorms[1];
//                sPts[2] = sPts[2] + dOffsetDistance * sNorms[2];
//                if (bReverseNormals) 
//                  {
//                    sNorms[0] = - sNorms[0];
//                    sNorms[1] = - sNorms[1];
//                    sNorms[2] = - sNorms[2];
//                  }
//                // Output a triangle
//                SER(rPolygonOutput.OutputPolygon(0,3,sPts,sNorms,sUVPts,pBSV, NULL));
//              }
//          }
//        
//        N_EndNurbs(&S);
//      
//        return SM_SUCCESS;
//      
//      } // end SmVolume::OutputPolygons

 
// gwc: a function to consider adding later on
//      /*******************************************************************//**
//      PURPOSE:  Change knot and control Point values to extend the BSpline's
//           Natural UVWDomain to the specified parameter value by extending the
//           range of one of the volume's boundary spans.  
//      
//      NOTES:
//        This method of volume extension can generate wildly changing volume shapes.
//        Only use this function to increase the domain of a volume by a tolerance
//        amount. 
//      
//        One of the 4 possible boundary values is modified.  The input 
//        eCrossSectionSurfParam value of SM_VP_U or SM_VP_V specifies
//        whether a U or V boundary is modified.
//      
//        When the given parameter value, dNewUVorWParam, is less than the current 
//        UVWDomain Min the lower boundary is moved, when the value is larger
//        than the current UVWDomain max the upper boundary is moved, else no
//        action is taken.
//        
//        This function does not change the number of knots of Control Points.
//          
//      ***********************************************************************/
//      
//      SmStatus SmBSplineVolume::ExpandBoundarySpan
//        (double dNewUVorWParam,                    // in : desired new U, V or W NaturalDomain boundary value
//         SmVolumeParamType eCrossSectionSurfParam) // in : oneof: SM_VP_U, SM_VP_V, SM_VP_W
//      {
//        // return value
//        SmStatus RtnStatus = SM_ERR ;
//      
//        // Bspline locals
//        SmExtent3d UVWDomain = GetNaturalParamDomain() ;
//      
//        // N_SrfExtendByParamDist locals
//        VOLUME *volP = this->GetOrCreateGwNurbPointer();
//        NL_FLAG     dflg =   (eCrossSectionSurfParam == SM_VP_U) ? NL_UDIR
//                        : (eCrossSectionSurfParam == SM_VP_V) ? NL_VDIR 
//                        :                                       NL_WDIR ;
//        NL_FLAG     end  =   (   (dflg == UDIR && dNewUVorWParam > UVWDomain.GetMax().x) 
//                           || (dflg == VDIR && dNewUVorWParam > UVWDomain.GetMax().y)
//                           || (dflg == WDIR && dNewUVorWParam > UVWDomain.GetMax().z)) ? NL_END
//                        : (   (dflg == UDIR && dNewUVorWParam < UVWDomain.GetMin().x) 
//                           || (dflg == VDIR && dNewUVorWParam < UVWDomain.GetMin().y)
//                           || (dflg == WDIR && dNewUVorWParam < UVWDomain.GetMin().z)) ? NL_START
//                        : 0 ;
//        NL_FLAG     cont = CMAX ;
//        VOLUME   surQ ; 
//        N_VolumeInitArrays(&surQ) ;
//        STACKS   SL ;  
//        
//        // no work - specified param value contained within UVWDomain
//        if(end == 0) return(SM_SUCCESS) ;        
//        
//        // Start NURBS
//        SmNLibStackHandler sSH(&SL);
//      
//        // pass the call along
//        NL_FLAG error = N_SrfExtendByParamDist(volP, dNewUVorWParam, dflg, end, cont, &surQ, &SL, &SL) ;
//        if(error == NL_YES) { return SM_ERR ; }
//      
//        // verify that volP is the same size as surQ
//        SmBoolean bSameSize =   (   volP->net->m == surQ.net->m
//                                 && volP->net->n == surQ.net->n
//                                 && volP->net->o == surQ.net->o
//                                 && volP->knu->n == surQ.knu->n
//                                 && volP->knv->n == surQ.knv->n 
//                                 && volP->knw->n == surQ.knw->n)
//                              ? TRUE
//                              : FALSE ;
//        SM_ASSERT(bSameSize) ;
//        
//        // when N_SrfExtendByParamDist worked as expected
//        if(bSameSize) { // copy surQ values into volP
//                        Notify(SM_NO_PRE_EDIT, this, NULL) ;
//                        N_SrfCopy(&surQ, volP, &SL) ;
//                        Notify(SM_NO_POST_EDIT, this, NULL) ;
//      
//                        RtnStatus = SM_SUCCESS ;
//                      }
//        else          { // something went wrong in the N_SrfExtendByParamDist call - leave volP unmodified
//                        RtnStatus = SM_ERR ;
//                      }    
//      
//        // all done
//        SM_DUMP_AND_ASSERT2_VALID(this) ;
//        return(RtnStatus);
//      
//      } // end SmBSplineVolume::ExpandBoundarySpan


/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
static ULONG s_lVolEvalCount = 0;
ULONG smsurf_GetVolumeEvalCount() 
 { return s_lVolEvalCount; 
 
 } // end smsurf_GetVolumeEvalCount

// this functionality is now in SmBSplineVolume::EvaluateSimple
//      /*******************************************************************//**
//      PURPOSE: Evaluate the point and partial derivatives of a volume.
//      
//      NOTES: You must allocate the array of output derivatives to be
//         size of derivatives you expect.
//      
//      Example:
//         ULONG lHighestDeriv = 2;  // 2nd derivative in U, V, and W
//         SmVector3d sDerivs[lHighestDeriv+1][lHighestDeriv+1][lHighestDeriv+1];
//         Evaluate(crUVW,lHighestDeriv, TRUE, TRUE, TRUE, sDerivs[0], TRUE);
//      
//         SmPoint3d sPoint = sDerivs[0][0][0];
//         SmVector3d sDU   = sDerivs[1][0][0];
//         SmVector3d sDV   = sDerivs[0][1][0];
//         SmVector3d sDW   = sDerivs[0][0][1];
//      
//         // Following available when lHighestDeriv >= 1   (bOnlyUpperHalf is always set to TRUE)
//         SmVector3d sDUV  = sDerivs[1][1][0];       which is the same as sDVU
//         SmVector3d sDVW  = sDerivs[0][1][1]; 
//         SmVector3d sDUW  = sDerivs[1][0][1];
//      
//         // Following available when lHighestDeriv >= 2
//         SmVector3d sDUU  = sDerivs[2][0][0];
//         SmVector3d sDVV  = sDerivs[0][2][0];
//         SmVector3d sDWW  = sDerivs[0][0][2];
//      
//         // Following available when lHighestDeriv >= 1
//         SmVector3d sDUUV  = sDerivs[2][1][0];    
//         SmVector3d sDUVV  = sDerivs[1][2][0];
//         SmVector3d sDUUVV = sDerivs[2][2][0];
//      
//         // This can go on until you get to the limit of the derivatives as follows:
//         SmVector3d sDUUUUVVVV = sDerivs[4][4][4]; 
//      
//         // The maximum derivative obtainable is MAX_DERIV which equals 5.
//         // SmVector3d sMax[MAX_DERIV+1][MAX_DERIV+1][MAX_DERIV+1] is the largest matrix 
//         // we can evaluate on a volume.
//      
//      Note: bOnlyUpperHalf as used in the SmSurface and SmCurve Evaluate functions always applies
//            and so is not included as an argument to the SmVolume::Evaluate function
//      
//      ***********************************************************************/
//      SmStatus SmBSplineVolume::SimpleEvaluate
//        (const SmPoint3d & crUVW,    // in : param value to evaluate
//         ULONG lHighestDeriv,        // in : number of derivatives, 0, 1, 2, or 3
//         SmBoolean bUFromLeft,       // in : if P is on U interval boundary
//                                     //      TRUE  = evaluate P in upper interval where P is on the left of the interval
//                                     //      FALSE = evaluate P in lower interval where P is on the right of the interval
//         SmBoolean bVFromLeft,       // in : if P is on V interval boundary
//                                     //      TRUE  = evaluate P in upper interval where P is on the left of the interval
//                                     //      FALSE = evaluate P in lower interval where P is on the right of the interval
//         SmBoolean bWFromLeft,       // in : if P is on V interval boundary
//                                     //      TRUE  = evaluate P in upper interval where P is on the left of the interval
//                                     //      FALSE = evaluate P in lower interval where P is on the right of the interval
//         SmVector3d *aDerivatives,   // out: matrix of evaluations values
//                                     //      sized:[lHighestDeriv+1][lHighestDeriv+1][lHighestDeriv+1]
//                                     //      lHighestDerivative 
//                                     //            0,  sized:[1]    order:[D]
//                                     //            1,  sized:[8]    order:[D  Dw  Dv  ---
//                                     //                                    Du --- --- ---]
//                                     //            2,  sized:[27]   order:[D   Dw   Dww   Dv   Dvw   ----   Dvv   ----   ----
//                                     //                                    Du  Duw  ----  Duv  ----  -----  ----  ----  -----
//                                     //                                    Duu ---- ----- ---- ----- ----- ----- ----- ------]
//                                     //            3,  sized:[64]   order:[D    Dw   Dww  Dwww  Dv   Dvw  Dvww ----  Dvv  Dvvw ---- ----  Dvvv ---- ---- ----
//                                     //                                    Du   Duw  Duww ----  Duv  Duvw ---- ----  Duvv ---- ---- ----  ---- ---- ---- ----
//                                     //                                    Duu  Duuw ---- ----  ---- ---- ---- ----  ---- ---- ---- ----  ---- ---- ---- ----
//                                     //                                    Duuu ---- ---- ----  ---- ---- ---- ----  ---- ---- ---- ----  ---- ---- ---- ----]
//                                     //      3d organized: D[UdirCnt][VDirCnt][WDirCnt]
//                                     //      1d organized: [D Dw Dww ... Dv Dvw Dvww ... Dvv Dvvw Dvvww ...
//                                     //                     Du Duw Duww ... Duv Duvw Duvww ... Duvv Duvvw Duvvww ...
//                                     //                     Duu Duuw Duuww ... Duuv Duuvw Duuvww ... Duuvv Duuvvw Duuvvww ...]
//         SmBoolean bNonZeroTangents, // in : TRUE = replace zero tangent vectors with properly oriented tol sized vectors 
//                                     //      FALSE= return exact tangent values, default:[TRUE]
//         SmBoolean bDoZeroSampling)  // in : for internal use only, always set to TRUE, default:[TRUE]
//        const 
//      {
//        // track total evaluate count
//        s_lVolEvalCount++;
//      
//        // give user a chance to escape out
//        static ULONG lCounter = 0;
//        lCounter ++;
//        if (lCounter % 10000 == 0) 
//          {
//            if (smos_ExcapeCallback()) 
//              { return SM_ERR; }
//          }
//      
//        // check input
//      
//        // bOnlyUpperHalf can be TRUE only if deriv requests are equal.
//        if ( lHighestDeriv > MAXDEG ) SER( SM_ERR_INVALID_INPUT );
//      
//        // no clamping in volumes - it's not supported
//        //      // See about clamping the input parameters to the domain
//        //      SmPoint3d sUVW = crUVW;
//        //      SmExtent3d sNaturalUVWDomain = this->GetNaturalParamDomain();
//        //      if(   !sNaturalUVWDomain.ContainsPoint3d(sUVW)
//        //         && !IsOutOfBoundsEnabled())  // GWC:MODIFIED OUTOFBOUNDS
//        //        {
//        //          sUVW = sNaturalUVWDomain.ClampPoint3d(sUVW);
//        //        }
//      
//        // Set up NLib style matrix indirection pointers 
//        //   Assign ptrs into aDerivatives to SD.
//        //     We are a little tricky casting here because we know
//        //     that a NL_POINT and a SmVector3d are both three doubles
//        //     in the same order (x,y,z).
//        ULONG i, j, sm, jw, jc;
//        NL_POINT **SD[ GW_MAX_DERIV+1 ];
//        NL_POINT *SDD[(GW_MAX_DERIV+1)*(GW_MAX_DERIV+1)];
//        for (i=0,sm=0,jw=0,jc=0; i<=lHighestDeriv; i++,sm+=lHighestDeriv+1) 
//          {
//            SD[i] = &SDD[sm];
//            for(j=0;j<=lHighestDeriv; j++,jc++,jw+=lHighestDeriv+1)
//              {
//                 SDD[jc] = SM_REINTERPRET_CAST(NL_POINT*,&aDerivatives[jw]);
//              }
//          }
//      
//        // set N_VolumeDerivs flags
//        NL_FLAG uflg = (bUFromLeft) ? NL_LEFT : NL_RIGHT;
//        NL_FLAG vflg = (bVFromLeft) ? NL_LEFT : NL_RIGHT;
//        NL_FLAG wflg = (bWFromLeft) ? NL_LEFT : NL_RIGHT;
//      
//        // pass the call along
//        NL_FLAG err =0;
//        gw_VOLUME *vol = ((SmBSplineVolume *)this)->GetOrCreateGwNurbPointer() ;
//        err = N_VolumeDerivs(vol, 
//                             sUVW.x, sUVW.y, sUVW.z,
//                             uflg, vflg, wflg, NL_TRUE, 
//                             lHighestDeriv, lHighestDeriv, lHighestDeriv, 
//                             SD);
//      
//        // check for errors
//        if (err == NL_YES) 
//          { SER(SM_ERR); }
//      
//        // when asked to protect against zero length tangent values
//        if(bNonZeroTangents && (lHighestDeriv > 0))
//          {
//            // position and tangents
//            //    watch out for aDerivatives boundary violations,
//            //    for now: when a derivative is missing - just assign the first derivative that is available to both values.
//            SmPoint3d  sOrigPnt = aDerivatives[0];
//            SmVector3d sOrigDU  = aDerivatives[(lHighestDeriv+1)*(lHighestDeriv+1)] ;
//            SmVector3d sOrigDV  = aDerivatives[(lHighestDeriv+1)] ;               
//            SmVector3d sOrigDW  = aDerivatives[1] ;
//      
//            // tangent sizes
//            double sOrigDULenSq = sOrigDU.LengthSquared() ;
//            double sOrigDVLenSq = sOrigDV.LengthSquared() ;
//            double sOrigDWLenSq = sOrigDW.LengthSquared() ;
//      
//            // tolerance
//            double     dTol         = SM_EFF_ZERO * (1.0 + sOrigPnt.GetMaxDimension());
//            double     dTolSq       = dTol * dTol;
//      
//            // test
//            SmBoolean bZeroDU = sOrigDULenSq < dTolSq ;
//            SmBoolean bZeroDV = sOrigDVLenSq < dTolSq ;
//            SmBoolean bZeroDW = sOrigDWLenSq < dTolSq ;
//            SmBoolean bFixedDU = !bZeroDU ;
//            SmBoolean bFixedDV = !bZeroDV ;
//            SmBoolean bFixedDW = !bZeroDW ;
//      
//            // no work -  no zero tangents to fix
//            //    when both original tangents are larger than tolerance.
//            //        This used to quit when tangents were short but the
//            //             ratio of their lengths was in tolerance.
//            //        I've removed that to let the trick attempt to find a nonZeroTangent value
//            //             for all short enough tangents.  GWC 
//            if( bZeroDU == FALSE && bZeroDV == FALSE && bZeroDW == FALSE )
//              { return SM_SUCCESS ; }
//      
//            // arrive here to kluge a short properly-directed vector for a zero 1st derivative.
//            //   Note: the magnitude of a kluged derivative doesn't matter
//            //         because it's not correct anyway.  (It's supposed to be zero.)
//            //         So just try to find its direction, and return something short.
//      
//            // When working with just one zero tangent vector,
//            //    Suv works best for both Su and Sv, else use their respective 2nd derivs.
//      
//            // If we're at the 'top' of the domain, i.e., the 'good' parameter is
//            // entering a singularity instead of leaving it, then the degenerate
//            // derivative is shrinking, so its change (Suv) is in the opposite
//            // direction of the derivative.
//            double dFlipUDeriv = sUVW.x > sNaturalUVWDomain.GetUInterval().GetMid() ? -1.0 : 1.0 ;
//            double dFlipVDeriv = sUVW.y > sNaturalUVWDomain.GetVInterval().GetMid() ? -1.0 : 1.0 ;
//            double dFlipWDeriv = sUVW.z > sNaturalUVWDomain.GetWInterval().GetMid() ? -1.0 : 1.0 ;
//      
//            // check for a point with two zero tangents
//            //  I've tried to anticipate this but we may need to rethink the behavior
//            //   in response to this rare case.  I suspect we may not get a good solution
//            //   without inspecting nearby points.
//            //   Example 1 :  a singular side next to a repeated endPoint row
//            //     solution:  get nonZeroTangent values in the repeated endPoint direction
//            //                at two points - use those to estimate sDUV (the 2nd derivative)
//            //                and then use sDUV as the estimate for the tangent in the
//            //                the degenerate direction.
//            //  Example 2 : a pair of singular sides meeting at a corner
//            //    solution:  get estimates from nearby points.  All the derivatives at
//            //               the corner will be zero and this trick won't work here.
//            //     I've not coded up anything as elaboate as mentioned above - in practice
//            //     we may need to do that. For now, I'll output a message so that if
//            //     a failed evaluate causes a bigger failure we'll have a hint of where to
//            //     look first.
//            SmBoolean bTwoZeroTangents   =    (bZeroDU && bZeroDV)
//                                           || (bZeroDV && bZeroDW)
//                                           || (bZeroDW && bZeroDU) ;
//            SmBoolean bThreeZeroTangents = (bZeroDU && bZeroDV && bZeroDW) ;
//      
//            // when not enough derivatives were asked for - recurse to this function
//            //   note: we may have to use the recursion trick when bTwoZeroTangents is TRUE
//            //         but we'll ignore that for now.
//            if(lHighestDeriv < 2)
//              {
//                // recurse values
//                SmVector3d aReDer[27] ;
//      
//                // pass the call along
//                SER(Evaluate(crUVW, 2, bUFromLeft, bVFromLeft, bWFromLeft, aReDer, TRUE)) ;
//      
//                // save the kluged tangent values
//                aDerivatives[(lHighestDeriv+1)*(lHighestDeriv+1)] = aReDer[9] ;  // sDU     
//                aDerivatives[(lHighestDeriv+1)]                   = aReDer[3] ;  // sDV     
//                aDerivatives[1]                                   = aReDer[1] ;  // sDW     
//      
//                // all done
//                return(SM_SUCCESS) ;
//      
//              }  // end recursion branch when asked for less than 2 derivatives
//      
//            // try getting 1st Tangent from Suv value
//            SmVector3d sDUV = aDerivatives[ (lHighestDeriv+1)*(lHighestDeriv+1)+(lHighestDeriv+1)]; // Suv
//            SmVector3d sDVW = aDerivatives[ lHighestDeriv+2 ];                                      // Svw
//            SmVector3d sDUW = aDerivatives[ (lHighestDeriv+1)*(lHighestDeriv+1)+1 ];                // Suw
//      
//            // try getting 1st tangents from 2nd derivative values that are associated with nonZero 1st derivatives
//      
//            // sDU from sDUV
//            if(    bZeroDU && !bFixedDU
//               && !bZeroDV 
//               &&  sDUV.LengthSquared() > dTolSq )
//              { sDUV.Unitize() ; 
//                sDUV *= ( 1.1*dTol ); 
//                bFixedDU = TRUE ; 
//                aDerivatives[(lHighestDeriv+1)*(lHighestDeriv+1)] = dFlipUDeriv * sDUV ; // sDU  
//              }       
//      
//            // sDU from sDUW
//            if(    bZeroDU && !bFixedDU 
//               && !bZeroDW 
//               &&  sDUW.LengthSquared() > dTolSq )
//              { sDUW.Unitize() ; 
//                sDUW *= ( 1.1*dTol ); 
//                bFixedDU = TRUE ; 
//                aDerivatives[(lHighestDeriv+1)*(lHighestDeriv+1)] = dFlipUDeriv * sDUW ; // sDU  
//              } 
//                    
//            // sDV from sDUV
//            if(    bZeroDV && !bFixedDV 
//               && !bZeroDU 
//               &&  sDUV.LengthSquared() > dTolSq )
//              { sDUV.Unitize() ; 
//                sDUV *= ( 1.1*dTol ); 
//                bFixedDV = TRUE ; 
//                aDerivatives[(lHighestDeriv+1)] = dFlipVDeriv * sDUV ; // sDV  
//              } 
//                                      
//            // sDV from sDVW
//            if(    bZeroDV && !bFixedDV 
//               && !bZeroDW 
//               &&  sDVW.LengthSquared() > dTolSq )
//              { sDVW.Unitize() ; 
//                sDVW *= ( 1.1*dTol ); 
//                bFixedDV = TRUE ; 
//                aDerivatives[(lHighestDeriv+1)] = dFlipVDeriv * sDVW ; // sDV  
//              }
//                                      
//            // sDW from sDUW
//            if(    bZeroDW && !bFixedDW 
//               && !bZeroDU 
//               &&  sDUW.LengthSquared() > dTolSq )
//              { sDUW.Unitize() ; 
//                sDUW *= ( 1.1*dTol ); 
//                bFixedDW = TRUE ; 
//                aDerivatives[1] = dFlipWDeriv * sDUW ; // sDW  
//              } 
//                                                     
//            // sDW from sDVW
//            if(    bZeroDW && !bFixedDW 
//               && !bZeroDV 
//               &&  sDVW.LengthSquared() > dTolSq )
//              { sDVW.Unitize() ; 
//                sDVW *= ( 1.1*dTol ); 
//                bFixedDW = TRUE ; 
//                aDerivatives[1] = dFlipWDeriv * sDVW ; // sDW  
//              }                                        
//      
//            // all done if we fixed all problems
//            if(bFixedDU && bFixedDV && bFixedDW)
//              { return(SM_SUCCESS) ; }
//      
//            // 2nd cross derivatives didn't work.
//            // arrive here for cases where 
//            //    all 1st derivatives are zero or
//            //    the nonzero 1st derivatives are associated with zero 2nd cross derivatives.
//      
//            // Suv didn't work, try Suu, Svv, Sww for the short tangent
//      
//            // Su and Suu
//            if(bZeroDU && !bFixedDU)
//              {
//                SmVector3d sDUU = aDerivatives[2*(lHighestDeriv+1)*(lHighestDeriv+1)] ;
//                if( sDUU.LengthSquared() > dTolSq )
//                  {
//                    sDUU.Unitize();
//                    sDUU *= 1.1 * dTol * dFlipUDeriv ;
//                    aDerivatives[(lHighestDeriv+1)*(lHighestDeriv+1)] = sDUU ;  // Su
//                    bFixedDU = TRUE ;
//                  }
//              }
//      
//            // Sv and Svv
//            if(bZeroDV && !bFixedDV)
//              {
//                SmVector3d sDVV = aDerivatives[2*(lHighestDeriv+1)] ;
//                if( sDVV.LengthSquared() > dTolSq )
//                  {
//                    sDVV.Unitize();
//                    sDVV *= 1.1 * dTol * dFlipVDeriv ;
//                    aDerivatives[(lHighestDeriv+1)] = sDVV ;  // Sv
//                    bFixedDV = TRUE ;
//                  }
//              }
//      
//            // Sw and Sww
//            if(bZeroDW && !bFixedDW)
//              {
//                SmVector3d sDWW = aDerivatives[2] ;
//                if( sDWW.LengthSquared() > dTolSq )
//                  {
//                    sDWW.Unitize();
//                    sDWW *= 1.1 * dTol * dFlipWDeriv ;
//                    aDerivatives[1] = sDWW ;  // Sw
//                    bFixedDW = TRUE ;
//                  }
//              }
//      
//            // all done if we fixed all problems
//            if(bFixedDU && bFixedDV && bFixedDW)
//              { return(SM_SUCCESS) ; }
//      
//            // arrive here when: need to kluge a direction for a zero 1st derivative, 
//            //                   and getting the kluged 1st der direction from 2nd derivative failed.
//      
//            //    (bZeroDU && !bFixedDU) 
//            // or (bZeroDV && !bFixedDV)
//            // or (bZeroDW && !bFixedDW)  - work until they are all Fixed
//      
//            // recurse values
//            SmVector3d aReDer[27] ;
//            double dStep = 1.0e-5/10.0;  // works well in practice.
//      
//            // Try finding a nonZeroTangent value with a more brute-force method.
//            // Step around and try to find a good one.
//            // for larger and larger step off distances
//            while(bDoZeroSampling && dStep < 0.1)
//              {
//                // increase the step off distance
//                dStep = dStep * 10.0;
//      
//                // for this step size: try 5 directions; last 4 are diagonals.
//                SmPoint3d sTestUVW[9];  
//      
//                // Make the first step go in the most likely direction:
//                // direction of good deriv, towards center of volume.
//                // This should have the smallest effect on things.
//                for(i=0;i<9;i++)
//                  { sTestUVW[i] = sUVW ; }
//      
//                // step in U direction 
//                if     (   sOrigDULenSq > sOrigDVLenSq
//                        && sOrigDULenSq > sOrigDWLenSq) 
//                  { sTestUVW[0].x += dFlipUDeriv * dStep ; }
//      
//                // step in V direction
//                else if(   sOrigDVLenSq > sOrigDULenSq
//                        && sOrigDVLenSq > sOrigDWLenSq) 
//                  { sTestUVW[0].y += dFlipVDeriv * dStep ; }
//      
//                // step in W direction
//                else 
//                  { sTestUVW[0].z += dFlipWDeriv * dStep ; }
//            
//                // If that doesn't work, just try all eight different octant directions
//                sTestUVW[1].x = sUVW.x + dStep;
//                sTestUVW[1].y = sUVW.y + dStep;
//                sTestUVW[1].z = sUVW.z + dStep;
//                         
//                sTestUVW[2].x = sUVW.x + dStep;
//                sTestUVW[2].y = sUVW.y + dStep;
//                sTestUVW[2].z = sUVW.z - dStep;
//                         
//                sTestUVW[3].x = sUVW.x + dStep;
//                sTestUVW[3].y = sUVW.y - dStep;
//                sTestUVW[3].z = sUVW.z + dStep;
//                         
//                sTestUVW[4].x = sUVW.x + dStep;
//                sTestUVW[4].y = sUVW.y - dStep;
//                sTestUVW[4].z = sUVW.z - dStep;
//                         
//                sTestUVW[5].x = sUVW.x - dStep;
//                sTestUVW[5].y = sUVW.y + dStep;
//                sTestUVW[5].z = sUVW.z + dStep;
//                         
//                sTestUVW[6].x = sUVW.x - dStep;
//                sTestUVW[6].y = sUVW.y + dStep;
//                sTestUVW[6].z = sUVW.z - dStep;
//                         
//                sTestUVW[7].x = sUVW.x - dStep;
//                sTestUVW[7].y = sUVW.y - dStep;
//                sTestUVW[7].z = sUVW.z + dStep;
//            
//                sTestUVW[8].x = sUVW.x - dStep;
//                sTestUVW[8].y = sUVW.y - dStep;
//                sTestUVW[8].z = sUVW.z - dStep;
//            
//                // for all 9 sample directions at this step size
//                for(i=0;i<9;i++)
//                  {
//                    // pass the call along
//                    sTestUVW[i]   = sNaturalUVWDomain.ClampPoint3d(sTestUVW[i]);
//                    SmStatus sRtn = Evaluate(sTestUVW[i], 2, bUFromLeft, bVFromLeft, bWFromLeft, aReDer, TRUE, FALSE) ;
//      
//                    // save the kluged tangent values
//                    if(sRtn == SM_SUCCESS)
//                      {
//                        aDerivatives[(lHighestDeriv+1)*(lHighestDeriv+1)] = aReDer[9] ; // sDU
//                        aDerivatives[(lHighestDeriv+1)]                   = aReDer[3] ; // sDV
//                        aDerivatives[1]                                   = aReDer[1] ; // sDW
//            
//                        // all done
//                        return(SM_SUCCESS) ;
//                      }
//      
//                  } // end iter all test points - recursion branch
//              } // end iter all larger step sizes
//      
//            if(bTwoZeroTangents)
//              {                
//                SM_DBG_WARN(_T("SM_ERR in SmBSplineSurface::Evaluate - evaluating a surface point with two zero tangents")) ;
//              }
//      
//            // If we make it to here we are unable to get both of the first
//            // derivatives to a non-zero size.  Return an error.
//            return SM_ERR;
//      
//          } // end asked to fix nonZeroTangents check
//      
//        // all done
//        return(SM_SUCCESS) ;
//      
//      } // end SmBSplineVolume::SimpleEvaluate

//      /*******************************************************************//**
//      PURPOSE: Given a parametric point on a volume evalute the corresponding
//          Euclidian point.
//      
//      NOTES: 
//      ***********************************************************************/
//      SmStatus SmBSplineVolume::EvaluatePoint
//        (const SmPoint3d & crUVW,         // in : Target Nurb UV Point
//         SmPoint3d       & rPoint)        // out: Resulting Image Point
//       const
//      {
//        // every 10,000 evals - give the escape key a check
//        static ULONG lCounter = 0;
//        lCounter ++;
//        if (lCounter % 10000 == 0) 
//          {
//            if (smos_ExcapeCallback()) 
//              {
//                return SM_ERR;
//              }
//          }
//      
//        // count the Volume evals
//      #ifdef SM_DEBUG_CODE
//        s_lVolEvalCount++;
//      #endif
//      
//        // clamp DomainPoints (when OutOfBounds evals are not enabled)
//        // Note: there was a typo here, in which the actual calls used
//        // crUVW instead of sUVW, i.e., the clamping didn't actually happen.
//        // However, 'fixing' it makes things worse, so don't clamp.
//        // (In which case, we don't need sVU at all.)
//        // [bd, 05Jan06; regressions: Two Tori, and my_shell_demo iters 0, 3, 6]
//      #if 0  // (don't clamp)
//        SmPoint3d sUVW = crUVW;
//        SmExtent3d sNaturalUVWDomain = this->GetNaturalParamDomain();
//        if (   !sNaturalUVWDomain.ContainsPoint3d(sUVW)
//            && !IsOutOfBoundsEnabled()) // GWC:MODIFIED OUTOFBOUNDS
//          {
//            sUVW = sNaturalUVWDomain.ClampPoint3d(sUVW);
//          }
//      #endif  // (don't clamp)
//      
//        
//        // pass the eval call along     
//        NL_FLAG flg = NL_LEFT;
//        NL_POINT  C;
//        NL_FLAG err = 0;
//        gw_VOLUME *vol = ((SmBSplineVolume *)this)->GetOrCreateGwNurbPointer() ;
//        err = N_VolumeEval(vol,crUVW.x,crUVW.y,crUVW.z,flg,flg,flg,&C);
//        if ( err != 0) 
//            SER(SM_ERR);
//      
//        // set output
//        COPY_XYZ(C,rPoint);
//        return SM_SUCCESS;
//      
//      } // end SmBSplineVolume::EvaluatePoint
//      
