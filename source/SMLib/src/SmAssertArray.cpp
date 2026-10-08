// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*******************************************************************//**
* FILE NAME --- SmAssertArray.cpp
* PURPOSE: Implementation of SmAssertReport, and
*                               SmAssertArray methods.
**********************************************************************/

#include "StdAfx.h"

#include <SmPoly.h>
// Remove Composites
// #include <SmCFace.h>
#include <SmAssembly.h>
#include <SmGraphicsOutput.h>
#include <SmCrvOnSurf.h>
#include <SmOffsetSurface.h>
#include <SmFilletGeom.h>
#include <stdarg.h>


// Global SM_ASSERT_VALID stream management
SmBoolean   SM_bAssertValidStream = 1 ;  // TRUE=SmAssertArray::Dump writes reports for empty AssertArrays, FALSE=doesn't
SmBoolean   SmSetAssertValidStream(SmBoolean bAssertValidStream) { SmBoolean bRtn = SM_bAssertValidStream ;
                                                                   SM_bAssertValidStream = bAssertValidStream ;
                                                                   return(bRtn) ;
                                                                 }
SmBoolean & SmGetAssertValidStream()                             { return(SM_bAssertValidStream) ; }

// a default SmAssertReportList - not expected to be used except in error
SmAssertReportLabel sAssertDefault_list[] =
{
  { SM_AT_UNKNOWN, _T("Unknown_Assert"), _T("Accessed an Unknown AssertReport_list - error") }
} ;

/*******************************************************************//**
PURPOSE: Global access to AssertLabelLists

NOTES:
***********************************************************************/
SmAssertReportLabel * SM_GetAssertLabelList
 (SM_TYPE lExeClassType,  // in : ClassType of ObjClass of ObjClass::AssertValid() method to execute
  ULONG   lListIndex)     // in : When a type has more than one array of labels, use this
                          //      index to specify which array is of interest.
                          //      default:[0]
{
  switch( lExeClassType )
    {
      case SmAxis2Placement_TYPE            : return(sAssertAxis2Placement_list) ;
      case SmPolarConversion_TYPE           : switch(lListIndex)
                                                { case 0 : return(sAssertPolarConversion_list) ;
                                                  case 1 : return(sAssertPolarIsCurrent_list) ;
                                                 default : return(sAssertDefault_list) ;
                                               }
      case SmCircle_TYPE                    : return(sAssertCircle_list) ;
      case SmEllipse_TYPE                   : return(sAssertEllipse_list) ;
      case SmAttribute_TYPE                 : return(sAssertAttribute_list) ;
      case SmBSplineCurve_TYPE              : return(sAssertBSplineCurve_list) ;
      case SmCurve_TYPE                     : return(sAssertCurve_list) ;
      case SmAObject_TYPE                   : return(sAssertAObject_list) ;
      case SmLine_TYPE                      : return(sAssertLine_list) ;
      case SmTree_TYPE                      : return(sAssertTree_list) ;
      case SmHashTable_TYPE                 : return(sAssertHashTable_list) ; 
      case SmObjsInVoxels_TYPE              : return(sAssertObjsInVoxels_list) ; 
      case SmSolutionArray_TYPE             : return(sAssertSolutionArray_list) ;
      case SmBSplineSurface_TYPE            : return(sAssertBSplineSurface_list) ;
      case SmPlane_TYPE                     : return(sAssertPlane_list) ;
      case SmCone_TYPE                      : return(sAssertCone_list) ;
      case SmSphere_TYPE                    : return(sAssertSphere_list) ;
      case SmCylinder_TYPE                  : return(sAssertCylinder_list) ;
      case SmTorus_TYPE                     : return(sAssertTorus_list) ;
      case SmSurfOfRevolution_TYPE          : return(sAssertSurfOfRevolution_list) ;
      case SmExtent1d_TYPE                  : return(sAssertExtent1d_list) ;
      case SmExtent2d_TYPE                  : return(sAssertExtent2d_list) ;
      case SmExtent3d_TYPE                  : return(sAssertExtent3d_list) ;
      case SmPeriodicExtent1d_TYPE          : return(sAssertPeriodicExtent1d_list) ;
      case SmPolarBox_TYPE                  : return(sAssertPolarBox_list) ;
      case SmPseudoBox_TYPE                 : return(sAssertPseudoBox_list) ;
      case SmDerivSurfDefinition_TYPE       : return(sAssertDerivSurfDefinition_list) ;
      case SmSurfaceCache_TYPE              : return(sAssertSurfaceCache_list) ;
      case SmSurfOfExtrusion_TYPE           : return(sAssertSurfOfExtrusion_list) ;
      case SmSurface_TYPE                   : return(sAssertSurface_list) ;
      case SmCrvOnSurf_TYPE                 : return(sAssertCrvOnSurf_list) ;
      case SmProjectedCurve_TYPE            : return(sAssertProjectedCurve_list) ;

      case SmFilletEdge_TYPE                : return(sAssertFilletEdge_list) ;
      case SmFilletBrep_TYPE                : return(sAssertFilletBrep_list) ;
      case SmFilletVertex_TYPE              : return(sAssertFilletVertex_list) ;
      case SmFilletVertexuse_TYPE           : return(sAssertFilletVertexuse_list) ;
      case SmFilletEdgeuse_TYPE             : return(sAssertFilletEdgeuse_list) ;

      case SmSpaceBend_TYPE                 : return(sAssertBend_list) ;
      case SmSpaceUnbend_TYPE               : return(sAssertUnbend_list) ;
      case SmOffsetSurface_TYPE             : return(sAssertOffsetSurface_list) ;
      case SmOffsetCurve_TYPE               : return(sAssertOffsetCurve_list) ;

      case SmAssembly_TYPE                  : return(sAssertAssembly_list) ;
      case SmAssemblyInstance_TYPE          : return(sAssertAssemblyInstance_list) ;
      case SmSAGObject_TYPE                 : return(sAssertSAGObject_list) ;
      case SmBrep_TYPE                      : switch(lListIndex)
                                               { case 0 : return(sAssertBrep_list) ;
                                                 case 1 : return(sAssertValidatePointers_list) ;
                                                 case 2 : return(sAssertValidateTolerances_list) ;
                                                 case 3 : return(sAssertCoincidentTopology_list) ;
                                                 case 4 : return(sAssertSubTopology_list) ;
                                                 default : return(sAssertDefault_list) ;
                                               }
// Remove Composites
//       case SmCEdge_TYPE                     : return(sAssertCEdge_list) ;
// Remove Composites
//       case SmCFace_TYPE                     : return(sAssertCFace_list) ;
      case SmEdge_TYPE                      : return(sAssertEdge_list) ;
      case SmFace_TYPE                      : return(sAssertFace_list) ;
      case SmCurveClassification_TYPE       : return(sAssertCurveClassification_list) ;
      case SmCurveInterval_TYPE             : return(sAssertCurveInterval_list) ;
      case SmPointClassification_TYPE       : return(sAssertPointClassification_list) ;
      case SmTopology_TYPE                  : return(sAssertTopology_list) ;
      case SmOwningTopology_TYPE            : return(sAssertOwningTopology_list) ;
      case SmEdgeuse_TYPE                   : return(sAssertEdgeuse_list) ;
      case SmFaceuse_TYPE                   : return(sAssertFaceuse_list) ;
      case SmVertexuse_TYPE                 : return(sAssertVertexuse_list) ;
      case SmVertex_TYPE                    : return(sAssertVertex_list) ;
      case SmLoop_TYPE                      : return(sAssertLoop_list) ;
      case SmLoopuse_TYPE                   : return(sAssertLoopuse_list) ;
      case SmRegion_TYPE                    : return(sAssertRegion_list) ;
      case SmShell_TYPE                     : return(sAssertShell_list) ;
      case SmPolyFace_TYPE                  : return(sAssertPolyFace_list) ;
      case SmCPolyFace_TYPE                 : return(sAssertCPolyFace_list) ;
      case SmPolyLoop_TYPE                  : return(sAssertPolyLoop_list) ;
      case SmPolyVertex_TYPE                : return(sAssertPolyVertex_list) ;
      case SmPolyEdge_TYPE                  : return(sAssertPolyEdge_list) ;
      case SmPolyRegion_TYPE                : return(sAssertPolyRegion_list) ;
      case SmPolyShell_TYPE                 : return(sAssertPolyShell_list) ;
      case SmPolyBrep_TYPE                  : return(sAssertPolyBrep_list) ;
      case SmTrimmingTools_TYPE             : switch(lListIndex)
                                               { case 0 : return(sTrimmingCheckLoopForMiniHourglass_list) ;
                                                 case 1 : return(sTrimmingCheckTessellation_list) ;
                                                 case 2 : return(sTrimmingCheckFace_list) ;
                                                 case 3 : return(sTrimmingCheckLoop_list) ;
                                                 default : return(sAssertDefault_list) ;
                                               }
      case SmCrvInVolume_TYPE               : return(sAssertCrvInVolume_list) ;
      case SmSrfInVolume_TYPE               : return(sAssertSrfInVolume_list) ;
                                            
      case SmVolume_TYPE                    : return(sAssertVolume_list) ;
      case SmBSplineVolume_TYPE             : return(sAssertBSplineVolume_list) ;
      case SmTransform_TYPE                 : return(sAssertTransform_list) ;
      case SmBendVolume_TYPE                : return(sAssertBendVolume_list) ;
      case SmTwistVolume_TYPE               : return(sAssertTwistVolume_list) ;
      case SmUnbendVolume_TYPE              : return(sAssertUnbendVolume_list) ;
                                            

      default                               : return(sAssertDefault_list) ;
    }

} // end SM_GetAssertLabelList

/*******************************************************************//**
PURPOSE: Hack to map class_TYPE to strings

NOTES: I'd think that this functionality could be provided by some
       kind of SM_COMMON macro or at leasta static SmObject method
       But I couldn't think of one that didn't require as much
       or more work than this method.  Replace this method when you
       see a better way of doing this.
***********************************************************************/
TCHAR const * SM_GetObjTypeString
 (SM_TYPE lExeClassType)       // in : ClassType of ObjClass of ObjClass::AssertValid() method to execute
{
  switch( lExeClassType )
    {
      case SmAxis2Placement_TYPE            : return(_T("SmAxis2Placement")) ;
      case SmPolarConversion_TYPE           : return(_T("SmPolarConversion")) ;
      case SmCircle_TYPE                    : return(_T("SmCircle")) ;
      case SmEllipse_TYPE                   : return(_T("SmEllipse")) ;
      case SmAttribute_TYPE                 : return(_T("SmAttribute")) ;
      case SmBSplineCurve_TYPE              : return(_T("SmBSplineCurve")) ;
      case SmCurve_TYPE                     : return(_T("SmCurve")) ;
      case SmAObject_TYPE                   : return(_T("SmAObject")) ;
      case SmLine_TYPE                      : return(_T("SmLine")) ;
      case SmTree_TYPE                      : return(_T("SmTree")) ;
      case SmSolutionArray_TYPE             : return(_T("SmSolutionArray")) ;
      case SmBSplineSurface_TYPE            : return(_T("SmBSplineSurface")) ;
      case SmPlane_TYPE                     : return(_T("SmPlane")) ;
      case SmCone_TYPE                      : return(_T("SmCone")) ;
      case SmSphere_TYPE                    : return(_T("SmSphere")) ;
      case SmCylinder_TYPE                  : return(_T("SmCylinder")) ;
      case SmTorus_TYPE                     : return(_T("SmTorus")) ;
      case SmSurfOfRevolution_TYPE          : return(_T("SmSurfOfRevolution")) ;
      case SmExtent1d_TYPE                  : return(_T("SmExtent1d")) ;
      case SmExtent2d_TYPE                  : return(_T("SmExtent2d")) ;
      case SmExtent3d_TYPE                  : return(_T("SmExtent3d")) ;
      case SmPeriodicExtent1d_TYPE          : return(_T("SmPeriodicExtent1d")) ;
      case SmPolarBox_TYPE                  : return(_T("SmPolarBox")) ;
      case SmPseudoBox_TYPE                 : return(_T("SmPseudoBox")) ;
      case SmDerivSurfDefinition_TYPE       : return(_T("SmDerivSurfDefinition")) ;
      case SmSurfaceCache_TYPE              : return(_T("SmSurfaceCache")) ;
      case SmSurfOfExtrusion_TYPE           : return(_T("SmSurfOfExtrusion")) ;
      case SmSurface_TYPE                   : return(_T("SmSurface")) ;
      case SmCrvOnSurf_TYPE                 : return(_T("SmCrvOnSurf")) ;
      case SmProjectedCurve_TYPE            : return(_T("SmProjectedCurve")) ;
      case SmCrvInVolume_TYPE               : return(_T("SmCrvInVolume")) ;
      case SmSrfInVolume_TYPE               : return(_T("SmSrfInVolume")) ;

      case SmFilletEdge_TYPE                : return(_T("SmFilletEdge")) ;
      case SmFilletBrep_TYPE                : return(_T("SmFilletBrep")) ;
      case SmFilletVertex_TYPE              : return(_T("SmFilletVertex")) ;
      case SmFilletVertexuse_TYPE           : return(_T("SmFilletVertexuse")) ;
      case SmFilletEdgeuse_TYPE             : return(_T("SmFilletEdgeuse")) ;
      case SmSpaceBend_TYPE                 : return(_T("SmSpaceBend")) ;
      case SmSpaceUnbend_TYPE               : return(_T("SmSpaceUnbend")) ;
      case SmOffsetSurface_TYPE             : return(_T("SmOffsetSurface")) ;
      case SmOffsetCurve_TYPE               : return(_T("SmOffsetCurve")) ;

      case SmAssembly_TYPE                  : return(_T("SmAssembly")) ;
      case SmAssemblyInstance_TYPE          : return(_T("SmAssemblyInstance")) ;
      case SmSAGObject_TYPE                 : return(_T("SmSAGObject")) ;
      case SmBrep_TYPE                      : return(_T("SmBrep")) ;
      case SmVolume_TYPE                    : return(_T("SmVolume")) ;
      case SmBendVolume_TYPE                : return(_T("SmBendVolume")) ;
      case SmUnbendVolume_TYPE              : return(_T("SmUnbendVolume")) ;
      case SmTransform_TYPE                 : return(_T("SmTransform")) ;
      case SmBSplineVolume_TYPE             : return(_T("SmBSplineVolume")) ;
      case SmTwistVolume_TYPE               : return(_T("SmTwistVolume")) ;
// Remove Composites
//       case SmCEdge_TYPE                     : return(_T("SmCEdge")) ;
// Remove Composites
//       case SmCFace_TYPE                     : return(_T("SmCFace")) ;
      case SmEdge_TYPE                      : return(_T("SmEdge")) ;
      case SmFace_TYPE                      : return(_T("SmFace")) ;
      case SmCurveClassification_TYPE       : return(_T("SmCurveClassification")) ;
      case SmCurveInterval_TYPE             : return(_T("SmCurveInterval")) ;
      case SmPointClassification_TYPE       : return(_T("SmPointClassification")) ;
      case SmTopology_TYPE                  : return(_T("SmTopology")) ;
      case SmOwningTopology_TYPE            : return(_T("SmOwningTopology")) ;
      case SmEdgeuse_TYPE                   : return(_T("SmEdgeuse")) ;
      case SmFaceuse_TYPE                   : return(_T("SmFaceuse")) ;
      case SmVertexuse_TYPE                 : return(_T("SmVertexuse")) ;
      case SmVertex_TYPE                    : return(_T("SmVertex")) ;
      case SmLoop_TYPE                      : return(_T("SmLoop")) ;
      case SmLoopuse_TYPE                   : return(_T("SmLoopuse")) ;
      case SmRegion_TYPE                    : return(_T("SmRegion")) ;
      case SmShell_TYPE                     : return(_T("SmShell")) ;
      case SmPolyFace_TYPE                  : return(_T("SmPolyFace")) ;
      case SmCPolyFace_TYPE                 : return(_T("SmCPolyFace")) ;
      case SmPolyLoop_TYPE                  : return(_T("SmPolyLoop")) ;
      case SmPolyVertex_TYPE                : return(_T("SmPolyVertex")) ;
      case SmPolyEdge_TYPE                  : return(_T("SmPolyEdge")) ;
      case SmPolyRegion_TYPE                : return(_T("SmPolyRegion")) ;
      case SmPolyShell_TYPE                 : return(_T("SmPolyShell")) ;
      case SmPolyBrep_TYPE                  : return(_T("SmPolyBrep")) ;
      case SmTrimmingTools_TYPE             : return(_T("SmTrimmingTools")) ;


      default                               : WARN(_T("SM_GetAssertLabelList: Passed an unhandled Type value.") ) ;
                                              return(_T("SmDefault")) ;
    }

} // end SM_GetObjTypeString

/*******************************************************************//**
PURPOSE: SmAssert Report constructor

NOTES:
***********************************************************************/
SmAssertReport::SmAssertReport
 (const void  * pReporter,           // in : this object of the AssertValid() method running this test.
  ULONG         lTestIndex,          // in : Index into associated AsssertFailure description array
  SM_TYPE       lTestClass,          // in : class type making the call - used to fetch AssertLabelList
  const void  * pOwner,              // in : Object identified by AssertValid as owner of this test.
  SM_TYPE       lOwnerType,          // in : this->GetType()
  const TCHAR * sOwnerTypeString,    // in : this->GetTypeString()
  const void  * pOther,              // in : Optional other object ptr, (good for bad gap reports)
  SM_TYPE       lOtherType,          // in : other->GetType()
  const TCHAR * sOtherTypeString,    // in : other->GetTypeString()
  SmTol3d       sTol3d,              // in : Optional sTol3d value used in this test if appropriate.
                                     //      to ignore:[SM_UNDEF_DOUBLE]
  double        dVal,                // in : Optional value checked against tol
                                     // in : to ignore:[SM_UNDEF_DOUBLE]
  ULONG         lListIndex,          // in : When a type has more than one array of labels, use this
                                     //      index to specify which array is of interest.
                                     //      default:[0]
  const TCHAR * pOptMessage          // in : notNULL = use this as the SmAssertReport dump string
                                     //      NULL    = use the TextIndex val to fetch dump string from Array
#ifdef SM_DEBUG_CODE
 ,const TCHAR * const file_name      // in : filename label for Assert report
 ,ULONG               line_num       // in : linenumber label for Assert report
#endif // SM_DEBUG_CODE
  )
: m_bOK             (FALSE),        // assume test failed - otherwise we wouldn't be constructing the report
  m_pReporter       (pReporter),
  m_lReportingType  (lTestClass),
  m_lListIndex      (lListIndex),
  m_lTestIndex      (lTestIndex),
  m_eAssertType     (SM_AT_UNKNOWN),
  m_pName           (_T("")),
  m_pMessage        ((pOptMessage && *pOptMessage) ? pOptMessage : _T("")),
  m_pMessageForFile (_T("")),
  // m_pHealName       (_T("")),
  // m_pHealMessage    (_T("")),
#ifdef SM_DEBUG_CODE
  m_file_name       (file_name),
  m_line_num        (line_num ),
#endif // SM_DEBUG_CODE
  m_pOwner          (pOwner),
  m_lOwnerType      (lOwnerType),
  m_sOwnerTypeString(sOwnerTypeString),
  m_pOther          (pOther),
  m_lOtherType      (lOtherType),
  m_sOtherTypeString(sOtherTypeString),
  m_sTol3d          (sTol3d),
  m_dValue          (dVal),
  m_dDist3d         (SM_UNDEF_DOUBLE)
{
  // store local SmAssertReportLabel values
  SmAssertReportLabel *pLabelList = SM_GetAssertLabelList(lTestClass, lListIndex) ;

  m_eAssertType     = pLabelList[lTestIndex].m_eAssertType ;
  m_pName           = pLabelList[lTestIndex].m_pName ;
  m_pMessage        = (pOptMessage && *pOptMessage) ? pOptMessage : pLabelList[lTestIndex].m_pMessage ;
  m_pMessageForFile = pLabelList[lTestIndex].m_pMessageForFile ;
  // m_pHealName    = pLabelList[lTestIndex].m_pHealName ;
  // m_pHealMessage = NULL ;

  m_sOwnerParam.SetUninitialized() ;
  m_sOtherParam.SetUninitialized() ;

  // assume this test failed - otherwise we wouldn't be writing the report
  // NOTE: PLACE BREAK POINT HERE TO STOP DEBUG RUN AT A FAILING ASSERTVALID CHECK
  m_bOK = FALSE ;

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  // draw
  if(bDebugMe)
    {
      Draw() ; sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;


      SmVertex    *pVertex        =   (SmVertex_TYPE          == lOwnerType) || (SmFilletVertex_TYPE    == lOwnerType) ?
                                    (SmVertex *)pOwner : NULL ;

      SmVertexuse *pVertexuse    =    (SmVertexuse_TYPE       == lOwnerType) || (SmFilletVertexuse_TYPE == lOwnerType) ?
                                    (SmVertexuse *)pOwner : NULL ;

// Remove Composites
// SmEdge      *pEdge        =   (SmEdge_TYPE   == lOwnerType) || (SmCEdge_TYPE  == lOwnerType) ?
      SmEdge      *pEdge        =   (SmEdge_TYPE   == lOwnerType) ?
                                    (SmEdge *)pOwner : NULL ;

      SmEdgeuse   *pEdgeuse        =   (SmEdgeuse_TYPE       == lOwnerType) || (SmFilletEdgeuse_TYPE == lOwnerType) ?
                                    (SmEdgeuse *)pOwner : NULL ;

      SmLoop      *pLoop      =    (SmLoop_TYPE == lOwnerType) ? (SmLoop *)pOwner : NULL ;
      SmLoopuse   *pLoopuse   =    (SmLoopuse_TYPE == lOwnerType) ? (SmLoopuse *)pOwner : NULL ;
// Remove Composites
//      SmFace      *pFace      =    (SmFace_TYPE == lOwnerType) || (SmCFace_TYPE == lOwnerType) ? (SmFace *)pOwner : NULL ;
      SmFace      *pFace      =    (SmFace_TYPE == lOwnerType) ? (SmFace *)pOwner : NULL ;
      SmFaceuse   *pFaceuse   =    (SmFaceuse_TYPE == lOwnerType) ? (SmFaceuse *)pOwner : NULL ;
      SmRegion    *pRegion    =    (SmRegion_TYPE == lOwnerType) ? (SmRegion *)pOwner : NULL ;
      SmBrep      *pBrep      =    (SmBrep_TYPE == lOwnerType)
                                || (SmFilletBrep_TYPE == lOwnerType) ? (SmBrep *)pOwner : NULL ;
      SmCurve     *pCurve     =  (   (SmCurve_TYPE          == lOwnerType)
                                  || (SmBSplineCurve_TYPE   == lOwnerType)
                                  || (SmCrvOnSurf_TYPE      == lOwnerType)
                                  || (SmEllipse_TYPE        == lOwnerType)
                                  || (SmTangentField_TYPE   == lOwnerType)
                                  || (SmCircle_TYPE         == lOwnerType)
                                  || (SmHyperbola_TYPE      == lOwnerType)
                                  || (SmLine_TYPE           == lOwnerType)
                                  || (SmParabola_TYPE       == lOwnerType)
                                  || (SmCompositeCurve_TYPE == lOwnerType)
                                  || (SmHermiteCurve_TYPE   == lOwnerType)
                                  || (SmIsoCurve_TYPE       == lOwnerType)
                                  || (SmOffsetCurve_TYPE    == lOwnerType)
                                  || (SmCrvInVolume_TYPE    == lOwnerType)) ? (SmCurve *)pOwner : NULL ;
      SmSurface   *pSurface   =  (   (SmSurface_TYPE             == lOwnerType)
                                  || (SmBSplineSurface_TYPE      == lOwnerType)
                                  || (SmPlane_TYPE               == lOwnerType)
                                  || (SmSurfOfExtrusion_TYPE     == lOwnerType)
                                  || (SmSurfOfRevolution_TYPE    == lOwnerType)
                                  || (SmCone_TYPE                == lOwnerType)
                                  || (SmCylinder_TYPE            == lOwnerType)
                                  || (SmSphere_TYPE              == lOwnerType)
                                  || (SmTorus_TYPE               == lOwnerType)
                                  || (SmCurveBoundedSurface_TYPE == lOwnerType)
                                  || (SmOffsetSurface_TYPE       == lOwnerType)
                                  || (SmSTEPSurface_TYPE         == lOwnerType)
                                  || (SmSrfInVolume_TYPE         == lOwnerType)) ? (SmSurface *)pOwner : NULL ;

      SmVolume   *pVolume   =    (   (SmVolume_TYPE             == lOwnerType)
                                  || (SmBSplineVolume_TYPE      == lOwnerType)
                                  || (SmBendVolume_TYPE         == lOwnerType)
                                  || (SmUnbendVolume_TYPE       == lOwnerType)
                                  || (SmTransform_TYPE          == lOwnerType)
                                  || (SmTwistVolume_TYPE        == lOwnerType)) ? (SmVolume *)pOwner : NULL ; 

      // fill in owners when possible
      pVertex  =   pVertex ? pVertex
                 : pVertexuse ? pVertexuse->GetVertex()
                 : pEdgeuse && pEdgeuse->GetVertexuse() ? pEdgeuse->GetVertexuse()->GetVertex()
                 : pLoopuse && pLoopuse->GetVertexuse() ? pLoopuse->GetVertexuse()->GetVertex()
                 : NULL ;
      pEdge    =   pEdge    ? pEdge
                 : pCurve   ? (SmEdge *)pCurve->GetEdge()
                 : pEdgeuse ? pEdgeuse->GetEdge()
                 : pLoopuse && (pLoopuse->GetLoopuseType() == SmEdgeuse_TYPE) && pLoopuse->GetEUorVU() ? (SmEdge *)pLoopuse->GetEUorVU()
                 : pVertexuse && pVertexuse->GetEdgeuse() ? pVertexuse->GetEdgeuse()->GetEdge()
                 : NULL ;
      pFace    =   pFace    ? pFace
                 : pSurface ? (SmFace *)pSurface->GetFace()
                 : pFaceuse ? pFaceuse->GetFace()
                 : pEdgeuse ? pEdgeuse->GetFace()
                 : pLoopuse   && pLoopuse->GetFaceuse() ? pLoopuse->GetFaceuse()->GetFace()
                 : pVertexuse && pVertexuse->GetFaceuse() ? pVertexuse->GetFaceuse()->GetFace()
                 : NULL ;
      pBrep    =   pBrep   ? pBrep
                 : pVertex ? pVertex->GetBrep()
                 : pEdge   ? pEdge->GetBrep()
                 : pFace   ? pFace->GetBrep()
                 : pRegion ? pRegion->GetBrep()
                 : NULL ;
      pCurve   =   pCurve ? pCurve
                 : pEdge  ? pEdge->GetCurve()
                 : NULL ;
      pSurface =   pSurface ? pSurface
                 : pFace  ? pFace->GetSurface()
                 : NULL ;

      if(pVertex    == pOwner) pVertex   ->Dump() ;
      if(pVertexuse == pOwner) pVertexuse->Dump() ;
      if(pEdge      == pOwner) pEdge     ->Dump() ;
      if(pEdgeuse   == pOwner) pEdgeuse  ->Dump() ;
      if(pLoop      == pOwner) pLoop     ->Dump() ;
      if(pLoopuse   == pOwner) pLoopuse  ->Dump() ;
      if(pFace      == pOwner) pFace     ->Dump() ;
      if(pFaceuse   == pOwner) pFaceuse  ->Dump() ;
      if(pRegion    == pOwner) pRegion   ->Dump() ;
      if(pBrep      == pOwner) pBrep     ->Dump() ;
      if(pCurve     == pOwner) pCurve    ->Dump() ;
      if(pSurface   == pOwner) pSurface  ->Dump() ;
      if(pVolume    == pOwner) pVolume   ->Dump() ;

      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 1,0,1) ; if(pVolume) pVolume->Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,1) ; if(pSurface) pSurface->DrawUV() ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,0,0) ; if(pFace) pFace->Draw(SM_DM_CROSSHATCH) ; sm_GraphicsLoop() ;
      smgfx_SetLook(2,3, 1,0,0) ; if(pCurve) pCurve->Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook(3,4, 1,0,1) ; if(pEdge) pEdge->Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook(2,3, 0,1,0) ; if(pEdgeuse) pEdgeuse->Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook(4,5, 1,.5,0); if(pVertex) pVertex->Draw() ; sm_GraphicsLoop() ;

      smgfx_SetLook(3,4, 1,0,0) ; if(pLoop)    pLoop   ->Draw(3,FALSE,NULL,FALSE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(2,3, 1,0,0) ; if(pLoopuse) pLoopuse->Draw() ; sm_GraphicsLoop() ;

      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

} // end SmAssertReport::SmAssertReport constructor

/*******************************************************************//**
PURPOSE: SmAssert Report copy constructor

NOTES:
***********************************************************************/
SmAssertReport::SmAssertReport(const SmAssertReport & crOriginal)
 : m_bOK             (crOriginal.m_bOK             ),
   m_pReporter       (crOriginal.m_pReporter       ),
   m_lReportingType  (crOriginal.m_lReportingType  ),
   m_lListIndex      (crOriginal.m_lListIndex      ),
   m_lTestIndex      (crOriginal.m_lTestIndex      ),
   m_eAssertType     (crOriginal.m_eAssertType     ),
   // something for strings
   m_pName           (crOriginal.m_pName           ),
   m_pMessage        (crOriginal.m_pMessage        ),
   m_pMessageForFile (crOriginal.m_pMessageForFile ),
   //  m_pHealName       (crOriginal.m_pHealName       ),
   //  m_pHealMessage    (crOriginal.m_pHealMessage    ),
   #ifdef SM_DEBUG_CODE
   m_file_name       (crOriginal.m_file_name       ),
   m_line_num        (crOriginal.m_line_num        ),
   #endif // SM_DEBUG_CODE
   m_pOwner          (crOriginal.m_pOwner          ),
   m_lOwnerType      (crOriginal.m_lOwnerType      ),
   m_sOwnerTypeString(crOriginal.m_sOwnerTypeString),
   m_pOther          (crOriginal.m_pOther          ),
   m_lOtherType      (crOriginal.m_lOtherType      ),
   m_sOtherTypeString(crOriginal.m_sOtherTypeString),
   m_sTol3d          (crOriginal.m_sTol3d          ),
   m_dValue          (crOriginal.m_dValue          ),
   m_sOwnerParam     (crOriginal.m_sOwnerParam     ),
   m_sOtherParam     (crOriginal.m_sOtherParam     ),
   m_dDist3d         (crOriginal.m_dDist3d         )
 { } // end SmAssertReport Copy Constructor

/*******************************************************************//**
PURPOSE: SmAssertReport Assignment operator

NOTES:
***********************************************************************/
SmAssertReport & SmAssertReport::operator=
 (const SmAssertReport &crOther )
 {
   if(&crOther == this) return *this ;
   m_bOK              = crOther.m_bOK;
   m_lReportingType   = crOther.m_lReportingType;
   m_lListIndex       = crOther.m_lListIndex;
   m_lTestIndex       = crOther.m_lTestIndex;
   m_eAssertType      = crOther.m_eAssertType;
   m_pName            = crOther.m_pName;
   m_pMessage         = crOther.m_pMessage;
   m_pMessageForFile  = crOther.m_pMessageForFile;
#ifdef SM_DEBUG_CODE
   m_file_name        = crOther.m_file_name ;
   m_line_num         = crOther.m_line_num ;
#endif // SM_DEBUG_CODE
   m_pOwner           = crOther.m_pOwner;
   m_lOwnerType       = crOther.m_lOwnerType;
   m_sOwnerTypeString = crOther.m_sOwnerTypeString ;
   m_pOther           = crOther.m_pOther ;
   m_lOtherType       = crOther.m_lOtherType;
   m_sOtherTypeString = crOther.m_sOtherTypeString;
   m_sTol3d           = crOther.m_sTol3d;
   m_dValue           = crOther.m_dValue;
   m_sOwnerParam      = crOther.m_sOwnerParam;
   m_sOtherParam      = crOther.m_sOtherParam;
   m_dDist3d          = crOther.m_dDist3d;
   return *this ;

 } // end SmAssertReport::operator=

/*******************************************************************//**
PURPOSE: SmAssertReport Equality operator

NOTES:
***********************************************************************/
SmBoolean SmAssertReport::operator==
 (const SmAssertReport &crOther )
 {
   if(&crOther == this) return TRUE ;

   if(m_lReportingType   != crOther.m_lReportingType) return(FALSE) ;
   if(m_lListIndex       != crOther.m_lListIndex)     return(FALSE) ;
   if(m_lTestIndex       != crOther.m_lTestIndex)     return(FALSE) ;
   if(m_eAssertType      != crOther.m_eAssertType)    return(FALSE) ;
   if(m_bOK              != crOther.m_bOK)            return(FALSE) ;
   if(m_pOwner           != crOther.m_pOwner)         return(FALSE) ;
   if(m_pOther           != crOther.m_pOther)         return(FALSE) ;
//  if(m_sTol.val          != crOther.m_sTol.val)       return(FALSE) ;

   return TRUE ;

 } // end SmAssertReport::operator==

/*******************************************************************//**
PURPOSE: a static Delete method for freeing SmAssertReports

NOTES:
***********************************************************************/
void SmAssertReport::Delete( SmAssertReport *pReport )
  { delete pReport; }

/*******************************************************************//**
PURPOSE: Pretty print Assert Reports

NOTES:
***********************************************************************/
void SmAssertReport::Dump() const
{
  Dump(0, 0) ;
} // end SmAssertReport::Dump()

/*******************************************************************//**
PURPOSE: Pretty print Assert Reports

NOTES:
***********************************************************************/
void SmAssertReport::Dump
 (size_t lReportTypeMax,   // in : max ReportType string len in calling AssertArray
  size_t lFileNameMax)     // in : max Filename string len in calling AssertArray
 const
{
  TCHAR sBuff[SM_TBLOCK_SIZE], sBuffForFile[SM_TBLOCK_SIZE];

  // state
  size_t ii, lCnt      = SM_STRLEN(SM_GetObjTypeString(m_lReportingType)) ;
  // SmBoolean bHeal      = m_eAssertType == SM_AT_HEALER ;
  // SmBoolean bNoHealYet = m_eAssertType == SM_AT_NO_HEAL_YET ;

  //  // Report intro: Heal Header
  //  smos_sprintf(sBuff, _T("%s"),   bHeal      ? _T(" HEAL")
  //                              : bNoHealYet ? _T(" NO HEAL YET for")
  //                              : _T("") );
  //  smos_WriteBuffer(sBuff) ;

  // Report intro: PASSED/FAILED
  smos_sprintf(sBuff, _T(" %s"), m_bOK ? _T("PASSED") : _T("FAILED"));
  smos_WriteBuffer(sBuff) ;

  // Report intro: TestIndex, m_pHealName, m_pHealMessage
  //  if(bHeal)
  //    {
  //      // Heal Action Name and Message
  //      smos_sprintf(sBuff, _T(" %s %s"), m_pHealName,
  //                                        m_pHealMessage);
  //      smos_WriteBuffer(sBuff) ;
  //    }
  //  else // end a Heal Report branch
    {
      // Report intro: ObjTypeString
      smos_sprintf(sBuff, _T(" %s"),SM_GetObjTypeString(m_lReportingType));
      smos_WriteBuffer(sBuff) ;

      // align columns
      sBuff[0] = '\0';
      for(ii=0;ii+lCnt<lReportTypeMax;ii++)  { SM_STRCAT(sBuff,_T(" ")) ; }
      smos_WriteBuffer(sBuff);

      // Test Number, Name, and Message
      smos_sprintf(sBuff, _T("::Test #%2ld : %s - %s"), m_lTestIndex,
                                                         m_pName,
                                                         m_pMessage);
      smos_sprintf(sBuffForFile, _T("::Test #%2ld : %s - %s"), m_lTestIndex,
                                                               m_pName,
                                                               m_pMessageForFile);
      smos_WriteBuffer(sBuff, sBuffForFile) ;

      // // Heal Action Name and Message
      // if(m_pHealMessage) { smos_sprintf(sBuff, _T(" %s"), m_pHealMessage);
      //                      smos_WriteBuffer(sBuff) ;
      //                    }
    } // end not a Heal Report branch

#ifdef SM_DEBUG_CODE
  lCnt = SM_STRLEN(m_file_name) ;
  smos_sprintf(sBuff,        _T("\n         %s(%6ld):"), m_file_name, m_line_num) ;
  smos_sprintf(sBuffForFile, _T("\n         %s( ):"), m_file_name) ;
  smos_WriteBuffer(sBuff, sBuffForFile);

  // align columns
  smos_sprintf(sBuff, _T("%s"), _T(" ")) ;
  for(ii=0;ii+lCnt<lFileNameMax;ii++)  { SM_STRCAT(sBuff,_T(" ")) ; }
  smos_WriteBuffer(sBuff);

#else // no SM_DEBUG_CODE
  lFileNameMax = 0 ;
  smos_sprintf(sBuff, _T("%s"), _T("\n       ") );
  smos_WriteBuffer(sBuff);
#endif // no SM_DEBUG_CODE

  // Owner
  smos_sprintf(sBuff,         _T("Owner[%s=0x%p]"), m_sOwnerTypeString,
                                                  m_pOwner) ;
  smos_sprintf(sBuffForFile,  _T("Owner[%s=%s]"),   m_sOwnerTypeString,
                                                  m_pOwner ? _T("NotNULL") : _T("NULL") ) ;
  smos_WriteBuffer(sBuff, sBuffForFile) ;

  // Optional Tolerance
  if(m_sTol3d.val != SM_UNDEF_DOUBLE) { smos_sprintf(sBuff,        _T(", Tol = %6.16lg"), m_sTol3d.val) ;
                                        smos_sprintf(sBuffForFile, _T(", Tol = %6.8lg"),  m_sTol3d.val) ;
                                        smos_WriteBuffer(sBuff, sBuffForFile) ;
                                      }
  // Optional Value
  if(m_dValue != SM_UNDEF_DOUBLE) { smos_sprintf(sBuff,        _T(", Val = %6.16lg"), m_dValue) ;
                                    smos_sprintf(sBuffForFile, _T(", Val = %6.8lg"),  m_dValue) ;
                                    smos_WriteBuffer(sBuff, sBuffForFile) ;
                                  }

  // Optional Dist
  if(m_dDist3d != SM_UNDEF_DOUBLE) { smos_sprintf(sBuff,        _T(", Val = %6.16lg"), m_dDist3d) ;
                                     smos_sprintf(sBuffForFile, _T(", Val = %6.8lg"),  m_dDist3d) ;
                                     smos_WriteBuffer(sBuff, sBuffForFile) ;
                                   }

  // Optional OwnerParam
  if(m_sOwnerParam.IsInitialized()) { smos_sprintf(sBuff,        _T(", OwnerParam = [%6.16lg"), m_sOwnerParam.x) ;
                                      smos_sprintf(sBuffForFile, _T(", OwnerParam = [%6.8lg"),  m_sOwnerParam.x) ;
                                      smos_WriteBuffer(sBuff, sBuffForFile) ;

                                      if(m_sOwnerParam.y != SM_UNDEF_DOUBLE)
                                        {  smos_sprintf(sBuff,        _T(", %6.16lg"), m_sOwnerParam.y) ;
                                           smos_sprintf(sBuffForFile, _T(", %6.8lg"),  m_sOwnerParam.y) ;
                                           smos_WriteBuffer(sBuff, sBuffForFile) ;
                                        }
                                      smos_WriteBuffer(_T("]")) ;
                                    }
  // Optional OtherParam
  if(m_sOtherParam.IsInitialized()) { smos_sprintf(sBuff,        _T(", OtherParam = [%6.16lg"), m_sOtherParam.x) ;
                                      smos_sprintf(sBuffForFile, _T(", OtherParam = [%6.8lg"),  m_sOtherParam.x) ;
                                      smos_WriteBuffer(sBuff, sBuffForFile) ;

                                      if(m_sOtherParam.y != SM_UNDEF_DOUBLE)
                                        {  smos_sprintf(sBuff,        _T(", %6.16lg"), m_sOtherParam.y) ;
                                           smos_sprintf(sBuffForFile, _T(", %6.8lg"),  m_sOtherParam.y) ;
                                           smos_WriteBuffer(sBuff, sBuffForFile) ;
                                        }
                                      smos_WriteBuffer(_T("]")) ;
                                    }
  // Optional Other
  if(m_pOther)
    {
      smos_sprintf(sBuff,         _T(", Other [%s=0x%p]"), m_sOtherTypeString, m_pOther) ;
      smos_sprintf(sBuffForFile,  _T(", Other [%s=%s]"), m_sOtherTypeString, m_pOther ? _T("NotNULL") : _T("NULL") ) ;
      smos_WriteBuffer(sBuff, sBuffForFile) ;
    }

} // end SmAssertReport::Dump

/*******************************************************************//**
PURPOSE: return string for every enum SmAssertType

NOTES:
***********************************************************************/
const TCHAR *SmAssertReport::GetAssertTypeString()
  const
{
  switch(m_eAssertType)
    {
      case SM_AT_UNKNOWN          : return _T("SM_AT_UNKNOWN         ") ;

      case SM_AT_ANGLE            : return _T("SM_AT_ANGLE           ") ;
      case SM_AT_BOX              : return _T("SM_AT_BOX             ") ;
      case SM_AT_CACHE            : return _T("SM_AT_CACHE           ") ;
      case SM_AT_CLOSED           : return _T("SM_AT_CLOSED          ") ;

      case SM_AT_COINCIDENCE      : return _T("SM_AT_COINCIDENCE     ") ;
      case SM_AT_DEGENERATE       : return _T("SM_AT_DEGENERATE      ") ;
      case SM_AT_DIRECTION        : return _T("SM_AT_DIRECTION       ") ;
      case SM_AT_DISTANCE         : return _T("SM_AT_DISTANCE        ") ;

      case SM_AT_DOMAIN           : return _T("SM_AT_DOMAIN          ") ;
      case SM_AT_FLAG             : return _T("SM_AT_FLAG            ") ;
      case SM_AT_GEOMETRIC        : return _T("SM_AT_GEOMETRIC       ") ;
      case SM_AT_GAP              : return _T("SM_AT_GAP             ") ;

      case SM_AT_INSIDE           : return _T("SM_AT_INSIDE          ") ;
      case SM_AT_KNOTS            : return _T("SM_AT_KNOTS           ") ;
      case SM_AT_LIST             : return _T("SM_AT_LIST            ") ;
      case SM_AT_MARK             : return _T("SM_AT_MARK            ") ;

      case SM_AT_MINMAX           : return _T("SM_AT_MINMAX          ") ;
      case SM_AT_PARAMETERIZATION : return _T("SM_AT_PARAMETERIZATION") ;
      case SM_AT_POINTER          : return _T("SM_AT_POINTER         ") ;
      case SM_AT_POINTS           : return _T("SM_AT_POINTS          ") ;

      case SM_AT_RADIUS           : return _T("SM_AT_RADIUS          ") ;
      case SM_AT_SCALE            : return _T("SM_AT_SCALE           ") ;
      case SM_AT_SIZE             : return _T("SM_AT_SIZE            ") ;
      case SM_AT_TYPE             : return _T("SM_AT_TYPE            ") ;

      case SM_AT_VALUES           : return _T("SM_AT_VALUES          ") ;
      case SM_AT_VECTOR           : return _T("SM_AT_VECTOR          ") ;
      case SM_AT_UNIT_VECTOR      : return _T("SM_AT_UNIT_VECTOR     ") ;

      case SM_AT_NESTED_TEST      : return _T("SM_AT_NESTED_TEST     ") ;

      default                     : return _T("Unknown SmAssertType  ") ;

   } // end switch on m_eAssertType

} // end SmAssertReport::GetAssertTypeString

/*******************************************************************//**
PURPOSE: format m_pMessage plus any stored tolerance/value for log output

NOTES:
***********************************************************************/
void SmAssertReport::FormatLogMessage
 (TCHAR * sBuff,
  ULONG   lBuffSize)
  const
{
  const TCHAR * pBaseMsg = m_pMessage ? m_pMessage : _T("?") ;

  if(!sBuff || lBuffSize == 0)
    { return ; }

  sBuff[0] = _T('\0') ;

  const SmBoolean bHasTol = (m_sTol3d.val != SM_UNDEF_DOUBLE) ;
  const SmBoolean bHasVal = (m_dValue != SM_UNDEF_DOUBLE) ;

  if(bHasTol && bHasVal)
    smos_sprintf(sBuff, _T("%s (offset=%6.8lg, tolerance=%6.8lg)"), pBaseMsg, m_dValue, m_sTol3d.val) ;
  else if(bHasVal)
    smos_sprintf(sBuff, _T("%s (offset=%6.8lg)"), pBaseMsg, m_dValue) ;
  else if(bHasTol)
    smos_sprintf(sBuff, _T("%s (tolerance=%6.8lg)"), pBaseMsg, m_sTol3d.val) ;
  else
    {
      SM_STRNCPY(sBuff, pBaseMsg, lBuffSize) ;
      sBuff[lBuffSize - 1] = _T('\0') ;
    }
} // end SmAssertReport::FormatLogMessage

/*******************************************************************//**
PURPOSE: Return a default Graphics Center point based on owner type
  and location that can be rendered in association with a containing
  Brep to draw labels on a Brep to show known problems.

NOTES:
  returns TRUE  = Owner has a meaningful 3d Geometric Point - ex: SmVertex, SmEdge, ...
  returns FALSE = Owner has no inherent 3D Geometric Point  - ex: SmExtent1d, SmPolarBox, ...
***********************************************************************/
SmBoolean SmAssertReport::GetGraphicsPoint
  (SmPoint3d &rGraphicsPoint,    // out: when return value is TRUE, set with display Graphic Point
                                 //      else set to SmVector3d::SetUninitialized().
   SM_TYPE   *pOptType)          // in : optional base type for m_pOwner, used for recursion,
                                 //      NULL to ignore, default:[NULL]
  const
{
  // init output value
  rGraphicsPoint.SetUninitialized() ;

  // locals
  const SmCurve        * pCurve ;
  SmTArray<SmEdge*>      sEdges ;
  const SmSurface      * pSurface ;
  SmTArray<SmFace*>      sFaces ;
  SmTArray<SmPoint2d>    sUV ;
  SmTArray<SmPoint3d>    sXYZ ;
  SmTArray<SmPolyFace*>  sPFaces ;

  const SmVolume       * pVolume ;
  SmTArray<SmPolyShell*> sPShells ;
  SmTArray<SmShell*>     sShells ;
  SmTArray<SmVertex*>    sVertices ;

  SmTArray<SmAObject*>   sAObjects ;
  SmTArray<SmSAGObject*> sSAGObjects ;
  SM_TYPE                sType ;

  // GWC TODO: 1. all these branches need to be turned into methods.  This
  //              giant switch programming style is for the birds.
  //           2. This is prototype code. Every return argument for every
  //              fetch operator has to be checked for NULL conditions.
  //              It's possible that empty objects will end up being asked
  //              for a graphics point and they should say they don't have one.
  //           3. A few classes could have points that currently don't labeled below as such.

  // switch on Owner type
  switch(pOptType ? *pOptType : m_lOwnerType)
    {
      case SmCurve_TYPE              :
      case SmLine_TYPE               :
      case SmCircle_TYPE             :
      case SmEllipse_TYPE            :
      case SmBSplineCurve_TYPE       : pCurve = ((SmCurve *)m_pOwner) ;
                                       pCurve->EvaluatePoint(pCurve->GetNaturalInterval().Evaluate(.4), rGraphicsPoint) ;
                                       break ;
      case SmSurface_TYPE            :
      case SmPlane_TYPE              :
      case SmCone_TYPE               :
      case SmSphere_TYPE             :
      case SmTorus_TYPE              :
      case SmSurfOfExtrusion_TYPE    :
      case SmSurfOfRevolution_TYPE   :
      case SmBSplineSurface_TYPE     : pSurface = ((SmSurface *)m_pOwner) ;
                                       pSurface->EvaluatePoint(pSurface->GetNaturalUVDomain().Evaluate(.6,.6), rGraphicsPoint) ;
                                       break ;

      case SmAttribute_TYPE          : if(pOptType) return(FALSE) ;
                                       { ((SmAttribute *)m_pOwner)->GetUsers(sAObjects) ;
                                         sType = sAObjects.GetAt(0)->GetType() ;
                                         SmAssertReport sReport(sAObjects.GetAt(0), 0,
                                                                sAObjects.GetAt(0)->GetType(),
                                                                sAObjects.GetAt(0),
                                                                sAObjects.GetAt(0)->GetType(),
                                                                sAObjects.GetAt(0)->GetTypeString(),
                                                                NULL, 
                                                                SmUnknown_TYPE, 
                                                                _T(""),
                                                                SM_UNDEF_DOUBLE, 
                                                                SM_UNDEF_DOUBLE, 
                                                                0,
                                                                _T("")
#ifdef SM_DEBUG_CODE
                                                                ,FILE_NAME
                                                                ,LINE_NUMBER
#endif // SM_DEBUG_CODE
                                                                ) ;
                                          return(sReport.GetGraphicsPoint(rGraphicsPoint, &sType)) ;
                                       }

      case SmObject_TYPE             :
      case SmAObject_TYPE            :
      case SmSAGObject_TYPE          :
      case SmTopology_TYPE           :
      case SmOwningTopology_TYPE     : if(pOptType) return(FALSE) ;
                                       sType = ((SmObject *)m_pOwner)->GetType() ;
                                       return(GetGraphicsPoint(rGraphicsPoint, &sType)) ;

      case SmAxis2Placement_TYPE     : rGraphicsPoint = ((SmAxis2Placement *)m_pOwner)->GetOrigin() ;
                                       break ;

      case SmFilletVertexuse_TYPE    : rGraphicsPoint = ((SmFilletVertexuse *)m_pOwner)->GetTsectPnt().CrvPos() ;
                                       break ;

      case SmFilletVertex_TYPE       : rGraphicsPoint = ((SmFilletVertex *)m_pOwner)->GetPoint() ;
                                       break ;

      case SmFilletEdge_TYPE         : pCurve = ((SmFilletEdge *)m_pOwner)->GetFilletBrepEdge()->GetCurve() ;
                                       pCurve->EvaluatePoint(pCurve->GetNaturalInterval().Evaluate(.4), rGraphicsPoint) ;
                                       break ;

      case SmFilletEdgeuse_TYPE      : pCurve = ((SmFilletEdgeuse *)m_pOwner)->GetEdge()->GetCurve() ;
                                       pCurve->EvaluatePoint(pCurve->GetNaturalInterval().Evaluate(.4), rGraphicsPoint) ;
                                       break ;

      case SmVertex_TYPE             : rGraphicsPoint = ((SmVertex *)m_pOwner)->GetPoint() ;
                                       break ;
      case SmVertexuse_TYPE          : rGraphicsPoint = ((SmVertexuse *)m_pOwner)->GetVertex()->GetPoint() ;
                                       break ;

      case SmPolyVertex_TYPE         : rGraphicsPoint = ((SmPolyVertex *)m_pOwner)->GetPoint() ;
                                       break ;

      case SmPolarConversion_TYPE    : pCurve = ((SmPolarConversion *)m_pOwner)->GetCurve() ;
                                       pCurve->EvaluatePoint(pCurve->GetNaturalInterval().Evaluate(.4), rGraphicsPoint) ;
                                       break ;

      case SmCurveClassification_TYPE: pCurve = ((SmCurveClassification *)m_pOwner)->GetCurve() ;
                                       pCurve->EvaluatePoint(pCurve->GetNaturalInterval().Evaluate(.4), rGraphicsPoint) ;
                                       break ;

      case SmCurveInterval_TYPE:       ((SmCurveInterval *)m_pOwner)->m_vStart.FindObjPoint3d(rGraphicsPoint) ;
                                       break ;

      case SmPointClassification_TYPE: ((SmPointClassification *)m_pOwner)->FindObjPoint3d(rGraphicsPoint) ;
                                       break ;

      case SmEdge_TYPE               : pCurve = ((SmEdge *)m_pOwner)->GetCurve() ;
                                       pCurve->EvaluatePoint(pCurve->GetNaturalInterval().Evaluate(.4), rGraphicsPoint) ;
                                       break ;

// Remove Composites
// case SmCEdge_TYPE              : ((SmCEdge *)m_pOwner)->GetEdges(sEdges) ;
//                                       pCurve = sEdges.GetAt(0)->GetCurve() ;
//                                       pCurve->EvaluatePoint(pCurve->GetNaturalInterval().Evaluate(.4), rGraphicsPoint) ;
//                                       break ;

      case SmEdgeuse_TYPE            : pCurve = ((SmEdgeuse *)m_pOwner)->GetEdge()->GetCurve() ;
                                       pCurve->EvaluatePoint(pCurve->GetNaturalInterval().Evaluate(.4), rGraphicsPoint) ;
                                       break ;

      case SmPolyEdge_TYPE           : rGraphicsPoint =  .4 * ((SmPolyEdge *)m_pOwner)->GetStartPoint()
                                                       + .6 * ((SmPolyEdge *)m_pOwner)->GetEndPoint() ;
                                       break ;

      case SmFace_TYPE               : ((SmFace *)m_pOwner)->GetPointsInFace(1, sUV, sXYZ) ;
                                       rGraphicsPoint = sXYZ[0] ;
                                       break ;

// Remove Composites
// case SmCFace_TYPE              : ((SmCFace *)m_pOwner)->GetFaces(sFaces) ;
//                                       sFaces.GetAt(0)->GetPointsInFace(1, sUV, sXYZ) ;
//                                       rGraphicsPoint = sXYZ[0] ;
//                                       break ;

      case SmFaceuse_TYPE            : ((SmFaceuse *)m_pOwner)->GetFace()->GetPointsInFace(1, sUV, sXYZ) ;
                                       rGraphicsPoint = sXYZ[0] ;
                                       break ;

      case SmPolyFace_TYPE           : rGraphicsPoint = ((SmPolyFace *)m_pOwner)->GetCentroid() ;
                                       break ;

      case SmCPolyFace_TYPE          : ((SmCPolyFace *)m_pOwner)->GetPolyFacesOfComposite(sPFaces) ;
                                       rGraphicsPoint = sPFaces.GetAt(0)->GetCentroid() ;
                                       break ;

      case SmSurfaceCache_TYPE       : pSurface = (((SmSurfaceCache *)m_pOwner)->GetSurface()) ;
                                       pSurface->EvaluatePoint(pSurface->GetNaturalUVDomain().Evaluate(.6,.6), rGraphicsPoint) ;
                                       break ;

      case SmVolume_TYPE             :
      case SmBSplineVolume_TYPE      : pVolume = ((SmVolume *)m_pOwner) ;
                                       pVolume->EvaluatePoint(pVolume->GetNaturalParamDomain().Evaluate(.1,.1,0), rGraphicsPoint) ;
                                       break ;

      case SmLoop_TYPE               : { SmFace * pFace = ((SmLoop *)m_pOwner)->GetFace() ;
                                         pFace->GetPointsInFace(1, sUV, sXYZ) ;
                                         if(sUV.GetSize() == 0) 
                                           { 
                                             SmExtent2d crUVDomain ;
                                             SmVector2d sUVPoint ; 
                                             pFace->CalculateUVDomainFromUVTrimCurves(crUVDomain) ;
                                             sUVPoint = crUVDomain.GetMid() ;
                                             pFace->GetSurface()->EvaluatePoint(sUVPoint, rGraphicsPoint) ;
                                           }
                                         else
                                           { rGraphicsPoint = sXYZ[0] ; }
                                       }
                                       break ;

      case SmLoopuse_TYPE            : { SmFace * pFace = ((SmLoopuse *)m_pOwner)->GetFaceuse()->GetFace() ;
                                         pFace->GetPointsInFace(1, sUV, sXYZ) ;
                                         if(sUV.GetSize() == 0) 
                                           { 
                                             SmExtent2d crUVDomain ;
                                             SmVector2d sUVPoint ; 
                                             pFace->CalculateUVDomainFromUVTrimCurves(crUVDomain) ;
                                             sUVPoint = crUVDomain.GetMid() ;
                                             pFace->GetSurface()->EvaluatePoint(sUVPoint, rGraphicsPoint) ;
                                           }
                                         else
                                           { rGraphicsPoint = sXYZ[0] ; }
                                       }
                                       break ;

      case SmPolyLoop_TYPE           : rGraphicsPoint = ((SmPolyLoop *)m_pOwner)->GetPolyFace()->GetCentroid() ;
                                       break ;

      case SmShell_TYPE              : rGraphicsPoint = ((SmShell *)m_pOwner)->GetGraphicsPoint() ;
                                       break ;

      case SmPolyShell_TYPE          : ((SmPolyShell *)m_pOwner)->GetPolyFaces(sPFaces) ;
                                       rGraphicsPoint = sPFaces.GetAt(0)->GetCentroid() ;
                                       break ;

      case SmRegion_TYPE             : ((SmRegion *)m_pOwner)->GetShells(sShells) ;
                                       rGraphicsPoint = sShells.GetAt(0)->GetGraphicsPoint() ;
                                       break ;

      case SmPolyRegion_TYPE         : ((SmPolyRegion *)m_pOwner)->GetPolyShells(sPShells) ;
                                       sPShells.GetAt(0)->GetPolyFaces(sPFaces) ;
                                       rGraphicsPoint = sPFaces.GetAt(0)->GetCentroid() ;
                                       break ;

      case SmBrep_TYPE               : ((SmBrep *)m_pOwner)->GetFaces(sFaces) ;
                                       ((SmBrep *)m_pOwner)->GetVertices(sVertices) ;
                                       if(sFaces.GetSize() > 0) { sFaces.GetAt(0)->GetPointsInFace(1, sUV, sXYZ) ;
                                                                  rGraphicsPoint = sXYZ[0] ;
                                                                }
                                       else if(sVertices.GetSize() > 0) { rGraphicsPoint = sVertices.GetAt(0)->GetPoint() ;
                                                                        }
                                       break ;

      case SmPolyBrep_TYPE           : rGraphicsPoint = ((SmPolyBrep *)m_pOwner)->GetFirstPolyVertex()->GetPoint() ;
                                       break ;

      case SmAssembly_TYPE           : if(pOptType) return(FALSE) ;
                                        // TODO: what to pick on Assembly?
                                       { /*((SmAssembly *)m_pOwner)->GetComponents(sSAGObjects) ;*/
                                         SmTArray<SmAssemblyInstance*> sInstances;
                                         ((SmAssembly*)m_pOwner)->GetAssemblyInstances(sInstances);
                                         SmSAGObject* pComponent = sInstances[0]->GetComponent();
                                         sType = sAObjects.GetAt(0)->GetType() ;
                                         SmAssertReport sReport(pComponent, 0,
                                                                pComponent->GetType(),
                                                                pComponent,
                                                                pComponent->GetType(),
                                                                pComponent->GetTypeString(),
                                                                NULL, 
                                                                SmUnknown_TYPE, 
                                                                _T(""),
                                                                SM_UNDEF_DOUBLE, 
                                                                SM_UNDEF_DOUBLE, 
                                                                0,
                                                                _T("")
#ifdef SM_DEBUG_CODE
                                                                ,FILE_NAME
                                                                ,LINE_NUMBER
#endif // SM_DEBUG_CODE
                                                                ) ;
                                         return(sReport.GetGraphicsPoint(rGraphicsPoint, &sType)) ;
                                       }
                                       break ;


      // things that could have a point - but haven't been coded up yet.
      case SmSolutionArray_TYPE      :
      case SmAssemblyInstance_TYPE   :
      case SmTree_TYPE               :


      // things that don't seem to have a natural graphics point
      case SmFilletBrep_TYPE         :
      case SmDerivSurfDefinition_TYPE:
      case SmExtent1d_TYPE           :
      case SmExtent2d_TYPE           :
      case SmExtent3d_TYPE           :
      case SmPeriodicExtent1d_TYPE   :
      case SmPolarBox_TYPE           :
      case SmPseudoBox_TYPE          :
      default                        : return FALSE ;
    } // end switch on lOwnerType

  // arrive here after setting rGraphicsPoint
  return(TRUE) ;

} // end SmAssertReport::GetGraphicsPoint

/*******************************************************************//**
* END SmAssertReport Method definitions
***********************************************************************/

/*******************************************************************//**
* BEGIN SmAssertArray Method definitions
***********************************************************************/

/*******************************************************************//**
PURPOSE: SmAssertArray destructor

NOTES:
   This calls SmAssertReport::Delete().  That exists so that these can
   always be deleted in this SMLib dll; deleting them from other dll's
   can cause a crash.
***********************************************************************/
SmAssertArray::~SmAssertArray()
{
  ULONG ii;
  for (ii=0; ii<m_lSize; ii++)
    {
      SmAssertReport * pRep = m_pData[ii];
      if (pRep)
        {
          SmAssertReport::Delete( pRep );
          m_pData[ii] = NULL;
        }
    }
  m_lSize = 0;

} // end SmAssertArray destructor

/*******************************************************************//**
PURPOSE: rtn TRUE when tgtAssertReport is already in Array, else return FALSE

NOTES:
***********************************************************************/
SmBoolean SmAssertArray::Contains
  (const SmAssertReport & crAssertReport) // in : Object to find
{
  ULONG ii ;

  // for every report
  for(ii=0; ii<this->GetSize(); ii++)
    {
      // return TRUE when we find a hit
      if(*(GetAt(ii)) == crAssertReport)
        { return TRUE ; }
    }

  // else return FALSE
  return FALSE;

} // end SmAssertArray::Contains

/*******************************************************************//**
PURPOSE: rtn TRUE when tgtAssertReport is in Array, else return FALSE

NOTES: When rtn == TRUE, nFoundIndex is set to the index of the matching
                         report in this AssertArray
***********************************************************************/
SmBoolean SmAssertArray::FindReport
 (const void  * pReporter,   // in : this pointer of the AssertValid() method generating this report.
  ULONG         lTestIndex,  // in : canonical number of the AssertValid method test
  SM_TYPE       lTestClass,  // in : class type making the call - used to fetch AssertLabelList 
  const void  * pOwner,      // in : Owner ptr of the AssertTest generating this report.
  const void  * pOther,      // in : Optional Other object ptr, (good for bad gap reports)
  ULONG       * pOptFoundIndex) // out: Optional FoundIndx, When Rtn==TRUE index of found report in this SmAssertArray
 const                       
{ 
  ULONG ii ;

  // init output
  if(pOptFoundIndex) { *pOptFoundIndex = SM_UNDEF_ULONG ; }

  // for every report
  for(ii=0;ii<GetSize();ii++)
    { 
      // when it's the target
      if( (*this)[ii]->IsThisReport(pReporter, 
                                    lTestIndex,
                                    lTestClass,
                                    pOwner,    
                                    pOther))
        {
          // remember the index and return TRUE for found
          if(pOptFoundIndex) { *pOptFoundIndex = ii ; }
          return(TRUE) ;
        }
    } // end iter every report

  // arrive here when target is not in this AssertArray
  return(FALSE) ;

} // end SmAssertArray::FindReport

/*******************************************************************//**
PURPOSE: Add an element to the array only if it is not already in the
    array otherwise delete the element

NOTES: Unlike most methods this one can free memory of the input argument.
       This untraditional style is adopted here solely to allow
       the SM_ASSERT_VALID macro interface to run as a function call
       that returns a boolean.  This method is not meant for general use.

       Side Effect: sets m_bLastAdd: TRUE = added a new SmAssertReport to this array.
                                     FALSE= found and deleted a duplicate SmAssertReport.

RETURN --  TRUE  = SmAssertReport was added
           FALSE = SmAssertReport is already a part of the list, was deleted,
                   and was not added this time.
***********************************************************************/
SmBoolean SmAssertArray::AddUniqueDeleteDuplicate
  (SmAssertReport *pNewElement)
{
  // init state - place breakpoint here for debugging failed AssertValid calls <===
  m_bLastAdd = FALSE ;

  // when given NewElement
  if(pNewElement)
    {
      // when NewElement is duplicate - delete it, else - add it
      if(this->Contains(*pNewElement)) { delete pNewElement ; pNewElement = NULL ; }
      else                             { this->Add(pNewElement);
                                         m_bLastAdd = TRUE ;
                                       }
    }

  // all done
  return( pNewElement != NULL) ;  // only TRUE when new AssertReport added to array

} // end SmTArray<TYPE>::AddUniqueDeleteDuplicate

// obsolete
// /*******************************************************************//**
// PURPOSE: Call AssertHeal on every member SmAssertReport
// 
// RETURNS: TRUE when all problem reports are fixed.
//          FALSE when any problem report fails to be fixed.
// 
// NOTES: 1. The m_bOK value is checked for every member SmAssertReport.
//           When m_pData[ii]->m_bOK == FALSE, AssertHeal() is called for that report
// 
//        2. Database problems cascade and interact with one another.
//           Fixing one problem might fix others, expose old problems not yet reported,
//           or even make new problems.  After calling Heal() once, rerun
//           AssertValid to rebuild the SmAssertArray to get an accurate status
//           of the part.
// ***********************************************************************/
// SmBoolean SmAssertArray::Heal()
// {
//   // locals
//   ULONG ii ;
//   SmBoolean bRtn = TRUE ;
// 
//   // for every AssertReport
//   for(ii=0;ii<GetSize();ii++)
//     {
//       SmAssertReport * pAssertReport = m_pData[ii] ;
// 
//      // for failing AssertReport - try healing
//      if(pAssertReport->m_bOK == FALSE)
//        {
//          // map stale pointers to valid pointers
//          SmTArray<SmObject*> sReporterChanges ;
//          SmTArray<SmObject*> sOwnerChanges ;
//          SmTArray<SmObject*> sOtherChanges ;
// 
//          MapObject((SmObject *)pAssertReport->m_pReporter, sReporterChanges) ;
//          MapObject((SmObject *)pAssertReport->m_pOwner,    sOwnerChanges) ;
//          MapObject((SmObject *)pAssertReport->m_pOther,    sOtherChanges) ;
//          ULONG lReporterNum = sReporterChanges.GetSize() ;
//          ULONG lOwnerNum    = sOwnerChanges.GetSize() ;
//          // ULONG lOtherNum    = sOtherChanges.GetSize() ;
// 
//          // GWC: I don't have the logic of mapping object pointers worked out.
//          //      So here's my first shot.
//          //      1. I'm assuming I only need one valid reporter.
//          //         If there are any reporter changes - temporarily change
//          //         this AssertReport's m_pReporter value to the 1st valid mapped value.
//          //         Otherwise leave the reporter value as given.
//          //      2. The Assert Report needs to run once for every OwnerChange.
//          //         a. With no Owner changes - run AssertReport with Owner value as given.
//          //         b. With 1 Owner change - temp change the Owner - and run.
//          //         c. With multiple Owner changes - copy the AssertReport - change
//          //            the owner value, and then run heal.
//          //            In the future the copied AssertReports probably need to be
//          //            added to the pAList.  For today - we'll just delete them
//          //            after their run.
//          //      3. There is no good plan yet for binary tests where m_pOther is not NULL.
//          //         So for now do the same thing as for reporter changes
// 
//          void * pTmpReporter = lReporterNum > 0 ? sReporterChanges[0] : (void *)pAssertReport->m_pReporter ;
//          void * pTmpOwner    = lOwnerNum    > 0 ? sOwnerChanges[0]    : (void *)pAssertReport->m_pOwner ;
//          // void * pTmpOther    = lOtherNum    > 0 ? sOtherChanges[0]    : (void *)pAssertReport->m_pOther ;
// 
//          // Tmp change Reporter (this works both  with and without change cases)
//          SmTemporaryChangeValue<void *> sChange1(*(void **)&pAssertReport->m_pReporter, pTmpReporter) ;
// 
//          // Tmp change Owner (this is all that's needed for the no change and 1 change cases)
//          SmTemporaryChangeValue<void *> sChange2(*(void **)&pAssertReport->m_pOwner,    pTmpOwner) ;
// 
//          // Tmp change Other (this is all that's needed for the no change and 1 change cases, but certainly needs
//          //                   work for the multiple change case)
// 
//          // when there are multiple Owner Changes
//          if(sOwnerChanges.GetSize() >= 1)
//            {
//              // for every owner change but the first
//              for(ii=1;ii<sOwnerChanges.GetSize();ii++)
//                {
//                  // copy the AssertReport as local - no memory leaks
//                  SmAssertReport sCopyAssertReport(*pAssertReport) ;
// 
//                  // change the AssertReport.Owner
//                  sCopyAssertReport.m_pOwner = sOwnerChanges[ii] ;
// 
//                  // call heal on copied report for this owner
//                  bRtn &= sCopyAssertReport.AssertHeal(this) ;
// 
//                  // TODO?: May want to save result in pAList here for future reporting
//                } // end iter every Owner change but the first
//            } // end multiple Owner change check
// 
//          // Heal the original report - with temporary object changes as needed
//          bRtn &= pAssertReport->AssertHeal(this) ;
// 
//        } // end failing AssertReport check
//     } // end iter every AssertReport
// 
//   // all done
//   return(bRtn) ;
// 
// } // end SmAssertArray::Heal
// end obsolete

// obsolete
//  /*******************************************************************//**
//  PURPOSE: Log one Object was replaced by Another.
//           Called by Heal Action Functions.
//  
//  NOTES: add <pDelObj, pNewObj> to sObjMap List
//          changes all <pKey, pDelObj> pairs to <pKey, pNewObj>
//  ***********************************************************************/
//  void SmAssertArray::LogReplaceObject
//   (SmObject * pDelObj,  // in : Object being replaced
//    SmObject * pNewObj)  // in : by this object
//  {
//    // check input
//    if(pDelObj == NULL)
//      { return ; }
//  
//  #ifdef SM_DEBUG_CODE
//  SmBoolean bDebugMe = FALSE ;
//    if(bDebugMe)
//      {
//        sObjMap.Dump(TRUE) ; // TRUE = Dump object pairs
//      }
//  #endif // SM_DEBUG_CODE
//  
//    // update sObjMap
//    sObjMap.AddUnique(pDelObj, pNewObj) ;
//    sObjMap.ChangeValues(pDelObj, pNewObj) ;
//  
//  #ifdef SM_DEBUG_CODE
//    if(bDebugMe)
//      {
//        sObjMap.Dump(TRUE) ; // TRUE = Dump object pairs
//      }
//  #endif // SM_DEBUG_CODE
//  
//  } // end SmAssertArray::LogReplaceObject
//  
//  /*******************************************************************//**
//  PURPOSE: Log one Object was Split into a set of Other objects.
//           Called by Heal Action Functions.
//  
//  NOTES:  add <pParent, rChildren[ii]> pairs to sObjMap list
//            replace all <pKey, pParent> pairs with <pKey, rChildren[ii]> pairs
//  ***********************************************************************/
//  void SmAssertArray::LogSplitObject
//   (SmObject            * pParent,    // in : Parent   of [parent -> Split to Children]
//    SmTArray<SmObject*> & rChildren)  // in : Children of [parent -> Split to Children]
//  {
//    ULONG ii, jj ;
//    SmTArray<SmObject*> sKeys ;
//  
//    //check input
//    if(   pParent == NULL
//       || rChildren.GetSize() == 0)
//      { return ; }
//  
//    // update sObjMap
//  #ifdef SM_DEBUG_CODE
//  SmBoolean bDebugMe = FALSE ;
//    if(bDebugMe)
//      {
//        sObjMap.Dump(TRUE) ; // TRUE = Dump object pairs
//      }
//  #endif // SM_DEBUG_CODE
//  
//    // add <pParent, rChildren[ii]> pairs to sObjMap list
//    for(ii=0;ii<rChildren.GetSize();ii++)
//      { sObjMap.AddUnique(pParent, rChildren[ii]) ; }
//  
//    // replace all <pKey, pParent> pairs with <pKey, rChildren[ii]> pairs
//    sObjMap.GetKeysFor(pParent, sKeys) ;
//    for(ii=0;ii<sKeys.GetSize();ii++)
//      {
//        sObjMap.RemoveAssoc(sKeys[ii], pParent) ;
//        for(jj=0;jj<rChildren.GetSize();jj++)
//          { sObjMap.AddUnique(pParent, rChildren[jj]) ; }
//      }
//  
//  #ifdef SM_DEBUG_CODE
//    if(bDebugMe)
//      {
//        sObjMap.Dump(TRUE) ; // TRUE = Dump object pairs
//      }
//  #endif // SM_DEBUG_CODE
//  
//  } // end SmAssertArray::LogSplitObject
//  
//  /*******************************************************************//**
//  PURPOSE: Log two Objects were Merged into one.
//           Called by Heal Action Functions.
//  
//  NOTES: adds <pParent1, pCheck> and <pParent2, pChild> to sObjMap list
//           changes all <pKey, pParent1> pairs to <pKey, pChild>
//                   all <pKey, pParent2> pairs to <pKey, pChild>
//  ***********************************************************************/
//  void SmAssertArray::LogMergeObject
//   (SmObject * pParent1,   // in : Parent1 of [Parent1/Parent2 -> Child]
//    SmObject * pParent2,   // in : Parent2 of [Parent1/Parent2 -> Child]
//    SmObject * pChild)     // in : Child   of [Parent1/Parent2 -> Child]
//  {
//    ULONG ii ;
//    SmTArray<SmObject*> sKeys ;
//  
//    // update sObjMap
//  #ifdef SM_DEBUG_CODE
//  SmBoolean bDebugMe = FALSE ;
//    if(bDebugMe)
//      {
//        sObjMap.Dump(TRUE) ; // TRUE = Dump object pairs
//      }
//  #endif // SM_DEBUG_CODE
//  
//    // adds <pParent1, pCheck> and <pParent2, pChild> to sObjMap list
//    sObjMap.AddUnique(pParent1, pChild) ;
//    sObjMap.AddUnique(pParent2, pChild) ;
//  
//    // changes all <pKey, pParent1> pairs to <pKey, pChild>
//    //         all <pKey, pParent2> pairs to <pKey, pChild>
//    sObjMap.GetKeysFor(pParent1, sKeys) ;
//    for(ii=0;ii<sKeys.GetSize();ii++)
//      {
//        sObjMap.SetAt(sKeys[ii], pParent1) ;
//      }
//  
//    sObjMap.GetKeysFor(pParent2, sKeys) ;
//    for(ii=0;ii<sKeys.GetSize();ii++)
//      {
//        sObjMap.SetAt(sKeys[ii], pParent2) ;
//      }
//  
//  #ifdef SM_DEBUG_CODE
//    if(bDebugMe)
//      {
//        sObjMap.Dump(TRUE) ; // TRUE = Dump object pairs
//      }
//  #endif // SM_DEBUG_CODE
//  
//  } // end SmAssertArray::LogMergeObject
//  
//  /*******************************************************************//**
//  PURPOSE: Log one Object was deleted and is now a stale pointer.
//           Called by Heal Action Functions.
//  
//  NOTES: adds <pDelObj, NULL> to sObjMap List
//           changes all <pKey, pDelObj> pairs to <pKey, NULL>
//  ***********************************************************************/
//  void SmAssertArray::LogDeleteObject
//   (SmObject * pDelObj) // in : Deleted object to remember
//  {
//    ULONG ii ;
//    SmTArray<SmObject*> sKeys ;
//  
//    // update sObjMap
//  #ifdef SM_DEBUG_CODE
//  SmBoolean bDebugMe = FALSE ;
//    if(bDebugMe)
//      {
//        sObjMap.Dump(TRUE) ; // TRUE = Dump object pairs
//      }
//  #endif // SM_DEBUG_CODE
//  
//    // adds <pParent1, pCheck> and <pParent2, pChild> to sObjMap list
//    sObjMap.AddUnique(pDelObj, NULL) ;
//  
//    // changes all <pKey, pParent1> pairs to <pKey, pChild>
//    //         all <pKey, pParent2> pairs to <pKey, pChild>
//    sObjMap.GetKeysFor(pDelObj, sKeys) ;
//    for(ii=0;ii<sKeys.GetSize();ii++)
//      {
//        sObjMap.SetAt(sKeys[ii], NULL) ;
//      }
//  
//  #ifdef SM_DEBUG_CODE
//    if(bDebugMe)
//      {
//        sObjMap.Dump(TRUE) ; // TRUE = Dump object pairs
//      }
//  #endif // SM_DEBUG_CODE
//  
//  } // end SmAssertArray::LogDeleteObject
// end obsolete
  
/*******************************************************************//**
PURPOSE:
NOTES:
***********************************************************************/
SmBoolean SmAssertArray::IsKindOf( SM_TYPE t ) const
{
  return ((SmAssertArray_TYPE == t) ? TRUE : SmObject::IsKindOf( (t) ));
}

/*******************************************************************//**
PURPOSE: Pretty print class object

NOTES:
***********************************************************************/
void SmAssertArray::Dump()
 const
{
  // pass the call along
  Dump(TRUE,       // TRUE = write report for empty AssertArrays, FALSE=don't
      (TCHAR*)0,   // in : filename label for AssertArray report, NULL to ignore, default:[NULL]
       0,          // in : linenumber label for AssertArray report, 0 to ignore, default:[0]
       NULL,       // in : function label for AssertArray report, NULL to ignore, default:[NULL]
      (TCHAR*)0) ; // in : tested obj's class string, NULL to ignore, default:[NULL]

} // end SmAssertArray::Dump

/*******************************************************************//**
PURPOSE: Pretty print class object

NOTES: Writes a report for empty AssertArrays (no problems) when
       bWriteGoodReport == TRUE or
       global SmGetAssertArrayStream() == TRUE

       use SmSetAssertArrayStream().
***********************************************************************/
void SmAssertArray::Dump
 (SmBoolean           bWriteGoodReport, // in : TRUE = write report for empty AssertArrays, FALSE=don't
  const TCHAR * const file_name,        // in : filename label for AssertArray report, NULL to ignore, default:[NULL]
  ULONG               line_num,         // in : linenumber label for AssertArray report, 0 to ignore, default:[0]
  const TCHAR * const func_name,        // in : function label for AssertArray report, NULL to ignore, default:[NULL]
  const TCHAR * const class_string)     // in : tested obj's class string, NULL to ignore, default:[NULL]
 const
{
  SmBoolean sbEmptyMessage = FALSE;    //cbi: TRUE;
  if(   sbEmptyMessage == FALSE 
     && GetSize()      == 0)
    { return; }

  TCHAR sBuff[SM_TBLOCK_SIZE], sBuffForFile[SM_TBLOCK_SIZE];

  if(   bWriteGoodReport == TRUE
     || SmGetAssertValidStream() == TRUE
     || GetSize() > 0)
    {
      smos_sprintf(sBuff, _T("\nBegin %s%s%sSmAssertArray::Dump()"),
                 GetSize() == 0 ? _T("Empty ") : _T(""),
                 class_string ? class_string : _T(""),
                 class_string ? _T("::") : _T(""));
      smos_WriteBuffer(sBuff);

      // label
      smos_sprintf(sBuff       , _T("%s  SmAssertArray[0x%p] has %ld item%s"), GetSize() == 0 ? _T(",") : _T("\n"),
                                                                             this,
                                                                             GetSize(),
                                                                             GetSize() != 1 ? _T("s") : _T("") );
      smos_sprintf(sBuffForFile, _T("%s  SmAssertArray[%s] has %ld item%s"), GetSize() == 0 ? _T(",") : _T("\n"),
                                                                           _T("notNULL"),
                                                                           GetSize(),
                                                                           GetSize() != 1 ? _T("s") : _T("") );
      smos_WriteBuffer(sBuff, sBuffForFile);

#ifdef SM_DEBUG_CODE
      smos_sprintf(sBuff,        _T("\n  %s(%6ld): in:[%s()]"), file_name, line_num, func_name) ;
      smos_sprintf(sBuffForFile, _T("\n  %s( ): in:[%s()]"), file_name, func_name) ;
      smos_WriteBuffer(sBuff, sBuffForFile);
#else
      SM_REF3(file_name, line_num, func_name);
#endif // SM_DEBUG_CODE
    }

  // for every item
  ULONG ii ;

  size_t lReportTypeMax = 0 ;
  size_t lFileNameMax   = 0 ;

  // to align columns get FileName size
  for(ii=0;ii<GetSize();ii++)
    {
      size_t lCnt = SM_STRLEN(SM_GetObjTypeString(GetAt(ii)->m_lReportingType)) ;
      if(lCnt > lReportTypeMax) lReportTypeMax = lCnt ;

#ifdef SM_DEBUG_CODE
      lCnt = SM_STRLEN(GetAt(ii)->m_file_name) ;
      if(lCnt > lFileNameMax) lFileNameMax = lCnt ;
#endif // SM_DEBUG_CODE

    } // end iter all reports getting string field max lengths

  // for every report - output label and Dump
  for(ii=0;ii<GetSize();ii++)
    {
      SmAssertReport *pReport = GetAt(ii) ;

      smos_sprintf(sBuff, _T("\n  [%3ld]:"), ii) ;
      smos_sprintf(sBuffForFile, _T("%s"), _T("\n  [xxx]:") ) ;
      smos_WriteBuffer(sBuff, sBuffForFile);

      pReport->Dump(lReportTypeMax, lFileNameMax) ;

    }

  // all done
  if(   bWriteGoodReport == TRUE
     || SmGetAssertValidStream() == TRUE
     || GetSize() > 0)
    {
      smos_WriteBuffer(_T("\n")) ;

      smos_sprintf(sBuff, _T("%sEnd %s%s%sSmAssertArray::Dump()"),
                 GetSize() == 0 ? _T("  ") : _T(""),
                 GetSize() == 0 ? _T("Empty ") : _T(""),
                 class_string ? class_string : _T(""),
                 class_string ? _T("::") : _T(""));
      smos_WriteBuffer(sBuff);
    }

} // end SmAssertArray::Dump

/*******************************************************************//**
PURPOSE:  Add SmAssertReport graphics for global display Parameters
             to new DisplayList added to global DisplayList array.

NOTES:
***********************************************************************/
SmDisplayList * SmAssertArray::Draw
 (SmBoolean       bAddToUIPickList,  // in : TRUE = Add this PointSequence to UI pick interface for debugging
                                     //      default:[FALSE]
  SmGfxArraySet * pOptGfxSet)        // i/o: used for OpenGL ES 2.0. When given, accumulate GfxVertexArrays not GL calls.
                                     //      NULL to ignore. default:[NULL]
 const
{
  SmDisplayList *pRtn = NULL ;
#ifdef SM_GFX_CODE

  // GWC: currently adding           // when asked - add this SmAssertArray  to UI pick list
 //       AssertArray to PickList    if(bAddToUIPickList)
 //       has no side effects          { sm_GraphicsAddToBrepList(this) ; }

  // start new DisplayList (unless displayList is already open)
  SmVector3d sColor = smgfx_GetOutputColor(pOptGfxSet) ;
  smgfx_Open(smgfx_GetRuleColor(this), NULL, NULL, FALSE, pOptGfxSet);

  // for every AssertReport
  ULONG ii ;
  for(ii=0;ii<GetSize();ii++)
    {
      SmAssertReport * pAssertReport = this->GetAt(ii) ;

      // pass the call along to each SmAssertReport
      pAssertReport->Draw(bAddToUIPickList, pOptGfxSet) ;
    }

  // end display list
  smgfx_OutputColor(sColor, pOptGfxSet) ;
  pRtn = smgfx_Close(pOptGfxSet) ;

#else
  SM_REF2(bAddToUIPickList, pOptGfxSet);
#endif // SM_GFX_CODE
  return(pRtn) ;

} // end SmAssertArray::Draw

/*******************************************************************//**
PURPOSE:  Add SmAssertReport graphics for global display Parameters
             to new DisplayList added to global DisplayList array.

NOTES: Something more will have to be done for graphics
***************************************************************/
SmDisplayList * SmAssertReport::Draw
  (SmBoolean       bAddToUIPickList,  // in : bAddToUIPickList: TRUE = Add this PointSequence to UI pick interface for debugging
                                      //      default:[FALSE]
   SmGfxArraySet * pOptGfxSet)        // i/o: used for OpenGL ES 2.0. When given, accumulate GfxVertexArrays not GL calls.
                                      //      NULL to ignore. default:[NULL]
 const
{
  SmDisplayList *pRtn = NULL ;
#ifdef SM_GFX_CODE

  // nothing pickable in this class
  //      // when asked - add this SmAssertReport to UI pick list
  //      if(bAddToUIPickList)
  //        { sm_GraphicsAddToBrepList(this) ; }

  SmObject * pOwnerObj = GetOwnerObject() ;

  SmObject * pOtherObj = GetOtherObject() ;

  // start new DisplayList (unless displayList is already open)
  SmVector3d sColor = smgfx_GetOutputColor(pOptGfxSet) ;
  smgfx_Open(smgfx_GetRuleColor(), NULL, NULL, FALSE, pOptGfxSet);

  // add graphics here - perhaps a virtual function that goes to an array of derived objects.
  // For now let's just plot a point for all those reports that can generate a graphics point.
  // Another thing we might do is to render all those reports with m_pOwners that have draw functions.
  //   We could change color and size to highlight them.
  SmPoint3d sGraphicsPoint ;
  if(GetGraphicsPoint(sGraphicsPoint))
    {
      // output the point
      smgfx_OutputPoint(sGraphicsPoint.x, sGraphicsPoint.y, sGraphicsPoint.z, pOptGfxSet) ;
    }

  // Owner Graphics
  if(pOwnerObj)
    {
      if     (pOwnerObj->IsKindOf(SmCurve_TYPE  ))
        { smgfx_SetLook(3,4, 1,0,0, pOptGfxSet) ; ((SmCurve   *)pOwnerObj)->DrawParams(NULL, pOptGfxSet) ;

          SmCurve   *pCurve    = (SmCurve   *)pOwnerObj ;
          SmObject  *pObj      = pCurve->GetOwner() ;
          SmEdge    *pEdge     = SM_CAST_PTR(SmEdge, pObj) ;
// Remove Composites
// SmCEdge   *pCEdge    = SM_CAST_PTR(SmCEdge, pObj) ;
          SmFace    *pFace     = SM_CAST_PTR(SmFace, pObj) ;
          SmEdgeuse *pEdgeuse  = SM_CAST_PTR(SmEdgeuse, pObj) ;
          SmSurface *pSurface  = pEdgeuse && pEdgeuse->GetFace() ? pEdgeuse->GetFace()->GetSurface() : NULL ;
          SmBrep    *pBrep =   pEdge    ? pEdge->GetBrep()
// Remove Composites
//                              : pCEdge   ? pCEdge->GetBrep()
                             : pFace    ? pFace->GetBrep()
                             : pEdgeuse ? pEdgeuse->GetBrep()
                             : NULL ;
          // double dFailedParam = m_sOwnerParam.x ;

          smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; // SM_GAPHICSLOOP() ;
          if(pCurve->GetDim() == 3)
            { smgfx_SetLook(3,4, 1,0,0) ; pCurve->Draw() ; // SM_GAPHICSLOOP() ;
              smgfx_SetLook(3,4, 1,0,0) ; pCurve->DrawParams() ; // SM_GAPHICSLOOP() ;
              smgfx_SetLook(7,8, 1,0,0) ; if(m_sOwnerParam.x != SM_UNDEF_DOUBLE) pCurve->DrawAt(m_sOwnerParam.x) ; // SM_GAPHICSLOOP() ;
            }
          else
            { if(pEdgeuse && pSurface)
                { SmCrvOnSurf sCrvOnSurf((SmCurve &)*pCurve, *pSurface) ;
                  smgfx_SetLook(3,4, 1,0,0) ; sCrvOnSurf.Draw() ; // SM_GAPHICSLOOP() ;
                  smgfx_SetLook(3,4, 1,0,0) ; sCrvOnSurf.DrawParams() ; // SM_GAPHICSLOOP() ;
                  smgfx_SetLook(7,8, 1,0,0) ; if(m_sOwnerParam.x != SM_UNDEF_DOUBLE) sCrvOnSurf.DrawAt(m_sOwnerParam.x) ; // SM_GAPHICSLOOP() ;
                }
            }
          smgfx_SetLook(5,6, 1,0,1) ; if(pEdge) pEdge->Draw() ; // SM_GAPHICSLOOP() ;
          smgfx_SetLook(1,2, 0,1,1) ; if(pSurface) pSurface->DrawUV() ; // SM_GAPHICSLOOP() ;
          smgfx_SetLook(1,2, 0,0,0) ; if(pFace) pFace->Draw(SM_DM_CROSSHATCH) ; // SM_GAPHICSLOOP() ;
          // SM_GAPHICSLOOP() ;

        }
      else if(pOwnerObj->IsKindOf(SmSurface_TYPE)) { smgfx_SetLook(2,3, 1,0,0, pOptGfxSet) ; ((SmSurface *)pOwnerObj)->DrawUV(8,8,FALSE,NULL,bAddToUIPickList,pOptGfxSet) ; }
      else if(pOwnerObj->IsKindOf(SmFace_TYPE   )) { smgfx_SetLook(2,3, 1,0,0, pOptGfxSet) ; ((SmFace    *)pOwnerObj)->DrawUV(7,7,FALSE,NULL,bAddToUIPickList,pOptGfxSet) ; }
      else if(pOwnerObj->IsKindOf(SmEdge_TYPE   )) { smgfx_SetLook(3,4, 1,0,0, pOptGfxSet) ; ((SmEdge    *)pOwnerObj)->Draw(pOptGfxSet) ; }
      else if(pOwnerObj->IsKindOf(SmVertex_TYPE )) { smgfx_SetLook(3,4, 1,0,0, pOptGfxSet) ; ((SmVertex  *)pOwnerObj)->Draw(pOptGfxSet) ; }
      else if(pOwnerObj->IsKindOf(SmEdgeuse_TYPE)) { smgfx_SetLook(2,3, 1,0,0, pOptGfxSet) ; ((SmEdgeuse *)pOwnerObj)->DrawEdgeFaceTrimCurveGap(1.0, pOptGfxSet) ; }
      else if(pOwnerObj->IsKindOf(SmLoop_TYPE   )) { smgfx_SetLook(3,4, 1,0,0, pOptGfxSet) ; ((SmLoop    *)pOwnerObj)->Draw(3,FALSE,NULL,FALSE, pOptGfxSet) ; }
      else if(pOwnerObj->IsKindOf(SmLoopuse_TYPE)) { smgfx_SetLook(2,3, 1,0,0, pOptGfxSet) ; ((SmLoopuse *)pOwnerObj)->Draw(3, FALSE, NULL, FALSE, pOptGfxSet) ; }   // FALSE = don't draw UVTrimCurves
    }

  // Other Graphics
  if(pOtherObj)
    {
      if     (pOtherObj->IsKindOf(SmCurve_TYPE  )) { smgfx_SetLook(3,4, 1,0,1, pOptGfxSet) ; ((SmCurve   *)pOtherObj)->DrawParams(NULL, pOptGfxSet) ; }
      else if(pOtherObj->IsKindOf(SmSurface_TYPE)) { smgfx_SetLook(2,3, 1,0,1, pOptGfxSet) ; ((SmSurface *)pOtherObj)->DrawUV(8,8,FALSE,NULL,bAddToUIPickList,pOptGfxSet) ; }
      else if(pOtherObj->IsKindOf(SmFace_TYPE   )) { smgfx_SetLook(2,3, 1,0,1, pOptGfxSet) ; ((SmFace    *)pOtherObj)->DrawUV(7,7,FALSE,NULL,bAddToUIPickList,pOptGfxSet) ; }
      else if(pOtherObj->IsKindOf(SmEdge_TYPE   )) { smgfx_SetLook(3,4, 1,0,1, pOptGfxSet) ; ((SmEdge    *)pOtherObj)->Draw(pOptGfxSet) ; }
      else if(pOtherObj->IsKindOf(SmVertex_TYPE )) { smgfx_SetLook(3,4, 1,0,1, pOptGfxSet) ; ((SmVertex  *)pOtherObj)->Draw(pOptGfxSet) ; }
      else if(pOtherObj->IsKindOf(SmEdgeuse_TYPE)) { smgfx_SetLook(2,3, 1,0,1, pOptGfxSet) ; ((SmEdgeuse *)pOtherObj)->DrawEdgeFaceTrimCurveGap(1.0, pOptGfxSet) ; }
      else if(pOwnerObj->IsKindOf(SmLoop_TYPE   )) { smgfx_SetLook(3,4, 1,0,0, pOptGfxSet) ; ((SmLoop    *)pOwnerObj)->Draw(3,FALSE,NULL,FALSE, pOptGfxSet) ; }
      else if(pOwnerObj->IsKindOf(SmLoopuse_TYPE)) { smgfx_SetLook(2,3, 1,0,0, pOptGfxSet) ; ((SmLoopuse *)pOwnerObj)->Draw(3, FALSE, NULL, FALSE, pOptGfxSet) ; }   // FALSE = don't draw UVTrimCurves
    }

  // end display list
  smgfx_OutputColor(sColor, pOptGfxSet) ;
  pRtn = smgfx_Close(pOptGfxSet) ;

#else
  SM_REF2(bAddToUIPickList, pOptGfxSet);
#endif // SM_GFX_CODE
  return(pRtn) ;

} // end SmAssertReport::Draw

// obsolete
// /*******************************************************************//**
// PURPOSE: helper function for SM_ASSERT_NUM_TEST(S) MACRO set to support
//          sending a variable number of specified tests to the method
//          sm_AssertValid() while allowing the supported macros to continue
//          evaluating to a boolean TRUE/FALSE value.
// 
// NOTES: Access this method only through the macros
//          SM_ASSERT_1_TEST(a,  type, t1)
//          SM_ASSERT_2_TESTS(a, type, t1, t2)
//          SM_ASSERT_3_TESTS(a, type, t1, t2, t3)
//          SM_ASSERT_4_TESTS(a, type, t1, t2, t3, t4)
//          . . .
// ***********************************************************************/
// SmBoolean sm_AssertTests
//  (SmObject          * pObj,       // in : pObject to test
//   SM_TYPE             lExeClassType,      // in : class type owner of AssertValid tests to run
//   const TCHAR * const file_name,  // in : filename label for AssertArray report
//   ULONG               line_num,   // in : linenumber label for AssertArray report
//   const TCHAR * const func_name,  // in : function label for AssertArray report
//   ULONG               lNum,       // in : Number of test listed next
//  ... )                            // in : list AssertValid test numbers to run, length:[lNum]
// {
//   // variable argument declaration
//   va_list vl ;
//   va_start( vl, lNum) ;
// 
//   // gather the variable length number of arguments into an SmTArray
//   ULONG ii ;
//   SmTArray<ULONG> sTestRequests ;
// 
//   // step through the list
//   for(ii=0;ii<lNum;ii++)
//     {
//       ULONG lTest = va_arg(vl, ULONG) ;
//       sTestRequests.Add(lTest) ;
// 
//     } // end iter variagble length arguments
// 
//   // make the AssertValid call -
//   SmBoolean bRtn = sm_AssertValid(pObj,            // in : Object to test
//                                   SM_NO_OBJDUMP,   // in : don't call a->Dump()
//                                   SM_NO_STREAM,    // in : only dump AssertArray report for failed AssertValid checks
//                                   SM_LEVEL_GIVEN,  // in : only execute AssertValid test number checks listed in pTestRequests input array
//                                   lExeClassType,           // in : class type owner of AssertValid tests to run - only used when Level == SM_LEVEL_GIVEN
//                                   &sTestRequests,  // in : SmTArray<ULONG> of test numbers to execute - only used when Level == SM_LEVEL_GIVEN
//                                   SM_NO_WALK,      // in : call AssertValid on target only
//                                   FALSE,           // in : no Graphics
//                                   file_name,       // in : linenumber label for AssertArray report
//                                   line_num,        // in : function label for AssertArray report
//                                   func_name) ;     // in : Number of test listed next
//   // all done
//   va_end( vl ) ;
//   return(bRtn) ;
// 
// } // end sm_AssertTests
// end obsolete

/*******************************************************************//**
PURPOSE:  In SM_DEBUG_CODE mode return
            TRUE  = pObj->AssertValid() passes.
            FALSE = pObj->AssertValid() fails.
          In release mode - always returns TRUE.

NOTES: ASSERT_VALID calls were originally macros because
        1. they would only appear in SM_DEBUG_CODE builds and
        2. many unrelated classes supported the call (a)->AssertValid().

       The ASSERT VALID MACROS were becoming awkward as more and more
       arguments were being added to control their behavior.

       All the AssertValid() call signatures were made uniform and
       organized under a single SmObject base class virtual function.

       The MACROS were rebuilt as functions but the old UPPER_CASE
       names left alone for backward compatibility
***************************************************************/
SmBoolean sm_AssertValid
 (const SmObject    * pObj,             // in : Target Object to check
  SmAssertObjDump     eDumpObjFlag,     // in : SM_NO_OBJDUMP     = never Dump Target Object
                                        //      SM_OBJDUMP_BEFORE = Dump Target Object before AssertArray report
                                        //      SM_OBJDUMP_AFTER  = Dump Target Object after  AssertArray report
                                        //      SM_OBJDUMP_ONLY   = Only Dump Target Object skip AssertArray report
                                        //      default:[SM_NO_OBJDUMP]
  SmAssertStream      eStreamFlag,          // in : SM_STREAM       = always dump AssertArray report
                                        //      SM_NO_STREAM    = only dump AssertArray report for failed AssertValid checks
                                        //      SM_GLOBAL_STREAM= Let global SM_bAssertValidStream value pick streaming
                                        //      default:[SM_GLOBAL_STREAM]
  SmAssertTestLevel   eLevelFlag,           // in : SM_LEVEL_0,    = do SM_LEVEL_O labeled Assert Checks                      (fastest)
                                        //      SM_LEVEL_1,    = do SM_LEVEL_0 and SM_LEVEL_1 labeled Assert Checks       (inbetween)
                                        //      SM_LEVEL_2     = do SM_LEVEL_0, SM_LEVEL_1, and SM_LEVEL_2 labeled Checks (most complete)
                                        //      SM_LEVEL_GIVEN = run tests in order requested in pTestRequests
                                        //      default:[SM_LEVEL_2]
  SM_TYPE             lExeClassType,            // in : ClassType of ObjClass of ObjClass::AssertValid() method to execute - used to navigate virtual stacks - only used when Level == SM_LEVEL_GIVEN
  SmTArray<ULONG>   * pTestRequests,    // in : when eTestLevel == SM_LEVEL_GIVEN, run these tests in this order, default:[NULL]
  SmAssertWalking     eWalkFlag,            // in : SM_WALK    = run AssertValid on target and any topology graph descendants
                                        //      SM_NO_WALK = run AssertValid only on target
                                        //      default:[SM_WALK]
  SmBoolean           bDrawFlag,            // in : TRUE = Add Graphics to open Stream for failing AssertReports
                                        //      FALSE= No Graphics
                                        //      default:[FALSE]
  // SmBoolean           bHeal,            // in : TRUE = Call AssertHeal on all failed AssertReports
  //                                       //      FALSE= No Healing
  //                                       //      default:[FALSE]
  const TCHAR * const file_name,        // in : filename label for AssertArray report
  ULONG               line_num,         // in : linenumber label for AssertArray report
  const TCHAR * const func_name)        // in : function label for AssertArray report
{
  // no work - no object
  if(pObj == NULL)
    { return(TRUE) ; }

  // take action only in Debug mode!
#ifndef SM_DEBUG_CODE
  SM_REF5(file_name, line_num, func_name, bDrawFlag, eWalkFlag) ; 
  SM_REF5(pTestRequests, lExeClassType, eLevelFlag, eStreamFlag, eDumpObjFlag) ;
  return(TRUE) ;

#else  // SM_DEBUG_CODE

  // return value
  SmBoolean bRtn = TRUE; // TRUE  = pObj okay
                         // FALSE = pObj has problems

  // locals
  static constexpr SmBoolean bThroughAssert = FALSE ; // set to false to stop hitting ErrorMessage breaks due to problem AssertReports
  SmAssertArray    sAList, * pAList = &sAList ;
  SmBoolean        bDump =   (   eStreamFlag == SM_STREAM)            //    when asked - also dump AssertValid reports with no failures
                          || (   eStreamFlag == SM_GLOBAL_STREAM      // or when checking global state and global state asks
                              && SM_bAssertValidStream == TRUE) ; //         - also dump no Failure cases

  // when asked - Dump A before AssertArray report
  if(   eDumpObjFlag == SM_OBJDUMP_BEFORE
     || eDumpObjFlag == SM_OBJDUMP_ONLY)
    { if(pObj->IsKindOf(SmBrep_TYPE))
        { 
          SmBrepDumpType eDumpType = SM_BD_GEOM_TYPES ; // SM_BD_BASE_ONLY  = output Brep summary data only
                                                        // SM_BD_MAX_GAPS   = output Brep Max Edge/UVTrimCurve, Vertex/Edge, and Vertex/Face gaps
                                                        // SM_BD_GEOM_TYPES = output Surface and Curve Type counts
                                                        // SM_BD_POINTS     = output vertex point location and nearest neighbor information
                                                        // SM_BD_CURVES     = output vertex and edge curve descriptions
                                                        // SM_BD_SURFACES   = output vertex, edge, and face surface descriptions
          ((SmBrep *)pObj)->Dump(eDumpType) ; 
        }
      else
        { pObj->Dump() ; }

      if(eDumpObjFlag == SM_OBJDUMP_ONLY)
        { return TRUE ; }
    }

  // Target->AssertValid
  if(eLevelFlag == SM_LEVEL_GIVEN)
    { bRtn = (  lExeClassType == SmVertex_TYPE         ? ((SmVertex *)pObj)->SmVertex::AssertValid(pAList, eLevelFlag, SM_NO_WALK, pTestRequests)
              : lExeClassType == SmEdge_TYPE           ? ((SmEdge *)pObj)->SmEdge::AssertValid(pAList, eLevelFlag, SM_NO_WALK, pTestRequests)
              : lExeClassType == SmCurve_TYPE          ? ((SmCurve *)pObj)->SmCurve::AssertValid(pAList, eLevelFlag, SM_NO_WALK, pTestRequests)
              : lExeClassType == SmBSplineCurve_TYPE   ? ((SmBSplineCurve *)pObj)->SmBSplineCurve::AssertValid(pAList, eLevelFlag, SM_NO_WALK, pTestRequests)
              : lExeClassType == SmFace_TYPE           ? ((SmFace *)pObj)->SmFace::AssertValid(pAList, eLevelFlag, SM_NO_WALK, pTestRequests)
              : lExeClassType == SmSurface_TYPE        ? ((SmSurface *)pObj)->SmSurface::AssertValid(pAList, eLevelFlag, SM_NO_WALK, pTestRequests)
              : lExeClassType == SmBSplineSurface_TYPE ? ((SmBSplineSurface *)pObj)->SmBSplineSurface::AssertValid(pAList, eLevelFlag, SM_NO_WALK, pTestRequests)
              : lExeClassType == SmOffsetSurface_TYPE  ? ((SmOffsetSurface *)pObj)->SmOffsetSurface::AssertValid(pAList, eLevelFlag, SM_NO_WALK, pTestRequests)
              : TRUE
             ) ;
    }
  else
    {
      bRtn = pObj->AssertValid(pAList, eLevelFlag, eWalkFlag, pTestRequests) ;
    }

  // when asked and with errors - Signal errors
  if (!bRtn && bThroughAssert)
    {
      TCHAR sBuff[SM_TBLOCK_SIZE], sBuffForFile[SM_TBLOCK_SIZE] ;

      smos_sprintf(sBuff       , _T("Object [%s=0x%p] Failed AssertValid Check "), pObj->GetClassString(), pObj) ;
      smos_sprintf(sBuffForFile, _T("Object [%s] Failed AssertValid Check "), pObj->GetClassString()) ;
      smos_AssertErrorMessage(SM_ERR_ASSERTVALID_FAILURE,FILE_NAME,LINE_NUMBER,sBuff,sBuffForFile,FALSE,func_name) ;
    }

  // when appropriate or asked - Dump AssertArray
  if(   (bRtn == FALSE)                  //    always dump AssertValid reports with failures
     || (bDump) )                        // or when asked - dump AssertValid reports with no failures
    {
      // obsolete
      // // arrive here only when an AssertValid report is requested
      // if(bHeal)
      //   {
      //     smos_WriteBuffer(_T("\n\nAssertValid List - Before Heal:")) ;
      //   }
      pAList->Dump(TRUE, file_name, line_num, func_name, pObj->GetClassString()) ;
    }

  // when asked - Draw problem objects
  if(bDrawFlag)
    {
      pAList->Draw(TRUE) ; sm_GraphicsLoop();
      sm_GraphicsLoop();
    }

  // when asked - Dump A after AssertArray report
  if(eDumpObjFlag == SM_OBJDUMP_AFTER)
    { if(pObj->IsKindOf(SmBrep_TYPE))
        { 
          SmBrepDumpType eDumpType = SM_BD_GEOM_TYPES ; // SM_BD_BASE_ONLY  = output Brep summary data only
                                                        // SM_BD_MAX_GAPS   = output Brep Max Edge/UVTrimCurve, Vertex/Edge, and Vertex/Face gaps
                                                        // SM_BD_GEOM_TYPES = output Surface and Curve Type counts
                                                        // SM_BD_POINTS     = output vertex point location and nearest neighbor information
                                                        // SM_BD_CURVES     = output vertex and edge curve descriptions
                                                        // SM_BD_SURFACES   = output vertex, edge, and face surface descriptions
          ((SmBrep *)pObj)->Dump(eDumpType) ; 
        }
      else
        { pObj->Dump() ; }
    }

  // obsolete
  // // when asked - try healing AssertArray failed reports
  // if(bHeal && pAList->GetSize() > 0)
  //   {
  //     bRtn = pAList->Heal() ;
  // 
  //     // when appropriate or asked - Dump AssertArray
  //     if(   (bRtn == FALSE)                  //    always dump AssertValid reports with failures
  //        || (bDump) )                        // or when asked - dump AssertValid reports with no failures
  //       {
  //         smos_WriteBuffer(_T("\n\nAssertValid List - Heal Results:")) ;
  // 
  //         // arrive here only when an AssertValid report is requested
  //         pAList->Dump(TRUE, file_name, line_num, func_name, pObj->GetClassString()) ;
  //       }
  // 
  //     // rerun Target->AssertValid
  //     pAList->ReSet() ;
  //     // Target->AssertValid
  //     if(eLevelFlag == SM_LEVEL_GIVEN || eLevelFlag == SM_LEVEL_HEAL)
  //       { bRtn = (  lExeClassType == SmVertex_TYPE         ? ((SmVertex *)pObj)->SmVertex::AssertValid(pAList, eLevelFlag, SM_NO_WALK, pTestRequests)
  //                 : lExeClassType == SmEdge_TYPE           ? ((SmEdge *)pObj)->SmEdge::AssertValid(pAList, eLevelFlag, SM_NO_WALK, pTestRequests)
  //                 : lExeClassType == SmCurve_TYPE          ? ((SmCurve *)pObj)->SmCurve::AssertValid(pAList, eLevelFlag, SM_NO_WALK, pTestRequests)
  //                 : lExeClassType == SmBSplineCurve_TYPE   ? ((SmBSplineCurve *)pObj)->SmBSplineCurve::AssertValid(pAList, eLevelFlag, SM_NO_WALK, pTestRequests)
  //                 : lExeClassType == SmFace_TYPE           ? ((SmFace *)pObj)->SmFace::AssertValid(pAList, eLevelFlag, SM_NO_WALK, pTestRequests)
  //                 : lExeClassType == SmSurface_TYPE        ? ((SmSurface *)pObj)->SmSurface::AssertValid(pAList, eLevelFlag, SM_NO_WALK, pTestRequests)
  //                 : lExeClassType == SmBSplineSurface_TYPE ? ((SmBSplineSurface *)pObj)->SmBSplineSurface::AssertValid(pAList, eLevelFlag, SM_NO_WALK, pTestRequests)
  //                 : TRUE
  //                ) ;
  //       }
  //     else
  //       {
  //         bRtn = pObj->AssertValid(pAList, eLevelFlag, eWalkFlag, pTestRequests) ;
  //       }
  // 
  //     // Dump AssertArray after AssertHeal
  //     smos_WriteBuffer(_T("\n\nRefreshed AssertValid List - After Heal:")) ;
  //     pAList->Dump(TRUE, file_name, line_num, func_name, pObj->GetClassString()) ;
  //   }
  // end obsolete
  
  // all done
  return(bRtn) ;

#endif // SM_DEBUG_CODE
} // end sm_AssertValid(SmObject *, args)

/*******************************************************************//**
PURPOSE: SM_ASSERT_VALID for class SmAxis2Placement
NOTES:
***************************************************************/
SmBoolean sm_AssertValid
 (const SmAxis2Placement * pObj,           // in :
  SmAssertObjDump          eDumpObjFlag,   // in :
  SmAssertStream           eStreamFlag,    // in :
  SmAssertTestLevel        eLevelFlag,     // in :
  SM_TYPE                  lExeClassType,  // NotUsed: in :
  SmTArray<ULONG>        * pTestRequests,  // NotUsed: in :
  SmAssertWalking          eWalkFlag,      // in :
  SmBoolean                bDrawFlag,      // in :
  // SmBoolean                bHeal,          // in :
  const TCHAR * const      file_name,      // in :
  ULONG                    line_num,       // in :
  const TCHAR * const      func_name)      // in :
{
  SM_REF7(lExeClassType, pTestRequests, func_name, line_num, file_name, bDrawFlag, eWalkFlag);
  SM_REF4(pObj, eDumpObjFlag, eStreamFlag, eLevelFlag);

  SmBoolean bRtn = TRUE ;
#ifdef SM_DEBUG_CODE
  SmAssertArray sAList, * pAList = &sAList ;
  SmBoolean     bDump = (eStreamFlag == SM_STREAM) || (eStreamFlag == SM_GLOBAL_STREAM && SM_bAssertValidStream == TRUE) ;
  bRtn = pObj != NULL ? pObj->AssertValid(pAList, eLevelFlag, eWalkFlag) : TRUE ;
  if(eDumpObjFlag == SM_OBJDUMP_BEFORE) { pObj->Dump() ; }
  if(eDumpObjFlag == SM_OBJDUMP_ONLY)   { pObj->Dump() ; return TRUE ; }
  if (!bRtn) { TCHAR sBuff[SM_TBLOCK_SIZE], sBuffForFile[SM_TBLOCK_SIZE] ;
               smos_sprintf(sBuff       , _T("Object [%s=0x%p] Failed AssertValid Check "), pObj->GetClassString(), pObj) ;
               smos_sprintf(sBuffForFile, _T("Object [%s] Failed AssertValid Check "), pObj->GetClassString()) ;
               smos_AssertErrorMessage(SM_ERR_ASSERTVALID_FAILURE,FILE_NAME,LINE_NUMBER,sBuff,sBuffForFile,FALSE,func_name) ;
             }
  if((bRtn == FALSE) || bDump) { // if(bHeal) { smos_WriteBuffer(_T("\n\nAssertValid List - Before Heal:")) ; }
                                 pAList->Dump(TRUE, file_name, line_num, func_name, pObj->GetClassString()) ;
                               }
  if(eDumpObjFlag == SM_OBJDUMP_AFTER) { pObj->Dump() ; }
  if(bDrawFlag) { pObj->Draw() ; }
  // obsolete
  // if(bHeal) { bRtn = pAList->Heal() ;
  //             if((bRtn == FALSE) || (bDump)) { smos_WriteBuffer(_T("\n\nAssertValid List - with Heal Results:")) ;
  //                                              pAList->Dump(TRUE, file_name, line_num, func_name, pObj->GetClassString()) ;
  //                                            }
  //             pAList->ReSet() ;
  //             bRtn = pObj->AssertValid(pAList, eLevelFlag, eWalkFlag) ;
  //             smos_WriteBuffer(_T("\n\nRefreshed AssertValid List - After Heal:")) ;
  //             pAList->Dump(TRUE, file_name, line_num, func_name, pObj->GetClassString()) ;
  //           }
  
#endif // SM_DEBUG_CODE
  return(bRtn) ;
} // end sm_AssertValid(SmAxis2Placement*, args)

/*******************************************************************//**
PURPOSE: SM_ASSERT_VALID for class SmPseudoBox
NOTES:
***************************************************************/
SmBoolean sm_AssertValid
 (const SmPseudoBox      * pObj,           // in :
  SmAssertObjDump          eDumpObjFlag,   // in :
  SmAssertStream           eStreamFlag,    // in :
  SmAssertTestLevel        eLevelFlag,     // in :
  SM_TYPE                  lExeClassType,  // NotUsed: in :
  SmTArray<ULONG>        * pTestRequests,  // NotUsed: in :
  SmAssertWalking          eWalkFlag,      // in :
  SmBoolean                bDrawFlag,      // in :
  // SmBoolean                bHeal,          // in :
  const TCHAR * const      file_name,      // in :
  ULONG                    line_num,       // in :
  const TCHAR * const      func_name)      // in :
{
  SM_REF2(lExeClassType, pTestRequests) ;
  SM_REF9(pObj, eDumpObjFlag, eStreamFlag, eLevelFlag, eWalkFlag, bDrawFlag, file_name, line_num, func_name);

  SmBoolean bRtn = TRUE ;
#ifdef SM_DEBUG_CODE
  SmAssertArray sAList, * pAList = &sAList ;
  SmBoolean     bDump = (eStreamFlag == SM_STREAM) || (eStreamFlag == SM_GLOBAL_STREAM && SM_bAssertValidStream == TRUE) ;
  bRtn = pObj != NULL ? pObj->AssertValid(pAList, eLevelFlag, eWalkFlag) : TRUE ;
  if(eDumpObjFlag == SM_OBJDUMP_BEFORE) { pObj->Dump() ; }
  if(eDumpObjFlag == SM_OBJDUMP_ONLY)   { pObj->Dump() ; return TRUE ; }
  if (!bRtn) { TCHAR sBuff[SM_TBLOCK_SIZE], sBuffForFile[SM_TBLOCK_SIZE] ;
               smos_sprintf(sBuff       , _T("Object [%s=0x%p] Failed AssertValid Check "), pObj->GetClassString(), pObj) ;
               smos_sprintf(sBuffForFile, _T("Object [%s] Failed AssertValid Check "), pObj->GetClassString()) ;
               smos_AssertErrorMessage(SM_ERR_ASSERTVALID_FAILURE,FILE_NAME,LINE_NUMBER,sBuff,sBuffForFile,FALSE,func_name) ;
             }
  if((bRtn == FALSE) || bDump) { // if(bHeal) { smos_WriteBuffer(_T("\n\nAssertValid List - Before Heal:")) ; }
                                 pAList->Dump(TRUE, file_name, line_num, func_name, pObj->GetClassString()) ;
                               }
  if(eDumpObjFlag == SM_OBJDUMP_AFTER) { pObj->Dump() ; }
  if(bDrawFlag) { pObj->Draw() ; }
  // obsolete
  // if(bHeal) { bRtn = pAList->Heal() ;
  //             if((bRtn == FALSE) || (bDump)) { smos_WriteBuffer(_T("\n\nAssertValid List - with Heal Results:")) ;
  //                                              pAList->Dump(TRUE, file_name, line_num, func_name, pObj->GetClassString()) ;
  //                                            }
  //             pAList->ReSet() ;
  //             bRtn = pObj->AssertValid(pAList, eLevelFlag, eWalkFlag) ;
  //             smos_WriteBuffer(_T("\n\nRefreshed AssertValid List - After Heal:")) ;
  //             pAList->Dump(TRUE, file_name, line_num, func_name, pObj->GetClassString()) ;
  //           }
  
#endif // SM_DEBUG_CODE
  return(bRtn) ;
} // end sm_AssertValid(SmPseudoBox*, args)

/*******************************************************************//**
PURPOSE: SM_ASSERT_VALID for class SmPeriodicExtent1d
NOTES:
***************************************************************/
SmBoolean sm_AssertValid
 (const SmPeriodicExtent1d * pObj,           // in :
  SmAssertObjDump            eDumpObjFlag,   // in :
  SmAssertStream             eStreamFlag,    // in :
  SmAssertTestLevel          eLevelFlag,     // in :
  SM_TYPE                    lExeClassType,  // NotUsed: in :
  SmTArray<ULONG>          * pTestRequests,  // NotUsed: in :
  SmAssertWalking            eWalkFlag,      // in :
  SmBoolean                  bDrawFlag,      // NotUsed: in :
  // SmBoolean                  bHeal,          // in :
  const TCHAR * const        file_name,      // in :
  ULONG                      line_num,       // in :
  const TCHAR * const        func_name)      // in :
{
  SM_REF3(lExeClassType, pTestRequests, bDrawFlag) ;
  SM_REF8(pObj, eDumpObjFlag, eStreamFlag, eLevelFlag, eWalkFlag, file_name, line_num, func_name);

  SmBoolean bRtn = TRUE ;
#ifdef SM_DEBUG_CODE
  SmAssertArray sAList, * pAList = &sAList ;
  SmBoolean     bDump = (eStreamFlag == SM_STREAM) || (eStreamFlag == SM_GLOBAL_STREAM && SM_bAssertValidStream == TRUE) ;
  bRtn = pObj != NULL ? pObj->AssertValid(pAList, eLevelFlag, eWalkFlag) : TRUE ;
  if(eDumpObjFlag == SM_OBJDUMP_BEFORE) { pObj->Dump() ; }
  if(eDumpObjFlag == SM_OBJDUMP_ONLY)   { pObj->Dump() ; return TRUE ; }
  if (!bRtn) { TCHAR sBuff[SM_TBLOCK_SIZE], sBuffForFile[SM_TBLOCK_SIZE] ;
               smos_sprintf(sBuff       , _T("Object [%s=0x%p] Failed AssertValid Check "), pObj->GetClassString(), pObj) ;
               smos_sprintf(sBuffForFile, _T("Object [%s] Failed AssertValid Check "), pObj->GetClassString()) ;
               smos_AssertErrorMessage(SM_ERR_ASSERTVALID_FAILURE,FILE_NAME,LINE_NUMBER,sBuff,sBuffForFile,FALSE,func_name) ;
             }
  if((bRtn == FALSE) || bDump) { // if(bHeal) { smos_WriteBuffer(_T("\n\nAssertValid List - Before Heal:")) ; }
                                 pAList->Dump(TRUE, file_name, line_num, func_name, pObj->GetClassString()) ;
                               }
  if(eDumpObjFlag == SM_OBJDUMP_AFTER) { pObj->Dump() ; }
  // if(bDrawFlag) { pObj->Draw() ; }
  // obsolete
  // if(bHeal) { bRtn = pAList->Heal() ;
  //             if((bRtn == FALSE) || (bDump)) { smos_WriteBuffer(_T("\n\nAssertValid List - with Heal Results:")) ;
  //                                              pAList->Dump(TRUE, file_name, line_num, func_name, pObj->GetClassString()) ;
  //                                            }
  //             pAList->ReSet() ;
  //             bRtn = pObj->AssertValid(pAList, eLevelFlag, eWalkFlag) ;
  //             smos_WriteBuffer(_T("\n\nRefreshed AssertValid List - After Heal:")) ;
  //             pAList->Dump(TRUE, file_name, line_num, func_name, pObj->GetClassString()) ;
  //           }

  #endif // SM_DEBUG_CODE
  return(bRtn) ;
} // end sm_AssertValid(SmPeriodicExtent1d*, args)

/*******************************************************************//**
PURPOSE: SM_ASSERT_VALID for class SmExtent1d
NOTES:
***************************************************************/
SmBoolean sm_AssertValid
 (const SmExtent1d         * pObj,           // in :
  SmAssertObjDump            eDumpObjFlag,   // in :
  SmAssertStream             eStreamFlag,    // in :
  SmAssertTestLevel          eLevelFlag,     // in :
  SM_TYPE                    lExeClassType,  // NotUsed: in :
  SmTArray<ULONG>          * pTestRequests,  // NotUsed: in :
  SmAssertWalking            eWalkFlag,      // in :
  SmBoolean                  bDrawFlag,      // NotUsed: in :
  // SmBoolean                  bHeal,          // in :
  const TCHAR * const        file_name,      // in :
  ULONG                      line_num,       // in :
  const TCHAR * const        func_name)      // in :
{
  SM_REF3(lExeClassType, pTestRequests, bDrawFlag) ;
  SM_REF3(file_name, line_num, func_name);
  SM_REF5(pObj, eDumpObjFlag, eStreamFlag, eLevelFlag, eWalkFlag);

  SmBoolean bRtn = TRUE ;
#ifdef SM_DEBUG_CODE
  SmAssertArray sAList, * pAList = &sAList ;
  SmBoolean     bDump = (eStreamFlag == SM_STREAM) || (eStreamFlag == SM_GLOBAL_STREAM && SM_bAssertValidStream == TRUE) ;
  bRtn = pObj != NULL ? pObj->AssertValid(pAList, eLevelFlag, eWalkFlag) : TRUE ;
  if(eDumpObjFlag == SM_OBJDUMP_BEFORE) { pObj->Dump() ; }
  if(eDumpObjFlag == SM_OBJDUMP_ONLY)   { pObj->Dump() ; return TRUE ; }
  if (!bRtn) { TCHAR sBuff[SM_TBLOCK_SIZE], sBuffForFile[SM_TBLOCK_SIZE] ;
               smos_sprintf(sBuff       , _T("Object [%s=0x%p] Failed AssertValid Check "), pObj->GetClassString(), pObj) ;
               smos_sprintf(sBuffForFile, _T("Object [%s] Failed AssertValid Check "), pObj->GetClassString()) ;
               smos_AssertErrorMessage(SM_ERR_ASSERTVALID_FAILURE,FILE_NAME,LINE_NUMBER,sBuff,sBuffForFile,FALSE,func_name) ;
             }
  if((bRtn == FALSE) || bDump) { // if(bHeal) { smos_WriteBuffer(_T("\n\nAssertValid List - Before Heal:")) ; }
                                 pAList->Dump(TRUE, file_name, line_num, func_name, pObj->GetClassString()) ;
                               }
  if(eDumpObjFlag == SM_OBJDUMP_AFTER) { pObj->Dump() ; }
  // if(bDrawFlag) { pObj->Draw() ; }
  // obsolete
  // if(bHeal) { bRtn = pAList->Heal() ;
  //             if((bRtn == FALSE) || (bDump)) { smos_WriteBuffer(_T("\n\nAssertValid List - with Heal Results:")) ;
  //                                              pAList->Dump(TRUE, file_name, line_num, func_name, pObj->GetClassString()) ;
  //                                            }
  //             pAList->ReSet() ;
  //             bRtn = pObj->AssertValid(pAList, eLevelFlag, eWalkFlag) ;
  //             smos_WriteBuffer(_T("\n\nRefreshed AssertValid List - After Heal:")) ;
  //             pAList->Dump(TRUE, file_name, line_num, func_name, pObj->GetClassString()) ;
  //           }
#endif // SM_DEBUG_CODE
  return(bRtn) ;
} // end sm_AssertValid(SmExtent1d*, args)

/*******************************************************************//**
PURPOSE: SM_ASSERT_VALID for class SmPolarConversion
NOTES:
***************************************************************/
SmBoolean sm_AssertValid
 (const SmPolarConversion  * pObj,           // in :
  SmAssertObjDump            eDumpObjFlag,   // in :
  SmAssertStream             eStreamFlag,    // in :
  SmAssertTestLevel          eLevelFlag,     // in :
  SM_TYPE                    lExeClassType,  // NotUsed: in :
  SmTArray<ULONG>          * pTestRequests,  // NotUsed: in :
  SmAssertWalking            eWalkFlag,      // in :
  SmBoolean                  bDrawFlag,      // NotUsed: in :
  // SmBoolean                  bHeal,          // in :
  const TCHAR * const        file_name,      // in :
  ULONG                      line_num,       // in :
  const TCHAR * const        func_name)      // in :
{
  SM_REF3(lExeClassType, pTestRequests, bDrawFlag) ;
  SM_REF3(file_name, line_num, func_name);
  SM_REF5(pObj, eDumpObjFlag, eStreamFlag, eLevelFlag, eWalkFlag);

  SmBoolean bRtn = TRUE ;
#ifdef SM_DEBUG_CODE
  SmAssertArray sAList, * pAList = &sAList ;
  SmBoolean     bDump = (eStreamFlag == SM_STREAM) || (eStreamFlag == SM_GLOBAL_STREAM && SM_bAssertValidStream == TRUE) ;
  bRtn = pObj != NULL ? pObj->AssertValid(pAList, eLevelFlag, eWalkFlag) : TRUE ;
  if(eDumpObjFlag == SM_OBJDUMP_BEFORE) { pObj->Dump() ; }
  if(eDumpObjFlag == SM_OBJDUMP_ONLY)   { pObj->Dump() ; return TRUE ; }
  if (!bRtn) { TCHAR sBuff[SM_TBLOCK_SIZE], sBuffForFile[SM_TBLOCK_SIZE] ;
               smos_sprintf(sBuff       , _T("Object [%s=0x%p] Failed AssertValid Check "), pObj->GetClassString(), pObj) ;
               smos_sprintf(sBuffForFile, _T("Object [%s] Failed AssertValid Check "), pObj->GetClassString()) ;
               smos_AssertErrorMessage(SM_ERR_ASSERTVALID_FAILURE,FILE_NAME,LINE_NUMBER,sBuff,sBuffForFile,FALSE,func_name) ;
             }
  if((bRtn == FALSE) || bDump) { // if(bHeal) { smos_WriteBuffer(_T("\n\nAssertValid List - Before Heal:")) ; }
                                 pAList->Dump(TRUE, file_name, line_num, func_name, pObj->GetClassString()) ;
                               }
  if(eDumpObjFlag == SM_OBJDUMP_AFTER) { pObj->Dump() ; }
  // if(bDrawFlag) { pObj->Draw() ; }
  // obsolete
  // if(bHeal) { bRtn = pAList->Heal() ;
  //             if((bRtn == FALSE) || (bDump)) { smos_WriteBuffer(_T("\n\nAssertValid List - with Heal Results:")) ;
  //                                              pAList->Dump(TRUE, file_name, line_num, func_name, pObj->GetClassString()) ;
  //                                            }
  //             pAList->ReSet() ;
  //             bRtn = pObj->AssertValid(pAList, eLevelFlag, eWalkFlag) ;
  //             smos_WriteBuffer(_T("\n\nRefreshed AssertValid List - After Heal:")) ;
  //             pAList->Dump(TRUE, file_name, line_num, func_name, pObj->GetClassString()) ;
  //           }
#endif // SM_DEBUG_CODE
  return(bRtn) ;
} // end sm_AssertValid(SmPolarConversion*, args)

/*******************************************************************//**
PURPOSE: SM_ASSERT_VALID for class SmPolarBox
NOTES:
***************************************************************/
SmBoolean sm_AssertValid
 (const SmPolarBox         * pObj,           // in :
  SmAssertObjDump            eDumpObjFlag,   // in :
  SmAssertStream             eStreamFlag,    // in :
  SmAssertTestLevel          eLevelFlag,     // in :
  SM_TYPE                    lExeClassType,  // NotUsed: in :
  SmTArray<ULONG>          * pTestRequests,  // NotUsed: in :
  SmAssertWalking            eWalkFlag,      // in :
  SmBoolean                  bDrawFlag,      // NotUsed: in :
  // SmBoolean                  bHeal,          // in :
  const TCHAR * const        file_name,      // in :
  ULONG                      line_num,       // in :
  const TCHAR * const        func_name)      // in :
{
  SM_REF3(lExeClassType, pTestRequests, bDrawFlag) ;
  SM_REF8(pObj, eDumpObjFlag, eStreamFlag, eLevelFlag, eWalkFlag, file_name, line_num, func_name);

  SmBoolean bRtn = TRUE ;
#ifdef SM_DEBUG_CODE
  SmAssertArray sAList, * pAList = &sAList ;
  SmBoolean     bDump = (eStreamFlag == SM_STREAM) || (eStreamFlag == SM_GLOBAL_STREAM && SM_bAssertValidStream == TRUE) ;
  bRtn = pObj != NULL ? pObj->AssertValid(pAList, eLevelFlag, eWalkFlag) : TRUE ;
  if(eDumpObjFlag == SM_OBJDUMP_BEFORE) { pObj->Dump() ; }
  if(eDumpObjFlag == SM_OBJDUMP_ONLY)   { pObj->Dump() ; return TRUE ; }
  if (!bRtn) { TCHAR sBuff[SM_TBLOCK_SIZE], sBuffForFile[SM_TBLOCK_SIZE] ;
               smos_sprintf(sBuff       , _T("Object [%s=0x%p] Failed AssertValid Check "), pObj->GetClassString(), pObj) ;
               smos_sprintf(sBuffForFile, _T("Object [%s] Failed AssertValid Check "), pObj->GetClassString()) ;
               smos_AssertErrorMessage(SM_ERR_ASSERTVALID_FAILURE,FILE_NAME,LINE_NUMBER,sBuff,sBuffForFile,FALSE,func_name) ;
             }
  if((bRtn == FALSE) || bDump) { // if(bHeal) { smos_WriteBuffer(_T("\n\nAssertValid List - Before Heal:")) ; }
                                 pAList->Dump(TRUE, file_name, line_num, func_name, pObj->GetClassString()) ;
                               }
  if(eDumpObjFlag == SM_OBJDUMP_AFTER) { pObj->Dump() ; }
  if(bDrawFlag) { pObj->Draw( SmPoint3d(0,0,0) ) ; }
  // obsolete
  // if(bHeal) { bRtn = pAList->Heal() ;
  //             if((bRtn == FALSE) || (bDump)) { smos_WriteBuffer(_T("\n\nAssertValid List - with Heal Results:")) ;
  //                                              pAList->Dump(TRUE, file_name, line_num, func_name, pObj->GetClassString()) ;
  //                                            }
  //             pAList->ReSet() ;
  //             bRtn = pObj->AssertValid(pAList, eLevelFlag, eWalkFlag) ;
  //             smos_WriteBuffer(_T("\n\nRefreshed AssertValid List - After Heal:")) ;
  //             pAList->Dump(TRUE, file_name, line_num, func_name, pObj->GetClassString()) ;
  //           }
#endif // SM_DEBUG_CODE
  return(bRtn) ;
} // end sm_AssertValid(SmPolarBox*, args)

/*******************************************************************//**
PURPOSE: SM_ASSERT_VALID for class SmExtent2d
NOTES:
***************************************************************/
SmBoolean sm_AssertValid
 (const SmExtent2d         * pObj,           // in :
  SmAssertObjDump            eDumpObjFlag,   // in :
  SmAssertStream             eStreamFlag,    // in :
  SmAssertTestLevel          eLevelFlag,     // in :
  SM_TYPE                    lExeClassType,  // NotUsed: in :
  SmTArray<ULONG>          * pTestRequests,  // NotUsed: in :
  SmAssertWalking            eWalkFlag,      // in :
  SmBoolean                  bDrawFlag,      // NotUsed: in :
  // SmBoolean                  bHeal,          // in :
  const TCHAR * const        file_name,      // in :
  ULONG                      line_num,       // in :
  const TCHAR * const        func_name)      // in :
{
  SM_REF3(lExeClassType, pTestRequests, bDrawFlag) ;
  SM_REF8(pObj, eDumpObjFlag, eStreamFlag, eLevelFlag, eWalkFlag, file_name, line_num, func_name);

  SmBoolean bRtn = TRUE ;
#ifdef SM_DEBUG_CODE
  SmAssertArray sAList, * pAList = &sAList ;
  SmBoolean     bDump = (eStreamFlag == SM_STREAM) || (eStreamFlag == SM_GLOBAL_STREAM && SM_bAssertValidStream == TRUE) ;
  bRtn = pObj != NULL ? pObj->AssertValid(pAList, eLevelFlag, eWalkFlag) : TRUE ;
  if(eDumpObjFlag == SM_OBJDUMP_BEFORE) { pObj->Dump() ; }
  if(eDumpObjFlag == SM_OBJDUMP_ONLY)   { pObj->Dump() ; return TRUE ; }
  if (!bRtn) { TCHAR sBuff[SM_TBLOCK_SIZE], sBuffForFile[SM_TBLOCK_SIZE] ;
               smos_sprintf(sBuff       , _T("Object [%s=0x%p] Failed AssertValid Check "), pObj->GetClassString(), pObj) ;
               smos_sprintf(sBuffForFile, _T("Object [%s] Failed AssertValid Check "), pObj->GetClassString()) ;
               smos_AssertErrorMessage(SM_ERR_ASSERTVALID_FAILURE,FILE_NAME,LINE_NUMBER,sBuff,sBuffForFile,FALSE,func_name) ;
             }
  if((bRtn == FALSE) || bDump) { // if(bHeal) { smos_WriteBuffer(_T("\n\nAssertValid List - Before Heal:")) ; }
                                 pAList->Dump(TRUE, file_name, line_num, func_name, pObj->GetClassString()) ;
                               }
  if(eDumpObjFlag == SM_OBJDUMP_AFTER) { pObj->Dump() ; }
  if(bDrawFlag) { pObj->Draw() ; }
  // obsolete
  // if(bHeal) { bRtn = pAList->Heal() ;
  //             if((bRtn == FALSE) || (bDump)) { smos_WriteBuffer(_T("\n\nAssertValid List - with Heal Results:")) ;
  //                                              pAList->Dump(TRUE, file_name, line_num, func_name, pObj->GetClassString()) ;
  //                                            }
  //             pAList->ReSet() ;
  //             bRtn = pObj->AssertValid(pAList, eLevelFlag, eWalkFlag) ;
  //             smos_WriteBuffer(_T("\n\nRefreshed AssertValid List - After Heal:")) ;
  //             pAList->Dump(TRUE, file_name, line_num, func_name, pObj->GetClassString()) ;
  //           }
#endif // SM_DEBUG_CODE
  return(bRtn) ;
} // end sm_AssertValid(SmExtent2d*, args)

/*******************************************************************//**
PURPOSE: SM_ASSERT_VALID for class SmExtent3d
NOTES:
***************************************************************/
SmBoolean sm_AssertValid
 (const SmExtent3d         * pObj,           // in :
  SmAssertObjDump            eDumpObjFlag,   // in :
  SmAssertStream             eStreamFlag,    // in :
  SmAssertTestLevel          eLevelFlag,     // in :
  SM_TYPE                    lExeClassType,  // NotUsed: in :
  SmTArray<ULONG>          * pTestRequests,  // NotUsed: in :
  SmAssertWalking            eWalkFlag,      // in :
  SmBoolean                  bDrawFlag,      // NotUsed: in :
  // SmBoolean                  bHeal,          // in :
  const TCHAR * const        file_name,      // in :
  ULONG                      line_num,       // in :
  const TCHAR * const        func_name)      // in :
{
  SM_REF3(lExeClassType, pTestRequests, bDrawFlag) ;
  SM_REF8(pObj, eDumpObjFlag, eStreamFlag, eLevelFlag, eWalkFlag, file_name, line_num, func_name);

  SmBoolean bRtn = TRUE ;
#ifdef SM_DEBUG_CODE
  SmAssertArray sAList, * pAList = &sAList ;
  SmBoolean     bDump = (eStreamFlag == SM_STREAM) || (eStreamFlag == SM_GLOBAL_STREAM && SM_bAssertValidStream == TRUE) ;
  bRtn = pObj != NULL ? pObj->AssertValid(pAList, eLevelFlag, eWalkFlag) : TRUE ;
  if(eDumpObjFlag == SM_OBJDUMP_BEFORE) { pObj->Dump() ; }
  if(eDumpObjFlag == SM_OBJDUMP_ONLY)   { pObj->Dump() ; return TRUE ; }
  if (!bRtn) { TCHAR sBuff[SM_TBLOCK_SIZE], sBuffForFile[SM_TBLOCK_SIZE] ;
               smos_sprintf(sBuff       , _T("Object [%s=0x%p] Failed AssertValid Check "), pObj->GetClassString(), pObj) ;
               smos_sprintf(sBuffForFile, _T("Object [%s] Failed AssertValid Check "), pObj->GetClassString()) ;
               smos_AssertErrorMessage(SM_ERR_ASSERTVALID_FAILURE,FILE_NAME,LINE_NUMBER,sBuff,sBuffForFile,FALSE,func_name) ;
             }
  if((bRtn == FALSE) || bDump) { // if(bHeal) { smos_WriteBuffer(_T("\n\nAssertValid List - Before Heal:")) ; }
                                 pAList->Dump(TRUE, file_name, line_num, func_name, pObj->GetClassString()) ;
                               }
  if(eDumpObjFlag == SM_OBJDUMP_AFTER) { pObj->Dump() ; }
  if(bDrawFlag) { pObj->Draw() ; }
  // obsolete
  // if(bHeal) { bRtn = pAList->Heal() ;
  //             if((bRtn == FALSE) || (bDump)) { smos_WriteBuffer(_T("\n\nAssertValid List - with Heal Results:")) ;
  //                                              pAList->Dump(TRUE, file_name, line_num, func_name, pObj->GetClassString()) ;
  //                                            }
  //             pAList->ReSet() ;
  //             bRtn = pObj->AssertValid(pAList, eLevelFlag, eWalkFlag) ;
  //             smos_WriteBuffer(_T("\n\nRefreshed AssertValid List - After Heal:")) ;
  //             pAList->Dump(TRUE, file_name, line_num, func_name, pObj->GetClassString()) ;
  //           }
#endif // SM_DEBUG_CODE
  return(bRtn) ;
} // end sm_AssertValid(SmExtent3d*, args)

/*******************************************************************//**
PURPOSE: SM_ASSERT_VALID for class SmFilletGeom
NOTES:
***************************************************************/
SmBoolean sm_AssertValid
 (const SmFilletGeom       * pObj,           // in :
  SmAssertObjDump            eDumpObjFlag,   // in :
  SmAssertStream             eStreamFlag,    // in :
  SmAssertTestLevel          eLevelFlag,     // in :
  SM_TYPE                    lExeClassType,  // NotUsed: in :
  SmTArray<ULONG>          * pTestRequests,  // NotUsed: in :
  SmAssertWalking            eWalkFlag,      // in :
  SmBoolean                  bDrawFlag,      // NotUsed: in :
  // SmBoolean                  bHeal,          // in :
  const TCHAR * const        file_name,      // in :
  ULONG                      line_num,       // in :
  const TCHAR * const        func_name)      // in :
{
  SM_REF3(lExeClassType, pTestRequests, bDrawFlag) ;
  SM_REF8(pObj, eDumpObjFlag, eStreamFlag, eLevelFlag, eWalkFlag, file_name, line_num, func_name);

  SmBoolean bRtn = TRUE ;
#ifdef SM_DEBUG_CODE
  SmAssertArray sAList, * pAList = &sAList ;
  SmBoolean     bDump = (eStreamFlag == SM_STREAM) || (eStreamFlag == SM_GLOBAL_STREAM && SM_bAssertValidStream == TRUE) ;
  bRtn = pObj != NULL ? pObj->AssertValid(pAList, eLevelFlag, eWalkFlag) : TRUE ;
  if(eDumpObjFlag == SM_OBJDUMP_BEFORE) { pObj->Dump() ; }
  if(eDumpObjFlag == SM_OBJDUMP_ONLY)   { pObj->Dump() ; return TRUE ; }
  if (!bRtn) { TCHAR sBuff[SM_TBLOCK_SIZE], sBuffForFile[SM_TBLOCK_SIZE] ;
               smos_sprintf(sBuff       , _T("Object [%s=0x%p] Failed AssertValid Check "), pObj->GetClassString(), pObj) ;
               smos_sprintf(sBuffForFile, _T("Object [%s] Failed AssertValid Check "), pObj->GetClassString()) ;
               smos_AssertErrorMessage(SM_ERR_ASSERTVALID_FAILURE,FILE_NAME,LINE_NUMBER,sBuff,sBuffForFile,FALSE,func_name) ;
             }
  if((bRtn == FALSE) || bDump) { // if(bHeal) { smos_WriteBuffer(_T("\n\nAssertValid List - Before Heal:")) ; }
                                 pAList->Dump(TRUE, file_name, line_num, func_name, pObj->GetClassString()) ;
                               }
  if(eDumpObjFlag == SM_OBJDUMP_AFTER) { pObj->Dump() ; }
  if(bDrawFlag) { pObj->Draw() ; }
  // obsolete
  // if(bHeal) { bRtn = pAList->Heal() ;
  //             if((bRtn == FALSE) || (bDump)) { smos_WriteBuffer(_T("\n\nAssertValid List - with Heal Results:")) ;
  //                                              pAList->Dump(TRUE, file_name, line_num, func_name, pObj->GetClassString()) ;
  //                                            }
  //             pAList->ReSet() ;
  //             bRtn = pObj->AssertValid(pAList, eLevelFlag, eWalkFlag) ;
  //             smos_WriteBuffer(_T("\n\nRefreshed AssertValid List - After Heal:")) ;
  //             pAList->Dump(TRUE, file_name, line_num, func_name, pObj->GetClassString()) ;
  //           }
#endif // SM_DEBUG_CODE
  return(bRtn) ;
} // end sm_AssertValid(SmFilletGeom*, args)

/*******************************************************************//**
PURPOSE: SM_ASSERT_VALID for class SmCurveInterval
NOTES:
***************************************************************/
SmBoolean sm_AssertValid
 (const SmCurveInterval    * pObj,           // in :
  SmAssertObjDump            eDumpObjFlag,   // in :
  SmAssertStream             eStreamFlag,    // in :
  SmAssertTestLevel          eLevelFlag,     // in :
  SM_TYPE                    lExeClassType,  // NotUsed: in :
  SmTArray<ULONG>          * pTestRequests,  // NotUsed: in :
  SmAssertWalking            eWalkFlag,      // in :
  SmBoolean                  bDrawFlag,      // NotUsed: in :
  // SmBoolean                  bHeal,          // in :
  const TCHAR * const        file_name,      // in :
  ULONG                      line_num,       // in :
  const TCHAR * const        func_name)      // in :
{
  SM_REF3(lExeClassType, pTestRequests, bDrawFlag) ;
  SM_REF8(pObj, eDumpObjFlag, eStreamFlag, eLevelFlag, eWalkFlag, file_name, line_num, func_name);

  SmBoolean bRtn = TRUE ;
#ifdef SM_DEBUG_CODE
  SmAssertArray sAList, * pAList = &sAList ;
  SmBoolean     bDump = (eStreamFlag == SM_STREAM) || (eStreamFlag == SM_GLOBAL_STREAM && SM_bAssertValidStream == TRUE) ;
  bRtn = pObj != NULL ? pObj->AssertValid(pAList, eLevelFlag, eWalkFlag) : TRUE ;
  if(eDumpObjFlag == SM_OBJDUMP_BEFORE) { pObj->Dump() ; }
  if(eDumpObjFlag == SM_OBJDUMP_ONLY)   { pObj->Dump() ; return TRUE ; }
  if (!bRtn) { TCHAR sBuff[SM_TBLOCK_SIZE], sBuffForFile[SM_TBLOCK_SIZE] ;
               smos_sprintf(sBuff       , _T("Object [%s=0x%p] Failed AssertValid Check "), pObj->GetClassString(), pObj) ;
               smos_sprintf(sBuffForFile, _T("Object [%s] Failed AssertValid Check "), pObj->GetClassString()) ;
               smos_AssertErrorMessage(SM_ERR_ASSERTVALID_FAILURE,FILE_NAME,LINE_NUMBER,sBuff,sBuffForFile,FALSE,func_name) ;
             }
  if((bRtn == FALSE) || bDump) { // if(bHeal) { smos_WriteBuffer(_T("\n\nAssertValid List - Before Heal:")) ; }
                                 pAList->Dump(TRUE, file_name, line_num, func_name, pObj->GetClassString()) ;
                               }
  if(eDumpObjFlag == SM_OBJDUMP_AFTER) { pObj->Dump() ; }
  if(bDrawFlag) { pObj->Draw() ; }
  // obsolete
  // if(bHeal) { bRtn = pAList->Heal() ;
  //             if((bRtn == FALSE) || (bDump)) { smos_WriteBuffer(_T("\n\nAssertValid List - with Heal Results:")) ;
  //                                              pAList->Dump(TRUE, file_name, line_num, func_name, pObj->GetClassString()) ;
  //                                            }
  //             pAList->ReSet() ;
  //             bRtn = pObj->AssertValid(pAList, eLevelFlag, eWalkFlag) ;
  //             smos_WriteBuffer(_T("\n\nRefreshed AssertValid List - After Heal:")) ;
  //             pAList->Dump(TRUE, file_name, line_num, func_name, pObj->GetClassString()) ;
  //           }
#endif // SM_DEBUG_CODE
  return(bRtn) ;
} // end sm_AssertValid(SmCurveInterval*, args)
