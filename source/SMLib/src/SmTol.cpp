// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************//**
* FILE NAME --- SmTol.h
* PURPOSE: Source file for Tolerance Computation methods.
**********************************************************************/

#include "StdAfx.h"

#include <SmTol.h>
#include <SmMessages.h>
#include <SmExtent1d.h>
#include <SmExtent2d.h>
#include <SmExtent3d.h>
#include <SmVector2d.h>
#include <SmVector3d.h>
#include <SmPseudoBox.h>
#include <SmLocalSolveNd.h>
#include <SmVolume.h>
#include <SmCurveClass.h>
#include <SmPointClass.h>
#include <SmCrvOnSurf.h>
#include <SmCacheMgr.h>
#include <SmContext.h>

#include <SmSurfaceIntersector.h>
#include <SmSurfaceTracer.h>

  #include <SmTopology.h>
  #include <SmBrep.h>
  #include <SmBrepData.h>
  #include <SmHCR.h>
  #include <SmPoly.h>
  #include <SmRegion.h>
  #include <SmShell.h>
  #include <SmFace.h>
  #include <SmEdge.h>
  #include <SmVertex.h>
  #include <SmGap.h>  // for UpdateObjectTolerance()

  #include <SmPolyIntersector.h>
  #include <SmTopologyIntersector.h>
  #include <SmLineSegClass.h>

#include <SmFilletCorner.h>   
#include <SmFilletSolver.h>
#include <SmOffsetGeometryCreation.h>

// Constant tolerance class objects to be used as method default argument values.
// SmXSectTol3d sm_XSecTol3d ;   // constant SmXSectTol3d object to be used for default method argument values
// JLMCC Avove was already declared in SmTol.h as thread local, leaving it there, removing it here


  // ModelSize => Default Tolerance scheme:
  //   Specify a guess of the overall model size good to within a factor of 100 or so
  //    ModelSizeEstimate e.g. .5, 50, 500, 50000
  //   and SMLib computes reasonable default tolerance sizes as follows :
  //      +---------------------+-----------+------------------+
  //      |     SizeEstimate    | ZoneTol3d | Expected Use     |
  //      +---------------------+-----------+------------------+
  //      |    0.00005-    0.005|  1.0e-7   |   Rare           |
  //      |    0.005  -    0.5  |  1.0e-6   |  Once in a while |
  //      |    0.5    -  500    |  1.0e-5   | Default          |
  //      |  500      -50000    |  1.0e-4   |  Once in a while |   
  //      |50000      -5e6      |  1.0e-3   |   Rare           |
  //      +---------------------+-----------+------------------+
  // 

  static SM_THREAD_LOCAL SmZoneTol3d s_ZoneTol3d(SM_ZONE_TOL_3D) ;
  static SM_THREAD_LOCAL double s_TolGapGain(SM_TOL_GAP_GAIN);
  static SM_THREAD_LOCAL double s_ModelSizeEstimate(SM_MODEL_SIZE_ESTIMATE);
  static SM_THREAD_LOCAL double s_LargeSmallSizeRatio(SM_LARGE_SMALL_SIZE_RATIO);

  // system wide stored ZoneTol3d value based on the last constructed Context's or Brep's ModelSizeEstimate
  SM_NEWTOL_LINE double SmTol::GetModelSizeEstimate(const SmContext & rContext) { return rContext.GetThisModelSizeEstimate() ; } 
  SM_NEWTOL_LINE double SmTol::GetModelSizeEstimate(const SmBrep    & rBrep)    { return rBrep.GetThisModelSizeEstimate() ; }
                 double SmTol::GetModelSizeEstimate()                           { return s_ModelSizeEstimate ; }

  SM_NEWTOL_LINE SmZoneTol3d SmTol::SetModelSizeEstimate(SmContext & rContext, double dModelSizeEstimate) { return rContext.SetThisModelSizeEstimate(dModelSizeEstimate) ; }
  SM_NEWTOL_LINE SmZoneTol3d SmTol::SetModelSizeEstimate(SmBrep    & rBrep,    double dModelSizeEstimate) { return rBrep.SetThisModelSizeEstimate(dModelSizeEstimate) ; }
                 SmZoneTol3d SmTol::SetModelSizeEstimate(double dModelSizeEstimate) { s_ModelSizeEstimate = dModelSizeEstimate == SM_USE_DEFAULT
                                                                                        ? SM_MODEL_SIZE_ESTIMATE
                                                                                        : dModelSizeEstimate ;
                                                                                      s_ZoneTol3d         = SizeEstimateToZoneTol3d(s_ModelSizeEstimate) ; 
                                                                                      return s_ZoneTol3d ;
                                                                                    }
                                                                                                                                                                     
  SM_NEWTOL_LINE double SmTol::GetLargeSmallSizeRatio(const SmContext & rContext) { return rContext.GetThisLargeSmallSizeRatio() ; }
  SM_NEWTOL_LINE double SmTol::GetLargeSmallSizeRatio(const SmBrep    & rBrep)    { return rBrep.GetThisLargeSmallSizeRatio() ; }
                 double SmTol::GetLargeSmallSizeRatio()                           { return s_LargeSmallSizeRatio ; }

  SM_NEWTOL_LINE void SmTol::SetLargeSmallSizeRatio(SmContext & rContext, double dLargeSmallSizeRatio) { return rContext.SetThisLargeSmallSizeRatio(dLargeSmallSizeRatio) ; }
  SM_NEWTOL_LINE void SmTol::SetLargeSmallSizeRatio(SmBrep    & rBrep,    double dLargeSmallSizeRatio) { return rBrep.SetThisLargeSmallSizeRatio(dLargeSmallSizeRatio) ; }
                 void SmTol::SetLargeSmallSizeRatio(double dLargeSmallSizeRatio)                       { s_LargeSmallSizeRatio = dLargeSmallSizeRatio == SM_USE_DEFAULT
                                                                                                            ? SM_LARGE_SMALL_SIZE_RATIO
                                                                                                            : dLargeSmallSizeRatio ;  
                                                                                                       }
#ifdef SM_USE_NEWTOL
 // // helper functions - not usually needed
 // SmZoneTol3d SizeEstimateToZoneTol3d(double dModelSizeEstimate)         { return (pow(floor(log10(fabs(dModelSizeEstimate)))/2.0,10)*1.0e-5) ; }
 // double      ZoneTol3dToSizeEstimate(SmZoneTol3d sZoneTol3d)            { return (pow(floor(log10((sZoneTol3d / 1.0e-5)))*2,10)*50) ; }  
  // s_LargeSmallSizeRatio is highly unlikely to ever need changing - only change this value if you're an expert
#endif // SM_USE_NEWTOL

  // default derived SmTol3d constructors
  SmZoneTol3d  ::SmZoneTol3d  (double dVal) : SmTol3d(dVal == SM_USE_DEFAULT ? s_ZoneTol3d.val     : dVal) { }  ;   
  SmXSectTol3d ::SmXSectTol3d (double dVal) : SmTol3d(dVal == SM_USE_DEFAULT ? s_ZoneTol3d.val * 2.0 : dVal) { }  ;  
  SmApproxTol3d::SmApproxTol3d(double dVal) : SmTol3d(dVal == SM_USE_DEFAULT ? s_ZoneTol3d.val / 2.0 : dVal) { }  ; 
  SmScaledZero ::SmScaledZero (double dVal) : SmTol3d(dVal) { }  ;
  SmTol2d      ::SmTol2d      (double dVal) : SmTol3d(dVal) { }  ;
  SmTol1d      ::SmTol1d      (double dVal) : SmTol3d(dVal) { }  ;

  // Value Metrics
  SmBoolean    SmTol::IsG1AngRad            (SmVector3d *pVec1, SmVector3d *pVec2, SmScaledZero *pOptOverrideEffZeroAngRad) 
                                            { SM_ASSERT_TOLPTR(pOptOverrideEffZeroAngRad) ;
                                              SmScaledZero sAngTolRad = pOptOverrideEffZeroAngRad ? *pOptOverrideEffZeroAngRad : GetEffG1AngRad() ;
                                              double dAngRad ; pVec1->AngleBetween(*pVec2, dAngRad) ;
                                              return( dAngRad < sAngTolRad ) ; 
                                            }
  SmBoolean    SmTol::IsRadiusSmall         (double dCurvatureRad,                 SmZoneTol3d  *pOptOverrideZoneTol3d)     
                                            { SM_ASSERT_TOLPTR(pOptOverrideZoneTol3d) ;
                                              SmZoneTol3d sMinRadiusOfCurvatureTol3d = pOptOverrideZoneTol3d ? *pOptOverrideZoneTol3d : GetZoneTol3d() ;
                                              return( dCurvatureRad < sMinRadiusOfCurvatureTol3d) ;
                                            }
  SmBoolean    SmTol::AreParallel           (SmVector3d *pVec1, SmVector3d *pVec2, double *pOptOverrideEffZeroAngRad) 
                                            { SM_ASSERT_TOLPTR(pOptOverrideEffZeroAngRad) ;
                                              double dAngTolRad = pOptOverrideEffZeroAngRad ? *pOptOverrideEffZeroAngRad : GetAngTolRad() ; 
                                              return( pVec1->IsParallelTo(*pVec2, SM_RAD2DEG(dAngTolRad))) ; 
                                            }
  SmBoolean    SmTol::ArePerpendicular      (SmVector3d *pVec1, SmVector3d *pVec2, double *pOptOverrideEffZeroAngRad)  
                                            { SM_ASSERT_TOLPTR(pOptOverrideEffZeroAngRad) ;
                                              double dAngTolRad = pOptOverrideEffZeroAngRad ? *pOptOverrideEffZeroAngRad : GetAngTolRad() ; 
                                              return( pVec1->IsPerpendicularTo(*pVec2, SM_RAD2DEG(dAngTolRad))) ; 
                                            }
  SmBoolean    SmTol::AreIndependentTangents(SmVector3d *pVec1, SmVector3d *pVec2, SmScaledZero *pOptOverrideScaledZero)    
                                            { SM_ASSERT_TOLPTR(pOptOverrideScaledZero) ;
                                              SmScaledZero sScaledZeroSq =    pOptOverrideScaledZero 
                                                                           ? (SmScaledZero)(*pOptOverrideScaledZero * *pOptOverrideScaledZero)
                                                                           : GetScaledZeroSq(*pVec1, *pVec2) ; 
                                              return((*pVec1 * *pVec2).LengthSquared() > sScaledZeroSq) ;
                                            }
  SmBoolean    SmTol::IsZeroSpeed           (double      dSpeed, double dMaxPosDimension,  SmScaledZero *pOptOverrideScaledZero)     
                                            { SM_ASSERT_TOLPTR(pOptOverrideScaledZero) ;
                                              SmBoolean bRtn = smos_Fabs(dSpeed) < (pOptOverrideScaledZero 
                                                                                    ? (double)*pOptOverrideScaledZero
                                                                                    : SM_EFF_ZERO * (1.0 + smos_Fabs(dMaxPosDimension))) ;
                                              return(bRtn) ; 
                                            }
  SmBoolean    SmTol::IsZeroNormal          (SmVector3d *pSurfaceNormal,           SmScaledZero *pOptOverrideScaledZero)  // currently a stub function   
                                            { SM_REF2(pSurfaceNormal, pOptOverrideScaledZero) ;
                                              SM_ASSERT_TOLPTR(pOptOverrideScaledZero) ;
                                              ERR_MSG(_T("NOT IMPLEMENTED")) ; return(FALSE) ; 
                                            }
#if 0
  // This does not seem used yet.
  // Temporarily replacing with code below for Linux compile
  SmBoolean    SmTol::AreSameParam           (double      dParam1, double dParam2,  SmScaledZero *pOptOverrideEffZeroParam)     
                                            { SM_ASSERT_TOLPTR(pOptOverrideEffZeroParam) ;
                                              return(smos_Fabs(dParam1 - dParam2) < (  pOptOverrideEffZeroParam
                                                                                     ? *pOptOverrideEffZeroParam
                                                                                     :  SM_EFF_ZERO_PARAM)) ;
                                            }
#endif
  SmBoolean    SmTol::AreSameParam(double      dParam1, double dParam2, SmScaledZero *pOptOverrideEffZeroParam)
                                 { SM_ASSERT_TOLPTR(pOptOverrideEffZeroParam);
                                   double dParamTol = SM_EFF_ZERO_PARAM;
                                   if (pOptOverrideEffZeroParam)
                                       dParamTol = *pOptOverrideEffZeroParam;
                                   if (smos_Fabs(dParam1 - dParam2) < dParamTol)
                                       return TRUE;
                                   return FALSE;
                                 }

  SmBoolean    SmTol::IsOnSeam              (const SmPoint2d &crUV,  const SmSurface &crSurface, SmZoneTol3d *pOptOverrideZoneTol3d)
                                            { SM_ASSERT_TOLPTR(pOptOverrideZoneTol3d) ;
                                              SmZoneTol3d sZoneTol3d = pOptOverrideZoneTol3d ? *pOptOverrideZoneTol3d : SmTol::GetZoneTol3d(&crSurface) ;
                                              SmVector2d  sUDir(1,0), sVDir(0,1) ; 
                                              SmTol2d     sZoneTol2d_U = MapTo2d(sZoneTol3d, crUV, sUDir, crSurface) ;
                                              SmTol2d     sZoneTol2d_V = MapTo2d(sZoneTol3d, crUV, sVDir, crSurface) ;
                                              SmTol2d     sTol2d = smos_Min(sZoneTol2d_U, sZoneTol2d_V) ;
                                              // gwc: todo SmSurface::IsOnSeam - needs to be updated to handle direction specific 2d tolerances
                                              return(crSurface.IsOnSeam(crUV, &sTol2d) ) ;
                                            }

  SmBoolean    SmTol::AreCoinPoints3d       (SmPoint3d  *pPt1,  SmPoint3d *pPt2,   SmXSectTol3d *pOptOverrideXSectTol3d)        // currently a stub function
                                            { SM_REF3(pPt1, pPt2, pOptOverrideXSectTol3d) ;
                                              SM_ASSERT_TOLPTR(pOptOverrideXSectTol3d) ;
                                              ERR_MSG(_T("NOT IMPLEMENTED")) ; return(FALSE) ; 
                                            }
  SmBoolean    SmTol::AreCoinPoints2d       (SmPoint3d  *pPt1,  SmPoint3d *pPt2,   SmTol2d sZoneTol2d_1, SmTol2d sZoneTol2d_2)  // currently a stub function  
                                            { SM_REF4(pPt1, pPt2, sZoneTol2d_1, sZoneTol2d_2) ;
                                              SM_ASSERT_TOL2(sZoneTol2d_1, sZoneTol2d_2) ;
                                              ERR_MSG(_T("NOT IMPLEMENTED")) ; return(FALSE) ; 
                                            }
  SmBoolean    SmTol::AreCoinPoints1d       (double     *pPt1,  double    *pPt2,   SmTol1d sZoneTol1d_1, SmTol1d sZoneTol1d_2)  // currently a stub function  
                                            { SM_REF4(pPt1, pPt2, sZoneTol1d_1, sZoneTol1d_2) ;
                                              SM_ASSERT_TOL2(sZoneTol1d_1, sZoneTol1d_2) ;
                                              ERR_MSG(_T("NOT IMPLEMENTED")) ; return(FALSE) ; 
                                            }
      // expected value = sZoneTol1d = MapTo1d(sZoneTol3d, dParam, sCurve) ; 
      // expected value = sZoneTol2d = MapTo2d(sZoneTol3d, sUV, sUVdir, sSurface) ; 

/*******************************************************************//**
PURPOSE: GetZoneTol3d Helper function
  Calc this Topo's unique ZoneTol3d for each higher dimension connection
  and return largest such ZoneTol3d value.

NOTES: 1. for all   : return 0.0 for not connected to a higher dimension object
          for Edge  : return largest EdgeZoneTol3d(Face) or
          for vertex: return largest VertexZoneTol3d(Face or Edge)

       2. the returned SmZoneTol3d value includes the DimGain effect.

       3. These Max values are used by SmTol::GetZoneTol3d to calc
           Vertex and Edge ZoneTol3d values.

METHOD:    For every Face to which an Edge connects
        or For every Face and Edge to which a Vertex connects directly,
           {  computes this object's ZoneTol3d value as if that were the only
                Face or Edge to which the object connects.
           } and
         returns the largest such ZoneTol3d value.
***********************************************************************/
#ifdef NOT_BEING_USED // not yet used - perhaps obsolete 
SmZoneTol3d SmTol::GetMaxUpDimZoneTol3d // rtn: Max UpDim DimGain*ZoneTol3d value
 (const SmTopology * pTopo,             // in : Vertex, Edge, or Face to target
  SmGap            & rMaxUpDimGap,      // out: Edge  : Max(Edge/Face Gaps)
                                        //      Vertex: Max(Vertex/Edge Gaps, Vertex/Face Gaps)
  SmBoolean          bUseNewTol)        // in : for internal use only - Always use default values
                                        //      TRUE = use NewTolerance Model, FALSE=use previous behavior,
                                        //      default:[SM_USE_NEWTOL ? TRUE : FALSE]
{
  // no work - no Edges or Vertices
  return(0.0) ;

  // No work - pTopo not an Edge or Vertex
  if(   !pTopo->IsKindOf(SmEdge_TYPE)
     && !pTopo->IsKindOf(SmVertex_TYPE))
    { return(0.0) ; }

  // init output
  SmZoneTol3d sMaxZoneTol3d = 0.0 ;
  rMaxUpDimGap              = 0.0 ;

  // local values
  ULONG       ii ;

  // Edge Branch - get Max ZoneTol3d for each Edge/Face connection
  if(pTopo->IsKindOf(SmEdge_TYPE))
    {
      SmEdge * pEdge = (SmEdge *)pTopo ;

      SM_PTR_ARRAY(sEdgeuses, SmEdgeuse, 16) ;
      pEdge->GetEdgeuses(sEdgeuses) ;

      // for every Edgeuse
      for(ii=0;ii<sEdgeuses.GetSize();ii++)
        {
          SmFace * pFace = sEdgeuses[ii]->GetFace() ;

          // when Edge is connected to a Face
          if(pFace)
            {
              // get ThisFace ZoneTol3d
              SmZoneTol3d sFaceZoneTol3d = SmTol::GetZoneTol3d(*pFace) ;

              // save the largest Dim Scaled ZoneTol3d
              if(sMaxZoneTol3d < sFaceZoneTol3d)
                { sMaxZoneTol3d = sFaceZoneTol3d ; }

              // get Edge/ThisFace gap
              SmEdgeFaceGap * pMaxEdgeFaceGap = sEdgeuses[ii]->GetMaxEdgeFaceGap() ;

              // save the largest Edge/Face Gap
              if(pMaxEdgeFaceGap && rMaxUpDimGap < *pMaxEdgeFaceGap)
                { rMaxUpDimGap = *pMaxEdgeFaceGap ; }

              // GWC - removed: Don't use Edge/FaceUVTrimCurves in Consistent Tolerance Model - Keep everything 3d
              //  // get Edge/ThisFaceTrimCurve gap
              //  SmEdgeFaceTrimCurveGap sMaxEdgeFaceTrimCurveGap = sEdgeuses[ii]->GetMaxEdgeFaceTrimCurveGap() ;
              //
              //  if(rMaxUpDimGap < sMaxEdgeFaceTrimCurveGap)
              //    { rMaxUpDimGap = sMaxEdgeFaceTrimCurveGap ; }

            } // end Edge connected to a Face check
        } // end iter every edgeuse
    } // end Edge Branch - setting sMaxZoneTol3d

  else // Vertex Branch - get Max ZoneTol3d for each Vertex/Edge and Vertex/Face connection
    {
      SM_ASSERT(pTopo->IsKindOf(SmVertex_TYPE)) ;
      SmVertex * pVertex = (SmVertex*)pTopo ;

      // locals
      ULONG ii ;
      SM_PTR_ARRAY(sVertexuses, SmVertexuse, 16) ;
      pVertex->GetVertexuses(sVertexuses) ;

      // for every Vertex connection
      for(ii=0;ii<sVertexuses.GetSize();ii++)
        {
          SmEdge * pEdge = sVertexuses[ii]->HasVertexEdgeGap() ? sVertexuses[ii]->GetEdgeuse()->GetEdge() : NULL ;
          SmFace * pFace = sVertexuses[ii]->HasVertexFaceGap() ? sVertexuses[ii]->GetFaceuse()->GetFace() : NULL ;

          // when connected to an Edge
          if(pEdge)
            {
              // ThisZoneTol3d = ThisEdge->ZoneTol3d + Vertex/ThisEdgeGap
              SmZoneTol3d sThisZoneTol3d = SmTol::GetZoneTol3d(*pEdge) ;

              // save the Max ThisZoneTol3d value seen
              if(sMaxZoneTol3d < sThisZoneTol3d)
                { sMaxZoneTol3d = sThisZoneTol3d ; }

              // fetch Vertex/ThisEdge gap
              SmGap * pVertexEdgeGap = sVertexuses[ii]->GetVertexEdgeGap() ;

              // save the largest Gap
              if(pVertexEdgeGap && rMaxUpDimGap < *pVertexEdgeGap)
                { rMaxUpDimGap = *pVertexEdgeGap ; }

            } // end Vertex connected to Edge check

          // when connected to a Face (through an Edge or directly)
          if(pFace)
            {
              // fetch Vertex/ThisFace gap
              SmGap *pVertexFaceGap = sVertexuses[ii]->GetVertexFaceGap() ;

              // let ThisZoneTol3d = ThisFace->ZoneTol3d + Vertex/ThisFaceGap
              SmZoneTol3d sThisZoneTol3d = SmTol::GetZoneTol3d(*pFace) ;

              // save the Max ThisZoneTol3d value seen
              if(sMaxZoneTol3d < sThisZoneTol3d)
                { sMaxZoneTol3d = sThisZoneTol3d ; }

              // save the largest Gap
              if(pVertexFaceGap && rMaxUpDimGap < *pVertexFaceGap)
                { rMaxUpDimGap = *pVertexFaceGap ; }

             } // end Vertex connected to Face check

        } // end iter every vertexuse
    } // end Vertex Branch - setting sMaxZoneTol3d

  // all done
  return(sMaxZoneTol3d) ;

} // end SmTol::GetMaxUpDimZoneTol3d
#endif // NOT_BEING_USED // not yet used - perhaps obsolete 

/*******************************************************************//**
PURPOSE: GetApproxTol3d Helper function
  Return TRUE when any of the UpDim objects are using a LocalZoneTol3d value.

NOTES: 1.
***********************************************************************/
#ifdef NOT_BEING_USED // not yet used - perhaps obsolete 
SmBoolean SmTol::HasUpDimLocalZoneTol3d // rtn: TRUE=a MaxUpDim obj uses a LocalDefTol3d object
 (const SmObject * pObj,                   // in : Vertex, Edge, or Face to target
  SmBoolean        bUseNewTol)             // in : for internal use only - Always use default values
                                           //      TRUE = use NewTolerance Model, FALSE=use previous behavior,
                                           //      default:[SM_USE_NEWTOL ? TRUE : FALSE]
{
  // no work - not using NewTol
  if(bUseNewTol == FALSE)
    { return(FALSE) ; }

  // no work - pTopo not an Edge or Vertex
  if(   !pObj->IsKindOf(SmEdge_TYPE)
     && !pObj->IsKindOf(SmVertex_TYPE))
    { return(FALSE) ; }

  // local values
  ULONG ii ;
  SmZoneTol3d sLocalZoneTol3d ;

  // Edge Branch - get Max ZoneTol3d for each Edge/Face connection
  if(pObj->IsKindOf(SmEdge_TYPE))
    {
      SmEdge * pEdge = (SmEdge *)pObj ;

      SM_PTR_ARRAY(sEdgeuses, SmEdgeuse, 16) ;
      pEdge->GetEdgeuses(sEdgeuses) ;

      // for every Edgeuse
      for(ii=0;ii<sEdgeuses.GetSize();ii++)
        {
          SmFace * pFace = sEdgeuses[ii]->GetFace() ;

          // when Edge is connected to a Face
          if(pFace)
            {
              // check ThisFace for a ZoneTol3d
              if(   HasLocalZoneTol3d(pFace, sLocalZoneTol3d, bUseNewTol)
                 && sLocalZoneTol3d != SM_UNINIT_TOL)
                { return(TRUE) ; }

            } // end Edge connected to a Face check
        } // end iter every edgeuse
    } // end Edge Branch - setting sMaxZoneTol3d

  else // Vertex Branch - get Max ZoneTol3d for each Vertex/Edge and Vertex/Face connection
    {
      SM_ASSERT(pObj->IsKindOf(SmVertex_TYPE)) ;
      SmVertex * pVertex = (SmVertex*)pObj ;

      // locals
      SM_PTR_ARRAY(sVertexuses, SmVertexuse, 16) ;
      pVertex->GetVertexuses(sVertexuses) ;

      // for every Vertex connection
      for(ii=0;ii<sVertexuses.GetSize();ii++)
        {
          SmEdge * pEdge = sVertexuses[ii]->HasVertexEdgeGap() ? sVertexuses[ii]->GetEdgeuse()->GetEdge() : NULL ;
          SmFace * pFace = sVertexuses[ii]->HasVertexFaceGap() ? sVertexuses[ii]->GetFaceuse()->GetFace() : NULL ;

          // when connected to an Edge
          if(pEdge)
            {
              // check ThisEdge for a ZoneTol3d
              if(   HasLocalZoneTol3d(pEdge, sLocalZoneTol3d, bUseNewTol)
                 && sLocalZoneTol3d != SM_UNINIT_TOL)
                { return(TRUE) ; }

            } // end Vertex connected to Edge check

          // when connected to a Face (through an Edge or directly)
          if(pFace)
            {
              // check ThisFace for a ZoneTol3d
              if(   HasLocalZoneTol3d(pFace, sLocalZoneTol3d, bUseNewTol)
                 && sLocalZoneTol3d != SM_UNINIT_TOL)
                { return(TRUE) ; }

             } // end Vertex connected to Face check

        } // end iter every vertexuse
    } // end Vertex Branch - setting sMaxZoneTol3d

  // arrive here when no UpDim objects are using a LocalZoneValue (expected common case)
  return(FALSE) ;

} // end SmTol::HasUpDimLocalZoneTol3d
#endif // NOT_BEING_USED // not yet used - perhaps obsolete 

/*******************************************************************//**
PURPOSE: Fetch pObj->GetLocalZoneTol3d value

NOTES: return TRUE for pObj == NULL or point
***********************************************************************/
#ifdef NOT_BEING_USED // not yet used - perhaps obsolete 
SmBoolean SmTol::HasZoneTol3d
 (const SmObject * pObj,               // in : Vertex, Edge, or Face to target
  SmZoneTol3d    & rZoneTol3d,         // out: When TRUE is returned, The object's ZoneTol3dValue
  SmBoolean        bUseNewTol)         // in : for internal use only - Always use default values
                                       //      TRUE = use NewTolerance Model, FALSE=use previous behavior,
                                       //      default:[SM_USE_NEWTOL ? TRUE : FALSE]
{
  // init output
  rZoneTol3d = 0.0 ;

  // no work - not using NewTol
  if(bUseNewTol == FALSE)
    { return(FALSE) ; }                  

  // Only Vertices, Edges, and Faces have LocalZoneTol3d values
  rZoneTol3d =   pObj->IsKindOf(SmVertex_TYPE) ? ((SmVertex*)pObj)->GetTolerance()
               : pObj->IsKindOf(SmEdge_TYPE)   ? ((SmEdge*)pObj)->GetTolerance()
               : pObj->IsKindOf(SmFace_TYPE)   ? ((SmFace*)pObj)->GetTolerance()
               : 0.0 ;

  // arrive here when pObj has no LocalXefZoneTol3d value
  return(rZoneTol3d != 0.0) ;

} // end sm_HasLocalZoneTol3d
#endif // NOT_BEING_USED - perhaps obsolete 

/*******************************************************************//**
PURPOSE: return TRUE = pCurve(sIvl)->BBox.GetMaxDim < GetXSectTol3d(pCurve)

NOTES: return TRUE for pObj == NULL 
***********************************************************************/
SmBoolean SmTol::IsDegenerate      
 (const SmCurve  * pCrv, SmExtent1d sIvl, SmXSectTol3d * pMyXSectTol3d)
{ 
  if(pCrv == NULL) 
    { return(TRUE) ; }

  SmZoneTol3d  sZoneTol3d  = SmTol::GetZoneTol3d(pCrv) ;
  SmXSectTol3d sXSectTol3d = (pMyXSectTol3d != NULL) ? *pMyXSectTol3d : SmTol::GetXSectTol3d(sZoneTol3d, sZoneTol3d) ;

  SmBoolean bRtn = pCrv->IsDegenerate(sXSectTol3d, &sIvl) ;
  return( bRtn ) ;

} // end SmTol::IsDegenerate  Curve(sIvl)

/*******************************************************************//**
PURPOSE: return TRUE = pVec.Length() < GetXSectTol3d()

NOTES: return TRUE for pVec == NULL 
***********************************************************************/
SmBoolean SmTol::IsDegenerate      
 (const SmVector2d sVec, SmXSectTol3d * pMyXSectTol3d)
{ 
  SmXSectTol3d sXSectTol3d = (pMyXSectTol3d != NULL) ? *pMyXSectTol3d : SmTol::GetXSectTol3d() ;

  SmBoolean bRtn = sVec.IsZero(sXSectTol3d) ;
  return( bRtn ) ;

} // end SmTol::IsDegenerate  SmVector2d

/*******************************************************************//**
PURPOSE: return TRUE = pVec.Length() < GetXSectTol3d()

NOTES: return TRUE for pVec == NULL 
***********************************************************************/
SmBoolean SmTol::IsDegenerate      
 (const SmVector3d sVec, SmXSectTol3d * pMyXSectTol3d)
{ 
  SmXSectTol3d sXSectTol3d = (pMyXSectTol3d != NULL) ? *pMyXSectTol3d : SmTol::GetXSectTol3d() ;

  SmBoolean bRtn = sVec.IsZero(sXSectTol3d) ;
  return( bRtn ) ;

} // end SmTol::IsDegenerate  SmVector3d

/*******************************************************************//**
PURPOSE: return TRUE = pObj->BBox.GetMaxDim < GetXSectTol3d(pObj)

NOTES: return TRUE for pObj == NULL or point
***********************************************************************/
SmBoolean SmTol::IsDegenerate      // rtn: TRUE= BBox of all pObj->geometry.MaxDim < GetScaledZero(pObj)
 (const SmObject * pObj,           // in : target topology or geometry object
  SmXSectTol3d   * pMyXSectTol3d)  // in : opt XSectTol3d, SM_USE_DEFAULT = use SmTol::GetXSectTol(pObj, pObj)
{
  // check state
  if(pObj == NULL)
    { return TRUE ; }

  // return and local values
  SmZoneTol3d  sObjZoneTol3d  = SmTol::GetZoneTol3d(pObj) ; 
  SmXSectTol3d sObjXSectTol3d = (pMyXSectTol3d != NULL) ? *pMyXSectTol3d : SmTol::GetXSectTol3d(sObjZoneTol3d, sObjZoneTol3d) ;

  // classify type - for upcoming switch statement
  if(pObj->IsKindOf(SmPolyEdge_TYPE)){ return( ((SmPolyEdge *) pObj)->IsDegenerate(pMyXSectTol3d) ); }
  else if(pObj->IsKindOf(SmPoint3d_TYPE)) { return( TRUE ) ; }
  else if(pObj->IsKindOf(SmCurve_TYPE))   { return( ((SmCurve *)pObj)->IsDegenerate(sObjXSectTol3d) ) ; }
  else if(pObj->IsKindOf(SmSurface_TYPE)) { return( ((SmSurface *)pObj)->IsDegeneratePoint(sObjXSectTol3d) ) ; }
  else if(pObj->IsKindOf(SmCurveClassification_TYPE)) { return( ((SmCurveClassification *)pObj)->GetCurve()->IsDegenerate(sObjXSectTol3d) ) ; }
  else if(pObj->IsKindOf(SmCurveInterval_TYPE))       { return( ((SmCurveInterval *)pObj)->GetCurveClassification()->GetCurve()->IsDegenerate(sObjXSectTol3d,
                                                                    &((SmCurveInterval *)pObj)->GetInterval()) ) ; }
  else if(pObj->IsKindOf(SmPointClassification_TYPE)) { return( TRUE ) ; }

  else if(pObj->IsKindOf(SmVertex_TYPE )) { return( TRUE ) ; }
  else if(pObj->IsKindOf(SmEdge_TYPE))    { SmExtent1d domain    = ((SmEdge *)pObj)->GetInterval(); return( ((SmEdge *)pObj)->GetCurve()->IsDegenerate((double)sObjXSectTol3d,&domain)); }
  else if(pObj->IsKindOf(SmFace_TYPE))    { SmExtent2d uv_domain = ((SmFace *)pObj)->GetUVDomain(); return( ((SmFace *)pObj)->GetSurface()->IsDegeneratePoint((double)sObjXSectTol3d,&uv_domain)); }
  else if(pObj->IsKindOf(SmBrep_TYPE))    { return( ((SmBrep *)pObj)->IsDegenerate(sObjXSectTol3d) ) ; }
  else if(pObj->IsKindOf(SmRegion_TYPE))  { return( ((SmRegion *)pObj)->IsDegenerate(sObjXSectTol3d) ) ; }
  else if(pObj->IsKindOf(SmShell_TYPE))   { return( ((SmShell *)pObj)->IsDegenerate(sObjXSectTol3d) ) ; }


  else return(FALSE) ;

} // end SmTol::IsDegenerate

/*******************************************************************//**
PURPOSE: return TRUE = pObj->BBox.GetMaxDim < GetZoneTol3d(pObj)

NOTES: return TRUE for pObj == NULL or point
***********************************************************************/
SmBoolean SmTol::IsPointSized      // rtn: TRUE= BBox of all pObj->geometry.MaxDim < GetScaledZero(pObj)
 (const SmObject * pObj,           // in : target topology or geometry object
  SmZoneTol3d    * pMyZoneTol3d)   // in : opt ZoneTol3d, SM_USE_DEFAULT = use SmTol::GetZoneTol3d(pObj)
{
  // no work - no obj
  if(pObj == NULL)
    { return TRUE ; }

  // locals
  SmZoneTol3d sZoneTol3d = (pMyZoneTol3d != NULL) ? *pMyZoneTol3d : GetZoneTol3d(pObj) ;

  // switch on Object type to check size
  if     (pObj->IsKindOf(SmPoint3d_TYPE)) { return( TRUE ) ; }
  else if(pObj->IsKindOf(SmCurve_TYPE))   { return( ((SmCurve *)pObj)->IsDegenerate(sZoneTol3d) ) ; }
  else if(pObj->IsKindOf(SmSurface_TYPE)) { return( ((SmSurface *)pObj)->IsDegeneratePoint(sZoneTol3d) ) ; }
  else if(pObj->IsKindOf(SmCurveClassification_TYPE)) { return( ((SmCurveClassification *)pObj)->GetCurve()->IsDegenerate(sZoneTol3d) ) ; }
  else if(pObj->IsKindOf(SmCurveInterval_TYPE))       { return( ((SmCurveInterval *)pObj)->GetCurveClassification()->GetCurve()->IsDegenerate(sZoneTol3d,
                                                                    &((SmCurveInterval *)pObj)->GetInterval()) ) ; }
  else if(pObj->IsKindOf(SmPointClassification_TYPE)) { return( TRUE ) ; }

  else if(pObj->IsKindOf(SmVertex_TYPE )) { return( TRUE ) ; }
  else if(pObj->IsKindOf(SmEdge_TYPE))    { SmExtent1d domain = ((SmEdge *)pObj)->GetInterval(); return( ((SmEdge *)pObj)->GetCurve()->IsDegenerate(sZoneTol3d,&domain)); }
  else if(pObj->IsKindOf(SmFace_TYPE))    { SmExtent2d uv_domain = ((SmFace *)pObj)->GetUVDomain(); return( ((SmFace *)pObj)->GetSurface()->IsDegeneratePoint(sZoneTol3d,&uv_domain)); }
  else if(pObj->IsKindOf(SmBrep_TYPE))    { return( ((SmBrep *)pObj)->IsDegenerate(sZoneTol3d) ) ; }
  else if(pObj->IsKindOf(SmRegion_TYPE))  { return( ((SmRegion *)pObj)->IsDegenerate(sZoneTol3d) ) ; }
  else if(pObj->IsKindOf(SmShell_TYPE))   { return( ((SmShell *)pObj)->IsDegenerate(sZoneTol3d) ) ; }


  else return(FALSE) ;

} // end SmTol::IsPointSized

/*******************************************************************//**
PURPOSE: IsClosed but !IsDegenerate

NOTES:
***********************************************************************/
SmBoolean SmTol::IsClosed    
 (const SmObject * pObj, 
  SmXSectTol3d   * pOptOverrideXSectTol3d)
{ 
  // no work
  if(pObj == NULL)
    { return(FALSE) ;  }

  SmZoneTol3d  sZoneTol3d  = GetZoneTol3d(pObj) ; 
  SmXSectTol3d sXSectTol3d = pOptOverrideXSectTol3d ? * pOptOverrideXSectTol3d : GetXSectTol3d(sZoneTol3d, sZoneTol3d) ;
  double       dXSectTol3d = sXSectTol3d ; 

  // Obj types
  if(pObj->IsKindOf(SmCurve_TYPE))   { return ((SmCurve *)pObj)->IsClosed(((SmCurve *)pObj)->GetNaturalInterval(), sXSectTol3d) ; }
  if(pObj->IsKindOf(SmSurface_TYPE)) { return (   ((SmSurface *)pObj)->IsClosed(((SmSurface *)pObj)->GetNaturalUVDomain(), SM_SP_U, &dXSectTol3d) 
                                               || ((SmSurface *)pObj)->IsClosed(((SmSurface *)pObj)->GetNaturalUVDomain(), SM_SP_V, &dXSectTol3d) ) ;
                                     }

  // arrive here - obj doesn't have a closed property
  return(FALSE) ;

} // end SmTol::IsClosed


/*******************************************************************//**
PURPOSE: GetGapGain from System

NOTES: GapGsain = fudge scale size used to make sure an Obj's stored 
                  ZoneTol3d is a tad larger than its largest Gap3d value/2.0
***********************************************************************/
double SmTol::GetGapGain()
{
  return(s_TolGapGain) ;

} // end SmTol::GetGapGain

/*******************************************************************//**
PURPOSE: GetZoneTol3d from System

NOTES:
***********************************************************************/
SmZoneTol3d SmTol::GetZoneTol3d
 (SmSmallTopoType eIsSmallTopology) 
{ 
  if (s_ZoneTol3d == SM_UNINIT_TOL) 
    {
      smos_AssertErrorMessage(SM_ERR_ASSERT_FAILURE, FILE_NAME, LINE_NUMBER, _T("Uninit-Tol"), NULL, 1, FUNC_NAME);
      return((SmZoneTol3d)SM_UNINIT_TOL);
    }
  else if (eIsSmallTopology == SM_ST_SMALL_SIZE || eIsSmallTopology == SM_ST_FORCE_SMALL_SIZE) 
    {
      return((SmZoneTol3d)s_LargeSmallSizeRatio * (s_ZoneTol3d));
    }

  return s_ZoneTol3d;

} // end SmTol::GetZoneTol3d - System

/*******************************************************************//**
PURPOSE: GetZoneTol3d from Context

NOTES:
***********************************************************************/
SmZoneTol3d SmTol::GetZoneTol3d
 (const SmContext * cpContext,         // in : 
  SmSmallTopoType   eIsSmallTopology)  // NotUsed: in : eIsSmallTopology  
{
  SM_REF1(eIsSmallTopology) ;
  if (s_ZoneTol3d == SM_UNINIT_TOL) 
    {
      smos_AssertErrorMessage(SM_ERR_ASSERT_FAILURE, FILE_NAME, LINE_NUMBER, _T("Uninit-Tol"), NULL, 1, FUNC_NAME);
      return((SmZoneTol3d)SM_UNINIT_TOL);
    }
    
  return(cpContext ? cpContext->m_sThisZoneTol3d : s_ZoneTol3d) ;

} // end SmTol::GetZoneTol3d - Context

/*******************************************************************//**
PURPOSE: GetZoneTol3d from Brep

NOTES:
***********************************************************************/
SmZoneTol3d SmTol::GetZoneTol3d
 (const SmBrep  * cpBrep,            // in : target Brep
  SmSmallTopoType eIsSmallTopology)  // NotUsed: in : eIsSmallTopology
{
  SM_REF1(eIsSmallTopology) ;
  double dTol = cpBrep->GetTolerance();

  if (dTol == SM_UNINIT_TOL) 
    {
      smos_AssertErrorMessage(SM_ERR_ASSERT_FAILURE, FILE_NAME, LINE_NUMBER, _T("Uninit-Tol"), NULL, 1, FUNC_NAME);
      return((SmZoneTol3d)SM_UNINIT_TOL);
    }
    
  return dTol;

} // end SmTol::GetZoneTol3d - Brep

/*******************************************************************//**
PURPOSE: GetZoneTol3d from BrepData

NOTES:
***********************************************************************/
SmZoneTol3d SmTol::GetZoneTol3d
 (const SmBrepData  * cpBrepData,    // in : target Brep
  SmSmallTopoType eIsSmallTopology)  // NotUsed: in : eIsSmallTopology
{
  SM_REF1(eIsSmallTopology) ;
  double dTol = cpBrepData->m_sZoneTol3d;

  if (dTol == SM_UNINIT_TOL) 
    {
      smos_AssertErrorMessage(SM_ERR_ASSERT_FAILURE, FILE_NAME, LINE_NUMBER, _T("Uninit-Tol"), NULL, 1, FUNC_NAME);
      return((SmZoneTol3d)SM_UNINIT_TOL);
    }
    
  return dTol;

} // end SmTol::GetZoneTol3d - Brep

/*******************************************************************//**
PURPOSE: GetZoneTol3d from Object

NOTES:
***********************************************************************/
SmZoneTol3d SmTol::GetZoneTol3d
 (const SmObject * cpObj,              // in : Target Object
  SmSmallTopoType  eIsSmallTopology)   // NotUsed: in : eIsSmallTopology
{
  SM_REF1(eIsSmallTopology) ;
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  if(bDebugMe)
    {
      SmTol::sm_TemplateTrial() ;
    }
#endif // SM_DEBUG_CODE

#ifdef SM_USE_NEWTOL
  SM_NEWTOL_LINE SmSmallTopoType eIsSmall =   eIsSmallTopology == SM_ST_FORCE_NORMAL_SIZE ? SM_ST_FORCE_NORMAL_SIZE
  SM_NEWTOL_LINE                            : eIsSmallTopology == SM_ST_FORCE_SMALL_SIZE  ? SM_ST_FORCE_SMALL_SIZE
  SM_NEWTOL_LINE                            : cpObj->IsKindOf(SmTopology_TYPE)              
  SM_NEWTOL_LINE                                 ?  ( ((const SmTopology &)cpObj)->IsSmallTopology() ? SM_ST_SMALL_SIZE : SM_ST_NORMAL_SIZE)
  SM_NEWTOL_LINE                                 : eIsSmallTopology ;
#else // SM_USE_OLDTOL
  SM_OLDTOL_LINE SmSmallTopoType eIsSmall = SM_ST_NORMAL_SIZE ; 
#endif // SM_USE_OLDTOL

  // For Topology objects with tolerance (SmFace, SmEdge, SmVertex, SmVertexuse, SmEdgeuse)
  if(cpObj == NULL)                         { return( GetZoneTol3d(eIsSmall)) ; }
                                                                  
  else if(cpObj->IsKindOf(SmTopology_TYPE)) { const SmTopology *pTopo = static_cast<const SmTopology *>(cpObj) ;
                              SM_OLDTOL_LINE return( pTopo->GetTolerance() ) ;
                               
                              // LINUX does not like SM_SCALE_WHEN_SMALL
                              //SM_NEWTOL_LINE return(  pTopo->GetBrep()    ? SmTol::GetZoneTol3d(pTopo->GetBrep(), eIsSmall) 
                              //SM_NEWTOL_LINE        : pTopo->GetContext() ? SmTol::GetZoneTol3d(pTopo->GetContext(), eIsSmall)
                              //SM_NEWTOL_LINE        :                      SM_SCALE_WHEN_SMALL(eIsSmall, s_ZoneTol3d) ) ;
                                             }

  // For Geometry objects (SmSurface, SmCurve)    note: SmPoint3d is handled as an overloaded SmTol::GetZoneTol3d() method                                                                         
  else if(cpObj->IsKindOf(SmSurface_TYPE))   { const SmSurface * pSurf = static_cast<const SmSurface *>(cpObj) ;
                              SM_OLDTOL_LINE   SmFace          * pFace = pSurf->GetFace() ;
                              SM_OLDTOL_LINE   return(  pFace ? pFace->GetTolerance() : GetZoneTol3d()) ;

                              // LINUX does not like SM_SCALE_WHEN_SMALL
                              //SM_NEWTOL_LINE  return(  pSurf->GetBrep()    ? SmTol::GetZoneTol3d(pSurf->GetBrep(), eIsSmall) 
                              //SM_NEWTOL_LINE         : pSurf->GetContext() ? SmTol::GetZoneTol3d(pSurf->GetContext(), eIsSmall)
                              //SM_NEWTOL_LINE         :                      SM_SCALE_WHEN_SMALL(eIsSmall, s_ZoneTol3d) ) ;
                                             }
                                                                        
  else if(cpObj->IsKindOf(SmCurve_TYPE))     { const SmCurve * pCurve = static_cast<const SmCurve *>(cpObj) ;
                              SM_OLDTOL_LINE   const SmEdge  * pEdge = pCurve->GetEdge() ;
                              SM_OLDTOL_LINE   return(  pEdge ? pEdge->GetTolerance() : GetZoneTol3d()) ;

                              // LINUX does not like SM_SCALE_WHEN_SMALL
                              //SM_NEWTOL_LINE  return(  pCurve->GetBrep()    ? SmTol::GetZoneTol3d(pCurve->GetBrep(), eIsSmall) 
                              //SM_NEWTOL_LINE         : pCurve->GetContext() ? SmTol::GetZoneTol3d(pCurve->GetContext(), eIsSmall)
                              //SM_NEWTOL_LINE         :                      SM_SCALE_WHEN_SMALL(eIsSmall, s_ZoneTol3d) ) ;
                                             }
                                             
  // for classes derived from SmObject with stored ThisZoneTol3d values
  else if(cpObj->IsKindOf(SmBrep_TYPE))                { return  GetZoneTol3d(static_cast<const SmBrep *>(cpObj), eIsSmall) ; }
  else if(cpObj->IsKindOf(SmBrepData_TYPE))            { return  GetZoneTol3d(static_cast<const SmBrepData *>(cpObj), eIsSmall) ; }
  else if(cpObj->IsKindOf(SmCurveClassification_TYPE)) { return  GetSrcZoneTol3d(static_cast<const SmCurveClassification *>(cpObj)) ; }

  else // branch not an object that we planned to have a ZoneTol3d
    { 
      WARN(_T("SmTol::GetZoneTol3d called for an unsupported object - may need to call GetSrcZoneTol3d() or IGetObjZoneTol3d()")) ;
      if (s_ZoneTol3d == SM_UNINIT_TOL) {
          smos_AssertErrorMessage(SM_ERR_ASSERT_FAILURE, FILE_NAME, LINE_NUMBER, _T("Uninit-Tol"), NULL, 1, FUNC_NAME);
          return((SmZoneTol3d)SM_UNINIT_TOL);
      }
      else if (eIsSmall == SM_ST_SMALL_SIZE || eIsSmall == SM_ST_FORCE_SMALL_SIZE) {
          return((SmZoneTol3d)s_LargeSmallSizeRatio * (s_ZoneTol3d));
      }

      return s_ZoneTol3d;
#if 0
      // LINUX does not like SM_SCALE_WHEN_SMALL
      return SM_SCALE_WHEN_SMALL(eIsSmall, s_ZoneTol3d) ;
#endif
    }

} // end SmTol::GetZoneTol3d - Object

/*******************************************************************//**
PURPOSE: SetZoneTol3d for Object

NOTES:   OldTol Model: 
           Topology Objects       - Set cpObj ZoneTol3d value
           Owned Geometry Objects - Set cpObj->Owner ZoneTol3d value
           SmCurveClassification  - Set cpObj->m_sSrcZoneTol3d

         NewTol Model:
           Topology Objects       - Set cpObj SmallTopology value when needed 
           Owned Geometry Objects - Set cpObj->Owner ZoneTol3d value
***********************************************************************/
void SmTol::SetZoneTol3d
 (const SmObject * cpObj,              // in : Target Object
  SmZoneTol3d      sNewZoneTol3d,      // in : New ZoneTol3d value to save (if possible)
  SmSmallTopoType  eIsSmallTopology)   // NotUsed: in : default:[SM_ST_NO_CHANGE]
{
  SM_REF1(eIsSmallTopology) ;
  // for topology objects - set smallTopoType
#ifdef SM_USE_NEWTOL
  SmTopology *pTopo = SM_CAST_PTR( SmTopology, cpObj );
  if(eIsSmallTopology != SM_ST_NO_CHANGE && pTopo != NULL)
    { 
      ((SmTopology *)cpObj)->SetIsSmallTopology(eIsSmallTopology) ; 
    }
#endif // no SM_USE_NEWTOL

  // for topology objects
  if(cpObj->IsKindOf(SmTopology_TYPE))       { SmTopology *pTopo = (SmTopology *)cpObj ;
                                               pTopo->SetTolerance(sNewZoneTol3d) ; 
                                             }

  // For Geometry objects (SmSurface)    
  else if(cpObj->IsKindOf(SmSurface_TYPE))   { SmSurface                 * pSurf = (SmSurface *)cpObj ;
                                               SM_OLDTOL_LINE  SmFace    * pFace = pSurf->GetFace() ;
                                               SM_OLDTOL_LINE  if(pFace) pFace->SetTolerance(sNewZoneTol3d) ;
                                             }
  // For Geometry objects (SmCurve)    
  else if(cpObj->IsKindOf(SmCurve_TYPE))     { SmCurve                * pCurve = (SmCurve *)cpObj ;
                                               SM_OLDTOL_LINE  SmEdge * pEdge  = (SmEdge*) pCurve->GetEdge() ;
                                               SM_OLDTOL_LINE  if(pEdge) pEdge->SetTolerance(sNewZoneTol3d) ;
                                             }
                                             
  // for classes derived from SmObject with stored ThisZoneTol3d values
  else if(cpObj->IsKindOf(SmBrep_TYPE))      { SmBrep * pBrep = (SmBrep *)cpObj ;
                                               SM_OLDTOL_LINE  if(pBrep) pBrep->SetTolerance(sNewZoneTol3d) ;
                                             }


  else if(cpObj->IsKindOf(SmCurveClassification_TYPE)) { SmCurveClassification * pCrvClassification = (SmCurveClassification *) cpObj ; 
                                                         if(pCrvClassification) pCrvClassification->SetSrcZoneTol3d(sNewZoneTol3d) ;
                                                       }

  else // branch not an object that we planned to have a ZoneTol3d
    { 
      WARN(_T("SmTol::SetZoneTol3d called for an unsupported object - may need to call SmTol::SetModelSizeEstimate()")) ;
    }

} // end SmTol::SetZoneTol3d - Object

/*******************************************************************//**
PURPOSE: GetZoneTol3d from Point

NOTES:
***********************************************************************/
SmZoneTol3d SmTol::GetZoneTol3d       // rtn: if(Brep) GetZoneTol3d(Brep)
 (const SmPoint3d * cpPoint,          // NotUsed: in : cpPoint
  SmBrep          * pOptBrep,         // in : elseif(Context) GetZoneTol3d(Context)
  SmContext       * pOptContext,      // in : else GetZoneTol3d() for System    
  SmSmallTopoType   eIsSmallTopology) // in : SM_ST_SMALL_SIZE=for Pinhole in Battleship, default:[SM_ST_NORMAL_SIZE] 
{
  SM_REF1(cpPoint) ;
  return(  pOptBrep    ? SmTol::GetZoneTol3d(pOptBrep, eIsSmallTopology)
         : pOptContext ? SmTol::GetZoneTol3d(pOptContext, eIsSmallTopology)
         : (eIsSmallTopology == SM_ST_SMALL_SIZE || eIsSmallTopology == SM_ST_FORCE_SMALL_SIZE) ? (SmZoneTol3d)(s_LargeSmallSizeRatio * (double)s_ZoneTol3d) : s_ZoneTol3d );

#if 0
  // LINUX does not like SM_SCALE_WHEN_SMALL
         : SM_SCALE_WHEN_SMALL(eIsSmallTopology, s_ZoneTol3d) ) ;
#endif

} // end SmTol::GetZoneTol3d - Point

// For classes not derived from SmObject storing ThisZoneTol3d
SmZoneTol3d  SmTol::GetSrcZoneTol3d(const SmCurveClassification     * cpCrvCl) { return cpCrvCl ? cpCrvCl->GetSrcZoneTol3d()         : GetZoneTol3d() ; }
SmZoneTol3d  SmTol::GetSrcZoneTol3d(const SmPointClassification     * cpPtCl)  { return cpPtCl  ? cpPtCl->GetSrcZoneTol3d()          : GetZoneTol3d() ; }
SmZoneTol3d  SmTol::GetSrcZoneTol3d(const SmCurveInterval           * cpIvlCl) { return cpIvlCl ? cpIvlCl->GetCurveClassification()->GetSrcZoneTol3d() : GetZoneTol3d() ; }

SmZoneTol3d  SmTol::GetObjZoneTol3d(const SmCurveInterval           * cpIvlCl) { return cpIvlCl ? cpIvlCl->m_vMid.GetObjZoneTol3d() : GetZoneTol3d(); }
SmZoneTol3d  SmTol::GetObjZoneTol3d(const SmPointClassification     * cpPtCl) { return cpPtCl ? cpPtCl->GetObjZoneTol3d() : GetZoneTol3d(); }

SmZoneTol3d  SmTol::GetSrcZoneTol3d(const SmLineSegClassification   * cpCrvCl) { return cpCrvCl ? cpCrvCl->GetSrcZoneTol3d()         : GetZoneTol3d() ; }
SmZoneTol3d  SmTol::GetSrcZoneTol3d(const SmPolyPointClassification * cpPtCl)  { return cpPtCl  ? cpPtCl->GetSrcZoneTol3d()          : GetZoneTol3d() ; }
SmZoneTol3d  SmTol::GetSrcZoneTol3d(const SmLineSegInterval         * cpIvlCl) { return cpIvlCl ? cpIvlCl->GetLineSegClassification()->GetSrcZoneTol3d() : GetZoneTol3d() ; }
SmZoneTol3d  SmTol::GetObjZoneTol3d(const SmPolyPointClassification * cpPPtCl) { return cpPPtCl ? GetZoneTol3d(cpPPtCl->GetObject()) : GetZoneTol3d() ; }
SmZoneTol3d  SmTol::GetObjZoneTol3d(const SmLineSegInterval         * cpIvlCl) { return cpIvlCl ? GetZoneTol3d(cpIvlCl->m_vMid.GetObject()) : GetZoneTol3d() ; }



/*******************************************************************//**
PURPOSE: GetApproxTol3d from System, Context, Brep, Object

NOTES:
***********************************************************************/
SmApproxTol3d SmTol::GetApproxTol3d(SmSmallTopoType eIsSmallTopology)                              { return( (SmApproxTol3d)(.5 * GetZoneTol3d(eIsSmallTopology) ) ) ; }
SmApproxTol3d SmTol::GetApproxTol3d(const SmContext * cpContext, SmSmallTopoType eIsSmallTopology) { return( (SmApproxTol3d)(.5 * GetZoneTol3d(cpContext, eIsSmallTopology) ) ) ; }

SmApproxTol3d SmTol::GetApproxTol3d(const SmBrep    * cpBrep,    SmSmallTopoType eIsSmallTopology) { return( (SmApproxTol3d)(.5 * GetZoneTol3d(cpBrep, eIsSmallTopology) ) ) ; }

SmApproxTol3d SmTol::GetApproxTol3d(const SmObject  * cpObj,     SmSmallTopoType eIsSmallTopology) 
{ 
  // For those classes derived from SmObject that store a ThisApproxTol3d 
  if     (cpObj == NULL) { return .5 * GetZoneTol3d(eIsSmallTopology) ; }

  if(cpObj->IsKindOf(SmFilletCorner_TYPE)) { return( ((SmFilletCorner*)cpObj)->GetThisApproxTol3d() ) ; }
  if(cpObj->IsKindOf(SmFilletSolver_TYPE)) { return( ((SmFilletSolver*)cpObj)->GetThisApproxTol3d() ) ; }


  // for all derived classes of SmObject that don't store a ThisApproxTol3d
  return( .5 * GetZoneTol3d(cpObj, eIsSmallTopology) ) ;  

} // end SmTol::GetApproxTol3d from Objects

// for classes that store ThisApproxTol3d
SmApproxTol3d SmTol::GetApproxTol3d(const SmOffsetGeometryCreation * cpCreation)             { return cpCreation            ? (SmApproxTol3d)cpCreation->GetThisApproxTol3d() : GetApproxTol3d(); }

SmApproxTol3d SmTol::GetApproxTol3d(const SmHCR                    * cpHCR)                  { return cpHCR                 ? (SmApproxTol3d)cpHCR->GetThisApproxTol3d() : GetApproxTol3d(); }
SmApproxTol3d SmTol::GetApproxTol3d(const SmPolyIntersector        * cpPolyIntersector)      { return cpPolyIntersector     ? (SmApproxTol3d)cpPolyIntersector->GetThisApproxTol3d() : GetApproxTol3d(); }
SmApproxTol3d SmTol::GetApproxTol3d(const SmSurfaceIntersector     * cpSurfaceIntersector)   { return cpSurfaceIntersector  ? (SmApproxTol3d)cpSurfaceIntersector->GetThisApproxTol3d() : GetApproxTol3d(); }
SmApproxTol3d SmTol::GetApproxTol3d(const SmSurfaceTracer          * cpSurfaceTracer)        { return cpSurfaceTracer       ? (SmApproxTol3d)cpSurfaceTracer->GetThisApproxTol3d() : GetApproxTol3d(); }
SmApproxTol3d SmTol::GetApproxTol3d(const SmTopologyIntersector    * cpTopologyIntersector)  { return cpTopologyIntersector ? (SmApproxTol3d)cpTopologyIntersector->GetThisApproxTol3d() : GetApproxTol3d(); }

/*******************************************************************//**
PURPOSE: GetXSectTol3d from System, Context, Brep, Object

NOTES:
***********************************************************************/
SmXSectTol3d SmTol::GetXSectTol3d(SmSmallTopoType eIsSmallTopology)                                { return( 2 * GetZoneTol3d(eIsSmallTopology) ) ; }
SmXSectTol3d SmTol::GetXSectTol3d(const SmContext  * cpContext,  SmSmallTopoType eIsSmallTopology) { return( 2 * GetZoneTol3d(cpContext,  eIsSmallTopology) ) ; }
SmXSectTol3d SmTol::GetXSectTol3d(const SmBrep     * cpBrep,     SmSmallTopoType eIsSmallTopology) { return( 2 * GetZoneTol3d(cpBrep,     eIsSmallTopology) ) ; } 
SmXSectTol3d SmTol::GetXSectTol3d(const SmBrepData * cpBrepData, SmSmallTopoType eIsSmallTopology) { return( 2 * GetZoneTol3d(cpBrepData, eIsSmallTopology) ) ; } 

SmXSectTol3d SmTol::GetXSectTol3d
 (const SmObject  * cpObj1,             // in : 
  const SmObject  * cpObj2,             // in : 
  SmContext       * pOptContext,        // NotUsed: in : pOptContext
  SmSmallTopoType   eIsSmallTopology)   // in : 
{
  SM_REF1(pOptContext) ;
  // force small scale when appropriate
#ifdef SM_USE_NEWTOL
  SM_NEWTOL_LINE eIsSmallTopology =   cpObj == NULL ? eIsSmallTopology
  SM_NEWTOL_LINE                    : (   (cpObj1->IsKindOf(SmTopology_TYPE) && ((SmTopology &)cpObj1)->IsSmallTopology())
  SM_NEWTOL_LINE                       || (cpObj2->IsKindOf(SmTopology_TYPE) && ((SmTopology &)cpObj2)->IsSmallTopology()) ) 
  SM_NEWTOL_LINE                    ? ( eIsSmallTopology == SM_ST_SMALL_SIZE ? SM_ST_FORCE_SMALL_SIZE : eIsSmallTopology)
  SM_NEWTOL_LINE                    : eIsSmallTopology ;
#endif // SM_USE_NEWTOL

  return(  GetZoneTol3d(cpObj1, eIsSmallTopology)
         + GetZoneTol3d(cpObj2, eIsSmallTopology) ) ;

} // end SmTol::GetXSectTol3d for object pairs


// for classes that store ThisTol3d values ... 
SmXSectTol3d SmTol::GetXSectTol3d(const SmPointClassification * cpPtCl) { return( cpPtCl ? GetXSectTol3d(cpPtCl->GetSrcZoneTol3d(),cpPtCl->GetObjZoneTol3d()) : GetXSectTol3d() ) ; }

// for classes that store ThisAngTolRad
double SmTol::GetAngTolRad(const SmFilletSolver         * cpFilletSolver)        { return cpFilletSolver        ? cpFilletSolver       ->GetThisAngTolRad() : GetAngTolRad(); }

double SmTol::GetAngTolRad(const SmHCR                  * cpHCR)                 { return cpHCR                 ? cpHCR                ->GetThisAngTolRad() : GetAngTolRad(); }
double SmTol::GetAngTolRad(const SmPolyIntersector      * cpPolyIntersector)     { return cpPolyIntersector     ? cpPolyIntersector    ->GetThisAngTolRad() : GetAngTolRad(); }
double SmTol::GetAngTolRad(const SmSurfaceIntersector   * cpSurfaceIntersector)  { return cpSurfaceIntersector  ? cpSurfaceIntersector ->GetThisAngTolRad() : GetAngTolRad(); }
double SmTol::GetAngTolRad(const SmSurfaceTracer        * cpSurfaceTracer)       { return cpSurfaceTracer       ? cpSurfaceTracer      ->GetThisAngTolRad() : GetAngTolRad(); }


/*******************************************************************//**
PURPOSE: use SM_USE_NEWTOL: Map Tol3d to UV space Tol2d for UVPoints on Surfaces
                            for the specified UVDir
         no  SM_USE_NEWTOL: return sTol2d = sTol3d (no mapping)

NOTES: 1. Tol2d depends on
          1a. UVDirection since rDU and rDV might have very different sized magnitudes
          1b. UVpt since rDU and rDV values change for different points in the UVDomain
          1c. Surface mapping from UVSpace to 3dSpace that defines rDU and rDV for point UVpt

       2. limit large Tol2d_U or Tol2d_V values found near singularities where Surf.1stDerivs go to zero.

       3. Tol2d(Tol3d, sUVPt, sUVDir) = dirU * Tol2d_U + dirV * Tol2d_V ;

                              Tol2d_U =   (Mag(rDU) > NearZero && !NearSingularityU)
                                        ? (Tol3d / Mag(rDU))
                                        : (Surf.UVDomain.U.Length/100) ;
                              Tol2d_V =   (Mag(rDV) > NearZero && !NearSingularityV)
                                        ? (Tol3d / Mag(rDV))
                                        : (Surf.UVDomain.V.Length/100) ;

          where :  Surf    = The surface mapping UV Pts to 3dPts
                   rUV     = [U V] a UV point within Surf.UVDomain.
                   rUVDir  = [dirU dirV] a unit vector2d specifying UV direction for this UVTol
                   rDU     = rDU of Surf.Evaluate1stDerivs(UVpt, rDU, rDV) ;
                   rDV     = rDV of Surf.Evaluate1stDerivs(UVpt, rDU, rDV) ;
***********************************************************************/
SmTol2d SmTol::MapTo2d
 (const SmTol3d    & rTol3d,     // in : 3d Tol value to map to UV Space
  const SmVector2d & rUV,        // in : UV point on Surface
  const SmVector2d & rUVDir,     // in : unit-vector UV direction of interest for this tolerance
  const SmSurface  & rSurface)   // in : Surface mapping UVTrimCurve to 3dSpace
{
  SM_ASSERT_TOL(rTol3d) ; 

  // locals
  SmPoint3d sPt, sDU, sDV ;

  // evaluate SurfacePt = rSurface(rUVTrimCurve(dParam))
  rSurface.Evaluate1stDerivatives(rUV, TRUE, TRUE, sPt, sDU, sDV) ;

  // locals
  SmExtent2d sUVDomain   = rSurface.GetNaturalUVDomain() ;
  double     dScaledZero = SmTol::GetScaledZero() ;
  double     dDUSize     = sDU.Length() ;
  double     dDVSize     = sDV.Length() ;

  // Scale dTol3d to dTolUV in the U and V directions - limit scaled dTolUV values when rDU or rDV are very small (near singularities)
  double dScaledU =   (   dDUSize < dScaledZero
                        /* || dDUSize < dDVSize / 1000.0 */ )    // Leave this test out. [B649]
                    ? ( sUVDomain.GetUInterval().GetLength() / 100.0)
                    : ( rTol3d.val / dDUSize ) ;
  double dScaledV =   (   dDVSize < dScaledZero
                        /* || dDVSize < dDUSize / 1000.0 */ )
                    ? ( sUVDomain.GetVInterval().GetLength() / 100.0)
                    : ( rTol3d.val / dDVSize ) ;

  // get directional Tol2d value when [dDirU dDirV] is not the zero vector
  //     otherwise limit Tol2d = Min(dScaledU, dScaledV)
  double dSize   = smos_Sqrt(rUVDir.x * rUVDir.x + rUVDir.y * rUVDir.y) ;
  SmTol2d sTolUV =   (dSize < dScaledZero)
                   ? smos_Min(dScaledU, dScaledV)
                   :   smos_Fabs(rUVDir.x)/dSize * dScaledU
                     + smos_Fabs(rUVDir.y)/dSize * dScaledV ;
  // all done
  return( sTolUV ) ;

} // SmTol::MapTo2d(rTol3d, DirU, DirV, rDU, rDV, rUVDomain)

/*******************************************************************//**
PURPOSE: use SM_USE_NEWTOL: convenience method to Map Tol3d to UV space
                              Tol2d for pts on Surfaces without UVDir
         no  SM_USE_NEWTOL: return sTol2d = rTol3d (no mapping)

NOTES: Passes the call along to Tol2d(dDirU, dDirV, rDU, rDV, rSurface.UVDomain)
***********************************************************************/
SmTol2d SmTol::MapTo2d
 (const SmTol3d    & rTol3d,     // in : 3d Tol value to map to UV Space
  const SmVector2d & rUV,        // in : UV point on Surface
  const SmSurface  & rSurface)   // in : Surface mapping UVTrimCurve to 3dSpace
{
  SM_ASSERT_TOL(rTol3d) ;

  // locals
  SmPoint3d sPt, sDU, sDV ;
  SmPoint2d sUVDir ;

  // evaluate SurfacePt = rSurface(rUVTrimCurve(dParam))
  rSurface.Evaluate1stDerivatives(rUV, TRUE, TRUE, sPt, sDU, sDV) ;

  // pick the max 1st deriv direction
  if(sDU.LengthSquared() > sDV.LengthSquared()) { sUVDir.Set(1.0, 0.0) ; }
  else                                          { sUVDir.Set(0.0, 1.0) ; }

  // pass the call along for Max 1st deriv (yields Min Tol2d value)
  return( MapTo2d(rTol3d, rUV, sUVDir, rSurface ) ) ;

} // end SmTol::MapTo2d(rTol3d, rDirVec, rSurface) - for max UV mapping direction

/*******************************************************************//**
PURPOSE: use SM_USE_NEWTOL: convenience method to Map Tol3d to UV space
                            Tol2d for UVTrimCurves on Surfaces in the
                            UVTrimCurve direction
         no  SM_USE_NEWTOL: return sTol2d = rTol3d (no mapping)


NOTES: Passes the call along to Tol2d(dDirU, dDirV, rDU, rDV, rSurface.UVDomain)
***********************************************************************/
SmTol2d SmTol::MapTo2d
 (const SmTol3d   & rTol3d,       // in : 3d Tol value to map to UV Space
  double            dParam,       // in : target UVTrimCurve param
  const SmCurve   & rTrimCurve,   // in : when Dim==2, TrimCurve is treated as a UVTrimCurve
                                  //           Dim==3, TrimCurve is evaluated and its pt and Tan are dropped to Surface UVSpace
  const SmSurface & rSurface)     // in : Surface mapping UVTrimCurve to 3dSpace
{
  SM_ASSERT_TOL(rTol3d) ;

  // locals
  SmPoint3d sPD[2] ;
  SmPoint2d sUV, sUVDir ;

  // evaluate SurfacePt = rSurface(rUVTrimCurve(dParam)) for UVTrimCurves and SmCrvOnSurfs
  if(   rTrimCurve.GetDim() == 2
     || (   rTrimCurve.IsKindOf(SmCrvOnSurf_TYPE)
         && &rSurface == ((SmCrvOnSurf &)rTrimCurve).GetBaseSurface()))
    {
      if( rTrimCurve.IsKindOf(SmCrvOnSurf_TYPE) ) 
        { ((SmCrvOnSurf &)rTrimCurve).GetUVCurve()->Evaluate(dParam, 1, TRUE, sPD) ; }
      else                                        
        { rTrimCurve.Evaluate(dParam, 1, TRUE, sPD) ; }

      sUV    = sPD[0] ;
      sUVDir = sPD[1] ;
    }
  else // general 3 dim Curve 
    {
      SmBoolean bSuccess ;
        SmBoolean bIsMulti;
      double    dGap = 0.0;

      rTrimCurve.Evaluate(dParam, 1, TRUE, sPD) ;
      rSurface.DropPoint(sPD[0], rSurface.GetNaturalUVDomain(), NULL, bSuccess, sUV, dGap, bIsMulti) ;
      rSurface.DropVectors(sUV, TRUE, TRUE, 1, &sPD[1], &sUVDir) ;
    }

  // pass the call along
  return( MapTo2d(rTol3d, sUV, sUVDir, rSurface ) ) ;

} // end SmTol::MapTo2d(rTol3d, rDirVec, rUV, rSurface)

/*******************************************************************//**
PURPOSE: use SM_USE_NEWTOL: convenience method to Map Tol3d to Param space
                              Tol1d for pts on Curves
         no  SM_USE_NEWTOL: return sTol1d = rTol3d (no mapping)

NOTES: Passes the call along to Tol1d(s!stDeriv, rCurve.NaturalInterval)
***********************************************************************/
SmTol1d SmTol::MapTo1d
 (SmTol3d       & rTol3d,     // in : 3d Tol value to map to Param Space
  double          dParam,     // in : point on Curve
  const SmCurve & rCurve)     // in : Curve mapping ParamSpace to 3dSpace
{
  SM_ASSERT_TOL(rTol3d) ;
  
  // locals
  SmPoint3d sPD[2] ;

  // evaluate CurvePt
  rCurve.Evaluate(dParam, 1, TRUE, sPD) ;

  // pass the call along
  return( MapTo1d(rTol3d, sPD[1], rCurve.GetNaturalInterval() ) ) ;

} // end SmTol::MapTo1d(rTol3d, dParam, rCurve)

/*******************************************************************//**
PURPOSE: use SM_USE_NEWTOL: convenience method to Map Tol3d to Param space
                              Tol1d for UVTrimCurves on Surfaces
         no  SM_USE_NEWTOL: return sTol1d = rTol3d (no mapping)

NOTES: Passes the call along to Tol1d(s1stDeriv, sNaturalInterval)
***********************************************************************/
SmTol1d SmTol::MapTo1d
 (SmTol3d         & rTol3d,       // in : 3d Tol value to map to Param Space
  double            dParam,       // in : target UVTrimCurve param
  const SmCurve   & rUVTrimCurve, // in : The UVTrimCurve
  const SmSurface & rSurface)     // in : Surface mapping UVTrimCurve to 3dSpace
{
  SM_ASSERT_TOL(rTol3d) ;
  
  // locals
  SmPoint3d sPD[2]  ;
  SmCrvOnSurf sCrvOnSurf((SmCurve &)rUVTrimCurve, (SmSurface &)rSurface) ;

  // evaluate CurvePt = rSurface(rUVTrimCurve(dParam))
  sCrvOnSurf.Evaluate(dParam, 1, TRUE, sPD) ;

  // pass the call along
  return( MapTo1d(rTol3d, sPD[1], rUVTrimCurve.GetNaturalInterval() ) ) ;

} // end SmTol::MapTo1d(rTol3d, Param, UVTrimCurve, Surface)

/*******************************************************************//**
PURPOSE: use SM_USE_NEWTOL: Map Tol3d to Param space
                              Tol1d for params on Curves
         no  SM_USE_NEWTOL: return sTol1d = rTol3d (no mapping)

NOTES: 1. Tol1d depends on the param value and the Curve mapping from Param Space to 3dSpace
       2. limit large Tol1d values where Curve.1stDerivs go to zero.
       3. Tol2d = (Mag(1stDerivs) > NearZero) ? (Tol3d / Mag(rDU)) : (Curve.NaturalInterval.Length/100) ;
***********************************************************************/
SmTol1d SmTol::MapTo1d
 (SmTol3d          & rTol3d,           // in : 3d Tol value to map to Param Space
  SmVector3d       & r1stDeriv,        // in : Curve 1st Derivative at the point where the tolerance is being used
  const SmExtent1d & rNaturalInterval) // in : Curve NaturalInterval - used to limit scaling sizes when r1stDeriv is very small (near singularities)
{
  SM_ASSERT_TOL(rTol3d) ;
  
  // locals
  double dScaledZero   = SmTol::GetScaledZero() ;
  double d1stDerivSize = r1stDeriv.Length() ;

  // Scale dTol3d to dTolUV in the U and V directions - limit scaled dTolUV values when rDU or rDV are very small (near singularities)
  SmTol1d sTol1d =   (d1stDerivSize < dScaledZero)
                   ? ( rNaturalInterval.GetLength() / 100.0)
                   : ( rTol3d.val / d1stDerivSize ) ;
  // all done
  return( sTol1d ) ;

} // SmTol::MapTo1d(rTol3d, 1stDeriv, NaturalInterval)

/*******************************************************************//**
PURPOSE: Update a topology object's tolerance value.

NOTES:
   bCheckGaps might be specified if, for instance, a Vertex's position
   might have changed a little.
***********************************************************************/
SmStatus SmTol::UpdateObjectTolerance
 (SmTopology * pTopo,      // in: topology object to update
  double       dThisGap,   // in: if there is a newly-discovered gap,
                           //     which could cause tol to increase
  double       dThisTol,   // in: any existing default to use
  SmBoolean    bCheckGaps) // in: TRUE = recompute gaps, FALSE=don't
                           //     Default:[FALSE]
{
  SM_ASSERT_TOL(dThisTol) ;
  
  // no work - no target object
  if ( pTopo == NULL )
    { return SM_SUCCESS; }

  double dTolToUse = smos_Max( 1.001*dThisGap, dThisTol ); // cbiTolProj: some places do 2*.

  // If both input gap and tol are zero, there's nothing to do.
  if ( dTolToUse < SM_EFF_ZERO )
    { return SM_ERR_INVALID_INPUT; }

  SM_TYPE eType = pTopo->GetType();
  switch ( eType )
    {
      case SmVertex_TYPE: { SmVertex * pVertex = SM_CAST_PTR( SmVertex, pTopo );
                            if ( pVertex == NULL )
                              { return SM_ERR_INVALID_INPUT; }
#ifdef SM_USE_OLDTOL
                            SM_OLDTOL_LINE pVertex->SetTolerance( dTolToUse, TRUE ); // TRUE: update only if larger.
#endif // no SM_USE_OLDTOL
                            // when asked - update tolerance from current gap sizes
                            if ( bCheckGaps )
                              {
                                // locals
                                SmEdge    * pE;
                                SmFace    * pF;
                                SmGapArray  sGaps;

                                // Get gaps to connected edges and faces
                                pVertex->GetMaxGap3d( &sGaps ) ;   // MaxGap3d = Max( Vertex/Edge Vertex/Face Gap )
                                ULONG ii, lNumGaps = sGaps.GetSize();

                                // for every vertex/ConnectedNeighbor gap
                                for ( ii=0; ii<lNumGaps; ii++ )
                                  {
                                    SmGap * pGap     = sGaps[ii];
                                    double  dGap = pGap->GetLength();

                                    // Note, SmBrep::UpdateAndValidateTolerances() checks against tol/2,
                                    // so have to double the gap (plus a little).
                                    pE = pGap->GetEdge();
                                    double dNewTol = 2.001*dGap;
                                    if ( pE != NULL )
                                      {
#ifdef SM_USE_OLDTOL
                                        SM_OLDTOL_LINE pE->SetTolerance( dNewTol, TRUE );   // TRUE: update only if larger.
#endif // SM_USE_OLDTOL
                                      }

                                    pF = pGap->GetFace();
#ifdef SM_USE_OLDTOL
                                    SM_OLDTOL_LINE if ( pF != NULL && pF->GetTolerance() < dGap - SM_EFF_ZERO )
                                    SM_OLDTOL_LINE   { pF->SetTolerance( dNewTol ); }
#endif // SM_USE_OLDTOL
                                  } // end iter every gap
                              }
                            break;
                          } // end SmVertex switch

      case SmEdge_TYPE:   { SmEdge * pEdge = SM_CAST_PTR( SmEdge, pTopo );
                            if ( pEdge == NULL )
                              { return SM_ERR_INVALID_INPUT; }
#ifdef SM_USE_OLDTOL
                            SM_OLDTOL_LINE pEdge->SetTolerance( dTolToUse, TRUE ); // TRUE: update only if larger.
#endif // SM_USE_OLDTOL

                            // We have to update lower-dimensional topology.  [cbiTolProj]
                            SmVertex *pVtx = pEdge->GetStartVertex();
                            UpdateObjectTolerance( pVtx, dThisGap, dThisTol );
                            pVtx = pEdge->GetOtherVertex( pVtx );
                            UpdateObjectTolerance( pVtx, dThisGap, dThisTol );

                            if ( bCheckGaps )
                              {
                                SmGapArray sGaps;
                                pEdge->GetMaxGap3d( &sGaps );
                                SmFace *pF;
                                ULONG ii, lNumGaps = sGaps.GetSize();
                                for ( ii=0; ii<lNumGaps; ii++ )
                                  {
                                    SmGap *pGap = sGaps[ii];
                                    double dGap = pGap->GetLength();

                                    pF = pGap->GetFace();
#ifdef SM_USE_OLDTOL
                                   SM_OLDTOL_LINE if ( pF != NULL && pF->GetTolerance() < dGap - SM_EFF_ZERO )
                                   SM_OLDTOL_LINE   { pF->SetTolerance( 2.001*dGap ); }
#endif // SM_USE_OLDTOL
                                  }
                              }
                            break;
                          } // end SmEdge switch

      case SmFace_TYPE: { SmFace * pFace = SM_CAST_PTR( SmFace, pTopo );
                          if ( pFace == NULL )
                            { return SM_ERR_INVALID_INPUT; }
                          double dFaceTol = pFace->GetTolerance();
#ifdef SM_USE_OLDTOL
                          SM_OLDTOL_LINE if ( dTolToUse > dFaceTol )
                          SM_OLDTOL_LINE   { pFace->SetTolerance( dTolToUse ); }
#endif // SM_USE_OLDTOL

                          // We have to update lower-dimensional topology.  [cbiTolProj]
                          // Edges are sufficient, as they will take care of their vertices (just above).
                          SmTArray<SmEdge*> sEdges ;
                          pFace->GetEdges(sEdges) ;
                          ULONG ii, lNumEdges = sEdges.GetSize();
                          for ( ii = 0; ii < lNumEdges; ii++ )
                            {
                              UpdateObjectTolerance( sEdges[ii], dThisGap, dThisTol );
                            }
                          break;
                        } // end SmFace switch

      default:
                        { SM_ASSERT_MSG( FALSE, _T("SMS_AddInOneBrep: Bad topo type in UpdateObjectTolerance") ); }

  } // end switch

  return SM_SUCCESS;

} // end SmTol::UpdateObjectTolerance


/*******************************************************************//**
PURPOSE: return SM_EFF_ZERO scaled by largest dimension for many
         base object types

NOTES:
***********************************************************************/
SmScaledZero SmTol::GetScaledZero    ( )                         { return( SM_EFF_ZERO); }
SmScaledZero SmTol::GetScaledZeroSq  ( )                         { return( SM_EFF_ZERO_SQ); }
SmScaledZero SmTol::GetScaledZeroSqrt( )                         { return( SM_EFF_ZERO_SQRT); }

SmScaledZero SmTol::GetScaledZero( const double       dVal)      { return( SM_EFF_ZERO * (1.0 + smos_Fabs(dVal))); }
SmScaledZero SmTol::GetScaledZero( const SmVector2d & rPnt)      { return( SM_EFF_ZERO * (1.0 + rPnt.GetMaxDimension())); }
SmScaledZero SmTol::GetScaledZero( const SmVector3d & rPnt)      { return( SM_EFF_ZERO * (1.0 + rPnt.GetMaxDimension())); }
SmScaledZero SmTol::GetScaledZero( const SmExtent1d & rIvl)      { return( SM_EFF_ZERO * (1.0 + rIvl.GetMaxDimension())); }
SmScaledZero SmTol::GetScaledZero( const SmExtent2d & rSqr)      { return( SM_EFF_ZERO * (1.0 + smos_Max( rSqr.GetMin().GetMaxDimension(),
                                                                                                          rSqr.GetMax().GetMaxDimension())));
                                                                 }
SmScaledZero SmTol::GetScaledZero( const SmExtent3d & rBox)      { return( SM_EFF_ZERO * (1.0 + smos_Max( rBox.GetMin().GetMaxDimension(),
                                                                                                          rBox.GetMax().GetMaxDimension())));
                                                                 }
SmScaledZero SmTol::GetScaledZero( const SmPseudoBox & rBox)     { double dA = rBox.GetInterval(0).GetMaxDimension() ;
                                                                   double dB = rBox.GetInterval(1).GetMaxDimension() ;
                                                                   double dC = rBox.GetInterval(2).GetMaxDimension() ;
                                                                   return( SM_EFF_ZERO * (1.0 + smos_3Max(dA, dB, dC)));
                                                                 }
SmScaledZero SmTol::GetScaledZero( const SmFace     & rFace)     { return( rFace.GetSurface() ? GetScaledZero(*rFace.GetSurface() ) : SmScaledZero(SM_EFF_ZERO) ) ; }
SmScaledZero SmTol::GetScaledZero( const SmEdge     & rEdge)     { return( rEdge.GetCurve()   ? GetScaledZero(*rEdge.GetCurve() )   : SmScaledZero(SM_EFF_ZERO) ) ; }
SmScaledZero SmTol::GetScaledZero( const SmVertex   & rVertex)   { return( GetScaledZero(rVertex.GetPoint()) ) ; }
SmScaledZero SmTol::GetScaledZero( const SmBrep     & rBrep )    { SmExtent3d sBBox ; rBrep.  CalculateBoundingBox(sBBox) ; return(GetScaledZero(sBBox)) ; }
SmScaledZero SmTol::GetScaledZero( const SmRegion   & rRegion )  { SmExtent3d sBBox ; rRegion.CalculateBoundingBox(sBBox) ; return(GetScaledZero(sBBox)) ; }
SmScaledZero SmTol::GetScaledZero( const SmShell    & rShell )   { SmExtent3d sBBox ; rShell. CalculateBoundingBox(sBBox) ; return(GetScaledZero(sBBox)) ; }

SmScaledZero SmTol::GetScaledZero( const SmCurve    & rCurve)    { SmExtent1d sIvl = rCurve.GetNaturalInterval() ;
                                                                   double     dLength = rCurve.ApproximateLength(sIvl, 3) ;
                                                                   SmPoint3d  sPnt ;
                                                                   rCurve.EvaluatePoint(sIvl.Evaluate(0.5), sPnt) ;

                                                                   return( GetScaledZero(sPnt, dLength) ) ;
                                                                 }
SmScaledZero SmTol::GetScaledZero( const SmSurface  & rSurface)  { SmPoint2d sSize ;
                                                                   SmPoint3d sPnt ;
                                                                   rSurface.ApproximateSize(sSize) ;
                                                                   rSurface.EvaluatePoint(rSurface.GetNaturalUVDomain().Evaluate(0.5,0.5), sPnt) ;
                                                                   return( GetScaledZero(sPnt, sSize) ) ;
                                                                 }
SmScaledZero SmTol::GetScaledZero( const SmVolume   & rVolume)   { SmExtent3d sParamDomain = rVolume.GetNaturalParamDomain() ;
                                                                   if(sParamDomain.IsBounded()) 
                                                                         { SmExtent3d sOutBox ;
                                                                           rVolume.EvaluateBoundingBox(sParamDomain, NULL, &sOutBox) ;
                                                                           return( GetScaledZero(sOutBox) ) ;
                                                                         } 
                                                                   else  { return 100.0 * SM_EFF_ZERO ; }
                                                                 }
// gwc: alternative method for GetScaledZero(rVol) to be tested against current version
// SmScaledZero SmTol::GetScaledZero( const SmVolume   & rVolume)   { SmPoint3d sPnt, sSize ;
//                                                                    rVolume.ApproximateUVWSize(sSize) ;
//                                                                    rVolume.EvaluatePoint(rVolume.GetNaturalUVWDomain().Evaluate(0.5,0.5,0.5), sPnt) ;
//                                                                    return( GetScaledZero(sPnt, sSize) ) ;
//                                                                  }
SmScaledZero SmTol::GetScaledZero( const SmCurveInterval & rCurveInterval)   { return( GetScaledZero(rCurveInterval.GetInterval()) ) ; }
SmScaledZero SmTol::GetScaledZero( const SmEvalNFunctionsObject & rEvalNObj) { return( rEvalNObj.GetScaledZero() ) ; }
SmScaledZero SmTol::GetScaledZero( const SmObject & rObj)
{   // classify type - for upcoming switch statement
  if     (rObj.IsKindOf(SmPoint3d_TYPE)) { return( GetScaledZero( (SmPoint3d &)rObj) ) ; }
  else if(rObj.IsKindOf(SmCurve_TYPE))   { return( GetScaledZero( (SmCurve   &)rObj) ) ; }
  else if(rObj.IsKindOf(SmSurface_TYPE)) { return( GetScaledZero( (SmSurface &)rObj) ) ; }
  else if(rObj.IsKindOf(SmCurveClassification_TYPE)) { return( GetScaledZero( *((SmCurveClassification &)rObj).GetCurve() )) ;  }
  else if(rObj.IsKindOf(SmCurveInterval_TYPE))       { return( GetScaledZero( *((SmCurveInterval &)rObj).GetCurveClassification()->GetCurve() )) ;  }
  else if(rObj.IsKindOf(SmPointClassification_TYPE)) { SmPoint3d sPt ;
                                                        SmStatus sStatus = ((SmPointClassification &)rObj).FindObjPoint3d(sPt) ;
                                                        return( (sStatus != SM_ERR) ? (double)GetScaledZero(sPt) : SM_EFF_ZERO ) ;
                                                      }

  else if(rObj.IsKindOf(SmVertex_TYPE )) { return( GetScaledZero(  ((SmVertex &)rObj).GetPoint() )) ; }
  else if(rObj.IsKindOf(SmEdge_TYPE))    { return( GetScaledZero( *((SmEdge   &)rObj).GetCurve() )) ; }
  else if(rObj.IsKindOf(SmFace_TYPE))    { return( GetScaledZero( *((SmFace   &)rObj).GetSurface() )) ; }
  else if(rObj.IsKindOf(SmBrep_TYPE))    { return( GetScaledZero(  ((SmBrep   &)rObj) )) ; }
  else if(rObj.IsKindOf(SmRegion_TYPE))  { return( GetScaledZero(  ((SmRegion &)rObj) )) ; }
  else if(rObj.IsKindOf(SmShell_TYPE))   { return( GetScaledZero(  ((SmShell  &)rObj) )) ; }

  else return(SM_EFF_ZERO) ;

} // end SmTol::GetScaledZero( const SmObject        * pObj)

/*******************************************************************//**
PURPOSE: Temporary local functions to exercise SmTol related classes

NOTES:
***********************************************************************/
#ifdef SM_DEBUG_CODE // for sm_TemplateTrial block
SmTol3d      sm_CastTry1(double dVal)       { return dVal ; }
double       sm_CastTry2(SmTol3d tVal)      { return tVal ; }
double       sm_CastTry3(double dVal)       { return dVal ; }
SmTol3d      sm_CastTry4(SmTol3d tVal)      { return tVal ; }
SmZoneTol3d  sm_TypeTry1(SmZoneTol3d tVal)  { return tVal ; }
SmXSectTol3d sm_TypeTry2(SmXSectTol3d tVal) { return tVal ; }

class SmHasTol
{ public: mutable SmZoneTol3d m_sZoneTol3d ;
} ;

/*******************************************************************//**
PURPOSE: Simple exercise of SmTol class methods

NOTES:
***********************************************************************/
SmBoolean SmTol::sm_TemplateTrial()
{
   SmBoolean bRtn = TRUE ;

   // verify interface functions
   SmZoneTol3d sTgtZoneTol3d = 1.0e-5 ;
   double      dModelSize = 50 ;
   SmZoneTol3d sEstZoneTol3d = SizeEstimateToZoneTol3d(dModelSize) ;
   // double      dEstModelSize = ZoneTol3dToSizeEstimate(sEstZoneTol3d) ;
   bRtn &=  sEstZoneTol3d == sTgtZoneTol3d ;
   // bRtn &=  sEstZoneTol3d == SizeEstimateToZoneTol3d(dEstModelSize) ;

   // SizeEstimateToZoneTol3d
   { double d1 = fabs(dModelSize) ;
     double d2 = log10(d1) ;
     double d3 = (d2)/2.0 ;
     double d4 = floor(d3) ;
     double d5 = pow(10,d4) ;
     double d6 = d5*1.0e-5 ;
     //double d7 = (pow(10,floor(log10(fabs(dModelSize))/2.0))*1.0e-5) ;
     bRtn = d6 == sEstZoneTol3d ;
   }
   // ZoneTol3dToSizeEstimate               
   { double d1 = sEstZoneTol3d / 1.0e-5 ;   
     double d2 = log10(d1) ;                
     double d3 = floor(d2) ;                
     double d4 = d3*2 ;                     
     double d5 = pow(10,d4) ;               
     double d6 = d5*50 ;              SM_ASSERT(d6 == d5*50) ;
     // double d7 =  (pow(10,floor(log10(sEstZoneTol3d / 1.0e-5))*2)*50) ;
     
     // bRtn = d6 == dEstModelSize ;
  }  

   sTgtZoneTol3d = 1.0e-3 ;
   dModelSize = 50000 ;
   sEstZoneTol3d = SizeEstimateToZoneTol3d(dModelSize) ;
   // dEstModelSize = ZoneTol3dToSizeEstimate(sEstZoneTol3d) ;
   bRtn &=  sEstZoneTol3d == sTgtZoneTol3d ;
   // bRtn &=  sEstZoneTol3d == SizeEstimateToZoneTol3d(dEstModelSize) ;

   sTgtZoneTol3d = 1.0e-7 ;
   dModelSize = 0.005 ;
   sEstZoneTol3d = SizeEstimateToZoneTol3d(dModelSize) ;
   // dEstModelSize = ZoneTol3dToSizeEstimate(sEstZoneTol3d) ;
   bRtn &=  sEstZoneTol3d == sTgtZoneTol3d ;
   // bRtn &=  sEstZoneTol3d == SizeEstimateToZoneTol3d(dEstModelSize) ;

   // verify SmTol3d class is interchangeable with double values for 'BeforeNewTol' releases
   SmTol3d  tVal0, tVal1(.5), tVal2(.75) ;
   const SmTol3d cVal0(.25) ;
   double    dVal = tVal1 ;
   //
   SmBoolean bTest ;
   tVal0 = .25 ;
   tVal1 = dVal ;
   tVal2 = tVal0 + tVal1 ;  tVal2 = 0.250 + tVal1 ;  tVal2 = tVal0 + 0.5 ; tVal2 = 0.250 + 0.5 ;
   tVal2 = tVal0 - tVal1 ;  tVal2 = 0.250 - tVal1 ;  tVal2 = tVal0 - 0.5 ; tVal2 = 0.250 - 0.5 ;
   tVal2 = tVal0 * tVal1 ;  tVal2 = 0.250 * tVal1 ;  tVal2 = tVal0 * 0.5 ; tVal2 = 0.250 * 0.5 ;
   tVal2 = tVal0 / tVal1 ;  tVal2 = 0.250 / tVal1 ;  tVal2 = tVal0 / 0.5 ; tVal2 = 0.250 / 0.5 ;
   bTest = tVal0 < tVal1 ;  bTest = 0.250 < tVal1 ;  bTest = tVal0 < 0.5 ; bTest = 0.250 < 0.5 ;
   bTest = tVal0 > tVal1 ;  bTest = 0.250 > tVal1 ;  bTest = tVal0 > 0.5 ; bTest = 0.250 > 0.5 ;
   bTest = tVal0 < dVal ;
   bTest = tVal0 > dVal ;
   bTest = dVal < tVal0 ;
   bTest = dVal > tVal0 ;
   tVal0 *= 7.0 ;
   tVal0 /= 7.0 ;
   tVal0 += 7.0 ;
   tVal0 -= 7.0 ;
   tVal0 = 7.0 ;
   bTest = tVal0 == 7.0 ; bTest = tVal0 == 7 ;
   bTest = 7.0 == tVal0 ; bTest = 7 == tVal0 ;


   dVal = (SmTol3d)cVal0 ;
   dVal = cVal0 ;
   dVal = (double)cVal0 > (double)dVal ? (double)cVal0 : (double)dVal ;

   tVal0 = sm_CastTry1(.75) ;  tVal0 = sm_CastTry1(tVal1) ;
   dVal  = sm_CastTry1(.75) ;  dVal  = sm_CastTry1(tVal1) ;
   tVal0 = sm_CastTry2(.75) ;  tVal0 = sm_CastTry2(tVal1) ;
   dVal  = sm_CastTry2(.75) ;  dVal  = sm_CastTry2(tVal1) ;
   tVal0 = sm_CastTry3(.75) ;  tVal0 = sm_CastTry3(tVal1) ;
   dVal  = sm_CastTry3(.75) ;  dVal  = sm_CastTry3(tVal1) ;
   tVal0 = sm_CastTry4(.75) ;  tVal0 = sm_CastTry4(tVal1) ;
   dVal  = sm_CastTry4(.75) ;  dVal  = sm_CastTry4(tVal1) ;

   bTest = tVal0 == tVal1 ;  // tVal0 = tVal1.Sqrt() ;
   bTest = 0.250 == tVal1 ;  // tVal0 = tVal1.Sq() ;
   bTest = 1     == tVal1 ;  // tVal0 = smos_Sqrt(tVal1) ;
   bTest = tVal0 == 0.250 ;
   bTest = tVal0 == 1     ;
   bTest = 0.250 == 1 ;

   // try intermixing types - with     SM_USE_NEWTOL_STRONG_TYPES desire following not to compile
   //                          without SM_USE_NEWTOL_STRONG_TYPES desire following to compile
   SmZoneTol3d    sZoneTol3d ;
   SmXSectTol3d   sXSectTol3d ;
   SmApproxTol3d  sApproxTol3d ;
   SmHasTol       sHasTol3d ;
   const SmZoneTol3d   cZoneTol3d ;
   const SmXSectTol3d  cXSectTol3d ;
   const SmApproxTol3d cApproxTol3d ;
   // const SmHasTol      cHasTol3d ;
   // double cVal = 0.5 ;

   // SmZoneTol3d  tZone  = sm_TypeTry1(sXSectTol3d) ; sZoneTol3d  = sXSectTol3d ;
   // SmXSectTol3d tXSect = sm_TypeTry2(sZoneTol3d) ;  sXSectTol3d = sZoneTol3d ;
   // SmZoneTol3d  tZone  = sm_TypeTry1(tVal0) ;
   // SmXSectTol3d tXSect = sm_TypeTry2(tVal0) ;
   //SmZoneTol3d  tZone  = sm_TypeTry1(.25) ;
   //SmXSectTol3d tXSect = sm_TypeTry2(.25) ;
   dVal = .5 ;
   sZoneTol3d   = dVal ;
   sXSectTol3d  = dVal ;
   sApproxTol3d = dVal ;
   sHasTol3d.m_sZoneTol3d = 0.25 ;
   sHasTol3d.m_sZoneTol3d = dVal ;
   sHasTol3d.m_sZoneTol3d = sZoneTol3d ;

   // OS X compiler error in this block.
   // Is this block even necessary?
   //  {
   //    SmTemporaryChangeValue<SmZoneTol3d> sStack1(sHasTol3d.m_sZoneTol3d, 7) ;
   //    SmTemporaryChangeValue<double>      sStack4((double)cVal, 7) ;
   //    sHasTol3d.m_sZoneTol3d = .25 ;
   //  }

   dVal = sZoneTol3d  ;
   dVal = sXSectTol3d ;
   dVal = sApproxTol3d ;

   dVal *= sZoneTol3d   ;   sZoneTol3d   = sZoneTol3d   * 7.0 ; sZoneTol3d   = 7.0 * sZoneTol3d   ;
   dVal /= sXSectTol3d  ;   sZoneTol3d   = sZoneTol3d   / 7.0 ; sZoneTol3d   = 7.0 / sZoneTol3d   ;
   dVal += sApproxTol3d ;   sZoneTol3d   = sZoneTol3d   + 7.0 ; sZoneTol3d   = 7.0 + sZoneTol3d   ;
   dVal -= 7.0 ;            sZoneTol3d   = sZoneTol3d   - 7.0 ; sZoneTol3d   = 7.0 - sZoneTol3d   ;

   sZoneTol3d *= 7.0 ;   sZoneTol3d   = sZoneTol3d   * 7.0 ; sZoneTol3d   = 7.0 * sZoneTol3d   ;
   sZoneTol3d /= 7.0 ;   sZoneTol3d   = sZoneTol3d   / 7.0 ; sZoneTol3d   = 7.0 / sZoneTol3d   ;
   sZoneTol3d += 7.0 ;   sZoneTol3d   = sZoneTol3d   + 7.0 ; sZoneTol3d   = 7.0 + sZoneTol3d   ;
   sZoneTol3d -= 7.0 ;   sZoneTol3d   = sZoneTol3d   - 7.0 ; sZoneTol3d   = 7.0 - sZoneTol3d   ;

   sXSectTol3d *= 7.0 ;  sXSectTol3d  = sXSectTol3d  * 7.0 ; sXSectTol3d  = 7.0 * sXSectTol3d  ;
   sXSectTol3d /= 7.0 ;  sXSectTol3d  = sXSectTol3d  / 7.0 ; sXSectTol3d  = 7.0 / sXSectTol3d  ;
   sXSectTol3d += 7.0 ;  sXSectTol3d  = sXSectTol3d  + 7.0 ; sXSectTol3d  = 7.0 + sXSectTol3d  ;
   sXSectTol3d -= 7.0 ;  sXSectTol3d  = sXSectTol3d  - 7.0 ; sXSectTol3d  = 7.0 - sXSectTol3d  ;

   sApproxTol3d *= 7.0 ; sApproxTol3d = sApproxTol3d * 7.0 ; sApproxTol3d = 7.0 * sApproxTol3d ;
   sApproxTol3d /= 7.0 ; sApproxTol3d = sApproxTol3d / 7.0 ; sApproxTol3d = 7.0 / sApproxTol3d ;
   sApproxTol3d += 7.0 ; sApproxTol3d = sApproxTol3d + 7.0 ; sApproxTol3d = 7.0 + sApproxTol3d ;
   sApproxTol3d -= 7.0 ; sApproxTol3d = sApproxTol3d - 7.0 ; sApproxTol3d = 7.0 - sApproxTol3d ;

   bTest = sZoneTol3d   < dVal ;
   bTest = sXSectTol3d  < dVal ;
   bTest = sApproxTol3d < dVal ;

   bTest = sZoneTol3d   > dVal ;
   bTest = sXSectTol3d  > dVal ;
   bTest = sApproxTol3d > dVal ;

   bTest = dVal < sZoneTol3d   ;
   bTest = dVal < sXSectTol3d  ;
   bTest = dVal < sApproxTol3d ;

   bTest = dVal > sZoneTol3d   ;
   bTest = dVal > sXSectTol3d  ;
   bTest = dVal > sApproxTol3d ;


   // locals
   // double      sVec1d = 88 ;
   SmVector2d  sVec2d(1,-80) ;
   SmVector3d  sVec3d(16,100,1000) ;
   SmExtent1d  sExt1d(-20,1) ;
   SmExtent2d  sExt2d(0,-40,100,2000) ;
   SmExtent3d  sExt3d(0,-30,-600,2000,100,10000) ;
   SmPseudoBox sBox3d ; sBox3d.SetIntervals(sExt1d, sExt1d, sExt1d) ;

   SmScaledZero dScaledZero1( .001) ;
   SmScaledZero dScaledZero2( .002) ;
   SmScaledZero dScaledZero3(-.003) ;
   dScaledZero1 = .004 ;                 bRtn &= dScaledZero1 == .004 ;
   dScaledZero1 = dScaledZero1 + .001 ;  bRtn &= dScaledZero1 == .005 ;
   dScaledZero1 = .001 + dScaledZero1 ;  bRtn &= dScaledZero1 == .006 ;
   dScaledZero1 = dScaledZero1 - .001 ;  bRtn &= dScaledZero1 == .005 ;
   dScaledZero1 = .001 - dScaledZero1 ;  bRtn &= dScaledZero1 == .004 ;
   SmBoolean b1 = dScaledZero1 > .001 ;           bRtn &=  b1  ;
   SmBoolean b2 = .001 > dScaledZero1 ;           bRtn &= !b2  ;
   SmBoolean b3 = dScaledZero1 < .001 ;           bRtn &= !b3  ;
   SmBoolean b4 = .001 < dScaledZero1 ;           bRtn &=  b4  ;
   SmBoolean b11 = dScaledZero1 > -.001 ;         bRtn &=  b11 ;
   SmBoolean b12 = -.001 > dScaledZero1 ;         bRtn &= !b12 ;
   SmBoolean b13 = dScaledZero1 < -.001 ;         bRtn &= !b13 ;
   SmBoolean b14 = -.001 < dScaledZero1 ;         bRtn &=  b14 ;
   SmBoolean b5 = dScaledZero1 > dScaledZero2 ;   bRtn &=  b5  ;
   SmBoolean b6 = dScaledZero2 > dScaledZero1 ;   bRtn &= !b6  ;
   SmBoolean b7 = dScaledZero1 < dScaledZero2 ;   bRtn &= !b7  ;
   SmBoolean b8 = dScaledZero2 < dScaledZero1 ;   bRtn &=  b8  ;

   SmBoolean ba = InTol(.001, dScaledZero1) ;   bRtn &=  ba ;  bRtn &= InTol(.001, dScaledZero1)  == !OutOfTol(.001, dScaledZero1) ;
   SmBoolean bb = InTol(dScaledZero1, .001) ;   bRtn &=  bb ;  bRtn &= InTol(dScaledZero1, .001)  == !OutOfTol(dScaledZero1, .001) ;
   SmBoolean bc = InTol(-.001, dScaledZero1) ;  bRtn &=  bc ;  bRtn &= InTol(-.001, dScaledZero1) == !OutOfTol(-.001, dScaledZero1) ;
   SmBoolean bd = InTol(dScaledZero1, -.001) ;  bRtn &=  bd ;  bRtn &= InTol(dScaledZero1, -.001) == !OutOfTol(dScaledZero1, -.001) ;

   SmBoolean be = InTol(.008, dScaledZero1) ;   bRtn &= !be ;  bRtn &= InTol(.008, dScaledZero1)  == !OutOfTol(.008, dScaledZero1) ;
   SmBoolean bf = InTol(dScaledZero1, .008) ;   bRtn &= !bf ;  bRtn &= InTol(dScaledZero1, .008)  == !OutOfTol(dScaledZero1, .008) ;
   SmBoolean bg = InTol(-.008, dScaledZero1) ;  bRtn &= !bg ;  bRtn &= InTol(-.008, dScaledZero1) == !OutOfTol(-.008, dScaledZero1) ;
   SmBoolean bh = InTol(dScaledZero1, -.008) ;  bRtn &= !bh ;  bRtn &= InTol(dScaledZero1, -.008) == !OutOfTol(dScaledZero1, -.008) ;

   SmBoolean b01 = SM_IS_ZERO(SM_EFF_ZERO/2) ;      bRtn &=  b01 ;
   SmBoolean b02 = SM_IS_ZERO(SM_EFF_ZERO*2) ;      bRtn &= !b02 ;
   SmBoolean b03 = SM_IS_ZERO_TO_TOL(.005, .008) ;  bRtn &=  b03 ;
   SmBoolean b04 = SM_IS_ZERO_TO_TOL(.005, .003) ;  bRtn &= !b04 ;
   SmBoolean b05 = SM_IS_ZERO(-SM_EFF_ZERO/2) ;     bRtn &=  b05 ;
   SmBoolean b06 = SM_IS_ZERO(-SM_EFF_ZERO*2) ;     bRtn &= !b06 ;
   SmBoolean b07 = SM_IS_ZERO_TO_TOL(-.005, .008) ; bRtn &=  b07 ;
   SmBoolean b08 = SM_IS_ZERO_TO_TOL(-.005, .003) ; bRtn &= !b08 ;

   SmBoolean b001 = SM_ARE_SAME(.001, .0010000000001) ;     bRtn &=  b001 ;
   SmBoolean b002 = SM_ARE_SAME(.001, .0010001) ;           bRtn &= !b002 ;
   SmBoolean b003 = SM_ARE_SAME(-.001, -.0010000000001) ;   bRtn &=  b003 ;
   SmBoolean b004 = SM_ARE_SAME(-.001, -.0010001) ;         bRtn &= !b004 ;
   SmBoolean b005 = SM_ARE_SAME_TO_TOL(.001, .0010000000001, SM_EFF_ZERO*10) ;    bRtn &=  b005 ;
   SmBoolean b006 = SM_ARE_SAME_TO_TOL(.001, .0010001, SM_EFF_ZERO*10) ;          bRtn &= !b006 ;
   SmBoolean b007 = SM_ARE_SAME_TO_TOL(-.001, -.0010000000001, -SM_EFF_ZERO*10) ; bRtn &=  b007 ;
   SmBoolean b008 = SM_ARE_SAME_TO_TOL(-.001, -.0010001, -SM_EFF_ZERO*10) ;       bRtn &= !b008 ;

   SmBoolean b011 = SM_IS_BETWEEN(1,0,2) ;                bRtn &=  b011 ;
   SmBoolean b012 = SM_IS_BETWEEN(0+SM_EFF_ZERO/2,0,2) ;  bRtn &= !b012 ;
   SmBoolean b013 = SM_IS_BETWEEN(0-SM_EFF_ZERO/2,0,2) ;  bRtn &= !b013 ;
   SmBoolean b014 = SM_IS_BETWEEN(2+SM_EFF_ZERO/2,0,2) ;  bRtn &= !b014 ;
   SmBoolean b015 = SM_IS_BETWEEN(2-SM_EFF_ZERO/2,0,2) ;  bRtn &= !b015 ;
   SmBoolean b016 = SM_IS_BETWEEN(0+SM_EFF_ZERO*5,0,2) ;  bRtn &=  b016 ;
   SmBoolean b017 = SM_IS_BETWEEN(0-SM_EFF_ZERO*5,0,2) ;  bRtn &= !b017 ;
   SmBoolean b018 = SM_IS_BETWEEN(2+SM_EFF_ZERO*5,0,2) ;  bRtn &= !b018 ;
   SmBoolean b019 = SM_IS_BETWEEN(2-SM_EFF_ZERO*5,0,2) ;  bRtn &=  b019 ;

   SmBoolean b111 = SM_IS_BETWEEN(1,0,-2) ;                 bRtn &= !b111 ;
   SmBoolean b112 = SM_IS_BETWEEN( 0+SM_EFF_ZERO/2,0,-2) ;  bRtn &= !b112 ;
   SmBoolean b113 = SM_IS_BETWEEN( 0-SM_EFF_ZERO/2,0,-2) ;  bRtn &= !b113 ;
   SmBoolean b114 = SM_IS_BETWEEN(-2+SM_EFF_ZERO/2,0,-2) ;  bRtn &= !b114 ;
   SmBoolean b115 = SM_IS_BETWEEN(-2-SM_EFF_ZERO/2,0,-2) ;  bRtn &= !b115 ;
   SmBoolean b116 = SM_IS_BETWEEN( 0+SM_EFF_ZERO*5,0,-2) ;  bRtn &= !b116 ;
   SmBoolean b117 = SM_IS_BETWEEN( 0-SM_EFF_ZERO*5,0,-2) ;  bRtn &=  b117 ;
   SmBoolean b118 = SM_IS_BETWEEN(-2+SM_EFF_ZERO*5,0,-2) ;  bRtn &=  b118 ;
   SmBoolean b119 = SM_IS_BETWEEN(-2-SM_EFF_ZERO*5,0,-2) ;  bRtn &= !b119 ;

   SmBoolean b121 = SM_IS_BETWEEN_TO_TOL(1,0,-2, SM_EFF_ZERO) ;                bRtn &= !b121 ;
   SmBoolean b122 = SM_IS_BETWEEN_TO_TOL(0+SM_EFF_ZERO/2,0,-2, SM_EFF_ZERO) ;  bRtn &= !b122 ;
   SmBoolean b123 = SM_IS_BETWEEN_TO_TOL(0-SM_EFF_ZERO/2,0,-2, SM_EFF_ZERO) ;  bRtn &= !b123 ;
   SmBoolean b124 = SM_IS_BETWEEN_TO_TOL(-2+SM_EFF_ZERO/2,0,-2, SM_EFF_ZERO) ; bRtn &= !b124 ;
   SmBoolean b125 = SM_IS_BETWEEN_TO_TOL(-2-SM_EFF_ZERO/2,0,-2, SM_EFF_ZERO) ; bRtn &= !b125 ;
   SmBoolean b126 = SM_IS_BETWEEN_TO_TOL(0+SM_EFF_ZERO*2,0,-2, SM_EFF_ZERO) ;  bRtn &= !b126 ;
   SmBoolean b127 = SM_IS_BETWEEN_TO_TOL(0-SM_EFF_ZERO*2,0,-2, SM_EFF_ZERO) ;  bRtn &=  b127 ;
   SmBoolean b128 = SM_IS_BETWEEN_TO_TOL(-2+SM_EFF_ZERO*2,0,-2, SM_EFF_ZERO) ; bRtn &=  b128 ;
   SmBoolean b129 = SM_IS_BETWEEN_TO_TOL(-2-SM_EFF_ZERO*2,0,-2, SM_EFF_ZERO) ; bRtn &= !b129 ;

   SmBoolean b041 = SM_IS_CONTAINED(1,0,2) ;                bRtn &=  b041 ;
   SmBoolean b042 = SM_IS_CONTAINED(0+SM_EFF_ZERO/2,0,2) ;  bRtn &=  b042 ;
   SmBoolean b043 = SM_IS_CONTAINED(0-SM_EFF_ZERO/2,0,2) ;  bRtn &=  b043 ;
   SmBoolean b044 = SM_IS_CONTAINED(2+SM_EFF_ZERO/2,0,2) ;  bRtn &=  b044 ;
   SmBoolean b045 = SM_IS_CONTAINED(2-SM_EFF_ZERO/2,0,2) ;  bRtn &=  b045 ;
   SmBoolean b046 = SM_IS_CONTAINED(0+SM_EFF_ZERO*5,0,2) ;  bRtn &=  b046 ;
   SmBoolean b047 = SM_IS_CONTAINED(0-SM_EFF_ZERO*5,0,2) ;  bRtn &= !b047 ;
   SmBoolean b048 = SM_IS_CONTAINED(2+SM_EFF_ZERO*5,0,2) ;  bRtn &= !b048 ;
   SmBoolean b049 = SM_IS_CONTAINED(2-SM_EFF_ZERO*5,0,2) ;  bRtn &=  b049 ;

   SmBoolean b141 = SM_IS_CONTAINED_TO_TOL(1,0,-2, SM_EFF_ZERO) ;                bRtn &= !b141 ;
   SmBoolean b142 = SM_IS_CONTAINED_TO_TOL( 0+SM_EFF_ZERO/2,0,-2, SM_EFF_ZERO) ; bRtn &=  b142 ;
   SmBoolean b143 = SM_IS_CONTAINED_TO_TOL( 0-SM_EFF_ZERO/2,0,-2, SM_EFF_ZERO) ; bRtn &=  b143 ;
   SmBoolean b144 = SM_IS_CONTAINED_TO_TOL(-2+SM_EFF_ZERO/2,0,-2, SM_EFF_ZERO) ; bRtn &=  b144 ;
   SmBoolean b145 = SM_IS_CONTAINED_TO_TOL(-2-SM_EFF_ZERO/2,0,-2, SM_EFF_ZERO) ; bRtn &=  b145 ;
   SmBoolean b146 = SM_IS_CONTAINED_TO_TOL( 0+SM_EFF_ZERO*2,0,-2, SM_EFF_ZERO) ; bRtn &= !b146 ;
   SmBoolean b147 = SM_IS_CONTAINED_TO_TOL( 0-SM_EFF_ZERO*2,0,-2, SM_EFF_ZERO) ; bRtn &=  b147 ;
   SmBoolean b148 = SM_IS_CONTAINED_TO_TOL(-2+SM_EFF_ZERO*2,0,-2, SM_EFF_ZERO) ; bRtn &=  b148 ;
   SmBoolean b149 = SM_IS_CONTAINED_TO_TOL(-2-SM_EFF_ZERO*2,0,-2, SM_EFF_ZERO) ; bRtn &= !b149 ;

   // exercise binary GetScaledZero
   // These create Linux compile warnings because varaibles are left unused
   SmVector3d sVec1d(1,0,0) ;

   SmScaledZero d01 = SmTol::GetScaledZero(sVec1d, sVec1d) ;
   SmScaledZero d02 = SmTol::GetScaledZero(sVec2d, sVec2d) ;
   SmScaledZero d03 = SmTol::GetScaledZero(sVec3d, sVec3d) ;
   SmScaledZero d04 = SmTol::GetScaledZero(sExt1d, sExt1d) ;
   SmScaledZero d05 = SmTol::GetScaledZero(sExt2d, sExt2d) ;
   SmScaledZero d06 = SmTol::GetScaledZero(sExt3d, sExt3d) ;
   SmScaledZero d07 = SmTol::GetScaledZero(sBox3d, sBox3d) ;

   SmScaledZero d11 = SmTol::GetScaledZero(sVec1d, sBox3d) ;
   SmScaledZero d12 = SmTol::GetScaledZero(sVec2d, sVec1d) ;
   SmScaledZero d13 = SmTol::GetScaledZero(sVec3d, sVec2d) ;
   SmScaledZero d14 = SmTol::GetScaledZero(sExt1d, sVec3d) ;
   SmScaledZero d15 = SmTol::GetScaledZero(sExt2d, sExt1d) ;
   SmScaledZero d16 = SmTol::GetScaledZero(sExt3d, sExt2d) ;
   SmScaledZero d17 = SmTol::GetScaledZero(sBox3d, sExt3d) ;

   SmScaledZero d21 = SmTol::GetScaledZero(sVec1d, sExt3d) ;
   SmScaledZero d22 = SmTol::GetScaledZero(sVec2d, sBox3d) ;
   SmScaledZero d23 = SmTol::GetScaledZero(sVec3d, sVec1d) ;
   SmScaledZero d24 = SmTol::GetScaledZero(sExt1d, sVec2d) ;
   SmScaledZero d25 = SmTol::GetScaledZero(sExt2d, sVec3d) ;
   SmScaledZero d26 = SmTol::GetScaledZero(sExt3d, sExt1d) ;
   SmScaledZero d27 = SmTol::GetScaledZero(sBox3d, sExt2d) ;

   SmScaledZero d31 = SmTol::GetScaledZero(sVec1d, sExt2d) ;
   SmScaledZero d32 = SmTol::GetScaledZero(sVec2d, sExt3d) ;
   SmScaledZero d33 = SmTol::GetScaledZero(sVec3d, sBox3d) ;
   SmScaledZero d34 = SmTol::GetScaledZero(sExt1d, sVec1d) ;
   SmScaledZero d35 = SmTol::GetScaledZero(sExt2d, sVec2d) ;
   SmScaledZero d36 = SmTol::GetScaledZero(sExt3d, sVec3d) ;
   SmScaledZero d37 = SmTol::GetScaledZero(sBox3d, sExt1d) ;

   SmScaledZero d41 = SmTol::GetScaledZero(sVec1d, sExt1d) ;
   SmScaledZero d42 = SmTol::GetScaledZero(sVec2d, sExt2d) ;
   SmScaledZero d43 = SmTol::GetScaledZero(sVec3d, sExt3d) ;
   SmScaledZero d44 = SmTol::GetScaledZero(sExt1d, sBox3d) ;
   SmScaledZero d45 = SmTol::GetScaledZero(sExt2d, sVec1d) ;
   SmScaledZero d46 = SmTol::GetScaledZero(sExt3d, sVec2d) ;
   SmScaledZero d47 = SmTol::GetScaledZero(sBox3d, sVec3d) ;

   SmScaledZero d51 = SmTol::GetScaledZero(sVec1d, sVec3d) ;
   SmScaledZero d52 = SmTol::GetScaledZero(sVec2d, sExt1d) ;
   SmScaledZero d53 = SmTol::GetScaledZero(sVec3d, sExt2d) ;
   SmScaledZero d54 = SmTol::GetScaledZero(sExt1d, sExt3d) ;
   SmScaledZero d55 = SmTol::GetScaledZero(sExt2d, sBox3d) ;
   SmScaledZero d56 = SmTol::GetScaledZero(sExt3d, sVec1d) ;
   SmScaledZero d57 = SmTol::GetScaledZero(sBox3d, sVec2d) ;

   SmScaledZero d61 = SmTol::GetScaledZero(sVec1d, sVec2d) ;
   SmScaledZero d62 = SmTol::GetScaledZero(sVec2d, sVec3d) ;
   SmScaledZero d63 = SmTol::GetScaledZero(sVec3d, sExt1d) ;
   SmScaledZero d64 = SmTol::GetScaledZero(sExt1d, sExt2d) ;
   SmScaledZero d65 = SmTol::GetScaledZero(sExt2d, sExt3d) ;
   SmScaledZero d66 = SmTol::GetScaledZero(sExt3d, sBox3d) ;
   SmScaledZero d67 = SmTol::GetScaledZero(sBox3d, sVec1d) ;

   // exercise tertiarty GetScaledZero
   SmScaledZero d001 = SmTol::GetScaledZero(sVec1d, sVec1d, sVec1d) ;
   SmScaledZero d002 = SmTol::GetScaledZero(sVec2d, sVec2d, sVec2d) ;
   SmScaledZero d003 = SmTol::GetScaledZero(sVec3d, sVec3d, sVec3d) ;
   SmScaledZero d004 = SmTol::GetScaledZero(sExt1d, sExt1d, sExt1d) ;
   SmScaledZero d005 = SmTol::GetScaledZero(sExt2d, sExt2d, sExt2d) ;
   SmScaledZero d006 = SmTol::GetScaledZero(sExt3d, sExt3d, sExt3d) ;
   SmScaledZero d007 = SmTol::GetScaledZero(sBox3d, sBox3d, sBox3d) ;

   SmScaledZero d111 = SmTol::GetScaledZero(sVec1d, sBox3d, sExt3d) ;
   SmScaledZero d112 = SmTol::GetScaledZero(sVec2d, sVec1d, sBox3d) ;
   SmScaledZero d113 = SmTol::GetScaledZero(sVec3d, sVec2d, sVec1d) ;
   SmScaledZero d114 = SmTol::GetScaledZero(sExt1d, sVec3d, sVec2d) ;
   SmScaledZero d115 = SmTol::GetScaledZero(sExt2d, sExt1d, sVec3d) ;
   SmScaledZero d116 = SmTol::GetScaledZero(sExt3d, sExt2d, sExt1d) ;
   SmScaledZero d117 = SmTol::GetScaledZero(sBox3d, sExt3d, sExt2d) ;

   // exercise tertiarty GetScaledZero
   SmScaledZero d0001 = SmTol::GetScaledZero(sVec1d, sVec1d, sVec1d, sVec1d) ;
   SmScaledZero d0002 = SmTol::GetScaledZero(sVec2d, sVec2d, sVec2d, sVec2d) ;
   SmScaledZero d0003 = SmTol::GetScaledZero(sVec3d, sVec3d, sVec3d, sVec3d) ;
   SmScaledZero d0004 = SmTol::GetScaledZero(sExt1d, sExt1d, sExt1d, sExt1d) ;
   SmScaledZero d0005 = SmTol::GetScaledZero(sExt2d, sExt2d, sExt2d, sExt2d) ;
   SmScaledZero d0006 = SmTol::GetScaledZero(sExt3d, sExt3d, sExt3d, sExt3d) ;
   SmScaledZero d0007 = SmTol::GetScaledZero(sBox3d, sBox3d, sBox3d, sBox3d) ;

   SmScaledZero d1111 = SmTol::GetScaledZero(sVec1d, sBox3d, sExt3d, sExt2d) ;
   SmScaledZero d1112 = SmTol::GetScaledZero(sVec2d, sVec1d, sBox3d, sExt3d) ;
   SmScaledZero d1113 = SmTol::GetScaledZero(sVec3d, sVec2d, sVec1d, sBox3d) ;
   SmScaledZero d1114 = SmTol::GetScaledZero(sExt1d, sVec3d, sVec2d, sVec1d) ;
   SmScaledZero d1115 = SmTol::GetScaledZero(sExt2d, sExt1d, sVec3d, sVec2d) ;
   SmScaledZero d1116 = SmTol::GetScaledZero(sExt3d, sExt2d, sExt1d, sVec3d) ;
   SmScaledZero d1117 = SmTol::GetScaledZero(sBox3d, sExt3d, sExt2d, sExt1d) ;

   // to prevent Linux unused variable compile warnings use all ScaledZero variables ;
   d01 = d02 = d03 = d04 = d05 = d06 = d07 = d11 = d12 = d13 = d14 = d15 = d16 = d17 ;
   d21 = d22 = d23 = d24 = d25 = d26 = d27 = d31 = d32 = d33 = d34 = d35 = d36 = d37 ;
   d41 = d42 = d43 = d44 = d45 = d46 = d47 = d51 = d52 = d53 = d54 = d55 = d56 = d57 ;
   d61 = d62 = d63 = d64 = d65 = d66 = d67 ;

   d001 = d002 = d003 = d004 = d005 = d006 = d007 ;
   d111 = d112 = d113 = d114 = d115 = d116 = d117 ;

   d0001 = d0002 = d0003 = d0004 = d0005 = d0006 = d0007 ;
   d1111 = d1112 = d1113 = d1114 = d1115 = d1116 = d1117 ;

   bRtn &= bTest;

   // all done
   return(bRtn) ;

} // end sm_TemplateTrial
#endif // SM_DEBUG_CODE - for sm_TemplateTrial block
