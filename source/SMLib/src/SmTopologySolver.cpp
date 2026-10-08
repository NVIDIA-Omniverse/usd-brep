// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************//**
* FILE NAME --- SmTopologySolver.cpp
* PURPOSE:
**********************************************************************/

#include "StdAfx.h"

#include <SmTopologySolver.h>
#include <SmTree.h>
#include <SmCurve.h>
#include <SmSurface.h>
#include <SmBSplineSurface.h>
#include <SmSolutionArray.h>
#include <SmCacheMgr.h>
#include <SmBrepCache.h>
#include <SmSurfaceCache.h>
#include <SmShape.h>
#include <SmGeomUtility.h>
#include <SmGraphicsExtern.h>
#include <SmAssertArray.h>

/****************************************************************
PURPOSE: This is an interface to get cached silhouettes 
            from a face either by retrieving them or constructing
            them.

NOTES:

*****************************************************************/ 
static SmStatus sm_GetFaceSilhouettes
  (const SmFace       * pFace,            // in : target face
   const SmVector3d   * cpOptVectors,     // in : cpOptVectors[0] = view vector used to specify silhouette curves.
   double               d3DTolerance,     // in : distance tolerance
   SmTArray<SmCurve*> & rTrim3DCurves,    // out: array of 3D silhouette curves
   SmTArray<SmCurve*> & rTrimUVCurves)    // out: associated UVTrimCurves
{
  // locals and temporaries
  SmSurface       *pSurface = pFace->GetSurface();
  SmSurfaceCache  *pSC      = smsurf_GetSurfaceCache(pSurface); NER(pSC);
  const SmContext *pContext = pFace->GetContext();
  SmObjsDelete<SmCurve*> sClean1(&rTrim3DCurves);
  SmObjsDelete<SmCurve*> sClean2(&rTrimUVCurves);

  SmBoolean bFalse = FALSE;
  SmApproxTol3d d3DTol    = SM_CAST_APPROXTOL3D(d3DTolerance * 10.0) ;
  double dAngleTol = 20.0 * SM_PI / 180;

  // when silhouette curves to these tolerances have been made - get copies
  if (!pSC->HasCachedCurves(*pContext,SM_AS_TRIMMED_SILHOUETTE,
                            d3DTol, dAngleTol, &cpOptVectors[0],
                            &bFalse, NULL, NULL, NULL,
                            &rTrim3DCurves,&rTrimUVCurves)) 
    {
      // No cached trimmed silhouette curves in existance -- create them
      SmTArray<SmCurve*> s3DCurves, sUVCurves;
      SmObjsDelete<SmCurve*> sClean3(&s3DCurves);
      SmObjsDelete<SmCurve*> sClean4(&sUVCurves);
      SmBSplineSurface *pSur = SM_CAST_PTR(SmBSplineSurface,pSurface);

      // Create the planar section curves from the brep and the plane
      SER(pSur->CreateSilhouetteCurves(*pContext,
                                       pFace->GetUVDomain(),
                                       cpOptVectors[0], 
                                       FALSE,
                                       &d3DTol, 
                                       &dAngleTol, 
                                       &s3DCurves,
                                       &sUVCurves));
      // Now trim the curves.
      SER(pFace->CreateCurvesByTrimming(*pContext, 
                                        SM_CAST_APPROXTOL3D_PTR(&d3DTol),
                                        &s3DCurves, 
                                        &sUVCurves, 
                                        &rTrim3DCurves, 
                                        &rTrimUVCurves));

      // Register trimmed curves and create copy by getting them again
      SER(pSC->AddCurvesToCache(SM_AS_TRIMMED_SILHOUETTE,
                                SM_CAST_DOUBLE(d3DTol), 
                                dAngleTol, 
                                &cpOptVectors[0], 
                                &bFalse,
                                NULL,NULL,NULL,
                                &rTrim3DCurves,
                                &rTrimUVCurves));

      // Don't clean up here we just use the curves that were created
      if (!pSC->HasCachedCurves(*pContext,
                                SM_AS_TRIMMED_SILHOUETTE,
                                SM_CAST_DOUBLE(d3DTol), 
                                dAngleTol, 
                                &cpOptVectors[0],
                                &bFalse, 
                                NULL, NULL, NULL,
                                &rTrim3DCurves,
                                &rTrimUVCurves)) 
        {
          SER(SM_ERR);  
        }
  }
  sClean1.Clear();
  sClean2.Clear();
  return SM_SUCCESS;

} // end sm_GetFaceSilhouettes


/*******************************************************************//**
PURPOSE: Do local solve between point and vertex.

NOTES: add solutions to m_rTopologySolver.m_pSolutions
***********************************************************************/
SmStatus SmTLSPointVertex::SolveIt
  (const SmObject *pObj1, 
   const SmObject *pObj2)
{
  // sol that when found is added to SmGlobalSolver::m_pSolutions list
  SmSolution sSol;

  // set sSol m_lNumObjects and m_apObjects array
  //   Normally the point comes first and the vertex second.  
  //   when m_bSwapOrder == TRUE, switch the order.
  SmVertex *pV;
  ULONG     lObjectCount = 0;
  sSol.m_apObjects[1] = NULL;
  if (!m_bSwapOrder) 
    {
      pV = (SmVertex*)pObj2;  NER(pV);
      if (m_pPointVertex) 
        {
          // If the point represents a vertex then save it in list
          sSol.m_apObjects[lObjectCount++] = SM_REINTERPRET_CAST(SmObject*, SM_CONST_CAST(SmVertex*,m_pPointVertex));
        }
      sSol.m_apObjects[lObjectCount++] = pV;
    }
  else 
    {
      pV = (SmVertex*)pObj1;  NER(pV);
      sSol.m_apObjects[lObjectCount++] = pV;
      if (m_pPointVertex) 
        {
          // If the point represents a vertex then save it in list
          sSol.m_apObjects[lObjectCount++] = SM_REINTERPRET_CAST(SmObject*, SM_CONST_CAST(SmVertex*,m_pPointVertex));
        }
    }
  sSol.m_lNumObjects = lObjectCount;

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
  if (bDebugMe) 
    {
      sm_GraphicsLoop();
      smgfx_Erase();
      smgfx_SetLook(2,4, 1,0,0); m_vPoint.Draw(); sm_GraphicsLoop();
      smgfx_SetLook(2,4, 0,0,1); pV->GetPoint().Draw(); sm_GraphicsLoop();
      smgfx_SetLook(1,1, 0,0,0); ((SmVertex*)pObj1)->GetBrep()->Draw(); sm_GraphicsLoop();
      smgfx_SetLook(1,1, 0,0.5,0.5); ((SmVertex*)pObj2)->GetBrep()->Draw(); sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
#endif

  // locals
  SmVector3d sDiff   = m_vPoint - pV->GetPoint();
  double     dDistSq = sDiff.LengthSquared();
  double     dSign   = 1.0;

  // When projecting, project the difference vector
  if (m_rTopologySolver.m_bProjectedOperation == TRUE) 
    {
      SmVector3d sPlaneNorm;
      SmVector3d sVecProj;
      if(   m_rTopologySolver.m_eSolverOperation == SM_SO_SIGNED_DIRECTED_MINIMIZE 
         || m_rTopologySolver.m_eSolverOperation == SM_SO_DIRECTED_MINIMIZE 
         || m_rTopologySolver.m_eSolverOperation == SM_SO_DIRECTED_MAXIMIZE)
        {
          sPlaneNorm = m_rTopologySolver.m_cpOptVectors[0];
          sVecProj   = sPlaneNorm * sDiff * sPlaneNorm;
          if (!sVecProj.IsParallelTo(m_rTopologySolver.m_cpOptVectors[1],2.0))
              return SM_SUCCESS; // no answers in this branch
        }
      else 
        {
          sPlaneNorm = m_rTopologySolver.m_cpOptVectors[0];
          sVecProj   = sPlaneNorm * sDiff * sPlaneNorm;
        }
      dDistSq = sVecProj.LengthSquared();
      if(   m_rTopologySolver.m_eSolverOperation == SM_SO_SIGNED_DIRECTED_MINIMIZE
         && sVecProj.Dot(m_rTopologySolver.m_cpOptVectors[1]) < 0) 
        {
          dSign = -1.0;
        }
    }

  // when order is reversed and doing a signed operation - reverse the sign of the result
  if (m_bSwapOrder && m_rTopologySolver.m_eOperationCategory == SM_OPERATION_SIGNED) 
    {
      dSign = -1.0 * dSign;
    }

  // switch on operation type to set dSign and dDistSq of Sol = dSign * sqrt(dDistSq)
  switch (m_rTopologySolver.m_eSolverOperation) 
    {
      case SM_SO_RAYFIRE:
        {
          double dParam;
          SER(smgu_LineClosestPoint(m_vPoint,
                                    m_rTopologySolver.m_cpOptVectors[0],
                                    pV->GetPoint(),
                                    dParam));
          double dTol   = m_rTopologySolver.m_d3dTolerance + pV->GetTolerance();
          double dScale = m_vPoint.GetMaxDimension();

          // No answer if we are out of tol on wrong side of ray
          if (dParam < -SM_EFF_ZERO*dScale) 
            {
              return SM_SUCCESS;  
            }

          // When in tol on wrong side or ray - snap to end point
          if (dParam < 0.0) 
            {
              dParam = 0.0; 
            }

          SmPoint3d sRayPnt    = m_vPoint + dParam * m_rTopologySolver.m_cpOptVectors[0];
          double    dDistToRay = sRayPnt.DistanceBetween(pV->GetPoint());

          // no answer - ray too from from vertex
          if (dDistToRay > dTol) 
            { return SM_SUCCESS ; }

          // The following assumes that the ray vector is unitized
          dDistSq = dParam * dParam;
        } // end SM_SO_RAYFIRE

      case SM_SO_3D_SIGNED_DIRECTED_MINIMIZE:
        {
          double dParam;
          SER(smgu_LineClosestPoint(m_vPoint,
                                    m_rTopologySolver.m_cpOptVectors[0],
                                    pV->GetPoint(),
                                    dParam));
          double dTol = m_rTopologySolver.m_d3dTolerance + pV->GetTolerance();

          SmPoint3d sRayPnt    = m_vPoint + dParam * m_rTopologySolver.m_cpOptVectors[0];
          double    dDistToRay = sRayPnt.DistanceBetween(pV->GetPoint());

          // no answer - ray too far from vertex
          if (dDistToRay > dTol) 
            { return SM_SUCCESS ; }

          if (m_bSwapOrder) dParam = - dParam;

          // The following assumes that the direction vector is unitized
          dDistSq = dParam * dParam;
          dSign   = 1.0;
          if (dParam < 0.0) dSign = -1.0;
       
        } // end SM_SO_3D_SIGNED_DIRECTED_MINIMIZE

      case SM_SO_INTERSECT:
          break;
  
      case SM_SO_PROJECTED_MINIMIZE:
      case SM_SO_MINIMIZE:
      case SM_SO_DIRECTED_MINIMIZE:
        {
          if (dDistSq > m_rTopologySolver.m_dBestAnswerSoFarSq) 
            {
              if (smos_Sqrt(dDistSq) - m_rTopologySolver.m_d3dTolerance > 
                  smos_Sqrt(m_rTopologySolver.m_dBestAnswerSoFarSq) ) 
                {
                  return SM_SUCCESS; // no answers in this branch
                }
            }
          break;
        } // end SM_SO_PROJECTED_MINIMIZE, SM_SO_MINIMIZE, SM_SO_DIRECTED_MINIMIZE

      case SM_SO_SIGNED_DIRECTED_MINIMIZE:
        {
          if (dSign*dDistSq > m_rTopologySolver.m_dBestAnswerSoFarSq) 
            {
              double dBestAnswer = smos_Sqrt(smos_Fabs(m_rTopologySolver.m_dBestAnswerSoFarSq));
              if (m_rTopologySolver.m_dBestAnswerSoFarSq < 0.0) dBestAnswer = - dBestAnswer;

              // no answers in this branch
              if (dSign*smos_Sqrt(dDistSq) - m_rTopologySolver.m_d3dTolerance > dBestAnswer) 
                { return SM_SUCCESS; }
            }
          break;
        } // end SM_SO_SIGNED_DIRECTED_MINIMIZE

      case SM_SO_PROJECTED_MAXIMIZE:
      case SM_SO_MAXIMIZE:
      case SM_SO_DIRECTED_MAXIMIZE:
        {
          if (dDistSq < m_rTopologySolver.m_dBestAnswerSoFarSq) 
            {
              if (smos_Sqrt(dDistSq) + m_rTopologySolver.m_d3dTolerance <
                  smos_Sqrt(m_rTopologySolver.m_dBestAnswerSoFarSq) ) 
                {
                  return SM_SUCCESS; // no answers in this branch
                }
            }
          break;
        } // end SM_SO_PROJECTED_MAXIMIZE, SM_SO_MAXIMIZE, SM_SO_DIRECTED_MAXIMIZE

      case SM_SO_AT_DISTANCE:
        {
          if (dDistSq < m_rTopologySolver.m_dAtDistanceSq - SM_EFF_ZERO*(1.0 + dDistSq))     // GWC: added numerical tolerance
            {
              // no answers in this branch
              if (smos_Sqrt(dDistSq) - m_rTopologySolver.m_d3dTolerance < 
                  smos_Sqrt(m_rTopologySolver.m_dAtDistanceSq) ) 
                { return SM_SUCCESS; }
            }  
            
          // no answers in this branch
          if (dDistSq > m_rTopologySolver.m_dAtDistanceSq + SM_EFF_ZERO*(1.0 + dDistSq))     // GWC: added numerical tolerance
            { 
              if (smos_Sqrt(dDistSq) + m_rTopologySolver.m_d3dTolerance > 
                  smos_Sqrt(m_rTopologySolver.m_dAtDistanceSq) ) 
                { return SM_SUCCESS; }
            }
          break;
        } // SM_SO_AT_DISTANCE
  
      case SM_SO_NORMALIZE:
          break;

      case SM_SO_FIND:
          break;

      default:
          ERR(SM_ERR);  // not handled yet

    } // end switch on m_rTopologySolver.m_eSolverOperation

  // arrive here when solution was found
  //   dSign   = +/- 1 to handle swap object and directed solution values
  //   dDistSq = solution value squared

  // set solution 
  sSol.m_eSolutionType           = SM_ST_SINGLE_VALUE;
  sSol.m_lNumVariables           = 0;
  sSol.m_vStart.m_dSolutionValue = dSign*smos_Sqrt(dDistSq);

  // accumulate unique solutions sorted by value into SmGlobalSolver::m_pSolutions
  m_rTopologySolver.AddSortedSolution(sSol,SM_SK_BY_SOLUTION_VALUE);

  // all done
  return SM_SUCCESS;

} // end SmTLSPointVertex::SolveIt

/*******************************************************************//**
PURPOSE: Local solver for Vertex Vertex.  Just invokes
   superclass after loading data.

NOTES:  add solutions to m_rTopologySolver.m_pSolutions
***********************************************************************/
SmStatus SmTLSVertexVertex::SolveIt
  (const SmObject *pObj1, 
   const SmObject *pObj2)
{
  SM_ASSERT(pObj1 != NULL && pObj2 != NULL);
  SM_ASSERT(pObj1->IsKindOf(SmVertex_TYPE));
  SM_ASSERT(pObj2->IsKindOf(SmVertex_TYPE));
  if (!m_bSwapOrder) { m_pPointVertex = (SmVertex*)pObj1; }
  else               { m_pPointVertex = (SmVertex*)pObj2; }

  // Load the Point value
  m_vPoint = m_pPointVertex->GetPoint();

  // pass the call along to the Point/Vertex solver
  SER(SmTLSPointVertex::SolveIt(pObj1,pObj2));
  return SM_SUCCESS;

} // end SmTLSVertexVertex::SolveIt

/*******************************************************************//**
PURPOSE: Do local solve between point and curve.

NOTES: add solutions to m_rTopologySolver.m_pSolutions
***********************************************************************/
SmStatus SmTLSPointCurve::SolveIt
  (const SmObject *pObj1, 
   const SmObject *pObj2)
{
  // Normally the point comes first and the curve second.  If
  // we want we can switch the order.
  SmSolution sSol;
  sSol.m_apObjects[0] = NULL;
  sSol.m_apObjects[1] = NULL;

  SmCurve *pCurve;
  ULONG lObjectCount = 0;
  if (!m_bSwapOrder) 
    {
      pCurve = SM_REINTERPRET_CAST(SmCurve*,SM_CONST_CAST(SmObject*,pObj2));
      if (m_pPointVertex) 
        {
          // If the point represents a vertex then save it in list
          sSol.m_apObjects[lObjectCount++] = SM_REINTERPRET_CAST(SmObject*,
              SM_CONST_CAST(SmVertex*,m_pPointVertex));
        }
      if (m_pCurveOwner) 
        {
          sSol.m_apObjects[lObjectCount++] = SM_CONST_CAST(SmObject*,m_pCurveOwner);
        }
      else 
        {
          sSol.m_apObjects[lObjectCount++] = pCurve;
        }
    }
  else 
    {
      pCurve = SM_REINTERPRET_CAST(SmCurve*,SM_CONST_CAST(SmObject*,pObj1));
      if (m_pCurveOwner) 
        {
          sSol.m_apObjects[lObjectCount++] = SM_CONST_CAST(SmObject*,m_pCurveOwner);
        }
      else 
        {
          sSol.m_apObjects[lObjectCount++] = pCurve;
        }
      if (m_pPointVertex) 
        {
          // If the point represents a vertex then save it in list
          sSol.m_apObjects[lObjectCount++] = SM_REINTERPRET_CAST(SmObject*,
              SM_CONST_CAST(SmVertex*,m_pPointVertex));
        }
    }


#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
  if (bDebugMe) 
    {
      pCurve->Dump();
      sm_GraphicsLoop();
      smgfx_SetLook(2,4, 0,0,1); pCurve->DrawWDeriv(m_vInterval,0); sm_GraphicsLoop();
      smgfx_SetLook(2,4, 1,0,0); m_vPoint.Draw(); sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
#endif        

  // locals for GlobalPointSolve() call 
  double *pdBestAnswer = NULL;
  double  dBestAnswer;
  m_rTopologySolver.GetBestAnswerSoFar(dBestAnswer, pdBestAnswer);
  double dSign = 1.0;
  const SmVector3d *pVecs = &m_rTopologySolver.m_cpOptVectors[0];
  SmVector3d sVecs[4];
  if (m_bSwapOrder && m_rTopologySolver.m_eSolverOperation == SM_SO_3D_SIGNED_DIRECTED_MINIMIZE) 
    {
      sVecs[0] = - m_rTopologySolver.m_cpOptVectors[0];
      pVecs = &sVecs[0];
    }
  else if (m_bSwapOrder && m_rTopologySolver.m_eOperationCategory == SM_OPERATION_SIGNED) 
    {
      // If we are doing a signed operation and the order is reversed then we have to
      // reverse the sign of the result.
      dSign = -1.0 * dSign;
    }
  if (pdBestAnswer) 
    {
      *pdBestAnswer = *pdBestAnswer * dSign;
    }
  SmSolution aData[10];
  SmSolutionArray sSolutions(10,aData);

  // make the PointSolve call
  SER(pCurve->GlobalPointSolve(m_vInterval,
                               m_rTopologySolver.m_eSolverOperation, 
                               m_vPoint,
                               m_rTopologySolver.m_d3dTolerance,
                               pdBestAnswer, pVecs, SM_SR_ALL,
                               sSolutions));

  // iter every solution building final solution list.
  //   Set the Solution Object values and 
  //   Sort into m_rTopologySolver.m_pSolutions array
  for (ULONG i=0; i<sSolutions.GetSize(); i++) 
    {
      SmSolution & rSol = sSolutions[i];

      // If we have a surface curve then put the corresponding,
      // surface parameters into the output instead of the curve parameters.
      rSol.m_lNumVariables = 1;
      if (m_pSurfaceCurve) 
        {
          SmPoint3d sPnt;
          SER(m_pSurfaceCurve->EvaluatePoint(rSol.m_vStart[0],sPnt));
          rSol.m_vStart[0] = sPnt.x;
          rSol.m_vStart[1] = sPnt.y;
          rSol.m_lNumVariables = 2;
        }
      rSol.m_apObjects[0] = sSol.m_apObjects[0];
      rSol.m_apObjects[1] = sSol.m_apObjects[1];
      rSol.m_lNumObjects = lObjectCount;
      rSol.m_vStart.m_dSolutionValue = dSign * rSol.m_vStart.m_dSolutionValue;
      if (rSol.m_eSolutionType  == SM_ST_RANGE_OF_VALUES) 
        {
          rSol.m_vEnd.m_dSolutionValue = dSign * rSol.m_vEnd.m_dSolutionValue;
        }
      m_rTopologySolver.AddSortedSolution(rSol,SM_SK_BY_SOLUTION_VALUE);
    
    } // end iter every solution building final solution list

  return SM_SUCCESS;

} // end SmTLSPointCurve::SolveIt

/*******************************************************************//**
PURPOSE: Do local solve between point and edge(s).

NOTES:  add solutions to m_rTopologySolver.m_pSolutions
***********************************************************************/
SmStatus SmTLSPointEdge::SolveIt
  (const SmObject *pObj1, 
   const SmObject *pObj2)
{
  // locals
  SmSolution sSol;

  // Normally the point comes first and the curve second.  
  // m_bSwapOrder is set when that ordering is switched.
  const SmCurve *pCurve;
  if (!m_bSwapOrder) { pCurve = (SmCurve*)pObj2; }
  else               { pCurve = (SmCurve*)pObj1; }
  

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  // draw 
  if(bDebugMe)
    {
      SM_ASSERT_VALID(pCurve) ;
      
      SmEdge *pEdge = (SmEdge *)pCurve->GetEdge() ;
      SmBrep *pBrep =  pEdge ? pEdge->GetBrep() : NULL ;
                      
      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(3,4, 1,0,0) ; if(pCurve) pCurve->Draw(NULL, TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(5,6, 1,0,1) ; if(pEdge) pEdge->Draw() ; sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

// Remove Composites - replaced Edge array with single curve Edge
  SM_ASSERT_MSG(pCurve->GetEdge() != NULL, _T("SmTLSPointEdge::SolveIt - Remove Composite single Edge assumption wrong here - needs debug")) ;
  SmEdge * pEdge = (SmEdge *)pCurve->GetEdge() ; 

// Remove Composites
//   // get curve edge(s)
//   //   note: composite curves have a list of CEdge SmEdge children.
//   //         regular curves have just one SmEdge.
//   SmEdge *sEData[16];
//   SmTArray<SmEdge*> sEdges(16,sEData);
//   SmBrep::GetEdgesOfCurve(pCurve,sEdges);
// 
//   // for every edge using the target curve
//   for (ULONG i=0; i<sEdges.GetSize(); i++) 
//     {
//       SmEdge *pEdge = sEdges[i];
//
//      // skip edges not listed - these edges get included when part of a CEdge
//      if(    m_pShape 
//         && !m_pShape->IsInShape(pEdge))
//        { continue; }

// Remove Composites - 1 check brought down from block above
  // when using m_pShape (called frome ShapePointSolve(), ShapeCurveSolve(), or ShapeShapeSolve()) - only operate on Edges in Shape
  if(m_pShape == NULL || m_pShape->IsInShape(pEdge))
    {
      m_pCurveOwner = pEdge;
      m_vInterval   = pEdge->GetInterval();
#ifdef SM_DEBUG_CODE
      if (bDebugMe) 
        {
          SmCurve *pEC = pEdge->GetCurve();
          SmBrep  *pBrep = pEdge->GetBrep() ;
          SmTopologySolver     &rTopologySolver  = GetTopologySolver() ;
          SmSolverOperationType eSolverOperation = rTopologySolver.GetSolverOperationType() ;
          SmVector3d sRayVector ;
          if(eSolverOperation == SM_SO_RAYFIRE)
            { sRayVector = rTopologySolver.GetOptVector(0) ; }

          smgfx_Erase();
          smgfx_SetLook(1,2, 0,0,1); if(pBrep) pBrep->Draw(TRUE); sm_GraphicsLoop();
          smgfx_SetLook(6,7, 0,1,0); m_vPoint.Draw(); sm_GraphicsLoop();
          smgfx_SetLook(1,2, 1,0,0); if(eSolverOperation == SM_SO_RAYFIRE) (30.0 * sRayVector).Draw(&m_vPoint) ; sm_GraphicsLoop() ;
          smgfx_SetLook(2,4, 0,0,0); pEdge->Draw(); sm_GraphicsLoop();
          smgfx_SetLook(4,6, 0,1,1); pEC->DrawWDeriv(pEdge->GetInterval(),0); sm_GraphicsLoop();
          sm_GraphicsLoop();
        }
#endif

      // arrive here after ensuring pEdge is part of the shape
      //  and loading m_vInterval with the limiting interval.
      //  m_vPoint is already loaded with the target point.

      // pass the call along to get solutions.
      //   The target point is stored in m_vPoint, not the input args.
      //   So - put curve in as both args makes sure this works
      //        no matter the value of m_bSwapOrder
      SER(SmTLSPointCurve::SolveIt(pCurve,pCurve));
    }

  return SM_SUCCESS;

} // end SmTLSPointEdge::SolveIt

/*******************************************************************//**
PURPOSE: Local solver for Vertex Edge.  Just invokes
   superclass after loading data.

NOTES:  add solutions to m_rTopologySolver.m_pSolutions
***********************************************************************/
SmStatus SmTLSVertexEdge::SolveIt
  (const SmObject *pObj1, 
   const SmObject *pObj2)
{
  SM_ASSERT(pObj1 != NULL && pObj2 != NULL);
  if (!m_bSwapOrder) 
    {
      SM_ASSERT(pObj1->IsKindOf(SmVertex_TYPE));
      SM_ASSERT(pObj2->IsKindOf(SmCurve_TYPE));
      m_pPointVertex = (SmVertex*)pObj1;
    }
  else 
    {
      SM_ASSERT(pObj1->IsKindOf(SmCurve_TYPE));
      SM_ASSERT(pObj2->IsKindOf(SmVertex_TYPE));
      m_pPointVertex = (SmVertex*)pObj2;
    }

  m_vPoint = m_pPointVertex->GetPoint();

  SER(SmTLSPointEdge::SolveIt(pObj1,pObj2));
  return SM_SUCCESS;

} // end SmTLSVertexEdge::SolveIt

/*******************************************************************//**
PURPOSE: Do local solve between point and a face(s).

NOTES:  add solutions to m_rTopologySolver.m_pSolutions
   for cases SM_SO_RAYFIRE and SM_SO_3D_SIGNED_DIRECTED_MINIMIZE:
     m_dSolutionValue is set to the parameter along the ray
     [0] Surface u-value
     [1] Surface v-value
   when !m_bSwapOrder, Sol.m_apObjects list:[m_pPointVertex (if available), pSurface]
   else                Sol.m_apObjects list:[pSurface, m_pPointVertex (if available)]
***********************************************************************/
SmStatus SmTLSPointFace::SolveIt
  (const SmObject *pObj1,  // in : !m_bSwapOrder ? NotUsed : Surface ;
   const SmObject *pObj2)  // in : !m_bSwapOrder ? Surface : NotUsed ;
{
  // locals
  ULONG       lObjectCount = 0;
  SmSurface * pSurface;
  SmFace    * sFData[16];
  SmTArray<SmFace*> sFaces(16,sFData);
  SmSolution  sSol;
  sSol.m_apObjects[1] = NULL;

  // Normally the point comes first and the surface second.  If
  // we want we can switch the order.
  if (!m_bSwapOrder) 
    {
      pSurface = (SmSurface*)pObj2; NER(pSurface);
      if (m_pPointVertex) 
        {
          // If the point represents a vertex then save it in list
          sSol.m_apObjects[lObjectCount++] = SM_REINTERPRET_CAST(SmObject*,
                                                                 SM_CONST_CAST(SmVertex*,m_pPointVertex));
        }
      sSol.m_apObjects[lObjectCount++] = SM_REINTERPRET_CAST(SmObject*,
                                                             SM_CONST_CAST(SmSurface*,pSurface));
    }
  else 
    {
      pSurface = (SmSurface*)pObj1; NER(pSurface);
      sSol.m_apObjects[lObjectCount++] = SM_REINTERPRET_CAST(SmObject*,
                                                             SM_CONST_CAST(SmSurface*,pSurface));
      if (m_pPointVertex) 
        {
          // If the point represents a vertex then save it in list
          sSol.m_apObjects[lObjectCount++] = SM_REINTERPRET_CAST(SmObject*,
                                                                 SM_CONST_CAST(SmVertex*,m_pPointVertex));
        }
    }

// Remove Composites - replaced face array with single surface face
  SM_ASSERT_MSG(pSurface->GetFace() != NULL, _T("SmTLSPointFace::SolveIt - Remove Composite single face assumption wrong here - needs debug")) ;
  SmFace * pFace = pSurface->GetFace() ; 
  
// Remove Composites
//  // get faces of surface
//  SmBrep::GetFacesOfSurface(pSurface,sFaces);
//  
//  // for every surface face
//  for (ULONG ii=0; ii<sFaces.GetSize(); ii++) 
//    {
//      SmFace    * pFace             = sFaces[ii];
//      // SmZoneTol3d sFaceZoneTol3d = SmTol::GetZoneTol3d(pFace) ;
//
//      // skip faces not listed - these faces get included when part of a CFace
//      if (m_pShape && !m_pShape->IsInShape(pFace))
//        { continue; }
//
//      // check state - m_bEditingEnabled must be off
//      //      SM_ASSERT(pFace->GetBrep()->m_bEditingEnabled == FALSE);
//      // gwc: remove check
//      //      if (pFace->GetBrep()->m_bEditingEnabled) 
//      //        { SER_MSG(SM_ERR, _T("Tried to edit SmTLSPointFace::SolveIt with m_bEditingEnabled == TRUE")); }
      
// Remove Composites - 1 check line brought down from block above
  // when using m_pShape (called frome ShapePointSolve(), ShapeCurveSolve(), or ShapeShapeSolve()) - skip pFace not in shape
  if(m_pShape == NULL || m_pShape->IsInShape(pFace))
    {
      // branch on m_eSolverOperation val to find and load solutions into output m_rTopologySolver.m_pSolutions 

      if(   m_rTopologySolver.m_eSolverOperation == SM_SO_RAYFIRE
         || m_rTopologySolver.m_eSolverOperation == SM_SO_3D_SIGNED_DIRECTED_MINIMIZE) 
        {
          // locals
          double        * pdBestAnswer = NULL;
          double          dBestAnswer;
          SmSolution      aData[10];
          SmSolutionArray sSolutions(10,aData);

          m_rTopologySolver.GetBestAnswerSoFar(dBestAnswer, pdBestAnswer);

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
          if (bDebugMe) 
            {
              smgfx_Erase();
              pFace->Draw(); sm_GraphicsLoop();
//                pSurface->Draw(); sm_GraphicsLoop();
              m_vPoint.Draw(); sm_GraphicsLoop();
              m_rTopologySolver.m_cpOptVectors[0].Draw(&m_vPoint); sm_GraphicsLoop();
              sm_GraphicsLoop();
            }
#endif // SM_DEBUG_CODE

          SmVector3d sLineVec = m_rTopologySolver.m_cpOptVectors[0];
          SmBoolean  bRayFire = TRUE;
          if (m_rTopologySolver.m_eSolverOperation == SM_SO_3D_SIGNED_DIRECTED_MINIMIZE) 
            {
              bRayFire = FALSE;
              if (m_bSwapOrder) sLineVec = - sLineVec;
            }
          SER(pFace->GetSurface()->GlobalLineIntersect(pFace->GetUVDomain(),
                                                    m_vPoint, sLineVec, NULL, bRayFire,
                                                    m_rTopologySolver.m_d3dTolerance + pFace->GetTolerance(),
                                                    sSolutions));

          for (ULONG i=0; i<sSolutions.GetSize(); i++) 
            {
              SmSolution & rSol = sSolutions[i];
              double dScale = m_vPoint.GetMaxDimension();
              if (m_rTopologySolver.m_eSolverOperation == SM_SO_RAYFIRE && 
                  rSol.m_vStart[0] < -SM_EFF_ZERO*dScale) 
                {
                  continue;
                }


              // We've found a result within the surface domain.
              // It must also be within the Face: this is a topology routine.
              // [060718 bd]
              SmZoneTol3d sSrcZoneTol3d = SmTol::GetZoneTol3d(m_pPointVertex) ; // if m_pPointVertex is allowed to be NULL
              SmPointClassification sPointClass(sSrcZoneTol3d, pFace->GetContext());  // this tol is unlikely to be correct - check it

              SmPoint2d sUV( rSol.m_vStart[1], rSol.m_vStart[2] );
              SmBoolean bDoBdries = TRUE; // don't care? Just making it the same as the others.
              SmBoolean bUseUV    = TRUE; // definitely: we have that info so use it.

                                                                                // then we need to store a SrcZoneTol3d value
              pFace->PointClassify( sUV, sSrcZoneTol3d, bDoBdries, bUseUV, sPointClass );
              if ( sPointClass.GetPointClass() == SM_PC_UNKNOWN ) {
                  continue;
              }

              // We need to do something here if we have a coincident segment to
              // classify the answer by seeing if the start point is on the 
              // surface.  If not then skip it.
              if ( rSol.m_eSolutionType == SM_ST_RANGE_OF_VALUES )
                {
                  SmPoint3d sStart = m_vPoint + rSol.m_vStart[0] * m_rTopologySolver.m_cpOptVectors[0];
                  if (pFace->Point3DClassify(sStart,m_rTopologySolver.m_d3dTolerance,
                                          TRUE, sPointClass) != SM_SUCCESS) 
                    {
                      if (pFace->Point3DClassify(sStart,m_rTopologySolver.m_d3dTolerance*100.0,
                                              TRUE, sPointClass) != SM_SUCCESS) 
                        {
                          continue;
                        }   
                    }
                  if (sPointClass.GetPointClass() != SM_PC_FACE) 
                    {
                      continue;
                    }
                }

              // Handle composite face situation
              if (pFace->GetSurface()->GetOwner() != pFace) 
                {
                  SmPoint3d sStart = m_vPoint + rSol.m_vStart[0] * m_rTopologySolver.m_cpOptVectors[0];
                  if (pFace->Point3DClassify(sStart,m_rTopologySolver.m_d3dTolerance,
                                          TRUE, sPointClass) != SM_SUCCESS) 
                    {
                      if (pFace->Point3DClassify(sStart,m_rTopologySolver.m_d3dTolerance*100.0,
                                              TRUE, sPointClass) != SM_SUCCESS) 
                        {
                          continue;
                        }
                    }
                  if (sPointClass.GetPointClass() != SM_PC_FACE) 
                    {
                      continue;
                    }
                } // end composite face check
      
              // load the solution
              rSol.m_apObjects[0] = sSol.m_apObjects[0];
              rSol.m_apObjects[1] = sSol.m_apObjects[1];
              if (sSol.m_apObjects[0] == pSurface) rSol.m_apObjects[0] = pFace;
              if (sSol.m_apObjects[1] == pSurface) rSol.m_apObjects[1] = pFace;

              rSol.m_lNumObjects = lObjectCount;
              rSol.m_lNumVariables = 2;
              // Shift the results for rayfire.
              rSol.m_vStart.m_dSolutionValue = rSol.m_vStart[0];
              rSol.m_vStart[0] = rSol.m_vStart[1];
              rSol.m_vStart[1] = rSol.m_vStart[2];

              // add solution to output m_rTopologySolver.m_pSolutions if appropriate
              m_rTopologySolver.AddSortedSolution(rSol,SM_SK_BY_SOLUTION_VALUE);

            } // end for each solution

        } // end SM_SO_RAYFIRE || SM_SO_3D_SIGNED_DIRECTED_MINIMIZE case

      else if (m_rTopologySolver.m_bProjectedOperation == TRUE) 
        {
          SmTArray<SmCurve*> sTrim3DCurves, sTrimUVCurves;
          SmObjsDelete<SmCurve*> sClean1(&sTrim3DCurves);
          SmObjsDelete<SmCurve*> sClean2(&sTrimUVCurves);
          SER(sm_GetFaceSilhouettes(pFace,m_rTopologySolver.m_cpOptVectors,
                                    m_rTopologySolver.m_d3dTolerance,
                                    sTrim3DCurves, sTrimUVCurves));
          // When we reach here there should be silhouette curves in sTrim3DCurves
          // We can use these to do measurement to the surface.  Please note that
          // the edges bounding the face would have already been measured.
          for (ULONG i=0; i<sTrim3DCurves.GetSize(); i++) 
            {
              SmCurve *pCurve = sTrim3DCurves[i];
              SmTLSPointCurve sTSLSolver(m_rTopologySolver, m_bSwapOrder, m_pShape);
              sTSLSolver.m_pCurveOwner = pFace;
              sTSLSolver.m_vInterval = pCurve->GetNaturalInterval();
              sTSLSolver.m_pSurfaceCurve = sTrimUVCurves[i];
              sTSLSolver.m_vPoint = m_vPoint;
              sTSLSolver.m_pPointVertex = m_pPointVertex;
              SER(sTSLSolver.SolveIt(pCurve,pCurve));
            }
        } // end a ProjectedOperation branch
      else // not a ProjectedOperation branch
        {
          double *pdBestAnswer = NULL;
          double dBestAnswer;
          m_rTopologySolver.GetBestAnswerSoFar(dBestAnswer, pdBestAnswer);
          double dSign = 1.0;
          if (m_bSwapOrder && m_rTopologySolver.m_eOperationCategory == SM_OPERATION_SIGNED) 
            {
              // If we are doing a signed operation and the order is reversed then we have to
              // reverse the sign of the result.
              dSign = -1.0 * dSign;
            }
          if (pdBestAnswer) 
            {
              *pdBestAnswer = *pdBestAnswer * dSign;
            }
          SmSolution aData[10];
          SmSolutionArray sSolutions(10,aData);
      
          SER(pFace->GetSurface()->GlobalPointSolve(pFace->GetUVDomain(),
              m_rTopologySolver.m_eSolverOperation, m_vPoint, 
              m_rTopologySolver.m_d3dTolerance + pFace->GetTolerance(),
              pdBestAnswer,SM_SR_ALL,sSolutions));
      
          for (ULONG i=0; i<sSolutions.GetSize(); i++) 
            {
              SmSolution & rSol = sSolutions[i];

              // Check whether the point is within the Face's trimmed boundaries.
              SmZoneTol3d sSrcZoneTol3d = SmTol::GetZoneTol3d(m_pPointVertex) ; // if m_pPointVertex is allowed to be NULL
              SmPointClassification sPtClass(sSrcZoneTol3d, pFace->GetContext());  // this tol is unlikely to be correct - check it
              SmPoint2d sUV( rSol.m_vStart[0], rSol.m_vStart[1] );
              pFace->PointClassify( sUV, sSrcZoneTol3d, TRUE, TRUE, sPtClass );
              if ( sPtClass.GetPointClass() == SM_PC_UNKNOWN )
                { continue; }

              // (Note: Surface::GlobalPointSolve() does not put anything
              // into its 'objects' arrays; doesn't have Point ptr anyway.)
              rSol.m_apObjects[0] = sSol.m_apObjects[0];
              rSol.m_apObjects[1] = sSol.m_apObjects[1];
              if (sSol.m_apObjects[0] == pSurface) rSol.m_apObjects[0] = pFace;
              if (sSol.m_apObjects[1] == pSurface) rSol.m_apObjects[1] = pFace;
              rSol.m_lNumObjects = lObjectCount;
              rSol.m_lNumVariables = 2;
              rSol.m_vStart.m_dSolutionValue = dSign * rSol.m_vStart.m_dSolutionValue;
              if (rSol.m_eSolutionType  == SM_ST_RANGE_OF_VALUES) 
                {
                  rSol.m_vEnd.m_dSolutionValue = dSign * rSol.m_vEnd.m_dSolutionValue;
                }
              m_rTopologySolver.AddSortedSolution(rSol,SM_SK_BY_SOLUTION_VALUE);
            }
        } // end not a ProjectedOperation branch

// Remove Composites - changed next closing scope comment
//    } // end iter every surface face finding and adding solutions to output m_rTopologySolver.m_pSolutions
    } // end surface->face scope for finding and adding solutions to output m_rTopologySolver.m_pSolutions

  // all done
  return SM_SUCCESS;

} // end SmTLSPointFace::SolveIt

/*******************************************************************//**
PURPOSE: Local solver for Vertex Face.  Just invokes
   superclass after loading data.

NOTES:  add solutions to m_rTopologySolver.m_pSolutions
***********************************************************************/
SmStatus SmTLSVertexFace::SolveIt
  (const SmObject *pObj1, 
   const SmObject *pObj2)
{
  SM_ASSERT(pObj1 != NULL && pObj2 != NULL);
  
  if (!m_bSwapOrder) 
    {
      SM_ASSERT(pObj1->IsKindOf(SmVertex_TYPE));
      SM_ASSERT(pObj2->IsKindOf(SmSurface_TYPE));
      m_pPointVertex = (SmVertex*)pObj1;
    }
  else 
    {
      SM_ASSERT(pObj1->IsKindOf(SmSurface_TYPE));
      SM_ASSERT(pObj2->IsKindOf(SmVertex_TYPE));
      m_pPointVertex = (SmVertex*)pObj2;
    }

  m_vPoint = m_pPointVertex->GetPoint();

  SER(SmTLSPointFace::SolveIt(pObj1,pObj2));
  return SM_SUCCESS;

} // end SmTLSVertexFace::SolveIt

/*******************************************************************//**
PURPOSE: Do local solve between Curve and vertex.

NOTES: add solutions to m_rTopologySolver.m_pSolutions 
***********************************************************************/
SmStatus SmTLSCurveVertex::SolveIt
  (const SmObject *pObj1,  // NotUsed: in : 
   const SmObject *pObj2)  // in : 
{
 SM_REF1(pObj1) ;
  // init PointCurve Solver
  SmTLSPointCurve sPCSolver(m_rTopologySolver, m_bSwapOrder, m_pShape);
  if (m_bSwapOrder) { sPCSolver.m_bSwapOrder = FALSE; }
  else              { sPCSolver.m_bSwapOrder = TRUE; }

  if (m_pCurveEdge) { sPCSolver.m_pCurveOwner = m_pCurveEdge;
                      sPCSolver.m_vInterval   = m_pCurveEdge->GetInterval();
                    }
  else              { sPCSolver.m_vInterval   = m_vInterval;
                    }

  sPCSolver.m_pPointVertex = (SmVertex*)pObj2;
  sPCSolver.m_vPoint       = sPCSolver.m_pPointVertex->GetPoint();

  // pass call along for Curve/Point solution - solutions added to m_rTopologySolver.m_pSolutions
  SER(sPCSolver.SolveIt(m_pCurve,m_pCurve));

  return SM_SUCCESS;

} // end SmTLSCurveVertex::SolveIt

/*******************************************************************//**
PURPOSE: Local solver for Edge Vertex.  Just invokes
   superclass after loading data.

NOTES: add solutions to m_rTopologySolver.m_pSolutions
***********************************************************************/
SmStatus SmTLSEdgeVertex::SolveIt
  (const SmObject *pObj1, 
   const SmObject *pObj2)
{
  SM_ASSERT(pObj1 != NULL && pObj2 != NULL);
  SmVertex *pVertex;
  if (!m_bSwapOrder) 
    {
      SM_ASSERT(pObj1->IsKindOf(SmCurve_TYPE));
      SM_ASSERT(pObj2->IsKindOf(SmVertex_TYPE));
      pVertex = (SmVertex*)pObj2;
    }
  else 
    {
      SM_ASSERT(pObj1->IsKindOf(SmVertex_TYPE));
      SM_ASSERT(pObj2->IsKindOf(SmCurve_TYPE));
      pVertex = (SmVertex*)pObj1;
    }

  const SmCurve *pCurve;
  if (!m_bSwapOrder) { pCurve = (SmCurve*)pObj2; }
  else               { pCurve = (SmCurve*)pObj1; }
  
// Remove Composites - replaced Edge array with single curve Edge
  SM_ASSERT_MSG(pCurve->GetEdge() != NULL, _T("SmTLSEdgeVertex::SolveIt - Remove Composite single Edge assumption wrong here - needs debug")) ;
  SmEdge * pEdge = (SmEdge *)pCurve->GetEdge() ; 

// Remove Composites
//   SmEdge *sEData[16];
//  SmTArray<SmEdge*> sEdges(16,sEData);
//  SmBrep::GetEdgesOfCurve(pCurve,sEdges);
//
//  for (ULONG i=0; i<sEdges.GetSize(); i++) 
//    {
//      SmEdge *pEdge = sEdges[i];
//
//      // skip edges not listed - these edge get included when part of a CEdge
//      if (m_pShape && !m_pShape->IsInShape(pEdge))
//        { continue; }

// Remove Composites - 1 check line brought down from block above
  // when using m_pShape (called frome ShapePointSolve(), ShapeCurveSolve(), or ShapeShapeSolve()) - skip pEdge not in shape
  if(m_pShape == NULL || m_pShape->IsInShape(pEdge))
    {
      m_pCurveEdge = pEdge;
      m_vInterval  = m_pCurveEdge->GetInterval();
      m_pCurve     = m_pCurveEdge->GetCurve();
      SER(SmTLSCurveVertex::SolveIt(pVertex,pVertex));
    }

  return SM_SUCCESS;

} // end SmTLSEdgeVertex::SolveIt

/*******************************************************************//**
PURPOSE: Do local solve between a Curve and the edge(s) of another Curve.

NOTES:  add solutions to m_rTopologySolver.m_pSolutions
***********************************************************************/
SmStatus SmTLSCurveEdge::SolveIt
  (const SmObject *pObj1, 
   const SmObject *pObj2)
{
  SmCurve *pCurve;

  if (!m_bSwapOrder) { pCurve = SM_CAST_PTR( SmCurve, pObj2 ); }
  else               { pCurve = SM_CAST_PTR( SmCurve, pObj1 ); }

  NER( pCurve );

// Remove Composites - replaced Edge array with single curve Edge
  SM_ASSERT_MSG(pCurve->GetEdge() != NULL, _T("SmTLSCurveEdge::SolveIt - Remove Composite single Edge assumption wrong here - needs debug")) ;
  SmEdge * pEdge = (SmEdge *)pCurve->GetEdge() ; 

// Remove Composites
//  // for every Edge of pCurve
//  SmEdge *sEData[16];
//  SmTArray<SmEdge*> sEdges(16,sEData);
//  SmBrep::GetEdgesOfCurve( pCurve, sEdges );
//  ULONG i;
//
//  for ( i=0; i<sEdges.GetSize(); i++ )
//    {
//      SmEdge *pEdge = sEdges[i];
//
//      // skip edges not listed - these edge get included when part of a CEdge
//      if ( m_pShape != NULL && !m_pShape->IsInShape(pEdge) )
//        { continue; }

// Remove Composites - 1 check line brought down from block above
  // when using m_pShape (called frome ShapePointSolve(), ShapeCurveSolve(), or ShapeShapeSolve()) - skip pEdge not in shape
  if(m_pShape == NULL || m_pShape->IsInShape(pEdge))
    {
      // locals
      m_pCurve2     = pEdge->GetCurve();
      m_vInterval2  = pEdge->GetInterval();
      m_pCurveEdge2 = pEdge;

      // Solve the Curve/Curve problem
      SER(SmTLSCurveCurve::SolveIt(pObj1,pObj2));
    
    } // end iter all cureve->edges

  return SM_SUCCESS;

} // end SmTLSCurveEdge::SolveIt

/*******************************************************************//**
PURPOSE: Do local solve between two Curves.

NOTES: add solutions to m_rTopologySolver.m_pSolutions
***********************************************************************/
SmStatus SmTLSCurveCurve::SolveIt
( const SmObject *,  // Inputs are in our data members.
 const SmObject * )
{
  SmSolution sSol;
  sSol.m_apObjects[1] = NULL;
  const SmCurve *pUVCurve1 = NULL;
  const SmCurve *pUVCurve2 = NULL;

  ULONG lObjectCount = 0;
  if(!m_bSwapOrder)
  {
    if(m_pCurveEdge)
    {
      sSol.m_apObjects[lObjectCount++] = SM_REINTERPRET_CAST( SmObject*,
          SM_CONST_CAST( SmEdge*, m_pCurveEdge ) );
    }
    else if(m_pCurveFace)
    {
      sSol.m_apObjects[lObjectCount++] = SM_REINTERPRET_CAST( SmObject*,
          SM_CONST_CAST( SmFace*, m_pCurveFace ) );
      pUVCurve1 = m_pSurfaceCurve;
    }
    else
    {
      sSol.m_apObjects[lObjectCount++] = SM_REINTERPRET_CAST( SmObject*,
          SM_CONST_CAST( SmCurve*, m_pCurve ) );
    }
    if(m_pCurveEdge2)
    {
      sSol.m_apObjects[lObjectCount++] = SM_REINTERPRET_CAST( SmObject*,
          SM_CONST_CAST( SmEdge*, m_pCurveEdge2 ) );
    }
    else if(m_pCurveFace2)
    {
      sSol.m_apObjects[lObjectCount++] = SM_REINTERPRET_CAST( SmObject*,
          SM_CONST_CAST( SmFace*, m_pCurveFace2 ) );
      pUVCurve2 = m_pSurfaceCurve2;
    }
    else
    {
      sSol.m_apObjects[lObjectCount++] = SM_REINTERPRET_CAST( SmObject*,
          SM_CONST_CAST( SmCurve*, m_pCurve2 ) );
    }
  }
  else
  {
    if(m_pCurveEdge2)
    {
      sSol.m_apObjects[lObjectCount++] = SM_REINTERPRET_CAST( SmObject*,
          SM_CONST_CAST( SmEdge*, m_pCurveEdge2 ) );
    }
    else if(m_pCurveFace2)
    {
      sSol.m_apObjects[lObjectCount++] = SM_REINTERPRET_CAST( SmObject*,
          SM_CONST_CAST( SmFace*, m_pCurveFace2 ) );
      pUVCurve1 = m_pSurfaceCurve2;
    }
    else
    {
      sSol.m_apObjects[lObjectCount++] = SM_REINTERPRET_CAST( SmObject*,
          SM_CONST_CAST( SmCurve*, m_pCurve2 ) );
    }
    if(m_pCurveEdge)
    {
      sSol.m_apObjects[lObjectCount++] = SM_REINTERPRET_CAST( SmObject*,
          SM_CONST_CAST( SmEdge*, m_pCurveEdge ) );
    }
    else if(m_pCurveFace)
    {
      sSol.m_apObjects[lObjectCount++] = SM_REINTERPRET_CAST( SmObject*,
          SM_CONST_CAST( SmFace*, m_pCurveFace ) );
      pUVCurve2 = m_pSurfaceCurve;
    }
    else
    {
      sSol.m_apObjects[lObjectCount++] = SM_REINTERPRET_CAST( SmObject*,
          SM_CONST_CAST( SmCurve*, m_pCurve ) );
    }
  }

#ifdef SM_DEBUG_CODE
  SmBoolean bDebugDump = FALSE;
  SmBoolean bDebugDraw = FALSE;
  if(bDebugDump)
  {
    m_pCurve->Dump();
    m_pCurve2->Dump();
  }
  if(bDebugDraw)
  {
    sm_GraphicsLoop();
    smgfx_Erase();
    smgfx_SetLook( 2, 4, 0, 0, 1 ); m_pCurve2->DrawWDeriv( m_vInterval2, 0 ); sm_GraphicsLoop();
    smgfx_SetLook( 2, 4, 1, 0, 0 ); m_pCurve->DrawWDeriv( m_vInterval, 0 );   sm_GraphicsLoop();
    sm_GraphicsLoop();
  }
#endif        

  double *pdBestAnswer = NULL;
  double dBestAnswer;
  m_rTopologySolver.GetBestAnswerSoFar( dBestAnswer, pdBestAnswer );
  double dSign = 1.0;
  if(m_bSwapOrder && m_rTopologySolver.m_eOperationCategory == SM_OPERATION_SIGNED)
  {
    // If we are doing a signed operation and the order is reversed then we have to
    // reverse the sign of the result.
    dSign = -1.0 * dSign;
  }
  if(pdBestAnswer)
  {
    *pdBestAnswer = *pdBestAnswer * dSign;
  }
  SmSolution aData[10];
  SmSolutionArray sSolutions( 10, aData );

  if(!m_bSwapOrder)
  {
    if(m_rTopologySolver.m_eSolverOperation == SM_SO_INTERSECT)
    {
      SER( m_pCurve->GlobalCurveIntersect( m_vInterval,
           *m_pCurve2, m_vInterval2,
           m_rTopologySolver.m_d3dTolerance,
           sSolutions ) );
    }
    else
    {
      SER( m_pCurve->GlobalCurveSolve( m_vInterval,
           *m_pCurve2, m_vInterval2,
           m_rTopologySolver.m_eSolverOperation,
           m_rTopologySolver.m_d3dTolerance,
           pdBestAnswer, m_rTopologySolver.m_cpOptVectors, SM_SR_ALL,
           sSolutions ) );
    }
  }
  else
  {
    if(m_rTopologySolver.m_eSolverOperation == SM_SO_INTERSECT)
    {
      SER( m_pCurve2->GlobalCurveIntersect( m_vInterval2,
           *m_pCurve, m_vInterval,
           m_rTopologySolver.m_d3dTolerance,
           sSolutions ) );
    }
    else
    {
      SER( m_pCurve2->GlobalCurveSolve( m_vInterval2,
           *m_pCurve, m_vInterval,
           m_rTopologySolver.m_eSolverOperation,
           m_rTopologySolver.m_d3dTolerance,
           pdBestAnswer, m_rTopologySolver.m_cpOptVectors, SM_SR_ALL,
           sSolutions ) );
    }
  }

  for(ULONG i = 0; i < sSolutions.GetSize(); i++)
  {
    SmSolution & rSol = sSolutions[i];
    double dT1 = rSol.m_vStart[0];
    double dT2 = rSol.m_vStart[1];
    rSol.m_apObjects[0] = sSol.m_apObjects[0];
    rSol.m_apObjects[1] = sSol.m_apObjects[1];
    rSol.m_lNumObjects = lObjectCount;
    rSol.m_lNumVariables = 0;
    if(pUVCurve1)
    {
      SmPoint3d sPnt;
      SER( pUVCurve1->EvaluatePoint( dT1, sPnt ) );
      rSol.m_vStart[rSol.m_lNumVariables++] = sPnt.x;
      rSol.m_vStart[rSol.m_lNumVariables++] = sPnt.y;
    }
    else
    {
      rSol.m_vStart[rSol.m_lNumVariables++] = dT1;
    }
    if(pUVCurve2)
    {
      SmPoint3d sPnt;
      SER( pUVCurve2->EvaluatePoint( dT2, sPnt ) );
      rSol.m_vStart[rSol.m_lNumVariables++] = sPnt.x;
      rSol.m_vStart[rSol.m_lNumVariables++] = sPnt.y;
    }
    else
    {
      rSol.m_vStart[rSol.m_lNumVariables++] = dT2;
    }
    rSol.m_vStart.m_dSolutionValue = dSign * rSol.m_vStart.m_dSolutionValue;
    if(rSol.m_eSolutionType == SM_ST_RANGE_OF_VALUES)
    {
      rSol.m_vEnd.m_dSolutionValue = dSign * rSol.m_vEnd.m_dSolutionValue;
    }
#ifdef SM_DEBUG_CODE
    if(bDebugDraw)
    {
      SmPoint3d sFndPnt1;
      rSol.GetPoint( 0, sFndPnt1 );

      SmPoint3d sFndPnt2;
      rSol.GetPoint( 1, sFndPnt2 );
      smgfx_SetLook( 2, 4, 1, 0, 0 ); sFndPnt1.Draw(); sm_GraphicsLoop();
      smgfx_SetLook( 2, 4, 1, 0, 0 ); sFndPnt2.Draw(); sm_GraphicsLoop();
      SmVector3d sDiff = sFndPnt1 - sFndPnt2;
      smgfx_SetLook( 2, 4, 0, 0, 0 ); sDiff.Draw( &sFndPnt2 ); sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
#endif        
    m_rTopologySolver.AddSortedSolution( rSol, SM_SK_BY_SOLUTION_VALUE );

  }

  return SM_SUCCESS;

} // end SmTLSCurveCurve::SolveIt

/*******************************************************************//**
PURPOSE: Local solver for Edge Edge.  Just invokes
   superclass after loading data.

NOTES:  add solutions to m_rTopologySolver.m_pSolutions
***********************************************************************/
SmStatus SmTLSEdgeEdge::SolveIt
  (const SmObject *pObj1, 
   const SmObject *pObj2)
{
  SM_ASSERT(pObj1 != NULL && pObj2 != NULL);
  SM_ASSERT(pObj1->IsKindOf(SmCurve_TYPE));
  SM_ASSERT(pObj2->IsKindOf(SmCurve_TYPE));

  const SmCurve *pCurve;
  if (!m_bSwapOrder) 
    {
      pCurve = (SmCurve*)pObj1;
    }
  else 
    {
      pCurve = (SmCurve*)pObj2;
    }

// Remove Composites - replaced Edge array with single curve Edge
  SM_ASSERT_MSG(pCurve->GetEdge() != NULL, _T("SmTLSEdgeEdge::SolveIt - Remove Composite single Edge assumption wrong here - needs debug")) ;
  SmEdge * pEdge = (SmEdge *)pCurve->GetEdge() ; 

// Remove Composites
//  SmEdge *sEData[16];
//  SmTArray<SmEdge*> sEdges(16,sEData);
//  SmBrep::GetEdgesOfCurve(pCurve,sEdges);
//
//  for (ULONG i=0; i<sEdges.GetSize(); i++) 
//    {
//      SmEdge *pEdge = sEdges[i];
//
//      // skip edges not listed - these edge get included when part of a CEdge
//      if (m_pShape && !m_pShape->IsInShape(pEdge))
//        { continue; }

// Remove Composites - 1 check line brought down from block above
  // when using m_pShape (called frome ShapePointSolve(), ShapeCurveSolve(), or ShapeShapeSolve()) - skip pEdge not in shape
  if(m_pShape == NULL || m_pShape->IsInShape(pEdge))

    {
      m_pCurveEdge = pEdge;
      m_vInterval  = m_pCurveEdge->GetInterval();
      m_pCurve     = m_pCurveEdge->GetCurve();

      SER(SmTLSCurveEdge::SolveIt(pObj1,pObj2));
    }

  return SM_SUCCESS;

} // end SmTLSEdgeEdge::SolveIt

/*******************************************************************//**
PURPOSE: Do local solve between Curve and a face. Place solutions
            into m_rTopologySolver.m_pSolutions.

NOTES: add solutions to m_rTopologySolver.m_pSolutions

IMPLEMENTATION --- 
  For Projection Type Operations
    1. Get face silhouette curves for projection plane normal stored in 
       m_rTopologySolver.m_cpOptVectors[0].
    2. Find and save Curve/SilhouetteCurve solutions.
  
  For nonProject Type Operations - 
  Save curve/Surface solutions that are within the face.
    1. get Curve/Surface solutions
       a. For SM_SO_INTERSECT, call SmSurface::GlobalCurveIntersect
       b. For all others,      call SmSurface::GlobalCurveSolve 
    2. For Every Curve/Surface solution
       a. For Intersection Operations cull solutions that don't classify to the face
       b. add sorted solutions to m_rTopologySolver.m_pSolutions

***********************************************************************/
SmStatus SmTLSCurveFace::SolveIt
  (const SmObject *pObj1,        // in : typically NotUsed (curve is found in 'this' member ptrs), is reversed when m_bSwapOrder == TRUE
   const SmObject *pObj2)        // in : typically the target Surface, is reversed when m_bSwapOrder == TRUE
{
  // locals
  SmSurface *pSurface      = NULL;
  SmSolution sSol;
  ULONG      lObjectCount  = 2;

  // load pSurface and sSol.m_apObjects from pObj1 and pObj2
  // Normally the Curve comes first and the Surface second.  
  // If we want we can switch the order.
  if (!m_bSwapOrder) 
    {
      pSurface = SM_REINTERPRET_CAST(SmSurface*, SM_CONST_CAST(SmObject*,pObj2)); 
      NER(pSurface);

      sSol.m_apObjects[0] = 
        (  m_pCurveEdge ? SM_REINTERPRET_CAST(SmObject*, SM_CONST_CAST(SmEdge*,m_pCurveEdge))
         : m_pCurveFace ? SM_REINTERPRET_CAST(SmObject*, SM_CONST_CAST(SmFace*,m_pCurveFace))
         :                SM_REINTERPRET_CAST(SmObject*, SM_CONST_CAST(SmCurve*,m_pCurve))) ;
      sSol.m_apObjects[1] = pSurface;
    }
  else 
    {
      pSurface = SM_REINTERPRET_CAST(SmSurface*,SM_CONST_CAST(SmObject*,pObj1)); 
      NER(pSurface);

      sSol.m_apObjects[0] = pSurface;
      sSol.m_apObjects[1] = 
        (  m_pCurveEdge ? SM_REINTERPRET_CAST(SmObject*, SM_CONST_CAST(SmEdge*,m_pCurveEdge))
         : m_pCurveFace ? SM_REINTERPRET_CAST(SmObject*, SM_CONST_CAST(SmFace*,m_pCurveFace))
         :                SM_REINTERPRET_CAST(SmObject*, SM_CONST_CAST(SmCurve*,m_pCurve))) ;
    } // end loading pSurface and sSol.m_apObjects

// Remove Composites  
//  // get pSurface faces (may be more than 1 for composites)
//  SmFace *sFData[16];
//  SmTArray<SmFace*> sFaces(16,sFData);
//
//  SmBrep::GetFacesOfSurface(pSurface,sFaces);
//  
//  // for every face
//  for (ULONG ii=0; ii<sFaces.GetSize(); ii++) 
//    {
//      SmFace    * pFace             = sFaces[ii];
//      // SmZoneTol3d sFaceZoneTol3d = SmTol::GetZoneTol3d(pFace) ;
//
//      // skip faces not listed - these faces get included when part of a CFace
//      if (m_pShape && !m_pShape->IsInShape(pFace))
//        { continue; }
//
//      // check state - m_bEditingEnabled must be off
//      //      SM_ASSERT(pFace->GetBrep() && pFace->GetBrep()->m_bEditingEnabled == FALSE);
//      // gwc: remove check
//      //      if (!pFace->GetBrep() || pFace->GetBrep()->m_bEditingEnabled) 
//      //        { SER_MSG(SM_ERR, _T("Tried to edit SmTLSCurveFace::SolveIt with m_bEditingEnabled == TRUE")); }

// Remove Composites - replaced face array with single surface face
  SM_ASSERT_MSG(pSurface->GetFace() != NULL, _T("SmTLSCurveFace::SolveIt - Remove Composite single face assumption wrong here - needs debug")) ;
  SmFace * pFace = pSurface->GetFace() ; 

// Remove Composites - 1 check line brought down from block above
  // when using m_pShape (called frome ShapePointSolve(), ShapeCurveSolve(), or ShapeShapeSolve()) - skip pFace not in shape
  if(m_pShape == NULL || m_pShape->IsInShape(pFace))
    {
        
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
static ULONG lCount      = 1 ; lCount++ ;
static ULONG lDebugCount = 0 ;
      if (bDebugMe || lCount == lDebugCount) 
        {
          sm_GraphicsLoop();
          smgfx_Erase();
          smgfx_SetLook(5,5, 1,0,0); m_pCurve->DrawWDeriv(m_vInterval,0); sm_GraphicsLoop();
          smgfx_SetLook(2,2, 0,0,0); pFace->Draw();                          sm_GraphicsLoop();
          sm_GraphicsLoop();
        }
#endif
      // when operation is one of the min/max 'projection' operations
      if (m_rTopologySolver.m_bProjectedOperation == TRUE) 
        {
          // temporary Curve arrays
          SmTArray<SmCurve*> sTrim3DCurves, sTrimUVCurves;
          SmObjsDelete<SmCurve*> sClean1(&sTrim3DCurves);
          SmObjsDelete<SmCurve*> sClean2(&sTrimUVCurves);

          // get or construct silhouette curves for face and projection plane normal
          SER(sm_GetFaceSilhouettes(pFace,
                                      m_rTopologySolver.m_cpOptVectors,  // projection plane normal
                                      m_rTopologySolver.m_d3dTolerance,
                                      sTrim3DCurves, sTrimUVCurves));

          // When we reach here there should be silhouette curves in sTrim3DCurves
          // We can use these to do measurement to the surface.  Please note that
          // the edges bounding the face would have already been measured.

          // for every silhouette curve
          for (ULONG i=0; i<sTrim3DCurves.GetSize(); i++) 
            {
              // construct a curve/curve solver
              SmTLSCurveCurve sTSLSolver(m_rTopologySolver, m_bSwapOrder, m_pShape);
              sTSLSolver.m_pCurve         = m_pCurve;
              sTSLSolver.m_pCurveEdge     = m_pCurveEdge;
              sTSLSolver.m_pCurveFace     = m_pCurveFace;
              sTSLSolver.m_vInterval      = m_vInterval;
              sTSLSolver.m_pSurfaceCurve  = m_pSurfaceCurve;
                                          
              SmCurve *pCurve             = sTrim3DCurves[i];
              sTSLSolver.m_pCurve2        = pCurve;
              sTSLSolver.m_pCurveEdge2    = NULL;
              sTSLSolver.m_vInterval2     = pCurve->GetNaturalInterval();
              sTSLSolver.m_pCurveFace2    = pFace;
              sTSLSolver.m_pSurfaceCurve2 = sTrimUVCurves[i];

              // solve it
              SER(sTSLSolver.SolveIt(pCurve,pCurve));
            } // end iter every silhouette curve

        } // end projection type operation branch
      else 
        { // not a projection type operation

          // locals
          double *pdBestAnswer, dBestAnswer = 0.0;
          m_rTopologySolver.GetBestAnswerSoFar(dBestAnswer, pdBestAnswer);

          // No: We reversed the direction vector, which takes care of it.
          // We do not want to reverse the function value.  [B650]
          //
          //// If we are doing a signed operation and the order is reversed then we have to
          //// reverse the sign of the result.
          //double dSign =  (   m_bSwapOrder 
          //                 && m_rTopologySolver.m_eOperationCategory == SM_OPERATION_SIGNED)
          //               ? -1.0
          //               :  1.0 ;
          //dBestAnswer = dBestAnswer * dSign;

          // construct solution array
          SmSolution aData[10];
          SmSolutionArray sSolutions(10,aData);
          
          // get Curve/Surface solutions

          // for intersection operations
          if (m_rTopologySolver.m_eSolverOperation == SM_SO_INTERSECT) 
            {
              // pass call to global surface/curve intersector
              SER(pFace->GetSurface()->GlobalCurveIntersect(pFace->GetUVDomain(),
                     *m_pCurve,m_vInterval,
                     m_rTopologySolver.m_d3dTolerance + pFace->GetTolerance(),
                     sSolutions));
            }
          else // for all nonIntersection operations
            {
              const SmVector3d * pVecs = m_rTopologySolver.m_cpOptVectors;
              SmVector3d sVecs[4];

              // init sVecs
              if (m_rTopologySolver.m_eSolverOperation == SM_SO_3D_SIGNED_DIRECTED_MINIMIZE) 
                {
                  if (m_bSwapOrder) 
                    {
                      sVecs[0] = - m_rTopologySolver.m_cpOptVectors[0];
                      pVecs = & sVecs[0];
                    }
                }

              // pass call to global surface/curve solver
              SER(pFace->GetSurface()->GlobalCurveSolve(pFace->GetUVDomain(),
                  *m_pCurve,m_vInterval,
                  m_rTopologySolver.m_eSolverOperation, 
                  m_rTopologySolver.m_d3dTolerance + pFace->GetTolerance(),
                  pdBestAnswer, pVecs, SM_SR_ALL,
                  sSolutions));
          } // end nonIntersection operations branch
          
          // for every solution - 
          //   1. cull any intersection ranges whose ends don't classify to the Face
          //   2. refine intersection range endPoint UV parameter solution values
          //          with a call to pSurface->GlobalPointSolve()
          //   3. set solution object values
          //   4. Add solution to m_rTopologySolver.m_pSolutions 
          for (ULONG i=0; i<sSolutions.GetSize(); i++) 
            {
              // set rSol from sSolutions ith member
              SmSolution & rSol = sSolutions[i];

              // for intersection operations
              if (m_rTopologySolver.m_eSolverOperation == SM_SO_INTERSECT) 
                {
                  // for range of values type solutions
                  if (   rSol.m_eSolutionType == SM_ST_RANGE_OF_VALUES 
                      && !SM_ARE_SAME(rSol.m_vStart[0],rSol.m_vEnd[0]) ) 
                    {
                      SmCurveInterval sCIData[10];

                      // get solution interval
                      SmExtent1d sIvl(rSol.m_vStart[0],rSol.m_vEnd[0]);

                      // construct curveClassification object over solution interval
                      SmCurveClassification sCurveClass(SM_CONST_CAST(SmCurve*,m_pCurve),
                                                        sIvl,
                                                        NULL,
                                                        m_rTopologySolver.m_d3dTolerance,
                                                        10,
                                                        sCIData);

                      // classify the solution interval against the face - currently the interval is just classified against the surface
                      SER(pFace->CurveOnClassify(TRUE,sCurveClass));

#ifdef SM_DEBUG_CODE
                      if (bDebugMe) 
                        {
                          smgfx_Erase();
                          smgfx_SetLook(4,4, 1,0,0); m_pCurve->DrawWDeriv(m_vInterval,0); sm_GraphicsLoop();
                          smgfx_SetLook(2,2, 0,0,0); pFace->Draw(); sm_GraphicsLoop();
                          sm_GraphicsLoop();
                          
                          sCurveClass.Dump();
                        }
#endif
                      // for every curve classification - a single surface/curve solution may end up as several face/curve solutions
                      for (ULONG kk=0; kk<sCurveClass.GetSize(); kk++) 
                        {
                          SmCurveInterval & rIvl = sCurveClass[kk];

                          // when interval is in the face
                          if (    rIvl.m_vMid.GetPointClass() == SM_PC_FACE 
                              ||  rIvl.m_vMid.GetPointClass() == SM_PC_EDGE )
                            {
                              // load interval data into rSol solution object
                              rSol.m_vStart[0] = rIvl.m_vInterval.GetMin();
                              rSol.m_vEnd[0]   = rIvl.m_vInterval.GetMax();

                              // One condition this may not work on is the case
                              // where the curve is split by some sort of interior
                              // edge or vertex.  I think that AddSortedSolution
                              // may just try to glue the segments back together.

                              // Set rSol surface parameters to correspond to curve endPoints
                              //  prior to inserting into the solutions array. 
                              SmPoint3d sStart, sEnd;
                              SER(m_pCurve->EvaluatePoint(rIvl.m_vInterval.GetMin(),sStart));
                              SER(m_pCurve->EvaluatePoint(rIvl.m_vInterval.GetMax(),sEnd));

                              // locals for GlobalPointSolve
                              SmSolution sSData[16];
                              SmSolutionArray sTmpSolutions(16,sSData);
                              SmSurfaceCache *pSC = smsurf_GetSurfaceCache(pSurface);  NER(pSC);
                              SmCacheCheckOutIn sCheckIO(pSC);
                                {
                                  // Turn off point testing so GlobalPointSolve() will keep all point solutions
                                  //   without classifying the solution point against the trim boundaries.
                                  SmTemporaryChangeValue<SmBoolean> sTCV(pSC->m_bPointTestEnabled,FALSE);
                                  SER(pSurface->GlobalPointSolve(pFace->GetUVDomain(),SM_SO_INTERSECT,
                                      sStart,2.0*(m_rTopologySolver.m_d3dTolerance + pFace->GetTolerance()),
                                      NULL,SM_SR_ALL,sTmpSolutions));
                                  if (sTmpSolutions.GetSize() == 0) // error - something is wrong here - quit
                                    { continue; }

                                  SmSolution & rTmpSol = sTmpSolutions[0];
                                  rSol.m_vStart[1]     = rTmpSol.m_vStart[0];
                                  rSol.m_vStart[2]     = rTmpSol.m_vStart[1];
                                  
                                  SER(pSurface->GlobalPointSolve(pFace->GetUVDomain(),SM_SO_INTERSECT,
                                      sEnd,2.0*(m_rTopologySolver.m_d3dTolerance + pFace->GetTolerance()),
                                      NULL,SM_SR_ALL,sTmpSolutions));
                                  if (sTmpSolutions.GetSize() == 0) // error - something is wrong here - quit
                                    { continue; }

                                  SmSolution & rTmpSol2 = sTmpSolutions[0];
                                  rSol.m_vEnd[1]      = rTmpSol2.m_vStart[0];
                                  rSol.m_vEnd[2]      = rTmpSol2.m_vStart[1];
                                }

                              // set rSol Objects - replace surface object with face object
                              rSol.m_apObjects[0]  = (sSol.m_apObjects[0] == pSurface) ? pFace : sSol.m_apObjects[0] ;
                              rSol.m_apObjects[1]  = (sSol.m_apObjects[1] == pSurface) ? pFace : sSol.m_apObjects[1] ;

                              rSol.m_lNumObjects   = lObjectCount;
                              rSol.m_lNumVariables = 3;

                              // when swapping
                              if (m_bSwapOrder) 
                                {
                                  // Need to put the surface parameters first - default is
                                  // for GlobalCurveSolve to put curve parameters first.
                                  double dTmp      = rSol.m_vStart[0];
                                  rSol.m_vStart[0] = rSol.m_vStart[1];
                                  rSol.m_vStart[1] = rSol.m_vStart[2];
                                  rSol.m_vStart[2] = dTmp;
                                  if (rSol.m_eSolutionType == SM_ST_RANGE_OF_VALUES) 
                                    {
                                      dTmp    = rSol.m_vEnd[0];
                                      rSol.m_vEnd[0] = rSol.m_vEnd[1];
                                      rSol.m_vEnd[1] = rSol.m_vEnd[2];
                                      rSol.m_vEnd[2] = dTmp;
                                    }
                                } // end when swapping check

                              // add rSol to output solution array
                              m_rTopologySolver.AddSortedSolution(rSol,SM_SK_BY_SOLUTION_VALUE,FALSE);
                            
                            } // end interval classifies to face check
                        } // end iter every curveClassification interval

                      // go to next solution
                      continue;

                    } // end range of values type solution check

                  // Classify surface/curve PointSolution against Face
                  SmPoint2d sUV(rSol.m_vStart[1],rSol.m_vStart[2]);
                  SmZoneTol3d sSrcZoneTol3d =   m_pCurveEdge ? SmTol::GetZoneTol3d(m_pCurveEdge)
                                              : m_pCurveFace ? SmTol::GetZoneTol3d(m_pCurveFace)
                                              :                SmTol::GetZoneTol3d(m_pCurve) ; // if m_pPointVertex is allowed to be NULL
                  SmPointClassification sPntClass(sSrcZoneTol3d, pFace->GetContext()); // this tol is unlikely to be correct - check it
                  SER(pFace->PointClassify(sUV,sSrcZoneTol3d,FALSE,FALSE,sPntClass));

                  // skip points that don't classify to the face
                  if (sPntClass.GetPointClass() == SM_PC_UNKNOWN) 
                    { // go to next solution
                      continue;
                    }
                } // end intersection operation branches

              // get here for all nonIntersection solutions and
              // for point Intersection solutions that classify to the face.
              //   (range intersection solutions are already handled)

              // currently rSol = ith solution
              //           sSol = local solve object whos m_apObjects are set
               
              // load rSol m_apObjects solution data from sSol m_apObjects data
              //   replace pSurface Object with pFace Object
              rSol.m_apObjects[0]  = (sSol.m_apObjects[0] == pSurface) ? pFace : sSol.m_apObjects[0];
              rSol.m_apObjects[1]  = (sSol.m_apObjects[1] == pSurface) ? pFace : sSol.m_apObjects[1];
              rSol.m_lNumObjects   = lObjectCount;
              rSol.m_lNumVariables = 3;

              // when swapping
              if (m_bSwapOrder) 
                {
                  // Need to put the surface parameters first - default is
                  // for GlobalCurveSolve to put curve parameters first.
                  double dTmp      = rSol.m_vStart[0];
                  rSol.m_vStart[0] = rSol.m_vStart[1];
                  rSol.m_vStart[1] = rSol.m_vStart[2];
                  rSol.m_vStart[2] = dTmp;
                  if (rSol.m_eSolutionType == SM_ST_RANGE_OF_VALUES) 
                    {
                      dTmp    = rSol.m_vEnd[0];
                      rSol.m_vEnd[0] = rSol.m_vEnd[1];
                      rSol.m_vEnd[1] = rSol.m_vEnd[2];
                      rSol.m_vEnd[2] = dTmp;
                    }
                }

              // No: see above.  [B650]
              //// set solution sign values for swapped SM_OPERATION_SIGNED operations
              //rSol.m_vStart.m_dSolutionValue = dSign * rSol.m_vStart.m_dSolutionValue;
              //if (rSol.m_eSolutionType  == SM_ST_RANGE_OF_VALUES) 
              //  { rSol.m_vEnd.m_dSolutionValue = dSign * rSol.m_vEnd.m_dSolutionValue; }

              // add rSol to m_rTopologySolver.m_pSolutions array
              // GWC: I need to study the new optional argument value
              //       to see if it should be set to TRUE or FALSE - for now
              //       we have the previous intersection case where solutions are
              //       classified to the face properly set to FALSE.  This branch
              //       may need to combine solution segments in which case
              //       the new value should be TRUE - for now run with previous
              //       behavior, default == TRUE.
              m_rTopologySolver.AddSortedSolution(rSol,SM_SK_BY_SOLUTION_VALUE);

            } // end iter every solution
        } // end operation not a projection type branch

    } // end iter every face

  return SM_SUCCESS;

} // end SmTLSCurveFace::SolveIt

/*******************************************************************//**
PURPOSE: Local solver for Edge Face.  Just invokes
   superclass after loading data.

NOTES: add solutions to m_rTopologySolver.m_pSolutions
***********************************************************************/
SmStatus SmTLSEdgeFace::SolveIt
  (const SmObject *pObj1, 
   const SmObject *pObj2)
{
  SM_ASSERT(pObj1 != NULL && pObj2 != NULL);
  SmCurve *pCurve;
  if (!m_bSwapOrder) 
    {
      SM_ASSERT(pObj1->IsKindOf(SmCurve_TYPE));
      SM_ASSERT(pObj2->IsKindOf(SmSurface_TYPE));
      pCurve = (SmCurve*)pObj1;
    }
  else 
    {
      SM_ASSERT(pObj1->IsKindOf(SmSurface_TYPE));
      SM_ASSERT(pObj2->IsKindOf(SmCurve_TYPE));
      pCurve = (SmCurve*)pObj2;
    }

// Remove Composites - replaced Edge array with single curve Edge
  SM_ASSERT_MSG(pCurve->GetEdge() != NULL, _T("SmTLSEdgeFace::SolveIt - Remove Composite single Edge assumption wrong here - needs debug")) ;
  SmEdge * pEdge = (SmEdge *)pCurve->GetEdge() ; 

// Remove Composites
//  SmEdge *sEData[16];
//  SmTArray<SmEdge*> sEdges(16,sEData);
//  SmBrep::GetEdgesOfCurve(pCurve,sEdges);
//
//  for (ULONG i=0; i<sEdges.GetSize(); i++) 
//    {
//      SmEdge *pEdge = sEdges[i];
//
//      // skip edges not listed - these edge get included when part of a CEdge
//      if (m_pShape && !m_pShape->IsInShape(pEdge))
//        { continue; }
    
// Remove Composites - 1 check line brought down from block above
  // when using m_pShape (called frome ShapePointSolve(), ShapeCurveSolve(), or ShapeShapeSolve()) - skip pEdge not in shape
  if(m_pShape == NULL || m_pShape->IsInShape(pEdge))
    {
      m_pCurveEdge = pEdge;
      m_vInterval  = m_pCurveEdge->GetInterval();
      m_pCurve     = m_pCurveEdge->GetCurve();

      SER(SmTLSCurveFace::SolveIt(pObj1,pObj2));
    }
  return SM_SUCCESS;

} // end SmTLSEdgeFace::SolveIt

/*******************************************************************//**
PURPOSE: Local solver for Face Face.  Just invokes
   superclass after loading data.

NOTES: add solutions to m_rTopologySolver.m_pSolutions
***********************************************************************/
SmStatus SmTLSFaceFace::SolveIt
  (const SmObject *pObj1, 
   const SmObject *pObj2)
{
  SM_ASSERT(pObj1 != NULL && pObj2 != NULL);
  SM_ASSERT(pObj1->IsKindOf(SmSurface_TYPE));
  SM_ASSERT(pObj2->IsKindOf(SmSurface_TYPE));
  SM_ASSERT(m_bSwapOrder == FALSE);

  SmSurface *pSurface1 = (SmSurface*)pObj1;
  SmSurface *pSurface2 = (SmSurface*)pObj2;

// Remove Composites - replaced face array with single surface face
  SM_ASSERT_MSG(pSurface1->GetFace() != NULL, _T("SmTLSFaceFace::SolveIt - - Remove Composite single face assumption wrong here - needs debug")) ;
  SmFace * pFace1 = pSurface1->GetFace() ; 

// Remove Composites - replaced face array with single surface face
  SM_ASSERT_MSG(pSurface2->GetFace() != NULL, _T("SmTLSFaceFace::SolveIt - - Remove Composite single face assumption wrong here - needs debug")) ;
  SmFace * pFace2 = pSurface2->GetFace() ; 

// Remove Composites
//  SmFace *sFData1[16];
//  SmTArray<SmFace*> sFaces1(16,sFData1);
//  SmBrep::GetFacesOfSurface(pSurface1,sFaces1);
//
//  SmFace *sFData2[16];
//  SmTArray<SmFace*> sFaces2(16,sFData2);
//  SmBrep::GetFacesOfSurface(pSurface2,sFaces2);

// Remove Composites
//   for (ULONG ii=0; ii<sFaces1.GetSize(); ii++) 
//    {
//      SmFace *pFace1 = sFaces1[ii];
//      // skip faces not listed - these faces get included when part of a CFace
//      if (m_pShape && !m_pShape->IsInShape(pFace1))
//        { continue; }
//
//      for (ULONG jj=0; jj<sFaces2.GetSize(); jj++) 
//        {
//          SmFace *pFace2 = sFaces2[jj];
//         
//          // skip faces not listed - these faces get included when part of a CFace
//          if (m_pShape && !m_pShape->IsInShape(pFace2))
//            { continue; }

// Remove Composites - 1 check line brought down from block above
  // when using m_pShape (called frome ShapePointSolve(), ShapeCurveSolve(), or ShapeShapeSolve()) - skip pFace1 not in shape
  if(m_pShape == NULL || m_pShape->IsInShape(pFace1))
    {
// Remove Composites - 1 check line brought down from block above
      // when using m_pShape (called frome ShapePointSolve(), ShapeCurveSolve(), or ShapeShapeSolve()) - skip pFace2 not in shape
      if(m_pShape == NULL || m_pShape->IsInShape(pFace2))
        {

          // check state - m_bEditingEnabled must be off
          //      SM_ASSERT(pFace1->GetBrep()->m_bEditingEnabled == FALSE) ;
          //      SM_ASSERT(pFace2->GetBrep()->m_bEditingEnabled == FALSE) ;
          // gwc: remove check
          //      if (pFace1->GetBrep()->m_bEditingEnabled) 
          //        { SER_MSG(SM_ERR, _T("Tried to edit SmTLSFaceFace::SolveIt with m_bEditingEnabled == TRUE")); }
          //      if (pFace2->GetBrep()->m_bEditingEnabled) 
          //        { SER_MSG(SM_ERR, _T("Tried to edit SmTLSFaceFace::SolveIt with m_bEditingEnabled == TRUE")); }

          double *pdBestAnswer = NULL;
          double dBestAnswer;
          m_rTopologySolver.GetBestAnswerSoFar(dBestAnswer, pdBestAnswer);

          // No: We reversed the direction vector, which takes care of it.
          // We do not want to reverse the function value.  [B650]
          //
          //double dSign = 1.0;
          //if (m_bSwapOrder && m_rTopologySolver.m_eOperationCategory == SM_OPERATION_SIGNED) 
          //  {
          //    // If we are doing a signed operation and the order is reversed then we have to
          //    // reverse the sign of the result.
          //    dSign = -1.0;
          //  }
          //if (pdBestAnswer) 
          //  {
          //    *pdBestAnswer = *pdBestAnswer * dSign;
          //  }
          
          if (m_rTopologySolver.m_bProjectedOperation == TRUE) 
            {
              SmTArray<SmCurve*> sTrim3DCurves, sTrimUVCurves;
              SmObjsDelete<SmCurve*> sClean1(&sTrim3DCurves);
              SmObjsDelete<SmCurve*> sClean2(&sTrimUVCurves);
              SER(sm_GetFaceSilhouettes(pFace1,m_rTopologySolver.m_cpOptVectors,
                  m_rTopologySolver.m_d3dTolerance,
                  sTrim3DCurves, sTrimUVCurves));
              // When we reach here there should be silhouette curves in sTrim3DCurves
              // We can use these to do measurement to the surface.  Please note that
              // the edges bounding the face would have already been measured.
              for (ULONG i=0; i<sTrim3DCurves.GetSize(); i++) 
                {
                  SmCurve *pCurve = sTrim3DCurves[i];
                  SmTLSCurveFace sTSLSolver(m_rTopologySolver, m_bSwapOrder, m_pShape);
                  sTSLSolver.m_pCurve = pCurve;
                  sTSLSolver.m_pCurveFace = pFace1;
                  sTSLSolver.m_pCurveEdge = NULL;
                  sTSLSolver.m_vInterval = pCurve->GetNaturalInterval();
                  sTSLSolver.m_pSurfaceCurve = sTrimUVCurves[i];
                  SER(sTSLSolver.SolveIt(pSurface2,pSurface2));
                }
            }
          else 
            {
              SmSolution aData[10];
              SmSolutionArray sSolutions(10,aData);
              
              SER(pSurface1->GlobalSurfaceSolve(pFace1->GetUVDomain(),
                  *pSurface2,pFace2->GetUVDomain(),
                  m_rTopologySolver.m_eSolverOperation, 
                  m_rTopologySolver.m_d3dTolerance,
                  pdBestAnswer, m_rTopologySolver.m_cpOptVectors,
                  m_rTopologySolver.m_eSolutionRequested,
                  sSolutions));
              
              for (ULONG i=0; i<sSolutions.GetSize(); i++) 
                {
                  SmSolution & rSol = sSolutions[i];
                  rSol.m_apObjects[0] = SM_CONST_CAST(SmFace*,pFace1);
                  rSol.m_apObjects[1] = SM_CONST_CAST(SmFace*,pFace2);
                  rSol.m_lNumObjects = 2;
                  rSol.m_lNumVariables = 4;

                  // No: see above.  [B650]
                  //rSol.m_vStart.m_dSolutionValue = dSign * rSol.m_vStart.m_dSolutionValue;
                  //if (rSol.m_eSolutionType  == SM_ST_RANGE_OF_VALUES) 
                  //  { rSol.m_vEnd.m_dSolutionValue = dSign * rSol.m_vEnd.m_dSolutionValue; }

                  // gwc: I don't think these solutions have been classified to the face
                  //      So for now I'll eave the new third arugment to the
                  //      AddSortedSolution call using its old behavior which
                  //      defaults to TRUE.  If a deeper study of this case shows
                  //      that the solutions have been classified to the face
                  //      boundaries then we do not want contiguous solutions joined
                  //      and the third argument should be set to FALSE.
                  m_rTopologySolver.AddSortedSolution(rSol,SM_SK_BY_SOLUTION_VALUE);
                }
            }

        } // For Faces2
    } // For Faces1
  return SM_SUCCESS;

} // end SmTLSFaceFace::SolveIt

/*******************************************************************//**
PURPOSE: This method solves a particular operation for a Brep and a
    point.  Valid operations include - SM_SO_MINIMIZE, SM_SO_MAXIMIZE,
    SM_SO_INTERSECT, SM_SO_NORMALIZE, SM_SO_PROJECTED_MINIMIZE,
    SM_SO_PROJECTED_MAXIMIZE, SM_SO_RAYFIRE. SM_SO_3D_SIGNED_DIRECTED_MINIMIZE. 
    Note that when doing minimization and maximization a very large 
    (SM_BIG_DOUBLE) or a very small (0.0) value should be set to 
    dBestAnswerSoFarSq unless you already have a limiting value.  
    In other words, if you wish to find a minimum only if it is less
    than 0.1 then you should set dBestAnswerSoFarSq = 0.1 * 0.1.
    The solutions array may or may not have existing solutions 
    already registered with it.

NOTES: 
  If the requested solution type SmSolutionRequestedType is passed in
  as SM_SR_ALL, then all solutions within tolerance will be returned.
  They will be sorted in order of the quality of the solution -- e.g.,
  for MINIMIZE, they will be sorted in increasing order of the solution
  value, which for MINIMIZE is the 3d distance between the test point
  and the closest point on the topology.

  In this case, if the test point is closest to a vertex, for example,
  the vertex will be returned, along with each edge and face that contains
  the vertex.  The solutions will be ordered according to solution value,
  but in this case it's probably not meaningful, as those values will vary
  only by bit noise.

  If the requested solution type SmSolutionRequestedType is passed in
  as SM_SR_SINGLE, then the first solution within d3dTolerance is returned.
  This method tests the Brep topology in order of increasing dimensionality:
  vertices first, then edges, and faces last.  (Except that for Normalize,
  vertices and edges are not tested.)  Therefore, if any vertex is within
  tolerance of the test point, it will be returned without checking edges
  or faces.  (Note, the closest vertex will be returned, even if more than
  one is within tolerance -- they all get tested.)  Note that if a vertex
  is returned, then all edges and faces that contain the vertex will also
  be equally good solutions.  [080731, 090813]

***********************************************************************/     
SmStatus SmTopologySolver::BrepPointSolve                                               
  (const SmBrep          * cpBrep,             // in : Brep to query
   const SmPoint3d       & crPoint,            // in : Query point
   SmSolverOperationType   eSolverOperation,   // oneof: SM_SO_INTERSECT, SM_SO_NORMALIZE, SM_SO_RAYFIRE,
                                               //        SM_SO_MINIMIZE, SM_SO_PROJECTED_MINIMIZE, SM_SO_DIRECTED_MINIMIZE,
                                               //        SM_SO_MAXIMIZE, SM_SO_PROJECTED_MAXIMIZE, SM_SO_DIRECTED_MAXIMIZE,
                                               //        SM_SO_SIGNED_DIRECTED_MINIMIZE, SM_SO_3D_SIGNED_DIRECTED_MINIMIZE.
   SmSolutionRequestedType eSolutionRequested, // oneof: SM_SR_SINGLE = Only produce one solution - the best
                                               //        SM_SR_ALL    = Find all function satisfying solutions to given tol
   double                  d3dTolerance,       // in : passed along to SmTopologySolver, max valid sol size                      
   double                  dBestAnswerSoFarSq, // in : passed along to SmTopologySolver,                     
   const SmVector3d      * cpOptVectors,       // in : used for some eSolverOperations - see SmSolverOperationType for more
   SmSolutionArray       & rSolutions)         // out: One SmSolution object per problem solution
{
  // check input
  if (   eSolverOperation != SM_SO_MINIMIZE 
      && eSolverOperation != SM_SO_MAXIMIZE 
      && eSolverOperation != SM_SO_RAYFIRE 
      && eSolverOperation != SM_SO_PROJECTED_MINIMIZE 
      && eSolverOperation != SM_SO_PROJECTED_MAXIMIZE 
      && eSolverOperation != SM_SO_SIGNED_DIRECTED_MINIMIZE 
      && eSolverOperation != SM_SO_3D_SIGNED_DIRECTED_MINIMIZE 
      && eSolverOperation != SM_SO_DIRECTED_MINIMIZE 
      && eSolverOperation != SM_SO_DIRECTED_MAXIMIZE 
      && eSolverOperation != SM_SO_INTERSECT 
      && eSolverOperation != SM_SO_NORMALIZE) 
    {
      SER(SM_ERR_INVALID_INPUT);
    }

  NER(cpBrep);

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
  if (bDebugMe) 
    { // draw input: Brep(black), Point(red)
      smgfx_Erase();
      smgfx_SetLook(3,5, 1,0,0); crPoint.Draw(); sm_GraphicsLoop();
      smgfx_SetLook(1,1, 0,0,0); cpBrep->Draw(); sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

  // check state - m_bEditingEnabled must be off
  //      SM_ASSERT(cpBrep->m_bEditingEnabled == FALSE) ;
  // gwc: remove check
  //      if (cpBrep->m_bEditingEnabled) 
  //        { SER_MSG(SM_ERR, _T("Tried to edit SmTopologySolver::BrepPointSolve with m_bEditingEnabled == TRUE")); }

  // init output
  rSolutions.ReSet();

  // locals
  SmTopologySolver sTS( eSolverOperation, eSolutionRequested, d3dTolerance,
                        dBestAnswerSoFarSq, cpOptVectors, rSolutions );
  SmBrepCache    * pBC = (SmBrepCache*)SmCacheMgr::GetOrCreateObjectCache( SM_OC_BREP, cpBrep ); 
  NER(pBC);
  SmTreeNode       sTreeNode;
  SmObjectList     sObjectList;
  SmTree           sPointTree( crPoint, 0, &sTreeNode, &sObjectList );

  // When request != Normalize operation, look for vertex and edge solutions
  if ( eSolverOperation != SM_SO_NORMALIZE )
    {
      // First compare the point with the vertices of the Brep.
      SmTree * pVertTree = pBC->GetVertexTree() ;
      SmTLSPointVertex sTSLPV(sTS, FALSE, NULL) ;
      sTSLPV.SetPoint(crPoint) ;
      SER(sTS.SolveTrees(sPointTree,*pVertTree,sTSLPV)) ;

      // exit condition - requested single solution, vtx solutions found, and sol good enough - return (first solution is rSolutions[0])
      if(   eSolutionRequested == SM_SR_SINGLE 
         && rSolutions.GetSize() > 0 
         && rSolutions[0].m_vStart.m_dSolutionValue <= d3dTolerance )
        { return SM_SUCCESS; }

      // Now compare the point with the edges of the Brep.
      SmTree *pCurveTree = pBC->GetCurveTree();
      SmTLSPointEdge sTLSPE(sTS, FALSE, NULL);
      sTLSPE.SetPoint(crPoint);
      SER(sTS.SolveTrees(sPointTree,*pCurveTree,sTLSPE));

      // exit condition - requested single solution, edge solutions found, and sol good enough - return (first solution is rSolutions[0])
      if(   eSolutionRequested == SM_SR_SINGLE 
         && rSolutions.GetSize() > 0  
         && rSolutions[0].m_vStart.m_dSolutionValue <= d3dTolerance )
        { return SM_SUCCESS; }

    } // end normalize operation check

  // arrive here when NonNormalize operations have no vertex or edge solutions and for all normalize operations

  // Now compare the point with the faces of the Brep.
  SmTree       * pSurfaceTree = pBC->GetSurfaceTree();
  SmTLSPointFace sTLSPF(sTS, FALSE, NULL);
  sTLSPF.SetPoint(crPoint);

  // Pass the call along to the SmTopologySolver
  SER(sTS.SolveTrees(sPointTree,*pSurfaceTree,sTLSPF));

  // all done
  return SM_SUCCESS;

} // end SmTopologySolver::BrepPointSolve

/*******************************************************************//**
PURPOSE: This method solves a particular operation for a Shape and a
    point.  A Shape is any collection of topology objects that don't
    necessarily have to be connected to one another.
    Valid operations include - 
       SM_SO_MINIMIZE,  
       SM_SO_MAXIMIZE,
       SM_SO_INTERSECT, 
       SM_SO_NORMALIZE, 
       SM_SO_PROJECTED_MINIMIZE,
       SM_SO_PROJECTED_MAXIMIZE, 
       SM_SO_RAYFIRE,
       SM_SO_3D_SIGNED_DIRECTED_MINIMIZE. 

NOTES:    
   Output,in rSolutions: for operations SM_SO_RAYFIRE and SM_SO_3D_SIGNED_DIRECTED_MINIMIZE:
     m_dSolutionValue is set to the parameter along the ray
     [0] Surface u-value
     [1] Surface v-value


    Note that when doing minimization and maximization a very large 
    (SM_BIG_DOUBLE) or a very small (0.0) value should be set to 
    dBestAnswerSoFarSq unless you already have a limiting value.  
    In otherwords, if you wish to find a minimum only if it is less
    than 0.1 then you should set dBestAnswerSoFarSq = 0.1 * 0.1.
    The solutions array may or may not have existing solutions 
    already registered with it.
 
***********************************************************************/
SmStatus SmTopologySolver::ShapePointSolve
  (const SmShape         * cpShape,            // in : List of topology objects to test for point
   const SmPoint3d       & crPoint,            // in : Point to be tested
   SmSolverOperationType   eSolverOperation,   // in : Operation to apply to point/shape combination
   SmSolutionRequestedType eSolutionRequested, // in : oneof: SM_SR_SINGLE
                                               //             SM_SR_ALL
   double                  d3dTolerance,       // in : Geometric Tolerance for operation
   double                  dBestAnswerSoFarSq, // in : limiting value for min/max operations
                                               //      Set to Large (or Small) value to ignore.
   const SmVector3d      * cpOptVectors,       // in : Extra values needed by some operations.
   SmSolutionArray       & rSolutions)         // out: List of solutions
{
  // check input
  if(   eSolverOperation != SM_SO_MINIMIZE 
     && eSolverOperation != SM_SO_MAXIMIZE 
     && eSolverOperation != SM_SO_RAYFIRE 
     && eSolverOperation != SM_SO_PROJECTED_MINIMIZE 
     && eSolverOperation != SM_SO_PROJECTED_MAXIMIZE 
     && eSolverOperation != SM_SO_3D_SIGNED_DIRECTED_MINIMIZE 
     && eSolverOperation != SM_SO_SIGNED_DIRECTED_MINIMIZE 
     && eSolverOperation != SM_SO_DIRECTED_MINIMIZE 
     && eSolverOperation != SM_SO_DIRECTED_MAXIMIZE 
     && eSolverOperation != SM_SO_INTERSECT 
     && eSolverOperation != SM_SO_NORMALIZE) 
    { SER(SM_ERR_INVALID_INPUT); }

// gwc:removed - no longer needed now that SmTopoLocalSolve has a pointer to SmShape
//        // start a PointerMap for all edges in shape
//        SmTArray<SmEdge*> sSolveEdges;
//        cpShape->GetEdges(sSolveEdges);
//        const SmContext *cpContext = cpShape->GetContext() ;  SM_ASSERT(cpContext != NULL) ;
//        SmMapPtrToPtr *pEMap = new (*cpContext) SmMapPtrToPtr();
//        for (ULONG i=0; i<sSolveEdges.GetSize(); i++) 
//          {
//            SmEdge *pEdge = sSolveEdges[i];
//            pEMap->SetAt(pEdge,pEdge);
//          }

  // init output
  rSolutions.ReSet();

  // this is a static function, so -
  // construct a TopologySolver using the input args and the empty Solution array
  SmTopologySolver sTS(eSolverOperation, eSolutionRequested, d3dTolerance,
                       dBestAnswerSoFarSq, cpOptVectors, rSolutions);
//        sTS.m_pTopologyToSolve = pEMap;

  //place the target point into a PointTree
  SmTreeNode sTreeNode;
  SmObjectList sObjectList;
  SmTree sPointTree(crPoint,0,&sTreeNode,&sObjectList);

  // compare the point with the vertices and the edges in Shape
  // except for operation Normalize)
  if (eSolverOperation != SM_SO_NORMALIZE) 
    {
      // First let's compare the point with the vertices of the Brep
      SmTree *pVertTree = cpShape->GetVertexTree();
      if ( pVertTree != NULL )
        {
          SmTLSPointVertex sTSLPV(sTS, FALSE, cpShape);
          sTSLPV.SetPoint(crPoint);
          SER(sTS.SolveTrees(sPointTree,*pVertTree,sTSLPV));
        }

      // Now compare the point with the curves of the SmShape
      SmTree *pCurveTree = cpShape->GetCurveTree();
      if ( pCurveTree != NULL )
        {
          SmTLSPointEdge sTLSPE(sTS, FALSE, cpShape);
          sTLSPE.SetPoint(crPoint);
          SER(sTS.SolveTrees(sPointTree,*pCurveTree,sTLSPE));
        }

    } // end Not a Normalize Operation check

  // Now compare the point with the faces in Shape
  SmTree *pSurfaceTree = cpShape->GetSurfaceTree();
  SmTLSPointFace sTLSPF(sTS, FALSE, cpShape);
  sTLSPF.SetPoint(crPoint);
  SER(sTS.SolveTrees(sPointTree,*pSurfaceTree,sTLSPF));

//        // clean up
//        SM_ASSERT(sTS.m_pTopologyToSolve != NULL) ; delete sTS.m_pTopologyToSolve ;
//        sTS.m_pTopologyToSolve = NULL;

  // all done
  return SM_SUCCESS;

} // end SmTopologySolver::ShapePointSolve

/*******************************************************************//**
PURPOSE: This method solves a particular operation for a Brep and a
    curve.  Valid operations include:
     
      SM_SO_MINIMIZE,           SM_SO_MAXIMIZE,
      SM_SO_INTERSECT,          SM_SO_NORMALIZE,
      SM_SO_INTERSECT_WIREFRAME, 
      SM_SO_PROJECTED_MINIMIZE,
      SM_SO_PROJECTED_MAXIMIZE, SM_SO_3D_SIGNED_DIRECTED_MINIMIZE,
      SM_SO_DIRECTED_MINIMIZE,  SM_SO_SIGNED_DIRECTED_MINIMIZE, 
      SM_SO_DIRECTED_MAXIMIZE.

    Note that when doing minimization and maximization a very large 
    (SM_BIG_DOUBLE) or a very small (0.0) value should be set to 
    dBestAnswerSoFarSq unless you already have a limiting value.  
    In otherwords, if you wish to find a minimum only if it is less
    than 0.1 then you should set dBestAnswerSoFarSq = 0.1 * 0.1.
    
    The solutions array is cleared in this call before being filled.

    When cpBrep->m_bEditingEnabled == TRUE an error is returned

NOTES: 
***********************************************************************/
SmStatus SmTopologySolver::BrepCurveSolve
  (const SmBrep          * cpBrep,                // in : target Brep
   const SmCurve         & crCurve,               // in : target Curve
   const SmExtent1d      & crInterval,            // in : target Curve interval
   SmSolverOperationType   eSolverOperation,      // in : Oneof the listed values above
   SmSolutionRequestedType eSolutionRequested,    // in : Oneof: SM_SR_SINGLE, SM_SR_ALL, SM_SR_NODES
   double                  d3dTolerance,          // in : size distance for solutions
   double                  dBestAnswerSoFarSq,    // in : Cull value for min/max operations
   const SmVector3d      * cpOptVectors,          // in :
   SmSolutionArray & rSolutions)                  // out: Solutions list. This list is reset
                                                  //      before being loaded in this call.
{
  // check input state
  if (   eSolverOperation != SM_SO_MINIMIZE 
      && eSolverOperation != SM_SO_MAXIMIZE 
      && eSolverOperation != SM_SO_PROJECTED_MINIMIZE 
      && eSolverOperation != SM_SO_PROJECTED_MAXIMIZE 
      && eSolverOperation != SM_SO_SIGNED_DIRECTED_MINIMIZE 
      && eSolverOperation != SM_SO_3D_SIGNED_DIRECTED_MINIMIZE 
      && eSolverOperation != SM_SO_DIRECTED_MINIMIZE 
      && eSolverOperation != SM_SO_DIRECTED_MAXIMIZE 
      && eSolverOperation != SM_SO_INTERSECT 
      && eSolverOperation != SM_SO_INTERSECT_WIREFRAME 
      && eSolverOperation != SM_SO_NORMALIZE) 
    {
      SER(SM_ERR_INVALID_INPUT);
    }

  // set state for operation == SM_SO_INTERSECT_WIREFRAME 
  SmBoolean bWiresOnly = FALSE;
  if (eSolverOperation == SM_SO_INTERSECT_WIREFRAME) 
    {
      bWiresOnly = TRUE;
      eSolverOperation = SM_SO_INTERSECT;
    }

  // init output
  rSolutions.ReSet();

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
  if (bDebugMe) 
    {
      smgfx_Erase();
      smgfx_SetLook(3,5, 1,0,0); crCurve.Draw(); sm_GraphicsLoop();
      smgfx_SetLook(1,3, 0,0,0); cpBrep->Draw(); sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

  // check state - m_bEditingEnabled must be off
  //      SM_ASSERT(cpBrep->m_bEditingEnabled == FALSE) ;
  // gwc: remove check
  //      if (cpBrep->m_bEditingEnabled) 
  //        { SER_MSG(SM_ERR, _T("Tried to edit SmTopologySolver::BrepCurveSolve with m_bEditingEnabled == TRUE")); }

  // instantiate the TopologySolver Object.
  // sTS will put the solutions into rSolutions.
  SmTopologySolver sTS(eSolverOperation,
                       eSolutionRequested,
                       d3dTolerance,
                       dBestAnswerSoFarSq,
                       cpOptVectors,
                       rSolutions);

  // get Brep spatial tree
  SmBrepCache *pBC = (SmBrepCache*)SmCacheMgr::GetOrCreateObjectCache(SM_OC_BREP,cpBrep); 
  NER(pBC);

  // build single node spatial tree for the curve
  // init a single node point tree
  SmTreeNode sTreeNode;
  SmObjectList sObjectList;
  SmTree sCurveTree(SmPoint3d(0,0,0),0,&sTreeNode,&sObjectList);

  // Modify the point tree to be a curve tree with the proper bounding box.
  SmExtent3d sCurveBox;
  SER(crCurve.CalculateBoundingBox(crInterval,&sCurveBox));
  sTreeNode.m_sBBox     = sCurveBox;
  sObjectList.m_sBBox   = sCurveBox;
  sObjectList.m_pObject = (SmCurve *)&crCurve ;

  // for all operations but NORMALIZE
  if (eSolverOperation != SM_SO_NORMALIZE) 
    {
      // First let's compare the Curve with the vertices of the Brep
      SmTree *pVertTree = pBC->GetVertexTree();
      SmTLSCurveVertex sTSLCV(sTS, FALSE, NULL);
      sTSLCV.SetCurve(&crCurve,crInterval);
      SER(sTS.SolveTrees(sCurveTree,*pVertTree,sTSLCV));

      // Now compare the Curve with the edges of the Brep
      SmTree *pCurveTree = pBC->GetCurveTree();
      SmTLSCurveEdge sTLSCE(sTS, FALSE, NULL);
      sTLSCE.SetCurve(&crCurve,crInterval);
      SER(sTS.SolveTrees(sCurveTree,*pCurveTree,sTLSCE));
    }

#ifdef SM_DEBUG_CODE
SmBoolean bMyDebug = FALSE ;
  if (bMyDebug)  
    {
      rSolutions.Dump();
      smgfx_SetLook( 3,4, 0,1,1 ); rSolutions.Draw(); sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

  // when not working only with wires
  if (!bWiresOnly) 
    {
      // Now compare the Curve with the faces of the Brep
      SmTree *pSurfaceTree = pBC->GetSurfaceTree();

      SmTLSCurveFace sTLSCF(sTS, FALSE, NULL);
      sTLSCF.SetCurve(&crCurve,crInterval);
      SER(sTS.SolveTrees(sCurveTree,*pSurfaceTree,sTLSCF));

      if (eSolverOperation == SM_SO_3D_SIGNED_DIRECTED_MINIMIZE) 
        {
          for (ULONG i=0; i<1; i++) 
            {
              // get endPoint
              double dT =   (i==0)
                          ? crInterval.GetMin()
                          : crInterval.GetMax() ;
              SmPoint3d sPnt;
              SER(crCurve.EvaluatePoint(dT,sPnt));

              // build endPoint single node spatial tree and brep Surface tree
              SmTreeNode sNode;
              SmObjectList sObjList;
              SmTree sPointTree(sPnt,0,&sNode,&sObjList);
              SmTree *pSrfTree = pBC->GetSurfaceTree();

              //
              SmTLSPointFace sTLSPF(sTS, FALSE, NULL);
              sTLSPF.SetPoint(sPnt);
              SER(sTS.SolveTrees(sPointTree,*pSrfTree,sTLSPF));

              // for every solution
              for (ULONG j=0; j<rSolutions.GetSize(); j++) 
                {
                  SmSolution & rSol = rSolutions[j];
                  if (   rSol.m_lNumObjects   == 1 
                      && rSol.m_lNumVariables == 2) 
                    {
                      rSol.m_lNumObjects   = 2;
                      rSol.m_apObjects[1]  = rSol.m_apObjects[0];
                      rSol.m_apObjects[0]  = SM_CONST_CAST(SmCurve*,&crCurve);

                      rSol.m_vStart[2]     = rSol.m_vStart[1];
                      rSol.m_vStart[1]     = rSol.m_vStart[0];
                      rSol.m_vStart[0]     = dT;
                      rSol.m_lNumVariables = 3;
                    }
                } // end iter every solution
            } // end iter EndPoint (only StartPoint right now) 
        } // end eSolverOperation == SM_SO_3D_SIGNED_DIRECTED_MINIMIZE check
    } // end !bWiresONly check

#ifdef SM_DEBUG_CODE
  if (bMyDebug) 
    {
      rSolutions.Dump();
      smgfx_SetLook( 3,4, 1,0,1 ); rSolutions.Draw(); sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

  return SM_SUCCESS;

} // end SmTopologySolver::BrepCurveSolve

/*******************************************************************//**
PURPOSE: This method solves a particular operation for a Shape and a
    curve.  Valid operations include - SM_SO_MINIMIZE, SM_SO_MAXIMIZE,
    SM_SO_INTERSECT, SM_SO_NORMALIZE, SM_SO_PROJECTED_MINIMIZE,
    SM_SO_PROJECTED_MAXIMIZE. SM_SO_3D_SIGNED_DIRECTED_MINIMIZE,
    Note that when doing minimization and maximization a very large 
    (SM_BIG_DOUBLE) or a very small (0.0) value should be set to 
    dBestAnswerSoFarSq unless you already have a limiting value.  
    In otherwords, if you wish to find a minimum only if it is less
    than 0.1 then you should set dBestAnswerSoFarSq = 0.1 * 0.1.
    The solutions array may or may not have existing solutions 
    already registered with it.

NOTES: 
***********************************************************************/
SmStatus SmTopologySolver::ShapeCurveSolve
  (const SmShape         * cpShape, 
   const SmCurve         & crCurve,
   const SmExtent1d      & crInterval,
   SmSolverOperationType   eSolverOperation,
   SmSolutionRequestedType eSolutionRequested,
   double                  d3dTolerance,
   double                  dBestAnswerSoFarSq,
   const SmVector3d      * cpOptVectors,
   SmSolutionArray       & rSolutions)
{
  // check input
  if (   eSolverOperation != SM_SO_MINIMIZE 
      && eSolverOperation != SM_SO_MAXIMIZE 
      && eSolverOperation != SM_SO_PROJECTED_MINIMIZE 
      && eSolverOperation != SM_SO_PROJECTED_MAXIMIZE 
      && eSolverOperation != SM_SO_SIGNED_DIRECTED_MINIMIZE 
      && eSolverOperation != SM_SO_3D_SIGNED_DIRECTED_MINIMIZE 
      && eSolverOperation != SM_SO_DIRECTED_MINIMIZE 
      && eSolverOperation != SM_SO_DIRECTED_MAXIMIZE 
      && eSolverOperation != SM_SO_INTERSECT 
      && eSolverOperation != SM_SO_NORMALIZE) 
    {
      SER(SM_ERR_INVALID_INPUT);
    }

  // init output
  rSolutions.ReSet();

  // TopologySolver - place rSolutions in m_pSolutions so that all
  //      subsequent solver calls will add solutions to this output array.
  SmTopologySolver sTS(eSolverOperation, eSolutionRequested, d3dTolerance,
                       dBestAnswerSoFarSq, cpOptVectors, rSolutions);
  SmTreeNode   sTreeNode;
  SmObjectList sObjectList;
  SmTree       sCurveTree(SmPoint3d(0,0,0),0,&sTreeNode,&sObjectList);

  // Modify the point tree to be a curve tree with the proper bounding box.
  SmExtent3d sCurveBox;
  SER(crCurve.CalculateBoundingBox(crInterval,&sCurveBox));
  sTreeNode.m_sBBox     = sCurveBox;
  sObjectList.m_sBBox   = sCurveBox;
  sObjectList.m_pObject = (SmCurve *)&crCurve ;

  // compare curve with vertices and edges (except for Operaition == Normalize)
  if (eSolverOperation != SM_SO_NORMALIZE) 
    {
      // First let's compare the Curve with the vertices of the Brep
      SmTree *pVertTree = cpShape->GetVertexTree();
      SmTLSCurveVertex sTSLCV(sTS, FALSE, cpShape);
      sTSLCV.SetCurve(&crCurve,crInterval);
      SER(sTS.SolveTrees(sCurveTree,*pVertTree,sTSLCV));
      
      // Now compare the Curve with the curves of the Brep
      SmTree *pCurveTree = cpShape->GetCurveTree();
      SmTLSCurveEdge sTLSCE(sTS, FALSE, cpShape);
      sTLSCE.SetCurve(&crCurve,crInterval);
      SER(sTS.SolveTrees(sCurveTree,*pCurveTree,sTLSCE));
    }

  // Now compare the Curve with the faces of the Brep
  SmTree *pSurfaceTree = cpShape->GetSurfaceTree();
  SmTLSCurveFace sTLSCF(sTS, FALSE, cpShape);
  sTLSCF.SetCurve(&crCurve,crInterval);
  SER(sTS.SolveTrees(sCurveTree,*pSurfaceTree,sTLSCF));

  //
  if (eSolverOperation == SM_SO_3D_SIGNED_DIRECTED_MINIMIZE) 
    {
      for (ULONG i=0; i<1; i++) 
        {
          // locals for Solve tree
          double dT = (i==0) ? crInterval.GetMin()
                             : crInterval.GetMax();
          SmPoint3d sPnt;
          SER(crCurve.EvaluatePoint(dT,sPnt));
          SmTreeNode sNode;
          SmObjectList sObjList;
          SmTree sPointTree(sPnt,0,&sNode,&sObjList);
          SmTLSPointFace sTLSPF(sTS, FALSE, cpShape);
          sTLSPF.SetPoint(sPnt);

          //
          SER(sTS.SolveTrees(sPointTree,*pSurfaceTree,sTLSPF));

          // 
          for (ULONG j=0; j<rSolutions.GetSize(); j++) 
            {
              SmSolution & rSol = rSolutions[j];
              if (rSol.m_lNumObjects == 1 && rSol.m_lNumVariables == 2) 
                {
                  rSol.m_lNumObjects = 2;
                  rSol.m_apObjects[1] = rSol.m_apObjects[0];
                  rSol.m_apObjects[0] = SM_CONST_CAST(SmCurve*,&crCurve);
                  rSol.m_vStart[2] = rSol.m_vStart[1];
                  rSol.m_vStart[1] = rSol.m_vStart[0];
                  rSol.m_vStart[0] = dT;
                  rSol.m_lNumVariables = 3;
                }
            }
        } // end 1 iteration
    } // end Operation == SM_SO_3D_SIGNED_DIRECTED_MINIMIZE check


#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  // draw 
  if(bDebugMe)
    {
      rSolutions.Dump();
    }
#endif // SM_DEBUG_CODE

  // all done
  return SM_SUCCESS;

} // end SmTopologySolver::ShapeCurveSolve

/*******************************************************************//**
PURPOSE: Constructor for the topology solver which accepts an array
    of vectors.  
Example:
      This constructor should be used for the following solver
    operations: 
    SM_SO_INTERSECT,                   // Intersect the objects
    SM_SO_MINIMIZE,                    // Minimize distance between the objects
    SM_SO_MAXIMIZE,                    // Maximize distance between the objects
    SM_SO_NORMALIZE,                   // Find points where the objects are parallel to each other

    SM_SO_AT_DISTANCE,                 // Find points on an object at a given distance from another object
    SM_SO_FIND,                        // Just looking for something
    SM_SO_3D_SIGNED_DIRECTED_MINIMIZE, // Minimize 3D signed distance along a given vector
                                       // Finds the largest magnitude negative value
    SM_SO_SIGNED_DIRECTED_MINIMIZE,    // Minimize 2D projected signed distance along a given vector
                                       // Finds the largest magnitude negative value

    SM_SO_DIRECTED_MINIMIZE,           // Minimize 2D projected absolute distance along a given vector
    SM_SO_DIRECTED_MAXIMIZE,           // Maximize 2D projected absolute distance along a given vector
    SM_SO_PROJECTED_MINIMIZE,          // Minimize 2D distance relative to a projection direction
    SM_SO_PROJECTED_MAXIMIZE,          // Maximize 2D distance relative to a projection direction

    SM_SO_ANGLE_MINIMIZE,              // Find minimum absolute angle to a plane relative to point on the plane
    SM_SO_SIGNED_ANGLE_MINIMIZE,       // Find minimum angle relative to a 0 to 180 and 0 to -180 
                                       // angular measurement (positive is counter clockwise relative to
                                       // the reference vector in the plane).  Any negative angle will
                                       // always override a positive angle.  A larger negative angle
                                       // will override a smaller negative value (i.e. -130 overrides
                                       // -100).
    SM_SO_RAYFIRE,                     // Minimize intersections from a point along a given vector in 3D

NOTES: 
***********************************************************************/
SmTopologySolver::SmTopologySolver
  (SmSolverOperationType   eSolverOperation,
   SmSolutionRequestedType eSolutionRequested,
   double                  d3dTolerance,
   double                  dBestAnswerSoFarSq,
   const SmVector3d      * cpOptVectors,
   SmSolutionArray       & rSolutions)
{
  SM_ASSERT(   eSolverOperation == SM_SO_INTERSECT 
            || eSolverOperation == SM_SO_MINIMIZE 
            || eSolverOperation == SM_SO_MAXIMIZE 
            || eSolverOperation == SM_SO_RAYFIRE 
            || eSolverOperation == SM_SO_NORMALIZE 
            || eSolverOperation == SM_SO_AT_DISTANCE 
            || eSolverOperation == SM_SO_FIND 
            || eSolverOperation == SM_SO_SIGNED_DIRECTED_MINIMIZE 
            || eSolverOperation == SM_SO_3D_SIGNED_DIRECTED_MINIMIZE 
            || eSolverOperation == SM_SO_DIRECTED_MINIMIZE   
            || eSolverOperation == SM_SO_DIRECTED_MAXIMIZE  
            || eSolverOperation == SM_SO_PROJECTED_MINIMIZE  
            || eSolverOperation == SM_SO_PROJECTED_MAXIMIZE 
            || eSolverOperation == SM_SO_ANGLE_MINIMIZE      
            || eSolverOperation == SM_SO_SIGNED_ANGLE_MINIMIZE);
  m_eSolverOperation   = eSolverOperation;
  m_eSolutionRequested = eSolutionRequested;
  m_d3dTolerance       = d3dTolerance;
  m_dBestAnswerSoFarSq = dBestAnswerSoFarSq;
  m_pSolutions         = &rSolutions;
  m_cpOptVectors       = cpOptVectors;
  m_pLocalSolver       = NULL;
  if (   eSolverOperation == SM_SO_SIGNED_DIRECTED_MINIMIZE 
      || eSolverOperation == SM_SO_3D_SIGNED_DIRECTED_MINIMIZE   
      || eSolverOperation == SM_SO_DIRECTED_MINIMIZE   
      || eSolverOperation == SM_SO_RAYFIRE   
      || eSolverOperation == SM_SO_DIRECTED_MAXIMIZE  
      || eSolverOperation == SM_SO_PROJECTED_MINIMIZE  
      || eSolverOperation == SM_SO_PROJECTED_MAXIMIZE 
      || eSolverOperation == SM_SO_ANGLE_MINIMIZE      
      || eSolverOperation == SM_SO_SIGNED_ANGLE_MINIMIZE) 
    {
      SM_ASSERT(cpOptVectors != NULL);
    }

  switch (m_eSolverOperation) 
    {
      case SM_SO_DIRECTED_MINIMIZE:        
      case SM_SO_PROJECTED_MINIMIZE:          m_bProjectedOperation = TRUE;
                                              
      case SM_SO_MINIMIZE:                    
      case SM_SO_ANGLE_MINIMIZE:              m_eOperationCategory = SM_OPERATION_MINIMIZE;
                                              break;
                                              
      case SM_SO_DIRECTED_MAXIMIZE:           
      case SM_SO_PROJECTED_MAXIMIZE:          m_bProjectedOperation = TRUE;
                                              
      case SM_SO_MAXIMIZE:                    m_eOperationCategory = SM_OPERATION_MAXIMIZE;
                                              break;
                                              
      case SM_SO_SIGNED_DIRECTED_MINIMIZE:    
      case SM_SO_SIGNED_ANGLE_MINIMIZE:       m_bProjectedOperation = TRUE;
                                              m_eOperationCategory = SM_OPERATION_SIGNED;
                                              break;

      case SM_SO_3D_SIGNED_DIRECTED_MINIMIZE: m_eOperationCategory = SM_OPERATION_SIGNED;
                                              break;

      default:                                m_eOperationCategory = SM_OPERATION_OTHER;
                                              m_bProjectedOperation = FALSE;
                                              break;
    }

} // end SmTopologySolver::SmTopologySolver

/*******************************************************************//**
PURPOSE: Return TRUE the two bounding boxes satisfy the criteria 
   necessary to satisfy the given solution.

NOTES: 
***********************************************************************/
SmBoolean SmTopologySolver::MayContainAnswer
  (const SmExtent3d & crBox1, 
   const SmExtent3d & crBox2) 
 const
{
 switch (m_eSolverOperation) 
   {

      case SM_SO_INTERSECT:
          {
              SmExtent3d sBBox = crBox1;
              sBBox.ExpandAbsolute(m_d3dTolerance);

              if (sBBox.AreDisjoint(crBox2)) 
                { 
                  return FALSE; // No answers in this branch
                }
              return TRUE;  // possible answers in this branch
          }

      case SM_SO_MINIMIZE:
          {
              double dMinDistSq = crBox1.MinimumDistanceSquared(crBox2);
              if (dMinDistSq > m_dBestAnswerSoFarSq) 
                {
                  if (smos_Sqrt(dMinDistSq) - m_d3dTolerance > 
                      smos_Sqrt(m_dBestAnswerSoFarSq) ) 
                    {
                      return FALSE; // no answers in this branch
                    }
                }
              return TRUE;
          }

      case SM_SO_MAXIMIZE:
          {
              double dMaxDistSq = crBox1.MaximumDistanceSquared(crBox2);
              if (dMaxDistSq < m_dBestAnswerSoFarSq) 
                {
                  if (smos_Sqrt(dMaxDistSq) + m_d3dTolerance <
                      smos_Sqrt(m_dBestAnswerSoFarSq) ) 
                    {
                      return FALSE; // no answers in this branch
                    }
                }
              return TRUE;
          }

      case SM_SO_AT_DISTANCE:
          {
              double dMaxDistSq = crBox1.MaximumDistanceSquared(crBox2);
              if (dMaxDistSq < m_dAtDistanceSq) 
                {
                  if (smos_Sqrt(dMaxDistSq) - m_d3dTolerance < 
                      smos_Sqrt(m_dAtDistanceSq) ) 
                    {
                      return FALSE; // no answers in this branch
                    }
                }
              double dMinDistSq = crBox1.MinimumDistanceSquared(crBox2);
              if (dMinDistSq > m_dAtDistanceSq) 
                {
                  if (smos_Sqrt(dMinDistSq) + m_d3dTolerance > 
                      smos_Sqrt(m_dAtDistanceSq) ) 
                    {
                      return FALSE; // no answers in this branch
                    }
                }
              return TRUE; // may be answers
          }

      case SM_SO_NORMALIZE:
          // No optimization here yet - should be added in virtual method
          // for lower level object.
          break;

      case SM_SO_FIND:
          // No optimization for plain searching - should be added as 
          // a virtual method
          break;

      case SM_SO_PROJECTED_MINIMIZE:
      case SM_SO_PROJECTED_MAXIMIZE:
          {
              SmPoint3d sSph1Cent, sSph2Cent;
              double dSph1Radius, dSph2Radius;
              crBox1.ComputeSphereBound(sSph1Cent,dSph1Radius);
              crBox2.ComputeSphereBound(sSph2Cent,dSph2Radius);
              SmPoint3d sProjPnt;
              SER(smgu_PointProjectToPlane(sSph2Cent,sSph1Cent,m_cpOptVectors[0],sProjPnt));
              if (m_eSolverOperation == SM_SO_PROJECTED_MINIMIZE) 
                {
                  double dMinProjDist = sProjPnt.DistanceBetween(sSph1Cent) -
                      dSph1Radius - dSph2Radius;
                  if (dMinProjDist > 0.0 && dMinProjDist*dMinProjDist > m_dBestAnswerSoFarSq) 
                    {
                      return FALSE;
                    }
                }            
              if (m_eSolverOperation == SM_SO_PROJECTED_MAXIMIZE) 
                {
                  double dMaxProjDist = sProjPnt.DistanceBetween(sSph1Cent) +
                      dSph1Radius + dSph2Radius;
                  if (dMaxProjDist*dMaxProjDist < m_dBestAnswerSoFarSq) 
                    {
                      return FALSE;
                    }
                }
          
              // If we were unable to elimiate using above sphere tests then
              // we just skip out here with a TRUE.
              return TRUE;
          }


      // If we are doing a directed minimization we can use the bounding
      // box information to eliminate additional comparisons 
      case SM_SO_DIRECTED_MINIMIZE:
      case SM_SO_SIGNED_DIRECTED_MINIMIZE:
      case SM_SO_DIRECTED_MAXIMIZE:
          {
              // This optimization can be done by creating a plane through the
              // center of one bounding box whose normal is perpendicular to the
              // direction vector and seeing if the distance to the
              // center of the other bounding box is greater than the sum of 
              // the two radii.
              SmVector3d sPlaneNorm = m_cpOptVectors[1] * m_cpOptVectors[0];
              SmPoint3d sSph1Cent, sSph2Cent;
              double dSph1Radius, dSph2Radius;
              crBox1.ComputeSphereBound(sSph1Cent,dSph1Radius);
              crBox2.ComputeSphereBound(sSph2Cent,dSph2Radius);
              SmPoint3d sProjPnt;
              SER(smgu_PointProjectToPlane(sSph2Cent,sSph1Cent,sPlaneNorm,sProjPnt));
              double dSumRadius = dSph1Radius + dSph2Radius;
              if (sProjPnt.DistanceBetweenSquared(sSph1Cent) > 
                  dSumRadius*dSumRadius+SM_EFF_ZERO_SQRT) 
                {
                  return FALSE; // These two nodes can not solve this directed minimization
                }
          
              if (m_eSolverOperation == SM_SO_SIGNED_DIRECTED_MINIMIZE) 
                {
                  SmVector3d sVec = sProjPnt - sSph1Cent;
                  double dDist = sVec.Length();
                  if (sVec.Dot(m_cpOptVectors[1]) < 0) 
                    {
                      dDist = - dDist;
                    }
                  double dMinProjDist = dDist - dSph1Radius - dSph2Radius;
                  if (dMinProjDist*smos_Fabs(dMinProjDist) > m_dBestAnswerSoFarSq) 
                    {
                      return FALSE;
                    }
                }
          
              if (m_eSolverOperation == SM_SO_DIRECTED_MINIMIZE) 
                {
                  double dMinProjDist = sProjPnt.DistanceBetween(sSph1Cent) -
                      dSph1Radius - dSph2Radius;
                  if (dMinProjDist > 0.0 && dMinProjDist*dMinProjDist > m_dBestAnswerSoFarSq) 
                    {
                      return FALSE;
                    }
                }
          
              if (m_eSolverOperation == SM_SO_DIRECTED_MAXIMIZE) 
                {
                  double dMaxProjDist = sProjPnt.DistanceBetween(sSph1Cent) +
                      dSph1Radius + dSph2Radius;
                  if (dMaxProjDist*dMaxProjDist < m_dBestAnswerSoFarSq) 
                    {
                      return FALSE;
                    }
                }
          
              return TRUE;
          }        

          // Doing a ray firing - minimization of intersections
          // along a vector.
      case SM_SO_RAYFIRE:
          {
              // get the ray
              SmVector3d sLinePnt = crBox1.GetMin();
              SmPoint3d  sLineVec = m_cpOptVectors[0];

              // expand target bounding box by tolerance
              SmExtent3d sObjectBox = crBox2 ;
              sObjectBox.ExpandAbsolute(m_d3dTolerance) ;

              // intersect ray with boundingbox + tol
              ULONG lNumFound ;
              double dTEnter, dTExit ;
              sObjectBox.IntersectLine(sLinePnt, sLineVec, lNumFound, dTEnter, dTExit) ;

              // when there is no intersection
              if(lNumFound == 0)
                { return FALSE ; }

              // when there is no intersection on the right side of the ray
              if(dTEnter < 0.0 && dTExit < 0.0)
                { return FALSE ; }

              // See if we are further out on the positive side of the ray than
              // the best answer so far.
              if(   dTEnter > 0.0 
                 && dTEnter*dTEnter > m_dBestAnswerSoFarSq)
                { return FALSE ; }

              // old method using sphere/line intersections
              //      SmPoint3d sSph1Cent;
              //      double dSph1Radius;
              //      crBox2.ComputeSphereBound(sSph1Cent,dSph1Radius);
              //      double dParam;
              //      SER(smgu_LineClosestPoint(sLinePnt,sLineVec,sSph1Cent,dParam));
              //      SmPoint3d sClPnt = sLinePnt + dParam * sLineVec;
              //      double dDist = sClPnt.DistanceBetween(sSph1Cent);
              //      // See if sphere does not intersect the infinite line
              //      if (dDist > dSph1Radius + m_d3dTolerance) 
              //        {
              //          return FALSE;
              //        }
              //      // See if we are on wrong side of ray start point
              //      if (dParam + dSph1Radius < 0.0) 
              //        {
              //          return FALSE;
              //        }
              //      // See if we are further out on the positive side of the ray than
              //      // the best answer so far.
              //      double dMinPossibleRayDistance = dParam - dSph1Radius - m_d3dTolerance;
              //      if (dMinPossibleRayDistance > 0.0) 
              //        {
              //          if (dMinPossibleRayDistance*dMinPossibleRayDistance > m_dBestAnswerSoFarSq) 
              //            {
              //              return FALSE;
              //            }
              //        }
              //      // Do some additional tests to see if the ray does not intersect with the
              //      // box of the object.
              //      double dLeng = 2.0 * (smos_Fabs(dParam) + dSph1Radius);
              //      SmPoint3d sEndPnt = sLinePnt +  dLeng * sLineVec;
              //      SmPoint3d sStartPnt = sLinePnt - dLeng * sLineVec;
              //      SmExtent3d sRayBox(sStartPnt);
              //      sRayBox.AddPoint3d(sEndPnt);
              //      sRayBox.ExpandAbsolute(m_d3dTolerance);
              //      if (sRayBox.AreDisjoint(crBox2)) 
              //        {
              //          return FALSE;
              //        }

              return TRUE;
          }    
      
          // If we are doing a 3D directed minimization we can use the bounding
          // box information to eliminate additional comparisons 
      case SM_SO_3D_SIGNED_DIRECTED_MINIMIZE:
          {
              SmPoint3d sSph1Cent, sSph2Cent;
              double dSph1Radius, dSph2Radius;
              crBox1.ComputeSphereBound(sSph1Cent,dSph1Radius);
              crBox2.ComputeSphereBound(sSph2Cent,dSph2Radius);
              SmVector3d sLinePnt = sSph1Cent;
              SmPoint3d sLineVec = m_cpOptVectors[0];
              double dParam;
              SER(smgu_LineClosestPoint(sLinePnt,sLineVec,sSph2Cent,dParam));
              SmPoint3d sClPnt = sLinePnt + dParam * sLineVec;
              double dDist = sClPnt.DistanceBetween(sSph2Cent);
              double dSumRadius = dSph1Radius + dSph2Radius;
              // If two spheres don't intersect as projected along the vector
              // return FALSE.
              if (dDist > dSumRadius + m_d3dTolerance) 
                {
                  return FALSE;
                }

              SmVector3d sVec = sClPnt - sSph1Cent;
              double dDistTo = sVec.Length();
              if (sVec.Dot(m_cpOptVectors[0]) < 0) 
                {
                  dDistTo = - dDistTo;
                }
              double dMinDist = dDistTo - dSph1Radius - dSph2Radius - m_d3dTolerance;
              if (dMinDist*smos_Fabs(dMinDist) > m_dBestAnswerSoFarSq) 
                {
                  return FALSE;
                }
          
              return TRUE;
          }
      
      default:
          ERR(SM_ERR);  // not handled yet
   }

 return TRUE;

} // end SmTopologySolver::MayContainAnswer

/*******************************************************************//**
PURPOSE: 

NOTES:
          local solve needs to be done for all intermediate nodes as well
          as all leaf nodes.  Assume that the local solver will be called for 
          both cases.  It should return rbNeedsMoreSubdivision = FALSE only when
          reaching all leaf nodes.  
***********************************************************************/

SmStatus SmTopologySolver::LocalSolve
  (SmTreeNode * apBranch[SM_GS_MAX_TREES],  // in : pair of treeNodes to compare
   SmBoolean & rbNeedsMoreSubdivision)      // out: TRUE  = Nodes have children that need to be processed
                                            //      FALSE = Bounding boxes are disjoint or
                                            //              Nodes have no more children
{
  // init output
  rbNeedsMoreSubdivision = FALSE;

  // locals
  SmTreeNode *pNode1     = apBranch[0];
  SmTreeNode *pNode2     = apBranch[1];
  
  SmObjectList* aData1[20];
  SmObjectList* aData2[20];
  SmTArray<SmObjectList*> sObjs1(20,aData1);
  SmTArray<SmObjectList*> sObjs2(20,aData2);

  // get all pNode1 objects
  pNode1->GetObjectList(sObjs1);
  ULONG i, j;

  // for all objects in pNode1
  for (i=0; i<sObjs1.GetSize(); i++) 
    {
      SmObjectList *pObjList1 = sObjs1[i];

      // skip objects with BoundingBoxes that don't intersect pNode2->BoundingBox
      if (!MayContainAnswer(pObjList1->m_sBBox,pNode2->m_sBBox))
        { continue; }

      // get pNode2 objects
      pNode2->GetObjectList(sObjs2);
      for (j=0; j<sObjs2.GetSize(); j++) 
        {
          SmObjectList *pObjList2 = sObjs2[j];

          // skip object pairs with disjoint bounding boxes
          if (!MayContainAnswer(pObjList1->m_sBBox,pObjList2->m_sBBox)) 
            { continue; }

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
          if (bDebugMe) 
            {
              SmBrep *pBrep1 = pObjList1->GetBrep() ;
              SmBrep *pBrep2 = pObjList2->GetBrep() ;
              smgfx_Erase();    // note: when raycasting pObjList1->m_pObject is NULL
              smgfx_SetLook(1,2, 0,0,1) ; if(pBrep1) pBrep1->Draw(TRUE) ; sm_GraphicsLoop() ;
              smgfx_SetLook(1,2, 0,1,0) ; if(pBrep2) pBrep2->Draw(TRUE) ; sm_GraphicsLoop() ;
              smgfx_SetLook(2,3, 1,0,0); pObjList1->Draw(); sm_GraphicsLoop();
              smgfx_SetLook(2,3, 0,0,0); pObjList2->Draw(); sm_GraphicsLoop();
              sm_GraphicsLoop();                                 
            }
#endif // SM_DEBUG_CODE

          // arrive here when we have two objects whose BBoxes intersect.
          // Invoke the local solver.
          SER(m_pLocalSolver->SolveIt(pObjList1->m_pObject, pObjList2->m_pObject));

        } // end iter all pNode2 objects
    }  // end iter all pNode1 objects

  // Set output to keep going down the trees if there are more nodes below
  if (   pNode1->m_pChild1 != NULL 
      || pNode2->m_pChild1 != NULL) 
    {
      rbNeedsMoreSubdivision = TRUE;
    }

  return SM_SUCCESS;

} // end SmTopologySolver::LocalSolve

/*******************************************************************//**
PURPOSE: Using an extent which represents some object, search the tree
    and find nodes which satisfy the desired solution.

NOTES: 
***********************************************************************/
SmStatus SmTopologySolver::SolveTrees
  (SmTree            & rTree1,       // in : 1st target tree
   SmTree            & rTree2,       // in : 2nd target tree
   SmTopoLocalSolver & rLocalSolve)  // in : Local Solver to place in m_pLocalSolver
{
  m_pLocalSolver  = &rLocalSolve;
  m_lNumVariables = 0;
  m_apTrees[0]    = &rTree1;
  m_apTrees[1]    = &rTree2;
  m_lNumTrees     = 2;
  
  // set up maximize problems with no size constraints
  if (   m_dBestAnswerSoFarSq == SM_BIG_DOUBLE 
      && m_eOperationCategory == SM_OPERATION_MAXIMIZE) 
    { m_dBestAnswerSoFarSq = 0.0; }
    

  // Crash avoidance:
  if ( m_apTrees[0] == NULL || m_apTrees[1] == NULL )
    {
      SM_DBG_WARN( _T("Null Tree passed to SmTopologySolver::SolveTrees()") );
      return SM_SUCCESS;
    }

  // locals
  SmTreeNode *sStartBranch[SM_GS_MAX_TREES];
  sStartBranch[0] = m_apTrees[0]->GetTopNode();
  sStartBranch[1] = m_apTrees[1]->GetTopNode();

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
if(bDebugMe) 
  { m_pSolutions->Dump() ;
    int lParentCount0=0, lLeafCount0=0;
    int lParentCount1=0, lLeafCount1=0;
    if(sStartBranch[0]) sStartBranch[0]->Dump(0,TRUE,&lParentCount0,&lLeafCount0) ;
    if(sStartBranch[1]) sStartBranch[1]->Dump(0,TRUE,&lParentCount1,&lLeafCount1) ;
  }
#endif

  // pass the call along
  SER(SolveBranch(sStartBranch));

  return SM_SUCCESS;
        
} // end SmTopologySolver::SolveTrees

/*******************************************************************//**
PURPOSE: Break one tree down into a list and solve against the other.

NOTES: 
***********************************************************************/
SmStatus SmTopologySolver::SolveTwoTrees
  (SmTree            & rTree1,
   SmTree            & rTree2,
   SmTopoLocalSolver & rLocalSolve)
{
  SmObjectList * sData[2048];
  SmTArray<SmObjectList*> sObjects(2048,sData);
  SER(rTree1.GetObjectList(sObjects));

  // Use a modified point tree and put each object into it one
  // at a time and solve against the other tree.
  SmTreeNode sTreeNode;
  SmObjectList sObjectList;
  SmTree sObjectTree(SmPoint3d(0,0,0),0,&sTreeNode,&sObjectList);

  m_pLocalSolver  = &rLocalSolve;
  m_lNumVariables = 0;
  m_apTrees[0]    = &sObjectTree;
  m_apTrees[1]    = &rTree2;
  m_lNumTrees     = 2;

  for (ULONG i=0; i<sObjects.GetSize(); i++) 
    {
      SmObjectList * pObj   = sObjects[i];
      sTreeNode.m_sBBox     = pObj->m_sBBox;
      sObjectList.m_sBBox   = pObj->m_sBBox;
      sObjectList.m_pNext   = NULL;
      sObjectList.m_pObject = pObj->m_pObject;
      SER(SolveTrees(sObjectTree,rTree2,rLocalSolve));
    }
  return SM_SUCCESS;    

} // end SmTopologySolver::SolveTwoTrees

/****************************************************************
PURPOSE: Static internal method to solve for two trees

NOTES:

*****************************************************************/ 

static SmStatus sm_SolveTwoTrees
  (SmTopologySolver      & rTS,
   SmSolverOperationType   eSolverOperation,
   SmTree                * pVertTree1,
   SmTree                * pCurveTree1,
   SmTree                * pSurfaceTree1,
   SmTree                * pVertTree2,
   SmTree                * pCurveTree2,
   SmTree                * pSurfaceTree2)
{
  if (eSolverOperation != SM_SO_NORMALIZE) 
    {
      // First let's compare vertices of each
      {
        SmTLSVertexVertex sTSLVV(rTS, FALSE, NULL);
        SER(rTS.SolveTwoTrees(*pVertTree1,*pVertTree2,sTSLVV));
      }
      // Now let's compare the edges with the vertices of each.
      {
        SmTLSVertexEdge sTSLVE(rTS, FALSE, NULL);
        SER(rTS.SolveTwoTrees(*pVertTree1,*pCurveTree2,sTSLVE));
      }
      // Do edges/vertices and swap order
      {
        SmTLSVertexEdge sTSLVE2(rTS, TRUE, NULL);
        SER(rTS.SolveTwoTrees(*pCurveTree1,*pVertTree2,sTSLVE2));
      }
      // Now let's compare the edges of each
      {
        SmTLSEdgeEdge sTSLEE(rTS, FALSE, NULL);
        SER(rTS.SolveTwoTrees(*pCurveTree1,*pCurveTree2,sTSLEE));
      }
      // Now that we have done the simple stuff - do the faces
      // which are more complex.

      // First do points vs. faces
      {
        SmTLSVertexFace sTSLVF(rTS, FALSE, NULL);
        SER(rTS.SolveTwoTrees(*pVertTree1,*pSurfaceTree2,sTSLVF));
      }
      {
        SmTLSVertexFace sTSLVF2(rTS, TRUE, NULL);
        SER(rTS.SolveTwoTrees(*pSurfaceTree1,*pVertTree2,sTSLVF2));
      }
      // Now do edges vs. faces
      {
        SmTLSEdgeFace sTSLEF(rTS, FALSE, NULL);
        SER(rTS.SolveTwoTrees(*pCurveTree1,*pSurfaceTree2,sTSLEF));
      }
      {
        SmTLSEdgeFace sTSLEF2(rTS, TRUE, NULL);
        SER(rTS.SolveTwoTrees(*pSurfaceTree1,*pCurveTree2,sTSLEF2));
      }
    }

  // Lastly compare the faces
  {
    SmTLSFaceFace sTLSFF(rTS, FALSE, NULL);
    SER(rTS.SolveTwoTrees(*pSurfaceTree1,*pSurfaceTree2,sTLSFF));
  }

  // all done
  return SM_SUCCESS;

} // end sm_SolveTwoTrees

/*******************************************************************//**
PURPOSE: This method solves for a particular operation between two
    Breps.  Valid operations include: SM_SO_MINIMIZE, SM_SO_MAXIMIZE,
    SM_SO_NORMALIZE, SM_SO_PROJECTED_MINIMIZE, SM_SO_PROJECTED_MAXIMIZE,
    SM_SO_INTERSECT, SM_SO_3D_SIGNED_DIRECTED_MINIMIZE.

NOTES: 
***********************************************************************/
SmStatus SmTopologySolver::BrepBrepSolve
  (const SmBrep          * cpBrep1, 
   const SmBrep          * cpBrep2, 
   SmSolverOperationType   eSolverOperation,
   SmSolutionRequestedType eSolutionRequested,
   double                  d3dTolerance,
   double                  dBestAnswerSoFarSq,
   const SmVector3d      * cpOptVectors,
   SmSolutionArray       & rSolutions)
{
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
  if (bDebugMe) 
    {
      smgfx_Erase();
      smgfx_SetLook(1,2, 0,0,1); cpBrep1->Draw(); sm_GraphicsLoop();
      smgfx_SetLook(1,2, 0,1,0); cpBrep2->Draw(); sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
#endif

  if (   eSolverOperation != SM_SO_MINIMIZE 
      && eSolverOperation != SM_SO_MAXIMIZE 
      && eSolverOperation != SM_SO_PROJECTED_MINIMIZE 
      && eSolverOperation != SM_SO_PROJECTED_MAXIMIZE 
      && eSolverOperation != SM_SO_3D_SIGNED_DIRECTED_MINIMIZE 
      && eSolverOperation != SM_SO_SIGNED_DIRECTED_MINIMIZE 
      && eSolverOperation != SM_SO_DIRECTED_MINIMIZE 
      && eSolverOperation != SM_SO_DIRECTED_MAXIMIZE 
      && eSolverOperation != SM_SO_INTERSECT 
      && eSolverOperation != SM_SO_NORMALIZE) 
    {
      SER(SM_ERR_INVALID_INPUT);
    }

  rSolutions.ReSet();

  // check state - m_bEditingEnabled must be off
  //      SM_ASSERT(cpBrep1->m_bEditingEnabled == FALSE) ;
  //      SM_ASSERT(cpBrep2->m_bEditingEnabled == FALSE) ;
  // gwc: remove check
  //     if (cpBrep1->m_bEditingEnabled) 
  //       { SER_MSG(SM_ERR, _T("Tried to edit SmTopologySolver::BrepBrepSolve with m_bEditingEnabled == TRUE")); }
  // gwc: remove check
  //     if (cpBrep2->m_bEditingEnabled) 
  //       { SER_MSG(SM_ERR, _T("Tried to edit SmTopologySolver::BrepBrepSolve with m_bEditingEnabled == TRUE")); }

  SmTopologySolver sTS(eSolverOperation,eSolutionRequested,d3dTolerance,
      dBestAnswerSoFarSq,cpOptVectors,rSolutions);

  SmBrepCache *pBC1 = (SmBrepCache*)SmCacheMgr::GetOrCreateObjectCache(SM_OC_BREP,cpBrep1); 
  NER(pBC1);
  SmBrepCache *pBC2 = (SmBrepCache*)SmCacheMgr::GetOrCreateObjectCache(SM_OC_BREP,cpBrep2); 
  NER(pBC2);

  SmTree *pVertTree1    = pBC1->GetVertexTree();
  SmTree *pCurveTree1   = pBC1->GetCurveTree();
  SmTree *pSurfaceTree1 = pBC1->GetSurfaceTree();

  SmTree *pVertTree2    = pBC2->GetVertexTree();
  SmTree *pCurveTree2   = pBC2->GetCurveTree();
  SmTree *pSurfaceTree2 = pBC2->GetSurfaceTree();

  SER(sm_SolveTwoTrees(sTS,eSolverOperation,
                         pVertTree1,pCurveTree1,pSurfaceTree1,
                         pVertTree2,pCurveTree2,pSurfaceTree2));

  return SM_SUCCESS;

} // end SmTopologySolver::BrepBrepSolve

/*******************************************************************//**
PURPOSE: This method solves for a particular operation between two
    Breps.  Valid operations include: SM_SO_MINIMIZE, SM_SO_MAXIMIZE,
    SM_SO_NORMALIZE, SM_SO_PROJECTED_MINIMIZE, SM_SO_PROJECTED_MAXIMIZE,
    SM_SO_INTERSECT, SM_SO_3D_SIGNED_DIRECTED_MINIMIZE.

NOTES: 
***********************************************************************/
SmStatus SmTopologySolver::ShapeShapeSolve
  (const SmShape         * cpShape1, 
   const SmShape         * cpShape2, 
   SmSolverOperationType   eSolverOperation,
   SmSolutionRequestedType eSolutionRequested,
   double                  d3dTolerance,
   double                  dBestAnswerSoFarSq,
   const SmVector3d      * cpOptVectors,
   SmSolutionArray       & rSolutions)
{
  if (   eSolverOperation != SM_SO_MINIMIZE 
      && eSolverOperation != SM_SO_MAXIMIZE 
      && eSolverOperation != SM_SO_PROJECTED_MINIMIZE 
      && eSolverOperation != SM_SO_PROJECTED_MAXIMIZE 
      && eSolverOperation != SM_SO_SIGNED_DIRECTED_MINIMIZE 
      && eSolverOperation != SM_SO_3D_SIGNED_DIRECTED_MINIMIZE 
      && eSolverOperation != SM_SO_DIRECTED_MINIMIZE 
      && eSolverOperation != SM_SO_DIRECTED_MAXIMIZE 
      && eSolverOperation != SM_SO_INTERSECT 
      && eSolverOperation != SM_SO_NORMALIZE) 
    {
      SER(SM_ERR_INVALID_INPUT);
    }

  rSolutions.ReSet();

  SmTopologySolver sTS(eSolverOperation,eSolutionRequested,d3dTolerance,
                       dBestAnswerSoFarSq,cpOptVectors,rSolutions);

  SmTree *pVertTree1    = cpShape1->GetVertexTree();
  SmTree *pCurveTree1   = cpShape1->GetCurveTree();
  SmTree *pSurfaceTree1 = cpShape1->GetSurfaceTree();

  SmTree *pVertTree2    = cpShape2->GetVertexTree();
  SmTree *pCurveTree2   = cpShape2->GetCurveTree();
  SmTree *pSurfaceTree2 = cpShape2->GetSurfaceTree();

  SER(sm_SolveTwoTrees(sTS,eSolverOperation,
                         pVertTree1,pCurveTree1,pSurfaceTree1,
                         pVertTree2,pCurveTree2,pSurfaceTree2));

  return SM_SUCCESS;

} // end SmTopologySolver::ShapeShapeSolve


/*******************************************************************//**
PURPOSE: This method gets the current best answer so far

NOTES: sets output when m_eOperationCategory is oneof
      SM_OPERATION_MINIMIZE  outputs(positive value)
      SM_OPERATION_MAXIMIZE  outputs(positive value)
      SM_OPERATION_SIGNED    outputs(positive or negative value)

   else inits output to 0.0 and NULL and returns.
***********************************************************************/
void SmTopologySolver::GetBestAnswerSoFar
  (double   & rdBestAnwser,      // out: set to sqrt(m_dBestAnswerSoFarSq) or 0.0
   double * & rpdBestAnswer)     // out: set to &rdBestAnwser or NULL
{
  // init output
  rpdBestAnswer = NULL;
  rdBestAnwser  = 0.0 ;

  if (   m_dBestAnswerSoFarSq <  SM_BIG_DOUBLE
      && m_eOperationCategory == SM_OPERATION_MINIMIZE) 
    {
      rdBestAnwser  = smos_Sqrt(m_dBestAnswerSoFarSq);
      rpdBestAnswer = &rdBestAnwser;
    }

  if (   m_dBestAnswerSoFarSq >  0.0
      && m_eOperationCategory == SM_OPERATION_MAXIMIZE) 
    {
      rdBestAnwser  = smos_Sqrt(m_dBestAnswerSoFarSq);
      rpdBestAnswer = &rdBestAnwser;
    }

  if (m_eOperationCategory == SM_OPERATION_SIGNED) 
    {
      rdBestAnwser = (  m_dBestAnswerSoFarSq >= 0.0
                      ?  smos_Sqrt(smos_Fabs(m_dBestAnswerSoFarSq))
                      : -smos_Sqrt(smos_Fabs(m_dBestAnswerSoFarSq))) ;
      rpdBestAnswer = &rdBestAnwser;
    }

} // end SmTopologySolver::GetBestAnswerSoFar

/*******************************************************************//**
PURPOSE:
NOTES:
***********************************************************************/
SmBoolean SmTopologySolver::IsKindOf( SM_TYPE t ) const
{
  return ((SmTopologySolver_TYPE == t) ? TRUE : SmGlobalSolver::IsKindOf( (t) ));
}

/*******************************************************************//**
PURPOSE: Pretty print   

NOTES: 
***********************************************************************/
void SmTopologySolver::Dump() const
{
  // TCHAR sBuff[SM_TBLOCK_SIZE];

  // start
  smos_WriteBuffer(_T("\nBegin SmTopologySolver::Dump()")) ;

  // dump base
  SmGlobalSolver::Dump() ;

  // pretty print values

  // end
  smos_WriteBuffer(_T("\nEnd SmTopologySolver::Dump()\n")) ;

} // end SmTopologySolver::Dump
