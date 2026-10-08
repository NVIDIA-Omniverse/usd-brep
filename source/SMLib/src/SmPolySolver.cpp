// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************//**
* FILE NAME --- SmPolySolver.cpp
* PURPOSE:
**********************************************************************/

#include "StdAfx.h"

#include <SmPoly.h>
#include <SmPolySolver.h>
#include <SmCacheMgrBrep.h>
#include <SmBrepCache.h>
#include <SmTree.h>
#include <SmSolutionArray.h>
#include <SmGeomUtility.h>
#include <SmGraphicsExtern.h>
#include <SmExtent2d.h>

/*******************************************************************//**
PURPOSE: Do local solve between point and vertex.

NOTES:
***********************************************************************/
SmStatus SmPLSPointVertex::SolveIt
  (const SmObject *pObj1,
   const SmObject *pObj2)
{
  // Normally the point comes first and the vertex second.  If
  // we want we can switch the order.
  SmSolution sSol;
  sSol.m_apObjects[0] = NULL;
  sSol.m_apObjects[1] = NULL;

  SmPolyVertex *pV;
  ULONG lObjectCount = 0;
  if(!m_bSwapOrder)
    {
      pV = SM_CAST_PTR(SmPolyVertex,pObj2); NER(pV);
      if(m_pPointVertex)
        {
          // If the point represents a vertex then save it in list
          sSol.m_apObjects[lObjectCount++] = SM_REINTERPRET_CAST(SmObject*,
                                             SM_CONST_CAST(SmPolyVertex*, m_pPointVertex));
        }
      sSol.m_apObjects[lObjectCount++] = pV;
    } // end no swap order brance
  else // swap order
    {
      pV = SM_CAST_PTR(SmPolyVertex,pObj1); NER(pV);
      sSol.m_apObjects[lObjectCount++] = pV;
      if(m_pPointVertex)
        {
          // If the point represents a vertex then save it in list
          sSol.m_apObjects[lObjectCount++] = SM_REINTERPRET_CAST(SmObject*,
                                             SM_CONST_CAST(SmPolyVertex*,m_pPointVertex));
        }
    } // end swap order branch

  // local PolyVertex data
  SmPoint3d sVertPnt = pV->GetPoint();
  SmVector3d sDiff   = m_vPoint - pV->GetPoint();
  double dDistSq     = sDiff.LengthSquared();
  double dSign       = 1.0;

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
  if(bDebugMe)
    {
      smgfx_SetLook(1,2, 1,0,0); m_vPoint.Draw(); sm_GraphicsLoop();
      smgfx_SetLook(1,2, 0,0,1); sVertPnt.Draw(); sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
#endif

  // If we are projecting then project the difference vector
  if(m_rPolySolver.m_bProjectedOperation == TRUE)
    {
      SmVector3d sPlaneNorm;
      SmVector3d sVecProj;
      if(   m_rPolySolver.m_eSolverOperation == SM_SO_SIGNED_DIRECTED_MINIMIZE
         || m_rPolySolver.m_eSolverOperation == SM_SO_DIRECTED_MINIMIZE
         || m_rPolySolver.m_eSolverOperation == SM_SO_DIRECTED_MAXIMIZE)

        {
          sPlaneNorm = m_rPolySolver.m_cpOptVectors[0];
          sVecProj = sPlaneNorm * sDiff * sPlaneNorm;
          if(!sVecProj.IsParallelTo(m_rPolySolver.m_cpOptVectors[1],2.0))
              return SM_SUCCESS; // no answers in this branch
        }
      else
        {
          sPlaneNorm = m_rPolySolver.m_cpOptVectors[0];
          sVecProj = sPlaneNorm * sDiff * sPlaneNorm;
        }
      dDistSq = sVecProj.LengthSquared();
      if(   m_rPolySolver.m_eSolverOperation == SM_SO_SIGNED_DIRECTED_MINIMIZE
         && sVecProj.Dot(m_rPolySolver.m_cpOptVectors[1]) < 0)
        {
          dSign = -1.0;
        }
    }

  if(m_bSwapOrder && m_rPolySolver.m_eOperationCategory == SM_OPERATION_SIGNED)
    {
      // If we are doing a signed operation and the order is reversed then we have to
      // reverse the sign of the result.
      dSign = -1.0 * dSign;
    }

  switch (m_rPolySolver.m_eSolverOperation)
    {
      case SM_SO_3D_SIGNED_DIRECTED_MINIMIZE:

            {
              SmVector3d sDirectionVec = m_rPolySolver.m_cpOptVectors[0];
              double dParam;
              SER(smgu_LineClosestPoint(m_vPoint,sDirectionVec,
                  pV->GetPoint(),dParam));
              double dTol = m_rPolySolver.m_d3dTolerance + pV->GetTolerance();

              SmPoint3d sRayPnt = m_vPoint + dParam * sDirectionVec;
              double dDistToRay = sRayPnt.DistanceBetween(pV->GetPoint());
              if(dDistToRay > dTol)
                {
                  return SM_SUCCESS; // No answer if vertex is too far from ray
                }
              if( m_bSwapOrder )
                { dParam = -dParam; }
              // The following assumes that the direction vector is unitized
              dDistSq = dParam * dParam;
              dSign = 1.0;
              if(dParam < 0.0) dSign = -1.0;
            }
          // Continue on!
      case SM_SO_SIGNED_DIRECTED_MINIMIZE:
          if(dSign*dDistSq > m_rPolySolver.m_dBestAnswerSoFarSq)
            {
              double dBestAnswer = smos_Sqrt(smos_Fabs(m_rPolySolver.m_dBestAnswerSoFarSq));
              if(m_rPolySolver.m_dBestAnswerSoFarSq < 0.0) dBestAnswer = - dBestAnswer;
              if(dSign*smos_Sqrt(dDistSq) - m_rPolySolver.m_d3dTolerance > dBestAnswer)
                {
                  return SM_SUCCESS; // no answers in this branch
                }
            }
          break;

      case SM_SO_PROJECTED_MINIMIZE:
      case SM_SO_MINIMIZE:
      case SM_SO_DIRECTED_MINIMIZE:
          if(dDistSq > m_rPolySolver.m_dBestAnswerSoFarSq)
            {
              if(smos_Sqrt(dDistSq) - m_rPolySolver.m_d3dTolerance >
                  smos_Sqrt(m_rPolySolver.m_dBestAnswerSoFarSq) )
                {
                  return SM_SUCCESS; // no answers in this branch
                }
            }
          break;

      case SM_SO_PROJECTED_MAXIMIZE:
      case SM_SO_MAXIMIZE:
      case SM_SO_DIRECTED_MAXIMIZE:
          if(dDistSq < m_rPolySolver.m_dBestAnswerSoFarSq)
            {
              if(smos_Sqrt(dDistSq) + m_rPolySolver.m_d3dTolerance <
                  smos_Sqrt(m_rPolySolver.m_dBestAnswerSoFarSq) )
                {
                  return SM_SUCCESS; // no answers in this branch
                }
            }
          break;

      case SM_SO_AT_DISTANCE:
          if(dDistSq < m_rPolySolver.m_dAtDistanceSq)
            {
              if(smos_Sqrt(dDistSq) - m_rPolySolver.m_d3dTolerance <
                  smos_Sqrt(m_rPolySolver.m_dAtDistanceSq) )
                {
                  return SM_SUCCESS; // no answers in this branch
                }
            }
          if(dDistSq > m_rPolySolver.m_dAtDistanceSq)
            {
              if(smos_Sqrt(dDistSq) + m_rPolySolver.m_d3dTolerance >
                  smos_Sqrt(m_rPolySolver.m_dAtDistanceSq) )
                {
                  return SM_SUCCESS; // no answers in this branch
                }
            }
          break;

      case SM_SO_INTERSECT:
          if(smos_Sqrt(dDistSq) > m_rPolySolver.m_d3dTolerance)
            {
              return SM_SUCCESS; // no answers in this branch
            }
          break;

      case SM_SO_NORMALIZE:
      case SM_SO_FIND:
          return SM_SUCCESS;

      default:
          ERR(SM_ERR);  // not handled yet
    }

  ULONG lPointIndex = (!m_bSwapOrder) ? 0 : 3 ;

  // set output
  sSol.m_eSolutionType           = SM_ST_POLY_SINGLE_VALUE;
  sSol.m_lNumVariables           = 6;
  sSol.m_vStart.m_dSolutionValue = dSign*smos_Sqrt(dDistSq);
  sSol.m_lNumObjects             = lObjectCount;
  sSol.m_vStart[lPointIndex]     = m_vPoint.x;
  sSol.m_vStart[lPointIndex+1]   = m_vPoint.y;
  sSol.m_vStart[lPointIndex+2]   = m_vPoint.z;
  sSol.m_vStart[3 - lPointIndex] = sVertPnt.x;
  sSol.m_vStart[4 - lPointIndex] = sVertPnt.y;
  sSol.m_vStart[5 - lPointIndex] = sVertPnt.z;

  // save output
  m_rPolySolver.AddSortedSolution(sSol,SM_SK_BY_SOLUTION_VALUE);

  return SM_SUCCESS;

} // end SmPLSPointVertex::SolveIt

/*******************************************************************//**
PURPOSE: Local solver for Vertex Vertex.  Just invokes
   superclass after loading data.

NOTES:
***********************************************************************/
SmStatus SmPLSVertexVertex::SolveIt
  (const SmObject *pObj1,
   const SmObject *pObj2)
{
  SM_ASSERT(pObj1 != NULL && pObj2 != NULL);
  SM_ASSERT(pObj1->IsKindOf(SmPolyVertex_TYPE));
  SM_ASSERT(pObj2->IsKindOf(SmPolyVertex_TYPE));

  // set SmPLSPoint members
  m_pPointVertex = (!m_bSwapOrder) ? (SmPolyVertex*)pObj2 : (SmPolyVertex*)pObj1 ;
  m_vPoint       = m_pPointVertex->GetPoint();

  // Pass call along to parent SmPLSPointVertex::SolveIt
  SER(SmPLSPointVertex::SolveIt(pObj2,pObj1));

  // all done
  return SM_SUCCESS;

} // end SmPLSVertexVertex::SolveIt


/*******************************************************************//**
PURPOSE: Do local solve between point and edge(s).

NOTES:
***********************************************************************/
SmStatus SmPLSPointEdge::SolveIt
  (const SmObject *pObj1,
   const SmObject *pObj2)
{
  // Normally the point comes first and the edge second.  If
  // we want we can switch the order.
  SmSolution sSol;
  sSol.m_apObjects[0] = NULL;
  sSol.m_apObjects[1] = NULL;

  SmPolyEdge * pPolyEdge;
  ULONG lObjectCount = 0;
  if(!m_bSwapOrder)
    {
      pPolyEdge = SM_CAST_PTR(SmPolyEdge,pObj2); NER(pPolyEdge);
      if(m_pPointVertex)
        {
          // If the point represents a vertex then save it in list
          sSol.m_apObjects[lObjectCount++] = SM_REINTERPRET_CAST(SmObject*,
                                             SM_CONST_CAST(SmPolyVertex*,m_pPointVertex));
        }
      sSol.m_apObjects[lObjectCount++] = SM_REINTERPRET_CAST(SmObject*,
                                         SM_CONST_CAST(SmPolyEdge*,pPolyEdge));
    }
  else
    {
      pPolyEdge = SM_CAST_PTR(SmPolyEdge,pObj1); NER(pPolyEdge);
      sSol.m_apObjects[lObjectCount++] = SM_REINTERPRET_CAST(SmObject*,
                                         SM_CONST_CAST(SmPolyEdge*,pPolyEdge));
      if(m_pPointVertex)
        {
          // If the point represents a vertex then save it in list
          sSol.m_apObjects[lObjectCount++] = SM_REINTERPRET_CAST(SmObject*,
                                             SM_CONST_CAST(SmPolyVertex*,m_pPointVertex));
        }
    }

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
  if(bDebugMe)
    {
      smgfx_SetLook(3,4, 0,0,1); pPolyEdge->Draw(); sm_GraphicsLoop();
      smgfx_SetLook(1,2, 1,0,0); m_vPoint.Draw(); sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
#endif

  SmPoint3d sPoint = m_vPoint;
  SmPoint3d sEdgeStart = pPolyEdge->GetStartPoint();
  SmPoint3d sEdgeEnd = pPolyEdge->GetEndPoint();
  if(m_rPolySolver.m_bProjectedOperation == TRUE)
    {
      SmVector3d sPlaneNorm = m_rPolySolver.m_cpOptVectors[0];
      sEdgeStart = sEdgeStart.ProjectToPlane(sPlaneNorm);
      sEdgeEnd = sEdgeEnd.ProjectToPlane(sPlaneNorm);
      sPoint = sPoint.ProjectToPlane(sPlaneNorm);
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe1 = FALSE;
      if(bDebugMe1)
        {
          SmVector3d sV = sEdgeEnd - sEdgeStart;

          smgfx_SetLook(3,4, 0,0,1);  sV.Draw(&sEdgeStart); sm_GraphicsLoop();
          smgfx_SetLook(1,2, 1,0,0); sPoint.Draw(); sm_GraphicsLoop();
          sm_GraphicsLoop();
        }
#endif
    }
  if(sEdgeStart.DistanceBetweenSquared(sEdgeEnd) < SM_EFF_ZERO_SQ)
    {
      return SM_SUCCESS; // no answers in this branch
    }

  double dParam  =  0.0 ;
  double dDistSq = -1.0 ;
  double dSign   =  1.0;

  switch (m_rPolySolver.m_eSolverOperation)
    {
      case SM_SO_MINIMIZE:
      case SM_SO_PROJECTED_MINIMIZE:
            {
              double dDistance;
              SER(smgu_SegmentPointDistance(sEdgeStart,sEdgeEnd,
                  sPoint,dDistance,dParam));
              if(dParam < SM_EFF_ZERO || dParam > 1.0-SM_EFF_ZERO)
                {
                  return SM_SUCCESS; // no answers in this branch
                }
              dDistSq = dDistance * dDistance;
              if(   dDistSq > m_rPolySolver.m_dBestAnswerSoFarSq
                 && dDistance - m_rPolySolver.m_d3dTolerance >
                    smos_Sqrt(m_rPolySolver.m_dBestAnswerSoFarSq))
                {
                  return SM_SUCCESS; // no answers in this branch
                }
            }
          break;

      case SM_SO_DIRECTED_MINIMIZE:
      case SM_SO_DIRECTED_MAXIMIZE:
      case SM_SO_SIGNED_DIRECTED_MINIMIZE:
      case SM_SO_3D_SIGNED_DIRECTED_MINIMIZE:
            {
              SmVector3d sDirectionVec;
              if(m_rPolySolver.m_eSolverOperation == SM_SO_3D_SIGNED_DIRECTED_MINIMIZE)
                {
                  sDirectionVec = m_rPolySolver.m_cpOptVectors[0];
                }
              else
                {
                  sDirectionVec = m_rPolySolver.m_cpOptVectors[1];
                }
              SmVector3d sEdgeVec = sEdgeEnd-sEdgeStart;
              double dParam2;
              if(   smgu_LineLineClosestPoint(sPoint,sDirectionVec,
                                              sEdgeStart,sEdgeVec,dParam2,dParam) != SM_SUCCESS
                 || dParam < SM_EFF_ZERO
                 || dParam > 1.0-SM_EFF_ZERO)
                {
                  return SM_SUCCESS; // no answers in this branch
                }
              SmPoint3d sRayPnt = sPoint + dParam2 * sDirectionVec;
              SmPoint3d sPntOnEdge = sEdgeStart + dParam * sEdgeVec;
              double dDistToRay = sRayPnt.DistanceBetween(sPntOnEdge);
              double dTol = m_rPolySolver.m_d3dTolerance + pPolyEdge->GetTolerance();
              if(dDistToRay > dTol)
                {
                  return SM_SUCCESS; // No answer if vertex is to far from ray
                }
              if(m_bSwapOrder) dParam2 = - dParam2;
              // The following assumes that the direction vector is unitized
              dDistSq = dParam2 * dParam2;
              if(m_rPolySolver.m_eOperationCategory == SM_OPERATION_SIGNED)
                {
                  if(dParam2 < 0.0) dSign = -1.0;
                }
              if(m_rPolySolver.m_eSolverOperation == SM_SO_DIRECTED_MAXIMIZE)
                {
                  if(dDistSq < m_rPolySolver.m_dBestAnswerSoFarSq)
                    {
                      if(smos_Sqrt(dDistSq) + m_rPolySolver.m_d3dTolerance <
                          smos_Sqrt(m_rPolySolver.m_dBestAnswerSoFarSq) )
                        {
                          return SM_SUCCESS; // no answers in this branch
                        }
                    }
                }
              else if(dSign*dDistSq > m_rPolySolver.m_dBestAnswerSoFarSq)
                {
                  double dBestAnswer = smos_Sqrt(smos_Fabs(m_rPolySolver.m_dBestAnswerSoFarSq));
                  if(m_rPolySolver.m_dBestAnswerSoFarSq < 0.0) dBestAnswer = - dBestAnswer;
                  if(dSign*smos_Sqrt(dDistSq) - m_rPolySolver.m_d3dTolerance > dBestAnswer)
                    {
                      return SM_SUCCESS; // no answers in this branch
                    }
                }
            }
            break;
      case SM_SO_INTERSECT:
            {
              double dDistance;
              SER(smgu_SegmentPointDistance(sEdgeStart,sEdgeEnd,
                  sPoint,dDistance,dParam));
              if(dDistance > m_rPolySolver.m_d3dTolerance)
                {
                  return SM_SUCCESS; // no answers in this branch
                }
              dDistSq = dDistance * dDistance;
            }
            break;

      case SM_SO_MAXIMIZE:
      case SM_SO_PROJECTED_MAXIMIZE:
            return SM_SUCCESS;

      case SM_SO_AT_DISTANCE:
            {
              double dDistance;
              if(sEdgeStart.DistanceBetweenSquared(sEdgeEnd) < SM_EFF_ZERO_SQ)
                {
                  return SM_SUCCESS; // no answers in this branch
                }
              SER(smgu_SegmentPointDistance(sEdgeStart,sEdgeEnd,
                  sPoint,dDistance,dParam));
              if(dParam < SM_EFF_ZERO || dParam > 1.0-SM_EFF_ZERO)
                {
                  return SM_SUCCESS; // no answers in this branch
                }
              dDistSq = dDistance * dDistance;
              if(dDistSq < m_rPolySolver.m_dAtDistanceSq)
                {
                  if(dDistance - m_rPolySolver.m_d3dTolerance <
                      smos_Sqrt(m_rPolySolver.m_dAtDistanceSq) )
                    {
                      return SM_SUCCESS; // no answers in this branch
                    }
                }
              if(dDistSq > m_rPolySolver.m_dAtDistanceSq)
                {
                  if(dDistance + m_rPolySolver.m_d3dTolerance >
                      smos_Sqrt(m_rPolySolver.m_dAtDistanceSq) )
                    {
                      return SM_SUCCESS; // no answers in this branch
                    }
                }
            }
            break;
      default:
            ERR(SM_ERR);  // not handled yet - no answers here
            return(SM_SUCCESS) ;

    } // end switch (m_rPolySolver.m_eSolverOperation)

  SmPoint3d sFoundPnt;
  SER(pPolyEdge->EvaluatePoint(dParam,sFoundPnt));

  ULONG lPointIndex = (!m_bSwapOrder) ? 0 : 3 ;

  // set output
  sSol.m_eSolutionType           = SM_ST_POLY_SINGLE_VALUE;
  sSol.m_lNumVariables           = 6;
  sSol.m_lNumObjects             = lObjectCount;
  sSol.m_vStart.m_dSolutionValue = dSign*smos_Sqrt(dDistSq);
  sSol.m_vStart[lPointIndex]     = m_vPoint.x;
  sSol.m_vStart[lPointIndex+1]   = m_vPoint.y;
  sSol.m_vStart[lPointIndex+2]   = m_vPoint.z;
  sSol.m_vStart[3 - lPointIndex] = sFoundPnt.x;
  sSol.m_vStart[4 - lPointIndex] = sFoundPnt.y;
  sSol.m_vStart[5 - lPointIndex] = sFoundPnt.z;

  // save output
  m_rPolySolver.AddSortedSolution(sSol,SM_SK_BY_SOLUTION_VALUE);

  return SM_SUCCESS;

} // end SmPLSPointEdge::SolveIt

/*******************************************************************//**
PURPOSE: Local solver for Vertex Edge.  Just invokes
   superclass after loading data.

NOTES:
***********************************************************************/
SmStatus SmPLSVertexEdge::SolveIt
  (const SmObject *pObj1,
   const SmObject *pObj2)
{
  SM_ASSERT(pObj1 != NULL && pObj2 != NULL) ;

#ifdef SM_DEBUG_CODE
  if(!m_bSwapOrder) { SM_ASSERT(pObj1->IsKindOf(SmPolyVertex_TYPE));
                       SM_ASSERT(pObj2->IsKindOf(SmPolyEdge_TYPE));
                     }
  else               { SM_ASSERT(pObj1->IsKindOf(SmPolyEdge_TYPE));
                       SM_ASSERT(pObj2->IsKindOf(SmPolyVertex_TYPE));
                     }
#endif // SM_DEBUG_CODE

  // set ancester SmPLSPoint members
  m_pPointVertex = (!m_bSwapOrder) ? (SmPolyVertex*)pObj1 : (SmPolyVertex*)pObj2 ;
  m_vPoint       = m_pPointVertex->GetPoint();

  // pass the call along to parent SmPLSPointEdge
  SER(SmPLSPointEdge::SolveIt(pObj1,pObj2));

  // all done
  return SM_SUCCESS;

} // end SmPLSVertexEdge::SolveIt


/*******************************************************************//**
PURPOSE: Do local solve between point and a face.

NOTES:
***********************************************************************/
SmStatus SmPLSPointFace::SolveIt
  (const SmObject *pObj1,
   const SmObject *pObj2)
{
  // Normally the point comes first and the face second.  If
  // we want we can switch the order.
  SmSolution sSol;
  sSol.m_apObjects[0] = NULL;
  sSol.m_apObjects[1] = NULL;

  SmPolyFace * pPolyFace;
  ULONG lObjectCount = 0;
  if(!m_bSwapOrder)
    {
      pPolyFace = SM_CAST_PTR(SmPolyFace,pObj2); NER(pPolyFace);
      if(m_pPointVertex)
        {
          // If the point represents a vertex then save it in list
          sSol.m_apObjects[lObjectCount++] = SM_REINTERPRET_CAST(SmObject*,
                                             SM_CONST_CAST(SmPolyVertex*,m_pPointVertex));
        }
      sSol.m_apObjects[lObjectCount++] = SM_REINTERPRET_CAST(SmObject*,
                                         SM_CONST_CAST(SmPolyFace*,pPolyFace));
    }
  else
    {
      pPolyFace = SM_CAST_PTR(SmPolyFace,pObj1); NER(pPolyFace);
      sSol.m_apObjects[lObjectCount++] = SM_REINTERPRET_CAST(SmObject*,
                                         SM_CONST_CAST(SmPolyFace*,pPolyFace));
      if(m_pPointVertex)
        {
          // If the point represents a vertex then save it in list
          sSol.m_apObjects[lObjectCount++] = SM_REINTERPRET_CAST(SmObject*,
                                             SM_CONST_CAST(SmPolyVertex*,m_pPointVertex));
        }
    }

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
  if(bDebugMe)
    {
      smgfx_SetLook(1,2, 0,0,1); pPolyFace->Draw(); sm_GraphicsLoop();
      smgfx_SetLook(1,2, 1,0,0); m_vPoint.Draw(); sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
#endif

  SmPolyLoop * pPolyLoop = pPolyFace->GetOuterPolyLoop();
  SmPoint3d sFaceOrigin = pPolyLoop->GetFirstPolyEdge()->GetStartPoint();
  SmVector3d sFaceNormal = pPolyFace->GetNormal();

  double dSign   = 1.0;
  double dDistSq = -1.0 ;
  SmPoint3d sFoundPoint;

  switch (m_rPolySolver.m_eSolverOperation)
    {
      case SM_SO_MINIMIZE:
            {
              SER(smgu_PointProjectToPlane(m_vPoint,
                  sFaceOrigin,sFaceNormal,sFoundPoint));
              dDistSq = m_vPoint.DistanceBetweenSquared(sFoundPoint);
              if(   dDistSq > m_rPolySolver.m_dBestAnswerSoFarSq
                 && smos_Sqrt(dDistSq) - m_rPolySolver.m_d3dTolerance >
                    smos_Sqrt(m_rPolySolver.m_dBestAnswerSoFarSq))
                {
                  return SM_SUCCESS; // no answers in this branch
                }

              SmBoolean bInside = FALSE;
              SmBoolean b3DTest = TRUE;
              SER(pPolyFace->PointInPolygon(sFoundPoint,bInside,b3DTest));
              if(!bInside)
                {
                  return SM_SUCCESS;
                }
            }
          break;

      case SM_SO_PROJECTED_MINIMIZE:
      case SM_SO_DIRECTED_MINIMIZE:
      case SM_SO_3D_SIGNED_DIRECTED_MINIMIZE:
            {
              SmVector3d sProjctionVec = m_rPolySolver.m_cpOptVectors[0];
              double dParam;
              if(smgu_LinePlaneIntersect(m_vPoint,sProjctionVec,
                  sFaceOrigin,sFaceNormal,dParam) != SM_SUCCESS)
                {
                  return SM_SUCCESS;
                }
              sFoundPoint = m_vPoint + dParam * sProjctionVec;
              // Consider only points that are inside of the polygon
              SmBoolean bInside = FALSE;
              SmBoolean b3DTest = TRUE;
              SER(pPolyFace->PointInPolygon(sFoundPoint,bInside,b3DTest));
              if(!bInside)
                {
                  return SM_SUCCESS;
                }
              dDistSq = 0.0;
              if(m_rPolySolver.m_eSolverOperation == SM_SO_3D_SIGNED_DIRECTED_MINIMIZE)
                {
                  if(m_bSwapOrder) dParam = - dParam;
                  dDistSq = dParam * dParam;
                  if(dParam < 0.0) dSign = -1.0;
                  if(dSign*dDistSq > m_rPolySolver.m_dBestAnswerSoFarSq)
                    {
                      double dBestAnswer = smos_Sqrt(smos_Fabs(m_rPolySolver.m_dBestAnswerSoFarSq));
                      if(m_rPolySolver.m_dBestAnswerSoFarSq < 0.0) dBestAnswer = - dBestAnswer;
                      if(dSign*smos_Sqrt(dDistSq) - m_rPolySolver.m_d3dTolerance > dBestAnswer)
                        {
                          return SM_SUCCESS; // no answers in this branch
                        }
                    }
                }
            }
            break;

      case SM_SO_SIGNED_DIRECTED_MINIMIZE:
      case SM_SO_MAXIMIZE:
      case SM_SO_PROJECTED_MAXIMIZE:
      case SM_SO_DIRECTED_MAXIMIZE:
          return SM_SUCCESS; // no answers in this branch

      case SM_SO_AT_DISTANCE:
            {
              SER(smgu_PointProjectToPlane(m_vPoint,
                  sFaceOrigin,sFaceNormal,sFoundPoint));
              dDistSq = m_vPoint.DistanceBetweenSquared(sFoundPoint);
              if(dDistSq < m_rPolySolver.m_dAtDistanceSq)
                {
                  if(smos_Sqrt(dDistSq) - m_rPolySolver.m_d3dTolerance <
                      smos_Sqrt(m_rPolySolver.m_dAtDistanceSq) )
                    {
                      return SM_SUCCESS; // no answers in this branch
                    }
                }
              if(dDistSq > m_rPolySolver.m_dAtDistanceSq)
                {
                  if(smos_Sqrt(dDistSq) + m_rPolySolver.m_d3dTolerance >
                      smos_Sqrt(m_rPolySolver.m_dAtDistanceSq) )
                    {
                      return SM_SUCCESS; // no answers in this branch
                    }
                }

              SmBoolean bInside = FALSE;
              SmBoolean b3DTest = TRUE;
              SER(pPolyFace->PointInPolygon(sFoundPoint,bInside,b3DTest));
              if(!bInside)
                {
                  return SM_SUCCESS;
                }
            }
            break;

      case SM_SO_INTERSECT:
            {
              SER(smgu_PointProjectToPlane(m_vPoint,
                  sFaceOrigin,sFaceNormal,sFoundPoint));
              dDistSq = m_vPoint.DistanceBetweenSquared(sFoundPoint);
              if(smos_Sqrt(dDistSq) > m_rPolySolver.m_d3dTolerance)
                {
                  return SM_SUCCESS; // no answers in this branch
                }
              SmBoolean bInside = FALSE;
              SmBoolean b3DTest = TRUE;
              SER(pPolyFace->PointInPolygon(sFoundPoint,bInside,b3DTest));
              if(!bInside)
                {
                  return SM_SUCCESS;
                }
            }
          break;

      case SM_SO_NORMALIZE:
          return SM_SUCCESS;
      default:
          ERR(SM_ERR);  // not handled yet - no answer here
          return SM_SUCCESS;

    } // end switch (m_rPolySolver.m_eSolverOperation)

  ULONG lPointIndex = (!m_bSwapOrder) ? 0 : 3 ;

  // set output
  sSol.m_eSolutionType           = SM_ST_POLY_SINGLE_VALUE;
  sSol.m_lNumVariables           = 6;
  sSol.m_lNumObjects             = lObjectCount;
  sSol.m_vStart.m_dSolutionValue = dSign*smos_Sqrt(dDistSq);;
  sSol.m_vStart[lPointIndex]     = m_vPoint.x;
  sSol.m_vStart[lPointIndex+1]   = m_vPoint.y;
  sSol.m_vStart[lPointIndex+2]   = m_vPoint.z;
  sSol.m_vStart[3 - lPointIndex] = sFoundPoint.x;
  sSol.m_vStart[4 - lPointIndex] = sFoundPoint.y;
  sSol.m_vStart[5 - lPointIndex] = sFoundPoint.z;

  // save output
  m_rPolySolver.AddSortedSolution(sSol,SM_SK_BY_SOLUTION_VALUE);

  return SM_SUCCESS;

} // end SmPLSPointFace::SolveIt


/*******************************************************************//**
PURPOSE: Local solver for Vertex Face.  Just invokes
   superclass after loading data.

NOTES:
***********************************************************************/
SmStatus SmPLSVertexFace::SolveIt
  (const SmObject *pObj1,
   const SmObject *pObj2)
{
  SM_ASSERT(   pObj1 != NULL
            && pObj2 != NULL);
#ifdef SM_DEBUG_CODE
  if(!m_bSwapOrder) { SM_ASSERT(pObj1->IsKindOf(SmPolyVertex_TYPE));
                      SM_ASSERT(pObj2->IsKindOf(SmPolyFace_TYPE));
                    }
  else              { SM_ASSERT(pObj1->IsKindOf(SmPolyFace_TYPE));
                      SM_ASSERT(pObj2->IsKindOf(SmPolyVertex_TYPE));
                    }
#endif // SM_DEBUG_CODE

  // set ancester SmPLSPoint members
  m_pPointVertex = (!m_bSwapOrder) ? (SmPolyVertex*)pObj1 : (SmPolyVertex*)pObj2 ;
  m_vPoint       = m_pPointVertex->GetPoint();

  // pass call along to parent
  SER(SmPLSPointFace::SolveIt(pObj1,pObj2));

  // all done
  return SM_SUCCESS;

} // end SmPLSVertexFace::SolveIt


/*******************************************************************//**
PURPOSE: Local solver for Edge Edge.

NOTES:
***********************************************************************/
SmStatus SmPLSEdgeEdge::SolveIt
  (const SmObject *pObj1,
   const SmObject *pObj2)
{
  SM_ASSERT(pObj1 != NULL && pObj2 != NULL);
  SmPolyEdge * pPolyEdge1 = SM_CAST_PTR(SmPolyEdge,pObj1); NER(pPolyEdge1);
  SmPolyEdge * pPolyEdge2 = SM_CAST_PTR(SmPolyEdge,pObj2); NER(pPolyEdge2);

  SmSolution sSol;
  sSol.m_apObjects[0] = NULL;
  sSol.m_apObjects[1] = NULL;

  ULONG lObjectCount = 0;
  if(!m_bSwapOrder)
    {
      sSol.m_apObjects[lObjectCount++] = SM_REINTERPRET_CAST(SmObject*,
                                         SM_CONST_CAST(SmPolyEdge*,pPolyEdge1));
      sSol.m_apObjects[lObjectCount++] = SM_REINTERPRET_CAST(SmObject*,
                                         SM_CONST_CAST(SmPolyEdge*,pPolyEdge2));
    }
  else
    {
      sSol.m_apObjects[lObjectCount++] = SM_REINTERPRET_CAST(SmObject*,
                                         SM_CONST_CAST(SmPolyEdge*,pPolyEdge2));
      sSol.m_apObjects[lObjectCount++] = SM_REINTERPRET_CAST(SmObject*,
                                         SM_CONST_CAST(SmPolyEdge*,pPolyEdge1));
    }

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
  if(bDebugMe)
    {
      smgfx_SetLook(3,4, 0,0,1); pPolyEdge1->Draw(); sm_GraphicsLoop();
      smgfx_SetLook(3,4, 1,0,0); pPolyEdge2->Draw(); sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
#endif

  double *pdBestAnswer = NULL;
  double dBestAnswer;
  m_rPolySolver.GetBestAnswerSoFar(dBestAnswer, pdBestAnswer);

  SmPoint3d sLineStart1 = pPolyEdge1->GetStartPoint();
  SmPoint3d sLineEnd1 = pPolyEdge1->GetEndPoint();
  SmPoint3d sLineStart2 = pPolyEdge2->GetStartPoint();
  SmPoint3d sLineEnd2 = pPolyEdge2->GetEndPoint();
  if(m_rPolySolver.m_bProjectedOperation == TRUE)
    {
      SmVector3d sPlaneNorm = m_rPolySolver.m_cpOptVectors[0];
      sLineStart1 = sLineStart1.ProjectToPlane(sPlaneNorm);
      sLineEnd1 = sLineEnd1.ProjectToPlane(sPlaneNorm);
      sLineStart2 = sLineStart2.ProjectToPlane(sPlaneNorm);
      sLineEnd2 = sLineEnd2.ProjectToPlane(sPlaneNorm);
    }
  SmVector3d sLineVec1 = sLineEnd1 - sLineStart1;
  SmVector3d sLineVec2 = sLineEnd2 - sLineStart2;
  if(   sLineVec1.LengthSquared() < SM_EFF_ZERO_SQ
     || sLineVec2.LengthSquared() < SM_EFF_ZERO_SQ)
    {
      return SM_SUCCESS;
    }

  double dParam1, dParam2;
  SmPoint3d sPnt1, sPnt2;
  double dDistSq = -1.0;
  double dSign   = 1.0;

  switch (m_rPolySolver.m_eSolverOperation)
    {
      case SM_SO_MINIMIZE:
      case SM_SO_3D_SIGNED_DIRECTED_MINIMIZE:
      case SM_SO_INTERSECT:
            {
              SmVector3d sDirectionVec;
              if(m_rPolySolver.m_eSolverOperation == SM_SO_3D_SIGNED_DIRECTED_MINIMIZE)
                {
                  sDirectionVec = m_rPolySolver.m_cpOptVectors[0];
                }
              else
                {
                  sDirectionVec = sLineVec1 * sLineVec2;
                }
              if(sDirectionVec.LengthSquared() < SM_EFF_ZERO_SQ) return SM_SUCCESS;

              SmVector3d sPlaneV1 = sDirectionVec * sLineVec1;
              SmVector3d sPlaneV2 = sDirectionVec * sLineVec2;
              SER(sPlaneV1.Unitize());
              SER(sPlaneV2.Unitize());
              if(   smgu_LinePlaneIntersect(sLineStart1,sLineVec1,sLineStart2,
                                            sPlaneV2,dParam1) != SM_SUCCESS
                 || smgu_LinePlaneIntersect(sLineStart2,sLineVec2,sLineStart1,
                                            sPlaneV1,dParam2) != SM_SUCCESS
                 || dParam1 < SM_EFF_ZERO
                 || dParam1 > 1.0-SM_EFF_ZERO
                 || dParam2 < SM_EFF_ZERO
                 || dParam2 > 1.0-SM_EFF_ZERO)
                {
                  return SM_SUCCESS; // no answers in this branch
                }
              sPnt1 = sLineStart1 + dParam1 * sLineVec1;
              sPnt2 = sLineStart2 + dParam2 * sLineVec2;
              SmVector3d sDistVec = sPnt2 - sPnt1;
              if(m_bSwapOrder) sDistVec = -sDistVec;
              dDistSq = sDistVec.LengthSquared();

              switch (m_rPolySolver.m_eSolverOperation)
                {
                  case SM_SO_INTERSECT:
                      if(smos_Sqrt(dDistSq) > m_rPolySolver.m_d3dTolerance)
                        {
                          return SM_SUCCESS; // no answers in this branch
                        }
                      break;
                  case SM_SO_3D_SIGNED_DIRECTED_MINIMIZE:
                      if(sDistVec.Dot(m_rPolySolver.m_cpOptVectors[0]) < 0)
                        {
                          dSign = -1.0;
                        }
                      // Continue on!!
                  default:
                      if(dSign*dDistSq > m_rPolySolver.m_dBestAnswerSoFarSq)
                        {
                          dBestAnswer = smos_Sqrt(smos_Fabs(m_rPolySolver.m_dBestAnswerSoFarSq));
                          if(m_rPolySolver.m_dBestAnswerSoFarSq < 0.0) dBestAnswer = - dBestAnswer;
                          if(dSign*smos_Sqrt(dDistSq) - m_rPolySolver.m_d3dTolerance > dBestAnswer)
                            {
                              return SM_SUCCESS; // no answers in this branch
                            }
                        }
                }
            }
            break;

      case SM_SO_PROJECTED_MINIMIZE:
      case SM_SO_DIRECTED_MINIMIZE:
      case SM_SO_SIGNED_DIRECTED_MINIMIZE:
            {
              ULONG lNumInt;
              SmPoint3d aPoints[3];
              SER(smgu_SegmentSegmentIntersect(sLineStart1,sLineEnd1,sLineStart2,
                  sLineEnd2,m_rPolySolver.m_d3dTolerance,lNumInt,aPoints));
              if(lNumInt == 0)
                {
                  return SM_SUCCESS; // no answers in this branch
                }
              sPnt1 = aPoints[0]; sPnt2 = aPoints[0];
            }
            break;

      case SM_SO_MAXIMIZE:
      case SM_SO_PROJECTED_MAXIMIZE:
      case SM_SO_DIRECTED_MAXIMIZE:
            return SM_SUCCESS; // no answers in this branch

      case SM_SO_AT_DISTANCE:
            {
              SmVector3d sDirectionVec = sLineVec1 * sLineVec2;
              if(sDirectionVec.LengthSquared() < SM_EFF_ZERO_SQ) return SM_SUCCESS;

              SmVector3d sPlaneV1 = sDirectionVec * sLineVec1;
              SmVector3d sPlaneV2 = sDirectionVec * sLineVec2;
              SER(sPlaneV1.Unitize());
              SER(sPlaneV2.Unitize());
              if(smgu_LinePlaneIntersect(sLineStart1,sLineVec1,sLineStart2,
                                         sPlaneV2,dParam1) != SM_SUCCESS
                 || smgu_LinePlaneIntersect(sLineStart2,sLineVec2,sLineStart1,
                                          sPlaneV1,dParam2) != SM_SUCCESS
                 || dParam1 < SM_EFF_ZERO
                 || dParam1 > 1.0-SM_EFF_ZERO
                 || dParam2 < SM_EFF_ZERO
                 || dParam2 > 1.0-SM_EFF_ZERO)
                {
                  return SM_SUCCESS; // no answers in this branch
                }
              sPnt1 = sLineStart1 + dParam1 * sLineVec1;
              sPnt2 = sLineStart2 + dParam2 * sLineVec2;
              SmVector3d sDistVec = sPnt2 - sPnt1;
              if(m_bSwapOrder) sDistVec = -sDistVec;
              dDistSq = sDistVec.LengthSquared();
              if(dDistSq < m_rPolySolver.m_dAtDistanceSq)
                {
                  if(smos_Sqrt(dDistSq) - m_rPolySolver.m_d3dTolerance <
                      smos_Sqrt(m_rPolySolver.m_dAtDistanceSq) )
                        {
                      return SM_SUCCESS; // no answers in this branch
                    }
                }
              if(dDistSq > m_rPolySolver.m_dAtDistanceSq)
                {
                  if(smos_Sqrt(dDistSq) + m_rPolySolver.m_d3dTolerance >
                      smos_Sqrt(m_rPolySolver.m_dAtDistanceSq) )
                    {
                      return SM_SUCCESS; // no answers in this branch
                    }
                }
            }
            break;

      case SM_SO_NORMALIZE:
          return SM_SUCCESS;

      default:
          ERR(SM_ERR);  // not handled yet - no answers here
          return SM_SUCCESS;
    }

  // set output
  sSol.m_eSolutionType           = SM_ST_POLY_SINGLE_VALUE;
  sSol.m_lNumVariables           = 6;
  sSol.m_lNumObjects             = lObjectCount;
  SM_ASSERT(dDistSq != -1.0) ;
  sSol.m_vStart.m_dSolutionValue = dSign*smos_Sqrt(dDistSq);
  sSol.m_vStart[0]               = sPnt1.x;
  sSol.m_vStart[1]               = sPnt1.y;
  sSol.m_vStart[2]               = sPnt1.z;
  sSol.m_vStart[3]               = sPnt2.x;
  sSol.m_vStart[4]               = sPnt2.y;
  sSol.m_vStart[5]               = sPnt2.z;

  // save output
  m_rPolySolver.AddSortedSolution(sSol,SM_SK_BY_SOLUTION_VALUE);

  return SM_SUCCESS;

} // end SmPLSEdgeEdge::SolveIt

/*******************************************************************//**
PURPOSE: Do local solve between an edge and a face.

NOTES:
***********************************************************************/
SmStatus SmPLSEdgeFace::SolveIt
  (const SmObject *pObj1,
   const SmObject *pObj2)
{
  // Normally the edge comes first and the face second.  If
  // we want we can switch the order.
  SmSolution sSol;
  sSol.m_apObjects[0] = NULL;
  sSol.m_apObjects[1] = NULL;

  SmPolyEdge * pPolyEdge;
  SmPolyFace * pPolyFace;
  ULONG lObjectCount = 0;
  if(!m_bSwapOrder)
    {
      pPolyEdge = SM_CAST_PTR(SmPolyEdge,pObj1); NER(pPolyEdge);
      pPolyFace = SM_CAST_PTR(SmPolyFace,pObj2); NER(pPolyFace);
      sSol.m_apObjects[lObjectCount++] = SM_REINTERPRET_CAST(SmObject*,
                                         SM_CONST_CAST(SmPolyEdge*,pPolyEdge));
      sSol.m_apObjects[lObjectCount++] = SM_REINTERPRET_CAST(SmObject*,
                                         SM_CONST_CAST(SmPolyFace*,pPolyFace));
    }
  else
    {
      pPolyFace = SM_CAST_PTR(SmPolyFace,pObj1); NER(pPolyFace);
      pPolyEdge = SM_CAST_PTR(SmPolyEdge,pObj2); NER(pPolyEdge);
      sSol.m_apObjects[lObjectCount++] = SM_REINTERPRET_CAST(SmObject*,
                                         SM_CONST_CAST(SmPolyFace*,pPolyFace));
      sSol.m_apObjects[lObjectCount++] = SM_REINTERPRET_CAST(SmObject*,
                                         SM_CONST_CAST(SmPolyEdge*,pPolyEdge));
    }

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
  if(bDebugMe)
    {
      smgfx_SetLook(3,4, 0,0,1); pPolyFace->Draw(); sm_GraphicsLoop();
      smgfx_SetLook(3,4, 1,0,0); pPolyEdge->Draw(); sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
#endif

  double dDistSq = 0.0;
  SmPoint3d sFoundPoint;

  switch (m_rPolySolver.m_eSolverOperation)
    {
      case SM_SO_MINIMIZE:
      case SM_SO_3D_SIGNED_DIRECTED_MINIMIZE:
      case SM_SO_INTERSECT:
          {
              // Determine if the edge intersected with the face
              SmPoint3d sEdgeStart = pPolyEdge->GetStartPoint();
              SmVector3d sEdgeVec = pPolyEdge->GetEndPoint()-sEdgeStart;
              SmVector3d sFaceNormal = pPolyFace->GetNormal();
              SmPolyLoop * pPolyLoop = pPolyFace->GetOuterPolyLoop();
              SmPoint3d sFaceOrigin = pPolyLoop->GetFirstPolyEdge()->GetStartPoint();
              double dParam;
              if(smgu_LinePlaneIntersect(sEdgeStart,sEdgeVec,
                                         sFaceOrigin,sFaceNormal,dParam) != SM_SUCCESS
                 || dParam < SM_EFF_ZERO
                 || dParam > 1.0-SM_EFF_ZERO)
                {
                  return SM_SUCCESS;
                }
              sFoundPoint = sEdgeStart + dParam * sEdgeVec;

              SmBoolean bInside = FALSE;
              SmBoolean b3DTest = TRUE;
              SER(pPolyFace->PointInPolygon(sFoundPoint,bInside,b3DTest));
              if(!bInside)
                {
                  return SM_SUCCESS;
                }
          }
          break;

      case SM_SO_PROJECTED_MINIMIZE:
      case SM_SO_DIRECTED_MINIMIZE:
      case SM_SO_SIGNED_DIRECTED_MINIMIZE:
      case SM_SO_MAXIMIZE:
      case SM_SO_PROJECTED_MAXIMIZE:
      case SM_SO_DIRECTED_MAXIMIZE:
          return SM_SUCCESS; // no answers in this branch

      case SM_SO_AT_DISTANCE:
      case SM_SO_NORMALIZE:
          return SM_SUCCESS;
      default:
          ERR(SM_ERR);  // not handled yet
    } // end switch (m_rPolySolver.m_eSolverOperation)

  ULONG lPointIndex = (!m_bSwapOrder) ? 0 : 3 ;

  // set output
  sSol.m_eSolutionType           = SM_ST_POLY_SINGLE_VALUE;
  sSol.m_lNumVariables           = 6;
  sSol.m_lNumObjects             = lObjectCount;
  sSol.m_vStart.m_dSolutionValue = smos_Sqrt(dDistSq);
  sSol.m_vStart[lPointIndex]     = sFoundPoint.x;
  sSol.m_vStart[lPointIndex+1]   = sFoundPoint.y;
  sSol.m_vStart[lPointIndex+2]   = sFoundPoint.z;
  sSol.m_vStart[3 - lPointIndex] = sFoundPoint.x;
  sSol.m_vStart[4 - lPointIndex] = sFoundPoint.y;
  sSol.m_vStart[5 - lPointIndex] = sFoundPoint.z;

  // save output
  m_rPolySolver.AddSortedSolution(sSol,SM_SK_BY_SOLUTION_VALUE);

  return SM_SUCCESS;

} // end SmPLSEdgeFace::SolveIt

/*******************************************************************//**
PURPOSE: Constructor for the Poly solver which accepts an array
    of vectors.

Example:
      This constructor should be used for the following solver operations:
    SM_SO_INTERSECT,          // Intersect the objects
    SM_SO_MINIMIZE,           // Minimize distance between the objects
    SM_SO_MAXIMIZE,           // Maximize distance between the objects
    SM_SO_NORMALIZE,          // Find points where the objects are parallel to each other
    SM_SO_AT_DISTANCE,        // Find points on an object at a given distance from another object
    SM_SO_FIND,               // Just looking for something
    SM_SO_3D_SIGNED_DIRECTED_MINIMIZE, // Minimize 3D signed distance along a given vector
                                    // Finds the largest magnitude negative value
    SM_SO_SIGNED_DIRECTED_MINIMIZE, // Minimize 2D projected signed distance along a given vector
                                    // Finds the largest magnitude negative value
    SM_SO_DIRECTED_MINIMIZE,  // Minimize 2D projected absolute distance along a given vector
    SM_SO_DIRECTED_MAXIMIZE,  // Maximize 2D projected absolute distance along a given vector
    SM_SO_PROJECTED_MINIMIZE, // Minimize 2D distance relative to a projection direction
    SM_SO_PROJECTED_MAXIMIZE, // Maximize 2D distance relative to a projection direction
    SM_SO_ANGLE_MINIMIZE,     // Find minimum absolute angle to a plane relative to point on the plane
    SM_SO_SIGNED_ANGLE_MINIMIZE,  // Find minimum angle relative to a 0 to 180 and 0 to -180
                                   // angular measurement (positive is counter clockwise relative to
                                   // the reference vector in the plane).  Any negative angle will
                                   // always override a positive angle.  A larger negative angle
                                   // will override a smaller negative value (i.e. -130 overrides
                                   // -100).
    SM_SO_RAYFIRE,            // Minimize intersections from a point along a given vector in 3D


NOTES:
***********************************************************************/
SmPolySolver::SmPolySolver
  (SmSolverOperationType   eSolverOperation,
   SmSolutionRequestedType eSolutionRequested,
   double                  d3dTolerance,
   double                  dBestAnswerSoFarSq,
   const SmVector3d      * cpOptVectors,
   SmSolutionArray       & rSolutions)
{
  // Only handle minimize for now
  SM_ASSERT(   eSolverOperation == SM_SO_INTERSECT
            || eSolverOperation == SM_SO_MINIMIZE
            || eSolverOperation == SM_SO_MAXIMIZE
            || eSolverOperation == SM_SO_AT_DISTANCE
            || eSolverOperation == SM_SO_SIGNED_DIRECTED_MINIMIZE
            || eSolverOperation == SM_SO_3D_SIGNED_DIRECTED_MINIMIZE
            || eSolverOperation == SM_SO_DIRECTED_MINIMIZE
            || eSolverOperation == SM_SO_DIRECTED_MAXIMIZE
            || eSolverOperation == SM_SO_PROJECTED_MINIMIZE
            || eSolverOperation == SM_SO_PROJECTED_MAXIMIZE
            || eSolverOperation == SM_SO_ANGLE_MINIMIZE
            || eSolverOperation == SM_SO_SIGNED_ANGLE_MINIMIZE);

  m_eSolverOperation    = eSolverOperation;
  m_eSolutionRequested  = eSolutionRequested;
  m_d3dTolerance        = d3dTolerance;
  m_dBestAnswerSoFarSq  = dBestAnswerSoFarSq;
  m_pSolutions          = &rSolutions;
  m_cpOptVectors        = cpOptVectors;
  m_bProjectedOperation = FALSE ;
  m_eOperationCategory  = SM_OPERATION_OTHER ;

  switch (m_eSolverOperation)
   {
     case SM_SO_DIRECTED_MINIMIZE          :
     case SM_SO_PROJECTED_MINIMIZE         : m_bProjectedOperation = TRUE;
     case SM_SO_MINIMIZE                   :
     case SM_SO_ANGLE_MINIMIZE             : m_eOperationCategory  = SM_OPERATION_MINIMIZE;
                                             break;
     case SM_SO_DIRECTED_MAXIMIZE          :
     case SM_SO_PROJECTED_MAXIMIZE         : m_bProjectedOperation = TRUE;
     case SM_SO_MAXIMIZE                   : m_eOperationCategory  = SM_OPERATION_MAXIMIZE;
                                             break;
     case SM_SO_SIGNED_DIRECTED_MINIMIZE   :
     case SM_SO_SIGNED_ANGLE_MINIMIZE      : m_bProjectedOperation = TRUE;
                                             m_eOperationCategory  = SM_OPERATION_SIGNED;
                                             break;
     case SM_SO_3D_SIGNED_DIRECTED_MINIMIZE: m_eOperationCategory  = SM_OPERATION_SIGNED;
                                             break;
     default:                                m_eOperationCategory  = SM_OPERATION_OTHER;
                                             m_bProjectedOperation = FALSE;
                                             break;
   }
} // end SmPolySolver::SmPolySolver constructor

/*******************************************************************//**
PURPOSE: This method solves a particular operation for a PolyBrep and a
    point.  Valid operations include - SM_SO_MINIMIZE, SM_SO_MAXIMIZE,
    SM_SO_INTERSECT, SM_SO_NORMALIZE, SM_SO_PROJECTED_MINIMIZE,
    SM_SO_PROJECTED_MAXIMIZE, SM_SO_RAYFIRE. SM_SO_3D_SIGNED_DIRECTED_MINIMIZE.
    Note that when doing minimization and maximization a very large
    (SM_BIG_DOUBLE) or a very small (0.0) value should be set to
    dBestAnswerSoFarSq unless you already have a limiting value.
    In otherwords, if you wish to find a minimum only if it is less
    than 0.1 then you should set dBestAnswerSoFarSq = 0.1 * 0.1.
    The solutions array may or may not have existing solutions
    already registered with it.

NOTES:
***********************************************************************/
SmStatus SmPolySolver::CachePointSolve
  (SmBrepCache           * pBC,
   const SmPoint3d       & crPoint,
   SmSolverOperationType   eSolverOperation,
   SmSolutionRequestedType eSolutionRequested,
   double                  d3dTolerance,
   double                  dBestAnswerSoFarSq,
   const SmVector3d      * cpOptVectors,
   SmSolutionArray       & rSolutions)
{
  if(eSolverOperation != SM_SO_MINIMIZE)
    {
      SER(SM_ERR_INVALID_INPUT);
    }

  NER(pBC);

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
  if(bDebugMe)
    {
      smgfx_Erase();
      smgfx_SetLook(3,4, 1,0,0); crPoint.Draw(); sm_GraphicsLoop();
      smgfx_SetLook(3,4, 0,0,0); pBC->Draw(); sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
#endif

  rSolutions.ReSet();

  SmPolySolver sPS(eSolverOperation,eSolutionRequested,d3dTolerance,
                   dBestAnswerSoFarSq,cpOptVectors,rSolutions);

  SmTreeNode sTreeNode;
  SmObjectList sObjectList;
  SmTree sPointTree(crPoint,0,&sTreeNode,&sObjectList);

  if(eSolverOperation != SM_SO_NORMALIZE)
    {
      // First let's compare the point with the vertices of the PolyBrep
      SmTree *pVertTree = pBC->GetVertexTree();
      SmPLSPointVertex sPSLPV(sPS);
      sPSLPV.SetPoint(crPoint);
      SER(sPS.SolveTrees(sPointTree,*pVertTree,sPSLPV));

      // Now compare the point with the edges of the PolyBrep
      SmTree *pEdgeTree = pBC->GetCurveTree();
      SmPLSPointEdge sPLSPE(sPS);
      sPLSPE.SetPoint(crPoint);
      SER(sPS.SolveTrees(sPointTree,*pEdgeTree,sPLSPE));
    }

  // Now compare the point with the faces of the PolyBrep
  SmTree *pFaceTree = pBC->GetSurfaceTree();
  SmPLSPointFace sPLSPF(sPS);
  sPLSPF.SetPoint(crPoint);
  SER(sPS.SolveTrees(sPointTree,*pFaceTree,sPLSPF));

  return SM_SUCCESS;

} // end SmPolySolver::CachePointSolve

/*******************************************************************//**
PURPOSE: This method solves a particular operation for a PolyBrep and a
    plane.  Possible operations include - SM_SO_MINIMIZE, SM_SO_MAXIMIZE,
    SM_SO_INTERSECT, SM_SO_NORMALIZE, SM_SO_PROJECTED_MINIMIZE,
    SM_SO_PROJECTED_MAXIMIZE, SM_SO_RAYFIRE. SM_SO_3D_SIGNED_DIRECTED_MINIMIZE.
    Note that when doing minimization and maximization a very large
    (SM_BIG_DOUBLE) or a very small (0.0) value should be set to
    dBestAnswerSoFarSq unless you already have a limiting value.
    In otherwords, if you wish to find a minimum only if it is less
    than 0.1 then you should set dBestAnswerSoFarSq = 0.1 * 0.1.
    The solutions array may or may not have existing solutions
    already registered with it.

NOTES: Currently, only SM_SO_MINIMIZE is implemented and each
    solution contained a signed distance to the plane.

***********************************************************************/
SmStatus SmPolySolver::PolyBrepPlaneSolve
  (const SmPolyBrep      * cpPolyBrep,
   const SmPoint3d       & crPlanePoint,
   const SmVector3d      & crPlaneNormal,
   SmSolverOperationType   eSolverOperation,
   SmSolutionRequestedType eSolutionRequested,
   double                  d3dTolerance,
   double                  dBestAnswerSoFarSq,
   const SmVector3d      * cpOptVectors,
   SmSolutionArray       & rSolutions)
{
  // current limitation - only SM_SO_MINIMIZE
  if(eSolverOperation != SM_SO_MINIMIZE)
    {
      SER(SM_ERR_INVALID_INPUT);
    }

  // check input
  NER(cpPolyBrep);

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
  if(bDebugMe)
    {
      smgfx_Erase();
      smgfx_SetLook(3,4, 1,0,0); crPlaneNormal.Draw(&crPlanePoint); sm_GraphicsLoop();
      smgfx_SetLook(3,4, 0,0,0); cpPolyBrep->Draw(); sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
#endif

  // init output
  rSolutions.ReSet();

  // construct the PlaneSolver
  SmPolyPlaneSolver sPS(eSolverOperation, crPlanePoint,
                        crPlaneNormal, eSolutionRequested, d3dTolerance,
                        dBestAnswerSoFarSq, cpOptVectors, rSolutions);

  // place PolyBrep objects into object trees
  SmBrepCache *pBC = (SmBrepCache*)SmCacheMgr::GetOrCreateObjectCache
                                   (SM_OC_BREP,cpPolyBrep);
  NER(pBC);

  SmTreeNode sTreeNode;
  SmObjectList sObjectList;

  // Solve for all cases but SM_SO_NORMALIZE
  if(eSolverOperation != SM_SO_NORMALIZE)
    {
      // First let's compare the plane with the vertices of the PolyBrep
      SmTree *pVertTree = pBC->GetVertexTree();
      SmPLSPlaneVertex sPSLPV(sPS);
      SER(sPS.SolveTree(*pVertTree,sPSLPV));

      // Now compare the plane with the edges of the PolyBrep
      SmTree *pEdgeTree = pBC->GetCurveTree();
      SmPLSPlaneEdge sPLSPE(sPS);
      SER(sPS.SolveTree(*pEdgeTree,sPLSPE));
    }

  // case SM_SO_MINIMIZE - currently the only supported case
  if(eSolverOperation == SM_SO_MINIMIZE)
    {
      // Now, all the solutions should contain unsigned distance (to plane)
      // Modify it to be signed distance
      for (ULONG k=0; k<rSolutions.GetSize(); k++)
        {
          SmSolution & rSol = rSolutions[k];
          SmPoint3d sPnt1, sPnt2;
          sPnt1.x = rSol.m_vStart[0];
          sPnt1.y = rSol.m_vStart[1];
          sPnt1.z = rSol.m_vStart[2];
          double dDistToPlane = rSol.m_vStart.m_dSolutionValue;
          if(dDistToPlane > SM_EFF_ZERO)
            {
              SmVector3d sV = sPnt1 - crPlanePoint;
              if(sV.Dot(crPlaneNormal) < 0.0)
                {
                  rSol.m_vStart.m_dSolutionValue = -dDistToPlane;
                } // end need to negate the distance check
            } // end positive solution value check
        } // end iter every solution
    } // end eOSolverOperation == SM_SO_MINIMIZE

  // all done
  return SM_SUCCESS;

} // end SmPolySolver::PolyBrepPlaneSolve


/*******************************************************************//**
PURPOSE: This method solves a particular operation for a PolyBrep and a
    point.  Valid operations include - SM_SO_MINIMIZE, SM_SO_MAXIMIZE,
    SM_SO_PROJECTED_MINIMIZE, SM_SO_PROJECTED_MAXIMIZE, SM_SO_DIRECTED_MINIMIZE,
    SM_SO_DIRECTED_MAXIMIZE, SM_SO_SIGNED_DIRECTED_MINIMIZE,
    SM_SO_3D_SIGNED_DIRECTED_MINIMIZE,
    Note that when doing minimization and maximization a very large
    (SM_BIG_DOUBLE) or a very small (0.0) value should be set to
    dBestAnswerSoFarSq unless you already have a limiting value.
    In otherwords, if you wish to find a minimum only if it is less
    than 0.1 then you should set dBestAnswerSoFarSq = 0.1 * 0.1.
    The solutions array may or may not have existing solutions
    already registered with it.

NOTES:
***********************************************************************/
SmStatus SmPolySolver::PolyBrepPointSolve
  (const SmPolyBrep      * cpPolyBrep,
   const SmPoint3d       & crPoint,
   SmSolverOperationType   eSolverOperation,
   SmSolutionRequestedType eSolutionRequested,
   double                  d3dTolerance,
   double                  dBestAnswerSoFarSq,
   const SmVector3d      * cpOptVectors,
   SmSolutionArray       & rSolutions)
{
  // check input - support these solver operations
  if(   eSolverOperation != SM_SO_MINIMIZE
     && eSolverOperation != SM_SO_MAXIMIZE
     && eSolverOperation != SM_SO_PROJECTED_MINIMIZE
     && eSolverOperation != SM_SO_PROJECTED_MAXIMIZE
     && eSolverOperation != SM_SO_SIGNED_DIRECTED_MINIMIZE
     && eSolverOperation != SM_SO_3D_SIGNED_DIRECTED_MINIMIZE
     && eSolverOperation != SM_SO_DIRECTED_MINIMIZE
     && eSolverOperation != SM_SO_DIRECTED_MAXIMIZE)
    {
      SER(SM_ERR_INVALID_INPUT);
    }

  // check input - must have PolyBrep
  NER(cpPolyBrep);

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
  if(bDebugMe)
    {
      smgfx_Erase();
      smgfx_SetLook(3,4, 1,0,0); crPoint.Draw(); sm_GraphicsLoop();
      smgfx_SetLook(3,4, 0,0,0); cpPolyBrep->Draw(); sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
#endif

  // init output
  rSolutions.ReSet();

  // construct PolySolver
  SmPolySolver sPS(eSolverOperation, eSolutionRequested, d3dTolerance,
                   dBestAnswerSoFarSq, cpOptVectors, rSolutions);

  // place PolyBrep objects into object trees
  SmBrepCache *pBC = (SmBrepCache*)SmCacheMgr::GetOrCreateObjectCache
                        (SM_OC_BREP,cpPolyBrep);
  NER(pBC);

  SmTreeNode sTreeNode;
  SmObjectList sObjectList;
  SmTree sPointTree(crPoint,0,&sTreeNode,&sObjectList);

  // all operations but SM_SO_NORMALIZE
  if(eSolverOperation != SM_SO_NORMALIZE)
    {
      // First let's compare the point with the vertices of the PolyBrep
      SmTree *pVertTree = pBC->GetVertexTree();
      SmPLSPointVertex sPSLPV(sPS);
      sPSLPV.SetPoint(crPoint);
      SER(sPS.SolveTrees(sPointTree,*pVertTree,sPSLPV));

      // Now compare the point with the edges of the PolyBrep
      SmTree *pEdgeTree = pBC->GetCurveTree();
      SmPLSPointEdge sPLSPE(sPS);
      sPLSPE.SetPoint(crPoint);
      SER(sPS.SolveTrees(sPointTree,*pEdgeTree,sPLSPE));
    }

  // Now compare the point with the faces of the PolyBrep
  SmTree *pFaceTree = pBC->GetSurfaceTree();
  SmPLSPointFace sPLSPF(sPS);
  sPLSPF.SetPoint(crPoint);
  SER(sPS.SolveTrees(sPointTree,*pFaceTree,sPLSPF));

  // all done
  return SM_SUCCESS;

} // end SmPolySolver::PolyBrepPointSolve

/*******************************************************************//**
PURPOSE: Static internal method to solve for pairs of object trees

NOTES:  Looks for all Vertex/Vertex  Vertex/Edge   Edge/Vertex
                               Edge/Edge      Vertex/Face   Face/Vertex
                               Edge/Face      Face/Edge
                 solutions.
   note: no Face/Face solver
   solutions accumulate in rPS.m_pSolutions
***********************************************************************/
static SmStatus sm_SolveTwoTrees
  (SmPolySolver          & rPS,               // in :
   SmSolverOperationType   eSolverOperation,  // in :
   SmTree                * pVertTree1,        // in :
   SmTree                * pEdgeTree1,        // in :
   SmTree                * pFaceTree1,        // in :
   SmTree                * pVertTree2,        // in :
   SmTree                * pEdgeTree2,        // in :
   SmTree                * pFaceTree2)        // in :
{
  if(eSolverOperation != SM_SO_NORMALIZE)
    {

      { // First let's compare vertices of each
        SmPLSVertexVertex sPSLVV(rPS);
        SER(rPS.SolveTwoTrees(*pVertTree1,*pVertTree2,sPSLVV));
      }

      { // Now let's compare the edges with the vertices of each.
        SmPLSVertexEdge sPSLVE(rPS);
        SER(rPS.SolveTwoTrees(*pVertTree1,*pEdgeTree2,sPSLVE));
      }

      { // Do edges/vertices and swap order
        SmPLSVertexEdge sPSLVE2(rPS,TRUE);
        SER(rPS.SolveTwoTrees(*pEdgeTree1,*pVertTree2,sPSLVE2));
      }

      {// Now let's compare the edges of each
        SmPLSEdgeEdge sPSLEE(rPS);
        SER(rPS.SolveTwoTrees(*pEdgeTree1,*pEdgeTree2,sPSLEE));
      }

      { // Do vertices/faces
        SmPLSVertexFace sPSLVF(rPS);
        SER(rPS.SolveTwoTrees(*pVertTree1,*pFaceTree2,sPSLVF));
      }

      {
        SmPLSVertexFace sPSLVF2(rPS,TRUE);
        SER(rPS.SolveTwoTrees(*pFaceTree1,*pVertTree2,sPSLVF2));
      }

      { // Do edges/faces
        SmPLSEdgeFace sPSLEF(rPS);
        SER(rPS.SolveTwoTrees(*pEdgeTree1,*pFaceTree2,sPSLEF));
      }

      { // Do edges/faces and swap order
        SmPLSEdgeFace sPSLEF2(rPS,TRUE);
        SER(rPS.SolveTwoTrees(*pFaceTree1,*pEdgeTree2,sPSLEF2));
      }
    }

  // all done
  return SM_SUCCESS;

} // end sm_SolveTwoTrees

/*******************************************************************//**
PURPOSE: This method solves for a particular operation between two
    PolyBreps.  Valid operations include: SM_SO_MINIMIZE, SM_SO_MAXIMIZE,
    SM_SO_PROJECTED_MINIMIZE, SM_SO_PROJECTED_MAXIMIZE, SM_SO_DIRECTED_MINIMIZE,
    SM_SO_DIRECTED_MAXIMIZE, SM_SO_SIGNED_DIRECTED_MINIMIZE,
    SM_SO_3D_SIGNED_DIRECTED_MINIMIZE,

NOTES:
***********************************************************************/
SmStatus SmPolySolver::PolyBrepPolyBrepSolve
  (const SmPolyBrep      * cpPolyBrep1,
   const SmPolyBrep      * cpPolyBrep2,
   SmSolverOperationType   eSolverOperation,
   SmSolutionRequestedType eSolutionRequested,
   double                  d3dTolerance,
   double                  dBestAnswerSoFarSq,
   const SmVector3d      * cpOptVectors,
   SmSolutionArray       & rSolutions)
{
  // check input - support these solver operations
  if(   eSolverOperation != SM_SO_MINIMIZE
     && eSolverOperation != SM_SO_MAXIMIZE
     && eSolverOperation != SM_SO_PROJECTED_MINIMIZE
     && eSolverOperation != SM_SO_PROJECTED_MAXIMIZE
     && eSolverOperation != SM_SO_SIGNED_DIRECTED_MINIMIZE
     && eSolverOperation != SM_SO_3D_SIGNED_DIRECTED_MINIMIZE
     && eSolverOperation != SM_SO_DIRECTED_MINIMIZE
     && eSolverOperation != SM_SO_DIRECTED_MAXIMIZE)
    {
      SER(SM_ERR_INVALID_INPUT);
    }

  //init output
  rSolutions.ReSet();

  // construct the PolySolver
  SmPolySolver sPS(eSolverOperation,
                   eSolutionRequested,
                   d3dTolerance,
                   dBestAnswerSoFarSq,
                   cpOptVectors,
                   rSolutions);

  // place PolyBrep objects into object trees
  SmBrepCache *pBC1 = (SmBrepCache*)SmCacheMgr::GetOrCreateObjectCache(SM_OC_BREP,cpPolyBrep1); NER(pBC1);
  SmBrepCache *pBC2 = (SmBrepCache*)SmCacheMgr::GetOrCreateObjectCache(SM_OC_BREP,cpPolyBrep2); NER(pBC2);

  // extract the cache object trees
  SmTree *pVertTree1 = pBC1->GetVertexTree();
  SmTree *pEdgeTree1 = pBC1->GetCurveTree();
  SmTree *pFaceTree1 = pBC1->GetSurfaceTree();

  SmTree *pVertTree2 = pBC2->GetVertexTree();
  SmTree *pEdgeTree2 = pBC2->GetCurveTree();
  SmTree *pFaceTree2 = pBC2->GetSurfaceTree();

  // solve tree pairs for
  SER(sm_SolveTwoTrees(sPS, eSolverOperation,
                         pVertTree1, pEdgeTree1, pFaceTree1,
                         pVertTree2, pEdgeTree2, pFaceTree2));

  return SM_SUCCESS;

} // end SmPolySolver::PolyBrepPolyBrepSolve

/*******************************************************************//**
PURPOSE: This method solves a particular operation for a PolyBrep and a
    rectangular bounded plane. It finds the intersection between the
    poly brep and returns any single point of intersection.

NOTES:  looks for Vertex/Vertex, Vertex/Edge, Vertex/Face
                  Edge/Edge, and Edge/Face point intersections.
        does not look for Face/Face intersections.
***********************************************************************/
SmStatus SmPolySolver::PolyBrepRectangleIntersect
  (const SmPolyBrep       * cpPolyBrep,       // in :
   const SmAxis2Placement & crRecPlacement,   // in : Location of the plane origin and X,Y-Axes
   const SmExtent2d       & crRecDomain,      // in : Extents of the bounded plane along X,Y-Axes
   double                   d3dTolerance,     // in :
   SmSolutionArray        & rSolutions)       // out:
{
  NER(cpPolyBrep);

  // First, make the bounded rectangle a temporary polygon
  const SmContext *cpContext      = cpPolyBrep->GetContext() ; SM_ASSERT(cpContext != NULL) ;
  SmPolyBrep      *pOtherPolyBrep = new (*cpContext) SmPolyBrep( d3dTolerance,
                                                                 50 + 50 + 10 +
                                                                 10 + 10 + 2 +
                                                                 2 );
  SmObjDelete sClean(pOtherPolyBrep);

  // Adjust the SmAxis2Placement such that the origin
  // will be on the 'bottom-left' corner of the rectangle
  SmAxis2Placement sRecPos    = crRecPlacement;
  SmPoint3d        sOrigin    = sRecPos.GetOriginRef();
  SmPoint2d        sDomainMin = crRecDomain.GetMin();

  sOrigin = sOrigin + sDomainMin.x * sRecPos.GetXAxisRef();
  sOrigin = sOrigin + sDomainMin.y * sRecPos.GetYAxisRef();
  sRecPos.SetCanonical(sOrigin,sRecPos.GetXAxisRef(),sRecPos.GetYAxisRef());

  SmPolyFace *pPolyFace = NULL;
  SmPoint2d   sRecSize  = crRecDomain.GetSize();
  SER(pOtherPolyBrep->CreateRectangle(d3dTolerance,
                                      sRecSize.x,
                                      sRecSize.y,
                                      sRecPos,
                                      pPolyFace));

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
  if(bDebugMe)
    {
      smgfx_Erase();
      smgfx_SetLook(3,4, 0,1,0); pOtherPolyBrep->Draw(); sm_GraphicsLoop();
      smgfx_SetLook(3,4, 0,0,1); cpPolyBrep->Draw(); sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
#endif
  // Store plane origin & normal information in solver
  SmVector3d sPlaneVectors[2];
  sPlaneVectors[0] = sOrigin;
  sPlaneVectors[1] = sRecPos.GetZAxis();

  // init output
  rSolutions.ReSet();

  // Setup a polybrep-polybrep solver
  SmSolverOperationType eSolverOperation = SM_SO_INTERSECT;
  SmPolySolver sPS(eSolverOperation,
                   SM_SR_SINGLE,
                   d3dTolerance,
                   SM_BIG_DOUBLE,
                   sPlaneVectors,
                   rSolutions);

  // place PolyBrep objects into object trees
  SmBrepCache *pBC1 = (SmBrepCache*)SmCacheMgr::GetOrCreateObjectCache(SM_OC_BREP,cpPolyBrep); NER(pBC1);
  SmBrepCache *pBC2 = (SmBrepCache*)SmCacheMgr::GetOrCreateObjectCache(SM_OC_BREP,pOtherPolyBrep); NER(pBC2);

  SmTree *pVertTree1 = pBC1->GetVertexTree();
  SmTree *pEdgeTree1 = pBC1->GetCurveTree();
  SmTree *pFaceTree1 = pBC1->GetSurfaceTree();

  SmTree *pVertTree2 = pBC2->GetVertexTree();
  SmTree *pEdgeTree2 = pBC2->GetCurveTree();
  SmTree *pFaceTree2 = pBC2->GetSurfaceTree();

  SER(sm_SolveTwoTrees(sPS, eSolverOperation,
                       pVertTree1, pEdgeTree1, pFaceTree1,
                       pVertTree2, pEdgeTree2, pFaceTree2));


#ifdef SM_DEBUG_CODE
  if(bDebugMe)
    {
      pVertTree1->Dump() ;
      pVertTree2->Dump() ;
      pEdgeTree1->Dump() ;
      pEdgeTree2->Dump() ;
      pFaceTree1->Dump() ;
      pFaceTree2->Dump() ;

      rSolutions.Dump() ;

      smgfx_Erase();
      smgfx_SetLook(1,2, 0,1,0); pOtherPolyBrep->Draw(); sm_GraphicsLoop();
      smgfx_SetLook(1,2, 0,0,1); cpPolyBrep->Draw(); sm_GraphicsLoop();
      smgfx_SetLook(3,4, 1,0,0); rSolutions.Draw() ; sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
#endif

  // clear the Solution Objects - half of them become stale when pOtherPolyBrep gets cleaned
  for(ULONG ii=0;ii<rSolutions.GetSize();ii++)
    {
      SmSolution &rSolution = rSolutions[ii] ;
      for(ULONG jj=0;jj<rSolution.m_lNumObjects;jj++)
        {
          rSolution.m_apObjects[jj] = NULL ;
        }
    }

  // all done
  return SM_SUCCESS;

} // end SmPolySolver::PolyBrepRectangleIntersect

/*******************************************************************//**
PURPOSE: Return TRUE the two bounding boxes satisfy the criteria
   necessary to satisfy the given solution.

NOTES:
***********************************************************************/
SmBoolean SmPolySolver::MayContainAnswer
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

              if(sBBox.AreDisjoint(crBox2))
                {
                  return FALSE; // No answers in this branch
                }
              return TRUE;  // possible answers in this branch
            }

      case SM_SO_MINIMIZE:
            {
              double dMinDistSq = crBox1.MinimumDistanceSquared(crBox2);
              if(dMinDistSq > m_dBestAnswerSoFarSq)
                {
                  if(smos_Sqrt(dMinDistSq) - m_d3dTolerance >
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
              if(dMaxDistSq < m_dBestAnswerSoFarSq)
                {
                  if(smos_Sqrt(dMaxDistSq) + m_d3dTolerance <
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
              if(dMaxDistSq < m_dAtDistanceSq)
                {
                  if(smos_Sqrt(dMaxDistSq) - m_d3dTolerance <
                      smos_Sqrt(m_dAtDistanceSq) )
                    {
                      return FALSE; // no answers in this branch
                    }
                }
              double dMinDistSq = crBox1.MinimumDistanceSquared(crBox2);
              if(dMinDistSq > m_dAtDistanceSq)
                {
                  if(smos_Sqrt(dMinDistSq) + m_d3dTolerance >
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
              if(m_eSolverOperation == SM_SO_PROJECTED_MINIMIZE)
                {
                  double dMinProjDist = sProjPnt.DistanceBetween(sSph1Cent) -
                      dSph1Radius - dSph2Radius;
                  if(dMinProjDist > 0.0 && dMinProjDist*dMinProjDist > m_dBestAnswerSoFarSq)
                    {
                      return FALSE;
                    }
                }
              if(m_eSolverOperation == SM_SO_PROJECTED_MAXIMIZE)
                {
                  double dMaxProjDist = sProjPnt.DistanceBetween(sSph1Cent) +
                      dSph1Radius + dSph2Radius;
                  if(dMaxProjDist*dMaxProjDist < m_dBestAnswerSoFarSq)
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
              if(sProjPnt.DistanceBetweenSquared(sSph2Cent) >
                  dSumRadius*dSumRadius+SM_EFF_ZERO_SQRT)
                {
                  return FALSE; // These two nodes can not solve this directed minimization
                }

              SmVector3d sVec = sProjPnt - sSph1Cent;
              if(m_eSolverOperation == SM_SO_SIGNED_DIRECTED_MINIMIZE)
                {
                  double dDist = sVec.Dot(m_cpOptVectors[1]);
                  double dMinProjDist = dDist - dSph1Radius - dSph2Radius;
                  if(dMinProjDist*smos_Fabs(dMinProjDist) > m_dBestAnswerSoFarSq)
                    {
                      return FALSE;
                    }
                }

              if(m_eSolverOperation == SM_SO_DIRECTED_MINIMIZE)
                {
                  double dDist = smos_Fabs(sVec.Dot(m_cpOptVectors[1]));
                  double dMinProjDist = dDist - dSph1Radius - dSph2Radius;
                  if(dMinProjDist > 0.0 && dMinProjDist*dMinProjDist > m_dBestAnswerSoFarSq)
                    {
                      return FALSE;
                    }
                }

              if(m_eSolverOperation == SM_SO_DIRECTED_MAXIMIZE)
                {
                  double dDist = smos_Fabs(sVec.Dot(m_cpOptVectors[1]));
                  double dMaxProjDist = dDist + dSph1Radius + dSph2Radius;
                  if(dMaxProjDist*dMaxProjDist < m_dBestAnswerSoFarSq)
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
              SmVector3d sLinePnt = crBox1.GetMin();
              SmPoint3d sLineVec = m_cpOptVectors[0];
              SmPoint3d sSph1Cent;
              double dSph1Radius;
              crBox2.ComputeSphereBound(sSph1Cent,dSph1Radius);
              double dParam;
              SER(smgu_LineClosestPoint(sLinePnt,sLineVec,sSph1Cent,dParam));
              SmPoint3d sClPnt = sLinePnt + dParam * sLineVec;
              double dDist = sClPnt.DistanceBetween(sSph1Cent);
              // See if sphere does not intersect the infinite line
              if(dDist > dSph1Radius + m_d3dTolerance)
                {
                  return FALSE;
                }
              // See if we are on wrong side of ray start point
              if(dParam + dSph1Radius < 0.0)
                {
                  return FALSE;
                }
              // See if we are further out on the positive side of the ray than
              // the best answer so far.
              double dMinPossibleRayDistance = dParam - dSph1Radius - m_d3dTolerance;
              if(dMinPossibleRayDistance > 0.0)
                {
                  if(dMinPossibleRayDistance*dMinPossibleRayDistance > m_dBestAnswerSoFarSq)
                    {
                      return FALSE;
                    }
                }
              // Do some additional tests to see if the ray does not intersect with the
              // box of the object.
              double dLeng = 2.0 * (smos_Fabs(dParam) + dSph1Radius);
              SmPoint3d sEndPnt = sLinePnt +  dLeng * sLineVec;
              SmPoint3d sStartPnt = sLinePnt - dLeng * sLineVec;
              SmExtent3d sRayBox(sStartPnt);
              sRayBox.AddPoint3d(sEndPnt);
              sRayBox.ExpandAbsolute(m_d3dTolerance);
              if(sRayBox.AreDisjoint(crBox2))
                {
                  return FALSE;
                }

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
              if(dDist > dSumRadius + m_d3dTolerance)
                {
                  return FALSE;
                }
              SmVector3d sVec = sClPnt - sSph1Cent;
              double dDistTo = sVec.Length();
              if(sVec.Dot(m_cpOptVectors[0]) < 0)
                {
                  dDistTo = - dDistTo;
                }
              double dMinDist = dDistTo - dSph1Radius - dSph2Radius - m_d3dTolerance;
              if(dMinDist*smos_Fabs(dMinDist) > m_dBestAnswerSoFarSq)
                {
                  return FALSE;
                }

              return TRUE;
            }

      default:
          ERR(SM_ERR);  // not handled yet

   } // end switch (m_eSolverOperation)

 return TRUE;

} // end SmPolySolver::MayContainAnswer

/*******************************************************************//**
PURPOSE: Do a local solve of the branch.

NOTES: Local solve needs to be done for all intermediate
    nodes as well as all leaf nodes.  Assume that the local solver will be
    called for both cases.  It should return rbNeedsMoreSubdivision = FALSE
    only when reaching all leaf nodes.
***********************************************************************/
SmStatus SmPolySolver::LocalSolve
  (SmTreeNode * apBranch[SM_GS_MAX_TREES],
   SmBoolean  & rbNeedsMoreSubdivision)
{
  rbNeedsMoreSubdivision = FALSE;
  SmTreeNode *pNode1 = apBranch[0];
  SmTreeNode *pNode2 = apBranch[1];

  SmObjectList* aData1[20];
  SmObjectList* aData2[20];
  SmTArray<SmObjectList*> sObjs1(20,aData1);
  SmTArray<SmObjectList*> sObjs2(20,aData2);
  pNode1->GetObjectList(sObjs1);

  for (ULONG i=0; i<sObjs1.GetSize(); i++)
    {
      SmObjectList *pObj1 = sObjs1[i];
      if(!MayContainAnswer(pObj1->m_sBBox,pNode2->m_sBBox)) continue;
      pNode2->GetObjectList(sObjs2);
      for (ULONG j=0; j<sObjs2.GetSize(); j++)
        {
          SmObjectList *pObj2 = sObjs2[j];
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
          if(bDebugMe)
            {
              smgfx_Erase();
              smgfx_SetLook(1,2, 1,0,0); pObj1->m_sBBox.Draw(NULL); sm_GraphicsLoop();
              smgfx_SetLook(1,2, 0,0,0); pObj2->m_sBBox.Draw(NULL); sm_GraphicsLoop();
              sm_GraphicsLoop();
            }
#endif
          if(!MayContainAnswer(pObj1->m_sBBox,pObj2->m_sBBox)) continue;
          // If made it to here then we have two objects which may contain an answer
          // Invoke the local solver.
          SER(m_pLocalSolver->SolveIt(pObj1->m_pObject,pObj2->m_pObject));
        }
    }

  // Keep going down the trees if there are more nodes below
  if(pNode1->m_pChild1 != NULL || pNode2->m_pChild1 != NULL)
    {
      rbNeedsMoreSubdivision = TRUE;
    }

  return SM_SUCCESS;

} // end SmPolySolver::LocalSolve

/*******************************************************************//**
PURPOSE: Determine if the given branch may contain answers.  In other
   words can we utilize existing information (i.e. bounding boxes) to
   eliminate the possibility of any answers in this branch.

NOTES:
***********************************************************************/
SmBoolean SmPolySolver::BranchMayContainAnswers
  (SmTreeNode * apBranch[SM_GS_MAX_TREES])
{
  // No natural optimization for single tree operations.
  // You'll have to do this by subclassing.
  if(m_lNumTrees==1)
    {
      return TRUE;
    }

#if SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
  if(bDebugMe)
    {
      smgfx_Erase();
      sm_GraphicsLoop();
      apBranch[0]->m_sBBox.Draw(NULL);
      apBranch[1]->m_sBBox.Draw(NULL);
      sm_GraphicsLoop();
    }
#endif
  switch (m_eSolverOperation)
    {
      case SM_SO_INTERSECT:
      case SM_SO_INTERSECTION_TEST:
            {
              for (ULONG i=0; i+1<m_lNumTrees; i++)  // note: can't say m_lNumTrees-1
                {
                  SmExtent3d sBBox = apBranch[i]->m_sBBox;
                  sBBox.ExpandAbsolute(m_d3dTolerance);

                  for (ULONG j=i+1; j<m_lNumTrees; j++)
                    {
                      SmExtent3d *pBBox2 = &apBranch[j]->m_sBBox;
                      if(sBBox.AreDisjoint(*pBBox2))
                        {
                          return FALSE; // No answers in this branch
                        }
                    }
                }
              return TRUE;  // possible answers in this branch
            }

      case SM_SO_MINIMIZE:
            {
              SM_ASSERT(m_lNumTrees == 2);
              double dMinDistSq = apBranch[0]->m_sBBox.MinimumDistanceSquared(apBranch[1]->m_sBBox);
              if(dMinDistSq > m_dBestAnswerSoFarSq)
                {
                  if(smos_Sqrt(dMinDistSq) - m_d3dTolerance >
                      smos_Sqrt(m_dBestAnswerSoFarSq) )
                    {
                      return FALSE; // no answers in this branch
                    }
                }
              return TRUE;
            }

      case SM_SO_MAXIMIZE:
            {
              SM_ASSERT(m_lNumTrees == 2);
              double dMaxDistSq = apBranch[0]->m_sBBox.MaximumDistanceSquared(apBranch[1]->m_sBBox);
              if(dMaxDistSq < m_dBestAnswerSoFarSq)
                {
                  if(smos_Sqrt(dMaxDistSq) + m_d3dTolerance <
                      smos_Sqrt(m_dBestAnswerSoFarSq) )
                    {
                      return FALSE; // no answers in this branch
                    }
                }
              return TRUE;
            }

      case SM_SO_AT_DISTANCE:
            {
              SM_ASSERT(m_lNumTrees == 2);
              double dMaxDistSq = apBranch[0]->m_sBBox.MaximumDistanceSquared(
                  apBranch[1]->m_sBBox);
              if(dMaxDistSq < m_dAtDistanceSq)
                {
                  if(smos_Sqrt(dMaxDistSq) - m_d3dTolerance <
                      smos_Sqrt(m_dAtDistanceSq) )
                    {
                      return FALSE; // no answers in this branch
                    }
                }
              double dMinDistSq = apBranch[0]->m_sBBox.MinimumDistanceSquared(
                  apBranch[1]->m_sBBox);
              if(dMinDistSq > m_dAtDistanceSq)
                {
                  if(smos_Sqrt(dMinDistSq) + m_d3dTolerance >
                      smos_Sqrt(m_dAtDistanceSq) )
                    {
                      return FALSE; // no answers in this branch
                    }
                }
              return TRUE; // may be answers
            }

      case SM_SO_PROJECTED_MINIMIZE:
      case SM_SO_PROJECTED_MAXIMIZE:
      case SM_SO_PROJECTED_INTERSECT:
      case SM_SO_ROTATED_PROJECTED_INTERSECT:
      case SM_SO_PERSPECTIVE_INTERSECT:
        {
          // get Sphere bounds for the branch bboxes
          SmPoint3d sSph1Center, sSph2Center;
          double    dSph1Radius, dSph2Radius;
          apBranch[0]->m_sBBox.ComputeSphereBound(sSph1Center,dSph1Radius);
          apBranch[1]->m_sBBox.ComputeSphereBound(sSph2Center,dSph2Radius);

          // Factor 3D tolerance by adding it to just one sphere radii.
          dSph1Radius = dSph1Radius + m_d3dTolerance;

          // project SphereCenters to common plane circles that can be quickly checked no intersections
          SmPoint3d sProjPnt1, sProjPnt2 ;
          if(m_eSolverOperation == SM_SO_ROTATED_PROJECTED_INTERSECT)
            {
              // m_cpOptVectors[1] = unitRotAxis, m_cpOptVectors[2] = unitXAxis
              SM_ASSERT( SM_IS_ZERO(m_cpOptVectors[1].Length() - 1.0) ) ;
              SM_ASSERT( SM_IS_ZERO(m_cpOptVectors[2].Length() - 1.0) ) ;
              SM_ASSERT( SM_IS_ZERO(m_cpOptVectors[1].Dot(m_cpOptVectors[2]))) ;

              // rotate project Sph1 and Sph2 CenterPts to same plane
              //  (rather than rotate Sph2Center to Sph1Center plane we rotate both to a common plane
              //   to avoid making all checks and spaecial cases if Sph1Center happens to be on the rotation axis)
              sProjPnt1 = sSph1Center.RotateProjectPointToPlane(m_cpOptVectors[0], m_cpOptVectors[1], m_cpOptVectors[2]) ;
              sProjPnt2 = sSph2Center.RotateProjectPointToPlane(m_cpOptVectors[0], m_cpOptVectors[1], m_cpOptVectors[2]) ;

            } // end SolerOperation == RotatedProjectedIntersect branch
          else if(m_eSolverOperation == SM_SO_PERSPECTIVE_INTERSECT)
            {
              // m_cpOptVectors[0] = point on view plane
              // m_cpOptVectors[1] = unit normal to view plane
              // m_cpOptVectors[2] = eye point
              SM_ASSERT( SM_IS_ZERO(m_cpOptVectors[1].Length() - 1.0) ) ;

              // perspective project Sph1 and Sph2 CenterPts to same plane
              sProjPnt1 = sSph1Center.PerspectiveProjectPointToPlane(m_cpOptVectors[0], m_cpOptVectors[1], m_cpOptVectors[2]) ;
              sProjPnt2 = sSph2Center.PerspectiveProjectPointToPlane(m_cpOptVectors[0], m_cpOptVectors[1], m_cpOptVectors[2]) ;

              // spheres project to ellipses - find max radius of sphere projections
              SmVector3d sEyeVec1 = sSph1Center - m_cpOptVectors[2] ;
              SmVector3d sDir1    = sEyeVec1 * m_cpOptVectors[1] ; // vec perp to eyevec and view norm
              sDir1.Unitize() ;
              SmVector3d sDir2    = sDir1 * sEyeVec1 ;             // vec perp to eyevec in view normal plane
              sDir2.Unitize() ;
              SmVector3d sRadVec ;
              double dNewRad1MaxSq = 0.0, dNewRadSq ;
              ULONG ii ;
              dSph1Radius -= m_d3dTolerance ;
              for(ii=0;ii<4;ii++)
                {
                  sRadVec   = sSph1Center + (  ii == 0 ?  sDir1 * dSph1Radius
                                             : ii == 1 ? -sDir1 * dSph1Radius
                                             : ii == 2 ?  sDir2 * dSph1Radius
                                             :           -sDir2 * dSph1Radius) ;
                  sRadVec   = sRadVec.PerspectiveProjectToPlane(sSph1Center, m_cpOptVectors[0], m_cpOptVectors[1], m_cpOptVectors[2]) ;
                  dNewRadSq = sRadVec.LengthSquared();
                  if(dNewRadSq > dNewRad1MaxSq) dNewRad1MaxSq = dNewRadSq ;
                }

              SmVector3d sEyeVec2 = sSph2Center - m_cpOptVectors[2] ;
              sDir1    = sEyeVec2 * m_cpOptVectors[1] ; // vec perp to eyevec and view norm
              sDir1.Unitize() ;
              sDir2    = sDir1 * sEyeVec2 ;             // vec perp to eyevec in view normal plane
              sDir2.Unitize() ;
              double dNewRad2MaxSq = 0.0 ;
              for(ii=0;ii<4;ii++)
                {
                  sRadVec   = sSph2Center + (  ii == 0 ?  sDir1 * dSph1Radius
                                             : ii == 1 ? -sDir1 * dSph1Radius
                                             : ii == 2 ?  sDir2 * dSph1Radius
                                             :           -sDir2 * dSph1Radius) ;
                  sRadVec   = sRadVec.PerspectiveProjectToPlane(sSph1Center, m_cpOptVectors[0], m_cpOptVectors[1], m_cpOptVectors[2]) ;
                  dNewRadSq = sRadVec.LengthSquared();
                  if(dNewRadSq > dNewRad2MaxSq) dNewRad2MaxSq = dNewRadSq ;
                }

              // arrive here when sProjPnt1 and sProjPnt2 are the projected centers of the spheres and
              //   dNewRad1Max and dNewRad2Max are the maximum radii of the sphere projections onto the viewing plane
              // update variables
              dSph1Radius = smos_Sqrt(dNewRad1MaxSq) + m_d3dTolerance ;
              dSph2Radius = smos_Sqrt(dNewRad2MaxSq) ;
            } // end SolerOperation == SM_SO_PERSPECTIVE_INTERSECT branch
          else
            {
              // project Sph2 CenterPt to same plane as Sph1 CenterPt
              SER(smgu_PointProjectToPlane(sSph2Center,        // in : target point
                                           sSph1Center,        // in : point on plane
                                           m_cpOptVectors[0],  // in : plane surface normal
                                           sProjPnt2));         // out: target point projected to plane
              sProjPnt1 = sSph1Center ;
            }  // end SolerOperation != RotatedProjectedIntersect branch

          // for each operation - see if spheres are properly positioned to exclude a possible solution
          switch(m_eSolverOperation)
            {
              case SM_SO_PROJECTED_INTERSECT:
              case SM_SO_ROTATED_PROJECTED_INTERSECT:
              case SM_SO_PERSPECTIVE_INTERSECT:
                {
                  double dMinProjDist =   sProjPnt1.DistanceBetween(sProjPnt2)
                                        - dSph1Radius
                                        - dSph2Radius;
                  if (dMinProjDist > 0.0)
                    {
                      return FALSE; // No answers in this branch
                    }
                } break ;

              case SM_SO_PROJECTED_MINIMIZE:
                {
                  double dMinProjDist =   sProjPnt1.DistanceBetween(sProjPnt2)
                                        - dSph1Radius
                                        - dSph2Radius;
                  if (   dMinProjDist > 0.0
                      && dMinProjDist * dMinProjDist > m_dBestAnswerSoFarSq)
                    {
                      return FALSE; // No answers in this branch
                    }
                } break ;

              case SM_SO_PROJECTED_MAXIMIZE:
                {
                  double dMaxProjDist =   sProjPnt1.DistanceBetween(sProjPnt2)
                                        + dSph1Radius
                                        + dSph2Radius;
                  if (dMaxProjDist * dMaxProjDist < m_dBestAnswerSoFarSq)
                    {
                      return FALSE; // No answers in this branch
                    }
                } break ;

              default:
                    break;
            } // end switch on m_eSolverOperation

          // If we were unable to elimiate using above sphere tests then
          // we just skip out here with a TRUE.
          return TRUE; // possible answers in this branch
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
              apBranch[0]->m_sBBox.ComputeSphereBound(sSph1Cent,dSph1Radius);
              apBranch[1]->m_sBBox.ComputeSphereBound(sSph2Cent,dSph2Radius);
              SmPoint3d sProjPnt;
              SER(smgu_PointProjectToPlane(sSph2Cent,sSph1Cent,sPlaneNorm,sProjPnt));
              double dSumRadius = dSph1Radius + dSph2Radius;
              if(sProjPnt.DistanceBetweenSquared(sSph2Cent) >
                  dSumRadius*dSumRadius+SM_EFF_ZERO_SQRT)
                {
                  return FALSE; // These two nodes can not solve this directed minimization
                }

              // Now take projected point and project it again up to projection plane
              // as defined going through sSph1Center
              SER(smgu_PointProjectToPlane(sProjPnt,sSph1Cent,m_cpOptVectors[0],sProjPnt));

              if(m_eSolverOperation == SM_SO_SIGNED_DIRECTED_MINIMIZE)
                {
                  SmVector3d sVec = sProjPnt - sSph1Cent;
                  double dDist = sVec.Length();
                  if(sVec.Dot(m_cpOptVectors[1]) < 0)
                    {
                      dDist = - dDist;
                    }
                  double dMinProjDist = dDist - dSph1Radius - dSph2Radius;
                  if(dMinProjDist*smos_Fabs(dMinProjDist) > m_dBestAnswerSoFarSq)
                    {
                      return FALSE;
                    }
                }

              if(m_eSolverOperation == SM_SO_DIRECTED_MINIMIZE)
                {
                  double dMinProjDist = sProjPnt.DistanceBetween(sSph1Cent) -
                      dSph1Radius - dSph2Radius;
                  if(dMinProjDist > 0.0 && dMinProjDist*dMinProjDist > m_dBestAnswerSoFarSq)
                    {
                      return FALSE;
                    }
                }

              if(m_eSolverOperation == SM_SO_DIRECTED_MAXIMIZE)
                {
                  double dMaxProjDist = sProjPnt.DistanceBetween(sSph1Cent) +
                      dSph1Radius + dSph2Radius;
                  if(dMaxProjDist*dMaxProjDist < m_dBestAnswerSoFarSq)
                    {
                      return FALSE;
                    }
                }

              return TRUE;
            }

      case SM_SO_SIGNED_PIVOT_MINIMIZE:
            {
              // First, find projected distance from pivot pt to each center of bounding spheres.
              // If the difference of those two distances is greater than the sum of
              // the two radii, we can determine that no solutions will come from these two nodes.
              // PlaneNorm: m_cpOptVectors[0];
              // Pivot: m_cpOptVectors[1];
              SmPoint3d sSph1Cent, sSph2Cent;
              double dSph1Radius, dSph2Radius;
              apBranch[0]->m_sBBox.ComputeSphereBound(sSph1Cent,dSph1Radius);
              apBranch[1]->m_sBBox.ComputeSphereBound(sSph2Cent,dSph2Radius);
              SmVector3d sVec1 = m_cpOptVectors[0]*(sSph1Cent-m_cpOptVectors[1])*m_cpOptVectors[0];
              SmVector3d sVec2 = m_cpOptVectors[0]*(sSph2Cent-m_cpOptVectors[1])*m_cpOptVectors[0];
              double dSumRadius = dSph1Radius + dSph2Radius;
              if(smos_Fabs(sVec1.Length() - sVec2.Length()) - m_d3dTolerance > dSumRadius)
                {
                  return FALSE; // These two nodes can not solve this pivot minimization
                }

              return TRUE;
            }

      case SM_SO_NORMALIZE:
            // No optimization here yet - should be added in virtual method
            // for lower level object.
            break;

      case SM_SO_FIND:
            // No optimization for plain searching - should be added as
            // a virtual method
            break;

            // Doing a ray firing - minimization of intersections
            // along a vector.
      case SM_SO_RAYFIRE:
            {
              SmVector3d sLinePnt = apBranch[0]->m_sBBox.GetMin();
              SmPoint3d sLineVec = m_cpOptVectors[0];
              SmPoint3d sSph1Cent;
              double dSph1Radius;
              apBranch[1]->m_sBBox.ComputeSphereBound(sSph1Cent,dSph1Radius);
              double dParam;
              SER(smgu_LineClosestPoint(sLinePnt,sLineVec,sSph1Cent,dParam));
              SmPoint3d sClPnt = sLinePnt + dParam * sLineVec;
              double dDist = sClPnt.DistanceBetween(sSph1Cent);
              // See if sphere does not intersect the infinite line
              if(dDist > dSph1Radius + m_d3dTolerance)
                {
                  return FALSE;
                }
              // See if we are on wrong side of ray start point
              if(dParam + dSph1Radius < 0.0)
                {
                  return FALSE;
                }

              // See if we are further out on the ray than the best answer so
              // far.
              double dMinPossibleRayDistance = dParam - dSph1Radius - m_d3dTolerance;
              if(dMinPossibleRayDistance > 0.0)
                {
                  if(dMinPossibleRayDistance*dMinPossibleRayDistance > m_dBestAnswerSoFarSq)
                    {
                      return FALSE;
                    }
                }
              // Do some additional tests to see if the ray does not intersect with the
              // box of the object.
              double dLeng = 2.0 * (smos_Fabs(dParam) + dSph1Radius);
              SmPoint3d sEndPnt = sLinePnt +  dLeng * sLineVec;
              SmPoint3d sStartPnt = sLinePnt - dLeng * sLineVec;
              SmExtent3d sRayBox(sStartPnt);
              sRayBox.AddPoint3d(sEndPnt);
              sRayBox.ExpandAbsolute(m_d3dTolerance);
              if(sRayBox.AreDisjoint(apBranch[1]->m_sBBox))
                {
                  return FALSE;
                }
              return TRUE;
            }

      // If we are doing a 3D directed minimization we can use the bounding
      // box information to eliminate additional comparisons
      case SM_SO_3D_SIGNED_DIRECTED_MINIMIZE:
            {
              SmPoint3d sSph1Cent, sSph2Cent;
              double dSph1Radius, dSph2Radius;
              apBranch[0]->m_sBBox.ComputeSphereBound(sSph1Cent,dSph1Radius);
              apBranch[1]->m_sBBox.ComputeSphereBound(sSph2Cent,dSph2Radius);
              SmVector3d sLinePnt = sSph1Cent;
              SmPoint3d sLineVec = m_cpOptVectors[0];
              double dParam;
              SER(smgu_LineClosestPoint(sLinePnt,sLineVec,sSph2Cent,dParam));
              SmPoint3d sClPnt = sLinePnt + dParam * sLineVec;
              double dDist = sClPnt.DistanceBetween(sSph2Cent);
              double dSumRadius = dSph1Radius + dSph2Radius;
              // If two spheres don't intersect as projected along the vector
              // return FALSE.
              if(dDist > dSumRadius + m_d3dTolerance)
                {
                  return FALSE;
                }

              SmVector3d sVec = sClPnt - sSph1Cent;
              double dDistTo = sVec.Length();
              if(sVec.Dot(m_cpOptVectors[0]) < 0)
                {
                  dDistTo = - dDistTo;
                }
              double dMinDist = dDistTo - dSph1Radius - dSph2Radius - m_d3dTolerance;
              if(dMinDist*smos_Fabs(dMinDist) > m_dBestAnswerSoFarSq)
                {
                  return FALSE;
                }

              return TRUE;
            }

      default:
          ERR(SM_ERR);  // not handled yet

    } // end switch (m_eSolverOperation)

  return SM_SUCCESS;

} // end SmPolySolver::BranchMayContainAnswers

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
static ULONG sm_FindSubdivisionIndex
  (ULONG        lNumTrees,
   SmTreeNode * apBranch[SM_GS_MAX_TREES])
{
  ULONG lFound = 99999;
  double dMaxSize = 0;
  for (ULONG i=0; i<lNumTrees; i++)
    {
      if(apBranch[i]->m_pChild1 == NULL) continue;

      SmVector3d sVec = apBranch[i]->m_sBBox.GetMax() -
              apBranch[i]->m_sBBox.GetMin();
      double dSize = sVec.LengthSquared();
      if(dSize >= dMaxSize)
        {
          dMaxSize = dSize;
          lFound = i;
        }
    }
  SM_ASSERT (lFound != 99999);
  return lFound;

} // end sm_FindSubdivisionIndex

/*******************************************************************//**
PURPOSE: Find which node should be subdivided and see if we need to
    swap the traversal order to achieve more optimal minimization and
    maximization.

NOTES:
***********************************************************************/
SmStatus SmPolySolver::FindBestSubdivisionNode
  (SmTreeNode * apBranch[SM_GS_MAX_TREES],
   ULONG      & rlBestSubdivisionIndex,
   SmBoolean  & rbSwapTraversalOrder)
  const
{
  rbSwapTraversalOrder = FALSE;
  rlBestSubdivisionIndex = sm_FindSubdivisionIndex(m_lNumTrees,apBranch);

  if(m_lNumTrees == 1) return SM_SUCCESS;

  // See if we can optimize based on distance between boxes

  switch (m_eSolverOperation)
    {

      case SM_SO_MINIMIZE:
      case SM_SO_DIRECTED_MINIMIZE:
      case SM_SO_SIGNED_DIRECTED_MINIMIZE:
      case SM_SO_PROJECTED_MINIMIZE:
            {
              SM_ASSERT(m_lNumTrees == 2);
              ULONG lOtherIndex = 1 - rlBestSubdivisionIndex;
              SmExtent3d *pOther = &apBranch[lOtherIndex]->m_sBBox;
              SmExtent3d *pSub1 = &apBranch[rlBestSubdivisionIndex]->m_pChild1->m_sBBox;
              SmExtent3d *pSub2 = &apBranch[rlBestSubdivisionIndex]->m_pChild2->m_sBBox;
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
              if(bDebugMe)
                {
                  smgfx_SetLook(1,2, 1,0,0); pOther->Draw(NULL); sm_GraphicsLoop() ;
                  smgfx_SetLook(3,4, 0,1,0); pSub1->Draw(NULL);  sm_GraphicsLoop() ;
                  smgfx_SetLook(3,4, 0,0,0); pSub2->Draw(NULL);  sm_GraphicsLoop() ;
                  sm_GraphicsLoop() ;
                }
#endif
              // Use mid point of box to determine which one is best choice
              SmPoint3d sPMid1 = pSub1->Evaluate(0.5,0.5,0.5);
              SmPoint3d sPMid2 = pSub2->Evaluate(0.5,0.5,0.5);
              SmPoint3d sPOther = pOther->Evaluate(0.5,0.5,0.5);
              SmVector3d sV1 = sPMid1 - sPOther;
              SmVector3d sV2 = sPMid2 - sPOther;
              if(sV2.LengthSquared() < sV1.LengthSquared())
                {
                  // Swap nodes in child branches
                  rbSwapTraversalOrder = TRUE;
                }
            }
            break;

      case SM_SO_INTERSECT:
            {
              SM_ASSERT(m_lNumTrees == 2);
              if(rlBestSubdivisionIndex == 1)
                {
                  // No further check if we are subdividing rectangle
                  return SM_SUCCESS;
                }
              // Determine which branch is more closer to the rectangle
              SmExtent3d *pSub1 = &apBranch[0]->m_pChild1->m_sBBox;
              SmExtent3d *pSub2 = &apBranch[0]->m_pChild2->m_sBBox;
              SmPoint3d sPMid1 = pSub1->Evaluate(0.5,0.5,0.5);
              SmPoint3d sPMid2 = pSub2->Evaluate(0.5,0.5,0.5);
              double dDistance1, dDistance2;
              SER(smgu_PlanePointDistance(m_cpOptVectors[0]/*origin*/,
                  m_cpOptVectors[1]/*normal*/,sPMid1,dDistance1));
              SER(smgu_PlanePointDistance(m_cpOptVectors[0]/*origin*/,
                  m_cpOptVectors[1]/*normal*/,sPMid2,dDistance2));
              if(dDistance2 < dDistance1)
                {
                  // Swap nodes in child branches
                  rbSwapTraversalOrder = TRUE;
                }
            }
            break;

      case SM_SO_MAXIMIZE:
      case SM_SO_DIRECTED_MAXIMIZE:
      case SM_SO_PROJECTED_MAXIMIZE:
      default:
      break;

    }

  return SM_SUCCESS;

} // end SmPolySolver::FindBestSubdivisionNode


/*******************************************************************//**
PURPOSE: Subdivide a branch into two sub-branches.

NOTES: For now just try the one with the largest box first.
   If that node is a leaf continue on until find one which is able
   to be subdivided.
***********************************************************************/
SmStatus SmPolySolver::Subdivide
  (SmTreeNode * apOriginal[SM_GS_MAX_TREES],
   SmTreeNode * apChildBranch1[SM_GS_MAX_TREES],
   SmTreeNode * apChildBranch2[SM_GS_MAX_TREES])
{
  ULONG lBestSubdivisionIndex;
  SmBoolean bSwapTraversalOrder;
  SER(FindBestSubdivisionNode(apOriginal,lBestSubdivisionIndex,bSwapTraversalOrder));

  // Do default subdivision
  for (ULONG i=0; i<m_lNumTrees; i++)
    {
      apChildBranch1[i] = apOriginal[i];
      apChildBranch2[i] = apOriginal[i];
    }
  // Now make two subbranches take the children of the subdivision node.
  apChildBranch1[lBestSubdivisionIndex] =
      apOriginal[lBestSubdivisionIndex]->m_pChild1;
  apChildBranch2[lBestSubdivisionIndex] =
      apOriginal[lBestSubdivisionIndex]->m_pChild2;

  // If we need to swap the traversal order to take Child2 first
  if(bSwapTraversalOrder)
    {
      SmTreeNode *pTmp = apChildBranch1[lBestSubdivisionIndex];
      apChildBranch1[lBestSubdivisionIndex] = apChildBranch2[lBestSubdivisionIndex];
      apChildBranch2[lBestSubdivisionIndex] = pTmp;
    }

  return SM_SUCCESS;

} // end SmPolySolver::Subdivide


/*******************************************************************//**
PURPOSE: Solves a subset of the global problem by solving for a
   given branch of a tree.

NOTES: This is a recursive method which traverses down the
   tree (subdivides) until it can do a local solve.
***********************************************************************/
SmStatus SmPolySolver::SolveBranch
  (SmTreeNode * apBranch[SM_GS_MAX_TREES])
{
  if(!BranchMayContainAnswers(apBranch))
    { return SM_SUCCESS;   }

  SmBoolean bNeedsMoreSubdivision;
  SER(LocalSolve(apBranch,bNeedsMoreSubdivision));
  if(!bNeedsMoreSubdivision)
    {
      return SM_SUCCESS;
    }

  SmTreeNode* sChild1[SM_GS_MAX_TREES];
  SmTreeNode* sChild2[SM_GS_MAX_TREES];

  // okay to use smos_MemSet on static (pointer) objects.
  smos_MemSet(sChild1,0,SM_GS_MAX_TREES*sizeof(SmTreeNode*));
  smos_MemSet(sChild2,0,SM_GS_MAX_TREES*sizeof(SmTreeNode*));

  SER(Subdivide(apBranch,sChild1,sChild2));
  SER(SolveBranch(sChild1));
  SER(SolveBranch(sChild2));

  return SM_SUCCESS;

} // end SmPolySolver::SolveBranch

/*******************************************************************//**
PURPOSE: Using an extent which represents some object, search the tree
    and find nodes which satisfy the desired solution.

NOTES:
***********************************************************************/
SmStatus SmPolySolver::SolveTrees
  (SmTree            & rTree1,
   SmTree            & rTree2,
   SmPolyLocalSolver & rLocalSolve)
{
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
  if(bDebugMe)
      this->m_pSolutions->Dump();
#endif
  m_pLocalSolver = &rLocalSolve;
  m_lNumVariables = 0;
  m_apTrees[0] = &rTree1;
  m_apTrees[1] = &rTree2;
  m_lNumTrees = 2;

  if(   m_dBestAnswerSoFarSq == SM_BIG_DOUBLE
     && m_eOperationCategory == SM_OPERATION_MAXIMIZE)
    {
      m_dBestAnswerSoFarSq = 0.0;
    }

  SmTreeNode *sStartBranch[SM_GS_MAX_TREES];
  for (ULONG i=0; i<m_lNumTrees; i++)
    {
      sStartBranch[i] = m_apTrees[i]->GetTopNode();
    }

  SER(SolveBranch(sStartBranch));
  return SM_SUCCESS;

} // end SmPolySolver::SolveTrees

/*******************************************************************//**
PURPOSE: Break one tree down into a list and solve against the other.

NOTES:
***********************************************************************/
SmStatus SmPolySolver::SolveTwoTrees
  (SmTree            & rTree1,         // in : target tree
   SmTree            & rTree2,         // in : target other tree
   SmPolyLocalSolver & rLocalSolve)    // in : oneof class: SmPLSVertexVertex
                                       //                   SmPLSVertexEdge
                                       //                   SmPLSVertexFace
                                       //                   SmPLSEdgeEdge
                                       //                   SmPLSEdgeFace
{
  // break tree1 down into a list of objects
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
      SmObjectList * pObj = sObjects[i];
      sTreeNode.m_sBBox = pObj->m_sBBox;
      sObjectList.m_sBBox = pObj->m_sBBox;
      sObjectList.m_pNext = NULL;
      sObjectList.m_pObject = pObj->m_pObject;
      SER(SolveTrees(sObjectTree,rTree2,rLocalSolve));
    }

  // all done
  return SM_SUCCESS;

} // end SmPolySolver::SolveTwoTrees

/*******************************************************************//**
PURPOSE: This method get the current best answer so far

NOTES:
***********************************************************************/
void SmPolySolver::GetBestAnswerSoFar
  (double & rdBestAnwser,
   double * & rpdBestAnswer)
{
  rpdBestAnswer = NULL;
  if(m_eOperationCategory == SM_OPERATION_MINIMIZE)
    {
      rdBestAnwser = SM_BIG_DOUBLE;
      if(m_dBestAnswerSoFarSq < SM_BIG_DOUBLE)
        {
          rdBestAnwser = smos_Sqrt(m_dBestAnswerSoFarSq);
        }
      rpdBestAnswer = &rdBestAnwser;
    }

} // end SmPolySolver::GetBestAnswerSoFar

/*******************************************************************//**
PURPOSE: Add a solution to the solutions list using an insertion
    sort and a given sort key.

NOTES:
***********************************************************************/
void SmPolySolver::AddSortedSolution
  (const SmSolution & crSolution,     // in : target solution
   SmSortKeyType      eSortKey)       // in : one of
                                      //      SM_SK_BY_SOLUTION_VALUE       = sort by value
                                      //      SM_SK_BY_FIRST_PARAMETER      = sort by first parameter
                                      //      SM_SK_BY_VALUE_WITH_PARAMETER = Sort by value - using parameters to resolve ties
                                      //      SM_SK_BY_FIRST_TWO_PARAMETERS = Sort by first two parameters - second used to resolve ties
                                      //      SM_SK_BY_ALL_PARAMETERS       = Use all parameters up to point of resolving ties

{
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;

  if(crSolution.m_vStart.m_dSolutionValue == SM_BIG_DOUBLE)
      return;
  if(bDebugMe)
    {
      smos_WriteBuffer(_T("\n\nSolution Being Added +++++++++++++++++++++++++++++++++\n"));
      crSolution.Dump();
      smos_WriteBuffer(_T("Current Solution Vector --------------------------------\n"));
      m_pSolutions->Dump();
    }
#endif
  // Add to end just to expand arrays and stuff -- also this works well
  // if it goes onto end of list any how

  if(m_eSolverOperation == SM_SO_MINIMIZE)
    {
      double dThisSolSq = crSolution.m_vStart.m_dSolutionValue*crSolution.m_vStart.m_dSolutionValue;
      if(dThisSolSq < m_dBestAnswerSoFarSq)
        {
          m_dBestAnswerSoFarSq = dThisSolSq;
        }
    }

  if(m_eOperationCategory == SM_OPERATION_SIGNED)
    {
      double dThisSolSq = crSolution.m_vStart.m_dSolutionValue;
      dThisSolSq = smos_Fabs(dThisSolSq) * dThisSolSq;
      if(dThisSolSq < m_dBestAnswerSoFarSq)
        {
          m_dBestAnswerSoFarSq = dThisSolSq;
        }
      else
        { // Stop here if answer is not in range
          double dSign = 1.0;
          if(m_dBestAnswerSoFarSq < 0.0) dSign = -1.0;
          double dBestAns = smos_Sqrt(smos_Fabs(m_dBestAnswerSoFarSq)) * dSign;
          if(dBestAns + m_d3dTolerance < crSolution.m_vStart.m_dSolutionValue)
            {
              return;
            }
        }
    }

  if(   m_eSolverOperation == SM_SO_MINIMIZE
     || m_eSolverOperation == SM_SO_DIRECTED_MINIMIZE
     || m_eSolverOperation == SM_SO_PROJECTED_MINIMIZE
     || m_eSolverOperation == SM_SO_RAYFIRE)
    {
      double dThisSolSq = crSolution.m_vStart.m_dSolutionValue*crSolution.m_vStart.m_dSolutionValue;
      if(dThisSolSq < m_dBestAnswerSoFarSq)
        {
          m_dBestAnswerSoFarSq = dThisSolSq;
        }
      else
        { // Stop here if answer is not in range
          double dBestAns = smos_Sqrt(m_dBestAnswerSoFarSq);
          if(dBestAns + m_d3dTolerance < crSolution.m_vStart.m_dSolutionValue)
            {
              return;
            }
        }
    }

  if(   m_eSolverOperation == SM_SO_MAXIMIZE
     || m_eSolverOperation == SM_SO_DIRECTED_MAXIMIZE
     || m_eSolverOperation == SM_SO_PROJECTED_MAXIMIZE)
    {
      double dThisSolSq = crSolution.m_vStart.m_dSolutionValue*crSolution.m_vStart.m_dSolutionValue;
      if(dThisSolSq > m_dBestAnswerSoFarSq)
        {
          m_dBestAnswerSoFarSq = dThisSolSq;
        }
      else
        { // Stop here if answer is not in range
          double dBestAns = smos_Sqrt(m_dBestAnswerSoFarSq);
          if(dBestAns - m_d3dTolerance > crSolution.m_vStart.m_dSolutionValue)
            {
              return;
            }
        }
    }

  // Now do insertion sort
  SmBoolean bFoundPlace = FALSE;
  ULONG lInsertionIndex = m_pSolutions->GetSize();
  for (ULONG i=0; i<m_pSolutions->GetSize(); i++)
    {
      SmSolution & rTestGS = (*m_pSolutions)[i];
      const SmSolution * pPreferredSol = NULL ;

      // If have duplicate solutions to the problem then don't
      // add it.
      if(IdenticalSolutions(crSolution, rTestGS, pPreferredSol))
        {
          // use the preferredSol
          if(pPreferredSol)
            {
              rTestGS = *pPreferredSol ;
            }
          // save the better solution - improve tolerances
          else if(smos_Fabs(rTestGS.m_vStart.m_dSolutionValue) > smos_Fabs(crSolution.m_vStart.m_dSolutionValue))
            {
#ifdef SM_DEBUG_CODE
              if(bDebugMe)
                {
                  smos_WriteBuffer(_T("\n\nSolution Being Considered +++++++++++++++++++++++++++++++++\n"));
                  crSolution.Dump();
                  smos_WriteBuffer(_T("Current Solution Vector --------------------------------"));
                  m_pSolutions->Dump();
                }
#endif
              rTestGS = crSolution ;
            }

          return; // Don't need to do anything in this case
        }
      switch (eSortKey)
        {

      case SM_SK_BY_SOLUTION_VALUE:
          if(   m_eSolverOperation == SM_SO_MAXIMIZE
             || m_eSolverOperation == SM_SO_DIRECTED_MAXIMIZE
             || m_eSolverOperation == SM_SO_PROJECTED_MAXIMIZE)
            {  // Best answer is biggest
              if(rTestGS.m_vStart.m_dSolutionValue < crSolution.m_vStart.m_dSolutionValue)
                {
                  lInsertionIndex = i;
                  bFoundPlace = TRUE;
                }
            }
          else
            {
              if(rTestGS.m_vStart.m_dSolutionValue > crSolution.m_vStart.m_dSolutionValue)
                {
                  lInsertionIndex = i;
                  bFoundPlace = TRUE;
                }
            }
          break;

      case SM_SK_BY_FIRST_TWO_PARAMETERS:
          if(rTestGS.m_vStart[0] > crSolution.m_vStart[0])
            {
              if(rTestGS.m_vStart[1] > crSolution.m_vStart[1])
                {
                  lInsertionIndex = i;
                  bFoundPlace = TRUE;
                }
            }
          break;

      case SM_SK_BY_FIRST_PARAMETER:
          if(rTestGS.m_vStart[0] > crSolution.m_vStart[0])
            {
              lInsertionIndex = i;
              bFoundPlace = TRUE;
            }
          // Do additional sorting based on intervals not just start and end values
          // This should put single values in front of intervals with same start
          // This will help us to process closed curves better

            {
          double dScale = (1.0 + rTestGS.m_vStart[0]) * SM_EFF_ZERO;
          if(smos_Fabs(rTestGS.m_vStart[0]-crSolution.m_vStart[0]) < dScale )
            {
              if(rTestGS.m_eSolutionType == SM_ST_RANGE_OF_VALUES)
                {
                  if(crSolution.m_eSolutionType == SM_ST_RANGE_OF_VALUES)
                    {
                      // This does actually happen when a curve is coincident
                      // with the seam of a surface
                    }
                  lInsertionIndex = i;
                  bFoundPlace = TRUE;
                }
            }
            }
          break;

      case SM_SK_BY_VALUE_WITH_PARAMETER:
          // If the values are nearly equal then sort by the parameter
          if(!SM_IS_ZERO(rTestGS.m_vStart.m_dSolutionValue-crSolution.m_vStart.m_dSolutionValue))
            {
              // Just use standard by value sorting
              if(   m_eSolverOperation == SM_SO_MAXIMIZE
                 || m_eSolverOperation == SM_SO_DIRECTED_MAXIMIZE
                 || m_eSolverOperation == SM_SO_PROJECTED_MAXIMIZE)
                {  // Best answer is biggest
                  if(rTestGS.m_vStart.m_dSolutionValue < crSolution.m_vStart.m_dSolutionValue)
                    {
                      lInsertionIndex = i;
                      bFoundPlace = TRUE;
                    }
                }
              else
                {
                  if(rTestGS.m_vStart.m_dSolutionValue > crSolution.m_vStart.m_dSolutionValue)
                    {
                      lInsertionIndex = i;
                      bFoundPlace = TRUE;
                    }
                }
              break; // Only break if solution values are distinct otherwise sort by parameters
            }

          // Note that we may intentionally fall through and sort by parameters if
          // values are nearly identical.

      case SM_SK_BY_ALL_PARAMETERS:

        {
          // The following should sort consecutive things together even if there
          // is tangency and periodic cases.  Basically it says that if a tie
          // exists then look at the next parameter to settle the deal.
          SmBoolean bFoundOne = FALSE;
          for (ULONG j=0; j<rTestGS.m_lNumVariables; j++)
            {
              // Take default answer for this parameter
              if(!bFoundOne)
                { // keep first answer
                  if(rTestGS.m_vStart[j] > crSolution.m_vStart[j])
                    {
                      bFoundOne = TRUE;
                      lInsertionIndex = i;
                      bFoundPlace = TRUE;
                    }
                  else if(rTestGS.m_vStart[j] < crSolution.m_vStart[j])
                    {
                      bFoundOne = TRUE;
                      bFoundPlace = FALSE;
                    }
                }

              double dScale = (1.0 + rTestGS.m_vStart[j]) * SM_EFF_ZERO_SQRT;
              if(smos_Fabs(rTestGS.m_vStart[j]-crSolution.m_vStart[j]) > dScale)
                {
                  // If answers are close to being same continue looking until you find
                  // an answer with acceptable deviation or until we have compared all of
                  // them
                  break;
                }

              // If we make it here then we have a tie so far.
              // Look to see if the next one exists differentiates the answers
              if(j+1 < rTestGS.m_lNumVariables)
                {
                  dScale = (1.0 + rTestGS.m_vStart[j+1]) * SM_EFF_ZERO_SQRT;
                  if(smos_Fabs(rTestGS.m_vStart[j+1]-crSolution.m_vStart[j+1]) > dScale)
                    {
                      if(rTestGS.m_vStart[j+1] > crSolution.m_vStart[j+1])
                        {
                          // answer differentiates and should be inserted
                          lInsertionIndex = i;
                          bFoundPlace = TRUE;
                          break;
                        }
                      else
                        { // answer differentiates and should not be inserted
                          bFoundPlace = FALSE;
                          break;
                        }
                    }
                }
            } // for
          }
          break;


      default:
          ERR(SM_ERR);
        } // Switch

      // If found an answer get out of loop
      if(bFoundPlace) break;

    } // For loop to search for place

  if(bFoundPlace)
    {
      // Insert it
      m_pSolutions->InsertAt(lInsertionIndex,crSolution,1);
    }
  else
    { // Just add it to the end
      m_pSolutions->Add(crSolution);
    }

#ifdef SM_DEBUG_CODE
  if(bDebugMe)
    {
      m_pSolutions->Dump();
    }
#endif

  // If requesting only a single answer just keep the best
  if(m_eSolutionRequested == SM_SR_SINGLE)
    {
      // This assumes that we sorted by a meaningful
      // value and that the first answer is the best.
      m_pSolutions->SetSize(1);
    }

  // If we are minimizing or maximizing only keep those
  // answers which are within tolerance of the best.
  //   BUT: m_dSolutionValue is not a measure of the quality,
  //   it's just the parameter along the pick line.
  //   (Should probably revise that.)
  //   This does not allow, for instance, an edge to be picked
  //   (graphically) if the edge is behind a face; the closest
  //   object of whatever type will always be picked.
  //   So just ignore this, and let subsequent processing take care of it.
//  if(   m_eSolverOperation == SM_SO_MINIMIZE
//     || m_eSolverOperation == SM_SO_MAXIMIZE
//     || m_eSolverOperation == SM_SO_RAYFIRE
//     || m_eSolverOperation == SM_SO_3D_SIGNED_DIRECTED_MINIMIZE
//     || m_eSolverOperation == SM_SO_DIRECTED_MINIMIZE
//     || m_eSolverOperation == SM_SO_DIRECTED_MAXIMIZE
//     || m_eSolverOperation == SM_SO_PROJECTED_MINIMIZE
//     || m_eSolverOperation == SM_SO_PROJECTED_MAXIMIZE
//     || m_eSolverOperation == SM_SO_SIGNED_DIRECTED_MINIMIZE
//     || m_eSolverOperation == SM_SO_SIGNED_PIVOT_MINIMIZE)
//    {
//      const SmSolution & rFirst = (*m_pSolutions)[0];
//      SmSolution * aData = m_pSolutions->GetDataArray();
//      SmSolution * pLast = &aData[m_pSolutions->GetSize()-1];
//
//      double dThisTol = m_d3dTolerance ;
//
//      // This is a key location for the correct handling of tolerances.
//      //  We'll have to revisit this bit of code and decide on the correct
//      //  behavior given our model of tolerances for every operation type.  For
//      //  now we'll just address the SM_SO_RAYFIRE operation.  This compare is deciding if
//      //  two points on two different geometries are the same point given
//      //  tolerances.  To do this we need to add together the tolerances
//      //  of the associated Topology Objects that have been intersected by the ray.
//      //  Even for this case there remains a question.  Should the locations of
//      //  the points on topology object be compared, or the locations of the
//      //  points on the ray.  To match old behavior, we check the distance between
//      //  intersection points on the ray.  However, it's unclear which
//      //  choice is the correct one here.
//      if( m_eSolverOperation == SM_SO_RAYFIRE )
//        {
//          double dFirstTol = rFirst.GetObjectTolerance(0) ;
//          double dLastTol  = pLast->GetObjectTolerance(0) ;
//          dThisTol = smos_Max(m_d3dTolerance, dFirstTol + dLastTol) ;
//        }
//
//      while (   m_pSolutions->GetSize() > 1
//             &&   smos_Fabs(rFirst.m_vStart.m_dSolutionValue - pLast->m_vStart.m_dSolutionValue)
//                > dThisTol)
//
//        {
//          m_pSolutions->SetSize(m_pSolutions->GetSize()-1);
//          pLast = &aData[m_pSolutions->GetSize()-1];
//        }
//    }

#ifdef SM_DEBUG_CODE
  if(bDebugMe)
    {
      m_pSolutions->Dump();
    }
#endif

} // end SmPolySolver::AddSortedSolution

/*******************************************************************//**
PURPOSE: Test to see if two solutions are identical.  This
   prevents us from adding more then one solution which is the
   same.  Note that the default definition of an identical
   solution is that the values are the same and that the
   interval values are the same.

NOTES: Note individual solvers may override this
   method if they wish to change the way IdenticalSolutions
   are determined.
***********************************************************************/
SmBoolean SmPolySolver::IdenticalSolutions
  (const SmSolution & crSol1,                            // in : 1st target solution
   const SmSolution & crSol2,                            // in : 2nd target solution
   const SmSolution *&pPreferredSol)                     // out: When identical, set to the preferred solution or NULL for none.
                                                         //      NULL to ignore. default:[NULL]
  const
{
  // init output
  pPreferredSol = NULL ;

  // different Object counts
  if(crSol1.m_lNumObjects != crSol2.m_lNumObjects)
    {
      return FALSE;
    }

  // If the object pointers are stored then they must be equal to be identical
  // solutions.
  for (ULONG j=0; j<crSol1.m_lNumObjects; j++)
    {
      SmObject * pObj = crSol1.m_apObjects[j];
      SmObject * pOtherObj = crSol2.m_apObjects[j];
      if(pObj != pOtherObj)
        {
          SmPolyEdge * pE = SM_CAST_PTR(SmPolyEdge,pObj);
          SmPolyEdge * pOtherE = SM_CAST_PTR(SmPolyEdge,pOtherObj);
          if(pE && pOtherE)
            {
              SmTArray<SmPolyEdge*> sRadialEdges;
              pE->GetAllRadials(sRadialEdges);
              SmBoolean bFoundRadial = FALSE;
              for (ULONG ii=0; ii<sRadialEdges.GetSize(); ii++)
                {
                  if(sRadialEdges[ii] == pOtherE)
                    {
                      bFoundRadial = TRUE; break;
                    }
                }
              if(!bFoundRadial) return FALSE;
            }
          else
            {
              return FALSE;
            }
        }
    }
  // Answers are same if value and all parameters are same
  if(smos_Fabs(crSol1.m_vStart.m_dSolutionValue -
      crSol2.m_vStart.m_dSolutionValue) > SM_EFF_ZERO_SQRT)
    {
      return FALSE;
    }

  // Solutions must have the same number of variables
  if(crSol1.m_lNumVariables != crSol2.m_lNumVariables)
    {
      return FALSE;
    }

  // Answers must have same type of solution
  if(crSol1.m_eSolutionType != crSol2.m_eSolutionType)
    {
      return FALSE;
    }

  for (ULONG i=0; i<crSol1.m_lNumVariables; i++)
    {
      double dScale = smos_Fabs(1.0 + smos_Fabs(crSol1.m_vStart[i])) * SM_EFF_ZERO;
      if(smos_Fabs(crSol1.m_vStart[i]-crSol2.m_vStart[i]) > dScale)
        {
          return FALSE;
        }
      if(crSol1.m_eSolutionType == SM_ST_RANGE_OF_VALUES)
        {
          if(smos_Fabs(crSol1.m_vEnd[i]-crSol2.m_vEnd[i]) > dScale)
            {
              return FALSE;
            }
        }
    }

  // If passed all of these tests then we have identical solutions

  // prefer the tighter solution
  pPreferredSol =  (crSol1.m_vStart.m_dSolutionValue < crSol2.m_vStart.m_dSolutionValue)
                 ? &crSol1
                 : &crSol2 ;
  return TRUE;

} // end SmPolySolver::IdenticalSolutions


// Start plane-polybrep solver
/*******************************************************************//**
PURPOSE: Do local solve between plane and vertex.

NOTES:
***********************************************************************/
SmStatus SmPLSPlaneVertex::SolveIt
  (const SmObject *pObj1,
   const SmObject *)
{
  SmSolution sSol;
  sSol.m_apObjects[0] = NULL;
  sSol.m_apObjects[1] = NULL;

  ULONG lObjectCount = 0;
  SmPolyVertex *pV = SM_CAST_PTR(SmPolyVertex,pObj1); NER(pV);
  sSol.m_apObjects[lObjectCount++] = SM_REINTERPRET_CAST(SmObject*,
                                     SM_CONST_CAST(SmPolyVertex*,pV));

  SmPoint3d sVertPnt = pV->GetPoint();
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
  if(bDebugMe)
    {
      smgfx_SetLook(3,4, 1,0,0); m_vPlanePoint.Draw(); sm_GraphicsLoop();
      smgfx_SetLook(3,4, 0,0,1); sVertPnt.Draw(); sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
#endif

  double *pdBestAnswer = NULL;
  double dBestAnswer = 0.0;
  m_rPolySolver.GetBestAnswerSoFar(dBestAnswer, pdBestAnswer);

  double dDistToPlane;
  SER(smgu_PlanePointDistance(m_vPlanePoint,m_vPlaneNormal,
      sVertPnt,dDistToPlane));

  switch (m_rPolySolver.m_eSolverOperation)
    {
      case SM_SO_MINIMIZE:
          if(dDistToPlane > dBestAnswer)
            {
              return SM_SUCCESS; // no answers in this branch
            }
          break;
      default:
          ERR(SM_ERR);  // not handled yet
    }

  // set output
  sSol.m_eSolutionType           = SM_ST_POLY_SINGLE_VALUE;
  sSol.m_lNumVariables           = 3;
  sSol.m_vStart.m_dSolutionValue = dDistToPlane;
  sSol.m_lNumObjects             = lObjectCount;
  sSol.m_vStart[0]               = sVertPnt.x;
  sSol.m_vStart[1]               = sVertPnt.y;
  sSol.m_vStart[2]               = sVertPnt.z;

  // save output
  m_rPolySolver.AddSortedSolution(sSol,SM_SK_BY_SOLUTION_VALUE);

  return SM_SUCCESS;

} // end SmPLSPlaneVertex::SolveIt

/*******************************************************************//**
PURPOSE: Do local solve between plane and edge.

NOTES:
***********************************************************************/
SmStatus SmPLSPlaneEdge::SolveIt
  (const SmObject *pObj1,
   const SmObject *)
{
  // Normally the point comes first and the edge second.  If
  // we want we can switch the order.
  SmSolution sSol;
  sSol.m_apObjects[0] = NULL;
  sSol.m_apObjects[1] = NULL;

  SmPolyEdge * pPolyEdge;
  ULONG lObjectCount = 0;
  pPolyEdge = SM_CAST_PTR(SmPolyEdge,pObj1); NER(pPolyEdge);
  sSol.m_apObjects[lObjectCount++] = SM_REINTERPRET_CAST(SmObject*,
                                     SM_CONST_CAST(SmPolyEdge*,pPolyEdge));

  double *pdBestAnswer = NULL;
  double dBestAnswer;
  m_rPolySolver.GetBestAnswerSoFar(dBestAnswer, pdBestAnswer);

  SmPoint3d sFoundPnt;
  double dDistance = SM_BIG_DOUBLE;

  switch (m_rPolySolver.m_eSolverOperation)
    {
      case SM_SO_MINIMIZE:
            {
              SmPoint3d sLineStart = pPolyEdge->GetStartPoint();
              SmVector3d sLineVec = pPolyEdge->GetEndPoint() - sLineStart;
              double dLineParam;
              if(   smgu_LinePlaneIntersect(sLineStart,sLineVec,
                                         m_vPlanePoint,m_vPlaneNormal,dLineParam) != SM_SUCCESS
                 || dLineParam <= SM_EFF_ZERO
                 || dLineParam >= 1.0-SM_EFF_ZERO)
                {
                  return SM_SUCCESS; // no answers in this branch
                }
              sFoundPnt = sLineStart + dLineParam*sLineVec;
              dDistance = 0.0;
            }
              break;
      default:
          ERR(SM_ERR);  // not handled yet
    }

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
  if(bDebugMe)
    {
      smgfx_SetLook(1,2, 0,0,1); pPolyEdge->Draw(); sm_GraphicsLoop();
      smgfx_SetLook(3,4, 0,1,0); m_vPlanePoint.Draw(); sm_GraphicsLoop();
      smgfx_SetLook(3,4, 1,0,0); sFoundPnt.Draw(); sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
#endif


  // set output
  sSol.m_eSolutionType           = SM_ST_POLY_SINGLE_VALUE;
  sSol.m_lNumVariables           = 3;
  sSol.m_lNumObjects             = lObjectCount;
  sSol.m_vStart.m_dSolutionValue = dDistance;
  sSol.m_vStart[0]               = sFoundPnt.x;
  sSol.m_vStart[1]               = sFoundPnt.y;
  sSol.m_vStart[2]               = sFoundPnt.z;

  // save output
  m_rPolySolver.AddSortedSolution(sSol,SM_SK_BY_SOLUTION_VALUE);

  return SM_SUCCESS;

} // end SmPLSPlaneEdge::SolveIt

/*******************************************************************//**
PURPOSE: Constructor for the SmPolyPlaneSolver.

Example:
    This constructor should be used for the following solver
    operations:
    SM_SO_MINIMIZE           // Minimize distance from the plane to objecPS

NOTES: Currently, only SM_SO_MINIMIZE is supported.
***********************************************************************/
SmPolyPlaneSolver::SmPolyPlaneSolver
  (SmSolverOperationType     eSolverOperation,
   const SmPoint3d         & crPlanePoint,
   const SmVector3d        & crPlaneNormal,
   SmSolutionRequestedType   eSolutionRequested,
   double                    d3dTolerance,
   double                    dBestAnswerSoFarSq,
   const SmVector3d        * cpOptVectors,
   SmSolutionArray         & rSolutions)
  : SmPolySolver(eSolverOperation,
                 eSolutionRequested,
                 d3dTolerance,
                 dBestAnswerSoFarSq,
                 cpOptVectors,
                 rSolutions),
    m_vPlanePoint(crPlanePoint),
    m_vPlaneNormal(crPlaneNormal)
{
  SE(m_vPlaneNormal.Unitize());

} // end SmPolyPlaneSolver::SmPolyPlaneSolver

/*******************************************************************//**
PURPOSE: Determine if the given branch may contain answers.  In other
   words can we utilize existing information (i.e. bounding boxes) to
   eliminate the possibility of any answers in this branch.

NOTES:
***********************************************************************/
SmBoolean SmPolyPlaneSolver::BranchMayContainAnswers
  (SmTreeNode * apBranch[SM_GS_MAX_TREES])
{
#if SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
  if(bDebugMe)
    {
      smgfx_Erase();
      sm_GraphicsLoop();
      apBranch[0]->m_sBBox.Draw(NULL);
      sm_GraphicsLoop();
    }
#endif
  switch (m_eSolverOperation)
    {

      case SM_SO_MINIMIZE:
            {
              SM_ASSERT(m_lNumTrees == 1);
              // Compute the distance from the bounding sphere to the plane
              SmPoint3d sSphCenter;
              double dSphRadius;
              apBranch[0]->m_sBBox.ComputeSphereBound(sSphCenter,dSphRadius);
              double dDistance;
              SER(smgu_PlanePointDistance(m_vPlanePoint,m_vPlaneNormal,
                  sSphCenter,dDistance));
              dDistance = dDistance-dSphRadius;
              if(dDistance > 0.0) dDistance = 0.0;
              if(dDistance*dDistance > m_dBestAnswerSoFarSq)
                {
                  if(dDistance - m_d3dTolerance >
                      smos_Sqrt(m_dBestAnswerSoFarSq) )
                    {
                      return FALSE; // no answers in this branch
                    }
                }
              return TRUE;
            }
      default:
          ERR(SM_ERR);  // not handled yet

    } // end switch (m_eSolverOperation)

  return SM_SUCCESS;

} // end SmPolyPlaneSolver::BranchMayContainAnswers

/*******************************************************************//**
PURPOSE: Local solver for SmPolyPlaneSolver

NOTES: Local solve needs to be done for all intermediate
    nodes as well as all leaf nodes.  Assume that the local solver will be
    called for both cases.  It should return rbNeedsMoreSubdivision = FALSE
    only when reaching all leaf nodes.
***********************************************************************/
SmStatus SmPolyPlaneSolver::LocalSolve
  (SmTreeNode * apBranch[SM_GS_MAX_TREES],
   SmBoolean  & rbNeedsMoreSubdivision)
{
  rbNeedsMoreSubdivision = FALSE;
  SmTreeNode *pNode1 = apBranch[0];

  SmObjectList* aData1[20];
  SmTArray<SmObjectList*> sObjs1(20,aData1);
  pNode1->GetObjectList(sObjs1);

  for (ULONG i=0; i<sObjs1.GetSize(); i++)
    {
      SmObjectList *pObj1 = sObjs1[i];
      SmExtent3d sDummyBox;
      if(!MayContainAnswer(pObj1->m_sBBox,sDummyBox)) continue;
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
      if(bDebugMe)
        {
          smgfx_Erase();
          smgfx_SetLook(1,2, 1,0,0); pObj1->m_sBBox.Draw(NULL); sm_GraphicsLoop();
          sm_GraphicsLoop();
        }
#endif
      // If made it to here then we have two objects which may contain an answer
      // Invoke the local solver.
      SER(m_pLocalSolver->SolveIt(pObj1->m_pObject,NULL));
    }

  // Keep going down the trees if there are more nodes below
  if(pNode1->m_pChild1 != NULL)
    {
      rbNeedsMoreSubdivision = TRUE;
    }

  return SM_SUCCESS;

} // end SmPolyPlaneSolver::LocalSolve

/*******************************************************************//**
PURPOSE: Return TRUE the bounding box satisfy the criteria
   necessary to satisfy the given solution.

NOTES:
***********************************************************************/
SmBoolean SmPolyPlaneSolver::MayContainAnswer
  (const SmExtent3d & crBox1,
   const SmExtent3d & /*crBox2*/)
  const
{
  switch (m_eSolverOperation)
    {

      case SM_SO_MINIMIZE:
            {
              // Compute the distance from the bounding sphere to the plane
              SmPoint3d sSphCenter;
              double dSphRadius;
              crBox1.ComputeSphereBound(sSphCenter,dSphRadius);
              double dDistance;
              SER(smgu_PlanePointDistance(m_vPlanePoint,m_vPlaneNormal,
                  sSphCenter,dDistance));
              dDistance = dDistance-dSphRadius;
              if(dDistance > 0.0) dDistance = 0.0;
              if(dDistance*dDistance > m_dBestAnswerSoFarSq)
                {
                  if(dDistance - m_d3dTolerance >
                      smos_Sqrt(m_dBestAnswerSoFarSq) )
                        {
                      return FALSE; // no answers in this branch
                    }
                }
              return TRUE;
            }

      default:
          ERR(SM_ERR);  // not handled yet
    }

  return TRUE;

} // end SmPolyPlaneSolver::MayContainAnswer

/*******************************************************************//**
PURPOSE: Using an extent which represents some object, search the tree
    and find nodes which satisfy the desired solution.

NOTES:
***********************************************************************/
SmStatus SmPolyPlaneSolver::SolveTree
  (SmTree            & rTree,
   SmPolyLocalSolver & rLocalSolve)
{
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
  if(bDebugMe)
      this->m_pSolutions->Dump();
#endif
  m_pLocalSolver = &rLocalSolve;
  m_lNumVariables = 0;
  m_apTrees[0] = &rTree;
  m_lNumTrees = 1;

  if(m_dBestAnswerSoFarSq == SM_BIG_DOUBLE &&
      m_eOperationCategory == SM_OPERATION_MAXIMIZE)
        {
      m_dBestAnswerSoFarSq = 0.0;
    }

  SmTreeNode *sStartBranch[SM_GS_MAX_TREES];
  sStartBranch[0] = m_apTrees[0]->GetTopNode();

  SER(SolveBranch(sStartBranch));
  return SM_SUCCESS;

} // end SmPolyPlaneSolver::SolveTree

// End plane-polybrep solver
