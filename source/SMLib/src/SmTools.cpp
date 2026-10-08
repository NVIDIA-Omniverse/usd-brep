// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************//**
* FILE NAME --- SmTools.cpp
* PURPOSE: Source file for SmTools object.
**********************************************************************/

#include "StdAfx.h"

#include <SmTools.h>

#ifndef __SMBREP_H__
#include <SmBrep.h>
#endif

#ifndef __SMVERTEX_H__
#include <SmVertex.h>
#endif

#ifndef __SMEDGE_H__
#include <SmEdge.h>
#endif

#include <SmTopologySolver.h>


/*******************************************************************//**
PURPOSE: Find the 1st vertex from a vertex list
            within tolerance to a target point.

NOTES:
   Tolerance      = PointTolerance + vertex->Tolerance, where
   PointTolerance = input d3dTolerance

   When vertex is found     - output Vertex/TargetPoint distance
   when vertex is not found - output nearestVertex/TargetPoint distance
***********************************************************************/
SmStatus SmTools::FindFirstVertexWithinTolToPnt
  (const SmTArray<SmVertex*> & crVertices,              // in : array of vertices to search
   const SmPoint3d           & crPnt,                   // in : target point
   double                      d3dTolerance,            // in : Tolerance size for crPnt
   SmBoolean                 & rbIsFound,               // out: TRUE = round a vertex within tolerance
   ULONG                     & rlFoundIndex,            // out: index of found vertex
   double                    & rdClosestVertexDistance) // out: targetPoint/FoundVertex distance
{
  // init output
  rdClosestVertexDistance = 0.0;
  rbIsFound               = FALSE;
  rlFoundIndex            = 99999;

  // Each vertex can be either: the closest of any, or within tol, or both.
  double dMinVertexDist     = SM_BIG_DOUBLE;
  double dMinQualifyingDist = SM_BIG_DOUBLE;

  // for every vertex
  ULONG ii, lNumVertices = crVertices.GetSize();
  for (ii=0; ii<lNumVertices; ii++)
    {
      SmVertex *pV   = (SmVertex*)crVertices[ii]; NER(pV);
      double    dTol = pV->GetTolerance() + d3dTolerance;

      // get vertex/targetPoint distance
      double dThisDist = crPnt.DistanceBetween( pV->GetPoint() );

      // when the vertex is extremely close, we can quit.  (It's often 0.0.)
      if ( dThisDist < SM_EFF_ZERO )
        {
          // save the hit and return
          rdClosestVertexDistance = dThisDist;
          rbIsFound               = TRUE;
          rlFoundIndex            = ii;
          return SM_SUCCESS;
        }

      // when the vertex is close enough
      if ( dThisDist < dTol )
        {
          rbIsFound = TRUE;
          if ( dThisDist < dMinQualifyingDist )
            {
              // save the hit
              dMinQualifyingDist = dThisDist;
              rlFoundIndex = ii;
            }
        }

      // save the closest distance value
      if (dThisDist < dMinVertexDist)
        {
          dMinVertexDist = dThisDist;
        }
    } // end iter every vertex

  rdClosestVertexDistance = dMinVertexDist;

  return SM_SUCCESS;

} // end FindFirstVertexWithinTolToPnt




/******************************************************************
PURPOSE: Search a list of edges for an edge coincident to input pCurve
   when pCurve endPoints are known to map to existing Brep vertices.

USER NOTES ---
  Coincident is found by checking for matching endPoint Vertices
  and the distance between curves at an interior point.

  End points are checked by the vertex (pointers) only: so this
  assumes that there are no coincident vertices.

  Returns on the first coincident edge found.  If there is a second
  coincident edge, it will not be reported.

  NOTE: This simple coincidence checker can be modified with the
        more expensive SmCurve::GlobalCoincidenceChecker() checker
        if experience shows that it is needed.
******************************************************************/
SmStatus SmTools::FindCoincidentEdge
  (const SmVertex          * pV1,            // in : Vertex mapping to pCurve StartPoint
   const SmVertex          * pV2,            // in : Vertex mapping to pCurve EndPoint
   const SmCurve           * pCurve,         // in : Target Curve
   const SmExtent1d        & rCurveIvl,      // in : Parameter range of curve (or its edge)
   double                    d3dTolerance,   // in : pCurve and pCurve->EndPoint tolerance
   const SmTArray<SmEdge*> & rEdges,         // in : List of edges to search
   SmBoolean               & rbFound,        // out: TRUE = pCurve is coincident
   SmBoolean               & rbSameOrient,   // out: TRUE = pCurve is oriented SAME as found coincident curve
   ULONG                   & rlFoundIndex,   // out: index in rEdges of found coincident curve
   double                  & rdMaxDeviation, // out: Distance between pCurve/FoundEdge MidPoints
   ULONG                     lStartIdx       // in : optional search startIndex into rEdges, default:[0]
  )                                       
{
  // init output
  rbFound        = FALSE;
  rdMaxDeviation = -SM_BIG_DOUBLE;

  // Locals
  SmBoolean bGoodDrop;
  double dThisDropParam, dDist;
  double dGuessParam = rCurveIvl.GetMid();

  // Check every input edge after lStartIdx.
  ULONG ii, lNumEdges = rEdges.GetSize();
  for (ii=lStartIdx; ii<lNumEdges; ii++)
    {
      SmEdge *pE = (SmEdge*)rEdges[ii];

      if ( pE == pCurve->GetOwner() )
        { continue; }

      // when pE->EndVertices match the input target vertices
      if (   (pE->GetVertex() == pV1 && pE->GetOtherVertex(pV1) == pV2)
          || (pE->GetVertex() == pV2 && pE->GetOtherVertex(pV2) == pV1) )
        {
          // Found case where edge goes between two vertices -
          // now check to see if the curve and edge are coincident

          // get dTol = min distance between distinct curve/edge points
          double dTol = d3dTolerance + pE->GetTolerance();

          // evaluate edge midPoint
          SmCurve *pEdgeCurve = pE->GetCurve();  NER(pEdgeCurve);
          SmExtent1d sEdgeIvl = pE->GetInterval();
          SmPoint3d sMidPt;
          double dEdgeMidParam = sEdgeIvl.Evaluate(0.5);
          SER(pEdgeCurve->EvaluatePoint( dEdgeMidParam, sMidPt ));

          // Drop EdgeMidPoint onto pCurve within tolerance
          SER(pCurve->DropPoint(rCurveIvl,       // in : target curve allowed domain
                                sMidPt,          // in : Point to drop to curve
                                NULL,            // in : For Snapping, NULL=NoSnap, Vec pointing to CrvInside from desired EndPt for snapping.
                                                 //      when DropPt is within dDistTol in cpVecToCrvInside dir of EndPt, Snap to Ivl EndPt.
                                                 //      Vec handles ambiguities and makes sure that curves are not just touching at the ends.
                                dTol,            // in : Used for SnapDist, and SM_SO_INTERSECT sXSectTol3d distance.
                                                 //      sXSectTol3d = dDistTol ;      SM_SO_INTERSECT 
                                                 //      sXSectTol3d = SM_BIG_DOUBLE ; SM_SO_MINIMIZE, SM_SO_NORMALIZE
                                                 //      sSnapTol3d  = dDistTol ;      SM_SO_MINIMIZE, SM_SO_NORMALIZE, SM_SO_INTERSECT    
                                &dGuessParam,    // in : If given, call cheap LocalPointSolve() else call expensive GlobalPointSolve()
                                bGoodDrop,       // out: TRUE = found a drop point
                                dThisDropParam,  // out: found drop curve param
                                dDist)) ;        // out: found drop distance
                                                 // in : SM_SO_MINIMIZE = allow nonNormal drops near endPoints
                                                 //      SM_SO_INTERSECT= like Minimize but point must be within dDistTol
                                                 //      SM_SO_NORMALIZE= exclude nonNormal drops near endPoints
                                                 //      default:[SM_SO_MINIMIZE] to preserve original behavior

          // skip edges that are not within distance of EdgeMidPoint
          if ( !bGoodDrop   ) { continue; }
          if ( dDist > dTol ) { continue; }

          // Here we have a candidate for a coincident curve.
          // Check its direction relative to the input curve.
          // Check tangent directions at the matching points we just found.

          SmVector3d sPVCrv[2];
          SmVector3d sPVEdge[2];
          SER( pEdgeCurve->Evaluate( dEdgeMidParam,  1, TRUE, sPVEdge ));
          SER( pCurve    ->Evaluate( dThisDropParam, 1, TRUE, sPVCrv  ));

          rbSameOrient =   ( sPVCrv[1].Dot( sPVEdge[1] ) > 0.0 )
                                  ? TRUE
                                  : FALSE ;

          // If we've made it this far, then we probably are coincident:
          // exceptions do happen, but they're rare.
          // Use a fairly large number of sample points.
          ULONG lNumSamples = 30;

          double dOtherT0 = sEdgeIvl.GetMin();
          double dOtherT1 = sEdgeIvl.GetMax();
          if ( ! rbSameOrient )
            {
              dOtherT0 = sEdgeIvl.GetMax();
              dOtherT1 = sEdgeIvl.GetMin();
            }
          SmStatus eStat = pCurve->CurveMaxDistanceBetween( rCurveIvl,
              *pEdgeCurve, dOtherT0, dOtherT1, lNumSamples,
              &dTol, dDist, NULL, NULL );

          if ( eStat != SM_SUCCESS ) { continue; }
          if ( dDist > dTol )        { continue; }

          // Looks good.  Set output and return.
          rbFound        = TRUE;
          rdMaxDeviation = dDist;
          rlFoundIndex   = ii;

          return SM_SUCCESS;

        } // end Edge->endVertices match input vertices check
    } // end iter every given edge

  // all done - no coincident curve found
  return SM_SUCCESS;

} // end FindCoincidentEdge


   /*******************************************************************//**
PURPOSE: Find the one point in an array of points that is within
  a given tolerance distance to a target point within the array.

NOTES: 
  Returns an error if two matching points are 
  found within approximately the same distance.

  This function is similar to FindFirstVertexWithinTolToPnt, only works
  for points, not vertices

METHOD ---
  Find the distance between every point and the target point.
  Return success when just one point is found within the tolerance distance.
  Sets the return rdClosestDistanceArg and rlFoundIndexArg values for the
    point closest to the target point in the array.
  Sets rbIsFoundArg == TRUE when the closest point is the only point within 
    the given tolerance distance to the target point. 
  
***********************************************************************/
SmStatus SmTools::FindFirstPointWithinTolToPnt
  (const SmTArray<SmPoint3d> & crPointsArg,  // in : array of points to check
   ULONG nPtIndexArg,                        // in : index of point to match
   double d3dToleranceArg,                   // in : max distance to travel to find a matching point
   SmBoolean & rbIsFoundArg,                 // out: TRUE=found a point within tolerance
   ULONG & rlFoundIndexArg,                  // out: index of found point
   double & rdClosestDistanceArg,            // out: dist between target and found point
   double & rd2ndClosestDistanceArg)         // out: dist between target and next nearest neighbor
{
    

    SmPoint3d sPt( crPointsArg[nPtIndexArg] );
 
    // init return values
    rdClosestDistanceArg    = SM_BIG_DOUBLE ;
    rd2ndClosestDistanceArg = SM_BIG_DOUBLE ;
    rbIsFoundArg            = FALSE;
    rlFoundIndexArg         = 0 ;
    //double dMinVecLenSq     = SM_BIG_DOUBLE;

    // for every input point
    for (ULONG i=0; i<crPointsArg.GetSize(); i++) 
      {
        // skip the target point
        if (i == nPtIndexArg ) continue;

        // get distance**2 from target point to current point
        const SmPoint3d& sCurrPt = crPointsArg[i];
        SmVector3d sVec(sCurrPt - sPt);
        double dVecLenSq = sVec.LengthSquared();

        // save the two closest distances to the target point
        if(rdClosestDistanceArg > dVecLenSq) 
          {
            rd2ndClosestDistanceArg = rdClosestDistanceArg ;
            rdClosestDistanceArg    = dVecLenSq ;
            rlFoundIndexArg         = i ;
          }
        else if(rd2ndClosestDistanceArg > dVecLenSq)
          {
            rd2ndClosestDistanceArg = dVecLenSq ;
          } 
      } // end iter every point

    // convert from dist**2 to dist
    rd2ndClosestDistanceArg = smos_Sqrt(rd2ndClosestDistanceArg) ;
    rdClosestDistanceArg    = smos_Sqrt(rdClosestDistanceArg) ;   

    // decide if we have just 1 hit - don't count no hits and double hits
    // 1 hit defined as a point less than tol away and no other hits
    //   within twice its distance.
    if(rdClosestDistanceArg < d3dToleranceArg)
      {
        if(rdClosestDistanceArg < 0.5 * rd2ndClosestDistanceArg)
          {
            rbIsFoundArg = TRUE ;
          }
        else
          {
            // found a double match
            // return(SM_ERR) ;

            // We allow this to accomodate two loops that touch at a vertex
            rbIsFoundArg = TRUE ;
          }
     } // end found hit check 
         
    // If here we did not find a matching vertex
    return SM_SUCCESS;


} // end FindFirstPointWithinTolToPnt


/*******************************************************************//**
PURPOSE: Given a point on or near an edge, find the index of the nearest
edge and the parameter of the closest point on it.  This is to help us
translate from User picking to pointers to our objects.

NOTES:   There is no distance limit: a point far from every edge still
         returns the nearest edge.
***********************************************************************/

SmStatus SmTools::FindEdge
(
  SmBrep          * pBrep,          ///< [in] : input brep <br>
  const SmPoint3d & crPntOnEdge,    ///< [in] : point 3d on edge <br>
  ULONG           & rlEdgeIndex,    ///< [out]: edge index in brep <br>
  double          & dParam          ///< [out]: parameter on edge  <br>
)
{
  rlEdgeIndex = 0;
  SmTArray<SmEdge*> sEdges;
  pBrep->GetEdges( sEdges );

  // SM_SO_MINIMIZE drops succeed on every edge, so keep the nearest one.
  SmBoolean bFound  = FALSE;
  double    dBestDist = SM_BIG_DOUBLE;
  for(ULONG i = 0; i<sEdges.GetSize(); i++)
  {
    SmEdge *pEdge = sEdges[i];
    SmCurve *pCurve = pEdge->GetCurve();
    double dDistToCurve;
    double dEdgeParam;
    SmBoolean bSuccess;
    SER( pCurve->DropPoint( pEdge->GetInterval(),        // in : target curve allowed domain
         crPntOnEdge,                 // in : Point to drop to curve
         NULL,                        // in : For Snapping, NULL=NoSnap, Vec pointing to CrvInside from desired EndPt for snapping.
                                      //      when DropPt is within dDistTol in cpVecToCrvInside dir of EndPt, Snap to Ivl EndPt.
                                      //      Vec handles ambiguities and makes sure that curves are not just touching at the ends.
         pEdge->GetTolerance()*10.0,  // in : Used for SnapDist, and SM_SO_INTERSECT sXSectTol3d distance.
                                      //      sXSectTol3d = dDistTol ;      SM_SO_INTERSECT 
                                      //      sXSectTol3d = SM_BIG_DOUBLE ; SM_SO_MINIMIZE, SM_SO_NORMALIZE
                                      //      sSnapTol3d  = dDistTol ;      SM_SO_MINIMIZE, SM_SO_NORMALIZE, SM_SO_INTERSECT    
         NULL,                        // in : If given, call cheap LocalPointSolve() else call expensive GlobalPointSolve()
         bSuccess,                    // out: TRUE = found a drop point
         dEdgeParam,                  // out: found drop curve param
         dDistToCurve ) );             // out: found drop distance
                                       // in : SM_SO_MINIMIZE = allow nonNormal drops near endPoints
                                       //      SM_SO_INTERSECT= like Minimize but point must be within dDistTol
                                       //      SM_SO_NORMALIZE= exclude nonNormal drops near endPoints
                                       //      default:[SM_SO_MINIMIZE] to preserve original behavior
    if(bSuccess && dDistToCurve < dBestDist)
    {
      bFound      = TRUE;
      dBestDist   = dDistToCurve;
      rlEdgeIndex = i;
      dParam      = dEdgeParam;
    }
  }
  return bFound ? SM_SUCCESS : SM_ERR;

} // End SmBrep::PickEdge


/*******************************************************************//**
PURPOSE: Given a point which lies on an face, find the pointer of the
face.  This is to help us translate from User picking to pointers
to our objects.

NOTES:
***********************************************************************/

SmStatus SmTools::FindFace
(
  SmBrep          * pBrep,          ///< [in] : input brep <br>
  const SmPoint3d & crPntOnFace,    ///< [in] : point 3d on face <br>
  SmFace         *& rpFace          ///< [out]: pointer to face <br>
)
{
  SmSolutionArray sSolutions;

  SmTopologySolver::BrepPointSolve( pBrep, crPntOnFace, SM_SO_MINIMIZE,
       SM_SR_ALL, pBrep->GetTolerance()*100.0, SM_BIG_DOUBLE, NULL, sSolutions ) ;

  for(ULONG i = 0; i<sSolutions.GetSize(); i++)
  {
    SmSolution & rSol = sSolutions[i];
    SmObject *pObject = (SmObject*)rSol.m_apObjects[0];
    if(pObject->IsKindOf( SmFace_TYPE ))
    {
      rpFace = (SmFace*)pObject;
      return SM_SUCCESS;
    }
  }
  return SM_ERR;

} // End SmBrep::FindFace
