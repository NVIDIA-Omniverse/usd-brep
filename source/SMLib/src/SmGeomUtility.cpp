// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************//**
* FILE NAME --- SmGeomUtility.cpp
* PURPOSE:
**********************************************************************/

#include "StdAfx.h"

#include <SmGeomUtility.h>
#include <SmPolynomial.h>
#include <SmMatrix.h>
#include <SmAxis2Placement.h>
#include <SmContext.h>

#ifdef SM_DEBUG_CODE
#include <SmSphere.h>
#include <SmCircle.h>
#endif // SM_DEBUG_CODE 

/**********************************************************************
contains : SmStatus  smgu_PolygonArea
           SmStatus  smgu_TriangleAreaSquared
           SmStatus  smgu_TriangleDecomposition
           SmStatus  smgu_TrianglePointDistance

           SmStatus  smgu_ParallelogramCrossVectors
           
           SmStatus  smgu_LinePointDistance    // in 2d and 3d
           SmStatus  smgu_LineLineClosestPoint
           SmStatus  smgu_LinePlaneIntersect   // for plane as pt and normal and plane as pt and two tangent vecs
           SmStatus  smgu_LineLine2dIntersect  // in 2d
           SmStatus  smgu_LineLineAtDistance
           SmStatus  smgu_LineClosestPoint

           SmStatus  smgu_SegmentPointDistance
           SmBoolean smgu_IsPointOnSegment
           SmStatus  smgu_SegmentSegmentProjectedIntersect
           SmStatus  smgu_SegmentSegmentPerpspectiveIntersect
           SmStatus  smgu_ArcSegmentPointDistance
           SmStatus  smgu_SegmentSegmentIntersect
           SmStatus  smgu_SegmentSegmentClosestPoint
           double    smgu_SegmentPointAngleDegToPerp
           SmStatus  smgu_SegmentPoint2DSide

           SmStatus  smgu_CircleFromPointsTangents
           SmStatus  smgu_CircleFromPointsOneTangent
           SmStatus  smgu_CircleCenterFrom3Points
           SmStatus  smgu_CircleFrom3Points
           SmStatus  smgu_CirclePlaneIntersect
           SmStatus  smgu_CircleFindSilhouettePoints
           SmBoolean smgu_ConesAreDisjointPerpen

           SmStatus  smgu_LineCoPlanarCircleIntersect
           SmStatus  smgu_LineCoPlanarEllipseIntersect
           SmStatus  smgu_CircleCoPlanarCircleIntersect
           SmStatus  smgu_LineIntersectSphere
           SmStatus  smgu_SphereCircleSphereCircleIntersect
           SmStatus  smgu_IntersectTwoPlanes
           SmStatus  smgu_IntersectThreePlanes
           SmStatus  smgu_CircleCircleApproxMaxGap         // eff: finds max gap from Circle1 to Circle2

           SmStatus  smgu_PlaneFromPoints
           SmPoint3d smgu_PlaneEvaluatePoint
           SmPoint2d smgu_PlaneDropPoint
           SmStatus  smgu_PlanePointDistance

           SmStatus  smgu_PointProjectToPlane
           SmStatus  smgu_VectorProjectToPlane
           SmStatus  smgu_TransformPointToStartPlane
           SmStatus  smgu_PointPerspectiveToPlane
           SmStatus  smgu_VectorPerspectiveToPlane
           SmStatus  smgu_VecLinCombTwoVectors

           SmStatus  smgu_BestFitPointToPoints
           SmStatus  smgu_BestFitLineToPoints
           SmStatus  smgu_BestFitCircleFromPointsTangents
           SmStatus  smgu_BestFitPlaneToPoints
           
           SmStatus  smgu_SphereFrom4Points
           SmStatus  smgu_BoundingSphereFromPointSet

           double    smgu_VecTripleProduct             // rtn: dot(a,cross(b,c))

           SmStatus  smgu_3x3Copy                      // eff: Copy Mat[3][3] to target
           SmStatus  smgu_3x3Mult                      // eff: rtnMat[3][3] = Mat1[3][3] * Mat2[3][3]
           double    smgu_3x3Determinate               // eff: compute 3x3 mat determinate from double array
           double    smgu_3x3Determinate               // eff: compute 3x3 mat determinate from coefficients
           double    smgu_Determinant3Vectors
           SmStatus  smgu_3x3Solve                     // eff: solve Ax=b, A declared A[3][3]
           SmStatus  smgu_3x3Solve                     // eff: solve Ax=b, A declared as 3 row vectors.
           SmStatus  smgu_3x3Inverse                   // eff: Invert A when A is a 3x3 matrix

           SmStatus  smgu_4x4Copy                      // eff: Copy Mat[4][4] to target
           SmStatus  smgu_4x4Mult                      // eff: rtnMat[4][4] = Mat1[4][4] * Mat2[4][4]
           double    smgu_4x4Determinate               // eff: compute 4x4 mat determinate
           SmStatus  smgu_4x4Inverse                   // eff: Invert A when A is a 4x4 matrix
           SmBoolean smgu_4x4AreEqual                  // rtn: TRUE when A == B to given Tol else FALSE

           SmStatus  smgu_SignedCurvature              // rtn: k = sign * |Cross(Wt,Wtt)|/|Wt|**3
           SmStatus  smgu_EvaluateCurveContinuity 
           SmStatus  smgu_FrenetFromDerivatives
           SmStatus  smgu_ClassifyTolerantIntersection 

           SM_EXPORT SmStatus smgu_SolveQuadraticEqn   // eff: solve  Ax**2 + Bx + C = 0                                    
           SM_EXPORT SmStatus smgu_SolveCubicEqn       // eff: solve  Ax**3 + Bx**2 + Cx + D = 0                            
           SM_EXPORT SmStatus smgu_SolveQuarticEqn     // eff: solve  Ax**4 + Bx**3 + Cx**2 + Dx + E = 0                    

           SmStatus  smgu_DirectionalDerivs            // eff: Find surf directional Derivs for a UV dir given UV Derivs 
           SmStatus  smgu_2ndDirectionalDerivs         // eff: Find surf 2nd directional Derivs for two UV dirs given UV Derivs 
           SmStatus  smgu_VolDirectionalDerivs         // eff: Find volume directional Derivs for a UVW dir given UVW Derivs

           SmBoolean smgu_IsDogLeg                     // rtn: TRUE=PointSequence forms a dog-leg shape, FALSE=doesn't

           SmStatus  smgu_InsertKnotsIntoKnotVector
           SmStatus  smgu_GetGrevilleAbscissa

           SmStatus  smgu_AddSorted                    // eff: insert a double value into a sorted double array
           void      smgu_ShellSort_double_array       // eff: sort a double array into ascending order
           SmStatus  smgu_SortOneModified              // eff: restore a sorted array after modifying one elem's value           
           ULONG     smgu_BinSearch                    // eff: helper function for smgu_SortOneModified()
                                         
        - helper functions for SmTArray<TYPE>::ShellSort() method
           SmBoolean smgu_Point3dLessThan              // eff: rtn TRUE when (Pt1 < Pt2) after projecting Pts to common direction vec
           SmBoolean smgu_Point2dLessThan              // eff: rtn TRUE when (Pt1 < Pt2) after projecting Pts to common direction vec
           SmBoolean smgu_DoubleLessThan               // eff: rtn TRUE when (Dbl1 < Dbl2)
           SmBoolean smgu_ULONGLessThan                // eff: rtn TRUE when (ULONG1 < ULONG2)
           SmBoolean smgu_IntLessThan                  // eff: rtn TRUE when (int1 < int2)
           SmBoolean smgu_FabsDoubleLessThan           // eff: rtn TRUE when (Fabs(Dbl1)   < Fabs(Dbl2))
           SmBoolean smgu_IabsIntLessThan              // eff: rtn TRUE when (Iabs(int1)   < Iabs(int2))

 GWC TODOs:
           SM_EXPORT SmStatus smgu_CombineTolerantCircleIntersections  // GWC TODO
           SM_EXPORT SmStatus smgu_CombineTolerantPoints               // GWC TODO
           SM_EXPORT SmStatus smgu_DropPointToEllipse() ;              // GWC TODO
           SM_EXPORT SmStatus smgu_DropPointToCircle() ;               // GWC TODO

**********************************************************************/

/*******************************************************************//**
 HELPER FUNCTIONS
***********************************************************************/

/*******************************************************************//**
PURPOSE: Helper function  for smgu_SegmentSegmentIntersect

NOTES: Add distinct points to the intersection.
       Cull points that are within tolerance of points already on the list.
***********************************************************************/
static SmStatus sm_AddPoint
  (      ULONG     & rlNum,      // i/o: number of points in array
         SmPoint3d   aPoints[2], // i/o: array of distinct points
         double      dTol,       // in : min distance between distinct points
   const SmPoint3d & crPnt)      // in : target point to check
{
  // for every point
  double dTol2 = dTol*dTol;
  for (ULONG i=0; i<rlNum; i++) 
    {
      // quit if a point within tolerance is already on the list
      if( crPnt.DistanceBetweenSquared(aPoints[i]) < dTol2)
          return SM_SUCCESS;
    }

  // error - trying to add more than 2 points to the list
  if (rlNum > 1) 
    { SER(SM_ERR); }

  // add the point
  aPoints[rlNum] = crPnt;
  rlNum++;

  // all done
  return SM_SUCCESS;

} // end sm_AddPoint

/*******************************************************************//**
PURPOSE: Internal helper function for smgu_SegmentSegmentClosestPoint.

NOTES: Add points to the closest point list.
***********************************************************************/

static SmStatus sm_AddClosestPoint
  (double dDist,
   double dParam1,
   double dParam2,
   double dTolerance,
   ULONG & rlNumClosestPoints,
   double adParamSeg1[2],
   double adParamSeg2[2],
   double adDistances[2])
                                     
{
    if (rlNumClosestPoints == 0) {
        adParamSeg1[0] = dParam1;
        adParamSeg2[0] = dParam2;
        adDistances[0] = dDist;
        rlNumClosestPoints = 1;
        return SM_SUCCESS;
    }
    if (dDist > adDistances[rlNumClosestPoints-1]+dTolerance) {
        return SM_SUCCESS;  // Too big of a distance skip it
    }

    if (dDist >= adDistances[rlNumClosestPoints-1]) {
        // Just off the end either skip it or add to second slot.
        if (rlNumClosestPoints == 1) {
            adParamSeg1[1] = dParam1;
            adParamSeg2[1] = dParam2;
            adDistances[1] = dDist;
            rlNumClosestPoints ++;
        }
        return SM_SUCCESS;
    }

    // Must be TRUE if (dDist < adDistances[rlNumClosestPoints-1])  

    if (dDist < adDistances[0]-dTolerance) {
        // Totally new one
        adParamSeg1[0] = dParam1;
        adParamSeg2[0] = dParam2;
        adDistances[0] = dDist;
        rlNumClosestPoints = 1;
        return SM_SUCCESS;
    }

    if (dDist > adDistances[0]) {
        // Must fall between 0 and 1
        adParamSeg1[1] = dParam1;
        adParamSeg2[1] = dParam2;
        adDistances[1] = dDist;
        rlNumClosestPoints = 2;
        return SM_SUCCESS;
    }

    // Final case is where we make 2 from 1
    rlNumClosestPoints = 2;
    adParamSeg1[1] = adParamSeg1[0];
    adParamSeg2[1] = adParamSeg2[0];
    adDistances[1] = adDistances[0];
    adParamSeg1[0] = dParam1;
    adParamSeg2[0] = dParam2;
    adDistances[0] = dDist;

    return SM_SUCCESS;

} // end sm_AddClosestPoint

/*******************************************************************//**
 EXPORTED FUNCTIONS
***********************************************************************/

/*******************************************************************//**
PURPOSE: Compute the area of a polygon projected to a plane.  

NOTES: 
   crNormal must be unit -- otherwise the result will be scaled by its length.

   This routine assumes the point list is closed: it connects the last and
   first points.  The input may or may not be closed (last point == first point);
   it makes no difference.
***********************************************************************/
SM_EXPORT SmStatus smgu_PolygonArea
(
  const SmPoint3d  * sPoints,        // in : ordered array of points bounding polygon
  ULONG              lNumPoints,     // in : number of points in sPoints
  const SmVector3d & crUnitNormal,   // in : unit normal to the polygon's plane
  double           & rdPolygonArea   // out: Area of the polygon
)  
{
  ULONG i;
  SmPoint3d  sCurrPoint, sLastPoint = sPoints[lNumPoints-1];
  SmVector3d sCross;

  double dTotalArea = 0;

  // for every point
  for (i=0; i<lNumPoints; i++)
    {
      sCurrPoint = sPoints[i];

      // Compute the area of a triangle [origin, sCurrPoint, sLastPoint]
      // The area of the triangle projected to the polygon plane is 
      //   1/2 the cross product vector, dotted with the plane normal.
      // The TotalArea is the sum of these partial areas.
      // Plus/minus falls out naturally.

      sCross      = sLastPoint * sCurrPoint;
      dTotalArea += sCross.Dot( crUnitNormal );

      sLastPoint = sCurrPoint;

    } // end iter every point pair

  // set output
  rdPolygonArea = dTotalArea /= 2.0;

  // all done
  return SM_SUCCESS;

} // end smgu_PolygonArea

/*******************************************************************//**
PURPOSE: Compute the area squared of a triangle defined by 3 points

NOTES: 
***********************************************************************/
SM_EXPORT SmStatus smgu_TriangleAreaSquared
(
  const SmPoint3d & rPoint0,             // in : 
  const SmPoint3d & rPoint1,             // in : 
  const SmPoint3d & rPoint2,             // in : 
  double          & rdTriAreaSquared     // out: 
)
{
  double AyBz = (rPoint1.y - rPoint0.y) * (rPoint2.z - rPoint0.z) ;
  double AzBx = (rPoint1.z - rPoint0.z) * (rPoint2.x - rPoint0.x) ;
  double AxBy = (rPoint1.x - rPoint0.x) * (rPoint2.y - rPoint0.y) ;

  double AzBy = (rPoint1.z - rPoint0.z) * (rPoint2.y - rPoint0.y) ;
  double AxBz = (rPoint1.x - rPoint0.x) * (rPoint2.z - rPoint0.z) ;
  double AyBx = (rPoint1.y - rPoint0.y) * (rPoint2.x - rPoint0.x) ;

  rdTriAreaSquared =  (  (AyBz - AzBy) * (AyBz - AzBy)
                       + (AzBx - AxBz) * (AzBx - AxBz)
                       + (AxBy - AyBx) * (AxBy - AyBx))
                     / 4.0 ;

  return SM_SUCCESS;

} // end smgu_TriangleAreaSquared

/*******************************************************************//**
PURPOSE: Decompose a triangle into orthonormal values.
   Given a triangle with a base length, and lengths of its right
   and left leg, determine the height of the point from the base and the
   distance from the joint with the left leg and the base projected to
   the base.  

NOTES: This sort of function is very useful for things like intersection
   of two circles where you know the distance between the center and the
   two radii.

RETURNS --- SM_ERR is returned when dBaseLength is zero
            or the sum of left and right leg lengths is less than the base length
            or 
***********************************************************************/
SM_EXPORT SmStatus smgu_TriangleDecomposition
(
  double dBaseLength,                      // in : triangle base length 
  double dLeftLegLength,                   // in : triangle left leg length
  double dRightLegLength,                  // in : triangle right leg length
  double & rdDistanceAlongBaseOfTopPoint,  // out: u coord of triangleTopPoint[u*BaseVec,v*PerpBaseVec]
  double & rdHeightOfTopPoint              // out: v coord of triangleTopPoint[u*BaseVec,v*PerpBaseVec]
                                           //      in coordinate system centered on leftLeg/Base vertex
                                           //      with uVec pointing along the base line and
                                           //      vVec perpendicular to that pointing towards the top point.
)
{
  // check inputs
  if (SM_IS_ZERO(dBaseLength)) SER(SM_ERR_INVALID_INPUT);

  if (  dLeftLegLength+dRightLegLength 
      < dBaseLength+(SM_EFF_ZERO*(1.0+dBaseLength))) 
    { SER(SM_ERR_INVALID_INPUT); }

  // let Q = dist from right end of base to location on the base under the top point
  // let L**2 = H**2 + (B-Q)**2    
  // let R**2 = H**2 + Q**2
  // then L**2 - R**2 = B**2 - 2BQ
  // and  Q = (L**2 - R**2 - B**2)/2B
  // and  H = Sqrt(R**2 - Q**2)
  // and  rdDistanceAlongBaseOfTopPoint = B-Q
  double q =   (  dLeftLegLength * dLeftLegLength 
                - dRightLegLength * dRightLegLength
                - dBaseLength * dBaseLength ) 
             / ( -2.0 * dBaseLength);

  if (smos_Fabs(q) > dRightLegLength) SER(SM_ERR_INVALID_INPUT);

  rdHeightOfTopPoint            = smos_Sqrt(dRightLegLength*dRightLegLength - q*q);
  rdDistanceAlongBaseOfTopPoint = dBaseLength - q;
  return SM_SUCCESS;

} // end smgu_TriangleDecomposition

/*******************************************************************//**
PURPOSE: Determine the distance from a point to a triangle.

NOTES: 
  Finds the closest approach to the finite triangle: if not in the
  interior, that point is on an edge or a vertex.

  The returned u- and v-parameters, and the closest point, represent
  the closest point on the actual triangle, not the projection of the
  test point to the plane.  (Those values can by found by other routines.)

  If the three points do not form a proper triangle, then they
  represent either a line segment or a single point.  The results
  will be found from that segment or point.
***********************************************************************/
SM_EXPORT SmStatus smgu_TrianglePointDistance
(
  const SmPoint3d & crPt0,             // in: 'origin'
  const SmPoint3d & crPt1,             // in: x-axis point
  const SmPoint3d & crPt2,             // in: 3rd point
  const SmPoint3d & crTestPt,          // in:
  double          & rdDistToTriangle,  // out:
  SmPoint3d       & rClosestPoint,     // out: see Usage Notes
  double          & rdParamU,          // out: see Usage Notes
  double          & rdParamV           // out: see Usage Notes
)
{
  rdDistToTriangle = -1;
  rdParamU = rdParamV = 0;

  // Find the uv-coordinates of the test point w.r.t.
  // the origin (crPt0) and the two vectors from it
  // to crPt1 and crPt2.  That will tell us whether
  // the point projects to the interior, or which side
  // it's off of.
  SmVector3d sVecX    ( crPt1    - crPt0 );
  SmVector3d sVecY    ( crPt2    - crPt0 );
  SmVector3d sTestVec ( crTestPt - crPt0 );

  SmPoint2d  sUV;
  SmStatus eStat = smgu_VecLinCombTwoVectors( sVecX, sVecY, sTestVec, sUV );

  if ( eStat != SM_SUCCESS )
  {
      // This means that the three points did not form a basis,
      // so they're collinear.  We can still return something meaningful:
      // the u-value is on the sVecX segment, and v on sVecY.

      double dDist1,  dDist2;
      // This routine behaves ok if pts are coincident:
      smgu_SegmentPointDistance( crPt0, crPt1, crTestPt, dDist1, rdParamU );
      smgu_SegmentPointDistance( crPt0, crPt2, crTestPt, dDist2, rdParamV );

      if ( dDist1 <= dDist2 )
      {
          rdDistToTriangle = dDist1;
          rClosestPoint = crPt0 + rdParamU * sVecX;
      }
      else
      {
          rdDistToTriangle = dDist2;
          rClosestPoint = crPt0 + rdParamV * sVecY;
      }
      return SM_SUCCESS;
  }

  // It's a proper triangle.
  // The coordinates in sUV tell the whole story.
  // The test point projects to the interior of the triangle
  // if and only if U > 0, V > 0, and U+V < 1.
  // If U < 0, then it's beyond sVecY; if V < 0 it's beyond sVecX,
  // and if U+V > 1, then it's beyond the (crPt1-crPt2) segment.

  rdParamU = sUV.x;
  rdParamV = sUV.y;

  // All of the if-else's boil down to:
  // if it's beyond the (crPt1-crPt2) segment ( U>0, V>0, and U+V>1 ),
  // then drop it to the (crPt1-crPt2) segment,
  // else just clamp both U and V to [0,1].
  if ( rdParamU <= 0 || rdParamV <= 0 )
  {
      if ( rdParamU < 0 ) { rdParamU = 0; }
      if ( rdParamU > 1 ) { rdParamU = 1; }
      if ( rdParamV < 0 ) { rdParamV = 0; }
      if ( rdParamV > 1 ) { rdParamV = 1; }
  }
  else if ( rdParamU + rdParamV > 1 )
  {
      smgu_SegmentPointDistance( crPt1, crPt2, crTestPt, rdDistToTriangle, rdParamV );
      rdParamU = 1 - rdParamV;
  }

  rClosestPoint    = crPt0 + rdParamU * sVecX + rdParamV * sVecY;
  rdDistToTriangle = crTestPt.DistanceBetween( rClosestPoint );

#ifdef SM_DEBUG_CODE
  SmBoolean bDebugMe = FALSE;
  if ( bDebugMe )
  {
      if ( FALSE )
        { smgfx_Erase(); }
      smgfx_SetLook( 2,2, 1,0,0 ); crPt0.DrawPointToPoint( crPt1 ); sm_GraphicsLoop();
      smgfx_SetLook( 2,2, 0,0,1 ); crPt0.DrawPointToPoint( crPt2 ); sm_GraphicsLoop();
      smgfx_SetLook( 2,2, 1,1,0 ); crPt1.DrawPointToPoint( crPt2 ); sm_GraphicsLoop();
      smgfx_SetLook( 2,2, 1,0,1 ); crTestPt.DrawPointToPoint( rClosestPoint ); sm_GraphicsLoop();
      sm_GraphicsLoop();
  }
#endif

  return SM_SUCCESS;

} // end smgu_TrianglePointDistance

/*******************************************************************//**
PURPOSE: Compute Cross Vectors of a Parallelogram.  These vectors go
    from one corner to the other.  

NOTES: 
***********************************************************************/
SM_EXPORT SmStatus smgu_ParallelogramCrossVectors
(
  const SmVector3d & crBaseVector,              // in :
  const SmVector3d & crSideVector,              // in :
  double             dDistanceTopToBottom,      // in :
  double             dDistanceBetweenSides,     // in :
  SmVector3d       & rVecLowLeftToUpRight,      // out:
  SmVector3d       & rVecLowRightToUpLeft       // out:
)
{
    SmVector3d sBase = crBaseVector;
    SmVector3d sSide = crSideVector;
    SER(sBase.Unitize());
    SER(sSide.Unitize());
    double dAngle;
    SER(sBase.AngleBetween(sSide,dAngle));
    double dSineAngle = smos_Sine(dAngle);
    double dLengSide = dDistanceTopToBottom / dSineAngle;
    double dLengBot = dDistanceBetweenSides / dSineAngle;
    rVecLowLeftToUpRight = sBase * dLengBot + sSide * dLengSide;
    rVecLowRightToUpLeft = sSide * dLengSide - sBase * dLengBot;
    return SM_SUCCESS;

} // end smgu_ParallelogramCrossVectors

/*******************************************************************//**
PURPOSE: Find the distance between an infinite line and a point in 2d.

NOTES: 
***********************************************************************/
SM_EXPORT SmStatus smgu_LinePointDistance
(
  const SmPoint2d  & crLinePnt,     // in : 
  const SmVector2d & crLineVec,     // in : 
  const SmPoint2d  & crTestPoint,   // in : 
  double           & rdDistance,    // out: 
  double           * pdParam        // out: NULL to ignore
)      
{
  double dParam;

  // find projection of Point onto Vector
  double dLineVecLengthSquared = crLineVec.LengthSquared();
  if ( dLineVecLengthSquared < SM_EFF_ZERO_SQ )
      // Just return the 'beginning' of the degenerate line.  [bd 060319]
    { dParam = 0.0; }
  else
    { dParam = (crTestPoint - crLinePnt).Dot( crLineVec ) / dLineVecLengthSquared; }

  // get projPoint/TestPoint distance
  SmPoint2d sPnt = crLinePnt + dParam * crLineVec;
  rdDistance = sPnt.DistanceBetween(crTestPoint);

  // set output
  if ( pdParam != NULL ) { *pdParam = dParam; }

  return SM_SUCCESS;

} // end smgu_LinePointDistance

/*******************************************************************//**
PURPOSE: Find the min distance between an infinite line and a point in 3d.

NOTES: 
***********************************************************************/
SM_EXPORT SmStatus smgu_LinePointDistance
(
  const SmPoint3d  & crLinePnt,     // in : Point on line
  const SmVector3d & crLineVec,     // in : any sized vector in tangent direction
  const SmPoint3d  & crTestPoint,   // in : Point to test
  double           & rdDistance,    // out: Min dist from crTestPoint to Line(LinePnt,Linevec)
  double           * pdParam        // out: NULL to ignore
)      
{
  double dParam;

  // find projection of Point onto Vector
  double dLineVecLengthSquared = crLineVec.LengthSquared();
  if ( dLineVecLengthSquared < SM_EFF_ZERO_SQ )
      // Just return the 'beginning' of the degenerate line.  [bd 060319]
    { dParam = 0.0; }
  else
    { dParam = (crTestPoint - crLinePnt).Dot( crLineVec ) / dLineVecLengthSquared; }

  // get projPoint/TestPoint distance
  SmPoint3d sPnt = crLinePnt + dParam * crLineVec;
  rdDistance = sPnt.DistanceBetween(crTestPoint);

  // set output
  if ( pdParam != NULL ) { *pdParam = dParam; }

  return SM_SUCCESS;

} // end smgu_LinePointDistance

/*******************************************************************//**
PURPOSE: Find the closest point between two lines in terms of the 
    parameters on the lines.  
    
Example:
    Parameters will be in terms of the current parameterization   
         SmPoint3d sPntOnLine1 = crLinePnt1 + rdLineParam1 * crLineVec1;

NOTES: The lines do not have to have unit vectors but must not
    be of zero length.  If lines are parallel or one of vectors 
    is zero then an error is returned.

IMPLEMENTATION NOTES ---
    This could be done more simply using analytic means:
    writing the line equations as L1(t1) and L2(t2), set the difference
    vector between those points to be perpendicular to both of the line
    vectors.  This gives a linear 2x2
     ( a00  a01 ) ( t1 ) = ( b0 )
     ( a10  a11 ) ( t2 )   ( b1 )
    where a00 = dot(v1,v1), a01 = -dot(v1,v2), a10 = -a01, a11 = -dot(v2,v2),
    b0 = dot(delta,v1), b1 = dot(delta,v2), where delta = p2 - p1.
    This is more straightforward than the current algorithm, but in
    practice, the current algorithm actually gives more precise results.
***********************************************************************/
SM_EXPORT SmStatus smgu_LineLineClosestPoint
(
  const SmPoint3d  & crLinePnt1,    // in : 1st line startPoint
  const SmVector3d & crLineVec1,    // in : 1st line vector
  const SmPoint3d  & crLinePnt2,    // in : 2nd line startPoint
  const SmVector3d & crLineVec2,    // in : 2nd line vector    
  double           & rdLine1Param,  // out: nearestPt param on Line1 = Start1 + param * Vec1
  double           & rdLine2Param   // out: nearestPt param on Line2 = Start2 + param * Vec2
)  
{
  // err - zero length vectors
  if (   crLineVec1.LengthSquared() < SM_EFF_ZERO_SQ
      || crLineVec2.LengthSquared() < SM_EFF_ZERO_SQ) 
    {
      return SM_ERR;
    }

  // error - parallel or coincident lines
  SmVector3d sCross = crLineVec1 * crLineVec2;
  if (sCross.LengthSquared() < SM_EFF_ZERO_SQ) 
    {
      return SM_ERR;
    }

  // get minimum distance vector direction
  SER(sCross.Unitize()); // scale back to good size

  // get vectors perp to line and minDistance vector 
  SmVector3d sPlaneV1 = sCross * crLineVec1;
  SmVector3d sPlaneV2 = sCross * crLineVec2;
  SER(sPlaneV1.Unitize());
  SER(sPlaneV2.Unitize());

  // minDist happens at XSect of lines with planes through otherPoint using the
  // perpVectors as normals
  if(SM_SUCCESS != smgu_LinePlaneIntersect(crLinePnt1,crLineVec1,
                                           crLinePnt2,sPlaneV2,
                                           rdLine1Param)) 
    { return SM_ERR; }

  if(SM_SUCCESS != smgu_LinePlaneIntersect(crLinePnt2,crLineVec2,
                                           crLinePnt1,sPlaneV1,
                                           rdLine2Param)) 
    { return SM_ERR; }

  // all done
  return SM_SUCCESS;

} // end smgu_LineLineClosestPoint

/*******************************************************************//**
PURPOSE: Intersect a line and a plane and return the result as the
    natural parameter of the line.

NOTES: An error will be returned if either of the vectors is
    of zero length or if the line is parallel to the plane.
***********************************************************************/
SM_EXPORT SmStatus smgu_LinePlaneIntersect
(
  const SmPoint3d  & crLinePnt,       // in : Line StartPoint
  const SmVector3d & crLineVec,       // in : Line Vector
  const SmPoint3d  & crPlanePnt,      // in : Plane Point
  const SmVector3d & crPlaneNorm,     // in : Plane Normal
  double           & rdLineParam      // out: intersect param on Line = Start + param*Vec
)     
{
  // return err for zero length vectors
  ZERO_VEC_ER(crLineVec);
  ZERO_VEC_ER(crPlaneNorm);

  double dPlaneEqD = - crPlaneNorm.Dot(crPlanePnt);
  double dTol      = SM_EFF_ZERO * crLineVec.GetMaxDimension() + SM_EFF_ZERO_SQ;
  double dDenom    = crLineVec.Dot(crPlaneNorm);

  // return err for parallel cases
  if (smos_Fabs(dDenom) < dTol) return SM_ERR;  // Parallel case

  // compute intersection point line param
  rdLineParam = - (dPlaneEqD + crLinePnt.Dot(crPlaneNorm)) / dDenom;

  // all done
  return SM_SUCCESS;

} // end smgu_LinePlaneIntersect

/*******************************************************************//**
PURPOSE: Intersect a line and a plane and return the result as the
    natural parameters of the line and the plane.

NOTES: An error will be returned if any of the vectors is
    of zero length or if the line is parallel to the plane.
***********************************************************************/
SM_EXPORT SmStatus smgu_LinePlaneIntersect
(
  const SmPoint3d  & crLinePt,        // in : Line StartPoint
  const SmVector3d & crLineVec,       // in : Line Vector
  const SmPoint3d  & crPlanePt,       // in : Plane Point
  const SmVector3d & crPlaneSu,       // in : Plane U-Derivative
  const SmVector3d & crPlaneSv,       // in : Plane V-Derivative
  double           & rdLineParam,     // out: Line  t-param of intersection
  double           & rdPlaneU,        // out: Plane u-param of intersection
  double           & rdPlaneV         // out: Plane v-param of intersection
)        
{
  // return err for zero length vectors
  ZERO_VEC_ER( crLineVec );
  ZERO_VEC_ER( crPlaneSu );
  ZERO_VEC_ER( crPlaneSv );

  SmVector3d sNorm( crPlaneSu * crPlaneSv );
  ZERO_VEC_ER( sNorm );

  double dTol = SM_EFF_ZERO * crLineVec.GetMaxDimension() + SM_EFF_ZERO_SQ;

  SmVector3d sDiff( crLinePt - crPlanePt );

  double dDenom = crLineVec.Dot( sNorm );

  if ( smos_Fabs(dDenom) < dTol ) { return SM_ERR; }  // Parallel case

  // Cramer's rule:
  double dNumerU =  ( sDiff * crPlaneSv ).Dot( crLineVec );
  double dNumerV =  ( crPlaneSu * sDiff ).Dot( crLineVec );
  double dNumerT = -(       sNorm       ).Dot( sDiff );

  rdLineParam = dNumerT / dDenom;
  rdPlaneU    = dNumerU / dDenom;
  rdPlaneV    = dNumerV / dDenom;

  // all done
  return SM_SUCCESS;

} // end smgu_LinePlaneIntersect

/*******************************************************************//**
PURPOSE: Intersect two lines in 2d and output the results as the
    pair of natural parameters of the lines.  

NOTES: An error will be returned if either of the vectors are
    of zero length or if the lines are parallel.
***********************************************************************/
SM_EXPORT SmStatus smgu_LineLine2dIntersect
(
  const SmPoint2d  & crLinePnt1,    // in : 1st line startPoint
  const SmVector2d & crLineVec1,    // in : 1st line vector
  const SmPoint2d  & crLinePnt2,    // in : 2nd line startPoint
  const SmVector2d & crLineVec2,    // in : 2nd line vector    
  double           & rdLine1Param,  // out: Intersect Pt param on Line1 = Start1 + param * Vec1
  double           & rdLine2Param   // out: Intersect Pt param on Line2 = Start2 + param * Vec2
)  
{
  // return err for zero length vectors
  ZERO_VEC_ER(crLineVec1);
  ZERO_VEC_ER(crLineVec2);

  // locals
  double dMax   = smos_4Max(crLinePnt1.GetMaxDimension(),
                            crLineVec1.GetMaxDimension(),
                            crLinePnt2.GetMaxDimension(),
                            crLineVec2.GetMaxDimension()) ;
  double dTol   = SM_EFF_ZERO * (1.0 + dMax) ;
  double dDenom = -crLineVec1.x * crLineVec2.y + crLineVec1.y*crLineVec2.x ;
  SmVector2d sP01(crLinePnt1.x - crLinePnt2.x, crLinePnt1.y - crLinePnt2.y) ;

  // return err for parallel cases
  if (smos_Fabs(dDenom) < dTol) return SM_ERR;  // Parallel case

  // compute intersection points line param
  rdLine1Param = (crLineVec2.y * sP01.x - crLineVec2.x * sP01.y) / dDenom ;
  rdLine2Param = (crLineVec1.y * sP01.x - crLineVec1.x * sP01.y) / dDenom ;

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  if(bDebugMe)
    {
      SmPoint2d sT1   = crLinePnt1 + rdLine1Param * crLineVec1 ;
      SmPoint2d sT2   = crLinePnt2 + rdLine2Param * crLineVec2 ;
      SmPoint2d sDiff = sT1 - sT2 ;
      double    sLeng = sDiff.Length() ;
      SM_ASSERT(sLeng < dTol) ; 
    }
#endif // SM_DEBUG_CODE

  // all done
  return SM_SUCCESS;

} // end smgu_LinePlaneIntersect

/*******************************************************************//**
PURPOSE: This function computes the points on two lines which are
    at a given distance from each other and have equal angles.  There
    are always 4 answers to this problem.

NOTES: If the distance between the lines is greater than
    the desired at distance then an error will be returned.
***********************************************************************/
SM_EXPORT SmStatus smgu_LineLineAtDistance
(
  const SmPoint3d  & crLinePnt1,  // in : Line 1 point
  const SmVector3d & crLineVec1,  // in : Line 1 NonZero Tangent
  const SmPoint3d  & crLinePnt2,  // in : Line 2 point
  const SmVector3d & crLineVec2,  // in : Line 2 NonZero Tangent
  double dAtDistance,             // in : distance between solutions
  double dLine1Params[4],         // out: 
  double dLine2Params[4]          // out:
)          
{
  double dCl1, dCl2;
  if (smgu_LineLineClosestPoint(crLinePnt1,crLineVec1,crLinePnt2,crLineVec2,
      dCl1,dCl2) != SM_SUCCESS) { 
      return SM_ERR;
  }
  SmPoint3d sP1 = crLinePnt1 + dCl1 * crLineVec1;
  SmPoint3d sP2 = crLinePnt2 + dCl2 * crLineVec2;
  double dDistBetween = sP1.DistanceBetween(sP2);
  // Projection of distance into plane whose normal is cross
  // of two line vectors.
  double dBeta;
  SER(crLineVec1.AngleBetween(crLineVec2,dBeta));
  dBeta /= 2.0;
  // Find 1/2 distance when projected into plane
  if (dDistBetween > dAtDistance) {
      return SM_ERR;  // sorry we fail.
  }
  double dHalfProjAtDistance = smos_Sqrt(dAtDistance*dAtDistance - 
                                  dDistBetween*dDistBetween) / 2.0;
  double dDistAlongA = dHalfProjAtDistance / smos_Sine(dBeta);
  double dDistAlongB = dHalfProjAtDistance / smos_Sine(SM_PI-dBeta);
  double dScale1 = 1.0 / crLineVec1.Length();
  double dParamDistAOn1 = dDistAlongA * dScale1;
  double dParamDistBOn1 = dDistAlongB * dScale1;
  double dScale2 = 1.0 / crLineVec2.Length();
  double dParamDistAOn2 = dDistAlongA * dScale2;
  double dParamDistBOn2 = dDistAlongB * dScale2;
  
  dLine1Params[0] = dCl1 + dParamDistAOn1;
  dLine2Params[0] = dCl2 + dParamDistAOn2;
  dLine1Params[1] = dCl1 - dParamDistAOn1;
  dLine2Params[1] = dCl2 - dParamDistAOn2;
  dLine1Params[2] = dCl1 + dParamDistBOn1;
  dLine2Params[2] = dCl2 + dParamDistBOn2;
  dLine1Params[3] = dCl1 - dParamDistBOn1;
  dLine2Params[3] = dCl2 - dParamDistBOn2;
  return SM_SUCCESS;

} // end smgu_LineLineAtDistance

/*******************************************************************//**
PURPOSE: Find the closest point to an infinite line.

NOTES:  Note that this method does not assume that crLineVec is
    a unit vector.  
***********************************************************************/
SM_EXPORT SmStatus smgu_LineClosestPoint
(
  const SmPoint3d  & crLinePnt,       // in : origin of the line
  const SmVector3d & crLineVec,       // in : direction vector of the line
  const SmPoint3d  & crTestPoint,     // in : Point to project to line
  double           & rdLineParameter  // out: closest param on line where line = crLinePnt + param * crLineVec
)
{
  SmVector3d sVecToPt              = crTestPoint - crLinePnt;
  double     dLineVecLengthSquared = crLineVec.LengthSquared();
  
  // find projection of Point onto Vector
  if ( dLineVecLengthSquared < SM_EFF_ZERO_SQ )
      // Just return the 'beginning' of the degenerate line.  [bd 060319]
    { 
      rdLineParameter = 0.0; 
    }
  else
    { 
      rdLineParameter = sVecToPt.Dot( crLineVec ) / dLineVecLengthSquared; 
    }
  
  return SM_SUCCESS;

} // end smgu_LineClosestPoint

/*******************************************************************//**
PURPOSE: Find the closest point between a segment and a point.

NOTES: Not the same as distance between a line and a point
    because distances to TestPoints beyond the ends of the segment are
    measured from the nearest segment endPoint.

    If test point does not project to the interior of the segment,
    rdParameter will be clamped to [0,1] and rdDistance will be from
    the test point to the closer segment end point.

    Always returns SM_SUCCESS
***********************************************************************/
SM_EXPORT SmStatus smgu_SegmentPointDistance
(
  const SmPoint3d & crLineStart,   // in : Segment Start
  const SmPoint3d & crLineEnd,     // in : Segment End
  const SmPoint3d & crTestPoint,   // in : TestPoint
  double          & rdDistance,    // out: TestPoint/Segment distance
  double          & rdParameter    // out: 0.0 to 1.0, when value is 0 or 1, TestPoint may 
                                   //     project to line point outside the segment
)
{
  // locals
  SmVector3d sLineVec = crLineEnd - crLineStart;
  double dParam;

  // get closest point on infinite line - smgu_LineClosestPoint only returns SM_SUCCESS
  SER( smgu_LineClosestPoint( crLineStart, sLineVec, crTestPoint, dParam ));

  // clip to specified endPoints
  if ( dParam < 0.0 ) dParam = 0.0;
  if ( dParam > 1.0 ) dParam = 1.0;

  // set output
  rdParameter         = dParam;
  SmPoint3d sFoundPnt = crLineStart + dParam * sLineVec;
  rdDistance          = sFoundPnt.DistanceBetween( crTestPoint );

  // all done
  return SM_SUCCESS;

} // end smgu_SegmentPointDistance

/*******************************************************************//**
PURPOSE: Test whether a point is within tolerance of a line segment.

NOTES: 
 - If test point does not project to the interior of the segment,
   rdParameter will be clamped to [0,1] and rdDistance will be from
   the test point to the closer segment end point.
***********************************************************************/
SM_EXPORT SmBoolean smgu_IsPointOnSegment
(
  const SmPoint3d & crLineStart, // in : line start point
  const SmPoint3d & crLineEnd,   // in : line end point
  const SmPoint3d & crTestPoint, // in : target point
  double            dTolerance,  // in : min dist between distinct points
  double          & rdDistance,  // out: point/segment distance
  double          & rdParameter  // out: param of nearest line point 
                                 //      of line = (1-param)*Start+(param)*End
)                              
{
  SER( smgu_SegmentPointDistance ( crLineStart, crLineEnd, crTestPoint,
                                   rdDistance, rdParameter ));

  return ( rdDistance <= dTolerance );
} // end smgu_IsPointOnSegment

/*******************************************************************//**
PURPOSE: Intersect two parallel projected segments.

NOTES: 
  1. crPlaneNormal does not have to be unitized.
  2. While there is both a smgu_SegmentSegmentProjectedIntersect and
                             smgu_SegmentSegmentPerspectiveIntersect, 
     there is no smgu_SegmentSegmentRotatedProjectIntersect yet
     because lines project to parabolas and I didn't take the time
     to implement that.
***********************************************************************/
SM_EXPORT SmStatus smgu_SegmentSegmentProjectedIntersect
(
  const SmPoint3d  & crStart1,                  // in :
  const SmPoint3d  & crEnd1,                    // in :
  const SmPoint3d  & crStart2,                  // in :
  const SmPoint3d  & crEnd2,                    // in :
  const SmVector3d & crPlaneNormal,             // in :
  double             dDistanceTolerance,        // in : Tolerance to use in intersection
  ULONG            & rlNumInt,                  // out:  0 - no intersections, 
                                                //       1 - crossing or ends touching intersection 
                                                //       2 - coincident segments.
  double             aParams1[2],               // out:
  double             aParams2[2],               // out:
  double             aProjectedDistances[2]     // out:
)
{
  double dDistanceTolerance2 = dDistanceTolerance*dDistanceTolerance;
  rlNumInt = 0;
  SmPoint3d sStart1, sEnd1, sStart2, sEnd2;

  sStart1 = crStart1;

  // project line end points to a common plane - the one containint sStart1
  SER(smgu_PointProjectToPlane(crEnd1,   sStart1, crPlaneNormal, sEnd1));
  SER(smgu_PointProjectToPlane(crStart2, sStart1, crPlaneNormal, sStart2));
  SER(smgu_PointProjectToPlane(crEnd2,   sStart1, crPlaneNormal, sEnd2));

  // Duplicated Code! 
  // the rest of this function is the same as smgu_SegmentSegmentProjectedIntersect code
  //   make any changes needed in both functions 

  // when curve one degenerates to a point
  if (sStart1.DistanceBetweenSquared(sEnd1) < dDistanceTolerance2) 
    { 
      // when curve 2 degenerates to a point
      if (sStart2.DistanceBetweenSquared(sEnd2) < dDistanceTolerance2) 
        {
          // Two possible cases - first they are disjoint - second they
          // fully intersect.
          if (sStart1.DistanceBetweenSquared(sStart2) > dDistanceTolerance2) 
            {
              return SM_SUCCESS;
            }
          rlNumInt = 2;
          aProjectedDistances[0] = aProjectedDistances[1] = sStart1.DistanceBetween(sStart2);
          aParams1[0] = aParams2[0] = 0.0;
          aParams1[1] = aParams2[1] = 1.0;
          return SM_SUCCESS;
        } // end degenerate curve 2 branch
      else // curve 2 is a line
        {
          // check point/line distance
          double dDist, dParam;
          SER(smgu_SegmentPointDistance(sStart2,sEnd2,sStart1,dDist,dParam));
          if (dDist > dDistanceTolerance) 
            {
              return SM_SUCCESS;
            }
          rlNumInt = 1;
          aProjectedDistances[0] = dDist;
          aParams1[0] = 0.0;
          aParams2[0] = dParam;
          return SM_SUCCESS;
        }  // end curve2 is a line branch
    } // end degenerate curve1 branch
  else // curve 1 is a line
    {
      // curve 2 is degenerate
      if (sStart2.DistanceBetweenSquared(sEnd2) < dDistanceTolerance2) 
        {
          // get point/curve distance
          double dDist, dParam;
          SER(smgu_SegmentPointDistance(sStart1,sEnd1,sStart2,dDist,dParam));
          if (dDist > dDistanceTolerance) 
            {
              return SM_SUCCESS;
            }
          rlNumInt = 1;
          aProjectedDistances[0] = dDist;
          aParams1[0] = dParam;
          aParams2[0] = 0.0;
          return SM_SUCCESS;
        }  // end degenerate curve2 check 

      // arrive here when curve1 and curve2 are both lines - intersect them
      ULONG lNumInt;
      SmPoint3d aPoints[3];
      SER(smgu_SegmentSegmentIntersect(sStart1,sEnd1,
                                       sStart2,sEnd2,
                                       dDistanceTolerance,
                                       lNumInt,
                                       aPoints));

      // when no intersections - done
      if (lNumInt < 1) 
        {
          return SM_SUCCESS;
        }

      // found an intersection - gather output data
      double dParam1, dDist, dParam2;
      SER(smgu_SegmentPointDistance(sStart1,sEnd1,aPoints[0],dDist,dParam1));
      SER(smgu_SegmentPointDistance(sStart2,sEnd2,aPoints[0],dDist,dParam2));
      rlNumInt = 1;
      aProjectedDistances[0] = dDist;
      aParams1[0] = dParam1;
      aParams2[0] = dParam2;

      // for coincident segments
      if (lNumInt > 1) 
        {
          rlNumInt = 2;
          SER(smgu_SegmentPointDistance(sStart1,sEnd1,aPoints[1],dDist,dParam1));
          SER(smgu_SegmentPointDistance(sStart2,sEnd2,aPoints[1],dDist,dParam2));
          aProjectedDistances[1] = dDist;
          aParams1[1] = dParam1;
          aParams2[1] = dParam2;
        }                            
    } // end curve 1 is a line branch
    
  // all done
  return SM_SUCCESS;

} // end smgu_SegmentSegmentProjectedIntersect

/*******************************************************************//**
PURPOSE: Intersect two perspective projected segments.

NOTES: while there is both a smgu_SegmentSegmentProjectedIntersect and
                             smgu_SegmentSegmentPerspectiveIntersect, 
  there is no smgu_SegmentSegmentRotatedProjectIntersect yet
  because lines project to parabolas and I didn't take the time
  to implement that.
***********************************************************************/
SM_EXPORT SmStatus smgu_SegmentSegmentPerpspectiveIntersect
(
  const SmPoint3d  & crStart1,                   // in :
  const SmPoint3d  & crEnd1,                     // in :
  const SmPoint3d  & crStart2,                   // in :
  const SmPoint3d  & crEnd2,                     // in :
  const SmPoint3d  & crPlanePoint,               // in :
  const SmVector3d & crPlaneUnitNormal,          // in :
  const SmPoint3d  & crEyePoint,                 // in :
  double             dDistanceTolerance,         // in : Tolerance to use in intersection  
  ULONG            & rlNumInt,                   // out:  0 - no intersections, 
                                                 //       1 - crossing or ends touching intersection
                                                 //       2 - coincident segments.
  double             aParams1[2],                // out:
  double             aParams2[2],                // out:
  double             aProjectedDistances[2]      // out:
)
{
  double dDistanceTolerance2 = dDistanceTolerance*dDistanceTolerance;
  rlNumInt = 0;
  SmPoint3d sStart1, sEnd1, sStart2, sEnd2;

  // perpsective project line end points to a common plane - the one containing sStart1
  SER(smgu_PointPerspectiveToPlane(crStart1, crPlanePoint, crPlaneUnitNormal, crEyePoint, sStart1));
  SER(smgu_PointPerspectiveToPlane(crEnd1,   crPlanePoint, crPlaneUnitNormal, crEyePoint, sEnd1));
  SER(smgu_PointPerspectiveToPlane(crStart2, crPlanePoint, crPlaneUnitNormal, crEyePoint, sStart2));
  SER(smgu_PointPerspectiveToPlane(crEnd2,   crPlanePoint, crPlaneUnitNormal, crEyePoint, sEnd2));

  // Duplicated Code! 
  // the rest of this function is the same as smgu_SegmentSegmentProjectedIntersect code
  //   make any changes needed in both functions 

  // when curve one degenerates to a point
  if (sStart1.DistanceBetweenSquared(sEnd1) < dDistanceTolerance2) 
    { 
      // when curve 2 degenerates to a point
      if (sStart2.DistanceBetweenSquared(sEnd2) < dDistanceTolerance2) 
        {
          // Two possible cases - first they are disjoint - second they
          // fully intersect.
          if (sStart1.DistanceBetweenSquared(sStart2) > dDistanceTolerance2) 
            {
              return SM_SUCCESS;
            }
          rlNumInt = 2;
          aProjectedDistances[0] = aProjectedDistances[1] = sStart1.DistanceBetween(sStart2);
          aParams1[0] = aParams2[0] = 0.0;
          aParams1[1] = aParams2[1] = 1.0;
          return SM_SUCCESS;
        } // end degenerate curve 2 branch
      else // curve 2 is a line
        {
          // check point/line distance
          double dDist, dParam;
          SER(smgu_SegmentPointDistance(sStart2,sEnd2,sStart1,dDist,dParam));
          if (dDist > dDistanceTolerance) 
            {
              return SM_SUCCESS;
            }
          rlNumInt = 1;
          aProjectedDistances[0] = dDist;
          aParams1[0] = 0.0;
          aParams2[0] = dParam;
          return SM_SUCCESS;
        }  // end curve2 is a line branch
    } // end degenerate curve1 branch
  else // curve 1 is a line
    {
      // curve 2 is degenerate
      if (sStart2.DistanceBetweenSquared(sEnd2) < dDistanceTolerance2) 
        {
          // get point/curve distance
          double dDist, dParam;
          SER(smgu_SegmentPointDistance(sStart1,sEnd1,sStart2,dDist,dParam));
          if (dDist > dDistanceTolerance) 
            {
              return SM_SUCCESS;
            }
          rlNumInt = 1;
          aProjectedDistances[0] = dDist;
          aParams1[0] = dParam;
          aParams2[0] = 0.0;
          return SM_SUCCESS;
        }  // end degenerate curve2 check 

      // arrive here when curve1 and curve2 are both lines - intersect them
      ULONG lNumInt;
      SmPoint3d aPoints[3];
      SER(smgu_SegmentSegmentIntersect(sStart1,sEnd1,
                                       sStart2,sEnd2,
                                       dDistanceTolerance,
                                       lNumInt,
                                       aPoints));

      // when no intersections - done
      if (lNumInt < 1) 
        {
          return SM_SUCCESS;
        }

      // found an intersection - gather output data
      double dParam1, dDist, dParam2;
      SER(smgu_SegmentPointDistance(sStart1,sEnd1,aPoints[0],dDist,dParam1));
      SER(smgu_SegmentPointDistance(sStart2,sEnd2,aPoints[0],dDist,dParam2));
      rlNumInt = 1;
      aProjectedDistances[0] = dDist;
      aParams1[0] = dParam1;
      aParams2[0] = dParam2;

      // for coincident segments
      if (lNumInt > 1) 
        {
          rlNumInt = 2;
          SER(smgu_SegmentPointDistance(sStart1,sEnd1,aPoints[1],dDist,dParam1));
          SER(smgu_SegmentPointDistance(sStart2,sEnd2,aPoints[1],dDist,dParam2));
          aProjectedDistances[1] = dDist;
          aParams1[1] = dParam1;
          aParams2[1] = dParam2;
        }                            
    } // end curve 1 is a line branch
    
  // all done
  return SM_SUCCESS;

} // end smgu_SegmentSegmentPerpspectiveIntersect

/*******************************************************************//**
PURPOSE: find nearest distance to circle semgent from given point

NOTES:  assumes CircleX is perpendicular to CircleY and both are unitized

METHOD ---
  // p      = projection of Point onto circle plane
  // C(a)   = r cos(a) dx + r sin(a) dy
  // C(a)-p = (r cos(a) - rp cos(ap)) dx + (r sin(a) - rp sin(ap)) dy
  // dist**2 = R**2 + Rp**2 - 2*Rp*R*Cos(dPointAngle-dCircleAngle)

***********************************************************************/
SM_EXPORT SmStatus smgu_ArcSegmentPointDistance
(
  const SmPoint3d  & crCircleCenter,  // in : O of circle = o + R*cos(ang)*X + R*sin(ang)*Y
  const SmVector3d & crCircleX,       // in : X of circle = o + R*cos(ang)*X + R*sin(ang)*Y
  const SmVector3d & crCircleY,       // in : Y of circle = o + R*cos(ang)*X + R*sin(ang)*Y
  double             dRadius,         // in : R of circle = o + R*cos(ang)*X + R*sin(ang)*Y
  double             dStartAngleDeg,  // in : min limit on ang  [-360 to +360]
  double             dEndAngleDeg,    // in : max limit on ang  [-360 to +360]
  const SmPoint3d  & sPoint,          // in : target point
  SmBoolean          bMinFlag,        // in : TRUE = find nearest distance, 
                                      //      FALSE= find furthest distance.
  double           & dDistance,       // out: min distance from point to circle segment
  double           & dAngleDeg        // out: angle of closest point on circle segment
)       
{
  // check inputs
  SM_ASSERT(SM_IS_CONTAINED(dStartAngleDeg, -360-SM_EFF_ZERO, 360+SM_EFF_ZERO)) ;
  SM_ASSERT(SM_IS_CONTAINED(dEndAngleDeg,   -360-SM_EFF_ZERO, 360+SM_EFF_ZERO)) ;

  // project point to circle plane
  SmVector3d sVec(sPoint - crCircleCenter) ;
  double dX   = sVec.Dot(crCircleX) ;
  double dY   = sVec.Dot(crCircleY) ;
  double dZ   = sVec.Dot(crCircleX * crCircleY) ;
  double dRp  = smos_Sqrt(dX * dX + dY * dY) ;
  double dScaledZero = SM_EFF_ZERO * (1.0 + crCircleCenter.GetMaxDimension()) ;

  // watch for center points
  if(   smos_Fabs(dX) < dScaledZero
     && smos_Fabs(dY) < dScaledZero)
    {
      // all angles have the same distance
      dDistance = smos_Sqrt(dRadius*dRadius + dZ*dZ) ;
      dAngleDeg = (dStartAngleDeg + dEndAngleDeg) / 2.0 ;

      // all done
      return(SM_SUCCESS) ;
    }

  // check for unity
#ifdef SM_DEBUG_CODE
  double dXSize2 = crCircleX.LengthSquared() ;
  double dYSize2 = crCircleY.LengthSquared() ;
  SM_ASSERT(SM_IS_ZERO(dXSize2 -1.0)) ; 
  SM_ASSERT(SM_IS_ZERO(dYSize2 -1.0)) ;
#endif    

  // get angle to closest circle point [-180 to 180]
  dAngleDeg = smos_ArcTangent2(dY, dX) * 180.0 / SM_PI ;  

  // when asked switch to max distance point
  if(bMinFlag == FALSE)
    {
      dAngleDeg =   dAngleDeg > 180.0 
                  ? dAngleDeg - 180.0
                  : dAngleDeg + 180.0 ;
    } // end need to maximize distance check

  // watch for periodicity
  if     (SM_IS_CONTAINED(dAngleDeg+360.0, dStartAngleDeg, dEndAngleDeg)) { dAngleDeg += 360.0 ; }
  else if(SM_IS_CONTAINED(dAngleDeg-360.0, dStartAngleDeg, dEndAngleDeg)) { dAngleDeg -= 360.0 ; }

  // C(a)   = r cos(a) dx + r sin(a) dy
  // C(a)-p = (r cos(a) - rp cos(ap)) dx + (r sin(a) - rp sin(ap)) dy
  // dist**2 = R**2 + Rp**2 - 2*Rp*R*Cos(dPointAngle-dCircleAngle)
  // find cos(ClosestCircleAngle - PointAngle) watching for segment endPoints
  double dCosDeltaAngle = 1.0 ;

  if(!SM_IS_CONTAINED(dAngleDeg, dStartAngleDeg, dEndAngleDeg))
    {
      // find closest(furthest) endPoint with largest(smallest) cosine angle 
      double dCosDeltaStart = smos_Cosine((dStartAngleDeg - dAngleDeg) * SM_PI / 180.0) ;
      double dCosDeltaEnd   = smos_Cosine((dEndAngleDeg   - dAngleDeg) * SM_PI / 180.0) ;
      if(   ( bMinFlag && dCosDeltaStart > dCosDeltaEnd) 
         || (!bMinFlag && dCosDeltaStart < dCosDeltaEnd)) { dCosDeltaAngle = dCosDeltaStart ;
                                                            dAngleDeg      = dStartAngleDeg ;
                                                          }
      else                                                { dCosDeltaAngle = dCosDeltaEnd ;
                                                            dAngleDeg      = dEndAngleDeg ;
                                                          }
    }

  // GET distance
  dDistance = smos_Sqrt(dRadius*dRadius + dRp*dRp - 2*dRadius*dRp*dCosDeltaAngle + dZ*dZ) ;

#ifdef SM_DEBUG_CODE
  // check
  SmVector3d sGap =   crCircleCenter
                    + dRadius * smos_Cosine(dAngleDeg * SM_PI / 180.0) * crCircleX 
                    + dRadius * smos_Sine(dAngleDeg * SM_PI / 180.0) * crCircleY 
                    - sPoint ;

  double dDist = sGap.Length() ; 
  SM_ASSERT(SM_ARE_SAME(dDist, dDistance)) ;                            
#endif // SM_DEBUG_CODE

  // all done
  return(SM_SUCCESS) ;

} // end smgu_ArcSegmentPointDistance

/*******************************************************************//**
PURPOSE: Intersect two line segments.  If more than one intersection
    exists that means there is a coincident segment which is bounded
    by the two points. 

NOTES: In cases of near coincidence.  The points returned will
    be on the first segment.

   sm_AddPoint() used to SER, which propagates all the way up.
   Change to SE.  [B608]
***********************************************************************/
SM_EXPORT SmStatus smgu_SegmentSegmentIntersect
(
  const SmPoint3d & crLineStart,        // in : 1st segment startPoint
  const SmPoint3d & crLineEnd,          // in : 1st segment endPoint
  const SmPoint3d & crOLineStart,       // in : 2nd segment startPoint
  const SmPoint3d & crOLineEnd,         // in : 2nd segment endPoint  
  double            d3DTolerance,       // in : min distance between distinct points
  ULONG           & rlNumIntersections, // out: number of intersections
  SmPoint3d         aPoints[2]          // out: intersection points (max of 2)
)         
{
  double dParam, dDist;
  rlNumIntersections = 0; 

  // Check for unset tolerance.  [B608]
  d3DTolerance = smos_Max( d3DTolerance, SM_EFF_ZERO );

  // Save OLineStart point when its on thisLineSegment
  if (smgu_IsPointOnSegment(crLineStart,crLineEnd,crOLineStart,d3DTolerance,
                            dDist,dParam)) 
    {
      SmPoint3d sPnt = crLineStart + dParam * (crLineEnd - crLineStart);
      SE(sm_AddPoint(rlNumIntersections,aPoints,d3DTolerance,sPnt));
    }
  
  // Save OLineEnd point when its on thisLineSegment
  if (smgu_IsPointOnSegment(crLineStart,crLineEnd,crOLineEnd,d3DTolerance,
                            dDist,dParam)) 
    {
      SmPoint3d sPnt = crLineStart + dParam * (crLineEnd - crLineStart);
      SE(sm_AddPoint(rlNumIntersections,aPoints,d3DTolerance,sPnt));
    }

  // Save ThisStart point when its on OtherLineSegment
  if (smgu_IsPointOnSegment(crOLineStart,crOLineEnd,crLineStart,d3DTolerance,
                            dDist,dParam)) 
    {
      SE(sm_AddPoint(rlNumIntersections,aPoints,d3DTolerance,crLineStart));
    }

  // Save ThisEnd point when its on OtherLineSegment
  if (smgu_IsPointOnSegment(crOLineStart,crOLineEnd,crLineEnd,d3DTolerance,
                            dDist,dParam)) 
    {
      SE(sm_AddPoint(rlNumIntersections,aPoints,d3DTolerance,crLineEnd));
    }
  
  // all done when 2 points are found
  if (rlNumIntersections > 1) 
    {
      return SM_SUCCESS;
    }

  // Look for interior intersection.
  SmVector3d sLineVec ( crLineEnd  - crLineStart  );
  SmVector3d sOLineVec( crOLineEnd - crOLineStart );
//   if (sLineVec.LengthSquared() < SM_EFF_ZERO_SQ) SER(SM_ERR);  // RCLxx
//   if (sOLineVec.LengthSquared() < SM_EFF_ZERO_SQ) SER(SM_ERR); // RCLxx

  // get line/line nearestPoint lineParameters 
  double dParam1, dParam2;
  if (smgu_LineLineClosestPoint(crLineStart,sLineVec,crOLineStart,sOLineVec,dParam1,dParam2) != SM_SUCCESS) 
    {
      // No real problem just parallel lines
      return SM_SUCCESS;
    }
  
  // snap solution1 to segment boundaries when needed
  SmBoolean bTrunc1 = FALSE;
  if (dParam1 < 0.0) { bTrunc1 = TRUE;
                       dParam1 = 0.0;
                     }
  if (dParam1 > 1.0) { bTrunc1 = TRUE;
                       dParam1 = 1.0;
                     }
  SmPoint3d sPnt = crLineStart + dParam1 * sLineVec;

  // find closest point on Other line to snapped boundaries
  if (bTrunc1) 
    {
      SER(smgu_LineClosestPoint(crOLineStart,sOLineVec,sPnt,dParam2));
    }

  // snap solution1 to segment boundaries when needed
  SmBoolean bTrunc2 = FALSE;
  if (dParam2 < 0.0) { bTrunc2 = TRUE;
                       dParam2 = 0.0;
                     }
  if (dParam2 > 1.0) { bTrunc2 = TRUE;
                       dParam2 = 1.0;
                     }
  SmPoint3d sPnt2 = crOLineStart + dParam2 * sOLineVec;
  
  // find closest point on this line to snapped boundaries
  if (bTrunc2) 
    {
      SER(smgu_LineClosestPoint(crLineStart,sLineVec,sPnt2,dParam1));
      if (dParam1 < 0.0) dParam1 = 0.0;
      if (dParam1 > 1.0) dParam1 = 1.0;
      sPnt = crLineStart + dParam1 * sLineVec;
    }

  // cull nearest point distances greater than tolerance
  dDist = sPnt.DistanceBetween(sPnt2);
  if (dDist > d3DTolerance) 
    { return SM_SUCCESS;
    }
  
  // with just 1 intersection
  if (rlNumIntersections == 1) 
    {
      // Looks like we have a small coincident segment or something.
      double dDistEndPt = sPnt.DistanceBetween( aPoints[0] );
      if ( dDistEndPt > d3DTolerance*2.0 )
        {
          // If the segments are nearly parallel, we can have an interior
          // intersection, but an end point can still be within tolerance.
          // Save both points only if they're both close. [B380]
          double dDistIntPt = sPnt.DistanceBetween( sPnt2 );
          if ( dDistEndPt < dDistIntPt * 100 )
            {
              aPoints[1] = sPnt;
              rlNumIntersections = 2;
              return SM_SUCCESS;
            }
        }

      // Use the analytic intersection instead of the end-point int.  
      // [bd 060808]
      aPoints[0] = sPnt;
      return SM_SUCCESS;
    }

  aPoints[0] = sPnt;
  rlNumIntersections = 1;

  return SM_SUCCESS;

} // end smgu_SegmentSegmentIntersect

/*******************************************************************//**
PURPOSE: Find the closest point(s) between two line segment.  If the
    segments are parallel there may be 2 answers.

NOTES: 
***********************************************************************/
SM_EXPORT SmStatus smgu_SegmentSegmentClosestPoint
(
  const SmPoint3d & crLineStart,          // in :
  const SmPoint3d & crLineEnd,            // in :
  const SmPoint3d & crOLineStart,         // in :
  const SmPoint3d & crOLineEnd,           // in :
  double            d3DTolerance,         // in :
  ULONG           & rlNumClosestPoints,   // out: If 1 then there is 1 closest point,
                                          //      If 2 then segments are parallel and overlapping
  double            adParamSeg1[2],       // out: Normalized parameter (between 0.0 and 1.0) for segment 1
  double            adParamSeg2[2],       // out: Normalized paraemter for segment 2 of closest point(s)
  double            adDistances[2]        // out:
)
{
    double dParam, dDist;

    // First check for degenerate lines:
    // just return the 'beginning' of the degenerate line.
    // [bd 060319]
    SmVector3d sLineVec = crLineEnd - crLineStart;
    if ( sLineVec.LengthSquared() < SM_EFF_ZERO_SQ )
    {
        rlNumClosestPoints = 1;
        adParamSeg1[0] = 0.0;
        SER( smgu_SegmentPointDistance( crOLineStart, crOLineEnd, crLineStart,
                dDist, adParamSeg2[0] ));

        return SM_SUCCESS;
    }

    SmVector3d sOLineVec = crOLineEnd - crOLineStart;
    if ( sOLineVec.LengthSquared() < SM_EFF_ZERO_SQ )
    {
        rlNumClosestPoints = 1;
        adParamSeg2[0] = 0.0;
        SER( smgu_SegmentPointDistance( crLineStart, crLineEnd, crOLineStart,
                dDist, adParamSeg1[0] ));

        return SM_SUCCESS;
    }

    // Now both lines have length.

    rlNumClosestPoints = 0;
    SER(smgu_SegmentPointDistance(crLineStart,crLineEnd,crOLineStart,dDist,dParam));
    SER(sm_AddClosestPoint(dDist,dParam,0.0,d3DTolerance,rlNumClosestPoints,adParamSeg1,adParamSeg2,adDistances));

    SER(smgu_SegmentPointDistance(crLineStart,crLineEnd,crOLineEnd,dDist,dParam));
    SER(sm_AddClosestPoint(dDist,dParam,1.0,d3DTolerance,rlNumClosestPoints,adParamSeg1,adParamSeg2,adDistances));

    SER(smgu_SegmentPointDistance(crOLineStart,crOLineEnd,crLineStart,dDist,dParam));
    SER(sm_AddClosestPoint(dDist,0.0,dParam,d3DTolerance,rlNumClosestPoints,adParamSeg1,adParamSeg2,adDistances));

    SER(smgu_SegmentPointDistance(crOLineStart,crOLineEnd,crLineEnd,dDist,dParam));
    SER(sm_AddClosestPoint(dDist,1.0,dParam,d3DTolerance,rlNumClosestPoints,adParamSeg1,adParamSeg2,adDistances));

    // Look for interior intersection.

    double dParam1, dParam2;
    if (smgu_LineLineClosestPoint(crLineStart,sLineVec,crOLineStart,sOLineVec,dParam1,dParam2) != SM_SUCCESS) {
        // No real problem just parallel lines
        return SM_SUCCESS;
    }
    if (dParam1 < 0.0) dParam1 = 0.0;
    if (dParam1 > 1.0) dParam1 = 1.0;
    if (dParam2 < 0.0) dParam2 = 0.0;
    if (dParam2 > 1.0) dParam2 = 1.0;
    SmPoint3d sPnt = crLineStart + dParam1 * sLineVec;
    SmPoint3d sPnt2 = crOLineStart + dParam2 * sOLineVec;

    dDist = sPnt.DistanceBetween(sPnt2);

    SER(sm_AddClosestPoint(dDist,dParam1,dParam2,d3DTolerance,rlNumClosestPoints,adParamSeg1,adParamSeg2,adDistances));

    return SM_SUCCESS;

} // end smgu_SegmentSegmentClosestPoint

/*******************************************************************//**
PURPOSE: Find nearest point on segment to targetPoint.

RETURNS --- Return AngleInDegrees from perpendicular between the
            TargetPoint to NearestSegmentPoint gap vector
            and the segment Tangent. 

            So, if the nearestSegmentPoint is inside the segment, the
            returned angle will be 0.0.  If the NearestSegmentPoint
            is a SegmentEndPoint then the return angle can be 
            anything from 0 to 90.

            TestPoint-Segment Gap/SegmentTangent Angle-90 [degrees]

NOTES: Note that if the point is on the line then the
    angle returned is 0 degrees.

***********************************************************************/
SM_EXPORT double smgu_SegmentPointAngleDegToPerp 
(
  const SmPoint3d & crLineStart,                  // in : Segment start point
  const SmPoint3d & crLineEnd,                    // in : segement end point
  const SmPoint3d & crTestPoint,                  // in : target point
  double & rNormalizedSegmentParam                // out: segment parameter closest to target point
                                                  //      runs from 0.0 to 1.0.
)
{
  SmVector3d sLineVec = crLineEnd - crLineStart;
  SER(smgu_LineClosestPoint(crLineStart,sLineVec,crTestPoint,rNormalizedSegmentParam));
  SmExtent1d sClipExt(0.0,1.0);
  // If both are outside of the clip range and on the same side
  // then make some additional tests.
  if (sClipExt.ContainsValue(rNormalizedSegmentParam)) { return 0.0; }
  
  rNormalizedSegmentParam = sClipExt.ClampValue(rNormalizedSegmentParam);
  SmPoint3d  sSegClPnt    = crLineStart + sLineVec * rNormalizedSegmentParam;
  SmVector3d sVecToPnt    = crTestPoint - sSegClPnt;

  // when targetPoint is on Segment - return angle = 0.0
  if (sVecToPnt.LengthSquared() < SM_EFF_ZERO_SQ) 
     { return 0.0; }
  
  // get Gap/Segment angle (0 to Pi)
  double dAng = 0;
  if (sVecToPnt.AngleBetween(sLineVec,dAng) != SM_SUCCESS) 
    {  return 0.0;  } // point is on the line 
  
  // get Angle from perpendicular in degrees
  double dAngDeg = smos_Fabs(dAng*180.0/SM_PI - 90.0) ;

  // only endPoints should have nonZero return angles
  SM_ASSERT(   SM_IS_ZERO(dAngDeg)                             // assert - angle is 90 degrees
            || rNormalizedSegmentParam <= SM_EFF_ZERO          //          or nearest point is 
            || rNormalizedSegmentParam >= 1.0 - SM_EFF_ZERO) ; //             a segment endPoint

  // all done
  return dAngDeg;

} // end smgu_SegmentPointAngleDegToPerp

/*******************************************************************//**
PURPOSE: Determine the side of a 2D segment on which a 2D point lies.

NOTES: 
***********************************************************************/
SM_EXPORT SmStatus smgu_SegmentPoint2DSide
(
  const SmPoint3d & crStartPoint,   // in :
  const SmPoint3d & crEndPoint,     // in :
  const SmPoint3d & crTestPoint,    // in :
  ULONG           & rlSide          // out: 0 - Point lies on line of segment
                                    //      1 - Point lies on left side of segment
                                    //      2 - Point lies on right side of segment
)
{
    SmVector3d sLineVec = crEndPoint - crStartPoint;
    double dParam;
    SER(smgu_LineClosestPoint(crStartPoint,sLineVec,crTestPoint,dParam));
    SmVector3d sClosestVec = crTestPoint - (crStartPoint + dParam * sLineVec);
    if (sClosestVec.LengthSquared() < SM_EFF_ZERO) {
        rlSide = 0;
        return SM_SUCCESS;
    }

    SmVector3d sZVec(0,0,1);
    double dAngleBetween;
    SER(sZVec.CCWAngleBetween(sLineVec,sClosestVec,dAngleBetween));
    if (dAngleBetween < 0.0) {
        rlSide = 2;
    }
    else {
        rlSide = 1;
    }
    return SM_SUCCESS;

} // end smgu_SegmentPoint2DSide

/*******************************************************************//**
PURPOSE: Determine if two points and tangent vectors are part of a
    circle and extract the information about that circle if they are.

NOTES: Circle is defined counter clockwise relative to P1 with
    Tan1 going clockwise.
***********************************************************************/
SM_EXPORT SmStatus smgu_CircleFromPointsTangents
(
  const SmPoint3d  & sP1,               // in :
  const SmVector3d & sTan1,             // in :
  const SmPoint3d  & sP2,               // in :
  const SmVector3d & sTan2,             // in :
  double             dAngleTolDeg,      // in :
  SmBoolean        & rbIsCircle,        // out:
  SmAxis2Placement & rPlacement,        // out:
  double           & rdRadius,          // out:
  double           & rdStartAngleDeg,   // out:
  double           & rdEndAngleDeg      // out:
)
{
    rbIsCircle = FALSE;

    SmVector3d sNorm = sTan1 * sTan2;
    if (sNorm.LengthSquared() < SM_EFF_ZERO_SQ) return SM_SUCCESS;
    SmVector3d sVecP1P2 = sP2 - sP1;
    SmVector3d sLineVec = sNorm * sTan1;
    if (sLineVec.Dot(sVecP1P2) < 0.0) {
        sLineVec = - sLineVec;
    }
    SER(sLineVec.Unitize());
    SmVector3d sPDiff = sP1 - sP2;

    // Solve equation that says difference between circle center and P2
    // is same as T which is the radius.  The line goes from P1 back through
    // the origin.
    // t^2 = ((P1-T*LV) - P2)^2
    // Set up quadratic equation coefficients.
    double coeff[3];
    coeff[2] = sLineVec.Dot(sLineVec) - 1;   // A of quadratic equation
    coeff[1] = 2.0 * sPDiff.Dot(sLineVec); // B
    coeff[0] = sPDiff.Dot(sPDiff);       // C 
    
    // Solve for t using quadratic equation.
    ULONG nSol;
    double dSol[2];
    SmPolynomial::SolveQuadraticEqn(coeff,SM_EFF_ZERO,nSol,dSol);
    double dT = 0.0;
    if (nSol == 0) return SM_SUCCESS;

    if (nSol == 1) {
        dT = dSol[0];
    }
    if (nSol == 2) {
        dT = smos_Min(dSol[0],dSol[1]);
    }

    if (smos_Fabs(dT) < SM_EFF_ZERO) return SM_SUCCESS;

    SmPoint3d sCenter = sP1 + dT * sLineVec;
    
    SmVector3d sCToP2 = sP2 - sCenter;
    SER(sCToP2.Unitize());

    double dAngRad;
    SER(sCToP2.AngleBetween(sTan2,dAngRad));
    double dAngDeg = dAngRad * 180.0 / SM_PI;
    if (smos_Fabs(dAngDeg - 90.0) > dAngleTolDeg) {
        return SM_SUCCESS; // Doesn't satisfy angle tolerance
    }

    // We have made it - it is a circle - now compute 
    // some vectors and angles that define it.
    SmVector3d sX = - sLineVec;
    sX.Unitize();

    SmVector3d sY = sNorm * sX;
    if (sY.Dot(sTan1) < 0.0) {
        sY = - sY;
        sNorm = - sNorm;
    }
    sY.Unitize();

    rPlacement.SetCanonical(sCenter,sX,sY);
    double dAngleRad;
    SER(sNorm.CCWAngleBetween(sX,sCToP2,dAngleRad));
    rdStartAngleDeg = 0.0;
    rdEndAngleDeg = dAngleRad * 180.0 / SM_PI;
    rdRadius = dT;
    rbIsCircle = TRUE;
    return SM_SUCCESS;

} // end smgu_CircleFromPointsTangents

/*******************************************************************//**
PURPOSE: Find the circle defined by a point, the tangent there, and
    one other point on the circle.

NOTES:
   The output is in two forms: Center point and radius, as well as
   the Bezier form.  The Bezier form is the standard three-point
   rational, whose points are (crP0, rsBezierPoint, crP1); the weights
   of crP0 and crP1 are 1.0, and that of rsBezierPoint is rdBezierWeight.

   This is written for circles that span less than 180 degrees.  If the input
   defines a circle of greater than 180, then the Bezier weight will be
   negative, which defines the circle complementary to the control polygon.
   That is a perfectly well-behaved Bezier circle, but NLib does not allow
   negative weights.  (Maybe it should.)  Similarly, if the inputs define
   a circle of 180 degrees, the weight will be 'zero' (approximated by
   SM_EFF_ZERO), and the middle Bezier point will be the point at 'infinity'
   (approximated by 1/SM_EFF_ZERO).  Again, this would define a well-behaved
   Bezier circle, but NLib will not allow it.

   At any rate, checking the weight will tell you whether or not you can
   create an NLib curve from the Bezier form: the smallest allowable
   weight is NL_WMIN = 0.001.
***********************************************************************/
SM_EXPORT SmStatus smgu_CircleFromPointsOneTangent
(
  const SmPoint3d  & crP0,            // in:
  const SmVector3d & crT0,            // in: Unit tangent at crP0.
  const SmPoint3d  & crP1,            // in:
  SmPoint3d        & rsCenter,        // out:
  double           & rdRadius,        // out:
  SmPoint3d        & rsBezierPoint,   // out: Middle Bezier point
  double           & rdBezierWeight   // out: Weight for rsBezierPoint
) 
{
  SmVector3d sVec( crP0 - crP1 );
  double dV_V = sVec.Dot( sVec );
  double dV_T = sVec.Dot( crT0 );

  double dL; // This will be the length of each leg of the Bezier control polygon.

  if ( smos_Fabs( dV_T ) < SM_EFF_ZERO )
  {
      // T0 is perp to V: semicircle
      rsCenter = ( crP0 + crP1 ) / 2;    // midpoint
      rdRadius = smos_Sqrt( dV_V ) / 2;  // half the distance between P0 and P1

      // Bezier pt will be a point at "infinity"
      rdBezierWeight = SM_EFF_ZERO;
      rsBezierPoint  = rsCenter + crT0 / SM_EFF_ZERO;

      return 0;
  }

  // Bezier weight:
  // The weight is the cosine of theta, where theta is the half-angle
  // between the two circle radii (from P0 to P1).
  // cos( theta ) = sin( alpha ),  where alpha is the half-angle between
  // where the two circle tangents intersect.  The sine of that angle
  // is dD / dL, where dD is half the distance between the points.
  // So the weight is just dD / dL.

  dL = - dV_V / ( 2 * dV_T ); //cbi explain this.
  double dHalfDist = smos_Sqrt( dV_V ) / 2;  // half the distance between P0 and P1
  rdBezierWeight = dHalfDist / dL;

  // Radius:
  double dSinHalfTheta = smos_Sqrt( 1.0 - rdBezierWeight * rdBezierWeight );  // sin^2 = 1 - cos^2.
  rdRadius = dHalfDist / dSinHalfTheta;

  // Center:
  SmVector3d sNorm( crT0 * sVec );
  double dTemp = sNorm.Length();
  if ( dTemp < SM_EFF_ZERO )
    { return -1; } //cbi figure it out.
  sNorm /= dTemp;

  SmVector3d sP0_to_C( crT0 * sNorm );
  dTemp = sP0_to_C.Length();
  if ( dTemp < SM_EFF_ZERO )
    { return -1; } //cbi figure it out.
  sP0_to_C /= dTemp;

  rsCenter = crP0 + rdRadius * sP0_to_C;

  // Bezier point:
  rsBezierPoint = crP0 + dL * crT0;

  return 0;

} // end smgu_CircleFromPointsOneTangent

/*******************************************************************//**
PURPOSE: Compute the center of a 2D circle given 3 non linear points.

NOTES: 
***********************************************************************/
SM_EXPORT SmStatus smgu_CircleCenterFrom3Points
(
  const SmPoint2d & crPoint1,            // in:
  const SmPoint2d & crPoint2,            // in:
  const SmPoint2d & crPoint3,            // in:
  SmPoint2d       & rCenterOfCircle      // out:
)
{
  // Give the points new names to make the equations simpler
  const SmPoint2d & A = crPoint1;
  const SmPoint2d & B = crPoint2;
  const SmPoint2d & C = crPoint3;

  double dTol = SM_EFF_ZERO_SQRT * (1.0 + A.GetMaxDimension() + C.GetMaxDimension());
  double dTol2 = dTol * dTol;
  // Do some checks for coincident points and colinear points
  if ( A.DistanceBetweenSquared( B ) < dTol2 ) { return FALSE; }
  if ( B.DistanceBetweenSquared( C ) < dTol2 ) { return FALSE; }
  if ( A.DistanceBetweenSquared( C ) < dTol2 ) { return FALSE; }
  SmVector3d sVec( C - A );

  double dDistToLine;
  SER(smgu_LinePointDistance(A,sVec,B,dDistToLine));
  // Use a more meaningful tol.  [B133]
  //  if ( dDistToLine < dTol )
  if ( dDistToLine < 2.0 * SM_EFF_ZERO * A.DistanceBetweenSquared( C ) )
    { return SM_ERR; }  // Radius would be bigger than 1 / SM_EFF_ZERO.

  SmPoint2d & O = rCenterOfCircle;

  // Change to a simpler algorithm:
  // saying ||A-O|| = ||B-O|| gives a linear equation in Ox and Oy:
  //   2 ( Ax-Bx ) * Ox  +  2 ( Ay - By ) * Oy  =  Ax^2 + Ay^2 - Bx^2 - By^2
  // Then the same thing for ||B-O|| = ||C-O|| gives a 2nd linear equation
  //   2 ( Bx-Cx ) * Ox  +  2 ( By - Cy ) * Oy  =  Bx^2 + By^2 - Cx^2 - Cy^2
  // Then just solve the 2x2.
  //
  // // Set up the equation that solves for Ox in terms of Oy using
  // // ||A-O|| = ||B-O|| 
  // // Ox = Oy * M + N  where
  // // M = (Ay-By)/(Bx-Ax)  and N = (Bx^2+By^2-Ax^2-Ay^2) / (2.0 * (Bx-Ax))
  //
  // double dDenom = (B.x-A.x);
  // if (smos_Fabs(dDenom) < dTol) {
  //     // Use simplified equation because they have same X value
  //     dDenom = 1.0;
  // }
  // double M = (A.y-B.y)/dDenom;
  // double N = (B.x*B.x+B.y*B.y-A.x*A.x-A.y*A.y) / (2.0 * dDenom);
  //
  // if (smos_Fabs(M) < dTol && smos_Fabs(N) < dTol) {
  //     return FALSE;
  // }
  //
  // // Solve for O.y = (Bx^2 - 2BxN + By^2 - Cx^2 + 2CxN- Cy^2) / (2.0( BxM + By - CxM - Cy ))
  // double dDenom2 = 2.0 * ( B.x*M + B.y - C.x*M - C.y );
  //
  //  if (smos_Fabs(dDenom) < dTol) {
  //      dDenom2 = 0.0;
  //  }
  //
  // double dNumer2 = B.x*B.x - 2.0*B.x*N + B.y*B.y - C.x*C.x + 2.0*C.x*N - C.y*C.y;
  // if ( smos_Fabs( dDenom2 ) < smos_Fabs( dNumer2 ) * SM_EFF_ZERO )
  //   { return FALSE; }  // O.y would be > 1 / SM_EFF_ZERO.
  //
  // O.y = dNumer2 / dDenom2;
  // O.x = O.y*M + N;

  // Set up the 2x2 as described.
  double a00 = 2.0 * ( A.x - B.x );
  double a01 = 2.0 * ( A.y - B.y );
  double a10 = 2.0 * ( B.x - C.x );
  double a11 = 2.0 * ( B.y - C.y );

  double b0 = A.Dot(A) - B.Dot(B);
  double b1 = B.Dot(B) - C.Dot(C);

  double dDeterm = a00 * a11 - a10 * a01;  // Cramer's rule
  double dNumer0 = b0  * a11 - b1  * a01;
  double dNumer1 = a00 * b1  - a10 * b0 ;

  if ( smos_Fabs( dDeterm ) < smos_Fabs( dNumer0 ) * SM_EFF_ZERO )
    { return SM_ERR; }
  if ( smos_Fabs( dDeterm ) < smos_Fabs( dNumer1 ) * SM_EFF_ZERO )
    { return SM_ERR; }

  O.x = dNumer0 / dDeterm;
  O.y = dNumer1 / dDeterm;


#ifdef SM_DEBUG_CODE
  double dRadA = O.DistanceBetween( A );
  double dRadB = O.DistanceBetween( B );
  SM_ASSERT( smos_Fabs(dRadB-dRadA) < dRadA * SM_EFF_ZERO );
  double dRadC = O.DistanceBetween( C );
  SM_ASSERT( smos_Fabs(dRadC-dRadA) < dRadA * SM_EFF_ZERO );
#endif // SM_DEBUG_CODE

    return SM_SUCCESS;

} // end smgu_CircleCenterFrom3Points

/*******************************************************************//**
PURPOSE: Compute and return the descriptive parameters of a circular
            arc from 3 points, the start, a midPoint, and an endPoint
            from its circumference 

RETURNS --- SM_ERR when the 3 points are colinear
                   or when lDimesion is not 2 or 3
METHOD --- 
  1. Find Center Point, CP, as intersection of 3 planes
        Given SP, MP, EP, 3 points on a circular arc
        P0 = (SP + MP) / 2 ;  N0 = MP - SP ;
        P1 = (MP + EP) / 2 ;  N1 = EP - MP ;
        P2 = (EP + SP) / 2 ;  N2 = N0 x N1 ;

        0 = (CP - P0) . N0    [ N0.x N0.y N0.z ] [ x ]   [ P0 . N0 ]
        0 = (CP - P1) . N1 => [ N1.x N1.y N1.z ] [ y ] = [ P1 . N1 ]
        0 = (CP - P2) . N2    [ N2.x N2.y N2.z ] [ z ]   [ P2 . N2 ]
  2. XAxis = ||SP - CP||
  3. YAxis = N0 * N1
  4. rAnalDomain = [0, CCW angle from SP-CP to EP-CP about N2 in degrees]
  5. radius = |SP-CP|
***********************************************************************/
SmStatus smgu_CircleFrom3Points
(
  const SmPoint3d & cStartPoint,       // in : Circular Arc Start Point 
  const SmPoint3d & cMidPoint,         // in : Circular Arc Mid Point
  const SmPoint3d & cEndPoint,         // in : Circular Arc End Point
  SmPoint3d       & rCenter,           // out: Center Point
  SmVector3d      & rXAxis,            // out: XAxis pointing from CenterPoint to StartPoint
  SmVector3d      & rYAxis,            // out: yAxis orthogonal in CCW direction to XAxis
  SmExtent1d      & rAnalDomain,       // out: Arc Domain in degrees from 0 to max of 360
  double          & dRadius,           // out: Distance from CenterPoint to Circumference
  ULONG             lDimension,        // in : dimensions of points and vectors [2 or 3]
  SmBoolean         bClosedCircle      // in : TRUE = Return rAnalDomain for closed Circle
                                       //      FALSE= Return rAnalDomain for CircularArc from StartPoint to EndPoint
)
{
  // check input
  SM_ASSERT(lDimension == 2 || lDimension == 3) ;
  if(lDimension < 2 || lDimension > 3)
    { return SM_ERR ; }

  // Locals
  SmPoint3d sSP = cStartPoint ;
  SmPoint3d sMP = cMidPoint ;
  SmPoint3d sEP = cEndPoint ;

  // When working in 2D set all Z values = 0.0
  if(lDimension == 2) { sSP.z = 0.0 ;
                        sMP.z = 0.0 ;
                        sEP.z = 0.0 ;
                      }

  // find the center point as the intersection of 3 planes

  // Plane data
  SmVector3d sP0 = (sSP + sMP) / 2 ; SmVector3d sN0 = sMP - sSP ;
  SmVector3d sP1 = (sMP + sEP) / 2 ; SmVector3d sN1 = sEP - sMP ;
  SmVector3d sP2 = (sEP + sSP) / 2 ; SmVector3d sN2 = sN0 * sN1 ;

  // make intersection equation matrix
  // [ N0.x N0.y N0.z ] [ x ]   [ P0 . N0 ]
  // [ N1.x N1.y N1.z ] [ y ] = [ P1 . N1 ]
  // [ N2.x N2.y N2.z ] [ z ]   [ P2 . N2 ]
  SmMatrix sA(3,3) ;
  SmTArray<double> sB(3,NULL,3), sX(3,NULL,3) ;
  sA[0][0] = sN0.x ; sA[0][1] = sN0.y ; sA[0][2] = sN0.z ; sB[0] = sP0.Dot(sN0) ;
  sA[1][0] = sN1.x ; sA[1][1] = sN1.y ; sA[1][2] = sN1.z ; sB[1] = sP1.Dot(sN1) ;
  sA[2][0] = sN2.x ; sA[2][1] = sN2.y ; sA[2][2] = sN2.z ; sB[2] = sP2.Dot(sN2) ;
  
  // solve for Center Point
  SER_MSG(sA.SolveLinearSystem(sB, sX), _T("smgu_CircleFrom3Points() Failed due to Colinear Point inputs")) ;

  sN2.Unitize();

  // set outputs (except rAnalDomain)
  rCenter.Set(sX[0], sX[1], sX[2]) ;
  rXAxis  = sSP - rCenter ; 
  rYAxis  = sN2 * rXAxis ;
  dRadius = rXAxis.Length() ;
  rXAxis /= dRadius ;
  rYAxis.Unitize() ;

  // get CCW angle from XAxis to (PE-CP) about sN2
  double dCCWAngle ;
  if(bClosedCircle) { dCCWAngle = 360.0 ; }
  else              { sN2.CCWAngleBetween(rXAxis, sEP - rCenter, dCCWAngle) ;
                      dCCWAngle *= 180.0/SM_PI ;
                    }
  if(dCCWAngle < 0.0) { dCCWAngle += 360.0 ; }

  // set rAnalDomain
  rAnalDomain.SetMinMax(0.0, dCCWAngle) ;

  // check output
#ifdef SM_DEBUG_CODE
  double dMax1       = smos_Fabs(sSP.GetMaxDimension()) ;
  double dMax2       = smos_Fabs(sMP.GetMaxDimension()) ;
  double dMax3       = smos_Fabs(sEP.GetMaxDimension()) ;
  double dScaledZero = SM_EFF_ZERO * (1.0 + smos_3Max(dMax1,
                                                      dMax2,
                                                      dMax3)) ;
  double dL1         = (sSP - rCenter).Length() ;
  double dL2         = (sMP - rCenter).Length() ;
  double dL3         = (sEP - rCenter).Length() ;
  SmVector3d sSPTest = rCenter + dRadius * rXAxis ;
  SmVector3d sEPTest = rCenter 
                      + smos_Cosine(rAnalDomain.GetMax()*SM_PI/180.0) * dRadius * rXAxis
                      + smos_Sine  (rAnalDomain.GetMax()*SM_PI/180.0) * dRadius * rYAxis ;

  SM_ASSERT_MSG(   SM_ARE_SAME(dL1, dRadius)
                && SM_ARE_SAME(dL2, dRadius)
                && SM_ARE_SAME(dL3, dRadius)
                && sSPTest.CloserThan(dScaledZero, sSP)
                && (   (bClosedCircle == TRUE  && sEPTest.CloserThan(dScaledZero, sSP))
                    || (bClosedCircle == FALSE && sEPTest.CloserThan(dScaledZero, sEP))),
                _T("smgu_CircleFrom3Points() Computed out of tolerance circle")); 

#endif // SM_DEBUG_CODE

  // all done
  return SM_SUCCESS ;

} // end smgu_CircleFrom3Points

/*******************************************************************//**
PURPOSE: Intersect a circle and an infinite plane.  

NOTES: Works best with planes that are not nearly parallel to
    the circle plane.

  If two solutions are returned, they may be arbitrarily close together;
  d3DTolerance is not considered in that case.
***********************************************************************/
SM_EXPORT SmStatus smgu_CirclePlaneIntersect
(
  double             d3DTolerance,       // in : 
  double             dRadius,            // in : circle radius
  const SmPoint3d  & crCircleOrigin,     // in : circle center point
  const SmVector3d & crCircleNormal,     // in : Circle Plane's Normal vector [unitized]
  const SmPoint3d  & crPlanePoint,       // in : plane point
  const SmVector3d & crPlaneNormal,      // in : Plane Normal Vector [Unitized]
  ULONG            & rlNumTsect,         // out: 0 - no intersections
                                         //      1 - grazing intersection
                                         //      2 - standard two point intersection
                                         //      3 - indicates plane of circle is coincident with the intersection
                                         //          plane.  No points returned in aTsectPoints
  SmPoint3d          aTsectPoints[2]     // out: 
)
{
  SmPoint3d sLinePt;
  SmVector3d sLineVec;
  rlNumTsect = 0;

  // Check to make sure planes not parallel
  SmVector3d sCross = crCircleNormal * crPlaneNormal;
  if (sCross.LengthSquared() < SM_EFF_ZERO) 
    {
      SmPoint3d sOnPt;
      SER(smgu_PointProjectToPlane(crCircleOrigin,crPlanePoint,crPlaneNormal,sOnPt));
      if (sOnPt.DistanceBetween(crCircleOrigin) < d3DTolerance) 
        {
          rlNumTsect = 3; // Circle on plane
          return SM_SUCCESS;
        }
      else 
        { // Circle on plane parallel to plane but > d
          rlNumTsect = 0;
          return SM_SUCCESS;
        }
    }

  // find Plane/CirclePlane intersection Line
  SER(smgu_IntersectTwoPlanes(crCircleOrigin,crCircleNormal,crPlanePoint,crPlaneNormal,
      sLinePt, sLineVec));

  // find xSectLine/Circle intersection points
  ULONG lNumTsect;
  double adParameters[2];
  SER(smgu_LineCoPlanarCircleIntersect(sLinePt,sLineVec,crCircleOrigin,crCircleNormal,
                                       dRadius, d3DTolerance,
                                       lNumTsect,adParameters));

  // convert line param output to xSectPoint output
  for (ULONG i=0; i<lNumTsect; i++) 
    {
      rlNumTsect ++;
      aTsectPoints[i] = sLinePt + adParameters[i]*sLineVec;
    }

  // all done
  return SM_SUCCESS;

} // end smgu_CirclePlaneIntersect

/*******************************************************************//**
PURPOSE: Find two silhouette points of a circle.

NOTES: 
***********************************************************************/
SM_EXPORT SmStatus smgu_CircleFindSilhouettePoints
(
  const SmPoint3d  & crCircleCenter,        // in :
  const SmVector3d & crCircleNormal,        // in :
  double             dCircleRadius,         // in :
  SmBoolean          bPerspective,          // in :
  const SmVector3d & crEyePointOrVector,    // in :
  SmBoolean        & rbFoundPoints,         // out:
  SmPoint3d        & rSilPoint1,            // out:
  SmPoint3d        & rSilPoint2             // out:
)
{
    rbFoundPoints = FALSE;
    if (bPerspective) {
        SmPoint3d sProjEyePoint;
        SER(smgu_PointProjectToPlane(crEyePointOrVector,crCircleCenter,
            crCircleNormal,sProjEyePoint));
        SmVector3d sToEye = crCircleCenter - sProjEyePoint;
        double dLength = sToEye.Length();
        // If the point is inside of the radius than no silhouette
        // is possible.
        if (dLength < dCircleRadius + SM_EFF_ZERO) {
            return SM_SUCCESS; // No answer
        }
        sToEye.Unitize();

        double dAngleRad = smos_ArcCosine( dCircleRadius / dLength );
        SmAxis2Placement sA2P;
        SmPoint3d sToP1, sToP2;
        sA2P.SetCanonical(crCircleCenter,SmVector3d(1,0,0),SmVector3d(0,1,0));
        sA2P.RotateAboutAxis(dAngleRad,crCircleNormal);
        sA2P.TransformPoint(sToEye,sToP1);
        sA2P.SetCanonical(crCircleCenter,SmVector3d(1,0,0),SmVector3d(0,1,0));
        sA2P.RotateAboutAxis(-dAngleRad,crCircleNormal);
        sA2P.TransformPoint(sToEye,sToP2);

        rbFoundPoints = TRUE;
        rSilPoint1 = sToP1 * dCircleRadius;
        rSilPoint2 = sToP2 * dCircleRadius;
        return SM_SUCCESS;
    }

    // Handle parallel projection.

    SmVector3d sOrthVector = crCircleNormal * crEyePointOrVector;
    if (sOrthVector.LengthSquared() < SM_EFF_ZERO_SQ) {
        return SM_SUCCESS;
    }
    sOrthVector.Unitize();

    rbFoundPoints = TRUE;
    rSilPoint1 = crCircleCenter + sOrthVector * dCircleRadius;
    rSilPoint2 = crCircleCenter - sOrthVector * dCircleRadius;

    return SM_SUCCESS;

} // end smgu_CircleFindSilhouettePoints

/*******************************************************************//**
PURPOSE: Determine if two cones are perpendicularly disjoint.  This
    test is used to determine if something has a perpendicular to a
    vector field.

NOTES: 
***********************************************************************/
SM_EXPORT SmBoolean smgu_ConesAreDisjointPerpen
(
  const SmVector3d & crCone1Vec,      // in :
  double             sCone1AngRad,    // in :
  const SmVector3d & crCone2Vec,      // in :
  double             sCone2AngRad     // in :
)
{
    double dAngle;
    SE(crCone1Vec.AngleBetween(crCone2Vec,dAngle));
    // Use angle which is less than 90 degrees.
    dAngle = smos_Fabs(dAngle - SM_PI/2.0);
    if (dAngle > sCone1AngRad + sCone2AngRad + SM_EFF_ZERO_SQRT) {
        return TRUE; // Yes they are not perpendicularly disjoint
    }
    return FALSE;  // No they are not perpen disjoint

} // end smgu_ConesAreDisjointPerpen

/*******************************************************************//**
PURPOSE: This function intersects a line and a circle in the plane 
   of the circle.  If the line is not exactly in the plane of the circle 
   it will be projected to it.
   
NOTES: 
  If they do not intersect but are within d2dTolerance, a single intersection is returned.
  If two solutions are returned, they may be arbitrarily close together;
  dTolerance is not considered in that case.

  Use convenience function, smgu_CombineTolerantCircleIntersections() to apply
  the standard circle/line tolerance rule of combining all solutions
  that are within tolerance of one another and projecting that
  solution back onto the circle.

  See smgu_LineCoPlanarEllipseIntersect() for ellipses.


  Criteria for deciding whether to return 0, 1, or 2 intersections:
  1. If the line misses the circle by more than tol, 0 ints.
  2. If the line misses the circle by tol or less, 1 int at closest approach.
  3. If "exactly" tangent, 1 int.
  Now the line intersects the circle in two places.
  4. If the two intersection points are within tol of each other, 1 int (at midpoint).
  5. Else, 2 ints.

  Algorthm:
  - Drop circle center to line.
  - Get distance.
    This distance will be used for most of the tests.
  - if ( dist > R + tol )
    - no solutions
  - if ( dist >= R )
    - one solution
  - if ( dist <= R - tol )
    - definitely 2 sols

  Arrive here when dist > R-tol and dist < R: fuzzy area
  where noise could be a big factor.
  - Use trigonometry to get 3d distance between the intersection points.
    This is the short leg of the right triangle: circle center, drop point
    on the line, and the proposed intersection point.

    (Using trig instead of calculating a quadratic equation allows us to
    avoid some subtractive cancellation, which can dominate and render
    meaningless results when it's close.)

  - if that dist < tol,
    - one sol.

  Decision: In this very fuzzy area, we will check the results of
  using either 1 or 2 intersections, and use whichever is better.
  - Calculate a point on the line at the proposed intersection.
  - Compare the distance to the circle center from this intersection point
    with the distance from the dropped point to the center, and use
    whichever is closer to the circle radius.

  If you have to tweak this vis-a-vis tolerances, see also [B98].
***********************************************************************/
SM_EXPORT SmStatus smgu_LineCoPlanarCircleIntersect
(
  const SmPoint3d  & crLinePnt,                    // in : pt  of line = pt + param * vec
  const SmVector3d & crLineVec,                    // in : vec of line = pt + param * vec
  const SmPoint3d  & crCircleCenter,               // in : circle center
  const SmVector3d & crCircleNormal,               // in : circle normal
  double             dCircleRadius,                // in : circle radius at XAxis
  double             dTolerance,                   // in : max distance for near-miss intersection
  ULONG            & rlNumIntersections,           // out: 0, 1=tangent, 2=crossing 
  double             adLineIntersectParams[2]      // out: array of line intersection parameters
)
{
  rlNumIntersections=0;
  
  // locals
  SmPoint3d  sLinePnt;
  SmVector3d sLineVec;

  // project line to plane
  SER( smgu_PointProjectToPlane ( crLinePnt, crCircleCenter,crCircleNormal, sLinePnt ));
  SER( smgu_VectorProjectToPlane( crLineVec, crCircleNormal, sLineVec ));


  // Unitize the line vector.
  // This makes all line parameter values be 3-d distances.
  // We reverse the scaling when we return the line parameters.
  double dScale = sLineVec.Length();
  if (dScale < SM_EFF_ZERO * dCircleRadius )
    { SER( SM_ERR ); }
  sLineVec /= dScale;

  // Drop the circle center to the line.
  double dT;
  SER( smgu_LineClosestPoint( sLinePnt, sLineVec, crCircleCenter, dT ));
  SmPoint3d sClPt = sLinePnt + dT * sLineVec;

  // Branch to get easy cases out of the way.
  SmVector3d sVecToClPt( sClPt - crCircleCenter );
  double     dDistToCtrSq = sVecToClPt.Dot( sVecToClPt );

  double dDistToCtr = smos_Sqrt( dDistToCtrSq );

  // 1. Check too far away.
  if ( dDistToCtr > dCircleRadius + dTolerance )
    {
      // No intersections, too far away.
      rlNumIntersections = 0;
      return SM_SUCCESS;
    }

  // 2. Check for just tangent, or miss within tol: one int.
  if ( dDistToCtr >= dCircleRadius )
    {
      rlNumIntersections = 1;
      adLineIntersectParams[0] = dT / dScale;
      return SM_SUCCESS;
    }

  // We'll need these quantities for the next tests.
  // These are the distance (squared) along the line,
  // between the closest point and the solutions.
  // These are actual 3d distances, because LineVec has been unitized.

  // We'll use right triangles instead of the algebraic formulation,
  // mainly because we already have the quantities calculated.
  // Also, it's probably more stable, because it doesn't include
  // the line definition point (sLinePnt), which could be anywhere.

  //   double dDistToMoveSq = dCircleRadius*dCircleRadius - dDistToCtrSq;
  //   double dDistToMove   = smos_Sqrt( dDistToMoveSq );

  // But: when dDistToCtr is close to Radius, subtracting their squares
  // can lead to serious subtractive cancellation, and can cause real errors.
  // This is exactly the situation we're looking out for.
  // That cancellation can be avoided by rearranging the calculation:
  // If l is the distance to move (the short side of the right triangle;
  // the radius is the hypotenuse), and d is the center/line distance,
  // then one way to do that math is:
  //
  //  l^2 = R^2 - d^2
  //      = (R+d) * (R-d)
  //
  // R+d is fine, and R-d is *much* better than R^2 - d^2.

  double dDistToMoveSq = ( dCircleRadius + dDistToCtr ) * ( dCircleRadius - dDistToCtr );
  double dDistToMove   = smos_Sqrt( dDistToMoveSq );

  if ( dDistToCtr <= dCircleRadius - dTolerance )
    {
      // Here there are definitely two solutions.
      rlNumIntersections = 2;

      // What are the line parameters.  dT is based on a unit vector, so it
      // corresponds to unit distance, so we just add dDistToMove to it,
      // then scale it.
      adLineIntersectParams[0] = ( dT - dDistToMove ) / dScale;
      adLineIntersectParams[1] = ( dT + dDistToMove ) / dScale;
      return SM_SUCCESS;
    }

  // Next test: if the resulting intersection points will be within
  // tolerance of each other, then it's definitely one intersection.
  // (Points will be separated by twice dDistToMove.)
  if ( dDistToMove * 2.0 <= dTolerance )
    {
      rlNumIntersections = 1;
      adLineIntersectParams[0] = dT / dScale;
      return SM_SUCCESS;
    }


  // Now we're in the fuzzy area.  dDistToMove is very small,
  // less than 2*tol, so the grazing distance between the line
  // is extremely small, on the order of dDistToMove^2.

  // Heuristic alert: we'll calculate what we have for the two
  // intersection points, on the line, and then compare the distance from
  // those points to the center against the distance from the line drop
  // point to the center, and use whichever is closer to the radius.

  double dT0 = dT - dDistToMove; // these are still 3d distances.
  double dT1 = dT + dDistToMove;

  SmPoint3d sIntPt0 = sLinePnt + dT0 * sLineVec;
  double dDistToInt = sIntPt0.DistanceBetween( crCircleCenter );

  double dErrorDropPt = smos_Fabs( dCircleRadius - dDistToCtr );
  double dErrorIntPts = smos_Fabs( dCircleRadius - dDistToInt );

  if ( dErrorDropPt < dErrorIntPts )
    {
      rlNumIntersections = 1;
      adLineIntersectParams[0] = dT / dScale;
    }
  else
    {
      rlNumIntersections = 2;

      adLineIntersectParams[0] = dT0 / dScale;
      adLineIntersectParams[1] = dT1 / dScale;
    }


// *****************************************************
// old way:
//
//  // get Line/CircleCenter distance
//  double dDist;
//  SER(smgu_LinePointDistance(sLinePnt,sLineVec,crCircleCenter,dDist));
//
//  // Test for missed intersection
//  if (dDist > dCircleRadius + dTolerance) 
//    {
//      rlNumIntersections = 0;
//      return SM_SUCCESS;
//    }
//  // arrive here when dDist <= dCircleRadius + d2Tolerance
//
//  // Test for touching or near touching intersection
//  // Note, as per the Usage Notes, return two solutions unless it's
//  // essentially exact: SM_EFF_ZERO is too big.  [B98]
//  // Actually just leave this test out: it gets checked later anyway.
//  // if ( dDist > dCircleRadius - SM_EFF_ZERO * (1.0 + dCircleRadius))
//  //   {
//  //     rlNumIntersections = 1;
//  //     double dParam;
//  //     SER(smgu_LineClosestPoint(sLinePnt,sLineVec,crCircleCenter,dParam));
//  //     adLineIntersectParams[0] = dParam;
//  //     return SM_SUCCESS;
//  //   }
//
//  if ( cbi3==0 ) 
//    {
//      // Now do normal intersection
//      SmVector3d sG = sLinePnt - crCircleCenter;
//      double a = sLineVec.Dot(sLineVec);
//      double b = 2.0 * (sLineVec.Dot(sG));
//      double c = sG.Dot(sG) - dCircleRadius*dCircleRadius;
//      double d = b*b - 4.0*a*c;  <-- NOTE: vary bad subtractive cancellation here.
//      if (smos_Fabs(d) < SM_EFF_ZERO_SQ) 
//        {
//          // Single answer case
//          rlNumIntersections = 1;
//          adLineIntersectParams[0] = ( - b / (2.0*a));
//          return SM_SUCCESS;
//        }
//      if (d < 0.0) 
//        {
//          // No analytic intersection - however we may be close enough to justify
//          // an intersection relative to our tolerance.  Test to see.
//          double dT;
//          SER(smgu_LineClosestPoint(sLinePnt,sLineVec,crCircleCenter,dT));
//          SmPoint3d  sClPt      = sLinePnt + dT * sLineVec;
//          SmVector3d sVecToClPt = sClPt - crCircleCenter;
//          if (smos_Fabs(sVecToClPt.Length()-dCircleRadius) > dTolerance) 
//            {
//              rlNumIntersections = 0;
//              return SM_SUCCESS;
//            }
//          // If made it here we have one intersection which is just a little outside
//          // of the circle.
//          rlNumIntersections = 1;
//          adLineIntersectParams[0] = dT;
//          return SM_SUCCESS;
//        }
//
//      // If make it here we have two intersections
//      rlNumIntersections = 2;
//      double dSqrtD = smos_Sqrt(d);
//      adLineIntersectParams[0] = ( - b - dSqrtD ) / (2.0 * a);
//      adLineIntersectParams[1] = ( - b + dSqrtD ) / (2.0 * a);
//
//    } 

#ifdef SM_DEBUG_CODE
  // check the accuracy of the solutions
  // (Note that we won't get here for the straightforward cases.)
  SmPoint3d sLinePoint1, sLinePoint2, sVec ;
  double dDist1, dDist2, dScaledZero ;

  // 1st point solution
  if(rlNumIntersections > 0)
    {
      sLinePoint1 = crLinePnt + adLineIntersectParams[0] * crLineVec ;
      SER( smgu_PointProjectToPlane( sLinePoint1, crCircleCenter, crCircleNormal, sLinePoint1 ));
      sVec        = sLinePoint1 - crCircleCenter;
      dDist1      = sVec.Length() ;
      dScaledZero = SM_EFF_ZERO * (1.0 + sLinePoint1.GetMaxDimension()
                                              + sVec.GetMaxDimension()) ;
      if ( ! SM_ARE_SAME_TO_TOL(dDist1, dCircleRadius, dScaledZero) )
        { SM_ASSERT( FALSE ); } ;
    }

  // 2nd point solution
  if(rlNumIntersections > 1)
    {
      sLinePoint2 = crLinePnt + adLineIntersectParams[1] * crLineVec ;
      SER( smgu_PointProjectToPlane( sLinePoint2, crCircleCenter, crCircleNormal, sLinePoint2 ));
      sVec        = sLinePoint2 - crCircleCenter;
      dDist2      = sVec.Length() ;
      dScaledZero = SM_EFF_ZERO * (1.0 + sLinePoint2.GetMaxDimension()
                                              + sVec.GetMaxDimension()) ;
      if ( ! SM_ARE_SAME_TO_TOL(dDist2, dCircleRadius, dScaledZero) )
        { SM_ASSERT( FALSE ); } ;
    }
#endif // SM_DEBUG_CODE

  // all done
  return SM_SUCCESS;

} // end smgu_LineCoPlanarCircleIntersect

/*******************************************************************//**
PURPOSE: This function intersects a line and an ellipse in the plane
   of the ellipse.  If the line is not exactly in the plane of the ellipse
   it will be projected to it.

NOTES: 
  If they do not intersect but are within dTolerance, a single intersection is returned.
  If two solutions are returned, they may be arbitrarily close together;
  dTolerance is not considered in that case.

  Use the convenience functions smgu_CombineTolerantPoints() and
  smgu_DropPointToEllipse() to implement the tolerant intersection
  rule that 2 point solutions within tolerance of one another are
  combined into 1 point solution projected onto the ellipse.

  See smgu_LineCoPlanarCircleIntersect() for circles.
***********************************************************************/
SM_EXPORT SmStatus smgu_LineCoPlanarEllipseIntersect
(
  const SmPoint3d  & crLinePnt,                // in : pt  of line = pt + param * vec
  const SmVector3d & crLineVec,                // in : vec of line = pt + param * vec
  const SmPoint3d  & crEllipseCenter,          // in : ctr of ellipse = ctr + rx * x * cos(s) + ry * y * sin(s) 
  const SmVector3d & crEllipseXAxis,           // in : x   of ellipse = ctr + rx * x * cos(s) + ry * y * sin(s) 
  double             dEllipseXRadius,          // in : rx  of ellipse = ctr + rx * x * cos(s) + ry * y * sin(s) 
  const SmVector3d & crEllipseYAxis,           // in : y   of ellipse = ctr + rx * x * cos(s) + ry * y * sin(s) 
  double             dEllipseYRadius,          // in : ry  of ellipse = ctr + rx * x * cos(s) + ry * y * sin(s) 
  double             dTolerance,               // in : min distance between distinct points
  ULONG            & rlNumIntersections,       // out: 0, 1=tangent, 2=crossing 
  double             adLineIntersectParams[2]  // out: array of line intersection parameters
) 
{
  SmPoint3d  sLinePnt;
  SmVector3d sLineVec;

  // check input
  SM_ASSERT(!SM_IS_ZERO (crLineVec.Length())) ;
  SM_ASSERT(!SM_IS_ZERO (dEllipseXRadius)) ;
  SM_ASSERT(!SM_IS_ZERO (dEllipseYRadius)) ;
  SM_ASSERT( SM_ARE_SAME(crEllipseXAxis.Length(),1.0)) ;
  SM_ASSERT( SM_ARE_SAME(crEllipseYAxis.Length(),1.0)) ;
  SM_ASSERT( SM_IS_ZERO (crEllipseXAxis.Dot(crEllipseYAxis))) ;

  // get Ellipse plane normal
  SmVector3d sEllipseNormal = crEllipseXAxis * crEllipseYAxis ;
  SM_ASSERT(sEllipseNormal.Length() > SM_EFF_ZERO) ;
  sEllipseNormal.Unitize() ;

  // project line to plane
  SER(smgu_PointProjectToPlane(crLinePnt,crEllipseCenter,sEllipseNormal,sLinePnt));
  SER(smgu_VectorProjectToPlane(crLineVec,sEllipseNormal,sLineVec));

  // transform problem into a circle of radius ry/line problem
  //  Let transform be defined by
  //    z = w + x * ((w-ctr)*x) * (ry-rx)/rx
  //  where z = points after transform
  //        w = points prior to transform
  double     dScale    = (dEllipseYRadius - dEllipseXRadius) / dEllipseXRadius ;
  SmPoint3d  sTLinePnt = crLinePnt + crEllipseXAxis * (crLinePnt-crEllipseCenter).Dot(crEllipseXAxis) * dScale ;
  SmVector3d sTLineVec = crLineVec + crEllipseXAxis * crLineVec.Dot(crEllipseXAxis) * dScale ;

  // pass the call along to the circle/line solver
  SER(smgu_LineCoPlanarCircleIntersect(sTLinePnt,
                                       sTLineVec,
                                       crEllipseCenter,
                                       sEllipseNormal,
                                       dEllipseYRadius,
                                       dTolerance,
                                       rlNumIntersections,
                                       adLineIntersectParams)) ;

#ifdef SM_DEBUG_CODE
  // check the accuracy of the solutions
  SmPoint3d sLinePoint1, sLinePoint2, sVec ;
  SmPoint3d sEllipsePoint1, sEllipsePoint2 ;
  double dAngle1, dAngle2, dX, dY ;
  double dDist1, dDist2, dLineDist, dEllipseDist, dScaledZero ;

  // 1st point solution
  if(rlNumIntersections > 0)
    {
      sLinePoint1 = sLinePnt + adLineIntersectParams[0] * sLineVec ;
      // from tan = m_dRadiusAtXAxis/m_dRadiusAtYAxis * dY / dX
      // get dAngleParam in radians [-Pi, +Pi]
      sVec    = sLinePoint1 - crEllipseCenter;
      dX      = sVec.Dot(crEllipseXAxis);
      dY      = sVec.Dot(crEllipseYAxis);
      dAngle1 = smos_ArcTangent2(dEllipseXRadius/dEllipseYRadius * dY, dX) ;

      // compare evaluations
      sEllipsePoint1 =   crEllipseCenter
                       + smos_Cosine(dAngle1) * dEllipseXRadius * crEllipseXAxis
                       + smos_Sine  (dAngle1) * dEllipseYRadius * crEllipseYAxis ;
      dDist1         = sEllipsePoint1.DistanceBetween(sLinePoint1) ;
      dLineDist      = (sVec).Length() ;
      dEllipseDist   = (sEllipsePoint1 - crEllipseCenter).Length() ;
      dScaledZero    = SM_EFF_ZERO * (1.0 + sLinePoint1.GetMaxDimension()
                                          + sLineVec.GetMaxDimension()
                                          + smos_Max(dEllipseXRadius,dEllipseYRadius)) ;
      if (    dDist1 >= dScaledZero
          && (dLineDist <= dEllipseDist || dDist1 > dTolerance))
        { SM_ASSERT( FALSE ); } // cbi breakpoint
    }

  // 2nd point solution
  if(rlNumIntersections > 1)
    {
      sLinePoint2 = sLinePnt + adLineIntersectParams[1] * sLineVec ;
      // from tan = m_dRadiusAtXAxis/m_dRadiusAtYAxis * dY / dX
      // get dAngleParam in radians [-Pi, +Pi]
      sVec    = sLinePoint2 - crEllipseCenter;
      dX      = sVec.Dot(crEllipseXAxis);
      dY      = sVec.Dot(crEllipseYAxis);
      dAngle2 = smos_ArcTangent2(dEllipseXRadius/dEllipseYRadius * dY, dX) ;

      // compare evaluations
      sEllipsePoint2 =   crEllipseCenter
                       + smos_Cosine(dAngle2) * dEllipseXRadius * crEllipseXAxis
                       + smos_Sine  (dAngle2) * dEllipseYRadius * crEllipseYAxis ;
      dDist2         = sEllipsePoint2.DistanceBetween(sLinePoint2) ;
      dLineDist      = (sVec).Length() ;
      dEllipseDist   = (sEllipsePoint2 - crEllipseCenter).Length() ;
      dScaledZero    = SM_EFF_ZERO * (1.0 + sLinePoint2.GetMaxDimension()
                                          + sLineVec.GetMaxDimension()
                                          + smos_Max(dEllipseXRadius,dEllipseYRadius)) ;
      SM_ASSERT(    dDist2 < dScaledZero
                || (dLineDist > dEllipseDist && dDist2 <= dTolerance)) ;
    }
#endif // SM_DEBUG_CODE

  return SM_SUCCESS;

} // end smgu_LineCoPlanarEllipseIntersect

/*******************************************************************//**
PURPOSE: Intersect two circles in a common plane.

NOTES: If the center of the circle 2 is not in the plane defined
    by the normal and the center of circle 1 then it will be projected.
    This means that the points may not necessarily lie on circle2.

    When the circles are within dTolerance of one another a single
    point solution is returned which lies on the circle 1 point which
    is closest to circle 2.   If two solutions are returned, 
    they may be arbitrarily close together; dTolerance is not considered 
    in that case.

    Use convenience function, smgu_CombineTolerantCircleIntersections() to apply
    the standard circle/circle tolerance rule of combining all solutions
    that are within tolerance of one another and projecting that
    solution back onto the circle.
***********************************************************************/
SM_EXPORT SmStatus smgu_CircleCoPlanarCircleIntersect
(
  const SmVector3d & crCircleNormals,      // in : normal for both circles
  const SmPoint3d  & crCircle1Center,      // in : Circle1 CenterPoint 
  double             dCircle1Radius,       // in : Circle1 Radius 
  const SmPoint3d  & crCircle2Center,      // in : Circle2 CenterPoint
  double             dCircle2Radius,       // in : Circle2 Radius     
  double             dTolerance,           // in : min distance between distinct 3d Points
  ULONG            & rlNumIntersections,   // out: number of intersections
                                           //      0 = none
                                           //      1 = tangent circles
                                           //      2 = intersecting circles
                                           //      3 = coincident circles
  SmPoint3d aIntersectionPoints[2]         // out: intersection points
)
{
  SmPoint3d sCircle2Center;

  // project circle2 Center to circle1 Plane
  SER(smgu_PointProjectToPlane(crCircle2Center,crCircle1Center,
      crCircleNormals,sCircle2Center));

  // check center/center distance
  double dCCDist = crCircle1Center.DistanceBetween(sCircle2Center);

  // when centers are coincident
  // gwc: tolerance note - as coded this would count two
  //      circles as coincident which in the worst case
  //      are tangent on one side and 2*tol (not tol) apart on the other.
  if (dCCDist < dTolerance) 
    {
      // check for coincident/concentric circles
      rlNumIntersections =   smos_Fabs(dCircle1Radius-dCircle2Radius) < dTolerance
                           ? 3   // coincident circles
                           : 0 ; // concentric circles  
      return SM_SUCCESS;
    }

  // arrive here when circle centers are not coincident

  // no work - no intersections 
  //    - circles too far apart, or
  //    - one small circle inside another and too far from edge
  if(   dCCDist > dCircle1Radius + dCircle2Radius + dTolerance  // too far apart (dTolerance)
     || dCCDist + dCircle1Radius < dCircle2Radius - dTolerance  // small circle1 too near circle2 center (dTolerance)
     || dCCDist + dCircle2Radius < dCircle1Radius - dTolerance) // small circle2 too near circle1 center (dTolerance)
    {
      rlNumIntersections = 0;  // no intersections too far apart
      return SM_SUCCESS;
    }

  // arrive here when circles are within dTolerance of one another or intersecting
  // and circle centers are not coincident.

  // when circles are tangent - 
  //   outside circles within tol to touching (SM_EFF_ZERO), or
  //   small circle1 inside circle2 within tol to touching (SM_EFF_ZERO), or
  //   small circle2 inside circle1 within tol to touching (SM_EFF_ZERO)
  //     note: no intersection case has already enforced the within tol bound,
  //           so just check the other bound, touching by no more than SM_EFF_ZERO
  double dZero = SM_EFF_ZERO * (1.0 + dCircle1Radius + dCircle2Radius );
  if (   dCCDist > dCircle1Radius + dCircle2Radius - dZero  // outside circles touching by no more than dZero
      || dCircle1Radius > dCCDist + dCircle2Radius - dZero  // small circle2 inside circle1 touching by no more than dZero
      || dCircle2Radius > dCCDist + dCircle1Radius - dZero) // small circle1 inside circle2 touching by no more than dZero
    {
      // Tangent intersection - from either inside or outside.
      rlNumIntersections = 1;
      SmVector3d sVec( sCircle2Center - crCircle1Center );
      sVec /= dCCDist; // Unitize

      // Calculate the point from both circles and average:
      // a little noise can make a big difference with tangent intersections.
      SmPoint3d sPt1, sPt2;
      double dScaledZero = SM_EFF_ZERO * (1.0 + crCircle1Center.GetMaxDimension());
      if (dCCDist < dCircle1Radius + dCircle2Radius - dScaledZero )
        { // Inside Cases
          if (dCircle2Radius < dCircle1Radius) 
            {
              sPt1 = crCircle1Center + dCircle1Radius * sVec;
              sPt2 = crCircle2Center + dCircle2Radius * sVec;
            }
          else 
            {
              sPt1 = crCircle1Center - dCircle1Radius * sVec;
              sPt2 = crCircle2Center - dCircle2Radius * sVec;
            }
        }
      else 
        { // Outside Case
          sPt1 = crCircle1Center + dCircle1Radius * sVec;
          sPt2 = crCircle2Center - dCircle2Radius * sVec;
        }

      aIntersectionPoints[0] = 0.5 * ( sPt1 + sPt2 );

#if SM_DEBUG_CODE
      double dDist = sPt1.DistanceBetween( sPt2 );
      SM_ASSERT( dDist < dTolerance );
#endif // SM_DEBUG_CODE

      return SM_SUCCESS;
    } // end tangent circle check

  // arrive here when two circles intersect in two points.

  // get distance to intersection points along vectors vec0 and vec1
  // from circle1Center, with: 
  //   vec0 = Circle2Center-Circle1Center
  //   vec1 = perpendicular to vec0 in circle plane
  double dHeight, dDistAlong;
  SmStatus dRtn = smgu_TriangleDecomposition(dCCDist,
                                             dCircle1Radius,
                                             dCircle2Radius,
                                             dDistAlong,
                                             dHeight) ;
  SM_ASSERT_MSG(dRtn == SM_SUCCESS,
                _T("smgu_CircleCoPlanarCircleIntersect: 2 Pt case failed when expected to succeed - Tolerances need review.")) ;
  SER(dRtn);
  
  // build Vec0
  SmVector3d sCCVec = sCircle2Center - crCircle1Center;
  SER(sCCVec.Unitize());

  // build Vec1
  SmVector3d sPerpVec = sCCVec * crCircleNormals;
  SER(sPerpVec.Unitize());

  // set output
  rlNumIntersections = 2;
  aIntersectionPoints[0] =   crCircle1Center 
                           + dDistAlong * sCCVec 
                           + dHeight * sPerpVec;
  aIntersectionPoints[1] =   crCircle1Center 
                           + dDistAlong * sCCVec 
                           - dHeight * sPerpVec;

  // all done
  return SM_SUCCESS;

} // end smgu_CircleCoPlanarCircleIntersect

/*******************************************************************//**
PURPOSE: Intersect a line and a sphere paying attention to tolerances.

NOTES: 
***********************************************************************/
SM_EXPORT SmStatus smgu_LineIntersectSphere
(
  const SmVector3d & crLinePnt,        // in : 
  const SmVector3d & crLineVec,        // in : 
  const SmVector3d & crSphereCenter,   // in : 
  double             dSphereRadius,    // in : 
  double             dTolerance,       // in : 
  ULONG            & rlNumberResults,  // out: rlNumberResults = 0 - no intersection, Line further than dTol from Sphere
                                       //      rlNumberResults = 1 - touching intersection, Line outside Sphere
                                       //                                   by less than dTol to touching (SM_EFF_ZERO)
                                       //      rlNumberResults = 2 - through intersecton, Line inside Sphere 
                                       //                                   by more than dTol
                                       //      rlNumberResults = 3 - grazing, Line inside Sphere by SM_EFF_TOL to dTol
                                       //                        adResults[0] = 1st Intersection Point
                                       //                        adResults[1] = average Intersection Point on Line
                                       //                        adResults[2] = 2nd Intersection Point
  double             adResults[3]      // out: line parameters of line/sphere intersections
)
{
  // init output
  rlNumberResults = 0;

  // locals
  SmVector3d V        = crLineVec;
  double     dLengthV = V.Length();

  // check input
  if (dLengthV < SM_EFF_ZERO) SER(SM_ERR_INVALID_INPUT);

  // unitize V
  V = V / dLengthV;  // Unitizes it

  // locals, v = vector from LinePoint to PointNearestSphereCenter
  //         b = line/Sphere Center distance
  SmVector3d sEO              = crSphereCenter - crLinePnt;
  double     dLengthSquaredEO = sEO.LengthSquared();
  double     v                = sEO.Dot(V);  // projection of EO to line vector
  double     b                = smos_Sqrt((dLengthSquaredEO) - (v*v));

  // no intersection - line further from Sphere Center than dTolerance
  if (b > dSphereRadius + dTolerance) 
    {  
      rlNumberResults = 0;
      return SM_SUCCESS;
    }

  // tangent - exactly touching (SM_EFF_ZERO) to dTolerance outside the sphere
  if (b > dSphereRadius - SM_EFF_ZERO*(1.0+dSphereRadius)) 
    { rlNumberResults = 1; 
      adResults[0] = v / dLengthV; // convert back to original line parameterization
      return SM_SUCCESS;
    }

  // arrive here for 2 pt solutions

  // compute 1/2 distance between the point solutions
  double disc = dSphereRadius*dSphereRadius - b*b;
  if (disc < SM_EFF_ZERO) 
    { SER(SM_ERR); } // can't really happen
  double d = smos_Sqrt(disc);

  // line distance lies from just about touching (SM_EFF_ZERO) to inside Sphere by dTol
  if (b > dSphereRadius - dTolerance) 
    { // grazing condition
      rlNumberResults = 3;
      adResults[0] = (v-d) / dLengthV;  // actual 1st intersection
      adResults[1] =   v   / dLengthV;  // mid point
      adResults[2] = (v+d) / dLengthV;  // actual 2nd intersection
      return SM_SUCCESS;
    }

  // now handle only remaining case - where line intersects at two points
  rlNumberResults = 2;
  adResults[0] = (v-d) / dLengthV;
  adResults[1] = (v+d) / dLengthV;

  // all done
  return SM_SUCCESS;

} // end smgu_LineIntersectSphere

/*******************************************************************//**
PURPOSE: Find the intersection points between two circles known
            to be on the same sphere.

NOTES: This is a pretty specialized
   convenience function for SmSphere implementation.  Don't expect it
   to be of much general use.

    When the circles are within dTolerance of one another a single
    point solution is returned which is placed on the Cir1 Curve as
    close to Cir2 as possible.   If two solutions are returned, 
    they may be arbitrarily close together; dTolerance is not considered 
    in that case.

    Use convenience function, smgu_CombineTolerantCircleIntersections() to apply
    the standard circle/circle tolerance rule of combining all solutions
    that are within tolerance of one another and projecting that
    solution back onto the circle.
***********************************************************************/
SM_EXPORT SmStatus smgu_SphereCircleSphereCircleIntersect
(
  const SmPoint3d  & rSphereCenter,    // in : center of Sphere
  double             dSphereRadius,    // in : radius of Sphere
  const SmPoint3d  & rCir1Center,      // in : center of circle 1
  const SmVector3d & rCir1Normal,      // in : normal vector defining plane of circle 1
  const SmPoint3d  & rCir2Center,      // in : center of circle 2                      
  const SmVector3d & rCir2Normal,      // in : normal vector defining plane of circle 2
  double             dTol3d,           // in : max distance between coincident points
  ULONG            & lIntersectCount,  // out: 0 = no intersections,
                                       //      1 = 1 tangent intersection point
                                       //      2 = 2 crossing intersection points
                                       //      3 = coincident circles 
  SmPoint3d         aPoints[2]         // out: Associated intersection points
)
{
  // Method: the circles are (by definition here) the intersections
  // of a plane and a sphere.  So we have three surfaces -- a sphere
  // and two planes.  The result is the srf/srf/srf intersection.
  // For that, all we have to do is to intersect the line of intersection
  // of the two planes:   smgu_IntersectTwoPlanes()
  // with the sphere:     smgu_LineIntersectSphere().

  // Init output.
  lIntersectCount = 0;

  // Locals.
  SmPoint3d  sLinePt;
  SmVector3d sLineVec;
  double adLineParams[3];
  SmStatus sRtn;
  ULONG i;

  // Get line of intersection of the planes.
  sRtn = (smgu_IntersectTwoPlanes( rCir1Center, rCir1Normal, rCir2Center, rCir2Normal,
                                   sLinePt, sLineVec ));
  if ( sRtn != SM_SUCCESS )
    {
      // Parallel planes.  If they're coincident, return 3, else 0.
      sLineVec = rCir1Center - rCir2Center;
      double dDot = sLineVec.Dot( rCir1Normal );
  
      // coincident test/ disjoint test
      lIntersectCount = ( smos_Fabs( dDot ) < dTol3d )
                        ? 3 
                        : 0 ;
      return ( SM_SUCCESS );
    }

  // Intersect that line with the sphere.
  sRtn = ( smgu_LineIntersectSphere( sLinePt, sLineVec, rSphereCenter, dSphereRadius, dTol3d,
                                     lIntersectCount, adLineParams ));
  if ( sRtn != SM_SUCCESS )
    {
      return SM_SUCCESS; // Can't really happen: only if sLineVec has zero length.
    }

  // Check and load outputs for grazing case (just inside the sphere)
  if ( lIntersectCount == 3 )
    {
      // smgu_LineIntersectSphere() returns 3 if grazing (just inside the sphere), 
      //     adLineParams[0] 1st XSect Pt, 
      //     adLineParams[1] midpoint on line, 
      //     adLineParams[2] 2nd XSect Pt.
      // So leave out the middle one.
      lIntersectCount = 2;
      adLineParams[1] = adLineParams[2];
    }

  // check for tolerant solutions when the intersection curve returns 0 or 1 solutions
  if(   lIntersectCount == 0
     || lIntersectCount == 1)
    {
      // compute the nearest point/point approach of the two circles to one another
      // this should happen in the plane of the sphere center and the two circle centers
      SmVector3d sCenterToCenter = rCir2Center - rCir1Center ;
      SmVector3d sCir1Dir        = rCir1Normal.NormalInPlane(rCir2Normal) ;
      SmVector3d sCir2Dir        = rCir2Normal.NormalInPlane(rCir1Normal) ;

      // sCir1Dir or sCir2Dir can be zero when the two Normals are parallel
      if(   sCir1Dir.Length() <= SM_EFF_ZERO
         || sCir2Dir.Length() <= SM_EFF_ZERO)
        {
          // coincident or parallel solution should already have been found 
          WARN(_T("smgu_SphereCircleSphereCircleIntersect: unexpected branch - needs review")) ;
          return(SM_ERR) ;
        }

      // get the direction from the circle centers to the circle points in the working plane
      sCir1Dir.Unitize() ;
      sCir2Dir.Unitize() ;

      // get the radius of each circle
      double     dCir1Radius2    = dSphereRadius * dSphereRadius - rCir1Center.DistanceBetweenSquared(rSphereCenter) ;
      double     dCir2Radius2    = dSphereRadius * dSphereRadius - rCir2Center.DistanceBetweenSquared(rSphereCenter) ;
      double     dCir1Radius     = dCir1Radius2 >= 0.0 ? smos_Sqrt(dCir1Radius2) : 0.0 ;
      double     dCir2Radius     = dCir2Radius2 >= 0.0 ? smos_Sqrt(dCir2Radius2) : 0.0 ;

      // there are two circle points on the plane for each circle and we need to 
      // find the pair of points which are the closest together.  I don't see a 
      // way to determine that in advance, so just check all 4 distances and save the
      // best one.
      ULONG ii ;
      SmPoint3d sP1, sP2 ;
      double dLength ;
      SmPoint3d sBestP1     = rCir1Center + dCir1Radius * sCir1Dir ; 
      SmPoint3d sBestP2     = rCir2Center + dCir2Radius * sCir2Dir ; 
      double    dBestLength = sBestP1.DistanceBetweenSquared(sBestP2) ;
      for(ii=1;ii<4;ii++)
        {
          sP1 = rCir1Center + (ii > 1    ? -1 : +1) * dCir1Radius * sCir1Dir ;
          sP2 = rCir2Center + (ii%2 == 0 ? -1 : +1) * dCir2Radius * sCir2Dir ;
          dLength = sP1.DistanceBetweenSquared(sP2) ;
          if(dLength < dBestLength)
            { dBestLength = dLength ;
              sBestP1     = sP1 ;
              sBestP2     = sP2 ;
            }
        } // end iter all test points
      dBestLength = smos_Sqrt(dBestLength) ;

      // now check the tolerance
      if(dBestLength > dTol3d)
        {
          // there are no solutions
          lIntersectCount = 0 ;
        }
      else // there is one tolerant solution
        {
          // place the Circ1 intersection into the output
          lIntersectCount = 1 ;
          aPoints[0] = sBestP1 ;
        }
          
      // compute the vector that runs from the closes Cir1 point to the closestCir2 point on the plane
      SmPoint3d sDist =   sCenterToCenter
                        - smos_Sgn(sCenterToCenter.Dot(sCir1Dir)) * dCir1Radius * sCir1Dir
                        + smos_Sgn(sCenterToCenter.Dot(sCir2Dir)) * dCir2Radius * sCir2Dir ;
    } // end 0 or 1 intersection point solutions branch
  else // load the 2 Solutions into the output 
    {
      // for every XSect Solution
      for ( i = 0; i < lIntersectCount; i++ )
        {
          // Note, putting them in backwards.  Shouldn't make any difference,
          // but it matches the way it used to work.

          // build output intersection
          aPoints[lIntersectCount-1-i] = sLinePt + adLineParams[i] * sLineVec;
        }
    } // end 2 intersection point solutions branch

#ifdef SM_DEBUG_CODE

  double dScaledZero  = SM_EFF_ZERO * (1.0 + smos_Max(rCir1Center.GetMaxDimension(),
                                                      rCir2Center.GetMaxDimension())) ;

  // get circleCenter/SphereCenter gap vectors and sizes of both circles
  SmVector3d sGap1        = rSphereCenter - rCir1Center ;
  SmVector3d sGap2        = rSphereCenter - rCir2Center ;
  double     dGap1Length  = sGap1.Length() ;
  double     dGap2Length  = sGap2.Length() ;
  double     dCir1Radius2 = dSphereRadius * dSphereRadius - dGap1Length * dGap1Length ;
  double     dCir2Radius2 = dSphereRadius * dSphereRadius - dGap2Length * dGap2Length ;

  if(   (dCir1Radius2 < -dScaledZero)
     || (dCir2Radius2 < -dScaledZero)
     || (dGap1Length > dScaledZero && !sGap1.IsParallelTo(rCir1Normal,1.0))
     || (dGap2Length > dScaledZero && !sGap2.IsParallelTo(rCir2Normal,1.0)))
    { SM_ASSERT( FALSE ); }

  double dCir1Radius = smos_Sqrt(smos_Fabs(dCir1Radius2)) ;
  double dCir2Radius = smos_Sqrt(smos_Fabs(dCir2Radius2)) ;

SmBoolean bDebugMe = FALSE ;
  if(bDebugMe)
    {
      // make the sphere and circles and draw them
      SmContext sContext ;
      SmSphere *pSphere ;
      SmBSplineCurve  *pCurve1, *pCurve2 ;
      SmCircle *pCirc1, *pCirc2 ;
      SmVector3d sX1, sX2, sY1, sY2, sZ1, sZ2 ;
      rCir1Normal.MakeUnitOrthoVectors(NULL, sZ1, sX1, sY1) ;
      rCir2Normal.MakeUnitOrthoVectors(NULL, sZ2, sX2, sY2) ;
      SmAxis2Placement sSpherePlacement(rSphereCenter, SmVector3d(1.0, 0.0, 0.0), SmVector3d(0.0, 1.0, 0.0)) ;
      SmAxis2Placement sCirc1Placement (rCir1Center, sX1, sY1) ;
      SmAxis2Placement sCirc2Placement (rCir2Center, sX2, sY2) ;
      SmSphere::CreateCanonical(sContext, sSpherePlacement, dSphereRadius, pSphere) ;
      if(dCir1Radius > dScaledZero) { SmCircle::CreateCanonical(sContext, sCirc1Placement, dCir1Radius, pCirc1) ;
                                      pCurve1 = pCirc1 ;
                                    }
      else                          { SmBSplineCurve::CreateDegenerateCurve(sContext, 3, rCir1Center, pCurve1) ;
                                    }
      if(dCir2Radius > dScaledZero) { SmCircle::CreateCanonical(sContext, sCirc2Placement, dCir2Radius, pCirc2) ;
                                      pCurve2 = pCirc2 ;
                                    }
      else                          { SmBSplineCurve::CreateDegenerateCurve(sContext, 3, rCir2Center, pCurve2) ;
                                    }
      
      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0,1,1) ; pSphere->DrawUV() ; sm_GraphicsLoop() ; 
      smgfx_SetLook(3,4, 0,0,1) ; pCurve1->Draw() ; sm_GraphicsLoop() ; 
      smgfx_SetLook(4,5, 0,1,0) ; pCurve2->Draw() ; sm_GraphicsLoop() ; 
      smgfx_SetLook(10,11, 1,0,0) ; if(lIntersectCount > 0) aPoints[0].Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook(12,13, 1,0,1) ; if(lIntersectCount > 1) aPoints[1].Draw() ; sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;

    } // end graphics

  // check that xSectPnts lie on Sphere and both circles

  // Get direction perpendicular to both circle normals
  SmVector3d sCN1     = rCir1Normal ;  sCN1.Unitize() ;
  SmVector3d sCN2     = rCir2Normal ;  sCN2.Unitize() ;
  SmVector3d sZ       = sCN1 * sCN2 ;
 // double     dZLength = sZ.Length() ; // dZLength ~= angle in radians between Cir1Norm and Cir2Norm when angle is small

  for(ULONG ii=0;ii<lIntersectCount;ii++)
    {
      // xsect point is on sphere
      double dSphereDist = (aPoints[ii] - rSphereCenter).Length() ;
      SmBoolean bTest = SM_ARE_SAME(dSphereDist, dSphereRadius) ;
      if(bTest == FALSE)
        { SM_ASSERT(bTest) ; }

      // xsect point is on both circle planes to within tolerance
      double dCirPlane1Dist = (aPoints[ii] - rCir1Center).Dot(sCN1) ;
      double dCirPlane2Dist = (aPoints[ii] - rCir2Center).Dot(sCN2) ;
      bTest =    dCirPlane1Dist < dTol3d 
              && dCirPlane2Dist < dTol3d ;
      if(bTest == FALSE)
        { SM_ASSERT(bTest) ; }

      // xsect point is on both cicle circumferences to within tolerance
      double dCir1Dist = (aPoints[ii] - rCir1Center).Length() ;
      double dCir2Dist = (aPoints[ii] - rCir2Center).Length() ;
      bTest =    SM_ARE_SAME_TO_TOL(dCir1Dist, dCir1Radius, dTol3d) 
              && SM_ARE_SAME_TO_TOL(dCir2Dist, dCir2Radius, dTol3d) ;
      if(bTest == FALSE)
        { SM_ASSERT(bTest) ; }

    } // end checking every intersection
#endif // SM_DEBUG_CODE

  // all done
  return(SM_SUCCESS) ;

} // end smgu_SphereCircleSphereCircleIntersect

/*******************************************************************//**
PURPOSE: Intersect two planes.

RETURNS --- 
  SM_ERR_INVALID_INPUT = non unit normal vectors
  SM_ERR               = parallel planes
  SM_SUCCESS           = intersecting planes 
***********************************************************************/
SM_EXPORT SmStatus smgu_IntersectTwoPlanes
(
  const SmPoint3d  & crP1,              // in : point on plane 1
  const SmVector3d & crV1,              // in : unitNormal for plane 1
  const SmPoint3d  & crP2,              // in : point on plane 2       
  const SmVector3d & crV2,              // in : unitNormal for plane 2 
  SmPoint3d        & rLinePnt,          // out: point on intersection line (currently: halfway between projections
                                        //      of crP1 and crP2 onto intersection line.)
  SmVector3d       & rLineVec,          // out: tangent of intersection line
  SmBoolean        * bOptCoincident     // out: TRUE = parallel and coincident planes NULL to ignore, default:[NULL] 
)
{
  // init output
  if(bOptCoincident) { *bOptCoincident = FALSE ; }

  // Check for unitized normal vectors
  SM_ASSERT(   SM_IS_ZERO(crV1.Dot(crV1)-1.0)
            && SM_IS_ZERO(crV2.Dot(crV2)-1.0)) ;
  if(   smos_Fabs(crV1.Dot(crV1)-1.0) > SM_EFF_ZERO
     || smos_Fabs(crV2.Dot(crV2)-1.0) > SM_EFF_ZERO) 
    { SER(SM_ERR_INVALID_INPUT); }

  // define a 3rd plane orthogonal to two input planes 
  SmVector3d sPlane3Pnt = (crP1 + crP2) / 2.0;  // average points
  SmVector3d sPlane3Norm = crV1 * crV2;

  // no intersection when planes are parallel
  if (sPlane3Norm.LengthSquared() < SM_EFF_ZERO_SQ) 
    {
      // when asked - check for coincidence
      if(bOptCoincident) 
        { double dScaledZero = SM_EFF_ZERO * (1.0 + smos_Max(crP1.GetMaxDimension(),
                                                             crP2.GetMaxDimension())) ;
          double dDist1 = (crP2 - crP1).Dot(crV1) ;
          double dDist2 = (crP1 - crP2).Dot(crV2) ;
          *bOptCoincident = (   dDist1 < dScaledZero
                             && dDist2 < dScaledZero) ;
        }
      return SM_ERR;
    }
  SER(sPlane3Norm.Unitize());

  // intersect 3 planes
  SmPoint3d sLinePnt;
  SER(smgu_IntersectThreePlanes(crP1,crV1,crP2,crV2,sPlane3Pnt,sPlane3Norm,sLinePnt));

  // all done
  rLinePnt = sLinePnt;
  rLineVec = sPlane3Norm;
  return SM_SUCCESS;

} // end smgu_IntersectTwoPlanes

/*******************************************************************//**
PURPOSE: Intersect three planes.

NOTES: Note -- we assume unit normal vectors for the plane
***********************************************************************/
SM_EXPORT SmStatus smgu_IntersectThreePlanes
(
  const SmPoint3d  & crP1,       // in : point on plane 1          
  const SmVector3d & crV1,       // in : UnitNormal for Plane 1    
  const SmPoint3d  & crP2,       // in : point on plane 2          
  const SmVector3d & crV2,       // in : UnitNormal for Plane 2    
  const SmPoint3d  & crP3,       // in : point on plane 3          
  const SmVector3d & crV3,       // in : UnitNormal for Plane 3    
  SmPoint3d        & rResult     // out: 3 plane intersection point
) 
{                               
   // First make sure vectors are unitized
   if(   smos_Fabs(crV1.Dot(crV1)-1.0) > SM_EFF_ZERO
      || smos_Fabs(crV2.Dot(crV2)-1.0) > SM_EFF_ZERO
      || smos_Fabs(crV3.Dot(crV3)-1.0) > SM_EFF_ZERO) 
     { SER(SM_ERR_INVALID_INPUT) ; } 

   double dDet = smgu_Determinant3Vectors(crV1,crV2,crV3);

   // Planes do not intersect - must be parallel
   if (smos_Fabs(dDet) < SM_EFF_ZERO) 
     { return SM_ERR_INVALID_INPUT ; }

   SmVector3d sWork =   (crP1.Dot(crV1) * (crV2 * crV3))
                      + (crP2.Dot(crV2) * (crV3 * crV1))
                      + (crP3.Dot(crV3) * (crV1 * crV2));
   rResult = sWork / dDet;

#ifdef SM_DEBUG_CODE
  double dErr1 = (rResult - crP1).Dot(crV1) ;
  double dErr2 = (rResult - crP2).Dot(crV2) ;
  double dErr3 = (rResult - crP3).Dot(crV3) ;
  double dTol  = SM_EFF_ZERO * ( 1.0 + crP1.GetMaxDimension()
                                     + crP2.GetMaxDimension()
                                     + crP3.GetMaxDimension()
                                     + rResult.GetMaxDimension()) ;            
  SM_ASSERT_MSG(   SM_IS_ZERO_TO_TOL(dErr1, dTol)
                && SM_IS_ZERO_TO_TOL(dErr2, dTol)
                && SM_IS_ZERO_TO_TOL(dErr3, dTol),
                _T("smgu_IntersectThreePlanes: found an out of tolerance result")) ;

#endif // SM_DEBUG_CODE
   // all done
   return SM_SUCCESS;

} // end smgu_IntersectThreePlanes

/*******************************************************************//**
PURPOSE: Estimate the max gap between two circles in 3 space. It is assumed
  this will be called on circles which are close to being coincident.

NOTES:

Circle Gaps:  The gap from any point on one circle to the 2nd circle is the
  vector connecting the 1st circle point to the nearest point on the
  the 2nd circle.  That vector will be perpendicular to the tangent
  on the 2nd circle but won't necessarily be perpendicular to the tangent
  on circle 1. This vector is 

    Gap(theta) = Circ1(theta) - Center2 - |Normal2 x (Circ1(theta)-Center2) x Normal2| * Radius2
    
    where theta        = angular paramerization of Circle 1, 
          Circ1(theta) = a point on Circle1, 
                       = Center1 + Radius1 * [ cos(theta) * X ]
                                             [ sin(theta) * Y ] 
          X vector     = vector perp to Normal1
          Y vector     = Normal1 x X

Max Gap:  The max gap is the largest gap vector between the two circles, conceptually
  found by finding all the gap vectors on circle 1 and picking
  the largest one.  This is a maxi-min problem in two dimensions where the two
  parameters are the parameterizations of the two circular curves.

  The max-gap vector will be perpendicular to both circle tangents.

  The max happens at d Gap(theta)/d theta = 0

  I don't have a solution for that.  The approximation in this function
  is that the gap functiom is sampled and the best value is returned.
***********************************************************************/
SM_EXPORT SmStatus smgu_CircleCircleApproxMaxGap
(
  SmPoint3d  & rCenter1,                 // in : center of circle 1
  SmVector3d & rNormal1,                 // in : normal vector to circle 1 plane (must be nonZero)
  double       dRadius1,                 // in : circle 1 radius
  SmPoint3d  & rCenter2,                 // in : center of circle 2
  SmVector3d & rNormal2,                 // in : normal vector to circle 2 plane (must be nonZero)
  double       dRadius2,                 // in : circle 2 radius
  double     & rdMaxGap,                 // out: Max Gap approximate length
  SmPoint3d  * pPoint1,                  // out: optional max gap point end on circle 1, NULL to ignore
  SmPoint3d  * pPoint2                   // out: optional max gap point end on circle 1, NULL to ignore
)                  
{
  // init output values
  rdMaxGap    = SM_UNDEF_DOUBLE ;
  if(pPoint1) pPoint1->SetUninitialized() ;
  if(pPoint2) pPoint2->SetUninitialized() ;

  // check input: consistent optional arguments
  if(   (pPoint1 == NULL && pPoint2 != NULL)
     || (pPoint1 != NULL && pPoint2 == NULL))
    {
      return(SM_ERR_INVALID_INPUT) ;
    }

  // normalize circle normals
  SmVector3d sNormal1 = rNormal1 ;
  SmVector3d sNormal2 = rNormal2 ;
  double     dNormal1 = rNormal1.Length() ;
  double     dNormal2 = rNormal2.Length() ;

  // check input: nonZero normals
  if(   dNormal1 < SM_EFF_ZERO
     || dNormal2 < SM_EFF_ZERO)
    {
      return(SM_ERR_INVALID_INPUT) ;
    }

  sNormal1 /= dNormal1 ;
  sNormal2 /= dNormal2 ;

  // locals
  ULONG ii, iCnt    = 45 ;
  double dAngRad    = 0.0 ; 
  double dDelAngRad = SM_PI * 2.0 / (double)(iCnt) ;
  double dMaxGap    = 0 ;
  double dV ;

  // parameterize circle 1
  SmPoint3d  sCirc1, sCirc2 ;
  SmVector3d sX1, sY1, sZ1, sV ;
  SmVector3d sX2, sY2, sZ2 ;
  rNormal1.MakeUnitOrthoVectors(NULL, sZ1, sX1, sY1) ;

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
SmBoolean bShowAll = TRUE ;
#endif // SM_DEBUG_CODE

  // for several gap values
  for(ii=0;ii<iCnt;ii++,dAngRad+=dDelAngRad)
    {
      // Point on circle1
      sCirc1 =   (dRadius1 * smos_Cosine(dAngRad)) * sX1
               + (dRadius1 * smos_Sine(dAngRad))   * sY1 ;

      // get radial vector needed to compute point on circle2
      sV  = rNormal2 * (sCirc1 - rCenter2) * rNormal2 ;
      dV = sV.Length() ;

      // degenerate case: circumference1 intersects Center2
      if(dV < SM_EFF_ZERO) { // any radial unit vector will do (they are all good)
                             rNormal2.MakeUnitOrthoVectors(NULL, sZ2, sV, sY2) ; 
                           } 
      else                 { sV /= dV ;
                           }
      sV.Unitize() ;

      // point on circle 2
      sCirc2 = rCenter2 + sV * dRadius2 ;

      // gap
      SmVector3d  sGap = sCirc1 - sCirc2 ;
      double      dGap = sGap.Length() ;

#ifdef SM_DEBUG_CODE
      // draw circles, sample points and gap, and max points and gap
      if(bDebugMe)
        {          
          rNormal2.MakeUnitOrthoVectors(NULL, sZ2, sX2, sY2) ;
          SmCircle sCircle1( rCenter1, sX1, sY1, SmExtent1d(0.0, 360.0), dRadius1) ;
          SmCircle sCircle2( rCenter2, sX2, sY2, SmExtent1d(0.0, 360.0), dRadius2) ;

          if(ii == 0 || bShowAll == FALSE) smgfx_Erase() ;
          smgfx_SetLook(1,2, 0,0,1) ; sCircle1.Draw() ; sm_GraphicsLoop() ;
          smgfx_SetLook(1,2, 0,1,0) ; sCircle1.Draw() ; sm_GraphicsLoop() ;
          smgfx_SetLook(2,4, 0,0,1) ; sCirc1.DrawPointToPoint(rCenter1) ; sm_GraphicsLoop() ;
          smgfx_SetLook(2,4, 0,1,0) ; sCirc2.DrawPointToPoint(rCenter2) ; sm_GraphicsLoop() ;
          smgfx_SetLook(2,4, 0,0,0) ; sCirc2.DrawPointToPoint(sCirc1)   ; sm_GraphicsLoop() ;
          if(!bShowAll)
            { smgfx_SetLook(2,3, 0,1,1) ; if(pPoint1 && pPoint2) pPoint1->DrawPointToPoint(*pPoint2) ; sm_GraphicsLoop() ; }
          sm_GraphicsLoop() ;
        }
#else
      SM_REF1(rCenter1);
#endif // SM_DEBUG_CODE

      // save max gap
      if(dGap > dMaxGap)
        {
          dMaxGap    = dGap ;
          if(pPoint1) { *pPoint1 = sCirc1 ; }
          if(pPoint2) { *pPoint2 = sCirc2 ; }
        } // end if max gap seen  

    } // end iter every sample point

#ifdef SM_DEBUG_CODE
      // draw circles, sample points and gap, and max points and gap
      if(bDebugMe)
        {          
          if(bShowAll)
            {
              smgfx_SetLook(2,3, 0,1,1) ; if(pPoint1 && pPoint2) pPoint1->DrawPointToPoint(*pPoint2) ; sm_GraphicsLoop() ;
            }
          sm_GraphicsLoop() ;
        }
#endif // SM_DEBUG_CODE

  // all done
  return(SM_SUCCESS) ;

} // end smgu_CircleCircleApproxMaxGap

// obsolete try
//      /*******************************************************************//**
//      PURPOSE: Estimate the max gap between two circles in 3 space. It is assumed
//        this will be called on circles which are close to being coincident.
//      
//      NOTES: I could not think of a closed form solution to this problem.
//        However, for circles which are nearly coincident, this solution is good
//        to 1st order.  So one could use this evaluation as a good guess
//        point to start a Newton-Raphson iteration to find the actual max gap 
//        between two circles.
//      
//      Max Gap:  The gap from any point on one circle to the 2nd circle is the
//        vector connecting the 1st circle point to the nearest point on the
//        the 2nd circle.  That vector will be perpendicular to the tangent
//        on the 2nd circle but won't necessarily be perpendicular to the tangent
//        on circle 1.  
//      
//        The max gap is the largest gap vector between the two circles, conceptually
//        found by finding all the gap vectors on circle 1 and picking
//        the largest one.  This is a maxi-min problem in two dimensions where the two
//        parameters are the parameterizations of the two circular curves.
//      
//        The max-gap vector will be perpendicular to both circle tangents.
//        
//      IMPLEMENTATION --- This function assumes the gap is composed of three components
//        1. a GapZ vector which is the component of the distance vector between circle  
//           centers in the Normal1 direction.
//               GapZ = (Center2 - Center1) - (Normal1 x (Center2 - Center1) x Normal1).
//      
//        2. a GapA vector which is the gap vector from circle1 to circle2 due only
//            to the distance between circle centers in the plane of circle 1 and the
//            difference in radii between circles 1 and 2.  GapA vectors are always
//            aligned with the radial vectors of circle2 and are perpendicular to 
//            tangents on circle2.  The GapA vector is the total gap vector when the two 
//            circles are coplanar. The GapA vector can be written as a function 
//            which varies for every point on the circumference of circle2.
//      
//            GapA(alpha) = (Radius2 - rs(alpha)) * A(alpha) 
//               where: rs(alpha)      = distance from circle2 center to circle1 circumference.
//                      alpha          = circle2 angle parameter of , (0 <= alpha <= 2PI radians)
//                      A vector       = [cos(alpha) X, sin(alpha) Y], the circle 2 radial vector
//                      circle2(alpha) = Center2 + Radius2 * A,
//                      X vector       = the direction in the circle1 plane
//                                        of the offset between circle centers,
//                      Y vector       = perpendicular to X in the circle1 plane,
//      
//              rs = - dx*cos(alpha) +/- sqrt(-dx*dx*sin(alpha)*sin(alpha) + Radius1*Radius1)
//      
//               where: dx             = ||(Normal1 x (Center2 - Center1) x Normal1)||
//                                       the distance between circle centers
//                                       in the circle 1 planar direction.
//      
//        For circles with parallel orientations, the gap vectors are 
//      
//               Gap_parallel_orientations(alpha) = GapA(alpha) + GapZ
//      
//      
//        3. A VecBn displacement vector due to any angle between the orientations 
//            of circles 1 and 2. This vector is defined by the movement of 
//            points on the circumference of circle2 when it's rotated
//            from the circle1 to the circle2 orientation. This vector varies
//            for every point on the circumference of circle2. In general VecBn vectors 
//            are not gap vectors, however the max of the VecBn(alpha)
//            function is the max gap vector when two circles of the same radius 
//            share a common center and have different orientations. For small
//            orientation angles, the VecBn(alpha) function is close to Gap(alpha)
//            function at all points for a pair of circles who share common radii
//            and centers.
//      
//            VecBn(alpha) =   Radius2*cos(alpha-theta0)*sin(phi)*Z 
//                           - Radius2*cos(alpha-theta0)*(1-cos(phi)*T
//                  where: Z vector = cross(X,Y), unit vector in Normal1 direction
//                         T vector = cos(theta0)*X + sin(theta0)*Y, 
//                                       unit vector in direction of orientation change projected to circle0 plane.
//                         theta0   = Normal1.AngleBetween(X,Normal1 x (Normal2 - Normal1) x Normal1)),
//                                       angle of orientation change in the circle0 plane
//                                          measured CCW from the X direction
//      
//            The VecBn vector has a component in the circle1 normal
//            direction, VecBn_z, and a component in the circle1 plane VecBn_a.  
//            For small orientation angles, VecBn_z is much larger than VecBn_a.
//      
//        In this function, the Gap vector is approxmated by
//         
//            Gap(alpha) = GapA(alpha) + GapZ + VecBn(alpha)
//      
//        The difference between this gap vector and the actual gap vector is due solely 
//        to the non radial component of the already small aVecBn_a vector.
//      
//        The max gap:
//        
//             Gap(alpha) =   (Radius2 + dx*cos(alpha) - sqrt(-dx*dx*sin(alpha)*sin(alpha) + Radius1*Radius1)) * A(alpha)
//                          + (Center2 - Center1) - (Normal1 x (Center2 - Center1) x Normal1)
//                          + 
//      
//         returned is the max of this relation and the error of this estimate
//        is of the size of the non radial component of the VecBn_a vector.
//      
//        An estimate of the error size and the pair of points found for the max
//        approximate gap are returned so that this computation can be used
//        as the input to an iterative technique to refine this approximation 
//        to the max gap when the two circle normal vectors are not parallel.
//      
//        In some cases, there are multiple max gap vectors of equal size.
//        In those cases, one max gap vector is reported at random in the
//        pPoint1 and pPoint2 outputs.
//      
//      ***********************************************************************/
//      SmStatus smgu_CircleCircleApproxMaxGap
//        (SmPoint3d  &rCenter1,                 // in : center of circle 1
//         SmVector3d &rNormal1,                 // in : normal vector to circle 1 plane (must be nonZero)
//         double      dRadius1,                 // in : circle 1 radius
//         SmPoint3d  &rCenter2,                 // in : center of circle 2
//         SmVector3d &rNormal2,                 // in : normal vector to circle 2 plane (must be nonZero)
//         double      dRadius2,                 // in : circle 2 radius
//         double     &rdMaxGap,                 // out: Max Gap approximate length
//         double     &rdMaxGapErr,              // out: estimate of the max gap error
//         SmPoint3d  *pPoint1,                  // out: optional max gap point end on circle 1, NULL to ignore
//         SmPoint3d  *pPoint2)                  // out: optional max gap point end on circle 1, NULL to ignore
//      {
//        // init output to uninit values
//        rdMaxGap    = SM_UNDEF_DOUBLE ;
//        rdMaxGapErr = SM_UNDEF_DOUBLE ;
//        if(pPoint1) pPoint1->SetUninitialized() ;
//        if(pPoint2) pPoint2->SetUninitialized() ;
//      
//        // check input: consistent optional arguments
//        if(   (pPoint1 == NULL && pPoint2 != NULL
//           || (pPoint1 != NULL && pPoint2 == NULL)
//          {
//            return(SM_ERR_INVALID_INPUT) ;
//          }
//      
//        // normalize circle normals
//        SmVector3d sNormal1 = rNormal1 ;
//        SmVector3d sNormal2 = rNormal2 ;
//        double     dNormal1 = rNormal1.Length() ;
//        double     dNormal2 = rNormal2.Length() ;
//      
//        // check input: nonZero normals
//        if(   dNormal1 < SM_EFF_ZERO
//           || dNormal2 < SM_EFF_ZERO)
//          {
//            return(SM_ERR_INVALID_INPUT) ;
//          }
//      
//        sNormal1 /= dNormal1 ;
//        sNormal2 /= dNormal2 ;
//      
//        // angle and direction between normal vectors
//        SmVector3d sNormalGap    = sNormal2 - sNormal1 ;
//        SmVector3d sNormalDir    = sNormalGap - sNormal1.Dot(sNormalGap) * sNormal1 ;
//        double     dNormalDir    = sNormalDir.Length() ;
//      
//        // center/center gap
//        SmVector3d sCenterGap    = rCenter2 - rCenter1 ;
//        double     dCenterGap    = sCenterGap.Length() ;
//      
//        // Center gap decomposed into circle 1 planar and normal components
//        SmVector3d sGapZ   = sNormal1.Dot(sCenterGap) * sNormal1 ;
//        SmVector3d sGapDir = sCenterGap - sGapZ ;
//        double     dGapZ   = sGapZ.Length() ;
//        double     dGapDir = sGapDir.Length() ;
//      
//        // concentric circles    
//        if(   dGapDir    < SM_EFF_ZERO
//           && dNormalDir < SM_EFF_ZERO)   // GWC does this check need to be scaled to radius values?
//          {
//            // max gap = radius size difference + GapZ
//            rdMaxGap    = smos_Sqrt((dRadius1 - dRadius2)*(dRadius1 - dRadius2) + dGapZ*dGapZ);
//            rdMaxGapErr = ???? ;
//      
//            if(pPoint1 && pPoint2)
//              {
//      
//              } // end optional output point check
//      
//            // all done
//            return(SM_SUCCESS) ;
//          }
//      
//        // center aligned circles
//        else if(dGapdir < SM_EFF_ZERO)
//          {
//            // gap in direction of normal rotation
//            SmVector3d sX1 = sGapDir / dGapDir ;
//            SmVector3d sX2 = sNormal2 * sX1 * sNormal2 ;
//            sX2.Unitize() ;
//            SmVector3d sMaxGap = (rCenter2 + sX2 * dRadius2) - (rCenter1 + sX1 * dRadius1) ;
//            rdMaxGap = sMaxGap.Length() ;
//      
//            // all done
//            return(SM_SUCCESS) ;
//          }
//        
//        // coplanar
//        else if(dNormalDir < SM_EFF_ZERO)
//          { 
//            
//      
//      
//        // generally oriented
//        else
//      
//        // zero length sGapDir (aligned or coincident circle centers)
//        if(dGapDir < SM_EFF_ZERO)
//          {
//            // angle of max gap = rotation max gap
//      
//        // coincident when max gap between large circles is less than tol and
//        // 
//      
//        long Gwc_smgu_CircleCircleApproxMaxGapNeedsImplementation ;
//      
//        // all done
//        return(SM_SUCCESS) ;
//      
//      } // end smgu_CircleCircleApproxMaxGap

/*******************************************************************//**
PURPOSE: Create a plane definition from an array of points
              by projecting points along a given normal vector.  

NOTES: Creates an SmAxis2Placement and SmExtent2d (user can create plane from those).
***********************************************************************/
SM_EXPORT SmStatus smgu_PlaneFromPoints
(
  const SmPoint3d  * sPoints,            // in : the point set to examine
  ULONG              lNumPoints,         // in : number of points in sPoints
  const SmVector3d & rNormal,            // in : given normal
  SmAxis2Placement & rAPlacement,        // out: X and Y axes of the point
  SmExtent2d       & rExt2d,             // out: Extent of plane that encompases all projected points
  double           & rdMaxGap            // out: max displacement from the plane to the sPoints
)           
{
    if (lNumPoints < 3) {
        SER(SM_ERR);
    }

    SmPoint3d    sOrigin(0.0,0.0,0.0);
    SmVector3d sVec, sXAxis(0.0,0.0,0.0);

    SmBoolean bFirst = TRUE;
    SmPoint3d sPLast = sPoints[lNumPoints-1];
    for (ULONG jj=0; jj<lNumPoints; jj++) {
        SmPoint3d sPCurrent(sPoints[jj]);
        sVec = sPLast - sPCurrent;
// ---  Add to the origin to get an average origin 
        sOrigin = sOrigin + sPCurrent;
        if (bFirst && (sVec.LengthSquared() > SM_EFF_ZERO_SQ)) {
            sXAxis = sVec;
            bFirst = FALSE;
        }
    }
    
// ---    Average the Origin from all of the points
    sOrigin.x = sOrigin.x / lNumPoints;
    sOrigin.y = sOrigin.y / lNumPoints;
    sOrigin.z = sOrigin.z / lNumPoints;

    ZERO_VEC_ER(sXAxis);

    rAPlacement.SetSTEPCanonical(sOrigin, rNormal, sXAxis);


// --- Add up the extent points  (and get Max gap as well)
    for(ULONG ii=0; ii < lNumPoints; ii++) {
        SmPoint2d sPoint = smgu_PlaneDropPoint(rAPlacement, sPoints[ii]);
        SmPoint3d sPnt1 = smgu_PlaneEvaluatePoint(rAPlacement, sPoint);
        double dDist = sPnt1.DistanceBetween(sPoints[ii]);
        if (dDist > SM_EFF_ZERO) {
            rdMaxGap = smos_Max(rdMaxGap, dDist);
        }
        rExt2d.AddPoint2d(sPoint);
    }

    return SM_SUCCESS;

} // end smgu_PlaneFromPoints

/*******************************************************************//**
PURPOSE: Evaluate a point on a plane defined by a reference frame.

NOTES: 
***********************************************************************/
SM_EXPORT SmPoint3d smgu_PlaneEvaluatePoint
(
  const SmAxis2Placement & rPlane,      // in :
  const SmPoint2d & crUV                // in :
)
{
    SmPoint3d sPoint =   rPlane.GetOriginRef() 
                       + crUV.x * rPlane.GetXAxisRef() 
                       + crUV.y * rPlane.GetYAxisRef();
    return sPoint;

} // end smgu_PlaneEvaluatePoint

/*******************************************************************//**
PURPOSE: Drop a 3D point onto a plane defined by a reference frame

NOTES: 
***********************************************************************/
SM_EXPORT SmPoint2d smgu_PlaneDropPoint
(
  const SmAxis2Placement & rPlane,    // in :
  const SmPoint3d        & cr3DPoint  // in :
)
{
    SmVector3d sVec = cr3DPoint - rPlane.GetOriginRef();
    SmPoint2d sRet;
    sRet.x = rPlane.GetXAxisRef().Dot(sVec);
    sRet.y = rPlane.GetYAxisRef().Dot(sVec);
    return sRet;

} // end smgu_PlaneDropPoint

/*******************************************************************//**
PURPOSE: Determine the distance from the point to the plane.

NOTES: Assumes that the plane normal is unitized.
***********************************************************************/
SM_EXPORT SmStatus smgu_PlanePointDistance
(
  const SmPoint3d  & crPlanePnt,       // in : PlanePt         of Plane(PlanePt, PlaneUnitNormal)
  const SmVector3d & crPlaneNorm,      // in : PlaneUnitNormal of Plane(PlanePt, PlaneUnitNormal)
  const SmPoint3d  & crTestPoint,      // in : TgtPoint to measure
  double           & rdDistanceToPlane // out: Distance from TgtPoint to Plane
)
{
    if (crPlaneNorm.LengthSquared() - 1.0 > SM_EFF_ZERO) 
      { SER(SM_ERR); }

    SmVector3d sProjVec = crTestPoint - crPlanePnt;
    double     dDot     = sProjVec.Dot(crPlaneNorm);
    rdDistanceToPlane   = smos_Fabs(dDot);
    return SM_SUCCESS;

} // end smgu_PlanePointDistance

/*******************************************************************//**
PURPOSE: Given a point and a plane find the point on the plane which
   corresponds to the normal projection of the point onto the plane.

NOTES: crPlaneNorm does *not* have to be unit.
***********************************************************************/
SM_EXPORT SmStatus smgu_PointProjectToPlane
(
  const SmPoint3d  & crPoint,          // in : target point
  const SmPoint3d  & crPlanePnt,       // in : point on plane
  const SmVector3d & crPlaneNorm,      // in : plane surface normal
  SmPoint3d        & rProjectedPoint   // out: target point projected to plane
)  
{
  // a tad faster
  double dK =   (  (crPoint.x - crPlanePnt.x) * crPlaneNorm.x
                 + (crPoint.y - crPlanePnt.y) * crPlaneNorm.y
                 + (crPoint.z - crPlanePnt.z) * crPlaneNorm.z)
              / crPlaneNorm.Dot(crPlaneNorm) ;

  // equivalent slower way
  //      double dD = -crPlanePnt.Dot(crPlaneNorm);
  //      double dK = (dD + crPoint.Dot(crPlaneNorm)) / crPlaneNorm.Dot(crPlaneNorm);
  
  rProjectedPoint = crPoint - dK * crPlaneNorm;
  return SM_SUCCESS;

} // end smgu_PointProjectToPlane

/*******************************************************************//**
PURPOSE: Given a vector and a plane find the vector on the plane which
   corresponds to the normal projection of the vector onto the plane.

NOTES: Assumes that the plane normal is unitized.
***********************************************************************/
SM_EXPORT SmStatus smgu_VectorProjectToPlane
(
  const SmVector3d & crVectorToProject,  // in : target vector
  const SmVector3d & crPlaneNorm,        // in : plane normal
  SmVector3d       & rProjectedVector    // out: component of target vector not in plane normal direction
)   
{
    // subtract component in the plane normal directon
    rProjectedVector =    crVectorToProject - (crVectorToProject.Dot(crPlaneNorm)) * crPlaneNorm;
    return SM_SUCCESS;

} // end smgu_VectorProjectToPlane

/*******************************************************************//**
PURPOSE: Rotate the given point about the Z axis until it is
         on the Z, X plane.  

NOTES:  return 2 solutions when point is on seam
        return 2 solutions when point is on ZAxis - up to caller to check rdDistToZAxis when needed
***********************************************************************/
SmStatus smgu_TransformPointToStartPlane
(
  const SmPoint3d        & crPointToTransform, // in : target point
  const SmAxis2Placement & rPosition,          // in : rotate about Origin and ZAxis, measure angles from XAxis
  const SmExtent1d       & rSweepIvlDeg,       // in : Interval of supported points (commonly [0 360], think SmEllipse and SmSurfOfRevolution AnalDomains)
  double                   dDistTol3d,         // in : Dist3d when points are close enough to seams to return 2 answers
  SmPoint3d              & rTransformedPoint,  // out: output point on the X/Z plane       
  double                 & rdDistToZAxis,      // out: distance to Z axis       
  ULONG                  & rlNumAngles,        // out: 0 - point is on axis
                                               //      1 - point is not on seam 
                                               //      2 - point is on seam of curve of revolution
  double                   adAnglesDeg[2],     // out: Angles in degrees to rotate rTransformedPoint back to original position
                                               //      range:[m_vAnalDomain] or positive(0 to 360.0)
  SmBoolean              & bInside,            // out: TRUE = point is inside trim domain
                                               //      FALSE= point is outside trim domain
  SmBoolean                bSnapToSeams        // in : TRUE = rtn 2 snapped values at seams, FALSE = rtn 1 exact and 1 snapped val at seams
                                               //      TRUE=previous behavior, default:[TRUE] 
)
{
  // locals
  const SmPoint3d  &rCenter = rPosition.GetOriginRef();
  SmVector3d        sZAxis  = rPosition.GetZAxis();

  // get distance from target point to Z_Axis
  SER(smgu_LinePointDistance(rPosition.GetOriginRef(),sZAxis,crPointToTransform,rdDistToZAxis));

  // low work - point is on the axis to within tolerance - e.g. an ambiguous center point
  double dScaledZero = SM_EFF_ZERO * (1.0 + crPointToTransform.GetMaxDimension());

  // When Point is on the axis of revolution - no unique solutions
  if (rdDistToZAxis < dScaledZero) 
    { 
      // return Min and Max Angles as answers
      adAnglesDeg[0]    = rSweepIvlDeg.GetMin();
      adAnglesDeg[1]    = rSweepIvlDeg.GetMax();
      rTransformedPoint = crPointToTransform;
      rlNumAngles       = 2;
      bInside           = TRUE ;
      return SM_SUCCESS;
    } // end point on Z_Axis check

  // locals
  const SmVector3d &rXAxis     = rPosition.GetXAxisRef();
  SmVector3d        sVecToPnt  = crPointToTransform - rCenter;
  double            dAngTolDeg = ANALYTIC_TOL_SCALE * SM_EFF_ZERO * (1.0 + 360.0)  ; // value orig used in SmSurfofRevolution::TransformPointToStartPlane
  //double            dMaxGapDeg = 0.05 ;                                              // value orig used in SmEllipse::TransformPointToStartPlane
  
  // get angle in degrees about Z_Axis from X_Axis to Target Point in degrees
  double dAngleRad;
  SER(sZAxis.CCWAngleBetween(rXAxis,sVecToPnt,dAngleRad));
  //double dAngleRawDeg = SM_RAD2DEG(dAngleRad) ;

  // get rid of numeric noise at zero
  if (dAngleRad < SM_EFF_ZERO) 
    { 
      if ( dAngleRad > -SM_EFF_ZERO )
          dAngleRad = 0; // just noise [Fillet regression tests 407 408 413 416]
      else
          dAngleRad += 2.0*SM_PI;
    }
  double dAngleDeg = SM_RAD2DEG(dAngleRad) ;

  // map periodic values into interval
  bInside = rSweepIvlDeg.ContainsPeriodicValue(dAngleDeg, 360.0, &dAngleDeg) ;

  // watchout for periodicity
  if(   !bInside
     && dAngleDeg < 0.0)
    { dAngleDeg += 360.0 ; }

  // when SweepIvl is closed

  // Get an angle tolerance from 3d values.  [B293 B351]
  double dBdryTolDeg = SM_RAD2DEG( dDistTol3d / rdDistToZAxis );

  // when Ellipse is closed and PointToTransform is close enough to the seam
  if(   SM_ARE_SAME_TO_TOL( rSweepIvlDeg.GetLength(), 360.0, dAngTolDeg )        // if interval is closed (has seams)
     && rSweepIvlDeg.IsValueOnPeriodicBoundary( dAngleDeg, 360.0, dBdryTolDeg )) // and dAngleDeg is close enough
    {
      // generate two solutions - one on either side of the seam
      rlNumAngles = 2 ;
      if(bSnapToSeams) { // put both solutions exactly on the seam
                         adAnglesDeg[0] = rSweepIvlDeg.GetMin() ;
                         adAnglesDeg[1] = rSweepIvlDeg.GetMax() ;
                       } // end return two snapped solutions branch
      else             { // else return one exact solutions and the far side snap to seam value
                         if(smos_Fabs(rSweepIvlDeg.GetMin()-dAngleDeg) < smos_Fabs(rSweepIvlDeg.GetMax()-dAngleDeg))
                           { adAnglesDeg[0] = dAngleDeg ;
                             adAnglesDeg[1] = rSweepIvlDeg.GetMax() ;
                           }
                         else
                           { adAnglesDeg[0] = rSweepIvlDeg.GetMin() ;
                             adAnglesDeg[1] = dAngleDeg ;
                           }
                       } // end return one exact solution branch
    }
  else // use the lone value
    {
      // only report the angle inside the sweep interval
      rlNumAngles = 1 ;
      adAnglesDeg[0] = dAngleDeg ;
    }

  // gwc: old code in conflict with SmSurfOfRevolution
  // double dTotalAngle = GetEndAngleDeg() - GetStartAngleDeg();
  // if(smos_Fabs(dTotalAngle-360.0) < dMaxGapDeg)
  //   {
  //     // when angle is on seam, add periodic seam value
  //     if(smos_Fabs(dAngleDeg - GetStartAngleDeg()) < dMaxGapDeg)
  //       { rlNumAngles = 2 ;
  //         adAnglesDeg[0] = dAngleDeg ;
  //         adAnglesDeg[1] = dAngleDeg + 360.0 ;
  //       }
  // 
  //     // when angle is on seam, add periodic seam value
  //     if(smos_Fabs(dAngleDeg - GetEndAngleDeg()) < dMaxGapDeg)
  //       { rlNumAngles = 2 ;
  //         adAnglesDeg[0] = dAngleDeg ;
  //         adAnglesDeg[1] = dAngleDeg - 360.0 ;
  //       }
  //    }

  // Now rotate the point about the Z axis to the Z, X plane
  // compute without using angles to avoid tolerance errors
  double dDotZ = sZAxis.Dot(sVecToPnt);
  SmVector3d sXYProj;
  SER(smgu_VectorProjectToPlane(sVecToPnt,sZAxis,sXYProj));
  double dXSize = sXYProj.Length();
  rTransformedPoint = rCenter + sZAxis*dDotZ + rXAxis*dXSize;

  // all done
  return SM_SUCCESS;

} // end smgu_TransformPointToStartPlane

/*******************************************************************//**
PURPOSE: Given a View plane and a eye point find the perspective point which
   corresponds to the perspective projection of a given point onto the view plane.

NOTES: crPlaneNorm has to be unit.
***********************************************************************/
SM_EXPORT SmStatus smgu_PointPerspectiveToPlane
(
  const SmPoint3d  & crPoint,          // in : target point
  const SmPoint3d  & crPlanePoint,     // in : point on plane
  const SmVector3d & crPlaneUnitNorm,  // in : plane surface unit-normal
  const SmVector3d & crEyePoint,       // in : eye position
  SmPoint3d        & rProjectedPoint   // out: target point projected to plane
)  
{
  // pass call along
  rProjectedPoint = crPoint.PerspectiveProjectPointToPlane(crPlanePoint,
                                                           crPlaneUnitNorm,
                                                           crEyePoint) ;
  
  return SM_SUCCESS;

} // end smgu_PointPerspectiveToPlane

/*******************************************************************//**
PURPOSE: Given a View plane and a eye point find the perspective vector which
   corresponds to the perspective projection of a given vector onto the view plane.

NOTES: crPlaneNorm has to be unit.
       vector projection depends on location - input requires a vector base point.
***********************************************************************/
SM_EXPORT SmStatus smgu_VectorPerspectiveToPlane
(
  const SmPoint3d  & crVectorBasePoint,  // in : target vector 3d base point
  const SmVector3d & crVectorToProject,  // in : target vector
  const SmPoint3d  & crPlanePoint,       // in : plane point
  const SmVector3d & crPlaneUnitNorm,    // in : plane surface unit-normal
  const SmVector3d & crEyePoint,         // in : eye position
  SmVector3d       & crProjectedVector   // out: component of target vector not in plane normal direction
)  
{
  // subtract component in the plane normal directon
  crProjectedVector =  crVectorToProject.PerspectiveProjectToPlane(crVectorBasePoint, 
                                                                   crPlanePoint, 
                                                                   crPlaneUnitNorm, 
                                                                   crEyePoint) ;
  return SM_SUCCESS;

} // end smgu_VectorPerspectiveToPlane

/*******************************************************************//**
PURPOSE: Express a given vector as a linear combination of
   two basis vectors: solve for u and v such that
   crVectorToProject = u * crBasisVec1  +  v * crBasisVec2

NOTES: If the given vector is not in the plane of the basis vectors,
   any component out of the plane will be ignored; the projection of the
   given vector on that plane is used.  When vector is perpendicular to the plane,
   returns rDUV = the zero vector.

   Returns SM_ERR only if the basis vectors do not form a basis (a unique plane).
***********************************************************************/
SM_EXPORT SmStatus smgu_VecLinCombTwoVectors
( 
  const SmVector3d & crBasisVec1,         // in : Vec1            of VectorToProject(uv) = u*Vec1 + v*Vec2
  const SmVector3d & crBasisVec2,         // in : Vec2            of VectorToProject(uv) = u*Vec1 + v*Vec2
  const SmVector3d & crVectorToProject,   // in : VectorToProject of VectorToProject(uv) = u*Vec1 + v*Vec2
        SmVector2d & rDUV                 // out: found uv        of VectorToProject(uv) = u*Vec1 + v*Vec2
)
{
  // Method: u * BasisVec1  +  v * BasisVec2  =  Vec
  // Dot this equation with BasisVec1, and with BasisVec2, to get two equations
  // for the two unknowns.

  // Init output.
  rDUV.Set( 0, 0 );

  double a00 = crBasisVec1.Dot( crBasisVec1 );
  double a01 = crBasisVec1.Dot( crBasisVec2 );
  double a10 = a01;
  double a11 = crBasisVec2.Dot( crBasisVec2 );

  double det = a00 * a11 - a10 * a01;

  double b0 = crBasisVec1.Dot( crVectorToProject );
  double b1 = crBasisVec2.Dot( crVectorToProject );

  double dNumer0 =  b0 * a11 -  b1 * a01;
  double dNumer1 = a00 *  b1 - a10 *  b0;

  if ( det == 0.0 ) { return SM_ERR; }  // In case dNumer0 or dNumer1 is 0 exactly.
  if ( smos_Fabs( det ) < smos_Fabs( dNumer0 ) * SM_EFF_ZERO ) { return SM_ERR; }
  if ( smos_Fabs( det ) < smos_Fabs( dNumer1 ) * SM_EFF_ZERO ) { return SM_ERR; }

  rDUV.x = dNumer0 / det;
  rDUV.y = dNumer1 / det;

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  if(bDebugMe)
    {
      SmVector3d sNorm       = crBasisVec1 * crBasisVec2 ;
      SmVector3d sTest       = rDUV.x * crBasisVec1 + rDUV.y * crBasisVec2 ;
      SmVector3d sGap        = sTest - crVectorToProject ;
      double     dGap        = sGap.Length() ;
      double     dScaledZero = SM_EFF_ZERO * (1.0 + smos_Max(crBasisVec1.GetMaxDimension(), crBasisVec2.GetMaxDimension())) ;

      SE_MSG((dGap < dScaledZero || sGap.IsParallelTo(sNorm, 1.0)) ? SM_SUCCESS : SM_ERR, 
            _T("Planar Vector Decomposition Failed")) ;
    }
#endif // SM_DEBUG_CODE

  // all done
  return SM_SUCCESS;

} // end smgu_VecLinCombTwoVectors

/*******************************************************************//**
PURPOSE: Find the geometric average of a set of points and
         the max gap from the center to any of the points in the set.  

NOTES: 
  1. returns SM_SUCCSS whenever eSetType is set.
  2. Check eSetType return value to see if PointSet is degenerate (Void - no points)
     and can't have a best fit point.
***********************************************************************/
SM_EXPORT SmStatus smgu_BestFitPointToPoints
(
  SmTArray<SmPoint3d> const & crPoints,   // in : target point set
  double                      dTol3d,     // in : Min distance between distinct 3d points
  SmPointSetType            & eSetType,   // out: one of: SM_PST_VOID,          // empty point set
                                          //              SM_PST_POINTSIZED,    // all pts within tol of BestCenterPt
                                          //              SM_PST_SCATTERED      // pts not within tol of target shape
  SmVector3d                & rCenterPt,  // out: rCenterPt = 1/n * Sum_i(crPoints[i])
  double                    & rdMaxGap    // out: max gap between crPoints[i] and rCenterPt
)   
{
  // init output
  eSetType = SM_PST_UNKNOWN ;
  rCenterPt.Set(0.0,0.0,0.0) ;
  rdMaxGap = 0.0 ;   

  // locals
  ULONG ii, cnt = crPoints.GetSize() ;

  // no/low work - no points or 1 point
  if(cnt == 0) { eSetType = SM_PST_VOID ; 
                 return(SM_SUCCESS) ; 
               }
  if(cnt == 1) { eSetType = SM_PST_POINTSIZED ; 
                 rCenterPt = crPoints[0] ; 
                 return(SM_SUCCESS) ;
               } 

  // let rLinerCenterPtPt = geometric center of points
  for(ii=0;ii<cnt;ii++)
    {
      rCenterPt += crPoints[ii] ;
    }
  rCenterPt /= crPoints.GetSize() ;

  // find max Gap
  for(ii=0;ii<cnt;ii++)
    {
      double dDist2 = rCenterPt.DistanceBetweenSquared(crPoints[ii]) ;
      if(rdMaxGap < dDist2) { rdMaxGap = dDist2 ; }
    }
  rdMaxGap  = smos_Sqrt(rdMaxGap) ;

  // report classification
  eSetType =   rdMaxGap <= dTol3d ? SM_PST_POINTSIZED : SM_PST_SCATTERED ;

  // all done
  return(SM_SUCCESS) ;

} // end smgu_BestFitPointToPoints

/*******************************************************************//**
PURPOSE: Find best fit line to a set of points using least squares.  
         Find least squares line fit to point set and max ga

NOTES: 
  1. returns SM_SUCCSS whenever eSetType is set.
  2. Check eSetType return value to see if PointSet is degenerate (Void or Point)
     and can't have a best fit point.
***********************************************************************/
SM_EXPORT SmStatus smgu_BestFitLineToPoints 
(
  SmTArray<SmPoint3d> const & crPoints,         // in : target point set
  double                      dTol3d,           // in : Min distance between distinct 3d points
  SmPointSetType            & eSetType,         // out: one of: SM_PST_VOID,          // empty point set
                                                //              SM_PST_POINTSIZED,    // all pts within tol of BestCenterPt
                                                //              SM_PST_LINEAR,        // all pts within tol of BestLine
                                                //              SM_PST_SCATTERED      // pts not within tol of BestLine - planar or scattered
  SmVector3d                & rLinePt,          // out: found line point = geometric center of all crPoints 
  SmVector3d                & rLineVec,         // out: for SM_PST_LINEAR     = found best line tangent
                                                //          SM_PST_SCATTERED  = found best line tangent
                                                //          SM_PST_POINTSIZED = not used set:[0 0 0] 
                                                //          SM_PST_VOID       = not used set:[0 0 0] 
  double                    & rdMaxLineGap,     // out: max gap SM_PST_POINTSIZED: between crPoints[i] and rCenterPt
                                                //              SM_PST_LINEAR    : between crPoints[i] and BestLine
                                                //              SM_PST_SCATTERED : between crPoints[i] and BestLine
  double                    & rdMaxCentroidGap, // out: max gap between crPoints[i] and rCenterPt
  SmMatrix                  * pOptM             // out: optional memory for intermediate M(LinePt) matrix for efficiency of
                                                //      smgu_BestFitPlaneToPoints,  sized:[3x3], NULL to ignore, default:[NULL]
)
{
  // init output
  eSetType = SM_PST_UNKNOWN ;
  rLineVec.Set(0.0,0.0,0.0) ;
  rLinePt. Set(0.0,0.0,0.0) ;
  rdMaxLineGap = 0.0 ;   

  // get rLinePt = geometric center of points
  smgu_BestFitPointToPoints(crPoints, dTol3d, eSetType, rLinePt, rdMaxCentroidGap) ;

  // low work - void or degenerate point set
  if(eSetType != SM_PST_SCATTERED)
    { return(SM_SUCCESS) ; } 

  // minimum problem: Pt[i] = LinePt + ui*LineVec + pi*LineVecPerp
  //                  where ui = LineVec.Dot(Pt[i]-LinePt)
  //                        LineVec is unit-Vec
  //                        LineVecPerp = any uni-vector perp to LineVec (doesn't matter - it drops out of the equations)
  //                  let Yi = Pt[i] - LinePt
  //                      Yi - ui*LineVec = pi*LineVecPerp
  //             
  //                  Minimize: E = Sum_i( pi**2 )
  //                  which yields form1 : Minimize E(LinePt, LineVec) = Sum_i(Yi_t * [ I - LineVec * LineVec_t ] * Yi)
  //                               form2 : Minimize E(LinePt, LineVec) = LineVec_t * Sum_i(((Yi * Yi) * I) - (Yi * Yi_t)) * LineVec
  //                                       let M(LinePt) = Sum_i(((Yi * Yi) * I) - (Yi * Yi_t))
  // 
  //                  0 = dE/dA of form 1: 0 = -2 * (I - LineVec * LineVec_t) * Sum_i( Yi )
  //                                       which is zero when Sum_i( Yi ) is zero, so
  //                                       LinePt = Sum_i( Pt[i] ) / m  (m = point count)
  //                  0 = dE/dA of form 2: Minimum of the Quadratic functional LineVec_t * C(LinePt) * LineVec
  //                                       is the smallest eigenvalue of M(LinePt) and
  //                                       LineVec is the associated unit-eigen vector, so
  //                                       LineVec = Eigenvector of M(LinePt) associated with its lowest eigenvalue
  //                                       with Pt[i] = [xi yi zi], and LinePt = [a b c]
  //                                       C(LinePt) = del * [1 0 0] - M(LinePt)
  //                                                         [0 1 0]   
  //                                                         [0 0 1]   
  //                                       with M(LinePt) = [ Sum_i((xi-a)**2)    Sum_i((xi-a)(yi-b)) Sum_i((xi-a)(zi-c)) ]
  //                                                        [ Sum_i((xi-a)(yi-b)) Sum_i((yi-b)**2)    Sum_i((yi-b)(zi-c)) ]
  //                                                        [ Sum_i((xi-a)(zi-c)) Sum_i((yi-b)(zi-c)) Sum_i((zi-c)**2)    ]
  //                                            del       =  Sum_i((xi-a)**2) + Sum_i((yi-b)**2) + Sum_i((zi-c)**2)

  // locals
  ULONG     ii, cnt = crPoints.GetSize() ;
  double    del = 0.0 ;
  SmMatrix  sM(3,3), sC(3,3) ;
  SmMatrix *pM = pOptM ? pOptM : &sM ;

  // set M(LinePt) = 0 
  pM->Clear() ;
                                    
  // build M(LinePt) = [ Sum_i((xi-a)**2)    Sum_i((xi-a)(yi-b)) Sum_i((xi-a)(zi-c)) ]
  //                   [ Sum_i((xi-a)(yi-b)) Sum_i((yi-b)**2)    Sum_i((yi-b)(zi-c)) ]
  //                   [ Sum_i((xi-a)(zi-c)) Sum_i((yi-b)(zi-c)) Sum_i((zi-c)**2)    ]
  //       del       = Sum_i((xi-a)**2) + Sum_i((yi-b)**2) + Sum_i((zi-c)**2)

  for(ii=0;ii<cnt;ii++)
    {
      double Yx = crPoints[ii].x - rLinePt.x ;
      double Yy = crPoints[ii].y - rLinePt.y ;
      double Yz = crPoints[ii].z - rLinePt.z ;

      pM->AddTo(0,0, Yx*Yx) ;
      pM->AddTo(0,1, Yx*Yy) ;
      pM->AddTo(0,2, Yx*Yz) ;
      pM->AddTo(1,1, Yy*Yy) ;
      pM->AddTo(1,2, Yy*Yz) ;
      pM->AddTo(2,2, Yz*Yz) ;

      del += Yx*Yx + Yy*Yy + Yz*Yz ;

    } // end iter building upper triangle of symmetric matrix

  // finish full symmetric M matrix
  pM->SetAt(1,0, pM->GetAt(0,1)) ;
  pM->SetAt(2,0, pM->GetAt(0,2)) ;
  pM->SetAt(2,1, pM->GetAt(1,2)) ;

  // build C(LinePt) = del * [1 0 0] - pM
  //                         [0 1 0]   
  //                         [0 0 1]   
  sC.SetAt(0,0, del - pM->GetAt(0,0)) ;
  sC.SetAt(0,1,       pM->GetAt(0,1)) ;
  sC.SetAt(0,2,       pM->GetAt(0,2)) ;
                                   
  sC.SetAt(1,0,       pM->GetAt(1,0)) ;
  sC.SetAt(1,1, del - pM->GetAt(1,1)) ;
  sC.SetAt(1,2,       pM->GetAt(1,2)) ;
                                   
  sC.SetAt(2,0,       pM->GetAt(2,0)) ;
  sC.SetAt(2,1,       pM->GetAt(2,1)) ;
  sC.SetAt(2,2, del - pM->GetAt(2,2)) ;

  // locals
  ULONG      lNumRealEigenValues ;
  double     adEigenValues[3] ;
  SmVector3d aEigenVectors[3] ;
  double     dDist, dU ;

  // Get Eigenvalues and EigenVectors of C(LinePt)
  sC.Compute3x3EigenVectors(lNumRealEigenValues, adEigenValues, aEigenVectors) ;

  // LineVec = unitized(aEigenVectors[2]) -- largest Eigenvalue.
  rLineVec = aEigenVectors[2] ;
  rLineVec.Unitize() ;

  // find line/pt[i] max gap
  for(rdMaxLineGap = 0.0,ii=0;ii<cnt;ii++)
    {
      // line point dist
      smgu_LinePointDistance(rLinePt, rLineVec, crPoints[ii], dDist, &dU) ;

      // save max dist
      if(rdMaxLineGap < dDist) { rdMaxLineGap = dDist ; }
    }

  // classify the point set
  eSetType = rdMaxLineGap <= dTol3d ? SM_PST_LINEAR : SM_PST_SCATTERED ;

  // all done
  return SM_SUCCESS ;

} // end smgu_BestFitLineToPoints

/*******************************************************************//**
PURPOSE: Determine if two points and tangent vectors are part of a
    circle and extract the information about that circle if they are.

NOTES: Circle is defined counter clockwise relative to P1 with
    Tan1 going clockwise.
***********************************************************************/
SM_EXPORT SmStatus smgu_BestFitCircleFromPointsTangents
(
  const SmPoint3d  & sP1,               // in :
  const SmVector3d & sTan1,             // in :
  const SmPoint3d  & sP2,               // in :
  const SmVector3d & sTan2,             // in :
  double             dAngleTolDeg,      // in :
  SmAxis2Placement & rPlacement,        // out:
  double           & rdRadius,          // out:
  double           & rdStartAngleDeg,   // out:
  double           & rdEndAngleDeg      // out:
)
{
    SmVector3d sTangent1 = sTan1;
    SmVector3d sTangent2 = sTan2;
    SmVector3d sNormal = sTangent1 * sTangent2;
    SmVector3d sXAxis;
    SmVector3d sYAxis;
    SmVector3d sZAxis;
    SmPoint3d sCenter;
    if (sNormal.LengthSquared() < SM_EFF_ZERO_SQ) {
        // Half circle
        sCenter = (sP1 + sP2)*0.5;
        sXAxis = sP1 - sCenter;
        rdRadius = sXAxis.Length();
        if (rdRadius < SM_EFF_ZERO) {
            SER(SM_ERR);
        }
        sXAxis = sXAxis/rdRadius;
        sYAxis = sTangent1;
        SER(sYAxis.Unitize());
        rPlacement.SetCanonical(sCenter,sXAxis,sYAxis);
        rdStartAngleDeg = 0.0;
        rdEndAngleDeg = 180.0;
        return SM_SUCCESS;
    }

    SmVector3d sVecP1P2 = sP2 - sP1;
    SER(sVecP1P2.Unitize());
    if (sTangent1.Dot(sVecP1P2) < 0.0) {
        sNormal = - sNormal;
    }
    SmVector3d sVec1 = sTangent1 * sNormal;
    SmVector3d sVec2 = sTangent2 * sNormal;
    SER(sVec1.Unitize());
    SER(sVec2.Unitize());
    double dT1, dT2;
    SER(smgu_LineLineClosestPoint(sP1,sVec1,sP2,sVec2,dT1,dT2));
    sCenter = sP1 + sVec1 * dT1;
    double dRadDiff = smos_Fabs(dT1-dT2);
    if (dRadDiff < SM_EFF_ZERO) {
        rdRadius = smos_Fabs(dT1);
        rdRadius = smos_CleanUpNoise(rdRadius);
        sXAxis = sVec1;
        sYAxis = sTangent1;
        SER(sYAxis.Unitize());
    }
    else {
        // Find the best fit circle
        SmVector3d sMidPnt = (sP1 + sP2)*0.5;
        SmVector3d sBiSector = sNormal * sVecP1P2;
        SER(sBiSector.Unitize());

        double dT;
        SER(smgu_LineClosestPoint(sMidPnt,sBiSector,sCenter,dT));
        // Snap center to bisector
        sCenter = sMidPnt + sBiSector*dT;
        sXAxis = sP1 - sCenter;
        sVec2 = sP2 - sCenter;
        rdRadius = sXAxis.Length();
        if (rdRadius < SM_EFF_ZERO) {
            SER(SM_ERR);
        }
        sXAxis = sXAxis/rdRadius;
        sNormal = sXAxis * sVec2;
        sYAxis = sNormal * sXAxis;
        SER(sYAxis.Unitize());

        // Test how good this is
        double dAngRad;
        SER(sVec1.AngleBetween(sXAxis,dAngRad));
        double dAngDeg = dAngRad * 180.0 / SM_PI;
        if (smos_Fabs(dAngDeg) > dAngleTolDeg) {
            SER(SM_ERR); // Doesn't satisfy angle tolerance
        }
    }

    rPlacement.SetCanonical(sCenter,sXAxis,sYAxis);
    rdStartAngleDeg = 0.0;
    double dAngRad;
    SER(sNormal.CCWAngleBetween(sXAxis,sVec2,dAngRad));
    rdEndAngleDeg = dAngRad * 180.0 / SM_PI;
    return SM_SUCCESS;

} // end smgu_BestFitCircleFromPointsTangents

/*******************************************************************//**
PURPOSE: Find best fit plane to array of points using least squares.  

NOTES: 
 1. returns SM_SUCCSS whenever eSetType is set
 2. Check eSetType return value to see if PointSet is degenerate (Void, Point, or Line)
    and can't have a best fit plane.
***********************************************************************/
SM_EXPORT SmStatus smgu_BestFitPlaneToPoints
(
  SmTArray<SmPoint3d> const & crPoints,          // in : target point set
  double                      dTol3d,            // in : Min distance between distinct 3d points
  SmPointSetType            & eSetType,          // out: one of: SM_PST_VOID,          // empty point set
                                                 //              SM_PST_POINTSIZED,    // all pts within tol of BestCenterPt
                                                 //              SM_PST_LINEAR,        // all pts within tol of BestLine
                                                 //              SM_PST_PLANAR,        // all pts within tol of BestPlane
                                                 //              SM_PST_SCATTERED      // pts not within tol of target shape
  SmVector3d                & rPlanePt,          // out: found point on plane - geometric center of all crPoints
  SmVector3d                & rPlaneNormal,      // out: for SM_PST_PLANAR     = found best plane normal
                                                 //          SM_PST_SCATTERED  = found best plane normal
                                                 //          SM_PST_LINEAR     = found best line tangent
                                                 //          SM_PST_POINTSIZED = not used set:[0 0 0] 
                                                 //          SM_PST_VOID       = not used set:[0 0 0] 
  double                    & rdMaxPlaneGap,     // out: max gap SM_PST_POINTSIZED: between crPoints[i] and rCenterPt
                                                 //              SM_PST_LINEAR    : between crPoints[i] and BestLine
                                                 //              SM_PST_PLANAR    : between crPoints[i] and BestPlane 
                                                 //              SM_PST_SCATTERED : between crPoints[i] and BestPlane
  double                    & rdMaxCentroidGap   // out: max gap between crPoints[i] and rCenterPt
)
{
  // init output
  eSetType = SM_PST_UNKNOWN ; 
  rPlaneNormal.Set(0.0,0.0,0.0) ;
  rPlanePt.    Set(0.0,0.0,0.0) ;
  rdMaxPlaneGap = 0.0 ;   

  // locals
  SmMatrix sM(3,3) ;

  // Get rPlanePt = geometric center of crPoints
  //     eSetType = oneof VOID, POINTSIZED, LINEAR, or SCATTERED (SCATTERED could be planar or scattered)
  //     sM       = minimization matrix needed to find best fit plane (happened to be built in this call)
  // 
  smgu_BestFitLineToPoints(crPoints, dTol3d, eSetType, rPlanePt, rPlaneNormal, rdMaxPlaneGap, rdMaxCentroidGap, &sM) ;

  // low work - point set is void, pointsized, or linear - not planar or scattered
  if(eSetType != SM_PST_SCATTERED)
    {
      // all done
      return(SM_SUCCESS) ;
    }

  // minimum problem: Pt[i] = PlanePt + ui*PlaneNrm + pi*PlaneNrmPerp
  //                  where ui = PlaneNrm.Dot(Pt[i]-PlanePt)
  //                        PlaneNrm is unit-Vec
  //                        PlaneNrmPerp = any uni-vector perp to PlaneNrm (doesn't matter - it drops out of the equations)
  //                  let Yi = Pt[i] - PlanePt
  //                      Yi - ui*PlaneNrm = pi*PlaneNrmPerp
  //                  Dist to plane for each pt is ui*PlaneNrm.
  //
  //                  Minimize: E = Sum_i( (ui*PlaneNrm)**2 ) = Sum_i ( ui**2 ) = Sum_i ( (PlaneNrm * Yi)**2 )
  //                  which yields form1 : Minimize E(PlanePt, PlaneNrm) = Sum_i(Yi_t * [ PlaneNrm * PlaneNrm_t ] * Yi)
  //                               form2 : Minimize E(PlanePt, PlaneNrm) = PlaneNrm_t * Sum_i((Yi * Yi_t)) * PlaneNrm
  //                                                                     = PlaneNrm_t * M(PlanePt) * PlaneNrm
  //                                       let M(PlanePt) = Sum_i(Yi * Yi_t)
  // 
  //                  0 = dE/dA of form 1: 0 = -2 * (PlaneNrm * PlaneNrm_t) * Sum_i( Yi )
  //                                       which is zero when Sum_i( Yi ) is zero, so
  //                                       PlanePt = Sum_i( Pt[i] ) / m  (m = point count)
  //                  0 = dE/dA of form 2: Minimum of the Quadratic functional PlaneNrm_t * M(PlanePt) * PlaneNrm
  //                                       is the smallest eigenvalue of M(PlanePt) and
  //                                       PlaneNrm is the associated unit-eigen vector, so
  //                                       PlaneNrm = Eigenvector of M(PlanePt) associated with its lowest eigenvalue
  //                                       with Pt[i] = [xi yi zi], and PlanePt = [a b c]
  //                                       with M(PlanePt) = [ Sum_i((xi-a)**2)    Sum_i((xi-a)(yi-b)) Sum_i((xi-a)(zi-c)) ]
  //                                                         [ Sum_i((xi-a)(yi-b)) Sum_i((yi-b)**2)    Sum_i((yi-b)(zi-c)) ]
  //                                                         [ Sum_i((xi-a)(zi-c)) Sum_i((yi-b)(zi-c)) Sum_i((zi-c)**2)    ]

  // locals
  ULONG      ii, cnt = crPoints.GetSize() ;
  ULONG      lNumRealEigenValues ;
  double     adEigenValues[3] ;
  SmVector3d aEigenVectors[3] ;

  // Get Eigenvalues and EigenVectors of M(LinePt)
  sM.Compute3x3EigenVectors(lNumRealEigenValues, adEigenValues, aEigenVectors) ;

  // LineVec = unitized(aEigenVectors[0]) -- smallest Eigenvalue.
  rPlaneNormal = aEigenVectors[0] ;
  rPlaneNormal.Unitize() ;

  // find plane/pt[i] max gap
  for(rdMaxPlaneGap=0.0,ii=0;ii<cnt;ii++)
    {
      // plane point dist
      double dDist = rPlaneNormal.Dot(crPoints[ii] - rPlanePt) ;

      // save max dist
      if(rdMaxPlaneGap < dDist) { rdMaxPlaneGap = dDist ; }
    }

  // classify the point set
  eSetType = rdMaxPlaneGap <= dTol3d ? SM_PST_PLANAR : SM_PST_SCATTERED ;

  // all done
  return SM_SUCCESS ;

} // end smgu_BestFitPlaneToPoints

/*******************************************************************//**
PURPOSE: Find sphere that interpolates 4 given points

NOTES: returns SM_ERR when 4 points cannot be placed on 
  the surface of a single sphere.

METHOD ---
  1. Find Center Point, CP, as intersection of 3 planes
        Given P0, P1, P2, P3 = 4 points on a sphere
        M0 = (P0 + P1) / 2 ;  N0 = P1 - P0 ;
        M1 = (P1 + P2) / 2 ;  N1 = P2 - P1 ;
        M2 = (P2 + P3) / 2 ;  N2 = P3 - P2 ;

        0 = (CP - M0) . N0    [ N0.x N0.y N0.z ] [ x ]   [ M0 . N0 ]
        0 = (CP - M1) . N1 => [ N1.x N1.y N1.z ] [ y ] = [ M1 . N1 ]
        0 = (CP - M2) . N2    [ N2.x N2.y N2.z ] [ z ]   [ M2 . N2 ]

  2. radius = | [x y z] - P0 |
***********************************************************************/
SM_EXPORT SmStatus smgu_SphereFrom4Points
(
  const SmPoint3d & crPoint0,       // in : 1st of 4 points
  const SmPoint3d & crPoint1,       // in : 2nd of 4 points
  const SmPoint3d & crPoint2,       // in : 3rd of 4 points
  const SmPoint3d & crPoint3,       // in : 4th of 4 points
  SmPoint3d       & rCenter,        // out: Center Point
  double          & dRadius         // out: Distance from CenterPoint to Circumference
)        
{
  // Plane data
  SmVector3d sM0 = (crPoint0 + crPoint1) / 2 ; SmVector3d sN0 = crPoint1 - crPoint0 ;
  SmVector3d sM1 = (crPoint1 + crPoint2) / 2 ; SmVector3d sN1 = crPoint2 - crPoint1 ;
  SmVector3d sM2 = (crPoint2 + crPoint3) / 2 ; SmVector3d sN2 = crPoint3 - crPoint2 ;

  // make intersection equation matrix
  // [ N0.x N0.y N0.z ] [ x ]   [ M0 . N0 ]
  // [ N1.x N1.y N1.z ] [ y ] = [ M1 . N1 ]
  // [ N2.x N2.y N2.z ] [ z ]   [ M2 . N2 ]
  SmMatrix sA(3,3) ;
  SmTArray<double> sB(3,NULL,3), sX(3,NULL,3) ;
  sA[0][0] = sN0.x ; sA[0][1] = sN0.y ; sA[0][2] = sN0.z ; sB[0] = sM0.Dot(sN0) ;
  sA[1][0] = sN1.x ; sA[1][1] = sN1.y ; sA[1][2] = sN1.z ; sB[1] = sM1.Dot(sN1) ;
  sA[2][0] = sN2.x ; sA[2][1] = sN2.y ; sA[2][2] = sN2.z ; sB[2] = sM2.Dot(sN2) ;
  
  // solve for Center Point
  SER_MSG(sA.SolveLinearSystem(sB, sX), _T("smgu_CircleFrom3Points() Failed due to Degenerate Point inputs")) ;

  // set outputs
  rCenter.Set(sX[0], sX[1], sX[2]) ;
  dRadius = (crPoint0-rCenter).Length() ;

  // check output
#ifdef SM_DEBUG_CODE
  double dMax1       = smos_Fabs(crPoint0.GetMaxDimension()) ;
  double dMax2       = smos_Fabs(crPoint1.GetMaxDimension()) ;
  double dMax3       = smos_Fabs(crPoint2.GetMaxDimension()) ;
  double dMax4       = smos_Fabs(crPoint3.GetMaxDimension()) ;
  double dScaledZero = SM_EFF_ZERO * (1.0 + smos_4Max(dMax1,
                                                      dMax2,
                                                      dMax3,
                                                      dMax4)) ;

  double dR0 = (rCenter - crPoint0).Length() ;
  //double dR1 = (rCenter - crPoint1).Length() ;
  //double dR2 = (rCenter - crPoint2).Length() ;
  //double dR3 = (rCenter - crPoint3).Length() ;
  
  SM_ASSERT_MSG(   SM_ARE_SAME_TO_TOL(dRadius, dR0, dScaledZero)
                && SM_ARE_SAME_TO_TOL(dRadius, dR0, dScaledZero)
                && SM_ARE_SAME_TO_TOL(dRadius, dR0, dScaledZero)
                && SM_ARE_SAME_TO_TOL(dRadius, dR0, dScaledZero),
                _T("smgu_SphereFrom4Points() Computed out of tolerance sphere")); 
#endif // SM_DEBUG_CODE

  // all done
  return SM_SUCCESS ;
 
} // end smgu_SphereFrom4Points

/*******************************************************************//**
PURPOSE: Find minimum sized circumscribing sphere for point set

NOTES: returns SM_ERR for empty point sets

METHOD --- every pair, triple, and quad of points defines
  a sphere.  One of those spheres is the minimum sized sphere that
  contains all the points in the given point set.
  
  This algorithm is a search through that space to find that
  one sphere without having to do an exhaustive search of all the
  combinatorial spheres.

NOTE --- The points on the boundary of the returned sphere are 
  members of this point set's convex boundary. 
***********************************************************************/
SM_EXPORT SmStatus smgu_BoundingSphereFromPointSet
(
  const SmTArray<SmPoint3d> &rPoints,       // in : PointSet to examine
  SmTArray<ULONG>           &rPtsOnSphere,  // out: indices of the points that define the minimum containing sphere
  SmPoint3d                 &rCenter,       // out: Center of minimum radius circumscribing sphere
  double                    &dRadius        // out: radius of minimum radius circumscribing sphere 
)       
{
  // check input
  if(rPoints.GetSize() == 0)
    { return(SM_ERR) ; }

  // init output
  rPtsOnSphere.ReSet() ;

  // locals
  ULONG       ii, jj, kk, ll, mm, nn ;
  SmBoolean   bOK ;
  SmStatus    bRtn ;
  ULONG       lIndex[4] = { 0,0,0,0 }, lTempIndex[3] ;
  const SmPoint3d  *pP[4], *pTempP[3] ;
  ULONG       iBoundedPtCnt = 0 ; // 2 = 2 Points on Sphere Surface
                                  // 3 = 3 Points on Sphere Surface
                                  // 4 = 4 Points on Sphere Surface
  // low work - one point
  if(rPoints.GetSize() == 1)
    { rCenter = rPoints[0] ;
      rPtsOnSphere.Add(0) ;
      dRadius = 0.0 ;
    }

  // start: sphere center and radius defined by p[0] and P[1]
  rCenter = (rPoints[0] + rPoints[1]) / 2.0 ;
  dRadius = (rPoints[0] - rPoints[1]).Length() / 2.0 ;

  // low work - two points, we're done
  if(rPoints.GetSize() == 2)
    { rPtsOnSphere.Add(0) ;
      rPtsOnSphere.Add(1) ;
      return(SM_SUCCESS) ; 
    }

  // set up two point sphere data
  iBoundedPtCnt = 2 ;
  pP[0]         = &rPoints[0] ; 
  pP[1]         = &rPoints[1] ; 
  lIndex[0]     = 0 ;
  lIndex[1]     = 1 ;
  double dRad2  = dRadius * dRadius ;
  double dTRad2 ;
  SmPoint3d sTCenter ;

  // arrive here for TWO POINT Sphere - 1st pass guarantees one of the two found pts is on the convex hull
  //                                  - final pass guarantees that all points are in the minimum sized sphere
  SmBoolean bDone = TRUE ;
  while(bDone == FALSE)
    {
      // for every point - check containment against current best guess for min sphere
      for(ii=0;ii<rPoints.GetSize();ii++)
        {
          const SmPoint3d *pPi  = &rPoints[ii] ;

          // skip defining points - they're already in the sphere
          if(   lIndex[0] == ii
             || lIndex[1] == ii
             || (iBoundedPtCnt > 2 && lIndex[2] == ii)
             || (iBoundedPtCnt > 3 && lIndex[3] == ii))
            { continue ; }

          // classify current point against current best sphere
          SmVector3d sVec  = *pPi - rCenter ;
          dTRad2 = sVec.LengthSquared() ;

          // skip points in the current sphere
          if(dTRad2 <= dRad2)
            continue ;

          // arrive here when pPi is outside of current Sphere.
          // Define a new bigger sphere that contains the outside point and the current defining points.
          SmBoolean bFoundNewSphere = FALSE ;

          // When sphere size is increased as we are about to do - we have to go one more big iteration later
          //   because the new bigger sphere is not guaranteed to contain all the points that
          //   the current smaller sphere might currently contain.
          bDone = FALSE ;

          // 2 POINT SPHERE CHECK 

          // see if any two point sphere made with 'outside' and a currently defining point will contain the other defining points
          for(jj=0;jj<iBoundedPtCnt && !bFoundNewSphere;jj++)
            {
              // when Sphere(Pj, pi) contains Other defining points
              sTCenter = (*pP[jj] + *pPi) / 2.0 ;
              dTRad2   = (*pP[jj] - *pPi).LengthSquared() / 4.0 ;

              // check other defining points for containment in this sphere
              bOK = TRUE ;
              for(kk=0;kk<iBoundedPtCnt && bOK;kk++)
                {
                  // skip the current defining point
                  if(kk == jj) continue ;

                  // check this defining point for containment
                  if((*pP[kk] - sTCenter).LengthSquared() < dTRad2) continue ;
                  else bOK = FALSE ;

                } // end iter other defining points checking for containment in trial sphere

              if(bOK)
                {
                  // found a 2 Point sphere which contains the current set of defining points
                  // set best sphere data and get back to checking remaining points
                  iBoundedPtCnt   = 2 ;
                  rCenter         = sTCenter ;
                  dRad2           = dTRad2 ;
                  pP[0]           = pP[jj] ;
                  pP[1]           = pPi ;
                  lIndex[0]       = lIndex[jj] ;
                  lIndex[1]       = ii ;
                  bFoundNewSphere = TRUE ;

                  // done with the 2 Pt sphere loop
                  break ;
                } // end FoundNewSphere check
            } // end iter all possible 2 Pt Spheres looking for a new larger containing sphere
      
          // arrive here when no 2 Pt sphere contains all the current defining points and the new 'outside' point
          SM_ASSERT(iBoundedPtCnt >= 2) ;

          // 3 POINT SPHERE CHECK

          // see if any of the possible 3 Pt spheres made with the 'outside' point contain the other defining point
          for(jj=0;jj<iBoundedPtCnt;jj++) 
            {
              for(kk=jj+1;kk<iBoundedPtCnt;kk++)
                {
                  SmVector3d sXAxis, sYAxis ;
                  SmExtent1d sIvl ;
                  bRtn = smgu_CircleFrom3Points(*pP[jj], *pP[kk], *pPi,
                                                sTCenter, sXAxis, sYAxis, sIvl,
                                                dRadius, 3, TRUE) ;
                  if(bRtn != SM_SUCCESS) continue ;
                  dTRad2 = dRadius * dRadius ;

                  // check other defining points for containment in this sphere
                  bOK = TRUE ;
                  for(ll=0;ll<iBoundedPtCnt && bOK; kk++)
                    {
                      // skip the current defining points
                      if(   ll == jj 
                         || ll == kk) continue ;

                      // check this defining point for containment
                      if((*pP[ll] - sTCenter).LengthSquared() < dTRad2) continue ;
                      else bOK = FALSE ;
                    } // end iter other defining points checking for containment in trial sphere

                  if(bOK)
                    {
                      // found a 3 Point Sphere which contains the current set of defining points.
                      // set best sphwere data and get back to checking remaining points
                      iBoundedPtCnt   = 3 ;
                      rCenter         = sTCenter ;
                      dRad2           = dTRad2 ;
                      pTempP[0]       = pP[jj] ;
                      pTempP[1]       = pP[kk] ;
                      lTempIndex[0]   = jj ;
                      lTempIndex[1]   = kk ;
                                      
                      pP[0]           = pTempP[0] ;
                      pP[1]           = pTempP[1] ;
                      pP[2]           = pPi ;
                      lIndex[0]       = lTempIndex[0] ;
                      lIndex[1]       = lTempIndex[1] ;
                      lIndex[2]       = ii ;
                      bFoundNewSphere = TRUE ;

                      // done with the 3 pt sphere loop
                      break ; 
                    } // end FoundNewSphere check
                } // end iter all possibble 3 Pt Spheres using just pPi and pP[jj]
            } // end iter all possibble 3 Pt Spheres 

          // arrive here when no 3 pt sphere contains all the current defining points and the new 'outside' point
          SM_ASSERT( iBoundedPtCnt >= 3) ;

          // 4 POINT SPHERE CHECK
          
          // find the 4 Pt sphere using the 'outside' point and current defining points that contains the other defining point
          for(mm=0;mm < ULONG(iBoundedPtCnt==3?1:4);mm++)
            {
              // test sequence [0 1 2] [0 1 3] [0 2 3] [1 2 3]; last three test cases only happen when iBoundedPtCnt = 4 ;
              jj = (mm == 3) ? 1 : 0 ;
              kk = (mm <  2) ? 1 : 2 ;
              ll = (mm == 0) ? 2 : 3 ;
              nn = 3 - mm ;  // the index of the not-used defining point

              
              bRtn = smgu_SphereFrom4Points(*pP[jj], *pP[kk], *pP[ll], *pPi, sTCenter, dRadius) ;
              if(bRtn != SM_SUCCESS) continue ;
              dTRad2 = dRadius * dRadius ;

              // check other defining points for containment in this sphere
              bOK = TRUE ;
              if(iBoundedPtCnt == 4)
                {
                  // check this defining point for containment
                  if( !((*pP[nn] - sTCenter).LengthSquared() < dTRad2)) 
                    { bOK = FALSE ; }
                } // end containment check for unused defining point

              if(bOK)
                {
                  // found a 4 Point Sphere which contains the current set of defining points.
                  // set best sphwere data and get back to checking remaining points
                  iBoundedPtCnt   = 3 ;
                  rCenter         = sTCenter ;
                  dRad2           = dTRad2 ;
                  pTempP[0]       = pP[jj] ;
                  pTempP[1]       = pP[kk] ;
                  pTempP[2]       = pP[ll] ;
                  lTempIndex[0]   = jj ;
                  lTempIndex[1]   = kk ;
                  lTempIndex[2]   = ll ;
                                  
                  pP[0]           = pTempP[0] ;
                  pP[1]           = pTempP[1] ;
                  pP[2]           = pTempP[2] ;
                  pP[3]           = pPi ;
                  lIndex[0]       = lTempIndex[0] ;
                  lIndex[1]       = lTempIndex[1] ;
                  lIndex[2]       = lTempIndex[2] ;
                  lIndex[3]       = ii ;
                  bFoundNewSphere = TRUE ;

                  // done with the 4 pt sphere loop
                  break ; 
                } // end FoundNewSphere check
            } // end iter all possible 4 pt spheres 

          // should only arrive here after finding a new bigger minimum sphere
          SM_ASSERT_MSG(bFoundNewSphere == FALSE, _T("growing sphere algorithm found an exception case")) ;
          return(SM_ERR) ;

        } // end iter all Points in this PointSet checking containment in the current guess minimum sphere 
    } // while looking for the minimum sphere

  // set output
  dRadius = smos_Sqrt(dRad2) ;
  for(ii=0;ii<iBoundedPtCnt;ii++) { rPtsOnSphere.Add(lIndex[ii]) ; }

  // check computation
#ifdef SM_DEBUG_CODDE
  // for every point - check containment against current best guess for min sphere
  for(ii=0;ii<rPoints.GetSize();ii++)
    {
      SmPoint3d *pPi  = &rPoints[ii] ;

      // skip defining points - they're already in the sphere
      if(   lIndex[0] == ii
         || lIndex[1] == ii
         || (iBoundedPtCnt > 2 && lIndex[2] == ii)
         || (iBoundedPtCnt > 3 && lIndex[3] == ii))
        { continue ; }

      // classify current point against current best sphere
      SmVector3d sVec  = *pPi - rCenter ;
      dTRad2 = sVec.LengthSquared() ;

      // skip points in the current sphere
      SM_ASSERT_MSG(dTRad2 <= dRad2, _T("BoundingSphereFromPointSet failed to compute containing sphere")) ;
    }
#endif // SM_DEBUG_CODE

  // all done
  return(SM_SUCCESS) ;

} // end smgu_BoundingSphereFromPointSet

/*******************************************************************//**
PURPOSE: compute dot(a,cross(b,c))

NOTES:
***********************************************************************/
SM_EXPORT double smgu_VecTripleProduct
(
  const double *pA,          // in : a of dot(a,cross(b,c))
  const double *pB,          // in : b of dot(a,cross(b,c))
  const double *pC           // in : c of dot(b,cropp(b,c))
)          
{ 
  return(  pA[0] * (pB[1] * pC[2] - pC[1] * pB[2])
         + pA[1] * (pB[2] * pC[0] - pC[2] * pB[0])
         + pA[2] * (pB[0] * pC[1] - pC[0] * pB[1]) ) ;

} // end smgu_VecTripleProduct

/*******************************************************************//**
PURPOSE: Copy a transform matrix

NOTES:
***********************************************************************/
SM_EXPORT void smgu_3x3Copy
(
  const double adMatFrom[3][3],  // in : double[3][3] array to copy
  double       adMatTo  [3][3]   // out: double[3][3] array to change
)
{
  if(adMatFrom != adMatTo)
    {
      // okay to use smos_MemCpy on base (double) and static class (gw_CPOINT) objects.
        SE(smos_MemCpy(adMatTo, adMatFrom, 9 * sizeof(double), 9 * sizeof(double)));
    }

} // end smgu_3x3Copy

/*******************************************************************//**
PURPOSE: Multiply a pair of 3x3 matrices

NOTES:
***********************************************************************/
SM_EXPORT void smgu_3x3Mult
(
  const double adMat1[3][3],       // in : Mat1   of Result = Mat1 * Mat2
  const double adMat2[3][3],       // in : Mat2   of Result = Mat1 * Mat2
  double       adMatResult[3][3]   // out: Result of Result = Mat1 * Mat2
)  
{
  // local result - allows rdMatResult to be either rdMat1 or rdMat2
  ULONG ii, jj, kk ;
  double adMat[3][3] ;
  double *pVec ;
  const double *pMat1 ;

  // do multiply into local matrix
  for(ii=0;ii<3;ii++) 
    {
      pVec  = adMat[ii] ;
      pMat1 = adMat1[ii] ;
      for(jj=0;jj<3;jj++) 
        {
          pVec[jj] = 0.0 ;
          for(kk=0;kk<3;kk++) 
            {
              pVec[jj] += pMat1[kk] * adMat2[kk][jj];
            }
        }
    } // end iter all row/col multiplies

  // set output
  smgu_3x3Copy(adMat, adMatResult) ;

} // end smgu_3x3Mult

/*******************************************************************//**
PURPOSE: Get 3x3 matrix determinate

NOTES:
***********************************************************************/
SM_EXPORT double smgu_3x3Determinate
(
  const double adA[3][3]    // in : matrix
)  
{ 
  return(  adA[0][0] * (adA[1][1] * adA[2][2] - adA[1][2] * adA[2][1])
         + adA[0][1] * (adA[1][2] * adA[2][0] - adA[1][0] * adA[2][2])
         + adA[0][2] * (adA[1][0] * adA[2][1] - adA[1][1] * adA[2][0])) ;

} // end smgu_3x3Determinate
                                             
/*******************************************************************//**
PURPOSE: Get 3x3 matrix determinate

NOTES:
***********************************************************************/
SM_EXPORT double smgu_3x3Determinate        
(
  double a00, double a01, double a02, // in : a oth row elements
  double a10, double a11, double a12, // in : a 1st row elements
  double a20, double a21, double a22  // in : a 2nd row elements
) 
{ 
  return(  a00 * (a11 * a22 - a12 * a21)
         + a01 * (a12 * a20 - a10 * a22)
         + a02 * (a10 * a21 - a11 * a20)) ;

} // end smgu_3x3Determinate                  

/*******************************************************************//**
PURPOSE: Find the determinent of a matrix defined by 3 vectors.

NOTES: 
***********************************************************************/
SM_EXPORT double smgu_Determinant3Vectors
(
  const SmVector3d & crV1,        // in :
  const SmVector3d & crV2,        // in :
  const SmVector3d & crV3         // in :
)
{
  double dRet = (  crV1.x * crV2.y * crV3.z - crV1.x * crV2.z * crV3.y 
                 + crV1.y * crV2.z * crV3.x - crV1.y * crV2.x * crV3.z 
                 + crV1.z * crV2.x * crV3.y - crV1.z * crV2.y * crV3.x) ;
  return dRet;

} // end smgu_Determinant3Vectors

/***************************************************//**
  smgu_3x3Solve solve Ax=b when A is a 3x3 matrix
*******************************************************/
SM_EXPORT SmStatus smgu_3x3Solve 
(
  const double       adA[3][3],     // in : A matrix
  const SmVector3d & rB,            // in : b of Ax = b
  SmVector3d       & rX             // out: x of Ax = b
)
{
  // init output
  rX.Set(0,0,0) ;

  // determinate
  double det = smgu_3x3Determinate(adA) ; 

  // watch out for singular matrices
  SER(SM_IS_ZERO(det) ? SM_ERR : SM_SUCCESS) ;

  // when input == output, copy input
  SmVector3d  sTmpB ;
  const SmVector3d *pB ;
  if(&rB == &rX) { sTmpB = rB ;
                   pB    = &sTmpB ;
                 }
  else           { pB    = &rB ; 
                 }
  
  // hard code solutions for Ax=b
  // with columns of A = [ A0 A1 A2 ], solutions are
  // x0 = |b  A1 A2| / |A0 A1 A2|
  // x1 = |A1 b  A2| / |A0 A1 A2|
  // x2 = |A1 A1 b | / |A0 A1 A2|

  rX.x = smgu_VecTripleProduct((double *)pB, adA[1], adA[2]) / det ;
  rX.y = smgu_VecTripleProduct(adA[0], (double *)pB, adA[2]) / det ;
  rX.z = smgu_VecTripleProduct(adA[0], adA[1], (double *)pB) / det ;

  // test result
  SM_ASSERT(SM_IS_ZERO(adA[0][0]*rX.x + adA[0][1]*rX.y + adA[0][2]*rX.z - (*pB)[0])) ;
  SM_ASSERT(SM_IS_ZERO(adA[1][0]*rX.x + adA[1][1]*rX.y + adA[1][2]*rX.z - (*pB)[1])) ;
  SM_ASSERT(SM_IS_ZERO(adA[2][0]*rX.x + adA[2][1]*rX.y + adA[2][2]*rX.z - (*pB)[2])) ;

  // all done - success
  return(0) ;

} // end smgu_3x3Solve

/***************************************************//**
  smgu_3x3Solve solve Ax=b when A is a 3x3 matrix
*******************************************************/
SM_EXPORT SmStatus smgu_3x3Solve
(
  const double *A0,            // in : 0th iRow of A matrix, ordered:[00 01 02], sized:[3]
  const double *A1,            // in : 1st iRow of A matrix, ordered:[10 11 12], sized:[3]
  const double *A2,            // in : 2nd iRow of A matrix, ordered:[20 21 22], sized:[3]
  const double *b,             // in : b of Ax = b, sized:[3]
  double       *dX             // out: dX of Ax = b, sized:[3]
)            
{
  // check input
  SER((A0 && A1 && A2 && b && dX) ? SM_SUCCESS : SM_ERR) ;

  // when input == output, copy output
  double aB[3] ;
  const double *pB ;
  if(b == dX) { aB[0] = b[0] ; aB[1] = b[1] ; aB[2] = b[2] ; 
                pB = aB ;
              }
  else        { pB = b ;
              }

  // init output
  dX[0] = dX[1] = dX[2] = 0.0 ;

  // determinate
  double dDet = smgu_VecTripleProduct(A0,A1,A2) ;

  // watch out for singular matrices
  SER(SM_IS_ZERO(dDet) ? SM_ERR : SM_SUCCESS) ;

  // hard code solutions for Ax=b
  // with columns of A = [ A0 A1 A2 ], solutions are
  // x0 = |b  A1 A2| / |A0 A1 A2|
  // x1 = |A1 b  A2| / |A0 A1 A2|
  // x2 = |A1 A1 b | / |A0 A1 A2|

  dX[0] = (  pB[0]  * A1[1] * A2[2] - pB[0]  * A1[2] * A2[1]
           + pB[1]  * A2[1] * A0[2] - pB[1]  * A2[2] * A0[1]
           + pB[2]  * A0[1] * A1[2] - pB[2]  * A0[2] * A1[1]) / dDet ;

  dX[1] = (  A0[0] * pB[1]  * A2[2] - A0[0] * A1[2] * pB[2]
           + A1[0] * pB[2]  * A0[2] - A1[0] * A2[2] * pB[0]
           + A2[0] * pB[0]  * A1[2] - A2[0] * A0[2] * pB[1]) / dDet ;

  dX[2] = (  A0[0] * A1[1] * pB[2]  - A0[0] * pB[1]  * A2[1]
           + A1[0] * A2[1] * pB[0]  - A1[0] * pB[2]  * A0[1]
           + A2[0] * A0[1] * pB[1]  - A2[0] * pB[0]  * A1[1]) / dDet ;

  // test result
  SM_ASSERT(SM_IS_ZERO(A0[0]*dX[0] + A0[1]*dX[1] + A0[2]*dX[2] - pB[0])) ;
  SM_ASSERT(SM_IS_ZERO(A1[0]*dX[0] + A1[1]*dX[1] + A1[2]*dX[2] - pB[1])) ;
  SM_ASSERT(SM_IS_ZERO(A2[0]*dX[0] + A2[1]*dX[1] + A2[2]*dX[2] - pB[2])) ;

  // all done - success
  return(0) ;

} // end smgu_3x3Solve

/*******************************************************************//**
PURPOSE: Invert a 3x3 matrix - solve Ax=b when A is a 3x3 matrix

NOTES:    [ a00 a01 a02 ]               [   a22a11-a21a12  -(a22a01-a21a02)   a12a01-a11a02  ]
       Inv[ a10 a11 a12 ]    =  1/DET * [ -(a22a10-a20a12)   a22a00-a20a02  -(a12a00-a10a02) ]
          [ a20 a21 a22 ]               [   a21a10-a20a11  -(a21a00-a20a01)   a11a00-a10a01  ]

RETURNS: SM_ERR when det[A] == 0, i.e. a degenerate matrix that can't be inverted.
         No error is signalled.
***********************************************************************/
SM_EXPORT SmStatus smgu_3x3Inverse      
(
  const double *A0,             // in : 0th iRow of A matrix, ordered:[00 01 02], sized:[3]
  const double *A1,             // in : 1st iRow of A matrix, ordered:[10 11 12], sized:[3]
  const double *A2,             // in : 2nd iRow of A matrix, ordered:[20 21 22], sized:[3]
  double        adInv[3][3]     // out: inverse of A
)    
{
  // check input
  SER((A0 && A1 && A2) ? SM_SUCCESS : SM_ERR) ;

  // determinate
  double dDet = smgu_VecTripleProduct(A0,A1,A2) ;

  // watch out for singular matrices
  if(SM_IS_ZERO(dDet))
    {
      return(SM_ERR) ;
    }

  //    [ a00 a01 a02 ]               [   a22a11-a21a12  -(a22a01-a21a02)   a12a01-a11a02  ]     
  // Inv[ a10 a11 a12 ]    =  1/DET * [ -(a22a10-a20a12)   a22a00-a20a02  -(a12a00-a10a02) ]     
  //    [ a20 a21 a22 ]               [   a21a10-a20a11  -(a21a00-a20a01)   a11a00-a10a01  ]
  
  // invert into temp memory so that inputs and outputs can be the same
  double tmp[3][3] ;     

  tmp[0][0] =  (A2[2] * A1[1] - A2[1] * A1[2]) / dDet ;
  tmp[0][1] = -(A2[2] * A0[1] - A2[1] * A0[2]) / dDet ;
  tmp[0][2] =  (A1[2] * A0[1] - A1[1] * A0[2]) / dDet ;
                                            
  tmp[1][0] = -(A2[2] * A1[0] - A2[0] * A1[2]) / dDet ;
  tmp[1][1] =  (A2[2] * A0[0] - A2[0] * A0[2]) / dDet ;
  tmp[1][2] = -(A1[2] * A0[0] - A1[0] * A0[2]) / dDet ;
                                            
  tmp[2][0] =  (A2[1] * A1[0] - A2[0] * A1[1]) / dDet ;
  tmp[2][1] = -(A2[1] * A0[0] - A2[0] * A0[1]) / dDet ;
  tmp[2][2] =  (A1[1] * A0[0] - A1[0] * A0[1]) / dDet ;

  // set output
  smgu_3x3Copy(tmp, adInv) ;

#ifdef SM_DEBUG_CODE

  double A[3][3] ;
  double C[3][3] ;
  SmBoolean bOK = TRUE ; 
  A[0][0] = A0[0] ; A[0][1] = A0[1] ; A[0][2] = A0[2] ; 
  A[1][0] = A1[0] ; A[1][1] = A1[1] ; A[1][2] = A1[2] ; 
  A[2][0] = A2[0] ; A[2][1] = A2[1] ; A[2][2] = A2[2] ; 

  smgu_3x3Mult(A, adInv, C) ;

  for(ULONG ii=0;ii<3&&bOK;ii++)
    {
      for(ULONG jj=0;jj<3&&bOK;jj++)
        {
          bOK &=   (ii == jj) 
                 ? SM_IS_ZERO(C[ii][jj] - 1.0) 
                 : SM_IS_ZERO(C[ii][jj]) ;
        }
    }
  if(bOK == FALSE)
    {
      SM_ASSERT(bOK) ;
    }
#endif // SM_DEBUG_CODE

  // all done
  return(SM_SUCCESS) ;

} // end smgu_3x3Inverse

/*******************************************************************//**
PURPOSE: Copy a transform matrix

NOTES:
***********************************************************************/
SM_EXPORT void smgu_4x4Copy
(
  const double adMatFrom[4][4],  // in : double[4][4] array to copy
  double       adMatTo  [4][4]   // out: double[4][4] array to change
)  
{
  if(adMatFrom != adMatTo)
    {
      adMatTo[0][0] = adMatFrom[0][0] ;
      adMatTo[0][1] = adMatFrom[0][1] ;
      adMatTo[0][2] = adMatFrom[0][2] ;
      adMatTo[0][3] = adMatFrom[0][3] ;
                                
      adMatTo[1][0] = adMatFrom[1][0] ;
      adMatTo[1][1] = adMatFrom[1][1] ;
      adMatTo[1][2] = adMatFrom[1][2] ;
      adMatTo[1][3] = adMatFrom[1][3] ;
                                
      adMatTo[2][0] = adMatFrom[2][0] ;
      adMatTo[2][1] = adMatFrom[2][1] ;
      adMatTo[2][2] = adMatFrom[2][2] ;
      adMatTo[2][3] = adMatFrom[2][3] ;
                                
      adMatTo[3][0] = adMatFrom[3][0] ;
      adMatTo[3][1] = adMatFrom[3][1] ;
      adMatTo[3][2] = adMatFrom[3][2] ;
      adMatTo[3][3] = adMatFrom[3][3] ;
    }

} // end smgu_4x4Copy

/*******************************************************************//**
PURPOSE: Multiply a pair of transformation matrices

NOTES:
***********************************************************************/
SM_EXPORT void smgu_4x4Mult
(
  const double adMat1[4][4],       // in : Mat1   of Result = Mat1 * Mat2
  const double adMat2[4][4],       // in : Mat2   of Result = Mat1 * Mat2
  double       adMatResult[4][4]   // out: Result of Result = Mat1 * Mat2
)  
{
  // local result - allows rdMatResult to be either rdMat1 or rdMat2
  ULONG ii, jj, kk ;
  double adMat[4][4] ;
  double *pVec ;
  const double *pMat1 ;

  // do multiply into local matrix
  for(ii=0;ii<4;ii++) 
    {
      pVec  = adMat[ii] ;
      pMat1 = adMat1[ii] ;
      for(jj=0;jj<4;jj++) 
        {
          pVec[jj] = 0.0 ;
          for(kk=0;kk<4;kk++) 
            {
              pVec[jj] += pMat1[kk] * adMat2[kk][jj];
            }
        }
    } // end iter all row/col multiplies

  // set output
  smgu_4x4Copy(adMat, adMatResult) ;

} // end smgu_4x4Mult

/*******************************************************************//**
PURPOSE: return 4x4 matrix determinate

NOTES:
***********************************************************************/
SM_EXPORT double smgu_4x4Determinate
(
  const double A[4][4]    // in : matrix
)         
{ 
  return( A[0][0] * (  A[1][1] * (A[2][2] * A[3][3] - A[2][3] * A[3][2])
                     + A[1][2] * (A[2][3] * A[3][1] - A[2][1] * A[3][3])
                     + A[1][3] * (A[2][1] * A[3][2] - A[2][2] * A[3][1]))

         -A[0][1] * (  A[1][0] * (A[2][2] * A[3][3] - A[2][3] * A[3][2])
                     + A[1][2] * (A[2][3] * A[3][0] - A[2][0] * A[3][3])
                     + A[1][3] * (A[2][0] * A[3][2] - A[2][2] * A[3][0]))

         +A[0][2] * (  A[1][0] * (A[2][1] * A[3][3] - A[2][3] * A[3][1])
                     + A[1][1] * (A[2][3] * A[3][0] - A[2][0] * A[3][3])
                     + A[1][3] * (A[2][0] * A[3][1] - A[2][1] * A[3][0]))

         -A[0][3] * (  A[1][0] * (A[2][1] * A[3][2] - A[2][2] * A[3][1])
                     + A[1][1] * (A[2][2] * A[3][0] - A[2][0] * A[3][2])
                     + A[1][2] * (A[2][0] * A[3][1] - A[2][1] * A[3][0])) )  ;

} // end smgu_4x4Determinate

/*******************************************************************//**
PURPOSE: Invert a 4x4 matrix

NOTES:                   [ a00 a01 a02 a03]     
       InvA = Inverse of [ a10 a11 a12 a13]  = 1/det * adj(A) 
                         [ a20 a21 a22 a14]     
                         [ a30 a31 a32 a33]

    adj(A) = [C]transpose
    Cij = (-1)**(i+j) * det[ijth minor]

    This requires a 4x4 determinate and 16 minor matrix determinates.
    which I think is more than LU decomposition.  But I've typed this
    in here just to have it.  Unless the inverse is put into a tight
    loop this number of floating point operations won't be a lot.

REETURNS: SM_ERR when det[A] is zero.  Does not signal an error.
***********************************************************************/
SM_EXPORT SmStatus smgu_4x4Inverse
(
  const double A[4][4],        // in : A matrix to invert
  double       InvA[4][4]      // out: inverse of A
)     
{
  // copy input in case output == input
  double TmpInvA[4][4] ;

  // determinate
  double det = smgu_4x4Determinate(A) ;

  // watch out for singular matrices
  if(SM_IS_ZERO(det))
    { return(SM_ERR) ; }

  //                   [ a00 a01 a02 a03]     
  // InvA = Inverse of [ a10 a11 a12 a13] = 1/det * adj(A) ; adj(A) = [C]transpose ; Cij = (-1)**(i+j) * det[ijth minor matrix]    
  //                   [ a20 a21 a22 a23]     
  //                   [ a20 a31 a32 a33]
  
  // invert into tmp array to allow input to equal output

  TmpInvA[0][0] = + smgu_3x3Determinate(A[1][1], A[1][2], A[1][3], 
                                        A[2][1], A[2][2], A[2][3], 
                                        A[3][1], A[3][2], A[3][3]) / det ;
  TmpInvA[1][0] = - smgu_3x3Determinate(A[1][0], A[1][2], A[1][3], 
                                        A[2][0], A[2][2], A[2][3], 
                                        A[3][0], A[3][2], A[3][3]) / det ;
  TmpInvA[2][0] = + smgu_3x3Determinate(A[1][0], A[1][1], A[1][3], 
                                        A[2][0], A[2][1], A[2][3], 
                                        A[3][0], A[3][1], A[3][3]) / det ;
  TmpInvA[3][0] = - smgu_3x3Determinate(A[1][0], A[1][1], A[1][2], 
                                        A[2][0], A[2][1], A[2][2], 
                                        A[3][0], A[3][1], A[3][2]) / det ;
                                                                    
  TmpInvA[0][1] = - smgu_3x3Determinate(A[0][1], A[0][2], A[0][3], 
                                        A[2][1], A[2][2], A[2][3], 
                                        A[3][1], A[3][2], A[3][3]) / det ;
  TmpInvA[1][1] = + smgu_3x3Determinate(A[0][0], A[0][2], A[0][3], 
                                        A[2][0], A[2][2], A[2][3], 
                                        A[3][0], A[3][2], A[3][3]) / det ;
  TmpInvA[2][1] = - smgu_3x3Determinate(A[0][0], A[0][1], A[0][3], 
                                        A[2][0], A[2][1], A[2][3], 
                                        A[3][0], A[3][1], A[3][3]) / det ;
  TmpInvA[3][1] = + smgu_3x3Determinate(A[0][0], A[0][1], A[0][2], 
                                        A[2][0], A[2][1], A[2][2], 
                                        A[3][0], A[3][1], A[3][2]) / det ;
                                                                    
  TmpInvA[0][2] = + smgu_3x3Determinate(A[0][1], A[0][2], A[0][3], 
                                        A[1][1], A[1][2], A[1][3], 
                                        A[3][1], A[3][2], A[3][3]) / det ;
  TmpInvA[1][2] = - smgu_3x3Determinate(A[0][0], A[0][2], A[0][3], 
                                        A[1][0], A[1][2], A[1][3], 
                                        A[3][0], A[3][2], A[3][3]) / det ;
  TmpInvA[2][2] = + smgu_3x3Determinate(A[0][0], A[0][1], A[0][3], 
                                        A[1][0], A[1][1], A[1][3], 
                                        A[3][0], A[3][1], A[3][3]) / det ;
  TmpInvA[3][2] = - smgu_3x3Determinate(A[0][0], A[0][1], A[0][2], 
                                        A[1][0], A[1][1], A[1][2], 
                                        A[3][0], A[3][1], A[3][2]) / det ;
                                                                 
  TmpInvA[0][3] = - smgu_3x3Determinate(A[0][1], A[0][2], A[0][3],         
                                        A[1][1], A[1][2], A[1][3],         
                                        A[2][1], A[2][2], A[2][3]) / det ; 
  TmpInvA[1][3] = + smgu_3x3Determinate(A[0][0], A[0][2], A[0][3],         
                                        A[1][0], A[1][2], A[1][3],         
                                        A[2][0], A[2][2], A[2][3]) / det ; 
  TmpInvA[2][3] = - smgu_3x3Determinate(A[0][0], A[0][1], A[0][3],         
                                        A[1][0], A[1][1], A[1][3],         
                                        A[2][0], A[2][1], A[2][3]) / det ; 
  TmpInvA[3][3] = + smgu_3x3Determinate(A[0][0], A[0][1], A[0][2],         
                                        A[1][0], A[1][1], A[1][2],         
                                        A[2][0], A[2][1], A[2][2]) / det ; 

#ifdef SM_DEBUG_CODE

  double C[4][4] ;
  SmBoolean bOK = TRUE ; 
  smgu_4x4Mult(A, TmpInvA, C) ;
  for(ULONG ii=0;ii<4&&bOK;ii++)
    {
      for(ULONG jj=0;jj<4&&bOK;jj++)
        {
          bOK &=   (ii == jj) 
                 ? SM_IS_ZERO(C[ii][jj] - 1.0) 
                 : SM_IS_ZERO(C[ii][jj]) ;
        }
    }
  SM_ASSERT(bOK) ;
    
#endif // SM_DEBUG_CODE

  // set output
  smgu_4x4Copy(TmpInvA, InvA) ;   

  // all done
  return(SM_SUCCESS) ;

} // end smgu_4x4Inverse

/*******************************************************************//**
PURPOSE: return TRUE when matrix A == B to given tol

NOTES:
***********************************************************************/
SM_EXPORT SmBoolean smgu_4x4AreEqual
(
  const double adA[4][4],     // in : A of A == B
  double       adB[4][4],     // out: B of A == B
  double       dTol           // tolerance for equality check, default:[SM_EFF_ZERO]
)          
{
  SmBoolean bRtn = TRUE ;

  bRtn &= SM_IS_ZERO_TO_TOL(adA[0][0] - adB[0][0], dTol) ;
  bRtn &= SM_IS_ZERO_TO_TOL(adA[0][1] - adB[0][1], dTol) ;
  bRtn &= SM_IS_ZERO_TO_TOL(adA[0][2] - adB[0][2], dTol) ;
  bRtn &= SM_IS_ZERO_TO_TOL(adA[0][3] - adB[0][3], dTol) ;
                                
  bRtn &= SM_IS_ZERO_TO_TOL(adA[1][0] - adB[1][0], dTol) ;
  bRtn &= SM_IS_ZERO_TO_TOL(adA[1][1] - adB[1][1], dTol) ;
  bRtn &= SM_IS_ZERO_TO_TOL(adA[1][2] - adB[1][2], dTol) ;
  bRtn &= SM_IS_ZERO_TO_TOL(adA[1][3] - adB[1][3], dTol) ;
                                
  bRtn &= SM_IS_ZERO_TO_TOL(adA[2][0] - adB[2][0], dTol) ;
  bRtn &= SM_IS_ZERO_TO_TOL(adA[2][1] - adB[2][1], dTol) ;
  bRtn &= SM_IS_ZERO_TO_TOL(adA[2][2] - adB[2][2], dTol) ;
  bRtn &= SM_IS_ZERO_TO_TOL(adA[2][3] - adB[2][3], dTol) ;
                                
  bRtn &= SM_IS_ZERO_TO_TOL(adA[3][0] - adB[3][0], dTol) ;
  bRtn &= SM_IS_ZERO_TO_TOL(adA[3][1] - adB[3][1], dTol) ;
  bRtn &= SM_IS_ZERO_TO_TOL(adA[3][2] - adB[3][2], dTol) ;
  bRtn &= SM_IS_ZERO_TO_TOL(adA[3][3] - adB[3][3], dTol) ;
  
  // all done
  return(bRtn) ;

} // end smgu_4x4AreEqual

/*******************************************************************//**
PURPOSE:  Compute curvature given the 1st and 2nd curve 
              derivative vectors

NOTES: rtn: k = sign * |Cross(Wt,Wtt)|/|Wt|**3
***********************************************************************/
SM_EXPORT SmStatus smgu_SignedCurvature 
(
  const SmVector3d &crWt,    // in : 1st Derivative
  const SmVector3d &crWtt,   // in : 2nd Derivative
  const SmVector3d &crN,     // in : Points to Osculating Circle Centers
                             //      which are given positive curvatures
  double &dCurvature         // out: Curvature signed so that positive means
                             //      osculating circle center is in direction of crN
)
{
  // zero length geometry vectors - yield zero curvatures
  if(   crWt.Length()  < SM_EFF_ZERO_SQRT
     || crWtt.Length() < SM_EFF_ZERO_SQRT) 
    { dCurvature = 0.0 ;
      return(SM_SUCCESS) ;
    }
  
  // check for zero length Normal Vector   
  SM_ASSERT(crN.Length() > SM_EFF_ZERO_SQRT) ;

  // get b = cross(Wt,Wtt)
  SmVector3d sBiNorm = crWt * crWtt ;

  // get vector lengths
  double dBLength = sBiNorm.Length() ;
  double dTLength = crWt.Length() ;

  // check for parallel Wt, Wtt vectors
  if(dBLength < SM_EFF_ZERO_SQRT) 
    { dCurvature = 0.0 ;
      return(SM_SUCCESS) ;
    }

  // Sign = Sign(dot(m,N)) where m = cross(b,t) and b = cross(Wt,Wtt)
  double dSign = smos_Sgn(crN.Dot(sBiNorm * crWt)) ;

  // set Curvature = |cross(Wt,Wtt)| / |dot(Wt,Wt)|**3
  dCurvature = dSign * ( dBLength / (dTLength * dTLength * dTLength)) ;

  // all done
  return(SM_SUCCESS) ;

} // end smgu_SignedCurvature

/*******************************************************************//**
PURPOSE: Convert a curve evaluation into a frenet frame set of vectors

OUTPUTS ---
 outputs all computable values and zero for those that can't be computed.

 Lines            - returns position, 1st derivative, and zero curvature and torsion vectors
 planar curves    - returns position, 1st derivative, curvature vectors, and zero torsion vector.
 non-planar curve - returns position, 1st derivative, curvature, and torsion vectors.

RETURNS --- SM_SUCCESS when all requested values are computable
   degenerate        curves return a SM_ERR for curvature and torsion requests.
   linear            curves return a SM_ERR for torsion requests.
   non-linear planar curves always return SM_SUCCESS
   non-planar        curves always return SM_SUCCESS
***********************************************************************/
SM_EXPORT SmStatus smgu_FrenetFromDerivatives
(
  SmVector3d aVals[],                        // in : aVals[0] = pos, aVals[1] = 1st deriv, aVals[2] = 2nd deriv, aVals[3] = 3rd deriv
  ULONG lNumGeometricVectors,                // in : rtn, 0 = position, 1=tangent, 2=curvature_Normal, 3=Torsion_BiNormal
  SmVector3d aPointAndFrenetFrameVectors[]   // out: a[0] = position
                                             //      a[1] = 1st derivative,   Mag = Speed,   direction = tangent vector
                                             //      a[2] = Curvature Vector, Mag = K,       direction = normal vector
                                             //      a[3] = Torsion Vector,   Mag = Torsion, direction = binormal vector
                                             // note: Osculating circle center: center = pos + RadOfCurv * NormalVector, RadOfCurv = 1/K
                                             //       center = a[0] + a[2]/a[2].LengthSquared(), when a[2].LengthSquared() is not zero.
)
{
  SM_ASSERT(lNumGeometricVectors <= 3);

  // save position
  aPointAndFrenetFrameVectors[0] = aVals[0];
  if (lNumGeometricVectors == 0) { return SM_SUCCESS; }

  // save tangent
  aPointAndFrenetFrameVectors[1] = aVals[1];
  if (lNumGeometricVectors == 1) { return SM_SUCCESS; }

  // init remaining output vectors aPointAndFrenetFrameVectors[2]
  aPointAndFrenetFrameVectors[2].Set(0.0, 0.0, 0.0) ;
  if(lNumGeometricVectors == 3) aPointAndFrenetFrameVectors[3].Set(0.0, 0.0, 0.0) ;

  // save curvature vector

  // error condition - zero length tangent
  double dTanLengSq = aVals[1].LengthSquared();
  if (dTanLengSq < SM_EFF_ZERO_SQ) { return SM_ERR;}  //  Zero first derivative unable to continue

  double dTanLengFourth = dTanLengSq * dTanLengSq;

  aPointAndFrenetFrameVectors[2] = (aVals[1] * aVals[2] * aVals[1]) / dTanLengFourth;
  if (lNumGeometricVectors == 2) { return SM_SUCCESS; }

  // save torsion vector

  double d2ndLeng = aVals[2].Length();
  if (SM_IS_ZERO(d2ndLeng)) { return SM_ERR; }  //  Zero 2nd derivative unable to continue
  //double d3rdLeng = aVals[3].Length();

  // First compute torsion value
  SmVector3d sCrossTan2nd = aVals[1] * aVals[2];
  double dCrossLeng = sCrossTan2nd.Length();
  double dDenominator = dCrossLeng * dCrossLeng;
  if (SM_IS_ZERO(dDenominator)) { return SM_ERR; }
  double dNumerator = smgu_Determinant3Vectors(aVals[1],aVals[2],aVals[3]);
  double dTorsion = dNumerator / dDenominator;

  SmVector3d sTorsionVector = aPointAndFrenetFrameVectors[1] * aPointAndFrenetFrameVectors[2];
  double dVecLength = sTorsionVector.Length() ;
  if(dVecLength < SM_EFF_ZERO) { return(SM_ERR) ; }
  sTorsionVector *= dTorsion / dVecLength ;
  aPointAndFrenetFrameVectors[3] = sTorsionVector;

  // all done
  return SM_SUCCESS;

} // end smgu_FrenetFromDerivatives

/*******************************************************************//**
PURPOSE: Find the continuity between two 1-dimensional evaluations

NOTES:  Given curve samples P(s), Ps(s), Pss(s), and optionally Psss(s)
                                 and Q(t), Qt(s), Qtt(t), and optionally Qttt(t)
  Cn
  C0 := DistanceBetween(P,Q)                            < dScaledZero = SM_EFF_ZERO * (1.0 + Max_Dimension(P,Q))
  C1 := C0 && NotZero(Ps,Qt) && DistanceBetween(Ps,Qt)  < dScaledZero = dParamEffZero * (1.0 + MaxLength(Ps,Qt))
  C2 := C1 && DistanceBetween(Pss,Qtt)                  < dScaledZero = dParamEffZero * (1.0 + MaxLength(Pss,Qtt))
  C3 := C2 && DistanceBetween(Psss,Qsss)                < dScaledZero = dParamEffZero * (1.0 + MaxLength(Psss,Qttt))
                                                          currently dParamEffZero hard coded to SM_ZONE_TOL_3D/10.0.
  Gn
  notes: given W(s) a parametric curve parameterized in arc length, s.
         1st ArcLength Deriv: dW/ds   = T                        T = unit tangent
         2nd ArcLength Deriv: d2W/ds2 = Nk,                      N = unit normal,   k = curvature
         3rd ArcLength Deriv: d3W/ds3 = k'N + k(-kT + tB),       B = unit binormal, t = torsion
                                                                 k' is rate of change of torsion
         G1: angleBetween(PT,QT) < dContinuityAngleTol
         G2: G1 and angleBetween(PN,QN) < dContinuityAngleTol and Pk/Qk within ratio tolerance := curvature continuous <=> G2
         G3: G2 and angleBetween(PB,QB) < dContinuityAngleTol and Pt/Qt within ratio tolerance := torsion continuous != G3
              and something for k'

  A torsion continuous curve does not guarantee a G3 curve because discontinuities in k' will
  yield non G3 surfaces.  I don't see a geometric test for this.  So the test for G3 will be two fold.
  Use geometric tests to determine torsion continuous and then use the algebraic test for
  the final G3 test.  The tolerance for that test will be estimated from the tolerances
  found in the lower order algebraic tests.

 // direct taylor series test for exact Gn continuity - not used because it did extend 
 //  well to tolerant checks
 //  G1 := DistanceBetween(Ps,a1*Qt)  < Ps.Length() * sin(dContinuityAngleTol)  // same as the geometry test
 //  G2 := DistanceBetween(Pss,a1**2*Qtt + a2*Qt) < tol
 //  G3 := DistanceBetween(Psss,a1**3*Qttt + 3*a1*a2*Qtt + a3*Qt) < tol


***********************************************************************/
SM_EXPORT SmStatus smgu_EvaluateCurveContinuity
(
  SmPoint3d        &rPos1,               // in : Eval 1 position 
  SmVector3d       &rVec1,               // in : Eval 1 tangent
  SmVector3d       &r2nd1,               // in : Eval 1 2nd derivative
  SmPoint3d        &rPos2,               // in : Eval 2 position      
  SmVector3d       &rVec2,               // in : Eval 2 tangent       
  SmVector3d       &r2nd2,               // in : Eval 2 2nd derivative
  SmContinuityType &rContinuityType,     // out: oneof SM_CT_DISCONTINUOUS
                                         //            SM_CT_C0           
                                         //            SM_CT_G1           
                                         //            SM_CT_G1_G2        
                                         //            SM_CT_G1_G2_G3        
                                         //            SM_CT_C1           
                                         //            SM_CT_C1_G2
                                         //            SM_CT_C1_G2_G3        
                                         //            SM_CT_C1_C2
                                         //            SM_CT_C1_C2_G3
                                         //            SM_CT_C1_C2_C3
   double           dContinuityAngleDeg, // in : max angle between G1 continuity, default:[SM_CONTINUITY_ANGLE]
   SmVector3d      *pOpt3rd1,            // in : Optional Eval 1 3rd Derivative, NULL to ignore, default:[NULL]
   SmVector3d      *pOpt3rd2             // in : Optional Eval 2 3rd Derivative, NULL to ignore, default:[NULL]
)            

{
  // init output
  rContinuityType = SM_CT_DISCONTINUOUS ;
  SmStatus   sRtn = SM_SUCCESS ;

  // function constants
  double dParamEffZero   = SM_ZONE_TOL_3D/10.0 ; // tolerance used for Cn continuity checks

  // locals
  double dScaledZero, dAngRad ;
  double a1=1.0, a2=1.0, a3=1.0 ;    
  double dG1Gap, dG2Gap, dG3Gap ;    
  double dPos1Max = rPos1.GetMaxDimension() ;
  double dVec1Max = rVec1.GetMaxDimension() ;
  double d2nd1Max = r2nd1.GetMaxDimension() ;
  double d3rd1Max = pOpt3rd1 ? pOpt3rd1->GetMaxDimension() : 1.0 ;
  double dPos2Max = rPos2.GetMaxDimension() ;
  double dVec2Max = rVec2.GetMaxDimension() ;
  double d2nd2Max = r2nd2.GetMaxDimension() ;
  double d3rd2Max = pOpt3rd2 ? pOpt3rd2->GetMaxDimension() : 1.0 ;

  double dPos12Max = smos_Max(dPos1Max, dPos2Max) ;
  double dVec12Max = smos_Max(dVec1Max, dVec2Max) ;
  double d2nd12Max = smos_Max(d2nd1Max, d2nd2Max) ;
  double d3rd12Max = smos_Max(d3rd1Max, d3rd2Max) ;

  // C0
  // when DistanceBetween(P,Q) < dScaledZero 
  // dScaledZero = SM_EFF_ZERO * (1.0 + Max_Dimension(P,Q))
  dScaledZero    = SM_EFF_ZERO * (1.0 + dPos12Max) ;
  double dPosGap = rPos1.DistanceBetween(rPos2) ;
  if(dPosGap > dScaledZero) { // not C0 - return SM_CT_DISCONTINUOUS
                              return sRtn ; 
                            }
  else                      { rContinuityType = SM_CT_C0 ; }

  // When Zero(Ps) - return SM_CT_C0
  dScaledZero        = SM_EFF_ZERO * (1.0 + dVec1Max);
  double dVec1Length = rVec1.Length() ;
  if(dVec1Length < dScaledZero) { // not G1 - return SM_CT_C0
                                  return sRtn ;
                                } 

  // When Zero(Qt) - return SM_CT_C0
  dScaledZero        = SM_EFF_ZERO * (1.0 + dVec2Max);
  double dVec2Length = rVec2.Length() ;
  if(dVec2Length < dScaledZero) { // not G1 - return SM_CT_C0
                                  return sRtn ; 
                                }                                          
  // G1
  // when C0 and angleBetween(tangent1, tangent2) < dContinuityAngleTol 
  SmVector3d sUnitVec1 = rVec1 ; sUnitVec1 /= dVec1Length ;
  SmVector3d sUnitVec2 = rVec2 ; sUnitVec2 /= dVec2Length ;
  sUnitVec1.AngleBetween(sUnitVec2, dAngRad) ;
  if(   sRtn == SM_ERR
     || (   SM_RAD2DEG(dAngRad) > dContinuityAngleDeg)) { // not G1 - return SM_CT_C0
                                                          return sRtn ; 
                                                        }
  else                                                  { rContinuityType = SM_CT_G1 ; }
                             
  // C1                      
  // when C0 and DistanceBetween(Ps,Qt)  < dScaledZero
  // dScaledZero  = dParamEffZero * (1.0 + MaxLength(Ps,Qt)) 
  dScaledZero     = dParamEffZero * (1.0 + dVec12Max) ;
  double dVecGap  = rVec1.DistanceBetween(rVec2) ;
  if(dVecGap < dScaledZero) { rContinuityType = SM_CT_C1 ; }

  // arrive here after passing either G1 or both G1 and C1 continuity tests

  // locals for Frenet Frame vectors
  ULONG lNumDerivs = (pOpt3rd1 && pOpt3rd2) ? 3 : 2 ; 
  SmVector3d aVals1[4], aFrenet1[4] ;
  SmVector3d aVals2[4], aFrenet2[4] ;
  aVals1[0] = rPos1 ;  aVals2[0] = rPos2 ;
  aVals1[1] = rVec1 ;  aVals2[1] = rVec2 ;
  aVals1[2] = r2nd1 ;  aVals2[2] = r2nd2 ;
  if(lNumDerivs == 3)
    { aVals1[3] = *pOpt3rd1 ; aVals2[3] = *pOpt3rd2 ; }

  // compute Frenet frame vectors from derivative inputs
  //      a[0] = position
  //      a[1] = 1st derivative,   Mag = Speed,   direction = tangent vector
  //      a[2] = Curvature Vector, Mag = K,       direction = normal vector
  //      a[3] = Torsion Vector,   Mag = Torsion, direction = binormal vector
  // note: Osculating circle center: center = pos + RadOfCurv * NormalVector
  //       center = a[0] + a[2]/a[2].LengthSquared(), when a[2].LengthSquared() is not zero.
  smgu_FrenetFromDerivatives(aVals1, lNumDerivs, aFrenet1) ; 
  smgu_FrenetFromDerivatives(aVals2, lNumDerivs, aFrenet2) ; 

  // G2
  //  when G2/C2 && Angle between normal vectors is less than continuity angle tolerance
  //         and Difference between radii of curvature and curvature are less than ratio tolerance
  double     dCurvature1    = aVals1[2].Length() ;
  double     dCurvature2    = aVals2[2].Length() ;
  double     dCurvatureGap  = dCurvature1 - dCurvature2 ;
             dAngRad        = 0.0 ;
  if(   !SM_IS_ZERO(dCurvature1) 
     && !SM_IS_ZERO(dCurvature2))
    {
      SmVector3d sNormal1 = aVals1[2] ; sNormal1 /= dCurvature1 ;
      SmVector3d sNormal2 = aVals2[2] ; sNormal2 /= dCurvature2 ;
      sNormal1.AngleBetween(sNormal1, dAngRad) ;
    }

  if(   (sRtn == SM_ERR)
     || !SM_IS_ZERO_TO_TOL(dCurvatureGap, SM_EFF_ZERO_SQRT/10.0) 
     || (SM_RAD2DEG(dAngRad) > dContinuityAngleDeg))
         { // not G2 - return SM_CT_G1 or SM_CT_C1
           return sRtn ; 
         }
  else   { rContinuityType =   rContinuityType == SM_CT_G1 
                             ? SM_CT_G1_G2                 
                             : SM_CT_C1_G2 ;               
         }                                                 

  // C2
  //  when C1 && DistanceBetween(Pss,Qtt) < dScaledZero
  // dScaledZero  = dParamEffZero * (1.0 + MaxLength(Pss,Qtt))
  if(rContinuityType == SM_CT_C1_G2)
    {
      dScaledZero    = dParamEffZero * (1.0 + d2nd12Max) ;
      double d2ndGap = r2nd1.DistanceBetween(r2nd2) ;
      if(d2ndGap < dScaledZero) { rContinuityType = SM_CT_C1_C2 ; }
    }

  // arrive here after passing G2 or both G2 and C2 continuity tests

  // G3 or C3
  if(pOpt3rd1 && pOpt3rd2)
    {
      // G3 when 3rd arcLength derivative is the same for both evaluations
      //   3rd ArcLength Derivative: 
      //     W' = T
      //     W'' = T' = kM
      //     W''' = k'M + k*(-k*T + t*b)
      // where W(s) = arclength parameterized curve
      //       T    = unit Tangent
      //       M    = unit normal
      //       B    = unit Binormal
      //       k    = curavture
      //       t    = torsion
      //  All of the above are functions of arcLength.  To be G3 a curve must
      //   be G2 (which implies T, M, and B continuity), curvature, torsion, and
      //   k' continuous.  I see how to check everything but k' continuity geometrically.
      //   So for now, I'm just going to check G3 algebraically by comparing the terms
      //   of the Taylor series expansion in the style of Barsky.
      //  Since we have already decided that the test is G2 then the G1 and G2 tests
      //  below are only being used to pick a tolerance for the G3 test.  This is
      //  a heuristic - there is no geometry based justification for this tolerance
      //  test, so feel free to change this if you see a better way to do it.

      // G1
      // seek tolerance and a1 such that DistanceBetween(Ps,a1*Qt) < dScaledZero 
      dScaledZero    = SM_ZONE_TOL_3D/10.0 * (1.0 + dVec12Max) ;
      smgu_LinePointDistance(SmPoint3d(0,0,0), rVec2, rVec1, dG1Gap, &a1) ;
      if(dScaledZero < dG1Gap) dScaledZero = dG1Gap ;

      // G2
      // seek tolerance and a2 such that DistanceBetween(Pss,a1**2*Qtt + a2*Qt) < dScaledZero 
      SmVector3d sG2Target = r2nd1 - a1*a1*r2nd2 ; 
      sRtn                 = smgu_LinePointDistance(SmPoint3d(0,0,0), rVec2, sG2Target, dG2Gap, &a2) ;
      if(dScaledZero < dG2Gap) dScaledZero = dG2Gap ; 

      // G3
      // seek a3 such that DistanceBetween(Psss,a1**3*Qttt + 3*a1*a2*Qtt + a3*Qt) < dScaledZero
      // dScaledZero = SM_ZONE_TOL_3D/10.0 * (1.0 + MaxLength(Ps,Qt,Pss,Qtt,Pss,Qttt)) 
      double dThisScaledZero = SM_ZONE_TOL_3D/10.0 * (1.0 + smos_3Max(dVec12Max, d2nd12Max, d3rd12Max)) ;
      dScaledZero            = smos_Max(dScaledZero, dThisScaledZero) ;
      SmVector3d sG3Target   = *pOpt3rd1 - a1*a1*a1*(*pOpt3rd2) - 3*a1*a2*r2nd2 ; 
      sRtn = smgu_LinePointDistance(SmPoint3d(0,0,0), rVec2, sG3Target, dG3Gap, &a3) ;
      if(   sRtn == SM_ERR
         || dG3Gap > dScaledZero) { // not G3 - return SM_CT_G1_G2, SM_CT_C1_G2, or SM_CT_C1_C2
                                    return sRtn ; 
                                  }
      else                        { rContinuityType =   rContinuityType == SM_CT_G1_G2 ? SM_CT_G1_G2_G3
                                                      : rContinuityType == SM_CT_C1_G2 ? SM_CT_C1_G2_G3
                                                      : rContinuityType == SM_CT_C1_C2 ? SM_CT_C1_C2_G3
                                                      : SM_CT_G1_G2_G3 ; 
                                  }

      // C3
      // when C2 && DistanceBetween(Psss,Qsss) < dScaledZero = dParamEffZero * (1.0 + MaxLength(Psss,Qttt))
      if(rContinuityType == SM_CT_C1_C2_G3)
        {
          dScaledZero    = dParamEffZero * (1.0 + d3rd12Max) ;
          double d3rdGap = (*pOpt3rd1).DistanceBetween(*pOpt3rd2) ;
          if(d3rdGap < dScaledZero) { rContinuityType = SM_CT_C1_C2_C3 ; }
        }
    } // end Opt 3rdVec existence check

  // arrive here for C3 or G3 continuity - that's as far as SMLib currently goes 
  // all done - return SM_CT_G1_G2_G3, SM_CT_C1_G2_G3, SM_CT_C1_C2_G3, or SM_CT_C1_C2_C3
  return(sRtn) ;

} // end smgu_EvaluateCurveContinuity

/*******************************************************************//**
PURPOSE: Compute and return the tolerant intersector classification
  for a pair of curve shape evaluations.

NOTES: 
   1. The input to these functions is the output from the call
      SmCurve::EvaluateGeometric(param, 3, bFromLeft, aPointAndFrenetFrame).

   2. When either curve evaluation is degenerate (the magnitude of the 1st 
        derivative is zero) the problem is classified as a point/point or 
        point/curve intersection and the output can only be
          SM_TC_UNKNOWN   = not intersecting
          SM_TC_CROSSING  = share a common point location (exactly or within tolerance)
          SM_TC_TANGENT   = intersecting and share a common  
***********************************************************************/
SM_EXPORT SmStatus smgu_ClassifyTolerantIntersection
(
  SmVector3d         aPointFrame1[4],     // in : SmCurve::EvaluateGeometric() output1
                                          //      a[0] = position   
                                          //      a[1] = 1st derivative,   Mag = Speed,   direction = tangent vector
                                          //      a[2] = Curvature Vector, Mag = K,       direction = normal vector
                                          //      a[3] = Torsion Vector,   Mag = Torsion, direction = binormal vector
                                          // note: Osculating circle center: center = pos + RadOfCurv * NormalVector, RadOfCurv = 1/K
                                          //       center = a[0] + a[2]/a[2].LengthSquared(), when a[2].LengthSquared() is not zero.
   SmIntervalPosition ePointType1,        // NotUsed: in : oneof SM_IP_START, SM_IP_INSIDE, SM_IP_END, SM_IP_OUTSIDE
                                          //
   SmVector3d         aPointFrame2[4],    // in : SmCurve::EvaluateGeometric() output2
   SmIntervalPosition ePointType2,        // NotUsed: in : oneof SM_IP_START, SM_IP_INSIDE, SM_IP_END, SM_IP_OUTSIDE
                                          //
   double             dTolerance3d,       // in : minimum 3d distance between distinct points 
                                          //
   SmTsectCurveType  &reTsectCurveType,   // out: oneof SM_TC_UNKNOWN         - not intersection
                                          //            SM_TC_CROSSING        - intersection with non-parallel tangents     
                                          //            SM_TC_TANGENT         - intersection with parallel tangents 
                                          //                                       and differing osculating circles      
                                          //            SM_TC_COINCIDENT      - intersection with coincident osculating circles  
   SmBoolean         &bTolerant,          // out: TRUE  = gap greater than scaled SM_EFF_ZERO
                                          //      FALSE = otherwise
   SmBoolean         &bAtMaxima           // out: TRUE  = Given points lie at actual intersection or at minimum or maximum
                                          //               of the curve/curve gap function.
                                          //               (zero Gap or Dot(Gap,Tangent1) = 0 and Dot(Gap,Tangent2) = 0)
                                          //      FALSE = otherwise
)
{
  SM_REF2(ePointType1, ePointType2) ;
  // init output
  reTsectCurveType = SM_TC_UNKNOWN ;
  bTolerant        = TRUE ;
  bAtMaxima        = FALSE ;
  SmStatus sRtn    = SM_SUCCESS ;

  // locals
  double     dScaledZero  = 0.0 ;
  double     dTangAngTol  = SM_TANGENT_ANGLE_RADIANS(dTolerance3d) ;
  SmVector3d sGap         = aPointFrame2[0] - aPointFrame1[0] ;
  double     dGap         = sGap.Length() ;
  double     dMag1        = aPointFrame1[1].Length() ;
  double     dMag2        = aPointFrame2[1].Length() ; 
  SmBoolean  bDegenerate1 = dMag1 < SM_EFF_ZERO * (1.0 + aPointFrame1[1].GetMaxDimension()) ;
  SmBoolean  bDegenerate2 = dMag2 < SM_EFF_ZERO * (1.0 + aPointFrame2[1].GetMaxDimension()) ;
  double     dGapAngRad1  = SM_PI/2.0 ;
  double     dGapAngRad2  = SM_PI/2.0 ;
  double     dTangAngRad  = 0.0 ;

  // at least crossing when dGap < dTolerance3d 
  if(dGap <= dTolerance3d)
    {  reTsectCurveType = SM_TC_CROSSING ; }

  // tolerant when dGap larger than scaled zero
  dScaledZero = (SM_EFF_ZERO * (1.0 + aPointFrame1[0].GetMaxDimension()
                                         + aPointFrame2[0].GetMaxDimension())) ;
  bTolerant = dGap > dScaledZero ;

  // tolerant intersections have gaps and gap angles
  if(bTolerant)
    {
      // GapAng1
      if(!bDegenerate1)
        { sGap.AngleBetween(aPointFrame1[1],dGapAngRad1) ; }

      // GapAng2
      if(!bDegenerate2)
        { sGap.AngleBetween(aPointFrame2[1],dGapAngRad2) ; }

      // AtMaxima when gap is perp to both tangent vectors
      bAtMaxima =    SM_IS_ZERO(dGapAngRad1 - SM_PI/2.0) 
                  && SM_IS_ZERO(dGapAngRad2 - SM_PI/2.0) ;

    } // end tolerant branch
  else // exact intersection
    {
      bAtMaxima = TRUE ;
    } // end exact branch

  // low work - not intersecting or degenerate curves
  if(   reTsectCurveType == SM_TC_UNKNOWN
     || bDegenerate1 
     || bDegenerate2)
    {
      // all done
      return(SM_SUCCESS) ;
    }

  // arrive here for a pair of NonDegenerate intersecting curve points
  // next: check for tangency

  // tangent angle
  aPointFrame1[1].AngleBetween(aPointFrame2[1],dTangAngRad) ;
  
  // at least tangent when tang angle is 0 or PI
  if(   dTangAngRad                  < dTangAngTol
     || smos_Fabs(dTangAngRad-SM_PI) < dTangAngTol)
    { reTsectCurveType = SM_TC_TANGENT ; }
  else
    {
      // all done
      return(SM_SUCCESS) ;
    }

  // arrive here for a pair of NonDegenerate intersecting tangent curve points
  // next: check for coincidence

#ifdef GWC
  long Gwc_smgu_ClassifyTolerantIntersectionNeedsImplementation ;
#endif

  // osculating circles - watch for lines
  double     dTiltAngRad=0.0;
  double     dK1           = aPointFrame1[2].Length() ;
  double     dK2           = aPointFrame2[2].Length() ;
                           
  SmBoolean  bLine1        = dK1 < SM_EFF_ZERO ;
  SmBoolean  bLine2        = dK2 < SM_EFF_ZERO ;
  double     dRadius1      = (!bLine1) ? 1.0/dK1 : SM_BIG_DOUBLE ;
  double     dRadius2      = (!bLine2) ? 1.0/dK2 : SM_BIG_DOUBLE ;
                           
  SmVector3d sTang1        = aPointFrame1[1] / dMag1 ;
  SmVector3d sTang2        = aPointFrame2[1] / dMag2 ;
  SmVector3d sNorm1        = (!bLine1) ? aPointFrame1[2] / dK1 : aPointFrame1[2] ;
  SmVector3d sNorm2        = (!bLine2) ? aPointFrame2[2] / dK2 : aPointFrame2[2] ;
                           
  SmVector3d sCenter1      = aPointFrame1[0] + dRadius1 * sNorm1 ;
  SmVector3d sCenter2      = aPointFrame2[0] + dRadius2 * sNorm2 ;
  SmVector3d sBiNorm1      = sTang1 * sNorm1 ;
  SmVector3d sBiNorm2      = sTang2 * sNorm2 ;

  // compute some circ/circle properties

  // circlePlane-circlePlane angle
  sBiNorm1.AngleBetween(sBiNorm2, dTiltAngRad) ;

  //      // quick check - assume 
  //      if(   SM_IS_ZERO_TO_TOL(dCenterGap + smos_Fabs(dRadius1 - dRadius2), dTolerance3d)

  // make it here when osculating circles may be coincident
  double dMaxGap = 0.0 ;
  SmPoint3d sPoint1, sPoint2 ;
  sRtn = smgu_CircleCircleApproxMaxGap(sCenter1, sBiNorm1, dRadius1,
                                       sCenter2, sBiNorm2, dRadius2,
                                       dMaxGap, &sPoint1, &sPoint2) ;

  // all done
  return( sRtn ) ;

} // end smgu_ClassifyTolerantIntersection

/*******************************************************************//**
PURPOSE: Find x values to solve  Ax**2 + Bx + C = 0

NOTES: The tolerance should typically be between 1.0e-8 and
    1.0e-12.  This method is taken from Graphics Gems I.
***********************************************************************/
SM_EXPORT SmStatus smgu_SolveQuadraticEqn                                 
(
  double  adCoefficients[3],              // in : coefficients, ordered:[C B A]                                
  double  dZeroTolerance,                 // in : Max allowed deviation from zero for a solution               
  ULONG & rNumSolutions,                  // out: 0 - imaginary roots                                          
                                          //      1 - a single double root                                     
                                          //      2 - two real roots                                           
  double  adSolutions[2]                  // out: Param values of zero crossings   
)
{
  // this is a wrapper function - pass the call along
  return( SmPolynomial::SolveQuadraticEqn(adCoefficients, dZeroTolerance, rNumSolutions, adSolutions) ) ;

} // end smgu_SolveQuadraticEqn
                                                                                                               
/*******************************************************************//**
PURPOSE: Find x values to solve  Ax**3 + Bx**2 + C + D = 0

NOTES: The tolerance should typically be between 1.0e-8 and
    1.0e-12.  This method is taken from Graphics Gems I.
***********************************************************************/
SM_EXPORT SmStatus smgu_SolveCubicEqn                             
(
  double  adCoefficients[4],            // in : coefficients, ordered:[D C B A]                              
  double  dZeroTolerance,               // in : Max allowed deviation from zero for a solution               
  ULONG & rNumSolutions,                // out: Real zero crossing cnt (imaginary and mult-roots not counted)
  double  adSolutions[3]                // out: Param values of zero crossings
)                                                
{
  // this is a wrapper function - pass the call along
  return( SmPolynomial::SolveCubicEqn(adCoefficients, dZeroTolerance, rNumSolutions, adSolutions) ) ;

} // end smgu_SolveCubicEqn
                                                                                                               
/*******************************************************************//**
PURPOSE: Find x values to solve  Ax**4 + Bx**3 + Cx**2 + Dx + E = 0

NOTES: The tolerance should typically be between 1.0e-8 and
    1.0e-12.  This method is taken from Graphics Gems I.
***********************************************************************/
SM_EXPORT SmStatus smgu_SolveQuarticEqn                   
(
  double  adCoefficients[5],              // in : coefficients, ordered:[E D C B A]                            
  double  dZeroTolerance,                 // in : Max allowed deviation from zero for a solution               
  ULONG & rNumSolutions,                  // out: Real zero crossing cnt (imaginary and mult-roots not counted)
  double  adSolutions[4]                  // out: Param values of zero crossings   
)
{
  // this is a wrapper function - pass the call along
  return( SmPolynomial::SolveQuarticEqn(adCoefficients, dZeroTolerance, rNumSolutions, adSolutions) ) ;

} // end smgu_SolveQuarticEqn

/*******************************************************************//**
PURPOSE: Find the surface first, second, and third directional derivatives 
            given a surface UV direction and a surface evaluation.

NOTES:
  When min(lHigestUDeriv,lHighestVDeriv) >= 1, compute 1st deriv
       min(lHigestUDeriv,lHighestVDeriv) >= 2, compute 1st and 2nd deriv
       min(lHigestUDeriv,lHighestVDeriv) >= 3, compute 1st, 2nd, and third deriv

  Dir  = unit vector in UV space specifying the direction
  Du   = 1st deriv in U,       Dv   = 1st deriv in V dir,
  Duu  = 2nd deriv in U,       Dvv  = 2nd deriv in V,    
  Duuu = third deriv in U      Dvvv = third deriv in V   
   
  Directional 1stDeriv =       Dir.x * Du 
                         +     Dir.y * Dv 
  Directonal  2ndDeriv =       Dir.x*Dir.x * Duu 
                         + 2 * Dir.x*Dir.y * Duv 
                         +     Dir.y*Dir.y * Dvv
  Directional 3rdDeriv =       Dir.x*Dir.x*Dir.x * Duuu 
                         + 3 * Dir.x*Dir.x*Dir.y * Duuv 
                         + 3 * Dir.x*Dir.y*Dir.y * Duvv 
                         +     Dir.y*Dir.y*Dir.y * Dvvv
***********************************************************************/
SmStatus smgu_DirectionalDerivs 
(
  SmVector2d sDir,             // i/o: directional UV vector, gets unitized. 
  ULONG lHighestUDeriv,        // in : number of U dirs in aDerivs, same as input argument to SmSurface::Evaluate
  ULONG lHighestVDeriv,        // in : number of V dirs in aDerivs, same as input argument to SmSurface::Evaluate
  SmVector3d *aDerivs,         // in : SmSurface::Evaluate output, 
                               //      2d organized: Du=[1][0], Dv=[0][1], Duu=[2][0], Duv=[1][1],Dvv=[0][2]
                               //      1d organized: Du=[lHighestVDeriv+1] , Dv=[1]
                               //                    Duu=[2(lHighestVDeriv+1)], Duv=[lHighestVDeriv+2],Dvv=[2]
                               //                    Duuu=[3(lHighestVDeriv+1)], Duuv=[ 2(lHighestVDeriv+1)+1]
                               //                    Duvv=[lHighestVDeriv+3],Dvvv=[3]
  SmVector3d aDirDeriv[]       // out: aDirDeriv[0] = position
                               //      aDirDeriv[1] = 1st directional derivative in sDir direction
                               //      aDirDeriv[2] = 2nd directional derivative in sDir direction
                               //      aDirDeriv[3] = 3rd directional derivative in sDir direction
                               //      sized:[min(lHighestUDeriv,LHighestVderiv)+1]
)
{
  // assumes
  // aDirDeriv is sized:[min(lHighestUDeriv,LHighestVderiv)+1]

  // unitize sDir
  sDir.Unitize() ;

  // position
  aDirDeriv[0] =   aDerivs[0] ;

  // 1st dir deriv = dir.x * Du + dir.y * Dv
  if(   lHighestUDeriv >= 1
     && lHighestVDeriv >= 1)
    {
      aDirDeriv[1] =   sDir.x * aDerivs[lHighestVDeriv+1] 
                     + sDir.y * aDerivs[1] ; 
    }

  // 2nd dir deriv =       Dir.x*Dir.x * Duu
  //                 + 2 * Dir.x*Dir.y * Duv
  //                 +     Dir.y*Dir.y * Dvv
  if(   lHighestUDeriv >= 2
     && lHighestVDeriv >= 2)
    {
      aDirDeriv[2] =       sDir.x * sDir.x * aDerivs[2*(lHighestVDeriv+1)]
                     + 2 * sDir.x * sDir.y * aDerivs[1*(lHighestVDeriv+1)+1] 
                     +     sDir.y * sDir.y * aDerivs[2] ;
    }

  // 3rd dir deriv =       Dir.x*Dir.x*Dir.x * Duuu
  //                 + 3 * Dir.x*Dir.x*Dir.y * Duuv
  //                 + 3 * Dir.x*Dir.y*Dir.y * Duvv
  //                 +     Dir.y*Dir.y*Dir.y * Dvvv
  if(   lHighestUDeriv >= 3
     && lHighestVDeriv >= 3)
    {
      aDirDeriv[3] =       sDir.x * sDir.x * sDir.x * aDerivs[3*(lHighestVDeriv+1)]
                     + 3 * sDir.x * sDir.x * sDir.y * aDerivs[2*(lHighestVDeriv+1)+1] 
                     + 3 * sDir.x * sDir.y * sDir.y * aDerivs[1*(lHighestVDeriv+1)+2] 
                     +     sDir.y * sDir.y * sDir.y * aDerivs[3] ;
    }

  // all done
  return(SM_SUCCESS) ;

} // end smgu_DirectionalDerivs

/*******************************************************************//**
PURPOSE: Find the surface first and second directional derivatives 
         given two surface UV directions and a surface evaluation.

NOTES:
  UVVec1  = non-unit vector in UV space specifying the first direction and magnitude
  UVVec2  = the second direction and magnitude
  The input aDerivs and output aDirDerivs are both organized as the output matrix
  from SmSurface::Evaluate() for 2 derivatives:
    2d organized: Du=[1][0], Dv=[0][1], Duu=[2][0], Duv=[1][1],Dvv=[0][2]
    1d organized: lHighestDeriv == 2: Du=[3] , Dv=[1] Duu=[6], Duv=[4], Dvv=[2]

METHOD: Chain rule, same as smgu_DirectionalDerivs() for 1st and 2nd derivatives,
   except that the 2nd differentiation is in a different direction.
***********************************************************************/
SM_EXPORT SmStatus smgu_2ndDirectionalDerivs
(
  const SmVector2d & crUVVec1,  // in : 1st uv direction and magnitude
  const SmVector2d & crUVVec2,  // in : 2nd uv direction and magnitude
  const SmVector3d *aDerivs,    // in : SmSurface::Evaluate() output, organized as above for 2 derivs
        SmVector3d *aDirDerivs  // out: organized same as aDerivs
) 
{
  // Rename, for convenience:
  const double & dV1x = crUVVec1.x;
  const double & dV1y = crUVVec1.y;
  const double & dV2x = crUVVec2.x;
  const double & dV2y = crUVVec2.y;
  const SmVector3d & rSu  = aDerivs[3];
  const SmVector3d & rSv  = aDerivs[1];
  const SmVector3d & rSuu = aDerivs[6];
  const SmVector3d & rSuv = aDerivs[4];
  const SmVector3d & rSvv = aDerivs[2];

  SmVector3d sS1 = dV1x * rSu  +  dV1y * rSv;
  SmVector3d sS2 = dV2x * rSu  +  dV2y * rSv;

  SmVector3d sS11 =    dV1x*dV1x * rSuu
                   + 2*dV1x*dV1y * rSuv
                   +   dV1y*dV1y * rSvv;

  SmVector3d sS12 =    dV1x*dV2x * rSuu
                   + ( dV1x*dV2y + dV1y*dV2x ) * rSuv
                   +   dV1y*dV2y * rSvv;

  SmVector3d sS22 =    dV2x*dV2x * rSuu
                   + 2*dV2x*dV2y * rSuv
                   +   dV2y*dV2y * rSvv;

  // Load outputs.
  aDirDerivs[0] = aDerivs[0];
  aDirDerivs[3] = sS1;
  aDirDerivs[1] = sS2;
  aDirDerivs[6] = sS11;
  aDirDerivs[4] = sS12;
  aDirDerivs[2] = sS22;

  // Just for good measure:
  aDirDerivs[5] = aDirDerivs[7] = aDirDerivs[8] = SmVector3d(0,0,0);

  return SM_SUCCESS;

} // end smgu_2ndDirectionalDerivs

/*******************************************************************//**
PURPOSE: Find the volume first, second, and third directional derivatives 
            given a volume UVW direction and a volume evaluation.

NOTES:
  When lHighestDeriv >= 1, compute 1st deriv
       lHighestDeriv >= 2, compute 1st and 2nd deriv
       lHighestDeriv >= 3, compute 1st, 2nd, and third deriv

  Dir  = unit vector in UVW space specifying the direction
  Du   = 1st deriv in U,       Dv   = 1st deriv in V dir,  Dw   = 1st deriv in W dir,
  Duu  = 2nd deriv in U,       Dvv  = 2nd deriv in V,      Dww  = 2nd deriv in W,    
  Duuu = third deriv in U      Dvvv = third deriv in V     Dwww = third deriv in W   
   
  Directional 1stDeriv =       Dir.x * Du 
                         +     Dir.y * Dv 
                         +     Dir.z * Dw
  Directonal  2ndDeriv =       Dir.x*Dir.x * Duu 
                         + 2 * Dir.x*Dir.y * Duv 
                         + 2 * Dir.x*Dir.z * Duw
                         +     Dir.y*Dir.y * Dvv
                         + 2 * Dir.y*Dir.z * Dvw
                         +     Dir.z*Dir.z * Dzz
  Directional 3rdDeriv =       Dir.x*Dir.x*Dir.x * Duuu 
                         + 3 * Dir.x*Dir.x*Dir.y * Duuv 
                         + 3 * Dir.x*Dir.x*Dir.z * Duuw 
                         + 3 * Dir.x*Dir.y*Dir.y * Duvv 
                         + 6 * Dir.x*Dir.y*Dir.z * Duvw 
                         + 3 * Dir.x*Dir.z*Dir.z * Duww 
                         +     Dir.y*Dir.y*Dir.y * Dvvv
                         + 3 * Dir.y*Dir.y*Dir.z * Dvvw
                         + 3 * Dir.y*Dir.z*Dir.z * Dvww
                         +     Dir.z*Dir.z*Dir.z * Dwww
***********************************************************************/
SM_EXPORT SmStatus smgu_VolDirectionalDerivs
(
  SmVector3d sDir,                       // i/o: directional UVW vector, gets unitized. 
  ULONG lHighestDeriv,                   // in : 0-Pos Only, 1=Pos+1st Derivs, ..., max 3
  SmVector3d *aDerivs,                   // in : matrix of OutSpace evaluations values
                                         //      3d organized: D[u][v][w]
                                         //      1d organized: D[i], i = u*n*n+v*n+w, for lHighestDeriv from 0 to 3
                                         //      sized       : [n+1][n+1][n+1], where n=lHighesDeriv
                                         //      lHghDrv = 0,   sized: [1],      
                                         //        i=0          order: [D]
                                         //      lHghDrv = 1,   sized: [8]    
                                         //        i=u*4+v*2+w  order: [D  Dw  Dv  ---
                                         //                             Du --- --- ---]
                                         //      lHghDrv = 2,   sized: [27]   
                                         //        i=u*9+v*3+w  order: [D   Dw  Dww Dv  Dvw --- Dvv --- ---
                                         //                             Du  Duw --- Duv --- --- --- --- ---
                                         //                             Duu --- --- --- --- --- --- --- ---]
                                         //      lHghDrv = 3,   sized: [81]   
                                         //        i=u*16+v*4+w order: [D   Dw   Dww   Dwww  Dv   Dvw  Dvww ---  Dvv  Dvw
                                         //                             --- ---  Dvvv  ---   ---  ---  Du   Duw  Duww ---
                                         //                             Duv Duvw ---   ---   Duvv ---  ---  ---  ---  ---
                                         //                             --- ---  Duu   Duuw  ---  ---  Duuv ---  ---  --- 
                                         //                             --- ---  ---   ---   ---  ---  ---  ---  Duuu --- 
                                         //                             --- ---  ---   ---   ---  ---  ---  ---  ---- --- 
                                         //                             --- ---  ---   ---   ---  ---  ---  ---  ---- --- 
                                         //                             --- ---  ---   ---   ---  ---  ---  ---  ---- --- 
                                         //                             --- --- ]
  SmVector3d aDirDeriv[]                 // out: aDirDeriv[0] = position
                                         //      aDirDeriv[1] = 1st directional derivative in sDir direction
                                         //      aDirDeriv[2] = 2nd directional derivative in sDir direction
                                         //      aDirDeriv[3] = 3rd directional derivative in sDir direction
                                         //      sized:[min(lHighestUDeriv,LHighestVderiv)+1]
)
{
  // assumes aDirDeriv is sized:[lHighestDeriv*lHighestDeriv*lHighestDeriv]
  // For indexing into the volume pt/deriv array:
#define sv(u,v,w) ((u*(lHighestDeriv+1)+v)*(lHighestDeriv+1)+w)

  // unitize sDir
  sDir.Unitize() ;

  // position
  aDirDeriv[0] =   aDerivs[sv(0,0,0)] ;

  // 1st dir deriv = Dir.x * Du + Dir.y * Dv + Dir.z * Dw
  if(lHighestDeriv >= 1)
    {
      aDirDeriv[1] =   sDir.x * aDerivs[sv(1,0,0)] 
                     + sDir.y * aDerivs[sv(0,1,0)]   
                     + sDir.z * aDerivs[sv(0,0,1)] ; 
    }

  // 2nd dir deriv =       Dir.x*Dir.x * Duu 
  //                 + 2 * Dir.x*Dir.y * Duv 
  //                 + 2 * Dir.x*Dir.z * Duw
  //                 +     Dir.y*Dir.y * Dvv
  //                 + 2 * Dir.y*Dir.z * Dvw
  //                 +     Dir.z*Dir.z * Dzz
  if(lHighestDeriv >= 2)
    {
      aDirDeriv[2] =       sDir.x*sDir.x * aDerivs[sv(2,0,0)]   // Duu
                     + 2 * sDir.x*sDir.y * aDerivs[sv(1,1,0)]   // Duv
                     + 2 * sDir.x*sDir.z * aDerivs[sv(1,0,1)]   // Duw
                     +     sDir.y*sDir.y * aDerivs[sv(0,2,0)]   // Dvv
                     + 2 * sDir.y*sDir.z * aDerivs[sv(0,1,1)]   // Dvw
                     +     sDir.z*sDir.z * aDerivs[sv(0,0,2)] ; // Dzz ;
    }

  // 3rd dir deriv =       Dir.x*Dir.x*Dir.x * Duuu 
  //                 + 3 * Dir.x*Dir.x*Dir.y * Duuv 
  //                 + 3 * Dir.x*Dir.x*Dir.z * Duuw 
  //                 + 3 * Dir.x*Dir.y*Dir.y * Duvv 
  //                 + 6 * Dir.x*Dir.y*Dir.z * Duvw 
  //                 + 3 * Dir.x*Dir.z*Dir.z * Duww 
  //                 +     Dir.y*Dir.y*Dir.y * Dvvv
  //                 + 3 * Dir.y*Dir.y*Dir.z * Dvvw
  //                 + 3 * Dir.y*Dir.z*Dir.z * Dvww
  //                 +     Dir.z*Dir.z*Dir.z * Dwww
  if(lHighestDeriv >= 3)
    {
      aDirDeriv[3] =       sDir.x*sDir.x*sDir.x * aDerivs[sv(3,0,0)]   // Duuu 
                     + 3 * sDir.x*sDir.x*sDir.y * aDerivs[sv(2,1,0)]   // Duuv 
                     + 3 * sDir.x*sDir.x*sDir.z * aDerivs[sv(2,0,1)]   // Duuw 
                     + 3 * sDir.x*sDir.y*sDir.y * aDerivs[sv(1,2,0)]   // Duvv 
                     + 6 * sDir.x*sDir.y*sDir.z * aDerivs[sv(1,1,1)]   // Duvw 
                     + 3 * sDir.x*sDir.z*sDir.z * aDerivs[sv(1,0,2)]   // Duww 
                     +     sDir.y*sDir.y*sDir.y * aDerivs[sv(0,3,0)]   // Dvvv
                     + 3 * sDir.y*sDir.y*sDir.z * aDerivs[sv(0,2,1)]   // Dvvw
                     + 3 * sDir.y*sDir.z*sDir.z * aDerivs[sv(0,1,2)]   // Dvww
                     +     sDir.z*sDir.z*sDir.z * aDerivs[sv(0,0,3)] ; // Dwww
    }                                          

#undef sv  
                                             
  // all done
  return(SM_SUCCESS) ;

} // end smgu_VolDirectionalDerivs

/*******************************************************************//**
PURPOSE: Return TRUE when 4 point sequence forms a "dog-leg" shape

NOTES: a dog leg is a 'z' shape in which the polygon running through
 the 4 points starts in one direction, makes a sharp turn, then
 another sharp turn to get back to running roughly in the same direction
***********************************************************************/
SM_EXPORT SmBoolean smgu_IsDogLeg
(
  SmVector3d & rP0,            // in : P0 of seq:[P0 P1 P2 P3] to check
  SmVector3d & rP1,            // in : P1 of seq:[P0 P1 P2 P3] to check
  SmVector3d & rP2,            // in : P2 of seq:[P0 P1 P2 P3] to check
  SmVector3d & rP3,            // in : P3 of seq:[P0 P1 P2 P3] to check
  double       dBendAngDeg,    // in : Max AngDeg deviation from 90 degs to count as bends at P1 and P2, default:[35]
  double       dParallDevDeg   // in : Max AngDeg between Segs P1-P2 and P0-P3 to count as a large enough offset
                               //       (smaller angles require larger P1-P2 offsets to be a dog-leg) default:[45]
)
{
  // locals
  SmVector3d sBegVec = rP1 - rP0 ; 
  SmVector3d sMidVec = rP2 - rP1 ;
  SmVector3d sEndVec = rP3 - rP2 ;
  SmVector3d sTotVec = rP3 - rP0 ; 

  // when When BegVec and EndVec are kindof perp to MidVec
  //      and MidVec is kindof parallel to TotVec - this is a dog leg shape
  if(   (sBegVec.IsPerpendicularTo(sMidVec, dBendAngDeg))
     && (sEndVec.IsPerpendicularTo(sMidVec, dBendAngDeg))
     && (sTotVec.IsParallelTo     (sMidVec, dParallDevDeg)))
    {
      return(TRUE) ;
    } // end beg/end seg has dog leg shape

  return(FALSE) ;

} // end smgu_IsDogLeg

/*******************************************************************//**
PURPOSE: Return TRUE when 4 point sequence forms a "dog-leg" shape

NOTES: a dog leg is a 'z' shape in which the polygon running through
 the 4 points starts in one direction, makes a sharp turn, then
 another sharp turn to get back to running roughly in the same direction
***********************************************************************/
SM_EXPORT SmBoolean smgu_IsDogLeg        
(
  SmVector2d & rP0,            // in : P0 of seq:[P0 P1 P2 P3] to check
  SmVector2d & rP1,            // in : P1 of seq:[P0 P1 P2 P3] to check
  SmVector2d & rP2,            // in : P2 of seq:[P0 P1 P2 P3] to check
  SmVector2d & rP3,            // in : P3 of seq:[P0 P1 P2 P3] to check
  double       dBendAngDeg,    // in : Max AngDeg deviation from 90 degs to count as bends at P1 and P2, default:[35]
  double       dParallDevDeg   // in : Max AngDeg between Segs P1-P2 and P0-P3 to count as a large enough offset
                               //       (smaller angles require larger P1-P2 offsets to be a dog-leg) default:[45]
)
{
  // locals
  SmVector3d sBegVec = rP1 - rP0 ; 
  SmVector3d sMidVec = rP2 - rP1 ;
  SmVector3d sEndVec = rP3 - rP2 ;
  SmVector3d sTotVec = rP3 - rP0 ; 
  double     dBegEndAngRad ;
  sBegVec.AngleBetween(sEndVec, dBegEndAngRad) ; // range:[0 Pi]

  // when When BegVec and EndVec are kindof parallel to one another
  //      and  BegVec and EndVec are both kindof perp to MidVec
  //      and  MidVec is kindof parallel to TotVec - this is a dog leg shape
  if(   (SM_RAD2DEG(dBegEndAngRad) < dParallDevDeg)
     && (sBegVec.IsPerpendicularTo(sMidVec, dBendAngDeg))
     && (sEndVec.IsPerpendicularTo(sMidVec, dBendAngDeg))
     && (sTotVec.IsParallelTo     (sMidVec, dParallDevDeg)))
    {
      return(TRUE) ;
    } // end point sequence has dog leg shape check

  return(FALSE) ;

} // end smgu_IsDogLeg

/*******************************************************************//**
PURPOSE: Given a knot vector, a count of knots to insert, and an interval
    within the knot vector, calculate the knot values to insert.

NOTES: 
   Will not create any multiple knots.
   Will insert at the limits of the given interval if they do not
   coincide with any original knots.
   sIvl is assumed to be contained within sKnotVec, but not checked.
***********************************************************************/
SM_EXPORT SmStatus smgu_InsertKnotsIntoKnotVector
  ( int                        iNumToInsert,  // in: how many knots to insert
    SmTArray< double > const & sKnotVec,      // in: existing knot vector, no mults
    SmExtent1d         const & sIvl,          // in: insert within this range
    SmTArray< double >       & sKnotsToInsert // out: values to be inserted
  )
{
  // Init output
  sKnotsToInsert.ReSet();
  if ( iNumToInsert < 1 ) { return SM_SUCCESS; }

  ULONG lNumOrigKnots = sKnotVec.GetSize();
  if ( lNumOrigKnots < 2 ) { return SM_ERR_INVALID_INPUT; }

  ULONG i, j;
  SmBoolean bProcessedLo = FALSE;

  // Set up a local knot array that starts and ends with knots
  // at the ends of the given interval, and contains all of the
  // existing knots that are in between those.
  // We will use that array to apportion the knots to be added
  // into the spans of the original knot vector.

  SmTArray< double > sLocalKnots;

  for( i = 0; i < sKnotVec.GetSize(); i++ )
  {
      double dThisKt = sKnotVec[i];

      // Tolerance for too-close knots:
      double dFudge =
            ( i == 0 )               ? ( sKnotVec[i+1] - sKnotVec[ i ] ) / 50
          : ( i >= lNumOrigKnots-1 ) ? ( sKnotVec[ i ] - sKnotVec[i-1] ) / 50
                                     : ( sKnotVec[i+1] - sKnotVec[i-1] ) / 100;

      // Knots below the interval of interest: skip.
      if ( dThisKt < sIvl.GetMin() - dFudge )
          continue;

      // Check whether this knot has passed the low end of the interval.
      // If bottom of interval of interest is between existing knots,
      // add one there.
      if ( bProcessedLo == FALSE && sIvl.GetMin() < dThisKt - dFudge )
      {
          sKnotsToInsert.Add( sIvl.GetMin() );
          sLocalKnots.Add   ( sIvl.GetMin() );
          iNumToInsert--;
          bProcessedLo = TRUE;
          continue;
      }

      // Check whether this knot has passed the high end of the interval.
      // If top of interval of interest is between existing knots,
      // add one there.
      if ( sIvl.GetMax() < dThisKt - dFudge )
      {
          sKnotsToInsert.Add( sIvl.GetMax() );
          sLocalKnots.Add( sIvl.GetMax() );
          iNumToInsert--;
          break; // we're done here.
      }

      // Knots within the interval of interest: just add to local list.
      if ( dThisKt < sIvl.GetMax() - dFudge )
      {
          sLocalKnots.Add( dThisKt );
          bProcessedLo = TRUE;
          continue;
      }

      // This knot is same as top of interval: add it and quit.
      sLocalKnots.Add( sIvl.GetMax() );
      break;

  } // end loop setting up sLocalKnots.

  if ( iNumToInsert < 1 ) { return SM_SUCCESS; }

  // State:
  // - sLocalKnots starts and ends at the boundaries of sIvl,
  //   and contains all unique existing knots in between.
  // - sKnotsToInsert contains sIvl min and/or max, only if those values
  //   fall in between existing knots.
  // - iNumToInsert has been decremented, if sIvl min or max will be added.

  // Next: apportion the knots to be added in between the knot values
  // in sLocalKnots.  For each span, calculate its share based on the
  // relative length of that span.
  SmTArray< ULONG > sNumToAddPerSpan;

  // Note: this will probably not come out to be exact: the total number
  // apportioned for all spans will probably not add up to iNumToInsert.
  // In that case, correct it by adjusting the largest spans, where
  // the influence of adding/removing a knot will be the least.
  // So we'll make lists of spans sorted by relative size.
  SmTArray< double > sSortedSpans;
  SmTArray< ULONG  > sSpanSortIndices;

  int iTotalAdded = 0;

  double dDelta = sIvl.GetLength();

  // For each span in sLocalKnots,
  // figure how many points for the span.
  // Also make up a list of spans sorted on knot density, for later adjustment.

  ULONG lNumSpans = sLocalKnots.GetSize() - 1;
  for ( i = 0; i < lNumSpans; i++ )
  {
    double dFrac = ( sLocalKnots[i+1] - sLocalKnots[i] ) / dDelta;

    double dNumThisSpan = dFrac * iNumToInsert;

    // We will overestimate the count per span.
    // This has the advantage that we always know which
    // direction we have to adjust: always downward.
    // Set up up so that every span gets at least one: add 1.01.
    // (Casting double to ULONG truncates -- 'floor'.)
    // This also makes the overall density more even, when all is done.
    ULONG  lNumThisSpan = (ULONG)( dNumThisSpan + 1.01 );

    sNumToAddPerSpan.Add( lNumThisSpan );

    iTotalAdded += lNumThisSpan;

    // When we cull these (next step), we will remove from those
    // with the smallest span size first.
    // Sort from smallest to largest span size (TRUE argument).

    double dSpanSize = dFrac / ( lNumThisSpan + 1 );
    smgu_AddSorted( dSpanSize, i, TRUE, sSortedSpans, &sSpanSortIndices );

  } // end loop on spans, apportioning add-knot counts.

  // Check state.
  SM_ASSERT( lNumSpans == sSortedSpans.GetSize() );

  // Now adjust the knots-to-add-per-span to match the input.
  ULONG lKnotCountExcess = iTotalAdded - iNumToInsert;

  // I don't think that this is possible:
  if ( iTotalAdded < iNumToInsert )
  {
      SM_DBG_WARN( _T("Unexpected bahavior in SmBSplineSurface::AddKnots() "));

      lKnotCountExcess = 0;
  }

  // If too many knots, subtract from the densest spans (smallest span size).

  for ( i = 0; i < lKnotCountExcess; i++ )
  {
      j = i % lNumSpans; // Can have more extra knots than there are spans.
      ULONG lSpanIdx = sSpanSortIndices[ j ];
      sNumToAddPerSpan[ lSpanIdx ] -= 1;
  }

  // Now calculate the parameter values for the new knots to be added.
  for ( i = 0; i < lNumSpans; i++ )
  {
      SmExtent1d sThisSpan( sLocalKnots[i], sLocalKnots[i+1] );
      ULONG lNumThisSpan = sNumToAddPerSpan[i];

      for ( j = 0; j < lNumThisSpan; j++ )
      {
          double dFrac = (double)(j+1) / (double)(lNumThisSpan+1);
          double dKnotToInsert = sThisSpan.Evaluate( dFrac );
          smgu_AddSorted( dKnotToInsert, 0, TRUE, sKnotsToInsert );
      }
  }

  return SM_SUCCESS;

} // end smgu_InsertKnotsIntoKnotVector

/*******************************************************************//**
PURPOSE: Find the Greville abscissa corresponding to a given
   control point in a B-Spline.

NOTES: 
   The knot values (breakpoints) in a B-Spline do not have any real
   correspondence to the control points.  There is, however, a best
   corresponding parameter value (not a knot) that corresponds to a
   control point.  It's called the Greville abscissa.

   Returns error, and return value 0, only if the array is too small.
***********************************************************************/
SM_EXPORT SmStatus smgu_GetGrevilleAbscissa
( 
  ULONG                      lIndex,          // in: index of control point, 0 .. nPts-1.
  SmTArray< double > const & rKnotVec,        // in:
  ULONG                      lDegree,         // in:
  double                   & rdGrevilleParam  // out: param value for the control point
)
{
  rdGrevilleParam = 0.0;

  if ( lIndex + lDegree >= rKnotVec.GetSize() ) { return SM_ERR_INVALID_INPUT; }

  // The Grevilla abscissa is the average of 'degree' knot values.
  ULONG ii;
  for ( ii = 1; ii <= lDegree; ii++ )
  {
      rdGrevilleParam += rKnotVec[ lIndex + ii ];
  }
  rdGrevilleParam /= lDegree;

  return SM_SUCCESS;

} // end smgu_GetGrevilleAbscissa

// -------------------------

// Sorts start here.

/*******************************************************************//**
PURPOSE: Add a value into a sorted array

NOTES: 
   The output arrays are assumed already to be in sorted order.
   If not, the results will not be sorted correctly either.
***********************************************************************/
SM_EXPORT SmStatus smgu_AddSorted
( 
  double             dValue,            // in : value to be sorted into array
  ULONG              lIdx,              // in : corresponding index of dValue, added to Indices
  SmBoolean          bIncreasing,       // in : True = sort in increasing order, else decreasing
  SmTArray<double> & rSortedValues,     // out: sorted
  SmTArray<ULONG>  * pSortIndices       // out: optional ordered indices into sorted array
)
{
  ULONG i, lNumValues = rSortedValues.GetSize();
  ULONG lFoundIdx = lNumValues;
  SmBoolean bFound = FALSE;

  for ( i = 0; i < lNumValues; i++ )
  {
      bFound = ( bIncreasing ) ? rSortedValues[i] > dValue : rSortedValues[i] < dValue;
      if ( bFound )
      {
          lFoundIdx = i;
          break;
      }
  }

  rSortedValues.InsertAt( lFoundIdx, dValue );
  if ( pSortIndices != NULL )
  {
      pSortIndices->InsertAt( lFoundIdx, lIdx   );
  }

  return SM_SUCCESS;

} // end smgu_AddSorted

/*******************************************************************//**
PURPOSE: Shell Sort Function for Ascending Order

NOTES: Sort a double array into ascending order
***********************************************************************/
SM_EXPORT void smgu_ShellSort_double_array 
(
  double *aDouble,                  // in : the array to sort                          
  ULONG   lCount                    // in : number of elements to sort in aDouble array
)                   
{
  ULONG ii ; 
  int flag = 1, d = lCount;
  double temp;

  while( flag || (d > 1))      // boolean flag (true when not equal to 0)
    {
      flag = 0;           // reset flag to 0 to check for future swaps
      d = (d+1) / 2;
      for (ii = 0; ii+d < lCount; ii++)   // can't say lCount - d
        {
          if (aDouble[ii + d] < aDouble[ii])
            {
              temp = aDouble[ii + d];      // swap positions ii+d and ii
              aDouble[ii + d] = aDouble[ii];
              aDouble[ii] = temp;
              flag = 1;                  // tells swap has occurred
            }
        }
    }

  return;

} // end smgu_ShellSort_double_array

/*******************************************************************//**
PURPOSE: Macro to Byte-wise swap two items of size SIZE.
NOTES: 
***********************************************************************/
#define SM_BYTE_SWAP(a, b, size)   \
  do{                                       \
      ULONG __size = (size);                \
      char *__a = (a), *__b = (b);          \
      do                                    \
      {                                     \
        char __tmp = *__a;                  \
        *__a++ = *__b;                      \
        *__b++ = __tmp;                     \
      } while (--__size > 0);               \
    } while (0)

/*******************************************************************//**
PURPOSE: Restore the sort order of an ordered array after
  one of the array's element values has been modified.

NOTES:  update pbase to be ordered after single element insertion
***********************************************************************/
SM_EXPORT SmStatus smgu_SortOneModified
 (const void * pbase,                     // in : the ordered array with one unsorted elem
  ULONG        beginning,                 // in : index of first element of pbase array to use in sort.  
  ULONG        total_elems,               // in : Size of array - must be greater than beggining
  ULONG        oldindex,                  // in : index of modified element.
  ULONG        size,                      // in : Size of element
  int   (*cmp)(const void*, const void*), // in : ptr to compare function
  char       * pworkbuff,                 // NotUsed: in : NO LONGER USED
  ULONG      & start,                     // out: index Start of modified region
  ULONG      & end)                       // out: index End of modified region
{
  SM_REF1(pworkbuff) ;
  char *base_ptr = (char *) pbase;

  // check input indices
  if(   oldindex < beginning
     || oldindex >= total_elems) 
    {
      // not within the range just ignore the request.
      start = oldindex;
      end   = oldindex;
      return SM_SUCCESS;
    }

  if (beginning == total_elems) 
    {
      start = oldindex;
      end = oldindex;
      return SM_SUCCESS;
    }
  int lSearchDirection = 0; // No search direction
  if (oldindex == beginning) 
    {
      lSearchDirection = 1; // Search forward only possible
    }
  else if (cmp(&base_ptr[oldindex*size],&base_ptr[(oldindex-1)*size]) == 1) 
    {
      lSearchDirection = 1; // Search forward only possible
    }
  else if (oldindex == total_elems-1) 
    {
      lSearchDirection = -1;
    }
  else if (cmp(&base_ptr[size*(oldindex+1)],&base_ptr[size*oldindex]) == 1) 
    {
      lSearchDirection = -1;
    }

  if (lSearchDirection == 1) 
    { // Forward search
      // Do a bindary search between oldindex+1 and total_elems to find
      // the position to move the element into.
      ULONG found_index = smgu_BinSearch(pbase,&base_ptr[size*oldindex],
                                         oldindex+1,total_elems,size,cmp);
      start = oldindex;
      end   = found_index;
      for (ULONG i=oldindex; i<found_index; i++) 
        {
          SM_BYTE_SWAP(&base_ptr[size*(i+1)],&base_ptr[size*i],size);
        }
    }
  else if (lSearchDirection == -1) 
    {
      ULONG found_index = smgu_BinSearch(pbase,&base_ptr[size*oldindex],
                                         beginning,oldindex-1,size,cmp);
      start = found_index;
      end = oldindex;
      for (ULONG i=0; i<oldindex-found_index; i++) 
        {
          SM_BYTE_SWAP(&base_ptr[size*(oldindex-i-1)],&base_ptr[size*(oldindex - i)],size);
        }
    }
  else 
    {
      start = oldindex;
      end  = oldindex;
    }

  // all done
  return SM_SUCCESS;

} // end smgu_SortOneModified

/*******************************************************************//**
PURPOSE: Do a binary search to find where an element should be 
   in an array. If there are ties it will go to the beginning of the
   tie area.

NOTES:  index marking pelem's place in the pbase ordered array
***********************************************************************/
SM_EXPORT ULONG smgu_BinSearch
(
  const void *pbase,                    // in : ptr to ordered array
  void *pelem,                          // in : Search element to be classified
  ULONG beginning,                      // in : index of first element of array to use in search.  Ignore previous stuff
  ULONG total_elems,                    // in : Size of array - or end of search + 1.
                                        //      must be greater than or equal to beggining
  ULONG size,                           // in : Size of element
  int (*cmp)(const void*, const void*)  // in : ptr to compare function
) 
{
  char *base_ptr = (char *) pbase;
  ULONG front=beginning;
  ULONG back=total_elems-1;
  ULONG middle = (front+back)/2;

  while (TRUE) 
    {
      int result = cmp(pelem,&base_ptr[size*middle]);
      if (result == 1) 
        {  // Keep moving up
          front = middle;
        }
      else if (result == -1) 
        {
          back = middle;
        }
      else 
        {
          return middle;
        }
      middle = (front + back ) / 2;
      if (middle == front) 
        {
          return middle;
        }
      if (middle == back) 
        {
          return middle;
        }
    }        
  return middle;

} // end smgu_BinSearch

/*******************************************************************//**
PURPOSE: helper function for SmTArray<SmPoint3d>::ShellSort(), GetMaxValue()
         and GetMinValue() methods

NOTES: rtn TRUE when (Pt1 < Pt2) after projecting Pts to common direction vec 
***********************************************************************/
SM_EXPORT SmBoolean smgu_Point3dLessThan    
(
  SmPoint3d   sElem1,           // in : sElem1 of sElem1 < sElem2
  SmPoint3d   sElem2,           // in : sElem2 of sElem1 < sElem2
  void      * pDirVec           // in : pDirVecType:[SmVector3d *], project dir - best when longest dir through point data
)                     
{
  SM_ASSERT(pDirVec!=NULL) ;

  double dVal1 = ((SmVector3d *)pDirVec)->Dot(sElem1) ;
  double dVal2 = ((SmVector3d *)pDirVec)->Dot(sElem2) ;

  return(dVal1 < dVal2) ;
} // end smgu_Point3dLessThan

/*******************************************************************//**
PURPOSE: helper function for SmTArray<SmPoint3d>::ShellSort(), GetMaxValue()
         and GetMinValue() methods

NOTES: rtn TRUE when (Pt1 < Pt2) after projecting Pts to common direction vec 
***********************************************************************/
SM_EXPORT SmBoolean smgu_Point3dLessThan    
(
  SmPoint3d * pElem1,           // in : sElem1 of sElem1 < sElem2
  SmPoint3d * pElem2,           // in : sElem2 of sElem1 < sElem2
  void      * pDirVec           // in : pDirVecType:[SmVector3d *], project dir - best when longest dir through point data
)                     
{
  SM_ASSERT(pDirVec!=NULL) ;

  double dVal1 = ((SmVector3d *)pDirVec)->Dot(*pElem1) ;
  double dVal2 = ((SmVector3d *)pDirVec)->Dot(*pElem2) ;

  return(dVal1 < dVal2) ;
} // end smgu_Point3dLessThan

/*******************************************************************//**
PURPOSE: helper function for SmTArray<SmPoint2d>::ShellSort(), GetMaxValue()
         and GetMinValue() methods

NOTES: rtn TRUE when (Pt1 < Pt2) after projecting Pts to common direction vec 
***********************************************************************/
SM_EXPORT SmBoolean smgu_Point2dLessThan    
(
  SmPoint2d   sElem1,           // in : sElem1 of sElem1 < sElem2
  SmPoint2d   sElem2,           // in : sElem2 of sElem1 < sElem2
  void      * pDirVec           // in : pDirVecType:[SmVector3d *], project dir - best when longest dir through point data
)                     
{
  SM_ASSERT(pDirVec!=NULL) ;

  double dVal1 = ((SmVector3d *)pDirVec)->Dot(sElem1) ;
  double dVal2 = ((SmVector3d *)pDirVec)->Dot(sElem2) ;

  return(dVal1 < dVal2) ;
} // end smgu_Point2dLessThan

/*******************************************************************//**
PURPOSE: helper function for SmTArray<SmPoint2d>::ShellSort(), GetMaxValue()
         and GetMinValue() methods

NOTES: rtn TRUE when (Pt1 < Pt2) after projecting Pts to common direction vec 
***********************************************************************/
SM_EXPORT SmBoolean smgu_Point2dLessThan    
(
  SmPoint2d * pElem1,           // in : sElem1 of sElem1 < sElem2
  SmPoint2d * pElem2,           // in : sElem2 of sElem1 < sElem2
  void      * pDirVec           // in : pDirVecType:[SmVector3d *], project dir - best when longest dir through point data
)                     
{
  SM_ASSERT(pDirVec!=NULL) ;

  double dVal1 = ((SmVector3d *)pDirVec)->Dot(*pElem1) ;
  double dVal2 = ((SmVector3d *)pDirVec)->Dot(*pElem2) ;

  return(dVal1 < dVal2) ;
} // end smgu_Point2dLessThan

/*******************************************************************//**
PURPOSE: helper function for SmTArray<double>::ShellSort(), GetMaxValue()
         and GetMinValue() methods

NOTES: rtn TRUE when (Dbl1 < Dbl2)
***********************************************************************/
SM_EXPORT SmBoolean smgu_DoubleLessThan   
 (double dVal1,                          // in : sElem1 of sElem1 < sElem2
  double dVal2,                          // in : sElem2 of sElem1 < sElem2
  void * pNotUsed)                       // NotUsed: in : default:[NULL]
{
  SM_REF1(pNotUsed) ; return (dVal1 < dVal2) ;
} // end smgu_DoubleLessThan

/*******************************************************************//**
PURPOSE: helper function for SmTArray<ULONG>::ShellSort(), GetMaxValue()
         and GetMinValue() methods

NOTES: rtn TRUE when (ULONG1 < ULONG2)
***********************************************************************/
SM_EXPORT SmBoolean smgu_ULONGLessThan
 (ULONG  lVal1,                           // in : sElem1 of sElem1 < sElem2
  ULONG  lVal2,                           // in : sElem2 of sElem1 < sElem2
  void * pNotUsed)                        // NotUsed: in : default:[NULL]
{ 
  SM_REF1(pNotUsed) ; return (lVal1 < lVal2) ; 
} // end smgu_ULONGLessThan

/*******************************************************************//**
PURPOSE: helper function for SmTArray<int>::ShellSort(), GetMaxValue()
         and GetMinValue() methods

NOTES: rtn TRUE when (ULONG1 < ULONG2)
***********************************************************************/
SM_EXPORT SmBoolean smgu_IntLessThan
 (int iVal1,                             // in : sElem1 of sElem1 < sElem2
  int iVal2,                             // in : sElem2 of sElem1 < sElem2
  void * pNotUsed)                       // NotUsed: in : default:[NULL]
{ 
  SM_REF1(pNotUsed) ; return (iVal1 < iVal2) ; 
} // end smgu_IntLessThan

/*******************************************************************//**
PURPOSE: helper function for SmTArray<double>::ShellSort(), GetMaxValue()
         and GetMinValue() methods

NOTES: rtn TRUE when (Dbl1 < Dbl2)
***********************************************************************/
SM_EXPORT SmBoolean smgu_FabsDoubleLessThan  
 (double dVal1,                             // in : sElem1 of sElem1 < sElem2
  double dVal2,                             // in : sElem2 of sElem1 < sElem2
  void * pNotUsed)                          // NotUsed: in : default:[NULL]
{ 
  SM_REF1(pNotUsed) ; return (smos_Fabs(dVal1) < smos_Fabs(dVal2)) ; 
} // end smgu_FabsDoubleLessThan

/*******************************************************************//**
PURPOSE: helper function for SmTArray<int>::ShellSort(), GetMaxValue()
         and GetMinValue() methods

NOTES: rtn TRUE when (ULONG1 < ULONG2)
***********************************************************************/
SM_EXPORT SmBoolean smgu_IabsIntLessThan
 (int iVal1,                            // in : sElem1 of sElem1 < sElem2
  int iVal2,                            // in : sElem2 of sElem1 < sElem2
  void * pNotUsed)                      // NotUsed: in : default:[NULL]
{ 
  SM_REF1(pNotUsed) ; return (smos_Iabs(iVal1) < smos_Iabs(iVal2)) ; 
} // end smgu_IabsIntLessThan

/*******************************************************************//**
PURPOSE: These are stub functions waiting GWC TODO

NOTES: 
***********************************************************************/
SM_EXPORT SmStatus smgu_CombineTolerantCircleIntersections  
 (const SmPoint3d  & crCircleCenter,          // NotUsed: in : circle center                        
  const SmVector3d & crCircleNormal,          // NotUsed: in : circle normal                        
  double             dCircleRadius,           // NotUsed: in : circle radius at XAxis               
  double             dTol3d,                  // NotUsed: in : min distance between distinct 3d Points
  ULONG            & rlPointCnt,              // NotUsed: i/o: number of points
  SmPoint3d          aPoints[2])              // NotUsed: i/o: array of points
{
  SM_REF6(crCircleCenter, crCircleNormal, dCircleRadius, dTol3d, rlPointCnt, aPoints) ;
  return SM_ERR;
} // end smgu_CombineTolerantCircleIntersections

/*******************************************************************//**
PURPOSE: These are stub functions waiting GWC TODO

NOTES: 
***********************************************************************/
SM_EXPORT SmStatus smgu_CombineTolerantPoints 
 (double            dTol3d,                   // NotUsed: in : min distance between distinct 3d Points
  ULONG           & rlPointCnt,               // NotUsed: i/o: number of points
  SmPoint3d         aPoints[2])               // NotUsed: i/o: array of points
{
  SM_REF3(dTol3d, rlPointCnt, aPoints) ;
  return SM_ERR;
}

/*******************************************************************//**
PURPOSE: These are stub functions waiting GWC TODO

NOTES: 
***********************************************************************/
SM_EXPORT SmStatus smgu_DropPointToEllipse()  
{
  return SM_ERR;
}

/*******************************************************************//**
PURPOSE: These are stub functions waiting GWC TODO

NOTES: 
***********************************************************************/
SM_EXPORT SmStatus smgu_DropPointToCircle() 
{
  return SM_ERR;
}
