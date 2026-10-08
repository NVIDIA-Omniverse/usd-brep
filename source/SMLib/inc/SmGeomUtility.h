// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*******************************************************************//**
* FILE NAME --- SmGeomUtility.h
* PURPOSE:  Header file for geometric utility intersection functions.
**********************************************************************/


#ifndef __SMGU_TSECT_H__
#define __SMGU_TSECT_H__

#include <SmVector3d.h>

class SmMatrix ;

/*******************************************************************//**
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

SM_EXPORT SmStatus smgu_PolygonArea
(
  const SmPoint3d  * sPoints,               // in : ordered array of points bounding polygon  
  ULONG              lNumPoints,            // in : number of points in sPoints               
  const SmVector3d & crNormal,              // in : unit normal to the polygon's plane        
  double           & rdPolygonArea          // out: Area of the polygon                       
);

SM_EXPORT SmStatus smgu_TriangleAreaSquared
(
  const SmPoint3d & rPoint0,                // in :       
  const SmPoint3d & rPoint1,                // in :       
  const SmPoint3d & rPoint2,                // in :       
  double          & rdTriAreaSquared        // out:       
);

SM_EXPORT SmStatus smgu_TriangleDecomposition
(                                                                                                              
  double dBaseLength,                      // in : triangle base length                                        
  double dLeftLegLength,                   // in : triangle left leg length                                    
  double dRightLegLength,                  // in : triangle right leg length                                   
  double & rdDistanceAlongBaseOfTopPoint,  // out: u coord of triangleTopPoint[u*BaseVec,v*PerpBaseVec]        
  double & rdHeightOfTopPoint              // out: v coord of triangleTopPoint[u*BaseVec,v*PerpBaseVec]        
                                           //    : in coordinate system centered on leftLeg/Base vertex        
                                           //    : with uVec pointing along the base line and                  
                                           //    : vVec perpendicular to that pointing towards the top point.  
);

SM_EXPORT SmStatus smgu_TrianglePointDistance
(
  const SmPoint3d & crPt0,                 // in : 'origin'        
  const SmPoint3d & crPt1,                 // in : x-axis point    
  const SmPoint3d & crPt2,                 // in : 3rd point       
  const SmPoint3d & crTestPt,              // in :                 
  double          & rdDistToTriangle,      // out:                 
  SmPoint3d       & rClosestPoint,         // out: see Usage Notes 
  double          & rdParamU,              // out: see Usage Notes 
  double          & rdParamV               // out: see Usage Notes 
);

SM_EXPORT SmStatus smgu_ParallelogramCrossVectors
(
  const SmVector3d & crBaseVector,              // in :   
  const SmVector3d & crSideVector,              // in :   
  double             dDistanceTopToBottom,      // in :   
  double             dDistanceBetweenSides,     // in :   
  SmVector3d       & rVecLowLeftToUpRight,      // out:   
  SmVector3d       & rVecLowRightToUpLeft       // out:   
);

SM_EXPORT SmStatus smgu_LinePointDistance
(
  const SmPoint3d  & crLinePnt,        // in : Point on line                                        
  const SmVector3d & crLineVec,        // in : any sized vector in tangent direction                
  const SmPoint3d  & crTestPoint,      // in : Point to test                                        
  double           & rdDistance,       // out: Min dist from crTestPoint to Line(LinePnt,Linevec)   
  double           * pdParam = NULL    // out: NULL to ignore                                       
);
 
SM_EXPORT SmStatus smgu_LinePointDistance
(
  const SmPoint2d  & crLinePnt,       // in : Point on line                                        
  const SmVector2d & crLineVec,       // in : any sized vector in tangent direction                
  const SmPoint2d  & crTestPoint,     // in : Point to test                                        
  double           & rdDistance,      // out: Min dist from crTestPoint to Line(LinePnt,Linevec)   
  double           * pdParam = NULL   // out: NULL to ignore                                       
);

SM_EXPORT SmStatus smgu_LineLineClosestPoint
(
  const SmPoint3d  & crLinePnt1,    // in : 1st line startPoint                                  
  const SmVector3d & crLineVec1,    // in : 1st line vector                                      
  const SmPoint3d  & crLinePnt2,    // in : 2nd line startPoint                                  
  const SmVector3d & crLineVec2,    // in : 2nd line vector                                      
  double           & rdLine1Param,  // out: nearestPt param on Line1 = Start1 + param * Vec1     
  double           & rdLine2Param   // out: nearestPt param on Line2 = Start2 + param * Vec2     
);

SM_EXPORT SmStatus smgu_LinePlaneIntersect
(
  const SmPoint3d  & crLinePnt,       // in : Line StartPoint                                
  const SmVector3d & crLineVec,       // in : Line Vector                                    
  const SmPoint3d  & crPlanePnt,      // in : Plane Point                                    
  const SmVector3d & crPlaneNorm,     // in : Plane Normal                                   
  double           & rdLineParam      // out: intersect param on Line = Start + param*Vec    
);

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
);

SM_EXPORT SmStatus smgu_LineLine2dIntersect
(
  const SmPoint2d  & crLinePnt1,      // in : 1st line startPoint                                   
  const SmVector2d & crLineVec1,      // in : 1st line vector                                       
  const SmPoint2d  & crLinePnt2,      // in : 2nd line startPoint                                   
  const SmVector2d & crLineVec2,      // in : 2nd line vector                                       
  double           & rdLine1Param,    // out: Intersect Pt param on Line1 = Start1 + param * Vec1   
  double           & rdLine2Param     // out: Intersect Pt param on Line2 = Start2 + param * Vec2   
);
 
SM_EXPORT SmStatus smgu_LineLineAtDistance
(
  const SmPoint3d  & crLinePnt1,      // in : Line 1 point                     
  const SmVector3d & crLineVec1,      // in : Line 1 NonZero Tangent           
  const SmPoint3d  & crLinePnt2,      // in : Line 2 point                     
  const SmVector3d & crLineVec2,      // in : Line 2 NonZero Tangent           
  double dAtDistance,                 // in : distance between solutions       
  double dLine1Params[4],             // out:                                  
  double dLine2Params[4]              // out:                                  
);

SM_EXPORT SmStatus smgu_LineClosestPoint
(
  const SmPoint3d  & crLinePnt,       // in : origin of the line                                                 
  const SmVector3d & crLineVec,       // in : direction vector of the line                                       
  const SmPoint3d  & crTestPoint,     // in : Point to project to line                                           
  double           & rdLineParameter  // out: closest param on line where line = crLinePnt + param * crLineVec   
);

SM_EXPORT SmStatus smgu_SegmentPointDistance
(
  const SmPoint3d & crLineStart,   // in : Segment Start                                      
  const SmPoint3d & crLineEnd,     // in : Segment End                                        
  const SmPoint3d & crTestPoint,   // in : TestPoint                                          
  double          & rdDistance,    // out: TestPoint/Segment distance                         
  double          & rdParameter    // out: 0.0 to 1.0, when value is 0 or 1, TestPoint may    
                                   //    : project to line point outside the segment          
);
                   
SM_EXPORT SmBoolean smgu_IsPointOnSegment
(                                                                                    
  const SmPoint3d & crLineStart,   // in : line start point                           
  const SmPoint3d & crLineEnd,     // in : line end point                             
  const SmPoint3d & crTestPoint,   // in : target point                               
  double            dTolerance,    // in : min dist between distinct points           
  double          & rdDistance,    // out: point/segment distance                     
  double          & rdParameter    // out: param of nearest line point                
                                   //    : of line = (1-param)*Start+(param)*End      
); 

SM_EXPORT SmStatus smgu_SegmentSegmentProjectedIntersect
(
  const SmPoint3d  & crStart1,                  // in :                                             
  const SmPoint3d  & crEnd1,                    // in :                                             
  const SmPoint3d  & crStart2,                  // in :                                             
  const SmPoint3d  & crEnd2,                    // in :                                             
  const SmVector3d & crPlaneNormal,             // in :                                             
  double             dDistanceTolerance,        // in : Tolerance to use in intersection            
  ULONG            & rlNumInt,                  // out: 0 - no intersections,                       
                                                //    : 1 - crossing or ends touching intersection  
                                                //    : 2 - coincident segments.                    
  double             aParams1[2],               // out:                                             
  double             aParams2[2],               // out:                                             
  double             aProjectedDistances[2]     // out:                                             
);

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
  ULONG            & rlNumInt,                   // out: 0 - no intersections,                       
                                                 //    : 1 - crossing or ends touching intersection  
                                                 //    : 2 - coincident segments.                    
  double             aParams1[2],                // out:                                             
  double             aParams2[2],                // out:                                             
  double             aProjectedDistances[2]      // out:                                             
);

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
                                      //    : FALSE= find furthest distance.                      
  double           & dDistance,       // out: min distance from point to circle segment           
  double           & dAngleDeg        // out: angle of closest point on circle segment            
);

SM_EXPORT SmStatus smgu_SegmentSegmentIntersect
(
  const SmPoint3d & crLineStart,        // in : 1st segment startPoint                     
  const SmPoint3d & crLineEnd,          // in : 1st segment endPoint                       
  const SmPoint3d & crOLineStart,       // in : 2nd segment startPoint                     
  const SmPoint3d & crOLineEnd,         // in : 2nd segment endPoint                       
  double            d3DTolerance,       // in : min distance between distinct points       
  ULONG           & rlNumIntersections, // out: number of intersections                    
  SmPoint3d         aPoints[2]          // out: intersection points (max of 2)             
);

SM_EXPORT SmStatus smgu_SegmentSegmentClosestPoint
(
  const SmPoint3d & crLineStart,          // in :                                                          
  const SmPoint3d & crLineEnd,            // in :                                                          
  const SmPoint3d & crOLineStart,         // in :                                                          
  const SmPoint3d & crOLineEnd,           // in :                                                          
  double            d3DTolerance,         // in :                                                          
  ULONG           & rlNumClosestPoints,   // out: If 1 then there is 1 closest point,                      
                                          //    : If 2 then segments are parallel and overlapping          
  double            adParamSeg1[2],       // out: Normalized parameter (between 0.0 and 1.0) for segment 1 
  double            adParamSeg2[2],       // out: Normalized paraemter for segment 2 of closest point(s)   
  double            adDistances[2]        // out:                                                          
);
                     
SM_EXPORT double smgu_SegmentPointAngleDegToPerp
(
  const SmPoint3d & crLineStart,                // in : Segment start point                          
  const SmPoint3d & crLineEnd,                  // in : segement end point                           
  const SmPoint3d & crTestPoint,                // in : target point                                 
  double & rNormalizedSegmentParam              // out: segment parameter closest to target point    
                                                //    : runs from 0.0 to 1.0.                        
);

SM_EXPORT SmStatus smgu_SegmentPoint2DSide
(
  const SmPoint3d & crStartPoint,               // in :                                            
  const SmPoint3d & crEndPoint,                 // in :                                            
  const SmPoint3d & crTestPoint,                // in :                                            
  ULONG           & rlSide                      // out: 0 - Point lies on line of segment          
                                                //    : 1 - Point lies on left side of segment     
                                                //    : 2 - Point lies on right side of segment    
);
  
SM_EXPORT SmStatus smgu_CircleFromPointsTangents
(
  const SmPoint3d  & sP1,                       // in :          
  const SmVector3d & sTan1,                     // in :          
  const SmPoint3d  & sP2,                       // in :          
  const SmVector3d & sTan2,                     // in :          
  double             dAngleTolDeg,              // in :          
  SmBoolean        & rbIsCircle,                // out:          
  SmAxis2Placement & rPlacement,                // out:          
  double           & rdRadius,                  // out:          
  double           & rdStartAngleDeg,           // out:          
  double           & rdEndAngleDeg              // out:          
);
  
SM_EXPORT SmStatus smgu_CircleFromPointsOneTangent
(
  const SmPoint3d  & crP0,                       // in :                             
  const SmVector3d & crT0,                       // in : Unit tangent at crP0.       
  const SmPoint3d  & crP1,                       // in :                             
  SmPoint3d        & rsCenter,                   // out:                             
  double           & rdRadius,                   // out:                             
  SmPoint3d        & rsBezierPoint,              // out: Middle Bezier point         
  double           & rdBezierWeight              // out: Weight for rsBezierPoint    
);

SM_EXPORT SmStatus smgu_CircleCenterFrom3Points
(
  const SmPoint2d & crPoint1,                   // in :        
  const SmPoint2d & crPoint2,                   // in :        
  const SmPoint2d & crPoint3,                   // in :        
  SmPoint2d       & rCenterOfCircle             // out:        
);

SmStatus smgu_CircleFrom3Points
(
  const SmPoint3d & cStartPoint,                // in : Circular Arc Start Point                                               
  const SmPoint3d & cMidPoint,                  // in : Circular Arc Mid Point                                                 
  const SmPoint3d & cEndPoint,                  // in : Circular Arc End Point                                                 
  SmPoint3d       & rCenter,                    // out: Center Point                                                           
  SmVector3d      & rXAxis,                     // out: XAxis pointing from CenterPoint to StartPoint                          
  SmVector3d      & rYAxis,                     // out: yAxis orthogonal in CCW direction to XAxis                             
  SmExtent1d      & rAnalDomain,                // out: Arc Domain in degrees from 0 to max of 360                             
  double          & dRadius,                    // out: Distance from CenterPoint to Circumference                             
  ULONG             lDimension = 3,             // in : dimensions of points and vectors [2 or 3]                              
  SmBoolean         bClosedCircle = FALSE       // in : TRUE = Return rAnalDomain for closed Circle                            
                                                //    : FALSE= Return rAnalDomain for CircularArc from StartPoint to EndPoint  
);

SM_EXPORT SmStatus smgu_CirclePlaneIntersect
(
  double             d3DTolerance,       // in :                                                                  
  double             dRadius,            // in : circle radius                                                    
  const SmPoint3d  & crCircleOrigin,     // in : circle center point                                              
  const SmVector3d & crCircleNormal,     // in : Circle Plane's Normal vector [unitized]                          
  const SmPoint3d  & crPlanePoint,       // in : plane point                                                      
  const SmVector3d & crPlaneNormal,      // in : Plane Normal Vector [Unitized]                                   
  ULONG            & rlNumTsect,         // out: 0 - no intersections                                             
                                         //    : 1 - grazing intersection                                         
                                         //    : 2 - standard two point intersection                              
                                         //    : 3 - indicates plane of circle is coincident with the intersection
                                         //    :     plane.  No points returned in aTsectPoints                   
  SmPoint3d          aTsectPoints[2]     // out:                                                                  
);

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
);

SM_EXPORT SmBoolean smgu_ConesAreDisjointPerpen
(
  const SmVector3d & crCone1Vec,      // in :              
  double             sCone1AngRad,    // in :              
  const SmVector3d & crCone2Vec,      // in :              
  double             sCone2AngRad     // in :              
);

// good for circle only - see next function for ellipses
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
);

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
);

SM_EXPORT SmStatus smgu_CircleCoPlanarCircleIntersect
(
  const SmVector3d & crCircleNormals,      // in : normal for both circles                     
  const SmPoint3d  & crCircle1Center,      // in : Circle1 CenterPoint                         
  double             dCircle1Radius,       // in : Circle1 Radius                              
  const SmPoint3d  & crCircle2Center,      // in : Circle2 CenterPoint                         
  double             dCircle2Radius,       // in : Circle2 Radius                              
  double             dTolerance,           // in : min distance between distinct 3d Points     
  ULONG            & rlNumIntersections,   // out: number of intersections                     
                                           //    : 0 = none                                    
                                           //    : 1 = tangent circles                         
                                           //    : 2 = intersecting circles                    
                                           //    : 3 = coincident circles                      
  SmPoint3d aIntersectionPoints[2]         // out: intersection points                         
);

SM_EXPORT SmStatus smgu_LineIntersectSphere
(
  const SmVector3d & crLinePnt,        // in :                                                                             
  const SmVector3d & crLineVec,        // in :                                                                             
  const SmVector3d & crSphereCenter,   // in :                                                                             
  double             dSphereRadius,    // in :                                                                             
  double             dTolerance,       // in :                                                                             
  ULONG            & rlNumberResults,  // out: rlNumberResults = 0 - no intersection, Line further than dTol from Sphere   
                                       //    : rlNumberResults = 1 - touching intersection, Line outside Sphere            
                                       //    :                              by less than dTol to touching (SM_EFF_ZERO)    
                                       //    : rlNumberResults = 2 - through intersecton, Line inside Sphere               
                                       //    :                              by more than dTol                              
                                       //    : rlNumberResults = 3 - grazing, Line inside Sphere by SM_EFF_TOL to dTol     
                                       //    :                   adResults[0] = 1st Intersection Point                     
                                       //    :                   adResults[1] = average Intersection Point on Line         
                                       //    :                   adResults[2] = 2nd Intersection Point                     
  double             adResults[3]      // out: line parameters of line/sphere intersections                                
);

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
                                       //    : 1 = 1 tangent intersection point              
                                       //    : 2 = 2 crossing intersection points            
                                       //    : 3 = coincident circles                        
  SmPoint3d         aPoints[2]         // out: Associated intersection points                
);
                      
SM_EXPORT SmStatus smgu_IntersectTwoPlanes
(
  const SmPoint3d  & crP1,                   // in : point on plane 1                                                    
  const SmVector3d & crV1,                   // in : unitNormal for plane 1                                              
  const SmPoint3d  & crP2,                   // in : point on plane 2                                                    
  const SmVector3d & crV2,                   // in : unitNormal for plane 2                                              
  SmPoint3d        & rLinePnt,               // out: point on intersection line (currently: halfway between projections  
                                             //    : of crP1 and crP2 onto intersection line.)                           
  SmVector3d       & rLineVec,               // out: tangent of intersection line                                        
  SmBoolean        * bOptCoincident = NULL   // out: TRUE = parallel and coincident planes NULL to ignore                
);

// rtn: SM_ERR_INVALID_INPUT for zero length normals or parallel planes (even coincident)
SM_EXPORT SmStatus smgu_IntersectThreePlanes
(
  const SmPoint3d  & crP1,       // in : point on plane 1                      
  const SmVector3d & crV1,       // in : UnitNormal for Plane 1                
  const SmPoint3d  & crP2,       // in : point on plane 2                      
  const SmVector3d & crV2,       // in : UnitNormal for Plane 2                
  const SmPoint3d  & crP3,       // in : point on plane 3                      
  const SmVector3d & crV3,       // in : UnitNormal for Plane 3                
  SmPoint3d        & rResult     // out: 3 plane intersection point            
);

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
);

// eff: project points along given normal to find plane and its extent
SM_EXPORT SmStatus smgu_PlaneFromPoints
(
  const SmPoint3d  * sPoints,            // in : the point set to examine                               
  ULONG              lNumPoints,         // in : number of points in sPoints                            
  const SmVector3d & rNormal,            // in : given normal                                           
  SmAxis2Placement & rAPlacement,        // out: X and Y axes of the point                              
  SmExtent2d       & rExt2d,             // out: Extent of plane that encompases all projected points   
  double           & rdMaxGap            // out: max displacement from the plane to the sPoints         
);

SM_EXPORT SmPoint3d smgu_PlaneEvaluatePoint
(
  const SmAxis2Placement & rPlane,      // in :          
  const SmPoint2d & crUV                // in :          
);

SM_EXPORT SmPoint2d smgu_PlaneDropPoint
(
  const SmAxis2Placement & rPlane,    // in :    
  const SmPoint3d        & cr3DPoint  // in :    
);

SM_EXPORT SmStatus smgu_PlanePointDistance
(
  const SmPoint3d  & crPlanePnt,        // in : PlanePt         of Plane(PlanePt, PlaneUnitNormal)             
  const SmVector3d & crPlaneNorm,       // in : PlaneUnitNormal of Plane(PlanePt, PlaneUnitNormal)             
  const SmPoint3d  & crTestPoint,       // in : TgtPoint to measure             
  double           & rdDistanceToPlane  // out: Distance from TgtPoint to Plane             
);

SM_EXPORT SmStatus smgu_PointProjectToPlane
(
  const SmPoint3d  & crPoint,          // in : target point                     
  const SmPoint3d  & crPlanePnt,       // in : point on plane                   
  const SmVector3d & crPlaneNorm,      // in : plane surface normal             
  SmPoint3d        & rProjectedPoint   // out: target point projected to plane  
);

SM_EXPORT SmStatus smgu_VectorProjectToPlane
(
  const SmVector3d & crVectorToProject,  // in : target vector                                             
  const SmVector3d & crPlaneNorm,        // in : plane normal                                              
  SmVector3d       & rProjectedVector    // out: component of target vector not in plane normal direction  
);

SmStatus smgu_TransformPointToStartPlane
(
  const SmPoint3d        & crPointToTransform, // in : target point                                                                                        
  const SmAxis2Placement & rPosition,          // in : rotate about Origin and ZAxis, measure angles from XAxis                                            
  const SmExtent1d       & rSweepIvlDeg,       // in : Interval of supported points (commonly [0 360], think SmEllipse and SmSurfOfRevolution AnalDomains) 
  double                   dDistTol3d,         // in : Dist3d when points are close enough to seams to return 2 answers                                    
  SmPoint3d              & rTransformedPoint,  // out: output point on the X/Z plane                                                                       
  double                 & rdDistToZAxis,      // out: distance to Z axis                                                                                  
  ULONG                  & rlNumAngles,        // out: 0 - point is on axis                                                                                
                                               //    : 1 - point is not on seam                                                                            
                                               //    : 2 - point is on seam of curve of revolution                                                         
  double                   adAnglesDeg[2],     // out: Angles in degrees to rotate rTransformedPoint back to original position                             
                                               //    : range:[m_vAnalDomain] or positive(0 to 360.0)                                                       
  SmBoolean              & bInside,            // out: TRUE = point is inside trim domain                                                                  
                                               //    : FALSE= point is outside trim domain                                                                 
  SmBoolean                bSnapToSeams        // in : TRUE = rtn 2 snapped values at seams, FALSE = rtn 1 exact and 1 snapped val at seams                
                                               //      TRUE=previous behavior, default:[TRUE]                                                              
);      

SM_EXPORT SmStatus smgu_PointPerspectiveToPlane
(
  const SmPoint3d  & crPoint,                 // in : target point                        
  const SmPoint3d  & crPlanePoint,            // in : point on plane                      
  const SmVector3d & crPlaneUnitNorm,         // in : plane surface unit-normal           
  const SmVector3d & crEyePoint,              // in : eye position                        
  SmPoint3d        & rProjectedPoint          // out: target point projected to plane     
);

SM_EXPORT SmStatus smgu_VectorPerspectiveToPlane
(
  const SmPoint3d  & crVectorBasePoint,       // in : target vector 3d base point                                
  const SmVector3d & crVectorToProject,       // in : target vector                                              
  const SmPoint3d  & crPlanePoint,            // in : plane point                                                
  const SmVector3d & crPlaneUnitNorm,         // in : plane surface unit-normal                                  
  const SmVector3d & crEyePoint,              // in : eye position                                               
  SmVector3d       & crProjectedVector        // out: component of target vector not in plane normal direction   
);

// Express a given vector as a linear combination of two basis vectors:
// solve for u and v such that
//  crVectorToProject = u * crBasisVec1  +  v * crBasisVec2
SM_EXPORT SmStatus smgu_VecLinCombTwoVectors 
( 
  const SmVector3d & crBasisVec1,          // in : vec1      of ProjR = u * Vec1 + v * Vec2    
  const SmVector3d & crBasisVec2,          // in : vec2      of ProjR = u * Vec1 + v * Vec2    
  const SmVector3d & crVectorToProject,    // in : vec R     of ProjR = u * Vec1 + v * Vec2    
        SmVector2d & rDUV                  // out: vec [u v] of ProjR = u * Vec1 + v * Vec2    
);

// eff: find geometric center and max gap for given point set
SM_EXPORT SmStatus smgu_BestFitPointToPoints
(
  SmTArray<SmPoint3d> const & crPoints,   // in : target point set                                                       
  double                      dTol3d,     // in : Min distance between distinct 3d points                                
  SmPointSetType            & eSetType,   // out: one of: SM_PST_VOID,          // empty point set                       
                                          //    :         SM_PST_POINTSIZED,    // all pts within tol of BestCenterPt    
                                          //    :         SM_PST_SCATTERED      // pts not within tol of target shape    
  SmVector3d                & rCenterPt,  // out: rCenterPt = 1/n * Sum_i(crPoints[i])                                   
  double                    & rdMaxGap    // out: max gap between crPoints[i] and rCenterPt                              
);

// eff: find least squares line fit to point set and max gap
SM_EXPORT SmStatus smgu_BestFitLineToPoints
(
  SmTArray<SmPoint3d> const & crPoints,         // in : target point set                                                                     
  double                      dTol3d,           // in : Min distance between distinct 3d points                                              
  SmPointSetType            & eSetType,         // out: one of: SM_PST_VOID,          // empty point set                                     
                                                //    :     SM_PST_POINTSIZED,    // all pts within tol of BestCenterPt                      
                                                //    :     SM_PST_LINEAR,        // all pts within tol of BestLine                          
                                                //    :     SM_PST_SCATTERED      // pts not within tol of BestLine - planar or scattered    
  SmVector3d                & rLinePt,          // out: found line point = geometric center of all crPoints                                  
  SmVector3d                & rLineVec,         // out: for SM_PST_LINEAR     = found best line tangent                                      
                                                //    : SM_PST_SCATTERED  = found best line tangent                                          
                                                //    : SM_PST_POINTSIZED = not used set:[0 0 0]                                             
                                                //    : SM_PST_VOID       = not used set:[0 0 0]                                             
  double                    & rdMaxLineGap,     // out: max gap SM_PST_POINTSIZED: between crPoints[i] and rCenterPt                         
                                                //    :     SM_PST_LINEAR    : between crPoints[i] and BestLine                              
                                                //    :     SM_PST_SCATTERED : between crPoints[i] and BestLine                              
  double                    & rdMaxCentroidGap, // out: max gap between crPoints[i] and rCenterPt                                            
  SmMatrix                  * pOptM = NULL      // out: optional memory for intermediate M(LinePt) matrix for efficiency of                  
                                                //    : smgu_BestFitPlaneToPoints,  sized:[3x3], NULL to ignore, default:[NULL]              
);

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
);

// rtn: SM_SUCCESS, else SM_ERR for void, point, or line PointSets
SM_EXPORT SmStatus smgu_BestFitPlaneToPoints
(
  SmTArray<SmPoint3d> const & crPoints,          // in : target point set                                                       
  double                      dTol3d,            // in : Min distance between distinct 3d points                                
  SmPointSetType            & eSetType,          // out: one of: SM_PST_VOID,          // empty point set                       
                                                 //    :         SM_PST_POINTSIZED,    // all pts within tol of BestCenterPt    
                                                 //    :         SM_PST_LINEAR,        // all pts within tol of BestLine        
                                                 //    :         SM_PST_PLANAR,        // all pts within tol of BestPlane       
                                                 //    :         SM_PST_SCATTERED      // pts not within tol of target shape    
  SmVector3d                & rPlanePt,          // out: found point on plane - geometric center of all crPoints                
  SmVector3d                & rPlaneNormal,      // out: for SM_PST_PLANAR     = found best plane normal                        
                                                 //    :     SM_PST_SCATTERED  = found best plane normal                        
                                                 //    :     SM_PST_LINEAR     = found best line tangent                        
                                                 //    :     SM_PST_POINTSIZED = not used set:[0 0 0]                           
                                                 //    :     SM_PST_VOID       = not used set:[0 0 0]                           
  double                    & rdMaxPlaneGap,     // out: max gap SM_PST_POINTSIZED: between crPoints[i] and rCenterPt           
                                                 //    :         SM_PST_LINEAR    : between crPoints[i] and BestLine            
                                                 //    :         SM_PST_PLANAR    : between crPoints[i] and BestPlane           
                                                 //    :         SM_PST_SCATTERED : between crPoints[i] and BestPlane           
  double                    & rdMaxCentroidGap   // out: max gap between crPoints[i] and rCenterPt                              
);

SM_EXPORT SmStatus smgu_SphereFrom4Points
(
  const SmPoint3d & crPoint0,       // in : 1st of 4 points                                
  const SmPoint3d & crPoint1,       // in : 2nd of 4 points                                
  const SmPoint3d & crPoint2,       // in : 3rd of 4 points                                
  const SmPoint3d & crPoint3,       // in : 4th of 4 points                                
  SmPoint3d       & rCenter,        // out: Center Point                                   
  double          & dRadius         // out: Distance from CenterPoint to Circumference     
);

SM_EXPORT SmStatus smgu_BoundingSphereFromPointSet
(
  const SmTArray<SmPoint3d> &rPoints,       // in : PointSet to examine                                                
  SmTArray<ULONG>           &rPtsOnSphere,  // out: indices of the points that define the minimum containing sphere    
  SmPoint3d                 &rCenter,       // out: Center of minimum radius circumscribing sphere                     
  double                    &dRadius        // out: radius of minimum radius circumscribing sphere                     
);

                                            // eff: compute dot(a,cross(b,c))
SM_EXPORT double smgu_VecTripleProduct
(
  const double *pA,                         // in : a of dot(a,cross(b,c))         
  const double *pB,                         // in : b of dot(a,cross(b,c))         
  const double *pC                          // in : c of dot(b,cropp(b,c))         
);
                 
SM_EXPORT void smgu_3x3Copy
(
  const double adMatFrom[3][3],             // in : double[3][3] array to copy     
  double       adMatTo[3][3]                // out: double[3][3] array to change   
);

SM_EXPORT void smgu_3x3Mult
(
  const double adMat1[3][3],                // in : Mat1   of Result = Mat1 * Mat2    
  const double adMat2[3][3],                // in : Mat2   of Result = Mat1 * Mat2    
  double       adMatResult[3][3]            // out: Result of Result = Mat1 * Mat2    
);
      
                                            // eff: compute 3x3 mat determinate
SM_EXPORT double smgu_3x3Determinate
(
  const double adA[3][3]                    // in : matrix    
);
                                                            
                                          // eff: compute 3x3 mat determinate
SM_EXPORT double smgu_3x3Determinate
(
  double a00, double a01, double a02,     // in : a oth row elements    
  double a10, double a11, double a12,     // in : a 1st row elements    
  double a20, double a21, double a22      // in : a 2nd row elements    
);

SM_EXPORT double smgu_Determinant3Vectors
(
  const SmVector3d & crV1,                // in :   
  const SmVector3d & crV2,                // in :   
  const SmVector3d & crV3                 // in :   
);                                         

                                          // eff: solve Ax=b when A is a 3x3 matrix
SM_EXPORT SmStatus smgu_3x3Solve
(
  const double       adA[3][3],           // in : A matrix         
  const SmVector3d & rB,                  // in : b of Ax = b      
  SmVector3d       & rX                   // out: x of Ax = b      
); 

SM_EXPORT SmStatus smgu_3x3Solve
(
  const double *A0,            // in : 0th iRow of A matrix, ordered:[00 01 02], sized:[3]    
  const double *A1,            // in : 1st iRow of A matrix, ordered:[10 11 12], sized:[3]    
  const double *A2,            // in : 2nd iRow of A matrix, ordered:[20 21 22], sized:[3]    
  const double *b,             // in : b of Ax = b, sized:[3]                                 
  double       *dX             // out: dX of Ax = b, sized:[3]                                
);
   
                                          // eff: Invert A when A is a 3x3 matrix
SM_EXPORT SmStatus smgu_3x3Inverse
(
  const double *A0,             // in : 0th iRow of A matrix, ordered:[00 01 02], sized:[3]    
  const double *A1,             // in : 1st iRow of A matrix, ordered:[10 11 12], sized:[3]    
  const double *A2,             // in : 2nd iRow of A matrix, ordered:[20 21 22], sized:[3]    
  double        adInv[3][3]     // out: inverse of A                                           
);
                                         
SM_EXPORT void smgu_4x4Copy
(
  const double adMatFrom[4][4],  // in : double[4][4] array to copy         
  double       adMatTo[4][4]     // out: double[4][4] array to change       
);
                                       
SM_EXPORT void smgu_4x4Mult
(
  const double adMat1[4][4],       // in : Mat1   of Result = Mat1 * Mat2    
  const double adMat2[4][4],       // in : Mat2   of Result = Mat1 * Mat2    
  double       adMatResult[4][4]   // out: Result of Result = Mat1 * Mat2    
);

                                          // eff: compute 4x4 mat determinate
SM_EXPORT double smgu_4x4Determinate
(
  const double A[4][4]    // in : matrix           
);

                                          // eff: Invert A when A is a 4x4 matrix
SM_EXPORT SmStatus smgu_4x4Inverse
(
  const double A[4][4],        // in : A matrix to invert   
  double       InvA[4][4]      // out: inverse of A         
);

SM_EXPORT SmBoolean smgu_4x4AreEqual
(
  const double adA[4][4],             // in : A of A == B                                           
  double       adB[4][4],             // out: B of A == B                                           
  double       dTol = SM_EFF_ZERO     // in : tolerance for equality check, default:[SM_EFF_ZERO]   
);
                                         
SM_EXPORT SmStatus smgu_SignedCurvature
(
  const SmVector3d &crWt,    // in : 1st Derivative                                   
  const SmVector3d &crWtt,   // in : 2nd Derivative                                   
  const SmVector3d &crN,     // in : Points to Osculating Circle Centers              
                             //    : which are given positive curvatures              
  double &dCurvature         // out: Curvature signed so that positive means          
                             //    : osculating circle center is in direction of crN  
);

SM_EXPORT SmStatus smgu_FrenetFromDerivatives
(
  SmVector3d aVals[],                        // in : aVals[0] = pos, aVals[1] = 1st deriv, aVals[2] = 2nd deriv, aVals[3] = 3rd deriv     
  ULONG lNumGeometricVectors,                // in : rtn, 0 = position, 1=tangent, 2=curvature_Normal, 3=Torsion_BiNormal                 
  SmVector3d aPointAndFrenetFrameVectors[]   // out: a[0] = position                                                                      
                                             //    : a[1] = 1st derivative,   Mag = Speed,   direction = tangent vector                   
                                             //    : a[2] = Curvature Vector, Mag = K,       direction = normal vector                    
                                             //    : a[3] = Torsion Vector,   Mag = Torsion, direction = binormal vector                  
                                             // note : Osculating circle center: center = pos + RadOfCurv * NormalVector, RadOfCurv = 1/K   
                                             //      : center = a[0] + a[2]/a[2].LengthSquared(), when a[2].LengthSquared() is not zero.    
);

// Find continuity between two 1-dimensional curve evaluations
SM_EXPORT SmStatus smgu_EvaluateCurveContinuity
(
  SmPoint3d        &rPos1,                                    // in : Eval 1 position                                  
  SmVector3d       &rVec1,                                    // in : Eval 1 tangent                                   
  SmVector3d       &r2nd1,                                    // in : Eval 1 2nd derivative                            
  SmPoint3d        &rPos2,                                    // in : Eval 2 position                                  
  SmVector3d       &rVec2,                                    // in : Eval 2 tangent                                   
  SmVector3d       &r2nd2,                                    // in : Eval 2 2nd derivative                            
  SmContinuityType &rContinuityType,                          // out: oneof SM_CT_DISCONTINUOUS                        
                                                              //    :       SM_CT_C0                                   
                                                              //    :       SM_CT_G1                                   
                                                              //    :       SM_CT_G1_G2                                
                                                              //    :       SM_CT_G1_G2_G3                             
                                                              //    :       SM_CT_C1                                   
                                                              //    :       SM_CT_C1_G2                                
                                                              //    :       SM_CT_C1_G2_G3                             
                                                              //    :       SM_CT_C1_C2                                
                                                              //    :       SM_CT_C1_C2_G3                             
                                                              //    :       SM_CT_C1_C2_C3                             
  double           dContinuityAngleDeg = SM_CONTINUITY_ANGLE, // in : max angle between G1 continuity                  
  SmVector3d      *pOpt3rd1 = NULL,                           // in : Optional Eval 1 3rd Derivative, NULL to ignore   
  SmVector3d      *pOpt3rd2 = NULL                            // in : Optional Eval 2 3rd Derivative, NULL to ignore   
);

SM_EXPORT SmStatus smgu_ClassifyTolerantIntersection
(
  SmVector3d         aPointFrame1[4],    // in : SmCurve::EvaluateGeometric() output1                                                   
                                         //    : a[0] = position                                                                        
                                         //    : a[1] = 1st derivative,   Mag = Speed,   direction = tangent vector                     
                                         //    : a[2] = Curvature Vector, Mag = K,       direction = normal vector                      
                                         //    : a[3] = Torsion Vector,   Mag = Torsion, direction = binormal vector                    
                                         // note: Osculating circle center: center = pos + RadOfCurv * NormalVector, RadOfCurv = 1/K     
                                         //     : center = a[0] + a[2]/a[2].LengthSquared(), when a[2].LengthSquared() is not zero.      
  SmIntervalPosition ePointType1,        // in : oneof SM_IP_START, SM_IP_INSIDE, SM_IP_END, SM_IP_OUTSIDE                              
  SmVector3d         aPointFrame2[4],    // in : SmCurve::EvaluateGeometric() output2                                                   
  SmIntervalPosition ePointType2,        // in : oneof SM_IP_START, SM_IP_INSIDE, SM_IP_END, SM_IP_OUTSIDE                              
  double             dTolerance3d,       // in : minimum 3d distance between distinct points                                            
  SmTsectCurveType  &reTsectCurveType,   // out: oneof SM_TC_UNKNOWN         - not intersection                                         
                                         //    :     SM_TC_CROSSING        - intersection with non-parallel tangents                    
                                         //    :     SM_TC_TANGENT         - intersection with parallel tangents                        
                                         //    :                                and differing osculating circles                        
                                         //    :     SM_TC_COINCIDENT      - intersection with coincident osculating circles            
  SmBoolean         &bTolerant,          // out: TRUE  = gap greater than scaled SM_EFF_ZERO                                            
                                         //    : FALSE = otherwise                                                                      
  SmBoolean         &bAtMaxima           // out: TRUE  = Given points lie at actual intersection or at minimum or maximum               
                                         //    :          of the curve/curve gap function.                                              
                                         //    :          (zero Gap or Dot(Gap,Tangent1) = 0 and Dot(Gap,Tangent2) = 0)                 
                                         //    : FALSE = otherwise                                                                      
);                                                                                 

                                          // eff: solve  Ax**2 + Bx + C = 0  
SM_EXPORT SmStatus smgu_SolveQuadraticEqn
(
  double  adCoefficients[3],              // in : coefficients, ordered:[C B A]                                
  double  dZeroTolerance,                 // in : Max allowed deviation from zero for a solution               
  ULONG & rNumSolutions,                  // out: 0 - imaginary roots                                          
                                          //    : 1 - a single double root                                     
                                          //    : 2 - two real roots                                           
  double  adSolutions[2]                  // out: Param values of zero crossings                     
);                                                                         
                                     
                                          // eff: solve  Ax**3 + Bx**2 + Cx + D = 0
SM_EXPORT SmStatus smgu_SolveCubicEqn
(
  double  adCoefficients[4],            // in : coefficients, ordered:[D C B A]                                
  double  dZeroTolerance,               // in : Max allowed deviation from zero for a solution                 
  ULONG & rNumSolutions,                // out: Real zero crossing cnt (imaginary and mult-roots not counted)  
  double  adSolutions[3]                // out: Param values of zero crossings                                 
);
                                                                         
                                          // eff: solve  Ax**4 + Bx**3 + Cx**2 + Dx + E = 0   
SM_EXPORT SmStatus smgu_SolveQuarticEqn
(
  double  adCoefficients[5],              // in : coefficients, ordered:[E D C B A]                              
  double  dZeroTolerance,                 // in : Max allowed deviation from zero for a solution                 
  ULONG & rNumSolutions,                  // out: Real zero crossing cnt (imaginary and mult-roots not counted)  
  double  adSolutions[4]                  // out: Param values of zero crossings                                 
);
                                         
                                          // eff: Find surf directional Derivs for a UV dir given UV Derivs
SmStatus smgu_DirectionalDerivs
(
  SmVector2d sDir,             // i/o: directional UV vector, gets unitized.                                       
  ULONG lHighestUDeriv,        // in : number of U dirs in aDerivs, same as input argument to SmSurface::Evaluate     
  ULONG lHighestVDeriv,        // in : number of V dirs in aDerivs, same as input argument to SmSurface::Evaluate     
  SmVector3d *aDerivs,         // in : SmSurface::Evaluate output,                                                    
                               //    : 2d organized: Du=[1][0], Dv=[0][1], Duu=[2][0], Duv=[1][1],Dvv=[0][2]          
                               //    : 1d organized: Du=[lHighestVDeriv+1] , Dv=[1]                                   
                               //    :               Duu=[2(lHighestVDeriv+1)], Duv=[lHighestVDeriv+2],Dvv=[2]        
                               //    :               Duuu=[3(lHighestVDeriv+1)], Duuv=[ 2(lHighestVDeriv+1)+1]        
                               //    :               Duvv=[lHighestVDeriv+3],Dvvv=[3]                                 
  SmVector3d aDirDeriv[]       // out: aDirDeriv[0] = position                                                        
                               //    : aDirDeriv[1] = 1st directional derivative in sDir direction                    
                               //    : aDirDeriv[2] = 2nd directional derivative in sDir direction                    
                               //    : aDirDeriv[3] = 3rd directional derivative in sDir direction                    
                               //    : sized:[min(lHighestUDeriv,LHighestVderiv)+1]                                   
);

SM_EXPORT SmStatus smgu_VolDirectionalDerivs
(
  SmVector3d sDir,                       // i/o: directional UVW vector, gets unitized.                                   
  ULONG lHighestDeriv,                   // in : 0-Pos Only, 1=Pos+1st Derivs, ..., max 3                                    
  SmVector3d *aDerivs,                   // in : matrix of OutSpace evaluations values                                       
                                         //    : 3d organized: D[u][v][w]                                                    
                                         //    : 1d organized: D[i], i = u*n*n+v*n+w, for lHighestDeriv from 0 to 3          
                                         //    : sized       : [n+1][n+1][n+1], where n=lHighesDeriv                         
                                         //    : lHghDrv = 0,   sized: [1],                                                  
                                         //    :   i=0          order: [D]                                                   
                                         //    : lHghDrv = 1,   sized: [8]                                                   
                                         //    :   i=u*4+v*2+w  order: [D  Dw  Dv  ---                                       
                                         //    :                        Du --- --- ---]                                      
                                         //    : lHghDrv = 2,   sized: [27]                                                  
                                         //    :   i=u*9+v*3+w  order: [D   Dw  Dww Dv  Dvw --- Dvv --- ---                  
                                         //    :                        Du  Duw --- Duv --- --- --- --- ---                  
                                         //    :                        Duu --- --- --- --- --- --- --- ---]                 
                                         //    : lHghDrv = 3,   sized: [81]                                                  
                                         //    :   i=u*16+v*4+w order: [D   Dw   Dww   Dwww  Dv   Dvw  Dvww ---  Dvv  Dvw    
                                         //    :                        --- ---  Dvvv  ---   ---  ---  Du   Duw  Duww ---    
                                         //    :                        Duv Duvw ---   ---   Duvv ---  ---  ---  ---  ---    
                                         //    :                        --- ---  Duu   Duuw  ---  ---  Duuv ---  ---  ---    
                                         //    :                        --- ---  ---   ---   ---  ---  ---  ---  Duuu ---    
                                         //    :                        --- ---  ---   ---   ---  ---  ---  ---  ---- ---    
                                         //    :                        --- ---  ---   ---   ---  ---  ---  ---  ---- ---    
                                         //    :                        --- ---  ---   ---   ---  ---  ---  ---  ---- ---    
                                         //    :                        --- --- ]                                            
  SmVector3d aDirDeriv[]                 // out: aDirDeriv[0] = position                                                     
                                         //    : aDirDeriv[1] = 1st directional derivative in sDir direction                 
                                         //    : aDirDeriv[2] = 2nd directional derivative in sDir direction                 
                                         //    : aDirDeriv[3] = 3rd directional derivative in sDir direction                 
                                         //    : sized:[min(lHighestUDeriv,LHighestVderiv)+1]                                
);
                                         
                                          // eff: Find surf 2nd directional Derivs for two UV dirs
SM_EXPORT SmStatus smgu_2ndDirectionalDerivs
(
  const SmVector2d & crUVVec1,            // in : 1st uv direction and magnitude                                       
  const SmVector2d & crUVVec2,            // in : 2nd uv direction and magnitude                                       
  const SmVector3d *aDerivs,              // in : SmSurface::Evaluate() output, organized as above for 2 derivs        
        SmVector3d *aDirDerivs            // out: organized same as aDerivs                                            
);

SM_EXPORT SmBoolean smgu_IsDogLeg         
  (SmVector3d & rP0,                      // in : P0 of seq:[P0 P1 P2 P3] to check                                            
   SmVector3d & rP1,                      // in : P1 of seq:[P0 P1 P2 P3] to check                                            
   SmVector3d & rP2,                      // in : P2 of seq:[P0 P1 P2 P3] to check                                            
   SmVector3d & rP3,                      // in : P3 of seq:[P0 P1 P2 P3] to check                                            
   double       dBendAngDeg=35.0,         // in : Max AngDeg deviation from 90 degs to count as bends at P1 and P2,           
   double       dParallDevDeg=45.0        // in : Max AngDeg between Segs P1-P2 and P0-P3 to count as a large enough offset   
                                          //    : (smaller angles require larger P1-P2 offsets to be a dog-leg)               
  );

SM_EXPORT SmBoolean smgu_IsDogLeg         
(
  SmVector2d & rP0,                      // in : P0 of seq:[P0 P1 P2 P3] to check                                          
  SmVector2d & rP1,                      // in : P1 of seq:[P0 P1 P2 P3] to check                                          
  SmVector2d & rP2,                      // in : P2 of seq:[P0 P1 P2 P3] to check                                          
  SmVector2d & rP3,                      // in : P3 of seq:[P0 P1 P2 P3] to check                                          
  double       dBendAngDeg=35.0,         // in : Max AngDeg deviation from 90 degs to count as bends at P1 and P2,         
  double       dParallDevDeg=45.0        // in : Max AngDeg between Segs P1-P2 and P0-P3 to count as a large enough offset 
                                         //    : (smaller angles require larger P1-P2 offsets to be a dog-leg)             
);

SM_EXPORT SmStatus  smgu_InsertKnotsIntoKnotVector
  ( int iNumToAdd,
    SmTArray< double > const & sKnotVec,      // in : existing knot vector, no mults                                       
    SmExtent1d         const & sIvl,          // in : insert within this range.                                            
    SmTArray< double >       & sKnotsToInsert // out: values to be inserted.                                               
  );

SM_EXPORT SmStatus smgu_GetGrevilleAbscissa
(
  ULONG                      lIndex,          // in : index of control point, 0 .. nPts-1.                                 
  SmTArray< double > const & rKnotVec,        // in :                                                                      
  ULONG                      lDegree,         // in :                                                                      
  double                   & rdGrevilleParam  // out: param value for the control point
);
                                                 
SM_EXPORT SmStatus smgu_AddSorted             // eff: insert one value in order into a sorted array
(
  double             dValue,                  // in : value to be sorted into array                            
  ULONG              lIdx,                    // in : corresponding index of dValue, added to Indices          
  SmBoolean          bIncreasing,             // in : True = sort in increasing order, else decreasing         
  SmTArray<double> & rSortedValues,           // out: sorted                                                   
  SmTArray<ULONG>  * pSortIndices = NULL      // out: optional ordered indices into sorted array               
);

                                          
SM_EXPORT void smgu_ShellSort_double_array    // eff: sort a double array into ascending order
(
  double *aDouble,                            // in : the array to sort                                  
  ULONG   lCount                              // in : number of elements to sort in aDouble array        
);
                                                   
                                         
SM_EXPORT SmStatus smgu_SortOneModified       // eff: update pbase to be ordered after single element insertion
(                                             
  const void *pbase,                          // in : the ordered array with one unsorted elem                 
  ULONG beginning,                            // in : index of first element of pbase array to use in sort.    
  ULONG total_elems,                          // in : Size of array - must be greater than beggining           
  ULONG oldindex,                             // in : index of modified element.                               
  ULONG size,                                 // in : Size of element                                          
  int( *cmp )(const void*, const void*),      // in : ptr to compare function                                  
  char * pworkbuff,                           // in : NO LONGER USED                                           
  ULONG & start,                              // out: index Start of modified region                           
  ULONG & end                                 // out: index End of modified region                             
);

                                         
SM_EXPORT ULONG smgu_BinSearch           // rtn: index marking pelem's place in the pbase ordered array
(
  const void *pbase,                     // in : ptr to ordered array                                                        
  void *pelem,                           // in : Search element to be classified                                             
  ULONG beginning,                       // in : index of first element of array to use in search.  Ignore previous stuff    
  ULONG total_elems,                     // in : Size of array - or end of search + 1.                                       
                                         //    : must be greater than or equal to beggining                                  
  ULONG size,                            // in : Size of element                                                             
  int( *cmp )(const void*, const void*)  // in : ptr to compare function                                                     
);

                                          
SM_EXPORT SmBoolean smgu_Point3dLessThan // eff: rtn TRUE when (Pt1 < Pt2) after projecting Pts to common direction vec
(
  SmPoint3d * pElem1,                    // in : pElem1 of pElem1 < sElem2                                                          
  SmPoint3d * pElem2,                    // in : pElem2 of pElem1 < sElem2                                                          
  void      * pDirVec                    // in : pDirVecType:[SmVector3d *], project dir - best when longest dir through point data 
);

SM_EXPORT SmBoolean smgu_Point3dLessThan // eff: rtn TRUE when (Pt1 < Pt2) after projecting Pts to common direction vec
(
  SmPoint3d   sElem1,                    // in : sElem1 of sElem1 < sElem2                                                          
  SmPoint3d   sElem2,                    // in : sElem2 of sElem1 < sElem2                                                          
  void      * pDirVec                    // in : pDirVecType:[SmVector3d *], project dir - best when longest dir through point data 
);

SM_EXPORT SmBoolean smgu_Point2dLessThan // eff: rtn TRUE when (Pt1 < Pt2) after projecting Pts to common direction vec
(
  SmPoint2d * pElem1,                    // in : pElem1 of pElem1 < sElem2                                                          
  SmPoint2d * pElem2,                    // in : pElem2 of pElem1 < sElem2                                                          
  void      * pDirVec                    // in : pDirVecType:[SmVector3d *], project dir - best when longest dir through point data 
);

SM_EXPORT SmBoolean smgu_Poin2dtLessThan // eff: rtn TRUE when (Pt1 < Pt2) after projecting Pts to common direction vec
(
  SmPoint2d   sElem1,                    // in : sElem1 of sElem1 < sElem2                                                          
  SmPoint2d   sElem2,                    // in : sElem2 of sElem1 < sElem2                                                          
  void      * pDirVec                    // in : pDirVecType:[SmVector3d *], project dir - best when longest dir through point data 
);

                                          
SM_EXPORT SmBoolean smgu_DoubleLessThan  // eff: rtn TRUE when (Dbl1 < Dbl2) 
(
  double dVal1,                          // in : sElem1 of sElem1 < sElem2      
  double dVal2,                          // in : sElem2 of sElem1 < sElem2      
  void * pNotUsed = NULL                 // NotUsed: in : 
);

                                          
SM_EXPORT SmBoolean smgu_ULONGLessThan   // eff: rtn TRUE when (ULONG1 < ULONG2)
(
  ULONG lVal1,                           // in : sElem1 of sElem1 < sElem2   
  ULONG lVal2,                           // in : sElem2 of sElem1 < sElem2   
  void * pNotUsed = NULL                 // NotUsed: in :
);

                                          // eff: rtn TRUE when (ULONG1 < ULONG2)
SM_EXPORT SmBoolean smgu_IntLessThan
(
  int iVal1,                             // in : sElem1 of sElem1 < sElem2       
  int iVal2,                             // in : sElem2 of sElem1 < sElem2       
  void * pNotUsed = NULL                 // NotUsed: in :
);

                                          // eff: rtn TRUE when (Dbl1 < Dbl2)
SM_EXPORT SmBoolean smgu_FabsDoubleLessThan
(
  double dVal1,                           // in : sElem1 of sElem1 < sElem2        
  double dVal2,                           // in : sElem2 of sElem1 < sElem2        
  void * pNotUsed = NULL                  // NotUsed: in :
);

                                         // eff: rtn TRUE when (ULONG1 < ULONG2)
SM_EXPORT SmBoolean smgu_IabsIntLessThan
(
  int iVal1,                             // in : sElem1 of sElem1 < sElem2        
  int iVal2,                             // in : sElem2 of sElem1 < sElem2        
  void * pNotUsed = NULL                 // NotUsed: in :
);

                                        // eff: UserData for smgu_PolyEdgeLessThan
struct SM_EXPORT SmEdgeLessThanData
{
  SmVector3d m_sDirVec;                  // -  : project dir - best when longest dir through point data
  ULONG      m_eSortByFlag=0;            // -  : SortBy: 0=MinParam, 1=MaxParam, 2=StartParam, 3=EndParam
};


// These are stub functions waiting GWC TODO
SM_EXPORT SmStatus smgu_CombineTolerantCircleIntersections  // GWC TODO
  (const SmPoint3d  & crCircleCenter,          // in : circle center                        
   const SmVector3d & crCircleNormal,          // in : circle normal                        
   double             dCircleRadius,           // in : circle radius at XAxis               
   double             dTol3d,                  // in : min distance between distinct 3d Points
   ULONG            & rlPointCnt,              // i/o: number of points
   SmPoint3d          aPoints[2]);             // i/o: array of points
                                             
SM_EXPORT SmStatus smgu_CombineTolerantPoints  // GWC TODO
  (double            dTol3d,                   // in : min distance between distinct 3d Points
   ULONG           & rlPointCnt,               // i/o: number of points
   SmPoint3d         aPoints[2]);              // i/o: array of points
                                             
SM_EXPORT SmStatus smgu_DropPointToEllipse() ;  // GWC TODO
SM_EXPORT SmStatus smgu_DropPointToCircle() ;   // GWC TODO


#endif // !__SMGU_TSECT_H__

