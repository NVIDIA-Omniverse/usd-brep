// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*******************************************************************//**
* FILE NAME --- SmTools.h
* PURPOSE: Header file for SmTools object.
**********************************************************************/

#ifndef __SMTOOLS_H__
#define __SMTOOLS_H__

#include <SmTypes.h>
#include <SmTArray.h>

class SmPoint3d;
class SmBrep;

/*******************************************************************//**
PURPOSE: This class provides misc tools for selection 

NOTES: 

***********************************************************************/
class SM_EXPORT SmTools
{
 public:
  static SmStatus FindFirstPointWithinTolToPnt      
  (
    const SmTArray<SmPoint3d> & crPointsArg,               ///< [in] : array of points to check                         <br>
    ULONG                       nPtIndexArg,               ///< [in] : index of point to match                          <br>      
    double                      d3dToleranceArg,           ///< [in] : max distance to travel to find a matching point  <br>     
    SmBoolean                 & rbIsFoundArg,              ///< [out]: TRUE=found a point within tolerance              <br>
    ULONG                     & rlFoundIndexArg,           ///< [out]: index of found point                             <br>    
    double                    & rdClosestDistanceArg,      ///< [out]: dist between target and found point              <br>   
    double                    & rd2ndClosestDistanceArg    ///< [out]: dist between target and next nearest neighbor    <br>    
  );

  static SmStatus FindFirstVertexWithinTolToPnt       
  (
    const SmTArray<SmVertex*> & crVertices,       ///< [in] : array of vertices to search              <br>
    const SmPoint3d           & sPnt,             ///< [in] : target point                             <br>
    double                      d3dTolerance,     ///< [in] : Tolerance size for sPnt                  <br>
    SmBoolean                 & rbIsFound,        ///< [out]: TRUE = found a vertex within tolerance   <br>
    ULONG                     & rlFoundIndex,     ///< [out]: index of found vertex                    <br>
    double           & rdClosestVertexDistance    ///< [out]: targetPoint/FoundVertex distance         <br>
  ) ; 

  static SmStatus FindCoincidentEdge   
  (
    const SmVertex          * pV1,              ///< [in] : Vertex mapping to pCurve StartPoint                       <br>
    const SmVertex          * pV2,              ///< [in] : Vertex mapping to pCurve EndPoint                         <br>
    const SmCurve           * pCurve,           ///< [in] : Target Curve                                              <br>
    const SmExtent1d        & rCurveIvl,        ///< [in] : Parameter range of curve (or its edge)                    <br>
    double                    d3dTolerance,     ///< [in] : pCurve and pCurve->EndPoint tolerance                     <br>       
    const SmTArray<SmEdge*> & rEdges,           ///< [in] : List of edges to search                                   <br>
    SmBoolean               & rbFound,          ///< [out]: TRUE = pCurve is coincident                               <br>  
    SmBoolean               & rbSameOrient,     ///< [out]: TRUE = pCurve is oriented SAME as found coincident curve  <br>  
    ULONG                   & rlFoundIndex,     ///< [out]: index in rEdges of found coincident curve                 <br>      
    double                  & rdMaxDeviation,   ///< [out]: Distance between pCurve/FoundEdge MidPoints               <br>     
    ULONG                     lStartIdx = 0     ///< [in,out]: allows ignoring the first part of rEdges               <br>
  ) ;                       

  static SmStatus FindEdge
  (
    SmBrep          * pBrep,            ///< [in] : input brep <br>
    const SmPoint3d & crPntOnEdge,      ///< [in] : point 3d on edge <br>
    ULONG           & rlEdgeIndex,      ///< [out]: edge index in brep <br>
    double          & dParam            ///< [out]: parameter on edge  <br>
  );

  static SmStatus FindFace
  (
    SmBrep          * pBrep,            ///< [in] : input brep       <br>
    const SmPoint3d & crPntOnFace,      ///< [in] : point 3d on face <br>
    SmFace         *& rpFace            ///< [out]: pointer to face  <br>
  );



} ; // end class SmTools

#endif // !__SMTOOLS_H__
