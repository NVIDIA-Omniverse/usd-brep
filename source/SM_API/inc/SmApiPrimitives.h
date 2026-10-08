// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************
FILE NAME: SmPrimitives.h

PURPOSE: 
    Contains popular, high level "C" type functions that create 
    primitive objects (cube, sphere, etc).

GENERAL NOTES: 
    High level functions may assume some input parameters 
    for ease of use. For maximum flexibility, related functions 
    can be found in SmPrimitiveCreation
**********************************************************************/


#ifndef __SmApiPrimitives_H__
#define __SmApiPrimitives_H__

#ifndef __SmApiTypes_H__
#include <SmApiTypes.h>
#endif

#include <SmTArray.h>

class SmContext;
class SmBrep;
class SmEdge;
class SmEdgeuse;
class SmFace;
class SmVector3d;
class SmBSplineCurve;

SMAPI_EXPORT SmApiStatus SmApiCreatePlane
( 
    const SmVector3d& crOrigin,      ///< [in ]: Position of bottom left corner of plane                      <br>
    double dX,                 ///< [in ]: Distance along X axis of frame                               <br>
    double dY,                 ///< [in ]: Distance along Y axis of frame                               <br>
    SmBrep*& rpResult         ///< [out]: Resulting SmBrep                                             <br>
);

SMAPI_EXPORT SmApiStatus SmApiCreatePlane
( 
    const SmVector3d& crOrigin,        ///< [in ]: Position of minU-minV corner                                     <br>
    double dLength,              ///< [in ]: Length along x-axis                                              <br>
    double dWidth,               ///< [in ]: Width along y-axis                                               <br>
    SmPlane*& rpResult          ///< [out]: Resulting SmPlane                                                <br>
); 

SMAPI_EXPORT SmApiStatus SmApiCreatePlanarCircle
( 
    const SmVector3d& crOrigin,      ///< [in ]: Position of bottom left corner of plane                     <br>
    double dRadius,            ///< [in ]: Radius of circle                                            <br>
    SmBrep*& rpResult         ///< [out]: Resulting SmBrep                                            <br>
);

SMAPI_EXPORT SmApiStatus SmApiCreateBox
( 
    const SmVector3d& crOrigin,      ///< [in ]: Position at lower left corner of the solid                 <br>
    double dLength,            ///< [in ]: Distance along the X axis                                  <br>
    double dWidth,             ///< [in ]: Distance along the Y axis                                  <br>
    double dHeight,            ///< [in ]: Distance along the Z axis                                  <br>
    SmBrep*& pResult          ///< [out]: Resulting SmBrep                                           <br>
);

SMAPI_EXPORT SmApiStatus SmApiCreateSphere
( 
    const SmVector3d& crOrigin,      ///< [in ]: Center of sphere                                            <br>
    double dRadius,            ///< [in ]: Distance radius of sphere                                   <br>
    SmBrep*& pResult          ///< [out]: Resulting SmBrep                                            <br>
); 

SMAPI_EXPORT SmApiStatus SmApiCreateCone
( 
    const SmVector3d& crOrigin,      ///< [in ]: Position of center of base                                  <br>
    double dRadiusBase,        ///< [in ]: Radius of base                                              <br>
    double dRadiusTop,         ///< [in ]: Radius of top                                               <br>
    double dHeight,            ///< [in ]: Distance bewteen base and top                               <br>
    SmBrep*& rpResult         ///< [out]: Resulting SmBrep                                            <br>
);

SMAPI_EXPORT SmApiStatus SmApiCreateCone
( 
    const SmVector3d& crOrigin,      ///< [in ]: Position of center of base                                    <br>
    double dRadiusBase,        ///< [in ]: Radius at base                                                <br>
    double dRadiusTop,         ///< [in ]: Radius at top (for truncated cones)                           <br>
    double dHeight,            ///< [in ]: Height of cone                                                <br>
    SmSurface*& rpResult      ///< [out]: Resulting SmCone                                              <br>
);

SMAPI_EXPORT SmApiStatus SmApiCreateCylinder
( 
    const SmVector3d& crOrigin,      ///< [in ]: Position of center of base                                  <br>
    double dRadius,            ///< [in ]: Radius of base                                              <br>
    double dHeight,            ///< [in ]: Distance bewteen base and top                               <br>
    SmBrep*& rpResult         ///< [out]: Resulting SmBrep                                            <br>
);

SMAPI_EXPORT SmApiStatus SmApiCreateCylinder
( 
    const SmVector3d& crOrigin,      ///< [in ]: Position of center of base                                   <br>
    double dRadius,            ///< [in ]: Radius                                                       <br>
    double dHeight,            ///< [in ]: Distance bewteen base and top                                <br>
    SmSurface*& rpResult      ///< [out]: Resulting SmCylinder                                         <br>
);

SMAPI_EXPORT SmApiStatus SmApiCreateTorus
( 
    const SmVector3d& crOrigin,      ///< [in ]: Position of center of base                                  <br>
    double dRadiusMajor,       ///< [in ]: Major Radius                                                <br>
    double dRadiusMinor,       ///< [in ]: Minor Radius                                                <br>
    SmBrep*& rpResult         ///< [out]: Resulting SmBrep                                            <br>
);

SMAPI_EXPORT SmApiStatus SmApiCreateTorus
( 
    const SmVector3d& crOrigin,      ///< [in ]: Position of center of base                                  <br>
    double dRadiusMajor,       ///< [in ]: Major Radius                                                <br>
    double dRadiusMinor,       ///< [in ]: Minor Radius                                                <br>
    SmSurface*& rpResult      ///< [out]: Resulting SmBrep                                            <br>
);

SMAPI_EXPORT SmApiStatus SmApiCreatePyramid
( 
    const SmVector3d& crOrigin,      ///< [in ]: Position of center of base                                  <br>
    double dLength,            ///< [in ]: Length and width                                            <br>
    double dHeight,            ///< [in ]: Height at apex                                              <br>
    SmBrep*& rpResult         ///< [out]: Resulting SmBrep                                            <br>
);

SMAPI_EXPORT SmApiStatus SmApiCreateCylindricalBox
(
    const SmVector3d& crOrigin,      ///< [in ]: Position of origin on Z axis at base                        <br>
    double dLength,            ///< [in ]: Length along Z axis, greater than zero                       <br>
    double dInsideRadius,      ///< [in ]: Inside radius from Z axis, greater than zero                <br>
    double dOutsideRadius,     ///< [in ]: Outside radius from Z axis, greater than inside radius      <br>
    double dStartAngleDeg,     ///< [in ]: Start angle from X axis in XY plane, degrees                <br>
    double dEndAngleDeg,       ///< [in ]: End angle from X axis in XY plane, degrees                  <br>
    SmBrep*& rpResult         ///< [out]: Resulting SmBrep                                            <br>
);

SMAPI_EXPORT SmApiStatus SmApiCreatePartialSphere
(
    const SmVector3d& crOrigin,      ///< [in ]: Center of sphere                                            <br>
    double dRadius,            ///< [in ]: Radius of sphere                                            <br>
    double dStartAngleDeg,     ///< [in ]: Start angle in XY plane from X axis, degrees                <br>
    double dEndAngleDeg,       ///< [in ]: End angle in XY plane from X axis, degrees                  <br>
    SmBrep*& rpResult         ///< [out]: Resulting SmBrep                                            <br>
);

SMAPI_EXPORT SmApiStatus SmApiCreateSphereNoPole
(
    const SmVector3d& crCenter,      ///< [in ]: Center of sphere                                            <br>
    double dRadius,            ///< [in ]: Radius of sphere                                            <br>
    SmBrep*& rpResult         ///< [out]: Resulting SmBrep                                            <br>
);

SMAPI_EXPORT SmApiStatus SmApiCreatePartialCone
(
    const SmVector3d& crOrigin,      ///< [in ]: Position of center of base                                  <br>
    double dRadiusBase,        ///< [in ]: Radius at base                                              <br>
    double dRadiusTop,         ///< [in ]: Radius at top                                               <br>
    double dHeight,            ///< [in ]: Height of cone                                              <br>
    double dStartAngleDeg,     ///< [in ]: Start angle in XY plane from X axis, degrees                <br>
    double dEndAngleDeg,       ///< [in ]: End angle in XY plane from X axis, degrees                  <br>
    SmBrep*& rpResult         ///< [out]: Resulting SmBrep                                            <br>
);

SMAPI_EXPORT SmApiStatus SmApiCreateConeEllipticEnds
(
    double dFrontRadius,       ///< [in ]: Radius at front end (before tilt)                           <br>
    double dRearRadius,        ///< [in ]: Radius at rear end (before tilt)                            <br>
    double dHeight,            ///< [in ]: Distance between end-ellipse centers                        <br>
    double dFrontYTiltDeg,     ///< [in ]: Front end tilt about X axis, degrees                        <br>
    double dRearYTiltDeg,      ///< [in ]: Rear end tilt about X axis, degrees                         <br>
    double dFrontXTiltDeg,     ///< [in ]: Front end tilt about Y axis, degrees                        <br>
    double dRearXTiltDeg,      ///< [in ]: Rear end tilt about Y axis, degrees                         <br>
    SmBoolean bEndCaps,        ///< [in ]: TRUE = close off end caps                                   <br>
    SmBrep*& rpResult         ///< [out]: Resulting SmBrep                                            <br>
);

SMAPI_EXPORT SmApiStatus SmApiCreatePartialCylinder
(
    const SmVector3d& crOrigin,      ///< [in ]: Position of center of base                                  <br>
    double dRadius,            ///< [in ]: Radius                                                      <br>
    double dHeight,            ///< [in ]: Height                                                      <br>
    double dStartAngleDeg,     ///< [in ]: Start angle in XY plane from X axis, degrees                <br>
    double dEndAngleDeg,       ///< [in ]: End angle in XY plane from X axis, degrees                  <br>
    SmBrep*& rpResult         ///< [out]: Resulting SmBrep                                            <br>
);

SMAPI_EXPORT SmApiStatus SmApiCreatePartialTorus
(
    const SmVector3d& crOrigin,      ///< [in ]: Position of center of torus                                 <br>
    double dRadiusMajor,       ///< [in ]: Major radius                                                <br>
    double dRadiusMinor,       ///< [in ]: Minor radius                                                <br>
    double dStartAngleDeg,     ///< [in ]: Start angle in XY plane from X axis, degrees                <br>
    double dEndAngleDeg,       ///< [in ]: End angle in XY plane from X axis, degrees                  <br>
    SmBrep*& rpResult         ///< [out]: Resulting SmBrep                                            <br>
);

/// rPlanarCurves and the curves it contains are borrowed. They remain
/// caller-owned and unchanged on both success and failure.
SMAPI_EXPORT SmApiStatus SmApiCreateOffsetProfile
(
    const SmTArray<SmCurve*>& rPlanarCurves,  ///< [in ]: Planar curves forming at least one closed loop     <br>
    double dOffsetDistance,              ///< [in ]: Offset distance                                    <br>
    ULONG lOffsetType,                  ///< [in ]: 1=LEFT, 2=RIGHT, 3=BOTH                            <br>
    SmBoolean bRoundCorners,            ///< [in ]: TRUE = round expanding corners, FALSE = extend      <br>
    SmBoolean bShellResult,             ///< [in ]: TRUE = shell result (LEFT or RIGHT only)            <br>
    SmBrep*& rpResult                  ///< [out]: Resulting SmBrep                                    <br>
);

SMAPI_EXPORT SmApiStatus SmApiCreateBlendPrimitive
(
    SmBrep* pBrep,                      ///< [in ]: Brep containing the boundary edges                  <br>
    SmEdge* pEdge1,                     ///< [in ]: First boundary edge (start of blend)                <br>
    SmFace* pFace1,                     ///< [in ]: Target start blend surface                          <br>
    SmEdge* pEdge2,                     ///< [in ]: Second boundary edge (end of blend)                 <br>
    SmFace* pFace2,                     ///< [in ]: Target end blend surface                            <br>
    SmBoolean bSameDirCurves,           ///< [in ]: TRUE = start of Edge1->Curve to start of Edge2->Curve<br>
                                        ///< [in ]: FALSE = start of Edge1->Curve to end of Edge2->Curve<br>
    SmFace*& rpBlendFace               ///< [out]: Resulting blend face                                <br>
);

SMAPI_EXPORT SmApiStatus SmApiCreateSwungPrimitive
(
    SmTArray<SmCurve*>& rXYCurves,      ///< [in ]: Curves defining the XY profile                     <br>
    SmTArray<SmCurve*>& rXZCurves,      ///< [in ]: Curves defining the XZ profile                     <br>
    double dScale,                      ///< [in ]: Scale factor                                        <br>
    SmBrep*& rpResult                  ///< [out]: Resulting SmBrep                                    <br>
);

SMAPI_EXPORT SmApiStatus SmApiCreateSkinPrimitive
(
    const SmTArray<ULONG>& rCurvesInEachProfile,       ///< [in ]: Positive curve counts for at least two profiles; <br>
                                                       ///<       sum must equal r3DCurves.GetSize()               <br>
    const SmTArray<const SmCurve*>& r3DCurves,         ///< [in ]: All profile curve segments concatenated; caller retains <br>
                                                       ///<       ownership and curves are unchanged                    <br>
    ULONG lDegree,                          ///< [in ]: Degree in loft direction: 1=linear, 2=quad, 3=cubic <br>
    SmBoolean bCapEnds,                     ///< [in ]: TRUE = cap only the first and last profiles;     <br>
                                            ///<       intermediate profiles guide the loft but do not   <br>
                                            ///<       create transverse faces                           <br>
    SmBrep*& rpResult,                      ///< [out]: Resulting SmBrep; NULL on failure                <br>
    SmTArray<SmFace*>* pOptStartFaces,      ///< [opt]: Start cap faces; empty on failure (NULL to ignore) <br>
    SmTArray<SmFace*>* pOptSideFaces,       ///< [opt]: Side loft faces; empty on failure (NULL to ignore) <br>
    SmTArray<SmFace*>* pOptEndFaces        ///< [opt]: End cap faces; empty on failure (NULL to ignore) <br>
);

SMAPI_EXPORT SmApiStatus SmApiCreateSkinFromFaces
(
    SmTArray<SmFace*>& rFaces,              ///< [in ]: Sequence of faces whose loops to loft between   <br>
    ULONG lDegree,                          ///< [in ]: Degree in loft direction: 1=linear, 2=quad, 3=cubic <br>
    SmTArray<SmFace*>& rNewFaces,           ///< [out]: Resulting lofted faces                          <br>
    SmBrep*& rpResult                      ///< [out]: Resulting SmBrep                                <br>
);


#endif
