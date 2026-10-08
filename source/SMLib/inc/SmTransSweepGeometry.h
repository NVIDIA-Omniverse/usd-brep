// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*******************************************************************//**
* FILE NAME --- SmTranslationalSweepGeometry.h
* PURPOSE   --- Header file for the class. 
**********************************************************************/

#ifndef __SMTRANSLATIONALSWEEPGEOMETRY_H__
#define __SMTRANSLATIONALSWEEPGEOMETRY_H__

#ifndef __SMSWEEPGEOMETRYCREATION_H__
#include <SmSweepGeometryCreation.h>
#endif

#ifndef __SMVECTOR3D_h__
#include <SmVector3d.h>
#endif

/*******************************************************************//**
PURPOSE: This derived class of SmSweepGeometryCreation creates geometry
            for simple translational sweep.

NOTES: 
***********************************************************************/
class SM_EXPORT SmTranslationalSweepGeometry : public SmSweepGeometryCreation
{
protected:
  SmVector3d m_sSweepVector;        // sweep length and direction

  // default constructor
  SmTranslationalSweepGeometry(double dEps = SM_ZONE_TOL_3D)      : SmSweepGeometryCreation(dEps) 
                                                                  { }

public:

  // constructor - destructor
  SmTranslationalSweepGeometry(const SmVector3d & crSweepVectArg, double dEps = SM_ZONE_TOL_3D) ;

  virtual ~SmTranslationalSweepGeometry() {}

  virtual SmSweepGeometryCreation *Copy() ;

  // create a curve by sweeping a vertex into a higher dimension
  SmStatus VertexSweepHigher
  (
    const SmContext & crContext,             ///< [in ]: context for new object construction <br>
    const SmVertex  * pOriginalVertex,       ///< [in ]: target vertex to sweep              <br>
    SmCurve        *& rpNewCurve,            ///< [out]: newly swept curve                   <br>
    double          & rdNewEdgeTolerance     ///< [out]:                                     <br>
  ) const ;

  // create a point by sweeping a vertex to its final position
  SmStatus VertexSweepSame
  (
    const SmVertex * pOriginalVertex,        ///< [in ]: target vertex to sweep              <br>
    SmPoint3d      & rPoint,                 ///< [out]: swept point                         <br>
    double         & rdNewPointTolerance     ///< [out]: new point tolerance                 <br>
  ) const ; 

  // create a surface and its bounding curves by sweeping an edge into a higher dimension
    virtual SmStatus EdgeSweepHigher
    (
      const SmContext             & crContext,               ///< [in ]: context for new object construction                                               <br>
       const SmEdge                * pEdgeToSweep,           ///< [in ]: Edge to be swept to a higher dimension                                            <br>
       SmSurface                  *& rpNewSurface,           ///< [out]: Surface created by the sweep of the edge                                          <br>
       double                      & rdNewFaceTol,           ///< [out]: Tolerance of the face - derived from edge tolerance.                              <br>
       SmCurve                    *& rpNewStartCurve,        ///< [out]: Curve Copy in orig position trimmed to edge ivl.                                  <br>
       double                      & rdNewStartCurveTol,     ///< [out]: Tolerance of the new start curve. value:[pEdgeArg->Tol]                           <br>
       SmCurve                    *& rpNewFarEndCurve,       ///< [out]: Curve copy moved to end sweep position.                                           <br>
       double                      & rdNewEdgeTol,           ///< [out]: Tolerance of this new edge. value:[pEdgeArg->Tol]                                 <br>
       SmCurve                    *& rpStartVertCurve,       ///< [out]: start vertex sweep curve.                                                         <br>
       double                      & rdNewStartVertEdgeTol,  ///< [out]: start vertex sweep curve tolerance.                                               <br>
       SmCurve                    *& rpEndVertCurve,         ///< [out]: end vertex sweep curve.                                                           <br>
       double                      & rdNewEndVertEdgeTol,    ///< [out]: end vertex sweep curve tolerance.                                                 <br>
       SmTArray<SmCurve*>          & r3DTrimmingCurves,      ///< [out]: ordered new 3D TrimCurves (ptrs to previously output curves)                      <br>
       SmTArray<SmBSplineCurve*>   & rUVTrimmingCurves,      ///< [out]: associated new UV TrimCurves when easy, not built when expensive.                 <br>
       SmTArray<SmOrientType>      & rOrients,               ///< [out]: associated TrimCurve Orients for a valid outer loop. SM_OT_SAME, SM_OT_OPPOSITE   <br>
       SmVector3d                  & rsNewSurfaceNormal      ///< [out]: The surface normal of the surface at the start vertex.                            <br>
    ) const ;

  // create a curve by sweeping an edge to its final position
  SmStatus EdgeSweepSame
  (
    const SmContext & crContext,            ///< [in ]: context for new object construction    <br>
    const SmEdge    * pOriginalEdge,        ///< [in ]: target edge to sweep                   <br>
    SmCurve        *& rpNewCurve,           ///< [out]: new curve at swept position            <br>
    double          & rdNewEdgeTolerance    ///< [out]: new edge tolerance                     <br>
  ) const ;

  // create a surface by sweeping an edge to its final position
  SmStatus FaceSweepSame
  (
    const SmContext & crContext,            ///< [in ]: context for new object construction    <br>
    const SmFace    * pOriginalFace,        ///< [in ]: target face to sweep                   <br>
    SmSurface      *& rpNewSurface,         ///< [out]: new surface in swept position          <br>
    double          & rdNewFaceTolerance    ///< [out]: new face tolerance                     <br>
  ) const;

    // FaceSweepHigher is not needed since this class creates only
    // geometry - and that can be done for a face/higher by existing 
    // methods.

    virtual SmStatus CheckCurveSweep
    ( 
      const SmCurve     & crCurve,       ///< [in ]: Curve to check for valid sweep                                                                        <br>
      const SmExtent1d  & crIvl,         ///< [in ]: Interval to sweep                                                                                     <br>
      double              dDistTol3d,    ///< [in ]: min dist between unique points                                                                        <br>
      const SmPoint3d   & crSweepPoint,  ///< NotUsed: [in ]: RotSweep:[RotAxis point],  TransSweep:[NotUsed]                                                       <br>
      const SmVector3d  & crSweepVec,    ///< [in ]: RotSweep:[RotAxis vector], TransSweep:[Trans vector]                                                  <br>
      double              dSweepAmount,  ///< [in ]: RotSweep:[SweepAngleDeg],  TransSweep:[distance]                                                      <br>
      SmSweepCheckType  & rSweepState    ///< [out]: SM_SC_OKAY               = okay to sweep this surface                                                 <br>
                                         ///<      : SM_SC_ROT_ON_CURVE       = revolving curve about point on curve produces a BowTie or ConeApex surface <br>
                                         ///<      : SM_SC_SWEEP_ALONG_LENGTH = sweeping curve along it's length produces degenerate surface points        <br>
                                         ///<      : SM_SC_SELF_INTERSECT     = sweeping this curve produces a self-intersecting surface                   <br>
                                         ///<      : SM_SC_UNKNOWN            = Sweep method failed so swept surface validity is unknown                   <br>
    )  const;

  virtual SmStatus GetStartCoordSystem
  ( 
    const SmPoint3d  & rPt,              ///< [in ]:       <br>
    SmAxis2Placement & rCoordSys         ///< [out]:       <br>
  ) const;

  virtual SmStatus GetEndCoordSystem
  ( 
    const SmPoint3d  & rPt,               ///< [in ]:      <br>
    SmAxis2Placement & rCoordSys          ///< [out]:      <br>
  ) const;

  virtual SmStatus GetSweepTransform  
  ( 
    SmAxis2Placement & rTransform         ///< [out]:      <br>
  ) const;

} ; // end class SmTranslationalSweepGeometry


#endif // !__SMTRANSLATIONALSWEEPGEOMETRY_H__




