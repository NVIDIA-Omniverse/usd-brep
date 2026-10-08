// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*******************************************************************//**
* FILE NAME --- SmSweepGeometryCreateion.h
* PURPOSE: Header file for the class. 
**********************************************************************/

#ifndef __SMSWEEPGEOMETRYCREATION_H__
#define __SMSWEEPGEOMETRYCREATION_H__


#ifndef __SMOS_TYPES_H__
#include <SmTypes.h>
#endif


#ifndef __SMVERTEX_H__
#include <SmVertex.h>
#endif


#ifndef __SMTOPO_TYPES_H__
#include <SmTopoTypes.h>
#endif

#ifndef __SMCURVE_H__
#include <SmCurve.h>
#endif

#ifndef __SMSURFACE_H__
#include <SmSurface.h>
#endif


/*******************************************************************//**
PURPOSE: The following class is used by the SmTopologySweep class to
            generate geometry for the sweep.  This is a purely abstract
            class that is subclassed by various sweep geometries.

NOTES: 
***********************************************************************/
class SM_EXPORT SmSweepGeometryCreation
{
protected:
  SmBoolean   m_bMakeAnalytics;
  double     m_dEps;           // Tolerance.  Used for checking geometry on axis,
                               //   whether sweep lengths are zero, whether a
                               //   curve is a line, etc.
public:

  SmSweepGeometryCreation( double dEps = SM_ZONE_TOL_3D )
    : m_bMakeAnalytics( TRUE ),
    m_dEps( dEps )
  {}

  virtual ~SmSweepGeometryCreation();

  virtual SmSweepGeometryCreation *Copy() { return NULL; }

  void SetMakeAnalytics( SmBoolean bMakeAnalytics )
  {
    m_bMakeAnalytics = bMakeAnalytics;
  }


  virtual SmStatus VertexSweepHigher
  ( 
    const SmContext & /*crContext*/,
    const SmVertex  * /*pOriginalVertex*/,
    SmCurve        *& /*rpNewCurve*/,
    double          & /*rdNewEdgeTolerance*/ 
  ) const
  {
    return SM_ERR;
  }


  virtual SmStatus VertexSweepSame
  ( 
    const SmVertex * /*pOriginalVertex*/,
    SmPoint3d      & /*rPoint*/,
    double         & /*rdNewPointTolerance*/ 
  )   const
  {
    return SM_ERR;
  }


  virtual SmStatus EdgeSweepHigher
  ( 
    const SmContext             & /*crContext*/,              ///< [in ]: context for new object construction                                               <br>
    const SmEdge                * /*pEdgeToSweep*/,           ///< [in ]: Edge to be swept to a higher dimension                                            <br>
    SmSurface                  *& /*rpNewSurface*/,           ///< [out]: Surface created by the sweep of the edge                                          <br>
    double                      & /*rdNewFaceTol*/,           ///< [out]: Tolerance of the face - derived from edge tolerance.                              <br>
    SmCurve                    *& /*rpNewStartCurve*/,        ///< [out]: Curve Copy in orig position trimmed to edge ivl.                                  <br>
    double                      & /*rdNewStartCurveTol*/,     ///< [out]: Tolerance of the new start curve. value:[pEdgeArg->Tol]                           <br>
    SmCurve                    *& /*rpNewFarEndCurve*/,       ///< [out]: Curve copy moved to end sweep position.                                           <br>
    double                      & /*rdNewEdgeTol*/,           ///< [out]: Tolerance of this new edge. value:[pEdgeArg->Tol]                                 <br>
    SmCurve                    *& /*rpStartVertCurve*/,       ///< [out]: start vertex sweep curve.                                                         <br>
    double                      & /*rdNewStartVertEdgeTol*/,  ///< [out]: start vertex sweep curve tolerance.                                               <br>
    SmCurve                    *& /*rpEndVertCurve*/,         ///< [out]: end vertex sweep curve.                                                           <br>
    double                      & /*rdNewEndVertEdgeTol*/,    ///< [out]: end vertex sweep curve tolerance.                                                 <br>
    SmTArray<SmCurve*>          & /*r3DTrimmingCurves*/,      ///< [out]: ordered new 3D TrimCurves (ptrs to previously output curves)                      <br>
    SmTArray<SmBSplineCurve*>   & /*rUVTrimmingCurves*/,      ///< [out]: associated new UV TrimCurves when easy, not built when expensive.                 <br>
    SmTArray<SmOrientType>      & /*rOrients*/,               ///< [out]: associated TrimCurve Orients for a valid outer loop. SM_OT_SAME, SM_OT_OPPOSITE   <br>
    SmVector3d                  & /*rsNewSurfaceNormal*/      ///< [out]: The surface normal of the surface at the start vertex.                            <br>
  )  const
  {
    return SM_ERR;
  }


  virtual SmStatus EdgeSweepSame
  ( 
    const SmContext & /*crContext*/,           ///< [in ]: context for new object construction    <br>
    const SmEdge *    /*pOriginalEdge*/,       ///< [in ]: target edge to sweep                   <br>
    SmCurve *&        /*rpNewCurve*/,          ///< [out]: new curve at swept position            <br>
    double &          /*rdNewEdgeTolerance*/   ///< [out]: new edge tolerance                     <br>
  ) const
  {
    return SM_ERR;
  }


  virtual SmStatus FaceSweepSame
  ( 
    const SmContext & /*crContext*/,           ///< [in ]: context for new object construction    <br>
    const SmFace *    /*pOriginalFace*/,       ///< [in ]: target face to sweep                   <br>
    SmSurface *&      /*rpNewSurface*/,        ///< [out]: new surface in swept position          <br>
    double &          /*rdNewFaceTolerance*/   ///< [out]: new face tolerance                     <br>
  )  const
  {
    return SM_ERR;
  }


  // FaceSweepHigher is not needed since this class creates only
  // geometry - and that can be done for a face/higher by existing 
  // methods.
  virtual SmStatus CheckCurveSweep
  ( 
    const SmCurve     & /*crCurve*/,        ///< [in ]: Curve to check for valid sweep                                                                       <br>
    const SmExtent1d  & /*crIvl*/,          ///< [in ]: Interval to sweep                                                                                    <br>
    double              /*dDistTol3d*/,     ///< [in ]: min dist between unique points                                                                       <br>
    const SmPoint3d   & /*crSweepPoint*/,   ///< [in ]: RotSweep:[RotAxis point],  TransSweep:[NotUsed]                                                      <br>
    const SmVector3d  & /*crSweepVec*/,     ///< [in ]: RotSweep:[RotAxis vector], TransSweep:[Trans vector]                                                 <br>
    double              /*dSweepAmount*/,   ///< [in ]: RotSweep:[SweepAngleDeg],  TransSweep:[distance]                                                     <br>
    SmSweepCheckType  &   rSweepState       ///< [out]: SM_SC_OKAY               = okay to sweep this surface                                                <br>
                                            ///<      : SM_SC_ROT_ON_CURVE       = revolving curve about point on curve produces a BowTie or ConeApex surface<br>
                                            ///<      : SM_SC_SWEEP_ALONG_LENGTH = sweeping curve along it's length produces degenerate surface points       <br>
                                            ///<      : SM_SC_SELF_INTERSECT     = sweeping this curve produces a self-intersecting surface                  <br>
                                            ///<      : SM_SC_UNKNOWN            = Sweep method failed so swept surface validity is unknown                  <br>
  ) const
  {
    rSweepState = SM_SC_UNKNOWN; return SM_ERR;
  }

  virtual SmStatus IsSweepClosed
  ( 
    ULONG            /*nRepeats*/,
    SmBoolean        & bRet 
  ) const
  {
    bRet = FALSE; return SM_SUCCESS;
  }    
  
  virtual SmStatus GetStartCoordSystem
  ( 
    const SmPoint3d  & /* rPt */,
    SmAxis2Placement & /* rCoordSys */ 
  ) const
  {
    return SM_ERR;
  }

  virtual SmStatus GetEndCoordSystem
  ( 
    const SmPoint3d  & /* rPt */,
    SmAxis2Placement & /* rCoordSys */ 
  ) const
  {
    return SM_ERR;
  }

  virtual SmStatus GetSweepTransform( SmAxis2Placement & /* the transform */ ) const
  {
    return SM_ERR;
  }

}; // end class SmSweepGeometryCreation

#endif // !__SMSWEEPGEOMETRYCREATION_H__


