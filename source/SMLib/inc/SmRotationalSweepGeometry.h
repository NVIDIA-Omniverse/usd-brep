// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*******************************************************************//**
* FILE NAME --- SmRotationalSweepGeometry.h
* PURPOSE   --- Header file for the class. 
**********************************************************************/

#ifndef __SMROTATIONALSWEEPGEOMETRY_H__
#define __SMROTATIONALSWEEPGEOMETRY_H__

#ifndef __SMSWEEPGEOMETRYCREATION_H__
#include <SmSweepGeometryCreation.h>
#endif

#ifndef __SMVECTOR3D_h__
#include <SmVector3d.h>
#endif

/*******************************************************************//**
PURPOSE: This derived class of SmSweepGeometryCreation creates geometry
            for simple rotational sweep.

NOTES: 
***********************************************************************/
class SM_EXPORT SmRotationalSweepGeometry : public SmSweepGeometryCreation
{
protected:
     SmPoint3d  m_sBasePt; // Base pt and axis define axis about which to rotate.
     SmVector3d m_sAxis;   // unit-dir of rotation axis (unitized in constructor)
     double     m_dAngle;  // In degrees.
     double     m_dEps;    // Tolerance.  Used for checking geometry on axis,
                           //   whether sweep lengths are zero, whether a
                           //   curve is a line, etc.

//   double     m_dRadius; // Not used.

     // Constructor
     SmRotationalSweepGeometry(){}

public:
    SmRotationalSweepGeometry(const SmPoint3d  & crCenterArg,
                              const SmVector3d & crAxisArg,
                              double             dAngleArg,
                              double             dEpsArg);

    virtual ~SmRotationalSweepGeometry() {}

    virtual SmSweepGeometryCreation *Copy();

    SmStatus VertexSweepHigher(const SmContext & crContext,
                             const SmVertex *pOriginalVertex,
                             SmCurve *& rpNewCurve,
                             double & rdNewEdgeTolerance) const;

    SmStatus VertexSweepSame(const SmVertex *pOriginalVertex,
                             SmPoint3d & rPoint,
                             double & rdNewPointTolerance) const; 

    virtual SmStatus EdgeSweepHigher
      (const SmContext             & crContext,             // in : context for new object construction
       const SmEdge                * pEdgeToSweep,          // in : Edge to be swept to a higher dimension
       SmSurface                  *& rpNewSurface,          // out: Surface created by the sweep of the edge
       double                      & rdNewFaceTol,          // out: Tolerance of the face - derived from edge tolerance.
       SmCurve                    *& rpNewStartCurve,       // out: Curve Copy in orig position trimmed to edge ivl.
       double                      & rdNewStartCurveTol,    // out: Tolerance of the new start curve. value:[pEdgeArg->Tol]
       SmCurve                    *& rpNewFarEndCurve,      // out: Curve copy moved to end sweep position.
       double                      & rdNewEdgeTol,          // out: Tolerance of this new edge. value:[pEdgeArg->Tol]
       SmCurve                    *& rpStartVertCurve,      // out: start vertex sweep curve.
       double                      & rdNewStartVertEdgeTol, // out: start vertex sweep curve tolerance.
       SmCurve                    *& rpEndVertCurve,        // out: end vertex sweep curve. 
       double                      & rdNewEndVertEdgeTol,   // out: end vertex sweep curve tolerance.
       SmTArray<SmCurve*>          & r3DTrimmingCurves,     // out: ordered new 3D TrimCurves (ptrs to previously output curves) 
       SmTArray<SmBSplineCurve*>   & rUVTrimmingCurves,     // out: associated new UV TrimCurves when easy, not built when expensive.
       SmTArray<SmOrientType>      & rOrients,              // out: associated TrimCurve Orients for a valid outer loop. SM_OT_SAME, SM_OT_OPPOSITE
       SmVector3d                  & rsNewSurfaceNormal)    // out: The surface normal of the surface at the start vertex.
      const ;
    
    SmStatus EdgeSweepSame(const SmContext & crContext,            // in : context for new object construction
                           const SmEdge    * pOriginalEdge,        // in : target edge to sweep
                           SmCurve        *& rpNewCurve,           // out: new curve at swept position
                           double          & rdNewEdgeTolerance)   // out: new edge tolerance
                         const;

    SmStatus FaceSweepSame(const SmContext & crContext,            // in : context for new object construction
                           const SmFace    * pOriginalFace,        // in : target face to sweep
                           SmSurface      *& rpNewSurface,         // out: new surface in swept position
                           double          & rdNewFaceTolerance)   // out: new face tolerance
                         const;

    virtual SmStatus CheckCurveSweep    ( const SmCurve     & crCurve,         // in : Curve to check for valid sweep
                                          const SmExtent1d  & crIvl,           // in : Interval to sweep
                                          double              dDistTol3d,      // in : min dist between unique points
                                          const SmPoint3d   & crSweepPoint,    // in : RotSweep:[RotAxis point],  TransSweep:[NotUsed]
                                          const SmVector3d  & crSweepVec,      // in : RotSweep:[RotAxis vector], TransSweep:[Trans vector]
                                          double              dSweepAmount,    // in : RotSweep:[SweepAngleDeg],  TransSweep:[distance]
                                          SmSweepCheckType  & rSweepState)     // out: SM_SC_OKAY               = okay to sweep this surface
                                         const;                                //      SM_SC_ROT_ON_CURVE       = revolving curve about point on curve produces a BowTie or ConeApex surface
                                                                               //      SM_SC_SWEEP_ALONG_LENGTH = sweeping curve along it's length produces degenerate surface points
                                                                               //      SM_SC_SELF_INTERSECT     = sweeping this curve produces a self-intersecting surface
                                                                               //      SM_SC_UNKNOWN            = Sweep method failed so swept surface validity is unknown
    virtual SmStatus IsSweepClosed      ( ULONG              nRepeats, 
                                          SmBoolean        & bRet)        
                                         const;

    virtual SmStatus GetStartCoordSystem( const SmPoint3d  & rPt,
                                          SmAxis2Placement & rCoordSys)   
                                         const;

    virtual SmStatus GetEndCoordSystem( const SmPoint3d  & rPt,
                                          SmAxis2Placement & rCoordSys)   
                                         const;

    virtual SmStatus GetSweepTransform  ( SmAxis2Placement & rTransform ) 
                                         const;

} ; // end SmRotationalSweepGeometry



#endif // !__SMROTATIONALSWEEPGEOMETRY_H__



