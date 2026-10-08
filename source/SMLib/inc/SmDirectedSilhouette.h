// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*******************************************************************//**
* FILE NAME --- SmDirectedSilhouette.h
* PURPOSE: Header file for SmDirectedSilhouette object.
**********************************************************************/

#ifndef __SMDIRECTEDSILHOUETTE_H__
#define __SMDIRECTEDSILHOUETTE_H__

#ifndef __SMMEMBLOCKMGR_H__
#include <SmMemBlockMgr.h>
#endif

#ifndef __SMBSPLINECURVE_H__
#include <SmBSplineCurve.h>
#endif

#ifndef __SMEXTENT1D_H__
#include <SmExtent1d.h>
#endif

class SmDirectedSilhouette;  //needed for gcc4.x

/*******************************************************************//**
PURPOSE: This object represents a homogeneous section of a curve
   projection.    

NOTES:  static class object - don't add virtual methods to SmDSSpan 
***********************************************************************/
class SM_EXPORT SmDSSpan
{
    friend class SmDirectedSilhouette;
private:
    SmDirectedSilhouette   * m_pDirectedSilhouette; // Pointer to corresponding
                                                    // directed silhouette which owns the span.
    const SmBSplineCurve   * m_pCurve;       // Pointer to 3D curve
    SmExtent1d               m_sInterval;    // Interval on curve span represents
    SmExtent1d               m_sLRInterval;  // Interval relative to the Left to Right
                             // direction vector of the end points of the interval.
    SmBoolean                     m_bDirection;   // If TRUE orientation of the curve interval 
                             // and the LR interval correspond.  If FALSE they are opposite.
public:
    SmStatus Init(SmDirectedSilhouette * pDirectedSilhouette,
                  const SmBSplineCurve * pCurve,
                  const SmExtent1d & rInterval);

    SmStatus LeftRightToCurveParameter(double dLeftRightParameter, 
                                       double & rdCurveParameter) const;

    SmStatus EvaluateProjection(double dCurveParameter,
                                SmPoint2d & rProjPlanePoint) const;

    SmStatus CurveParameterToLeftRight(double dCurveParameter,
                                       double & rdLeftRightParameter) const;

    double GetLRParameter(SmBoolean bDoRightEnd) const;
};

/*******************************************************************//**
PURPOSE: The directed silhouette object finds the visibility
     of a set of 3D curves projected into a plane.  It is like a 
     hidden line algorithm that operates in a 2D plane.  

NOTES: 
***********************************************************************/
class SM_EXPORT SmDirectedSilhouette 
{
    friend class SmDSSpan;
private:
    const SmContext & m_crContext;          // Context to create things in
    double            m_d3DTolerance;       // Tolerance used in computations
    SmVector3d        m_vPlaneNormal;       // Normal to a plane into which the curves
                                            // are projected for the visibility determination.  The curves are projected
                                            // into the plane using parallel projection.
    SmVector3d        m_vViewingDirection;  // Viewing direction (must be in the plane) 
                                            // which is used to determine visibility.  Note that visibility determination
                                            // within the plane also uses only parallel projection.
    SmVector3d        m_vLeftRightVector;   // Vector in the viewing plane which is
                                            // perpendicular to the viewing direction.  This vector is used to sort the
                                            // spans in a left to right fashon.
    SmMemBlockMgr     m_vSpanMgr;           // Manages memory for the SmDSSpan objects used
                                            // by the SmDirectedSilhouette.
    SmTArray<SmDSSpan*> * m_pStartSpans;    // Contains a sorted list of span starts
    SmTArray<SmDSSpan*> * m_pEndSpans;      // Contains a sorted list of span ends
    SmTArray<SmDSSpan*> * m_pFrontSpans;    // Results directed silhouette operation in
                                            // span format sorted left to right.

public:
    ~SmDirectedSilhouette();

    SmDirectedSilhouette(const SmContext & crContext,
                         const SmVector3d & crPlaneNormal,
                         const SmVector3d & crViewingDirection);

    SmStatus CreateSpans(const SmTArray<SmBSplineCurve*> & crCurves);

    SmStatus InsertSpanIntoList(SmDSSpan * pSpan,
                                SmBoolean bDoingEnd,
                                SmTArray<SmDSSpan*> & rSortedSpans);

    SmStatus FindVisibleSegments(const SmTArray<SmBSplineCurve*> & crCurves,
                                 double d3DTolerance);

    SmStatus ExtractVisibleSegments(SmTArray<SmBSplineCurve*> & rVisibleSegments) const;

    SmStatus FindFrontmostSpans();

    SmStatus GetNextStartSpans(SmTArray<SmDSSpan*> & rNextStartSpans,
                               double & rdSpanStartValue) const;

    SmStatus GetNextEndSpans(SmTArray<SmDSSpan*> & rNextEndSpans,
                             double & rdSpanEndValue) const;

    SmStatus FlushFrontSpan(const SmDSSpan * pSpanToFlush,
                            double dStartLRParam, 
                            double dEndLRParam);
};

#endif // !__SMDIRECTEDSILHOUETTE_H__


