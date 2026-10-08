// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*******************************************************************//**
* FILE NAME --- SmCurveFilletExecutive.h
* PURPOSE: Header file for the SmCurveFilletExecutive class.
**********************************************************************/

#ifndef __SMCURVEFILLETEXECUTIVE_H__
#define __SMCURVEFILLETEXECUTIVE_H__

class SM_EXPORT SmCurveFilletExecutive
{
protected:
  SmCurve * m_pCurve1;
  SmCurve * m_pCurve2;
  double    m_dRadius;

  SmBoolean m_bDoTrim;  // Whether to trim the input curves to the fillet
                        // Default: TRUE
  SmBoolean m_bDoJoin;  // Whether to join the three curves after filleting.
                        // Note, this requires m_bDoTrim == True.
                        // If Join is requested without Trim, DoFillet() will Trim and Join.
                        // This also requires that both of the given curves
                        // are SmBSplineCurves (or derived from it).
                        // Default: FALSE

  // Location (side) specifiers.
  // For two curves that cross each other, there are four possible
  // places to put the fillet.
  // The location can be specified using the two parameter values:
  // The fillet goes on the sides of the curves towards
  // these parameters, relative to the intersection of the curves.
  // They could be start or end parameters.
  // These can also be used to indicate which intersection to use,
  // if the curves intersect more than once.
  //
  // If these parameters are not specified, then the location of
  // the fillet is toward the larger portion of both curves
  // (i.e., the midpoints are used as selection parameters).
  // This is handy if the curves meet at or near their ends:
  // in that case no side indicators are necessary.
  //
  double m_dParam1, m_dParam2;

  // Shape specifiers:  Not currently implemented.
  // double    m_dThumbweight;
  // double    m_dBias;

  // The resulting fillet curve:
  SmBSplineCurve *m_pResult;


public:

  // Constructor
  SmCurveFilletExecutive
  (
    SmCurve * pCurve1,           ///< [in] :
    SmCurve * pCurve2,           ///< [in] :
    double    dRadius,           ///< [in] :
    SmBoolean bDoTrim=TRUE,      ///< [in] :
    SmBoolean bDoJoin=FALSE      ///< [in] :
  );

  // Access methods
  SmCurve * GetCurve1()    { return m_pCurve1; }
  SmCurve * GetCurve2()    { return m_pCurve2; }
  double    GetRadius()    { return m_dRadius; }
  SmBoolean GetDoTrim()    { return m_bDoTrim; }
  SmBoolean GetDoJoin()    { return m_bDoJoin; }
  SmBSplineCurve * GetResult() { return m_pResult; }

  // Note, no SetCurve methods: just create a new object of this class.
  void SetRadius( double dRad )          { m_dRadius = dRad; }
  void SetDoTrim( SmBoolean bDoTrim )    { m_bDoTrim = bDoTrim; }
  void SetDoJoin( SmBoolean bDoJoin )    { m_bDoJoin = bDoJoin; }

  // Workers
  SmStatus DoFillet();
  SmStatus DoFillet( SmBSplineCurve *& rpFilletCurve );

}; // end class SmCurveFilletExecutive

#endif  // __SMCURVEFILLETEXECUTIVE_H__
