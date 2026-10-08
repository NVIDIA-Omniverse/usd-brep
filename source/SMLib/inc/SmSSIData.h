// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*******************************************************************//**
* FILE NAME --- SmSurfTypes.h
* PURPOSE: Declaration of surface and volume types.
**********************************************************************/

#ifndef __SMSSIDATA_H__
#define __SMSSIDATA_H__

#ifndef __SMOS_TYPES_H__
#include <SmTypes.h>
#endif

#ifndef __SMTARRAY_H__
#include <SmTArray.h>
#endif

#ifndef __SMBSPLINECURVE_H__
#include <SmBSplineCurve.h>
#endif
/*******************************************************************//**
PURPOSE: This is just a convenience structure to hold intersection
   data produced by the surface/surface intersection.

NOTES: 
***********************************************************************/
class SmSSIData 
{
  public:
    SmTArray<SmCurve*>         m_v3DCurves;    // array of surf1/surf2 intersection curves
    SmTArray<SmBSplineCurve*>  m_vUVCurves1;   // associated surf1 UVTrimCurves
    SmTArray<SmBSplineCurve*>  m_vUVCurves2;   // associated surf2 UVTrimCurves
    SmTArray<SmTsectCurveType> m_vCurveTypes;  // associated intersection classification
    SmTArray<double>           m_vDeviations;  // associated max devitations 

} ; // end class class SmSSIData

#endif // __SMSSIDATA_H__
