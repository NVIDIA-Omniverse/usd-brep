// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*******************************************************************//**
* FILE NAME --- SmFilletCustomSolver.h 
* PURPOSE: Header file for SmFilletSolver object.
**********************************************************************/

#ifndef __SMFILLETCUSTOMSOLVER_H__
#define __SMFILLETCUSTOMSOLVER_H__

#ifndef __SMFILLETSOLVER_H__
#include <SmFilletSolver.h>
#endif

#ifndef __SMFILLETSTANDARDSOLVER_H__
#include <SmFilletStandardSolver.h>
#endif

/*******************************************************************//**
PURPOSE: This fillet solver provides the equations for constant 
    radius filleting between two surfaces with two different offset
    surfaces

NOTES: 
***********************************************************************/
class SM_EXPORT SmEiffelTowerFS : public SmConstantRadiusAssistedFS
{
public:
    double  m_dTowerAngle;  // Secant angle for surface 2

public:
    virtual ~SmEiffelTowerFS() {}

    SmEiffelTowerFS(const SmContext & crContext,
                    double dThisApproxTol3d,
                    double dAngleTolerance,
                    double dTangencyTolerance,
                    double dFilletRadius1,
                    double dFilletRadius2,
                    double dTowerAngle,
                    SmEdgeuse *pEdgeuse);

    virtual void GetName(TCHAR * pcMyName, size_t lMyNameAllocLen) {
        smos_WStrCpy(pcMyName, lMyNameAllocLen, _T("SmEffleTowerFS")); }

    // include slightly different FilletSolver function sets in a variety of derived SmEvalNFunctionsObjects
    virtual SmStatus LoadJacobian(ULONG & rlNumEquations, 
                                  ULONG & rlNumParameters,
                                  ULONG lRailIndex,
                                  SmSurface *& rpSurface1,
                                  ULONG & rlSurf1Offset,
                                  SmSurface *& rpSurface2,
                                  ULONG & rlSurf2Offset,
                                  const SmTArray<double> & crX, 
                                  SmTArray<double> & rF, 
                                  SmMatrix * pOptJacobian,
                                  SmBoolean & rbFoundAnswer);

    virtual SmStatus LoadInitialValues(ULONG & rbDoSurf2Calcs,
                                       ULONG lRailIndex,
                                       const SmSurface & crSurface1,
                                       ULONG lSurface1Index,
                                       SmSurface *& pSurface2,
                                       ULONG & lSurface2Index,
                                       SmExtentNd & rExtents,
                                       SmTArray<SmBoolean> & rPeriodicities,
                                       SmTArray<double> & rGuessT);

};


/*******************************************************************//**
PURPOSE: Create a fillet surface with 'Eiffel Tower' cross section.

NOTES: 
***********************************************************************/
class SM_EXPORT
SmEiffelTowerCrossSectionFSG : public SmFilletSurfaceGenerator
{
public:
    SmEiffelTowerCrossSectionFSG() {}
    virtual ~SmEiffelTowerCrossSectionFSG() {}
    virtual SmStatus CreateSurface(SmFilletSolver & rFilletSolver,
                                   SmBSplineCurve * pCenterLine,
                                   SmBSplineCurve * pRail1Curve,
                                   SmBSplineCurve * pRail2Curve,
                                   const SmTArray<SmTsectPnt*> & crFilletPoints,
                                   SmSurface *pSurf1,  
                                   SmBSplineCurve *pUV1,
                                   SmSurface *pSurf2,
                                   SmBSplineCurve *pUV2,
                                   SmBSplineSurface *& rpFilletSurface);
};

/*******************************************************************//**
PURPOSE: This fillet solver provides the equations for constant 
    radius filleting between two surfaces with one extra condition:
    The rolling ball will intersect with the surface which corresponds
    to the input edgeuse by a secant angle. On the other hand, the ball
    will still touch the other surface tangentially.

NOTES: 
***********************************************************************/
class SM_EXPORT SmSecantConstantRadiusFS : public SmConstantRadiusAssistedFS
{
public:
    double              m_dSecantAngle;  // Secant angle for surface 2

public:
    virtual ~SmSecantConstantRadiusFS() {}

    SmSecantConstantRadiusFS(const SmContext & crContext,
                             double dThisApproxTol3d,
                             double dAngleTolerance,
                             double dTangencyTolerance,
                             double dFilletRadius,
                             double dSecantAngle,
                             SmEdgeuse *pEdgeuse);

    virtual void GetName(TCHAR * pcMyName, size_t lMyNameAllocLen) {
        smos_WStrCpy(pcMyName, lMyNameAllocLen, _T("SmSecantConstantRadiusFS")); }

    // include slightly different FilletSolver function sets in a variety of derived SmEvalNFunctionsObjects
    virtual SmStatus LoadJacobian(ULONG & rlNumEquations, 
                                  ULONG & rlNumParameters,
                                  ULONG lRailIndex,
                                  SmSurface *& rpSurface1,
                                  ULONG & rlSurf1Offset,
                                  SmSurface *& rpSurface2,
                                  ULONG & rlSurf2Offset,
                                  const SmTArray<double> & crX, 
                                  SmTArray<double> & rF, 
                                  SmMatrix * pOptJacobian,
                                  SmBoolean & rbFoundAnswer);

    virtual SmStatus LoadInitialValues(ULONG & rbDoSurf2Calcs,
                                       ULONG lRailIndex,
                                       const SmSurface & crSurface1,
                                       ULONG lSurface1Index,
                                       SmSurface *& pSurface2,
                                       ULONG & lSurface2Index,
                                       SmExtentNd & rExtents,
                                       SmTArray<SmBoolean> & rPeriodicities,
                                       SmTArray<double> & rGuessT);

};

#endif // !__SMFILLETCUSTOMSOLVER_H__

