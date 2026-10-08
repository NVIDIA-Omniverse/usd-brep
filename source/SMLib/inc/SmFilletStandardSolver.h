// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*******************************************************************//**
* FILE NAME --- SmFilletStandardSolver.h 
* PURPOSE: Header file for SmFilletSolver object.
**********************************************************************/

#ifndef __SMFILLETSTANDARDSOLVER_H__
#define __SMFILLETSTANDARDSOLVER_H__

#ifndef __SMFILLETSOLVER_H__
#include <SmFilletSolver.h>
#endif

/*******************************************************************//**
PURPOSE: This fillet solver provides the equations for constant 
    radius filleting between two surfaces.

NOTES: 
***********************************************************************/
class SM_EXPORT SmConstantRadiusFS : public SmFilletSolver
{
protected:
    double            m_dFilletRadii[2]; // Radii of offset for each surface.

public:
    virtual ~SmConstantRadiusFS();

    SmConstantRadiusFS(const SmContext & crContext,
                       double            dThisApproxTol3d,
                       double            dAngleTolerance,
                       double            dTangencyTolerance,
                       double            dFilletRadius,
                       SmEdgeuse       * pEdgeuse,
                       double          * pdOptSecondRadius = NULL);

    SmConstantRadiusFS(const SmContext & crContext,
                       double dThisApproxTol3d,
                       double dAngleTolerance,
                       double dTangencyTolerance,
                       double dOffsetRadiusSurface1,  
                       double dOffsetRadiusSurface2,  
                       const SmSurface & crSurface1, // in : origSurf to fillet - stored in new OffsetSurface
                       const SmSurface & crSurface2, // in : origSurf to fillet - stored in new OffsetSurface
                       SmBoolean dOrientationSurface1,
                       SmBoolean dOrientationSurface2);

    // virtual SmBoolean CanMakeAnalytic();  // use base class version.

    virtual ULONG GetJacobianSize() const;

    virtual void GetName(TCHAR * pcMyName, size_t lMyNameAllocLen) {
        smos_WStrCpy(pcMyName,lMyNameAllocLen,_T("SmConstantRadiusFS")); }

    double GetFilletRadius(ULONG lIndex) { return m_dFilletRadii[lIndex]; }

    virtual SmFilletSolverType GetSolverType() { return SM_FS_CONST_RADIUS; }

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

    virtual SmBoolean OffsetRadiiCanChange() { return FALSE; }

    virtual SmStatus SetupOffsetValues(const SmTsectPnt &rTsectPnt,
                                       double * pOffsetDist = NULL);

    SM_COMMON( SmConstantRadiusFS, SmFilletSolver, SmConstantRadiusFS_TYPE );

} ; // end class SmConstantRadiusFS

/*******************************************************************//**
PURPOSE: This fillet solver provides the equations for constant 
    radius filleting between two surfaces with two different offset
    surfaces

NOTES: 
***********************************************************************/
class SM_EXPORT SmConstantRadiusAssistedFS : public SmConstantRadiusFS
{
protected:
    SmConstantRadiusAssistedFS(const SmContext & crContext,
                               double dThisApproxTol3d,
                               double dAngleTolerance,
                               double dTangencyTolerance,
                               double dFilletRadius1,
                               double dFilletRadius2,
                               SmEdgeuse *pEdgeuse);

    virtual ~SmConstantRadiusAssistedFS() {}

    virtual SmFilletSolverType GetSolverType() { return SM_FS_CONST_RADIUS_ASSISTED; }

    SmStatus GuessPointOnPlaneSolve(const SmPoint3d  & crPlaneOrig,
                                    const SmVector3d & crPlaneNormal,
                                    const SmExtent2d & crUVDomain1,
                                    const SmExtent2d & crUVDomain2,
                                    const SmVector2d & rUV1,
                                    const SmVector2d & rUV2,
                                    SmBoolean        & rbFoundSolution,
                                    SmTsectPnt       & rTsectPnt);

    virtual SmStatus PointOnPlaneSolve(const SmPoint3d  & crPlaneOrig,     // in :
                                       const SmVector3d & crPlaneNormal,   // in :
                                       const SmExtent2d & crUVDomain1,     // in :
                                       const SmExtent2d & crUVDomain2,     // in :
                                       const SmVector2d & rUV1,            // in :
                                       const SmVector2d & rUV2,            // NotUsed: in :
                                       SmBoolean        & rbFoundSolution, // out:
                                       SmTsectPnt       & rTsectPnt);      // out:

    SM_COMMON( SmConstantRadiusAssistedFS, SmConstantRadiusFS, SmConstantRadiusAssistedFS_TYPE );

} ; // end class SmConstantRadiusAssistedFS

/*******************************************************************//**
PURPOSE: This fillet solver provides a rolling ball fillet between
    two surfaces which do not necessarily share a common edge.  

NOTES: 
***********************************************************************/
class SM_EXPORT SmSurfaceSurfaceFS : public SmConstantRadiusFS
{
protected:
    SmPoint2d               m_vUVGuesses[2];  // Start point guess for tracing fillet.
    SmBoundaryTrimmingType  m_eTrimType;      // How to trim the resulting fillet surface.

public:
    virtual ~SmSurfaceSurfaceFS();

    SmSurfaceSurfaceFS
      (const SmContext & crContext,             // in : Creation context for new geometry 
       double            dThisApproxTol3d,     // in : Tolerance used to specify maximal distance between 3D
                                                //      rail curves and the surface on which it lies.
       double            dAngleTolerance,       // in : The largest angle that the rail curves can 
                                                //      traverse before generating a new knot.  
       double            dTangencyTolerance,    // in : Angle used to determine quality of tangency angle 
                                                //      between fillet surface and rail surfaces.
       double            dOffsetRadiusSurface1, // in : Radius of the rolling ball used to generate the fillet
       double            dOffsetRadiusSurface2, //      for each surface.  Note that the radii are signed and  
                                                //      correspond to the offset direction of the surface that 
                                                //      is used to generate the centerline curve of the fillet.
       const SmSurface & crSurface1,            // in : Underlying Orig surface1, stored as BaseSurface of new OffsetSurfaces
       const SmSurface & crSurface2,            // in : Underlying Orig surface2, stored as BaseSurface of new OffsetSurfaces
       SmBoolean         dOrientationSurface1,  // in : offset orientation for Sur1
       SmBoolean         dOrientationSurface2,  // in : offset orientation for Sur2
       const SmPoint2d & crUVGuessSurface1,     // in : UV position to start rolling the ball on pSur1.
       const SmPoint2d & crUVGuessSurface2);    // in : UV position to start rolling the ball on pSur2.  
                                                //      note that these points do not have to lie exactly on the  
                                                //      rails but should be near them.
    virtual SmStatus CalcFilletGeom();

    virtual void GetName(TCHAR * pcMyName, size_t lMyNameAllocLen) {
        smos_WStrCpy(pcMyName,lMyNameAllocLen,_T("SmSurfaceSurfaceFS")); }

    virtual SmFilletSolverType GetSolverType() { return SM_FS_SURF_SURF; }

    void SetFilletTrimType(SmBoundaryTrimmingType eTrimType) { m_eTrimType = eTrimType; }

    virtual void SetupOffsetExtension(const SmSurface * pSurf, SmExtent2d & rDomain) ;

    SM_COMMON( SmSurfaceSurfaceFS, SmConstantRadiusFS, SmSurfaceSurfaceFS_TYPE );

} ; // end class SmSurfaceSurfaceFS

/*******************************************************************//**
PURPOSE: This fillet solver provides the equations for constant 
    distance fillet/chamfer between two surfaces.  All cross sections
    will have the same length reguardless of the angle between the 
    faces along the edge.  

NOTES: Note that a thumb-nail fillet can be created by using 
    a circular profile instead of a line segment.
***********************************************************************/
class SM_EXPORT SmConstantDistanceFS : public SmFilletSolver
{
protected:
    double            m_dDistance;  // specified distance between corresponding points on rail curves

public:
    virtual ~SmConstantDistanceFS();

    SmConstantDistanceFS(const SmContext & crContext,
                         double dThisApproxTol3d,
                         double dAngleTolerance,
                         double dTangencyTolerance,
                         double dDistance,
                         SmEdgeuse *pEdgeuse);

    // Override the default (which says it can make an analytic
    // if the radius cannot change), because with this type,
    // the radius could change, but might not, depending
    // on the geometry.
    virtual SmBoolean CanMakeAnalytic() { return TRUE; }

    double  GetDistance() { return m_dDistance; }

    virtual ULONG GetJacobianSize() const;

    virtual void GetName(TCHAR * pcMyName, size_t lMyNameAllocLen) {
        smos_WStrCpy(pcMyName,lMyNameAllocLen,_T("SmConstantDistanceFS")); }

    virtual SmFilletSolverType GetSolverType() { return SM_FS_CONST_DIST; }

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

    virtual SmStatus SetupOffsetValues(const SmTsectPnt &rTsectPnt,
                                       double * pOffsetDist = NULL);

    SM_COMMON( SmConstantDistanceFS, SmFilletSolver, SmConstantDistanceFS_TYPE );

} ; // end class SmConstantDistanceFS

/*******************************************************************//**
PURPOSE: This is an abstract class for all fillet solvers which depend
     on an edge or curve to either determine stepping or utilize a
     law.  The main purpose of this is allow the curve belonging to the
     edge to be extended.

NOTES: This method assumes that the base curve has G1 or better continuity.  
***********************************************************************/
class SM_EXPORT SmCurveBasedFS : public SmFilletSolver
{
protected:
    const SmCurve  * m_pOriginal;  // Curve of Edge being filleted: not owned by us.
    SmBSplineCurve * m_pExtended;  // Extended Edge curve, if not Null.

    virtual SmStatus EvaluateBaseCurve(double dParameter,
                                       ULONG lNumDerivatives,
                                       SmVector3d aPointAndDerivatives[]);

public:
    virtual ~SmCurveBasedFS();
    SmCurveBasedFS(const SmContext & crContext,
                   double dThisApproxTol3d,
                   double dAngleTolerance,
                   double dTangencyTolerance)
        : SmFilletSolver(crContext,dThisApproxTol3d,dAngleTolerance,dTangencyTolerance),
        m_pOriginal(NULL), m_pExtended(NULL) {}


    virtual void GetName(TCHAR * pcMyName, size_t lMyNameAllocLen) {
        smos_WStrCpy(pcMyName,lMyNameAllocLen,_T("SmCurveBasedFS")); }

    const SmCurve * GetOriginalCurve() { return m_pOriginal; }

    virtual void SetOriginalCurve( SmCurve * pNewCurve ) { m_pOriginal = pNewCurve; }

    virtual SmFilletSolverType GetSolverType() { return SM_FS_CURVE_BASED; }

    SM_COMMON( SmCurveBasedFS, SmFilletSolver, SmCurveBasedFS_TYPE );

} ; // end class SmCurveBasedFS

/*******************************************************************//**
PURPOSE: This fillet solver provides the equations for variable
    radius fillet/chamfer between two surfaces.  The radius is governed
    by a law applied to the parameter along the edge.

NOTES: 
***********************************************************************/
class SM_EXPORT SmVariableRadiusFS : public SmCurveBasedFS
{
protected:
    SmFilletLaw * m_rLaw;              // Object which produces the law map for 
                                       // the parameter values of the edge.
                                       // deleted when this SmVariableRadiusFS is deleted.
    SmBoolean     m_bLawOrientation;   // If TRUE we reverse the orientation of the law
                                       // relative to the edge.   FALSE both are same.
    SmExtent1d    m_vLawInterval;

public:
    virtual ~SmVariableRadiusFS();

    SmVariableRadiusFS(const SmContext & crContext,
                       double            dThisApproxTol3d,
                       double            dAngleTolerance,
                       double            dTangencyTolerance,
                       double            dDistance,
                       SmEdgeuse       * pEdgeuse,
                       SmFilletLaw     & rFilletLaw,
                       SmBoolean         bLawOrientation);

    SmVariableRadiusFS(const SmContext & crContext,
                       double            dThisApproxTol3d,
                       double            dAngleTolerance,
                       double            dTangencyTolerance,
                       SmSurface       * pSurface,
                       SmSurface       * pSurface2,
                       double            dOffsetOrientation,
                       double            dOffsetOrientation2,
                       SmCurve         * pCenterCurve,
                       SmFilletLaw     & rFilletLaw,
                       SmBoolean         bLawOrientation);
    
    SmStatus ReCalcFilletGeom(SmBoolean    bIsAnalyticFillet, // in : 
                              SmVector3d * pOptMarchDir,      // NotUsed: in : 
                              SmCurve    * pOptRefCurve);     // NotUsed: in : 

    // Member access
    SmFilletLaw * GetFilletLaw()            { return m_rLaw; }
    SmBoolean     GetFilletLawOrientation() { return m_bLawOrientation; }
    SmExtent1d    GetFilletLawInterval()    { return m_vLawInterval; }


    void SetFilletLaw            ( SmFilletLaw *pLaw      ) { m_rLaw = pLaw; }
    void SetFilletLawOrientation ( SmBoolean bOrient      ) { m_bLawOrientation = bOrient; }
    void SetFilletLawInterval    ( const SmExtent1d &rIvl ) { m_vLawInterval = rIvl; }

    double GetFilletRadius(double dParam);

    // virtual SmBoolean CanMakeAnalytic();  // use base class version.

    virtual SmBoolean OffsetRadiiCanChange();

    virtual ULONG GetJacobianSize() const;

    virtual void GetName(TCHAR * pcMyName, size_t lMyNameAllocLen) { smos_WStrCpy(pcMyName,lMyNameAllocLen,_T("SmVariableRadiusFS")); }

    virtual SmFilletSolverType GetSolverType() { return SM_FS_VARIABLE_RADIUS; }

    // include slightly different FilletSolver function sets in a variety of derived SmEvalNFunctionsObjects
    virtual SmStatus LoadJacobian(ULONG                  & rlNumEquations, 
                                  ULONG                  & rlNumParameters,
                                  ULONG                    lRailIndex,
                                  SmSurface             *& rpSurface1,
                                  ULONG                  & rlSurf1Offset,
                                  SmSurface             *& rpSurface2,
                                  ULONG                  & rlSurf2Offset,
                                  const SmTArray<double> & crX, 
                                  SmTArray<double>       & rF, 
                                  SmMatrix               * pOptJacobian,
                                  SmBoolean              & rbFoundAnswer);

    virtual SmStatus LoadInitialValues(ULONG               & rbDoSurf2Calcs,
                                       ULONG                 lRailIndex,
                                       const SmSurface     & crSurface1,
                                       ULONG                 lSurface1Index,
                                       SmSurface          *& pSurface2,
                                       ULONG               & lSurface2Index,
                                       SmExtentNd          & rExtents,
                                       SmTArray<SmBoolean> & rPeriodicities,
                                       SmTArray<double>    & rGuessT);

    virtual void SetExtensionFlags();

    virtual SmStatus SetupOffsetValues(const SmTsectPnt & rTsectPnt,
                                       double           * pOffsetDist = NULL);

    SM_COMMON( SmVariableRadiusFS, SmCurveBasedFS, SmVariableRadiusFS_TYPE );

} ; // end class SmVariableRadiusFS

#endif // !__SMFILLETSTANDARDSOLVER_H__


