// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/******************************************************************//**
* FILE NAME --- SmBlendSurface.h
* PURPOSE: Header file for STEP Surface class.
**********************************************************************/

#ifndef __SMBLENDSURFACE_H__
#define __SMBLENDSURFACE_H__

#ifndef __SMOS_TYPES_H__
#include <SmTypes.h>
#endif

#ifndef __SMTARRAY_H__
#include <SmTArray.h>
#endif

class SmBSplineCurve;
class SmSurface;


/*******************************************************************//**
PURPOSE: Defines the cross boundary continuity type.

NOTES: 
***********************************************************************/
enum SmCrossBoundaryType 
{
    SM_CB_G0,  // Don't really care about cross boundary - no need to make curves for me
    SM_CB_G1,  // G1 curves
    SM_CB_G2,
    SM_CB_G3,
    SM_CB_G4,
    SM_CB_CIRCLE,
    SM_CB_LINE
};

/*******************************************************************//**
PURPOSE: This object defines a blend section curve used in blending or
   filleting.  The curve is defined by either a 3D curve or a 2D curve
   and a surface.

NOTES: The curves should be trimmed to desired lengths.  It is also
   assumed that matching curves are somewhat parameterized the same.
***********************************************************************/
class SM_EXPORT SmBlendSection 
{
    friend class SmBlendSurface;
private:
    SmCrossBoundaryType   m_eCrossBoundaryType;       // What is cross curve being constructed
    SmOrientType          m_eOrientation;             // Defines orientation of the curve relative to the normalized parameter
    const SmBSplineCurve* m_p3DCurve;                 // 3D curve - either a 3D curve or UV and Surface
    const SmBSplineCurve* m_pUVCurve;                 // UV curve which is defined to lie on the surface will be created if not given
    const SmSurface     * m_pSurface;                 // Surface Pointer
    SmSurfParamType       m_eSurfParam;               // Surface Param corresponding to more or less the direction of m_pUVCurve
    SmTArray<SmVector3d>  m_vStartCrossDerivs;        // Up to 3 cross derivitives used at start of curve
    SmTArray<SmVector3d>  m_vEndCrossDerivs;          // Up to 3 cross derivatives at end of curve
    SmTArray<SmVector3d>  m_vStartCrossDerivsInFrame; // Up to 3 cross derivitives used at start of curve in local frame
    SmTArray<SmVector3d>  m_vEndCrossDerivsInFrame;   // Up to 3 cross derivatives at end of curve in local frame
    
    // Unused
    // double                m_dStartTanVsPoint;         // Parameter opposite point falls when projected to 3D tangent
    // double                m_dEndTanVsPoint;  

public:
    SmBlendSection(const SmBSplineCurve *p3DCurve)
            : m_eCrossBoundaryType(SM_CB_G1), 
              m_p3DCurve(p3DCurve), 
              m_pUVCurve(NULL), 
              m_pSurface(NULL)
            {}

    SmBlendSection
    (
      const SmBSplineCurve *pUVCurve,          ///< [in] :   <br>
      const SmSurface *pSurface,               ///< [in] :   <br>
      const SmBSplineCurve *pOpt3DCurve=NULL,  ///< [in] :   <br>
      SmSurfParamType eSurfParam=SM_SP_U)      ///< [in] :   <br>
      : m_eCrossBoundaryType(SM_CB_G1), 
        m_p3DCurve(pOpt3DCurve), 
        m_pUVCurve(pUVCurve), 
        m_pSurface(pSurface), 
        m_eSurfParam(eSurfParam)
    {}

    void SetCrossDerivs
    (
      ULONG lNumDerivs,                    ///< [in] :   <br>
      SmVector3d *pStartDerivs,            ///< [in] :   <br>
      SmVector3d *pEndDerivs               ///< [in] :   <br>
    );

    SmStatus ComputeFrameAt
    (
      double dNormalizedParameter,          ///< [in] :   <br> 
      SmPoint3d & rOrigin,
      SmVector3d & rCurveTangent,
      SmVector3d & rXAxis,
      SmVector3d & rYAxis,
      SmBlendSection *pOptMateSection=NULL  ///< [in] : pOptMatingSection = If given it will be used to align the Y of the local axis.  
                                            ///<       It will override the tangent of curve being the local frame
    ) const;

    SmStatus ComputeCrossConstraints
    (
      double dNormalizedParam,                      ///< [in] :   <br>
      SmBoolean bUseCrossCurveDerivs,               ///< [in] :   <br>
      SmTArray<SmPoint3d> & rPoints,
      SmTArray<SmVector3d> & rTangents,
      SmTArray<SmVector3d> & rHigherOrderDerivs,
      SmBlendSection *pOptMateSection=NULL
    ) const;

    SmStatus ComputeIsoCrossConstraints
    (
      double dNormalizedParameter,                 ///< [in] :   <br>
      ULONG lCrossSection,                         ///< [in] :   <br>
      const SmPoint2d & rUV,
      SmTArray<SmPoint3d> & rPoints, 
      SmTArray<SmVector3d> & rTangents,
      SmTArray<SmVector3d> & rHigherOrderDerivs
    );

    SmStatus ComputeLocalFrameVectors
    (
      double dNormalizedParameter,                ///< [in] :   <br>
      ULONG lNumVectors,                          ///< [in] :   <br>
      SmVector3d * pEuclidVectors,
      SmVector3d * pLocalFrameVectors
    );

    SmStatus ComputeLocalStartEndInFrame();

    SmStatus ComputeLocalFrameDerivatives
    (
      double dParameter,                            ///< [in] :   <br>
      SmTArray<SmVector3d> & rDerivs
    ) const;

    SmStatus ComputeSectionCrossConstraints
    (
      double dNormalizedParam,                      ///< [in] :   <br>
      const SmVector3d & crPlaneNormal,             ///< [in] :   <br>
      SmTArray<SmPoint3d> & rPoints, 
      SmTArray<SmVector3d> & rTangents,
      SmTArray<SmVector3d> & rHigherOrderDerivs
    ) const;

    SmBoolean IsSpanAccurate
    (
      double dStart,                                 ///< [in] :   <br>
      const SmTArray<SmVector3d> & crStartVectors,   ///< [in] :   <br>
      double dEnd,                                   ///< [in] :   <br>
      SmTArray<SmVector3d> & rEndVectors,            ///< [in] :   <br>
      const SmTArray<double> & crTolerances          ///< [in] :   <br>
    );

    SmStatus Evaluate
    (
      double dNormalizedParameter,          ///< [in] :   <br>
      ULONG lNumberDerivatives,             ///< [in] :   <br>
      SmPoint3d & rPoint,
      SmTArray<SmVector3d> & rDerivatives,
      double * pdArcLength = NULL
    ) const;

    SmStatus EvaluateUV
    (
      double dNormalizedParameter,        ///< [in] :   <br>
      ULONG lNumberDerivatives,           ///< [in] :   <br>
      SmPoint3d & rPoint,                 ///< [out]:   <br>
      SmTArray<SmVector3d> & rDerivatives ///< [out]:   <br>
    ) const;

    void Dump(void) const;

    SmDisplayList * Draw(void) const;

} ; // end class SmBlendSection

/*******************************************************************//**
PURPOSE: This object is a surface which represents a fillet or blend
   surface.  It defines a blending between 4 boundary curves which may lie 
   on 4 surfaces.  The boundary curves define continuity information.

        1) Left and right profile are parameterized from the bottom to the top
        2) Left and right curve are parameterized from the bottom curve to the top

NOTES: Note that the curves need to be defined using the correct
   orientation.  A static method for doing that is shown below.
***********************************************************************/
class SM_EXPORT SmBlendSurface 
{
private:
    SmBlendSection * m_pLeftProfile;
    SmBlendSection * m_pRightProfile;
    SmBlendSection * m_pBottomRail;
    SmBlendSection * m_pTopRail;

    ULONG            m_lNumberRailSteps;       // If specified all tolerances are ignored and uniform
                                               // steps are used for Rail Approximation
    ULONG            m_lNumberProfileSteps;    // If specified all tolerances are ignored for steps 
                                               // crosswise along the other curves.
    SmBoolean        m_bUseCrossCurveDerivs;   // Use derivatives from cross curves instead of surface
                                               // as higher order blending. This should be used for circular
                                               // blends.
    SmBoolean        m_bUCurvesAreRails;       // If TRUE constant U curves are the Rails
    SmTArray<double> m_vRailKnotsNormalized;   // Normalized rail knots if specified will force
                                               // cross sections at these intermediate points.
    double           m_d3DTolerance;

    // Unused
    // double           m_dG1AngleToleranceDeg;   
    // double           m_dG2RadialTolerance;
    // double           m_dG3VectorTolerance;

    SmTArray<SmBSplineCurve*> m_vProfiles;    // profile curves defined along blend
    SmTArray<SmBSplineCurve*> m_vCrossCurves; // cross section curves going orthoginal to profile curves

public:
    // Construction
    SmBlendSurface
    (
      SmBlendSection *pLeftProfile,         ///< [in] :   <br>
      SmBlendSection *pRightProfile,        ///< [in] :   <br>
      SmBlendSection *pBottomRail,          ///< [in] :   <br>
      SmBlendSection *pTopRail,             ///< [in] :   <br>
      double d3DTolerance,                  ///< [in] :   <br>
      double dG1AngleToleranceDeg = 2.0,    ///< NotUsed: [in] :   <br>
      double dG2RadialTolerance = 0,        ///< NotUsed: [in] :   <br>
      double dG3VectorTolerance = 0         ///< NotUsed: [in] :   <br>
    );

    ~SmBlendSurface();

    SmStatus ApproximateBoundaryCurves
    (
      const SmContext & crContext,       ///< [in] :   <br>
      SmBlendSection *pSec1,             ///< [in] :   <br>
      ULONG lNumberSteps,                ///< [in] :   <br>
      SmBSplineCurve *& rpSec1Curve,     ///< [out]:   <br>
      double & rdAccuracy                ///< [out]:   <br>
    );

    SmStatus ApproximateBoundaryCurves
    (
      const SmContext & crContext,        ///< [in] :   <br>
      SmBlendSection *pSec1,              ///< [in] :   <br>
      SmBlendSection *pSec2,              ///< [in] :   <br>
      ULONG lNumberSteps,                 ///< [in] :   <br>
      SmBSplineCurve *& rpSec1Curve,      ///< [out]:   <br>
      SmBSplineCurve *& rpSec2Curve       ///< [out]:   <br>
    );

    SmStatus ApproximateSectionBoundaryCurves
    (
      const SmContext & crContext,         ///< [in] :   <br>
      const SmVector3d & crSectionPlane,   ///< [in] :   <br>
      SmBoolean bAdjustPlane,              ///< [in] :   <br>
      SmBlendSection *pSec1,               ///< [in] :   <br>
      SmBlendSection *pSec2,               ///< [in] :   <br>
      ULONG lNumberSteps,                  ///< [in] :   <br>
      SmBSplineCurve *& rpSec1Curve,       ///< [out]:   <br>
      SmBSplineCurve *& rpSec2Curve        ///< [out]:   <br>
    );

    SmStatus ApproximateIsoBoundaryCurve
    (
      const SmContext & crContext,        ///< [in] :   <br>
      SmBlendSection *pSec1,              ///< [in] :   <br>
      ULONG lNumberDivisions,             ///< [in] :   <br>
      SmBSplineCurve *& rpSec1Curve,      ///< [out]:   <br>
      SmBSplineCurve *& rpSec1UVCurve,    ///< [out]:   <br>
      double & rdAccuracy                 ///< [out]:   <br>
    );

    SmStatus ApproximateIsoSteppingBoundaryCurves
    (
      const SmContext & crContext,         ///< [in] :   <br>
      SmBlendSection *pSec1,               ///< [in] :   <br>
      SmBlendSection *pSec2,               ///< [in] :   <br>
      ULONG lNumberSteps,                  ///< [in] :   <br>
      ULONG lNumberDivisions,              ///< [in] :   <br>
      SmBSplineCurve *& rpSec1Curve,       ///< [out]:   <br>
      SmBSplineCurve *& rpSec2Curve,       ///< [out]:   <br>
      SmBSplineCurve *& rpSec1UVCurve,     ///< [out]:   <br>
      SmBSplineCurve *& rpSec2UVCurve      ///< [out]:   <br>
    );

    SmStatus CreateIsoSteppingProfileCurves(const SmContext & crContext);

    SmStatus CreateIsoSteppingBlend
    (
      const SmContext & crContext,          ///< [in] :   <br>
      SmBSplineSurface *& rpBlendingSurface ///< [out]:   <br>
    );

    SmStatus CreateProfileCurves(const SmContext & crContext);

    SmStatus CreateCrossCurves(const SmContext & crContext);

    SmStatus CreateDoubleBlendSurface
    (
      const SmContext & crContext,           ///< [in] :   <br>
      SmBSplineSurface *& rpBlendingSurface  ///< [out]:   <br>
    );

    SmStatus CreateOrientedSweepProfileCurves
    (
      const SmContext & crContext,           ///< [in] :   <br>
      SmBoolean bAdjustNormal                ///< [in] :   <br>
    );

    SmStatus CreateTangentOrientedSweep
    (
      const SmContext & crContext,            ///< [in] :   <br>
      SmBoolean bAdjustNormal,                ///< [in] :   <br>
      SmBSplineSurface *& rpBlendingSurface
    );

    void SetNumberRailSteps(ULONG lNumSteps) { m_lNumberRailSteps = lNumSteps; }

    void SetNumberProfileSteps(ULONG lNumSteps) { m_lNumberProfileSteps = lNumSteps; }

    // Internal - non-public methods
    // Define  GetType(), IsKindOf(), and virtual Dump() (Dump needs local implementation)
//    SM_COMMON(SmBlendSurface,SmSurface,SmBlendSurface_TYPE);
};


#endif // !__SMBLENDSURFACE_H__


