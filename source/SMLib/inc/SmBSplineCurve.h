// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/******************************************************************//**
* FILE NAME --- SmBSplineCurve.h
* PURPOSE: Header file for Curve class.
**********************************************************************/


#ifndef __SMBSPLINECURVE_H__
#define __SMBSPLINECURVE_H__

#ifndef __SMCURVE_H__
#include <SmCurve.h>
#endif
 
#ifndef __SMGFX_EXTERN_H__
#include <SmGraphicsExtern.h>
#endif

#ifndef __Sm_Nurbs_CRV_H__
#include <SmNurbsCrv.h>
#endif

class SmSurface ;
class SmHermiteCurve ;
class SmDatabaseIO ;

/*******************************************************************//**
PURPOSE: This object is a curve which provides an implementation for
    NURBS curves.  This is the primary curve type used in the system.

NOTES: The Nurb/Nub curve is stored in m_pNurb.
  For BSplines the relationship between
    m = number of knots
    n = number of control points
    p = degree

    is m = n + p + 1 ;

  In SMLib, Nurb/Nub Curves are expected to start and end
  with fully multiple knots, so a typical knot vector will be
  of the form

    U = {a,...,a,u[p+1],...,u[m-1 - p-1],b,...,b}
         |_____|                         |_____|
           p+1                             p+1
  where any internal knot value may be duplicated up
  to degree, p, times.
***********************************************************************/
class SM_EXPORT SmBSplineCurve : public SmCurve
{
  friend class SmCurveCache;
  friend class SmTree;
  
protected:
  gw_CURVE          * m_pNurb = NULL ;                          // ptr to bspline data of type gw_CURVE, sized:[sm_ComputeNurbCurveSize()]
  SmBoolean           m_bNurbIsBorrowed = UNSURE ;              // TRUE = m_pNurb not deleted when this object is deleted
                                                                // FALSE= m_pNurb  is deleted when this object is deleted
                                                                
  SmBoolean           m_bOutOfBoundsEnabled = UNSURE ;          // GWC:OUTOFBOUNDS MODIFICATION
                                                                // TRUE=return evaluations for uv points beyond domain
                                                                // default:[FALSE]
  SmKnotType          m_eKnotType = SM_KT_UNKOWN ;
  SmBSplineCurveForm  m_eBSplineCurveForm = SM_CF_UNSPECIFIED ; // Set by Derived Class Objects
                                                                // oneof: SM_CF_POLYLINE_FORM,
                                                                //        SM_CF_CIRCULAR_ARC,
                                                                //        SM_CF_ELLIPTIC_ARC,
                                                                //        SM_CF_PARABOLIC_ARC,
                                                                //        SM_CF_HYPERBOLIC_ARC,
                                                                //        SM_CF_HELICAL_ARC,
                                                                //        SM_CF_UNSPECIFIED     - [default]
  
public:
  // Constructors - note: Std Constructor is implemented 3 times for ptr type compatability
  // Construct Curve with no NURB object
  SmBSplineCurve(ULONG lDim=3) ;                ///< [in] :
  
  SmBSplineCurve(gw_CURVE        * pAlreadyAllocatedNurb,    ///< [in] : Fast constructor, stores pAlreadyAllocatedNurb NURB object            <br>
                 ULONG             lDim,                     ///< [in] :                                                                       <br>
                 SmBoolean         bNurbIsBorrowed = FALSE,  ///< [in] : TRUE=m_pNurb not deleted when this object is deleted,FALSE=is deleted <br>
                 const SmContext * pContext = NULL) ;        ///< [in] :
  
  SmBSplineCurve(ULONG             lDim,                     ///< [in] : dimension (2 or 3)                    <br>
                 ULONG             n,                        ///< [in] : high index of Pw                      <br>
                 ULONG             m,                        ///< [in] : high index of U, note: m = n + p + 1  <br>
                 short             p,                        ///< [in] : degree                                <br>
                 gw_CPOINT       * Pw,                       ///< [in] : Control polygon                       <br>
                 double          * U,                        ///< [in] : knots                                 <br>
                 const SmContext * pContext = NULL) ;        ///< [in] : context for new object construction   <br>
  
  // Std Constructor, copies gw_CURVE NURB object
  SmBSplineCurve(ULONG lDim, const gw_CURVE* cpGwNurb) ;
  
  // Copy Constructor
  SmBSplineCurve(const SmBSplineCurve & crSourceCurve) ;
  
  // destructor
  virtual ~SmBSplineCurve() ;
  
  // equality operator
  virtual SmBoolean operator==(const SmCurve &crOther) const ;
  
  // Public methods
  
  // Select a different section of the curve
  virtual SmStatus AdjustSTEPInterval(const SmExtent1d & crNewSTEPInterval) ;
  
  // approximate an arc with a non-rational degree 3 BSpline Curve
  static SmStatus ApproximateArc(const SmContext        & crContext,                   ///< [in] : context for new object construction                                 <br>
                                 ULONG                    lDimensionOfResult,          ///< [in] : image space size, 2 or 3                                            <br>
                                 ULONG                  * plOptNumberOfControlPoints,  ///< [in] : CtrlPoint count in new Curve                                        <br>
                                 const SmAxis2Placement & crReferenceFrame,            ///< [in] : Specify start and orientation of arc                                <br>
                                                                                       ///<        Angles measured from XAxis in X/Y plane in CCW direction            <br>
                                 double                   dRadius,                     ///< [in] : output Curve radius                                                 <br>
                                 double                   dStartAngle,                 ///< [in] : output Curve start angle in degrees                                 <br>
                                 double                   dEndAngle,                   ///< [in] : output Curve end angle in degrees                                   <br>
                                 double                   dTolPercentRadius,           ///< [in] : max allowed percentage deviation of output curve from circular arc  <br>
                                 SmBSplineCurve        *& rpNewCurve) ;                ///< [out]: new curve                                                           <br>
  
  // approximate a sequence of points with a non-rational BSplineCurve [CtrlPtCount <= SamplePtCount]
  //  when given an optional tol, iterate from a degree 2 to a lDegree curve until tolerances are xd after knot removal.
  //       Can return a curve with fewer CtrlPts than SamplePts.
  //  without optional tol, build and return the best fit lDegree curve [CtrlPtCount == SamplePtCount].
  static SmStatus ApproximatePoints(const SmContext           & crContext,        ///< [in] : context for new object construction                                <br>
                                    const SmTArray<SmPoint3d> & crPoints,         ///< [in] : array of ordered points to approximate                             <br>
                                    ULONG                       lDegree,          ///< [in] : degree of created approximating BSplineCurve                       <br>
                                    SmVector3d                * pOptStartTangent, ///< [in] : Start Tangent, NULL to ignore                                      <br>
                                    SmVector3d                * pOptEndTangent,   ///< [in] : End Tangent, NULL to ignore                                        <br>
                                    SmBoolean                   bIsClosedCurve,   ///< [in] : TRUE = create closed curve, FALSE = don't                          <br>
                                    double                    * pdOptTolerance,   ///< [in] : max distance between points and created curve, NULL to ignore      <br>
                                    SmBSplineCurve           *& rpNewCurve) ;     ///< [out]: The new curve                                                      <br>
  
  // approximate a rational BSpline Curve with a degree 3 non-rational one preserving
  //   original curve parameterization via sampling the original curve and interpolating those sample points.
  SmStatus ApproximateRationalCurve(const SmContext & crContext,        ///< [in] : context for new object construction             <br>
                                    double            dThisApproxTol3d, ///< [in] : max distance between approximate and this curve <br>
                                    SmBSplineCurve *& rpNewCurve) ;     ///< [out]: the approximated curve                          <br>
  
  // Exposes N_FitCrvApproxPts()
  SmStatus ApproximateRefit(const SmTArray< SmPoint3d > & sApproxPts,   ///< [in] :   <br>
                            SmTArray< ULONG  > * pFixIndices = NULL,    ///< [in] :   <br>
                            SmTArray< double > * pFixParams  = NULL,    ///< [in] :   <br>
                            ULONG  lFixEndsFlag = 0,                    ///< [in] :   <br>
                            double dAlpha0 = -1, double dAlpha1 = -1,   ///< [in] :   <br>
                            double dBeta0  = -1, double dBeta1  = -1,   ///< [in] :   <br>
                            SmTArray< double > * pErrors = NULL) ;      ///< [in] :   <br>
  
  static SmStatus C1NonRationalCubicInterpolate(const SmContext           & crContext,      ///< [in] :   <br>
                                                const SmTArray<SmPoint3d> & crPointsList,   ///< [in] :   <br>
                                                SmBSplineCurve           *& rpNewCurve) ;   ///< [in] :   <br>
  
  virtual SmStatus CalculateBoundingBox(const SmExtent1d & crInterval,         ///< [in] : target interval                               <br>
                                         SmExtent3d       * pNormalBox = NULL,  ///< [out]: Axis aligned box                              <br>
                                         SmPseudoBox      * pPseudoBox = NULL,  ///< [out]: Non-axis aligned box                          <br>
                                         SmPolarBox       * pPolarBox = NULL,   ///< [out]: Surface normal vector field bounding box      <br>
                                         SmBoolean          bExpandBox = TRUE)  ///< [in] : bExpandBox = not-used                         <br>
                                        const ;                                 ///<        TRUE = Expand BBox before return FALSE= don't <br>
  
  virtual SmStatus CalculateContinuities(SmContinuityType           & reMinContinuityInCurve,                    ///< [in] :    <br>
                                         SmTArray<SmContinuityType> & rContinuitiesAtKnots,                      ///< [out]:    <br>
                                         double                       dContinuityAngleTol = SM_CONTINUITY_ANGLE) ///< [in] :    <br>
                                        const ;
  
  virtual SmStatus ConvertTo2D() ;
  
  virtual SmStatus ConvertTo3D() ;
  
  // make an exact copy of any curve
  virtual SmStatus Copy(const SmContext & crContext,     ///< [in] :   <br>
                        SmCurve        *& rpNewCurve)    ///< [out]:   <br>
                       const ;
  
  // make an exact copy of any curve
  SmStatus Copy(const SmContext& crContext,    ///< [in] :   <br>
                SmBSplineCurve*& rpNewCurve)   ///< [out]:   <br>
               const ;
  
  // when possible make a SmBSpline Copy of an Analytic Curve - else set output to NULL
  virtual SmStatus CopyAnalyticAsNurb(const SmContext & crContext,       ///< [in] :  <br>
                                      SmCurve        *& rpNewNurbCurve)  ///< [out]:  <br>
                                     const ;
  
  // curve is already a BSpline - make and return an exact copy
  virtual SmStatus MakeExactBSplineIfPossible(SmBSplineCurve *& rpNewBSplineCurve) const { SmStatus sRtn  = Copy(*GetContext(), (SmCurve *&)rpNewBSplineCurve) ;
                                                                                           return(sRtn) ; 
                                                                                         }
  
  // when possible make an analytic copy of a BSplineCurve - else make exact copy
  virtual SmStatus CreateAnalyticCurve(const SmContext  & crContext,   ///< [in] : context for construction               <br>
                                       const SmExtent1d & crInterval,  ///< [in] : interval for review                    <br>
                                       SmCurve         *& rpNewCurve)  ///< [out]: Set to new SmLine or SmCircle when     <br>
                                                                       ///<        this curve can be represented by such  <br>
                                                                       ///<        otherwise set to NULL.                 <br>
                                      const ;
  
  // approximate set of points with optional 1st, 2nd, and/or 3rd end curve derivative constraints
  static SmStatus CreateApproximatingCurve(const SmContext            & crContext,                   ///< [in] : context for new object construction                                                              <br>
                                           ULONG                        lNumPoints,                  ///< [in] : Number of control points in new curve                                                            <br>
                                           ULONG                        lParameterization,           ///< [in] : 0 - NL_UNIFORM, 1 = NL_CHORDLENGTH, 2 - NL_CENTRIPETAL                                                    <br>
                                           ULONG                        lDimensionOfResult,          ///< [in] : Must be 2 or 3                                                                                   <br>
                                           ULONG                        lDegree,                     ///< [in] : Must be 3 or higher                                                                              <br>
                                           const SmTArray<SmPoint3d>  & crPointsToInterpolate,       ///< [in] : List of points to approximate                                                                    <br>
                                           const SmTArray<SmVector3d> & crVectorsToInterpolate,      ///< [in] : optional end derivs                                                                              <br>
                                                                                                     ///<      : sized:[0] - no given end derivs                                                                  <br>
                                                                                                     ///<      : sized:[1] - start deriv only,     crVec[0] = start 1st deriv                                     <br>
                                                                                                     ///<      : sized:[2] - start and end derivs, crVec[0] = start 1st derivative,                               <br>
                                                                                                     ///<      :                                   crVec[1] = end 1st deriv.                                      <br>
                                           const SmTArray<SmVector3d> * cpMoreVectorsToInterpolate,  ///< [in] : sized:[2] - crMoreVec[0] = start 2nd deriv, only used when crVectorsToInterpolate sized:[1 or 2] <br>
                                                                                                     ///<      :             crMoreVec[1] = end   2nd deriv, only used when crVectorsToInterpolate sized:[2]      <br>
                                                                                                     ///<      : sized:[4] - crMoveVec[2] = start 3rd deriv, only used when crVectorsToInterpolate sized:[1 or 2] <br>
                                                                                                     ///<      :             crMoreVec[3] = end   3rd deriv, only used when crVectorsToInterpolate sized:[2]      <br>
                                           SmBSplineCurve            *& rpNewBSplineCurve) ;         ///< [out]: approximating curve                                                                              <br>
  
  static SmStatus CreateArcFromPoints(const SmContext  & crContext,            ///< [in] : new object construction                 <br>
                                      ULONG              lDimensionOfResult,   ///< NotUsed: [in] : 2 or 3                                  <br>
                                      const SmPoint3d  & crCenter,             ///< [in] : Arc Center point                        <br>
                                      const SmPoint3d  & crPoint1,             ///< [in] : Arc Start Point                         <br>
                                      const SmPoint3d  & crPoint2,             ///< [in] : Arc End Point                           <br>
                                      SmNurbCircleParam  eParameterization,    ///< [in] : SM_CO_QUADRATIC = build degree 2 curve  <br>
                                                                               ///<      : SM_CO_QUINTIC   = build degree 5 curve  <br>
                                      SmBSplineCurve  *& rpNewBSplineCurve) ;  ///< [out]: new BSpline Arc                         <br>
  
  static SmStatus CreateByJoining(const SmContext                 & crContext,           ///< [in] : context for new object construction                           <br>
                                  const SmTArray<SmBSplineCurve*> & crSegments,          ///< [in] : Individual B-Spline curves which must be end to end connected.<br>
                                  const SmTArray<SmBoolean>       * cpOptSenses,         ///< [in] : for each segment, TRUE= no reverse,FALSE=do                   <br>
                                                                                         ///<        NULL=no segments are reversed                                 <br>
                                  SmBSplineCurve                 *& rpNewBSplineCurve) ; ///< [out]:                                                               <br>
  
  static SmStatus CreateCanonical(const SmContext           & crContext,            ///< [in] : context for new object construction                            <br>
                                  ULONG                       lDimension,           ///< [in] : valid dimension - 2 or 3                                       <br>
                                  ULONG                       lDegree,              ///< [in] : valid degree - 1 through 32                                    <br>
                                  const SmTArray<SmPoint3d> & crControlPointsList,  ///< [in] : Euclidian form of the control points.                          <br>
                                  SmBSplineCurveForm          eBSplineCurveForm,    ///< [in] : oneof  SM_CF_POLYLINE_FORM,  SM_CF_PARABOLIC_ARC,              <br>
                                                                                    ///<      :    SM_CF_CIRCULAR_ARC,   SM_CF_HYPERBOLIC_ARC,                 <br>
                                                                                    ///<      :    SM_CF_ELLIPTIC_ARC,   SM_CF_UNSPECIFIED,                    <br>
                                                                                    ///<      :    SM_CF_HELICAL_ARC                                           <br>
                                  const SmTArray<ULONG>     & crKnotMultiplicities, ///< [in] : Multiplicities of the end knots must be lDegree+1 and          <br>
                                                                                    ///<      : and internal multiplicities must be lDegree or less.           <br>
                                  const SmTArray<double>    & crKnots,              ///< [in] : unique knots in ascending order                                <br>
                                  SmKnotType                  eKnotType,            ///< [in] : oneof  SM_KT_UNIFORM_KNOTS,  SM_KT_QUASI_UNIFORM_KNOTS,        <br>
                                                                                    ///<      :    SM_KT_UNSPECIFIED,    SM_KT_PIECEWISE_BEZIER_KNOTS          <br>
                                  const SmTArray<double>    * cpOptWeights,         ///< [in] : optional associated weigths for each control point             <br>
                                  const SmExtent1d          * cpOptTrimInterval,    ///< [in] : If specified contains a trimming interval to resize the curve  <br>
                                  SmBSplineCurve           *& rpNewBSplineCurve) ;  ///< [out]: New BSplineCurve                                               <br>
  
  static SmStatus CreateCardinalSpline(const SmContext     & crContext,                   ///< [in] :                                                <br>
                                       SmTArray<SmPoint3d> & rPoints,                     ///< [in] : Points to be interpolated.                     <br>
                                       SmTArray<double>    & rTensionParams,              ///< [in] : rTensionParams[i] corresponds to rPoints[i].   <br>
                                       SmBoolean             bIsPeriodic,                 ///< [in] : see notes above.                               <br>
                                       SmBSplineCurve     *& rpNewBSP,                    ///< [out]: resulting spline                               <br>
                                       SmCurveParameterizationType ePz = SM_CP_UNIFORM) ; ///< [in] : opt: one of SM_CP_UNIFORM, SM_CP_CHORDLENGTH,  <br>
                                                                                          ///<        or SM_CP_CENTRIPETAL.  Default: SM_CP_UNIFORM. <br>
  
  static SmStatus CreateCircleSegment(const SmContext        & crContext,           ///< [in] : new object context                                   <br>
                                      ULONG                    lDimensionOfResult,  ///< [in] : 2 or 3                                               <br>
                                      const SmAxis2Placement & crReferenceFrame,    ///< [in] : Circle lies on the X, Y plane of the reference       <br>
                                                                                    ///<      : frame with zero degrees at the X axis.               <br>
                                      double                   dRadius,             ///< [in] : any number > 0.0                                     <br>
                                      double                   dStartAngDeg,        ///< [in] : angle from XAxis in degrees: [-360,360]              <br>
                                      double                   dEndAngDeg,          ///< [in] : angle from XAxis in degrees: [-360,360]              <br>
                                      SmNurbCircleParam        eParameterization,   ///< [in] : SM_CO_QUADRATIC - produces a circle using a          <br>
                                                                                    ///<      :               quadratic (degree 2) parameterization. <br>
                                                                                    ///<      : SM_CO_QUINTIC   - produces a circle using a quintic  <br>
                                                                                    ///<      :               (degree 5) parametrization.            <br>
                                      SmBSplineCurve        *& rpNewBSplineCurve) ; ///< [out]: the new circle                                       <br>
  
  static SmStatus CreateConicSegment(const SmContext & crContext,           ///< [in] :     <br>
                                     double dA, double dB, double dC,       ///< [in] :     <br>
                                     double dD, double dE, double dF,       ///< [in] :     <br>
                                     SmBSplineCurve *& rpNewBSplineCurve) ; ///< [out]:     <br>
  
  static SmStatus CreateDegenerateCurve(const SmContext  & crContext,           ///< [in] : context for new object construction    <br>
                                       ULONG              lDimensionOfResult,   ///< [in] : desired curve dimension (2 or 3)       <br>
                                       const SmPoint3d  & crPoint,              ///< [in] : Point marking the degenerate curve     <br>
                                       SmBSplineCurve  *& rpNewBSplineCurve) ;  ///< [out]: the new degenerate curve               <br>
  
  static SmStatus CreateEllipseSegment(const SmContext        & crContext,           ///< [in] : context for new object construction                                                   <br>
                                       ULONG                    lDimensionOfResult,  ///< [in] : 2 = Make UVTrimCurve in 2d, 3 = Make 3d XYZ curve                                     <br>
                                       const SmAxis2Placement & crReferenceFrame,    ///< [in] : ellipse center:[RefFrame origin], running CCW:[StartAng, EndAng in RefFrame XY plane] <br>
                                       double                   dRadiusAtXAxis,      ///< [in] : ellipse Radius at RefFrame X, must be > 0,  when dRadiusAtXAxis == dRadiusAtYAxis     <br>
                                       double                   dRadiusAtYAxis,      ///< [in] : ellipse Radius at RefFrame y, must be > 0,  rtns circular arc                         <br>
                                       double                   dStartAngle,         ///< [in] : Start Angle Degrees, where: |dEndAngDeg - dStartAngDeg| <= 360.0                      <br>
                                       double                   dEndAngle,           ///< [in] : End Angle Degrees,   where: |dEndAngDeg - dStartAngDeg| <= 360.0                      <br>
                                       SmNurbCircleParam        eParameterization,   ///< [in] : SM_CO_QUADRATIC: degree 2 ellipse is created with double internal knots               <br>
                                                                                     ///<      : SM_CO_QUARTIC  : degree 4 ellipse is created possibly with quadruple internal knots   <br>
                                                                                     ///<      :                (but better parameterization than SM_CO_QUADRATIC)                     <br>
                                                                                     ///<      : SM_CO_QUINTIC  : degree 5 ellipse is created with no internal knots                   <br>
                                       SmBSplineCurve        *& rpNewBSplineCurve) ; ///< [out]: New BSplineEllipseSegment                                                             <br>
  
  static SmStatus CreateHelixSegment(const SmContext        & crContext,           ///< [in] : context for new object construction                                                         <br>
                                     const SmAxis2Placement & crReferenceFrame,    ///< [in] : Helix centered on Z Axis starting at Z=0 running to Z=Height                                <br>
                                     double                   dHeight,             ///< [in] : Helix Length along RefFrame Z Axis, Height:[0=build spiral,>0=build helix]                  <br>
                                     double                   dRadiuStart,         ///< [in] : Helix Radius at RefFrame Z = 0      (linearly interpolated from 0 to Height)                <br>
                                     double                   dRadiusEnd,          ///< [in] : Helix Radius at RefFrame Z = Height (linearly interpolated from 0 to Height)                <br>
                                     double                   dNumTurns,           ///< [in] : Num of 360 deg Helix Turns in H=[0,Height] (Fractions okay), PeriodLength:[Height/NumTurns] <br>
                                     SmBoolean                bRightHanded,        ///< [in] : TRUE = spiral is right handed, FALSE=left handed                                            <br>
                                     SmApproxTol3d            sApproxTol3d,        ///< [in] : Max allowed deviation between NewBSplineCurve and theoretical Helix                         <br>
                                     SmBSplineCurve        *& rpNewBSplineCurve) ; ///< [out]: The new Helix curve.                                                                        <br>
  
  SmStatus CreateExtendedCurve(const SmContext & crContext,                        ///< [in] : memory pool for output curve                                        <br>
                               double            dParameter,                       ///< [in] : tgt parameter defines curve extension amount                        <br>
                               SmContinuityType  eExtensionContinuity,             ///<      :  SM_CT_G1        - linear extension                                 <br>
                                                                                   ///<      :  SM_CT_G1_G2     - extension with second derivative                 <br>
                                                                                   ///<      :  SM_CT_CINFINITY - infinite continuity                              <br>
                               SmBSplineCurve *& rpExtended,                       ///< [out]: Constructed curve                                                   <br>
                               SmBoolean         bPreciseExtension = FALSE) ;      ///< [in] : 0 = extend curve beyond dParameter by an extra curve-interval amount<br>
                                                                                   ///<      : 1 = extend curve to given dParameter                                <br>
  
  SmStatus CreateExtendedCurve(const SmContext & crContext,                        ///< [in] : memory pool for output curve                       <br>
                               double            dDistance,                        ///< [in] : additional length of extension                     <br>
                               int               StartorEnd,                       ///< [in] : START (1) or END (2)                               <br>
                               SmContinuityType  eExtensionContinuity,             ///<      : SM_CT_G1        - linear extension                 <br>
                                                                                   ///<      : SM_CT_G1_G2     - extension with second derivative <br>
                                                                                   ///<      : SM_CT_CINFINITY - infinite continuity              <br>
                               SmBSplineCurve *& rpNewBSplineCurve) ;              ///< [out]: Constructed curve                                  <br>
  
  static SmStatus CreateFromHermiteCurve(const SmContext       & crContext,          ///< [in] : context for new object construction    <br>
                                         const SmHermiteCurve  & rHermiteCurve,      ///< [in] : Curve to translate into a BSpline      <br>
                                         SmBSplineCurve       *& rpNewBSplineCurve,  ///< [out]: Newly constructed curve                <br>
                                         double                  dParamStart=0.0,    ///< [in] : BSplineCurve param start, default:[0]  <br>
                                         double                  dParamEnd=1.0) ;    ///< [in] : BSplineCurve param end, default:[1]    <br>
  
  // Create SmBSplineCurve by interpolating a set of points and corresponding optional tangent and higher order derivatives.
  static SmStatus CreateInterpolatingCurve(const SmContext            & crContext,                   ///< [in] : context for new object construction                                    <br>
                                           SmCurveParameterizationType  eParameterization,           ///< [in] : How to parameterize between points                                     <br>
                                                                                                     ///<      : 0 - SM_CP_UNIFORM,                                                     <br>
                                                                                                     ///<      : 1 - SM_CP_CHORDLENGTH,                                                 <br>
                                                                                                     ///<      : 2 - SM_CP_CENTRIPETAL                                                  <br>
                                           ULONG                        lDimensionOfResult,          ///< [in] : Must be 2 or 3                                                         <br>
                                           ULONG                        lDegree,                     ///< [in] : Must be 2 or 3 if we are interpolating                                 <br>
                                                                                                     ///<      : with a full set of vectors.  It can be 2 or higher                     <br>
                                                                                                     ///<      : if we are just using end vectors or no vectors.                        <br>
                                           const SmTArray<SmPoint3d>  & crPointsToInterpolate,       ///< [in] : array of target points                                                 <br>
                                           const SmTArray<SmVector3d> & crVectorsToInterpolate,      ///< [in] : May contain No vectors,                                                <br>
                                                                                                     ///<      :         2 vectors - a start and an end vector,                         <br>
                                                                                                     ///<      :      or 1 vector for each point to interpolate.                        <br>
                                           const SmTArray<SmVector3d> * cpMoreVectorsToInterpolate,  ///< [in] : If this exists then it contains higher order derivatives (2nd and 3rd) <br>
                                           SmBoolean                    bIgnoreVectorMagnitude,      ///< [in] : This only applies to case where more than two vectors are used.        <br>
                                                                                                     ///<      : TRUE = use vector directions - not their magnitudes                    <br>
                                           SmBSplineCurve            *& rpNewBSplineCurve) ;         ///< [out]: the newly constructed curve                                            <br>
  
  static SmStatus CreateLineSegment(const SmContext  & crContext,                   ///< [in] : context for new object construction <br>
                                    ULONG              lDimensionOfResult,          ///< [in] : specify desired dimension: 2 or 3   <br>
                                    const SmPoint3d  & crStartPoint,                ///< [in] : Line Start Point                    <br>
                                    const SmPoint3d  & crEndPoint,                  ///< [in] : Line End Point                      <br>
                                    SmBSplineCurve  *& rpNewBSplineCurve) ;         ///< [out]: new BSpline Line [NULL on input]    <br>
                                                                                    
  virtual SmStatus CreateMirrorCurve(const SmContext        & crContext,            ///< [in] :   <br>
                                     const SmAxis2Placement & crMirrorPlane,        ///< [in] :   <br>
                                     SmCurve               *& rpMirrorCurve)        ///< [out]:   <br>
                                    /*const*/ ;
  
  virtual SmStatus CreatePlaneProjection(const SmContext  & crContext,               ///< [in] : context for new object construction                                                <br>
                                         SmProjectionType   eProjectionType,         ///< [in] : oneof SM_PT_PARALLEL, SM_PT_PERSPECTIVE                                            <br>
                                         const SmPoint3d  & rProjectionPlanePoint,   ///< [in] : Point on target plane                                                              <br>
                                         const SmVector3d & rProjectionPlaneNormal,  ///< [in] : Normal of target plane                                                             <br>
                                         const SmVector3d & rProjDirOrCenterOfProj,  ///< [in] : When eProjectionType == SM_PT_PARALLEL, vector is projection direction.            <br>
                                                                                     ///<        When eProjectionType == SM_PT_PERSPECTIVE, vector is 'eye' point of the projection.<br>
                                         SmCurve         *& rpProjectedCurve)        ///< [out]:                                                                                    <br>
                                        const ;
  
  // create a degenerate curve parameterized from 0 to 1 - see EditParameterization() when a specific param range is desired
  static SmStatus CreatePointCurve(const SmContext & crContext,       ///< [in] : context for new object construction  <br>
                                  SmPoint3d       & rPnt,             ///< [in] : target point                         <br>
                                  SmBSplineCurve *& rpCurve) ;        ///< [out]: new Curve                            <br>
  
  virtual SmStatus CreateSimpleOffset(const SmContext  & crContext,                         ///< [in] : context for new object construction                                                      <br>
                                      double             dApproxTol3d,                      ///< [in] : Distance between curve created and theoretical                                           <br>
                                                                                            ///<      : exact offset, (1.0e-2 to 1.0e-6) are typical                                             <br>
                                                                                            ///<      : values used for curves of size 1.0.                                                      <br>
                                      const SmVector3d & crOffsetPlaneNormal,               ///< [in] : Normal of the reference plane used for offseting.                                        <br>
                                      double             dOffsetDistance,                   ///< [in] : BaseCurve/OffsetCurve dist in OffsetPlane.                                               <br>
                                                                                            ///<      : If this is a positive value the offset will lie to                                       <br>
                                                                                            ///<      : the right hand side of the curve as viewed from the                                      <br>
                                                                                            ///<      : positive side of the offset plane.  A negative offset                                    <br>
                                                                                            ///<      : value produces a curve on the left hand side of the                                      <br>
                                                                                            ///<      : base curve.                                                                              <br>
                                      SmBSplineCurve  *& rpNewBSplineCurve,                 ///< [out]: If there is no valid offset, this value will be NULL.                                    <br>
                                                                                            ///<      : Otherwise, it will be offset curve.                                                      <br>
                                      double           & rdMaxGap3d,                        ///< [out]: Max error of offset approximation.                                                       <br>
                                      SmBoolean          bOptMatchParameterization = FALSE) ///< [in] : TRUE = approxCurve(param) within 3DApproxTol of ThisCurve(param) for all param values    <br>
                                     const ;                                                ///<      : FALSE= ApproxCurve(param) within 3d ApproxTol of ThisCurve(AnyParam)                     <br>
                                                                                            ///<      : default:[FALSE], FALSE produces lower control point count curves for slightly more cost. <br>
  SmStatus CreateByScaleTransRot(const SmContext  & crContext,           ///< [in] :   <br>
                                 const SmVector3d & crStartPointVec,     ///< [in] :   <br>
                                 const SmVector3d & crEndPointVec,       ///< [in] :   <br>
                                 double             dRotationDeg,        ///< [in] :   <br>
                                 SmBSplineCurve  *& rpnewBSplineCurve)   ///< [out]:   <br>
                                const ;
  
  SmStatus CreateSectionCurve(const SmContext                 & crContext,             ///< [in] :   Context where new curve construction takes place <br>
                              double                            dNormalizedParameter,  ///< [in] :   Parameter on each input curve through which to build new curve <br>
                              ULONG                             lParameterization,     ///< [in] :   0 - Uniform, 1 - Chord Length, 2 - Centripetal <br>   
                              double                            dTangentStrength,      ///< [in] :   Determine magnitude of tangent (only when cpStart(End)Tangent specified )  <br>
                              ULONG                             lContinuity,           ///< [in] :   0 - C0 continuity,  1 - G1 Continuity, 2 - G2 Continuity <br> 
                              const SmTArray<SmBSplineCurve*> & crSectionCurves,       ///< [in] :   Input curves <br>
                              const SmBSplineSurface          * cpStartTangent,        ///< [in] :   Optional input surface to determine start tangent of result <br>  
                              const SmBSplineSurface          * cpEndTangent,          ///< [in] :   Optional input surface to determine end tangent of result   <br>
                              SmBSplineCurve                 *& rpNewSection)          ///< [out]:   Result curve <br>
                             const;
  
  virtual SmStatus DampenKnotSpacing(double dMaximumDeltaFactor,       ///< [in] :   <br>
                                     ULONG  lMultiplicityOfNewKnots) ; ///< [out]:   <br>
  
  SmStatus DegreeElevate(ULONG lNewDegree) ;
  
  SmStatus DegreeReduction(double    dTolerance,            ///< [in] :                                                      <br>
                           SmBoolean bMaxReduce = FALSE) ;  ///< [in] : If TRUE, will reduce the degree as much as possible  <br>

  // Deform curve smoothly to force its ends to interpolate input EndPt and EndTan tgts. Meant for gap healing and small tol-sized moves
  virtual SmStatus DeformToEndPointTargets(SmPoint3d     * pTgtStartPt,                    // in :  NotNULL = Curve StartPt new Tgt position 
                                                                                           //       NULL    = leave as is
                                           SmPoint3d     * pTgtEndPt,                      // in :  NotNULL = Curve EndPt new Tgt position
                                                                                           //       NULL    = leave as is
                                           SmCurve      ** ppOptNewCurve   = NULL,         // out: Ptr to new curve when editing shape changes this curve's type
                                                                                           //      Not used when this curve can be edited in place without changing its type.
                                                                                           //      default:[NULL]. Returns Err if not given when its needed
                                           SmVector3d    * pOptTgtStartTan    = NULL,      // in : optional Start 1stDir tangent dir (NonUnit but only uses dir, not speed)
                                           SmInValueType   eTgtStartTanType= SM_IV_SAME,   // in : oneof SM_IV_SPECIFIED    : set Start1stDir = pTgtStartTan dir
                                                                                           //            SM_IV_SAME         : set Start1stDir = Init Start1stDir
                                                                                           //            SM_IV_UNCONSTRAINED: set Start1stDir = Unspecified
                                                                                           //      default:[SM_IV_SAME]
                                           SmVector3d    * pOptTgtEndTan      = NULL,      // in : optional End 1stDir tangent dir (NonUnit but only uses dir, not speed)
                                           SmInValueType   eTgtEndTanType  = SM_IV_SAME,   // in : oneof SM_IV_SPECIFIED    : set End1stDir = pTgtEndTan dir
                                                                                           //            SM_IV_SAME         : set End1stDir = Init End1stDir
                                                                                           //            SM_IV_UNCONSTRAINED: set End1stDir = Unspecified
                                                                                           //      default:[SM_IV_SAME]
                                           SmSurface     * pOptSurfCrvOnSurf = NULL,       // in : Optional surface for upon which the target points are define. CrvOnSurf editing. If the CrvOnSurf surf agrees with pOptSurfCrvOnSurf
                                                                                           //      then we edit the underlying UV curve instead of approximating. Default is NULL.
                                           SmOrientType  * pOptCrvOrientation = NULL);     // in : orientation for when we have a crv on surf and will adjust UV points.
                                       
  //virtual SmStatus DropPoint(const SmExtent1d    & crInterval,                       ///< [in] : target curve allowed domain                                                            <br>
  //                       const SmPoint3d     & crPointToDrop,                    ///< [in] : Point to drop to curve                                                                 <br>
  //                       const SmVector3d    * cpVecToCrvInside,                 ///< [in] : For Snapping, NULL=NoSnap, Vec pointing to CrvInside from desired EndPt for snapping.  <br>
  //                                                                               ///<      : when DropPt is within dDistTol in cpVecToCrvInside dir of EndPt, Snap to Ivl EndPt.    <br>
  //                                                                               ///<      : Vec handles ambiguities and makes sure that curves are not just touching at the ends.  <br>                                                                    
  //                       double                dDistTol,                         ///< [in] : Used for SnapDist, and SM_SO_INTERSECT sXSectTol3d distance.                           <br>
  //                                                                               ///<      : sXSectTol3d = dDistTol ;      SM_SO_INTERSECT                                          <br>
  //                                                                               ///<      : sXSectTol3d = SM_BIG_DOUBLE ; SM_SO_MINIMIZE, SM_SO_NORMALIZE                          <br>
  //                                                                               ///<      : sSnapTol3d  = dDistTol ;      SM_SO_MINIMIZE, SM_SO_NORMALIZE, SM_SO_INTERSECT         <br>
  //                                                                               ///<      : DropPTs kept when DropDist < sXSectTol3d                                               <br>
  //                       const double        * pOptGuessParam,                   ///< [in] : If given, call cheap LocalPointSolve() else call expensive GlobalPointSolve()          <br>
  //                       SmBoolean           & rbSuccess,                        ///< [out]: TRUE = found a drop point                                                              <br>
  //                       double              & rdDroppedParameter,               ///< [out]: found drop curve param                                                                 <br>
  //                       double              & rdDistanceToCurve,                ///< [out]: found drop distance                                                                    <br>
  //                       SmSolverOperationType eOperationType = SM_SO_MINIMIZE)  ///< [in] : SM_SO_MINIMIZE = allow nonNormal drops near endPoints                                  <br>
  //                                                                               ///<      : SM_SO_INTERSECT= like Minimize but point must be within dDistTol                       <br>
  //                                                                               ///<      : SM_SO_NORMALIZE= exclude nonNormal drops near endPoints                                <br>
  //                                                                               ///< [out]: Portions of ThisCurve on pos side of ininfite plane                                    <br>                  
  //                      const ;

  SmStatus EditEndPoint(const SmPoint3d  sNewEndPoint,                      ///< [in] : If weight is given: X,Y,Z of a homogeneous point.                                       <br>
                                                                            ///<      : Otherwise           X,Y,Z of a Euclidian point.                                         <br>
                        SmBoolean        bEditStartPoint,                   ///< [in] : TRUE = modify startPoint, else endPoint                                                 <br>
                        const double   * pdOptWeight = NULL,                ///< [in] : Used to set rational CPoints, NULL for NonRational Curves                               <br>
                        SmEditEndType    eNewEndPointType = SM_EE_UNKNOWN,  ///< NotUsed: [in] : Specify how sNewEndPoint was selected (for debug purposes)                              <br>
                                                                            ///<      : default:[SM_EE_UNKNOWN]                                                                 <br>
                        SmBoolean        bHealUseFlag = TRUE) ;             ///< NotUsed: [in] : TRUE = called to heal loop gap problems, when in debug mode, output large move warnings <br>
                                                                            ///<      : FALSE = called to construct geometry when in debug mode, don't output move warnings     <br>

  // Change NURB parameterization of curve to given extent range - no Analytic-STEP range change
  virtual SmStatus EditParameterization(const SmExtent1d & crNewParameterization, ///< [in] : new parameter range for curve                    <br>
                                        SmBoolean          bNotify = TRUE) ;      ///< [in] : TRUE  = make notify calls (previous behavior)    <br>
                                                                                  ///<        FALSE = Skip notify call                         <br>
  
  SmStatus EquallySpacedPoints(const double          t0,           ///< [in] : begin interval parameter                           <br>
                               const double          t1,           ///< [in] : end   interval parameter                           <br>
                               const ULONG           n,            ///< [in] : number of intervals                                <br>
                               const double          dTol,         ///< [in] : 3d tolerance for evenly spaced                     <br>
                               SmTArray<SmPoint3d> * pOptPoints,   ///< [out]: sequence of n+1 approximately evenly spaced points <br>
                               SmTArray<double>    * pOptParams)   ///< [out]: associated parameters                              <br>
                              const ;
  
  virtual SmStatus Evaluate(double     dParameter,                 ///< [in] : tgt param                                                                    <br>
                            ULONG      lNumDerivatives,            ///< [in] : 0=pos, 1=pos+tang, 2=pos+tang+2nd, . . .                                     <br>
                            SmBoolean  bFromLeft,                  ///< [in] : if P is on interval boundary                                                 <br>
                                                                   ///<      : TRUE  = evaluate P in upper interval where P is on the left of the interval  <br>
                                                                   ///<      : FALSE = evaluate P in lower interval where P is on the right of the interval <br>
                            SmVector3d aPointAndDerivatives[],     ///< [out]: (pos, tang, 2nd, ...) sized:[lNumDerivatives+1]                              <br>
                            SmBoolean  bNonZeroTangents=FALSE)     ///< [in] : TRUE = replace zero tangent vectors with properly oriented tol sized vectors <br>
                                                                   ///<      : FALSE= return exact tangent values                                           <br>
                                                                   ///<      : note: Surprisingly TRUE is the common choice because most tangent uses       <br>
                                                                   ///<      :   are for their direction (Binorm, SurfNorm comps), but when the             <br>
                                                                   ///<      :   tangent is being used for its magnitude (like an arc-length comp)          <br>
                                                                   ///<      :   then set this to FALSE.                                                    <br>
                           const;
  
  virtual SmStatus EvaluatePoint    (double dParameter, SmPoint3d & rPoint)     const ;
  virtual SmStatus EvaluateCurvature(double dParameter, double    & rCurvature) const ;
  
  SmStatus FixupKnotVector(double dTolerance=SM_EFF_ZERO) ;
  
  // Separate duplicate end Control points on non-degenerate curves
  virtual SmBoolean FixRepeatedEndControlPoints() ;
  
  static SmStatus FairBlendCurveDerivatives(SmTArray<SmPoint3d>  & rPoints,                       ///< [in] : [1st Curve EndPoint, 2nd Curve EndPoint]                                 <br>
                                            SmTArray<SmVector3d> & rTangents,                     ///< [in,out]: [1st Curve End 1stDeriv, 2nd Curve 1stDeriv]                          <br>
                                            SmTArray<SmVector3d> * pOptHigherOrderDerivs = NULL,  ///< [in,out]: Optional [1st Curve End (i+2)th Deriv, 2nd Curve (i+2)th Deriv] pairs <br>
                                            SmBoolean              bPreventInflection=FALSE) ;    ///< [in] :                                                                          <br>
  
  SmStatus ForceThroughPoint(const SmPoint3d & crPoint,          ///< [in] :                                                 <br>
                             SmBoolean & rbWasModified,          ///< [out]: did we modify the curve at all?                 <br>
                             double    * pdParam = NULL,         ///< [in] : opt: parameter to put the point at              <br>
                             double    * pdGuessParam = NULL,    ///< [in] : opt: if *pdParam is Null, guess param           <br>
                                                                 ///<        for DropPoint.  Ignored if pdParam is not Null. <br>
                             double      dTol = SM_EFF_ZERO) ;   ///< [in] : default SM_EFF_ZERO                             <br>
  
  // simple SmBSplineCurve access
  SmStatus GetCanonical(ULONG               & rlDimension,          ///< [out]: Image Space Dimension Size                        <br>
                        ULONG               & rlDegree,             ///< [out]: Curve polynomial degree                           <br>
                        SmTArray<SmPoint3d> & rControlPointsList,   ///< [out]: Euclidian form of the control points.             <br>
                        SmBSplineCurveForm  & reBSplineCurveForm,   ///<      : SM_CF_POLYLINE_FORM,                              <br>
                                                                    ///<      : SM_CF_CIRCULAR_ARC,                               <br>
                                                                    ///<      : SM_CF_ELLIPTIC_ARC,                               <br>
                                                                    ///<      : SM_CF_PARABOLIC_ARC,                              <br>
                                                                    ///<      : SM_CF_HYPERBOLIC_ARC,                             <br>
                                                                    ///<      : SM_CF_HELICAL_ARC,                                <br>
                                                                    ///<      : SM_CF_UNSPECIFIED                                 <br>
                        SmTArray<ULONG>     & rKnotMultiplicities,  ///< [out]: multiplicity value for each knot                  <br>
                        SmTArray<double>    & rUniqueKnots,         ///< [out]: Unique knot vector                                <br>
                        SmKnotType          & reKnotType,           ///< [out]: oneof:                                            <br>
                                                                    ///<      : SM_KT_UNIFORM_KNOTS,                              <br>
                                                                    ///<      : SM_KT_UNSPECIFIED,                                <br>
                                                                    ///<      : SM_KT_QUASI_UNIFORM_KNOTS,                        <br>
                                                                    ///<      : SM_KT_PIECEWISE_BEZIER_KNOTS                      <br>
                        SmTArray<double>    & rWeights)             ///< [out]: associated weight for every ControlPoint,         <br>
                                                                    ///<      : rWeigths.GetSize() == 0 for non-rational curves   <br>
                       const ;
  
  SmBoolean &              GetOutOfBoundsEnabled   () { return m_bOutOfBoundsEnabled; }
  virtual SmExtent1d       GetNaturalInterval      () const;
  virtual SmExtent1d       GetSTEPInterval         () const  { return GetNaturalInterval() ; }
  virtual SmExtent1d       GetMaxAnalyticDomain    () const; // rtn: max reasonable domain for extensions
  virtual ULONG            GetNumberNaturalKnots   () const;
  ULONG                    GetNumberOfUniqueKnots  () const;
  virtual ULONG            GetNumberControlPoints  () const;
  virtual ULONG            GetDegree               () const;
  virtual void             GetEnds                 (SmPoint3d & rStartPoint,
                                                    SmPoint3d & rEndPoint,
                                                    double    * pdOptStartW = NULL,
                                                    double    * pdOptEndW   = NULL) const ;
  
  virtual SmBoolean        GetInsideOut() const    { return FALSE ; }  // SmLine,SmEllipse,SmCircle have an bInsideOut bit - other curves don't
  virtual SmBSplineCurve * GetRootCurve() const    { return((SmBSplineCurve*)this) ; }
  SmStatus                 GetNullSegments         (SmTArray<double> & rSegments) const; // rtn: param pairs bounding intervals of zero 1st derivative values
  
  gw_CURVE *               GetGwNurbPointer        () const { return m_pNurb ; }  // retrieve the NLIB NURB struct
  gw_CURVE *               GetOrCreateGwNurbPointer()       { if(m_pNurb==NULL) {MakeNurb() ; }
                                                              SM_ASSERT(m_pNurb != NULL) ;
                                                              return m_pNurb ;
                                                            }
  // Knot access
  virtual SmStatus         GetKnotsAll             (SmTArray<double> & rKnots) const;              // get all Knots copy
  SmStatus                 GetKnotsExpert          (ULONG lStartIndex, ULONG lEndIndex, double *adKnots) const;
  virtual SmStatus         GetKnots                (SmTArray<double> & rUniqueKnots,               ///< [out]: Unique knot vector                                          <br>
                                                    SmTArray<ULONG>  * pKnotMultiplicities = NULL, ///< [out]: multiplicity value for each knot                            <br>
                                                    const SmExtent1d * pOptIvl = NULL) const ;     ///< [in] : interval of interest, NULL=Natural Interval, default:[NULL] <br>
  // Greville Abscissa access
  double            GetGrevilleAbscissa    ( ULONG lCtrlPointIndex ) const;
  SmStatus          GetGrevilleAbscissae   ( SmTArray< double > &vGrevilles ) const ;
  
  // Control Point access
  SmStatus GetPolygon(const SmExtent1d    & crInterval,           ///< [in] : interval of interest                 <br>
                      SmTArray<SmPoint3d> & rEuclidianPolygon)    ///< [out]: ControlPoints within given interval  <br>
                     const;
  
  SmStatus GetControlPolygon(SmTArray<SmPoint3d> & rControlPointsList,   ///< [out]: all control points in cartesian coordinates <br>
                             SmTArray<double>    & rWeights)             ///< [out]: associated weights for Rational Curves      <br>
                            const ;         
  
  // get pointer to interal controlPoint array
  SmStatus GetControlPointsPointer(ULONG   &lControlPointCount,               ///< [out]:   <br>
                                   double *&pControlPoints)                   ///< [out]:   <br>
                                  const ;
  
  SmStatus GetControlPoint(SmControlPointFormType eCtrlPointForm,  ///< [in] : SM_CP_NON_RATIONAL - do perspective projection and           <br>
                                                                   ///<        set W=1.0 if it is rational.                                 <br>
                                                                   ///<        SM_CP_HOMOGENEOUS_RATIONAL - don't do division and return W  <br>
                                                                   ///<        SM_CP_EUCLIDIAN_RATIONAL - do division and return W          <br>
                           ULONG lIndex,                           ///< [in] :                                                              <br>
                           SmPoint3d & rControlPoint,              ///< [out]:                                                              <br>
                           double & rdWeight)                      ///< [out]:
                          const;
  
  // copy a subset of the control point array into an ordered 1d array (may be spaced out in the output)
  SmStatus GetControlPointsExpert(SmControlPointFormType eCtrlPointForm,  ///< [in] : SM_CP_NON_RATIONAL           - output euclidean coords only                  <br>
                                                                          ///<      : SM_CP_HOMOGENEOUS_RATIONAL   - output homogeneous coords and weights         <br>
                                                                          ///<      : SM_CP_EUCLIDIAN_RATIONAL     - output euclidean coords and weights           <br>
                                  ULONG lStartIndex,                      ///< [in] : First control point index to be copied                                       <br>
                                  ULONG lEndIndex,                        ///< [in] : Last  control point index to be copied                                       <br>
                                  ULONG lCtrlPointStride,                 ///< [in] : spacing between control point data in output adControlPoints                 <br>
                                                                          ///<      : lCtrlPointStride >= ((SM_CP_NON_RATIONAL) ? this->Dim() : this->Dim() + 1)   <br>
                                  double *adControlPoints)                ///< [out]: sized >= [lCtrlPointStride * (lEndIndex - lStartIndex + 1)]                  <br>
                                 const ;                                  ///<      : SM_CP_NON_RATIONAL         && 2d ? ordered:[X Y          X Y          ...]   <br>
                                                                          ///<      : SM_CP_HOMOGENEOUS_RATIONAL && 2d ? ordered:[X Y W        X Y W        ...]   <br>
                                                                          ///<      : SM_CP_EUCLIDIAN_RATIONAL   && 2d ? ordered:[X/W Y/W W    X/W Y/W W    ...]   <br>
                                                                          ///<      : SM_CP_NON_RATIONAL         && 3d ? ordered:[X Y Z        X Y Z        ...]   <br>
                                                                          ///<      : SM_CP_HOMOGENEOUS_RATIONAL && 3d ? ordered:[X Y Z W      X Y Z W      ...]   <br>
                                                                          ///<      : SM_CP_EUCLIDIAN_RATIONAL   && 3d ? ordered:[X/W Y/W Z/W  X/W Y/W Z/W  ...]   <br>
  // simple queries
  virtual SmBoolean IsRational             () const ;
  SmBoolean         IsPiecewiseC0          () const ;
  virtual SmBoolean IsDegenerate(double             d3DTol = SM_EFF_ZERO,  ///< [in] : min distance between distinct points,
                                 const SmExtent1d * pIvl = NULL)           ///< [in] : target interval to check, NULL = use NaturalInterval
                                const;                                   
  
  // return TRUE when curve is of the form made by the general intersector algorithm
  //   note: it might help if one day such curves identified themselves with an attribute or a bit.
  SmBoolean         IsIntersectorApprox    () const { return( GetDegree() == 3 && IsPiecewiseC0() == TRUE ) ; }
  
  SmBoolean         IsNurbBorrowed         () const { return m_bNurbIsBorrowed ; }
  SmBoolean         IsOutOfBoundsEnabled   () const { return m_bOutOfBoundsEnabled ; }
  SmBoolean         HasNullSegments        () const;
  SmBoolean         HasSameParameterization(const SmBSplineCurve *cpOtherCurve) const ;
  virtual SmBoolean HasRepeatedEndControlPoints(double dTol=SM_ZONE_TOL_3D) const ;
  virtual SmBoolean HasInternalPole(double dTol=SM_ZONE_TOL_3D) const ;
  virtual SmBoolean HasKnotMultiplicityGreaterThanDegree() const ;
  
  virtual SmBoolean IsLine(ULONG        lNumberOfSamplePoints,  ///< [in] : number of sample points to generate and test               <br>
                           double       dTol,                   ///< [in] : max deviation allowed for any samplePoint from ideal line  <br>
                           SmPoint3d  & rLinePoint,             ///< [out]: line's base point                                          <br>
                           SmVector3d & rLineVector)            ///< [out]: line's direction vector
                          const;
  
  virtual SmBoolean IsSegmentLine(const SmExtent1d & crIntervalArg,         ///< [in] : param interval to query                                            <br>
                                  ULONG              lSamplePointCountArg,  ///< [in] : number of points to check                                          <br>
                                  double             dToleranceArg,         ///< [in] : min dist between distinct points                                   <br>
                                  SmPoint3d        & rLinePointArg,         ///< [out]: curve start point                                                  <br>
                                  SmVector3d       & rLineVectorArg)        ///< [out]: curve unit chord when curve length > tol, else could be degenerate <br>
                                 const;
  
  //virtual SmBoolean IsArc(ULONG              lNumberOfSamplePoints, ///< [in] : number of points to sample and test                                  <br>
  //                        double             dTol,                  ///< [in] : max deviation from exact Arc allowed each samplePoint                <br>
  //                        SmAxis2Placement & rReferenceFrame,       ///< [out]: orientation for found arc,                                           <br>
  //                                                                  ///<      : XAxis = centerPoint to StartPoint of curve's interval                <br>
  //                                                                  ///<      : YAxis = perp to XAxis in direction of Pt on Curve at .15 of interval <br>
  //                        double           & rdRadius,              ///< [out]: Found Arc radius                                                     <br>
  //                        double           & rdStartAngDeg,         ///< [out]: Start angle in degrees                                               <br>
  //                                                                  ///<      : relative to a counter clockwise angle about Z from                   <br>
  //                                                                  ///<      : the X axis of reference frame.                                       <br>
  //                        double           & rdEndAngDeg)           ///< [out]: End angle in degrees                                                 <br>
  //                                                                  ///<      : relative to a counter clockwise angle about Z from                   <br>
  //                                                                  ///<      : the X axis of the reference frame.                                   <br>
  //                       const ;
  
  virtual SmStatus  IsOnPlane(const SmPoint3d  & crPlanePointArg,       ///< [in] : point on plane                                                              <br>
                              const SmVector3d & crPlaneNormalArg,      ///< [in] : plane normal direction                                                      <br>
                              double             d3DToleranceArg,       ///< [out]: max allowed deviation between curve and plane                               <br>
                              SmBoolean        & rbIsOnPlaneArg,        ///< [out]: TRUE = all curve points are within tol of plane                             <br>
                              double           & rdMaxDistToPlaneArg,   ///< [out]: max distance between the curve's control polygon and the given plane        <br>
                              const SmExtent1d * pOptInterval = NULL)   ///< [in] : optional curve interval to check, NULL for Natural Interval, default:[NULL] <br>
                             const;
  
  SmStatus InsertOneKnot(double dNewKnot,            ///< [in] : knot value                       <br>
                         ULONG lNumKnotInsertions) ; ///< [in] : Number of times of insertions    <br>
  
  SmStatus RebuildCurve(const SmContext & crContext,     ///< [in] : new object context                         <br>
                        double          * dOptTol,       ///< [in] :                                            <br>
                        double          * dOptMaxDev,    ///< [out]: max deviation of result from input surface <br>
                        SmBSplineCurve *& rpNewCurve) ;  ///< [out]: new approx surface                         <br>
  
  SmStatus RebuildCurveWithArcLengthParam(const SmContext & crContext,    ///< [in] : new object context                                                    <br>
                                          ULONG nIntervals,               ///< [in] : number of intervals (passed to EquallySpacedPoints)                   <br>
                                                                          ///<      : select small number for relatively linear curves                      <br>
                                                                          ///<      : select large number for larger, wavy, curves                          <br>
                                                                          ///<      : if zero, use default calculation                                      <br>
                                          double* dOptTol,                ///< [in] : 3d tolerance for evenly spaced points (passed to EquallySpacedPoints) <br>
                                          double* dOptMaxDev,             ///< [out]: max deviation of result from input curve                              <br>
                                          SmBSplineCurve *& rpNewCurve) ; ///< [out]: new approx curve                                                      <br>
  
  // insert one through point into a piecewise C0 Intersection curve
  SmStatus SetIntersectionCurveThroughPoint(double                     & dNewKnot,                     ///< [in,out]: suggested param value updated to actual param value                                       <br>
                                            const SmPoint3d            & crThrough3dPoint,             ///< [in] : through point position                                                                       <br>
                                            const SmVector3d           * cpOpt3dTangent,               ///< [in] : optional tangent value, NULL to ignore                                                       <br>
                                            SmTArray<SmBSplineCurve*>  & rUVCurve,                     ///< [in] : UVTrimCurves associated with this curve, expected (not required) to equal number of surfaces <br>
                                            SmTArray<const SmSurface*> & rSurface,                     ///< [in] : corresponding surfaces for each pUVCurve                                                     <br>
                                            SmTArray<SmVector2d*>      & rOptUVPoint,                  ///< [in] : UVPoint on pSurface corresponding to Through3dPoint,                                         <br>
                                                                                                       ///<      : A NULL entry=project crThrough3dPoint onto pSurface                                          <br>
                                            SmBoolean                  & bModifiedCurve,               ///< [out]: TRUE=modified curve, FALSE=didn't                                                            <br>
                                            double                       dMinimumSpanPercent = .001) ; ///< [in] : smallest allowed size for new spans.                                                         <br>
                                                                                                       ///<      : A curve span is either split and updated or                                                  <br>
                                                                                                       ///<      : just updated.  Splitting is skipped when the split                                           <br>
                                                                                                       ///<      : point would create a span less than dMinimunSpanPercent                                      <br>
                                                                                                       ///<      : of the current span size.
  
  static SmStatus SmoothJoin(SmBSplineCurve * pCurve1,                     ///< [in] : One of two input curves to join          <br>
                             SmBoolean        bAtStart1,                   ///< [in] : TRUE = join at start FALSE join at end   <br>
                             SmBSplineCurve * pCurve2,                     ///< [in] : Second of two input curves to join       <br>
                             SmBoolean        bAtStart2,                   ///< [in] : TRUE = join at start FALSE join at end   <br>
                             double           dPreserveEndFactor = 0.5) ;  ///< [in] :                                          <br>
  
  SmStatus InsertKnots(SmTArray<double> & rNewKnots) ;
  
  SmStatus InsertKnots( int iNumToAdd, SmExtent1d * pOptInterval = NULL ) ;
  
  SmStatus InsertKnotsByDistance(double dDistance) ;
  
  // use NLIB to interpolate points with optional endTangents with open or closed BSpline
  static SmStatus   InterpolatePoints(const SmContext & crContext,            ///< [in] : context for new object construction      <br>
                                      const SmTArray<SmPoint3d> & crPoints,   ///< [in] : array of points to interpolate (>1)      <br>
                                      const SmTArray<double> * cpOptParams,   ///< [in] : optional corresponding parameter values  <br>
                                      ULONG lDegree,                          ///< [in] : output curve degree                      <br>
                                      SmVector3d * pOptStartTangent,          ///< [in] : Start Tangent, NULL to ignore            <br>
                                      SmVector3d * pOptEndTangent,            ///< [in] : End   Tangent, NULL to ignore            <br>
                                      SmBoolean bIsClosedCurve,               ///< [in] : TRUE =Output curve is closed             <br>
                                                                              ///<      : FALSE=Output curve is open               <br>
                                      SmInterpolationType eParameterization,  ///< [in] : oneof: SM_IT_UNIFORM,                    <br>
                                                                              ///<      :      SM_IT_CHORDLENGTH,                  <br>
                                                                              ///<      :      SM_IT_CENTRIPETAL                   <br>
                                      SmBSplineCurve *& rpNewCurve) ;         ///< [out]: The interpolating curve                  <br>
  
  void SetOutOfBoundsEnabled(SmBoolean bOutOfBoundsEnabled){ m_bOutOfBoundsEnabled = bOutOfBoundsEnabled ; }
  
  SmStatus          MakeNonRational() ;
  
  virtual SmStatus  JoinWith(ULONG lJoinEndThis,             ///< [in] : 0 = Join at thisCurve start                                  <br>
                                                             ///<      : 1 = Join at thisCurve end                                    <br>
                             SmBSplineCurve *pOtherCurve,    ///< [in] :                                                              <br>
                             ULONG lJoinEndOther,            ///< [out]: 0 = Join at OtherCurve start                                 <br>
                                                             ///<      : 1 = Join at OtherCurve end                                   <br>
                             double* pGapTolerance = NULL) ; ///< [in] : Optional tolerance reprsenting max gap between endpoints     <br>
                                                             ///<      : If not specified then use default tolerance based on length  <br>
  
  virtual SmStatus  MakeNurb      () ;
  virtual SmStatus  MatchWeights  ( const SmBSplineCurve *pOtherCurve ) ;
  virtual SmStatus  Multiply      ( double dScale ) ;
  virtual SmStatus  MultiplyLinear( double dScale0, double dScale1 ) ;
  
  static SmStatus   MakeCurvesCompatible(const SmTArray<SmBSplineCurve*> & crCurves, double dTol = 1.0e-8) ;
  
  virtual SmBoolean PassesValidityCheck(SmValidityCheckType   eChecks,           ///< [in] :                             <br>
                                        SmValidityCheckType & reCheckFailed)     ///< [out]: is the check which failed   <br>
                                       const ;
  
  static SmStatus PiecewiseArcInterpolate(const SmContext           & crContext,          ///< [in] :                             <br>
                                          const SmTArray<SmPoint3d> & crPoints,           ///< [in] :                             <br>
                                          SmInterpolationType         eParameterization,  ///< [in] :                             <br>
                                          SmBSplineCurve           *& rpNewCurve) ;       ///< [out]:                             <br>
  
  SmStatus RaiseDegree(ULONG lNewDegree) { return DegreeElevate(lNewDegree) ; }
  SmStatus RefineCurve(const SmTArray<double> & crNewKnots) ;
  SmStatus Refine     (double dTol) ;
  
  // refresh analytic data after editing the Nurb representation directly to keep the two reps in sync
  // only derived classes have something to do here
  virtual SmStatus RefreshAnalytics() { return SM_SUCCESS ; }
  
  SmStatus RemoveKnots(double    dThisApproxTol3d,                ///< [in] : max allowed Current/New Curve deviation                          <br>
                       SmBoolean bConstrainEndDeriv = FALSE,      ///< [in] : TRUE = Constrain end derivatives, FALSE = don't                  <br>
                       ULONG     lHighestDerivConstraint = 0) ;   ///< [in] : Highest derivative constraint i.e. it maintains the 1-st to the  <br>
                                                                  ///<      : lHighest'th derivatives at the end points                        <br>
                                                                  
  SmStatus RemoveOneKnot(double  dKnot,                           ///< [in] : Knot to be removed                                                          <br>
                         ULONG   lNumKnotsRemoval,                ///< [in] : Number of times of removal, set to degree to remove all knot multiplicities <br>
                         double  dThisApproxTol3d,                ///< [in] : max allowed Current/New Curve deviation                                     <br>
                         ULONG & rlNumKnotsRemoved) ;             ///< [out]: Actual number of knots removed                                              <br>
  
  SmStatus RemoveExtraKnots(double d3DRemovabilityTolerance) ;
  
  SmStatus ReparametrizeWithArcLength() ;
  
  // reverse curve parameterization while preserving its NaturalInterval range
  // e.g. NewPoint(sNatIvl.Min) == OldPoint(sNatIvl.Max) with negated tangents.
  virtual SmStatus ReverseParameterization(const SmExtent1d & crOldInterval,  ///< [in] : interval of interest, may be a subset of natural interval <br>
                                           SmExtent1d       & rNewInterval) ; ///< [out]: new domain for crOldInterval on modified curve            <br>
  
  // scale curve's control points
  SmStatus Scale(double dScaleFactor) ;           
  SmStatus Scale(SmVector3d& rScaleVec, SmPoint3d& rScaleCenter) ;
  
  SmStatus ScaleKnotVector(double dIntervalMin,    ///< [out]:                                    <br>
                           double dIntervalMax) ;  ///< [out]: scale knots to fit given interval  <br>
  
  virtual SmStatus SetFromGwNurb(ULONG, const gw_CURVE * cpGwNurbCurve) ;
  
  // typically prefer SetFromGwNurb() over SetGwNurbPointer()
  void SetGwNurbPointer(void *pGwNurbCurve) ;
  
  SmStatus SetCanonical(ULONG                       lDimension,                ///< [in] :    <br>
                        ULONG                       lDegree,                   ///< [in] :    <br>
                        const SmTArray<SmPoint3d> & crControlPointsList,       ///< [in] :    <br>
                        SmBSplineCurveForm          eBSplineCurveForm,         ///< [in] :    <br>
                        const SmTArray<ULONG>     & crKnotMultiplicities,      ///< [in] :    <br>
                        const SmTArray<double>    & crKnots,                   ///< [in] :    <br>
                        SmKnotType                  eKnotType,                 ///< [in] :    <br>
                        const SmTArray<double>    * cpOptWeights,              ///< [in] :    <br>
                        const SmExtent1d          * cpOptTrimInterval=NULL) ;  ///< [in] :    <br>
  
  SmStatus SetControlPoint(SmControlPointFormType  eCtrlPointForm,  ///< [in] : Either SM_CP_EUCLIDIAN_RATIONAL with a weight <br>
                                                                    ///< [in] : or SM_CP_NON_RATIONAL which ignores the weight<br>
                           ULONG                   lIndex,          ///< [in] :                                               <br>
                           const SmPoint3d       & crControlPoint,  ///< [in] :                                               <br>
                           double                  dWeight) ;       ///< [in] :                                               <br>
  
  SmStatus SetExpert(ULONG                   lDimension,         ///< [in] :                                             <br>
                     ULONG                   lDegree,            ///< [in] :                                             <br>
                     SmBSplineCurveForm      eBSplineCurveForm,  ///< [in] :                                             <br>
                     SmEndKnotFormType       eEndKnotForm,       ///< [in] : Defines how many knots on end of curve      <br>
                     ULONG                   lNumKnots,          ///< [in] :                                             <br>
                     const double          * cadKnots,           ///< [in] :                                             <br>
                     SmControlPointFormType  eCtrlPointForm,     ///< [in] :                                             <br>
                     ULONG                   lCtrlPointStride,   ///< [in] : Doubles to skip between points in data array<br>
                     const double          * cadCtrlPoints) ;   ///< [in] : Double array of control points              <br>
  
  SmStatus Simplify(SmBSplineCurve * & rpSimplifiedCurve,  ///< [out]: Approximate curve built with                       <br>
                                                           ///<        old: Tol = 1.0e-4 * Curve's BoundingBoxSize        <br>
                                                           ///<        new: GetApproxTol3d()                              <br>
                    double *pOptMaxGap3d = NULL) ;         ///< [out]: achieved max gap for approximation, NULL to ignore <br>
  
  virtual SmStatus SnapInsideInterval(const SmExtent1d & rInterval,            ///< [in] :      <br>
                                      double             dDistanceTolerance,   ///< [in] :      <br>
                                      double             dCurveParameter,      ///< [in] :      <br>
                                      SmBoolean        & bSuccessfulSnap,      ///< [in] :      <br>
                                      double           & rdSnappedParameter,   ///< [in] :      <br>
                                      SmBoolean        & bOnBoundary)          ///< [in] :      <br>
                                     const;
  
  SmStatus SubdivideAtDiscontinuity(const SmContext           & crContext,            ///< [in] : context for new object construction                                        <br>
                                    SmContinuityType            eContinuityToSplit,   ///< [in] : curve is split at internal continuities <= to this value                   <br>
                                    SmTArray<SmBSplineCurve*> & rResultingCurves,     ///< [out]: newly copied curves - may be 1 curve copy when no internal discontinuities <br>
                                    double dContinuityAngleTol=SM_CONTINUITY_ANGLE) ; ///< [in] : continuity angle tolerance                                                 <br>
  
  SmStatus SubdivideToLinesAndArcs(const SmContext           & crContext,            ///< [in] :    <br>
                                   double                      dTol,                 ///< [in] :    <br>
                                   SmTArray<SmBSplineCurve*> & rResultingCurves) ;   ///< [out]:    <br>
  
  static SmStatus SyncronizeKnotsOfCurves(SmTArray<SmBSplineCurve*> & rCurvesToSyncronize,           ///< [in] :   <br>
                                          double                      dToleranceForIdenticalKnots) ; ///< [in] :   <br>
  
  SmStatus TestForFilletCrossSection(const SmContext & crContext,                ///< [in] : context for new object construction                          <br>
                                     double            d3DTolerance,             ///< [in] : max allowed deviation between curve and classification type  <br>
                                     SmBoolean       & rbIsFilletCrossSection,   ///< [out]: TRUE = might be a fillet cross section                       <br>
                                                                                 ///<      : FALSE= Not a fillet cross section                            <br>
                                     ULONG           & rlFilletCrossSection,     ///< [out]: 0 - circular,   1 - Approx Circular,                         <br>
                                                                                 ///<      : 2 - Linear      3 - G1 Blend,                                <br>
                                                                                 ///<      : 4 - G2 Blend,   5 - G3 Blend                                 <br>
                                                                                 ///<      :10 - Degenerate                                               <br>
                                     double          & rdDistance,               ///< [out]: chord length between beg/end curve points                    <br>
                                     double          & rdRadius,                 ///< [out]: radius of circular and approx circular arcs                  <br>
                                     double          & rdBlendScale)             ///< [out]:                                                              <br>
                                    const ;
  
  virtual SmStatus Tessellate(const SmExtent1d     & crInterval,                  ///< [in] : interval to tessellate                                 <br>
                              double                 dChordHeightTolerance,       ///< [in] : max height/length ratio between tess pts, 0=ignore     <br>
                              double                 dAngleTolDeg,                ///< [in] : max angle between tess tangents, 0=ignore              <br>
                              ULONG                  lMinimumNumberOfSegments,    ///< [in] : max distance between tess pts, 0=ignore                <br>
                              SmTArray<double>     * pParameters = NULL,          ///< [out]: array of split parameter values, NULL=ignore           <br>
                              SmTArray<SmPoint3d>  * pPoints = NULL,              ///< [out]: associated 3D points, NULL=ignore                      <br>
                              SmTArray<SmVector3d> * pOptTangents = NULL)         ///< [out]: Optional array of tangent points for each sample point <br>
                             const ;
  
  SmStatus TestDegenerate(double   Tolerance, ///< [in] : Min 3d distance between distinct ControlPoints    <br>
                          ULONG  & rNu)       ///< [out]: Index of 1st ControlPoint with a duplicate or 0   <br>
                         const;
  
  virtual SmStatus Transform(const SmAxis2Placement & crRotateNMove,     // in : affine rotate and move transformation      
                             const SmVector3d       * cpOptScale=NULL) ; // in : optional scaling about current origin point before RotateNMove
                                                                         //      BSplines, planes, lines, PolyBreps - support nonisotropic scaling
                                                                         //      other geom types only support isoptropic scaling
  
  virtual SmStatus Trim(SmExtent1d & crTrimInterval,          ///< [i/o] : desired new Trim Ivl - can be snapped by tol to existing knots <br>
                        SmBoolean    bNotify=TRUE,            ///< [in] : internal use: use default value   <br>
                        SmBoolean    bSkipDebugCheck=FALSE) ; ///< [in] : internal use: use default value   <br>
  
  int TrimGap(SmBSplineCurve * pCurve1,   ///< [in,out]: curve whose endPoint might be moved                               <br>
              int              CrvToFix,  ///< [in] : 0 = change both curve end points to XSect of extended curves         <br>
                                          ///<        1 = change only pCurve1 endpoint to 'this' curve initial endPoint    <br>
                                          ///<        2 = change only 'this' curve  endpoint to pCurve1 initial endPoint  <br>
              double           dMin) ;    ///< [in] : max 3d gap size to be closed                                         <br>
  
  // Internal - non-public methods
  
  // write array of curves to file
  static SmStatus WriteArrayToFile(const TCHAR                      * cOutputFileName,          ///< [in] : target file name                   <br>
                                   const SmTArray<SmBSplineCurve *> & rCrvArr,                  ///< [in] : array of curves to write           <br>
                                   SmBoolean                          bWriteAttributes = FALSE) ///< [in] : TRUE=Write attributes, FALSE=don't <br>
                                 {
                                   ULONG ii;
                                   SmTArray<SmCurve *> sWriteCurves;
                                   for(ii = 0; ii < rCrvArr.GetSize(); ii++) { sWriteCurves.Add( (SmCurve*)rCrvArr[ii] ); }
                                   return(SmCurve::WriteArrayToFile( cOutputFileName, sWriteCurves, bWriteAttributes ));
                                 }
  
  // read a curve from file
  static SmStatus ReadFromFile(const SmContext  & crContext,
                               const TCHAR      * cInputFileName,
                               SmBSplineCurve  *& rpNewCurve,
                               const ULONG        lFileOffsetInBytes = 0)
                             {
                               SmCurve *pReadCurve = NULL;
                               SmStatus eStat = SmCurve::ReadFromFile( crContext, cInputFileName, pReadCurve, lFileOffsetInBytes );
                               rpNewCurve = SM_CAST_PTR( SmBSplineCurve, pReadCurve );
                               return eStat;
                             }
  
  // read array of curves from file
  static SmStatus ReadArrayFromFile(const SmContext            & crContext,
                                    const TCHAR                * cInputFileName,
                                    SmTArray<SmBSplineCurve *> & rNewCurves,
                                    const ULONG                  lFileOffsetInBytes = 0,
                                    SmTArray<SmCurve *>        * pTestCurves = NULL)
                                 {
                                   SmTArray<SmCurve *> sReadCurves;
                                   SmStatus eStat = SmCurve::ReadArrayFromFile( crContext, cInputFileName, sReadCurves,
                                                                             lFileOffsetInBytes, pTestCurves );
                                   ULONG ii;
                                   for(ii = 0; ii < sReadCurves.GetSize(); ii++)
                                   {
                                     rNewCurves.Add( (SmBSplineCurve*)sReadCurves[ii] );
                                   }
                                   return eStat;
                                 }
  
  // I/O assist
  virtual SmStatus WriteToDB(SmDatabaseIO & rDB,       ///< [in] : target output stream                               <br>
                             ULONG lDBVersionNumber)   ///< [in] : database version to get proper sequence of writes  <br>
                            const ; 
  
  static  SmStatus ReadFromDB(SM_TYPE            lType,               ///< NotUsed: [in] : Object type to be read                                                     <br>
                              SmDatabaseIO     & rDB,                 ///< [in] : target output stream                                                       <br>
                              ULONG              lDim,                ///< [in] : curve image space dim, 2 or 3                                              <br>
                              const SmContext  & crContext,           ///< [in] : context for new object construction                                        <br>
                              SmCurve         *& rpNewCurve,          ///< [out]: NULL on input = new object allocated in this routine built from stream data<br>
                                                                      ///<      : NotNULL on input = pointer to an empty object to be filled by this routine <br>
                              ULONG              lDBVersionNumber) ; ///< [in] : database version to get proper sequence of writes                          <br>
  
  // Define  GetType(), IsKindOf(), and virtual Dump() (Dump needs local implementation)
  SM_COMMON(SmBSplineCurve,SmCurve,SmBSplineCurve_TYPE) ;
  
  // get Curve Memory size - plus its attributes
  virtual ULONG GetMemoryUsed(ULONG    & rlMemoryAllocated,        ///< [out]: bigger size of all allocated memory in bytes  <br>
                              SmMarkType eMarkType=SM_MT_NOMARK)   ///< [in] : uses without increment eMarkType value        <br>
                             const ;
  
  virtual SmBoolean AssertValid(SmAssertArray    * pAList=NULL,           ///< [in,out]: Accumulating list of failed Asserts, NULL to ignore                           <br>
                                SmAssertTestLevel  eTestLevel=SM_LEVEL_0, ///< [in] : SM_LEVEL_0=fewest tests, SM_LEVEL_1, SM_LEVEL_2=all tests,                       <br>
                                                                          ///<      : SM_LEVEL_GIVEN = run tests in order requested in pTestRequests                   <br>
                               SmAssertWalking    eWalkTree=SM_WALK,      ///< NotUsed: [in] : SM_WALK = run AssertValid on any topology graph descendants, SM_NO_WALK=don't    <br>
                               SmTArray<ULONG>  * pTestRequests=NULL)     ///< [in] : when eTestLevel == SM_LEVEL_GIVEN, run these tests in this order, default:[NULL] <br>
                              const ;
  
  // obsolete
  // virtual SmBoolean AssertHeal (SmAssertReport & rAReport, SmAssertArray * pAList) ;
  
  SmBoolean AssertNoPeriodicJump(SmBoolean    bSurfaceClosedU,      ///< [in] : TRUE = Surface closed in U (U isoLines are closed) <br>
                                 SmBoolean    bSurfaceClosedV,      ///< [in] : TRUE = Surface closed in V (V isoLines are closed) <br>
                                 SmExtent2d & rSurfaceUVDomain) ;   ///< [in] : Surface Domain                                     <br>
  
  virtual void Dump(const TCHAR * message) const ;
  virtual void Dump(ULONG) const ;
  virtual void Dump(SmBoolean bAbbrev)const ;
  
} ; // end class SmBSplineCurve

#ifndef SM_PREDEFINE_BSC
// GWC:BIND_TEMPLATES_MOVE     SM_TARRAY_TEMPLATE_PREDECLARATION(SmBSplineCurve*) ;
#define SM_PREDEFINE_BSC 1
#endif

#endif // !__SMBSPLINECURVE_H__

