// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/******************************************************************//**
* FILE NAME --- SmCone.h
* PURPOSE: Header file for Cone Surface class.
**********************************************************************/

#ifndef __SMCONE_H__
#define __SMCONE_H__

#ifndef __SMSURFOFREVOLUTION_H__
#include <SmSurfOfRevolution.h>
#endif

#ifndef __SMAXIS2PLACEMENT_H__
#include <SmAxis2Placement.h>
#endif

/*******************************************************************//**
PURPOSE: Represent a Cone surface made from a SmSurfOfRotation surface.

    The STEP equation of the cone is defined by the quantities:

    U parameter = CCW rotation around the Cone's Z axis from the XAxis in degrees,
                  U rangeDeg:[-360 to 360] maxLength = 360
    V parameter = height (linear 3d distance) from Cone's BasePoint along cone Z axis.
                  The BasePoint is m_vPosition.m_vOrigin.
                  So V == 0 at the BasePoint 
                    (but 0.0 does not have to start nor even be contained in m_vAnalUVDomain V interval)
    minV             = m_vAnalUVDomain.GetMin().y
    maxV             = m_vAnalUVDomain.GetMax().y
    double height    = maxV - minV
    m_dBaseRadius    = radius at the BasePoint.
    double BotRadius = m_dBaseRadius + smos_Tangent(m_dSemiAngleDeg) * minV
    double TopRadius = m_dBaseRadius + smos_Tangent(m_dSemiAngleDeg) * maxV

    as Rad(V)    = m_dBaseRadius + smos_Tangent(m_dSemiAngleDeg) * V
       Cone(U,V) =   origin                             
                   + cos(U) * Rad(V) * XAxis
                   + sin(U) * Rad(V) * YAxis
                   + V               * ZAxis ;
       with U rangeDeg:[-360 to 360] maxLength = 360
            V range   : bounded  [minV to maxV], maxV - minV == height
                        infinite [-SM_INFINITE_PARAMETER, SM_INFINITE_PARAMETER]

    Note that the V parameter for SmCone is different from that of the
    parent class SmSurfOfRevolution.  There it is the parameterization
    of the Generator curve; here it is the linear distance from our origin
    along the z axis.  Although here we adjust the parameterization of the
    Generator curve so that it corresponds to the linear distance along the
    axis, so it stays the same.

NOTES:
 The STEP parameterization used for the cone, in the angular direction,
 is different from the Nurb parameterization used for the underlying NURB
 surface.  The map from one to the other is not linear. 

  Translating between STEP and NURB parameters:
    m_bSwapUV    is set when the underlying NURB V param
                 maps to the rotation direction.
    m_bInsideout is set when the underlying NURB linear direction
                 runs from maxV to minV.

    1. Rotation direction - SmSurfOfRevolution::m_vPolarConverter
        The ZAxis is always set so that the cone sweeps are made
        CCW about Z

    2. When Cone is bounded 
       When m_bInsideout == FALSE:

           (Nurb_V - Nurb_VMin)      (Step_V - Step_VMin)
         ----------------------- = -----------------------
         (Nurb_VMax - Nurb_VMin)   (Step_VMax - Step_VMin)

       When m_bInsideout == TRUE:

           (Nurb_V - Nurb_VMax)      (Step_V - Step_VMin)
         ----------------------- = -----------------------
         (Nurb_VMin - Nurb_VMax)   (Step_VMax - Step_VMin)

    3. When Cone is infinite
       Nurb and Step Linear param = 'zero' points are set equal
       When m_bInsideout == FALSE:  Nurb_V =  Step_V
       When m_bInsideout == TRUE :  Nurb_V = -Step_V

    4. m_bSwapUV = TRUE 
       NurbU = STEPV and
       NurbV = STEPU

  NOTES: m_bSwapUV and m_bInsideOut are not used to make STEP evaluations

         STEP/Nurb Evaluators when NurbUV = Convert(StepUV)
           EvaluatePoint(NurbUV) == EvaluateSTEPPoint(StepUV) 
           However Tangents might have different magnitudes.
                   Tangents are swapped when m_bSwapUV is true,
                            and GenCurve tangent is negated when m_bInsideOut is true.
                   Normals will be negated when
                            (m_bSwapUV XOR m_bInsideOut).


  SmCone::CreateCanonical builds an unbounded cone when the input dHeight parameter
  is not specified ( <= 0 ).
     infinite cylinders are parameterized between +/- SM_INFINITE_PARAMETER       
     infinite cones     are parameterized between apex and +SM_INFINITE_PARAMETER 

   +----------------------------+-------------------+----------------------------+
   | Evaluator                  |       Input       |  OUTPUT                    |
   +----------------------------+-------------------+----------------------------+
   | EvaluatePoint()            | NURB parameter    | Euclidean Point            |
   | Evaluate()                 | NURB parameter    | Euclidean Point and derivs |
   | EvaluateSTEPPoint()        | STEP parameter    | Euclidean Point            |
   | EvaluateSTEP()             | STEP parameter    | Euclidean Point and derivs |
   | ConvertUVFromSTEPToNURBS() | STEP parameter    | NURB parameter             |
   | ConvertUVFromNURBSToSTEP() | NURB parameter    | STEP parameter             |
   | STEPInversion()            | Euclidean Point   | STEP parameter             |
   | DropPointFast()            | Euclidean Point   | NURB parameter             |
   | GlobalPointSolve()         | Euclidean Point   | NURB parameter             |
   | LocalPointSolve()          | Euclidean Point   | NURB parameter             |
   +----------------------------+-------------------+----------------------------+

***********************************************************************/
class SM_EXPORT SmCone : public SmSurfOfRevolution
{
protected:
  // inherited
// Remove Composites
// // SmSurface::m_pOwner                 - NULL or ptr to SmFace or SmCFace
  // SmSurface::m_pOwner                 - NULL or ptr to SmFace
  // SmBSplineSurface::m_pNurb           - ptr to BSplineSurface controlPoints and knots
  // SmSurfOfRevolution::m_vPosition     - m_vOrigin = Base point, on axis (with radius = m_dBaseRadius,
  //                                                                        becomes the apex when m_dBaseRadius = 0.0)
  //                                       m_vZAxis  = axis of revolution
  //                                       m_vXAxis  = 0/360 vector of angular domain
  //
  // SmSurfOfRevolution::m_pGenCurve;    - 3D generator curve
  //            Always an SmLine for SmCone.
  //            Rule 1: ensure analytic domain of the SmLine 
  //                   is the same as the v interval of m_vAnalUVDomain,
  //                   although it is not actually used at the SmCone level.
  //            Rule 2: ensure line analytic m_dScale value is proportional to the SemiAngle as
  //                    GenCurve->m_dLineScale = 1.0/smos_Cosine(SM_DEG2RAD(m_dSemiAngleDeg)).
  //                    A unit change in the SmLine parameter should correspond to
  //                    a unit change in the 3d distance along the rotation Z axis.
  //           constraints: 1. m_pGenCurve->AnalyticDomain == m_vAnalUVDomain->GetVInterval()
  //                        2. m_pGenCurve->LineScale == 1.0/SM_DEG2RAD(m_dSemiAngleDeg)
  //
  // SmSurfOfRevolution::m_vAnalUVDomain - x = angular domain in degs measured CCW from m_vXAxis about m_vZAxis 
  //                                       y = linear domain from origin
  // SmSurfOfRevolution::m_bSwapUV       - TRUE: NurbU = STEPV,  FALSE: NurbU = STEPU
  //                                             NurbV = STEPU          NurbV = STEPV


  double    m_dBaseRadius   = SM_UNDEF_DOUBLE ;  // Radius of the cone in the plane passing through origin
  double    m_dSemiAngleDeg = SM_UNDEF_DOUBLE ;  // (between -90 to 90 degrees)
                                                 // when SemiAngle  > 0 then TopRadius >= BotRadius
                                                 //      SemiAngle  < 0 then TopRadius <= BotRadius
                                                 //      SemiAngle == 0 then it's a cylinder
  SmBoolean m_bInsideOut = FALSE ;               // FALSE= m_vPosition.ZAxis points from bot to top of Nurb
                                                 // TRUE = m_vPosition.ZAxis points from top to bot of Nurb
                                                 //        (not necessary to the apex)

  // Construction - private methods, don't use outside of SmCone methods

  // build an infinitely long cone, 
  //   cone     domain: from apex to SM_INFINITE_PARAMETER
  //   cylinder domain: from -SM_INFINITE_PARAMETER to SM_INFINITE_PARAMETER
  // when bSwapUV==FALSE and bInsideOUt==FALSE
  //   u-parameterization (rotational): [ dStartRotAngle, dEndRogAngle ]
  //   v-parameterization (along axis): [ 0, dHeight ]
  //   Cone SrfNormal points from SrfPt away from cone axis
  SmCone
  (
    const SmPoint3d  & crOrigin,       ///< [in] : STEP BotCircle Center Point                                            <br>
    const SmVector3d & crXAxis,        ///< [in] : STEP Surface  0 degree rotation                                        <br>
    const SmVector3d & crYAxis,        ///< [in] : STEP Surface 90 degree rotation                                        <br>
    double             dBaseRadius,    ///< [in] : radius of cone at origin                                               <br>
    double             dSemiAngleDeg,  ///< [in] : Angle from cone ZAxis to Cone Surface (degrees [0,90])                 <br>
    SmBoolean          bSwapUV,        ///< [in] : TRUE = underlying NURB surface v dir maps to rotation direction        <br>
                                       ///<      : FALSE= underlying NURB surface u dir maps to rotation direction        <br>
    SmBoolean          bInsideOut      ///< [in] : TRUE = underlying NURB linear dir runs from TopCircle to BotCircle     <br>
                                       ///<      : FALSE= underlying NURB linear dir runs from BotCircle to TopCircle     <br>
  ) ;   
                                            
  // copy constructor
  SmCone(const SmCone & crCone) ;

  // empty constructor for I/O
  SmCone() { } 

  // build a finite length cone. 
  // Cone bottom is at crOrigin.
  // when bSwapUV==FALSE and bInsideOUt==FALSE
  // u-parameterization (rotational): [ dStartRotAngle, dEndRogAngle ]
  // v-parameterization (along axis): [ 0, dHeight ]
  //   Cone SrfNormal points from SrfPt away from cone axis
  SmCone
  ( 
    const SmPoint3d  & crOrigin,            ///< [in] : BotCircle Center Point (v=0.0) upon creation.                            <br>
                                            ///<      : although that may be subsequently modified.                              <br>
    const SmVector3d & crXAxis,             ///< [in] : cone Surface  0 degree rotation                                          <br>
    const SmVector3d & crYAxis,             ///< [in] : cone Surface 90 degree rotation                                          <br>
    double             dBotRadius,          ///< [in] : radius of bottom circular boundary                                       <br>
    double             dTopRadius,          ///< [in] : radius of top    circular boundary                                       <br>
    double             dStartRotAngle,      ///< [in] : start angle boundary from XAxis (degrees [-360, 360])                    <br>
    double             dEndRotAngle,        ///< [in] : end   angle boundary from xAxis (degress [-360, 360]) maxRange=360       <br>
    double             dHeight,             ///< [in] : distance from bottom to top circular boundaries                          <br>
    SmBoolean          bSwapUV,             ///< [in] : TRUE = underlying NURB surface v dir maps to rotation direction          <br>
    SmBoolean          bInsideOut,          ///< [in] : TRUE = underlying NURB linear dir runs from TopCircle to BotCircle       <br>
                                            ///<      : FALSE= underlying NURB linear dir runs from BotCircle to TopCircle       <br>
    SmBoolean          bMakeNurbGenCurve    ///< [in] : TRUE = GenCurve is type SmBSplineCurve                                   <br>
                                            ///<      : FALSE= GenCurve is type SmLine                                           <br>
  );  
                                                 

public:
  // destructor
  virtual ~SmCone() { }

  // equality operator
  virtual SmBoolean operator==(const SmSurface &crOther) const;

  // set surf analytic domain and genCurve analytic interval, range:
  //  u: [-360 to 360] Max diff=360,
  //  v: min dist to max dist from base point along Z axis
  // This modifies the 3d extents of the surface.
  virtual SmStatus AdjustSTEPUVDomain(const SmExtent2d & crNewSTEPUVDomain);    

  virtual SmStatus Copy
  (
    const SmContext & crContext,    ///< [in] : new object context
    SmSurface      *& rpNewSurface  ///< [out]: new SmCone copy
  ) const;

  virtual SmStatus RebuildSTEPFromNURBParameters() ;
  // also SmBSplineSurface::AreSTEPAndNURBCurrent() const ;
  //      SmBSplineSurface::RebuildNURBFromSTEPParameters() ;

  // U param = CCW from XAxis about ZAxis in degrees: range:[0 to 360]
  // V param = height along ZAxis from origin:
  //            cylinder range:[-SM_INFINITE_PARAMETER,SM_INFINITE_PARAMETER]
  //            cone     range:[ ApexParam,            SM_INFINITE_PARAMETER]
  //   Cone SrfNormal points from SrfPt away from cone axis

  // create a step-parameterized infinite (optionally finite) height cone surface                          
  static SmStatus CreateCanonical
  (
    const SmContext        & crContext,           ///< [in] : new object context                                                       <br>
    const SmAxis2Placement & crOrigin,            ///< [in] : m_vOrigin = STEP BotCircle Center Point                                  <br>
                                                  ///<      : m_vXAxis  = STEP Surface  0 degree rotation                              <br>
                                                  ///<      : m_vYAxis  = STEP Surface 90 degree rotation                              <br>
    double                   dBotRadius,          ///< [in] : Cone BottomCircle radius at crOrigin.m_vOrigin                           <br>
    double                   dSemiAngleDeg,       ///< [in] : ang from Z axis to Cone Surface, range:[0 - 90]                          <br>
    SmCone                *& rpNewCone,           ///< [out]: new object                                                               <br>
    double                 * pOptHeight = NULL,   ///< [in] : Distance from Bottom to TopCircle along Z axis of bounded cone           <br>
                                                  ///<      : NULL = make infinite cylinder, default:[NULL]                            <br>
    SmBoolean                bSwapUV = FALSE,     ///< [in] : optional: FALSE= map:[NurbU<->AnalU], TRUE=map:[NurbU<->AnalV],          <br>
                                                  ///<      :                      [NurbV<->AnalV],          [NurbV<->AnalU],          <br>
                                                  ///<      : default:[FALSE], for experts - always use default value                  <br>
    SmBoolean                bInsideOut = FALSE   ///< [in] : optional: FALSE= ZAxis from Bot to Top, TRUE=negate,                     <br>
  ) ;

  // create a step-parameterized finite length cone surface
  static SmStatus CreateCanonical
  (
    const SmContext        & crContext,           ///< [in] : new object context                                                           <br>
    const SmAxis2Placement & crOrigin,            ///< [in] : m_vOrigin = STEP BotCircle Center Point                                      <br>
                                                  ///<      : m_vXAxis  = STEP Surface  0 degree rotation                                  <br>
                                                  ///<      : m_vYAxis  = STEP Surface 90 degree rotation                                  <br>
    double                   dBotRadius,          ///< [in] : Cone BottomCircle radius at crOrigin.m_vOrigin                               <br>
    double                   dTopRadius,          ///< [in] : Cone TopCircle radius                                                        <br>
    double                   dHeight,             ///< [in] : Distance from Center to TopCircle along Z axis of bounded cone               <br>
                                                  ///<      : 0 = make infinite cone                                                       <br>
    SmCone                *& rpNewCone,           ///< [out]: new object                                                                   <br>
    SmBoolean                bSwapUV = FALSE,     ///< [in] : optional: FALSE= map:[NurbU<->AnalU], TRUE=map:[NurbU<->AnalV],              <br>
                                                  ///<      :                      [NurbV<->AnalV],          [NurbV<->AnalU],              <br>
    SmBoolean                bInsideOut = FALSE   ///< [in] : optional: FALSE= ZAxis from Bot to Top, TRUE=negate,                         <br>
  );

  // create offset surface - Out Surf->Domain(s) may be trimmed but not scaled
  virtual SmStatus CreateOffsetSurface
  (
    const SmContext      & crContext,               ///< [in] : context for new obj construction                                           <br>
    double                 dSignedOffsetDistance,   ///< [in] : offset dist, (neg val = Offset dir opposite surface normal)                <br>
    SmApproxTol3d          sApproxTol3d,            ///< [in] : Max Dist between ApproxOffsetSurface and ideal offset shape                <br>
    SmSurface* &           rOffsetSurface           ///< [out]: Offset Surf Approx, may have self-intersections                            <br>
  ) const ; 

  // create IsoParamCurve from underlying NurbSurfaceConvertUVFromSTEPToNURBS
  // uses analytic params to return SmLine or SmCircle when possible
  virtual SmStatus CreateIsoParametricCurve
  (
    const SmContext  & crContext,                   ///< [in] : context for created objects                                                                    <br>
    SmSurfParamType    eSurfParam,                  ///< [in] : Defines which Nurb parameter direction on surface to extract curve from                        <br>
                                                    ///<      : SM_SP_U = create constant u isoParameter curve                                                 <br>
                                                    ///<      : SM_SP_V = create constant v isoParameter curve                                                 <br>
    double             dIsoParameter,               ///< [in] : Defines Nurb parametric value at which to extract the curve.                                   <br>
                                                    ///<      : If eSurfParam==SM_SP_U this is a U parameter, if eSurfParam==SM_SP_V                           <br>
                                                    ///<      : then this is the V parameter                                                                   <br>
    SmApproxTol3d      sApproxTol3d,                ///< NotUsed: [in] : passed to ApproximateCurve() when approximation is required.                                   <br>
                                                    ///<      : If set to 0.0, tolerance is set by system: old[curve length * 1.0e-4] new[GetApproxTol3d()]    <br>
    SmBSplineCurve  *& rpNewIsoCurve,               ///< [out]: 3d IsoParameterCurve                                                                           <br>
    const SmExtent2d * pOptDomain = NULL,           ///< [in] : optional trim bound for IsoParameterCurve, NULL to ignore, default:[NULL]                      <br>
    double           * pOptMaxGap3d = NULL,         ///< [out]: opt achieved max gap, NULL to ignore, default:[NULL]                                           <br>
    SmCurve         ** pOptUVIsoCurve = NULL        ///< [out]: opt 2d UVTrimCurve Line (diff parameterization), NULL to ignore, default:[NULL]                <br>
  ) const;

  // create SmCircle from analytic parameters and given point - can return a degenerate curve
  SmStatus CreateIsoCircleFromPoint
  (
    const SmContext & crContext,                     ///< [in] : context for new object construction                           <br>
    const SmPoint3d & crReferencePoint,              ///< [in] : Point on plane defining cone/plane circle                     <br>
    double            d3DTolerance,                  ///< [in] : min distance between unique 3d Points                         <br>
    SmBSplineCurve *& rpCircle                       ///< [out]: Return Curve either NULL, degenerate Curve,  or SmCircle      <br>
  ) const;

  virtual SmStatus ConvertUVFromSTEPToNURBS
  (
    const SmPoint2d & crSTEPUV,                     ///< [in] : Target 2d Point in [degrees, height]           <br>
    SmPoint2d       & rNURBSUV                      ///< [out]: 2d Point in SMLib parameterization             <br>
  ) const;

  virtual SmStatus ConvertUVFromNURBSToSTEP
  (
    const SmPoint2d & crNURBSUV,                    ///< [in] : Target 2d Point in SMLib parameterization       <br>
    SmPoint2d       & rSTEPUV                       ///< [out]: 2d Point in [degrees, height]                   <br>
  ) const;

  virtual SmStatus Evaluate
  (
    const SmPoint2d & crUV,                         ///< [in] : param value to evaluate                                                                    <br>
    ULONG             lHighestUDeriv,               ///< [in] : number of U derivatives                                                                    <br>
    ULONG             lHighestVDeriv,               ///< [in] : number of V derivatives to compute                                                         <br>
    SmBoolean         bUFromLeft,                   ///< [in] : if P is on U interval boundary                                                             <br>
                                                    ///<      : TRUE  = evaluate P in upper interval where P is on the left of the interval                <br>
                                                    ///<      : FALSE = evaluate P in lower interval where P is on the right of the interval               <br>
    SmBoolean         bVFromLeft,                   ///< [in] : if P is on V interval boundary                                                             <br>
                                                    ///<      : TRUE  = evaluate P in upper interval where P is on the left of the interval                <br>
                                                    ///<      : FALSE = evaluate P in lower interval where P is on the right of the interval               <br>
    SmBoolean         bOnlyUpperHalf,               ///< [in] : TRUE=compute upper half of matrix only                                                     <br>
                                                    ///<      : ex. 1,1 = [D  Du] 2,2 = [D    Du    Duu] where -- = an untouched memory value              <br>
                                                    ///<      :           [Dv --]       [Dv   Duv   ---]          (the memory has to be allocated)         <br>
                                                    ///<      :                         [Dvv  ---   ---]                                                   <br>
    SmVector3d      * aDerivatives,                 ///< [out]: matrix of evaluations values                                                               <br>
                                                    ///<      : sized:[lHighestUDeriv+1][lHighestVDeriv+1]                                                 <br>
                                                    ///<      : 2d organized: [D    Du    Duu    Duuu    Duuuu   ]  (the same no matter the value of)      <br>
                                                    ///<      :               [Dv   Duv   Duuv   Duuuv   Duuuuv  ]  (  bOnlyUpperHalf               )      <br>
                                                    ///<      :               [Dvv  Duvv  Duuvv  Duuuvv  Duuuuvv ]                                         <br>
                                                    ///<      :               [Dvvv Duvvv Duuvvv Duuuvvv Duuuuvvv]                                         <br>
                                                    ///<      : 1d organized: [D, Dv, Dvv, Dvvv,.. Du, Duv, Duvv, Duvvv,.. Duu, Duuv, Duuvv, Duuvvv,...]   <br>
    SmBoolean         bNonZeroTangents=TRUE,        ///< [in] : TRUE = replace zero tangent vectors with properly oriented tol sized vectors               <br>
                                                    ///<      : FALSE= return exact tangent values                                                         <br>
                                                    ///<      : note: Surprisingly TRUE is the common choice because most tangent uses                     <br>
                                                    ///<      :       are for their direction (Binorm, SurfNorm comps), but when the                       <br>
                                                    ///<      :       tangent is being used for its magnitude (like an arc-length comp)                    <br>
                                                    ///<      :       then set this to FALSE.                                                              <br>
                                                    ///<      : default:[TRUE]                                                                             <br>
    SmBoolean         bDoZeroSampling=TRUE          ///< [in] : for internal use only, always set to TRUE, default:[TRUE]                                  <br>
  ) const ;

  virtual SmStatus EvaluatePoint
  (
    const SmPoint2d & rNurbUV,                      ///< [in] : Target Nurb Parameter        <br>
    SmPoint3d & rPoint                              ///< [out]: euclidean point              <br>
  ) const;

  // Evaluate the point and triangular derivatives through order 3 at the given STEP UV point.
  // Angular parameters are in degrees; derivatives are per degree. See SmSTEPSurface.h.
  virtual SmStatus EvaluateSTEP
  (
    const SmPoint2d & crUV,               ///< [in] : U=CCW Rot about Z from X in degrees, [0 to 360]                                           <br>
                                          ///<      : V=param from bottom to top: linear with distance                                          <br>
                                          ///<      : when m_bMakeNurbGenCurve = FALSE, V is height along axis                                  <br>
                                          ///<      :      m_bMakeNurbGenCurve = TRUE , V is parameterization of gen curve                      <br>
    ULONG             lHighestUDeriv,     ///< [in] : Requested triangular derivative order in U; must equal lHighestVDeriv; max 3               <br>
    ULONG             lHighestVDeriv,     ///< [in] : Requested triangular derivative order in V; must equal lHighestUDeriv; max 3               <br>
    SmBoolean         bUFromLeft,         ///< [in] : not used - if P is on U interval boundary                                                 <br>
                                          ///<      : TRUE  = evaluate P in upper interval where P is on the left of the interval               <br>  
                                          ///<      : FALSE = evaluate P in lower interval where P is on the right of the interval              <br>
    SmBoolean         bVFromLeft,         ///< [in] : not used - if P is on V interval boundary                                                 <br>
                                          ///<      : TRUE  = evaluate P in upper interval where P is on the left of the interval               <br>
                                          ///<      : FALSE = evaluate P in lower interval where P is on the right of the interval              <br>
    SmBoolean         bOnlyUpperHalf,     ///< [in] : not used output always set for bOnlyUpperHalf=TRUE                                        <br>
    SmVector3d      * aDerivatives        ///< [out]: matrix of evaluations values                                                              <br>
                                          ///<      : sized:[lHighestUDeriv+1][lHighestVDeriv+1]                                                <br>
                                          ///<      : 2d organized: [D    Du   ]                                                                <br>
                                          ///<      :               [Dv   -    ]                                                                <br>
                                          ///<      : 1d organized: [D, Dv, Dvv, Dvvv,.. Du, Duv, Duvv, Duvvv,.. Duu, Duuv, Duuvv, Duuvvv,...]  <br>
  ) const ;

  // Get 3dPoint for given UVPoint [0_to_360, -90_to_90]
  virtual SmStatus EvaluateSTEPPoint(const SmPoint2d & crUV, SmPoint3d & rPoint) const;

  virtual SmBoolean GetInsideOut()            const  { return m_bInsideOut; }
  double            GetRadius(double dHeight) const  { return m_dBaseRadius + smos_Tangent(SM_DEG2RAD(m_dSemiAngleDeg)) * dHeight ; }  // dHeight == v parameter
  double            GetBotRadius()            const  { return GetRadius( m_vAnalUVDomain.GetVMin() ); }
  double            GetTopRadius()            const  { return GetRadius( m_vAnalUVDomain.GetVMax() ); }
  double            GetHeight   ()            const ;
  double            GetBotHeight()            const  { return m_vAnalUVDomain.GetVMin(); } // == Anal v parameter at bottom
  double            GetTopHeight()            const  { return m_vAnalUVDomain.GetVMax(); } // == v parameter at top
  SmPoint3d         GetBotPoint ()            const  { SmPoint3d sBotXYZ, sUVW(0,0,m_vAnalUVDomain.GetVMin()) ;
                                                       m_vPosition.TransformPoint(sUVW, sBotXYZ) ;
                                                       return sBotXYZ ;
                                                     }
  SmPoint3d         GetTopPoint ()            const  { SmPoint3d sTopXYZ, sUVW(0,0,m_vAnalUVDomain.GetVMax()) ;
                                                       m_vPosition.TransformPoint(sUVW, sTopXYZ) ;
                                                       return sTopXYZ ;
                                                     }

  void SetBaseRadius( double dBaseRadius ) { SM_ASSERT( dBaseRadius >= 0.0 );  m_dBaseRadius = dBaseRadius; }

  SmStatus GetApex    
  (
    SmBoolean & bCylinder,                    ///< [out]: FALSE = shape is a cone and has an apex point                                  <br>
                                              ///<      : TRUE  = shape is a cylinder without an apex and the cone center is returned    <br>
    SmPoint3d & rApexPoint,                   ///< [out]: Cone apex point when bIsCylinder==FALSE, else Cylinder origin point            <br>
    double    & dApexParam                    ///< [out]: AnalV Param of rApexPoint
  ) const ;

  SmStatus GetCanonical
  (
    SmAxis2Placement & rOrigin,               ///< [out]: m_vOrigin = STEP BotCircle Center Point                     <br>
                                              ///<      : m_vXAxis  = STEP Surface  0 degree rotation                 <br>
                                              ///<      : m_vYAxis  = STEP Surface 90 degree rotation                 <br>
    double           & rdRadius,              ///< [out]: BotCircle Radius at rOrigin.m_vOrigin                       <br>
    double           & rdSemiAngleDeg,        ///< [out]: angle between Surface and cone axis                         <br>
                                              ///<      : 0.0      = Cylinder (BotCircleRadius == TopCircleRadius)    <br>
                                              ///<      : Positive = BotCircleRadius < TopCircleRadius                <br>
                                              ///<      : Negative = BotCircleRadius > TopCircleRadius                <br>
    SmBoolean        * pOptSwapUV = NULL,     ///< [out]: TRUE = Nurb and Analytic U and V directions are swapped     <br>
                                              ///<      : NULL to ignore, default:[NULL]                              <br>
    SmBoolean        * pOptInsideOut = NULL   ///< [out]: TRUE = Nurb and Analytic genCurve directions are swapped    <br>
                                              ///<      : NULL to ignore, default:[NULL]                              <br>
  ) const ;

  SmBoolean GetPositivePoint
  (
    const SmPoint3d & crTestPoint,            ///< [in] : Point to check                                                                <br>
    SmPoint3d       & rPositivePoint,         ///< [out]: equal to rTestPoint when TestPoint is on the postive side of the apex         <br>
                                              ///<      : else set to rTestPoint reflected through apex point                           <br>
    double          & dApexParam              ///< [out]: Param value on the GenCurve for the apex point.                               <br>
  ) const ;

  SmExtent1d GetSTEPLinearParamExtent(SmSurfParamType * pSurfParam = NULL) const;

  virtual SmExtent2d GetMaxAnalyticDomain() const;

  virtual SmStatus GlobalSurfaceIntersect
  (
    const SmContext     & crContext,               ///< [in] : Context for new object construction                                                     <br>
    const SmExtent2d    & crUVDomain,              ///< [in] : this Surface intersection limits                                                        <br>
    const SmSurface     & crOtherSurface,          ///< [in] : target 2nd intersecting surface                                                         <br>
    const SmExtent2d    & crOtherUVDomain,         ///< [in] : 2nd Surface intersection limits                                                         <br>
    const SmBoolean       bUseSurfaceEdges[2],     ///< [in] : TRUE = Find intersection curve start points by intersecting                             <br>
                                                   ///<      :        the edges of one surface with the other.                                         <br>
                                                   ///<      : Normally both are TRUE unless you know                                                  <br>
                                                   ///<      : that the edges of one surface do not intersect                                          <br>
                                                   ///<      : the other surface.  It is a slight optimization                                         <br>
                                                   ///<      : to set the flag to FALSE                                                                <br>
    const SmApproxTol3d * pdOptApproxTol3d,        ///< [in] : If not given it uses 1/1000 of surface size as approximation tolerance                  <br>
    const double        * pdOptAngTolRad,          ///< [in] : If not given it uses 30 degrees                                                         <br>
    SmTArray<SmCurve*>  * pOpt3DCurves,            ///< [out]: 3D curves produced by intersection                                                      <br>
    SmTArray<SmCurve*>  * pOptSurface1UVCurves,    ///< [out]: UV curves on this surface produced by intersection                                      <br>
    SmTArray<SmCurve*>  * pOptSurface2UVCurves,    ///< [out]: UV curves on crOtherSurface produced by intersection                                    <br>
    SmTArray<SmTsectCurveType> * pOptCurveTypes,   ///< [out]: What type of curve is produced -                                                        <br>
                                                   ///<      :    SM_TC_TOUCHING   - single point intersection (surf norms parallel)                   <br>
                                                   ///<      :    SM_TC_CROSSING   - curve intersection (surf norms not parallel)                      <br>
                                                   ///<      :    SM_TC_TANGENT    - curve intersection (surf norms parallel)                          <br>
                                                   ///<      :    SM_TC_COINCIDENT - curve intersection (surf norms parallel & cross-tangents equal)   <br>
                                                   ///<      :    SM_TC_NEAR_TANGENT    - curve has small angle of intersection                        <br>
                                                   ///<      :    SM_TC_REGION_BOUNDARY - curve bounds region within which the surfs are coincident    <br>
    SmTArray<double>    * pOptDeviations           ///< [out]: produced Curve Deviations from each UVCurve to true xSect, NULL to ignore               <br>
  ) const;

  // special case plane intersections called by GlobalSurfaceIntersect for speed when available
  SmStatus IntersectWithPlane
  (
    const SmContext     & crContext,                ///< [in] : context for new object construction                                                        <br>
    const SmExtent2d    & crUVDomain,               ///< [in] : intersection limit for this surface                                                        <br>
    const SmPlane       & crOtherPlane,             ///< [in] : target intersection plane                                                                  <br>
    const SmExtent2d    & crOtherUVDomain,          ///< [in] : intersection limit for target plane                                                        <br>
    const SmBoolean       bUseSurfaceEdges[2],      ///< [in] : TRUE = find xsect curve start points from boundaryCurve/surface xsects                     <br>
                                                    ///<      : typically these values are TRUE - its a small savings if you know                          <br>
                                                    ///<      : the boundaries of one surface don't intersect the other surface                            <br>
    const SmApproxTol3d * pdOptApproxTol3d,         ///< [in] : min distance between distinct 3dPoints                                                     <br>
    const double        * pdOptAngTolRad,           ///< [in] :                                                                                            <br>
    SmBoolean           & rbNeedsMoreIntersections, ///< [out]: TRUE = special case intersection failed-use general intersection                           <br>
    SmTArray<SmCurve*>  * pOpt3DCurves,             ///< [out]: Intersection 3DCurves, NULL to ignore                                                      <br>
    SmTArray<SmCurve*>  * pOptSurface1UVCurves,     ///< [out]: associated UVTrimCurves on this surface, NULL to ignore                                    <br>
    SmTArray<SmCurve*>  * pOptSurface2UVCurves,     ///< [out]: associated UVTrimCurves on plane, NULL to ignore                                           <br>
    SmTArray<SmTsectCurveType> * pOptCurveTypes,    ///< [out]: oneof for each 3DCurve, NULL to ignore                                                     <br>
                                                    ///<      : SM_TC_TOUCHING        - single point intersection (surf norms parallel)                    <br>
                                                    ///<      : SM_TC_CROSSING        - curve intersection (surf norms not parallel)                       <br>
                                                    ///<      : SM_TC_TANGENT         - curve intersection (surf norms parallel)                           <br>
                                                    ///<      : SM_TC_COINCIDENT      - curve intersection (surf norms parallel & cross-tangents equal)    <br>
                                                    ///<      : SM_TC_NEAR_TANGENT    - curve has small angle of intersection                              <br>
                                                    ///<      : SM_TC_REGION_BOUNDARY - curve bounds region within which the surfs are coincident          <br>
    SmTArray<double>    * pOptDeviations            ///< [out]: associated max 3DCurve to surface distance, NULL to ignore                                 <br>
  ) const;

  // special case Cone intersections called by GlobalSurfaceIntersect for speed when available
  SmStatus IntersectWithCone
  (
    const SmContext     & crContext,                ///< [in] : context for new object construction                           <br>
    const SmExtent2d    & crUVDomain,               ///< [in] : This cone intersection NurbDomain                             <br>
    const SmCone        & crOtherCone,              ///< [in] : other cone                                                    <br>
    const SmExtent2d    & crOtherUVDomain,          ///< [in] : other cone intersection NurbDomain                            <br>
    const SmBoolean       bUseSurfaceEdges[2],      ///< [in] : bUseSurfaceEdges - not used                                   <br>
    const SmApproxTol3d * pdOptApproxTol3d,         ///< [in] : intersection 3d tol or                                        <br>
                                                    ///<      : SM_EFF_ZERO_SQRT * (1.0 + otherConeBBox.Length())             <br>
    const double        * pdOptAngTolRad,           ///< [in] : pdOptAngTolRad - not used                                     <br>
    SmBoolean           & rbNeedsMoreIntersections, ///< [out]: TRUE = use general surf/surf intersector to find intersection <br>
                                                    ///<      : FALSE= intersection curves returned by this function          <br>
    SmTArray<SmCurve*>  * pOpt3DCurves,             ///< [out]: required array of output intersections                        <br>
    SmTArray<SmCurve*>  * pOptSurface1UVCurves,     ///< [out]: optional associated cone1 UVTrimCurves                        <br>
    SmTArray<SmCurve*>  * pOptSurface2UVCurves,     ///< [out]: optional associated cone2 UVTrimCurves                        <br>
    SmTArray<SmTsectCurveType> * pOptCurveTypes,    ///< [out]: optional associated intersection types                        <br>
    SmTArray<double>    * pOptDeviations            ///< [out]: optional associated deviations fro each xSect Curve           <br>
  ) const;

  virtual SmBoolean IsCylinder() const { return (smos_Fabs(m_dSemiAngleDeg) < SM_EFF_ZERO) ; }

  static SmBoolean IsNurbSurfaceCone
  (
    const SmContext        & crContext,               ///< [in] : context for new object construction                                         <br>
    const SmBSplineSurface * pTestSurface,            ///< [in] : Surface to test                                                             <br>
    SmCone                *& rpCone,                  ///< [out]: new Cone when surface is cone - else NULL                                   <br>
                                                      ///<      : rpCone parameterization is likely to differ from pTestSurface               <br>
    double                   dToleranceScale = 1.0    ///< [in] : tol = dToleranceScale * SM_EFF_ZERO * ANALYTIC_TOL_SCALE * BoundingBoxSize  <br>
  );

  SmStatus RotationAboutAxisZ
  (
    const SmContext & crContext,            ///< [in] : new object context            <br>
    double            dAngleDeg,            ///< [in] : rotation amount in degrees    <br>
    SmCone         *& rpNewCone             ///< [out]: new rotated cone              <br>
  );

  virtual SmStatus STEPInversion
  (
    const SmExtent2d & crAnalUVDomain,      ///< [in] : crAnalDomain               <br>
    const SmPoint3d  & crPointOnSurf,       ///< [in] : target point               <br>
    double             d3DTolerance,        ///< [in] :                            <br>
    SmPoint2d        & rdAnalUVParameter,   ///< [out]:                            <br>
    SmLocationType   & reLocation,          ///< [out]:                            <br>
    SmPoint2d        * pUVGuess = NULL      ///< [in] : opt: guess parameter.      <br>
  ) const;

  virtual SmStatus Transform(const SmAxis2Placement & crRotateNMove,     // in : affine rotate and move transformation      
                             const SmVector3d       * cpOptScale=NULL) ; // in : optional scaling about current origin point before RotateNMove
                                                                         //      BSplines, planes, lines, PolyBreps - support nonisotropic scaling
                                                                         //      other geom types only support isoptropic scaling

  // get memory used for curve but not its attributes
  virtual ULONG GetMemoryUsed
  (
    ULONG    & rlMemoryAllocated,           ///< [out]: bigger size of all allocated memory in bytes     <br>
    SmMarkType eMarkType=SM_MT_NOMARK       ///< [in] : uses without increment eMarkType value           <br>
  ) const ;

  virtual void Dump(SmBoolean bAbbrev)  const;  // NotUsed: in :

  // Internal - non-public methods 
  virtual SmBoolean AssertValid
  (
    SmAssertArray    * pAList=NULL,           ///< [in,out]: Accumulating list of failed Asserts, NULL to ignore                                            <br>
    SmAssertTestLevel  eTestLevel=SM_LEVEL_0, ///< [in] : SM_LEVEL_0=fewest tests, SM_LEVEL_1, SM_LEVEL_2=all tests,                                        <br>
                                              ///<      : SM_LEVEL_GIVEN = run tests in order requested in pTestRequests                                    <br>
                                              ///<      : default:[SM_LEVEL_0]                                                                              <br>
    SmAssertWalking    eWalkTree=SM_WALK,     ///< NotUsed: [in] : SM_WALK = run AssertValid on any topology graph descendants, SM_NO_WALK=don't, default:[SM_WALK]  <br>
    SmTArray<ULONG>  * pTestRequests=NULL     ///< [in] : when eTestLevel == SM_LEVEL_GIVEN, run these tests in this order, default:[NULL]                  <br>
  ) const ;

  // obsolete
  // virtual SmBoolean AssertHeal (SmAssertReport & rAReport, SmAssertArray * pAList) ;

  virtual SmStatus WriteToDB
  (
    SmDatabaseIO & rDB,                      ///< [in] : target output stream                                                          <br>
    ULONG          lDBVersionNumber          ///< [in] : database version to get proper sequence of writes                             <br>                                          
  ) const;

  static  SmStatus ReadFromDB
  (
    SM_TYPE           lType,                 ///< NotUsed: [in] : Object type to be read                                                         <br>
    SmDatabaseIO    & rDB,                   ///< [in] : target output stream                                                           <br>
    const SmContext & crContext,             ///< [in] : context for new object construction                                            <br>
    SmSurface      *& rpNewSurface,          ///< [out]: NULL on input = new object allocated in this routine built from stream data    <br>
                                             ///<      : NotNULL on input = pointer to an empty object to be filled by this routine     <br>
    ULONG             lDBVersionNumber       ///< [in] : database version to get proper sequence of writes                              <br>
  ) ; 

  // Define  GetType(), IsKindOf(), and virtual Dump() (Dump needs local implementation)
  SM_COMMON(SmCone,SmSurfOfRevolution,SmCone_TYPE);

} ; // end class SmCone
 
#endif // !__SMCONE_H__
