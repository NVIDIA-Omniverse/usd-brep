// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************//**
* FILE NAME --- SmTorus.h
* PURPOSE: Header file for Torus Surface class.
**********************************************************************/

#ifndef __SMTORUS_H__
#define __SMTORUS_H__

#ifndef __SMSURFOFREVOLUTION_H__
#include <SmSurfOfRevolution.h>
#endif

#ifndef __SMAXIS2PLACEMENT_H__
#include <SmAxis2Placement.h>
#endif

/*******************************************************************//**
PURPOSE: Represent a Torus surface made from a SmSurfOfRotation surface.

    The STEP equation of the torus  is

    S(U,V) =   origin                             
             + MajRadius * cos(U) * XAxis   
             + MajRadius * sin(U) * YAxis
             + MinRadius * cos(V) * (  cos(U) * XAxis
                                     + sin(U) * YAxis)
             + MinRadius * sin(V) * ZAxis

       with U rangeDeg:[-360 to 360] maxLength = 360
            V rangeDeg:[-360 to 360]
    
    U parameter = CCW major rotation around the torus' Z axis from the XAxis in degrees (0.0 - 360.0). 
    V parameter = CCW minor rotation about majorRadius point 
                      around MajorAxis*ZAxis vectorfrom XAxis in degrees

NOTES:
    The STEP parameterization used for the torus is different than
    the Nurb parameterization used for the underlying NURB surface.
    The map from one to the other is not linear. 

  Translating between STEP and NURB parameters:
    m_bSwapUV    is set when the underlying NURB V param
                 maps to the rotation direction.
    m_bInsideout is set when tThe underlying GenCurve NURB direction
                 rotates in a CW fashion about the MajorAxis*ZAxis vector.

    1. Rotation direction - SmSurfOfRevolution::m_vPolarConverter
        The ZAxis is always set so that the torus sweeps are made
        CCW about Z
    2. Torus genCurves are SmCircles and have 3 parameter spaces:
         1. StepParams for a 'STEPGenCurve' are in degrees 
         2. PolarParams for a 'PolarCurve' are in degrees and either 
                 the Same or Opposite of the STEPParams as:
            PolarParam = m_bInsideOut ? -StepParam.y : StepParam.y.
         3. Nurb params are determined by the genCurve's knot vector but
            always run in the same general direction as the PolarParams
       To Convert GenCurve STEPToNurb:  StepParam -> PolarParam -> NurbParam
       To Convert GenCurve NurbToStep:  NurbParam -> PolarParam -> StepParam
    3. m_bSwapUV = TRUE 
       NurbU = STEPV and
       NurbV = STEPU

  NOTES: m_bSwapUV and m_bInsideOut are not used to make STEP evaluations

         STEP/Nurb Evaluators when NurbUV = Convert(StepUV)
           EvaluatePoint(NurbUV) == EvaluateSTEPPoint(StepUV) 
           However Tangents will have different magnitudes and
                            are swapped when m_bSwapUV is true
                            has a negated GenCurve tangent when m_bInsideOut is true.
                   Normals will be negated when
                            (m_bSwapUV XOR m_bInsideOut).

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
   | GlobalPointSolve()         |                   | NURB parameter             |
   | LocalPointSolve()          | NURB parameter    | NURB parameter             |
   +----------------------------+-------------------+----------------------------+

***********************************************************************/
class SM_EXPORT SmTorus : public SmSurfOfRevolution
{
protected:
  // inherited
// Remove Composites
// // SmSurface::m_pOwner                 - NULL or ptr to SmFace or SmCFace
  // SmSurface::m_pOwner                 - NULL or ptr to SmFace
  // SmBSplineSurface::m_pNurb           - ptr to BSplineSurface controlPoints and knots
  // SmSurfOfRevolution::m_vPosition     - ZAxis = axis of revolution
  //                                       XAxis = 0/360 vector of angular domain
  // SmSurfOfRevolution::m_vAnalUVDomain - x = angular domain in degs measured CCW from x 
  //                                       y = linear domain from bottom to top
  // SmSurfOfRevolution::m_bSwapUV       - TRUE: NurbU = STEPV,  FALSE: NurbU = STEPU
  //                                             NurbV = STEPU          NurbV = STEPV

  double     m_dMajorRadius = SM_UNDEF_DOUBLE ; // size of offset circle centered on placement origin
  double     m_dMinorRadius = SM_UNDEF_DOUBLE ; // size of genCurve Circle swept about the offset circle

  // GWC:NOTE m_bInsideOut not fully supported yet
  SmBoolean  m_bInsideOut = FALSE ; // FALSE if normal does not point outward.

  // GWC:NOTE TORUS needs m_bInsideOut to support
  //          Nurbs whose minor circles have reversed parameterization 

  // empty constructor for I/O
  SmTorus() { }

  // copy constructor
  SmTorus(const SmTorus & crTorus);

public:
  // Construction
  SmTorus
  (
    const SmPoint3d  & crOrigin,             ///< [in ]: center of major circle                                                 <br>
    const SmVector3d & crXAxis,              ///< [in ]: major circle start/end direction [0 and 360 degrees]                   <br>
    const SmVector3d & crYAxis,              ///< [in ]: major circle 90 degree direction                                       <br>
    const SmExtent2d & crAnalUVDomain,       ///< [in ]: [major circle interval, minor circle interval] in degrees              <br>
    double             dMajorRadius,         ///< [in ]: major circle radius                                                    <br>
    double             dMinorRadius,         ///< [in ]: minor circle radius                                                    <br>
    SmBoolean          bSwapUV=FALSE,        ///< [in ]: TRUE = underlying NURB UV directions will be switched                  <br>
    SmBoolean          bInsideOut=FALSE,     ///< [in ]: TRUE = GenCurve runs in reverse direction from defintion               <br>
                                             ///<      : FALSE=                                                                 <br>
    SmBSplineCurve   * pOptCircleNurb=NULL,  ///< [in ]: Optional Curve Pointer to define GenCurve parameterization             <br>
                                             ///<      : SurfOfRevolution owns this curve and will delete it when destructed.   <br>
    const SmContext  * cpContext=NULL        ///< [in ]: Set context if given, default:[NULL]                                   <br>
  );

  // destructor
  virtual ~SmTorus() { }

  // equality operator
  virtual SmBoolean operator==(const SmSurface &crOther) const;

  virtual SmStatus Copy
  (
    const SmContext & crContext,
    SmSurface *& rpNewSurface
  ) const;
                            
  virtual SmStatus  RebuildSTEPFromNURBParameters() ;

  // also SmBSplineSurface::AreSTEPAndNURBCurrent() const ;
  //      SmBSplineSurface::RebuildNURBFromSTEPParameters() ;

  // create a step parameterized surface                          
  static SmStatus CreateCanonical
  (
    const SmContext        & crContext,                 ///< [in ]: new object construction                                             <br>
    const SmAxis2Placement & crOrigin,                  ///< [in ]: m_vOrigin = center of torus                                         <br>
                                                        ///<      : m_vXAxis  = Sweep Start                                             <br>
                                                        ///<      : m_vYAxis  = Sweep 90 degree direction                               <br>
    double                   dMajorRadius,              ///< [in ]: radius of offset circle centerd on m_vOrigin                        <br>
    double                   dMinorRadius,              ///< [in ]: radius of genCurve Circle being swept                               <br>
    SmTorus               *& rpNewTorus,                ///< [out]: new object                                                          <br>
    SmBoolean                bSwapUV = FALSE,           ///< [in ]: TRUE = underlying NURB UV directions will be switched               <br>
    SmBoolean                bInsideOut = FALSE,        ///< [in ]: TRUE = GenCurve runs in reverse direction from defintion            <br>
                                                        ///<      : FALSE=                                                              <br>
    SmBSplineCurve         * pOptCircleNurb = NULL,     ///< [in ]: Optional Curve Pointer to define GenCurve parameterization          <br>
                                                        ///<      : SurfOfRevolution owns this curve and will delete it when destructed.<br>
    const SmExtent2d       * cpOptAnalUVDomain = NULL   ///< [in ]: Analytic UV Domain [u=rotation in degrees, v=genCurve param]
  );

  // create offset surface - Out Surf->Domain(s) may be trimmed but not scaled
  virtual SmStatus CreateOffsetSurface
  (
    const SmContext      & crContext,               ///< [in ]: context for new obj construction                                             <br>
    double                 dSignedOffsetDistance,   ///< [in ]: offset dist, (neg val = Offset dir opposite surface normal)                  <br>
    SmApproxTol3d          sApproxTol3d,            ///< [in ]: Max Dist between ApproxOffsetSurface and ideal offset shape                  <br>
    SmSurface* &           rOffsetSurface           ///< [out]: Offset Surf Approx, may be more than 1 when offsets have self-intersections  <br>
  ) const;

  virtual SmStatus ConvertUVFromSTEPToNURBS
  (
    const SmPoint2d & crSTEPUV,                     ///< [in ]: Target 2d Point in [degrees, height]        <br>
    SmPoint2d       & rNURBSUV                      ///< [out]: 2d Point in SMLib parameterization          <br>
  ) const;

  virtual SmStatus ConvertUVFromNURBSToSTEP
  (
    const SmPoint2d & crNURBSUV,                    ///< [in ]: Target 2d Point in SMLib parameterization    <br>
    SmPoint2d       & rSTEPUV                       ///< [out]: 2d Point in [degrees, height]                <br>
  ) const;

  // Evaluate the point and triangular derivatives through order 3 at the given STEP UV point.
  // Angular parameters are in degrees; derivatives are per degree. See SmSTEPSurface.h.
  virtual SmStatus EvaluateSTEP
  (
    const SmPoint2d & crUV,            ///< [in ]: U=CCW Rot about Z from X in degrees,          [0 to 360]                                  <br>
                                       ///<      : V=param from bot pole to top pole in degrees, [-90 to 90]                                 <br>
                                       ///<      : when m_bMakeNurbGenCurve = FALSE, V is angle in degrees                                   <br>
                                       ///<      :      m_bMakeNurbGenCurve = TRUE , V is close to an angle in degrees                       <br>
    ULONG             lHighestUDeriv,  ///< [in ]: Requested triangular derivative order in U; must equal lHighestVDeriv; max 3               <br>
    ULONG             lHighestVDeriv,  ///< [in ]: Requested triangular derivative order in V; must equal lHighestUDeriv; max 3               <br>
    SmBoolean         bUFromLeft,      ///< [in ]: not used - if P is on U interval boundary                                                 <br>
                                       ///<      : TRUE  = evaluate P in upper interval where P is on the left of the interval               <br>
                                       ///<      : FALSE = evaluate P in lower interval where P is on the right of the interval              <br>
    SmBoolean         bVFromLeft,      ///< [in ]: not used - if P is on V interval boundary                                                 <br>
                                       ///<      : TRUE  = evaluate P in upper interval where P is on the left of the interval               <br>
                                       ///<      : FALSE = evaluate P in lower interval where P is on the right of the interval              <br>
    SmBoolean         bOnlyUpperHalf,  ///< [in ]: not used output always set for bOnlyUpperHalf=TRUE                                        <br>
    SmVector3d      * aDerivatives     ///< [out]: matrix of evaluations values                                                              <br>
                                       ///<      : sized:[lHighestUDeriv+1][lHighestVDeriv+1]                                                <br>
                                       ///<      : 2d organized: [D    Du   ]                                                                <br>
                                       ///<      :               [Dv   -    ]                                                                <br>
                                       ///<      : 1d organized: [D, Dv, Dvv, Dvvv,.. Du, Duv, Duvv, Duvvv,.. Duu, Duuv, Duuvv, Duuvvv,...]  <br>
  ) const;

  virtual SmStatus EvaluateSTEPPoint
  (
    const SmPoint2d & crUV,           ///< [in ]: target UVPoint, range:[0 to 360, -90 to 90]     <br>
    SmPoint3d       & rPoint          ///< [out]: sphere point                                    <br>
  ) const;

  // create IsoParamCurve from underlying NurbSurface and use analytic Params to return SmCircle
  virtual SmStatus CreateIsoParametricCurve
  (
    const SmContext  & crContext,                  ///< [in ]: context for created objects                                                                  <br>
    SmSurfParamType    eSurfParam,                 ///< [in ]: Defines which Nurb parameter direction on surface to extract curve from                      <br>
                                                   ///<      : SM_SP_U = create constant u isoParameter curve                                               <br>
                                                   ///<      : SM_SP_V = create constant v isoParameter curve                                               <br>
    double             dIsoParameter,              ///< [in ]: Defines Nurb parametric value at which to extract the curve.                                 <br>
                                                   ///<      : If eSurfParam==SM_SP_U this is a U parameter, if eSurfParam==SM_SP_V                         <br>
                                                   ///<      : then this is the V parameter                                                                 <br>
    SmApproxTol3d      sApproxTol3d,               ///< NotUsed: [in ]: passed to ApproximateCurve() when approximation is required.                                 <br>
                                                   ///<      : If set to 0.0, tolerance is set by system: old[curve length * 1.0e-4] new[GetApproxTol3d()]  <br>
    SmBSplineCurve  *& rpNewIsoCurve,              ///< [out]: 3d IsoParameterCurve                                                                         <br>
    const SmExtent2d * pOptDomain = NULL,          ///< [in ]: optional trim bound for IsoParameterCurve, NULL to ignore, default:[NULL]                    <br>
    double           * pOptMaxGap3d = NULL,        ///< [out]: opt achieved max gap, NULL to ignore, default:[NULL]                                         <br>
    SmCurve         ** pOptUVIsoCurve = NULL       ///< [out]: opt 2d UVTrimCurve Line (diff parameterization), NULL to ignore, default:[NULL]              <br>
  ) const;

  virtual SmBoolean GetInsideOut()   const { return m_bInsideOut; }

  double            GetMajorRadius() const { return m_dMajorRadius; }  

  double            GetMinorRadius() const { return m_dMinorRadius; }   

  SmStatus          GetCanonical
  (
    SmAxis2Placement & rOrigin,                     ///< [out]: m_vOrigin = STEP BotCircle Center Point                     <br>
                                                    ///<      : m_vXAxis  = STEP Surface  0 degree rotation                 <br>
                                                    ///<      : m_vYAxis  = STEP Surface 90 degree rotation                 <br>
    double           & rdMajorRadius,               ///< [out]: radius from torus center to minor circle center             <br>
    double           & rdMinorRadius,               ///< [out]: radius of minor circle                                      <br>
    SmBoolean        * pOptSwapUV = NULL,           ///< [out]: TRUE = Nurb and Analytic U and V directions are swapped     <br>
                                                    ///<      : NULL to ignore, default:[NULL]                              <br>
    SmBoolean        * pOptInsideOut = NULL         ///< [out]: TRUE = Nurb and Analytic genCurve directions are swapped    <br>
                                                    ///<      : NULL to ignore, default:[NULL]                              <br>
  ) const ;

  virtual SmExtent2d GetMaxAnalyticDomain() const
  {
    SmExtent2d sDom( 0, 0, 360, 360 ); return sDom;
  }

  // when Nurb is Torus, rtn TRUE and make SmTorus object with same Nurb parameterization
  static SmBoolean IsNurbSurfaceTorus
  (                                                                                                                     
    const SmContext        & crContext,                    ///< [in ]: new object context                                    <br>
    const SmBSplineSurface * pTestSurface,                 ///< [in ]: BSplineSurface to be checked                          <br>
    SmTorus               *& rpTorus,                      ///< [out]: new Torus or NULL                                     <br>
    double                   dToleranceScale = 1.0         ///< [in ]: extra scale value for ScaledZero computation          <br>
  );                                                      

  SmStatus RotationAboutAxisZ
  (
    const SmContext & crContext,
    double            dAngleDeg,
    SmTorus        *& rpNewTorus
  );

  virtual SmStatus Transform(const SmAxis2Placement & crRotateNMove,     // in : affine rotate and move transformation      
                             const SmVector3d       * cpOptScale=NULL) ; // in : optional scaling about current origin point before RotateNMove
                                                                         //      BSplines, planes, lines, PolyBreps - support nonisotropic scaling
                                                                         //      other geom types only support isoptropic scaling

  // utilities

  // get memory used for curve but not its attributes
  virtual ULONG GetMemoryUsed
  (
    ULONG    & rlMemoryAllocated,             ///< [out]: bigger size of all allocated memory in bytes        <br>
    SmMarkType eMarkType=SM_MT_NOMARK         ///< [in ]: uses without increment eMarkType value              <br>
  ) const ;

  virtual SmBoolean AssertValid
  (
    SmAssertArray    * pAList=NULL,           ///< [in,out]: Accumulating list of failed Asserts, NULL to ignore                                            <br>
    SmAssertTestLevel  eTestLevel=SM_LEVEL_0, ///< [in ]: SM_LEVEL_0=fewest tests, SM_LEVEL_1, SM_LEVEL_2=all tests,                                        <br>
                                              ///<      : SM_LEVEL_GIVEN = run tests in order requested in pTestRequests                                    <br>
                                              ///<      : default:[SM_LEVEL_0]                                                                              <br>
    SmAssertWalking    eWalkTree=SM_WALK,     ///< NotUsed: [in ]: SM_WALK = run AssertValid on any topology graph descendants, SM_NO_WALK=don't, default:[SM_WALK]  <br>
    SmTArray<ULONG>  * pTestRequests=NULL     ///< [in ]: when eTestLevel == SM_LEVEL_GIVEN, run these tests in this order, default:[NULL]                  <br>
  ) const ;                                                                                                                                              

  // obsolete
  // virtual SmBoolean AssertHeal (SmAssertReport & rAReport, SmAssertArray * pAList) ;

  virtual SmStatus WriteToDB
  (
    SmDatabaseIO & rDB,                      ///< [in ]: target output stream                                <br>
    ULONG          lDBVersionNumber          ///< [in ]: database version to get proper sequence of writes   <br>
  ) const ;                                                                        

  static  SmStatus ReadFromDB
  (
    SM_TYPE           lType,                 ///< NotUsed: [in ]: Object type to be read                                                       <br>
    SmDatabaseIO    & rDB,                   ///< [in ]: target output stream                                                         <br>
    const SmContext & crContext,             ///< [in ]: context for new object construction                                          <br>
    SmSurface      *& rpNewSurface,          ///< [out]: NULL on input = new object allocated in this routine built from stream data  <br>
                                             ///<      : NotNULL on input = pointer to an empty object to be filled by this routine   <br>
    ULONG             lDBVersionNumber       ///< [in ]: database version to get proper sequence of writes                            <br>
  );

  // Define  GetType(), IsKindOf(), and virtual Dump() (Dump needs local implementation)
  SM_COMMON(SmTorus,SmSurfOfRevolution,SmTorus_TYPE);

} ; // end class SmTorus


#endif // !__SMTORUS_H__


