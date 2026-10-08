// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*******************************************************************//**
* FILE NAME --- SmCylinder.h
* PURPOSE: Header file for Cone Surface class.
**********************************************************************/

#ifndef __SMCYLINDER_H__
#define __SMCYLINDER_H__

#ifndef __SMCONE_H__
#include <SmCone.h>
#endif

#ifndef __SMAXIS2PLACEMENT_H__
#include <SmAxis2Placement.h>
#endif

/*******************************************************************//**
PURPOSE: Represent a Cylinder surface made from a Cone which is
            made from an SmSurfOfRotation surface.

    The STEP equation of the cylinder is simplified from the Cone by setting
    m_dSemiAngleDeg = 0.0 

    When m_dSemiAngle = 0.0
        Radius  = m_dBaseRadius 
       Cone(U,V) =   origin                             
                   + cos(U) * Radius * XAxis
                   + sin(U) * Radius * YAxis
                   + V               * ZAxis ;
       with U rangeDeg:[-360 to 360] maxLength = 360
            V range   :[minV to maxV], maxV - minV == height
    
    U parameter = CCW rotation around the cylinder's Z axis from the XAxis in degrees.
                  U rangeDeg:[-360 to 360] maxLength = 360.
                  U Parameterization is in degrees. 
    V parameter = Up and Down the Cylinder's Z axis. range:[0 Height]

    Translating between STEP and NURB parameters is inherited behavior from SmCone and
    SmSurfOfRevolution.  The representation allows maps the NURB U and V parameters to
    the STEP U and V parameters based on the m_dSwapUV and m_dInsideOut parmeters.
    These values are not used to make STEP evaluations but are used to map NURB
    evaluations to their final positions to be equivalent to the STEP evaluations.

    When m_bSwapUV is TRUE, STEPV maps to NurbU and STEPU maps to NurbV, otherwise the opposite.
    When m_bInsideOut is TRUE, the underlying NURB linear direction runs from STEPMaxV to STEPMinV.

NOTES:

 The STEP parameterization used for the cylinder is different than
 the Nurb parameterization used for the underlying NURB surface.
 The map from one to the other is not linear. 

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
class SM_EXPORT SmCylinder : public SmCone
{
  // inherited
// Remove Composites
// // SmSurface::m_pOwner                 - NULL or ptr to SmFace or SmCFace
  // SmSurface::m_pOwner                 - NULL or ptr to SmFace
  // SmBSplineSurface::m_pNurb           - ptr to BSplineSurface controlPoints and knots
  // SmSurfOfRevolution::m_vPosition     - ZAxis = axis of revolution
  //                                       XAxis = 0/360 vector of angular domain
  // SmSurfOfRevolution::m_vAnalUVDomain - x = angular domain in degs measured CCW from x 
  //                                       y = linear domain from bottom to top
  // SmCone::m_dBotRadius                - radius of cylinder

protected:
  // Construction is private and used only by cylinder methods.
    
  // create an infinite closed cylinder - centered on crOrigin
  //                                    - when bSwapUV==FALSE and bInsideOUt==FALSE
  //                                      - u-parameterization (rotational): [ 0, 360 ]
  //                                      - v-parameterization (along axis): [ -SM_INFINITE_PARAMETER, SM_INFINITE_PARAMETER ]
  //                                      - Cylinder SrfNormal points from SrfPt away from Cylinder axis

  SmCylinder
  (
    const SmPoint3d  & crOrigin,            ///< [in] : point on cylinder axis                                             <br>
    const SmVector3d & crXAxis,             ///< [in] : vector perp to axis to cylinder start point                        <br>
    const SmVector3d & crYAxis,             ///< [in] : vector perp to axis and X Axis                                     <br>
    double             dRadius,             ///< [in] : distance from axis to cylinder wall                                <br>
    SmBoolean          bSwapUV = FALSE,     ///< [in] : TRUE = underlying NURB surface v dir maps to rotation direction    <br>
                                            //      FALSE= underlying NURB surface u dir maps to rotation direction        <br>
    SmBoolean          bInsideOut = FALSE,  ///< [in] : TRUE = underlying NURB linear dir runs from TopCircle to BotCircle <br>
                                            //      FALSE= underlying NURB linear dir runs from BotCircle to TopCircle     <br>
    const SmContext  * cpContext=NULL       ///< [in] : Set context if given, default:[NULL]                               <br>
  );     

  // empty constructor for I/O
  SmCylinder() { }

  // copy constructor
  SmCylinder(const SmCylinder & crCylinder);

public:
  // create a bounded cylinder - most users use CreateCanonical() 
  //                           - when bSwapUV==FALSE and bInsideOUt==FALSE
  //                             - cylinder BotCircle centered on crOrigin.
  //                             - u-parameterization (rotational): [ dStartRotAngle, dEndRogAngle ]
  //                             - v-parameterization (along axis): [ 0, dHeight ]
  //                             - Cylinder SrfNormal points from SrfPt away from Cylinder axis
  SmCylinder
  (
    const SmPoint3d  & crOrigin,            ///< [in] : zero Point on Cylinder's ZAxis                                     <br>
    const SmVector3d & crXAxis,             ///< [in] : Cylinder's X Axis (vector perp to axis to cylinder start point)    <br>
    const SmVector3d & crYAxis,             ///< [in] : Cylinder's Y Axis (vector perp to axis and X Axis)                 <br>
    double             dRadius,             ///< [in] : Cylinder's radius (distance from axis to cylinder wall)            <br>
    double             dStartAngleDeg,      ///< [in] : CCW about Z from X, rangeDeg:[-360 to 360] maxLength = 360         <br>
    double             dEndAngleDeg,        ///< [in] : CCW about Z from X, rangeDeg:[-360 to 360] maxLength = 360         <br>
    double             dHeight,             ///< [in] : Defined domain from zero point in Z direction                      <br>
    SmBoolean          bSwapUV,             ///< [in] : TRUE = underlying NURB surface v dir maps to rotation direction    <br>
                                            ///<      : FALSE= underlying NURB surface u dir maps to rotation direction    <br>
    SmBoolean          bInsideOut,          ///< [in] : TRUE = underlying NURB linear dir runs from TopCircle to BotCircle <br>
                                            ///<      : FALSE= underlying NURB linear dir runs from BotCircle to TopCircle <br>
    SmBoolean          bMakeNurbGenCurve,   ///< [in] : TRUE = GenCurve is type SmBSplineCurve                             <br>
                                            ///<      : FALSE= GenCurve is type SmLine                                     <br>
    const SmContext  * cpContext=NULL       ///< [in] : Set context if given, default:[NULL]                               <br>
  );     

  // destructor
  virtual ~SmCylinder() { }

  // equality operator
  virtual SmBoolean operator==(const SmSurface &crOther) const;

  // deep copy operator
  virtual SmStatus Copy
  (
    const SmContext & crContext,
    SmSurface      *& rpNewSurface
  ) const;
                              
  // create a step parameterized infinite closed Cylinder                          
  static SmStatus CreateCanonical
  (
    const SmContext        & crContext,       ///< [in] : context for new object construction                <br>
    const SmAxis2Placement & crOrigin,        ///< [in] : origin = Point on cylinder centerline              <br>
                                              ///<      : XAxis  = zero degree direction from centerline     <br>
                                              ///<      : YAxis  = 90 degree direction from centerline       <br>
                                              ///<      : ZAxis  = centerline direction                      <br>
    double                   dRadius,         ///< [in] : cylinder radius                                    <br>
    SmCylinder            *& rpNewCylinder    // out: new infinite cylinder                                  <br>
  ) ; 

  // simple access
  virtual SmExtent2d GetMaxAnalyticDomain() const;

  double             GetRadius           () const { return GetBotRadius(); }

  SmStatus           GetCanonical        (SmAxis2Placement & rOrigin,  double & rdRadius) const ;

  // predicates
  virtual SmBoolean IsCylinder() const { return TRUE; }

  static  SmBoolean IsNurbSurfaceCylinder
  (
    const SmContext        & crContext,               ///< [in] : <br>
    const SmBSplineSurface * pTestSurface,            ///< [in] : <br>
    SmCylinder            *& rpCylinder,              ///< [in] : <br>
    double                   dToleranceScale = 1.0    ///< [in] : <br>
  );

  // Construct a new Cylinder rotated about this ZAxis by dAngleDeg - don't modify this object
  SmStatus RotationAboutAxisZ
  (
    const SmContext & crContext,       ///< [in] :    <br>
    double            dAngleDeg,       ///< [in] :    <br>
    SmCylinder     *& rpNewCylinder    ///< [in] :    <br>
  );

  // utilities

  virtual SmBoolean AssertValid
  (
    SmAssertArray    * pAList=NULL,           /// [in,out]: Accumulating list of failed Asserts, NULL to ignore                            <br>
    SmAssertTestLevel  eTestLevel=SM_LEVEL_0, ///< [in] : SM_LEVEL_0=fewest tests, SM_LEVEL_1, SM_LEVEL_2=all tests,                       <br>
                                              ///<      : SM_LEVEL_GIVEN = run tests in order requested in pTestRequests                   <br>
    SmAssertWalking    eWalkTree=SM_WALK,     ///< NotUsed: [in] : SM_WALK = run AssertValid on any topology graph descendants, SM_NO_WALK=don't    <br>
    SmTArray<ULONG>  * pTestRequests=NULL     ///< [in] : when eTestLevel == SM_LEVEL_GIVEN, run these tests in this order                 <br>
  ) const ;

  // obsolete
  // virtual SmBoolean AssertHeal (SmAssertReport & rAReport, SmAssertArray * pAList) ;

  virtual SmStatus  WriteToDB 
  (
    SmDatabaseIO & rDB,                      ///< [in] : target output stream                                <br>
     ULONG          lDBVersionNumber         ///< [in] : database version to get proper sequence of writes   <br>
  ) const ;                                                                        
                                
  static  SmStatus  ReadFromDB
  (
    SM_TYPE           lType,              ///< NotUsed: [in] : Object type to be read                                                          <br>
    SmDatabaseIO    & rDB,                ///< [in] : target output stream                                                            <br>
    const SmContext & crContext,          ///< [in] : context for new object construction                                             <br>
    SmSurface      *& rpNewSurface,       ///< [out]: NULL on input = new object allocated in this routine built from stream data     <br>
                                          ///<      : NotNULL on input = pointer to an empty object to be filled by this routine      <br>
    ULONG             lDBVersionNumber    ///< [in] : database version to get proper sequence of writes                               <br>
  ) ; 
                      
  // Define  GetType(), IsKindOf(), and virtual Dump() (Dump needs local implementation)
  SM_COMMON(SmCylinder,SmCone,SmCylinder_TYPE);

} ; // end class SmCylinder

#endif // !__SMCYLINDER_H__












