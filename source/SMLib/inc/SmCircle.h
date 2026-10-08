// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/******************************************************************//**
* FILE NAME --- SmCircle.h
* PURPOSE: Header file for Ellipse curve.
**********************************************************************/

#ifndef __SMCIRCLE_H__
#define __SMCIRCLE_H__

#ifndef __SMELLIPSE_H__
#include <SmEllipse.h>
#endif

/*******************************************************************//**
PURPOSE: The SmCircle class defines a circle.
    A circle is a conic section defined by a radius, the location and the
    orientation of the circle.

NOTES:
  Circle(s) =   Origin                   
              + Rad * XAxis * cos(s) 
              + Rad * YAxis * sin(s)  for all s in m_vAnalDomain

  where m_vAnalDomain range:[-360 <= Min <= Max <= 360], Max-Min <= 360.0

  For circles, s = CCW angle from xAxis moving to YAxis

         note: A circle has two confusing domains, angular arc and
               NURB parameter, SMLib works in NURB parameters.

         note: Circle always Starts at its XAxis extrema point
               moving towards the YAxis endPoint.


  For compatibility with Spheres and Tori, Circles have a m_bInsideOut which
         is set when the Nurb domain runs opposite to the STEP domain.

  To translate from Step to Nurb for circles, use 3 domains as:
           STEP Domain <=> Polar Domain <=> Nurb Domain

  When m_bInsideOut == FALSE: Polar_p = STEP_s
  When m_bInsideOut == TRUE : Polar_p = STEP_Min + STEP_max - STEP_s, which swaps the ends of the curve
  Translating between Polar and Nurb domains is done by the class SmPolarConversion

  +---------------------------+--------------------------+--------------------------------------+
  | Evaluator                 |       Input              |  OUTPUT                              |
  +---------------------------+--------------------------+--------------------------------------+
  | EvaluatePoint()           | NURB parameter           | Euclidean Point                      |
  | Evaluate()                | NURB parameter           | Euclidean Point and derivs           |
  | EvaluateSTEP()            | Circle Angle in degrees  | Euclidean Point                      |
  | STEPInversion()           | Euclidean Point          | Circle Angle in degrees              |
  | DropPoint()               | Euclidean Point          | Nearest Circle Point NURB parameter  |
  | ConvertTFromSTEPToNURBS() | Circle Angle in degrees  | NURB parameter                       |
  | ConvertTFromNURBSToSTEP() | NURB parameter           | Circle Angle in degrees              |
  +---------------------------+--------------------------+--------------------------------------+ 
***********************************************************************/
class SM_EXPORT SmCircle : public SmEllipse
{
  // inherited
  //   SmAxis2Placement SmEllipse::m_vPosition;       // origin, X_Axis, and Y_Axis
  //   SmExtent1d       SmEllipse::m_vAnalDomain;     // angular arc domain in degrees,
  //                                                  //   range:[-360 <= Min <= Max <= 360], MaxLength=360.0
  //   double           SmEllipse::m_dRadiusAtXAxis;  // X_Axis radius
  //   double           SmEllipse::m_dRadiusAtYAxis;  // Y_Axis radius (equal for Circle)
public:
  // Construct circle from CenterPoint, Orientation, Radius, and Interval
  SmCircle(const SmPoint3d  & crCenter,              // in : Center of Circle                                          
           const SmVector3d & crXAxis,               // in : X axis of Circle - corresponds to an angle of 0 degrees   
           const SmVector3d & crYAxis,               // in : Y axis of Circle - corresponds to an angle of 90 degrees  
           const SmExtent1d & crAnalDomain,          // in : Angular domain.  Must be between -360 and 360 inclusive   
           double             dRadius,               // in : radius                                                    
           ULONG              lDimension = 3,        // in : space dimension containing circle (2 or 3)                
           const SmContext  * cpContext = NULL,      // in : must be given for automatic variables, optional for       
                                                     //    : stack variables built with overloaded new.                
           const SmBSplineCurve * pOptNurb = NULL,   // in : Optional Nurb copied to make m_pNurb object<br>           
                                                     //    : If Given, when m_bInsideOut == FALSE                      
                                                     //    :       its StartPoint == CircleStart                       
                                                     //    :       its EndPoint   == CricleEnd                         
                                                     //    :       it must lie in the plane [crCenter, cross(X,Y)]     
                                                     //    :       its radius     == radius                            
                                                     //    : When m_bInsideOut == TRUE                                 
                                                     //    :       The endPoints must be swapped.                      
           SmBoolean bInsideOut = FALSE) ;           // in : TRUE = Nurb Tangent = -LineVector                         
                                                     //    : FALSE= Nurb Tangent =  LineVector                         

  // Construct full circle or arc from 3 points
  SmCircle(const SmPoint3d & cStartPoint,            // : Circular Arc Start Point                                                
           const SmPoint3d & cMidPoint,              // : Circular Arc Mid Point                                                  
           const SmPoint3d & cEndPoint,              // : Circular Arc End Point                                                  
           ULONG             lDimension = 3,         // : dimensions of points vectors [2 or 3]                                   
           SmBoolean         bClosedCircle = FALSE,  // : TRUE = Return rAnalDomain for closed Circle                             
                                                     // : FALSE= Return rAnalDomain for Circular Arc from StartPoint to EndPoint  
           const SmContext * cpContext = NULL) ;     // : For compatibility with Spheres and Tori                                 
                                                     // : must be given for automatic variables, optional for                     
                                                     // : stack variables built with overloaded new.                              

   // empty constructor for I/O
   SmCircle() { }

  // copy constructor
  SmCircle(const SmCircle & crCircle) ;

  // destructor
  virtual ~SmCircle() { }

  // equality operator
  virtual SmBoolean operator==(const SmCurve &crOther) const ;

  // create a step parameterized curve                          
  static SmStatus   CreateCanonical    (const SmContext        & crContext,                   // in : new object context                                                
                                        const SmAxis2Placement & crOrigin,                    // in : circle orientation (origin, xAxis, yAxis)                         
                                        double                   dRadius,                     // in : dist from origin to circle circumference                          
                                        SmCircle              *& rpNewCircle,                 // out: new object                                                        
                                        const SmExtent1d       * pInterval = NULL,            //    : opt: Specify interval in degrees [-360 <= min <= max <= 360.0]    
                                                                                              //    : NULL = [0.0 360.0]                                                
                                        SmBoolean              * pInsideOut = FALSE) ;        //    : opt: TRUE: for compatibility with sphere and torus,               
                                                                                              //    : Nurb curve runs in opposite direction from Analytic curve         
                                                                                              //    : FALSE: Normal case - nurb and analytic curves are the same shape. 
                                                                                              
  virtual SmStatus  Copy                (const SmContext  & crContext,                        // in :  
                                          SmCurve        *& rpNewCurve)                       // out:  
                                        const ;                                               
                                                                                              
  SmStatus          GetCanonical        (SmAxis2Placement & rOrigin,                          // out:          
                                         double           & rdRadius,                         // out:          
                                         SmBoolean        * pOptInsideOut = NULL)             // out: Optional 
                                        const ;                                               
                                                                                              
  double            GetRadius()         const { return m_dRadiusAtXAxis; }                    
                                                                                              
  static SmBoolean  IsNurbCurveCircle  (const SmContext      & crContext,                     // in :      
                                        const SmBSplineCurve * pTestCurve,                    // in :      
                                        SmCircle            *& rpCircle,                      // in :      
                                        double                 dToleranceScale = 1.0) ;       // in :      
                                                                                              
  virtual SmBoolean IsArc              (ULONG              lNumberOfSamplePoints,             // in : unused:  lNumberOfSamplePoints  
                                        double             dTol,                              // in : unused:  dTol                   
                                        SmAxis2Placement & rReferenceFrame,                   // out:                                 
                                        double           & rdRadius,                          // out:                                 
                                        double           & rdStartAngDeg,                     // out:                                 
                                        double           & rdEndAngDeg)                       // out:                                 
                                       const ;                                                
                                       
  virtual SmStatus  CreateSimpleOffset (const SmContext  & crContext,                         // in :   
                                        double             dApproxTol3d,                      // in :   
                                        const SmVector3d & crOffsetPlaneNormal,               // in :   
                                        double             dOffsetDistance,                   // in :   
                                        SmBSplineCurve  *& rpNewBSplineCurve,                 // out:   
                                        double           & rdMaxGap3d,                        // in :   
                                        SmBoolean          bOptMatchParameterization = FALSE) // in :   
                                       const ;
                                       
  // Simple member setting             
  void              SetRadius          (double dNewRadius)  { m_dRadiusAtXAxis = m_dRadiusAtYAxis = dNewRadius; }

  virtual SmBoolean AssertValid        (SmAssertArray    * pAList=NULL,                       // in,out: Accumulating list of failed Asserts, NULL to ignore                                             
                                        SmAssertTestLevel  eTestLevel=SM_LEVEL_0,             // in : SM_LEVEL_0=fewest tests, SM_LEVEL_1, SM_LEVEL_2=all tests,                                         
                                                                                              //    : SM_LEVEL_GIVEN = run tests in order requested in pTestRequests                                     
                                                                                              //    : default:[SM_LEVEL_0]                                                                               
                                        SmAssertWalking    eWalkTree = SM_WALK,               // NotUsed: [in] : SM_WALK = run AssertValid on any topology graph descendants, SM_NO_WALK=don't, default:[SM_WALK]
                                        SmTArray<ULONG>  * pTestRequests = NULL)              // in : when eTestLevel == SM_LEVEL_GIVEN, run these tests in this order, default:[NULL]                   
                                       const ;
                                       
  // obsolete
  // virtual SmBoolean AssertHeal         (SmAssertReport & rAReport, SmAssertArray * pAList) ;
                                       
  virtual SmStatus  WriteToDB          (SmDatabaseIO & rDB,                                   // in : target output stream                               
                                        ULONG          lDBVersionNumber)                      // in : database version to get proper sequence of writes  
                                       const ;                                              
                                                                                            
  static  SmStatus  ReadFromDB         (SM_TYPE           lType,                              // NotUsed: in : Object type to be read                                              
                                        SmDatabaseIO    & rDB,                                // in : target output stream                                                         
                                        ULONG             lDim,                               // in : curve image space dim, 2 or 3                                                              
                                        const SmContext & crContext,                          // in : context for new object construction                                          
                                        SmCurve         *&rpNewCurve,                         // out: NULL on input = new object allocated in this routine built from stream data  
                                                                                              //    : NotNULL on input = pointer to an empty object to be filled by this routine   
                                        ULONG             lDBVersionNumber) ;                 // in : database version to get proper sequence of writes                            
                    

  // Define  GetType(), IsKindOf(), and virtual Dump() (Dump needs local implementation)
  SM_COMMON(SmCircle,SmEllipse,SmCircle_TYPE);

} ; // end class SmCircle

#endif // !__SMCIRCLE_H__

