// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*******************************************************************//**
* FILE NAME --- SmEllipse.h
* PURPOSE: Header file for Ellipse curve.
**********************************************************************/

#ifndef __SMELLIPSE_H__
#define __SMELLIPSE_H__

#ifndef __SMBSPLINECURVE_H__
#include <SmBSplineCurve.h>
#endif

#ifndef __SMVECTOR3D_H__
#include <SmVector3d.h>
#endif

#ifndef __SMAXIS2PLACEMENT_H__
#include <SmAxis2Placement.h>
#endif

// forward declares
class SmEllipse ;

/*******************************************************************//**
PURPOSE: This class converts back and forth between a NURBS and Polar
     parameterization of a circle.

NOTES: 
  1. SmPolarConversion only works on circular curves represented
     by degree 2 BSplines with triple knots on the ends and double
     knots for every interior natural knot.

  2. The SmPolarConversion class stores matching arrays of 
     natural knot values and angle values in degrees for the
     bounds of every curve span.

  3. Conversion within a span is based on the relation:
      (CAGD - Hoschek/Lasser, p.157, A K PETERS)
                         2                        2
                    (1-t)  + 2t(1-t)COS(theta) + t COS(2theta)
      COS(angle) = --------------------------------------------
                         2                        2
                    (1-t)  + 2t(1-t)COS(theta) + t
         t     = span's parameter, range[0 to 1]
         theta = 1/2 angle to span's endAngle
         angle = polar angle from span start to span Point for param t.

  4. This conversion is exact when the weights within the span are constant
     and approximate to about 1.0E-06 when the weights vary.
   
     note: `class SmPolarConversion' has no virtual functions

***********************************************************************/
class SM_EXPORT SmPolarConversion 
{
private:
  SmBoolean             m_bPolarConversionPossible; // Set true after tables are initialized for curve span endPoints.
                                                    // Always set to False for curves that don't meet the representation requirements 
  SmTArray<double>      m_vNaturalKnots;            // array of natural knot values
  SmTArray<double>      m_vAngles;                  // array of angles (degrees) for every natural knot value
  SmTArray<SmBoolean>   m_vExactConversion;         // TRUE  = exact conversion for span  (Span ControlPoint weights are constant)
                                                    // FALSE = approx conversion for span (Span ControlPoint weights are NOT constant)
  const SmEllipse      *m_cpCurve ;                 // The curve for which conversions are being executed
  SmBoolean             m_bIsBorrowed ;             // TRUE  = m_cpCurve is owned by another object
                                                    // FALSE = m_cpCurve is deleted when this object is deleted
public:

  // constructors and destructors
  SmPolarConversion() : m_bPolarConversionPossible(FALSE),
                        m_cpCurve(NULL),
                        m_bIsBorrowed(TRUE)
                        { }

  // copy constructor needs Context ref since it's not an object but builds one(m_cpCurve)
  SmPolarConversion(const SmContext & crContext, const SmPolarConversion & crSource);
    
  // destructor
  ~SmPolarConversion() ;

  // equality operator
  SmBoolean operator==(const SmPolarConversion &crOther) const;

  void CopyFrom(const SmPolarConversion & crSource);

  const SmEllipse *GetCurve()      const      { return m_cpCurve ; }
  ULONG            GetAngleCount() const      { return m_vAngles.GetSize() ; }
  double           GetFirstAngle() const      { return m_vAngles[0] ; }
  double           GetLastAngle()  const      { return m_vAngles.GetLast() ; }

  void SetContext(const SmContext * cpContext) { if (!m_bIsBorrowed && m_cpCurve)
                                                 ((SmCurve*)m_cpCurve)->SetContext(cpContext); }

  SmBoolean IsPolarConversionPossible() const { return m_bPolarConversionPossible; }

  void      SetPolarConversionPossible( SmBoolean bIsPossible ) { m_bPolarConversionPossible = bIsPossible; }

  SmBoolean IsCurrent
  (                                               
    SmAssertArray    * pAList=NULL,                   ///< [in,out]: Accumulating list of failed Asserts, NULL to ignore                           <br>
    SmAssertTestLevel  eTestLevel=SM_LEVEL_0,         ///< [in] : SM_LEVEL_0=fewest tests, SM_LEVEL_1, SM_LEVEL_2=all tests,                       <br>
                                                      ///<      : SM_LEVEL_GIVEN = run tests in order requested in pTestRequests                   <br>
                                                      ///<      : default:[SM_LEVEL_0]                                                             <br>
    SmAssertWalking    eWalkTree=SM_WALK,             ///< NotUsed: [in] : SM_WALK = run AssertValid on any topology graph descendants, SM_NO_WALK=don't    <br>
    SmTArray<ULONG>  * pTestRequests=NULL             ///< [in] : when eTestLevel == SM_LEVEL_GIVEN, run these tests in this order                 <br>
  ) const ;

  
  SmStatus  SetUpPolarConversion
  (
    SmEllipse              * cpQuadCircleCurve,       ///< [in] :                                                                                  <br>
    SmBoolean                bIsBorrowed,             ///< [in] :                                                                                  <br>
    const SmExtent1d       & crIntervalDeg,           ///< NotUsed: [in] : analytic interval for circle [-360, 360] MaxLength=360                           <br>
    const SmAxis2Placement & crPlacement              ///< NotUsed: [in] : circle's axis placement                                                          <br>
  ) ;            
                                                                                    
           
  // convert target Angle in Degree to NURBS parameter
  SmStatus ConvertToNURBSParameter
  (
    double                 dPolarParameterDeg,       ///< [in] : angle in degrees to convert                                       <br>
    double               & rdNurbsParameter,         ///< [out]: curve parameter value                                             <br>
    SmBoolean            & bExactConversion,         ///< [out]: TRUE  = conversion was exact                                      <br>
                                                     ///<      : FALSE = conversion was approximate to about SM_ZONE_TOL_3D/10.0   <br>
    SmZoneTol3d            dZoneTol3d = 0.0          ///< [in] : Tolerance for snapping near max param to min. Default=0.0         <br>
  ) const;

  // convert target NURBs parameter to Polar Degrees
  SmStatus ConvertToPolarParameter
  (
    double                 dNurbsParameter,          ///< [in] : target nurbs parameter                                            <br>
    double               & rdPolarParameterDeg,      ///< [out]: equivalent polar parameter in degrees                             <br>
    SmBoolean            & bExactConversion          ///< [out]: TRUE  = conversion was exact                                      <br>
                                                     ///<      : FALSE = conversion was approximate to about SM_ZONE_TOL_3D/10.0   <br>
  ) const;

  SmStatus WriteToDB
  (          
    SmDatabaseIO & rDB,                              ///< [in] : target output stream                                               <br>
    ULONG          lDBVersionNumber                  ///< [in] : database version to get proper sequence of writes                  <br>
  ) const ;                                                                       

  static  SmStatus ReadFromDB
  (
    SmDatabaseIO       & rDB,                        ///< [in] : target output stream                                                                   <br>
    const SmContext    & crContext,                  ///< [in] : context for new object construction                                                    <br>
    const SmEllipse    * cpCurve,                    ///< [in] : curve to contain this PolarConverter saved as the back ptr m_cpCurve                   <br>
    SmPolarConversion *&rpNewPolarConversion,        ///< [out]: NULL on input = new object allocated in this routine built from stream data            <br>
                                                     ///<      : NotNull on input = assumed empty object already allocated filled here from stream data <br>
    ULONG               lDBVersionNumber             ///< [in] : database version to get proper sequence of writes                                      <br>
  ) ;   

  SmBoolean AssertValid
  (
    SmAssertArray    * pAList=NULL,                  ///< [in,out]: Accumulating list of failed Asserts, NULL to ignore                                 <br>
    SmAssertTestLevel  eTestLevel=SM_LEVEL_0,        ///< [in] : SM_LEVEL_0=fewest tests, SM_LEVEL_1, SM_LEVEL_2=all tests,                             <br>
                                                     ///<      : SM_LEVEL_GIVEN = run tests in order requested in pTestRequests                         <br>
    SmAssertWalking    eWalkTree=SM_WALK,            ///< NotUsed: [in] : SM_WALK = run AssertValid on any topology graph descendants, SM_NO_WALK=don't          <br>
    SmTArray<ULONG>  * pTestRequests=NULL            ///< [in] : when eTestLevel == SM_LEVEL_GIVEN, run these tests in this order                       <br>
  ) const ;

  void  Dump() const ;

  SM_TYPE       GetType()            const { return(SmPolarConversion_TYPE) ; }
  const TCHAR  *GetTypeString()      const { return(_T("SmPolarConversion_TYPE")) ; }
  const TCHAR  *GetClassString()     const { return(_T("SmPolarConversion")) ; }
  SM_TYPE       GetClassType()       const { return(SmPolarConversion_TYPE) ; }
  const TCHAR  *GetClassTypeString() const { return(_T("SmPolarConversion_TYPE")) ; }

private:
  SmStatus AngleParamToNurbParam
  (
    double dStartSpanNurbParam,         ///< [in] : Span Start parameter - not curve StartParam               <br>
    double dEndSpanNurbParam,           ///< [in] : Span End parameter   - not curve EndParam                 <br>
    double dStartSpanAngleDeg,          ///< [in] : Span Start angle [degrees] - not curve StartAngle         <br>
    double dEndSpanAngleDeg,            ///< [in] : Span End angle [degrees]   - not curve EndAngle           <br>
    double dAngleParamDeg,              ///< [in] : angular parameter [degrees]                               <br>
    double & rdNurbParam                ///< [out]: Nurb param in given Nurb interval                         <br>
  ) const ;

  SmStatus NurbParamToAngleParam
  (
    double dStartSpanNurbParam,          ///< [in] : Span Start parameter - not curve StartParam               <br>
    double dEndSpanNurbParam,            ///< [in] : Span End parameter   - not curve EndParam                 <br>
    double dStartSpanAngleDeg,           ///< [in] : Span Start angle [degrees] - not curve StartAngle         <br>
    double dEndSpanAngleDeg,             ///< [in] : Span End angle [degrees]   - not curve EndAngle           <br>
    double dNurbParam,                   ///< [in] : input quadratic parameter                                 <br>
    double & rdAngleParamDeg             ///< [out]: output angular parameter in degrees                       <br>
  ) const ;
    
} ; // end class SmPolarConversion

/*******************************************************************//**
PURPOSE: The SmEllipse class defines an ellipse.

NOTES:
 
  Ellipse(s) =   Origin                   
               + MajRad * XAxis * cos(s) 
               + MinRad * YAxis * sin(s)  for all s in m_vAnalDomain

         where m_vAnalDomain range:[-360 <= Min <= Max <= 360], Max-Min <= 360.0
               s = ellipse angle in radians (or the STEP parameterization).
                   When ellipse is a circle (MajRad == MinRad) 
                     the ellipse angle is equal to the CCW angle from the
                     the ellipse xAxis to a point on the ellipse,
                   Otherwise the ellipse angle does not equal the CCW angle.
                     It is just the value of (s) in the above relationship that
                     gives a point on the ellipse.

         note: An ellipse has two confusing domains, angular arc and
               NURB parameter, SMLib works in NURB parameters. Translate
               between the two with  ConvertTFromSTEPToNURBS and
                                     ConvertTFromNURBSToSTEP.

         note: An Ellipse always Starts at its XAxis extrema point
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
  | EvaluateSTEP()            | Ellipse Angle in degrees | Euclidean Point                      |
  | STEPInversion()           | Euclidean Point          | Ellipse Angle in degrees             |
  | DropPoint()               | Euclidean Point          | Nearest Ellipse Point NURB parameter |
  | ConvertTFromSTEPToNURBS() | Ellipse Angle in degrees | NURB parameter                       |
  | ConvertTFromNURBSToSTEP() | NURB parameter           | Ellipse Angle in degrees             |
  +---------------------------+--------------------------+--------------------------------------+
***********************************************************************/
class SM_EXPORT SmEllipse : public SmBSplineCurve
{
protected:
  // inherited
  // SmBSplineCurve::m_pNurb ;             // ptr to bspline data of type gw_CURVE
  // SmBSplineCurve::m_eBSplineCurveForm;  // Set by Derived Class Objects to 
                                           // oneof: SM_CF_CIRCULAR_ARC,  
                                           //        SM_CF_ELLIPTIC_ARC  as appropriate  

  SmAxis2Placement    m_vPosition;       // origin, X_Axis, and Y_Axis
  SmExtent1d          m_vAnalDomain;     // angular arc domain in degrees,
                                         //   range:[-360 <= Min <= Max <= 360], MaxLength=360.0
  double              m_dRadiusAtXAxis;  // X_Axis radius
  double              m_dRadiusAtYAxis;  // Y_Axis radius
  SmPolarConversion   m_vPolarConverter; // A cached table used to convert polar  
                                         // coordinates to/from the NURBS parameters.
                                         // Like all caches - kept up to date with the Notify mechanism.
  SmBoolean           m_bInsideOut;      // Included for compatibility with Spheres and Tori
                                         //   When not used as a Sphere/Tori GenCurve, set to FALSE.
                                         // TRUE = Nurb Tangent = -Step Tangent
                                         // FALSE= Nurb Tangent =  Step Tangent

  
  // constructors and destructors
  SmEllipse(ULONG             lDimension = 3,
            const SmContext * cpContext  = NULL) ;

public:
  // constructor
  SmEllipse(const SmPoint3d      & crCenter,           // in : circle origin                                      
            const SmVector3d     & crXAxis,            // in : Vector to Ellipse Arc Start Point                  
            const SmVector3d     & crYAxis,            // in : Vector to Ellipse Arc 90 degree point              
            const SmExtent1d     & crAnalDomain,       // in : in degrees, [-360 +360] where lengthMax = 360.0    
            double                 dRadiusAtXAxis,     // in : Radius of Ellipse Arc Start Point                  
            double                 dRadiusAtYAxis,     // in : Radius of Ellipse Arc 90 degree Point              
            ULONG                  lDimension=3,       // in : 2 or 3                                             
            const SmContext      * cpContext=NULL,     // in : must be given for automatic variables, optional for
                                                       //    : stack variables built with overloaded new.         
            const SmBSplineCurve * pOptNurb=NULL,      // in : Optional Nurb copied to make m_pNurb object        
            SmBoolean              bInsideOut=FALSE) ; // in : For compatibility with Spheres and Tori            
                                                       //    : TRUE = Nurb Tangent = -LineVector                  
                                                       //    : FALSE= Nurb Tangent =  LineVector                  

  // copy constructor
  SmEllipse(const SmEllipse & crEllipse);

  // destructor
  virtual ~SmEllipse() {}

  // equality operator
  virtual SmBoolean operator==(const SmCurve &crOther) const;

  // Select a different section of the curve
  virtual SmStatus AdjustSTEPInterval(const SmExtent1d & crNewSTEPInterval);

  virtual SmStatus ConvertTo2D();
  virtual SmStatus ConvertTo3D();

  virtual SmStatus ConvertTFromSTEPToNURBS(double dSTEPParam,             // in : angle in degrees         
                                           double & rdNURBSParam)         // out: parameter in Nurb space  
                                          const;                                
                                                                                
  virtual SmStatus ConvertTFromNURBSToSTEP(double dNURBSParam,            // in : parameter in Nurb space  
                                           double & rdSTEPParam)          // out: angle in degrees         
                                          const;

  virtual SmStatus Copy                   (const SmContext & crContext,   // in :     
                                           SmCurve        *& rpNewCurve)  // out:     
                                          const ;

  // create a step parameterized curve                          
  static SmStatus CreateCanonical(const SmContext        & crContext,            // in : new object context                                                            
                                  const SmAxis2Placement & crOrigin,             // in : circle orientation (origin, xAxis, yAxis)                                     
                                  double                  dRadius1,              // in : dist from origin to ellipse circumference in XAxis direction                  
                                  double                  dRadius2,              // in : dist from origin to ellipse circumference in YAxis direction                  
                                  SmEllipse            *& rpNewEllipse,          // out: new object                                                                    
                                  const SmExtent1d      * pInterval = NULL,      // in : Specify interval in degrees [-360 <= min <= max <= 360.0], NULL = [0.0 360.0] 
                                  SmBoolean             * pInsideOut = FALSE) ;  // in : TRUE: for compatibility with sphere and torus,                                
                                                                                 //    :       Nurb curve runs in opposite direction from Analytic curve               
                                                                                 //    : FALSE: Normal case - nurb and analytic curves are the same shape.             

  // try to find a fast way to drop points, (drop vector is either perp to curve or to nearest curve EndPoint)
  // rtn: SM_SUCCESS=case supported, SM_ERR=call GeneralPointSolve() to get solution
  virtual SmStatus DropPointFast(const SmExtent1d      & crInterval,              // in : Nurb Domain of curve to search for solutions                                         
                                 SmSolverOperationType   eSolverOperation,        // in : oneof: SM_SO_MINIMIZE =find closest point (more than one for closed curves)          
                                                                                  //    :    to curve or if cpdOptTargetDistance is given                                      
                                                                                  //    :    point within pdOptTargetDistance + dDistanceTolerance.                            
                                                                                  //    :    SM_SO_INTERSECT=find closest point within dDistanceTolerance.                     
                                 const SmPoint3d       & crTestPoint,             // in : target point                                                                         
                                 const SmVector3d      * cpOptInPointingVector,   // in : specifies end (EndTang = cpOptInPointingVector) saved for drops on closed curve seams
                                 double                  dDistanceTolerance,      // in : Skip Solutions whose drop distance is too far away                                   
                                                                                  //    : operation == MINIMIZE save solution if cpdOptTargetDistance == NULL                  
                                                                                  //    :                       or DropDist < cpdOptTargetDistance + dDistanceTolerance        
                                                                                  //    : operation == INTERSECT save solution if DropDist < dDistanceTolerance                
                                 const double          * cpdOptTargetDistance,    // in : only used for operation Minimize.  When given                                        
                                                                                  //    : skip solutions whose dropDist > cpdOptTargetDistance + dDistanceTolerance.           
                                                                                  //    : else keep all solutions.                                                             
                                 SmSolutionRequestedType eSolutionRequested,      // in : SM_SR_SINGLE=get best solution, SM_SR_ALL=get all solutions                          
                                 SmSolutionArray       & rSolutions)              // out: array of problem solutions reported as Curve parameter values                        
                                const ;      

  virtual SmExtent1d       GetSTEPInterval()      const { return m_vAnalDomain; }
  virtual SmExtent1d       GetMaxAnalyticDomain() const;
  virtual ULONG            GetDegree()            const { return 2; }
  double                   GetStartAngleDeg()     const { return m_vAnalDomain.GetMin(); }
  double                   GetEndAngleDeg()       const { return m_vAnalDomain.GetMax(); }
  const SmAxis2Placement & GetPosition()          const { return m_vPosition ; }
  double                   GetXRadius()           const { return m_dRadiusAtXAxis; }
  double                   GetYRadius()           const { return m_dRadiusAtYAxis; }
  virtual SmBoolean        GetInsideOut()         const { return m_bInsideOut ; }

  SmStatus                 GetCanonical(SmAxis2Placement & rOrigin,                       // out: ellipse center point                                                         
                                                                                          //    : XAxis = unitVector to ellipse  0 degree endPoint                             
                                                                                          //    : YAxis = unitVector to ellipse 90 degree Point                                
                                        double           & rdRadius1,                     // out: radius of  0 degree ellipse Point                                            
                                        double           & rdRadius2,                     // out: radius of 90 degree ellipse Point                                            
                                        SmExtent1d       * pOptAnalDomain = NULL,         // out: deg interval for ellipse                                                     
                                                                                          //    : NULL to ignore, default:[NULL]                                               
                                        SmBoolean        * pOptInsideOut = NULL)          // out: TRUE = Nurb and Analytic definitions run in opposite directions              
                                                                                          //    : FALSE= Nurb and Analytic definitions run in same directions NULL to ignore    
                                      const ;

  virtual SmStatus GlobalPointSolveSTEP(const SmExtent1d        & crInterval,             // NotUsed: [in] : STEP domain for solutions even when outside curve's domain  
                                        SmSolverOperationType     eSolverOperation,       // in :                                                                
                                        const SmPoint3d         & crPointToDrop,          // in :                                                                
                                        double                    dDistanceTolerance,     // in :                                                                
                                        const double            * cpdOptTargetDistance,   // in :                                                                
                                        const SmVector3d        * cpOptVectors,           // NotUsed: [in] :                                                     
                                        SmSolutionRequestedType   eSolutionRequested,     // in :                                                                
                                        SmSolutionArray         & rSolutions) ;           // out: Step Domain parameters                                         
                                                                                                                                                                 
  virtual SmStatus EvaluateSTEP        (double    dSTEPParameter,                         // in : ellipse angle in degrees (near to but not the CCW angle from X)                
                                        ULONG     lNumDerivatives,                        // in : 0 = Euclidian Point only                                                       
                                                                                          //    : 1 = Euclidian Point and 1st derivative                                         
                                        SmBoolean bFromLeft,                              // NotUsed: [in] : if P is on interval boundary                                        
                                                                                          //    : TRUE  = evaluate P in upper interval where P is on the left of the interval    
                                                                                          //    : FALSE = evaluate P in lower interval where P is on the right of the interval   
                                        SmVector3d aPointAndDerivatives[],                // out: values                                                                         
                                        SmZoneTol3d dZoneTol3d = 0.0)                     // in : Tolerance for error checking snap when Param is near STEP domain Max           
                                       const ;

  virtual SmStatus EvaluateSTEPPoint  (double        dSTEPParameter,                      // in : ellipse angle in degrees (near to but not the CCW angle from X)            
                                       SmPoint3d   & rPoint,                              // out: Euclidian point                                                            
                                       SmZoneTol3d   dZoneTol3d = 0.0)                    // in : Tolerance for error checking snap when Param ~ Max                         
                                      const ;                                             

  virtual SmBoolean IsAnalytic  () const { return TRUE; }
  SmBoolean         IsCircle    () const ;
  //virtual SmBoolean IsArc(ULONG, double, SmAxis2Placement&, double&, double&, double&) const;
  virtual SmBoolean IsDegenerate(double             d3DTolerance=SM_EFF_ZERO, // in : 
                                 const SmExtent1d * pInterval=NULL)    // NotUsed: in : 
                                const ;

//    virtual SmBoolean IsDegenerate(double d3DTolerance=SM_EFF_ZERO, SmExtent1d *pInterval=NULL) const;
  virtual SmSurfParamType IsDomainBoundary(const SmExtent2d &rDomain, double dTol=SM_EFF_ZERO) const
                                          { SM_REF2(rDomain, dTol) ; return SM_SP_NEITHER; }

  virtual SmStatus IntersectWithEllipse   (const SmExtent1d & crInterval,          // in : line interval                                         
                                           const SmEllipse  & crOtherCurve,        // in : other curve to intersect                              
                                           const SmExtent1d & crOtherInterval,     // in : other curve interval                                  
                                           double dDistanceTolerance,              // in : Find points where curves are within this 3D distance  
                                           SmBoolean & rbNeedsMoreIntersections,   // out: TRUE = pass call to general curve/curve intersector   
                                                                                   //    : FALSE= intersections found here                       
                                           SmSolutionArray & rSolutions)           // out: solutions                                             
                                          const ;

  virtual SmStatus IntersectWithLine      (const SmExtent1d & crInterval,          // in : ellipse interval in Nurb Domain                          
                                           const SmLine & crOtherCurve,            // in : line to intersect                                         
                                           const SmExtent1d & crOtherInterval,     // in : other line interval in Nurb Domain                        
                                           double dDistanceTolerance,              // in : Find points where curves are within this 3D distance      
                                           SmBoolean & rbNeedsMoreIntersections,   // out: TRUE = pass call to general curve/curve intersector       
                                                                                   //    : FALSE= intersections found here                           
                                           SmSolutionArray & rSolutions)           // out: solutions:  sSol.m_vStart[0] = ellipse param              
                                         const ;                                   //    :             sSol.m_vStart[1] = line param                 

  virtual SmStatus JoinWith              (ULONG lJoinEndThis,                      // in : 0 = Join at thisCurve start                                   
                                                                                   //    : 1 = Join at thisCurve end                                     
                                          SmBSplineCurve *pOtherCurve,             // in :                                                               
                                          ULONG lJoinEndOther,                     // out: 0 = Join at OtherCurve start                                  
                                                                                   //    : 1 = Join at OtherCurve end                                    
                                          double* pGapTolerance = NULL) ;          // in :  Optional tolerance reprsenting max gap between endpoints     
                                                                                   //    : If not specified then use default tolerance based on length   
  
  virtual SmStatus MakeNurb();

  // give ellipse a chance to update its cached m_vPolarConverter
  virtual void     Notify(SmNotifyOperation eNotifyOperation, // expected calls: caller->Notify(Event, pData1, pData2, pData2)                                   
                          SmObject * pData1,                  //       event                | caller      |  pData1  | pData2                | pData3                   
                          SmObject * pData2,                  //----------------------------+-------------+----------+-----------------------+--------------------------
                          SmObject * pData3) ;                // SM_NO_ADD_TO_BREP          | Brep        | AddObj   | Brep                  | AddObj's GeomPtr or NULL 
                                                              // SM_NO_SPLIT_IN_BREP        | Brep/TopoObj| OrigObj  | Child1                | Child2                   
                                                              // SM_NO_MERGE_IN_BREP        | Brep/TopoObj| SurvObj  | DelObj                | Brep                     
                                                              // SM_NO_TRIM_NO_SPLIT_IN_BREP| Brep        | TgtObj   | AddedBndryObj         | NULL                     
                                                              // SM_NO_COINCIDENT           | BrepA       | BrepAObj | BrepBObj              | BrepB
                                                              // SM_NO_RM_FROM_BREP         | Brep        | RmObj    | Brep                  | RmObj's GeomPtr or NULL  
                                                              // SM_NO_CHANGE_GEOMETRY      | TopoObj     | NewGeom  | Brep or NULL          | OldGeom or NULL          
                                                              // SM_NO_CHANGE_OWNER         | GeomObj     | NewOwner | NewOwner Brep or NULL | OldOwner or NULL         
                                                              // SM_NO_CONSTRUCTION         | NewObj      |  NewObj  | CopyFromObj or NULL   | NULL                     
                                                              // SM_NO_COPY                 | FromObj     | ToObj    | ToObj's Owner or NULL | FromObj's Owner or NULL  
                                                              // SM_NO_PRE_EDIT             | EditObj     | EditObj  | EditObj Owner or NULL | NULL                     
                                                              // SM_NO_POST_EDIT            | EditObj     | EditObj  | EditObj Owner or NULL | NULL                     
                                                              // SM_NO_SPLIT                | SplitObj    | Child1   | Child2                | SplitObj's Owner or NULL 
                                                              // SM_NO_MERGE                | MergeObj    | OrigObj1 | OrigObj2              | MergeObj's Owner or NULL 
                                                              // SM_NO_REG_PROPAGATION      | MergeReg    | ThisRegs | OtherBrep->SrcRegs    | ThisBrep->MergeReg
                                                              // SM_NO_DESTRUCTION          | DelObj      | DelObj   |  NULL                 |  NULL                    
                         
  virtual SmStatus ReverseParameterization(const SmExtent1d & crOldInterval,            // in : current curve interval         
                                           SmExtent1d       & rNewInterval) ;           // out: curve interval after reversal  

  virtual SmStatus STEPInversion          (const SmPoint3d      & crPointOnCurve,       // in : targetPoint - must be on the ellipse to within SM_EFF_ZERO  
                                           double               & rdAnalyticParameter,  // out: angular parameter in degrees,                               
                                                                                        //    : range:m_vAnalDomain or positive                             
                                           SmCurveLocationType  * pOptLoc = NULL,       // out: Point's classification to curve                             
                                                                                        //    : NULL to ignore, default:[NULL]                              
                                           SmZoneTol3d            dZoneTol3d = 0.0)     // in : Tolerance for returning max param as min on closed curve    
                                          const ;  

  // Simple member setting
  void SetXRadius   (double dNewRadius)      { m_dRadiusAtXAxis = dNewRadius; }
  void SetYRadius   (double dNewRadius)      { m_dRadiusAtYAxis = dNewRadius; }

  // This one can render the curve inconsistent.
  // Use with caution: intended for very temporary changes.
  // Use AdjustSTEPInterval() to make things consistent.
  void SetSTEPInterval( SmExtent1d & rNewDomain ) { m_vAnalDomain = rNewDomain; }

  virtual SmStatus Transform(const SmAxis2Placement & crRotateNMove,     // in : affine rotate and move transformation      
                             const SmVector3d       * cpOptScale=NULL) ; // in : optional scaling about current origin point before RotateNMove
                                                                         //      BSplines, planes, lines, PolyBreps - support nonisotropic scaling
                                                                         //      other geom types only support isoptropic scaling
                            
  // rotate point about Z axis until is is on the Z, X Plane
  SmStatus TransformPointToStartPlane(const SmPoint3d & crPointToTransform,      // in : Target Point                                                                          
                                      double            dDistTol3d,              // in : Dist3d when points are close enough to seams to return 2 answers                      
                                      SmPoint3d       & rTransformedPoint,       // out: Point rotated about rotation axis to start plane                                      
                                      ULONG           & rlNumAngles,             // out: 0 - point is on axis                                                                  
                                                                                 //    : 1 - point is not on seam                                                              
                                                                                 //    : 2 - point is on seam of surface of revolution                                         
                                      double            adAnglesDeg[2],          // out: Angles in degrees to rotate rTransformedPoint back to original position               
                                                                                 //    : range:[m_vAnalDomain] or positive(0 to 360.0)                                         
                                      SmBoolean       & bInside,                 // out: TRUE = point is in trim interval, FALSE=isn't                                         
                                      SmBoolean         bSnapToSeams=FALSE)      // in : TRUE = rtn 2 snapped values at seams, FALSE = rtn 1 exact and 1 snapped val at seams  
                                     const ;                                     //    : TRUE = previous behavior,default:[FALSE]                                              

  virtual SmStatus  Trim             (SmExtent1d & crTrimInterval,               // in : desired new Trim Ivl - this virtual method does not snap crTrimIvl
                                      SmBoolean    bNotify=TRUE,                 // in : internal use: use default value      
                                      SmBoolean    bSkipDebugCheck=FALSE) ;      // NotUsed: [in] : internal use: use default value      

  // get Curve Memory size - plus its attributes
  virtual ULONG     GetMemoryUsed    (ULONG    & rlMemoryAllocated,              // out: bigger size of all allocated memory in bytes   
                                      SmMarkType eMarkType=SM_MT_NOMARK)         // in : uses without increment eMarkType value         
                                     const ;                                     
                                                                                 
  virtual SmBoolean AssertValid      (SmAssertArray    * pAList=NULL,            // in,out: Accumulating list of failed Asserts, NULL to ignore                                            
                                      SmAssertTestLevel  eTestLevel=SM_LEVEL_0,  // in : SM_LEVEL_0=fewest tests, SM_LEVEL_1, SM_LEVEL_2=all tests,                                        
                                                                                 //    : SM_LEVEL_GIVEN = run tests in order requested in pTestRequests                                    
                                      SmAssertWalking    eWalkTree=SM_WALK,      // NotUsed: [in] : SM_WALK = run AssertValid on any topology graph descendants, SM_NO_WALK=don't, default:[SM_WALK] 
                                      SmTArray<ULONG>  * pTestRequests=NULL)     // in : when eTestLevel == SM_LEVEL_GIVEN, run these tests in this order, default:[NULL]                
                                     const ;

  // obsolete
  // virtual SmBoolean AssertHeal (SmAssertReport & rAReport, SmAssertArray * pAList) ;

  virtual SmStatus  WriteToDB       (SmDatabaseIO & rDB,                         // in : target output stream                                  
                                     ULONG          lDBVersionNumber)            // in : database version to get proper sequence of writes     
                                    const ;   
                           
  static  SmStatus  ReadFromDB      (SM_TYPE           lType,                    // NotUsed: [in] : Object type to be read
                                     SmDatabaseIO    & rDB,                      // in : target output stream                                                             <br>
                                     ULONG             lDim,                     // in : curve image space dim, 2 or 3                                                    <br>                                
                                     const SmContext & crContext,                // in : context for new object construction                                              <br>
                                     SmCurve         *&rpNewCurve,               // out: NULL on input = new object allocated in this routine built from stream data   <br>
                                                                                 //      NotNULL on input = pointer to an empty object to be filled by this routine           <br>
                                     ULONG             lDBVersionNumber) ;       // in : database version to get proper sequence of writes                                <br>
                             
  // Define  GetType(), IsKindOf(), and virtual Dump() (Dump needs local implementation)
  SM_COMMON(SmEllipse,SmBSplineCurve,SmEllipse_TYPE);

} ; // end class SmEllipse

#endif // !__SMELLIPSE_H__

