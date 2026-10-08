// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*******************************************************************//**
* FILE NAME --- SmLine.h
* PURPOSE: Header file for cubic Hermite curve.
**********************************************************************/

#ifndef __SMLINE_H__
#define __SMLINE_H__

#ifndef __SMBSPLINECURVE_H__
#include <SmBSplineCurve.h>
#endif

#ifndef __SMVECTOR3D_H__
#include <SmVector3d.h>
#endif

#ifndef __SMEXTENT3D_H__
#include <SmExtent3d.h>
#endif

/*******************************************************************//**
PURPOSE: The SmLine class defines an infinite line.

NOTES: 
  Line(s) = m_vLinePoint + s * m_vLineVector * m_dScale  for all STEP params, s, in m_vAnalDomain

    m_vLineVector = unit direction vector
    m_dScale      = Curve Speed

  Translating between STEP and NURB parameters is done
  When Line is bounded - Nurb and Step Min and Max Points are set equal to one another
    When m_bInsideout == FALSE:    (Nurb_S - Nurb_SMin)      (Step_S - Step_SMin)
                                 ----------------------- = -----------------------
                                 (Nurb_SMax - Nurb_SMin)   (Step_SMax - Step_SMin)


    When m_bInsideout == TRUE :    (Nurb_S - Nurb_SMax)      (Step_S - Step_SMin)
                                 ----------------------- = -----------------------
                                 (Nurb_SMin - Nurb_SMax)   (Step_SMax - Step_SMin)
  When Line is infinite
    Nurb and Step param = 0 points are set equal
    When m_bInsideout == FALSE:  Nurb_S =  Step_S
    When m_bInsideout == TRUE :  Nurb_S = -Step_S

***********************************************************************/
class SM_EXPORT SmLine : public SmBSplineCurve
{
private:
  SmPoint3d  m_vLinePoint ;               // Start point of line
  SmVector3d m_vLineVector ;              // Vector of line - unitized
  SmExtent1d m_vAnalDomain ;              // STEP Interval for bounded line
                                          //   for Lines: AnalDomain = NaturalDomain 
                                          //   unbounded line interval:[-SM_INFINITE_PARAMETER, SM_INFINITE_PARAMETER]                    
  double     m_dScale = SM_UNDEF_DOUBLE ; // Scale to be applied
  SmBoolean  m_bInsideOut = FALSE ;       // Included for compatibility with Cones.
                                          //     When not used as a cone GenCurve, set to FALSE.
                                          //   TRUE = Nurb Tangent = -LineVector
                                          //   FALSE= Nurb Tangent =  LineVector
             
public:
  // constructor - create bounded 2 - point line, analytic domain = [0.0 1.0]
  SmLine                           (const SmPoint3d      & crStartPoint, // in : Start of line = Start + s*(End-Start), for s:[0 1]
                                    const SmPoint3d      & crEndPoint,   // in : End   of line = Start + s*(End-Start), for s:[0 1]
                                                                         //    : will be unitized before storing                   
                                    ULONG                  lDimension,   // in : sizeof LinePoint and LineVector, default:[3]      
                                    const SmContext      * cpContext) ;  // in :                                                   
             
  // constructor - create bounded Line(s)  = m_vLinePoint + s * m_vLineVector * m_dScale             
  //                   stores  m_vLinePoint  = crLinePoint
  //                           m_vLineVector = crLineUnitVector.unitizied()
  //                           m_vAnalDomain = crAnalDomain
  //                           m_dScale      = dScale
  SmLine                           (const SmPoint3d      & crLinePoint,          // in : P     of line = P + s*scale*unitV                                        
                                    const SmVector3d     & crUnitVector,         // in : V     of line = P + s*scale*unitV                                        
                                                                                 //    : will be unitized before storing                                          
                                    const SmExtent1d     & crAnalDomain,         // in : limits on s, crInterval.Min <= s <= crInterval.Max                       
                                    double                 dScale = 1.0,         // in : scale of line = P + s*scale*unitV                                        
                                    ULONG                  lDimension = 3,       // in : sizeof LinePoint and LineVector, default:[3]                             
                                    const SmContext      * cpContext = NULL,     // in : required when making an automatic variable                               
                                                                                 //    : optionally when using overloaded new.                                    
                                    const SmBSplineCurve * pOptNurb = NULL,      // in : Optional Nurb copied to make m_pNurb object                              
                                                                                 //    : If Given, its StartPoint == bInsideOut ? StepEndPoint   : StepStartPoint 
                                                                                 //    :           its EndPoint   == bInsideOut ? StepStartPoint : StepEndPoint   
                                    SmBoolean              bInsideOut = FALSE) ; // in : TRUE = Nurb Tangent = -LineVector                                        
                                                                                 //    : FALSE= Nurb Tangent =  LineVector                                        
  // constructor - create bounded 2-point line             
  static SmStatus CreateLineSegment(const SmContext & crContext,             // in : context for new object construction   
                                    ULONG             lDimensionOfResult,    // in : 2 or 3                                
                                    const SmPoint3d & crStartPoint,          // in : LineStartPoint                        
                                    const SmPoint3d & crEndPoint,            // in : LineEndPoint                          
                                    SmLine         *& rpNewLine,             // out: NewLine                               
                                    SmExtent1d      * pOptInterval = NULL) ; // in : parameter range for the line


  // constructor - rarely used - 
  // create infinite Line(s) = m_vLinePoint + s * m_vLineVector * m_dScale
  //   stores  m_vLinePoint  = crLinePoint
  //           m_vLineVector = crLineNonUnitVector.Unitize()
  //           m_vAnalDomain = [-SM_INFINITE_PARAMETER,SM_INFINITE_PARAMETER]
  //           m_dScale      = crLineNonUnitVector.Length()
  SmLine                           (const SmPoint3d  & crLinePoint,          // in : P of line = P + s* NonUnitV                                         
                                    const SmVector3d & crLineNonUnitVector,  // in : V of line = P + s* NonUnitV                                         
                                    ULONG              lDimension = 3,       // in : default:[3]                                                         
                                    SmBoolean          bMakeNurb = TRUE,     // in : NotUsed: FALSE = this line just for evals, skip MakeNurb() for efficiency    
                                                                             //    : TRUE  = make Nurb with NaturalInterval = AnalDomain                 
                                    const SmContext  * cpContext = NULL,     // in : must be given for automatic variables, optional for                 
                                                                             //    : stack variables built with overloaded new.                          
                                    SmBoolean          bInsideOut = FALSE) ; // in : TRUE = Nurb Tangent = -LineVector                                   
                                                                             //    : FALSE= Nurb Tangent =  LineVector                                   
                                                                             
  SmLine                           (ULONG lDimension)   // in : Dimension of Line                                                 
                                   : SmBSplineCurve(lDimension)
                                   { } ;

  // empty constructor for I/O
  SmLine() { }

  // copy constructor
  SmLine(const SmLine & crLine) ;

  // destructor
  virtual ~SmLine() { }

  // equality operator
  virtual SmBoolean operator==                    (const SmCurve &crOther)  const ;
                                                 
  // Select a different section of the curve     
  virtual SmStatus AdjustSTEPInterval             (const SmExtent1d & crNewSTEPInterval) ;

  // return polygonal length between equally domain spaced SamplePoints
  virtual double ApproximateLength                (const SmExtent1d & crInterval,              // in : curve interval to query                                            
                                                   ULONG              lSampleCnt,              // NotUsed: in : number of smp pts including end points, 5 is a reasonable number   
                                                   double           * pOptUVTurnAngDeg = NULL) // out: 2d Curves only. Optional UV Space turning angle. 0.0 for 3dCurves  
                                                  const ;
                                                  
  virtual SmStatus ConvertTo2D();                 
  virtual SmStatus ConvertTo3D();                 
                                                  
  virtual SmStatus ConvertTFromSTEPToNURBS        (double dSTEPParam,     // in : Analytic Domain parameter    
                                                   double & rdNURBSParam) // out: Nurb Domain parameter        
                                                  const ;
                                                  
  virtual SmStatus ConvertTFromNURBSToSTEP        (double dNURBSParam,   // in : Nurb Domain parameter         
                                                   double & rdSTEPParam) // out: Analytic Domain parameter     
                                                  const ;
                                                  
  virtual SmStatus Copy                           (const SmContext & crContext,  // in :          
                                                   SmCurve        *& rpNewCurve) // out:          
                                                  const ;
                                            
  // create infinite line, Domain=[-SM_INFINITE_PARAMETER, SM_INFINITE_PARAMETER], Line(0) = LinePoint, Line(1)= LinePoint + LineVector                         
  static SmStatus CreateCanonical                 (const SmContext  & crContext,    // in : context for new object construction    
                                                   const SmPoint3d  & crLinePoint,  // in : Origin of Line                         
                                                   const SmVector3d & crUnitVector, // in : Unitized Vector                        
                                                   SmLine          *& rpNewLine) ;  // out: new object                             
                                                  
  virtual SmStatus CreateSimpleOffset             (const SmContext  & crContext,                         // in :                      
                                                   double             dApproxTol3d,                      // in : Not used here.       
                                                   const SmVector3d & crOffsetPlaneNormal,               // in :                      
                                                   double             dOffsetDistance,                   // in :                      
                                                   SmBSplineCurve  *& rpNewBSplineCurve,                 // out:                      
                                                   double           & rdMaxGap3d,                        // out: Always 0.0 here.     
                                                   SmBoolean          bOptMatchParameterization = FALSE) // in : opt: Not used here.  
                                                  const ;
                                                  
  // get bounding box for Domain interval         
  virtual SmStatus CalculateBoundingBox           (const SmExtent1d & crNurbInterval,     // in : Desired Nurb Domain Interval               
                                                   SmExtent3d       * pNormalBox = NULL,  // out: Axis alligned box                          
                                                   SmPseudoBox      * pPseudoBox = NULL,  // out: Non-axis aligned box                       
                                                   SmPolarBox       * pPolarBox = NULL,   // out: Surface normal vector field bounding box   
                                                   SmBoolean          bExpandBox = TRUE)  // in : bExpandBox = not-used                      
                                                  const ;                                 //    : FALSE = don't expand returned bounding box 
                                              
                                            
  // get precise bounding box for Domain interval
  virtual SmStatus CalculateTightBoundingBox      (const SmExtent1d & crNurbInterval,
                                                   SmExtent3d       * pNormalBox = NULL)
                                                  const
                                                  {
                                                    return CalculateBoundingBox( crNurbInterval, pNormalBox, NULL, NULL );
                                                  }

  // try to find a fast way to drop points, (drop vector is either perp to curve or to nearest curve EndPoint)
  // rtn: SM_SUCCESS=case supported, SM_ERR=call GeneralPointSolve() to get solution
  virtual SmStatus DropPointFast                 (const SmExtent1d      & crNurbInterval,        // in : Nurb Domain of curve to search for solutions                                          
                                                  SmSolverOperationType   eSolverOperation,      // in : oneof: SM_SO_MINIMIZE =find closest point (more than one for closed curves)           
                                                                                                 //    :        to curve or if cpdOptTargetDistance is given                                   
                                                                                                 //    :        point within pdOptTargetDistance + dDistanceTolerance.                         
                                                                                                 //    :        SM_SO_INTERSECT=find closest point within dDistanceTolerance.                  
                                                  const SmPoint3d       & crTestPoint,           // in : target point                                                                          
                                                  const SmVector3d      * cpOptInPointingVector, // in : specifies end (EndTang = cpOptInPointingVector) saved for drops on closed curve seams 
                                                  double                  dDistanceTolerance,    // in : Skip Solutions whose drop distance is too far away                                    
                                                                                                 //    : operation == MINIMIZE save solution if cpdOptTargetDistance == NULL                   
                                                                                                 //    :                       or DropDist < cpdOptTargetDistance + dDistanceTolerance         
                                                                                                 //    : operation == INTERSECT save solution if DropDist < dDistanceTolerance                 
                                                  const double          * cpdOptTargetDistance,  // in : only used for operation Minimize.  When given                                         
                                                                                                 //    : skip solutions whose dropDist > cpdOptTargetDistance + dDistanceTolerance.            
                                                                                                 //    : else keep all solutions.                                                              
                                                  SmSolutionRequestedType eSolutionRequested,    // in : SM_SR_SINGLE=get best solution, SM_SR_ALL=get all solutions                           
                                                  SmSolutionArray       & rSolutions)            // out: array of problem solutions reported as Curve parameter values                         
                                                 const ;
                                                 
  virtual SmStatus EvaluateCurvature             (double   dParameter,     // NotUsed: in :
                                                  double & rCurvature)     // out:
                                                 const ;

  virtual SmExtent1d  GetSTEPInterval            () const { return m_vAnalDomain; }
  virtual SmExtent1d  GetNaturalInterval         () const ;
  virtual SmExtent1d  GetMaxAnalyticDomain       () const { SmExtent1d sDom( -1000, 1000 ); return sDom; }
  virtual ULONG       GetDegree                  () const { return 1; }
  const SmPoint3d   & GetLinePoint               () const { return(m_vLinePoint); }
  const SmVector3d  & GetLineVector              () const { return(m_vLineVector); }
  double              GetLineScale               () const { return m_dScale; }
  virtual ULONG       GetNumberNaturalKnots      () const { return(4); }
  virtual SmBoolean   GetInsideOut               () const { return(m_bInsideOut); }
                                                 
  SmStatus            GetCanonical               (SmPoint3d  & rLinePoint,
                                                  SmVector3d & rUnitVector,
                                                  double     & rScale,
                                                  SmExtent1d & rAnalDomain,
                                                  SmBoolean  * pOptInsideOut = NULL)
                                                 const ;
                                                 
  SmStatus            GetCanonical               (SmPoint3d  & rLinePoint,            // out: Pt    of Line = Pt + u * Vec                                                      
                                                  SmVector3d & rNonUnitVector,        // out: Vec   of Line = Pt + u * Vec, Vec is not usually unit length                      
                                                  SmExtent1d * pOptAnalDomain = NULL, // out: Valid range of parameter values u,    NULL to ignore                              
                                                  SmBoolean  * pOptInsideOut = NULL)  // out: TRUE = Nurb and Analytic definitions run in opposite directions NULL to ignore    
                                                 const ;
                                                 
  SmStatus            SetCanonical               (SmPoint3d  & rLinePoint,    // in : Pt    of Line = Pt + u * Scale * Vec                              
                                                  SmVector3d & rUnitVector,   // in : Vec   of Line = Pt + u * Scale * Vec, Must be unit length         
                                                  double       dScale,        // in : Scale of Line = Pt + u * Scale * Vec, Must be greater than 0.0    
                                                  SmExtent1d & rAnalDomain) ; // in : Valid range of parameter values u,                                
                                                 
  void                SetLineScale               (double     dNewScale)  { m_dScale = dNewScale; }
                                                             
  void                SetAnalyticDomain          (SmExtent1d dNewDomain) { m_vAnalDomain = dNewDomain; }
                                                 
                                                 
                                                 
  virtual SmStatus   GlobalPointSolveSTEP        (const SmExtent1d        & crSTEPInterval,       // in : domain for solutions even when outside curve's domain    
                                                  SmSolverOperationType     eSolverOperation,     // in : oneof SM_SO_MINIMIZE                                     
                                                                                                  //    :       SM_SO_MAXIMIZE                                     
                                                                                                  //    :       SM_SO_NORMALIZE                                    
                                                                                                  //    :       SM_SO_INTERSECT                                    
                                                  const SmPoint3d         & crTestPoint,          // in : Target Point                                             
                                                  double                    dDistanceTolerance,   // in : Max allowed deviation for SM_SO_INTERSECT                
                                                  const double            * cpdOptTargetDistance, // in : cpdOptTargetDistance = Not Used                          
                                                  const SmVector3d        * cpOptVectors,         // in : cpOptVectors         = Not Used                          
                                                  SmSolutionRequestedType   eSolutionRequested,   // in : eSolutionRequested   = Not Used                          
                                                  SmSolutionArray         & rSolutions) ;         // out: Solution Param in StepDomain                             
                                                                                                  //    : Usually 1 solution                                       
                                                                                                  //    : Possibly 2 for SM_SO_MAXIMIZE and                        
                                                                                                  //    : point is equaldistant from both endPoints                 
                                                                                                   
  virtual SmStatus   Evaluate                    (double     dParameter,              // in : tgt param                                                                        
                                                  ULONG      lNumDerivatives,         // in : 0=pos, 1=pos+tang, 2=pos+tang+2nd, . . .                                         
                                                  SmBoolean  bFromLeft,               // NotUsed: in : if P is on interval boundary                                                     
                                                                                      //    : TRUE  = evaluate P in upper interval where P is on the left of the interval      
                                                                                      //    : FALSE = evaluate P in lower interval where P is on the right of the interval     
                                                  SmVector3d aPointAndDerivatives[],  // out: (pos, tang, 2nd, ...) sized:[lNumDerivatives+1]                                  
                                                  SmBoolean  bNonZeroTangents = TRUE) // in : TRUE = replace zero tangent vectors with properly oriented tol sized vectors     
                                                 const ;                              //    : FALSE= return exact tangent values                                               
                                                                                      //    : note: Surprisingly TRUE is the common choice because most tangent uses           
                                                                                      //    :       are for their direction (Binorm, SurfNorm comps), but when the             
                                                                                      //    :       tangent is being used for its magnitude (like an arc-length comp)          
                                                                                      //    :       then set this to FALSE.                                                    
                                                 
  virtual SmStatus   EvaluateSTEP                (double dSTEPParameter,             // in : target STEP parameter to evaluate                                                 
                                                  ULONG lNumDerivatives,             // in : 0 - produces Euclidian point only - see SmCurve::EvaluatePoint                    
                                                                                     //    : 1 - produces first derivative and point                                           
                                                                                     //    : 2 - produces second derivative, first derivative and point.                       
                                                                                     //    : N - produces N-th derivative and lower derivatives                                
                                                  SmBoolean bFromLeft,               // NotUsed: in : bFromLeft = if P is on interval boundary                                          
                                                                                     //    : TRUE  = evaluate P in upper interval where P is on the left of the interval       
                                                                                     //    : FALSE = evaluate P in lower interval where P is on the right of the interval      
                                                  SmVector3d aPointAndDerivatives[], // out: array of [position, 1st deriv, 2nd deriv, ...] values.                            
                                                                                     //    : expected size:[lNumDerivatives+1]                                                 
                                                  SmZoneTol3d dZoneTol3d = 0.0)      // NotUsed: in : Unused                                                                            
                                                 const ;                             
                                                 
  virtual SmStatus   EvaluatePoint               (double      dNurbParam,
                                                  SmPoint3d & rPoint)
                                                 const ;
                                                 
  virtual SmBoolean       IsAnalytic             ()                                              const { return TRUE; }
  virtual SmBoolean       IsBounded              ()                                              const ; // TRUE= finite lines - both ends are bound
  virtual SmBoolean       IsClosed               (const SmExtent1d & crIvl,
                                                  double             dTol = 0.0 )                const { SM_REF2(crIvl, dTol) ; return FALSE; }
  virtual SmBoolean       IsDegenerate           (double             d3DTolerance = SM_EFF_ZERO,
                                                  const SmExtent1d * pInterval = NULL) const ;
  virtual SmSurfParamType IsDomainBoundary       (const SmExtent2d & rDomain,
                                                  double             dTol = SM_EFF_ZERO)         const ;
  virtual SmBoolean       IsLine                 (ULONG        lNumberOfSamplePoints,                    // in : lNumberOfSamplePoints = not used in this function       
                                                  double       dTol,                                     // in : dTol = not used in this function                        
                                                  SmPoint3d  & rLinePoint,                               // out: Start Point of Line                                     
                                                  SmVector3d & rLineVector)                      const ; // out: LineVec = (EndPoint - StartPoint)                       
  virtual SmBoolean       IsArc(ULONG, double, SmAxis2Placement&, double&, double&, double&)     const { return FALSE; }                                                 
  virtual SmStatus        IntersectWithEllipse   (const SmExtent1d & crInterval,               // in : line interval in Nurb Domain                          
                                                  const SmEllipse  & crOtherCurve,             // in : other curve to intersect                              
                                                  const SmExtent1d & crOtherInterval,          // in : other curve interval                                  
                                                  double             dDistanceTolerance,       // in : Find points where curves are within this 3D distance  
                                                  SmBoolean        & rbNeedsMoreIntersections, // out: TRUE = pass call to general curve/curve intersector   
                                                                                               //    : FALSE= intersections found here                       
                                                  SmSolutionArray  & rSolutions)               // out: solutions                                             
                                                 const ;
                                                 
  virtual SmStatus        IntersectWithLine      (const SmExtent1d & crInterval,               // in : line interval in Nurb Domain                          
                                                  const SmLine     & crOtherCurve,             // in : other line to intersect                               
                                                  const SmExtent1d & crOtherInterval,          // in : other line interval in Nurb Domain                    
                                                  double             dDistanceTolerance,       // in : Find points where curves are within this 3D distance  
                                                  SmBoolean        & rbNeedsMoreIntersections, // out: TRUE = pass call to general curve/curve intersector   
                                                                                               //    : FALSE= intersections found here                       
                                                  SmSolutionArray  & rSolutions)               // out: solutions: sSol.m_vStart[0] = this param              
                                                 const ;                                       //    :            sSol.m_vStart[1] = other param             
                                                 
  virtual SmStatus        JoinWith               (ULONG            lJoinEndThis,           // in : 0 = Join at thisCurve start                                
                                                                                           //    : 1 = Join at thisCurve end                                  
                                                  SmBSplineCurve * pOtherCurve,            // in :                                                            
                                                  ULONG            lJoinEndOther,          // out: 0 = Join at OtherCurve start                               
                                                                                           //    : 1 = Join at OtherCurve end                                 
                                                  double*          pGapTolerance = NULL) ; // in : Optional tolerance reprsenting max gap between endpoints   
                                                                                           //    : If not specified then use default tolerance based on length
                                                 
  virtual SmStatus        Length                 (const SmExtent1d & crInterval,       // in : Nurb Domain of interest             
                                                  double             dDesiredAccuracy, // NotUsed: in : not used                            
                                                  double           & rdLength)         // out: length of line for given interval   
                                                 const ;
                                                 
  virtual SmStatus        MakeNurb               () ;
                                                 
  // refresh analytic data after editing the Nurb representation directly to keep the two reps in sync
  virtual SmStatus        RefreshAnalytics       () ;
                          
  virtual SmStatus        ReverseParameterization(const SmExtent1d & crOldInterval,           // in : current curve interval in Nurb Domain  
                                                  SmExtent1d       & rNewInterval) ;          // out: curve interval after reversal          
                                                                                              
  virtual SmStatus        STEPInversion          (const SmPoint3d      & crPointOnCurve,      // in : Target Point                             
                                                  double               & rdAnalyticParameter, // out: STEPParameter closest to Target Point    
                                                  SmCurveLocationType  * pOptLoc = NULL,      // out: Point's classification to curve          
                                                                                              //    : NULL to ignore, default:[NULL]           
                                                  SmZoneTol3d            dZoneTol3d = 0.0)    // NotUsed: in : Unused                                   
                                                 const ;
                                                 
  virtual SmStatus        Tessellate             (const SmExtent1d     & crInterval,            // in : line interval in Nurb domain                                          
                                                  double                 dChordHeightTolerance, // in : dChordHeightTolerance - NOT USED                                      
                                                  double                 dAngleTolDeg,          // in : dAngleTolDeg - NOT USED                                               
                                                  ULONG           lMinimumNumberOfSegments,     // in : Min number of segments in this tessellation                           
                                                  SmTArray<double>     * pOptParameters = NULL, // out: Opt Nurb Parameters for each tessellation point NULL to ignore        
                                                  SmTArray<SmPoint3d>  * pOptPoints = NULL,     // out: Every tessellation point NULL to ignore                               
                                                  SmTArray<SmVector3d> * pOptTangents = NULL)   // out: Optional array of tangent points for each sample point NULL to ignore 
                                                 const ;                                        
                                                 
  virtual SmStatus        Transform              (const SmAxis2Placement & crRotateNMove,         // in : Rotation and transformation         
                                                  const SmVector3d       * cpOptScale) ;          // in : Scale - any scaling is allowed      
                          
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
                                                                                                   
  virtual SmStatus        Trim                   (SmExtent1d & crTrimInterval,            // in : desired new Trim Ivl - this virtual method does not snap crTrimIvl                               
                                                  SmBoolean    bNotify = TRUE,            // in : internal use only - use default, default:[TRUE]                    
                                                                                          //    : TRUE  = call Notify after trimming (previous behavior)             
                                                                                          //    : FALSE = skip Notify after trimming                                 
                                                                                          //    : UNSURE= skip notify, skip trimming, just recompute TrimInterval    
                                                  SmBoolean    bSkipDebugCheck = FALSE) ; // NotUsed: in : internal use only - use default, default:[FALSE]                   
                                                                                          //    : FALSE= in debug mode silently run this->AssertValid()              
                                                                                          //    : TRUE = don't run AssertValid() before returning                    
                                                  
  // get Curve Memory size - plus its attributes
  virtual ULONG           GetMemoryUsed          (ULONG    & rlMemoryAllocated,           // out: bigger size of all allocated memory in bytes   
                                                  SmMarkType eMarkType = SM_MT_NOMARK)    // in : uses without increment eMarkType value         
                                                 const ;                               
                         
  virtual SmBoolean       AssertValid            (SmAssertArray    * pAList = NULL,           // in,out: Accumulating list of failed Asserts, NULL to ignore, default:[NULL]                           
                                                  SmAssertTestLevel  eTestLevel = SM_LEVEL_0, // in : SM_LEVEL_0=fewest tests, SM_LEVEL_1, SM_LEVEL_2=all tests,                                       
                                                                                              //    : SM_LEVEL_GIVEN = run tests in order requested in pTestRequests                                   
                                                  SmAssertWalking    eWalkTree = SM_WALK,     // NotUsed: in : SM_WALK = run AssertValid on any topology graph descendants, SM_NO_WALK=don't, default:[SM_WALK] 
                                                  SmTArray<ULONG>  * pTestRequests = NULL)    // in : when eTestLevel == SM_LEVEL_GIVEN, run these tests in this order, default:[NULL]                 
                                                 const ;
                                                 
  // obsolete
  // virtual SmBoolean AssertHeal (SmAssertReport & rAReport, SmAssertArray * pAList ) ;
                          
  virtual void            Dump                   (const TCHAR * message) const ;
  virtual void            Dump                   (ULONG)                 const ;
                          
  virtual SmStatus        WriteToDB              (SmDatabaseIO & rDB,              // in : target output stream                                   
                                                  ULONG          lDBVersionNumber) // in : database version to get proper sequence of writes      
                                                 const ;   
                                                 
  static  SmStatus        ReadFromDB             (SM_TYPE           lType,              // NotUsed: in : Object type to be read                                                          
                                                  SmDatabaseIO    & rDB,                // in : target output stream                                                            
                                                  ULONG             lDim,               // in : curve image space dim, 2 or 3                                                                                 
                                                  const SmContext & crContext,          // in : context for new object construction                                             
                                                  SmCurve         *&rpNewCurve,         // out: NULL on input = new object allocated in this routine built from stream data     
                                                                                        //    : NotNULL on input = pointer to an empty object to be filled by this routine      
                                                  ULONG             lDBVersionNumber) ; // in : database version to get proper sequence of writes                               
                                                 
  // Define  GetType(), IsKindOf(), and virtual Dump() (Dump needs local implementation)
  SM_COMMON( SmLine, SmBSplineCurve, SmLine_TYPE );

} ; // end class SmLine

#endif // !__SMLINE_H__



