// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*******************************************************************//**
* FILE NAME --- SmExtent1d.h
* PURPOSE: Header file for SmExtent1d class.
**********************************************************************/

#ifndef __SMEXTENT1D_H__
#define __SMEXTENT1D_H__

#ifndef __SMCORE_TYPES_H__
#include <SmCoreTypes.h>
#endif

#ifndef __SMOS_MATH_H__
#include <SmMath.h>
#endif

/*******************************************************************//**
PURPOSE: TENUM for point classification within an extent
***********************************************************************/
enum SmExtentPointType {
    SM_EP_START,         // Param is on min boundary of extent
    SM_EP_INSIDE,        // Param is in the interior of the extent
    SM_EP_END,           // Param is on max boundary of extent
    SM_EP_BOTH,          // Param is on both max and min boundaries when extent
                         //   is degenerate
    SM_EP_OUTSIDE,       // Param is outside natural intervals of extent
    SM_EP_UNKNOWN
};

/*******************************************************************//**
PURPOSE: This object represents an interval in one dimension.  It
    contains a minimum and maximum value.  The values may be equal.  

NOTES: This object is often used as the parametric domain
    of a curve.
***********************************************************************/
class SM_EXPORT SmExtent1d
{
protected:
  double m_dMin =  SM_BIG_DOUBLE ;  // rule: m_dMin <= m_dMax     
  double m_dMax = -SM_BIG_DOUBLE ;

public:
  // constructors, destructor
  SmExtent1d() { Init(); }
  SmExtent1d(double dMin, double dMax);
  SmExtent1d(double dMinAndMax) { m_dMin = dMinAndMax; m_dMax = dMinAndMax; }
  SmExtent1d(const SmExtent1d & crOriginal);
  SmExtent1d& operator=( SmExtent1d const &obj )
  {
    if(&obj == this) return *this;
    m_dMin = obj.m_dMin;
    m_dMax = obj.m_dMax;
    return *this;
  }

 ~SmExtent1d() { m_dMin =  SM_UNDEF_DOUBLE; 
                 m_dMax = -SM_UNDEF_DOUBLE; 
               }                                                                                                                              

  // modifiers
  void        Init             () { m_dMin = SM_BIG_DOUBLE; m_dMax = -SM_BIG_DOUBLE; }
  void        AddValue         (double dValue);
  SmStatus    SetMinMax        (double dMin, double dMax);  // rtn: SmErr when dMax < dMin
  SmStatus    SetMin           (double dMin) ;              // rtn: SM_ERR_INVALID_INPUT when assignment makes a negative interval
  SmStatus    SetMax           (double dMax) ;              // rtn: SM_ERR_INVALID_INPUT when assignment makes a negative interval
  void        SetUnbounded     () { m_dMin = -SM_INFINITE_PARAMETER; m_dMax = SM_INFINITE_PARAMETER; }
  SmExtent1d &ExpandAbsolute   (double dExpansion);
  SmExtent1d &ExpandRelative   (double dExpansionFactor);
  SmExtent1d &Scale            (double dScale) ;           // eff: Scale interval (scales center - neg numbers and zero are okay)
  SmExtent1d &Negate           ()              { double dTmp = m_dMin ; m_dMin = -m_dMax ; m_dMax = -dTmp ; return(*this) ; }
  SmExtent1d &Translate        (double dValue) { m_dMin = SM_IS_INFINITE(m_dMin) ? m_dMin : m_dMin + dValue ; 
                                                 m_dMax = SM_IS_INFINITE(m_dMax) ? m_dMax : m_dMax + dValue ;
                                                 return *this ; 
                                               }
  // 
  double      ClampValue       (double dValue) const; // rtn: value clamped to interval
  double      SnapValue        (double dValue, double dTol=SM_EFF_ZERO) const ; // rtn: value clamped to interval and snapped to end points
  double      OnlySnapValue    (double dValue, double dTol=SM_EFF_ZERO) const ; // rtn: values within tol of endPoints snapped to end points        
  double      Evaluate         (double dNormalizedParameter) const;

  // find SmExtent1d param (from 0.0 to 1.0) given a contained value
  SmStatus    Inversion        (double dValue, double & rdParameter) const;
  SmBoolean   GetTLeftEval     (double dUValue) const;

  // simple data access
  double      GetMin           () const { SM_ASSERT_DEFINED(this); return m_dMin; }
  double      GetMax           () const { SM_ASSERT_DEFINED(this); return m_dMax; }
  double      GetMid           () const { SM_ASSERT_DEFINED(this); return (m_dMax + m_dMin)/2.0 ; }
  double      GetLength        () const { SM_ASSERT_DEFINED(this); return m_dMax - m_dMin; }
  double      GetMaxDimension  () const { SM_ASSERT_DEFINED(this); return smos_Max(smos_Fabs(m_dMax),smos_Fabs(m_dMin)); }

  // Operations - Compute results from input extents
  void        Union            (const SmExtent1d & crOther, SmExtent1d & rResult) const;
  SmStatus    Intersect        (const SmExtent1d & crOther, SmExtent1d & rResult) const;
  double      MaximumDistance  (const SmExtent1d & crOther) const;
  double      MinimumDistance  (const SmExtent1d & crOther) const;
  double      DistanceFrom     (const double dValue       ) const;

  // predicates and classification
  SmBoolean   AreDisjoint          (const SmExtent1d & crOther, double dTol=0.0) const;
  SmBoolean   AreOverlapping       (const SmExtent1d & crOther, double dTol=0.0) const;
  SmBoolean   AreEqual             (const SmExtent1d & crOther, double dTol=0.0) const;
  SmBoolean   AreEqual             (double dMin, double dMax,   double dTol=0.0) const;
  SmBoolean   IsContainedBy        (const SmExtent1d & crOther, double dTol=0.0) const;
  SmBoolean   ContainsValue        (double dValue,              double dTol=0.0) const;  // rtn: (dVal > dMin-dTol) && (dVal < dMax+dTol)
  SmBoolean   ContainsValueRelative(double dValue,              double dRelTol ) const;
  SmBoolean   IsValueOnBoundary    (double dValue,              double dTol=SM_EFF_ZERO, // rtn: (fabs(dVal-Min) < dTol) || (fabs(dVal-Max) < dTol)
                                                                double *pOptOtherBndry=NULL) const;

  SmExtentPointType ClassifyPoint  (double dValue,              double dTol=SM_EFF_ZERO) const;

  SmBoolean   IsDegenerate         (double dTol=0.0) const ;                      // rtn: TRUE = has a zero or neg length dimension, FALSE=doesn't
  SmBoolean   HasNegativeLength    () const ;                                     // rtn: TRUE = has neg length as set by init, FALSE=doesn't 
  SmBoolean   IsInit               () const ;                                     // rtn: TRUE = MinMax=[SM_BIG_DOUBLE -SM_BIG_DOUBLE]
  SmBoolean   IsBounded            (SmBoundaryType *eOptBndryType=NULL) const ;   // rtn: TRUE = both boundaries are bounded (neither equal to SM_INFINITE_PARAMETER)      
  SmBoolean   AnyBounds            (SmBoundaryType *eOptBndryType=NULL) const ;   // rtn: TRUE = any boundary is bounded (either equal to SM_INFINITE_PARAMETER)      
  SmBoolean   operator==           (const SmExtent1d&) const;

  // periodic methods            
  double      PeriodicWrap             (double dValue) const;                     // rtn: value when whole interval is periodic

  double      ClampPeriodicValue       (double dValue, double dPeriod, double dTol=-1.0 ) const;

  SmStatus    IntersectPeriodic        (const SmExtent1d     & crOther, 
                                        double                 dPeriod, 
                                        SmTArray<SmExtent1d> & rResult, 
                                        SmBoolean bDontCrossBoundaries=FALSE) const;

  SmBoolean   IsClosed                 (double dPeriod) const;

  SmBoolean   AreDisjointPeriodic      (const SmExtent1d & crOther, double dPeriod, double dTol=-1.0 ) const;                                       // when two intervals are just part of a periodic space
  
  SmBoolean   ContainsPeriodicValue    (double dValue,    // when interval is just part of a periodic space
                                        double dPeriod,
                                        double *pOptPeriodicValue=NULL,
                                        double *pOptDistToMin=NULL,
                                        double *pOptDistToMax=NULL,
                                        double dTol=-1.0 ) const ; 

  SmBoolean   IsValueOnPeriodicBoundary(double dValue, double dPeriod,
                                        double dTolerance=SM_EFF_ZERO,
                                        double *pOptDist=NULL) const;

  // specialty functions
  SmExtent1d   ApproximateUnbounded                        // eff: return a bounded SmExtent1d to approximate an unbounded one                  
                 (double       dUnboundedCenter=0.0,       // in : center of unbounded interval, default:[0.0]
                  double       dUnboundedHalfSize=         // in : the size used for infinite 1/2 spaces
                                SM_BOUNDED_INFINITE_PARAM, //       a totally unbounded interval is approximated by an interval twice this size
                  SmExtent1d * pOptExpandedApprox=NULL)    // out: a 2nd extent expanded a small bit - used by graphics, NULL to ignore, default:[NULL]           
                 const ;

  // utilities
  // note: do not use SM_COMMON_BASE() because no virtual methods are allowed for SmExtent1d
  void          Dump             (void) const;
  SmBoolean     AssertValid(SmAssertArray    * pAList=NULL,           // i/o: Accumulating list of failed Asserts, NULL to ignore
                            SmAssertTestLevel  eTestLevel=SM_LEVEL_0, // in : SM_LEVEL_0=fewest tests, SM_LEVEL_1, SM_LEVEL_2=all tests, 
                                                                      //      SM_LEVEL_GIVEN = run tests in order requested in pTestRequests 
                                                                      //      default:[SM_LEVEL_0] 
                            SmAssertWalking    eWalkTree=SM_WALK,     // NotUsed: in : SM_WALK = run AssertValid on any topology graph descendants, SM_NO_WALK=don't, default:[SM_WALK] 
                            SmTArray<ULONG>  * pTestRequests=NULL)    // in : when eTestLevel == SM_LEVEL_GIVEN, run these tests in this order, default:[NULL]
                           const ;
  SmBoolean     AssertDefined    () const;

  SM_TYPE       GetType()            const { return(SmExtent1d_TYPE) ; }
  const TCHAR * GetTypeString()      const { return(_T("SmExtent1d_TYPE")) ; }
  const TCHAR * GetClassString()     const { return(_T("SmExtent1d")) ; }
  SM_TYPE       GetClassType()       const { return(SmExtent1d_TYPE) ; }
  const TCHAR * GetClassTypeString() const { return(_T("SmExtent1d_TYPE")) ; }
              
} ; // end class SmExtent1d
#endif // !__SMEXTENT1D_H__


