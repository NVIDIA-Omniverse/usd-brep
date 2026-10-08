// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*******************************************************************//**
* FILE NAME --- SmPeriodicExtent1d.h
* PURPOSE: Header file for SmPeriodicExtent1d class.
**********************************************************************/

#ifndef __SMPERIODICEXTENT1D_H__
#define __SMPERIODICEXTENT1D_H__

#ifndef __SMCORE_TYPES_H__
#include <SmCoreTypes.h>
#endif

#ifndef __SMOS_MATH_H__
#include <SmMath.h>
#endif

/*******************************************************************//**
PURPOSE: This enum describes the result of the intersection of 
            2 periodic extents. It is needed because we do not want
            the intersector to crash when the result is singular, the 
            user may need such results too.

NOTES: 
***********************************************************************/
enum SmPeriodicExtentIntersectionType {
    SM_PI_DISJOINT,      // no           XSect
    SM_PI_REGION,        // 1 Ivl        XSect
    SM_PI_POINT,         // 1 Pt         XSect
    SM_PI_2POINTS,       // 2 Pts        XSect
    SM_PI_2REGIONS,      // 2 Ivls       XSect
    SM_PI_REGION_POINT   // 1 Ivl & 1 pt XSect
};

/*******************************************************************//**
PURPOSE: This object represents an interval in one dimension on a periodic
    domain defined over:[0, m_dPeriod].  
    
NOTES:  1. The interval is defined by a minimum and maximum value.
           a. When min < max the interval does not cross the periodic
              boundary and looks like an interval in a non-periodic domain.
           b. When min > max the interval crosses the boundary.  It consists
              of two inteval segments:[max->Period] and [0->min].
           c. When min == max the interval is a single point.
           d. When min == 0 and max == periodic
                && m_bFullPeriod == TRUE,  the interval spans the entire domain.
                && m_bFullPeriod == FALSE, the interval is a point on the seam.
           e. No way to represent an empty set interval

        2. If upon construction, equal min and max values are given, they denote a 
              single point interval. If one wishes to construct the object to 
              contain the whole periodic domain, construct it by 0->period.


              0   Min     Max    Period
              +----|*******|-------+  when Min < Max, Exclude Seam, Ivl:[Min->Max], 
                                      Length = (m_dMax - m_dMin)

              0   Max     Min    Period
              +****|-------|*******+  when Min > Max, Include Seam, Ivl:[Min->Seam Seam->Max], 
                                      Length = (m_dPeriod - m_dMin) + (m_dMax - 0)

              when Min==0 && Max==Period - represent either a seam point or a full interval
              0                  Period
              +********************+    when m_bFullPeriod == TRUE  - every point in the period
              *--------------------*    when m_bFullPeriod == FALSE - a point on the seam

        3. No way to naturally represent Empty set and uninitialized Ivls, so we use magic numbers.

              empty interval:[SM_BIG_DOUBLE, -SM_BIG_DOUBLE]
***********************************************************************/
class SM_EXPORT SmPeriodicExtent1d
{
 protected:
  double    m_dMin =  SM_BIG_DOUBLE ;        // in range:[0 m_dPeriod], ivl start as: (m_dMin < m_dMax) ? [m_dMin m_dMax] else [ m_dMin->m_dPeriod 0->m_dMax] 
  double    m_dMax = -SM_BIG_DOUBLE ;        // in range:[0 m_dPeriod], ivl end   as: (m_dMin < m_dMax) ? [m_dMin m_dMax] else [ m_dMin->m_dPeriod 0->m_dMax] 
                                             // When m_dMin < m_dMax, Exclude Seam, Ivl:[Min->Max], 
                                             //                                     Length:[m_dMax - m_dMin]
                                             // When m_dMax < m_dMin, Include Seam, Ivl:[Min->Seam Seam->Max], 
                                             //                                     Length:[(m_dPeriod - m_dMin) + (m_dMax - 0)] = [m_dMax - m_dMin + m_dPeriod]

  double    m_dPeriod = -SM_BIG_DOUBLE ;     // Periodic range as:[0 m_dPeriod]
  SmBoolean m_bFullPeriod = FALSE ;          // TRUE : interval:[0 m_dPeriod] is full
                                             // FALSE: Interval:[0 m_dPeriod] is two points on seam or
                                             //        interval:[m_dMin m_dMax]

  // private empty constructor
  SmPeriodicExtent1d() { } // not to be used

  void Normalize(void);

 public:
  // constructructors
  SmPeriodicExtent1d(double dMin, double dMax, double dPeriod); // construct Specified Ivl extent
  SmPeriodicExtent1d(double dPeriodArg);                        // construct FullPeriod Ivl extent

  // copy constructor
  SmPeriodicExtent1d(const SmPeriodicExtent1d & crOriginal);

  // destructor
  ~SmPeriodicExtent1d() {}

  // equality operator
  SmBoolean operator==       (const SmPeriodicExtent1d & crOther) const;

  // side effects
  void      AddValue      (double dValue, SmBoolean bForward) ;
  SmStatus  AddInterval   (double dFrom, double dTo, SmBoolean bForward) ; 
  void      Invert        () ; // Preserves Ivl length - moves old min and max dists from 0 to become min and max dists from Period
  void      Compliment    () ; // switches between the NoSeam and WithSeam intervals
  void      ExpandAbsolute(double dExpansion) ;
  void      Translate     (double) ;

  // simple data access
  double    GetMax       ()               const { return m_dMax; }
  double    GetMin       ()               const { return m_dMin; }
  double    GetMid       ()               const ;
  double    GetLength    ()               const ;
  double    GetPeriod    ()               const { return m_dPeriod ; }
  SmBoolean GetFullPeriod()               const { return(m_bFullPeriod) ; }
  SmBoolean GetTLeftEval (double dUValue) const ;
  SmStatus  SetMinMax    (double dMin, double dMax, double * pOptPeriod=NULL, SmBoolean bSetSinglePoint = FALSE) ;
  void      SetFullPeriod(SmBoolean bFullPeriod) { if(bFullPeriod) { m_bFullPeriod = TRUE ; } }
  void      SetEmpty     ()                      { m_dMin =  SM_BIG_DOUBLE ;
                                                   m_dMax = -SM_BIG_DOUBLE ;
                                                 }
  // predicates
  SmBoolean IsInit           ()              const { return(m_bFullPeriod != -SM_BIG_DOUBLE) ; }
  SmBoolean IsEmpty          ()              const { return(m_dMin == SM_BIG_DOUBLE && m_dMax == -SM_BIG_DOUBLE) ; }
  SmBoolean IsPoint          ()              const ; // tol = ScaledZero(dPeriod)
  SmBoolean IsIvl            ()              const { return(!IsEmpty() && !IsPoint() && !IsFullPeriod()) ; }
  SmBoolean IsFullPeriod     ()              const { return(m_bFullPeriod) ; }
  SmBoolean CrossesSeam      ()              const { return(m_dMax < m_dMin) ; }
  SmBoolean ContainsValue    (double dValue) const ;
  SmBoolean IsValueOnBoundary(double dValue,
                              double dTol1d=SM_EFF_ZERO) const ;
  SmBoolean IsContainedBy    (const SmPeriodicExtent1d & crOther) const ;
  SmBoolean AreDisjoint      (const SmPeriodicExtent1d & crOther) const ;

  //These 2 do not make much sense in this context
  //double MaximumDistance(const SmPeriodicExtent1d & crOther) const;
  //double MinimumDistance(const SmPeriodicExtent1d & crOther) const;

  // computations
  double   MapValue      (double dLocation)            const { return( NormalizeValue(dLocation) ) ; }
  double   NormalizeValue(double dLocation)            const ; // map value to primary period (does snapping to end values)
  double   Evaluate      (double dNormalizedParameter) const ;
  SmStatus Union    (const SmPeriodicExtent1d   & crOther, 
                     SmPeriodicExtent1d         & rResult)  const ;
  SmStatus Intersect(const SmPeriodicExtent1d         & crOther,
                     ULONG                            & clResultCnt,
                     SmPeriodicExtent1d               & rResult1,
                     SmPeriodicExtent1d               & rResult2,
                     SmPeriodicExtentIntersectionType & bResultType) const ;

  SmStatus Subtract (const SmPeriodicExtent1d   & crOther, 
                     ULONG                      & clResultCnt,
                     SmPeriodicExtent1d         & rResult1,
                     SmPeriodicExtent1d         & rResult2) const ;

  // utilities
  // note: do not use SM_COMMON_BASE() because no virtual methods are allowed for SmPeriodicExtent1d
  void           Dump       (void) const;
  SmBoolean      AssertValid(SmAssertArray    * pAList=NULL,           // i/o: Accumulating list of failed Asserts, NULL to ignore
                             SmAssertTestLevel  eTestLevel=SM_LEVEL_0, // in : SM_LEVEL_0=fewest tests, SM_LEVEL_1, SM_LEVEL_2=all tests, 
                                                                       //      SM_LEVEL_GIVEN = run tests in order requested in pTestRequests 
                                                                       //      default:[SM_LEVEL_0] 
                             SmAssertWalking    eWalkTree=SM_WALK,     // NotUsed: in : SM_WALK = run AssertValid on any topology graph descendants, SM_NO_WALK=don't, default:[SM_WALK] 
                             SmTArray<ULONG>  * pTestRequests=NULL)    // in : when eTestLevel == SM_LEVEL_GIVEN, run these tests in this order, default:[NULL]
                            const ;
                 
  SM_TYPE        GetType()            const { return(SmPeriodicExtent1d_TYPE) ; }
  const TCHAR  * GetTypeString()      const { return(_T("SmPeriodicExtent1d_TYPE")) ; }
  const TCHAR  * GetClassString()     const { return(_T("SmPeriodicExtent1d")) ; }
  SM_TYPE        GetClassType()       const { return(SmPeriodicExtent1d_TYPE) ; }
  const TCHAR  * GetClassTypeString() const { return(_T("SmPeriodicExtent1d_TYPE")) ; }


} ; // end class SmPeriodicExtent1d


#endif // !__SMPERIODICEXTENT1D_H__
