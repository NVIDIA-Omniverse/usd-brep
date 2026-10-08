// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************//**
* FILE NAME --- SmExtent1d.cpp
* PURPOSE: Implementation of SmExtent1d methods.
**********************************************************************/

#include "StdAfx.h"

#include <SmExtent1d.h>
#include <SmTArray.h>
#include <SmAssertArray.h>

/*******************************************************************//**
PURPOSE: Get the T Evaluation side of a curve domain.
            
RETURN ---  TRUE = TValue is in lower half of interval
            FALSE= TVAlue is in upper half of interval  

NOTES: This method is used to specify the bFromLeft 
    flag needed for computing curve and surface derivatives at
    discontinuity points.  (the discontinuity is expected to
    be on the boundary of the interval.)

    +----1----+----2----+        Curve with two intervals
              P
    When evaluating a point P on an interval boundary and 
    bFromLeft = TRUE  evaluate P in upper interval 2, P is on the left of the interval
                FALSE evaluate P in lower interval 1, P is on the right of the interval
***********************************************************************/
SmBoolean SmExtent1d::GetTLeftEval
  (double dTValue)         // in : interval parameter to test
 const
{
  SM_ASSERT_DEFINED(this);
  double    dTMid = (m_dMin + m_dMax) / 2.0;
  SmBoolean bRet  = (dTValue <= dTMid)
                    ? TRUE
                    : FALSE ;
  return(bRet) ;

} // end SmExtent1d::GetTLeftEval

/*******************************************************************//**
PURPOSE: AssertValid() Test reporting static labels
***********************************************************************/
SmAssertReportLabel sAssertExtent1d_list[] =
{
  {SM_AT_MINMAX, _T("Min Max"), _T("m_dMin <= m_dMax") }
} ;


/*******************************************************************//**
PURPOSE: Determine validity of the SmExtent1d.  

RETURNS ---  TRUE  = OK
             FALSE = Problem
***********************************************************************/
SmBoolean SmExtent1d::AssertValid
 (SmAssertArray    * pAList,        // i/o: Accumulating list of failed Asserts, NULL to ignore, default:[NULL] 
  SmAssertTestLevel  eTestLevel,    // in : SM_LEVEL_0=fewest tests, SM_LEVEL_1, SM_LEVEL_2=all tests, 
                                    //      SM_LEVEL_GIVEN = run tests in order requested in pTestRequests 
                                    //      default:[SM_LEVEL_0] 
  SmAssertWalking    eWalkTree,     // NotUsed: in : SM_WALK = run AssertValid on any topology graph descendants, SM_NO_WALK=don't, default:[SM_WALK] 
  SmTArray<ULONG>  * pTestRequests) // in : when eTestLevel == SM_LEVEL_GIVEN, run these tests in this order, default:[NULL]
 const
{
  SM_REF1(eWalkTree) ;
  SM_REF1(pTestRequests);

  // init output
  SmBoolean bRtn = TRUE;

  bRtn &= SM_ASSERT_BOOLEAN_REPORT(0, SM_LEVEL_0, (m_dMin <= m_dMax), _T("") ) ;

  SM_ASSERT_BREAK(bRtn) ;
  return(bRtn) ;

} // end SmExtent1d::AssertValid

/*******************************************************************//**
PURPOSE: return TRUE when member values are defined, else FALSE. 

NOTES: Values are set to undefined when object is destructed.
***********************************************************************/
SmBoolean SmExtent1d::AssertDefined(void) const
{
  SmBoolean bRtn = (   m_dMin!= SM_UNDEF_DOUBLE 
                    && m_dMax!=-SM_UNDEF_DOUBLE) ;

  SM_ASSERT_BREAK(bRtn) ;
  return(bRtn) ;

} // end SmExtent1d::AssertDefined

/*******************************************************************//**
PURPOSE: Copy constructor.  

NOTES: It initializes the values from from an existing SmExtent1d.
***********************************************************************/
SmExtent1d::SmExtent1d
  (const SmExtent1d & crOriginal)
{
   m_dMin = crOriginal.m_dMin;
   m_dMax = crOriginal.m_dMax;

} // end SmExtent1d::SmExtent1d

/*******************************************************************//**
PURPOSE: Constructor which takes a minimum and maximum value.

NOTES: The dMin must be less than or equal to dMax.
***********************************************************************/
SmExtent1d::SmExtent1d
  (double dMin, 
   double dMax) 
{ 
  m_dMin = dMin; 
  m_dMax = dMax; 
  if ( dMin > dMax )
    {
      SE( SM_ERR );
      m_dMin = dMax; 
      m_dMax = dMin; 
    }

} // end SmExtent1d::SmExtent1d

/*******************************************************************//**
PURPOSE: Given a value, extend the interval of the extent to cover
     the value if necessary.  

NOTES: If the value is inside of the interval
     then there is no effect on the extent.
***********************************************************************/
void SmExtent1d::AddValue
 (double dValue)
{ 
  if (dValue < m_dMin) m_dMin = dValue; 
  if (dValue > m_dMax) m_dMax = dValue; 

} // end SmExtent1d::AddValue

/*******************************************************************//**
PURPOSE: Clamp the value to be within this interval.
   
NOTES:
***********************************************************************/
double SmExtent1d::ClampValue
  (double dValue) 
 const
{
  SM_ASSERT_DEFINED(this);
  return(  ( dValue < m_dMin) ? m_dMin
         : ( dValue > m_dMax) ? m_dMax
         :                      dValue) ; 

  // double dRet = dValue;
  // if      (dRet < m_dMin) { dRet = m_dMin; }
  // else if (dRet > m_dMax) { dRet = m_dMax; }
  // 
  // return dRet;

} // end SmExtent1d::ClampValue

/*******************************************************************//**
PURPOSE: Clamp and Snap value to the EndPoints of this interval.
   
NOTES: 1. Snap values within Tol of endPoints to EndPoint values
       2. Values more than tol inside of the end points are not changed
***********************************************************************/
double SmExtent1d::SnapValue
  (double dValue,  // in : input value to check
   double dTol)    // in : Max1d dist from end points snapped to endpoint values 
 const
{
  SM_ASSERT_DEFINED(this);
  return(  ( dValue < m_dMin+dTol) ? m_dMin
         : ( dValue > m_dMax-dTol) ? m_dMax
         :                           dValue) ;

  // double dRet = dValue ;
  // if      (dRet < m_dMin+dTol) { dRet = m_dMin ; }
  // else if (dRet > m_dMax-dTol) { dRet = m_dMax ; }
  // 
  // return dRet;

} // end SmExtent1d::SnapValue

/*******************************************************************//**
PURPOSE: Only Snap value to the EndPoints of this interval.
   
NOTES: 1. Snap values within Tol of endPoints to EndPoint values
       2. otherwise return input value unchanged
***********************************************************************/
double SmExtent1d::OnlySnapValue
  (double dValue,  // in : input value to check
   double dTol)    // in : Max1d dist from end points snapped to endpoint values 
 const
{
  SM_ASSERT_DEFINED(this);
  double dRet = dValue;
  if      ((dRet < m_dMin+dTol) && (dRet > m_dMin-dTol)) { dRet = m_dMin; }
  else if ((dRet < m_dMax+dTol) && (dRet > m_dMax-dTol)) { dRet = m_dMax; }

  return dRet;

} // end SmExtent1d::SnapValue

/*******************************************************************//**
PURPOSE: Perform a normalized evaluation on the extent.  

NOTES: The normalized parameter is a value beteween 0.0 and 1.0
   inclusive.  0.0 maps to Min and 1.0 maps to Max.
   This is the inverse of Inversion().

   The result is not clamped.  The caller can do that if appropriate.
***********************************************************************/
double SmExtent1d::Evaluate
  (double dNormalizedParameter) 
 const
{
  SM_ASSERT_DEFINED(this);
  if ( dNormalizedParameter == 0.0 )
    { return m_dMin; }
  if ( dNormalizedParameter == 1.0 )
    { return m_dMax; }
  return (m_dMin + dNormalizedParameter * (m_dMax - m_dMin));

} // end SmExtent1d::Evaluate

/*******************************************************************//**
PURPOSE: Set the values for the interval of the extent.

NOTES: returns: SM_SUCCESS when           dMin <= dMax
                SM_ERR_INVALID_INPUT when dMin  > dMax
***********************************************************************/
SmStatus SmExtent1d::SetMinMax
  (double dMin, 
   double dMax)    
{ 
  SM_ASSERT_DEFINED(this);
  SmStatus sRet = SM_SUCCESS;

  // prohibit negative intervals - allow zero or positive intervals
  if (dMin > dMax) 
    {
      SE(SM_ERR_INVALID_INPUT); 
      sRet = SM_ERR_INVALID_INPUT;
    }
  else {m_dMin = dMin; m_dMax = dMax;}
  return sRet;

} // end SmExtent1d::SetMinMax

/*******************************************************************//**
PURPOSE: Set the min value 

RETURNS: SM_SUCCESS or 
         SM_ERR_INVALID_INPUT when assignment makes a negative interval
***********************************************************************/
SmStatus SmExtent1d::SetMin
  (double dMin)    
{ 
  SM_ASSERT_DEFINED(this);

  if ( dMin > m_dMax ) SER(SM_ERR_INVALID_INPUT);
  m_dMin = dMin;
  return SM_SUCCESS;

} // end SmExtent1d::SetMin

/*******************************************************************//**
PURPOSE: Set the max value 

RETURNS: SM_SUCCESS or
         SM_ERR_INVALID_INPUT when assignment makes a negative interval
***********************************************************************/
SmStatus SmExtent1d::SetMax
  (double dMax)    
{ 
  SM_ASSERT_DEFINED(this);

  // assignment
  if ( dMax < m_dMin ) SER(SM_ERR_INVALID_INPUT);
  m_dMax = dMax;
  return SM_SUCCESS;

} // end SmExtent1d::SetMax

/*******************************************************************//**
PURPOSE: Perform a union operation between two extents.  

NOTES: This produces an interval whose maximum is the maximum of
   both maximums and whose minimum is the mimimum of both minimums.
***********************************************************************/
void SmExtent1d::Union
  (const SmExtent1d & crOther, 
   SmExtent1d       & rResult) 
 const
{
  SM_ASSERT_DEFINED(this);
  SM_ASSERT_DEFINED(&crOther);
  rResult.m_dMin = smos_Min ( m_dMin, crOther.m_dMin );
  rResult.m_dMax = smos_Max ( m_dMax, crOther.m_dMax );

} // end SmExtent1d::Union

/*******************************************************************//**
PURPOSE: Determine the maximum distance spaned by the union of
    these two intervals.  

NOTES: 
***********************************************************************/
double SmExtent1d::MaximumDistance
  (const SmExtent1d & crOther) 
 const
{
  SM_ASSERT_DEFINED(this);
  SM_ASSERT_DEFINED(&crOther);
  SmExtent1d sUnion;
  Union(crOther,sUnion);
  return sUnion.m_dMax-sUnion.m_dMin;

} // end SmExtent1d::MaximumDistance

/*******************************************************************//**
PURPOSE: Determine the minimum distance between these two intervals.

NOTES: Intervals which touch or intersect will return a 0.0.
***********************************************************************/
double SmExtent1d::MinimumDistance
  (const SmExtent1d & crOther) 
 const
{
  SM_ASSERT_DEFINED(this);
  SM_ASSERT_DEFINED(&crOther);
  double dRet = 0.0;
  if     (m_dMin > crOther.m_dMax) dRet = m_dMin - crOther.m_dMax;
  else if(m_dMax < crOther.m_dMin) dRet = crOther.m_dMin - m_dMax;
  return dRet;

} // end SmExtent1d::MinimumDistance

/*******************************************************************//**
PURPOSE: Determine the distance from a point (double value) to this interval.

NOTES: Returns 0.0 for values within the interval.
***********************************************************************/
double SmExtent1d::DistanceFrom
  (const double dValue) 
 const
{
  SM_ASSERT_DEFINED(this);

  if ( dValue < m_dMin ) { return m_dMin - dValue; }
  if ( dValue > m_dMax ) { return dValue - m_dMax; }
  return 0.0;

} // end SmExtent1d::DistanceFrom

/*******************************************************************//**
PURPOSE: Determine the intersection of two intervals which are not 
    disjoint.  

RETURNS AND OUTPUT --- 
   SM_SUCCESS = input intervals overlap,      rResult set = overlap interval
   SM_ERR     = input intervals are disjoint, rResult set = Init() (Negative Interval)
***********************************************************************/
SmStatus SmExtent1d::Intersect
  (const SmExtent1d & crOther,   // in : other interval to intersect
   SmExtent1d       & rResult)   // out: intersection interval or
                                 //      Init() interval when disjoint
  const
{
  SM_ASSERT_DEFINED(this);
  SM_ASSERT_DEFINED(&crOther);
  SmStatus sRet = SM_SUCCESS;
  if (AreDisjoint(crOther)) 
    {
      rResult.Init() ;
      sRet = SM_ERR;
    }
  else 
    {
      rResult.m_dMin = smos_Max ( m_dMin, crOther.m_dMin );
      rResult.m_dMax = smos_Min ( m_dMax, crOther.m_dMax );
    }
  return sRet;

} // end SmExtent1d::Intersect

/*******************************************************************//**
PURPOSE: Expand the interval by a value.

NOTES: dExpansion can be positive or negative
***********************************************************************/
SmExtent1d & SmExtent1d::ExpandAbsolute
  (double dExpansion)
{
  SM_ASSERT_DEFINED(this);
  m_dMin -= (!SM_IS_INFINITE(m_dMin)) ? dExpansion : 0.0 ;
  m_dMax += (!SM_IS_INFINITE(m_dMax)) ? dExpansion : 0.0 ; 

  // all done
  return(*this) ;

} // end SmExtent1d::ExpandAbsolute

/*******************************************************************//**
PURPOSE: Expand the interval by a relative factor in both directions.

NOTES: 
***********************************************************************/
SmExtent1d & SmExtent1d::ExpandRelative(double dExpansionFactor)
{
  SM_ASSERT_DEFINED(this);
  SM_ASSERT_MSG(SM_IS_INFINITE(dExpansionFactor)==FALSE, _T("SmExtent1d::ExpandRelative passed an infinite scale factor")) ;
  double sParamInc( ( m_dMax - m_dMin ) * dExpansionFactor );
  m_dMin -= (!SM_IS_INFINITE(m_dMin)) ? sParamInc : 0.0 ;
  m_dMax += (!SM_IS_INFINITE(m_dMax)) ? sParamInc : 0.0 ;

  return(*this) ;

} // end SmExtent1d::ExpandRelative

/*******************************************************************//**
PURPOSE: Scale the extent.

NOTES: okay to have negative numbers 
       scaling by zero gets a degenerate interval on the origin
***********************************************************************/
SmExtent1d & SmExtent1d::Scale(double dScale)
{
  SM_ASSERT_DEFINED(this) ; 
  SM_ASSERT_MSG(SM_IS_INFINITE(dScale)==FALSE, _T("SmExtent1d::Scale passed an infinite scale factor")) ;
  m_dMin *= (!SM_IS_INFINITE(m_dMin)) ? dScale : 1.0 ;
  m_dMax *= (!SM_IS_INFINITE(m_dMax)) ? dScale : 1.0 ;
  if(m_dMin > m_dMax) { double dTmp = m_dMin ;
                        m_dMin      = m_dMax ; 
                        m_dMax      = dTmp ;
                      }
  return(*this) ;

} // end SmExtent1d::Scale

/*******************************************************************//**
PURPOSE: Determine the parameter corresponding to the value which is
   inside or on the interval boundaries.

NOTES: 
   This is the inverse of Evaluate().
   The result is not clamped.  The caller can do that if appropriate.

RETURNS: 
  SM_SUCCESS and rdParamenter = inverse,         when value is within tol of interval
  SM_ERR     and rdParamenter = inverse,         when value is more than tol out of the interval
  SM_ERR     and rdParamenter = SM_UNDEF_DOUBLE, when interval is zero length
***********************************************************************/
SmStatus SmExtent1d::Inversion
  (double   dValue, 
   double & rdParameter) 
  const
{
  SM_ASSERT_DEFINED(this);

  // init return values
  SmStatus sRet = SM_SUCCESS;
  rdParameter   = SM_UNDEF_DOUBLE ;

  // Set return value to SM_ERR if the input value
  // is not in or close to our interval.
  if ( !ContainsValue(dValue) )
    { sRet = SM_ERR ; }

  // do the linear mapping
  double dNumer = dValue - m_dMin;
  double dDenom = m_dMax - m_dMin;
  if ( dDenom < smos_Fabs( dNumer ) * SM_EFF_ZERO )
    {
      sRet = SM_ERR;
      SM_DBG_WARN(_T("Attempted an inversersion on a zero length interval")) ;
    }
  else // do the linear mapping
    {
      rdParameter = dNumer / dDenom;
    }

  // all done
  return sRet;

} // end SmExtent1d::Inversion

/*******************************************************************//**
PURPOSE: Determine if two intervals are disjoint on the real line.

NOTES: They are not considered disjoint if they touch.
***********************************************************************/
SmBoolean SmExtent1d::AreDisjoint
  (const SmExtent1d & crOther,
   double             dTol) // default:[0.0]
 const
{
  SM_ASSERT_DEFINED(this);
  SM_ASSERT_DEFINED(&crOther);
  SmBoolean bRet = (   (m_dMin > crOther.m_dMax - dTol)
                    || (m_dMax < crOther.m_dMin + dTol)) ;
  return bRet;

} // end SmExtent1d::AreDisjoint

/*******************************************************************//**
PURPOSE: Determine if two intervals overlap by more than tolerance

NOTES: They are not considered overlapping if they touch or overlap by
       less than tolerance.
***********************************************************************/
SmBoolean SmExtent1d::AreOverlapping
  (const SmExtent1d & crOther,
   double             dTol) // default:[0.0]
 const
{
  SM_ASSERT_DEFINED(this);
  SM_ASSERT_DEFINED(&crOther);
  SmBoolean bNotOverlapping = (  ( (m_dMin < crOther.m_dMax) && (m_dMax <= crOther.m_dMin + dTol) )
                               || ( (crOther.m_dMin < m_dMax) && (crOther.m_dMax <= m_dMin + dTol)) ) ;
  return !bNotOverlapping ;

} // end SmExtent1d::AreOverlapping

/*******************************************************************//**
PURPOSE: Determine if two intervals are equal (to dTol) on the real line.

NOTES:
***********************************************************************/
SmBoolean SmExtent1d::AreEqual
  (const SmExtent1d & crOther,
   double             dTol) // default:[0.0]
 const
{
  SM_ASSERT_DEFINED(this);
  SM_ASSERT_DEFINED(&crOther);
  return AreEqual( crOther.m_dMin, crOther.m_dMax, dTol );

} // end SmExtent1d::AreEqual

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
SmBoolean SmExtent1d::AreEqual
  (double dMin,
   double dMax,
   double dTol) // default:[0.0]
 const
{
  SM_ASSERT_DEFINED(this);
  SmBoolean bRet = (   SM_ARE_SAME_TO_TOL(m_dMin, dMin, dTol) 
                    && SM_ARE_SAME_TO_TOL(m_dMax, dMax, dTol)) ;
  return bRet;

} // end SmExtent1d::AreEqual

/*******************************************************************//**
PURPOSE: Determine if one interval is completely contained by another.

NOTES: Equal intervals are contained in each other.
***********************************************************************/
SmBoolean SmExtent1d::IsContainedBy
  (const SmExtent1d & crOther,
   double             dTol) // default:[0.0]
 const
{
  SM_ASSERT_DEFINED(this);
  SM_ASSERT_DEFINED(&crOther);
  SmBoolean bRtn = (   crOther.m_dMin - dTol <= m_dMin
                    && crOther.m_dMax + dTol >= m_dMax ) ;
  return bRtn;

} // end SmExtent1d::IsContainedBy

/*******************************************************************//**
PURPOSE: Determine if the interval of the extent contains a given value.

NOTES: If the value equal to one of the boundaries of the extent
   (taking dTol into account), then true is returned.
***********************************************************************/
SmBoolean SmExtent1d::ContainsValue
  (double dValue,      // in : target value to check
   double dTol)        // in : default:[0.0]
 const
{ 
  SM_ASSERT_DEFINED(this);

  // return ((dValue < m_dMin || dValue > m_dMax ) ? FALSE : TRUE);
  SmBoolean bRtn = (   dValue >= m_dMin - dTol 
                    && dValue <= m_dMax + dTol) ;
  return bRtn ;

} // end SmExtent1d::ContainsValue

/*******************************************************************//**
PURPOSE: Determine if the interval of the extent contains a given value,
   given a relative tolerance.

NOTES: 
***********************************************************************/
SmBoolean SmExtent1d::ContainsValueRelative( double dValue, double dRelativeTol ) const
{
  SM_ASSERT_DEFINED(this);

  double dAbsoluteTol = dRelativeTol * ( m_dMax - m_dMin );

  if ( dValue < m_dMin - dAbsoluteTol ) { return FALSE; }
  if ( dValue > m_dMax + dAbsoluteTol ) { return FALSE; }
  return TRUE;

} // end SmExtent1d::ContainsValue

/*******************************************************************//**
PURPOSE: Determine if the value is on the boundary to within the given
    tolerance value.

NOTES: 
***********************************************************************/
SmBoolean SmExtent1d::IsValueOnBoundary
  (double   dValue,          // in : Target param value
   double   dOptTol,         // in : Optional Tol value, default:[SM_EFF_ZERO]
   double * pOptOBndryValue) // out: When dValue is OnBndry, Optional OtherBndry value
  const                      //      NULL to ignore, default:[NULL]
{
  SM_ASSERT_DEFINED(this);
  double    dDelMin = smos_Fabs(dValue-m_dMin) ;
  double    dDelMax = smos_Fabs(dValue-m_dMax) ;
  SmBoolean bMin = dDelMin < dOptTol ;
  SmBoolean bMax = dDelMax < dOptTol ;

  // optional output
  if(pOptOBndryValue)
    {
      *pOptOBndryValue =  (bMin && bMax) ? ( (dDelMin < dDelMax) ? m_dMax : m_dMin)
                         : bMin ? m_dMax : m_dMin ; 
    }

  return(bMin || bMax) ;

} // end SmExtent1d::IsValueOnBoundary

/*******************************************************************//**
PURPOSE: Classify Point to the boundaries, interior, and exterior of
  the extent.

NOTES: 
***********************************************************************/
SmExtentPointType SmExtent1d::ClassifyPoint
  (double dValue,
   double dTol) // default:[SM_EFF_ZERO]
  const
{
  SM_ASSERT_DEFINED(this);
  SmExtentPointType bRtn =    (   (smos_Fabs(dValue-m_dMin) <= dTol)
                               && (smos_Fabs(dValue-m_dMax) <= dTol)) ? SM_EP_BOTH
                            : (smos_Fabs(dValue-m_dMin) <= dTol)      ? SM_EP_START
                            : (smos_Fabs(dValue-m_dMax) <= dTol)      ? SM_EP_END
                            : (   (dValue-m_dMin >= 0.0)               
                               && (m_dMax-dValue >= 0.0))             ? SM_EP_INSIDE
                            : SM_EP_OUTSIDE ;
  return bRtn ;

} // end SmExtent1d::ClassifyPoint

/*******************************************************************//**
PURPOSE: Return TRUE when dimension extent is negative.   

NOTES: 
***********************************************************************/
SmBoolean SmExtent1d::IsDegenerate
  (double dTol)       // in : default:[0.0]
 const
{
  SM_ASSERT_DEFINED(this);
  return(   (m_dMin >= m_dMax - dTol)
         && (m_dMin <= m_dMax + dTol)) ;

} // end SmExtent3d::HasNegativeVolume

/*******************************************************************//**
PURPOSE: Return TRUE when dimension extent is negative.   

NOTES: 
***********************************************************************/
SmBoolean SmExtent1d::HasNegativeLength
  () 
 const
{
  SM_ASSERT_DEFINED(this);
  return(m_dMin > m_dMax) ;

} // end SmExtent3d::HasNegativeVolume

/*******************************************************************//**
PURPOSE: Return TRUE when dimensions are all set to init values

NOTES: 
***********************************************************************/
SmBoolean SmExtent1d::IsInit() 
 const
{
  return(   m_dMin ==  SM_BIG_DOUBLE
         && m_dMax == -SM_BIG_DOUBLE) ; 

} // end SmExtent1d::IsInit

/*******************************************************************//**
PURPOSE: Return TRUE when both boundaries are bound,
         no +/-SM_INFINITE_PARAMETER values.

NOTES: 
***********************************************************************/
SmBoolean SmExtent1d::IsBounded
 (SmBoundaryType *eOptBndryType)   // out: oneof SM_BT_BOUNDED,     
                                   //            SM_BT_UNBOUNDED_MIN
                                   //            SM_BT_UNBOUNDED_MAX
                                   //            SM_BT_UNBOUNDED,   
                                   //      NULL to ignore, default:[NULL]
 const
{
  SmBoundaryType  eBndryType ; 
  SmBoundaryType *pBndryType = eOptBndryType ? eOptBndryType : &eBndryType ; 
  
  // classify interval
  *pBndryType =  (   SM_IS_INFINITE(m_dMax)
                  && SM_IS_INFINITE(m_dMin)) ? SM_BT_UNBOUNDED
               : SM_IS_INFINITE(m_dMin)      ? SM_BT_UNBOUNDED_MIN
               : SM_IS_INFINITE(m_dMax)      ? SM_BT_UNBOUNDED_MAX
               :                               SM_BT_BOUNDED ;

  // all done return bounded status
  return( *pBndryType == SM_BT_BOUNDED ) ;                                   

} // end SmExtent1d::IsBounded

/*******************************************************************//**
PURPOSE: Return TRUE when any boundary is bound,
         any value not euqal to +/-SM_INFINITE_PARAMETER values.

NOTES: 
***********************************************************************/
SmBoolean SmExtent1d::AnyBounds
 (SmBoundaryType *eOptBndryType)   // out: oneof SM_BT_BOUNDED,     
                                   //            SM_BT_UNBOUNDED_MIN
                                   //            SM_BT_UNBOUNDED_MAX
                                   //            SM_BT_UNBOUNDED,   
                                   //      NULL to ignore, default:[NULL]
 const
{
  SmBoundaryType  eBndryType ;
  SmBoundaryType *pBndryType = eOptBndryType ? eOptBndryType : &eBndryType ;

  IsBounded(pBndryType) ;

  return( *pBndryType != SM_BT_UNBOUNDED ) ;

} // end SmExtent1d::AnyBounds

/*******************************************************************//**
PURPOSE: return a bounded SmExtent1d to approximate an unbounded one

NOTES: 0. returns a SmExtent1d whose infinite boundary values 
          have been replaced by finite values.
          Those replacement values are based upon the dUnboundedHalfSize,
          pUnboundedCenter, and the interval's opposing end value.

       1. This is used for graphics and sampling to allow an application
          to easily define an area of focus for infinite extents.
          
       2. This method handles all the combinations of half spaces and unbounded spaces.
          
       3. The approximation of a BOUNDED extent is an exact copy of the original extent.
***********************************************************************/
SmExtent1d SmExtent1d::ApproximateUnbounded  // eff: return a bounded SmExtent1d to approximate an unbounded one                  
 (double      dUnboundedCenter,              // in : center of unbounded interval, default:[0.0]
  double      dUnboundedHalfSize,            // in : the size used for infinite 1/2 spaces, default:[SM_BOUNDED_INFINITE_PARAM]
                                             //      a totally unbounded interval is approximated by an interval twice this size
  SmExtent1d *pOptExpandedApprox)            // out: a 2nd extent expanded a small bit - used by graphics, NULL to ignore, default:[NULL]           
 const  
{
  // locals
  SmBoundaryType eBdryType ;

  // copy min/max points
  double dMin = m_dMin ;
  double dMax = m_dMax ;

  // check for infinite boundaries
  SmBoolean bBounded = this->IsBounded(&eBdryType) ;
                                                        
  // check for infinite boundaries
  if(bBounded == FALSE)
    {
      // move infintie boundaries to viewable places - draw them as a double plane with an internal x
      dMin =   eBdryType == SM_BT_BOUNDED       ? dMin
             : eBdryType == SM_BT_UNBOUNDED_MIN ? (  (dMax < dUnboundedCenter + dUnboundedHalfSize)
                                                   ? (dMax - 2 * dUnboundedHalfSize)
                                                   : (dUnboundedCenter - dUnboundedHalfSize))
             : eBdryType == SM_BT_UNBOUNDED_MAX ? (  (dMin > dUnboundedCenter - dUnboundedHalfSize)
                                                   ? (dMin)
                                                   : (dUnboundedCenter - dUnboundedHalfSize))
             : eBdryType == SM_BT_UNBOUNDED     ? dUnboundedCenter - dUnboundedHalfSize : dMin ;

      dMax =   eBdryType == SM_BT_BOUNDED       ? dMax
             : eBdryType == SM_BT_UNBOUNDED_MIN ? (  (dMax < dUnboundedCenter + dUnboundedHalfSize)
                                                   ? (dMax)
                                                   : (dUnboundedCenter + dUnboundedHalfSize))
             : eBdryType == SM_BT_UNBOUNDED_MAX ? (  (dMin > dUnboundedCenter - dUnboundedHalfSize)
                                                   ? (dMin + 2 * dUnboundedHalfSize)
                                                   : (dUnboundedCenter + dUnboundedHalfSize))
             : eBdryType == SM_BT_UNBOUNDED     ? dUnboundedCenter + dUnboundedHalfSize : dMax ;

    } // end unbounded check

  // when asked - build an expanded extent as well
  if(pOptExpandedApprox)
    {
      double dMinInc = dMin ;
      double dMaxInc = dMax ;
      double dInfInc = dUnboundedHalfSize/12.0 ;

      // when extent is not bounded - it needs to be expanded
      if(bBounded == FALSE)
        {
          // else expand the bbox by dInfInc
          if(   eBdryType == SM_BT_UNBOUNDED_MIN
             || eBdryType == SM_BT_UNBOUNDED)     { dMinInc -= dInfInc ; }    
          if(   eBdryType == SM_BT_UNBOUNDED_MAX
             || eBdryType == SM_BT_UNBOUNDED)     { dMaxInc += dInfInc ; }    

        } // end bBounded == FALSE check

      // set output
      pOptExpandedApprox->SetMinMax(dMinInc, dMaxInc) ;

    } // end need to build extended extent check

  // all done
  return(SmExtent1d(dMin, dMax)) ;

} // end SmExtent1d::ApproximateUnbounded

/*******************************************************************//**
PURPOSE: Equality operator 

NOTES: 
***********************************************************************/
SmBoolean SmExtent1d::operator==
  (const SmExtent1d& crOther) 
 const
{

  // tolerance check, tol = SM_EFF_ZERO * (1 + smos_Max(a,b))
  if (   SM_ARE_SAME(m_dMin, crOther.m_dMin)  && SM_ARE_SAME(m_dMax, crOther.m_dMax) )
    {
      return TRUE;
    }
  return FALSE;

} // end SmExtent1d::operator==

/*******************************************************************//**
PURPOSE: Determine if two intervals are disjoint on a periodic interval.

NOTES: They are not considered disjoint if they touch.
***********************************************************************/
SmBoolean SmExtent1d::AreDisjointPeriodic
  (const SmExtent1d & crOther,     // in : 2nd interval in same periodic space
   double dPeriod,                 // in : length of periodic space, 0 = not periodic
   double dTol)                    // in, opt: default: -1.
 const
{
  // check input
  SM_ASSERT_DEFINED(this);
  SM_ASSERT_DEFINED(&crOther);

  // intervals are disjoint when neither interval contains an other's endPoint
  SmBoolean bRet = (   !this->ContainsPeriodicValue( crOther.m_dMin, dPeriod, NULL, NULL, NULL, dTol )
                    && !this->ContainsPeriodicValue( crOther.m_dMax, dPeriod, NULL, NULL, NULL, dTol )
                    && !crOther.ContainsPeriodicValue( this->m_dMin, dPeriod, NULL, NULL, NULL, dTol )
                    && !crOther.ContainsPeriodicValue( this->m_dMax, dPeriod, NULL, NULL, NULL, dTol ))
                   ? TRUE 
                   : FALSE ;
  return bRet;

} // end SmExtent1d::AreDisjointPeriodic

/*******************************************************************//**
PURPOSE: This method assumes that the extent interval is periodic
    and it finds the corresponding value within the interval.  

NOTES: For example if the extent goes from 0 to 2, a value of 
    4.5 would return 0.5.  A value of -1.0 would return 1.

  This works to machine precision.  Any tolerance considerations
  should be decided before calling this.
***********************************************************************/
double SmExtent1d::PeriodicWrap
  (double dValue) 
 const
{
  SM_ASSERT_DEFINED(this);
  // this only works when dPeriod = m_dMax - m_dMin
  double dPeriod = m_dMax - m_dMin ;
 
  // a zero period is a bug in the calling program.
  // Check for the problem with Assert and return an input
  // value to prevent the system from getting stuck in an infinite loop. 
  SM_ASSERT_BREAK(dPeriod > SM_EFF_ZERO) ;
  if(dPeriod <= SM_EFF_ZERO)
    { return dValue ; } 

  // snap to endPoints to avoid tolerance problems
  if(smos_Fabs(dValue - m_dMin) < SM_EFF_ZERO) dValue = m_dMin ;
  if(smos_Fabs(dValue - m_dMax) < SM_EFF_ZERO) dValue = m_dMax ;

  // Check for overflow, for example, if SM_BIG_DOUBLE were passed in.
  // That would cause infinite loop, because subtracting dPeriod from
  // such a huge value does not change it at all: below machine precision.
  // Moreover, there's not much we can do about it: you would normally
  // calculate an integer ratio, and subtract an integral number of
  // periods, but SM_BIG_DOUBLE / 360 is way too big for a 32-bit integer.
  if ( dValue + dPeriod == dValue )
    {
      SM_ASSERT( FALSE );
      return dValue;
    }

  // push the value into the desired period
  // Since we snapped the original values, we have to snap each try as well. [B96]
  while ( dValue < m_dMin )
    {
      dValue = dValue + dPeriod ;
      if( smos_Fabs(dValue - m_dMin) < SM_EFF_ZERO ) dValue = m_dMin;
      if( smos_Fabs(dValue - m_dMax) < SM_EFF_ZERO ) dValue = m_dMax;
    }
  while (dValue > m_dMax)
    {
      dValue = dValue - dPeriod ;
      if( smos_Fabs(dValue - m_dMin) < SM_EFF_ZERO ) dValue = m_dMin;
      if( smos_Fabs(dValue - m_dMax) < SM_EFF_ZERO ) dValue = m_dMax;
    }
  return dValue;

} // end SmExtent1d::PeriodicWrap

/*******************************************************************//**
PURPOSE: Determine the intersection of two periodic intervals that 
            come from the same space, 
         e.g both are intervals of [-360 to +360], maxLength < 360.0, period = 360
         or  both are intervals in [0 1], period = 1.

NOTES: outputs are returned in a contiguous fashion if possible
     rather than in the 'this' or 'other' interval.

     ex:  [0 360] intersect [-90 90 ]  outputs:  [-90 90] rather than the set {[270 360], [0 90]}   

  rResult.GetSize():  0 = disjoint
                      1 = typical intersection - one interval overlaps the other
                      2 = periodic intersection - intervals are long enough that 
                           they wrap around the period so that their ends overlap twice

   Tolerances are all SM_EFF_ZERO in this method.

METHOD ---
 1. Consider period [ThisMin, ThisMin + dPeriod]
 2. Map ThisMax,
        OtherMin, and 
        OtherMax into target period.
 3. Consider cases
    a.  OtherMin  < OtherMax - (at most 2 intervals total)
                               output = 1 nonPeriodic xSects of 
                                        [ThisMin ThisMapMax]/[OtherMapMin OtherMapMax] (at most 1 interval)
                               output = 1 degenerate point at m_dMin (at most 1 interval)
                              
    b.  OtherMin  > OtherMax - (at most 2 intervals total)
                               output = 2 nonPeriodic xSects of
                                        [ThisMin ThisMapMax]/[ThisMin OtherMapMax] (at most 1 interval)
                                        [ThisMin ThisMapMax]/[OtherMapMin ThisMax] (at most 1 interval)
                               
    c.  OtherMin == OtherMax && OtherLength == 0.0     (otherInterval is degenerate)
                             - output = [ThisMin ThisMapMax]/[OtherMin OtherMin]   (at most 1 degenerate interval)

    d.  OtherMin == OtherMax && OtherLength == dPeriod (otherInterval is closed)
                             - output = [ThisMin ThisMapMax]                       (always 1 interval equal to this interval)

NOTE -- one can visualize all the cases using permutations of the following figure.

   +ThisMin                          +ThisMax   +Period
   |---+---------------+-------------+----------|
       +OtherMin/Max   +OtherMax/Min

***********************************************************************/
SmStatus SmExtent1d::IntersectPeriodic
  (const SmExtent1d     & crOther,   // in : other interval to intersect
   double                 dPeriod,   // in : length of periodic space, 0 = treat intervals as not periodic
   SmTArray<SmExtent1d> & rResult,   // out: intersection interval or
                                     //      NULL interval
   SmBoolean bDontCrossBoundaries)   // in : TRUE = Split return intervals that span this interval boundaries
                                     //        ex: Intersect([0, 360] [-90 90]) returns [270 0] [0 90] 
                                     //      FALSE= Okay to return a single interval that spans this interval boundary as
                                     //        ex: Intersect([0, 360] [-90 90]) returns [-90 90]
                                     //      default:[FALSE]
  const                           
{
  SM_ASSERT_DEFINED(this);
  SM_ASSERT_DEFINED(&crOther);

  // init output
  SmStatus sRet = SM_SUCCESS;
  rResult.ReSet() ;

  // pass not periodic cases along
  if(dPeriod == 0.0) { SmExtent1d sResult ;
                       sRet = Intersect(crOther, sResult) ; 
                       rResult.Add(sResult) ;     
                       return(sRet) ;
                     }

  // 1. get interval to consider
  SmExtent1d sIvl(m_dMin, m_dMin + dPeriod) ;

  // 2. map all boundaries into this interval
  double dThisMin  = m_dMin ;
  double dThisMax  = m_dMax ; SM_ASSERT_BREAK(SM_ARE_SAME(m_dMax, sIvl.PeriodicWrap(m_dMax))) ;
  double dOtherMin = sIvl.PeriodicWrap(crOther.m_dMin) ;
  double dOtherMax = sIvl.PeriodicWrap(crOther.m_dMax) ;

  // 3. switch on cases
  if     (dOtherMin < dOtherMax - SM_EFF_ZERO)
    {
      // when mapped intervals intersect
      if(   (dThisMin <= dOtherMax + SM_EFF_ZERO)
         && (dThisMax >= dOtherMin - SM_EFF_ZERO))
        {
          // get interval endPoints
          double dMin = smos_Max(dThisMin, dOtherMin) ;
          double dMax = smos_Min(dThisMax, dOtherMax) ;

          // sanp degenerate segments
          if(SM_ARE_SAME(dMin,dMax)) dMax = dMin ;

          // add intersection to result
          rResult.Add(SmExtent1d(dMin, dMax)) ;
        }

      // when otherMax touches an open thisMin add a degenerate interval output
      if(   !IsClosed(dPeriod)                          // this is open
         && !crOther.IsClosed(dPeriod)                  // other is open
         && SM_ARE_SAME(dOtherMax, dThisMin + dPeriod)) // and otherMax is on the upper boundary
        { // add a degenerate interval at dThisMin
          rResult.Add(SmExtent1d(dThisMin, dThisMin)) ;
        }
    } // end 'just like nonPeriodic case' branch

  else if(dOtherMin > dOtherMax + SM_EFF_ZERO)
    {
      SmBoolean bThisClosed = IsClosed(dPeriod) ;

      // when this is closed and its ok - just return the other interval
      if(bThisClosed && bDontCrossBoundaries == FALSE) 
        { 
          // output one interval = OtherInterval
          rResult.Add(crOther) ; 
        }
      else // check for two distinct outputs
        {
          // intersect [ThisMin ThisMapMax]/[ThisMin OtherMapMax]
          if(dThisMin <= dOtherMax + SM_EFF_ZERO)
            {
              if(dThisMin > dOtherMax) { dOtherMax = dThisMin ; }
               
              // skip degenerate intervals on closed curves
              //  that point gets included in the other interval
              if(! (bThisClosed && SM_ARE_SAME(dThisMin, dOtherMax)))
                {
                  // add intersection to result
                  rResult.Add(SmExtent1d(dThisMin,
                                         smos_Min(dThisMax, dOtherMax))) ;
                }
            }

          // intersect [ThisMin ThisMapMax]/[OtherMapMin ThisMax]
          if(dThisMax >= dOtherMin - SM_EFF_ZERO)           
            {
              if(dThisMax < dOtherMin) { dOtherMin = dThisMax ; }

              // skip degenerate intervals on closed curves
              //  that point gets included in the other interval
              if(! (bThisClosed && SM_ARE_SAME(dThisMax, dOtherMin)))
                {
                  // add intersection to result
                  rResult.Add(SmExtent1d(smos_Max(dThisMin,dOtherMin),
                                         dThisMax)) ;
                }
            }
        } // end this not Closed branch
    } // end possibly two wrapped intervals

  else // dOtherMin == dOtherMax (to SM_EFF_ZERO): either complete period or degenerate.
    {
      if ( crOther.IsClosed( dPeriod ) )  // other is full period
        {
          if ( bDontCrossBoundaries )
            {
              // Other Start/End are the same.
              // If that Start/End is within 'this', split 'this' there,
              // else Other contains all of 'this'.
              // Note: dOtherMin is the result of PeriodicWrap(), so just check ContainsValue().
              // Note: negative Tolerance: on-boundary is considered Out.
              if ( this->ContainsValue( dOtherMin, -SM_EFF_ZERO ) )
                {
                  rResult.Add( SmExtent1d( dThisMin, dOtherMax ) );
                  rResult.Add( SmExtent1d( dOtherMin, dThisMax ) );
                }
              else
                { rResult.Add(*this); }  // Other completely contains 'this'.
            }
          else
            { rResult.Add(*this); }  // Ok to cross our boundaries
        }
      else // other is degenerate
        {
          if(dOtherMin <= dThisMax)
            { rResult.Add(SmExtent1d(dOtherMin, dOtherMin)) ; }
        }
    }

  // all done
  return sRet;

} // end SmExtent1d::IntersectPeriodic

/*******************************************************************//**
PURPOSE: When periodic value is out of interval, snap to nearest end point,
            else return periodic value within the interval. 

NOTES: 
***********************************************************************/
double SmExtent1d::ClampPeriodicValue
  (double dValue,          // in : target value 
   double dPeriod,         // in : length of periodic space, 0 = not periodic
   double dTol )           // in, opt : default -1.
 const
{
  SM_ASSERT_DEFINED(this);
  // get PeriodicValue in period immediately above m_dMin
  double dPeriodicValue, dDistToMin, dDistToMax ;
  SmBoolean bInside = ContainsPeriodicValue(dValue, dPeriod,
                                            &dPeriodicValue,
                                            &dDistToMin, 
                                            &dDistToMax,
                                            dTol );
  
  // when outside the interval
  if(bInside == FALSE)
    {
      // snap to closest periodic boundary
      dPeriodicValue =  (dDistToMin < dDistToMax)
                       ? m_dMin
                       : m_dMax ;
    
    } // end need to snap value 

  // return the clamped value
  return(dPeriodicValue) ;

} // end SmExtent1d::ClampPeriodicValue

/*******************************************************************//**
PURPOSE: Determine if the interval of the extent contains a given value

NOTES: when the value, after accounting for periodicity,
   lies within tolerance of this extent's range,
   then TRUE is returned.
***********************************************************************/
SmBoolean SmExtent1d::ContainsPeriodicValue
  (double  dValue,          // in : target value 
   double  dPeriod,         // in : length of periodic space, 0 = not periodic
   double *dPeriodicValue,  // out: Best "Contained" periodic value with
                            //      no snapping, NULL to ignore
   double *dDistToMin,      // out: Min dist from PeriodicValue to MinBoundary, NULL to ignore
   double *dDistToMax,      // out: Min dist from PeriodicValue to MaxBoundary, NULL to ignore
   double  dTol )           // in, opt: default: -1.0 which then uses SM_EFF_ZERO_DEG
 const
{
  SM_ASSERT_DEFINED(this);
  SmBoolean bRtn ;
  
  // locals (GWC: increased tolerance to account for common degree domains ;
  double dScaledZero = ( dTol > 0.0 ) ? dTol : SM_EFF_ZERO_DEG;

  bRtn = ContainsValue(dValue, dScaledZero) ;

  // catch nonPeriodic and 'value within this period' cases
  if( bRtn || dPeriod == 0) { if(dPeriodicValue) *dPeriodicValue = dValue ;
                              if(dDistToMin)     *dDistToMin     = smos_Fabs(dValue - m_dMin) ;
                              if(dDistToMax)     *dDistToMax     = smos_Fabs(dValue - m_dMax) ;
                              return(bRtn) ;
                            }

  // get interval [dMin, dMin+Period]
  SmExtent1d sIvl(m_dMin, m_dMin + dPeriod) ;

  // Map value to this interval.
  // But PeriodicWrap() works to machine precision,
  // so don't do it if the value is within tol of the base interval.
  // [091001]
  double dMapValue = dValue;
  if ( ! sIvl.ContainsValue( dValue, dScaledZero ) )
    {
      dMapValue = sIvl.PeriodicWrap( dValue );
    }

  // check inside or out with tolerances
  bRtn = (   dMapValue > m_dMin - dScaledZero
          && dMapValue < m_dMax + dScaledZero);

  // If our interval is smaller than dPeriod, and dMapValue is very near
  // the top of the period (sIvl), it should be at the start of the interval.
  if ( ! bRtn )
    {
      if ( SM_ARE_SAME_TO_TOL( dMapValue, sIvl.GetMax(), dScaledZero ) )
        {
          if ( SM_ARE_SAME_TO_TOL( dMapValue-dPeriod, m_dMin, dScaledZero ) )
            {
              dMapValue = m_dMin;
              bRtn = TRUE;
            }
        }
    }

  // set optional outputs
  if(dPeriodicValue) { // when INPUT  value is close enough to a boundary value - report it 
                       // when MAPPED value is just below min_Value - report small negative number
                       // else just report the MAPPED value  
                       if(bRtn && (   (smos_Fabs(dValue - m_dMin) < dScaledZero) 
                                   || (smos_Fabs(dValue - m_dMax) < dScaledZero)))
                         {  *dPeriodicValue = dValue ; }
                       else if(bRtn && dMapValue > m_dMax + dScaledZero)
                         {  *dPeriodicValue = dMapValue - dPeriod ; }
                       else 
                         {  *dPeriodicValue = dMapValue ; }
                     }
  if(dDistToMin)     { *dDistToMin     = dMapValue - m_dMin ;
                       if(*dDistToMin > dPeriod/2.0) 
                         { *dDistToMin = dPeriod - *dDistToMin ; }
                     }
  if(dDistToMax)     { *dDistToMax     = smos_Fabs(m_dMax - dMapValue) ;
                       if(*dDistToMax > dPeriod/2.0) 
                         { *dDistToMax = dPeriod - *dDistToMax ; }
                     }
  // all done
  return( bRtn ) ;

} // end SmExtent1d::ContainsPeriodicValue

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
SmBoolean SmExtent1d::IsClosed
  (double dPeriod)  // in : length of periodic space, 0 = not periodic
 const
{
  SM_ASSERT_DEFINED(this);
  return(   dPeriod != 0.0
         && SM_ARE_SAME(m_dMax - m_dMin, dPeriod)) ;

} // end ::IsClosed
  
/*******************************************************************//**
PURPOSE: Determine if the value is on the boundary to within the given
            tolerance value accounting for periodicity.

NOTES: 
***********************************************************************/
SmBoolean SmExtent1d::IsValueOnPeriodicBoundary
  (double dValue,   // in : target value 
   double dPeriod,  // in : length of periodic space, 0 = not periodic
   double dTol,     // in : max allowed separation for a hit.  Default SM_EFF_ZERO.
   double *pOptDist)// out: min distance from point to periodic boundary
                    //      NULL to ignore, default:[NULL]
  const
{
  SM_ASSERT_DEFINED(this);

  // figure out where the value is in relation to the boundaries
  double dPeriodicValue, dDistToMin, dDistToMax ;
  ContainsPeriodicValue(dValue, dPeriod, &dPeriodicValue, &dDistToMin, &dDistToMax, dTol );
  
  // set output
  SmBoolean bRtn = (   dDistToMin < dTol
                    || dDistToMax < dTol) ;
  if(pOptDist) 
    { 
      *pOptDist = smos_Min(dDistToMin, dDistToMax) ; 
    }

  // all done
  return(bRtn) ;                  

} // end SmExtent1d::IsValueOnPeriodicBoundary

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
void SmExtent1d::Dump(void) const
{
  TCHAR sBuff[SM_TBLOCK_SIZE];
  SM_ASSERT_DEFINED(this);

  SM_SPRINTF(sBuff,_T(" [%16.16lf -> %16.16lf] "),m_dMin,m_dMax);
  smos_WriteBuffer(sBuff);

} // end SmExtent1d::Dump


