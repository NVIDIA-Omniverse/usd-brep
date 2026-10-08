// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************//**
* FILE NAME --- SmPeriodicExtent1d.cpp
* PURPOSE: Implementation of SmPeriodicExtent1d methods.
**********************************************************************/

#include "StdAfx.h"

#include <SmPeriodicExtent1d.h>
#include <SmAssertArray.h>

/*******************************************************************//**
PURPOSE: Constructor which takes a period to construct an extent that 
            covers the full period.

NOTES: 
***********************************************************************/
SmPeriodicExtent1d::SmPeriodicExtent1d
 (double dPeriod)  // in : range of periodic interval:[0 dPeriod]
{ 
  m_dMin        = 0.0; 
  m_dMax        = dPeriod; 
  m_dPeriod     = dPeriod;
  m_bFullPeriod = TRUE;

} // end SmPeriodicExtent1d::SmPeriodicExtent1d

/*******************************************************************//**
PURPOSE: Constructor which takes a minimum and maximum value.

NOTES: 
  Seams are always on 0 and m_dPeriod
  When m_dMin < m_dMax, Exclude Seam, Ivl:[Min->Max], Length = m_dMax - m_dMin
  When m_dMax < m_dMin, Include Seam, Ivl:[Min->Seam Seam->Max], Length = m_dMax - m_dMin + m_dPeriod

  The given period must be positive and nonZero
  The given boundary values must be within the interval [0.0->dPeriod]
  GWC?: this can't be true - "Only dMin can be 0.0, and only dMax can be dPeriod."
        Otherwise a point Ivl on a seam can't be built.
***********************************************************************/
SmPeriodicExtent1d::SmPeriodicExtent1d
 (double dMin,    // in : must be in range:[0 dPeriod]
  double dMax,    // in : must be in range:[0 dPeriod]
  double dPeriod) // in : defines range:[0 dPeriod]
{ 
  // period must be positive
  if(dPeriod < 0.0) 
    { SE(SM_ERR); }

  // bounds must be in range:[0 period]
  if(   dMin < 0.0 || dMin > dPeriod 
     || dMax < 0.0 || dMax > dPeriod ) 
    { SE(SM_ERR); }

  SmScaledZero sScaledZero = SmTol::GetScaledZero(dPeriod) ;

  m_dMin        = dMin; 
  m_dMax        = dMax; 
  m_dPeriod     = dPeriod;
  m_bFullPeriod = (smos_Fabs(m_dPeriod - (m_dMax - m_dMin)) < sScaledZero) ;
  if(m_bFullPeriod)
    {
      m_dMin = 0.0 ;
      m_dMax = m_dPeriod ;
    }

} // end SmPeriodicExtent1d::SmPeriodicExtent1d

/*******************************************************************//**
PURPOSE: Copy constructor.  It initializes the values from
    from an existing SmPeriodicExtent1d.

NOTES: 
***********************************************************************/
SmPeriodicExtent1d::SmPeriodicExtent1d
 (const SmPeriodicExtent1d & crOriginal)
{
  m_dMin        = crOriginal.m_dMin;
  m_dMax        = crOriginal.m_dMax;
  m_dPeriod     = crOriginal.m_dPeriod;
  m_bFullPeriod = crOriginal.m_bFullPeriod;

} // end SmPeriodicExtent1d::SmPeriodicExtent1d

/*******************************************************************//**
PURPOSE: Convert the extent to refer to the opposite reference. 
         Preserves Ivl length. An original interval based on an origin at 0
         running in the positive direction will look like an
         interval based on an origin at Period running in the negative direction. 

NOTES:       |----------------->OrigMax
             |---->OrigMin       |
       in : [0------|************|----Period]
       out: [0----|************|------Period]
                  |       OrigMin<----|
             OrigMax<----------+------|
              NewMin       NewMax

  This can be used, if 2 extents are constructed with opposite
  reference directions, and they need to converted to the 
  same reference (for intersection, etc);
***********************************************************************/
void SmPeriodicExtent1d::Invert()
{
  // no work - full or empty extent
  if(m_bFullPeriod || IsEmpty()) 
    { return; }
                             // in : [ Min  Max]              ex: [.2 .6]  period = 1.0 
  // negate values           // out: [-Min -Max]                  [-.2 -.6]
  m_dMin = -m_dMin;          // out: [Norm(-Min) Norm(-Max)]      [.8 .4]
  m_dMax = -m_dMax;          // out: Swap=[Norm(-Max) Norm(-Min)] [.4 .8]

  // Map value to periodic interval:[0.0 m_dPeriod]
  m_dMin = NormalizeValue(m_dMin);
  m_dMax = NormalizeValue(m_dMax);

  // switch values
  SM_SWAP(double, m_dMin, m_dMax);

  // remember full extents
  SmScaledZero sScaledZero = SmTol::GetScaledZero(m_dPeriod) ;
  m_bFullPeriod = (smos_Fabs(m_dPeriod - (m_dMax - m_dMin)) < sScaledZero) ;

  // adjust This PeriodicExtent to fit within range:[0 period]
  Normalize();

} // end SmPeriodicExtent1d::Invert

/*******************************************************************//**
PURPOSE: Create the compliment periodic interval

NOTES: given   0------|************|----Period 
       return  0******|------------|****Period

       given   0******|------------|****Period
       return  0------|************|----Period 

       given   0-------------||---------Period  - a point interval
       return  full period

       given   full period
       return  a point interval                 - what point to use?  pick mid point
***********************************************************************/
void SmPeriodicExtent1d::Compliment()
{
  // empty set
  if(IsEmpty())
    {
      m_dMin = 0.0 ;
      m_dMax = m_dPeriod ;
      m_bFullPeriod = TRUE ;
      return ;
    }

  // full period
  if(m_bFullPeriod) 
    { // Given Full Period, Return empty set
      m_bFullPeriod = FALSE ;                                                
      m_dMin        =  SM_BIG_DOUBLE ;
      m_dMax        = -SM_BIG_DOUBLE ;
    }
  else if( SM_ARE_SAME_TO_TOL(m_dMax, m_dMin, SmTol::GetScaledZero(m_dPeriod)))
    { // Given Point Interval, Return Full Interval
      m_bFullPeriod = TRUE ;
      m_dMin        = 0 ;
      m_dMax        = m_dPeriod ;
    }
  else
    {
      SM_SWAP(double, m_dMin, m_dMax) ; 
    }

} // end SmPeriodicExtent1d::CreateCompliment

/*******************************************************************//**
PURPOSE: Given a value, extend the interval of the extent to cover
     the value if necessary.  If the value is inside of the interval
     then there is no effect on the extent.

     The flag indicates if the extent is to be be expanded forward,
     i.e. by increasing the upper end, or backward, i.e decreasing
     the lower end.

NOTES: Do not add value when within ScaledZero of current interval

      if OrigMin < OrigMax and NewValue changes that to NewMin > NewMax,
      then an interval which originally did not contain the seam
      has been expanded past the seam and on so that the newInterval includes
      the seam.
***********************************************************************/
void SmPeriodicExtent1d::AddValue
 (double    dValue,    // in : new value in range:[0 m_dPeriod]
  SmBoolean bForward)  // in : TRUE = set Max value = dValue
                       //      FALSE= set Min value = dValue
{ 
  // no work - already in interval
  if(ContainsValue(dValue))
    { return; }

  // low work - emptry set
  if(IsEmpty())
    {
      m_dMin = dValue ;
      m_dMax = dValue ;
      m_bFullPeriod = FALSE ;
      return ;
    }

  SmScaledZero sScaledZero = SmTol::GetScaledZero(m_dPeriod) ;

  // bForward: TRUE = set m_dMax, FALSE = set m_dMin
  if (bForward) { m_dMax = dValue; 
                  if(SM_IS_ZERO_TO_TOL(m_dMax, sScaledZero)) // gwc changed from: (m_dMax == 0.0)
                      m_dMax = m_dPeriod;
                }
  else          { m_dMin = dValue; 
                  if(SM_IS_ZERO_TO_TOL(m_dMin - m_dPeriod, sScaledZero)) // gwc changed from: (m_dMin == m_dPeriod)
                      m_dMax = 0.0;
                }

  // check for Ivls extended into Full Periods
  m_bFullPeriod = smos_Fabs(m_dPeriod - (m_dMax - m_dMin)) < sScaledZero ;

  if(m_bFullPeriod)
    {
      m_dMin = 0.0 ;
      m_dMax = m_dPeriod ;
    }

} // end SmPeriodicExtent1d::AddValue

/*******************************************************************//**
PURPOSE: Extend current Extent to include the given span.

NOTES: 0. RETURNS SM_ERR when dFrom or dTo are not within the Extent's period.
       1. RETURNS SM_ERR when input span is adjoint to current extent.
            SmPeriodicExtent1d cannot represent a disjoint span.
       2. To represent a Point,        pass interval:[Param Param]
       3. To represent a FullInterval, pass interval:[0 Period]

METHOD: There are 48 combinations between Extent min and max values and
        the interval To and From values after checking for Full Extents and Full Intervals.

        That reduces to 24 combinations after swapping the input 
        so that bForward is always true. Those combinations can be
        characterized as:
        case 0) Min < Max  && From < To, 6 cases (min->max no seam)      (from->to no seam)
        case 1) Min < Max  && From > To, 6 cases (min->max no seam)      (from->to crosses seam)
        case 2) Min > Max  && From < To, 6 cases (min->max crosses seam) (from->to no seam)
        case 3) Min > Max  && From > To, 6 cases (min->max crosses seam) (from->to crosses seam)

        That further reduces to 18 combinations after turning case 2 
        into case 1 by swapping Min <=> To and Max <=> From.

        I looked for, but could not find, a general rule that worked for
        all the cases. So the following is implemented as a bunch
        of conditionals.

***********************************************************************/
SmStatus SmPeriodicExtent1d::AddInterval
 (double    dFrom,     // in : from of interval:[from to]  if  bForward: |   from-->to   | or |-->to   from-->|
  double    dTo,       // in : to   of interval:[from to]  if !bForward: |<--from   to<--| or |   to<--from   |
  SmBoolean bForward)  // in : TRUE = move to the right from dFrom to dTo crossing seams if necessary.
                       //      FALSE= move to the Left from dFrom to dTo crossing seams if necessary.
{
  // check input
  if(   !SM_IS_CONTAINED(dFrom, 0.0, m_dPeriod)
     || !SM_IS_CONTAINED(dTo,   0.0, m_dPeriod))
    { return(SM_ERR) ; }

  // when bForward is FALSE, swap [From To] to make all cases bForward == TRUE
  if(bForward == FALSE) 
    { SM_SWAP(double, dTo, dFrom) ; } 

  // locals
  SmScaledZero sScaledZero = SmTol::GetScaledZero(m_dPeriod) ;
  SmBoolean    bSwap       = FALSE ;  // remember switching case 2 into case 1

  // check for Full Period intervals
  if(   IsFullPeriod()
     || (   SM_IS_ZERO_TO_TOL(dFrom, sScaledZero) 
         && SM_IS_ZERO_TO_TOL(dTo - m_dPeriod, sScaledZero)) )
    {
      m_dMin        = 0.0 ;
      m_dMax        = m_dPeriod ;
      m_bFullPeriod = TRUE ;
      return(SM_SUCCESS) ;
    }

  // empty sets
  if(IsEmpty())
    { m_bFullPeriod = (m_dPeriod - smos_Fabs(m_dMax - m_dMin)) < sScaledZero ;
      if(m_bFullPeriod) { m_dMin = 0.0 ;
                          m_dMax = m_dPeriod ;
                        }
      else              { m_dMin = dFrom ;
                          m_dMax = dTo ;
                        }
      return(SM_SUCCESS) ;
    }

  // turn With/Without Seam properties into a number for upcoming switch statement
  ULONG lCaseId =   ((m_dMin <= m_dMax) ? 0 : 2)   // case 0) Min < Max  && From < To
                  + ((dFrom  <= dTo)    ? 0 : 1) ; // case 1) Min < Max  && From > To
                                                   // case 2) Min > Max  && From < To
                                                   // case 3) Min > Max  && From > To
  // map case 2 into 1
  if(lCaseId == 2) { lCaseId = 1 ;
                     SM_SWAP(double, m_dMin, dFrom) ;
                     SM_SWAP(double, m_dMax, dTo) ;
                     bSwap = TRUE ; 
                   }

  // switch on remaining 3 cases
  switch(lCaseId)
    {
      case 0 : // case 0) Min < Max  && From < To             
               //         extent   =  |   min-->max   |  or  |   M-->X   |     M = m_dMin, X = m_dMax    
               //         interval =  |   from-->to   |      |   F-->T   |     F = dFrom , T = dTo

               // disjoint cases                              // |       M--->X       | extent - no seam
               if(   (dTo   < m_dMin - sScaledZero)           // |  F->T              | interval cases
                  || (dFrom > m_dMax + sScaledZero))          // |              F->T  |        - no seam
                 { return(SM_ERR) ; }                       
               if(dFrom < m_dMin) { m_dMin = dFrom ; }        // |  F------>T         | 
               if(dTo   > m_dMax) { m_dMax = dTo ; }          // |  F------------->T  |                      
               break ;                                        // |        F->T        |
                                                              // |        F------->T  |
                                                                                      
      case 1 : // case 1) Min < Max  && From > To             
               //         extent   =  |   min-->max   |  or  |   M-->X   |     
               //         interval =  |-->to   from-->|      |-->T   F-->|
                                                              
               // disjoint case                                  // |       M--->X       | extent - no seam                          
               if(   (dFrom > m_dMax + sScaledZero)              // |->T              F->| interval cases                       
                  && (dTo   < m_dMin - sScaledZero))             //                               - with seam
                 { if(bSwap) { m_dMin = dFrom ; m_dMax = dTo ; }      
                   return(SM_ERR) ; 
                 }                                 
               // Full case                                           
               if(   (dFrom < m_dMax + sScaledZero)                     
                  && (dTo   > m_dMin - sScaledZero))             // |------->T  F------->|
                 { m_dMin        = 0.0 ;
                   m_dMax        = m_dPeriod ;
                   m_bFullPeriod = TRUE ;                  
                   return(SM_SUCCESS) ;                    
                 }                                         
                                                                 // |       M--->X       | extent - no seam
               if(dTo < m_dMin)                                  // |->T       F-------->| interval cases
                    { if(dFrom < m_dMin) { m_dMin = dFrom ; }    // |->T  F------------->|        - with seam
                      m_dMax = dTo ;                             
                    }                                            
               else { m_dMin = dFrom ;                           // |------->T        F->|
                      if(dTo   > m_dMax) { m_dMax = dTo ; }      // |------------->T  F->|
                    }
               break ;

      case 3 : // case 3) Min > Max  && From > To                
               //         extent   =  |-->max  min--->|          // |------>X    M------>| extent - with seam
               //         interval =  |-->to   from-->|          // |->T              F->| interval cases
                                                                 // |->T       F-------->|        - with seam
               if(dFrom < m_dMin) { m_dMin = dFrom ; }           // |------->T        F->|       
               if(dTo   > m_dMax) { m_dMax = dTo ; }             // |------->T  F------->|   
                
               if(m_dMin < m_dMax - sScaledZero) { // Full case  // |------------->T  F->|  
                                     m_dMin        = 0.0 ;       // |->T  F------------->|  
                                     m_dMax        = m_dPeriod ;   
                                     m_bFullPeriod = TRUE ;        
                                     return(SM_SUCCESS) ;          
                                   }                                      
               break ;                                                    

      default: SM_ASSERT_MSG(FALSE, _T("SmPeriodicExtent1d::AddInterval - implementation error - needs debug")) ; 
    } // end switch on Extent/Interval cases

  // all done
  return(SM_SUCCESS) ;

} // end SmPeriodicExtent1d::AddValue

/*******************************************************************//**
PURPOSE: Determine if the interval of the extent contains a given
     value.  If the value is equal to one of the boundaries of the extent
     then TRUE is returned.

NOTES: Contains if matches endpoints by numeric error
***********************************************************************/
SmBoolean SmPeriodicExtent1d::ContainsValue
 (double dValue)   // in : target value to check
 const
{ 
  // low work - Full Period, all points are in
  if(IsFullPeriod()) 
   { return TRUE; }

  // low work - empty set, no points are in
  if(IsEmpty())
    { return FALSE ; }

  SmScaledZero sScaledZero = SmTol::GetScaledZero(m_dPeriod) ;

  // cases: Pt Ivl        : TRUE = Val == m_dMin || Val == m_dMax   to sScaledZero
  //        no seam Ivl   : TRUE = Val in Ivl  [Min-Max]            to sScaledZero
  //        cross seam Ivl: TRUE = Val in OneOf[Min-Period, 0-Max]  to sScaledZero
  if(smos_Fabs(m_dMin - m_dMax) < sScaledZero) { return((smos_Fabs(m_dMin - dValue) < sScaledZero) || (smos_Fabs(m_dMax - dValue) < sScaledZero) ) ; }       // Pt Ivl        
  if(m_dMin < m_dMax)                          { return( (dValue > m_dMin - sScaledZero) && (dValue < m_dMax + sScaledZero) ) ; } // no seam Ivl   
  else                                         { return( (dValue > m_dMin + sScaledZero) || (dValue < m_dMax + sScaledZero) ) ; } // cross seam Ivl

} // end SmPeriodicExtent1d::ContainsValue

/*******************************************************************//**
PURPOSE: Determine if the value is on the boundary to within the given
    tolerance value.

NOTES: Does not accept a tolerance that is less the numeric 
                instability of the extent.
***********************************************************************/
SmBoolean SmPeriodicExtent1d::IsValueOnBoundary
 (double dValue,   // in : target value to check
  double dTol1d)   // in : check tolerance
 const
{
  // low work - full and empty periods have no boundaries
  if(IsFullPeriod() || IsEmpty())
    { return FALSE; }

  // Limit smallest sized tolerances
  SmScaledZero sScaledZero = SmTol::GetScaledZero(m_dPeriod) ;
  if(dTol1d < sScaledZero)
    { dTol1d = sScaledZero; }

  // Check vanilla interval boundaries
  SmBoolean bOnStart = (smos_Fabs(dValue-m_dMin) < dTol1d) ;
  SmBoolean bOnEnd   = (smos_Fabs(dValue-m_dMax) < dTol1d) ;

  // check interval boundaries that terminate on the seam

  // If the argument is close to the seam, check if 'this' terminates at 
  // the same (the above misses out the m_dMin == 0 && dValue == m_dPeriod 
  // etc cases)

  // when Value is near the seam
  if(smos_Fabs(dValue) < dTol1d || smos_Fabs(m_dPeriod - dValue) < dTol1d ) 
    {

      // if either interval end is on the seam - return true

      if(m_dMin < dTol1d || smos_Fabs(m_dPeriod - m_dMin) < dTol1d)
        { bOnStart = TRUE ; }

      if(m_dMax < dTol1d || smos_Fabs(m_dPeriod - m_dMax) < dTol1d) 
        { bOnEnd = TRUE ; }
    }

  // Problem case: might warn! - nope we support point sized intervals
  // SM_WARN(bOnStart && bOnEnd);

  return(bOnStart || bOnEnd) ;

} // end SmPeriodicExtent1d::IsValueOnBoundary

/*******************************************************************//**
PURPOSE: Perform a normalized evaluation on the extent.  The normalized
   parameter is a value beteween 0.0 and 1.0 inclusive.  If the parameter
   is 0.0 then the minimum value of the extent is returned.  1.0 produces
   the maximum value of the extent and 0.5 produced the middle value of the
   extent.  Other parameters will produce other values from the extent.

NOTES: Prefers returning 0 to Period.
***********************************************************************/
double SmPeriodicExtent1d::Evaluate
 (double dNormalizedParameter)  // in : Normalized param in range:[0 1]
 const
{
  // check state
  SM_ASSERT( SM_IS_CONTAINED(dNormalizedParameter, 0.0, 1.0) ) ; 

  // low work - zero length ivls  
  if (GetLength() == 0.0)
    { return m_dMin; }
    
  // low work - empty sets
  if (IsEmpty())
    { return 0.0 ; }
    
  // adjust for Intervals that include or exclude the seam
  double dCorrection = (m_dMin < m_dMax) ? 0.0 : m_dPeriod;

  // map Normalized Param to Ivl param
  double dRet = (m_dMin + dNormalizedParameter * (m_dMax + dCorrection - m_dMin));

  // make sure dRet is in primary period
  dRet = NormalizeValue(dRet);

  // GWC this seems wrong? - map end Period pts to begin period pts
  if (dRet == m_dPeriod)
    { return 0.0 ; }
  return dRet;

} // end SmPeriodicExtent1d::Evaluate

/*******************************************************************//**
PURPOSE: Set the values for the interval of the extent.

NOTES: dMin and dMax bust be in the range:[0, dPeriod]
       When dMin  < dMax - the interval is [dMin, dMax]
       when dMax  < dMin - the interval is [dMin, dPeriod] and [0, dMax]
       when dMin == dMax - the interval is the full period [dMin, dPeriod] and [0, dMax]
***********************************************************************/
SmStatus SmPeriodicExtent1d::SetMinMax
 (double   dMin,       // in : Ivl low , When Min < Max, Exclude Seam, Ivl:[Min->Max],             Length = m_dMax - m_dMin 
  double   dMax,       // in : Ivl high, When Min > Max, Include Seam, Ivl:[Min->Seam, Seam->Max], Length = m_dMax - m_dMin + m_dPeriod  
  double  * pOptPeriod,      // in : Optional new Period value, NULL to ignore, default:[NULL]
  SmBoolean bSetSinglePoint) // in : see header file comment describing full period representation
                             //      TRUE = when dMin == dMax, interval set to a degenerate single point
                             //              by setting m_dMin = dMin
                             //                         m_dMax = dMax
                             //                         m_bFullPeriod = FALSE ;
                             //      FALSE= when dMin == dMax, interval set to FullPeriod
                             //              by setting m_dMin = 0.0
                             //                         m_dMax = pOptPeriod ? *pOptPeriod : m_dPeriod ;
                             //                         m_bFullPeriod = TRUE ;
                             //      default:[FALSE]
{
  // init output
  SmStatus sRet = SM_SUCCESS;

  // local
  double dPeriod = pOptPeriod ? *pOptPeriod : m_dPeriod ; 

  // check input
  SM_ASSERT_BREAK_MSG((IsInit() == TRUE) || (pOptPeriod != NULL), _T("SmPeriodicExtent1d::SetMinMax: input pOptPeriod required when Ivl is uninit.")) ; 
  SM_ASSERT_BREAK_MSG(   (dMin >= 0.0) && (dMin <= dPeriod)
                      && (dMax >= 0.0) && (dMax <= dPeriod),
                      _T("SmPeriodicExtent1d::SetMinMax: Bad input - Min and Max are not both within the extent's period")) ;

  // check input - bounds must be in range:[0 m_dPeriod]
  if(   dMin < 0.0 || dMin > dPeriod 
     || dMax < 0.0 || dMax > dPeriod ) 
    {
      SE(SM_ERR_INVALID_INPUT); 
      sRet = SM_ERR_INVALID_INPUT;
   }

  // tolerance
  SmScaledZero sScaledZero = SmTol::GetScaledZero(dPeriod) ;

  // period
  m_dPeriod = dPeriod ;

  // dMin == dMax
  if(smos_Fabs(dMax - dMin) < sScaledZero)
    { // degen pt inv
      if(bSetSinglePoint) { m_dMin = dMin ;
                            m_dMax = dMax ;
                            m_bFullPeriod = FALSE ;
                          }
      // Full period inv
      else                { m_dMin = 0.0 ;
                            m_dMax = dPeriod ;
                            m_bFullPeriod = TRUE ;
                          }
    } // end dMin == dMax branch

   // dMin != dMax
  else
    {
      m_dMin        = dMin; 
      m_dMax        = dMax; 
      m_bFullPeriod = (m_dPeriod - smos_Fabs(m_dMax - m_dMin)) < sScaledZero ;
    }

  // all done
  return sRet;

} // end SmPeriodicExtent1d::SetMinMax

/*******************************************************************//*******
PURPOSE: Determine if this interval is completely contained by the other.

NOTES: Equal intervals are contained in each other.
       The 2 intervals must have the same period;
****************************************************************************/
SmBoolean SmPeriodicExtent1d::IsContainedBy
 (const SmPeriodicExtent1d & crOther) // in : Extent to test
 const
{
  // tolerance
  SmScaledZero sScaledZero = SmTol::GetScaledZero(m_dPeriod) ;

  // check state
  SM_ASSERT_VALID_NO_STREAM(this);
  SM_ASSERT_VALID_NO_STREAM(&crOther);

  // check input
  SM_ASSERT(smos_Fabs(m_dPeriod - crOther.m_dPeriod) < sScaledZero);

  // low work - crOther is FullPeriod
  if(crOther.IsFullPeriod())
    { return TRUE ; }

  // low work - crOther is Empty
  if(crOther.IsEmpty())
    { return FALSE ; }

  // when this Ivl Crosses Seam
  if(CrossesSeam())
    { 
      // low work - only a SeamCrossing Ivl can contain another SeamCrossing Ivl
      // case ThisIsSeam OtherNoSeam
      if(crOther.CrossesSeam() == FALSE)
    { return FALSE ; }

      // A seam crossing Ivl contains another Seam crossing interval when its boundaries are contained
      // case ThisIsSeam OtherIsSeam
      return(   (m_dMax < crOther.m_dMax + sScaledZero)
             && (m_dMin > crOther.m_dMin - sScaledZero)) ;
    } // end this Ivl Crosses Seam branch

  // arrive here when this_Ivl does not cross seam

  if(crOther.CrossesSeam())
    {
      // this is contained when both its end points are in one end or the other of split crOther intervals
      // case ThisNoSeam OtherIsSeam
      return(   ((m_dMin < crOther.m_dMax + sScaledZero) && (m_dMax < crOther.m_dMax + sScaledZero))
             || ((m_dMin > crOther.m_dMin - sScaledZero) && (m_dMax > crOther.m_dMin - sScaledZero))) ;
    } // end crOther does cross Seam branch
  else // crOther does not cross seam; This Ivl is contained when its endPoints are contained
    {
      // case ThisNoSeam OtherNoSeam
      return(   crOther.ContainsValue(m_dMin)
             && crOther.ContainsValue(m_dMax)) ; 
    } // end crOther does not cross seam branch

  // touching in 2 points:
  //
  //                  *  *                        *.  *.          
  //             *             *             *.            *.    
  //         *                     *     *.                    *.
  //          .  complement        .    *.     coincident        *.
  //             .             . 
  //                  .  . 
  //            Not Contained By              Contained By

  // overlapping (large extents)
  //                  *  *  
  //             *             *
  //           *.               .*
  //           *.  overlap      .*
  //             .             .
  //              .          . 
  //                  .  . 
  //             Not Contained By

} // end SmPeriodicExtent1d::IsContainedBy

/*******************************************************************//**
PURPOSE: Determine if two intervals are disjoint on the periodic interval.

NOTES: They are not considered disjoint if they touch at just a point. 
       The periods must be the same.
***********************************************************************/
SmBoolean SmPeriodicExtent1d::AreDisjoint
 (const SmPeriodicExtent1d & crOther) // in : target extent to check
 const
{
  // check state
  SM_ASSERT_VALID_NO_STREAM(this);
  SM_ASSERT_VALID_NO_STREAM(&crOther);
  SM_ASSERT(m_dPeriod == crOther.m_dPeriod);

  // low work: this or crOther are FullPeriods
  if(   IsFullPeriod()
     || crOther.IsFullPeriod())
    { return FALSE ; }

  // low work: either interval is the NULL set
  if(IsEmpty() || crOther.IsEmpty())
    { return TRUE ; }

  // low work: either interval contains a boundary of the other
  if(   crOther.ContainsValue(m_dMin)
     || crOther.ContainsValue(m_dMax)
     || ContainsValue(crOther.m_dMin)
     || ContainsValue(crOther.m_dMax))
    { return FALSE ; }

  // The ends of this are off the other. What remains is the case
  // of this embracing the other:
  if(crOther.IsContainedBy(*this))
    { return FALSE ; }

  // all done - these are disjoint
  return TRUE;
            
} // end SmPeriodicExtent1d::AreDisjoint

/*******************************************************************//**
PURPOSE: Equality operator 

NOTES: 
***********************************************************************/
SmBoolean SmPeriodicExtent1d::operator==
  (const SmPeriodicExtent1d & crOther) 
 const
{
  // tolerance check, tol = SM_EFF_ZERO * (1 + smos_Max(a,b))
  return(   m_bFullPeriod == crOther.m_bFullPeriod
         && SM_ARE_SAME(m_dPeriod, crOther.m_dPeriod)
         && SM_ARE_SAME(m_dMin, crOther.m_dMin)
         && SM_ARE_SAME(m_dMax, crOther.m_dMax) ) ;

} // end SmPeriodicExtent1d::operator==

/*******************************************************************//**
PURPOSE: Perform a union operation between two extents.  This produces
    an interval whose maximum is the maximum of both maximums and whose 
    minimum is the mimimum of both minimums.

NOTES: RETURNS SM_ERR when intervals are disjoint, 
               sorry this class does not represent disjoint intervals.
***********************************************************************/
SmStatus SmPeriodicExtent1d::Union
 (const SmPeriodicExtent1d & crOther, 
  SmPeriodicExtent1d       & rResult) 
 const
{
  // check state
  SM_ASSERT_VALID_NO_STREAM(this);
  SM_ASSERT_VALID_NO_STREAM(&crOther);
  SM_ASSERT(m_dPeriod == crOther.m_dPeriod);

  // when either interval is empty - return the other
  if(IsEmpty())         { rResult = crOther ; return SM_SUCCESS ; }
  if(crOther.IsEmpty()) { rResult = *this ;   return SM_SUCCESS ; }

  // illegal case - sorry we don't represent two disjoint intervals in one Extent
  if(AreDisjoint(crOther)) { SE(SM_ERR) ; return SM_ERR ; }

  // when this is FullPeriod - return this
  if(IsFullPeriod()) { rResult = *this ; return SM_SUCCESS ; }

  // when crOther is FullPeriod - return crOther
  if(crOther.IsFullPeriod()) { rResult = crOther ; return SM_SUCCESS ; }

  // when this is contained by crOther - return crOther
  if(IsContainedBy(crOther)) { rResult = crOther ; return SM_SUCCESS ; }

  // when crOther is contained by this - return this
  if(crOther.IsContainedBy(*this)) { rResult = *this ; return SM_SUCCESS ; }

  // when extents are complimentary (coincident case already handled) - return FullPeriod
  if(   IsValueOnBoundary(crOther.m_dMin) 
     && IsValueOnBoundary(crOther.m_dMax))
   {
     rResult.m_dMin        = 0.0;
     rResult.m_dMax        = m_dPeriod;
     rResult.m_dPeriod     = m_dPeriod;
     rResult.m_bFullPeriod = TRUE ; 
     return SM_SUCCESS;        
   }
    
  // They are both finite and overlap. Try this first and see if it works:
  SmPeriodicExtent1d sCandidateRet(m_dMin, crOther.m_dMax, m_dPeriod);
  if(!sCandidateRet.ContainsValue(m_dMax)) 
    {
      sCandidateRet.SetMinMax(crOther.m_dMin, m_dMax) ;
    }

  // set output
  rResult = sCandidateRet ;

  // all done
  SM_ASSERT(   this->IsContainedBy(sCandidateRet)
            && crOther.IsContainedBy(sCandidateRet)) ;
  return SM_SUCCESS;


} // end SmPeriodicExtent1d::Union

/*******************************************************************//**
PURPOSE: Intersect two periodic intervals.

NOTES: 1. The input argument interval periods must be the same. 
       2. when the result is:
            - the NULL set: rResult and rResult2 are undefined
            - 1 result    : rResult contains one solution, rResult2 is undefined
            - 2 results   : rResult contains one solution, rResult2 the other
***********************************************************************/
SmStatus SmPeriodicExtent1d::Intersect
 (const SmPeriodicExtent1d         & crOther,      // in : Other PeriodicExtent to intersect
  ULONG                            & rlResultCnt,  // out: 0 = Empty set, rResult undefine, rResult2 undefined
                                                   //      1 = rResult contains 1st result, rResult2 undefined
                                                   //      2 = rResult contains 1st result, rResult2 the 2nd
  SmPeriodicExtent1d               & rResult,      // out: 1st result, or undefined for NULL set outputs
  SmPeriodicExtent1d               & rResult2,     // out:
  SmPeriodicExtentIntersectionType & rbResultType) // out: oneof: SM_PI_DISJOINT,      // no           XSect
 const                                             //             SM_PI_REGION,        // 1 Ivl        XSect
                                                   //             SM_PI_POINT,         // 1 Pt         XSect
                                                   //             SM_PI_2POINTS,       // 2 Pts        XSect
                                                   //             SM_PI_2REGIONS,      // 2 Ivls       XSect
                                                   //             SM_PI_REGION_POINT   // 1 Ivl & 1 pt XSect
{
  // check state
  SM_ASSERT_VALID_NO_STREAM(this);
  SM_ASSERT_VALID_NO_STREAM(&crOther);

  // check input
  SM_ASSERT(m_dPeriod == crOther.m_dPeriod);

  // init output
  rlResultCnt = 1;
  rResult. SetEmpty() ;
  rResult2.SetEmpty() ;
  rbResultType = SM_PI_REGION;

  // low work - disjoint
  if (AreDisjoint(crOther)) { rlResultCnt = 0;
                              rbResultType = SM_PI_DISJOINT;
                              return SM_SUCCESS;
                            }

  // low work - this is FullPeriod - return crOther
  if(IsFullPeriod() && crOther.IsContainedBy(*this)) { rlResultCnt = 1 ;
                                                       rResult = crOther;
                                                       return SM_SUCCESS;
                                                     }

  // low work - crOther is FullPeriod - return this
  if(crOther.IsFullPeriod() && IsContainedBy(crOther)) { rlResultCnt = 1 ;
                                                         rResult = *this;
                                                         return SM_SUCCESS;
                                                       }

  // low work - this contained by crOther - return this
  if(IsContainedBy(crOther)) { rlResultCnt = 1 ;
                               rResult = *this;
                               return SM_SUCCESS;
                             }

  // low work - crOther contained by this - return crOther
  if(crOther.IsContainedBy(*this)) { rlResultCnt = 1 ;
                                     rResult = crOther;
                                     return SM_SUCCESS;
                                   }

  // Full containment has already be excluded. See if they have just 2 
  // common points:
  if(IsValueOnBoundary(crOther.m_dMin) && IsValueOnBoundary(crOther.m_dMax))
    {
      rResult  = *this; // the startpoint of rResult will represent the first point.
      rResult2 = *this; // the startpoint of rResult2 will represent the other point.
      rResult.m_dMax  = rResult.m_dMin; 
      rResult2.m_dMin = rResult2.m_dMax; 
      rResult.Normalize();
      rResult2.Normalize();
      rlResultCnt = 2;
      rbResultType = SM_PI_2POINTS;
      return SM_SUCCESS;
   }
    
  // get state - containment of endPts by intervals
  SmBoolean bThisContainsOMin = ContainsValue(crOther.m_dMin);
  SmBoolean bThisContainsOMax = ContainsValue(crOther.m_dMax);
  SmBoolean bOContainsThisMin = crOther.ContainsValue(m_dMin);
  SmBoolean bOContainsThisMax = crOther.ContainsValue(m_dMax);
 
  // See if they overlap TWICE:
  if(   bThisContainsOMin && bThisContainsOMax 
     && bOContainsThisMin && bOContainsThisMax) 
    {
      // first check, if one of the results is 0dimensional:
      SmBoolean bRet1Single = IsValueOnBoundary(crOther.m_dMax);
      SmBoolean bRet2Single = IsValueOnBoundary(crOther.m_dMin);

      // Here both alternatives work (as opposed to the case below): 
      SmPeriodicExtent1d 
          sCandidateRet1(        m_dMin, crOther.m_dMax, m_dPeriod),
          sCandidateRet2(crOther.m_dMin,         m_dMax, m_dPeriod);

      if(bRet1Single && bRet2Single) { SER(SM_ERR); }
      // should have singled out above

      rlResultCnt = 2;
      rbResultType = SM_PI_2REGIONS;

      if(bRet2Single || bRet1Single) 
        {
          rlResultCnt = 2;
          rbResultType = SM_PI_REGION_POINT;
        }

      rResult  = bRet1Single ? sCandidateRet2 : sCandidateRet1;
      rResult2 = bRet1Single ? sCandidateRet1 : sCandidateRet2;
      rResult.Normalize();
      rResult2.Normalize();
      return SM_SUCCESS;
    }

  // They now touch or overlap. 
  // Check if this really holds. 
  //    - each should contain exactly 1 endpoint of the other,
  //    - exactly one Min and one Max should be contained in the other.
  if (bThisContainsOMin && bThisContainsOMax) { SER(SM_ERR); }
  if (bOContainsThisMin && bOContainsThisMax) { SER(SM_ERR); }
  if (!bThisContainsOMin && !bThisContainsOMax) { SER(SM_ERR); }
  if (!bOContainsThisMin && !bOContainsThisMax) { SER(SM_ERR); }

  if (bThisContainsOMin && bOContainsThisMin) { SER(SM_ERR); }
  if (bThisContainsOMax && bOContainsThisMax) { SER(SM_ERR); }

  double dMin = bThisContainsOMin ? crOther.m_dMin : m_dMin;
  double dMax = bThisContainsOMax ? crOther.m_dMax : m_dMax;

  SmPeriodicExtent1d sCandidateRet(dMin, dMax, m_dPeriod);
  // should be normalized: composed of 1 Min and 1 Max
  // sCandidateRet.Normalize();

  // The case of the single point result
  if(sCandidateRet.GetLength() == 0.0) 
    {
      rlResultCnt  = 1 ;
      rbResultType = SM_PI_POINT;
      rResult = sCandidateRet;
      return SM_SUCCESS;
    }

  // Singe point result can emerge like this too:
  if(sCandidateRet.IsFullPeriod()) 
    {
      rlResultCnt  = 1 ;
      rbResultType = SM_PI_POINT;
      sCandidateRet.m_dMin = sCandidateRet.m_dMax = 0.0;
      rResult = sCandidateRet;
      return SM_SUCCESS;
    }

  // Now everything should be set: the result is in sCandidateRet, the 
  // type is REGION:
  rResult = sCandidateRet;
  return SM_SUCCESS;

} // end SmPeriodicExtent1d::Intersect

/*******************************************************************//**
PURPOSE: Subtract one periodic interval from another

NOTES: 1. The input argument interval periods must be the same. 
       2. when rlResultCnt is set to
            0 result    : NULL set result: rResult and rResult2  are Empty:[min=SM_BIG_DOUBLE,max=-SM_BIG_DOUBLE]
            1 result    : rResult contains one solution, rResult2 is Empty:[min=SM_BIG_DOUBLE,max=-SM_BIG_DOUBLE]
            2 results   : rResult contains one solution, rResult2 the other
       3. Special case: subtracting a PtInterval from the end of ThisIvl just return ThisIvl,
                        ie. if(OtherMin == OtherMax == ThisMin to Tol) { rResult = { ThisMin ThisMax } }
                            if(OtherMin == OtherMax == ThisMax to Tol) { rResult = { ThisMin ThisMax } }
***********************************************************************/
SmStatus SmPeriodicExtent1d::Subtract
 (const SmPeriodicExtent1d & crOther,      // in : Other PeriodicExtent to subtract
  ULONG                    & rlResultCnt,  // out: 0 = Empty set, rResult undefine, rResult2 undefined
                                           //      1 = rResult contains 1st result, rResult2 undefined
                                           //      2 = rResult contains 1st result, rResult2 the 2nd
  SmPeriodicExtent1d       & rResult,      // out: 1st result, or undefined for NULL set outputs
  SmPeriodicExtent1d       & rResult2)     // out: 2nd result, or undefined for NULL set and single result outputs
 const
{
  // check state
  SM_ASSERT_VALID_NO_STREAM(this);
  SM_ASSERT_VALID_NO_STREAM(&crOther);

  // check input
  SM_ASSERT(m_dPeriod == crOther.m_dPeriod);

  // init output
  rResult.SetEmpty() ;
  rResult2.SetEmpty() ;

  // low work - disjoint (handles all IsEmpty() cases - return this
  if(AreDisjoint(crOther)) { rlResultCnt = 1 ;
                             rResult = *this ;
                             return SM_ERR;
                           }

  // low work - this contained by crOther (handles crOther is full case) - return NULL set
  if(IsContainedBy(crOther)) { rlResultCnt = 0 ;
                               return SM_SUCCESS;
                             }

  // low work - this is FullPeriod - return crOther compliment
  if(IsFullPeriod()) { rlResultCnt = 1;
                       rResult = crOther ;
                       rResult.Compliment() ;
                       return SM_SUCCESS;
                     }

  // remember when OtherIvl is a PtIvl near This Min or Max values
  double dTol = SmTol::GetScaledZero(m_dPeriod) ; 
  SmBoolean bIsOtherPtIvl = SM_ARE_SAME_TO_TOL(crOther.m_dMax, crOther.m_dMin, dTol) ;
  SmBoolean bNearMin      = bIsOtherPtIvl && (SM_ARE_SAME_TO_TOL(crOther.m_dMin, m_dMin, dTol) || SM_ARE_SAME_TO_TOL(crOther.m_dMax, m_dMin, dTol)) ;
  SmBoolean bNearMax      = bIsOtherPtIvl && (SM_ARE_SAME_TO_TOL(crOther.m_dMin, m_dMax, dTol) || SM_ARE_SAME_TO_TOL(crOther.m_dMax, m_dMax, dTol)) ;

  // all full and empty cases are handled - handle subtracting two ivls

  // convert to 1d extents of 2 period lengths
  double dThisMin  = m_dMin ; 
  double dThisMax  = m_dMax + ((m_dMin > m_dMax) ? m_dPeriod : 0.0) ;
  double dThisMinP = m_dMin + m_dPeriod ;
  double dOtherMin = crOther.m_dMin ; 
  double dOtherMax = crOther.m_dMax + ((crOther.m_dMin > crOther.m_dMax) ? crOther.m_dPeriod : 0.0) ;

  // map Other Min/Max to one of the A intervals between pts { 0, Amin, Amax, AminP, 2P }
  ULONG lMinClassify =   dOtherMin < dThisMin  ? 0
                       : dOtherMin < dThisMax  ? 1
                       : dOtherMin < dThisMinP ? 2
                       :                         3 ;

  ULONG lMaxClassify =   dOtherMax < dThisMin  ? 0
                       : dOtherMax < dThisMax  ? 1
                       : dOtherMax < dThisMinP ? 2
                       :                         3 ;
  SM_ASSERT_MSG(lMinClassify <= lMaxClassify, _T("SmPeriodicaExtent1d::Subtract: error misclassified Extent end points - needs debugging")) ;

  // switch on the Other end pt classifications
  switch(lMinClassify * 10 + lMaxClassify)
    {
      case 00 : // A-B = {Amin, Amax}:: A: 0         Amin--------Amax     AminP    2P A-B = {Amin, Amax}
                //                   :: B:   Bmin-Bmax
                { rlResultCnt = 1 ;
                  rResult.SetMinMax (this->  m_dMin, this->  m_dMax, NULL, TRUE) ;
                } break ;                                                               
      case 01 : // A-B = {Bmax, Amax}:: A: 0         Amin--------Amax     AminP    2P  
                //                   :: B:   Bmin-----------Bmax   
                { rlResultCnt = 1 ; 
                  rResult.SetMinMax (bNearMin ? this->m_dMin : crOther.m_dMax, this->  m_dMax) ; 
                } break ;                                                               
      case 02 : // A-B = { }         :: A: 0         Amin--------Amax     AminP    2P
                //                   :: B:   Bmin--------------------Bmax   
                { rlResultCnt = 0 ; } break ;                                                                
      case 03 : // B bigger than P   :: A: 0         Amin--------Amax     AminP    2P  // not possible
                // Not possible      :: B:   Bmin------------------------------Bmax    // B is longer than a single period P
                { SM_ASSERT_ERR_MSG (_T("SmPeriodicaExtent1d::Subtract: error misclassified Extent end points - needs debugging")) ;
                } break ;
      case 11 : // A-B = {Amin, Bmin}:: A: 0       Amin---------Amax     AminP      2P
                //       {Bmax, Amax}:: B:             Bmin-Bmax                       
                {
                  if(bNearMin || bNearMax) // subtract an endPtIvl from Ivl = Ivl
                    { rlResultCnt = 1 ;
                      rResult.SetMinMax (this->  m_dMin, this->  m_dMax, NULL, TRUE) ;
                    }
                  else // subtract a InsidePtIvl or an InsideIvl from Ivl = 2 Ivls
                    { 
                      rlResultCnt = 2 ; 
                      rResult.SetMinMax (this->  m_dMin, crOther.m_dMin, NULL, TRUE) ;
                      rResult2.SetMinMax(crOther.m_dMax, this->  m_dMax, NULL, TRUE) ;
                    }
                } break ;
      case 12 : // A-B = {Amin, Bmin}:: A: 0       Amin---------Amax     AminP      2P
                //                   :: B:             Bmin---------Bmax               
                { rlResultCnt = 1 ; 
                  rResult.SetMinMax (this->  m_dMin, bNearMax ? this->m_dMax : crOther.m_dMin, NULL, TRUE) ;
                } break ;                                                                
      case 13 : // A-B = {Bmax, Bmin}:: A: 0       Amin---------Amax     AminP      2P
                //                   :: B:             Bmin--------------------Bmax   
                { rlResultCnt = 1 ; 
                  rResult.SetMinMax (crOther.m_dMax, crOther.m_dMin, NULL, TRUE) ;
                } break ;                                                                
      case 22 : // A-B = {Amin, Amax}:: A: 0       Amin---------Amax           AminP      2P
                //                   :: B:                           Bmin-Bmax   
                { rlResultCnt = 1 ; 
                  rResult.SetMinMax (this->  m_dMin, this->  m_dMax, NULL, TRUE) ;
                } break ;                                                                
      case 23 : // A-B = {Bmax, Amax}:: A: 0       Amin---------Amax           AminP      2P
                //                   :: B:                           Bmin------------Bmax 
                { rlResultCnt = 1 ; 
                  rResult.SetMinMax (bNearMin ? this->m_dMin : crOther.m_dMax, this->  m_dMax, NULL, TRUE) ;
                } break ;                                                                
      case 33 : // Bmin > Amin+P     :: A: 0       Amin---------Amax    AminP             2P  // not possible 
                // invalid rep       :: B:                                     Bmin-Bmax      // Bmin can't be bigger than AminP
                { SM_ASSERT_ERR_MSG (_T("SmPeriodicaExtent1d::Subtract: error misclassified Extent end points - needs debugging")) ;
                } break ;
      default:  // not possible - all cases handled
                { SM_ASSERT_ERR_MSG (_T("SmPeriodicaExtent1d::Subtract: error misclassified Extent end points - needs debugging")) ;
                } break ;
    } // end switch on end pt cases

  // all done
  return SM_SUCCESS ;

} // end SmPeriodicExtent1d::Subtract

/*******************************************************************//**
PURPOSE: Expand the interval by an absolute value.

NOTES: 
***********************************************************************/
void SmPeriodicExtent1d::ExpandAbsolute
 (double dExpansion)
{
  SM_ASSERT(dExpansion >= 0.0);

  // empty intervals - expand about their seam
  if(IsEmpty())
    {  m_dMin = m_dPeriod - dExpansion ;
       m_dMax =dExpansion ;
    }

  double dNewMin = m_dMin - dExpansion;
  if (ContainsValue(dNewMin)) {
     m_dMin = 0.0;
     m_dMax = m_dPeriod;
     return;
  }
     
  m_dMin -= dExpansion;
  m_dMax += dExpansion;
  m_dMin = NormalizeValue(m_dMin);
  m_dMax = NormalizeValue(m_dMax);
  Normalize();
  return;

} // end SmPeriodicExtent1d::ExpandAbsolute

/*******************************************************************//**
PURPOSE: Shift Ivl forward or backward along periodic extent (linear transform).

NOTES: 
***********************************************************************/
void SmPeriodicExtent1d::Translate
 (double dValue) // in : param dist to move extent boundaries
{
  // no work - empty
  if(IsEmpty())
    { return ; }

  m_dMin += dValue;
  m_dMax += dValue;
  m_dMin = NormalizeValue(m_dMin);
  m_dMax = NormalizeValue(m_dMax);
  Normalize();

  // all done
  return;

} // end SmPeriodicExtent1d::Translate

/*******************************************************************//**
PURPOSE: return a mid interval value accounting for seams when needed

NOTES: 
***********************************************************************/
double SmPeriodicExtent1d::GetMid() const
{
  // empty set
  if(IsEmpty() )
    { return 0.0 ; }

  // normal interval
   return(  ((m_dMin + m_dMax) / 2.0)            // without seam 
          + (  (m_dMin > m_dMax)                 // add in seam effect
             ? (  (m_dMin + m_dMax) < m_dPeriod) 
                ? ( m_dPeriod/2.0)               //   in 2nd half of period
                : (-m_dPeriod/2.0)               //   in 1st half of period
             : 0)) ;                             // no seam effect to add
} // end SmPeriodicExtent1d::GetMid

/*******************************************************************//**
PURPOSE: Get the T Evaluation side of a curve domain.
            
RETURN ---  TRUE = TValue is in lower half of interval
            FALSE= TValue is in upper half of interval  

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
SmBoolean SmPeriodicExtent1d::GetTLeftEval
 (double dTValue) 
 const
{
  // low work - Full or empty Period
  if(IsFullPeriod() || IsEmpty())
    { return TRUE ; }

  // handle Ivls including or excluding the seam
  double dCorrection = (m_dMin > m_dMax) ? - m_dPeriod: 0.0 ;

  // mid parameter
  double dTMid = (m_dMin + m_dMax + dCorrection) / 2.0;

  // pick side and return
  SmBoolean bRet = (dTValue <= dTMid) ; 
  return bRet;

} // end SmPeriodicExtent1d::GetTLeftEval

/*******************************************************************//**
PURPOSE: AssertValid() Test reporting static labels
***********************************************************************/
SmAssertReportLabel sAssertPeriodicExtent1d_list[] =
{
  {SM_AT_VALUES, _T("Bad Period"),       _T("m_dPeriod must be greater 0.0 and it's not") },
  {SM_AT_VALUES, _T("Bad Min Boundary"), _T("m_dMin must be in range:[0 m_dPeriod] and it's not") },
  {SM_AT_VALUES, _T("Bad Max Boundary"), _T("m_dMax must be in range:[0 m_dPeriod] and it's not") }
} ;

/*******************************************************************//**
PURPOSE: Determine validity of the SmPeriodicExtent1d.  Basically
    it tests if the values are within the period.

RETURNS ---  TRUE  = OK
             FALSE = Problem
***********************************************************************/
SmBoolean SmPeriodicExtent1d::AssertValid
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

  // init return value
  SmBoolean bRtn = TRUE ;

  bRtn &= SM_ASSERT_VALUE_REPORT(0, SM_LEVEL_0, (m_dPeriod > 0.0), 0.0, m_dPeriod, _T("") ) ;
  bRtn &= SM_ASSERT_VALUE_REPORT(1, SM_LEVEL_0, (IsEmpty() == TRUE) || SM_IS_CONTAINED(m_dMin, 0.0, m_dPeriod), SM_EFF_ZERO, m_dMin, _T("") ) ;
  bRtn &= SM_ASSERT_VALUE_REPORT(2, SM_LEVEL_0, (IsEmpty() == TRUE) || SM_IS_CONTAINED(m_dMax, 0.0, m_dPeriod), SM_EFF_ZERO, m_dMax, _T("") ) ;

  // all done
  // SM_ASSERT(bRtn) ;
  return(bRtn) ;

} // end SmPeriodicExtent1d::AssertValid

/*******************************************************************//**
PURPOSE: Get the length of the interval, a value from [0 m_dPeriod]

NOTES: 1. zero length intervals are snapped exactly to 0.0
       2. empty set intervals are defined to be 0.0
       3. negative intervals are made positive by adding m_dPeriod
***********************************************************************/
double SmPeriodicExtent1d::GetLength() const
{
  // tolerance
  SmScaledZero sScaledZero = SmTol::GetScaledZero(m_dPeriod) ;

  // no work - full interval
  if (IsFullPeriod())
   { return m_dPeriod; }

  // no work - empty set
  if(IsEmpty())
    { return 0.0 ; }

  // get raw interval length
  double dRet = m_dMax - m_dMin;

  // snap short intervals to zero
  if (smos_Fabs(dRet) < sScaledZero )
   { return 0.0; }

  // negative lengths include the seam as part of the interval - size accordingly
  if(dRet < 0.0)
   { dRet += m_dPeriod; }

  // snap short intervals to zero
  if (smos_Fabs(dRet) < sScaledZero )
      return 0.0;

  // length should be positive and within[0 m_dPeriod]
  SM_ASSERT(dRet <= m_dPeriod);

  // all done
  return dRet;

} // end SmPeriodicExtent1d::GetLength

/*******************************************************************//**
PURPOSE: Return TRUE when range is a single point, else return FALSE

NOTES: tolerance is SmTol::GetScaledZero(dPeriod)
***********************************************************************/
SmBoolean SmPeriodicExtent1d::IsPoint() const
{
  // check state 
  SM_ASSERT_BREAK(IsInit() == FALSE) ;

  // tolerance
  SmScaledZero sScaledZero = SmTol::GetScaledZero(m_dPeriod) ;

  // check for internal and seam pts
  return(   (   (   IsEmpty() == FALSE)))
         && (   (   SM_ARE_SAME_TO_TOL(m_dMin, m_dMax, sScaledZero) )                     // same points - could assert m_bFullPeriod is FALSE
             || (   SM_ARE_SAME_TO_TOL(smos_Fabs(m_dMax-m_dMin), m_dPeriod, sScaledZero)  // seam point and
                 && m_bFullPeriod == FALSE)) ;                                            // not the full periods

} // end SmPeriodicExtent1d::IsPoint

/*******************************************************************//**
PURPOSE: Map value to periodic interval:[0.0 m_dPeriod]

NOTES: 1. Map values from neighbor periods to primary period
          by addin +/- m_dPeriod when needed.
       2. returns dValue for values in the primary period
       3. fixes minor numeric turbulences by snapping values 
          within SmTol::GetScaledZero(m_dPerior) of 0.0 and m_dPerior
***********************************************************************/
double SmPeriodicExtent1d::NormalizeValue
 (double dValue) 
 const
{
  SmScaledZero sScaledZero = SmTol::GetScaledZero(m_dPeriod) ;

  // clamp value to [0 m_dPeriod]
  dValue =   (smos_Fabs(dValue)             < sScaledZero) ? 0.0
           : (smos_Fabs(dValue - m_dPeriod) < sScaledZero) ? m_dPeriod
           : dValue ;

  // for empty sets - just return 0.0
  if(IsEmpty())
    { return 0.0 ; }

  // map value to Ivl:[0 m_dPeriod] by one period
  return(  (dValue < 0.0)       ? dValue + m_dPeriod
         : (dValue > m_dPeriod) ? dValue - m_dPeriod
         : dValue ) ;

} // end SmPeriodicExtent1d::NormalizeValue

/*******************************************************************//**
PURPOSE: Adjusts interval to be within periodic range and sets m_bFullPeriod

NOTES: maps [min max bFullPeriod] to [min max bFullPeriod]
       case [TolVal TolVal FALSE] to [0 period FALSE]
       case [0 Period False]      to [0 period TRUE]
       This function also fixes minor numeric turbulences.
       This sets m_bFullPeriod == TRUE for all cases using the full period
       That means a point on the seam will be turned into a full
       interval by this call.
***********************************************************************/
void SmPeriodicExtent1d::Normalize()
{
  // no work - empty set
  if(IsEmpty())
    { return ; }

  SmScaledZero sScaledZero = SmTol::GetScaledZero(m_dPeriod) ;

  // gwc: this seems wrong - a try to fix m_bFull values? 
  // gwc: any extent using the full period here is set to FullPeriod
  //      that means: one can't represenet a point on the seam and call this method
  if (smos_Fabs(m_dPeriod - (m_dMax - m_dMin)) < sScaledZero) 
    { m_bFullPeriod = TRUE; }

  // Single points at seam should be 0.0
  if(   GetLength() == 0.0 
     && smos_Fabs(m_dMin - m_dPeriod) < sScaledZero) 
    {
      m_dMin = 0.0;
      m_dMax = 0.0;
    }

  // The start point should not be the period:
  if(smos_Fabs(m_dMin - m_dPeriod) < sScaledZero)
      m_dMin = 0.0;

  // The end point should not be 0.0:
  if(smos_Fabs(m_dMax) < sScaledZero)
      m_dMax = m_dPeriod;

} // end SmPeriodicExtent1d::Normalize

/*******************************************************************//**
PURPOSE: Pretty print SmPeriodicExtent to curent output stream

NOTES: 
***********************************************************************/
void SmPeriodicExtent1d::Dump(void) const
{
  TCHAR sBuff[SM_TBLOCK_SIZE];
  if(m_dMax < m_dMin)
    { // interval contains seam
      smos_sprintf(sBuff,_T(" [-->%16.16lf   %16.16lf-->] contains seam in range:[0 %16.16lf], FullPeriod:[%s]"), 
                   m_dMin, 
                   m_dMax, 
                   m_dPeriod,
                   m_bFullPeriod ? _T("Yes") : _T("No")) ;
    }
  else // interval does not contain seam
    {
  smos_sprintf(sBuff,_T(" [%16.16lf -> %16.16lf] in range:[0 %16.16lf], FullPeriod:[%s]"), 
             m_dMin, 
             m_dMax, 
             m_dPeriod,
             m_bFullPeriod ? _T("Yes") : _T("No")) ;
    }
  smos_WriteBuffer(sBuff);

} // end SmPeriodicExtent1d::Dump

