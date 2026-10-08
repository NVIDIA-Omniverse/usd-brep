// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************//**
* FILE NAME --- SmMath.cpp
* PURPOSE: Implementation of utility math functions
**********************************************************************/

#include "StdAfx.h"

#include <SmMath.h>  // includes SmTol.h

/***********************************************************
PURPOSE:

NOTES:
***********************************************************/
SM_EXPORT double sm_sqrt( double x )
{ 
  SM_ASSERT_BREAK(x >= -SM_EFF_ZERO_SQ) ;
  if(x < -SM_EFF_ZERO_SQ) { ERR(SM_ERR); return (0.0); }
  else if (x <=  0.0)     { return(0.0); }
  else                    { return sqrt(x); }
} // end sm_sqrt
/***********************************************************
PURPOSE:

NOTES:
***********************************************************/
SM_EXPORT double sm_cbrt(double x) 
{
  return(std::cbrt(x)) ;
} // end sm_cbrt

/***********************************************************
PURPOSE:

NOTES:
***********************************************************/
SM_EXPORT double sm_acos(double x) 
{ 
  if ( x >  1.0 + SM_EFF_ZERO ) { ERR(SM_ERR); }
  if ( x < -1.0 - SM_EFF_ZERO ) { ERR(SM_ERR); }
  if ( x >=  1.0 ) return 0.0;
  if ( x <= -1.0 ) return SM_PI;
  return acos( x );

} // end sm_acos

/***********************************************************
PURPOSE:

NOTES:
***********************************************************/
SM_EXPORT double sm_asin(double x)   
{ 
  if ( x >  1.0 + SM_EFF_ZERO ) { ERR(SM_ERR); }
  if ( x < -1.0 - SM_EFF_ZERO ) { ERR(SM_ERR); }
  if ( x >=  1.0 ) return  SM_PI/2.0;
  if ( x <= -1.0 ) return -SM_PI/2.0;
  return asin( x );
} // end sm_asin

/***********************************************************
PURPOSE: return 10**lLog10 for ULONG values

NOTES: examples: +----------------+-----------------+
                 | 10**lLog10     | return:         |
                 +----------------+-----------------+
                 |      0,        | 1.0e0 = 1.0     |
                 |      1,        | 1.0e1 = 10.0    |
                 |      2,        | 1.0e2 = 100.0   |
                 |      3,..      | 1.0e3 = 1000.0  |
                 |      N         | 1.0eN = 10**N   |
                 +----------------+-----------------+
***********************************************************/
SM_EXPORT double smos_exp10(ULONG lLog10)       
{ switch(lLog10) 
    { case 0: return(1.0e0) ;  case 5: return(1.0e5) ;  case 10: return(1.0e10) ; 
      case 1: return(1.0e1) ;  case 6: return(1.0e6) ;  case 11: return(1.0e11) ; 
      case 2: return(1.0e2) ;  case 7: return(1.0e7) ;  case 12: return(1.0e12) ; 
      case 3: return(1.0e3) ;  case 8: return(1.0e8) ;  case 13: return(1.0e13) ; 
      case 4: return(1.0e4) ;  case 9: return(1.0e9) ;  case 14: return(1.0e14) ; 
      default: { double dGain = 1.0e15 ; 
                 for(ULONG ii=15;ii<lLog10;ii++) { dGain *= 10.0 ; }
                 return(dGain) ; 
               }
    } // end switch on lLog10
} // end smos_exp10

/***********************************************************
PURPOSE: round down integers (neg numbers get more negative)
            leaving digits number of significant figures       
            examples: smos_RoundInt(   32145, 2) =    32000    
                      smos_RoundInt(-4563218, 3) = -4570000    
                      smos_RoundInt(     376, 4) =      376    
NOTES:
***********************************************************/
SM_EXPORT int smos_RoundInt(int num, ULONG digits) 
{ 
  if(abs(num) > pow(10.0,(double)digits) && digits > 0)
       { double base = pow(10.0, floor(log10((double)(abs(num))) - digits + 1)) ;
         return( (int)(floor(num / base) * base)) ;
       }
  else { return(num) ;
       }
}

/***********************************************************
PURPOSE:  convert numbers like 1.1300000000007 to 1.13

NOTES:
  Removes the first 4 decimal places of the number and if the
  remainder is less than tolerance - removes the remainder from
  the number otherwise does nothing.

  So 1.1234000000007 returns 1.1234          and
     1.1234500000007 returns 1.1234500000007
***********************************************************/
SM_EXPORT double smos_CleanUpNoise(double x) 
{
#define NUM_DECIMAL_PLACES  4
  // let   x = dRound + dDecimals
  // where  dRound    = largest integer smaller than x
  // and    dDecimals = the decimals in x.
  double dTolScaled = SM_EFF_ZERO * (1.0 + smos_Fabs(x));
  double dRound     = floor(x);
  double dDecimals  = x - dRound;
  double dIncr      = 1.0;
  for (ULONG i=1; i<=NUM_DECIMAL_PLACES; i++) 
    {
      // if dDecimals are equal to 1.0
      if (1.0-dDecimals < dTolScaled) 
        {
          // return integer value (dRound + 1)
          dRound += dIncr;
          return dRound;
        }

      // when dDecimals are equal to 0.0
      if (dDecimals < dTolScaled) 
        {
          // return integer value (dRound)
          return dRound;
        }

      // left shift (base 10) dDecimals so that its largest numeral is now an integer
      // move that numeral to dRound so that
      //   x = dRound + dDecimals/(i*10),
      //   where dRound is no longer integer
      // left shift tolerance to match left shift in dDecimals
      dIncr            *= 0.1;
      dDecimals        *= 10.0;
      dTolScaled       *= 10.0;
      double dOneDigit  = floor(dDecimals);
      dDecimals        -= dOneDigit;
      dRound           += dIncr*dOneDigit;
    }
  return x;   // No change
#undef NUM_DECIMAL_PLACES

} // end smos_CleanUpNoise(double x)

