// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*******************************************************************//**
* FILE NAME --- SmMath.h
* PURPOSE:  Header file for math interface.
**********************************************************************/

#ifndef __SMOS_MATH_H_
#define __SMOS_MATH_H_

#ifndef __SMMESSAGES_H__
#include <SmMessages.h>
#endif

#ifndef _INC_MATH
#include <math.h>
#endif

// extrema values
#define SM_BIG_DOUBLE              1.0e20
#define SM_UNDEF_DOUBLE            1.23456e-61
#define SM_BIG_ULONG               99999999
#define SM_UNDEF_ULONG             123456789   /* make SM_UNDEF_ULONG > SM_BIG_ULONG so AssertValid(ln < SM_BIG_ULONG) checks still work */
#define SM_INFINITE_PARAMETER      1234567.0
#define SM_INFINITE_PARAMETER_SQRT 1111.111    /* used for representations that compute with this number squared */
#define SM_BOUNDED_INFINITE_PARAM  33.0        /* Size to approx infinite interval with a bounded one - typically for sampling and graphics */
#define SM_UNINIT_TOL              1.23456e-61 /* temporary value (SM_UNDEF_DOUBLE) for incremental release - change to 0.0 when things settle down */

// PI
#define SM_HALFPI  1.5707963267948965192313216916397514420985846996875
#define SM_PI      3.1415926535897932384626433832795028841971693993751
#define SM_3HALFPI 4.7123889803846897576939650749192543262957530990626
#define SM_2PI     6.2831853071795864769252867665590057683943387987502
#define SM_SQRT3   1.7320508075688772935274463415058723669428052538104

// ScaledZero constant
#define SM_EFF_ZERO      1.0e-12                         /* Zero numeric tol, used by SmTol::GetScaledZero() methods       */ 
#define SM_EFF_ZERO_PARAM 1.2e-8                          /* Zero numeric tol for Knot Param values: tad larger than NLib's */
#define SM_EFF_ZERO_DEG   1.0E-4                          /* Zero numeric tol for Angle degrees                             */
#define SM_EFF_ZERO_RAD   (SM_EFF_ZERO_DEG * SM_PI/180.0) /*   = 1.74e-5 radians                                            */

// Convenience Tolerance values for efficiency 
#define SM_EFF_ZERO_SQRT     1.0e-6
#define SM_EFF_ZERO_SQ       1.0e-24

// Tolerances: NumericZero, NumericAngleZero, ZoneTol3d, and Parallel and Continuity AngleZero
#define SM_MODEL_SIZE_ESTIMATE       50.0        // Tolerances default val user-interface. See SmTol::SetSystemModelSizeEstimate()
#define SM_LARGE_SMALL_SIZE_RATIO    1000        // Tolerances default val user-interface. See SmTol::SetLargeSmallSizeRatio()
#define SM_USE_DEFAULT            9876543.21     // used in Brep & Context constructors to say: use current System Vals

#define SM_ZONE_TOL_3D    1.0e-5                          /* Local Neighborhood Size, used by SmTol::GetZoneTol3d() methods */
#define SM_XSECT_TOL_3D   (2.0*SM_ZONE_TOL_3D)            /* Default Gap and SmTol::GetXSectTol3d Size                      */
#define SM_APPROX_TOL_3D  (SM_ZONE_TOL_3D/2.0)            /* Default SmTol::GetApproxTol3d Size                             */
#define SM_ANG_TOL_DEG    0.5                             /* ZeroAngDeg geometric tol (max ang between parallel vecs)       */
#define SM_ANG_TOL_RAD    (SM_ANG_TOL_DEG * SM_PI/180.0)  /*   = 8.72e-3 radians                                            */
#define SM_TOL_GAP_GAIN   1.1                             /* OverSize amount for Tols above an obj's MaxGap3d size */ 

// convenience constants used in tolerances
#define SM_SINE_10DEG     0.173648177666930348851716626769315    /* used for 'near' parallel checks */
#define SM_ANGLE_TOL      (SM_EFF_ZERO_DEG)               /* used to be: (SM_EFF_ZERO * 1000.0 * (1.0 + 360.0)) equals 3.61E-7 */

//  GWC: the following needs to be rethought and moved to SmTol. Current thoughts:
//       parallel could depend on length; lines are parallel if they are within XSectTol3d over their whole length
//         longer lines would have to have smaller angles to be parallel than short ones.
//         very short lines would have to be limited by some absolute angle value.
//       continuity would probably never depend on length - it's likely to be a constant angle value.
//         It's possible the parallel angle limit and the continuity angle might be one and the same value.
//       or compute tangent angle tolerance from 3d zone tolerance. 
//          Let tangent angle be that angle between a pair of lines
//          that let the lines lie within tolerance of one another
//          over a range specified in the constant SM_COINCIDENT_RANGE.
#define SM_CONTINUITY_ANGLE  (SM_ANG_TOL_DEG) /* GWC: used to be 1.0. SMLib may need both continuity and parallel angle tolerances - for now they are the same */
#define SM_COINCIDENT_LENGTH 10.0
#define SM_TANGENT_ANGLE_RADIANS(dZoneTol3d) smos_ArcTangent(2.0*(dZoneTol3d)/SM_COINCIDENT_LENGTH)

// converting Radians and Degrees
#define SM_DEG2RAD(adeg)            ((adeg) * (SM_PI / 180.0))
#define SM_RAD2DEG(arad)            ((arad) * (180.0 / SM_PI))

// obsolete - supported for backward compatibility
#define SM_RADIANS(adeg)            (SM_DEG2RAD(adeg))
#define SM_DEGREES(arad)            (SM_RAD2DEG(arad))
#define SM_DEGREES_TO_RADIANS(adeg) (SM_DEG2RAD(adeg))
#define SM_RADIANS_TO_DEGREES(arad) (SM_RAD2DEG(arad))

// SMLib interface to standard math ops - allows overriding when needed
SM_EXPORT double sm_sqrt(double x);       /* Note that we have provided an implementation of  */
SM_EXPORT double sm_cbrt(double x);       /* square root, cubed root, ArcCosine, and ArcSine. */ 
SM_EXPORT double sm_acos(double x);       /* You may utilize the default system functions by  */
SM_EXPORT double sm_asin(double x);       /* removing the sm_ in the following defines.       */

// JLMCC review change
#define smos_Fabs fabs
//inline    double smos_Fabs(double a)                       { return (((a)<0) ? -(a) : (a)); }   
inline    long   smos_Labs(long   a)                       { return (((a)<0) ? -(a) : (a)); }  
inline    int    smos_Iabs(int    a)                       { return (((a)<0) ? -(a) : (a)); }
SM_EXPORT double smos_CleanUpNoise(double x) ;             // convert numbers like 1.1300000000007 to 1.13
SM_EXPORT int    smos_RoundInt(int num, ULONG digits) ;    // round down integers (neg numbers get more negative)
                                                           // leaving digits number of significant figures  
SM_EXPORT double smos_exp10(ULONG lGainLog10);             // return 10.0 ** lGainLog10, e.g if lGainLog10 = 2, return 100.0
#define          smos_Log             log                  /* double smos_Log( double x );  - Natural Log  */
#define          smos_Log10           log10                /* double smos_Log10( double x); - log base 10  */
#define          smos_CubeRoot(a)     sm_cbrt(a)           /* double smos_CubeRoot( double x )             */
#define          smos_Sqrt(a)         sm_sqrt(a)           /* double smos_Sqrt( double x );                */                                                    
// compute smos_sqrt(a*a + b*b) without destructive overflow or underflow  // from Numerical Recipes in C
#define          smos_Pythag(a,b)    (  (smos_Fabs(a) > smos_Fabs(b)) ? ( (a)*smos_Sqrt(1.0 + ((b)/(a))*((b)/(a))) ) \
                                      : ( ((b) != 0.0)                ? ( (b)*smos_Sqrt(1.0 + ((a)/(b))*((a)/(b))) ) \
                                      : 0.0) )

#define          smos_ArcCosine(a)    sm_acos(a)           /* double smos_ArcCosine( double x );           */
#define          smos_ArcSine(a)      sm_asin(a)           /* double smos_ArcSine( double x );             */                                                    
#define          smos_ArcTangent2     atan2                /* double smos_ArcTan2( double y, double x ); Rtn: -Pi to Pi  */
#define          smos_ArcTangent      atan                 /* double smos_ArcTangent( double x );        Rtn: -Pi/2 to Pi/2  */
                 
#define          smos_Cosine(a)       cos(a)               /* double smos_Cosine( double x );              */
#define          smos_CosDeg(a)       cos(SM_DEG2RAD(a))
#define          smos_CosRad(a)       cos(a)
                                      
#define          smos_Sine(a)         sin(a)                /* double smos_Sine( double x );                */
#define          smos_SinDeg(a)       sin(SM_DEG2RAD(a))                                                   
#define          smos_SinRad(a)       sin(a) 
                                                                                                           
#define          smos_Tangent(a)      tan(a)               /* double smos_Tan( double x );                 */
#define          smos_TanDeg(a)       tan(SM_DEG2RAD(a))   
#define          smos_TanRad(a)       tan(a)               
                                                           
#define          smos_CosineH         cosh                 /* double smos_CosineH( double x );             */
#define          smos_SineH           sinh                 /* double smos_SineH( double x );               */

#define          smos_Ceil            ceil                 /* double smos_Ceil(double x);                  */
#define          smos_Floor           floor                /* double smos_Floor(double x);                 */
#define          smos_Fmod            fmod                 /* double smos_Fmod(double x, double y);        */
#define          smos_Pow             pow                  /* double smos_Pow( double a, ULONG power );    */
#define          smos_Sgn(a)          ((a) >= 0.0 ? (1) : (-1))           /* +1 for pos or 0 vals else -1  */
#define          smos_Sign(a,b)       ((b) >= 0.0 ? fabs(a) : - fabs(a))  /* a with same sign as b         */
#define          smos_IsEven(a)       ((((int)(a))/2) * 2 == (int)(a))    /* true when a is even           */
#define          smos_IsOdd(a)        (!(smos_IsEven(a)))                 /* true when a is odd            */
                 
#define          smos_Min(a,b)        (((a)<(b)) ? (a):(b))
#define          smos_3Min(a,b,c)     ( ((a)<(b)) ? (((a)<(c)) ? (a):(c)) : (((b)<(c)) ? (b):(c)) )
#define          smos_4Min(a,b,c,d)   smos_Min(smos_Min((a),(b)), smos_Min((c),(d)) ) 
#define          smos_5Min(a,b,c,d,e) smos_Min(smos_3Min((a),(b),(c)), smos_Min((d),(e)) )
                 
#define          smos_Max(a,b)        (((a)>(b)) ? (a):(b))
#define          smos_3Max(a,b,c)     ( ((a)>(b)) ? (((a)>(c)) ? (a):(c)) : (((b)>(c)) ? (b):(c)) )
#define          smos_4Max(a,b,c,d)   smos_Max(smos_Max((a),(b)), smos_Max((c),(d)) ) 
#define          smos_5Max(a,b,c,d,e) smos_Max(smos_3Max((a),(b),(c)), smos_Max((d),(e)) )

#define          SM_SWAP(S_TYPE,a,b)     {S_TYPE tmp=(a); (a)=(b); (b)=tmp;}
#define          SM_SWAP_PTR(S_TYPE,a,b) {S_TYPE* tmp =(a); (a)=(b); (b)=tmp;} 
                                    
// scaled zero values (scale SM_EFF_ZERO by max dimension)
#include <SmTol.h>  // this line comes after SM_UNDEF_DOUBLE and smos_Max defs due to SmTol::GetScaledZero() template dependencies.

// Useful numerical predicates 

// return false when a is NaN, +/1.#INF, -1.#IND and other bad math values, else return true
#define SM_IS_VALID_DOUBLE(a)             (   ((a) == (a))                                                      /* catches Nan and -1.#IND                           */  \
                                           && ((a) <= DBL_MAX               && (a) >= -DBL_MAX)                 /* catches inf and +/-1.INF                          */  \
                                           && ((a) <= (SM_BIG_DOUBLE/100.0) && (a) >= (-SM_BIG_DOUBLE/100.0)) ) /* catches some numbers computed from SM_BIG_DOUBLE  */
#define SM_IS_INFINITE(a)                 (   (a) == SM_INFINITE_PARAMETER      || (a) == -SM_INFINITE_PARAMETER \
                                           || (a) == SM_INFINITE_PARAMETER_SQRT || (a) == -SM_INFINITE_PARAMETER_SQRT)
#define SM_IS_ZERO(a)                     ( SmTol::InTol( (a), SmTol::GetScaledZero() ))
#define SM_IS_ZERO_SQUARED(a)             ( SmTol::InTol( (a), SmTol::GetScaledZeroSq() ))

#define SM_IS_ZERO_TO_TOL(a,tol)          ( SmTol::InTol( (a), SmScaledZero(tol) ))

#define SM_ARE_SAME(a,b)                  ( SmTol::InTol( ((a)-(b)), SmTol::GetScaledZero((a),(b))) )
#define SM_ARE_SAME_TO_TOL(a,b,tol)       ( SmTol::InTol( ((a)-(b)), SmScaledZero(tol)) )
#define SM_ARE_SAME_TO_SCALED_ZERO(a,b,scale) ( SmTol::InTol( ((a)-(b)), SmTol::GetScaledZero(scale)) )

// is c inside open interval(a,b) boundaries
#define SM_IS_BETWEEN(c,a,b)              (   ((((a)+SmTol::GetScaledZero(a,b))<(c)) && ((c)<((b)-SmTol::GetScaledZero(a,b)))) \
                                           || ((((b)+SmTol::GetScaledZero(a,b))<(c)) && ((c)<((a)-SmTol::GetScaledZero(a,b)))) ) 
#define SM_IS_BETWEEN_TO_TOL(c,a,b,tol)   (   ((((a)+(tol))<(c)) && ((c)<((b)-(tol)))) \
                                           || ((((b)+(tol))<(c)) && ((c)<((a)-(tol)))) ) 

// is c inside or on interval[a,b] boundaries
#define SM_IS_CONTAINED(c,a,b)            (   (((a)<=((c)+SmTol::GetScaledZero(a,b))) && (((c)-SmTol::GetScaledZero(a,b))<=(b))) \
                                           || (((b)<=((c)+SmTol::GetScaledZero(a,b))) && (((c)-SmTol::GetScaledZero(a,b))<=(a))) ) 
#define SM_IS_CONTAINED_TO_TOL(c,a,b,tol) (   (((a)<=((c)+(tol))) && (((c)-(tol))<=(b))) \
                                           || (((b)<=((c)+(tol))) && (((c)-(tol))<=(a))) ) 

// clamp any c value to given interval [a,b] - a may be greater or less than b
#define SM_CLAMP_TO_INTERVAL(c,a,b) (  ((c) < (a) && (c) < (b)) ? ((a) < (b) ? (a) : (b)) \
                                     : ((c) > (a) && (c) > (b)) ? ((a) > (b) ? (a) : (b)) \
                                     : (c))

// clamp any value to the period [0.0, p]                                      
#define SM_CLAMP_TO_ZERO_PERIOD(v, p) (  ((v) < (0.0)) ? (smos_Fmod( (smos_Fmod((v),(p)) + (p)), (p)) ) \
                                       : ((v) >   (p)) ? (smos_Fmod((v),(p)))                           \
                                       : ((v)) )

// clamp any value to any period [p1, p2]                                      
#define SM_CLAMP_TO_PERIOD(v, p1, p2) (SM_CLAMP_TO_ZERO_PERIOD((v)-(p1), (p2)-(p1)) + (p1))

// Periodic angle clamp - clamp angle to range [0,360] or [0,2*SM_PI]
#define SM_CLAMP_PERIODIC_DEG(adeg) ( SM_CLAMP_TO_ZERO_PERIOD((adeg),   360.0) )
#define SM_CLAMP_PERIODIC_RAD(arad) ( SM_CLAMP_TO_ZERO_PERIOD((arad), 2*SM_PI) )


#endif // !__SMOS_MATH_H_
