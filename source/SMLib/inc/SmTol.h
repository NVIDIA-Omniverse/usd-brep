// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*******************************************************************//**
* FILE NAME --- SmTol.h
* PURPOSE: Header file for Tolerance Computations
**********************************************************************/

// all SMLib code (new and existing) should switch to the SmTol interface for tolerances,
//
// compiled with SM_USE_NEWTOL = new consistent tolerance model behavior
// compiled with SM_USE_OLDTOL = old tolerance model behavior

#ifndef __SMTOLERANCE_H__
#define __SMTOLERANCE_H__

#ifndef __SMOS_MATH_H_
#include <SmMath.h>     // includes SmTol.h
#endif
#include <SmCoreTypes.h>  // for class SmPoint3d ;
#include <SmSurfTypes.h>
#include <SmCurveTypes.h>
#include <SmTopoTypes.h>

class SmObject ;
class SmCurve ;
class SmSurface ;
class SmVolume ;
class SmPseudoBox ;
class SmCurveInterval ;
class SmEvalNFunctionsObject ;
class SmTol1d ;
class SmTol2d ;
class SmTol3d ;
class SmGap ;
class SmPointClassification ;
class SmCurveInterval ;
class SmCurveClassification ;
class SmOffsetGeometryCreation ;
class SmPolyIntersector ;
class SmSurfaceIntersector  ;
class SmSurfaceTracer ;
class SmFilletSolver ; 


class SmTopology ;
class SmBrep ;
class SmRegion ;
class SmShell ; 
class SmFace ;
class SmEdge ;
class SmVertex ;
class SmHCR ;
class SmTopologyIntersector ;
class SmPolyPointClassification ;
class SmLineSegInterval ;
class SmLineSegClassification ;

#ifndef SmPoint2d
  #define SmPoint2d SmVector2d
#endif // no SmPoint2d

/*******************************************************************//**
PURPOSE: Control flags for bIsSmallTopology arguments

NOTES: 
***********************************************************************/
enum SmSmallTopoType
{
  SM_ST_NORMAL_SIZE       =  0,   // if pObj has bIsSmallTopology use it, else compute tolerances for Large (normal) scale
  SM_ST_SMALL_SIZE        =  1,   // if pObj has bIsSmallTopology use it, else compute tolernaces for Small (pinhole in battleship) scale
  SM_ST_FORCE_NORMAL_SIZE =  2,   // Always compute tolerances for Large (normal) scale
  SM_ST_FORCE_SMALL_SIZE  =  3,   // Always compute tolernaces for Small (pinhole in battleship) scale
  SM_ST_NO_CHANGE         =  4    // NoAction value for SmTol::SetZoneTol3d
} ;

#if 0
#ifdef SM_DEBUG_CODE
#define SM_SCALE_WHEN_SMALL(eIsSmall, a) (  ((a) == SM_UNINIT_TOL) \
                                          ?  (smos_AssertErrorMessage(SM_ERR_ASSERT_FAILURE,FILE_NAME,LINE_NUMBER,_T("Uninit-Tol"),NULL,1,FUNC_NAME), SM_UNINIT_TOL) \
                                          : (eIsSmall == SM_ST_SMALL_SIZE || eIsSmall == SM_ST_FORCE_SMALL_SIZE) \
                                          ? s_LargeSmallSizeRatio * (a) \
                                          : (a) )
#else  // no SM_DEBUG_CODE
#define SM_SCALE_WHEN_SMALL(eIsSmall, a) (  SM_ASSERT_TOL(a) \
                                            (eIsSmall == SM_ST_SMALL_SIZE || eIsSmall == SM_ST_FORCE_SMALL_SIZE) \
                                          ? s_LargeSmallSizeRatio * (a) \
                                          : (a) )
#endif // no SM_DEBUG_CODE
#endif

/*******************************************************************//**
PURPOSE: compile constants to switch between 
  1. ConsistentToleranceModel and PreviousToleranceModel behaviors,
  When SM_USE_NEWTOL is defined:
    Topology Tolerances default to system values rather 
    than being stored on each topology object.
***********************************************************************/

/*******************************************************************//**
PURPOSE: SmTol3d = Class to give 3d Tolerance values a type

  base Tol3d class:           SmTol3d       { public: double val ; }

  derived StrongType classes: SmZoneTol3d   : public SmTol3d
                              SmXSectTol3d  : public SmTol3d
                              SmApproxTol3d : public SmTol3d
                              SmScaledZero  : public SmTol3d

  SmTol3d repressents:
   1. a Tol3d as a double value

  SmTol3d implements: (features inherited by derived classes)
   1. Auto-Conversion (SmTol3d <=> double)
         When compiled with    SM_USE_NEWTOL_STRONG_TYPES - No automatic conversion 
         When compiled without SM_USE_NEWTOL_STRONG_TYPES - automatic conversion
         note: a. auto-converting SmTol3d and double values is a temporary measure
                 supported until SMLib is completely converted from
                 its original representation of tolerance values as doubles
                 to its new representation of tolerance values as class types.
                 Auto-converting allows a transitional mixed-SMLib where some tolerances
                 are represented as doubles and others as class types.
               b. without auto-converting, places in the code where
                  one tolerance type is used as another are compile time errors.
***********************************************************************/
class SM_EXPORT SmTol3d     
{ 
  public: double val ;      // always a positive value

#ifdef SM_USE_NEWTOL_STRONG_TYPES
  SmBoolean operator==(double dVal) { return smos_Fabs(smos_Fabs(dVal) - val) <= (SM_EFF_ZERO*(1+val)) ; }
  SmTol3d & operator+=(double dVal) { SM_ASSERT_TOL2(val, dVal) ; val += dVal;   return *this; }  // for smos_Max MACROS
  SmTol3d & operator-=(double dVal) { SM_ASSERT_TOL2(val, dVal) ; val -= dVal;   return *this; }  // for smos_Max MACROS
  SmTol3d & operator*=(double dVal) { SM_ASSERT_TOL2(val, dVal) ; val *= dVal;   return *this; }
  SmTol3d & operator/=(double dVal) { SM_ASSERT_TOL2(val, dVal) ; val /= dVal;   return *this; }
  SmTol3d & Sqrt      ()            { SM_ASSERT_TOL(val) ; val = smos_Sqrt(val); return *this; }
  SmTol3d & Sq        ()            { SM_ASSERT_TOL(val) ; val = val * val ;     return *this; }

#else // no SM_USE_NEWTOL_STRONG_TYPES  // Auto-Convert (SmTol3d <=> double) to allow compiles
                                        // of transitional code where some SMLib tolerances
                                        // are still represented as doubles and others have
                                        // been recoded to be represented with SmTol classes.  
                                        // In the end all tolerances will be represented by 
                                        // SmTol classes and compiling without 
                                        // SM_USE_NEWTOL_STRONG_TYPES will not be an option.
  // Auto-Convert:[SmTol3d<=>double] 
  SmTol3d(double dVal=0.0)  { SM_DBG_WARN_IF(dVal < 0.0, _T("tried to assign neg val to tol, changed to:[fabs(val)]")) ; 
                              val = smos_Fabs(dVal) ;    // gwc: needed to interchange SmTol3d with double
                            }
  SmTol3d      & operator= (double dVal)        { SM_DBG_WARN_IF(dVal < 0.0, _T("tried to assign neg val to tol, changed to:[fabs(val)]")) ; 
                                                  val = smos_Fabs(dVal) ; return *this ; 
                                                }
#ifndef SM_BORLAND
  // assignment
  SmTol3d      & operator= (const SmTol3d sTol) { val = sTol.val ; return *this ; }
#endif // no SM_BORLAND

  // reference operator
  operator const double&() const { return val ; }
  operator       double&()       { return val ; }             // needed to interchange SmTol3d with double
                                                              //  remove these when NEWTOL model is complete
#endif // no SM_USE_NEWTOL_STRONG_TYPES

} ; // end class SmTol3d

// strongly typed tolerance classes
#define SM_TOL_CALLS(Type, InitVal) Type(double dVal=(InitVal)) ; \
                                    Type & operator=(double dVal) { val = smos_Fabs(dVal) ; return *this ; } \
                                    operator const double&() const { return val ; } \
                                    operator       double&()       { return val ; }
class SM_EXPORT SmZoneTol3d  : public SmTol3d { public: SM_TOL_CALLS(SmZoneTol3d  , SM_USE_DEFAULT) } ;                    
class SM_EXPORT SmXSectTol3d : public SmTol3d { public: SM_TOL_CALLS(SmXSectTol3d , SM_USE_DEFAULT) } ;                   
class SM_EXPORT SmApproxTol3d: public SmTol3d { public: SM_TOL_CALLS(SmApproxTol3d, SM_USE_DEFAULT) } ; 
class SM_EXPORT SmScaledZero : public SmTol3d { public: SM_TOL_CALLS(SmScaledZero , SM_EFF_ZERO              ) } ;
class SM_EXPORT SmTol2d      : public SmTol3d { public: SM_TOL_CALLS(SmTol2d      , 0.0                      ) } ;
class SM_EXPORT SmTol1d      : public SmTol3d { public: SM_TOL_CALLS(SmTol1d      , 0.0                      ) } ;

// enable TolValues to work with doubles - (maybe these methods will be obsolete once SMLib is changed to use SmTol definitions)
inline SmTol3d & operator*= (SmTol3d & rTolVal, double d) { SM_ASSERT_TOL2(rTolVal.val, d) ; rTolVal.val *= d; return rTolVal; }
inline SmTol3d & operator/= (SmTol3d & rTolVal, double d) { SM_ASSERT_TOL2(rTolVal.val, d) ;
    if ( smos_Fabs(d) > smos_Fabs(rTolVal.val)*SM_EFF_ZERO ) { rTolVal.val /= d; } return rTolVal; }
inline SmTol3d & operator+= (SmTol3d & rTolVal, double d) { SM_ASSERT_TOL2(rTolVal.val, d) ; rTolVal.val += d; return rTolVal; }
inline SmTol3d & operator-= (SmTol3d & rTolVal, double d) { SM_ASSERT_TOL2(rTolVal.val, d) ; rTolVal.val -= d; return rTolVal; }

// Constant tolerance class objects to be used as default values.
extern SM_THREAD_LOCAL SmXSectTol3d sm_XSecTol3d ;   // constant SmXSectTol3d object to be used for default method argument values

#ifdef SM_USE_NEWTOL_STRONG_TYPES
inline int operator<  (double d, const SmTol3d & rTolVal)                              { SM_ASSERT_TOL(rTolVal.val) ; return d            < rTolVal.val ; }
inline int operator<  (const SmZoneTol3d   & rZone1,   const SmZoneTol3d   & rZone2)   { SM_ASSERT_TOL2(rZone1.val,  rZone2.val  ) ;  return rZone1.val   < rZone2.val ; }
inline int operator<  (const SmApproxTol3d & rApprox1, const SmApproxTol3d & rApprox2) { SM_ASSERT_TOL2(rApprox1.val,rApprox2.val) ; return rApprox1.val < rApprox2.val ; }
inline int operator<  (const SmXSectTol3d  & rXSect1,  const SmXSectTol3d  & rXSect2)  { SM_ASSERT_TOL2(rXSect1.val ,rXSect2.val ) ; return rXSect1.val  < rXSect2.val ; }
inline int operator<= (double d, const SmTol3d & rTolVal)                              { SM_ASSERT_TOL(rTolVal.val) ; return d            <= rTolVal.val ; }
inline int operator<= (const SmZoneTol3d   & rZone1,   const SmZoneTol3d   & rZone2)   { SM_ASSERT_TOL2(rZone1.val,  rZone2.val  ) ; return rZone1.val   <= rZone2.val ; }
inline int operator<= (const SmApproxTol3d & rApprox1, const SmApproxTol3d & rApprox2) { SM_ASSERT_TOL2(rApprox1.val,rApprox2.val) ; return rApprox1.val <= rApprox2.val ; }
inline int operator<= (const SmXSectTol3d  & rXSect1,  const SmXSectTol3d  & rXSect2)  { SM_ASSERT_TOL2(rXSect1.val ,rXSect2.val ) ; return rXSect1.val  <= rXSect2.val ; }
inline int operator>  (double d, const SmTol3d & rTolVal)                              { SM_ASSERT_TOL(rTolVal.val) ; return d            > rTolVal.val ; }
inline int operator>  (const SmZoneTol3d   & rZone1,   const SmZoneTol3d   & rZone2)   { SM_ASSERT_TOL2(rZone1.val,  rZone2.val  ) ; return rZone1.val   > rZone2.val ; }
inline int operator>  (const SmApproxTol3d & rApprox1, const SmApproxTol3d & rApprox2) { SM_ASSERT_TOL2(rApprox1.val,rApprox2.val) ; return rApprox1.val > rApprox2.val ; }
inline int operator>  (const SmXSectTol3d  & rXSect1,  const SmXSectTol3d  & rXSect2)  { SM_ASSERT_TOL2(rXSect1.val ,rXSect2.val ) ; return rXSect1.val  > rXSect2.val ; }
inline int operator>= (double d, const SmTol3d & rTolVal)                              { SM_ASSERT_TOL(rTolVal.val) ; return d            >= rTolVal.val ; }
inline int operator>= (const SmZoneTol3d   & rZone1,   const SmZoneTol3d   & rZone2)   { SM_ASSERT_TOL2(rZone1.val,  rZone2.val  ) ; return rZone1.val   >= rZone2.val ; }
inline int operator>= (const SmApproxTol3d & rApprox1, const SmApproxTol3d & rApprox2) { SM_ASSERT_TOL2(rApprox1.val,rApprox2.val) ; return rApprox1.val >= rApprox2.val ; }
inline int operator>= (const SmXSectTol3d  & rXSect1,  const SmXSectTol3d  & rXSect2)  { SM_ASSERT_TOL2(rXSect1.val ,rXSect2.val ) ; return rXSect1.val  >= rXSect2.val ; }
#endif // SM_USE_NEWTOL_STRONG_TYPES

// for the transition only - casts for later review and fixing (temporary only)
//  note: Where ever a mixed type use causes a tolerance problem, fix it with
//        with these casts so that SMLib can continue compiling now, but
//        will be marked where bits of code need to be reviewed and changed
//        for tolerance strong type use.

#ifdef SM_USE_NEWTOL
SM_NEWTOL_LINE // For values
#define SM_CAST_ZONETOL3D(a)   (a)
#define SM_CAST_XSECTTOL3D(a)  (a)
#define SM_CAST_APPROXTOL3D(a) (a)
#define SM_CAST_SCALEDZERO(a)  (a)
#define SM_CAST_DOUBLE(a)      (a)

SM_NEWTOL_LINE // for Pointers
#define SM_CAST_ZONETOL3D_PTR(a)   (a)
#define SM_CAST_XSECTTOL3D_PTR(a)  (a)
#define SM_CAST_APPROXTOL3D_PTR(a) (a)
#define SM_CAST_SCALEDZERO_PTR(a)  (a)
#define SM_CAST_DOUBLE_PTR(a)      (a)

SM_NEWTOL_LINE // for References
#define SM_CAST_ZONETOL3D_REF(a)   (a)
#define SM_CAST_XSECTTOL3D_REF(a)  (a)
#define SM_CAST_APPROXTOL3D_REF(a) (a)
#define SM_CAST_SCALEDZERO_REF(a)  (a)
#define SM_CAST_DOUBLE_REF(a)      (a)

#else // SM_USE_OLDTOL
SM_OLDTOL_LINE // For values
#define SM_CAST_ZONETOL3D(a)   ((SmZoneTol3d  ) a)
#define SM_CAST_XSECTTOL3D(a)  ((SmXSectTol3d ) a)
#define SM_CAST_APPROXTOL3D(a) ((SmApproxTol3d) a)
#define SM_CAST_SCALEDZERO(a)  ((SmScaledZero ) a)
#define SM_CAST_DOUBLE(a)      ((double)        a)

SM_OLDTOL_LINE // for Pointers
#define SM_CAST_ZONETOL3D_PTR(a)   ((SmZoneTol3d  *) a)
#define SM_CAST_XSECTTOL3D_PTR(a)  ((SmXSectTol3d *) a)
#define SM_CAST_APPROXTOL3D_PTR(a) ((SmApproxTol3d*) a)
#define SM_CAST_SCALEDZERO_PTR(a)  ((SmScaledZero *) a)
#define SM_CAST_DOUBLE_PTR(a)      ((double       *) a)

SM_OLDTOL_LINE // for References
#define SM_CAST_ZONETOL3D_REF(a)   ((SmZoneTol3d  &) a)
#define SM_CAST_XSECTTOL3D_REF(a)  ((SmXSectTol3d &) a)
#define SM_CAST_APPROXTOL3D_REF(a) ((SmApproxTol3d&) a)
#define SM_CAST_SCALEDZERO_REF(a)  ((SmScaledZero &) a)
#define SM_CAST_DOUBLE_REF(a)      ((double       &) a)
#endif // SM_USE_OLDTOL

// conversions
#define SM_ZONE_TO_XSECTTOL3D(a)   ((SmXSectTol3d)  (a)*2.0)
#define SM_APPROX_TO_XSECTTOL3D(a) ((SmXSectTol3d)  (a)*4.0)

#define SM_XSECT_TO_ZONETOL3D(a)   ((SmZoneTol3d)   (a)/2.0)
#define SM_APPROX_TO_ZONETOL3D (a) ((SmZoneTol3d)   (a)*2.0)

#define SM_XSECT_TO_APPROXTOL3D(a) ((SmApproxTol3d) (a)/4.0)
#define SM_ZONE_TO_APPROXTOL3D(a)  ((SmApproxTol3d) (a)/2.0)

/*******************************************************************//**
PURPOSE: SmTol = SMLib Interface to tolerance values, ScaledZero values, 
         and Degenerate Geometry predicates.  

NOTES: 
   1. Consistent Tolerance Model interface: 

       note 1: SMLib's tolerance interface is encapsulated in the SmTol class static methods.
       note 2: SMLib coding style rule: Use these static methods to code all expressions 
                                         that fetch, check, or compute with tolerances.
       note 3: The SmTol static methods are implemented with many overloaded versions to make 
               calling for tolerance values as simple as possible using whatever information 
               happens to be available at the time.

         // Get Object Tolerances                 // Get Default Context Tolerances 
         SmTol::GetZoneTol3d(pObj)                SmTol::GetZoneTol3d(cpContext, bIsSmall=FALSE) 
         SmTol::GetXSectTol3d(pObj1, pObj2)       SmTol::GetXSectTol3d(cpContent, bIsSmall=FALSE) 
         SmTol::GetApproxTol3d(pObj, pOptObj2)    SmTol::GetApproxTol3d(cpContext, bIsSmall=FALSE)

         // ScaledZero Tolerances
         SmTol::GetScaledZero(pObj)               SmTol::GetScaledZero() or SmTol::GetScaledZero(double) 

         // NLib compatible min dist for unique Param points
         SmTol::GetEffZeroParam()
  
         // Map Tolerances to ParamSpaces 3d=>2d, 3d=>1d
         SmTol2d SmTol::MapTo2d(sTol3d, sUV, sUVdir, sSurface) ;
         SmTol1d SmTol::MapTo1d(sTol3d, dParam, sCurve) ;  

         // Tolerance predicates - (perhaps too much)  
         SmTol::InTol(double, SmTol3d)  
         SmTol::InTol(SmTol3d, double)
         
         SmTol::OutOfTol(double, SmTol3d)
         SmTol::OutOfTol(SmTol3d, double)  
  
         // Predicates to classify geometry based on tolerances and object geometric and parametric properties
           - Degeneracies  -  SmTol::IsDegenerate(pObj)
                              SmTol::IsPointSized(pObj)
                              SmTol::IsSliver(pObj)
                              SmTol::IsNecking(pObj)
                              SmTol::HasSmallCurvatureRadius(pObj)
                              SmTol::HasZeroNormal(pObj)
                              SmTol::HasCoincidentKnots(pObj)
                              SmTol::HasZeroSpeed(pObj)
                           
           - Closed/Open   -  SmTol::IsClosed(pObj)
           - Coincidence   -  SmTol::AreCoincident(pObj1, pObj2)
           - Intersection  -  SmTol::AreIntersecting(pObj1, pObj2)
                           -  SmTol::IsSelfIntersection(pObj1)
           - G1 Continuity -  SmTol::IsG1(pObj) 
           - Singularity   -  SmTol::HasSingularity(pObj)
           - Bad Shape     -  SmTol::HasBadShape(pObj)  // for hooks, cusps, bow ties, etc  

           - Value Metrics -  SmTol::IsG1AngRad(pVec1, pVec2)
                              SmTol::AreIndependentTangents(pVec1, pVec2) 
                              SmTol::IsValidCurvatureRadius(dCurvatureRad) 
                              SmTol::AreParallel(pVec1, pVec2)
                              SmTol::ArePerpendicular(pVec1, pVec2)
                              SmTol::AreCoinPoints3d(pVec3d_1, pVec3d_2, sXSectTol3d)  "coin" = "coinincident"
                              SmTol::AreCoinPoints2d(pVecUV_1, pVecUV_2, sXSectTol2d)  sXSectTol2d = MapTo2d(sXSectTol3d, sUV, sUVdir, sSurface) ;
                              SmTol::AreCoinPoints1d(dParam1, dParam2,   sXSectTol1d)  sXSectTol1d = MapTo1d(sXSectTol3d, dParam, sCurve) ;
                              SmTol::IsZeroSpeed(dSpeed)
                              SmTol::IsZeroNormal(pSurfaceNormal)
                              SmTol::AreSameParam(dParam1,dParam2)  // test to NLib 1.0e-08 constant
                              SmTol::IsOnSeam(pUV, pSurface, pOptZoneTol3d)

  2. Conditional Compile Constants
       Compile with SM_USE_NEWTOL to return tolerances computed to the Consistent Tolerance Model
       Compile with SM_USE_OLDTOL to return tolerances as computed prior to the Consistent Tolerance Model
                                       (Object tolerances stored on each object)

  3. 3d Tolerance Note: 

       SMLib distinguishes between Topological and Geometric tolerances.  The three kinds of topology tolerances
       are ZoneTol3d, ApproxTol3d, and XSectTol3d. Topological ZoneTol3d tolerances define the
       local neighborhood of points considered close to the topology objects within a topology graph.  

       Geometry tolerances are used by geometry calculating methods to compute geometric properties.  
       
       It's often the case that a topology editing method will call geometry methods with tolerances 
       equal to the Topology Tolerances but those methods can also be called with different tolerances 
       depending on the calling algorithms need. Those methods which are editing topology models make 
       SmTol::StaticMethod() calls to fetch topological tolerance values.  Geometric methods using tolerances 
       are passed tolerance values as input arguments and don't make SmTol::StaticMethod() calls.
         
       SMLib uses a naming convention where geometry tolerance names are just prefixed topology tolerance
       names to distinguish between the two very similar cases as:

           Topology Tolerance           Geometry Tolerance
           ZoneTol3d                    ThisZoneTol3d
           ApproxTol3d                  ThisApproxTol3d
           XSectTol3d                   ThisXSectTol3d
           Geometry Specific Tolerance  This'Description'Tol3d
       
       ZoneTol3d 
        - Every Face, Edge, and Vertex has a ZoneTol3d value that defines the size and shape of its
            local neighborhood of point that are considered "close" to the object.
            (example use: Two objects intersect when any portion of their local neighborhoods touch.) 
          o. Old Tolerance Model: Each TopologyObject stores a ZoneTol3d value sized to guarantee that 
                                  connected pair of topology objects within a topology graph intersect 
                                  one another to a tolerance of XSectTol3d(pObj1,pObj2) = ZoneTol3d(pObj1) + ZoneTol3d(pObj2)
          o. New Tolerance Model: Every TopologyObject has the same ZoneTol3d value - stored on the SmContext object.
                                  Vertexuse and Edgeuse objects are also given a local neighborhood centered on 
                                  their GapVectors and sized to the system ZoneTol3d value.

       ApproxTol3d and XSectTol3d tolerances are based on ZoneTol3d tolerances.  
          - an ApproxTol3d tol is a function of one ZoneTol3d tol values as ApproxTol3d = ZoneTol3d / 2
             (ex: A shape approximation is good when MaxDeviation(ApproxGeometry, IdealGeometry) < sApproxTol3d.)
          - an XSectTol3d  tol is a function of two ZoneTol3d tol values as XSectTol3d = ZoneTol3d_1 + ZoneTol3d_2
             (ex: Two objects intersect when their ZoneTol3d sized tolerant neighborhoods intersect)
          - All object degeneracy and property checks are defined in terms of GEOMETRY tolerances 
            commonly used as
               SmTol::Predicate(ThisXSectTol3d ) where callers often but not always set ThisXSectTol3d  = XSectTol3d, 
               SmTol::Predicate(ThisApproxTol3d) where callers often but not always set ThisApproxTol3d = ApproxTol3d, and 
               SmTol::Predicate(ThisZoneTol3d  ) where callers often but not always set ThisZoneTol3d   = ZoneTol3d.

     3a. An Object's ZoneTol3d offset size defines a tolerant neighborhood region 
         whose points are considered near enough to classify to that object. 
         
         use: SmTol::GetZoneTol3d(pObj) and 
              SmTol::GetZoneTol3d(Context, bIsSmallTopology) 

         Old Tolerance Model - ZoneTol3d sized tolerant neighborhoods are designed so that the
              union of a set of connected topology object tolerant neighborhoods
              divide space into a set Venn-diagram-like regions that have the 
              following Point Classification properties: 

              1. Pts classifying to an EdgeEnd      also classify to the connected Vertex.
              2. Pts classifying to a  FaceBoundary also classify to the connected Edge.
              3. Pts classifying to a  FaceCorner   also classify to the connected Vertex and Edges.
                 (For examples: it’s a tolerance error when
                    - a point classifies to an EdgeEnd     without also classifying to the connected Vertex.
                    - a point classifies to a FaceBoundary without also classifying to the connected Edge.
                    - a point classifies to a FaceCorner   without also classifying to the connected EdgeEnds
                                                                                   and the connected Vertex.)

            - to help enforce the classification rules, different default ZoneTol3d values are given objects
              of different domain dimensions.  Vertex ZoneTol3d values are bigger than Edge ZoneTol3d
              values which are bigger than Face ZoneTol3d values.

          New Tolerance Model - ZoneTol3d sized tolerance neighborhoods are a constant size.  Vertex,
              Edge, Face, NonZeroGap-Vertexuse, and NonZeroGap-Edgeuse objects all have tolerant neighborhoods.  
              When an object intersects a Vertexuse (Edgeuse) local neighborhood its considered to be 
              close enough to form a tolerant connection to the Verteuse's Vertex (Edgeuse's Edge).

              When gaps are present in the topology graph:
              1. Pts classifying to an EdgeEnd      may or may not classify to the connected Vertex.
              2. Pts classifying to a  FaceBoundary may or may not classify to the connected Edge.
              3. Pts classifying to a  FaceCorner   may or may not classify to the connected Vertex and Edges.

              When no gaps are present in the topology graph the old and new Topology models become the same and
              1. Pts classifying to an EdgeEnd      also classify to the connected Vertex.
              2. Pts classifying to a  FaceBoundary also classify to the connected Edge.
              3. Pts classifying to a  FaceCorner   also classify to the connected Vertex and Edges.
          
     3b. XSectTol3d = max dist between geometry at which intersections exist.
         It's also the min distance between distinct points.  
         The XSectTol3d value is computed for a pair of objects as the sum of their ZoneTol3d values
         
         use: SmTol::GetXSectTol3d(pObj1, pObj2) and
              SmTol::GetXSectTol3d(Context, bIsSmallTopology1, bIsSmallTopology2) 
              
     3c. ApproxTol3d = MaxDeviatian allowed between an approximate and ideal geometry pair.
         Approx geometry defaults to fitting snugly within the tolerant neighborhood
         of the original geometry. To make that happen ApproxTol3d tolerance values are
         smaller than ZoneTol3d tolerance values.

         use SmTol::GetApproxTol3d(pObj) and 
             SmTol::GetApproxTol3d(Context, bIsSmallTopology)

     3d. The SmTol::GetZoneTol3d(pObj),       
             SmTol::GetXSectTol3d(pObj,pObj2),
             SmTol::GetApproxTol3d(pObj),     
         methods are overloaded so that the SmPoint3d class can be treated as an object just
         like the SmCurve, SmSurface, SmVertex, SmEdge, and SmFace classes.

     3e. New Topology Model:
         Setting system ZoneTol3d system values
           Users supply a single ModelSizeEstimate value when creating an SmContext
           or SmBrep from which SMLib computes and store the default ZoneTol3d value as
              ModelSizeEstimate     ZoneTol3d
              1.0e6 - 1.0e4         1.0e-3   extremely rare
              100   - 10000         1.0e-4   rare
              1     - 100           1.0e-5   default
              .01   - 1             1.0e-6   rare
              .0001 - .01           1.0e-7   extremely rare

     3f. New Topology Model:
         The pinhole in a battleship problem.  It's rare but occasionally
         geometry of vastly different size scales are modeled together.
         To support these models SMLib allows users to insert geometry
         intended to be vastly smaller than the rest of the model.  The user is
         obligated to tell SMLib which geometry is intended to be small.
         Small Vertex, Edge, and Face objects use a boolean value to store their 
         small state.  Smaller ZoneTol3d, XSectTol3d, and ApproxTol3d values
         are returned from SmTol::GetXxxxTol3d(pObj) methods when their
         input argument objects have the IsSmallTopology bit set.


         New Topology Model::
         SMLib is very different from the IGES and STEP Tolerance standards
         in which one tolerance is stored with each object. The problems
         with storing one tolerance with each object include 
           a. the meaning of that one tolerance is ambiguous. Is it
              a ZoneTol3d, ApproxTol3d, XSectTol3d, or a 2D or 1D value?
           b. That value is often stale.  Every method that creates or
              edits geometry has to update tolerance values.  The
              rules for making consistent tolerances are complicated
              and should not be encoded into every method.  Instead,
              SMLib computes run-time tolerances based on the
              run-time gap sizes found between connected objects.

    3g. The SmTol::GetXxxxTol3d() methods support an optional argument 
        to over-ride the default and the local ZoneTol3d values.  
        This feature allows an operation like Stitch() runtime control
        of the default ZoneTol3d value.  The Stitch() operator uses
        this feature to find coincidences between topology objects 
        that have gaps that exceed the runtime default XSectTol3d values.

   5. The Consistent Tolerance Model

     SMLib defines a local neighborhood of points for SmPoint, SmCurve, 
     SmSurface, SmVertex, SmEdge, SmFace, SmVertexuse, and SmEdgeuse objects 
     whose shapes are all sized by one single ZoneTol3d size.

     SmContext and SmBrep objects store defauilt ZoneTol3d values.  Any topology 
     object connected to a topology graph inherits its ZoneTol3d value from its
     containing SmBrep object, otherwise the ZoneTol3d value is inherited from
     the object's Context. Default ZoneTol3d values are set indirectly 
     from a ModelSizeEstimate value defaulted input argument passed to 
     both the SmContext and the SmBrep constructors.

working - need to add notes about the following: Gaps and small angles.
           1. Pts classifying to an EdgeEnd        also classify to the connected Vertex.
           2. Pts classifying to a  FaceBoundary   also classify to the connected Edge.
           3. Pts classifying to a  FaceCorner     also classify to the connected Vertex and Edges.
           4. Pts classifying to two or more Edges also classify to a Vertex connected between the Edges.
           5. Pts classifying to two or more Faces also classify to an Edge  connected between the Faces.

         Gaps and Small Angle Connections make satisfying the consistent classification 
         rules a bit difficult.  

         Small Angle Connections: The two kinds of connection angles are the angle between 
         two edges connecting together at a vertex and the dihedral angle (a function
         of the edge param) between two faces connecting together along an edge. 
         Both kinds of small angle connections form a problem classification sub-region 
         where 3d points classify to both connecting topology objects without also classifying
         to the intermediate bounding object (violating rule 4 or 5 above).
End working 
***********************************************************************/
class SM_EXPORT SmTol
{ 
public:
  // Tolerance Size Interface: 
  //   Specify Model Size Estimate => System Calc's appropriate tolerance size
  //      +---------------------+-----------+------------------+
  //      |     SizeEstimate    | ZoneTol3d | Expected Use     |
  //      +---------------------+-----------+------------------+
  //      |    0.00005-    0.005|  1.0e-7   |   Rare           |
  //      |    0.005  -    0.5  |  1.0e-6   |  Once in a while |
  //      |    0.5    -  500    |  1.0e-5   | Default          |
  //      |  500      -50000    |  1.0e-4   |  Once in a while |   
  //      |50000      -5e6      |  1.0e-3   |   Rare           |
  //      +---------------------+-----------+------------------+
  // ModelSize Sets Default ZoneTol3d - SmTol stores a ModelSize - That's probably the only ModelSize that needs setting
  //                                    SmContext stores a ModelSize - When Created SmContext saves SmTol::ModelSize
  //                                    SmBrep    stores a ModelSize - When Create SmBrep saves SmContext::ModelSize
  SM_NEWTOL_LINE static double      GetModelSizeEstimate  (const SmContext & rContext) ;  // from Context
  SM_NEWTOL_LINE static double      GetModelSizeEstimate  (const SmBrep    & rBrep) ;     // from Brep
                 static double      GetModelSizeEstimate  () ;                            // from SmTol default                                                  

  SM_NEWTOL_LINE static SmZoneTol3d SetModelSizeEstimate  (SmContext & rContext, double dModelSizeEstimate=SM_USE_DEFAULT) ; // set Context::m_dModelSizeEstimate & m_dZoneTol3d 
  SM_NEWTOL_LINE static SmZoneTol3d SetModelSizeEstimate  (SmBrep    & rBrep,    double dModelSizeEstimate=SM_USE_DEFAULT) ; // set Brep::m_dModelSizeEstimate & m_dZoneTol3d 
                 static SmZoneTol3d SetModelSizeEstimate  (double dModelSizeEstimate=SM_MODEL_SIZE_ESTIMATE) ;     

  //   SystemLargeSmallSizeRatio: Set Tolerances for pinholes in Battleships
  //                              only change if you're an expert
  SM_NEWTOL_LINE static double      GetLargeSmallSizeRatio(const SmContext & rContext) ;  // from Context
  SM_NEWTOL_LINE static double      GetLargeSmallSizeRatio(const SmBrep    & rBrep) ;     // from Brep
                 static double      GetLargeSmallSizeRatio() ;                            // from SmTol default                          

  SM_NEWTOL_LINE static void        SetLargeSmallSizeRatio(SmContext & rContext, double dLargeSmallSizeRatio=SM_USE_DEFAULT) ; // set Context::m_dLargeSmallSizeRatio
  SM_NEWTOL_LINE static void        SetLargeSmallSizeRatio(SmBrep    & rBrep,    double dLargeSmallSizeRatio=SM_USE_DEFAULT) ; // set Brep::m_dLargeSmallSizeRatio
  static void                       SetLargeSmallSizeRatio(double dLargeSmallSizeRatio=SM_LARGE_SMALL_SIZE_RATIO) ; 

  // helper functions - not usually needed

  // obsolete: No Need for this kind of rounding off - the SizeToModelSizeEstimate() and the SizeEstimateToZoneTol3d()
  // encode the same ModelSize_to_ZoneTol3d tables
  // round off a size value to a generalized ModelSizeEstimate
  //  static double SizeToModelSizeEstimate(double dSize) { return   (dSize <= 500   && dSize > .5)     ?     50
  //                                                               : (dSize <= 50000 && dSize > 500)    ?   5000
  //                                                               : (dSize <= .5    && dSize > .005)   ?      0.05
  //                                                               : (dSize <= 5e6   && dSize > 50000)  ? 500000
  //                                                               : (dSize <= .005  && dSize > .00005) ?      0.0005
  //                                                               : dSize ; 
  //                                                      }
  // map ModelSizeEst to ZoneTol3d 
  static SmZoneTol3d SizeEstimateToZoneTol3d(double dModelSizeEstimate) 
                      { return(  (dModelSizeEstimate == (double)SM_USE_DEFAULT) ? SizeEstimateToZoneTol3d((double)SM_MODEL_SIZE_ESTIMATE)
                               : (dModelSizeEstimate >=     0.5     &&    500     >= dModelSizeEstimate) ? ((SmZoneTol3d)1.0e-5)
                               : (dModelSizeEstimate >=   500       &&  50000     >= dModelSizeEstimate) ? ((SmZoneTol3d)1.0e-4)
                               : (dModelSizeEstimate >=     0.005   &&      0.5   >= dModelSizeEstimate) ? ((SmZoneTol3d)1.0e-6) 
                               : (dModelSizeEstimate >= 50000       &&    5e6     >= dModelSizeEstimate) ? ((SmZoneTol3d)1.0e-3)
                               : (dModelSizeEstimate >=     0.00005 &&      0.005 >= dModelSizeEstimate) ? ((SmZoneTol3d)1.0e-7)
                               : (SmZoneTol3d)(smos_Pow(10,floor(log10(smos_Fabs(dModelSizeEstimate))/2.0))*1.0e-5)) ;
                      }    
  // obsolete: 
  // static double ZoneTol3dToSizeEstimate(SmZoneTol3d sZoneTol3d)    { return (pow(10,floor(log10(sZoneTol3d / 1.0e-5))*2)*50) ; }  

  // Tolerance Size Interface: 
  //   Specify Model Size Estimate => System Calc's appropriate tolerance size
  //      +---------------------+-----------+------------------+
  //      |     SizeEstimate    | ZoneTol3d | Expected Use     |
  //      +---------------------+-----------+------------------+
  //      |    0.00005-    0.005|  1.0e-7   |   Rare           |
  //      |    0.005  -    0.5  |  1.0e-6   |  Once in a while |
  //      |    0.5    -  500    |  1.0e-5   | Default          |
  //      |  500      -50000    |  1.0e-4   |  Once in a while |   
  //      |50000      -5e6      |  1.0e-3   |   Rare           |
  //      +---------------------+-----------+------------------+
  //   SystemLargeSmallSizeRatio: Set Tolerances for pinholes in Battleships
  //                              only change if you're an expert

  // check Vals against tolerances (note: InTol(x,y) == !OutOfTol(x,y))
  //   gwc: these are not exactly the same as the operator< and operator> inline functions defined above
  //        but close enough so maybe we should just remove these to simplify the user interface
  static SmBoolean InTol    (double dVal, const SmTol3d & crTolVal) { SM_ASSERT_TOL(crTolVal.val) ; return smos_Fabs(dVal) <= crTolVal.val ; }
  static SmBoolean InTol    (const SmTol3d & crTolVal, double dVal) { SM_ASSERT_TOL(crTolVal.val) ; return smos_Fabs(dVal) <= crTolVal.val ; }
                                                                            
  static SmBoolean OutOfTol (const SmTol3d & crTolVal, double dVal) { SM_ASSERT_TOL(crTolVal.val) ; return smos_Fabs(dVal) >  crTolVal.val ; }
  static SmBoolean OutOfTol (double dVal, const SmTol3d & crTolVal) { SM_ASSERT_TOL(crTolVal.val) ; return smos_Fabs(dVal) >  crTolVal.val ; }

  // calc TopologyObject ZoneTol3d values for Vertices, Edges, and Faces given MaxGap3d values
  //  current GapBased Tol Model : FaceZoneTol3d   = DefZoneTol3d
  //                               EdgeZoneTol3d   = Max(DefZoneTol3d,
  //                                                     MaxZoneTolOfConnectedFaces,
  //                                                     GapGain * MaxGap_VertexEdge / 2.0,
  //                                                     GapGain * MaxGap_VertexFace - sDefZoneTol3d)
  //                               VertexZoneTol3d = Max(DefZoneTol3d,
  //                                                     MaxZoneTolOfConnectedEdges,
  //                                                     GapGain * MaxGap_EdgeEdge / 2.0,
  //                                                     GapGain * MaxGap_EdgeFace - sDefZoneTol3d)
  //  Question not yet answered  : What should DefZoneTol3d be? Derived from Context or Brep?
  //                               Currently both methods exist, but Brep methods are called in Healer.
  //                               Multiple Breps with different Model Size Estimates can exist in one Context.
  //                        JGU  : Topology should derive their tolerances from their Breps Model Size Estimate.
  static SmZoneTol3d CalcFaceZoneTol3d(const SmContext * cpContext=NULL)                                  
                         { return( SmTol::GetZoneTol3d(cpContext) ) ; }
  static SmZoneTol3d CalcFaceZoneTol3d(SmBrep * pBrep)                                  
                         { return( SmTol::GetZoneTol3d(pBrep) ) ; }

  static SmZoneTol3d CalcEdgeZoneTol3d  (double dEdgeEdge_MaxGapLength3d,
                                         double dEdgeFace_MaxGapLength3d,
                                         SmZoneTol3d     * pMaxFaceZoneTol3d=NULL,
                                         const SmContext * cpContext        =NULL)
                         { SmZoneTol3d sDefZoneTol3d = GetZoneTol3d(cpContext) ; 
                           double      dGapGain      = GetGapGain() ;
                           SmZoneTol3d sRtn = ( pMaxFaceZoneTol3d ) ?
                                              smos_4Max(sDefZoneTol3d.val,
                                                        pMaxFaceZoneTol3d->val,
                                                        dGapGain * dEdgeEdge_MaxGapLength3d / 2.0,
                                                        dGapGain * dEdgeFace_MaxGapLength3d - sDefZoneTol3d) :
                                              smos_3Max(sDefZoneTol3d.val,
                                                        dGapGain * dEdgeEdge_MaxGapLength3d / 2.0,
                                                        dGapGain * dEdgeFace_MaxGapLength3d - sDefZoneTol3d) ;
                           return(sRtn) ;
                         }
  static SmZoneTol3d CalcEdgeZoneTol3d  (double dEdgeEdge_MaxGapLength3d,
                                         double dEdgeFace_MaxGapLength3d,
                                         SmBrep       * pBrep,
                                         SmZoneTol3d  * pMaxFaceZoneTol3d=NULL)
                         { SmZoneTol3d sDefZoneTol3d = GetZoneTol3d(pBrep) ; 
                           double      dGapGain      = GetGapGain() ;
                           SmZoneTol3d sRtn = ( pMaxFaceZoneTol3d ) ?
                                              smos_4Max(sDefZoneTol3d.val,
                                                        pMaxFaceZoneTol3d->val,
                                                        dGapGain * dEdgeEdge_MaxGapLength3d / 2.0,
                                                        dGapGain * dEdgeFace_MaxGapLength3d - sDefZoneTol3d) :
                                              smos_3Max(sDefZoneTol3d.val,
                                                        dGapGain * dEdgeEdge_MaxGapLength3d / 2.0,
                                                        dGapGain * dEdgeFace_MaxGapLength3d - sDefZoneTol3d) ;
                           return(sRtn) ;
                         }

  static SmZoneTol3d CalcVertexZoneTol3d(double dVertexEdge_MaxGapLength3d,
                                         double dVertexFace_MaxGapLength3d,
                                         SmZoneTol3d     * pMaxEdgeZoneTol3d=NULL,
                                         const SmContext * cpContext        =NULL)
                         { SmZoneTol3d sDefZoneTol3d = GetZoneTol3d(cpContext) ; 
                           double      dGapGain      = GetGapGain() ;
                           SmZoneTol3d sRtn = ( pMaxEdgeZoneTol3d ) ?
                                              smos_4Max(sDefZoneTol3d.val,
                                                        pMaxEdgeZoneTol3d->val,
                                                        dGapGain * dVertexEdge_MaxGapLength3d / 2.0,
                                                        dGapGain * dVertexFace_MaxGapLength3d - sDefZoneTol3d) :
                                              smos_3Max(sDefZoneTol3d.val,
                                                        dGapGain * dVertexEdge_MaxGapLength3d / 2.0,
                                                        dGapGain * dVertexFace_MaxGapLength3d - sDefZoneTol3d) ;
                           return(sRtn) ;
                         }
  static SmZoneTol3d CalcVertexZoneTol3d(double dVertexEdge_MaxGapLength3d,
                                         double dVertexFace_MaxGapLength3d,
                                         SmBrep      * pBrep,
                                         SmZoneTol3d * pMaxEdgeZoneTol3d=NULL)
                         { SmZoneTol3d sDefZoneTol3d = GetZoneTol3d(pBrep) ; 
                           double      dGapGain      = GetGapGain() ;
                           SmZoneTol3d sRtn = ( pMaxEdgeZoneTol3d ) ?
                                              smos_4Max(sDefZoneTol3d.val,
                                                        pMaxEdgeZoneTol3d->val,
                                                        dGapGain * dVertexEdge_MaxGapLength3d / 2.0,
                                                        dGapGain * dVertexFace_MaxGapLength3d - sDefZoneTol3d) :
                                              smos_3Max(sDefZoneTol3d.val,
                                                        dGapGain * dVertexEdge_MaxGapLength3d / 2.0,
                                                        dGapGain * dVertexFace_MaxGapLength3d - sDefZoneTol3d) ;
                           return(sRtn) ;
                         }

  // geometry - Degeneracies  
  //          - Closed/Open   
  //          - Coincidence                              
  //          - Intersection                                                                    
  //          - G1 Continuity 
  //          - Singularity 
  //          - Bad Shape - for hooks, cusps, bow ties, etc
  //          - Bad Shape - for poor knot vectors
  static SmBoolean IsDegenerate      (const SmObject   * pObj,                  SmXSectTol3d * pMyXSectTol3d  = NULL) ; // TRUE = pObj.BBox.GetMaxDim < GetScaledZero(pObj)
  static SmBoolean IsDegenerate      (const SmCurve    * pCrv, SmExtent1d sIvl, SmXSectTol3d * pMyXSectTol3d  = NULL) ;
  static SmBoolean IsDegenerate      (const SmVector2d   sVec,                  SmXSectTol3d * pMyXSectTol3d  = NULL) ; 
  static SmBoolean IsDegenerate      (const SmVector3d   sVec,                  SmXSectTol3d * pMyXSectTol3d  = NULL) ; 
  static SmBoolean IsPointSized      (const SmObject   * pObj,                  SmZoneTol3d  * pMyZoneTol3d   = NULL) ; // TRUE = pObj.BBox.GetMaxDim < GetZoneTol3d(pObj)
  static SmBoolean IsSliver          (const SmObject   * pObj,                  SmXSectTol3d * pMyXSectTol3d  = NULL) { ERR_MSG(_T("NOT IMPLEMENTED")) ; SM_REF2(pObj, pMyXSectTol3d  ) ; return(FALSE) ; }
  static SmBoolean IsNecking         (const SmObject   * pObj,                  SmXSectTol3d * pMyXSectTol3d  = NULL) { ERR_MSG(_T("NOT IMPLEMENTED")) ; SM_REF2(pObj, pMyXSectTol3d  ) ; return(FALSE) ; }
  static SmBoolean HasSmallRadius    (const SmObject   * pObj,                  SmZoneTol3d  * pMyZoneTol3d   = NULL) { ERR_MSG(_T("NOT IMPLEMENTED")) ; SM_REF2(pObj, pMyZoneTol3d   ) ; return(FALSE) ; }
  static SmBoolean HasZeroNormal     (const SmObject   * pObj,                  SmScaledZero * pMyScaledZero  = NULL) { ERR_MSG(_T("NOT IMPLEMENTED")) ; SM_REF2(pObj, pMyScaledZero  ) ; return(FALSE) ; }
  static SmBoolean HasZeroSpeed      (const SmObject   * pObj,                  SmScaledZero * pMyScaledZero  = NULL) { ERR_MSG(_T("NOT IMPLEMENTED")) ; SM_REF2(pObj, pMyScaledZero  ) ; return(FALSE) ; }
  static SmBoolean HasCoincidentKnots(const SmObject   * pObj,                  SmScaledZero * pMyEffZeroParam= NULL) { ERR_MSG(_T("NOT IMPLEMENTED")) ; SM_REF2(pObj, pMyEffZeroParam) ; return(FALSE) ; }
  
  static SmBoolean AreCoincident        (const SmObject * pObj1, const SmObject * pObj2,                         
                                         SmXSectTol3d   * pOptOverrideXSectTol3d=NULL)   { SM_ASSERT_TOLPTR(pOptOverrideXSectTol3d) ; ERR_MSG(_T("NOT IMPLEMENTED")) ; SM_REF3(pObj1, pObj2, pOptOverrideXSectTol3d) ; return(FALSE) ; }
  static SmBoolean AreIntersecting      (const SmObject * pObj1, const SmObject * pObj2, 
                                         SmXSectTol3d   * pOptOverrideXSectTol3d=NULL)   { SM_ASSERT_TOLPTR(pOptOverrideXSectTol3d) ; ERR_MSG(_T("NOT IMPLEMENTED")) ; SM_REF3(pObj1, pObj2, pOptOverrideXSectTol3d) ; return(FALSE) ; }
  static SmBoolean IsSelfIntersection   (const SmObject * pObj,                            
                                         SmXSectTol3d   * pOptOverrideXSectTol3d=NULL)   { SM_ASSERT_TOLPTR(pOptOverrideXSectTol3d) ; ERR_MSG(_T("NOT IMPLEMENTED")) ; SM_REF2(pObj, pOptOverrideXSectTol3d) ; return(FALSE) ; }
  static SmBoolean IsClosed             (const SmObject * pObj, 
                                         SmXSectTol3d   * pOptOverrideXSectTol3d=NULL)   ;
  static SmBoolean IsG1                 (const SmObject * pObj, 
                                         SmScaledZero   * pOptOverrideEffZeroAngRad=NULL){ SM_ASSERT_TOLPTR(pOptOverrideEffZeroAngRad) ; ERR_MSG(_T("NOT IMPLEMENTED")) ; SM_REF2(pObj, pOptOverrideEffZeroAngRad) ; return(FALSE) ; }
  static SmBoolean HasSingularity       (const SmObject * pObj, 
                                         SmXSectTol3d   * pOptOverrideXSectTol3d=NULL, 
                                         SmScaledZero   * pOptOverrideScaledZero=NULL )  { SM_ASSERT_TOLPTR2(pOptOverrideXSectTol3d, pOptOverrideScaledZero) ; ERR_MSG(_T("NOT IMPLEMENTED")) ; SM_REF3(pObj, pOptOverrideXSectTol3d, pOptOverrideScaledZero) ; return(FALSE) ; }
  static SmBoolean HasBadShape          (const SmObject * pObj, 
                                         SmZoneTol3d    * pOptOverrideZoneTol3d=NULL)    { SM_ASSERT_TOLPTR(pOptOverrideZoneTol3d) ; ERR_MSG(_T("NOT IMPLEMENTED")) ; SM_REF2(pObj, pOptOverrideZoneTol3d) ; return(FALSE) ; } 
  static SmBoolean IsPoorlyParameterized(const SmObject * pObj,
                                         SmScaledZero   * pOptOverrideScaledZero=NULL)    { SM_ASSERT_TOLPTR(pOptOverrideScaledZero) ; ERR_MSG(_T("NOT IMPLEMENTED")) ; SM_REF2(pObj, pOptOverrideScaledZero) ; return(FALSE) ; }

  // Value Metrics 
  static SmBoolean    IsG1AngRad            (SmVector3d *pVec1, SmVector3d *pVec2, SmScaledZero *pOptOverrideEffZeroAngRad=NULL) ;
  static SmBoolean    IsRadiusSmall         (double dCurvatureRad,                 SmZoneTol3d  *pOptOverrideZoneTol3d=NULL) ;
  static SmBoolean    AreParallel           (SmVector3d *pVec1, SmVector3d *pVec2, double *pOptOverrideEffZeroAngRad=NULL) ;
  static SmBoolean    ArePerpendicular      (SmVector3d *pVec1, SmVector3d *pVec2, double *pOptOverrideEffZeroAngRad=NULL) ;
  static SmBoolean    AreIndependentTangents(SmVector3d *pVec1, SmVector3d *pVec2, SmScaledZero *pOptOverrideScaledZero=NULL) ;    
                                                                                                                                 
                                                                                                                                 
  static SmBoolean    IsZeroSpeed           (double      dSpeed,  double dMaxPosDimension, SmScaledZero *pOptOverrideScaledZero=NULL) ;
  static SmBoolean    IsZeroNormal          (SmVector3d *pSurfaceNormal,            SmScaledZero *pOptOverrideScaledZero=NULL) ;   // currently a stub function
  static SmBoolean    AreSameParam          (double      dParam1, double dParam2,   SmScaledZero *pOptOverrideEffZeroParam=NULL) ;  /* test to NLib 1.0e-08 constant */
  static SmBoolean    IsOnSeam              (const SmPoint2d &crUV,  const SmSurface &crSurface, SmZoneTol3d *pOptOverrideZoneTol3d=NULL) ;
                      
  static SmBoolean    AreCoinPoints3d       (SmPoint3d  *pPt1,  SmPoint3d *pPt2,   SmXSectTol3d *pOptOverrideXSectTol3d=NULL) ;   // currently a stub function
  static SmBoolean    AreCoinPoints2d       (SmPoint3d  *pPt1,  SmPoint3d *pPt2,   SmTol2d sZoneTol2d_1, SmTol2d sZoneTol2d_2) ;  // currently a stub function
  static SmBoolean    AreCoinPoints1d       (double     *pPt1,  double    *pPt2,   SmTol1d sZoneTol1d_1, SmTol1d sZoneTol1d_2) ;  // currently a stub function
      // expected value = sZoneTol1d = MapTo1d(sZoneTol3d, dParam, sCurve) ; 
      // expected value = sZoneTol2d = MapTo2d(sZoneTol3d, sUV, sUVdir, sSurface) ; 

  // Constant Tolerances  NLib's MinDist between unique knot values, Angle Tolerances
  static SmScaledZero GetEffZero          () { return (SmScaledZero)SM_EFF_ZERO ; }
  static SmScaledZero GetEffZeroSq        () { return (SmScaledZero)SM_EFF_ZERO_SQ ; }
  static SmScaledZero GetEffZeroParam     () { return (SmScaledZero)SM_EFF_ZERO_PARAM ; }
  static SmScaledZero GetEffZeroAngRad    () { return (SmScaledZero)SM_EFF_ZERO_RAD ; }    // these angles are the same today but likely to be different
  static SmScaledZero GetEffG1AngRad      () { return (SmScaledZero)SM_EFF_ZERO_RAD ; }    // by the time the NewTolerance model is complete
  static SmScaledZero GetEffParallelAngRad() { return (SmScaledZero)SM_EFF_ZERO_RAD ; }

  static double GetGapGain() ; // return fudge scale size used to make sure an Obj's stored ZoneTol3d is a tad larger than its largest Gap3d value/2.0

  // Topology Tolerances: System Default and Obj Specific 
  //   ZoneTol3d   = Topo Obj's tolerant neighborhood offset size
  //   XSectTol3d  = max dist between topo Objs at which intersections exist (min dist between distinct points)
  //   ApproxTol3d = max gap limit between an approx geometry and its original geometry

  // Places to get ZoneTol3d values
  //  System  ZoneTol3d - SmTol::GetZoneTol3d()         Set with SetSystemModelSizeEstimate()
  //  Context ZoneTol3d - SmTol::GetZoneTol3d(rContext) default:[System ZoneTol3d], override when constructed or SmContext::SetModelSizeEstimate()
  //  Brep    ZoneTol3d - SmTol::GetZoneTol3d(rBrep)    default:[System ZoneTol3d], override when constructed, read, or SmBrep::SetModelSizeEstimate()
  //  Object  ZoneTol3d - SmTol::GetZoneTol3d(rObj)     equals first available, Brep, Context, or System ZoneTol3d val
  //     note: bIsSmallTopology  // in : For rarely modeled Pinholes in Battleships
  //                             //      FALSE = return normal ZoneTol3d
  //                             //      TRUE  = (rarely used) return SmallTopology ZoneTol3d, 
  //                             //      default:[FALSE]
  static SmZoneTol3d GetZoneTol3d(                               SmSmallTopoType eIsSmallTopology=SM_ST_NORMAL_SIZE) ; // ZoneTol3d = System  ZoneTol3d       
  static SmZoneTol3d GetZoneTol3d(const SmContext  * cpContext,  SmSmallTopoType eIsSmallTopology=SM_ST_NORMAL_SIZE) ; // ZoneTol3d = Context ZoneTol3d, // NotUsed: in : eIsSmallTopology
  static SmZoneTol3d GetZoneTol3d(const SmObject   * cpObj,      SmSmallTopoType eIsSmallTopology=SM_ST_NORMAL_SIZE) ; // ZoneTol3d = Object  ZoneTol3d  // NotUsed: in : eIsSmallTopology
                                                                                                                       //              or Object's ThisZoneTol3d if any
  static void        SetZoneTol3d(const SmObject   * cpObj,      SmZoneTol3d sNewZoneTol3d, SmSmallTopoType eIsSmallTopology=SM_ST_NORMAL_SIZE) ;
                                                                 
  static SmZoneTol3d GetZoneTol3d(const SmBrep     * cpBrep,     SmSmallTopoType eIsSmallTopology=SM_ST_NORMAL_SIZE) ; // ZoneTol3d = Brep    ZoneTol3d  // NotUsed: in : eIsSmallTopology
  static SmZoneTol3d GetZoneTol3d(const SmBrepData * cpBrepData, SmSmallTopoType eIsSmallTopology=SM_ST_NORMAL_SIZE) ; // ZoneTol3d = Brep    ZoneTol3d  // NotUsed: in : eIsSmallTopology
  static SmZoneTol3d GetZoneTol3d(const SmPoint3d  * cpPoint,    SmBrep        * pOptBrep=NULL,                        // ZoneTol3d =  Brep  ? BrepZoneTol3d, // NotUsed: in : cpPoint
                                                                 SmContext     * pOptContext=NULL,                     //            : Context? ContextZoneTol3d
                                                                 SmSmallTopoType eIsSmallTopology=SM_ST_NORMAL_SIZE) ; //            : System ZoneTol3d
  static SmZoneTol3d GetZoneTol3d(SmXSectTol3d & crXSectTol3d) { return crXSectTol3d / 2.0 ; }

  static SmZoneTol3d GetSrcZoneTol3d(const SmCurveClassification * cpCC) ;         // SrcZoneTol3d = ZoneTol3d of Curve being classified
  static SmZoneTol3d GetSrcZoneTol3d(const SmPointClassification * cpPC) ;         // SrcZoneTol3d = ZoneTol3d of Curve being classified
  static SmZoneTol3d GetSrcZoneTol3d(const SmCurveInterval       * cpIC) ;         // SrcZoneTol3d = ZoneTol3d of IC->m_vMid.SrcZoneTol3d
  
  static SmZoneTol3d GetObjZoneTol3d(const SmCurveInterval       * cpIC);         // ObjZoneTol3d = ZoneTol3d of IC->m_vMid.m_cpObj
  static SmZoneTol3d GetObjZoneTol3d(const SmPointClassification * cpPC) ;         // ObjZoneTol3d = ZoneTol3d of PC->m_cpObj


  static SmZoneTol3d GetSrcZoneTol3d(const SmLineSegClassification   * cpLSC) ;  // SrcZoneTol3d = ZoneTol3d of LineSeg being classified
  static SmZoneTol3d GetSrcZoneTol3d(const SmPolyPointClassification * cpPPC) ;  // SrcZoneTol3d = ZoneTol3d of LineSeg being classified
  static SmZoneTol3d GetSrcZoneTol3d(const SmLineSegInterval         * cpLSI) ;  // SrcZoneTol3d = ZoneTol3d of LSI->m_vMid.SrcZoneTol3d
  
  static SmZoneTol3d GetObjZoneTol3d(const SmPolyPointClassification * cpPPC) ;  // ObjZoneTol3d = ZoneTol3d of PC->m_cpObj
  static SmZoneTol3d GetObjZoneTol3d(const SmLineSegInterval         * cpIvC) ;  // ObjZoneTol3d = ZoneTol3d of IC->m_vMid.m_cpObj

  // ZoneTol3d for the result object of an Intersection of two objects
  static SmZoneTol3d GetZoneTol3d_ForXSectResult(const SmZoneTol3d & crInput1_ZoneTol3d,
                                                 const SmZoneTol3d & crInput2_ZoneTol3d) { return smos_Max(crInput1_ZoneTol3d, crInput2_ZoneTol3d) ; }
  static SmZoneTol3d GetZoneTol3d_ForXSectResult(const SmObject  * cpObj1, // needs to be extended for IsSmall values
                                                 const SmObject  * cpObj2) { return smos_Max(GetZoneTol3d(cpObj1), GetZoneTol3d(cpObj2)) ; }

  // sApproxTol3d
  static SmApproxTol3d GetApproxTol3d(                             SmSmallTopoType eIsSmallTopology=SM_ST_NORMAL_SIZE) ; // for System  ApproxTol3d 
  static SmApproxTol3d GetApproxTol3d(const SmContext * cpContext, SmSmallTopoType eIsSmallTopology=SM_ST_NORMAL_SIZE) ; // for Context ApproxTol3d
  static SmApproxTol3d GetApproxTol3d(const SmBrep    * cpBrep,    SmSmallTopoType eIsSmallTopology=SM_ST_NORMAL_SIZE) ; // for Brep ApproxTol3d
  static SmApproxTol3d GetApproxTol3d(const SmObject  * cpObj,     SmSmallTopoType eIsSmallTopology=SM_ST_NORMAL_SIZE) ; // for Object ApproxTol3d
                                                                                                                         // SmFilletCorner::ThisApproxTol3d
                                                                                                                         // SmFilletSolver::ThisApproxTol3d
  static SmApproxTol3d GetApproxTol3d(SmXSectTol3d & crXSectTol3d) { return crXSectTol3d / 4.0 ; }

  // for classes that store ThisApproxTol3d not derived from SmObject,... 
  static SmApproxTol3d GetApproxTol3d(const SmOffsetGeometryCreation * cpCreation) ;
  static SmApproxTol3d GetApproxTol3d(const SmHCR                    * cpHCR) ;
  static SmApproxTol3d GetApproxTol3d(const SmPolyIntersector        * cpPolyIntersector) ;
  static SmApproxTol3d GetApproxTol3d(const SmSurfaceIntersector     * cpSurfaceIntersector) ;
  static SmApproxTol3d GetApproxTol3d(const SmSurfaceTracer          * cpSurfaceTracer) ;
  static SmApproxTol3d GetApproxTol3d(const SmTopologyIntersector    * cpTopologyIntersector) ;

  // sXSectTol3d
  static SmXSectTol3d GetXSectTol3d(                               SmSmallTopoType eIsSmallTopology=SM_ST_NORMAL_SIZE) ;
  static SmXSectTol3d GetXSectTol3d(const SmContext  * cpContext,  SmSmallTopoType eIsSmallTopology=SM_ST_NORMAL_SIZE) ;
                                                     
  static SmXSectTol3d GetXSectTol3d(const SmBrep     * cpBrep,     SmSmallTopoType eIsSmallTopology=SM_ST_NORMAL_SIZE) ;
  static SmXSectTol3d GetXSectTol3d(const SmBrepData * cpBrepData, SmSmallTopoType eIsSmallTopology=SM_ST_NORMAL_SIZE) ;


  static SmXSectTol3d GetXSectTol3d(const SmObject   * cpObj1,                  
                                    const SmObject   * cpObj2,                  
                                    SmContext        * pOptContext=NULL,                     // NotUsed: in : only used when crObjs can't find a Brep or Context
                                    SmSmallTopoType    eIsSmallTopology=SM_ST_NORMAL_SIZE) ; // in : only used when crObjs can't find a Brep or Context

  // for classes that store ThisTol3d values ,...
  static SmXSectTol3d GetXSectTol3d(const SmPointClassification * cpPointClassification) ;  // rtn XSectTol3d(SrcZoneTol3d,ObjZoneTol3d)

  // build XSectTol3d from ZoneTol3ds                                           
  static SmXSectTol3d GetXSectTol3d(const SmZoneTol3d & crZoneTol1,
                                    const SmZoneTol3d & crZoneTol2) { return( crZoneTol1 + crZoneTol2 ) ; }

  // for classes that store ThisAngTolRad ,... 
  static double GetAngTolRad()                            { return SM_ANG_TOL_RAD ; }  // from the System

  static double GetAngTolRad(const SmFilletSolver         *cpFilletSolver) ;

  static double GetAngTolRad(const SmHCR                  *cpHCR) ;
  static double GetAngTolRad(const SmPolyIntersector      *cpPolyIntersector) ;
  static double GetAngTolRad(const SmSurfaceIntersector   *cpSurfaceIntersector) ;
  static double GetAngTolRad(const SmSurfaceTracer        *cpSurfaceTracer) ;
  static double GetAngTolRad(const SmTopologyIntersector  *cpTopologyIntersector) ;

  // Map Tol3d to Surface UVSpace Tol2d values
    // Map Tol3d to Tol2d(SurfaceUVPt, SurfUVDir, Surface) 
    static SmTol2d MapTo2d(const SmTol3d    & rTol3d,         // in : 3d Tol value to map to UV Space
                           const SmVector2d & rUV,            // in : UV point on Surface
                           const SmVector2d & rUVDir,         // in : unit-vector UV direction of interest for this tolerance
                           const SmSurface  & rSurface) ;     // in : Surface mapping UVTrimCurve to 3dSpace
                                  
    // Map Tol3d to Tol2d(SurfaceUVPt, Surface) : return(sTol3d), without UVDir returns Min(Tol2dU, Tol2dV)
    static SmTol2d MapTo2d(const SmTol3d    & rTol3d,            // in : 3d Tol value to map to UV Space
                           const SmVector2d & rUV,               // in : UV point on Surface
                           const SmSurface  & rSurface) ;        // in : Surface mapping UVTrimCurve to 3dSpace
                                  
    //  Map Tol3d to Tol2d(UVTrimCurveParam, UVTrimCurve, Surface)
    static SmTol2d MapTo2d(const SmTol3d   & rTol3d,            // in : 3d Tol value to map to UV Space
                           double            dParam,            // in : target UVTrimCurve param
                           const SmCurve   & rTrimCurve,        // in : when Dim==2, TrimCurve is treated as a UVTrimCurve
                                                                //           Dim==3, TrimCurve is evaluated and its pt and Tan are dropped to Surface UVSpace
                           const SmSurface & rSurface) ;        // in : Surface mapping UVTrimCurve to 3dSpace
  
  // map Tol3d to Curve ParamSpace Tol1d values
    // Map Tol3d to Tol1d(CurveParam, Curve)
    static SmTol1d MapTo1d(SmTol3d       & rTol3d,          // in : 3d Tol value to map to Param Space
                           double          dParam,          // in : target point on Curve
                           const SmCurve & rCurve) ;        // in : Curve mapping ParamSpace to 3dSpace
           
    // Map Tol3d to Tol1d(UVTrimCurveParam, UVTrimCurve, Surface)
    static SmTol1d MapTo1d(SmTol3d         & rTol3d,        // in : 3d Tol value to map to Param Space
                           double            dParam,        // in : target UVTrimCurve param
                           const SmCurve   & rUVTrimCurv,   // in : The UVTrimCurve
                           const SmSurface & rSurface) ;    // in : Surface mapping UVTrimCurve to 3dSpace
  
    // Map Tol3d to Tol1d(Curve1stDeriv, CurveIvl)
    static SmTol1d MapTo1d(SmTol3d          & rTol3d,        // in : 3d Tol value to map to Param Space
                           SmVector3d       & r1stDeriv,     // in : Curve 1st Deriv at the pt where the tol is being used
                           const SmExtent1d & rNaturalIvl) ; // in : ivl limits map size when r1stDeriv is very small (near singularities)   
           
  // Mapping Tolerance NOTES:  
  //  1. Tol2d in UVSpace for a Surface is a directional property depending on a UVPoint and a UVDir 
  //        (it's a 1st order tensor: it changes for different UV dirs and UV points)
  //     Tol2d = dirU * SmTol2d.U + dirV * SmTol2d.V
  //  
  //     where : SmTol2d.U =   (Mag(rDU) > 0.0 && !NearSingularityU) 
  //                          ? (Tol3d / Mag(rDU)) 
  //                          : (Surf.UVDomain.U.Length/100) ;
  //             SmTol2d.V =   (Mag(rDV) > 0.0 && !NearSingularityV) 
  //                          ? (3dTol / Mag(rDV)) 
  //                          : (Surf.UVDomain.V.Length/100) ;
  //     where :  Surf    = The surface mapping UV Pts to 3dPts
  //              UVPt    = [U V] a UV point within Surf.UVDomain.
  //              UVdir   = [dirU dirV] a unit vector2d specifying UV direction for this UVTol
  //              rDU     = 1stDeriv in U dir of Surf.Evaluate1stDerivs(UVpt, rDU, rDV) ;
  //              rDV     = 1stDeriv in V dir of Surf.Evaluate1stDerivs(UVpt, rDU, rDV) ;
  //
  //  2. Tol1d in ParamSpace for a Curve only depends on the Param point
  //     Tol1d =   Mag(1stDeriv(Param)) > 0.0
  //             ? (Tol3d / Mag(1stDeriv(Param))
  //             : (Curve.NaturalInterval.Length/100)
  //
  //  3. Near Singularities
  //       Mapped Tol Values become very large near surface or Curve singularities where 1stDerivs go to zero.
  //       Very large Mapped Tol Values are limited to be much smaller than the Curve or Surface domain size.


  // Update the tolerance value on a Face, Edge, or Vertex.
  // GWC:TODO - replace this block of code with a call to the pTopo->RefreshTolerance() - call made after topo-graph connections are made
  static SmStatus UpdateObjectTolerance( SmTopology * pTopo,              // in: topology object to update
                                         double       dThisGap=0,         // in: if there is a new-discovered gap,
                                                                          //     which could cause tol to increase
                                         double       dThisTol=0,         // in: any existing, default to use
                                         SmBoolean    bCheckGaps=FALSE) ; // in: if True, recompute gaps

 // HELPER FUNCTIONS

#ifdef NOT_BEING_USED // not yet used - perhaps obsolete  
  // Get largest ZoneTol3d and Gap values for this pTopo's UpDim Objects
  static SmZoneTol3d GetMaxUpDimZoneTol3d           // rtn: Max UpDim DimGain*ZoneTol3d value
   (const SmTopology * pTopo,                       // in : Target Vertex, Edge, or Face
    SmGap            & rMaxUpDimGap) ;              // out: Edge  : Max(Edge/Face Gaps)
                                                    //      Vertex: Max(Vertex/Edge Gaps, Vertex/Face Gaps)
  
  // rtn TRUE when any UpDimObject uses a LocalZoneTol3d value
  static SmBoolean HasUpDimLocalZoneTol3d           // rtn: TRUE=a MaxUpDim obj uses a LocalDefTol3d object
   (const SmObject * pObj) ;                        // in : Vertex, Edge, or Face to target

  // rtn TRUE when Obj uses a LocalZoneTol3d value, Get LocalZoneTol3d value
  static SmBoolean HasZoneTol3d
   (const SmObject * pObj,                          // in : Vertex, Edge, or Face to target
    SmZoneTol3d    & rZoneTol3d) ;                  // out: When TRUE is returned, The object's ZoneTol3dValue
#endif // NOT_BEING_USED - // not yet used - perhaps obsolete

  // ScaledZero  : Numerical Tolerances = the numerical tolerance limit for many iterative routines [order 1E-12]
  // ScaledZeroSq:  Scaled Zeros - ScaledZero values for values, points, extents, etc.

  // default - no argument ScaledZero
  static SmScaledZero GetScaledZero() ;              // rtn: SM_EFF_ZERO
  static SmScaledZero GetScaledZeroSq() ;            // rtn: SM_EFF_ZERO_SQ
  static SmScaledZero GetScaledZeroSqrt() ;          // rtn: SM_EFF_ZERO_SQRT

  // unary ScaledZero
  static SmScaledZero GetScaledZero( const double                   dVal ) ;
  static SmScaledZero GetScaledZero( const SmObject               & rObj) ;
  static SmScaledZero GetScaledZero( const SmCurve                & rCurve ) ; 
  static SmScaledZero GetScaledZero( const SmSurface              & rSurface ) ; 
  static SmScaledZero GetScaledZero( const SmVolume               & rVolume ) ; 
  static SmScaledZero GetScaledZero( const SmPseudoBox            & rBox ) ; 
  static SmScaledZero GetScaledZero( const SmCurveInterval        & rCurveInterval ) ; 
  static SmScaledZero GetScaledZero( const SmEvalNFunctionsObject & rEvalNFunctionsObject ) ;
  static SmScaledZero GetScaledZero( const SmExtent1d             & rIvl ) ;
  static SmScaledZero GetScaledZero( const SmExtent2d             & rIvl ) ;
  static SmScaledZero GetScaledZero( const SmExtent3d             & rIvl ) ;
  static SmScaledZero GetScaledZero( const SmVector2d             & rPnt ) ;
  static SmScaledZero GetScaledZero( const SmVector3d             & rPnt ) ;                     
  static SmScaledZero GetScaledZero( const SmBrep                 & rBrep ) ;
  static SmScaledZero GetScaledZero( const SmRegion               & rRegion ) ;
  static SmScaledZero GetScaledZero( const SmShell                & rShell ) ;
  static SmScaledZero GetScaledZero( const SmFace                 & rFace ) ; 
  static SmScaledZero GetScaledZero( const SmEdge                 & rEdge ) ; 
  static SmScaledZero GetScaledZero( const SmVertex               & rVertex ) ; 

  // unary ScaledZeroSq 
  template<class cA> 
  static SmScaledZero GetScaledZeroSq(const cA & rA) { SmScaledZero sRtn = GetScaledZero(rA) ; return(sRtn * sRtn) ; }

  // unary ScaledZeroSqrt
  template<class cA> 
  static SmScaledZero GetScaledZeroSqrt(const cA & rA) { SmScaledZero sRtn = GetScaledZero(rA) ; return(smos_Sqrt(sRtn)) ; }
 
  // Multi-argument ScaledZero(. . .) - arguments can be any type defined with a unary ScaledZero(type) methods

  // binary ScaledZero
  template<class cA, class cB> 
  static SmScaledZero GetScaledZero( const cA & rA,
                                     const cB & rB) { return(smos_Max(SmTol::GetScaledZero(rA), 
                                                                      SmTol::GetScaledZero(rB))) ; 
                                                    }
  // binary ScaledZeroSq
  template<class cA, class cB> 
  static SmScaledZero GetScaledZeroSq(const cA & rA,
                                      const cB & rB) { SmScaledZero sRtn = GetScaledZero(rA, rB) ; return(sRtn * sRtn) ; }

  // binary ScaledZeroSqrt
  template<class cA, class cB> 
  static SmScaledZero GetScaledZeroSqrt(const cA & rA,
                                        const cB & rB) { SmScaledZero sRtn = GetScaledZero(rA, rB) ; return(smos_Sqrt(sRtn)) ; }

  // tertiary ScaledZero
  template<class cA, class cB, class cC> 
  static SmScaledZero GetScaledZero( const cA & rA,
                                     const cB & rB,
                                     const cC & rC) { return(smos_3Max(SmTol::GetScaledZero(rA), 
                                                                       SmTol::GetScaledZero(rB), 
                                                                       SmTol::GetScaledZero(rC))) ; 
                                                    }
  // tertiary ScaledZeroSq
  template<class cA, class cB, class cC> 
  static SmScaledZero GetScaledZeroSq(const cA & rA,
                                      const cB & rB,
                                      const cC & rC) { SmScaledZero sRtn = GetScaledZero(rA, rB, rC) ; return(sRtn * sRtn) ; }
  
  // tertiary ScaledZeroSqrt
  template<class cA, class cB, class cC> 
  static SmScaledZero GetScaledZeroSqrt(const cA & rA,
                                        const cB & rB,
                                        const cC & rC) { SmScaledZero sRtn = GetScaledZero(rA, rB, rC) ; return(smos_Sqrt(sRtn)) ; }
  
  // quaternary ScaledZero
  template<class cA, class cB, class cC, class cD> 
  static SmScaledZero GetScaledZero( const cA & rA,
                                     const cB & rB,
                                     const cC & rC,
                                     const cD & rD) { return(smos_4Max(SmTol::GetScaledZero(rA), 
                                                                       SmTol::GetScaledZero(rB), 
                                                                       SmTol::GetScaledZero(rC), 
                                                                       SmTol::GetScaledZero(rD))) ; 
                                                    }
  // quaternary ScaledZeroSq
  template<class cA, class cB, class cC, class cD>
  static SmScaledZero GetScaledZeroSq(const cA & rA,
                                      const cB & rB,
                                      const cC & rC,
                                      const cD & rD) { SmScaledZero sRtn = GetScaledZero(rA, rB, rC, rD) ; return(sRtn * sRtn) ; }

  // quaternary ScaledZeroSqrt
  template<class cA, class cB, class cC, class cD>
  static SmScaledZero GetScaledZeroSqrt(const cA & rA,
                                        const cB & rB,
                                        const cC & rC,
                                        const cD & rD) { SmScaledZero sRtn = GetScaledZero(rA, rB, rC, rD) ; return(smos_Sqrt(sRtn)) ; }

   // temp template test method
#ifdef SM_DEBUG_CODE
   static SmBoolean sm_TemplateTrial() ;
#endif /// SM_DEBUG_CODE

} ; // end class SmTol

#endif // !__SMTOLERANCE_H__

