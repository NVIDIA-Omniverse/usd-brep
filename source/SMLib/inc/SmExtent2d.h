// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*******************************************************************//**
* FILE NAME --- SmExtent2d.h
* PURPOSE: Header file for SmExtent2d class.
**********************************************************************/

#ifndef __SMEXTENT2D_H__
#define __SMEXTENT2D_H__

#ifndef __SMVECTOR2D_H__
#include <SmVector2d.h>
#endif

#ifndef __SMCORE_TYPES_H__
#include <SmCoreTypes.h>
#endif

#ifndef __SMEXTENT1D_H__
#include <SmExtent1d.h>
#endif

class SmContext ;
class SmGfxArraySet ;
class SmSurface ;  // for Draw() method

//      #ifndef __SMEXTENT3D_H__
//      #include <SmExtent3d.h>
//      #endif

/*******************************************************************//**
PURPOSE: This enum specifies the classification values for a specified
  direction from a given Point2d relative to the Extent2d domain.

NOTES: 
1. Given a Point2d and a Vector2d classify the
UVPts in the local neighborhood of Point2d in the Vector2d
direction relative to a UVDomain 
2. Don't change element order - some methods depend on them
***********************************************************************/
enum SmExtentDirType{ 
  SM_ED_UNEXPECTED     =0, // default val for unexpected locs (never expected to be used unless there is a bug)
  SM_ED_UNINIT         =1, // Var declared but unassigned temp value.
  SM_ED_NONE           =2, // default val for unclassifiable [sUV dDir] locs (when UVDomain is degenerate)
  SM_ED_GOING_IN       =3, // SrfVec at a Pt2d points towards 'in' Pt2ds
                           //    all  SrfVecs on a SrfInPt - are 'in'
                           //    some SrfVecs on a SrfBndryPt - are 'in'
  SM_ED_GOING_OUT      =4, // SrfVec at a Pt2d points towards 'out' Pt2ds
                           //    all  SrfVecs on a SrfOutPt - are 'out'
                           //    some SrfVecs on a SrfBndryPt - are 'out'
  SM_ED_GOING_TAN_POSU =5, // SrfVec on a SrfBndryPt is tangent to SrfBndry in Pos U dir
  SM_ED_GOING_TAN_POSV =6, // SrfVec on a SrfBndryPt is tangent to SrfBndry in Pos V dir
  SM_ED_GOING_TAN_NEGU =7, // SrfVec on a SrfBndryPt is tangent to SrfBndry in Neg U dir
  SM_ED_GOING_TAN_NEGV =8  // SrfVec on a SrfBndryPt is tangent to SrfBndry in Neg V dir
} ;                        

/*******************************************************************//**
PURPOSE: This object represents a two dimensional domain.  It contains
   a minimum and maximum 2D point.  The points may be equal.

NOTES: This object is often used as the parametric domain for
   a surface.
***********************************************************************/
class SM_EXPORT SmExtent2d
{
protected:
  SmPoint2d m_vMin ;
  SmPoint2d m_vMax ;
public:
  /* empty constructor         */ SmExtent2d() { Init(); }                                                      
  /* 1 point constructor       */ SmExtent2d(const SmPoint2d & rMinAndMax) { m_vMin = rMinAndMax;               
                                                                             m_vMax = rMinAndMax;               
                                                                           }                                    
  /* 2 point constructor       */ SmExtent2d(const SmPoint2d & rMin,                                            
                                             const SmPoint2d & rMax);                                           
  /* 4 parameter constructor   */ SmExtent2d(double UMin, double VMin,                                          
                                             double UMax, double VMax)     { m_vMin.x = UMin ;                  
                                                                             m_vMin.y = VMin ;                  
                                                                             m_vMax.x = UMax ;                  
                                                                             m_vMax.y = VMax ;                  
                                                                           }                                    
  /* 2 extent1ds constructor   */ SmExtent2d(const SmExtent1d & rXRange,                                        
                                             const SmExtent1d & rYRange)   { m_vMin.x = rXRange.GetMin() ;      
                                                                             m_vMin.y = rYRange.GetMin() ;      
                                                                             m_vMax.x = rXRange.GetMax() ;      
                                                                             m_vMax.y = rYRange.GetMax() ;      
                                                                           }                                    
  /* 2d -> 2d copy constructor */ SmExtent2d(const SmExtent2d & crOriginal);                              
  /* 3d -> 2d copy constructor */ SmExtent2d(const SmExtent3d & crExtent3d);                              
  /* assignment operator       */ SmExtent2d& operator= (SmExtent2d const &crOther) { if(&crOther == this) return *this ;
                                                                                      m_vMin = crOther.m_vMin ;
                                                                                      m_vMax = crOther.m_vMax ;
                                                                                      return *this ;  
                                                                                    }
  /* equality operator         */ SmBoolean   operator==(const SmExtent2d &crOther) const;  // tol = GetScaledZero()              
  /* destructor                */ ~SmExtent2d() { m_vMin.x = m_vMin.y =  SM_UNDEF_DOUBLE;                  
                                                  m_vMax.x = m_vMax.y = -SM_UNDEF_DOUBLE; 
                                                }

  // modifiers
  void        Init() { m_vMin.x = m_vMin.y = SM_BIG_DOUBLE; m_vMax.x = m_vMax.y = -SM_BIG_DOUBLE; }
  void        AddPoint2d(const SmPoint2d & rPoint);
  SmStatus    Set(const SmExtent2d &crExt2d) ;
  SmStatus    SetMinMax(const SmPoint2d & rMin, const SmPoint2d & rMax);
  SmStatus    SetMinMax(double dMinX, double dMinY, double dMaxX, double dMaxY) ;
  SmStatus    SetUInterval( const SmExtent1d &crUIvl );
  SmStatus    SetVInterval( const SmExtent1d &crVIvl );
  SmStatus    SetUMin( double dNewVal );
  SmStatus    SetUMax( double dNewVal );
  SmStatus    SetVMin( double dNewVal );
  SmStatus    SetVMax( double dNewVal );
  void        SetUnbounded() { m_vMin.x = m_vMin.y = -SM_INFINITE_PARAMETER; m_vMax.x = m_vMax.y = SM_INFINITE_PARAMETER; }
  void        Transpose() ;  // swap U and V intervals 
                         
  SmExtent2d &ExpandAbsolute(      double     dExpansionDistance);
  SmExtent2d &ExpandAbsolute(const SmVector2d dExpansionVector);
  SmExtent2d &ExpandRelative(      double     dExpansionFactor);
  SmExtent2d &Scale         (      double     dScale, double *pOptScaleV=NULL) ;   // eff: Scale interval (scales center)

  // simple data access
  SmPoint2d  GetMin()          const { SM_ASSERT_DEFINED(this) ; return m_vMin; }
  SmPoint2d  GetMid()          const { SM_ASSERT_DEFINED(this) ; return (m_vMin+m_vMax)/2.0; }
  SmPoint2d  GetMax()          const { SM_ASSERT_DEFINED(this) ; return m_vMax; }

  double     GetUMin()         const { SM_ASSERT_DEFINED(this) ; return m_vMin.x ; }
  double     GetUMid()         const { SM_ASSERT_DEFINED(this) ; return (m_vMin.x+m_vMax.x)/2.0 ; }
  double     GetUMax()         const { SM_ASSERT_DEFINED(this) ; return m_vMax.x ; }

  double     GetVMin()         const { SM_ASSERT_DEFINED(this) ; return m_vMin.y ; }
  double     GetVMid()         const { SM_ASSERT_DEFINED(this) ; return (m_vMin.y+m_vMax.y)/2.0 ; }
  double     GetVMax()         const { SM_ASSERT_DEFINED(this) ; return m_vMax.y ; }

  SmPoint2d  GetSize()         const ;
  SmExtent1d GetUInterval()    const { SM_ASSERT_DEFINED(this) ; return SmExtent1d(m_vMin.x, m_vMax.x) ; }
  SmExtent1d GetVInterval()    const { SM_ASSERT_DEFINED(this) ; return SmExtent1d(m_vMin.y, m_vMax.y) ; }
  double     XLength()         const ;            // rtn: Max.x - Min.x
  double     YLength()         const ;            // rtn: Max.y - Min.y
  double     GetMinLength()    const { SM_ASSERT_DEFINED(this) ; return smos_Min(XLength(), YLength()) ; }
  double     GetMaxLength()    const { SM_ASSERT_DEFINED(this) ; return smos_Max(XLength(), YLength()) ; }
  double     GetMaxDimension() const { SM_ASSERT_DEFINED(this) ; return smos_Max(m_vMin.GetMaxDimension(),
                                                                                 m_vMax.GetMaxDimension() ) ; 
                                     }

  // Operations - Compute results from input extents
  void        Union                 (const SmExtent2d & crOther, 
                                           SmExtent2d & rResult) const ;
  SmStatus    Intersect             (const SmExtent2d & crOther,       
                                           SmExtent2d & rResult) const ;
  double      GetMaxBoundaryDist2d    (const SmExtent2d & crOther) const ; // rtn: Max(Xmin-OtherXmin,XMax-OtherMax,YMin-OtherYMin,YMax-OtherYMax)
  double      MaximumDistanceSquared(const SmExtent2d & crOther) const ;
  double      MinimumDistanceSquared(const SmExtent2d & crOther) const ;
  double      MaximumDistance       (const SmExtent2d & crOther) const { return smos_Sqrt(MaximumDistanceSquared(crOther)) ; }
  double      MinimumDistance       (const SmExtent2d & crOther) const { return smos_Sqrt(MinimumDistanceSquared(crOther)) ; }

  // predicates and classification
  SmBoolean   AreDisjoint       (const SmExtent2d & crOther, double dTol=0.0) const;
  SmBoolean   AreEqual          (const SmExtent2d & crOther, double dTol=0.0) const;
  SmBoolean   IsDegenerate      (double dTol=0.0)         const ; // rtn: TRUE = has at least one zero or neg length dimension, FALSE=doesn't
  SmBoolean   IsPoint           (double dTol=SM_EFF_ZERO) const ; // rtn: TRUE = has two zero length dimensions
  SmBoolean   IsLine            (double dTol=SM_EFF_ZERO) const ; // rtn: TRUE = has exactly one zero length dimension
  SmBoolean   HasNegativeArea   ()                        const ; // rtn: TRUE = has neg area (note:Init() sets a NegArea), FALSE=doesn't 
  SmBoolean   IsInit            ()                        const ; // rtn: TRUE = MinMax=[SM_BIG_DOUBLE -SM_BIG_DOUBLE]
  SmBoolean   IsBounded         (SmBoundaryType eOptIBType[2]=NULL) const ;   // rtn: TRUE = all boundaries are bounded (none equal to SM_INFINITE_PARAMETER)      
  SmBoolean   AnyBounds         (SmBoundaryType eOptIBType[2]=NULL) const ;   // rtn: TRUE = any boundary is bounded (any not equal to SM_INFINITE_PARAMETER)      
  SmBoolean   IsContainedBy     (const SmExtent2d & crOther, double dTol=0.0) const;  // rtn: TRUE = crOther extent is larger or equal to this extent
  SmBoolean   ContainsPoint2d   (const SmPoint2d  & rPoint,  double dTol=0.0, double *pOptMinOutsideDist = NULL) const;
  SmBoolean   ContainsPoint2dRelative(const SmPoint2d  & rPoint,  double dRelTol) const; // where dTol = dRelTol * ( m_dMax - m_dMin );
  void        ClassifyPoint2d   (const SmPoint2d   & rPoint,   
                                 SmExtentPointType & rExtentUType,
                                 SmExtentPointType & rExtentVType,
                                 double              dTolU=SM_EFF_ZERO,
                                 double            * pOptTolV=NULL)           const;
  void        ClassifyPointDir2d(SmPoint2d          sUV,                  // in : Tgt UVPoint
                                 SmVector2d         sDir,                 // in : Tgt UVDir at UVPoint
                                 SmExtentPointType &reExtentUType,        // out: oneof:[SM_EP_START, SM_EP_INSIDE, SM_EP_END, SM_EP_BOTH, SM_EP_OUTSIDE]
                                 SmExtentPointType &reExtentVType,        // out: oneof:[SM_EP_START, SM_EP_INSIDE, SM_EP_END, SM_EP_BOTH, SM_EP_OUTSIDE]
                                 SmExtentDirType   &reExtentDir,          // out: oneof:[SM_ED_GOING_IN,  SM_ED_TAN_POSU, SM_ED_TAN_NEGU,  
                                                                          //             SM_ED_GOING_OUT, SM_ED_TAN_POSV, SM_ED_TAN_NEGV]
                                 double             dTolU,                // in : max allowed param U distance to count as being on a U boundary
                                                                          //      default:[SM_EFF_ZERO]
                                 double           * pOptTolV) const ;     // in : max allowed param V distance to count as being on a V boundary
                                                                          //      NULL= dTolV = dTolU, default:NULL
  SmBoolean   IsTouchingOnePointAt2DCorner(SmExtent2d & rOther, 
                                           double dTol=0.0)                const;
  SmBoolean   IsPoint2dOnBoundary(const SmPoint2d & crPoint,    
                                  double            dTol=SM_EFF_ZERO,
                                  SmVector2d      * pOptBiNorm=NULL)       const;
  ULONG       GetPoint2dBoundaries(const SmPoint2d &crPoint,                      // rtn: SM_SS_NONE or orof:
                                   double           dTol=SM_EFF_ZERO)      const; //      [SM_SS_UMIN SM_SS_VMIN SM_SS_UMAX SM_SS_VMAX]
  SmExtent1d  GetInsideSectorAtPoint2d(SmPoint2d & rUV,                            // rtn: AngDeg Interval Inside Extent where 0=VecU, 90=VecV, 180=-VecU,... 
                                       SmTol2d     sTol2d=SM_EFF_ZERO_PARAM)const; //      max vals:[-90 360] max range 360
  // periodic intervals
  SmBoolean   ContainsPeriodicPoint2d(const SmPoint2d & rPoint,
                                      double dUPeriod,          // in : U Period length, 0=not periodic
                                      double dVPeriod,          // in : V Period length, 0=not periodic
                                      double d2dTolerance=0.0,
                                      SmPoint2d *pPeriodicPoint=NULL) const;
  SmBoolean   AreDisjointPeriodic(const SmExtent2d & crOther,   // when two intervals are just part of a periodic space
                                  double dUPeriod,              // in : U Period length, 0=not periodic
                                  double dVPeriod) const;       // in : V Period length, 0=not periodic
  SmStatus    IntersectPeriodic(const SmExtent2d & crOther,     
                                double dUPeriod,                // in : U Period length, 0=not periodic
                                double dVPeriod,                // in : V Period length, 0=not periodic
                                SmTArray<SmExtent2d> & rResult, 
                                SmBoolean bDontCrossBoundaries=FALSE) const;   
  // computations
  SmPoint2d   Evaluate    (double dNormalizedX, double dNormalizedY) const;
  double      EvaluateU   (double dNormalizedX) const;
  double      EvaluateV   (double dNormalizedY) const;
  SmStatus    Inversion   (const SmPoint2d & crPoint,
                           SmPoint2d       & rNormalizedPoint,
                           double            dTolerance = SM_EFF_ZERO) const;
  SmPoint2d   ClampPoint2d(const SmPoint2d  & rPoint) const;
  double      ClipLine2d  (const SmPoint2d  & rLinePoint,   // rtn: 1st line/extent intersection 
                           const SmVector2d & rLineVec, 
                           double dLineParam) const;
  SmStatus    IntersectWithInfiniteLine(const SmPoint2d  & rLinePoint, 
                                        const SmVector2d & rLineVec, 
                                        SmBoolean        & rbFoundInterval,
                                        SmExtent1d       & rTrimmedInterval,
                                        double           * dOptTolU_V=NULL,
                                        double           * dOptTolV  =NULL) const;
  SmBoolean   GetULeftEval(double dUValue) const;
  SmBoolean   GetVLeftEval(double dVValue) const;
  
  SmStatus    Point2dMapToDomain(const SmPoint2d & crPointInThisDomain,
                                 const SmExtent2d & crOther,
                                 SmBoolean bSwitchParameters,
                                 SmPoint2d & rPointInOtherDomain) const;
  // specialty functions
  SmExtent3d   ApproximateUnbounded                        // eff: return a bounded SmExtent3d to approximate an unbounded one                  
                 (SmPoint2d  * pUnboundedCenter=NULL,      // in : center of unbounded intervals, NULL = [0,0,0], default:[NULL]
                  double       dUnboundedHalfSize=         // in : the size used for infinite 1/2 spaces
                                SM_BOUNDED_INFINITE_PARAM, //      a totally unbounded plane will is approximated by a square twice this size
                  SmExtent2d * pOptExpandedApprox=NULL)    // out: a 2nd extent expanded a small bit - used by graphics, NULL to ignore, default:[NULL]           
                 const ;

  // utilities
  // note: do not use SM_COMMON_BASE() because no virtual methods are allowed for SmExtent2d
  void            Dump(void) const;
  SmDisplayList * Draw(const SmContext * pContext=NULL,               // NotUsed: in :
                       const SmSurface * pOptOutSurface=NULL,         // in : Draw on this Surface, NULL=draw on z=0 plane, default:[NULL]
                       SmGfxArraySet   * pOptGfxSet=NULL) const;      // i/o:
  SmBoolean       AssertValid(SmAssertArray    * pAList=NULL,           // i/o: Accumulating list of failed Asserts, NULL to ignore
                              SmAssertTestLevel  eTestLevel=SM_LEVEL_0, // in : SM_LEVEL_0=fewest tests, SM_LEVEL_1, SM_LEVEL_2=all tests, 
                                                                        //      SM_LEVEL_GIVEN = run tests in order requested in pTestRequests 
                                                                        //      default:[SM_LEVEL_0] 
                              SmAssertWalking    eWalkTree=SM_WALK,     // NotUsed: in : SM_WALK = run AssertValid on any topology graph descendants, SM_NO_WALK=don't, default:[SM_WALK] 
                              SmTArray<ULONG>  * pTestRequests=NULL)    // in : when eTestLevel == SM_LEVEL_GIVEN, run these tests in this order, default:[NULL]
                             const ;
  SmBoolean       AssertDefined() const;

  SM_TYPE         GetType()            const { return(SmExtent2d_TYPE) ; }
  const TCHAR   * GetTypeString()      const { return(_T("SmExtent2d_TYPE")) ; }
  const TCHAR   * GetClassString()     const { return(_T("SmExtent2d")) ; }
  SM_TYPE         GetClassType()       const { return(SmExtent2d_TYPE) ; }
  const TCHAR   * GetClassTypeString() const { return(_T("SmExtent2d_TYPE")) ; }
  
} ; // end class SmExtent2d

#endif // !__SMEXTENT2D_H__


