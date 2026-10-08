// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*******************************************************************//**
* FILE NAME --- SmExtent3d.h
* PURPOSE: Header file for SmExtent3d class.
**********************************************************************/

#ifndef __SMEXTENT3D_H__
#define __SMEXTENT3D_H__

#ifndef __SMVECTOR3D_H__
#include <SmVector3d.h>
#endif

#ifndef __SMEXTENT1D_H__
#include <SmExtent1d.h>
#endif

#ifndef __SMEXTENT2D_H__
#include <SmExtent2d.h>
#endif

class SmGfxArraySet ;

/*******************************************************************//**
PURPOSE: This object represents a three dimensional domain in 
   Euclidian space.  It contains a minimum and maximum 3D points.
   The points may be the same.  

NOTES: This object is often used as a bounding box.
***********************************************************************/
class SM_EXPORT SmExtent3d
{
protected:
  SmPoint3d m_vMin ;
  SmPoint3d m_vMax ;
public:
  // constructors, destructor
  SmExtent3d() { Init() ; }
  SmExtent3d(const SmPoint3d & rMin, 
             const SmPoint3d & rMax);
  SmExtent3d(const SmPoint3d & rMinAndMax) { m_vMin = rMinAndMax; 
                                             m_vMax = rMinAndMax; 
                                           }
  SmExtent3d(double UMin, double VMin, double WMin,
             double UMax, double VMax, double WMax) { m_vMin.x = UMin ;
                                                      m_vMin.y = VMin ;
                                                      m_vMin.z = WMin ; 
                                                      m_vMax.x = UMax ;
                                                      m_vMax.y = VMax ;
                                                      m_vMax.z = WMax ;
                                                    }
  SmExtent3d(const SmExtent3d & crOriginal) ;
  SmExtent3d(const SmPseudoBox & crPseudoBox)       { Circumscribe(crPseudoBox) ; } 
  SmExtent3d& operator=(SmExtent3d const &obj)      { if(&obj == this) return *this ;
                                                      m_vMin = obj.m_vMin ;
                                                      m_vMax = obj.m_vMax ;
                                                      return *this ;  
                                                    }

  SmBoolean    operator==(const SmExtent3d&) const;
 ~SmExtent3d() { m_vMin.x = m_vMin.y = m_vMin.z =  SM_UNDEF_DOUBLE; 
                 m_vMax.x = m_vMax.y = m_vMax.z = -SM_UNDEF_DOUBLE; }

  // modifiers 
  void         Init();                                   // eff: set to negative volume
  void         AddPoint3d(const SmPoint3d & rPoint);     // eff: enlarge to contain point
  SmStatus     SetMinMax (const SmPoint3d & rMin,        // eff: Set extent min max points
                          const SmPoint3d & rMax); 
  SmStatus     SetMinMax (double dMinX, double dMinY, double dMinZ,        // eff: Set extent min max points
                          double dMaxX, double dMaxY, double dMaxZ); 
  SmStatus     SetUInterval( const SmExtent1d &crUIvl );
  SmStatus     SetVInterval( const SmExtent1d &crVIvl );
  SmStatus     SetWInterval( const SmExtent1d &crWIvl );
  SmStatus     SetUMin( double dNewVal ); // rtn: SM_ERR_INVALID_INPUT if assignment would make a negative interval
  SmStatus     SetUMax( double dNewVal ); // rtn: SM_ERR_INVALID_INPUT if assignment would make a negative interval
  SmStatus     SetVMin( double dNewVal ); // rtn: SM_ERR_INVALID_INPUT if assignment would make a negative interval
  SmStatus     SetVMax( double dNewVal ); // rtn: SM_ERR_INVALID_INPUT if assignment would make a negative interval
  SmStatus     SetWMin( double dNewVal ); // rtn: SM_ERR_INVALID_INPUT if assignment would make a negative interval
  SmStatus     SetWMax( double dNewVal ); // rtn: SM_ERR_INVALID_INPUT if assignment would make a negative interval
  void         SetUnbounded() { m_vMin.x = m_vMin.y = m_vMin.z = -SM_INFINITE_PARAMETER; m_vMax.x = m_vMax.y = m_vMax.z = SM_INFINITE_PARAMETER; }
  SmExtent3d & ExpandAbsolute(double dExpansion);        // eff: expand extent boundaries by given amount in all directions - preserve center
  SmExtent3d & ExpandRelative(double dExpansionFactor);  // eff: multiply extent size by given factor - preserve center
  SmExtent3d & Scale         (double dScale, double *pOptScaleV=NULL, double *pOptScaleW=NULL) ;  // eff: Scale interval (scales center)
  void         Circumscribe (SmPseudoBox const &crPseudoBox) ;

  // simple data access
  SmPoint3d    GetMin()  const { SM_ASSERT_DEFINED(this) ; return m_vMin; }                        // rtn: Min Point
  SmPoint3d    GetMax()  const { SM_ASSERT_DEFINED(this) ; return m_vMax; }                        // rtn: Max Point
  SmPoint3d    GetMid()  const { SM_ASSERT_DEFINED(this) ; return ((m_vMax + m_vMin)/2.0); }       // rtn: Mid Point (center)
  double       GetUMin() const { return m_vMin.x ; }
  double       GetUMid() const { return (m_vMin.x + m_vMax.x)/2.0 ; } 
  double       GetUMax() const { return m_vMax.x ; }
  double       GetVMin() const { return m_vMin.y ; }
  double       GetVMid() const { return (m_vMin.y + m_vMax.y)/2.0 ; } 
  double       GetVMax() const { return m_vMax.y ; }
  double       GetWMin() const { return m_vMin.z ; }
  double       GetWMid() const { return (m_vMin.z + m_vMax.z)/2.0 ; } 
  double       GetWMax() const { return m_vMax.z ; }
  double       GetMin(ULONG ii) const { return m_vMin[ii] ; }
  double       GetMid(ULONG ii) const { return (m_vMin[ii] + m_vMax[ii]) / 2.0 ; }
  double       GetMax(ULONG ii) const { return m_vMax[ii] ; }

  SmVector3d   GetSize() const;                                          // rtn: vector from minPoint to maxPoint
  SmExtent1d   GetUInterval() const { SM_ASSERT_DEFINED(this) ; return SmExtent1d(m_vMin.x, m_vMax.x) ; }
  SmExtent1d   GetVInterval() const { SM_ASSERT_DEFINED(this) ; return SmExtent1d(m_vMin.y, m_vMax.y) ; }
  SmExtent1d   GetWInterval() const { SM_ASSERT_DEFINED(this) ; return SmExtent1d(m_vMin.z, m_vMax.z) ; }
  SmExtent2d   GetUVDomain()  const { SM_ASSERT_DEFINED(this) ; return SmExtent2d(m_vMin.x, m_vMin.y, m_vMax.x, m_vMax.y) ; }
  SmExtent2d   GetVWDomain()  const { SM_ASSERT_DEFINED(this) ; return SmExtent2d(m_vMin.y, m_vMin.z, m_vMax.y, m_vMax.z) ; }
  SmExtent2d   GetUWDomain()  const { SM_ASSERT_DEFINED(this) ; return SmExtent2d(m_vMin.x, m_vMin.z, m_vMax.x, m_vMax.z) ; }
  void         GetCorners(SmTArray<SmPoint3d> &rCornerPoints) const ;  // out: corners ordered:{ 000 010 100 110 001 011 101 111 }
  void         GetEdges  (SmTArray<SmPoint3d> &rEdgeEndPoints) const ; // every pair of points [iEven,iEven+1] marks one edge
  void         GetPlanes (SmTArray<SmPoint3d> &rPlanePoints, SmTArray<SmVector3d> &rPlaneNormals) const ;  

  double       XLength() const;                                      // rtn: Max.x - Min.x
  double       YLength() const;                                      // rtn: Max.y - Min.y
  double       ZLength() const;                                      // rtn: Max.z - Min.z
  double       GetMaxLength() const    { return (smos_3Max(XLength(), YLength(), ZLength())) ; }
  double       GetMaxDimension() const { SM_ASSERT_DEFINED(this) ; 
                                         return smos_Max(m_vMin.GetMaxDimension(),
                                                         m_vMax.GetMaxDimension()); 
                                       }
  // Operations - Compute results from input extents
  void         Union     (const SmExtent3d & crOther,                // eff: build union of two extents
                                SmExtent3d & rResult) const;         //
  SmStatus     Intersect (const SmExtent3d & crOther,                // eff: build intersection of two extents
                                SmExtent3d & rResult) const; 
  SmStatus     SubdivideLargestDirection(SmExtent3d & rMinExtent,    // eff: build 2 extents from this extent by 
                                         SmExtent3d & rMaxExtent,    //      subdividing this extent's largest dimension
                                         ULONG & rlDirection) const; //
  double       MaximumDistance       (const SmExtent3d & crOther) const; // rtn: max possible dist between two points in union of two extents.              
  double       MinimumDistance       (const SmExtent3d & crOther) const; // rtn: min possible dist between two points in two extents                      
  double       MaximumDistanceSquared(const SmExtent3d & crOther) const; // rtn: max possible dist squared between two points in union of two extents.
  double       MinimumDistanceSquared(const SmExtent3d & crOther) const; // rtn: min possible dist squard between two points in two extents          
  double       DistanceToPlane( const SmPoint3d &crPlanePt, const SmVector3d &crPlaneUnitNorm ) const;

  // predicates and classifications
  SmBoolean    ContainsPoint3d        (const SmPoint3d & crPoint, double dTol=0.0) const ; // 
  SmBoolean    ContainsLineSeg3d      (const SmPoint3d & crStartPoint, const SmPoint3d & crEndPoint, double dTol=0.0) const ; //     
  SmBoolean    ContainsPoint3dRelative(const SmPoint3d & crPoint, double dRelTol) const;      // AbsTol = RelativeTol * (Max - Min)
  SmBoolean    IsPoint3dOnBoundary    (const SmPoint3d & crPoint, double dTol=SM_EFF_ZERO,    
                                                SmVector3d * pOptBiNorm=NULL) const ; // out: in pointing vVec, ex:[1 1 1] = pt on Min corner
                               
  void         ClassifyPoint3d  (const  SmPoint3d  & rPoint, SmExtentPointType &rExtentUType,
                                                             SmExtentPointType &rExtentVType,
                                                             SmExtentPointType &rExtentWType,
                                                             double dTol=SM_EFF_ZERO) const ;
  SmBoolean    IntersectsLine3d (const  SmPoint3d  & crLinePoint,         // rtn: TRUE = line intersects bounding box within tol
                                 const  SmVector3d & crLineVector,
                                 double dTol=0.0) const ;

  SmBoolean    IsPointSized     (double dTol=SM_EFF_ZERO) const ;
  SmBoolean    HasNegativeVolume()                const ;                // rtn: TRUE = has neg volume as set by init, FALSE=doesn't 
  SmBoolean    IsPositiveVolume()                 const ;                // rtn: TRUE = has Positive volume, FALSE=doesn't 
  SmBoolean    IsPositiveArea()                   const ;                // rtn: TRUE = has Positive area on Z=0 plane, FALSE=doesn't
  SmBoolean    IsInit()                           const ;                // rtn: TRUE = MinMax=[SM_BIG_DOUBLE -SM_BIG_DOUBLE]
  SmBoolean    IsBounded(SmBoundaryType eOptIBType[3]=NULL) const ;      // rtn: TRUE = all boundaries are bounded (none equal to SM_INFINITE_PARAMETER)      
  SmBoolean    AnyBounds(SmBoundaryType eOptIvlBdryType[3]=NULL) const ; // rtn: TRUE = any boundary is bounded (any not equal to SM_INFINITE_PARAMETER)      
                                                                         // opt: return all BndryType classifications
  SmBoolean    AreDisjoint      (const  SmExtent3d & crOther, double dTol=0.0) const ; //                                        
  SmBoolean    IsContainedBy    (const  SmExtent3d & crOther, double dTol=0.0) const ; //

  // computations
  SmPoint3d    Evaluate(double dNormalizedX,                             // rtn: linearly interpolated point
                        double dNormalizedY,                             //      coord values from    0 to 1 are inside  extent
                        double dNormalizedZ) const;                      //      coord values outside 0 to 1 are outside extent
  SmStatus     Invert(const SmPoint3d & cr3DPoint,                       // eff: map 3d point to extent normalized coordinates
                      SmPoint3d & rNormalizedParameters) const;          //
  SmPoint3d    ClampPoint3d(const SmPoint3d & rPoint) const;
  SmPoint3d    SnapPoint3dToBoundary(const SmPoint3d & rPoint, double dTol=SM_EFF_ZERO) const ;
  void         ComputeSphereBound(SmVector3d & rSphereCenter,            // eff: get center and radius of sphere enclosing extent
                                  double & rdSphereRadius,
                                  double * pOptMinSphereRadius=NULL) const;   
  SmStatus     ComputeConeBoundTo(const SmExtent3d & crOther,            // eff: get conical bound between 2 extents, the vector field 
                                  SmVector3d & rConeVector,              //      of all possible vectors between points in this extent 
                                  double & rdConeAngleInRadians) const;  //      going to points in crOther extent.                                     
  SmStatus     IntersectLine(const SmPoint3d  & crLinePoint,             // eff: intersect line with extent
                             const SmVector3d & crLineVector,            
                             ULONG            & rlNumFound,              
                             double           & rdTEnter,                
                             double           & rdTExit) const;

  // specialty functions
  SmExtent3d   ApproximateUnbounded                        // eff: return a bounded SmExtent3d to approximate an unbounded one                  
                 (SmPoint3d  *pUnboundedCenter=NULL,       // in : center of unbounded intervals, NULL = [0,0,0], default:[NULL]
                  double      dUnboundedHalfSize=          // in : the size used for infinite 1/2 spaces 
                                SM_BOUNDED_INFINITE_PARAM, //      a totally unbounded cube is approximated by a cube twice this size
                  SmExtent3d *pOptExpandedApprox=NULL)     // out: a 2nd extent expanded a small bit - used by graphics, NULL to ignore, default:[NULL]           
                 const ;

  // utilities
  // note: do not use SM_COMMON_BASE() because no virtual methods are allowed for SmExtent3d
  void            Dump(void) const;                                      // eff: pretty print extent contents
  SmDisplayList * Draw                                                   // eff: draw extent's rectilinear solid outline
                      (const SmContext * pContext=NULL,                  // NotUsed: in :
                       SmPoint3d       * pUnboundedCenter=NULL,          // in : center of interest for unbounded extents
                       SmGfxArraySet   * pOptGfxSet=NULL) const;         // i/o:   
  SmBoolean       AssertValid(SmAssertArray    * pAList=NULL,           // i/o: Accumulating list of failed Asserts, NULL to ignore
                              SmAssertTestLevel  eTestLevel=SM_LEVEL_0, // in : SM_LEVEL_0=fewest tests, SM_LEVEL_1, SM_LEVEL_2=all tests, 
                                                                        //      SM_LEVEL_GIVEN = run tests in order requested in pTestRequests 
                                                                        //      default:[SM_LEVEL_0] 
                              SmAssertWalking    eWalkTree=SM_WALK,     // NotUsed: in : SM_WALK = run AssertValid on any topology graph descendants, SM_NO_WALK=don't, default:[SM_WALK] 
                              SmTArray<ULONG>  * pTestRequests=NULL)    // in : when eTestLevel == SM_LEVEL_GIVEN, run these tests in this order, default:[NULL]
                             const ;
  SmBoolean       AssertDefined() const;

  SM_TYPE         GetType()            const { return(SmExtent3d_TYPE) ; }
  const TCHAR   * GetTypeString()      const { return(_T("SmExtent3d_TYPE")) ; }
  const TCHAR   * GetClassString()     const { return(_T("SmExtent3d")) ; }
  SM_TYPE         GetClassType()       const { return(SmExtent3d_TYPE) ; }
  const TCHAR   * GetClassTypeString() const { return(_T("SmExtent3d_TYPE")) ; }

} ; // end class SmExtent3d

#endif // !__SMEXTENT3D_H__


