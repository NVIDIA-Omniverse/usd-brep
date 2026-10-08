// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*******************************************************************//**
* FILE NAME --- SmPseudoBox.h
* PURPOSE: Header file for SmPseudoBox class.
**********************************************************************/

#ifndef __SMPSEUDOBOX_H__
#define __SMPSEUDOBOX_H__

#ifndef __SMVECTOR3D_H__
#include <SmVector3d.h>
#endif

#ifndef __SMEXTENT1D_H__
#include <SmExtent1d.h>
#endif

#ifndef __SMCORE_TYPES_H__
#include <SmCoreTypes.h> 
#endif

#ifndef __SMCONTEXT_H__
#include <SmContext.h>
#endif

class SmPseudoBox;
class SmGfxArraySet ;

/*******************************************************************//**
PURPOSE: This object represents a non-axis aligned, non-orthogonal
    box in space.  It contains three independent unit vectors and 
    and an interval of interest along each vector.  The intersection
    of the intervals along the vectors represents a volume in space.

NOTES: It is used to create tight bounding volumes for curves
    and surfaces.
***********************************************************************/
class SM_EXPORT SmPseudoBox
{
protected:
    SmBoolean  m_bOrthogonalBasis = TRUE ;  // TRUE=x,y,z are orthogonal     
    SmVector3d m_aBasis[3] ;               // unit x,y,z vectors (not usually orthognal)     
    SmExtent1d m_aIntervals[3] ;           // x,y,z intervals defining the pseudoBox domain     

public:
    // constructors, destructor, initializer
    SmPseudoBox() ;                                                    // init basis to x,y,z
    SmPseudoBox(SmVector3d &rXDir, SmVector3d &rYDir, SmVector3d &rZDir,
                SmExtent1d &rXIvl, SmExtent1d &rYIvl, SmExtent1d &rZIvl)  { SetBasis(rXDir, rYDir, rZDir) ;
                                                                            SetIntervals(rXIvl, rYIvl, rZIvl) ;
                                                                          }
    SmPseudoBox(const SmPseudoBox & crOriginal) ;
    SmPseudoBox(const SmExtent3d  & crBBox) ;                // make a pseudoBox representation of a SmExtent3d BBox
    SmPseudoBox & operator= (const SmPseudoBox &crOther) ;
    SmPseudoBox & operator= (const SmExtent3d  &crOther) ;   // set  a pseudoBox representation of a SmExtent3d BBox
    SmBoolean     operator==(const SmPseudoBox &crOther) const ;
    void Init() ;                                            // init intervals, leave basis vectors alone
    void InitBasis() ;                                       // init basis vectors to [100 010 001]
   ~SmPseudoBox() { }

    // Project the pseudo box onto a plane 
    SmStatus ProjectToPlane(SmPseudoBox       & rProjectedBox,        // out: PseudoBox projected to plane
                            SmProjectionType    eProjectionType,      // in : oneof: SM_PT_PARALLEL, SM_PT_PERSPECTIVE, SM_PT_ROTATION
                            const SmPoint3d   * cpProjPoint,          // in : point on plane       or point on rotation axis
                            const SmVector3d  * cpProjUnitVector,     // in : unit-normal to plane or unit-vec of rotation axis
                            const SmVector3d  * cpAuxData) const ;    // in : not used, eye point  or unit-XAxis perp to rotation Axis
    // increase box domain 
    void ExpandAbsolute(double dExpansion);
    void ExpandSweep(const SmVector3d & sSweepVector) ;
    void AddPoint3d(const SmPoint3d & rPoint);   // used after SetBasis() ;
    SmStatus Union(const SmPseudoBox & crOther, 
                   const SmVector3d  * aBasis, 
                   SmPseudoBox       & crResult) const ;

    // simple data access
    SmStatus          SetBasis    (const SmVector3d & rBasis1,    const SmVector3d & rBasis2,    const SmVector3d & rBasis3);
    void              SetIntervals(const SmExtent1d & rInterval1, const SmExtent1d & rInterval2, const SmExtent1d & rInterval3);
    SmStatus          SetBasis    (ULONG lIndex, const SmVector3d & rBasis)   { if(rBasis.IsZero()) { SER(SM_ERR_INVALID_INPUT) ; }
                                                                                m_aBasis[lIndex] = rBasis ; 
                                                                                return(SM_SUCCESS) ;
                                                                              }
    void              SetInterval (ULONG lIndex, const SmExtent1d & rInterval) { m_aIntervals[lIndex] = rInterval ; }

    void              SetUnbounded() ;
    SmStatus          SetMinMax   (double dMinU, double dMinV, double dMinW,        // eff: Set interval min max points
                                   double dMaxU, double dMaxV, double dMaxW); 
    SmStatus          SetUMin     ( double dNewUMin ); // rtn: SM_ERR_INVALID_INPUT when assignment makes a negative interval
    SmStatus          SetUMax     ( double dNewUMax ); // rtn: SM_ERR_INVALID_INPUT when assignment makes a negative interval
    SmStatus          SetVMin     ( double dNewVMin ); // rtn: SM_ERR_INVALID_INPUT when assignment makes a negative interval
    SmStatus          SetVMax     ( double dNewVMax ); // rtn: SM_ERR_INVALID_INPUT when assignment makes a negative interval
    SmStatus          SetWMin     ( double dNewWMin ); // rtn: SM_ERR_INVALID_INPUT when assignment makes a negative interval
    SmStatus          SetWMax     ( double dNewWMax ); // rtn: SM_ERR_INVALID_INPUT when assignment makes a negative interval

    void              GetBasis    (SmVector3d & rBasis1, SmVector3d & rBasis2, SmVector3d & rBasis3) const;
    const SmVector3d *GetBasis    () const             { return &m_aBasis[0] ; }
    const SmVector3d &GetBasis    (ULONG lIndex) const { SM_ASSERT(lIndex <= 2) ;
                                                         return(m_aBasis[lIndex]) ;
                                                       }
    void              GetIntervals(SmExtent1d & rInterval1, SmExtent1d & rInterval2, SmExtent1d & rInterval3) const;
    const SmExtent1d &GetInterval (ULONG lIndex) const { SM_ASSERT(lIndex <= 2) ;
                                                         return(m_aIntervals[lIndex]) ;
                                                       }
    SmPoint3d         GetIntervalSizes() const;        // rtn: size of each interval length
    double            GetMaxDimension()  const ;
    double            GetXYArea()        const ;
    double            GetVolume()        const ;

    // computations
    SmPoint3d Evaluate(double dNormalizedX,             // rtn: linearly interpolated point
                       double dNormalizedY,             //      coord values from    0 to 1 are inside  extent
                       double dNormalizedZ) const;      //      coord values outside 0 to 1 are outside extent

    SmStatus  Invert(const SmPoint3d & cr3DPoint,                // eff: map 3d point to extent normalized coordinates
                     SmPoint3d & rNormalizedParameters) const;   //

    SmPoint3d EvaluateAbsolute(double dAbsoluteX,         // rtn: 3DPoint =   AbsoluteParameters[0] * m_aBasis[0]
                               double dAbsoluteY,         //                + AbsoluteParameters[1] * m_aBasis[1]
                               double dAbsoluteZ) const;  //                + AbsoluteParameters[2] * m_aBasis[2]

    SmStatus  InvertAbsolute(const SmPoint3d & cr3DPoint,             // eff: map 3d point to extent absolute coordinates
                             SmPoint3d & rAbsoluteParameters) const;  //

    SmStatus  ConvertAbsoluteToNormal(const SmPoint3d & rAbsoluteParameters,          // eff: map absolute to normalized parameters
                                      SmPoint3d       & rNormalizedParameters) const; //

    // generate pBox boundary points and boundary planes
    void       GetCorners  (SmTArray<SmPoint3d> &rCornerPoints)  const ;
    void       GetEdges    (SmTArray<SmPoint3d> &rEdgeEndPoints) const ; // every pair of points [iEven,iEven+1] marks one edge
    void       CalcCorners (SmPoint3d aCorners[8]) const;
    void       CalcPlanes  (SmPoint3d aPoints[6], SmVector3d aVectors[6]) const;
    void       GetPlanes   (SmTArray<SmPoint3d> &rPlanePoints, SmTArray<SmVector3d> &rPlaneNormals) const ; // sizes arrays and passes call to CalcPlanes()
    SmExtent1d GetUInterval(const SmPoint3d &crPoint3d) const ;  // Rtn inside UIvl for line = sPoint3d + u * [1 0 0], No Ivl = Ivl.Init()
    SmExtent1d GetVInterval(const SmPoint3d &crPoint3d) const ;  // Rtn inside VIvl for line = sPoint3d + v * [0 1 0], No Ivl = Ivl.Init()
    SmExtent1d GetWInterval(const SmPoint3d &crPoint3d) const ;  // Rtn inside WIvl for line = sPoint3d + w * [0 0 1], No Ivl = Ivl.Init()
                                                                                                                        
    // predicates
    SmBoolean IsInit() const ;  
    SmBoolean IsBounded      ( SmBoundaryType eOptIvlBdryType[3]=NULL) const ;     // rtn: TRUE = all boundaries are bounded (none equal to SM_INFINITE_PARAMETER)      
                                                                                   // opt: return all BndryType classifications
    SmBoolean IsAxisAligned                                  // rtn: TRUE = bases are orthogonal and parallel to XYZ (in any order)
                           (long alAxisMap[3]) const ;       // out: alAxisMap[0] = index+1 of basis parallel to X, neg = in negative direction
                                                             //      alAxisMap[1] = index+1 of basis parallel to Y, neg = in negative direction
                                                             //      alAxisMap[2] = index+1 of basis parallel to Z, neg = in negative direction
    SmBoolean AreAligned                                     // rtn: TRUE = bases are are parallel to one another (in any order)
                           (const SmPseudoBox & crOther,     // in : other arg
                            long                alAxisMap[3])// out: alAxisMap[0] = index+1 of basis parallel to X, neg = in negative direction
                            const ;                          //      alAxisMap[1] = index+1 of basis parallel to Y, neg = in negative direction
                                                             //      alAxisMap[2] = index+1 of basis parallel to Z, neg = in negative direction
    SmBoolean ContainsPoint3d(const SmPoint3d & crPoint, 
                              double d3dTolerance=0.0)      const;
    SmBoolean AreDisjoint    (const SmPseudoBox & crOther,                         // in : other arg
                              const SmVector3d  * cpOptProjUnitVector=NULL) const; // in : notNULL=check boxes projected to common plane
                                                                                   //      NULL   =check boxes in 3d
    SmBoolean AreProjectDisjoint(const SmPseudoBox & crOther,              // in : other pseudo box
                                 SmProjectionType    eProjectionType,      // in : oneof: SM_PT_PARALLEL, SM_PT_PERSPECTIVE, SM_PT_ROTATION
                                 const SmPoint3d   * cpProjPoint,          // in : point on plane       or point on rotation axis
                                 const SmVector3d  * cpProjUnitNormal,     // in : unit-normal to plane or unit-vec of rotation axis
                                 const SmVector3d  * cpAuxData) const ;    // in : not used, eye point, or unit-XAxis perp to rotation Axis

    SmBoolean IsContainedBy  (const SmPseudoBox & crOther,
                              double d3dTolerance=0.0)                const;

    void      ClassifyPoint3d(const SmPoint3d   & rPoint,  
                              SmExtentPointType &rExtentUType,
                              SmExtentPointType &rExtentVType,
                              SmExtentPointType &rExtentWType,
                              double dTol=SM_EFF_ZERO) const ;

    SmBoolean IsPointSized   (double      dTol3d,                           // in : max allowed deviation from CenterPoint
                              SmPoint3d  *pOptCenterPoint=NULL,             // out: center of PseudoBox
                              double     *pOptActualDeviation=NULL) const;  // out: Max Boundary/CenterPoint distance seen

    SmBoolean IsLinear       (double      dTol3d,                           // in : max allowed deviation from Plane
                              SmPoint3d  *pOptLinePoint=NULL,               // out: Point on Line
                              SmVector3d *pOptLineTangent=NULL,             // out: Unit Tangent Vector
                              double     *pOptActualDeviation=NULL) const;  // out: Max Boundary/Line distance

    SmBoolean IsPlanar       (double      dTol3d=SM_EFF_ZERO,               // in : max allowed deviation from Plane
                              SmPoint3d  *pOptPlanePoint=NULL,              // out: Point on Plane
                              SmVector3d *pOptPlaneNormal=NULL,             // out: Unit Surface Normal
                              double     *pOptActualDeviation=NULL) const;  // out: Max Boundary/Plane distance 

    SmBoolean HasNegativeVolume() const ;                 // rtn: TRUE = has neg volume as set by init, FALSE=doesn't 
    double    DistanceToPoint(const SmPoint3d & rPoint)                const;
    double    DistanceToPlane(const SmPoint3d & crPlanePoint,
                              const SmVector3d & crPlaneNormal)        const;
    SmStatus  IntersectLine  (const SmPoint3d  & crLinePoint,      // in : point on line
                              const SmVector3d & crLineVector,     // in : vector defining line's direction
                              ULONG            & rlNumFound,       // out: XSect Cnt: 0 = there is no intersection.   
                                                                   //                 1 = grazes a corner.
                                                                   //                 2 = portion of ray is inside box.
                              double           & rdTEnter,         // out: Start XSect Interval Param      
                              double           & rdTExit) const;   // out: End XSect Interval Param
                              
                                     

    // specialty functions
    SmPseudoBox ApproximateUnbounded                               // eff: return a bounded SmPseudoBox to approximate an unbounded one                  
                  (SmPoint3d  *pUnboundedCenter=NULL,              // in : opt center of unbounded intervals, NULL = [0,0,0], default:[NULL]
                   double      dUnboundedHalfSize=                 // in : the size used for infinite 1/2 spaces a totally
                                 SM_BOUNDED_INFINITE_PARAM,        //      a totally unbounded volume will is approximated by a box twice this size
                   SmPseudoBox *pOptExpandedApprox=NULL)           // out: a 2nd Extent expanded a small bit - used by graphics, NULL to ignore, default:[NULL]           
                  const ;

    // maintenance
    void Dump(void) const;
    SmDisplayList * Draw(const SmContext * pContext =NULL,          // NotUsed: in : 
                         SmGfxArraySet   * pOptGfxSet=NULL) const;  // i/o:

    SmBoolean AssertValid(SmAssertArray    * pAList=NULL,           // i/o: Accumulating list of failed Asserts, NULL to ignore
                          SmAssertTestLevel  eTestLevel=SM_LEVEL_0, // in : SM_LEVEL_0=fewest tests, SM_LEVEL_1, SM_LEVEL_2=all tests, 
                                                                    //      SM_LEVEL_GIVEN = run tests in order requested in pTestRequests 
                                                                    //      default:[SM_LEVEL_0] 
                          SmAssertWalking    eWalkTree=SM_WALK,     // NotUsed: in : SM_WALK = run AssertValid on any topology graph descendants, SM_NO_WALK=don't, default:[SM_WALK] 
                          SmTArray<ULONG>  * pTestRequests=NULL)    // in : when eTestLevel == SM_LEVEL_GIVEN, run these tests in this order, default:[NULL]
                         const ;
    SmBoolean AssertDefined() const ;


    SM_TYPE       GetType()            const { return(SmPseudoBox_TYPE) ; }
    const TCHAR  *GetTypeString()      const { return(_T("SmPseudoBox_TYPE")) ; }
    const TCHAR  *GetClassString()     const { return(_T("SmPseudoBox")) ; }
    SM_TYPE       GetClassType()       const { return(SmPseudoBox_TYPE) ; }
    const TCHAR  *GetClassTypeString() const { return(_T("SmPseudoBox_TYPE")) ; }
} ; // end class SmPseudoBox

#endif // !__SMPSEUDOBOX_H__

