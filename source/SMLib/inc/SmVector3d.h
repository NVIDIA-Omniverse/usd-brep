// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*******************************************************************//**
* FILE NAME --- SmVector3d.h
* PURPOSE: Header file for SmVector3d
**********************************************************************/

#ifndef __SMVECTOR3D_H__
#define __SMVECTOR3D_H__

#ifndef __SMCORE_TYPES_H__
#include <SmCoreTypes.h>
#endif

#ifndef __SMVECTOR2D_H__
#include <SmVector2d.h>
#endif

#ifdef Length
#undef Length
#endif /* Length */


class SmVector3d ;
class SmGfxArraySet ;

inline SmVector3d operator - (const SmVector3d& v);                        // -v1
inline SmVector3d operator + (const SmVector3d& a, const SmVector3d& b);   // v1 + v2
inline SmVector3d operator - (const SmVector3d& a, const SmVector3d& b);   // v1 - v2
inline SmVector3d operator * (const SmVector3d& a, const double d);        // v1 * 3.0
inline SmVector3d operator * (const double d, const SmVector3d& a);        // 3.0 * v1
inline SmVector3d operator * (const SmVector3d& a, const SmVector3d& b);   // cross product
inline SmVector3d operator / (const SmVector3d& a, const double d);        // v1 / 3.0 
inline int        operator== (const SmVector3d & crVec1, const SmVector3d & crVec2);
inline int        operator!= (const SmVector3d & crVec1, const SmVector3d & crVec2);

/*******************************************************************//**
PURPOSE: This object is a three dimensional vector.  It is represented
   by an X, Y and Z value.  

NOTES: SmPoint3d is another name for this same class.

   NO VIRTUAL METHODS: The members of this class are accessed in arrays by offsets.  
   A virtual function introduces a virtual function pointer to every object
   thus destroying the offset trick, so No Virtual Methods in this class.

   Also other classes manage arrays of SmVector3d objects with calls to 
   smos_MemSet() which would corrupt the SmVector3d virtual function table pointer
   if it had one.
***********************************************************************/
class SM_EXPORT SmVector3d
{  
protected:
public: 
  double x = SM_UNDEF_DOUBLE ;
  double y = SM_UNDEF_DOUBLE ;
  double z = SM_UNDEF_DOUBLE ;
  SmVector3d() = default;
  ~SmVector3d() = default;

 SmVector3d( double dX, double dY, double dZ )
 {
   SM_ASSERT_BREAK( dX != SM_UNDEF_DOUBLE && dY != SM_UNDEF_DOUBLE && dZ != SM_UNDEF_DOUBLE && dZ != NL_NOZ );
   x = dX; y = dY; z = dZ;
 }

 SmVector3d( double *pVec )
 {
   SM_ASSERT_BREAK( pVec[0] != SM_UNDEF_DOUBLE && pVec[1] != SM_UNDEF_DOUBLE && pVec[2] != SM_UNDEF_DOUBLE && pVec[2] != NL_NOZ );
   if(pVec) { x = pVec[0]; y = pVec[1]; z = pVec[2]; }
   else { SetUninitialized(); }
 }

 explicit SmVector3d(double* pVec, bool fromNL)
 {
   SM_REF1(fromNL) ;
   x = pVec[0]; y = pVec[1]; z = (pVec[2] != NL_NOZ ? pVec[2] : 0.);
   //rolled back additional divide by zero check due to performance penalty
   //double dDenomChk = smos_Fabs( pVec[3] );
   if (    NL_NOW != pVec[3]
        //&& dDenomChk > SM_EFF_ZERO * smos_Fabs( x )
        //&& dDenomChk > SM_EFF_ZERO * smos_Fabs( y )
        //&& dDenomChk > SM_EFF_ZERO * smos_Fabs( z )
      )
        { x /= pVec[3]; y /= pVec[3]; z /= pVec[3]; }
 }
 SmVector3d( const SmPoint3d &rStartPt, const SmPoint3d &rEndPt )
 {
   SM_ASSERT_BREAK( rStartPt.x != SM_UNDEF_DOUBLE && rStartPt.y != SM_UNDEF_DOUBLE && rStartPt.z != SM_UNDEF_DOUBLE && rStartPt.z != NL_NOZ );
   SM_ASSERT_BREAK( rEndPt.x != SM_UNDEF_DOUBLE && rEndPt.y != SM_UNDEF_DOUBLE && rEndPt.z != SM_UNDEF_DOUBLE && rEndPt.z != NL_NOZ );
   x = rEndPt.x - rStartPt.x; y = rEndPt.y - rStartPt.y; z = rEndPt.z - rStartPt.z;
 }

 SmVector3d( const SmVector3d & crVector )
 {
   SM_ASSERT_BREAK( crVector.x != SM_UNDEF_DOUBLE && crVector.y != SM_UNDEF_DOUBLE && crVector.z != SM_UNDEF_DOUBLE && crVector.z != NL_NOZ );
   x = crVector.x;   y = crVector.y;   z = crVector.z;
 }

 SmVector3d( const SmVector2d & cr2DVector )
 {
#ifdef SM_DEBUG_CODE
   if(cr2DVector.x == SM_UNDEF_DOUBLE || cr2DVector.y == SM_UNDEF_DOUBLE)
   {
     SM_ASSERT_BREAK( cr2DVector.x != SM_UNDEF_DOUBLE && cr2DVector.y != SM_UNDEF_DOUBLE );
   }
#endif
   x = cr2DVector.x; y = cr2DVector.y; z = 0.0;
 }

 // assign only iniitialized values
 SmVector3d& operator=( SmVector3d const &obj )
 {
   if(&obj == this) return *this;
   SM_ASSERT_BREAK( obj.x != SM_UNDEF_DOUBLE && obj.y != SM_UNDEF_DOUBLE && obj.z != SM_UNDEF_DOUBLE && obj.z != NL_NOZ );
   x = obj.x;
   y = obj.y;
   z = obj.z;
   return *this;
 }

 // copy - copy initialized and uninitialized values
 SmVector3d& Copy( SmVector3d const &obj )
 {
   if(&obj == this) return *this;
   x = obj.x;
   y = obj.y;
   z = obj.z;
   return *this;
 }

  void       SetUninitialized ()       { x = y = z = SM_UNDEF_DOUBLE ; }
  SmBoolean  IsInitialized    () const { return    x != SM_UNDEF_DOUBLE
                                                && y != SM_UNDEF_DOUBLE
                                                && z != SM_UNDEF_DOUBLE; }
  SmBoolean  IsUndef          () const { return(   x == SM_UNDEF_DOUBLE || !SM_IS_VALID_DOUBLE(x)
                                                || y == SM_UNDEF_DOUBLE || !SM_IS_VALID_DOUBLE(y)
                                                || z == SM_UNDEF_DOUBLE || !SM_IS_VALID_DOUBLE(z)) ; } 
  SmBoolean  IsColinearWith   (const SmVector3d & crEndVec,
                               const SmPoint3d  & crStartPnt,
                               const SmPoint3d  & crEndPnt) const;
  SmBoolean  IsParallelTo     (const SmVector3d & crOther, double dAngTolDeg=SM_EFF_ZERO_DEG, SmBoolean *pOptAntiParallel=NULL) const;
  SmBoolean  IsPerpendicularTo(const SmVector3d & crOther, double dAngTolDeg=SM_EFF_ZERO_DEG) const;
  SmBoolean  IsZero           (double dDistTol3d=SM_EFF_ZERO) const { return( LengthSquared() < dDistTol3d * dDistTol3d) ; }
  SmBoolean  Is2d             () const                              { return( z == NL_NOZ ) ; }   // sometimes Z=0.0 for 2d which will be missed.      
  
  // is this vector inside given co-planar sector boundaries
  SmBoolean  IsInsideSector   
  (
    const SmVector3d & crTangentVector1,      ///< [in ]: first vector defining sector
    const SmVector3d & crBinormalVector1,     ///< [in ]: vector pointing to inside of sector on first vector boundary
    const SmVector3d & crTangentVector2       ///< [in ]: 2nd vector defining sector
  ) const;

  int IsInTriangle
  (
    const SmVector3d & rP0,
    const SmVector3d & rP1,
    const SmVector3d & rP2
  ) const;

  double     LengthSquared( void ) const
  {
    SM_ASSERT_BREAK( x != SM_UNDEF_DOUBLE && y != SM_UNDEF_DOUBLE && z != SM_UNDEF_DOUBLE && z != NL_NOZ );
    return (x*x) + (y*y) + (z*z);
  }

  double     Length( void ) const
  {
    SM_ASSERT_BREAK( x != SM_UNDEF_DOUBLE && y != SM_UNDEF_DOUBLE && z != SM_UNDEF_DOUBLE && z != NL_NOZ );
    return smos_Sqrt( (x*x) + (y*y) + (z*z) );
  }

  double     GetMaxDimension         () const;
  double     GetMinDimension         () const;
  SmStatus   Unitize                 (SmBoolean bSignalBadInput=TRUE);    

  double     Dot( const SmVector3d & crOther ) const
  {
    SM_ASSERT_BREAK( x != SM_UNDEF_DOUBLE && y != SM_UNDEF_DOUBLE && z != SM_UNDEF_DOUBLE && z != NL_NOZ );
    SM_ASSERT_BREAK( crOther.x != SM_UNDEF_DOUBLE
                    && crOther.y != SM_UNDEF_DOUBLE
                    && crOther.z != SM_UNDEF_DOUBLE );
    return (x*crOther.x) + (y*crOther.y) + (z*crOther.z);
  }

  // rtn: = this dot (crB * crC)
  double     TripleProduct           (const SmVector3d & crB, const SmVector3d & crC) const ;

  double     DistanceBetween         (const SmVector3d & crOther) const;
  double     DistanceBetweenSquared  (const SmVector3d & crOther) const;
  SmBoolean  CloserThan              (double dMax, const SmVector3d & crOther) const;

  // rtn: Ang3dRad from 0 to Pi
  SmStatus   AngleBetween            (const SmVector3d & crOther, double & rdAngRad) const;

  // rtn: AngProjRad about this vec from -Pi to Pi
  SmStatus   CCWAngleBetween         (const SmVector3d & crStartVector,                     
                                      const SmVector3d & crEndVector,
                                      double           & rdStartToEndAngRad) const;

  // rResult=[x/crOther.x, y/crOther.y, z/crOther.z]
  SmStatus   Divide                  (const SmVector3d & crOther, SmVector3d & rResult) const; 

  // rtn: [x*crOther.x, y*crOther.y, z*crOther.z]
  SmVector3d Multiply                (const SmVector3d & crOther) const;                       

  SmStatus   MakeUnitOrthoVectors
  ( 
    const SmVector3d * pYReference,    ///< [in ]: optional X/Y Plane specification, NULL to ignore
    SmVector3d & rXAxis,               ///< [out]: unit vec in 'this' vector direction
    SmVector3d & rYAxis,               ///< [out]: orthogonal to rXAxis and if given, rotated to pYReference direction
    SmVector3d & rZAxis                ///< [out]: cross(X,Y)
  ) const;

  // rtn: vec projected to plane along plane normal
  SmVector3d ProjectToPlane           (const SmVector3d & crPlaneUnitNormal) const;

  // rtn: pt  projected to plane along plane normal
  SmPoint3d  ProjectPointToPlane      (const SmPoint3d  & crPlanePoint, const SmVector3d & crPlaneUnitNormal) const;

  // rtn: vec projected to plane about rotation axis
  SmVector3d RotateProjectToPlane     (const SmVector3d & crRotateUnitAxis, const SmVector3d & crXUnitAxis) const;      

  // rtn: pt  projected to plane about rotation axis
  SmPoint3d  RotateProjectPointToPlane
  (
    const SmPoint3d  & crRotatePoint,       ///< [in ]: pt on rotation axis     
    const SmVector3d & crRotateUnitAxis,    ///< [in ]: Unit-vector rotation axis     
    const SmVector3d & crXUnitAxis          ///< [in ]: Unit-vector X Axis
  ) const;

  // rtn: vec projected to plane towards eye view pt
  SmVector3d PerspectiveProjectToPlane     
  (
    const SmPoint3d  & crVecBasePt,          ///< [in ]: 3d vector base pt (vec projections change based on location)
    const SmPoint3d  & crPlanePoint,         ///< [in ]: point on viewing plane
    const SmVector3d & crPlaneUnitNormal,    ///< [in ]: unit-normal to viewing plane (the viewing direction)           
    const SmPoint3d  & crEyePoint            ///< [in ]: eye location   
  ) const;  

  // rtn: pt  projected to plane towards eye view pt
  SmPoint3d  PerspectiveProjectPointToPlane
  (
    const SmPoint3d  & crPlanePoint,        ///< [in ]: point on viewing plane
    const SmVector3d & crPlaneUnitNormal,   ///< [in ]: unit-normal to viewing plane (the viewing direction)       
    const SmPoint3d  & crEyePoint           ///< [in ]: eye location   
  ) const;

  // rtn: vec rotated about axis by given angle deg
  SmVector3d RotateVecAboutAxis (const SmVector3d & crAxisVec, double       dAngRad) const;   

  // rtn: pt  rotated about axis by given angle deg
  SmPoint3d  RotatePtAboutLine        
  (
    const SmPoint3d  & crLinePt,          ///< [in ]: Point on rotation line  
    const SmVector3d & crLineVec,         ///< [in ]: Vector defining rotation line direction   
    double             dAngRad            ///< [in ]: rotation angle in radians
  ) const ;

  // rtn: this * crVecInPlane * this 
  SmVector3d NormalInPlane           (const SmVector3d & crVecInPlane) const ; 

  // eff: rResult = Unitized(this*cr2ndVec) or x=y=z=SM_UNDEF_DOUBLE when degenerate
  SmStatus   NormalToPlane           
  (
    const SmVector3d & cr2ndVec,             
    SmVector3d       & rResult
  ) const ; 

  // rtn: rResult = Unitized((Pt2-Pt1)*(Pt3-Pt2)) or x=y=z=SM_UNDEF_DOUBLE when degenerate
  SmStatus   NormalToPlane           
  (
    const SmPoint3d & cr2ndPt,               
    const SmPoint3d & cr3rdPt,               
    SmVector3d      & rResult
  ) const ;        

  SmVector3d UnitizedDerivative      (const SmVector3d & crVec_t ) const;

  SmStatus   UnitizedDerivative2     
  (
    const SmVector3d & crVec_t,      ///< [in ]:  V'  = 1st deriv of this vector    
    const SmVector3d & crVec_tt,     ///< [in ]:  V'' = 2nd deriv of this vector
    SmVector3d & rUnitVec_t,         ///< [out]:  (V/|V|)' = 1st deriv of the unit vector, (V/|V|)
    SmVector3d & rUnitVec_tt         ///< [out]:  (V/|V|)''= 2nd deriv of the unit vector, (V/|V|)
  ) const;

  SmStatus   UnitizedDerivative3     
  (
    const SmVector3d & crVec_t,      ///< [in ]:  V'   = 1st deriv of this vector                  
    const SmVector3d & crVec_tt,     ///< [in ]:  V''  = 2nd deriv of this vector                  
    const SmVector3d & crVec_ttt,    ///< [in ]:  V''' = 3rd deriv of this vector 
    SmVector3d & rUnitVec_t,         ///< [out]:  (V/|V|)'  = 1st deriv of the unit vector, (V/|V|)
    SmVector3d & rUnitVec_tt,        ///< [out]:  (V/|V|)'' = 2nd deriv of the unit vector, (V/|V|)
    SmVector3d & rUnitVec_ttt        ///< [out]:  (V/|V|)'''= 2nd deriv of the unit vector, (V/|V|) 
  ) const;

  SmVector3d DerivativeOfCrossProduct
  (
    const SmVector3d & crOtherVector,    ///< [in ]: OtherVector
    const SmVector3d & crThisPrime,      ///< [in ]: Derivative of this vector
    const SmVector3d & crOtherPrime      ///< [in ]: Derivative of other vector
  ) const;

  void       Set                     (double dX, double dY, double dZ);

  double     SumFabs                 () { return(smos_Fabs(x)+smos_Fabs(y)+smos_Fabs(z));}

  SmStatus   SolveTwoLinearEquations (const SmVector3d & crOtherEquation, SmPoint2d & rSolutionValues) const;

  void       Make2d()                { z = NL_NOZ ; }
  void       Make3d()                { z = 0.0 ; }

  SmVector2d Get2dXY(){ SmVector2d sReturn(x,y); return sReturn; }

  SmStatus   BaryCentric             (const SmTArray<SmVector3d> &crFaceVects, SmTArray<double> & rWeights) const;
                                   
  void       Swap                    (SmVector3d & crOtherVector);
                                                                             // dot    product, use a.Dot(b)
  friend SmVector3d operator - (const SmVector3d& v);                        // vector negate,   -v1       
  friend SmVector3d operator + (const SmVector3d& a, const SmVector3d& b);   // vector sum,       v1  + v2   
  friend SmVector3d operator - (const SmVector3d& a, const SmVector3d& b);   // vector sum,       v1  - v2   
  friend SmVector3d operator * (const SmVector3d& a, const double d);        // scalar multiply,  v1  * 3.0 
  friend SmVector3d operator * (const double d, const SmVector3d& a);        // scalar multiply,  3.0 * v1  
  friend SmVector3d operator * (const SmVector3d& a, const SmVector3d& b);   // cross  product,             
  friend SmVector3d operator / (const SmVector3d& a, const double d);        // scalar multiply,  v1  / 3.0                                                                                

  friend int operator==(const SmVector3d & crVec1, const SmVector3d & crVec2);
  friend int operator!=(const SmVector3d & crVec1, const SmVector3d & crVec2) { return( !(crVec1 == crVec2) ) ; }
  double     operator[] (ULONG lIndex) const;
  double &   operator[] (ULONG lIndex);

  SmVector3d & operator+=(const SmVector3d & rVec2);
  SmVector3d & operator-=(const SmVector3d & rVec2);
  SmVector3d & operator*=(double d);
  SmVector3d & operator/=(double d);

  // This needs to be here for HwStdMap
  SmBoolean operator < (SmVector3d const &) const
          { return FALSE; }

  // utilities
  // note: do not use SM_COMMON_BASE() because no virtual methods are allowed for SmVector3d
  void     Dump(void)              const;
  void     Dump(const TCHAR * message)   const;
  void     Dump(SmBoolean bAbbrev) const;
  void     Dump(ULONG)             const; 
  SmStatus Write(FILE *pFile)  const;

  // pVectorOrigin == NULL, draw Point
  // pVectorOrigin != NULL, draw Vector starting at pVectorOrigin
  SmDisplayList * Draw
  (
    const SmVector3d * pVectorOrigin=NULL, // in :
    const SmContext  * pContext=NULL,      // NotUsed: in :
    SmGfxArraySet    * pOptGfxSet=NULL     // in :
  ) const;

  // draw 2 point icons and a connecting line
  SmDisplayList * DrawPointToPoint
  (
    const SmVector3d & rOtherPoint, 
    const SmContext  * pContext=NULL,
    SmGfxArraySet    * pOptGfxSet=NULL
  ) const;

  // draw plane icon perp to this normal vector
  SmDisplayList * DrawPlane
  (
    const SmVector3d & rPlanePoint,
    const SmContext  * pContext=NULL,
    SmGfxArraySet    * pOptGfxSet=NULL
  ) const;

  const TCHAR * GetTypeString() const { return _T("SmVector3d") ; }
  SM_TYPE       GetClassType()        { return SmVector3d_TYPE; }

} ; // end class SmVector3d

// Friend functions

/*******************************************************************//**
PURPOSE: Negate a vector.

NOTES: vResult = - v;
***********************************************************************/
inline SmVector3d operator - 
  (const SmVector3d& v)         
{ SM_ASSERT_BREAK(   v.x!=SM_UNDEF_DOUBLE  
                  && v.y!=SM_UNDEF_DOUBLE 
                  && v.z!=SM_UNDEF_DOUBLE
                  && v.z!=NL_NOZ);

  return SmVector3d(-v.x, -v.y, -v.z); 

} // end operator - 

/*******************************************************************//**
PURPOSE: Add two vectors.

NOTES: vResult = v1 + v2;
***********************************************************************/
inline SmVector3d operator +
( const SmVector3d& a, const SmVector3d& b )
{
  SM_ASSERT_BREAK( a.x != SM_UNDEF_DOUBLE && b.x != SM_UNDEF_DOUBLE
                 && a.y != SM_UNDEF_DOUBLE && b.y != SM_UNDEF_DOUBLE
                 && a.z != SM_UNDEF_DOUBLE && b.z != SM_UNDEF_DOUBLE
                 && a.z != NL_NOZ          && b.z != NL_NOZ );

  return SmVector3d( a.x + b.x, a.y + b.y, a.z + b.z );

} // end operator + 
    
/*******************************************************************//**
PURPOSE: Subtract one vector from another.

NOTES: vResult = v1 - v2;
***********************************************************************/
inline SmVector3d operator -
( const SmVector3d& a, const SmVector3d& b )
{
  SM_ASSERT_BREAK( a.x != SM_UNDEF_DOUBLE && b.x != SM_UNDEF_DOUBLE
                 && a.y != SM_UNDEF_DOUBLE && b.y != SM_UNDEF_DOUBLE
                 && a.z != SM_UNDEF_DOUBLE && b.z != SM_UNDEF_DOUBLE
                 && a.z != NL_NOZ          && b.z != NL_NOZ );

  return SmVector3d( a.x - b.x, a.y - b.y, a.z - b.z );

} // end operator -

/*******************************************************************//**
PURPOSE: Multiply a vector by a constant.

NOTES: vResult = v * 2.0;
***********************************************************************/
inline SmVector3d operator *
( const SmVector3d& a, const double d )
{
#ifdef SM_DEBUG_CODE
  if(a.x == SM_UNDEF_DOUBLE
     || a.y == SM_UNDEF_DOUBLE
     || a.z == SM_UNDEF_DOUBLE
     || a.z == NL_NOZ)
  {
    SM_ASSERT_BREAK( a.x != SM_UNDEF_DOUBLE
                   && a.y != SM_UNDEF_DOUBLE
                   && a.z != SM_UNDEF_DOUBLE
                   && a.z != NL_NOZ );
  }
#endif // SM_DEBUG_CODE

  return SmVector3d( a.x * d, a.y * d, a.z * d );

} // end operator * 

/*******************************************************************//**
PURPOSE: Multiply a constant by a vector.

NOTES: vResult = 3.0 * v;
***********************************************************************/
inline SmVector3d operator *
( const double d, const SmVector3d& a )
{
#ifdef SM_DEBUG_CODE
  if(a.x == SM_UNDEF_DOUBLE
     || a.y == SM_UNDEF_DOUBLE
     || a.z == SM_UNDEF_DOUBLE
     || a.z == NL_NOZ)
  {
    SM_ASSERT_BREAK( a.x != SM_UNDEF_DOUBLE
                    && a.y != SM_UNDEF_DOUBLE
                    && a.z != SM_UNDEF_DOUBLE
                    && a.z != NL_NOZ );
  }
#endif // SM_DEBUG_CODE

  return SmVector3d( a.x * d, a.y * d, a.z * d );

} // end operator * 

/*******************************************************************//**
PURPOSE: Compute the cross product of two 3D vectors.

NOTES: vResult = 3.0 * v;
***********************************************************************/
inline SmVector3d operator *
( const SmVector3d& a, const SmVector3d& b )
{
  SM_ASSERT_BREAK( a.x != SM_UNDEF_DOUBLE && b.x != SM_UNDEF_DOUBLE
                 && a.y != SM_UNDEF_DOUBLE && b.y != SM_UNDEF_DOUBLE
                 && a.z != SM_UNDEF_DOUBLE && b.z != SM_UNDEF_DOUBLE
                 && a.z != NL_NOZ          && b.z != NL_NOZ );

  return SmVector3d( (a.y * b.z) - (a.z * b.y),
    (a.z * b.x) - (a.x * b.z),
    (a.x * b.y) - (a.y * b.x) );

} // end operator * 

/*******************************************************************//**
PURPOSE: Divide a vector by a constant.

NOTES: vResult = v / 2.0;
***********************************************************************/
inline SmVector3d operator /
( const SmVector3d& a, const double d )
{
  SM_ASSERT_BREAK( d != 0.0 );
  SM_ASSERT_BREAK( a.x != SM_UNDEF_DOUBLE
                  && a.y != SM_UNDEF_DOUBLE
                  && a.z != SM_UNDEF_DOUBLE
                  && a.z != NL_NOZ );

  //removed additional divide by zero check due to performance penalty.
  //double dDenomChk = smos_Fabs( d );
  //if (    dDenomChk <= SM_EFF_ZERO * smos_Fabs( a.x )
  //     || dDenomChk <= SM_EFF_ZERO * smos_Fabs( a.y )
  //     || dDenomChk <= SM_EFF_ZERO * smos_Fabs( a.z )
  //   )
  //{ return a; }

  return SmVector3d( a.x / d, a.y / d, a.z / d );

} // end operator /


// Non trivial inline methods

/*******************************************************************//**
PURPOSE: Unitize a vector in place.  

NOTES: Return an error if the vector is a zero vector. SER(v.Unitize());
***********************************************************************/
inline SmStatus SmVector3d::Unitize( SmBoolean bSignalBadInput ) ///< [in ]: TRUE = signal bad input when Vec is ZeroLength
                                                               //      FALSE= don't
{
  SM_ASSERT_BREAK( x != SM_UNDEF_DOUBLE
                  && y != SM_UNDEF_DOUBLE
                  && z != SM_UNDEF_DOUBLE
                  && z != NL_NOZ );

  double dLenSq = LengthSquared();
  SmStatus sRet = SM_SUCCESS;

  if(dLenSq < SM_EFF_ZERO_SQ)
  {
    if(bSignalBadInput)
    {
      SE( SM_ERR_INVALID_INPUT );
    }
    x = 0.0;
    y = 0.0;
    z = 1.0;  // default return for noise (0,0,1)
    sRet = SM_ERR_INVALID_INPUT;
  }
  else if(smos_Fabs( dLenSq - 1.0 ) > SM_EFF_ZERO_SQ)
  {
    double dLen = smos_Sqrt( dLenSq );
    x = x / dLen;
    y = y / dLen;
    z = z / dLen;
  }
  return sRet;

} // end SmVector3d::Unitize

/*******************************************************************//**
PURPOSE: Compute the triple product = this dot (crB * crC).

NOTES: 
***********************************************************************/
inline double SmVector3d::TripleProduct( const SmVector3d & crB, const SmVector3d & crC ) const
{
  SM_ASSERT_BREAK( x != SM_UNDEF_DOUBLE     && crB.x != SM_UNDEF_DOUBLE
                 && y != SM_UNDEF_DOUBLE     && crB.y != SM_UNDEF_DOUBLE
                 && z != SM_UNDEF_DOUBLE     && crB.z != SM_UNDEF_DOUBLE
                 && z != NL_NOZ              && crB.z != NL_NOZ
                 && crC.x != SM_UNDEF_DOUBLE
                 && crC.y != SM_UNDEF_DOUBLE
                 && crC.z != SM_UNDEF_DOUBLE
                 && crC.z != NL_NOZ );

  return (x * ((crB.y * crC.z) - (crB.z * crC.y))
          + y * ((crB.z * crC.x) - (crB.x * crC.z))
          + z * ((crB.x * crC.y) - (crB.y * crC.x)));

} // end SmVector3d::TripleProduct

/*******************************************************************//**
PURPOSE: Compute a vector normal to this vector in the plane
            defined by the this vector and the input 
            vector, (const SmVector3d & crVecInPlane) const
NOTES: 
            NormalInPlane = this * crVecInPlane * this
***********************************************************************/
inline SmVector3d SmVector3d::NormalInPlane( const SmVector3d & crVecInPlane ) const
{
  SM_ASSERT_BREAK( x != SM_UNDEF_DOUBLE && crVecInPlane.x != SM_UNDEF_DOUBLE
                 && y != SM_UNDEF_DOUBLE && crVecInPlane.y != SM_UNDEF_DOUBLE
                 && z != SM_UNDEF_DOUBLE && crVecInPlane.z != SM_UNDEF_DOUBLE
                 && z != NL_NOZ          && crVecInPlane.z != NL_NOZ );

  return SmVector3d( y * ((crVecInPlane.x * y) - (crVecInPlane.y * x)) - z * ((crVecInPlane.z * x) - (crVecInPlane.x * z)),
                    z * ((crVecInPlane.y * z) - (crVecInPlane.z * y)) - x * ((crVecInPlane.x * y) - (crVecInPlane.y * x)),
                    x * ((crVecInPlane.z * x) - (crVecInPlane.x * z)) - y * ((crVecInPlane.y * z) - (crVecInPlane.z * y)) );

} // end SmVector3d::NormalInPlane

/*******************************************************************//**
PURPOSE: Compute unit vector normal to both input vectors

NOTES:   When Length(*this * cr2ndVec) < SM_EFF_ZERO,
          set rResult[x = y = z = SM_UNDEF_DOUBLE] ;
          return SM_ERR
         Else return SM_SUCCESS
***********************************************************************/
inline SmStatus SmVector3d::NormalToPlane( const SmVector3d & cr2ndVec, SmVector3d       & rResult ) const
{
  SM_ASSERT_BREAK( x != SM_UNDEF_DOUBLE && cr2ndVec.x != SM_UNDEF_DOUBLE
                 && y != SM_UNDEF_DOUBLE && cr2ndVec.y != SM_UNDEF_DOUBLE
                 && z != SM_UNDEF_DOUBLE && cr2ndVec.z != SM_UNDEF_DOUBLE
                 && z != NL_NOZ          && cr2ndVec.z != NL_NOZ );

  SmVector3d sNorm( (y * cr2ndVec.z) - (z * cr2ndVec.y),
    (z * cr2ndVec.x) - (x * cr2ndVec.z),
    (x * cr2ndVec.y) - (y * cr2ndVec.x) );

  if(SM_SUCCESS != sNorm.Unitize())
  {
    rResult.SetUninitialized();
    return SM_ERR;
  }

  rResult = sNorm;
  return SM_SUCCESS;

} // end SmVector3d::NormalToPlane

/*******************************************************************//**
PURPOSE: Compute unit vector normal to 3 points

NOTES:   When Length((Pt2-Pt1)*(Pt3-Pt2)) < SM_EFF_ZERO,
          set rResult[x = y = z = SM_UNDEF_DOUBLE] ;
          return SM_ERR
         Else return SM_SUCCESS
***********************************************************************/
inline SmStatus SmVector3d::NormalToPlane( const SmPoint3d & cr2ndPt, const SmPoint3d & cr3rdPt, SmVector3d       & rResult ) const
{
  SM_ASSERT_BREAK( x != SM_UNDEF_DOUBLE && cr2ndPt.x != SM_UNDEF_DOUBLE && cr3rdPt.x != SM_UNDEF_DOUBLE
                 && y != SM_UNDEF_DOUBLE && cr2ndPt.y != SM_UNDEF_DOUBLE && cr3rdPt.y != SM_UNDEF_DOUBLE
                 && z != SM_UNDEF_DOUBLE && cr2ndPt.z != SM_UNDEF_DOUBLE && cr3rdPt.z != SM_UNDEF_DOUBLE
                 && z != NL_NOZ          && cr2ndPt.z != NL_NOZ          && cr3rdPt.z != NL_NOZ );

  SmVector3d sNorm( ((cr2ndPt.y - y) * (cr3rdPt.z - cr2ndPt.z)) - ((cr2ndPt.z - z) * (cr3rdPt.y - cr2ndPt.y)),
    ((cr2ndPt.z - z) * (cr3rdPt.x - cr2ndPt.x)) - ((cr2ndPt.x - x) * (cr3rdPt.z - cr2ndPt.z)),
    ((cr2ndPt.x - x) * (cr3rdPt.y - cr2ndPt.y)) - ((cr2ndPt.y - y) * (cr3rdPt.x - cr2ndPt.x)) );

  if(SM_SUCCESS != sNorm.Unitize())
  {
    rResult.SetUninitialized();
    return SM_ERR;
  }

  rResult = sNorm;
  return SM_SUCCESS;

} // end SmVector3d::NormalToPlane

/*******************************************************************//**
PURPOSE: Get the absolute value of the maximum coordinate dimension.

NOTES: 
***********************************************************************/
inline double SmVector3d::GetMaxDimension() const
{
  SM_ASSERT_BREAK( x != SM_UNDEF_DOUBLE
                  && y != SM_UNDEF_DOUBLE
                  && z != SM_UNDEF_DOUBLE
                  && z != NL_NOZ );

  double dRet = smos_Fabs( x ) > smos_Fabs( y )
    ? smos_Fabs( x )
    : smos_Fabs( y );
  if(smos_Fabs( z ) > dRet) dRet = smos_Fabs( z );
  return dRet;

} // end SmVector3d::GetMaxDimension

/*******************************************************************//**
PURPOSE: Get the absolute value of the minimum coordinate dimension.

NOTES: 
***********************************************************************/
inline double SmVector3d::GetMinDimension() const
{
  SM_ASSERT_BREAK( x != SM_UNDEF_DOUBLE
                  && y != SM_UNDEF_DOUBLE
                  && z != SM_UNDEF_DOUBLE
                  && z != NL_NOZ );

  double dRet = smos_Fabs( x ) < smos_Fabs( y )
    ? smos_Fabs( x )
    : smos_Fabs( y );
  if(smos_Fabs( z ) < dRet) dRet = smos_Fabs( z );
  return dRet;

} // end SmVector3d::GetMaxDimension

/*******************************************************************//**
PURPOSE: Determine if two vectors are parallel (or anti-parallel)
    to each other within a given angle.

NOTES: zero length vectors are not parallel to other vectors.
***********************************************************************/
inline SmBoolean SmVector3d::IsParallelTo
  (const SmVector3d & crOther,          // in : target vector 
   double             dAngTolDeg,       // in : max angle between parallel vectors, default:[SM_EFF_ZERO_DEG]
   SmBoolean        * pOptAntiParallel) // out: when rtn is TRUE: TRUE = Vecs are anti-parallel(180 deg), else parallel(0), 
 const                                  //      NULL to ignore, default:[NULL]
{
  SM_ASSERT_BREAK(   crOther.x!=SM_UNDEF_DOUBLE && x!=SM_UNDEF_DOUBLE  
                  && crOther.y!=SM_UNDEF_DOUBLE && y!=SM_UNDEF_DOUBLE 
                  && crOther.z!=SM_UNDEF_DOUBLE && z!=SM_UNDEF_DOUBLE
                  && crOther.z!=NL_NOZ          && z!=NL_NOZ);

  SmBoolean bRet = FALSE;
  if(   LengthSquared()         > SM_EFF_ZERO_SQ
     && crOther.LengthSquared() > SM_EFF_ZERO_SQ) 
    {
      double dAngRad;
      SE(AngleBetween(crOther,dAngRad));  // dAngRad value from 0 to Pi
      if (dAngRad > SM_PI/2.0) { dAngRad = SM_PI - dAngRad ;
                                 if(pOptAntiParallel) { *pOptAntiParallel = TRUE ; }
                               }
      else                     { if(pOptAntiParallel) { *pOptAntiParallel = FALSE ; }
                               }
      if (dAngRad < SM_DEG2RAD(dAngTolDeg)) 
        {
          bRet = TRUE;
        }
    }
  return bRet;

} // end SmVector3d::IsParallelTo

/*******************************************************************//**
PURPOSE: Determine if two vectors are perpendicular to each other
         to with a given angle

NOTES: zero length vectors are not perpendicular to other vectors
***********************************************************************/
inline SmBoolean SmVector3d::IsPerpendicularTo
 (const SmVector3d & crOther,     ///< [in ]: target vector 
  double             dAngTolDeg)  ///< [in ]: max angle from 90 degrees for Perp vectors, default:[SM_EFF_ZERO_DEG]
 const
{
  SM_ASSERT_BREAK(   crOther.x!=SM_UNDEF_DOUBLE && x!=SM_UNDEF_DOUBLE 
                  && crOther.y!=SM_UNDEF_DOUBLE && y!=SM_UNDEF_DOUBLE
                  && crOther.z!=SM_UNDEF_DOUBLE && z!=SM_UNDEF_DOUBLE
                  && crOther.z!=NL_NOZ          && z!=NL_NOZ);

  SmBoolean bRet = FALSE;
  if (   LengthSquared()         > SM_EFF_ZERO_SQ 
      && crOther.LengthSquared() > SM_EFF_ZERO_SQ) 
    {
      double dAngRad;
      SE(AngleBetween(crOther, dAngRad));
      if (smos_Fabs(dAngRad-SM_PI/2.0) < SM_DEG2RAD(dAngTolDeg)) 
        {
          bRet = TRUE;
        }
    }

  return bRet;

} // end SmVector3d::IsPerpendicularTo

/*******************************************************************//**
PURPOSE: Project this vector to a plane given by a normal.

NOTES: 
   - Plane normal must be a unit vector.
***********************************************************************/
inline SmVector3d SmVector3d::ProjectToPlane( const SmVector3d & crPlaneUnitNormal ) const
{
  SM_ASSERT_BREAK( crPlaneUnitNormal.x   != SM_UNDEF_DOUBLE && x != SM_UNDEF_DOUBLE
                  && crPlaneUnitNormal.y != SM_UNDEF_DOUBLE && y != SM_UNDEF_DOUBLE
                  && crPlaneUnitNormal.z != SM_UNDEF_DOUBLE && z != SM_UNDEF_DOUBLE
                  && crPlaneUnitNormal.z != NL_NOZ          && z != NL_NOZ );

  SM_ASSERT_BREAK( SM_ARE_SAME( crPlaneUnitNormal.Length(), 1.0 ) );

  return (*this) - crPlaneUnitNormal * ((*this).Dot( crPlaneUnitNormal ));

} // end SmVector3d::ProjectToPlane

/*******************************************************************//**
PURPOSE: Project this point to a plane given by point and normal.

NOTES: 
   - Plane normal must be a unit vector.
***********************************************************************/
inline SmPoint3d SmVector3d::ProjectPointToPlane(const SmPoint3d  & crPlanePnt, const SmVector3d & crPlaneUnitNormal) const
{
  SM_ASSERT_BREAK(   crPlanePnt.x!=SM_UNDEF_DOUBLE && crPlaneUnitNormal.x!=SM_UNDEF_DOUBLE 
                  && crPlanePnt.y!=SM_UNDEF_DOUBLE && crPlaneUnitNormal.y!=SM_UNDEF_DOUBLE
                  && crPlanePnt.z!=SM_UNDEF_DOUBLE && crPlaneUnitNormal.z!=SM_UNDEF_DOUBLE
                  && crPlanePnt.z!=NL_NOZ          && crPlaneUnitNormal.z!=NL_NOZ
                  && x!=SM_UNDEF_DOUBLE  
                  && y!=SM_UNDEF_DOUBLE 
                  && z!=SM_UNDEF_DOUBLE
                  && z!=NL_NOZ);

  SM_ASSERT_BREAK(SM_ARE_SAME(crPlaneUnitNormal.Length(),1.0));

  SmVector3d sDiffVec( (*this) - crPlanePnt );
  return (*this) - crPlaneUnitNormal * ( sDiffVec.Dot( crPlaneUnitNormal ) );

} // end SmVector3d::ProjectPointToPlane

/*******************************************************************//**
PURPOSE: Project this vector to the [crXUnitAxis, crRotUnitAxis] 
         plane through a rotation about the crRotUnitAxis.

NOTES: 
   - RotateUnitAxis and XUnitAxis must be unit vectors.
***********************************************************************/
inline SmVector3d SmVector3d::RotateProjectToPlane
  (const SmVector3d & crRotateUnitAxis,     ///< [in ]: Unit-vector rotation axis
   const SmVector3d & crXUnitAxis)          ///< [in ]: Unit-vector X Axis
  const
{
  SM_ASSERT_BREAK(   crRotateUnitAxis.x!=SM_UNDEF_DOUBLE && crXUnitAxis.x!=SM_UNDEF_DOUBLE 
                  && crRotateUnitAxis.y!=SM_UNDEF_DOUBLE && crXUnitAxis.y!=SM_UNDEF_DOUBLE
                  && crRotateUnitAxis.z!=SM_UNDEF_DOUBLE && crXUnitAxis.z!=SM_UNDEF_DOUBLE
                  && crRotateUnitAxis.z!=NL_NOZ          && crXUnitAxis.z!=NL_NOZ
                  && x!=SM_UNDEF_DOUBLE  
                  && y!=SM_UNDEF_DOUBLE 
                  && z!=SM_UNDEF_DOUBLE
                  && z!=NL_NOZ);

  SM_ASSERT_BREAK(SM_ARE_SAME(crRotateUnitAxis.Length(),1.0));
  SM_ASSERT_BREAK(SM_ARE_SAME(crXUnitAxis.Length(),1.0));

  return (  crRotateUnitAxis * ( (*this).Dot(crRotateUnitAxis) )
          + crXUnitAxis      * ( (*this).Dot(crXUnitAxis) ) ) ;

} // end SmVector3d::RotateProjectToPlane

/*******************************************************************//**
PURPOSE: Project this Point to the [crRotPoint, crXUnitAxis, crRotUnitAxis] 
         plane through a rotation about the crRotUnitAxis.

NOTES: 
   - RotateUnitAxis and XUnitAxis must be unit vectors.
***********************************************************************/
inline SmPoint3d SmVector3d::RotateProjectPointToPlane
  (const SmPoint3d  & crRotatePoint,     ///< [in ]: pt on rotation axis
   const SmVector3d & crRotateUnitAxis,  ///< [in ]: Unit-vector rotation axis
   const SmVector3d & crXUnitAxis)       ///< [in ]: Unit-vector X Axis
  const
{
  SM_ASSERT_BREAK(   crRotatePoint.x!=SM_UNDEF_DOUBLE    && crXUnitAxis.x!=SM_UNDEF_DOUBLE
                  && crRotatePoint.y!=SM_UNDEF_DOUBLE    && crXUnitAxis.y!=SM_UNDEF_DOUBLE
                  && crRotatePoint.z!=SM_UNDEF_DOUBLE    && crXUnitAxis.z!=SM_UNDEF_DOUBLE
                  && crRotatePoint.z!=NL_NOZ             && crXUnitAxis.z!=NL_NOZ
                  && crRotateUnitAxis.x!=SM_UNDEF_DOUBLE && x!=SM_UNDEF_DOUBLE  
                  && crRotateUnitAxis.y!=SM_UNDEF_DOUBLE && y!=SM_UNDEF_DOUBLE 
                  && crRotateUnitAxis.z!=SM_UNDEF_DOUBLE && z!=SM_UNDEF_DOUBLE
                  && crRotateUnitAxis.z!=NL_NOZ          && z!=NL_NOZ);  
                    
  SM_ASSERT_BREAK(SM_ARE_SAME(crRotateUnitAxis.Length(),1.0));
  SM_ASSERT_BREAK(SM_ARE_SAME(crXUnitAxis.Length(),1.0));

  SmVector3d sDiffVec( (*this) - crRotatePoint );

  return (  crRotatePoint
          + crRotateUnitAxis * ( sDiffVec.Dot(crRotateUnitAxis) )
          + crXUnitAxis      * ( sDiffVec.Dot(crXUnitAxis) ) ) ;

} // end SmVector3d::RotateProjectPointToPlane

/*******************************************************************//**
PURPOSE: Project this vector to a plane towards an eye location.

NOTES: 
   - Plane normal must be a unit vector.
***********************************************************************/
inline SmVector3d SmVector3d::PerspectiveProjectToPlane
 (const SmPoint3d  & crVecBasePt,         ///< [in ]: 3d vector base pt (vec projections change based on location)
  const SmPoint3d  & crPlanePoint,        ///< [in ]: point on viewing plane
  const SmVector3d & crPlaneUnitNormal,   ///< [in ]: unit-normal to viewing plane (the viewing direction)      
  const SmPoint3d  & crEyePoint)          ///< [in ]: eye location   
 const
{
  SM_ASSERT_BREAK(   crVecBasePt.x!=SM_UNDEF_DOUBLE  && crPlaneUnitNormal.x!=SM_UNDEF_DOUBLE
                  && crVecBasePt.y!=SM_UNDEF_DOUBLE  && crPlaneUnitNormal.y!=SM_UNDEF_DOUBLE
                  && crVecBasePt.z!=SM_UNDEF_DOUBLE  && crPlaneUnitNormal.z!=SM_UNDEF_DOUBLE
                  && crVecBasePt.z!=NL_NOZ           && crPlaneUnitNormal.z!=NL_NOZ
                  && crPlanePoint.x!=SM_UNDEF_DOUBLE && crEyePoint.x!=SM_UNDEF_DOUBLE   
                  && crPlanePoint.y!=SM_UNDEF_DOUBLE && crEyePoint.y!=SM_UNDEF_DOUBLE 
                  && crPlanePoint.z!=SM_UNDEF_DOUBLE && crEyePoint.z!=SM_UNDEF_DOUBLE
                  && crPlanePoint.z!=NL_NOZ          && crEyePoint.z!=NL_NOZ
                  && x!=SM_UNDEF_DOUBLE  
                  && y!=SM_UNDEF_DOUBLE 
                  && z!=SM_UNDEF_DOUBLE
                  && z!=NL_NOZ);

  SM_ASSERT_BREAK(SM_ARE_SAME(crPlaneUnitNormal.Length(),1.0));

  SmVector3d sEyePlaneDiff(crPlanePoint           - crEyePoint) ;
  SmVector3d sEyePointDiff( crVecBasePt           - crEyePoint) ;
  SmVector3d sEyeVecDiff  ( crVecBasePt + (*this) - crEyePoint) ;

  return (  sEyeVecDiff   * (sEyePlaneDiff.Dot(crPlaneUnitNormal) / sEyeVecDiff.Dot(crPlaneUnitNormal)) 
          - sEyePointDiff * (sEyePlaneDiff.Dot(crPlaneUnitNormal) / sEyePointDiff.Dot(crPlaneUnitNormal)) );

} // end SmVector3d::PerspectiveProjectToPlane

/*******************************************************************//**
PURPOSE: Project this point to a plane towards an eye location.

NOTES: 
   - Plane normal must be a unit vector.
***********************************************************************/
inline SmPoint3d SmVector3d::PerspectiveProjectPointToPlane
 (const SmPoint3d  & crPlanePoint,        ///< [in ]: point on viewing plane
  const SmVector3d & crPlaneUnitNormal,   ///< [in ]: unit-normal to viewing plane (the viewing direction)      
  const SmPoint3d  & crEyePoint)          ///< [in ]: eye location   
  const
{
  SM_ASSERT_BREAK(   crPlanePoint.x!=SM_UNDEF_DOUBLE      && crEyePoint.x!=SM_UNDEF_DOUBLE 
                  && crPlanePoint.y!=SM_UNDEF_DOUBLE      && crEyePoint.y!=SM_UNDEF_DOUBLE 
                  && crPlanePoint.z!=SM_UNDEF_DOUBLE      && crEyePoint.z!=SM_UNDEF_DOUBLE
                  && crPlanePoint.z!=NL_NOZ               && crEyePoint.z!=NL_NOZ
                  && crPlaneUnitNormal.x!=SM_UNDEF_DOUBLE && x!=SM_UNDEF_DOUBLE  
                  && crPlaneUnitNormal.y!=SM_UNDEF_DOUBLE && y!=SM_UNDEF_DOUBLE 
                  && crPlaneUnitNormal.z!=SM_UNDEF_DOUBLE && z!=SM_UNDEF_DOUBLE
                  && crPlaneUnitNormal.z!=NL_NOZ          && z!=NL_NOZ); 

  SM_ASSERT_BREAK(SM_ARE_SAME(crPlaneUnitNormal.Length(),1.0));

  SmVector3d sEyePlaneDiff(crPlanePoint - crEyePoint) ;
  SmVector3d sEyePointDiff(  (*this)    - crEyePoint) ;

  return crEyePoint + sEyePointDiff * ( sEyePlaneDiff.Dot(crPlaneUnitNormal) / sEyePointDiff.Dot(crPlaneUnitNormal) ) ;

} // end SmVector3d::PerspectiveProjectPointToPlane

/*******************************************************************//**
PURPOSE: Compute the distance between two points.

NOTES: 
***********************************************************************/
inline double SmVector3d::DistanceBetween
  (const SmVector3d & crOther) 
 const 
{
  SM_ASSERT_BREAK(   crOther.x!=SM_UNDEF_DOUBLE && x!=SM_UNDEF_DOUBLE  
                  && crOther.y!=SM_UNDEF_DOUBLE && y!=SM_UNDEF_DOUBLE 
                  && crOther.z!=SM_UNDEF_DOUBLE && z!=SM_UNDEF_DOUBLE
                  && crOther.z!=NL_NOZ          && z!=NL_NOZ);

  return( smos_Sqrt(  (this->x - crOther.x)*(this->x - crOther.x)
                    + (this->y - crOther.y)*(this->y - crOther.y)
                    + (this->z - crOther.z)*(this->z - crOther.z))) ;

} // end SmVector3d::DistanceBetween

/*******************************************************************//**
PURPOSE: Compute the squared distance between two points.

NOTES: 
***********************************************************************/
inline double SmVector3d::DistanceBetweenSquared
  (const SmVector3d & crOther) 
 const 
{
  SM_ASSERT_BREAK(   crOther.x!=SM_UNDEF_DOUBLE && x!=SM_UNDEF_DOUBLE   
                  && crOther.y!=SM_UNDEF_DOUBLE && y!=SM_UNDEF_DOUBLE 
                  && crOther.z!=SM_UNDEF_DOUBLE && z!=SM_UNDEF_DOUBLE
                  && crOther.z!=NL_NOZ          && z!=NL_NOZ);

  return(  (this->x - crOther.x)*(this->x - crOther.x)
         + (this->y - crOther.y)*(this->y - crOther.y)
         + (this->z - crOther.z)*(this->z - crOther.z)) ;

} // end SmVector3d::DistanceBetweenSquared

/*******************************************************************//**
PURPOSE: Return TRUE if distance between two points is less than or 
            equal to a given distance,(dMax) else FALSE

NOTES: 
***********************************************************************/
inline SmBoolean SmVector3d::CloserThan // eff: rtn (Dist(this,Other) < dMax)
  (double             dMax,             ///< [in ]: dMax  of Dist(this,Other) < dMax
   const SmVector3d & crOther)          ///< [in ]: Other of Dist(this,Other) < dMax 
 const 
{ SM_ASSERT_BREAK(   x!=SM_UNDEF_DOUBLE && crOther.x!=SM_UNDEF_DOUBLE  
                  && y!=SM_UNDEF_DOUBLE && crOther.y!=SM_UNDEF_DOUBLE
                  && z!=SM_UNDEF_DOUBLE && crOther.z!=SM_UNDEF_DOUBLE
                  && z!=NL_NOZ          && crOther.z!=NL_NOZ
                  && dMax != SM_UNDEF_DOUBLE);

 if(   (smos_Fabs(x - crOther.x) > dMax) 
    || (smos_Fabs(y - crOther.y) > dMax) 
    || (smos_Fabs(z - crOther.z) > dMax) 
    || ((x - crOther.x)*(x - crOther.x) + 
        (y - crOther.y)*(y - crOther.y) +
        (z - crOther.z)*(z - crOther.z)) > dMax*dMax)
   { return (FALSE) ; }
  return TRUE ;

} // end SmVector3d::CloserThan

/*******************************************************************//**
PURPOSE: Divide the this components by the crOther components 
            and place the result in the output value, rResult.  

NOTES: Return an error if the divisor (crOther components) 
                has any zeros in it.

Example:
    SER(v1.Divide(v2,vResult));
***********************************************************************/
inline SmStatus SmVector3d::Divide
  (const SmVector3d & crOther,     ///< [in ]: Other vector 
   SmVector3d       & rResult)     ///< [out]: rResult = [x/Other->x, y/Other->y, z/Other->z]
 const
{
  SM_ASSERT_BREAK(   crOther.x!=SM_UNDEF_DOUBLE && x!=SM_UNDEF_DOUBLE  
                  && crOther.y!=SM_UNDEF_DOUBLE && y!=SM_UNDEF_DOUBLE 
                  && crOther.z!=SM_UNDEF_DOUBLE && z!=SM_UNDEF_DOUBLE
                  && crOther.z!=NL_NOZ          && z!=NL_NOZ);

  SmStatus sRet = SM_SUCCESS;
  if (   smos_Fabs(crOther.x) < SM_EFF_ZERO 
      || smos_Fabs(crOther.y) < SM_EFF_ZERO 
      || smos_Fabs(crOther.z) < SM_EFF_ZERO) 
    {
      SE(SM_ERR_INVALID_INPUT);
      sRet = SM_ERR_INVALID_INPUT;
    }
  else 
    {
      rResult.Set(this->x / crOther.x, this->y / crOther.y, this->z / crOther.z);
    }
  return sRet;

} // end SmVector3d::Divide

/*******************************************************************//**
PURPOSE: Multiply two vectors together.  This method multiplies
    the components of the vectors and returns the result in a vector.

NOTES: 

Example:
  vResult = v1.Multiply(v2);
***********************************************************************/
inline SmVector3d SmVector3d::Multiply
  (const SmVector3d & crOther) 
 const
{ SM_ASSERT_BREAK(   crOther.x!=SM_UNDEF_DOUBLE && x!=SM_UNDEF_DOUBLE  
                  && crOther.y!=SM_UNDEF_DOUBLE && y!=SM_UNDEF_DOUBLE 
                  && crOther.z!=SM_UNDEF_DOUBLE && z!=SM_UNDEF_DOUBLE
                  && crOther.z!=NL_NOZ          && z!=NL_NOZ);

  return SmVector3d(this->x * crOther.x, this->y * crOther.y, this->z * crOther.z); 

} // end SmVector3d::Multiply 

/*******************************************************************//**
PURPOSE: Compute the derivative of the cross product between two vectors.
            Given the 2 vectors and their deriviatives.

NOTES: 
***********************************************************************/
inline SmVector3d SmVector3d::DerivativeOfCrossProduct
  (const SmVector3d & crOtherVector,    ///< [in ]: OtherVector
   const SmVector3d & crThisPrime,      ///< [in ]: Derivative of this vector
   const SmVector3d & crOtherPrime)     ///< [in ]: Derivative of other vector
  const
{
  SM_ASSERT_BREAK(   crOtherVector.x!=SM_UNDEF_DOUBLE  && crOtherPrime.x!=SM_UNDEF_DOUBLE  
                  && crOtherVector.y!=SM_UNDEF_DOUBLE  && crOtherPrime.y!=SM_UNDEF_DOUBLE 
                  && crOtherVector.z!=SM_UNDEF_DOUBLE  && crOtherPrime.z!=SM_UNDEF_DOUBLE
                  && crOtherVector.z!=NL_NOZ           && crOtherPrime.z!=NL_NOZ
                  && crThisPrime.x!=SM_UNDEF_DOUBLE    && x!=SM_UNDEF_DOUBLE  
                  && crThisPrime.y!=SM_UNDEF_DOUBLE    && y!=SM_UNDEF_DOUBLE 
                  && crThisPrime.z!=SM_UNDEF_DOUBLE    && z!=SM_UNDEF_DOUBLE
                  && crThisPrime.z!=NL_NOZ             && z!=NL_NOZ);

  // return value
  SmVector3d sCPDeriv;

  const SmVector3d & a  = *this;
  const SmVector3d & b  = crOtherVector;
  const SmVector3d & da = crThisPrime;
  const SmVector3d & db = crOtherPrime;
  sCPDeriv.x = da.y*b.z + a.y*db.z - (da.z*b.y + a.z*db.y);
  sCPDeriv.y = da.z*b.x + a.z*db.x - (da.x*b.z + a.x*db.z);
  sCPDeriv.z = da.x*b.y + a.x*db.y - (da.y*b.x + a.y*db.x);

  // all done
  return sCPDeriv;

} // end SmVector3d::DerivativeOfCrossProduct

/*******************************************************************//**
PURPOSE: Set the coordinate values for a SmVector3d object.

NOTES: 
***********************************************************************/
inline void SmVector3d::Set
  (double dX, 
   double dY, 
   double dZ)
{ SM_ASSERT_BREAK(   dX!=SM_UNDEF_DOUBLE  
                  && dY!=SM_UNDEF_DOUBLE 
                  && dZ!=SM_UNDEF_DOUBLE
                  && dZ!=NL_NOZ);
  x = dX; y = dY; z = dZ; 

} // end SmVector3d::Set

/*******************************************************************//**
PURPOSE:  Solve a simple linear equations of two variables.

NOTES: 
Example:
  The Equation is as follows
    X1 * t + Y1 * u = Z1  ==>  [X1 Y1] * [t] = [Z1] ==> [t] = [ Y2 -Y1] * [Z1]
    X2 * t + Y2 * u = Z2       [X2 Y2] * [u] = [Z2]     [u] = [-X2  X1] * [Z2]

***********************************************************************/
inline SmStatus SmVector3d::SolveTwoLinearEquations
 (const SmVector3d & r2,
  SmPoint2d        & rOut) 
 const
{
  SM_ASSERT_BREAK(   r2.x!=SM_UNDEF_DOUBLE && x!=SM_UNDEF_DOUBLE 
                  && r2.y!=SM_UNDEF_DOUBLE && y!=SM_UNDEF_DOUBLE
                  && r2.z!=SM_UNDEF_DOUBLE && z!=SM_UNDEF_DOUBLE
                  && r2.z!=NL_NOZ          && z!=NL_NOZ);

  double dDenom = (r2.y*x) - (r2.x*y) ;
  if (smos_Fabs(dDenom) >= SM_EFF_ZERO_SQ) 
    {
      rOut.y = (r2.z*x - r2.x*z) / dDenom ;
      if(   (   x >=  SM_EFF_ZERO_SQ
             || x <= -SM_EFF_ZERO_SQ)
         && smos_Fabs(x) > smos_Fabs(r2.x))
      {
          rOut.x = (z - y * rOut.y) / x;  // note: x is not numerically zero
          return SM_SUCCESS;
      }
      else if(smos_Fabs(r2.x) >= SM_EFF_ZERO_SQ)
      {
          rOut.x = (r2.z - r2.y * rOut.y) / r2.x;
          return SM_SUCCESS;
      }
    }
  return SM_ERR ;

} // end SmVector3d::SolveTwoLinearEquations

/*******************************************************************//**
PURPOSE: Swap the values of these two vectors.

NOTES: 
***********************************************************************/
inline void SmVector3d::Swap
  (SmVector3d & crOtherVector)
{
  SM_ASSERT_BREAK(   crOtherVector.x!=SM_UNDEF_DOUBLE  && x!=SM_UNDEF_DOUBLE  
                  && crOtherVector.y!=SM_UNDEF_DOUBLE  && y!=SM_UNDEF_DOUBLE 
                  && crOtherVector.z!=SM_UNDEF_DOUBLE  && z!=SM_UNDEF_DOUBLE
                  && crOtherVector.z!=NL_NOZ           && z!=NL_NOZ);

  SmVector3d sVTmp = crOtherVector;
  crOtherVector    = *this;
  *this            = sVTmp;

} // end SmVector3d::Swap

/*******************************************************************//**
PURPOSE: This method allows the indexing of points component values.

NOTES: 
Example:
  Where the following: 
     0 - x component
     1 - y component
     2 - z component
***********************************************************************/
inline double SmVector3d::operator[] (ULONG lIndex) const
{ SM_ASSERT_BREAK(   x!=SM_UNDEF_DOUBLE  
                  && y!=SM_UNDEF_DOUBLE 
                  && z!=SM_UNDEF_DOUBLE
                  && z!=NL_NOZ);
  double dRet = 0;
  if      (lIndex==0) dRet = x;
  else if (lIndex==1) dRet = y;
  else if (lIndex==2) dRet = z;
  else SE(SM_ERR);
  return dRet;

} // end SmVector3d::operator[]

/*******************************************************************//**
PURPOSE: This method allows the indexing of points component references.

NOTES: 
Example:
    Where the following: 
     0 - x component
     1 - y component
     2 - z component
***********************************************************************/
inline double & SmVector3d::operator[] (ULONG lIndex)
{ double *pdRet = NULL;
  if      (lIndex==0) { pdRet = &x; }
  else if (lIndex==1) { pdRet = &y; }
  else if (lIndex==2) { pdRet = &z; }
  else                { SE(SM_ERR); }
  return *pdRet;

} // end SmVector3d::operator[]

/*******************************************************************//**
PURPOSE: This is the vector test for equality.

NOTES: 
***********************************************************************/
inline int operator==
  (const SmVector3d & crVec1, 
   const SmVector3d & crVec2)
{ 
  SmBoolean bInit1 = crVec1.IsInitialized() ;
  SmBoolean bInit2 = crVec2.IsInitialized() ;

  // case: two uninitialized points
  if(!bInit1 && !bInit2)
    { return(TRUE) ; }

  // case: one uninitialized point
  if(!bInit1 || !bInit2)
   { return(FALSE) ; }

  // case: two initialized points
  int bRet = FALSE;
  double dScaledZero = SM_EFF_ZERO * (1.0 + crVec1.GetMaxDimension());
  double dDist  =  smos_Fabs(crVec1.x-crVec2.x) 
                 + smos_Fabs(crVec1.y-crVec2.y)
                 + smos_Fabs(crVec1.z-crVec2.z);
  if (dDist < dScaledZero) bRet = TRUE;
  return bRet;

} // end operator==

/*******************************************************************//**
PURPOSE: The += operator for vectors.

NOTES: 
***********************************************************************/
inline SmVector3d & SmVector3d::operator+=
  (const SmVector3d & rVec2)
{ SM_ASSERT_BREAK(   x!=SM_UNDEF_DOUBLE  && rVec2.x!=SM_UNDEF_DOUBLE   
                  && y!=SM_UNDEF_DOUBLE  && rVec2.y!=SM_UNDEF_DOUBLE 
                  && z!=SM_UNDEF_DOUBLE  && rVec2.z!=SM_UNDEF_DOUBLE
                  && z!=NL_NOZ           && rVec2.z!=NL_NOZ);
                 
  x += rVec2.x;
  y += rVec2.y;
  z += rVec2.z;
  return *this;

} // end SmVector3d::operator+=

/*******************************************************************//**
PURPOSE: The -= operator for vectors.

NOTES: 
***********************************************************************/
inline SmVector3d & SmVector3d::operator-=
  (const SmVector3d & rVec2)
{ SM_ASSERT_BREAK(   x!=SM_UNDEF_DOUBLE && rVec2.x!=SM_UNDEF_DOUBLE  
                  && y!=SM_UNDEF_DOUBLE && rVec2.y!=SM_UNDEF_DOUBLE 
                  && z!=SM_UNDEF_DOUBLE && rVec2.z!=SM_UNDEF_DOUBLE
                  && z!=NL_NOZ          && rVec2.z!=NL_NOZ);
                  
  x -= rVec2.x;
  y -= rVec2.y;
  z -= rVec2.z;
  return *this;

} // end SmVector3d::operator-=

/*******************************************************************//**
PURPOSE: The *= operator for vectors with scalars.

NOTES: 
***********************************************************************/
inline SmVector3d & SmVector3d::operator*=
  (double d)
{ SM_ASSERT_BREAK(   x!=SM_UNDEF_DOUBLE  
                  && y!=SM_UNDEF_DOUBLE 
                  && z!=SM_UNDEF_DOUBLE
                  && z!=NL_NOZ);
  x *= d;
  y *= d;
  z *= d;
  return *this;

} // end SmVector3d::operator*=

/*******************************************************************//**
PURPOSE: The /= operator for vectors with scalars.

NOTES: 
***********************************************************************/
inline SmVector3d & SmVector3d::operator/=
  (double d)
{ SM_ASSERT_BREAK(   x!=SM_UNDEF_DOUBLE  
                  && y!=SM_UNDEF_DOUBLE 
                  && z!=SM_UNDEF_DOUBLE
                  && z!=NL_NOZ);

  double dDenomChk = smos_Fabs( d );
  if (    dDenomChk <= SM_EFF_ZERO * smos_Fabs( x )
       && dDenomChk <= SM_EFF_ZERO * smos_Fabs( y )
       && dDenomChk <= SM_EFF_ZERO * smos_Fabs( z )
     )
    { return *this; }

  x /= d;
  y /= d;
  z /= d;
  return *this;

} // end SmVector3d::operator/=

#endif // !__SMVECTOR3D_H__


