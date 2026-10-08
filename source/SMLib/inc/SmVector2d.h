// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*******************************************************************//**
* FILE NAME --- SmVector2d.h
* PURPOSE: Header file for SmVector2d
**********************************************************************/

#ifndef __SMVECTOR2D_H__
#define __SMVECTOR2D_H__

#ifndef __SMCORE_TYPES_H__
#include <SmCoreTypes.h>
#endif

#ifndef __SMOS_MATH_H__
#include <SmMath.h>
#endif

#ifdef Length
#undef Length
#endif /* Length */

// forward declarations
class SmVector2d;
class SmDisplayList ;
class SmGfxArraySet ;

// Note: Borland compiler does not like this statement
//typedef struct _iobuf FILE;

inline SmVector2d operator - (const SmVector2d& v);                        // -v1
inline SmVector2d operator + (const SmVector2d& a, const SmVector2d& b);   // v1 + v2
inline SmVector2d operator - (const SmVector2d& a, const SmVector2d& b);   // v1 - v2
inline SmVector2d operator * (const SmVector2d& a, const double d);        // v1 * 3.0
inline SmVector2d operator * (const double d, const SmVector2d& a);        // 3.0 * v1
inline SmVector2d operator / (const SmVector2d& a, const double d);        // v1 / 3.0    
inline int        operator== (const SmVector2d & crVec1, const SmVector2d & crVec2);
inline int        operator!= (const SmVector2d & crVec1, const SmVector2d & crVec2);

/*******************************************************************//**
PURPOSE: This object is a two dimensional vector.  It is represented
   by an X and Y value.

NOTES: SmPoint2d is another name for this class and they
   may be used interchangably.

   NO VIRTUAL METHODS: The members of this class are accessed in arrays by offsets.  
   A virtual function introduces a virtual function pointer to every object
   thus destroying the offset trick, so No Virtual Methods in this class.
***********************************************************************/
class SM_EXPORT SmVector2d
{
protected:
public:
  double x = SM_UNDEF_DOUBLE ;  // X coordinate value - sometimes interpreted as U in parameter space
  double y = SM_UNDEF_DOUBLE ;  // Y coordinate value - sometimes interpreted as V in parameter space
  SmVector2d() = default;

  SmVector2d( double dX, double dY )
  {
    SM_ASSERT_BREAK( dX != SM_UNDEF_DOUBLE && dY != SM_UNDEF_DOUBLE );
    x = dX; y = dY;
  }

  SmVector2d( double *pVec )
  {
    SM_ASSERT_BREAK( pVec[0] != SM_UNDEF_DOUBLE && pVec[1] != SM_UNDEF_DOUBLE );
    if(pVec) { x = pVec[0]; y = pVec[1]; }
    else { x = y = SM_UNDEF_DOUBLE; }
  }

  SmVector2d( const SmVector2d & crVector )
  {
    SM_ASSERT_BREAK( crVector.x != SM_UNDEF_DOUBLE && crVector.y != SM_UNDEF_DOUBLE );
    x = crVector.x;   y = crVector.y;
  }

  ~SmVector2d() = default;

  SmVector2d& operator=(SmVector3d const &obj) ; // convenience function for - Point2d.x = Point3d.x; Point2d.y = Point3d.y

  // assign only initialized values
  SmVector2d& operator=( SmVector2d const &obj )
  {
    if(&obj == this) return *this;
    x = obj.x; y = obj.y;
    return *this;
  }

  // copy - copy initialized and uninitialized values
  SmVector2d& Copy( SmVector2d const &obj )
  {
    if(&obj == this) return *this;
    x = obj.x; y = obj.y;
    return *this;
  }

  void       SetUninitialized() { x = y = SM_UNDEF_DOUBLE; }

  SmBoolean  IsZero       (double dDistTol3d=SM_EFF_ZERO) const { return( LengthSquared() < dDistTol3d * dDistTol3d) ; }

  int        IsInTriangle (const SmPoint2d & rP0,const SmPoint2d & rP1,const SmPoint2d & rP2) const;

  SmBoolean  IsInitialized() const { return(x != SM_UNDEF_DOUBLE && y != SM_UNDEF_DOUBLE ); }

  SmBoolean  IsUndef() const
  {
    return(x == SM_UNDEF_DOUBLE || !SM_IS_VALID_DOUBLE( x ) || y == SM_UNDEF_DOUBLE || !SM_IS_VALID_DOUBLE( y ));
  }

  SmStatus   AngleBetween          (const SmVector2d & crOther, double & rdAngRad) const;  // angle [0 to PI]

  SmStatus   CCWAngleBetween       (const SmVector2d & crOther, double & rdAngRad) const;  // from this to other [0 to 2PI]

  double     LengthSquared( void ) const
  {
    SM_ASSERT_BREAK( x != SM_UNDEF_DOUBLE && y != SM_UNDEF_DOUBLE );
    return (x*x) + (y*y);
  }

  double     Length( void ) const
  {
    SM_ASSERT_BREAK( x != SM_UNDEF_DOUBLE && y != SM_UNDEF_DOUBLE );
    return smos_Sqrt( (x*x) + (y*y) );
  }

  SmStatus   Unitize               (void);

  double     Cross( const SmVector2d & crOther ) const
  {
    SM_ASSERT_BREAK( x != SM_UNDEF_DOUBLE && y != SM_UNDEF_DOUBLE );
    return (x*crOther.y) - (y*crOther.x);
  }

  double     Dot( const SmVector2d & crOther ) const
  {
    SM_ASSERT_BREAK( x != SM_UNDEF_DOUBLE && y != SM_UNDEF_DOUBLE );
    return (x*crOther.x) + (y*crOther.y);
  }

  double     DistanceBetween       (const SmVector2d & crOther) const;
  double     DistanceBetweenSquared(const SmVector2d & crOther) const;
  SmBoolean  CloserThan            (double dMax, const SmVector2d & crOther) const;
  SmStatus   Divide                (const SmVector2d & crOther, SmVector2d & rResult) const;
  double     GetMaxDimension       () const;
  double     GetMinDimension       () const;
  SmVector2d Multiply              (const SmVector2d & crOther) const; // return(x*crOther.x, y*crOther.y)
  SmVector2d Rotate                (double dAngRad) const ;            // return vec rotated CCW about a Z axis
  void       Set                   (double dX, double dY);
  void       SwapXY                ();

  friend SmVector2d operator - (const SmVector2d& v);                        // -v1
  friend SmVector2d operator + (const SmVector2d& a, const SmVector2d& b);   // v1 + v2
  friend SmVector2d operator - (const SmVector2d& a, const SmVector2d& b);   // v1 - v2
  friend SmVector2d operator * (const SmVector2d& a, const double d);        // v1 * 3.0
  friend SmVector2d operator * (const double d, const SmVector2d& a);        // 3.0 * v1
  friend SmVector2d operator / (const SmVector2d& a, const double d);        // v1 / 3.0    
  friend int operator==(const SmVector2d & crVec1, const SmVector2d & crVec2);
  friend int operator!=(const SmVector2d & crVec1, const SmVector2d & crVec2);

  double   operator[] (ULONG lIndex) const;   // can use lIndex = [0, 0, SM_SP_U, SM_SP_V]
  double & operator[] (ULONG lIndex);         // can use lIndex = [0, 0, SM_SP_U, SM_SP_V]

  SmVector2d & operator+=(const SmVector2d & rVec2);
  SmVector2d & operator-=(const SmVector2d & rVec2);
  SmVector2d & operator*=(double d);
  SmVector2d & operator/=(double d);

  // utilities
  // note: do not use SM_COMMON_BASE() because no virtual methods are allowed for SmVector2d
  void            Dump (void)                                 const;
  void            Dump (const TCHAR * message)                const;
  void            Dump (ULONG)                                const;
  SmDisplayList * Draw (const SmVector2d * pVectorOrigin=NULL,        // in :
                        const SmContext  * pContext=NULL,             // NotUsed: in :
                        SmGfxArraySet    * pOptGfxSet=NULL)   const;  // in :
  SmStatus        Write(FILE *pFile)                          const;
  SmStatus        Read (FILE *pFile);
  const TCHAR   * GetTypeString() const { return _T("SmVector2d") ; }
  SM_TYPE         GetClassType()        { return SmVector2d_TYPE; }

} ; // end class SmVector2d

// Friend functions

/*******************************************************************//**
PURPOSE: Negate a vector.

NOTES: vResult = - v;
***********************************************************************/
inline SmVector2d operator - ( const SmVector2d& v )
{
  SM_ASSERT_BREAK( v.x != SM_UNDEF_DOUBLE
                 && v.y != SM_UNDEF_DOUBLE );

  return SmVector2d( -v.x, -v.y );
}

/*******************************************************************//**
PURPOSE: Add two vectors.

NOTES: vResult = v1 + v2;
***********************************************************************/
inline SmVector2d operator + ( const SmVector2d& a, const SmVector2d& b )
{
  SM_ASSERT_BREAK( a.x != SM_UNDEF_DOUBLE
                 && a.y != SM_UNDEF_DOUBLE
                 && b.x != SM_UNDEF_DOUBLE
                 && b.y != SM_UNDEF_DOUBLE );

  return SmVector2d( a.x + b.x, a.y + b.y );
}
    
/*******************************************************************//**
PURPOSE: Subtract one vector from another.

NOTES: vResult = v1 - v2;
***********************************************************************/
inline SmVector2d operator - ( const SmVector2d& a, const SmVector2d& b )
{
  SM_ASSERT_BREAK( a.x != SM_UNDEF_DOUBLE
                 && a.y != SM_UNDEF_DOUBLE
                 && b.x != SM_UNDEF_DOUBLE
                 && b.y != SM_UNDEF_DOUBLE );

  return SmVector2d( a.x - b.x, a.y - b.y );
}

/*******************************************************************//**
PURPOSE: Multiply a vector by a constant.

NOTES: vResult = v * 2.0;
***********************************************************************/
inline SmVector2d operator * ( const SmVector2d& a, const double d )
{
  SM_ASSERT_BREAK( a.x != SM_UNDEF_DOUBLE
                 && a.y != SM_UNDEF_DOUBLE );

  return SmVector2d( a.x * d, a.y * d );
}

/*******************************************************************//**
PURPOSE: Multiply a constant by a vector.

NOTES: vResult = 3.0 * v;
***********************************************************************/
inline SmVector2d operator * ( const double d, const SmVector2d& a )
{
  SM_ASSERT_BREAK( a.x != SM_UNDEF_DOUBLE
                 && a.y != SM_UNDEF_DOUBLE );

  return SmVector2d( a.x * d, a.y * d );
}

/*******************************************************************//**
PURPOSE: Divide a vector by a constant.

NOTES: vResult = v / 2.0;
***********************************************************************/
inline SmVector2d operator / ( const SmVector2d& a, const double d )
{
  SM_ASSERT_BREAK( d != 0.0 );
  SM_ASSERT_BREAK( a.x != SM_UNDEF_DOUBLE
                  && a.y != SM_UNDEF_DOUBLE );

  if ( smos_Fabs(d) < smos_Fabs(a.x) * SM_EFF_ZERO )
    { return a; }
  if ( smos_Fabs(d) < smos_Fabs(a.y) * SM_EFF_ZERO )
    { return a; }

  return SmVector2d( a.x / d, a.y / d );
}

// Non trivial inline member functions

/*******************************************************************//**
PURPOSE: Unitize a vector.  

NOTES: Return an error if the vector is a zero
    vector.  SER(v.Unitize());
***********************************************************************/
inline SmStatus SmVector2d::Unitize(void)
{
  SM_ASSERT_BREAK(   x!=SM_UNDEF_DOUBLE  
                  && y!=SM_UNDEF_DOUBLE);

  double dLenSq = LengthSquared();
  SmStatus sRet = SM_SUCCESS;
  if (dLenSq < SM_EFF_ZERO_SQ) 
       {
         SE(SM_ERR_INVALID_INPUT);
         sRet = SM_ERR_INVALID_INPUT; 
       } 
  else if (smos_Fabs (dLenSq - 1.0) > SM_EFF_ZERO_SQ)
  {
      double dLen = smos_Sqrt(dLenSq);
      x = x/dLen; y = y/dLen; 
  }

  return sRet;  

} // end SmVector2d::Unitize


/*******************************************************************//**
PURPOSE: Compute the distance between two points.

NOTES: 
***********************************************************************/
inline double SmVector2d::DistanceBetween( const SmVector2d & crOther ) const
{
  SM_ASSERT_BREAK( x != SM_UNDEF_DOUBLE
                 && y != SM_UNDEF_DOUBLE
                 && crOther.x != SM_UNDEF_DOUBLE
                 && crOther.y != SM_UNDEF_DOUBLE );
  SmVector2d sBetween = *this - crOther;
  return sBetween.Length();
}

/*******************************************************************//**
PURPOSE: Compute the squared distance between two points.

NOTES: 
***********************************************************************/
inline double SmVector2d::DistanceBetweenSquared( const SmVector2d & crOther ) const
{
  SM_ASSERT_BREAK( x != SM_UNDEF_DOUBLE
                 && y != SM_UNDEF_DOUBLE
                 && crOther.x != SM_UNDEF_DOUBLE
                 && crOther.y != SM_UNDEF_DOUBLE );
  SmVector2d sBetween = *this - crOther;
  return sBetween.LengthSquared();
}

/*******************************************************************//**
PURPOSE: Return TRUE if distance between two points is less 
            than or equal to given distance (dMax), else FALSE

NOTES: 
***********************************************************************/
inline SmBoolean SmVector2d::CloserThan( double dMax,const SmVector2d & crOther ) const
{
  SM_ASSERT_BREAK( x != SM_UNDEF_DOUBLE
                 && y != SM_UNDEF_DOUBLE
                 && crOther.x != SM_UNDEF_DOUBLE
                 && crOther.y != SM_UNDEF_DOUBLE
                 && dMax != SM_UNDEF_DOUBLE );

  if((smos_Fabs( x - crOther.x ) > dMax)
      || (smos_Fabs( y - crOther.y ) > dMax)
      || ((x - crOther.x)*(x - crOther.x)
      + (y - crOther.y)*(y - crOther.y)) > dMax*dMax)
    return (FALSE);
  return TRUE;
}

/*******************************************************************//**
PURPOSE: Divide the values of this by crOther and return the result
    of the component wise division in rResult.  

NOTES: Return an error if the divisor (crOther) has any
    zeros in it.
    SER(v1.Divide(v2,vResult));
***********************************************************************/
inline SmStatus SmVector2d::Divide( const SmVector2d & crOther, SmVector2d       & rResult ) const
{
  SM_ASSERT_BREAK( x != SM_UNDEF_DOUBLE
                 && y != SM_UNDEF_DOUBLE
                 && crOther.x != SM_UNDEF_DOUBLE
                 && crOther.y != SM_UNDEF_DOUBLE );

  SmStatus sRet = SM_SUCCESS;
  if(smos_Fabs( crOther.x ) < SM_EFF_ZERO ||
      smos_Fabs( crOther.y ) < SM_EFF_ZERO)
  {
    SE( SM_ERR_INVALID_INPUT );
    sRet = SM_ERR_INVALID_INPUT;
  }
  else
  {
    rResult.Set( this->x / crOther.x, this->y / crOther.y );
  }
  return sRet;
}

/*******************************************************************//**
PURPOSE: Get the absolute value of the maximum coordinate dimension.

NOTES: 
***********************************************************************/
inline double SmVector2d::GetMaxDimension() const
{
  SM_ASSERT_BREAK( x != SM_UNDEF_DOUBLE
                 && y != SM_UNDEF_DOUBLE );
  return(smos_Fabs( x ) > smos_Fabs( y )
         ? smos_Fabs( x )
         : smos_Fabs( y ));
}

/*******************************************************************//**
PURPOSE: Get the absolute value of the minimum coordinate dimension.

NOTES: 
***********************************************************************/
inline double SmVector2d::GetMinDimension() const
{
  SM_ASSERT_BREAK( x != SM_UNDEF_DOUBLE
                 && y != SM_UNDEF_DOUBLE );
  return(smos_Fabs( x ) < smos_Fabs( y )
         ? smos_Fabs( x )
         : smos_Fabs( y ));
}

/*******************************************************************//**
PURPOSE: Set the coordinate values for a SmVector2d object.

NOTES: 
***********************************************************************/
inline void SmVector2d::Set( double dX, double dY )
{
  SM_ASSERT_BREAK( dX != SM_UNDEF_DOUBLE
                 && dY != SM_UNDEF_DOUBLE );
  x = dX; y = dY;
}

/*******************************************************************//**
PURPOSE: Swap the coordinate values for a SmVector2d object.

NOTES: 
***********************************************************************/
inline void SmVector2d::SwapXY()
{
  SM_ASSERT_BREAK( x != SM_UNDEF_DOUBLE
                 && y != SM_UNDEF_DOUBLE );
  double dTmp = x; x = y; y = dTmp;
}

/*******************************************************************//**
PURPOSE: Multiply two vectors together.  This method multiplies
    the components of the vectors and returns the result in a vector.

NOTES: vResult = v1.Multiply(v2);
***********************************************************************/
inline SmVector2d SmVector2d::Multiply( const SmVector2d & crOther ) const
{
  SM_ASSERT_BREAK( x != SM_UNDEF_DOUBLE
                 && y != SM_UNDEF_DOUBLE
                 && crOther.x != SM_UNDEF_DOUBLE
                 && crOther.y != SM_UNDEF_DOUBLE );
  return SmVector2d( this->x * crOther.x, this->y * crOther.y );
}

/*******************************************************************//**
PURPOSE: Return vec rotated from this vector CCW about a Z axis

NOTES: vResult = v1.Multiply(v2);
***********************************************************************/
inline SmVector2d SmVector2d::Rotate( double dAngRad ) const
{
  SM_ASSERT_BREAK( x != SM_UNDEF_DOUBLE
                 && y != SM_UNDEF_DOUBLE );
  double dCos = smos_Cosine( dAngRad );
  double dSin = smos_Sine( dAngRad );

  return SmVector2d( dCos * x - dSin * y, dSin * x + dCos * y );
}

/*******************************************************************//**
PURPOSE: Test for equality.

NOTES: 
***********************************************************************/
inline int operator== ( const SmVector2d & crVec1, const SmVector2d & crVec2 )
{
  SM_ASSERT_BREAK( crVec1.x != SM_UNDEF_DOUBLE
                 && crVec1.y != SM_UNDEF_DOUBLE
                 && crVec2.x != SM_UNDEF_DOUBLE
                 && crVec2.y != SM_UNDEF_DOUBLE );
  int bRet = FALSE;
  double dScale = (1.0 + crVec1.GetMaxDimension());
  double dDist = smos_Fabs( crVec1.x - crVec2.x ) + smos_Fabs( crVec1.y - crVec2.y );
  if(dDist < SM_EFF_ZERO*dScale) { bRet = TRUE; }
  return bRet;

} // end operator==

/*******************************************************************//**
PURPOSE: Test for inequality.

NOTES: 
***********************************************************************/
inline int operator!= ( const SmVector2d & crVec1, const SmVector2d & crVec2 )
{
  SM_ASSERT_BREAK( crVec1.x != SM_UNDEF_DOUBLE
                 && crVec1.y != SM_UNDEF_DOUBLE
                 && crVec2.x != SM_UNDEF_DOUBLE
                 && crVec2.y != SM_UNDEF_DOUBLE );
  int bRet = FALSE;
  double dScale = (1.0 + crVec1.GetMaxDimension());
  double dDist = smos_Fabs( crVec1.x - crVec2.x ) + smos_Fabs( crVec1.y - crVec2.y );
  if(dDist >= SM_EFF_ZERO*dScale) bRet = TRUE;
  return bRet;

} // end operator!=

/*******************************************************************//**
PURPOSE: This method allows the indexing of points component values.

NOTES:  

Example:
  Where the following: 
    0 - x component
    1 - y component


***********************************************************************/
inline double SmVector2d::operator[] ( ULONG lIndex ) const
{
  SM_ASSERT_BREAK( x != SM_UNDEF_DOUBLE
                 && y != SM_UNDEF_DOUBLE );
  double dRet = 0;
  if(lIndex == 0) { dRet = x; }
  else if(lIndex == 1) { dRet = y; }
  else { SE( SM_ERR ); }
  return dRet;
}

/*******************************************************************//**
PURPOSE: This method allows the indexing of points component references.
    
NOTES: 

Example:
  Where the following: 
    0 - x component
    1 - y component

***********************************************************************/
inline double & SmVector2d::operator[] ( ULONG lIndex )
{
  double *pdRet = NULL;
  if(lIndex == 0) { pdRet = &x; }
  else if(lIndex == 1) { pdRet = &y; }
  else { SE( SM_ERR ); }
  return *pdRet;
}

/*******************************************************************//**
PURPOSE: The += operator for vectors.

NOTES: 
***********************************************************************/
inline SmVector2d & SmVector2d::operator+=( const SmVector2d & rVec2 )
{
  SM_ASSERT_BREAK( x != SM_UNDEF_DOUBLE
                 && y != SM_UNDEF_DOUBLE
                 && rVec2.x != SM_UNDEF_DOUBLE
                 && rVec2.y != SM_UNDEF_DOUBLE );
  x += rVec2.x;
  y += rVec2.y;
  return *this;
}
                             
/*******************************************************************//**
PURPOSE: The -= operator for vectors.

NOTES: 
***********************************************************************/
inline SmVector2d & SmVector2d::operator-=( const SmVector2d & rVec2 )
{
  x -= rVec2.x;
  y -= rVec2.y;
  return *this;
}
                             
/*******************************************************************//**
PURPOSE: The *= operator for vectors with scalars.

NOTES: 
***********************************************************************/
inline SmVector2d & SmVector2d::operator*=( double d )
{
  SM_ASSERT_BREAK( x != SM_UNDEF_DOUBLE
                 && y != SM_UNDEF_DOUBLE );
  x *= d;
  y *= d;
  return *this;
}
                             
/*******************************************************************//**
PURPOSE: The /= operator for vectors with scalars.

NOTES: 
***********************************************************************/
inline SmVector2d & SmVector2d::operator/=( double d )
{
  SM_ASSERT_BREAK( x != SM_UNDEF_DOUBLE
                 && y != SM_UNDEF_DOUBLE );

  if ( smos_Fabs(d) < smos_Fabs(x) * SM_EFF_ZERO )
    { return *this; }
  if ( smos_Fabs(d) < smos_Fabs(y) * SM_EFF_ZERO )
    { return *this; }

  x /= d;
  y /= d;
  return *this;
}
                             
#endif // !__SMVECTOR2D_H__


