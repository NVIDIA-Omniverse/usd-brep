// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*******************************************************************//**
* FILE NAME --- SmVectorNd.h
* PURPOSE: Header file for SmVectorNd
**********************************************************************/

#ifndef __SMVECTORND_H__
#define __SMVECTORND_H__

#ifndef __SMCORE_TYPES_H__
#include <SmCoreTypes.h>
#endif

#ifndef __SMTARRAY_H__
#include <SmTArray.h>
#endif

#ifdef Length
#undef Length
#endif /* Length */

#define SM_VECND(name, size) \
    double name##Data[size]; \
    SmVectorNd name(size, name##Data, size)

class SmVectorNd;

// These are supported but composition of multiple expressions a = b + c * d can
// result in memory allocation/deallocation.                         
inline SmVectorNd operator - (const SmVectorNd& v);                        // -v1
inline SmVectorNd operator + (const SmVectorNd& a, const SmVectorNd& b);   // v1 + v2
inline SmVectorNd operator - (const SmVectorNd& a, const SmVectorNd& b);   // v1 - v2
inline SmVectorNd operator * (const SmVectorNd& a, const double d);        // v1 * 3.0
inline SmVectorNd operator * (const double d, const SmVectorNd& a);        // 3.0 * v1
inline SmVectorNd operator * (const SmVectorNd& a, const SmVectorNd& b);   // cross product
inline SmVectorNd operator / (const SmVectorNd& a, const double d);        // v1 / 3.0 

/*******************************************************************//**
PURPOSE: This object is a N-dimensional vector.  It is represented
   by an array of doubles.  

NOTES: 
***********************************************************************/
class SM_EXPORT SmVectorNd
{  
protected:
  SmTArray<double> m_vVec;

public:
  // default constructor
  SmVectorNd(ULONG    lDataSize=0,                   
             double * pData = NULL,                  
             ULONG    lInitialSize = 0); 
  
  // copy constructor            
  SmVectorNd(const SmVectorNd & crVector)               { m_vVec.ReSet(); 
                                                          m_vVec.Append(crVector.m_vVec); 
                                                        }
  // destructor                                        
 ~SmVectorNd()                                          { }
                                                        
  // data access
  ULONG                    GetSize()  const             { return m_vVec.GetSize(); }
  const SmTArray<double> & GetArray() const             { return m_vVec; }

  void        SetSize    (ULONG lNumElements)           { m_vVec.SetSize(lNumElements); }
  inline void SetValues  (ULONG         lStartIndex, 
                          const double * cpDoubles, 
                          ULONG          lNumberToAdd);
  double      operator[] (ULONG lIndex) const           { return m_vVec[lIndex]; }
  double &    operator[] (ULONG lIndex)                 { return m_vVec[lIndex]; }
  inline void Swap       (SmVectorNd & crOtherVector);

  // operations
  inline SmStatus  Unitize() ;  
  inline double    Length()                                                 const;   
  inline double    LengthSquared()                                          const;    
  inline double    Dot            (const SmVectorNd & crOther)              const;
  inline SmBoolean CloserThan     (double dMax, const SmVectorNd & crOther) const;
  inline double    DistanceBetween(const SmVectorNd & crOther)              const;
  inline double    DistanceBetweenSquared(const SmVectorNd & crOther)       const;

  // operators
  friend SmVectorNd operator - (const SmVectorNd& v);                      // -v1
  friend SmVectorNd operator + (const SmVectorNd& a, const SmVectorNd& b); // v1 + v2
  friend SmVectorNd operator - (const SmVectorNd& a, const SmVectorNd& b); // v1 - v2
  friend SmVectorNd operator * (const SmVectorNd& a, const double d);      // v1 * d
  friend SmVectorNd operator * (const double d, const SmVectorNd& a);      // d  * v1
  friend SmVectorNd operator * (const SmVectorNd& a, const SmVectorNd& b); // v1 * v2
  friend SmVectorNd operator / (const SmVectorNd& a, const double d);      // v1 / d 
  friend SmVectorNd operator / (const SmVectorNd& a, const SmVectorNd& b); // v1 / v2 

  // Use of the incremental operators is prefered because they will not
  // allocate new memory.
  SmVectorNd& operator += (const SmVectorNd& a); // v1 += v2 - add v1 and v2 and store results in v1
  SmVectorNd& operator -= (const SmVectorNd& a); // v1 -= v2 - v1 minus v2 and store results in v1
  SmVectorNd& operator *= (const SmVectorNd& a); // v1 *= v2 - component multiplication and store results in v1
  SmVectorNd& operator *= (double a);            // v1 *= d - component multiplication by a double and store results in v1
  SmVectorNd& operator /= (const SmVectorNd& a); // v1 /= v2 - component multiplication and store results in v1
  SmVectorNd& operator /= (double a);            // v1 /= d - component multiplication by a double and store results in v1

  // get memory used and allocated
  ULONG GetMemoryUsed(ULONG &rlMemoryAllocated) const 
     { ULONG lAllocated, lUsed = m_vVec.GetMemoryUsed(lAllocated) ;
       rlMemoryAllocated = sizeof(this) + lAllocated - sizeof(SmTArray<SmAObject*>) ;
       return(             sizeof(this) + lUsed      - sizeof(SmTArray<SmAObject*>)) ;
     }
 
  // utilities
  void Dump(void) const;
  const TCHAR * GetTypeString() const { return _T("SmVectorNd") ; }

} ; // end class SmVectorNd

// Friend functions

/*******************************************************************//**
PURPOSE: Insert some doubles into this vector starting at the 
    lStartIndex.

NOTES: 
***********************************************************************/
inline void SmVectorNd::SetValues(ULONG lStartIndex, const double * cpDoubles, ULONG lNumberToAdd)
{
    if (lStartIndex+lNumberToAdd > GetSize()) {
        SetSize(lStartIndex+lNumberToAdd);
    }
    for (ULONG i=0; i<lNumberToAdd; i++) {
        m_vVec[i+lStartIndex] = cpDoubles[i];
    }
}

/*******************************************************************//**
PURPOSE: += Operator implementation

NOTES: vResult += v;
***********************************************************************/
inline SmVectorNd& SmVectorNd::operator += (const SmVectorNd& v)
{
    if (GetSize() != v.GetSize()) {
        SE(SM_ERR);
    }
    for (ULONG i=0; i<GetSize(); i++) {
        m_vVec[i] += v.m_vVec[i];
    }
    return *this;
}

/*******************************************************************//**
PURPOSE: *= Operator implementation

NOTES: vResult *= v;
***********************************************************************/
inline SmVectorNd& SmVectorNd::operator -= (const SmVectorNd& v)
{
    if (GetSize() != v.GetSize()) {
        SE(SM_ERR);
    }
    for (ULONG i=0; i<GetSize(); i++) {
        m_vVec[i] *= v.m_vVec[i];
    }
    return *this;
}

/*******************************************************************//**
PURPOSE: *= Operator implementation

NOTES: vResult *= doubleval;
***********************************************************************/
inline SmVectorNd& SmVectorNd::operator *= (double dValue)
{
    for (ULONG i=0; i<GetSize(); i++) {
        m_vVec[i] *= dValue;
    }
    return *this;
}

/*******************************************************************//**
PURPOSE: *= Operator implementation

NOTES: vResult *= v;
***********************************************************************/
inline SmVectorNd& SmVectorNd::operator *= (const SmVectorNd& v)
{
    if (GetSize() != v.GetSize()) {
        SE(SM_ERR);
    }
    for (ULONG i=0; i<GetSize(); i++) {
        m_vVec[i] *= v.m_vVec[i];
    }
    return *this;
}

/*******************************************************************//**
PURPOSE: /= Operator implementation

NOTES: vResult /= v;
***********************************************************************/
inline SmVectorNd& SmVectorNd::operator /= (const SmVectorNd& v)
{
    if (GetSize() != v.GetSize()) {
        SE(SM_ERR);
    }
    for (ULONG i=0; i<GetSize(); i++) {
        m_vVec[i] /= v.m_vVec[i];
    }
    return *this;
}

/*******************************************************************//**
PURPOSE: /= Operator implementation

NOTES: vResult -= doublevalue;
***********************************************************************/
inline SmVectorNd& SmVectorNd::operator /= (double dValue)
{
    for (ULONG i=0; i<GetSize(); i++) {
        m_vVec[i] /= dValue;
    }
    return *this;
}

/*******************************************************************//**
PURPOSE: Negate a vector.

NOTES: vResult = - v;
***********************************************************************/
inline SmVectorNd operator - (const SmVectorNd& v)
{
    double dData[32];
    SmVectorNd sTmpVec(32,dData);
    sTmpVec.SetSize(v.GetSize());
    for (ULONG i=0; i<v.GetSize(); i++) {
        sTmpVec[i] = - v[i];
    }
    return sTmpVec;
}

/*******************************************************************//**
PURPOSE: Add two vectors.

NOTES: vResult = v1 + v2;
***********************************************************************/
inline SmVectorNd operator + (const SmVectorNd& a, const SmVectorNd& b)
{
    double dData[32];
    SmVectorNd sTmpVec(32,dData);
    if (a.GetSize() != b.GetSize()) {
        SE(SM_ERR);
    }
    ULONG lSize = smos_Min(a.GetSize(),b.GetSize());
    sTmpVec.SetSize(lSize);
    for (ULONG i=0; i<lSize; i++) {
        sTmpVec[i] = a[i] + b[i];
    }
    return sTmpVec;
}

    
/*******************************************************************//**
PURPOSE: Subtract one vector from another.

NOTES: vResult = v1 - v2;
***********************************************************************/
inline SmVectorNd operator - (const SmVectorNd& a, const SmVectorNd& b)
{
    double dData[32];
    SmVectorNd sTmpVec(32,dData);
    if (a.GetSize() != b.GetSize()) {
        SE(SM_ERR);
    }
    ULONG lSize = smos_Min(a.GetSize(),b.GetSize());
    sTmpVec.SetSize(lSize);
    for (ULONG i=0; i<lSize; i++) {
        sTmpVec[i] = a[i] - b[i];
    }
    return sTmpVec;
}

/*******************************************************************//**
PURPOSE: Multiply a vector by a constant.

NOTES: vResult = v * 2.0;
***********************************************************************/
inline SmVectorNd operator * (const SmVectorNd& a, const double d)
{
    double dData[32];
    SmVectorNd sTmpVec(32,dData);
    sTmpVec.SetSize(a.GetSize());
    for (ULONG i=0; i<sTmpVec.GetSize(); i++) {
        sTmpVec[i] = a[i] * d;
    }
    return sTmpVec;
}


/*******************************************************************//**
PURPOSE: Multiply a constant by a vector.

NOTES: vResult = 3.0 * v;
***********************************************************************/
inline SmVectorNd operator * (const double d, const SmVectorNd& a)
{
    double dData[32];
    SmVectorNd sTmpVec(32,dData);
    sTmpVec.SetSize(a.GetSize());
    for (ULONG i=0; i<sTmpVec.GetSize(); i++) {
        sTmpVec[i] = a[i] * d;
    }
    return sTmpVec;
}


/*******************************************************************//**
PURPOSE: Compute the product of two vectors.

NOTES: vResult = a * b;
***********************************************************************/
inline SmVectorNd operator * (const SmVectorNd& a, const SmVectorNd& b)
{
    double dData[32];
    SmVectorNd sTmpVec(32,dData);
    if (a.GetSize() != b.GetSize()) {
        SE(SM_ERR);
    }
    ULONG lSize = smos_Min(a.GetSize(),b.GetSize());
    sTmpVec.SetSize(lSize);
    for (ULONG i=0; i<lSize; i++) {
        sTmpVec[i] = a[i] * b[i];
    }
    return sTmpVec;
}

/*******************************************************************//**
PURPOSE: Compute the division of two vectors.

NOTES: vResult = a / b;
***********************************************************************/
inline SmVectorNd operator / (const SmVectorNd& a, const SmVectorNd& b)
{
    double dData[32];
    SmVectorNd sTmpVec(32,dData);
    if (a.GetSize() != b.GetSize()) {
        SE(SM_ERR);
    }
    ULONG lSize = smos_Min(a.GetSize(),b.GetSize());
    sTmpVec.SetSize(lSize);
    for (ULONG i=0; i<lSize; i++) {
        if (smos_Fabs(b[i]) > SM_EFF_ZERO) {
            sTmpVec[i] = a[i] / b[i];
        }
        else {
            SE(SM_ERR);
            sTmpVec[i] = a[i];
        }
    }
    return sTmpVec;
}

/*******************************************************************//**
PURPOSE: Divide a vector by a constant.

NOTES: vResult = v / 2.0;
***********************************************************************/
inline SmVectorNd operator / (const SmVectorNd& a, const double d)
{
    double dData[32];
    SmVectorNd sTmpVec(32,dData);
    sTmpVec.SetSize(a.GetSize());
    for (ULONG i=0; i<sTmpVec.GetSize(); i++) {
        if (smos_Fabs(d) > SM_EFF_ZERO) {
            sTmpVec[i] = a[i] / d;
        }
        else {
            SE(SM_ERR);
            sTmpVec[i] = a[i];
        }
    }
    return sTmpVec;
}


// Non trivial inline methods

/*******************************************************************//**
PURPOSE: Constructor for the Vector.

NOTES: 
***********************************************************************/
inline SmVectorNd::SmVectorNd(ULONG lDataSize,
                              double *pData, 
                              ULONG lInitialSize)
  : m_vVec(lDataSize,pData,lInitialSize)
{
}


/*******************************************************************//**
PURPOSE: Dot product of two vectors.

NOTES: 
***********************************************************************/
inline double SmVectorNd::Dot(const SmVectorNd & crOther) const
{
    if (GetSize() != crOther.GetSize()) {
        SE(SM_ERR);
    }
    ULONG lSize = smos_Min(GetSize(),crOther.GetSize());
    double dRet = 0.0;
    for (ULONG i=0; i<lSize; i++) {
        dRet = dRet + (*this)[i] * crOther[i];
    }
    return dRet;
}

/*******************************************************************//**
PURPOSE: Compute the length of the vector.

NOTES: 
***********************************************************************/
inline double SmVectorNd::Length(void) const
{
  return smos_Sqrt(LengthSquared());
}

/*******************************************************************//**
PURPOSE: Compute the squared length of the vector.

NOTES: 
***********************************************************************/
inline double SmVectorNd::LengthSquared(void) const
{
    double dRet = 0.0;
    for (ULONG i=0; i<GetSize(); i++) {
        dRet = dRet + (*this)[i] * (*this)[i];
    }
    return dRet;
}

/*******************************************************************//**
PURPOSE: Return TRUE if distance between two points is less than or 
            equal to a given distance (dMax), else FALSE

NOTES: 
***********************************************************************/
inline SmBoolean SmVectorNd::CloserThan(double dMax, const SmVectorNd & crOther) const 
{ 
  SmVectorNd Diff = crOther - *this;
  if (Diff.LengthSquared() > dMax*dMax)
      return (FALSE);
  return TRUE;
}


/*******************************************************************//**
PURPOSE: Unitize a vector.  Return an error if the vector is a zero
    vector.

NOTES: SER(v.Unitize());
***********************************************************************/
inline SmStatus SmVectorNd::Unitize(void) 
{
    double dLenSq = LengthSquared();
    SmStatus sRet = SM_SUCCESS;
    if (dLenSq < SM_EFF_ZERO_SQ) { 
        SE(SM_ERR_INVALID_INPUT); 
        sRet = SM_ERR_INVALID_INPUT; 
    }
    else if (smos_Fabs(dLenSq-1.0) < SM_EFF_ZERO) {
        sRet = SM_SUCCESS;
    }
    else { 
        double dLen = smos_Sqrt(dLenSq); 
        *this = *this / dLen;
    }
    return sRet;
}


/*******************************************************************//**
PURPOSE: Compute the distance between two points.

NOTES: 
***********************************************************************/
inline double SmVectorNd::DistanceBetween(const SmVectorNd & crOther) const 
{
    double sData[32];
    SmVectorNd sBetween(32,sData);
    sBetween = *this - crOther;
    return sBetween.Length(); 
}

/*******************************************************************//**
PURPOSE: Compute the squared distance between two points.

NOTES: 
***********************************************************************/
inline double SmVectorNd::DistanceBetweenSquared(const SmVectorNd & crOther) const 
{
    double sData[32];
    SmVectorNd sBetween(32,sData);
    sBetween = *this - crOther;
    return sBetween.LengthSquared(); 
}


/*******************************************************************//**
PURPOSE: Swap the values of these two vectors.

NOTES: 
***********************************************************************/
inline void SmVectorNd::Swap(SmVectorNd & crOtherVector)
{
    double sData[32];
    SmVectorNd sVTmp(32,sData);
    sVTmp = crOtherVector;
    crOtherVector = *this;
    *this = sVTmp;
}

#endif // !__SMVECTORND_H__


