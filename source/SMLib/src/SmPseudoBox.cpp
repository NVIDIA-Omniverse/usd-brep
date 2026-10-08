// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************//**
* FILE NAME --- SmPseudoBox.cpp
* PURPOSE: Implementation of methods for the PseudoBox.  The
*  Pseudo Box defines a volume in space by 6 bounding planes.  It
*  does this by establishing intervals along 3 direction vectors 
*  in which the object being bounded is contained.  
**********************************************************************/

#include "StdAfx.h"

#include <SmPseudoBox.h>
#include <SmGeomUtility.h>
#include <SmGraphicsExtern.h>
#include <SmAssertArray.h>

/*******************************************************************//**
PURPOSE: Default constructor which initializes the pseudo box to
    the default coordinate system.

NOTES: basis = [100 010 001]
       m_bOrthogonalBasis = TRUE
       each interval set to [SM_BIG_DOUBLE, -SM_BIG_DOUBLE]
***********************************************************************/
SmPseudoBox::SmPseudoBox()
{ 
  InitBasis() ;
  Init() ;

} // end SmPseudoBox::SmPseudoBox constructor

/*******************************************************************//**
PURPOSE: Copy constructor.

NOTES: 
***********************************************************************/
SmPseudoBox::SmPseudoBox
  (const SmPseudoBox & crOriginal)
{
  m_bOrthogonalBasis = crOriginal.m_bOrthogonalBasis;
  m_aBasis[0]        = crOriginal.m_aBasis[0];
  m_aBasis[1]        = crOriginal.m_aBasis[1];
  m_aBasis[2]        = crOriginal.m_aBasis[2];
  m_aIntervals[0]    = crOriginal.m_aIntervals[0];
  m_aIntervals[1]    = crOriginal.m_aIntervals[1];
  m_aIntervals[2]    = crOriginal.m_aIntervals[2];

} // end SmPseudoBox::SmPseudoBox copy constructor

/*******************************************************************//**
PURPOSE: Copy SmExtent3d constructor.

NOTES: 
***********************************************************************/
SmPseudoBox::SmPseudoBox
  (const SmExtent3d & crBBox)
{
  m_bOrthogonalBasis = TRUE ;
  m_aBasis[0].Set(1,0,0) ;
  m_aBasis[1].Set(0,1,0) ;
  m_aBasis[2].Set(0,0,1) ;
  m_aIntervals[0].SetMinMax(crBBox.GetUMin(), crBBox.GetUMax()) ;
  m_aIntervals[1].SetMinMax(crBBox.GetVMin(), crBBox.GetVMax()) ;
  m_aIntervals[2].SetMinMax(crBBox.GetWMin(), crBBox.GetWMax()) ;

} // end SmPseudoBox::SmPseudoBox copy constructor

/*******************************************************************//***
PURPOSE: Assignment operator from Other PseudoBox

NOTES: 
************************************************************************/
SmPseudoBox &SmPseudoBox::operator=
  ( const SmPseudoBox &crOther )
{
  // no work condition
  if(&crOther == this) return *this ;

  // copy members
  m_bOrthogonalBasis = crOther.m_bOrthogonalBasis;

  m_aBasis[0]        = crOther.m_aBasis[0] ;   
  m_aBasis[1]        = crOther.m_aBasis[1] ;   
  m_aBasis[2]        = crOther.m_aBasis[2] ;   
                                              
  m_aIntervals[0]    = crOther.m_aIntervals[0] ;
  m_aIntervals[1]    = crOther.m_aIntervals[1] ;
  m_aIntervals[2]    = crOther.m_aIntervals[2] ;
  
  // all done
  return *this;

} // end SmPseudoBox::operator=

/*******************************************************************//***
PURPOSE: Assignment operator from Other Extent3d

NOTES: 
************************************************************************/
SmPseudoBox &SmPseudoBox::operator=
  ( const SmExtent3d &crOther )
{
  m_bOrthogonalBasis = TRUE ;
  m_aBasis[0].Set(1,0,0) ;
  m_aBasis[1].Set(0,1,0) ;
  m_aBasis[2].Set(0,0,1) ;
  m_aIntervals[0].SetMinMax(crOther.GetUMin(), crOther.GetUMax()) ;
  m_aIntervals[1].SetMinMax(crOther.GetVMin(), crOther.GetVMax()) ;
  m_aIntervals[2].SetMinMax(crOther.GetWMin(), crOther.GetWMax()) ;
  
  // all done
  return *this;

} // end SmPseudoBox::operator=

/*******************************************************************//**
PURPOSE: Equality operator 

NOTES: 
***********************************************************************/
SmBoolean SmPseudoBox::operator==
  (const SmPseudoBox& crOther) 
 const
{
  // quick check 
  if(this == &crOther) { return TRUE ; }

  // check member equality
  if(   ( m_bOrthogonalBasis == crOther.m_bOrthogonalBasis)
     && ( m_aBasis[0]        == crOther.m_aBasis[0])
     && ( m_aBasis[1]        == crOther.m_aBasis[1])
     && ( m_aBasis[2]        == crOther.m_aBasis[2])
     && ( m_aIntervals[0]    == crOther.m_aIntervals[0])  
     && ( m_aIntervals[1]    == crOther.m_aIntervals[1])  
     && ( m_aIntervals[2]    == crOther.m_aIntervals[2]) )
   { return TRUE ; }
 return FALSE ;                                          

} // end SmPseudoBox::operator==

/*******************************************************************//**
PURPOSE: Initialize Pseudo Box so that next Add() call will
            center the pseudo on that point. 

NOTES: This is done by initializing each interval to 
       its uninitialized state:[SM_BIG_DOUBLE, -SM_BIG_DOUBLE]
***********************************************************************/
void SmPseudoBox::Init()
{ 
  // set each interval to [SM_BIG_DOUBLE, -SM_BIG_DOUBLE]
   m_aIntervals[0].Init() ;
   m_aIntervals[1].Init() ;
   m_aIntervals[2].Init() ;

} // end SmPseudoBox::Init

/*******************************************************************//**
PURPOSE: Initialize basis vectors to (100 010 001) 

NOTES: This is done by initializing each interval
***********************************************************************/
void SmPseudoBox::InitBasis()
{ 
  m_bOrthogonalBasis = TRUE;
  
  m_aBasis[0].Set(1, 0, 0) ;
  m_aBasis[1].Set(0, 1, 0) ;
  m_aBasis[2].Set(0, 0, 1) ;

} // end SmPseudoBox::InitBasis

/*******************************************************************//**
PURPOSE: Set the minimum and maximum points of this extent.

NOTES: 
***********************************************************************/
SmStatus SmPseudoBox::SetMinMax
  (double dMinU,
   double dMinV,
   double dMinW,
   double dMaxU,
   double dMaxV,
   double dMaxW) 
{ 
  SmStatus sRet0 =  m_aIntervals[0].SetMinMax(dMinU, dMaxU) ;
  SmStatus sRet1 =  m_aIntervals[1].SetMinMax(dMinV, dMaxV) ;
  SmStatus sRet2 =  m_aIntervals[2].SetMinMax(dMinW, dMaxW) ;

  return((   sRet0 == SM_SUCCESS
          && sRet1 == SM_SUCCESS
          && sRet2 == SM_SUCCESS) ? SM_SUCCESS : SM_ERR) ;

} // end SmPseudoBox::SetMinMax

/*******************************************************************//**
PURPOSE: Set the minimum U-value of the 3D extent.

NOTES: Return SM_ERR if dNewUMin > the max U value.
***********************************************************************/
SmStatus SmPseudoBox::SetUMin( double dNewUMin )
{
  if ( dNewUMin > m_aIntervals[0].GetMin() ) SER(SM_ERR_INVALID_INPUT);
  m_aIntervals[0].SetMin(dNewUMin) ;
  return SM_SUCCESS;
}
/*******************************************************************//**
PURPOSE: Set the maximum U-value of the 3D extent.

NOTES: Return SM_ERR if dNewUMax < the min U value.
***********************************************************************/
SmStatus SmPseudoBox::SetUMax( double dNewUMax )
{
  if ( dNewUMax > m_aIntervals[0].GetMax() ) SER(SM_ERR_INVALID_INPUT);
  m_aIntervals[0].SetMax(dNewUMax) ;
  return SM_SUCCESS;
}

/*******************************************************************//**
PURPOSE: Set the minimum V-value of the 3D extent.

NOTES: Return SM_ERR if dNewVMin > the max V value.
***********************************************************************/
SmStatus SmPseudoBox::SetVMin( double dNewVMin )
{
  if ( dNewVMin > m_aIntervals[0].GetMin() ) SER(SM_ERR_INVALID_INPUT);
  m_aIntervals[0].SetMin(dNewVMin) ;
  return SM_SUCCESS;
}
/*******************************************************************//**
PURPOSE: Set the maximum V-value of the 3D extent.

NOTES: Return SM_ERR if dNewVMax < the min V value.
***********************************************************************/
SmStatus SmPseudoBox::SetVMax( double dNewVMax )
{
  if ( dNewVMax > m_aIntervals[0].GetMax() ) SER(SM_ERR_INVALID_INPUT);
  m_aIntervals[0].SetMax(dNewVMax) ;
  return SM_SUCCESS;
}

/*******************************************************************//**
PURPOSE: Set the minimum W-value of the 3D extent.

NOTES: Return SM_ERR if dNewWMin > the max W value.
***********************************************************************/
SmStatus SmPseudoBox::SetWMin( double dNewWMin )
{
  if ( dNewWMin > m_aIntervals[0].GetMin() ) SER(SM_ERR_INVALID_INPUT);
  m_aIntervals[0].SetMin(dNewWMin) ;
  return SM_SUCCESS;
}
/*******************************************************************//**
PURPOSE: Set the maximum W-value of the 3D extent.

NOTES: Return SM_ERR if dNewWMax < the min W value.
***********************************************************************/
SmStatus SmPseudoBox::SetWMax( double dNewWMax )
{
  if ( dNewWMax > m_aIntervals[0].GetMax() ) SER(SM_ERR_INVALID_INPUT);
  m_aIntervals[0].SetMax(dNewWMax) ;
  return SM_SUCCESS;
}

/*******************************************************************//**
PURPOSE: Add a point to the pseudo box and expand pseudo box if 
    necessary.

NOTES: 
***********************************************************************/
void SmPseudoBox::AddPoint3d
  (const SmPoint3d & crPoint)
{ 
  SmPoint3d sA ;

  // map crPoint to PseudoBox absolute parameters
  InvertAbsolute(crPoint, sA) ;

  // add each PseudoBox parameter to the intervals
  m_aIntervals[0].AddValue(sA.x);
  m_aIntervals[1].AddValue(sA.y);
  m_aIntervals[2].AddValue(sA.z);

} // end SmPseudoBox::AddPoint3d

/*******************************************************************//**
PURPOSE: Expand the pseudo box in all directions by the given value.

NOTES: 
***********************************************************************/
void SmPseudoBox::ExpandAbsolute
  (double dExpansion)
{
  SM_ASSERT(dExpansion >= 0.0);
  m_aIntervals[0].ExpandAbsolute(dExpansion);
  m_aIntervals[1].ExpandAbsolute(dExpansion);
  m_aIntervals[2].ExpandAbsolute(dExpansion);

} // end SmPseudoBox::ExpandAbsolute

/*******************************************************************//**
PURPOSE: Expand the pseudo box by sweeping it by the given vector.

NOTES: 
***********************************************************************/
void SmPseudoBox::ExpandSweep
  (const SmVector3d &rSweepVector)
{
  SmPoint3d sA ;

  // map crPoint to PseudoBox absolute parameters
  InvertAbsolute(rSweepVector, sA) ;

  //double dSweepInc = rSweepVector.Dot(m_aBasis[0]) ;
  if(sA.x > 0) m_aIntervals[0].SetMinMax(m_aIntervals[0].GetMin(),        m_aIntervals[0].GetMax() + sA.x) ;
  else         m_aIntervals[0].SetMinMax(m_aIntervals[0].GetMin() + sA.x, m_aIntervals[0].GetMax()) ;

  if(sA.y > 0) m_aIntervals[1].SetMinMax(m_aIntervals[1].GetMin(),        m_aIntervals[1].GetMax() + sA.y) ;
  else         m_aIntervals[1].SetMinMax(m_aIntervals[1].GetMin() + sA.y, m_aIntervals[1].GetMax()) ;

  if(sA.z > 0) m_aIntervals[2].SetMinMax(m_aIntervals[2].GetMin(),        m_aIntervals[2].GetMax() + sA.z) ;
  else         m_aIntervals[2].SetMinMax(m_aIntervals[2].GetMin() + sA.z, m_aIntervals[2].GetMax()) ;

} // end SmPseudoBox::ExpandSweep

/*******************************************************************//**
PURPOSE: Determine if the pseudo box contains the given point to within
   a tolerance.

NOTES: 
***********************************************************************/
SmBoolean SmPseudoBox::ContainsPoint3d
  (const SmPoint3d & crPoint, 
   double            d3dTolerance) 
  const
{
  SmPoint3d sA ;

  // map crPoint to PseudoBox normalized parameters
  InvertAbsolute(crPoint, sA) ;

  // check that all parameters are within tol of their intervals
  SmBoolean bRtn =    m_aIntervals[0].ContainsValue(sA.x, d3dTolerance) 
                   && m_aIntervals[1].ContainsValue(sA.y, d3dTolerance)  
                   && m_aIntervals[2].ContainsValue(sA.z, d3dTolerance) ;

  // all done
  return bRtn;

} // end SmPseudoBox::ContainsPoint3d

/*******************************************************************//**
PURPOSE: Do a normalized evaluation to produce a point in the
    extent.  

NOTES: For example: (0,0,0) produces the minimum point;
    (1,1,1) produces the maximum point; (0.5,0.5,0.5) produces the
    point in the center of the extent.
***********************************************************************/
SmPoint3d SmPseudoBox::Evaluate
  (double dNormalizedX,          // in : 
   double dNormalizedY,          // in : 
   double dNormalizedZ)          // in : 
  const 
{
  SM_ASSERT_DEFINED(&m_aIntervals[0]) ; 
  SM_ASSERT_DEFINED(&m_aIntervals[1]) ; 
  SM_ASSERT_DEFINED(&m_aIntervals[2]) ; 

  SM_ASSERT_BREAK(-SM_EFF_ZERO <= dNormalizedX && dNormalizedX <= 1.0+SM_EFF_ZERO);
  SM_ASSERT_BREAK(-SM_EFF_ZERO <= dNormalizedY && dNormalizedY <= 1.0+SM_EFF_ZERO);
  SM_ASSERT_BREAK(-SM_EFF_ZERO <= dNormalizedZ && dNormalizedZ <= 1.0+SM_EFF_ZERO);

  SmPoint3d sPoint =   m_aBasis[0] * m_aIntervals[0].Evaluate(dNormalizedX)
                     + m_aBasis[1] * m_aIntervals[1].Evaluate(dNormalizedY)
                     + m_aBasis[2] * m_aIntervals[2].Evaluate(dNormalizedZ);
  return sPoint;

} // end SmPseudoBox::Evaluate

/*******************************************************************//**
PURPOSE: Find the normalized parameters corresponding to this point.

NOTES: normalized parameters are defined as

  3DPoint =   m_aIntervals[0].Evaluate(NormalizedParameters[0]) * m_aBasis[0]
            + m_aIntervals[1].Evaluate(NormalizedParameters[1]) * m_aBasis[1]
            + m_aIntervals[2].Evaluate(NormalizedParameters[2]) * m_aBasis[2]

  The input cr3DPoint is inside the PseudoBox when all 3 returned
  normalized parameters are in the range 0.0 <= NormalizedParam <= 1.0.

RETURNS: SM_SUCCESS and valid  rNormalizedParameters when point is in PseudoBox and
         SM_ERR     and valid  rNormalizedParameters when point is out of PseudoBox 
         SM_ERR     and valid  rNormalizedParameters when inversion is not equivalent due
                                 zero length PseudoBox intervals
         SM_ERR     and uninit rNormalizedParameters when inversion fails 
                                due to undefined PseudoBox or singular basis vector set.
***********************************************************************/
SmStatus SmPseudoBox::Invert
  (const SmPoint3d & cr3DPoint,
   SmPoint3d       & rNormalizedParameters) 
  const
{
  // check input
  if(!AssertDefined()) return(SM_ERR) ;

  // init output
  rNormalizedParameters.SetUninitialized() ;

  // locals    
  SmPoint3d   sA ; 
  SmPoint3d & rP = rNormalizedParameters ;

  // map 3D point to absolute coordinates
  SER(InvertAbsolute(cr3DPoint, sA)) ;

  // map absolute coordinates to normalized coordinates
  //  returns: SM_ERR when conversion is not equivaent due to a zero length interval
  SmStatus sRtn = ConvertAbsoluteToNormal(sA, rP) ;

#ifdef SM_DEBUG_CODE
  // test result
  SmPoint3d sTestPoint  = Evaluate(rP.x, rP.y, rP.z) ;
  double    dScaledZero = SM_EFF_ZERO * (1.0 + cr3DPoint.GetMaxDimension()) ;
    if( !cr3DPoint.CloserThan(dScaledZero, sTestPoint) 
      && sRtn == SM_SUCCESS)
    {
      SM_ASSERT(   (cr3DPoint.CloserThan(dScaledZero, sTestPoint))
                || (sRtn == SM_ERR)) ; 
    }
#endif // SM_DEBUG_CODE

  // set output - switch SM_SUCCESS to SM_ERR when cr3DPoint is out of the PseudoBox
  if(   rP.x < -SM_EFF_ZERO || rP.x > 1.0 + SM_EFF_ZERO
     || rP.y < -SM_EFF_ZERO || rP.y > 1.0 + SM_EFF_ZERO
     || rP.z < -SM_EFF_ZERO || rP.z > 1.0 + SM_EFF_ZERO)
    { sRtn = SM_ERR ; }

  // all done
  return( sRtn ) ;

} // end SmPseudoBox::Invert

/*******************************************************************//**
PURPOSE: Do a absolute evaluation to produce a point from the PseudoBox Coordinate system. 

NOTES: 3DPoint =   AbsoluteParameters[0] * m_aBasis[0]
                 + AbsoluteParameters[1] * m_aBasis[1]
                 + AbsoluteParameters[2] * m_aBasis[2]
***********************************************************************/
SmPoint3d SmPseudoBox::EvaluateAbsolute
  (double dAbsoluteX,          // in : 
   double dAbsoluteY,          // in : 
   double dAbsoluteZ)          // in : 
  const
{
  SmPoint3d sPoint =   dAbsoluteX * m_aBasis[0]
                     + dAbsoluteY * m_aBasis[1]
                     + dAbsoluteZ * m_aBasis[2] ;
  return sPoint;

} // end SmPseudoBox::EvaluateAbsolute

/*******************************************************************//**
PURPOSE: Find the absolute parameters corresponding to this 3D point.

NOTES: absolute parameters are defined as

  3DPoint =   AbsoluteParameters[0] * m_aBasis[0]
            + AbsoluteParameters[1] * m_aBasis[1]
            + AbsoluteParameters[2] * m_aBasis[2] 

  The input cr3DPoint is inside the PseudoBox when all 3 returned
  absolute parameters are within their respective intervals as
  m_aIntervals[i].GetMin() <= AbsoluteParameters[i] <= m_aIntervals[i].GetMax()

RETURNS: SM_ERR when basis functions are singular, i.e. they don't span 3 space,
         else returns SM_SUCCESS
***********************************************************************/
SmStatus SmPseudoBox::InvertAbsolute
  (const SmPoint3d & cr3DPoint,
   SmPoint3d       & rAbsoluteParameters) 
  const
{
  // check input
  if(!AssertDefined()) return(SM_ERR) ;
  
  // locals     
  const SmPoint3d  & rX    = cr3DPoint ;
  const SmVector3d & rB0   = m_aBasis[0] ;
  const SmVector3d & rB1   = m_aBasis[1] ;
  const SmVector3d & rB2   = m_aBasis[2] ;
  SmPoint3d        & sA    = rAbsoluteParameters ;

  // determinate
  double det = smgu_Determinant3Vectors(rB0, rB1, rB2) ;

  // watch out for singular basis vector sets
  SER( SM_IS_ZERO(det) == FALSE ? SM_SUCCESS : SM_ERR) ;

  // hard code Kramer's rule solutions for Matrix*sA=rX
  // with columns of matrix = [ rB0 rB1 rB2 ], solutions are
  // P0 = |rX  rB1 rB2| / |rB0 rB1 rB2|
  // P1 = |rB0 rX  rB2| / |rB0 rB1 rB2|
  // P2 = |rB0 rB1 rX | / |rB0 rB1 rB2|

  // solve for absolute PseudoBox coordinates
  sA.x = smgu_Determinant3Vectors(rX, rB1, rB2) / det ;
  sA.y = smgu_Determinant3Vectors(rB0, rX, rB2) / det ;
  sA.z = smgu_Determinant3Vectors(rB0, rB1, rX) / det ;

#ifdef SM_DEBUG_CODE
  // test result
  SmPoint3d sTestPoint  = EvaluateAbsolute(sA.x, sA.y, sA.z) ;
  double    dScaledZero = SM_EFF_ZERO * (1.0 + cr3DPoint.GetMaxDimension()) ;
  if( !cr3DPoint.CloserThan(dScaledZero, sTestPoint) )
    {
      SM_ASSERT(   (cr3DPoint.CloserThan(dScaledZero, sTestPoint))
                || (   m_aIntervals[0].GetLength() < SM_EFF_ZERO
                    || m_aIntervals[1].GetLength() < SM_EFF_ZERO
                    || m_aIntervals[2].GetLength() < SM_EFF_ZERO)) ; 
    }
#endif // SM_DEBUG_CODE

  // all done
  return(SM_SUCCESS) ;
                                                           
} // end SmPseudoBox::InvertAbsolute

/*******************************************************************//**
PURPOSE: Convert a Point in PseudoBox Absolute coordinates to 
         PseudoBox Normalized coordinates  

NOTES: 
  Each absolute coordinate is mapped to its coresponding interval. 
       
  If an interval is zero length, 
    and if the absolute parameter happens to be that zero length value, 
           then the normalized parameter is arbitrariy set 
                to 0.0 (any value from 0.0 to 1.0 would be valid),
        else the normalized parameter is set to -1.0.

RETURNS: SM_SUCCESS when mapping succeeds,
         SM_ERR when PseudoBox is undefined.
         SM_ERR when conversion is not equivalent due to a zero length interval.
***********************************************************************/
SmStatus SmPseudoBox::ConvertAbsoluteToNormal
 (const SmPoint3d & rAbsoluteParameters,    // in :       
  SmPoint3d       & rNormalizedParameters)  // out: 
 const 
{
  // check input
  if(!AssertDefined()) return(SM_ERR) ;
  
  // locals 
  SmStatus           sRtn      = SM_SUCCESS ;
  const SmPoint3d  & sA        = rAbsoluteParameters ;
  SmPoint3d        & sP        = rNormalizedParameters ;

  // map absolute PseudoBox absolute coordinates to absolute coordinates
  if     (m_aIntervals[0].GetLength() > SM_EFF_ZERO)        { m_aIntervals[0].Inversion(sA.x, sP.x) ; }
  else if(m_aIntervals[0].ContainsValue(sA.x, SM_EFF_ZERO)) { sP.x =  0.0 ; } 
  else                                                      { sP.x = -1.0 ; sRtn = SM_ERR ; }
                                                           
  if     (m_aIntervals[1].GetLength() > SM_EFF_ZERO)        { m_aIntervals[1].Inversion(sA.y, sP.y) ; }
  else if(m_aIntervals[1].ContainsValue(sA.y, SM_EFF_ZERO)) { sP.y =  0.0 ; } 
  else                                                      { sP.y = -1.0 ; sRtn = SM_ERR ; }
                                                           
  if     (m_aIntervals[2].GetLength() > SM_EFF_ZERO)        { m_aIntervals[2].Inversion(sA.z, sP.z) ; }
  else if(m_aIntervals[2].ContainsValue(sA.z, SM_EFF_ZERO)) { sP.z =  0.0 ; } 
  else                                                      { sP.z = -1.0 ; sRtn = SM_ERR ; }
                                                           
#ifdef SM_DEBUG_CODE
  // test result
  SmPoint3d sTestAbsolute   = EvaluateAbsolute(sA.x, sA.y, sA.z) ;
  SmPoint3d sTestNormalized = Evaluate(sP.x, sP.y, sP.z) ;
  double    dScaledZero     = SM_EFF_ZERO * (1.0 + sTestAbsolute.GetMaxDimension()) ;
  if(   !sTestAbsolute.CloserThan(dScaledZero, sTestNormalized) 
     &&  sRtn == SM_SUCCESS)
    {
      SM_ASSERT(   (sTestAbsolute.CloserThan(dScaledZero, sTestNormalized))
                || (sRtn == SM_ERR)) ; 
    }
#endif // SM_DEBUG_CODE

  // all done 
  return( sRtn ) ;

} // end SmPseudoBox::ConvertAbsoluteToNormal

// /*******************************************************************//**
// PURPOSE: compute a ScaledZero value based on largest dimension of this pseudobox
// 
// NOTES:
// ***********************************************************************/
// double SmPseudoBox::GetScaledZero
//   (SmPseudoBox * pOpt2ndBox)   // in : 2nd extent when working with two extents
//                                //      NULL to ignore, default:[NULL]
//  const
// {
//   double dScaledZero1 = SmTol::ScaledZero(*this) ;
//   if( pOpt2ndBox )
//     {
//       double dScaledZero2 = SmTol::ScaledZero(*pOpt2ndBox) ;
//       if(dScaledZero2 > dScaledZero1) 
//         { dScaledZero1 = dScaledZero2 ; }
//     }
//   return(dScaledZero1) ;
// 
// }  // end SmExtent3d::GetScaledZero

/*******************************************************************//**
PURPOSE: Union the extents of two polar boxes using the input basis vectors.

NOTES: 
***********************************************************************/
SmStatus SmPseudoBox::Union
  (const SmPseudoBox & crOther,      // in :
   const SmVector3d  * aBasis,       // in : sized:[3]
   SmPseudoBox       & rResult)      // out:
 const
{
  // Get input boxes' corner points
  SmPoint3d sPoints[16] ;
  this->CalcCorners(sPoints) ;
  crOther.CalcCorners(&sPoints[8]) ;

  // set Basis
  rResult.SetBasis(aBasis[0], aBasis[1], aBasis[2]) ;

  // Add in all corner points
  for(ULONG ii=0;ii<16;ii++)
    {
      rResult.AddPoint3d(sPoints[ii]) ;
    }

  return SM_SUCCESS;

} // end SmPseudoBox::Union

/*******************************************************************//**
PURPOSE: Set the basis vectors of the pseudo box.  Note that the three
   vectors should not all lie within a plane or on a line.

NOTES: 

RETURNS: SM_ERR_INVALID_INPUT and makes no changes 
            when input basis vectors are not linearly independent and
            when input basis vectors are not unitized
***********************************************************************/
SmStatus SmPseudoBox::SetBasis
  (const SmVector3d & rBasis1,  // in : unitized vector0 
   const SmVector3d & rBasis2,  // in : unitized vector1
   const SmVector3d & rBasis3)  // in : unitized vector2
{
  // check input: Basis vectors must be linearly independent
  double dDet = smgu_Determinant3Vectors(rBasis1,rBasis2,rBasis3);
  if (smos_Fabs(dDet) < SM_EFF_ZERO) 
    { SER(SM_ERR_INVALID_INPUT) ; }

  // make the assignment
  m_aBasis[0] = rBasis1;
  m_aBasis[1] = rBasis2;
  m_aBasis[2] = rBasis3;

  // Basis vectors must be unitized 
  double dD1 = rBasis1.LengthSquared() ;
  double dD2 = rBasis2.LengthSquared() ;
  double dD3 = rBasis3.LengthSquared() ;

  // unitize the vectors that need it
  if(!SM_IS_ZERO(dD1 - 1.0)) { m_aBasis[0] /= smos_Sqrt(dD1) ; }
  if(!SM_IS_ZERO(dD2 - 1.0)) { m_aBasis[1] /= smos_Sqrt(dD2) ; }
  if(!SM_IS_ZERO(dD3 - 1.0)) { m_aBasis[2] /= smos_Sqrt(dD3) ; }

  // remember if basis vectors are orthogonal or not
  m_bOrthogonalBasis = (   smos_Fabs(m_aBasis[0].Dot(m_aBasis[1])) < SM_EFF_ZERO 
                        && smos_Fabs(m_aBasis[1].Dot(m_aBasis[2])) < SM_EFF_ZERO  
                        && smos_Fabs(m_aBasis[2].Dot(m_aBasis[0])) < SM_EFF_ZERO) ;

  // all done
  return SM_SUCCESS;

} // end SmPseudoBox::SetBasis

/*******************************************************************//**
PURPOSE: Get the basis vectors of this pseudo box.

NOTES: 
***********************************************************************/
void SmPseudoBox::GetBasis
  (SmVector3d & rBasis1,  // out: 
   SmVector3d & rBasis2,  // out: 
   SmVector3d & rBasis3)  // out: 
 const
{
  rBasis1 = m_aBasis[0] ;
  rBasis2 = m_aBasis[1] ;
  rBasis3 = m_aBasis[2] ;

} // end SmPseudoBox::GetBasis

/*******************************************************************//**
PURPOSE: Set the intervals of this pseudo box.

NOTES: 
***********************************************************************/
void SmPseudoBox::SetIntervals
  (const SmExtent1d & rInterval1,  // in : 
   const SmExtent1d & rInterval2,  // in : 
   const SmExtent1d & rInterval3)  // in : 
{
  m_aIntervals[0] = rInterval1;
  m_aIntervals[1] = rInterval2;
  m_aIntervals[2] = rInterval3;

} // end SmPseudoBox::SetIntervals

/*******************************************************************//**
PURPOSE: Set the intervals of this pseudo box to unbounded.

NOTES: 
***********************************************************************/
void SmPseudoBox::SetUnbounded()
{
  m_aIntervals[0].SetMinMax(-SM_INFINITE_PARAMETER, SM_INFINITE_PARAMETER) ; 
  m_aIntervals[1].SetMinMax(-SM_INFINITE_PARAMETER, SM_INFINITE_PARAMETER) ; 
  m_aIntervals[2].SetMinMax(-SM_INFINITE_PARAMETER, SM_INFINITE_PARAMETER) ; 

} // end SmPseudoBox::SetUnbounded

/*******************************************************************//**
PURPOSE: Get the intervals of this pseudo box.

NOTES: 
***********************************************************************/
void SmPseudoBox::GetIntervals
  (SmExtent1d  & rInterval1,  // out: 
   SmExtent1d  & rInterval2,  // out: 
   SmExtent1d  & rInterval3)  // out: 
 const
{
  rInterval1 = m_aIntervals[0];
  rInterval2 = m_aIntervals[1];
  rInterval3 = m_aIntervals[2];

} // end SmPseudoBox::GetIntervals

/*******************************************************************//**
PURPOSE: return a point whose coordinates are equal to the size of each
         respective interval.

NOTES: 
***********************************************************************/
SmPoint3d SmPseudoBox::GetIntervalSizes() 
 const
{
  SM_ASSERT_DEFINED(this) ; 
  return(SmPoint3d(m_aIntervals[0].GetLength(),
                   m_aIntervals[1].GetLength(),
                   m_aIntervals[2].GetLength())) ;

} // end SmPseudoBox::GetIntervalSizes

/*******************************************************************//**
PURPOSE: Determine the nearest distance from a point to the pseudo box.

NOTES:  returns 0.0 for points inside the box.
***********************************************************************/
double SmPseudoBox::DistanceToPoint
  (const SmPoint3d & rPoint) 
 const
{
  SmPoint3d  aPlPnts[6];
  SmVector3d aPlNorms[6];
  CalcPlanes(aPlPnts,aPlNorms);
  ULONG  alFoundIdx[6];
  double adDistances[6];

  ULONG lNumFound = 0; // number of planes which rPoint is on positive side of
  for (ULONG i=0; i<6; i++) 
    {
      // Calculate D of plane equation, A, B, C are in aVectors
      double dD = - aPlPnts[i].Dot(aPlNorms[i]);
      double dRes = aPlNorms[i].Dot(rPoint) + dD;

      // when point is outside the pseudo box plane
      if (dRes > SM_EFF_ZERO) 
        {
          // remeber the distance and increment the counter
          adDistances[lNumFound]  = dRes;
          alFoundIdx[lNumFound++] = i;
        }
    }

  // when point was inside all 6 planes
  if (lNumFound == 0) return 0.0;  // point inside pseudo box

  // when point was outside just 1 plane - the dist to that plane is the min distance
  if (lNumFound == 1) 
    { // point closest to plane of pseudo box
      return adDistances[0];
    }

  // when point was outside two planes - find distance to nearest pseudoBox edge (xsect of two planes)
  if (lNumFound == 2) 
    { // point closest to edge of pseudo box
      // Intersect planes and find distance to line
      SmPoint3d sLinePnt;
      SmVector3d sLineVec;
      smgu_IntersectTwoPlanes(aPlPnts[alFoundIdx[0]],aPlNorms[alFoundIdx[0]],
                              aPlPnts[alFoundIdx[1]],aPlNorms[alFoundIdx[1]],
                              sLinePnt, sLineVec);
      double dLineParam;
      smgu_LineClosestPoint(sLinePnt,sLineVec,rPoint,dLineParam);
      SmPoint3d sPntOnLine = sLinePnt + dLineParam * sLineVec;
      return sPntOnLine.DistanceBetween(rPoint);
    }

  // when point was outside three planes - find dist to nearest pseudoBox vertex (xsect of three planes)
  if (lNumFound == 3) 
    { // point closest to vertex of pseudo box
      SmPoint3d sVertexPnt;
      smgu_IntersectThreePlanes(aPlPnts[alFoundIdx[0]],aPlNorms[alFoundIdx[0]],
                                aPlPnts[alFoundIdx[1]],aPlNorms[alFoundIdx[1]],
                                aPlPnts[alFoundIdx[2]],aPlNorms[alFoundIdx[2]],
                                sVertexPnt);
      return sVertexPnt.DistanceBetween(rPoint);
    }

  // should never make it to here
  // a point should not be outside more than 3 pseudo box planes at one time
  SE_MSG(SM_ERR, _T("SmPseudoBox::DistanceToPoint classified a point outside of more than 3 planes - this is a bug"));
  return(SM_ERR) ;

} // end SmPseudoBox::DistanceToPoint

/*******************************************************************//**
PURPOSE: Compute the distance the Pseudo box is from a given plane.
    If the Pseudo Box intersects the plane the distance is zero.

NOTES: 
***********************************************************************/
double SmPseudoBox::DistanceToPlane
  (const SmPoint3d  & crPlanePoint,   // in : point on test plane
   const SmVector3d & crPlaneNormal)  // in : normal for test plane
  const
{
  // locals
  SmPoint3d sCorners[8];
  CalcCorners(sCorners);
  long lDirection = 0;

  // Compute D of plane equation
  double dPlaneD = - (crPlaneNormal.Dot(crPlanePoint));
  SmVector3d sPlaneABC(crPlaneNormal);
  SmBoolean bDoIntersect = FALSE;
  double dMinDist = SM_BIG_DOUBLE;

  // for every pseudoBox corner
  for (ULONG j=0; j<8; j++) 
    {
      // Test each point to see if on same side of plane by more than tolerance
      double dTest = sPlaneABC.Dot(sCorners[j]) + dPlaneD;

      // corner is below plane
      if (dTest < SM_EFF_ZERO) 
        {
          // done when corners span the plane
          if (lDirection == 1) { bDoIntersect = TRUE; break; }

          // else corners on the same plane side - remember the side and save min dist
          lDirection = -1;
          if (smos_Fabs(dTest) < dMinDist) 
            {
              dMinDist = smos_Fabs(dTest);
            }
        }

      // corner is above plane
      else if (dTest > SM_EFF_ZERO) 
        {
          // done when corners span the plane
          if (lDirection == -1) { bDoIntersect = TRUE; break; }
          
          // else corners on the same plane side - remember the side and save min dist
          if (smos_Fabs(dTest) < dMinDist) 
            {
              dMinDist = smos_Fabs(dTest);
            }
        }

      // done when corner is on plane
      else { bDoIntersect = TRUE; break; }
    }

  // set output and return
  if (bDoIntersect) dMinDist = 0.0;
  return dMinDist;

} // end SmPseudoBox::DistanceToPlane

/*******************************************************************//**
PURPOSE: Intersect a line and the pseudo bounding box.

NOTES: 
***********************************************************************/
SmStatus SmPseudoBox::IntersectLine
  (const SmPoint3d  & crLinePoint,     // in : point on line
   const SmVector3d & crLineVector,    // in : vector defining line's direction
   ULONG            & rlNumFound,      // out: number of intersections 
                                       //      0 = there is no intersection.  
                                       //      1 = grazes a corner.
                                       //      2 = portion of ray is inside box.
   double           & rdTEnter,        // out: Start XSect Interval Param
   double           & rdTExit)         // out: End XSect Interval Param
  const
{
  SmStatus sRet = SM_SUCCESS;
  
  // check input
  SM_ASSERT(   !HasNegativeVolume()
            && crLineVector.LengthSquared() >= SM_EFF_ZERO_SQ) ;
  if(   crLineVector.LengthSquared() < SM_EFF_ZERO_SQ
     || HasNegativeVolume()) 
    {
      return(SM_ERR_INVALID_INPUT) ;
    }

  // arrive here when input is valid

#ifdef SM_DEBUG_CODE
  SmBoolean bDebugMe = FALSE;
  if ( bDebugMe ) 
    {
      if ( FALSE )
        { smgfx_Erase(); }
      smgfx_SetLook( 1,2, 0,0,0 ); this -> Draw(NULL); sm_GraphicsLoop();
      smgfx_SetLook( 1,2, 0,1,1 ); ( 25*crLineVector).Draw(&crLinePoint); sm_GraphicsLoop();
      smgfx_SetLook( 1,2, 0,1,1 ); (-25*crLineVector).Draw(&crLinePoint); sm_GraphicsLoop();
      smgfx_SetLook( 1,4, 0,1,1 ); crLinePoint.Draw(); sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
#endif  // SM_DEBUG_CODE

  // get pseudobox plane normals
  SmVector3d sNormal[3] ;
  sNormal[0] = m_aBasis[1] * m_aBasis[2] ;
  sNormal[1] = m_aBasis[2] * m_aBasis[0] ;
  sNormal[2] = m_aBasis[0] * m_aBasis[1] ;

  // Get pseudobox plane min/max corner points
  SmPoint3d sMinPt =   m_aIntervals[0].GetMin() * m_aBasis[0]
                     + m_aIntervals[1].GetMin() * m_aBasis[1]
                     + m_aIntervals[2].GetMin() * m_aBasis[2] ;
  SmPoint3d sMaxPt =   m_aIntervals[0].GetMax() * m_aBasis[0]
                     + m_aIntervals[1].GetMax() * m_aBasis[1]
                     + m_aIntervals[2].GetMax() * m_aBasis[2] ;

  SmPoint3d sTMin, sTMax;
  double dNormalDotLineVector = sNormal[0].Dot(crLineVector) ;
  if(dNormalDotLineVector > SM_EFF_ZERO) 
    {
      sTMin.x = (sMinPt - crLinePoint).Dot(sNormal[0]) / dNormalDotLineVector;
      sTMax.x = (sMaxPt - crLinePoint).Dot(sNormal[0]) / dNormalDotLineVector;
    }
  else if(dNormalDotLineVector < -SM_EFF_ZERO) 
    {
      sTMax.x = (sMinPt - crLinePoint).Dot(sNormal[0]) / dNormalDotLineVector;
      sTMin.x = (sMaxPt - crLinePoint).Dot(sNormal[0]) / dNormalDotLineVector;
    }
  else 
    {
      sTMin.x = - SM_BIG_DOUBLE;
      sTMax.x =   SM_BIG_DOUBLE;
    }
  
  dNormalDotLineVector = sNormal[1].Dot(crLineVector) ;
  if(dNormalDotLineVector > SM_EFF_ZERO) 
    {
      sTMin.y = (sMinPt - crLinePoint).Dot(sNormal[1]) / dNormalDotLineVector;
      sTMax.y = (sMaxPt - crLinePoint).Dot(sNormal[1]) / dNormalDotLineVector;
    }
  else if(dNormalDotLineVector < -SM_EFF_ZERO) 
    {
      sTMax.y = (sMinPt - crLinePoint).Dot(sNormal[1]) / dNormalDotLineVector;
      sTMin.y = (sMaxPt - crLinePoint).Dot(sNormal[1]) / dNormalDotLineVector;
    }
  else 
    {
      sTMin.y = - SM_BIG_DOUBLE;
      sTMax.y =   SM_BIG_DOUBLE;
    }
  
  dNormalDotLineVector = sNormal[2].Dot(crLineVector) ;
  if(dNormalDotLineVector > SM_EFF_ZERO) 
    {
      sTMin.z = (sMinPt - crLinePoint).Dot(sNormal[2]) / dNormalDotLineVector;
      sTMax.z = (sMaxPt - crLinePoint).Dot(sNormal[2]) / dNormalDotLineVector;
    }
  else if(dNormalDotLineVector < -SM_EFF_ZERO) 
    {
      sTMax.z = (sMinPt - crLinePoint).Dot(sNormal[2]) / dNormalDotLineVector;
      sTMin.z = (sMaxPt - crLinePoint).Dot(sNormal[2]) / dNormalDotLineVector;
    }
  else 
    {
      sTMin.z = - SM_BIG_DOUBLE;
      sTMax.z =   SM_BIG_DOUBLE;
    }

  // the intersection interval is bounded by the Max(minParam) and Min(maxParam
  //  double dMin = smos_Max(sTMin.z,smos_Max(sTMin.x,sTMin.y));
  //  double dMax = smos_Min(sTMax.z,smos_Min(sTMax.x,sTMax.y));
  double dMin = smos_3Max(sTMin.z,sTMin.x,sTMin.y) ;
  double dMax = smos_3Min(sTMax.z,sTMax.x,sTMax.y) ;
  
  SmPoint3d sMid = crLinePoint + ((dMin+dMax)/2.0) * crLineVector;
  
  double dTol = SM_EFF_ZERO * (1.0 + sMid.GetMaxDimension());
  if (!ContainsPoint3d(sMid,dTol)) 
    {
      rlNumFound = 0;
    }
  else 
    {
      if (smos_Fabs(dMin-dMax) < SM_EFF_ZERO) 
        {
          rlNumFound = 1;

          // avg dT to make ivl exactly degenerate and never inverted
          rdTEnter = rdTExit  = (dMin + dMax) / 2.0 ;
        }
      else if (dMin < dMax) 
        {
          rlNumFound = 2;
          rdTEnter = dMin;
          rdTExit  = dMax;
        }
      else 
        {
          rlNumFound = 0;
        }
    }

  return sRet;

} // end SmPseudoBox::IntersectLine 

/*******************************************************************//**
PURPOSE: Return TRUE when any dimension in Extent dimension span is negative.   

NOTES: 
***********************************************************************/
SmBoolean SmPseudoBox::HasNegativeVolume
  () 
 const
{
  return(   m_aIntervals[0].HasNegativeLength()
         || m_aIntervals[1].HasNegativeLength()
         || m_aIntervals[2].HasNegativeLength()) ;

} // end SmPseudoBox::HasNegativeVolume

/*******************************************************************//**
PURPOSE: Return longest PseudoBox interval length

NOTES:
***********************************************************************/
double SmPseudoBox::GetMaxDimension() const  
{
  double dDist1 = m_aIntervals[0].GetLength() ;
  double dDist2 = m_aIntervals[1].GetLength() ;
  double dDist3 = m_aIntervals[2].GetLength() ;
  return(smos_3Max(dDist1, dDist2, dDist3)) ;

} // end SmPseudoBox::GetMaxDimension

/*******************************************************************//**
PURPOSE: get area of pseudoBox XY parallelagrm

NOTES: Volume = triple product of sized basis vectors
***********************************************************************/
double SmPseudoBox::GetXYArea() const  
{
  double dD01  =   m_aIntervals[0].GetLength()
                 * m_aIntervals[1].GetLength() ;
              
  double dArea = (m_aBasis[0] * m_aBasis[1]).Length() ;
                   
  return(dD01 * dArea) ;

} // end SmPseudoBox::GetXYArea

/*******************************************************************//**
PURPOSE: get volume of pseudo box

NOTES: Volume = triple product of sized basis vectors
***********************************************************************/
double SmPseudoBox::GetVolume() const  
{
  double dD012 =   m_aIntervals[0].GetLength()
                 * m_aIntervals[1].GetLength()
                 * m_aIntervals[2].GetLength() ;
              
  double dVolume = (  m_aBasis[0].x * (m_aBasis[1].y * m_aBasis[2].z - m_aBasis[1].z * m_aBasis[2].y )
                    + m_aBasis[0].y * (m_aBasis[1].z * m_aBasis[2].x - m_aBasis[1].x * m_aBasis[2].z )
                    + m_aBasis[0].z * (m_aBasis[1].x * m_aBasis[2].y - m_aBasis[1].y * m_aBasis[2].x )) ;
                   
  return(dD012 * dVolume) ;

} // end SmPseudoBox::GetVolume

/*******************************************************************//**
PURPOSE: Check pseudoBox for PointSized

NOTES: returns true for pointsized psedoboxes
***********************************************************************/
SmBoolean SmPseudoBox::IsPointSized           
  (double      dTol3d,               // in : max allowed deviation from Plane
   SmPoint3d  *pOptCenterPoint,      // out: PseudoBox center Point
   double     *pOptActualDeviation)  // out: Max ControlPoint/Plane distance seen
 const 
{ 
  // test for planarity on Surface PseudoBox
  double dDist0      = m_aIntervals[0].GetLength() ;
  double dDist1      = m_aIntervals[1].GetLength() ;
  double dDist2      = m_aIntervals[2].GetLength() ;
  double dScaledZero =   dTol3d == SM_EFF_ZERO
                       ? SM_EFF_ZERO * (1.0 + GetMaxDimension())  
                       : dTol3d ;

  // set outputs
  if(pOptActualDeviation) *pOptActualDeviation = smos_3Max(dDist0,dDist1,dDist2)/2.0 ;
  if(pOptCenterPoint)     *pOptCenterPoint     = Evaluate(.5,.5,.5) ;

  // check for pointsized
  if(   dDist0/2.0 < dScaledZero
     && dDist1/2.0 < dScaledZero
     && dDist2/2.0 < dScaledZero) { return(TRUE)  ; }
  else                            { return(FALSE) ; }

} // end SmPseudoBox::IsPointSized

/*******************************************************************//**
PURPOSE: Check pseudoBox for Linearity

NOTES: returns true for pointsized and linear pseudo boxes
***********************************************************************/
SmBoolean SmPseudoBox::IsLinear               
  (double      dTol3d,               // in : max allowed deviation from Plane
   SmPoint3d  *pOptLinePoint,        // out: Point on Line
   SmVector3d *pOptLineTangent,      // out: Line Unit Tangent
   double     *pOptActualDeviation)  // out: Max ControlPoint/Plane distance seen
 const 
{ 
  // test for planarity on Surface PseudoBox
  double dDist1      = m_aIntervals[1].GetLength() ;
  double dDist2      = m_aIntervals[2].GetLength() ;
  double dScaledZero =   dTol3d == SM_EFF_ZERO
                       ? SM_EFF_ZERO * (1.0 + GetMaxDimension())  
                       : dTol3d ;

  // set outputs
  if(pOptActualDeviation) *pOptActualDeviation = smos_Max(dDist1,dDist2)/2.0 ;
  if(pOptLinePoint)       *pOptLinePoint       = Evaluate(.5,.5,.5) ;
  if(pOptLineTangent)     *pOptLineTangent     = m_aBasis[0] ;

  // check for linearity
  if(   dDist1/2.0 < dScaledZero
     && dDist2/2.0 < dScaledZero) { return(TRUE)  ; }
  else                            { return(FALSE) ; }

} // end SmPseudoBox::IsLinear

/*******************************************************************//**
PURPOSE: Check pseudoBox for planarity

NOTES:
   Returns true for PointSized, Linear, and planar pseudo boxes.
    
   When pseudo box is linear, pOptPlaneNormal will be
   set vector perpendicular to xy plane.  It will be just one of
   an infinite number of planes that contain the line.

    System calculates tol if dTol3d == SM_EFF_ZERO on input
***********************************************************************/
SmBoolean SmPseudoBox::IsPlanar               
  (double      dTol3d,               // in : max allowed deviation from Plane
   SmPoint3d  *pOptPlanePoint,       // out: Point on Plane
   SmVector3d *pOptPlaneNormal,      // out: Unit Surface Normal
   double     *pOptActualDeviation)  // out: Max ControlPoint/Plane distance seen
 const 
{ 
  // test for planarity on Surface PseudoBox
  double dDist0      = m_aIntervals[0].GetLength() ;
  double dDist1      = m_aIntervals[1].GetLength() ;
  double dDist2      = m_aIntervals[2].GetLength() ;
  double dScaledZero =   dTol3d == SM_EFF_ZERO
                       ? SM_EFF_ZERO * (1.0 + GetMaxDimension())  
                       : dTol3d ;

  // set outputs
  if(pOptPlanePoint)      *pOptPlanePoint      = Evaluate(.5,.5,.5) ;
  if(pOptPlaneNormal)     *pOptPlaneNormal     = m_aBasis[0] * m_aBasis[1] ;
  if(pOptActualDeviation) *pOptActualDeviation = smos_3Min(dDist0, dDist1, dDist2) / 2.0 ;

  // check for planarity
  ULONG lDegenDimCnt =   ((dDist0/2.0  < dScaledZero) ? 1 : 0)
                       + ((dDist1/2.0  < dScaledZero) ? 1 : 0)
                       + ((dDist2/2.0  < dScaledZero) ? 1 : 0) ;

  // planes, lines, and points are all planar
  if ( lDegenDimCnt >= 1 )
    {
      if ( dDist2/2.0  < dScaledZero ) 
        { return(TRUE) ; }
      else
        { return(TRUE) ; }  // place for a break point
    }
  return(FALSE) ; 

} // end SmPseudoBox::IsPlanar

/*******************************************************************//**
PURPOSE: Determine if two pseudo boxes are disjoint.  They are disjoint
   if they share no common points.

NOTES:
  When cpOptProjUnitVector is NULL - check disjointness in 3d 
  When cpOptProjUnitVector notNULL - project pseudo boxes to common plane before
                                 computing disjointness.  This supports
                                 projection problems like computing the visual
                                 intersection of two curves along a sight line for
                                 hidden curve rendering.

***********************************************************************/
SmBoolean SmPseudoBox::AreDisjoint
  (const SmPseudoBox & crOther,              // in : Other target pseudoBox
   const SmVector3d  * cpOptProjUnitVector)  // in : NotNULL = Project PseudoBoxes to common plane
                                             //                then determine if they are disjoint
                                             //      NULL to ignore
 const
{
  // check input
  SM_ASSERT(   cpOptProjUnitVector == NULL
            || SM_IS_ZERO(cpOptProjUnitVector[0].Length() -1.0)) ;

#ifdef SM_DEBUG_CODE
  SmBoolean bDebugMe = FALSE;
  if ( bDebugMe ) 
    {
      if ( FALSE )
        { smgfx_Erase(); }
      smgfx_SetLook( 1,2, 0,0,0 ); this -> Draw(NULL); sm_GraphicsLoop();
      smgfx_SetLook( 1,2, 0,1,1 ); crOther.Draw(NULL); sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
#endif  // SM_DEBUG_CODE

  // locals
  ULONG ii, jj ;
  SmPoint3d   aPoints[6];
  SmVector3d  aVectors[6];
  SmPoint3d   aCorners[8]; 
  SmPseudoBox projThis;
  SmPseudoBox projOther;
  
  // When given a projection vector - check boxes after projecting to common plane
  //  then GetPlanes for this object
  //       GetCorners for other object
  if (cpOptProjUnitVector != NULL) 
    {
      // Do projection of the pseudo box
      SmPoint3d sOrig(0,0,0) ;
      if (ProjectToPlane(projThis, SM_PT_PARALLEL, &sOrig, cpOptProjUnitVector, NULL) != SM_SUCCESS) return(FALSE);
      if (crOther.ProjectToPlane(projOther, SM_PT_PARALLEL, &sOrig, cpOptProjUnitVector, NULL) != SM_SUCCESS) return(FALSE);
      projThis.CalcPlanes(aPoints,aVectors);
      projOther.CalcCorners(aCorners);
    }
  else 
    {
      CalcPlanes(aPoints,aVectors);
      crOther.CalcCorners(aCorners);
    }

  // quick test: when all OtherBox corners are outside any one ThisBox Plane
  //             then boxes are disjoint
  for (ii=0; ii<6; ii++) 
    {
      // Calculate D of plane equation, A, B, C are in aVectors
      double dD = - aPoints[ii].Dot(aVectors[ii]);
      SmBoolean bOutsidePlane = TRUE;

      // For every corner of other pseudobox 
      for(jj=0;jj<8;jj++) 
        {
          double dRes = aVectors[ii].Dot(aCorners[jj]) + dD;
          if (dRes < SM_EFF_ZERO) 
            {
              // Not outside this plane
              bOutsidePlane = FALSE;
              break;
            }
        } // for each corner

      // when all corners are outside any thisPseudoBox plane
      //  then boxes are disjoint
      if (bOutsidePlane)
        { return TRUE; }
    } // test against each plane

  // need more work if quick test failed
  // switch things around - check all ThisBox corners against OtherBox Planes
  //   either: boxes are disjoint and all corners will be outside some plane or
  //           boxes intersect and no plane places all corners outside.

  // Get OtherBox Planes and ThisBox corners
  if (cpOptProjUnitVector != NULL) 
    {
      projOther.CalcPlanes(aPoints,aVectors);
      projThis.CalcCorners(aCorners);
    }
  else 
    {
      crOther.CalcPlanes(aPoints,aVectors);
      CalcCorners(aCorners);
    }

  // for every otherBox Plane
  for(ii=0;ii<6;ii++) 
    {
      // Calculate D of plane equation, A, B, C are in aVectors
      double dD = - aPoints[ii].Dot(aVectors[ii]);
      SmBoolean bOutsidePlane = TRUE;

      // See if all ThisBox corners are outside the plane
      for(jj=0;jj<8;jj++) 
        {
          double dRes = aVectors[ii].Dot(aCorners[jj]) + dD;
          if (dRes < SM_EFF_ZERO) 
            {
              // Not outside this plane
              bOutsidePlane = FALSE;
              break;
            }
        } // for each corner
      // If all of points are outside of this plane we are done,
      // otherwise continue on to the next plane.
      if (bOutsidePlane)
        { return TRUE; }
    } // test against each plane

  return FALSE;

} // end SmPseudoBox::AreDisjoint

/*******************************************************************//**
PURPOSE: Determine if two pseudo boxes are disjoint after being
   projected to a common plane.  
   

NOTES: PseudoBoxes are disjoint if the projections share no common points.

  supports 3 different projections
  SM_PT_PARALLEL    - project along the surface normal to a common plane
                      (lines project to lines)
  SM_PT_PERSPECTIVE - project down eye lines towards a common eye point
                        to a common plane
                      (lines project to lines)
  SM_PT_ROTATION    - project about a axis or rotation to a common plane
                      (lines project to parabolas)

+------------------+----------------+--------------------+----------------+
|eProjectionType   | cpProjPoint    | cpProjUnitVector   | cpAuxData      |
+------------------+----------------+--------------------+----------------+
|SM_PT_PARALLEL    | pt on plane    | unit-norm to plane | not-used       |
|SM_PT_PERPSECTIVE | pt on plane    | unit-norm to plane | eye-pt         |
|SM_PT_ROTATION    | pt on rot-axis | rot-axis unit-dir  | unit-XAxis perp| 
|                  |                |                    | to rot-axis in |
|                  |                |                    | common plane   |
+------------------+----------------+--------------------+----------------+
       Input semantics for different vals of eProjectionType 

***********************************************************************/
SmBoolean SmPseudoBox::AreProjectDisjoint
  (const SmPseudoBox & crOther,              // in : Other target pseudoBox
   SmProjectionType    eProjectionType,      // in : oneof: SM_PT_PARALLEL
                                             //             SM_PT_PERSPECTIVE
                                             //             SM_PT_ROTATION
   const SmPoint3d   * cpProjPoint,          // in : case SM_PT_PARALLEL    - point on plane
                                             //           SM_PT_PERSPECTIVE - point on plane
                                             //           SM_PT_ROTATION    - point on rotation axis
   const SmVector3d  * cpProjUnitVector,     // in : case SM_PT_PARALLEL    - unit-normal to plane
                                             //           SM_PT_PERSPECTIVE - unit-normal to plane
                                             //           SM_PT_ROTATION    - unit-vec of rotation axis
   const SmVector3d  * cpAuxData)            // in : case SM_PT_PARALLEL    - not used
                                             //           SM_PT_PERSPECTIVE - eye point
                                             //           SM_PT_ROTATION    - unit-XAxis perp to rotation Axis
 const
{
  // check input - given vectors must be unit length
  SM_ASSERT(   cpProjUnitVector != NULL && SM_IS_ZERO(cpProjUnitVector->Length() - 1.0)
            && (   eProjectionType != SM_PT_ROTATION 
                || ( cpAuxData != NULL && SM_IS_ZERO(cpAuxData->Length() - 1.0)))) ;
                
#ifdef SM_DEBUG_CODE
  SmBoolean bDebugMe = FALSE;
  if ( bDebugMe ) 
    {
      if ( FALSE )
        { smgfx_Erase(); }
      smgfx_SetLook( 1,2, 0,0,0 ); this -> Draw(NULL); sm_GraphicsLoop();
      smgfx_SetLook( 1,2, 0,1,1 ); crOther.Draw(NULL); sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

  // locals
  SmPseudoBox projThis;
  SmPseudoBox projOther;

  // build the projected pseudo boxes
  if (this->ProjectToPlane  (projThis,  eProjectionType, cpProjPoint, cpProjUnitVector, cpAuxData) != SM_SUCCESS) return(FALSE);
  if (crOther.ProjectToPlane(projOther, eProjectionType, cpProjPoint, cpProjUnitVector, cpAuxData) != SM_SUCCESS) return(FALSE);

  // pass the call along
  return( projThis.AreDisjoint(projOther) ) ;

} // end SmPseudoBox::AreProjectDisjoint

/*******************************************************************//**
PURPOSE: Determine if one pseudo box contains the other.  

NOTES: 
***********************************************************************/
SmBoolean SmPseudoBox::IsContainedBy
  (const SmPseudoBox & crOther,
   double d3dTolerance) 
 const
{
  // return value
  SmBoolean bRtn = TRUE ;

  // locals
  SmPoint3d sCorner[8] ;

  // compute corner point locations
  CalcCorners(sCorner) ;

  // for all 8 corners
  for(ULONG ii=0;bRtn && ii<8;ii++)
    {
      bRtn &= crOther.ContainsPoint3d(sCorner[ii], d3dTolerance) ;
    } // end iter every corner

  // all done
  return(bRtn) ;

} // end SmPseudoBox::IsContainedBy

/*******************************************************************//**
PURPOSE: Classify Point with respect to each non-orthogonal 1d extent.

NOTES: 
***********************************************************************/
void SmPseudoBox::ClassifyPoint3d
  (const SmPoint3d & crPoint,         // in : target point
   SmExtentPointType &rExtentUType,   // out: Classification of point for U extent
   SmExtentPointType &rExtentVType,   // out: Classification of point for V extent
   SmExtentPointType &rExtentWType,   // out: Classification of point for W extent
   double dTol)                       // in : max allowed distance to count as being on a boundary
  const
{
  SM_ASSERT_DEFINED(this) ;

  SmPoint3d sA ;

  // map crPoint to PseudoBox normalized parameters
  SmStatus sStatus = InvertAbsolute(crPoint, sA) ;
  SM_ASSERT(sStatus == SM_SUCCESS) ;

  // project point onto the basis vectors
  SmPoint3d sMin(m_aIntervals[0].GetMin(), m_aIntervals[1].GetMin(), m_aIntervals[2].GetMin()) ;
  SmPoint3d sMax(m_aIntervals[0].GetMax(), m_aIntervals[1].GetMax(), m_aIntervals[2].GetMax()) ;

  // check projections against the intervals
  rExtentUType =    (smos_Fabs(sA.x-sMin.x) <= dTol) &&
                    (smos_Fabs(sA.x-sMax.x) <= dTol) ? SM_EP_BOTH
                  : (smos_Fabs(sA.x-sMin.x) <= dTol) ? SM_EP_START
                  : (smos_Fabs(sA.x-sMax.x) <= dTol) ? SM_EP_END
                  : (sA.x-sMin.x >= 0.0) &&
                    (sMax.x-sA.x >= 0.0)             ? SM_EP_INSIDE
                  : SM_EP_OUTSIDE ;

  rExtentVType =    (smos_Fabs(sA.y-sMin.y) <= dTol) &&
                    (smos_Fabs(sA.y-sMax.y) <= dTol) ? SM_EP_BOTH
                  : (smos_Fabs(sA.y-sMin.y) <= dTol) ? SM_EP_START
                  : (smos_Fabs(sA.y-sMax.y) <= dTol) ? SM_EP_END
                  : (sA.y-sMin.y >= 0.0) &&
                    (sMax.y-sA.y >= 0.0)             ? SM_EP_INSIDE
                  : SM_EP_OUTSIDE ;

  rExtentWType =    (smos_Fabs(sA.z-sMin.z) <= dTol) &&
                    (smos_Fabs(sA.z-sMax.z) <= dTol) ? SM_EP_BOTH
                  : (smos_Fabs(sA.z-sMin.z) <= dTol) ? SM_EP_START
                  : (smos_Fabs(sA.z-sMax.z) <= dTol) ? SM_EP_END
                  : (sA.z-sMin.z >= 0.0) &&
                    (sMax.z-sA.z >= 0.0)             ? SM_EP_INSIDE
                  : SM_EP_OUTSIDE ;

} // end SmPseudoBox::ClassifyPoint3d

/*******************************************************************//**
PURPOSE: Compute six planes which bound this pseudo box.  Note that
    the vectors are pointing away from the center of the pseudo box.

NOTES: 
   The planes' points will be the center of each plane of the PseudoBox,
   and the normal is the plane normal.  [B313]
***********************************************************************/
void SmPseudoBox::CalcPlanes
  (SmPoint3d aPoints[6], 
   SmVector3d aVectors[6]) 
 const
{
#ifdef SM_DEBUG_CODE
  SM_ASSERT_VALID_NO_STREAM(this);
#endif // SM_DEBUG_CODE

  double dDet = smgu_Determinant3Vectors( m_aBasis[0], m_aBasis[1], m_aBasis[2] );
  SmBoolean bNoFlip = ( dDet > 0 );

  SmPoint3d sOrigin = m_aBasis[0] * m_aIntervals[0].GetMin()
                    + m_aBasis[1] * m_aIntervals[1].GetMin()
                    + m_aBasis[2] * m_aIntervals[2].GetMin();

  SmVector3d sVec0 = m_aBasis[0] * m_aIntervals[0].GetLength();
  SmVector3d sVec1 = m_aBasis[1] * m_aIntervals[1].GetLength();
  SmVector3d sVec2 = m_aBasis[2] * m_aIntervals[2].GetLength();

  SmVector3d sNormal = m_aBasis[1] * m_aBasis[2];
  sNormal.Unitize();
  aPoints [0] = sOrigin + 0.5 * ( sVec1 + sVec2 );
  aVectors[0] = ( bNoFlip ) ? -sNormal : sNormal;
  aPoints [1] = aPoints[0] + sVec0;
  aVectors[1] = -aVectors[0];

  sNormal = m_aBasis[2] * m_aBasis[0];
  sNormal.Unitize();
  aPoints [2] = sOrigin + 0.5 * ( sVec2 + sVec0 );
  aVectors[2] = ( bNoFlip ) ? -sNormal : sNormal;
  aPoints [3] = aPoints[2] + sVec1;
  aVectors[3] = -aVectors[2];

  sNormal = m_aBasis[0] * m_aBasis[1];
  sNormal.Unitize();
  aPoints [4] = sOrigin + 0.5 * ( sVec0 + sVec1 );
  aVectors[4] = ( bNoFlip ) ? -sNormal : sNormal;
  aPoints [5] = aPoints[4] + sVec2;
  aVectors[5] = -aVectors[4];

} // end SmPseudoBox::CalcPlanes

/*******************************************************************//**
PURPOSE: Convenience function to CalcPlanes

NOTES: 
   The planes' points will be the center of each plane of the PseudoBox,
   and the normal is the plane normal.  [B313]
***********************************************************************/
void SmPseudoBox::GetPlanes
 (SmTArray<SmPoint3d>  & rPlanePoints,     // out: Point on each SmExtent3d bounding plane 
  SmTArray<SmVector3d> & rPlaneNormals)    // out: Associate normal on each SmExtent3d bounding plane
 const 
{
  // init outputs
  rPlanePoints.SetSize(6) ;
  rPlaneNormals.SetSize(6) ; 

  // pass the call along
  CalcPlanes(rPlanePoints.GetDataArray(), rPlaneNormals.GetDataArray()) ;

} // end SmPseudoBox::GetPlanes

/*******************************************************************//**
PURPOSE: Rtn inside UIvl for line = sPoint3d + u * [1 0 0]

NOTES: When Line does Not intersect PseudoBox return Ivl set to Init()
***********************************************************************/
SmExtent1d SmPseudoBox::GetUInterval
 (const SmPoint3d &crPoint3d)   // in : Point acting as a through point for the U directed line
 const    
{
  // locals
  SmExtent1d sIvl ;
  SmVector3d sLineVec(1,0,0) ; 
  ULONG      lNumFound ;              
  double     dTEnter ;               
  double     dTExit ;

  // intersect line = crPoint3d + u * [1 0 0] with PseudoBox
  IntersectLine(crPoint3d, sLineVec, lNumFound, dTEnter, dTExit) ;
  
  // Switch on the intersection type
  switch(lNumFound)
    { 
      case 0 : sIvl.Init() ; break ; 
      case 1 : sIvl.SetMinMax(dTEnter, dTEnter) ; break ; 
      case 2 : sIvl.SetMinMax(dTEnter, dTExit) ;  break ; 
    }

  // all done 
  return(sIvl) ;

} // end SmPseudoBox::GetUInterval

/*******************************************************************//**
PURPOSE: Rtn inside VIvl for line = crPoint3d + u * [0 1 0]

NOTES: When Line does Not intersect PseudoBox return Ivl set to Init()
***********************************************************************/
SmExtent1d SmPseudoBox::GetVInterval
 (const SmPoint3d &crPoint3d)   // in : Point acting as a through point for the V directed line
 const    
{
  // locals
  SmExtent1d sIvl ;
  SmVector3d sLineVec(0,1,0) ; 
  ULONG      lNumFound ;              
  double     dTEnter ;               
  double     dTExit ;

  // intersect line = crPoint3d + v * [0 1 0] with PseudoBox
  IntersectLine(crPoint3d, sLineVec, lNumFound, dTEnter, dTExit) ;
  
  // Switch on the intersection type
  switch(lNumFound)
    { 
      case 0 : sIvl.Init() ; break ; 
      case 1 : sIvl.SetMinMax(dTEnter, dTEnter) ; break ; 
      case 2 : sIvl.SetMinMax(dTEnter, dTExit) ;  break ; 
    }

  // all done 
  return(sIvl) ;

} // end SmPseudoBox::GetVInterval

/*******************************************************************//**
PURPOSE: Rtn inside WIvl for line = crPoint3d + u * [0 0 1]

NOTES: When Line does Not intersect PseudoBox return Ivl set to Init()
***********************************************************************/
SmExtent1d SmPseudoBox::GetWInterval
 (const SmPoint3d &crPoint3d)   // in : Point acting as a through point for the W directed line
 const    
{
  // locals
  SmExtent1d sIvl ;
  SmVector3d sLineVec(0,0,1) ; 
  ULONG      lNumFound ;              
  double     dTEnter ;               
  double     dTExit ;

  // intersect line = crPoint3d + w * [0 0 1] with PseudoBox
  IntersectLine(crPoint3d, sLineVec, lNumFound, dTEnter, dTExit) ;
  
  // Switch on the intersection type
  switch(lNumFound)
    { 
      case 0 : sIvl.Init() ; break ; 
      case 1 : sIvl.SetMinMax(dTEnter, dTEnter) ; break ; 
      case 2 : sIvl.SetMinMax(dTEnter, dTExit) ;  break ; 
    }

  // all done 
  return(sIvl) ;

} // end SmPseudoBox::GetWInterval

/*******************************************************************//**
PURPOSE: Return TRUE when dimensions are all set to init values

NOTES: 
***********************************************************************/
SmBoolean SmPseudoBox::IsInit
  () 
 const
{
  return(   m_aIntervals[0].IsInit() 
         && m_aIntervals[1].IsInit() 
         && m_aIntervals[2].IsInit()) ; 

} // end SmPseudoBox::IsInit

/*******************************************************************//**
PURPOSE: Return TRUE when all boundaries are bound,
         no +/-SM_INFINITE_PARAMETER values.

NOTES: 
***********************************************************************/
SmBoolean SmPseudoBox::IsBounded
 (SmBoundaryType eOptBndryType[3])   // in : opt ptr to an array [BT_Type_U, _BTTYPE_V, _BTTYPE_W]
                                     //      NULL to ignore, default:[NULL]
 const
{
  SmBoundaryType eType[3] ; 
  
  // classify U interval 
  m_aIntervals[0].IsBounded(&eType[0]) ;
  m_aIntervals[1].IsBounded(&eType[1]) ;
  m_aIntervals[2].IsBounded(&eType[2]) ;

  // set output
  if(eOptBndryType) 
    {
      eOptBndryType[0] = eType[0] ; 
      eOptBndryType[1] = eType[1] ; 
      eOptBndryType[2] = eType[2] ; 
    }

  // all done - return bounded status
  return(   eType[0] == SM_BT_BOUNDED
         && eType[1] == SM_BT_BOUNDED
         && eType[2] == SM_BT_BOUNDED ) ;                                   

} // end SmPseudoBox::IsBounded

/*******************************************************************//**
PURPOSE: Return TRUE when bases are orthogonal to one another
         and parallel to the X, Y, and Z axes.
NOTES: Useful for building a BBOx from a Pseudo box.
***********************************************************************/
SmBoolean SmPseudoBox::IsAxisAligned  // rtn: TRUE = bases are orthogonal and parallel to XYZ.
 (long alAxisMap[3])                  // out: alAxisMap[0] = index+1 of basis parallel to X, neg = in negative direction
 const                                //      alAxisMap[1] = index+1 of basis parallel to Y, neg = in negative direction
                                      //      alAxisMap[2] = index+1 of basis parallel to Z, neg = in negative direction
{
  // init output
  alAxisMap[0] = SM_BIG_ULONG ; 
  alAxisMap[1] = SM_BIG_ULONG ; 
  alAxisMap[2] = SM_BIG_ULONG ; 

  // no work - not orthogonal
  if(m_bOrthogonalBasis == FALSE)
    { return FALSE ; }

  // locals
  ULONG ii ;
  SmVector3d sX(1,0,0), sY(0,1,0), sZ(0,0,1) ;

  // for each basis
  for(ii=0;ii<3;ii++)
    {
      double dLength = m_aBasis[ii].Length() ;
      double dXDot = m_aBasis[ii].Dot(sX)/dLength ; 
      double dYDot = m_aBasis[ii].Dot(sY)/dLength ; 
      double dZDot = m_aBasis[ii].Dot(sZ)/dLength ;              
                                                         
      if     (SM_ARE_SAME(dXDot,  1.0)) alAxisMap[ii] =  1 ;  // use index+1. zero not used, direction can be the sign of the value
      else if(SM_ARE_SAME(dXDot, -1.0)) alAxisMap[ii] = -1 ;  // use index+1. zero not used, direction can be the sign of the value
      else if(SM_ARE_SAME(dYDot,  1.0)) alAxisMap[ii] =  2 ;  // use index+1. zero not used, direction can be the sign of the value
      else if(SM_ARE_SAME(dYDot, -1.0)) alAxisMap[ii] = -2 ;  // use index+1. zero not used, direction can be the sign of the value
      else if(SM_ARE_SAME(dZDot,  1.0)) alAxisMap[ii] =  3 ;  // use index+1. zero not used, direction can be the sign of the value
      else if(SM_ARE_SAME(dZDot, -1.0)) alAxisMap[ii] = -3 ;  // use index+1. zero not used, direction can be the sign of the value
      else { return(FALSE) ; }

    } // end iter every basis

  // all done
  return(TRUE) ; 

} // end SmPseudoBox::IsAxisAligned

/*******************************************************************//**
PURPOSE: Return TRUE when bases are orthogonal to one another
         and parallel to the X, Y, and Z axes.
NOTES: Useful for building a BBOx from a Pseudo box.
***********************************************************************/
SmBoolean SmPseudoBox::AreAligned  // rtn: TRUE = bases are parallel to one another (any order)
 (const SmPseudoBox & crOther,     // in : other arg
  long alAxisMap[3])               // out: alAxisMap[0] = index+1 of basis parallel to crOther.X, neg = in negative direction
 const                             //      alAxisMap[1] = index+1 of basis parallel to crOther.Y, neg = in negative direction
                                   //      alAxisMap[2] = index+1 of basis parallel to crOther.Z, neg = in negative direction
{
  // init output
  alAxisMap[0] = SM_BIG_ULONG ; 
  alAxisMap[1] = SM_BIG_ULONG ; 
  alAxisMap[2] = SM_BIG_ULONG ; 

  // no work - not orthogonal
  if(m_bOrthogonalBasis == FALSE)
    { return FALSE ; }

  // locals
  ULONG ii ; 
  SmVector3d rX = crOther.GetBasis(0)/crOther.GetBasis(0).Length() ; 
  SmVector3d rY = crOther.GetBasis(1)/crOther.GetBasis(1).Length() ; 
  SmVector3d rZ = crOther.GetBasis(2)/crOther.GetBasis(2).Length() ; 

  // for each basis
  for(ii=0;ii<3;ii++)
    {
      double dLength = m_aBasis[ii].Length() ;
      double dXDot   = m_aBasis[ii].Dot(rX)/dLength ; 
      double dYDot   = m_aBasis[ii].Dot(rY)/dLength ; 
      double dZDot   = m_aBasis[ii].Dot(rZ)/dLength ;              
                                                         
      if     (SM_ARE_SAME(dXDot,  1.0)) alAxisMap[ii] =  1 ;  // use index+1. zero not used, direction can be the sign of the value
      else if(SM_ARE_SAME(dXDot, -1.0)) alAxisMap[ii] = -1 ;  // use index+1. zero not used, direction can be the sign of the value
      else if(SM_ARE_SAME(dYDot,  1.0)) alAxisMap[ii] =  2 ;  // use index+1. zero not used, direction can be the sign of the value
      else if(SM_ARE_SAME(dYDot, -1.0)) alAxisMap[ii] = -2 ;  // use index+1. zero not used, direction can be the sign of the value
      else if(SM_ARE_SAME(dZDot,  1.0)) alAxisMap[ii] =  3 ;  // use index+1. zero not used, direction can be the sign of the value
      else if(SM_ARE_SAME(dZDot, -1.0)) alAxisMap[ii] = -3 ;  // use index+1. zero not used, direction can be the sign of the value
      else { return(FALSE) ; }

    } // end iter every basis

  // all done
  return(TRUE) ; 

} // end SmPseudoBox::AreAligned

/*******************************************************************//**
PURPOSE: return a bounded SmPseudoBox to approximate an unbounded one

NOTES: 0. returns a SmPseudoBox whose infinite boundary values 
          have been replaced by finite values.
          Those replacement values are based upon the dUnboundedHalfSize,
          pUnboundedCenter, and interval opposing end values.

       1. This is used for graphics and sampling to allow an application
          to easily define an area of focus for infinite extents.
          
       2. This method handles all the combinations of half spaces and unbounded spaces.
          
       3. The approximation of a BOUNDED extent is an exact copy of the original extent.
***********************************************************************/
SmPseudoBox SmPseudoBox::ApproximateUnbounded    // eff: return a bounded SmPseudoBox to approximate an unbounded one                  
 (SmPoint3d   * pUnboundedCenter,                // in : center of unbounded intervals, NULL = [0,0,0], default:[NULL]
  double        dUnboundedHalfSize,              // in : the size used for infinite 1/2 spaces, default:[SM_BOUNDED_INFINITE_PARAM]
                                                 //      a totally unbounded volume will is approximated by a box twice this size
  SmPseudoBox * pOptExpandedApprox)              // out: a 2nd extent expanded a small bit - used by graphics, NULL to ignore, default:[NULL]
 const
{
  // locals
  SmPoint3d sCenter(0,0,0) ;
  if(pUnboundedCenter) sCenter = *pUnboundedCenter ;
  SmExtent1d sExpandedU, sExpandedV, sExpandedW ; 

  // approximate the intervals
  SmExtent1d sIvlU = m_aIntervals[0].ApproximateUnbounded(sCenter.x, dUnboundedHalfSize, &sExpandedU) ;
  SmExtent1d sIvlV = m_aIntervals[1].ApproximateUnbounded(sCenter.y, dUnboundedHalfSize, &sExpandedV) ;
  SmExtent1d sIvlW = m_aIntervals[2].ApproximateUnbounded(sCenter.z, dUnboundedHalfSize, &sExpandedW) ;

  // set output
  if( pOptExpandedApprox )
    {
      pOptExpandedApprox->SetIntervals(sExpandedU, sExpandedV, sExpandedW) ;
    }

  // copy this PseudoBox - and bound its intervals
  SmPseudoBox sRtnBox(*this) ;
  sRtnBox.SetIntervals(sIvlU, sIvlV, sIvlW) ;

  // all done
  return( sRtnBox ) ;

} // end SmPseudoBox::ApproximateUnbounded

/*******************************************************************//**
PURPOSE: Get corners of Extent  

NOTES: 
***********************************************************************/
void SmPseudoBox::GetCorners
 (SmTArray<SmPoint3d> &rCornerPoints)
 const
{
  // init output
  rCornerPoints.SetSize(8) ;

  // pass the call along
  CalcCorners(rCornerPoints.GetDataArray()) ; 

} // end SmPseudoBox::GetCorners

/*******************************************************************//**
PURPOSE: Get Edge endpoints of Extent  

NOTES: Every Pair of output rCornerPoints [iEven iEven+1] bound one of the
       12 edges of the PseudoBox.  There are always 24 points returned
       in the output rCornterPoints.
***********************************************************************/
void SmPseudoBox::GetEdges
 (SmTArray<SmPoint3d> &rCornerPoints)  // out: every pair of points [iEven,iEven+1] marks one extent edge boundary
 const
{
  // init output
  rCornerPoints.SetSize(24) ;

  // Evaluate and load edge end points for the x == 0 plane
  rCornerPoints[0] = Evaluate(0,0,0) ;
  rCornerPoints[1] = Evaluate(0,0,1) ;
  rCornerPoints[2] = Evaluate(0,0,1) ;
  rCornerPoints[3] = Evaluate(0,1,1) ;
  rCornerPoints[4] = Evaluate(0,1,1) ;
  rCornerPoints[5] = Evaluate(0,1,0) ;
  rCornerPoints[6] = Evaluate(0,1,0) ;
  rCornerPoints[7] = Evaluate(0,0,0) ;

  // Evaluate and load edge end points for the x == 1 plane
  rCornerPoints[8]  = Evaluate(1,0,0) ;
  rCornerPoints[9]  = Evaluate(1,0,1) ;
  rCornerPoints[10] = Evaluate(1,0,1) ;
  rCornerPoints[11] = Evaluate(1,1,1) ;
  rCornerPoints[12] = Evaluate(1,1,1) ;
  rCornerPoints[13] = Evaluate(1,1,0) ;
  rCornerPoints[14] = Evaluate(1,1,0) ;
  rCornerPoints[15] = Evaluate(1,0,0) ;

  // Evaluate and load edge end points that run from x=0 to x=1
  rCornerPoints[16] = Evaluate(0,0,0) ;
  rCornerPoints[17] = Evaluate(1,0,0) ;
  rCornerPoints[18] = Evaluate(0,0,1) ;
  rCornerPoints[19] = Evaluate(1,0,1) ;
  rCornerPoints[20] = Evaluate(0,1,1) ;
  rCornerPoints[21] = Evaluate(1,1,1) ;
  rCornerPoints[22] = Evaluate(0,1,0) ;
  rCornerPoints[23] = Evaluate(1,1,0) ;

} // end SmPseudoBox::GetCorners

/*******************************************************************//**
PURPOSE: Compute the eight corners of the pseudo box.

NOTES: 
  Corners are returned in this order:
  000, 100, 110, 010,
  001, 101, 111, 011
***********************************************************************/
void SmPseudoBox::CalcCorners
  (SmPoint3d aCorners[8]) 
 const
{
#ifdef SM_DEBUG_CODE
  SM_ASSERT_VALID_NO_STREAM(this);
#endif // SM_DEBUG_CODE
  
  // min corner
  aCorners[0] =   m_aBasis[0] * m_aIntervals[0].GetMin()
                + m_aBasis[1] * m_aIntervals[1].GetMin()
                + m_aBasis[2] * m_aIntervals[2].GetMin();

  // vectors to far corners
  SmVector3d sDel0 = m_aBasis[0] * m_aIntervals[0].GetLength();
  SmVector3d sDel1 = m_aBasis[1] * m_aIntervals[1].GetLength();
  SmVector3d sDel2 = m_aBasis[2] * m_aIntervals[2].GetLength();

  // hop around to all other corners
  aCorners[1] = aCorners[0] + sDel0;
  aCorners[2] = aCorners[1] + sDel1;
  aCorners[3] = aCorners[0] + sDel1;

  aCorners[4] = aCorners[0] + sDel2;
  aCorners[5] = aCorners[1] + sDel2;
  aCorners[6] = aCorners[2] + sDel2;
  aCorners[7] = aCorners[3] + sDel2;

} // end SmPseudoBox::CalcCorners

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
void SmPseudoBox::Dump
  (void) 
 const
{
  TCHAR sBuff[SM_TBLOCK_SIZE], sBuffForFile[SM_TBLOCK_SIZE];
  smos_sprintf(sBuff,       _T("SmPseudoBox = 0x%p, "),this);
  smos_sprintf(sBuffForFile,_T("SmPseudoBox = %s, "), _T("notNULL") );
  smos_WriteBuffer(sBuff, sBuffForFile);

  for (ULONG i=0; i<3; i++) 
    {
      smos_sprintf(sBuff,_T("\n    Basis[%ld] = [%lf,%lf,%lf], Interval:[%lf %lf]"),
          i, 
          m_aBasis[i].x, 
          m_aBasis[i].y, 
          m_aBasis[i].z,
          m_aIntervals[i].GetMin(), 
          m_aIntervals[i].GetMax());
      smos_WriteBuffer(sBuff);
    }

  // output box size measures
  smos_sprintf(sBuff, _T("\n    PseudoBox Volume[%16.16lf], XYArea[%16.16lf], MaxDim[%16.16lf]"), 
                    GetVolume(),
                    GetXYArea(),
                    GetMaxDimension());
  smos_WriteBuffer(sBuff) ; 
  smos_WriteBuffer(_T("\n"));

} // end SmPseudoBox::Dump

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
SmDisplayList * SmPseudoBox::Draw
 (const SmContext * pContext,     // NotUsed: in :
  SmGfxArraySet   * pOptGfxSet)   // i/o:
 const
{
  SM_REF1(pContext) ;
  SmDisplayList *pRtn = NULL ;

#ifdef SM_GFX_CODE
    SmPoint3d aCorners[8];
    CalcCorners(aCorners);
    smgfx_Open(smgfx_GetRuleColor(), NULL, NULL, FALSE, pOptGfxSet);
    smgfx_DrawPoint(aCorners[0].x,aCorners[0].y,aCorners[0].z, pOptGfxSet);

    smgfx_DrawLine(aCorners[0].x,aCorners[0].y,aCorners[0].z,
                   aCorners[1].x,aCorners[1].y,aCorners[1].z, pOptGfxSet);
    smgfx_DrawLine(aCorners[1].x,aCorners[1].y,aCorners[1].z,
                   aCorners[2].x,aCorners[2].y,aCorners[2].z, pOptGfxSet);
    smgfx_DrawLine(aCorners[2].x,aCorners[2].y,aCorners[2].z,
                   aCorners[3].x,aCorners[3].y,aCorners[3].z, pOptGfxSet);
    smgfx_DrawLine(aCorners[3].x,aCorners[3].y,aCorners[3].z,
                   aCorners[0].x,aCorners[0].y,aCorners[0].z, pOptGfxSet);

    smgfx_DrawLine(aCorners[4].x,aCorners[4].y,aCorners[4].z,
                   aCorners[5].x,aCorners[5].y,aCorners[5].z, pOptGfxSet);
    smgfx_DrawLine(aCorners[5].x,aCorners[5].y,aCorners[5].z,
                   aCorners[6].x,aCorners[6].y,aCorners[6].z, pOptGfxSet);
    smgfx_DrawLine(aCorners[6].x,aCorners[6].y,aCorners[6].z,
                   aCorners[7].x,aCorners[7].y,aCorners[7].z, pOptGfxSet);
    smgfx_DrawLine(aCorners[7].x,aCorners[7].y,aCorners[7].z,
                   aCorners[4].x,aCorners[4].y,aCorners[4].z, pOptGfxSet);

    smgfx_DrawLine(aCorners[0].x,aCorners[0].y,aCorners[0].z,
                   aCorners[4].x,aCorners[4].y,aCorners[4].z, pOptGfxSet);
    smgfx_DrawLine(aCorners[1].x,aCorners[1].y,aCorners[1].z,
                   aCorners[5].x,aCorners[5].y,aCorners[5].z, pOptGfxSet);
    smgfx_DrawLine(aCorners[2].x,aCorners[2].y,aCorners[2].z,
                   aCorners[6].x,aCorners[6].y,aCorners[6].z, pOptGfxSet);
    smgfx_DrawLine(aCorners[3].x,aCorners[3].y,aCorners[3].z,
                   aCorners[7].x,aCorners[7].y,aCorners[7].z, pOptGfxSet);


  pRtn = smgfx_Close(pOptGfxSet) ;

#else
  SM_REF1(pOptGfxSet);
#endif // end SM_GFX_CODE
  return(pRtn) ;

} // end SmPseudoBox::Draw

/*******************************************************************//**
PURPOSE: AssertValid() Test reporting static labels
***********************************************************************/
SmAssertReportLabel sAssertPseudoBox_list[] =
{
  /* 0 */ {SM_AT_MINMAX,      _T("Min Max"),    _T("m_aIntervals Min <= Max") },
  /* 1 */ {SM_AT_UNIT_VECTOR, _T("UnitVector"), _T("m_aBasis vectors must be unit length") }
} ;

/*******************************************************************//**
PURPOSE: Determine if the pseudo box is valid.

RETURNS ---  TRUE  = OK
             FALSE = Problem
***********************************************************************/
SmBoolean SmPseudoBox::AssertValid
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

  for (ULONG i=0; i<3; i++) 
    {
      bRtn &= SM_ASSERT_VALUE_REPORT(0, SM_LEVEL_0, (m_aIntervals[i].GetMin() <= m_aIntervals[i].GetMax()), 0.0, m_aIntervals[i].GetMin() - m_aIntervals[i].GetMax(), _T("") ) ;
      bRtn &= SM_ASSERT_VALUE_REPORT(1, SM_LEVEL_0, (SM_IS_ZERO(m_aBasis[i].Length() - 1.0)), SM_EFF_ZERO, m_aBasis[i].Length() - 1.0, _T("") ) ;
    }

  // all done
  SM_ASSERT_BREAK(bRtn) ;

  return(bRtn) ;

} // end SmPseudoBox::AssertValid 

/*******************************************************************//**
PURPOSE: return TRUE when member values are defined, else FALSE.

NOTES: 
***********************************************************************/
SmBoolean SmPseudoBox::AssertDefined() const
{
  // init return value
  SmBoolean bRtn ;

  // check members
  bRtn  = m_aIntervals[0].AssertDefined() ;
  bRtn &= m_aIntervals[1].AssertDefined() ;
  bRtn &= m_aIntervals[2].AssertDefined() ;

  bRtn &= m_aBasis[0].IsInitialized() ;
  bRtn &= m_aBasis[1].IsInitialized() ;
  bRtn &= m_aBasis[2].IsInitialized() ;

  // all done
  // SM_ASSERT(bRtn) ;
  return(bRtn) ;

} // end SmPseudoBox::AssertDefined

/*******************************************************************//**
PURPOSE: Project the pseudo box onto a plane.

NOTES: supports 3 different projections
  SM_PT_PARALLEL    - project along the surface normal to a common plane
                      (lines project to lines)
  SM_PT_PERSPECTIVE - project down eye lines towards a common eye point
                        to a common plane
                      (lines project to lines)
  SM_PT_ROTATION    - project about a axis or rotation to a common plane
                      (lines project to parabolas)

+------------------+----------------+--------------------+----------------+
|eProjectionType   | cpProjPoint    | cpProjUnitVector   | cpAuxData      |
+------------------+----------------+--------------------+----------------+
|SM_PT_PARALLEL    | pt on plane    | unit-norm to plane | not-used       |
|SM_PT_PERPSECTIVE | pt on plane    | unit-norm to plane | eye-pt         |
|SM_PT_ROTATION    | pt on rot-axis | rot-axis unit-dir  | unit-XAxis perp| 
|                  |                |                    | to rot-axis in |
|                  |                |                    | common plane   |
+------------------+----------------+--------------------+----------------+
       Input semantics for different vals of eProjectionType 

***********************************************************************/
SmStatus SmPseudoBox::ProjectToPlane
  (SmPseudoBox       & rProjectedBox,     // out: PseudoBox projected to plane, may be same as this SmPsdudoBox
   SmProjectionType    eProjectionType,   // in : oneof: SM_PT_PARALLEL
                                          //             SM_PT_PERSPECTIVE
                                          //             SM_PT_ROTATION
   const SmPoint3d   * cpProjPoint,       // in : case SM_PT_PARALLEL    - point on plane
                                          //           SM_PT_PERSPECTIVE - point on plane
                                          //           SM_PT_ROTATION    - point on rotation axis
   const SmVector3d  * cpProjUnitVector,  // in : case SM_PT_PARALLEL    - unit-normal to plane
                                          //           SM_PT_PERSPECTIVE - unit-normal to plane
                                          //           SM_PT_ROTATION    - unit-vec of rotation axis
   const SmVector3d  * cpAuxData)         // in : case SM_PT_PARALLEL    - not used
                                          //           SM_PT_PERSPECTIVE - eye point
                                          //           SM_PT_ROTATION    - unit-XAxis perp to rotation Axis
 const
{
  // check input - valid eProjectType values
  SM_ASSERT(   eProjectionType == SM_PT_PARALLEL
            || eProjectionType == SM_PT_PERSPECTIVE
            || eProjectionType == SM_PT_ROTATION) ;

  // check input - cpProjPoint must be nonNULL
  SM_ASSERT( cpProjPoint != NULL ) ; 

  // check input - cpProjUnitVector must be unit length
  SM_ASSERT( cpProjUnitVector != NULL  && SM_IS_ZERO(cpProjUnitVector->Length() - 1.0) ) ;

  // check input - when SM_PT_ROTATION cpAuxData must be unit length
  SM_ASSERT(   (eProjectionType != SM_PT_PERSPECTIVE && eProjectionType != SM_PT_ROTATION)
            || (   cpAuxData != NULL 
                && (eProjectionType != SM_PT_ROTATION || SM_IS_ZERO(cpAuxData->Length() - 1.0)))) ;
    
  // locals
  ULONG ii, jj, kk, ll, mm ;
  double      adLen[4] = {0,0,0,0};     // contains projected vector * Ivl.size lengths
  SmVector3d  aBasis[4] ;               // contains projected basis vectors
  SmPoint3d   aCorners[8] ;             // contains projected corner points

  // get pseudo box corners
  //   Corners are returned in this order:
  //   000, 100, 110, 010,
  //   001, 101, 111, 011
  CalcCorners(aCorners) ;

  // project every Basis vector into array aBasis sorted in order of the store projected lengths in adLen
  for (ii=0; ii<3; ii++) 
    {
      // length of projected basis * Ivl.Length  (watch indices - tricky here to minimize sort of three things cost)
      aBasis[ii+1] = (  eProjectionType == SM_PT_PARALLEL    ? m_aBasis[ii].ProjectToPlane(*cpProjUnitVector) 
                      : eProjectionType == SM_PT_ROTATION    ? m_aBasis[ii].RotateProjectToPlane(*cpProjUnitVector, *cpAuxData)
                      :                  /*SM_PT_PERSPECITVE*/ m_aBasis[ii].PerspectiveProjectToPlane(*cpProjPoint, 
                                                                                                      *cpProjPoint, 
                                                                                                      *cpProjUnitVector, 
                                                                                                      *cpAuxData)) ;
      adLen[ii+1]  =  aBasis[ii+1].Length() 
                    * (m_aIntervals[ii].GetMax() - m_aIntervals[ii].GetMin()) ;

      // sort the basis vectors based on length  // after ii == 0 vals=[x 0 x x]  0,1,2 = values computed in passes ii==0,1,2
      //                                         // after ii == 1 vals=[L S 1 x]  L = longest vec seen so far
      //                                         // after ii == 2 vals=[L M S 2]  S = shortest vec seen so far, M = mid length vector
      if(ii==1)
        { if(adLen[2] > adLen[1]) { adLen[0] = adLen[2] ; aBasis[0] = aBasis[2] ;}
          else                    { adLen[0] = adLen[1] ; aBasis[0] = aBasis[1] ;
                                    adLen[1] = adLen[2] ; aBasis[1] = aBasis[2] ;}
        }
      if(ii==2)
        { if     (adLen[3] > adLen[0]) { adLen[2] = adLen[1] ; aBasis[2] = aBasis[1] ;
                                         adLen[1] = adLen[0] ; aBasis[1] = aBasis[0] ;
                                         adLen[0] = adLen[3] ; aBasis[0] = aBasis[3] ;}
          else if(adLen[3] > adLen[1]) { adLen[2] = adLen[1] ; aBasis[2] = aBasis[1] ;
                                         adLen[1] = adLen[3] ; aBasis[1] = aBasis[3] ;}
          else                         { adLen[2] = adLen[3] ; aBasis[2] = aBasis[3] ;}
        }
        
    } // end iter every basis looking for largest projected vector

  // arrive here when done accessing 'this' pseudobBox properties
  // In case 'this' pseudoBox == &rProjectedBox
  //   make sure: 1.) never to access original 'this' properties after this point
  //              2.) change original 'this' properties prior to this point

  // init output - set rProjectedBox to an empty space
  rProjectedBox.Init() ; // init to large negative intervals, leave basis vectors alone

  // when all basis vectors project to zero length - make any basis vector set
  if (adLen[0] < SM_EFF_ZERO) 
    {
      // init basis vectors to [100 010 001] - leave intervals alone
      rProjectedBox.InitBasis() ; 
    }
  else // make basis set placing 'X' axis in direction of largest projected vector
    {  //                placing 'y' axis in direction of next independent projected vector
       //                placing 'z' = x cross y
       // when X is the only nondegenerate vector - set basis to an orthogonal set

       SmVector3d sXCrossY = aBasis[0] * aBasis[1] ;
       SmVector3d sXCrossZ = aBasis[0] * aBasis[2] ;

       double dXCrossYLenSq = sXCrossY.LengthSquared() ;
       double dXCrossZLenSq = sXCrossZ.LengthSquared() ;

       // build new basis from combination of largest possible independent projected basis vectors
       if     (dXCrossYLenSq > SM_EFF_ZERO_SQ) // Projected X and Y are independent
         { aBasis[0].Unitize() ;
           aBasis[1].Unitize() ;
           sXCrossY.Unitize() ;
           rProjectedBox.SetBasis(aBasis[0], aBasis[1], sXCrossY) ;
         }
       else if(dXCrossZLenSq > SM_EFF_ZERO_SQ) // Projected X and Z are independent
         { aBasis[0].Unitize() ;
           aBasis[2].Unitize() ;
           sXCrossZ.Unitize() ;
           rProjectedBox.SetBasis(aBasis[0], aBasis[2], sXCrossZ) ; 
         }
       else // X, Y and Z don't span the plane                                   
         { // build any orthogonal basis about aBasis[0] placing X and Y in the projection plane
           
           SmVector3d sYRef ;
           if(eProjectionType == SM_PT_ROTATION) 
             {
               SmVector3d sXCrossRotAxis = aBasis[0] * *cpProjUnitVector ;
               SmVector3d sXCrossXAxis   = aBasis[0] * *cpAuxData ;
               sYRef =  (sXCrossRotAxis.LengthSquared() > sXCrossXAxis.LengthSquared()) 
                       ? *cpProjUnitVector
                       : *cpAuxData ;
             }
           else
             {
               sYRef = aBasis[0] * *cpProjUnitVector ;
             }
           SER(aBasis[0].MakeUnitOrthoVectors(&sYRef, 
                                              rProjectedBox.m_aBasis[0],
                                              rProjectedBox.m_aBasis[1], 
                                              rProjectedBox.m_aBasis[2]));
         
         } // end X Y and Z don't span projection plane check
    } // end use projected basis vectors to build pseudo basis vectors branch

  // Size new pseduo box by adding every original ThisBox corner to it
  for (ii=0; ii<8; ii++) 
    {
      // Project Corner to Plane and add to outputBox
      aCorners[ii] = aCorners[ii].ProjectPointToPlane(*cpProjPoint, *cpProjUnitVector);
      rProjectedBox.AddPoint3d(aCorners[ii]);
    }

  // arrive here after orienting and loading a projected pseudobox
  //  This is complete for projections that map lines to lines.  
  //  But for the rotated-projection - this box has to be increased
  //  to make sure that the projected parabolas of the actual projected
  //  pseudo box all fit within the linear sides of the approximated
  //  projected pseudobox that will be returned.

  if(eProjectionType == SM_PT_ROTATION)
    {
      // rotating an XYZ point to a common plane is equivalent to changing to cylindrical
      // coordinates (Radius, Theta, Z) and then setting theta to zero.
      // the mapping for Z is simple; Zcylindrical = Zcartesian.  
      // The R value is just the distance between the point and the rotation axis.
      // rotating a line to the common plane results in a parabola.  For the pseudo
      // box rotation all we need is to compute the maximum and minimum values
      // of that parabola as measured in the directions of the rotated-projected
      // pseudoBox basis vectors.  Those maxima will occur at either the 
      // original pseudoBox corner points or somewhere along the edges where ever
      // the projected parabolas become perpendicular to the rotated-projected Pseudo
      // Box axis. 
      //
      // Let's call
      //   P0     = start point of Line L(s)  where L(s) is an edge of the original pseudo box
      //   P1     = end   point of Line L(s)   
      //   P(s)   = (1-s)*P0 + (s)*P1,               The line between P0 and P1
      //   Ps     = (P1-P0), 1st derivative of P(s) = d(P(s))/ds
      //   C      = a point on the rotation axis.
      //   N      = unit direction of the rotation axis
      //   X      = unit direction perpendicular to the rotation axis
      //            (The rotated-projected common-plane is spanned by vectors N and X)
      //   Q(s)   = P(s)-C
      //   Qs     = Ps = (P1-P0)
      //   A0     = 1st basis vector of the rotated-projected pseudoBox (in the CommonPlane)
      //   A1     = 2nd basis vector of the rotated-projected pseudoBox (in the CommonPlane)
      //
      //   R(s)   = The rotated-projection of a 3d line into the common plane (it's a parabola)
      //   R(s)   =   Sqrt( (Q(s) - Q(s).N*N) . (Q(s) - Q(s).N*N)) * X
      //            +                    Q(s).N                    * N       
      //
      //   Rs     = 1st derivative of R(s) = d(R(s))/ds 
      //   Rs     =         ((Qs) - Qs.N*N) . (Q(s) - Q(s).N*N)
      //              ---------------------------------------------------- * X
      //               Sqrt((Q(s) - Q(s).N*N) . (Q(s) - Q(s).N*N))
      //            +                    (Qs).N                            * N 
      //
      //  The line point which projects to a maxima coordinate value in the direction of some vector A 
      //  happens where the Tangent(R(s)) is perpendicular to vector A.
      //
      //    0 = Rs.A
      // 
      //    Let A be A0=[a0x*X + a0n*N] or A1=[a1x*X + a1n*N], and solve for s to find sMax.  
      //    where a0x = A0.X,   a1x = A1.X
      //          a0n = A0.N,   a1n = A1.N
      //    Compute R(sMax) to find the min/max axMax, axMin, anMax, and anMin values needed 
      //    for the new Rotated-Projected PseudoBox
      //
      // 
      //    0 = Rs.A
      //    0 =         ((Qs) - Qs.N*N) . (Q(s) - Q(s).N*N)       
      //          ----------------------------------------------- * ax * X.X     where X.X = 1
      //           Sqrt((Q(s) - Q(s).N*N) . (Q(s) - Q(s).N*N))                   where N.N = 1
      //        +                    (Qs).N                       * an * N.N
      //
      //   yields
      //    (Qs - Qs.N*N) . (Q(s) - Q(s).N*N) * ax =  -Qs.N * an * Sqrt( (Q(s) - Q(s).N*N) . (Q(s) - Q(s).N*N))
      //
      //    + 1*Qs.Q(s)       * ax =  -(Qs).N * an * Sqrt(  Q(s).Q(s)          
      //    - 1*Qs.N*Q(s).N   * ax                        - Q(s).N * Q(s).N)
      //                                                           
      //   which can be squared to get rid of the Sqrt function and then simplified to
      //
      //   square( + Qs.Q(s)                =  Qs.N * Qs.N * an * an * (  Q(s).Q(s)                          
      //           - Qs.N*Q(s).N) * ax * ax                             - Q(s).N * Q(s).N )         
      //                                               
      //   ax*ax* (+   Q(s).Qs * Q(s).Qs               =  Qs.N*Qs.N*an*an*(  Q(s).Q(s)        
      //           - 2*Q(s).Qs * Q(s).N * Qs.N                             - Q(s).N * Q(s).N )                          
      //           +   Q(s).N  * Q(s).N * Qs.N * Qs.N)
      //
      //   Since the equation was squared we expect the linear equation to become quadratic with
      //   a pair of real roots since there is only one solution to the associated linear equation.         
      //                                       
      //   with Q(s) = ((1-s)*P0 + (s)*P1 - C)
      //        Qs   = (P1 - P0)  
      // 
      //   we can write a quadratic equation in s as:                                      
      //                                           
      //   ax*ax* (+ (s*s)*(+  (+P0.P10 * P0.P10)               =  P10.N*P10.N*an*an*(+ (s*s)*(+1*(P0.P0 + P0.N * -P0.N)     
      //                    -2*(+P0.P10 * P1.P10)                                              -2*(P0.P1 + P1.N * -P0.N)     
      //                    +  (+P1.P10 * P1.P10)                                              +1*(P1.P1 + P1.N * -P1.N) )            
      //                    +  (+P0.N * P0.N * P10.N * P10.N)                         +  (s) *(-2*(P0.P0 + P0.N * -P0.N)      
      //                    -2*(+P0.N * P1.N * P10.N * P10.N)                                  +2*(P0.P1 + P1.N * -P0.N)          
      //                    +  (+P1.N * P1.N * P10.N * P10.N)                                  -2*(-P0.C + P0.N *   C.N)                            
      //                    +  (+P0.P10 * P0.N * -2 * P10.N)                                   +2*(-P1.C + P1.N *   C.N) )
      //                    -  (+P1.P10 * P0.N * -2 * P10.N)                          +  (1) *(+1*(P0.P0 + P0.N * -P0.N)            
      //                    -  (+P0.P10 * P1.N * -2 * P10.N)                          +        +2*(-P0.C + P0.N *   C.N)            
      //                    +  (+P1.P10 * P1.N * -2 * P10.N) )                        +        +1*(  C.C + -C.N *   C.N) )  )                         
      //           +  (s) *(+2*(+P0.P10 * P1.P10)               
      //                    -2*(+P0.P10 * P0.P10)                                         
      //                    -2*(+P0.P10 * -C.P10)               
      //                    +2*(+P1.P10 * -C.P10)               
      //                    -2*(+P0.P10 * P0.N * -2 * P10.N)                               
      //                    +  (+P1.P10 * P0.N * -2 * P10.N)    
      //                    -  ( -C.P10 * P0.N * -2 * P10.N)    
      //                    +  (+P0.P10 * P1.N * -2 * P10.N)    
      //                    +  ( -C.P10 * P1.N * -2 * P10.N)    
      //                    -  (+P0.P10 * -C.N * -2 * P10.N)    
      //                    +  (+P1.P10 * -C.N * -2 * P10.N)    
      //                    -2*(+P0.N * P0.N * P10.N * P10.N)                             
      //                    +2*(+P1.N * P0.N * P10.N * P10.N)   
      //                    -2*(+P0.N * -C.N * P10.N * P10.N)   
      //                    +2*(+P1.N * -C.N * P10.N * P10.N) )
      //           +  (1) *(+1*(+P0.P10 * P0.P10)               
      //                    + 2*(+P0.P10 * -C.P10)               
      //                    + 1*(- C.P10 * -C.P10)               
      //                    + 1*(+P0.P10 * P0.N * -2 * P10.N)    
      //                    + 1*(+P0.P10 * -C.N * -2 * P10.N)    
      //                    + 1*(- C.P10 * P0.N * -2 * P10.N)    
      //                    + 1*(- C.P10 * -C.N * -2 * P10.N)    
      //                    + 1*(+P0.N * P0.N * P10.N * P10.N)   
      //                    + 2*(+P0.N * -C.N * P10.N * P10.N)   
      //                    + 1*(- C.N * -C.N * P10.N * P10.N) ) ) 

      // locals for Quadratic solution
      double     dACoef[3] ;
      ULONG      lNumSolves ;
      double     adSols[2] ;

      // coefficient locals
      SmPoint3d sP0, sP1, sP10, sP ;               // pre rotated-projected Box edge EndPoints, ChordVector, and EdgePoint
      SmVector3d sN(*cpProjUnitVector) ;           // rotation unit-dir vector
      SmVector3d sC(*cpProjPoint) ;                // rotation point
      SmVector3d sX(*cpAuxData) ;                  // unit-vec perp to sN defining the common plane spanned by sN and sX
      SmVector3d sA0(rProjectedBox.GetBasis(0)) ;  // new PseudoBox 1st basis vector
      SmVector3d sA1(rProjectedBox.GetBasis(1)) ;  // new PseudoBox 2nd basis vector
      //double     dCN  = sC.Dot(sN) ;
      double     dCC  = sC.Dot(sC) ;
      double     dA0x = sA0.Dot(sX) ; // Let A be A0=[a0x*X + a0n*N] or A1=[a1x*X + a1n*N]
      double     dA0n = sA0.Dot(sN) ;
      double     dA1x = sA1.Dot(sX) ;
      double     dA1n = sA1.Dot(sN) ;

      // for every ThisBox Edge
      //   aCorners is in this order:
      //   000, 100, 110, 010,      // Edges: [000 100] [100 110] [110 010] [010 000],  = Indices: [0 1] [1 2] [2 3] [3 0]   jj =     (ii+1)%4
      //   001, 101, 111, 011       //        [001 101] [101 111] [111 011] [011 001],             [4 5] [5 6] [6 7] [7 4]   jj = 4 + (ii+1)%4
      //                            //        [000 001] [100 101] [110 111] [010 011]              [0 4] [1 5] [2 6] [3 7]   jj =      ii+4
      // note the edge end index pattern:  EdgeStartIndex = ii%8, EdgeEndIndex = function of EdgeStartIndex
      for (ii=0,kk=0; ii<3; ii++)
        {
          for( jj=0;jj<4; jj++, kk++) 
            {
              sP0 = aCorners[kk%8] ;
              sP1 = aCorners[ii < 2 ? ii*4 + (jj+1)%4 : jj+4] ;
              sP10 = sP1 - sP0 ;

              // build quadratic term EdgePoint dependent constants
              double dP0P0  = sP0.Dot(sP0) ;
              double dP0P1  = sP0.Dot(sP1) ;
              double dP1P1  = sP1.Dot(sP1) ;

              double dP0P10 = sP0.Dot(sP10) ;
              double dP1P10 = sP1.Dot(sP10) ;
              double dP0N   = sP0.Dot(sN) ;
              double dP1N   = sP1.Dot(sN) ; 
              double dP10N  = sP10.Dot(sN) ;
              double dP0C   = sP0.Dot(sC) ;
              double dP1C   = sP1.Dot(sC) ;
              double dP10C  = sP10.Dot(sC) ;
              double dCN    = sC.Dot(sN) ;

              double dAxSSCoef = (+  (+dP0P10 * dP0P10)             
                                  -2*(+dP0P10 * dP1P10)             
                                  +  (+dP1P10 * dP1P10)             
                                  +  (+dP0N * dP0N * dP10N * dP10N) 
                                  -2*(+dP0N * dP1N * dP10N * dP10N) 
                                  +  (+dP1N * dP1N * dP10N * dP10N) 
                                  +  (+dP0P10 * dP0N * -2 * dP10N)  
                                  -  (+dP1P10 * dP0N * -2 * dP10N)  
                                  -  (+dP0P10 * dP1N * -2 * dP10N)  
                                  +  (+dP1P10 * dP1N * -2 * dP10N) ) ;
              double dAxSCoef = (+2*(+dP0P10 * dP1P10)              
                                 -2*(+dP0P10 * dP0P10)              
                                 -2*(+dP0P10 * -dP10C)              
                                 +2*(+dP1P10 * -dP10C)              
                                 -2*(+dP0P10 * dP0N * -2 * dP10N)   
                                 +  (+dP1P10 * dP0N * -2 * dP10N)   
                                 -  ( -dP10C * dP0N * -2 * dP10N)   
                                 +  (+dP0P10 * dP1N * -2 * dP10N)   
                                 +  ( -dP10C * dP1N * -2 * dP10N)   
                                 -  (+dP0P10 * -dCN * -2 * dP10N)   
                                 +  (+dP1P10 * -dCN * -2 * dP10N)   
                                 -2*(+dP0N * dP0N * dP10N * dP10N)  
                                 +2*(+dP1N * dP0N * dP10N * dP10N)  
                                 -2*(+dP0N * -dCN * dP10N * dP10N)  
                                 +2*(+dP1N * -dCN * dP10N * dP10N) ) ;
              double dAxConst = (+1*(+dP0P10 * dP0P10)               
                                 + 2*(+dP0P10 * -dP10C)              
                                 + 1*(+-dP10C * -dP10C)              
                                 + 1*(+dP0P10 * dP0N * -2 * dP10N)   
                                 + 1*(+dP0P10 * -dCN * -2 * dP10N)   
                                 + 1*(+-dP10C * dP0N * -2 * dP10N)   
                                 + 1*(+-dP10C * -dCN * -2 * dP10N)   
                                 + 1*(+dP0N * dP0N * dP10N * dP10N)  
                                 + 2*(+dP0N * -dCN * dP10N * dP10N)  
                                 + 1*(+-dCN * -dCN * dP10N * dP10N) ) ;

              double dAnSSCoef = (+1*(dP0P0 + dP0N * -dP0N)  
                                  -2*(dP0P1 + dP1N * -dP0N)   
                                  +1*(dP1P1 + dP1N * -dP1N) ) ;
              double dAnSCoef = (-2*(dP0P0 + dP0N * -dP0N)  
                                 +2*(dP0P1 + dP1N * -dP0N)  
                                 -2*(-dP0C + dP0N *   dCN)  
                                 +2*(-dP1C + dP1N *   dCN) ) ;

              double dAnConst = (+1*(dP0P0 + dP0N * -dP0N)  
                                 +2*(-dP0C + dP0N *   dCN)  
                                 +1*(  dCC + -dCN *   dCN) ) ;

              // for both basis vectors
              for(ll=0;ll<2;ll++)
                {
                  double dAx = ll == 0 ? dA0x : dA1x ;
                  double dAn = ll == 0 ? dA0n : dA1n ;

                  // build the quadratic coefficients
                  dACoef[0] = dAx*dAx*dAxSSCoef - dP10N * dP10N * dAn * dAn * dAnSSCoef ;
                  dACoef[1] = dAx*dAx*dAxSCoef  - dP10N * dP10N * dAn * dAn * dAnSCoef  ;
                  dACoef[2] = dAx*dAx*dAxConst  - dP10N * dP10N * dAn * dAn * dAnConst  ;

                  // solve for s in A*s*s + B*s*s + C = 0
                  double dScaledZero = SM_EFF_ZERO * (1.0 + smos_3Max(dACoef[0], dACoef[1], dACoef[2])) ;
                  smgu_SolveQuadraticEqn(dACoef, dScaledZero, lNumSolves, adSols) ;

                  // for every solution
                  for(mm=0;mm<lNumSolves;mm++)
                    {
                      // look for unique solutions between 0 and 1
                      if(   SM_IS_BETWEEN_TO_TOL(adSols[mm], 0.0, 1.0, SM_EFF_ZERO)
                         && (mm == 0 || !SM_IS_ZERO_TO_TOL(adSols[0] - adSols[1], SM_EFF_ZERO)))
                        {
                          // evaluate 3d Point P(s)
                          sP = (1 - adSols[mm])*sP0 + (adSols[mm])*sP1 ;

                          // Project EdgePoint to Plane and add to outputBox
                          sP = sP.ProjectPointToPlane(*cpProjPoint, *cpProjUnitVector);
                          rProjectedBox.AddPoint3d(sP);

                        } // end Edge has an internal maximum point check
                    } // end iter every solution for s, a line parameter where rotated-projected edge is a maxima in the new pseudoBox basis vectors
                } // end iter both newPseudoBox basis vectors
            } // end iter jj of every old pseudo box edge
        } // end iter ii of every old pseudo box edge
    } // end case SM_PT_ROTATE, need to account for pseudo box edge maximas check

  // all done
  return(SM_SUCCESS);

} // end SmPseudoBox::ProjectToPlane
