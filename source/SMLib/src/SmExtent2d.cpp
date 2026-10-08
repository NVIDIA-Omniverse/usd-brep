// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************//**
* FILE NAME --- SmExtent2d.cpp
* PURPOSE: Source file for implementation of SmExtent2d methods.
**********************************************************************/

#include "StdAfx.h"

#include <SmExtent2d.h>
#include <SmExtent1d.h>
#include <SmGraphicsExtern.h>
#include <SmExtent3d.h>
#include <SmAssertArray.h>
#include <SmContext.h>

#include <SmSurface.h>    // for Draw() method
#include <SmLine.h>       // for Draw() method
#include <SmCrvOnSurf.h>  // for Draw() method

/*******************************************************************//**
PURPOSE: Constructor which takes minimum and maximum 2D points.

NOTES: 
***********************************************************************/
SmExtent2d::SmExtent2d
  (const SmPoint2d & rMin, 
   const SmPoint2d & rMax)
{
  m_vMin = rMin ;
  m_vMax = rMax ;
  
  if(m_vMin.x > m_vMax.x) { SM_SWAP(double, m_vMin.x, m_vMax.x) ; } 
  if(m_vMin.y > m_vMax.y) { SM_SWAP(double, m_vMin.y, m_vMax.y) ; } 
} // end corner points constructor

/*******************************************************************//**
PURPOSE: 2d -> 2d Copy constructor.

NOTES: 
***********************************************************************/
SmExtent2d::SmExtent2d(const SmExtent2d & crOriginal) 
{  SM_ASSERT_DEFINED(&crOriginal) ; 
   m_vMin = crOriginal.m_vMin;
   m_vMax = crOriginal.m_vMax;
} // end copy constructor

/*******************************************************************//**
PURPOSE: 3d -> 2d copy constructor

NOTES: 
***********************************************************************/
SmExtent2d::SmExtent2d(const SmExtent3d & crExtent3d)
{ m_vMin = SmPoint2d(crExtent3d.GetMin().x,crExtent3d.GetMin().y);
  m_vMax = SmPoint2d(crExtent3d.GetMax().x,crExtent3d.GetMax().y);
}

/*******************************************************************//**
PURPOSE: Equality operator 

NOTES: 
***********************************************************************/
SmBoolean SmExtent2d::operator==
  (const SmExtent2d& crOther) 
 const
{
  if (   SM_ARE_SAME(m_vMin.x, crOther.m_vMin.x)
      && SM_ARE_SAME(m_vMin.y, crOther.m_vMin.y)
                                             
      && SM_ARE_SAME(m_vMax.x, crOther.m_vMax.x)
      && SM_ARE_SAME(m_vMax.y, crOther.m_vMax.y) )
    {
      return TRUE;
    }
  return FALSE;

} // end SmExtent2d::operator==

/*******************************************************************//**
PURPOSE: Modify the 2D extent such that it covers the given point.
     
NOTES: It may expand the extent if the point is not already inside of the
     extent.
***********************************************************************/
void SmExtent2d::AddPoint2d(const SmPoint2d & rPoint)
{ 
  if (rPoint.x < m_vMin.x) m_vMin.x = rPoint.x;
  if (rPoint.y < m_vMin.y) m_vMin.y = rPoint.y;
  if (rPoint.x > m_vMax.x) m_vMax.x = rPoint.x;
  if (rPoint.y > m_vMax.y) m_vMax.y = rPoint.y;
} // end SmExtent2d::AddPoint2d

/*******************************************************************//**
PURPOSE: Set the 2D extent from a given 2d extent

NOTES: 
***********************************************************************/
SmStatus SmExtent2d::Set
  (const SmExtent2d & crExt2d)
{ 
  m_vMin.x = crExt2d.m_vMin.x ;
  m_vMin.y = crExt2d.m_vMin.y ;
  m_vMax.x = crExt2d.m_vMax.x ;
  m_vMax.y = crExt2d.m_vMax.y ;

  return SM_SUCCESS ;

} // end SmExtent2d::Set

/*******************************************************************//**
PURPOSE: Set the minimum and maxumum points of the 2D extent.

NOTES: 
***********************************************************************/
SmStatus SmExtent2d::SetMinMax
  (const SmPoint2d & rMin, 
   const SmPoint2d & rMax)
{ 
  SmStatus sRet = SM_SUCCESS;
  if(   rMin.x > rMax.x 
     || rMin.y > rMax.y) 
    { SE(SM_ERR_INVALID_INPUT);
      sRet = SM_ERR_INVALID_INPUT; 
    } 
  else { m_vMin = rMin; 
         m_vMax = rMax; 
       } 
  return sRet;
} // end SmExtent2d::SetMinMax

/*******************************************************************//**
PURPOSE: Set the u-interval.

NOTES: 
***********************************************************************/
SmStatus SmExtent2d::SetUInterval
  ( const SmExtent1d & crUIvl )
{ 
  SmBoolean bOK = TRUE;
  
#ifdef SM_DEBUG_CODE 
  bOK = SM_ASSERT_VALID_NO_STREAM(&crUIvl) ;
#endif // SM_DEBUG_CODE

 if ( bOK ) 
   { 
     m_vMin.x = crUIvl.GetMin(); 
     m_vMax.x = crUIvl.GetMax();
     return(SM_SUCCESS) ; 
   } 
 else                              
   { SE(SM_ERR_INVALID_INPUT);
     return(SM_ERR_INVALID_INPUT) ; 
   } 

} // end SmExtent2d::SetUInterval

/*******************************************************************//**
PURPOSE: Set the v-interval.

NOTES: 
***********************************************************************/
SmStatus SmExtent2d::SetVInterval ( const SmExtent1d & crVIvl )
{ 
  SmBoolean bOK = TRUE;

#ifdef SM_DEBUG_CODE 
  bOK = SM_ASSERT_VALID_NO_STREAM(&crVIvl) ;
#endif // SM_DEBUG_CODE

  if ( bOK ) 
    { 
      m_vMin.y = crVIvl.GetMin(); 
      m_vMax.y = crVIvl.GetMax();
      return(SM_SUCCESS) ; 
    } 
  else                              
    { SE(SM_ERR_INVALID_INPUT);
      return(SM_ERR_INVALID_INPUT) ; 
    } 

} // end SmExtent2d::SetVInterval

/*******************************************************************//**
PURPOSE: Set the minimum and maxumum points of the 2D extent.

NOTES: 
***********************************************************************/
SmStatus SmExtent2d::SetMinMax
  (double dMinX, 
   double dMinY,
   double dMaxX,
   double dMaxY)
{ 
  SmStatus sRet = SM_SUCCESS;
  if (   dMinX > dMaxX 
      || dMinY > dMaxY) { SE(SM_ERR_INVALID_INPUT);
                          sRet = SM_ERR_INVALID_INPUT; 
                        } 
  else                  { m_vMin.x = dMinX; 
                          m_vMin.y = dMinY; 
                          m_vMax.x = dMaxX;
                          m_vMax.y = dMaxY; 
                        } 
  return sRet;

} // end SmExtent2d::SetMinMax

/*******************************************************************//**
PURPOSE: Set the minimum u-value of the 2D extent.

NOTES: Return SM_ERR if dNewUMin > the max u value.
***********************************************************************/
SmStatus SmExtent2d::SetUMin( double dNewUMin )
{
  if ( dNewUMin > m_vMax.x )
      SER(SM_ERR_INVALID_INPUT);
  m_vMin.x = dNewUMin;
  return SM_SUCCESS;
}
/*******************************************************************//**
PURPOSE: Set the maximum u-value of the 2D extent.

NOTES: Return SM_ERR if dNewUMax < the min u value.
***********************************************************************/
SmStatus SmExtent2d::SetUMax( double dNewUMax )
{
  if ( dNewUMax < m_vMin.x )
      SER(SM_ERR_INVALID_INPUT);
  m_vMax.x = dNewUMax;
  return SM_SUCCESS;
}
/*******************************************************************//**
PURPOSE: Set the minimum v-value of the 2D extent.

NOTES: Return SM_ERR if dNewVMin > the max v value.
***********************************************************************/
SmStatus SmExtent2d::SetVMin( double dNewVMin )
{
  if ( dNewVMin > m_vMax.y )
      SER(SM_ERR_INVALID_INPUT);
  m_vMin.y = dNewVMin;
  return SM_SUCCESS;
}
/*******************************************************************//**
PURPOSE: Set the maximum v-value of the 2D extent.

NOTES: Return SM_ERR if dNewVMax < the min v value.
***********************************************************************/
SmStatus SmExtent2d::SetVMax( double dNewVMax )
{
  if ( dNewVMax < m_vMin.y )
      SER(SM_ERR_INVALID_INPUT);
  m_vMax.y = dNewVMax;
  return SM_SUCCESS;
}

/*******************************************************************//**
PURPOSE: Swap U and V intervals.

NOTES:
***********************************************************************/
void SmExtent2d::Transpose()
{ double dTmp ;
  dTmp = m_vMin.x ; m_vMin.x = m_vMin.y ; m_vMin.y = dTmp ;
  dTmp = m_vMax.x ; m_vMax.x = m_vMax.y ; m_vMax.y = dTmp ;
} 

/*******************************************************************//**
PURPOSE: Expand the extent by a value in all directions.

NOTES: dExpansion can be positive or negative
***********************************************************************/
SmExtent2d & SmExtent2d::ExpandAbsolute(double dExpansion)
{
  SM_ASSERT_DEFINED(this);

  m_vMin.x -= (!SM_IS_INFINITE(m_vMin.x)) ? dExpansion : 0.0 ;
  m_vMin.y -= (!SM_IS_INFINITE(m_vMin.y)) ? dExpansion : 0.0 ;
  m_vMax.x += (!SM_IS_INFINITE(m_vMax.x)) ? dExpansion : 0.0 ;
  m_vMax.y += (!SM_IS_INFINITE(m_vMax.y)) ? dExpansion : 0.0 ; 

  // all done
  return(*this) ;
}

/*******************************************************************//**
PURPOSE: Expand the extent by a 2d vector in all directions.

NOTES: dExpansion can be positive or negative
***********************************************************************/
SmExtent2d & SmExtent2d::ExpandAbsolute(const SmVector2d crExpansion)
{
  SM_ASSERT_DEFINED(this);

  m_vMin.x -= (!SM_IS_INFINITE(m_vMin.x)) ? crExpansion.x : 0.0 ;
  m_vMin.y -= (!SM_IS_INFINITE(m_vMin.y)) ? crExpansion.y : 0.0 ;
  m_vMax.x += (!SM_IS_INFINITE(m_vMax.x)) ? crExpansion.x : 0.0 ;
  m_vMax.y += (!SM_IS_INFINITE(m_vMax.y)) ? crExpansion.y : 0.0 ; 

  // all done
  return(*this) ;
}

/*******************************************************************//**
PURPOSE: Expand the extent by a relative factor in all directions.

NOTES: 
***********************************************************************/
SmExtent2d & SmExtent2d::ExpandRelative(double dExpansionFactor)
{
  SM_ASSERT_DEFINED(this);
  SmVector2d sExpVec( ( m_vMax - m_vMin ) * dExpansionFactor );

  m_vMin.x -= (!SM_IS_INFINITE(m_vMin.x)) ? sExpVec.x : 0.0 ;
  m_vMin.y -= (!SM_IS_INFINITE(m_vMin.y)) ? sExpVec.y : 0.0 ;
  m_vMax.x += (!SM_IS_INFINITE(m_vMax.x)) ? sExpVec.x : 0.0 ;
  m_vMax.y += (!SM_IS_INFINITE(m_vMax.y)) ? sExpVec.y : 0.0 ; 

  return(*this) ;

} // end SmExtent2d::ExpandRelative

/*******************************************************************//**
PURPOSE: Scale the extent.

NOTES: 
***********************************************************************/
SmExtent2d & SmExtent2d::Scale
 (double dScale,        // in : U dir scale factor
  double *pOptScaleV)   // in : V dir scale factor, NULL = use dScale, default:[NULL]
{
  double dScaleV = pOptScaleV ? *pOptScaleV : dScale ;

  SM_ASSERT_DEFINED(this);
  m_vMin.x *= (!SM_IS_INFINITE(m_vMin.x)) ? dScale : 1.0 ;
  m_vMax.x *= (!SM_IS_INFINITE(m_vMax.x)) ? dScale : 1.0 ;

  m_vMin.y *= (!SM_IS_INFINITE(m_vMin.y)) ? dScaleV : 1.0 ;
  m_vMax.y *= (!SM_IS_INFINITE(m_vMax.y)) ? dScaleV : 1.0 ;

  return(*this) ;

} // end SmExtent2d::Scale

/*******************************************************************//**
PURPOSE: Get the size of the extent.  It basically returns the diagonal
    to the extent.

NOTES: 
***********************************************************************/
SmPoint2d SmExtent2d::GetSize() const
{
  SM_ASSERT_DEFINED(this);
  return(SmPoint2d(m_vMax.x-m_vMin.x,m_vMax.y-m_vMin.y)) ;
}

/*******************************************************************//**
PURPOSE: Get the size of the extent in the x direction.  

NOTES: 
***********************************************************************/
double SmExtent2d::XLength() const
{
  SM_ASSERT_DEFINED(this);
  return(m_vMax.x - m_vMin.x) ;
}

/*******************************************************************//**
PURPOSE: Get the size of the extentin the y direction.

NOTES: 
***********************************************************************/
double SmExtent2d::YLength() const
{
  SM_ASSERT_DEFINED(this);
  return(m_vMax.y - m_vMin.y) ;
}

/*******************************************************************//**
PURPOSE: The Union produces a 2D extent which minimally contains each of
     the two input extents.  

NOTES: 
***********************************************************************/
void SmExtent2d::Union
 (const SmExtent2d & crOther,           // in : other domain to union
  SmExtent2d       & rResult)           // out: result = this union crOther
const
{
  SM_ASSERT_DEFINED(this);
  SM_ASSERT_DEFINED(&crOther);
  rResult.m_vMin.x = smos_Min ( m_vMin.x, crOther.m_vMin.x );
  rResult.m_vMax.x = smos_Max ( m_vMax.x, crOther.m_vMax.x );
  rResult.m_vMin.y = smos_Min ( m_vMin.y, crOther.m_vMin.y );
  rResult.m_vMax.y = smos_Max ( m_vMax.y, crOther.m_vMax.y );
} // end SmExtent2d::Union

/*******************************************************************//**
PURPOSE: Compute an extent which represents the intersection of 
    two existing non-disjoint extents.

NOTES: Disjoint extents return an error.
***********************************************************************/
SmStatus SmExtent2d::Intersect
  (const SmExtent2d & crOther,          // in : Other domain to intersect
   SmExtent2d       & rResult)          // out: result this XSect crOther
 const
{
  SM_ASSERT_DEFINED(this);
  SM_ASSERT_DEFINED(&crOther);
  SmStatus sRet = SM_SUCCESS;
  if (AreDisjoint(crOther)) 
    {
      SM_DBG_WARN(_T("Invalid input: attempting to intersect disjoint SmExtent2d's"));
      rResult.Init() ;
      sRet = SM_ERR_INVALID_INPUT;
    }
  else 
    {
      rResult.m_vMin.x = smos_Max ( m_vMin.x, crOther.m_vMin.x );
      rResult.m_vMax.x = smos_Min ( m_vMax.x, crOther.m_vMax.x );
      rResult.m_vMin.y = smos_Max ( m_vMin.y, crOther.m_vMin.y );
      rResult.m_vMax.y = smos_Min ( m_vMax.y, crOther.m_vMax.y );
    }
  return sRet;
} // end SmExtent2d::Intersect

/*******************************************************************//**
PURPOSE: Find the maximum possible distance squared between two points
    which are each in their respective extents.  

NOTES: If the extents are not disjoint, then 0.0 will be returned.
***********************************************************************/
double SmExtent2d::MaximumDistanceSquared(const SmExtent2d & crOther) const
{
  SM_ASSERT_DEFINED(this);
  SM_ASSERT_DEFINED(&crOther);
  SmExtent2d sUnion;
  Union(crOther,sUnion);
  SmVector2d sDistVec = sUnion.m_vMax - sUnion.m_vMin;
  return sDistVec.LengthSquared();
} // end SmExtent2d::MaximumDistanceSquared

/*******************************************************************//**
PURPOSE: Find the minimum possible distance squared between two points 
    which are each in their respective extents.  

NOTES: If the extents are not disjoint, then 0.0 will be returned.
***********************************************************************/
double SmExtent2d::MinimumDistanceSquared(const SmExtent2d & crOther) const
{
  SM_ASSERT_DEFINED(this);
  SM_ASSERT_DEFINED(&crOther);
  SmVector2d sDistVec(0.0,0.0);
  if      (m_vMin.x > crOther.m_vMax.x) sDistVec.x += (m_vMin.x - crOther.m_vMax.x);
  else if (m_vMax.x < crOther.m_vMin.x) sDistVec.x += (crOther.m_vMin.x - m_vMax.x);
                                               
  if      (m_vMin.y > crOther.m_vMax.y) sDistVec.y += (m_vMin.y - crOther.m_vMax.y);
  else if (m_vMax.y < crOther.m_vMin.y) sDistVec.y += (crOther.m_vMin.y - m_vMax.y);

  return sDistVec.LengthSquared();
} // end SmExtent2d::MinimumDistanceSquared

/*******************************************************************//**
PURPOSE: return the max distance between common extent boundaries  

NOTES: 1. returns max(Xmin-OtherXmin,XMax-OtherMax,YMin-OtherYMin,YMax-OtherYMax)
       2. Two extents are equal when return MaxBoundaryDist < SM_EFF_ZERO_PARAM
***********************************************************************/
double SmExtent2d::GetMaxBoundaryDist2d(const SmExtent2d & crOther) const
{
  SM_ASSERT_DEFINED(this);
  SM_ASSERT_DEFINED(&crOther) ;
  double dThisDist, dMaxDist = 0.0 ;
           
  dThisDist = smos_Fabs(m_vMin.x - crOther.m_vMin.x) ; if(dThisDist > dMaxDist) { dMaxDist = dThisDist ; }
  dThisDist = smos_Fabs(m_vMax.x - crOther.m_vMax.x) ; if(dThisDist > dMaxDist) { dMaxDist = dThisDist ; }
  dThisDist = smos_Fabs(m_vMin.y - crOther.m_vMin.y) ; if(dThisDist > dMaxDist) { dMaxDist = dThisDist ; }
  dThisDist = smos_Fabs(m_vMax.y - crOther.m_vMax.y) ; if(dThisDist > dMaxDist) { dMaxDist = dThisDist ; }

  return dMaxDist ;
} // SmExtent2d::GetMaxBoundaryDist2d

/*******************************************************************//**
PURPOSE: Determine if two extents are disjoint.  

NOTES: If they just touch they are not considered disjoint.
***********************************************************************/
SmBoolean SmExtent2d::AreDisjoint
  (const SmExtent2d & crOther,  // in : other Extent2d to compare 
   double             dTol)     // in : sXSectDist2d, max dist between unique pts in UV space
 const
{
  SM_ASSERT_DEFINED(this);
  SM_ASSERT_DEFINED(&crOther);
  SmBoolean bRet = FALSE;
  if      (m_vMin.x > crOther.m_vMax.x + dTol) bRet = TRUE;
  else if (m_vMin.y > crOther.m_vMax.y + dTol) bRet = TRUE;
  else if (m_vMax.x < crOther.m_vMin.x - dTol) bRet = TRUE;
  else if (m_vMax.y < crOther.m_vMin.y - dTol) bRet = TRUE;
  return bRet;

} // end SmExtent2d::AreDisjoint

/*******************************************************************//**
PURPOSE: Determine if two extents are equal to tolerance.  

NOTES: If all 4 boundaries are within tolerance of one another
                Then they are considered equal.
***********************************************************************/
SmBoolean SmExtent2d::AreEqual
  (const SmExtent2d & crOther,           // in : other Extent2d to compare
   double             dTol)              // in : sXSectDist2d, max dist between unique pts in UV space

 const
{
  SM_ASSERT_DEFINED(this);
  SM_ASSERT_DEFINED(&crOther);
  SmBoolean bRet = (   SM_ARE_SAME_TO_TOL(m_vMin.x, crOther.m_vMin.x, dTol) 
                    && SM_ARE_SAME_TO_TOL(m_vMin.y, crOther.m_vMin.y, dTol)
                    && SM_ARE_SAME_TO_TOL(m_vMax.x, crOther.m_vMax.x, dTol)
                    && SM_ARE_SAME_TO_TOL(m_vMax.y, crOther.m_vMax.y, dTol) );
  return bRet;

} // end SmExtent2d::AreEqual

/*******************************************************************//**
PURPOSE: Return TRUE when any dimension is zero or negative.   

NOTES: 
***********************************************************************/
SmBoolean SmExtent2d::IsDegenerate(double dTol) const
{
  SM_ASSERT_DEFINED(this);

  return(   m_vMin.x >= m_vMax.x - dTol
         || m_vMin.y >= m_vMax.y - dTol) ;

} // end SmExtent3d::IsDegenerate

/*******************************************************************//**
PURPOSE: Return TRUE when both dimensions are zero  

NOTES: 
***********************************************************************/
SmBoolean SmExtent2d::IsPoint(double dTol) const
{
  SM_ASSERT_DEFINED(this);

  SmBoolean bDegenU = SM_IS_ZERO_TO_TOL(m_vMin.x - m_vMax.x, dTol) ;
  SmBoolean bDegenV = SM_IS_ZERO_TO_TOL(m_vMin.y - m_vMax.y, dTol) ;

  return(bDegenU && bDegenV) ;

} // end SmExtent3d::IsPoint

/*******************************************************************//**
PURPOSE: Return TRUE when just one dimension is zero  

NOTES: 
***********************************************************************/
SmBoolean SmExtent2d::IsLine(double dTol) const
{
  SM_ASSERT_DEFINED(this);

  SmBoolean bDegenU = SM_IS_ZERO_TO_TOL(m_vMin.x - m_vMax.x, dTol) ;
  SmBoolean bDegenV = SM_IS_ZERO_TO_TOL(m_vMin.y - m_vMax.y, dTol) ;

  return(   (bDegenU || bDegenV)
         && (bDegenU != bDegenV)) ;

} // end SmExtent3d::IsLine

/*******************************************************************//**
PURPOSE: Return TRUE when any dimension extent is negative.   

NOTES: 
***********************************************************************/
SmBoolean SmExtent2d::HasNegativeArea() const
{
  SM_ASSERT(!this->IsInit());
  return(   m_vMin.x > m_vMax.x
         || m_vMin.y > m_vMax.y) ;

} // end SmExtent3d::HasNegativeArea

/*******************************************************************//**
PURPOSE: Return TRUE when dimensions are all set to init values

NOTES: 
***********************************************************************/
SmBoolean SmExtent2d::IsInit
  () 
 const
{
  return(   m_vMin.x ==  SM_BIG_DOUBLE
         && m_vMin.y ==  SM_BIG_DOUBLE
         && m_vMax.x == -SM_BIG_DOUBLE
         && m_vMax.y == -SM_BIG_DOUBLE) ;

} // end SmExtent2d::IsInit

/*******************************************************************//**
PURPOSE: Return TRUE when all boundaries are bound,
         no +/-SM_INFINITE_PARAMETER values.

NOTES: 
***********************************************************************/
SmBoolean SmExtent2d::IsBounded
 (SmBoundaryType eOptBndryType[2])   // in : opt ptr to an array [IB_Type_U, IB_TYPE_V]
                                     //      NULL to ignore, default:[NULL]                  
 const                                                                                       
{                                                                                            
  SmBoundaryType  eBndryType[2] ;                                                            
  SmBoundaryType *pBndryType = eOptBndryType ? eOptBndryType : eBndryType ;
  
  // classify U interval 
  pBndryType[0] =   (   SM_IS_INFINITE(m_vMax.x)
                     && SM_IS_INFINITE(m_vMin.x)) ? SM_BT_UNBOUNDED
                  : (   SM_IS_INFINITE(m_vMin.x)) ? SM_BT_UNBOUNDED_MIN
                  : (   SM_IS_INFINITE(m_vMax.x)) ? SM_BT_UNBOUNDED_MAX
                  :                                 SM_BT_BOUNDED ;


  // classify V interval 
  pBndryType[1] =   (   SM_IS_INFINITE(m_vMax.y)
                     && SM_IS_INFINITE(m_vMin.y)) ? SM_BT_UNBOUNDED
                  : (   SM_IS_INFINITE(m_vMin.y)) ? SM_BT_UNBOUNDED_MIN
                  : (   SM_IS_INFINITE(m_vMax.y)) ? SM_BT_UNBOUNDED_MAX
                  :                                 SM_BT_BOUNDED ;

  // all done - return bounded status
  return(   pBndryType[0] == SM_BT_BOUNDED
         && pBndryType[1] == SM_BT_BOUNDED ) ;                                   

} // end SmExtent2d::IsBounded

/*******************************************************************//**
PURPOSE: Return TRUE when any boundary is bound,
         any boundary values not equal to +/-SM_INFINITE_PARAMETER values.

NOTES: 
***********************************************************************/
SmBoolean SmExtent2d::AnyBounds
 (SmBoundaryType eOptBndryType[2])   // in : opt ptr to an array [BT_Type_U, BT_Type_V]
                                     //      NULL to ignore, default:[NULL]
 const
{
  SmBoundaryType  eBndryType[2] ;
  SmBoundaryType *pBndryType = eOptBndryType ? eOptBndryType : eBndryType ;

  IsBounded(pBndryType) ;

  return(   pBndryType[0] != SM_BT_UNBOUNDED
         || pBndryType[1] != SM_BT_UNBOUNDED ) ;

} // end SmExtent2d::AnyBounds

/*******************************************************************//**
PURPOSE: Determines if this extent is completely contained by crOther.
    In otherwords, is this a subset of crOther.

NOTES: 
***********************************************************************/
SmBoolean SmExtent2d::IsContainedBy
  (const SmExtent2d & crOther,
   double             dTol) 
 const
{
  SM_ASSERT_DEFINED(this);
  SM_ASSERT_DEFINED(&crOther);
  SmBoolean bRet = TRUE;
  if      (m_vMin.x < crOther.m_vMin.x - dTol) bRet = FALSE;
  else if (m_vMin.y < crOther.m_vMin.y - dTol) bRet = FALSE;
  else if (m_vMax.x > crOther.m_vMax.x + dTol) bRet = FALSE;
  else if (m_vMax.y > crOther.m_vMax.y + dTol) bRet = FALSE;
  return bRet;

} // end SmExtent2d::IsContainedBy

/*******************************************************************//**
PURPOSE: Determine if the 2D extent contains a 2D point.  

NOTES: The tolerance
   is used to allow points which are very close to the boundary but outside
   to be considered inside of the extent. 
***********************************************************************/
SmBoolean SmExtent2d::ContainsPoint2d
  (const SmPoint2d & rPoint,             // in : point to check
   double            dTol,               // in : max allowed 2d dist between unique points
   double          * pOptMinOutsideDist) // out: for outside points - MinDist to Extent, for inside pts = 0.0, NULL to ignore, default:[NULL]
  const
{ 
  SM_ASSERT_DEFINED(this);
  SmBoolean bRet = TRUE;

  // when asked - calc distance to Extent for outside points
  if(pOptMinOutsideDist)
    {
      double dDistSquared = 0.0 ; 
      if     (rPoint.x + dTol < m_vMin.x) { bRet = FALSE ; dDistSquared += (m_vMin.x - rPoint.x) * (m_vMin.x - rPoint.x) ; }
      else if(rPoint.x - dTol > m_vMax.x) { bRet = FALSE ; dDistSquared += (rPoint.x - m_vMax.x) * (rPoint.x - m_vMax.x) ; }
      if     (rPoint.y + dTol < m_vMin.y) { bRet = FALSE ; dDistSquared += (m_vMin.y - rPoint.y) * (m_vMin.y - rPoint.y) ; }
      else if(rPoint.y - dTol > m_vMax.y) { bRet = FALSE ; dDistSquared += (rPoint.y - m_vMax.y) * (rPoint.y - m_vMax.y) ; }
      *pOptMinOutsideDist = smos_Sqrt(dDistSquared) ;
    }
  else // don't calc dist
    {
      if      (rPoint.x + dTol < m_vMin.x) { bRet = FALSE ; }
      else if (rPoint.x - dTol > m_vMax.x) { bRet = FALSE ; }
      else if (rPoint.y + dTol < m_vMin.y) { bRet = FALSE ; }
      else if (rPoint.y - dTol > m_vMax.y) { bRet = FALSE ; }
    }

  // all done
  return bRet;

} // end SmExtent2d::ContainsPoint2d

/*******************************************************************//**
PURPOSE: Determine if the 2D extent contains a 2D point
   within a relative tolerance.

NOTES: 
***********************************************************************/
SmBoolean SmExtent2d::ContainsPoint2dRelative( const SmPoint2d & crPoint, double dRelativeTol ) const
{
  SM_ASSERT_DEFINED(this);

  if ( ! GetUInterval().ContainsValueRelative( crPoint.x, dRelativeTol ) )
    { return FALSE; }

  if ( ! GetVInterval().ContainsValueRelative( crPoint.y, dRelativeTol ) )
    { return FALSE; }

  return TRUE;

} // end SmExtent2d::ContainsPoint2dRelative

/*******************************************************************//**
PURPOSE: Classify Point with respect to each orthogonal 1d extent.

NOTES: 
***********************************************************************/
void SmExtent2d::ClassifyPoint2d
 (const SmPoint2d   & rPoint,       // in : target point
  SmExtentPointType & rExtentUType, // out: Classification of point for U extent
  SmExtentPointType & rExtentVType, // out: Classification of point for V extent
  double              dTolU,        // in : max allowed param U distance to count as being on a U boundary
                                    //      default:[SM_EFF_ZERO]
  double            * pOptTolV)     // in : max allowed param V distance to count as being on a V boundary
                                    //      NULL= dTolV = dTolU, default:NULL
 const
{
  SM_ASSERT_DEFINED(this);
  double dTolV = pOptTolV ? *pOptTolV : dTolU ;
  SM_ASSERT_MSG(dTolU < XLength()/2.0, _T("SmExtent2d::ClassifyPoint2d - Warning large Tol: dTolU > Extent2d::ULength/2.0")) ; 
  SM_ASSERT_MSG(dTolV < YLength()/2.0, _T("SmExtent2d::ClassifyPoint2d - Warning large Tol: dTolV > Extent2d::VLength/2.0")) ; 

  rExtentUType =    (smos_Fabs(rPoint.x-m_vMin.x) <= dTolU) &&
                    (smos_Fabs(rPoint.x-m_vMax.x) <= dTolU) ? SM_EP_BOTH
                  : (smos_Fabs(rPoint.x-m_vMin.x) <= dTolU) ? SM_EP_START
                  : (smos_Fabs(rPoint.x-m_vMax.x) <= dTolU) ? SM_EP_END
                  : (rPoint.x-m_vMin.x >= 0.0) &&
                    (m_vMax.x-rPoint.x >= 0.0)             ? SM_EP_INSIDE
                  : SM_EP_OUTSIDE ;

  rExtentVType =    (smos_Fabs(rPoint.y-m_vMin.y) <= dTolV) &&
                    (smos_Fabs(rPoint.y-m_vMax.y) <= dTolV) ? SM_EP_BOTH
                  : (smos_Fabs(rPoint.y-m_vMin.y) <= dTolV) ? SM_EP_START
                  : (smos_Fabs(rPoint.y-m_vMax.y) <= dTolV) ? SM_EP_END
                  : (rPoint.y-m_vMin.y >= 0.0) &&
                    (m_vMax.y-rPoint.y >= 0.0)             ? SM_EP_INSIDE
                  : SM_EP_OUTSIDE ;

} // end SmExtent2d::ClassifyPoint2d

/*******************************************************************//**
PURPOSE: Return SmSurfaceDirType classification for given UV Pt and Dir

NOTES: returns oneof: SM_SD_UNINIT   
                      SM_SD_IN     // SrfPt is 'in' Face UVDomain
                      SM_SD_ON     // SrfDir at SrfPt 'On' FaceBndry points inside surface
                      SM_SD_OUT    // SrfPt is 'out' Face UVDomain
                      SM_SD_LAMINA // SrfDir on a Lamina SrfBndry points out of the Surface
                      SM_SD_SEAM   // SrfDir on a Seam SrfBndry points out of the Surface
                      SM_SD_POLE   // SrfDir on a Pole SrfBndry points out of the Surface

***********************************************************************/
void SmExtent2d::ClassifyPointDir2d     
 (SmPoint2d          sUV,               // in : Tgt UVPoint
  SmVector2d         sDir,              // in : Tgt UVDir at UVPoint
  SmExtentPointType &reExtentUType,     // in : oneof:[SM_EP_START, SM_EP_INSIDE, SM_EP_END, SM_EP_BOTH, SM_EP_OUTSIDE]
  SmExtentPointType &reExtentVType,     // in : oneof:[SM_EP_START, SM_EP_INSIDE, SM_EP_END, SM_EP_BOTH, SM_EP_OUTSIDE]
  SmExtentDirType   &reExtentDir,       // in : oneof:[SM_ED_GOING_IN,  SM_ED_GOING_TAN_POSU, SM_ED_GOING_TAN_NEGU,  
                                        //             SM_ED_GOING_OUT, SM_ED_GOING_TAN_POSV, SM_ED_GOING_TAN_NEGV]
  double             dTolU,             // in : max allowed param U distance to count as being on a U boundary
                                        //      default:[SM_EFF_ZERO]
  double           * pOptTolV)          // in : max allowed param V distance to count as being on a V boundary
                                        //      NULL= dTolV = dTolU, default:NULL
 const
{
  // locals
  double     dAngRad = 0.0;
  double     dRadTol = SmTol::GetAngTolRad() ;
  SmVector2d sU(1,0) ; 
  sUV.CCWAngleBetween(sU, dAngRad) ;

  SM_REF1(sDir);

  // classify the sUV against sUVDomain
  ClassifyPoint2d(sUV, reExtentUType, reExtentVType, dTolU, pOptTolV) ;

  // Classify the SrfDir at sUV
  reExtentDir =  // inside case
                 (reExtentUType == SM_EP_INSIDE  && reExtentVType == SM_EP_INSIDE)  ? SM_ED_GOING_IN

                 // outside case
               : (reExtentUType == SM_EP_OUTSIDE || reExtentVType == SM_EP_OUTSIDE) ? SM_ED_GOING_OUT

               // min/min corner case
               : (reExtentUType == SM_EP_START   && reExtentVType == SM_EP_START)  
                 ? (  (   smos_Fabs(  0.0   - dAngRad) < dRadTol
                       || smos_Fabs( SM_2PI - dAngRad) < dRadTol) ? SM_ED_GOING_TAN_POSU
                    : (smos_Fabs(  SM_PI/2 - dAngRad) < dRadTol)  ? SM_ED_GOING_TAN_POSV
                    : (dAngRad < SM_PI/2)                         ? SM_ED_GOING_IN
                    :                                              SM_ED_GOING_OUT) 

               // min/max corner case
               : (reExtentUType == SM_EP_START   && reExtentVType == SM_EP_END)
                 ? (  (   smos_Fabs(   0.0    - dAngRad) < dRadTol
                       || smos_Fabs(  SM_2PI  - dAngRad) < dRadTol) ? SM_ED_GOING_TAN_POSU
                    : (   smos_Fabs(3*SM_PI/2 - dAngRad) < dRadTol) ? SM_ED_GOING_TAN_NEGV
                    : (dAngRad > 3*SM_PI/2)                         ? SM_ED_GOING_IN
                    :                                                 SM_ED_GOING_OUT) 
               
               // max/min corner case 
               : (reExtentUType == SM_EP_END     && reExtentVType == SM_EP_START) 
                 ? (  (   smos_Fabs( SM_PI  - dAngRad) < dRadTol) ? SM_ED_GOING_TAN_NEGU
                    : (smos_Fabs(  SM_PI/2 - dAngRad) < dRadTol)  ? SM_ED_GOING_TAN_POSV
                    : (dAngRad > SM_PI/2 && dAngRad < SM_PI)      ? SM_ED_GOING_IN
                    :                                               SM_ED_GOING_OUT) 
               
               // max/max corner case
               : (reExtentUType == SM_EP_END     && reExtentVType == SM_EP_END)                    
                 ? (  (smos_Fabs(  SM_PI   - dAngRad) < dRadTol) ? SM_ED_GOING_TAN_NEGU
                    : (smos_Fabs(3*SM_PI/2 - dAngRad) < dRadTol) ? SM_ED_GOING_TAN_NEGV
                    : (dAngRad > SM_PI && dAngRad < 3*SM_PI/2)   ? SM_ED_GOING_IN
                    :                                              SM_ED_GOING_OUT) 
               
               // MinU side case
               : (reExtentUType == SM_EP_START   && reExtentVType == SM_EP_INSIDE)  
                 ? (  (smos_Fabs(  SM_PI/2 - dAngRad) < dRadTol)  ? SM_ED_GOING_TAN_POSV
                    : (smos_Fabs(3*SM_PI/2 - dAngRad) < dRadTol)  ? SM_ED_GOING_TAN_NEGV
                    : (dAngRad < SM_PI/2 || dAngRad > 3*SM_PI/2)  ? SM_ED_GOING_IN
                    :                                               SM_ED_GOING_OUT)
               
               // MinV side case 
               : (reExtentUType == SM_EP_INSIDE  && reExtentVType == SM_EP_START)                  
                 ? (  (   smos_Fabs(  0.0   - dAngRad) < dRadTol
                       || smos_Fabs( SM_2PI - dAngRad) < dRadTol) ? SM_ED_GOING_TAN_POSU
                    : (   smos_Fabs( SM_PI  - dAngRad) < dRadTol) ? SM_ED_GOING_TAN_NEGU
                    : (dAngRad < SM_PI)                           ? SM_ED_GOING_IN
                    :                                               SM_ED_GOING_OUT)
               
               // MaxV side case
               : (reExtentUType == SM_EP_INSIDE  && reExtentVType == SM_EP_END)                   
                 ? (  (   smos_Fabs(  0.0   - dAngRad) < dRadTol
                       || smos_Fabs( SM_2PI - dAngRad) < dRadTol) ? SM_ED_GOING_TAN_POSU
                    : (   smos_Fabs( SM_PI  - dAngRad) < dRadTol) ? SM_ED_GOING_TAN_NEGU
                    : (dAngRad > SM_PI)                           ? SM_ED_GOING_IN
                    :                                               SM_ED_GOING_OUT)
               
               // MaxU side case
               : (reExtentUType == SM_EP_END     && reExtentVType == SM_EP_INSIDE)                  
                 ? (  (smos_Fabs(  SM_PI/2 - dAngRad) < dRadTol) ? SM_ED_GOING_TAN_POSV
                    : (smos_Fabs(3*SM_PI/2 - dAngRad) < dRadTol) ? SM_ED_GOING_TAN_NEGV
                    : (dAngRad > SM_PI/2 || dAngRad < 3*SM_PI/2) ? SM_ED_GOING_IN
                    :                                              SM_ED_GOING_OUT)

               // degen cases
               : (reExtentUType == SM_EP_BOTH  || reExtentVType == SM_EP_BOTH) ? SM_ED_NONE

               // default - unexpected cases
               : SM_ED_UNEXPECTED ;

} // end SmExtent2d::ClassifyPointDir2d

/*******************************************************************//**
PURPOSE: Determine if a box is touching another box at only a corner point

NOTES: 
***********************************************************************/
SmBoolean SmExtent2d::IsTouchingOnePointAt2DCorner
  (SmExtent2d & rOther,
   double       dTol)  
  const
{
    SM_ASSERT_DEFINED(this);
    double dSize = GetSize().Length();
    double dOtherSize = rOther.GetSize().Length();
    if(dTol < SM_EFF_ZERO*(smos_Min(dSize,dOtherSize)))
      { dTol = SM_EFF_ZERO*(smos_Min(dSize,dOtherSize)) ; }
    SmPoint2d sMin = GetMin();
    SmPoint2d sMax = GetMax();
    SmPoint2d sOtherMin = rOther.GetMin();
    SmPoint2d sOtherMax = rOther.GetMax();

    if (   smos_Fabs(sMin.x - sOtherMax.x) < dTol
        || smos_Fabs(sMax.x - sOtherMin.x) < dTol) 
      {
        if (   smos_Fabs(sMin.y - sOtherMax.y) < dTol
            || smos_Fabs(sMax.y - sOtherMin.y) < dTol) 
          {
            return TRUE;
          }
      }
    return FALSE;

} // end SmExtent2d::IsTouchingOnePointAt2DCorner

/*******************************************************************//**
PURPOSE: Determine if the 2D point is on the extent boundary 
            to within the given tolerance value.

NOTES: 
***********************************************************************/
SmBoolean SmExtent2d::IsPoint2dOnBoundary
  (const SmPoint2d & crPoint,    // in : point to test
   double            dTol,       // in : max allowed distance between point and boundary, default:[SM_EFF_ZERO]
   SmVector2d      * pOptBiNorm) // out: UVVector pointing to interior from 1st OnBoundary found
                                 //      NULL to ignore. default:[NULL]
 const
{
  SM_ASSERT_DEFINED(this);
  SmBoolean bRet = FALSE;
  double du=0.0, dv=0.0 ;
  if      (smos_Fabs(crPoint.x-m_vMin.x) <= dTol) { bRet = TRUE; du= 1.0 ; }
  else if (smos_Fabs(crPoint.x-m_vMax.x) <= dTol) { bRet = TRUE; du=-1.0 ; }
  if      (smos_Fabs(crPoint.y-m_vMin.y) <= dTol) { bRet = TRUE; dv= 1.0 ; }
  else if (smos_Fabs(crPoint.y-m_vMax.y) <= dTol) { bRet = TRUE; dv=-1.0 ; }
  if(pOptBiNorm) { pOptBiNorm->Set(du,dv) ; }
  return bRet;

} // end SmExtent2d::IsPoint2dOnBoundary

/*******************************************************************//**
PURPOSE: Return bit array marking all the boundaries that are within tol of TestPoint

RETURNS: SM_SS_NONE or orof:[SM_SS_UMIN SM_SS_VMIN SM_SS_UMAX SM_SS_VMAX]
 where:    SM_SS_NONE 0      // crPoint is not on any boundaries
           SM_SS_UMIN 1      // when set, crPoint is on the u = UMin boundary
           SM_SS_VMIN 2      // when set, crPoint is on the v = VMin boundary
           SM_SS_UMAX 4      // when set, crPoint is on the u = UMax boundary
           SM_SS_VMAX 8      // when set, crPoint is on the v = VMax boundary
***********************************************************************/
ULONG SmExtent2d::GetPoint2dBoundaries // rtn: SM_SS_NONE or orof:[SM_SS_UMIN SM_SS_VMIN SM_SS_UMAX SM_SS_VMAX]
  (const SmPoint2d & crPoint,          // in : point to test
   double dTol)                        // in : max allowed distance between point and boundary, default:[SM_EFF_ZERO]
 const
{
  SM_ASSERT_DEFINED(this) ;

  // init return value
  ULONG lBndries = SM_SS_NONE ;

  // check each boundary
  if (smos_Fabs(crPoint.x-m_vMin.x) <= dTol) { lBndries |= SM_SS_UMIN ; }
  if (smos_Fabs(crPoint.y-m_vMin.y) <= dTol) { lBndries |= SM_SS_VMIN ; }
  if (smos_Fabs(crPoint.x-m_vMax.x) <= dTol) { lBndries |= SM_SS_UMAX ; }
  if (smos_Fabs(crPoint.y-m_vMax.y) <= dTol) { lBndries |= SM_SS_VMAX ; }

  // all done
  return lBndries;

} // end SmExtent2d::GetPoint2dBoundaries

/*******************************************************************//**
PURPOSE: Return AngleSector centered on input point2d that's inside the Extent

RETURNS: When UV Point is in the interior of the extent return [0 360] else
         when UV point classifies as:
 Outside Domain rtn [init] = [SM_BIG_DOUBLE -SM_BIG_DOUBLE]
 Inside Domain and
   SM_SS_NONE rtn [  0 360],                    
   SM_SS_UMIN rtn [-90  90], SM_SS_UMIN & SM_SS_VMIN rtn [  0  90]
   SM_SS_VMIN rtn [  0 180], SM_SS_VMIN & SM_SS_UMAX rtn [ 90 180]
   SM_SS_UMAX rtn [ 90 270], SM_SS_UMAX & SM_SS_VMAX rtn [180 270]
   SM_SS_VMAX rtn [180 360], SM_SS_VMAX & SM_SS_UMIN rtn [270 360]
***********************************************************************/
SmExtent1d SmExtent2d::GetInsideSectorAtPoint2d
 (SmPoint2d & rUV,    // in : Point2d to query
  SmTol2d     sTol2d) // in : UVDomain tolerance, default:[SM_EFF_ZERO_PARAM]
 const     
{
  // init return
  SmExtent1d sRtnIvl ; 

  // classify UV Point to Extent2d interior
  SmBoolean bInside = ContainsPoint2d(rUV, sTol2d) ; 

  // low work - UVPoint outside of domain
  if(bInside == FALSE)
    { sRtnIvl.Init() ; 
      return sRtnIvl ;
    }

  // classify UV point to Extent2d boundaries
  ULONG sBndrys = GetPoint2dBoundaries(rUV, sTol2d) ; 

  // Translate classifications to Intervals and return
  if(SM_SS_NONE)                                { sRtnIvl.SetMinMax(  0, 360) ; }
  else if(sBndrys == (SM_SS_UMIN | SM_SS_VMIN)) { sRtnIvl.SetMinMax(  0,  90) ; }
  else if(sBndrys == (SM_SS_VMIN | SM_SS_UMAX)) { sRtnIvl.SetMinMax( 90, 180) ; }
  else if(sBndrys == (SM_SS_UMAX | SM_SS_VMAX)) { sRtnIvl.SetMinMax(180, 270) ; }
  else if(sBndrys == (SM_SS_VMAX | SM_SS_UMIN)) { sRtnIvl.SetMinMax(270, 360) ; }
  else if(sBndrys == (SM_SS_UMIN))              { sRtnIvl.SetMinMax(-90,  90) ; }
  else if(sBndrys == (SM_SS_VMIN))              { sRtnIvl.SetMinMax(  0, 180) ; }
  else if(sBndrys == (SM_SS_UMAX))              { sRtnIvl.SetMinMax( 90, 270) ; }
  else if(sBndrys == (SM_SS_VMAX))              { sRtnIvl.SetMinMax(180, 360) ; }

  // all done
  return(sRtnIvl) ;

} // end SmExtent2d::GetInsideSectorAtPoint2d

/*******************************************************************//**
PURPOSE: Determine if the 2D extent contains a 2D point when
            either or both U and V are part of a periodic space.  

NOTES: The tolerance
   is used to allow points which are very close to the boundary but outside
   to be considered inside of the extent. 
***********************************************************************/

SmBoolean SmExtent2d::ContainsPeriodicPoint2d
  (const SmPoint2d & rPoint,         // in : target point
   double            dUPeriod,       // in : length of U direction period or 0 = not periodic
   double            dVPeriod,       // in : length of V direction period or 0 = not periodic
   double            dTolIgnore,     // in : max distance outside to be considered in.
   SmPoint2d       * pPeriodicPoint) // out: optional Point value adjusted to the 1st period
  const
{
  // check input
  SM_ASSERT_DEFINED(this);
 // SmBoolean bRet = FALSE;

  // locals
  SmExtent1d sUIvl(m_vMin.x, m_vMax.x) ;
  SmExtent1d sVIvl(m_vMin.y, m_vMax.y) ;

  // Point is contained contained in both U or V intervals
  double dTemp1, dTemp2;
  SmBoolean bUContained = sUIvl.ContainsPeriodicValue(
      rPoint.x, dUPeriod, pPeriodicPoint ? &pPeriodicPoint->x : NULL, &dTemp1, &dTemp2, dTolIgnore );
  SmBoolean bVContained = sVIvl.ContainsPeriodicValue(
      rPoint.y, dVPeriod, pPeriodicPoint ? &pPeriodicPoint->y : NULL, &dTemp1, &dTemp2, dTolIgnore );

  // all done
  return(bUContained && bVContained) ;

} // end SmExtent2d::ContainsPeriodicPoint2d

/*******************************************************************//**
PURPOSE: Determine if two intervals are disjoint on a periodic interval.

NOTES: They are not considered disjoint if they touch.
***********************************************************************/
SmBoolean SmExtent2d::AreDisjointPeriodic
  (const SmExtent2d & crOther,     // in : 2nd interval in same periodic space
   double dUPeriod,                // in : length of periodic U space or 0.0 = Not periodic
   double dVPeriod)                // in : length of periodic V space or 0.0 = Not periodic
 const
{
  // check input
  SM_ASSERT_DEFINED(this);
  SM_ASSERT_DEFINED(&crOther);
//  SmBoolean bRet = FALSE;

  // locals
  SmExtent1d sUIvl(m_vMin.x, m_vMax.x) ;
  SmExtent1d sVIvl(m_vMin.y, m_vMax.y) ;
  SmExtent1d sOtherUIvl(crOther.m_vMin.x, crOther.m_vMax.x) ;
  SmExtent1d sOtherVIvl(crOther.m_vMin.y, crOther.m_vMax.y) ;

  // domains are disjoint when either U or V intervals are disjoint
  SmBoolean bUDisjoint = sUIvl.AreDisjointPeriodic(sOtherUIvl,dUPeriod) ;
  SmBoolean bVDisjoint = sVIvl.AreDisjointPeriodic(sOtherVIvl,dVPeriod) ;

  // all done
  return(bUDisjoint || bVDisjoint) ;

} // end SmExtent2d::AreDisjointPeriodic

/*******************************************************************//**
PURPOSE: Determine the intersection of two periodic intervals which are not 
    disjoint.  

NOTES: outputs 0, 1, or 2 intervals.
    returns SM_ERR when output interval count == 0 else returns SM_SUCCESS
   
***********************************************************************/
SmStatus SmExtent2d::IntersectPeriodic
  (const SmExtent2d & crOther,      // in : 2nd interval in same periodic space
   double dUPeriod,                 // in : length of periodic U space or 0.0 = Not periodic
   double dVPeriod,                 // in : length of periodic V space or 0.0 = Not periodic   
   SmTArray<SmExtent2d> & rResult,  // out: intersection intervals 
   SmBoolean bDontCrossBoundaries)   // in : TRUE = Split return intervals that span this interval boundaries
                                     //        ex: Intersect([0, 360] [-90 90]) returns [270 0] [0 90] 
                                     //      FALSE= Okay to return a single interval that spans this interval boundary as
                                     //        ex: Intersect([0, 360] [-90 90]) returns [-90 90]
  const                           
{
  SM_ASSERT_DEFINED(this);
  SM_ASSERT_DEFINED(&crOther);

  // init output
  SmStatus sRet = SM_SUCCESS;
  rResult.ReSet() ;

  // locals
  ULONG ii, jj ;
  SmExtent1d sUIvl(m_vMin.x, m_vMax.x) ;
  SmExtent1d sVIvl(m_vMin.y, m_vMax.y) ;
  SmExtent1d sOtherUIvl(crOther.m_vMin.x, crOther.m_vMax.x) ;
  SmExtent1d sOtherVIvl(crOther.m_vMin.y, crOther.m_vMax.y) ;
  SmTArray<SmExtent1d> sUXSects ;
  SmTArray<SmExtent1d> sVXSects ;

  // intersect U and V intervals
  sUIvl.IntersectPeriodic(sOtherUIvl, dUPeriod, sUXSects, bDontCrossBoundaries) ;
  sVIvl.IntersectPeriodic(sOtherVIvl, dVPeriod, sVXSects, bDontCrossBoundaries) ;

  // output 1 2d domain for every combination of 1d intersection intervals
  for(ii=0;ii<sUXSects.GetSize();ii++)
    {
      SmExtent1d rUIvl = sUXSects[ii] ;

      for(jj=0;jj<sVXSects.GetSize();jj++)
        {
          SmExtent1d rVIvl = sVXSects[jj] ;

          // build and output an intersection extent
          rResult.Add(SmExtent2d(rUIvl.GetMin(), rVIvl.GetMin(), 
                                 rUIvl.GetMax(), rVIvl.GetMax())) ;
        }
    } // end iter every U interval intersection

  // all done
  return sRet;

} // end SmExtent2d::IntersectPeriodic

/*******************************************************************//**
PURPOSE: Given two normalized parameters (0.0 to 1.0) evaluate the
extent and produce the corresponding point in the extent.  

NOTES: For example:
    (0.0,0.0) produces the minimum point; (1.0,0.0) produces the lower 
    rightmost corner; etc.
***********************************************************************/
SmPoint2d SmExtent2d::Evaluate(double dNormalizedX, double dNormalizedY) const
{
  SM_ASSERT_DEFINED(this) ;
  SM_ASSERT_BREAK(0.0 <= dNormalizedX && dNormalizedX <= 1.0);
  SM_ASSERT_BREAK(0.0 <= dNormalizedY && dNormalizedY <= 1.0);
  SmPoint2d sRet;

  sRet.x =   ( dNormalizedX == 0.0 ) ? m_vMin.x 
           : ( dNormalizedX == 1.0 ) ? m_vMax.x 
           : m_vMin.x + dNormalizedX * (m_vMax.x - m_vMin.x) ;
  sRet.y =   ( dNormalizedY == 0.0 ) ? m_vMin.y 
           : ( dNormalizedY == 1.0 ) ? m_vMax.y 
           : m_vMin.y + dNormalizedY * (m_vMax.y - m_vMin.y);

  return( ClampPoint2d(sRet) ) ;

} // SmExtent2d::Evaluate

/*******************************************************************//**
PURPOSE: Given two normalized parameters (0.0 to 1.0) evaluate the
extent and produce the corresponding point in the extent.  

NOTES: For example:
    (0.0,0.0) produces the minimum point; (1.0,0.0) produces the lower 
    rightmost corner; etc.
***********************************************************************/
double SmExtent2d::EvaluateU(double dNormalizedX) const
{
  SM_ASSERT_DEFINED(this) ;
  SM_ASSERT_BREAK(0.0 <= dNormalizedX && dNormalizedX <= 1.0);
  double dRet;

  dRet =   ( dNormalizedX == 0.0 ) ? m_vMin.x 
         : ( dNormalizedX == 1.0 ) ? m_vMax.x 
         : m_vMin.x + dNormalizedX * (m_vMax.x - m_vMin.x) ;
         
  return( SM_CLAMP_TO_INTERVAL(dRet, m_vMin.x, m_vMax.x) ) ;

} // SmExtent2d::EvaluateU

/*******************************************************************//**
PURPOSE: Given two normalized parameters (0.0 to 1.0) evaluate the
extent and produce the corresponding point in the extent.  

NOTES: For example:
    (0.0,0.0) produces the minimum point; (1.0,0.0) produces the lower 
    rightmost corner; etc.
***********************************************************************/
double SmExtent2d::EvaluateV(double dNormalizedY) const
{
  SM_ASSERT_DEFINED(this) ;
  SM_ASSERT_BREAK(0.0 <= dNormalizedY && dNormalizedY <= 1.0);
  double dRet;

  dRet =   ( dNormalizedY == 0.0 ) ? m_vMin.y 
         : ( dNormalizedY == 1.0 ) ? m_vMax.y 
         : m_vMin.y + dNormalizedY * (m_vMax.y - m_vMin.y);

  return( SM_CLAMP_TO_INTERVAL(dRet, m_vMin.y, m_vMax.y) ) ;

} // SmExtent2d::EvaluateV

/*******************************************************************//**
PURPOSE: Determine normalized parameter on the domain of the point in
 or on the boundary of the domain.

NOTES: 
***********************************************************************/
SmStatus SmExtent2d::Inversion
  (const SmPoint2d & crPoint,      // in : target point
   SmPoint2d & rNormalizedPoint,   // out: Normalized Point (coords from 0.0 to 1.0)
   double      dTolerance)         // in : max allowed distance between point and boundary
                                   //      default:[SM_EFF_ZERO]
  const
{
  SM_ASSERT_DEFINED(this);
  SmStatus  sRet = SM_SUCCESS;
  SmPoint2d sPnt = crPoint;

  // snap points to boundary
  if (IsPoint2dOnBoundary(sPnt,dTolerance)) 
    {
      sPnt = ClampPoint2d(sPnt);
    }

  // Error when point is not in extent
  if (!ContainsPoint2d(sPnt, dTolerance)) 
    {
      sRet = SM_ERR;
    }
  else // point is in extent
    {

      // get normalized coordinates
      double dNumX = sPnt.x   - m_vMin.x ;
      double dNumY = sPnt.y   - m_vMin.y ;
      double dDenX = m_vMax.x - m_vMin.x ;
      double dDenY = m_vMax.y - m_vMin.y ;

      // watch out for divide by zeros
      rNormalizedPoint.x =   !SM_IS_ZERO_TO_TOL(dDenX, SM_EFF_ZERO_SQRT) ? (dNumX / dDenX)
                           :  SM_IS_ZERO_TO_TOL(dNumX, SM_EFF_ZERO_SQRT) ? 1.0
                           : -1.0 ;

      rNormalizedPoint.y =   !SM_IS_ZERO_TO_TOL(dDenY, SM_EFF_ZERO_SQRT) ? (dNumY / dDenY)
                           :  SM_IS_ZERO_TO_TOL(dNumY, SM_EFF_ZERO_SQRT) ? 1.0
                           : -1.0 ;

      if(   rNormalizedPoint.x == -1.0
         || rNormalizedPoint.y == -1.0)
        { sRet = SM_ERR ; } 
      else
        {
          // snap points to boundary when needed
          SmExtent2d sNormDomain(0.0, 0.0, 1.0, 1.0);
          rNormalizedPoint = sNormDomain.ClampPoint2d(rNormalizedPoint);
        }
    }

  // all done
  return sRet;

} // end SmExtent2d::Inversion


/*******************************************************************//**
PURPOSE: If the point is outside of the extent, clamp it to the 
    nearest boundary of the extent. 

NOTES: 
***********************************************************************/
SmPoint2d SmExtent2d::ClampPoint2d(const SmPoint2d & rPoint) const
{ 
  SM_ASSERT_DEFINED(this) ;
  SmPoint2d sRet = rPoint;
  if (rPoint.x < m_vMin.x) sRet.x = m_vMin.x; 
  if (rPoint.y < m_vMin.y) sRet.y = m_vMin.y;
  if (rPoint.x > m_vMax.x) sRet.x = m_vMax.x;
  if (rPoint.y > m_vMax.y) sRet.y = m_vMax.y;
  return sRet;
} // end SmExtent2d::ClampPoint2d

/*******************************************************************//**
PURPOSE: Determine if a line segment endPoint lies within the extent,
     if not - return line segment parameter of lineSeg/ExtentBoundary
     intersection closest to StartPoint.

NOTES:
    Returns:
      IF the given end point is within the extent, return given line parameter
      ELSE return parameter value of intersection of given line segment with
        nearest boundary of this extent.
    Note: If the given end point is outside the extent (so the first check
      doesn't return first), AND the given start point is outside the extent,
      then the return value may be negative (if the vector is heading away
      from the extent), or greater than the input parameter (if heading
      towards the extent).
    Error return: returns SM_BIG_DOUBLE if the line cannot be clipped to
      the domain boundary.
      An example is a vertical line to the left of the domain.
      However, if the line does not intersect the domain, but can be clipped
      to the extensions of the boundaries, it will be clipped to those extensions.
      In those cases, the result is presumably meaningless.

      These error conditions can happen only if the start and end point
      are both outside of the domain.  Therefore, the caller is strongly
      advised to make sure that at least the start point is inside the domain.

***********************************************************************/
double SmExtent2d::ClipLine2d      // rtn: if given endPoint parameter is in Extent 
                                   //           return given endPoint parameter
                                   //      else return parameter of closest line/extentBoundary intersection
  (const SmPoint2d  & rLinePoint,  // in : line's base point
   const SmVector2d & rLineVec,    // in : line's vector
   double             dLineParam)  // in : line's end point Parameter to check
 const
{
  SM_ASSERT_DEFINED(this);

  // get sign of LineVec for given line segment
  double dSign =   (dLineParam < 0.0)
                 ? -1.0
                 :  1.0 ;

  // When LinePoint is on extent boundary and lineSeg points out,
  // return startPoint parameter = 0.0

  if (   (SM_ARE_SAME(m_vMin.x,rLinePoint.x) && rLineVec.x*dSign < - SM_EFF_ZERO*100.0) 
      || (SM_ARE_SAME(m_vMin.y,rLinePoint.y) && rLineVec.y*dSign < - SM_EFF_ZERO*100.0) 
      || (SM_ARE_SAME(m_vMax.x,rLinePoint.x) && rLineVec.x*dSign >   SM_EFF_ZERO*100.0) 
      || (SM_ARE_SAME(m_vMax.y,rLinePoint.y) && rLineVec.y*dSign >   SM_EFF_ZERO*100.0)) 
    {
      return 0.0;
    }

  // get line segment endPoint and a tolerance
  double    dRet   = dLineParam;
  double    dPSTol = SM_EFF_ZERO_SQRT/10000.00;
  SmPoint2d sEndPt =  rLinePoint 
                    + dLineParam * rLineVec;

  // when endPoint is contained by extent - return endPoint parameter
  if ( ContainsPoint2d(sEndPt,dPSTol) ) 
    {
      return dRet;
    }

  // find line 1st segment extent boundary intersection
  if (sEndPt.x < m_vMin.x - dPSTol) 
    {
      // Compute intersept with X min and reset line end point and parameter
      if ( smos_Fabs( sEndPt.x-rLinePoint.x ) < SM_EFF_ZERO * smos_Fabs( m_vMin.x - rLinePoint.x ) )
        { return SM_BIG_DOUBLE; }

      dRet = dRet * (m_vMin.x - rLinePoint.x) / (sEndPt.x - rLinePoint.x);
      sEndPt = rLinePoint + dRet * rLineVec;
    }
  else if (sEndPt.x > m_vMax.x + dPSTol) 
    {
      if ( smos_Fabs( sEndPt.x-rLinePoint.x ) < SM_EFF_ZERO * smos_Fabs( m_vMax.x - rLinePoint.x ) )
        { return SM_BIG_DOUBLE; }

      dRet = dRet * (m_vMax.x - rLinePoint.x) / (sEndPt.x - rLinePoint.x);
      sEndPt = rLinePoint + dRet * rLineVec;
    }

  if (sEndPt.y < m_vMin.y - dPSTol) 
    {
      // Compute intersept with Y min and reset line end point and parameter
      if ( smos_Fabs( sEndPt.y-rLinePoint.y ) < SM_EFF_ZERO * smos_Fabs( m_vMin.y - rLinePoint.y ) )
        { return SM_BIG_DOUBLE; }

      dRet = dRet * (m_vMin.y - rLinePoint.y) / (sEndPt.y - rLinePoint.y);
      sEndPt = rLinePoint + dRet * rLineVec;
    }
  else if (sEndPt.y > m_vMax.y + dPSTol) 
    {
      if ( smos_Fabs( sEndPt.y-rLinePoint.y ) < SM_EFF_ZERO * smos_Fabs( m_vMax.y - rLinePoint.y ) )
        { return SM_BIG_DOUBLE; }

      dRet = dRet * (m_vMax.y - rLinePoint.y) / (sEndPt.y - rLinePoint.y);
      sEndPt = rLinePoint + dRet * rLineVec;
    }

  // all done
  return dRet;

} // end SmExtent2d::ClipLine2d

/*******************************************************************//**
PURPOSE: Given a definition of a line segment, find the portion of it
    which lies inside of the 2D domain.

NOTES: 
***********************************************************************/
SmStatus SmExtent2d::IntersectWithInfiniteLine
  (const SmPoint2d  & rLinePoint,       // in : Pt  of Line = Pt + s*Vec
   const SmVector2d & rLineVec,         // in : Vec of Line = Pt + s*Vec
   SmBoolean        & rbFoundInterval,  // out: TRUE = Line intersects 2dDomain
                                        //      FALSE= no intersection
   SmExtent1d       & rTrimmedInterval, // out: Line interval of line/Extent intersection
   double           * dOptTolU_V,       // in : max allowed separation between intersecting uvPoints
                                        //      If dOptTolV is Null, this is used for both u and v.
   double           * dOptTolV)         // in : max allowed separation in v between intersecting uvPoints
  const                                 //      NULL = 100. * SM_EFF_ZERO, default:[NULL]
{
  SM_ASSERT_DEFINED(this);

  // init output
  rbFoundInterval = FALSE;

  // locals
  SmExtent1d sIvl;
  double dPrm;  // Parameter along line of our domain boundaries
  double dInt;  // x (or y) value at y (or x) domain boundary intersections
  double dTolU = dOptTolU_V ? *dOptTolU_V : SM_EFF_ZERO * 100.0 ;
  double dTolV = dOptTolV   ? *dOptTolV   : dTolU;
  double dMinX = m_vMin.x - dTolU ;
  double dMaxX = m_vMax.x + dTolU ;
  double dMinY = m_vMin.y - dTolV ;
  double dMaxY = m_vMax.y + dTolV ;

  // When line is not vertical
  if (smos_Fabs(rLineVec.x) > SM_EFF_ZERO) 
    {
      // get Min.x/Line intersectionPoint and LineParameter - save contained answers
      dPrm = (m_vMin.x - rLinePoint.x) / (rLineVec.x); // param on line of our x-min
      dInt = rLinePoint.y + dPrm * rLineVec.y ;        // y-intercept of line with our x-min
      if(dInt >= dMinY && dInt <= dMaxY) { sIvl.AddValue(dPrm); }
      
      // get Max.x/Line intersectionPoint and LineParameter - save contained answers
      dPrm = (m_vMax.x - rLinePoint.x) / (rLineVec.x);
      dInt = rLinePoint.y + dPrm * rLineVec.y ;
      if(dInt >= dMinY && dInt <= dMaxY) { sIvl.AddValue(dPrm); }
      
    } // end not a vertical line check

  // When line is not horizontal
  if (smos_Fabs(rLineVec.y) > SM_EFF_ZERO) 
    {
      // get Min.y/Line intersectionPoint and LineParameter - save contained answers
      dPrm = (m_vMin.y - rLinePoint.y) / (rLineVec.y);
      dInt = rLinePoint.x + dPrm * rLineVec.x ;
      if(dInt >= dMinX && dInt <= dMaxX) { sIvl.AddValue(dPrm); }
      
      // get Max.y/Line intersectionPoint and LineParameter - save contained answers
      dPrm = (m_vMax.y - rLinePoint.y) / (rLineVec.y);
      dInt = rLinePoint.x + dPrm * rLineVec.x ;
      if(dInt >= dMinX && dInt <= dMaxX) { sIvl.AddValue(dPrm); }
      
    } // end not a horizontal line check

  //
  if ( sIvl.GetMax() > sIvl.GetMin() - smos_Max(dTolU, dTolV) )  // (Preserve old behavior)
    {
      rbFoundInterval  = TRUE;
      rTrimmedInterval = sIvl;
     
#ifdef SM_DEBUG_CODE
      SmPoint2d sSegStart = rLinePoint + sIvl.GetMin() * rLineVec ;
      SmPoint2d sSegEnd   = rLinePoint + sIvl.GetMax() * rLineVec ;

      // Guard against huge input vector.
      double dScaledZero = SM_EFF_ZERO * rLineVec.GetMaxDimension();
      dScaledZero = smos_3Max( dScaledZero, dTolU, dTolV );
      SM_ASSERT_BREAK( ContainsPoint2d( sSegStart, dScaledZero ));
      SM_ASSERT_BREAK( ContainsPoint2d( sSegEnd,   dScaledZero ));
#endif // SM_DEBUG_CODE

    }

  // all done
  return SM_SUCCESS;

} // end SmExtent2d::IntersectWithInfiniteLine

/*******************************************************************//**
PURPOSE: Get the U Evaluation side of a surface domain.  

RETURN ---  TRUE = TValue is in lower half of interval
            FALSE= TVAlue is in upper half of interval  

NOTES: This method is used to specify the bFromLeft 
    flag needed for computing curve and surface derivatives at
    discontinuity points.  (the discontinuity is expected to
    be on the boundary of the interval.)

    +----1----+----2----+        Curve with two intervals
              P
    When evaluating a point P on an interval boundary and 
    bFromLeft = TRUE  evaluate P in upper interval 2, P is on the left of the interval
                FALSE evaluate P in lower interval 1, P is on the right of the interval
***********************************************************************/
SmBoolean SmExtent2d::GetULeftEval(double dUValue) const
{
  SM_ASSERT_DEFINED(this) ; 
  double dUMid = (m_vMin.x + m_vMax.x) / 2.0;
  SmBoolean bRet = TRUE;
  if (dUValue > dUMid) bRet = FALSE;
  return bRet;
}

/*******************************************************************//**
PURPOSE: Get the v Evaluation side of a surface domain.  

RETURN ---  TRUE = TValue is in lower half of interval
            FALSE= TVAlue is in upper half of interval  

NOTES: This method is used to specify the bFromLeft 
    flag needed for computing curve and surface derivatives at
    discontinuity points.  (the discontinuity is expected to
    be on the boundary of the interval.)

    +----1----+----2----+        Curve with two intervals
              P
    When evaluating a point P on an interval boundary and 
    bFromLeft = TRUE  evaluate P in upper interval 2, P is on the left of the interval
                FALSE evaluate P in lower interval 1, P is on the right of the interval
***********************************************************************/
SmBoolean SmExtent2d::GetVLeftEval(double dVValue) const
{
  SM_ASSERT_DEFINED(this) ; 
  double dVMid = (m_vMin.y + m_vMax.y) / 2.0;
  SmBoolean bRet = TRUE;
  if (dVValue > dVMid) bRet = FALSE;
  return bRet;
}

/*******************************************************************//**
PURPOSE: Map a point from ThisDomain to OtherDomain 
    assuming the two domains map to one another linearly, e.g.
    Map = ThisDomainPoint->NormalizedCoordinates->OtherDomainPoint.

NOTES: The original point needs to be in or on this domain.
***********************************************************************/
SmStatus SmExtent2d::Point2dMapToDomain
  (const SmPoint2d  & crPointInThisDomain,    // in : Target Point to map
   const SmExtent2d & crOther,                // in : The other extent
   SmBoolean          bSwitchParameters,      // in : TRUE = Swap U/V parameters
   SmPoint2d        & rPointInOtherDomain)    // out: The transformed point
  const
{
  SM_ASSERT_DEFINED(this);
  SM_ASSERT_DEFINED(&crOther);

  // locals
  SmPoint2d sNat;

  // get crPointInThisDomain's Normalized coordinates
  if (Inversion(crPointInThisDomain,sNat) != SM_SUCCESS) 
    {
      return(SM_ERR) ;
    }

  // Evaluate NormalizedPoint in OtherDomain (transposed when asked)
  if (!bSwitchParameters) 
    { rPointInOtherDomain = crOther.Evaluate(sNat.x,sNat.y);
    }
  else                    
    { rPointInOtherDomain = crOther.Evaluate(sNat.y,sNat.x);
    }
  
  // all done
  return(SM_SUCCESS) ;

} // end SmExtent2d::Point2dMapToDomain

/*******************************************************************//**
PURPOSE: return a bounded SmExtent2d to approximate an unbounded one

NOTES: 0. returns a SmExtent2d whose infinite boundary values 
          have been replaced by finite values.
          Those replacement values are based upon the dUnboundedHalfSize,
          pUnboundedCenter, and interval opposing end values.

       1. This is used for graphics and sampling to allow an application
          to easily define an area of focus for infinite extents.
          
       2. This method handles all the combinations of half spaces and unbounded spaces.
          
       3. The approximation of a BOUNDED extent is an exact copy of the original extent.
***********************************************************************/
SmExtent3d SmExtent2d::ApproximateUnbounded // eff: return a bounded SmExtent3d to approximate an unbounded one                  
 (SmPoint2d  *pUnboundedCenter,             // in : center of unbounded intervals, NULL = [0,0,0], default:[NULL]
  double      dUnboundedHalfSize,           // in : the size used for infinite 1/2 spaces, default:[SM_BOUNDED_INFINITE_PARAM]
                                            //      a totally unbounded plane is approximated by a square twice this size
  SmExtent2d *pOptExpandedApprox)           // out: a 2nd extent expanded a small bit - used by graphics, NULL to ignore, default:[NULL]
 const
{
  // locals
  SmBoundaryType eBdryType[2] ;
  SmPoint2d sCenter(0,0) ;
  if(pUnboundedCenter) sCenter = *pUnboundedCenter ; 

  // copy min/max points
  SmPoint2d sMin    = GetMin();
  SmPoint2d sMax    = GetMax();

  // check for infinite boundaries
  SmBoolean bBounded = this->IsBounded(eBdryType) ;
  
  // check for infinite boundaries
  if(bBounded == FALSE)
    {
      // move infintie boundaries to viewable places - draw them as a double plane with an internal x
      sMin.x =   eBdryType[0] == SM_BT_BOUNDED       ? sMin.x
               : eBdryType[0] == SM_BT_UNBOUNDED_MIN ? (  (   (pUnboundedCenter == NULL) 
                                                           || (sMax.x < pUnboundedCenter->x + dUnboundedHalfSize))
                                                        ? (sMax.x - 2 * dUnboundedHalfSize)
                                                        : (pUnboundedCenter->x - dUnboundedHalfSize))
               : eBdryType[0] == SM_BT_UNBOUNDED_MAX ? (  (   (pUnboundedCenter == NULL) 
                                                           || (sMin.x > pUnboundedCenter->x - dUnboundedHalfSize))
                                                        ? (sMin.x)
                                                        : (pUnboundedCenter->x - dUnboundedHalfSize))
               : eBdryType[0] == SM_BT_UNBOUNDED     ? sCenter.x - dUnboundedHalfSize : sMin.x ;

      sMax.x =   eBdryType[0] == SM_BT_BOUNDED       ? sMax.x
               : eBdryType[0] == SM_BT_UNBOUNDED_MIN ? (  (   (pUnboundedCenter == NULL) 
                                                           || (sMax.x < pUnboundedCenter->x + dUnboundedHalfSize))
                                                        ? (sMax.x)
                                                        : (pUnboundedCenter->x + dUnboundedHalfSize))
               : eBdryType[0] == SM_BT_UNBOUNDED_MAX ? (  (   (pUnboundedCenter == NULL) 
                                                           || (sMin.x > pUnboundedCenter->x - dUnboundedHalfSize))
                                                        ? (sMin.x + 2 * dUnboundedHalfSize)
                                                        : (pUnboundedCenter->x + dUnboundedHalfSize))
               : eBdryType[0] == SM_BT_UNBOUNDED     ? sCenter.x + dUnboundedHalfSize : sMax.x ;

      sMin.y =   eBdryType[1] == SM_BT_BOUNDED       ? sMin.y
               : eBdryType[1] == SM_BT_UNBOUNDED_MIN ? (  (   (pUnboundedCenter == NULL) 
                                                           || (sMax.y < pUnboundedCenter->y + dUnboundedHalfSize))
                                                        ? (sMax.y - 2 * dUnboundedHalfSize)
                                                        : (pUnboundedCenter->y - dUnboundedHalfSize))
               : eBdryType[1] == SM_BT_UNBOUNDED_MAX ? (  (   (pUnboundedCenter == NULL) 
                                                           || (sMin.y > pUnboundedCenter->y - dUnboundedHalfSize))
                                                        ? (sMin.y)
                                                        : (pUnboundedCenter->y - dUnboundedHalfSize))
               : eBdryType[1] == SM_BT_UNBOUNDED     ? sCenter.y - dUnboundedHalfSize : sMin.y ;

      sMax.y =   eBdryType[1] == SM_BT_BOUNDED       ? sMax.y
               : eBdryType[1] == SM_BT_UNBOUNDED_MIN ? (  (   (pUnboundedCenter == NULL) 
                                                           || (sMax.y < pUnboundedCenter->y + dUnboundedHalfSize))
                                                        ? (sMax.y)
                                                        : (pUnboundedCenter->y + dUnboundedHalfSize))
               : eBdryType[1] == SM_BT_UNBOUNDED_MAX ? (  (   (pUnboundedCenter == NULL) 
                                                           || (sMin.y > pUnboundedCenter->y - dUnboundedHalfSize))
                                                        ? (sMin.y + 2 * dUnboundedHalfSize)
                                                        : (pUnboundedCenter->y + dUnboundedHalfSize))
               : eBdryType[1] == SM_BT_UNBOUNDED     ? sCenter.y + dUnboundedHalfSize : sMax.y ;

    } // end unbounded check

  // when asked - build an expanded extent as well
  if(pOptExpandedApprox)
    {
      SmPoint2d sMinInc = sMin ;
      SmPoint2d sMaxInc = sMax ;
      double dInfInc = dUnboundedHalfSize/12.0 ;

      // when extent is not bounded - it needs to be expanded
      if(bBounded == FALSE)
        {
          // else expand the bbox by dInfInc
          if(   eBdryType[0] == SM_BT_UNBOUNDED_MIN
             || eBdryType[0] == SM_BT_UNBOUNDED)     { sMinInc.x -= dInfInc ; }    
          if(   eBdryType[0] == SM_BT_UNBOUNDED_MAX
             || eBdryType[0] == SM_BT_UNBOUNDED)     { sMaxInc.x += dInfInc ; }    

          if(   eBdryType[1] == SM_BT_UNBOUNDED_MIN
             || eBdryType[1] == SM_BT_UNBOUNDED)     { sMinInc.y -= dInfInc ; }    
          if(   eBdryType[1] == SM_BT_UNBOUNDED_MAX
             || eBdryType[1] == SM_BT_UNBOUNDED)     { sMaxInc.y += dInfInc ; }    

        } // end bBounded == FALSE check
          
      // set output
      pOptExpandedApprox->SetMinMax(sMinInc, sMaxInc) ;    
        
    } // end need to build extended extent check

  // all done
  return(SmExtent3d(sMin, sMax)) ;

} // end SmExtent2d::ApproximateUnbounded

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
void SmExtent2d::Dump(void) const
{
  SM_ASSERT_DEFINED(this);
  TCHAR sBuff[SM_TBLOCK_SIZE];
  smos_sprintf(sBuff,
             _T("SmExtent2d Min:[%lf, %lf],  Max:[%lf, %lf] \n"),
             m_vMin.x,
             m_vMin.y,
             m_vMax.x,
             m_vMax.y);
  smos_WriteBuffer(sBuff);
} // end SmExtent2d::Dump

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
SmDisplayList * SmExtent2d::Draw
 (const SmContext * pContext,        // NotUsed: in :
  const SmSurface * pOptOutSurface,  // in : Draw on this plane, NULL=draw on z=0 plane, default:[NULL]
  SmGfxArraySet   * pOptGfxSet)      // i/o: used for OpenGL ES 2.0. When given, accumulate GfxVertexArrays not GL calls.
                                     //      NULL to ignore. default:[NULL]
 const
{
  SM_REF1(pContext) ;
  SmDisplayList *pRtn = NULL ;

#ifdef SM_GFX_CODE
  smgfx_Open(smgfx_GetRuleColor(), NULL, NULL, FALSE, pOptGfxSet);

  if(pOptOutSurface)
    {
      // draw on plane
      if(pOptOutSurface->IsPlanar())
        {
          SmPoint3d sPt00, sPt01, sPt11, sPt10 ;
          SmPoint2d sUV01(m_vMin.x, m_vMax.y) ;
          SmPoint2d sUV10(m_vMax.x, m_vMin.y) ; 
          pOptOutSurface->EvaluatePoint(m_vMin, sPt00) ;
          pOptOutSurface->EvaluatePoint(sUV01,  sPt01) ;
          pOptOutSurface->EvaluatePoint(m_vMax, sPt11) ;
          pOptOutSurface->EvaluatePoint(sUV10,  sPt10) ;

          smgfx_DrawPoint(sPt00.x, sPt00.y, sPt00.z, pOptGfxSet);
          smgfx_DrawLine (sPt00.x, sPt00.y, sPt00.z, sPt01.x, sPt01.y, sPt01.z, pOptGfxSet);
          smgfx_DrawLine (sPt01.x, sPt01.y, sPt01.z, sPt11.x, sPt11.y, sPt11.z, pOptGfxSet);
          smgfx_DrawLine (sPt11.x, sPt11.y, sPt11.z, sPt10.x, sPt10.y, sPt10.z, pOptGfxSet);
          smgfx_DrawLine (sPt10.x, sPt10.y, sPt10.z, sPt00.x, sPt00.y, sPt00.z, pOptGfxSet);
        }
      else // Draw on general surface
        {
          SmPoint3d sPt00(m_vMin) ;
          SmPoint3d sPt01(m_vMin.x, m_vMax.y, 0.0) ; 
          SmPoint3d sPt11(m_vMax) ; 
          SmPoint3d sPt10(m_vMax.x, m_vMin.y, 0.0) ; 

          SmContext sContext, * pThisContext = pOptOutSurface ? (SmContext *)pOptOutSurface->GetContext() : &sContext ;
          SmLine sLine0(sPt00, sPt01, 2, pThisContext) ; SmCrvOnSurf sCrv0(sLine0, *(SmSurface *)pOptOutSurface, NULL, 0, pContext) ;
          SmLine sLine1(sPt01, sPt11, 2, pThisContext) ; SmCrvOnSurf sCrv1(sLine1, *(SmSurface *)pOptOutSurface, NULL, 0, pContext) ;
          SmLine sLine2(sPt11, sPt10, 2, pThisContext) ; SmCrvOnSurf sCrv2(sLine2, *(SmSurface *)pOptOutSurface, NULL, 0, pContext) ;
          SmLine sLine3(sPt10, sPt00, 2, pThisContext) ; SmCrvOnSurf sCrv3(sLine3, *(SmSurface *)pOptOutSurface, NULL, 0, pContext) ;

          sCrv0.Draw() ; 
          sCrv1.Draw() ; 
          sCrv2.Draw() ; 
          sCrv3.Draw() ; 
        }
    }
  else // Draw raw
    {
      smgfx_DrawPoint(m_vMin.x, m_vMin.y, 0.0, pOptGfxSet);
      smgfx_DrawLine (m_vMin.x, m_vMin.y, 0.0, m_vMax.x, m_vMin.y, 0.0, pOptGfxSet);
      smgfx_DrawLine (m_vMax.x, m_vMin.y, 0.0, m_vMax.x, m_vMax.y, 0.0, pOptGfxSet);
      smgfx_DrawLine (m_vMax.x, m_vMax.y, 0.0, m_vMin.x, m_vMax.y, 0.0, pOptGfxSet);
      smgfx_DrawLine (m_vMin.x, m_vMax.y, 0.0, m_vMin.x, m_vMin.y, 0.0, pOptGfxSet);
    }

  pRtn = smgfx_Close(pOptGfxSet) ;

#else
  SM_REF2(pOptOutSurface, pOptGfxSet);
#endif // end SM_GFX_CODE
  return(pRtn) ;

} // end SmExtent2d::Draw

/*******************************************************************//**
PURPOSE: AssertValid() Test reporting static labels
***********************************************************************/
SmAssertReportLabel sAssertExtent2d_list[] =
{
  { SM_AT_MINMAX, _T("Min Max"), _T("m_vMin <= m_vMax") }
} ;

/*******************************************************************//**
PURPOSE: Determine if the 2D extent is valid. 
 
RETURNS ---  TRUE  = OK
             FALSE = Problem
***********************************************************************/
SmBoolean SmExtent2d::AssertValid
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

  SmBoolean bRtn = TRUE;
  
  bRtn &= SM_ASSERT_BOOLEAN_REPORT(0, SM_LEVEL_0, (m_vMin.x <= m_vMax.x && m_vMin.y <= m_vMax.y), _T("") ) ;
  
  // all done
  SM_ASSERT_BREAK(bRtn) ;
  return(bRtn) ;

} // end SmExtent2d::AssertValid

/*******************************************************************//**
PURPOSE: Check for undefined values. 
 
NOTES: Values are set to undefined when object is destructed.
***********************************************************************/
SmBoolean SmExtent2d::AssertDefined(void) const
{
  SmBoolean bRtn = (   (   m_vMin.x!= SM_UNDEF_DOUBLE 
                        && m_vMin.y!= SM_UNDEF_DOUBLE 
                        && m_vMax.x!=-SM_UNDEF_DOUBLE
                        && m_vMax.y!=-SM_UNDEF_DOUBLE) ) ;
                    //      && !IsInit() 
                    //      && (m_vMax.x >= m_vMin.x) 
                    //      && (m_vMax.y >= m_vMin.y));
#ifdef SM_DEBUG_CODE
  if(bRtn == FALSE)
    { SM_ASSERT_BREAK(bRtn) ; }
#endif // SM_DEBUG_CODE
  return(bRtn) ;

} // end SmExtent2d::AssertDefined



