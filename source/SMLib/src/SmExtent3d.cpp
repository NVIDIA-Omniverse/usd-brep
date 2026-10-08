// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************//**
* FILE NAME --- SmExtent3d.cpp
* PURPOSE: Source file for implementation of SmExtent3d methods.
**********************************************************************/

#include "StdAfx.h"

#include <SmExtent3d.h>
#include <SmGraphicsExtern.h>
#include <SmGraphicsOutput.h>
#include <SmAssertArray.h>
#include <SmPseudoBox.h>

/*******************************************************************//**
PURPOSE: Initialize the 3D extent to values which will enable points
    to be added later.  

NOTES: It is inside out infinite.  
   The extent is invalid until points are added to it.
***********************************************************************/
void SmExtent3d::Init() 
{ 
  m_vMin.x = m_vMin.y = m_vMin.z =  SM_BIG_DOUBLE; 
  m_vMax.x = m_vMax.y = m_vMax.z = -SM_BIG_DOUBLE; 

} // end SmExtent3d::Init 

/*******************************************************************//**
PURPOSE: AssertValid() Test reporting static labels
***********************************************************************/
SmAssertReportLabel sAssertExtent3d_list[] =
{
  {SM_AT_MINMAX, _T("Min Max"), _T("m_vMin <= m_vMax") }
} ;

/*******************************************************************//**
PURPOSE: Determine the validity of the extent.

RETURNS ---  TRUE  = OK
             FALSE = Problem
***********************************************************************/
SmBoolean SmExtent3d::AssertValid
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
  
  bRtn &= SM_ASSERT_VALUE_REPORT(0, SM_LEVEL_0, 
                                 (   m_vMin.x <= m_vMax.x  
                                  && m_vMin.y <= m_vMax.y  
                                  && m_vMin.z <= m_vMax.z), 
                                 0.0,
                                 smos_3Max(m_vMin.x - m_vMax.x, 
                                           m_vMin.y - m_vMax.y,
                                           m_vMin.z - m_vMax.z), 
                                 _T("") ) ;
  
  // all done
  SM_ASSERT_BREAK(bRtn) ;
  return(bRtn) ;

} // end SmExtent3d::AssertValid

/*******************************************************************//**
PURPOSE: check for undefined values.

NOTES: After destruction values are set to undefined.
***********************************************************************/
SmBoolean SmExtent3d::AssertDefined() const
{
  SmBoolean bRtn = (   m_vMin.x!= SM_UNDEF_DOUBLE 
                    && m_vMin.y!= SM_UNDEF_DOUBLE 
                    && m_vMin.z!= SM_UNDEF_DOUBLE 
                    && m_vMax.x!=-SM_UNDEF_DOUBLE
                    && m_vMax.y!=-SM_UNDEF_DOUBLE
                    && m_vMax.z!=-SM_UNDEF_DOUBLE);
  SM_ASSERT_BREAK(bRtn) ;
  return(bRtn) ;

} // end SmExtent3d::AssertDefined

/*******************************************************************//**
PURPOSE: Construct an extent from a minimum and maxumum point.

NOTES: 
***********************************************************************/
SmExtent3d::SmExtent3d(const SmPoint3d & rMin, const SmPoint3d & rMax)
{ 
  m_vMin = rMin ;
  m_vMax = rMax ;
  
  if(m_vMin.x > m_vMax.x) { SM_SWAP(double, m_vMin.x, m_vMax.x) ; } 
  if(m_vMin.y > m_vMax.y) { SM_SWAP(double, m_vMin.y, m_vMax.y) ; } 
  if(m_vMin.z > m_vMax.z) { SM_SWAP(double, m_vMin.z, m_vMax.z) ; } 

} // end SmExtent3d::SmExtent3d

/*******************************************************************//**
PURPOSE: Copy constructor.

NOTES: 
***********************************************************************/
SmExtent3d::SmExtent3d
  (const SmExtent3d & crOriginal) 
{ 
  m_vMin = crOriginal.m_vMin; 
  m_vMax = crOriginal.m_vMax; 

} // end SmExtent3d::SmExtent3d

/*******************************************************************//**
PURPOSE: set BBox to bound SmPseudoBox

NOTES: Make an SmExtent3d that circumscribes the Original PseudoBox
       The Extent3d will be larger than nonAxis aligned Pseudo boxes
       and the same size for Axis aligned pseudo boxes
***********************************************************************/
void SmExtent3d::Circumscribe
  (const SmPseudoBox & crPseudoBox) 
{ 
  // locals
  ULONG ii ;
  SmTArray<SmPoint3d> sCorners(8,NULL,8) ; 
  long alAxisMap[3] ;

  // low work - axis aligned PseudoBoxp
  if(crPseudoBox.IsAxisAligned(alAxisMap))  // out: alAxisMap[0] = index+1 of basis parallel to X, neg = in negative direction
                                            //      alAxisMap[1] = index+1 of basis parallel to Y, neg = in negative direction
                                            //      alAxisMap[2] = index+1 of basis parallel to Z, neg = in negative direction
    {
      // Set m_vMin and m_vMax directly from the PSeudoBox intervals
      //  the reason this extra branch exists is to allow partially
      //   unbound BBoxes to be built for partially unbound PseudoBoxes.
      //  That's helpful to the SmVolume class
      for(ii=0;ii<3;ii++)
        {
          ULONG lx = smos_Labs(alAxisMap[ii]) - 1 ;
          m_vMin[ii] = (alAxisMap[ii] > 0) ? crPseudoBox.GetInterval(lx).GetMin() : -crPseudoBox.GetInterval(lx).GetMax() ;
          m_vMax[ii] = (alAxisMap[ii] > 0) ? crPseudoBox.GetInterval(lx).GetMax() : -crPseudoBox.GetInterval(lx).GetMin() ;
        }

      // all done
      return ;

    }  // end Axis aligned PseudoBoxp check

  // circumscribe un algined unbounded PseudoBoxes with infinite extents - until a better idea comes along
  if(!crPseudoBox.IsBounded())
    {
      m_vMin.x = m_vMin.y = m_vMin.z = -SM_INFINITE_PARAMETER ;
      m_vMax.x = m_vMax.y = m_vMax.z =  SM_INFINITE_PARAMETER ;
    }
  else // circumscribe unaligned finite PseudoBox corners
    {
      // Build BBox containing all corners
      crPseudoBox.GetCorners(sCorners) ; 

      m_vMin = sCorners[0] ; m_vMax = sCorners[0] ; 

      if(m_vMin.x > sCorners[1].x) 
        m_vMin.x = sCorners[1].x ; 
      if(m_vMax.x < sCorners[1].x) 
        m_vMax.x = sCorners[1].x ;
      if(m_vMin.y > sCorners[1].y) 
        m_vMin.y = sCorners[1].y ; 
      if(m_vMax.y < sCorners[1].y) 
        m_vMax.y = sCorners[1].y ;
      if(m_vMin.z > sCorners[1].z) 
        m_vMin.z = sCorners[1].z ; 
      if(m_vMax.z < sCorners[1].z) 
        m_vMax.z = sCorners[1].z ;
 
      if(m_vMin.x > sCorners[2].x) 
        m_vMin.x = sCorners[2].x ; 
      if(m_vMax.x < sCorners[2].x) 
        m_vMax.x = sCorners[2].x ;
      if(m_vMin.y > sCorners[2].y) 
        m_vMin.y = sCorners[2].y ; 
      if(m_vMax.y < sCorners[2].y) 
        m_vMax.y = sCorners[2].y ;
      if(m_vMin.z > sCorners[2].z) 
        m_vMin.z = sCorners[2].z ; 
      if(m_vMax.z < sCorners[2].z) 
        m_vMax.z = sCorners[2].z ;
 
      if(m_vMin.x > sCorners[3].x) 
        m_vMin.x = sCorners[3].x ; 
      if(m_vMax.x < sCorners[3].x) 
        m_vMax.x = sCorners[3].x ;
      if(m_vMin.y > sCorners[3].y) 
        m_vMin.y = sCorners[3].y ; 
      if(m_vMax.y < sCorners[3].y) 
        m_vMax.y = sCorners[3].y ;
      if(m_vMin.z > sCorners[3].z) 
        m_vMin.z = sCorners[3].z ; 
      if(m_vMax.z < sCorners[3].z) 
        m_vMax.z = sCorners[3].z ;
 
      if(m_vMin.x > sCorners[4].x) 
        m_vMin.x = sCorners[4].x ; 
      if(m_vMax.x < sCorners[4].x) 
        m_vMax.x = sCorners[4].x ;
      if(m_vMin.y > sCorners[4].y) 
        m_vMin.y = sCorners[4].y ; 
      if(m_vMax.y < sCorners[4].y) 
        m_vMax.y = sCorners[4].y ;
      if(m_vMin.z > sCorners[4].z) 
        m_vMin.z = sCorners[4].z ; 
      if(m_vMax.z < sCorners[4].z) 
        m_vMax.z = sCorners[4].z ;
 
      if(m_vMin.x > sCorners[5].x)
        m_vMin.x = sCorners[5].x ; 
      if(m_vMax.x < sCorners[5].x) 
        m_vMax.x = sCorners[5].x ;
      if(m_vMin.y > sCorners[5].y) 
        m_vMin.y = sCorners[5].y ; 
      if(m_vMax.y < sCorners[5].y) 
        m_vMax.y = sCorners[5].y ;
      if(m_vMin.z > sCorners[5].z) 
        m_vMin.z = sCorners[5].z ; 
      if(m_vMax.z < sCorners[5].z) 
        m_vMax.z = sCorners[5].z ;
 
      if(m_vMin.x > sCorners[6].x) 
        m_vMin.x = sCorners[6].x ; 
      if(m_vMax.x < sCorners[6].x) 
        m_vMax.x = sCorners[6].x ;
      if(m_vMin.y > sCorners[6].y) 
        m_vMin.y = sCorners[6].y ; 
      if(m_vMax.y < sCorners[6].y) 
        m_vMax.y = sCorners[6].y ;
      if(m_vMin.z > sCorners[6].z) 
        m_vMin.z = sCorners[6].z ; 
      if(m_vMax.z < sCorners[6].z) 
        m_vMax.z = sCorners[6].z ;
 
      if(m_vMin.x > sCorners[7].x) 
        m_vMin.x = sCorners[7].x ; 
      if(m_vMax.x < sCorners[7].x) 
        m_vMax.x = sCorners[7].x ;
      if(m_vMin.y > sCorners[7].y) 
        m_vMin.y = sCorners[7].y ; 
      if(m_vMax.y < sCorners[7].y) 
        m_vMax.y = sCorners[7].y ;
      if(m_vMin.z > sCorners[7].z) 
        m_vMin.z = sCorners[7].z ; 
      if(m_vMax.z < sCorners[7].z) 
        m_vMax.z = sCorners[7].z ;
    }
 
} // end SmExtent3d::Circumscribe(SmPseudoBox)

/*******************************************************************//**
PURPOSE: Extents the extent to cover this point if necessary.

NOTES: 
***********************************************************************/
void SmExtent3d::AddPoint3d
  (const SmPoint3d & rPoint)
{ 
    if (rPoint.x < m_vMin.x) m_vMin.x = rPoint.x;
    if (rPoint.y < m_vMin.y) m_vMin.y = rPoint.y;
    if (rPoint.z < m_vMin.z) m_vMin.z = rPoint.z;
    if (rPoint.x > m_vMax.x) m_vMax.x = rPoint.x;
    if (rPoint.y > m_vMax.y) m_vMax.y = rPoint.y;
    if (rPoint.z > m_vMax.z) m_vMax.z = rPoint.z;

} // end SmExtent3d::AddPoint3d

/*******************************************************************//**
PURPOSE: Set the minimum and maximum points of this extent.

NOTES: 
***********************************************************************/
SmStatus SmExtent3d::SetMinMax
  (const SmPoint3d & rMin, 
   const SmPoint3d & rMax)
{ 
    SmStatus sRet = SM_SUCCESS;
    if (rMin.x > rMax.x || rMin.y > rMax.y || rMin.z > rMax.z) 
      {
        SE(SM_ERR_INVALID_INPUT);
        sRet = SM_ERR_INVALID_INPUT;
      }
    else 
      {
        m_vMin = rMin; m_vMax = rMax; 
      }
    return sRet;

} // end SmExtent3d::SetMinMax

/*******************************************************************//**
PURPOSE: Set the minimum and maximum points of this extent.

NOTES: 
***********************************************************************/
SmStatus SmExtent3d::SetMinMax
  (double dMinX,
   double dMinY,
   double dMinZ,
   double dMaxX,
   double dMaxY,
   double dMaxZ) 
{ 
  SmStatus sRet = SM_SUCCESS;
  if (   dMinX > dMaxX 
      || dMinY > dMaxY
      || dMinZ > dMaxZ) { SE(SM_ERR_INVALID_INPUT);
                          sRet = SM_ERR_INVALID_INPUT; 
                        } 
  else                  { m_vMin.x = dMinX; 
                          m_vMin.y = dMinY; 
                          m_vMin.z = dMinZ; 

                          m_vMax.x = dMaxX;
                          m_vMax.y = dMaxY; 
                          m_vMax.z = dMaxZ; 
                        } 
  return sRet;

} // end SmExtent3d::SetMinMax

/*******************************************************************//**
PURPOSE: Set the u-interval.

NOTES: 
***********************************************************************/
SmStatus SmExtent3d::SetUInterval
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

} // end SmExtent3d::SetUInterval

/*******************************************************************//**
PURPOSE: Set the v-interval.

NOTES: 
***********************************************************************/
SmStatus SmExtent3d::SetVInterval
  ( const SmExtent1d & crVIvl )
{ 
  SmBoolean bOK = TRUE ;
  
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

} // end SmExtent3d::SetVInterval

/*******************************************************************//**
PURPOSE: Set the w-interval.

NOTES: 
***********************************************************************/
SmStatus SmExtent3d::SetWInterval( const SmExtent1d & crWIvl )
{ 
  SmBoolean bOK = TRUE ;
  
#ifdef SM_DEBUG_CODE
  bOK = SM_ASSERT_VALID_NO_STREAM(&crWIvl) ;
#endif // SM_DEBUG_CODE

  if ( bOK )  
    { 
      m_vMin.z = crWIvl.GetMin(); 
      m_vMax.z = crWIvl.GetMax();
      return(SM_SUCCESS) ; 
    } 
  else                              
    { SE(SM_ERR_INVALID_INPUT);
      return(SM_ERR_INVALID_INPUT) ; 
    } 

} // end SmExtent3d::SetWInterval

/*******************************************************************//**
PURPOSE: Set the minimum u-value of the 3D extent.

NOTES: Return SM_ERR if dNewUMin > the max u value.
***********************************************************************/
SmStatus SmExtent3d::SetUMin( double dNewUMin )
{
  if ( dNewUMin > m_vMax.x ) SER(SM_ERR_INVALID_INPUT);
  m_vMin.x = dNewUMin;
  return SM_SUCCESS;
}
/*******************************************************************//**
PURPOSE: Set the maximum u-value of the 3D extent.

NOTES: Return SM_ERR if dNewUMax < the min u value.
***********************************************************************/
SmStatus SmExtent3d::SetUMax( double dNewUMax )
{
  if ( dNewUMax < m_vMin.x ) SER(SM_ERR_INVALID_INPUT);
  m_vMax.x = dNewUMax;
  return SM_SUCCESS;
}
/*******************************************************************//**
PURPOSE: Set the minimum v-value of the 3D extent.

NOTES: Return SM_ERR if dNewVMin > the max v value.
***********************************************************************/
SmStatus SmExtent3d::SetVMin( double dNewVMin )
{
  if ( dNewVMin > m_vMax.y ) SER(SM_ERR_INVALID_INPUT);
  m_vMin.y = dNewVMin;
  return SM_SUCCESS;
}
/*******************************************************************//**
PURPOSE: Set the maximum v-value of the 3D extent.

NOTES: Return SM_ERR if dNewVMax < the min v value.
***********************************************************************/
SmStatus SmExtent3d::SetVMax( double dNewVMax )
{
  if ( dNewVMax < m_vMin.y ) SER(SM_ERR_INVALID_INPUT);
  m_vMax.y = dNewVMax;
  return SM_SUCCESS;
}
/*******************************************************************//**
PURPOSE: Set the minimum w-value of the 3D extent.

NOTES: Return SM_ERR if dNewVMin > the max w value.
***********************************************************************/
SmStatus SmExtent3d::SetWMin( double dNewWMin )
{
  if ( dNewWMin > m_vMax.z ) SER(SM_ERR_INVALID_INPUT);
  m_vMin.z = dNewWMin;
  return SM_SUCCESS;
}
/*******************************************************************//**
PURPOSE: Set the maximum w-value of the 3D extent.

NOTES: Return SM_ERR if dNewVMax < the min w value.
***********************************************************************/
SmStatus SmExtent3d::SetWMax( double dNewWMax )
{
  if ( dNewWMax < m_vMin.z ) SER(SM_ERR_INVALID_INPUT);
  m_vMax.z = dNewWMax;
  return SM_SUCCESS;
}

/*******************************************************************//**
PURPOSE: Do a normalized evaluation to produce a point in the
    extent.  

NOTES: For example: (0,0,0) produces the minimum point;
    (1,1,1) produces the maximum point; (0.5,0.5,0.5) produces the
    point in the center of the extent.
***********************************************************************/
SmPoint3d SmExtent3d::Evaluate
  (double dNormalizedX, 
   double dNormalizedY, 
   double dNormalizedZ) 
  const
{
    SM_ASSERT_DEFINED(this) ; 
    SM_ASSERT_BREAK(0.0 <= dNormalizedX && dNormalizedX <= 1.0);
    SM_ASSERT_BREAK(0.0 <= dNormalizedY && dNormalizedY <= 1.0);
    SM_ASSERT_BREAK(0.0 <= dNormalizedZ && dNormalizedZ <= 1.0);
    SmPoint3d sRet;

    // sRet.x = m_vMin.x + dNormalizedX * (m_vMax.x - m_vMin.x);
    // sRet.y = m_vMin.y + dNormalizedY * (m_vMax.y - m_vMin.y);
    // sRet.z = m_vMin.z + dNormalizedZ * (m_vMax.z - m_vMin.z);
    sRet.x =
      ( dNormalizedX == 0.0 ) ? m_vMin.x :
      ( dNormalizedX == 1.0 ) ? m_vMax.x :
            m_vMin.x + dNormalizedX * (m_vMax.x - m_vMin.x);
    sRet.y =
      ( dNormalizedY == 0.0 ) ? m_vMin.y :
      ( dNormalizedY == 1.0 ) ? m_vMax.y :
            m_vMin.y + dNormalizedY * (m_vMax.y - m_vMin.y);
    sRet.z =
      ( dNormalizedZ == 0.0 ) ? m_vMin.z :
      ( dNormalizedZ == 1.0 ) ? m_vMax.z :
            m_vMin.z + dNormalizedZ * (m_vMax.z - m_vMin.z);
    return sRet;

} // end SmExtent3d::Evaluate

/*******************************************************************//**
PURPOSE: Find the normalized parameters corresponding to this point
    or return an error if the point is not inside of the domain.  

NOTES: This method does the opposite of Evaluate.
***********************************************************************/
SmStatus SmExtent3d::Invert
  (const SmPoint3d & cr3DPoint,
   SmPoint3d & rNormalizedParameters) 
  const
{
    SM_ASSERT_DEFINED(this) ; 
    rNormalizedParameters = (cr3DPoint - m_vMin);
    SmVector3d sVec = (m_vMax - m_vMin);
    rNormalizedParameters.x /= sVec.x;
    rNormalizedParameters.y /= sVec.y;
    rNormalizedParameters.z /= sVec.z;
    if (rNormalizedParameters.x < 0.0 || rNormalizedParameters.x > 1.0) { return SM_ERR; }
    if (rNormalizedParameters.y < 0.0 || rNormalizedParameters.y > 1.0) { return SM_ERR; }
    if (rNormalizedParameters.z < 0.0 || rNormalizedParameters.z > 1.0) { return SM_ERR; }
    return SM_SUCCESS;

} // end SmExtent3d::Invert

/*******************************************************************//**
PURPOSE: If the point is outside of the extent, clamp it to the 
    nearest boundary of the extent. 

NOTES: 
***********************************************************************/
SmPoint3d SmExtent3d::ClampPoint3d(const SmPoint3d & rPoint) const
{ 
  SM_ASSERT_DEFINED(this) ;
  SmPoint3d sRet = rPoint;
  if (rPoint.x < m_vMin.x) sRet.x = m_vMin.x; 
  if (rPoint.y < m_vMin.y) sRet.y = m_vMin.y;
  if (rPoint.z < m_vMin.z) sRet.z = m_vMin.z;
  if (rPoint.x > m_vMax.x) sRet.x = m_vMax.x;
  if (rPoint.y > m_vMax.y) sRet.y = m_vMax.y;
  if (rPoint.z > m_vMax.z) sRet.z = m_vMax.z;
  return sRet;
} // end SmExtent3d::ClampPoint3d

/*******************************************************************//**
PURPOSE: If the point is within tol of a boundary, clamp point to the 
    boundary of the extent. 

NOTES: 
***********************************************************************/
SmPoint3d SmExtent3d::SnapPoint3dToBoundary
  (const SmPoint3d & rPoint,
   double            dTol) 
  const
{ 
  SM_ASSERT_DEFINED(this) ;
  SmPoint3d sRet = rPoint;
  if ((rPoint.x > m_vMin.x - dTol) && (rPoint.x < m_vMin.x + dTol)) sRet.x = m_vMin.x; 
  if ((rPoint.y > m_vMin.y - dTol) && (rPoint.y < m_vMin.y + dTol)) sRet.y = m_vMin.y;
  if ((rPoint.z > m_vMin.z - dTol) && (rPoint.z < m_vMin.z + dTol)) sRet.z = m_vMin.z;
  if ((rPoint.x > m_vMax.x - dTol) && (rPoint.x < m_vMax.x + dTol)) sRet.x = m_vMax.x;
  if ((rPoint.y > m_vMax.y - dTol) && (rPoint.y < m_vMax.y + dTol)) sRet.y = m_vMax.y;
  if ((rPoint.z > m_vMax.z - dTol) && (rPoint.z < m_vMax.z + dTol)) sRet.z = m_vMax.z;

  return sRet;

} // end SmExtent3d::SnapPoint3dToBoundary

/*******************************************************************//**
PURPOSE: Compute the extent which is the union of the two input
   extents.   

NOTES: The result contains both of the input extents. 
***********************************************************************/
void SmExtent3d::Union
  (const SmExtent3d & crOther, 
   SmExtent3d & rResult) 
 const
{
    SM_ASSERT_DEFINED(this) ; 
    SM_ASSERT_DEFINED(&crOther) ; 
    rResult.m_vMin.x = smos_Min ( m_vMin.x, crOther.m_vMin.x );
    rResult.m_vMax.x = smos_Max ( m_vMax.x, crOther.m_vMax.x );

    rResult.m_vMin.y = smos_Min ( m_vMin.y, crOther.m_vMin.y );
    rResult.m_vMax.y = smos_Max ( m_vMax.y, crOther.m_vMax.y );

    rResult.m_vMin.z = smos_Min ( m_vMin.z, crOther.m_vMin.z );
    rResult.m_vMax.z = smos_Max ( m_vMax.z, crOther.m_vMax.z );

} // end SmExtent3d::Union

/*******************************************************************//**
PURPOSE: Find the maximum possible distance squared between two points
    in the union of the two input extents.

NOTES: Making this a Squared operation allows us to optimize
    and reduce the number of Square Roots performed.
***********************************************************************/
double SmExtent3d::MaximumDistanceSquared
  (const SmExtent3d & crOther) 
 const
{
    SmExtent3d sUnion;
    Union(crOther,sUnion);
    SmVector3d sDistVec = sUnion.m_vMax - sUnion.m_vMin;
    return sDistVec.LengthSquared();

} // end SmExtent3d::MaximumDistanceSquared

/*******************************************************************//**
PURPOSE: Find the minimum possible distance squared between two points 
    which are each in their respective extents.  

NOTES: If the extents are not
    disjoint, then 0.0 will be returned.
    Making this a Squared operation allows us to optimize
    and reduce the number of Square Roots performed.
***********************************************************************/
double SmExtent3d::MinimumDistanceSquared
  (const SmExtent3d & crOther) 
 const
{
    SM_ASSERT_DEFINED(this);
    SM_ASSERT_DEFINED(&crOther);

    SmVector3d sDistVec(0.0,0.0,0.0);

    if (m_vMin.x > crOther.m_vMax.x) sDistVec.x = sDistVec.x + (m_vMin.x - crOther.m_vMax.x);
    if (m_vMax.x < crOther.m_vMin.x) sDistVec.x = sDistVec.x + (crOther.m_vMin.x - m_vMax.x);

    if (m_vMin.y > crOther.m_vMax.y) sDistVec.y = sDistVec.y + (m_vMin.y - crOther.m_vMax.y);
    if (m_vMax.y < crOther.m_vMin.y) sDistVec.y = sDistVec.y + (crOther.m_vMin.y - m_vMax.y);

    if (m_vMin.z > crOther.m_vMax.z) sDistVec.z = sDistVec.z + (m_vMin.z - crOther.m_vMax.z);
    if (m_vMax.z < crOther.m_vMin.z) sDistVec.z = sDistVec.z + (crOther.m_vMin.z - m_vMax.z);

    return sDistVec.LengthSquared();

} // end SmExtent3d::MinimumDistanceSquared

/*******************************************************************//**
PURPOSE: Find the maximum possible distance between two points
    in the union of the two input extents.

NOTES: 
***********************************************************************/
double SmExtent3d::MaximumDistance
  (const SmExtent3d & crOther) 
 const
{
    SM_ASSERT_DEFINED(this) ; 
    SM_ASSERT_DEFINED(&crOther) ; 
    return smos_Sqrt(MaximumDistanceSquared(crOther));

} // end SmExtent3d::MaximumDistance

/*******************************************************************//**
PURPOSE: Find the minimum possible distance squared between two points 
    which are each in their respective extents.  

NOTES: If the extents are not 
    disjoint, then 0.0 will be returned.
***********************************************************************/
double SmExtent3d::MinimumDistance
  (const SmExtent3d & crOther) 
 const
{
    SM_ASSERT_DEFINED(this) ; 
    SM_ASSERT_DEFINED(&crOther) ; 
    return smos_Sqrt(MinimumDistanceSquared(crOther));

} // end SmExtent3d::MinimumDistance

/*******************************************************************//**
PURPOSE: Find the minimum distance from this box to a plane.

NOTES:
  Assumes the plane normal is unit length.
    If not, the resulting distance will be scaled by the length of the normal.
  The distance is signed according to the direction of the plane normal.
  If the plane intersects the box, 0.0 is returned.
  Failure case: zero-length plane normal: returns SM_BIG_DOUBLE.
***********************************************************************/
  double SmExtent3d::DistanceToPlane(
      const SmPoint3d  & crPlanePt,
      const SmVector3d & crPlaneUnitNorm
  ) const
{
  double dMinDist = SM_BIG_DOUBLE;
  double dThisDist;
  SmPoint3d sThisPt;

  int i, j, k;
  double x, y, z;

  x = m_vMin.x;
  for ( i = 0; i < 2; i++ )
  {
      y = m_vMin.y;
      for ( j = 0; j < 2; j++ )
      {
          z = m_vMin.z;
          for ( k = 0; k < 2; k++ )
          {
              sThisPt.Set( x, y, z );
              dThisDist = crPlaneUnitNorm.Dot( sThisPt - crPlanePt );
              if ( dMinDist < SM_BIG_DOUBLE-1 && dThisDist * dMinDist <= 0 )
                { return 0.0; }  // opposite sides of (or in) plane

              if ( smos_Fabs( dThisDist ) < smos_Fabs( dMinDist ) )
              {
                  dMinDist = dThisDist;
              }
              z = m_vMax.z;
          }
          y = m_vMax.y;
      }
      x = m_vMax.x;
  }

  return dMinDist;
}

/*******************************************************************//**
PURPOSE: Compute an extent which represents the intersection of 
    two existing non-disjoint extents.

NOTES: Disjoint extents return an error.
***********************************************************************/
SmStatus SmExtent3d::Intersect
  (const SmExtent3d & crOther, 
   SmExtent3d       & rResult) 
 const
{
    SM_ASSERT_DEFINED(this) ; 
    SM_ASSERT_DEFINED(&crOther) ; 
    
    SmStatus sRet = SM_SUCCESS;
    if (AreDisjoint(crOther)) {
        SE(SM_ERR_INVALID_INPUT);
        sRet = SM_ERR_INVALID_INPUT;
    }
    else {
        rResult.m_vMin.x = smos_Max ( m_vMin.x, crOther.m_vMin.x );
        rResult.m_vMax.x = smos_Min ( m_vMax.x, crOther.m_vMax.x );
        
        rResult.m_vMin.y = smos_Max ( m_vMin.y, crOther.m_vMin.y );
        rResult.m_vMax.y = smos_Min ( m_vMax.y, crOther.m_vMax.y );
        
        rResult.m_vMin.z = smos_Max ( m_vMin.z, crOther.m_vMin.z );
        rResult.m_vMax.z = smos_Min ( m_vMax.z, crOther.m_vMax.z );
    }
    return sRet;

} // end SmExtent3d::Intersect

/*******************************************************************//**
PURPOSE: Expand the extent by a factor of the size of the box.

NOTES: 0.5 expands each side of the box by 1/2 the side of
    the diagonal length.  
***********************************************************************/
SmExtent3d & SmExtent3d::ExpandRelative
  (double dExpansionFactor)
{
  SM_ASSERT_DEFINED(this) ; 
  SmVector3d sExpVec = GetSize() * dExpansionFactor;

  m_vMin.x -= (!SM_IS_INFINITE(m_vMin.x)) ? sExpVec.x : 0.0 ;
  m_vMin.y -= (!SM_IS_INFINITE(m_vMin.y)) ? sExpVec.y : 0.0 ;
  m_vMin.z -= (!SM_IS_INFINITE(m_vMin.z)) ? sExpVec.z : 0.0 ;
  m_vMax.x += (!SM_IS_INFINITE(m_vMax.x)) ? sExpVec.x : 0.0 ; 
  m_vMax.y += (!SM_IS_INFINITE(m_vMax.y)) ? sExpVec.y : 0.0 ; 
  m_vMax.z += (!SM_IS_INFINITE(m_vMax.z)) ? sExpVec.z : 0.0 ; 

  return(*this) ;

} // end SmExtent3d::ExpandRelative

/*******************************************************************//**
PURPOSE: Scale the extent.

NOTES: 
***********************************************************************/
SmExtent3d & SmExtent3d::Scale
 (double dScale,        // in : U dir scale factor
  double *pOptScaleV,   // in : V dir scale factor, NULL = use dScale, default:[NULL]
  double *pOptScaleW)   // in : W dir scale factor, NULL = use dScale, default:[NULL]
{
  double dScaleV = pOptScaleV ? *pOptScaleV : dScale ;
  double dScaleW = pOptScaleW ? *pOptScaleW : dScale ;

  SM_ASSERT_DEFINED(this);
  m_vMin.x *= !SM_IS_INFINITE(m_vMin.x) ? dScale  : 1.0 ;
  m_vMax.x *= !SM_IS_INFINITE(m_vMax.x) ? dScale  : 1.0 ;
              
  m_vMin.y *= !SM_IS_INFINITE(m_vMin.y) ? dScaleV : 1.0 ;
  m_vMax.y *= !SM_IS_INFINITE(m_vMax.y) ? dScaleV : 1.0 ;
                                                 
  m_vMin.z *= !SM_IS_INFINITE(m_vMin.z) ? dScaleW : 1.0 ;
  m_vMax.z *= !SM_IS_INFINITE(m_vMax.z) ? dScaleW : 1.0 ;

  return(*this) ;

} // end SmExtent3d::Scale

/*******************************************************************//**
PURPOSE: Expand the extent by a value in all directions.

NOTES: dExpansion can be positive or negative
***********************************************************************/
SmExtent3d & SmExtent3d::ExpandAbsolute
  (double dExpansion)
{
  SM_ASSERT_DEFINED(this) ;

  m_vMin.x -= !SM_IS_INFINITE(m_vMin.x) ? dExpansion : 0.0 ;
  m_vMin.y -= !SM_IS_INFINITE(m_vMin.y) ? dExpansion : 0.0 ;
  m_vMin.z -= !SM_IS_INFINITE(m_vMin.z) ? dExpansion : 0.0 ;
  m_vMax.x += !SM_IS_INFINITE(m_vMax.x) ? dExpansion : 0.0 ; 
  m_vMax.y += !SM_IS_INFINITE(m_vMax.y) ? dExpansion : 0.0 ; 
  m_vMax.z += !SM_IS_INFINITE(m_vMax.z) ? dExpansion : 0.0 ; 

  // all done
  return(*this) ;

} // end SmExtent3d::ExpandAbsolute

/*******************************************************************//**
PURPOSE: Get the size of the extent.  It basically returns the diagonal
    to the extent.

NOTES: 
***********************************************************************/
SmVector3d SmExtent3d::GetSize
  () 
 const
{
    SM_ASSERT_DEFINED(this) ; 
  return(SmPoint3d(m_vMax.x-m_vMin.x,m_vMax.y-m_vMin.y,m_vMax.z-m_vMin.z)) ;

} // end SmExtent3d::GetSize

/*******************************************************************//**
PURPOSE: Get corners of Extent  

NOTES: Corner order fixed as
       { 000 010 100 110
         001 011 101 111 }
***********************************************************************/
void SmExtent3d::GetCorners
 (SmTArray<SmPoint3d> &rCornerPoints) // out: corners ordered:{ 000 010 100 110 001 011 101 111 }
 const
{
  // init output
  rCornerPoints.SetSize(8) ;

  // Evaluate and load corner points - order is guaranteed - don't change
  rCornerPoints[0] = Evaluate(0,0,0) ;
  rCornerPoints[1] = Evaluate(0,1,0) ;
  rCornerPoints[2] = Evaluate(1,0,0) ;
  rCornerPoints[3] = Evaluate(1,1,0) ;

  rCornerPoints[4] = Evaluate(0,0,1) ;
  rCornerPoints[5] = Evaluate(0,1,1) ;
  rCornerPoints[6] = Evaluate(1,0,1) ;
  rCornerPoints[7] = Evaluate(1,1,1) ;

} // end SmExtent3d::GetCorners

/*******************************************************************//**
PURPOSE: Get Edge endpoints of Extent  

NOTES: 
***********************************************************************/
void SmExtent3d::GetEdges
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

} // end SmExtent3d::GetCorners

/*******************************************************************//**
PURPOSE: Get the size of the extent in the x direction.  

NOTES: 
***********************************************************************/
double SmExtent3d::XLength() const
{
    SM_ASSERT_DEFINED(this) ; 
  return(m_vMax.x - m_vMin.x) ;
}

/*******************************************************************//**
PURPOSE: Get the size of the extentin the y direction.

NOTES: 
***********************************************************************/
double SmExtent3d::YLength() const
{
    SM_ASSERT_DEFINED(this) ; 
  return(m_vMax.y - m_vMin.y) ;
}

/*******************************************************************//**
PURPOSE: Get the size of the extentin the z direction.

NOTES: 
***********************************************************************/
double SmExtent3d::ZLength() const
{
  SM_ASSERT_DEFINED(this) ; 
  return(m_vMax.z - m_vMin.z) ;
}

/*******************************************************************//**
PURPOSE: Compute the center and radius of a sphere which encloses the
    extent.

NOTES: 
***********************************************************************/
void SmExtent3d::ComputeSphereBound
  (SmVector3d & rSphereCenter,       // out: Bounding Box Center
   double     & rdSphereRadius,      // out: Radius of sphere that contains all BBox
   double     * pOptMinSphereRadius)   // out: Radius of sphere totally containd by BBox
  const
{
  SM_ASSERT_DEFINED(this) ; 
  rSphereCenter   = Evaluate(0.5,0.5,0.5);
  SmPoint3d sSize = Evaluate(1.0,1.0,1.0);
  rdSphereRadius  = sSize.DistanceBetween(rSphereCenter);

  if(pOptMinSphereRadius)
    {
      double dXSize = smos_Fabs(rSphereCenter.x - sSize.x) ;
      double dYSize = smos_Fabs(rSphereCenter.y - sSize.y) ;
      double dZSize = smos_Fabs(rSphereCenter.z - sSize.z) ;
      double dSize  = smos_Min(dXSize, dYSize) ;
      *pOptMinSphereRadius = smos_Min(dSize, dZSize) ;
    }   

} // end SmExtent3d::ComputeSphereBound

/*******************************************************************//**
PURPOSE:  Given two extents, compute the conical bound of one 
    one extent to the other.  

NOTES: The cone bound is a vector field that
    represents all possible vectors between points in this going to
    points in crOther.
***********************************************************************/
SmStatus SmExtent3d::ComputeConeBoundTo
  (const SmExtent3d & crOther,     // in : range of target points
  SmVector3d & rConeVector,        // out:
  double & rdConeAngleInRadians)   // out:
 const
{
  SM_ASSERT_DEFINED(this) ; 
  SM_ASSERT_DEFINED(&crOther) ; 
  SmExtent3d sTempBox      = crOther;
  SmPoint3d  sHalfThisSize = GetSize()/2.0;

  // Expand either side of sTempBox by corresponding half sizes of this.
  sTempBox.m_vMin = sTempBox.m_vMin - sHalfThisSize;
  sTempBox.m_vMax = sTempBox.m_vMax + sHalfThisSize;

  // Now compute sphere of the box.
  SmPoint3d sSphCent;
  double dSphRad = 0.0;
  sTempBox.ComputeSphereBound(sSphCent,dSphRad);

  // Get the center of this and compute vector to sphere center
  SmPoint3d sThisCent            = Evaluate(0.5,0.5,0.5);
  SmVector3d sVec                = sSphCent - sThisCent;
  double dDistanceBetweenCenters = sVec.Length();

  // Test for case where sphere surrounds point.
  SmStatus sStatus = SM_SUCCESS;
  double dScale = (1.0 + sSphCent.GetMaxDimension());
  if (dDistanceBetweenCenters < dSphRad+SM_EFF_ZERO*dScale) 
    {
      if (sVec.LengthSquared() > SM_EFF_ZERO*dScale) 
        {
          // let vector between centers be coneVector
          rConeVector = sVec;
          if (rConeVector.Unitize() != SM_SUCCESS) 
            {
              sStatus = SM_ERR;
              SE(SM_ERR);
            }
        }
      else 
        { // it makes no difference what the vector is in this case
          rConeVector.Set(0,0,1);  
        }

      // when one box is within the other - all directions are possible
      rdConeAngleInRadians = SM_PI;
    } // end center within sphere check
  else 
    {
      // Now compute angle
      rdConeAngleInRadians = smos_ArcSine(dSphRad/dDistanceBetweenCenters);
      rConeVector = sVec;
    }

  return sStatus;

} // end SmExtent3d::ComputeConeBoundTo

/*******************************************************************//**
PURPOSE: Subdivide the bounding box in the largest direction.

NOTES: 
***********************************************************************/
SmStatus SmExtent3d::SubdivideLargestDirection
  (SmExtent3d & rMinExtent,
   SmExtent3d & rMaxExtent,
   ULONG & rlDirection) 
  const
{
    SM_ASSERT_DEFINED(this) ; 
    rMinExtent = *this;
    rMaxExtent = *this;
    SmVector3d sSize = GetSize();
    if (sSize.x >= sSize.y && sSize.x >= sSize.z) { // X greatest
        double dSplit = (m_vMin.x + m_vMax.x) / 2.0;
        rMinExtent.m_vMax.x = dSplit;
        rMaxExtent.m_vMin.x = dSplit;
        rlDirection = 0;
    }
    else if (sSize.y > sSize.z) { // Y greatest
        double dSplit = (m_vMin.y + m_vMax.y) / 2.0;
        rMinExtent.m_vMax.y = dSplit;
        rMaxExtent.m_vMin.y = dSplit;
        rlDirection = 1;
    }
    else {
        double dSplit = (m_vMin.z + m_vMax.z) / 2.0;
        rMinExtent.m_vMax.z = dSplit;
        rMaxExtent.m_vMin.z = dSplit;
        rlDirection = 2;
    }
    return SM_SUCCESS;

} // end SmExtent3d::SubdivideLargestDirection

/*******************************************************************//**
PURPOSE: Determines if the two extents are disjoint.

NOTES: If they touch in any way, they are not disjoint.
***********************************************************************/
SmBoolean SmExtent3d::AreDisjoint
  (const SmExtent3d & crOther,       // in: other extent to test     
   double             dTol)          // in: tol added to other extent
 const
{
    SM_ASSERT_DEFINED(this) ; 
    SM_ASSERT_DEFINED(&crOther) ; 
    SmBoolean bRet = FALSE;
    if      (m_vMin.x > crOther.m_vMax.x + dTol) bRet = TRUE;
    else if (m_vMin.y > crOther.m_vMax.y + dTol) bRet = TRUE;
    else if (m_vMin.z > crOther.m_vMax.z + dTol) bRet = TRUE;

    else if (m_vMax.x < crOther.m_vMin.x - dTol) bRet = TRUE;
    else if (m_vMax.y < crOther.m_vMin.y - dTol) bRet = TRUE;
    else if (m_vMax.z < crOther.m_vMin.z - dTol) bRet = TRUE;
    return bRet;

} // end SmExtent3d::AreDisjoint

/*******************************************************************//**
PURPOSE: Determine if one extent is completely contained by 
    an other extent.   

NOTES: 
***********************************************************************/
SmBoolean SmExtent3d::IsContainedBy
  (const  SmExtent3d & crOther,      // in: other extent to test
   double              dTol)         // in: tol added to other extent
 const
{
    SM_ASSERT_DEFINED(this) ; 
    SM_ASSERT_DEFINED(&crOther) ; 
    SmBoolean bRet = TRUE;
    if      (m_vMin.x < crOther.m_vMin.x - dTol) bRet = FALSE;
    else if (m_vMin.y < crOther.m_vMin.y - dTol) bRet = FALSE;
    else if (m_vMin.z < crOther.m_vMin.z - dTol) bRet = FALSE;
    
    else if (m_vMax.x > crOther.m_vMax.x + dTol) bRet = FALSE;
    else if (m_vMax.y > crOther.m_vMax.y + dTol) bRet = FALSE;
    else if (m_vMax.z > crOther.m_vMax.z + dTol) bRet = FALSE;
    return bRet;

} // end SmExtent3d::IsContainedBy

/*******************************************************************//**
PURPOSE: return TRUE when size of extent is less than given tolerancer   

NOTES: 
***********************************************************************/
void SmExtent3d::GetPlanes        
 (SmTArray<SmPoint3d>  & rPlanePoints,     // out: Point on each SmExtent3d bounding plane 
  SmTArray<SmVector3d> & rPlaneNormals)    // out: Associate normal on each SmExtent3d bounding plane
 const 
{
  // init outputs
  rPlanePoints.SetSize(6) ;
  rPlaneNormals.SetSize(6) ; 

  // load plane arrays
  rPlanePoints[0] = m_vMin ;  rPlaneNormals[0].Set( 1, 0, 0) ;  
  rPlanePoints[1] = m_vMin ;  rPlaneNormals[1].Set( 0, 1, 0) ;  
  rPlanePoints[2] = m_vMin ;  rPlaneNormals[2].Set( 0, 0, 1) ;  
                                                                
  rPlanePoints[3] = m_vMax ;  rPlaneNormals[3].Set(-1, 0, 0) ;  
  rPlanePoints[4] = m_vMax ;  rPlaneNormals[4].Set( 0,-1, 0) ;  
  rPlanePoints[5] = m_vMax ;  rPlaneNormals[5].Set( 0, 0,-1) ;  

} // end SmExtent3d::IsCurveIn

/*******************************************************************//**
PURPOSE: return TRUE when size of extent is less than given tolerancer   

NOTES: 
***********************************************************************/
SmBoolean SmExtent3d::IsPointSized
  (double  dTol)         // in: min distance between points
 const
{
  SM_ASSERT_DEFINED(this) ;
  SM_ASSERT(!HasNegativeVolume()) ; 

  if(smos_Fabs(m_vMax.x - m_vMin.x) > dTol) { return FALSE ; }
  if(smos_Fabs(m_vMax.y - m_vMin.y) > dTol) { return FALSE ; }
  if(smos_Fabs(m_vMax.z - m_vMin.z) > dTol) { return FALSE ; }

  // extent is point sized
  return(TRUE) ;

} // end SmExtent3d::IsPointSized

/*******************************************************************//**
PURPOSE: Determine if the 3D extent contains a 3D point.  

NOTES: The tolerance
   is used to allow points which are very close to the boundary but outside
   to be considered inside of the extent. 
***********************************************************************/
SmBoolean SmExtent3d::ContainsPoint3d
  (const SmPoint3d & rPoint,        // in: Point to test   
   double            dTol)          // in: tol added to extent
  const
{ 
    SM_ASSERT_DEFINED(this) ; 
    SmBoolean bRet = TRUE;
    if      (rPoint.x < m_vMin.x - dTol) bRet = FALSE;
    else if (rPoint.y < m_vMin.y - dTol) bRet = FALSE;
    else if (rPoint.z < m_vMin.z - dTol) bRet = FALSE;
    else if (rPoint.x > m_vMax.x + dTol) bRet = FALSE;
    else if (rPoint.y > m_vMax.y + dTol) bRet = FALSE;
    else if (rPoint.z > m_vMax.z + dTol) bRet = FALSE;
    return bRet;

} // end SmExtent3d::ContainsPoint3d

/*******************************************************************//**
PURPOSE: Determine if the 3D extent contains a 3D point.  

NOTES: The tolerance
   is used to allow points which are very close to the boundary but outside
   to be considered inside of the extent. 
***********************************************************************/
SmBoolean SmExtent3d::ContainsLineSeg3d
  (const SmPoint3d & rStartPoint,   // in: Point to test   
   const SmPoint3d & rEndPoint,     // in: Point to test   
   double            dTol)          // in: tol added to extent
  const
{ 
  // Extent3d is convex so it contains line segment if it contains both points
  SmBoolean bRtn = ContainsPoint3d(rStartPoint, dTol) ;
  bRtn          |= ContainsPoint3d(rEndPoint, dTol) ;

  // all done
  return bRtn;

} // end SmExtent3d::ContainsPoint3d

/*******************************************************************//**
PURPOSE: Determine if the 3D extent contains a 3D point
   within a relative tolerance.

NOTES: [AbsTolU = RelativeTol * (MaxU - MinU)]
       [AbsTolV = RelativeTol * (MaxV - MinV)]
       [AbsTolW = RelativeTol * (MaxW - MinW)]
***********************************************************************/
SmBoolean SmExtent3d::ContainsPoint3dRelative
 (const SmPoint3d & crPoint,        // in: Point to test
  double            dRelativeTol )  // in: relative tol added to extent [AbsTol = RelativeTol * (Max - Min)]
 const
{
  SM_ASSERT_DEFINED(this);

  if ( ! GetUInterval().ContainsValueRelative( crPoint.x, dRelativeTol ) )
    { return FALSE; }

  if ( ! GetVInterval().ContainsValueRelative( crPoint.y, dRelativeTol ) )
    { return FALSE; }

  if ( ! GetWInterval().ContainsValueRelative( crPoint.z, dRelativeTol ) )
    { return FALSE; }

  return TRUE;

} // end SmExtent3d::ContainsValueRelative

/*******************************************************************//**
PURPOSE: Return TRUE if line intersects bounding box to within tol

NOTES: 
***********************************************************************/
SmBoolean SmExtent3d::IntersectsLine3d
  (const SmPoint3d  & crLinePoint,     // in : point on line
   const SmVector3d & crLineVector,    // in : vector defining line's direction
   double             dTol)            // in : tol added to extent
 const
{
  SM_ASSERT_DEFINED(this) ; 
  
  // treat degenerate lines as points
  if (crLineVector.LengthSquared() < SM_EFF_ZERO_SQ) // gwc: was SM_EFF_ZERO_SQRT which looked like a typo to me
    { return( ContainsPoint3d(crLinePoint, dTol) ) ; }

  // line params for bounding plane intersections
  SmPoint3d sTMin, sTMax;

  // trim X coordinates to bounding box
  if (crLineVector.x > SM_EFF_ZERO) 
    {
      sTMin.x = (m_vMin.x - dTol - crLinePoint.x) / crLineVector.x;
      sTMax.x = (m_vMax.x + dTol - crLinePoint.x) / crLineVector.x;
    }
  else if (crLineVector.x < -SM_EFF_ZERO) 
    {
      sTMax.x = (m_vMin.x - dTol - crLinePoint.x) / crLineVector.x;
      sTMin.x = (m_vMax.x + dTol - crLinePoint.x) / crLineVector.x;
    }
  else 
    {
      sTMin.x = - SM_BIG_DOUBLE;
      sTMax.x =   SM_BIG_DOUBLE;
    }
  
  // trim Y coordinates to bounding box
  if (crLineVector.y > SM_EFF_ZERO) 
    {
      sTMin.y = (m_vMin.y - dTol - crLinePoint.y) / crLineVector.y;
      sTMax.y = (m_vMax.y + dTol - crLinePoint.y) / crLineVector.y;
    }
  else if (crLineVector.y < -SM_EFF_ZERO) 
    {
      sTMax.y = (m_vMin.y - dTol - crLinePoint.y) / crLineVector.y;
      sTMin.y = (m_vMax.y + dTol - crLinePoint.y) / crLineVector.y;
    }
  else 
    {
      sTMin.y = - SM_BIG_DOUBLE;
      sTMax.y =   SM_BIG_DOUBLE;
    }
  
  //  trim Z coordinates to bounding box
  if (crLineVector.z > SM_EFF_ZERO) 
    {
      sTMin.z = (m_vMin.z - dTol - crLinePoint.z) / crLineVector.z;
      sTMax.z = (m_vMax.z + dTol - crLinePoint.z) / crLineVector.z;
    }
  else if (crLineVector.z < -SM_EFF_ZERO) 
    {
      sTMax.z = (m_vMin.z - dTol - crLinePoint.z) / crLineVector.z;
      sTMin.z = (m_vMax.z + dTol - crLinePoint.z) / crLineVector.z;
    }
  else 
    {
      sTMin.z = - SM_BIG_DOUBLE;
      sTMax.z =   SM_BIG_DOUBLE;
    }
  
  // get last MinPlane and first MaxPlane intersections
  double dMin = smos_3Max(sTMin.z, sTMin.x, sTMin.y);
  double dMax = smos_3Min(sTMax.z, sTMax.x, sTMax.y);

  
  // test MidPoint for Box containment
  SmPoint3d sMid = crLinePoint + ((dMin+dMax)/2.0) * crLineVector;
  if(dTol < SM_EFF_ZERO * (1.0 + sMid.GetMaxDimension()))
    { dTol = SM_EFF_ZERO * (1.0 + sMid.GetMaxDimension()) ; }

  // line intersects when midPoint is contained and interval is degenerate or positive
  SmBoolean bRtn = (   ContainsPoint3d(sMid,dTol)
                    && dMin < dMax + SM_EFF_ZERO) ;
  return(bRtn) ;

} // end SmExtent3d::IntersectsLine3d

/*******************************************************************//**
PURPOSE: Equality operator 

NOTES: 
***********************************************************************/
SmBoolean SmExtent3d::operator==
  (const SmExtent3d& crOther) 
 const
{
  // quick check 
  if(this == &crOther) { return TRUE ; }
                                             
  if (   SM_ARE_SAME(m_vMin.x, crOther.m_vMin.x)
      && SM_ARE_SAME(m_vMin.y, crOther.m_vMin.y)
      && SM_ARE_SAME(m_vMin.z, crOther.m_vMin.z)
                                                         
      && SM_ARE_SAME(m_vMax.x, crOther.m_vMax.x)
      && SM_ARE_SAME(m_vMax.y, crOther.m_vMax.y)
      && SM_ARE_SAME(m_vMax.z, crOther.m_vMax.z) )
    {
      return TRUE;
    }
  return FALSE;

} // end SmExtent3d::operator==

/*******************************************************************//**
PURPOSE: Classify Point with respect to each orthogonal 1d extent.

NOTES: 
***********************************************************************/
void SmExtent3d::ClassifyPoint3d
  (const SmPoint3d & rPoint,          // in : target point
   SmExtentPointType &rExtentUType,   // out: Classification of point for U extent
   SmExtentPointType &rExtentVType,   // out: Classification of point for V extent
   SmExtentPointType &rExtentWType,   // out: Classification of point for W extent
   double dTol)                       // in : max allowed param distance to count as being on a boundary
  const
{
  SM_ASSERT_DEFINED(this);

  rExtentUType =    (smos_Fabs(rPoint.x-m_vMin.x) <= dTol) &&
                    (smos_Fabs(rPoint.x-m_vMax.x) <= dTol) ? SM_EP_BOTH
                  : (smos_Fabs(rPoint.x-m_vMin.x) <= dTol) ? SM_EP_START
                  : (smos_Fabs(rPoint.x-m_vMax.x) <= dTol) ? SM_EP_END
                  : (rPoint.x-m_vMin.x >= 0.0) &&
                    (m_vMax.x-rPoint.x >= 0.0)             ? SM_EP_INSIDE
                  : SM_EP_OUTSIDE ;

  rExtentVType =    (smos_Fabs(rPoint.y-m_vMin.y) <= dTol) &&
                    (smos_Fabs(rPoint.y-m_vMax.y) <= dTol) ? SM_EP_BOTH
                  : (smos_Fabs(rPoint.y-m_vMin.y) <= dTol) ? SM_EP_START
                  : (smos_Fabs(rPoint.y-m_vMax.y) <= dTol) ? SM_EP_END
                  : (rPoint.y-m_vMin.y >= 0.0) &&
                    (m_vMax.y-rPoint.y >= 0.0)             ? SM_EP_INSIDE
                  : SM_EP_OUTSIDE ;

  rExtentWType =    (smos_Fabs(rPoint.z-m_vMin.z) <= dTol) &&
                    (smos_Fabs(rPoint.z-m_vMax.z) <= dTol) ? SM_EP_BOTH
                  : (smos_Fabs(rPoint.z-m_vMin.z) <= dTol) ? SM_EP_START
                  : (smos_Fabs(rPoint.z-m_vMax.z) <= dTol) ? SM_EP_END
                  : (rPoint.z-m_vMin.z >= 0.0) &&
                    (m_vMax.z-rPoint.z >= 0.0)             ? SM_EP_INSIDE
                  : SM_EP_OUTSIDE ;

} // end SmExtent3d::ClassifyPoint3d

/*******************************************************************//**
PURPOSE: Determine if the 2D point is on the extent boundary 
            to within the given tolerance value.

NOTES: return TRUE when crPoint is within dTol of any boundary
***********************************************************************/
SmBoolean SmExtent3d::IsPoint3dOnBoundary
  (const SmPoint3d & crPoint,    // in : point to test
   double            dTol,       // in : max allowed distance between point and boundary, default:[SM_EFF_ZERO]
   SmVector3d      * pOptBiNorm) // out: UVVector pointing to interior from 1st OnBoundary found
                                 //      NULL to ignore. default:[NULL]
 const
{
  SM_ASSERT_DEFINED(this);

  SmBoolean bRet = FALSE;
  double du=0.0, dv=0.0, dw=0.0 ;
  if      (smos_Fabs(crPoint.x-m_vMin.x) <= dTol) { bRet = TRUE; du= 1.0 ; }
  else if (smos_Fabs(crPoint.x-m_vMax.x) <= dTol) { bRet = TRUE; du=-1.0 ; }
  if      (smos_Fabs(crPoint.y-m_vMin.y) <= dTol) { bRet = TRUE; dv= 1.0 ; }
  else if (smos_Fabs(crPoint.y-m_vMax.y) <= dTol) { bRet = TRUE; dv=-1.0 ; }
  if      (smos_Fabs(crPoint.z-m_vMin.z) <= dTol) { bRet = TRUE; dw= 1.0 ; }
  else if (smos_Fabs(crPoint.z-m_vMax.z) <= dTol) { bRet = TRUE; dw=-1.0 ; }
  if(pOptBiNorm) { pOptBiNorm->Set(du,dv,dw) ; }
  return bRet;

} // end SmExtent2d::IsPoint3dOnBoundary

/*******************************************************************//**
PURPOSE: Intersect a line and the bounding box.

NOTES: supports any mixture of Unbounded Extent intervals
***********************************************************************/
SmStatus SmExtent3d::IntersectLine
  (const SmPoint3d  & crLinePoint,     // in : point on line
   const SmVector3d & crLineVector,    // in : vector defining line's direction
   ULONG            & rlNumFound,      // out: number of intersections 
                                       //      0 = there is no intersection.  
                                       //      1 = grazes a corner.
                                       //      2 = portion of ray is inside box.
   double           & rdTEnter,        // out: entering ray parameter of ray/box xsect
   double           & rdTExit)         // out: exiting  ray parameter of ray/box xsect
  const
{
  SmStatus sRet = SM_SUCCESS;
  
  // check input
  SM_ASSERT(   AssertDefined()
            && crLineVector.LengthSquared() >= SM_EFF_ZERO_SQ) ;
  if(   crLineVector.LengthSquared() < SM_EFF_ZERO_SQ
     || !AssertDefined()) 
    {
      return(SM_ERR_INVALID_INPUT) ;
    }

  // arrive here when input is valid  
  SmPoint3d sTMin, sTMax;
  if (crLineVector.x > SM_EFF_ZERO) 
    {
      sTMin.x = !SM_IS_INFINITE(m_vMin.x) ? (m_vMin.x - crLinePoint.x) / crLineVector.x : -SM_INFINITE_PARAMETER ;
      sTMax.x = !SM_IS_INFINITE(m_vMax.x) ? (m_vMax.x - crLinePoint.x) / crLineVector.x :  SM_INFINITE_PARAMETER ;
    }
  else if (crLineVector.x < -SM_EFF_ZERO) 
    {
      sTMax.x = !SM_IS_INFINITE(m_vMin.x) ? (m_vMin.x - crLinePoint.x) / crLineVector.x :  SM_INFINITE_PARAMETER ;
      sTMin.x = !SM_IS_INFINITE(m_vMax.x) ? (m_vMax.x - crLinePoint.x) / crLineVector.x : -SM_INFINITE_PARAMETER ;
    }
  else 
    {
      sTMin.x = - SM_BIG_DOUBLE ;
      sTMax.x =   SM_BIG_DOUBLE ;
    }
  
  if (crLineVector.y > SM_EFF_ZERO) 
    {
      sTMin.y = !SM_IS_INFINITE(m_vMin.y) ? (m_vMin.y - crLinePoint.y) / crLineVector.y : -SM_INFINITE_PARAMETER ;
      sTMax.y = !SM_IS_INFINITE(m_vMax.y) ? (m_vMax.y - crLinePoint.y) / crLineVector.y :  SM_INFINITE_PARAMETER ;
    }
  else if (crLineVector.y < -SM_EFF_ZERO) 
    {
      sTMax.y = !SM_IS_INFINITE(m_vMin.y) ? (m_vMin.y - crLinePoint.y) / crLineVector.y :  SM_INFINITE_PARAMETER ;
      sTMin.y = !SM_IS_INFINITE(m_vMax.y) ? (m_vMax.y - crLinePoint.y) / crLineVector.y : -SM_INFINITE_PARAMETER ;
    }
  else 
    {
      sTMin.y = - SM_BIG_DOUBLE;
      sTMax.y =   SM_BIG_DOUBLE;
    }
  
  if (crLineVector.z > SM_EFF_ZERO) 
    {
      sTMin.z = !SM_IS_INFINITE(m_vMin.z) ? (m_vMin.z - crLinePoint.z) / crLineVector.z : -SM_INFINITE_PARAMETER ;
      sTMax.z = !SM_IS_INFINITE(m_vMax.z) ? (m_vMax.z - crLinePoint.z) / crLineVector.z :  SM_INFINITE_PARAMETER ;
    }                                                                                              
  else if (crLineVector.z < -SM_EFF_ZERO)                                                          
    {                                                                                              
      sTMax.z = !SM_IS_INFINITE(m_vMin.z) ? (m_vMin.z - crLinePoint.z) / crLineVector.z :  SM_INFINITE_PARAMETER ;
      sTMin.z = !SM_IS_INFINITE(m_vMax.z) ? (m_vMax.z - crLinePoint.z) / crLineVector.z : -SM_INFINITE_PARAMETER ;
    }
  else 
    {
      sTMin.z = - SM_BIG_DOUBLE;
      sTMax.z =   SM_BIG_DOUBLE;
    }
  
  double dMin = smos_3Max(sTMin.z, sTMin.x, sTMin.y);
  double dMax = smos_3Min(sTMax.z, sTMax.x, sTMax.y);
  
  // Note: is an extreme grazing case, max (e.g.) could be extremely
  // close to the box, while min is quite far away, resulting in
  // a midpoint that is not close to the box, even though the line
  // passes well within tolerance.  So we should check both min and
  // max, instead of the midpoint.  [B70]
  // But, if it's not an extreme grazing case, then the midpoint
  // should be checked as well.  So do that first.
  // GWC Note: This extreme grazing case is switching the definition
  //           of machine precision tolerance for a tolerant intersection.
  //           The original code said, if the length of the
  //           interval running from a max plane to a min plane
  //           was less than dScaledZero, then the corner was intersected.
  //           The modification now says a machine precision miss happens
  //           whenever an end point on an interval outside the box
  //           is within dScaledZero of the corner of the box.
  //           Not implemented is a third choice which would cost even
  //           more, tolerant intersection happens when any point
  //           on the span comes within dScaledZero of the corner.
  //           This change was made because a case came up where
  //           a tolerant intersection between a curve and surf
  //           was missed in this function.  This change fixed that
  //           case, but the change was done in the wrong way.
  //           In that case, the MayHaveIntersections function needs
  //           to take into account an IntersectionTolerance, and
  //           not depend on the behavior of this algorithm's machineTolerance
  //           behavior to get that right.  So, Bug70 is running
  //           correctly, but it's not fixed.  What needs to be done
  //           is to restore this function (nobody should care
  //           how an individual function manages machine precision)
  //           and the calling function needs to be modified to properly
  //           handle intersectionTolerances to find its tolerant
  //           intersections.  That is left for a later time.

  SmPoint3d sMid = crLinePoint + ((dMin+dMax)/2.0) * crLineVector;
  double    dTol = SM_EFF_ZERO * (1.0 + sMid.GetMaxDimension());

  if ( ContainsPoint3d(sMid,dTol) )
    {
      if (smos_Fabs(dMin-dMax) < SM_EFF_ZERO) 
        {
          rlNumFound = 1;

          // avg dT to make ivl exactly degenerate and never inverted
          rdTEnter = rdTExit = (dMin + dMax) / 2.0 ;
        }
      else if (dMin < dMax) 
        {
          rlNumFound = 2;
          rdTEnter   = dMin;
          rdTExit    = dMax;
        }
      else 
        {
          rlNumFound = 0;
        }
    }
  else 
    {
      // Check min and max as well, as per comment above.
      // If one of these is close, then we have one intersection:
      // can't be two intersections if midpoint was outside.
      SmPoint3d sTest = crLinePoint + dMin * crLineVector;
      if ( ContainsPoint3d( sTest, dTol ) )
        {
          rlNumFound = 1;
          rdTEnter = rdTExit = dMin;
        }
      else
        {
          sTest = crLinePoint + dMax * crLineVector;
          if ( ContainsPoint3d( sTest, dTol ) )
            {
              rlNumFound = 1;
              rdTEnter = rdTExit = dMax;
            }
          else
            { rlNumFound = 0; }
        }
    }

  return sRet;

} // end SmExtent3d::IntersectLine 

/*******************************************************************//**
PURPOSE: Return TRUE when any dimension in Extent dimension span is negative.   

NOTES: 
***********************************************************************/
SmBoolean SmExtent3d::HasNegativeVolume
  () 
 const
{
  return(   m_vMin.x > m_vMax.x
         || m_vMin.y > m_vMax.y
         || m_vMin.z > m_vMax.z) ;

} // end SmExtent3d::HasNegativeVolume

/*******************************************************************//**
PURPOSE: Return TRUE when dimensions are all set to init values

NOTES: 
***********************************************************************/
SmBoolean SmExtent3d::IsInit
  () 
 const
{
  return(   m_vMin.x ==  SM_BIG_DOUBLE
         && m_vMin.y ==  SM_BIG_DOUBLE
         && m_vMin.z ==  SM_BIG_DOUBLE  
         && m_vMax.x == -SM_BIG_DOUBLE
         && m_vMax.y == -SM_BIG_DOUBLE
         && m_vMax.z == -SM_BIG_DOUBLE) ; 

} // end SmExtent3d::IsInit

/*******************************************************************//**
PURPOSE: Return TRUE when all dimension in Extent are positive.   

NOTES: 
***********************************************************************/
SmBoolean SmExtent3d::IsPositiveVolume
  () 
 const
{
  return(   m_vMin.x < m_vMax.x
         && m_vMin.y < m_vMax.y
         && m_vMin.z < m_vMax.z) ;

} // end SmExtent3d::IsPositiveVolume

/*******************************************************************//**
PURPOSE: Return TRUE x and y spans are both positive.   

NOTES: 
***********************************************************************/
SmBoolean SmExtent3d::IsPositiveArea
  () 
 const
{
  return(   m_vMin.x < m_vMax.x
         && m_vMin.y < m_vMax.y) ;

} // end SmExtent3d::IsPositiveArea

/*******************************************************************//**
PURPOSE: Return TRUE when all boundaries are bound,
         no +/-SM_INFINITE_PARAMETER values.

NOTES: 
***********************************************************************/
SmBoolean SmExtent3d::IsBounded
 (SmBoundaryType eOptBndryType[3])   // in : opt ptr to an array [BT_Type_U, BT_TYPE_V, BT_TYPE_W]
                                     //      NULL to ignore, default:[NULL]
 const
{
  SmBoundaryType  eBndryType[3] ;
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

  
  // classify W interval 
  pBndryType[2] =   (   SM_IS_INFINITE(m_vMax.z)
                     && SM_IS_INFINITE(m_vMin.z)) ? SM_BT_UNBOUNDED
                  : (   SM_IS_INFINITE(m_vMin.z)) ? SM_BT_UNBOUNDED_MIN
                  : (   SM_IS_INFINITE(m_vMax.z)) ? SM_BT_UNBOUNDED_MAX
                  :                                 SM_BT_BOUNDED ;

  // all done - return bounded status
  return(   pBndryType[0] == SM_BT_BOUNDED
         && pBndryType[1] == SM_BT_BOUNDED
         && pBndryType[2] == SM_BT_BOUNDED ) ;                                   

} // end SmExtent3d::IsBounded

/*******************************************************************//**
PURPOSE: Return TRUE when any boundary is bound,
         any boundary value not equal to +/-SM_INFINITE_PARAMETER values.

NOTES: 
***********************************************************************/
SmBoolean SmExtent3d::AnyBounds
 (SmBoundaryType eOptBndryType[3])   // in : opt ptr to an array [BT_Type_U, _BTTYPE_V, _BTTYPE_W]
                                     //      NULL to ignore, default:[NULL]
 const
{
  SmBoundaryType  eBndryType[3] ;
  SmBoundaryType *pBndryType = eOptBndryType ? eOptBndryType : eBndryType ;

  IsBounded(pBndryType) ;

  return(   pBndryType[0] != SM_BT_UNBOUNDED
         || pBndryType[1] != SM_BT_UNBOUNDED
         || pBndryType[2] != SM_BT_UNBOUNDED ) ;

} // end SmExtent3d::AnyBounds

/*******************************************************************//**
PURPOSE: return a bounded SmExtent3d to approximate an unbounded one

NOTES: 0. returns a SmExtent3d whose infinite boundary values 
          have been replaced by finite values.
          Those replacement values are based upon the dUnboundedHalfSize,
          pUnboundedCenter, and interval opposing end values.

       1. This is used for graphics and sampling to allow an application
          to easily define an area of focus for infinite extents.
          
       2. This method handles all the combinations of half spaces and unbounded spaces.
          
       3. The approximation of a BOUNDED extent is an exact copy of the original extent.
***********************************************************************/
SmExtent3d SmExtent3d::ApproximateUnbounded // eff: return a bounded SmExtent3d to approximate an unbounded one                  
 (SmPoint3d  *pUnboundedCenter,             // in : center of unbounded intervals, NULL = [0,0,0], default:[NULL]
  double      dUnboundedHalfSize,           // in : the size used for infinite 1/2 spaces, default:[SM_BOUNDED_INFINITE_PARAM]
                                            //      a totally unbounded cube is approximated by a cube twice this size
  SmExtent3d *pOptExpandedApprox)           // out: a 2nd extent expanded a small bit - used by graphics, NULL to ignore, default:[NULL]
 const
{
  // locals
  SmBoundaryType eBdryType[3] ;
  SmPoint3d sCenter(0,0,0) ;
  if(pUnboundedCenter) sCenter = *pUnboundedCenter ; 

  // copy min/max points
  SmPoint3d sMin    = GetMin();
  SmPoint3d sMax    = GetMax();

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

      sMin.z =   eBdryType[2] == SM_BT_BOUNDED       ? sMin.z
               : eBdryType[2] == SM_BT_UNBOUNDED_MIN ? (  (   (pUnboundedCenter == NULL) 
                                                           || (sMax.z < pUnboundedCenter->z + dUnboundedHalfSize))
                                                        ? (sMax.z - 2 * dUnboundedHalfSize)
                                                        : (pUnboundedCenter->z - dUnboundedHalfSize))
               : eBdryType[2] == SM_BT_UNBOUNDED_MAX ? (  (   (pUnboundedCenter == NULL) 
                                                           || (sMin.z > pUnboundedCenter->z - dUnboundedHalfSize))
                                                        ? (sMin.z)
                                                        : (pUnboundedCenter->z - dUnboundedHalfSize))
               : eBdryType[2] == SM_BT_UNBOUNDED     ? sCenter.z - dUnboundedHalfSize : sMin.z ;

      sMax.z =   eBdryType[2] == SM_BT_BOUNDED       ? sMax.z
               : eBdryType[2] == SM_BT_UNBOUNDED_MIN ? (  (   (pUnboundedCenter == NULL) 
                                                           || (sMax.z < pUnboundedCenter->z + dUnboundedHalfSize))
                                                        ? (sMax.z)
                                                        : (pUnboundedCenter->z + dUnboundedHalfSize))
               : eBdryType[2] == SM_BT_UNBOUNDED_MAX ? (  (   (pUnboundedCenter == NULL) 
                                                           || (sMin.z > pUnboundedCenter->z - dUnboundedHalfSize))
                                                        ? (sMin.z + 2 * dUnboundedHalfSize)
                                                        : (pUnboundedCenter->z + dUnboundedHalfSize))
               : eBdryType[2] == SM_BT_UNBOUNDED     ? sCenter.z + dUnboundedHalfSize : sMax.z ;
    
    } // end unbounded check

  // when asked - build an expanded extent as well
  if(pOptExpandedApprox)
    {
      SmPoint3d sMinInc = sMin ;
      SmPoint3d sMaxInc = sMax ;
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

          if(   eBdryType[2] == SM_BT_UNBOUNDED_MIN
             || eBdryType[2] == SM_BT_UNBOUNDED)     { sMinInc.z -= dInfInc ;  }    
          if(   eBdryType[2] == SM_BT_UNBOUNDED_MAX
             || eBdryType[2] == SM_BT_UNBOUNDED)     { sMaxInc.z += dInfInc ; }
        } // end bBounded == FALSE check
          
      // set output
      pOptExpandedApprox->SetMinMax(sMinInc, sMaxInc) ;    
        
    } // end need to build extended extent check

  // all done
  return(SmExtent3d(sMin, sMax)) ;

} // end SmExtent3d::ApproximateUnbounded

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
void SmExtent3d::Dump
  (void) 
 const
{
  SM_ASSERT_DEFINED(this) ; 
  TCHAR sBuff[SM_TBLOCK_SIZE];

  smos_sprintf(sBuff,_T("\nSmExtent3d Min = [%lf, %lf, %lf] "), m_vMin.x, m_vMin.y, m_vMin.z);
  smos_WriteBuffer(sBuff);
  smos_sprintf(sBuff,_T("\n           Max = [%lf, %lf, %lf] "), m_vMax.x, m_vMax.y, m_vMax.z);
  smos_WriteBuffer(sBuff);
  smos_sprintf(sBuff,_T("\n           Lengths = [%lf, %lf, %lf], Vol : %lf "), 
             XLength(), YLength(), ZLength(),
             XLength() * YLength() * ZLength());
  smos_WriteBuffer(sBuff);

} // end SmExtent3d::Dump

/*******************************************************************//**
PURPOSE: Add SmExtent3d Graphics to current graphics stream

NOTES: For bounded boxes - draws the box as given
       For unbounded boxes - 1. draws a finite approx of the unbounded box centered 
                                on pUnboundedCenter when given, else the origin.
                             2. draws a 2nd box slightly larger than the approx box
                                as a graphical indication that the displayed box is unbounded.
***********************************************************************/
SmDisplayList * SmExtent3d::Draw
  (const SmContext * pContext,            // NotUsed: in :
   SmPoint3d       * pUnboundedCenter,    // in : center of interest for unbounded extents
   SmGfxArraySet   * pOptGfxSet)          // i/o: used for OpenGL ES 2.0. When given output GfxVertexArrays not GL calls.
                                          //      NULL to ignore. default:[NULL]
 const
{
  SM_REF1(pContext) ;
  SmDisplayList *pRtn = NULL ;

#ifdef SM_GFX_CODE
  smgfx_Open(smgfx_GetRuleColor(),NULL,NULL,FALSE,pOptGfxSet);
  smgfx_OutputColor(smgfx_GetColor(), pOptGfxSet) ;

  // no work - SmExtent is Init
  if(IsInit())
    { return pRtn ; }

  // locals
  ULONG ii ;
  double dUnboundedHalfSize = 33 ;

  // classify for infinite boundaries
  SmBoundaryType eBdryType[3] ;  
  SmBoolean bBounded = this->IsBounded(eBdryType) ;

  // approximate unbounded extents about the pUnboundedCenter
  SmExtent3d sApproxExp ; 
  SmExtent3d sApprox = ApproximateUnbounded(pUnboundedCenter, dUnboundedHalfSize, &sApproxExp) ;

  // for two passes - 1st the bounded bbox - 2nd extra graphics for unbounded boxes
  for(ii=0;ii<2;ii++)
    {
      // low work - 2nd iteration not needed for bounded Extent3ds
      if(bBounded == TRUE && ii == 2)
        { break ; }
       
      // get the min and max points to display
      SmPoint3d mmm = ii == 0 ? sApprox.GetMin() : sApproxExp.GetMin() ;
      SmPoint3d MMM = ii == 0 ? sApprox.GetMax() : sApproxExp.GetMax() ;

      // name the other corners to be rendered
      SmPoint3d Mmm(MMM.x,mmm.y,mmm.z);
      SmPoint3d mMm(mmm.x,MMM.y,mmm.z);
      SmPoint3d mmM(mmm.x,mmm.y,MMM.z);
      SmPoint3d mMM(mmm.x,MMM.y,MMM.z);
      SmPoint3d MmM(MMM.x,mmm.y,MMM.z);
      SmPoint3d MMm(MMM.x,MMM.y,mmm.z);

      // draw bounded cube graphics
      smgfx_DrawLine(&mmm,&Mmm, pOptGfxSet);
      smgfx_DrawLine(&mmm,&mMm, pOptGfxSet);
      smgfx_DrawLine(&mmm,&mmM, pOptGfxSet);
      smgfx_DrawLine(&MMM,&MMm, pOptGfxSet);
      smgfx_DrawLine(&MMM,&MmM, pOptGfxSet);
      smgfx_DrawLine(&MMM,&mMM, pOptGfxSet);
      smgfx_DrawLine(&mMm,&mMM, pOptGfxSet);
      smgfx_DrawLine(&mmM,&mMM, pOptGfxSet);
      smgfx_DrawLine(&mMm,&MMm, pOptGfxSet);
      smgfx_DrawLine(&Mmm,&MmM, pOptGfxSet);
      smgfx_DrawLine(&Mmm,&MMm, pOptGfxSet);
      smgfx_DrawLine(&mmM,&MmM, pOptGfxSet);

      smgfx_DrawPoint(mmm.x,mmm.y,mmm.z, pOptGfxSet);

      // when working with an unbounded volume - add some more graphics - digaonals on unbounded boundaries
  
      // 1st diagonals accross infinite boundaries planes
      if(bBounded == FALSE)
        {
          SmPoint3d sP0, sP1 ;
          switch(eBdryType[0])
            {
              case SM_BT_BOUNDED       : break ;
              case SM_BT_UNBOUNDED_MIN : sP0 = mmm ; sP1 = .93*mmm + .07*mMM ; smgfx_DrawLine(&sP0, &sP1, pOptGfxSet);
                                         sP0 = mMM ; sP1 = .93*mMM + .07*mmm ; smgfx_DrawLine(&sP0, &sP1, pOptGfxSet);
                                         sP0 = mmM ; sP1 = .93*mmM + .07*mMm ; smgfx_DrawLine(&sP0, &sP1, pOptGfxSet);
                                         sP0 = mMm ; sP1 = .93*mMm + .07*mmM ; smgfx_DrawLine(&sP0, &sP1, pOptGfxSet);
                                         break ;
              case SM_BT_UNBOUNDED_MAX : sP0 = Mmm ; sP1 = .93*Mmm + .07*MMM ; smgfx_DrawLine(&sP0, &sP1, pOptGfxSet);
                                         sP0 = MMM ; sP1 = .93*MMM + .07*Mmm ; smgfx_DrawLine(&sP0, &sP1, pOptGfxSet);
                                         sP0 = MmM ; sP1 = .93*MmM + .07*mMm ; smgfx_DrawLine(&sP0, &sP1, pOptGfxSet);
                                         sP0 = MMm ; sP1 = .93*MMm + .07*MmM ; smgfx_DrawLine(&sP0, &sP1, pOptGfxSet);
                                         break ;
              case SM_BT_UNBOUNDED     : sP0 = mmm ; sP1 = .93*mmm + .07*mMM ; smgfx_DrawLine(&sP0, &sP1, pOptGfxSet);
                                         sP0 = mMM ; sP1 = .93*mMM + .07*mmm ; smgfx_DrawLine(&sP0, &sP1, pOptGfxSet);
                                         sP0 = mmM ; sP1 = .93*mmM + .07*mMm ; smgfx_DrawLine(&sP0, &sP1, pOptGfxSet);
                                         sP0 = mMm ; sP1 = .93*mMm + .07*mmM ; smgfx_DrawLine(&sP0, &sP1, pOptGfxSet);

                                         sP0 = Mmm ; sP1 = .93*Mmm + .07*MMM ; smgfx_DrawLine(&sP0, &sP1, pOptGfxSet);
                                         sP0 = MMM ; sP1 = .93*MMM + .07*Mmm ; smgfx_DrawLine(&sP0, &sP1, pOptGfxSet);
                                         sP0 = MmM ; sP1 = .93*MmM + .07*MMm ; smgfx_DrawLine(&sP0, &sP1, pOptGfxSet);
                                         sP0 = MMm ; sP1 = .93*MMm + .07*MmM ; smgfx_DrawLine(&sP0, &sP1, pOptGfxSet);
                                         break ; 
              case SM_BT_UNKNOWN       :
                                         break;
            }
                                                                             
          switch(eBdryType[1])
            {
              case SM_BT_BOUNDED       : break ;
              case SM_BT_UNBOUNDED_MIN : sP0 = mmm ; sP1 = .93*mmm + .07*MmM ; smgfx_DrawLine(&sP0, &sP1) ; // smgfx_DrawLine(&mmm,&MmM, pOptGfxSet);
                                         sP0 = MmM ; sP1 = .93*MmM + .07*mmm ; smgfx_DrawLine(&sP0, &sP1) ;
                                         sP0 = mmM ; sP1 = .93*mmM + .07*Mmm ; smgfx_DrawLine(&sP0, &sP1) ; // (&mmM,&Mmm, pOptGfxSet);
                                         sP0 = Mmm ; sP1 = .93*Mmm + .07*mmM ; smgfx_DrawLine(&sP0, &sP1) ;
                                         break ;
              case SM_BT_UNBOUNDED_MAX : sP0 = mMm ; sP1 = .93*mMm + .07*MMM ; smgfx_DrawLine(&sP0, &sP1) ; // (&mMm,&MMM, pOptGfxSet);
                                         sP0 = MMM ; sP1 = .93*MMM + .07*mMm ; smgfx_DrawLine(&sP0, &sP1) ;
                                         sP0 = mMM ; sP1 = .93*mMM + .07*MMm ; smgfx_DrawLine(&sP0, &sP1) ; // (&mMM,&MMm, pOptGfxSet);
                                         sP0 = MMm ; sP1 = .93*MMm + .07*mMM ; smgfx_DrawLine(&sP0, &sP1) ;
                                         break ;
              case SM_BT_UNBOUNDED     : sP0 = mmm ; sP1 = .93*mmm + .07*MmM ; smgfx_DrawLine(&sP0, &sP1) ; // (&mmm,&MmM, pOptGfxSet);
                                         sP0 = MmM ; sP1 = .93*MmM + .07*mmm ; smgfx_DrawLine(&sP0, &sP1) ;
                                         sP0 = mmM ; sP1 = .93*mmM + .07*Mmm ; smgfx_DrawLine(&sP0, &sP1) ; // (&mmM,&Mmm, pOptGfxSet);
                                         sP0 = Mmm ; sP1 = .93*Mmm + .07*mmM ; smgfx_DrawLine(&sP0, &sP1) ;
                                         sP0 = mMm ; sP1 = .93*mMm + .07*MMM ; smgfx_DrawLine(&sP0, &sP1) ; // (&mMm,&MMM, pOptGfxSet);
                                         sP0 = MMM ; sP1 = .93*MMM + .07*mMm ; smgfx_DrawLine(&sP0, &sP1) ;
                                         sP0 = mMM ; sP1 = .93*mMM + .07*MMm ; smgfx_DrawLine(&sP0, &sP1) ; // (&mMM,&MMm, pOptGfxSet);
                                         sP0 = MMm ; sP1 = .93*MMm + .07*mMM ; smgfx_DrawLine(&sP0, &sP1) ;
                                         break ;
              case SM_BT_UNKNOWN       :
                                         break;
            }                                                                     

          switch(eBdryType[2])
            {
              case SM_BT_BOUNDED       : break ;
              case SM_BT_UNBOUNDED_MIN : sP0 = mmm ; sP1 = .93*mmm + .07*MMm ; smgfx_DrawLine(&sP0, &sP1) ; // (&mmm,&MMm, pOptGfxSet);
                                         sP0 = MMm ; sP1 = .93*MMm + .07*mmm ; smgfx_DrawLine(&sP0, &sP1) ;
                                         sP0 = mMm ; sP1 = .93*mMm + .07*Mmm ; smgfx_DrawLine(&sP0, &sP1) ; // (&mMm,&Mmm, pOptGfxSet);
                                         sP0 = Mmm ; sP1 = .93*Mmm + .07*mMm ; smgfx_DrawLine(&sP0, &sP1) ;
                                         break ;
              case SM_BT_UNBOUNDED_MAX : sP0 = mmM ; sP1 = .93*mmM + .07*MMM ; smgfx_DrawLine(&sP0, &sP1) ; // (&mmM,&MMM, pOptGfxSet);
                                         sP0 = MMM ; sP1 = .93*MMM + .07*mmM ; smgfx_DrawLine(&sP0, &sP1) ;
                                         sP0 = mMM ; sP1 = .93*mMM + .07*MmM ; smgfx_DrawLine(&sP0, &sP1) ; // (&mMM,&MmM, pOptGfxSet);
                                         sP0 = MmM ; sP1 = .93*MmM + .07*mMM ; smgfx_DrawLine(&sP0, &sP1) ;
                                         break ;
              case SM_BT_UNBOUNDED     : sP0 = mmm ; sP1 = .93*mmm + .07*MMm ; smgfx_DrawLine(&sP0, &sP1) ; // (&mmm,&MMm, pOptGfxSet);
                                         sP0 = MMm ; sP1 = .93*MMm + .07*mmm ; smgfx_DrawLine(&sP0, &sP1) ;
                                         sP0 = mMm ; sP1 = .93*mMm + .07*Mmm ; smgfx_DrawLine(&sP0, &sP1) ; // (&mMm,&Mmm, pOptGfxSet);
                                         sP0 = Mmm ; sP1 = .93*Mmm + .07*mMm ; smgfx_DrawLine(&sP0, &sP1) ;
                                         sP0 = mmM ; sP1 = .93*mmM + .07*MMM ; smgfx_DrawLine(&sP0, &sP1) ; // (&mmM,&MMM, pOptGfxSet);
                                         sP0 = MMM ; sP1 = .93*MMM + .07*mmM ; smgfx_DrawLine(&sP0, &sP1) ;
                                         sP0 = mMM ; sP1 = .93*mMM + .07*MmM ; smgfx_DrawLine(&sP0, &sP1) ; // (&mMM,&MmM, pOptGfxSet);
                                         sP0 = MmM ; sP1 = .93*MmM + .07*mMM ; smgfx_DrawLine(&sP0, &sP1) ;
                                         break ;
              case SM_BT_UNKNOWN       :
                                         break;
            }                                                                     
        } // end unbounded check
    } // end iter ii, two times for bounded and unbounded graphics

  pRtn = smgfx_Close(pOptGfxSet) ;

#else
  SM_REF2(pUnboundedCenter, pOptGfxSet);
#endif // end SM_GFX_CODE
  return(pRtn) ;

} // end SmExtent3d::Draw

