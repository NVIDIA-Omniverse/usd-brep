// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************//**
* FILE NAME --- SmVector2d.cpp
* PURPOSE: Source file for SmVector2d methods
**********************************************************************/

#include "StdAfx.h"

#include <SmVector2d.h>
#include <SmVector3d.h>
#include <SmGraphicsExtern.h>
#include <SmGraphicsOutput.h>

// GWC: don't add a SmVector2d(SmVector3d const &obj) - 
//      it creates compiler automatic type conversion conflicts.
//      a line like  "SmVector3d = SmVector2d + SmVector3d" won't compile:
//  1> error C2666: 'operator +' : 2 overloads have similar conversions
//  1>   : could be 'SmVector3d operator +(const SmVector3d &,const SmVector3d &)'
//  1>   : or       'SmVector2d operator +(const SmVector2d &,const SmVector2d &)'
//  1> while trying to match the argument list '(SmVector3d, const SmVector2d)'
//
//  alternatively one could add all the mixed operator definitions - but how bothersome.  
//   
//   /*******************************************************************//**
//   PURPOSE: convenience Constructor for 
//              Point2d.x = Point3d.x, Point2d.y = Point3d.y
//   NOTES: 
//   ***********************************************************************/
//   SmVector2d::SmVector2d(SmVector3d const &obj) 
//   {
//     SM_ASSERT_BREAK(obj.x!=SM_UNDEF_DOUBLE && obj.y!=SM_UNDEF_DOUBLE) ;
//     x = obj.x ; 
//     y = obj.y ;
//   } // end Point3d to Point2d constructor operator

/*******************************************************************//**
PURPOSE: convenience assignment for 
           Point2d.x = Point3d.x, Point2d.y = Point3d.y
NOTES: 
***********************************************************************/
SmVector2d& SmVector2d::operator=(SmVector3d const &obj) 
{
  SM_ASSERT_BREAK(obj.x!=SM_UNDEF_DOUBLE && obj.y!=SM_UNDEF_DOUBLE) ;
   x = obj.x ; 
   y = obj.y ;
   return *this ;  
} // end Point3d to Point2d assignment operator

/*******************************************************************//**
PURPOSE: Compute the angle in radians between two vectors.
  
NOTES: The result is in radians and is between zero and pi.
***********************************************************************/
SmStatus SmVector2d::AngleBetween
  (const SmVector2d & crOther,  // in
   double & rdAngRad)           // out
  const
{
  SM_ASSERT_BREAK(   x!=SM_UNDEF_DOUBLE  
                  && y!=SM_UNDEF_DOUBLE 
                  && crOther.x!=SM_UNDEF_DOUBLE  
                  && crOther.y!=SM_UNDEF_DOUBLE);  
  
  // return value
  SmStatus sRet = SM_SUCCESS;

  // vector lengths
  double dV1LenSquared = this->LengthSquared();
  double dV2LenSquared = crOther.LengthSquared();

  // no work - zero length vectors
  if (SM_IS_ZERO_SQUARED(dV1LenSquared) || SM_IS_ZERO_SQUARED(dV2LenSquared)) 
    {
      SE(SM_ERR_INVALID_INPUT);
      sRet = SM_ERR_INVALID_INPUT;
    }
  else // angle = arcCos(Dot(this,other)/thisSize/otherSize)
    {
      double dDotPrep = this->Dot(crOther);
      double dDotSign = dDotPrep < 0 ? -1.0 : 1.0;
      double dDotSquared = (dDotPrep * dDotPrep) / dV1LenSquared / dV2LenSquared;
      double dDot = dDotSign * smos_Sqrt(dDotSquared);

      // Using acos of dot loses accuracy when dot is near +- 1.
      // Use asin of cross in that case.
      // The cutoff value is very arbitrary, which is ok because
      // the results are the same except when near the edges.
      if (dDotSquared < 0.81)
        {
          rdAngRad = smos_ArcCosine(dDot);
        }
      else
        {
          double dCrossPrep = x*crOther.y - y*crOther.x;
          double dCross =  smos_Sqrt((dCrossPrep * dCrossPrep) / dV1LenSquared / dV2LenSquared);
          rdAngRad = dDot < 0 ? SM_PI - smos_ArcSine(dCross) : smos_ArcSine(dCross);
        }
    }

  return sRet;

} // end SmVector2d::AngleBetween
    
/*******************************************************************//**
PURPOSE: Compute the angle in radians from this vector to the other
            in a CCW direction
  
NOTES: The result is in radians and is between zero and 2pi.
  Tangent vectors always return as 0.0 and never as 2*Pi

  The CCWAngle for vector pairs whose CrossProducts are zero to within
  2 * ScaledZero are snapped to 0.0 or SM_PI as appropriate.
***********************************************************************/
SmStatus SmVector2d::CCWAngleBetween
 (const SmVector2d & crOther,       // in : Other vector
  double           & rdAngRad)      // out: angle from ThisVector to OtherVector
                                    //      in CCW direction [0 to 2*Pi]
 const
{
  SM_ASSERT_BREAK(   x!=SM_UNDEF_DOUBLE  
                  && y!=SM_UNDEF_DOUBLE 
                  && crOther.x!=SM_UNDEF_DOUBLE  
                  && crOther.y!=SM_UNDEF_DOUBLE);  

  // get 0 to Pi angle - does not distinguish between + and - angles
  SmStatus sRet = AngleBetween(crOther, rdAngRad) ;

  // when angle was computed
  if(sRet == SM_SUCCESS)
    {
      // check to see if the vector is from Pi to 2*pi
      // when cross(this,other) < 0.0)
      double dScaledZero = SM_EFF_ZERO * (1.0 + smos_Max(crOther.GetMaxDimension(), GetMaxDimension())) ;
      double dCross = (x * crOther.y - y * crOther.x) ;
      if(dCross < -2.0 * dScaledZero)
        {
          // mirror dAngle about xAxis
          rdAngRad = 2.0*SM_PI - rdAngRad ;
        }
      else if(dCross < 0.0)  // gwc: why isn't this (dCross < 2.0 * dScaledZero) for symmetry?
        {
          // angle is either 0 or Pi
          rdAngRad = (smos_Fabs(rdAngRad) < 0.1)
                    ? 0.0 
                    : SM_PI ;
        }
    }

  // all done
  return(sRet) ;

} // end SmVector2d::CCWAngleBetween

/*******************************************************************//**
PURPOSE:  Classify a point with respect to a triangle defined by
             3 2D points

    input: 'this'               = Point to be classified
           rP0, rP1, rP2        = 3 points defining a triangle
    output: 
           return value        = 0  point is outside the triangle
                               = 1 the point is inside the triangle
                               = 2 the point is within the edge of the triangle
                               = 3 the point is on a vertex of the triangle
                               = 4 the triangle is degenerate.
    
NOTES: The test is for 2d
         The tolerance is SM_EFF_ZERO for the dot of the vectors.

***********************************************************************/
int SmVector2d::IsInTriangle(const SmPoint2d & rP0,const SmPoint2d & rP1,const SmPoint2d & rP2) const

{
    double myzero = SM_EFF_ZERO;
 
    double dot01 = ((*this - rP0).Cross(rP1 - rP0));
    double dot12 = ((*this - rP1).Cross(rP2 - rP1));
    double dot20 = ((*this - rP2).Cross(rP0 - rP2));
    if (fabs(dot01) < myzero) dot01 = 0.0;
    if (fabs(dot12) < myzero) dot12 = 0.0;
    if (fabs(dot20) < myzero) dot20 = 0.0;

    int nZeroDots = 0;
    if (dot01 == 0.0) nZeroDots++;
    if (dot12 == 0.0) nZeroDots++;
    if (dot20 == 0.0) nZeroDots++;
    if ( nZeroDots == 3) return (4); // degenerate
    if ( nZeroDots == 2) return (3); // at vertex
    int nNotPositive = 0;
    if (dot01 <= 0.0) nNotPositive++;
    if (dot12 <= 0.0) nNotPositive++;
    if (dot20 <= 0.0) nNotPositive++;
    
    if (nNotPositive == 0 || nNotPositive == 3) 
        return ((nZeroDots == 0) ? 1 : 2) ;  // (inside : on edge)

    return ((nZeroDots == 0) ? 0 : 2) ;  // ( outside : on edge)

}  // end IsInTriangle

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
void SmVector2d::Dump
  (void) 
 const
{ 
  TCHAR sBuff[SM_TBLOCK_SIZE];
  SM_ASSERT_BREAK(   x!=SM_UNDEF_DOUBLE  
                  && y!=SM_UNDEF_DOUBLE);
  
  if(   x==SM_UNDEF_DOUBLE 
     || y==SM_UNDEF_DOUBLE)  
    smos_sprintf(sBuff,_T("%s"),_T(" [UNINITIALIZED] "));                                                
  else
    smos_sprintf(sBuff,_T(" [%lf,%lf] "),x,y);
  smos_WriteBuffer(sBuff);

} // end SmVector2d::Dump

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
void SmVector2d::Dump
  (const TCHAR * message) 
 const
{ SM_ASSERT_BREAK(   x!=SM_UNDEF_DOUBLE  
                  && y!=SM_UNDEF_DOUBLE);
  
  TCHAR sBuff[SM_TBLOCK_SIZE];
  smos_sprintf(sBuff,_T("\n%s "), message);
  smos_WriteBuffer(sBuff);
  this->Dump();

} // end SmVector2d::Dump

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
void SmVector2d::Dump
  (ULONG i) 
 const
{ SM_ASSERT_BREAK(   x!=SM_UNDEF_DOUBLE  
                  && y!=SM_UNDEF_DOUBLE);
  
  TCHAR sBuff[SM_TBLOCK_SIZE];
  smos_sprintf(sBuff,_T("\n%ld "), i);
  smos_WriteBuffer(sBuff);
  this->Dump();

} // end SmVector2d::Dump

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
SmDisplayList * SmVector2d::Draw
  (const SmVector2d * pVectorOrigin,   // in :
   const SmContext  * pContext,        // NotUsed: in :
   SmGfxArraySet    * pOptGfxSet)      // i/o: used for OpenGL ES 2.0. When given, accumulate GfxVertexArrays not GL calls.
 const                                 //      NULL to ignore. default:[NULL]
{
  SM_REF1(pContext) ; 
  SmDisplayList *pRtn = NULL ;
  SM_ASSERT_BREAK(   x!=SM_UNDEF_DOUBLE  
                  && y!=SM_UNDEF_DOUBLE
                  && (   pVectorOrigin == NULL
                      || (   pVectorOrigin->x!=SM_UNDEF_DOUBLE  
                          && pVectorOrigin->y!=SM_UNDEF_DOUBLE)));
  
#ifdef SM_GFX_CODE
    smgfx_Open(smgfx_GetRuleColor(), NULL, NULL, FALSE, pOptGfxSet);
    smgfx_OutputColor(smgfx_GetColor()) ;
    if (pVectorOrigin) 
      {
        // We are drawing a vector
        SmVector2d sVecEnd = *pVectorOrigin + *this;
        smgfx_DrawLine(pVectorOrigin->x,pVectorOrigin->y,0,
                       sVecEnd.x,sVecEnd.y,0, pOptGfxSet);
      }
    else 
      {
        // We are drawing a point
        smgfx_DrawPoint(x,y,0, pOptGfxSet);
      }
  pRtn = smgfx_Close(pOptGfxSet) ;

#else
  SM_REF1(pOptGfxSet);
#endif // end SM_GFX_CODE
  return(pRtn) ;

} // end SmVector2d::Draw

/*******************************************************************//**
PURPOSE: Write SmVector2d to open output FILE

NOTES: 
***********************************************************************/
SmStatus SmVector2d::Write
  (FILE *pFile)
  const
{ SM_ASSERT_BREAK(   x!=SM_UNDEF_DOUBLE  
                  && y!=SM_UNDEF_DOUBLE);
  
  SM_FPRINTF(pFile,_T("SmVector2d: %18.16f %18.16f\n"), x, y) ;
  return SM_SUCCESS;

} // end SmVector2d::WriteToFile

/*******************************************************************//**
PURPOSE: Read SmVector2d from open input FILE

NOTES: 
***********************************************************************/
SmStatus SmVector2d::Read
  (FILE *pFile)
{
  TCHAR sBuff[SM_TBLOCK_SIZE] ;
  if (!pFile) {
      smos_sprintf(sBuff,_T("%s"),_T("  *** ERROR Bad File Pointer - \n"));
      smos_WriteBuffer(sBuff);
      return SM_ERR;
  }
  int nRet = 0;
  // read into member memory
  nRet = SM_FSCANF(pFile,_T("%s"),sBuff) ;
  nRet = SM_FSCANF(pFile,_T("%lf"), &x) ;
  nRet = SM_FSCANF(pFile,_T("%lf"), &y) ;
  nRet = SM_FSCANF(pFile,_T("\n")) ;
  
  SM_REF1(nRet);

  SM_ASSERT_BREAK(   x!=SM_UNDEF_DOUBLE  
                  && y!=SM_UNDEF_DOUBLE);
  
  return SM_SUCCESS;

} // end SmVector2d::Read

