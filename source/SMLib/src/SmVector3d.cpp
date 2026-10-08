// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************//**
* FILE NAME --- SmVector3d.cpp
* PURPOSE: Implementation of SmVector3D methods
**********************************************************************/

#include "StdAfx.h"

#include <SmVector3d.h>
#include <SmGraphicsExtern.h>
#include <SmGraphicsOutput.h>

double CoTangent(const SmVector3d &crVecA, const SmVector3d &crVecB, const SmVector3d &crVecC);

/*******************************************************************//**
PURPOSE: Determine if this vector is within or on a 2D sector defined by 
    the given tangents and binormal.  

NOTES: The two tangents define the boundaries of an in sector 
    and an out sector (just like cutting one piece of pie 
    creates both the piece and the rest of the pie).
      
    The 1st tangent's binormal vector points in the direction in which
    the first tangent is rotated to define the 'in' sector.
    
    This method assumes all vectors lie in the same plane.

    Returns:
   1: TRUE  when this vector lies between crBoundVector1 and crBoundVector2
            moving from crBoundVector1 in the direction of crBinormalVector1;
   2: TRUE  when this vector is equal to crBoundVector1 or crBoundVector2,
            to an angular tolerance of SM_EFF_ZERO;
   3: FALSE otherwise.

***********************************************************************/
SmBoolean SmVector3d::IsInsideSector
  (const SmVector3d & crBoundVector1,    // in : first vector defining sector
   const SmVector3d & crBinormalVector1, // in : vector pointing to inside of sector on first vector boundary
   const SmVector3d & crBoundVector2)    // in : 2nd vector defining sector
 const
{
  SM_ASSERT_BREAK(   crBoundVector1.x!=SM_UNDEF_DOUBLE  
                  && crBoundVector1.y!=SM_UNDEF_DOUBLE 
                  && crBoundVector1.z!=SM_UNDEF_DOUBLE
                  && crBoundVector1.z!=NL_NOZ
                  && crBinormalVector1.x!=SM_UNDEF_DOUBLE  
                  && crBinormalVector1.y!=SM_UNDEF_DOUBLE 
                  && crBinormalVector1.z!=SM_UNDEF_DOUBLE
                  && crBinormalVector1.z!=NL_NOZ
                  && crBoundVector2.x!=SM_UNDEF_DOUBLE  
                  && crBoundVector2.y!=SM_UNDEF_DOUBLE 
                  && crBoundVector2.z!=SM_UNDEF_DOUBLE
                  && crBoundVector2.z!=NL_NOZ
                  && x!=SM_UNDEF_DOUBLE 
                  && y!=SM_UNDEF_DOUBLE 
                  && z!=SM_UNDEF_DOUBLE
                  && z!=NL_NOZ) ;
                                                   
  SmVector3d sNorm = crBoundVector1 * crBinormalVector1;
  SM_ASSERT_BREAK(sNorm.LengthSquared() > SM_EFF_ZERO_SQ);
  double dSectorAngle, dThisAngle;
  SE( sNorm.Unitize() );
  SE( sNorm.CCWAngleBetween( crBoundVector1, crBoundVector2, dSectorAngle ));
  SE( sNorm.CCWAngleBetween( crBoundVector1, *this,          dThisAngle ));

  // CCWAngleBetween() returns [-PI,PI], we need [0,2PI]
  if ( dSectorAngle < 0.0 ) dSectorAngle += 2*SM_PI;
  if ( dThisAngle   < -SM_EFF_ZERO ) dThisAngle   += 2*SM_PI;

  // One preliminary check: due to numerical noise, it's possible
  // to get a mixup very close to 0 and 360 degrees, which could result in
  // one of the angles being very near 0 and the other near 360, which are
  // essentially identical.  Check that.
  if ( smos_Fabs( dThisAngle - dSectorAngle ) >= 2*SM_PI - SM_EFF_ZERO )
  {
      // All three vectors point in essentially the same direction.
      // We've decided to call that "Inside".
      return TRUE;
  }

  return (    dThisAngle > -SM_EFF_ZERO
           && dThisAngle < dSectorAngle + SM_EFF_ZERO );

} // end SmVector3d::IsInsideSector

/*******************************************************************//**
PURPOSE:  Classify a point with respect to a triangle defined by 3 points

    input: 'this'               = Point to be classified
           rP0, rP1, rP2        = 3 points defining a triangle
    output: 
           return value        = 0  point is outside the triangle
                               = 1 the point is inside the triangle
                               = 2 the point is within the edge of the triangle
                               = 3 the point is on a vertex of the triangle
                               = 4 the triangle is degenerate.
    
NOTES: The test is for 3d, but the orientation is wrt the plane of 
         the triangle.  The point is  'on' if the 'z' component is off or on.
         The tolerance is SM_EFF_ZERO for the dot of the vectors.

***********************************************************************/
int SmVector3d::IsInTriangle(const SmVector3d & rP0, const SmVector3d & rP1, const SmVector3d & rP2) const
{
    // crude distance (no square-root) test to disqualify triangles a long way away.
    SmPoint3d sMean((rP0 + rP1 + rP2)/3.0);

    double sum0 = (rP2 - rP1).SumFabs();
    double sum1 = (rP0 - rP2).SumFabs();
    double sum2 = (rP1 - rP0).SumFabs();
    double dMax = sum0;
    if (sum1 > dMax)dMax = sum1;
    if (sum2 > dMax)dMax = sum1;
    if (!(this->CloserThan(dMax*0.8, sMean))) return (0);

    SmPoint3d pNormal = (rP1 - rP0)*(rP2 - rP0);
    double myzero = SM_EFF_ZERO;
 
    double dot01 = ((*this - rP0)*(rP1 - rP0)).Dot(pNormal);
    double dot12 = ((*this - rP1)*(rP2 - rP1)).Dot(pNormal);
    double dot20 = ((*this - rP2)*(rP0 - rP2)).Dot(pNormal);
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
PURPOSE:  Build a set of orthogonal unit vectors 
    corresponding to the X, Y, and Z axis of a coordinate system. 
    
    input: 'this' vector        = X Axis direction
           optional pYReference = general direction of Y Axis 
    
    output: rXAxis = unitVector in 'this' vector direction
            rYAxis = unitVector perp to X in the 'this'/pYReference plane with
                     DotProduct(rYAxis, pYReference) > 0.0
            rZAxis = crossProduct(rXAxix, rYAxis)
    
NOTES: When pYReference == NULL, an arbitrary direction
    perpendicular to 'this' vector is picked for rYAxis.

***********************************************************************/
SmStatus SmVector3d::MakeUnitOrthoVectors // eff: make orthNormal vector set with XAxis = 'this' vector direction
  (const SmVector3d * pYReference,        // in : optional X/Y Plane specification, NULL to ignore 
   SmVector3d & rXAxis,                   // out: unitVector in 'this' vector direction
   SmVector3d & rYAxis,                   // out: unitVector perp to X in the 'this'/pYReference plane with
                                          //       DotProduct(rYAxis, pYReference) > 0.0
   SmVector3d & rZAxis)                   // out: crossProduct(rXAxix, rYAxis)
  const
{
 SM_ASSERT_BREAK(   x!=SM_UNDEF_DOUBLE 
                 && y!=SM_UNDEF_DOUBLE 
                 && z!=SM_UNDEF_DOUBLE
                 && z!=NL_NOZ) ;
 SM_ASSERT_BREAK(   pYReference == NULL
                 || (   pYReference->x!=SM_UNDEF_DOUBLE 
                     && pYReference->y!=SM_UNDEF_DOUBLE 
                     && pYReference->z!=SM_UNDEF_DOUBLE
                     && pYReference->z!=NL_NOZ)) ;
                                                   
  // given this as the X direction and pReference as the general
  // direction of Y then make the orthogonal unit vectors.
  // If pReference is not given then pick arbitrary Y axis
  rXAxis = *this;
  SmStatus sRet = rXAxis.Unitize();
  if ( sRet != SM_SUCCESS)
      return(sRet);

  if (pYReference) {
      rZAxis = rXAxis * (*pYReference);
      double dLengSq = rZAxis.LengthSquared();
      if (dLengSq > SM_EFF_ZERO_SQ) {
          rZAxis.Unitize();
          rYAxis = rZAxis * rXAxis;
          rYAxis.Unitize(); // probably not needed
          return SM_SUCCESS;
      }
  }
  // If made it to here choose arbitrary axis
  SmVector3d sYRef(0,1,0);  // try this one first
  rZAxis = rXAxis * sYRef;
  double dLengSq = rZAxis.LengthSquared();
  if (dLengSq < SM_EFF_ZERO_SQ) {
      sYRef.Set(0,0,1); // This must work
      rZAxis = rXAxis * sYRef;
      if (rZAxis.LengthSquared() < SM_EFF_ZERO_SQ) SER(SM_ERR);
  }

  rZAxis.Unitize();
  rYAxis = rZAxis * rXAxis;
  rYAxis.Unitize(); // probably not needed
  return SM_SUCCESS;

} // end SmVector3d::MakeUnitOrthoVectors    

/*******************************************************************//**
PURPOSE: return vector rotated about a given axis by a given amount

NOTES:
***********************************************************************/
SmVector3d SmVector3d::RotateVecAboutAxis
  (const SmVector3d &crAxis,     // in : Vector defining rotation center
   double            dAngRad)    // in : rotation angle in radians
 const
{
  // rotation parameters
  double dCos      = smos_CosRad(dAngRad) ;
  double dSin      = smos_SinRad(dAngRad) ;

  // avoid errors - unitize the input
  SmVector3d sZ    = crAxis ;
  sZ.Unitize() ;

  // Get the component perp to the rotation axis
  double     dDot  = this->Dot(sZ) ;
  SmVector3d sX    = *this - dDot*sZ ;
  double     dXLen = sX.Length() ;
  if (dXLen < SM_EFF_ZERO)
      { return(SmVector3d(this->x, this->y, this->z)); }

  sX /= dXLen ;

  // get component perp to rotate and x axes
  SmVector3d sY    = sZ * sX ;

  // return the rotated vector added back to the z component
  return(  dXLen * dCos * sX 
         + dXLen * dSin * sY
         + dDot  * sZ ) ;

} // end SmVector3d::RotateVecAboutAxis

/*******************************************************************//**
PURPOSE: return vector rotated about a given axis by a given amount

NOTES:
***********************************************************************/
SmPoint3d SmVector3d::RotatePtAboutLine
  (const SmPoint3d  &crLinePt,   // in : Point on rotation line
   const SmVector3d &crLineVec,  // in : Vector defining rotation line direction
   double            dAngRad)    // in : rotation angle in radians
 const
{
  // 1. let vec = this - crLinePt
  // 2. rotate vec about crLineVec
  // 3. return answer = crPointPt + rotatedVec
  return( crLinePt + (*this - crLinePt).RotateVecAboutAxis(crLineVec, dAngRad) ) ;

} // end SmVector3d::RotatePtAboutLine

/*******************************************************************//**
PURPOSE: Compute the derivative of (v/|v|) given v'.

NOTES: 
Example:
  The equation used as follows.
                                     v'.v 
   (v/|v|)' =   v'*Sqrt(v.v) - v * ---------
                                   Sqrt(v.v)
                -------------------------
                           v.v


However, to actually do this, it's always a good idea
to avoid the quotient rule.  A little algebra gives:

  say vec is *this, and vec' is given,
  then mag is the length of vec, and unit is vec/mag.
  Rewrite to avoid the quotient:

    vec = mag * unit
    vec' = mag' * unit + mag * unit'   <-- solve for unit'
    unit' = ( vec' - unit * mag' ) / mag  <-- we need mag'

    mag = ( vec . vec ) ^ 1/2
    mag' = [ (1/2) * ( vec . vec ) ^ (-1/2) ] * [ 2 * vec * vec' ]
         = [ 2 * vec * vec' ] / [ 2 * ( vec . vec ) ^ 1/2 ]
         = ( vec . vec' ) / mag
         = unit . vec'

Note, this approach becomes indispensable for higher derivatives.

***********************************************************************/
SmVector3d SmVector3d::UnitizedDerivative    // rtn : 1st derivative of unitized this vector, (v/|v|)
  (const SmVector3d & crVec_t)               // in  : v', 1st derivative of this vector (tangent)
 const                                       // note: v is the this vector              (position)
{ 
  SM_ASSERT_BREAK(   crVec_t.x!=SM_UNDEF_DOUBLE  
                  && crVec_t.y!=SM_UNDEF_DOUBLE 
                  && crVec_t.z!=SM_UNDEF_DOUBLE
                  && crVec_t.z!=NL_NOZ
                  && x!=SM_UNDEF_DOUBLE  
                  && y!=SM_UNDEF_DOUBLE 
                  && z!=SM_UNDEF_DOUBLE
                  && z!=NL_NOZ);

  // return value:
  SmVector3d sUnitVec_t( 0, 0, 0 );

  double dMag = this->Length();
  if ( dMag < SM_EFF_ZERO )
    {
      return crVec_t; // just return the derivative.
    }

  SmVector3d sUnitVec = *this / dMag;
  double     dMag_t   = sUnitVec.Dot( crVec_t );
  sUnitVec_t          = crVec_t - dMag_t * sUnitVec;
  sUnitVec_t         /= dMag;

  return sUnitVec_t;

} // end SmVector3d::UnitizedDerivative

/*******************************************************************//**
PURPOSE: Compute the first two derivatives of (v/|v|) given v' and v''.

NOTES: Just continue the differentiation in UnitizedDerivative.
***********************************************************************/
SmStatus SmVector3d::UnitizedDerivative2 // note: V   = this vector
  (const SmVector3d & crVec_t,           // in :  V'  = 1st deriv of this vector    
   const SmVector3d & crVec_tt,          // in :  V'' = 2nd deriv of this vector
         SmVector3d & rUnitVec_t,        // out:  (V/|V|)' = 1st deriv of the unit vector, (V/|V|)
         SmVector3d & rUnitVec_tt)       // out:  (V/|V|)''= 2nd deriv of the unit vector, (V/|V|)
     const
{ 
  SM_ASSERT_BREAK(   crVec_t.x!=SM_UNDEF_DOUBLE  
                  && crVec_t.y!=SM_UNDEF_DOUBLE 
                  && crVec_t.z!=SM_UNDEF_DOUBLE
                  && crVec_t.z!=NL_NOZ
                  && x!=SM_UNDEF_DOUBLE  
                  && y!=SM_UNDEF_DOUBLE 
                  && z!=SM_UNDEF_DOUBLE
                  && z!=NL_NOZ);

  double dMag = this->Length();
  if ( dMag < SM_EFF_ZERO )
    {
      rUnitVec_t  = crVec_t;  // Just return the derivatives.
      rUnitVec_tt = crVec_tt;
      return SM_ERR;
    }

  // 1st derivative
  SmVector3d sUnitVec = *this / dMag;
  double    dMag_t    = sUnitVec.Dot( crVec_t );
  rUnitVec_t          = crVec_t - dMag_t * sUnitVec;
  rUnitVec_t         /= dMag;

  // 2nd derivative
  double dMag_tt = rUnitVec_t.Dot( crVec_t ) + sUnitVec.Dot( crVec_tt );
  rUnitVec_tt    = crVec_tt - 2 * dMag_t * rUnitVec_t - dMag_tt * sUnitVec;
  rUnitVec_tt   /= dMag;

  // all done
  return SM_SUCCESS;

} // end SmVector3d::UnitizedDerivative2

/*******************************************************************//**
PURPOSE: Compute the first three derivatives of (v/|v|) given v', v'', and v'''.

NOTES: Just continue the differentiation in UnitizedDerivative2.
***********************************************************************/
SmStatus SmVector3d::UnitizedDerivative3  // note: V    = this vector                               
  (const SmVector3d & crVec_t,            // in :  V'   = 1st deriv of this vector                  
   const SmVector3d & crVec_tt,           // in :  V''  = 2nd deriv of this vector                  
   const SmVector3d & crVec_ttt,          // in :  V''' = 3rd deriv of this vector 
         SmVector3d & rUnitVec_t,         // out:  (V/|V|)'  = 1st deriv of the unit vector, (V/|V|)
         SmVector3d & rUnitVec_tt,        // out:  (V/|V|)'' = 2nd deriv of the unit vector, (V/|V|)
         SmVector3d & rUnitVec_ttt)       // out:  (V/|V|)'''= 2nd deriv of the unit vector, (V/|V|)                      
 const                                                          
{ 
  SM_ASSERT_BREAK(   crVec_t.x!=SM_UNDEF_DOUBLE  
                  && crVec_t.y!=SM_UNDEF_DOUBLE 
                  && crVec_t.z!=SM_UNDEF_DOUBLE
                  && crVec_t.z!=NL_NOZ
                  && x!=SM_UNDEF_DOUBLE  
                  && y!=SM_UNDEF_DOUBLE 
                  && z!=SM_UNDEF_DOUBLE
                  && z!=NL_NOZ);

  double dMag = this->Length();
  if ( dMag < SM_EFF_ZERO )  // Just return the derivatives.
  {
      rUnitVec_t   = crVec_t;
      rUnitVec_tt  = crVec_tt;
      rUnitVec_ttt = crVec_ttt;
      return SM_ERR;
  }

  SmVector3d sUnitVec = *this / dMag;
  double dMag_t    = sUnitVec.Dot( crVec_t );
  rUnitVec_t  = crVec_t - dMag_t * sUnitVec;
  rUnitVec_t /= dMag;

  // 2nd deriv:
  double dMag_tt = rUnitVec_t.Dot( crVec_t ) + sUnitVec.Dot( crVec_tt );
  rUnitVec_tt = crVec_tt - 2 * dMag_t * rUnitVec_t - dMag_tt * sUnitVec;
  rUnitVec_tt /= dMag;

  // 3rd deriv:
  double dMag_ttt =     rUnitVec_tt.Dot( crVec_t )
                  + 2 * rUnitVec_t.Dot( crVec_tt )
                  +     sUnitVec.Dot( crVec_ttt );
  rUnitVec_ttt = crVec_ttt
                  - 3 * dMag_t * rUnitVec_tt
                  - 3 * dMag_tt * rUnitVec_t
                  -     dMag_ttt * sUnitVec;
  rUnitVec_ttt /= dMag;

  return SM_SUCCESS;

} // end SmVector3d::UnitizedDerivative3

/*******************************************************************//**
PURPOSE: Calculate the angle between this and another vector.

NOTES: 3d angle; result runs from 0 to PI radians.
       when either input vector is ZeroLength
         returns SM_ERR_INVALID_INPUT and sets rdAngRad == 0.0
***********************************************************************/
SmStatus SmVector3d::AngleBetween
  (const SmVector3d & crOther,     // in : Other Vector
   double           & rdAngRad)    // out: angle (radians) from 0 to PI
  const
{
  SM_ASSERT_BREAK(   x!=SM_UNDEF_DOUBLE  
                  && y!=SM_UNDEF_DOUBLE 
                  && z!=SM_UNDEF_DOUBLE
                  && z!=NL_NOZ
                  && crOther.x!=SM_UNDEF_DOUBLE  
                  && crOther.y!=SM_UNDEF_DOUBLE 
                  && crOther.z!=SM_UNDEF_DOUBLE
                  && crOther.z!=NL_NOZ);

  // init return values
  SmStatus sRet = SM_SUCCESS;
  rdAngRad = 0;

  // vector lengths
  double dV1LenSquared = this->LengthSquared();
  double dV2LenSquared = crOther.LengthSquared();

  // Problem: dVxlen can be zero vector....if degenerate derivative 
  // probably 2 identical points
  if (SM_IS_ZERO_SQUARED(dV1LenSquared) || SM_IS_ZERO_SQUARED(dV2LenSquared)) 
    {
      SE(SM_ERR_INVALID_INPUT);
      rdAngRad = 0.0 ;
      sRet = SM_ERR_INVALID_INPUT;
    }
  else // angle = arcCos( Dot(this,other) / thisSize / otherSize )
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
          SmVector3d crs = (*this) * crOther;
          double crsNorm = crs.LengthSquared(); 
          double mag = smos_Sqrt( crsNorm / dV1LenSquared / dV2LenSquared);
          rdAngRad = dDot < 0 ? SM_PI - smos_ArcSine(mag) : smos_ArcSine(mag);
        }
    }

  return sRet;
}

/*******************************************************************//**
PURPOSE: Determine the counter clockwise angleRad between two vectors 
   that are projected into the plane perpendicular to the this vector.  

NOTES: The returned angle in radians range is [-PI, PI].
   
   Note that if either of the input vectors are parallel to the this 
   plane NormalVector of the projection plane (this) then the angle
    will be zero and no error will be returned.
***********************************************************************/
SmStatus SmVector3d::CCWAngleBetween
  (const SmVector3d & crStartVector,        // in : Start Vector
   const SmVector3d & crEndVector,          // in : End Vector
   double           & rdStartToEndAngRad)   // out: angle in radians from -PI to PI
   const
{
  SM_ASSERT_BREAK(   crStartVector.x!=SM_UNDEF_DOUBLE  
                  && crStartVector.y!=SM_UNDEF_DOUBLE 
                  && crStartVector.z!=SM_UNDEF_DOUBLE
                  && crStartVector.z!=NL_NOZ
                  && crEndVector.x!=SM_UNDEF_DOUBLE  
                  && crEndVector.y!=SM_UNDEF_DOUBLE 
                  && crEndVector.z!=SM_UNDEF_DOUBLE
                  && crEndVector.z!=NL_NOZ
                  && x!=SM_UNDEF_DOUBLE 
                  && y!=SM_UNDEF_DOUBLE 
                  && z!=SM_UNDEF_DOUBLE
                  && z!=NL_NOZ) ;
                                                   
  // project crStartVector onto plane perpendicular to this vector
  SmVector3d sThisVec   = *this;
  SmVector3d sStartProj = sThisVec * crStartVector * sThisVec;

  // for vectors parallel to this vector
  if (sStartProj.LengthSquared() < 100*SM_EFF_ZERO_SQ) 
    {
      // return Success and set output to 0.0
      rdStartToEndAngRad = 0.0;
      return SM_SUCCESS;
    }

  // project crEndVector onto plane perpendicular to this vector
  SmVector3d sEndProj = sThisVec * crEndVector * sThisVec;

  // for vectors parallel to this vector
  if (sEndProj.LengthSquared() < 100*SM_EFF_ZERO_SQ) 
    {
      // return Success and set output to 0.0
      rdStartToEndAngRad = 0.0;
      return SM_SUCCESS;
    }

  // compute angle (0 to PI) between projected start and end vectors
  double dProjAng;
  SER(sStartProj.AngleBetween(sEndProj,dProjAng));

  // negate angle's sign when Cross(start,end) opposes this vector angle
  SmVector3d sCross = sStartProj * sEndProj;
  if (sCross.Dot(sThisVec) < 0.0) 
    {
      dProjAng = - dProjAng;
    }

  // set output: an angle between -PI and PI
  rdStartToEndAngRad = dProjAng;
  return SM_SUCCESS;

} // end SmVector3d::CCWAngleBetween

/*******************************************************************//**
PURPOSE: Determine if two vectors are colinear along the line
         connecting two points.

NOTES: 
***********************************************************************/
SmBoolean SmVector3d::IsColinearWith
  (const SmVector3d & crEndVec,   // in : Direction to test against lineSeg
   const SmPoint3d  & crStartPnt, // in : StartPnt of LineSeg:[StartPnt EndPnt]
   const SmPoint3d  & crEndPnt)   // in : EndPnt   of LineSeg:[StartPnt EndPnt]
  const
{
  SM_ASSERT_BREAK(   crEndVec.x!=SM_UNDEF_DOUBLE  
                  && crEndVec.y!=SM_UNDEF_DOUBLE 
                  && crEndVec.z!=SM_UNDEF_DOUBLE
                  && crEndVec.z!=NL_NOZ
                  && crStartPnt.x!=SM_UNDEF_DOUBLE  
                  && crStartPnt.y!=SM_UNDEF_DOUBLE 
                  && crStartPnt.z!=SM_UNDEF_DOUBLE
                  && crStartPnt.z!=NL_NOZ
                  && crEndPnt.x!=SM_UNDEF_DOUBLE  
                  && crEndPnt.y!=SM_UNDEF_DOUBLE 
                  && crEndPnt.z!=SM_UNDEF_DOUBLE
                  && crEndPnt.z!=NL_NOZ
                  && x!=SM_UNDEF_DOUBLE  
                  && y!=SM_UNDEF_DOUBLE 
                  && z!=SM_UNDEF_DOUBLE
                  && z!=NL_NOZ);

  SmVector3d sVec = crEndPnt - crStartPnt;
  if (sVec.LengthSquared() < SM_EFF_ZERO_SQ) 
    { return TRUE; }

  sVec.Unitize();
  SmVector3d sDir1 = *this;
  sDir1.Unitize();
  SmVector3d sDir2 = crEndVec;
  sDir2.Unitize();

  if (1.0 - smos_Fabs(sVec.Dot(sDir1)) < SM_EFF_ZERO &&
      1.0 - smos_Fabs(sVec.Dot(sDir2)) < SM_EFF_ZERO)
    { return TRUE; }

  return FALSE;

} // end SmVector3d::IsColinearWith

/*******************************************************************//**
PURPOSE: 

NOTES: 
***********************************************************************/
double CoTangent
  (const SmVector3d &crVecA, 
   const SmVector3d &crVecB, 
   const SmVector3d &crVecC) 
{
  SM_ASSERT_BREAK(   crVecA.x!=SM_UNDEF_DOUBLE  
                  && crVecA.y!=SM_UNDEF_DOUBLE 
                  && crVecA.z!=SM_UNDEF_DOUBLE
                  && crVecA.z!=NL_NOZ
                  && crVecB.x!=SM_UNDEF_DOUBLE  
                  && crVecB.y!=SM_UNDEF_DOUBLE 
                  && crVecB.z!=SM_UNDEF_DOUBLE
                  && crVecB.z!=NL_NOZ
                  && crVecC.x!=SM_UNDEF_DOUBLE  
                  && crVecC.y!=SM_UNDEF_DOUBLE 
                  && crVecC.z!=SM_UNDEF_DOUBLE
                  && crVecC.z!=NL_NOZ);
  SmVector3d sBA = crVecA - crVecB;
  SmVector3d sBC = crVecC - crVecB;

  SmVector3d sTemp3 = sBC * sBA;
  double dLeng = sTemp3.Length();
  if (dLeng < SM_EFF_ZERO) return 1.0;

  return ((sBC.Dot(sBA)) / (dLeng));

} // end CoTangent

/*******************************************************************//**
PURPOSE: Compute BaryCentric coordinate of a point in a face.

NOTES: 
***********************************************************************/
SmStatus SmVector3d::BaryCentric
  (const SmTArray<SmVector3d> &crFaceVects,
   SmTArray<double> & rWeights) 
  const
{
 SM_ASSERT_BREAK(   x!=SM_UNDEF_DOUBLE 
                 && y!=SM_UNDEF_DOUBLE 
                 && z!=SM_UNDEF_DOUBLE
                 && z!=NL_NOZ) ;
                                                   
  ULONG iNum = crFaceVects.GetSize();

  ULONG j, iPrev, iNext;
  double dWeightSum = 0.0;
  rWeights.SetSize(iNum);

  if ( iNum < 1 ) { return SM_ERR; }

  for (ULONG ii=0; ii<iNum; ii++) {
      rWeights[ii] = 0.0;
  }    
  if ( iNum > 4 ) { return SM_ERR; }

  // Check to see if we need to do linear interpolation instead
  // of standard BaryCentric
  for (j = 0; j< iNum; j++) {
      iNext = (j + 1) % iNum;
      SmVector3d sDiff = crFaceVects[iNext] - crFaceVects[j];
      SmVector3d sToPnt = *this - crFaceVects[j];
      double dEPSILON = SM_EFF_ZERO * (1.0 + this->GetMaxDimension());
      SmVector3d sCross = sDiff * sToPnt;
      if (sCross.Length() < dEPSILON) {
          // Do linear interpolation
          double dDist1 = this->DistanceBetween(crFaceVects[j]);
          double dDist2 = this->DistanceBetween(crFaceVects[iNext]);
          double dTotal = dDist1+dDist2;
          rWeights[j] = dDist2 / dTotal; 
          rWeights[iNext] = dDist1 / dTotal;
          return SM_SUCCESS;
      }
  }

  // --- For each vertex compute weight
  for (j = 0; j < iNum; j++) {
      iPrev = ((j+iNum) - 1) % iNum;
      iNext = (j + 1) % iNum;
      SmVector3d sTempVec = *this - crFaceVects[j];
      double dLenSq = sTempVec.LengthSquared();
      const SmVector3d &crTemp1 = crFaceVects[j];
      const SmVector3d &crTemp2 = crFaceVects[iPrev];
      const SmVector3d &crTemp3 = crFaceVects[iNext];
      if (dLenSq < SM_EFF_ZERO_SQ) continue;
      rWeights[j] = (CoTangent(*this, crTemp1, crTemp2) + 
                     CoTangent(*this, crTemp1, crTemp3)) / dLenSq;
      dWeightSum += rWeights[j];
  }

  // --- Normalize the weights
  for (j = 0; j < iNum; j++) {
      rWeights[j] = rWeights[j] / dWeightSum;
  }

  return SM_SUCCESS;

} // end SmVector3d::BaryCentric

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
void SmVector3d::Dump
  (void) 
 const
{
   Dump(FALSE);

} // end SmVector3d::Dump

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
void SmVector3d::Dump
  (SmBoolean bAbbrev) 
 const
{
  TCHAR sBuff[SM_TBLOCK_SIZE];

  SM_ASSERT_BREAK(   x!=SM_UNDEF_DOUBLE  
                  && y!=SM_UNDEF_DOUBLE
                  && z!=SM_UNDEF_DOUBLE
                  && z!=NL_NOZ);

  if(   x==SM_UNDEF_DOUBLE 
     || y==SM_UNDEF_DOUBLE 
     || z==SM_UNDEF_DOUBLE
     || z==NL_NOZ)  
     smos_sprintf(sBuff,_T("%s"), _T(" [UNINITIALIZED] "));
  else if ( bAbbrev)
     smos_sprintf(sBuff,_T(" %6.6lf %6.6lf %6.6lf "),x,y,z);
  else 
     smos_sprintf(sBuff,_T(" [%16.16lf, %16.16lf, %16.16lf] "),x,y,z);

  smos_WriteBuffer(sBuff);

} // end SmVector3d::Dump

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
void SmVector3d::Dump
  (const TCHAR * message) 
 const
{
  SM_ASSERT_BREAK(   x!=SM_UNDEF_DOUBLE 
                  && y!=SM_UNDEF_DOUBLE 
                  && z!=SM_UNDEF_DOUBLE
                  && z!=NL_NOZ) ;
                                                     
  TCHAR sBuff[SM_TBLOCK_SIZE];
  smos_sprintf(sBuff,_T("\n%s "), message);
  smos_WriteBuffer(sBuff);
  this->Dump();

} // end SmVector3d::Dump


/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
void SmVector3d::Dump
  (ULONG i) 
 const
{
  SM_ASSERT_BREAK(   x!=SM_UNDEF_DOUBLE 
                  && y!=SM_UNDEF_DOUBLE 
                  && z!=SM_UNDEF_DOUBLE
                  && z!=NL_NOZ) ;
                                                     
  TCHAR sBuff[SM_TBLOCK_SIZE];
  smos_sprintf(sBuff,_T("\n%ld "), i);
  smos_WriteBuffer(sBuff);
  this->Dump();

} // end SmVector3d::Dump

/*******************************************************************//**
PURPOSE: Write SmVector3d to open output FILE

NOTES: 
***********************************************************************/

SmStatus SmVector3d::Write
  (FILE *pFile)
  const
{ SM_ASSERT_BREAK(   x!=SM_UNDEF_DOUBLE  
                  && y!=SM_UNDEF_DOUBLE
                  && z!=SM_UNDEF_DOUBLE
                  && z!=NL_NOZ);
  
  SM_FPRINTF(pFile,_T("SmVector3d: %18.16f %18.16f %18.16f\n"), x, y, z) ;
  return SM_SUCCESS;

} // end SmVector3d::WriteToFile

/*******************************************************************//**
PURPOSE: pVectorOrigin == NULL, draw point icon
            pVectorOrigin != NULL, draw vector starting at pVectorOrigin

NOTES: 
***********************************************************************/
SmDisplayList * SmVector3d::Draw
  (const SmVector3d * pVectorOrigin,        // in :
   const SmContext  * pContext,             // NotUsed: in :
   SmGfxArraySet    * pOptGfxSet)           // i/o: used for OpenGL ES 2.0. When given, accumulate GfxVertexArrays not GL calls.
                                            //      NULL to ignore. default:[NULL]
 
 const
{
  SM_REF1(pContext) ; 
  SmDisplayList *pRtn = NULL ;

  SM_ASSERT_BREAK(   x!=SM_UNDEF_DOUBLE 
                  && y!=SM_UNDEF_DOUBLE 
                  && z!=SM_UNDEF_DOUBLE
                  && z!=NL_NOZ) ;
     
#ifdef SM_GFX_CODE

  // can't currently add points to pick list because points are not derived from SmObject
  //      // when asked - add this Point to UI pick list
  //      if(bAddToUIPickList) 
  //        { sm_GraphicsAddToBrepList(this) ; }

  // start displayList (unless one is already open)
  smgfx_Open(smgfx_GetRuleColor(), NULL, NULL, FALSE, pOptGfxSet);
//  smgfx_OutputColor(smgfx_GetColor(pOptGfxSet)) ;

  // output optional line and point
  if (pVectorOrigin) 
    {
      // We are drawing a vector
      SmVector3d sVecEnd = *this + *pVectorOrigin;
      smgfx_DrawLine(pVectorOrigin->x,pVectorOrigin->y,pVectorOrigin->z,
                     sVecEnd.x,sVecEnd.y,sVecEnd.z, pOptGfxSet);
    }
  else 
    {
      // We are drawing a point
      smgfx_DrawPoint(x,y,z, pOptGfxSet);
    }

  // all done
  pRtn = smgfx_Close(pOptGfxSet) ;

#else
  SM_REF2(pVectorOrigin, pOptGfxSet);
#endif // end SM_GFX_CODE
  return(pRtn) ;

} // end SmVector3d::Draw(const SmVector3d *pVectorOrigin)

/*******************************************************************//**
PURPOSE: Draw pair of points and a line connecting them

NOTES: 
***********************************************************************/
SmDisplayList * SmVector3d::DrawPointToPoint
  (const SmVector3d & rOtherPoint,          // in :
   const SmContext  * pContext,             // in :
   SmGfxArraySet    * pOptGfxSet)           // i/o: used for OpenGL ES 2.0. When given, accumulate GfxVertexArrays not GL calls.
                                            //      NULL to ignore. default:[NULL]
 const
{
  SM_REF1(pContext) ; 
  SM_ASSERT_BREAK(   x!=SM_UNDEF_DOUBLE 
                  && y!=SM_UNDEF_DOUBLE 
                  && z!=SM_UNDEF_DOUBLE
                  && z!=NL_NOZ) ;
  SmDisplayList *pRtn = NULL ;

#ifdef SM_GFX_CODE
  // can't currently add points to pick list because points are not derived from SmObject
  //      // when asked - add this Point to UI pick list
  //      if(bAddToUIPickList) 
  //        { sm_GraphicsAddToBrepList(this) ; }

  smgfx_Open(smgfx_GetRuleColor(), NULL, NULL, FALSE, pOptGfxSet);

  // draw two point icons and a line
  smgfx_DrawPoint(x,y,z, pOptGfxSet);
  smgfx_DrawPoint(rOtherPoint.x,rOtherPoint.y,rOtherPoint.z, pOptGfxSet) ;
  smgfx_DrawLine(x,y,z,rOtherPoint.x,rOtherPoint.y,rOtherPoint.z, pOptGfxSet) ;

  pRtn = smgfx_Close(pOptGfxSet) ;

#else
  SM_REF2(rOtherPoint, pOptGfxSet);
#endif // end SM_GFX_CODE
  return(pRtn) ;

} // end SmVector3d::DrawPointToPoint

/*******************************************************************//**
PURPOSE: Draw plane icon centered on rPlanePoint and perp to this normal vector

NOTES: 
***********************************************************************/
SmDisplayList * SmVector3d::DrawPlane
  (const SmVector3d & rPlanePoint,          // in :
   const SmContext  * pContext,             // NotUsed: in :
   SmGfxArraySet    * pOptGfxSet)           // i/o: used for OpenGL ES 2.0. When given, accumulate GfxVertexArrays not GL calls.
                                            //      NULL to ignore. default:[NULL]
 
 const
{
  SM_REF1(pContext) ; 
  SM_ASSERT_BREAK(   x!=SM_UNDEF_DOUBLE 
                  && y!=SM_UNDEF_DOUBLE 
                  && z!=SM_UNDEF_DOUBLE
                  && z!=NL_NOZ) ;
  SM_ASSERT_BREAK(   rPlanePoint.x!=SM_UNDEF_DOUBLE 
                  && rPlanePoint.y!=SM_UNDEF_DOUBLE 
                  && rPlanePoint.z!=SM_UNDEF_DOUBLE
                  && rPlanePoint.z!=NL_NOZ) ;
  SmDisplayList *pRtn = NULL ;

#ifdef SM_GFX_CODE
  // can't currently add points to pick list because points are not derived from SmObject
  //      // when asked - add this Point to UI pick list
  //      if(bAddToUIPickList) 
  //        { sm_GraphicsAddToBrepList(this) ; }

  smgfx_Open(smgfx_GetRuleColor(), NULL, NULL, FALSE, pOptGfxSet);

  // draw properly oriented plane icon
  smgfx_DrawPlane(rPlanePoint, *this, pOptGfxSet);

  pRtn = smgfx_Close(pOptGfxSet) ;

#else
  SM_REF1(pOptGfxSet);
#endif // end SM_GFX_CODE
  return(pRtn) ;

} // end SmVector3d::DrawPlane

