// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************//**
* FILE NAME --- SmPolarBox.cpp
* PURPOSE: Implementation of polar box methods.
**********************************************************************/

#include "StdAfx.h"

#include <SmPolarBox.h>
#include <SmGraphicsExtern.h>
#include <SmGraphicsOutput.h>
#include <SmAssertArray.h>

#ifdef SM_DEBUG_CODE
static constexpr SmBoolean bCheckMe = TRUE;
#endif

/*******************************************************************//**
PURPOSE: Constructor for the polar box which initializes the basis vector.

NOTES:
***********************************************************************/
SmPolarBox::SmPolarBox
  (const SmVector3d & crVector)
{
  SM_ASSERT(crVector.LengthSquared() > SM_EFF_ZERO_SQ);

  if(crVector.LengthSquared() < SM_EFF_ZERO_SQ)
    {m_ePolarBoxState = SM_PS_UNINITIALIZED;
    }
  else
    {
      m_ePolarBoxState = SM_PS_SINGLE_POINT;
      m_sPolarDomain.SetMinMax(0.0,0.0,0.0,0.0) ;
      m_aBasis[0]      = crVector;
      m_aBasis[0].MakeUnitOrthoVectors(NULL, m_aBasis[0], m_aBasis[1], m_aBasis[2]) ;
      //      m_aBasis[0].Unitize();
    }
} // end SmPolarBox::SmPolarBox (single vector)

/*******************************************************************//**
PURPOSE: Assignment operator for the polar box

NOTES:
***********************************************************************/

SmPolarBox& SmPolarBox::operator=(SmPolarBox const &obj)
{
  // no work condition
  if(&obj == this) return *this ;

  // copy values
  m_ePolarBoxState = obj.m_ePolarBoxState ;
  m_sPolarDomain   = obj.m_sPolarDomain ;
  switch(m_ePolarBoxState)
    { case SM_PS_POINT_WITH_BASIS :
      case SM_PS_REGION       :
      case SM_PS_ARC          :
      case SM_PS_SINGLE_POINT : m_aBasis[0] = obj.m_aBasis[0] ;
                                m_aBasis[1] = obj.m_aBasis[1] ;
                                m_aBasis[2] = obj.m_aBasis[2] ;
      default                 : break ;
    }
  return *this ;

} // end SmPolarBox::operator=

/*******************************************************************//**
PURPOSE: Given a 3D vector compute the corresponding polar coordinate
   in terms of angle relative to the coordinate system defined by the
   polar box.

NOTES: The X value of the 2D point returned represents the
   counter clockwise angle beteen m_aBasis[0] and the input vector
   when looking from m_aBasis[1].  The Y value of the 2D point returned
   represents the counter clockwise angle between m_aBasis[0] as seen
   from m_aBasis[2].  Note that the angles are in radians and can be
   between -PI and +PI.

  The returned vector does not have to be contained within m_sPolarDomain

  An uninitialized vector is returned for illegal queries.
  1. m_ePolarBoxState == SM_PS_UNINITIALIZED or SM_PS_UNBOUNDED
***********************************************************************/
SmPoint2d SmPolarBox::ComputePolarCoord
  (const SmVector3d & crVector)
  const
{
  // check state
#ifdef SM_DEBUG_CODE
if ( bCheckMe ) {
  if(crVector.LengthSquared() <= SM_EFF_ZERO_SQ)
    { SM_ASSERT(crVector.LengthSquared() > SM_EFF_ZERO_SQ); }
} // end if bCheckMe
#endif

  // locals
  double dAngle1 = 0.0;
  double dAngle2 = 0.0;

  // should never compute polar coordinates for some cases
  if (   m_ePolarBoxState == SM_PS_UNINITIALIZED
      || m_ePolarBoxState == SM_PS_UNBOUNDED)
    {
      // Should never make it to here
      SE(SM_ERR);
      return SmPoint2d() ;
    }

// gwc: gave case SM_PS_SINGLE_POINT a coordinate system so this function can be called for all cases
//        // for first axis
//        if (   m_ePolarBoxState == SM_PS_ARC
//            || m_ePolarBoxState == SM_PS_REGION)
//          {
//      //        SE(m_aBasis[1].CCWAngleBetween(m_aBasis[0],crVector,dAngle1));
//      //        SE(m_aBasis[2].CCWAngleBetween(m_aBasis[0],crVector,dAngle2));

      // Inline CCWAngleBetween for faster performance

      // project crVector to v0/v2 plane
      SmVector3d sProj1 = m_aBasis[1] *  crVector * m_aBasis[1];

      // dAngle1 is angle of vector projected onto v0/v2 plane from v0
      // when crVector is parallel to m_aBasis[1]
      if (sProj1.LengthSquared() < SM_EFF_ZERO_SQ)
        { dAngle1 = 0.0; }
      else
        {
          SE(sProj1.Unitize());
          double dDot = sProj1.Dot(m_aBasis[0]);
          // Prevent tolerance errors: dDot must be between -1 and +1,
          // but even two unit vectors can have roundoff enough to crash ArcCosine.
          // Also optimize a bit.
          if ( dDot >  1.0 - SM_EFF_ZERO )
            { dAngle1 = 0.0; }
          else if ( dDot < -1.0 + SM_EFF_ZERO )
            { dAngle1 = SM_PI; }
          else
            { dAngle1 = smos_ArcCosine(dDot); }  // gets angle from 0 to Pi

          // Set dAngle1 sign so it's the CCW angle when viewed from V1 vector end.
          SmVector3d sCross = m_aBasis[0] * sProj1;
          if (sCross.Dot(m_aBasis[1]) < 0.0)
            {
              // note: positive a2 angles have positive v1 components
              //       positive a1 angles have negative v2 components
              dAngle1 = - dAngle1;
            }
        }
      // end first axis

      // Do it again for other basis

      // project crVector to v0/v1 plane
      SmVector3d sProj2 = m_aBasis[2] *  crVector * m_aBasis[2];

      // dAngle2 is angle of vector projected onto v0/v1 plane from v0
      // when crVector is parallel to m_aBasis[2]
      if (sProj2.LengthSquared() < SM_EFF_ZERO_SQ)
        { dAngle2 = 0.0; }
      else
        {
          SE(sProj2.Unitize());
          double dDot = sProj2.Dot(m_aBasis[0]);
          // Prevent tolerance errors, and optimize a bit.
          if ( dDot >  1.0 - SM_EFF_ZERO )
            { dAngle2 = 0.0; }
          else if ( dDot < -1.0 + SM_EFF_ZERO )
            { dAngle2 = SM_PI; }
          else
            { dAngle2 = smos_ArcCosine(dDot); }  // gets angle from 0 to Pi

          // Set dAngle2 sign so it's the CCW angle when viewed from v2 vector end.
          SmVector3d sCross = m_aBasis[0] * sProj2;
          if (sCross.Dot(m_aBasis[2]) < 0.0)
            {
              // note: positive a2 angles have positive v1 components
              //       positive a1 angles have negative v2 components
              dAngle2 = - dAngle2;
            }
        }
  //      }

  return SmPoint2d(dAngle1,dAngle2);

} // end SmPolarBox::ComputePolarCoord

/*******************************************************************//**
PURPOSE: get Unit Sphere Surface Area covered by polar box

NOTES:
  Lune = shape defined between two great circles on a sphere (a lemon slice)
  Area of Lune = 2 * R**2 * theta, where theta = angle between planes of the great circles
  SmPolarBox Area = intersection of two lunes
                  = Lune Area - upper Spherical Triangle Area
                              - lower Spherical Triangle Area
  Spherical Triangle Area = R**2*(r+g+b - Pi)
     where r, g, b are the spherical triangle corner angles in radians.
           triangle corner angles = the angle between the great planes

    Let the SmPolarBox great circles and the planes they lie upon
        be identified by their defining angles,
      a0Min (m_sPolarDomain.Min.x)
      a0Max (m_sPolarDomain.Max.x)
      a1Min (m_sPolarDomain.Min.y)
      a1Max (m_sPolarDomain.Max.y)

    The 4 great circles of the SmPolarBox intersect at 4 corners
    around the SmPolar Box boundary.

      b00 = PlanePlaneAngle(a0Min,a1Min)
      b01 = PlanePlaneAngle(a0Min,a1Max)
      b11 = PlanePlaneAngle(a0Max,a1Max)
      b10 = PlanePlaneAngle(a0Max,a1Min)

    Area SmPolarBox = 2*(1*1)*(a0Max-a0Min)
                      - (1*1)*(a0Max-a0Min + b00 + b01 - Pi)
                      - (1*1)*(a0Max-a0Min + b10 + b11 - PI)
                    = (1*1) * (2*Pi - b00 - b01 - b11 - b10) ;
***********************************************************************/
double SmPolarBox::GetPolarArea() const
{
  // switch on the m_ePolarBoxState
  switch(m_ePolarBoxState)
    {
      case SM_PS_POINT_WITH_BASIS:
      case SM_PS_SINGLE_POINT:
        { // the v0 vector is the only vector in the box - return 0.0 for area
          return 0.0 ;
        }
      case SM_PS_ARC :
        { // all vectors lie in the v0/v2 plane -
          // return( m_sPolarDomain.XLength()) ;
          return( 0.0 ) ;
        }
      case SM_PS_REGION :
        { // Vectors are reconstructed from angles of
          //   the projections onto the v0/v2 and v0/v1 planes.
          // Reconstructing the vectors from projection angles
          //   has to remember that the coordinate system is left handed

          SmPoint2d sAMin = m_sPolarDomain.GetMin() ;
          SmPoint2d sAMax = m_sPolarDomain.GetMax() ;

          double dCA1Min = smos_Cosine(sAMin.x) ;
          double dSA1Min = smos_Sine  (sAMin.x) ;
          double dCA2Min = smos_Cosine(sAMin.y) ;
          double dSA2Min = smos_Sine  (sAMin.y) ;

          double dCA1Max = smos_Cosine(sAMax.x) ;
          double dSA1Max = smos_Sine  (sAMax.x) ;
          double dCA2Max = smos_Cosine(sAMax.y) ;
          double dSA2Max = smos_Sine  (sAMax.y) ;

          // normal for each great circle plane
          SmVector3d sA1MinNorm =  -dSA1Min * m_aBasis[0] - dCA1Min * m_aBasis[2] ;
          SmVector3d sA1MaxNorm =  -dSA1Max * m_aBasis[0] - dCA1Max * m_aBasis[2] ;
          SmVector3d sA2MinNorm =  -dSA2Min * m_aBasis[0] + dCA2Min * m_aBasis[1] ;
          SmVector3d sA2MaxNorm =  -dSA2Max * m_aBasis[0] + dCA2Max * m_aBasis[1] ;

          double dB00, dB01, dB11, dB10 ;
          sA1MinNorm.AngleBetween( sA2MinNorm, dB00) ;
          sA1MinNorm.AngleBetween(-sA2MaxNorm, dB01) ;
          sA1MaxNorm.AngleBetween( sA2MaxNorm, dB11) ;
          sA1MaxNorm.AngleBetween(-sA2MinNorm, dB10) ;

          double dPolarArea = 2 * SM_PI - dB00 - dB01 - dB11 - dB10 ;
          SM_ASSERT(dPolarArea >= - SM_EFF_ZERO) ;
          return(dPolarArea) ;
        }
      case SM_PS_UNBOUNDED:
        {
          return (4 * SM_PI) ;
        }
      default: return(0.0) ;
    } // end switch on m_ePolarBoxState

} // end SmPolarBox::GetPolarArea


/*******************************************************************//**
PURPOSE: Expand the polar box by a given angle.

NOTES:
***********************************************************************/
SmBoolean SmPolarBox::IsPerpendicularDisjointWithCone
  (const SmVector3d & crConeVec,
   double dConeAngleInRadians)
  const
{

  // low work
  if (m_ePolarBoxState == SM_PS_UNBOUNDED)
    {
      return FALSE;
    }

  SmVector3d sPerpVector = crConeVec * m_aBasis[0] * crConeVec;

  if (sPerpVector.LengthSquared() < SM_EFF_ZERO_SQ) return FALSE;

  return IsDisjointWithCone(sPerpVector,dConeAngleInRadians);

} // end SmPolarBox::IsPerpendicularDisjointWithCone

/*******************************************************************//**
PURPOSE: Determine if a polar box and a cone intersect in their
    vector mappings.

NOTES:
***********************************************************************/
SmBoolean SmPolarBox::IsDisjointWithCone
  (const SmVector3d & crConeVec,
   double dConeAngleInRadians)
  const
{
  // low work
  if (m_ePolarBoxState == SM_PS_UNBOUNDED)
    { return FALSE; }

  // ensure ConeVec points in same direction as PolarBox
  double dDot = m_aBasis[0].Dot(crConeVec);
  SmVector3d sConeVec = crConeVec;
  if (dDot < 0.0) sConeVec = - sConeVec;

  // Handle special case where only single vector is known for this
  if (   m_ePolarBoxState == SM_PS_SINGLE_POINT
      || m_ePolarBoxState == SM_PS_POINT_WITH_BASIS)
    {
      double dAngFound;
      if (sConeVec.AngleBetween(m_aBasis[0],dAngFound) != SM_SUCCESS)
        {
          return FALSE;
        }
      if (dAngFound > SM_PI/2.0)
        { dAngFound = SM_PI - dAngFound;   }


      if (dAngFound > dConeAngleInRadians)
        { return TRUE; }
      else
        { return FALSE; }
    }

  SmPoint2d  sPolarCoord = ComputePolarCoord(sConeVec);
  SmExtent2d sDomain     = m_sPolarDomain;
  // Expand the angles of the domain by the cone angle
  // and test to see if the cone vector is inside of it.
  sDomain.ExpandAbsolute(dConeAngleInRadians);
  if (!sDomain.ContainsPoint2d(sPolarCoord))
    {
      return TRUE; // Disjoint situation.
    }
  return FALSE;

} // end SmPolarBox::IsDisjointWithCone

/*******************************************************************//**
PURPOSE: Determine if two polar boxes are disjoint in their vector
    mappings.  Note that vector mappings are like infinite cones and
    apply to both sides of the origin of the defining sphere.

NOTES:
***********************************************************************/
SmBoolean SmPolarBox::AreDisjoint
  (const SmPolarBox & crOther)
 const
{
    // Non initialized boxes should never make it to this method
    if(   m_ePolarBoxState == SM_PS_UNINITIALIZED
       || crOther.m_ePolarBoxState == SM_PS_UNINITIALIZED)
      {
        SE(SM_ERR);
        return FALSE;
      }

    // If either of boxes is unbounded then they are not disjoint
    if(   m_ePolarBoxState         == SM_PS_UNBOUNDED
       || crOther.m_ePolarBoxState == SM_PS_UNBOUNDED)
      {
        return FALSE;
      }

    // If both are single vector
    if(   (   m_ePolarBoxState         == SM_PS_SINGLE_POINT
           || m_ePolarBoxState         == SM_PS_POINT_WITH_BASIS)
       && (   crOther.m_ePolarBoxState == SM_PS_SINGLE_POINT
           || crOther.m_ePolarBoxState == SM_PS_POINT_WITH_BASIS))
     {
        double dDot = m_aBasis[0].Dot(crOther.m_aBasis[0]);
        return( (smos_Fabs(dDot) > 1.0 - SM_EFF_ZERO_SQRT) ? FALSE : TRUE ) ;
    }

    // Get angle between two polarBox basis[0] vectors
    double dDot = m_aBasis[0].Dot(crOther.m_aBasis[0]);
    if (dDot >  1.0) dDot =  1.0;
    if (dDot < -1.0) dDot = -1.0;
    double dAngleBetweenBasis = smos_ArcCosine(dDot);
    if (dDot < 0.0)
      {
        dAngleBetweenBasis = SM_PI - dAngleBetweenBasis;
      }

    //
    double dPolarBox1MaxAngle = 0.0;
    double dPolarBox2MaxAngle = 0.0;
    if (   m_ePolarBoxState != SM_PS_SINGLE_POINT
        && m_ePolarBoxState != SM_PS_POINT_WITH_BASIS)
      {
        SmPoint2d sSize = m_sPolarDomain.GetSize();
        dPolarBox1MaxAngle      = sSize.Length();
      }

    if (   crOther.m_ePolarBoxState != SM_PS_SINGLE_POINT
        && crOther.m_ePolarBoxState != SM_PS_POINT_WITH_BASIS)
      {
        SmPoint2d sSizeOther = crOther.m_sPolarDomain.GetSize();
        dPolarBox2MaxAngle = sSizeOther.Length();
      }

    // low work angle between basis bigger than Sum(MaxAngles)
    if (dAngleBetweenBasis > dPolarBox1MaxAngle + dPolarBox2MaxAngle + SM_EFF_ZERO_SQRT)
      {
        return TRUE;
      }

    // If the angle between the basis is small (11 degrees) quit here
    if (dAngleBetweenBasis < SM_PI/16.0)
      {
        return FALSE;
      }

    // Determine sign to use - this should tell us which
    // side of the cone to look at when doing comparisons
    double dSign = 1.0;
    SmVector3d sCentThis  = EvaluateNormalized(0.5,0.5);
    SmVector3d sCentOther = crOther.EvaluateNormalized(0.5,0.5);
    if (sCentThis.Dot(sCentOther) < 0.0)
      {
        dSign = - 1.0;
      }


    // Project crOther vectors onto this and test domains
    if (   m_ePolarBoxState != SM_PS_SINGLE_POINT
        && m_ePolarBoxState != SM_PS_POINT_WITH_BASIS)
      {
        SmVector3d aVecs[4];
        ULONG lNumVecs;
        crOther.GetBoundaryVectors(lNumVecs,aVecs);
        SM_ASSERT(lNumVecs > 0);
        SmPoint2d sPolarCoord = ComputePolarCoord(aVecs[0]*dSign);
        SmExtent2d sPolarDomain(sPolarCoord);
        for (ULONG j=1; j<lNumVecs; j++)
          {
            sPolarCoord = ComputePolarCoord(aVecs[j]*dSign);
            if (m_sPolarDomain.ContainsPoint2d(sPolarCoord))
              {
                return FALSE;
              }
            sPolarDomain.AddPoint2d(sPolarCoord);
          }
        // Now compare domain of this with computed polar domain.
        sPolarDomain.ExpandAbsolute(SM_EFF_ZERO_SQRT);
        if (sPolarDomain.AreDisjoint(m_sPolarDomain))
          {
            return TRUE;
          }
      }

    // Project this vectors onto crOther and test domains
    if (   crOther.m_ePolarBoxState != SM_PS_SINGLE_POINT
        && crOther.m_ePolarBoxState != SM_PS_POINT_WITH_BASIS)
    {
        SmVector3d aVecs[4];
        ULONG lNumVecs;
        GetBoundaryVectors(lNumVecs,aVecs);
        SM_ASSERT(lNumVecs > 0);
        SmPoint2d sPolarCoord = crOther.ComputePolarCoord(aVecs[0]*dSign);
        SmExtent2d sPolarDomain(sPolarCoord);
        for (ULONG j=1; j<lNumVecs; j++) {
            sPolarCoord = crOther.ComputePolarCoord(aVecs[j]*dSign);
            if (crOther.m_sPolarDomain.ContainsPoint2d(sPolarCoord)) {
                return FALSE;
            }
            sPolarDomain.AddPoint2d(sPolarCoord);
        }
        // Now compare domain of this with computed polar domain.
        sPolarDomain.ExpandAbsolute(SM_EFF_ZERO_SQRT);
        if (sPolarDomain.AreDisjoint(crOther.m_sPolarDomain)) {
            return TRUE;
        }
    }

    // If made it to here then we can't determine if they are disjoint
    // so just return FALSE
    return FALSE;

} // end SmPolarBox::AreDisjoint

/*******************************************************************//**
PURPOSE: Find the cross product between two polar boxes.

NOTES:
    This is used to find the normal box given the partial derivative boxes
    of a surface.

    The polar cross product is a polar box which bounds
    the direction of all the vectors created by taking the cross-product
    of any vector in this polarBox with any vector from the crOther polar Box.
***********************************************************************/
SmStatus SmPolarBox::Cross
  (const SmPolarBox & crOther,
   SmPolarBox & rCrossBox)
const
{
  if(   m_ePolarBoxState         == SM_PS_UNINITIALIZED
     || crOther.m_ePolarBoxState == SM_PS_UNINITIALIZED)
    {
      rCrossBox.m_ePolarBoxState = SM_PS_UNBOUNDED;
      return SM_SUCCESS;
    }

  // If either of boxes is unbounded then result is unbounded
  if(   m_ePolarBoxState         == SM_PS_UNBOUNDED
     || crOther.m_ePolarBoxState == SM_PS_UNBOUNDED)
    {
      rCrossBox.m_ePolarBoxState = SM_PS_UNBOUNDED;
      return SM_SUCCESS;
    }

  // Cross polar boxes by crossing each of the vectors and
  // adding the crosses to the cross box
  rCrossBox.m_ePolarBoxState = SM_PS_UNINITIALIZED;

  // Cross the center vectors to make sure that we keep
  // reasonably centered.
  SmVector3d sCrossVec = m_aBasis[0] * crOther.m_aBasis[0];
  if (sCrossVec.LengthSquared() > SM_EFF_ZERO_SQ)
    {
      rCrossBox.AddVector3d(sCrossVec);
      // If we have two points then we can quit right here
      if (   (   m_ePolarBoxState == SM_PS_SINGLE_POINT
              || m_ePolarBoxState == SM_PS_POINT_WITH_BASIS)
          && (   crOther.m_ePolarBoxState == SM_PS_SINGLE_POINT
              || crOther.m_ePolarBoxState == SM_PS_POINT_WITH_BASIS))
      {
          return SM_SUCCESS;
      }
      // Take second vector for basis which is derived from second
      rCrossBox.m_aBasis[1] = m_aBasis[0];
      rCrossBox.m_aBasis[2] = rCrossBox.m_aBasis[0] * rCrossBox.m_aBasis[1];
      rCrossBox.m_ePolarBoxState = SM_PS_ARC;
      rCrossBox.m_sPolarDomain   = SmExtent2d(SmPoint2d(0,0));
    }

  // get boundary Vectors for both polar boxes
  SmVector3d aThisVecs[4], aOtherVecs[4];
  ULONG lNumThisVecs, lNumOtherVecs;
  GetBoundaryVectors(lNumThisVecs,aThisVecs);
  crOther.GetBoundaryVectors(lNumOtherVecs,aOtherVecs);

  // for every binary pair of boundary vectors
  for (ULONG i=0; i<lNumThisVecs; i++)
    {
      for (ULONG j=0; j<lNumOtherVecs; j++)
        {
          // get the boundary vector cross products
          SmVector3d sCross = aThisVecs[i] * aOtherVecs[j];
          if (sCross.LengthSquared() > SM_EFF_ZERO_SQ)
            {
              // and add it to the rCrossBox output
              rCrossBox.AddVector3d(sCross);
            }
        }
    } // end iter all boundary vectors

  return SM_SUCCESS;

} // end SmPolarBox::Cross

/*******************************************************************//**
PURPOSE: Given a polar box evaluate it using normalized
           (between 0 and 1) parameters.

NOTES: Return vector is set to (0,0,0) when requested
                dAlpha, dBeta values can not be turned into a unique
                vector.  This happens whenever
                   m_ePolarBoxState == SM_PS_UNITIALIZED
                or m_ePolarBoxState == SM_PS_UNBOUNDED
                or dAlpha or dBeta map to an angle greater than or equal to 90 degrees.
***********************************************************************/
SmVector3d SmPolarBox::EvaluateNormalized
  (double dAlpha,        // in : only used for m_ePolarBoxState == SM_PS_ARC && SM_PS_REGION
   double dBeta)         // in : only used for m_ePolarBoxState == SM_PS_REGION
  const
{
  // switch on the m_ePolarBoxState
  switch(m_ePolarBoxState)
    {
      case SM_PS_POINT_WITH_BASIS:
      case SM_PS_SINGLE_POINT:
        { // the v0 vector is the only vector in the box - return it
          return m_aBasis[0] ;
        }
      case SM_PS_ARC :
        { // all vectors lie in the v0/v2 plane - compute and return the vector
          SmPoint2d sDomainPoint = m_sPolarDomain.Evaluate(dAlpha,dBeta);
          return (  smos_Cosine(sDomainPoint.x) * m_aBasis[0]
                  - smos_Sine  (sDomainPoint.x) * m_aBasis[2]) ;
        }
      case SM_PS_REGION :
        { // Vectors are reconstructed from angles of
          //   the projections onto the v0/v2 (angle1) and v0/v1 (angle2) planes.
          //   Angles are measured from the Basis[0] vector
          //   in a CCW direction when viewed from the tip of the
          //   basis vector perpendicular to the plane of projection.

          SmPoint2d sDomainPoint = m_sPolarDomain.Evaluate(dAlpha,dBeta);

          double dCA1 = smos_Cosine(sDomainPoint.x) ;
          double dSA1 = smos_Sine  (sDomainPoint.x) ;
          double dCA2 = smos_Cosine(sDomainPoint.y) ;
          double dSA2 = smos_Sine  (sDomainPoint.y) ;

          // for angles == 90 degrees we can't reconstruct the vector uniquely
          if(   dCA1 <= SM_EFF_ZERO
             || dCA2 <= SM_EFF_ZERO)
            { // return the 0 vector
              return SmVector3d(0.0, 0.0, 0.0) ;
            }

          // The direction of the reconstructed vector is found
          // as the sum of the projection vector on the v0/v2 plane
          // plus a component in the v1 direction.  That component's
          // magnitude is found by scaling the v0/v1 projection vector
          // so that its v0 component equals the v0 component of the v0/v2 plane projection.
          // The v1 component magnitude of the reconstructed vector
          // is just the v1 component of that scaled projection vector.

          // construct the vector
          //   - remember the left hand coordinate system
          //   - remember to scale the v0/v1 projection vector before using it to get
          //              the v1 component.

          // build the output vector (nonNormalized)
          SmVector3d sVec =   dCA1 * m_aBasis[0]
                            - dSA1 * m_aBasis[2]
                            + dSA2 * dCA1/dCA2 * m_aBasis[1] ;

          // normalize the output
          sVec.Unitize() ;

          // gwc: build vector directly - if this works with better tolerances perhaps we can replace previous code
#ifdef SM_DEBUG_CODE
if ( bCheckMe ) {
          double dDen   = (1 - dSA1*dSA2) * (1 + dSA1*dSA2) ;

          double dSSize = smos_Sqrt( (1-dSA1*dSA1) / dDen) ;
          double dQSize = smos_Sqrt( (1-dSA2*dSA2) / dDen) ;
          SmVector3d sTrialVec =   dSSize * dSA2 * m_aBasis[1]
                                 - dQSize * dSA1 * m_aBasis[2]
                                 + dQSize * dCA1 * m_aBasis[0] ;
          SmBoolean bZSame   = SM_ARE_SAME_TO_TOL(dSSize * dCA2, dQSize * dCA1, SM_EFF_ZERO_RAD) ;
          SmBoolean bVecSame = sVec.CloserThan(100.0 * SM_EFF_ZERO_RAD, sTrialVec) ;
          if(!bZSame || !bVecSame)
            {
              SM_ASSERT_MSG(bZSame,   _T("Broken PolarAngle Conversion - not the same ZComponent") ) ;
              SM_ASSERT_MSG(bVecSame, _T("Broken PolarAngle Conversion - not the same Vector") ) ;
            }
} // end if bCheckMe
#endif // SM_DEBUG_CODE

          // all done
          return(sVec) ;
        }
      default:
        { // inverse is not defined for cases SM_PS_UNINITIALIZED and SM_PS_UNBOUNDED
          return(SmVector3d(0.0, 0.0, 0.0)) ;
        }
    } // end switch on m_ePolarBoxState

} // end SmPolarBox::EvaluateNormalized

/*******************************************************************//**
PURPOSE: Add a vector to the polar box and increase the domain
   of the box if necessary.

NOTES: If input vector is zero length
                or if input vector is parallel to an existing vector
                or
                - do nothing
***********************************************************************/
SmStatus SmPolarBox::AddVector3d
  (const SmVector3d & crVectorToAdd,   // in : vector to add, increases polar box as needed,
                                       //      1st two independent vectors define basis vectors.
   const SmVector3d *pOptBasis2)       // in : only used when adding the 1st and 2nd point
                                       //      to help set the Basis Vectors. When given
                                       //      the polar box m_aBasis[2] vector will equal the
                                       //      pOptBasis2 vector.  Assumes pOptBasis2 is unitized on input.
{
  // no work - zero vector
  if (crVectorToAdd.LengthSquared() < SM_EFF_ZERO_SQ)
    { return(SM_SUCCESS) ;
    }

  switch(m_ePolarBoxState)
    {
      case SM_PS_UNINITIALIZED:
        {
          m_ePolarBoxState = SM_PS_SINGLE_POINT;
          m_sPolarDomain.SetMinMax(0.0,0.0,0.0,0.0) ;

          if(pOptBasis2 == NULL)
            {
              m_aBasis[0] = crVectorToAdd;
              m_aBasis[0].MakeUnitOrthoVectors(NULL, m_aBasis[0], m_aBasis[1], m_aBasis[2]) ;
            }
          else
            {
              SM_ASSERT(SM_ARE_SAME_TO_TOL(pOptBasis2->LengthSquared(), 1.0, SM_EFF_ZERO)) ;

              // let Basis[0] = projection of vector to orthogonal plane
              //     Basis[2] = pOptBasis2
              //     Basis[1] = cross(Basis[2], Basis[0]) ;

              m_aBasis[0] = *pOptBasis2 * crVectorToAdd * *pOptBasis2 ;
              m_aBasis[2] = *pOptBasis2 ;
              double dProjLength2 = m_aBasis[0].LengthSquared() ;
              if(dProjLength2 > SM_EFF_ZERO_SQ)
                {
                   m_aBasis[0].Unitize() ;
                   m_aBasis[2] = *pOptBasis2 ;
                   m_aBasis[1] = m_aBasis[2] * m_aBasis[0] ;
                   m_ePolarBoxState = SM_PS_POINT_WITH_BASIS ;
                   AddVector3d(crVectorToAdd) ;
                }
              else
                {
                  m_aBasis[0] = crVectorToAdd;
                  m_aBasis[0].MakeUnitOrthoVectors(NULL, m_aBasis[0], m_aBasis[1], m_aBasis[2]) ;
                }
            }
        }
        break ;

      case SM_PS_POINT_WITH_BASIS:
      case SM_PS_SINGLE_POINT:
        { SmVector3d sCross  = crVectorToAdd * m_aBasis[0];
          double     dLength = sCross.Length();
          double     dDot    = m_aBasis[0].Dot(crVectorToAdd) ;

          // Vector is parallel to current basis
          if (dLength < SM_EFF_ZERO)
            {
              // and in opposite direction
              if (dDot < SM_EFF_ZERO)
                {
                  // Here vectors point in opposite direction - only thing we can
                  // do is to set it to be unbounded
                  m_ePolarBoxState = SM_PS_UNBOUNDED;
                }
              else
                {
                  // m_ePolarBoxState = SM_PS_SINGLE_POINT;
                }
            }
          else // vector is not parallel with current basis
            {
              m_sPolarDomain.SetMinMax(0,0,0,0) ;

              if(m_ePolarBoxState == SM_PS_SINGLE_POINT)
                {
                  // let v1 = unitized(cross(vec,v0))
                  //     v2 = unitized(cross(v1,v0))
                  // When m_ePolarBoxState == SM_PS_POINT_WITH_BASIS,
                  //     v1 and v2 are already set - don't set them again.
                  sCross.Unitize();
                  m_aBasis[1] = sCross;  // place vector in the v0/v2 plane
                  m_aBasis[2] = m_aBasis[0] * m_aBasis[1];
                  m_aBasis[2].Unitize();
                }

              if(dDot < SM_EFF_ZERO)
                {
                  m_ePolarBoxState = SM_PS_UNBOUNDED ;
                }
              else
                {
                  m_ePolarBoxState = SM_PS_ARC;
                  SmPoint2d sNewAngles = ComputePolarCoord(crVectorToAdd);
                  m_sPolarDomain.AddPoint2d(sNewAngles);
                  if(smos_Fabs(sNewAngles.y) > SM_EFF_ZERO_SQRT)
                    {
                      m_ePolarBoxState = SM_PS_REGION ;
                    }
                }
            }
         break ;
       }

     case SM_PS_ARC :
       { double     dDot      = m_aBasis[0].Dot(crVectorToAdd) ;

         // and in opposite direction
         if (dDot < SM_EFF_ZERO)
           {
             // Here vectors point in opposite direction - only thing we can
             // do is to set it to be unbounded
             m_ePolarBoxState = SM_PS_UNBOUNDED;
           }
         else
           {
             SmPoint2d sNewAngles = ComputePolarCoord(crVectorToAdd);

             // when vectors fall out of the v0/v2 plane
             if (smos_Fabs(sNewAngles.y) > SM_EFF_ZERO_SQRT)
               {
                 // change classification from SM_PS_ARC to IS_PS_REGION
                 m_ePolarBoxState = SM_PS_REGION;
               }

             // If we go beyond 90 degrees from reference vector then
             // we will consider the polar box to be unbounded.
             SM_ASSERT(   smos_Fabs(sNewAngles.x)         < SM_PI/2.0
                       && smos_Fabs(sNewAngles.y)         < SM_PI/2.0
                       && smos_Fabs(m_sPolarDomain.GetMin().x) < SM_PI/2.0
                       && smos_Fabs(m_sPolarDomain.GetMin().y) < SM_PI/2.0
                       && smos_Fabs(m_sPolarDomain.GetMax().x) < SM_PI/2.0
                       && smos_Fabs(m_sPolarDomain.GetMax().y) < SM_PI/2.0) ;

             // increase domain to include new vector as needed
             m_sPolarDomain.AddPoint2d(sNewAngles);
           }
         break ;
       }

     case SM_PS_REGION :
       { double dDot        = m_aBasis[0].Dot(crVectorToAdd) ;
         double dScaledZero = SM_EFF_ZERO * (1.0 + crVectorToAdd.GetMaxDimension()) ;

         // and in opposite direction
         if (dDot < dScaledZero)
           {
             // Here vectors point in opposite direction - only thing we can
             // do is to set it to be unbounded
             m_ePolarBoxState = SM_PS_UNBOUNDED;
           }
         else
           {
             SmPoint2d sNewAngles = ComputePolarCoord(crVectorToAdd);

             // If we go beyond 90 degrees from reference vector then
             // we will consider the polar box to be unbounded.
             SM_ASSERT(   smos_Fabs(sNewAngles.x)         < SM_PI/2.0
                       && smos_Fabs(sNewAngles.y)         < SM_PI/2.0
                       && smos_Fabs(m_sPolarDomain.GetMin().x) < SM_PI/2.0
                       && smos_Fabs(m_sPolarDomain.GetMin().y) < SM_PI/2.0
                       && smos_Fabs(m_sPolarDomain.GetMax().x) < SM_PI/2.0
                       && smos_Fabs(m_sPolarDomain.GetMax().y) < SM_PI/2.0) ;

             // increase domain to include new vector as needed
             m_sPolarDomain.AddPoint2d(sNewAngles);

             // make sure arcs are labeled as arcs.
             if (   smos_Fabs(m_sPolarDomain.XLength()) < SM_EFF_ZERO_SQRT
                 && m_ePolarBoxState != SM_PS_UNBOUNDED)
               {
                 SmVector3d sTmp  = -m_aBasis[1] ;
                 m_aBasis[1]      = m_aBasis[2] ;
                 m_aBasis[2]      = sTmp ;
                 m_sPolarDomain.SetMinMax( m_sPolarDomain.GetUMin(), m_sPolarDomain.GetVMin(),
                                           m_sPolarDomain.GetUMax(), m_sPolarDomain.GetVMax()) ;
                 m_ePolarBoxState = SM_PS_ARC ;
               }
           }
         break ;
       }
     case SM_PS_UNBOUNDED :
       // Don't need to do anything here
       break ;

     default:
       // Should never make it here
       SER(SM_ERR);
       break ;

    } // end switch on m_ePolarBoxState

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
         if(bDebugMe)
           {
             SmPoint2d sAngles, sNormalizedPt ;
             SmPoint3d sCenterPoint(0,0,0) ;

             sAngles = ComputePolarCoord(crVectorToAdd) ;
             SM_ASSERT(m_sPolarDomain.ContainsPoint2d(sAngles)) ;
             m_sPolarDomain.Inversion(sAngles, sNormalizedPt) ;
             SmVector3d sTestVec = EvaluateNormalized(sNormalizedPt.x, sNormalizedPt.y) ;
             SmVector3d sUnitVec = crVectorToAdd ;
             sUnitVec.Unitize() ;
             SM_ASSERT(sTestVec.CloserThan(SM_EFF_ZERO_RAD, sUnitVec)) ;

             Dump() ;

             smgfx_SetRotationCenter(sCenterPoint) ;

             smgfx_Erase() ;
             smgfx_SetLook(2,3, 1,0,0) ;  crVectorToAdd.Draw(&sCenterPoint) ; sm_GraphicsLoop() ;
             smgfx_SetLook(1,2, 0,1,1) ;  this->Draw(sCenterPoint) ; sm_GraphicsLoop() ;
             sm_GraphicsLoop() ;

           }
#endif // SM_DEBUG_CODE


  return SM_SUCCESS;

} // end SmPolarBox::AddVector3d

/*******************************************************************//**
PURPOSE: Get the one, two, or four vectors that define the boundary
   corners of the polar box.

NOTES:
***********************************************************************/
SmStatus SmPolarBox::GetBoundaryVectors
  (ULONG & rlNumVectors,
   SmVector3d sVectors[4]) const
{
  if(   m_ePolarBoxState == SM_PS_UNINITIALIZED
     || m_ePolarBoxState == SM_PS_UNBOUNDED)
    {
      SER(SM_ERR_INVALID_INPUT);
    }
  if (   m_ePolarBoxState == SM_PS_SINGLE_POINT
      || m_ePolarBoxState == SM_PS_POINT_WITH_BASIS)
    {
      rlNumVectors = 1;
      sVectors[0]  = m_aBasis[0];
    }
  else if (m_ePolarBoxState == SM_PS_ARC)
    {
      rlNumVectors = 2;
      sVectors[0] = EvaluateNormalized(0,0);
      sVectors[1] = EvaluateNormalized(1,1);
    }
  else
    {
      rlNumVectors = 4;
      sVectors[0] = EvaluateNormalized(0,0);
      sVectors[1] = EvaluateNormalized(1,0);
      sVectors[2] = EvaluateNormalized(1,1);
      sVectors[3] = EvaluateNormalized(0,1);
    }
  return SM_SUCCESS;

} // end SmPolarBox::GetBoundaryVectors



/*******************************************************************//**
PURPOSE: Determine if there is a vector in the polar box which is
    perpendicular to the given vector.

NOTES:
***********************************************************************/
SmBoolean SmPolarBox::HasPerpendicularToVector
  (const SmVector3d & crTestVector)   // in : assumed to be a
   const
{
  ULONG lNumVec;
  SmVector3d sVectors[4] ;

  // low work - all vectors within the PolarBox - return TRUE
  if(   m_ePolarBoxState == SM_PS_UNINITIALIZED
     || m_ePolarBoxState == SM_PS_UNBOUNDED)
    {
      return TRUE;
    }

  SmVector3d sUnitTestVector = crTestVector ;
  sUnitTestVector.Unitize() ;

  // low work - single vector bounding box is perp to
  GetBoundaryVectors(lNumVec,sVectors);
  long lSign = 0;
  if (lNumVec == 1)
    {
      SmBoolean bRtn = smos_Fabs(sVectors[0].Dot(sUnitTestVector)) < SM_EFF_ZERO_SQRT*100.0 ;
      return (bRtn) ;
    }

  // This test sees if the vectors defining the pseudo box has a perpendicular
  // vector by testing to see all of the dot products have the same sign.
  for (ULONG i=0; i<lNumVec; i++)
    {
      if (smos_Fabs(sVectors[i].Dot(sUnitTestVector)) < SM_EFF_ZERO_SQRT*100.0)
        {
          return TRUE;
        }

      if (sVectors[i].Dot(crTestVector) > SM_EFF_ZERO_SQRT*100.0)
        {
          if      (lSign  < 0) return TRUE;
          else if (lSign == 0) lSign = 1;
        }
      if (sVectors[i].Dot(crTestVector) <  -SM_EFF_ZERO_SQRT*100.0)
        {
          if      (lSign  > 0) return TRUE;
          else if (lSign == 0) lSign = -1;
        }
    }

  // all done
  return FALSE;

} // end SmPolarBox::HasPerpendicularToVector


/*******************************************************************//**
PURPOSE: Union the extents of two polar boxes.  It tries to be smart
    by averaging the extents of the two original boxes.  This way things
    should be more or less centered.

NOTES:
***********************************************************************/
SmStatus SmPolarBox::Union
  (const SmPolarBox & crOther,
   SmPolarBox & crResult)
 const
{
  // low work - union of an Unbounded box is another unbounded box
  if(   m_ePolarBoxState         == SM_PS_UNBOUNDED
     || crOther.m_ePolarBoxState == SM_PS_UNBOUNDED)
    {
      crResult.m_ePolarBoxState = SM_PS_UNBOUNDED;
      return SM_SUCCESS;
    }

  // guess some kind of mid vector
  SmVector3d sMid = m_aBasis[0] + crOther.m_aBasis[0];
  double     dDot = m_aBasis[0].Dot(crOther.m_aBasis[0]) ;

  // low work - mid vector disappears or oppose one another
  if(   sMid.LengthSquared() < SM_EFF_ZERO_SQ
     || dDot < SM_EFF_ZERO)
    {
      crResult.m_ePolarBoxState = SM_PS_UNBOUNDED;
      return SM_SUCCESS;
    }

  // init output
  crResult.ReSet() ;
  crResult.AddVector3d(sMid);

  // get boundary vectors for this and other
  ULONG i, lThisNumVec, lOtherNumVec;
  SmVector3d sThisVectors[4], sOtherVectors[4] ;
  GetBoundaryVectors(lThisNumVec,sThisVectors);
  crOther.GetBoundaryVectors(lOtherNumVec,sOtherVectors);

  // Add this boundary vectors
  for (i=0; i<lThisNumVec; i++)
    {
      crResult.AddVector3d(sThisVectors[i]);
    }

  // add other boundary vectors
  for (i=0; i<lOtherNumVec; i++)
    {
      crResult.AddVector3d(sOtherVectors[i]);
    }

  // make sure the parent classifies
  if(crResult.m_ePolarBoxState < this->m_ePolarBoxState)   { crResult.m_ePolarBoxState = this->m_ePolarBoxState ; }
  if(crResult.m_ePolarBoxState < crOther.m_ePolarBoxState) { crResult.m_ePolarBoxState = crOther.m_ePolarBoxState ; }

#ifdef SM_DEBUG_CODE
if ( bCheckMe ) {
  if(   !this->IsContainedBy(crResult)
     || !crOther.IsContainedBy(crResult))
    {
      SM_ASSERT(   this->IsContainedBy(crResult)
                && crOther.IsContainedBy(crResult)) ;
    }
} // end if bCheckMe

SmBoolean bDebugMe = FALSE ;
  if(bDebugMe)
    {
      SmPoint3d sOrigin(0,0,0) ;
      //SmBoolean bContained1 = this->IsContainedBy(crResult) ;
      //SmBoolean bContained2 = crOther.IsContainedBy(crResult) ;

      this->Dump() ;
      crOther.Dump() ;
      crResult.Dump() ;

      smgfx_Erase() ;
      smgfx_SetLook(2,3, 0,1,1) ; for(i=0;i<lThisNumVec;i++)  { sThisVectors[i].Draw(&sOrigin) ; } sm_GraphicsLoop() ;
      smgfx_SetLook(2,3, 1,0,1) ; for(i=0;i<lOtherNumVec;i++) { sOtherVectors[i].Draw(&sOrigin) ; } sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,0,1) ; this->Draw(SmPoint3d(0,0,0)) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,0) ; crOther.Draw(SmPoint3d(0,0,0)) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 1,0,0) ; crResult.Draw(SmPoint3d(0,0,0)) ; sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

  return SM_SUCCESS;

} // end SmPolarBox::Union

/*******************************************************************//**
PURPOSE: Returns True when the This sphere domain is wholly
            contained by the crOther boundaries.

NOTES: Equal domains are considered contained
***********************************************************************/
SmBoolean SmPolarBox::IsContainedBy
  (const SmPolarBox & crOther,   // in : PolarBox to Test
   double dTol)                  // in : Max Allowed deviation still counted the same, default:[SM_EFF_ZERO_RAD]
 const
{
  // check state - uninitialized boxes should not make it here
  if(   crOther.m_ePolarBoxState == SM_PS_UNINITIALIZED
     || this->m_ePolarBoxState   == SM_PS_UNINITIALIZED)
    {
      SE(SM_ERR) ;
      return(FALSE) ;
    }

  // low work - other is unbounded
  if(crOther.m_ePolarBoxState == SM_PS_UNBOUNDED)
    { return TRUE ; }

  // low work - incompatible domain shapes
  if(   crOther.m_ePolarBoxState < m_ePolarBoxState
     && !(   crOther.m_ePolarBoxState == SM_PS_POINT_WITH_BASIS
          && m_ePolarBoxState         == SM_PS_SINGLE_POINT))
    { return FALSE ; }



  switch(m_ePolarBoxState)
    {
      case SM_PS_POINT_WITH_BASIS :
      case SM_PS_SINGLE_POINT : { SmVector3d sV00 = EvaluateNormalized(0.0, 0.0) ;
                                  SmVector2d sA00 = crOther.ComputePolarCoord(sV00) ;
                                  SmBoolean  bA00 = crOther.m_sPolarDomain.ContainsPoint2d(sA00, dTol) ;
                                  return(bA00) ;
                                }

      case SM_PS_ARC :          { SmVector3d sV00 = EvaluateNormalized(0.0, 0.0) ;
                                  SmVector3d sV10 = EvaluateNormalized(1.0, 0.0) ;
                                  SmVector2d sA00 = crOther.ComputePolarCoord(sV00) ;
                                  SmVector2d sA10 = crOther.ComputePolarCoord(sV10) ;
                                  SmBoolean  bA00 = crOther.m_sPolarDomain.ContainsPoint2d(sA00, dTol) ;
                                  SmBoolean  bA10 = crOther.m_sPolarDomain.ContainsPoint2d(sA10, dTol) ;
                                  return(   bA00
                                         && bA10) ;
                                }
      case SM_PS_REGION :       { SmVector3d sV00 = EvaluateNormalized(0.0, 0.0) ;
                                  SmVector3d sV10 = EvaluateNormalized(1.0, 0.0) ;
                                  SmVector3d sV11 = EvaluateNormalized(1.0, 1.0) ;
                                  SmVector3d sV01 = EvaluateNormalized(0.0, 1.0) ;
                                  SmVector2d sA00 = crOther.ComputePolarCoord(sV00) ;
                                  SmVector2d sA10 = crOther.ComputePolarCoord(sV10) ;
                                  SmVector2d sA11 = crOther.ComputePolarCoord(sV11) ;
                                  SmVector2d sA01 = crOther.ComputePolarCoord(sV01) ;
                                  SmBoolean  bA00 = crOther.m_sPolarDomain.ContainsPoint2d(sA00, dTol) ;
                                  SmBoolean  bA10 = crOther.m_sPolarDomain.ContainsPoint2d(sA10, dTol) ;
                                  SmBoolean  bA11 = crOther.m_sPolarDomain.ContainsPoint2d(sA11, dTol) ;
                                  SmBoolean  bA01 = crOther.m_sPolarDomain.ContainsPoint2d(sA01, dTol) ;
                                  return(   bA00
                                         && bA10
                                         && bA11
                                         && bA01) ;
                                }
      case SM_PS_UNINITIALIZED:
      case SM_PS_UNBOUNDED:
          break;

    } // end switch on polar boxes

  // should never arrive here
  return(FALSE) ;

} // end SmPolarBox::IsContainedBy


/*******************************************************************//**
PURPOSE: Returns True when the sphere domains are the same to
            within tolerance even when basis may vary.  Domain squares
            must align.

NOTES:
***********************************************************************/
SmBoolean SmPolarBox::AreEqual
  (const SmPolarBox & crOther)
 const
{
  // equal when both are contained by one another
  SmBoolean bRtn =    IsContainedBy(crOther)
                   && crOther.IsContainedBy(*this) ;

  // all done
  return(bRtn) ;

} // end SmPolarBox::AreEqual

/*******************************************************************//**
PURPOSE: Determine if this has a larger span on the sphere than
    the other polar box.  Note that if they are equal FALSE is returned.

NOTES: This is not a comparison of polar box areas.
  Its a comparison of the largest arc contained within the polarBox
  so a large SM_PS_ARC PolarBox will be bigger than a small SM_PS_REGION box.
***********************************************************************/
SmBoolean SmPolarBox::HasLargerSpan
  (const SmPolarBox & crOther)
 const
{
  // Non initialized boxes should never make it to this method
  if(   m_ePolarBoxState         == SM_PS_UNINITIALIZED
     || crOther.m_ePolarBoxState == SM_PS_UNINITIALIZED)
    {
      SE(SM_ERR);
      return FALSE;
    }

  // If both are unbounded, equal domain sizes is FALSE
  if(   m_ePolarBoxState         == SM_PS_UNBOUNDED
     && crOther.m_ePolarBoxState == SM_PS_UNBOUNDED)
    {
      return FALSE;
    }

  // crOther must be bounded so this domain is larger, return TRUE
  if(   m_ePolarBoxState == SM_PS_UNBOUNDED)
    {
      return TRUE;
    }

  // If both are single vector, this domain is not larger than other domain, return FALSE
  if(   (   m_ePolarBoxState         == SM_PS_SINGLE_POINT
         || m_ePolarBoxState         == SM_PS_POINT_WITH_BASIS)
     && (   crOther.m_ePolarBoxState == SM_PS_SINGLE_POINT
         || crOther.m_ePolarBoxState == SM_PS_POINT_WITH_BASIS))
    {
      return FALSE;
    }

  // If other domain is a single point, this domain must be larger domain, return TRUE
  if(   crOther.m_ePolarBoxState == SM_PS_SINGLE_POINT
     || crOther.m_ePolarBoxState == SM_PS_POINT_WITH_BASIS)
    {
      return TRUE;
    }

  double dMaxAngle1 = 0.0;
  double dMaxAngle2 = 0.0;

  // get this polar box corner to corner domain size
  if (   m_ePolarBoxState != SM_PS_SINGLE_POINT
      && m_ePolarBoxState != SM_PS_POINT_WITH_BASIS)
    {
      SmPoint2d sSize = m_sPolarDomain.GetSize();
      dMaxAngle1      = sSize.Length();
    }

  // get other polar box corner to corner domain size
  if (   crOther.m_ePolarBoxState != SM_PS_SINGLE_POINT
      && crOther.m_ePolarBoxState != SM_PS_POINT_WITH_BASIS)
    {
      SmPoint2d sSizeOther = crOther.m_sPolarDomain.GetSize();
      dMaxAngle2 = sSizeOther.Length();
    }

  // When this angle span is larger than other by tolerance return TRUE
  return( dMaxAngle1 > dMaxAngle2+SM_EFF_ZERO) ;

} // end SmPolarBox::HasLargerSpan


/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
void SmPolarBox::Dump
  (void)
 const
{
  TCHAR sBuff[SM_TBLOCK_SIZE], sBuffForFile[SM_TBLOCK_SIZE];
  smos_sprintf(sBuff,       _T("SmPolarBox = 0x%p, m_ePolarBoxState["),this);
  smos_sprintf(sBuffForFile,_T("SmPolarBox = %s, m_ePolarBoxState["),_T("notNULL") );
  smos_WriteBuffer(sBuff, sBuffForFile);

  switch (m_ePolarBoxState)
    {
      case SM_PS_UNINITIALIZED:
          smos_WriteBuffer(_T("SM_PS_UNINITIALIZED]"));
          break;
      case SM_PS_POINT_WITH_BASIS:
          smos_WriteBuffer(_T("SM_PS_POINT_WITH_BASIS]"));
          break;
      case SM_PS_SINGLE_POINT:
          smos_WriteBuffer(_T("SM_PS_SINGLE_POINT]"));
          break;
      case SM_PS_ARC:
          smos_WriteBuffer(_T("SM_PS_ARC]"));
          break;
      case SM_PS_REGION:
          smos_WriteBuffer(_T("SM_PS_REGION]"));
          break;
      case SM_PS_UNBOUNDED:
          smos_WriteBuffer(_T("SM_PS_UNBOUNDED]"));
          break;
    }
  if(   m_ePolarBoxState == SM_PS_UNINITIALIZED
     || m_ePolarBoxState == SM_PS_UNBOUNDED)
    {
      smos_WriteBuffer(_T("\n"));
      return;
    }

  smos_WriteBuffer(_T("\n        Basis vector[0] = ")); m_aBasis[0].Dump();
  //      if(   m_ePolarBoxState == SM_PS_SINGLE_POINT
  //         || m_ePolarBoxState == SM_PS_POINT_WITH_BASIS)
  //        {
  //          smos_WriteBuffer(_T("\n"));
  //          return;
  //        }
  smos_WriteBuffer(_T("\n        Basis vector[1] = ")); m_aBasis[1].Dump();
  smos_WriteBuffer(_T("\n        Basis vector[2] = ")); m_aBasis[2].Dump();
  smos_WriteBuffer(_T("\n        Domain = ")); m_sPolarDomain.Dump();
  smos_sprintf(sBuff,       _T("        PolarArea = %16.16lf"), GetPolarArea());
  smos_WriteBuffer(sBuff) ;
  smos_WriteBuffer(_T("\n"));

} // end SmPolarBox::Dump

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
SmDisplayList * SmPolarBox::Draw
  (const SmPoint3d & crCenter)
 const
{
  SmDisplayList *pRtn = NULL ;

  // skip uninitialized boxes
  if (   m_ePolarBoxState == SM_PS_UNINITIALIZED)
    {
      return(NULL);
    }

#ifdef SM_GFX_CODE
  smgfx_Open(smgfx_GetRuleColor());
  smgfx_OutputColor(smgfx_GetColor()) ;

  ULONG i;
  SmPoint3d sPnts[25];
  ULONG lNumPts=20;
  double dScale = 15.0 ;
  SmVector3d sVec ;

  // output center
  smgfx_DrawPoint(crCenter.x,crCenter.y,crCenter.z) ;

  if(   m_ePolarBoxState == SM_PS_POINT_WITH_BASIS
     || m_ePolarBoxState == SM_PS_SINGLE_POINT
     || m_ePolarBoxState == SM_PS_ARC
     || m_ePolarBoxState == SM_PS_REGION)
    {
      sVec = dScale * EvaluateNormalized(0.0,0.0);
      // Output center vector
      smgfx_DrawLine(crCenter.x,crCenter.y,crCenter.z,
                     crCenter.x+sVec.x,
                     crCenter.y+sVec.y,
                     crCenter.z+sVec.z);

      // output basis vectors
      smgfx_DrawLine(crCenter.x,crCenter.y,crCenter.z,
                     crCenter.x+m_aBasis[0].x,
                     crCenter.y+m_aBasis[0].y,
                     crCenter.z+m_aBasis[0].z) ;
      smgfx_DrawLine(crCenter.x,crCenter.y,crCenter.z,
                     crCenter.x+m_aBasis[1].x,
                     crCenter.y+m_aBasis[1].y,
                     crCenter.z+m_aBasis[1].z) ;
      smgfx_DrawLine(crCenter.x,crCenter.y,crCenter.z,
                     crCenter.x+m_aBasis[2].x,
                     crCenter.y+m_aBasis[2].y,
                     crCenter.z+m_aBasis[2].z) ;

      // output box
      if(   m_ePolarBoxState == SM_PS_ARC
         || m_ePolarBoxState == SM_PS_REGION)
        {
          // sample and draw v=0 isoparameter line
          for (i=0; i<=lNumPts; i++)
            {
              double dAlpha = i/20.0;
              sVec = dScale * EvaluateNormalized(dAlpha,0.0);
              // We are drawing a vector
              if(i==0 || i==lNumPts)
                { smgfx_DrawLine(crCenter.x,crCenter.y,crCenter.z,
                                 crCenter.x+sVec.x,
                                 crCenter.y+sVec.y,
                                 crCenter.z+sVec.z);
                }
              sPnts[i] = sVec + crCenter;
            }

          smgfx_DrawPolyline((double*)sPnts,lNumPts+1);

          if (   m_ePolarBoxState == SM_PS_REGION)
            {
              // sample and draw v=1.0 isoparameter line
              for (i=0; i<=lNumPts; i++)
                {
                  double dAlpha = 1.0 - i/20.0;
                  sVec = dScale * EvaluateNormalized(dAlpha,1.0);
                  // We are drawing a vector
                  if(i==0 || i==lNumPts)
                    { smgfx_DrawLine(crCenter.x,crCenter.y,crCenter.z,
                                     crCenter.x+sVec.x,
                                     crCenter.y+sVec.y,
                                     crCenter.z+sVec.z);
                    }
                  sPnts[i] = sVec + crCenter;
                }
              smgfx_DrawPolyline((double*)sPnts,lNumPts+1);

              // sample and draw u=1.0 isoparameter line
              for (i=0; i<=lNumPts; i++)
                {
                  double dBeta = i/20.0;
                  sVec = dScale * EvaluateNormalized(1.0,dBeta);
                  // We are drawing a vector
                  if(i==0 || i==lNumPts)
                    { smgfx_DrawLine(crCenter.x,crCenter.y,crCenter.z,
                                     crCenter.x+sVec.x,
                                     crCenter.y+sVec.y,
                                     crCenter.z+sVec.z);
                    }
                  sPnts[i] = sVec + crCenter;
                }
              smgfx_DrawPolyline((double*)sPnts,lNumPts+1);

              // sample and draw u = 0.0 isoparameter line
              for (i=0; i<=lNumPts; i++)
                {
                  double dBeta = 1.0 - i/20.0;
                  sVec = dScale * EvaluateNormalized(0.0,dBeta);
                  // We are drawing a vector
                  if(i==0 || i==lNumPts)
                    { smgfx_DrawLine(crCenter.x,crCenter.y,crCenter.z,
                                     crCenter.x+sVec.x,
                                     crCenter.y+sVec.y,
                                     crCenter.z+sVec.z);
                    }
                  sPnts[i] = sVec + crCenter;
                }
              smgfx_DrawPolyline((double*)sPnts,lNumPts+1);
            } // end Region check
        } // end Arc, Region check
    } // end SinglePoint, Arc, Region check

  pRtn = smgfx_Close() ;

#else
  SM_REF1(crCenter);
#endif // end SM_GFX_CODE
  return(pRtn) ;

} // end SmPolarBox::Draw

/*******************************************************************//**
PURPOSE: AssertValid() Test reporting static labels
***********************************************************************/
SmAssertReportLabel sAssertPolarBox_list[] =
{
  {SM_AT_BOX, _T("Transform"), _T("Transform is ok") },
  {SM_AT_BOX, _T("Box State"), _T("m_ePolarBoxState initialized") }
} ;

/*******************************************************************//**
PURPOSE: Virtual method used to determine validity of objects.

RETURNS ---  TRUE  = OK
             FALSE = Problem
***********************************************************************/
SmBoolean SmPolarBox::AssertValid
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

  // return value
  SmBoolean bRtn = TRUE ;

  // Test Transform
  SmBoolean bOk;
  TestTransform(bOk) ;

  bRtn &= SM_ASSERT_BOOLEAN_REPORT(0, SM_LEVEL_0, (bOk), _T("") ) ;

  // only validate initialized BBoxes
  bRtn &= SM_ASSERT_BOOLEAN_REPORT(1, SM_LEVEL_0, (m_ePolarBoxState != SM_PS_UNINITIALIZED), _T("") ) ;

  // all done
  return(bRtn) ;

} // end SmPolarBox::AssertValid

/*******************************************************************//**
PURPOSE: Create an array of vectors and test the cartesian/polar
   transformations.

NOTES: Set rbPassedTheTest if the transformations are inversions,
   that is
   vec = EvaluateNormalized(ComputePolarCoord(vec))
***********************************************************************/
SmStatus SmPolarBox::TestTransform
  (SmBoolean &rbPassedTheTest)  // out: TRUE = passed the test, else FALSE
  const
{
  // no work - polar boxes without domains
  if(   m_ePolarBoxState == SM_PS_UNINITIALIZED
     || m_ePolarBoxState == SM_PS_UNBOUNDED)
    {
      rbPassedTheTest = TRUE ;
      return(SM_SUCCESS) ;
    }
  // locals
  ULONG lCount = 5 ;
  SmVector3d sCartVec ;
  SmPoint2d  sPolarPointIn, sPolarPointOut, sNormalizedPointIn, sNormalizedPointOut ;
  double dNormA1, dNormA2 ;

  // for array of valid angle values
  for(ULONG i=0; i<lCount ; i++)
    {
      for(ULONG j=0; j<lCount ; j++)
        {
          // pick some normalized angles
          dNormA1 = (double)i/(double)(lCount-1) ;
          dNormA2 = (double)j/(double)(lCount-1) ;
          sNormalizedPointIn.Set(dNormA1, dNormA2) ;
          sPolarPointIn = m_sPolarDomain.Evaluate(dNormA1,dNormA2);

          // skip points near the onerous Basis[1]/Basis[2] plane
          if(   sPolarPointIn.x > SM_PI/2.0 - SM_EFF_ZERO_RAD
             || sPolarPointIn.y > SM_PI/2.0 - SM_EFF_ZERO_RAD)
            {
              continue ;
            }

          // build cartesian CVec
          sCartVec = EvaluateNormalized(dNormA1, dNormA2) ;

          // transform the vector into polar space
          sPolarPointOut = ComputePolarCoord(sCartVec) ;

          // transform PolarPoint into normalized coordinates
          GetDomain().Inversion(sPolarPointOut,sNormalizedPointOut) ;

          // compare normalizedPoint with input angles
          SM_ASSERT(   (   m_ePolarBoxState == SM_PS_SINGLE_POINT
                        || m_ePolarBoxState == SM_PS_POINT_WITH_BASIS
                        || smos_Fabs(sPolarPointIn.x - sPolarPointOut.x) < 1000*SM_EFF_ZERO_RAD)
                    && (   m_ePolarBoxState == SM_PS_SINGLE_POINT
                        || m_ePolarBoxState == SM_PS_POINT_WITH_BASIS
                        || m_ePolarBoxState == SM_PS_ARC
                        || smos_Fabs(sPolarPointIn.y - sPolarPointOut.y) < 1000*SM_EFF_ZERO_RAD)) ;


          if(   (   m_ePolarBoxState != SM_PS_SINGLE_POINT
                 && m_ePolarBoxState != SM_PS_POINT_WITH_BASIS
                 && smos_Fabs(sPolarPointIn.x - sPolarPointOut.x) > 1000*SM_EFF_ZERO_RAD)
             || (   m_ePolarBoxState != SM_PS_SINGLE_POINT
                 && m_ePolarBoxState != SM_PS_POINT_WITH_BASIS
                 && m_ePolarBoxState != SM_PS_ARC
                 && smos_Fabs(sPolarPointIn.y - sPolarPointOut.y) > 1000*SM_EFF_ZERO_RAD))
            {
              rbPassedTheTest = FALSE ;
              return(SM_SUCCESS) ;
            }

        } // end iter array of angles
    } // end iter array of angle

  // get here when the transform works
  rbPassedTheTest = TRUE ;
  return(SM_SUCCESS) ;

} // end SmPolarBox::TestTransform

/*******************************************************************//**
PURPOSE: Expand the polar box min/max angles by the given angle value
            in radians without changing the PolarBox state.

NOTES:
  Box domains are modifed depending on their m_ePolarBoxState value as:

    SM_PS_UNDEFINED    boxes                       : No change in domain size
    SM_PS_UNBOUNDED    boxes                       : No Change in domain size
    SM_PS_SINGLE_POINT boxes stay single point     : No change in domain size
    SM_PS_ARC          boxes stay arc boxes        : increase angle1 size only, leave angle2 range = [0:0]
    SM_PS_REGION       boxes stay region boxes     : increate angle1 and angle2 sizes.

***********************************************************************/
void SmPolarBox::ExpandAbsolute
  (double dExpansion)  // in : expansion angle in radians
{
    SM_ASSERT(dExpansion >= 0.0);
    double SM_NEAR_PI = SM_PI - SM_EFF_ZERO ;

    // no work for Uninitialized, singlePoint and Unbounded boxes

    // work for Arc
    if(m_ePolarBoxState == SM_PS_ARC)
      { // only increment and check 1st dimension angles
        SmPoint2d sMin = m_sPolarDomain.GetMin() ;
        SmPoint2d sMax = m_sPolarDomain.GetMax() ;
        sMin.x -= dExpansion ;
        sMax.x += dExpansion ;
        m_sPolarDomain.SetMinMax(sMin, sMax) ;
        if(   sMin.x <= -SM_NEAR_PI
           || sMax.x >=  SM_NEAR_PI)
          { m_ePolarBoxState = SM_PS_UNBOUNDED ;
          }
        return ;
      }

    // work for Region
    else if(m_ePolarBoxState == SM_PS_REGION)
      { // increment and check both dimension angles
        m_sPolarDomain.ExpandAbsolute(dExpansion) ;
        if(   m_sPolarDomain.GetMin().x <= -SM_NEAR_PI
           || m_sPolarDomain.GetMin().y <= -SM_NEAR_PI
           || m_sPolarDomain.GetMax().x >=  SM_NEAR_PI
           || m_sPolarDomain.GetMax().y >=  SM_NEAR_PI)
          { m_ePolarBoxState = SM_PS_UNBOUNDED ;
          }
        return ;
      }

} // end SmPolarBox::ExpandAbsolute
