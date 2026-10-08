// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*******************************************************************//**
* FILE NAME --- SmAxis2Placement.cpp 
* PURPOSE: Implementation of Axis2Placement methods
**********************************************************************/

#include "StdAfx.h"

#include <SmAxis2Placement.h>
#include <SmGraphicsExtern.h>
#include <SmAssertArray.h>   
#include <SmGeomUtility.h>

/*******************************************************************//**
PURPOSE: Construct any SmAxis2Placememnt object that represents the
        plane: Ax + By + Cz + D = 0 
          where: plane normal = [A B C]

NOTES:     
***********************************************************************/
SmAxis2Placement::SmAxis2Placement
  (const SmVector3d &crNormal,    // in : Normal to plane: [A B C]
   double dD)                     // in : D in Ax + By + Cz + D = 0
{
  SM_ASSERT(crNormal.LengthSquared() > SM_EFF_ZERO_SQ);

  // set m_vOrigin
  double dSize = crNormal.Length() ;
  m_vOrigin    = crNormal ;
  m_vOrigin   *= -dD/dSize/dSize ;

  // set m_vXAxis and m_vYAxis axis
  SmVector3d sX;
  SE(crNormal.MakeUnitOrthoVectors(NULL, sX, m_vXAxis, m_vYAxis));

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  if(bDebugMe)
    {
      double dError = crNormal.Dot(m_vOrigin) + dD ;
      double dScaleZero = (1.0 + smos_Max(smos_Fabs(dD), crNormal.GetMaxDimension())) * SM_EFF_ZERO ;
      SM_ASSERT(dError < dScaleZero) ;
    }
#endif // SM_DEBUG_CODE
} // end construct from Plane definition

/*******************************************************************//**
PURPOSE: Set the values of an axis 2 placement object given an
    Origin, X Axis, and Y Axis.  

NOTES: The X and Y axis should be perpendicular.
    The coordinate system defined is right handed.  In other words the
    implied Z axis is X axis cross Y axis.

    When X and Y axes are not unit length - a message is signaled and the
    vectors are resized to unit length.

    When X and Y axes are not perpendicular to one another, an error is returned
    after the input axis values are stored in the SmAxis2Placement object.
     
***********************************************************************/
SmStatus SmAxis2Placement::SetCanonical
( 
  const SmPoint3d  & crOrigin,  // in : coordinate origin
  const SmVector3d & crXAxis,   // in : coordinate X Axis (should be perp to Y axis)
  const SmVector3d & crYAxis    // in : coordinate Y Axis (should be perp to X axis)
)
{
  m_vOrigin = crOrigin;
  m_vXAxis  = crXAxis;
  m_vYAxis  = crYAxis;
  
  // make sure X Axis is a unit vector
  if (smos_Fabs(m_vXAxis.Dot(m_vXAxis)-1.0) > SM_EFF_ZERO/100.0) 
    {
      SER(m_vXAxis.Unitize());
    }

  // make sure Y Axis is a unit vector
  if (smos_Fabs(m_vYAxis.Dot(m_vYAxis)-1.0) > SM_EFF_ZERO/100.0) 
    {
      SER(m_vYAxis.Unitize());
    }

  // make sure vectors are perpendicular
  if (smos_Fabs( m_vXAxis.Dot(m_vYAxis)) > SM_EFF_ZERO) 
    {
      SmVector3d sX, sY, sZ ;
      SER(crXAxis.MakeUnitOrthoVectors(&crYAxis,sX, sY, sZ)) ;
      m_vXAxis = sX;
      m_vYAxis = sY;
    }

  return SM_SUCCESS;

} // end SmAxis2Placement::SetCanonical

/*******************************************************************//**
PURPOSE: Set the values of an axis 2 placement object given an
    Origin, Z Axis, and reference direction for the X Axis.    

NOTES: 
***********************************************************************/
SmStatus SmAxis2Placement::SetSTEPCanonical
  (const SmVector3d & crOrigin,        // in : coordinate origin                           
   const SmVector3d & crZAxis,         // in : coordinate Z Axis 
   const SmVector3d & crXRefDirection) // in : vector that picks Positive X axis direction 
{
    SmVector3d sZAxis, sXAxis, sYAxis;
    SER(crZAxis.MakeUnitOrthoVectors(&crXRefDirection,sZAxis,sXAxis,sYAxis));
    m_vOrigin = crOrigin;
    m_vXAxis  = sXAxis;
    m_vYAxis  = sYAxis;
    return SM_SUCCESS;

} // end SmAxis2Placement::SetSTEPCanonical

/*******************************************************************//**
PURPOSE: Get the Origin, X Axis and Y Axis out of the axis 2 placement.

NOTES: 
***********************************************************************/
void SmAxis2Placement::GetCanonical( SmPoint3d & crOrigin, SmVector3d & crXAxis, SmVector3d & crYAxis ) const
{
  crOrigin = m_vOrigin;
  crXAxis = m_vXAxis;
  crYAxis = m_vYAxis;

} // end SmAxis2Placement::GetCanonical

/*******************************************************************//**
***********************************************************************/

static void sm_CopyMatrix
  (double adMatFrom[4][4],
   double adMatTo[4][4])
{
    for (ULONG i=0; i<4; i++) {
        for (ULONG j=0; j<4; j++) {
            adMatTo[i][j] = adMatFrom[i][j];
        }
    }

} // end sm_CopyMatrix

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
static void sm_MultMatrix
  (double adMat1[4][4],
   double adMat2[4][4],
   double adMatResult[4][4])
{
    for (ULONG i=0; i<4; i++) {
        for (ULONG j=0; j<4; j++) {
            double dVal = 0.0;
            for (ULONG k=0; k<4; k++) {
                dVal += adMat1[i][k] * adMat2[k][j];
            }
            adMatResult[i][j] = dVal;
        }
    }

} // end sm_MultMatrix

/*******************************************************************//**
PURPOSE: Get a 4x4 matrix that represents a mirror matrix.  

NOTES: The mirror is about the X, Y plane of the axis 2 placement.
***********************************************************************/
void SmAxis2Placement::GetMirrorMatrix
  (double * const * padMat) 
 const
{
    double dMat1[4][4], dMat2[4][4], dMat3[4][4];
    double *pdMat[4];

    pdMat[0] = dMat1[0];
    pdMat[1] = dMat1[1];
    pdMat[2] = dMat1[2];
    pdMat[3] = dMat1[3];

    SmAxis2Placement sInv;
    Invert(sInv);

    GetMatrix(pdMat);
    sm_CopyMatrix(dMat1,dMat2);

    SmAxis2Placement sIdent;
    sIdent.GetMatrix(pdMat);
    dMat1[2][2] = -1.0;

    sm_MultMatrix(dMat2,dMat1,dMat3);

    sInv.GetMatrix(pdMat);

    sm_MultMatrix(dMat3,dMat1,dMat2);

    for (ULONG i=0; i<4; i++) {
        for (ULONG j=0; j<4; j++) {
            padMat[i][j] = dMat2[i][j];
        }
    }

} // end SmAxis2Placement::GetMirrorMatrix

/*******************************************************************//**
PURPOSE: Get the matrix form of the SmAxis2Placement.

NOTES: 
***********************************************************************/
void SmAxis2Placement::GetMatrix
  (double * const * padMat)   // out: mat [Xx Xy Xz 0]
                              //          [Yx Yy Yz 0]
                              //          [Zx Zy Zz 0]
                              //          [Ox Oy Oz 1]
                              //      on input allocated as double mat[4][4] ; or equivalent.
 const
{
    const SmVector3d &rX = GetXAxisRef();
    const SmVector3d &rY = GetYAxisRef();
    SmVector3d        sZ = GetZAxis();
    const SmPoint3d  &rP = GetOriginRef();

    padMat[0][0] = rX.x;
    padMat[0][1] = rY.x;
    padMat[0][2] = sZ.x;

    padMat[1][0] = rX.y;
    padMat[1][1] = rY.y;
    padMat[1][2] = sZ.y;

    padMat[2][0] = rX.z;
    padMat[2][1] = rY.z;
    padMat[2][2] = sZ.z;

    padMat[0][3] = rP.x;
    padMat[1][3] = rP.y;
    padMat[2][3] = rP.z;
    
    padMat[3][0] = 0.0;
    padMat[3][1] = 0.0;
    padMat[3][2] = 0.0;
    padMat[3][3] = 1.0;

} // end SmAxis2Placement::GetMatrix

/*******************************************************************//**
PURPOSE: Mirror a point about the X, Y plane of the placement.

NOTES: 
***********************************************************************/
void SmAxis2Placement::MirrorPoint
  (const SmPoint3d & crPointToMirror,   // in : point to mirror
   SmPoint3d       & rMirror)           // out: mirrored point
  const
{
  SmVector3d        sNormal = GetZAxis() ;     // a unit-vector
  const SmPoint3d & rOrigin = GetOriginRef();

  // a tad faster - double the distance from PointToMirror to MirrorPlane
  double dK =   2.0
              * (  (crPointToMirror.x - rOrigin.x) * sNormal.x
                 + (crPointToMirror.y - rOrigin.y) * sNormal.y
                 + (crPointToMirror.z - rOrigin.z) * sNormal.z) ;

  // Mirror About XY plane Point = PointToMirror - 2*ProjToPlaneDist * sNormal
  rMirror   = crPointToMirror - dK * sNormal ;

  //      // equivalent slower way
  //      SmPoint3d         sPntOnPlane ;
  //      // project point to plane
  //      SE(smgu_PointProjectToPlane(crPointToMirror,rOrigin,sZAxis,sPntOnPlane));
  //      
  //      // double the projection to get the mirror point
  //      SmVector3d sVec = sPntOnPlane - crPointToMirror;
  //      rMirror = sPntOnPlane + sVec;
  //      return SM_SUCCESS;

} // end SmAxis2Placement::MirrorPoint

/*******************************************************************//**
PURPOSE: Mirror a vector about the XY plane of the placement.

NOTES: 
***********************************************************************/
void SmAxis2Placement::MirrorVector
( 
  const SmVector3d & crPointToMirror,  // in : vector to mirror
  SmPoint3d & rMirror                  // out: mirrored vector
) const
{
  SmVector3d        sNormal = GetZAxis() ;     // a unit-vector
  //const SmPoint3d & rOrigin = GetOriginRef();

  // 2 times the to MirrorPlane vector component size
  double dK = 2.0 * crPointToMirror.Dot(sNormal) ;

  // Mirror About XY plane Vector = VectorToMirror - 2*ProjToPlaneDist * Normal
  rMirror   = crPointToMirror - dK * sNormal ;

} // end SmAxis2Placement::MirrorVector

/*******************************************************************//**
PURPOSE: Apply a translation vector to the axis 2 placement.

NOTES: 
***********************************************************************/
void SmAxis2Placement::Translate
  (const SmVector3d & crTranslation)
{
  m_vOrigin.x = m_vOrigin.x + crTranslation.x;
  m_vOrigin.y = m_vOrigin.y + crTranslation.y;
  m_vOrigin.z = m_vOrigin.z + crTranslation.z;

} // end SmAxis2Placement::Translate

/*******************************************************************//**
PURPOSE: Rotate the axis 2 placement about the given axis.  

NOTES: The rotation
     angle is defined as the counter-clockwise angle about the axis.  
     This is following the right-hand rule .
***********************************************************************/
void SmAxis2Placement::RotateAboutAxis
  (double              dAngleRadians, // in : rotation angle (radians)
    const SmVector3d & crAxis)        // in : unit-vector rotation axis in
{
  SmVector3d sA     = crAxis ;
  SmStatus   eStat = sA.Unitize() ; 
  if(eStat != SM_SUCCESS)
    {
      SE_MSG(SM_ERR, _T("SmAxis2Placement::RotateAboutAxis given a zero length rotation axis")) ;
      // do the best we can
      sA.Set(1,0,0) ; 
    }

  // locals
  double s = smos_Sine(dAngleRadians);
  double c = smos_Cosine(dAngleRadians);
  double t = 1 - c;
  SmVector3d sOrigin(0,0,0);

  // let XAxis = [1,0,0] rotated to new position
  //      Decompose X unit-vector into its A and PerpToA components.
  //      Define    NormToA = cross(A,PerpToA)   - assumes A is a unit-vector
  //      Rotate the perpToA bit by dAngleRadians about Axis, A.
  //     XAxis = A.Dot(X)*A + cos * (X - A.Dot(X)*A) + sin * Cross(A, X - A.Dot(X)*A)
  //     XAxis = A.x*A + cos * X - cos * A.x*A + sin * Cross(A,X) - sin * A.x * Cross(A,A)
  //     XAxis = (1 - cos)*A.x*A + cos * [1 0 0] + sin * [0 A.z -A.y]
  SmVector3d sXAxis;
  sXAxis.x = t * sA.x * sA.x + c;
  sXAxis.y = t * sA.x * sA.y + s * sA.z;
  sXAxis.z = t * sA.x * sA.z - s * sA.y;

  // let Y Axis = [0,1,0] rotated into new position
  // YAxis = A.Dot(Y)*A + cos * (Y - A.Dot(Y)*A) + sin * Cross(A, Y - A.Dot(Y)*A)
  // YAxis = (1 - cos)*A.y*A + cos * [0 1 0] + sin * [-A.z 0 A.x]
  SmVector3d sYAxis;
  sYAxis.x = t * sA.y * sA.x - s * sA.z;
  sYAxis.y = t * sA.y * sA.y + c;
  sYAxis.z = t * sA.y * sA.z + s * sA.x;

  // reunitize
  SE(sXAxis.Unitize());
  SE(sYAxis.Unitize());

  // reorthogonalize to eliminate tolerance build up
  sYAxis = sXAxis * sYAxis * sXAxis;
  SE(sYAxis.Unitize());

  // create orthoNormal axis from new Origin, X, and Y 
  SmAxis2Placement sOther;
  if (sOther.SetCanonical(sOrigin,sXAxis,sYAxis) != SM_SUCCESS) 
    {
      ERR(SM_ERR);
      return;
    }

  // apply transformation to this coordinate system
  // this = this * sOther 
  this->TransformAxis2Placement(sOther,*this);

} // end SmAxis2Placement::RotateAboutAxis

/*******************************************************************//**
PURPOSE: Rotate the axis 2 placement about an axis which goes through
            the given point and is directed as crAxis. 

NOTES:  The rotation
            angle is defined as the counter-clockwise angle about the axis.  
            (right hand rule).
***********************************************************************/
void SmAxis2Placement::RotateAboutAxisAtPoint
  (double             dAngleRadians, // in : rotation angle radians
   const SmPoint3d&   rCenterArg,    // in : point on rotation axis
   const SmVector3d & crAxis)        // in : direction of rotation axis
{
    // make sure unit vector
    SM_ASSERT(smos_Fabs(crAxis.Dot(crAxis)-1.0) < SM_EFF_ZERO);

    this->Translate(-rCenterArg);
    this->RotateAboutAxis(dAngleRadians, crAxis);

    //      SmAxis2Placement sRot;
    //      sRot.RotateAboutAxis(dAngleRadians, crAxis);
    //      
    //      TransformAxis2Placement(sRot, *this);
 
    Translate(rCenterArg);

} // end SmAxis2Placement::RotateAboutAxisAtPoint

/*******************************************************************//**
PURPOSE: Transform a point using the axis 2 placement. 

NOTES:  This procedure projects a local coordinate Point into
                 global coordinates.  It applies the transformation:

    Outpoint = [InPoint.x InPoint.y InPoint.z 1] * [X0 X1 X2 0]
                                                   [Y0 Y1 Y2 0]
                                                   [Z0 Z1 Z2 0]
                                                   [O0 O1 O2 1]
    which is the same as: OutPoint = Origin + InPoint.x * XAxis
                                            + InPoint.y * YAxis
                                            + InPoint.z * ZAxis
***********************************************************************/
void SmAxis2Placement::TransformPoint
( 
  const SmPoint3d & crUVW,   // in : point to transform
  SmPoint3d       & rXYZ     // out: transformed point
) const
{
  SmVector3d sTmp;
  SmVector3d sZAxis = GetZAxis();
  sTmp.x = m_vOrigin.x + crUVW.x*m_vXAxis.x + crUVW.y*m_vYAxis.x + crUVW.z*sZAxis.x;
  sTmp.y = m_vOrigin.y + crUVW.x*m_vXAxis.y + crUVW.y*m_vYAxis.y + crUVW.z*sZAxis.y;
  sTmp.z = m_vOrigin.z + crUVW.x*m_vXAxis.z + crUVW.y*m_vYAxis.z + crUVW.z*sZAxis.z;
  rXYZ = sTmp;

} // end SmAxis2Placement::TransformPoint

/*******************************************************************//**
PURPOSE: Transform a direction vector using the placement.
  
NOTES: This 
    method essientally takes a vector as defined in the local coordinate
    system of the placement and produces the equivalent vector as defined
    in the global coordinate system.  It only applies the rotation of the
    placement to the vector.

    [outX outY outz 0] = [InX InY InZ 0] * [X0 X1 X2 0]
                                           [Y0 Y1 Y2 0]
                                           [Z0 Z1 Z2 0]
                                           [O0 O1 O2 1]
***********************************************************************/
void SmAxis2Placement::TransformVector
  (const SmVector3d & crUVW,    // in : vector to transform
   SmVector3d       & rXYZ)     // out: transformed vector
  const
{
  SmVector3d sTmp;
  SmVector3d sZAxis = GetZAxis();
  sTmp.x = crUVW.x*m_vXAxis.x + crUVW.y*m_vYAxis.x + crUVW.z*sZAxis.x;
  sTmp.y = crUVW.x*m_vXAxis.y + crUVW.y*m_vYAxis.y + crUVW.z*sZAxis.y;
  sTmp.z = crUVW.x*m_vXAxis.z + crUVW.y*m_vYAxis.z + crUVW.z*sZAxis.z;
  rXYZ = sTmp;

} // end SmAxis2Placement::TransformVector

/*******************************************************************//**
PURPOSE: Express a point with respect to the axis 2 placement. 

NOTES:  This procedure transforms a global coordinate Point into
                 local coordinates.  The operation is the inverse of TransformPoint.
***********************************************************************/
void SmAxis2Placement::InvTransformPoint( 
  const SmPoint3d & crXYZ,   // in : point to inversely transform
  SmPoint3d       & rUVW     // out: point inversely transformed
) const
{
  SmVector3d sDiffVec( crXYZ - m_vOrigin );
  InvTransformVector( sDiffVec, rUVW );

} // end SmAxis2Placement::InvTransformPoint

/*******************************************************************//**
PURPOSE: Express a vector with respect to the axis 2 placement. 

NOTES:  This procedure transforms a global coordinate Point into
                 local coordinates.  The operation is the inverse of TransformVector.

***********************************************************************/
void SmAxis2Placement::InvTransformVector
  (const SmVector3d & crXYZ,   // in : 
   SmVector3d       & rUVW)    // out: 
  const
{
  SmVector3d sTmp;
  SmVector3d sZAxis = GetZAxis();

  sTmp.x = crXYZ.Dot( m_vXAxis );
  sTmp.y = crXYZ.Dot( m_vYAxis );
  sTmp.z = crXYZ.Dot(   sZAxis );

  rUVW = sTmp;
  return;

} // end SmAxis2Placement::InvTransformVector

/*******************************************************************//**
PURPOSE: Compute the placement which is the concatenation of two
    placement objects so that the combined transformation is equivalent
    to applying the this transformation to a point followed by applying
    the crInput transformation.  

NOTES: The order of concatenation is this * crInput = rResults

  [X0 X1 X2 0]   [X0 X1 X2 0]   [X0 X1 X2 0]
  [Y0 Y1 Y2 0] = [Y0 Y1 Y2 0] * [Y0 Y1 Y2 0]
  [Z0 Z1 Z2 0]   [Z0 Z1 Z2 0]   [Z0 Z1 Z2 0]
  [O0 O1 O2 1]   [O0 O1 O2 1]   [O0 O1 O2 1]

   (rResults)  =     this     *    crInput

  so that
  [outX outY outZ 1] = [inX inY inZ 1] * CombinedTransformation, 
  
  is the same as the two step transformation:

  [midX midY midZ 1] = [inX  inY  inZ  1] * ThisTransformation
  [outX outY outZ 1] = [midX midY midZ 1] * crInputTransformaion.

  1. only X, Y, and O (origin) are stored. 
     Each Z is computed as cross(X,Y).
  2. The crInput transformation is assumed to be orthonormal.

***********************************************************************/
void SmAxis2Placement::TransformAxis2Placement
  (const SmAxis2Placement & crInput,     // in : Input    of rResults = this * crInput
   SmAxis2Placement       & rResult)     // out: rResults of rResults = this * crInput
 const
{
  SmVector3d sOrig;
  SmVector3d sXAxis;
  SmVector3d sYAxis;
  SmVector3d sInZAxis = crInput.GetZAxis();

  sXAxis.x = m_vXAxis.x * crInput.m_vXAxis.x + m_vXAxis.y * crInput.m_vYAxis.x + m_vXAxis.z * sInZAxis.x;
  sXAxis.y = m_vXAxis.x * crInput.m_vXAxis.y + m_vXAxis.y * crInput.m_vYAxis.y + m_vXAxis.z * sInZAxis.y;
  sXAxis.z = m_vXAxis.x * crInput.m_vXAxis.z + m_vXAxis.y * crInput.m_vYAxis.z + m_vXAxis.z * sInZAxis.z;

  sYAxis.x = m_vYAxis.x * crInput.m_vXAxis.x + m_vYAxis.y * crInput.m_vYAxis.x + m_vYAxis.z * sInZAxis.x;
  sYAxis.y = m_vYAxis.x * crInput.m_vXAxis.y + m_vYAxis.y * crInput.m_vYAxis.y + m_vYAxis.z * sInZAxis.y;
  sYAxis.z = m_vYAxis.x * crInput.m_vXAxis.z + m_vYAxis.y * crInput.m_vYAxis.z + m_vYAxis.z * sInZAxis.z;

  sOrig.x = m_vOrigin.x * crInput.m_vXAxis.x + m_vOrigin.y * crInput.m_vYAxis.x + m_vOrigin.z * sInZAxis.x +
              crInput.m_vOrigin.x;
  sOrig.y = m_vOrigin.x * crInput.m_vXAxis.y + m_vOrigin.y * crInput.m_vYAxis.y + m_vOrigin.z * sInZAxis.y +
              crInput.m_vOrigin.y;
  sOrig.z = m_vOrigin.x * crInput.m_vXAxis.z + m_vOrigin.y * crInput.m_vYAxis.z + m_vOrigin.z * sInZAxis.z +
              crInput.m_vOrigin.z;

  rResult.m_vOrigin = sOrig;
  rResult.m_vXAxis  = sXAxis;
  rResult.m_vYAxis  = sYAxis;

} // end SmAxis2Placement::TransformAxis2Placement

/*******************************************************************//**
PURPOSE: Find the axis placement which is the inversion of the given
    placement.  

NOTES: The resulting placement takes points from the global
    coordinate system and transforms them into the local coordinate 
    system of the original placement.
***********************************************************************/
void SmAxis2Placement::Invert(SmAxis2Placement & rResult) const
{
    SmVector3d sMyZAxis = GetZAxis();
    SmVector3d sXAxis(m_vXAxis.x,m_vYAxis.x,sMyZAxis.x);
    SmVector3d sYAxis(m_vXAxis.y,m_vYAxis.y,sMyZAxis.y);
    SmVector3d sZAxis(m_vXAxis.z,m_vYAxis.z,sMyZAxis.z);

    // rResult.m_vOrigin.Set(0,0,0);
    
    SmVector3d sNegTran = -m_vOrigin;
    SmVector3d sInvOrig;
    sInvOrig.x = - m_vXAxis.Dot(m_vOrigin);
    sInvOrig.y = - m_vYAxis.Dot(m_vOrigin);
    sInvOrig.z = - sMyZAxis.Dot(m_vOrigin);

#ifdef SM_DEBUG_CODE
    SmAxis2Placement sCopy(*this);
#endif // SM_DEBUG_CODE

    rResult.m_vOrigin = sInvOrig;
    rResult.m_vXAxis  = sXAxis;
    rResult.m_vYAxis  = sYAxis;

    // For Debugging
#ifdef SM_DEBUG_CODE
    SmAxis2Placement sTmp, sIdentity;
    sCopy.TransformAxis2Placement(rResult,sTmp); // sTmp should be the identity transform
    SM_ASSERT(sTmp == sIdentity) ;
#endif // SM_DEBUG_CODE


} // end SmAxis2Placement::Invert

/*******************************************************************//**
PURPOSE: Decompose the transformation implied by the vectors of the 
    placement into a set of sequencial rotation angles.  

NOTES: 
***********************************************************************/
void SmAxis2Placement::DecomposeToAngles
  (double & rdXRotRad,      // out:
   double & rdYRotRad,      // out:
   double & rdZRotRad)      // out:
 const
{
  const SmVector3d &rX = GetXAxisRef();
  const SmVector3d &rY = GetYAxisRef();
  SmVector3d sZ = GetZAxis();

  rdYRotRad = smos_ArcSine(-rX.z);

  if ( fabs(smos_Cosine(rdYRotRad)) > SM_EFF_ZERO) 
    {
      rdXRotRad = smos_ArcTangent2(rY.z, sZ.z);
      rdZRotRad = smos_ArcTangent2(rX.y, rX.x);
    } else 
    {
      rdXRotRad = 0.0;
      rdZRotRad = smos_ArcTangent2(-rY.x, rY.y);
    }

#ifdef SM_DEBUG_CODE
  // The following code can be used to validate the
  // results of inversion
SmBoolean bDebugMe = FALSE ;
  if(bDebugMe)
    {
      SmAxis2Placement sTestA2P;
      SmPoint3d sOrigin = this->GetOrigin();
      sTestA2P.ComposeFromAngles(rdXRotRad,rdYRotRad,rdZRotRad,&sOrigin);
      this->Dump();
      sTestA2P.Dump();
    }

#endif

} // end SmAxis2Placement::DecomposeToAngles

/*******************************************************************//**
PURPOSE: Compose the transformation from the set of sequencial rotation angles.  

NOTES:  Returned SmAxis2Placement is centered on the origin
***********************************************************************/
void SmAxis2Placement::ComposeFromAngles
  (double      dXRotRad,     // in : 1st rotation about X Axis 
   double      dYRotRad,     // in : 2nd rotation about Y Axis
   double      dZRotRad,     // in : 3rd rotation about Z Axis
   SmPoint3d * pOptCenter)   // in : Center, NULL=leave at origin
                             //      default:[NULL]
{
#ifdef SM_DEBUG_CODE
  SmAxis2Placement sOrigA2P(*this) ;
#endif

  // init to x      = [1,0,0]
  //         y      = [0,1,0]
  //         origin = [0,0,0]
  Init() ;

  // rotate about global X by wAngleX
  RotateAboutAxis(dXRotRad, SmVector3d(1, 0, 0));

  // rotate about global Y by wAngleY
  RotateAboutAxis(dYRotRad, SmVector3d(0, 1, 0));

  // rotate about global Z by wAngleZ
  RotateAboutAxis(dZRotRad, SmVector3d(0, 0, 1));

  // set transformation origin
  if(pOptCenter) 
    {
      Translate(*pOptCenter) ;
    }

#ifdef SM_DEBUG_CODE
  // The following code can be used to validate the
  // results of inversion
SmBoolean bDebugMe = FALSE ;
  if(bDebugMe)
    {
      SmPoint3d sTestOrigin ;
      double dTestXRotRad, dTestYRotRad, dTestZRotRad ;
      DecomposeToAngles(dTestXRotRad, dTestYRotRad, dTestZRotRad) ;
      sTestOrigin = GetOriginRef() ;

      SM_ASSERT(SM_IS_ZERO(dTestXRotRad - dXRotRad)) ;
      SM_ASSERT(SM_IS_ZERO(dTestYRotRad - dYRotRad)) ;
      SM_ASSERT(SM_IS_ZERO(dTestZRotRad - dZRotRad)) ;
      
      if(pOptCenter)
        {
          SM_ASSERT(SM_IS_ZERO(sTestOrigin.x - pOptCenter->x)) ;
          SM_ASSERT(SM_IS_ZERO(sTestOrigin.y - pOptCenter->y)) ;
          SM_ASSERT(SM_IS_ZERO(sTestOrigin.z - pOptCenter->z)) ;
        }

      sOrigA2P.Dump() ;
      this->Dump() ;
    }
#endif

} // end SmAxis2Placement::ComposeFromAngles

/*******************************************************************//**
PURPOSE: Given a placement, generate the equivalent 4x4 matrix used
    by many systems.

NOTES: 
***********************************************************************/
void SmAxis2Placement::Load4x4
  (double ad4x4Matrix[16])       // out: Matrix to load
 const
{
// Note that this is a graphical 4x4 and is different from the typical geometric 4x4

  // load X and Y rows of matrix
  ad4x4Matrix[0]    = m_vXAxis.x;
  ad4x4Matrix[1]    = m_vXAxis.y;
  ad4x4Matrix[2]    = m_vXAxis.z;
  ad4x4Matrix[3]    = 0;
  ad4x4Matrix[4]    = m_vYAxis.x;
  ad4x4Matrix[5]    = m_vYAxis.y;
  ad4x4Matrix[6]    = m_vYAxis.z;
  ad4x4Matrix[7]    = 0;

  // compute Z now that we can
  SmVector3d sZAxis = GetZAxis();

  // load rest of matrix
  ad4x4Matrix[8]    = sZAxis.x;
  ad4x4Matrix[9]    = sZAxis.y;
  ad4x4Matrix[10]   = sZAxis.z;
  ad4x4Matrix[11]   = 0.0;
  ad4x4Matrix[12]   = m_vOrigin.x;
  ad4x4Matrix[13]   = m_vOrigin.y;
  ad4x4Matrix[14]   = m_vOrigin.z;
  ad4x4Matrix[15]   = 1.0;

} // end SmAxis2Placement::Load4x4

/*******************************************************************//**
PURPOSE: Given the equivalent 4x4 matrix used by many systems
  set Origin, XAxis and YAxis values.

NOTES: 
***********************************************************************/
void SmAxis2Placement::SetFrom4x4
  (double ad4x4Matrix[16])       // out: Matrix to load
{
// Assume the ad4x4Matrix[16] is orthonormal
// we ignore the Z information and use only the x, y and origin data

  // load X and Y rows of matrix
  m_vXAxis.x = ad4x4Matrix[0] ;
  m_vXAxis.y = ad4x4Matrix[1] ;
  m_vXAxis.z = ad4x4Matrix[2] ;

  m_vYAxis.x = ad4x4Matrix[4] ;
  m_vYAxis.y = ad4x4Matrix[5] ;
  m_vYAxis.z = ad4x4Matrix[6] ;

  m_vOrigin.x  =  ad4x4Matrix[12] ; 
  m_vOrigin.y  =  ad4x4Matrix[13] ; 
  m_vOrigin.z  =  ad4x4Matrix[14] ; 

} // end SmAxis2Placement::Set4x4

/*******************************************************************//**
PURPOSE: Equality operator for SmAxis2Placement

NOTES: 
***********************************************************************/
SmBoolean SmAxis2Placement::operator==
  (const SmAxis2Placement& crOther) 
 const
{
  // when objects are equivalent
  if ( this == &crOther ) { return TRUE; }

  if ( ! SM_ARE_SAME(m_vOrigin.x, crOther.m_vOrigin.x) ) { return FALSE; }
  if ( ! SM_ARE_SAME(m_vOrigin.y, crOther.m_vOrigin.y) ) { return FALSE; }
  if ( ! SM_ARE_SAME(m_vOrigin.z, crOther.m_vOrigin.z) ) { return FALSE; }
  
  if ( ! SM_ARE_SAME(m_vXAxis.x,  crOther.m_vXAxis.x) ) { return FALSE; }
  if ( ! SM_ARE_SAME(m_vXAxis.y,  crOther.m_vXAxis.y) ) { return FALSE; }
  if ( ! SM_ARE_SAME(m_vXAxis.z,  crOther.m_vXAxis.z) ) { return FALSE; }
                                                    
  if ( ! SM_ARE_SAME(m_vYAxis.x,  crOther.m_vYAxis.x) ) { return FALSE; }
  if ( ! SM_ARE_SAME(m_vYAxis.y,  crOther.m_vYAxis.y) ) { return FALSE; }
  if ( ! SM_ARE_SAME(m_vYAxis.z,  crOther.m_vYAxis.z) ) { return FALSE; }
  
  return TRUE;

} // end SmAxis2Placement::operator==

/*******************************************************************//**
PURPOSE: Is this the identity transform?

NOTES: 
***********************************************************************/
SmBoolean SmAxis2Placement::IsIdentity
  ( double dTol )  // in: default [ SM_EFF_ZERO ]
 const
{
   if ( m_vOrigin.x > dTol ) { return FALSE; }
   if ( m_vOrigin.y > dTol ) { return FALSE; }
   if ( m_vOrigin.z > dTol ) { return FALSE; }

   if ( m_vXAxis.x > 1.0+dTol ) { return FALSE; }
   if ( m_vXAxis.y >     dTol ) { return FALSE; }
   if ( m_vXAxis.z >     dTol ) { return FALSE; }

   if ( m_vYAxis.x >     dTol ) { return FALSE; }
   if ( m_vYAxis.y > 1.0+dTol ) { return FALSE; }
   if ( m_vYAxis.z >     dTol ) { return FALSE; }

   if ( m_vOrigin.x < -dTol ) { return FALSE; }
   if ( m_vOrigin.y < -dTol ) { return FALSE; }
   if ( m_vOrigin.z < -dTol ) { return FALSE; }

   if ( m_vXAxis.x < 1.0-dTol ) { return FALSE; }
   if ( m_vXAxis.y <    -dTol ) { return FALSE; }
   if ( m_vXAxis.z <    -dTol ) { return FALSE; }

   if ( m_vYAxis.x <    -dTol ) { return FALSE; }
   if ( m_vYAxis.y < 1.0-dTol ) { return FALSE; }
   if ( m_vYAxis.z <    -dTol ) { return FALSE; }

   return TRUE;

} // end SmAxis2Placement::IsIdentity


/*******************************************************************//**
PURPOSE: Do two SmAxis2Placements have the same axis?

NOTES: The argument dSizeScale is the distance from the origins (of
   both SmAxis2Placements) at which we want the axes to be within sTol3d.
   If an angle tolerance is more appropriate, that could be implemented.
***********************************************************************/
SmBoolean SmAxis2Placement::AreCoaxial( const SmAxis2Placement & crOther,
                                              SmApproxTol3d      sTol3d,
                                              double             dSizeScale
        ) const
{
  const SmPoint3d  &rThisPt  = this->GetOriginRef();
  const SmVector3d  sThisVec = this->GetZAxis();
  const SmPoint3d  &rOthrPt  = crOther.GetOriginRef();
  const SmVector3d  sOthrVec = crOther.GetZAxis();

  double dDist=0;

  smgu_LinePointDistance( rThisPt, sThisVec, rOthrPt, dDist );
  if ( dDist > sTol3d )
    { return FALSE; }

  smgu_LinePointDistance( rOthrPt, sOthrVec, rThisPt, dDist );
  if ( dDist > sTol3d )
    { return FALSE; }

  // Get a point on the axis that is dSizeScale away from origin.
  SmPoint3d sPt = rOthrPt + dSizeScale * sOthrVec;
  smgu_LinePointDistance( rThisPt, sThisVec, sPt, dDist );
  if ( dDist > sTol3d )
    { return FALSE; }

  sPt = rOthrPt - dSizeScale * sOthrVec;
  smgu_LinePointDistance( rThisPt, sThisVec, sPt, dDist );
  if ( dDist > sTol3d )
    { return FALSE; }

  sPt = rThisPt + dSizeScale * sThisVec;
  smgu_LinePointDistance( rOthrPt, sOthrVec, sPt, dDist );
  if ( dDist > sTol3d )
    { return FALSE; }

  sPt = rThisPt - dSizeScale * sThisVec;
  smgu_LinePointDistance( rOthrPt, sOthrVec, sPt, dDist );
  if ( dDist > sTol3d )
    { return FALSE; }

  return TRUE;

} // end AreCoaxial

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
void SmAxis2Placement::Dump(void) const
{
    TCHAR sBuff[SM_TBLOCK_SIZE], sBuffForFile[SM_TBLOCK_SIZE];
    smos_sprintf(sBuff,       _T("SmAxis2Placement = 0x%p"),this);
    smos_sprintf(sBuffForFile,_T("SmAxis2Placement = %s"),_T("notNULL"));
    smos_WriteBuffer(sBuff, sBuffForFile);
    smos_WriteBuffer(_T("\n       Origin = "));
    m_vOrigin.Dump();
    smos_WriteBuffer(_T("\n       XAxis  = "));
    m_vXAxis.Dump();
    smos_WriteBuffer(_T("\n       YAxis  = "));
    m_vYAxis.Dump(); 
    smos_WriteBuffer(_T("\n"));

} // end SmAxis2Placement::Dump

/*******************************************************************//**
PURPOSE: Draw Axis - origin(black), x(red), y(green), z(blue)

NOTES:
***********************************************************************/
SmDisplayList *SmAxis2Placement::Draw(SmExtent2d *pOptUVDomain) const
{
    SmDisplayList *pRtn = NULL ;

#ifdef SM_GFX_CODE
    SmVector3d sColor = smgfx_GetRuleColor();

    smgfx_Open(smgfx_GetRuleColor());
    SmVector3d sZAxis = GetZAxis();

    smgfx_SetColor(0,0,0); m_vOrigin.Draw();
    smgfx_SetColor(1,0,0); m_vXAxis.Draw(&m_vOrigin);
    smgfx_SetColor(0,1,0); m_vYAxis.Draw(&m_vOrigin);
    smgfx_SetColor(0,0,1); sZAxis.Draw(&m_vOrigin);
    smgfx_SetColor(sColor);

    // some plane graphics
    ULONG ii ;
    double daPoints[15] ;
    for(ii=0;ii<3;ii++)
      {
        double dScale = (ii+1.0) * (ii+1.0) * (ii+1.0) ;
        SmPoint3d *pTgt0 = (SmPoint3d *)daPoints ;
        SmPoint3d *pTgt1 = (SmPoint3d *)(daPoints+3) ;
        *pTgt0 = m_vOrigin + dScale * (m_vXAxis + m_vYAxis) ; 
        *pTgt1 = *pTgt0 - 2 * dScale * m_vXAxis ; pTgt0++ ; pTgt1++ ;
        *pTgt1 = *pTgt0 - 2 * dScale * m_vYAxis ; pTgt0++ ; pTgt1++ ;
        *pTgt1 = *pTgt0 + 2 * dScale * m_vXAxis ; pTgt0++ ; pTgt1++ ;
        *pTgt1 = *pTgt0 + 2 * dScale * m_vYAxis ; pTgt0++ ; pTgt1++ ;

        smgfx_DrawPolyline(daPoints, 5) ;
      } // end iter plane graphics

    // optional extent graphics
    if(pOptUVDomain)
      {
        double dLineWidth = smgfx_GetLineWidth() ;
        double dPointSize = smgfx_GetPointSize() ;
        smgfx_SetLook(dLineWidth+2,dPointSize, 0,0,0) ;

        (SmPoint3d &)(*(daPoints+ 0)) = m_vOrigin + pOptUVDomain->GetUMin() * m_vXAxis + pOptUVDomain->GetVMin() * m_vYAxis ; 
        (SmPoint3d &)(*(daPoints+ 3)) = m_vOrigin + pOptUVDomain->GetUMin() * m_vXAxis + pOptUVDomain->GetVMax() * m_vYAxis ; 
        (SmPoint3d &)(*(daPoints+ 6)) = m_vOrigin + pOptUVDomain->GetUMax() * m_vXAxis + pOptUVDomain->GetVMax() * m_vYAxis ; 
        (SmPoint3d &)(*(daPoints+ 9)) = m_vOrigin + pOptUVDomain->GetUMax() * m_vXAxis + pOptUVDomain->GetVMin() * m_vYAxis ; 
        (SmPoint3d &)(*(daPoints+12)) = m_vOrigin + pOptUVDomain->GetUMin() * m_vXAxis + pOptUVDomain->GetVMin() * m_vYAxis ; 
                        
        smgfx_DrawPolyline(daPoints, 5) ;
                        
        smgfx_SetLook(dLineWidth, dPointSize, sColor.x, sColor.y, sColor.z) ;

      } // end option UVDomain existence check


    pRtn = smgfx_Close() ;

#else
    SM_REF1(pOptUVDomain);
#endif
    return(pRtn) ;

} // end SmAxis2Placement::Draw

/*******************************************************************//**
PURPOSE: AssertValid() Test reporting static labels
***********************************************************************/
SmAssertReportLabel sAssertAxis2Placement_list[] =
{
  {SM_AT_ANGLE,       _T("Ortho Vectors"), _T("m_vXAxis and m_vYAxis are orthogonal") },
  {SM_AT_UNIT_VECTOR, _T("Unit X Vector"), _T("m_vXAxis is a unit vector") },
  {SM_AT_UNIT_VECTOR, _T("Unit Y Vector"), _T("m_vYAxis is a unit vector") }
} ;

/*******************************************************************//**
PURPOSE:

RETURNS ---  TRUE  = OK
             FALSE = Problem
***********************************************************************/
SmBoolean SmAxis2Placement::AssertValid
 (SmAssertArray    * pAList,        // i/o: Accumulating list of failed Asserts, NULL to ignore, default:[NULL] 
  SmAssertTestLevel  eTestLevel,    // in : SM_LEVEL_0=fewest tests, SM_LEVEL_1, SM_LEVEL_2=all tests, 
                                    //      SM_LEVEL_GIVEN = run tests in order requested in pTestRequests 
                                    //      default:[SM_LEVEL_0] 
  SmAssertWalking    eWalkTree,     // NotUsed: in : SM_WALK = run AssertValid on any topology graph descendants, SM_NO_WALK=don't, default:[SM_WALK] 
  SmTArray<ULONG>  * pTestRequests) // in : when eTestLevel == SM_LEVEL_GIVEN, run these tests in this order, default:[NULL]
 const
{
  SM_REF2(eWalkTree, pTestRequests);

  // init return value
  SmBoolean bRtn = TRUE ;

  // X and Y are perpendicular
  bRtn &= SM_ASSERT_VALUE_REPORT(0, SM_LEVEL_0, (smos_Fabs(m_vXAxis.Dot(m_vYAxis)) < SM_EFF_ZERO), SM_EFF_ZERO, smos_Fabs(m_vXAxis.Dot(m_vYAxis)), _T("")) ;

  // X is a unit vector
  bRtn &= SM_ASSERT_VALUE_REPORT(1, SM_LEVEL_0, (smos_Fabs(m_vXAxis.Dot(m_vXAxis)-1.0) < SM_EFF_ZERO), SM_EFF_ZERO, smos_Fabs(m_vXAxis.Dot(m_vXAxis)-1.0), _T("")) ;

  // Y is a unit vectors
  bRtn &= SM_ASSERT_VALUE_REPORT(2, SM_LEVEL_0, (smos_Fabs(m_vYAxis.Dot(m_vYAxis)-1.0) < SM_EFF_ZERO), SM_EFF_ZERO, m_vYAxis.Dot(m_vYAxis)-1.0, _T("")) ;

  // all done
  return(bRtn) ;

} // end SmAxis2Placement::AssertValid

