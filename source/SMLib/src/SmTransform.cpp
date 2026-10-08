// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*******************************************************************//**
* FILE NAME --- SmTransform.cpp 
* PURPOSE: Implementation of Axis2Placement methods
**********************************************************************/

#include "StdAfx.h"

#include <SmTransform.h>
#include <SmGeomUtility.h>
#include <SmGraphicsExtern.h>
#include <SmAssertArray.h>
#include <SmAxis2Placement.h>
#include <SmPseudoBox.h>
#include <SmBSplineCurve.h>
#include <SmBSplineSurface.h>
#include <SmLine.h>
#include <SmPlane.h>
#include <SmNurbsCrv.h>
#include <SmDatabaseIO.h>

/*******************************************************************//**
PURPOSE: copy constructor

NOTES:
***********************************************************************/
SmTransform::SmTransform 
 (const SmTransform &crFromObject,   // in : SourceVolume to copy
  SmBoolean          bSimpleMapOnly) // in : TRUE = Copy this Volume omitting any compounding volumes
                                     //      FALSE= Copy this Volumes with any compounding volumes
: SmVolume(crFromObject, bSimpleMapOnly)
{ 
  // copy the transform matrix
  smgu_4x4Copy(crFromObject.m_adT, m_adT) ; 

} // end SmTransform::SmTransform copy constructor

/*******************************************************************//**
PURPOSE: assignment operator

NOTES:
***********************************************************************/
SmTransform & SmTransform::operator=
  (const SmTransform &crTransform)       // in : object to copy
{ 
  if(this == &crTransform) return(*this) ;
  
  // assign base values
  SmVolume::operator=(crTransform) ;
                                                              
  // copy the transform matrix
  smgu_4x4Copy(crTransform.m_adT, m_adT) ; 

  // all done
  return(*this) ;

} // end SmTransform::operator=

/*******************************************************************//**
PURPOSE: Equality operator for SmTransform

NOTES: Call base equivalence to check type and then check 
       members for equivalence
***********************************************************************/
SmBoolean SmTransform::operator==
  (const SmVolume &crOther) 
 const
{
  // low work
  if(this == &crOther) { return TRUE ; }

  // first check the base
  SmBoolean bRtn = SmVolume::operator ==(crOther) ;

  if(bRtn)
    {
      // OK to cast
      SmTransform &rOther = (SmTransform &)crOther ;

      // check equivalence of these objects
      bRtn = (   SM_IS_ZERO(m_adT[0][0] - rOther.m_adT[0][0])
              && SM_IS_ZERO(m_adT[0][1] - rOther.m_adT[0][1])   
              && SM_IS_ZERO(m_adT[0][2] - rOther.m_adT[0][2])   
              && SM_IS_ZERO(m_adT[0][3] - rOther.m_adT[0][3])   
                                                             
              && SM_IS_ZERO(m_adT[1][0] - rOther.m_adT[1][0])   
              && SM_IS_ZERO(m_adT[1][1] - rOther.m_adT[1][1])   
              && SM_IS_ZERO(m_adT[1][2] - rOther.m_adT[1][2])   
              && SM_IS_ZERO(m_adT[1][3] - rOther.m_adT[1][3])   
                                                             
              && SM_IS_ZERO(m_adT[2][0] - rOther.m_adT[2][0])   
              && SM_IS_ZERO(m_adT[2][1] - rOther.m_adT[2][1])   
              && SM_IS_ZERO(m_adT[2][2] - rOther.m_adT[2][2])   
              && SM_IS_ZERO(m_adT[2][3] - rOther.m_adT[2][3])   
                                                             
              && SM_IS_ZERO(m_adT[3][0] - rOther.m_adT[3][0])   
              && SM_IS_ZERO(m_adT[3][1] - rOther.m_adT[3][1])   
              && SM_IS_ZERO(m_adT[3][2] - rOther.m_adT[3][2])   
              && SM_IS_ZERO(m_adT[3][3] - rOther.m_adT[3][3]) ) ;  
    }

  // all done
  return bRtn ;

} // end SmTransform::operator==

/*******************************************************************//**
PURPOSE: Create a SmBendVolume from component data.  

NOTES: 
***********************************************************************/
SmStatus SmTransform::CreateCanonical
 (const SmContext  & crContext,         // in : context for new object construction
  const SmVector3d & crToOrigin,        // in : ToSpace origin
  const SmVector3d & crToXAxis,         // in : ToSpace XAxis 
  const SmVector3d & crToYAxis,         // in : ToSpace YAxis 
  const SmVector3d * cpToZAxis,         // in : opt ToSpace ZAxis, NULL: let ToSpaceZ = cross(ToXAxis, ToYAxis)
  SmTransform     *& rpNewVolume)       // out: New SmBendVolume, NULL on input 
{
  // build the object
  rpNewVolume = new (crContext) SmTransform(&crContext,
                                            crToOrigin,   
                                            crToXAxis,    
                                            crToYAxis,    
                                            cpToZAxis) ;

  // all done
  return(SM_SUCCESS) ; 

} // end SmTransform::CreateCanonical

/*******************************************************************//**
PURPOSE: Construct a rotate and translate transform from 
         a moveTo vector and a set of rotateTo vectors for the x y and z axes.  

NOTES: 
  1. when the rotateToVectors are orthogonal unit-vectors the
     constructed transform is a pure rotation about the origin followed by
     a pure translation to the ToOrigin.
  
  2. NonUnit RotateToVectors add scaling to the transform.

  3. NonOrthogonal RotateToVectors 
     
***********************************************************************/
void SmTransform::SetCanonical
  (const SmVector3d & crToOrigin,        // in : ToSpace origin
   const SmVector3d & crToXAxis,         // in : ToSpace XAxis 
   const SmVector3d & crToYAxis,         // in : ToSpace YAxis 
   const SmVector3d * cpToZAxis)         // in : opt ToSpace ZAxis, NULL: let ToSpaceZ = cross(ToXAxis, ToYAxis)
                                         //      default:[NULL]
{
  // check for optional Z axis
  SmVector3d sZAxis = cpToZAxis ? *cpToZAxis :  crToXAxis * crToYAxis ;

  // set the T array
  SetToDisp (crToOrigin) ;
  SetToXAxis(crToXAxis) ;
  SetToYAxis(crToYAxis) ;
  SetToZAxis(sZAxis) ;
  InitExt() ; 

} // end SmTransform::SetCanonical

/*******************************************************************//**
PURPOSE: Get the Origin, X Axis and Y Axis out of the axis 2 placement.

NOTES: 
***********************************************************************/
void SmTransform::GetCanonical
 (SmVector3d & rToDisp,           // out: ToSpace displacement
  SmVector3d & rToXAxis,          // out: ToSpace XAxis 
  SmVector3d & rToYAxis,          // out: ToSpace YAxis 
  SmVector3d & rToZAxis)          // out: ToSpace ZAxis  
 const
{
   rToDisp  = GetToDisp() ;
   rToXAxis = GetToXAxis() ;
   rToYAxis = GetToYAxis() ;
   rToZAxis = GetToZAxis() ;

} // end SmTransform::GetCanonical

/*******************************************************************//**
PURPOSE: Transform a point from paramUVW space to projXYZ space 

NOTES:  It applies the transform

    ProjPoint = [ParamPoint.x ParamPoint.y ParamPoint.z 1] * [X0 X1 X2 0]
                                                             [Y0 Y1 Y2 0]
                                                             [Z0 Z1 Z2 0]
                                                             [O0 O1 O2 1]
***********************************************************************/
void SmTransform::TransformPoint
  (const SmPoint3d & crParamPoint,   // in : point to transform
    SmPoint3d       & rProjPoint)     // out: transformed point
  const
{
  // locals
  SmVector3d &rToX = GetToXAxis() ;
  SmVector3d &rToY = GetToYAxis() ;
  SmVector3d &rToZ = GetToZAxis() ;
  SmVector3d &rToD = GetToDisp () ;

  // multiply into local memory to allow rProjPoint to be same memory as crParamPoint
  SmVector3d sTmp;
  sTmp.x = rToD.x + crParamPoint.x*rToX.x + crParamPoint.y*rToY.x + crParamPoint.z*rToZ.x;
  sTmp.y = rToD.y + crParamPoint.x*rToX.y + crParamPoint.y*rToY.y + crParamPoint.z*rToZ.y;
  sTmp.z = rToD.z + crParamPoint.x*rToX.z + crParamPoint.y*rToY.z + crParamPoint.z*rToZ.z;

  // set output
  rProjPoint = sTmp;

} // end SmTransform::TransformPoint

/*******************************************************************//**
Transform a vector from paramUVW space to projXYZ space 

NOTES:  It applies the transform

    ProjPoint = [ParamPoint.x ParamPoint.y ParamPoint.z 0] * [X0 X1 X2 0]
                                                             [Y0 Y1 Y2 0]
                                                             [Z0 Z1 Z2 0]
                                                             [O0 O1 O2 1]
***********************************************************************/
void SmTransform::TransformVector
  (const SmVector3d & crParamVec,     // in : vector to transform
   SmVector3d       & rProjVec)       // out: transformed vector
  const
{
  // locals
  SmVector3d &rToX = GetToXAxis() ;
  SmVector3d &rToY = GetToYAxis() ;
  SmVector3d &rToZ = GetToZAxis() ;

  // multiply into local memory to allow rProjVec to be same memory as crParamVec
  SmVector3d sTmp;
  sTmp.x = crParamVec.x*rToX.x + crParamVec.y*rToY.x + crParamVec.z*rToZ.x;
  sTmp.y = crParamVec.x*rToX.y + crParamVec.y*rToY.y + crParamVec.z*rToZ.y;
  sTmp.z = crParamVec.x*rToX.z + crParamVec.y*rToY.z + crParamVec.z*rToZ.z;

  // set output
  rProjVec = sTmp;

} // end SmTransform::TransformVector

/*******************************************************************//**
PURPOSE: Transform a point from XYZ ProjSpace to UVW ParamSpace

NOTES:  This is the inverse mapping done by TransformPoint
  It applies the transform
   
      rParamPoint = [(crProjPoint - rToDisp) 0] * Inv[X0 X1 X2]
                                                     [Y0 Y1 Y2]
                                                     [Z0 Z1 Z2]
***********************************************************************/
SmStatus SmTransform::InvTransformPoint
  (const SmPoint3d & crProjPoint,  // in : point existing in XYZ ProjSpace
   SmPoint3d       & rParamPoint)  // out: point inversely transformed to UVW ParamSpace
  const
{
  // locals
  SmVector3d &rToDisp = GetToDisp () ;

  // do the easy part - subtract current displacement
  SmVector3d sMid(crProjPoint - rToDisp) ;

  // Map result through Axis matrix inverse
  return(InvTransformVector(sMid, rParamPoint)) ;

} // end SmTransform::InvTransformPoint

/*******************************************************************//**
PURPOSE: Transform a vector from XYZ ProjSpace to UVW ParamSpace

NOTES:  This is the inverse mapping done by TransformVector
  It applies the transform

      ParamVec = [ProjVec.x ProjVec.y ProjVec.z 0] * Inv[X0 X1 X2 0]
                                                        [Y0 Y1 Y2 0]
                                                        [Z0 Z1 Z2 0]
                                                        [O0 O1 O2 1]
***********************************************************************/
SmStatus SmTransform::InvTransformVector
  (const SmVector3d & crProjVec,  // in : vector existing in XYZ ProjSpace
   SmVector3d       & rParamVec)  // out: vector inversely transformed to UVW ParamSpace
  const
{
  double adInv[3][3] ;

  // invert the rotation part of the Transform - return SM_ERR for degenerate matrices
  if(SM_ERR == smgu_3x3Inverse((double *)&GetToXAxis(), 
                               (double *)&GetToYAxis(), 
                               (double *)&GetToZAxis(), 
                               adInv))
    { // matrix can't be inverted - it has a zero determinate (it's degenerate)
      // this is expected for Transforms that project to a plane but an error for everything else.
      return(SM_ERR) ; 
    }

  // Put Inv into a transform with 0 displacement
  SmVector3d  sOrig(0,0,0) ;
  SmTransform sInv(GetContext(), sOrig, *((SmVector3d *)adInv[0]),  *((SmVector3d *)adInv[1]),  ((SmVector3d *)adInv[2])) ;

  // map the vector
  sInv.TransformVector(crProjVec, rParamVec) ;

  // all done
  return(SM_SUCCESS) ;

} // end SmTransform::InvTransformVector

/*******************************************************************//**
PURPOSE: concatanate input Transform to end of this Transform sequence

NOTES: builds this->Matrix = this->Matrix * input->Matrix
***********************************************************************/
SmStatus SmTransform::ConcatTransform              
 (const SmTransform & crInput) 
{ 
  // do the matrix multiply
  smgu_4x4Mult( m_adT, crInput.m_adT, m_adT ) ;
  
  // all done
  return(SM_SUCCESS) ; 

} // end SmTransform::ConcatTransform

/*******************************************************************//**
PURPOSE: Apply a mirror about a given plane to the this transform

NOTES: Plane is specifed in ProjSpace  
***********************************************************************/
SmStatus SmTransform::MirrorSimple
 (const SmPoint3d   & crPlaneProjPt,      // in : Point on plane
  const SmVector3d  & crPlaneProjNormal)  // in : plane's normal
{
  // unitize Plane Normal
  SmVector3d sN = crPlaneProjNormal ;
  SER(sN.Unitize()) ;

  // Build mirror transform            = [1-2NxNx  -2NxNy  -2NxNz  0]              
  //                                     [ -2NyNy 1-2NyNy  -2NyNz  0]
  //  with D = crPlanePt.Dot(pNormal) ;  [ -2NzNx  -2NzNy 1-2NzNz  0]
  //                                     [  2D*Nx   2D*Ny   2D*Nz  1]
  double      dD = crPlaneProjPt.Dot(sN) ;
  SmTransform sMirror ;
  sMirror.SetToXAxis(1-2*sN.x*sN.x,  -2*sN.x*sN.y,  -2*sN.x*sN.z) ;
  sMirror.SetToYAxis( -2*sN.y*sN.x, 1-2*sN.y*sN.y,  -2*sN.y*sN.z) ;
  sMirror.SetToZAxis( -2*sN.z*sN.x,  -2*sN.z*sN.y, 1-2*sN.z*sN.z) ;
  sMirror.SetToDisp (  2*dD*sN.x,     2*dD*sN.y,     2*dD*sN.z  ) ;

  // Mirror current transform
  this->ConcatTransform(sMirror) ;

  // all done
  return(SM_SUCCESS) ;

} // end SmTransform::MirrorSimple

/*******************************************************************//**
PURPOSE: Apply a XYZ scaling to the this transform

NOTES:  
***********************************************************************/
SmStatus SmTransform::ScaleSimple
  (const SmVector3d & crScaleProjVec,    // in : ScaleVec coordinates define xyz scaling factors.
   const SmPoint3d  * cpOptProjCenter)   // in : Sole point at which the scaled location equals the input location
                                         //      NULL=(0,0,0), default:[NULL]
{
  // Scale ToAxis vectors
  SmVector3d &rToX = GetToXAxis() ;
  SmVector3d &rToY = GetToYAxis() ;
  SmVector3d &rToZ = GetToZAxis() ;
  SmVector3d &rToD = GetToDisp() ;

  // build  TNew = Translate(-cpOptCenter) ;
  //               Scale(scScaleVec) ;
  //               Translate( cpOptCenter) ;
  //        TNew = [ 1  0  0  0] [S1  0  0  0] [ 1  0  0  0]
  //               [ 0  2  0  0] [ 0 S2  0  0] [ 0  2  0  0]
  //               [ 0  0  1  0] [ 0  0 S1  0] [ 0  0  1  0]
  //               [-X -Y -Z  1] [ 0  0  0  1] [ X  Y  Z  1]

  rToX.x *= crScaleProjVec.x ;
  rToY.x *= crScaleProjVec.x ;
  rToZ.x *= crScaleProjVec.x ;

  rToX.y *= crScaleProjVec.y ;
  rToY.y *= crScaleProjVec.y ;
  rToZ.y *= crScaleProjVec.y ;
               
  rToX.z *= crScaleProjVec.z ;
  rToY.z *= crScaleProjVec.z ;
  rToZ.z *= crScaleProjVec.z ;

  // order dependent - These lines come after the above rotation change
  if(cpOptProjCenter)
    {
      rToD.x += -rToX.x * cpOptProjCenter->x - rToY.x * cpOptProjCenter->y - rToZ.x * cpOptProjCenter->z ;
      rToD.y += -rToX.y * cpOptProjCenter->x - rToY.y * cpOptProjCenter->y - rToZ.y * cpOptProjCenter->z ;
      rToD.z += -rToX.z * cpOptProjCenter->x - rToY.z * cpOptProjCenter->y - rToZ.z * cpOptProjCenter->z ;
    }
                                             
  // all done
  return(SM_SUCCESS) ;

} // end SmTransform::ScaleSimple

/*******************************************************************//**
PURPOSE: Apply a translation vector to the this Transform.

NOTES: 
***********************************************************************/
SmStatus SmTransform::TranslateSimple
  (const SmVector3d & crProjTranslation)
{
  // add crTranslation to this Disp vector
  SmVector3d &rToDisp  = GetToDisp() ;
  rToDisp             += crProjTranslation ;

  // all done
  return(SM_SUCCESS) ;

} // end SmTransform::TranslateSimple

/*******************************************************************//**
PURPOSE: Apply a rotation about a given axis running through the origin
         to this transform  

NOTES: The rotation
     angle is defined as the counter-clockwise angle about the axis.  
     This is following the right-hand rule .
***********************************************************************/
SmStatus SmTransform::RotateAboutAxisSimple
 (double             dAngRad,        // in : rotation angle (radians)
  const SmVector3d & crRotateAxis)   // in : rotation axis 
{
  SmVector3d sA    = crRotateAxis ;
  SmStatus   eStat = sA.Unitize() ; 
  if(eStat != SM_SUCCESS)
    {
      SE_MSG(SM_ERR, _T("SmTransform::RotateAboutAxis given a zero length rotation axis")) ;
      // do the best we can
      sA.Set(1,0,0) ; 
    }

  // locals
  double u = sA.x ;
  double v = sA.y ;
  double w = sA.z ;

  double u2 = u*u ;
  double v2 = v*v ;
  double w2 = w*w ;

  double cosT = smos_Cosine(dAngRad);
  double sinT = smos_Sine(dAngRad);
  double oneMinusCosT = 1 - cosT;
  double adT[4][4] ;

  // Build the matrix entries element by element.
  adT[0][0] = (u2 + (v2 + w2) * cosT) ;
  adT[1][0] = (u*v * oneMinusCosT - w*sinT) ;
  adT[2][0] = (u*w * oneMinusCosT + v*sinT) ;
  adT[3][0] = 0 ;                           
                                            
  adT[0][1] = (u*v * oneMinusCosT + w*sinT) ;
  adT[1][1] = (v2 + (u2 + w2) * cosT) ;      
  adT[2][1] = (v*w * oneMinusCosT - u*sinT) ;
  adT[3][1] = 0 ;                           
                                            
  adT[0][2] = (u*w * oneMinusCosT - v*sinT) ;
  adT[1][2] = (v*w * oneMinusCosT + u*sinT) ;
  adT[2][2] = (w2 + (u2 + v2) * cosT) ;
  adT[3][2] = 0 ;  

  adT[0][3] = 0 ;
  adT[1][3] = 0 ;
  adT[2][3] = 0 ;
  adT[3][3] = 1 ;

  // create pure rotation transform from rotated X and Y axes 
  SmTransform sOther(NULL, adT) ;

  // apply transform to this coordinate system
  // this = this * sOther 
  this->ConcatTransform(sOther) ;

  // all done
  return(SM_SUCCESS) ;

} // end SmTransform::RotateAboutAxisSimple

/*******************************************************************//**
PURPOSE: Apply a rotation about an axis running through a given point
         to this transform 

NOTES:  
***********************************************************************/
SmStatus SmTransform::RotateAboutAxisAtPointSimple
  (double             dAngRad,        // in : rotation angle radians
   const SmPoint3d  & crRotateOrigin, // in : ProjSpace Point on rotation axis
   const SmVector3d & crRotateAxis)   // in : ProjSpace direction of rotation axis
{
  SmVector3d sA    = crRotateAxis ;
  SmStatus   eStat = sA.Unitize() ; 
  if(eStat != SM_SUCCESS)
    {
      SE_MSG(SM_ERR, _T("SmTransform::RotateAboutAxis given a zero length rotation axis")) ;
      // do the best we can
      sA.Set(1,0,0) ; 
    }

  // locals
  double a = crRotateOrigin.x ;
  double b = crRotateOrigin.y ;
  double c = crRotateOrigin.z ;
  double u = sA.x ;
  double v = sA.y ;
  double w = sA.z ;

  double u2 = u*u ;
  double v2 = v*v ;
  double w2 = w*w ;

  double cosT = smos_Cosine(dAngRad);
  double sinT = smos_Sine(dAngRad);
  double oneMinusCosT = 1 - cosT;
  double adT[4][4] ;

  // Build the matrix entries element by element.
  adT[0][0] = (u2 + (v2 + w2) * cosT) ;
  adT[1][0] = (u*v * oneMinusCosT - w*sinT) ;
  adT[2][0] = (u*w * oneMinusCosT + v*sinT) ;
  adT[3][0] = ((a*(v2 + w2) - u*(b*v + c*w)) * oneMinusCosT + (b*w - c*v)*sinT) ;
  
  adT[0][1] = (u*v * oneMinusCosT + w*sinT) ;
  adT[1][1] = (v2 + (u2 + w2) * cosT) ;
  adT[2][1] = (v*w * oneMinusCosT - u*sinT) ;
  adT[3][1] = ((b*(u2 + w2) - v*(a*u + c*w)) * oneMinusCosT + (c*u - a*w)*sinT) ;
  
  adT[0][2] = (u*w * oneMinusCosT - v*sinT) ;
  adT[1][2] = (v*w * oneMinusCosT + u*sinT) ;
  adT[2][2] = (w2 + (u2 + v2) * cosT) ;
  adT[3][2] = ((c*(u2 + v2) - w*(a*u + b*v)) * oneMinusCosT + (a*v - b*u)*sinT) ;   // minus differenc in paper (b*u - a*v)  yuk!

  adT[0][3] = 0 ;
  adT[1][3] = 0 ;
  adT[2][3] = 0 ;
  adT[3][3] = 1 ;

  // create pure rotation transform from rotated X and Y axes 
  SmTransform sOther(NULL, adT) ;

  // apply transform to this coordinate system
  // this = this * sOther 
  this->ConcatTransform(sOther) ;

  // all done
  return(SM_SUCCESS) ;

} // end SmTransform::RotateAboutAxisAtPointSimple

/*******************************************************************//**
PURPOSE: Add a RotateNMove and then an optional Scale transform 
         to the existing transform sequence

NOTES: 
***********************************************************************/
SmStatus SmTransform::TransformSimple
  (const SmAxis2Placement & crRotateNMove,   // in : 1st transform added to end of current transform sequence
   const SmVector3d       * cpOptScale)      // out: 2nd transform added to end of current transform sequence
{
  SmTransform sRotateNMove ;
  sRotateNMove.SetAxis2Placement(crRotateNMove) ;

  // add rotate and move to transform
  this->ConcatTransform(sRotateNMove) ;

  // scale transform
  if(cpOptScale)
    {
      this->ScaleSimple(*cpOptScale) ;
    }

  // all done
  return(SM_SUCCESS) ;

} // end SmTransform::TransformSimple

/*******************************************************************//**
PURPOSE: Apply a project to given plane to the this transform

NOTES:  
***********************************************************************/
SmStatus SmTransform::ProjectToPlaneSimple
 (const SmPoint3d   & crPlaneProjPt,      // in : ProjSpace Point on plane
  const SmVector3d  & crPlaneProjNormal)  // in : ProjSpace plane's normal
{
  // unitize Plane Normal
  SmVector3d sN = crPlaneProjNormal ;
  SER(sN.Unitize()) ;

  // Build project transform           = [1-NxNx  -NxNy  -NxNz  0]              
  //                                     [ -NyNy 1-NyNy  -NyNz  0]
  //  with D = crPlanePt.Dot(pNormal) ;  [ -NzNx  -NzNy 1-NzNz  0]
  //                                     [  D*Nx   D*Ny   D*Nz  1]
  double      dD = crPlaneProjPt.Dot(sN) ;
  SmTransform sProject ;
  sProject.SetToXAxis(1-sN.x*sN.x,  -sN.x*sN.y,  -sN.x*sN.z) ;
  sProject.SetToYAxis( -sN.y*sN.x, 1-sN.y*sN.y,  -sN.y*sN.z) ;
  sProject.SetToZAxis( -sN.z*sN.x,  -sN.z*sN.y, 1-sN.z*sN.z) ;
  sProject.SetToDisp (  dD*sN.x,     dD*sN.y,     dD*sN.z  ) ;

  // Mirror current transform
  this->ConcatTransform(sProject) ;

  // all done
  return(SM_SUCCESS) ;

} // end SmTransform::ProjectToPlaneSimple

/*******************************************************************//**
PURPOSE: Find the axis placement which is the inversion of the given
    placement.  

NOTES: The resulting placement takes points from the global
    coordinate system and transforms them into the local coordinate 
    system of the original placement.
***********************************************************************/
SmStatus SmTransform::InvertSimple() 
{
  // invert the 4x4 transform matrix
  return( smgu_4x4Inverse(m_adT, m_adT) ) ;

} // end SmTransform::InvertSimple

/*******************************************************************//**
PURPOSE: return TUE when axes subMatrix of the transform matrix forms
 a right-handed, unit, orthonormal coordinate system  

NOTES: The resulting placement takes points from the global
    coordinate system and transforms them into the local coordinate 
    system of the original placement.
***********************************************************************/
SmBoolean SmTransform::IsUnitOrthonormal() const
{
  // Determinate must be 1
  double dDet = smgu_4x4Determinate(m_adT) ;

  if(!SM_IS_ZERO(dDet - 1.0))
    { return( FALSE ) ; }

  // Each axis length must be one - need only to check two
  SmVector3d &rX = GetToXAxis() ;
  double      dXSizeSq = rX.Dot(rX) ;
  if(!SM_IS_ZERO( dXSizeSq - 1.0))  
    { return( FALSE ) ; }

  SmVector3d &rY = GetToYAxis() ;
  double      dYSizeSq = rY.Dot(rY) ;
  if(!SM_IS_ZERO( dYSizeSq - 1.0))  
    { return( FALSE ) ; }

  // arrive here for right handed ortho normal ToAxes sets
  return(TRUE) ; 

} // end SmTransform::IsUnitOrthonormal

/*******************************************************************//**
PURPOSE: Load a SmAxis2Placment object from a SmTransform when possible 

NOTES:  returns SM_ERR when axes sub matrix is not right-handed, unit, ortho-normal
***********************************************************************/
SmStatus SmTransform::LoadAxis2Placement
  (SmAxis2Placement &rAxis2Placement)  // out: object made equivalent to this when possible
 const
{
  // init output
  rAxis2Placement.Init() ;

  // check for right-handed coordinate system
  SmBoolean bIsOrthoNormal = IsUnitOrthonormal() ;
  if(!bIsOrthoNormal)
    { return(SM_ERR) ; }

  // locals
  SmVector3d & rX    = GetToXAxis() ;
  SmVector3d & rY    = GetToYAxis() ;
  SmVector3d & rOrig = GetToDisp() ;

  // set output
  rAxis2Placement.SetCanonical(rOrig, rX, rY) ; 

  // all done
  return(SM_SUCCESS) ;

} // end SmTransform::LoadAxis2Placement

/*******************************************************************//**
PURPOSE: set SmTransform from a SmAxis2Placment object 

NOTES:  
***********************************************************************/
void SmTransform::SetAxis2Placement
  (const SmAxis2Placement &rAxis2Placement)  // in : object to copy from
{
  // locals
  SmVector3d sZ = rAxis2Placement.GetZAxis() ;

  // set the transform
   SetToXAxis( rAxis2Placement.GetXAxisRef() ) ;
   SetToYAxis( rAxis2Placement.GetYAxisRef() ) ;
   SetToZAxis( sZ ) ;
   SetToDisp ( rAxis2Placement.GetOriginRef() ) ;
   InitExt   ( ) ;

} // end SmTransform::SetAxis2Placement

/*******************************************************************//**
PURPOSE: transpose the tranformation matrix

NOTES: the NLib and OpenGL tranformation matrices are the transpose of the SMLib matrix
       Use this method prior to passing the transformation matrix
       to NLib. 
***********************************************************************/
void SmTransform::Transpose()
{
  double dTmp ;

  // transpose the off diagonal elements

  // 0th col/rows 1, 2, 3
  dTmp        = m_adT[1][0] ;
  m_adT[1][0] = m_adT[0][1] ;
  m_adT[0][1] = dTmp ;

  dTmp        = m_adT[2][0] ;
  m_adT[2][0] = m_adT[0][2] ;
  m_adT[0][2] = dTmp ;

  dTmp        = m_adT[3][0] ;
  m_adT[3][0] = m_adT[0][3] ;
  m_adT[0][3] = dTmp ;

  // 1st col/row rows 2, 3
  dTmp        = m_adT[2][1] ;
  m_adT[2][1] = m_adT[1][2] ;
  m_adT[1][2] = dTmp ;

  dTmp        = m_adT[3][1] ;
  m_adT[3][1] = m_adT[1][3] ;
  m_adT[1][3] = dTmp ;

  // 2nd col/row 3
  dTmp        = m_adT[3][2] ;
  m_adT[3][2] = m_adT[2][3] ;
  m_adT[2][3] = dTmp ;

} // end SmTransform::Transpose

/*******************************************************************//**
PURPOSE: Load a NLib 4x4 tranform matrix object from a SmTransform

NOTES: 1. NLib transform is transpose of SMLib SmTransform matrix.
       2. NLib mat is double ** while SMLib mat is double[4][4] ;
***********************************************************************/
SmStatus  SmTransform::LoadNLibTransform
  (double **adNLibMat) 
 const 
{
  adNLibMat[0][0] = m_adT[0][0] ;
  adNLibMat[0][1] = m_adT[1][0] ;
  adNLibMat[0][2] = m_adT[2][0] ;
  adNLibMat[0][3] = m_adT[3][0] ;
                              
  adNLibMat[1][0] = m_adT[0][1] ;
  adNLibMat[1][1] = m_adT[1][1] ;
  adNLibMat[1][2] = m_adT[2][1] ;
  adNLibMat[1][3] = m_adT[3][1] ;
                              
  adNLibMat[2][0] = m_adT[0][2] ;
  adNLibMat[2][1] = m_adT[1][2] ;
  adNLibMat[2][2] = m_adT[2][2] ;
  adNLibMat[2][3] = m_adT[3][2] ;
                              
  adNLibMat[3][0] = m_adT[0][3] ;
  adNLibMat[3][1] = m_adT[1][3] ;
  adNLibMat[3][2] = m_adT[2][3] ;
  adNLibMat[3][3] = m_adT[3][3] ;

  // all done
  return(SM_SUCCESS) ;

} // end SmTransform::LoadNLibTransform

/*******************************************************************//**
PURPOSE: Load SmTransform tranform matrix from a NLib 4x4 tranform matrix

NOTES: 1. NLib transform is transpose of SMLib SmTransform matrix.
       2. NLib mat is double ** while SMLib mat is double[4][4] ;
***********************************************************************/
void SmTransform::SetNLibTransform (double **adNLibMat) 
{
  m_adT[0][0] = adNLibMat[0][0] ;
  m_adT[0][1] = adNLibMat[1][0] ;
  m_adT[0][2] = adNLibMat[2][0] ;
  m_adT[0][3] = adNLibMat[3][0] ;
                              
  m_adT[1][0] = adNLibMat[0][1] ;
  m_adT[1][1] = adNLibMat[1][1] ;
  m_adT[1][2] = adNLibMat[2][1] ;
  m_adT[1][3] = adNLibMat[3][1] ;
                              
  m_adT[2][0] = adNLibMat[0][2] ;
  m_adT[2][1] = adNLibMat[1][2] ;
  m_adT[2][2] = adNLibMat[2][2] ;
  m_adT[2][3] = adNLibMat[3][2] ;
                              
  m_adT[3][0] = adNLibMat[0][3] ;
  m_adT[3][1] = adNLibMat[1][3] ;
  m_adT[3][2] = adNLibMat[2][3] ;
  m_adT[3][3] = adNLibMat[3][3] ;

} // end SmTransform::SetNLibTransform

/*******************************************************************//**
PURPOSE: Decompose the transform implied by the vectors of the 
    placement into a set of sequencial rotation angles.  

NOTES: returns SM_ERR when axes sumMatrix is not right-handed unit orthoNormal
               after setting the output angles all to zero.
***********************************************************************/
SmStatus SmTransform::DecomposeToAngles
  (double & rdXRotRad,      // out: 1st rotation about X Axis 
   double & rdYRotRad,      // out: 2nd rotation about Y Axis 
   double & rdZRotRad)      // out: 3rd rotation about Z Axis 
 const
{
  // init output
   rdXRotRad  = 0.0 ;
   rdYRotRad  = 0.0 ;
   rdZRotRad  = 0.0 ;

  // return SM_ERR when axes subMatrix is not a right-handed, unit, coordinate system
  SmBoolean bIsCoords = IsUnitOrthonormal() ;
  if(!bIsCoords)
    { return(SM_ERR) ; }

  // locals
  const SmVector3d &rX = GetToXAxis() ;
  const SmVector3d &rY = GetToYAxis() ;
  const SmVector3d &rZ = GetToZAxis() ;

  rdYRotRad = smos_ArcSine(-rX.z);

  if ( fabs(smos_Cosine(rdYRotRad)) > SM_EFF_ZERO) 
    {
      rdXRotRad = smos_ArcTangent2(rY.z, rZ.z);
      rdZRotRad = smos_ArcTangent2(rX.y, rX.x);
    } 
  else 
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
      SmTransform sTestA2P;
      SmPoint3d sOrigin = this->GetToDisp();
      sTestA2P.ComposeFromAngles(rdXRotRad,rdYRotRad,rdZRotRad,&sOrigin);
      this->Dump();
      sTestA2P.Dump();
    }

#endif

  // all done
  return(SM_SUCCESS) ;

} // end SmTransform::DecomposeToAngles

/*******************************************************************//**
PURPOSE: Compose the transform from the set of sequencial rotation angles.  

NOTES:  Returned SmTransform is centered on the origin
***********************************************************************/
void SmTransform::ComposeFromAngles
  (double      dXRotRad,     // in : 1st rotation about X Axis 
   double      dYRotRad,     // in : 2nd rotation about Y Axis
   double      dZRotRad,     // in : 3rd rotation about Z Axis
   SmPoint3d * pOptCenter)   // in : Center, NULL=leave at origin
                             //      default:[NULL]
{
#ifdef SM_DEBUG_CODE
  SmTransform sOrigA2P(*this) ;
#endif

  // init to x      = [1,0,0]
  //         y      = [0,1,0]
  //         origin = [0,0,0]
  Init() ;

  // rotate about global X by wAngleX
  RotateAboutAxisSimple(dXRotRad, SmVector3d(1, 0, 0));

  // rotate about global Y by wAngleY
  RotateAboutAxisSimple(dYRotRad, SmVector3d(0, 1, 0));

  // rotate about global Z by wAngleZ
  RotateAboutAxisSimple(dZRotRad, SmVector3d(0, 0, 1));

  // set transform origin
  if(pOptCenter) 
    {
      TranslateSimple(*pOptCenter) ;
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
      sTestOrigin = GetToDisp() ;

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

} // end SmTransform::ComposeFromAngles

/*******************************************************************//**
PURPOSE: convenience function to look like a SmBSplineVolume object that
         gets effective knot lists

NOTES: for cache purposes - let BendVolume mascarade as a BSpline kind of shape. 
***********************************************************************/
SmStatus SmTransform::GetKnots
  (SmVolumeParamType  eVolumeParam,  // NotUsed: in : one of SM_VP_U, SM_VP_V, SM_VP_W                              
   SmTArray<double> & rKnots,        // out: ParamSpace knot list                                          
   SmTArray<ULONG>  * pKnotMults,    // out: associated knot multipliticies. NULL to ignore. default:[NULL]
   const SmExtent1d * pOptIvl)       // NotUsed: in : interval of interest, NULL=Natural Interval, default:[NULL]   
  const     
{ 
  SM_REF2(eVolumeParam, pOptIvl) ;
  
  // init output
  rKnots.ReSet() ;
  if(pKnotMults) { pKnotMults->ReSet() ; } 

  // knots

  // -SM_INFINITE_PARAMETER
  if(pOptIvl == NULL || pOptIvl->ContainsValue(-SM_INFINITE_PARAMETER, SM_EFF_ZERO)) 
    { rKnots.Add(-SM_INFINITE_PARAMETER) ;
      if(pKnotMults) { pKnotMults->Add(1) ; }
    }

  // SM_INFINITE_PARAMETER
  if(pOptIvl == NULL || pOptIvl->ContainsValue(SM_INFINITE_PARAMETER, SM_EFF_ZERO)) 
    { rKnots.Add( SM_INFINITE_PARAMETER) ;
      if(pKnotMults) { pKnotMults->Add(1) ; }
    }

  // all done
  return(SM_SUCCESS) ; 

} // end SmTransform::GetKnots

/*******************************************************************//**
PURPOSE: When possible build the exact Curve produced by projecting
    rInputCurve to 1st OutSpace

NOTES: when eInputSpace == SM_VS_PARAM_SPACE: rInputCurve is mapped from ParamSpace to 1stOutSpace
       else eInputSpace == SM_VS_INSPACE    : rInputCurve is mapped from InSpace to 1stOutSpace
       
  the Transform 'maps' SmCurves derived from SmBSplineCurve when
  transform can be represented by SmAxis2Placement objects.  Otherwise
  it assumes the curve cannot by mapped exactly and sets rpNewCurve to NULL
  and returns SM_SUCCESS. 

  It is assumed that the SmTransform->m_pOrientMap == NULL and the mapping
  from ParamSpace or InputSpace to 1stOutputSpace is the same.

  (To fix this method if the OrientMap==NULL assumption is false, 
   Add one more mapping of the InputCurve through m_pOrient to this method.)
***********************************************************************/
SmStatus SmTransform::MakeExactBSpline1stOutCurve
 (SmVolumeSpaceTYPE    eInputSpace,   // in : one of SM_VS_PARAM_SPACE = InputCurve is mapped from ParamSpace
                                      //             SM_VS_IN_SPACE    = InputCurve is mapped from InSpace
  const SmCurve      & rInputCurve,   // in : Target InSpace Curve
  SmBSplineCurve    *& rpNewCurve)    // out: NonNULL when exact surface is built, else NULL
 const 
{
  SM_REF1(eInputSpace) ;

  // init output
  rpNewCurve = NULL ;

  // check input assumption
  SM_ASSERT_MSG(eInputSpace == SM_VS_IN_SPACE || GetOrientMap() == NULL,
                _T("SmTransform::MakeExactBSpline1stOutCurve() assumption that m_pOrient is NULL is FALSE - function needs extension")) ;
  //   (To fix this method if the OrientMap==NULL assumption is false, 
  //    Add one more mapping of the InputCurve through m_pOrient to this method.)

  // for geometry derived from SmBSpline
  if(rInputCurve.IsKindOf(SmBSplineCurve_TYPE))
    {
      // for Transforms which can be represented by SmAxis2Placement mappings (i.e. no shear and non-orthogonal basis vectors)
      SmAxis2Placement sAxis2Placement ;
      if(SM_SUCCESS == LoadAxis2Placement(sAxis2Placement))
        {
          SmCurve         * pThisNewCurve = NULL ;
          const SmContext * pContext = rInputCurve.GetContext() ;

          // copy and transform the curve
          rInputCurve.Copy(*pContext, pThisNewCurve) ;
          pThisNewCurve->Transform(sAxis2Placement) ;

          // set output
          rpNewCurve = (SmBSplineCurve *)pThisNewCurve ;

        } // end Transform is a simple Affine transformation check
    } // end InSpaceCurve is derived from SmBSpline check

  // all done
  return SM_SUCCESS ; 

} // end MakeExactBSpline1stOutCurve

/*******************************************************************//**
PURPOSE: When possible build the exact Surface produced by projecting
    rInnputSurface to 1st OutSpace

NOTES: the Transform 'maps' SmSurfaces derived from SmBSplineSurface when
  transform can be represented by SmAxis2Placement objects.  Otherwise
  it assumes the surface cannot by mapped exactly and sets rpNewSurface to NULL
  and returns SM_SUCCESS. 

  It is assumed that the SmTransform->m_pOrientMap == NULL and the mapping
  from ParamSpace or InputSpace to 1stOutputSpace is the same.

  (To fix this method if the OrientMap==NULL assumption is false, 
   Add one more mapping of the InputCurve through m_pOrient to this method.)
***********************************************************************/
SmStatus SmTransform::MakeExactBSpline1stOutSurface
 (SmVolumeSpaceTYPE   eInputSpace,   // in : SM_VS_IN_SPACE   = InputSurface is projected from InSpace
                                     //      SM_VS_PARAM_SPACE= InputSurface is projected from ParamSpace
  const SmSurface   & rInputSurface, // in : Surface to project to Last OutSpace
  SmBSplineSurface *& rpNewSurface)  // out: 1st OutSpace projection or NULL for not possible
 const 
{
  SM_REF1(eInputSpace) ;

  // init output
  rpNewSurface = NULL ;

  // check input assumption
  SM_ASSERT_MSG(eInputSpace == SM_VS_IN_SPACE || GetOrientMap() == NULL,
                _T("SmTransform::MakeExactBSpline1stOutCurve() assumption that m_pOrient is NULL is FALSE - function needs extension")) ;
  //   (To fix this method if the OrientMap==NULL assumption is false, 
  //    Add one more mapping of the InputCurve through m_pOrient to this method.)

  // for geometry derived from SmBSpline
  if(rInputSurface.IsKindOf(SmBSplineSurface_TYPE))
    {
      // for Transforms which can be represented by SmAxis2Placement mappings (i.e. no shear and non-orthogonal basis vectors)
      SmAxis2Placement sAxis2Placement ;
      if(SM_SUCCESS == LoadAxis2Placement(sAxis2Placement))
        {
          SmSurface       * pThisNewSurface = NULL ;
          const SmContext * pContext        = rInputSurface.GetContext() ;

          // copy and transform the surface
          rInputSurface.Copy(*pContext, pThisNewSurface) ;
          pThisNewSurface->Transform(sAxis2Placement) ;

          // set output
          rpNewSurface = (SmBSplineSurface *)pThisNewSurface ;

        } // end Transform is a simple Affine transformation check
    } // end InSpaceSurface is derived from SmBSpline check

  // all done
  return SM_SUCCESS ; 

} // end MakeExactBSpline1stOutSurface

/*******************************************************************//**
PURPOSE: Map Point from InSpace to Outspace with derivatives without compounding

NOTES: 
***********************************************************************/
SmStatus SmTransform::EvaluateSimple
 (const SmPoint3d & crParamPoint,         // in : ParamSpace point to map to ProjSpace point
  ULONG             lHighestDeriv,        // in : 0-Pos Only, 1=Pos+1st Derivs, ..., max 3
  SmBoolean         bUFromLeft,           // NotUsed: in : if P is on U, V, or w interval boundary
  SmBoolean         bVFromLeft,           // NotUsed: in : TRUE  = evaluate P in upper interval where P is on the left of the interval
  SmBoolean         bWFromLeft,           // NotUsed: in : FALSE = evaluate P in lower interval where P is on the right of the interval
  SmVector3d      * aDerivatives,         // out: matrix of ParamSpace evaluations values
                                          //      3d organized: D[u][v][w]
                                          //      1d organized: D[i], i = u*n*n+v*n+w, for lHighestDeriv from 0 to 3
                                          //      sized       : [n+1][n+1][n+1], where n=lHighesDeriv
                                          //      lHghDrv = 0,   sized: [1],      
                                          //        i=0          order: [D]
                                          //      lHghDrv = 1,   sized: [8]    
                                          //        i=u*4+v*2+w  order: [D  Dw  Dv  ---
                                          //                             Du --- --- ---]
                                          //      lHghDrv = 2,   sized: [27]   
                                          //        i=u*9+v*3+w  order: [D   Dw  Dww Dv  Dvw --- Dvv --- ---
                                          //                             Du  Duw --- Duv --- --- --- --- ---
                                          //                             Duu --- --- --- --- --- --- --- ---]
                                          //      lHghDrv = 3,   sized: [81]   
                                          //        i=u*16+v*4+w order: [D   Dw   Dww   Dwww  Dv   Dvw  Dvww ---  Dvv  Dvw
                                          //                             --- ---  Dvvv  ---   ---  ---  Du   Duw  Duww ---
                                          //                             Duv Duvw ---   ---   Duvv ---  ---  ---  ---  ---
                                          //                             --- ---  Duu   Duuw  ---  ---  Duuv ---  ---  --- 
                                          //                             --- ---  ---   ---   ---  ---  ---  ---  Duuu --- 
                                          //                             --- ---  ---   ---   ---  ---  ---  ---  ---- --- 
                                          //                             --- ---  ---   ---   ---  ---  ---  ---  ---- --- 
                                          //                             --- ---  ---   ---   ---  ---  ---  ---  ---- --- 
                                          //                             --- --- ]
  SmBoolean bNonZeroTangents,             // NotUsed: in : TRUE = replace zero tangent vectors with properly oriented tol sized vectors
                                          //      FALSE= return exact tangent values, default:[TRUE]
                                          //      note: Surprisingly TRUE is the common choice because most tangent uses
                                          //            are for their direction (Binorm, SurfNorm comps), but when the 
                                          //            tangent is being used for its magnitude (like an arc-length comp)
                                          //            then set this to FALSE.
                                          //      default:[TRUE]               
  SmBoolean bDoZeroSampling)              // NotUsed: in : for internal use only, always set to TRUE, default:[TRUE]
const 
{
  // stop compile warnings - reference unused arguments)
  SM_REF5(bUFromLeft, bVFromLeft, bWFromLeft, bNonZeroTangents, bDoZeroSampling) ;

  // check input
  if (lHighestDeriv > GW_MAX_DERIV) SER(SM_ERR_INVALID_INPUT);

  // init output array - leave aDerivatives[0] alone, needed so that &crParamPoint can be same memory as aDerivatives
  ULONG ii, dASize = (lHighestDeriv+1) * (lHighestDeriv+1) * (lHighestDeriv+1) ;
  for(ii=1;ii<dASize;ii++)
    {
      aDerivatives[ii].Set(0,0,0) ;
    }

  // For indexing into the volume pt/deriv array:
#define sv(u,v,w) ((u*(lHighestDeriv+1)+v)*(lHighestDeriv+1)+w)

  // locals
  SmVector3d &rX = GetToXAxis() ;
  SmVector3d &rY = GetToYAxis() ;
  SmVector3d &rZ = GetToZAxis() ;
  SmVector3d &rD = GetToDisp() ;

  // position
  aDerivatives[0] = rD + crParamPoint.x * rX 
                       + crParamPoint.y * rY 
                       + crParamPoint.z * rZ ; 

  // 1st derivs
  if(lHighestDeriv >= 1)
    {
     
      aDerivatives[sv(1,0,0)] = rX ;     // Du
      aDerivatives[sv(0,1,0)] = rY ;     // Dv
      aDerivatives[sv(0,0,1)] = rZ ;     // Dw
    }

  // all higher derivs are zero

  // all done
  return(SM_SUCCESS) ;

#undef sv

} // end SmTransform::EvaluateSimple

/*******************************************************************//**
PURPOSE: Compute the ProjSpace Bounding and/or Pseudo Boxes that 
   contain the projection of a given BBox from ParamSpace to ProjSpace.

NOTES: 
    One or more of the outputs must be non-NULL.
    Bounding boxes are built for whole volumes not subdomains.

  pProjPseudoBox orientation
    Default pProjPseudoBox orientation: ProjX = EvaluateSimple(ParamX, CenterOf crParamBox)
                                        ProjY = EvaluateSimple(ParamY, CenterOf crParamBox)
                                        ProjZ = EvaluateSimple(ParamZ, CenterOf crParamBox)
    When given pOptParamPseudoBox     : ProjX = EvaluateSimple(pOptParamPseudoBox.GetToXAxis, CenterOf crParamBox)
                                        ProjY = EvaluateSimple(pOptParamPseudoBox.GetToYAxis, CenterOf crParamBox)
                                        ProjZ = EvaluateSimple(pOptParamPseudoBox.GetToZAxis, CenterOf crParamBox)
    In words: Without pOptParamPseudoBox, the output pProjPseudoBox orientation
    is found by projecting a set of axisAligned vectors through this volume's map
    at the center of the given CrParamBox.  When pOptParamPseudoBox is given
    the output pProjPseudoBox orientation is found by projecting the pProjPseudoBox
    basis vectors at the center of the given crParamBox.  

***********************************************************************/
SmStatus SmTransform::EvaluateBoundingBoxSimple
  (const SmExtent3d & crParamBox,               // in : ParamSpace BBox to project to Project Space 
   SmPseudoBox      * pOptParamPseudoBox,       // in : optional ParamSpace PseudoBox used to set output PseudoBox orientations,
                                                //      NULL   : ProjPseudoBox Basis = Project X Y Z ParamVecs to ProjSpace at crParamBox Center
                                                //      NotNULL: ProjPseudoBox Basis = Project pOptParamPseudoBox BasisVecs to ProjSpace at crParamBox Center
                                                //      default:[NULL]
   SmExtent3d       * pProjBox,                 // out: ProjectSpace Axis aligned box
   SmPseudoBox      * pProjPseudoBox)           // out: ProjectSpace Non-axis aligned box
  const
{
  // no work - no output
  if(   pProjBox == NULL
     && pProjPseudoBox == NULL)
    { return SM_SUCCESS ; }

  // when input ParamBox is same as output ProjBox - copy input ParamBox
  SmExtent3d        sBox ;
  const SmExtent3d *pBox ;
  if(&crParamBox == pProjBox) {  sBox = crParamBox ;
                                 pBox = &sBox ;
                              }
  else                        {  pBox = &crParamBox ;
                              }

  // when input ParamPseudoBox is same as output ProjPseudoBox - copy input ParamPseudoBox
  SmPseudoBox sPseudoBox;
  if(   pProjPseudoBox
     && pOptParamPseudoBox == pProjPseudoBox) { sPseudoBox = *pOptParamPseudoBox ; }

  // indirection: from here on out - pBox       == ParamSpace Box
  //                               - pPseudoBox == ParamSpace PseudoBox

  // get Transform's InSpacePseudoBox
  SmPseudoBox sThisProjPseudoBox, sThisParamPseudoBox ;
  long        alAxisMap[3] ; 
  SmBoolean   bAreAligned ; 

  // init output
  if(pProjBox)          { pProjBox->Init() ; }  // init ProjBox with uninitialized intervals
  if(   pProjBox
     || pProjPseudoBox) { // init sThisProjPseudoBox with ParamSpace basis vectors
                          if(pOptParamPseudoBox) // align sThisProjPseudoBox with pOptParamPseudoBox Bases
                            { sThisProjPseudoBox = *pOptParamPseudoBox ; }
                          // else sThisProjPseudoBox set with Param X Y Z basis vectors

                          // project sThisProjPseudoBox current ParamSpace Basis vectors to ProjSpace vecs
                          SmVector3d sX, sY, sZ ;
                          TransformVector(sThisProjPseudoBox.GetBasis(0), sX) ;
                          TransformVector(sThisProjPseudoBox.GetBasis(1), sY) ;
                          TransformVector(sThisProjPseudoBox.GetBasis(2), sZ) ;

                          // init sThisProjPseudoBox to projected Basis vector directions and uninitialized intervals
                          sThisProjPseudoBox.SetBasis(sX, sY, sZ) ; 
                          sThisProjPseudoBox.Init() ; // init intervals - leave basis vectors alone
                        }

  // Map ParamBox into a PseudoBox in the transformed coordinate system
  // when Oriented ProjPseudoBox is aligned with the paramSpace basis vectors projected to ProjSpace
  bAreAligned = sThisParamPseudoBox.AreAligned( *pOptParamPseudoBox, alAxisMap );
  if(pOptParamPseudoBox == NULL || bAreAligned)
    {
      // get Ivls of mapping ParamBox directly to ProjSpace
      SmVector3d sO = GetToOrigin() ;

      SmVector3d sX = GetToXAxis() ; double sXLength = sX.Length() ;
      SmVector3d sY = GetToYAxis() ; double sYLength = sY.Length() ;
      SmVector3d sZ = GetToZAxis() ; double sZLength = sZ.Length() ;
      
      SmExtent1d sIvls[3] ;
      sIvls[0] = crParamBox.GetUInterval().Scale(1./sXLength).Translate(sO.Dot(sX)) ; 
      sIvls[1] = crParamBox.GetVInterval().Scale(1./sYLength).Translate(sO.Dot(sY)) ; 
      sIvls[2] = crParamBox.GetWInterval().Scale(1./sZLength).Translate(sO.Dot(sZ)) ;
      
      // when sThisProjPseudoBox bases need to be swapped and/or negated to match pProjPseudoBox basis orientations
      if(pOptParamPseudoBox && ( alAxisMap[0] != 1 || alAxisMap[1] != 2 || alAxisMap[2] !=3))
        { SmExtent1d sTmps[3] ;
          ULONG i0 = smos_Labs(alAxisMap[0])-1 ;
          ULONG i1 = smos_Labs(alAxisMap[1])-1 ;
          ULONG i2 = smos_Labs(alAxisMap[2])-1 ;

          sTmps[0] = alAxisMap[0] > 0 ? sIvls[i0] : sIvls[i0].Negate() ;
          sTmps[1] = alAxisMap[1] > 0 ? sIvls[i1] : sIvls[i1].Negate() ;
          sTmps[2] = alAxisMap[2] > 0 ? sIvls[i2] : sIvls[i2].Negate() ;

          sIvls[0] = sTmps[0] ;
          sIvls[1] = sTmps[1] ;
          sIvls[2] = sTmps[2] ;
        }

      // set sThisProjPseudoBox intervals
      sThisProjPseudoBox.SetIntervals(sIvls[0], sIvls[1], sIvls[2]) ;

      // set output
      if(pProjPseudoBox) { *pProjPseudoBox = sThisProjPseudoBox ; }
      if(pProjBox)       {  pProjBox->Circumscribe(sThisProjPseudoBox) ; }
    }
  else // given an unaligned pOptParamPseudoBox PseudoBox
    {
      // when pBox is unaligned and unbounded  
      if(!pBox->IsBounded())
        {
          // return unbounded BBoxes
          if(pProjBox)       { pProjBox->SetUnbounded() ; }
          if(pProjPseudoBox) { pProjPseudoBox->SetUnbounded() ; }

        } // end pBox pBox is unaligned and unbounded branch
      else // pBox is unaligned and bound 
        {
          if(pProjPseudoBox) { *pProjPseudoBox = sThisProjPseudoBox ; }

          // accumulate pBox->CornerPoint mappings into output BBoxes
          //  don't sample edges and faces since Transform is an affine transformation and convex polygons map to convex polygons
          ULONG ii, jj, kk ;
          for(ii=0;ii<2;ii++)
            {
              for(jj=0;jj<2;jj++)
                {
                  for(kk=0;kk<2;kk++)
                    {
                      SmPoint3d sUVW = pBox->Evaluate(ii, jj, kk) ;
                      SmPoint3d sXYZ ;
                      EvaluatePoint(sUVW, sXYZ) ;

                      // build the boxes
                      if(pProjBox)       { pProjBox->AddPoint3d(sXYZ) ; }
                      if(pProjPseudoBox) { pProjPseudoBox->AddPoint3d(sXYZ) ; }
                    }
                }
            }
        } // end pBox is bound and unaligned branch
    } // end pBox is unaligned and bound branch

  // all done
  return(SM_SUCCESS) ;

} // end SmTransform::EvaluateBoundingBoxSimple

/*******************************************************************//**
PURPOSE: Create ProjSpace IsoCurve from a ParamSpace IsoParamLine

NOTES: Specify isoline as pair of constant param values as:
   +-----------------+----------------+----------------+-------------------+
   | eConstantParams | dIsoParameter1 | dIsoParameter2 | Varying Parameter |
   +-----------------+----------------+----------------+-------------------+
   |    SM_VPS_UV    |   constant u   |   constant v   |     varying w     |
   |    SM_VPS_UW    |   constant u   |   constant w   |     varying v     |
   |    SM_VPS_VW    |   constant v   |   constant w   |     varying u     |
   +-----------------+----------------+----------------+-------------------+

***********************************************************************/
SmStatus SmTransform::EvaluateIsoParametricCurveSimple
 (const SmContext     & crContext,            // in : context for created objects
  SmVolumeParamsType    eConstantParams,      // in : oneof: SM_VPS_UV_IN,
                                              //             SM_VPS_UW_IN,
                                              //             SM_VPS_VW_IN.
  double                dIsoParam1,           // in : 1st constant SM_VP_U_IN or SM_VP_V_IN parameter value
  double                dIsoParam2,           // in : 2nd constant SM_VP_V_IN or SM_VP_W_IN parameter value
  double                d3DTolerance,         // NotUsed: in : Max ApproxCurve to IdealCurve deviation
  SmCurve            *& rpNewIsoCurve,        // out: the ProjSpace IsoCurve
  const SmExtent3d    * pOptParamTrimDomain)  // in : limiting domain, NULL to ignore. default:[NULL]  
 const
{
  SM_REF1(d3DTolerance) ;
  // init output
  rpNewIsoCurve = NULL ;
  
  // locals 
  SmPoint3d         sPuvw ;
  SmVector3d        sVuvw ;
  SmExtent1d        sFreeIvl ;
  SmExtent1d        sFixedIvl1 ;
  SmExtent1d        sFixedIvl2 ;
  SmExtent3d        sParamDomain = GetNaturalParamDomain() ;
  const SmExtent3d *pTgtDomain   = pOptParamTrimDomain ? pOptParamTrimDomain : &sParamDomain ; 

  // switch on varying parameter
  switch(eConstantParams)
    {
      // varying u
      case SM_VPS_VW : sFreeIvl   = pTgtDomain->GetUInterval() ; 
                       sFixedIvl1 = pTgtDomain->GetVInterval() ; 
                       sFixedIvl2 = pTgtDomain->GetWInterval() ; 
                       sPuvw.Set(0.0, dIsoParam1, dIsoParam2) ;
                       sVuvw.Set(1.0, 0.0, 0.0) ;
                       break ;

      // varying v
      case SM_VPS_UW : sFreeIvl = pTgtDomain->GetVInterval() ; 
                       sFixedIvl1 = pTgtDomain->GetUInterval() ; 
                       sFixedIvl2 = pTgtDomain->GetWInterval() ;                     
                       sPuvw.Set(dIsoParam1, 0.0, dIsoParam2) ;
                       sVuvw.Set(0.0, 1.0, 0.0) ;                      
                       break ;

      // varying w
      case SM_VPS_UV : sFreeIvl = pTgtDomain->GetWInterval() ; 
                       sFixedIvl1 = pTgtDomain->GetUInterval() ; 
                       sFixedIvl2 = pTgtDomain->GetVInterval() ;                     
                       sPuvw.Set(dIsoParam1, dIsoParam2, 0.0) ;
                       sVuvw.Set(0.0, 0.0, 1.0) ;                      
                       break ;

      default : SER(SM_ERR) ;
    }

  // watch for trimming
  if(   pOptParamTrimDomain == NULL
     || (   sFixedIvl1.ContainsValue(dIsoParam1, SM_EFF_ZERO)
         && sFixedIvl2.ContainsValue(dIsoParam2, SM_EFF_ZERO)))
    {
      // map point and vector to project space
      SmPoint3d  sPxyz ;
      SmVector3d sVxyz ;
      TransformPoint(sPuvw, sPxyz) ;
      TransformVector(sVuvw, sVxyz) ;

      // watch out for degenerate projections
      double dLenSq = sVxyz.LengthSquared() ;

      // make the isocurve
      if(!SM_IS_ZERO_TO_TOL(dLenSq, SM_EFF_ZERO_SQ)) 
        {
          // infinite line with Scale = sVxyz.Length()
          rpNewIsoCurve = new (crContext) SmLine(sPxyz, sVxyz) ;
          
          // bound the line
          rpNewIsoCurve->AdjustSTEPInterval(sFreeIvl) ;
        }
      else                    
        { // degenerate line
          SmCurve::CreateDegenerateCurve(crContext, 3, sPxyz, rpNewIsoCurve) ;  
        }

    } // end watch for trimming check

  // all done
  return SM_SUCCESS ;

} // end SmTransform::EvaluateIsoParametricCurveSimple

/*******************************************************************//**
PURPOSE: Create ProjSpace IsoSurface from a ParamSpace IsoParamPlane

NOTES: Specify IsoPlane as a constant param value as:
     +----------------+---------------+--------------------+
     | eConstantParam | dIsoParam | varying parameters |
     +----------------+---------------+--------------------+
     |   SM_VP_U      | constant u    | varying v, w       |
     |   SM_VP_V      | constant v    | varying u, w       |
     |   SM_VP_W      | constant w    | varying u, v       |
     +----------------+---------------+--------------------+

    Returns SM_ERR whenever the project degenerates such that the 
      isoSurface degenerates to a curve or point and sets the
      return argument to NULL.
***********************************************************************/
SmStatus SmTransform::EvaluateIsoParametricSurfaceSimple
 (const SmContext   & crContext,        // in : context for created objects
  SmVolumeParamType   eConstantParam,   // in : oneof: SM_VP_U_IN,
                                        //             SM_VP_V_IN,
                                        //             SM_VP_W_IN
  double              dIsoParam,        // in : constant param value
  double              d3DTolerance,     // NotUsed: in : Max ApproxSurface to IdealSurface deviation        
  SmSurface        *& rpNewIsoSurface,  // out: the ProjSpace IsoSurface                        
  const SmExtent3d  * pOptParamDomain)  // in : limiting domain, NULL to ignore. default:[NULL]
const
{
  SM_REF1(d3DTolerance) ;
  // init output
  rpNewIsoSurface = NULL ;

  // locals
  SmPoint3d         sPuvw ;
  SmExtent2d        sFreeUV ;
  SmVector2d        sUVScale(1.0,1.0) ;
  SmExtent3d        sParamDomain = GetNaturalParamDomain() ;
  const SmExtent3d *pTgtDomain   = pOptParamDomain ? pOptParamDomain : &sParamDomain ;
  SmExtent1d        sFixedIvl ;

  // need all 3 direction projections
  SmVector3d sU(1,0,0), sV(0,1,0), sW(0,0,1) ;
  SmVector3d sPu, sPv, sPw, sNormal, sDir1, sDir2 ;
  TransformVector(sU, sPu) ;
  TransformVector(sV, sPv) ;
  TransformVector(sW, sPw) ;

  // switch on varying parameter
  switch(eConstantParam)
    {
      // constant u
      case SM_VP_U : sFreeUV.SetMinMax(pTgtDomain->GetVMin(),
                                       pTgtDomain->GetWMin(),
                                       pTgtDomain->GetVMax(),
                                       pTgtDomain->GetWMax()) ;
                     sFixedIvl = pTgtDomain->GetUInterval() ;
                     sPuvw.Set(dIsoParam, 0.0, 0.0) ;
                     sNormal = sPu ;
                     sDir1   = sPv ;
                     sDir2   = sPw ;
                     break ;

      // constant v
      case SM_VP_V : sFreeUV.SetMinMax(pTgtDomain->GetWMin(),
                                       pTgtDomain->GetUMin(),
                                       pTgtDomain->GetWMax(),
                                       pTgtDomain->GetUMax()) ;
                     sFixedIvl = pTgtDomain->GetVInterval() ;
                     sPuvw.Set(0.0, dIsoParam, 0.0) ;
                     sNormal = sPv ;
                     sDir1   = sPw ;
                     sDir2   = sPu ;                      
                     break ;

      // constant w
      case SM_VP_W : sFreeUV.SetMinMax(pTgtDomain->GetUMin(),
                                       pTgtDomain->GetVMin(),
                                       pTgtDomain->GetUMax(),
                                       pTgtDomain->GetVMax()) ;
                     sFixedIvl = pTgtDomain->GetWInterval() ;
                     sPuvw.Set(0.0, 0.0, dIsoParam) ;
                     sNormal = sPw ;
                     sDir1   = sPu ;
                     sDir2   = sPv ;                      
                     break ;

      default : SER(SM_ERR) ;
    }

  // watch for trimming
  if(   pOptParamDomain == NULL
     || sFixedIvl.ContainsValue(dIsoParam, SM_EFF_ZERO))
    {
      // map point to xyz space
      SmPoint3d sPxyz ;
      TransformPoint(sPuvw, sPxyz) ;

      // watch out for degenerate projections
      SmStatus eStatNormal = sNormal.Unitize() ;
      double   dLen1       = sDir1.Length() ;
      double   dLen2       = sDir2.Length() ;

      // when isosurface is non degenerate
      if(   !SM_IS_ZERO(dLen1)
         && !SM_IS_ZERO(dLen2))
        {
          // ok to degenerte to a plane - all isoplanes will end up being the same
          if(eStatNormal != SM_SUCCESS)
            {
              sNormal = sDir1 * sDir2 ;
              SER(sNormal.Unitize()) ;
            }

          // plane
          sUVScale.Set(dLen1, dLen2) ;
          sDir1 /= dLen1 ;
          sDir2 /= dLen2 ;
          rpNewIsoSurface = new (crContext) SmPlane(sPxyz, sDir1, sDir2, sUVScale, sFreeUV, &crContext) ;

          //      // not needed - already trimmed to sFreeUV // when asked - trim
          //      if(pOptParamDomain)
          //        {
          //          rpNewIsoSurface->AdjustSTEPUVDomain(sFreeUV) ; 
          //        }

          // all done 
          return(SM_SUCCESS) ;
   
        } // end check for nondegenerate iso surface
    } // end watch for trimming check

  // arrive here for degenerate isosurfaces
  return(SM_ERR) ;

} // end SmTransform::EvaluateIsoParametricSurfaceSimple

/*******************************************************************//**
PURPOSE: Do a quick approximate inverse mapping for upcoming newton raphson

NOTES: Transform is linear - just do the inverse mapping - no guessing
***********************************************************************/
SmStatus SmTransform::InvEvaluateGuessPointSimple
 (const SmPoint3d     & crProjPoint,        // in : ProjSpace Point to map back to ParamSpace Point
  SmTArray<SmPoint3d> & rGuessParamPoints)  // out: ParamSpace Point near Target Point actual map back to ParamSpace
 const
{
  // init output
  rGuessParamPoints.SetSize(1) ;

  // return the inverse mapping
  return( InvTransformPoint(crProjPoint, rGuessParamPoints[0]) ) ;

} // end SmTransform::InvEvaluateGuessPointSimple

/*******************************************************************//**
PURPOSE: inverse map a point from ProjSpace back to ParamSpace when 
         no ParamPoint guess is available

NOTES: Transform is linear - just do the inverse mapping - no guessing
***********************************************************************/
SmStatus SmTransform::GlobalPointSolveSimple // eff: Find volume UVW Point that maps to TargetPoint
 (const SmPoint3d  & crProjPoint,            // in : ProjSpace Point to map back to ParamSpace
  SmBoolean        & rbFoundAnswer,          // out: TRUE = successfully mapped ProjSpace Point to a ParamSpace Point
  SmSolutionArray  & rSolutions,             // out: Contains ParamSpace Found Point
                                             //      rSolution[i].m_eSolutionType           = SM_ST_SINGLE_VALUE                                               
                                             //      rSolution[i].m_vStart.m_dSolutionValue = PointOnVolume.DistanceBetween(crProjPoint);                      
                                             //      rSolution[i].m_vStart[0] =  U of [U,V,W] the found ParamSpace point location                    
                                             //      rSolution[i].m_vStart[1] =  V of [U,V,W] the found ParamSpace point location                    
                                             //      rSolution[i].m_vStart[2] =  W of [U,V,W] the found ParamSpace point location                    
  const SmExtent3d * pOptParamDomain)        // NotUsed: in : ParamSpace domain over which to search for the inverse point
 const                                       //      NULL = use Map's NaturalDomain, default:[NULL]
{
  SM_REF1(pOptParamDomain) ;
  // init output
  rSolutions.SetSize(1) ;

  // inverse mapping
  SmPoint3d sPuvw ; 
  SmStatus eStat = InvTransformPoint(crProjPoint, sPuvw) ;

  // set output
  if(eStat == SM_SUCCESS)
    {
      rbFoundAnswer = TRUE ;

      rSolutions[0].m_eSolutionType            = SM_ST_SINGLE_VALUE ;
      rSolutions[0].m_lNumVariables            = 3 ;
      rSolutions[0].m_vStart.m_dSolutionValue  = 0.0 ;
      rSolutions[0].m_vStart.m_adParameters[0] = sPuvw.x ;
      rSolutions[0].m_vStart.m_adParameters[1] = sPuvw.y ;
      rSolutions[0].m_vStart.m_adParameters[2] = sPuvw.z ;
      rSolutions[0].m_lNumObjects              = 1 ;
      rSolutions[0].m_apObjects[0]             = (SmObject *)this ;
    }
  else // mapping is not invertable
    {
      rbFoundAnswer             = FALSE ;
      rSolutions[0].m_eSolutionType = SM_ST_UNKNOWN ;
    }
  
  // all done
  return(eStat) ;  

} // end SmTransform::GlobalPointSolveSimple

/*******************************************************************//**
PURPOSE: inverse map a point from ProjSpace back to ParamSpace with a guess point

NOTES: Transform is linear - just do the inverse mapping - no guessing
***********************************************************************/
SmStatus SmTransform::LocalPointSolveSimple
 (const SmPoint3d  & crProjPoint,          // in : ProjSpace Point to map back to ParamSpace
  const SmPoint3d  & crParamPointGuess,    // NotUsed: in : ParamSpace point guess, the closer to the actual ParamSpace point the better
  SmBoolean        & rbFoundAnswer,        // out: TRUE = successfully mapped ProjSpace Point to a ParamSpace Point            
  SmSolution       & rSolution,            // out: Contains ParamSpace Found Point
                                           //      rSolution[i].m_eSolutionType           = SM_ST_SINGLE_VALUE                                               
                                           //      rSolution[i].m_vStart.m_dSolutionValue = PointOnVolume.DistanceBetween(crProjPoint);                      
                                           //      rSolution[i].m_vStart[0]  =  U of [U,V,W] the found ParamSpace point location                    
                                           //      rSolution[i].m_vStart[1]  =  V of [U,V,W] the found ParamSpace point location                    
                                           //      rSolution[i].m_vStart[2]  =  W of [U,V,W] the found ParamSpace point location                    
                                           //      rSolution.m_lNumVariables = 3 ;               
                                           //      rSolution.m_lNumObjects   = 1 ;               
                                           //      rSolution.m_apObjects[0]  = (SmVolume *)this ;
  const SmExtent3d * pOptParamDomain)      // in : ParamSpace domain over which to search for the inverse point
 const                                     //      NULL = use Map's NaturalDomain, default:[NULL]
{
  // no need for the guess point
  SM_REF1(crParamPointGuess) ;
  SmSolutionArray sSolutions ;

  // pass the call along
  SER(GlobalPointSolveSimple(crProjPoint, rbFoundAnswer, sSolutions, pOptParamDomain)) ;

  // set output
  rSolution = sSolutions[0] ;

  // all done
  return(SM_SUCCESS) ;

} // end SmTransform::LocalPointSolveSimple

/*******************************************************************//**
PURPOSE: Return TRUE when bases are orthogonal to one another.
NOTES: Useful for figuring out when partially unbounded BBoxes can project 
       to another partially unbounded BBox.
***********************************************************************/
SmBoolean SmTransform::IsOrthogonal() 
 const 
{
  // check orthogonal
  double dXSize = GetToXAxis().Length() ; 
  double dYSize = GetToYAxis().Length() ; 
  double dZSize = GetToZAxis().Length() ; 
                                                         
  double dDotXY = GetToXAxis().Dot(GetToYAxis())/dXSize/dYSize ;
  double dDotYZ = GetToYAxis().Dot(GetToZAxis())/dYSize/dZSize ;
  double dDotZX = GetToZAxis().Dot(GetToXAxis())/dZSize/dXSize ;

  SmBoolean bRtn = (   SM_IS_ZERO(dDotXY)
                     && SM_IS_ZERO(dDotYZ)
                     && SM_IS_ZERO(dDotZX)) ;
                                        
  // all done
  return(bRtn) ; 

} // end SmTransform::IsOrthogonal

/*******************************************************************//**
PURPOSE: Return TRUE when bases are orthogonal to one another
         and parallel to the X, Y, and Z axes.
NOTES: Useful for figuring out when partially unbounded BBoxes can project 
       to another partially unbounded BBox.
***********************************************************************/
SmBoolean SmTransform::IsAxisAligned
 (long alAxisMap[3])                  // out: alAxisMap[0] = index + 1 of basis parallel to X, neg = in negative direction
 const                                //      alAxisMap[1] = index + 1 of basis parallel to Y, neg = in negative direction
                                      //      alAxisMap[2] = index + 1 of basis parallel to Z, neg = in negative direction
 
{
  // low work - not orthogonal
  if(IsOrthogonal() == FALSE)
    { return FALSE ; }

  // locals
  ULONG ii ;
  SmVector3d sX(1,0,0), sY(0,1,0), sZ(0,0,1) ;

  // for each basis
  for(ii=0;ii<3;ii++)
    {
      SmVector3d &sBasisII =   ii == 0 ? GetToXAxis()
                             : ii == 1 ? GetToYAxis()
                             :           GetToZAxis() ;
      double dLength = sBasisII.Length() ;

      double dXDot = sBasisII.Dot(sX)/dLength ; 
      double dYDot = sBasisII.Dot(sY)/dLength ; 
      double dZDot = sBasisII.Dot(sZ)/dLength ;          
                                                         
      if     (SM_ARE_SAME(dXDot,  1.0)) alAxisMap[ii] =  1 ;
      else if(SM_ARE_SAME(dXDot, -1.0)) alAxisMap[ii] = -1 ;
      else if(SM_ARE_SAME(dYDot,  1.0)) alAxisMap[ii] =  2 ;
      else if(SM_ARE_SAME(dYDot, -1.0)) alAxisMap[ii] = -2 ;
      else if(SM_ARE_SAME(dZDot,  1.0)) alAxisMap[ii] =  3 ;
      else if(SM_ARE_SAME(dZDot, -1.0)) alAxisMap[ii] = -3 ;
      else { return(FALSE) ; }

    } // end iter every basis

  // all done
  return(TRUE) ; 

} // end SmTransform::IsAxisAligned

/*******************************************************************//**
PURPOSE: 

NOTES: 
***********************************************************************/
SmBoolean SmTransform::IsClosedSimple
 (SmBoolean        & rbClosedU,          // out: TRUE = closed in U direction, [check Pos[Umin,v,w] == Pos[Umax,v,w] for v,w samples
  SmBoolean        & rbClosedV,          // out: TRUE = closed in V direction, [check Pos[u,Vmin,w] == Pos[u,Vmax,w] for w,u samples
  SmBoolean        & rbClosedW,          // out: TRUE = closed in W direction, [check Pos[u,v,Wmin] == Pos[u,v,Wmax] for u,v samples
  double           * pdOptTolerance,     // NotUsed: out: pdOptTolerance = Tolerance to allow for check
  const SmExtent3d * pOptParamDomain,    // NotUsed: in : ParamSpace domain over which to search for the inverse point
                                         //      NULL = use Map's NaturalDomain, default:[NULL]
  SmContinuityType * peOptContinuityU,   // out: U dir Continuity when closed 
  SmContinuityType * peOptContinuityV,   // out: V dir Continuity when closed      
  SmContinuityType * peOptContinuityW)   // out: W dir Continuity when closed
                                         //      oneof SM_CT_DISCONTINUOUS      
                                         //            SM_CT_C0                 
                                         //            SM_CT_G1                 
                                         //            SM_CT_G1_G2              
                                         //            SM_CT_G1_G2_G3              
                                         //            SM_CT_C1                 
                                         //            SM_CT_C1_G2        
                                         //            SM_CT_C1_G2_G3        
                                         //            SM_CT_C1_C2        
                                         //            SM_CT_C1_G3        
                                         //            SM_CT_C1_C3        
 const
{
  SM_REF2(pdOptTolerance, pOptParamDomain) ;
  rbClosedU = FALSE ;
  rbClosedV = FALSE ;
  rbClosedW = FALSE ;

  if(peOptContinuityU) * peOptContinuityU = SM_CT_CINFINITY ;
  if(peOptContinuityV) * peOptContinuityV = SM_CT_CINFINITY ;
  if(peOptContinuityW) * peOptContinuityW = SM_CT_CINFINITY ;

  // all done
  return(FALSE) ;

} // end SmTransform::IsClosedSimple    

/*******************************************************************//**
PURPOSE: 

NOTES: 
***********************************************************************/
SmBoolean SmTransform::IsPeriodicSimple
 (SmBoolean        & rbPeriodicU,      // out: TRUE = G1 or better in U Dir 
  SmBoolean        & rbPeriodicV,      // out: TRUE = G1 or better in V Dir
  SmBoolean        & rbPeriodicW,      // out: TRUE = G1 or better in W Dir
  const SmExtent3d * pOptParamDomain)  // NotUsed: in : ParamSpace domain over which to search for the inverse point
 const                                 //      NULL = use Map's NaturalDomain, default:[NULL]
{
  SM_REF1(pOptParamDomain) ;
  rbPeriodicU = FALSE ;
  rbPeriodicV = FALSE ;
  rbPeriodicW = FALSE ;

  // all done
  return(FALSE) ;

} // end SmTransform::IsPeriodicSimple 

/*******************************************************************//**
PURPOSE: 

NOTES: 
***********************************************************************/   
SmBoolean SmTransform::IsSingularitySimple      
  (const SmPoint3d   & crUVWToTest,        // NotUsed: in : UVpoint to test                 
   SmBoolean         & rbSingularU,        // out: TRUE = [Su=0]
   SmBoolean         & rbSingularV,        // out: TRUE = [Sv=0]
   SmBoolean         & rbSingularW,        // out: TRUE = [Sw=0]
   double              d3dTol)             // NotUsed: in : min 3d distance between distinct points) const; 
  const 
{
  SM_REF2(crUVWToTest, d3dTol) ;
  rbSingularU = FALSE ;
  rbSingularV = FALSE ;
  rbSingularW = FALSE ;

  // all done
  return(FALSE) ;

} // end SmTransform::IsSingularitySimple    

/*******************************************************************//**
PURPOSE: 

NOTES: 
***********************************************************************/
SmBoolean SmTransform::IsOnBoundarySimple
  (const SmPoint3d    & crParamPoint,    // NotUsed: in : Volume UVWPoint to test
   SmBoolean          & rbOnU,           // out: TRUE = TargetPoint on UMin or UMax
   SmBoolean          & rbOnV,           // out: TRUE = TargetPoint on VMin or VMax
   SmBoolean          & rbOnW,           // out: TRUE = TargetPoint on WMin or WMax
   double             * pdOptTolerance,  // NotUsed: in : max deviation allowed for point on seam
                                         //      NULL = use SM_EFF_ZERO * 1000 * (1 + maxDimension())
  const SmExtent3d    * pOptParamDomain) // NotUsed: in : ParamSpace domain over which to search for the inverse point
 const                                   //      NULL = use Map's NaturalDomain, default:[NULL]
{
  SM_REF3(crParamPoint, pdOptTolerance, pOptParamDomain) ;
  rbOnU = FALSE ;
  rbOnV = FALSE ;
  rbOnW = FALSE ;

  // all done
  return(FALSE) ;

} // end SmTransform::IsOnBoundarySimple    

/*******************************************************************//**
PURPOSE: Write SmTransform to given output stream.

NOTES: 
***********************************************************************/
SmStatus SmTransform::WriteToDB
 (SmDatabaseIO & rDB,              // in : target output stream
  ULONG          lDBVersionNumber) // in : database version to get proper sequence of writes                                                                       
 const
{
  // file type, ASCII or BINARY
  SmFileType eType = rDB.GetFileType();

  // Volume locals
  double *pX = (double *)&GetToXAxis() ; 
  double *pY = (double *)&GetToYAxis() ; 
  double *pZ = (double *)&GetToZAxis() ; 
  double *pD = (double *)&GetToDisp() ; 

  // Create the output file 
  if (eType == SM_ASCII) { std::ostream & rFileOut = *rDB.GetOutStreamPtr() ;

                           rFileOut << pX[0] << " " << pX[1] << " " << pX[2] << " " << pX[3] << " Transform X row \n" ;
                           rFileOut << pY[0] << " " << pY[1] << " " << pY[2] << " " << pY[3] << " Transform Y row \n" ;
                           rFileOut << pZ[0] << " " << pZ[1] << " " << pZ[2] << " " << pZ[3] << " Transform Z row \n" ;
                           rFileOut << pD[0] << " " << pD[1] << " " << pD[2] << " " << pD[3] << " Transform Disp row \n" ;
                         }
  else /* Binary */      { SER(rDB.WriteDouble(pX[0])) ;
                           SER(rDB.WriteDouble(pX[1])) ;
                           SER(rDB.WriteDouble(pX[2])) ;
                           SER(rDB.WriteDouble(pX[3])) ;
                                                       
                           SER(rDB.WriteDouble(pY[0])) ;
                           SER(rDB.WriteDouble(pY[1])) ;
                           SER(rDB.WriteDouble(pY[2])) ;
                           SER(rDB.WriteDouble(pY[3])) ;
                                                       
                           SER(rDB.WriteDouble(pZ[0])) ;
                           SER(rDB.WriteDouble(pZ[1])) ;
                           SER(rDB.WriteDouble(pZ[2])) ;
                           SER(rDB.WriteDouble(pZ[3])) ;
                                                       
                           SER(rDB.WriteDouble(pD[0])) ;
                           SER(rDB.WriteDouble(pD[1])) ;
                           SER(rDB.WriteDouble(pD[2])) ;
                           SER(rDB.WriteDouble(pD[3])) ;
                         }

  // write base class data
  SmVolume::WriteToDB(rDB, lDBVersionNumber) ;

  // all done
  return SM_SUCCESS;

} // end SmTransform::WriteToDB

/*******************************************************************//**
PURPOSE: static method to Read a SmTransform from a given stream  

NOTES: 
***********************************************************************/
SmStatus SmTransform::ReadFromDB
 (SM_TYPE           lType,              // NotUsed: in : curve type to be read
  SmDatabaseIO    & rDB,                // in : target output stream
  const SmContext & crContext,          // in : context for new object construction
  SmVolume       *&rpNewVolume,         // out:    NULL on input = new object allocated in this routine built from stream data
                                        //      NotNULL on input = pointer to an empty object to be filled by this routine
  ULONG             lDBVersionNumber)   // in : database version to get proper sequence of writes
{
  SM_REF1(lType) ;
  // check input
  SER(  (   rpNewVolume == NULL
         || rpNewVolume->IsKindOf(SmTransform_TYPE)) ? SM_SUCCESS : SM_ERR) ;

  // init output object
  SmTransform *pTransformVolume =   (rpNewVolume == NULL)
                                 ? new (crContext) SmTransform()
                                 : (SmTransform *)rpNewVolume ;

  // file type
  SmFileType eType = rDB.GetFileType();
  

  // Volume locals
  double *pX = (double *)&pTransformVolume->GetToXAxis() ; 
  double *pY = (double *)&pTransformVolume->GetToYAxis() ; 
  double *pZ = (double *)&pTransformVolume->GetToZAxis() ; 
  double *pD = (double *)&pTransformVolume->GetToDisp() ; 

  if (eType == SM_ASCII) 
    {
      std::istream & rFileIn = *rDB.GetInStreamPtr();
      rFileIn >> pX[0] >> pX[1] >> pX[2] >> pX[3] ;  rDB.GoToNextLine() ;
      rFileIn >> pY[0] >> pY[1] >> pY[2] >> pY[3] ;  rDB.GoToNextLine() ;
      rFileIn >> pZ[0] >> pZ[1] >> pZ[2] >> pZ[3] ;  rDB.GoToNextLine() ;
      rFileIn >> pD[0] >> pD[1] >> pD[2] >> pD[3] ;  rDB.GoToNextLine() ;
    }
  else 
    {
      SER(rDB.ReadDouble(pX[0]));
      SER(rDB.ReadDouble(pX[1]));
      SER(rDB.ReadDouble(pX[2]));
      SER(rDB.ReadDouble(pX[3]));

      SER(rDB.ReadDouble(pY[0]));
      SER(rDB.ReadDouble(pY[1]));
      SER(rDB.ReadDouble(pY[2]));
      SER(rDB.ReadDouble(pY[3]));

      SER(rDB.ReadDouble(pZ[0]));
      SER(rDB.ReadDouble(pZ[1]));
      SER(rDB.ReadDouble(pZ[2]));
      SER(rDB.ReadDouble(pZ[3]));

      SER(rDB.ReadDouble(pD[0]));
      SER(rDB.ReadDouble(pD[1]));
      SER(rDB.ReadDouble(pD[2]));
      SER(rDB.ReadDouble(pD[3]));
    }

  // set output
  rpNewVolume = pTransformVolume ;

  // read base class data
  SmVolume::ReadFromDB(SmVolume_TYPE, rDB, crContext, rpNewVolume, lDBVersionNumber ) ; 

  // all done
  return SM_SUCCESS;

} // end SmTransform::ReadFromDB

/*******************************************************************//**
PURPOSE: Compute the total size of the memory used by the SmEllipse.

NOTES: includes attribute memory
***********************************************************************/
ULONG SmTransform::GetMemoryUsed    // rtn: smaller size of actually used memory in bytes
 (ULONG    & rlMemoryAllocated,     // out: bigger size of all allocated memory in bytes
  SmMarkType eMarkType)             // in : uses without increment eMarkType value
 const
{
  // in case this method is called directly - get a mark for attribute memory usage
  SmNewMarkAndLock sMarkLock ;
  if(eMarkType == SM_MT_NOMARK)
    {
      eMarkType = sMarkLock.SetContext((SmContext *)GetContext()) ;
    }

  // this + m_pNurb memory
  rlMemoryAllocated = sizeof(*this) ;

  // + attribute memory
  ULONG lThisAllocated ;
  ULONG lUsed       = rlMemoryAllocated + this->GetAttributeMemoryUsed(lThisAllocated, 
                                                                       eMarkType) ;  // note: uses without increment eMarkType value
  rlMemoryAllocated += lThisAllocated ;

  // all done
  return(lUsed) ;

} // end SmTransform::GetMemoryUsed

/*******************************************************************//**
PURPOSE:
NOTES:
***********************************************************************/
SmBoolean SmTransform::IsKindOf( SM_TYPE t ) const
{
  return ((SmTransform_TYPE == t) ? TRUE : SmVolume::IsKindOf( (t) ));
}

/*******************************************************************//**
PURPOSE: pretty print

NOTES:
***********************************************************************/
void SmTransform::Dump() const
{
  Dump(FALSE, 0) ;
}

/*******************************************************************//**
PURPOSE: pretty print

NOTES:
***********************************************************************/
void SmTransform::Dump
 (SmBoolean bAbbrev,  // in : TRUE = skip nested object dumps, FALSE=include nested object dumps
  ULONG lIndentCnt)   // in : Indent print statements by lIndentCnt number of spaces
 const
{
  // locals
  ULONG ii ;
  TCHAR sBuff[SM_TBLOCK_SIZE], sBuffForFile[SM_TBLOCK_SIZE], sIndent[32] = {};

  // build prefix
  for(ii=0;ii<lIndentCnt;ii++) 
    { SM_STRCAT(sIndent, _T(" ")) ; }

  // locals
  double *rX = (double *)&GetToXAxis() ;
  double *rY = (double *)&GetToYAxis() ;
  double *rZ = (double *)&GetToZAxis() ;
  double *rD = (double *)&GetToDisp() ;

  // header
  smos_sprintf(sBuff,        _T("\n%sBegin SmTransform[0x%p]::Dump()"), sIndent, this) ;
  smos_sprintf(sBuffForFile, _T("\n%sBegin SmTransform::Dump()"), sIndent) ;
  smos_WriteBuffer(sBuff, sBuffForFile);

  // header
  smos_sprintf(sBuff,       _T("\n%s  SmTransform = 0x%p"), sIndent, this);
  smos_sprintf(sBuffForFile,_T("\n%s  SmTransform = %s"), sIndent, _T("notNULL") );
  smos_WriteBuffer(sBuff, sBuffForFile);

  smos_sprintf(sBuff,_T("\n%s    XAxisRow  =  [%16.16lf, %16.16lf, %16.16lf, %16.16lf] "), sIndent, rX[0],rX[1],rX[2],rX[3]);
  smos_WriteBuffer(sBuff, sBuffForFile);

  smos_sprintf(sBuff,_T("\n%s    YAxisRow  =  [%16.16lf, %16.16lf, %16.16lf, %16.16lf] "), sIndent, rY[0],rY[1],rY[2],rY[3]);
  smos_WriteBuffer(sBuff, sBuffForFile);

  smos_sprintf(sBuff,_T("\n%s    ZAxisRow  =  [%16.16lf, %16.16lf, %16.16lf, %16.16lf] "), sIndent, rZ[0],rZ[1],rZ[2],rZ[3]);
  smos_WriteBuffer(sBuff, sBuffForFile);

  smos_sprintf(sBuff,_T("\n%s    DispRow   =  [%16.16lf, %16.16lf, %16.16lf, %16.16lf] "), sIndent, rD[0],rD[1],rD[2],rD[3]);
  smos_WriteBuffer(sBuff, sBuffForFile);

  // dump the base class 
  SmVolume::Dump(bAbbrev, lIndentCnt+2) ;

  smos_sprintf(sBuff,        _T("\n%sEnd SmTransform[0x%p]::Dump()%s"), sIndent, this, lIndentCnt==0?_T("\n"):_T("")) ;
  smos_sprintf(sBuffForFile, _T("\n%sEnd SmTransform::Dump()%s"), sIndent, lIndentCnt==0?_T("\n"):_T("")) ;
  smos_WriteBuffer(sBuff, sBuffForFile);
  
} // end SmTransform::Dump

/*******************************************************************//**
PURPOSE: Draw Axis - origin(black), x(red), y(green), z(blue)

NOTES: Assumes current coordinate system is ParamSpace
       Displays the OutSpace coordinate system  (red, green, blue)
       Display the ParamSpace coordinate system GreysOf(red, green blue)
***********************************************************************/
SmDisplayList *SmTransform::DrawAxis
  (SmExtent2d *pOptUVDomain,      // in : optionally output an XYPlane subDomain graphic, NULL to ignore
                                  //      default:[NULL]
   SmGfxArraySet * pOptGfxSet)    // in : NULL = draw display lists, else draw to ArraySet
                                  //      default:[NULL]
 const
{
  SmDisplayList *pRtn = NULL ;

#ifdef SM_GFX_CODE
  SmVector3d sParamU(1,0,0) ;
  SmVector3d sParamV(0,1,0) ;
  SmVector3d sParamW(0,0,1) ;
  SmVector3d sParamOrigin(0,0,0) ;

  SmVector3d &rOutX = GetToXAxis() ;
  SmVector3d &rOutY = GetToYAxis() ;
  SmVector3d &rOutZ = GetToZAxis() ;
  SmVector3d &rOutOrigin = GetToDisp() ;

  SmVector3d sColor     = smgfx_GetRuleColor();
  double     dLineWidth = smgfx_GetLineWidth() ;

  smgfx_Open(smgfx_GetRuleColor(),NULL, NULL, FALSE, pOptGfxSet);

  smgfx_SetLineWidth(1.0, pOptGfxSet) ;
  smgfx_SetColor(0,0,0,pOptGfxSet); sParamOrigin.Draw(NULL,NULL,pOptGfxSet);
  smgfx_SetColor(.8,.4,.4,pOptGfxSet); sParamU.Draw(&sParamOrigin,NULL,pOptGfxSet);
  smgfx_SetColor(.4,.8,.4,pOptGfxSet); sParamV.Draw(&sParamOrigin,NULL,pOptGfxSet);
  smgfx_SetColor(.4,.4,.8,pOptGfxSet); sParamW.Draw(&sParamOrigin,NULL,pOptGfxSet);
                         
  smgfx_SetLineWidth(dLineWidth, pOptGfxSet) ;
  smgfx_SetColor(0,0,0,pOptGfxSet); rOutOrigin.Draw(NULL,NULL,pOptGfxSet);
  smgfx_SetColor(1,0,0,pOptGfxSet); rOutX.Draw(&rOutOrigin,NULL,pOptGfxSet);
  smgfx_SetColor(0,1,0,pOptGfxSet); rOutY.Draw(&rOutOrigin,NULL,pOptGfxSet);
  smgfx_SetColor(0,0,1,pOptGfxSet); rOutZ.Draw(&rOutOrigin,NULL,pOptGfxSet);

  // from-to icon - an arrow with a head drawn in black
  SmVector3d sFromTo = rOutOrigin - sParamOrigin ;
  double     dLength  = sFromTo.Length() ;
  double     dLength1 = rOutX.Length() ;
  double     dLength2 = rOutY.Length() ;
  double     dLength3 = rOutZ.Length() ;
  double     dMinLength = smos_3Min(dLength1 > SM_EFF_ZERO ? dLength1 : SM_BIG_DOUBLE,
                                    dLength2 > SM_EFF_ZERO ? dLength2 : SM_BIG_DOUBLE,
                                    dLength3 > SM_EFF_ZERO ? dLength3 : SM_BIG_DOUBLE) ;
  if(dLength > SM_EFF_ZERO)
    {
      dLength = smos_Min(dLength, dMinLength > SM_EFF_ZERO ? dMinLength : SM_BIG_DOUBLE) ;

      SmVector3d sOrtho1, sOrtho2, sAxis ;
      sFromTo.MakeUnitOrthoVectors(NULL, sAxis, sOrtho1, sOrtho2) ;
      sOrtho1 *= .05 * dLength ;
      sOrtho2 *= .05 * dLength ;
      SmPoint3d sP0 = .85*rOutOrigin+.15*sParamOrigin ;
      SmPoint3d sP1 = sP0 + sOrtho1 ;
      SmPoint3d sP2 = sP0 + sOrtho2 ;
      SmPoint3d sP3 = sP0 - sOrtho1 ;
      SmPoint3d sP4 = sP0 - sOrtho2 ;
      smgfx_SetColor(0,0,0,pOptGfxSet); 
      smgfx_SetLineWidth(1.0, pOptGfxSet) ;

      smgfx_DrawLine(&sParamOrigin, &rOutOrigin, pOptGfxSet) ;

      smgfx_DrawLine(&sP0,&sP1,pOptGfxSet) ;
      smgfx_DrawLine(&sP0,&sP2,pOptGfxSet) ;
      smgfx_DrawLine(&sP0,&sP3,pOptGfxSet) ;
      smgfx_DrawLine(&sP0,&sP4,pOptGfxSet) ;

      smgfx_DrawLine(&rOutOrigin,&sP1,pOptGfxSet) ;
      smgfx_DrawLine(&rOutOrigin,&sP2,pOptGfxSet) ;
      smgfx_DrawLine(&rOutOrigin,&sP3,pOptGfxSet) ;
      smgfx_DrawLine(&rOutOrigin,&sP4,pOptGfxSet) ;
            
      smgfx_DrawLine(&sP2,&sP1,pOptGfxSet) ;
      smgfx_DrawLine(&sP3,&sP2,pOptGfxSet) ;
      smgfx_DrawLine(&sP4,&sP3,pOptGfxSet) ;
      smgfx_DrawLine(&sP1,&sP4,pOptGfxSet) ;

      smgfx_SetColor(sColor,pOptGfxSet) ;
      smgfx_SetLineWidth(dLineWidth, pOptGfxSet) ;
    }

  // some plane graphics
  ULONG ii ;
  double daPoints[15] ;

  // ProjSpace XY Plane Graphics - drawn in Blue (same as Z unit vector)
  smgfx_SetLook(1,2, 0,0,1,pOptGfxSet); 
  for(ii=0;ii<3;ii++)
    {
      double dScale = (ii+1.0) * (ii+1.0) * (ii+1.0) ;
      SmPoint3d *pTgt0 = (SmPoint3d *)daPoints ;
      SmPoint3d *pTgt1 = (SmPoint3d *)(daPoints+3) ;
      *pTgt0 = rOutOrigin + dScale * (rOutX + rOutY) ; 
      *pTgt1 = *pTgt0 - 2 * dScale * rOutX ; pTgt0++ ; pTgt1++ ;
      *pTgt1 = *pTgt0 - 2 * dScale * rOutY ; pTgt0++ ; pTgt1++ ;
      *pTgt1 = *pTgt0 + 2 * dScale * rOutX ; pTgt0++ ; pTgt1++ ;
      *pTgt1 = *pTgt0 + 2 * dScale * rOutY ; pTgt0++ ; pTgt1++ ;

      smgfx_DrawPolyline(daPoints, 5) ;

    } // end iter plane graphics

  // ParamSpace UV PlaneGraphics - draw in GreyBlue (same as W unit vector)
  smgfx_SetLook(1,2, .4,.4,.8,pOptGfxSet); 
  for(ii=0;ii<3;ii++)
    {
      double dScale = (ii+1.0) * (ii+1.0) * (ii+1.0) ;
      SmPoint3d *pTgt0 = (SmPoint3d *)daPoints ;
      SmPoint3d *pTgt1 = (SmPoint3d *)(daPoints+3) ;
      *pTgt0 = sParamOrigin + dScale * (sParamU + sParamV) ; 
      *pTgt1 = *pTgt0 - 2 * dScale * sParamU ; pTgt0++ ; pTgt1++ ;
      *pTgt1 = *pTgt0 - 2 * dScale * sParamV ; pTgt0++ ; pTgt1++ ;
      *pTgt1 = *pTgt0 + 2 * dScale * sParamU ; pTgt0++ ; pTgt1++ ;
      *pTgt1 = *pTgt0 + 2 * dScale * sParamV ; pTgt0++ ; pTgt1++ ;

      smgfx_DrawPolyline(daPoints, 5) ;

    } // end iter plane graphics

  smgfx_SetColor(sColor,pOptGfxSet) ;
  smgfx_SetLineWidth(dLineWidth, pOptGfxSet) ;

  // optional extent graphics
  if(pOptUVDomain)
    {
      dLineWidth = smgfx_GetLineWidth() ;
      double dPointSize = smgfx_GetPointSize() ;
      smgfx_SetLook(dLineWidth+2,dPointSize, 0,0,0, pOptGfxSet) ;

      (SmPoint3d &)(*(daPoints+ 0)) = rOutOrigin + pOptUVDomain->GetUMin() * rOutX + pOptUVDomain->GetVMin() * rOutY ; 
      (SmPoint3d &)(*(daPoints+ 3)) = rOutOrigin + pOptUVDomain->GetUMin() * rOutX + pOptUVDomain->GetVMax() * rOutY ; 
      (SmPoint3d &)(*(daPoints+ 6)) = rOutOrigin + pOptUVDomain->GetUMax() * rOutX + pOptUVDomain->GetVMax() * rOutY ; 
      (SmPoint3d &)(*(daPoints+ 9)) = rOutOrigin + pOptUVDomain->GetUMax() * rOutX + pOptUVDomain->GetVMin() * rOutY ; 
      (SmPoint3d &)(*(daPoints+12)) = rOutOrigin + pOptUVDomain->GetUMin() * rOutX + pOptUVDomain->GetVMin() * rOutY ; 
                      
      smgfx_DrawPolyline(daPoints, 5, pOptGfxSet) ;
                      
      smgfx_SetLook(dLineWidth, dPointSize, sColor.x, sColor.y, sColor.z, pOptGfxSet) ;

    } // end option UVDomain existence check

  // all done
  pRtn = smgfx_Close(pOptGfxSet) ;

#else
  SM_REF2(pOptUVDomain, pOptGfxSet);
#endif
  return(pRtn) ;

} // end SmTransform::DrawAxis

/*******************************************************************//**
PURPOSE: AssertValid() Test reporting static labels
***********************************************************************/
SmAssertReportLabel sAssertTransform_list[] =
{
  {SM_AT_UNKNOWN, _T("UNKNOWN"), _T("No constraints on m_adT to check for yet") }
} ;

/*******************************************************************//**
PURPOSE:

NOTES: returns TRUE  = OK
                        FALSE = Problem
***********************************************************************/
SmBoolean SmTransform::AssertValid
 (SmAssertArray    * pAList,        // i/o: Accumulating list of failed Asserts, NULL to ignore, default:[NULL] 
  SmAssertTestLevel  eTestLevel,    // in : SM_LEVEL_0=fewest tests, SM_LEVEL_1, SM_LEVEL_2=all tests, 
                                    //      SM_LEVEL_GIVEN = run tests in order requested in pTestRequests 
                                    //      default:[SM_LEVEL_0] 
  SmAssertWalking    eWalkTree,     // in : SM_WALK = run AssertValid on any topology graph descendants, SM_NO_WALK=don't, default:[SM_WALK] 
  SmTArray<ULONG>  * pTestRequests) // in : when eTestLevel == SM_LEVEL_GIVEN, run these tests in this order, default:[NULL]
 const
{
  // check base class 
  SmBoolean bRtn = SmVolume::AssertValid(pAList, eTestLevel, eWalkTree, pTestRequests) ;

  // run SmTransform checks
  // currently there are no limitations on the transform matrix since
  //  it can be used to mirror, project to a plane, scale anisotropically
  //  and be inverted, the transform matrix is allowed to be pretty much any 4x4 matrix 

  // all done
  return(bRtn) ;

} // end SmTransform::AssertValid

// obsolete
// /*******************************************************************//**
// PURPOSE: Fix Assert Report failures reported by an AssertValid call
// 
// RETURNS ---  TRUE  = OK
//              FALSE = Problem
// ***********************************************************************/
// SmBoolean SmTransform::AssertHeal
//  (SmAssertReport & rAReport,  // in : a report generated by AssertValid
//   SmAssertArray  * pAList)    // in : AssertArray holding rAReport 
// {
//   SmBoolean bRtn = FALSE ;
// 
//   // check state - no work
//   if(rAReport.m_bOK == TRUE)
//     { return( TRUE ) ; }
// 
//   // check state - not the class that generated this report - pass call to parent class
//   if(rAReport.m_lReportingType != GetClassType())
//     {
//       // pass the call along to the parent - return ( Parent::AssertHeal(rAReport, pAList) ) ;
//       return ( SmVolume::AssertHeal(rAReport, pAList) ) ;
//     }
//    
//   // branch on the report type
//   switch(rAReport.m_lTestIndex)
//     {
//       case 99 : { // set case number appropriately - run fix code here
//                   // if fix works set rAReport.m_bOK = TRUE ; 
//                 }
//                 break ;
// 
//       default: rAReport.m_eAssertType  = SM_AT_NO_HEAL_YET ;  
//                rAReport.m_pHealMessage = _T("SmTransform::AssertHeal fix not yet supported") ;  
//                 
//     }
// 
//   // all done
//   return(bRtn) ;
// 
// } // end SmTransform::AssertHeal
// end obsolete

