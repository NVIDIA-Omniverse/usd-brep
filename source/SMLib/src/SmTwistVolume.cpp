// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*******************************************************************//**
* FILE NAME --- SmTwistVolume.cpp
* PURPOSE: Implementation of SmTwistVolume methods.
* Oct-2006 - gwc - author
**********************************************************************/

#include "StdAfx.h"

#include <SmTwistVolume.h>
#include <SmConfig.h>
#include <SmPseudoBox.h>
#include <SmCircle.h>
#include <SmLine.h>
#include <SmCylinder.h>
#include <SmPlane.h>
#include <SmNurbsCrv.h>
#include <SmDatabaseIO.h>
#include <SmTransform.h>
#include <SmGeomUtility.h>
#include <SmAssertArray.h>
#include <SmAxis2Placement.h>
#include <SmTol.h>

#ifdef SM_DEBUG_CODE
#include <SmEdge.h>
#include <SmFace.h>
#include <SmBrep.h>
#endif // SM_DEBUG_CODE

/*******************************************************************//**
PURPOSE: Constructor

NOTES: 
***********************************************************************/
SmTwistVolume::SmTwistVolume
  (const SmContext  & crContext,        // in : context for new OrientMap construction                     
   const SmPoint3d  & rTwistOrigin,     // in : InSpace (and OutSpace) point on the twist center line                      
   const SmVector3d & rTwistAxis,       // in : InSpace (and OutSpace) direction of the twist center line marking W axis of Twist Coordinate System                                                
   const SmVector3d & rTwistMidDir,     // in : InSpace (and OutSpace) vector orthoganal to TwistAxis marking U axis of Twist Coordinate System                       
   double             dTwistPeriodDist, // in : distance along the TwistAxis for one complete Twist revolution                    
   double             dPeriodMinDist)   // in : min allowed value dTwistPeriodDist
                                        //      dPeriodMinDist must be >= SM_ZONE_TOL_3D
: SmVolume(&crContext),
  m_dTwistPeriodDist(dTwistPeriodDist),
  m_dPeriodMinDist(dPeriodMinDist)
{
  // check input - dPeriodMinDist >= SM_ZONE_TOL_3D
  if(m_dPeriodMinDist < SM_ZONE_TOL_3D)
    { 
      SE_MSG( SM_ERR,
             _T("TwistVolume dPeriodMinDist must be >= SM_ZONE_TOL_3D - Bad Construction - changing dPeriodMinDist to SM_ZONE_TOL_3D")) ;
      m_dPeriodMinDist = SM_ZONE_TOL_3D ;
    }

  // check input - Positive Definite TwistPeriodDistance
  if(smos_Fabs(dTwistPeriodDist) < m_dPeriodMinDist)
    {
      SE_MSG( SM_ERR,
             _T("TwistVolume given too small a PeriodDist - Bad Construction - changing TwistPeriodDist to dPeriodMinDist")) ;

      // do the best we can
      m_dTwistPeriodDist = dPeriodMinDist >= 0 ? dPeriodMinDist : -dPeriodMinDist ;
    }

  // unitize twist vectors
  SmVector3d sTwistAxis  (rTwistAxis) ;
  SmVector3d sTwistMidDir(rTwistMidDir) ;
  SmVector3d sTwistBinormal ;
  SmStatus   eStat1 = sTwistAxis.Unitize() ; 
  SmStatus   eStat2 = sTwistMidDir.Unitize() ;

  // check input - nonZero input vectors
  if(eStat1 != SM_SUCCESS || eStat2 != SM_SUCCESS)
    {
      SE_MSG( SM_ERR,
             _T("Twist Volume defining vectors were zero length - Bad Construction - Changing TwistVectors to have length")) ; 

      // do the best we can
      if(eStat1 == SM_ERR && eStat2 == SM_ERR) { sTwistAxis.Set(0,0,1) ; 
                                                 sTwistMidDir.Set(1,0,0) ; 
                                               }
      else if(eStat1 == SM_ERR)                { sTwistMidDir.MakeUnitOrthoVectors(NULL, sTwistMidDir,
                                                                                         sTwistBinormal,
                                                                                         sTwistAxis) ; 
                                               }
      else                                     { sTwistAxis.MakeUnitOrthoVectors(NULL, sTwistAxis,
                                                                                       sTwistMidDir,
                                                                                       sTwistBinormal) ; 
                                               }
    } // end do something when given zero length vectors branch

  // check input - twist vectors must be perpendicular to one another
  SmBoolean bPerp = sTwistAxis.IsPerpendicularTo(sTwistMidDir, SM_EFF_ZERO_DEG) ;
  if(!bPerp)
    {
       SE_MSG( SM_ERR,
              _T("TwistAxis not perpendicular to TwistMidDir - Bad Construction - Changing TwistVectors to be perpendicular")) ;

       SmBoolean bParallel = sTwistAxis.IsParallelTo(sTwistMidDir, SM_EFF_ZERO_DEG) ;

      if(!bParallel)
        {
         sTwistMidDir = sTwistAxis * (sTwistMidDir * sTwistAxis) ;
          sTwistMidDir.Unitize() ;
        }
      else // Axis and StartDir don't span a plane
        {
          // do the best we can
          sTwistMidDir.MakeUnitOrthoVectors(NULL, sTwistMidDir,
                                                 sTwistBinormal,
                                                 sTwistAxis) ;
        }
    }
    
  // set TwistBinormal perp to both TwistAxis and TwistMidDir
  sTwistBinormal = sTwistAxis * sTwistMidDir ;

  // build TwistCoordinates into m_pOrientMap
  SmTransform *pOrientMap = new (crContext) SmTransform(&crContext, rTwistOrigin, sTwistMidDir, sTwistBinormal, &sTwistAxis) ;
  SetOrientMap(pOrientMap, 2) ;

} // end SmTwistVolume constructor

/*******************************************************************//**
PURPOSE: Copy constructor for a TwistVolume Volume.

NOTES:      
***********************************************************************/
SmTwistVolume::SmTwistVolume
 (const SmTwistVolume & crSourceVolume,   // in : SourceVolume to copy
  SmBoolean             bSimpleMapOnly)   // in : TRUE = Copy this Volume omitting any compounding volumes
                                          //      FALSE= Copy this Volume with any compounding volumes 
: SmVolume(crSourceVolume, bSimpleMapOnly),
  m_dTwistPeriodDist(crSourceVolume.m_dTwistPeriodDist),
  m_dPeriodMinDist (crSourceVolume.m_dPeriodMinDist)
{  

} // end SmTwistVolume::SmTwistVolume constructor

/*******************************************************************//**
PURPOSE: Virtual Copy a SmTwistVolume.

NOTES: 
***********************************************************************/
SmStatus SmTwistVolume::Copy
 (const SmContext & crContext,      // in : context for new object construction
  SmVolume       *& rpNewVolume,    // out: The copied Volume
  SmBoolean         bSimpleMapOnly) // in : TRUE = Copy this Volume omitting any compounding volumes
 const                              //      FALSE= Copy this Volumes with any compounding volumes
                                    //      default:[FALSE]
{
  rpNewVolume = new (crContext) SmTwistVolume(*this, bSimpleMapOnly);
  NER(rpNewVolume);
  return SM_SUCCESS;

} // end SmTwistVolume::Copy

/*******************************************************************//**
PURPOSE: assignment operator

NOTES:
***********************************************************************/
SmTwistVolume & SmTwistVolume::operator=
  (const SmTwistVolume &crTwistVolume)       // in : object to copy
{ 
  if(this == &crTwistVolume) return(*this) ;
  
  // assign base values
  SmVolume::operator=(crTwistVolume) ;
                                                              
  // local member values
  m_dTwistPeriodDist = crTwistVolume.m_dTwistPeriodDist ;
  m_dPeriodMinDist   = crTwistVolume.m_dPeriodMinDist ;

  // all done
  return(*this) ;

} // end SmTwistVolume::operator=

/*******************************************************************//**
PURPOSE: Equality operator for SmTwistVolume

NOTES: Call base equivalence to check type and then check 
       members for equivalence
***********************************************************************/
SmBoolean SmTwistVolume::operator==
  (const SmVolume& crOther) 
 const
{
  // low work - same objects are equal
  if(this == &crOther) { return TRUE ; }

  // low work - different types are not equal
  if(!crOther.IsKindOf(SmTwistVolume_TYPE)) 
    { return FALSE ; }

  // first check the base
  SmBoolean bRtn = SmVolume::operator ==(crOther) ;

  // then check the members
  if(bRtn)
    {
      // OK to cast
      SmTwistVolume &rOther = (SmTwistVolume &)crOther ;

      // check equivalence of these objects
      bRtn = SM_IS_ZERO(m_dTwistPeriodDist - rOther.m_dTwistPeriodDist) ;
      bRtn = SM_IS_ZERO(m_dPeriodMinDist   - rOther.m_dPeriodMinDist) ;
    }

  // all done
  return bRtn ;

} // end SmTwistVolume::operator==

/*******************************************************************//**
PURPOSE: Create a SmTwistVolume from component data.  

NOTES: 
***********************************************************************/
SmStatus SmTwistVolume::CreateCanonical
 (const SmContext & crContext,        // in : context for new object construction
  SmPoint3d       & rTwistOrigin,     // in : InSpace (and OutSpace) point on the twist center line
  SmVector3d      & rTwistAxis,       // in : InSpace (and OutSpace) direction of the twist center line marking W axis of Twist Coordinate System            
  SmVector3d      & rTwistMidDir,     // in : InSpace (and OutSpace) vector orthoganal to TwistAxis marking U axis of Twist Coordinate System
  double            dTwistPeriodDist, // in : distance along the TwistAxis for one complete Twist revolution
  SmTwistVolume   *& rpNewVolume,     // out: New SmTwistVolume, NULL on input
  double            dPeriodMinDist)   // in : min allowed value dTwistPeriodDist
                                      //      dPeriodMinDist must be >= SM_ZONE_TOL_3D
{
  // build the object
  rpNewVolume = new (crContext) SmTwistVolume(crContext, rTwistOrigin, rTwistAxis, rTwistMidDir, dTwistPeriodDist, dPeriodMinDist) ;

  // all done
  return(SM_SUCCESS) ; 

} // end SmTwistVolume::CreateCanonical

/*******************************************************************//**
PURPOSE: Get the STEP canonical data out of a TwistVolume.

NOTES: Please note that the control points are in Euclidian
    space (not homogeneous) even if the volume is rational.  That is the 
    homogeneous division has been performed on x,y,z prior to returning 
    the data in rControlPointsList.
***********************************************************************/
SmStatus SmTwistVolume::GetCanonical
 (SmPoint3d  & rTwistOrigin,      // out: InSpace (and OutSpace) point on the twist center line 
  SmVector3d & rTwistAxis,        // out: InSpace (and OutSpace) direction of the twist center line marking W axis of Twist Coordinate System                     
  SmVector3d & rTwistMidDir,      // out: InSpace (and OutSpace) vector orthoganal to TwistAxis marking U axis of Twist Coordinate System 
  double     & rdTwistPeriodDist, // out: distance along the TwistAxis for one complete Twist revolution 
  double     & rdPeriodMinDist)   // out: min allowed value dTwistPeriodDist 
 const
{
  SM_DUMP_AND_ASSERT2_VALID(this) ;

  // set outputs
  rTwistOrigin       = GetTwistOrigin() ; 
  rTwistAxis         = GetTwistAxis() ;    
  rTwistMidDir       = GetTwistMidDir() ;
  rdTwistPeriodDist  = m_dTwistPeriodDist ;
  rdPeriodMinDist    = m_dPeriodMinDist ;

  // all done
  return SM_SUCCESS;

} // end SmTwistVolume::GetCanonical

/*******************************************************************//**
PURPOSE: Set all SmTwistVolume data.

NOTES: 
***********************************************************************/
SmStatus SmTwistVolume::SetCanonical
  (SmPoint3d  & rTwistOrigin,     // in : InSpace (and OutSpace) point on the twist center line
   SmVector3d & rTwistAxis,       // in : InSpace (and OutSpace) direction of the twist center line marking W axis of Twist Coordinate System                 
   SmVector3d & rTwistMidDir,     // in : InSpace (and OutSpace) vector orthoganal to TwistAxis marking U axis of Twist Coordinate System
   double       dTwistPeriodDist, // in : distance along the TwistAxis for one complete Twist revolution
   double       dPeriodMinDist)   // in : min allowed value dTwistPeriodDist
{
  // prepare the public
  Notify(SM_NO_PRE_EDIT, this, SM_NO_GET_OWNER(this), NULL) ;

  // note: SetCanonical is called by the constructors - so don't return errors - always find a way to complete.

  // check input - dPeriodMinDist >= SM_ZONE_TOL_3D
  if(dPeriodMinDist < SM_ZONE_TOL_3D)
    { 
      SE_MSG( SM_ERR,
             _T("TwistVolume dPeriodMinDist must be >= SM_ZONE_TOL_3D - Bad Construction - changing dPeriodMinDist to SM_ZONE_TOL_3D")) ;
      dPeriodMinDist = SM_ZONE_TOL_3D ;
    }

  // check input - Positive Definite TwistPeriodDistance
  if(dTwistPeriodDist < dPeriodMinDist)
    {
      SE_MSG( SM_ERR,
             _T("TwistVolume must have a PositiveDefinite NeutralDistance - Bad Construction - changing TwistPeriodDist to dPeriodMinDist")) ;

      // do the best we can
      dTwistPeriodDist = dTwistPeriodDist >= 0.0 ? dPeriodMinDist : -dPeriodMinDist ;
    }

  // check state - unit vectors
  SmVector3d sTwistAxis         = rTwistAxis ;
  SmVector3d sTwistMidDir       = rTwistMidDir ;
  SmStatus   sTwistAxisStatus   = sTwistAxis.Unitize() ;
  SmStatus   sTwistMidDirStatus = sTwistMidDir.Unitize() ;

  // zero length TwistAxis vector
  if(sTwistAxisStatus != SM_SUCCESS)
    {
      SE_MSG(SM_ERR, _T("SmTwistVolume::SetCanonical: Zero length TwistAxis - changing to (0,0,1) ")) ;
      sTwistAxis.Set(0,0,1) ;
    }

  // zero length TwistMidDir vector
  if(sTwistMidDirStatus != SM_SUCCESS)
    {
      SE_MSG(SM_ERR, _T("SmTwistVolume::SetCanonical: Zero length TwistMidDir - changing to (1,0,0) ")) ;
      sTwistAxis.Set(1,0,0) ;
    }

  // check state - orthogonal vectors
  double dDotProd = sTwistAxis.Dot(sTwistMidDir) ;

  // non orthogonal TwistAxis/TwistMidDir vector pair
  if(!SM_IS_ZERO(dDotProd))
    {
      SE_MSG(SM_ERR, _T("SmTwistVolume::SetCanonical: TwistAxis and TwistMidDir not orthogonal - changing TwistMidDir")) ;
      sTwistMidDir = sTwistAxis * (sTwistMidDir * sTwistAxis) ;
      sTwistMidDir.Unitize() ;
    }

  // locals
  SmVector3d sTwistBinormal = rTwistAxis * rTwistMidDir ;

  // set OrientMap
  SmTransform *pOrientMap = GetOrientMap() ;
  pOrientMap->SetCanonical(rTwistOrigin, rTwistMidDir, sTwistBinormal, &rTwistAxis) ;

  // set Period distance
  m_dTwistPeriodDist = dTwistPeriodDist ; 
  
  // set MinPeriod distance
  m_dPeriodMinDist = dPeriodMinDist ;        

  // inform the public
  Notify(SM_NO_POST_EDIT, this, SM_NO_GET_OWNER(this), NULL) ;

  // all done
  return SM_SUCCESS;

} // end SmTwistVolume::SetCanonical

/*******************************************************************//**
PURPOSE: convenience function to look like a SmBSplineVolume object that
         gets effective knot lists

NOTES: for cache purposes - let UnbendVolume mascarade as a BSpline kind of shape. 
       pretend to be a degree 3, 4 control point, 8 knot BSpline basis in all directions
***********************************************************************/
SmStatus SmTwistVolume::GetKnots
  (SmVolumeParamType  eParam,        // in : one of SM_VP_U, SM_VP_V, SM_VP_W                              
   SmTArray<double> & rKnots,        // out: ParamSpace knot list                                          
   SmTArray<ULONG>  * pKnotMults,    // out: associated knot multipliticies. NULL to ignore. default:[NULL]
   const SmExtent1d * pOptIvl)       // NotUsed: in : interval of interest, NULL=Natural Interval, default:[NULL]   
  const     
{ 
  SM_REF1(pOptIvl) ;
  
  // init output
  rKnots.ReSet() ;
  if(pKnotMults) { pKnotMults->ReSet() ; } 

  // check input
  if(   eParam != SM_VP_U && eParam != SM_VP_X_IN
     && eParam != SM_VP_V && eParam != SM_VP_Y_IN
     && eParam != SM_VP_W && eParam != SM_VP_Z_IN)
    {
      SER_MSG(SM_ERR, _T("\nUnsupported SmVolumeParamType sent to SmUnbendVolume::GetKnots ")) ;
    }

  // knot vectors are the same in all param directions

  // -SM_INFINITE_PARAMETER
  if(pOptIvl == NULL || pOptIvl->ContainsValue(-SM_INFINITE_PARAMETER, SM_EFF_ZERO)) 
    { rKnots.Add(-SM_INFINITE_PARAMETER) ;
      if(pKnotMults) { pKnotMults->Add(4) ; }
    }

  // SM_INFINITE_PARAMETER
  if(pOptIvl == NULL || pOptIvl->ContainsValue(SM_INFINITE_PARAMETER, SM_EFF_ZERO)) 
    { rKnots.Add( SM_INFINITE_PARAMETER) ;
      if(pKnotMults) { pKnotMults->Add(4) ; }
    }

  // all done
  return(SM_SUCCESS) ; 

} // end SmTwistVolume::GetKnots                                          
                                                                           
/*******************************************************************//**
PURPOSE: When possible build the exact Curve produced by projecting
    rInputCurve to 1st OutSpace

NOTES: when eInputSpace == SM_VS_PARAM_SPACE: rInputCurve is mapped from ParamSpace to 1stOutSpace
       else eInputSpace == SM_VS_INSPACE    : rInputCurve is mapped from InSpace to 1stOutSpace

   the TwistVolume 'maps' 
    InputSpace curves in Planes perpendicualar to the InputSpace Twist Axis to the same curve rotated into OutSpace

  All other curve shapes and orientations are not projected to Analytic shapes and this 
  method sets rpNewCurve == NULL and returns SM_SUCCESS.
***********************************************************************/
SmStatus SmTwistVolume::MakeExactBSpline1stOutCurve
 (SmVolumeSpaceTYPE    eInputSpace,   // in : one of SM_VS_PARAM_SPACE = InputCurve is mapped from ParamSpace
                                      //             SM_VS_IN_SPACE    = InputCurve is mapped from InSpace
  const SmCurve      & rInputCurve,   // in : Target InSpace Curve
  SmBSplineCurve    *& rpNewCurve)    // out: NonNULL when exact surface is built, else NULL
 const 
{ 
  // return and init values
  SmStatus sRtn = SM_SUCCESS ;
  rpNewCurve    = NULL ;

  // check input
  if(eInputSpace != SM_VS_IN_SPACE && eInputSpace != SM_VS_PARAM_SPACE)
    { SER(SM_ERR) ; }

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  if(bDebugMe)
    {
      const SmEdge *pEdge = rInputCurve.GetEdge() ;
      const SmBrep *pBrep = pEdge ? pEdge->GetBrep() : NULL ;

      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(3,4, 0,1,0) ; rInputCurve.DrawParams() ; sm_GraphicsLoop() ;
      smgfx_SetLook(3,4, 1,0,0) ; this->Draw() ; sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

  // locals: ParamSpace and InSpace Twist orientations - Twist's Neutral plane is perp to the Twist's MidDirection
  SmVector3d sParamOrigin  (0,0,0), sTwistOrigin   = GetTwistOrigin() ;
  SmVector3d sParamMidDir  (1,0,0), sTwistMidDir   = GetTwistMidDir() ;
  SmVector3d sParamAxis    (0,0,1), sTwistAxis     = GetTwistAxis() ;
  SmVector3d sParamBinormal(0,1,0), sTwistBinormal = GetTwistBinormal() ;

  // locals: making InputSpace either ParamSpace or InSpace
  SmVector3d sInputOrigin, sInputMidDir, sInputAxis, sInputBinormal ;
  if(eInputSpace == SM_VS_IN_SPACE) { sInputOrigin   = sTwistOrigin ;
                                      sInputMidDir   = sTwistMidDir ;
                                      sInputAxis     = sTwistAxis ;   
                                      sInputBinormal = sTwistBinormal ;
                                    }
  else /* InputSpce = ParamSpace */ { sInputOrigin   = sParamOrigin ;
                                      sInputMidDir   = sParamMidDir ;
                                      sInputAxis     = sParamAxis ;   
                                      sInputBinormal = sParamBinormal ;
                                    }
  // locals 
  ULONG       ii ;
  const ULONG lSmpSize = 11 ;
  double      sSmpU ;

  SmApproxTol3d     sApproxTol = SmTol::GetApproxTol3d() ;
  const SmContext * pContext   = rInputCurve.GetContext() ;
  SmExtent1d        sIvl       = rInputCurve.GetNaturalInterval() ;
  SmVector3d        sInputPt, sOnAxisPt, sOnAxisVec ; 

  double            dAxisPlane ;        // dist from Sample Pt to planes perp to Twist Axes.
  SmExtent1d        sAxisPlaneRange ;   // range on distances
  double            dAxisPlaneSum=0.0 ; // Sum of SamplePt distances to planes
  SmBoolean         bConstAxisPlane = false ;

  // See if the InputCurve is linear and parallel to one of the TwistAxes (parallel to two TwistPlanes)
  for(ii=0;ii<=lSmpSize;ii++)
    {
      sSmpU = sIvl.Evaluate((double)ii/(double)lSmpSize) ;

      // Curve sample 
      rInputCurve.EvaluatePoint(sSmpU, sInputPt) ;

      // no work - sample point not within twist domain
      SmBoolean bIsIn =   (eInputSpace == SM_VS_IN_SPACE)
                        ? IsPointInInSpaceDomain(sInputPt) 
                        : IsPointInParamDomain(sInputPt) ;
      if(bIsIn == FALSE)
        { return SM_SUCCESS ; }
      
      // save 1st and last sample pts and pick a point on Axis near line to keep vector sizes small
      switch(ii)
        { case 0        : { SmVector3d sToOrigVec = sInputPt - sInputOrigin ;
                            sOnAxisPt  = sInputOrigin  + sInputAxis.Dot(sToOrigVec) * sInputAxis ;
                          } break ;
          default: break ;
        }

      // Input vec from SamplePt to OnAxisPt
      sOnAxisVec = sInputPt - sOnAxisPt ;

      // InputSpace distances to planes perp to Twist axes
      dAxisPlane = sInputAxis.Dot(sOnAxisVec) ;     // distance to Axis Plane

      // accumulate the range of Plane distances
      sAxisPlaneRange.AddValue(dAxisPlane) ;

      // accumulate the sum of Plane distances
      dAxisPlaneSum += dAxisPlane ;

      // remember when curve is const dist from twist plane
      bConstAxisPlane = sAxisPlaneRange.GetLength() < sApproxTol ;

      // no work - curve not const distance plane perp to TwistAxis 
      if( !bConstAxisPlane )
        { return SM_SUCCESS ; }

    } // end iter ii - gather samples for parallel test

  // arrive here when curve const dist from plane perp to TwistAxis
  SM_ASSERT_MSG(bConstAxisPlane, _T("Missed a case - this is a bug; check code.")) ;

  // compute the rotation 
  SmVector3d sToOrigVec = sOnAxisPt - sInputOrigin ;
  double     dZ         = sInputAxis.Dot(sToOrigVec) ;
  double     dAngRad    = dZ * 2*SM_PI / GetTwistPeriodDist() ;

  SmAxis2Placement sRotate ;
  sRotate.RotateAboutAxisAtPoint(dAngRad, sOnAxisPt, sInputAxis) ;

  // copy the input curve and rotate it
  rInputCurve.Copy(*pContext, (SmCurve *&) rpNewCurve) ;
  rpNewCurve->Transform(sRotate) ;

#ifdef SM_DEBUG_CODE
  if(bDebugMe)
    {
      const SmEdge *pEdge = rInputCurve.GetEdge() ;
      const SmBrep *pBrep = pEdge ? pEdge->GetBrep() : NULL ;

      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(3,4, 0,1,0) ; rInputCurve.Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook(3,4, 0,1,0) ; rInputCurve.DrawParams() ; sm_GraphicsLoop() ;
      smgfx_SetLook(5,6, 1,0,0) ; rpNewCurve->Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook(5,6, 1,0,0) ; rpNewCurve->DrawParams() ; sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

  // all done
  return(sRtn) ;

} // end SmTwistVolume::MakeExactBSpline1stOutCurve

/*******************************************************************//**
PURPOSE: When possible build the exact Surface produced by projecting
    rInputSurface to 1st OutSpace

NOTES: the TwistVolume maps 
  InputSpace surfaces in planes perp to InputSpace TwistAxis axis to the same surface rotated into OutSpace

  All other surface shapes and orientations are not projected to Analytic shapes and this 
  method sets rpNewSurface == NULL and returns SM_SUCCESS.
***********************************************************************/
SmStatus SmTwistVolume::MakeExactBSpline1stOutSurface
 (SmVolumeSpaceTYPE   eInputSpace,     // in : SM_VS_IN_SPACE   = InputSurface is projected from InSpace
                                       //      SM_VS_PARAM_SPACE= InputSurface is projected from ParamSpace
  const SmSurface   & rInputSurface,   // in : Surface to project to Last OutSpace
  SmBSplineSurface *& rpNewSurface)    // out: 1st OutSpace projection or NULL for not possible
 const 
{ 
  // return and init values
  SmStatus sRtn = SM_SUCCESS ;
  rpNewSurface  = NULL ;

  // check input
  if(eInputSpace != SM_VS_IN_SPACE && eInputSpace != SM_VS_PARAM_SPACE)
    { SER(SM_ERR) ; } 

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  if(bDebugMe)
    {
      const SmFace *pFace = (SmFace *)(rInputSurface.GetFace()) ;
      const SmBrep *pBrep = pFace ? pFace->GetBrep() : NULL ;

      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(3,4, 0,1,0) ; rInputSurface.DrawUV() ; sm_GraphicsLoop() ;
      smgfx_SetLook(3,4, 0,1,0) ; rInputSurface.DrawParams() ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,1) ; this->Draw(TRUE, TRUE) ;  sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

  // locals: ParamSpace and InSpace Twist orientations - Twist's Neutral plane is perp to the Twist's MidDirection
  SmVector3d sParamOrigin  (0,0,0), sTwistOrigin   = GetTwistOrigin() ;
  SmVector3d sParamMidDir  (1,0,0), sTwistMidDir   = GetTwistMidDir() ;
  SmVector3d sParamAxis    (0,0,1), sTwistAxis     = GetTwistAxis() ;
  SmVector3d sParamBinormal(0,1,0), sTwistBinormal = GetTwistBinormal() ;
  // double     dTwistPeriodDist = GetTwistPeriodDist() ;

  // locals: making InputSpace either ParamSpace or InSpace
  SmVector3d sInputOrigin, sInputMidDir, sInputAxis, sInputBinormal ;
  if(eInputSpace == SM_VS_IN_SPACE) { sInputOrigin   = sTwistOrigin ;
                                      sInputMidDir   = sTwistMidDir ;
                                      sInputAxis     = sTwistAxis ;   
                                      sInputBinormal = sTwistBinormal ;
                                    }
  else /* InputSpce = ParamSpace */ { sInputOrigin   = sParamOrigin ;
                                      sInputMidDir   = sParamMidDir ;
                                      sInputAxis     = sParamAxis ;   
                                      sInputBinormal = sParamBinormal ;
                                    }
  // locals 
  ULONG ii, jj, lSmpSize = 5 ;
  SmVector2d        sSmpUV ;
  SmVector3d        sInputPt, sInputVec ; 

  SmApproxTol3d     sApproxTol = SmTol::GetApproxTol3d() ;
  const SmContext * pContext = rInputSurface.GetContext() ;
  SmExtent1d        sIvlU = rInputSurface.GetNaturalUVDomain().GetUInterval() ; 
  SmExtent1d        sIvlV = rInputSurface.GetNaturalUVDomain().GetVInterval() ;

  // note: for Twist mapping Twist principle axes happen to be the same in InSpace and OutSpace
  double            dAxisPlane ;                     // InputSpace Properties
  double            dAxisPlaneSum=0.0 ;              // InputSpace Properties
  SmExtent1d        sAxisPlaneRange ;                // InputSpace Properties
  SmBoolean         bConstAxisPlane = FALSE;         // InputSpace Properties

  // See if the InputSurface is perpendicular to any of the Twist principle axes (parallel to any Twist principle planes)
  for(ii=0;ii<=lSmpSize;ii++)
    {
      sSmpUV.x = sIvlU.Evaluate((double)ii/(double)lSmpSize) ;

      for(jj=0;jj<=lSmpSize;jj++)
        {
          sSmpUV.y = sIvlV.Evaluate((double)jj/(double)lSmpSize) ;

          // Surface sample 
          rInputSurface.EvaluatePoint(sSmpUV, sInputPt) ;
          sInputVec = sInputPt - sInputOrigin ;

          // no work - sample point not within twist domain
          SmBoolean bIsIn =   (eInputSpace == SM_VS_IN_SPACE)
                            ? IsPointInInSpaceDomain(sInputPt) 
                            : IsPointInParamDomain(sInputPt) ;
          if(bIsIn == FALSE)
            { return SM_SUCCESS ; }

          // Input distance to plane containing the axis parallel to the NeutralPlane
          //      distance to Origin in Binormal direction (gives cylinder Start and Stop AngleDegs)
          //      distance to Origin in Axis direction
          dAxisPlane = sTwistAxis.Dot(sInputVec) ;     // distance to OrigPlane perpendicular to AxisPlane

          // accumulate the range of radius values
          //            the range of width values
          //            the range of height values
          sAxisPlaneRange.AddValue(dAxisPlane) ;

          // accumlate the sum of Plane distances
          dAxisPlaneSum += dAxisPlane ;

          // remember when curve is const dist from twist plane
          bConstAxisPlane = sAxisPlaneRange.GetLength()     < sApproxTol ;

          // no work - Surface not const distance from at least one plane - hack here, depends on: TRUE = 1, FALSE=0
          if(!bConstAxisPlane)
            { return SM_SUCCESS ; }

        } // end iter jj - gather sampels for parallel test
    } // end iter ii - gather samples for parallel test

  // arrive here when rInputSurface is in plane perp to TwistAxis
  SM_ASSERT_MSG(bConstAxisPlane, _T("Missed a case - this is a bug; check code.")) ;

  // compute the rotation 
  double dZ      = dAxisPlaneSum/(lSmpSize+1.0)/(lSmpSize+1.0) ;
  double dAngRad = dZ * 2*SM_PI / GetTwistPeriodDist() ;

  SmAxis2Placement sRotate ;
  sRotate.RotateAboutAxisAtPoint(dAngRad, sInputOrigin+dZ*sInputAxis, sInputAxis) ;

  // copy the input Surface and rotate it
  rInputSurface.Copy(*pContext, (SmSurface *&)rpNewSurface) ;
  rpNewSurface->Transform(sRotate) ;

#ifdef SM_DEBUG_CODE
  if(bDebugMe)
    {
      const SmFace *pFace = (SmFace *)(rInputSurface.GetFace()) ;
      const SmBrep *pBrep = pFace ? pFace->GetBrep() : NULL ;

      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(3,4, 0,1,0) ; rInputSurface.DrawUV() ; sm_GraphicsLoop() ;
      smgfx_SetLook(3,4, 0,1,0) ; rInputSurface.DrawParams() ; sm_GraphicsLoop() ;
      smgfx_SetLook(3,4, 1,0,0) ; rpNewSurface->DrawUV() ; sm_GraphicsLoop() ;
      smgfx_SetLook(3,4, 1,0,0) ; rpNewSurface->DrawParams() ; sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

  // all done
  return(sRtn) ;

} // end SmTwistVolume::MakeExactBSpline1stOutSurface

/*******************************************************************//**
PURPOSE: compute OutPoint = Evaluate(ParamPoint) - optional derived implementation.

NOTES: 
 Map UVW ParamSpace point to xyz OutSpace point and iso-derivatives.

  EvaluateSimple : d      = TwistPeriodDistance

                   a      = 2Pi * W/d
                   Dw(a)  = 2Pi/d ;
                   Dww(a) = 0
  Position
    PProj       = [xProj] = [ cos(a) * U + sin(a) * V]
                  [yProj] = [-sin(a) * U + cos(a) * V]
                  [zProj] = [ W                      ]
                
  1st Derivs
    Du(PProj)   = [ cos(a), -sin(a), 0]
    Dv(PProj)   = [ sin(a),  cos(a), 0]
    Dw(PProj)   = [2Pi/d*(-sin(a)*U +  cos(a)*V)]
                  [2Pi/d*(-cos(a)*U + -sin(a)*V)]
                  [           1                 ]
                
  2nd Derivs    
    Duu(PProj)  =  0 ;
    Duv(PProj)  =  0 ;
    Duw(PProj)  = [2Pi/d* -sin(a)]
                  [2Pi/d* -cos(a)]
                  [  0           ]
    Dvv(PProj)  = 0 ;
    Dvw(PProj)  = [2Pi/d*  cos(a)]
                  [2Pi/d* -sin(a)]
                  [  0           ]
    Dww(PProj)  = [(2Pi/d)*(2Pi/d)*(-cos(a)*U + -sin(a)*V)]
                  [(2Pi/d)*(2Pi/d)*( sin(a)*U + -cos(a)*V)]
                  [  0                                        ]
                
  3rd Derivs
    Duuu(PProj) = 0 ;
    Duuv(PProj) = 0 ;
    Duuw(PProj) = 0 ;
    Duvv(PProj) = 0 ;
    Duvw(PProj) = 0 ;
    Duww(PProj) = [(2Pi/d)*(2Pi/d)* -cos(a)]
                  [(2Pi/d)*(2Pi/d)*  sin(a)]
                  [  0                     ]
    Dvvv(PProj) = 0 ;
    Dvvw(PProj) = 0 ;
    Dvww(PProj) = [(2Pi/d)*(2Pi/d)* -sin(a)]
                  [(2Pi/d)*(2Pi/d)* -cos(a)]
                  [  0                     ]
    Dwww(PProj) = [(2Pi/d)*(2Pi/d)*(2Pi/d)*( sin(a)*U + -cos(a)*V)]
                  [(2Pi/d)*(2Pi/d)*(2Pi/d)*( cos(a)*U +  sin(a)*V)]
                  [  0                                            ]

***********************************************************************/
SmStatus SmTwistVolume::Evaluate
 (const SmPoint3d & crParamPoint,   // in : ParamSpace point to map to ProjSpace point
  ULONG             lHighestDeriv,  // in : 0-Pos Only, 1=Pos+1st Derivs, ..., max 3
  SmBoolean         bUFromLeft,     // NotUsed: in : if P is on U, V, or w interval boundary
  SmBoolean         bVFromLeft,     // NotUsed: in : TRUE  = evaluate P in upper interval where P is on the left of the interval
  SmBoolean         bWFromLeft,     // NotUsed: in : FALSE = evaluate P in lower interval where P is on the right of the interval
  SmVector3d      * aDerivatives,   // out: matrix of ParamSpace evaluations values
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
  SmBoolean bNonZeroTangents,       // NotUsed: in : TRUE = replace zero tangent vectors with properly oriented tol sized vectors
                                    //      FALSE= return exact tangent values, default:[TRUE]
                                    //      note: Surprisingly TRUE is the common choice because most tangent uses
                                    //            are for their direction (Binorm, SurfNorm comps), but when the 
                                    //            tangent is being used for its magnitude (like an arc-length comp)
                                    //            then set this to FALSE.
                                    //      default:[TRUE]               
  SmBoolean bDoZeroSampling,        // NotUsed: in : for internal use only, always set to TRUE, default:[TRUE]
  SmBoolean bWithCompounding)       // in : TRUE = Evaluate compounds, FALSE = Evaluate just this map, default:[TRUE]
 const 
{
  // stop compile warnings - reference unused arguments)
  SM_REF5(bUFromLeft, bVFromLeft, bWFromLeft, bNonZeroTangents, bDoZeroSampling) ;

  // check input - limit the number of derivatives
  if (lHighestDeriv > 3) SER(SM_ERR_INVALID_INPUT);

  // For indexing into the volume pt/deriv array:
#define sv(u,v,w) ((u*(lHighestDeriv+1)+v)*(lHighestDeriv+1)+w)

  // locals
  double dU = crParamPoint.x ;
  double dV = crParamPoint.y ;
  double dW = crParamPoint.z ;
  double dPer = 2*SM_PI/m_dTwistPeriodDist ;
  double dA   = dW * dPer ;

  double dCos = smos_Cosine(dA) ;
  double dSin = smos_Sine  (dA) ;

  SmVector3d sTwistMidDir    = GetTwistMidDir() ;
  SmVector3d sTwistBinormal  = GetTwistBinormal() ;
  SmVector3d sTwistAxis      = GetTwistAxis() ;
                                                                            
  // init output array 
  ULONG ii, dASize = (lHighestDeriv+1) * (lHighestDeriv+1) * (lHighestDeriv+1) ;
  for(ii=1;ii<dASize;ii++)
    {
      aDerivatives[ii].Set(0,0,0) ;
    }

  // position
  aDerivatives[0] =   GetTwistOrigin()
                    + ( dCos*dU + -dSin*dV) * sTwistMidDir 
                    + ( dSin*dU +  dCos*dV) * sTwistBinormal 
                    +          dW           * sTwistAxis ;

  // 1st derivs
  if(lHighestDeriv >= 1)
    {
      /* Du */ aDerivatives[sv(1,0,0)] =  dCos * sTwistMidDir + dSin * sTwistBinormal ;
      /* Dv */ aDerivatives[sv(0,1,0)] = -dSin * sTwistMidDir + dCos * sTwistBinormal ;
      /* Dw */ aDerivatives[sv(0,0,1)] =  + dPer*(-dSin*dU + -dCos*dV) * sTwistMidDir 
                                          + dPer*( dCos*dU + -dSin*dV) * sTwistBinormal 
                                          +                              sTwistAxis ;
      // NonZero 2nd derivs
      if(lHighestDeriv >= 2)
        {
          double dPer2 = dPer * dPer ;

          /* Duw */ aDerivatives[sv(1,0,1)] =  dPer * -dSin * sTwistMidDir + dPer *  dCos * sTwistBinormal ;  
          /* Dvw */ aDerivatives[sv(0,1,1)] =  dPer * -dCos * sTwistMidDir + dPer * -dSin * sTwistBinormal ;
          /* Dww */ aDerivatives[sv(0,0,2)] =    dPer2*(-dCos*dU +  dSin*dV) * sTwistMidDir  
                                               + dPer2*(-dSin*dU + -dCos*dV) * sTwistBinormal ;

          // NonZreo 3rd derivs
          if(lHighestDeriv >= 3)
            {
              double dPer3 = dPer2 * dPer ;
              /* Duww */ aDerivatives[sv(1,0,2)] =   dPer2 * -dCos * sTwistMidDir + dPer2 * -dSin * sTwistBinormal ;
              /* Dvww */ aDerivatives[sv(0,1,2)] =   dPer2 *  dSin * sTwistMidDir + dPer2 * -dCos * sTwistBinormal ;
              /* Dwww */ aDerivatives[sv(0,0,3)] =   dPer3 *( dSin*dU +  dCos*dV)* sTwistMidDir  
                                                   + dPer3 *(-dCos*dU +  dSin*dV)* sTwistBinormal ;
                                                   
            } // end 3rd derivs needed check        
        } // end 2nd derivs needed check
    } // end 1st derivs needed check

  // we don't go any higher all higher derivatives set to zero

  // done with local indexing macro
#undef sv

  // locals
  SmVector3d aNextDerivs[4*4*4] ; 

  // Map OutSpace Point through concatenated NextMaps
  if(m_pNextMap && bWithCompounding)
    {
      SER(m_pNextMap->Map(aDerivatives[0], lHighestDeriv, 
                          bUFromLeft, bVFromLeft, bWFromLeft, 
                          aNextDerivs, 
                          bNonZeroTangents, bDoZeroSampling)) ;

      // Concatenate Evaluate(ParamPoint) = Next(Mid(ParamPoint)) - when needed
      smvol_ConcatEvals(lHighestDeriv, aDerivatives, aNextDerivs, aDerivatives) ;
   
    } // end need to concatenate check

  // all done
  return SM_SUCCESS ;

} // end SmTwistVolume::Evaluate

/*******************************************************************//**
PURPOSE: compute ProjPoint = EvaluateSimple(ParamPoint) - requires derived implementation.

NOTES: 
 Map UVW ParamSpace point to xyz ProjSpace point and iso-derivatives.

  EvaluateSimple : d      = TwistPeriodDistance

                   a      = 2Pi * W/d
                   Dw(a)  = 2Pi/d ;
                   Dww(a) = 0
  Position
    PProj       = [xProj] = [ cos(a) * U + -sin(a) * V]
                  [yProj] = [ sin(a) * U +  cos(a) * V]
                  [zProj] = [ W                      ]
                
  1st Derivs
    Du(PProj)   = [ cos(a),  sin(a), 0]
    Dv(PProj)   = [-sin(a),  cos(a), 0]
    Dw(PProj)   = [2Pi/d*(-sin(a)*U + -cos(a)*V)]
                  [2Pi/d*( cos(a)*U + -sin(a)*V)]
                  [           1                 ]
                
  2nd Derivs    
    Duu(PProj)  =  0 ;
    Duv(PProj)  =  0 ;
    Duw(PProj)  = [2Pi/d* -sin(a)]
                  [2Pi/d*  cos(a)]
                  [  0           ]
    Dvv(PProj)  = 0 ;
    Dvw(PProj)  = [2Pi/d* -cos(a)]
                  [2Pi/d* -sin(a)]
                  [  0           ]
    Dww(PProj)  = [(2Pi/d)*(2Pi/d)*(-cos(a)*U +  sin(a)*V)]
                  [(2Pi/d)*(2Pi/d)*(-sin(a)*U + -cos(a)*V)]
                  [  0                                        ]
                
  3rd Derivs
    Duuu(PProj) = 0 ;
    Duuv(PProj) = 0 ;
    Duuw(PProj) = 0 ;
    Duvv(PProj) = 0 ;
    Duvw(PProj) = 0 ;
    Duww(PProj) = [(2Pi/d)*(2Pi/d)* -cos(a)]
                  [(2Pi/d)*(2Pi/d)* -sin(a)]
                  [  0                     ]
    Dvvv(PProj) = 0 ;
    Dvvw(PProj) = 0 ;
    Dvww(PProj) = [(2Pi/d)*(2Pi/d)*  sin(a)]
                  [(2Pi/d)*(2Pi/d)* -cos(a)]
                  [  0                     ]
    Dwww(PProj) = [(2Pi/d)*(2Pi/d)*(2Pi/d)*( sin(a)*U +  cos(a)*V)]
                  [(2Pi/d)*(2Pi/d)*(2Pi/d)*(-cos(a)*U +  sin(a)*V)]
                  [  0                                            ]

***********************************************************************/
SmStatus SmTwistVolume::EvaluateSimple
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

  // check input - limit the number of derivatives
  if (lHighestDeriv > 3) SER(SM_ERR_INVALID_INPUT);

  // For indexing into the volume pt/deriv array:
#define sv(u,v,w) ((u*(lHighestDeriv+1)+v)*(lHighestDeriv+1)+w)

  // locals
  double dU   = crParamPoint.x ;  // needed so &crParamPoint can be same memory as aDerivatives
  double dV   = crParamPoint.y ;  // needed so &crParamPoint can be same memory as aDerivatives
  double dW   = crParamPoint.z ;  // needed so &crParamPoint can be same memory as aDerivatives
  double dPer = 2*SM_PI/m_dTwistPeriodDist ; // negative for left hand twists
  double dA   = dW * dPer ;

  double dCos = smos_Cosine(dA) ;
  double dSin = smos_Sine  (dA) ;
                                                                            
  // init output array
  ULONG ii, dASize = (lHighestDeriv+1) * (lHighestDeriv+1) * (lHighestDeriv+1) ;
  for(ii=1;ii<dASize;ii++)
    {
      aDerivatives[ii].Set(0,0,0) ;
    }

  // position
  aDerivatives[0].Set( dCos*dU-dSin*dV,  dSin*dU+dCos*dV, dW ) ;

  // 1st derivs
  if(lHighestDeriv >= 1)
    {
      /* Du */ aDerivatives[sv(1,0,0)].Set(  dCos,  dSin, 0.0 ) ; 
      /* Dv */ aDerivatives[sv(0,1,0)].Set( -dSin,  dCos, 0.0 ) ; 
      /* Dw */ aDerivatives[sv(0,0,1)].Set(dPer*(-dSin*dU - dCos*dV),
                                           dPer*( dCos*dU - dSin*dV),
                                           1) ;  

      // 2nd derivs
      if(lHighestDeriv >= 2)
        {
          double dPer2 = dPer*dPer ;

          /* Duu */ // already set to zero: aDerivatives[sv(2,0,0)].Set(           0.0,           0.0,  0.0 ) ;  
          /* Duv */ // already set to zero: aDerivatives[sv(1,1,0)].Set(           0.0,           0.0,  0.0 ) ;
          /* Duw */                         aDerivatives[sv(1,0,1)].Set(  dPer*-dSin,  dPer*-dCos,  0.0 ) ;  
          /* Dvv */ // already set to zero: aDerivatives[sv(0,2,0)].Set(           0.0,           0.0,  0.0 ) ; 
          /* Dvw */                         aDerivatives[sv(0,1,1)].Set(  dPer*-dCos, -dPer* dSin,  0.0 ) ;
          /* Dww */                         aDerivatives[sv(0,0,2)].Set(  dPer2*(-dCos*dU +  dSin*dV),
                                                                          dPer2*(-dSin*dU + -dCos*dV),
                                                                          0) ;
          // 3rd derivs
          if(lHighestDeriv >= 3)
            {
              double dPer3 = dPer2*dPer ;

              /* Duuu */ // already set to zero: aDerivatives[sv(3,0,0)].Set(           0.0,           0.0,  0.0 ) ; 
              /* Duuv */ // already set to zero: aDerivatives[sv(2,1,0)].Set(           0.0,           0.0,  0.0 ) ; 
              /* Duuw */ // already set to zero: aDerivatives[sv(2,0,1)].Set(           0.0,           0.0,  0.0 ) ; 
              /* Duvv */ // already set to zero: aDerivatives[sv(1,2,0)].Set(           0.0,           0.0,  0.0 ) ;  
              /* Duvw */ // already set to zero: aDerivatives[sv(1,1,1)].Set(           0.0,           0.0,  0.0 ) ; 
              /* Duww */                         aDerivatives[sv(1,0,2)].Set( dPer2*-dCos,
                                                                              dPer2*-dSin,
                                                                               0) ; 
              /* Dvvv */ // already set to zero: aDerivatives[sv(0,3,0)].Set(           0.0,           0.0,  0.0 ) ;
              /* Dvvw */ // already set to zero: aDerivatives[sv(0,2,1)].Set(           0.0,           0.0,  0.0 ) ; 
              /* Dvww */                         aDerivatives[sv(0,1,2)].Set( dPer2*  dSin,
                                                                              dPer2* -dCos,
                                                                               0) ;                     
              /* Dwww */                         aDerivatives[sv(0,0,3)].Set(dPer3*( dSin*dU +  dCos*dV),
                                                                             dPer3*(-dCos*dU +  dSin*dV),
                                                                               0) ;                                            

            } // end 3rd derivs needed check        
        } // end 2nd derivs needed check
    } // end 1st derivs needed check

  // we don't go any higher all higher derivatives set to zero

  // done with local indexing macro
#undef sv

  // all done
  return SM_SUCCESS ;

} // end SmTwistVolume::EvaluateSimple

/*******************************************************************//**
PURPOSE: Compute the ProjSpace Bounding and/or Pseudo Boxes that 
   contain the projection of a given BBox from ParamSpace to ProjSpace.   
    
NOTES: 
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
SmStatus SmTwistVolume::EvaluateBoundingBoxSimple
  (const SmExtent3d & crParamBox,               // in : ParamSpace BBox to project to Project Space 
   SmPseudoBox      * pOptParamPseudoBox,       // NotUsed: in : optional ParamSpace PseudoBox used to set output PseudoBox orientations,
                                                //      NULL   : ProjPseudoBox Basis = Project X Y Z ParamVecs to ProjSpace at crParamBox Center
                                                //      NotNULL: ProjPseudoBox Basis = Project pOptParamPseudoBox BasisVecs to ProjSpace at crParamBox Center
                                                //      default:[NULL]
   SmExtent3d       * pProjBox,                 // out: ProjectSpace Axis aligned box
   SmPseudoBox      * pProjPseudoBox)           // out: ProjectSpace Non-axis aligned box
  const
{ 
  SM_REF1(pOptParamPseudoBox) ;
  // no work - no output
  if(   pProjBox == NULL
     && pProjPseudoBox == NULL)
    { return SM_SUCCESS ; }

  // locals
  SmExtent3d sProjBox ;

  // simple case - no bounds
  if(FALSE == crParamBox.AnyBounds())
    {
      // return unbounded ProjSpace boxes
      pProjBox->SetUnbounded() ;
      if(pProjPseudoBox)
        {  pProjPseudoBox->InitBasis() ;
           pProjPseudoBox->SetUnbounded() ;
        }
      
      // all done
      return(SM_SUCCESS) ;

    } // end simple case - no bounds check

  // BBox - computed directly from the properties of the ParamBox
  // locals
  ULONG ii ;
  double             dPiHalf        = SM_PI / 2 ; 
  double             dPi            = SM_PI ;
  double             d3PiHalf       = SM_PI * 3/2 ;
  double             d2Pi           = SM_PI * 2 ;
  double             dThetaMinRad   = crParamBox.GetWMin() * d2Pi / m_dTwistPeriodDist ;  
  double             dThetaMaxRad   = crParamBox.GetWMax() * d2Pi / m_dTwistPeriodDist ; 
  SmExtent1d         sThetaRangeRad(dThetaMinRad, dThetaMaxRad) ; 

  double             dRad1        = smos_Sqrt(crParamBox.GetUMin()*crParamBox.GetUMin() + crParamBox.GetVMin()*crParamBox.GetVMin()) ;
  double             dRad2        = smos_Sqrt(crParamBox.GetUMin()*crParamBox.GetUMin() + crParamBox.GetVMax()*crParamBox.GetVMax()) ;
  double             dRad3        = smos_Sqrt(crParamBox.GetUMax()*crParamBox.GetUMax() + crParamBox.GetVMax()*crParamBox.GetVMax()) ;
  double             dRad4        = smos_Sqrt(crParamBox.GetUMax()*crParamBox.GetUMax() + crParamBox.GetVMin()*crParamBox.GetVMin()) ;
  double             dAngRad1     = smos_ArcTangent2(crParamBox.GetVMin(),crParamBox.GetUMin()) ;
  double             dAngRad2     = smos_ArcTangent2(crParamBox.GetVMax(),crParamBox.GetUMin()) ;
  double             dAngRad3     = smos_ArcTangent2(crParamBox.GetVMax(),crParamBox.GetUMax()) ;
  double             dAngRad4     = smos_ArcTangent2(crParamBox.GetVMin(),crParamBox.GetUMax()) ;

  // The BBox UV square on the W=0 plane is swept in a helix from WMin to WMax sweeping through
  // associated angles AngMin and AngMax. Each corner sweeps from dAngRadCorner + AngMin to dAndRadCorner + AngMax
  //   The ProjSpace BBox Z Ext   = [WMin,WMax],
  //   The ProjSpace BBox UV Exts = BBox containing the 4 corner circular arc sweeps
  if(sThetaRangeRad.GetLength() >= d2Pi)
    {
      // Make BBox contain an axis centered circle of Max Radius
      double dMaxRad = smos_4Max(dRad1, dRad2, dRad3, dRad4) ;
      sProjBox.SetMinMax(-dMaxRad, -dMaxRad, crParamBox.GetWMin(),
                          dMaxRad,  dMaxRad, crParamBox.GetWMax()) ;
    }
  else // build a bounding box that holds the 4 swept circular arcs of the UV square 
    {
      double dThetaRad1 = 0.0, dThetaRad2 = 0.0, dRad = 0.0;
      SmVector3d sProjPt1, sProjPt2 ;

      // Add the eight circle arc end points to ProjBox.
      for(ii=0;ii<4;ii++)
        {
          switch(ii)
            {
              case 0 : dRad = dRad1 ; dThetaRad1 = dThetaMinRad + dAngRad1 ; sProjPt1.z = crParamBox.GetWMin() ;  
                                      dThetaRad2 = dThetaMaxRad + dAngRad1 ; sProjPt2.z = crParamBox.GetWMax() ; break ;
              case 1 : dRad = dRad2 ; dThetaRad1 = dThetaMinRad + dAngRad2 ;  
                                      dThetaRad2 = dThetaMaxRad + dAngRad2 ; break ;
              case 2 : dRad = dRad3 ; dThetaRad1 = dThetaMinRad + dAngRad3 ; 
                                      dThetaRad2 = dThetaMaxRad + dAngRad3 ; break ;
              case 3 : dRad = dRad4 ; dThetaRad1 = dThetaMinRad + dAngRad4 ; 
                                      dThetaRad2 = dThetaMaxRad + dAngRad4 ; break ;
            }

          // arc end points
          sProjPt1.x = dRad * smos_Cosine(dThetaRad1) ;
          sProjPt1.y = dRad * smos_Sine  (dThetaRad1) ;

          sProjPt2.x = dRad * smos_Cosine(dThetaRad2) ;
          sProjPt2.y = dRad * smos_Sine  (dThetaRad2) ;

          sProjBox.AddPoint3d(sProjPt1) ;
          sProjBox.AddPoint3d(sProjPt2) ;
                                              
          // internal arc extrema points; [-4pi -2pi]=>-2, [-2pi 0]=>-1, [0 2Pi]=>0, [2pi 4pi]=>1 
          long   iPerCnt  = (dThetaRad1 > 0) ? (long)(dThetaRad1/d2Pi) : (long)((dThetaRad1-d2Pi)/d2Pi); 
          double dZeroRad = iPerCnt * d2Pi ; 
          double dRadMin    = dThetaRad1 - dZeroRad ;   // in range:[0 2Pi] max:[2Pi]
          double dRadMax    = dThetaRad2 - dZeroRad ;   // in range:[dRad0, dRad0+2Pi] Max:[4Pi]

          if(SM_IS_CONTAINED_TO_TOL(0            , dRadMin, dRadMax, SM_EFF_ZERO_DEG)) { if(sProjBox.GetUMax() <  dRad) sProjBox.SetUMax( dRad) ; }
          if(SM_IS_CONTAINED_TO_TOL(dPiHalf      , dRadMin, dRadMax, SM_EFF_ZERO_DEG)) { if(sProjBox.GetVMax() <  dRad) sProjBox.SetVMax( dRad) ; }
          if(SM_IS_CONTAINED_TO_TOL(dPi          , dRadMin, dRadMax, SM_EFF_ZERO_DEG)) { if(sProjBox.GetUMin() > -dRad) sProjBox.SetUMin(-dRad) ; }
          if(SM_IS_CONTAINED_TO_TOL(d3PiHalf     , dRadMin, dRadMax, SM_EFF_ZERO_DEG)) { if(sProjBox.GetVMin() > -dRad) sProjBox.SetVMin(-dRad) ; }
          if(SM_IS_CONTAINED_TO_TOL(d2Pi         , dRadMin, dRadMax, SM_EFF_ZERO_DEG)) { if(sProjBox.GetUMax() <  dRad) sProjBox.SetUMax( dRad) ; }
          if(SM_IS_CONTAINED_TO_TOL(d2Pi+dPiHalf , dRadMin, dRadMax, SM_EFF_ZERO_DEG)) { if(sProjBox.GetVMax() <  dRad) sProjBox.SetVMax( dRad) ; }
          if(SM_IS_CONTAINED_TO_TOL(d2Pi+dPi     , dRadMin, dRadMax, SM_EFF_ZERO_DEG)) { if(sProjBox.GetUMin() > -dRad) sProjBox.SetUMin(-dRad) ; }
          if(SM_IS_CONTAINED_TO_TOL(d2Pi+d3PiHalf, dRadMin, dRadMax, SM_EFF_ZERO_DEG)) { if(sProjBox.GetVMin() > -dRad) sProjBox.SetVMin(-dRad) ; }
          if(SM_IS_CONTAINED_TO_TOL(d2Pi+d2Pi    , dRadMin, dRadMax, SM_EFF_ZERO_DEG)) { if(sProjBox.GetUMax() <  dRad) sProjBox.SetUMax( dRad) ; }
                                                                                                          
        } // end iter all 4 end points
    } // end build BBox to contain circular arc branch

  // we are not going to find a a PseudoBox much better than the current ProjBBox - set outputs
  if(pProjBox)       { *pProjBox       = sProjBox ; }
  if(pProjPseudoBox) { *pProjPseudoBox = sProjBox ; }

  // all done
  return(SM_SUCCESS) ;

} // end SmTwistVolume::EvaluateBoundingBoxSimple
 
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
SmStatus SmTwistVolume::EvaluateIsoParametricCurveSimple
 (const SmContext     & crContext,        // in : context for created objects
  SmVolumeParamsType    eConstantParams,  // in : oneof: SM_VPS_UV_IN,
                                          //             SM_VPS_UW_IN,
                                          //             SM_VPS_VW_IN.
  double                dIsoParam1,       // in : 1st constant SM_VP_U_IN or SM_VP_V_IN parameter value
  double                dIsoParam2,       // in : 2nd constant SM_VP_V_IN or SM_VP_W_IN parameter value
  double                d3DTolerance,     // in : Max ApproxCurve to IdealCurve deviation
  SmCurve            *& rpNewIsoCurve,    // out: the ProjSpace IsoCurve
  const SmExtent3d    * pOptParamDomain)  // in : limiting domain, NULL to ignore. default:[NULL]  
 const
{
  // init output
  rpNewIsoCurve = NULL ; 

  // gwc: there seems to be a bug in the NLib part of the CreateHelicalSweep() 
  //      For now just use the base class version

  // pass the call along
  return( SmVolume::EvaluateIsoParametricCurveSimple(crContext,       
                                                     eConstantParams, 
                                                     dIsoParam1,      
                                                     dIsoParam2,      
                                                     d3DTolerance,    
                                                     rpNewIsoCurve,   
                                                     pOptParamDomain) ) ;

// #ifdef SM_DEBUG_CODE
// SmBoolean bDebugMe = FALSE ;
// #endif // SM_DEBUG_CODE
// 
//   // locals        
//   SmExtent3d        sParamDomain = GetNaturalParamDomain() ;
//   SmApproxTol3d     sApproxTol3d = SmTol::GetApproxTol3d(&crContext) ;
// 
//   const SmExtent3d *pTgtDomain   = pOptParamDomain ? pOptParamDomain : &sParamDomain ;
// 
//   // varying u or v - return line perpendicular (radiating radially) to TwistAxis
//   if(   eConstantParams == SM_VPS_VW
//      || eConstantParams == SM_VPS_UW)
//     {
//       // locals
//       double dUMin     = eConstantParams == SM_VPS_VW ? pTgtDomain->GetUMin() : dIsoParam1 ; 
//       double dUMax     = eConstantParams == SM_VPS_VW ? pTgtDomain->GetUMax() : dIsoParam1 ; 
//       double dVMin     = eConstantParams == SM_VPS_VW ? dIsoParam1            : pTgtDomain->GetVMin() ;
//       double dVMax     = eConstantParams == SM_VPS_VW ? dIsoParam1            : pTgtDomain->GetVMax() ;
//       double dW        = eConstantParams == SM_VPS_VW ? dIsoParam2            : dIsoParam2 ;
//       double dThetaRad = 2 * SM_PI * dW / m_dTwistPeriodDist ;
// 
//       // when dV or dW is not within pOptParamDomain
//       if(   (eConstantParams == SM_VPS_VW && !pTgtDomain->GetVInterval().ContainsValue(dVMin, SM_EFF_ZERO))
//          || (eConstantParams == SM_VPS_UW && !pTgtDomain->GetUInterval().ContainsValue(dUMin, SM_EFF_ZERO))
//          || !pTgtDomain->GetWInterval().ContainsValue(dW, SM_EFF_ZERO)) 
//         {
//           // no curve to return - all done
//           return(SM_SUCCESS) ;
//         }
// 
//       double dCos = smos_Cosine(dThetaRad) ;
//       double dSin = smos_Sine(dThetaRad) ;
// 
//       SmVector3d sStartPoint( dCos * dUMin + -dSin * dVMin,
//                               dSin * dUMin +  dCos * dVMin,
//                               dW) ;           
//       SmVector3d sEndPoint  ( dCos * dUMax + -dSin * dVMax,
//                               dSin * dUMax +  dCos * dVMax,
//                               dW) ;
// 
//       // cull degenerate curves
//       if(sStartPoint.DistanceBetweenSquared(sEndPoint) < SM_EFF_ZERO_SQ)
//         {
//           // return degenerate curve
//           SmCurve::CreateDegenerateCurve(crContext, 3, sStartPoint, rpNewIsoCurve) ; 
//           return(SM_SUCCESS) ;
//         }
// 
// #ifdef SM_DEBUG_CODE
//       if(bDebugMe)
//         {
//           SmVector3d sParamStartPoint(dUMin, dVMin, dW) ;
//           SmVector3d sParamEndPoint(dUMax, dVMax, dW) ; 
// 
//           SmVector3d sOrigin = GetTwistOrigin();
// 
//           smgfx_Erase() ;
//           smgfx_SetLook(2,3, 1,0,0) ; (10.0 * GetTwistAxis()).Draw(&sOrigin) ; sm_GraphicsLoop() ;
//           smgfx_SetLook(2,3, 0,0,1) ; GetTwistMidDir().Draw(&sOrigin) ; sm_GraphicsLoop() ;
//           smgfx_SetLook(2,3, 0,1,0) ; GetTwistBinormal().Draw(&sOrigin) ; sm_GraphicsLoop() ;
// 
//           smgfx_SetLook(2,4, 0,0,1) ; sParamStartPoint.Draw() ; sm_GraphicsLoop() ;
//           smgfx_SetLook(2,4, 1,0,0) ; sParamEndPoint.Draw() ; sm_GraphicsLoop() ;
//           smgfx_SetLook(2,3, 0,1,0) ; (sParamEndPoint-sParamStartPoint).Draw(&sParamStartPoint) ; sm_GraphicsLoop() ;
// 
//           smgfx_SetLook(2,6, 0,.3,1) ; sStartPoint.Draw() ; sm_GraphicsLoop() ;
//           smgfx_SetLook(2,6, 1,.3,0) ; sEndPoint.Draw() ; sm_GraphicsLoop() ;
//           smgfx_SetLook(3,4, .3,1,0) ; (sEndPoint-sStartPoint).Draw(&sStartPoint) ; sm_GraphicsLoop() ;
//           sm_GraphicsLoop() ;
//         }
// #endif // SM_DEBUG_CODE
// 
// // create IsoLine
//       rpNewIsoCurve = new (crContext) SmLine(sStartPoint,       // in : Start of line = Start + s*(End-Start), for s:[0 1]
//                                              sEndPoint,         // in : End   of line = Start + s*(End-Start), for s:[0 1]
//                                                                 //      will be unitized before storing
//                                              3,                 // in : sizeof LinePoint and LineVector, default:[3]
//                                             &crContext) ;       // in : 
//                                       
//     } // end varying u return line radiating radially from TwistAxis branch
// 
//   // varying w - return helix centered on TwistAxis
//   else if(eConstantParams == SM_VPS_UV)
//     {
//       double dU           = dIsoParam1 ;
//       double dV           = dIsoParam2 ;
//       double dWMin        = pTgtDomain->GetWMin() ;
//       double dWMax        = pTgtDomain->GetWMax() ;
//                           
//       double dRadius      = smos_Sqrt(dU*dU + dV*dV) ;
//       double dPer         = 2 * SM_PI / m_dTwistPeriodDist ;
//       double dThetaRadMin = dPer * dWMin ;
//       // double dThetaRadMax = dPer * dWMax ;
// 
//       double dCos = smos_Cosine(dThetaRadMin) ;
//       double dSin = smos_Sine(dThetaRadMin) ;
// 
//       // CreateHelixSegment() RefFrame placed at Helix start with XAxis pointing to Helix start point
//       double     dHeight   = dWMax - dWMin ;
//       double     dNumTurns = dHeight / dPer ;
//       SmVector3d sRefO( 0,    0, dWMin) ;
//       SmVector3d sRefX( dCos, dSin,  0) ;  // Vec to Helix start at dThetaRadMin
//       SmVector3d sRefY(-dSin, dCos,  0) ;  // Vec perp to Vec to Helix start at dThetaRadMin
// 
//       // cull degenerate curves
//       if(dHeight < SM_EFF_ZERO)
//         {
//            
//           // return degenerate curve
//           SmCurve::CreateDegenerateCurve(crContext, 3, sRefO+dRadius*sRefX, rpNewIsoCurve) ; 
//           return(SM_SUCCESS) ;
//         }
// 
//       // create the helix
//       SmBSplineCurve  * pNewBSplineCurve = NULL ;
//       SmAxis2Placement  sRefFrame(sRefO, sRefX, sRefY) ;
// 
//       SmBSplineCurve::CreateHelixSegment
//         (crContext,           // in : context for new object construction
//          sRefFrame,           // in : Helix centered on Z Axis starting at Z=0 on X Axis, running to Z=Height
//          dHeight,             // in : Helix Length along RefFrame Z Axis, Height:[0=build spiral,>0=build helix]
//          dRadius,             // in : Helix Radius at RefFrame Z = 0      (linearly interpolated from 0 to Height)
//          dRadius,             // in : Helix Radius at RefFrame Z = Height (linearly interpolated from 0 to Height)
//          smos_Fabs(dPer),     // in : Helix Length to twist 360 degrees, PeriodCnt = dHeight/dPeriodLength
//          dNumTurns,           // in : Num of 360 deg Helix Turns in H=[0,Height] (Fractions okay), PeriodLength:[Height/NumTurns]
//          sApproxTol3d,        // in : Max allowed deviation between NewBSplineCurve and theoretical Helix
//          pNewBSplineCurve) ;  // out: The new Helix curve.
// 
//       rpNewIsoCurve = (SmCurve *)pNewBSplineCurve ; 
// 
//     } // end varying w return a line parallel to the TwistAxis branch
// 
//   else
//     {
//       SER_MSG(SM_ERR, _T("Bad Input eConstantParams value")) ; 
//     }
//       
// #ifdef SM_DEBUG_CODE
//   if(bDebugMe)
//     {
//       SmVector3d sOrigin = GetTwistOrigin();
// 
//       smgfx_Erase() ;
//       smgfx_SetLook(2,3, 1,0,0) ; (10.0 *GetTwistAxis()).Draw(&sOrigin) ; sm_GraphicsLoop() ;
//       smgfx_SetLook(2,3, 0,0,1) ; GetTwistMidDir().Draw(&sOrigin) ; sm_GraphicsLoop() ;
//       smgfx_SetLook(2,3, 0,1,0) ; GetTwistBinormal().Draw(&sOrigin) ; sm_GraphicsLoop() ;
//       smgfx_SetLook(1,2, 0,0,1) ; if(rpNewIsoCurve) rpNewIsoCurve->Draw() ; sm_GraphicsLoop() ;
//       sm_GraphicsLoop() ;
//     }
// #endif // SM_DEBUG_CODE
// 
//   // all done
//   return SM_SUCCESS ;

} // end SmTwistVolume::EvaluateIsoParametricCurveSimple

/*******************************************************************//**
PURPOSE: Create ProjSpace IsoSurface from a ParamSpace IsoParamPlane

NOTES: Specify IsoPlane as a constant param value as:
     +----------------+---------------+--------------------+
     | eConstantParam | dIsoParam     | varying parameters |
     +----------------+---------------+--------------------+
     |   SM_VP_U      | constant u    | varying v, w       |
     |   SM_VP_V      | constant v    | varying u, w       |
     |   SM_VP_W      | constant w    | varying u, v       |
     +----------------+---------------+--------------------+

    for SmTwistVolume - Make a BSpline Surface  (Approximate)
    Use SmVolume::EvaluateIsoParametricSurfaceSimple() to make a SmSrfInVolume (exact)
***********************************************************************/
SmStatus SmTwistVolume::EvaluateIsoParametricSurfaceSimple
 (const SmContext   & crContext,        // in : context for created objects
  SmVolumeParamType   eConstantParam,   // in : oneof: SM_VP_U_IN,
                                        //             SM_VP_V_IN,
                                        //             SM_VP_W_IN
  double              dIsoParam,        // in : constant param value
  double              d3DTolerance,     // in : Max ApproxSurface to IdealSurface deviation        
  SmSurface        *& rpNewIsoSurface,  // out: the ProjSpace IsoSurface                        
  const SmExtent3d  * pOptParamDomain)  // in : limiting domain, NULL to ignore. default:[NULL]
const
{
  // init output
  rpNewIsoSurface = NULL ;

  // gwc: there seems to be a bug in the NLib part of the CreateHelicalSweep() 
  //      For now just use the base class version

  // pass the call along
  return( SmVolume::EvaluateIsoParametricSurfaceSimple(crContext,       
                                                       eConstantParam,  
                                                       dIsoParam,       
                                                       d3DTolerance,    
                                                       rpNewIsoSurface, 
                                                       pOptParamDomain) ) ;

// #ifdef SM_DEBUG_CODE
// SmBoolean bDebugMe = FALSE ;
// #endif // SM_DEBUG_CODE
// 
//   // locals
//   SmVector3d        sParamStart, sParamMid, sParamEnd, sParamBot, sParamTop ;
//   SmVector3d        sProjStart,  sProjMid,  sProjEnd,  sProjBot,  sProjTop ;
//   SmExtent3d        sParamDomain = GetNaturalParamDomain() ;
//   const SmExtent3d *pTgtDomain   = pOptParamDomain ? pOptParamDomain : &sParamDomain ;
// 
//   // constant u or constant v - return Helical Sweep of Line in WMin plane
//   if( eConstantParam == SM_VP_U || eConstantParam == SM_VP_V)
//     {
//       double dUMin = eConstantParam == SM_VP_U ? dIsoParam             : pTgtDomain->GetUMin() ; 
//       double dUMax = eConstantParam == SM_VP_U ? dIsoParam             : pTgtDomain->GetUMax() ;
//       double dVMin = eConstantParam == SM_VP_U ? pTgtDomain->GetVMin() : dIsoParam ;
//       double dVMax = eConstantParam == SM_VP_U ? pTgtDomain->GetVMax() : dIsoParam ;     
//       double dWMin = pTgtDomain->GetWMin() ;     
//       double dWMax = pTgtDomain->GetWMax() ; 
//       SmExtent1d sGenLineIvl(eConstantParam == SM_VP_U ? dVMin : dUMin,
//                              eConstantParam == SM_VP_U ? dVMax : dUMax) ;
// 
//       double dThetaMinRad = 2 * SM_PI * dWMin / m_dPeriodMinDist ;
//       double dCos         = smos_Cosine(dThetaMinRad) ;
//       double dSin         = smos_Sine(dThetaMinRad) ;
// 
//       // define ProjSpace RefFrame on Helix center line at dWMin (rotate X and Y axes by dThetaMinRad)
//       SmVector3d sOrigin = GetTwistOrigin() + dWMin * GetTwistAxis() ;
//       SmVector3d sXAxis  = dCos * GetTwistMidDir() + -dSin * GetTwistBinormal() ;
//       SmVector3d sYAxis  = dSin * GetTwistMidDir() +  dCos * GetTwistBinormal() ;
//       SmAxis2Placement sRefFrame(sOrigin, sXAxis, sYAxis) ;
// 
//       // Param (on W=0 plane and rotated by dThetaMinRad) and Proj Space Line End and Mid pts
//       sParamStart.Set(dCos * dUMin + -dSin * dVMin, dSin * dUMin + dCos * dVMin, 0.0) ;
//       sParamEnd.  Set(dCos * dUMax + -dSin * dVMax, dSin * dUMax + dCos * dVMax, 0.0) ;
//       sParamMid = (sParamStart + sParamEnd) / 2.0 ; 
//       sParamBot.Set(sParamStart.x, sParamStart.y, dWMin) ;
//       sParamTop.Set(sParamEnd.x,   sParamEnd.y,   dWMax) ;
// 
//       EvaluatePointSimple(sParamStart, sProjStart) ;
//       EvaluatePointSimple(sParamMid,   sProjMid) ;
//       EvaluatePointSimple(sParamEnd,   sProjEnd) ;
//       EvaluatePointSimple(sParamBot,   sProjBot) ;
//       EvaluatePointSimple(sParamTop,   sProjTop) ;
// 
//       // cull degenerate GenCurves
//       if(   sProjStart.DistanceBetweenSquared(sProjEnd) <= SM_EFF_ZERO_SQ
//          && sProjMid.  DistanceBetweenSquared(sProjEnd) <= SM_EFF_ZERO_SQ)
//         {
//           // no surface to return - all done
//           return(SM_SUCCESS) ;
//         }
// 
//       // cull degenerate sweep distances
//       if(sProjBot.DistanceBetweenSquared(sProjTop) <= SM_EFF_ZERO_SQ)   
//         {
//           // no surface to return - all done
//           return(SM_SUCCESS) ;
//         }
// 
//       // twist locals
//       double             dRadius      = smos_Sqrt(dUMin*dUMin + dVMin*dVMin) ;
//       double             dHeight      = dWMax - dWMin ;
//       double             dNumTurns    = dHeight / m_dPeriodMinDist ;
//       double             dScale       = 1.0 ;
//       SmBoolean          bRightHanded = m_dTwistPeriodDist >= 0.0 ;
//       SmApproxTol3d      sApproxTol3d = SmTol::GetApproxTol3d(&crContext) ;
//       SmBSplineSurface * pNewBSplineSurface = NULL ;
// 
//       // GenLine params set so that
//       //   ParamStart = LinePoint * sGenLineIvl.GetMin() * LinVec ;
//       //   ParamEnd   = LinePoint * sGenLineIvl.GetMax() * LinVec ;
//       SmVector3d sGenLineVector = sParamEnd - sParamStart ;
//       sGenLineVector.Unitize() ;
//       SmVector3d sGenLinePoint = sParamStart - sGenLineIvl.GetMin() * sGenLineVector ;
//       
//       // build properly parameterized line (on W=dMinW plane and rotated by dThetaMinRad)
//       SmLine * pGenLine = new (crContext) SmLine(sGenLinePoint, sGenLineVector, sGenLineIvl, dScale, 3, &crContext) ; 
//       
// #ifdef SM_DEBUG_CODE
//       if(bDebugMe)
//         {
//           sOrigin = GetTwistOrigin();
//           
//           smgfx_Erase() ;
//           smgfx_SetLook(2,3, 1,0,0) ; (10.0 *GetTwistAxis()).Draw(&sOrigin) ; sm_GraphicsLoop() ;
//           smgfx_SetLook(2,3, 0,0,1) ; GetTwistMidDir().Draw(&sOrigin) ; sm_GraphicsLoop() ;
//           smgfx_SetLook(2,3, 0,1,0) ; GetTwistBinormal().Draw(&sOrigin) ; sm_GraphicsLoop() ;
//           smgfx_SetLook(2,6, 0,0,1) ; if(pGenLine) pGenLine->Draw(NULL, TRUE) ; sm_GraphicsLoop() ;
//           sm_GraphicsLoop() ;
//         }
// #endif // SM_DEBUG_CODE
// 
//       // set output = helically swept line
//       SmBSplineSurface:: CreateHelicalSweep
//         (crContext,            // in : context for new object construction
//          sRefFrame,            // in : Helix centered on Z Axis starting at Z=0 on X Axis, running to Z=Height
//          *pGenLine,            // in : Planar BSplineCurve built in the RefFrame's Z=0 plane
//          dHeight,              // in : Helix Length along RefFrame Z Axis
//          dRadius,              // in : Helix Radius at RefFrame Z = 0      (linearly interpolated from 0 to Height)
//          dRadius,              // in : Helix Radius at RefFrame Z = Height (linearly interpolated from 0 to Height)
//          dNumTurns,            // in : Num of 360 deg Helix Turns in H=[0,Height] (Fractions okay), PeriodLength:[Height/NumTurns]
//          bRightHanded,         // in : TRUE = spiral is right handed, FALSE=left handed
//          sApproxTol3d,         // in : Max allowed deviation between NewBSplineCurve and theoretical Helix
//          pNewBSplineSurface) ; // out: The new Surface = Helical Sweep of input Surface.
// 
//        rpNewIsoSurface = pNewBSplineSurface ; 
// 
//     } // end constant u return cylinder branch
// 
//   // constant w - return rotated plane
//   else if(eConstantParam == SM_VP_W)
//     {
//       double dUMin = pTgtDomain->GetUMin() ;
//       double dUMax = pTgtDomain->GetUMax() ;
//       double dVMin = pTgtDomain->GetVMin() ;
//       double dVMax = pTgtDomain->GetVMax() ;     
//       double dW    = dIsoParam ; 
// 
//       // Output coordinate system
//       double dThetaRad = 2 * SM_PI * dW / m_dTwistPeriodDist ;
//       double dCos      = smos_Cosine(dThetaRad) ; 
//       double dSin      = smos_Sine(dThetaRad) ;
//       SmVector2d sUVScale(1,1) ;
//       SmExtent2d sUVDomain(dUMin, dVMin, dUMax, dVMax) ;
//       
//       // plane rotated to Z=dW plane
//       SmVector3d sOrig  =  GetTwistOrigin() + dW * GetTwistAxis() ;
//       SmVector3d sXAxis =  dCos * GetTwistMidDir() + -dSin * GetTwistBinormal() ;
//       SmVector3d sYAxis =  dSin * GetTwistMidDir() +  dCos * GetTwistBinormal() ;
// 
//       // cull degenerate GenCurves
//       if(sUVDomain.IsDegenerate())
//         {
//           // no surface to return - all done
//           return(SM_SUCCESS) ;
//         }
// 
//       // set output = plane moved to Z=dW plane and rotated by dThetaRad
//       rpNewIsoSurface = new (crContext) SmPlane(sOrig, sXAxis, sYAxis, sUVScale, sUVDomain, &crContext) ;
// 
//     } // end constant w return disc branch
// 
//   // default - bad eConstantParam value - inform the public
//   else
//     {
//       SER_MSG(SM_ERR, _T("Bad Input eConstantParam value")) ; 
//     }
//       
// #ifdef SM_DEBUG_CODE
//   if(bDebugMe)
//     {
//       SmVector3d sOrigin = GetTwistOrigin();
//       
//       smgfx_Erase() ;
//       smgfx_SetLook(2,3, 1,0,0) ; (10.0 *GetTwistAxis()).Draw(&sOrigin) ; sm_GraphicsLoop() ;
//       smgfx_SetLook(2,3, 0,0,1) ; GetTwistMidDir().Draw(&sOrigin) ; sm_GraphicsLoop() ;
//       smgfx_SetLook(2,3, 0,1,0) ; GetTwistBinormal().Draw(&sOrigin) ; sm_GraphicsLoop() ;
//       smgfx_SetLook(1,2, 0,0,1) ; if(rpNewIsoSurface) rpNewIsoSurface->Draw(TRUE) ; sm_GraphicsLoop() ;
//       sm_GraphicsLoop() ;
//     }
// #endif // SM_DEBUG_CODE
// 
//   // all done
//   return SM_SUCCESS;

} // end SmTwistVolume::EvaluateIsoParametricSurfaceSimple

/*******************************************************************//**
PURPOSE: Do a quick approximate inverse mapping from ProjSpace back 
         to ParamSpace for upcoming newton raphson

NOTES:  just do the inverse mapping - no guessing

RETURNS: SM_SUCCESS.
***********************************************************************/
SmStatus SmTwistVolume::InvEvaluateGuessPointSimple
 (const SmPoint3d     & crProjPoint,        // in : ProjSpace Point to map back to ParamSpace Point
  SmTArray<SmPoint3d> & rGuessParamPoints)  // out: ParamSpace Point near Target Point actual map back to ParamSpace
 const
{
  // init output
  rGuessParamPoints.SetSize( 1 ) ;

  // compute the inverse mapping rotation angle
  double dThetaRad = -2 * SM_PI * crProjPoint.z / m_dTwistPeriodDist ; // neg sign = rotate from ProjSpace to ParamSpace

  double dCos      = smos_Cosine(dThetaRad) ; 
  double dSin      = smos_Sine(dThetaRad) ;

  // set output - same as positive twist mapping but with the dThteaAng negated
  rGuessParamPoints[0].x =  dCos * crProjPoint.x + -dSin * crProjPoint.y ;        
  rGuessParamPoints[0].y =  dSin * crProjPoint.x +  dCos * crProjPoint.y ;
  rGuessParamPoints[0].z =  crProjPoint.z ;

  // all done
  return( SM_SUCCESS ) ;

} // end SmTwistVolume::InvEvaluateGuessPointSimple

/*******************************************************************//**
PURPOSE: inverse map a point from ProjSpace back to ParamSpace

NOTES: Transform is invertible where its not degenerate - just do the inverse mapping - no guessing,
       Map XYZ ProjSpace point back to UVW ParamSpace.
***********************************************************************/
SmStatus SmTwistVolume::GlobalPointSolveSimple // eff: Find volume UVW Point that maps to TargetPoint
 (const SmPoint3d  & crProjPoint,          // in : ProjSpace Point to map back to ParamSpace
  SmBoolean        & rbFoundAnswer,        // out: TRUE = successfully mapped ProjSpace Point to a ParamSpace Point
  SmSolutionArray  & rSolutions,           // out: Contains ParamSpace Found Point
                                           //      rSolution[i].m_eSolutionType           = SM_ST_SINGLE_VALUE                                               
                                           //      rSolution[i].m_vStart.m_dSolutionValue = PointOnVolume.DistanceBetween(crProjPoint);                      
                                           //      rSolution[i].m_vStart[0] =  U of [U,V,W] the found ParamSpace point location                    
                                           //      rSolution[i].m_vStart[1] =  V of [U,V,W] the found ParamSpace point location                    
                                           //      rSolution[i].m_vStart[2] =  W of [U,V,W] the found ParamSpace point location                    
  const SmExtent3d * pOptParamDomain)      // in : ParamSpace domain over which to search for the inverse point
 const                                     //      NULL = use Map's NaturalDomain, default:[NULL]
{
  // init output
  rSolutions.ReSet() ;
  rbFoundAnswer = FALSE ;

  // locals
  SmTArray<SmVector3d> sParamPoints ;

  // inverse mapping - InvEvaluateGuessPointSimple is exact
  InvEvaluateGuessPointSimple(crProjPoint, sParamPoints) ;

  // set output
  if(   pOptParamDomain == NULL
     || pOptParamDomain->ContainsPoint3d(sParamPoints[0]))
    {
      rbFoundAnswer                            = TRUE ;
      rSolutions.SetSize(1) ; 

      rSolutions[0].m_eSolutionType            = SM_ST_SINGLE_VALUE ;
      rSolutions[0].m_lNumVariables            = 3 ;
      rSolutions[0].m_vStart.m_dSolutionValue  = 0.0 ;
      rSolutions[0].m_vStart.m_adParameters[0] = sParamPoints[0].x ;
      rSolutions[0].m_vStart.m_adParameters[1] = sParamPoints[0].y ;
      rSolutions[0].m_vStart.m_adParameters[2] = sParamPoints[0].z ;
      rSolutions[0].m_lNumObjects              = 1 ;
      rSolutions[0].m_apObjects[0]             = (SmObject *)this ;
    }

  // all done
  return(SM_SUCCESS) ;  

} // end SmTwistVolume::GlobalPointSolveSimple

/*******************************************************************//**
PURPOSE: inverse map a point from ProjSpace back to ParmSpace with a guess point

NOTES: Transform is invertible except at degeneracy - just do the inverse mapping - no guessing,
       Map XYZ OutSpace point back to UVW InSpace.
***********************************************************************/
SmStatus SmTwistVolume::LocalPointSolveSimple
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
  SM_REF1(crParamPointGuess) ;
  // init output
  rbFoundAnswer = FALSE ;
  rSolution.ReSet() ;

  // locals
  SmTArray<SmVector3d> sParamPoints ;

  // inverse mapping - InvEvaluateGuessPointSimple is exact
  InvEvaluateGuessPointSimple(crProjPoint, sParamPoints) ;

  // set output
  if(   pOptParamDomain == NULL
     || pOptParamDomain->ContainsPoint3d(sParamPoints[0]))
    {
      rbFoundAnswer                        = TRUE ;

      rSolution.m_eSolutionType            = SM_ST_SINGLE_VALUE ;
      rSolution.m_lNumVariables            = 3 ;
      rSolution.m_vStart.m_dSolutionValue  = 0.0 ;
      rSolution.m_vStart.m_adParameters[0] = sParamPoints[0].x ;
      rSolution.m_vStart.m_adParameters[1] = sParamPoints[0].y ;
      rSolution.m_vStart.m_adParameters[2] = sParamPoints[0].z ;
      rSolution.m_lNumObjects              = 1 ;
      rSolution.m_apObjects[0]             = (SmObject *)this ;
    }

  // all done
  return(SM_SUCCESS) ;  

} // end SmTwistVolume::LocalPointSolveSimple

/*******************************************************************//**
PURPOSE: return TRUE if any part of the natural param domain is bounded

NOTES: Twist is defined on the Positive halfspace from the TwistOrigin to
       the TwistMidDir direction. In TwistOrigined ABC coordinates
       that's the halfspace defined by a > 0.
***********************************************************************/
SmBoolean SmTwistVolume::IsBoundedSimple() const           
{
  // all done
  return(FALSE) ;

} // end SmTwistVolume::IsBoundedSimple    

/*******************************************************************//**
PURPOSE:  

NOTES: TwistVolumes are periodic over v = [-d*Pi, +d*Pi ]
       d = m_vTwistNeturalDistance
***********************************************************************/
SmBoolean SmTwistVolume::IsClosedSimple
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

} // end SmTwistVolume::IsClosedSimple    

/*******************************************************************//**
PURPOSE: 

NOTES:  TwistVolumes are periodic over v = [-d*Pi, +d*Pi ]
       d = m_vTwistNeturalDistance
***********************************************************************/
SmBoolean SmTwistVolume::IsPeriodicSimple
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

} // end SmTwistVolume::IsPeriodicSimple 

/*******************************************************************//**
PURPOSE: 

NOTES: degenerate mapping for a = 0 
        where a = distance from TwistOrigin in TwistMidDir direction
***********************************************************************/   
SmBoolean SmTwistVolume::IsSingularitySimple      
  (const SmPoint3d   & crParamPoint,       // NotUsed: in : UVpoint to test                 
   SmBoolean         & rbSingularU,        // out: TRUE = [Su=0]
   SmBoolean         & rbSingularV,        // out: TRUE = [Sv=0]
   SmBoolean         & rbSingularW,        // out: TRUE = [Sw=0]
   double              d3dTol)             // NotUsed: in : min 3d distance between distinct points) const; 
  const 
{
  SM_REF2(crParamPoint, d3dTol) ;
  // locals
  SmBoolean bRtn = FALSE ; 

  rbSingularU = FALSE ;
  rbSingularV = FALSE ;
  rbSingularW = FALSE ;

  // all done
  return(bRtn) ;

} // end SmTwistVolume::IsSingularitySimple    

/*******************************************************************//**
PURPOSE: 

NOTES: This may have to be rethought.  Currently Twist mappings are half spaces.
       But they have a degeneracy along the a = 0 plane and are periodic
       in v over the block = [-d*Pi, +d*Pi].
***********************************************************************/
SmBoolean SmTwistVolume::IsOnBoundarySimple
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
  return(rbOnU || rbOnV || rbOnW) ;

} // end SmTwistVolume::IsOnBoundarySimple    

/*******************************************************************//**
PURPOSE: return TRUE when crParamPoint can be uniquely mapped to ProjSpace
         from the PrimaryPeriod

NOTES: The SmTwistVolume's Natural Param Space includes the portion of the
       positive half-space, u > 0, that is within the mapping primary's 
       period.
***********************************************************************/
SmBoolean SmTwistVolume::IsPointInParamDomainSimple
 (const SmPoint3d  & crParamPoint)   // NotUsed: in : ParamSpace point to test for inclusion                                  
      const                                
{
  SM_REF1(crParamPoint) ;
  // all done
  return(TRUE) ;

} // end SmTwistVolume::IsPointInParamDomainSimple

/*******************************************************************//**
PURPOSE: return TRUE when crProjPoint can be uniquely mapped back to ParamSpace

NOTES: The SmTwistVolume can only invert points within the positive half-space
       u > 0.
***********************************************************************/
SmBoolean SmTwistVolume::IsPointInProjDomainSimple
 (const SmPoint3d  & crProjPoint,          // in : project space point to test for inversion                                  
  SmPoint3d        * pOptParamPoint)       // out: if the point is invertible, go ahead and get the inverse
      const                                
{ 
  // return the inverse mapping
  //   Position
  //     [U] = [ cos(a) * xProj +  sin(a) * yProj]
  //     [V] = [-sin(a) * xProj +  cos(a) * yProj]
  //     [W] = [ zProj                           ]
  if(pOptParamPoint)
    {
      SmTArray<SmVector3d> sParamPoints ; 

      // inverse mapping - InvEvaluateGuessPointSimple is exact
      InvEvaluateGuessPointSimple(crProjPoint, sParamPoints) ;

      // set output
      *pOptParamPoint = sParamPoints[0] ;

    } // end asked for ParamPoint check

  // all done
  return( TRUE ) ;

} // end SmTwistVolume::IsPointInProjDomainSimple

/*******************************************************************//**
PURPOSE: return TRUE when ParamLine maps completely within the
       Natural Domain and does not cross any discontinuities.

NOTES: The SmTwistVolume's Natural Param Space includes the portion of the
       positive half-space, u > 0, that is within the mapping primary's 
       period.

       TwistVolume ParamDomain is convex and contains no internal dicontinutities
       so the Line is 'in' when both EndPoints are 'in'
***********************************************************************/
SmBoolean SmTwistVolume::IsLineInParamDomainSimple
 (const SmPoint3d  & crStartParamPoint,       // NotUsed: in : ParamSpace Line StartPoint to test for inclusion                                  
  const SmPoint3d  & crEndParamPoint)         // NotUsed: in : ParamSpace Line EndPoint to test for inclusion
 const                                
{
  SM_REF2(crStartParamPoint, crEndParamPoint) ;
  // ParamLine is in when both EndParamPoints are in

  // all done
  return(TRUE) ;

} // end SmTwistVolume::IsLineInParamDomainSimple

/*******************************************************************//**
PURPOSE: Return TRUE when Curve does not cross any volume's boundaries or
         internal C1 discontinuities and is contained within the ParamSpace domain

NOTES: The TwistVolume mapping is limited to the half space U > m_dPeriodMinDist
       and to one period (2Pi * m_dPeriodMinDist) in the V direction. That
       period can be centered on any V = const plane.  For now,
       this function assumes the period of interest is centered on the V = 0 plane.

ASSUMPTIONS: assumes the V direction period is centered on V=0 plane when
       in reality it can be centered on any V=const plane.
***********************************************************************/
SmBoolean SmTwistVolume::IsCurveInParamDomainSimple
 (const SmCurve & crParamCurve)       // NotUsed: in : Tgt ParamSpace Curve to check
 const       
{ 
  SM_REF1(crParamCurve) ;
  // crParamCurve is in when all ParamCurvePoints are in

  // all done
  return(TRUE) ; 

 } // end SmTwistVolume::IsCurveInParamDomainSimple

/*******************************************************************//**
PURPOSE: Return TRUE when Surface does not cross any volume's boundaries or
         internal C1 discontinuities and is contained within the ParamSpace domain

NOTES: The TwistVolume mapping is limited to the half space U > m_dPeriodMinDist
       and to one period (2Pi * m_dPeriodMinDist) in the V direction. That
       period can be centered on any V = const plane.  For now,
       this function assumes the period of interest is centered on the V = 0 plane.

ASSUMPTIONS: assumes the V direction period is centered on V=0 plane when
       in reality it can be centered on any V=const plane.
***********************************************************************/
SmBoolean SmTwistVolume::IsSurfaceInParamDomainSimple
 (const SmSurface & crParamSurface)   // NotUsed: in : Tgt ParamSpace Surface to check
 const 
{ 
  SM_REF1(crParamSurface) ;
  // crParamSurface is in when all ParamSurfacePoints are in

  // all done
  return(TRUE) ; 

} // end SmTwistVolume::IsSurfaceInParamDomainSimple

/*******************************************************************//**
PURPOSE: Write SmTransform to given output stream.

NOTES: 
***********************************************************************/
SmStatus SmTwistVolume::WriteToDB
 (SmDatabaseIO & rDB,              // in : target output stream
  ULONG          lDBVersionNumber) // in : database version to get proper sequence of writes                                                                       
 const
{
  // file type, ASCII or BINARY
  SmFileType eType = rDB.GetFileType();

  // Volume locals
  //   stored in SmVolume::OrientMap    double *pC = (double *)&GetTwistOrigin() ; 
  //                                    double *pA = (double *)&GetTwistAxis() ; 
  //                                    double *pS = (double *)&GetTwistMidDir() ; 
  //                                    double *pB = (double *)&GetTwistBinormal() ;

  // Create the output file 
  if (eType == SM_ASCII) { std::ostream & rFileOut = *rDB.GetOutStreamPtr() ;

                           rFileOut << m_dTwistPeriodDist  << "TwistVolume Twist NeutralDistance \n" ;
                           rFileOut << m_dPeriodMinDist    << "TwistVolume SingularityTol: min legal distance from singularity at U = 0\n" ;
                         }                                                                           
  else /* Binary */      { 
                           SER(rDB.WriteDouble(m_dTwistPeriodDist)) ;
                           SER(rDB.WriteDouble(m_dPeriodMinDist )) ;
                         }

  // Write base class data
  SmVolume::WriteToDB(rDB, lDBVersionNumber) ;

  // all done
  return SM_SUCCESS;

} // end SmTwistVolume::WriteToDB

/*******************************************************************//**
PURPOSE: static method to Read a SmTransform from a given stream  

NOTES: 
***********************************************************************/
SmStatus SmTwistVolume::ReadFromDB
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
         || rpNewVolume->IsKindOf(SmTwistVolume_TYPE)) ? SM_SUCCESS : SM_ERR) ;

  // init output object
  SmTwistVolume *pTwistVolume =   (rpNewVolume == NULL)
                              ? new (crContext) SmTwistVolume()
                              : (SmTwistVolume *)rpNewVolume ;

  // file type
  SmFileType eType = rDB.GetFileType();
  

  // Volume locals
  double *pTwistPeriodDist = &pTwistVolume->m_dTwistPeriodDist ;
  double *pSingularityTol  = &pTwistVolume->m_dPeriodMinDist ; 

  if (eType == SM_ASCII) 
    {
      std::istream & rFileIn = *rDB.GetInStreamPtr();
      rFileIn >> pTwistPeriodDist[0] ;  rDB.GoToNextLine() ;
      rFileIn >> pSingularityTol[0] ;   rDB.GoToNextLine() ;
    }
  else 
    {
      SER(rDB.ReadDouble(pTwistPeriodDist[0]));
      SER(rDB.ReadDouble(pSingularityTol[0]));
    }

  // set output
  rpNewVolume = pTwistVolume ;

  // read base class data
  SmVolume::ReadFromDB(SmVolume_TYPE, rDB, crContext, rpNewVolume, lDBVersionNumber ) ; 

  // all done
  return SM_SUCCESS;

} // end SmTwistVolume::ReadFromDB

/*******************************************************************//**
PURPOSE: Compute the total size of the memory used by the SmTwistVolume.

NOTES: includes attribute memory
***********************************************************************/
ULONG SmTwistVolume::GetMemoryUsed    // rtn: smaller size of actually used memory in bytes
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
  rlMemoryAllocated = sizeof(*this) + SmVolume::GetMemoryUsed(rlMemoryAllocated) ; 

  // + attribute memory
  ULONG lThisAllocated ;
  ULONG lUsed       = rlMemoryAllocated + this->GetAttributeMemoryUsed(lThisAllocated, 
                                                                       eMarkType) ;  // note: uses without increment eMarkType value
  rlMemoryAllocated += lThisAllocated ;

  // all done
  return(lUsed) ;

} // end SmTwistVolume::GetMemoryUsed

/*******************************************************************//**
PURPOSE: AssertValid() Test reporting static labels
***********************************************************************/
SmAssertReportLabel sAssertTwistVolume_list[] =
{
  /*  0 */ {SM_AT_UNIT_VECTOR, _T("unit-vector"),        _T("TwistAxis has to be unit lenghth") },
  /*  1 */ {SM_AT_UNIT_VECTOR, _T("unit-vector"),        _T("TwistMidDir has to be unit lenghth") },
  /*  2 */ {SM_AT_UNIT_VECTOR, _T("unit-vector"),        _T("TwistBinormal has to be unit lenghth") },
  /*  3 */ {SM_AT_GEOMETRIC,   _T("orthogonal"),         _T("Twist Axis/StartDir/Binormal must be mutually orthoganal") },
  /*  4 */ {SM_AT_GEOMETRIC,   _T("orthogonal"),         _T("Twist Axis/StartDir/Binormal must be right handed") },
  /*  5 */ {SM_AT_GEOMETRIC,   _T("NonZero"),            _T("Twist PeriodDistance abs value has to be greater than PeriodMinDist") }
} ;

/*******************************************************************//**
PURPOSE: Make sure m_pNurb is not NULL by calling MakeNurb() when needed.

NOTES: returns TRUE  = OK
                        FALSE = Problem
***********************************************************************/
SmBoolean SmTwistVolume::AssertValid
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

  // Run SmTwistVolume checks

  // TwistAxis is a unit vector
  bRtn &= SM_ASSERT_VALUE_REPORT(0, SM_LEVEL_0, (smos_Fabs(GetTwistAxis().Dot(GetTwistAxis())-1.0) < SM_EFF_ZERO), SM_EFF_ZERO, smos_Fabs(GetTwistAxis().Dot(GetTwistAxis())-1.0), _T("")) ;

  // TwistMidDir is a unit vector
  bRtn &= SM_ASSERT_VALUE_REPORT(1, SM_LEVEL_0, (smos_Fabs(GetTwistMidDir().Dot(GetTwistMidDir())-1.0) < SM_EFF_ZERO), SM_EFF_ZERO, smos_Fabs(GetTwistMidDir().Dot(GetTwistMidDir())-1.0), _T("")) ;

  // TwistBinormal is a unit vector
  bRtn &= SM_ASSERT_VALUE_REPORT(2, SM_LEVEL_0, (smos_Fabs(GetTwistBinormal().Dot(GetTwistBinormal())-1.0) < SM_EFF_ZERO), SM_EFF_ZERO, smos_Fabs(GetTwistBinormal().Dot(GetTwistBinormal())-1.0), _T("")) ;

  // basis triple product - should be +1
  double dTripleProduct = GetTwistMidDir().TripleProduct(GetTwistBinormal(), GetTwistAxis()) ; 

  // basis vectors have to be mutually orthogonal
  bRtn &= SM_ASSERT_VALUE_REPORT(3, SM_LEVEL_0, (smos_Fabs(dTripleProduct)-1.0) < SM_EFF_ZERO, SM_EFF_ZERO, (smos_Fabs(dTripleProduct)-1.0), _T("")) ;

  // basis vectors have to be right handed
  bRtn &= SM_ASSERT_VALUE_REPORT(4, SM_LEVEL_0, (dTripleProduct-1.0) < SM_EFF_ZERO, SM_EFF_ZERO, (dTripleProduct-1.0), _T("")) ;

  // Twist PeriodDistance abs value has to be greater than PeriodMinDist
  bRtn &= SM_ASSERT_VALUE_REPORT(5, SM_LEVEL_0 , smos_Fabs(m_dTwistPeriodDist) >= m_dPeriodMinDist, SM_EFF_ZERO, m_dTwistPeriodDist, _T("")) ;

  // all done 
  return(bRtn) ;

} // end SmTwistVolume::AssertValid

// obsolete
// /*******************************************************************//**
// PURPOSE: Fix Assert Report failures reported by an AssertValid call
// 
// RETURNS ---  TRUE  = OK
//              FALSE = Problem
// ***********************************************************************/
// SmBoolean SmTwistVolume::AssertHeal
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
//                rAReport.m_pHealMessage = _T("SmTwistVolume::AssertHeal fix not yet supported") ;  
//                 
//     }
// 
//   // all done
//   return(bRtn) ;
// 
// } // end SmTwistVolume::AssertHeal
// end obsolete

/*******************************************************************//**
PURPOSE: Draw Axis - origin(black), x(red), y(green), z(blue)
         Draw Neutral Plane graphics
         Draw Twist Geometry
NOTES:
***********************************************************************/
SmDisplayList *SmTwistVolume::Draw
 (SmBoolean       bShowOutSpace,    // in : TRUE = draw point = Evaluate(sParam)), FALSE=don't
                                    //      default:[TRUE]
  SmBoolean       bShowInSpace,     // in : TRUE = draw point = Orient(sParam), FALSE=don't
                                    //      default:[FALSE]
  SmExtent3d    * pOptParamDomain,  // in : optionally output an XYPlane subDomain graphic, NULL to ignore
                                    //      default:[NULL]
  SmGfxArraySet * pOptGfxSet)       // i/o: used for OpenGL ES 2.0. When given, accumulate GfxVertexArrays not GL calls.
                                    //      NULL to ignore. default:[NULL]
  const 
{
  SmDisplayList *pRtn = NULL ;

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  if(bDebugMe)
    {
      SM_DUMP_AND_ASSERT_VALID(this) ;
    }
#endif // SM_DEBUG_CODE

#ifdef SM_GFX_CODE
  SmVector3d sColor = smgfx_GetRuleColor() ;

  smgfx_Open(smgfx_GetRuleColor()) ;

  // locals
  const SmContext *cpContext = GetContext() ;

  // choose TgtDomain as asked and approximate unbounded intervals
  SmExtent3d  sParamDomain = GetNaturalParamDomain() ; 
  SmExtent3d *pTgtDomain   = pOptParamDomain ? pOptParamDomain : &sParamDomain ;
  *pTgtDomain = pTgtDomain->ApproximateUnbounded() ;

  // 1. limit TgtDomain->IntervalV 
  // 2. for a better display, set UMin = 1/2 NeutralDistance
  // 3. for a better display, set WIvl proportional to VIvl
  double dMaxV =  m_dTwistPeriodDist * SM_PI * 3/4 ;
  double dMinV = -m_dTwistPeriodDist * SM_PI * 3/4 ;
  dMaxV = smos_Min(pTgtDomain->GetVMax(), dMaxV) ;    
  dMinV = smos_Max(pTgtDomain->GetVMin(), dMinV) ;

  double dMaxU =  smos_Min(3.0 * m_dTwistPeriodDist / 2.0, dMaxV) ;
  double dMinU =  dMaxU / 6.0 ;

  double dMaxW =  1.5 * dMaxV ;
  double dMinW = -1.5 * dMaxV ;
  dMaxW = smos_Min(pTgtDomain->GetWMax(), dMaxW) ;    
  dMinW = smos_Max(pTgtDomain->GetWMin(), dMinW) ;

  // specify the tgt draw domain
  pTgtDomain->SetMinMax(dMinU, dMinV, dMinW,
                        dMaxU, dMaxV, dMaxW) ;

  // remember when W=0 plane is part of the display
  SmBoolean bDrawW0 = SM_IS_CONTAINED(0.0, dMinW, dMaxW) ;

  // when asked - show InSpace
  if(bShowInSpace)
    {
      //   Build v=vmin, w=wmin    IsoParameterLine
      //   Build v=vmin, w=wmax    IsoParameterLine
      //   Build v=vmax, w=wmin    IsoParameterLine
      //   Build v=vmax, w=wmax    IsoParameterLine
      SmCurve *pIsoCurve1 = NULL ; 
      SmCurve *pIsoCurve2 = NULL ;
      SmCurve *pIsoCurve3 = NULL ;
      SmCurve *pIsoCurve4 = NULL ;
      SmCurve *pIsoCurve5 = NULL ; 
      SmCurve *pIsoCurve6 = NULL ;
      SmCurve *pIsoCurve7 = NULL ;
      SmCurve *pIsoCurve8 = NULL ;
      SmCurve *pIsoCurve9 = NULL ;
      SmCurve *pIsoCurve10 = NULL ;
      EvaluateIsoParametricCurve(*cpContext, SM_VPS_VW, pTgtDomain->GetVMin(), pTgtDomain->GetWMin(), 0.0, NULL, &pIsoCurve1, pTgtDomain) ; 
      EvaluateIsoParametricCurve(*cpContext, SM_VPS_VW, pTgtDomain->GetVMin(), pTgtDomain->GetWMax(), 0.0, NULL, &pIsoCurve2, pTgtDomain) ; 
      EvaluateIsoParametricCurve(*cpContext, SM_VPS_VW, pTgtDomain->GetVMax(), pTgtDomain->GetWMin(), 0.0, NULL, &pIsoCurve3, pTgtDomain) ; 
      EvaluateIsoParametricCurve(*cpContext, SM_VPS_VW, pTgtDomain->GetVMax(), pTgtDomain->GetWMax(), 0.0, NULL, &pIsoCurve4, pTgtDomain) ; 
      if(bDrawW0)
        {
          EvaluateIsoParametricCurve(*cpContext, SM_VPS_VW, pTgtDomain->GetVMin(), 0.0, 0.0, NULL, &pIsoCurve5 , pTgtDomain) ; 
          EvaluateIsoParametricCurve(*cpContext, SM_VPS_VW, pTgtDomain->GetVMax(), 0.0, 0.0, NULL, &pIsoCurve6 , pTgtDomain) ; 
        }
      //      EvaluateIsoParametricCurve(*cpContext, SM_VPS_VW, .75*pTgtDomain->GetVMin() + .25*pTgtDomain->GetVMax(), pTgtDomain->GetWMin(), 0.0, NULL, &pIsoCurve7 , pTgtDomain) ; 
      //      EvaluateIsoParametricCurve(*cpContext, SM_VPS_VW, .25*pTgtDomain->GetVMin() + .75*pTgtDomain->GetVMax(), pTgtDomain->GetWMax(), 0.0, NULL, &pIsoCurve8 , pTgtDomain) ; 
      //      EvaluateIsoParametricCurve(*cpContext, SM_VPS_VW, .50*pTgtDomain->GetVMin() + .50*pTgtDomain->GetVMax(), pTgtDomain->GetWMax(), 0.0, NULL, &pIsoCurve9 , pTgtDomain) ; 
      //      EvaluateIsoParametricCurve(*cpContext, SM_VPS_VW, .75*pTgtDomain->GetVMin() + .25*pTgtDomain->GetVMax(), pTgtDomain->GetWMax(), 0.0, NULL, &pIsoCurve10, pTgtDomain) ; 
      SmObjDelete sClean1(pIsoCurve1) ;
      SmObjDelete sClean2(pIsoCurve2) ;
      SmObjDelete sClean3(pIsoCurve3) ;
      SmObjDelete sClean4(pIsoCurve4) ;
      SmObjDelete sClean5(pIsoCurve5) ;
      SmObjDelete sClean6(pIsoCurve6) ;
      SmObjDelete sClean7(pIsoCurve7) ;
      SmObjDelete sClean8(pIsoCurve8) ;
      SmObjDelete sClean9(pIsoCurve9) ;
      SmObjDelete sClean10(pIsoCurve10) ;

#ifdef SM_DEBUG_CODE
  if(bDebugMe)
    {
      SM_DUMP_AND_ASSERT_VALID(pIsoCurve1) ;
      SM_DUMP_AND_ASSERT_VALID(pIsoCurve2) ;
      SM_DUMP_AND_ASSERT_VALID(pIsoCurve3) ;
      SM_DUMP_AND_ASSERT_VALID(pIsoCurve4) ;
    }
#endif // SM_DEBUG_CODE

      //   Build u = umin            IsoParameterPlane
      //   Build u = NeutralDistance IsoParameterPlane
      //   Build u = umax            IsoParameterPlane
      SmSurface *pIsoSurface1 = NULL ;
      SmSurface *pIsoSurface2 = NULL ;
      SmSurface *pIsoSurface3 = NULL ;
      SmSurface *pIsoSurface4 = NULL ;
      EvaluateIsoParametricSurface(*cpContext, SM_VP_U, pTgtDomain->GetUMin(),    0.0, NULL, &pIsoSurface1, pTgtDomain) ; 
      EvaluateIsoParametricSurface(*cpContext, SM_VP_U, pTgtDomain->GetUMid(),    0.0, NULL, &pIsoSurface2, pTgtDomain) ; 
      EvaluateIsoParametricSurface(*cpContext, SM_VP_U, pTgtDomain->GetUMax(),    0.0, NULL, &pIsoSurface3, pTgtDomain) ;
      if(bDrawW0)
        {
           EvaluateIsoParametricSurface(*cpContext, SM_VP_W, 0.0, 0.0, NULL, &pIsoSurface4, pTgtDomain) ; 
        }
      SmObjDelete sCleanS1(pIsoSurface1) ;                                                  
      SmObjDelete sCleanS2(pIsoSurface2) ;
      SmObjDelete sCleanS3(pIsoSurface3) ;
      SmObjDelete sCleanS4(pIsoSurface4) ;

#ifdef SM_DEBUG_CODE
  if(bDebugMe)
    {
      SM_DUMP_AND_ASSERT_VALID(pIsoSurface1) ;
      SM_DUMP_AND_ASSERT_VALID(pIsoSurface2) ;
      SM_DUMP_AND_ASSERT_VALID(pIsoSurface3) ;
      SM_DUMP_AND_ASSERT_VALID(pIsoSurface4) ;
    }
#endif // SM_DEBUG_CODE

      // draw isoCurves and isoSurfaces
      if(pIsoCurve1) pIsoCurve1->Draw(NULL, FALSE, NULL, pOptGfxSet) ;
      if(pIsoCurve2) pIsoCurve2->Draw(NULL, FALSE, NULL, pOptGfxSet) ;
      if(pIsoCurve3) pIsoCurve3->Draw(NULL, FALSE, NULL, pOptGfxSet) ;
      if(pIsoCurve4) pIsoCurve4->Draw(NULL, FALSE, NULL, pOptGfxSet) ;
      if(pIsoCurve5) pIsoCurve5->Draw(NULL, FALSE, NULL, pOptGfxSet) ;
      if(pIsoCurve6) pIsoCurve6->Draw(NULL, FALSE, NULL, pOptGfxSet) ;
      if(pIsoCurve7) pIsoCurve7->Draw(NULL, FALSE, NULL, pOptGfxSet) ;
      if(pIsoCurve8) pIsoCurve8->Draw(NULL, FALSE, NULL, pOptGfxSet) ;
      if(pIsoCurve9) pIsoCurve9->Draw(NULL, FALSE, NULL, pOptGfxSet) ;
      if(pIsoCurve10) pIsoCurve10->Draw(NULL, FALSE, NULL, pOptGfxSet) ;

      if(pIsoSurface1) pIsoSurface1->DrawUV(8,8,FALSE,NULL,FALSE,pOptGfxSet) ;
      if(pIsoSurface2) pIsoSurface2->DrawUV(8,8,FALSE,NULL,FALSE,pOptGfxSet) ;
      if(pIsoSurface3) pIsoSurface3->DrawUV(8,8,FALSE,NULL,FALSE,pOptGfxSet) ;
      if(pIsoSurface4) pIsoSurface4->DrawUV(8,8,FALSE,NULL,FALSE,pOptGfxSet) ;
      
    } // end bShowInSpace check

  // when asked - show OutSpace
  if(bShowOutSpace)
    {
      //   Build v=vmin, w=wmin    IsoParameterLine
      //   Build v=vmin, w=wmax    IsoParameterLine
      //   Build v=vmax, w=wmin    IsoParameterLine
      //   Build v=vmax, w=wmax    IsoParameterLine
      SmCurve *pIsoCurve1 = NULL ; 
      SmCurve *pIsoCurve2 = NULL ;
      SmCurve *pIsoCurve3 = NULL ;
      SmCurve *pIsoCurve4 = NULL ;
      SmCurve *pIsoCurve5 = NULL ; 
      SmCurve *pIsoCurve6 = NULL ;
      SmCurve *pIsoCurve7 = NULL ;
      SmCurve *pIsoCurve8 = NULL ;
      SmCurve *pIsoCurve9 = NULL ;
      SmCurve *pIsoCurve10 = NULL ;
      EvaluateIsoParametricCurve(*cpContext, SM_VPS_VW, pTgtDomain->GetVMin(), pTgtDomain->GetWMin(), 0.0, &pIsoCurve1, NULL, pTgtDomain) ; 
      EvaluateIsoParametricCurve(*cpContext, SM_VPS_VW, pTgtDomain->GetVMin(), pTgtDomain->GetWMax(), 0.0, &pIsoCurve2, NULL, pTgtDomain) ; 
      EvaluateIsoParametricCurve(*cpContext, SM_VPS_VW, pTgtDomain->GetVMax(), pTgtDomain->GetWMin(), 0.0, &pIsoCurve3, NULL, pTgtDomain) ; 
      EvaluateIsoParametricCurve(*cpContext, SM_VPS_VW, pTgtDomain->GetVMax(), pTgtDomain->GetWMax(), 0.0, &pIsoCurve4, NULL, pTgtDomain) ; 
      if(bDrawW0)
        {
          EvaluateIsoParametricCurve(*cpContext, SM_VPS_VW, pTgtDomain->GetVMin(), 0.0, 0.0, &pIsoCurve5 , NULL, pTgtDomain) ; 
          EvaluateIsoParametricCurve(*cpContext, SM_VPS_VW, pTgtDomain->GetVMax(), 0.0, 0.0, &pIsoCurve6 , NULL, pTgtDomain) ; 
        }
      //      EvaluateIsoParametricCurve(*cpContext, SM_VPS_VW, .75*pTgtDomain->GetVMin() + .25*pTgtDomain->GetVMax(), pTgtDomain->GetWMin(), 0.0, &pIsoCurve7 , NULL, pTgtDomain) ; 
      //      EvaluateIsoParametricCurve(*cpContext, SM_VPS_VW, .25*pTgtDomain->GetVMin() + .75*pTgtDomain->GetVMax(), pTgtDomain->GetWMax(), 0.0, &pIsoCurve8 , NULL, pTgtDomain) ; 
      //      EvaluateIsoParametricCurve(*cpContext, SM_VPS_VW, .50*pTgtDomain->GetVMin() + .50*pTgtDomain->GetVMax(), pTgtDomain->GetWMax(), 0.0, &pIsoCurve9 , NULL, pTgtDomain) ; 
      //      EvaluateIsoParametricCurve(*cpContext, SM_VPS_VW, .75*pTgtDomain->GetVMin() + .25*pTgtDomain->GetVMax(), pTgtDomain->GetWMax(), 0.0, &pIsoCurve10, NULL, pTgtDomain) ; 
      SmObjDelete sClean1(pIsoCurve1) ;
      SmObjDelete sClean2(pIsoCurve2) ;
      SmObjDelete sClean3(pIsoCurve3) ;
      SmObjDelete sClean4(pIsoCurve4) ;
      SmObjDelete sClean5(pIsoCurve5) ;
      SmObjDelete sClean6(pIsoCurve6) ;
      SmObjDelete sClean7(pIsoCurve7) ;
      SmObjDelete sClean8(pIsoCurve8) ;
      SmObjDelete sClean9(pIsoCurve9) ;
      SmObjDelete sClean10(pIsoCurve10) ;

#ifdef SM_DEBUG_CODE
  if(bDebugMe)
    {
      SM_DUMP_AND_ASSERT_VALID(pIsoCurve1) ;
      SM_DUMP_AND_ASSERT_VALID(pIsoCurve2) ;
      SM_DUMP_AND_ASSERT_VALID(pIsoCurve3) ;
      SM_DUMP_AND_ASSERT_VALID(pIsoCurve4) ;
    }
#endif // SM_DEBUG_CODE

      //   Build u = umin            IsoParameterPlane
      //   Build u = NeutralDistance IsoParameterPlane
      //   Build u = umax            IsoParameterPlane
      SmSurface *pIsoSurface1 = NULL ;
      SmSurface *pIsoSurface2 = NULL ;
      SmSurface *pIsoSurface3 = NULL ;
      SmSurface *pIsoSurface4 = NULL ;
      EvaluateIsoParametricSurface(*cpContext, SM_VP_U, pTgtDomain->GetUMin(),    0.0, &pIsoSurface1, NULL, pTgtDomain) ; 
      EvaluateIsoParametricSurface(*cpContext, SM_VP_U, pTgtDomain->GetUMid(),    0.0, &pIsoSurface2, NULL, pTgtDomain) ; 
      EvaluateIsoParametricSurface(*cpContext, SM_VP_U, pTgtDomain->GetUMax(),    0.0, &pIsoSurface3, NULL, pTgtDomain) ; 
      if(bDrawW0)
        {
          EvaluateIsoParametricSurface(*cpContext, SM_VP_W, 0.0, 0.0, &pIsoSurface4, NULL, pTgtDomain) ; 
        }
      SmObjDelete sCleanS1(pIsoSurface1) ;
      SmObjDelete sCleanS2(pIsoSurface2) ;
      SmObjDelete sCleanS3(pIsoSurface3) ;
      SmObjDelete sCleanS4(pIsoSurface4) ;

#ifdef SM_DEBUG_CODE
  if(bDebugMe)
    {
      SM_DUMP_AND_ASSERT_VALID(pIsoSurface1) ;
      SM_DUMP_AND_ASSERT_VALID(pIsoSurface2) ;
      SM_DUMP_AND_ASSERT_VALID(pIsoSurface3) ;
      SM_DUMP_AND_ASSERT_VALID(pIsoSurface4) ;
    }
#endif // SM_DEBUG_CODE

      // draw isoCurves and isoSurfaces
      if(pIsoCurve1) pIsoCurve1->Draw(NULL, FALSE, NULL, pOptGfxSet) ;
      if(pIsoCurve2) pIsoCurve2->Draw(NULL, FALSE, NULL, pOptGfxSet) ;
      if(pIsoCurve3) pIsoCurve3->Draw(NULL, FALSE, NULL, pOptGfxSet) ;
      if(pIsoCurve4) pIsoCurve4->Draw(NULL, FALSE, NULL, pOptGfxSet) ;
      if(pIsoCurve5) pIsoCurve5->Draw(NULL, FALSE, NULL, pOptGfxSet) ;
      if(pIsoCurve6) pIsoCurve6->Draw(NULL, FALSE, NULL, pOptGfxSet) ;
      if(pIsoCurve7) pIsoCurve7->Draw(NULL, FALSE, NULL, pOptGfxSet) ;
      if(pIsoCurve8) pIsoCurve8->Draw(NULL, FALSE, NULL, pOptGfxSet) ;
      if(pIsoCurve9) pIsoCurve9->Draw(NULL, FALSE, NULL, pOptGfxSet) ;
      if(pIsoCurve10) pIsoCurve10->Draw(NULL, FALSE, NULL, pOptGfxSet) ;

      if(pIsoSurface1) pIsoSurface1->DrawUV(8,8,FALSE,NULL,FALSE,pOptGfxSet) ;
      if(pIsoSurface2) pIsoSurface2->DrawUV(8,8,FALSE,NULL,FALSE,pOptGfxSet) ;
      if(pIsoSurface3) pIsoSurface3->DrawUV(8,8,FALSE,NULL,FALSE,pOptGfxSet) ;

      // Make W=0 plane a little heavier to see it better

      // get global SmGraphicsExtern.cpp:s_Disp display parameters
      SmDisplayParameters &rDisp = smgfx_RefGlobalDisplayParameters() ;
      double dCrossHatchLineWidth = rDisp.m_dCrossHatchLineWidth ; rDisp.m_dCrossHatchLineWidth = 2 * dCrossHatchLineWidth ;

      if(pIsoSurface4) pIsoSurface4->DrawUV(8,8,FALSE,NULL,FALSE,pOptGfxSet) ;
      rDisp.m_dCrossHatchLineWidth = dCrossHatchLineWidth ;
                                                                            
    } // end bShowOutSpace check                                            
                                                                            
  // Draw twist coordinate system - U vector scaled to NeutralDistance      
  SmVector3d sTwistOrigin = GetTwistOrigin();                               
  smgfx_SetColor(0,0,0) ; sTwistOrigin.Draw(NULL, NULL, pOptGfxSet) ;       
  smgfx_SetColor(1,0,0) ; (m_dTwistPeriodDist * GetTwistMidDir()).Draw(&sTwistOrigin, NULL, pOptGfxSet) ;
  smgfx_SetColor(0,1,0) ; GetTwistBinormal().Draw(&sTwistOrigin, NULL, pOptGfxSet) ;
  smgfx_SetColor(0,0,1) ; GetTwistAxis().Draw(&sTwistOrigin, NULL, pOptGfxSet) ;
  smgfx_SetColor(sColor, pOptGfxSet) ;
  
  // Draw Twist axis over W extent
  SmExtent1d sIvlW      = pTgtDomain->GetWInterval() ;
  SmPoint3d  sStartAxis = GetTwistOrigin() + sIvlW.GetMin() * GetTwistAxis() ;
  SmPoint3d  sEndAxis   = GetTwistOrigin() + sIvlW.GetMax() * GetTwistAxis() ;
  double dLineWidth = smgfx_GetLineWidth() ;
  smgfx_SetLineWidth(2*dLineWidth, pOptGfxSet) ;
  smgfx_DrawLine(sStartAxis.x, sStartAxis.y, sStartAxis.z,
                 sEndAxis.x,   sEndAxis.y,   sEndAxis.z, pOptGfxSet) ;
  smgfx_SetLineWidth(dLineWidth, pOptGfxSet) ;
  
  pRtn = smgfx_Close() ;

#else
  SM_REF4(bShowOutSpace, bShowInSpace, pOptParamDomain, pOptGfxSet);
#endif // SM_GFX_CODE

  // all done
  return(pRtn) ;

} // end SmTwistVolume::Draw

/*******************************************************************//**
PURPOSE:
NOTES:
***********************************************************************/
SmBoolean SmTwistVolume::IsKindOf( SM_TYPE t ) const
{
  return ((SmTwistVolume_TYPE == t) ? TRUE : SmVolume::IsKindOf( (t) ));
}

/*******************************************************************//**
PURPOSE: Dump bspline volume data out for debugging.
            Dump is in similar sequence to Write to file

NOTES: 
***********************************************************************/
void SmTwistVolume::Dump() const
{
  Dump(FALSE, 0) ;
}
/*******************************************************************//**
PURPOSE: Dump bspline volume data out for debugging.
            Dump is in similar sequence to Write to file

NOTES: 
***********************************************************************/
void SmTwistVolume::Dump
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

  // header
  smos_sprintf(sBuff,        _T("\n%sBegin SmTwistVolume[0x%p]::Dump()"), sIndent, this) ;
  smos_sprintf(sBuffForFile, _T("\n%sBegin SmTwistVolume::Dump()"), sIndent) ;
  smos_WriteBuffer(sBuff, sBuffForFile);

  // header
  smos_sprintf(sBuff,       _T("\n%s  SmTwistVolume = 0x%p"), sIndent, this);
  smos_sprintf(sBuffForFile,_T("\n%s  SmTwistVolume = %s"), sIndent, _T("notNULL"));
  smos_WriteBuffer(sBuff, sBuffForFile);

  smos_WriteBuffer(_T("\n")) ;
  smos_sprintf(sBuff, _T("\n%s  TwistOrigin     = InSpace point on the twist center line"), sIndent) ; smos_WriteBuffer(sBuff) ;
  smos_sprintf(sBuff, _T("\n%s  TwistAxis       = InSpace twist center line unit-vector direction; Twist Coordinate system ZAxis"), sIndent) ; smos_WriteBuffer(sBuff) ;
  smos_sprintf(sBuff, _T("\n%s  TwistMidDir     = InSpace orthoganal to TwistAxis;                 Twist Coordinate system XAxis"), sIndent) ; smos_WriteBuffer(sBuff) ;
  smos_sprintf(sBuff, _T("\n%s  TwistBinormal   = InSpace unit-vector = TwistAxis * TwistMidDir ;  Twist Coordinate system YAxis"), sIndent) ; smos_WriteBuffer(sBuff) ;
  smos_sprintf(sBuff, _T("\n%s  TwistPeriodDist = Dist along TwistAxis for one complete Twist revolution: pos:RightHanded, neg:LeftHanded, must be NonZero"), sIndent) ; smos_WriteBuffer(sBuff) ;
  smos_sprintf(sBuff, _T("\n%s  PeriodMinDist   = Min allowed TwistPeriodDist value"), sIndent) ; smos_WriteBuffer(sBuff) ;
  smos_WriteBuffer(_T("\n")) ;
  
  smos_sprintf(sBuff, _T("\n%s    TwistOrigin     = "), sIndent) ; smos_WriteBuffer(sBuff) ; GetTwistOrigin().Dump() ;
  smos_sprintf(sBuff, _T("\n%s    TwistAxis       = "), sIndent) ; smos_WriteBuffer(sBuff) ; GetTwistAxis().Dump() ;
  smos_sprintf(sBuff, _T("\n%s    TwistMidDir     = "), sIndent) ; smos_WriteBuffer(sBuff) ; GetTwistMidDir().Dump() ;
  smos_sprintf(sBuff, _T("\n%s    TwistBinormal   = "), sIndent) ; smos_WriteBuffer(sBuff) ; GetTwistBinormal().Dump() ;
  smos_sprintf(sBuff, _T("\n%s    TwistPeriodDist =  %16.16lf"), sIndent, GetTwistPeriodDist()) ; smos_WriteBuffer(sBuff) ;
  smos_sprintf(sBuff, _T("\n%s    PeriodMinDist   =  %16.16lf"), sIndent, GetPeriodMinDist()) ;  smos_WriteBuffer(sBuff) ;

  // dump the base class 
  SmVolume::Dump(bAbbrev, lIndentCnt+2) ;

  smos_sprintf(sBuff,        _T("\n%sEnd SmTwistVolume[0x%p]::Dump()%s"), sIndent, this, lIndentCnt==0?_T("\n"):_T("")) ;
  smos_sprintf(sBuffForFile, _T("\n%sEnd SmTwistVolume::Dump()%s"), sIndent, lIndentCnt==0?_T("\n"):_T("")) ;
  smos_WriteBuffer(sBuff, sBuffForFile);
  
} // end SmTwistVolume::Dump

