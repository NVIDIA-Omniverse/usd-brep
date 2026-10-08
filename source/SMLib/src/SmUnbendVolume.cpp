// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*******************************************************************//**
* FILE NAME --- SmUnbendVolume.cpp
* PURPOSE: Implementation of SmUnbendVolume methods.
* Oct-2006 - gwc - author
**********************************************************************/

#include "StdAfx.h"

#include <SmUnbendVolume.h>
#include <SmConfig.h>
#include <SmPseudoBox.h>
#include <SmCircle.h>
#include <SmLine.h>
#include <SmCylinder.h>
#include <SmPlane.h>
#include <SmNurbsCrv.h>
#include <SmDatabaseIO.h>
#include <SmCrvInVolume.h>
#include <SmSrfInVolume.h>
#include <SmAssertArray.h>
#include <SmCurveClass.h>
#include <SmTransform.h>
#include <SmGeomUtility.h>
#include <SmGraphicsOutput.h>

#ifdef SM_DEBUG_CODE
#include <SmEdge.h>
#include <SmFace.h>
#include <SmBrep.h>
#endif // SM_DEBUG_CODE

/*******************************************************************//**
PURPOSE: Constructor

NOTES: 
***********************************************************************/
SmUnbendVolume::SmUnbendVolume
  (const SmContext & crContext,           // in : for new OrientMap construction                                                                                           
   const SmPoint3d       & rUnbendCenter, // in : InSpace (and OutSpace) point on the Unbend center line                                                                     
   const SmVector3d      & rUnbendAxis,   // in : InSpace (and OutSpace) direction of the Unbend center line marking W axis of Unbend Coordinate System                                                 
   const SmVector3d      & rUnbendMidDir, // in : InSpace (and OutSpace) vector orthoganal to UnbendAxis marking U axis of Unbend Coordinate System                            
   double            dUnbendNeutralDist,  // in : InSpace (and OutSpace) distance along UnbendMidDir between UnbendOrigin and NeutralPlane.                                 
   double            dSingularityTol)     // in : min ProjSpace dist between NonSingular pts and the Volume's singularity at the ParameterPlane U=0.
                                          //      dSingularityTol must be >= SM_ZONE_TOL_3D, default:[SM_ZONE_TOL_3D]
                                          //  note: The UnbendMidDir/UnbendBinormal plane is the symmetric plane                                                            
                                          //        of the Unbend mapping                                                                                                                  
: SmVolume(&crContext),
  m_dUnbendNeutralDist(dUnbendNeutralDist),
  m_dSingularityTol(dSingularityTol)
{
  // check input - dSingularityTol >= SM_ZONE_TOL_3D
  if(m_dSingularityTol < SM_ZONE_TOL_3D)
    { 
      SE_MSG( SM_ERR,
             _T("UnbendVolume dSingularityTol must be >= SM_ZONE_TOL_3D - Bad Construction - changing dSingularityTol to SM_ZONE_TOL_3D")) ;
      m_dSingularityTol = SM_ZONE_TOL_3D ;
    }

  // check input - Positive Definite UnbendNeutralDistance
  if(dUnbendNeutralDist < m_dSingularityTol)
    {
      SE_MSG( SM_ERR,
             _T("UnbendVolume must have a PositiveDefinite NeutralDistance - Bad Construction - changing UnbendNeutralDist to 1.0")) ;

      // do the best we can
      m_dUnbendNeutralDist = 1.0 ;
    }

  // unitize Unbend vectors
  SmVector3d sUnbendAxis(rUnbendAxis) ;
  SmVector3d sUnbendMidDir(rUnbendMidDir) ;
  SmVector3d sUnbendBinormal ;
  SmStatus   eStat1 = sUnbendAxis.Unitize() ; 
  SmStatus   eStat2 = sUnbendMidDir.Unitize() ;

  // check input - nonZero input vectors
  if(eStat1 != SM_SUCCESS || eStat2 != SM_SUCCESS)
    {
      SE_MSG( SM_ERR,
             _T("Unbend Volume defining vectors were zero length - Bad Construction - Changing UnbendVectors to have length")) ; 

      // do the best we can
      if(eStat1 == SM_ERR && eStat2 == SM_ERR) { sUnbendAxis.Set(0,0,1) ; 
                                                 sUnbendMidDir.Set(1,0,0) ; 
                                               }
      else if(eStat1 == SM_ERR)                { sUnbendMidDir.MakeUnitOrthoVectors(NULL, sUnbendMidDir,
                                                                                          sUnbendBinormal,
                                                                                          sUnbendAxis) ; 
                                               }
      else                                     { sUnbendAxis.MakeUnitOrthoVectors(NULL, sUnbendAxis,
                                                                                        sUnbendMidDir,
                                                                                        sUnbendBinormal) ; 
                                               }
    } // end do something when given zero length vectors branch

  // check input - Unbend vectors must be perpendicular to one another
  SmBoolean bPerp = sUnbendAxis.IsPerpendicularTo(sUnbendMidDir, SM_EFF_ZERO_DEG) ;
  if(!bPerp)
    {
       SE_MSG( SM_ERR,
              _T("UnbendAxis not perpendicular to UnbendMidDir - Bad Construction - Changing UnbendVectors to be perpendicular")) ;

       SmBoolean bParallel = sUnbendAxis.IsParallelTo(sUnbendMidDir, SM_EFF_ZERO_DEG) ;

      if(!bParallel)
        {
         sUnbendMidDir = sUnbendAxis * (sUnbendMidDir * sUnbendAxis) ;
          sUnbendMidDir.Unitize() ;
        }
      else // Axis and StartDir don't span a plane
        {
          // do the best we can
          sUnbendMidDir.MakeUnitOrthoVectors(NULL, sUnbendMidDir,
                                                 sUnbendBinormal,
                                                 sUnbendAxis) ;
        }
    }
    
  // set UnbendBinormal perp to both UnbendAxis and UnbendMidDir
  sUnbendBinormal = sUnbendAxis * sUnbendMidDir ;

  // build UnbendCoordinates into m_pOrientMap
  SmTransform *pOrientMap = new (crContext) SmTransform(&crContext, rUnbendCenter, sUnbendMidDir, sUnbendBinormal, &sUnbendAxis) ;
  SetOrientMap(pOrientMap, 2) ;

} // end SmUnbendVolume constructor

/*******************************************************************//**
PURPOSE: Copy a SmUnbendVolume.

NOTES: 
***********************************************************************/
SmStatus SmUnbendVolume::Copy
 (const SmContext & crContext,      // in : context for new object construction
  SmVolume       *& rpNewVolume,    // out: The copied Volume
  SmBoolean         bSimpleMapOnly) // in : TRUE = Copy this Volume omitting any compounding volumes
 const                              //      FALSE= Copy this Volumes with any compounding volumes
                                    //      default:[FALSE]
{
  rpNewVolume = new (crContext) SmUnbendVolume(*this, bSimpleMapOnly);
  NER(rpNewVolume);
  return SM_SUCCESS;

} // end SmUnbendVolume::Copy

/*******************************************************************//**
PURPOSE: assignment operator

NOTES:
***********************************************************************/
SmUnbendVolume & SmUnbendVolume::operator=
  (const SmUnbendVolume &crUnbendVolume)       // in : object to copy
{ 
  if(this == &crUnbendVolume) return(*this) ;
  
  // assign base values
  SmVolume::operator=(crUnbendVolume) ;
                                                              
  // copy local members
  m_dUnbendNeutralDist = crUnbendVolume.m_dUnbendNeutralDist ;
  m_dSingularityTol    = crUnbendVolume.m_dSingularityTol ;

  // all done
  return(*this) ;

} // end SmUnbendVolume::operator=

/*******************************************************************//**
PURPOSE: Equality operator for SmUnbendVolume

NOTES: Call base equivalence to check type and then check 
       members for equivalence
***********************************************************************/
SmBoolean SmUnbendVolume::operator==
  (const SmVolume& crOther) 
 const
{
  // low work - same objects are equal
  if(this == &crOther) { return TRUE ; }

  // low work - different types are not equal
  if(!crOther.IsKindOf(SmUnbendVolume_TYPE))
    { return FALSE ; }

  // first check the base
  SmBoolean bRtn = SmVolume::operator ==(crOther) ;

  // then check the members
  if(bRtn)
    {
      // OK to cast
      SmUnbendVolume &rOther = (SmUnbendVolume &)crOther ;

      // check equivalence of these objects
      bRtn = SM_IS_ZERO(m_dUnbendNeutralDist - rOther.m_dUnbendNeutralDist) ;
      bRtn = SM_IS_ZERO(m_dSingularityTol    - rOther.m_dSingularityTol) ;
    }

  // all done
  return bRtn ;

} // end SmUnbendVolume::operator==

/*******************************************************************//**
PURPOSE: Create a SmUnbendVolume from component data.  

NOTES: 
***********************************************************************/
SmStatus SmUnbendVolume::CreateCanonical
 (const SmContext & crContext,          // in : context for new object construction                                                                            
  SmPoint3d       & rUnbendOrigin,      // in : InSpace (and OutSpace) point on the bend center line                                                           
  SmVector3d      & rUnbendAxis,        // in : InSpace (and OutSpace) direction of the bend center line marking W axis of Unbend Coordinate System                                       
  SmVector3d      & rUnbendMidDir,      // in : InSpace (and OutSpace) vector orthoganal to UnbendAxis marking U axis of Unbend Coordinate System                  
  double            dUnbendNeutralDist, // in : InSpace (and OutSpace) distance along rsUnbendMidDir between UnbendOrigin and NeutralPlane.                      
  SmUnbendVolume *& rpNewVolume,        // out: New SmUnbendVolume, NULL on input                                                                                
  double            dSingularityTol)    // in : min dist between NonSingular pts and Volume's singularity at U=0,
                                        //      dSingularityTol must be >= SM_ZONE_TOL_3D, default:[SM_ZONE_TOL_3D]
{
  // build the object
  rpNewVolume = new (crContext) SmUnbendVolume(crContext, rUnbendOrigin, rUnbendAxis, rUnbendMidDir, dUnbendNeutralDist, dSingularityTol) ;

  // all done
  return(SM_SUCCESS) ; 

} // end SmUnbendVolume::CreateCanonical

/*******************************************************************//**
PURPOSE: Get the STEP canonical data out of a UnbendVolume.

NOTES: Please note that the control points are in Euclidian
    space (not homogeneous) even if the volume is rational.  That is the 
    homogeneous division has been performed on x,y,z prior to returning 
    the data in rControlPointsList.
***********************************************************************/
SmStatus SmUnbendVolume::GetCanonical
 (SmPoint3d  & rUnbendCenter,       // out: InSpace (and OutSpace) point on the bend center line                                              
  SmVector3d & rUnbendAxis,         // out: InSpace (and OutSpace) direction of the bend center line marking W axis of Unbend Coordinate System                           
  SmVector3d & rUnbendMidDir,       // out: InSpace (and OutSpace) vector orthoganal to UnbendAxis marking U axis of Unbend Coordinate System
  double     & rdUnbendNeutralDist, // out: InSpace (and OutSpace) distance along rUnbendMidDir between UnbendOrigin and NeutralPlane.
  double     & rdSingularityTol)    // out: min ProjSpace dist between NonSingular pts and the Volume's singularity at the ParameterPlane U=0.
 const
{
  SM_DUMP_AND_ASSERT2_VALID(this) ;

  // set outputs
  rUnbendCenter       = GetUnbendOrigin() ; 
  rUnbendAxis         = GetUnbendAxis() ;    
  rUnbendMidDir       = GetUnbendMidDir() ;
  rdUnbendNeutralDist = m_dUnbendNeutralDist ;
  rdSingularityTol    = m_dSingularityTol ;

  // all done
  return SM_SUCCESS;

} // end SmUnbendVolume::GetCanonical

/*******************************************************************//**
PURPOSE: Set all SmUnbendVolume data.

NOTES: 
***********************************************************************/
SmStatus SmUnbendVolume::SetCanonical
  (SmPoint3d  & rUnbendCenter,        // in : InSpace (and OutSpace) point on the bend center line                                              
   SmVector3d & rUnbendAxis,          // in : InSpace (and OutSpace) direction of the bend center line marking W axis of Unbend Coordinate System                           
   SmVector3d & rUnbendMidDir,        // in : InSpace (and OutSpace) vector orthoganal to UnbendAxis marking U axis of Unbend Coordinate System
   double       dUnbendNeutralDist,   // in : InSpace (and OutSpace) distance along rUnbendMidDir between UnbendOrigin and NeutralPlane.
   double       dSingularityTol)    // in : min ProjSpace dist between NonSingular pts and the Volume's singularity at the ParameterPlane U=0.
                                    //      dSingularityTol must be >= SM_ZONE_TOL_3D, default:[SM_ZONE_TOL_3D]
{
  // prepare the public
  Notify(SM_NO_PRE_EDIT, this, SM_NO_GET_OWNER(this), NULL) ;

  // note: SetCanonical is called by the constructors - so don't return errors - always find a way to complete.

  // check input - dSingularityTol >= SM_ZONE_TOL_3D
  if(dSingularityTol < SM_ZONE_TOL_3D)
    { 
      SE_MSG( SM_ERR,
             _T("UnbendVolume dSingularityTol must be >= SM_ZONE_TOL_3D - Bad Construction - changing dSingularityTol to SM_ZONE_TOL_3D")) ;
      dSingularityTol = SM_ZONE_TOL_3D ;
    }

  // check input - Positive Definite UnbendNeutralDistance
  if(dUnbendNeutralDist < dSingularityTol)
    {
      SE_MSG( SM_ERR,
             _T("UnbendVolume must have a PositiveDefinite NeutralDistance - Bad Construction - changing UnbendNeutralDist to 1.0")) ;

      // do the best we can
      dUnbendNeutralDist = 1.0 ;
    }

  // check state - unit vectors
  SmVector3d sUnbendAxis         = rUnbendAxis ;
  SmVector3d sUnbendMidDir       = rUnbendMidDir ;
  SmStatus   sUnbendAxisStatus   = sUnbendAxis.Unitize() ;
  SmStatus   sUnbendMidDirStatus = sUnbendMidDir.Unitize() ;

  // zero length UnbendAxis vector
  if(sUnbendAxisStatus != SM_SUCCESS)
    {
      SE_MSG(SM_ERR, _T("SmUnbendVolume::SetCanonical: Zero length UnbendAxis - changing to (0,0,1) ")) ;
      sUnbendAxis.Set(0,0,1) ;
    }

  // zero length UnbendMidDir vector
  if(sUnbendMidDirStatus != SM_SUCCESS)
    {
      SE_MSG(SM_ERR, _T("SmUnbendVolume::SetCanonical: Zero length UnbendMidDir - changing to (1,0,0) ")) ;
      sUnbendAxis.Set(1,0,0) ;
    }

  // check state - orthogonal vectors
  double dDotProd = sUnbendAxis.Dot(sUnbendMidDir) ;

  // non orthogonal UnbendAxis/UnbendMidDir vector pair
  if(!SM_IS_ZERO(dDotProd))
    {
      SE_MSG(SM_ERR, _T("SmUnbendVolume::SetCanonical: UnbendAxis and UnbendMidDir not orthogonal - changing UnbendMidDir")) ;
      sUnbendMidDir = sUnbendAxis * (sUnbendMidDir * sUnbendAxis) ;
      sUnbendMidDir.Unitize() ;
    }

  // locals
  SmVector3d sUnbendBinormal = rUnbendAxis * rUnbendMidDir ;

  // set OrientMap
  SmTransform *pOrientMap = GetOrientMap() ;
  pOrientMap->SetCanonical(rUnbendCenter, rUnbendMidDir, sUnbendBinormal, &rUnbendAxis) ;

  // set Neutral distance
  m_dUnbendNeutralDist = dUnbendNeutralDist ; 
  
  // set SingularityTol
  m_dSingularityTol = dSingularityTol ;        

  // inform the public
  Notify(SM_NO_POST_EDIT, this, SM_NO_GET_OWNER(this), NULL) ;

  // all done
  return SM_SUCCESS;

} // end SmUnbendVolume::SetCanonical

/*******************************************************************//**
PURPOSE: convenience function to look like a SmBSplineVolume object that
         gets effective knot lists

NOTES: for cache purposes - let UnbendVolume mascarade as a BSpline kind of shape. 
       pretend to be a degree 3, 4 control point, 8 knot BSpline basis in all directions
***********************************************************************/
SmStatus SmUnbendVolume::GetKnots
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

} // end SmUnbendVolume::GetKnots                                          
                                                                           
/*******************************************************************//**   
PURPOSE: When possible build the exact Curve produced by projecting
    rInputCurve to 1st OutSpace

NOTES: when eInputSpace == SM_VS_PARAM_SPACE: rInputCurve is mapped from ParamSpace to 1stOutSpace
       else eInputSpace == SM_VS_INSPACE    : rInputCurve is mapped from InSpace to 1stOutSpace

   the UnbendVolume 'maps' 
  InputSpace Lines parallel to the Bend axis             to OutSpace lines parallel to the OutSpace Bend Axis           
  InputSpace circles centered on the Bend axis           to OutSpace lines parallel to the OutSpace Bend binormal 
  InputSpace lines radiating radially from the Bend Axis to OutSpace lines parallel to the OutSpace Bend MidDir  

  All other curve shapes and orientations are not projected to Analytic shapes and this 
  method sets rpNewCurve == NULL and returns SM_SUCCESS.
***********************************************************************/
SmStatus SmUnbendVolume::MakeExactBSpline1stOutCurve
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

  // locals: ParamSpace and InSpace Unbend orientations - Unbend's Neutral plane is perp to the Unbend's MidDirection
  SmVector3d sParamOrigin  (0,0,0), sUnbendOrigin   = GetUnbendOrigin() ;
  SmVector3d sParamMidDir  (1,0,0), sUnbendMidDir   = GetUnbendMidDir() ;
  SmVector3d sParamAxis    (0,0,1), sUnbendAxis     = GetUnbendAxis() ;
  SmVector3d sParamBinormal(0,1,0), sUnbendBinormal = GetUnbendBinormal() ;

  // locals: making InputSpace either ParamSpace or InSpace
  SmVector3d sInputOrigin, sInputMidDir, sInputAxis, sInputBinormal ;
  if(eInputSpace == SM_VS_IN_SPACE) { sInputOrigin   = sUnbendOrigin ;
                                      sInputMidDir   = sUnbendMidDir ;
                                      sInputAxis     = sUnbendAxis ;   
                                      sInputBinormal = sUnbendBinormal ;
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
  SmVector3d  sInputFirst,  sInputLast ;
  SmVector3d  sOutSpaceFirst, sOutSpaceLast ;

  SmApproxTol3d     sApproxTol = SmTol::GetApproxTol3d() ;
  const SmContext * pContext   = rInputCurve.GetContext() ;
  SmExtent1d        sIvl       = rInputCurve.GetNaturalInterval() ; 
  SmVector3d        sInputPt, sOnAxisPt, sOnAxisVec ; 

  double            dAxisPlane, dMidDirPlane, dBinormalPlane, dAxisLine, dMidDirAngRad ; // dist from Sample Pt to planes perp to Unbend Axes.
  SmExtent1d        sAxisPlaneRange, sMidDirPlaneRange, sBinormalPlaneRange, sAxisLineRange, sMidDirAngRadRange ;           // range on distances
  double            dAxisPlaneSum=0.0, dMidDirPlaneSum=0.0, dBinormalPlaneSum=0.0, sAxisLineSum=0.0, dMidDirAngRadSum=0.0 ; // Sum of SamplePt distances to planes
  SmBoolean         bConstAxisPlane = false, bConstMidDirPlane = false, bConstBinormalPlane = false, bConstAxisLine = false, bConstMidDirAngRad = false;

  // See if the InputCurve is linear and parallel to one of the UnbendAxes (parallel to two UnbendPlanes)
  for(ii=0;ii<=lSmpSize;ii++)
    {
      sSmpU = sIvl.Evaluate((double)ii/(double)lSmpSize) ;

      // Curve sample 
      rInputCurve.EvaluatePoint(sSmpU, sInputPt) ;

      // no work - sample point not within bend domain
      SmBoolean bIsIn =   (eInputSpace == SM_VS_IN_SPACE)
                        ? IsPointInInSpaceDomain(sInputPt) 
                        : IsPointInParamDomain(sInputPt) ;
      if(bIsIn == FALSE)
        { return SM_SUCCESS ; }
      
      // save 1st and last sample pts and pick a point on Axis near line to keep vector sizes small
      switch(ii)
        { case 0        : { sInputFirst = sInputPt ;
                            SmVector3d sToOrigVec = sInputPt - sInputOrigin ;
                            sOnAxisPt  = sInputOrigin  + sInputAxis.Dot(sToOrigVec) * sInputAxis ;
                          } break ;
          case lSmpSize : { sInputLast = sInputPt ; 
                          } break ;
          default: break ;
        }

      // Input vec from SamplePt to OnAxisPt
      sOnAxisVec = sInputPt - sOnAxisPt ;

      // InputSpace distances to planes perp to Unbend axes
      dAxisPlane     = sInputAxis.Dot(sOnAxisVec) ;                         // distance to Axis Plane
      dMidDirPlane   = sInputMidDir.Dot(sOnAxisVec) ;                       // distance to MidDir Plane
      dBinormalPlane = sInputBinormal.Dot(sOnAxisVec) ;                     // distance to Binormal Plane
      dAxisLine      = smos_Sqrt(dMidDirPlane*dMidDirPlane + dBinormalPlane*dBinormalPlane) ; // distance to Axis line
      sInputAxis.CCWAngleBetween(sInputMidDir, sOnAxisVec, dMidDirAngRad) ; // rot angle from sMidDirAxis

      // accumulate the range of Plane distances
      sAxisPlaneRange.     AddValue(dAxisPlane) ;                      
      sMidDirPlaneRange.   AddValue(dMidDirPlane) ;                 
      sBinormalPlaneRange. AddValue(dBinormalPlane) ;                 
      sAxisLineRange.      AddValue(dAxisLine) ;
      sMidDirAngRadRange.  AddValue(dMidDirAngRad) ;

      // accumulate the sum of Plane distances
      dAxisPlaneSum     += dAxisPlane ;
      dMidDirPlaneSum   += dMidDirPlane ;
      dBinormalPlaneSum += dBinormalPlane ;
      sAxisLineSum      += dAxisLine ;
      dMidDirAngRadSum  += dMidDirAngRad ;

      // remember when curve is const dist from bend plane
      bConstAxisPlane     = sAxisPlaneRange.GetLength()     < sApproxTol ;
      bConstMidDirPlane   = sMidDirPlaneRange.GetLength()   < sApproxTol ;
      bConstBinormalPlane = sBinormalPlaneRange.GetLength() < sApproxTol ;
      bConstAxisLine      = sAxisLineRange.GetLength()      < sApproxTol ;
      bConstMidDirAngRad  = sMidDirAngRadRange.GetLength()  < SM_EFF_ZERO_RAD ; 

      // no work - curve not one of the special case curves - hack here, depends on: TRUE = 1, FALSE=0
      if(   !(bConstMidDirPlane  && bConstBinormalPlane) // lines parallel to the Bend Axis
         && !(bConstAxisLine     && bConstAxisPlane)     // circles centered on the Bend Axis
         && !(bConstMidDirAngRad && bConstAxisPlane))    // lines radiating radially from the Bend Axis 
        { return SM_SUCCESS ; }

    } // end iter ii - gather samples for parallel test

  // arrive here when curve is a line, const dist from at least two planes
  SM_ASSERT_MSG(   (bConstMidDirPlane  && bConstBinormalPlane) // lines parallel to the Bend Axis
                || (bConstAxisLine     && bConstAxisPlane)     // circles centered on the Bend Axis
                || (bConstMidDirAngRad && bConstAxisPlane),    // lines radiating radially from the Bend Axis
                _T("Missed a case - this is a bug; check code.")) ;

  // OutSpace 1st and Last points
  if(eInputSpace == SM_VS_IN_SPACE)
    {
      MapPoint(sInputFirst, sOutSpaceFirst, FALSE) ;  // without compounding
      MapPoint(sInputLast,  sOutSpaceLast, FALSE) ;   // without compounding
    }
  else // eInputSpace == SM_VS_PARAM_SPACE
    {
      EvaluatePoint(sInputFirst, sOutSpaceFirst, FALSE) ; // without compounding 
      EvaluatePoint(sInputLast,  sOutSpaceLast, FALSE) ;  // without compounding 
    }

  // lines parallel to the Bend Axis             => map to OutSpace lines
  // circles centered on the Bend Axis           => map to OutSpace lines
  // lines radiating radially from the Bend Axis => map to OutSpace lines
  if(   (bConstMidDirPlane  && bConstBinormalPlane) // lines parallel to the Bend Axis
     || (bConstAxisLine     && bConstAxisPlane)     // circles centered on the Bend Axis
     || (bConstMidDirAngRad && bConstAxisPlane))    // lines radiating radially from the Bend Axis
    {
      // create the OutSpace Line
      rpNewCurve = new (pContext) SmLine(sOutSpaceFirst, sOutSpaceLast, 3, pContext) ;
    }

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

} // end SmUnbendVolume::MakeExactBSpline1stOutCurve

/*******************************************************************//**
PURPOSE: When possible build the exact Surface produced by projecting               
    rInputSurface to 1st OutSpace                                                   
                                                                                    
NOTES: the BendVolume maps 
  InputSpace Cylinders[center:Outspace axis, radius: Input Plane/Orig dist] to OutSpace planes perp to InSpace Unbend MidBend  axis
  InputSpace planes containing Outspace Axis (radiating radially)           to Outspace planes perp to InSpace Unbend Binormal axis   
  InputSpace planes perp Unbend Axis                                        to OutSpace planes perp to InSpace Unbend BendAxis axis   

  All other surface shapes and orientations are not projected to Analytic shapes and this 
  method sets rpNewSurface == NULL and returns SM_SUCCESS.
***********************************************************************/
SmStatus SmUnbendVolume::MakeExactBSpline1stOutSurface
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

  // locals: ParamSpace and InSpace Unbend orientations - Unbend's Neutral plane is perp to the Unbend's MidDirection
  SmVector3d sParamOrigin  (0,0,0), sUnbendOrigin   = GetUnbendOrigin() ;
  SmVector3d sParamMidDir  (1,0,0), sUnbendMidDir   = GetUnbendMidDir() ;
  SmVector3d sParamAxis    (0,0,1), sUnbendAxis     = GetUnbendAxis() ;
  SmVector3d sParamBinormal(0,1,0), sUnbendBinormal = GetUnbendBinormal() ;
  double     dUnbendNeutralDist = GetUnbendNeutralDist() ;

  // locals: making InputSpace either ParamSpace or InSpace
  SmVector3d sInputOrigin, sInputMidDir, sInputAxis, sInputBinormal ;
  if(eInputSpace == SM_VS_IN_SPACE) { sInputOrigin   = sUnbendOrigin ;
                                      sInputMidDir   = sUnbendMidDir ;
                                      sInputAxis     = sUnbendAxis ;   
                                      sInputBinormal = sUnbendBinormal ;
                                    }
  else /* InputSpce = ParamSpace */ { sInputOrigin   = sParamOrigin ;
                                      sInputMidDir   = sParamMidDir ;
                                      sInputAxis     = sParamAxis ;   
                                      sInputBinormal = sParamBinormal ;
                                    }

  // locals 
  ULONG ii, jj, lSmpSize = 5 ;
  SmVector2d        sSmpUV ;
  SmVector3d        sInputPt, sOutSpacePt, sOnAxisPt, sOnAxisVec, sOnAxisOffset ; 

  SmApproxTol3d     sApproxTol = SmTol::GetApproxTol3d() ;
  const SmContext * pContext = rInputSurface.GetContext() ;
  SmExtent1d        sIvlU = rInputSurface.GetNaturalUVDomain().GetUInterval() ; 
  SmExtent1d        sIvlV = rInputSurface.GetNaturalUVDomain().GetVInterval() ;

  // note: for Unbend mapping Unbend principle axes happen to be the same in InSpace and OutSpace
  double            dAxisPlane, dMidDirPlane, dBinormalPlane, dAxisLine, dMidDirAngRad ; // dist from Sample Pt to planes perp to Unbend Axes.
  SmExtent1d        sAxisPlaneRange, sMidDirPlaneRange, sBinormalPlaneRange, sAxisLineRange, sMidDirAngRadRange ;           // range on distances
  double            dAxisPlaneSum=0.0, dMidDirPlaneSum=0.0, dBinormalPlaneSum=0.0, sAxisLineSum=0.0, dMidDirAngRadSum=0.0 ; // Sum of SamplePt distances to planes
  SmBoolean         bConstAxisPlane = FALSE, bConstAxisLine = FALSE, bConstMidDirAngRad = FALSE;
  SmPseudoBox       sOutSpaceBBox(sUnbendMidDir,       sUnbendBinormal,       sUnbendAxis,         // OutSpace Property
                                  sMidDirPlaneRange, sBinormalPlaneRange, sAxisPlaneRange) ; 

  // See if the InputSurface is perpendicular to any of the Unbend principle axes (parallel to any Unbend principle planes)
  for(ii=0;ii<=lSmpSize;ii++)
    {
      sSmpUV.x = sIvlU.Evaluate((double)ii/(double)lSmpSize) ;

      for(jj=0;jj<=lSmpSize;jj++)
        {
          sSmpUV.y = sIvlV.Evaluate((double)jj/(double)lSmpSize) ;

          // Surface sample 
          rInputSurface.EvaluatePoint(sSmpUV, sInputPt) ;

          // set one OnAxisOrigin
          if(ii==0 && jj==0)
            {
              sOnAxisPt = sInputOrigin + (sInputPt-sInputOrigin).Dot(sInputAxis) * sInputAxis ;
            } 

          // inputVec measured from sOnAxisPt
          sOnAxisVec = sInputPt - sOnAxisPt ;

          // save OutSpace Extents of the corner points
          if( (ii == 0 || ii == lSmpSize) && (jj == 0 || jj == lSmpSize) )
            {
              if(eInputSpace == SM_VS_IN_SPACE)
                { MapPoint(sInputPt, sOutSpacePt, FALSE) ; } // without compounding  
              else
                { EvaluatePoint(sInputPt, sOutSpacePt, FALSE) ; } // wihtout compounding
              sOutSpaceBBox.AddPoint3d(sOutSpacePt-sOnAxisPt) ;

#ifdef SM_DEBUG_CODE
              if(bDebugMe)
                {
                  const SmFace *pFace = (SmFace *)(rInputSurface.GetFace()) ;
                  const SmBrep *pBrep = pFace ? pFace->GetBrep() : NULL ;
                  SmPseudoBox  sDrawOutSpaceBBox(sOutSpaceBBox) ; // translate sOutSpaceBox from sOnAxisPt origin to UnbendOrigin
                  sDrawOutSpaceBBox.Init() ;
                  sDrawOutSpaceBBox.AddPoint3d(sOutSpaceBBox.Evaluate(0,0,0)+sOnAxisPt) ;
                  sDrawOutSpaceBBox.AddPoint3d(sOutSpaceBBox.Evaluate(1,1,1)+sOnAxisPt) ;

                  smgfx_Erase() ;
                  smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
                  smgfx_SetLook(3,4, 0,1,0) ; rInputSurface.DrawUV() ; sm_GraphicsLoop() ;
                  smgfx_SetLook(3,4, 0,1,0) ; rInputSurface.DrawParams() ; sm_GraphicsLoop() ;
                  smgfx_SetLook(5,6, 0,1,0) ; sInputPt.Draw() ;  sm_GraphicsLoop() ;
                  smgfx_SetLook(5,6, 1,0,0) ; sOutSpacePt.Draw() ;  sm_GraphicsLoop() ;
                  smgfx_SetLook(3,4, 1,0,1) ; sDrawOutSpaceBBox.Draw() ; sm_GraphicsLoop() ; 
                  smgfx_SetLook(1,2, 0,1,1) ; this->Draw(TRUE, TRUE) ;  sm_GraphicsLoop() ;

                  sm_GraphicsLoop() ;
                }
#endif // SM_DEBUG_CODE

            }

          // no work - sample point not within bend domain
          SmBoolean bIsIn =   (eInputSpace == SM_VS_IN_SPACE)
                            ? IsPointInInSpaceDomain(sInputPt) 
                            : IsPointInParamDomain(sInputPt) ;
          if(bIsIn == FALSE)
            { return SM_SUCCESS ; }

          // Input distance to plane containing the axis parallel to the NeutralPlane
          //      distance to Origin in Binormal direction (gives cylinder Start and Stop AngleDegs)
          //      distance to Origin in Axis direction
          dMidDirPlane   = sUnbendMidDir.Dot(sOnAxisVec) ;   // distance to OrigPlane perpendicular to MidPlane
          dBinormalPlane = sUnbendBinormal.Dot(sOnAxisVec) ; // distance to OrigPlane perpendicular to BinormalPlane
          dAxisPlane     = sUnbendAxis.Dot(sOnAxisVec) ;     // distance to OrigPlane perpendicular to AxisPlane
          dAxisLine      = smos_Sqrt(dMidDirPlane*dMidDirPlane + dBinormalPlane*dBinormalPlane) ; // distance to Axis line
          sInputAxis.CCWAngleBetween(sInputMidDir, sOnAxisVec, dMidDirAngRad) ; // rot angle from sMidDirAxis

          // accumulate the range of radius values
          //            the range of width values
          //            the range of height values
          sMidDirPlaneRange.  AddValue(dMidDirPlane) ;
          sAxisPlaneRange.    AddValue(dAxisPlane) ;
          sBinormalPlaneRange.AddValue(dBinormalPlane) ;
          sAxisLineRange.     AddValue(dAxisLine) ;
          sMidDirAngRadRange. AddValue(dMidDirAngRad) ;

          // accumlate the sum of Plane distances
          dAxisPlaneSum     += dAxisPlane ;
          dMidDirPlaneSum   += dMidDirPlane ;
          dBinormalPlaneSum += dBinormalPlane ;
          sAxisLineSum      += dAxisLine ;
          dMidDirAngRadSum  += dMidDirAngRad ;

          // remember when curve is const dist from bend plane
          bConstAxisPlane     = sAxisPlaneRange.GetLength()     < sApproxTol ;
          // bConstMidDirPlane   = sMidDirPlaneRange.GetLength()   < sApproxTol ;
          // bConstBinormalPlane = sBinormalPlaneRange.GetLength() < sApproxTol ;
          bConstAxisLine      = sAxisLineRange.GetLength()      < sApproxTol ;
          bConstMidDirAngRad  = sMidDirAngRadRange.GetLength()  < SM_EFF_ZERO_RAD ; 

          // no work - Surface not one of the special case surfaces - hack here, depends on: TRUE = 1, FALSE=0
          if(   !(bConstAxisLine)      // InputSpace cylinder centered on the Bend Axis
             && !(bConstMidDirAngRad)  // InputSpace planes containing Outspace Axis (radiating radially)  
             && !(bConstAxisPlane))    // InputSpace planes perp Bend Axis 
            { return SM_SUCCESS ; }

        } // end iter jj - gather sampels for parallel test
    } // end iter ii - gather samples for parallel test

  // arrive here when rInputSurface is a special case shape that projects to a simple analytic shape
  SM_ASSERT_MSG(   (bConstAxisLine)      // InputSpace cylinder centered on the Bend Axis
                || (bConstMidDirAngRad)  // InputSpace planes containing Outspace Axis (radiating radially)  
                || (bConstAxisPlane),    // InputSpace planes perp Bend Axis 
                _T("Missed a case - this is a bug; check code.")) ;

  // find the current rInputSurface surface normal - it's a plane or a cylinder so sample it once anywhere
  SmVector2d sInputSurfaceMidUV(sIvlU.Evaluate(.5), sIvlV.Evaluate(.5)) ;
  SmVector3d sInputSurfacePt, sInputSurfaceNormal ;             
  rInputSurface.EvaluatePoint(sInputSurfaceMidUV, sInputSurfacePt) ;
  rInputSurface.EvaluateNormal(sInputSurfaceMidUV, TRUE, TRUE, sInputSurfaceNormal) ;

  // create NewSurface locals
  SmBoolean  bPositive = TRUE ;
  SmVector2d sUVScale(1,1) ;
  SmVector3d sXVector, sYVector ;
  SmExtent2d sUVDomain ;

  // InputSpace cylinder centered on the Unbend Axis map to OutSpace planes perp to InSpace Unbend MidBend axis
  if(bConstAxisLine)
    {
      // check orientation of surface (positive = SrfNormals pt outwards from UnbendAxis)
      bPositive = (sInputSurfacePt-sOnAxisPt).Dot(sInputSurfaceNormal) > 0.0 ;

      // offset from the OnAxisPt = Radius * UnbendMidDir
      sOnAxisOffset = sAxisLineRange.GetMid() * sUnbendMidDir ;

      // UVDomain = BBox3d U (UnbendMidDir) and V (UnbendAxis) extents in OutSpace (Positive == TRUE) ;
      sUVDomain.SetUInterval(sOutSpaceBBox.GetInterval(1)) ;
      sUVDomain.SetVInterval(sOutSpaceBBox.GetInterval(2)) ;

      // NewPlane Axes OutSpace (Positive == TRUE) ;
      sXVector = sUnbendBinormal ;
      sYVector = sUnbendAxis ;

    } // end InputSpaceCylinder centered on the UnbendAxis check

  // InputSpace planes containing Outspace Axis (radiating radially) to Outspace planes perp to InSpace Unbend Binormal axis   
  if(bConstMidDirAngRad)
    {
      // check orientation of surface (positive = SrfNormals pt in CCW direction about UnbendAxis
      double dInspaceAngRad = sMidDirAngRadRange.GetMid() ; 
      bPositive = (smos_Cosine(dInspaceAngRad)*sInputBinormal - smos_Sine(dInspaceAngRad)*sInputMidDir).Dot(sInputSurfaceNormal) > 0.0 ;

      // offset from the OnAxisPt = InspaceAngRad * NeutralDist * UnbendBinormal
      sOnAxisOffset = (dInspaceAngRad * dUnbendNeutralDist) * sUnbendBinormal ;

      // UVDomain = BBox3d U (UnbendMidDir) and V (UnbendAxis) extents in OutSpace (Positive == TRUE) ;
      sUVDomain.SetUInterval(sAxisLineRange) ;
      sUVDomain.SetVInterval(sOutSpaceBBox.GetInterval(2)) ;

      // NewPlane Axes OutSpace (Positive == TRUE) ;
      sXVector = sUnbendMidDir ;
      sYVector = sUnbendAxis ;                        

    } // end InputSpace planes containing Outspace Axis check

  // InputSpace planes perp Unbend Axis to OutSpace planes perp to InSpace Unbend BendAxis axis
  if(bConstAxisPlane)
    {
      // check orientation of surface (positive = SrfNormals pt in CCW direction about UnbendAxis
      bPositive = sUnbendAxis.Dot(sInputSurfaceNormal) > 0.0 ;

      // offset from the OnAxisPt = (0,0,0) ;
      sOnAxisOffset.Set(0,0,0) ;

      // Ensure sOutSpaceBBox is large enough in the UnbendMidDir direction
      // The Minimum UnbendMidDir dist for all curves other than lines radiating out from the BendAxis 
      //  can occur at a internal point and not on the curve end points.
      // sOutSpaceBBox currently is defined by the surfaces - corner domain points.
      // 
      // rather than check the rInputSurface's natural boundaries for UnbendMidDir dist minimums and maximums 
      // use the sampled radius values saved in sAxisLineRange.  Bump those limits up a bit in case sampling missed
      // an actual extrema.  Setting the UVDomain too big is not a problem as long as the minimum is bigger than m_dSingularityTol.

      // UVDomain = BBox3d U (AxisLineRange) and V (MidDirAngRange*NeutralDist) extents in OutSpace (Positive == TRUE) ;
      sUVDomain.SetMinMax(  ((sAxisLineRange.GetMin() * .8) > (GetSingularityTol()*1.1))   // in : minX
                          ? (sAxisLineRange.GetMin()  * .8)                                // 
                          : (GetSingularityTol() * 1.1),                                   // 
                          sMidDirAngRadRange.GetMin() * dUnbendNeutralDist,                // in : minY
                          sAxisLineRange.GetMax() * 1.2,                                   // in : maxX
                          sMidDirAngRadRange.GetMax() * dUnbendNeutralDist) ;              // in : maxY

      // NewPlane Axes OutSpace (Positive == TRUE) ;
      sXVector = sUnbendMidDir ;
      sYVector = sUnbendBinormal ;                        

    } // end plane perpendicular to Binormal Axis check

   // when bPositive == FALSE, swap Plane normal
   if(!bPositive)
     {
       SmVector3d sTmp = sXVector ;
                  sXVector = sYVector ;
                  sYVector = sTmp ;
       sUVDomain.Transpose() ;
     } // end bPositive==FALSE check

   // create new plane - positive orientation:[SrfNormal = UnbendMidDir X UnbendBinormal]
   rpNewSurface = new (pContext) SmPlane(sOnAxisPt+sOnAxisOffset, // in : 3d loc of parametric origin = Plane_Evaluate(0,0)
                                         sXVector,                // in : 3D X Axis - made unit
                                         sYVector,                // in : 3D Y Axis - made unit - should be perp to X
                                         sUVScale,                // in : UScale and VScale
                                         sUVDomain,               // in : UV Domain limiting allowed evaluations
                                         pContext) ;              // in : Set context if given, default:[NULL]
#ifdef SM_DEBUG_CODE
  if(bDebugMe)
    {
      const SmFace *pFace = (SmFace *)(rInputSurface.GetFace()) ;
      const SmBrep *pBrep = pFace ? pFace->GetBrep() : NULL ;

      SM_DUMP_AND_ASSERT_VALID(&rInputSurface) ;
      SM_DUMP_AND_ASSERT_VALID(rpNewSurface) ;

      SmVector3d sOnAxis = sOnAxisPt + sOnAxisOffset;

      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(3,4, 0,1,0) ; rInputSurface.DrawUV() ; sm_GraphicsLoop() ;
      smgfx_SetLook(3,4, 0,1,0) ; rInputSurface.DrawParams() ; sm_GraphicsLoop() ;
      smgfx_SetLook(3,4, 1,0,0) ; rpNewSurface->DrawUV() ; sm_GraphicsLoop() ;
      smgfx_SetLook(3,4, 1,0,0) ; rpNewSurface->DrawParams() ; sm_GraphicsLoop() ;
      smgfx_SetLook(3,4, 1,0,0) ; (sOnAxisPt+sOnAxisOffset).Draw() ;
      smgfx_SetLook(3,4, 1,0,0) ; sXVector.Draw(&sOnAxis) ; sm_GraphicsLoop() ;
      smgfx_SetLook(3,4, 1,0,1) ; sYVector.Draw(&sOnAxis) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,1) ; this->Draw(TRUE, TRUE) ;  sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

  // all done
  return(sRtn) ;

} // end SmUnbendVolume::MakeExactBSpline1stOutSurface

/*******************************************************************//**
PURPOSE: compute OutPoint = Evaluate(ParamPoint) - optional derived implementation.

NOTES: 
 Map UVW ParamSpace point to xyz ProjSpace point and iso-derivatives.

  EvaluateSimple : r     = sqrt(u**2 + v**2) ; 
                   theta = ArcTan(v,u) ;       
                   d     = UnbendNeutralDist ;
                      
                   xProj = r ;                 
                   yProj = d * theta ;         
                   zProj = w ;                 

  With:
  X    = (u**2 + v**2)**(1/2)                                      Y    = d * (ArcTan(v/u))                                      
                                                                     
  Xu   = u/(u**2 + v**2)**(1/2)                                    Yu   = d * (-v/(u**2 + v**2))                            
  Xv   = v/(u**2 + v**2)**(1/2)                                    Yv   = d * ( u/(u**2 + v**2))
                                                                      
  Xuu  = 1/(u**2 + v**2)**(1/2) - u*u/(u**2 + v**2)**(3/2)         Yuu  = d * ( 2*u*v/(u**2 + v**2)**2)
  Xuv  = -u*v/(u**2 + v**2)**(3/2)                                 Yuv  = d * (-1/(u**2 + v**2) + 2*v*v/(u**2 + v**2)**2)
  Xvv  = 1/(u**2 + v**2)**(1/2) - v*v/(u**2 + v**2)**(3/2)         Yvv  = d * (-2*u*v/(u**2 + v**2)**2)
                                                               
  Xuuu = -3*u/(u**2 + v**2)**(3/2) + 3*u*u*u/(u**2 + v**2)**(5/2)  Yuuu = d * ( 2*v/(u**2 + v**2)**2 - 8*u*u*v/(u**2 + v**2)**3)
  Xuuv = -  v/(u**2 + v**2)**(3/2) + 3*u*u*v/(u**2 + v**2)**(5/2)  Yuuv = d * ( 2*u/(u**2 + v**2)**2 - 8*u*v*v/(u**2 + v**2)**3)
  Xuvv = -  u/(u**2 + v**2)**(3/2) + 3*u*v*v/(u**2 + v**2)**(5/2)  Yuvv = d * (-2*v/(u**2 + v**2)**2 + 8*u*u*v/(u**2 + v**2)**3)
  Xvvv = -3*v/(u**2 + v**2)**(3/2) + 3*v*v*v/(u**2 + v**2)**(5/2)  Yvvv = d * (-2*u/(u**2 + v**2)**2 + 8*u*v*v/(u**2 + v**2)**3)

  Position: 
    pOut  = UnbendCenter + sqrt(u**2 + v**2) * UnbendMidDir  
                         + d * ArcTan(v/u)   * UnbendBinormal  
                         + w                 * UnbendAxis              

  1st Derivs                                        
    Du(POut)  = Du(xProj)*UnbendMidDir + Du(yProj)*UnbendBinormal ;  
    Dv(POut)  = Dv(xProj)*UnbendMidDir + Dv(yProj)*UnbendBinormal ;
    Dw(POut)  = UnbendAxis ;

    Du(xProj)  = u/(u**2 + v**2)**(1/2) ; Dv(xProj)  = v/(u**2 + v**2)**(1/2) ; Dw(xProj)  = 0 ;
    Du(yProj)  = d * (-v/(u**2 + v**2)) ; Dv(yProj)  = d * ( u/(u**2 + v**2)) ; Dw(yProj)  = 0 ;
    Du(zProj)  = 0 ;                      Dv(zProj)  = 0 ;                      Dw(zProj)  = 1 ;          
                                     
  2nd Derivs:                               
    Duu(POut) = Duu(xProj)*UnbendMidDir + Duu(yProj)*UnbendBinormal ;          
    Duv(POut) = Duv(xProj)*UnbendMidDir + Duv(yProj)*UnbendBinormal ; 
    Duw(POut) = 0 ;            
    Dvv(POut) = Dvv(xProj)*UnbendMidDir + Dvv(yProj)*UnbendBinormal ;  
    Dvw(POut) = 0 ;
    Dww(POut) = 0 ;

    Duu(xProj) =  1/(u**2 + v**2)**(1/2) - u*u/(u**2 + v**2)**(3/2) ;                   
    Duu(yProj) =  d * ( 2*u*v/(u**2 + v**2)**2) ;                   
    Duu(zProj) =  0 ;                             
                                     
    Duv(xProj)  = -u*v/(u**2 + v**2)**(3/2) ;                     
    Duv(yProj)  =  d*(-1/(u**2 + v**2) + 2*v*v/(u**2 + v**2)**2) ;                     
    Duv(zProj)  =  0 ; 
    
    Dvv(xProj) =  1/(u**2 + v**2)**(1/2) - v*v/(u**2 + v**2)**(3/2) ;
    Dvv(yProj) =  d*(-2*u*v/(u**2 + v**2)**2) ;
    Dvv(zProj) =  0 ;                                           
    
    Duw(xProj)  = 0 ;  Dvw(xProj)  = 0 ; Dww(xProj)  = 0 ;
    Duw(yProj)  = 0 ;  Dvw(yProj)  = 0 ; Dww(yProj)  = 0 ;
    Duw(zProj)  = 0 ;  Dvw(zProj)  = 0 ; Dww(zProj)  = 0 ;
                                           
  3rd Derivs:
    Duuu(POut) = Duuu(xProj)*UnbendMidDir + Duuu(yProj)*UnbendBinormal ;
    Duuv(POut) = Duuv(xProj)*UnbendMidDir + Duuv(yProj)*UnbendBinormal ;
    Duuw(POut) = 0 ;
    Duvv(POut) = Duvv(xProj)*UnbendMidDir + Duvv(yProj)*UnbendBinormal ; 
    Duvw(POut) = 0 ;
    Duww(POut) = 0 ;
    Dvvv(POut) = Dvvv(xProj)*UnbendMidDir + Dvvv(yProj)*UnbendBinormal ; 
    Dvvw(POut) = 0 ;
    Dvww(POut) = 0 ;
    Dwww(POut) = 0 ;   

    Duuu(xProj) = -3*u/(u**2 + v**2)**(3/2) + 3*u*u*u/(u**2 + v**2)**(5/2) ;
    Duuu(yProj) =  d*( 2*v/(u**2 + v**2)**2 - 8*u*u*v/(u**2 + v**2)**3) ;
    Duuu(zProj) =  0 ;
  
    Duuv(xProj) = -  v/(u**2 + v**2)**(3/2) + 3*u*u*v/(u**2 + v**2)**(5/2) ;
    Duuv(yProj) =  d*( 2*u/(u**2 + v**2)**2 - 8*u*v*v/(u**2 + v**2)**3) ;
    Duuv(zProj) =  0 ;

    Duvv(xProj) = -  u/(u**2 + v**2)**(3/2) + 3*u*v*v/(u**2 + v**2)**(5/2) ;
    Duvv(yProj) =  d*(-2*v/(u**2 + v**2)**2 + 8*u*u*v/(u**2 + v**2)**3) ;
    Duvv(zProj) =  0 ; 
                
    Dvvv(xProj) = -3*v/(u**2 + v**2)**(3/2) + 3*v*v*v/(u**2 + v**2)**(5/2) ;
    Dvvv(yProj) =  d*(-2*u/(u**2 + v**2)**2 + 8*u*v*v/(u**2 + v**2)**3) ;
    Dvvv(zProj) =  0 ;         
                                         
    Duuw(xProj) = 0 ;  Duvw(xProj) = 0 ;  Duww(xProj)  = 0 ;
    Duuw(yProj) = 0 ;  Duvw(yProj) = 0 ;  Duww(yProj)  = 0 ;
    Duuw(zProj) = 0 ;  Duvw(zProj) = 0 ;  Duww(zProj)  = 0 ;
                                                                    
    Dvvw(xProj) = 0 ;  Dvww(xProj)  = 0 ; Dwww(xProj)  = 0 ;                         
    Dvvw(yProj) = 0 ;  Dvww(yProj)  = 0 ; Dwww(yProj)  = 0 ;                         
    Dvvw(zProj) = 0 ;  Dvww(zProj)  = 0 ; Dwww(zProj)  = 0 ;                         
                                                                    
  InvEvaluateSimple :  r     = xProj ;              u     = r*Cos(theta) ;
                       theta = yProj/d ;            v     = r*Sin(theta) ;
                       d     = UnbendNeutralDist ;    w     = zProj ;       
           
         u = xProj*Cos(yProj/d) ;             
         v = xProj*Sin(yProj/d) ;                 
         w = zProj ;       

 1. At the UnbendOrigin (ParamPoint(0,0,0), this function's derivatives go to zero
    and are undefined, that is as UnbendOrigin is approached in the
    limit the derivative values go to zero and the deriviate directions vary
    depending on the direction of the limit.  As such, the argument
    bNonZeroTangents is not used and derivatives taken at the point UnbendOrigin
    are returned as zero.
 2. The function has a zero point at the UnbendOrigin and no other internal discontinuities.
    As such, the bFromLeft arguments are not used.
***********************************************************************/
SmStatus SmUnbendVolume::Evaluate
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
  if (lHighestDeriv > GW_MAX_DERIV) SER(SM_ERR_INVALID_INPUT);

  // check input - crParamPoint is within the valid ParamDomain (u > 0 implemented as u >= m_dSingularityTol)
  if(   smos_Fabs(crParamPoint.x) < GetLegalMinU()
     && smos_Fabs(crParamPoint.y) < GetLegalMinU())
    {
      SE_MSG(SM_ERR, _T("SmUnbendVolume::Evaluate given out of NaturalParamDomain ParamPoint (u < GetLegalMinU()) - ignoring")) ;
    }

  // For indexing into the volume pt/deriv array:
#define sv(u,v,w) ((u*(lHighestDeriv+1)+v)*(lHighestDeriv+1)+w)

  // locals
  double dD = GetUnbendNeutralDist() ;
  double dU = crParamPoint.x ;
  double dV = crParamPoint.y ;
  double dW = crParamPoint.z ;

  double dR2 = dU*dU + dV*dV  ;    // (u**2 + v**2)
  double dR1 = smos_Sqrt(dR2) ;   // (u**2 + v**2)**(1/2)

  double dATan = smos_ArcTangent2(dV,dU) ;
                                                                                         
  SmVector3d sUnbendMidDir    = GetUnbendMidDir() ;
  SmVector3d sUnbendBinormal  = GetUnbendBinormal() ;
  SmVector3d sUnbendAxis      = GetUnbendAxis() ;
                                                                            
  // init output array 
  ULONG ii, dASize = (lHighestDeriv+1) * (lHighestDeriv+1) * (lHighestDeriv+1) ;
  for(ii=1;ii<dASize;ii++)
    {
      aDerivatives[ii].Set(0,0,0) ;
    }

  // position
  aDerivatives[0] =   GetUnbendOrigin()
                    + dR1      * sUnbendMidDir 
                    + dD*dATan * sUnbendBinormal 
                    + dW       * sUnbendAxis ;

  // 1st derivs
  //  Du(POut)  = Du(xProj)*UnbendMidDir + Du(yProj)*UnbendBinormal ;  
  //  Dv(POut)  = Dv(xProj)*UnbendMidDir + Dv(yProj)*UnbendBinormal ;
  //  Dw(POut)  = UnbendAxis ;
  // 
  //  Du(xProj)  = u/(u**2 + v**2)**(1/2) ; Dv(xProj)  = v/(u**2 + v**2)**(1/2) ; Dw(xProj)  = 0 ;
  //  Du(yProj)  = d * (-v/(u**2 + v**2)) ; Dv(yProj)  = d * ( u/(u**2 + v**2)) ; Dw(yProj)  = 0 ;
  //  Du(zProj)  = 0 ;                      Dv(zProj)  = 0 ;                      Dw(zProj)  = 1 ;          
  if(lHighestDeriv >= 1)
    {
      // double dDi1 = 1.0 / m_dUnbendNeutralDist ; 

      /* Du */ aDerivatives[sv(1,0,0)] = dU/dR1 * sUnbendMidDir - dD*dV/dR2 * sUnbendBinormal ;
      /* Dv */ aDerivatives[sv(0,1,0)] = dV/dR1 * sUnbendMidDir + dD*dU/dR2 * sUnbendBinormal ;
      /* Dw */ aDerivatives[sv(0,0,1)] = sUnbendAxis ;

      // NonZero 2nd derivs
      //   Duu(POut) = Duu(xProj)*UnbendMidDir + Duu(yProj)*UnbendBinormal ;          
      //   Duv(POut) = Duv(xProj)*UnbendMidDir + Duv(yProj)*UnbendBinormal ; 
      //   Dvv(POut) = Dvv(xProj)*UnbendMidDir + Dvv(yProj)*UnbendBinormal ;  
      //
      //   Duu(xProj) =  1/(u**2 + v**2)**(1/2) - u*u/(u**2 + v**2)**(3/2) ;                   
      //   Duu(yProj) =  d * ( 2*u*v/(u**2 + v**2)**2) ;                   
      //                                    
      //   Duv(xProj) = -u*v/(u**2 + v**2)**(3/2) ;                     
      //   Duv(yProj) =  d*(-1/(u**2 + v**2) + 2*v*v/(u**2 + v**2)**2) ;                     
      //   
      //   Dvv(xProj) =  1/(u**2 + v**2)**(1/2) - v*v/(u**2 + v**2)**(3/2) ;
      //   Dvv(yProj) =  d*(-2*u*v/(u**2 + v**2)**2) ;
      if(lHighestDeriv >= 2)
        {
          double dR3 = dR2 * dR1 ;  // (u**2 + v**2)**(3/2)
          double dR4 = dR2 * dR2 ;  // (u**2 + v**2)**(2) == (u**2 + v**2)**(4/2)
          double dUU = dU*dU ;
          double dUV = dU*dV ;
          double dVV = dV*dV ;

          /* Duu */ aDerivatives[sv(2,0,0)] =  (1/dR1 - dUU/dR3) * sUnbendMidDir + ( dD*          2*dUV/dR4 ) * sUnbendBinormal ;
          /* Duv */ aDerivatives[sv(1,1,0)] =  (      - dUV/dR3) * sUnbendMidDir + ( dD*(-1/dR2 + 2*dVV/dR4)) * sUnbendBinormal ;
          /* Dvv */ aDerivatives[sv(0,2,0)] =  (1/dR1 - dVV/dR3) * sUnbendMidDir + (-dD*          2*dUV/dR4 ) * sUnbendBinormal ;

          // NonZreo 3rd derivs
          //   Duuu(POut) = Duuu(xProj)*UnbendMidDir + Duuu(yProj)*UnbendBinormal ;
          //   Duuv(POut) = Duuv(xProj)*UnbendMidDir + Duuv(yProj)*UnbendBinormal ;
          //   Duvv(POut) = Duvv(xProj)*UnbendMidDir + Duvv(yProj)*UnbendBinormal ; 
          //   Dvvv(POut) = Dvvv(xProj)*UnbendMidDir + Dvvv(yProj)*UnbendBinormal ; 
          //   
          //   Duuu(xProj) = -3*u/(u**2 + v**2)**(3/2) + 3*u*u*u/(u**2 + v**2)**(5/2) ;
          //   Duuu(yProj) =  d*( 2*v/(u**2 + v**2)**2 - 8*u*u*v/(u**2 + v**2)**3) ;
          //   
          //   Duuv(xProj) = -  v/(u**2 + v**2)**(3/2) + 3*u*u*v/(u**2 + v**2)**(5/2) ;
          //   Duuv(yProj) =  d*( 2*u/(u**2 + v**2)**2 - 8*u*v*v/(u**2 + v**2)**3) ;
          //   
          //   Duvv(xProj) = -  u/(u**2 + v**2)**(3/2) + 3*u*v*v/(u**2 + v**2)**(5/2) ;
          //   Duvv(yProj) =  d*(-2*v/(u**2 + v**2)**2 + 8*u*u*v/(u**2 + v**2)**3) ;
          //               
          //   Dvvv(xProj) = -3*v/(u**2 + v**2)**(3/2) + 3*v*v*v/(u**2 + v**2)**(5/2) ;
          //   Dvvv(yProj) =  d*(-2*u/(u**2 + v**2)**2 + 8*u*v*v/(u**2 + v**2)**3) ;
          if(lHighestDeriv >= 3)
            {
              double dR5 = dR3 * dR2 ;
              double dR6 = dR3 * dR3 ;

              double dUUU = dUU*dU ;
              double dUVV = dU*dVV ;
              double dUUV = dUU*dV ;
              double dVVV = dVV*dV ;

              /* Duuu */ aDerivatives[sv(3,0,0)] = (-3*dU/dR3 + 3*dUUU/dR5) * sUnbendMidDir + (dD*( 2*dV/dR4 - 8*dUUV/dR6))*sUnbendBinormal ;
              /* Duuv */ aDerivatives[sv(2,1,0)] = (-  dV/dR3 + 3*dUUV/dR5) * sUnbendMidDir + (dD*( 2*dU/dR4 - 8*dUVV/dR6))*sUnbendBinormal ;
              /* Duvv */ aDerivatives[sv(1,2,0)] = (-  dU/dR3 + 3*dUVV/dR5) * sUnbendMidDir + (dD*(-2*dV/dR4 + 8*dUUV/dR6))*sUnbendBinormal ;
              /* Dvvv */ aDerivatives[sv(0,3,0)] = (-3*dV/dR3 + 3*dVVV/dR5) * sUnbendMidDir + (dD*(-2*dU/dR4 + 8*dUVV/dR6))*sUnbendBinormal ;
                                                
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

} // end SmUnbendVolume::Evaluate

/*******************************************************************//**
PURPOSE: compute ProjPoint = EvaluateSimple(ParamPoint) - requires derived implementation.

NOTES: 
 Map UVW ParamSpace point to xyz ProjSpace point and iso-derivatives.

  EvaluateSimple : r     = sqrt(u**2 + v**2) ; 
                   theta = ArcTan(v,u) ;       
                   d     = UnbendNeutralDist ;
                      
                   xProj = r ;                 
                   yProj = d * theta ;         
                   zProj = w ;                 

  With:
  X    = (u**2 + v**2)**(1/2)                                      Y    = d * (ArcTan(v/u))                                      
                                                                     
  Xu   = u/(u**2 + v**2)**(1/2)                                    Yu   = d * (-v/(u**2 + v**2))                            
  Xv   = v/(u**2 + v**2)**(1/2)                                    Yv   = d * ( u/(u**2 + v**2))
                                                                      
  Xuu  = 1/(u**2 + v**2)**(1/2) - u*u/(u**2 + v**2)**(3/2)         Yuu  = d * ( 2*u*v/(u**2 + v**2)**2)
  Xuv  = -u*v/(u**2 + v**2)**(3/2)                                 Yuv  = d * (-1/(u**2 + v**2) + 2*v*v/(u**2 + v**2)**2)
  Xvv  = 1/(u**2 + v**2)**(1/2) - v*v/(u**2 + v**2)**(3/2)         Yvv  = d * (-2*u*v/(u**2 + v**2)**2)
                                                               
  Xuuu = -3*u/(u**2 + v**2)**(3/2) + 3*u*u*u/(u**2 + v**2)**(5/2)  Yuuu = d * ( 2*v/(u**2 + v**2)**2 - 8*u*u*v/(u**2 + v**2)**3)
  Xuuv = -  v/(u**2 + v**2)**(3/2) + 3*u*u*v/(u**2 + v**2)**(5/2)  Yuuv = d * ( 2*u/(u**2 + v**2)**2 - 8*u*v*v/(u**2 + v**2)**3)
  Xuvv = -  u/(u**2 + v**2)**(3/2) + 3*u*v*v/(u**2 + v**2)**(5/2)  Yuvv = d * (-2*v/(u**2 + v**2)**2 + 8*u*u*v/(u**2 + v**2)**3)
  Xvvv = -3*v/(u**2 + v**2)**(3/2) + 3*v*v*v/(u**2 + v**2)**(5/2)  Yvvv = d * (-2*u/(u**2 + v**2)**2 + 8*u*v*v/(u**2 + v**2)**3)

  Position:                            
    xProj = sqrt(u**2 + v**2) ;  
    yProj = d * ArcTan(v/u) ;  
    zProj = w ;              

  1st Derivs                                        
    Du(xProj)  = u/(u**2 + v**2)**(1/2) ; Dv(xProj)  = v/(u**2 + v**2)**(1/2) ; Dw(xProj)  = 0 ;
    Du(yProj)  = d * (-v/(u**2 + v**2)) ; Dv(yProj)  = d * ( u/(u**2 + v**2)) ; Dw(yProj)  = 0 ;
    Du(zProj)  = 0 ;                      Dv(zProj)  = 0 ;                      Dw(zProj)  = 1 ;          
                                     
  2nd Derivs:                                              
    Duu(xProj) =  1/(u**2 + v**2)**(1/2) - u*u/(u**2 + v**2)**(3/2) ;                   
    Duu(yProj) =  d * ( 2*u*v/(u**2 + v**2)**2) ;                   
    Duu(zProj) =  0 ;                             
                                     
    Duv(xProj)  = -u*v/(u**2 + v**2)**(3/2) ;                     
    Duv(yProj)  =  d*(-1/(u**2 + v**2) + 2*v*v/(u**2 + v**2)**2) ;                     
    Duv(zProj)  =  0 ; 
    
    Dvv(xProj) =  1/(u**2 + v**2)**(1/2) - v*v/(u**2 + v**2)**(3/2) ;
    Dvv(yProj) =  d*(-2*u*v/(u**2 + v**2)**2) ;
    Dvv(zProj) =  0 ;                                           
    
    Duw(xProj)  = 0 ;  Dvw(xProj)  = 0 ; Dww(xProj)  = 0 ;
    Duw(yProj)  = 0 ;  Dvw(yProj)  = 0 ; Dww(yProj)  = 0 ;
    Duw(zProj)  = 0 ;  Dvw(zProj)  = 0 ; Dww(zProj)  = 0 ;
                                           
  3rd Derivs:
    Duuu(xProj) = -3*u/(u**2 + v**2)**(3/2) + 3*u*u*u/(u**2 + v**2)**(5/2) ;
    Duuu(yProj) =  d*( 2*v/(u**2 + v**2)**2 - 8*u*u*v/(u**2 + v**2)**3) ;
    Duuu(zProj) =  0 ;
  
    Duuv(xProj) = -  v/(u**2 + v**2)**(3/2) + 3*u*u*v/(u**2 + v**2)**(5/2) ;
    Duuv(yProj) =  d*( 2*u/(u**2 + v**2)**2 - 8*u*v*v/(u**2 + v**2)**3) ;
    Duuv(zProj) =  0 ;

    Duvv(xProj) = -  u/(u**2 + v**2)**(3/2) + 3*u*v*v/(u**2 + v**2)**(5/2) ;
    Duvv(yProj) =  d*(-2*v/(u**2 + v**2)**2 + 8*u*u*v/(u**2 + v**2)**3) ;
    Duvv(zProj) =  0 ; 
                
    Dvvv(xProj) = -3*v/(u**2 + v**2)**(3/2) + 3*v*v*v/(u**2 + v**2)**(5/2) ;
    Dvvv(yProj) =  d*(-2*u/(u**2 + v**2)**2 + 8*u*v*v/(u**2 + v**2)**3) ;
    Dvvv(zProj) =  0 ;         
                                         
    Duuw(xProj) = 0 ;  Duvw(xProj) = 0 ;  Duww(xProj)  = 0 ;
    Duuw(yProj) = 0 ;  Duvw(yProj) = 0 ;  Duww(yProj)  = 0 ;
    Duuw(zProj) = 0 ;  Duvw(zProj) = 0 ;  Duww(zProj)  = 0 ;
                                                                    
    Dvvw(xProj) = 0 ;  Dvww(xProj)  = 0 ; Dwww(xProj)  = 0 ;                         
    Dvvw(yProj) = 0 ;  Dvww(yProj)  = 0 ; Dwww(yProj)  = 0 ;                         
    Dvvw(zProj) = 0 ;  Dvww(zProj)  = 0 ; Dwww(zProj)  = 0 ;                         
                                                                    
  InvEvaluateSimple :  r     = xProj ;              u     = r*Cos(theta) ;
                       theta = yProj/d ;            v     = r*Sin(theta) ;
                       d     = UnbendNeutralDist ;    w     = zProj ;       
           
         u = xProj*Cos(yProj/d) ;             
         v = xProj*Sin(yProj/d) ;                 
         w = zProj ;       

 1. At the UnbendOrigin (ParamPoint(0,0,0), this function's derivatives go to zero
    and are undefined, that is as UnbendOrigin is approached in the
    limit the derivative values go to zero and the deriviate directions vary
    depending on the direction of the limit.  As such, the argument
    bNonZeroTangents is not used and derivatives taken at the point UnbendOrigin
    are returned as zero.
 2. The function has a zero point at the UnbendOrigin and no other internal discontinuities.
    As such, the bFromLeft arguments are not used.
***********************************************************************/
SmStatus SmUnbendVolume::EvaluateSimple
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
  if (lHighestDeriv > GW_MAX_DERIV) SER(SM_ERR_INVALID_INPUT) ;

  // For indexing into the volume pt/deriv array:
#define sv(u,v,w) ((u*(lHighestDeriv+1)+v)*(lHighestDeriv+1)+w)

  // locals
  double dD = GetUnbendNeutralDist() ;
  double dU = crParamPoint.x ; // needed so that &crParamPoint can be same memory as aDerivatives
  double dV = crParamPoint.y ; // needed so that &crParamPoint can be same memory as aDerivatives
  double dW = crParamPoint.z ; // needed so that &crParamPoint can be same memory as aDerivatives

  double dR2 = dU*dU + dV*dV  ;    // (u**2 + v**2)
  double dR1 = smos_Sqrt(dR2) ;   // (u**2 + v**2)**(1/2)

  double dATan = smos_ArcTangent2(dV,dU) ;
             
  // init output array
  ULONG ii, dESize = (lHighestDeriv+1) * (lHighestDeriv+1) * (lHighestDeriv+1) ;
  for(ii=1;ii<dESize;ii++)
    {
      aDerivatives[ii].Set(0,0,0) ;
    }

  // position
  aDerivatives[0].Set(dR1, dD*dATan, dW) ;

  // 1st Derivs                                        
  //   Du(xProj)  = u/(u**2 + v**2)**(1/2) ; Dv(xProj)  = v/(u**2 + v**2)**(1/2) ; Dw(xProj)  = 0 ;
  //   Du(yProj)  = d * (-v/(u**2 + v**2)) ; Dv(yProj)  = d * ( u/(u**2 + v**2)) ; Dw(yProj)  = 0 ;
  //   Du(zProj)  = 0 ;                      Dv(zProj)  = 0 ;                      Dw(zProj)  = 1 ;          
  if(lHighestDeriv >= 1)
    {
      /* Du */  aDerivatives[sv(1,0,0)].Set(dU/dR1, -dD*dV/dR2, 0) ; 
      /* Dv */  aDerivatives[sv(0,1,0)].Set(dV/dR1,  dD*dU/dR2, 0) ; 
      /* Dw */  aDerivatives[sv(0,0,1)].Set(0,     0,          1) ; 

      // NonZero 2nd Derivs:                                              
      //   Duu(xProj) =  1/(u**2 + v**2)**(1/2) - u*u/(u**2 + v**2)**(3/2) ;                   
      //   Duu(yProj) =  d * ( 2*u*v/(u**2 + v**2)**2) ;                   
      //   Duu(zProj) =  0 ;                             
      //                                    
      //   Duv(xProj)  = -u*v/(u**2 + v**2)**(3/2) ;                     
      //   Duv(yProj)  =  d*(-1/(u**2 + v**2) + 2*v*v/(u**2 + v**2)**2) ;                     
      //   Duv(zProj)  =  0 ; 
      //   
      //   Dvv(xProj) =  1/(u**2 + v**2)**(1/2) - v*v/(u**2 + v**2)**(3/2) ;
      //   Dvv(yProj) =  d*(-2*u*v/(u**2 + v**2)**2) ;
      //   Dvv(zProj) =  0 ;                                           
      if(lHighestDeriv >= 2)
        {
          double dR3 = dR2 * dR1 ;  // (u**2 + v**2)**(3/2)
          double dR4 = dR2 * dR2 ;  // (u**2 + v**2)**(2) == (u**2 + v**2)**(4/2)
          double dUU = dU*dU ;
          double dUV = dU*dV ;
          double dVV = dV*dV ;

          /* Duu */ aDerivatives[sv(2,0,0)].Set(  1/dR1 - dUU/dR3,  dD*          2*dUV/dR4,  0 ) ;
          /* Duv */ aDerivatives[sv(1,1,0)].Set( -        dUV/dR3,  dD*(-1/dR2 + 2*dVV/dR4), 0 ) ;
          /* Dvv */ aDerivatives[sv(0,2,0)].Set(  1/dR1 - dVV/dR3, -dD*          2*dUV/dR4,  0 ) ;

          /* Duw */ // aDerivatives[sv(1,0,1)].Set( 0, 0, 0) ;
          /* Dvw */ // aDerivatives[sv(0,1,1)].Set( 0, 0, 0) ;
          /* Dww */ // aDerivatives[sv(0,0,2)].Set( 0, 0, 0) ;
                                           

          // NonZero 3rd Derivs:
          //   Duuu(xProj) = -3*u/(u**2 + v**2)**(3/2) + 3*u*u*u/(u**2 + v**2)**(5/2) ;
          //   Duuu(yProj) =  d*( 2*v/(u**2 + v**2)**2 - 8*u*u*v/(u**2 + v**2)**3) ;
          //   Duuu(zProj) =  0 ;
          // 
          //   Duuv(xProj) = -  v/(u**2 + v**2)**(3/2) + 3*u*u*v/(u**2 + v**2)**(5/2) ;
          //   Duuv(yProj) =  d*( 2*u/(u**2 + v**2)**2 - 8*u*v*v/(u**2 + v**2)**3) ;
          //   Duuv(zProj) =  0 ;
          // 
          //   Duvv(xProj) = -  u/(u**2 + v**2)**(3/2) + 3*u*v*v/(u**2 + v**2)**(5/2) ;
          //   Duvv(yProj) =  d*(-2*v/(u**2 + v**2)**2 + 8*u*u*v/(u**2 + v**2)**3) ;
          //   Duvv(zProj) =  0 ; 
          //               
          //   Dvvv(xProj) = -3*v/(u**2 + v**2)**(3/2) + 3*v*v*v/(u**2 + v**2)**(5/2) ;
          //   Dvvv(yProj) =  d*(-2*u/(u**2 + v**2)**2 + 8*u*v*v/(u**2 + v**2)**3) ;
          //   Dvvv(zProj) =  0 ;         
          if(lHighestDeriv >= 3)
            {
              double dR5 = dR3 * dR2 ;
              double dR6 = dR3 * dR3 ;

              double dUUU = dUU*dU ;
              double dUVV = dU*dVV ;
              double dUUV = dUU*dV ;
              double dVVV = dVV*dV ;

              /* Duuu */ aDerivatives[sv(3,0,0)].Set(-3*dU/dR3 + 3*dUUU/dR5, dD*( 2*dV/dR4 - 8*dUUV/dR6), 0 ) ;
              /* Duuv */ aDerivatives[sv(2,1,0)].Set(-  dV/dR3 + 3*dUUV/dR5, dD*( 2*dU/dR4 - 8*dUVV/dR6), 0 ) ;
              /* Duvv */ aDerivatives[sv(1,2,0)].Set(-  dU/dR3 + 3*dUVV/dR5, dD*(-2*dV/dR4 + 8*dUUV/dR6), 0 ) ;
              /* Dvvv */ aDerivatives[sv(0,3,0)].Set(-3*dV/dR3 + 3*dVVV/dR5, dD*(-2*dU/dR4 + 8*dUVV/dR6), 0 ) ;

              /* Duuw */ // aDerivatives[sv(2,0,1)].Set( 0, 0, 0) ;
              /* Duvw */ // aDerivatives[sv(1,1,1)].Set( 0, 0, 0) ;
              /* Duww */ // aDerivatives[sv(1,0,2)].Set( 0, 0, 0) ;
              /* Dvvw */ // aDerivatives[sv(0,2,1)].Set( 0, 0, 0) ;
              /* Dvww */ // aDerivatives[sv(0,1,2)].Set( 0, 0, 0) ;
              /* Dwww */ // aDerivatives[sv(0,0,3)].Set( 0, 0, 0) ;
                                                

            } // end 3rd derivs needed check        
        } // end 2nd derivs needed check
    } // end 1st derivs needed check

  // we don't go any higher all higher derivatives set to zero

  // done with local indexing macro
#undef sv

  // all done
  return SM_SUCCESS ;

} // end SmUnbendVolume::EvaluateSimple

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
SmStatus SmUnbendVolume::EvaluateBoundingBoxSimple
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
  SmPseudoBox sPBox;
  if(   pProjPseudoBox
     && pOptParamPseudoBox == pProjPseudoBox) { sPBox = *pOptParamPseudoBox ; }

  // indirection: from here on out - pBox  == ParamSpace Box
  //                               - pPBox == ParamSpace PseudoBox

  // when pBox is unbounded return unbounded BBoxes
  if(!pBox->IsBounded())
    {
      if(pProjBox)       { pProjBox->SetUnbounded() ; }
      if(pProjPseudoBox) { SmExtent1d sIvl ;
                           sIvl.SetUnbounded() ;
                           pProjPseudoBox->SetBasis(GetUnbendMidDir(), GetUnbendBinormal(), GetUnbendAxis()) ; 
                           pProjPseudoBox->SetIntervals(sIvl, sIvl, sIvl) ; 
                         }
      // all done
      return(SM_SUCCESS) ; 
    }
  
  // arrive here given a UVWDomain which maps to a thick wall cylinder in XYZ space.
  // Union together the bounding boxes of all the UVW box corner edges mapped to XYZ space

  // init output
  if(pProjBox)       { pProjBox->Init() ; }
  if(pProjPseudoBox) { pProjPseudoBox->Init() ; } // init intervals, leave basis vectors alone

  // locals 
  SmContext  sContext ;
  ULONG ii, jj, kk, cnt=0 ;
  SmLine     *pLine = NULL ;
  SmExtent1d sIvl(0.0, 1.0) ;
  SmPoint3d  sPtMin, sPtMax, sLineVec ;
  SmExtent3d  sThisBBox,      *pThisBBox      = pProjBox       ? &sThisBBox      : NULL ;
  SmPseudoBox sThisPseudoBox, *pThisPseudoBox = pProjPseudoBox ? &sThisPseudoBox : NULL ;

  // temporarily make this a simple map
  // GWC:temp change - the next two lines do not compile on gcc - there might be a missing precompile - this has to be fixed.
  // SmTemporaryChangeValue<SmTransform *> sChangeOrient((SmTransform *)m_pOrientMap, NULL) ;
  // SmTemporaryChangeValue<SmVolume *>    sChangeNext  ((SmVolume *)   m_pNextMap,   NULL) ;

  // for every box edge (marked by box corner pairs
  for(ii=0;ii<3;ii++)
    {
      for(jj=0;jj<2;jj++)
        {
          for(kk=0;kk<2;kk++,cnt++)
            {
              // get UVWBox corner point pairs bounding edges
              if(ii==0)      { sPtMin   = pBox->Evaluate(0,jj,kk) ;
                               sPtMax   = pBox->Evaluate(1,jj,kk) ;
                             }
              else if(ii==1) { sPtMin = pBox->Evaluate(jj,0,kk) ;
                               sPtMax = pBox->Evaluate(jj,1,kk) ;
                             }
              else           { sPtMin = pBox->Evaluate(jj,kk,0) ;
                               sPtMax = pBox->Evaluate(jj,kk,1) ;
                             }

              // Build line from one UVWDomain corner to the other
              SmLine::CreateLineSegment(sContext, 3, sPtMin, sPtMax, pLine) ;

              // map line to ProjSpace space (OK because this has been temporarily made a Simple Map)
              SmCrvInVolume sMapLine(*pLine,                  // in : projected Curve   - When(lOwnerFlag&1) rCurve->m_pOwner = this
                                     TRUE,                    // in : TRUE = rCurve is in rVolume's ParamSpace, FALSE= in InSpace
                                     *(SmUnbendVolume*)this,  // in : projecting Volume - When(lOwnerFlag&2) rVolume->m_pOwner = this
                                     0,                       // in : 0 = saves curve and volume without copying
                                     1) ;                     // in : 1 = delete curve but not volume when destructed

              // Get mapped line's bounding box
              sMapLine.CalculateBoundingBox(pLine->GetNaturalInterval(), pThisBBox, pThisPseudoBox, NULL, FALSE) ;

              // union the line BBoxes into the output
              if(pProjBox)       { if(cnt==0) { *pProjBox = *pThisBBox ; } 
                                   else       {  pProjBox->Union(*pThisBBox, *pProjBox) ; }
                                 }
              if(pProjPseudoBox) { if(cnt==0) { *pProjPseudoBox = *pThisPseudoBox ; } 
                                   else       {  pProjPseudoBox->Union(*pThisPseudoBox, pProjPseudoBox->GetBasis(), *pProjPseudoBox) ; }
                                 }
            } // end iter kk
        } // end iter jj
    } // end iter ii - all edges of the input box

  // all done
  return(SM_SUCCESS) ;

} // end SmUnbendVolume::EvaluateBoundingBoxSimple

/*******************************************************************//**
PURPOSE: Find the portion of the InSpace Line interval that maps to the NaturalParamDomain

NOTES: Unbend ParamDomain is unbounded with discontinuities along the
       UnbendAxis and the Y=0 1/2 plane where X<0.

       The CurrentIvl is split where ever it intersects the UnbendAxis or
       the UnbendDiscontinuity HalfPlane.

RETURNS: SM_ERR when crCurrentIvl is trimmed to the empty set, which happens
         when the InSpaceLine is coincident with the UnbendAxis 
         else returns SM_SUCCESS
***********************************************************************/
SmStatus SmUnbendVolume::FindParamIntervalForInSpaceLineSimple
 (const SmPoint3d      & crInSpacePoint,   // in : LinePoint         of Line(s) = LinePoint + s * LineVector               
  const SmVector3d     & crInSpaceVector,  // in : scaled LineVector of Line(s) = LinePoint + s * LineVector               
  const SmExtent1d     & crCurrentIvl,     // in : current limits on s interval
  SmTArray<SmExtent1d> & rTrimIvls)        // out: Interval of line that maps legally within the NaturalParamDomains
 const                                     
{ 
  // init output
  rTrimIvls.ReSet() ;

  // locals
  ULONG ii, jj ; 
  SmVector3d      sUnbendOrig     = GetOrientOrigin() ;
  SmVector3d      sUnbendAxis     = GetOrientZAxis() ;
  SmVector3d      sUnbendBiNorm   = GetOrientYAxis() ;
  SmExtent1d      sInSpaceIvl     = crCurrentIvl ;
  SmVector3d      sInSpaceUnitVec = crInSpaceVector ;
  double          dLen            = crInSpaceVector.Length() ;
  double          dZoneTol3d      = GetLegalMinU() ;
  SmBoolean       bNeedsMoreIntersections ;
  SmSolutionArray sSolutions ;
  SmTArray<double> sXSectParams ; 
   
  // check input - unit vector
  if(!SM_IS_ZERO(dLen - 1.0))
    {
      // adjust line scaling
      sInSpaceUnitVec /= dLen ;
      sInSpaceIvl.Scale(dLen) ;
    }
  else // degenerate line branch
    {
      rTrimIvls.Add(crCurrentIvl) ; // Valid domain = All space
      return(SM_SUCCESS) ;
    }

  // bounded line to classify
  SmLine sInSpaceLine(crInSpacePoint,   // in : P     of line = P + s*scale*unitV
                      sInSpaceUnitVec,  // in : V     of line = P + s*scale*unitV
                      sInSpaceIvl) ;    // in : limits on s, crInterval.Min <= s <= crInterval.Max

  // unbounded bend Axis line
  SmLine sInSpaceUnbendLine(sUnbendOrig, sUnbendAxis) ;

  // intersect the InSpaceLine with the UnbendAxis line
  sInSpaceUnbendLine.IntersectWithLine(sInSpaceUnbendLine.GetNaturalInterval(), // in : line interval in Nurb Domain                        
                                       sInSpaceLine,                            // in : other line to intersect                             
                                       sInSpaceIvl,                             // in : other line interval in Nurb Domain                  
                                       dZoneTol3d,                              // in : Find points where curves are within this 3D distance
                                       bNeedsMoreIntersections,                 // out: TRUE = pass call to general curve/curve intersector 
                                                                                //      FALSE= intersections found here                     
                                       sSolutions) ;                            // out: solutions: sSol.m_vStart[0] = this param
                                                                                //                 sSol.m_vStart[1] = other param  
                                                                                
  // accumulate Line/Unbend Axis solutions
  if(sSolutions.GetSize() > 0)
    {
      SM_ASSERT_MSG(sSolutions.GetSize() == 1, _T("SmUnbendVolume::FindParamIntervalForInSpaceLineSimple: Expected only 1 line/line solution")) ; 
      SmSolution &rSol = sSolutions[0] ;
      
      // for point solutions - save the param
      if(rSol.m_eSolutionType == SM_ST_SINGLE_VALUE)
        {
          sXSectParams.Add(rSol.m_vStart[0]) ; 
        }
      
      else // range solution - Line is coincident with Unbend axis - not legal - no solution
        {
          SM_ASSERT_MSG(rSol.m_eSolutionType == SM_ST_RANGE_OF_VALUES, _T("SmUnbendVolume::FindParamIntervalForInSpaceLineSimple: Expected a range solution")) ;
          return(SM_ERR) ;
        }
    
    } // end lines intersect check
                                                                                             
  // build the Unbend Y=0 domain infinite discontinuity plane (discontinuous for half plane x<0)
  SmPlane sInSpaceUnbendXZPlane(sUnbendOrig, sUnbendBiNorm) ;

  // intersect InSpaceLine with the Unbend Y=0 plane
  sInSpaceUnbendXZPlane.GlobalLineIntersect(sInSpaceUnbendXZPlane.GetNaturalUVDomain(), // in : Plane's Domain
                                            crInSpacePoint,                             // in : Point on the infinite line
                                            sInSpaceUnitVec,                            // in : Direction vector of the infinite line
                                            NULL,                                       // in : If specified bounds the line to a specific segment
                                            FALSE,                                      // in : TRUE = bounded at StartPt and proceeds along Vec to infinity.
                                                                                        //      FALSE= bounded by cpOptLineInterval if given, else infinite.
                                            dZoneTol3d,                                 // in : Max Dist at which two points still intersect
                                            sSolutions) ;                               // out: solutions: sSol.m_vStart[0] = Line param
                                                                                        //                 sSol.m_vStart[1] = Plane param U
                                                                                        //                 sSol.m_vStart[2] = Plane param V

  // accumulate Line/Unbend Y=0 solutions
  if(sSolutions.GetSize() > 0)
    {
      SM_ASSERT_MSG(sSolutions.GetSize() == 1, _T("SmUnbendVolume::FindParamIntervalForInSpaceLineSimple: Expected only 1 line/plane solution")) ; 
      SmSolution &rSol = sSolutions[0] ;
      
      // for point solutions - if PlaneU < 0, save the param w
      if(rSol.m_eSolutionType == SM_ST_SINGLE_VALUE)
        {
          if(rSol.m_vStart[1] < 0.0)
            {
              sXSectParams.Add(rSol.m_vStart[0]) ;
            } 
        } // end Line/Plane pt intersect check
      
      else // range solution - Line is coincident with Unbend Y=0 plane - split it at the X=0 plane
        {
          SM_ASSERT_MSG(rSol.m_eSolutionType == SM_ST_RANGE_OF_VALUES, _T("SmUnbendVolume::FindParamIntervalForInSpaceLineSimple: Expected a range solution")) ;

          // build the Unbend X=0 domain infinite discontinuity plane (discontinuous for half plane x<0)
          SmPlane sInSpaceUnbendYZPlane(sUnbendOrig, GetOrientXAxis()) ;

          // intersect InSpaceLine with the Unbend Y=0 plane
          sInSpaceUnbendYZPlane.GlobalLineIntersect(sInSpaceUnbendYZPlane.GetNaturalUVDomain(), // in : Plane's Domain
                                                    crInSpacePoint,                             // in : Point on the infinite line
                                                    sInSpaceUnitVec,                            // in : Direction vector of the infinite line
                                                    NULL,                                       // in : If specified bounds the line to a specific segment
                                                    FALSE,                                      // in : TRUE = bounded at StartPt and proceeds along Vec to infinity.
                                                                                                //      FALSE= bounded by cpOptLineInterval if given, else infinite.
                                                    dZoneTol3d,                                 // in : Max Dist at which two points still intersect
                                                    sSolutions) ;                               // out: solutions: sSol.m_vStart[0] = Line param
                                                                                                //                 sSol.m_vStart[1] = Plane param U
                                                                                                //                 sSol.m_vStart[2] = Plane param V
          if(sSolutions.GetSize() > 0)
            {       
              SM_ASSERT_MSG(rSol.m_eSolutionType == SM_ST_SINGLE_VALUE, _T("SmUnbendVolume::FindParamIntervalForInSpaceLineSimple: Expected a line/plane pt solution")) 
              sXSectParams.Add(rSol.m_vStart[0]) ;
            }
        } // end Line/Plane coincident check
    } // end lines intersect check
                                                                                             
  // arrive here when sXSectParams contains InSpaceLine params for 
  // InSpaceLine/UnbendAxis and InSpaceLine/UnbendDiscontinuityPlane XSects
  //  Expect 0, 1, or 2

  // Next load rTrimIvls with 1, 2, or 3 intervals spanning all of crCurrentIvl

  // init rTrimIvls with crCurrentIvl
  rTrimIvls.Add(crCurrentIvl) ;

  // for every sXSectParams
  for(ii=0;ii<sXSectParams.GetSize();ii++)
    {
      // for every rTrimIvls
      for(jj=0;jj<rTrimIvls.GetSize();jj++)
        {
          // classify the Pt against the Ivl
          SmExtentPointType ePtType = rTrimIvls[jj].ClassifyPoint(sXSectParams[ii], dZoneTol3d) ;

          // when sXSectParam is within rTrimIvl
          if(SM_EP_INSIDE == ePtType)
            {
              // split the Ivl 
              double dMax = rTrimIvls[jj].GetMax() ;
              SmExtent1d sIvl(sXSectParams[ii], dMax) ;

              rTrimIvls[jj].SetMinMax(rTrimIvls[jj].GetMin(), sXSectParams[ii]) ;
              rTrimIvls.Add(sIvl) ; 

              // done with this sXSectParam - move onto the next
              break ; 
            }
           else if(SM_EP_OUTSIDE != ePtType) // XSect is on Ivl boundary
            {
              // leave this Ivl alone and move onto the next XSectParam
              break ;
            }

          // arrive here when XSectParam is not in this Ivl - keep looking
                
        } // end iter every rTrimIvl

    } // end iter every sXSectParams entry

  // all done
  return(SM_SUCCESS) ;

} // end SmUnbendVolume::FindParamIntervalForInSpaceLineSimple

/*******************************************************************//**
PURPOSE: Trim ParamSpace BBox to Natural Parameter Domain

NOTES: Unbend ParamDomain is unbounded with discontinuities along the
       UnbendAxis and the Y=0 1/2 plane where X<0.  

  There is a choice - Unbend ParamBoxes can or can't contain the
       unbend mapping discontinuities which is determined by
       the intended semantics of the ParamBox.

       In this implementation the ParamBox is built so that it can
       be mapped to InSpace and OutSpace to show the image regions
       of the mapping (unbounded).  An unchosen alternative was
       to interpret the meaning of the ParamBox as a valid region
       of the mapping not containing any discontinuities that have
       to be avoided.

       Callers of this method are responsible for knowing that
       some of the points within a returned Trimmed ParamBox
       may be on the Unbend's discontinuities and to treat those
       points appropriately.  A point being within a Unbend ParamBox is
       no guarantee that the point is not on the Unbend discontinuities. 

RETURNS: SM_ERR when ParamBox is trimmed to the empty set, else returns SM_SUCCESS
***********************************************************************/
SmStatus SmUnbendVolume::TrimParamBoundingBoxSimple
 (const SmExtent3d & crParamBox,              // in : Tgt Param Box to trim
  SmExtent3d       & rParamTrimBox)           // in : Box trimmed to Natural Param Domain
 const  
{
  // Unbend Param is unbounded - just copy the input into the output
  if(&crParamBox != &rParamTrimBox)
    {
      rParamTrimBox = crParamBox ;
    }

  // all done
  return(SM_SUCCESS) ;

// GWC change: the following is code that trimmed the ParamBox to exclude
//             discontinuities.  That meant that a ParamBox that
//             was built to contain a piece of geometry, might not
//             contain that geometry after being trimmed even though
//             every point in the geometry was on a valid Unbend mapping
//             point.
// begin obsolete
//    // locals
//    double dUMin = crParamBox.GetUMin() ;
//    double dVMin = crParamBox.GetVMin() ;
//    double dWMin = crParamBox.GetWMin() ;
//  
//    double dUMax = crParamBox.GetUMax() ;
//    double dVMax = crParamBox.GetVMax() ;
//    double dWMax = crParamBox.GetWMax() ;
//  
//    // discontinuities only interact with ParamBoxes in the negative X half plane
//    if(dUMin < 0.0)
//      {
//        // discontinuities happen when Y bounds cross Y=0
//        if(   dVMin < 0.0
//           && dVMax > 0.0)
//           {
//             // arrive here when ParamBox needs to be trimmed
//             // 3 options - Move XMin
//             //             Move YMin
//             //             Move YMax
//             // Pick the one that leaves the largest ParamBox
//  
//             // locals
//             double dZoneTol3d = GetLegalMinU() ; 
//             double dDelXMin   = (dUMax >  dZoneTol3d) ? (dUMax      -dZoneTol3d)/(dUMax-dUMin) : 0.0 ;
//             double dDelYMin   = (dVMax >  dZoneTol3d) ? (dVMax      -dZoneTol3d)/(dVMax-dVMin) : 0.0 ;
//             double dDelYMax   = (dVMin < -dZoneTol3d) ? (-dZoneTol3d-dVMin     )/(dVMax-dVMin) : 0.0 ;
//  
//             // Trim to the biggest ParamBox not containing a discontinuity
//             if     ((dDelXMin >= dDelYMin) && (dDelXMin >= dDelYMax)) { dUMin =  dZoneTol3d ; }
//             else if((dDelYMin >= dDelXMin) && (dDelYMin >= dDelYMax)) { dVMin =  dZoneTol3d ; }
//             else                                                      { SM_ASSERT((dDelYMax > dDelXMin) && (dDelYMax > dDelYMin)) ;
//                                                                         dVMax = -dZoneTol3d ; 
//                                                                       }
//           } // end ParamBox contains Y=0 check
//      } // end ParamBox contains X<0 values
//  
//    // set output
//    rParamTrimBox.SetMinMax( dUMin, dVMin, dWMin, dUMax, dVMax, dWMax) ;
//  
//    // all done
//    return(SM_SUCCESS) ;
// end obsolete

} // end SmUnbendVolume::TrimParamBoundingBoxSimple                                                 

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
SmStatus SmUnbendVolume::EvaluateIsoParametricCurveSimple
 (const SmContext     & crContext,        // in : context for created objects
  SmVolumeParamsType    eConstantParams,  // in : oneof: SM_VPS_UV_IN,
                                          //             SM_VPS_UW_IN,
                                          //             SM_VPS_VW_IN.
  double                dIsoParam1,       // in : 1st constant SM_VP_U_IN or SM_VP_V_IN parameter value
  double                dIsoParam2,       // in : 2nd constant SM_VP_V_IN or SM_VP_W_IN parameter value
  double                d3DTolerance,     // NotUsed: in : Max ApproxCurve to IdealCurve deviation
  SmCurve            *& rpNewIsoCurve,    // out: the ProjSpace IsoCurve
  const SmExtent3d    * pOptParamDomain)  // in : limiting domain, NULL to ignore. default:[NULL]  
 const
{
  SM_REF1(d3DTolerance) ;
  // init output
  rpNewIsoCurve = NULL ;

  // locals
  SmVector3d  sVec ;
  double      dU = 0.0, dV = 0.0, dW = 0;
  SmExtent1d  sIvl ;
  SmLine     *pLine = NULL ;

  if     (eConstantParams == SM_VPS_UV) { dU = dIsoParam1 ;
                                          dV = dIsoParam2 ;
                                          dW = 0.0 ;
                                          sVec.Set(0,0,1) ;
                                          if(pOptParamDomain) { sIvl = pOptParamDomain->GetWInterval() ; }
                                        }
  else if(eConstantParams == SM_VPS_UW) { dU = dIsoParam1 ;
                                          dV = 0.0 ;
                                          dW = dIsoParam2 ;
                                          sVec.Set(0,1,0) ;
                                          if(pOptParamDomain) { sIvl = pOptParamDomain->GetVInterval() ; }
                                        }
  else if(eConstantParams == SM_VPS_VW) { dU = 0.0 ;
                                          dV = dIsoParam1 ;
                                          dW = dIsoParam2 ;
                                          sVec.Set(1,0,0) ;
                                          if(pOptParamDomain) { sIvl = pOptParamDomain->GetUInterval() ; }
                                        }
  // Build a param space line
  SmPoint3d sPuvw(dU, dV, dW) ;
  sIvl.ApproximateUnbounded() ;
  if(!pOptParamDomain) { pLine = new (crContext) SmLine(sPuvw, sVec) ; }
  else                 { pLine = new (crContext) SmLine(sPuvw, sVec, sIvl) ; }
  
  // temporarily make this a simple map
  // GWC:temp change - the next two lines do not compile on gcc - there might be a missing precompile - this has to be fixed.
  // SmTemporaryChangeValue<SmTransform *> sChangeOrient((SmTransform *)m_pOrientMap, NULL) ;
  // SmTemporaryChangeValue<SmVolume *>    sChangeNext  ((SmVolume *)   m_pNextMap,   NULL) ;

  // map that through the UnbendVolume
  SmCrvInVolume *pCrvInVolume = new (crContext) SmCrvInVolume
                                  (*pLine,            // in : projected Curve   - When(lOwnerFlag&1) rCurve->m_pOwner = this
                                   TRUE,              // in : TRUE = rCurve is in rVolume's ParamSpace, FALSE= in InSpace
                                   *(SmVolume*)this,  // in : projecting Volume - When(lOwnerFlag&2) rVolume->m_pOwner = this
                                   2,                 // 2 = copy volume and save curve orig
                                   3) ;               // 3 = delete both curve and volume when destructed

  // make the pCrvInVolume->m_pVolume a simple map so that pCrvInVolume sits in ProjSpace
  pCrvInVolume->GetVolume()->MakeMapSimple() ;
                                                                
  // set output
  rpNewIsoCurve = pCrvInVolume ;

  // all done
  return SM_SUCCESS ;

} // end SmUnbendVolume::EvaluateIsoParametricCurveSimple

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

    for SmUnbendVolume - Make a BSpline Surface           (exact)
***********************************************************************/
SmStatus SmUnbendVolume::EvaluateIsoParametricSurfaceSimple
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
  SmPoint3d   sOrigin ;
  SmVector3d  sX, sY, sNormal ;
  SmVector2d  sUVScale(1,1) ;
  SmExtent2d  sUVDomain ;
  SmPlane    *pPlane = NULL ;

  if( eConstantParam == SM_VP_U)      { sOrigin.Set(dIsoParam,0,0) ;
                                        sX.Set(0,1,0) ;
                                        sY.Set(0,0,1) ;
                                        sNormal.Set(1,0,0) ;
                                        if(pOptParamDomain) { sUVDomain = pOptParamDomain->GetVWDomain() ; }
                                      }
  else if( eConstantParam == SM_VP_V) { sOrigin.Set(0,dIsoParam,0) ;
                                        sX.Set(0,0,1) ;
                                        sY.Set(1,0,0) ;
                                        sNormal.Set(0,1,0) ;
                                        if(pOptParamDomain) { sUVDomain.SetUInterval(pOptParamDomain->GetWInterval()) ; 
                                                              sUVDomain.SetVInterval(pOptParamDomain->GetUInterval()) ;
                                                            }
                                      }
  else if( eConstantParam == SM_VP_W) { sOrigin.Set(0,0,dIsoParam) ;
                                        sX.Set(1,0,0) ;
                                        sY.Set(0,1,0) ;
                                        sNormal.Set(0,0,1) ;
                                        if(pOptParamDomain) { sUVDomain = pOptParamDomain->GetUVDomain() ; }
                                      }
  // Build a UVW space surface
  if(!pOptParamDomain) { pPlane = new (crContext) SmPlane(sOrigin, sNormal) ; }
  else                 { pPlane = new (crContext) SmPlane(sOrigin, sX, sY, sUVScale, sUVDomain) ; }
  
  // temporarily make this a simple map
  // GWC:temp change - the next two lines do not compile on gcc - there might be a missing precompile - this has to be fixed.
  // SmTemporaryChangeValue<SmTransform *> sChangeOrient((SmTransform *)m_pOrientMap, NULL) ;
  // SmTemporaryChangeValue<SmVolume *>    sChangeNext  ((SmVolume *)   m_pNextMap,   NULL) ;

  // map that through the UnbendVolume
  SmSrfInVolume *pSrfInVolume = new (crContext) SmSrfInVolume
                                                  (*pPlane,           // in : projected Surface - When(lOwnerFlag&1) rSurface->m_pOwner = this
                                                   TRUE,              // in : TRUE = m_pSurface is in m_pVolume's ParamSpace, FALSE = in InSpace
                                                   *(SmVolume*)this,  // in : projecting Volume - When(lOwnerFlag&2) rVolume->m_pOwner = this
                                                   2,                 // in : 2 = copy volume and save surface orig
                                                   3) ;               // in : 3 = delete both surface and volume when destructed

  // make pSrfInVolume->m_pVolume a simple map so that pSrfInVolume sits in projSpace
  pSrfInVolume->GetVolume()->MakeMapSimple() ; 

  // set output
  rpNewIsoSurface = pSrfInVolume ;

  // all done
  return SM_SUCCESS ;

} // end SmUnbendVolume::EvaluateIsoParametricSurfaceSimple

/*******************************************************************//**
PURPOSE: Do a quick approximate inverse mapping from ProjSpace back 
         to ParamSpace for upcoming newton raphson

NOTES: Transform is known - just do the inverse mapping - no guessing,
       Map XYZ ProjSpace point back to UVW InSpace.
***********************************************************************/
SmStatus SmUnbendVolume::InvEvaluateGuessPointSimple
 (const SmPoint3d     & crProjPoint,        // in : ProjSpace Point to map back to ParamSpace Point
  SmTArray<SmPoint3d> & rGuessParamPoints)  // out: ParamSpace Point near Target Point actual map back to ParamSpace
 const
{
  // init output
  rGuessParamPoints.SetSize(1) ;

  // locals
  double dR     = crProjPoint.x ;          
  double dD     = m_dUnbendNeutralDist ;
  double dTheta = crProjPoint.y/dD ;
  
  // the inverse map from ProjSpace to ParamSpace (same as SmBendVolume mapping from ParamSpace to ProjSpace)       
  double dU     = dR*smos_Cosine(dTheta) ;   
  double dV     = dR*smos_Sine(dTheta) ;   
  double dW     = crProjPoint.z ;          

  // inverted ParamPoint
  rGuessParamPoints[0].Set(dU, dV, dW) ;    

  // all done
  return( SM_SUCCESS ) ;

} // end SmUnbendVolume::InvEvaluateGuessPointSimple

/*******************************************************************//**
PURPOSE: inverse map a point from ProjSpace back to ParamSpace

NOTES: Transform is invertible where its not degenerate - just do the inverse mapping - no guessing,
       Map XYZ ProjSpace point back to UVW ParamSpace.

RETURNS: SM_ERR when crProjPoint is on one of the Unbend discontinuities, else inverts the point and returns SM_SUCCESS
***********************************************************************/
SmStatus SmUnbendVolume::GlobalPointSolveSimple // eff: Find volume UVW Point that maps to TargetPoint
 (const SmPoint3d  & crProjPoint,               // in : ProjSpace Point to map back to ParamSpace
  SmBoolean        & rbFoundAnswer,             // out: TRUE = successfully mapped ProjSpace Point to a ParamSpace Point
  SmSolutionArray  & rSolutions,                // out: Contains ParamSpace Found Point
                                                //      rSolution[i].m_eSolutionType           = SM_ST_SINGLE_VALUE                                               
                                                //      rSolution[i].m_vStart.m_dSolutionValue = PointOnVolume.DistanceBetween(crProjPoint);                      
                                                //      rSolution[i].m_vStart[0] =  U of [U,V,W] the found ParamSpace point location                    
                                                //      rSolution[i].m_vStart[1] =  V of [U,V,W] the found ParamSpace point location                    
                                                //      rSolution[i].m_vStart[2] =  W of [U,V,W] the found ParamSpace point location                    
  const SmExtent3d * pOptParamDomain)           // in : ParamSpace domain over which to search for the inverse point
 const                                          //      NULL = use Map's NaturalDomain, default:[NULL]
{
  // init output
  rSolutions.SetSize(1) ;

  // locals
  double dR     = crProjPoint.x ;          
  double dD     = m_dUnbendNeutralDist ;
  double dTheta = crProjPoint.y/dD ;
  double dGap   = 0.0 ;

  // check - crProjPoint must be in a valid domain
  if(!SmUnbendVolume::IsPointInParamDomain(crProjPoint, TRUE))
    {
      SER_MSG(SM_ERR, _T("SmUnbendVolume::GlobalPointSolveSimple - Point is invertible, it is not in the half space ProjPt.x > 0")) ;
    }

  // check - crProjPoint must be in OptParamDomain
  if(   pOptParamDomain
     && !pOptParamDomain->ContainsPoint3d(crProjPoint, SM_EFF_ZERO))
    {
      SER_MSG(SM_ERR, _T("SmUnbendVolume::GlobalPointSolveSimple - Point is not contained in the given optional ParamDomain Box > 0")) ;
    }
  
  // the inverse map from ProjSpace to ParamSpace (same as SmBendVolume mapping from ParamSpace to ProjSpace)       
  double dU     = dR*smos_Cosine(dTheta) ;   
  double dV     = dR*smos_Sine(dTheta) ;   
  double dW     = crProjPoint.z ;          

  // inverted ParamPoint
  SmPoint3d sParamPoint(dU, dV, dW) ;    

  // set output
  rbFoundAnswer = TRUE ;

  rSolutions[0].m_eSolutionType            = SM_ST_SINGLE_VALUE ;
  rSolutions[0].m_lNumVariables            = 3 ;
  rSolutions[0].m_vStart.m_dSolutionValue  = dGap ;
  rSolutions[0].m_vStart.m_adParameters[0] = sParamPoint.x ;
  rSolutions[0].m_vStart.m_adParameters[1] = sParamPoint.y ;
  rSolutions[0].m_vStart.m_adParameters[2] = sParamPoint.z ;
  rSolutions[0].m_lNumObjects              = 1 ;
  rSolutions[0].m_apObjects[0]             = (SmObject *)this ;
  
  // all done
  return(SM_SUCCESS) ;  

} // end SmUnbendVolume::GlobalPointSolveSimple

/*******************************************************************//**
PURPOSE: inverse map a point from ProjSpace back to ParmSpace with a guess point

NOTES: Transform is invertible except at degeneracy - just do the inverse mapping - no guessing,
       Map XYZ OutSpace point back to UVW InSpace.
***********************************************************************/
SmStatus SmUnbendVolume::LocalPointSolveSimple
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
  SmSolutionArray sSolutions ;

  // pass the call along - no need for the guess point
  SER(GlobalPointSolveSimple( crProjPoint, rbFoundAnswer, sSolutions, pOptParamDomain)) ;

  // set output
  rSolution = sSolutions[0] ;

  // all done
  return(SM_SUCCESS) ;

} // end SmUnbendVolume::LocalPointSolveSimple

/*******************************************************************//**
PURPOSE: return TRUE if SmUnbendVolume simple map is bounded

NOTES: If we worked in cylindrical coordinates UnbendVolume would
       be the domain: r > 0
                      Pi > theta > -Pi
                      all of z

       So that's all of space with a singularity boundary at r = 0 (which
       is the ZAxis) and a multivalued pair of boundaries 
       for theta values of Pi and -Pi (which is the 1/2 plane V=0 and U<0)

       Since, U, V, and W are unbounded (with discontinuities)
       this function just returns FALSE
***********************************************************************/
SmBoolean SmUnbendVolume::IsBoundedSimple() const           
{
  // all done
  return(FALSE) ;

} // end SmUnbendVolume::IsBoundedSimple    

/*******************************************************************//**
PURPOSE: return TRUE if Unbend mapping is closed - it's not 

NOTES: UnbendVolumes are not periodic. They map all
       of UVW space to a slice of XYZ space bounded by y = [-d*Pi, +d*Pi ]*UnbendBiNormal
       d = m_vUnbendNeturalDistance

       If we worked in cylindrical coordinates UnbendVolume would
       be the domain: r > 0
                      Pi > theta > -Pi
                      all of z

       So that's all of space with a singularity boundary at r = 0 (which
       is the ZAxis) and a multivalued pair of boundaries 
       for theta values of Pi and -Pi (which is the 1/2 plane V=0 and U<0)

       Since, U, V, and W are unbounded (with discontinuities)
       this function just returns FALSE
***********************************************************************/
SmBoolean SmUnbendVolume::IsClosedSimple
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

} // end SmUnbendVolume::IsClosedSimple    

/*******************************************************************//**
PURPOSE: return TRUE if Unbend mapping is periodic - it's not

NOTES:  UnbendVolumes are not periodic. They map all
       of UVW space to a slice of XYZ space bounded by y = [-d*Pi, +d*Pi ]*UnbendBiNormal
       d = m_vUnbendNeturalDistance

       If we worked in cylindrical coordinates UnbendVolume would
       be the domain: r > 0
                      Pi > theta > -Pi
                      all of z

       So that's all of space with a singularity boundary at r = 0 (which
       is the ZAxis) and a multivalued pair of boundaries 
       for theta values of Pi and -Pi (which is the 1/2 plane V=0 and U<0)

       Since, U, V, and W are unbounded (with discontinuities)
       this function just returns FALSE

***********************************************************************/
SmBoolean SmUnbendVolume::IsPeriodicSimple
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

} // end SmUnbendVolume::IsPeriodicSimple 

/*******************************************************************//**
PURPOSE: return TRUE when ParamPoint is on an Unbend mapping singularity

NOTES: Unbend maps all of UVW space to a slice of XYZ space bounded 
       by y = [-d*Pi, +d*Pi ]*UnbendBiNormal
       d = m_vUnbendNeturalDistance

       If we worked in cylindrical coordinates UnbendVolume would
       be the domain: r > 0
                      Pi > theta > -Pi
                      all of z

       So that's all of space with a singularity boundary at r = 0 (which
       is the ZAxis) and a multivalued pair of boundaries 
       for theta values of Pi and -Pi (which is the 1/2 plane V=0 and U<0)

       this function returns TRUE when ParamPoint is on the WAxis or
       the 1/2 plane V=0 and U<0
***********************************************************************/   
SmBoolean SmUnbendVolume::IsSingularitySimple      
  (const SmPoint3d   & crParamPoint,       // in : UVpoint to test                 
   SmBoolean         & rbSingularU,        // out: TRUE = TargetPoint on RAxis boundary:[r=0] (the WAxis)
   SmBoolean         & rbSingularV,        // out: TRUE = TargetPoint on theta boundary:[-pi pi] (1/2 plane V=0 and U<0)
   SmBoolean         & rbSingularW,        // out: Always FALSE
   double              d3dTol)             // in : min 3d distance between distinct points
  const 
{
  // see if point is on a discontinuity
  SmBoolean bRtn = IsOnBoundarySimple(crParamPoint, rbSingularU, rbSingularV, rbSingularW, &d3dTol, NULL) ;

  // all done
  return(bRtn) ;

} // end SmUnbendVolume::IsSingularitySimple    

/*******************************************************************//**
PURPOSE: return TRUE when ParamPoint is on the boundary of the domain
         which happens at the discontinuities of the Unbend mapping

NOTES:  Boundaries are the discontinuities 
        If we worked in cylindrical coordinates
          r     = 1/2 space where r > 0
          theta = bounded at -Pi and Pi
          z     = infinite
        but we don't so the OnU, OnV, and OnW values don't make a lot of sense
        so for what it's worth:
          let OnU be the r domain.  Set True when ParamPoint is on the Z axis
              OnV be the theta domain.  Set True when ParamPoint is on the V=0 and u<0 half plane
              OnW be the z domain. Always set to FALSE
***********************************************************************/
SmBoolean SmUnbendVolume::IsOnBoundarySimple
  (const SmPoint3d    & crParamPoint,    // in : Volume UVWPoint to test
   SmBoolean          & rbOnU,           // out: TRUE = TargetPoint on RAxis boundary:[r=0] (the WAxis)
   SmBoolean          & rbOnV,           // out: TRUE = TargetPoint on theta boundary:[-pi pi] (1/2 plane V=0 and U<0)
   SmBoolean          & rbOnW,           // out: TRUE = TargetPoint on Z boundary: Z is unbounded (always FALSE)
   double             * pdOptTolerance,  // in : max deviation allowed for point on seam
                                         //      NULL = use SM_EFF_ZERO * 1000 * (1 + maxDimension())
  const SmExtent3d    * pOptParamDomain) // NotUsed: in : ParamSpace domain over which to search for the inverse point
 const                                   //      NULL = use Map's NaturalDomain, default:[NULL]
{
  SM_REF1(pOptParamDomain) ;
  // locals
  double dU   = crParamPoint.x ;       
  double dV   = crParamPoint.y ;    
  double dTol = pdOptTolerance ? smos_Max(*pdOptTolerance, m_dSingularityTol) : m_dSingularityTol ;    

  // set output
  rbOnU = (dU*dU + dV*dV) <= (dTol * dTol) ;             // on r=0 boundary also onWAxis
  rbOnV = (dU < dTol) && SM_IS_ZERO_TO_TOL(dV, dTol) ;   // on theta=[-pi pi] boundaries also 1/2 plane V=0 and U<0
  rbOnW = FALSE ; 
  
  // all done
  return(rbOnU || rbOnV) ;

} // end SmUnbendVolume::IsOnBoundarySimple    

/*******************************************************************//**
PURPOSE: return TRUE when ParamLine maps completely within the
       Natural Domain and does not cross any discontinuities.

NOTES: The SmUnbendVolume's Natural Param Space includes all of Param
       space with a 1/2 infinite discontinuity plane with Y = 0 and
       X < 0 and a singularity line on the Z axis.

       A Line is 'in' the ParamDomain if it does not cross the 
       discontinuities.
***********************************************************************/
SmBoolean SmUnbendVolume::IsLineInParamDomainSimple
 (const SmPoint3d  & crStartParamPoint,       // in : ParamSpace Line StartPoint to test for inclusion                                  
  const SmPoint3d  & crEndParamPoint)         // in : ParamSpace Line EndPoint to test for inclusion
 const                                
{
  // return
  SmBoolean bRtn = TRUE ; 

  // low work - degenerate line
  if(crStartParamPoint.DistanceBetweenSquared(crEndParamPoint) <= SM_EFF_ZERO_SQ)
    {
      // pass the call along
      return(IsPointInParamDomainSimple(crStartParamPoint)) ;
    } 

  // locals
  SmBoolean bStartDiscontinuity = crStartParamPoint.x <= m_dSingularityTol && SM_ARE_SAME(crStartParamPoint.y,m_dSingularityTol) ; 
  SmBoolean bEndDiscontinuity   = crEndParamPoint.x   <= m_dSingularityTol && SM_ARE_SAME(crEndParamPoint.y,  m_dSingularityTol) ; 

  // Does line cross the Y=0 plane - no: both on plane, one on plane, or crosses plane when x<0
  if(   !bStartDiscontinuity && !bEndDiscontinuity       // points are not on the discontinuity
     && crStartParamPoint.y * crEndParamPoint.y < 0.0)   // line crosses y=0 plane
    {
      // check x value at y = 0 crossing 
      double x =   crStartParamPoint.x 
                 + (crEndParamPoint.x - crStartParamPoint.x)*(-crStartParamPoint.y)
                 / (crEndParamPoint.y - crStartParamPoint.y) ;

      // line is still 'in' if Z=0 crossing happens in the x > 0 half plane
      bRtn &= (x >= m_dSingularityTol - SM_EFF_ZERO) ;
    }

  // lines intersecting the z axis are not in the domain
  if(bRtn)
    {
      SmVector3d sZPt(0,0,0), sZVec(0,0,1), sLineVec = crEndParamPoint - crStartParamPoint ;
      double     dZParam, dLineParam ; 

      // find closest point between two lines - returns SM_ERR when lines are parallel or coincident or vecs are degenerate
      SmStatus sStatus = smgu_LineLineClosestPoint(sZPt, sZVec, crStartParamPoint, sLineVec,
                                                   dZParam, dLineParam) ;

      // When Line is parallel or coincident to ZAxis or LineVec is degenerate
      if(sStatus == SM_ERR)
        {
          // When crStartParamPoint is close to ZAxis the parallel pr degenerate ParamLine crosses the singularity
          if(  (crStartParamPoint.x * crStartParamPoint.x + crStartParamPoint.y * crStartParamPoint.y) <= m_dSingularityTol * m_dSingularityTol)
            { bRtn = FALSE ; }
        }
      else // lines have a unique point of closest approach branch
        {
          // when gap is smaller than m_dSingularityTol ParamLine crosses singularity and is not completely 'in' ParamDomain
          SmVector3d sMinGap = crStartParamPoint + dLineParam * sLineVec - dZParam * sZVec ;
          if(sMinGap.LengthSquared() <= (m_dSingularityTol - SM_EFF_ZERO) * (m_dSingularityTol - SM_EFF_ZERO) )
            { bRtn = FALSE ; }
        } // end lines have a unique point of closest approach branch
    }                                                

  // all done
  return(bRtn) ;

} // end SmUnbendVolume::IsLineInParamDomainSimple

/*******************************************************************//**
PURPOSE: Return TRUE when Curve does not cross any volume's boundaries or
         internal C1 discontinuities and is contained within the ParamSpace domain

NOTES: The UnbendVolume mapping is defined over all space with a 1/2 infinite
       discontinuity plane where V = 0 and U < 0.
***********************************************************************/
SmBoolean SmUnbendVolume::IsCurveInParamDomainSimple
 (const SmCurve & crParamCurve)       // in : Tgt ParamSpace Curve to check
 const       
{ 
  // locals
  SM_PTR_ARRAY(sPlanes,       SmPlane,    3) ; 
  SmContext  sContext ;
  SmVector2d sUVScale(1,1) ;
  SmVector3d sU(1,0,0), sW(0,0,1) ;
  SmPoint3d  sPlanePoint(0,0,0) ; 

  // set 1/2 plane valid intervals - note: infinite 1/2 planes are being approximated by +/- SM_BIG_DOUBLE
  SmExtent1d sUIvl( m_dSingularityTol, -SM_BIG_DOUBLE) ;
  SmExtent1d sWIvl(-SM_BIG_DOUBLE,      SM_BIG_DOUBLE) ;
  SmExtent2d sPlaneDomain(sWIvl.GetMin(), sUIvl.GetMin(), sWIvl.GetMax(), sUIvl.GetMax()) ; 

  // make and store 1/2 infinite discontinuity plane
  SmPlane *pPlane = new (sContext) SmPlane(sPlanePoint,
                                           sW,
                                           sU,
                                           sUVScale,
                                           sPlaneDomain,
                                           &sContext) ; 
  sPlanes.Add(pPlane) ;

  // Make pPlanes memory temporary
  SmObjsDelete<SmPlane *> sClean(&sPlanes) ; 

  // arrive here when sPlanes, are built.

  // check for curve crossing discontinuity planes check
  SmBoolean bHasPlaneCrossings = crParamCurve.HasPlaneCrossings(sPlanes) ;
  SmBoolean bRtn               = !bHasPlaneCrossings ;

  // all done - return final classification
  return bRtn ;

 } // end SmUnbendVolume::IsCurveInParamDomainSimple

/*******************************************************************//**
PURPOSE: Return TRUE when Surface does not cross any volume's boundaries or
         internal C1 discontinuities and is contained within the ParamSpace domain

NOTES: The UnbendVolume mapping is defined over all space with a 1/2 infinite
       discontinuity plane where V = 0 and U < 0.
***********************************************************************/
SmBoolean SmUnbendVolume::IsSurfaceInParamDomainSimple
 (const SmSurface & crParamSurface)   // in : Tgt ParamSpace Surface to check
 const 
{ 
  // locals
  SM_PTR_ARRAY(sPlanes,       SmPlane,    3) ; 
  SmContext  sContext ;
  SmVector2d sUVScale(1,1) ;
  SmVector3d sU(1,0,0), sW(0,0,1) ;
  SmPoint3d  sPlanePoint(0,0,0) ; 

  // set 1/2 plane valid intervals - note: infinite 1/2 planes are being approximated by +/- SM_BIG_DOUBLE
  SmExtent1d sUIvl( m_dSingularityTol, -SM_BIG_DOUBLE) ;
  SmExtent1d sWIvl(-SM_BIG_DOUBLE,      SM_BIG_DOUBLE) ;
  SmExtent2d sPlaneDomain(sWIvl.GetMin(), sUIvl.GetMin(), sWIvl.GetMax(), sUIvl.GetMax()) ; 

  // make and store 1/2 infinite discontinuity plane
  SmPlane *pPlane = new (sContext) SmPlane(sPlanePoint,
                                           sW,
                                           sU,
                                           sUVScale,
                                           sPlaneDomain,
                                           &sContext) ; 
  sPlanes.Add(pPlane) ;

  // Make pPlanes memory temporary
  SmObjsDelete<SmPlane *> sClean(&sPlanes) ; 

  // arrive here when sPlanes, are built.

  // check for surface crossing discontinuity planes check
  SmBoolean bHasPlaneCrossings = crParamSurface.HasPlaneCrossings(sPlanes) ;
  SmBoolean bRtn               = !bHasPlaneCrossings ;

  // all done - return final classification
  return bRtn ;

} // end SmUnbendVolume::IsSurfaceInParamDomainSimple

/*******************************************************************//**
PURPOSE: Write SmTransform to given output stream.

NOTES:  
***********************************************************************/
SmStatus SmUnbendVolume::WriteToDB
 (SmDatabaseIO & rDB,              // in : target output stream
  ULONG          lDBVersionNumber) // in : database version to get proper sequence of writes                                                                       
 const
{
  // file type, ASCII or BINARY
  SmFileType eType = rDB.GetFileType();

  // Volume locals
  //   stored in SmVolume::OrientMap    double *pC = (double *)&GetUnbendOrigin() ; 
  //                                    double *pA = (double *)&GetUnbendAxis() ; 
  //                                    double *pS = (double *)&GetUnbendMidDir() ; 
  //                                    double *pB = (double *)&GetUnbendBinormal() ;

  // Create the output file 
  if (eType == SM_ASCII) { std::ostream & rFileOut = *rDB.GetOutStreamPtr() ;

                           //      rFileOut << pC[0] << " " << pC[1] << " " << pC[2] << " " << "UnbendVolume Unbend Center \n" ;
                           //      rFileOut << pA[0] << " " << pA[1] << " " << pA[2] << " " << "UnbendVolume Unbend Axis \n" ;
                           //      rFileOut << pS[0] << " " << pS[1] << " " << pS[2] << " " << "UnbendVolume Unbend StartDir \n" ;
                           //      rFileOut << pB[0] << " " << pB[1] << " " << pB[2] << " " << "UnbendVolume Unbend Binormal \n" ;
                           rFileOut << m_dUnbendNeutralDist  << "UnbendVolume Unbend NeutralDistance \n" ;
                           rFileOut << m_dSingularityTol   << "UnbendVolume SingularityTol: min legal distance from singularity at U = 0\n" ;
                         }                                                                           
  else /* Binary */      { //      SER(rDB.WriteDouble(pC[0])) ;
                           //      SER(rDB.WriteDouble(pC[1])) ;
                           //      SER(rDB.WriteDouble(pC[2])) ;
                           //                                 
                           //      SER(rDB.WriteDouble(pA[0])) ;
                           //      SER(rDB.WriteDouble(pA[1])) ;
                           //      SER(rDB.WriteDouble(pA[2])) ;
                           //                                 
                           //      SER(rDB.WriteDouble(pS[0])) ;
                           //      SER(rDB.WriteDouble(pS[1])) ;
                           //      SER(rDB.WriteDouble(pS[2])) ;
                           //                                 
                           //      SER(rDB.WriteDouble(pB[0])) ;
                           //      SER(rDB.WriteDouble(pB[1])) ;
                           //      SER(rDB.WriteDouble(pB[2])) ;

                           SER(rDB.WriteDouble(m_dUnbendNeutralDist)) ;
                           SER(rDB.WriteDouble(m_dSingularityTol )) ;
                         }

  // Write base class data
  SmVolume::WriteToDB(rDB, lDBVersionNumber) ;

  // all done
  return SM_SUCCESS;

} // end SmUnbendVolume::WriteToDB

/*******************************************************************//**
PURPOSE: static method to Read a SmTransform from a given stream  

NOTES: 
***********************************************************************/
SmStatus SmUnbendVolume::ReadFromDB
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
         || rpNewVolume->IsKindOf(SmUnbendVolume_TYPE)) ? SM_SUCCESS : SM_ERR) ;

  // init output object
  SmUnbendVolume *pUnbendVolume =   (rpNewVolume == NULL)
                                  ? new (crContext) SmUnbendVolume()
                                  : (SmUnbendVolume *)rpNewVolume ;

  // file type
  SmFileType eType = rDB.GetFileType();
  

  // Volume locals
  //      double *pC = (double *)&pUnbendVolume->GetUnbendOrigin() ; 
  //      double *pA = (double *)&pUnbendVolume->GetUnbendAxis() ; 
  //      double *pS = (double *)&pUnbendVolume->GetUnbendMidDir() ; 
  //      double *pB = (double *)&pUnbendVolume->GetUnbendBinormal() ;
  double *pUnbendNeutralDist = &pUnbendVolume->m_dUnbendNeutralDist ;
  double *pSingularityTol  = &pUnbendVolume->m_dSingularityTol ; 

  if (eType == SM_ASCII) 
    {
      std::istream & rFileIn = *rDB.GetInStreamPtr();
      //      rFileIn >> pC[0] >> pC[1] >> pC[2] ;  rDB.GoToNextLine() ;
      //      rFileIn >> pA[0] >> pA[1] >> pA[2] ;  rDB.GoToNextLine() ;
      //      rFileIn >> pS[0] >> pS[1] >> pS[2] ;  rDB.GoToNextLine() ;
      //      rFileIn >> pB[0] >> pB[1] >> pB[2] ;  rDB.GoToNextLine() ;
      rFileIn >> pUnbendNeutralDist[0] ;  rDB.GoToNextLine() ;
      rFileIn >> pSingularityTol[0] ;   rDB.GoToNextLine() ;
    }
  else 
    {
      //      SER(rDB.ReadDouble(pC[0]));
      //      SER(rDB.ReadDouble(pC[1]));
      //      SER(rDB.ReadDouble(pC[2]));
      //                           
      //      SER(rDB.ReadDouble(pA[0]));
      //      SER(rDB.ReadDouble(pA[1]));
      //      SER(rDB.ReadDouble(pA[2]));
      //                           
      //      SER(rDB.ReadDouble(pS[0]));
      //      SER(rDB.ReadDouble(pS[1]));
      //      SER(rDB.ReadDouble(pS[2]));
      //                           
      //      SER(rDB.ReadDouble(pB[0]));
      //      SER(rDB.ReadDouble(pB[1]));
      //      SER(rDB.ReadDouble(pB[2]));

      SER(rDB.ReadDouble(pUnbendNeutralDist[0]));
      SER(rDB.ReadDouble(pSingularityTol[0]));
    }

  // set output
  rpNewVolume = pUnbendVolume ;

  // read base class data
  SmVolume::ReadFromDB(SmVolume_TYPE, rDB, crContext, rpNewVolume, lDBVersionNumber ) ; 

  // all done
  return SM_SUCCESS;

} // end SmUnbendVolume::ReadFromDB

/*******************************************************************//**
PURPOSE: Compute the total size of the memory used by the SmUnbendVolume.

NOTES: includes attribute memory
***********************************************************************/
ULONG SmUnbendVolume::GetMemoryUsed  // rtn: Total Memory being used for all objects
 (ULONG    & rlMemoryAllocated,      // out: bigger size of all allocated memory in bytes
  SmMarkType eMarkType)              // in : uses without increment eMarkType value
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

} // end SmUnbendVolume::GetMemoryUsed

/*******************************************************************//**
PURPOSE: AssertValid() Test reporting static labels
***********************************************************************/
SmAssertReportLabel sAssertUnbendVolume_list[] =
{
  /*  0 */ {SM_AT_UNIT_VECTOR, _T("unit-vector"),        _T("UnbendAxis has to be unit lenghth") },
  /*  1 */ {SM_AT_UNIT_VECTOR, _T("unit-vector"),        _T("UnbendMidDir has to be unit lenghth") },
  /*  2 */ {SM_AT_UNIT_VECTOR, _T("unit-vector"),        _T("UnbendBinormal has to be unit lenghth") },
  /*  3 */ {SM_AT_GEOMETRIC,   _T("orthogonal"),         _T("Unbend Axis/StartDir/Binormal must be mutually orthoganal") },
  /*  4 */ {SM_AT_GEOMETRIC,   _T("orthogonal"),         _T("Unbend Axis/StartDir/Binormal must be right handed") },
  /*  5 */ {SM_AT_GEOMETRIC,   _T("Positive-Distance"),  _T("Unbend NeutralDistance must be PositiveDefinite") },
  /*  6 */ {SM_AT_GEOMETRIC,   _T("Positive-Tolerance"), _T("Unbend SingularityTol must be PositiveDefinite") }
} ;

/*******************************************************************//**
PURPOSE: Make sure m_pNurb is not NULL by calling MakeNurb() when needed.

NOTES: returns TRUE  = OK
                        FALSE = Problem
***********************************************************************/
SmBoolean SmUnbendVolume::AssertValid
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

  // Run SmUnbendVolume checks

  // UnbendAxis is a unit vector
  bRtn &= SM_ASSERT_VALUE_REPORT(0, SM_LEVEL_0, (smos_Fabs(GetUnbendAxis().Dot(GetUnbendAxis())-1.0) < SM_EFF_ZERO), SM_EFF_ZERO, smos_Fabs(GetUnbendAxis().Dot(GetUnbendAxis())-1.0), _T("")) ;

  // UnbendMidDir is a unit vector
  bRtn &= SM_ASSERT_VALUE_REPORT(1, SM_LEVEL_0, (smos_Fabs(GetUnbendMidDir().Dot(GetUnbendMidDir())-1.0) < SM_EFF_ZERO), SM_EFF_ZERO, smos_Fabs(GetUnbendMidDir().Dot(GetUnbendMidDir())-1.0), _T("")) ;

  // UnbendBinormal is a unit vector
  bRtn &= SM_ASSERT_VALUE_REPORT(2, SM_LEVEL_0, (smos_Fabs(GetUnbendBinormal().Dot(GetUnbendBinormal())-1.0) < SM_EFF_ZERO), SM_EFF_ZERO, smos_Fabs(GetUnbendBinormal().Dot(GetUnbendBinormal())-1.0), _T("")) ;

  // basis triple product - should be +1
  double dTripleProduct = GetUnbendMidDir().TripleProduct(GetUnbendBinormal(), GetUnbendAxis()) ; 

  // basis vectors have to be mutually orthogonal
  bRtn &= SM_ASSERT_VALUE_REPORT(3, SM_LEVEL_0, (smos_Fabs(dTripleProduct)-1.0) < SM_EFF_ZERO, SM_EFF_ZERO, (smos_Fabs(dTripleProduct)-1.0), _T("")) ;

  // basis vectors have to be right handed
  bRtn &= SM_ASSERT_VALUE_REPORT(4, SM_LEVEL_0, (dTripleProduct-1.0) < SM_EFF_ZERO, SM_EFF_ZERO, (dTripleProduct-1.0), _T("")) ;

  // UnbendNeutralDist must be positive definite
  bRtn &= SM_ASSERT_VALUE_REPORT(5, SM_LEVEL_0 , m_dUnbendNeutralDist >= m_dSingularityTol, SM_EFF_ZERO, m_dUnbendNeutralDist, _T("")) ;

  // Unbend SingularityTol must be positive definite
  bRtn &= SM_ASSERT_VALUE_REPORT(6, SM_LEVEL_0 , m_dSingularityTol >= SM_ZONE_TOL_3D, SM_EFF_ZERO, m_dSingularityTol, _T("")) ;

  // all done 
  return(bRtn) ;

} // end SmUnbendVolume::AssertValid

// obsolete
// /*******************************************************************//**
// PURPOSE: Fix Assert Report failures reported by an AssertValid call
// 
// RETURNS ---  TRUE  = OK
//              FALSE = Problem
// ***********************************************************************/
// SmBoolean SmUnbendVolume::AssertHeal
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
//                rAReport.m_pHealMessage = _T("SmUnbendVolume::AssertHeal fix not yet supported") ;  
//                 
//     }
// 
//   // all done
//   return(bRtn) ;
// 
// } // end SmUnbendVolume::AssertHeal
// end obsolete

/*******************************************************************//**
PURPOSE: Draw Axis - origin(black), x(red), y(green), z(blue)
         Draw Neutral Cylinder graphics
         Draw Unend Geometry
NOTES:
***********************************************************************/
SmDisplayList *SmUnbendVolume::Draw
 (SmBoolean       bShowOutSpace,    // in : TRUE = draw point = Evaluate(sParam)), FALSE=don't
                                    //      default:[TRUE]
  SmBoolean       bShowInSpace,     // in : TRUE = draw point = Orient(sParam), FALSE=don't
                                    //      default:[FALSE]
  SmExtent3d    * pOptParamDomain,  // in : optional ParamDomain to Draw, NULL=Draw Approximated Unbounded Region
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

  // make sure TgtDomain contains some of the Neutral plane distance and is limited to 1/2 plane u>0
  double dMaxU = smos_Max( 2*m_dUnbendNeutralDist, pTgtDomain->GetUMax()) ;
  double dMinU = smos_Min(.5*m_dUnbendNeutralDist, pTgtDomain->GetUMin()) ;
  if(dMinU < m_dSingularityTol) { dMinU = m_dSingularityTol ; }
  double dMaxV = smos_Max( 2*m_dUnbendNeutralDist, pTgtDomain->GetVMax()) ;
  double dMinV = smos_Min(-2*m_dUnbendNeutralDist, pTgtDomain->GetVMin()) ;

  // for a better display, set WIvl proportional to VIvl
  double dMaxW =  1.5 * dMaxV ;
  double dMinW = -1.5 * dMaxV ;
  dMaxW = smos_Min(pTgtDomain->GetWMax(), dMaxW) ;    
  dMinW = smos_Max(pTgtDomain->GetWMin(), dMinW) ;

  // specify the tgt draw domain
  pTgtDomain->SetMinMax(dMinU, dMinV, dMinW,
                        dMaxU, dMaxV, dMaxW) ;

  // build 3 Unbend Axis InSpace cylinders
  SmCylinder      *pCylinder1 = NULL, *pCylinder2=NULL, *pCylinder3=NULL ;

  SmAxis2Placement sCoords ;  // init to:[orig = 0,0,0  X = 1,0,0  Y = 0,1,0]
  if(GetOrientMap()) 
    { GetOrientMap()->LoadAxis2Placement(sCoords) ; }

  // build unbounded cylinders
  SmCylinder::CreateCanonical( *cpContext, sCoords, 0.5 * m_dUnbendNeutralDist, pCylinder1 );
  SmCylinder::CreateCanonical( *cpContext, sCoords, 1.0 * m_dUnbendNeutralDist, pCylinder2 );
  SmCylinder::CreateCanonical( *cpContext, sCoords, 1.5 * m_dUnbendNeutralDist, pCylinder3 );
  SmObjDelete sClean1(pCylinder1) ;
  SmObjDelete sClean2(pCylinder2) ;
  SmObjDelete sClean3(pCylinder3) ;

  // Cylinder STEPUV domains
  SmExtent2d sCylStepUVDomain(-120, dMinW, 120, dMaxW) ;

  // Set the Cylinder trims 
  pCylinder1->AdjustSTEPUVDomain( sCylStepUVDomain );
  pCylinder2->AdjustSTEPUVDomain( sCylStepUVDomain );
  pCylinder3->AdjustSTEPUVDomain( sCylStepUVDomain );

  // build 4 lines connecting the corners of the cylinders
  SmPoint2d sUVCorners[4] ; 
  SmPoint3d sInSpaceCorners[8] ;

  sUVCorners[0].Set(-120, dMinW) ;
  sUVCorners[1].Set(-120, dMaxW) ;
  sUVCorners[2].Set( 120, dMinW) ;
  sUVCorners[3].Set( 120, dMaxW) ;

  // Get cylinder corner locs
  pCylinder1->EvaluateSTEPPoint(sUVCorners[0], sInSpaceCorners[0]) ;
  pCylinder1->EvaluateSTEPPoint(sUVCorners[1], sInSpaceCorners[2]) ;
  pCylinder1->EvaluateSTEPPoint(sUVCorners[2], sInSpaceCorners[4]) ;
  pCylinder1->EvaluateSTEPPoint(sUVCorners[3], sInSpaceCorners[6]) ;

  pCylinder3->EvaluateSTEPPoint(sUVCorners[0], sInSpaceCorners[1]) ;
  pCylinder3->EvaluateSTEPPoint(sUVCorners[1], sInSpaceCorners[3]) ;
  pCylinder3->EvaluateSTEPPoint(sUVCorners[2], sInSpaceCorners[5]) ;
  pCylinder3->EvaluateSTEPPoint(sUVCorners[3], sInSpaceCorners[7]) ;

  // build lines between the corners
  SmLine sLine1(sInSpaceCorners[0], sInSpaceCorners[1], 3, cpContext) ; 
  SmLine sLine2(sInSpaceCorners[2], sInSpaceCorners[3], 3, cpContext) ; 
  SmLine sLine3(sInSpaceCorners[4], sInSpaceCorners[5], 3, cpContext) ; 
  SmLine sLine4(sInSpaceCorners[6], sInSpaceCorners[7], 3, cpContext) ; 
              
  // build isoParameter lines on pCylinder2 - z direction:[line], y direction:[circle]
  SmLine   sIsoParamCurve1(  sCoords.GetOriginRef() 
                           + smos_CosDeg(sCylStepUVDomain.GetUMin())*.5*m_dUnbendNeutralDist*sCoords.GetXAxisRef()
                           + smos_SinDeg(sCylStepUVDomain.GetUMin())*.5*m_dUnbendNeutralDist*sCoords.GetYAxisRef(), 
                           sCoords.GetZAxis(), 
                           sCylStepUVDomain.GetVInterval(), 1.0, 3, cpContext) ;
  SmLine   sIsoParamCurve2(  sCoords.GetOriginRef() 
                           + smos_CosDeg(sCylStepUVDomain.GetUMin())*1.0*m_dUnbendNeutralDist*sCoords.GetXAxisRef()
                           + smos_SinDeg(sCylStepUVDomain.GetUMin())*1.0*m_dUnbendNeutralDist*sCoords.GetYAxisRef(), 
                           sCoords.GetZAxis(), 
                           sCylStepUVDomain.GetVInterval(), 1.0, 3, cpContext) ;
  SmLine   sIsoParamCurve3(  sCoords.GetOriginRef() 
                           + smos_CosDeg(sCylStepUVDomain.GetUMin())*1.5*m_dUnbendNeutralDist*sCoords.GetXAxisRef()
                           + smos_SinDeg(sCylStepUVDomain.GetUMin())*1.5*m_dUnbendNeutralDist*sCoords.GetYAxisRef(), 
                           sCoords.GetZAxis(), 
                           sCylStepUVDomain.GetVInterval(), 1.0, 3, cpContext) ;
  SmCircle sIsoParamCurve4(sCoords.GetOriginRef() + sCylStepUVDomain.GetVMin()*sCoords.GetZAxis(), 
                           sCoords.GetXAxisRef(), 
                           sCoords.GetYAxisRef(), 
                           sCylStepUVDomain.GetUInterval(), 
                           0.5 * m_dUnbendNeutralDist, 3, cpContext) ;
  SmCircle sIsoParamCurve5(sCoords.GetOriginRef() + sCylStepUVDomain.GetVMin()*sCoords.GetZAxis(), 
                           sCoords.GetXAxisRef(), 
                           sCoords.GetYAxisRef(), 
                           sCylStepUVDomain.GetUInterval(), 
                           1.0 * m_dUnbendNeutralDist, 3, cpContext) ;
  SmCircle sIsoParamCurve6(sCoords.GetOriginRef() + sCylStepUVDomain.GetVMin()*sCoords.GetZAxis(), 
                           sCoords.GetXAxisRef(), 
                           sCoords.GetYAxisRef(), 
                           sCylStepUVDomain.GetUInterval(), 
                           1.5 * m_dUnbendNeutralDist, 3, cpContext) ;

  // when asked - show InSpace
  if(bShowInSpace)
    {
      // draw isoCurves
      sLine1.Draw(NULL, FALSE, NULL, pOptGfxSet) ;
      sLine2.Draw(NULL, FALSE, NULL, pOptGfxSet) ;
      sLine3.Draw(NULL, FALSE, NULL, pOptGfxSet) ;
      sLine4.Draw(NULL, FALSE, NULL, pOptGfxSet) ;

#ifdef SM_DEBUG_CODE
  if(bDebugMe)
    {
      SM_DUMP_AND_ASSERT_VALID(&sLine1) ;
      SM_DUMP_AND_ASSERT_VALID(&sLine2) ;
      SM_DUMP_AND_ASSERT_VALID(&sLine3) ;
      SM_DUMP_AND_ASSERT_VALID(&sLine4) ;
    }
#endif // SM_DEBUG_CODE

      // draw isoSurfaces
      if(pCylinder1) pCylinder1->DrawUV(8,8,FALSE,NULL,FALSE,pOptGfxSet) ;
      if(pCylinder2) pCylinder2->DrawUV(8,8,FALSE,NULL,FALSE,pOptGfxSet) ;
      if(pCylinder3) pCylinder3->DrawUV(8,8,FALSE,NULL,FALSE,pOptGfxSet) ;

      // Draw Cylinder IsoParamCurve
      double dPointSize = smgfx_GetPointSize() ;
      double dLineWidth = smgfx_GetLineWidth() ;
      smgfx_SetPointSize(6*dLineWidth, pOptGfxSet) ;      
      sIsoParamCurve1.DrawParams(NULL, pOptGfxSet) ;
      sIsoParamCurve2.DrawParams(NULL, pOptGfxSet) ;
      sIsoParamCurve3.DrawParams(NULL, pOptGfxSet) ;
      sIsoParamCurve4.DrawParams(NULL, pOptGfxSet) ;
      sIsoParamCurve5.DrawParams(NULL, pOptGfxSet) ;
      sIsoParamCurve6.DrawParams(NULL, pOptGfxSet) ;
      smgfx_SetPointSize(dPointSize, pOptGfxSet) ;      
#ifdef SM_DEBUG_CODE
  if(bDebugMe)
    {
      SM_DUMP_AND_ASSERT_VALID(pCylinder1) ;
      SM_DUMP_AND_ASSERT_VALID(pCylinder2) ;
      SM_DUMP_AND_ASSERT_VALID(pCylinder3) ;
    }
#endif // SM_DEBUG_CODE

    } // end bShowInSpace check

  // when asked - show OutSpace
  if(bShowOutSpace)
    {
      // Map the lines and cylinders from InSpace to OutSpace
      SmCrvInVolume sOutSpaceLine1(sLine1, FALSE, (SmVolume &)*this, 0, 0, cpContext) ; 
      SmCrvInVolume sOutSpaceLine2(sLine2, FALSE, (SmVolume &)*this, 0, 0, cpContext) ; 
      SmCrvInVolume sOutSpaceLine3(sLine3, FALSE, (SmVolume &)*this, 0, 0, cpContext) ; 
      SmCrvInVolume sOutSpaceLine4(sLine4, FALSE, (SmVolume &)*this, 0, 0, cpContext) ; 

      SmSrfInVolume sOutSpaceCylinder1(*pCylinder1, FALSE, (SmVolume &)*this, 0, 0, cpContext) ; 
      SmSrfInVolume sOutSpaceCylinder2(*pCylinder2, FALSE, (SmVolume &)*this, 0, 0, cpContext) ; 
      SmSrfInVolume sOutSpaceCylinder3(*pCylinder3, FALSE, (SmVolume &)*this, 0, 0, cpContext) ; 

      // Draw Cylinder IsoParamCurve
      SmCrvInVolume sOutIsoParamCurve1(sIsoParamCurve1, FALSE, (SmVolume &)*this, 0, 0, cpContext) ;
      SmCrvInVolume sOutIsoParamCurve2(sIsoParamCurve2, FALSE, (SmVolume &)*this, 0, 0, cpContext) ;
      SmCrvInVolume sOutIsoParamCurve3(sIsoParamCurve3, FALSE, (SmVolume &)*this, 0, 0, cpContext) ;
      SmCrvInVolume sOutIsoParamCurve4(sIsoParamCurve4, FALSE, (SmVolume &)*this, 0, 0, cpContext) ;
      SmCrvInVolume sOutIsoParamCurve5(sIsoParamCurve5, FALSE, (SmVolume &)*this, 0, 0, cpContext) ;
      SmCrvInVolume sOutIsoParamCurve6(sIsoParamCurve6, FALSE, (SmVolume &)*this, 0, 0, cpContext) ;
      
#ifdef SM_DEBUG_CODE
  if(bDebugMe)
    {
      SM_DUMP_AND_ASSERT_VALID(&sOutSpaceLine1) ;
      SM_DUMP_AND_ASSERT_VALID(&sOutSpaceLine2) ;
      SM_DUMP_AND_ASSERT_VALID(&sOutSpaceLine3) ;
      SM_DUMP_AND_ASSERT_VALID(&sOutSpaceLine4) ;

      SM_DUMP_AND_ASSERT_VALID(&sOutSpaceCylinder1) ;
      SM_DUMP_AND_ASSERT_VALID(&sOutSpaceCylinder2) ;
      SM_DUMP_AND_ASSERT_VALID(&sOutSpaceCylinder3) ;

    }
#endif // SM_DEBUG_CODE

      // draw isoCurves
      sOutSpaceLine1.Draw(NULL, FALSE, NULL, pOptGfxSet) ;
      sOutSpaceLine2.Draw(NULL, FALSE, NULL, pOptGfxSet) ;
      sOutSpaceLine3.Draw(NULL, FALSE, NULL, pOptGfxSet) ;
      sOutSpaceLine4.Draw(NULL, FALSE, NULL, pOptGfxSet) ;

      // draw isoSurfaces
      if(pCylinder1) sOutSpaceCylinder1.DrawUV(8,8,FALSE,NULL,FALSE,pOptGfxSet) ;
      if(pCylinder2) sOutSpaceCylinder2.DrawUV(8,8,FALSE,NULL,FALSE,pOptGfxSet) ;
      if(pCylinder3) sOutSpaceCylinder3.DrawUV(8,8,FALSE,NULL,FALSE,pOptGfxSet) ;

      // Draw Cylinder IsoParamCurves
      double dPointSize = smgfx_GetPointSize() ;
      double dLineWidth = smgfx_GetLineWidth() ;
      smgfx_SetPointSize(6*dLineWidth, pOptGfxSet) ;      
      sOutIsoParamCurve1.DrawParams(NULL, pOptGfxSet) ;
      sOutIsoParamCurve2.DrawParams(NULL, pOptGfxSet) ;
      sOutIsoParamCurve3.DrawParams(NULL, pOptGfxSet) ;
      sOutIsoParamCurve4.DrawParams(NULL, pOptGfxSet) ;
      sOutIsoParamCurve5.DrawParams(NULL, pOptGfxSet) ;
      sOutIsoParamCurve6.DrawParams(NULL, pOptGfxSet) ;
      smgfx_SetPointSize(dPointSize, pOptGfxSet) ;      
      
    } // end bShowOutSpace check

  // Draw Unbend coordinate system - U vector scaled to NeutralDistance
  SmVector3d sUnbendOrigin = GetUnbendOrigin();
  smgfx_SetColor(0,0,0) ; sUnbendOrigin.Draw(NULL, NULL, pOptGfxSet) ;
  smgfx_SetColor(1,0,0) ; (m_dUnbendNeutralDist * GetUnbendMidDir()).Draw(&sUnbendOrigin, NULL, pOptGfxSet) ;
  smgfx_SetColor(0,1,0) ; GetUnbendBinormal().Draw(&sUnbendOrigin, NULL, pOptGfxSet) ;
  smgfx_SetColor(0,0,1) ; GetUnbendAxis().Draw(&sUnbendOrigin, NULL, pOptGfxSet) ;
  smgfx_SetColor(sColor, pOptGfxSet) ;
  
  // Draw Unbend axis over W extent
  SmExtent1d sIvlW      = pTgtDomain->GetWInterval() ;
  SmPoint3d  sStartAxis = GetUnbendOrigin() + sIvlW.GetMin() * GetUnbendAxis() ;
  SmPoint3d  sEndAxis   = GetUnbendOrigin() + sIvlW.GetMax() * GetUnbendAxis() ;
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

} // end SmUnbendVolume::Draw

/*******************************************************************//**
PURPOSE:
NOTES:
***********************************************************************/
SmBoolean SmUnbendVolume::IsKindOf( SM_TYPE t ) const
{
  return ((SmUnbendVolume_TYPE == t) ? TRUE : SmVolume::IsKindOf( (t) ));
}

/*******************************************************************//**
PURPOSE: Dump bspline volume data out for debugging.
            Dump is in similar sequence to Write to file

NOTES: 
***********************************************************************/
void SmUnbendVolume::Dump() const
{
  Dump(FALSE, 0) ;
}

/*******************************************************************//**
PURPOSE: Dump bspline volume data out for debugging.
            Dump is in similar sequence to Write to file

NOTES: UnbendVolume and BendVolume use the same coordinate system
       and are inverse mappings of one another.  
***********************************************************************/
void SmUnbendVolume::Dump
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
  smos_sprintf(sBuff,        _T("\n%sBegin SmUnbendVolume[0x%p]::Dump()"), sIndent, this) ;
  smos_sprintf(sBuffForFile, _T("\n%sBegin SmUnbendVolume::Dump()"), sIndent) ;
  smos_WriteBuffer(sBuff, sBuffForFile);

  // header
  smos_sprintf(sBuff,       _T("\n%s  SmUnbendVolume = 0x%p"), sIndent, this);
  smos_sprintf(sBuffForFile,_T("\n%s  SmUnbendVolume = %s"), sIndent, _T("notNULL"));
  smos_WriteBuffer(sBuff, sBuffForFile);

  smos_WriteBuffer(_T("\n")) ;
  smos_sprintf(sBuff, _T("\n%s  UnbendOrigin      = InSpace point on the unbend center line"), sIndent) ; smos_WriteBuffer(sBuff) ;
  smos_sprintf(sBuff, _T("\n%s  UnbendAxis        = InSpace unbend center line unit-vector direction"), sIndent) ; smos_WriteBuffer(sBuff) ;
  smos_sprintf(sBuff, _T("\n%s  UnbendMidDir      = InSpace orthoganal to UnbendAxis marking Unbend Middle and start of Unbend Coordinate system"), sIndent) ; smos_WriteBuffer(sBuff) ;
  smos_sprintf(sBuff, _T("\n%s  UnbendBinormal    = InSpace unit-vector = UnbendAxis * UnbendMidDir"), sIndent) ; smos_WriteBuffer(sBuff) ;
  smos_sprintf(sBuff, _T("\n%s  UnbendNeutralDist = InSpace radial dist from center line to Neutral unbend cylinder"), sIndent) ; smos_WriteBuffer(sBuff) ;
  smos_sprintf(sBuff, _T("\n%s  UnbendSingularityTol  = min dist between NonSingular pts and Volume's singularity at ParamU=ParamV=0"), sIndent) ; smos_WriteBuffer(sBuff) ;
  smos_sprintf(sBuff, _T("\n%s                        dSingularityTol must be >= SM_ZONE_TOL_3D, default:[SM_ZONE_TOL_3D]"), sIndent) ; smos_WriteBuffer(sBuff) ;
  smos_sprintf(sBuff, _T("\n%s    note: The Unbend Mapping has two symmetric planes:"), sIndent) ; smos_WriteBuffer(sBuff) ;
  smos_sprintf(sBuff, _T("\n%s         1. The UnbendMidDir/UnbendAxis plane splits the unbend lengthwise"), sIndent) ; smos_WriteBuffer(sBuff) ;
  smos_sprintf(sBuff, _T("\n%s         2. The UnbendMidDir/UnbendBinormal plane splits the unbend crosswise at the UnbendOrigin"), sIndent) ; smos_WriteBuffer(sBuff) ;
  smos_sprintf(sBuff, _T("\n%s         3. Axis centered cylinders and axis centered arcs in planes parallel to UnbendMidDir/UnbendBinormal plane map to planes and lines"), sIndent) ; smos_WriteBuffer(sBuff) ;
  smos_sprintf(sBuff, _T("\n%s         4. Axis parallel lines map to lines"), sIndent) ; smos_WriteBuffer(sBuff) ;
  smos_WriteBuffer(_T("\n")) ;
  
  smos_sprintf(sBuff, _T("\n%s    UnbendOrigin      = "), sIndent) ; smos_WriteBuffer(sBuff) ; GetUnbendOrigin().Dump() ;
  smos_sprintf(sBuff, _T("\n%s    UnbendAxis        = "), sIndent) ; smos_WriteBuffer(sBuff) ; GetUnbendAxis().Dump() ;
  smos_sprintf(sBuff, _T("\n%s    UnbendMidDir      = "), sIndent) ; smos_WriteBuffer(sBuff) ; GetUnbendMidDir().Dump() ;
  smos_sprintf(sBuff, _T("\n%s    UnbendBinormal    = "), sIndent) ; smos_WriteBuffer(sBuff) ; GetUnbendBinormal().Dump() ;
  smos_sprintf(sBuff, _T("\n%s    UnbendNeutralDist = %16.16lf"), sIndent, GetUnbendNeutralDist()) ; smos_WriteBuffer(sBuff) ; 
  smos_sprintf(sBuff, _T("\n%s    UnbendSingularityTol =  %16.16lf"), sIndent, GetSingularityTol()) ;  smos_WriteBuffer(sBuff) ;
  
  // dump the base class - skip dumping the SmUnbendVolume Object - all that data has been dumped above
  //   skip calling: SmUnbendVolume::Dump(bAbbrev, lIndentCnt+2) ;
  SmVolume::Dump(bAbbrev, lIndentCnt+2) ;

  smos_sprintf(sBuff,        _T("\n%sEnd SmUnbendVolume[0x%p]::Dump()%s"), sIndent, this, lIndentCnt==0?_T("\n"):_T("")) ;
  smos_sprintf(sBuffForFile, _T("\n%sEnd SmUnbendVolume::Dump()%s"), sIndent, lIndentCnt==0?_T("\n"):_T("")) ;
  smos_WriteBuffer(sBuff, sBuffForFile);
  
} // end SmUnbendVolume::Dump

