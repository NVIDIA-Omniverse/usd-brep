// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*******************************************************************//**
* FILE NAME --- SmBendVolume.cpp
* PURPOSE: Implementation of SmBendVolume methods.
* Oct-2006 - gwc - author
**********************************************************************/

#include "StdAfx.h"

#include <SmBendVolume.h>
#include <SmTransform.h>
#include <SmAssertArray.h>
#include <SmCircle.h>
#include <SmLine.h>
#include <SmCylinder.h>
#include <SmPlane.h>
#include <SmDatabaseIO.h>



#ifdef SM_DEBUG_CODE
#include <SmEdge.h>
#include <SmFace.h>
#include <SmBrep.h>
#endif // SM_DEBUG_CODE

/*******************************************************************//**
PURPOSE: Constructor

NOTES: 
***********************************************************************/
SmBendVolume::SmBendVolume
  (const SmContext & crContext,        // in : for new OrientMap construction                                                                                           
   const SmPoint3d & rBendCenter,      // in : InSpace (and OutSpace) point on the bend center line                                                                     
   const SmVector3d & rBendAxis,       // in : InSpace (and OutSpace) direction of the bend center line marking W axis of Bend Coordinate System                                                 
   const SmVector3d & rBendMidDir,     // in : InSpace (and OutSpace) vector orthoganal to BendAxis marking U axis of Bend Coordinate System                            
   double            dBendNeutralDist, // in : InSpace (and OutSpace) distance along BendMidDir between BendOrigin and NeutralPlane.                                 
   double            dSingularityTol)  // in : min ProjSpace dist between NonSingular pts and the Volume's singularity at the ParameterPlane U=0.
                                       //      dSingularityTol must be >= SM_ZONE_TOL_3D, default:[SM_ZONE_TOL_3D]
                                       //  note: The BendMidDir/BendBinormal plane is the symmetric plane                                                            
                                       //        of the Bend mapping                                                                                                                  
: SmVolume(&crContext),
  m_dBendNeutralDist(dBendNeutralDist),
  m_dSingularityTol(dSingularityTol)
{
  // check input - dSingularityTol >= SM_ZONE_TOL_3D
  if(m_dSingularityTol < SM_ZONE_TOL_3D)
    { 
      SE_MSG( SM_ERR,
             _T("BendVolume dSingularityTol must be >= SM_ZONE_TOL_3D - Bad Construction - changing dSingularityTol to SM_ZONE_TOL_3D")) ;
      m_dSingularityTol = SM_ZONE_TOL_3D ;
    }

  // check input - Positive Definite BendNeutralDistance
  if(dBendNeutralDist < m_dSingularityTol)
    {
      SE_MSG( SM_ERR,
             _T("BendVolume must have a PositiveDefinite NeutralDistance - Bad Construction - changing BendNeutralDist to 1.0")) ;

      // do the best we can
      m_dBendNeutralDist = 1.0 ;
    }

  // unitize bend vectors
  SmVector3d sBendAxis(rBendAxis) ;
  SmVector3d sBendMidDir(rBendMidDir) ;
  SmVector3d sBendBinormal ;
  SmStatus   eStat1 = sBendAxis.Unitize() ; 
  SmStatus   eStat2 = sBendMidDir.Unitize() ;

  // check input - nonZero input vectors
  if(eStat1 != SM_SUCCESS || eStat2 != SM_SUCCESS)
    {
      SE_MSG( SM_ERR,
             _T("Bend Volume defining vectors were zero length - Bad Construction - Changing BendVectors to have length")) ; 

      // do the best we can
      if(eStat1 == SM_ERR && eStat2 == SM_ERR) { sBendAxis.Set(0,0,1) ; 
                                                 sBendMidDir.Set(1,0,0) ; 
                                               }
      else if(eStat1 == SM_ERR)                { sBendMidDir.MakeUnitOrthoVectors(NULL, sBendMidDir,
                                                                                           sBendBinormal,
                                                                                           sBendAxis) ; 
                                               }
      else                                     { sBendAxis.MakeUnitOrthoVectors(NULL, sBendAxis,
                                                                                      sBendMidDir,
                                                                                      sBendBinormal) ; 
                                               }
    } // end do something when given zero length vectors branch

  // check input - bend vectors must be perpendicular to one another
  SmBoolean bPerp = sBendAxis.IsPerpendicularTo(sBendMidDir, SM_EFF_ZERO_DEG) ;
  if(!bPerp)
    {
       SE_MSG( SM_ERR,
              _T("BendAxis not perpendicular to BendMidDir - Bad Construction - Changing BendVectors to be perpendicular")) ;

       SmBoolean bParallel = sBendAxis.IsParallelTo(sBendMidDir, SM_EFF_ZERO_DEG) ;

      if(!bParallel)
        {
         sBendMidDir = sBendAxis * (sBendMidDir * sBendAxis) ;
          sBendMidDir.Unitize() ;
        }
      else // Axis and StartDir don't span a plane
        {
          // do the best we can
          sBendMidDir.MakeUnitOrthoVectors(NULL, sBendMidDir,
                                                 sBendBinormal,
                                                 sBendAxis) ;
        }
    }
    
  // set BendBinormal perp to both BendAxis and BendMidDir
  sBendBinormal = sBendAxis * sBendMidDir ;

  // build BendCoordinates into m_pOrientMap
  SmTransform *pOrientMap = new (crContext) SmTransform(&crContext, rBendCenter, sBendMidDir, sBendBinormal, &sBendAxis) ;
  SetOrientMap(pOrientMap, 2) ;

} // end SmBendVolume constructor

/*******************************************************************//**
PURPOSE: Copy constructor for a BendVolume Volume.

NOTES:      
***********************************************************************/
SmBendVolume::SmBendVolume
 (const SmBendVolume & crSourceVolume,   // in : SourceVolume to copy
  SmBoolean            bSimpleMapOnly)   // in : TRUE = Copy this Volume omitting any compounding volumes
                                         //      FALSE= Copy this Volumes with any compounding volumes 
: SmVolume(crSourceVolume, bSimpleMapOnly),
  m_dBendNeutralDist(crSourceVolume.m_dBendNeutralDist),
  m_dSingularityTol (crSourceVolume.m_dSingularityTol)
{  

} // end SmBendVolume::SmBendVolume constructor

/*******************************************************************//**
PURPOSE: assignment operator

NOTES:
***********************************************************************/
SmBendVolume & SmBendVolume::operator=
  (const SmBendVolume &crBendVolume)       // in : object to copy
{ 
  if(this == &crBendVolume) return(*this) ;
  
  // assign base values
  SmVolume::operator=(crBendVolume) ;
                                                              
  // copy local members
  m_dBendNeutralDist = crBendVolume.m_dBendNeutralDist ;
  m_dSingularityTol  = crBendVolume.m_dSingularityTol ;

  // all done
  return(*this) ;

} // end SmBendVolume::operator=

/*******************************************************************//**
PURPOSE: Virtual Copy a SmBendVolume.

NOTES: 
***********************************************************************/
SmStatus SmBendVolume::Copy
  (const SmContext & crContext,      // in : context for new object construction
   SmVolume       *& rpNewVolume,    // out: The copied Volume
   SmBoolean         bSimpleMapOnly) // in : TRUE = Copy this Volume omitting any compounding volumes
  const                              //      FALSE= Copy this Volumes with any compounding volumes
                                     //      default:[FALSE]
{
  rpNewVolume = new (crContext) SmBendVolume(*this, bSimpleMapOnly);
  NER(rpNewVolume);
  return SM_SUCCESS;

} // end SmBendVolume::Copy

/*******************************************************************//**
PURPOSE: Equality operator for SmBendVolume

NOTES: Call base equivalence to check type and then check 
       members for equivalence
***********************************************************************/
SmBoolean SmBendVolume::operator==
  (const SmVolume& crOther) 
 const
{
  // low work - same objects are equal
  if(this == &crOther) { return TRUE ; }

  // low work - different types are not equal
  if(!crOther.IsKindOf(SmBendVolume_TYPE))
    { return FALSE ; }

  // first check the base
  SmBoolean bRtn = SmVolume::operator ==(crOther) ;

  // then check the members
  if(bRtn)
    {
      // OK to cast
      SmBendVolume &rOther = (SmBendVolume &)crOther ;

      // check equivalence of these objects
      bRtn &= SM_IS_ZERO(m_dBendNeutralDist - rOther.m_dBendNeutralDist) ;
      bRtn &= SM_IS_ZERO(m_dSingularityTol  - rOther.m_dSingularityTol) ;
    }

  // all done
  return bRtn ;

} // end SmBendVolume::operator==

/*******************************************************************//**
PURPOSE: Create a SmBendVolume from component data.  

NOTES: 
***********************************************************************/
SmStatus SmBendVolume::CreateCanonical
  (const SmContext & crContext,          // in : context for new object construction
   SmPoint3d       & rBendCenter,        // in : InSpace (and OutSpace) point on the bend center line                                              
   SmVector3d      & rBendAxis,          // in : InSpace (and OutSpace) direction of the bend center line marking W axis of Bend Coordinate System                           
   SmVector3d      & rBendMidDir,        // in : InSpace (and OutSpace) vector orthoganal to BendAxis marking U axis of Bend Coordinate System
   double            dBendNeutralDist,   // in : InSpace (and OutSpace) distance along rBendMidDir between BendOrigin and NeutralPlane.
   SmBendVolume   *& rpNewVolume,        // out: New SmBendVolume, NULL on input 
   double            dSingularityTol)    // in : min dist between NonSingular pts and Volume's singularity at U=0,
                                         //      dSingularityTol must be >= SM_ZONE_TOL_3D, default:[SM_ZONE_TOL_3D]
{
  // build the object
  rpNewVolume = new (crContext) SmBendVolume(crContext, rBendCenter, rBendAxis, rBendMidDir, dBendNeutralDist, dSingularityTol) ;

  // all done
  return(SM_SUCCESS) ; 

} // end SmBendVolume::CreateCanonical

/*******************************************************************//**
PURPOSE: Get the STEP canonical data out of a BendVolume.

NOTES: Please note that the control points are in Euclidian
    space (not homogeneous) even if the volume is rational.  That is the 
    homogeneous division has been performed on x,y,z prior to returning 
    the data in rControlPointsList.
***********************************************************************/
SmStatus SmBendVolume::GetCanonical
 (SmPoint3d  & rBendCenter,       // out: InSpace (and OutSpace) point on the bend center line                                              
  SmVector3d & rBendAxis,         // out: InSpace (and OutSpace) direction of the bend center line marking W axis of Bend Coordinate System                           
  SmVector3d & rBendMidDir,       // out: InSpace (and OutSpace) vector orthoganal to BendAxis marking U axis of Bend Coordinate System
  double     & rdBendNeutralDist, // out: InSpace (and OutSpace) distance along rBendMidDir between BendOrigin and NeutralPlane.
  double     & rdSingularityTol)  // out: min ProjSpace dist between NonSingular pts and the Volume's singularity at the ParameterPlane U=0.
 const
{
  SM_DUMP_AND_ASSERT2_VALID(this) ;

  // set outputs
  rBendCenter       = GetBendOrigin() ; 
  rBendAxis         = GetBendAxis() ;    
  rBendMidDir       = GetBendMidDir() ;
  rdBendNeutralDist = m_dBendNeutralDist ;
  rdSingularityTol  = m_dSingularityTol ;

  // all done
  return SM_SUCCESS;

} // end SmBendVolume::GetCanonical

/*******************************************************************//**
PURPOSE: Set all SmBendVolume data.

NOTES: 
***********************************************************************/
SmStatus SmBendVolume::SetCanonical
  (SmPoint3d  & rBendCenter,        // in : InSpace (and OutSpace) point on the bend center line                                              
   SmVector3d & rBendAxis,          // in : InSpace (and OutSpace) direction of the bend center line marking W axis of Bend Coordinate System                           
   SmVector3d & rBendMidDir,        // in : InSpace (and OutSpace) vector orthoganal to BendAxis marking U axis of Bend Coordinate System
   double       dBendNeutralDist,   // in : InSpace (and OutSpace) distance along rBendMidDir between BendOrigin and NeutralPlane.
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
             _T("BendVolume dSingularityTol must be >= SM_ZONE_TOL_3D - Bad Construction - changing dSingularityTol to SM_ZONE_TOL_3D")) ;
      dSingularityTol = SM_ZONE_TOL_3D ;
    }

  // check input - Positive Definite BendNeutralDistance
  if(dBendNeutralDist < dSingularityTol)
    {
      SE_MSG( SM_ERR,
             _T("BendVolume must have a PositiveDefinite NeutralDistance - Bad Construction - changing BendNeutralDist to 1.0")) ;

      // do the best we can
      dBendNeutralDist = 1.0 ;
    }

  // check state - unit vectors
  SmVector3d sBendAxis         = rBendAxis ;
  SmVector3d sBendMidDir       = rBendMidDir ;
  SmStatus   sBendAxisStatus   = sBendAxis.Unitize() ;
  SmStatus   sBendMidDirStatus = sBendMidDir.Unitize() ;

  // zero length BendAxis vector
  if(sBendAxisStatus != SM_SUCCESS)
    {
      SE_MSG(SM_ERR, _T("SmBendVolume::SetCanonical: Zero length BendAxis - changing to (0,0,1) ")) ;
      sBendAxis.Set(0,0,1) ;
    }

  // zero length BendMidDir vector
  if(sBendMidDirStatus != SM_SUCCESS)
    {
      SE_MSG(SM_ERR, _T("SmBendVolume::SetCanonical: Zero length BendMidDir - changing to (1,0,0) ")) ;
      sBendAxis.Set(1,0,0) ;
    }

  // check state - orthogonal vectors
  double dDotProd = sBendAxis.Dot(sBendMidDir) ;

  // non orthogonal BendAxis/BendMidDir vector pair
  if(!SM_IS_ZERO(dDotProd))
    {
      SE_MSG(SM_ERR, _T("SmBendVolume::SetCanonical: BendAxis and BendMidDir not orthogonal - changing BendMidDir")) ;
      sBendMidDir = sBendAxis * (sBendMidDir * sBendAxis) ;
      sBendMidDir.Unitize() ;
    }

  // locals
  SmVector3d sBendBinormal = rBendAxis * rBendMidDir ;

  // set OrientMap
  SmTransform *pOrientMap = GetOrientMap() ;
  pOrientMap->SetCanonical(rBendCenter, rBendMidDir, sBendBinormal, &rBendAxis) ;

  // set Neutral distance
  m_dBendNeutralDist = dBendNeutralDist ; 
  
  // set SingularityTol
  m_dSingularityTol = dSingularityTol ;        

  // inform the public
  Notify(SM_NO_POST_EDIT, this, SM_NO_GET_OWNER(this), NULL) ;

  // all done
  return SM_SUCCESS;

} // end SmBendVolume::SetCanonical

/*******************************************************************//**
PURPOSE: BendVolumes are unbounded 1/2 spaces for all u > zero  

NOTES: The ParamSpace BendVolume half space domain is aligned with the ParamSpace coordinate vectors.
       The InSpace BendVolume domain half space is not
       aligned with the InSpace coordinate vectors. 
***********************************************************************/
SmExtent3d SmBendVolume::GetNaturalParamDomain
  () 
 const
{
  double dMinY = - SM_PI * m_dBendNeutralDist ;
  double dMaxY = - dMinY ;

  // init output to positive definite u half-space
  SmExtent3d sBox(m_dSingularityTol,     dMinY, -SM_INFINITE_PARAMETER,
                  SM_INFINITE_PARAMETER, dMaxY,  SM_INFINITE_PARAMETER) ;

  // all done
  return(sBox) ;

} // end SmBendVolume::GetNaturalParamDomain

// obsolete - moved to SmVolume class
//  /*******************************************************************//**
//  PURPOSE: BendVolumes are unbounded 1/2 spaces for all a > zero  
//  
//  NOTES: a is the 'u' coordinated of the BendCenteredCoordinate system
//    The UVW NaturalUVWDomain is a projections of the BendCenteredCoordinate
//    Sytem to UVW InSpace. In general the bend won't be aligned with the
//    UVW coordinate system and the NaturalUVWDomain won't be a simple
//    axis aligned box.  
//  ***********************************************************************/
//  SmPseudoBox SmBendVolume::GetNaturalInSpaceDomain
//    () 
//   const
//  {
//    // init output
//    SmPseudoBox sInSpaceDomain ;
//  
//    // locals
//    SmVector3d sBendOrigin    = GetBendOrigin() ;
//    SmVector3d sBendAxis      = GetBendAxis() ;
//    SmVector3d sBendMidDir    = GetBendMidDir() ;
//    SmVector3d sBendBinormal  = GetBendBinormal() ;
//    double     dDistToOrigin  = sBendMidDir.Dot(sBendOrigin) ;
//    SmExtent1d sInfiniteIvl( -SM_INFINITE_PARAMETER,             SM_INFINITE_PARAMETER) ;
//    SmExtent1d sHalfIvl    (  dDistToOrigin + m_dSingularityTol, SM_INFINITE_PARAMETER) ;
//  
//    // orient axes
//    sInSpaceDomain.SetBasis(sBendMidDir, sBendBinormal, sBendAxis) ;
//  
//    // set intervals 
//    sInSpaceDomain.SetIntervals(sHalfIvl, sInfiniteIvl, sInfiniteIvl) ;
//    
//    // all done
//    return(sInSpaceDomain) ;
//  
//  } // end SmBendVolume::GetNaturalInSpaceDomain

/*******************************************************************//**
PURPOSE: convenience function to look like a SmBSplineVolume object that
         gets effective knot lists

NOTES: for cache purposes - let BendVolume mascarade as a BSpline kind of shape. 
***********************************************************************/
SmStatus SmBendVolume::GetKnots
  (SmVolumeParamType  eVolumeParam, // in : one of SM_VP_U, SM_VP_V, SM_VP_W
   SmTArray<double> & rKnots,       // out: ParamSpace knot list 
   SmTArray<ULONG>  * pKnotMults,   // out: associated knot multipliticies. NULL to ignore. default:[NULL]
   const SmExtent1d * pOptIvl)      // NotUsed: in : interval of interest, NULL=Natural Interval, default:[NULL]
  const     
{ 
  SM_REF1(pOptIvl) ;
  
  // init output
  rKnots.ReSet() ;
  if(pKnotMults) { pKnotMults->ReSet() ; } 

  // check input
  if(   eVolumeParam != SM_VP_U && eVolumeParam != SM_VP_X_IN
     && eVolumeParam != SM_VP_V && eVolumeParam != SM_VP_Y_IN
     && eVolumeParam != SM_VP_W && eVolumeParam != SM_VP_Z_IN)
    {
      SER_MSG(SM_ERR, _T("\nUnsupported SmVolumeParamType sent to SmBendVolume::GetKnots ")) ;
    }

  // knots (treat InSpace coordinates the same as ParamSpace ones to allow for user confusion)
  switch(eVolumeParam)
    {
      case SM_VP_X_IN:
      case SM_VP_U : if(pOptIvl == NULL || pOptIvl->ContainsValue( m_dSingularityTol, SM_EFF_ZERO )) 
                       { rKnots.Add( m_dSingularityTol ) ;
                         if(pKnotMults) { pKnotMults->Add(4) ; }
                       }
                     if(pOptIvl == NULL || pOptIvl->ContainsValue(SM_INFINITE_PARAMETER, SM_EFF_ZERO))
                       { rKnots.Add( SM_INFINITE_PARAMETER) ;
                         if(pKnotMults) { pKnotMults->Add(4) ; }
                       }
                     break ;
      case SM_VP_Y_IN:
      case SM_VP_V : if(pOptIvl == NULL || pOptIvl->ContainsValue(-SM_INFINITE_PARAMETER, SM_EFF_ZERO)) 
                       { rKnots.Add(-SM_INFINITE_PARAMETER) ;
                         if(pKnotMults) { pKnotMults->Add(4) ; }
                       }
                     if(pOptIvl == NULL || pOptIvl->ContainsValue(SM_INFINITE_PARAMETER, SM_EFF_ZERO)) 
                       { rKnots.Add( SM_INFINITE_PARAMETER) ;
                         if(pKnotMults) { pKnotMults->Add(4) ; }
                       }
                     break ;
      case SM_VP_Z_IN:
      case SM_VP_W : if(pOptIvl == NULL || pOptIvl->ContainsValue(-SM_INFINITE_PARAMETER, SM_EFF_ZERO)) 
                       { rKnots.Add(-SM_INFINITE_PARAMETER) ;
                         if(pKnotMults) { pKnotMults->Add(4) ; }
                       }
                     if(pOptIvl == NULL || pOptIvl->ContainsValue(SM_INFINITE_PARAMETER, SM_EFF_ZERO)) 
                       { rKnots.Add( SM_INFINITE_PARAMETER) ;
                         if(pKnotMults) { pKnotMults->Add(4) ; }
                       }
                     break ;
      default : SER_MSG(SM_ERR, _T("\nUnsupported SmVolumeParamType sent to SmBendVolume::GetKnots ")) ;
    
    } // end switch on eVolumeParam 

  // all done
  return(SM_SUCCESS) ; 

} // end SmBendVolume::GetKnots

/*******************************************************************//**
PURPOSE: When possible build the exact Curve produced by projecting
    rInputCurve to 1st OutSpace

NOTES: when eInputSpace == SM_VS_PARAM_SPACE: rInputCurve is mapped from ParamSpace to 1stOutSpace
       else eInputSpace == SM_VS_INSPACE    : rInputCurve is mapped from InSpace to 1stOutSpace

   the BendVolume 'maps' 
  InputSpace lines parallel to the InputSpace Bend Axis     to OutSpace Lines parallel to the bend axis 
  InputSpace lines parallel to the InputSpace Bend binormal to OutSpace circles centered on the bend axis
  InputSpace lines parallel to the InputSpace Bend MidDir   to OutSpace lines radiating radially from the Bend Axis

  All other curve shapes and orientations are not projected to Analytic shapes and this 
  method sets rpNewCurve == NULL and returns SM_SUCCESS.
***********************************************************************/
SmStatus SmBendVolume::MakeExactBSpline1stOutCurve
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

  // locals: ParamSpace and InSpace Bend orientations - Bend's Neutral plane is perp to the Bend's MidDirection
  SmVector3d sParamOrigin  (0,0,0), sBendOrigin   = GetBendOrigin() ;
  SmVector3d sParamMidDir  (1,0,0), sBendMidDir   = GetBendMidDir() ;
  SmVector3d sParamAxis    (0,0,1), sBendAxis     = GetBendAxis() ;
  SmVector3d sParamBinormal(0,1,0), sBendBinormal = GetBendBinormal() ;

  // locals: making InputSpace either ParamSpace or InSpace
  SmVector3d sInputOrigin, sInputMidDir, sInputAxis, sInputBinormal ;
  if(eInputSpace == SM_VS_IN_SPACE) { sInputOrigin   = sBendOrigin ;
                                      sInputMidDir   = sBendMidDir ;
                                      sInputAxis     = sBendAxis ;   
                                      sInputBinormal = sBendBinormal ;
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
  SmVector3d        sInputPt, sInputVec, sOnAxisPt, sOnAxisVec ; 

  double            dAxisPlane, dMidDirPlane, dBinormalPlane ;                      // dist from Sample Pt to planes perp to Bend Axes.
  SmExtent1d        sAxisPlaneRange, sMidDirPlaneRange, sBinormalPlaneRange ;       // range on distances
  double            dAxisPlaneSum=0.0, dMidDirPlaneSum=0.0, dBinormalPlaneSum=0.0 ; // Sum of SamplePt distances to planes
  SmBoolean         bConstAxisPlane = false, bConstMidDirPlane = false, bConstBinormalPlane = false ;

  // See if the InputCurve is linear and parallel to one of the BendAxes (parallel to two BendPlanes)
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

      // InputSpace distances to planes perp to Bend axes
      dAxisPlane     = sInputAxis.Dot(sOnAxisVec) ;     // distance to Axis Plane
      dMidDirPlane   = sInputMidDir.Dot(sOnAxisVec) ;   // distance to MidDir Plane
      dBinormalPlane = sInputBinormal.Dot(sOnAxisVec) ; // distance to Binormal Plane

      // accumulate the range of Plane distances
      sAxisPlaneRange.     AddValue(dAxisPlane) ;
      sMidDirPlaneRange.   AddValue(dMidDirPlane) ;
      sBinormalPlaneRange. AddValue(dBinormalPlane) ;

      // accumulate the sum of Plane distances
      dAxisPlaneSum     += dAxisPlane ;
      dMidDirPlaneSum   += dMidDirPlane ;
      dBinormalPlaneSum += dBinormalPlane ;

      // remember when curve is const dist from bend plane
      bConstAxisPlane     = sAxisPlaneRange.GetLength()     < sApproxTol ;
      bConstMidDirPlane   = sMidDirPlaneRange.GetLength()   < sApproxTol ;
      bConstBinormalPlane = sBinormalPlaneRange.GetLength() < sApproxTol ;

      // no work - curve not const distance from at least two planes - hack here, depends on: TRUE = 1, FALSE=0
      if( (bConstAxisPlane + bConstMidDirPlane + bConstBinormalPlane) < 2 )
        { return SM_SUCCESS ; }

    } // end iter ii - gather samples for parallel test

  // arrive here when curve is a line, const dist from at least two planes
  SM_ASSERT_MSG((bConstAxisPlane + bConstMidDirPlane + bConstBinormalPlane) == 2, _T("Missed a case - this is a bug; check code.")) ;

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

  // lines parallel to BendAxis => map to OutSpace lines
  // lines parallel to MidBendAxis => map to OutSpace lines
  if(   (bConstMidDirPlane && bConstBinormalPlane)   // parallel to Axis vec
     || (bConstAxisPlane   && bConstBinormalPlane))  // Parallel to MidDir vec
    {
      // create the OutSpace Line
      rpNewCurve = new (pContext) SmLine(sOutSpaceFirst, sOutSpaceLast, 3, pContext) ;
    }

  // lines parallel to BinormalAxis => map to OutSpace circles
  if(bConstMidDirPlane && bConstAxisPlane)
    {
      // OutSpace bend parameters
      SmVector3d sOutSpaceMidDir      = GetBendMidDir() ;
      SmVector3d sOutSpaceAxis        = GetBendAxis() ;
      SmVector3d sOutSpaceBinormal    = GetBendBinormal() ;

      // find the current rInputCurve curve tangent - it's a line so sample it once anywhere
      SmVector3d sInputCurvePN[2], sOutSpaceOnAxis, sXAxis, sYAxis, sZAxis ;           
      rInputCurve.Evaluate(sIvl.Evaluate(.5), 1, TRUE, sInputCurvePN) ;  

      // check orientation of curve against the Input BinormalAxis axis and select appropriate coordinate system
      SmBoolean bPositive = sInputCurvePN[1].Dot(sInputBinormal) > 0.0 ;
      if(bPositive) { sXAxis = sOutSpaceMidDir ;   sYAxis = sOutSpaceBinormal ; sZAxis =  sOutSpaceAxis ; }
      else          { sXAxis = sOutSpaceBinormal ; sYAxis = sOutSpaceMidDir ;   sZAxis = -sOutSpaceAxis ; }

      // OutSpace circle circ center and radius
      if(eInputSpace == SM_VS_IN_SPACE)            
        { MapPoint(sOnAxisPt, sOutSpaceOnAxis, FALSE) ; }      // without compounding
      else /* eInputSpace == SM_VS_PARAM_SPACE) */ 
        { EvaluatePoint(sOnAxisPt, sOutSpaceOnAxis, FALSE) ; } // without compounding

      double dRadius1 = (sOutSpaceFirst - sOutSpaceOnAxis).Length() ;
      double dRadius2 = (sOutSpaceLast  - sOutSpaceOnAxis).Length() ;
      double dRadius = (dRadius1 + dRadius2) / 2.0 ;

      // Start and End AngDeg 
      double dStartAngRad, dEndAngRad ;

      sZAxis.CCWAngleBetween(sXAxis, sOutSpaceFirst - sOutSpaceOnAxis, dStartAngRad) ;
      sZAxis.CCWAngleBetween(sXAxis, sOutSpaceLast  - sOutSpaceOnAxis, dEndAngRad) ;

      SmExtent1d sAngDomainDeg ; 
      sAngDomainDeg.AddValue(SM_RAD2DEG(dStartAngRad)) ;
      sAngDomainDeg.AddValue(SM_RAD2DEG(dEndAngRad)) ;

#ifdef SM_DEBUG_CODE
      if(bDebugMe)
        {
          const SmEdge *pEdge = rInputCurve.GetEdge() ;
          const SmBrep *pBrep = pEdge ? pEdge->GetBrep() : NULL ;

          smgfx_Erase() ;
          smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
          smgfx_SetLook(3,4, 0,1,0) ; rInputCurve.Draw() ; sm_GraphicsLoop() ;
          smgfx_SetLook(3,4, 0,1,0) ; rInputCurve.DrawParams() ; sm_GraphicsLoop() ;
          smgfx_SetLook(5,6, 0,0,1) ; sOutSpaceOnAxis.Draw() ; sm_GraphicsLoop() ;
          smgfx_SetLook(5,6, 0,1,0) ; (sOutSpaceFirst - sOutSpaceOnAxis).Draw(&sOutSpaceOnAxis) ; sm_GraphicsLoop() ;
          smgfx_SetLook(5,6, 1,0,0) ; (sOutSpaceLast  - sOutSpaceOnAxis).Draw(&sOutSpaceOnAxis) ; sm_GraphicsLoop() ;
          smgfx_SetLook(5,6, 1,0,0) ; sXAxis.Draw(&sOutSpaceOnAxis) ; sm_GraphicsLoop() ;
          smgfx_SetLook(5,6, 0,1,0) ; sYAxis.Draw(&sOutSpaceOnAxis) ; sm_GraphicsLoop() ;
          smgfx_SetLook(5,6, 0,0,1) ; sZAxis.Draw(&sOutSpaceOnAxis) ; sm_GraphicsLoop() ;
          sm_GraphicsLoop() ;
        }
#endif // SM_DEBUG_CODE

      // create the OutSpace Circular Arc
      rpNewCurve = new (pContext) SmCircle(sOutSpaceOnAxis, // in : circle center 
                                           sXAxis,          // in : circle unit X axis, points to Circle Start/End
                                           sYAxis,          // in : circle unit Y axis, points to Circle 90 deg point
                                           sAngDomainDeg,   // in : angular arc domain in degrees.
                                                            //        range:[-360 <= Min <= Max <= 360], MaxLength=360.0
                                           dRadius,         // in : radius
                                           3,               // in : sizeof crCenter, crXAxis, and crYAxis, default:[3]
                                           pContext) ;      // in : context must be given for automatic variables
    } // end build a circle case

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

} // end SmBendVolume::MakeExactBSpline1stOutCurve

/*******************************************************************//**
PURPOSE: When possible build the exact Surface produced by projecting
    rInputSurface to 1st OutSpace

NOTES: the BendVolume maps 
  InputSpace planes perp to InputSpace Bend MidBend  axis to OutSpace Cylinders[center:Outspace axis, radius: Input Plane/Orig dist]
  InputSpace planes perp to InputSpace Bend Binormal axis to Outspace planes containing Outspace Axis (radiating radially)
  InputSpace planes perp to InputSpace Bend BendAxis axis to OutSpace planes perp Bend Axis

  All other surface shapes and orientations are not projected to Analytic shapes and this 
  method sets rpNewSurface == NULL and returns SM_SUCCESS.
***********************************************************************/
SmStatus SmBendVolume::MakeExactBSpline1stOutSurface
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

  // locals: ParamSpace and InSpace Bend orientations - Bend's Neutral plane is perp to the Bend's MidDirection
  SmVector3d sParamOrigin  (0,0,0), sBendOrigin   = GetBendOrigin() ;
  SmVector3d sParamMidDir  (1,0,0), sBendMidDir   = GetBendMidDir() ;
  SmVector3d sParamAxis    (0,0,1), sBendAxis     = GetBendAxis() ;
  SmVector3d sParamBinormal(0,1,0), sBendBinormal = GetBendBinormal() ;
  double     dBendNeutralDist = GetBendNeutralDist() ;

  // locals: making InputSpace either ParamSpace or InSpace
  SmVector3d sInputOrigin, sInputMidDir, sInputAxis, sInputBinormal ;
  if(eInputSpace == SM_VS_IN_SPACE) { sInputOrigin   = sBendOrigin ;
                                      sInputMidDir   = sBendMidDir ;
                                      sInputAxis     = sBendAxis ;   
                                      sInputBinormal = sBendBinormal ;
                                    }
  else /* InputSpce = ParamSpace */ { sInputOrigin   = sParamOrigin ;
                                      sInputMidDir   = sParamMidDir ;
                                      sInputAxis     = sParamAxis ;   
                                      sInputBinormal = sParamBinormal ;
                                    }

  // locals 
  ULONG ii, jj, lSmpSize = 5 ;
  SmVector2d        sSmpUV ;
  SmVector3d        sInputPt, sInputVec, sOutSpacePt ; 

  SmApproxTol3d     sApproxTol = SmTol::GetApproxTol3d() ;
  const SmContext * pContext = rInputSurface.GetContext() ;
  SmExtent1d        sIvlU = rInputSurface.GetNaturalUVDomain().GetUInterval() ; 
  SmExtent1d        sIvlV = rInputSurface.GetNaturalUVDomain().GetVInterval() ;

  // note: for Bend mapping Bend principle axes happen to be the same in InSpace and OutSpace
  double            dAxisPlane, dMidPlane, dBinormalPlane ;                                            // InputSpace Properties
  double            dAxisPlaneSum=0.0, dMidPlaneSum=0.0, dBinormalPlaneSum=0.0 ;                       // InputSpace Properties
  SmExtent1d        sAxisPlaneRange, sMidDirPlaneRange, sBinormalPlaneRange ;                          // InputSpace Properties
  SmBoolean         bConstAxisPlane = FALSE, bConstMidDirPlane = FALSE, bConstBinormalPlane = FALSE;   // InputSpace Properties
  SmPseudoBox       sOutSpaceBBox(sBendMidDir,       sBendBinormal,       sBendAxis,                   // OutSpace Property
                                  sMidDirPlaneRange, sBinormalPlaneRange, sAxisPlaneRange) ; 

  // See if the InputSurface is perpendicular to any of the Bend principle axes (parallel to any Bend principle planes)
  for(ii=0;ii<=lSmpSize;ii++)
    {
      sSmpUV.x = sIvlU.Evaluate((double)ii/(double)lSmpSize) ;

      for(jj=0;jj<=lSmpSize;jj++)
        {
          sSmpUV.y = sIvlV.Evaluate((double)jj/(double)lSmpSize) ;

          // Surface sample 
          rInputSurface.EvaluatePoint(sSmpUV, sInputPt) ;
          sInputVec = sInputPt - sInputOrigin ;

          // save OutSpace Extents of the corner points
          if( (ii == 0 || ii == lSmpSize) && (jj == 0 || jj == lSmpSize) )
            {
              if(eInputSpace == SM_VS_IN_SPACE)
                { MapPoint(sInputPt, sOutSpacePt, FALSE) ; } // without compounding  
              else
                { EvaluatePoint(sInputPt, sOutSpacePt, FALSE) ; } // wihtout compounding
              sOutSpaceBBox.AddPoint3d(sOutSpacePt-sBendOrigin) ;
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
          dMidPlane      = sBendMidDir.Dot(sInputVec) ;   // distance to OrigPlane perpendicular to MidPlane
          dBinormalPlane = sBendBinormal.Dot(sInputVec) ; // distance to OrigPlane perpendicular to BinormalPlane
          dAxisPlane     = sBendAxis.Dot(sInputVec) ;     // distance to OrigPlane perpendicular to AxisPlane

          // accumulate the range of radius values
          //            the range of width values
          //            the range of height values
          sMidDirPlaneRange.AddValue(dMidPlane) ;
          sAxisPlaneRange.AddValue(dAxisPlane) ;
          sBinormalPlaneRange.AddValue(dBinormalPlane) ;

          // accumlate the sum of Plane distances
          dAxisPlaneSum     += dAxisPlane ;
          dMidPlaneSum      += dMidPlane ;
          dBinormalPlaneSum += dBinormalPlane ;

          // remember when curve is const dist from bend plane
          bConstAxisPlane     = sAxisPlaneRange.GetLength()     < sApproxTol ;
          bConstMidDirPlane   = sMidDirPlaneRange.GetLength()   < sApproxTol ;
          bConstBinormalPlane = sBinormalPlaneRange.GetLength() < sApproxTol ;

          // no work - Surface not const distance from at least one plane - hack here, depends on: TRUE = 1, FALSE=0
          if((bConstAxisPlane + bConstMidDirPlane + bConstBinormalPlane) < 1 )
            { return SM_SUCCESS ; }

        } // end iter jj - gather sampels for parallel test
    } // end iter ii - gather samples for parallel test

  // arrive here when rInputSurface is parallel to one of the bend's principle planes
  SM_ASSERT_MSG((bConstAxisPlane + bConstMidDirPlane + bConstBinormalPlane) == 1, _T("Missed a case - this is a bug; check code.")) ;

  // find the current rInputSurface surface normal - it's a plane so sample it once anywhere
  SmVector2d sInputSurfaceMidUV(sIvlU.Evaluate(.5), sIvlV.Evaluate(.5)) ;
  SmVector3d sInputSurfaceNormal ;             
  rInputSurface.EvaluateNormal(sInputSurfaceMidUV, TRUE, TRUE, sInputSurfaceNormal) ;

  // surfaces perpendicular to the BendAxis map to planes with the same orientation perpendicular to the BendAxis 
  if(bConstAxisPlane)
    {
      SmVector3d sOrigin = sBendOrigin + dAxisPlaneSum/(lSmpSize+1.0)/(lSmpSize+1.0) * sBendAxis ;
      SmVector2d sUVScale(1,1) ;

      // check orientation of surface against the Input BendAxis axis
      SmBoolean bPositive = sInputSurfaceNormal.Dot(sInputAxis) > 0.0 ;

      // UVDomain = BBox3d U (BendMidDir) and V (BendBiNormal) extents
      SmExtent2d sUVDomain(sOutSpaceBBox.GetInterval(0), sOutSpaceBBox.GetInterval(1)) ;

      // bump BBox for maximums not in the corners
      SmExtent1d sAngRangeRad(sBinormalPlaneRange.GetMin() / dBendNeutralDist,
                              sBinormalPlaneRange.GetMax() / dBendNeutralDist) ;
      if(sAngRangeRad.ContainsValue(-SM_PI, SM_EFF_ZERO_DEG)    ) { sUVDomain.SetUMin(-sMidDirPlaneRange.GetMax()) ; }
      if(sAngRangeRad.ContainsValue(-SM_PI/2.0, SM_EFF_ZERO_DEG)) { sUVDomain.SetVMin(-sMidDirPlaneRange.GetMax()) ; }
      if(sAngRangeRad.ContainsValue(0.0, SM_EFF_ZERO_DEG)       ) { sUVDomain.SetUMax( sMidDirPlaneRange.GetMax()) ; }
      if(sAngRangeRad.ContainsValue(SM_PI/2.0, SM_EFF_ZERO_DEG) ) { sUVDomain.SetVMax( sMidDirPlaneRange.GetMax()) ; }
      if(sAngRangeRad.ContainsValue(SM_PI, SM_EFF_ZERO_DEG)     ) { sUVDomain.SetUMin(-sMidDirPlaneRange.GetMax()) ; }

      // Create Plane with SrfNormal = rInputSurface->Surface Normal
      SmVector3d sXVector, sYVector ;
      if(bPositive) { sXVector = sBendMidDir ;
                      sYVector = sBendBinormal ;
                    }
      else          { sXVector = sBendBinormal ;
                      sYVector = sBendMidDir ;    
                      sUVDomain.Transpose() ;
                    }

      // create new plane - positive orientation:[SrfNormal = BendMidDir X BendBinormal]
      rpNewSurface = new (pContext) SmPlane(sOrigin,         // in : 3d loc of parametric origin = Plane_Evaluate(0,0)
                                            sXVector,        // in : 3D X Axis - made unit
                                            sYVector,        // in : 3D Y Axis - made unit - should be perp to X
                                            sUVScale,        // in : UScale and VScale
                                            sUVDomain,       // in : UV Domain limiting allowed evaluations
                                            pContext) ;      // in : Set context if given, default:[NULL]
    } // end plane perp to Bend Axis check

  // surfaces perpendicular to the MidDir plane map to BendAxis centered cylinders
  if(bConstMidDirPlane)
    {
      // set SmCylinder constructor params
      double     dRadius         = dMidPlaneSum / (lSmpSize+1.0) / (lSmpSize+1.) ;
      double     dAxisLength     = sAxisPlaneRange.GetLength() ;
      SmVector3d sCylinderOrigin = sBendOrigin + sAxisPlaneRange.GetMin() * sBendAxis ;
      double     dStartAngleDeg  = SM_RAD2DEG(sBinormalPlaneRange.GetMin() / dBendNeutralDist) ;
      double     dStopAngleDeg   = SM_RAD2DEG(sBinormalPlaneRange.GetMax() / dBendNeutralDist) ;

      // check orientation of surface against the Input BendMidDir axis
      SmBoolean bPositive = sInputSurfaceNormal.Dot(sInputMidDir) > 0.0 ;

      // create Cylinder with SrfNormal matching rInputSurface surface normal
      SmBoolean bInsideOut = bPositive ? FALSE : TRUE ;   // FALSE=outward pointing SrfNormals, TRUE=inward pointing SrfNormals

      // create Cylinder with SrfNormal from SrfPt outward away from BendAxis:[bInsideOut = FALSE]
      rpNewSurface = new (pContext) SmCylinder(sBendOrigin,     // zero Point on Cylinder's ZAxis
                                               sBendMidDir,     // Cylinder's X Axis (vector perp to axis to cylinder start point)
                                               sBendBinormal,   // Cylinder's Y Axis (vector perp to axis and X Axis)
                                               dRadius,         // Cylinder's radius (distance from axis to cylinder wall)
                                               dStartAngleDeg,  // CCW about Z from X, rangeDeg:[-360 to 360] maxLength = 360
                                               dStopAngleDeg,   // CCW about Z from X, rangeDeg:[-360 to 360] maxLength = 360
                                               dAxisLength,     // Defined domain from zero point in Z direction 
                                               FALSE,           // in : TRUE = underlying NURB surface v dir maps to rotation direction
                                                                //      FALSE= underlying NURB surface u dir maps to rotation direction
                                               bInsideOut,      // in : TRUE = underlying NURB linear dir runs from TopCircle to BotCircle
                                                                //      FALSE= underlying NURB linear dir runs from BotCircle to TopCircle
                                               FALSE,           // in : TRUE = GenCurve is type SmBSplineCurve
                                                                //      FALSE= GenCurve is type SmLine
                                               pContext) ;      // in : Set context if given, default:[NULL]

    } // end plane perpendicular to MidDir Axis check

  // surfaces perpendicular to the Binormal Axis map to planes containing the BendAxis (radiating out)
  if(bConstBinormalPlane)
    {
      double dAngRad = (dBinormalPlaneSum/(lSmpSize+1.0)/(lSmpSize+1.0)) / dBendNeutralDist ;
      SmVector3d sXVector = sBendAxis ;
      SmVector3d sYVector = smos_Cosine(dAngRad) * sBendMidDir + smos_Sine(dAngRad) * sBendBinormal ;

      SmVector2d sUVScale(1,1), crUVDomain ;
      SmExtent2d sUVDomain(sAxisPlaneRange, sMidDirPlaneRange) ;  // because planes are convex - we don't have to increase here due to sampling missing a max or min value

      // check orientation of surface against the Input BendBinormal axis
      SmBoolean bPositive = sInputSurfaceNormal.Dot(sBendBinormal) > 0.0 ;

      // Create Plane with SrfNormal = rInsideSurface->Surface Normal
      if(!bPositive) { sXVector = sYVector ;
                       sYVector = sBendAxis ;
                       sUVDomain.Transpose() ;
                     }

      // create new plane - positive orientation:[SrfNormal = sBendAxis X sYVector] (Normal points in pos rot dir about BendAxis)
      rpNewSurface = new (pContext) SmPlane(sBendOrigin,     // in : 3d loc of parametric origin = Plane_Evaluate(0,0)
                                            sXVector,        // in : 3D X Axis - made unit
                                            sYVector,        // in : 3D Y Axis - made unit - should be perp to X
                                            sUVScale,        // in : UScale and VScale
                                            sUVDomain,       // in : UV Domain limiting allowed evaluations
                                            pContext) ;      // in : Set context if given, default:[NULL]
    } // end plane perpendicular to Binormal Axis check

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

} // end SmBendVolume::MakeExactBSpline1stOutSurface

/*******************************************************************//**
PURPOSE: compute OutPoint = Evaluate(ParamPoint) - optional derived implementation.

NOTES: 
 Map UVW ParamSpace point to xyz ProjSpace point and iso-derivatives.

  EvaluateSimple : r      = u ;                
                   theta  = v/d ;              
                   d      = BendNeutralDist ;  
  Position                            
    POut  = BendCenter + u*Cos(v/d)*BendMidDir   
                       + u*Sin(v/d)*BendBinormal    
                       + w         *BendAxis ;                   
  
  1st Derivs                                        
    Du(POut)  =      Cos(v/d)*BendMidDir +     Sin(v/d)*BendBinormal ;  
    Dv(POut)  = u/d*-Sin(v/d)*BendMidDir + u/d*Cos(v/d)*BendBinormal ;
    Dw(POut)  = BendAxis ;
                                                
  2nd Derivs                                         
    Duu(POut) = 0 ;                      
    Duv(POut) =   -1/d*Sin(v/d)*BendMidDir +    1/d*Cos(v/d)*BendBinormal ; 
    Duw(POut) = 0 ;                        
    Dvv(POut) = -u/d/d*Cos(v/d)*BendMidDir + -u/d/d*Sin(v/d)*BendBinormal ;  
    Dvw(POut) = 0 ;  
    Dww(POut) = 0 ;
                                         
  3rd Derivs 
    Duuu = 0 ;                                                    
    Duuv = 0 ; 
    Duuw = 0 ;                                                    
    Duvv =   -1/d/d*Cos(v/d)*BendMidDir +   -1/d/d*Sin(v/d)*BendBinormal ; 
    Duvw = 0 ;                                                    
    Duww = 0 ;                                                    
    Dvvv =  u/d/d/d*Sin(v/d)*BendMidDir + -u/d/d/d*Cos(v/d)*BendBinormal ; 
    Dvvw = 0 ; 
    Dvww = 0 ; 
    Dwww = 0 ;                                                               

 1. At the BendOrigin (ParamPoint(0,0,0), this function's derivatives go to zero
    and are undefined, that is as BendOrigin is approached in the
    limit the derivative values go to zero and the deriviate directions vary
    depending on the direction of the limit.  As such, the argument
    bNonZeroTangents is not used and derivatives taken at the point BendOrigin
    are returned as zero.
 2. The function has a zero point at the BendOrigin and no other internal discontinuities.
    As such, the bFromLeft arguments are not used.
***********************************************************************/
SmStatus SmBendVolume::Evaluate
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

  // check input - crParamPoint is within the valid ParamDomain (u > 0 implemented as u >= m_dSingularityTol)
  if(crParamPoint.x <  GetLegalMinU())
    {
      SE_MSG(SM_ERR, _T("SmBendVolume::Evaluate given out of NaturalParamDomain ParamPoint (u < GetLegalMinU()) - ignoring")) ;
    }

  // For indexing into the volume pt/deriv array:
#define sv(u,v,w) ((u*(lHighestDeriv+1)+v)*(lHighestDeriv+1)+w)

  // locals
  double dU = crParamPoint.x ;
  double dV = crParamPoint.y ;
  double dW = crParamPoint.z ;

  double dCos = smos_Cosine(dV/m_dBendNeutralDist) ;
  double dSin = smos_Sine(dV/m_dBendNeutralDist) ;

  SmVector3d sBendMidDir    = GetBendMidDir() ;
  SmVector3d sBendBinormal  = GetBendBinormal() ;
  SmVector3d sBendAxis      = GetBendAxis() ;
                                                                            
  // init output array 
  ULONG ii, dASize = (lHighestDeriv+1) * (lHighestDeriv+1) * (lHighestDeriv+1) ;
  for(ii=1;ii<dASize;ii++)
    {
      aDerivatives[ii].Set(0,0,0) ;
    }

  // position
  aDerivatives[0] =   GetBendOrigin()
                    + dU * dCos * sBendMidDir 
                    + dU * dSin * sBendBinormal 
                    + dW        * sBendAxis ;

  // 1st derivs
  //   Du(POut)  =      Cos(v/d)*BendMidDir +     Sin(v/d)*BendBinormal ;  
  //   Dv(POut)  = u/d*-Sin(v/d)*BendMidDir + u/d*Cos(v/d)*BendBinormal ;
  //   Dw(POut)  = BendAxis ;
  if(lHighestDeriv >= 1)
    {
      double dDi1 = 1.0 / m_dBendNeutralDist ; 

      /* Du */ aDerivatives[sv(1,0,0)] =           dCos * sBendMidDir +         dSin * sBendBinormal ;
      /* Dv */ aDerivatives[sv(0,1,0)] =  -dU*dDi1*dSin * sBendMidDir + dU*dDi1*dCos * sBendBinormal ;
      /* Dw */ aDerivatives[sv(0,0,1)] =                  sBendAxis ;

      // NonZero 2nd derivs
      //   Duv(POut) =   -1/d*Sin(v/d)*BendMidDir +    1/d*Cos(v/d)*BendBinormal ; 
      //   Dvv(POut) = -u/d/d*Cos(v/d)*BendMidDir + -u/d/d*Sin(v/d)*BendBinormal ;  
      if(lHighestDeriv >= 2)
        {
          double dDi2 = dDi1 * dDi1 ;

          /* Duv */  aDerivatives[sv(1,1,0)] =  -   dDi1*dSin * sBendMidDir +     dDi1*dCos * sBendBinormal ;
          /* Dvv */  aDerivatives[sv(0,2,0)] =  -dU*dDi2*dCos * sBendMidDir + -dU*dDi2*dSin * sBendBinormal ;

          // NonZreo 3rd derivs
          //   Duvv =   -1/d/d*Cos(v/d)*BendMidDir +   -1/d/d*Sin(v/d)*BendBinormal ; 
          //   Dvvv =  u/d/d/d*Sin(v/d)*BendMidDir + -u/d/d/d*Cos(v/d)*BendBinormal ; 
          if(lHighestDeriv >= 3)
            {
              double dDi3 = dDi2 * dDi1 ;

              /* Duvv */ aDerivatives[sv(1,2,0)] =   -   dDi2*dCos * sBendMidDir + -   dDi2*dSin * sBendBinormal ;
              /* Dvvv */ aDerivatives[sv(0,3,0)] =    dU*dDi3*dSin * sBendMidDir + -dU*dDi3*dCos * sBendBinormal ;
                                                
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

} // end SmBendVolume::Evaluate

/*******************************************************************//**
PURPOSE: compute ProjPoint = EvaluateSimple(ParamPoint) - requires derived implementation.

NOTES: 
 Map UVW ParamSpace point to xyz ProjSpace point and iso-derivatives.

  EvaluateSimple : r      = u ;                xProj  = r*Cos(theta) ;
                   theta  = v/d ;              yProj  = r*Sin(theta) ;
                   d      = BendNeutralDist ;  zProj  = w ;
  Position                            
    xProj  = u*Cos(v/d) ;   
    yProj  = u*Sin(v/d) ;   
    zProj  = w ;              
  
  1st Derivs                                        
    Du(xProj)  =   Cos(v/d) ;        Dv(xProj)  = -u/d*Sin(v/d) ;    Dw(xProj)  = 0 ;
    Du(yProj)  =   Sin(v/d) ;        Dv(yProj)  =  u/d*Cos(v/d) ;    Dw(yProj)  = 0 ;
    Du(zProj)  =   0 ;               Dv(zProj)  =  0 ;               Dw(zProj)  = 1 ;          
                                     
  2nd Derivs                                              
    Duu(xProj) = 0 ;                 Dvv(xProj) = -u/d/d*Cos(v/d) ;  Dww(xProj)  = 0 ;
    Duu(yProj) = 0 ;                 Dvv(yProj) = -u/d/d*Sin(v/d) ;  Dww(yProj)  = 0 ;
    Duu(zProj) = 0 ;                 Dvv(zProj) =  0 ;               Dww(zProj)  = 0 ;          
                                     
    Duv(xProj)  = -1/d*Sin(v/d) ;    Duw(xProj)  = 0 ;               Dvw(xProj)  = 0 ;  
    Duv(yProj)  =  1/d*Cos(v/d) ;    Duw(yProj)  = 0 ;               Dvw(yProj)  = 0 ;  
    Duv(zProj)  =  0 ;               Duw(zProj)  = 0 ;               Dvw(zProj)  = 0 ;                  
                                           
  3rd Derivs 
    Duvv(xProj)  = -1/d/d*Cos(v/d) ; Dvvv(xProj) =  u/d/d/d*Sin(v/d) ;
    Duvv(yProj)  = -1/d/d*Sin(v/d) ; Dvvv(yProj) = -u/d/d/d*Cos(v/d) ;
    Duvv(zProj)  =  0 ;              Dvvv(zProj) =  0 ;         
                                         
    Duuu(xProj) = 0 ;                Duuv(xProj) = 0 ;               Duuw(xProj) = 0 ;
    Duuu(yProj) = 0 ;                Duuv(yProj) = 0 ;               Duuw(yProj) = 0 ;
    Duuu(zProj) = 0 ;                Duuv(zProj) = 0 ;               Duuw(zProj) = 0 ;
                                                                    
    Duvw(xProj) = 0 ;                Duww(xProj)  = 0 ;              Dvvw(xProj) = 0 ;
    Duvw(yProj) = 0 ;                Duww(yProj)  = 0 ;              Dvvw(yProj) = 0 ;
    Duvw(zProj) = 0 ;                Duww(zProj)  = 0 ;              Dvvw(zProj) = 0 ;
                                                                    
    Dvww(xProj)  = 0 ;               Dwww(xProj)  = 0 ;             
    Dvww(yProj)  = 0 ;               Dwww(yProj)  = 0 ;             
    Dvww(zProj)  = 0 ;               Dwww(zProj)  = 0 ;             
                                                                    
  InvEvaluateSimple : r     = sqrt(xProj**2 + yProj**2) ;    u = r ;        
                      theta = ArcTan(yProj,xProj) ;          v = d * theta ;
                      d     = BendNeutralDist ;              w = zProj ; 
           
     u     = sqrt(xProj**2 + yProj**2) ;                         
     v     = BendNeutralDist * ArcTan(yProj,xProj) ;                 
     w     = zProj ;                     

 1. At the BendOrigin (ParamPoint(0,0,0), this function's derivatives go to zero
    and are undefined, that is as BendOrigin is approached in the
    limit the derivative values go to zero and the deriviate directions vary
    depending on the direction of the limit.  As such, the argument
    bNonZeroTangents is not used and derivatives taken at the point BendOrigin
    are returned as zero.
 2. The function has a zero point at the BendOrigin and no other internal discontinuities.
    As such, the bFromLeft arguments are not used.
***********************************************************************/
SmStatus SmBendVolume::EvaluateSimple
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

  // check input - crParamPoint is within the valid ParamDomain (u > 0) implemented as (u >= m_dSingularityTol)
  //               allow position evaluations at BendVolume singularities - only the derivatives are undefined
  if( (lHighestDeriv > 0) && (crParamPoint.x < GetLegalMinU()) )
    {
      SE_MSG(SM_ERR, _T("SmBendVolume::EvaluateSimple given out of NaturalParamDomain ParamPoint (u < GetLegalMinU() ) - ignoring")) ;
    }

  // For indexing into the volume pt/deriv array:
#define sv(u,v,w) ((u*(lHighestDeriv+1)+v)*(lHighestDeriv+1)+w)

  // locals
  double dU   = crParamPoint.x ; // needed so that &crParamPoint can be same memory as aDerivatives
  double dV   = crParamPoint.y ; // needed so that &crParamPoint can be same memory as aDerivatives
  double dW   = crParamPoint.z ; // needed so that &crParamPoint can be same memory as aDerivatives

  double dCos = smos_Cosine(dV/m_dBendNeutralDist) ;
  double dSin = smos_Sine(dV/m_dBendNeutralDist) ;
                                                                            
  // init output array
  ULONG ii, dASize = (lHighestDeriv+1) * (lHighestDeriv+1) * (lHighestDeriv+1) ;
  for(ii=1;ii<dASize;ii++)
    {
      aDerivatives[ii].Set(0,0,0) ;
    }

  // position
  aDerivatives[0].Set( dU*dCos, dU*dSin, dW ) ;

  // 1st derivs
  if(lHighestDeriv >= 1)
    {
      double dDi1 = 1.0/m_dBendNeutralDist ;

      /* Du */ aDerivatives[sv(1,0,0)].Set(         dCos,          dSin, 0.0 ) ; 
      /* Dv */ aDerivatives[sv(0,1,0)].Set( -dU*dDi1*dSin, dU*dDi1*dCos, 0.0 ) ; 
      /* Dw */ aDerivatives[sv(0,0,1)].Set(          0.0,           0.0, 1.0 ) ;  

      // 2nd derivs
      if(lHighestDeriv >= 2)
        {
          double dDi2 = dDi1*dDi1 ;

          /* Duu */ // already set to zero: aDerivatives[sv(2,0,0)].Set(           0.0,           0.0,  0.0 ) ;  
          /* Duv */                         aDerivatives[sv(1,1,0)].Set( -   dDi1*dSin,     dDi1*dCos,  0.0 ) ;  
          /* Duw */ // already set to zero: aDerivatives[sv(1,0,1)].Set(           0.0,           0.0,  0.0 ) ;  
          /* Dvv */                         aDerivatives[sv(0,2,0)].Set( -dU*dDi2*dCos, -dU*dDi2*dSin,  0.0 ) ;  
          /* Dvw */ // already set to zero: aDerivatives[sv(0,1,1)].Set(           0.0,           0.0,  0.0 ) ;  
          /* Dww */ // already set to zero: aDerivatives[sv(0,0,2)].Set(           0.0,           0.0,  0.0 ) ;  
                                          

          // 3rd derivs
          if(lHighestDeriv >= 3)
            {
              double dDi3 = dDi2*dDi1 ;

              /* Duuu */ // already set to zero: aDerivatives[sv(3,0,0)].Set(           0.0,           0.0,  0.0 ) ; 
              /* Duuv */ // already set to zero: aDerivatives[sv(2,1,0)].Set(           0.0,           0.0,  0.0 ) ; 
              /* Duuw */ // already set to zero: aDerivatives[sv(2,0,1)].Set(           0.0,           0.0,  0.0 ) ; 
              /* Duvv */                         aDerivatives[sv(1,2,0)].Set( -   dDi2*dCos, -   dDi2*dSin,  0.0 ) ; 
              /* Duvw */ // already set to zero: aDerivatives[sv(1,1,1)].Set(           0.0,           0.0,  0.0 ) ; 
              /* Duww */ // already set to zero: aDerivatives[sv(1,0,2)].Set(           0.0,           0.0,  0.0 ) ; 
              /* Dvvv */                         aDerivatives[sv(0,3,0)].Set(  dU*dDi3*dSin, -dU*dDi3*dCos,  0.0 ) ; 
              /* Dvvw */ // already set to zero: aDerivatives[sv(0,2,1)].Set(           0.0,           0.0,  0.0 ) ; 
              /* Dvww */ // already set to zero: aDerivatives[sv(0,1,2)].Set(           0.0,           0.0,  0.0 ) ; 
              /* Dwww */ // already set to zero: aDerivatives[sv(0,0,3)].Set(           0.0,           0.0,  0.0 ) ;                                                               

            } // end 3rd derivs needed check        
        } // end 2nd derivs needed check
    } // end 1st derivs needed check

  // we don't go any higher all higher derivatives set to zero

  // done with local indexing macro
#undef sv

  // all done
  return SM_SUCCESS ;

} // end SmBendVolume::EvaluateSimple

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
SmStatus SmBendVolume::EvaluateBoundingBoxSimple
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

  // check input - crParamBox can not include the singularity at U = 0
  SmExtent1d sParamUIvl = crParamBox.GetUInterval() ;
  if(smos_Fabs(sParamUIvl.GetMin()) < GetLegalMinU())
    { 
      SE_MSG(SM_ERR, _T("SmBendVolume::EvaluateBoundingBoxSimple given crParamBox that include singularity at (u < GetLegalMinU() ) - ignoring")) ;
    }

  // locals
  SmExtent1d sIvlV = crParamBox.GetVInterval() ;
  double     dVMin = crParamBox.GetVMin() ;
  double     dVMax = crParamBox.GetVMax() ;
  double     dThetaMin = dVMin / m_dBendNeutralDist ;
  double     dThetaMax = dVMax / m_dBendNeutralDist ;
  double     dCosThetaMin   = smos_Cosine(dThetaMin) ;
  double     dSinThetaMin   = smos_Sine(dThetaMin) ;
  double     dCosThetaMax   = smos_Cosine(dThetaMax) ;
  double     dSinThetaMax   = smos_Sine(dThetaMax) ;

  SmExtent1d sIvlU = crParamBox.GetUInterval() ;
  double     dUMin = crParamBox.GetUMin() ;
  double     dUMax = crParamBox.GetUMax() ;

  // when asked build ProjectSPace BBox
  if(pProjBox)
    {
      double dXMax, dXMin, dYMax, dYMin ;

      if( dUMax == SM_INFINITE_PARAMETER )
        {
          // dXMax - infinite when ParamBox maps above the X = 0 plane
          if(   SM_IS_CONTAINED( dThetaMax, -SM_PI/2.0 , SM_PI/2.0)
             || SM_IS_CONTAINED( dThetaMin, -SM_PI/2.0 , SM_PI/2.0)
             || (dThetaMin < -SM_PI/2.0 && dThetaMax > SM_PI/2.0))
            { 
              dXMax = SM_INFINITE_PARAMETER ; 
            }
          else // set Max X from Min Radius Corner Values
            {
              dXMax = smos_Max(dCosThetaMin, dCosThetaMax) * dUMin ;
            }

          // dXMin - negative infinite when ParamPox maps below the X = 0 plane
          if(   SM_IS_CONTAINED( dThetaMin, -SM_PI/2.0 , -SM_PI) || SM_IS_CONTAINED( dThetaMin, SM_PI/2.0 , SM_PI)
             || SM_IS_CONTAINED( dThetaMax, -SM_PI/2.0 , -SM_PI) || SM_IS_CONTAINED( dThetaMax, SM_PI/2.0 , SM_PI))
            {
              dXMin = - SM_INFINITE_PARAMETER ;
            }
          else // set Min X from min Radius Corner Values
            {
              dXMin = smos_Min(dCosThetaMin, dCosThetaMax) * dUMin ;
            }

          // dYMax - infinite when Param Box maps above the Y = 0 plane
          if( dThetaMin >= 0 || dThetaMax > 0)
            {
              dYMax = SM_INFINITE_PARAMETER ;
            }
          else // dYMax is taken from the min radius corner points
            {
              dYMax = smos_Max(dCosThetaMin, dCosThetaMax) * dUMin ;
            }

          // dYMin - negative infinite when paramBox maps below the Y = 0 plane
          if( dThetaMin < 0 || dThetaMax <= 0)
            {
              dYMin = -SM_INFINITE_PARAMETER ;
            }
          else // dYMin is taken from the min radius corner points
            {
              dYMin = smos_Min(dSinThetaMin, dSinThetaMax) * dUMin ;
            }
        } // end infinite param U branch

      else // else working with a finite box

        {
          // X interval is defined by corner points and the Theta = 0 map point
          if(  SM_IS_CONTAINED( 0.0, dThetaMin, dThetaMax))        
            { 
              // Theta = 0.0 is within the ParamBox domain
              dXMax = dUMax ;
              dXMin = smos_4Min( dCosThetaMin * dUMin, dCosThetaMin * dUMax, dCosThetaMax * dUMin, dCosThetaMax * dUMax) ;  
            }
          else // Theta = 0.0 is not part of the domain
            { 
              // X Min and Max are the min and max values of the 4 mapped corner points
              dXMin = smos_4Min( dCosThetaMin * dUMin, dCosThetaMin * dUMax, dCosThetaMax * dUMin, dCosThetaMax * dUMax) ;  
              dXMax = smos_4Max( dCosThetaMin * dUMin, dCosThetaMin * dUMax, dCosThetaMax * dUMin, dCosThetaMax * dUMax) ; 
            }

          // Y interval is defined by the corner points and the Theta = Pi/2 and - Pi/2 points
          if(   SM_IS_CONTAINED( SM_PI/2.0, dThetaMin, dThetaMax)
             && SM_IS_CONTAINED(-SM_PI/2.0, dThetaMin, dThetaMax))
            {
              dYMax =  dUMax ;
              dYMin = -dUMax ;
            }
          else if( SM_IS_CONTAINED( SM_PI/2.0, dThetaMin, dThetaMax))
            {
              dYMax = dUMax ;
              dYMin = smos_4Min( dSinThetaMin * dUMin, dSinThetaMin * dUMax, dSinThetaMax * dUMin, dSinThetaMax * dUMax) ;  
            }
          else if( SM_IS_CONTAINED(-SM_PI/2.0, dThetaMin, dThetaMax))
            {
              dYMax = smos_4Max( dSinThetaMin * dUMin, dSinThetaMin * dUMax, dSinThetaMax * dUMin, dSinThetaMax * dUMax) ; 
              dYMin = -dUMax ;
            }
          else // the corners define the box
            {
              dYMax = smos_4Max( dSinThetaMin * dUMin, dSinThetaMin * dUMax, dSinThetaMax * dUMin, dSinThetaMax * dUMax) ; 
              dYMin = smos_4Min( dSinThetaMin * dUMin, dSinThetaMin * dUMax, dSinThetaMax * dUMin, dSinThetaMax * dUMax) ;
            }
        } // end finit ParamBox branch

      // set the output
      pProjBox->SetMinMax(dXMin, dYMin, crParamBox.GetWMin(), dXMax, dYMax, crParamBox.GetWMax()) ;
   
    } // end build pProjBox check

  // when asked - build a ProjPseduoBox its oriented about the center of mapped ParamBox
  if(pProjPseudoBox)
    {
      double dVMid = ((dVMin + dVMax) / 2.0) ;
      double dVEnd = dVMax - dVMid ;

      double dThetaMid = dVMid / m_dBendNeutralDist ;
      double dThetaEnd = dVEnd / m_dBendNeutralDist ;

      double dCosThetaMid = smos_Cosine(dThetaMid) ;
      double dSinThetaMid = smos_Sine(dThetaMid) ;

      // double dCosThetaEnd = smos_Cosine(dThetaEnd) ;
      double dSinThetaEnd = smos_Sine(dThetaEnd) ;
      double dXMax        = dUMax ;
      double dXMin        = dThetaEnd < SM_PI ? dSinThetaEnd * dUMin : dSinThetaEnd * dUMax ;
      double dYMax        = dThetaEnd < SM_PI ? dSinThetaEnd * dUMax : dUMax ;
      double dYMin        = - dYMax ;
      
      // bases
      SmVector3d sPseudoX( dCosThetaMid, dSinThetaMid, 0.0) ;
      SmVector3d sPseudoY(-dSinThetaMid, dCosThetaMid, 0.0) ;
      SmVector3d sPseudoZ(0.0, 0.0, 1.0) ;

      // intervals
      SmExtent1d sPseudoIvlX( dXMin, dXMax) ;
      SmExtent1d sPseudoIvlY( dYMin, dYMax) ;
      SmExtent1d sPseudoIvlZ(crParamBox.GetWMin(), crParamBox.GetWMax()) ;

      // set output
      pProjPseudoBox->SetBasis(sPseudoX, sPseudoY, sPseudoZ) ;
      pProjPseudoBox->SetIntervals(sPseudoIvlX, sPseudoIvlY, sPseudoIvlZ) ; 
    
    } // end build pProjPseudoBox check

// obsolete
//      
//        // when given pOptParamPseudoBox -get orientation in ProjSpace
//        SmVector3d sPseudoProjX, 
//        
//        EvaluateDirectionalDerivs( = pOptParamPseudoBox->GetBasis(0) ;
//      
//      
//        // when input ParamBox is same as output ProjBox - copy input ParamBox
//        SmExtent3d        sBox ;
//        const SmExtent3d *pBox ;
//        if(&crParamBox == pProjBox) {  sBox = crParamBox ;
//                                       pBox = &sBox ;
//                                    }
//        else                        {  pBox = &crParamBox ;
//                                    }
//      
//        // when input ParamPseudoBox is same as output ProjPseudoBox - copy input ParamPseudoBox
//        SmPseudoBox sPBox, *pPBox ;
//        if(   pProjPseudoBox
//           && pOptParamPseudoBox == pProjPseudoBox) { sPBox = *pOptParamPseudoBox ;
//                                                      pPBox = &sPBox ;
//                                                    }
//        else                                        { pPBox = pOptParamPseudoBox ;
//                                                    }
//      
//        // indirection: from here on out - pBox  == ParamSpace Box
//        //                               - pPBox == ParamSpace PseudoBox
//      
//        // when pBox is unbounded in any dimension return unbounded BBoxes
//        if(!pBox->IsBounded())
//          {
//            if(pProjBox)       { pProjBox->SetUnbounded() ; }
//            if(pProjPseudoBox) { SmExtent1d sIvl ;
//                                 sIvl.SetUnbounded() ;
//                                 pProjPseudoBox->SetBasis(GetBendMidDir(), GetBendBinormal(), GetBendAxis()) ; 
//                                 pProjPseudoBox->SetIntervals(sIvl, sIvl, sIvl) ; 
//                               }
//            // all done
//            return(SM_SUCCESS) ; 
//          }
//        
//        // arrive here given a ParamSpace Domain block to map to a thick wall cylinder in ProjSpace
//      
//        SmContext  sContext ; 
//        SmVector3d sXAxisBendMidDir(1,0,0) ;
//        SmVector3d sYAxisBendBinormal(0,1,0) ;
//        SmVector3d sZAxisBendAxis(0,0,1) ;
//      
//        // ParamSpace Domain locals
//        SmPoint3d sMinUVW = pBox->GetMin() ;
//        SmPoint3d sMaxUVW = pBox->GetMax() ;
//        double    dRMin   = sMinUVW.x ;      // (sMinUVW - m_vBendOrigin).Dot(GetBendMidDir()) ;
//        double    dRMax   = sMaxUVW.x ;      // (sMaxUVW - m_vBendOrigin).Dot(GetBendMidDir()) ;
//        double    dVMin   = sMinUVW.y ;      // (sMinUVW - m_vBendOrigin).Dot(GetBendBinormal()) ;
//        double    dVMax   = sMaxUVW.y ;      // (sMaxUVW - m_vBendOrigin).Dot(GetBendBinormal()) ;
//        double    dWMin   = sMinUVW.z ;      // (sMinUVW - m_vBendOrigin).Dot(GetBendAxis()) ;
//        double    dWMax   = sMaxUVW.z ;      // (sMaxUVW - m_vBendOrigin).Dot(GetBendAxis()) ;
//        double    dThetaMinRad = SM_IS_ZERO(dRMin) ? 0.0 : dVMin / m_dBendNeutralDist ; 
//        double    dThetaMaxRad = SM_IS_ZERO(dRMax) ? 0.0 : dVMax / m_dBendNeutralDist ;
//        dThetaMinRad = SM_CLAMP_TO_PERIOD(dThetaMinRad, -SM_PI, SM_PI) ; 
//        dThetaMaxRad = SM_CLAMP_TO_PERIOD(dThetaMaxRad, -SM_PI, SM_PI) ; 
//      
//        // locals for thick wall cylinder top and bot bounding circular arcs
//        SmPoint3d  sBendCenterBot(0,0,dWMin) ;
//        SmPoint3d  sBendCenterTop(0,0,dWMax) ;
//        SmExtent1d sIvlDeg(SM_RAD2DEG(dThetaMinRad), SM_RAD2DEG(dThetaMaxRad)) ;
//      
//        // circular arcs bounding mapped ParamSpace box extremes
//        SmCircle sTopMinRadius(sBendCenterTop, sXAxisBendMidDir, sYAxisBendBinormal,
//                               sIvlDeg,
//                               dRMin, 3,
//                               &sContext) ;
//        SmCircle sTopMaxRadius(sBendCenterTop, sXAxisBendMidDir, sYAxisBendBinormal,
//                               sIvlDeg,
//                               dRMax, 3,
//                               &sContext) ;
//        SmCircle sBotMinRadius(sBendCenterBot, sXAxisBendMidDir, sYAxisBendBinormal,
//                               sIvlDeg,
//                               dRMin, 3,
//                               &sContext) ;
//        SmCircle sBotMaxRadius(sBendCenterBot, sXAxisBendMidDir, sYAxisBendBinormal,
//                               sIvlDeg,
//                               dRMax, 3,
//                               &sContext) ;
//      
//        // ProjSpace Bounding boxes for arcs
//        SmExtent3d  sThisProjBox,       *pThisProjBox       = pProjBox ?       &sThisProjBox       : NULL ; 
//        SmPseudoBox sThisProjPseudoBox, *pThisProjPseudoBox = pProjPseudoBox ? &sThisProjPseudoBox : NULL ;
//      
//        // build a ProjSpace bounding box for every mapped arc and union those
//      
//        // TopMinRadius Boxes
//        sTopMinRadius.CalculateBoundingBox(sTopMinRadius.GetNaturalInterval(), pThisProjBox, pThisProjPseudoBox) ;
//        if(pProjBox)       *pProjBox       = *pThisProjBox ;
//        if(pProjPseudoBox) *pProjPseudoBox = *pThisProjPseudoBox ;
//      
//        // TopMaxRadius Boxes
//        sTopMaxRadius.CalculateBoundingBox(sTopMaxRadius.GetNaturalInterval(), pThisProjBox, pThisProjPseudoBox) ;
//        if(pProjBox)       pProjBox->Union(*pThisProjBox, *pProjBox) ;
//        if(pProjPseudoBox) pProjPseudoBox->Union(*pThisProjPseudoBox, pProjPseudoBox->GetBasis(), *pProjPseudoBox) ;
//      
//        // BotMinRadius Boxes
//        sBotMinRadius.CalculateBoundingBox(sBotMinRadius.GetNaturalInterval(), pThisProjBox, pThisProjPseudoBox) ;
//        if(pProjBox)       pProjBox->Union(*pThisProjBox, *pProjBox) ;
//        if(pProjPseudoBox) pProjPseudoBox->Union(*pThisProjPseudoBox, pProjPseudoBox->GetBasis(), *pProjPseudoBox) ;
//      
//        // BotMaxRadius Boxes
//        sBotMaxRadius.CalculateBoundingBox(sBotMaxRadius.GetNaturalInterval(), pThisProjBox, pThisProjPseudoBox) ;
//        if(pProjBox)       pProjBox->Union(*pThisProjBox, *pProjBox) ;
//        if(pProjPseudoBox) pProjPseudoBox->Union(*pThisProjPseudoBox, pProjPseudoBox->GetBasis(), *pProjPseudoBox) ;

  // all done
  return(SM_SUCCESS) ;

} // end SmBendVolume::EvaluateBoundingBoxSimple
 
/*******************************************************************//**
PURPOSE: Find the portion of the InSpace Line interval that maps to the NaturalParamDomain

NOTES: 

RETURNS: SM_ERR when ParamBox is trimmed to the empty set, else returns SM_SUCCESS
***********************************************************************/
SmStatus SmBendVolume::FindParamIntervalForInSpaceLineSimple
 (const SmPoint3d      & crInSpacePoint,   // in : LinePoint         of Line(s) = LinePoint + s * LineVector               
  const SmVector3d     & crInSpaceVector,  // in : scaled LineVector of Line(s) = LinePoint + s * LineVector               
  const SmExtent1d     & crCurrentIvl,     // in : current limits on s interval
  SmTArray<SmExtent1d> & rTrimIvls)        // out: Interval of line that maps legally within the NaturalParamDomains
 const                                     
{ 
  // init output
  rTrimIvls.SetSize(1) ;

  // locals
  SmPoint3d  sParamLinePoint, sParamLineVec ;
  SmVector3d sInSpaceUnitVec = crInSpaceVector ;
  SmExtent1d sInSpaceIvl     = crCurrentIvl, sParamIvl  ;     
  double     dLen = crInSpaceVector.Length() ;

  // check input - nonZero InSpaceVector
  if(SM_IS_ZERO(dLen))
    {
      rTrimIvls.ReSet() ;
      SER_MSG(SM_ERR, _T("SmBendVolume::FindParamIntervalForInSpaceLineSimple bad input - zero length tangent vector")) ;
    }

  // check input - unit vector
  if(!SM_IS_ZERO(dLen - 1.0))
    {
      // adjust line scaling
      sInSpaceUnitVec /= dLen ;
      sInSpaceIvl.Scale(dLen) ;
    }

  // map InSpace line to ParamSpace
  SmBoolean bSuccess = TRUE ;
  double    dGap     = 0.0 ;
  InvOrientPoint (crInSpacePoint, bSuccess, sParamLinePoint, dGap) ;
  InvOrientVector(sInSpaceUnitVec, crInSpacePoint, sParamLineVec) ;

  // find InSpace Ivl mapping to NaturalParamDomain (may be infinite or bounded depending on orientation)
  // The BendVolume NaturalParamDomain is bounded by 3 planes, VMin, VMax, and UMin.  There are no W bounds.
  double   dVMin   = -SM_PI * m_dBendNeutralDist ;
  double   dVMax   = -dVMin ;
  SmBoolean bDegen = FALSE ;

  SmBoolean bPointInU = sParamLinePoint.x > GetLegalMinU() ;
  SmBoolean bPointInV = SM_IS_CONTAINED( sParamLinePoint.y, dVMin, dVMax) ;
  SmBoolean bZeroDU   = SM_IS_ZERO(sParamLineVec.x) ;
  SmBoolean bZeroDV   = SM_IS_ZERO(sParamLineVec.y) ;
  double dDU    = sParamLineVec.x ;
  double dDV    = sParamLineVec.y ;

  // LineS values at line/boundary plane intersections
  double dUMinS = bZeroDU ? 0.0 : (m_dSingularityTol - sParamLinePoint.x) / sParamLineVec.x ;
  double dVMinS = bZeroDV ? 0.0 : (dVMin - sParamLinePoint.y) / sParamLineVec.y ; 
  double dVMaxS = bZeroDV ? 0.0 : (dVMax - sParamLinePoint.y) / sParamLineVec.y ;

  // set intervals based on line slope and relative values of dUMinS, dVMinS, and dVMaxS
  if(bZeroDU && bZeroDV) { // lines parallel to w axis are NULL or unbounded
                           if(bPointInU && bPointInV) { sParamIvl.SetMinMax(-SM_INFINITE_PARAMETER, SM_INFINITE_PARAMETER) ; }
                           else                       { bDegen = TRUE ; }
                         }
  else if(bZeroDU)       { // lines with DV and no DU components move between dVMinS to dVMaxS
                           if(bPointInU) { if(dDV > 0) { sParamIvl.SetMinMax(dVMinS, dVMaxS) ; } 
                                           else        { sParamIvl.SetMinMax(dVMaxS ,dVMinS) ; }
                                         }
                           else          { bDegen = TRUE ; }
                         } 
  else if(bZeroDV)       { // lines with DU and no dV components move between dUMinS and infinity
                           if(bPointInV) { if(dDU > 0) { sParamIvl.SetMinMax( dUMinS, SM_INFINITE_PARAMETER) ; } 
                                           else        { sParamIvl.SetMinMax(-SM_INFINITE_PARAMETER, dUMinS) ; }
                                         }
                           else          { bDegen = TRUE ; }
                         }
  else                   { // lines with dU and dV components move between dUMinS, dVMinS, and dVMaxS depending
                           // on slope and ordering of the lineS values 
                           SmBoolean bPosDU = dDU > 0.0 ;
                           SmBoolean bPosDV = dDV > 0.0 ;
                           SmBoolean bMidU  = SM_IS_CONTAINED(dUMinS, smos_Min(dVMinS, dVMaxS), smos_Max(dVMinS, dVMaxS)) ;
                           // when dUMinS not between dVMinS and dVMaxS - interval is null or between VMinS and VMaxS
                           if(!bMidU) { if(   ( bPosDU && !bPosDV)
                                           || (!bPosDU &&  bPosDV)
                                           || ( bPosDU &&  bPosDV)
                                           || (!bPosDU && !bPosDV)) { bDegen = TRUE ; }
                                      }
                           if(bDegen == FALSE)
                             {
                               // init interval to VMinS, VMaxS
                               sParamIvl.SetMinMax(smos_Min(dVMinS, dVMaxS), smos_Max(dVMinS, dVMaxS)) ; }

                               // When bMidU is within VMinS and VMaxS
                               if(bMidU) { // Trim ivl as needed
                                           if(bPosDU) { sParamIvl.SetMinMax(dUMinS, sParamIvl.GetMax()) ; }
                                           else       { sParamIvl.SetMinMax(sParamIvl.GetMin(), dUMinS) ; }
                                         }
                             } // end not a degenerate curve case
                                        
  // arrive here when bDegen == TRUE or sParamIvl is set for ParamLine

  // watch for NULL lines
  if(bDegen || sParamIvl.IsDegenerate(SM_EFF_ZERO))
    {
      // no trim interval to return
      rTrimIvls.ReSet() ;
      return(SM_ERR) ;
    }

  // check input - unit vector
  if(!SM_IS_ZERO(dLen - 1.0))
    {
      // adjust line scaling back to InSpace
      sParamIvl.Scale(1.0/dLen) ;
    }

  // else set rTrimIvls[0] to intersection of sParamIvl and crCurrentIvl
  SmStatus eStat = sParamIvl.Intersect(crCurrentIvl, rTrimIvls[0]) ;

  // watch for disjoint sets
  if(eStat != SM_SUCCESS)
    { rTrimIvls.ReSet() ; }

  // all done
  return(eStat) ;

} // end SmBendVolume::FindParamIntervalForInSpaceLineSimple

// GWC: exclude FindParamExtentForInSpacePlaneSimple from first release. Code is written but I don't
//      see the bug in SmBendVolume::FindParamExtentForInSpacePlaneSimple().  I'll fix that later.
//      
//       *******************************************************************//**
//      PURPOSE: Find the portion of the InSpace Plane extent that maps to the 
//               NaturalParamDomain
//      
//      NOTES: This is an under-determined problem. If one imagines choosing
//        a rectangle when given an orientation by selecting a start corner and 
//        then setting the extents of the sides so tht the rectangle fits within
//        a restricted domain, many different rectangles can be found by
//        varying the selection of the first start corner. 
//      
//      RETURNS: SM_ERR when ParamBox is trimmed to the empty set, else returns SM_SUCCESS
//      *********************************************************************** 
//      SmStatus SmBendVolume::FindParamExtentForInSpacePlaneSimple
//       (const SmPoint3d      & crInSpacePoint,   // in : LinePoint          of Line(s) = LinePoint + u * LineVecU + v * LineVecV              
//        const SmVector3d     & crInSpaceVecU,    // in : scaled LineVectorU of Line(s) = LinePoint + u * LineVecU + v * LineVecV              
//        const SmVector3d     & crInSpaceVecV,    // in : scaled LineVectorV of Line(s) = LinePoint + u * LineVecU + v * LineVecV              
//        const SmExtent2d     & crCurrentUV,      // in : current limits on UV extent
//        SmTArray<SmExtent2d> & rTrimUVs)         // out: UVExtent of plane that maps legally within the NaturalParamDomains
//       const
//      { 
//        // locals
//        SmPoint3d  sParamPlanePoint, sParamPlaneVecU, sParamPlaneVecV ;
//        //      SmVector3d sInSpaceUnitVecU = crInSpaceVecU ;
//        //      SmVector3d sInSpaceUnitVecV = crInSpaceVecV ;
//        //      SmExtent2d sInSpaceUV       = crCurrentUV ;     
//        double     dLenU            = crInSpaceVecU.Length() ;
//        double     dLenV            = crInSpaceVecV.Length() ;
//      
//        // check input - nonZero InSpaceVectors
//        if(SM_IS_ZERO(dLenU) || SM_IS_ZERO(dLenV))
//          {
//            rTrimUVs.ReSet() ;
//            SER_MSG(SM_ERR, _T("SmBendVolume::FindParamExtentForInSpacePlaneSimple bad input - zero length tangent vector")) ;
//          }
//      
//        //      // min dist from plane to NaturalParamDomain corners
//        //      double dDistToPlane = sNaturalParamDomain.DistanceToPlane(sPin, sNormal) ;
//        //      
//        //      // no IsoSurface to return when plane does not intersect NaturalParamDomain
//        //      if(dDistToPlane > SM_EFF_ZERO)
//        //        {
//        //          return(SM_SUCCESS) ;
//        //        }
//      
//        // The BendVolume NaturalParamDomain is bounded by 3 planes, VMin, VMax, that limit the domain to the principle period
//        // and UMin which limits the domain from the singularity at the plane U=0.  There are no W bounds.
//        double     dUMin  = m_dSingularityTol ;
//        double     dVMin  = -SM_PI * m_dBendNeutralDist ;
//        double     dVMax  = -dVMin ;
//        SmVector3d sAxisU(1,0,0) ;
//        SmVector3d sAxisV(0,1,0) ;
//        SmVector3d sAxisW(0,0,1) ;
//        SmBoolean  bDegen = FALSE ;
//      
//        //      // adjust for nonUnit vectors
//        //      SmBoolean bScaleU = SM_IS_ZERO(dLenU - 1.0) == FALSE ;
//        //      SmBoolean bScaleV = SM_IS_ZERO(dLenV - 1.0) == FALSE ;
//        //      if(bScaleU) { sInSpaceUnitVecU /= dLenU ; }
//        //      if(bScaleV) { sInSpaceUnitVecV /= dLenV ; }
//        //      if(bScaleU || bScaleV)
//        //        {
//        //          sInSpaceUV.Scale(dLenU, &dLenV) ;
//        //        }
//      
//        // map InSpace Plane point and tangents to ParamSpace
//        SmBoolean bSuccess = TRUE ;
//        double    dGap     = 0.0 ;
//        InvOrientPoint (crInSpacePoint, bSuccess, sParamPlanePoint, dGap) ;
//        InvOrientVector(crInSpaceVecU, crInSpacePoint, sParamPlaneVecU) ;
//        InvOrientVector(crInSpaceVecV, crInSpacePoint, sParamPlaneVecV) ;
//      
//        // ParamPlane Normal Vector
//        SmVector3d sParamPlaneNormal = sParamPlaneVecU * sParamPlaneVecV ;
//        sParamPlaneNormal.Unitize() ;
//      
//        // find intersection points of Umin/Vmin/Plane and UMin/VMax/Plane planes
//        SmPoint3d sMinCornerParamPoint, sMaxCornerParamPoint ;
//        SmPoint3d sVMinPoint(0,dVMin,0) ;
//        SmPoint3d sVMaxPoint(0,dVMax,0) ;
//        SmPoint3d sUMinPoint(dUMin,0,0) ;
//        SmStatus eStatMin = smgu_IntersectThreePlanes(sVMinPoint,       sAxisV, 
//                                                      sUMinPoint,       sAxisU,  
//                                                      sParamPlanePoint, sParamPlaneNormal,  
//                                                      sMinCornerParamPoint) ; 
//        SmStatus eStatMax = smgu_IntersectThreePlanes(sVMaxPoint,       sAxisV,  
//                                                      sUMinPoint,       sAxisU,  
//                                                      sParamPlanePoint, sParamPlaneNormal,  
//                                                      sMaxCornerParamPoint) ; 
//        SM_ASSERT(eStatMin == eStatMax) ;
//      
//        // locals
//        SmVector3d sVec, sIncludePoint ;
//        double     dExtMinU, dExtMaxU=0, dExtMinV, dExtMaxV=0 ;
//        double     dExtU=0, dExtV=0 ;
//      
//        // when plane normal is parallel to V or U param axis, eStateMin == SM_ERR_INVALID_INPUT
//        if(eStatMin == SM_ERR_INVALID_INPUT)
//          {
//            // when plane normal is parallel to sAxisU
//            if(sAxisU.IsParallelTo(sParamPlaneNormal, SM_EFF_ZERO_DEG))
//              {
//                // found a plane parallel to VW plane, Check for outside domain U values
//                if(sParamPlanePoint.x < GetLegalMinU())
//                  {
//                    // no intersection
//                    rTrimUVs.SetSize(0) ;
//                    return(SM_SUCCESS) ;
//                  }
//      
//                // legal domain of plane includes all W values and V values between Vmin and VMax
//                // the domain is limited by the orientation of the given basis vectors
//      
//                // when sParamPlaneVecU is parallel to W Axis - VecU extent is unbounded, VecV extent bounded by Vmin/Vmax intersections
//                if(sParamPlaneVecU.IsParallelTo(sAxisW, SM_EFF_ZERO_DEG))
//                  {
//                    dExtMinU = -SM_INFINITE_PARAMETER ;
//                    dExtMaxU =  SM_INFINITE_PARAMETER ;
//      
//                    dExtMinV = (dVMin - sParamPlanePoint.y) / sParamPlaneVecV.y ;
//                    dExtMaxV = (dVMax - sParamPlanePoint.y) / sParamPlaneVecV.y ;
//      
//                  } // end sParamPlaneNormal parallel to U and sParamPlaneVecU is parallel to W branch
//      
//                // when sParamPlaneVecV is parallel to W Axis - VecV extent is unbounded, VecU extent bounded by Vmin/Vmax intersections
//                else if(sParamPlaneVecV.IsParallelTo(sAxisW, SM_EFF_ZERO_DEG))
//                  {
//                    dExtMinU = (dVMin - sParamPlanePoint.y) / sParamPlaneVecU.y ;
//                    dExtMaxU = (dVMax - sParamPlanePoint.y) / sParamPlaneVecU.y ;
//      
//                    dExtMinV = -SM_INFINITE_PARAMETER ; 
//                    dExtMaxV =  SM_INFINITE_PARAMETER ; 
//      
//                  } // end sParamPlaneNormal parallel to U and sParamPlaneVecV is parallel to W branch
//      
//                // else pick extents to include point [UPlane, Vmin, 0] and size the extents so that 
//                //  (extentUPlaneMax-extentUPlaneMin)*sin(AngleVToUPlane) = 1/2 (Vmax - Vmin)
//                //  (extentVPlaneMax-extentVPlaneMin)*sin(AngleVToVPlane) = 1/2 (Vmax - Vmin)
//                else
//                  {
//                    double dSinVToUPlaneRad, dSinVToVPlaneRad ;
//                    sAxisV.AngleBetween(sParamPlaneVecU, dSinVToUPlaneRad) ;
//                    sAxisV.AngleBetween(sParamPlaneVecV, dSinVToVPlaneRad) ;
//      
//                    // sVec = sTgt - sParamPlanePoint
//                    sVec.Set(0, dVMin - sParamPlanePoint.y, -sParamPlanePoint.z) ;
//                     
//                    dExtMinU = sParamPlaneVecU.Dot(sVec) ;
//                    dExtMinV = sParamPlaneVecV.Dot(sVec) ;
//                    dExtMaxU = dExtMinU + (dVMax - dVMin)/2.0/dSinVToUPlaneRad ;
//                    dExtMaxV = dExtMinV + (dVMax - dVMin)/2.0/dSinVToVPlaneRad ;
//                    
//                  } // end sParamPlaneNormal parallel to U and sParamPlaneVecU and sParamPlaneVecV are not parallel to W
//              } // end sParamPlaneNormal is parallel with U branch
//      
//            else // plane is parallel to UW plane (i.e. PlaneNormal is parallel to V axis)
//              {
//                SM_ASSERT( sAxisV.IsParallelTo(sParamPlaneNormal, SM_EFF_ZERO_DEG) ) ;
//      
//                // found a plane parallel to UW plane, check V to see that it's inside Vmin to Vmax range
//                if(!SM_IS_CONTAINED(sParamPlanePoint.y, dVMin, dVMax))
//                  {
//                    // no intersection
//                    rTrimUVs.SetSize(0) ;
//                    return(SM_SUCCESS) ;
//                  }
//      
//                // legal domain of plane includes all W values and U values >= m_dSingularityTolerance
//      
//                // when sParamPlaneVecU is parallel to W Axis - VecU extent is unbounded, VecV extent bounded by Umin intersection
//                if(sParamPlaneVecU.IsParallelTo(sAxisW, SM_EFF_ZERO_DEG))
//                  {
//                    dExtMinU = -SM_INFINITE_PARAMETER ;
//                    dExtMaxU =  SM_INFINITE_PARAMETER ;
//      
//                    dExtMinV = (dUMin - sParamPlanePoint.x) / sParamPlaneVecV.x ;
//                    dExtMaxV = sParamPlaneVecV.x > 0.0 ? SM_INFINITE_PARAMETER : -SM_INFINITE_PARAMETER ;
//      
//                  } // end sParamPlaneNormal parallel to V and sParamPlaneVecU is parallel to W branch
//      
//                // when sParamPlaneVecV is parallel to W Axis - VecV extent is unbounded, VecU extent bounded by Umin intersection
//                if(sParamPlaneVecV.IsParallelTo(sAxisW, SM_EFF_ZERO_DEG))
//                  {
//                    dExtMinU = (dUMin - sParamPlanePoint.x) / sParamPlaneVecU.x ;
//                    dExtMaxU = sParamPlaneVecU.x > 0.0 ? SM_INFINITE_PARAMETER : -SM_INFINITE_PARAMETER ;
//      
//                    dExtMinV = -SM_INFINITE_PARAMETER ;
//                    dExtMaxV =  SM_INFINITE_PARAMETER ;
//      
//                  } // end sParamPlaneNormal parallel to V and sParamPlaneVecV is parallel to W branch
//      
//                // else pick extents to include point [m_dSingularityTol, 0, 0] and size the extents so that 
//                //  they run to infinity in appropriate directions
//                //  
//                else
//                  {
//                    // sVec = sTgt - sParamPlanePoint
//                    sVec.Set(m_dSingularityTol-sParamPlanePoint.x,  -sParamPlanePoint.y, -sParamPlanePoint.z) ; 
//      
//                    dExtMinU = sParamPlaneVecU.Dot(sVec) ;
//                    dExtMinV = sParamPlaneVecV.Dot(sVec) ;
//      
//                    dExtMaxU = sParamPlaneVecU.x > 0.0 ? SM_INFINITE_PARAMETER : -SM_INFINITE_PARAMETER ;
//                    dExtMaxV = sParamPlaneVecV.x > 0.0 ? SM_INFINITE_PARAMETER : -SM_INFINITE_PARAMETER ;
//                    
//                  } // end sParamPlaneNormal parallel to V and sParamPlaneVecU and sParamPlaneVecV are not parallel to W
//              } // end sParamPlaneNormal parallel to V branch
//          } // end sParamPlaneNormal parallel to U or V axis branch
//        else // plane is not parallel to one of the NaturalDomainBoundaries
//          {
//            // legal domain of plane includes all W values, V values between Vmin and VMax, and U values >= m_dSingularityTolerance
//            
//            // consider the projections of sParamPlaneVecU and sParamPlaneVecV to the UV plane
//            // call those projections PlaneU and PlaneV
//            SmBoolean bPlaneUParallelU = SM_IS_ZERO(sParamPlaneVecU.y) ;
//            SmBoolean bPlaneVParallelU = SM_IS_ZERO(sParamPlaneVecV.y) ;
//      
//            SmBoolean bPlaneVecUPosSlope = (sParamPlaneVecU.x * sParamPlaneVecU.y) > 0 ;
//            SmBoolean bPlaneVecVPosSlope = (sParamPlaneVecV.x * sParamPlaneVecV.y) > 0 ;
//      
//            // cases
//            // if     ( bPlaneUParallelU &&  bPlaneVParallelV)  - Uext(dUMinXSect, Infinity), Vext(dVMinXSect, dVMaxXSect)
//            // else if( bPlaneUParallelU && !bPlaneVParallelV)  - include point(bPlaneVecVPosSlope ? sMinCorner : sMaxCorner)
//            //                                                  - Uext(dUMinXSect, Infinity), Vext(IncludePoint, VrangeRule)
//            // else if(!bPlaneUParallelU &&  bPlaneVParallelV)  - include point (sMidCornerParamPoint)
//            //                                                  - Uext(IncludePoint, 1/2 VrangeRule)
//            //                                                  - Vext(IncludePoint, 1/2 VrangeRule)
//            //
//            // else if( bPlaneVParallelU &&  bPlaneUParallelV)  - Vext(dUMinXSect, Infinity), Uext(dVMinXSect, dVMaxXSect)   
//            // else if( bPlaneVParallelU && !bPlaneUParallelV)  - include point(bPlaneVecUPosSlope ? sMinCorner : sMaxCorner)
//            //                                                  - Vext(dUMinXSect, Infinity), Uext(IncludePoint, VrangeRule) 
//            // else if(!bPlaneVParallelU &&  bPlaneUParallelV)  - include point (sMidCornerParamPoint)                       
//            //                                                  - Uext(IncludePoint, 1/2 VrangeRule)                         
//            //                                                  - Vext(IncludePoint, 1/2 VrangeRule)                         
//            //
//            // else if(!bPlaneVParallelU && !bPlaneUParallelV)  - include point (sMidCornerParamPoint)
//            //                                                  - Uext(IncludePoint, 1/2 VrangeRule)  
//            //                                                  - Vext(IncludePoint, 1/2 VrangeRule)  
//      
//            if     (bPlaneUParallelU) { dExtMaxU = sParamPlaneVecU.x > 0.0 ? SM_INFINITE_PARAMETER : -SM_INFINITE_PARAMETER ;  
//                                        // include point (bPlaneVecUPosSlope ? sMinCornerParamPoint : sMaxCornerParamPoint)
//                                        // Uext(IncludePoint, Infinity)
//                                        // Vext(IncludePointXSect, OtherPlaneXSect)
//                                        sIncludePoint.Set( m_dSingularityTol, bPlaneVecVPosSlope ? dVMin : dVMax, sParamPlanePoint.z) ;
//                                        dExtV = (dVMax - dVMin) / sParamPlaneVecV.y ;
//                                      }
//                   
//            else if(bPlaneVParallelU) { dExtMaxV = sParamPlaneVecV.x > 0.0 ? SM_INFINITE_PARAMETER : -SM_INFINITE_PARAMETER ;  
//                                        // include point (bPlaneVecUPosSlope ? sMinCornerParamPoint : sMaxCornerParamPoint)
//                                        // Uext(IncludePoint, Infinity)
//                                        // Vext(IncludePointXSect, OtherPlaneXSect)
//                                        sIncludePoint.Set( m_dSingularityTol, bPlaneVecUPosSlope ? dVMin : dVMax, sParamPlanePoint.z) ;
//                                        dExtU = (dVMax - dVMin) / sParamPlaneVecU.y ;
//                                      }
//      
//            else                      { // include point is sMidCornerParamPoint
//                                        sIncludePoint.Set( m_dSingularityTol, (dVMin + dVMax)/2.0, sParamPlanePoint.z) ;
//                                        dExtU = (dExtU == 0) ? (dVMax - dVMin)/ 2.0 / sParamPlaneVecU.y : dExtU ;
//                                        dExtV = (dExtV == 0) ? (dVMax - dVMin)/ 2.0 / sParamPlaneVecV.y : dExtV ;
//                                      }
//      
//            // set ExtMinPt to the IncludePoint
//            sVec     = sIncludePoint - sParamPlanePoint ;
//            dExtMinU = sParamPlaneVecU.Dot(sVec) ;
//            dExtMinV = sParamPlaneVecV.Dot(sVec) ;
//      
//            // set ExtMaxPt to get desired size of 
//            dExtMaxU = (dExtMaxU != 0 ? dExtMaxU : dExtMinU + dExtU) ;
//            dExtMaxV = (dExtMaxV != 0 ? dExtMaxV : dExtMinV + dExtV) ;
//      
//          } // end plane normal not parallel to U or V axis branch
//      
//        // init output
//        rTrimUVs.SetSize(1) ;
//      
//        rTrimUVs[0].SetMinMax(smos_Min(dExtMinU, dExtMaxU), 
//                              smos_Min(dExtMinV, dExtMaxV),
//                              smos_Max(dExtMinU, dExtMaxU), 
//                              smos_Max(dExtMinV, dExtMaxV)) ;
//      
//        // Trim found maximum extent to limits  
//        SmStatus sStat = rTrimUVs[0].Intersect(crCurrentUV, rTrimUVs[0]) ;
//      
//      #ifdef SM_DEBUG_CODE
//          // locals
//          ULONG ii, jj ;
//          SmPoint3d sInSpacePoint, sParamPoint ;
//          SmBoolean bTestSuccess ;
//          double dTestGap ;
//      
//          // check corners are valid param points
//          for(ii=0;ii<2;ii++)
//            {
//              for(jj=0;jj<2;jj++)
//                {
//                  SmPoint2d sPlaneParam = rTrimUVs[0].Evaluate(ii, jj) ;
//                  sInSpacePoint = crInSpacePoint + sPlaneParam.x * crInSpaceVecU + sPlaneParam.y * crInSpaceVecV ;
//                  InvOrientPoint(sInSpacePoint, bTestSuccess, sParamPoint, dTestGap) ;
//      
//                  SmBoolean bInParamDomain   = IsPointInParamDomain(sParamPoint) ;
//                  SmBoolean bInInSpaceDomain = IsPointInInSpaceDomain(sInSpacePoint) ;
//      
//                  SM_ASSERT_MSG((bInParamDomain && bInInSpaceDomain), _T("SmBendVolume::FindParamExtentForInSpacePlaneSimple generated a bad domain point")) ;
//      
//                } // end iter jj for every corner
//            } // end iter ii for every corner
//      
//      #endif // SM_DEBUG_CODE
//        
//        // all done
//        return(sStat) ;
//      
//      } // end SmBendVolume::FindParamExtentForInSpacePlaneSimple

/*******************************************************************//**
PURPOSE: Trim ParamSpace BBox to Natural Parameter Domain

NOTES: Trim crParamBox to 
          1. the positive 1/2 space, U > 0
          2. the Primary Period
RETURNS: SM_ERR when ParamBox is trimmed to the empty set, else returns SM_SUCCESS
***********************************************************************/
SmStatus SmBendVolume::TrimParamBoundingBoxSimple
 (const SmExtent3d & crParamBox,              // in : Tgt Param Box to trim
  SmExtent3d       & rParamTrimBox)           // in : Box trimmed to Natural Param Domain
 const  
{
  // locals
  double dUMin = crParamBox.GetUMin() ;
  double dVMin = crParamBox.GetVMin() ;
  double dWMin = crParamBox.GetWMin() ;

  double dUMax = crParamBox.GetUMax() ;
  double dVMax = crParamBox.GetVMax() ;
  double dWMax = crParamBox.GetWMax() ;

  // Trim to 1/2 space, U > 0 - boxes totally outside the legal domain are turned into negative volumes
  if(dUMax < m_dSingularityTol) { rParamTrimBox.Init() ; 
                                  return(SM_ERR) ;
                                }
  if(dUMin < m_dSingularityTol) { dUMin = m_dSingularityTol ; }

  // Get primary period
  double dPeriodMin = m_dBendNeutralDist * -SM_PI ;
  double dPeriodMax = -dPeriodMin ;

  // boxes outside the legal domain are turned into negative volumes
  if(   dVMax < dPeriodMin - SM_EFF_ZERO
     || dVMin > dPeriodMax + SM_EFF_ZERO)
    {
      rParamTrimBox.Init() ; 
      return(SM_ERR) ;
    }

  // Trim to Primary Period 
  dVMin = SM_CLAMP_TO_PERIOD(dVMin, dPeriodMin, dPeriodMax) ;
  dVMax = SM_CLAMP_TO_PERIOD(dVMax, dPeriodMin, dPeriodMax) ;

  // set output
  rParamTrimBox.SetMinMax( dUMin, dVMin, dWMin, dUMax, dVMax, dWMax) ;

  // all done
  return(SM_SUCCESS) ;

} // end SmBendVolume::TrimParamBoundingBoxSimple                                                 

//     // GWC - removed from interface because it made no sense: Even very simple cases of trimming a BBox to a long skinny PseduoBox showed
//     //       that the inscribed Box could be very small and non-unique.  It won't necessarily capture the notion of a good
//     //       bounding box designed to help narrow down the space of options for functions that need to check that.  In general,
//     //       check wether individual InSpace and ProjPace Points map back to legal ParamDomain Points, one point at a time. 
//  /*******************************************************************//**
//  PURPOSE: Trim an InSpace BBox so that it maps totally within the Natural 
//           ParamSpace Domain
//  
//  METHOD: 1. Map InSpace BBox to ParamSpace
//          2. Trim ParamSpace BBox to NaturalParamSpaceDomain
//          3. Apply the same trimming ratios to the input InSpace BBox
//  
//  NOTES: This function is useful for methods like MapIsoParametricCurve() which
//         would like to size an IsoCurve to the limits of the NaturalParamDomain
//         while working in InSpace.
//  
//         The problem is that those functions start with the InSpace BBox returned
//         by GetNaturalInSpaceDomain() which is just the bounding box of the NaturalParamDomain mapped
//         back to InSpace.  When the m_pInvOrient mapping includes a rotation
//         the NaturalInSpace box will include more points than just those that mapped from 
//         the NaturalParamSpaceDomain.  When samples from that InSpaceBox are mapped
//         back to ParamSpace they can easily contain points outside the NaturalParamDomain.
//         To prevent that problem, this method will reduce the size of a InSpaceBBox so 
//         that it maps totally within the NaturalParamDomain.  This BBox won't include
//         all points that map to the ParamSpaceDomain, but it will only include points that
//         do.
//  
//  RETURNS: SM_ERR when InSpaceBox is trimmed to the empty set, else returns SM_SUCCESS
//  ***********************************************************************/
//  SmStatus SmBendVolume::TrimInSpaceBoundingBoxSimple
//   (const SmExtent3d & crInSpaceBox,              // in : Tgt InSpace Box to trim
//    SmExtent3d       & rInSpaceTrimBox)           // in : Box trimmed to Natural InSpace Domain
//   const  
//  {
//    // locals
//    SmExtent3d  sParamBox, sTrimParamBox ;
//    SmVolume  * pOrientMap = GetOrientMap() ;
//  
//    // Map crInSpaceBox to ParamSpace
//    if(pOrientMap)
//      {
//        // The OrientMap ParamSpace->OutSpace is equal to the SmBendVolume InSpace->ParamSpace mapping.
//        pOrientMap->EvaluateBoundingBox(crInSpaceBox, NULL,  // in : OrientMap Param boxes
//                                       &sParamBox, NULL,     // out: OrientMap OutSpace boxes
//                                        NULL, NULL,          // out: OrientMap InSpace boxes
//                                        FALSE) ;             // in : bWithCompounding
//      }
//    else // without OrientMap branch
//      {
//        sParamBox = crInSpaceBox ;
//      }
//  
//    // Trim the ParamBox
//    SmStatus eStat = TrimParamBoundingBoxSimple(sParamBox, sTrimParamBox) ;
//  
//    // watch out for empty boxes
//    if(eStat != SM_SUCCESS)
//      {
//        rInSpaceTrimBox.Init() ; 
//        return(SM_ERR) ;
//      }
//  
//    // next apply ParamBox trims to the InSpaceBox
//  
//    // locals
//    double dUMinS = (sTrimParamBox.GetUMin() - sParamBox.GetUMin()) / (sParamBox.GetUMax() - sParamBox.GetUMin()) ;
//    double dUMaxS = (sTrimParamBox.GetUMax() - sParamBox.GetUMin()) / (sParamBox.GetUMax() - sParamBox.GetUMin()) ;
//  
//    double dVMinS = (sTrimParamBox.GetVMin() - sParamBox.GetVMin()) / (sParamBox.GetVMax() - sParamBox.GetVMin()) ;
//    double dVMaxS = (sTrimParamBox.GetVMax() - sParamBox.GetVMin()) / (sParamBox.GetVMax() - sParamBox.GetVMin()) ;
//  
//    double dWMinS = (sTrimParamBox.GetWMin() - sParamBox.GetWMin()) / (sParamBox.GetWMax() - sParamBox.GetWMin()) ;
//    double dWMaxS = (sTrimParamBox.GetWMax() - sParamBox.GetWMin()) / (sParamBox.GetWMax() - sParamBox.GetWMin()) ;
//  
//    double dUMin = (1.0 - dUMinS) * crInSpaceBox.GetUMin() + dUMinS * crInSpaceBox.GetUMax() ;
//    double dVMin = (1.0 - dVMinS) * crInSpaceBox.GetVMin() + dVMinS * crInSpaceBox.GetVMax() ;
//    double dWMin = (1.0 - dWMinS) * crInSpaceBox.GetWMin() + dWMinS * crInSpaceBox.GetWMax() ;
//  
//    double dUMax = (1.0 - dUMaxS) * crInSpaceBox.GetUMin() + dUMaxS * crInSpaceBox.GetUMax() ;
//    double dVMax = (1.0 - dVMaxS) * crInSpaceBox.GetVMin() + dVMaxS * crInSpaceBox.GetVMax() ;
//    double dWMax = (1.0 - dWMaxS) * crInSpaceBox.GetWMin() + dWMaxS * crInSpaceBox.GetWMax() ;
//  
//    // set output
//    rInSpaceTrimBox.SetMinMax( dUMin, dVMin, dWMin, dUMax, dVMax, dWMax) ;
//  
//    // all done
//    return(SM_SUCCESS) ;
//  
//  } // end SmBendVolume::TrimInSpaceBoundingBoxSimple                                                 
// End GWC removed obsolete Method
   
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
SmStatus SmBendVolume::EvaluateIsoParametricCurveSimple
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

  // check input - do not generate IsoParamCurves too close to the Volume Singularity at U = 0
  if(   (   eConstantParams == SM_VPS_UV
         || eConstantParams == SM_VPS_UW)
     && (dIsoParam1 < GetLegalMinU()))
    { 
      SER_MSG(SM_ERR, _T("SmBendVolume::EvaluateIsoParametricCurveSimple() bad U param must be positive due to singularity at (u < GetLegalMinU() ) - ignoring")) ;
    }

  // locals        
  SmVector3d        sU(1,0,0) ;
  SmVector3d        sV(0,1,0) ;
  SmVector3d        sW(0,0,1) ;
  SmExtent3d        sParamDomain = GetNaturalParamDomain() ;
  const SmExtent3d *pTgtDomain   = pOptParamDomain ? pOptParamDomain : &sParamDomain ;

  // varying u - return half-infinite line perpendicular (radiating radially) to BendAxis
  if(eConstantParams == SM_VPS_VW)
    {
      // locals
      double dUMin     = pTgtDomain->GetUMin() ; 
      double dUMax     = pTgtDomain->GetUMax() ;
      double dV        = dIsoParam1 ;  
      double dW        = dIsoParam2 ;  
      double dThetaRad = dV / m_dBendNeutralDist ;

      // limit the rotation to +/-180 deg
      dThetaRad = smos_Max(-SM_PI, dThetaRad) ; 
      dThetaRad = smos_Min( SM_PI, dThetaRad) ; 

      // when dV or dW is not within pOptParamDomain
      if(   !pTgtDomain->GetVInterval().ContainsValue(dV, SM_EFF_ZERO) 
         || !pTgtDomain->GetWInterval().ContainsValue(dW, SM_EFF_ZERO)) 
        {
          // no curve to return - all done
          return(SM_SUCCESS) ;
        }

      // Trim UIvl
      dUMin = pTgtDomain->GetUMin() ;
      dUMax = pTgtDomain->GetUMax() ;

      // check input - dU must be positive
      if(dUMin < GetLegalMinU())
        { SE_MSG(SM_ERR,_T("SmBendVolume::EvaluateIsoParametricCurveSimple() bad min U value, most be positive - attempting to fix")) ; 
          dUMin = m_dSingularityTol ;
        }
      if(dUMax < GetLegalMinU())
        { 
          SE_MSG(SM_ERR,_T("SmBendVolume::EvaluateIsoParametricCurveSimple() bad max U interval, most be positive - no curve to return")) ; 
      
          // no curve to return - all done
          return(SM_SUCCESS) ;
        }

      // IsoLine properties
      SmPoint3d  sLinePoint(dW * sW) ;
      SmPoint3d  sLineVector = smos_Cosine(dThetaRad)*sU + smos_Sine(dThetaRad)*sV ;
      SmExtent1d sLineAnalDomain(dUMin, dUMax) ;

      // cull degenerate curves
      if(sLineAnalDomain.IsDegenerate())
        {
          // return degenerate curve
          SmCurve::CreateDegenerateCurve(crContext, 3, sLinePoint, rpNewIsoCurve) ; 
          return(SM_SUCCESS) ;
        }

      // create IsoLine
      rpNewIsoCurve = new (crContext) SmLine(sLinePoint, sLineVector, sLineAnalDomain) ; 

    } // end varying u return line radiating radially from BendAxis branch

  // varying v - return circular arc
  else if(eConstantParams == SM_VPS_UW)
    {
      double dU = dIsoParam1 ; 
      double dW = dIsoParam2 ;
      double dThetaMinDeg = SM_RAD2DEG(pTgtDomain->GetVMin() / m_dBendNeutralDist) ;
      double dThetaMaxDeg = SM_RAD2DEG(pTgtDomain->GetVMax() / m_dBendNeutralDist) ;

      // limit Thetas to +/-180 degrees
      dThetaMinDeg = SM_CLAMP_TO_PERIOD(dThetaMinDeg, -180.0, 180.0) ; 
      dThetaMaxDeg = SM_CLAMP_TO_PERIOD(dThetaMaxDeg, -180.0, 180.0) ; 
      
      // check input - dU must be positive
      if(dU < GetLegalMinU())
        { 
          SE_MSG(SM_ERR,_T("SmBendVolume::EvaluateIsoParametricCurveSimple() bad U value, most be positive - no curve to return")) ; 
          
          // no curve to return - all done
          return(SM_SUCCESS) ;
        }
      
      // when dU or dW is not within pOptParamDomain
      if(   !pTgtDomain->GetUInterval().ContainsValue(dU, SM_EFF_ZERO) 
         || !pTgtDomain->GetWInterval().ContainsValue(dW, SM_EFF_ZERO)) 
        {
          // no curve to return - all done
          return(SM_SUCCESS) ;
        }

      // circle properties 
      SmPoint3d  sCenter(dW * sW) ;
      SmExtent1d sIvlDeg(dThetaMinDeg, dThetaMaxDeg) ;

      // cull degenerate curves
      if(sIvlDeg.IsDegenerate())
        {
          // return degenerate curve
          SmPoint3d sPoint = sCenter + dU * smos_CosDeg(dThetaMinDeg) * sU
                                     + dU * smos_SinDeg(dThetaMinDeg) * sV ;
          SmCurve::CreateDegenerateCurve(crContext, 3, sPoint, rpNewIsoCurve) ; 
          return(SM_SUCCESS) ;
        }

      // create new circle - XAxis = BendMidDir, YAxis = BendBinormal
      rpNewIsoCurve = new (crContext) SmCircle(sCenter, sU, sV, sIvlDeg, dU) ; 

    } // end varying v return circular arc branch

  // varying w - return line parallel to GetBendAxis()
  else if(eConstantParams == SM_VPS_UV)
    {
      double dU        =  dIsoParam1 ;
      double dV        =  dIsoParam2 ;
      double dWMin     =  pTgtDomain->GetWMin() ;
      double dWMax     =  pTgtDomain->GetWMax() ;
      double dThetaRad =  dV / m_dBendNeutralDist ; 
      
      // limit Theta to +/-180 degrees
      dThetaRad = smos_Max(dThetaRad, -SM_PI) ;
      dThetaRad = smos_Min(dThetaRad,  SM_PI) ;
      
      // check input - dU must be positive
      if(dU < GetLegalMinU())
        { 
          SE_MSG(SM_ERR,_T("SmBendVolume::EvaluateIsoParametricCurveSimple() bad U value, most be positive - no curve to return")) ; 
          
          // no curve to return - all done
          return(SM_SUCCESS) ;
        }
      
      // when dU or dV is not within pOptParamDomain
      if(   !pTgtDomain->GetUInterval().ContainsValue(dU, SM_EFF_ZERO) 
         || !pTgtDomain->GetVInterval().ContainsValue(dV, SM_EFF_ZERO)) 
        {
          // no curve to return - all done
          return(SM_SUCCESS) ;
        }

      // line properties 
      SmPoint3d sLinePoint = dU*smos_Cosine(dThetaRad)*sU + dU*smos_Sine(dThetaRad)*sV ;
      SmExtent1d sLineIvl(dWMin, dWMax) ;

      // cull degenerate curves
      if(sLineIvl.IsDegenerate())
        {
          // return degenerate curve
          SmCurve::CreateDegenerateCurve(crContext, 3, sLinePoint, rpNewIsoCurve) ; 
          return(SM_SUCCESS) ;
        }

      // create the line
      rpNewIsoCurve = new (crContext) SmLine(sLinePoint, sW, sLineIvl) ; 

    } // end varying w return a line parallel to the BendAxis branch

  else
    {
      SER_MSG(SM_ERR, _T("Bad Input eConstantParams value")) ; 
    }
      
  // all done
  return SM_SUCCESS ;

} // end SmBendVolume::EvaluateIsoParametricCurveSimple

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

    for SmBendVolume - Make a BSpline Surface           (exact)
***********************************************************************/
SmStatus SmBendVolume::EvaluateIsoParametricSurfaceSimple
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
  SmVector3d        sOrigin(0,0,0) ;
  SmVector3d        sU(1,0,0) ;
  SmVector3d        sV(0,1,0) ;
  SmVector3d        sW(0,0,1) ;
  SmExtent3d        sParamDomain = GetNaturalParamDomain() ;
  const SmExtent3d *pTgtDomain   = pOptParamDomain ? pOptParamDomain : &sParamDomain ;

  // constant u - return cylinder
  if(eConstantParam == SM_VP_U)
    {
      double dU = dIsoParam ; 
      double dThetaMinDeg = SM_RAD2DEG(pTgtDomain->GetVMin() / m_dBendNeutralDist) ;
      double dThetaMaxDeg = SM_RAD2DEG(pTgtDomain->GetVMax() / m_dBendNeutralDist) ;     
      double dMinW        = pTgtDomain->GetWMin() ;     
      double dMaxW        = pTgtDomain->GetWMax() ;     

      // limit Thetas to +/-180 degrees
      dThetaMinDeg = SM_CLAMP_TO_PERIOD(dThetaMinDeg, -180.0, 180.0) ; 
      dThetaMaxDeg = SM_CLAMP_TO_PERIOD(dThetaMaxDeg, -180.0, 180.0) ; 

      // check input - dU must be positive
      if(dIsoParam < GetLegalMinU())        
        {                                 
          SE_MSG(SM_ERR,_T("SmBendVolume::EvaluateIsoParametricSurfaceSimple() bad U value, most be positive - no surface to return")) ; 
          
          // no surface to return - all done
          return(SM_SUCCESS) ; 
        }

      // When dU is not contained within pTgtDomain
      if( !pTgtDomain->GetUInterval().ContainsValue(dU, SM_EFF_ZERO))
        {
          // no surface to return - all done
          return(SM_SUCCESS) ;
        }

      // cylinder Analytic domain: u: [-360 to 360] Max diff = 360
      //                           v: [minDist maxDist] from base point along z axis
      SmExtent2d sSTEPUVDomain(dThetaMinDeg, dMinW, dThetaMaxDeg, dMaxW) ;

      // cull degenerate surfaces
      if(sSTEPUVDomain.IsDegenerate())
        {
          // no surface to return - all done
          return(SM_SUCCESS) ;
        }

      // make infinite cylinder
      SmAxis2Placement sCoord(sOrigin, sU, sV) ; 
      SmCylinder::CreateCanonical(crContext, sCoord, dU, (SmCylinder *&)rpNewIsoSurface) ;

      // Trim Cylinder to given pOptParamDomain
      rpNewIsoSurface->AdjustSTEPUVDomain(sSTEPUVDomain) ;

    } // end constant u return cylinder branch

  // constant v - return plane radiating radially from the bend axis
  else if(eConstantParam == SM_VP_V)
    {
      double     dV        = dIsoParam ; 
      double     dThetaRad = dV / m_dBendNeutralDist ;
      SmVector3d sNormal   = - smos_Sine  (dThetaRad) * sU 
                             + smos_Cosine(dThetaRad) * sV ;

      // limit Theta to +/- SM_PI
      dThetaRad = smos_Max(dThetaRad, -SM_PI) ;
      dThetaRad = smos_Min(dThetaRad,  SM_PI) ;

      // set Plane X axis in the radial direction    (Bend varying U direction for constant v in ProjSpace)
      //           Y axis in the bend axis direction (Bend varying W direction in ProjSpace)
      SmVector3d sYAxis = sW ;
      SmVector3d sXAxis = sNormal * sYAxis ;

      // Set Plane domain 
      double dUMin = pTgtDomain->GetUMin() ;          
      double dUMax = pTgtDomain->GetUMax() ;
      double dWMin = pTgtDomain->GetWMin() ;
      double dWMax = pTgtDomain->GetWMax() ;

      // When dV is not contained within pOptParamDomain
      if( !pTgtDomain->GetVInterval().ContainsValue(dV, SM_EFF_ZERO))
        {
          // no surface to return - all done
          return(SM_SUCCESS) ;
        }

      // check input - dU must be positive
      if(dUMin < GetLegalMinU())
        { SE_MSG(SM_ERR,_T("SmBendVolume::EvaluateIsoParametricCurveSimple() bad min U value, most be positive - attempting to fix")) ; 
          dUMin = m_dSingularityTol ;
        }
      if(dUMax < GetLegalMinU())
        { 
          SE_MSG(SM_ERR,_T("SmBendVolume::EvaluateIsoParametricCurveSimple() bad max U interval, most be positive - no curve to return")) ; 
      
          // no surface to return - all done
          return(SM_SUCCESS) ;
        }

      // plane Analytic domain: u: Bend [UMin UMax]
      //                        v: Bend [WMin WMax]
      SmExtent2d sSTEPUVDomain(dUMin, dWMin, dUMax, dWMax) ;

      // cull degenerate surfaces
      if(sSTEPUVDomain.IsDegenerate())
        {
          // no surface to return - all done
          return(SM_SUCCESS) ;
        }

      // Build a trim plane
      SmVector2d sScale(1.0, 1.0) ; 
      rpNewIsoSurface = new (crContext) SmPlane(sOrigin, sXAxis, sYAxis, sScale, sSTEPUVDomain, &crContext) ;
      
      // when given an ParamDomain - Trim plane
    } // end constant v return plane radiating radially from BendAxis branch 

  // constant w - return disc
  else if(eConstantParam == SM_VP_W)
    {
      double dW           = dIsoParam ; 
      double dUMin        = pTgtDomain->GetUMin() ;
      double dUMax        = pTgtDomain->GetUMax() ;
      double dThetaMinDeg = SM_RAD2DEG(pTgtDomain->GetVMin() / m_dBendNeutralDist) ;
      double dThetaMaxDeg = SM_RAD2DEG(pTgtDomain->GetVMax() / m_dBendNeutralDist) ;     

      // limit Thetas to +/-180 degrees
      dThetaMinDeg = SM_CLAMP_TO_PERIOD(dThetaMinDeg, -180.0, 180.0) ; 
      dThetaMaxDeg = SM_CLAMP_TO_PERIOD(dThetaMaxDeg, -180.0, 180.0) ; 

      // When dW is not contained within pOptParamDomain
      if( !pOptParamDomain->GetWInterval().ContainsValue(dW, SM_EFF_ZERO))
        {
          // no surface to return - all done
          return(SM_SUCCESS) ;
        }

      // check input - dU must be positive
      if(dUMin < GetLegalMinU())
        { SE_MSG(SM_ERR,_T("SmBendVolume::EvaluateIsoParametricCurveSimple() bad min U value, most be positive - attempting to fix")) ; 
          dUMin = m_dSingularityTol ;
        }
      if(dUMax < GetLegalMinU())
        { 
          SE_MSG(SM_ERR,_T("SmBendVolume::EvaluateIsoParametricCurveSimple() bad max U interval, most be positive - no curve to return")) ; 
      
          // no surface to return - all done
          return(SM_SUCCESS) ;
        }

      // disc properties
      // disc Analytic domain: u: [-360 to 360] Max diff = 360
      //                       v: Bend [UMin UMax]            
      SmExtent2d sSTEPUVDomain(dThetaMinDeg, dUMin, dThetaMaxDeg, dUMax) ;
      SmPoint3d  sDiscCenter = dW * sW ;
      SmExtent1d sGenIvl(dUMin, dUMax) ;
      
      // cull degenerate surfaces
      if(sSTEPUVDomain.IsDegenerate())
        {
          // no surface to return - all done
          return(SM_SUCCESS) ; 
        } 

      // create the GenCurve line - radiates radially from the BendAxis in the BendMidDir direction
      SmLine *pGenCurve = new (crContext) SmLine(sDiscCenter , sU, sGenIvl) ;

      // create the disc
      SmSurfOfRevolution::CreateCanonical(crContext, pGenCurve, sOrigin, sW, (SmSurfOfRevolution *&)rpNewIsoSurface) ; 

      // Trim Cylinder to given pOptParamDomain
      rpNewIsoSurface->AdjustSTEPUVDomain(sSTEPUVDomain) ;

    } // end constant w return disc branch

  // default - bad eConstantParam value - inform the public
  else
    {
      SER_MSG(SM_ERR, _T("Bad Input eConstantParam value")) ; 
    }
      
  // all done
  return SM_SUCCESS;

} // end SmBendVolume::EvaluateIsoParametricSurfaceSimple

/*******************************************************************//**
PURPOSE: Do a quick approximate inverse mapping from ProjSpace back 
         to ParamSpace for upcoming newton raphson

NOTES: Transform is eaxact for half-space U > 0 - just do the inverse mapping - no guessing

RETURNS: SM_SUCCESS when ProjPoint is in positive half-space 
         SM_ERR     otherwise.
***********************************************************************/
SmStatus SmBendVolume::InvEvaluateGuessPointSimple
 (const SmPoint3d     & crProjPoint,        // in : ProjSpace Point to map back to ParamSpace Point
  SmTArray<SmPoint3d> & rGuessParamPoints)  // out: ParamSpace Point near Target Point actual map back to ParamSpace
 const
{
  // compute the inverse mapping
  double dR     = smos_Sqrt(crProjPoint.x*crProjPoint.x + crProjPoint.y*crProjPoint.y) ;

  // for points within the singularty set dTheta = 0.0 - otherwise dTheta = ATan(y,x)
  double dTheta = (dR < GetLegalMinU()) ? 0.0 : smos_ArcTangent2(crProjPoint.y, crProjPoint.x) ;      
  double dD     = m_dBendNeutralDist ;          

  // return value
  SmStatus eRtn = (dR < GetLegalMinU()) ? SM_ERR : SM_SUCCESS ;

  // init output - remember 2 solutions for seams
  SmBoolean bOnSeam = SM_IS_ZERO(dTheta - SM_PI) || SM_IS_ZERO(dTheta + SM_PI) ;
  
  // init output
  rGuessParamPoints.SetSize( bOnSeam ? 2 : 1 ) ;

  // set output  
  rGuessParamPoints[0].x = dR ;        
  rGuessParamPoints[0].y = dD * dTheta ;
  rGuessParamPoints[0].z = crProjPoint.z ;
  
  // when rGuessParamPoint is on the seam - add 2nd solution
  if(bOnSeam) { rGuessParamPoints[1].x = dR ;           
                rGuessParamPoints[1].y = dD * -dTheta ;  
                rGuessParamPoints[1].z = crProjPoint.z ;
              }

  // all done
  return( eRtn ) ;

} // end SmBendVolume::InvEvaluateGuessPointSimple

/*******************************************************************//**
PURPOSE: inverse map a point from ProjSpace back to ParamSpace

NOTES: Transform is invertible where its not degenerate - just do the inverse mapping - no guessing,
       Map XYZ ProjSpace point back to UVW ParamSpace.
***********************************************************************/
SmStatus SmBendVolume::GlobalPointSolveSimple // eff: Find volume UVW Point that maps to TargetPoint
 (const SmPoint3d  & crProjPoint,          // in : ProjSpace Point to map back to ParamSpace
  SmBoolean        & rbFoundAnswer,        // out: TRUE = successfully mapped ProjSpace Point to a ParamSpace Point
  SmSolutionArray  & rSolutions,           // out: Contains ParamSpace Found Point
                                           //      rSolution[i].m_eSolutionType           = SM_ST_SINGLE_VALUE                                               
                                           //      rSolution[i].m_vStart.m_dSolutionValue = PointOnVolume.DistanceBetween(crProjPoint);                      
                                           //      rSolution[i].m_vStart[0] =  U of [U,V,W] the found ParamSpace point location                    
                                           //      rSolution[i].m_vStart[1] =  V of [U,V,W] the found ParamSpace point location                    
                                           //      rSolution[i].m_vStart[2] =  W of [U,V,W] the found ParamSpace point location                    
  const SmExtent3d * pOptParamDomain)      // NotUsed: in : ParamSpace domain over which to search for the inverse point
 const                                     //      NULL = use Map's NaturalDomain, default:[NULL]
{
  SM_REF1(pOptParamDomain) ;
  // inverse mapping
  double dR     = smos_Sqrt(crProjPoint.x*crProjPoint.x + crProjPoint.y*crProjPoint.y) ;

  // for points within the singularty set dTheta = 0.0 - otherwise dTheta = ATan(y,x)
  double dTheta = (dR < GetLegalMinU()) ? 0.0 : smos_ArcTangent2(crProjPoint.y, crProjPoint.x) ;      
  double dD     = m_dBendNeutralDist ;          
  double dGap   = 0.0 ;

  // init output - remember 2 solutions for seams
  SmBoolean bOnSeam = SM_IS_ZERO(dTheta - SM_PI) || SM_IS_ZERO(dTheta + SM_PI) ;
  
  // inverted ParamPoint
  SmPoint3d sParamPoint(dR, dD * dTheta, crProjPoint.z) ;    

  // init output
  rSolutions.SetSize( bOnSeam ? 2 : 1 ) ;

  // set output
  rbFoundAnswer                            = TRUE ;
  rSolutions[0].m_eSolutionType            = SM_ST_SINGLE_VALUE ;
  rSolutions[0].m_lNumVariables            = 3 ;
  rSolutions[0].m_vStart.m_dSolutionValue  = dGap ;
  rSolutions[0].m_vStart.m_adParameters[0] = sParamPoint.x ;
  rSolutions[0].m_vStart.m_adParameters[1] = sParamPoint.y ;
  rSolutions[0].m_vStart.m_adParameters[2] = sParamPoint.z ;
  rSolutions[0].m_lNumObjects              = 1 ;
  rSolutions[0].m_apObjects[0]             = (SmObject *)this ;

  // when on the seam - set the 2nd solution
  if(bOnSeam)
    {
      rSolutions[1].m_eSolutionType            =  SM_ST_SINGLE_VALUE ;
      rSolutions[1].m_lNumVariables            =  3 ;
      rSolutions[1].m_vStart.m_dSolutionValue  =  dGap ;
      rSolutions[1].m_vStart.m_adParameters[0] =  sParamPoint.x ;
      rSolutions[1].m_vStart.m_adParameters[1] = -sParamPoint.y ;
      rSolutions[1].m_vStart.m_adParameters[2] =  sParamPoint.z ;
      rSolutions[1].m_lNumObjects              =  1 ;
      rSolutions[1].m_apObjects[0]             = (SmObject *)this ;
    }
  
  // all done
  return(SM_SUCCESS) ;  

} // end SmBendVolume::GlobalPointSolveSimple

/*******************************************************************//**
PURPOSE: inverse map a point from ProjSpace back to ParmSpace with a guess point

NOTES: Transform is invertible except at degeneracy - just do the inverse mapping - no guessing,
       Map XYZ OutSpace point back to UVW InSpace.
***********************************************************************/
SmStatus SmBendVolume::LocalPointSolveSimple
 (const SmPoint3d  & crProjPoint,          // in : ProjSpace Point to map back to ParamSpace
  const SmPoint3d  & crParamPointGuess,    // in : ParamSpace point guess, the closer to the actual ParamSpace point the better
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
  // locals
  SmSolutionArray sSolutions ;

  // pass the call along - no need for the guess point
  SER(GlobalPointSolveSimple(crProjPoint, rbFoundAnswer, sSolutions, pOptParamDomain)) ;

  if(sSolutions.GetSize() == 1)
    {
      rSolution = sSolutions[0] ;
    }
  else // on a seam - return solution closest to ParamGuessPoint
    {
      SmPoint3d sParamPoint0(sSolutions[0].m_vStart.m_adParameters[0],
                             sSolutions[0].m_vStart.m_adParameters[1],
                             sSolutions[0].m_vStart.m_adParameters[2]) ;
                              
      SmPoint3d sParamPoint1(sSolutions[1].m_vStart.m_adParameters[0],
                             sSolutions[1].m_vStart.m_adParameters[1],
                             sSolutions[1].m_vStart.m_adParameters[2]) ;

      double dDistSq0 = sParamPoint0.DistanceBetweenSquared(crParamPointGuess) ;
      double dDistSq1 = sParamPoint1.DistanceBetweenSquared(crParamPointGuess) ;

      rSolution = (dDistSq0 < dDistSq1) ? sSolutions[0] : sSolutions[1] ;
    }

  // all done
  return(SM_SUCCESS) ;

} // end SmBendVolume::LocalPointSolveSimple

/*******************************************************************//**
PURPOSE: return TRUE if any part of the natural param domain is bounded

NOTES: Bend is defined on the Positive halfspace from the BendOrigin to
       the BendMidDir direction. In BendCentered ABC coordinates
       that's the halfspace defined by a > 0.
***********************************************************************/
SmBoolean SmBendVolume::IsBoundedSimple() const           
{
  // all done
  return(TRUE) ;

} // end SmBendVolume::IsBoundedSimple    

/*******************************************************************//**
PURPOSE:  

NOTES: BendVolumes are periodic over v = [-d*Pi, +d*Pi ]
       d = m_vBendNeturalDistance
***********************************************************************/
SmBoolean SmBendVolume::IsClosedSimple
 (SmBoolean        & rbClosedU,          // out: TRUE = closed in U direction, [check Pos[Umin,v,w] == Pos[Umax,v,w] for v,w samples
  SmBoolean        & rbClosedV,          // out: TRUE = closed in V direction, [check Pos[u,Vmin,w] == Pos[u,Vmax,w] for w,u samples
  SmBoolean        & rbClosedW,          // out: TRUE = closed in W direction, [check Pos[u,v,Wmin] == Pos[u,v,Wmax] for u,v samples
  double           * pdOptTolerance,     // out: pdOptTolerance = Tolerance to allow for check
  const SmExtent3d * pOptParamDomain,    // in : ParamSpace domain over which to search for the inverse point
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
  rbClosedU = FALSE ;
  rbClosedV = TRUE ;
  rbClosedW = FALSE ;

  if(peOptContinuityU) *peOptContinuityU = SM_CT_DISCONTINUOUS ;
  if(peOptContinuityV) *peOptContinuityV = SM_CT_CINFINITY ;
  if(peOptContinuityW) *peOptContinuityW = SM_CT_DISCONTINUOUS ;

  if(pdOptTolerance)   *pdOptTolerance    = SM_UNDEF_DOUBLE ;
  if(pOptParamDomain)  ((SmExtent3d*)pOptParamDomain)->Init() ;  

  // all done
  return(FALSE) ;

} // end SmBendVolume::IsClosedSimple    

/*******************************************************************//**
PURPOSE: 

NOTES:  BendVolumes are periodic over v = [-d*Pi, +d*Pi ]
       d = m_vBendNeturalDistance
***********************************************************************/
SmBoolean SmBendVolume::IsPeriodicSimple
 (SmBoolean        & rbPeriodicU,      // out: TRUE = G1 or better in U Dir 
  SmBoolean        & rbPeriodicV,      // out: TRUE = G1 or better in V Dir
  SmBoolean        & rbPeriodicW,      // out: TRUE = G1 or better in W Dir
  const SmExtent3d * pOptParamDomain)  // NotUsed: in : ParamSpace domain over which to search for the inverse point
 const                                 //      NULL = use Map's NaturalDomain, default:[NULL]
{
  SM_REF1(pOptParamDomain) ;
  rbPeriodicU = FALSE ;
  rbPeriodicV = TRUE ;
  rbPeriodicW = FALSE ;

  // all done
  return(FALSE) ;

} // end SmBendVolume::IsPeriodicSimple 

/*******************************************************************//**
PURPOSE: 

NOTES: degenerate mapping for a = 0 
        where a = distance from BendCenter in BendMidDir direction
***********************************************************************/   
SmBoolean SmBendVolume::IsSingularitySimple      
  (const SmPoint3d   & crParamPoint,       // in : UVpoint to test                 
   SmBoolean         & rbSingularU,        // out: TRUE = [Su=0]
   SmBoolean         & rbSingularV,        // out: TRUE = [Sv=0]
   SmBoolean         & rbSingularW,        // out: TRUE = [Sw=0]
   double              d3dTol)             // NotUsed: in : min 3d distance between distinct points) const; 
  const 
{
  SM_REF1(d3dTol) ;
  // locals
  SmBoolean bRtn = FALSE ; 

  // see if point is on the a=0 plane
  double dU = crParamPoint.x ;

  // branch on dist from a=0 plane
  if(dU < GetLegalMinU()) { rbSingularU = TRUE ;
                            rbSingularV = TRUE ;
                            rbSingularW = FALSE ;
                            bRtn        = TRUE ;
                          }
  else                    { rbSingularU = FALSE ;
                            rbSingularV = FALSE ;
                            rbSingularW = FALSE ;
                            bRtn        = FALSE ;
                          }
  // all done
  return(bRtn) ;

} // end SmBendVolume::IsSingularitySimple    

/*******************************************************************//**
PURPOSE: 

NOTES: This may have to be rethought.  Currently Bend mappings are half spaces.
       But they have a degeneracy along the a = 0 plane and are periodic
       in v over the block = [-d*Pi, +d*Pi].
***********************************************************************/
SmBoolean SmBendVolume::IsOnBoundarySimple
  (const SmPoint3d    & crParamPoint,    // in : Volume UVWPoint to test
   SmBoolean          & rbOnU,           // out: TRUE = TargetPoint on UMin or UMax
   SmBoolean          & rbOnV,           // out: TRUE = TargetPoint on VMin or VMax
   SmBoolean          & rbOnW,           // out: TRUE = TargetPoint on WMin or WMax
   double             * pdOptTolerance,  // NotUsed: in : max deviation allowed for point on seam
                                         //      NULL = use SM_EFF_ZERO * 1000 * (1 + maxDimension())
  const SmExtent3d    * pOptParamDomain) // NotUsed: in : ParamSpace domain over which to search for the inverse point
 const                                   //      NULL = use Map's NaturalDomain, default:[NULL]
{
  SM_REF2(pdOptTolerance, pOptParamDomain) ;
  // project UVW point into Bend Coordinates
  double dU = crParamPoint.x ;       
  double dV = crParamPoint.y ;

  rbOnU = smos_Fabs(dU) < GetLegalMinU() ;
  rbOnV = SM_IS_ZERO(smos_Fabs(dV) - SM_PI * m_dBendNeutralDist) ;
  rbOnW = FALSE ;

  // all done
  return(rbOnU || rbOnV || rbOnW) ;

} // end SmBendVolume::IsOnBoundarySimple    

/*******************************************************************//**
PURPOSE: return TRUE when crParamPoint can be uniquely mapped to ProjSpace
         from the PrimaryPeriod

NOTES: The SmBendVolume's Natural Param Space includes the portion of the
       positive half-space, u > 0, that is within the mapping primary's 
       period.
***********************************************************************/
SmBoolean SmBendVolume::IsPointInParamDomainSimple
 (const SmPoint3d  & crParamPoint)          // in : ParamSpace point to test for inclusion                                  
      const                                
{
  // Only ParamPoints in the positive half-space, u>0, and in the PrimaryPeriod
  // are within the SmBendVolume's Natural ParamSpace domain.
  double    dThetaRad        = crParamPoint.y / m_dBendNeutralDist ;
  SmBoolean bInPrimaryPeriod = SM_IS_CONTAINED(dThetaRad, -SM_PI, SM_PI) ;
  SmBoolean bInHalfSpace     = crParamPoint.x > GetLegalMinU() ;

  // all done
  return(bInPrimaryPeriod && bInHalfSpace) ;

} // end SmBendVolume::IsPointInParamDomainSimple

/*******************************************************************//**
PURPOSE: return TRUE when crProjPoint can be uniquely mapped back to ParamSpace

NOTES: The SmBendVolume can only invert points within the positive half-space
       u > 0.
***********************************************************************/
SmBoolean SmBendVolume::IsPointInProjDomainSimple
 (const SmPoint3d  & crProjPoint,          // in : project space point to test for inversion                                  
  SmPoint3d        * pOptParamPoint)       // out: if the point is invertible, go ahead and get the inverse
      const                                
{ 
  // return the inverse mapping
  double dR = smos_Sqrt(crProjPoint.x*crProjPoint.x + crProjPoint.y*crProjPoint.y) ;

  // return value
  SmBoolean bInvertible = (dR < GetLegalMinU()) ? FALSE : TRUE ;

  // when asked - do the inversion
  if(pOptParamPoint)
    {
      // for points within the singularty set dTheta = 0.0 - otherwise dTheta = ATan(y,x)
      double dTheta = (dR < GetLegalMinU()) ? 0.0 : smos_ArcTangent2(crProjPoint.y, crProjPoint.x) ;      
      double dD     = m_dBendNeutralDist ;          

      pOptParamPoint->x = dR ;        
      pOptParamPoint->y = dD * dTheta ;
      pOptParamPoint->z = crProjPoint.z ;    
    }

  // all done
  return( bInvertible ) ;

} // end SmBendVolume::IsPointInProjDomainSimple

/*******************************************************************//**
PURPOSE: return TRUE when ParamLine maps completely within the
       Natural Domain and does not cross any discontinuities.

NOTES: The SmBendVolume's Natural Param Space includes the portion of the
       positive half-space, u > 0, that is within the mapping primary's 
       period.

       BendVolume ParamDomain is convex and contains no internal dicontinutities
       so the Line is 'in' when both EndPoints are 'in'
***********************************************************************/
SmBoolean SmBendVolume::IsLineInParamDomainSimple
 (const SmPoint3d  & crStartParamPoint,       // in : ParamSpace Line StartPoint to test for inclusion                                  
  const SmPoint3d  & crEndParamPoint)         // in : ParamSpace Line EndPoint to test for inclusion
 const                                
{
  // ParamLine is in when both EndParamPoints are in
  // ParamPoints are in when in the positive half-space, u>0, and in the theta PrimaryPeriod:[-SM_PI, SM_PI]
  SmBoolean bRtn  = IsPointInParamDomainSimple(crStartParamPoint) ;
  bRtn           &= IsPointInParamDomainSimple(crEndParamPoint) ;

  // all done
  return(bRtn) ;

} // end SmBendVolume::IsLineInParamDomainSimple

/*******************************************************************//**
PURPOSE: Return TRUE when Curve does not cross any volume's boundaries or
         internal C1 discontinuities and is contained within the ParamSpace domain

NOTES: The BendVolume mapping is limited to the half space U > m_dSingularityTol
       and to one period (2Pi * m_dSingularityTol) in the V direction. That
       period can be centered on any V = const plane.  For now,
       this function assumes the period of interest is centered on the V = 0 plane.

ASSUMPTIONS: assumes the V direction period is centered on V=0 plane when
       in reality it can be centered on any V=const plane.
***********************************************************************/
SmBoolean SmBendVolume::IsCurveInParamDomainSimple
 (const SmCurve & crParamCurve)       // in : Tgt ParamSpace Curve to check
 const       
{ 
  // locals
  SM_OBJ_ARRAY(sPlanePoints, SmPoint3d, 3) ;   sPlanePoints.SetSize(3) ;
  SM_OBJ_ARRAY(sPlaneNormals, SmVector3d, 3) ; sPlaneNormals.SetSize(3) ; 

  // load the 3 domain bounding planes with normals pointing into the allowed 1/2 space
  sPlanePoints[0].Set(m_dSingularityTol, 0, 0) ;           
  sPlanePoints[1].Set(0,  SM_PI * m_dBendNeutralDist, 0) ; // note: any 2Pi*d interval is legal - this one is centered on V=0  
  sPlanePoints[2].Set(0, -SM_PI * m_dBendNeutralDist, 0) ; // note: any 2Pi*d interval is legal - this one is centered on V=0 

  sPlaneNormals[0].Set(1,  0, 0) ; 
  sPlaneNormals[1].Set(0, -1, 0) ;
  sPlaneNormals[2].Set(0,  1, 0) ; 

  // is curve coincident, touching, or on positive side of all bounding planes
  SmBoolean bRtn = crParamCurve.IsOnPositiveSideOfPlanes(sPlanePoints, sPlaneNormals) ;

  // all done
  return(bRtn) ; 

 } // end SmBendVolume::IsCurveInParamDomainSimple

/*******************************************************************//**
PURPOSE: Return TRUE when Surface does not cross any volume's boundaries or
         internal C1 discontinuities and is contained within the ParamSpace domain

NOTES: The BendVolume mapping is limited to the half space U > m_dSingularityTol
       and to one period (2Pi * m_dSingularityTol) in the V direction. That
       period can be centered on any V = const plane.  For now,
       this function assumes the period of interest is centered on the V = 0 plane.

ASSUMPTIONS: assumes the V direction period is centered on V=0 plane when
       in reality it can be centered on any V=const plane.
***********************************************************************/
SmBoolean SmBendVolume::IsSurfaceInParamDomainSimple
 (const SmSurface & crParamSurface)   // in : Tgt ParamSpace Surface to check
 const 
{ 
  // locals
  ULONG ii ;
  SM_PTR_ARRAY(sPlanes,       SmPlane,    3) ; 
  SM_OBJ_ARRAY(sPlanePoints,  SmPoint3d,  3) ; sPlanePoints.SetSize(3) ;
  SM_OBJ_ARRAY(sPlaneUs,      SmVector3d, 3) ; sPlaneUs.SetSize(3) ; 
  SM_OBJ_ARRAY(sPlaneVs,      SmVector3d, 3) ; sPlaneVs.SetSize(3) ; 
  SM_OBJ_ARRAY(sPlaneDomains, SmExtent2d, 3) ; sPlaneDomains.SetSize(3) ; 

  SmContext  sContext ;
  SmVector2d sUVScale(1,1) ;
  SmVector3d sU(1,0,0), sV(0,1,0), sW(0,0,1) ;

  // set 1/2 plane valid intervals - note: infinite 1/2 planes are being approximated by +/- SM_BIG_DOUBLE
  SmExtent1d sUIvl( m_dSingularityTol,          SM_BIG_DOUBLE) ;
  SmExtent1d sVIvl(-SM_PI * m_dBendNeutralDist, SM_PI * m_dBendNeutralDist) ; // note: any 2Pi*d interval is legal - this one is centered on V=0
  SmExtent1d sWIvl(-SM_BIG_DOUBLE,              SM_BIG_DOUBLE) ;

  // load the 3 domain bounding planes with normals pointing into the allowed 1/2 space
  sPlanePoints[0].Set(m_dSingularityTol, 0, 0) ;           
  sPlanePoints[1].Set(0,  SM_PI * m_dBendNeutralDist, 0) ; // note: any 2Pi*d interval is legal - this one is centered on V=0  
  sPlanePoints[2].Set(0, -SM_PI * m_dBendNeutralDist, 0) ; // note: any 2Pi*d interval is legal - this one is centered on V=0 

  sPlaneUs[0] = sV ;  sPlaneVs[0] = sW ;  // so that: sPlaneNormals[0] = (1,  0, 0) ; 
  sPlaneUs[1] = sU ;  sPlaneVs[1] = sW ;  // so that: sPlaneNormals[1] = (0, -1, 0) ;
  sPlaneUs[2] = sW ;  sPlaneVs[2] = sU ;  // so that: sPlaneNormals[2] = (0,  1, 0) ; 

  sPlaneDomains[0].SetUInterval(sVIvl) ;  sPlaneDomains[0].SetVInterval(sWIvl) ;
  sPlaneDomains[1].SetUInterval(sUIvl) ;  sPlaneDomains[1].SetVInterval(sWIvl) ;
  sPlaneDomains[2].SetUInterval(sWIvl) ;  sPlaneDomains[2].SetVInterval(sUIvl) ; 

  // make and store 1/2 space bounding Trim planes
  for(ii=0;ii<3;ii++)
    {
      SmPlane *pPlane = new (sContext) SmPlane(sPlanePoints[ii],
                                               sPlaneUs[ii],
                                               sPlaneVs[ii],
                                               sUVScale,
                                               sPlaneDomains[ii],
                                               &sContext) ; 

      sPlanes.Add(pPlane) ;
    }

  // Make pPlanes memory temporary
  SmObjsDelete<SmPlane *> sClean(&sPlanes) ; 

  // arrive here when sPlanes, are built.

  // return value - assume FALSE - update by classifying surface against discontinuity planes
  SmBoolean bRtn = FALSE ; 

  // check for surface crossing discontinuity planes check
  SmBoolean bHasPlaneCrossings = crParamSurface.HasPlaneCrossings(sPlanes) ;

  // when surface is on the inside or the outside of the Volume ParamDomain
  if(bHasPlaneCrossings == FALSE)
    {
      // classify one SurfacePoint 
  
      // Surface MidPoint loc
      SmPoint2d sMidUVPoint    = crParamSurface.GetNaturalUVDomain().Evaluate(.5,.5) ;
      SmPoint3d sMidParamPoint ;
      crParamSurface.EvaluatePoint(sMidUVPoint, sMidParamPoint) ;

      // classify MidPoint against Volume ParamDomain
      bRtn =    sUIvl.ContainsValue(sMidParamPoint.x, SM_EFF_ZERO)
             && sVIvl.ContainsValue(sMidParamPoint.y, SM_EFF_ZERO)
             && sWIvl.ContainsValue(sMidParamPoint.z, SM_EFF_ZERO) ;
    }

  // all done - return final classification
  return bRtn ;

} // end SmBendVolume::IsSurfaceInParamDomainSimple

/*******************************************************************//**
PURPOSE: Write SmTransform to given output stream.

NOTES: 
***********************************************************************/
SmStatus SmBendVolume::WriteToDB
 (SmDatabaseIO & rDB,              // in : target output stream
  ULONG          lDBVersionNumber) // in : database version to get proper sequence of writes                                                                       
 const
{
  // file type, ASCII or BINARY
  SmFileType eType = rDB.GetFileType();

  // Volume locals
  //   stored in SmVolume::OrientMap    double *pC = (double *)&GetBendOrigin() ; 
  //                                    double *pA = (double *)&GetBendAxis() ; 
  //                                    double *pS = (double *)&GetBendMidDir() ; 
  //                                    double *pB = (double *)&GetBendBinormal() ;

  // Create the output file 
  if (eType == SM_ASCII) { std::ostream & rFileOut = *rDB.GetOutStreamPtr() ;

                           //      rFileOut << pC[0] << " " << pC[1] << " " << pC[2] << " " << "BendVolume Bend Center \n" ;
                           //      rFileOut << pA[0] << " " << pA[1] << " " << pA[2] << " " << "BendVolume Bend Axis \n" ;
                           //      rFileOut << pS[0] << " " << pS[1] << " " << pS[2] << " " << "BendVolume Bend StartDir \n" ;
                           //      rFileOut << pB[0] << " " << pB[1] << " " << pB[2] << " " << "BendVolume Bend Binormal \n" ;
                           rFileOut << m_dBendNeutralDist  << "BendVolume Bend NeutralDistance \n" ;
                           rFileOut << m_dSingularityTol   << "BendVolume SingularityTol: min legal distance from singularity at U = 0\n" ;
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

                           SER(rDB.WriteDouble(m_dBendNeutralDist)) ;
                           SER(rDB.WriteDouble(m_dSingularityTol )) ;
                         }

  // Write base class data
  SmVolume::WriteToDB(rDB, lDBVersionNumber) ;

  // all done
  return SM_SUCCESS;

} // end SmBendVolume::WriteToDB

/*******************************************************************//**
PURPOSE: static method to Read a SmTransform from a given stream  

NOTES: 
***********************************************************************/
SmStatus SmBendVolume::ReadFromDB
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
         || rpNewVolume->IsKindOf(SmBendVolume_TYPE)) ? SM_SUCCESS : SM_ERR) ;

  // init output object
  SmBendVolume *pBendVolume =   (rpNewVolume == NULL)
                              ? new (crContext) SmBendVolume()
                              : (SmBendVolume *)rpNewVolume ;

  // file type
  SmFileType eType = rDB.GetFileType();
  

  // Volume locals
  //      double *pC = (double *)&pBendVolume->GetBendOrigin() ; 
  //      double *pA = (double *)&pBendVolume->GetBendAxis() ; 
  //      double *pS = (double *)&pBendVolume->GetBendMidDir() ; 
  //      double *pB = (double *)&pBendVolume->GetBendBinormal() ;
  double *pBendNeutralDist = &pBendVolume->m_dBendNeutralDist ;
  double *pSingularityTol  = &pBendVolume->m_dSingularityTol ; 

  if (eType == SM_ASCII) 
    {
      std::istream & rFileIn = *rDB.GetInStreamPtr();
      //      rFileIn >> pC[0] >> pC[1] >> pC[2] ;  rDB.GoToNextLine() ;
      //      rFileIn >> pA[0] >> pA[1] >> pA[2] ;  rDB.GoToNextLine() ;
      //      rFileIn >> pS[0] >> pS[1] >> pS[2] ;  rDB.GoToNextLine() ;
      //      rFileIn >> pB[0] >> pB[1] >> pB[2] ;  rDB.GoToNextLine() ;
      rFileIn >> pBendNeutralDist[0] ;  rDB.GoToNextLine() ;
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

      SER(rDB.ReadDouble(pBendNeutralDist[0]));
      SER(rDB.ReadDouble(pSingularityTol[0]));
    }

  // set output
  rpNewVolume = pBendVolume ;

  // read base class data
  SmVolume::ReadFromDB(SmVolume_TYPE, rDB, crContext, rpNewVolume, lDBVersionNumber ) ; 

  // all done
  return SM_SUCCESS;

} // end SmBendVolume::ReadFromDB

/*******************************************************************//**
PURPOSE: Compute the total size of the memory used by the SmBendVolume.

NOTES: includes attribute memory
***********************************************************************/
ULONG SmBendVolume::GetMemoryUsed    // rtn: smaller size of actually used memory in bytes
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

} // end SmBendVolume::GetMemoryUsed

/*******************************************************************//**
PURPOSE: AssertValid() Test reporting static labels
***********************************************************************/
SmAssertReportLabel sAssertBendVolume_list[] =
{
  /*  0 */ {SM_AT_UNIT_VECTOR, _T("unit-vector"),        _T("BendAxis has to be unit lenghth") },
  /*  1 */ {SM_AT_UNIT_VECTOR, _T("unit-vector"),        _T("BendMidDir has to be unit lenghth") },
  /*  2 */ {SM_AT_UNIT_VECTOR, _T("unit-vector"),        _T("BendBinormal has to be unit lenghth") },
  /*  3 */ {SM_AT_GEOMETRIC,   _T("orthogonal"),         _T("Bend Axis/StartDir/Binormal must be mutually orthoganal") },
  /*  4 */ {SM_AT_GEOMETRIC,   _T("orthogonal"),         _T("Bend Axis/StartDir/Binormal must be right handed") },
  /*  5 */ {SM_AT_GEOMETRIC,   _T("Positive-Distance"),  _T("Bend NeutralDistance must be PositiveDefinite") },
  /*  6 */ {SM_AT_GEOMETRIC,   _T("Positive-Tolerance"), _T("Bend SingularityTol must be PositiveDefinite") }
} ;

/*******************************************************************//**
PURPOSE: Make sure m_pNurb is not NULL by calling MakeNurb() when needed.

NOTES: returns TRUE  = OK
                        FALSE = Problem
***********************************************************************/
SmBoolean SmBendVolume::AssertValid
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

  // Run SmBendVolume checks

  // BendAxis is a unit vector
  bRtn &= SM_ASSERT_VALUE_REPORT(0, SM_LEVEL_0, (smos_Fabs(GetBendAxis().Dot(GetBendAxis())-1.0) < SM_EFF_ZERO), SM_EFF_ZERO, smos_Fabs(GetBendAxis().Dot(GetBendAxis())-1.0), _T("")) ;

  // BendMidDir is a unit vector
  bRtn &= SM_ASSERT_VALUE_REPORT(1, SM_LEVEL_0, (smos_Fabs(GetBendMidDir().Dot(GetBendMidDir())-1.0) < SM_EFF_ZERO), SM_EFF_ZERO, smos_Fabs(GetBendMidDir().Dot(GetBendMidDir())-1.0), _T("")) ;

  // BendBinormal is a unit vector
  bRtn &= SM_ASSERT_VALUE_REPORT(2, SM_LEVEL_0, (smos_Fabs(GetBendBinormal().Dot(GetBendBinormal())-1.0) < SM_EFF_ZERO), SM_EFF_ZERO, smos_Fabs(GetBendBinormal().Dot(GetBendBinormal())-1.0), _T("")) ;

  // basis triple product - should be +1
  double dTripleProduct = GetBendMidDir().TripleProduct(GetBendBinormal(), GetBendAxis()) ; 

  // basis vectors have to be mutually orthogonal
  bRtn &= SM_ASSERT_VALUE_REPORT(3, SM_LEVEL_0, (smos_Fabs(dTripleProduct)-1.0) < SM_EFF_ZERO, SM_EFF_ZERO, (smos_Fabs(dTripleProduct)-1.0), _T("")) ;

  // basis vectors have to be right handed
  bRtn &= SM_ASSERT_VALUE_REPORT(4, SM_LEVEL_0, (dTripleProduct-1.0) < SM_EFF_ZERO, SM_EFF_ZERO, (dTripleProduct-1.0), _T("")) ;

  // BendNeutralDist must be positive definite
  bRtn &= SM_ASSERT_VALUE_REPORT(5, SM_LEVEL_0 , m_dBendNeutralDist >= m_dSingularityTol, SM_EFF_ZERO, m_dBendNeutralDist, _T("")) ;

  // Bend SingularityTol must be positive definite
  bRtn &= SM_ASSERT_VALUE_REPORT(6, SM_LEVEL_0 , m_dSingularityTol >= SM_ZONE_TOL_3D, SM_EFF_ZERO, m_dSingularityTol, _T("")) ;

  // all done 
  return(bRtn) ;

} // end SmBendVolume::AssertValid

// obsolete
// /*******************************************************************//**
// PURPOSE: Fix Assert Report failures reported by an AssertValid call
// 
// RETURNS ---  TRUE  = OK
//              FALSE = Problem
// ***********************************************************************/
// SmBoolean SmBendVolume::AssertHeal
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
//                rAReport.m_pHealMessage = _T("SmBendVolume::AssertHeal fix not yet supported") ;  
//                 
//     }
// 
//   // all done
//   return(bRtn) ;
// 
// } // end SmBendVolume::AssertHeal
// end obsolete

/*******************************************************************//**
PURPOSE: Draw Axis - origin(black), x(red), y(green), z(blue)
         Draw Neutral Plane graphics
         Draw Bend Geometry
NOTES:
***********************************************************************/
SmDisplayList *SmBendVolume::Draw
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

  // 1. limit TgtDomain->IntervalV so that NeutralPlane is at most a closed circle
  // 2. for a better display, set UMin = 1/2 NeutralDistance
  // 3. for a better display, set WIvl proportional to VIvl
  double dMaxV =  m_dBendNeutralDist * SM_PI * 3/4 ;
  double dMinV = -m_dBendNeutralDist * SM_PI * 3/4 ;
  dMaxV = smos_Min(pTgtDomain->GetVMax(), dMaxV) ;    
  dMinV = smos_Max(pTgtDomain->GetVMin(), dMinV) ;

  double dMaxU =  3.0 * m_dBendNeutralDist / 2.0 ;
  double dMinU =  m_dBendNeutralDist / 2.0 ;
  dMaxU = smos_Max(m_dSingularityTol, dMaxU) ;
  dMinU = smos_Max(m_dSingularityTol, dMinU) ;

  double dMaxW =  1.5 * dMaxV ;
  double dMinW = -1.5 * dMaxV ;
  dMaxW = smos_Min(pTgtDomain->GetWMax(), dMaxW) ;    
  dMinW = smos_Max(pTgtDomain->GetWMin(), dMinW) ;

  // specify the tgt draw domain
  pTgtDomain->SetMinMax(dMinU, dMinV, dMinW,
                        dMaxU, dMaxV, dMaxW) ;

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
      //      EvaluateIsoParametricCurve(*cpContext, SM_VPS_VW, .25*pTgtDomain->GetVMin() + .75*pTgtDomain->GetVMax(), pTgtDomain->GetWMin(), 0.0, NULL, &pIsoCurve5 , pTgtDomain) ; 
      //      EvaluateIsoParametricCurve(*cpContext, SM_VPS_VW, .50*pTgtDomain->GetVMin() + .50*pTgtDomain->GetVMax(), pTgtDomain->GetWMin(), 0.0, NULL, &pIsoCurve6 , pTgtDomain) ; 
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
      EvaluateIsoParametricSurface(*cpContext, SM_VP_U, m_dBendNeutralDist,       0.0, NULL, &pIsoSurface2, pTgtDomain) ; 
      //      EvaluateIsoParametricSurface(*cpContext, SM_VP_U, 1.5 * m_dBendNeutralDist, 0.0, NULL, &pIsoSurface3, pTgtDomain) ; 
      EvaluateIsoParametricSurface(*cpContext, SM_VP_U, pTgtDomain->GetUMax(),    0.0, NULL, &pIsoSurface4, pTgtDomain) ; 
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
      //      EvaluateIsoParametricCurve(*cpContext, SM_VPS_VW, .25*pTgtDomain->GetVMin() + .75*pTgtDomain->GetVMax(), pTgtDomain->GetWMin(), 0.0, &pIsoCurve5 , NULL, pTgtDomain) ; 
      //      EvaluateIsoParametricCurve(*cpContext, SM_VPS_VW, .50*pTgtDomain->GetVMin() + .50*pTgtDomain->GetVMax(), pTgtDomain->GetWMin(), 0.0, &pIsoCurve6 , NULL, pTgtDomain) ; 
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
      EvaluateIsoParametricSurface(*cpContext, SM_VP_U, m_dBendNeutralDist,       0.0, &pIsoSurface2, NULL, pTgtDomain) ; 
      //      EvaluateIsoParametricSurface(*cpContext, SM_VP_U, 1.5 * m_dBendNeutralDist, 0.0, &pIsoSurface3, NULL, pTgtDomain) ; 
      EvaluateIsoParametricSurface(*cpContext, SM_VP_U, pTgtDomain->GetUMax(),    0.0, &pIsoSurface4, NULL, pTgtDomain) ; 
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

    } // end bShowOutSpace check

  // Draw bend coordinate system - U vector scaled to NeutralDistance
  SmVector3d sBendOrigin = GetBendOrigin();
  smgfx_SetColor(0,0,0) ; sBendOrigin.Draw(NULL, NULL, pOptGfxSet) ;
  smgfx_SetColor(1,0,0) ; (m_dBendNeutralDist * GetBendMidDir()).Draw(&sBendOrigin, NULL, pOptGfxSet) ;
  smgfx_SetColor(0,1,0) ; GetBendBinormal().Draw(&sBendOrigin, NULL, pOptGfxSet) ;
  smgfx_SetColor(0,0,1) ; GetBendAxis().Draw(&sBendOrigin, NULL, pOptGfxSet) ;
  smgfx_SetColor(sColor, pOptGfxSet) ;
  
  // Draw Bend axis over W extent
  SmExtent1d sIvlW      = pTgtDomain->GetWInterval() ;
  SmPoint3d  sStartAxis = GetBendOrigin() + sIvlW.GetMin() * GetBendAxis() ;
  SmPoint3d  sEndAxis   = GetBendOrigin() + sIvlW.GetMax() * GetBendAxis() ;
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

} // end SmBendVolume::Draw

/*******************************************************************//**
PURPOSE:
NOTES:
***********************************************************************/
SmBoolean SmBendVolume::IsKindOf( SM_TYPE t ) const
{
  return ((SmBendVolume_TYPE == t) ? TRUE : SmVolume::IsKindOf( (t) ));
}

/*******************************************************************//**
PURPOSE: Dump bspline volume data out for debugging.
            Dump is in similar sequence to Write to file

NOTES: 
***********************************************************************/
void SmBendVolume::Dump() const
{
  Dump(FALSE, 0) ;
}
/*******************************************************************//**
PURPOSE: Dump bspline volume data out for debugging.
            Dump is in similar sequence to Write to file

NOTES: 
***********************************************************************/
void SmBendVolume::Dump
 (SmBoolean bAbbrev,  // in : TRUE = skip nested object dumps, FALSE=include nested object dumps
  ULONG lIndentCnt)   // in : Indent print statements by lIndentCnt number of spaces
 const
{
  // locals
  ULONG ii ;
  TCHAR sBuff[SM_TBLOCK_SIZE], sBuffForFile[SM_TBLOCK_SIZE], sIndent[32] ;

  // build prefix
  for(ii=0;ii<lIndentCnt;ii++) 
    { SM_STRCAT(sIndent, _T(" ")) ; }

  // header
  smos_sprintf(sBuff,        _T("\n%sBegin SmBendVolume[0x%p]::Dump()"), sIndent, this) ;
  smos_sprintf(sBuffForFile, _T("\n%sBegin SmBendVolume::Dump()"), sIndent) ;
  smos_WriteBuffer(sBuff, sBuffForFile);

  // header
  smos_sprintf(sBuff,       _T("\n%s  SmBendVolume = 0x%p"), sIndent, this);
  smos_sprintf(sBuffForFile,_T("\n%s  SmBendVolume = %s"), sIndent, _T("notNULL"));
  smos_WriteBuffer(sBuff, sBuffForFile);

  smos_WriteBuffer(_T("\n")) ;
  smos_sprintf(sBuff, _T("\n%s  BendOrigin      = InSpace point on the bend center line"), sIndent) ; smos_WriteBuffer(sBuff) ;
  smos_sprintf(sBuff, _T("\n%s  BendAxis        = InSpace bend center line unit-vector direction"), sIndent) ; smos_WriteBuffer(sBuff) ;
  smos_sprintf(sBuff, _T("\n%s  BendMidDir      = InSpace orthoganal to BendAxis marking Bend Middle and start of Bend Coordinate system"), sIndent) ; smos_WriteBuffer(sBuff) ;
  smos_sprintf(sBuff, _T("\n%s  BendBinormal    = InSpace unit-vector = BendAxis * BendMidDir"), sIndent) ; smos_WriteBuffer(sBuff) ;
  smos_sprintf(sBuff, _T("\n%s  BendNeutralDist = InSpace dist from center line to Neutral bend plane"), sIndent) ; smos_WriteBuffer(sBuff) ;
  smos_sprintf(sBuff, _T("\n%s  BendSingularityTol  = min dist between NonSingular pts and Volume's singularity at U=0"), sIndent) ; smos_WriteBuffer(sBuff) ;
  smos_sprintf(sBuff, _T("\n%s                        dSingularityTol must be >= SM_ZONE_TOL_3D, default:[SM_ZONE_TOL_3D]"), sIndent) ; smos_WriteBuffer(sBuff) ;
  smos_sprintf(sBuff, _T("\n%s    note: The Bend Mapping has two symmetric planes:"), sIndent) ; smos_WriteBuffer(sBuff) ;
  smos_sprintf(sBuff, _T("\n%s         1. The BendMidDir/BendAxis plane splits the bend lengthwise"), sIndent) ; smos_WriteBuffer(sBuff) ;
  smos_sprintf(sBuff, _T("\n%s         2. The BendMidDir/BendBinormal plane splits the bend crosswise at the BendOrigin"), sIndent) ; smos_WriteBuffer(sBuff) ;
  smos_WriteBuffer(_T("\n")) ;
  
  smos_sprintf(sBuff, _T("\n%s    BendOrigin         = "), sIndent) ; smos_WriteBuffer(sBuff) ; GetBendOrigin().Dump() ;
  smos_sprintf(sBuff, _T("\n%s    BendAxis           = "), sIndent) ; smos_WriteBuffer(sBuff) ; GetBendAxis().Dump() ;
  smos_sprintf(sBuff, _T("\n%s    BendMidDir         = "), sIndent) ; smos_WriteBuffer(sBuff) ; GetBendMidDir().Dump() ;
  smos_sprintf(sBuff, _T("\n%s    BendBinormal       = "), sIndent) ; smos_WriteBuffer(sBuff) ; GetBendBinormal().Dump() ;
  smos_sprintf(sBuff, _T("\n%s    BendNeutralDist    =  %16.16lf"), sIndent, GetBendNeutralDist()) ; smos_WriteBuffer(sBuff) ;
  smos_sprintf(sBuff, _T("\n%s    BendSingularityTol =  %16.16lf"), sIndent, GetSingularityTol()) ;  smos_WriteBuffer(sBuff) ;

  // dump the base class 
  SmVolume::Dump(bAbbrev, lIndentCnt+2) ;

  smos_sprintf(sBuff,        _T("\n%sEnd SmBendVolume[0x%p]::Dump()%s"), sIndent, this, lIndentCnt==0?_T("\n"):_T("")) ;
  smos_sprintf(sBuffForFile, _T("\n%sEnd SmBendVolume::Dump()%s"), sIndent, lIndentCnt==0?_T("\n"):_T("")) ;
  smos_WriteBuffer(sBuff, sBuffForFile);
  
} // end SmBendVolume::Dump

