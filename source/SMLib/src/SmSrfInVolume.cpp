// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************//**
* FILE NAME --- SmSrfInVolume.cpp
* PURPOSE: Source file for SmSrfInVolume methods.
**********************************************************************/

#include "StdAfx.h"

#include <nurbs.h>
#include <SmSrfInVolume.h>

#include <SmExtent2d.h>
#include <SmExtent3d.h>
#include <SmBSplineSurface.h>
#include <SmGraphicsExtern.h>
#include <SmPseudoBox.h>
#include <SmPolarBox.h>
#include <SmDatabaseIO.h>
#include <SmVolume.h>
#include <SmAssertArray.h>
#include <SmCrvInVolume.h>
#include <SmCrvOnSurf.h>
#include <SmLine.h>

#ifdef SM_DEBUG_CODE
 #include <SmBrep.h>
 #include <SmFace.h>
 #include <SmGap.h>
#endif // SM_DEBUG_CODE

#ifdef SM_GFX_OUTPUT_CODE
 #include <SmGraphicsOutput.h>
#endif

/*******************************************************************//**
PURPOSE: Constructor for Surface in Volume objects.  

NOTES: It loads values and checks to make sure that surface is valid.

  Never let an rSurface or rVolume object be owned by two objects.

  When (lOwnerFlag&1) rSurface->m_pOwner set to this.  
  If rSurface is owned on input by a Face (or any other object), the
  caller must make sure to fix that parent's now obsolete pointer.
  For example, When rSurface->OldOwner is a Face, a call to 
  pFace->ReplaceSurface fixes the problem.
***********************************************************************/
SmSrfInVolume::SmSrfInVolume
 (SmSurface        & rSurface,       // in : projected Surface - When(lOwnerFlag&1) rSurface->m_pOwner = this
  SmBoolean          bInParamSpace,  // in : TRUE = m_pSurface is in m_pVolume's ParamSpace, FALSE = in InSpace
  SmVolume         & rVolume,        // in : projecting Volume - When(lOwnerFlag&2) rVolume->m_pOwner = this
  ULONG              lCopyFlag,      // in : 0 = saves surface and volume without copying
                                     //      1 = copy surface and save volume orig
                                     //      2 = copy volume and save surface orig
                                     //      3 = copy both surface and volume
                                     //      default:[0]
  ULONG              lOwnerFlag,     // in : 0 = deletes nothing when destructed
                                     //      1 = delete surface but not volume when destructed
                                     //      2 = delete volume but not surface when destructed
                                     //      3 = delete both surface and volume when destructed
                                     //      default:[0]
  const SmContext  * cpContext)      // in : req for new stack objs, opt for new heap objs
                                     // in : default:[NULL]
                         
 : m_pSurface(&rSurface),
   m_bInParamSpace(bInParamSpace), 
   m_pVolume(&rVolume),
   m_lOwnerFlag(lOwnerFlag)
{
  // when passed a context - use it
  if(cpContext) 
    { 
      SM_ASSERT(GetContext() == NULL || GetContext() == cpContext) ;
      SetContext(cpContext) ; 
    }
  else if(m_cpContext == NULL)          
    { 
      m_cpContext = rSurface.GetContext() ; 
    }
                
  // When input Context is NULL, set from Surface or Volume context when possible
  if(GetContext() == NULL && rVolume.GetContext() != NULL)
    { SetContext(rVolume.GetContext()) ; }

  if(GetContext() == NULL && rSurface.GetContext() != NULL)
    { SetContext(rSurface.GetContext()) ; }

  // When Surface is to be copied - copy it
  if (   lCopyFlag == 1 
      || lCopyFlag == 3) 
    {
      const SmContext * pContext = GetContext() ;
      if (pContext == NULL) { SE(SM_ERR) ; return; }
      m_pSurface->Copy(*pContext, m_pSurface) ; 
      m_lOwnerFlag |= 1 ;
    }

  // when Volume is to be copied - copy it
  if (   lCopyFlag == 2 
      || lCopyFlag == 3) 
    {
      const SmContext * pContext = GetContext() ;
      if (pContext == NULL) { SE(SM_ERR) ; return; }
      m_pVolume->Copy(*pContext, m_pVolume) ;
      m_lOwnerFlag |= 2 ;
    }

  // set the contained Surface and Volume owner values
  if(m_lOwnerFlag & 1) { m_pSurface->SetOwner(this) ; }
  if(m_lOwnerFlag & 2) { m_pVolume->SetOwner(this) ; }

  // try to make sure all objects are using the same context
  if(m_pSurface->GetContext() == NULL) { m_pSurface->SetContext(GetContext()) ; }
  if(m_pVolume->GetContext()  == NULL) { m_pVolume->SetContext(GetContext()) ; }

} // end SmSrfInVolume::SmSrfInVolume constructor

/*******************************************************************//**
PURPOSE: Copy constructor for SmSrfInVolume object.

NOTES: The constructed object will make deep copies of both the target
  surface's m_pSurface and m_pVolume when ever the target surface being copied
  owns either.  When the target surface just has an unowned reference to either
  then an unowned reference will be placed in the constructed object.
***********************************************************************/
SmSrfInVolume::SmSrfInVolume
  (const SmSrfInVolume & crSurfaceToCopy)
 : SmSurface         (crSurfaceToCopy), 
   m_pSurface        (crSurfaceToCopy.m_pSurface),
   m_bInParamSpace   (crSurfaceToCopy.m_bInParamSpace),
   m_pVolume         (crSurfaceToCopy.m_pVolume), 
   m_lOwnerFlag      (crSurfaceToCopy.m_lOwnerFlag)
{
  // figure out what the target surface owns - those things will be deep copied
  SmBoolean bTargetOwnsSurface = (   m_lOwnerFlag == 1
                                  || m_lOwnerFlag == 3) ;
  SmBoolean bTargetOwnsVolume  = (   m_lOwnerFlag == 2
                                  || m_lOwnerFlag == 3) ; 
  
  // get a context for new object constrution                                
  const SmContext *pContext =   GetContext()                             ? GetContext()
                              : crSurfaceToCopy.GetContext()             ? crSurfaceToCopy.GetContext()
                              : crSurfaceToCopy.m_pSurface->GetContext() ? crSurfaceToCopy.m_pSurface->GetContext()
                              : crSurfaceToCopy.m_pVolume->GetContext()  ? crSurfaceToCopy.m_pVolume->GetContext()
                              : NULL ;                                    
  if (pContext == NULL) 
    { SE_MSG(SM_ERR, _T("SmSrfInVolume copy constructor can't find a SmContext for new object construction")) ; }
                                      
  // copy Surface when its owned by this object
  if (bTargetOwnsSurface) 
    {
      // copy the Surface
      crSurfaceToCopy.m_pSurface->Copy(*pContext, m_pSurface) ;
    }

  // copy Volume when its owned by this object
  if (bTargetOwnsSurface || bTargetOwnsVolume) 
    {
      // copy the Volume
      crSurfaceToCopy.m_pVolume->Copy(*pContext, m_pVolume) ;
    }

  // set the owner flag
  m_lOwnerFlag =   (bTargetOwnsSurface == TRUE && bTargetOwnsVolume == TRUE) ? 3
                 : (bTargetOwnsSurface == TRUE) ? 1
                 : (bTargetOwnsVolume  == TRUE) ? 2
                 : 0 ;

  // set the contained Surface and Volume owner values
  if(m_lOwnerFlag & 1) { m_pSurface->SetOwner(this) ; }
  if(m_lOwnerFlag & 2) { m_pVolume->SetOwner(this) ; }

  // inform the public
  ((SmSurface &)crSurfaceToCopy).Notify(SM_NO_COPY, this, SM_NO_GET_OWNER(this), SM_NO_GET_OWNER(crSurfaceToCopy)) ;

} // end SmSrfInVolume::SmSrfInVolume copy constructor

/*******************************************************************//**
PURPOSE: Copy a Surface in Volume.

NOTES: 
***********************************************************************/
SmStatus SmSrfInVolume::Copy
  (const SmContext & crContext,
   SmSurface      *& rpNewSurface) 
  const
{
  rpNewSurface = new (crContext) SmSrfInVolume(*this) ;
  NER(rpNewSurface) ;
  return SM_SUCCESS;

} // end SmSrfInVolume::Copy

/*******************************************************************//**
PURPOSE: Destructor for the SmSrfInVolume object.  It cleans up
    memory for the UV surface and volume if it owns them.

NOTES: 
***********************************************************************/
SmSrfInVolume::~SmSrfInVolume()
{
  // not needed - called in base class destructor
  //      // notify mechanism removes CurveCache and Attributes, and calls the UserCallback 
  //      Notify(SM_NO_DESTRUCTION, this, NULL) ; 

  // delete contained structures when appropriate
  if (m_pSurface) { if(OwnsSurface()) { delete m_pSurface ; m_pSurface = NULL ; } }
  if (m_pVolume ) { if(OwnsVolume())  { delete m_pVolume ;  m_pVolume  = NULL ; } }

} // end SmSrfInVolume::~SmSrfInVolume destructor

/*******************************************************************//**
PURPOSE: Equality operator for SmSrfInVolume

NOTES: Call base equivalence to check type and then check 
       members for equivalence
***********************************************************************/
SmBoolean SmSrfInVolume::operator==
  (const SmSurface& crOther) 
 const
{
  // low work
  if(this == &crOther) { return TRUE ; }

  // first check the base
  SmBoolean bRtn = SmSurface::operator ==(crOther) ;

  if(bRtn)
    {
      // OK to cast
      SmSrfInVolume &rOther = (SmSrfInVolume &)crOther ;

      // check equivalence of these objects
      bRtn =  (   (   m_bInParamSpace == rOther.m_bInParamSpace)
               && (   ( m_pSurface == rOther.m_pSurface)
                   || ( m_pSurface == NULL && rOther.m_pSurface == NULL)
                   || (   m_pSurface != NULL && rOther.m_pSurface != NULL
                       && *m_pSurface == *rOther.m_pSurface))
               && (   ( m_pVolume == rOther.m_pVolume)
                   || ( m_pVolume == NULL && rOther.m_pVolume == NULL)
                   || (   m_pVolume != NULL && rOther.m_pVolume != NULL
                       && *m_pVolume == *rOther.m_pVolume))) ;
    }

  // all done
  return bRtn ;

} // end SmSrfInVolume::operator==

/*******************************************************************//**
PURPOSE: Create a mirror surface of an SmSrfInVolume surface.

NOTES: 
***********************************************************************/
SmStatus SmSrfInVolume::CreateMirrorSurface
  (const SmContext        & crContext,        // in : context for new object 
   const SmAxis2Placement & crMirrorPlane,    // in : mirror plane
   SmSurface             *& rpMirrorSurface)  // out: mirrored surface
  const
{
  // copy this surface
  SmSrfInVolume *pNewSurface = new (crContext) SmSrfInVolume(*this) ;
  
  // mirror the the volume
  SER(pNewSurface->m_pVolume->Mirror(crMirrorPlane)) ;

  // create the return object
  rpMirrorSurface = pNewSurface ;

  // all done
  return SM_SUCCESS;

} // end SmSrfInVolume::CreateMirrorSurface

/*******************************************************************//**
PURPOSE: For special orientations of some shapes make and return
         an exact BSpline equivalent Surface to this SmSrfInVolume object.

NOTES: An SmSrfInVolume object projects its m_pSurface from the m_pVolume's 
       InSpace to its final(Compounding) OutSpace
***********************************************************************/
SmStatus SmSrfInVolume::MakeExactBSplineIfPossible
 (SmBSplineSurface *& rpNewBSplineSurface) 
 const
{ 
  // locals
  SmStatus         sRtn    = SM_SUCCESS ;
  const SmVolume * pVolume = GetVolume() ;

  // Ask Volume to make exact BSpline if Possible - handles compounding
  if(m_bInParamSpace)
    {
      sRtn = pVolume->MakeExactBSplineOutSurfaceFromParamSpace(*this->GetSurface(), rpNewBSplineSurface) ;
    }
  else
    {
      sRtn = pVolume->MakeExactBSplineOutSurfaceFromInSpace(*this->GetSurface(), rpNewBSplineSurface) ;
    }

  // all done
  return(sRtn) ; 

} // end SmSrfInVolume::MakeExactBSplineIfPossible

/*******************************************************************//**
PURPOSE: Return list of internal discontinuities inherited from the base surface.
    
NOTES: SmSrfInVolume does not really have knots, it has internal
    discontinuities inherited from the discontinuities of the base surface
    plus the discontinuities of the volume.  Since tbe base surface
    and the volume may align arbitrarity, it's impossible to assemble a simple
    U and V list of such discontinuities.  For now - this method just returns
    the subset of discontinuities inherited from the base surface.

    This is the STEP compatible form of the knots not the
    typical knot vector associated with NURBS.
***********************************************************************/
SmStatus SmSrfInVolume::GetKnots
  (SmSurfParamType    eSurfParam,            // in :    
   SmTArray<double> & rKnots,                // out: 
   SmTArray<ULONG>  * pKnotMultiplicities,   // out:
   const SmExtent1d * pOptIvl)               // in : interval of interest, NULL=Natural Interval, default:[NULL] 
  const
{    
  SER(m_pSurface->GetKnots(eSurfParam,rKnots,pKnotMultiplicities,pOptIvl)) ;

  return SM_SUCCESS;

} // end SmSrfInVolume::GetKnots

/*******************************************************************//**
PURPOSE: Set Owns contained Surface and Volume flat

NOTES: When a contained Surface or Volume becomes owned,
       its m_pOwner pointer is set to this
***********************************************************************/
void SmSrfInVolume::SetOwnerFlag
 (ULONG lOwnerFlag)    // in : 0 = deletes nothing when destructed
                       //      1 = deletes only surface when destructed
                       //      2 = deletes only volume  when destructed
                       //      3 = deletes both surface and volume when destructed
                       //      default:[0]
{ 
  // maek the assignment
  m_lOwnerFlag = lOwnerFlag ; 

  // when owned, set the contained Surface and Volume owner values
  if(m_lOwnerFlag & 1) { m_pSurface->SetOwner(this) ; }
  if(m_lOwnerFlag & 2) { m_pVolume->SetOwner(this) ; }

} // end SmSrfInVolume::SetOwnerFlag

/*******************************************************************//**
PURPOSE: Ensures this surface owns its base Surface and compounding volume

NOTES: Any unowned object is copied and its owner flag is modified
***********************************************************************/
SmStatus SmSrfInVolume::MakeOwner()
{
  // copy and own unowned surfaces
  if(!OwnsSurface())
    {
      // copy the surface
      SER(m_pSurface->Copy(*GetContext(), m_pSurface)) ;
      m_lOwnerFlag |= 1 ;
    }

  // copy and own unowned volumes
  if(!OwnsVolume())
    {
      // copy the volume
      SER(m_pVolume->Copy(*GetContext(), m_pVolume)) ;

      // remember to own the volume
      m_lOwnerFlag |= 2 ; 

    } // end need to deep copy volume check

  // when owned, set the contained Surface and Volume owner values
  if(m_lOwnerFlag & 1) { m_pSurface->SetOwner(this) ; }
  if(m_lOwnerFlag & 2) { m_pVolume->SetOwner(this) ; }

  // all done
  return(SM_SUCCESS) ;

} // end SmSrfInVolume::MakeOwner

/*******************************************************************//**
PURPOSE: Set Base Surface

NOTES: Make surface copy when needed
***********************************************************************/
SmStatus SmSrfInVolume::SetBaseSurface
 (SmSurface *pSurface,        // in : new base surface
  SmBoolean  bInParamSpace,   // in : TRUE = cpCurve is in Volume's ParamSpace, FAlSE=Curve in InSpace
  SmBoolean  bCopySurface,    // in : TRUE  = save a copy of pSurface, FALSE = save pSurface
                              //      default:[FALSE]
  SmBoolean  bOwnsSurface)    // in : TRUE = delete this surface when destructed, FALSE = don't
                              //      default:[FALSE]
{ 
  // when needed - clean out old surface
  if(m_pSurface && OwnsSurface())
    { delete m_pSurface ; m_pSurface = NULL ; }

  // make the assignment
  m_pSurface      = pSurface ;
  m_bInParamSpace = bInParamSpace ;
  m_lOwnerFlag   &= 2 ; // clear owns surface bit

  // Set owner flag and copy surface when necessary 
  if( bCopySurface) 
    { 
      // find a context
      const SmContext *pContext = GetContext() ? GetContext() : m_pSurface->GetContext() ;
      SER_MSG(pContext == NULL ? SM_ERR : SM_SUCCESS, _T("No Context available")) ;

      // copy the surface
      m_pSurface->Copy(*pContext, m_pSurface) ;

      // set owns surface bit
      m_lOwnerFlag |= 1 ; 
    }

  // when asked - remember to own this BaseSurface
  if(bOwnsSurface)
    {
      m_lOwnerFlag |= 1 ; 
    }

  // when owned, set the contained Surface and Volume owner values
  if(m_lOwnerFlag & 1) { m_pSurface->SetOwner(this) ; }
  if(m_lOwnerFlag & 2) { m_pVolume->SetOwner(this) ; }

  // all done
  return(SM_SUCCESS) ;

} // end SmSrfInVolume::SetBaseSurface

/*******************************************************************//**
PURPOSE: Set compounding Volume

NOTES: Make volume copy when needed
***********************************************************************/
SmStatus SmSrfInVolume::SetVolume
  (SmVolume  *pVolume,        // in : new base volume
   SmBoolean  bCopyVolume,    // in : TRUE  = save a copy of pVolume, FALSE = save pVolume
                              //      default:[FALSE]
   SmBoolean  bOwnsVolume)    // in : TRUE = delete this volume when destructed, FALSE = don't
                              //      default:[FALSE]
{ 
  // when needed - clean out old volume
  if(m_pVolume && OwnsVolume())
    { delete m_pVolume ; m_pVolume = NULL ; }

  // make the assignment
  m_pVolume     = pVolume ;
  m_lOwnerFlag &= 1 ;    // clear owns volume bit
  
  // Set owner flag and copy volume when necessary 
  if( bCopyVolume) 
    { 
      // find a context
      const SmContext *pContext = GetContext() ? GetContext() : m_pVolume->GetContext() ;
      SER_MSG(pContext == NULL ? SM_ERR : SM_SUCCESS, _T("No Context available")) ;

      // copy the volume
      m_pVolume->Copy(*pContext, m_pVolume) ;

       // set owns volume bit
      m_lOwnerFlag |= 2 ; 
    }

  // when asked - remember to own this Volume
  if(bOwnsVolume)
    {
      m_lOwnerFlag |= 2 ; 
    }

  // when owned, set the contained Surface and Volume owner values
  if(m_lOwnerFlag & 1) { m_pSurface->SetOwner(this) ; }
  if(m_lOwnerFlag & 2) { m_pVolume->SetOwner(this) ; }

  // all done
  return(SM_SUCCESS) ;

} // end SmSrfInVolume::SetVolume

/*******************************************************************//**
PURPOSE: Evaluate a point Surface in a volume.

NOTES: SrfInVolume(uv) = m_pVolume(m_pSurface(uv))
***********************************************************************/
SmStatus SmSrfInVolume::EvaluatePoint
  (const SmPoint2d & crUV,        // in : Target Domain Point
   SmPoint3d       & rPoint)      // out: Image Point
 const
{
  // pass the call along
  return(Evaluate(crUV, 0, 0, TRUE, TRUE, TRUE, &rPoint, FALSE )) ;

} // end SmSrfInVolume::EvaluatePoint

/*******************************************************************//**
PURPOSE: Evaluation of the normal of an SmSrfInVolume surface at a point.  
   This routine returns a unitized normal vector
NOTES:  
***********************************************************************/ 
SmStatus SmSrfInVolume::EvaluateNormal
  (const SmPoint2d & crUV,          // in : target domain point
   SmBoolean         bUFromLeft,    // in : if P is on U interval boundary
                                    //      TRUE  = evaluate P in upper interval where P is on the left of the interval
                                    //      FALSE = evaluate P in lower interval where P is on the right of the interval
   SmBoolean         bVFromLeft,    // in : if P is on V interval boundary - same as for U
   SmVector3d     & rSurfaceNormal) // out: Surface unit-normal
  const
{
  SmVector3d aDerivs[2 * 2] ;

  // evaluate 1st derivs with nonZeroTangents
  SER(Evaluate(crUV, 1, 1, bUFromLeft, bVFromLeft, TRUE, aDerivs, TRUE)) ;

  // compute the normal = Du cross Dv
  rSurfaceNormal = aDerivs[2] * aDerivs[1] ;
  
  // normalize the output
  SER(rSurfaceNormal.Unitize()) ; 

  // all done
  return(SM_SUCCESS) ; 

} // end SmSrfInVolume::EvaluateNormal

/*******************************************************************//**
PURPOSE: Evaluate a point and derivatives on the SmSrfInVolume surface.

NOTES: This eval does clamping.
***********************************************************************/
SmStatus SmSrfInVolume::Evaluate
  (const SmPoint2d & crUV,   // in : param value to evaluate
   ULONG lHighestUDeriv,     // in : number of U derivatives
   ULONG lHighestVDeriv,     // in : number of V derivatives to compute
   SmBoolean bUFromLeft,     // in : if P is on U interval boundary
                             //      TRUE  = evaluate P in upper interval where P is on the left of the interval
                             //      FALSE = evaluate P in lower interval where P is on the right of the interval
   SmBoolean bVFromLeft,     // in : if P is on V interval boundary
                             //      TRUE  = evaluate P in upper interval where P is on the left of the interval
                             //      FALSE = evaluate P in lower interval where P is on the right of the interval
   SmBoolean bOnlyUpperHalf, // in : TRUE=compute upper half of matrix only
                             //      ex. 1,1 = [D  Du] 2,2 = [D    Du    Duu] where -- = an untouched memory value 
                             //                [Dv --]       [Dv   Duv   ---]           (the memory has to be allocated)
                             //                              [Dvv  ---   ---]
   SmVector3d *aDerivatives, // out: matrix of evaluations values
                             //      sized:[lHighestUDeriv+1][lHighestVDeriv+1]
                             //      2d organized: [D    Du    Duu    Duuu    Duuuu   ]  (the same no matter the value of)
                             //                    [Dv   Duv   Duuv   Duuuv   Duuuuv  ]  (  bOnlyUpperHalf               )
                             //                    [Dvv  Duvv  Duuvv  Duuuvv  Duuuuvv ]
                             //                    [Dvvv Duvvv Duuvvv Duuuvvv Duuuuvvv]
                             //      1d organized: [D, Dv, Dvv, Dvvv,.. Du, Duv, Duvv, Duvvv,.. Duu, Duuv, Duuvv, Duuvvv,...]
                             //                     Du, Duv, Duvv, Duvvv,.. 
                             //                     Duu, Duuv, Duuvv, Duuvvv,...]
  SmBoolean bNonZeroTangents,// in : TRUE = replace zero tangent vectors with properly oriented tol sized vectors
                             //      FALSE= return exact tangent values
                             //      note: Surprisingly TRUE is the common choice because most tangent uses
                             //            are for their direction (Binorm, SurfNorm comps), but when the 
                             //            tangent is being used for its magnitude (like an arc-length comp)
                             //            then set this to FALSE.
                             //      default:[TRUE]
  SmBoolean )                // in : bDoZeroSampling = for internal use only, always set to TRUE, default:[TRUE]
 const 
{
  // check input
  // bOnlyUpperHalf == FALSE is supported for lHighestUDerv == 1 only
  // lHighestUDeriv != lHighestVDeriv is supported for cases lHighestUDeriv == 0 || lHighestVDeriv == 0.
  if(!(   ( (lHighestUDeriv == lHighestVDeriv) && (   (bOnlyUpperHalf == FALSE && lHighestUDeriv <= 1)
                                                   || (bOnlyUpperHalf == TRUE)))
       || ( (lHighestUDeriv != lHighestVDeriv) && (  (lHighestUDeriv == 0)
                                                   ||(lHighestVDeriv == 0)))))
    {
      SER_MSG( ( (lHighestUDeriv == lHighestVDeriv) && (   (bOnlyUpperHalf == FALSE && lHighestUDeriv <= 1)
                                                        || (bOnlyUpperHalf == TRUE)))
              ? SM_SUCCESS : SM_ERR,
              _T("only supports bOnlyUpperHalf == FALSE when lHighestUDeriv == lHighestVDeriv and lHighestUDeriv <= 1 - ask for support")) ; 

      SER_MSG( ( (lHighestUDeriv != lHighestVDeriv) && (   lHighestUDeriv == 0
                                                        || lHighestVDeriv == 0))
              ? SM_SUCCESS : SM_ERR,
              _T("only supports lHighestUDeriv != lHighestVDeriv when lHighestUDeriv == 0 || lHighestVDeriv == 0")) ; 
    }

  // limit the number of derivatives to 3
  const ULONG MAX_DERIVS           = 3;
  ULONG       lNumDerivsToEval     = smos_Max(lHighestUDeriv, lHighestVDeriv) ; 
  ULONG       lTempNumDerivsToEval = lNumDerivsToEval ;
  if(lNumDerivsToEval > MAX_DERIVS) lNumDerivsToEval = MAX_DERIVS ;

  // when bOnlyUpperHalf == TRUE && lHighestDeriv == 1, to compute Duv compute all 2nd Derivatives
  if( bOnlyUpperHalf == FALSE && lHighestUDeriv <= 1 && lHighestUDeriv == lHighestVDeriv )
    {
      lTempNumDerivsToEval = 2 ;
    }

  // For indexing into the volume pt/deriv array:
#define ss(u,v,HighestVDeriv)   (u*(HighestVDeriv+1)+v)
#define sv(u,v,w,n) ((u*(n+1)+v)*(n+1)+w)

  // surface eval locals
  SmStatus         eStat    = SM_SUCCESS ;
  SmPoint2d        sUV      = crUV ;
  const SmSurface *pSurface = m_pSurface;
  SmVector3d       aSrfPtDerivs[ (1+MAX_DERIVS) * (1+MAX_DERIVS) ] ;
  SmVector3d       aVolPtDerivs[ (1+MAX_DERIVS) * (1+MAX_DERIVS) * (1+MAX_DERIVS) ] ;
  SmVector3d       aTempDerivs [ (1+MAX_DERIVS) * (1+MAX_DERIVS) ] ;
  SmExtent2d       sDomain  = m_pSurface->GetNaturalUVDomain() ;

  // Check for evaluating outside of BaseSurface domain.
  // If so, check whether we have an extended surface.
  // If not, then we either clamp the point or try to
  // create an extended surface, depending on some
  // other criteria, below.

  // When EvalPoint is still outside of baseDomain - extend BaseSurface or clamp EvalPoint as possible
  if (!sDomain.ContainsPoint2d(sUV)) 
    {
      // just clamp the UVPoint   // GWC - this duplicates the SmOffsetSurface behavior - are we happy with that?
      sUV = sDomain.ClampPoint2d(sUV) ;
    } // end UVPoint not in Surf->Domain check

  // arrive here when EvalPoint (sUV) is within pSurface domain

  // Surface eval
  SER(pSurface->Evaluate(sUV,
                         lTempNumDerivsToEval, lTempNumDerivsToEval, // lNumDerivsToEval = smos_Max(lHighestUDeriv, lHighestVDeriv)
                         bUFromLeft, bVFromLeft,
                         TRUE, // bOnlyUpperHalf, 
                         aSrfPtDerivs,
                         bNonZeroTangents )) ; // True: adjust to non-zero first derivatives

  // Volume eval
  //  gwc: something smarter needs to be worked out for the FromLeft values
  if(m_bInParamSpace)
    {
      eStat = m_pVolume->Evaluate ( aSrfPtDerivs[0],
                                    lTempNumDerivsToEval, 
                                    (aSrfPtDerivs[ss(1,0,lNumDerivsToEval)].x > 0.0) ? bUFromLeft : !bUFromLeft, 
                                    (aSrfPtDerivs[ss(1,0,lNumDerivsToEval)].y > 0.0) ? bUFromLeft : !bUFromLeft, 
                                    (aSrfPtDerivs[ss(1,0,lNumDerivsToEval)].z > 0.0) ? bUFromLeft : !bUFromLeft,
                                    aVolPtDerivs,
                                    bNonZeroTangents ) ;
    } // end Surface in ParamSpace branch
  else // Surface in InSpace branch
    {
      eStat = m_pVolume->Map ( aSrfPtDerivs[0],
                               lTempNumDerivsToEval, 
                               (aSrfPtDerivs[ss(1,0,lNumDerivsToEval)].x > 0.0) ? bUFromLeft : !bUFromLeft, 
                               (aSrfPtDerivs[ss(1,0,lNumDerivsToEval)].y > 0.0) ? bUFromLeft : !bUFromLeft, 
                               (aSrfPtDerivs[ss(1,0,lNumDerivsToEval)].z > 0.0) ? bUFromLeft : !bUFromLeft,
                               aVolPtDerivs,
                               bNonZeroTangents ) ;
    } // end Surface in InSpace branch


  // GWC: When a Mapping fails - this function fails - it's too confusing to get partial results
  //      // If the volume can't handle this many derivatives: 
  //      while ( eStat != SM_SUCCESS && lNumDerivsToEval > 0 )
  //        {
  //          lNumDerivsToEval--;
  //      
  //          // try a Volume Eval with fewer derivatives
  //          //  gwc: something smarter needs to be worked out for the FromLeft values
  //          eStat = m_pVolume->Map ( aSrfPtDerivs[0],
  //                                   lNumDerivsToEval, 
  //                                   (aSrfPtDerivs[ss(1,0,lNumDerivsToEval)].x > 0.0) ? bUFromLeft : !bUFromLeft, 
  //                                   (aSrfPtDerivs[ss(1,0,lNumDerivsToEval)].y > 0.0) ? bUFromLeft : !bUFromLeft, 
  //                                   (aSrfPtDerivs[ss(1,0,lNumDerivsToEval)].z > 0.0) ? bUFromLeft : !bUFromLeft,
  //                                   aVolPtDerivs,
  //                                   bNonZeroTangents ) ;
  //        } // end while looking for volume max derivative count
  SER( eStat ) ;

  // set outputs

  // Position
  aDerivatives[0] = aVolPtDerivs[0];

  // 1st Derivs
  if ( lNumDerivsToEval >= 1 )
    {
      // When managing the special cases of UpperHalf == FALSE or lHighestUDeriv != lHighestVDeriv evals use temporary memory
      //   the case where lNumDerivsToEval != lTempNumDerivsToEval, NumDerivs == 1 && bOnlyUpperHalf == FALSE,
      //   the case where lHighestUDeriv != lHighestVDeriv, LHighestUDeriv == 0 || lHighestVDeriv == 0.
      SmVector3d *aDerivs =  (lHighestUDeriv == lHighestVDeriv)
                            ? aDerivatives
                            : aTempDerivs ;

      smvol_LiftFirstDerivative(aSrfPtDerivs[ ss(1,0, lTempNumDerivsToEval) ],
                                aSrfPtDerivs[ ss(0,1, lTempNumDerivsToEval) ],

                                aVolPtDerivs[ sv(1,0,0, lTempNumDerivsToEval) ],
                                aVolPtDerivs[ sv(0,1,0, lTempNumDerivsToEval) ],
                                aVolPtDerivs[ sv(0,0,1, lTempNumDerivsToEval) ],

                                aDerivs[ ss(1,0, lNumDerivsToEval) ],
                                aDerivs[ ss(0,1, lNumDerivsToEval) ]) ;

      if(lHighestUDeriv != lHighestVDeriv)
        {
          if(lHighestUDeriv > 0) aDerivatives[ ss(1,0, lHighestVDeriv) ] = aDerivs[ ss(1,0, lNumDerivsToEval) ] ; 
          if(lHighestVDeriv > 0) aDerivatives[ ss(0,1, lHighestVDeriv) ] = aDerivs[ ss(0,1, lNumDerivsToEval) ] ; 
        }
         
    } // end 1st Derivs
  
  // 2nd Derivs
  if (   lNumDerivsToEval  >= 2 
      || lTempNumDerivsToEval >= 2)
    {
      // When managing the special cases of UpperHalf == FALSE or lHighestUDeriv != lHighestVDeriv evals use temporary memory
      //   the case where lNumDerivsToEval != lTempNumDerivsToEval, NumDerivs == 1 && bOnlyUpperHalf == FALSE,
      //   the case where lHighestUDeriv != lHighestVDeriv, LHighestUDeriv == 0 || lHighestVDeriv == 0.
      SmVector3d *aDerivs =  (   (lNumDerivsToEval == lTempNumDerivsToEval)
                              && (lHighestUDeriv   == lHighestVDeriv))
                             ? aDerivatives
                             : aTempDerivs ;

      smvol_LiftSecondDerivative( aSrfPtDerivs[ ss(1,0, lTempNumDerivsToEval) ],
                                  aSrfPtDerivs[ ss(0,1, lTempNumDerivsToEval) ],

                                  aSrfPtDerivs[ ss(2,0, lTempNumDerivsToEval) ],
                                  aSrfPtDerivs[ ss(1,1, lTempNumDerivsToEval) ],
                                  aSrfPtDerivs[ ss(0,2, lTempNumDerivsToEval) ],

                                  aVolPtDerivs[ sv(1,0,0, lTempNumDerivsToEval) ],
                                  aVolPtDerivs[ sv(0,1,0, lTempNumDerivsToEval) ],
                                  aVolPtDerivs[ sv(0,0,1, lTempNumDerivsToEval) ],

                                  aVolPtDerivs[ sv(2,0,0, lTempNumDerivsToEval) ],
                                  aVolPtDerivs[ sv(1,1,0, lTempNumDerivsToEval) ],
                                  aVolPtDerivs[ sv(1,0,1, lTempNumDerivsToEval) ],
                                  aVolPtDerivs[ sv(0,2,0, lTempNumDerivsToEval) ],
                                  aVolPtDerivs[ sv(0,1,1, lTempNumDerivsToEval) ],
                                  aVolPtDerivs[ sv(0,0,2, lTempNumDerivsToEval) ],

                                  aDerivs[ ss(2,0, lTempNumDerivsToEval) ],
                                  aDerivs[ ss(1,1, lTempNumDerivsToEval) ],
                                  aDerivs[ ss(0,2, lTempNumDerivsToEval) ] ) ;

      // When managing the special cases move results from temporary memory to output memory
      //   the case where lNumDerivsToEval != lTempNumDerivsToEval, NumDerivs == 1 && bOnlyUpperHalf == FALSE,
      if(lNumDerivsToEval != lTempNumDerivsToEval)
        {
          SM_ASSERT_MSG(lNumDerivsToEval     == 1,     _T("SmSrfInVolume::Evaluate is using a special case branch inappropriately")) ;
          SM_ASSERT_MSG(lTempNumDerivsToEval == 2,     _T("SmSrfInVolume::Evaluate is using a special case branch inappropriately")) ;
          SM_ASSERT_MSG(bOnlyUpperHalf       == FALSE, _T("SmSrfInVolume::Evaluate is using a special case branch inappropriately")) ;

          // copy the final DUV derivative to the output
          aDerivatives[ ss(1,1, lHighestVDeriv) ] = aDerivs [ ss(1,1, lTempNumDerivsToEval) ] ;
        }

      //   the case where lHighestUDeriv != lHighestVDeriv, LHighestUDeriv == 0 || lHighestVDeriv == 0.
      else if(lHighestUDeriv != lHighestVDeriv)
        {
          SM_ASSERT_MSG(lHighestUDeriv == 0 || lHighestVDeriv == 0, _T("SmSrfInVolume::Evaluate is using a special case branch inappropriately")) ;
          if(lHighestUDeriv > 1) { aDerivatives[ ss(2,0, lHighestVDeriv) ] = aDerivs [ ss(2,0, lTempNumDerivsToEval) ] ; } 
          if(lHighestVDeriv > 1) { aDerivatives[ ss(0,2, lHighestVDeriv) ] = aDerivs [ ss(0,2, lTempNumDerivsToEval) ] ; }
        }

    } // end 2nd Derivs
  
  // 3rd Derivs
  if ( lNumDerivsToEval >= 3 )
    {
      SM_ASSERT_MSG(lNumDerivsToEval == lTempNumDerivsToEval, _T("SmSrfInVolume::Evaluate NumDerivs computation is out of synch - needs review")) ;

      // when managing the special case of lHighestUDeriv   != lHighestVDeriv is when one or the other is 0.
      SmVector3d *aDerivs =  (lHighestUDeriv   == lHighestVDeriv)
                             ? aDerivatives
                             : aTempDerivs ;

      smvol_LiftThirdDerivative( aSrfPtDerivs[ ss(1,0, lTempNumDerivsToEval) ],
                                 aSrfPtDerivs[ ss(0,1, lTempNumDerivsToEval) ],

                                 aSrfPtDerivs[ ss(2,0, lTempNumDerivsToEval) ],
                                 aSrfPtDerivs[ ss(1,1, lTempNumDerivsToEval) ],
                                 aSrfPtDerivs[ ss(0,2, lTempNumDerivsToEval) ],

                                 aSrfPtDerivs[ ss(3,0, lTempNumDerivsToEval) ],
                                 aSrfPtDerivs[ ss(2,1, lTempNumDerivsToEval) ],
                                 aSrfPtDerivs[ ss(1,2, lTempNumDerivsToEval) ],
                                 aSrfPtDerivs[ ss(0,3, lTempNumDerivsToEval) ],

                                 aVolPtDerivs[ sv(1,0,0, lTempNumDerivsToEval) ],
                                 aVolPtDerivs[ sv(0,1,0, lTempNumDerivsToEval) ],
                                 aVolPtDerivs[ sv(0,0,1, lTempNumDerivsToEval) ],

                                 aVolPtDerivs[ sv(2,0,0, lTempNumDerivsToEval) ],
                                 aVolPtDerivs[ sv(1,1,0, lTempNumDerivsToEval) ],
                                 aVolPtDerivs[ sv(1,0,1, lTempNumDerivsToEval) ],
                                 aVolPtDerivs[ sv(0,2,0, lTempNumDerivsToEval) ],
                                 aVolPtDerivs[ sv(0,1,1, lTempNumDerivsToEval) ],
                                 aVolPtDerivs[ sv(0,0,2, lTempNumDerivsToEval) ],
                                                    
                                 aVolPtDerivs[ sv(3,0,0, lTempNumDerivsToEval) ],
                                 aVolPtDerivs[ sv(2,1,0, lTempNumDerivsToEval) ],
                                 aVolPtDerivs[ sv(2,0,1, lTempNumDerivsToEval) ],
                                 aVolPtDerivs[ sv(1,2,0, lTempNumDerivsToEval) ],
                                 aVolPtDerivs[ sv(1,1,1, lTempNumDerivsToEval) ],
                                 aVolPtDerivs[ sv(1,0,2, lTempNumDerivsToEval) ],
                                 aVolPtDerivs[ sv(0,3,0, lTempNumDerivsToEval) ],
                                 aVolPtDerivs[ sv(0,2,1, lTempNumDerivsToEval) ],
                                 aVolPtDerivs[ sv(0,1,2, lTempNumDerivsToEval) ],
                                 aVolPtDerivs[ sv(0,0,3, lTempNumDerivsToEval) ],

                                 aDerivs[ ss(3,0, lTempNumDerivsToEval) ],
                                 aDerivs[ ss(2,1, lTempNumDerivsToEval) ],
                                 aDerivs[ ss(1,2, lTempNumDerivsToEval) ],
                                 aDerivs[ ss(0,3, lTempNumDerivsToEval) ]) ;

      // When managing the special cases move results from temporary memory to output memory
      //   the case where lHighestUDeriv != lHighestVDeriv, LHighestUDeriv == 0 || lHighestVDeriv == 0.
      if(lHighestUDeriv != lHighestVDeriv)
        {
          SM_ASSERT_MSG(lHighestUDeriv == 0 || lHighestVDeriv == 0, _T("SmSrfInVolume::Evaluate is using a special case branch inappropriately")) ;
          if(lHighestUDeriv > 2) { aDerivatives[ ss(3,0, lHighestVDeriv) ] = aDerivs [ ss(3,0, lTempNumDerivsToEval) ] ; } 
          if(lHighestVDeriv > 2) { aDerivatives[ ss(0,3, lHighestVDeriv) ] = aDerivs [ ss(0,3, lTempNumDerivsToEval) ] ; }
        }

    } // end 3rd Derivs

  // If too many requested, fill with zeros.
  ULONG ii, jj, kk ; 
  for ( kk = MAX_DERIVS+1; kk <= lNumDerivsToEval; kk++ )
    {
      for(ii=kk,jj=0; jj<=kk; ii--,jj++)
      aDerivatives[ ss(ii,jj, lHighestVDeriv) ].Set( 0, 0, 0 ) ;
    }

  // all done
#undef ss
#undef sv
  return SM_SUCCESS;

} // end SmSrfInVolume::Evaluate

/*******************************************************************//**
PURPOSE: Evaluate a point and derivatives on the SmSrfInVolume surface,
            without trying to correct zero-length first derivatives.

NOTES: 
***********************************************************************/
SmStatus SmSrfInVolume::EvaluateSimple
  (const SmPoint2d & crUV,    // in : target surface point
   ULONG lHighestUDeriv,      // in : Requested highest U derivative
   ULONG lHighestVDeriv,      // in : Requested highest V derivative
   SmBoolean bUFromLeft,      // in : if P is on U interval boundary
                              //      TRUE  = evaluate P in upper interval where P is on the left of the interval
                              //      FALSE = evaluate P in lower interval where P is on the right of the interval
   SmBoolean bVFromLeft,      // in : if P is on V interval boundary
                              //      TRUE  = evaluate P in upper interval where P is on the left of the interval
                              //      FALSE = evaluate P in lower interval where P is on the right of the interval
   SmBoolean bOnlyUpperHalf,  // in : TRUE=compute upper half of matrix only
                              //      ex. 1,1 = [D  Du] 2,2 = [D    Du    Duu] where -- = an untouched memory value 
                              //                [Dv --]       [Dv   Duv   ---] 
                              //                              [Dvv  ---   ---]
   SmVector3d *aDerivatives)  // out: matrix of evaluations values                   
  const                       //      sized:[lHighestUDeriv+1][lHighestVDeriv+1]     
                              //      2d organized: [D    Du    Duu    Duuu    Duuuu   ] 
                              //                    [Dv   Duv   Duuv   Duuuv   Duuuuv  ]
                              //                    [Dvv  Duvv  Duuvv  Duuuvv  Duuuuvv ]
                              //                    [Dvvv Duvvv Duuvvv Duuuvvv Duuuuvvv]
                              //      1d organized: [D, Dv, Dvv, Dvvv,.. Du, Duv, Duvv, Duvvv,.. Duu, Duuv, Duuvv, Duuvvv,...]

{
  // pass the call along
  return( Evaluate(crUV, 
                   lHighestUDeriv, lHighestVDeriv, 
                   bUFromLeft,     bVFromLeft,
                   bOnlyUpperHalf, aDerivatives, FALSE )) ;

} // end SmSrfInVolume::EvaluateSimple

/*******************************************************************//**
PURPOSE: Invert a 3d point on the surface, using the STEP representation if appropriate.

NOTES: Returns SM_ERR if the point is not on the surface within tolerance.
***********************************************************************/
SmStatus SmSrfInVolume::STEPInversion
(
  const SmExtent2d & crAnalUVDomain,      // in : 
  const SmPoint3d  & crPointOnSurf,       // in : 
  double             dDistanceTolerance,  // in : 
  SmPoint2d        & rdAnalUVParameter,   // out: 
  SmLocationType   & reLocation,          // out: 
  SmPoint2d        * pUVGuess             // NotUsed: in : 
) const
{
  SM_REF1(pUVGuess) ;
  // volume inversion locals
  SmBoolean bSuccess = FALSE ;
  SmTArray<SmPoint3d> sParamPoints ;
  SmTArray<double>    dGaps ;

  if(m_bInParamSpace)
    {
      // map the point back through the volume
      SER(m_pVolume->InvEvaluatePoint(crPointOnSurf, 
                                      NULL, 
                                      bSuccess,
                                      sParamPoints, 
                                      dGaps)) ;
    } // end Surface in ParamSpace branch
  else // Surface in InSpace branch
    {
      // map the point back through the volume
      SER(m_pVolume->InvMapPoint(crPointOnSurf, 
                                 NULL, 
                                 bSuccess,
                                 sParamPoints, 
                                 dGaps)) ;
    } // end Surface in InSpace branch

  SER_MSG(bSuccess == TRUE ? SM_SUCCESS : SM_ERR,
          _T("SmSrfInVolume::STEPInversion: failed to project point back through m_pVolume.")) ; 

  // Project the inverted volume point back through the base surface
  SER(m_pSurface->STEPInversion(crAnalUVDomain, 
                                sParamPoints[0], 
                                dDistanceTolerance, 
                                rdAnalUVParameter, 
                                reLocation)) ; 

  // all done
  return SM_SUCCESS ;

} // end SmSrfInVolume::STEPInversion

/*******************************************************************//**
PURPOSE: Reverse one of the parameterizations of the surface.

NOTES: 
***********************************************************************/
SmStatus SmSrfInVolume::Reverse(SmSurfParamType eSurfParam)
{
  // check input - Can not Reverse an unowned surface
  SER_MSG(OwnsSurface() ? SM_SUCCESS : SM_ERR,
          _T("Can not Reverse Surface when SmSrfInVolume does not own the surface")) ;

  Notify(SM_NO_PRE_EDIT, this, SM_NO_GET_OWNER(this), NULL) ;

  // base surface Reverse
  SER(m_pSurface->Reverse(eSurfParam)) ;

  Notify(SM_NO_POST_EDIT, this, SM_NO_GET_OWNER(this), NULL) ;                       
  return SM_SUCCESS;

} // end SmSrfInVolume::Reverse

/*******************************************************************//**
PURPOSE: Split surface.

NOTES: 
***********************************************************************/
SmStatus SmSrfInVolume::SplitAt
  (const SmContext & crContext,        // in : context for new object construction     
   double            dParam,           // in : split parameter                         
   SmSurfParamType   eSurfParam,       // in : oneof SM_SP_U = split u domain at dParam
                                       //            SM_SP_V = split v domain at dParam
   SmSurface      *& rpLeftSurface,    // out: SmBSplineSurface (left  or bot)         
   SmSurface      *& rpRightSurface)   // out: SmBSplineSurface (right or top)         
{
  // split Base Surface
  SmSurface *pSurLeft, *pSurRight;
  SER(m_pSurface->SplitAt(crContext,dParam,eSurfParam,pSurLeft,pSurRight)) ;

  // build left split surface from left baseSurface split
  SmSrfInVolume *pLeftSplit  = new (crContext) SmSrfInVolume(*pSurLeft,  m_bInParamSpace, *m_pVolume, 2, 3) ;
  
  // build right split surface from right baseSurface split
  SmSrfInVolume *pRightSplit = new (crContext) SmSrfInVolume(*pSurRight, m_bInParamSpace, *m_pVolume, 2, 3) ;

  // set output
  rpLeftSurface  = pLeftSplit;
  rpRightSurface = pRightSplit;

  Notify( SM_NO_SPLIT, rpLeftSurface, rpRightSurface, GetOwner() );

  // all done
  return SM_SUCCESS;

} // end SmSrfInVolume::SplitAt

/*******************************************************************//**
PURPOSE: Swap the UV parameterization of the surface.

NOTES: 
***********************************************************************/
SmStatus SmSrfInVolume::SwapUV()
{
  // check input - Can not swap UV on an unowned surface
  SER_MSG(OwnsSurface() ? SM_SUCCESS : SM_ERR,
          _T("Can not swap UV when SmSrfInVolume does not own the surface")) ;

  Notify(SM_NO_PRE_EDIT, this, SM_NO_GET_OWNER(this), NULL) ;

  // base surface Swap 
  SER(m_pSurface->SwapUV()) ;

  Notify(SM_NO_POST_EDIT, this, SM_NO_GET_OWNER(this), NULL) ;                       

  // all done
  return SM_SUCCESS ;

} // end SmSrfInVolume::SwapUV

/*******************************************************************//**
PURPOSE: Scale and transform a SmSrfInVolume surface.
   If scale is passed in apply it first then the rotation and
   move defined by the placement.

NOTES: When current BaseSurface shape is defined in InSpace.  
       The RotateNMove argument defines the
       position and orientation of that local coordinate system in
       global coordinates called OutSpace.  Transform() projects
       the surface from InSpace to OutSpace and then scales the result
       about the global origin.
***********************************************************************/
SmStatus SmSrfInVolume::Transform
  (const SmAxis2Placement & crRotateNMove, // in : affine rotate and move transformation      
   const SmVector3d       * cpOptScale)    // in : optional scaling about current origin point before RotateNMove
                                           //      BSplines, planes, lines, PolyBreps - support nonisotropic scaling
                                           //      other geom types only support isoptropic scaling
{
  SmVector3d sIdentityScale(1, 1, 1);

  // no work - identity transform
  if (crRotateNMove.IsIdentity() && (cpOptScale == NULL || *cpOptScale == sIdentityScale))
  {
      return SM_SUCCESS;
  }

  Notify(SM_NO_PRE_EDIT, this, SM_NO_GET_OWNER(this), NULL) ;

  // compounding volume transform 
  SER_MSG(m_pVolume->Transform(crRotateNMove, cpOptScale),
          _T("CompoundingVolume failed to be Transformed - Ask for support.")) ;

  Notify(SM_NO_POST_EDIT, this, SM_NO_GET_OWNER(this), NULL) ;                       
  
  // all done  
  return SM_SUCCESS;

} // end SmSrfInVolume::Transform

/*******************************************************************//**
PURPOSE: Trim the SmSrfInVolume BaseSurface to a domain when possible.

NOTES: 
  This function preserves the input surface geometry and its
  parameterization at the domain corners exactly, however the surface's 
  parameterization between domain corners may vary slightly but by
  amounts easily larger than reasonable tolerance sizes.  

  As such, existing Edgeuse->UVTrimCurves that reference the surface
  being trimmed should be deleted and rebuilt after this call.
***********************************************************************/
SmStatus SmSrfInVolume::TrimWithDomain
  (SmExtent2d & crTrimDomain)
{
  // check input - Can not trim an unowned surface
  SER_MSG(OwnsSurface() ? SM_SUCCESS : SM_ERR,
          _T("Can not Trim when SmSrfInVolume does not own the surface")) ;

  // base surface trim
  Notify(SM_NO_PRE_EDIT, this, SM_NO_GET_OWNER(this), NULL) ;

  SER(m_pSurface->TrimWithDomain(crTrimDomain)) ;

  Notify(SM_NO_POST_EDIT, this, SM_NO_GET_OWNER(this), NULL) ;                       
  
  // all done      
  return SM_SUCCESS;

} // end SmSrfInVolume::TrimWithDomain

/*******************************************************************//**
PURPOSE: Compute the total size of the memory used by the SmSrfInVolume.

NOTES: Does not add in attribute memory
***********************************************************************/
ULONG SmSrfInVolume::GetMemoryUsed  // rtn: smaller size of actually used memory in bytes
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

  // this + surface memory
  ULONG lThisAllocated = 0, lThisAttribAllocated = 0 ;

  ULONG lMemUsed = 0;
  if(m_pSurface)
  {
    lMemUsed = m_pSurface->GetMemoryUsed( lThisAllocated );
  }

  ULONG lUsed        =    sizeof(*this) 
                        + lMemUsed
                        + this->GetAttributeMemoryUsed(lThisAttribAllocated, 
                                                       eMarkType) ;  // note: uses without increment eMarkType value

  rlMemoryAllocated  =   sizeof(*this) + lThisAllocated + lThisAttribAllocated;

  // volume
  lThisAllocated     = 0 ;
  lUsed             += m_pVolume ? m_pVolume->GetMemoryUsed(lThisAllocated) : 0 ;
  rlMemoryAllocated += lThisAllocated ;

  // + cache memory
  if ( m_pCacheObj )
  {
      lUsed += m_pCacheObj->GetMemoryUsed( lThisAllocated );
      rlMemoryAllocated += lThisAllocated;
  }

  // all done
  return(lUsed) ;

} // end SmSrfInVolume::GetMemoryUsed

/*******************************************************************//**
PURPOSE: Add SmSrfInVolume Surface graphics to new or open drawList 
            added to global drawList array.

NOTES:
***********************************************************************/
SmDisplayList * SmSrfInVolume::Draw
 (SmBoolean       bAddToUIPickList,
  SmGfxArraySet * pOptGfxSet)
 const
//      
//      
//      SmDisplayList * SmSrfInVolume::Draw
//       (SmBoolean       bAddToUIPickList,    // in : TRUE = Add this surface to UI pick interface for debugging        
//        SmGfxArraySet * pOptGfxSet)           // i/o: used for OpenGL ES 2.0. When given output GfxVertexArrays not GL calls.
//                                             //      NULL to ignore, default:[NULL]                                         
//       const
{
  SmDisplayList *pRtn = NULL ;

#ifdef SM_GFX_OUTPUT_CODE
  // start DisplayList (unless one is already open)
  SmVector3d sColor = smgfx_GetOutputColor(pOptGfxSet) ;
  smgfx_Open(smgfx_GetRuleColor(this), NULL, NULL, FALSE, pOptGfxSet);

  // Draw the Surface
  pRtn = SmSurface::Draw(bAddToUIPickList, pOptGfxSet) ;

  // Draw the BaseSurface in BaseSurfaceColor
  SmVector3d sColor1 = smgfx_SetColor(smgfx_GetRuleColor(this, SM_CR_BASESURFACE), pOptGfxSet) ;
  if(m_pSurface) m_pSurface->SmSurface::Draw(FALSE, pOptGfxSet) ;
  smgfx_SetColor(sColor1, pOptGfxSet) ; 

  // close displayList
  smgfx_OutputColor(sColor, pOptGfxSet) ;
  pRtn = smgfx_Close(pOptGfxSet) ;

#else
  SM_REF2(bAddToUIPickList, pOptGfxSet);
#endif // end SM_GFX_CODE

  return(pRtn) ;

} // end SmSrfInVolume::Draw

// Unused function for now
#if 0
/*******************************************************************//**
PURPOSE:  Create and display a UV Line projected through a surface
          and then projected through a volume

NOTES: When crUVStart and crUVEnd are lined up horizontally or
  vertically, an isoParameter surface is built, displayed, and deleted.
  Else, a SmSrfOnSurf is built and displayed.
***********************************************************************/
static SmStatus sm_DrawSrfInVolumeUVCurve
  (const SmDisplayParameters & crDisp,        // in : Current display parameters
   const SmSurface           * pSurface,      // in : target surface
   SmBoolean                   bInParamSpace, // in : TRUE = Surface in Volume's ParamSpace, FALSE=Surface in InSpace
   const SmVolume            * pVolume,       // in : compounding volume
   const SmPoint2d           & crUVStart,     // in : Curve start point
   const SmPoint2d           & crUVEnd,       // in : if Curve end point.x == startPoint.x draw SM_SP_U IsoParam
                                              //         else draw SM_SP_V IsoParam
   const SmExtent2d          * pOptDomain,    // in : domain limit for UVTrimCurves
                                              //      NULL = use Surface->NaturalUVDomain
   SmGfxArraySet             * pOptGfxSet)    // i/o: used for OpenGL ES 2.0. When given output GfxVertexArrays not GL calls.
                                              //      NULL to ignore, default:[NULL]
{
  SmContext sContext ;
  const SmContext *cpContext = pSurface->GetContext() ? pSurface->GetContext() : &sContext ;

  //      SmContext sContext ;
  //      const SmContext *pContext = pSurface->GetContext() ? pSurface->GetContext() : &sContext ;

  SmBSplineCurve *pBSC = NULL ;

  // constant u isoparameter curve
  if (SM_ARE_SAME(crUVStart.x, crUVEnd.x))
    {
      SER(pSurface->CreateIsoParametricCurve(*cpContext, SM_SP_U,
                                              crUVStart.x, 0.0, 
                                              pBSC, pOptDomain ));

      SmCrvInVolume sCrvInVolume(*pBSC, bInParamSpace, (SmVolume &)*pVolume, 0, 1) ;
      sCrvInVolume.Draw(FALSE, FALSE, NULL, pOptGfxSet) ;
    }

  // constant v isoparameter curve
  else if(SM_ARE_SAME(crUVStart.y, crUVEnd.y))
    {
      SER(pSurface->CreateIsoParametricCurve(*cpContext, SM_SP_V,
                                              crUVStart.y, 0.0, 
                                              pBSC, pOptDomain ));

      SmCrvInVolume sCrvInVolume(*pBSC, bInParamSpace, (SmVolume &)*pVolume, 0, 1) ;
      sCrvInVolume.Draw(FALSE, FALSE, NULL, pOptGfxSet) ;
    }

  else // a parametric line
    {
      SmVector3d    sVec(crUVEnd - crUVStart) ;
      SmLine        sLine(SmPoint3d(crUVStart),
                          SmVector3d(crUVEnd - crUVStart),
                          SmExtent1d(0,1),
                          2) ;
      SmCrvOnSurf   sProjCrv    (sLine, *(SmSurface *)pSurface) ;
      SmCrvInVolume sCrvInVolume(*pBSC, bInParamSpace, (SmVolume &)*pVolume, 0, 0) ;
      sCrvInVolume.Draw(FALSE, FALSE, NULL, pOptGfxSet) ;
    }

  // all done
  return SM_SUCCESS;

} // end static SmStatus sm_DrawSrfInVolumeUVCurve
#endif

/*******************************************************************//**
PURPOSE: Add SmSrfInVolume Surface graphics to new DisplayList added to
            global displayList array using

              sDisp.m_bDrawCrossHatch   = TRUE ;        
              sDisp.m_lCrossHatchUCount = lNumBetweenU ;
              sDisp.m_lCrossHatchVCount = lNumBetweenV ;
              sDisp.m_bVaryCrossHatchColor = Draw U Lines in ObjectColor - Draw V lines in m_VaryCrossHatchColor  

NOTES: 
   special case: when sDisp.m_lCrossHatchUCount == 999
                  and sDisp.m_lCrossHatchVCount == 999,
                 then draw surface as 4 corners connected by line segments.

                 This is used to draw the surface cache bezier surface fragments.
***********************************************************************/
SmDisplayList * SmSrfInVolume::DrawUV      
  (ULONG              lNumBetweenU,         // in : number of U IsoParameter lines between knots
                                            //      default:[8]
   ULONG              lNumBetweenV,         // in : number of V IsoParameter lines between knots
                                            //      default:[8]
   SmBoolean          bVaryCrossHatchColor, // in : TRUE = Draw U Lines in ObjectColor
                                            //             Draw V lines in m_VaryCrossHatchColor
                                            //      FALSE= Draw both U and V Lines in ObjectColor
                                            //      default:[FALSE]
   const SmExtent2d * pOptUVDomain,         // in : UVDomain to crossHatch, NULL=use NaturalUVDomain
                                            //      default:[NULL]
   SmBoolean          bAddToUIPickList,     // in : TRUE=Add to UI pick list, FALSE=don't
                                            //      default:[FALSE]
   SmGfxArraySet    * pOptGfxSet)            // i/o: used for OpenGL ES 2.0. When given output GfxVertexArrays not GL calls.
                                            //      NULL to ignore, default:[NULL]
  const
{
  // trial - pass the call along
  return(SmSurface::DrawUV(lNumBetweenU, lNumBetweenV, bVaryCrossHatchColor, pOptUVDomain, bAddToUIPickList, pOptGfxSet)) ;
//      
//      #ifdef SM_GFX_OUTPUT_CODE
//      
//        // locals
//        const SmDisplayParameters &crDisp = smgfx_RefGlobalDisplayParameters() ;
//        SmSurface *pSurface               = m_pSurface ; 
//        if(bAddToUIPickList) { sm_GraphicsAddToBrepList(pSurface) ;
//                             }
//        
//        // start DisplayList (unless one is already open)
//        SmVector3d sColor = smgfx_GetOutputColor(pOptGfxSet) ;
//        smgfx_Open(smgfx_GetRuleColor(this, crDisp.GetShadedColorRule()), NULL, NULL, FALSE, pOptGfxSet);
//      
//        // Draw base surface CrossHatch U and V IsoParameter Curves  projected through volume
//        if (crDisp.m_bDrawCrossHatch)
//          {
//            double     dLineWidth = smgfx_OutputLineWidth(crDisp.m_dCrossHatchLineWidth, pOptGfxSet);
//            SmVector3d sColor     = smgfx_GetOutputColor(pOptGfxSet) ;
//      
//            // Select number of between knot isoParameter curves to show
//            ULONG dNumBetU, dNumBetV ;
//      
//            // this is a bug to work out later brought forward from previous
//            // behavior - it creates a conflict between crosshatch and polygon drawing
//            if(   crDisp.m_bDrawKnots
//               || crDisp.m_bDrawPolygon
//               || crDisp.m_bDrawControlPoints) { dNumBetU = 0;
//                                                 dNumBetV = 0;
//                                               }
//            else                               { dNumBetU = crDisp.m_lCrossHatchUCount ;
//                                                 dNumBetV = crDisp.m_lCrossHatchVCount ;
//                                               }
//      
//            // get U and V knot vectors
//            SmTArray<double> sUKnots;
//            SmTArray<double> sVKnots;
//            pSurface->GetKnots(SM_SP_U,sUKnots);
//            pSurface->GetKnots(SM_SP_V,sVKnots);
//            SmExtent1d sUIvl(sUKnots[0],sUKnots.GetLast());
//            SmExtent1d sVIvl(sVKnots[0],sVKnots.GetLast());
//            SmExtent2d sUVDomain = GetNaturalUVDomain() ;
//      
//            // when given a subDomain - trim to it when possible
//            if(pOptUVDomain && !sUVDomain.AreDisjoint(*pOptUVDomain))
//              {
//                // crosshatch FaceDomain/pOptDomain intersection
//                sUVDomain.Intersect(*pOptUVDomain, sUVDomain) ;
//                sUIvl.SetMinMax(smos_Max(sUVDomain.GetMin().x,sUKnots[0]),
//                                smos_Min(sUVDomain.GetMax().x,sUKnots.GetLast())) ;
//                sVIvl.SetMinMax(smos_Max(sUVDomain.GetMin().y,sVKnots[0]),
//                                smos_Min(sUVDomain.GetMax().y,sVKnots.GetLast())) ;
//              }
//      
//            // Make sure we draw the domain boundaries.
//            // These usually are knots, and are drawn as such, but not always.
//            SmBoolean bDidMin = FALSE, bDidMax = FALSE;
//      
//            // for every V knot
//            ULONG i;
//            for (i=0; i<sVKnots.GetSize(); i++)
//              {
//                // skip some V knots
//                if (   crDisp.m_bDrawKnots   == 0
//                    && (   crDisp.m_bDrawPolygon == 1
//                        || crDisp.m_bDrawControlPoints == 1)
//                    && (   i > 0
//                        && i < sVKnots.GetSize()-1)) 
//                  { continue; }
//      
//                // draw V isoParameter line
//                double dV = sVKnots[i];
//                if(!sVIvl.ContainsValue(dV, SM_EFF_ZERO))
//                  { continue ; }
//      
//                SmPoint2d sStart(sUIvl.GetMin(),dV);
//                SmPoint2d sEnd  (sUIvl.GetMax(),dV);
//                smgfx_OutputLineWidth(crDisp.m_dCrossHatchKnotLineWidth, pOptGfxSet);
//      
//                if ( pOptUVDomain == NULL || sVIvl.ContainsValue(dV) )
//                  {
//                    sm_DrawSrfInVolumeUVCurve( crDisp, pSurface, m_bInParamSpace, m_pVolume, sStart, sEnd, &sUVDomain, pOptGfxSet );
//      
//                    if ( smos_Fabs( dV - sVIvl.GetMin() ) < SM_EFF_ZERO_SQRT ) { bDidMin = TRUE; }
//                    if ( smos_Fabs( dV - sVIvl.GetMax() ) < SM_EFF_ZERO_SQRT ) { bDidMax = TRUE; }
//                  }
//      
//                // draw between V knot isoParameter curves
//                smgfx_OutputLineWidth(crDisp.m_dCrossHatchLineWidth, pOptGfxSet);
//                if (   i > 0
//                    && sVKnots.GetSize() < dNumBetV*3)
//                  {
//                   SmExtent1d sTmpVIvl(sVKnots[i-1],sVKnots[i]);
//                    for (ULONG j=1; j<= dNumBetV; j++)
//                      {
//                        double dV = sTmpVIvl.Evaluate(((double)j)/(dNumBetV+1));
//                        if(!sVIvl.ContainsValue(dV, SM_EFF_ZERO))
//                          { continue ; }
//                        SmPoint2d sStart(sUIvl.GetMin(),dV);
//                        SmPoint2d sEnd  (sUIvl.GetMax(),dV);
//      
//                        if(pOptUVDomain == NULL || sVIvl.ContainsValue(dV))
//                          { sm_DrawSrfInVolumeUVCurve(crDisp, pSurface, m_bInParamSpace, m_pVolume, sStart, sEnd, &sUVDomain, pOptGfxSet); }
//      
//                      } // end every inbetween knot curve
//                  } // end need inbetween knot curves check
//              } // end iter every V knot
//      
//            // Check domain min and max.
//            smgfx_OutputLineWidth(crDisp.m_dCrossHatchKnotLineWidth, pOptGfxSet);
//            if ( ! bDidMin )
//              {
//                SmPoint2d sStart( sUIvl.GetMin(), sVIvl.GetMin() );
//                SmPoint2d sEnd  ( sUIvl.GetMax(), sVIvl.GetMin() );
//                sm_DrawSrfInVolumeUVCurve( crDisp, pSurface, m_bInParamSpace, m_pVolume, sStart, sEnd, &sUVDomain, pOptGfxSet );
//              }
//            if ( ! bDidMax )
//              {
//                SmPoint2d sStart( sUIvl.GetMin(), sVIvl.GetMax() );
//                SmPoint2d sEnd  ( sUIvl.GetMax(), sVIvl.GetMax() );
//                sm_DrawSrfInVolumeUVCurve( crDisp, pSurface, m_bInParamSpace, m_pVolume, sStart, sEnd, &sUVDomain, pOptGfxSet );
//              }
//      
//            bDidMin = bDidMax = FALSE;
//      
//            // vary the constant U isoparameter lines when asked
//            if(crDisp.m_bVaryCrossHatchColor)
//              {
//                smgfx_OutputObjectColor(this, SM_CR_VARYCROSSHATCH, pOptGfxSet) ;
//              }
//      
//           // for every U knot
//           for (i=0; i<sUKnots.GetSize(); i++)
//             {
//               // skip some knots
//               if (   crDisp.m_bDrawKnots == 0
//                   && (   crDisp.m_bDrawPolygon       == 1
//                       || crDisp.m_bDrawControlPoints == 1)
//                   && (   i > 0
//                       && i < sUKnots.GetSize()-1)) { continue; }
//      
//               // draw U knot isoParameter curve
//               double dU = sUKnots[i];
//               if(!sUIvl.ContainsValue(dU, SM_EFF_ZERO))
//                 { continue ; }
//               SmPoint2d sStart(dU,sVIvl.GetMin());
//               SmPoint2d sEnd  (dU,sVIvl.GetMax());
//               smgfx_OutputLineWidth(crDisp.m_dCrossHatchKnotLineWidth, pOptGfxSet);
//      
//               if(pOptUVDomain == NULL || sUIvl.ContainsValue(dU))
//                 {
//                   sm_DrawSrfInVolumeUVCurve( crDisp, pSurface, m_bInParamSpace, m_pVolume, sStart, sEnd, &sUVDomain, pOptGfxSet );
//      
//                    if ( smos_Fabs( dU - sUIvl.GetMin() ) < SM_EFF_ZERO_SQRT ) { bDidMin = TRUE; }
//                    if ( smos_Fabs( dU - sUIvl.GetMax() ) < SM_EFF_ZERO_SQRT ) { bDidMax = TRUE; }
//                 }
//      
//               // draw between U knot isoParameter curves
//               smgfx_OutputLineWidth(crDisp.m_dCrossHatchLineWidth, pOptGfxSet);
//               if (   i > 0
//                   && sUKnots.GetSize() < dNumBetU*3)
//                 {
//                   SmExtent1d sTmpUIvl(sUKnots[i-1],sUKnots[i]);
//                   for (ULONG j=1; j<= dNumBetU; j++)
//                     {
//                       double dU = sTmpUIvl.Evaluate(((double)j)/(dNumBetU+1));
//                       if(!sUIvl.ContainsValue(dU, SM_EFF_ZERO))
//                         { continue ; }
//                       SmPoint2d sStart(dU,sVIvl.GetMin());
//                       SmPoint2d sEnd  (dU,sVIvl.GetMax());
//      
//                       if(pOptUVDomain == NULL || sUIvl.ContainsValue(dU))
//                         { sm_DrawSrfInVolumeUVCurve(crDisp, pSurface, m_bInParamSpace, m_pVolume, sStart, sEnd, &sUVDomain, pOptGfxSet); }
//      
//                     } // end iter every between U knot isoParameter curve
//                  } // end need to draw between knot curves check
//              } // end iter every U knot
//      
//            // Check domain min and max.
//            smgfx_OutputLineWidth(crDisp.m_dCrossHatchKnotLineWidth, pOptGfxSet);
//            if ( ! bDidMin )
//              {
//                SmPoint2d sStart( sUIvl.GetMin(), sVIvl.GetMin() );
//                SmPoint2d sEnd  ( sUIvl.GetMin(), sVIvl.GetMax() );
//                sm_DrawSrfInVolumeUVCurve( crDisp, pSurface, m_bInParamSpace, m_pVolume, sStart, sEnd, &sUVDomain, pOptGfxSet );
//              }
//            if ( ! bDidMax )
//              {
//                SmPoint2d sStart( sUIvl.GetMin(), sVIvl.GetMin() );
//                SmPoint2d sEnd  ( sUIvl.GetMin(), sVIvl.GetMax() );
//                sm_DrawSrfInVolumeUVCurve( crDisp, pSurface, m_bInParamSpace, m_pVolume, sStart, sEnd, &sUVDomain, pOptGfxSet );
//              }
//      
//            // vary the constant U isoparameter lines when asked
//            if(crDisp.m_bVaryCrossHatchColor) { smgfx_OutputColor(sColor, pOptGfxSet) ; }
//            smgfx_OutputLineWidth(dLineWidth, pOptGfxSet) ;
//            smgfx_SetColor(sColor, pOptGfxSet) ; 
//      
//          } // end Draw Surface IsoParameter crossHatch lines
//      
//        // close displayList
//        smgfx_OutputColor(sColor, pOptGfxSet) ;
//        pRtn = smgfx_Close(pOptGfxSet) ;
//      
//      #endif // end SM_GFX_CODE
//      
//        return(pRtn) ;
//      
} // end SmSrfInVolume::DrawUV

/*******************************************************************//**
PURPOSE: Write SmSrfInVolume to given output stream.

NOTES: 
***********************************************************************/
SmStatus SmSrfInVolume::WriteToDB
 (SmDatabaseIO & rDB,       // in : target output stream
  ULONG lDBVersionNumber)   // in : database version to get proper sequence of writes                                                                       
 const
{
  // file type, ASCII or BINARY
  SmFileType       eType    =  rDB.GetFileType() ;
  std::ostream   & rFileOut = *rDB.GetOutStreamPtr() ;
      
  // base surface
  if (eType == SM_ASCII) { rFileOut << " SrfInVolume->BaseSurface \n"; }
  SER(rDB.WriteType(m_pSurface->GetType())) ; 
  SER(m_pSurface->WriteToDB(rDB, lDBVersionNumber)) ;

  // bInParamSpace value
  rFileOut << m_bInParamSpace << " InParamSpace: 0=Curve in Volume's ParamSpace, 1=Curve in InSpace\n" ; 

  // compounding volume
  if (eType == SM_ASCII) { rFileOut << " SrfInVolume->Volume \n"; }
  SER(rDB.WriteType(m_pVolume->GetType())) ; 
  SER(m_pVolume->WriteToDB(rDB, lDBVersionNumber)) ;

  // all done
  return SM_SUCCESS;

} // end SmSrfInVolume::WriteToDB

/*******************************************************************//**
PURPOSE: static method to Read a SmSrfInVolume from a given stream  

NOTES: 
***********************************************************************/
SmStatus SmSrfInVolume::ReadFromDB
 (SM_TYPE           lType,              // NotUsed: in : surface type to be read
  SmDatabaseIO    & rDB,                // in : target output stream
  const SmContext & crContext,          // in : context for new object construction
  SmSurface      *& rpNewSurface,       // out:    NULL on input = new object allocated in this routine built from stream data
                                        //      NotNULL on input = pointer to an empty object to be filled by this routine
  ULONG             lDBVersionNumber)   // in : database version to get proper sequence of writes
{
  SM_REF1(lType) ;
  // check input
  SER(  (   rpNewSurface == NULL
         || rpNewSurface->IsKindOf(SmSrfInVolume_TYPE)) ? SM_SUCCESS : SM_ERR) ;

  // init output object
  SmSrfInVolume *pSrfInVolume =   (rpNewSurface == NULL)
                                    ? new (crContext) SmSrfInVolume()
                                    : (SmSrfInVolume *)rpNewSurface ;

  // file type
  SmFileType      eType   =  rDB.GetFileType() ;
  std::istream  & rFileIn = *rDB.GetInStreamPtr() ;
      
  // surface locals
  SmSurface *pBaseSurface=NULL ;
  SM_TYPE    lBaseType ;  

  // base surface
  if (eType == SM_ASCII) { rDB.GoToNextLine() ; }
  SER(rDB.ReadType(lBaseType)) ; 
  SER(SmSurface::ReadFromDB(lBaseType, rDB, crContext, pBaseSurface, lDBVersionNumber)) ; 

  // bInParamSpace value
  rFileIn >> pSrfInVolume->m_bInParamSpace ; rDB.GoToNextLine() ;

  // volume locals
  SmVolume  *pVolume=NULL ;
  SM_TYPE    lVolType  ;  

  // volume
  if (eType == SM_ASCII) { rDB.GoToNextLine() ; }
  SER(rDB.ReadType(lVolType)) ; 
  SER(SmVolume::ReadFromDB(lVolType, rDB, crContext, pVolume, lDBVersionNumber)) ; 

  // load the object
  pSrfInVolume->m_lOwnerFlag       = 3 ;
  pSrfInVolume->m_pSurface         = pBaseSurface ;
  pSrfInVolume->m_pVolume          = pVolume ; 

  // all done
  rpNewSurface = pSrfInVolume ;
  return SM_SUCCESS;

} // end SmSrfInVolume::ReadFromDB

/*******************************************************************//**
PURPOSE:
NOTES:
***********************************************************************/
SmBoolean SmSrfInVolume::IsKindOf( SM_TYPE t ) const
{
  return ((SmSrfInVolume_TYPE == t) ? TRUE : SmSurface::IsKindOf( (t) ));
}

/*******************************************************************//**
PURPOSE:

NOTES: 
***********************************************************************/
void SmSrfInVolume::Dump() const
{
  TCHAR sBuff[SM_TBLOCK_SIZE], sBuffForFile[SM_TBLOCK_SIZE];
  smos_WriteBuffer(_T("\nBegin SmSrfInVolume::Dump()")) ;

  smos_sprintf(sBuff,       
             _T("\nSmSrfInVolume = [0x%p], BaseSurface = [0x%p], SurfaceIn [%sSpace], CompoundingVolume = [0x%p]"),
             this, 
             m_pSurface, 
             m_bInParamSpace ? _T("Param") : _T("In"),
             m_pVolume) ;
  smos_sprintf(sBuffForFile,
             _T("\nSmSrfInVolume = %s, BaseSurface = %s, SurfaceIn [%sSpace], CompoundingVolume = %s"),
             _T("notNULL"),
             m_pSurface      ? _T("notNULL") : _T("NULL"), 
             m_bInParamSpace ? _T("Param")   : _T("In"),
             m_pVolume       ? _T("notNULL") : _T("NULL")) ;
  smos_WriteBuffer(sBuff, sBuffForFile) ;

  // m_lOwnerFlag 
  smos_sprintf( sBuff, (_T(", OwnerFlag = [%ld]")), m_lOwnerFlag);
  smos_WriteBuffer(sBuff);
  smos_WriteBuffer(_T("\n  OwnerFlag: 0 = deletes nothing when destructed ")); 
  smos_WriteBuffer(_T("\n             1 = deletes only surface when destructed ")); 
  smos_WriteBuffer(_T("\n             2 = deletes only volume when destructed ")); 
  smos_WriteBuffer(_T("\n             3 = deletes surface and volume when destructed ")); 

  // report Cache data
  smos_WriteBuffer(_T("\nBegin SmSrfInVolume Base Class SmSurface Dump "));
  SmSurface::Dump(FALSE) ;
  smos_WriteBuffer(_T("End SmSrfInVolume Base Class SmSurface Dump "));

  smos_WriteBuffer(_T("\n\nBegin contained SmSrfInVolume->BaseSurface Dump")) ;
  m_pSurface->Dump() ;
  smos_WriteBuffer(_T("End contained SmSrfInVolume->BaseSurface Dump\n")) ;

  smos_WriteBuffer(_T("\n\nBegin contained SmSrfInVolume->Volume Dump")) ;
  m_pVolume->Dump() ;
  smos_WriteBuffer(_T("End contained SmSrfInVolume->Volume Dump\n")) ;

  smos_WriteBuffer(_T("End SmSrfInVolume::Dump()\n")) ;

} // end SmSrfInVolume::Dump

/*******************************************************************//**
PURPOSE: AssertValid() Test reporting static labels
***********************************************************************/
SmAssertReportLabel sAssertSrfInVolume_list[] =
{
  /*  0 */ {SM_AT_NESTED_TEST, _T("Base Surface"),       _T("Base Surface must pass AssertValid") },
  /*  1 */ {SM_AT_NESTED_TEST, _T("Compounding Volume"), _T("Compounding Volume must pass AssertValid") },
  /*  2 */ {SM_AT_POINTER,  _T("Bad BaseSurface Owner"), _T("When owned, BaseSurface owner must be this SmSrfInVolume object") },
  /*  3 */ {SM_AT_POINTER,  _T("Bad BaseVolume Owner"),  _T("When owned, BaseVolume owner must be this SmSrtInVolume object") },
  /*  4 */ {SM_AT_POINTER,  _T("Bad BaseSurface Context"), _T("When owned, BaseSurface context must be this SmSrfInVolume context") },
  /*  5 */ {SM_AT_POINTER,  _T("Bad BaseVolume Context"),  _T("When owned, BaseVolume context must be this SmSrtInVolume context") }
} ;

/*******************************************************************//**
PURPOSE:

NOTES: returns TRUE  = OK
                        FALSE = Problem
***********************************************************************/
SmBoolean SmSrfInVolume::AssertValid
 (SmAssertArray    * pAList,        // i/o: Accumulating list of failed Asserts, NULL to ignore, default:[NULL] 
  SmAssertTestLevel  eTestLevel,    // in : SM_LEVEL_0=fewest tests, SM_LEVEL_1, SM_LEVEL_2=all tests, 
                                    //      SM_LEVEL_GIVEN = run tests in order requested in pTestRequests 
                                    //      default:[SM_LEVEL_0] 
  SmAssertWalking    eWalkTree,     // in : SM_WALK = run AssertValid on any topology graph descendants, SM_NO_WALK=don't, default:[SM_WALK] 
  SmTArray<ULONG>  * pTestRequests) // in : when eTestLevel == SM_LEVEL_GIVEN, run these tests in this order, default:[NULL]
 const
{
  /*  0 */ // test contained surface
  SmBoolean bRtn = SM_ASSERT_BOOLEAN_REPORT(0, SM_LEVEL_0, m_pSurface->AssertValid(pAList, eTestLevel, eWalkTree, pTestRequests), _T("")) ;

  /*  1 */ // test contained volume
  bRtn &= SM_ASSERT_BOOLEAN_REPORT(1, SM_LEVEL_0, m_pVolume->AssertValid(pAList, eTestLevel, eWalkTree, pTestRequests), _T("")) ;

  /*  2 */ // Bad BaseSurface Owner
  bRtn &= SM_ASSERT_BOOLEAN_REPORT(2, SM_LEVEL_0, (m_lOwnerFlag & 1) && m_pSurface->GetOwner() == this, _T("")) ;

  /*  3 */ // Bad BaseVolume Owner 
  bRtn &= SM_ASSERT_BOOLEAN_REPORT(3, SM_LEVEL_0, (m_lOwnerFlag & 2) && m_pVolume->GetOwner() == this, _T("")) ;

  /*  2 */ // Bad BaseSurface Context
  bRtn &= SM_ASSERT_BOOLEAN_REPORT(4, SM_LEVEL_0, (m_lOwnerFlag & 1) && m_pSurface->GetContext() == m_cpContext, _T("")) ;

  /*  3 */ // Bad BaseVolume Context 
  bRtn &= SM_ASSERT_BOOLEAN_REPORT(5, SM_LEVEL_0, (m_lOwnerFlag & 2) && m_pVolume->GetContext() == m_cpContext, _T("")) ;
  // all done
  return(bRtn) ;

} // end SmSrfInVolume::AssertValid

// obsolete
// /*******************************************************************//**
// PURPOSE: Fix Assert Report failures reported by an AssertValid call
// 
// RETURNS ---  TRUE  = OK
//              FALSE = Problem
// ***********************************************************************/
// SmBoolean SmSrfInVolume::AssertHeal
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
//       return ( SmSurface::AssertHeal(rAReport, pAList) ) ;
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
//                rAReport.m_pHealMessage = _T("SmSrfInVolume::AssertHeal fix not yet supported") ;  
//                 
//     }
// 
//   // all done
//   return(bRtn) ;
// 
// } // end SmSrfInVolume::AssertHeal
// end obsolete

