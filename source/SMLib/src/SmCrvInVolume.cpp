// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************//**
* FILE NAME --- SmCrvInVolume.cpp
* PURPOSE: Source file for SmCrvInVolume methods.
**********************************************************************/

#include "StdAfx.h"
#include <SmCrvInVolume.h>

/*******************************************************************//**
PURPOSE: Constructor for Curve in Volume objects.  

NOTES: It loads values and checks to make sure that curve is valid.

  Never let an rCurve or rVolume object be owned by two objects.

  When (lOwnerFlag&1) rCurve->m_pOwner set to this.  
  If rCurve is owned on input by a Edge (or any other object), the
  caller must make sure to fix that parent's now obsolete pointer.
  For example, When rCurve->OldOwner is a Edge, a call to 
  pEdge->ReplaceCurve fixes the problem.
***********************************************************************/
SmCrvInVolume::SmCrvInVolume
  (SmCurve          & rCurve,        // in : projected Curve   - When(lOwnerFlag&1) rCurve->m_pOwner = this
   SmBoolean          bInParamSpace, // in : TRUE = rCurve is in rVolume's ParamSpace
                                     //      FALSE= rCurve is in rVolume's InSpace
   SmVolume         & rVolume,       // in : projecting Volume - When(lOwnerFlag&2) rVolume->m_pOwner = this
   ULONG              lCopyFlag,     // in : 0 = saves curve and volume without copying
                                     //      1 = copy curve and save volume orig
                                     //      2 = copy volume and save curve orig
                                     //      3 = copy both curve and volume
                                     //      default:[0]
   ULONG              lOwnerFlag,    // in : 0 = deletes nothing when destructed
                                     //      1 = delete curve but not volume when destructed
                                     //      2 = delete volume but not curve when destructed
                                     //      3 = delete both curve and volume when destructed
                                     //      default:[0]
   const SmContext  * cpContext)     // in : req for new stack objs, opt for new heap objs
                                     // in : default:[NULL]
                         
 : SmCurve(3), 
   m_pCurve(&rCurve), 
   m_bInParamSpace(bInParamSpace),
   m_pVolume(&rVolume),
   m_bNeedBreaks(TRUE),
   m_lOwnerFlag(lOwnerFlag)
{
  // when passed a context - use it
  if(cpContext) 
    {  
      SM_ASSERT(GetContext() == NULL || GetContext() == cpContext) ;
      SetContext(cpContext) ; 
    }
                
  // When input Context is NULL, set from Curve or Volume context when possible
  if(GetContext() == NULL && rVolume.GetContext() != NULL)
    { SetContext(rVolume.GetContext()) ; }

  if(GetContext() == NULL && rCurve.GetContext() != NULL)
    { SetContext(rCurve.GetContext()) ; }

  // When Curve is to be copied - copy it
  if (   lCopyFlag == 1 
      || lCopyFlag == 3) 
    {
      const SmContext * pContext = GetContext();
      if (pContext == NULL) { SE(SM_ERR); return; }
      m_pCurve->Copy(*pContext, m_pCurve) ;
      m_lOwnerFlag |= 1 ; 
    }

  // when Volume is to be copied - copy it
  if (   lCopyFlag == 2
      || lCopyFlag == 3) 
    {
      const SmContext * pContext = GetContext();
      if (pContext == NULL) { SE(SM_ERR); return; }
      m_pVolume->Copy(*pContext, m_pVolume) ;
      m_lOwnerFlag |= 2 ;
    }

  // set contained Curve and Volume owner values
  if(m_lOwnerFlag & 1) { m_pCurve->SetOwner(this) ; }
  if(m_lOwnerFlag & 2) { m_pVolume->SetOwner(this) ; }

  // try to make sure all objects are using the same context
  if(m_pCurve->GetContext()  == NULL) { m_pCurve->SetContext(GetContext()) ; }
  if(m_pVolume->GetContext() == NULL) { m_pVolume->SetContext(GetContext()) ; }

} // end SmCrvInVolume::SmCrvInVolume constructor

/*******************************************************************//**
PURPOSE: Copy constructor for SmCrvInVolume object.

NOTES: The constructed object will make deep copies of both the target
  curve's m_pCurve and m_pVolume when ever the target curve being copied
  owns either.  When the target curve just has an unowned reference to either
  then an unowned reference will be placed in the constructed object.
***********************************************************************/
SmCrvInVolume::SmCrvInVolume
  (const SmCrvInVolume & crCurveToCopy)
 : SmCurve(crCurveToCopy), 
   m_pCurve(crCurveToCopy.m_pCurve),
   m_bInParamSpace(crCurveToCopy.m_bInParamSpace),
   m_pVolume(crCurveToCopy.m_pVolume), 
   m_bNeedBreaks(crCurveToCopy.m_bNeedBreaks),
   m_lOwnerFlag(crCurveToCopy.m_lOwnerFlag) 
{
  // figure out what the target curve owns - those things will be deep copied
  SmBoolean bTargetOwnsCurve   = (   m_lOwnerFlag == 1
                                  || m_lOwnerFlag == 3) ;
  SmBoolean bTargetOwnsVolume  = (   m_lOwnerFlag == 2
                                  || m_lOwnerFlag == 3) ;   
                                  
  const SmContext *pContext =   GetContext()                          ? GetContext()
                              : crCurveToCopy.GetContext()            ? crCurveToCopy.GetContext()
                              : crCurveToCopy.m_pCurve->GetContext()  ? crCurveToCopy.m_pCurve->GetContext()
                              : crCurveToCopy.m_pVolume->GetContext() ? crCurveToCopy.m_pVolume->GetContext()
                              : NULL ;                                    
  if (pContext == NULL) 
    { SE_MSG(SM_ERR, _T("SmCrvInVolume copy constructor can't find a SmContext for new object construction")); }

  // copy Curve when its owned by this object
  if (bTargetOwnsCurve) 
    {
      // copy the Curve
      crCurveToCopy.m_pCurve->Copy(*pContext, m_pCurve) ;
    }

  // copy Volume when its owned by this object
  if (bTargetOwnsCurve || bTargetOwnsVolume) 
    {
      // copy the Volume
      crCurveToCopy.m_pVolume->Copy(*pContext, m_pVolume) ;
    }

  // set the owner flag
  m_lOwnerFlag =   (bTargetOwnsCurve  == TRUE && bTargetOwnsVolume == TRUE) ? 3
                 : (bTargetOwnsCurve  == TRUE) ? 1
                 : (bTargetOwnsVolume == TRUE) ? 2
                 : 0 ;

  // copy knots when appropriate
  if(m_bNeedBreaks == FALSE) 
    { 
      m_sBreaks = crCurveToCopy.m_sBreaks ; 
      m_sConts  = crCurveToCopy.m_sConts ; 
    }  

  // set contained Curve and Volume owner values
  if(m_lOwnerFlag & 1) { m_pCurve->SetOwner(this) ; }
  if(m_lOwnerFlag & 2) { m_pVolume->SetOwner(this) ; }

  // inform the public
  ((SmCurve &)crCurveToCopy).Notify(SM_NO_COPY, this, SM_NO_GET_OWNER(this), SM_NO_GET_OWNER(&crCurveToCopy)) ;

} // end SmCrvInVolume::SmCrvInVolume copy constructor

/*******************************************************************//**
PURPOSE: Equality operator for SmCrvInVolume

NOTES: Call base equivalence to check type and then check 
       members for equivalence
***********************************************************************/
SmBoolean SmCrvInVolume::operator==
  (const SmCurve& crOther) 
 const
{
  // low work
  if(this == &crOther) { return TRUE ; }

  // first check the base
  SmBoolean bRtn = SmCurve::operator ==(crOther) ;

  if(bRtn)
    {
      // OK to cast
      SmCrvInVolume &rOther = (SmCrvInVolume &)crOther ;

      // check equivalence of these objects
      bRtn = (   (   m_bInParamSpace == rOther.m_bInParamSpace)
              && (   ( m_pCurve == rOther.m_pCurve)
                  || ( m_pCurve == NULL && rOther.m_pCurve == NULL)
                  || (   m_pCurve != NULL && rOther.m_pCurve != NULL
                      && *m_pCurve == *rOther.m_pCurve)) 
              && (   ( m_pVolume == rOther.m_pVolume)
                  || ( m_pVolume == NULL && rOther.m_pVolume == NULL)
                  || (   m_pVolume != NULL && rOther.m_pVolume != NULL
                      && *m_pVolume == *rOther.m_pVolume)) ) ;
       // don't check cache equivalence - cache variations don't negate object equality
    }

  // all done
  return bRtn ;

} // end SmCrvInVolume::operator==

/*******************************************************************//**
PURPOSE: Copy a Curve in Volume.

NOTES: 
***********************************************************************/
SmStatus SmCrvInVolume::Copy
  (const SmContext & crContext,
   SmCurve        *& rpNewCurve) 
  const
{
  rpNewCurve = new (crContext) SmCrvInVolume(*this);
  NER(rpNewCurve);
  return SM_SUCCESS;

} // end SmCrvInVolume::Copy

/*******************************************************************//**
PURPOSE: Create a mirror curve of an SmCrvInVolume curve.

NOTES: 
***********************************************************************/
SmStatus SmCrvInVolume::CreateMirrorCurve
  (const SmContext        & crContext,        // in : context for new object 
   const SmAxis2Placement & crMirrorPlane,    // in : mirror plane
   SmCurve               *& rpMirrorCurve)    // out: mirrored curve
  //const
{
  // copy this curve
  SmCrvInVolume *pNewCurve = new (crContext) SmCrvInVolume(*this) ;
  
  // mirror the the volume
  SER(pNewCurve->m_pVolume->Mirror(crMirrorPlane)) ;

  // create the return object
  rpMirrorCurve = pNewCurve ;

  // all done
  return SM_SUCCESS;

} // end SmCrvInVolume::CreateMirrorCurve

/*******************************************************************//**
PURPOSE: Destructor for the SmCrvInVolume object.  It cleans up
    memory for the contained curve and volume if it owns them.

NOTES: 
***********************************************************************/
SmCrvInVolume::~SmCrvInVolume()
{
  // not needed - called in base class destructor
  //      // notify mechanism removes CurveCache and Attributes, and calls the UserCallback 
  //      Notify(SM_NO_DESTRUCTION, this, NULL) ; 

  // delete contained structures when appropriate
  if (m_pCurve ) { if(OwnsCurve())  { delete m_pCurve;  m_pCurve  = NULL ; } }
  if (m_pVolume) { if(OwnsVolume()) { delete m_pVolume; m_pVolume = NULL ; } }

} // end SmCrvInVolume::~SmCrvInVolume destructor

/*******************************************************************//**
PURPOSE: Edit the parameterization of this curve.

NOTES: 
***********************************************************************/
SmStatus SmCrvInVolume::EditParameterization
  (const SmExtent1d & crNewParameterization, // in : new parameter range for curve
   SmBoolean          bNotify)               // in : TRUE  = make notify calls (previous behavior)
                                             //      FALSE = Skip notify call
                                             //      default:[TRUE]
{ 
  Notify(SM_NO_PRE_EDIT, this, SM_NO_GET_OWNER(this), NULL);

  // when appropriate scale breaks
  if(m_bNeedBreaks == FALSE)
    {
      ULONG ii ;
      SmExtent1d sIvl = m_pCurve->GetNaturalInterval() ;

      if(   sIvl.GetLength() > SM_EFF_ZERO
         && crNewParameterization.GetLength() > SM_EFF_ZERO)
        {
          // for every break
          for(ii=0;ii<m_sBreaks.GetSize();ii++)
            {
              m_sBreaks[ii] =   ((m_sBreaks[ii] - sIvl.GetMin()) / sIvl.GetLength())
                              * crNewParameterization.GetLength()
                              + crNewParameterization.GetMin() ;

              // no need to update m_sConts

            } // end iter every break
        } // end nonZero intervals check
      else // working with zero length intervals
        {
          m_sBreaks.ReSet() ;
          m_sConts.ReSet() ;
          m_bNeedBreaks = TRUE ;
        }
    } // end scale breaks check

  // pass the call along to m_pCurve              
  SER(m_pCurve->EditParameterization(crNewParameterization, bNotify));
  
  // call notify on SmCrvInVolume container
  if( bNotify == TRUE )
    { 
      Notify(SM_NO_POST_EDIT, this, SM_NO_GET_OWNER(this), NULL);
    }

  // all done
  return SM_SUCCESS;

} // end SmCrvInVolume::EditParameterization

/*******************************************************************//**
PURPOSE: Given the parameteric value of the curve determine the
    corresponding Euclidian point and optionally derivatives.

NOTES: For discontinuous points on the curve the derivatives
    may be obtained either from the left or from the right of the 
    discontinuity.
***********************************************************************/
SmStatus SmCrvInVolume::Evaluate
 (double     dParameter,              // in : tgt param
  ULONG      lNumDerivatives,         // in : 0=pos, 1=pos+tang, 2=pos+tang+2nd, . . .
  SmBoolean  bFromLeft,               // in : if P is on interval boundary
                                      //      TRUE  = evaluate P in upper interval where P is on the left of the interval
                                      //      FALSE = evaluate P in lower interval where P is on the right of the interval
  SmVector3d aPointAndDerivatives[],  // out: array or (pos, tang, 2nd deriv, ...), sized:[lNumDerivatives+1]
  SmBoolean  bNonZeroTangents)        // in : TRUE = replace zero tangent vectors with properly oriented tol sized vectors
 const                                //      FALSE= return exact tangent values
                                      //      note: Surprisingly TRUE is the common choice because most tangent uses
                                      //            are for their direction (Binorm, SurfNorm comps), but when the 
                                      //            tangent is being used for its magnitude (like an arc-length comp)
                                      //            then set this to FALSE.
                                      //      default:[TRUE]
{ 
  // limit the number of derivatives to 3
  const ULONG MAX_DERIVS = 3;
  ULONG lNumDerivsToEval = (lNumDerivatives < MAX_DERIVS) ?  lNumDerivatives : MAX_DERIVS;

  // For subscripting into the volume pt/deriv array:
#define ss(u,v,w,n) ((u*(n+1)+v)*(n+1)+w)

  // Curve eval locals
  SmStatus eStat = SM_SUCCESS ;
  SmPoint3d  sCurve3dPt;
  SmVector3d aCrvPtDerivs[ 1 + MAX_DERIVS ];
  SmVector3d aVolPtDerivs[ (1+MAX_DERIVS) * (1+MAX_DERIVS) * (1+MAX_DERIVS)];

  // Curve eval - get param point
  SER( m_pCurve->Evaluate(dParameter, 
                          lNumDerivsToEval, 
                          bFromLeft,
                          aCrvPtDerivs,
                          bNonZeroTangents) ) ;

  // Volume Evaluate - from Param to compounded last OutSpace
  if(m_bInParamSpace)
    {
      eStat = m_pVolume->Evaluate(aCrvPtDerivs[0],
                                  lNumDerivsToEval, 
                                  (aCrvPtDerivs[1].x > 0.0) ? bFromLeft : !bFromLeft, 
                                  (aCrvPtDerivs[1].y > 0.0) ? bFromLeft : !bFromLeft, 
                                  (aCrvPtDerivs[1].z > 0.0) ? bFromLeft : !bFromLeft,
                                  aVolPtDerivs,
                                  bNonZeroTangents) ;
    } // end Volume Evaluate branch
  else // Volume map - from InSpace to compounded last OutSpace
    {
      eStat = m_pVolume->Map(aCrvPtDerivs[0],
                             lNumDerivsToEval, 
                             (aCrvPtDerivs[1].x > 0.0) ? bFromLeft : !bFromLeft, 
                             (aCrvPtDerivs[1].y > 0.0) ? bFromLeft : !bFromLeft, 
                             (aCrvPtDerivs[1].z > 0.0) ? bFromLeft : !bFromLeft,
                             aVolPtDerivs,
                             bNonZeroTangents) ;
    } // end Volume Map branch

  // If the volume can't handle this many derivatives:
  while ( eStat != SM_SUCCESS && lNumDerivsToEval > 0 )
    {
      lNumDerivsToEval--;

      // try a Volume Eval with fewer derivatives
      if(m_bInParamSpace)
        {
          eStat = m_pVolume->Evaluate(aCrvPtDerivs[0],
                                      lNumDerivsToEval, 
                                      (aCrvPtDerivs[1].x > 0.0) ? bFromLeft : !bFromLeft, 
                                      (aCrvPtDerivs[1].y > 0.0) ? bFromLeft : !bFromLeft, 
                                      (aCrvPtDerivs[1].z > 0.0) ? bFromLeft : !bFromLeft,
                                      aVolPtDerivs,
                                      bNonZeroTangents) ;
        } // end Volume Evaluate branch
      else // Volume map - from InSpace to compounded last OutSpace
        {
          eStat = m_pVolume->Map(aCrvPtDerivs[0],
                                 lNumDerivsToEval, 
                                 (aCrvPtDerivs[1].x > 0.0) ? bFromLeft : !bFromLeft, 
                                 (aCrvPtDerivs[1].y > 0.0) ? bFromLeft : !bFromLeft, 
                                 (aCrvPtDerivs[1].z > 0.0) ? bFromLeft : !bFromLeft,
                                 aVolPtDerivs,
                                 bNonZeroTangents) ;
        } // end Volume Map branch
    } // end while looking for volume max derivative count
  SER( eStat );

  // set outputs

  // Position
  aPointAndDerivatives[0] = aVolPtDerivs[0];

  // 1st Derivs
  if ( lNumDerivsToEval >= 1 )
    {
      aPointAndDerivatives[1] =  smvol_LiftFirstDerivative
                                   (aCrvPtDerivs[1],

                                    aVolPtDerivs[ ss(1,0,0, lNumDerivsToEval) ],
                                    aVolPtDerivs[ ss(0,1,0, lNumDerivsToEval) ],
                                    aVolPtDerivs[ ss(0,0,1, lNumDerivsToEval) ] ) ;
    }
  
  // 2nd Derivs
  if ( lNumDerivsToEval >= 2 )
    {
      aPointAndDerivatives[2] = smvol_LiftSecondDerivative
                                  ( aCrvPtDerivs[1], 
                                    aCrvPtDerivs[2],

                                    aVolPtDerivs[ ss(1,0,0, lNumDerivsToEval) ],
                                    aVolPtDerivs[ ss(0,1,0, lNumDerivsToEval) ],
                                    aVolPtDerivs[ ss(0,0,1, lNumDerivsToEval) ],

                                    aVolPtDerivs[ ss(2,0,0, lNumDerivsToEval) ],
                                    aVolPtDerivs[ ss(1,1,0, lNumDerivsToEval) ],
                                    aVolPtDerivs[ ss(1,0,1, lNumDerivsToEval) ],
                                    aVolPtDerivs[ ss(0,2,0, lNumDerivsToEval) ],
                                    aVolPtDerivs[ ss(0,1,1, lNumDerivsToEval) ],
                                    aVolPtDerivs[ ss(0,0,2, lNumDerivsToEval) ] );
    }
  
  // 3rd Derivs
  if ( lNumDerivsToEval >= 3 )
    {
      aPointAndDerivatives[3] = smvol_LiftThirdDerivative
                                  ( aCrvPtDerivs[1], 
                                    aCrvPtDerivs[2],
                                    aCrvPtDerivs[3],

                                    aVolPtDerivs[ ss(1,0,0, lNumDerivsToEval) ],
                                    aVolPtDerivs[ ss(0,1,0, lNumDerivsToEval) ],
                                    aVolPtDerivs[ ss(0,0,1, lNumDerivsToEval) ],

                                    aVolPtDerivs[ ss(2,0,0, lNumDerivsToEval) ],
                                    aVolPtDerivs[ ss(1,1,0, lNumDerivsToEval) ],
                                    aVolPtDerivs[ ss(1,0,1, lNumDerivsToEval) ],
                                    aVolPtDerivs[ ss(0,2,0, lNumDerivsToEval) ],
                                    aVolPtDerivs[ ss(0,1,1, lNumDerivsToEval) ],
                                    aVolPtDerivs[ ss(0,0,2, lNumDerivsToEval) ],
                                                       
                                    aVolPtDerivs[ ss(3,0,0, lNumDerivsToEval) ],
                                    aVolPtDerivs[ ss(2,1,0, lNumDerivsToEval) ],
                                    aVolPtDerivs[ ss(2,0,1, lNumDerivsToEval) ],
                                    aVolPtDerivs[ ss(1,2,0, lNumDerivsToEval) ],
                                    aVolPtDerivs[ ss(1,1,1, lNumDerivsToEval) ],
                                    aVolPtDerivs[ ss(1,0,2, lNumDerivsToEval) ],
                                    aVolPtDerivs[ ss(0,3,0, lNumDerivsToEval) ],
                                    aVolPtDerivs[ ss(0,2,1, lNumDerivsToEval) ],
                                    aVolPtDerivs[ ss(0,1,2, lNumDerivsToEval) ],
                                    aVolPtDerivs[ ss(0,0,3, lNumDerivsToEval) ] );
    }

  // If too many requested, fill with zeros.
  for ( ULONG ii = MAX_DERIVS+1; ii <= lNumDerivatives; ii++ )
    {
      aPointAndDerivatives[ii].Set( 0, 0, 0 );
    }

  // all done
#undef ss
  return SM_SUCCESS;

} // end SmCrvInVolume::Evaluate

/*******************************************************************//**
PURPOSE: Given the parametric value of the curve determine the
    corresponding Euclidian point on the curve.      

NOTES: This methods assumes the curve is not discontinuous
    in its position.  The parameter must be within the natural range
    of the curve.
***********************************************************************/
SmStatus SmCrvInVolume::EvaluatePoint
  (double dParameter, 
   SmPoint3d & rPoint) 
 const
{ 
  SmPoint3d sPnt ;

  // Curve eval - Get ParamSpace point
  SER(m_pCurve->EvaluatePoint(dParameter,sPnt)) ;

  // volume eval
  SER(m_pVolume->MapPoint(sPnt,rPoint)) ;

  // Volume Evaluate - from Param to compounded last OutSpace
  if(m_bInParamSpace)
    {
      SER(m_pVolume->EvaluatePoint(sPnt,rPoint)) ;

    } // end Volume Evaluate branch
  else // Volume map - from InSpace to compounded last OutSpace
    {
      SER(m_pVolume->MapPoint(sPnt,rPoint)) ;

    } // end Volume Map branch

  // all done
  return SM_SUCCESS;

} // end SmCrvInVolume::EvaluatePoint

/*******************************************************************//**
PURPOSE: Given the parameteric value of the curve determine the
    corresponding Volume point and derivatives.

NOTES: For discontinuous points on the curve the derivatives
    may be obtained either from the left or from the right of the 
    discontinuity.
***********************************************************************/
SmStatus SmCrvInVolume::EvaluateVolume
 (double      dParameter,       // in : tgt param
  ULONG       lHighestDeriv,    // in : number of derivatives, max 3            
  SmBoolean   bFromLeft,        // in : if P is on interval boundary
                                //      TRUE  = evaluate P in upper interval where P is on the left of the interval
                                //      FALSE = evaluate P in lower interval where P is on the right of the interval
  SmVector3d *aDerivatives,     // out: Volume values at Curve(dParameter)
                                //      sized:[n+1][n+1][n+1], where n=lHighesVDeriv
                                //      indexing:[1dIndex = u*(n+1)*(n+1)+v*(n+1)+w], where u,v,w=number of partial derivs 
                                //      lHighestDerivative 
                                //            0,  sized:[1]    order:[D]
                                //            1,  sized:[8]    order:[D  Dw  Dv  ---
                                //                                    Du --- --- ---]
                                //            2,  sized:[27]   order:[D   Dw   Dww   Dv   Dvw   ----   Dvv   ----   ----
                                //                                    Du  Duw  ----  Duv  ----  -----  ----  ----  -----
                                //                                    Duu ---- ----- ---- ----- ----- ----- ----- ------]
                                //            3,  sized:[64]
                                //      3d organized: D[UdirCnt][VDirCnt][WDirCnt]
                                //      1d organized: [D Dw Dww ... Dv Dvw Dvww ... Dvv Dvvw Dvvww ...
                                //                     Du Duw Duww ... Duv Duvw Duvww ... Duvv Duvvw Duvvww ...
                                //                     Duu Duuw Duuww ... Duuv Duuvw Duuvww ... Duuvv Duuvvw Duuvvww ...]
  SmVector3d *pOptBaseCurvePV,  // out: BaseCurve Pos, 1stDeriv, 2ndDeriv, 3rdDeriv before being projected through VolumeMap,
                                //      sized:[lHighestDeriv+1]
  SmBoolean   bNonZeroTangents) // in : TRUE = replace zero tangent vectors with properly oriented tol sized vectors
          const                 //      FALSE= return exact tangent values
                                //      note: Surprisingly TRUE is the common choice because most tangent uses
                                //            are for their direction (Binorm, SurfNorm comps), but when the 
                                //            tangent is being used for its magnitude (like an arc-length comp)
                                //            then set this to FALSE.
                                //      default:[TRUE]
{
  // locals
  SmPoint3d sCurvePointPV[2] ;
  
  // Base Curve eval
  if(pOptBaseCurvePV)
    { 
      SER( m_pCurve->Evaluate(dParameter, lHighestDeriv, bFromLeft, pOptBaseCurvePV, bNonZeroTangents )) ; 
    }
  else
    { 
      SER( m_pCurve->Evaluate(dParameter, 1, bFromLeft, sCurvePointPV, bNonZeroTangents )) ;
    }

  // Volume eval
  SmStatus eStat = SM_SUCCESS ;

  if(m_bInParamSpace)
    {  
      eStat = m_pVolume->Map(pOptBaseCurvePV ? pOptBaseCurvePV[0] : sCurvePointPV[0],
                             lHighestDeriv, 
                             (sCurvePointPV[1].x > 0.0) ? bFromLeft : !bFromLeft, 
                             (sCurvePointPV[1].y > 0.0) ? bFromLeft : !bFromLeft, 
                             (sCurvePointPV[1].z > 0.0) ? bFromLeft : !bFromLeft,
                             aDerivatives,
                             bNonZeroTangents );
    } // end Volume Evaluate branch
  else // m_pCurve is in InSpace
    {
      eStat = m_pVolume->Evaluate(pOptBaseCurvePV ? pOptBaseCurvePV[0] : sCurvePointPV[0],
                                  lHighestDeriv, 
                                  (sCurvePointPV[1].x > 0.0) ? bFromLeft : !bFromLeft, 
                                  (sCurvePointPV[1].y > 0.0) ? bFromLeft : !bFromLeft, 
                                  (sCurvePointPV[1].z > 0.0) ? bFromLeft : !bFromLeft,
                                  aDerivatives,
                                  bNonZeroTangents) ;
    }

  // all done
  return eStat ;

} // end SmCrvInVolume::EvaluateVolume

/*******************************************************************//**
PURPOSE: Calculate the minimum continuity of the curve and the 
         continuities at each of the unique discontinuity break pointss
    
NOTES: SmCrvInVolume does not really have knots but it does have 
    discontinuities which include the knots of the defining base curve
    and every BaseCurve/VolumeKnotIsoParameterPlane intersection point.
    
    Currently, this function gathers all the discontinuity points together
    and assigns to each an arbitrary SM_CT_C1 value.  That rating should
    be computed by inheriting the BaseCurve properties projected through 
    the volume.  For now, that's not been done.  
     
    The output includes a continuity value for the start and end of the 
    curve interval and all internal discontinuities. The minimum continuity 
    of the curve will be be the minimum continuity of the internal knots. 
    The end knots will always be discontinuous.
***********************************************************************/
SmStatus SmCrvInVolume::CalculateContinuities
  (SmContinuityType           & reMinContinuityInCurve,  // out: min continuity of all internal knots 
   SmTArray<SmContinuityType> & rContinuitiesAtKnots,    // out: continuity at every knot value for curve
   double                       dContinuityAngleTol)     // in : 
  const
{
  // init output
  rContinuitiesAtKnots.ReSet() ;
  reMinContinuityInCurve = SM_CT_CINFINITY ;

  // locals
  SmContinuityType eThisContinuity ;
  SmVector3d sThisEval[4], sOtherEval[4] ;

  // get effective breaks and continuities
  SmTArray<double> sBreaks ;
  GetKnots(sBreaks) ;

  // start and end breaks
  rContinuitiesAtKnots.SetSize(sBreaks.GetSize()) ;
  rContinuitiesAtKnots.SetAt(0, SM_CT_DISCONTINUOUS) ;
  rContinuitiesAtKnots.SetAt(sBreaks.GetSize()-1, SM_CT_DISCONTINUOUS) ;

  // build the output
  ULONG ii ;
  for(ii=1;ii<sBreaks.GetSize()-1;ii++)
    {
      // evaluate geometric continuities
      EvaluateContinuity(sBreaks[ii], TRUE, 
                        *this, sBreaks[ii], FALSE,
                         eThisContinuity,
                         sThisEval, sOtherEval, 
                         dContinuityAngleTol) ;

      // set output
      rContinuitiesAtKnots.SetAt(ii, eThisContinuity) ;  
      
      // save min continuity                                 
      if (reMinContinuityInCurve > eThisContinuity)   
        { reMinContinuityInCurve = eThisContinuity ; }
      
    } // end iter every knot

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
  if (bDebugMe) 
    {
      Dump();
    }
#endif // SM_DEBUG_CODE

  // all done
  return SM_SUCCESS;

} // end SmCrvInVolume::CalculateContinuities

/*******************************************************************//**
PURPOSE: For special orientations of some shapes make and return
         an exact BSpline equivalent Curve to this SmCrvInVolume object.

NOTES: An SmCrvInVolume object projects its m_pCurve from the m_pVolume's 
       InSpace to its final(Compounding) OutSpace
***********************************************************************/
SmStatus SmCrvInVolume::MakeExactBSplineIfPossible
 (SmBSplineCurve *& rpNewBSplineCurve) 
 const
{ 
  // locals
  SmStatus         sRtn    = SM_SUCCESS ;
  const SmVolume * pVolume = GetVolume() ;

  // Ask Volume to make exact BSpline if Possible
  if(m_bInParamSpace)
    {
      sRtn = pVolume->MakeExactBSplineOutCurveFromParamSpace(*this->GetCurve(), rpNewBSplineCurve) ;
    } // end ParamSpace branch
  else // Curve is InSpace
    {
      sRtn = pVolume->MakeExactBSplineOutCurveFromInSpace(*this->GetCurve(), rpNewBSplineCurve) ;
    } // end InSpace branch

  // all done
  return(sRtn) ; 

} // end SmCrvInVolume::MakeExactBSplineIfPossible

/*******************************************************************//**
PURPOSE: Get the knots and optionally the multiplicities of this curve.

NOTES: SmCrvInVolume doesn't really have knots, but it does have
 internal discontinuties inherited from the knots of the nested Curve
 and all the Curve/VolumeKnotIsoPlane intersections.  These discontinuities
 are reported as Knots with Multiplicities.
***********************************************************************/
SmStatus SmCrvInVolume::GetKnots
  (SmTArray<double> & rKnots,               // out: unique knot list
   SmTArray<ULONG>  * pOptMults,            // out: multiplicity reported for each knot
   const SmExtent1d * pOptIvl)              // in : interval of interest, NULL=Natural Interval, default:[NULL] 
  const
{
  // low work - Knots and Conts already cached
  ULONG ii, jj ;
  if(m_bNeedBreaks == FALSE)
    { 
      // copy saved knot array
      if(pOptIvl == NULL) { rKnots = m_sBreaks ; }
      else                { for(ii=0;ii<m_sBreaks.GetSize();ii++)
                              { if( pOptIvl->ContainsValue(m_sBreaks[ii]))
                                  { rKnots.Add(m_sBreaks[ii]) ; }
                              }
                          }
      if(pOptMults)       { GenerateMults(rKnots, *pOptMults) ; } 
      return( SM_SUCCESS ); 
    }

  // locals
  SmTArray<ULONG>  sMults ; 
  SmTArray<ULONG> *pMults = pOptMults ? pOptMults : &sMults ;
  SmTArray<SmContinuityType> sConts ;

  // init output
  rKnots.ReSet() ;
  pMults->ReSet() ; 

  // pick a UVWSpace tolerance
  double dVolumeParamSize = m_pVolume->GetNaturalParamDomain().ApproximateUnbounded().GetMaxDimension() ;
  double dCurveParamSize  = m_pCurve->GetNaturalInterval().ApproximateUnbounded().GetLength() ;

  //m_pVolume->ApproximateUVWSize(sVolumeSize) ;
  double dTolUVW = SM_EFF_ZERO * ( 1.0 + smos_Max(dVolumeParamSize, dCurveParamSize)) ;

  // get the Curve knots and multiplicities for the natural curve not the pOptIvl
  m_pCurve->GetKnots(rKnots, pMults) ;

  // modify the multiplicity list to act as if Curve is a degree 3 BSpline
  ULONG lCrvDegree = m_pCurve->GetDegree() ;
  
  // when adjusting is needed
  if(lCrvDegree != 3 && pMults)
    {
      for(ii=0;ii<pMults->GetSize();ii++)
        {
          ULONG lDelMult =  ( lCrvDegree < pMults->GetAt(ii) + 3)
                          ?  pMults->GetAt(ii) + 3 - lCrvDegree
                          :  1 ;
          pMults->SetAt(ii, lDelMult) ;  
        }
    } 

  // classify object for upcoming volume classification 
  SmCurveClassification sCurveClassification(m_pCurve, 
                                             m_pCurve->GetNaturalInterval(),
                                             NULL,
                                             dTolUVW) ;

  // classify the BaseCurve against the volume knot isoParameter lines.
  SmContext sContext ; 
  SER(sCurveClassification.Intersect3DDiscontinuities(*m_pVolume, m_bInParamSpace, sContext)) ;

  // what we should have here is the list of volume multiplicities 
  //  that associate with each classification boundary
  //  until that is done - just treat each Curve/SurfaceIsoLine intersection as a C1 discontiinuity

  // add the classification boundaries uniquely and in order to the rKnots and pMults lists
  ULONG  lCnt     = sCurveClassification.GetSize() ;
  double dUVWKnot = rKnots[0] ;

  // for every Classification boundary (include the ends in case pOptIvl != NULL) - add ClassBoundaries to knot arrays
  for(ii=0,jj=0;ii<=lCnt;ii++)
    {
      double dClassBndry =   (ii < lCnt) 
                           ? (sCurveClassification.GetAt(ii).m_vInterval.GetMin())
                           : (sCurveClassification.GetAt(ii-1).m_vInterval.GetMax()) ;

      // check the next UVWKnots until we get one bigger or equal to dClassBndry
      while ( dClassBndry > dUVWKnot + dTolUVW) 
        { jj++ ;
          dUVWKnot = (jj == rKnots.GetSize()) ? SM_BIG_DOUBLE : rKnots[jj] ;
        }

      // when dClassBndry value is already in rKnots - done with this ClassKnot - move onto next ClassKnot
      if ( dClassBndry  > dUVWKnot - dTolUVW) 
        { continue ; }

      // arrive here when classKnot needs to be added to Knot array at index jj
      // add class boundary as if it were a C1 knot into the m_pCurve
      rKnots.InsertAt(jj, dClassBndry, 1) ;
      if(pMults) { pMults->InsertAt(jj, 2, 1) ; }

      // assuming the Curve knots and the Classification boundaries are both in order and unique 
      // skip to next UVWKnot and continue
      jj += 1 ;
      if (jj >= rKnots.GetSize()) { jj = rKnots.GetSize() - 1 ; }
      dUVWKnot = rKnots[jj] ; 

    } // end iter every Classification boundary

  SM_ASSERT(rKnots.GetSize() == pMults->GetSize()) ;
  
  // cache the knots
  lCnt = rKnots.GetSize() ;
  ((SmCrvInVolume *)this)->m_sBreaks     = rKnots ; 
  ((SmCrvInVolume *)this)->m_bNeedBreaks = FALSE ; 
  ((SmCrvInVolume *)this)->m_sConts.SetSize(lCnt) ;

  // for every break - convert multiplicities into a SmContinuityType stored in m_sConts
  for(ii=0;ii<lCnt;ii++)
    {
      ULONG lMult = pMults->GetAt(ii) ;

      // convert lMult to Continuity assuming degree 3 curve
      SmContinuityType eCont =   ii == 0      ? SM_CT_DISCONTINUOUS
                               : ii == lCnt-1 ? SM_CT_DISCONTINUOUS
                               : lMult == 0   ? SM_CT_CINFINITY
                               : lMult == 1   ? SM_CT_C1_C2
                               : lMult == 2   ? SM_CT_C1
                               : lMult == 3   ? SM_CT_C0
                               :                SM_CT_DISCONTINUOUS ;

      // add elems to m_sConts
      ((SmCrvInVolume *)this)->m_sConts.SetAt(ii, eCont) ;

    } // end iter every break setting continuity types 

  // set output
  
  // when given pOptIvl - cull unwanted output values
  if(   pOptIvl
     && (   rKnots[0]      < pOptIvl->GetMin() - dTolUVW
         || rKnots[lCnt-1] > pOptIvl->GetMax() + dTolUVW))  
    { 
      double dMin = pOptIvl->GetMin() ;
      double dMax = pOptIvl->GetMax() ;

      // for every break - ii = index into preCulled array, jj = index into postCulled array
      for(jj=0,ii=0;ii<lCnt;ii++)
        {
          double dClassBndry = rKnots[ii] ;
          
          // for rKnot values below the OptIvl
          if(dMin > dClassBndry + dTolUVW)      { // don't increment jj
                                                }
          else if(dMax < dClassBndry - dTolUVW) { break ; }
          else // rKnot value is contained
            {
              // when needed - compress output arrays
              if(ii != jj) 
                {
                  // compress the rKnot array
                  rKnots.SetAt(jj, rKnots.GetAt(ii)) ;

                  // when asked - compress the pOptMult array
                  if(pOptMults) { pOptMults->SetAt(jj, pOptMults->GetAt(ii)) ; }

                }

              // increment jj
              jj++ ;
            } // end found a value to keep
        } // end iter every break value

      // reset output arrays sizes
      rKnots.SetSize(jj) ;
      if(pOptMults) { pOptMults->SetSize(jj) ; }

    } // end given pOptIvl check

  // all done
  return SM_SUCCESS;

} // end SmCrvInVolume::GetKnots

/*******************************************************************//**
PURPOSE: Get the knot multiplicities at each curve discontinuity

NOTES: This is an internal helper function only to be used after
       m_bNeedBreaks has been set to FALSE and m_sBreaks is current.

       An SmCrvInVolume is not a BSpline curve but the old SMLib
       interface pretends that all curves are BSplines.  So, this
       function assumes that the degree of the curve is 3 and computes
       an "effective" multiplicity at every discontinuity based
       on the discontinuities m_sConts value. 

       In a perfect world, we would remove the pretend-to-be-a-bspline
       virtual functions from SmCurve and switch the interface to only
       reporting discontinuites.
***********************************************************************/
SmStatus SmCrvInVolume::GenerateMults
  (SmTArray<double> & rKnots,       // in : ascending param array of discontinuities of interest (expected to be in m_vKnots
   SmTArray<ULONG>  & rMults)       // out: associated array of "effective" multiplicities.
  const
{
  // check state - cached m_sKnots and m_sConts are current
  if(   m_bNeedBreaks == TRUE
     || ( m_bNeedBreaks == FALSE && m_sBreaks.GetSize() != m_sConts.GetSize() ) )
    { SER_MSG(SM_ERR, _T("SmCrvInVolume::GenerateMults called when cached data is not up to date.")) ; }

  // init output
  rMults.SetSize(rKnots.GetSize()) ;

  // locals
  ULONG ii, bi ;
  double dParam, dBreak ;
  SmBoolean bFoundKnot = FALSE ;

  // for every rKnots param value
  for(bi=0,ii=0;ii<rKnots.GetSize();ii++,bFoundKnot=FALSE)
    {
      dParam = rKnots[ii] ;
      
      // find the corresponding m_sBreaks value
      for(;bi<m_sBreaks.GetSize();)
        {
          dBreak = m_sBreaks[bi] ;

          // when break matches current knot
          if(SM_IS_ZERO_TO_TOL(dParam - dBreak, SM_EFF_ZERO))  // potential problem for cases with knots close to one another!
            {
              // stop searching
              bFoundKnot = TRUE ;
              break ;
            }
          else if(dBreak > dParam)
            {
              // given knot value is not a discontinuity
              rMults[ii] = 0 ;
              break ;
            }

          // else - try searching the rest of the knots
          bi++ ;
        } // end search for matching knot values
       
       // error condition - ran out of breaks to search 
       if(bi >= m_sBreaks.GetSize())
         {
           SER_MSG(SM_ERR,_T("SmCrvInVolume::GenerateMults given param outside of specified domain")) ;
         }

       // when a match was not found move onto the next Knot
       if(bFoundKnot == FALSE)
         { continue ; }

       // check cache currency
       SmContinuityType eContType = m_sConts[ii] ;
       if(eContType == SM_CT_UNDEFINED)
         { SER_MSG(SM_ERR, _T("SmCrvInVolume::GenerateMults has an undefined SmContinuityType in m_vKnots cache")) ; }

       // set an effective multiplicity
       rMults[ii] =   eContType == SM_CT_DISCONTINUOUS ? 3
                    : eContType <= SM_CT_G1_G2_G3      ? 2
                    : eContType <= SM_CT_C1_C2_G3      ? 1
                    : 0 ;

    } // end iter every knot of interest

  // all done
  return(SM_SUCCESS) ;

} // end SmCrvInVolume::GenerateMults
    
/*******************************************************************//**
PURPOSE: Get the knots and optionally the multiplicities of this curve.

NOTES: SmCrvInVolume does not really have knots, it has internal discontinuities
 inherited from the contained curve's knots and the Curve/VolumeKnotIsoPlane
 intersections.  Those are treated as if they were knots in this function.
***********************************************************************/
ULONG SmCrvInVolume::GetNumberNaturalKnots()            
 const
{
  // return value
  ULONG lCnt = 0 ;

  // locals
  ULONG ii ;
  SmTArray<double> sKnots ;
  SmTArray<ULONG>  sMults ;
  GetKnots(sKnots, &sMults) ;

  // count the knots
  for(ii=0;ii<sKnots.GetSize();ii++)
    {
      lCnt += sMults[ii] ;

    } // end iter every knot

  // all done
  return(lCnt) ; 

} // end SmCrvInVolume::GetNumberNaturalKnots

/*******************************************************************//**
PURPOSE: Set Base Curve

NOTES: Make curve copy when needed
***********************************************************************/
SmStatus SmCrvInVolume::SetBaseCurve
 (SmCurve   *pCurve,        // in : new base curve
  SmBoolean  bInParamSpace, // in : TRUE = cpCurve is in Volume's ParamSpace, FAlSE=Curve in InSpace
  SmBoolean  bCopyCurve,    // in : TRUE  = save a copy of pCurve, FALSE = save pCurve
                            //      default:[FALSE]
  SmBoolean  bOwnsCurve)    // in : TRUE = delete this Curve when destructed, FALSE = don't
                            //      default:[FALSE]
{ 
  // when needed - clean out old curve
  if ( m_pCurve )
    {
      if ( OwnsCurve() )
      { delete m_pCurve; }
      m_pCurve = NULL; 

      Notify( SM_NO_PRE_EDIT, this, SM_NO_GET_OWNER( this ), NULL );
    }

  // make the assignment
  m_pCurve        = pCurve ;
  m_bInParamSpace = bInParamSpace ; 
  m_lOwnerFlag   &= 2 ; // clear owns curve bit

  // Set owner flag and copy curve when necessary 
  if( bCopyCurve) 
    { 
      // find a context
      const SmContext *pContext = GetContext() ? GetContext() : m_pCurve->GetContext() ;
      SER_MSG(pContext == NULL ? SM_ERR : SM_SUCCESS, _T("No Context available")) ;

      // copy the curve
      m_pCurve->Copy(*pContext, m_pCurve) ;

      // set owns curve bit
      m_lOwnerFlag |= 1 ; 
    }

  // when asked - remember to own this curve
  if(bOwnsCurve)
    {
      m_lOwnerFlag |= 1 ; 
    }

  // when owned, set the contained Surface and Volume owner values
  if(m_lOwnerFlag & 1) { m_pCurve->SetOwner(this) ; }
  if(m_lOwnerFlag & 2) { m_pVolume->SetOwner(this) ; }

  // all done
  return(SM_SUCCESS) ;

} // end SmCrvInVolume::SetBaseCurve

/*******************************************************************//**
PURPOSE: Set compounding volume

NOTES: Make volume copy when needed
***********************************************************************/
SmStatus SmCrvInVolume::SetVolume
( SmVolume  *pVolume,        // in : new base volume
  SmBoolean  bCopyVolume,    // in : TRUE  = save a copy of pVolume
                             //      FALSE = save pVolume
                             //      default:[FALSE]
  SmBoolean  bOwnsVolume )    // in : TRUE = delete this volume when destructed, FALSE = don't
                             //      default:[FALSE]
{
    // when needed - clean out old volume
  if ( m_pVolume )
    {
      if ( OwnsVolume() )
      { delete m_pVolume; }
      m_pVolume = NULL;

      Notify( SM_NO_PRE_EDIT, this, SM_NO_GET_OWNER( this ), NULL );
    }

  // make the assignment
  m_pVolume     = pVolume ;
  m_lOwnerFlag &= 1 ;    // clear owns curve bit
  
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
  if(m_lOwnerFlag & 1) { m_pCurve->SetOwner(this) ; }
  if(m_lOwnerFlag & 2) { m_pVolume->SetOwner(this) ; }

  // all done
  return(SM_SUCCESS) ;

} // end SmCrvInVolume::SetVolume

/*******************************************************************//**
PURPOSE: Set Owns contained Curve and Volume flat

NOTES: When a contained Curve or Volume becomes owned,
       its m_pOwner pointer is set to this
***********************************************************************/
void SmCrvInVolume::SetOwnerFlag
 (ULONG lOwnerFlag)    // in : 0 = deletes nothing when destructed
                       //      1 = deletes only  curve when destructed
                       //      2 = deletes only volume when destructed
                       //      3 = deletes both curve and volume when destructed
                       //      default:[0]
{ 
  // maek the assignment
  m_lOwnerFlag = lOwnerFlag ; 

  // when owned, set the contained Surface and Volume owner values
  if(m_lOwnerFlag & 1) { m_pCurve->SetOwner(this) ; }
  if(m_lOwnerFlag & 2) { m_pVolume->SetOwner(this) ; }

} // end SmCrvInVolume::SetOwnerFlag

/*******************************************************************//**
PURPOSE: Ensures this curve owns its base curve and compounding volume

NOTES: Any unowned object is copied and its owner flag is modified
***********************************************************************/
SmStatus SmCrvInVolume::MakeOwner()
{
  // copy and own unowned curves
  if(!OwnsCurve())
    {
      // copy the curve
      SER(m_pCurve->Copy(*GetContext(), m_pCurve)) ;
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
  if(m_lOwnerFlag & 1) { m_pCurve->SetOwner(this) ; }
  if(m_lOwnerFlag & 2) { m_pVolume->SetOwner(this) ; }

  // all done
  return(SM_SUCCESS) ;

} // end SmCrvInVolume::MakeOwner
    
/*******************************************************************//**
PURPOSE: Reverse curve parameterizationwhile preserving its NaturalInterval range.

NOTES: 1. example: NewPoint(sNatIvl.Min) == OldPoint(sNatIvl.Max) with negated tangents.
       2. For convenience, Given an old interval of interest in crOldInterval, 
                           Output the new domain for crOldInterval in rNewInterval.
          The crOldInterval value does not affect the reparameterization of the curve.
***********************************************************************/
SmStatus SmCrvInVolume::ReverseParameterization
  (const SmExtent1d & crOldInterval,  // in : interval of interest, may be a subset of natural interval
   SmExtent1d       & rNewInterval)   // out: new domain for crOldInterval on modified curve           
{ 
  Notify(SM_NO_PRE_EDIT, this, SM_NO_GET_OWNER(this), NULL);

  // Don't own curve can not reverse parameterization
  if (!OwnsCurve()) 
    { SER(SM_ERR) ; }

  // pass the call along to the Curve
  SER(m_pCurve->ReverseParameterization(crOldInterval,rNewInterval));
   
  Notify(SM_NO_POST_EDIT, this, SM_NO_GET_OWNER(this), NULL);

  return SM_SUCCESS;

} // end SmCrvInVolume::ReverseParameterization

/*******************************************************************//**
PURPOSE: Trim this curve.

NOTES: 
***********************************************************************/
SmStatus SmCrvInVolume::Trim
  (SmExtent1d & crTrimInterval,    // i/o: desired new interval, can be snapped by tol to existing knots
   SmBoolean    bNotify,           // in : TRUE  = call Notify after trimming (previous behavior)
                                   //      FALSE = skip Notify after trimming
                                   //      UNSURE= skip notify, skip trimming, just recompute TrimInterval
                                   //      default:[TRUE]
   SmBoolean    bSkipDebugCheck)   // in : internal use: use default value
{ 
  // Don't own curve can not trim the curve
  if (!OwnsCurve()) 
    { SER(SM_ERR) ; }

  if(bNotify != UNSURE)
    { Notify(SM_NO_PRE_EDIT, this, SM_NO_GET_OWNER(this), NULL) ; }

  // pass the call along to the Curve
  SER(m_pCurve->Trim(crTrimInterval, bNotify, bSkipDebugCheck)) ; // may snap sIvl by tol to existing knots

  // clear the knots
  if(m_bNeedBreaks == FALSE)
    {
      m_sBreaks.ReSet() ;
      m_sConts.ReSet() ;
      m_bNeedBreaks = TRUE ;
    }

  if(bNotify == TRUE)
    { Notify(SM_NO_POST_EDIT, this, SM_NO_GET_OWNER(this), NULL) ; }

  return SM_SUCCESS;

} // end SmCrvInVolume::Trim

/*******************************************************************//**
PURPOSE: Transform the SmCrvInVolume object by concatenating the 
         RotateNMove and Scale transforms into the current Volume

NOTES: 
***********************************************************************/
SmStatus SmCrvInVolume::Transform
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

  Notify(SM_NO_PRE_EDIT, this, SM_NO_GET_OWNER(this), NULL);

  // compound the Transform onto the contained volume  
  SER_MSG(m_pVolume->Transform(crRotateNMove, cpOptScale),
          _T("CompoundingVolume failed to be Transformed - Ask for support.")) ;

  Notify(SM_NO_POST_EDIT, this, SM_NO_GET_OWNER(this), NULL);

  return SM_SUCCESS;

} // end SmCrvInVolume::Transform

/*******************************************************************//**
PURPOSE: Compute the total size of the memory used by the SmCrvInVolume.

NOTES: Does not include the curve's attribute memory
***********************************************************************/
ULONG SmCrvInVolume::GetMemoryUsed  // rtn: smaller size of actually used memory in bytes
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

  // locals
  ULONG lUsed, lCurveAllocated=0, lSurfaceAllocated=0, lThisAllocated ;

  // this + m_pNurb memory
  lUsed =   sizeof(*this) 
          + (  (m_pCurve  && OwnsCurve())  ? m_pCurve->GetMemoryUsed(lCurveAllocated)    : 0)
          + (  (m_pVolume && OwnsVolume()) ? m_pVolume->GetMemoryUsed(lSurfaceAllocated) : 0)  
          + this->GetAttributeMemoryUsed(lThisAllocated, 
                                         eMarkType) ;  // note: uses without increment eMarkType value

  rlMemoryAllocated = sizeof(*this) + lCurveAllocated + lSurfaceAllocated + lThisAllocated ;

  // + cache memory
  if ( m_pCacheObj )
  {
      lUsed += m_pCacheObj->GetMemoryUsed( lThisAllocated );
      rlMemoryAllocated += lThisAllocated;
  }

  // all done
  return(lUsed) ;

} // end SmCrvInVolume::GetMemoryUsed

/*******************************************************************//**
PURPOSE: Write SmCrvInVolume to given output stream.

NOTES: 
***********************************************************************/
SmStatus SmCrvInVolume::WriteToDB
 (SmDatabaseIO & rDB,       // in : target output stream
  ULONG lDBVersionNumber)   // in : database version to get proper sequence of writes                                                                       
 const
{
  // file type, ASCII or BINARY
  SmFileType       eType    =  rDB.GetFileType();
  std::ostream   & rFileOut = *rDB.GetOutStreamPtr();

  // curve locals
  
  if (eType == SM_ASCII) 
    {
      rFileOut << m_bNeedBreaks          << " Has knots flag \n" ;
    }
  else 
    {
      SER(rDB.WriteBoolean( m_bNeedBreaks)) ;  
    }

  // space curve
  ULONG lCrvDim = m_pCurve->GetDim() ;
  if (eType == SM_ASCII) { rFileOut << " Begin CrvInVolume->Curve Section \n"; }
  SER(rDB.WriteType(m_pCurve->GetType(), &lCrvDim)) ;
  SER(m_pCurve->WriteToDB(rDB, lDBVersionNumber)) ;

  // bInParamSpace value
  rFileOut << m_bInParamSpace << " InParamSpace: 0=Curve in Volume's ParamSpace, 1=Curve in InSpace\n" ; 

  // compounding volume
  if (eType == SM_ASCII) { rFileOut << " Begin CrvInVolume->Volume Section \n"; }
  SER(rDB.WriteType(m_pVolume->GetType())) ; 
  SER(m_pVolume->WriteToDB(rDB, lDBVersionNumber)) ;

  // when appropriate - knots
  if(m_bNeedBreaks == FALSE)
    {
      // number of knots
      if (eType == SM_ASCII) { rFileOut << m_sBreaks.GetSize() << " Number of Curve internal discontinuities\n" ; }
      else /* Binary */      { SER(rDB.WriteLong(m_sBreaks.GetSize())) ; }

      // for every knot
      ULONG ii ; 
      if(eType == SM_ASCII)
        {
          for(ii=0;ii<m_sBreaks.GetSize();ii++ ) { rFileOut << m_sBreaks[ii] << " " ; }
          if(m_sBreaks.GetSize() > 0) { rFileOut << " \n" ; }
        }
      else /* Binary */
        {
          for(ii=0;ii<m_sBreaks.GetSize();ii++ ) { SER(rDB.WriteDouble(m_sBreaks[ii])) ; }
        }

      // for every knot continuity
      if(eType == SM_ASCII)
        {
          for(ii=0;ii<m_sConts.GetSize();ii++ ) { rFileOut << m_sConts[ii] << " " ; }
          if(m_sConts.GetSize() > 0) { rFileOut << " \n" ; }
        }
      else /* Binary */
        {
          for(ii=0;ii<m_sConts.GetSize();ii++ ) { SER(rDB.WriteLong(m_sConts[ii])) ; }
        }
    }  

  // all done
  return SM_SUCCESS;

} // end SmCrvInVolume::WriteToDB

/*******************************************************************//**
PURPOSE: static method to Read a SmCrvInVolume from a given stream  

NOTES: 
***********************************************************************/
SmStatus SmCrvInVolume::ReadFromDB
 (SM_TYPE           lType,              // NotUsed: in : curve type to be read
  SmDatabaseIO    & rDB,                // in : target output stream
  ULONG             lDim,               // NotUsed: in : curve image space dim, 2 or 3                                                                                    
  const SmContext & crContext,          // in : context for new object construction
  SmCurve         *&rpNewCurve,         // out:    NULL on input = new object allocated in this routine built from stream data
                                        //      NotNULL on input = pointer to an empty object to be filled by this routine
  ULONG             lDBVersionNumber)   // in : database version to get proper sequence of writes
{
  SM_REF2(lType, lDim) ;
  // check input
  SER(  (   rpNewCurve == NULL
         || rpNewCurve->IsKindOf(SmCrvInVolume_TYPE)) ? SM_SUCCESS : SM_ERR) ;

  // init output object
  SmCrvInVolume *pCrvInVolume =   (rpNewCurve == NULL)
                                ? new (crContext) SmCrvInVolume()
                                : (SmCrvInVolume *)rpNewCurve ;
  // file type
  SmFileType      eType   = rDB.GetFileType();
  std::istream  & rFileIn = *rDB.GetInStreamPtr();
      
  // locals
  SmPoint2d sMin, sMax ; 
  SmBoolean bNeedKnots ;

  if (eType == SM_ASCII) 
    {
      rFileIn >> bNeedKnots ; rDB.GoToNextLine() ;
    }
  else 
    {
      SER(rDB.ReadBoolean(bNeedKnots));
    }

  // curve locals
  SmCurve *pCurve=NULL ;
  SM_TYPE  lCurveType ;  
  ULONG    lCurveDim ;

  // nested curve
  if (eType == SM_ASCII) { rDB.GoToNextLine() ; }
  SER(rDB.ReadType(lCurveType, &lCurveDim)) ; 
  SER(SmCurve::ReadFromDB(lCurveType, rDB, lCurveDim, crContext, pCurve, lDBVersionNumber)) ; 

  // bInParamSpace value
  rFileIn >> pCrvInVolume->m_bInParamSpace ; rDB.GoToNextLine() ;

  // compounding volume locals
  SmVolume *pVolume=NULL ;
  SM_TYPE   lVolumeType ;  

  // compounding volume
  if (eType == SM_ASCII) { rDB.GoToNextLine() ; }
  SER(rDB.ReadType(lVolumeType)) ; 
  SER(SmVolume::ReadFromDB(lVolumeType, rDB, crContext, pVolume, lDBVersionNumber)) ;
  
  // when appropriate - knots
  if(bNeedKnots == FALSE)
    {
      ULONG ii, lKnotCnt ;

      // knot number
      if (eType == SM_ASCII) { rFileIn >> lKnotCnt ; rDB.GoToNextLine() ;}
      else /* Binary */      { SER(rDB.ReadLong(lKnotCnt)); }

      // manage Knot array
      pCrvInVolume->m_sBreaks.SetSize(lKnotCnt) ; 
      double *dU = pCrvInVolume->m_sBreaks.GetDataArray() ;

      // read every knot
      if (eType == SM_ASCII) { for(ii=0;ii<lKnotCnt;ii++ ) { rFileIn >> dU[ii]; }
                               if(lKnotCnt > 0) { rDB.GoToNextLine() ; }
                             }
      else                   { for(ii=0;ii<lKnotCnt;ii++ ) { SER(rDB.ReadDouble(dU[ii])); }
                             } 

      // manage Conts array
      pCrvInVolume->m_sConts.SetSize(lKnotCnt) ; 
      SmContinuityType *eCont = pCrvInVolume->m_sConts.GetDataArray() ;
      ULONG lC ;         

      // read every Continuity
      if (eType == SM_ASCII) { for(ii=0;ii<lKnotCnt;ii++ ) { rFileIn >> lC ;
                                                             eCont[ii] = (SmContinuityType)lC ;
                                                           }
                               if(lKnotCnt > 0) { rDB.GoToNextLine() ; }
                             }
      else                   { for(ii=0;ii<lKnotCnt;ii++ ) { SER(rDB.ReadLong(lC)); 
                                                             eCont[ii] = (SmContinuityType)lC ;
                                                           }
                             } 
    } 

  // load obj
  pCrvInVolume->m_pCurve      = pCurve ;   
  pCrvInVolume->m_pVolume     = pVolume ;   
  pCrvInVolume->m_bNeedBreaks = bNeedKnots ;
  pCrvInVolume->m_lOwnerFlag  = 3 ; 
  
  // all done
  rpNewCurve = pCrvInVolume ;
  return SM_SUCCESS;

} // end SmCrvInVolume::ReadFromDB

/*******************************************************************//**
PURPOSE:
NOTES:
***********************************************************************/
SmBoolean SmCrvInVolume::IsKindOf( SM_TYPE t ) const
{
  return ((SmCrvInVolume_TYPE == t) ? TRUE : SmCurve::IsKindOf( (t) ));
}

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
void SmCrvInVolume::Dump
  () 
 const
{
  TCHAR sBuff[SM_TBLOCK_SIZE], sBuffForFile[SM_TBLOCK_SIZE];
  
  // SmCrvInVolume data dump
  smos_WriteBuffer(_T("\nBegin SmCrvInVolume::Dump()")) ;
  
  // m_pCurve and m_pVolume
  smos_sprintf( sBuff, (_T("\nSmCrvInVolume = [0x%p], BaseCurve = [0x%p], CurveIn [%sSpace], CompoundingVolume = [0x%p]")), 
              this, 
              m_pCurve, 
              m_bInParamSpace ? _T("Param") : _T("In"),
              m_pVolume);
  smos_sprintf(sBuffForFile,
             _T("\nSmCrvInVolume = %s, BaseCurve = %s, CurveIn [%sSpace], CompoundingVolume = %s\n"),
                               _T("notNULL"),
             m_pCurve        ? _T("notNULL") : _T("NULL"), 
             m_bInParamSpace ? _T("Param")   : _T("In"), 
             m_pVolume       ? _T("notNULL") : _T("NULL")) ;
  smos_WriteBuffer(sBuff, sBuffForFile) ;

  // m_lOwnerFlag and m_bNeedBreaks 
  smos_sprintf( sBuff, _T(", OwnerFlag = [%ld], NeedsBreaks = [%s]"), 
              m_lOwnerFlag,
              m_bNeedBreaks ? _T("TRUE = breaks not yet cached") : _T("FALSE = breaks cached"));
  smos_WriteBuffer(sBuff);
  smos_WriteBuffer(_T("\n   OwnerFlag: 0 = deletes nothing when destructed ")); 
  smos_WriteBuffer(_T("\n              1 = deletes only curve when destructed ")); 
  smos_WriteBuffer(_T("\n              2 = deletes only volume when destructed ")); 
  smos_WriteBuffer(_T("\n              3 = deletes curve and volume when destructed ")); 

  // cached data
  ULONG ii ;

  smos_sprintf( sBuff, _T("\n  Cached Break Cnt = [%ld]"), m_sBreaks.GetSize()) ;
                      
  smos_WriteBuffer(sBuff) ;

  // for every break - output param and continuity value
  for(ii=0;ii<m_sBreaks.GetSize();ii++)
    {
      TCHAR* pString = SmGetContinuityTypeString(m_sConts[ii]);
      smos_sprintf( sBuff, _T("\n         Break [%ld] = [%16.16lf - %s]"), 
                  ii, m_sBreaks[ii], pString) ;
      smos_WriteBuffer(sBuff) ;
      delete pString;  pString = NULL;
    } // end iter every break

  // output base class data
  smos_WriteBuffer(_T("\nBegin SmCrvInVolume Base Class SmCurve Dump "));
  SmCurve::Dump(FALSE) ;
  smos_WriteBuffer(_T("End SmCrvInVolume Base Class SmCurve Dump "));
  
  // dump the contained Curve
  smos_WriteBuffer(_T("\n\nBegin contained SmCrvInVolume->Curve Dump")) ;
  m_pCurve->Dump();
  smos_WriteBuffer(_T("End contained SmCrvInVolume->Curve Dump")) ;

  // dump the contained Volume
  smos_WriteBuffer(_T("\n\nBegin contained SmCrvInVolume->Volume Dump")) ;
  m_pVolume->Dump();
  smos_WriteBuffer(_T("End contained SmCrvInVolume->Volume Dump")) ;

  smos_WriteBuffer(_T("\nEnd SmCrvInVolume::Dump()\n")) ;

} // end SmCrvInVolume::Dump

/*******************************************************************//**
PURPOSE: AssertValid() Test reporting static labels
***********************************************************************/
SmAssertReportLabel sAssertCrvInVolume_list[] =
{
  /*  0 */ {SM_AT_NESTED_TEST, _T("Base Curve"),             _T("BaseCurve must pass AssertValid") },
  /*  1 */ {SM_AT_NESTED_TEST, _T("Compounding Volume"),     _T("Compounding Volume must pass AssertValid") },
  /*  2 */ {SM_AT_CACHE,       _T("Inconsistent Cache"),     _T("Cached breaks and associated continuities must be consistent") },
  /*  3 */ {SM_AT_POINTER,     _T("Bad BaseCurve Owner"),    _T("When owned, BaseCurve owner must be this SmCrvInVolume object") },
  /*  4 */ {SM_AT_POINTER,     _T("Bad BaseVolume Owner"),   _T("When owned, BaseVolume owner must be this SmCrvInVolume object") },
  /*  5 */ {SM_AT_POINTER,     _T("Bad BaseCurve Context"),  _T("When owned, BaseCurve->context must be this SmCrvInVolume->context") },
  /*  6 */ {SM_AT_POINTER,     _T("Bad BaseVolume Context"), _T("When owned, BaseVolume->context must be this SmCrvInVolume->context") } 
} ;

/*******************************************************************//**
PURPOSE:

NOTES: returns TRUE  = OK
                        FALSE = Problem
***********************************************************************/
SmBoolean SmCrvInVolume::AssertValid
 (SmAssertArray    * pAList,        // i/o: Accumulating list of failed Asserts, NULL to ignore, default:[NULL] 
  SmAssertTestLevel  eTestLevel,    // in : SM_LEVEL_0=fewest tests, SM_LEVEL_1, SM_LEVEL_2=all tests, 
                                    //      SM_LEVEL_GIVEN = run tests in order requested in pTestRequests 
                                    //      default:[SM_LEVEL_0] 
  SmAssertWalking    eWalkTree,     // in : SM_WALK = run AssertValid on any topology graph descendants, SM_NO_WALK=don't, default:[SM_WALK] 
  SmTArray<ULONG>  * pTestRequests) // in : when eTestLevel == SM_LEVEL_GIVEN, run these tests in this order, default:[NULL]
 const
{
  /*  0 */ // test BaseCurve curve
  SmBoolean bRtn = SM_ASSERT_BOOLEAN_REPORT(0, SM_LEVEL_0, m_pCurve->AssertValid(pAList, eTestLevel, eWalkTree, pTestRequests), _T("")) ;

  /*  1 */ // test contained volume
  bRtn &= SM_ASSERT_BOOLEAN_REPORT(1, SM_LEVEL_0, m_pVolume->AssertValid(pAList, eTestLevel, eWalkTree, pTestRequests), _T("")) ;

  /*  2 */ // consistent internal cache
  SmBoolean bConsistentCache = (   (m_bNeedBreaks == TRUE  && (m_sBreaks.GetSize() == 0 && m_sConts.GetSize() == 0))
                                || (m_bNeedBreaks == FALSE && (m_sBreaks.GetSize() == m_sConts.GetSize()))) ;
  bRtn &= SM_ASSERT_BOOLEAN_REPORT(2, SM_LEVEL_0, bConsistentCache, _T("")) ;

  /*  3 */ // Bad BaseCurve Owner
  bRtn &= SM_ASSERT_BOOLEAN_REPORT(3, SM_LEVEL_0, (m_lOwnerFlag & 1) && m_pCurve->GetOwner() == this, _T("")) ;

  /*  4 */ // Bad BaseVolume Owner
  bRtn &= SM_ASSERT_BOOLEAN_REPORT(4, SM_LEVEL_0, (m_lOwnerFlag & 2) && m_pVolume->GetOwner() == this, _T("")) ;

  /*  3 */ // Bad BaseCurve context
  bRtn &= SM_ASSERT_BOOLEAN_REPORT(5, SM_LEVEL_0, (m_lOwnerFlag & 1) && m_pCurve->GetContext() == GetContext(), _T("")) ;

  /*  4 */ // Bad BaseVolume context
  bRtn &= SM_ASSERT_BOOLEAN_REPORT(6, SM_LEVEL_0, (m_lOwnerFlag & 2) && m_pVolume->GetContext() == GetContext(), _T("")) ;
  // all done
  return(bRtn) ;

} // end SmCrvInVolume::AssertValid

// obsolete
// /*******************************************************************//**
// PURPOSE: Fix Assert Report failures reported by an AssertValid call
// 
// RETURNS ---  TRUE  = OK
//              FALSE = Problem
// ***********************************************************************/
// SmBoolean SmCrvInVolume::AssertHeal
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
//       return ( SmCurve::AssertHeal(rAReport, pAList) ) ;
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
//                rAReport.m_pHealMessage = _T("SmCrvInVolume::AssertHeal fix not yet supported") ;  
//                 
//     }
// 
//   // all done
//   return(bRtn) ;
// 
// } // end SmCrvInVolume::AssertHeal
// end obsolete

