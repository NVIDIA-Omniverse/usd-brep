// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************//**
* FILE NAME --- SmCrvOnSurf.cpp
* PURPOSE: Source file for SmCrvOnSurf methods.
**********************************************************************/

#include "StdAfx.h"
#include <SmCrvOnSurf.h>

/*******************************************************************//**
PURPOSE: Reload an SmCrvOnSurf values and checks to make sure that 
         curve is valid.

NOTES: 
***********************************************************************/
void SmCrvOnSurf::SetAll
  (SmCurve          & rUVCurve,    // in : UVCurve to be projected into 3d space
   SmSurface        & rSurface,    // in : Surface used to project UVCurve into 3d
   const SmExtent2d * cpUVDomain,  // in : Surface domain limit (UVCurve will fit in this domain)
                                   //      NULL = use SurfaceNaturalUVDomain
                                   //      default:[NULL]
   ULONG              lOwnerFlag,  // in : 0 = deletes nothing when destructed
                                   //      1 = copies curve and deletes copy when destructed
                                   //      2 = copies both inputs, deletes both copies when destructed
                                   //      3 = saves both inputs, deletes both originals when destructed
                                   //      default:[0]
   const SmContext  * cpContext)   // in : must be given for automatic variables, optional for
                                   //      stack variables built with overloaded new.
                                   // in : default:[NULL]                         
{
  // base class value
  SM_ASSERT(GetDim() == 3) ; 

  // init member values
  m_pUVCurve    = &rUVCurve ; // when m_lOwnerFlag == 1 or 2 value gets overwritten later with a copy
  m_pSurface    = &rSurface ; // when m_lOwnerFlag == 1 or 2 value gets overwritten later with a copy
  m_bNeedBreaks = TRUE ;
  m_lOwnerFlag  = lOwnerFlag ;
  m_pBSApproxCurve = NULL;

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
       
  // check state
  if( rUVCurve.GetDim() != 2)
    { SM_ASSERT_MSG(rUVCurve.GetDim() == 2, _T("SmCrvOnSurf:: setting UVTrimCurve to a non 2D Curve")) ; }

#ifdef SM_USE_CONSTRUCTOR_ASSERT_VALID
  SmBoolean bOk = SM_ASSERT_VALID_CONSTRUCTION(&rUVCurve) ;
#endif // SM_USE_CONSTRUCTOR_ASSERT_VALID

  if(bDebugMe)
    {
      SM_DUMP_AND_ASSERT_VALID(&rUVCurve) ;
      SM_DUMP_AND_ASSERT_VALID(&rSurface) ;
      SmTemporaryChangeValue<SmBoolean> sTemp1(bDebugMe, FALSE) ; // to prevent infinite looping
      SmCrvOnSurf sCrvOnSurf(rUVCurve, rSurface) ;                // duplicated here for display

      smgfx_Erase() ;
      smgfx_SetLook(1,2,  0,1,1) ;   rSurface.DrawUV(8,8,FALSE,NULL,TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(3,4, .3,.3,.3) ; rSurface.DrawSeams() ; sm_GraphicsLoop() ;
      smgfx_SetLook(3,4, .8,0,.8) ;  rSurface.DrawPoles() ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,8, .1,0,0) ;   rSurface.DrawParams() ; sm_GraphicsLoop() ;
      smgfx_SetLook(2,3,  0,0,1) ; sCrvOnSurf.Draw(NULL, TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(2,8,  0,0,1) ; sCrvOnSurf.DrawParams() ; sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

  // Moved up from end of routine, where they can be missed:
  if(GetContext() == NULL && rSurface.GetContext() != NULL)
    { SetContext(rSurface.GetContext()) ; }

  if(GetContext() == NULL && rUVCurve.GetContext() != NULL)
    { SetContext(rUVCurve.GetContext()) ; }

  // set m_vUVDomain 
  m_vUVDomain =   (cpUVDomain) 
                ? *cpUVDomain 
                : m_pSurface->GetNaturalUVDomain() ;

  // m_lOwnerFlag 1 or 2: Copy Curve to be deleted when SmCrvOnSurf is destructed
  if (   lOwnerFlag == 1 
      || lOwnerFlag == 2) 
    {
      const SmContext * pContext = GetContext();
      if (pContext == NULL) { SE(SM_ERR); return; }
      m_pUVCurve->Copy(*pContext, m_pUVCurve) ; 
      // owner of Copy updated in next section
    }

  // m_lOwnerFlag 2: Copy Surface to be deleted when SmCrvOnSurf is destructed
  if (lOwnerFlag == 2) 
    {
      const SmContext * pContext = GetContext();
      if (pContext == NULL) { SE(SM_ERR); return; }
      m_pSurface->Copy(*pContext, m_pSurface) ;
      // owner of Copy updated in next section
    }

  // when Curve or Surface is owned (copied or not) - set the obj's owner to be this
  if(lOwnerFlag == 1 || lOwnerFlag == 2 || lOwnerFlag == 3)
    {
      m_pUVCurve->SetOwner(this) ; 
    }
  if(lOwnerFlag == 2 || lOwnerFlag == 3)
    {
      m_pSurface->SetOwner(this) ; 
    }

  // when passed a context - use it
  if(cpContext)
  {
    SM_ASSERT( GetContext() == NULL || GetContext() == cpContext );
    SetContext( cpContext );

    if(m_pUVCurve->GetContext() == NULL)
    {
      m_pUVCurve->SetContext( cpContext );
    }
  }

  // check state
  SM_ASSERT(GetContext() != NULL) ;

} // end SmCrvOnSurf::SetAll

/*******************************************************************//**
PURPOSE: constructor - UVLineSegment on a Surface

NOTES: returns a SmCrvOnSurf with
          o. m_pUVCurve owned (deleted when this new SmCrvOnSurf is deleted)
          o. m_pSurface not owned (not deleted when this new SmCrvOnSurf is deleted)
***********************************************************************/
SmCrvOnSurf::SmCrvOnSurf
 (SmPoint2d        & rStartUV,     // in : Start point of pUVCurve UVline
  SmPoint2d        & rEndUV,       // in : End   point of pUVCurve UVline
  SmSurface        & rSurface,     // in : Base Surface
  const SmExtent2d * cpUVDomain,   // in : Domain of interest on the surface, NULL = NaturalUVDomain  
  ULONG              lOwnerFlag,   // in : 1 = saves rSurface and owns new m_pUVCurve and only deletes m_pUVCurve when destructed
                                   //      2 = copies rSurface and deletes copy and m_pUVCurve when destructed
                                   //      3 = saves rSurface and deletes rSurface and m_pUVCurve when destructed
  const SmContext  * cpContext)    // in : Context for object construction
 : SmCurve( 3 )
{
  // select context
  SM_ASSERT(   (GetContext() == NULL && cpContext != NULL)
            || (GetContext() != NULL && cpContext == NULL)
            || (GetContext() != NULL && cpContext != NULL && GetContext() == cpContext)) ;
  const SmContext * pContext = cpContext ? cpContext : GetContext() ;
  if (pContext == NULL) { SE(SM_ERR); return; }

  // new UVLine
  SmLine *pNewLine = NULL ; 
  SmLine::CreateLineSegment(*pContext, 2, rStartUV, rEndUV, pNewLine) ;

  // pSurface
  SmSurface *pSurface = &rSurface ;

  // m_lOwnerFlag 2: Copy Surface to be deleted when SmCrvOnSurf is destructed
  if (lOwnerFlag == 2) 
    { rSurface.Copy(*pContext, pSurface) ; }

  // set all the values - use lOwnerFlag == 0 to just store rSurface and pNewLine values without copy - we'll fix ownership next
  SetAll(*pNewLine, rSurface, cpUVDomain, 0, pContext) ;

  // adjust the lOwnerFlag - always delete m_pUVCurve and delete m_pSurface when asked when destructed
  m_lOwnerFlag =   lOwnerFlag == 0 ? 1    // don't-delete m_pSurface, delete m_pUVCurve
                 : lOwnerFlag == 1 ? 1    // don't-delete m_pSurface, delete m_pUVCurve
                 : lOwnerFlag == 2 ? 2    //    do-delete m_pSurface, delete m_pUVCurve
                 : lOwnerFlag == 3 ? 3    //    do-delete m_pSurface, delete m_pUVCurve
                 :                   1 ;  // don't-delete m_pSurface, delete m_pUVCurve 

} // end constructor - UVLineSegment on a Surface

/*******************************************************************//**
PURPOSE: Copy constructor for SmCrvOnSurf object.

NOTES: The constructed object will make deep copies of both the target
  curve's m_pUVCurve and m_pSurface when ever the target curve being copied
  owns either.  When the target curve just has an unowned reference to either
  then an unowned reference will be placed in the constructed object.
***********************************************************************/
SmCrvOnSurf::SmCrvOnSurf
  (const SmCrvOnSurf & crCurveToCopy)
 : SmCurve(crCurveToCopy), 
   m_pUVCurve(crCurveToCopy.m_pUVCurve),
   m_pSurface(crCurveToCopy.m_pSurface), 
   m_vUVDomain(crCurveToCopy.m_vUVDomain),
   m_bNeedBreaks(crCurveToCopy.m_bNeedBreaks),
   m_lOwnerFlag(crCurveToCopy.m_lOwnerFlag),
   m_pBSApproxCurve(crCurveToCopy.m_pBSApproxCurve)
{
  // figure out what the target curve owns - those things will be deep copied
  SmBoolean bTargetOwnsCurve   = (   m_lOwnerFlag == 1
                                  || m_lOwnerFlag == 2
                                  || m_lOwnerFlag == 3) ;
  SmBoolean bTargetOwnsSurface = (   m_lOwnerFlag == 2
                                  || m_lOwnerFlag == 3) ;   
                                  
  // copy Curve when its owned by this object
  if (bTargetOwnsCurve) 
    {
      const SmContext * pContext = GetContext();
      if (pContext == NULL) 
        { SE(SM_ERR); return; }

      // copy the UVCurve
      crCurveToCopy.m_pUVCurve->Copy(*pContext, m_pUVCurve) ;
      m_pUVCurve->SetOwner(this) ;
    }

  // copy Surface when its owned by this object
  if (bTargetOwnsCurve || bTargetOwnsSurface) 
    {
      const SmContext * pContext = GetContext();
      if (pContext == NULL) 
        { SE(SM_ERR); return; }

      // copy the Surface
      crCurveToCopy.m_pSurface->Copy(*pContext, m_pSurface) ;
      m_pSurface->SetOwner(this) ;
    }

  // set the owner flag
  m_lOwnerFlag =   (bTargetOwnsCurve == TRUE && bTargetOwnsSurface == TRUE) ? 2
                 : (bTargetOwnsCurve == TRUE) ? 1
                 : 0 ;

  // copy curve discontinuity break points when appropriate
  if(m_bNeedBreaks == FALSE) 
    { m_vBreaks = crCurveToCopy.m_vBreaks ; }

  if (m_pBSApproxCurve)
  {
      crCurveToCopy.m_pBSApproxCurve->Copy(*GetContext(), m_pBSApproxCurve);
      m_pBSApproxCurve->SetOwner(this);
      m_dMaxGap3d = crCurveToCopy.m_dMaxGap3d;
      m_bParametrizationMatches = crCurveToCopy.m_bParametrizationMatches;
  }
  
} // end SmCrvOnSurf::SmCrvOnSurf copy constructor

/*******************************************************************//**
PURPOSE: Equality operator for SmCrvOnSurf

NOTES: Call base equivalence to check type and then check 
       members for equivalence
***********************************************************************/
SmBoolean SmCrvOnSurf::operator==
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
      SmCrvOnSurf &rOther = (SmCrvOnSurf &)crOther ;

      // check equivalence of these objects
      bRtn &= (   ( m_pUVCurve == rOther.m_pUVCurve)
               || ( m_pUVCurve == NULL && rOther.m_pUVCurve == NULL)
               || (   m_pUVCurve != NULL && rOther.m_pUVCurve != NULL
                   && *m_pUVCurve == *rOther.m_pUVCurve)) ;
                    
      bRtn &= (   ( m_pSurface == rOther.m_pSurface)
               || ( m_pSurface == NULL && rOther.m_pSurface == NULL)
               || (   m_pSurface != NULL && rOther.m_pSurface != NULL
                   && *m_pSurface == *rOther.m_pSurface)) ;

      bRtn &= m_vUVDomain == rOther.m_vUVDomain ;
    }

  // all done
  return bRtn ;

} // end SmCrvOnSurf::operator==

/*******************************************************************//**
PURPOSE: Copy a Curve on Surface.

NOTES: 
***********************************************************************/
SmStatus SmCrvOnSurf::Copy
  (const SmContext & crContext,
   SmCurve        *& rpNewCurve) 
  const
{
  rpNewCurve = new (crContext) SmCrvOnSurf(*this);
  NER(rpNewCurve);
  return SM_SUCCESS;

} // end SmCrvOnSurf::Copy

/*******************************************************************//**
PURPOSE: Destructor for the SmCrvOnSurf object.  It cleans up
    memory for the UV curve and surface if it owns them.

NOTES: 
***********************************************************************/
SmCrvOnSurf::~SmCrvOnSurf()
{
  // delete contained structures when appropriate
  if (m_pUVCurve && m_lOwnerFlag > 0) 
    { delete m_pUVCurve; m_pUVCurve = NULL ; }

  if (m_pSurface && m_lOwnerFlag > 1) 
    {  delete m_pSurface; m_pSurface = NULL ; }

  if (m_pBSApproxCurve)
      {  delete m_pBSApproxCurve; m_pBSApproxCurve = NULL ; }

} // end SmCrvOnSurf::~SmCrvOnSurf destructor

/*******************************************************************/ /**
 PURPOSE: Approximate a curve with a B-Spline using a hermite based
    approximation algorithm. This can be used to reduce the number of
    control points of heavy BSpline curves.

 NOTES:
    1. on output, rpNewBSplineCurve->Dimension == 3,
       When creating UVTrimCurves, call ConvertTo2D()

    2. arguments:
    - bOptMatchParameterization: default:[FALSE]
         TRUE = create a curve with matching parameterization between breakparams.
         FALSE= the parameterization of the approximation can slip
                a little with respect to the true curve between break point, although not very much.
                The parameterization is still exact at every final breakparam,
                which are the ends of all segments into which the approximation
                is tessellated.  So the overall parameterization is still quite close.
                Allowing the parameterization to slip results in somewhat fewer control
                points, but generally not a dramatic difference.

      Use True only if a parameter-to-parameter match
        (to within the given approximation tolerance) is important to your application.

      If the resulting curve has more control points than the input curve, then
      a simple copy of the input curve is returned.
 ***********************************************************************/
SmStatus SmCrvOnSurf::ApproximateCurve(const SmContext& crContext, // in : memory context for new object
                                   const SmTArray<double>& crBreakParams, // in : input curve params exactly
                                                                          // interpolated by approx curve.
                                                                          //      Always representing Interval end
                                                                          //      points and commonly also representing
                                                                          //      orig curve discontinuities.
                                   double dApproxTol3D, // in : Max dist allowed between approx and original curves.
                                   double& rdAchievedTol, // out: Actual Max dist between approx and original curves.
                                                          //      note: value is not exact, only based on sampling.
                                   SmBSplineCurve*& rpNewBSplineCurve, // out: the new BSpline (always Dim==3)
                                   SmBoolean bOptCreateAnalytics, // in : TRUE = create derived type analytics (SmLine,
                                                                  // SmCircle, ...) when possible
                                                                  //      FALSE= don't, default:[TRUE]
                                   SmBoolean bOptMatchParameterization, // in : TRUE = approxCurve(param) within
                                                                        // 3DApproxTol of ThisCurve(param) for all param
                                                                        // values
                                                                        //      FALSE= ApproxCurve(param) within 3d
                                                                        //      ApproxTol of ThisCurve(AnyParam)
                                                                        //      default:[FALSE], FALSE produces lower
                                                                        //      control point count curves for slightly
                                                                        //      more cost.
                                   SmBoolean bJustCopyBSplines) // in : TRUE = if BSplineCurve or derived from
                                                                // BSplineCurve, just copy it (preserves Crv_TYPE and
                                                                // CrvParams)
                                                                //      FALSE= approximate BSplineCurves (changes
                                                                //      CrvParams), NonBSplineCrvs always Approximated
                                                                //      default:[FALSE]
    const
{
      SM_REF2(bJustCopyBSplines, bOptCreateAnalytics);

      SmBSplineCurve* pBSCurve;
      SmStatus eStat = SM_CONST_CAST(SmCrvOnSurf*, this)
              ->GetOrCreateBSApproxPointer(
                  pBSCurve, dApproxTol3D, SM_CONST_CAST(SmTArray<double>*, &crBreakParams), &bOptMatchParameterization);
      if(pBSCurve)
          pBSCurve->Copy(crContext, rpNewBSplineCurve);
      rdAchievedTol = this->GetBSApproxMaxGap3d();
      return eStat;

} // end SmCrvOnSurf::ApproximateCurve

/*******************************************************************//**
PURPOSE: Edit the parameterization of this curve.

NOTES: 
***********************************************************************/
SmStatus SmCrvOnSurf::EditParameterization
  (const SmExtent1d & crNewParameterization, // in : new parameter range for curve
   SmBoolean bNotify)                        // in : TRUE  = make notify calls (previous behavior)
                                             //      FALSE = Skip notify call
                                             //      default:[TRUE]
{ 
  Notify(SM_NO_PRE_EDIT, this, SM_NO_GET_OWNER(this), NULL);

  // when appropriate scale breaks
  if(m_bNeedBreaks == FALSE)
    {
      ULONG ii ;
      SmExtent1d sIvl = m_pUVCurve->GetNaturalInterval() ;

      if(   sIvl.GetLength() > SM_EFF_ZERO
         && crNewParameterization.GetLength() > SM_EFF_ZERO)
        {
          // for every break
          for(ii=0;ii<m_vBreaks.GetSize();ii++)
            {
              m_vBreaks[ii] =   ((m_vBreaks[ii] - sIvl.GetMin()) / sIvl.GetLength())
                              * crNewParameterization.GetLength()
                              + crNewParameterization.GetMin() ;
            } // end iter every break
        } // end nonZero intervals check
      else // working with zero length intervals
        {
          m_vBreaks.ReSet() ;
          m_bNeedBreaks = TRUE ;
        }
    } // end scale breaks check

  // pass the call along to m_pUVCurve              
  SER(m_pUVCurve->EditParameterization(crNewParameterization, bNotify));
  
  // call notify on SmCrvOnSurf container
  if( bNotify == TRUE )
    { 
      Notify(SM_NO_POST_EDIT, this, SM_NO_GET_OWNER(this), NULL);
    }

  // all done
  return SM_SUCCESS;

} // end SmCrvOnSurf::EditParameterization

/*******************************************************************//**
PURPOSE: Given the parameteric value of the curve determine the
    corresponding Euclidian point and optionally derivatives.

NOTES: For discontinuous points on the curve the derivatives
    may be obtained either from the left or from the right of the 
    discontinuity.
***********************************************************************/
SmStatus SmCrvOnSurf::Evaluate
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

  // For subscripting into the surface pt/deriv array:
#define ss(u,v,n) (u*(n+1)+v)

  // Curve eval locals
  SmPoint3d  sUVCurve3dPt;
  SmBoolean  bOnlyUpperHalf = TRUE;
  SmVector3d aUVCrvPtDerivs[ 1 + MAX_DERIVS ];
  SmVector3d aSrfPtDerivs[ (1+MAX_DERIVS) * (1+MAX_DERIVS) ];

#ifdef SM_DEBUG_CODE
  if(   !m_pUVCurve->GetNaturalInterval().ContainsValue(dParameter, SM_EFF_ZERO))
    {
      SmExtent1d sIvl = m_pUVCurve->GetNaturalInterval() ;
      SM_ASSERT_MSG(   sIvl.ContainsValue(dParameter, SM_EFF_ZERO),
                    _T("SmCrvOnSurf found a param for not contained in Curve Natural Interval.")) ;
    }
#endif // SM_DEBUG_CODE

  // Curve eval
  SER( m_pUVCurve->Evaluate( dParameter, 
                             lNumDerivsToEval, 
                             bFromLeft,
                             aUVCrvPtDerivs,
                             bNonZeroTangents ));

  // surface eval locals
  SmPoint2d sUVCurvePt( aUVCrvPtDerivs[0].x, aUVCrvPtDerivs[0].y );

#ifdef SM_DEBUG_CODE
  SmExtent2d sDomain = m_pSurface->GetNaturalUVDomain() ;
  static constexpr double sdRelTol = 0.001;  // check 1/1000 domain size.
  if(   ! sDomain.ContainsPoint2dRelative( sUVCurvePt, sdRelTol )
     && !m_pSurface->IsKindOf(SmOffsetSurface_TYPE))
    {
#ifdef SM_NMTLIB_7166 // computing CrvCrv GapFunctions in problem cases is generating lots of these errors.
                      // in Fillet Case 231: it's because the fillet rail curves are being built badly
                      // For now - skip these error messages until after the consistent tolerance model
                      //           changes are released - then come back and debug the broken cases.     
      SM_ASSERT_MSG(    sDomain.ContainsPoint2dRelative( sUVCurvePt, sdRelTol )
                    || m_pSurface->IsKindOf(SmOffsetSurface_TYPE),   // these are extendable surfaces
                    _T("SmCrvOnSurf found a param for which the CurveUV is not contained in the Surface Domain.")) ;
#endif // SM_NMTLIB_7166
    }
#endif // SM_DEBUG_CODE

  // Surface eval - clamps UVCurvePt to m_pSurface domain when IsOutOfBoundsEnabled() == FALSE
  SmStatus eStat = m_pSurface->Evaluate ( sUVCurvePt,
                                          lNumDerivsToEval, 
                                          lNumDerivsToEval, 
                                          bFromLeft, 
                                          bFromLeft,
                                          bOnlyUpperHalf, 
                                          aSrfPtDerivs,
                                          bNonZeroTangents );

  // If the surface can't handle this many derivatives:
  while ( eStat != SM_SUCCESS && lNumDerivsToEval > 0 )
    {
      lNumDerivsToEval--;

      // try a Surface Eval with fewer derivatives
      eStat = m_pSurface->Evaluate ( sUVCurvePt,
                                     lNumDerivsToEval, 
                                     lNumDerivsToEval, 
                                     bFromLeft, 
                                     bFromLeft,
                                     bOnlyUpperHalf, 
                                     aSrfPtDerivs );
    } // end while looking for surface max derivative count
  SER( eStat );

  // set outputs

  // Position
  aPointAndDerivatives[0] = aSrfPtDerivs[0];

  // 1st Derivs
  if ( lNumDerivsToEval >= 1 )
    {
      aPointAndDerivatives[1] =  // lift 1st derivs from uv to xyz:
           aUVCrvPtDerivs[1].x * aSrfPtDerivs[ ss(1,0,lNumDerivsToEval) ]
         + aUVCrvPtDerivs[1].y * aSrfPtDerivs[ ss(0,1,lNumDerivsToEval) ];
    }
  
  // 2nd Derivs
  if ( lNumDerivsToEval >= 2 )
    {
      aPointAndDerivatives[2] = smsurf_LiftSecondDerivative
                                  ( aUVCrvPtDerivs[1], 
                                    aUVCrvPtDerivs[2],

                                    aSrfPtDerivs[ ss(1,0,lNumDerivsToEval) ],
                                    aSrfPtDerivs[ ss(0,1,lNumDerivsToEval) ],

                                    aSrfPtDerivs[ ss(2,0,lNumDerivsToEval) ],
                                    aSrfPtDerivs[ ss(1,1,lNumDerivsToEval) ],
                                    aSrfPtDerivs[ ss(0,2,lNumDerivsToEval) ] ) ;
    }
  
  // 3rd Derivs
  if ( lNumDerivsToEval >= 3 )
    {
      aPointAndDerivatives[3] = smsurf_LiftThirdDerivative
                                  ( aUVCrvPtDerivs[1], 
                                    aUVCrvPtDerivs[2], 
                                    aUVCrvPtDerivs[3],

                                    aSrfPtDerivs[ ss(1,0,lNumDerivsToEval) ],
                                    aSrfPtDerivs[ ss(0,1,lNumDerivsToEval) ],

                                    aSrfPtDerivs[ ss(2,0,lNumDerivsToEval) ],
                                    aSrfPtDerivs[ ss(1,1,lNumDerivsToEval) ],
                                    aSrfPtDerivs[ ss(0,2,lNumDerivsToEval) ],

                                    aSrfPtDerivs[ ss(3,0,lNumDerivsToEval) ],
                                    aSrfPtDerivs[ ss(2,1,lNumDerivsToEval) ],
                                    aSrfPtDerivs[ ss(1,2,lNumDerivsToEval) ],
                                    aSrfPtDerivs[ ss(0,3,lNumDerivsToEval) ] ) ;
    }

  // If too many requested, fill with zeros.
  for ( ULONG ii = MAX_DERIVS+1; ii <= lNumDerivatives; ii++ )
    {
      aPointAndDerivatives[ii].Set( 0, 0, 0 );
    }

  // all done
  return SM_SUCCESS;

} // end SmCrvOnSurf::Evaluate

/*******************************************************************//**
PURPOSE: Given the parametric value of the curve determine the
    corresponding Euclidian point on the curve.      

NOTES: This methods assumes the curve is not discontinuous
    in its position.  The parameter must be within the natural range
    of the curve.
***********************************************************************/
SmStatus SmCrvOnSurf::EvaluatePoint
  (double dParameter, 
   SmPoint3d & rPoint) 
 const
{ 
  SmPoint3d sPnt ;

  // UVCurve eval
  SER(m_pUVCurve->EvaluatePoint(dParameter,sPnt)) ;

  // surface eval
  SmPoint2d sUV(sPnt.x,sPnt.y) ;
  SER(m_pSurface->EvaluatePoint(sUV,rPoint)) ;

  // all done
  return SM_SUCCESS ;

} // end SmCrvOnSurf::EvaluatePoint

/*******************************************************************//**
PURPOSE: Given the parametric value of the curve determine the
    corresponding Surface UV Point.      

NOTES: To get the associated 3D point call SmCrvOnSurf::EvaluatePoint()
***********************************************************************/
// evaluate SurfUV = curve(dParameter)
SmStatus SmCrvOnSurf::EvaluateUVPoint
 (double      dParameter, 
  SmPoint2d & rUVPoint ) 
 const
{
  // local
  SmPoint3d sPnt;

  // Eval UVPoint
  SER( m_pUVCurve->EvaluatePoint( dParameter, sPnt ) );

  // set output
  rUVPoint.Set( sPnt.x, sPnt.y );

  // all done
  return SM_SUCCESS;

} // end SmCrvOnSurf::EvaluateUVPoint

/*******************************************************************//**
PURPOSE: Given the parameteric value of the curve determine the
    corresponding Surface point and derivatives.

NOTES: For discontinuous points on the curve the derivatives
    may be obtained either from the left or from the right of the 
    discontinuity.
***********************************************************************/
SmStatus SmCrvOnSurf::EvaluateSurface
 (double      dParameter,       // in : tgt param
  ULONG       lHighestUDeriv,   // in : number of U derivatives            
  ULONG       lHighestVDeriv,   // in : number of V derivatives to compute 
  SmBoolean   bFromLeft,        // in : if P is on interval boundary
                                //      TRUE  = evaluate P in upper interval where P is on the left of the interval
                                //      FALSE = evaluate P in lower interval where P is on the right of the interval
  SmBoolean   bOnlyUpperHalf,   // in : TRUE=compute upper half of matrix only
                                //      ex. 1,1 = [D  Du] 2,2 = [D    Du    Duu] where -- = an untouched memory value 
                                //                [Dv --]       [Dv   Duv   ---]          (the memory has to be allocated)
                                //                              [Dvv  ---   ---]
  SmVector2d  aUV[2],           // out: UVTrimCurve UVposition and UVtangent
  SmVector3d *aDerivatives,     // out: matrix of evaluations values
                                //      sized:[lHighestUDeriv+1][lHighestVDeriv+1]
                                //      2d organized: [D    Du    Duu    Duuu    Duuuu   ]  (the same no matter the value of)
                                //                    [Dv   Duv   Duuv   Duuuv   Duuuuv  ]  (  bOnlyUpperHalf               )
                                //                    [Dvv  Duvv  Duuvv  Duuuvv  Duuuuvv ]
                                //                    [Dvvv Duvvv Duuvvv Duuuvvv Duuuuvvv]
                                //      1d organized: [D, Dv, Dvv, Dvvv,.. Du, Duv, Duvv, Duvvv,.. Duu, Duuv, Duuvv, Duuvvv,...]
  SmBoolean   bNonZeroTangents) // in : TRUE = replace zero tangent vectors with properly oriented tol sized vectors
          const                 //      FALSE= return exact tangent values
                                //      note: Surprisingly TRUE is the common choice because most tangent uses
                                //            are for their direction (Binorm, SurfNorm comps), but when the 
                                //            tangent is being used for its magnitude (like an arc-length comp)
                                //            then set this to FALSE.
                                //      default:[TRUE]
{ 
  // Curve eval locals
  SmVector3d aUVCrvPtDerivs[2];

#ifdef SM_DEBUG_CODE
  if(   !m_pUVCurve->GetNaturalInterval().ContainsValue(dParameter, SM_EFF_ZERO))
    {
      SmExtent1d sIvl = m_pUVCurve->GetNaturalInterval() ;
      SM_ASSERT_MSG(   sIvl.ContainsValue(dParameter, SM_EFF_ZERO),
                    _T("SmCrvOnSurf found a param for not contained in Curve Natural Interval.")) ;
    }
#endif // SM_DEBUG_CODE

  // Curve eval
  SER( m_pUVCurve->Evaluate(dParameter, 1, bFromLeft, aUVCrvPtDerivs, FALSE ));  // exact tangent values

  // set outputs
  aUV[0].Set( aUVCrvPtDerivs[0].x, aUVCrvPtDerivs[0].y );
  aUV[1].Set( aUVCrvPtDerivs[1].x, aUVCrvPtDerivs[1].y );

#ifdef SM_DEBUG_CODE

  if(   !m_pSurface->GetNaturalUVDomain().ContainsPoint2d(aUV[0], SM_EFF_ZERO)
     && !m_pSurface->IsKindOf(SmOffsetSurface_TYPE))
    {
      SmExtent2d sDomain = m_pSurface->GetNaturalUVDomain() ;
      SM_ASSERT_MSG(   sDomain.ContainsPoint2d(aUV[0], SM_EFF_ZERO)
                    || m_pSurface->IsKindOf(SmOffsetSurface_TYPE),   // these are extendable surfaces
                    _T("SmCrvOnSurf found a param for which the CurveUV is not contained in the Surface Domain.")) ;
    }

//  SmExtent2d sDomain = m_pSurface->GetNaturalUVDomain() ;
//  if(   ! sDomain.ContainsPoint2dRelative( aUV[0], 0.001 )  // check 1/1000 domain size.
//     && !m_pSurface->IsKindOf(SmOffsetSurface_TYPE))
//    {
//      SM_ASSERT_MSG(   sDomain.ContainsPoint2dRelative(aUV[0], 0.001)
//                    || m_pSurface->IsKindOf(SmOffsetSurface_TYPE),   // these are extendable surfaces
//                    _T("SmCrvOnSurf found a param for which the CurveUV is not contained in the Surface Domain.")) ;
//    }
#endif // SM_DEBUG_CODE

  // Surface eval
  SmStatus eStat = m_pSurface->Evaluate(aUV[0],
                                        lHighestUDeriv, 
                                        lHighestVDeriv, 
                                        bFromLeft, 
                                        bFromLeft,
                                        bOnlyUpperHalf, 
                                        aDerivatives,
                                        bNonZeroTangents );
  // all done
  return eStat ;

} // end SmCrvOnSurf::EvaluateSurface

/*******************************************************************//**
PURPOSE: Given the parameteric value of the curve determine the
    corresponding Surface Normal.

NOTES: 
***********************************************************************/
SmStatus SmCrvOnSurf::EvaluateSurfaceNormal
 (double dParameter,             // in : tgt param
  SmBoolean bFromLeft,           // in : if P is on interval boundary
                                 //      TRUE  = evaluate P in upper interval where P is on the left of the interval
                                 //      FALSE = evaluate P in lower interval where P is on the right of the interval
  SmVector2d aUV[2],             // out: UVTrimCurve UVposition and UVtangent
  SmVector3d &rSurfaceNormal)    // out: Surface normal
{ 
  // Curve eval locals
  SmVector3d aUVCrvPtDerivs[2];

#ifdef SM_DEBUG_CODE
  if(   !m_pUVCurve->GetNaturalInterval().ContainsValue(dParameter, SM_EFF_ZERO))
    {
      SmExtent1d sIvl = m_pUVCurve->GetNaturalInterval() ;
      SM_ASSERT_MSG(   sIvl.ContainsValue(dParameter, SM_EFF_ZERO),
                    _T("SmCrvOnSurf found a param for not contained in Curve Natural Interval.")) ;
    }
#endif // SM_DEBUG_CODE

  // Curve eval
  SER( m_pUVCurve->Evaluate(dParameter, 1, bFromLeft, aUVCrvPtDerivs, FALSE ));  // exact tangent values

  // set outputs
  aUV[0].Set( aUVCrvPtDerivs[0].x, aUVCrvPtDerivs[0].y );
  aUV[1].Set( aUVCrvPtDerivs[1].x, aUVCrvPtDerivs[1].y );

#ifdef SM_DEBUG_CODE
  if(   !m_pSurface->GetNaturalUVDomain().ContainsPoint2d(aUV[0], SM_EFF_ZERO)
     && !m_pSurface->IsKindOf(SmOffsetSurface_TYPE))
    {
      SmExtent2d sDomain = m_pSurface->GetNaturalUVDomain() ;
      SM_ASSERT_MSG(   sDomain.ContainsPoint2d(aUV[0], SM_EFF_ZERO)
                    || m_pSurface->IsKindOf(SmOffsetSurface_TYPE),   // these are extendable surfaces
                    _T("SmCrvOnSurf found a param for which the CurveUV is not contained in the Surface Domain.")) ;
    }
//  SmExtent2d sDomain = m_pSurface->GetNaturalUVDomain() ;
//  static double sdRelTol = 0.001;  // check 1/1000 domain size.
//  if(   ! sDomain.ContainsPoint2dRelative( aUV[0], sdRelTol )
//     && !m_pSurface->IsKindOf(SmOffsetSurface_TYPE))
//    {
//      SM_ASSERT_MSG(   sDomain.ContainsPoint2dRelative( aUV[0], sdRelTol )
//                    || m_pSurface->IsKindOf(SmOffsetSurface_TYPE),   // these are extendable surfaces
//                    _T("SmCrvOnSurf found a param for which the CurveUV is not contained in the Surface Domain.")) ;
//    }
#endif // SM_DEBUG_CODE

  // Surface eval
  SmStatus eStat = m_pSurface->EvaluateNormal( aUV[0],
                                               bFromLeft, 
                                               bFromLeft,
                                               rSurfaceNormal);
  // all done
  return eStat;

} // end SmCrvOnSurf::EvaluateSurfaceNormal

/*******************************************************************//**
PURPOSE: Calculate the derivatives of the surface's first derivatives
   with respect to the curve parameter.

NOTES: Currently we do only one derivative; if requested, higher
   derivatives are set to zero.
***********************************************************************/
SmStatus SmCrvOnSurf::EvaluateSurface1stDerivDerivs
 (double       dT,            // in : tgt param
  ULONG        lNumDers,      // in : number of derivatives (w.r.t. t) required.
  SmBoolean    bFromLeft,     // in : if P is on interval boundary
                              //      TRUE  = evaluate P in upper interval where P is on the left of the interval
                              //      FALSE = evaluate P in lower interval where P is on the right of the interval
  SmVector3d * aSurfaceUDers, // out: Derivs of Su w.r.t. t
  SmVector3d * aSurfaceVDers) // out: Derivs of Sv w.r.t. t
{ 
  // Evaluation locals
  SmVector3d aUVCrvPtDer[2];
  SmVector3d sS, sSu, sSv, sSuu, sSuv, sSvv;

  // Curve eval
  SER( m_pUVCurve->Evaluate( dT, 1, bFromLeft, aUVCrvPtDer, TRUE ));  // non-zero tangent values
  SmPoint2d  sUV  = SmPoint2d( aUVCrvPtDer[0].x, aUVCrvPtDer[0].y );
  SmVector3d sUVt = aUVCrvPtDer[1];

  // Surface eval
  SER( m_pSurface->Evaluate2ndDerivatives( sUV, bFromLeft, bFromLeft, sS, sSu, sSv, sSuv, sSuu, sSvv ));

  // set outputs
  aSurfaceUDers[0] = sSu;
  aSurfaceVDers[0] = sSv;

  aSurfaceUDers[1] = sUVt.x * sSuu  +  sUVt.y * sSuv;
  aSurfaceVDers[1] = sUVt.x * sSuv  +  sUVt.y * sSvv;

  if ( lNumDers > 1 )
    {
      ULONG ii;
      for ( ii = 2; ii > lNumDers; ii++ )
        {
          aSurfaceUDers[ii].Set( 0,0,0 );
          aSurfaceVDers[ii].Set( 0,0,0 );
        }
    }

  // all done
  return SM_SUCCESS;

} // end SmCrvOnSurf::EvaluateSurface1stDerivDerivs


/*******************************************************************/ /**
 PURPOSE: Calculate the bbox of a CrvOnSurf by using a bspline approximation to
 to the 3d curve. If we fail to produce a bspline approximation, we fall back to the SmCurve bbox.

 NOTES: Will build a bspline approximation of the 3d curve if it does not already exist.

 ***********************************************************************/
SmStatus SmCrvOnSurf::CalculateBoundingBox(const SmExtent1d& crInterval, // in : target interval of interest - this is used
                                       SmExtent3d* pBBox, // out: Axis alligned box, NULL to ignore, default:[NULL]
                                       SmPseudoBox* pPseudoBox, // out: Non-axis aligned box, NULL to ignore,
                                                                // default:[NULL]
                                       SmPolarBox* pPolarBox, // out: Curve tangent vector field bounding box, NULL to
                                                              // ignore, default:[NULL]
                                       SmBoolean bExpandBox) // in : TRUE = expand BBoxes prior to return
                                                             //      FALSE= don't
                                                             //      default:[TRUE]
  const
{

  SmStatus eStat = SM_SUCCESS;

  SmBSplineCurve* pBSCurve = NULL;

  eStat = SM_CONST_CAST(SmCrvOnSurf*, this)->GetOrCreateBSApproxPointer(pBSCurve);

  if (eStat != SM_SUCCESS)
  {
      return this->SmCurve::CalculateBoundingBox(crInterval, pBBox, pPseudoBox, pPolarBox, bExpandBox);
  }
  else
  {
      return pBSCurve->CalculateBoundingBox(crInterval, pBBox, pPseudoBox, pPolarBox, bExpandBox);
  }
} // end SmCrvOnSurf::CalculateBoundingBox

/*******************************************************************//**
PURPOSE: Calculate the minimum continuity of the curve and the 
         continuities at each of the unique knots
    
NOTES: SmCrvOnSurf does not really have knots but it does have 
    discontinuities which include the knots of the defining UVCurve curve
    and every UVCurve/SurfaceKnotIsoParameterLine intersection point.
    
    Currently, this function gathers all the discontinuity points together
    and assigns to each an arbitrary SM_CT_C1 value.  That rating should
    be computed by inheriting the UVTrimCurve properties projected through 
    the surface.  For now, that's not been done.  
     
    The output includes a coninuity value for the start and end of the 
    curve interval and all internal discontinuities. The minimum continuity 
    of the curve will be be the minimum continuity of the internal knots. 
    The end knots will always be discontinuous.

***********************************************************************/
SmStatus SmCrvOnSurf::CalculateContinuities
  (SmContinuityType           & reMinContinuityInCurve,  // out: min continuity of all internal knots 
   SmTArray<SmContinuityType> & rContinuitiesAtKnots,    // out: continuity at every knot value for curve
   double                       dContinuityAngleTol)     // in : 
  const
{
  // init output
  rContinuitiesAtKnots.ReSet() ;
  reMinContinuityInCurve = SM_CT_UNDEFINED ;

  // locals
  SmContinuityType eThisContinuity ;
  SmVector3d sThisEval[4], sOtherEval[4] ;

  // get effective knots and continuities
  SmTArray<double> sKnots ;
  GetKnots(sKnots) ;

  // start and end knots
  rContinuitiesAtKnots.SetSize(sKnots.GetSize()) ;
  rContinuitiesAtKnots.SetAt(0, SM_CT_DISCONTINUOUS) ;
  rContinuitiesAtKnots.SetAt(sKnots.GetSize()-1, SM_CT_DISCONTINUOUS) ;

  // build the output
  ULONG ii ;
  for(ii=1;ii+1<sKnots.GetSize();ii++) // note: can't say sKnots.GetSize()-1
    {
      // evaluate geometric continuities
      EvaluateContinuity(sKnots[ii], TRUE, 
                        *this, sKnots[ii], FALSE,
                         eThisContinuity,
                         sThisEval, sOtherEval, 
                         dContinuityAngleTol) ;

      // set output
      rContinuitiesAtKnots.SetAt(ii, eThisContinuity) ;  
      
      // save min continuity                                 
      if (   reMinContinuityInCurve == SM_CT_UNDEFINED
          || reMinContinuityInCurve > eThisContinuity)   
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

} // end SmCrvOnSurf::CalculateContinuities

/*******************************************************************//**
PURPOSE: Create a new curve which is a mirror of the curve about the
    X, Y plane of the SmAxis2Placement.

NOTES: 

METHOD: Mirrors the supporting surface, UVCurve and domain are copied. 
***********************************************************************/
SmStatus SmCrvOnSurf::CreateMirrorCurve
  (
      const SmContext&        crContext,     ///< [in] :     <br>
      const SmAxis2Placement& crMirrorPlane, ///< [in] :     <br>
            SmCurve*&         rpMirrorCurve  ///< [out]:     <br>
  ) //const
{
    // locals
    SmCurve   * pNewCurve   = NULL;
    SmSurface * pMirrorSurf = NULL;

    // Create the new geometry
    SER(m_pSurface->CreateMirrorSurface(crContext, crMirrorPlane, pMirrorSurf));
    m_pUVCurve->Copy(crContext, pNewCurve);

    // Create the new curve on mirrored geometry. It owns the curve and surf
    rpMirrorCurve = new (crContext) SmCrvOnSurf(*pNewCurve, *pMirrorSurf, &m_vUVDomain, 3);

    return SM_SUCCESS;
}

/*******************************************************************//**
PURPOSE: Create a curve by projecting an existing curve into
    a plane using either parallel or perspective projection.

NOTES: If parallel projection then the direction of the 
    projection can not be perpendicular to the plane normal.  If 
    perspective projection the center of projection can not lie on
    the plane.

METHOD: Lifts curve to an SmBSplineCurve, then calls
    SmBSplineCurve::CreatePlaneProjection
***********************************************************************/
SmStatus SmCrvOnSurf::CreatePlaneProjection
(
    const SmContext  & crContext,                 ///< [in] : context for new object construction                                                 <br>
    SmProjectionType   eProjectionType,           ///< [in] : oneof SM_PT_PARALLEL, SM_PT_PERSPECTIVE                                             <br>
    const SmPoint3d  & rProjectionPlanePoint,     ///< [in] : Point on target plane                                                               <br>
    const SmVector3d & rProjectionPlaneNormal,    ///< [in] : Normal of target plane                                                              <br>
    const SmVector3d & rProjDirOrCenterOfProj,    ///< [in] : When eProjectionType == SM_PT_PARALLEL, vector is projection direction.             <br>
                                                  ///<      : When eProjectionType == SM_PT_PERSPECTIVE, vector is 'eye' point of the projection. <br>
    SmCurve         *& rpProjectedCurve           ///< [out]:                                                                                     <br>
) const
{
    // Locals
    SmBSplineCurve * p3dBSCurve = NULL;
    SmBSplineCurve * pBSUVCurve = SM_CAST_PTR( SmBSplineCurve, m_pUVCurve ); NER( pBSUVCurve );
    SmApproxTol3d    sApproxTol = SmTol::GetApproxTol3d( this );
    double dDist = 0.0;

    // Lift this curve to a BSplineCurve
    SER( m_pSurface->LiftCurve( crContext,                           // in : context for new object construction
                                m_vUVDomain,                         // in : this surface limit
                               *pBSUVCurve,                          // in : 2d Parameter curve defined in surface parameter space to lift
                                pBSUVCurve->GetNaturalInterval(),    // in : curve segment to lift
                                sApproxTol,                          // in : max allowed distance between output curve and Surface
                                dDist,                               // out: max dist from output curve to surface
                                p3dBSCurve ) );                      // out: 3d Curve = Surface(crUVCurveToLift(crInterval))
                                                                     // in : TRUE = this Surface is at least C1 continuous and
                                                                     //             internal surface C0 continuity checks are skipped.
                                                                     //      FALSE= check Surface for C0 discontinuities -
                                                                     //             break lifted curve at each such point
                                                                     //      default:[FALSE]

    // Pass along the call to SmBSplineCurve::CreatePlaneProjection
    return p3dBSCurve->CreatePlaneProjection( crContext, eProjectionType, rProjectionPlanePoint, rProjectionPlaneNormal, rProjDirOrCenterOfProj, rpProjectedCurve );
}

/*******************************************************************//**
PURPOSE: Drop a point onto this curve using LocalPointSolve(with GuessParam) or
         GlobalPointSolve(without GuessParam) and snap to endPoints when asked.

NOTES:

***********************************************************************/
SmStatus SmCrvOnSurf::DropPoint
  (const SmExtent1d & crInterval,         // in : target curve allowed domain
   const SmPoint3d  & crPointToDrop,      // in : Point to drop to curve
   const SmVector3d * cpVecToCrvInside,   // in : For Snapping, NULL=NoSnap, Vec pointing to CrvInside from desired EndPt for snapping.
                                          //      when DropPt is within dDistTol in cpVecToCrvInside dir of EndPt, Snap to Ivl EndPt.
                                          //      Vec handles ambiguities and makes sure that curves are not just touching at the ends.
   double             dDistTol,           // in : Used for SnapDist, and SM_SO_INTERSECT sXSectTol3d distance.
                                          //      sXSectTol3d = dDistTol ;      SM_SO_INTERSECT 
                                          //      sXSectTol3d = SM_BIG_DOUBLE ; SM_SO_MINIMIZE, SM_SO_NORMALIZE
                                          //      sSnapTol3d  = dDistTol ;      SM_SO_MINIMIZE, SM_SO_NORMALIZE, SM_SO_INTERSECT    
   const double     * pOptGuessParam,     // in : If given, call cheap LocalPointSolve() else call expensive GlobalPointSolve()
   SmBoolean        & rbSuccess,          // out: TRUE = found a drop point
   double           & rdDroppedParameter, // out: found drop curve param
   double           & rdDistanceToCurve,  // out: found drop distance
   SmSolverOperationType eOperationType)  // in : SM_SO_MINIMIZE = allow nonNormal drops near endPoints
 const                                    //      SM_SO_INTERSECT= like Minimize but point must be within dDistTol
                                          //      SM_SO_NORMALIZE= exclude nonNormal drops near endPoints
                                          //      default:[SM_SO_MINIMIZE] to preserve original behavior
{
  // init output
  rbSuccess = FALSE;
  rdDroppedParameter = 0.0;
  rdDistanceToCurve  = 0.0;

  // check inputs
  if(   eOperationType != SM_SO_MINIMIZE
     && eOperationType != SM_SO_INTERSECT
     && eOperationType != SM_SO_NORMALIZE)
    {
      WARN(_T("Bad DropPoint eOperationType - selecting eOperationType = SM_SO_MINIMIZE")) ;
      eOperationType = SM_SO_MINIMIZE ;
    }

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
  // draw PointToDrop(red), TargetCurve(blue)
  if (bDebugMe)
    {
      Dump();

      smgfx_SetLook(3,5, 1,0,0); crPointToDrop.Draw(); sm_GraphicsLoop();
      smgfx_SetLook(3,5, 0,0,1); DrawWDeriv(crInterval,0); sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

  if (pOptGuessParam)
    {
      return this->SmCurve::DropPoint(crInterval, crPointToDrop, cpVecToCrvInside, dDistTol, pOptGuessParam, rbSuccess,
                                      rdDroppedParameter, rdDistanceToCurve, eOperationType);
    } // given a guessPoint branch

  SmBSplineCurve* pBSCurve;
  SmStatus eStat = SM_CONST_CAST(SmCrvOnSurf*, this)->GetOrCreateBSApproxPointer(pBSCurve);

  if (eStat != SM_SUCCESS)
  {
      return this->SmCurve::DropPoint(crInterval, crPointToDrop, cpVecToCrvInside, dDistTol, pOptGuessParam, rbSuccess,
                                      rdDroppedParameter, rdDistanceToCurve, eOperationType);
  }

  SmTArray<SmPoint3d> sControlPoints;
  SmTArray<double> sWeights;

  pBSCurve->GetControlPolygon(sControlPoints, sWeights);

  double dMin = SM_BIG_DOUBLE;
  ULONG lMinIndex = 0;

  for (ULONG ii = 0; ii < sControlPoints.GetSize(); ii++)
  {
      double dDistSq = sControlPoints[ii].DistanceBetweenSquared(crPointToDrop);
      if (dDistSq < dMin)
      {
          dMin = dDistSq;
          lMinIndex = ii;
      }
  }

  const double dMinParam = crInterval.SnapValue(pBSCurve->GetGrevilleAbscissa(lMinIndex));

  return this->SmCurve::DropPoint(crInterval, crPointToDrop, cpVecToCrvInside, dDistTol, &dMinParam, rbSuccess,
                                  rdDroppedParameter, rdDistanceToCurve, eOperationType);

} // end SmCrvOnSurf::DropPoint


/*******************************************************************//**
PURPOSE: Get the knots and optionally the multiplicities of this curve.

NOTES: SmCrvOnSurf returns the list of discontinuity break points as
 the list of knots.  That includes all the knots of the m_pUVCurve
 plus the param values of every m_pUVCurve intersection with the
 surface discontinuities (typically the isoparameter curves defined by
 the knot vectors of the surface).
***********************************************************************/
SmStatus SmCrvOnSurf::GetKnots
  (SmTArray<double> & rKnots,               // out: unique knot list
   SmTArray<ULONG>  * pKnotMultiplicities,  // out: multiplicity reported for each knot
   const SmExtent1d * pOptIvl)              // in : interval of interest, NULL=Natural Interval, default:[NULL] 
  const
{
  // init output
  rKnots.ReSet() ;
  if(pKnotMultiplicities) { pKnotMultiplicities->ReSet() ; }

  // low work
  if ( m_bNeedBreaks == FALSE )
  {
      ULONG lNumKts = m_vBreaks.GetSize();

      // JGU: As per Sol, a crash was happening when lNumKts < 2. Can't get part to check. [201204]
      // How could m_vBreaks.GetSize() < 2? Based on call hierachy, Fixing crash implies size == 0, 
      // but rKnots[0] use below suggests at one time it had at least one entry.
      // So, if less than 2 knots, something happened to m_vBreaks. Refresh it. 
      if ( lNumKts >= 2 )
      {
          // copy saved breaks array
          rKnots = m_vBreaks;

          if ( pKnotMultiplicities != NULL )
          {
              // Set these up as if degree 3.
              pKnotMultiplicities->Add( 4 );
              pKnotMultiplicities->InsertAt( 1, 2, lNumKts - 2 );
              pKnotMultiplicities->Add( 4 );
          }
          return( SM_SUCCESS );
      }
  } // end low work

  // locals
  ULONG ii, jj ;

  // pick a UVSpace tolerance
  double dTol2d = SM_EFF_ZERO * ( 1.0 + m_pSurface->GetNaturalUVDomain().GetMaxDimension() ) ;

  // get the UVCurve knots and multiplicities
  m_pUVCurve->GetKnots(rKnots, pKnotMultiplicities, pOptIvl) ;

  // modify the mult list to act as if pUVCurve is a degree 3 BSpline
  ULONG lUVDegree = m_pUVCurve->GetDegree() ;
  
  // when adjusting is needed
  if(lUVDegree != 3 && pKnotMultiplicities)
    {
      for(ii=0;ii<pKnotMultiplicities->GetSize();ii++)
        {
          ULONG lDelMult =  ( lUVDegree < pKnotMultiplicities->GetAt(ii) + 3)
                          ?  pKnotMultiplicities->GetAt(ii) + 3 - lUVDegree
                          :  1 ;
          pKnotMultiplicities->SetAt(ii, lDelMult) ;  
        }
    } 

  // classify the UVTrimCurve against the surface knot isoParameter lines.
  SmCurveClassification sCurveClassification(m_pUVCurve, 
                                             m_pUVCurve->GetNaturalInterval(),
                                             NULL,
                                             dTol2d,
                                             0, NULL, FALSE, TRUE, m_pSurface) ;
  // make the classification
  SmContext sContext ;
  SER(sCurveClassification.IntersectUVDiscontinuties(*m_pSurface, sContext)) ;

  // what we should have here is the list of surface multiplicities 
  //  that associate with each classification boundary
  //  until that is done - just treat each UVCurve/SurfaceIsoLine intersection as a C1 discontiinuity

  // add the classification boundaries uniquely and in order to the pknot and mult lists
  ULONG lCnt     = sCurveClassification.GetSize() ;
  double dUVKnot = rKnots[0] ;

  // for every Classification boundary (include the ends in case pOptIvl != NULL) - add ClassBoundaries to knot arrays
  for(ii=0,jj=0;ii<=lCnt;ii++)
    {
      double dClassBndry =   (ii < lCnt) 
                           ? (sCurveClassification.GetAt(ii).m_vInterval.GetMin())
                           : (sCurveClassification.GetAt(ii-1).m_vInterval.GetMax()) ;

      // when given pOptIvl - skip ClassKnots outside the requested interval
      if(pOptIvl)
        { 
          // skip too small param values
          if(pOptIvl->GetMin() > dClassBndry + dTol2d) { continue ; }

          // quit when a too large param value is seen (dClassBndry is ordered and only larger param values are coming)
          if(pOptIvl->GetMax() < dClassBndry - dTol2d) { break ; }

        } // end given pOptIvl check

      // check the next UVKnots until we get one bigger or equal to dClassBndry
      while ( dClassBndry > dUVKnot + dTol2d) 
        { jj++ ;
          dUVKnot = (jj == rKnots.GetSize()) ? SM_BIG_DOUBLE : rKnots[jj] ;
        }

      // when dClassBndry value is already in rKnots - done with this ClassKnot - move onto next ClassKnot
      if ( dClassBndry  > dUVKnot - dTol2d) 
        { continue ; }

      // arrive here when classKnot needs to be added to Knot array at index jj
      // add class boundary as if it were a C1 knot into the m_pUVCurve
      rKnots.InsertAt(jj, dClassBndry, 1) ;
      if(pKnotMultiplicities) { pKnotMultiplicities->InsertAt(jj, 2, 1) ; }

      // assuming the UVCurve knots and the Classification boundaries are both in order and unique 
      // skip to next UVKnot and continue
      jj += 1 ;
      if (jj >= rKnots.GetSize()) { jj = rKnots.GetSize() - 1 ; }
      dUVKnot = rKnots[jj] ; 

    } // end iter every Classification boundary

  // cache the discontinuity break points
  ((SmCrvOnSurf *)this)->m_vBreaks     = rKnots ; 
  ((SmCrvOnSurf *)this)->m_bNeedBreaks = FALSE ; 

  // all done
  return SM_SUCCESS;

} // end SmCrvOnSurf::GetKnots
    
/*******************************************************************//**
PURPOSE: Get the 'knots' of this curve.

NOTES: These are not B-Spline knots, they are locations of potential
   discontinuities in the curve, caused by both curve knots and possible
   discontinuities in the underlying surface.
***********************************************************************/
ULONG SmCrvOnSurf::GetNumberNaturalKnots()            
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

} // end SmCrvOnSurf::GetNumberNaturalKnots

/*******************************************************************/ /**
 PURPOSE: Get or create cache BSpline approximation of the 3d CrvOnSurf

 NOTES: Greatly speeds bbox computation.

 ***********************************************************************/
SmStatus SmCrvOnSurf::GetOrCreateBSApproxPointer
        (SmBSplineCurve  *& pBSCurve,
         double             dApproxTol,
         SmTArray<double> * pOptBreaks,
         SmBoolean        * pOptParameterizationMatches)
{
  SmStatus eStat = SM_SUCCESS;

  double dMaxGap3d = this->GetBSApproxMaxGap3d();

  SmBoolean bMatchParam = pOptParameterizationMatches ? *pOptParameterizationMatches : TRUE;

  SmBoolean bRecompute = FALSE;

  // If we need breaks and have been given them, or have breaks and they are not the same,
  // then we will build a bspline using the supplied breaks.
  if ((m_bNeedBreaks && pOptBreaks) || (pOptBreaks && !((*pOptBreaks) == m_vBreaks)))
  {
      bRecompute = TRUE;

      m_vBreaks.Copy((*pOptBreaks));
      m_bNeedBreaks = FALSE;
  }

  if (bMatchParam != m_bParametrizationMatches)
  {
      bRecompute = TRUE;
  }

  if (bRecompute)
  {
      if (m_pBSApproxCurve)
      {
          delete m_pBSApproxCurve;
          m_pBSApproxCurve = NULL;
      }
  }

  // If we have an approximation and the tolerance is good enough, use it
  if (m_pBSApproxCurve && dMaxGap3d <= dApproxTol)
  {
      pBSCurve = m_pBSApproxCurve;
  }
  else // otherwise make one
  {
      if (m_bNeedBreaks)
      {
          SmTArray<ULONG> sMults;
          SmTArray<double> sKnots;
          this->GetKnots(sKnots, &sMults);
          m_vBreaks.Copy(sKnots);
          m_bNeedBreaks = FALSE;
      }
      if (m_pBSApproxCurve)
      {
          delete m_pBSApproxCurve;
          m_pBSApproxCurve = NULL;
      }

      m_bParametrizationMatches = bMatchParam;

      eStat = this->SmCurve::ApproximateCurve(*GetContext(), m_vBreaks, dApproxTol, m_dMaxGap3d, m_pBSApproxCurve, TRUE, bMatchParam, FALSE);
      pBSCurve = m_pBSApproxCurve;
  }

  // all done
  return eStat;

} // end SmCrvOnSurf::GetOrCreateBSApproxPointer

/*******************************************************************//**
PURPOSE: search this object and all its contained objs to find
         one that references a Context

NOTES: 
***********************************************************************/
const SmContext * SmCrvOnSurf::FindContext() const                     
{ 
  return(  GetContext() ? GetContext()
         : m_pUVCurve && m_pUVCurve->GetContext() ? m_pUVCurve->GetContext()
         : m_pUVCurve && m_pUVCurve->GetOwner() && m_pUVCurve->GetOwner()->GetContext() ? m_pUVCurve->GetOwner()->GetContext()
         : m_pSurface && m_pSurface->GetContext() ? m_pSurface->GetContext()
         : m_pSurface && m_pSurface->GetOwner() && m_pSurface->GetOwner()->GetContext() ? m_pSurface->GetOwner()->GetContext()
         : NULL) ;
} // end SmCrvOnSurf::FindContext

/*******************************************************************//**
PURPOSE: 

NOTES: 
***********************************************************************/
void SmCrvOnSurf::SetUVCurve
 (SmCurve *cpUVCurve)       
{ 
#ifdef SM_DEBUG_CODE
  if(cpUVCurve != NULL && cpUVCurve->GetDim() != 2)
    {
      SM_ASSERT_MSG(cpUVCurve == NULL || cpUVCurve->GetDim() == 2, _T("SmCrvOnSurf::SetUVCurve passed a non-2D UVTrimCurve")) ;
    }
#endif // SM_DEBUG_CODE

  m_pUVCurve = cpUVCurve; 

} // end SmCrvOnSurf::SetUVCurve
    
/*******************************************************************//**
PURPOSE: Reverse the parameterization of this curve.

NOTES: 
***********************************************************************/
SmStatus SmCrvOnSurf::ReverseParameterization
  (const SmExtent1d & crOldInterval,  // in : current curve interval       
   SmExtent1d       & rNewInterval)   // out: curve interval after reversal
{ 
  Notify(SM_NO_PRE_EDIT, this, SM_NO_GET_OWNER(this), NULL);

  // Don't own curve can not reverse parameterization
  if (m_lOwnerFlag == 0) 
    { SER(SM_ERR) ; }

  // pass the call along to the UVCurve
  SER(m_pUVCurve->ReverseParameterization(crOldInterval,rNewInterval));
   
  Notify(SM_NO_POST_EDIT, this, SM_NO_GET_OWNER(this), NULL);

  return SM_SUCCESS;

} // end SmCrvOnSurf::ReverseParameterization

/*******************************************************************//**
PURPOSE: Trim this curve.

NOTES: 
***********************************************************************/
SmStatus SmCrvOnSurf::Trim
 (SmExtent1d & crTrimInterval,  // i/o: desired new interval, can be snapped by tol to existing knots
  SmBoolean    bNotify,         // in : internal use only - use default, default:[TRUE]
                                //      TRUE  = call Notify after trimming (previous behavior)
                                //      FALSE = skip Notify after trimming
                                //      UNSURE= skip notify, skip trimming, just recompute TrimInterval
  SmBoolean    bSkipDebugCheck) // in : internal use only - use default, default:[FALSE]
                                //      FALSE= in debug mode silently run this->AssertValid()
                                //      TRUE = don't run AssertValid() before returning
{ 
  // Don't own curve can not trim the curve
  if (m_lOwnerFlag == 0) 
    { SER(SM_ERR) ; }

  if(bNotify != UNSURE)
    { Notify(SM_NO_PRE_EDIT, this, SM_NO_GET_OWNER(this), NULL) ; }

  // pass the call along to the UVCurve
  SER(m_pUVCurve->Trim(crTrimInterval, bNotify, bSkipDebugCheck)) ; // may snap sIvl by tol to existing knots

  // clear the knots
  if(m_bNeedBreaks == FALSE)
    {
      m_vBreaks.ReSet() ;
      m_bNeedBreaks = TRUE ;
    }

  if(bNotify == TRUE)
    { Notify(SM_NO_POST_EDIT, this, SM_NO_GET_OWNER(this), NULL) ; }

  return SM_SUCCESS;

} // end SmCrvOnSurf::Trim

/*******************************************************************//**
PURPOSE: Transform the CrvOnSurf object.  Only need to transform it
    if it owns the base surface.  Otherwise it just follows its surface.
    We do need to clean up any nurb that it carries.

NOTES: 
   This is a 3d transform, so we would not move the uv curve,
   just transform the surface and we're done.
***********************************************************************/
SmStatus SmCrvOnSurf::Transform
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

  // Transform cached bspline approximation
  if (m_pBSApproxCurve)
  {
      m_pBSApproxCurve->Transform(crRotateNMove, cpOptScale);
  }

  // If we don't own the surface, the owner will transform it.
  if (m_lOwnerFlag < 2) 
    { return SM_SUCCESS; }

  Notify(SM_NO_PRE_EDIT, this, SM_NO_GET_OWNER(this), NULL);

  // pass the call along to the surface
  SER(m_pSurface->Transform(crRotateNMove,cpOptScale));

  Notify(SM_NO_POST_EDIT, this, SM_NO_GET_OWNER(this), NULL);

  return SM_SUCCESS;

} // end SmCrvOnSurf::Transform

/*******************************************************************//**
PURPOSE: Local helper to keep code cleaner.
   Calculate the distance and deviation at a point on this curve.

NOTES: 
   Distance  == distance from surface along normal
   Deviation == distance from surface normal in surface tangent plane
   These are the two components of the gap vector, w.r.t. the surface normal.
***********************************************************************/
static SmStatus sm_EvalDistAndDev
 (const SmCrvOnSurf *pSurfCrv,     // in : CrvOnSurf to inspect
        double       dParam1,      // in : Param for pSurfCrv
  const SmCurve     *cp3dCurve,    // in : Other Curve
        double      *pdGuess2,     // in : GuessParam for cp3dCurve
  const SmExtent1d  &crIvl2,       // in : Interval for cp3dCurve
        double       dDropTol3d,   // in : min dist between distinct solutions
        SmBoolean   &rbSuccess,    // out: TRUE = call to cp3dCurve->DropPoint() worked, FALSE=didn't
        double      &rdOtherParam, // out: Param drop point value
        double      &rdDist,       // out: component distance along SurfNormal of DropVec
        double      &rdDev )       // out: component distance perp to SurfNormal of DropVec
{
  // init outputs
  rbSuccess = FALSE;
  rdDist = rdDev = 0;
  rdOtherParam = *pdGuess2;

  // Evaluate surface point and normal at the given parameter.
  SmPoint3d sSurfPt;
  SmVector3d sNorm;
  SmPoint2d sUVPtDer[2];
  SmCrvOnSurf *pNonConstCrv = SM_CONST_CAST( SmCrvOnSurf*, pSurfCrv ); NER( pNonConstCrv );
  SER( pNonConstCrv->EvaluateSurfaceNormal( dParam1, TRUE, sUVPtDer, sNorm ));
  const SmSurface *pSrf = pSurfCrv->GetBaseSurface(); NER( pSrf );
  SER( pSrf->EvaluatePoint( sUVPtDer[0], sSurfPt ));

  // Find point on 3d curve that would drop here: Intersect it with Surface Pt/Normal.
  SmLine sLine( sSurfPt, sNorm, 3, FALSE, pSurfCrv->GetContext() );
  SmExtent1d sLineDom( -1000, 1000 );
  SmBoolean bNeedsMore;
  SmSolutionArray sSolutions;
  SER( cp3dCurve->IntersectWithLine( crIvl2, sLine, sLineDom, dDropTol3d, bNeedsMore, sSolutions ));

  double d3dCrvParam = 0.0;
  SmPoint3d s3dCrvPt;

  if ( sSolutions.GetSize() > 0  &&  ! bNeedsMore )
    {
      // In case of multiple solutions, take the closest one.
      ULONG ii;
      double dMinDist = SM_BIG_DOUBLE;

//cbi breakpoint:
if ( sSolutions.GetSize() > 1 )
  { dMinDist /= 2.0; }

      for ( ii=0; ii<sSolutions.GetSize(); ii++ )
      {
          double dThisParam = sSolutions[ii].m_vStart[0];
          SER( cp3dCurve->EvaluatePoint( dThisParam, s3dCrvPt ));
          SmVector3d sGap( s3dCrvPt - sSurfPt );
          double dDist = smos_Fabs( sGap.Dot( sNorm ) );  // sNorm is unit, so this is 3d distance -- signed.
          if ( dDist < dMinDist )
          {
              dMinDist = dDist;
              d3dCrvParam = dThisParam;
          }
      }
    }
  else  // If that didn't work, use DropPoint.
    {
      // Drop that point to the other curve.
      double d3dCrvDist;

      SER(cp3dCurve->DropPoint(crIvl2,        // in : target curve allowed domain
                               sSurfPt,       // in : Point to drop to curve
                               NULL,          // in : For Snapping, NULL=NoSnap, Vec pointing to CrvInside from desired EndPt for snapping.
                                              //      when DropPt is within dDistTol in cpVecToCrvInside dir of EndPt, Snap to Ivl EndPt.
                                              //      Vec handles ambiguities and makes sure that curves are not just touching at the ends.
                               dDropTol3d,    // in : Used for SnapDist, and SM_SO_INTERSECT sXSectTol3d distance.
                                              //      sXSectTol3d = dDistTol ;      SM_SO_INTERSECT 
                                              //      sXSectTol3d = SM_BIG_DOUBLE ; SM_SO_MINIMIZE, SM_SO_NORMALIZE
                                              //      sSnapTol3d  = dDistTol ;      SM_SO_MINIMIZE, SM_SO_NORMALIZE, SM_SO_INTERSECT    
                               pdGuess2,      // in : If given, call cheap LocalPointSolve() else call expensive GlobalPointSolve()
                               rbSuccess,     // out: TRUE = found a drop point
                               d3dCrvParam,   // out: found drop curve param
                               d3dCrvDist)) ; // out: found drop distance
                                              // in : SM_SO_MINIMIZE = allow nonNormal drops near endPoints
                                              //      SM_SO_INTERSECT= like Minimize but point must be within dDistTol
                                              //      SM_SO_NORMALIZE= exclude nonNormal drops near endPoints
                                              //      default:[SM_SO_MINIMIZE] to preserve original behavior
      if ( !rbSuccess )
        { return SM_SUCCESS; }
    }

  SER( cp3dCurve->EvaluatePoint( d3dCrvParam, s3dCrvPt ));

  // Get the gap and find its two components: parallel and perp to the normal.
  SmVector3d sGap( s3dCrvPt - sSurfPt );

  rdDist = sGap.Dot( sNorm );  // sNorm is unit, so this is 3d distance -- signed.

  SmPoint3d sNormPt( sSurfPt + rdDist * sNorm );
  rdDev  = s3dCrvPt.DistanceBetween( sNormPt );

  rdOtherParam = d3dCrvParam;

  rdDist = smos_Fabs( rdDist );  // for return.
  rbSuccess = TRUE;

  return SM_SUCCESS;

} // end local sm_EvalDistAndDev

/*******************************************************************//**
PURPOSE: Find the max distance and max deviation between OtherCurve and thisCurve
         making no assumptions about curve parameterizations.

NOTES:
   rdMaxDistFound is signed, in case the caller cares about above/below
   the tangent plane.  We leave it to the caller to take the
   absolute value if appropriate.

   lNumSamples specifies the minimum number of sample points to check
     and the returned values is the largest gap seen.
   When lNumSamples == 0, a slow global solution is used to find the
     maximum gap vector, and the result will be very precise.

   When pdOptMaxDistanceNeeded is specified, the value returned in
     rdMaxDistanceFound will not necessarily be the actual max distance,
     it will be the first gap found that is bigger than *pdOptMaxDistanceNeeded.

METHOD ---
   Sample OtherCurve uniformly from its Start to its End finding
   the nearest point on ThisCurve to each samplePoint. Set
   rdMaxDistanceFound = to max gap vector size.

   You should be able
   to determine if they are coincident segments or just at the ends
   using this routine.  If the optional max distance needed is not null the
   testing will stop as soon as a point is found which is greater than
   this value.  It will save us some time.
***********************************************************************/
SmStatus SmCrvOnSurf::SurfaceCurveMaxDistanceBetween
  (const SmExtent1d & crInterval,            // in : interval limit for this curve
   const SmCurve    & crOtherCurve,          // in : other curve to test
   double dOtherCurveParameterAtStartOfThis, // in : OtherCurve param mapping to ThisCurve Interval.Min value
   double dOtherCurveParameterAtEndOfThis,   // in : OtherCurve param mapping to ThisCurve Interval.Max value
   ULONG  lNumSamples,                       // in : Min number of samples to take - it measures at least this many points
                                             //      at a uniform spacing on the otherCurve finding the
                                             //      corresponding ThisCurve closest points.
                                             //      If 0 is given it will do its best to perform
                                             //      a precise measurement and will be much slower.
   double   dDropTolerance3d,                // in : Distance where nearby DropPoint solutions will be considered the same solution
   double * pdOptMaxNeeded,                  // in : Max allowed gap, either Dist or Dev.
                                             //      Quit searching once this value is exceeded.
                                             //      NULL to ignore.   Never quit search when NULL.
   double & rdMaxDistFound,                  // out: Set to signed max distance seen along the surface normal.
                                             //        a. When lNumSamples == 0 This is the max curve/surface gap.
                                             //        b. When pdOptMaxDistanceNeeded this is either the max sampled
                                             //           curve/surface gap which is less than pdOptMaxDistanceNeeded or
                                             //           the 1st gap seen larger than pdOptMaxDistanceNeeded.
   double & rdMaxDevFound,                   // out: Set to max lateral deviation seen.
                                             //        a. When lNumSamples == 0 This is the max curve/curve gap
                                             //           in the surface tangent plane.
                                             //        b. When pdOptMaxDistanceNeeded this is either the max sampled
                                             //           curve/curve gap which is less than pdOptMaxDistanceNeeded or
                                             //           the 1st gap seen larger than pdOptMaxDistanceNeeded.
   double * pOptCurveTDist,                  // out: Curve param for returned MaxDist Found, NULL to ignore.
                                             //      default:[NULL]
   double * pOptOtherTDist,                  // out: OtherCurve param for returned MaxDist Found, NULL to ignore.
                                             //      default:[NULL]
   double * pOptCurveTDev,                   // out: Curve param for returned MaxDev Found, NULL to ignore.
                                             //      default:[NULL]
   double * pOptOtherTDev)                   // out: OtherCurve param for returned MaxDev Found, NULL to ignore.
                                             //      default:[NULL]
  const
{
  // a local
  SmExtent1d sOtherInterval(dOtherCurveParameterAtStartOfThis);
  sOtherInterval.AddValue(dOtherCurveParameterAtEndOfThis);

  // Init outputs
  rdMaxDistFound = rdMaxDevFound = 0.0;
  if(pOptCurveTDist) { *pOptCurveTDist = crInterval.GetMid(); }
  if(pOptOtherTDist) { *pOptOtherTDist = sOtherInterval.GetMid(); }
  if(pOptCurveTDev ) { *pOptCurveTDev  = crInterval.GetMid(); }
  if(pOptOtherTDev ) { *pOptOtherTDev  = sOtherInterval.GetMid(); }

  // No work: curves and intervals are the same.
  if ( &crOtherCurve == this )
    {
      // Intervals must be the same as well.
      if (   (     SM_ARE_SAME( crInterval.GetMin(), dOtherCurveParameterAtStartOfThis )
                && SM_ARE_SAME( crInterval.GetMax(), dOtherCurveParameterAtEndOfThis   )
             )
          || (     SM_ARE_SAME( crInterval.GetMin(), dOtherCurveParameterAtEndOfThis   )
                && SM_ARE_SAME( crInterval.GetMax(), dOtherCurveParameterAtStartOfThis )
             )
         )
        {
          // Already set outputs.
          return SM_SUCCESS;
        }
    } // end no-work check.

  // more locals
  double    dOrientation  = (dOtherCurveParameterAtEndOfThis >= dOtherCurveParameterAtStartOfThis)
                            ?  1.0    // curves run in same direction
                            : -1.0 ;  // curves run in opposite directions
  double    dMaxDist      = 0.0;
  double    dMaxDev       = 0.0;
  SmBoolean bDoPrecise    = FALSE;

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
  // draw Breps(blue,green), Curves(Cyan,red)
  if (bDebugMe)
    {
      Dump();
      crOtherCurve.Dump();

      smgfx_Erase();
      smgfx_SetLook(3,5, 0,1,1); this->GetBaseSurface()->DrawUV(4,4); sm_GraphicsLoop();
      smgfx_SetLook(3,5, 0,0,1); this->Draw(&crInterval); sm_GraphicsLoop();
      smgfx_SetLook(4,6, 0,1,0); crOtherCurve.Draw(&sOtherInterval); sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
#endif

  // Do a very large sample and adjust it upward to compensate for
  // cases where there are a large number of knots.  This still will
  // not guarantee that we get a sample for each span in each curve
  // but it should get one 99.9 percent of the time.
  if (lNumSamples == 0)
    {
      bDoPrecise = TRUE;
      SmTArray<double> sKnots, sOtherKnots;
      GetKnots(sKnots);
      crOtherCurve.GetKnots(sOtherKnots);

      lNumSamples = 22 + sKnots.GetSize() + sOtherKnots.GetSize();
    }

  // special case line/line check
  if (   GetDegree() == 1
      && GetNumberNaturalKnots() == 4
      && crOtherCurve.GetDegree() == 1
      && crOtherCurve.GetNumberNaturalKnots() == 4)
    {
      bDoPrecise = FALSE;
      lNumSamples = 2;
    }

  // init maxGap Point to interval midPoints
  double dThisTMaxDist  = crInterval.GetMid();
  double dOtherTMaxDist = sOtherInterval.GetMid();
  double dThisTMaxDev   = crInterval.GetMid();
  double dOtherTMaxDev  = sOtherInterval.GetMid();

  // Uniformly sample this Curve from beginning to end.
  //   Find Nearest OtherCurve Point to each samplePoint and save max Gaps.
  // Note: sample this curve and drop to the other, because it's probably
  // more expensive to iterate on this curve than on the other.
  double dThisDist = SM_BIG_DOUBLE;
  double dThisDev  = SM_BIG_DOUBLE;

  double dPer, dTGuess1, dTGuess2, dParam2;
  ULONG ii;
  SmBoolean bSuccess;

  for (ii=0; ii<lNumSamples; ii++)
    {
      dPer     = (double)ii / (double)(lNumSamples-1);
      dTGuess1 = crInterval.Evaluate(dPer);
      dTGuess2 =   dOrientation > 0.0
                 ? sOtherInterval.Evaluate(dPer)
                 : sOtherInterval.Evaluate(1.0-dPer);

//cbi smaller intervals: dTGuess2 +- 1 / (lNumSamples-1), truncated to crOtherInterval.
      SER( sm_EvalDistAndDev( this,           // in : CrvOnSurf to inspect
                              dTGuess1,       // in : Param for pSurfCrv
                             &crOtherCurve,   // in : Other Curve
                             &dTGuess2,       // in : GuesParam for cp3dCurve
                              sOtherInterval, // in : Interval for cp3dCurve
                              dDropTolerance3d, // in : Min distance between distinct solutions
                              bSuccess,       // out: TRUE = call to cp3dCurve->DropPoint() worked, FALSE=didn't
                              dParam2,        // out: Param drop point value
                              dThisDist,      // out: component distance along SurfNormal of DropVec
                              dThisDev ));    // out: component distance perp to SurfNormal of DropVec

      // when no solution was found for 1st guess
      if ( ! bSuccess && ii == 0)
        {
          // try moving ThisCurvePoint a small distance from its endPoint
          dTGuess1 = crInterval.Evaluate(0.001);
          SER( sm_EvalDistAndDev( this, dTGuess1, &crOtherCurve, &dTGuess2, sOtherInterval,
                                  dDropTolerance3d, // in : Min distance between distinct solutions
                                  bSuccess, dParam2, dThisDist, dThisDev ));
        }

      // when no solution was found for last guess
      if ( ! bSuccess && ii == lNumSamples - 1)
        {
          // try moving ThisCurvePoint a small distance from its endPoint
          dTGuess1 = crInterval.Evaluate(0.999);
          SER( sm_EvalDistAndDev( this, dTGuess1, &crOtherCurve, &dTGuess2, sOtherInterval,
                                  dDropTolerance3d, // in : Min distance between distinct solutions
                                  bSuccess, dParam2, dThisDist, dThisDev ));
        }

#ifdef SM_DEBUG_CODE
      if (bDebugMe)
        {
          smgfx_Erase();
          smgfx_SetLook(2,3, 0,1,1); DrawWDeriv(crInterval,0); sm_GraphicsLoop();
          smgfx_SetLook(3,4, 1,0,0); crOtherCurve.DrawWDeriv(sOtherInterval,0); sm_GraphicsLoop();
          sm_GraphicsLoop();
        }
#endif
      // Save biggest solution gap vectors
      if ( bSuccess && dThisDist > dMaxDist )
        {
          dMaxDist       = dThisDist;
          dThisTMaxDist  = dTGuess1;
          dOtherTMaxDist = dParam2;
        }
      if ( bSuccess && dThisDev > dMaxDev )
        {
          dMaxDev       = dThisDev;
          dThisTMaxDev  = dTGuess1;
          dOtherTMaxDev = dParam2;
        }

      // when LocalPoint solver failed to find a solution - try GlobalPoint Solver
      // Note: the DropPoint call in sm_EvalDistAndDev() already does this.


      // When given a MaxDistance limit - and the limit is exceeded
      if (   ( pdOptMaxNeeded && dMaxDist > *pdOptMaxNeeded )
          || ( pdOptMaxNeeded && dMaxDev  > *pdOptMaxNeeded ) )
        {
          // save the excess amount and quit
          rdMaxDistFound = dMaxDist;
          if ( pOptCurveTDist ) { *pOptCurveTDist = dThisTMaxDist;  }
          if ( pOptOtherTDist ) { *pOptOtherTDist = dOtherTMaxDist; }

          rdMaxDevFound = dMaxDev;
          if ( pOptCurveTDev ) { *pOptCurveTDev = dThisTMaxDev;  }
          if ( pOptOtherTDev ) { *pOptOtherTDev = dOtherTMaxDev; }

          return SM_SUCCESS;
        }
    } // end iter every sample point

#ifdef SM_DEBUG_CODE
  // draw max gap points found
  if (bDebugMe)
    {
      smgfx_Erase();
      smgfx_SetLook(1,2, 0,0,1); DrawWDeriv(crInterval,0); sm_GraphicsLoop();
      smgfx_SetLook(1,2, 0,1,0); crOtherCurve.DrawWDeriv(sOtherInterval,0); sm_GraphicsLoop();

      SmPoint3d sPnt1, sPnt2;
      SER(EvaluatePoint(dThisTMaxDist,sPnt1));
      SER(crOtherCurve.EvaluatePoint(dOtherTMaxDist,sPnt2));
      SmVector3d sVec = sPnt1 - sPnt2;

      smgfx_SetLook(2,4, 0,0,1); sPnt1.Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook(2,4, 0,1,0); sPnt2.Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,3, 1,0,0); sVec.Draw(&sPnt2) ; sm_GraphicsLoop() ;
      sm_GraphicsLoop();

      SER(EvaluatePoint(dThisTMaxDev,sPnt1));
      SER(crOtherCurve.EvaluatePoint(dOtherTMaxDev,sPnt2));
      sVec = sPnt1 - sPnt2;

      smgfx_SetLook(2,4, 0,0,1); sPnt1.Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook(2,4, 0,1,0); sPnt2.Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,3, 0,1,1); sVec.Draw(&sPnt2) ; sm_GraphicsLoop() ;
      sm_GraphicsLoop();
    }
#endif



//cbi: Currently, the following two CurveSolve() methods are not
//cbi  necessarily working correctly, so just use what the iteration gave us.
//cbi  Accomplish that by doing this:
  bDoPrecise = FALSE;


  // If not asking for precise result, set output and return.
  if ( ! bDoPrecise )
    {
      rdMaxDistFound = dMaxDist;
      if ( pOptCurveTDist ) { *pOptCurveTDist = dThisTMaxDist;  }
      if ( pOptOtherTDist ) { *pOptOtherTDist = dOtherTMaxDist; }

      rdMaxDevFound = dMaxDev;
      if ( pOptCurveTDev ) { *pOptCurveTDev = dThisTMaxDev;  }
      if ( pOptOtherTDev ) { *pOptOtherTDev = dOtherTMaxDev; }

      return SM_SUCCESS;
    }

  // Looking for precise result.
  // When MaxDist found is greater than zero, iterate to get maxDist.
  // Need surface point and normal for that operation.
  SmPoint3d sPt;
  SmVector3d sNorm;
  SmPoint2d sUVPtDer[2];
  const SmSurface *pBaseSrf = this->GetBaseSurface(); NER( pBaseSrf );
  SmCrvOnSurf *pNonConstThis = SM_CONST_CAST( SmCrvOnSurf*, this ); NER( pNonConstThis );
  SER(pNonConstThis->EvaluateSurfaceNormal( dThisTMaxDist, TRUE, sUVPtDer, sNorm ));
  pBaseSrf->EvaluatePoint( sUVPtDer[0], sPt );
  double dScaledZero = SM_EFF_ZERO * (1.0 + sPt.GetMaxDimension());

  if ( dMaxDist > dScaledZero )
    {
      SmSolution sSolution;
      SmBoolean bFoundAnswer;

//cbi smaller interval: dThisTMaxDev +- 1 / (lNumSamples-1), truncated to crOtherInterval.
      // find the curve-surface distance maximum near the largest sampled point
      SmExtent2d sSrfDomain = pBaseSrf->GetNaturalUVDomain();
      SER( pBaseSrf->LocalCurveSolve( sSrfDomain,
                          crOtherCurve,
                          sOtherInterval,
                          SM_SO_MAXIMIZE,
                          SM_EFF_ZERO_SQRT, // Tol  cbi: ??
                          NULL,  // opt distance limit
                          NULL,  // opt vectors
                          sUVPtDer[0],   // uv guess
                          dOtherTMaxDev, //  t guess
                          bFoundAnswer,
                          sSolution));

      // when largest gap maximum was found
      if (bFoundAnswer)
        {
          // get Gap points and sizes
          dThisDist = sSolution.m_vStart.m_dSolutionValue;

          // save largest gap seen
          if (dThisDist > dMaxDist)
            {
              dMaxDist = dThisDist;
              double dTMax2 = sSolution.m_vStart[0];  // other curve param

              dOtherTMaxDist = dTMax2;

              // Find corresponding param on this curve.
              crOtherCurve.EvaluatePoint( dTMax2, sPt );
              double dTMax1 = 0.0, dDist = 0.0;
//cbi smaller interval: dThisTMaxDev +- 1 / (lNumSamples-1), truncated to crInterval.
              this->DropPoint(crInterval,       // in : target curve allowed domain
                              sPt,              // in : Point to drop to curve
                              NULL,             // in : For Snapping, NULL=NoSnap, Vec pointing to CrvInside from desired EndPt for snapping.
                                                //      when DropPt is within dDistTol in cpVecToCrvInside dir of EndPt, Snap to Ivl EndPt.
                                                //      Vec handles ambiguities and makes sure that curves are not just touching at the ends.
                              SM_EFF_ZERO_SQRT, // in : Used for SnapDist, and SM_SO_INTERSECT sXSectTol3d distance.
                                                //      sXSectTol3d = dDistTol ;      SM_SO_INTERSECT 
                                                //      sXSectTol3d = SM_BIG_DOUBLE ; SM_SO_MINIMIZE, SM_SO_NORMALIZE
                                                //      sSnapTol3d  = dDistTol ;      SM_SO_MINIMIZE, SM_SO_NORMALIZE, SM_SO_INTERSECT    
                                                // in : If given, call cheap LocalPointSolve() else call expensive GlobalPointSolve()
                              &dThisTMaxDev,    // out: TRUE = found a drop point
                              bFoundAnswer,     // out: found drop curve param
                              dTMax1,           // out: found drop distance
                              dDist) ;          // in : SM_SO_MINIMIZE = allow nonNormal drops near endPoints
                                                //      SM_SO_INTERSECT= like Minimize but point must be within dDistTol
                                                //      SM_SO_NORMALIZE= exclude nonNormal drops near endPoints
                                                //      default:[SM_SO_MINIMIZE] to preserve original behavior

              if ( bFoundAnswer )
                { dThisTMaxDist = dTMax1; }

            } // end if LocalCurveSolve found a better solution
        } // end if LocalCurveSolve found a solution at all

#ifdef SM_DEBUG_CODE
          if ( bDebugMe )
            {
              SmPoint3d sSrfPt, sCrvPt;
              this->       EvaluatePoint( dThisTMaxDist , sSrfPt );
              crOtherCurve.EvaluatePoint( dOtherTMaxDist, sCrvPt );
              SmVector3d sVec( sCrvPt - sSrfPt );
              smgfx_SetLook(2,4, 0,0,1); sSrfPt.Draw() ; sm_GraphicsLoop() ;
              smgfx_SetLook(2,4, 0,1,0); sCrvPt.Draw() ; sm_GraphicsLoop() ;
              smgfx_SetLook(2,4, 1,0,0); sVec.Draw(&sSrfPt) ; sm_GraphicsLoop() ;
              sm_GraphicsLoop();
            }
#endif

    } // end asked to find precise MaxDist check

  // When MaxDev found is greater than zero, iterate to get maxDev.
  // Need surface point and normal for that operation.
  SmPoint3d sPnt;
  SER( pNonConstThis->EvaluateSurfaceNormal( dThisTMaxDev, TRUE, sUVPtDer, sNorm ));
  this->GetBaseSurface()->EvaluatePoint( sUVPtDer[0], sPnt );
  dScaledZero = SM_EFF_ZERO * (1.0 + sPnt.GetMaxDimension());

  if ( dMaxDev > dScaledZero )
    {
      SmSolution sSolution;
      SmBoolean bFoundAnswer;

      // find the curve-surface normal deviation maximum near the largest sampled point
//cbi smaller intervals: crv params +- 1 / (lNumSamples-1), truncated to crInterval.
      SER(LocalCurveSolve(crInterval,       crOtherCurve,
                          sOtherInterval,   SM_SO_PROJECTED_MAXIMIZE,
                          SM_EFF_ZERO_SQRT, NULL, &sNorm,
                          dThisTMaxDev,
                          dOtherTMaxDev,
                          bFoundAnswer,
                          sSolution));

      // when largest gap maximum was found
      if (bFoundAnswer)
        {
          // get Gap points and size
          double dTMax1   = sSolution.m_vStart[0];
          double dTMax2   = sSolution.m_vStart[1];
          double dThisGap = sSolution.m_vStart.m_dSolutionValue;

          // save largest gap seen
          if (dThisGap > dMaxDev)
            {
              dMaxDev       = dThisGap;
              dThisTMaxDev  = dTMax1;
              dOtherTMaxDev = dTMax2;
            }

#ifdef SM_DEBUG_CODE
          if ( bDebugMe )
            {
              SmPoint3d sSrfPt, sCrvPt;
              SmVector3d sVec( sCrvPt - sSrfPt );
              smgfx_SetLook(2,4, 0,0,1); sSrfPt.Draw() ; sm_GraphicsLoop() ;
              smgfx_SetLook(2,4, 0,1,0); sCrvPt.Draw() ; sm_GraphicsLoop() ;
              smgfx_SetLook(2,4, 1,0,0); sVec.Draw(&sCrvPt) ; sm_GraphicsLoop() ;
              sm_GraphicsLoop();
            }
#endif
        } // end found MaxDist check

    } // end asked to find precise MaxDist check

  // set output and return
  rdMaxDistFound = dMaxDist;
  if ( pOptCurveTDist ) { *pOptCurveTDist = dThisTMaxDist;  }
  if ( pOptOtherTDist ) { *pOptOtherTDist = dOtherTMaxDist; }

  rdMaxDevFound = dMaxDev;
  if ( pOptCurveTDev ) { *pOptCurveTDev = dThisTMaxDev;  }
  if ( pOptOtherTDev ) { *pOptOtherTDev = dOtherTMaxDev; }

  return SM_SUCCESS;

} // end SmCurve::SurfaceCurveMaxDistanceBetween


/*******************************************************************//**
PURPOSE: Compute the total size of the memory used by the SmCrvOnSurf.

NOTES: 
***********************************************************************/
ULONG SmCrvOnSurf::GetMemoryUsed   // rtn: smaller size of actually used memory in bytes
  (ULONG    & rlMemoryAllocated,   // out: bigger size of all allocated memory in bytes
   SmMarkType eMarkType)           // in : uses without increment eMarkType value
  const
{
  // in case this method is called directly - get a mark for attribute memory usage
  SmNewMarkAndLock sMarkLock ;
  if(eMarkType == SM_MT_NOMARK)
    {
      eMarkType = sMarkLock.SetContext((SmContext *)GetContext()) ;
    }

  // locals
  ULONG lUsed, lCurveAllocated=0, lSurfaceAllocated=0 ;

  // this + m_pNurb memory
  lUsed =   sizeof(*this) 
          + (  (m_pUVCurve && m_lOwnerFlag > 0) ? m_pUVCurve->GetMemoryUsed(lCurveAllocated, eMarkType)   : 0)
          + (  (m_pSurface && m_lOwnerFlag > 1) ? m_pSurface->GetMemoryUsed(lSurfaceAllocated, eMarkType) : 0) ;

  rlMemoryAllocated = sizeof(*this) + lCurveAllocated + lSurfaceAllocated ;

  // + attribute memory
  ULONG lThisAllocated ;
  lUsed += this->GetAttributeMemoryUsed(lThisAllocated, 
                                        eMarkType) ;  // note: uses without increment eMarkType value
  rlMemoryAllocated += lThisAllocated ;

  // + cache memory
  if ( m_pCacheObj )
  {
      lUsed += m_pCacheObj->GetMemoryUsed( lThisAllocated );
      rlMemoryAllocated += lThisAllocated;
  }

  // all done
  return(lUsed) ;

} // end SmCrvOnSurf::GetMemoryUsed

/*******************************************************************//**
PURPOSE: Write SmCrvOnSurf to given output stream.

NOTES: 
***********************************************************************/
SmStatus SmCrvOnSurf::WriteToDB
 (SmDatabaseIO & rDB,                // in : target output stream
  ULONG          lDBVersionNumber)   // in : database version to get proper sequence of writes                                                                       
 const
{
  // file type, ASCII or BINARY
  SmFileType      eType    =  rDB.GetFileType();
  std::ostream  & rFileOut = *rDB.GetOutStreamPtr();

  // curve locals
  
  if (eType == SM_ASCII) 
    {
      rFileOut <<        m_vUVDomain.GetUMin() 
               << " " << m_vUVDomain.GetVMin() 
               << " " << m_vUVDomain.GetUMax() 
               << " " << m_vUVDomain.GetVMax() << " SmCrvOnSurf Domain of interest \n" ;
      rFileOut << m_bNeedBreaks         << " SmCrvOnSurf Has breaks flag \n" ;
    }
  else 
    {
      SER(rDB.WriteDouble ( m_vUVDomain.GetUMin())) ;  
      SER(rDB.WriteDouble ( m_vUVDomain.GetVMin())) ;  
      SER(rDB.WriteDouble ( m_vUVDomain.GetUMax())) ;  
      SER(rDB.WriteDouble ( m_vUVDomain.GetVMax())) ;  
      SER(rDB.WriteBoolean( m_bNeedBreaks)) ;  
    }

  // parameter space curve
  ULONG lUVDim = m_pUVCurve->GetDim() ;
  if (eType == SM_ASCII) { rFileOut << " CrvOnSurf->UVCurve \n"; }
  SER(rDB.WriteType(m_pUVCurve->GetType(), &lUVDim)) ;
  SER(m_pUVCurve->WriteToDB(rDB, lDBVersionNumber)) ;

  // compounding surface
  if (eType == SM_ASCII) { rFileOut << " CrvOnSurf->Surface \n"; }
  SER(rDB.WriteType(m_pSurface->GetType())) ; 
  SER(m_pSurface->WriteToDB(rDB, lDBVersionNumber)) ;

  // when appropriate - breaks
  if(m_bNeedBreaks == FALSE)
    {
      // number of breaks
      if (eType == SM_ASCII) { rFileOut << m_vBreaks.GetSize() << " Number of Curve internal discontinuities\n" ; }
      else /* Binary */      { SER(rDB.WriteLong(m_vBreaks.GetSize())) ; }

      // for every break
      ULONG ii ; 
      if (eType == SM_ASCII) { for(ii=0;ii<m_vBreaks.GetSize();ii++ ) { rFileOut << m_vBreaks[ii] << " " ; }
                               if(m_vBreaks.GetSize() > 0) { rFileOut << " \n" ; }
                             }
       else                  { for(ii=0;ii<m_vBreaks.GetSize();ii++ ) { SER(rDB.WriteDouble(m_vBreaks[ii])) ; }
                             }
    }  

  // all done
  return SM_SUCCESS;

} // end SmCrvOnSurf::WriteToDB

/*******************************************************************//**
PURPOSE: static method to Read a SmCrvOnSurf from a given stream  

NOTES: 
***********************************************************************/
SmStatus SmCrvOnSurf::ReadFromDB
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
         || rpNewCurve->IsKindOf(SmCrvOnSurf_TYPE)) ? SM_SUCCESS : SM_ERR) ;

  // init output object
  SmCrvOnSurf *pCrvOnSurf =   (rpNewCurve == NULL)
                            ? new (crContext) SmCrvOnSurf()
                            : (SmCrvOnSurf *)rpNewCurve ;
  // file type
  SmFileType     eType   = rDB.GetFileType();
  std::istream & rFileIn = *rDB.GetInStreamPtr();
      
  // locals
  SmPoint2d        sMin, sMax ; 
  SmBoolean        bNeedBreaks ;

  if (eType == SM_ASCII) 
    {
      rFileIn >> sMin.x >> sMin.y >> sMax.x >> sMax.y  ; rDB.GoToNextLine() ;
      rFileIn >> bNeedBreaks ;                           rDB.GoToNextLine() ;
    }
  else 
    {
      SER(rDB.ReadDouble(sMin.x));
      SER(rDB.ReadDouble(sMin.y));
      SER(rDB.ReadDouble(sMax.x));
      SER(rDB.ReadDouble(sMax.y));
      SER(rDB.ReadBoolean(bNeedBreaks));
    }

  // curve locals
  SmCurve *pUVCurve=NULL ;
  SM_TYPE  lUVType ;  
  ULONG    lUVDim ;

  // UV curve
  if (eType == SM_ASCII) { rDB.GoToNextLine() ; }
  SER(rDB.ReadType(lUVType, &lUVDim)) ; 
  SER(SmCurve::ReadFromDB(lUVType, rDB, lUVDim, crContext, pUVCurve, lDBVersionNumber)) ; 

  // compounding surface locals
  SmSurface *pSurface=NULL ;
  SM_TYPE    lSurfaceType ;  

  // compounding surface
  if (eType == SM_ASCII) { rDB.GoToNextLine() ; }
  SER(rDB.ReadType(lSurfaceType)) ; 
  SER(SmSurface::ReadFromDB(lSurfaceType, rDB, crContext, pSurface, lDBVersionNumber)) ;
  
  // when appropriate - breaks
  if(bNeedBreaks == FALSE)
    {
      ULONG ii, lBreakCnt ;

      // break number
      if (eType == SM_ASCII) { rFileIn >> lBreakCnt ; rDB.GoToNextLine(); }
      else /* Binary */      { SER(rDB.ReadLong(lBreakCnt)); }

      // manage break array
      pCrvOnSurf->m_vBreaks.SetSize(lBreakCnt) ; 
      double *dU = pCrvOnSurf->m_vBreaks.GetDataArray() ;

      // read every break
      if (eType == SM_ASCII) { for(ii=0;ii<lBreakCnt;ii++ ) { rFileIn >> dU[ii] ; }
        if(lBreakCnt > 0) { rDB.GoToNextLine(); }
      }
      else /* Binary */      { for(ii=0;ii<lBreakCnt;ii++ ) { SER(rDB.ReadDouble(dU[ii])); }
                             }
    } 

  // load obj
  pCrvOnSurf->m_pUVCurve    = pUVCurve ;   
  pCrvOnSurf->m_pSurface    = pSurface ;   
  pCrvOnSurf->m_vUVDomain.SetMinMax(sMin, sMax) ;  
  pCrvOnSurf->m_bNeedBreaks = bNeedBreaks ;
  pCrvOnSurf->m_lOwnerFlag  = 2 ; 
  
  // all done
  rpNewCurve = pCrvOnSurf ;
  return SM_SUCCESS;

} // end SmCrvOnSurf::ReadFromDB

/*******************************************************************//**
PURPOSE:
NOTES:
***********************************************************************/
SmBoolean SmCrvOnSurf::IsKindOf( SM_TYPE t ) const
{
  return ((SmCrvOnSurf_TYPE == t) ? TRUE : SmCurve::IsKindOf( (t) ));
}

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
void SmCrvOnSurf::Dump
  () 
 const
{
  TCHAR sBuff[SM_TBLOCK_SIZE];
  
  // SmCrvOnSurf data dump
  smos_WriteBuffer(_T("\nBegin SmCrvOnSurf::Dump()")) ;
  
  // m_pCurve and m_pSurface
  smos_sprintf( sBuff, (_T("\n  pUVCurve  = [0x%p], pSurface = [0x%p]  ")), m_pUVCurve, m_pSurface);
  smos_WriteBuffer(sBuff);    

  // m_vUVDomain
  smos_WriteBuffer(_T("\n  UVDomain  = ")); m_vUVDomain.Dump() ; // don't start next Dump String with '\n' because m_vUVDomain.Dump() ends on a '\n'
  
  // m_lOwnerFlag 
  smos_sprintf( sBuff, (_T("  OwnerFlag = [%ld], ")), m_lOwnerFlag);  
  smos_WriteBuffer(sBuff);
  smos_WriteBuffer(_T("\n    0 = deletes nothing when destructed                          ")); 
  smos_WriteBuffer(_T("\n    1 = copies curve and deletes copy when destructed            ")); 
  smos_WriteBuffer(_T("\n    2 = copies both inputs, deletes both copies when destructed  ")); 
  smos_WriteBuffer(_T("\n    3 = saves both inputs, deletes both originals when destructed")); 

  // output cache data
  SmCurve::Dump(FALSE) ;
 
  // dump the contained UVTrimCurve
  smos_WriteBuffer(_T("\n  Begin SmCrvOnSurf->UVTrimCurve Dump")) ;
  m_pUVCurve->Dump();
  smos_WriteBuffer(_T("\n  End SmCrvOnSurf->UVTrimCurve Dump")) ;

  // dump the contained Surface
  smos_WriteBuffer(_T("\n  Begin SmCrvOnSurf->Surface Dump")) ;
  m_pSurface->Dump();
  smos_WriteBuffer(_T("\n  End SmCrvOnSurf->Surface Dump")) ;

  smos_WriteBuffer(_T("\n End SmCrvOnSurf::Dump()\n")) ;

} // end SmCrvOnSurf::Dump

/*******************************************************************//**
PURPOSE: AssertValid() Test reporting static labels
***********************************************************************/
SmAssertReportLabel sAssertCrvOnSurf_list[] =
{
 /*  0 */ {SM_AT_NESTED_TEST, _T("Base Curve"),           _T("Base Curve must pass AssertValid") },
 /*  1 */ {SM_AT_NESTED_TEST, _T("Compounding Surface"),  _T("Compounding Surface must pass AssertValid") },
 /*  2 */ {SM_AT_GEOMETRIC,   _T("Contained Base Curve"), _T("Base Curve must map to domain of the surface") },
 /*  3 */ {SM_AT_POINTER,     _T("Bad UVCurve Owner"),    _T("When owned by CrvOnSurf, ContainedUVCurve->owner must be this SmCrvOnSurf object") },
 /*  4 */ {SM_AT_POINTER,     _T("Bad Surface Owner"),    _T("When owned by CrvOnSurf, ContainedSurface->owner must be this SmCrvOnSurf object") },
 /*  5 */ {SM_AT_POINTER,     _T("Bad UVCurve Context"),    _T("When owned by CrvOnSurf, ContainedUVCurve->context must be this SmCrvOnSurf->context") },
 /*  6 */ {SM_AT_POINTER,     _T("Bad Surface Context"),    _T("When owned by CrvOnSurf, ContainedSurface->context must be this SmCrvOnSurf->context") } 
} ;

/*******************************************************************//**
PURPOSE:

RETURNS ---  TRUE  = OK
             FALSE = Problem
***********************************************************************/
SmBoolean SmCrvOnSurf::AssertValid
 (SmAssertArray    * pAList,        // i/o: Accumulating list of failed Asserts, NULL to ignore, default:[NULL] 
  SmAssertTestLevel  eTestLevel,    // in : SM_LEVEL_0=fewest tests, SM_LEVEL_1, SM_LEVEL_2=all tests, 
                                    //      SM_LEVEL_GIVEN = run tests in order requested in pTestRequests 
                                    //      default:[SM_LEVEL_0] 
  SmAssertWalking    eWalkTree,     // in : SM_WALK = run AssertValid on any topology graph descendants, SM_NO_WALK=don't, default:[SM_WALK] 
  SmTArray<ULONG>  * pTestRequests) // in : when eTestLevel == SM_LEVEL_GIVEN, run these tests in this order, default:[NULL]
 const
{
  SM_REF1(pTestRequests);

  SmBoolean bRtn = TRUE ; 

  // test contained curve
  bRtn &= SM_ASSERT_BOOLEAN_REPORT(0, SM_LEVEL_0, m_pUVCurve->AssertValid(pAList, eTestLevel, eWalkTree), _T("")) ;

  // test contained surface
  bRtn &= SM_ASSERT_BOOLEAN_REPORT(1, SM_LEVEL_0, m_pSurface->AssertValid(pAList, eTestLevel, eWalkTree), _T("")) ;

  // Curve UV BBox
  SmExtent3d sCurveBBox ;
  m_pUVCurve->CalculateBoundingBox(m_pUVCurve->GetNaturalInterval(), &sCurveBBox) ;
  SmExtent2d sCurveUVBox(sCurveBBox.GetUMin(), sCurveBBox.GetVMin(),
                         sCurveBBox.GetUMax(), sCurveBBox.GetVMax()) ;
  
  // Surface UVDomain
  SmExtent2d sSurfaceUVBox = m_pSurface->GetNaturalUVDomain() ;

  // Test containment of curve within surface domain
  bRtn &= SM_ASSERT_BOOLEAN_REPORT(2, SM_LEVEL_0, sCurveUVBox.IsContainedBy(sSurfaceUVBox, 100*SM_EFF_ZERO), _T("")) ;
                 
  /*  3 */ // When owned by CrvOnSurf, ContainedUVCurve->owner must be this SmCrvOnSurf object 
  bRtn &= SM_ASSERT_BOOLEAN_REPORT(3, SM_LEVEL_0, m_lOwnerFlag == 0 || m_pUVCurve->GetOwner() == this, _T("")) ;

  /*  4 */ // When owned by CrvOnSurf, ContainedSurface->owner must be this SmCrvOnSurf object
  bRtn &= SM_ASSERT_BOOLEAN_REPORT(4, SM_LEVEL_0, m_lOwnerFlag == 0 || m_lOwnerFlag == 1 || m_pSurface->GetOwner() == this, _T("")) ;

  /*  5 */ // When owned by CrvOnSurf, ContainedUVCurve->context must be this SmCrvOnSurf->context
  bRtn &= SM_ASSERT_BOOLEAN_REPORT(5, SM_LEVEL_0, m_lOwnerFlag == 0 || m_pUVCurve->GetContext() == GetContext(), _T("")) ;

  /*  6 */ // When owned by CrvOnSurf, ContainedSurface->context must be this SmCrvOnSurf->context
  bRtn &= SM_ASSERT_BOOLEAN_REPORT(6, SM_LEVEL_0, m_lOwnerFlag == 0 || m_lOwnerFlag == 1 || m_pSurface->GetContext() == GetContext(), _T("")) ;

  // all done
  return(bRtn) ;

} // end SmCrvOnSurf::AssertValid

// obsolete
// /*******************************************************************//**
// PURPOSE: Fix Assert Report failures reported by an AssertValid call
// 
// RETURNS ---  TRUE  = OK
//              FALSE = Problem
// ***********************************************************************/
// SmBoolean SmCrvOnSurf::AssertHeal
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
//                rAReport.m_pHealMessage = _T("SmCrvOnSurf::AssertHeal fix not yet supported") ;  
//                 
//     }
// 
//   // all done
//   return(bRtn) ;
// 
//  } // end SmCrvOnSurf::AssertHeal  '
// end obsolete
