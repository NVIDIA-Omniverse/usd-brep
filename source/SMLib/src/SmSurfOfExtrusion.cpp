// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************//**
* FILE NAME --- SmSurfOfExtrusion.cpp
* PURPOSE: Implementation of SmSurfOfExtrusion methods.
**********************************************************************/

#include "StdAfx.h"

#include <SmSurfOfExtrusion.h>
#include <SmPlane.h>
#include <SmCone.h>
#include <SmLine.h>
#include <SmCompositeCurve.h>
#include <nurbs.h>
#include <SmNurbsSrf.h>
#include <SmGeomUtility.h>
#include <SmCurveClass.h>
#include <SmDatabaseIO.h>
#include <SmAssertArray.h>

#ifdef SM_DEBUG_CODE
  #include <SmBrep.h>
  #include <SmFace.h>
  #include <SmGap.h>
#endif // SM_DEBUG_CODE

/*******************************************************************//**
PURPOSE: Constructor for Finite Surface of Extrustion

NOTES: 1. Leaves m_pNurb = NULL ;
                2. Owns pGenCurve (deletes pGenCurve when this object is deleted)
                3. pGenCurve AnalInterval must equal crAnalUVDomain.GetUInteval
                4. pGenCurve.GetInsideOut() must equal FALSE
***********************************************************************/
SmSurfOfExtrusion::SmSurfOfExtrusion
  (SmBSplineCurve   * pGenCurve,          // in : curve being extruded
   const SmPoint3d  & crOrigin,           // in : Some point on GenCurve
   const SmVector3d & crExtrusionVec,     // in : extrusion vector (can be unit or non-unit)
   const SmExtent2d & crAnalUVDomain,     // in : U Range = GenCurve domain,
                                          //      V Range = extrusion limits.
   SmBoolean bSwapUV)                     // in : TRUE  = GenCurve is a constant U iso-Curve in the m_pNurb Surface
                                          //              and m_pNurb U Range = GenCurve domain,
                                          //                  m_pNurb V range = extrusion limits.
                                          //      FALSE = GenCurve is a constant V iso-Curve in the m_pNurb Surface
                                          //              and m_pNurb U Range = extrusion limits,
                                          //                  m_pNurb V range = GenCurve domain.
 : m_pGenCurve    (pGenCurve),
   m_vOrigin      (crOrigin),
   m_vExtrusionVec(crExtrusionVec),
   m_vAnalUVDomain(crAnalUVDomain),
   m_bSwapUV      (bSwapUV)
{
  SM_ASSERT(pGenCurve != NULL);

  // set the GenCurve owner
  pGenCurve->SetOwner(this) ;

  // Check input GenCurve->InsideOut bit must be FALSE
  SE_MSG((pGenCurve->GetInsideOut() == FALSE)? SM_SUCCESS : SM_ERR,
         _T("Bad Extrusion Input: GenCurve InsideOut bit must be FALSE")) ;

  // check input GenCurve->STEPInteval must match crAnalUVDomain interval
  SE_MSG(pGenCurve->GetSTEPInterval().AreEqual(crAnalUVDomain.GetUInterval(), SM_EFF_ZERO) ? SM_SUCCESS : SM_ERR,
         _T("Bad Extrusion Input: GenCurve STEP interval must match crAnalUVDomain UInterval")) ;

  // select tol based on origin and mid Point evaluation locations
  SmPoint3d sPnt;
  SE(pGenCurve->EvaluatePoint(pGenCurve->GetNaturalInterval().Evaluate(0.45678),sPnt));
  double dTol = ANALYTIC_TOL_SCALE * SM_EFF_ZERO * (1.0 + crOrigin.GetMaxDimension() + sPnt.GetMaxDimension());

  // if GenCurve is linear
  if(pGenCurve->IsLinear(dTol, &m_vCurvePoint, &m_vCurveVec) )
    {
      m_eOrientation = SM_CO_LINEAR ;
    }

  // if GenCurve is non planarity
  else if (!pGenCurve->IsPlanar(dTol, &m_vCurvePoint, &m_vCurveVec))
    {
      m_eOrientation = SM_CO_NOT_PLANAR ;
    }

  // else curve is planar - get planes orientation to ExtrusionVec
  else
    {
      // get angle between curve planeNormal and ExtrusionVec
      double dAngleRad ;
      m_vCurveVec.AngleBetween(crExtrusionVec, dAngleRad) ;

      // Avoid scaling problems due to angles
      // When close to perpendicular check by testing curve against plane perp to extrusion vector
      if(   SM_IS_ZERO_TO_TOL(dAngleRad,         SM_EFF_ZERO_RAD)
         || SM_IS_ZERO_TO_TOL(dAngleRad - SM_PI, SM_EFF_ZERO_RAD))
        {
          // normalize the extrusion vec
          SmVector3d sExtrusionNormal = crExtrusionVec;
          SE(sExtrusionNormal.Unitize());

          // see if the curve lies on the plane perpendicular to the extrusion vec to tolerance
          double dMaxDistToSurf;
          SmBoolean bOnPlane;
          SE(pGenCurve->IsOnPlane(sPnt,
                                  sExtrusionNormal,
                                  dTol,
                                  bOnPlane,
                                  dMaxDistToSurf));
          m_eOrientation = bOnPlane
                           ? SM_CO_PERPENDICULAR
                           : SM_CO_INDEPENDENT ;

        } // end curve is close to perpendicular check
      else if(SM_IS_ZERO_TO_TOL(dAngleRad - SM_PI/2.0, SM_EFF_ZERO_RAD))
        {
          // ExtrusionVec is in plane of ExtrusionVec
          m_eOrientation = SM_CO_COPLANAR ;
        }
      else // angle is independent but not perpendicular to ExtrusionVec
        {
          m_eOrientation = SM_CO_INDEPENDENT ;
        }

    } // end curve is planar - but in what direction? branch

} // end SmSurfOfExtrusion::SmSurfOfExtrusion default constructor

/*******************************************************************//**
PURPOSE: Constructor for Infinite Surface of Extrusion

NOTES: The surface of extrusion will own the generator curve.

NOTES: 1. Leaves m_pNurb = NULL ;
                2. Owns for deleting pGenCurve
                3. Use AdjustSTEPUVDomain() to change this from an infinite
                    to a finite surface.
***********************************************************************/
SmSurfOfExtrusion::SmSurfOfExtrusion
  (SmBSplineCurve   * pGenCurve,        // in : curve being extruded
   const SmVector3d & crExtrusionVec,   // in : extrusion vector (typically non-unit)
   SmBoolean          bSwapUV)          // in : TRUE  = GenCurve is a constant U iso-Curve in the m_pNurb Surface
                                        //              and m_pNurb U Range = GenCurve domain,
                                        //                  m_pNurb V range = extrusion limits.
                                        //      FALSE = GenCurve is a constant V iso-Curve in the m_pNurb Surface
                                        //              and m_pNurb U Range = extrusion limits,
                                        //                  m_pNurb V range = GenCurve domain.
 : m_pGenCurve(pGenCurve),
   m_vExtrusionVec(crExtrusionVec),
   m_bSwapUV(bSwapUV)
{
  SM_ASSERT(pGenCurve != NULL);

  // set the GenCurve owner
  pGenCurve->SetOwner(this) ;

  // Check input GenCurve->InsideOut bit must be FALSE
  SE_MSG(pGenCurve->GetInsideOut() ? SM_ERR : SM_SUCCESS,
         _T("Bad Extrusion Input: GenCurve InsideOut bit must be FALSE")) ;

  // set m_vOrigin
  SmExtent1d sCrvIvl = pGenCurve->GetSTEPInterval();
  SE(pGenCurve->EvaluateSTEP(sCrvIvl.Evaluate(0.45678),0,TRUE,&m_vOrigin));

  // set m_vAnalUVDomain for an infinitely long extrusion
  m_vAnalUVDomain = SmExtent2d(SmPoint2d(sCrvIvl.GetMin(),-SM_INFINITE_PARAMETER),
                               SmPoint2d(sCrvIvl.GetMax(), SM_INFINITE_PARAMETER));

  // select tol based on origin and mid Point evaluation locations
  SmPoint3d sPnt;
  SE(pGenCurve->EvaluatePoint(pGenCurve->GetNaturalInterval().Evaluate(0.0), sPnt));
  double dTol = ANALYTIC_TOL_SCALE * SM_EFF_ZERO * (1.0 + m_vOrigin.GetMaxDimension() + sPnt.GetMaxDimension());

  // if GenCurve is linear
  if(pGenCurve->IsLinear(dTol, &m_vCurvePoint, &m_vCurveVec) )
    {
      m_eOrientation = SM_CO_LINEAR ;
    }

  // if GenCurve is non planarity
  else if (!pGenCurve->IsPlanar(dTol, &m_vCurvePoint, &m_vCurveVec))
    {
      m_eOrientation = SM_CO_NOT_PLANAR ;
    }

  // else curve is planar - get planes orientation to ExtrusionVec
  else
    {
      // get angle between curve planeNormal and ExtrusionVec
      double dAngleRad ;
      m_vCurveVec.AngleBetween(crExtrusionVec, dAngleRad) ;

      // Avoid scaling problems due to angles
      // When close to perpendicular check by testing curve against plane perp to extrusion vector
      if(   SM_IS_ZERO_TO_TOL(dAngleRad,         SM_EFF_ZERO_RAD)
         || SM_IS_ZERO_TO_TOL(dAngleRad - SM_PI, SM_EFF_ZERO_RAD))
        {
          // normalize the extrusion vec
          SmVector3d sExtrusionNormal = crExtrusionVec;
          SE(sExtrusionNormal.Unitize());

          // see if the curve lies on the plane perpendicular to the extrusion vec to tolerance
          double dMaxDistToSurf;
          SmBoolean bOnPlane;
          SE(pGenCurve->IsOnPlane(m_vOrigin,
                                  sExtrusionNormal,
                                  dTol,
                                  bOnPlane,
                                  dMaxDistToSurf));
          m_eOrientation = bOnPlane
                           ? SM_CO_PERPENDICULAR
                           : SM_CO_INDEPENDENT ;

        } // end curve is close to perpendicular check
      else if(SM_IS_ZERO_TO_TOL(dAngleRad - SM_PI/2.0, SM_EFF_ZERO_RAD))
        {
          // ExtrusionVec is in plane of ExtrusionVec
          m_eOrientation = SM_CO_COPLANAR ;
        }
      else // angle is independent but not perpendicular to ExtrusionVec
        {
          m_eOrientation = SM_CO_INDEPENDENT ;
        }

    } // end curve is planar - but in what direction? branch

} // end SmSurfOfExtrusion::SmSurfOfExtrusion constructor

/*******************************************************************//**
PURPOSE: Copy constructor for SmSurfOfExtrusion

NOTES: Signals errors when input m_pGenCurve is NULL
                or fails to copy as a SmBSplineCurve object.
***********************************************************************/
SmSurfOfExtrusion::SmSurfOfExtrusion
  (const SmSurfOfExtrusion & crSource)
 : SmBSplineSurface(crSource),
   m_pGenCurve     (NULL),
   m_vOrigin       (crSource.m_vOrigin),
   m_vExtrusionVec (crSource.m_vExtrusionVec),
   m_vAnalUVDomain (crSource.m_vAnalUVDomain),
   m_eOrientation  (crSource.m_eOrientation),
   m_bSwapUV       (crSource.m_bSwapUV),
   m_vCurvePoint   (crSource.m_vCurvePoint),
   m_vCurveVec     (crSource.m_vCurveVec)
{
  const SmContext *pContext = GetContext();
  if (crSource.m_pGenCurve)
    {
      SmCurve *pGenCurve;
      crSource.m_pGenCurve->Copy(*pContext, pGenCurve);
      m_pGenCurve = SM_CAST_PTR(SmBSplineCurve, pGenCurve);
      if (!m_pGenCurve)
        { SE(SM_ERR); }
    }
  else
    { SE(SM_ERR); }

  // set the GenCurve owner
  m_pGenCurve->SetOwner(this) ;

} // end SmSurfOfExtrusion::SmSurfOfExtrusion copy constructor

/*******************************************************************//**
PURPOSE: Destructor to clean up Surface of Extrustion

NOTES:
***********************************************************************/
SmSurfOfExtrusion::~SmSurfOfExtrusion
  ()
{
  if(m_pGenCurve) { delete m_pGenCurve ; m_pGenCurve = NULL ; }

} // end SmSurfOfExtrusion::~SmSurfOfExtrusion destructor

/*******************************************************************//**
PURPOSE: Equality operator for SmSurfOfExtrusion

NOTES: Call base equivalence to check type and then check
       members for equivalence
***********************************************************************/
SmBoolean SmSurfOfExtrusion::operator==
  (const SmSurface& crOther)
 const
{
  // low work
  if(this == &crOther) { return TRUE ; }

  // first check the base
  SmBoolean bRtn = SmBSplineSurface::operator ==(crOther) ;

  if(bRtn)
    {
      // OK to cast
      SmSurfOfExtrusion &rOther = (SmSurfOfExtrusion &)crOther ;

      // check equivalence of these objects
      bRtn =  (   (   ( m_pGenCurve == rOther.m_pGenCurve)
                   || ( m_pGenCurve == NULL && rOther.m_pGenCurve == NULL)
                   || (   m_pGenCurve != NULL && rOther.m_pGenCurve != NULL
                       && *m_pGenCurve == *rOther.m_pGenCurve))
               && m_vOrigin       == rOther.m_vOrigin
               && m_vExtrusionVec == rOther.m_vExtrusionVec
               && m_vAnalUVDomain == rOther.m_vAnalUVDomain
               && m_eOrientation  == rOther.m_eOrientation
               && m_bSwapUV       == rOther.m_bSwapUV
               && m_vCurvePoint   == rOther.m_vCurvePoint
               && m_vCurveVec     == rOther.m_vCurveVec   ) ;
    }

  // all done
  return bRtn ;

} // end SmSurfOfExtrusion::operator==

/*******************************************************************//**
PURPOSE: Set Analytic domain to the given STEP-domain and rebuild
            the underlying m_pNurb surface.

NOTES:
  1. Modifies the associated GenCurve Domain so that
     GenCurve->Interval == STEPUVDomain->GetUInterval() ;

  2. This function IS a trim function changing the shape of the 3d
     surface by trimming the existing surface to the given domain.

     This is NOT a scale domain function where the 3d shape of the
     surface stays constant as the underlying UVDomain gets scaled.
***********************************************************************/
SmStatus SmSurfOfExtrusion::AdjustSTEPUVDomain
  (const SmExtent2d & crNewSTEPUVDomain)
{
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  if(bDebugMe)
    {
      SM_ASSERT_VALID(this) ;

      SmFace *pFace = (SmFace *)this->GetFace() ;
      SmBrep *pBrep = pFace ? pFace->GetBrep() : NULL ;

      smgfx_Erase() ;
      smgfx_SetLook( 1, 2, 0, 0, 1 ); if(pBrep) { pBrep->Draw( TRUE ); sm_GraphicsLoop(); }
      smgfx_SetLook(1,2, 0,1,1) ; this->DrawUV(8,8,FALSE,NULL,TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook( 1, 2, 0, 0, 0 ); if(pFace) { pFace->Draw( SM_DM_CROSSHATCH ); sm_GraphicsLoop(); }
      smgfx_SetLook( 3, 4, 1, 0, 0 ); if(m_pGenCurve) { m_pGenCurve->Draw(); sm_GraphicsLoop(); }
      smgfx_SetLook(5,6, 0,1,0) ; m_vOrigin.Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 1,0,0) ; (m_vAnalUVDomain.GetVInterval().GetLength() * m_vExtrusionVec).Draw(&m_vOrigin) ; sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

  // no work - new UVDomain == current UVDomain
  if(   crNewSTEPUVDomain.IsContainedBy(m_vAnalUVDomain, SM_EFF_ZERO)
     && m_vAnalUVDomain.IsContainedBy(crNewSTEPUVDomain, SM_EFF_ZERO))
    { return SM_SUCCESS; }

  // set the new domain
  m_vAnalUVDomain = crNewSTEPUVDomain;

  // set GenCurve analytic domain from U part of new surface domain
  //  This trims the GenCurve in the same manner as this call
  //  is trimming the UV analytic domain.
  SER( m_pGenCurve->AdjustSTEPInterval( m_vAnalUVDomain.GetUInterval() ));

  // SmSurfOfExtrusion parameterization rules
  // Rule 1. GenCurve->AnalInterval  == AnalUVDomain.GetUInterval
  // Rule 2. GenCurve->NurbInterval  == m_bSwapUV ? sNaturalUVDomain.GetVInterval : sNaturalUVDomain.GetUInterval
  // Rule 3. GenCurve NURBKnotVector == m_bSwapUV ? Surface->NURBKnotV : Surface->NURBKnotU

  // build Nurb domain from Analytic domain
  SmExtent2d sNurbDomain = crNewSTEPUVDomain ; 
  if(m_bSwapUV)
    { sNurbDomain.Transpose() ; }

  // set Surface and GenCurve nurb domain and enforce rule 2 assuming input GenCurve/Surf and PolarCurve/Surf knot vector compatibility
  // note: We reparameterize only to initialize the intervals and domain values so that MakeNurb() will build
  //       new Nurb structures trimmed and scaled to the input UVDomains
  Reparameterize(sNurbDomain) ; 

  // rebuild the associated m_pNurb
  SER(MakeNurb());

#ifdef SM_DEBUG_CODE
  // draw
  if(bDebugMe)
    {
      SM_DUMP_AND_ASSERT_VALID(this) ;

      SmFace *pFace = (SmFace *)this->GetFace() ;
      SmBrep *pBrep = pFace ? pFace->GetBrep() : NULL ;

      smgfx_Erase() ;
      smgfx_SetLook( 1, 2, 0, 0, 1 ); if(pBrep) { pBrep->Draw( TRUE ); sm_GraphicsLoop(); }
      smgfx_SetLook(1,2, 0,1,1) ; this->DrawUV(8,8,FALSE,NULL,TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook( 1, 2, 0, 0, 0 ); if(pFace) { pFace->Draw( SM_DM_CROSSHATCH ); sm_GraphicsLoop(); }
      smgfx_SetLook( 3, 4, 1, 0, 0 ); if(m_pGenCurve) { m_pGenCurve->Draw(); sm_GraphicsLoop(); }
      smgfx_SetLook(5,6, 0,1,0) ; m_vOrigin.Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 1,0,0) ; (m_vAnalUVDomain.GetVInterval().GetLength() * m_vExtrusionVec).Draw(&m_vOrigin) ; sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

  // all done
  return SM_SUCCESS;

} // end SmSurfOfExtrusion::AdjustSTEPUVDomain

/*******************************************************************//**
PURPOSE: Trim Analytic domain and m_pGenCurve to the given
   Nurbs domain (after converting from Nurbs to STEP) without
   rebuilding underlying m_pNurb surface.

NOTES:
  1. Sets m_vAnalUVDomain = ConvertUVFromNURBSToSTEP(crNewNurbsDomain).

  2. Modifies the associated GenCurve Domain so that
     GenCurve->STEPInterval == NewAnalUVDomain.GetUInterval
     GenCurve->NurbInterval == m_bSwapUV
                               ? crNewNurbsDomain.GetVInterval()
                               : crNewNurbsDomain.GetUInterval() ;

  3. This function IS a trim function changing the shape of the 3d
     surface by trimming the existing surface to the given domain.

     This is NOT a scale domain function where the 3d shape of the
     surface stays constant as the underlying UVDomain gets scaled.

  4. Does not rebuild the underlying m_pNurb surface.
     Assumes the Nurbs representation has already been dealt with.
***********************************************************************/
SmStatus SmSurfOfExtrusion::UpdateAnalyticalDomain
  ( const SmExtent2d & crNewNurbsDomain )   // in : Desired new Nurb UVDomain
{
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  // draw
  if(bDebugMe)
    {
      SM_DUMP_AND_ASSERT_VALID(this) ;

      SmFace *pFace = (SmFace *)this->GetFace() ;
      SmBrep *pBrep =   pFace ? pFace->GetBrep() : NULL ;

      smgfx_Erase() ;
      smgfx_SetLook( 1, 2, 0, 0, 1 ); if(pBrep) { pBrep->Draw( TRUE ); sm_GraphicsLoop(); }
      smgfx_SetLook(1,2, 0,1,1) ; this->DrawUV(8,8,FALSE,NULL,TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook( 1, 2, 0, 0, 0 ); if(pFace) { pFace->Draw( SM_DM_CROSSHATCH ); sm_GraphicsLoop(); }
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

  // convert given NurbUVDomain corners into STEPUVPoints
  SmPoint2d sMinAnalUV, sMaxAnalUV;
  ConvertUVFromNURBSToSTEP( crNewNurbsDomain.GetMin(), sMinAnalUV );
  ConvertUVFromNURBSToSTEP( crNewNurbsDomain.GetMax(), sMaxAnalUV );

  // Build desired new STEP UVDomain
  // Don't SetMinMax because Min might not be smaller (if swapped).
  // m_vAnalUVDomain.SetMinMax( sMinAnalUV, sMaxAnalUV );
  m_vAnalUVDomain.Init();
  m_vAnalUVDomain.AddPoint2d( sMinAnalUV );
  m_vAnalUVDomain.AddPoint2d( sMaxAnalUV );

  // set GenCurve analytic domain from U part of new surface Anal domain.
  //  Rebuilds GenCurve->m_pNurb without ensuring that interval
  //    matches the surface->NurbDomain proper interval.
  SER( m_pGenCurve->AdjustSTEPInterval( m_vAnalUVDomain.GetUInterval() ));

  // make sure m_pGenCurve Nurb domain matches crNewNurbsDomain
  SmExtent1d sGenIvl =   m_bSwapUV
                       ? crNewNurbsDomain.GetVInterval()
                       : crNewNurbsDomain.GetUInterval() ;
  m_pGenCurve->ScaleKnotVector(sGenIvl.GetMin(), sGenIvl.GetMax()) ;

  // recompute a m_vOrigin point to ensure that the origin is still on the genCurve
  m_pGenCurve->EvaluatePoint(m_pGenCurve->GetNaturalInterval().GetMin(), m_vOrigin) ;

  // all done
  return SM_SUCCESS;

} // end SmSurfOfExtrusion::UpdateAnalyticalDomain

/*******************************************************************//**
PURPOSE: Copy a Surface of Extrusion

NOTES:
***********************************************************************/
SmStatus SmSurfOfExtrusion::Copy
  (const SmContext & crContext,
   SmSurface *& rpNewSurface)
  const
{
    SmSurfOfExtrusion *pCopy = new(crContext) SmSurfOfExtrusion(*this);
    NER(pCopy);
    rpNewSurface = pCopy;
    return SM_SUCCESS;

} // end SmSurfOfExtrusion::Copy

/*******************************************************************//**
PURPOSE: Reparameterize the NURB domain of a B-Spline Surface

NOTES: This method is only available for users with NLib
***********************************************************************/
SmStatus SmSurfOfExtrusion::Reparameterize
  (const SmExtent2d & crTrimDomain        // in : new parameter range for BSPlineSurface
      // SmBoolean    bUpdateSTEPParams   // in : TRUE = Rebuild STEP params to match Bspline parameters
  )                                       //      FALSE= skip rebuild step params - needed for analytic constructors
                                          //      Not used.
{
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  if(bDebugMe)
    {
      SM_ASSERT_VALID(this) ;
    }
#endif // SM_DEBUG_CODE

  if (m_vAnalUVDomain.IsContainedBy(crTrimDomain) && crTrimDomain.IsContainedBy(m_vAnalUVDomain))
    { return SM_SUCCESS; }
  
  Notify(SM_NO_PRE_EDIT, this, SM_NO_GET_OWNER(this), NULL);

  // new NURB locals
  NL_RECTANGLE R;

  R.ul = crTrimDomain.GetMin().x;
  R.ur = crTrimDomain.GetMax().x;
  R.vb = crTrimDomain.GetMin().y;
  R.vt = crTrimDomain.GetMax().y;

  NL_SURFACE * surP = GetOrCreateGwNurbPointer();
  NER(surP);

  // set NURB Surface domain parameterization - no change in shape
  N_SrfReparamToInterval(surP,R, NL_UVDIR);

  // SmSurfOfExtrusion parameterization rules
  // Rule 1. GenCurve->AnalInterval  == AnalUVDomain.GetUInterval  (No Changes)
  // Rule 2. GenCurve->NurbInterval  == m_bSwapUV ? sNaturalUVDomain.GetVInterval : sNaturalUVDomain.GetUInterval (Update)
  // Rule 3. GenCurve NURBKnotVector == m_bSwapUV ? Surface->NURBKnotV : Surface->NURBKnotU  (No Changes)

  // Rule 2
  m_pGenCurve->EditParameterization(m_bSwapUV ? crTrimDomain.GetVInterval() : crTrimDomain.GetUInterval()) ;

  // done with modifications
  Notify(SM_NO_POST_EDIT, this, SM_NO_GET_OWNER(this), NULL);

  // all done
#ifdef SM_DEBUG_CODE
  if(bDebugMe)
    {
      SM_ASSERT_VALID(this) ;
    }
#endif // SM_DEBUG_CODE

  return SM_SUCCESS;

} // end SmSurfOfExtrusion::Reparameterize

/*******************************************************************//**
PURPOSE: Given a point on the Surface-Of-Extrusion, find its
    corresponding analytic UV-parameter.

NOTES: This method requires a point exactly on the surface.
    Therefore, users should be cautious when calling this method.

   If the result is on the seam of a closed surface, and a guess was
   given, the output will be set to whichever side of the seam is closer
   to the guess.  If no guess was given, it will be set to the low end
   of the closed domain.

RETURNS --- SM_ERR for points outside the domain
            SM_SUCCESS for points inside the domain

METHOD ---

***********************************************************************/
SmStatus SmSurfOfExtrusion::STEPInversion
  (const SmExtent2d & crAnalDomain,       // in : domain limit for valid transformations
                                          //      Passed on to SmBSplineSurface::STEPINVERSION for nonPlanar GenCurves
   const SmPoint3d  & crPointOnSurf,      // in : Target Point - must be on surface within Tolerance
   double             d3DTolerance,       // in : Max allowed distance between Point and Surface
   SmPoint2d        & rdAnalUVParameter,  // out: STEP UV params for target point
                                          //      [U = Degrees about rotation Axis,
                                          //       V = STEP Param of rotatedPoint on genCurve]
   SmLocationType   & reLocation,         // out: oneof SM_LT_POLE,
                                          //            SM_LT_U_SEAM,
                                          //            SM_LT_V_SEAM,
                                          //            SM_LT_UV_SEAM,
                                          //            SM_LT_INTERIOR,
                                          //            SM_LT_EXTERIOR
   SmPoint2d        * pUVGuess)           // in, opt: guess parameter.
  const
{
  // gwc:Question - why not just intersect a sweep line with the genCurve?
  // Something like the following with some extra stuff put in to get
  // the reLocation output properly labeled.
  //
  //      // locals
  //      SmContext *pContext = GetContext() ;
  //      SmSolution sSData[16];
  //      SmSolutionArray sSolutions(16,sSData);
  //
  //      // build line through PointOnSurf Back to GenCurve
  //      SmVector3d sLineVec         = -m_vExtrusionVec * (1.0 + m_vAnalUVDomain.GetMax().y) ;
  //      double     sLineLength      = sLineVec.Length() ;
  //                 sLineVec        /= sLineLength ;
  //      SmLine     sLine(crPointOnSurf, sLineVec, SmExtent1d(0.0, sLineLength), 1.0, pContext) ;
  //
  //      // Intersect this line with GenCurve
  //      SmExtent1d sIvl = m_pGenCurve->GetNaturalInterval();
  //      SER(m_pGenCurve->GlobalCurveIntersect(m_pGenCurve->GetNaturalInterval(),
  //                                            sLine,
  //                                            sLine->GetNaturalInterval(),
  //                                            d3DTolerance,
  //                                            sSolutions) ;
  //      // something for reLocation . . .
  //

  // Init outputs
  reLocation = SM_LT_EXTERIOR;

  SmStatus eStat = SM_ERR;

  // When the extrusion was not perpendicular to the genCurve plane
  if (m_eOrientation != SM_CO_PERPENDICULAR)
    {
      // Not perpendicular, must use the general solver.

      // convert analytical domain to NURBS domain.
      SmPoint2d  sStepUVMin, sStepUVMax, sNurbsUVMin, sNurbsUVMax;
      sStepUVMin = crAnalDomain.GetMin();
      sStepUVMax = crAnalDomain.GetMax();
      ConvertUVFromSTEPToNURBS( sStepUVMin, sNurbsUVMin );
      ConvertUVFromSTEPToNURBS( sStepUVMax, sNurbsUVMax );
      SmExtent2d sNurbsDomain(sNurbsUVMin, sNurbsUVMax) ;

      // map 3dPoint to NurbUV parameter
      SmPoint2d sNurbsParameter;

      eStat = SmBSplineSurface::STEPInversion( sNurbsDomain,
                                               crPointOnSurf,
                                               d3DTolerance,
                                               sNurbsParameter,
                                               reLocation,
                                               pUVGuess) ;
      if ( eStat != SM_SUCCESS )
        { return eStat; } // No need to SER.

      // convert NurbUV point to StepUV point
      SER(ConvertUVFromNURBSToSTEP( sNurbsParameter, rdAnalUVParameter ));
    }

  else // Extrusion is perpendicular to GenCurve's plane,
    {
      SM_ASSERT(m_eOrientation == SM_CO_PERPENDICULAR) ;

      // init output
      reLocation = SM_LT_INTERIOR;

      // build extrusion line off of m_vOrigin that spans extrusion surface
      SmVector3d sZAxis           = m_vExtrusionVec;
      double     dExtrusionLength = m_vAnalUVDomain.GetSize().y;
      SmVector3d sLineVec         = sZAxis * dExtrusionLength;
      SmPoint3d  sLineOrigin      = m_vOrigin + m_vAnalUVDomain.GetMin().y * m_vExtrusionVec;

      // find UVAnal.X value by projecting PointOnSurf onto Sweep Line that span Swept Surface
      double dLineParam;
      SER(smgu_LineClosestPoint(sLineOrigin,sLineVec,crPointOnSurf,dLineParam));
      // No: the passed-in domain might be bigger than ours (OutOfBounds might be enabled.)
      // if (dLineParam < 0.0) dLineParam = 0.0;
      // if (dLineParam > 1.0) dLineParam = 1.0;
      // rdAnalUVParameter = m_vAnalUVDomain.Evaluate(dLineParam,dLineParam);  // Set Y parameter
      SmExtent1d sAnalVDomain = m_vAnalUVDomain.GetVInterval();
      rdAnalUVParameter.y = sAnalVDomain.GetMin() + dLineParam * sAnalVDomain.GetLength();

      // Project PointOnSurf along sweep direction back to plane containing GenCurve
      double dPointInPlaneParam;
      SER(smgu_LineClosestPoint(crPointOnSurf,sLineVec,m_vOrigin,dPointInPlaneParam));
      SmPoint3d sPointInPlane = crPointOnSurf + dPointInPlaneParam * sLineVec;

      // Find where the projected point intersects the GenCurve
      SmSolution sSData[16];
      SmSolutionArray sSolutions(16,sSData);
      SmExtent1d sIvl = m_pGenCurve->GetSTEPInterval();

      if ( IsOutOfBoundsEnabled() )
        {
          sIvl = m_pGenCurve->GetMaxAnalyticDomain();
          m_pGenCurve->SetOutOfBoundsEnabled( TRUE );
        }

      eStat = (m_pGenCurve->GlobalPointSolveSTEP( sIvl,
                                                  SM_SO_INTERSECT,
                                                  sPointInPlane,
                                                  d3DTolerance,
                                                  NULL, NULL,
                                                  SM_SR_ALL,
                                                  sSolutions));
      if ( eStat != SM_SUCCESS )
        { return eStat; } // No need to SER.

      // failure - projected point is not on GenCurve
      if (sSolutions.GetSize() == 0)
        { return SM_ERR; }

      // set outputs
      SmSolution &rSol = sSolutions[0];
      rdAnalUVParameter.x = rSol.m_vStart[0];

      // when point falls on a seam - remember that in the output
      if (sSolutions.GetSize() > 1)
        {
          if(   m_pGenCurve->IsClosed(sIvl,d3DTolerance)
             && sIvl.IsValueOnBoundary(rdAnalUVParameter.x) )
            {
              if (reLocation == SM_LT_INTERIOR)
                {  reLocation = SM_LT_U_SEAM ; }

              // On a seam.  If a guess was given,
              // set u-parameter to what's closer to the guess.
              if ( pUVGuess != NULL )
                {
                  SmExtent1d sAnalUDomain = m_vAnalUVDomain.GetUInterval();
                  if ( pUVGuess->x > sAnalUDomain.GetMid() )
                    { rdAnalUVParameter.x = sAnalUDomain.GetMax(); }
                }
            }
        }
    } // end m_eOrientation == SM_CO_PERPENDICULAR check

  // all done
  return SM_SUCCESS;


} // end SmSurfOfExtrusion::STEPInversion

/*******************************************************************//**
PURPOSE: Convert from STEP to NURBS parameterization.

NOTES:

 NOTE: this routine assumes that the STEP and NURBs parameterizations
       are in sync with each other.  Do not call this when that is
       not the case, as during trimming.

 Parameterization:
  STEP: u-parameter is along the generator curve,
        equal to the generator curve's parameter.
        Note, if the generator curve is analytic, it might have
        its own STEP/Nurbs mapping.
        v-parameter is along the extrusion.
  NURB: u and v may be swapped, according to m_bSwapUV.
        u (if not swapped) or v (swapped): same as STEP u.
        other: may be shifted and/or scaled linearly,
        according to the respective ranges.
***********************************************************************/
SmStatus SmSurfOfExtrusion::ConvertUVFromSTEPToNURBS
  (const SmPoint2d & crSTEPUV,  // in : STEP Domain UV point
         SmPoint2d & rNURBSUV)  // out: Nurb Domain UV point
  const
{
  // locals
  SmExtent2d sNaturalUVDomain = GetNaturalUVDomain();
  SmExtent2d sStepUVDomain    = GetSTEPUVDomain();

  // (Don't check this: any parameter can be converted, it's up to
  // the caller to use it properly.  [bd July 07] )
  // if (!sStepUVDomain.ContainsPoint2d(crSTEPUV))
  //   {
  //     SER(SM_ERR);
  //   }

  // Map the STEP u-param to Nurbs u:
  SER( m_pGenCurve->ConvertTFromSTEPToNURBS( crSTEPUV.x, rNURBSUV.x ));

  // Map the STEP v-param to Nurbs v: a linear mapping
  double dFraction =  (crSTEPUV.y - sStepUVDomain.GetMin().y) / sStepUVDomain.YLength();
  SmExtent1d sNurbsExtrusionDomain =   ( m_bSwapUV )
                                     ? sNaturalUVDomain.GetUInterval()
                                     : sNaturalUVDomain.GetVInterval();
  rNURBSUV.y = sNurbsExtrusionDomain.Evaluate( dFraction );

  // when SwapUV is true, remember to swap the NurbsUV values
  if ( m_bSwapUV )
    { SM_SWAP(double, rNURBSUV.x, rNURBSUV.y ); }

#ifdef SM_DEBUG_CODE
  // check conversions reciprocity
  SmPoint2d sCheckUV ;
  ConvertUVFromNURBSToSTEP(rNURBSUV, sCheckUV) ;
  double dUVDist = (crSTEPUV - sCheckUV).Length() ;
  double dScaledUVZero = SM_EFF_ZERO * 10.0 * (1.0 + crSTEPUV.GetMaxDimension()) ;

  SM_ASSERT(dUVDist < dScaledUVZero) ;

  // check quality of the conversion
  SmPoint3d sSTEPPoint, sNurbPoint ;
  Evaluate(rNURBSUV, 0, 0, TRUE, TRUE, TRUE, &sNurbPoint) ;
  EvaluateSTEP(crSTEPUV, 0, 0, TRUE, TRUE, TRUE, &sSTEPPoint) ;
  double dDist = (sNurbPoint -sSTEPPoint).Length() ;
  double dScaledZero = SM_EFF_ZERO * (1.0 + sNurbPoint.GetMaxDimension()) ;

  SM_ASSERT(dDist < dScaledZero) ;
#endif // SM_DEBUG_CODE

  // all done
  return SM_SUCCESS;

} // end SmSurfOfExtrusion::ConvertUVFromSTEPToNURBS

/*******************************************************************//**
PURPOSE: Convert from NURBS to STEP parameterization.

NOTES:

 NOTE: this routine assumes that the STEP and NURBs parameterizations
       are in sync with each other.  Do not call this when that is
       not the case, as during trimming.

  Parameterization: see ConvertUVFromSTEPToNURBS Usage notes.
***********************************************************************/
SmStatus SmSurfOfExtrusion::ConvertUVFromNURBSToSTEP
  (const SmPoint2d & crNURBSUV,    // in :
         SmPoint2d & rSTEPUV)      // out:
  const
{
  // (Don't check this: any parameter can be converted, it's up to
  // the caller to use it properly.  [bd July 07] )
  // if (!sNatural.ContainsPoint2d(crNURBSUV)) {
  //     SER(SM_ERR);
  // }

  // locals
  SmPoint2d sNurbsUV( crNURBSUV );

  // when SwapUV is true, remember to transpose the NurbsUV values
  if ( m_bSwapUV )
    { SM_SWAP(double, sNurbsUV.x, sNurbsUV.y) ; }

  // STEP u-param == gen curve param, possibly with its own mapping:
  SER( m_pGenCurve->ConvertTFromNURBSToSTEP( sNurbsUV.x, rSTEPUV.x ));

  // STEP v-param: linear mapping:
  SmExtent1d sStepUVDomain = GetSTEPUVDomain().GetVInterval();
  SmExtent1d sNurbUVDomain =   ( m_bSwapUV )
                             ? GetNaturalUVDomain().GetUInterval()
                             : GetNaturalUVDomain().GetVInterval();
  double dFraction = ( sNurbsUV.y - sNurbUVDomain.GetMin() ) / sNurbUVDomain.GetLength();
  rSTEPUV.y = sStepUVDomain.Evaluate( dFraction );

  // all done
  return SM_SUCCESS;

} // end SmSurfOfExtrusion::ConvertUVFromNURBSToSTEP

/*******************************************************************//**
PURPOSE: Create a surface-of-extrusion canonically that is
            finite along the length of the given genCurve but that
            is infinite in the direction of the extrusion vector

NOTES: Build m_pNurb
***********************************************************************/
SmStatus SmSurfOfExtrusion::CreateCanonical
  (const SmContext    & crContext,              // in : context for new object construction
   SmBSplineCurve     * pSweptCurve,            // in : GenCurve
   const SmVector3d   & crExtrusionVec,         // in :
   SmSurfOfExtrusion *& rpNewSurfOfExtrusion)   // out:
{
  //double dLeng = pSweptCurve->ApproximateLength(pSweptCurve->GetNaturalInterval(),5);
  //double dTol = dLeng * SM_EFF_ZERO_SQRT;

  // make an infinitly long extrusion of pSweptCurve
  rpNewSurfOfExtrusion = new (crContext) SmSurfOfExtrusion(pSweptCurve,
                                                           crExtrusionVec,
                                                           FALSE);
  NER(rpNewSurfOfExtrusion);

  // Make the associated m_pNurb Surface
  SER(rpNewSurfOfExtrusion->MakeNurb());

  // all done
  return SM_SUCCESS;

} // end SmSurfOfExtrusion::CreateCanonical

/*******************************************************************//**
PURPOSE: Evaluate the point & derivatives using the STEP parameterization.
            EvaluatePoint(NurbUV) = EvaluateSTEPPoint(StepUV).
            But tangents may vary in magnitude and
            u/v may be swapped.

NOTES:
  SurfOfExtrusion(u,v) = GenCurve(u) + v * ExtrusionVec
***********************************************************************/
SmStatus SmSurfOfExtrusion::EvaluateSTEP
  (const SmPoint2d & crUV,    // in : target surface point of Surf(u,v) = GenCurve(u) + v * ExtrusionVex ;
   ULONG lHighestUDeriv,      // in : Requested highest U derivative
   ULONG lHighestVDeriv,      // in : Requested highest V derivative
   SmBoolean ,                // in : bUFromLeft = if P is on U interval boundary
                              //      TRUE  = evaluate P in upper interval where P is on the left of the interval
                              //      FALSE = evaluate P in lower interval where P is on the right of the interval
   SmBoolean ,                // in : bVFromLeft = if P is on V interval boundary
                              //      TRUE  = evaluate P in upper interval where P is on the left of the interval
                              //      FALSE = evaluate P in lower interval where P is on the right of the interval
   SmBoolean ,                // in : bOnlyUpperHalf = TRUE=compute upper half of matrix only
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
  // check state:
  const ULONG lMaxDeriv = 5; // 5: MAXDER in NLib -- nurbsdef.h
  if ( lHighestUDeriv > lMaxDeriv )
    { SER(SM_ERR); }
  SM_ASSERT(lHighestUDeriv == lHighestVDeriv);  // we insist on these being equal.

  // Set zero derivative values - anything with a partial V term
  ULONG i, j, iOff ;
  for(i=0, iOff=0;i<=lHighestUDeriv;i++)
    {
      for(j=0;j<=lHighestVDeriv;j++,iOff++)
        {
          // only zero out the Partial V terms that are to be set -
          //   this leaves a bit of the output array untouched as advertised.
          if(   j > 0
             && i+j <= lHighestUDeriv
             && i+j <= lHighestVDeriv)
            { aDerivatives[iOff].Set( 0,0,0 ); }
        }
    }

  // per SMLib practice clamp input point to surface domain
  SmPoint2d sUV = crUV;
  if ( !IsOutOfBoundsEnabled() && !m_vAnalUVDomain.ContainsPoint2d( sUV ) )
    { sUV = m_vAnalUVDomain.ClampPoint2d( sUV ); }

  // If we are allowing out-of-bounds eval, then the gen curve needs to too.
  // Note, we can't use the macro here because it only turns it on,
  // and so would have to be in an if/else, an so it would go out of scope
  // before we could use it.
  SmSurfOfExtrusion * pNonConstThis = SM_CONST_CAST( SmSurfOfExtrusion*, this );
  SmTemporaryChangeValue<SmBoolean> sChangeOB( m_pGenCurve  ->GetOutOfBoundsEnabled(),
                                               pNonConstThis->GetOutOfBoundsEnabled() );

  // locals
  double dU = crUV.x ;
  double dV = crUV.y ;

  // get GenCurve position and requested derivative values
  SmVector3d  sPV[ lMaxDeriv+1 ] ;
  SER( m_pGenCurve->EvaluateSTEP( dU, lHighestUDeriv, TRUE, sPV ));

  // position
  aDerivatives[0] = sPV[0] + dV * m_vExtrusionVec ;

  // derivatives:
  //   Su = sPV[1]
  //   Sv = m_vExtrusionVec
  // Higher derivatives:
  // - All pure 'u' derivs are equal to the GenCurve's derivatives;
  // - All derivs with any 'v' in them (beyond Sv) are zero.

  // 1st derivatives
  if ( lHighestVDeriv >= 1 )
    { aDerivatives[1] = m_vExtrusionVec; }

  // all U derivatives
  for ( i = 1; i <= lHighestUDeriv; i++ )
    { aDerivatives[ i * (lHighestVDeriv+1) ] = sPV[i];}

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
  if (bDebugMe)
    {
      SmVector3d sVec = aDerivatives[lHighestVDeriv+1] * 180.0 / SM_PI;

      smgfx_SetColor(1,0,0); aDerivatives[0].Draw(); sm_GraphicsLoop();
      aDerivatives[lHighestVDeriv].Draw(&aDerivatives[0]); sm_GraphicsLoop();
      smgfx_SetColor(0,0.6,0); sVec.Draw(&aDerivatives[0]); sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

#ifdef SM_DEBUG_CODE
  if(bDebugMe)
    {
      Dump() ;
    }
#endif // SM_DEBUG_CODE

  // all done
  return SM_SUCCESS;

} // end SmSurfOfExtrusion::EvaluateSTEP

/*******************************************************************//**
PURPOSE: Evaluate the point using the STEP parameterization.

NOTES:
***********************************************************************/
SmStatus SmSurfOfExtrusion::EvaluateSTEPPoint
  (const SmPoint2d & crUV,  // in : target step domain point
   SmPoint3d & rPoint)      // out: 3D point
 const
{
  // per SMLib practice clamp input point to surface domain
  SmPoint2d sUV = crUV;
  if ( !IsOutOfBoundsEnabled() && !m_vAnalUVDomain.ContainsPoint2d( sUV ) )
    { sUV = m_vAnalUVDomain.ClampPoint2d( sUV ); }

  // get GenCurve position and requested derivative values
  SmVector3d  sPV ;
  SER( m_pGenCurve->EvaluateSTEP( crUV.x, 0, TRUE, &sPV ));

  // position
  rPoint = sPV + crUV.y * m_vExtrusionVec ;

  // all done
  return SM_SUCCESS;

} // end SmSurfOfExtrusion::EvaluateSTEPPoint

/*******************************************************************//**
PURPOSE: Get canonical data from a SmSurfOfExtrusion

NOTES:
***********************************************************************/
SmStatus SmSurfOfExtrusion::GetCanonical
  (SmBSplineCurve *& rpSweptCurve,    // out: GenCurve pointer
   SmVector3d      & rExtrusionVec)   // out: ExtrusionVector
  const
{
  rpSweptCurve  = m_pGenCurve;
  rExtrusionVec = m_vExtrusionVec;

  // all done
  return SM_SUCCESS;

} // end SmSurfOfExtrusion::GetCanonical

/*******************************************************************//**
PURPOSE: Get STEPdomain range of surface's associated extrusion
            and optionally, the STEP parameter associated with the extrusion
            which is always SM_SP_V.

NOTES:
***********************************************************************/
SmExtent1d SmSurfOfExtrusion::GetSTEPLinearParamExtent
  (SmSurfParamType * pOptExtrusionParam)  // out: step param direction associated with extrusion,
                                          //      always SM_SP_V.
 const
{
  // set optional output - V is always the STEP linear direction for swept surfaces
  if (pOptExtrusionParam) *pOptExtrusionParam = SM_SP_V;

  // all done
  return ( m_vAnalUVDomain.GetVInterval() ) ;

} // end SmSurfOfExtrusion::GetSTEPLinearParamExtent

/*******************************************************************//**
PURPOSE: Get NURB domain range of surface's extrusion and optionally,
           the NURB Surf parameter associated with the extrusion.

NOTES:  when m_bSwapUV == FALSE, pOptExtSurfParam = SM_SP_V
             m_bSwapUV == TRUE,  pOptExtSurfParam = SM_SP_U
***********************************************************************/
SmExtent1d SmSurfOfExtrusion::GetLinearParamExtent
  (SmSurfParamType * pOptExtSurfParam)    // out: optional NURB param ivl which spans the extrusion direction
                                          //      NULL to ignore, default:[NULL]
 const
{
  // get m_pNurb UVDomain
  SmExtent2d sNurbUVDomain = GetNaturalUVDomain();

  // V is always the STEP linear parameter
  // which may be swapped with the NURB parameter
  if (m_bSwapUV == FALSE)
    {
      if (pOptExtSurfParam) *pOptExtSurfParam = SM_SP_V;
      return( sNurbUVDomain.GetVInterval() ) ;
    }
  else
    {
      if (pOptExtSurfParam) *pOptExtSurfParam = SM_SP_U;
      return( sNurbUVDomain.GetUInterval() ) ;
    }

} // end SmSurfOfExtrusion::GetLinearParamExtent

/*******************************************************************//**
PURPOSE: Get NURB domain range of surface's GenCurve direction and optionally,
           the NURB Surf parameter associated with the GenCurve direction.

NOTES:  when m_bSwapUV == FALSE, pOptGenSurfParam = SM_SP_U
             m_bSwapUV == TRUE,  pOptGenSurfParam = SM_SP_V
***********************************************************************/
SmExtent1d SmSurfOfExtrusion::GetGenDirParamExtent
  (SmSurfParamType * pOptGenSurfParam)    // out: optional NURB param ivl which spans the GenDir direction
                                          //      NULL to ignore, default:[NULL]
 const
{
  // get m_pNurb UVDomain
  SmExtent2d sNurbUVDomain = GetNaturalUVDomain();

  // V is always the STEP linear parameter
  // which may be swapped with the NURB parameter
  if (m_bSwapUV == FALSE)
    {
      if (pOptGenSurfParam) *pOptGenSurfParam = SM_SP_U;
      return( sNurbUVDomain.GetUInterval() ) ;
    }
  else
    {
      if (pOptGenSurfParam) *pOptGenSurfParam = SM_SP_V;
      return( sNurbUVDomain.GetVInterval() ) ;
    }

} // end SmSurfOfExtrusion::GetGenDirParamExtent

/*******************************************************************//**
PURPOSE: Get the maximum allowable domain for an analytic representaion
   of this surface.

NOTES:
   The u/v parameters correspond to the STEP parameterization.

   The extrusion direction (v) is linear, so that can be big.
   In the u-direction, for gen-curves of degree 3 or higher, or even 2,
   this expansion can result in wildly pathological surface behavior
   even for the expansion factors used here.  Use this method with caution.
***********************************************************************/
SmExtent2d SmSurfOfExtrusion::GetMaxAnalyticDomain() const
{
  SmExtent2d sDom = GetSTEPUVDomain();

  // Expand u and v separately.
  SmExtent1d sUDom = m_pGenCurve->GetMaxAnalyticDomain();

  SmExtent1d sVDom = sDom.GetVInterval();
  sVDom.ExpandRelative( 100 );

  sDom.SetMinMax( sUDom.GetMin(), sVDom.GetMin(), sUDom.GetMax(), sVDom.GetMax() );

  return sDom;

} // end SmSurfOfExtrusion::GetMaxAnalyticDomain


/*******************************************************************//**
PURPOSE:
Return the 3d distance, along the extrusion vector,
from the origin to the start plane of the surface.
***********************************************************************/
double SmSurfOfExtrusion::GetBotHeight( void ) const
{
  return m_vAnalUVDomain.GetMin().y * m_vExtrusionVec.Length();
}

/*******************************************************************//**
PURPOSE:
Return the 3d distance, along the extrusion vector,
from the origin to the end plane of the surface.
***********************************************************************/
double SmSurfOfExtrusion::GetTopHeight( void ) const
{
  return m_vAnalUVDomain.GetMax().y * m_vExtrusionVec.Length();
}

/*******************************************************************//**
PURPOSE:

***********************************************************************/
SmVector3d SmSurfOfExtrusion::GetExtrusionVector( void ) const
{
  return m_vExtrusionVec;
}

/*******************************************************************//**
PURPOSE:

***********************************************************************/
SmPoint3d SmSurfOfExtrusion::GetOrigin( void ) const
{
  return m_vOrigin;
}

/*******************************************************************//**
PURPOSE: Create an offset surface for the extrusion.

NOTES:  acceleration based on offset surface = Extrusion of Offset Curve
       (offset curve in plane perpendicular to extrusion vector)
       Offset Surf->Domain(s) may be trimmed but not scaled so any
       OffsetSurface->EvaluatePoint(UVPoint) corresponds to the OriginSurface->EvaluatePoint(UVPoint)
***********************************************************************/

/* JLMCC This method was removed 11/18/2022 due to a bug, it generates multiple surfaces but only returns one. Fixing would require reworking the signature
* for all CreateOffsetSurface methods.
* 
SmStatus SmSurfOfExtrusion::CreateOffsetSurface
(
  const SmContext      & crContext,             // in : context for new obj construction
  double                 dSignedOffsetDistance, // in : offset dist, (neg val = Offset dir opposite surface normal)
  SmApproxTol3d          dThisApproxTol3d,      // in : Max Dist between ApproxOffsetSurface and ideal offset shape
  SmSurface* &           rOffsetSurface         // out: Offset Surf Approx, may be more than 1 when offsets have self-intersections
) const
{

  // locals
  SmTArray    <SmCurve*>        sCCurves;
  SmTArray    <SmBSplineCurve*> sTrimmedOffsets;
  SmObjsDelete<SmBSplineCurve*> sCleanOffs(&sTrimmedOffsets);

  // Copy the GenCurve and use it to make a composite curve
  SmCurve *pCopy;
  SER(m_pGenCurve->Copy(crContext,pCopy));
  sCCurves.Add(pCopy);
  SmCompositeCurve *pCC = new (crContext) SmCompositeCurve
                                            (3, sCCurves,
                                             m_pGenCurve->IsClosed(m_pGenCurve->GetNaturalInterval()),
                                             NULL, NULL);
  SmObjDelete sClean( pCC );

  // Now do offset of composite curve.

  // get the offset vector
  double     dOffsetDistance = smos_Fabs(dSignedOffsetDistance);
  double     dSign           =  (    (m_bSwapUV || dSignedOffsetDistance < 0.0)
                                 && !(m_bSwapUV && dSignedOffsetDistance < 0.0))
                               ? -1.0
                               :  1.0;
  SmVector3d sOffVec         = m_vExtrusionVec * dSign;

  // Offset the composite curve.

  // GWC_NEEDS_WORK GET_BETTER_VALUES_FOR_TOLERANCE_IN_FOLLOWING_CALL GWC_LINE ;
  SER(pCC->CreateTrimmedOffsets       // eff: call CreateOffsetsOfManyCurves for this curve.
       (crContext,                    // in : context for new object construction
        dThisApproxTol3d,             // in : Minimum distance at which offset curve end-gaps are filled with corner curves
        dThisApproxTol3d/10.0,        // in : Tolerance to which BSpline Approximations to exact offset curves are built
        sOffVec,                      // in : Defines, along with the curve's parameter direction,
                                      //      the right and left hand offset directions.
        SM_OC_FILLET_CORNER,          // in : SM_OC_LINEAR_EXTENSION: corner = 2 lines from given ends to common linear extension xsect point.
                                      //      SM_OC_FILLET_CORNER   : corner = fillet arc centered on crVertexPoint running to given end points
                                      //      SM_OC_LINEAR_CHAMFER  : corner = line between given end points (result is actually within offset distance so bTrimResults must = false)
        SM_OD_RIGHT_HAND_SIDE,        // in : Corresponding direction of each composite
                                      //      curve member to offset.  1-LEFT, 2-RIGHT, 3-BOTH
        TRUE,                         // in : TRUE = concave raw offsets are intersected and trimmed back to common intersection points
                                      //      FALSE= skip trim step
        dOffsetDistance,              // in : offset distance (a negative value negates the offset direction)
        sTrimmedOffsets));            // out: Resulting new offset curves will have attribute attached
                                      //      describing origination of curve
                                      // in : TRUE = if BSplineCurve just copy it (preserves CrvParams)
                                      //      FALSE= approximate BSplineCurves (changes CrvParams), NonBSplineCrvs always Approximated
                                      //      default:[FALSE]

  // check that parameterization range is being preserved
  SM_ASSERT(   sTrimmedOffsets.GetSize() > 0
            && SM_ARE_SAME(sTrimmedOffsets[0]->GetNaturalInterval().GetMin(), m_pGenCurve->GetNaturalInterval().GetMin())
            && SM_ARE_SAME(sTrimmedOffsets[sTrimmedOffsets.GetSize()-1]->GetNaturalInterval().GetMax(), m_pGenCurve->GetNaturalInterval().GetMax())) ;

  // for every trimmed offset curve
  for (ULONG i=0; i<sTrimmedOffsets.GetSize(); i++)
    {
      SmBSplineCurve *pOffCurve = sTrimmedOffsets[i];

      // simplify the OffsetCurve
      SER(pOffCurve->RemoveExtraKnots(dThisApproxTol3d/100.0));

      // OffsetCurve locals needed for upcoming constructor call
      SmPoint3d sOrigin ;
      SmExtent1d sOffCurveIvl = pOffCurve->GetNaturalInterval() ;
      pOffCurve->EvaluatePoint(sOffCurveIvl.GetMin(), sOrigin) ;
      SmExtent2d sAnalUVDomain(sOffCurveIvl.GetMin(), m_vAnalUVDomain.GetMin().y,
                               sOffCurveIvl.GetMax(), m_vAnalUVDomain.GetMax().y) ;

      // make a new SmSurfOfExtrusion from the offset trim curve
      SmSurfOfExtrusion *pOffSurf = new (crContext) SmSurfOfExtrusion(pOffCurve,
                                                                      sOrigin,
                                                                      m_vExtrusionVec,
                                                                      sAnalUVDomain,
                                                                      m_bSwapUV) ;

      // now using pOffCurve - don't clean it up
      sTrimmedOffsets.SetAt(i, NULL) ;

      //      // sweep the offset curve
      //      //  This call builds an extrusion NURB surface whose param directions are
      //      //  swapped from the STEP SmSurfOfExtrusion directions, i.e.
      //      //    SmSurfOfExtrusion Sweep Param = SM_SP_V (constant u isoParamCurves are lines)
      //      //    this NURB surface Sweep Param = SM_SP_U (constant v isoParamCurves are lines)
      //      //  UVDomain =  { (0.0, CurveToSweep_Ivl->Min),
      //      //                (1.0, CurveToSweep_Ivl->Max) }
      //      SmBSplineSurface *pOffSurf         = NULL ;
      //      double            dExtrusionLength = m_vAnalUVDomain.GetSize().y;
      //      SER(SmBSplineSurface::CreateLinearSweep(crContext,
      //                                             *pOffCurve,
      //                                              dExtrusionLength*m_vExtrusionVec,
      //                                              pOffSurf));
      //
      //
      //      // Change newly constructed pOffSet UVDomain to
      //      //   UVDomain =  { (ExtrusionSurfaceLinearIvl->Min, CurveToSweep_Ivl->Min),
      //      //                 (ExtrusionSurfaceLinearIvl->Max, CurveToSweep_Ivl->Max) }
      //      SmExtent2d sDomain = pOffSurf->GetNaturalUVDomain();
      //      SmExtent1d sIvl    = GetSTEPLinearParamExtent(NULL);
      //      sDomain.SetMinMax(SmPoint2d(sIvl.GetMin(),sDomain.GetMin().y),
      //                        SmPoint2d(sIvl.GetMax(),sDomain.GetMax().y));
      //      pOffSurf->Reparameterize(sDomain, FALSE);
      //
      //      // see if newly constructed pOffSet Surface needs to have its UV directions swapped
      //      // to comply with current value of the m_bSwapUV flag.
      //      // note: when m_bSwapUV == TRUE - everything is already OK.
      //      if (!m_bSwapUV)
      //        { SER(pOffSurf->SwapUV()); }

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
      if (bDebugMe)
        {
          smgfx_Erase();
          pOffSurf->DrawUV(4,4);
          sm_GraphicsLoop();
          DrawUV(10,10);
          sm_GraphicsLoop();
          pOffSurf->Dump();
          Dump();
        }
#endif // SM_DEBUG_CODE

//        SER(pOffSurf->Reparameterize(GetNaturalUVDomain()));

      // add surface copy to output
      SM_DUMP_AND_ASSERT2_VALID(pOffSurf);
      rOffsetSurface = pOffSurf;

    } // end iter every offset curve

  // all done
  return SM_SUCCESS;

} // end SmSurfOfExtrusion::CreateOffsetSurface
*/
/*******************************************************************//**
PURPOSE: Given an SmSurfOfExtrusion and a parameter value in the
            V sweep direction of the UVANalDomain,
            return appropriately shifted copy of GenCurve.

NOTES: The given parameter must be within the V range of m_vAnalUVDomain.

  This is nearly a constant IsoParameter curve function except
  that the input dLocationAlongExtrusionArg only corresponds exactly
  to a STEP V parameter value when the AnalUVDomain V Range starts at zero.
***********************************************************************/
SmStatus SmSurfOfExtrusion::CreateGeneratorFromDistance
  (const SmContext & crContext,                   // in : context for new object construction
   double            dVStepParam,                 // in : Tgt parameter, range:[m_vAnalUVDomain.GetMin().y,
                                                  //                            m_vAnalUVDomain.GetMax().y]
   SmBSplineCurve *& rpGeneratorCurveArg)         // out: appropriately translated version of GenCurve
  const

{
  // clamp out of range parameters to m_vAnalUVDomain
  double dExtrusionParam = m_vAnalUVDomain.GetUInterval().ClampValue(dVStepParam) ;

  // bad input - param value is not in or near m_vAnalUVDomain V interval
  if(!SM_IS_ZERO_TO_TOL(dExtrusionParam - dVStepParam, SM_EFF_ZERO_SQRT))
    { SER(SM_ERR); }

  // copy the gen curve
  SmBSplineCurve* pBSC = new (crContext) SmBSplineCurve(*m_pGenCurve);
  NER(pBSC);

  // build a translation transformation matrix
  SmAxis2Placement sRF;
  sRF.Translate(dVStepParam * m_vExtrusionVec);

  // translate the copied curve
  pBSC->Transform(sRF);

  // set output
  rpGeneratorCurveArg = pBSC;

  // all done
  return SM_SUCCESS;

} // end SmSurfOfExtrusion::CreateGeneratorFromDistance

/*******************************************************************//**
PURPOSE: Given a point on an SmSurfOfExtrusion surface, return
            the directrix passing through the given point.

            Directrices are lines, parallel to the extrusion vector,
            passing through the given point  This routine returns the
            portion of that line that corresponds to this extruded surface.

NOTES: The point need not be on the generator curve but must be on the surface.
***********************************************************************/
SmStatus SmSurfOfExtrusion::CreateDirectrixFromPoint
  (const SmContext & crContext,       // in : context for new object construction
   const SmPoint3d & rPointOnSurface, // in : Point to interpolate
   double            dTol,            // in : max 3d distance for out of domain points
   SmBSplineCurve *& rpDirectrix)     // out: lineSeg through rPointOnSurface with length
                                      //      and direction of ExtrusionVector
  const
{
  // This function obtains the directrix from geometry.
  // without knowing its UV representation.

  // Find the height of the given point along the sweep direction:
  // how far along the surface sweep parameter range.
  double dLineParam, dCurrentHeight = 0;

  // switch on GenCurve Orientation
  switch(m_eOrientation)
    {
      case SM_CO_LINEAR :        // get distance along line:[m_vOrigin, ExtrusionVec] to line:[m_vCurvePoint, m_vCurveVec]
                                 SER(smgu_LineLineClosestPoint(m_vOrigin,       m_vExtrusionVec,
                                                               m_vCurvePoint,   m_vCurveVec,
                                                               dCurrentHeight,  dLineParam)) ;
                                 break ;
      case SM_CO_PERPENDICULAR : // get closest point on line:[origin, ExtrusionVec] to rPointOnSurface
                                 SER(smgu_LineClosestPoint(m_vOrigin,
                                                           m_vExtrusionVec,
                                                           rPointOnSurface,
                                                           dCurrentHeight));
                                 break ;

      case SM_CO_INDEPENDENT :   // Intersect line:[PointOnSurface,ExtrusionVec] with plane containing curve.
                                 SER(smgu_LinePlaneIntersect(rPointOnSurface,
                                                             m_vExtrusionVec,
                                                             m_vOrigin,
                                                             m_vCurveVec,
                                                             dLineParam));
                                 dCurrentHeight    = -dLineParam;
                                 break ;
      case SM_CO_COPLANAR :
      case SM_CO_NOT_PLANAR :    // intersect line:[PointOnSurface, ExtrusionVec] with GenCurve
                                 {
                                   // line and curve locals
                                   SmLine sLine( rPointOnSurface, m_vExtrusionVec, 3, FALSE, &crContext );
                                   SmExtent1d sLineDomain( -10000, 10000 );  // sort of infinite
                                   SmExtent1d sCurveDomain = GetSTEPUVDomain().GetUInterval();

                                   // Get Curve/Line intersection
                                   SmBoolean bNeedsMoreIntersections;
                                   SmSolutionArray sSolutions;
                                   GetGenCurve()->IntersectWithLine(sCurveDomain,
                                                                    sLine,
                                                                    sLineDomain,
                                                                    dTol,
                                                                    bNeedsMoreIntersections,
                                                                    sSolutions);

                                   if ( bNeedsMoreIntersections || sSolutions.GetSize() < 1 )
                                     {
                                       GetGenCurve()->GlobalCurveIntersect(sCurveDomain,
                                                                           sLine,
                                                                           sLineDomain,
                                                                           dTol,
                                                                           sSolutions );
                                     }

                                   // CurrentHeight equals negative Line Param solution
                                   if ( sSolutions.GetSize() > 0 )
                                     {
                                       dCurrentHeight = -sSolutions[0].m_vStart[1];
                                     }
                                   else
                                     { SER(SM_ERR) ; }
                                 }
                                 break ;
      case SM_CO_UNDEFINED:      break;
  } // end switch on m_vOrientation

  // arrive here after getting dCurrentHeight.

  // get surface sweep parameter range
  SmExtent1d sHeightIvl = GetSTEPLinearParamExtent(NULL);

  // snap projected points close to Domain boundaries
  if ( sHeightIvl.IsValueOnBoundary( dCurrentHeight, dTol ))
    {
      dCurrentHeight = sHeightIvl.ClampValue(dCurrentHeight);
    }

  // check state - projected point not contained in domain range
  // No: this is not a problem.  [SMS12]
  // if (!sHeightIvl.ContainsValue(dCurrentHeight))
  //   {
  //     SER(SM_ERR);
  //   }

  // create line running through rPointOnSurface from domain start to end
  SmPoint3d sBotPt( rPointOnSurface - ( dCurrentHeight - sHeightIvl.GetMin() ) * m_vExtrusionVec );
  SmPoint3d sTopPt( rPointOnSurface + ( sHeightIvl.GetMax() - dCurrentHeight ) * m_vExtrusionVec );

  SER(SmBSplineCurve::CreateLineSegment(crContext,
                                        3,            // lDimensionOfResult
                                        sBotPt, sTopPt,
                                        rpDirectrix));
  NER( rpDirectrix );

  // all done
  return SM_SUCCESS;

} // end SmSurfOfExtrusion::CreateDirectrixFromPoint

/*******************************************************************//**
PURPOSE: Given a point in Euclidian space determine the corresponding
     extrema points on the surface based on STEP-parametrization.
     Valid solver operations for this method include:
     SM_SO_MINIMIZE, SM_SO_MAXIMIZE, SM_SO_NORMALIZE, SM_SO_INTERSECT.

NOTES:
***********************************************************************/
SmStatus SmSurfOfExtrusion::GlobalPointSolveSTEP
  (const SmExtent2d      & crAnalUVDomain,            // in : STEP-based Domain of surface
   SmSolverOperationType   eSolverOperation,          // in : oneof SM_SO_MINIMIZE
                                                      //            SM_SO_MAXIMIZE
                                                      //            SM_SO_NORMALIZE
                                                      //            SM_SO_INTERSECT
   const SmPoint3d       & crTestPoint,               // in : target point
   double                  dDistanceTolerance,        // in :
   const double          * cpdOptTargetDistance,      // in :
   SmSolutionRequestedType eSolutionRequested,        // in : oneof SM_SR_ALL, SM_SR_SINGLE
   SmSolutionArray       & rSolutions)                // out:
{
  SM_ASSERT(   eSolverOperation == SM_SO_MINIMIZE
            || eSolverOperation == SM_SO_MAXIMIZE
            || eSolverOperation == SM_SO_NORMALIZE
            || eSolverOperation == SM_SO_INTERSECT);

  // validate
  SM_DUMP_AND_ASSERT2_VALID(this) ;

  // map given STEP sub-domain into Nurb domain
  SmExtent2d sNurbUVDomain = crAnalUVDomain ;
  if(m_bSwapUV) { sNurbUVDomain.Transpose() ; }

  // gwc: old method
  //      // Build a Nurb Surface to given subDomain
  //      // gwc Not a good idea - let's not copy the surface to solve this problem
  //      SER(AdjustSTEPUVDomain(crAnalUVDomain));
  //      SmExtent2d sNurbUVDomain = GetNaturalUVDomain();

  // Pass call along to NURBS-based solver
  SER(GlobalPointSolve(sNurbUVDomain,
                       eSolverOperation,
                       crTestPoint,
                       dDistanceTolerance,
                       cpdOptTargetDistance,
                       eSolutionRequested,
                       rSolutions));

  // no solutions - all done
  if(rSolutions.GetSize() == 0)
    { return SM_SUCCESS; }

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
    if (bDebugMe)
      {
        SmPoint3d sPt,sDU,sDV;
        Evaluate1stDerivatives(SmPoint2d(0,0),TRUE,TRUE,sPt,sDU,sDV);

        smgfx_Erase();
        smgfx_SetColor(0,0,0); DrawUV(4,4); sm_GraphicsLoop();
        smgfx_SetColor(1,0,0); if(m_pGenCurve->IsBounded()) m_pGenCurve->Draw(); sm_GraphicsLoop() ;
        smgfx_SetColor(1,0,0); crTestPoint.Draw(); sm_GraphicsLoop();
        sm_GraphicsLoop();
      }
#endif // SM_DEBUG_CODE

    // Process all solutions

    // gwc: removed because I couldn't figure out the need for it.
    //      SmPoint2d sUVSize = crAnalUVDomain.GetSize();
    //      double    dUVTol  = 100.0*SM_EFF_ZERO_SQRT*(1.0+sUVSize.x+sUVSize.y);

    // for every solution
    for (ULONG i=0; i<rSolutions.GetSize(); i++)
      {
        SmSolution & rSol = rSolutions[i];

        // Nurb UParam = rSol.m_vStart[0]
        // Nurb VParam = rSol.m_vStart[1]

        // when m_pGenCurve is not analytic
        //  NurbUVPoints = StepUVPoints modulo m_bSwapUV
        if (!m_pGenCurve->IsAnalytic())
          {
            if (m_bSwapUV)
              { SM_SWAP(double, rSol.m_vStart[0], rSol.m_vStart[1]) ; }
            continue;
          }

        else // m_pGenCurve is Analytic
          {

            // else m_pGenCurve is analytic and NurbUPoints != StepUPoints
            // convert NurbUVPoints into StepUVPoints
            SmPoint2d sNurbUV(rSol.m_vStart[0], rSol.m_vStart[1]) ;
            SmPoint2d sStepUV ;
            ConvertUVFromNURBSToSTEP(sNurbUV, sStepUV) ;

            // save the Step points in the solution
            rSol.m_vStart[0] = sStepUV.x;
            rSol.m_vStart[1] = sStepUV.y;

          } // end m_pGenCurve is analytic branch


        // gwc: I don't see why we should move a redundant solution.
        //      Moreover, Odds are this rarely or no longer runs now that
        //      the global point solver does a better job of not
        //      returning multiple answers.  As such I'm removing this bit of code.

        // gwc: removed
        //      for (ULONG jj=0; jj<i; jj++)
        //        {
        //          SmSolution & rExistedSol = rSolutions[jj];
        //          SmPoint2d sExistedUV(rExistedSol.m_vStart[0],
        //                               rExistedSol.m_vStart[1]);
        //          if (sStepUVOnGenCurve.DistanceBetween(sExistedUV) > dUVTol)
        //              continue;
        //          rSol.m_vStart[0] = crAnalUVDomain.GetMax().x;
        //        }

      } // end iter every solution

    // all done
    return SM_SUCCESS;

} // end SmSurfOfExtrusion::GlobalPointSolveSTEP

/*******************************************************************//**
PURPOSE: This virtual method may invoke special cases of intersection
    of analytics with planes.

NOTES:
***********************************************************************/
SmStatus SmSurfOfExtrusion::GlobalSurfaceIntersect
  (const SmContext     & crContext,               // in : Context for the creation of curves
   const SmExtent2d    & crUVDomain,              //
   const SmSurface     & crOtherSurface,          // in : target 2nd intersecting surface
   const SmExtent2d    & crOtherUVDomain,         //
   const SmBoolean       bUseSurfaceEdges[2],     // in :  = Normally both are TRUE unless you know
                                                  //      that the edges of one surface do not intersect
                                                  //      the other surface.  It is a slight optimization
                                                  //      to set the flag to FALSE
   const SmApproxTol3d * pdOptApproxTol3d,        // in : If not given it uses 1/1000 of surface size
                                                  //      approximation tolerance
   const double        * pdOptAngTolRad,          // in : If not given it uses 30 degrees
   SmTArray<SmCurve*>  * pOpt3DCurves,            // out: 3D curves produced by intersection
   SmTArray<SmCurve*>  * pOptSurface1UVCurves,    // out: UV curves on this surface produced by intersection
   SmTArray<SmCurve*>  * pOptSurface2UVCurves,    // out: UV curves on crOtherSurface produced by intersection
   SmTArray<SmTsectCurveType> * pOptCurveTypes,   // out: What type of curve is produced -
                                                  //      oneof: SM_TC_TOUCHING   - single point intersection (surf norms parallel)
                                                  //             SM_TC_CROSSING   - curve intersection (surf norms not parallel)
                                                  //             SM_TC_TANGENT    - curve intersection (surf norms parallel)
                                                  //             SM_TC_COINCIDENT - curve intersection (surf norms parallel & cross-tangents equal)
                                                  //             SM_TC_NEAR_TANGENT    - curve has small angle of intersection
                                                  //             SM_TC_REGION_BOUNDARY - curve bounds region within which the surfs are coincident
   SmTArray<double>    * pOptDeviations)          // out:
  const
{
  SmBoolean bNeedNurbIntersection = TRUE;
  switch (crOtherSurface.GetType())
    {
      case SmPlane_TYPE:
          SER(IntersectWithPlane(crContext, crUVDomain, (SmPlane&)crOtherSurface,
                                 crOtherUVDomain, bUseSurfaceEdges, pdOptApproxTol3d,
                                 pdOptAngTolRad, bNeedNurbIntersection,
                                 pOpt3DCurves, pOptSurface1UVCurves,
                                 pOptSurface2UVCurves, pOptCurveTypes, pOptDeviations));
          break;

      case SmCone_TYPE:
          SER(IntersectWithCylinder(crContext, crUVDomain, (SmCone&)crOtherSurface,
                                    crOtherUVDomain, bUseSurfaceEdges, pdOptApproxTol3d,
                                    pdOptAngTolRad, bNeedNurbIntersection,
                                    pOpt3DCurves, pOptSurface1UVCurves,
                                    pOptSurface2UVCurves, pOptCurveTypes, pOptDeviations));
          break;

      case SmSurfOfExtrusion_TYPE:
          SER(IntersectWithSurfOfExtrusion(crContext, crUVDomain,
                                           (SmSurfOfExtrusion&)crOtherSurface,
                                           crOtherUVDomain, bUseSurfaceEdges, pdOptApproxTol3d,
                                           pdOptAngTolRad, bNeedNurbIntersection,
                                           pOpt3DCurves, pOptSurface1UVCurves,
                                           pOptSurface2UVCurves, pOptCurveTypes, pOptDeviations));
          break;

      default:
          break;
    }

  // when bNeedNurbIntersection == TRUE, intersection was too complicated for analytic intersectors
  if (bNeedNurbIntersection)
    {
      SER(SmSurface::GlobalSurfaceIntersect(crContext,crUVDomain,crOtherSurface,
                                            crOtherUVDomain,bUseSurfaceEdges,pdOptApproxTol3d,
                                            pdOptAngTolRad,pOpt3DCurves,pOptSurface1UVCurves,
                                            pOptSurface2UVCurves,pOptCurveTypes,pOptDeviations));
    }

  // all done
  return SM_SUCCESS;

} // end SmSurfOfExtrusion::GlobalSurfaceIntersect

/*******************************************************************//**
PURPOSE: Drop a point to a surface of Extrusion very fast.
    Note that this is an optimization routine and should not be
    used for general dropping.
    For general surfaces this method returns an error.

NOTES: For now, it will skip non-perpendicular extrusion cases.
***********************************************************************/
SmStatus SmSurfOfExtrusion::DropPointFast
  (const SmExtent2d      & crUVDomain,                 // in : Domain of surface to search for solutions
   SmSolverOperationType   eSolverOperation,           // in : oneof: SM_SO_MINIMIZE, SM_SO_MAXIMIZE, SM_SO_NORMALIZE, SM_SO_INTERSECT
   const SmPoint3d       & crTestPoint,                // in : target point
   const SmVector3d      * cpOptInPointingVector,      // in : specifies end (EndTang = cpOptInPointingVector) saved for drops on closed curve seams
   double                  dDistanceTolerance,         // in :
   const double          * cpdOptTargetDistance,       // in : The pdOptTargetDistance, if not NULL, will be the corresponding
                                                       //      limit to a minimize/maximize operations.  In otherwords, it will
                                                       //      ask the solver to find a minimum value only if it is less than
                                                       //      the target distance or maximum value only if it is greater than
                                                       //      the target distance.
   SmSolutionRequestedType eSolutionRequested,         // in : SM_SR_SINGLE=get best solution, SM_SR_ALL=get all solutions
   SmSolutionArray       & rSolutions)                 // out: array of problem solutions reported as nurb surface UV parameter values
   const
{
  // return SM_ERR when this function can't compute a solution.
  // callers of this function will trap the return and call the
  // more general DropPoint function.
  if(m_eOrientation != SM_CO_PERPENDICULAR)
    { return SM_ERR; }

  // init output
  rSolutions.ReSet();

  // locals
  SmExtent2d sNatDomain  = GetNaturalUVDomain();
  SmVector3d sLineVec    = m_vAnalUVDomain.YLength() * m_vExtrusionVec ;
  SmPoint3d  sLineOrigin = m_vOrigin + m_vAnalUVDomain.GetMin().y * m_vExtrusionVec;
  double     dLineParam ;
  double     dPointInPlaneParam ;
  SmExtent1d sIvl        =   m_bSwapUV
                           ? crUVDomain.GetVInterval()
                           : crUVDomain.GetUInterval() ;

  // If we are allowing out-of-bounds eval, then the gen curve needs to too.
  SmSurfOfExtrusion * pNonConstThis = SM_CONST_CAST( SmSurfOfExtrusion*, this );
  SmTemporaryChangeValue<SmBoolean> sChangeOB( m_pGenCurve  ->GetOutOfBoundsEnabled(),
                                               pNonConstThis->GetOutOfBoundsEnabled() );

  // Get and save the sweep param value
  //  method: Project TestPoint to line:[orig,ExtrusionVec]
  //          and scale that LineParam to a sweep V param
  SER(smgu_LineClosestPoint(sLineOrigin,sLineVec,crTestPoint,dLineParam));
  if (dLineParam < 0.0) dLineParam = 0.0;
  if (dLineParam > 1.0) dLineParam = 1.0;
  SmPoint2d sUVPoint = sNatDomain.Evaluate(dLineParam,dLineParam);

  // Project TestPoint to GenCurve Plane -
  //  method: project m_vOrigin to line[TestPoint,ZAxis]
  SER(smgu_LineClosestPoint(crTestPoint,sLineVec,m_vOrigin,dPointInPlaneParam));
  SmPoint3d sPointInPlane = crTestPoint + dPointInPlaneParam * sLineVec;

  // Now solve the problem for TestPoint projected to GenCurvePlane against m_pGenCurve
  SmSolution sSData[16];
  SmSolutionArray sSolutions(16,sSData);
  SER(m_pGenCurve->GlobalPointSolve(sIvl,
                                    eSolverOperation,
                                    sPointInPlane,
                                    dDistanceTolerance,
                                    cpdOptTargetDistance,
                                    NULL,
                                    eSolutionRequested,
                                    sSolutions));

  SmSolution sSol;
  sSol.m_eSolutionType = SM_ST_SINGLE_VALUE;
  sSol.m_lNumObjects   = 1;
  sSol.m_apObjects[0]  = SM_CONST_CAST(SmSurfOfExtrusion*,this);
  sSol.m_apNodes[0]    = NULL;
  sSol.m_lNumVariables = 2;

  // for every solution - add a solution to the output solution array
  for (ULONG j=0; j<sSolutions.GetSize(); j++)
    {
      const SmSolution & crSol = sSolutions[j];
      double             dT    = crSol.m_vStart[0];

      // when given an option interior pointing vector
      // and we have a possible set of seam solutions
      //  - skip solutions not wanted
      if (   sSolutions.GetSize() > 1
          && cpOptInPointingVector
          && m_pGenCurve->IsPeriodic(sIvl))
        {
          // when solution is at start of curve
          if (SM_ARE_SAME(dT,sIvl.GetMin()))
            {
              // skip solution when opt vector points out of the curve
              SmVector3d sPV[2];
              SER(m_pGenCurve->Evaluate(dT,1,TRUE,sPV));
              double dDot = sPV[1].Dot(*cpOptInPointingVector);
              if (dDot < -SM_EFF_ZERO_SQRT)
                {
                  continue;
                }
            }

          // when solution is at end of curve
          if (SM_ARE_SAME(dT,sIvl.GetMax()))
            {
              // skip solutions when opt vector points out of the curve
              SmVector3d sPV[2];
              SER(m_pGenCurve->Evaluate(dT,1,TRUE,sPV));
              double dDot = sPV[1].Dot(*cpOptInPointingVector);
              if (dDot > SM_EFF_ZERO_SQRT)
                {
                  continue;
                }
            }
        } // end need to skip seam solutions due to opt interior pointing vector check

      // set the output GenCurve parameter value (sweep param value is already set)
      if (m_bSwapUV) { sUVPoint.y = crSol.m_vStart[0]; }
      else           { sUVPoint.x = crSol.m_vStart[0]; }

      // compute amd save the drop point distance as the m_dSolutionValue
      SmPoint3d sPointOnSurf;
      SER(EvaluatePoint(sUVPoint,sPointOnSurf));
      double dDist = sPointOnSurf.DistanceBetween(crTestPoint);
      sSol.m_vStart.m_dSolutionValue = dDist;

      // save the UVPoint value as the Solution Param values
      sSol.m_vStart[0] = sUVPoint.x;
      sSol.m_vStart[1] = sUVPoint.y;

      // add solution to output array
      rSolutions.Add(sSol);

    } // end iter every solution adding an entries into the output solutionArray

  // all done
  return SM_SUCCESS;

} // end SmSurfOfExtrusion::DropPointFast

/*******************************************************************//**
PURPOSE: Find intersection curve (line) of this with other plane

NOTES:
***********************************************************************/
SmStatus SmSurfOfExtrusion::IntersectWithPlane
  (const SmContext     & crContext,                  // in : context for new object construction
   const SmExtent2d    & crUVDomain,                 // in : intersection limit in Nurb Domain for this surface
   const SmPlane       & crPlane,                    // in : target intersection plane
   const SmExtent2d    & crOtherUVDomain,            // in : intersection limit for target plane
   const SmBoolean       /* bUseSurfaceEdges */[2],  // in : TRUE = find xsect curve start points from boundaryCurve/surface xsects
                                                     //      typically these values are TRUE - its a small savings if you know
                                                     //      the boundaries of one surface don't intersect the other surface
   const SmApproxTol3d * pdOptApproxTol3d,           // in :
   const double        * /* pdOptAngTolRad */,       // in :
   SmBoolean           & rbNeedsMoreIntersections,   // out: TRUE = special case intersection failed-use general intersection
   SmTArray<SmCurve*>  * pOpt3DCurves,               // out: Intersection 3DCurves, NULL to ignore
   SmTArray<SmCurve*>  * pOptSurface1UVCurves,       // out: associated UVTrimCurves on this surface, NULL to ignore
   SmTArray<SmCurve*>  * pOptSurface2UVCurves,       // out: associated UVTrimCurves on plane, NULL to ignore
   SmTArray<SmTsectCurveType> * pOptCurveTypes,      // out: one of for each 3DCurve, NULL to ignore
                                                     //      SM_TC_TOUCHING        - single point intersection (surf norms parallel)
                                                     //      SM_TC_CROSSING        - curve intersection (surf norms not parallel)
                                                     //      SM_TC_TANGENT         - curve intersection (surf norms parallel)
                                                     //      SM_TC_COINCIDENT      - curve intersection (surf norms parallel & cross-tangents equal)
                                                     //      SM_TC_NEAR_TANGENT    - curve has small angle of intersection
                                                     //      SM_TC_REGION_BOUNDARY - curve bounds region within which the surfs are coincident
   SmTArray<double>    * pOptDeviations)             // out: associated max 3DCurve to surface distance, NULL to ignore
 const
{
  SM_DUMP_AND_ASSERT2_VALID(this) ;

#ifdef SM_DEBUG_CODE // draw input surfaces and their breps and faces if possible
SmBoolean bDebugMe = FALSE;
  static ULONG lCount      = 1 ; lCount++ ;
  static ULONG lDebugCount = 0 ;
  if (bDebugMe || lCount == lDebugCount)
    {
      SmFace *pFace1 = (SmFace *)GetFace() ;
      SmFace *pFace2 = (SmFace *)crPlane.GetFace() ;
      SmBrep *pBrep1 = pFace1 ? pFace1->GetBrep() : NULL ;
      SmBrep *pBrep2 = pFace2 ? pFace2->GetBrep() : NULL ;

      smgfx_Erase();
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep1) { pBrep1->Draw(TRUE) ; } sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,0) ; if(pBrep2) { pBrep2->Draw(TRUE) ; } sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,1) ; DrawUV() ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 1,1,0) ; crPlane.DrawUV() ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,0,0) ; if(pFace1) { pFace1->Draw(SM_DM_CROSSHATCH,10,10) ; } sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,0,0) ; if(pFace2) { pFace2->Draw(SM_DM_CROSSHATCH,10,10) ; } sm_GraphicsLoop() ;
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

  // init output
  rbNeedsMoreIntersections = FALSE;
  if (pOpt3DCurves)         { pOpt3DCurves->ReSet(); }
  if (pOptSurface1UVCurves) { pOptSurface1UVCurves->ReSet(); }
  if (pOptSurface2UVCurves) { pOptSurface2UVCurves->ReSet(); }
  if (pOptCurveTypes)       { pOptCurveTypes->ReSet(); }
  if (pOptDeviations)       { pOptDeviations->ReSet(); }

  // plane and ExtrusionSurface locals
  ULONG ii ;
  SmVector3d       sPlaneNormal  = crPlane.GetPosition().GetZAxis();
  const SmPoint3d &rPlaneOrigin  = crPlane.GetPosition().GetOriginRef();
  double           dAngTol       = SM_EFF_ZERO_SQRT;
  double           dTol          =  0.0;
  if( pdOptApproxTol3d ) dTol = smos_Min((double)*pdOptApproxTol3d, SM_EFF_ZERO_SQRT);
  else dTol = SM_EFF_ZERO_SQRT;

  SmExtent1d sGenCurveNurbIvl    = m_bSwapUV ? crUVDomain.GetVInterval()
                                             : crUVDomain.GetUInterval() ;
  SmVector3d sSweptNormal ;
  switch(m_eOrientation)
    {
      case SM_CO_LINEAR   : sSweptNormal = m_vCurveVec * m_vExtrusionVec ; break ;
      case SM_CO_COPLANAR : sSweptNormal = m_vCurveVec ; break ;
      default             : break ;
    }

  // If we are allowing out-of-bounds eval, then the gen curve needs to too.
  SmSurfOfExtrusion * pNonConstThis = SM_CONST_CAST( SmSurfOfExtrusion*, this );
  SmTemporaryChangeValue<SmBoolean> sChangeOB( m_pGenCurve  ->GetOutOfBoundsEnabled(),
                                               pNonConstThis->GetOutOfBoundsEnabled() );

  // When GenCurve is linear or Planar and swept in its own plane
  // and SweptPlanarSurface is not parallel to input Surface
  // Then,
  //   XSect = Line
  if(   (   m_eOrientation == SM_CO_LINEAR
         || m_eOrientation == SM_CO_COPLANAR)
     && !sSweptNormal.IsParallelTo(sPlaneNormal, dAngTol))
    {
      // build a plane bigger than the Surface
      SmPlane sThisPlane(m_vCurvePoint, sSweptNormal) ;
      sThisPlane.SetContext(GetContext()) ;

      // get the Plane/Plane intersection line trimmed to input Plane
      //  builds an SmLine or an SmBSplineCurve made by CreateDegenerateCurve().
      //  note: XSect curve gets trimmed to swept curve bounding box - coming up.
      SmBoolean bUseSurfaceEdges[2] ;
      SmTArray<SmCurve*> s3dCurves ; SmObjsDelete<SmCurve*>  sClean1( &s3dCurves) ;
      crPlane.IntersectWithPlane(crContext, crOtherUVDomain,
                                 sThisPlane, sThisPlane.GetSTEPUVDomain(), bUseSurfaceEdges,
                                 pdOptApproxTol3d, NULL, rbNeedsMoreIntersections,
                                 &s3dCurves, NULL, NULL, NULL, NULL) ;

#ifdef SM_DEBUG_CODE
      if (bDebugMe || lCount == lDebugCount)
        {
          for(ii=0;ii<s3dCurves.GetSize();ii++)
            { s3dCurves[ii]->Dump() ; }

          SmFace *pFace1 = (SmFace *)GetFace() ;
          SmFace *pFace2 = (SmFace *)crPlane.GetFace() ;
          SmBrep *pBrep1 = pFace1 ? pFace1->GetBrep() : NULL ;
          SmBrep *pBrep2 = pFace2 ? pFace2->GetBrep() : NULL ;

          smgfx_Erase();
          smgfx_SetLook(1,2, 0,0,1) ; if(pBrep1) { pBrep1->Draw(TRUE) ; } sm_GraphicsLoop() ;
          smgfx_SetLook(1,2, 0,1,0) ; if(pBrep2) { pBrep2->Draw(TRUE) ; } sm_GraphicsLoop() ;
          smgfx_SetLook(1,2, 0,1,1) ; DrawUV(8,8,FALSE,NULL,TRUE) ; sm_GraphicsLoop() ;
          smgfx_SetLook(1,2, 1,1,0) ; crPlane.DrawUV(8,8,FALSE,NULL,TRUE) ; sm_GraphicsLoop() ;
          smgfx_SetLook(1,2, 1,.5,0); sThisPlane.DrawUV(8,8,FALSE,NULL,TRUE) ; sm_GraphicsLoop() ;
          smgfx_SetLook(1,2, 0,0,0) ; if(pFace1) { pFace1->Draw(SM_DM_CROSSHATCH,10,10) ; } sm_GraphicsLoop() ;
          smgfx_SetLook(1,2, 0,0,0) ; if(pFace2) { pFace2->Draw(SM_DM_CROSSHATCH,10,10) ; } sm_GraphicsLoop() ;
          smgfx_SetLook(2,3, 1,0,0) ; for(ii=0;ii<s3dCurves.GetSize();ii++)
                                       { s3dCurves[ii]->Draw() ; sm_GraphicsLoop() ; }
          sm_GraphicsLoop();
        }
#endif // SM_DEBUG_CODE

      // no work - no solutions
      if(s3dCurves.GetSize() == 0)
        { return SM_SUCCESS ; }

      // arrive here when two planes intersect in one line
      SM_ASSERT(s3dCurves.GetSize() == 1) ;

      // classify the 3dCurves against the natural boundaries of this surface (for trimming)
      SmCurveClassification sCurveClassification(s3dCurves[0],
                                                 s3dCurves[0]->GetNaturalInterval(),
                                                 NULL,
                                                 dTol) ;
      this->CurveOnClassify(sCurveClassification) ;

      // for every classification interval in this surface
      for(ii=0;ii<sCurveClassification.GetSize();ii++)
        {
          SmCurveInterval &sCurveInterval = sCurveClassification[ii] ;

          // skip intervals outside the surface
          if(sCurveInterval.m_vMid.GetPointClass() == SM_PC_UNKNOWN)
            { continue ; }

          // add interval to output
          SmExtent1d sIvl = sCurveInterval.GetInterval() ;

          // quick case - the whole curve
          if(sIvl.AreEqual(s3dCurves[0]->GetNaturalInterval(), SM_EFF_ZERO))
            {
              // set output
              sClean1.Clear() ;
              if(pOpt3DCurves)   { pOpt3DCurves->Add(s3dCurves[0]) ; }
              if(pOptCurveTypes) { pOptCurveTypes->Add(SM_TC_CROSSING); }
              continue ;
            }

          // else - add a trimmed curve to the output
          SmCurve *pCurve = NULL;
          s3dCurves[0]->Copy(crContext, pCurve) ;
          pCurve->Trim(sIvl) ;   // may snap sIvl by tol to existing knots

#ifdef SM_DEBUG_CODE
          if(bDebugMe)
            {
              SM_ASSERT_VALID(s3dCurves[0]) ;
              SM_ASSERT_VALID(pCurve) ;
              sIvl.Dump() ;
            }
#endif // SM_DEBUG_CODE

          // set output
          if(pOpt3DCurves)   { pOpt3DCurves->Add(pCurve) ; }
          if(pOptCurveTypes) { pOptCurveTypes->Add(SM_TC_CROSSING); }

        } // end iter every classification interval

#ifdef SM_DEBUG_CODE
      if (bDebugMe || lCount == lDebugCount)
        {
          sCurveClassification.Dump() ;

          SmFace *pFace1 = (SmFace *)GetFace() ;
          SmFace *pFace2 = (SmFace *)crPlane.GetFace() ;
          SmBrep *pBrep1 = pFace1 ? pFace1->GetBrep() : NULL ;
          SmBrep *pBrep2 = pFace2 ? pFace2->GetBrep() : NULL ;

          smgfx_Erase();
          smgfx_SetLook(1,2, 0,0,1) ; if(pBrep1) { pBrep1->Draw(TRUE) ; } sm_GraphicsLoop() ;
          smgfx_SetLook(1,2, 0,1,0) ; if(pBrep2) { pBrep2->Draw(TRUE) ; } sm_GraphicsLoop() ;
          smgfx_SetLook(1,2, 0,1,1) ; DrawUV(8,8,FALSE,NULL,TRUE) ; sm_GraphicsLoop() ;
          smgfx_SetLook(1,2, 1,1,0) ; crPlane.DrawUV(8,8,FALSE,NULL,TRUE) ; sm_GraphicsLoop() ;
          smgfx_SetLook(1,2, 1,.5,0); sThisPlane.DrawUV(8,8,FALSE,NULL,TRUE) ; sm_GraphicsLoop() ;
          smgfx_SetLook(1,2, 0,0,0) ; if(pFace1) { pFace1->Draw(SM_DM_CROSSHATCH,10,10) ; } sm_GraphicsLoop() ;
          smgfx_SetLook(1,2, 0,0,0) ; if(pFace2) { pFace2->Draw(SM_DM_CROSSHATCH,10,10) ; } sm_GraphicsLoop() ;
          smgfx_SetLook(2,3, 1,0,0) ; for(ii=0;ii<s3dCurves.GetSize();ii++)
                                       { s3dCurves[ii]->Draw() ; sm_GraphicsLoop() ; }
          smgfx_SetLook(4,5, 1,0,0) ; sCurveClassification.Draw(FALSE) ; sm_GraphicsLoop() ;
          sm_GraphicsLoop();
        }
#endif // SM_DEBUG_CODE

      //      // trim line to bounding box of this extrusion surface
      //      //    temporary implementation: todo rewrite section to trim to both surfaces not just the one.
      //      SmCurve *pCurve = s3dCurves[0] ;
      //      if(pCurve->IsKindOf(SmLine_TYPE))
      //        {
      //          SmLine *pLine = (SmLine*)s3dCurves[0] ;
      //          SmExtent1d sLineIvl = pLine->GetSTEPInterval() ;
      //
      //          // trim result back to SweptPlanarSurface's pseudo box
      //          // build swept face pseudo box - cheap from swept gencurve
      //          //SmPseudoBox sPseudoBox ;
      //          //m_pGenCurve->CalculateBoundingBox(m_pGenCurve->GetNaturalInterval(), NULL, &sPseudoBox, NULL) ;
      //          //sPseudoBox.ExpandSweep( m_vExtrusionVec );
      //          // since m_vExtrusionVec seems to always be unit vector we must take extrusion distance into account
      //          //double extrusionDist = m_vAnalUVDomain.GetVInterval().GetMax() - m_vAnalUVDomain.GetVInterval().GetMin();
      //          //sPseudoBox.ExpandSweep( m_vExtrusionVec * extrusionDist ) ;
      //
      //          // get a good box from the face itself
      //          SmExtent3d sPseudoBox;
      //          this->CalculateBoundingBox( this->GetNaturalUVDomain(), &sPseudoBox );
      //
      //          // No need for extending the pseudo box - extended intersections cause problems
      //          //  // avoid tolerance problems and increase the sBBox by a substantial step
      //          //  sPseudoBox.ExpandAbsolute(.01) ;
      //
      //          // intersect surf/surf line with pseudobox
      //          ULONG lNumFound ;
      //          double dTEnter, dTExit ;
      //          sPseudoBox.IntersectLine(pLine->GetLinePoint(),
      //                                   pLine->GetLineScale() * pLine->GetLineVector(),
      //                                   lNumFound,
      //                                   dTEnter, dTExit) ;
      //
      //          //      // get this surfaces bounding box
      //          //      SmExtent3d sBBox ;
      //          //      CalculateBoundingBox(crUVDomain, &sBBox) ;
      //          //
      //          //      // avoid tolerance problems and increase the sBBox by a substantial step
      //          //      sBBox.ExpandAbsolute(.01) ;
      //          //
      //          //      // intersect line with BBox
      //          //      ULONG lNumFound ;
      //          //      double dTEnter, dTExit ;
      //          //      sBBox.IntersectLine(pLine->GetLinePoint(),
      //          //                          pLine->GetLineScale() * pLine->GetLineVector(),
      //          //                          lNumFound,
      //          //                          dTEnter, dTExit) ;
      //
      //          // no work - no intersection
      //          if(lNumFound == 0)
      //            {  return SM_SUCCESS ; }
      //
      //          // Trim Line to bounding box by intersecting the line's interval with the bounding box interval
      //          SmExtent1d sTrimIvl(dTEnter, dTExit) ;
      //          sTrimIvl.Intersect(sLineIvl, sTrimIvl) ;
      //
//      #ifdef SM_DEBUG_CODE
//                if (bDebugMe || lCount == lDebugCount)
//                  {
//                    pLine->Dump() ;
//
//                    SmFace *pFace1 = (SmFace *)GetFace() ;
//                    SmFace *pFace2 = (SmFace *)crPlane.GetFace() ;
//                    SmBrep *pBrep1 = pFace1 ? pFace1->GetBrep() : NULL ;
//                    SmBrep *pBrep2 = pFace2 ? pFace2->GetBrep() : NULL ;
//
//                    smgfx_Erase();
//                    smgfx_SetLook(1,2, 0,0,1) ;    if(pBrep1) { pBrep1->Draw(TRUE) ; } sm_GraphicsLoop() ;
//                    smgfx_SetLook(1,2, 0,1,0) ;    if(pBrep2) { pBrep2->Draw(TRUE) ; } sm_GraphicsLoop() ;
//                    smgfx_SetLook(1,2, 0,1,1) ;    DrawUV() ; sm_GraphicsLoop() ;
//                    smgfx_SetLook(1,2, 1,1,0) ;    crPlane.DrawUV() ; sm_GraphicsLoop() ;
//                    smgfx_SetLook(1,2, 0,0,0) ;    if(pFace1) { pFace1->Draw(SM_DM_CROSSHATCH,10,10) ; } sm_GraphicsLoop() ;
//                    smgfx_SetLook(1,2, 0,0,0) ;    if(pFace2) { pFace2->Draw(SM_DM_CROSSHATCH,10,10) ; } sm_GraphicsLoop() ;
//                    smgfx_SetLook(1,2, .3,.3,.3) ; sPseudoBox.Draw() ; sm_GraphicsLoop() ;
//                    smgfx_SetLook(2,3, 1,0,0) ;    pLine->Draw(&sLineIvl) ; sm_GraphicsLoop() ;
//                    smgfx_SetLook(3,4, 0,1,0) ;    pLine->Draw(&sTrimIvl) ; sm_GraphicsLoop() ;
//                    sm_GraphicsLoop();
//                  }
//      #endif // SM_DEBUG_CODE
        //        // no work - no intersections
        //        if(   sTrimIvl.IsInit()
        //           || sTrimIvl.GetLength() < 0.0)
        //          { return(SM_SUCCESS) ; }
        //
        //        else if(sTrimIvl.GetLength() < SM_EFF_ZERO)
        //          { // todo: might return a degenerate point solution.
        //            // for now - return nothing
        //            return(SM_SUCCESS) ;
        //          }
        //
        //        // adjust the pLine Interval
        //        pLine->AdjustSTEPInterval(sTrimIvl) ;
        //
        //        // set output
        //        sClean1.Clear() ;
        //        if(pOpt3DCurves)   { pOpt3DCurves->Add(s3dCurves[0]) ; }
        //        if(pOptCurveTypes) { pOptCurveTypes->Add(SM_TC_CROSSING); }
        //
        //      } // end need to Trim XSectLine check

    } // end GenCurve is linear or Planar and swept in its own plane
      // and SweptPlanarSurface is not parallel to input Surface branch

  // When GenCurve is planar with an independent sweep direction
  // and crPlane and plane containing GenCurve are parallel
  // Then,
  //   XSect = translated copy of sweptCurve
  else if(   (   m_eOrientation == SM_CO_PERPENDICULAR
              || m_eOrientation == SM_CO_INDEPENDENT)
          && m_vCurveVec.IsParallelTo(sPlaneNormal, dAngTol))
    {
      // get param along Extrusion Vector where InputPlane intersects ExtrusionVector
      double dDistCenterBaseToPlane = (rPlaneOrigin - m_vOrigin).Dot(m_vCurveVec) ;
      double dAngleRad = 0.0;
      m_vCurveVec.AngleBetween(m_vExtrusionVec, dAngleRad) ;
      double dDistAlongExtrusion    = dDistCenterBaseToPlane * smos_Cosine(dAngleRad) ;
      double dParamAlongExtrusion   = dDistAlongExtrusion / m_vExtrusionVec.Length() ;

      // get extrusionSurface's sweep interval
      SmExtent1d sOrigIvl(m_vAnalUVDomain.GetVInterval());

#ifdef SM_DEBUG_CODE // draw input surfaces and their breps and faces if possible
      if (bDebugMe || lCount == lDebugCount)
        {
          SmFace *pFace1 = (SmFace *)GetFace() ;
          SmFace *pFace2 = (SmFace *)crPlane.GetFace() ;
          SmBrep *pBrep1 = pFace1 ? pFace1->GetBrep() : NULL ;
          SmBrep *pBrep2 = pFace2 ? pFace2->GetBrep() : NULL ;

          smgfx_Erase();
          smgfx_SetLook(2,3, 1,0,0) ; rPlaneOrigin.DrawPointToPoint(m_vOrigin) ; sm_GraphicsLoop() ;
          smgfx_SetLook(3,4, 0,0,1) ; m_vCurveVec.Draw(&m_vOrigin) ; sm_GraphicsLoop() ;
          smgfx_SetLook(3,4, 0,1,0) ; m_vExtrusionVec.Draw(&m_vOrigin) ; sm_GraphicsLoop() ;
          smgfx_SetLook(2,3, 0,0,0) ; m_pGenCurve->Draw(); sm_GraphicsLoop();
          smgfx_SetLook(1,2, 0,0,1) ; if(pBrep1) { pBrep1->Draw(TRUE) ; } sm_GraphicsLoop() ;
          smgfx_SetLook(1,2, 0,1,0) ; if(pBrep2) { pBrep2->Draw(TRUE) ; } sm_GraphicsLoop() ;
          smgfx_SetLook(1,2, 0,1,1) ; DrawUV(3,3) ; sm_GraphicsLoop() ;
          smgfx_SetLook(1,2, 1,1,0) ; crPlane.DrawUV() ; sm_GraphicsLoop() ;
          smgfx_SetLook(1,2, 0,0,0) ; if(pFace1) { pFace1->Draw(SM_DM_CROSSHATCH,10,10) ; } sm_GraphicsLoop() ;
          smgfx_SetLook(1,2, 0,0,0) ; if(pFace2) { pFace2->Draw(SM_DM_CROSSHATCH,10,10) ; } sm_GraphicsLoop() ;
          sm_GraphicsLoop();
        }
#endif // SM_DEBUG_CODE

      // snap nearby distanceValues to extrusion begin position
      if (smos_Fabs(dParamAlongExtrusion-sOrigIvl.GetMin()) < dTol)
        {
          dParamAlongExtrusion = sOrigIvl.GetMin();
        }
      // snap nearby distanceValues to extrusion end position
      else if (smos_Fabs(dParamAlongExtrusion-sOrigIvl.GetMax()) < dTol)
        {
          dParamAlongExtrusion = sOrigIvl.GetMax();
        }
      // skip planes outside of the extrusion domain
      else if (!sOrigIvl.ContainsValue(dParamAlongExtrusion))
        {
          return SM_SUCCESS;  // No intersections
        }

      // intersectionCurve = sweptCurveCopy moved to target plane along extrusion vector
      // It works better to project to the plane, along our directrix,
      // instead of just translating.  That way we're sure it's truly in
      // both the plane and our surface, in case of tiny differences
      // between the directrix and the surface normal, e.g.
      // [ BD 070310 ]
      SmCurve *pProjCurve = NULL;
      m_pGenCurve->CreatePlaneProjection( crContext,
                                          SM_PT_PARALLEL,
                                          rPlaneOrigin,
                                          sPlaneNormal,
                                          m_vExtrusionVec,
                                          pProjCurve );
      SmObjDelete sCleanCrv( pProjCurve );

#ifdef SM_DEBUG_CODE
       // draw Breps(blue,green), Surfaces(cyan,yellow)
       // draw XSectCurve(red), OriginalSweptCurve(black)
       if (bDebugMe)
         {
           SmBrep *pBrep1 = GetFace()         ? ((SmFace *)GetFace())->GetBrep() : NULL ;
           SmBrep *pBrep2 = crPlane.GetFace() ? ((SmFace *)crPlane.GetFace())->GetBrep() : NULL ;

           smgfx_Erase();
           smgfx_SetLook(1,1, 0,0,1) ; if(pBrep1) { pBrep1->Draw(TRUE) ; } sm_GraphicsLoop() ;
           smgfx_SetLook(1,1, 0,1,0) ; if(pBrep2) { pBrep2->Draw(TRUE) ; } sm_GraphicsLoop() ;
           smgfx_SetLook(1,2, 0,1,1) ; DrawUV(3,3); sm_GraphicsLoop();
           smgfx_SetLook(1,2, 1,1,0) ; crPlane.DrawUV(3,3); sm_GraphicsLoop();
           smgfx_SetLook(2,3, 0,0,0) ; m_pGenCurve->Draw(); sm_GraphicsLoop();
           smgfx_SetLook(4,5, 1,0,0) ; pProjCurve->Draw(); sm_GraphicsLoop();
           sm_GraphicsLoop();
         }
#endif // SM_DEBUG_CODE

      // get plane and intersection curve bounding boxes
      SmPseudoBox sCurvePBox, sPlanePBox ;
      crPlane.CalculateBoundingBox(crPlane.GetNaturalUVDomain(),
                                   NULL, &sPlanePBox, NULL) ;
      pProjCurve->CalculateBoundingBox(pProjCurve->GetNaturalInterval(),
                                       NULL, &sCurvePBox, NULL) ;

      // only use curves intersecting plane domain
      sPlanePBox.ExpandAbsolute(dTol) ;
      if(!sPlanePBox.AreDisjoint(sCurvePBox))
        {
          // trim curves to plane domain
          SmTArray<SmCurve *> s3dCurves ;
          SmObjsDelete<SmCurve *> sCleanArray(&s3dCurves) ;
          crPlane.TrimCurveToPlaneDomain(crOtherUVDomain, *pProjCurve, dTol, s3dCurves) ;

          // add trim curves to output
          if (pOpt3DCurves) { sCleanArray.Clear();
                              pOpt3DCurves->Append(s3dCurves);
                            }
        } // end intersecting curve/TrimmedPlane check

    } // end XSect = translated copy of sweptCurve branch

  // when extrusion vec is perpendicular to plane normal
  // - xsect is one or more lines each line generated by sweeping
  //   a plane/GenCurve intersection point.
  else if (m_vExtrusionVec.IsPerpendicularTo(sPlaneNormal,dAngTol))
    {
      // get sweptCurve/plane intersection point
      SmSolutionArray sSolutions;
      double dD = -rPlaneOrigin.Dot(sPlaneNormal);
      SER(m_pGenCurve->GlobalPropertyAnalysis(m_pGenCurve->GetNaturalInterval(),
                                              SM_CP_PLANE_INTERSECTION,
                                              &dD, &sPlaneNormal,
                                              dTol, sSolutions));

#ifdef SM_DEBUG_CODE
      if (bDebugMe || lCount == lDebugCount)
        {
          SmFace *pFace1 = (SmFace *)GetFace() ;
          SmFace *pFace2 = (SmFace *)crPlane.GetFace() ;
          SmBrep *pBrep1 = pFace1 ? pFace1->GetBrep() : NULL ;
          SmBrep *pBrep2 = pFace2 ? pFace2->GetBrep() : NULL ;

          smgfx_Erase();
          smgfx_SetLook(3,4, 1,0,0) ; sSolutions.Draw() ; sm_GraphicsLoop() ;
          smgfx_SetLook(1,2, 0,0,1) ; if(pBrep1) { pBrep1->Draw(TRUE) ; } sm_GraphicsLoop() ;
          smgfx_SetLook(1,2, 0,1,0) ; if(pBrep2) { pBrep2->Draw(TRUE) ; } sm_GraphicsLoop() ;
          smgfx_SetLook(1,2, 0,1,1) ; DrawUV() ; sm_GraphicsLoop() ;
          smgfx_SetLook(1,2, 1,1,0) ; crPlane.DrawUV() ; sm_GraphicsLoop() ;
          sm_GraphicsLoop();
        }
#endif // SM_DEBUG_CODE

      // extrusion surf locals
      // SmExtent1d sLinearExt = GetSTEPLinearParamExtent();
      // double     dLineScale = dExtrusionLength/sLinearExt.GetLength();

      // for every SweptCurve/Plane intersection
      for(ii=0;ii<sSolutions.GetSize();ii++)
        {
          SmSolution & rSol = sSolutions[ii];
          if (rSol.m_eSolutionType == SM_ST_RANGE_OF_VALUES)
            {
              // Some Surf/Surf coincidence - handle it at a higher level
              rbNeedsMoreIntersections = TRUE;
              return SM_SUCCESS;
            }
        } // end search for RangeOfValues solutions

      // arrive here when all solutions are PointSolutions
      SmBoolean bIsClosed = m_pGenCurve->IsClosed(sGenCurveNurbIvl) ;

      // for every SweptCurve/Plane point intersection - build, trim and if nonNULL, output a line
      SmBoolean bTangent, bLastTangent= FALSE ;
      SmPoint3d sCrvPt[2], sLastCrvPt, sFirstCrvPt ;
      for(ii=0;ii<sSolutions.GetSize();ii++)
        {
          // create line segment intersection curve
          SmBSplineCurve * pCrv = NULL;
          m_pGenCurve->Evaluate(sSolutions[ii].m_vStart[0], 1, TRUE, sCrvPt);
          CreateDirectrixFromPoint(crContext,sCrvPt[0],SM_EFF_ZERO_SQRT,pCrv);
          SmObjDelete sCleanCrv(pCrv);

#ifdef SM_DEBUG_CODE
          // draw Breps(blue,green), Surfaces(cyan,yellow)
          // draw XSectPoint(red), XSectCurve(red), OriginalSweptCurve(black)
          if (bDebugMe)
            {
              SmBrep *pBrep1 = GetFace()         ? ((SmFace *)GetFace())->GetBrep() : NULL ;
              SmBrep *pBrep2 = crPlane.GetFace() ? ((SmFace *)crPlane.GetFace())->GetBrep() : NULL ;

              smgfx_Erase();
              smgfx_SetLook(1,1, 0,0,1) ; if(pBrep1) { pBrep1->Draw(TRUE) ; } sm_GraphicsLoop() ;
              smgfx_SetLook(1,1, 0,1,0) ; if(pBrep2) { pBrep2->Draw(TRUE) ; } sm_GraphicsLoop() ;
              smgfx_SetLook(1,2, 0,1,1) ; DrawUV(5,5); sm_GraphicsLoop();
              smgfx_SetLook(1,2, 1,1,0) ; crPlane.DrawUV(6,6); sm_GraphicsLoop();
              smgfx_SetLook(1,4, 1,0,0) ; sCrvPt[0].Draw(); sm_GraphicsLoop();
              smgfx_SetLook(2,3, 1,0,0) ; sCrvPt[1].Draw(&sCrvPt[0]) ; sm_GraphicsLoop() ;
              smgfx_SetLook(3,4, 1,0,0) ; pCrv->Draw(); sm_GraphicsLoop();
              smgfx_SetLook(2,3, 0,0,0) ; m_pGenCurve->Draw(); sm_GraphicsLoop();
              sm_GraphicsLoop();
            }
#endif // SM_DEBUG_CODE
          // when an intersection curve was found
          if(pCrv)
            {
              // trim line control Points to plane
              gw_CURVE *pNurb = pCrv->GetOrCreateGwNurbPointer() ;
              SmBoolean bFoundInterval ;
              SmPoint3d sStartPt, sEndPt ;
              TO_EUCLID(pNurb->pol->Pw[0], sStartPt) ;
              TO_EUCLID(pNurb->pol->Pw[1], sEndPt) ;
              crPlane.TrimPlaneLineToPlaneDomain(crPlane.GetNaturalUVDomain(),
                                                 sStartPt, sEndPt, dTol,
                                                 sStartPt, sEndPt, bFoundInterval) ;

              // only use curves that intersect plane domain
              if(bFoundInterval)
                {
                  // get trimmed xsectCurve length
                  double dLength = sStartPt.DistanceBetween(sEndPt) ;

                  // turn short curves into degenerate ones by snapping endPoints
                  if(dLength < dTol)
                    { // make a degenerate line
                      sStartPt = (sStartPt + sEndPt) / 2.0 ;
                      sEndPt   = sStartPt ;
                    }

                  // Trim the curve definition - move control points
                  SM_ASSERT(pCrv->GetType() == SmBSplineCurve_TYPE) ;
                  COPY_XYZ(sStartPt, pNurb->pol->Pw[0]) ;
                  COPY_XYZ(sEndPt  , pNurb->pol->Pw[1]) ;

                  // classify the solution - crossing or tangent
                  double dCrvSpeed = sCrvPt[1].Length() ;
                  double dScaledZero = SM_EFF_ZERO * (1.0 + smos_Max(dCrvSpeed,
                                                                     sCrvPt[0].GetMaxDimension())) ;
                  SM_ASSERT(dCrvSpeed > dScaledZero) ;
                  double    dAngleTol  = SM_DEG2RAD(2.0) ;
                  bTangent =  (   (dCrvSpeed > dScaledZero)
                               && (sCrvPt[1]/dCrvSpeed).IsPerpendicularTo(sPlaneNormal,dAngleTol))
                             ? TRUE
                             : FALSE ;

                  // cheesy heuristic - the real solution is to modify GlobalPropertyAnalysis.
                  // GlobalPropertyAnalysis returns multiple close hits for a single tangent intersection
                  // and intersections on closed seam boundaries
                  // The solutions are sorted by param value - so just save the best one.
                  if(ii > 0) // there is a last solution
                    {
                      // remove multiple solutions due to tangent points
                      if(   bTangent                                        // this solution is tangent
                         && bLastTangent                                    // last solution is tangent
                         && sLastCrvPt.DistanceBetween(sCrvPt[0]) < dTol)   // this and the last solution points are close
                        {
                          // only save the best solution
                          if(sSolutions[ii].m_vStart.m_dSolutionValue < sSolutions[ii-1].m_vStart.m_dSolutionValue)
                            {
                              // replace the last saved solution with this solution
                              if ( pOpt3DCurves != NULL )
                                {
                                  SmCurve *pLastCurve = pOpt3DCurves->GetLast() ;
                                  pOpt3DCurves->RemoveLast() ;
                                  SM_ASSERT(pLastCurve != NULL) ; delete pLastCurve ; pLastCurve = NULL ;
                                }
                              if ( pOptCurveTypes != NULL )
                                { pOptCurveTypes->RemoveLast(); }
                            }
                          else
                            { // don't save this solution
                              continue ;
                            }
                         } // end multiple tangent solutions check

                       // remove multiple solutions due to close curve start/end hits
                       // Check vs. the first curve found -- that would be the other end of the seam. [210505]
                       //cbi But shouldn't we return both sides of a seam?
                       //cbi Well, the general SSI doesn't, so we'll stick with this.
                       else if(   bIsClosed
                               && sFirstCrvPt.DistanceBetween(sCrvPt[0]) < dTol)   // this and the first solution points are close
                        {
                          // Save only the better solution.
                          if(sSolutions[ii].m_vStart.m_dSolutionValue < sSolutions[0].m_vStart.m_dSolutionValue)
                            {
                              // replace the first saved solution with this solution
                              if ( pOpt3DCurves != NULL )
                                {
                                  SmCurve *pFirstCurve = pOpt3DCurves->GetAt(0) ;
                                  pOpt3DCurves->RemoveAt(0) ;
                                  SM_ASSERT(pFirstCurve != NULL) ; delete pFirstCurve ; pFirstCurve = NULL ;
                                }
                              if ( pOptCurveTypes != NULL )
                                { pOptCurveTypes->RemoveAt(0); }
                            }
                          else
                            { // don't save this solution
                              continue ;
                            }
                        }
                    } // end cheesy heuristic check to eliminate multiple hits on a tangent point

                  // prepare for next iteration
                  bLastTangent = bTangent ;
                  sLastCrvPt   = sCrvPt[0] ;
                  if(ii == 0) { sFirstCrvPt = sCrvPt[0] ; }

                  // add curve to ouput
                  if(pOpt3DCurves && pCrv)
                    {
                      sCleanCrv.Clear();
                      if(pOpt3DCurves  ) { pOpt3DCurves->Add(pCrv); }
                      if(pOptCurveTypes) { if(pCrv->IsDegenerate())
                                              { pOptCurveTypes->Add(SM_TC_TOUCHING); }
                                            else if(bTangent)
                                              { pOptCurveTypes->Add(SM_TC_TANGENT); }
                                            else
                                              { pOptCurveTypes->Add(SM_TC_CROSSING); }
                                          }
                    } // end save output check
                } // end found a trimmmed xSectCurve check
            } // end pCrv existence check
        } // end iter every sweptCurve/plane point intersection
    } // end extrusion vec and plane normal are perpendicular branch
  else // this is not a special case intersection
    {
      //  plane cuts through SurfOfExtrusion at an angle,
      rbNeedsMoreIntersections = TRUE;
      return SM_SUCCESS;
    }

  // set output
  ULONG lCnt = pOpt3DCurves->GetSize() ;
  for(ii=0; ii<lCnt; ii++)
    {
      if (pOptSurface1UVCurves)                 { pOptSurface1UVCurves->Add(NULL); }
      if (pOptSurface2UVCurves)                 { pOptSurface2UVCurves->Add(NULL); }
      if (   pOptCurveTypes                     // Line solutions have already added pOptCurveTypes
          && lCnt != pOptCurveTypes->GetSize()) { SmCurve *pCrv = (*pOpt3DCurves)[ii] ;
                                                  if(pCrv->IsDegenerate())
                                                     { pOptCurveTypes->Add(SM_TC_TOUCHING); }
                                                   else
                                                     { pOptCurveTypes->Add(SM_TC_CROSSING); }
                                                }
      if (pOptDeviations)                       { pOptDeviations->Add(0.0); }
    }

  SM_ASSERT(   pOptCurveTypes == NULL
            || pOpt3DCurves   == NULL
            || pOpt3DCurves->GetSize() == pOptCurveTypes->GetSize()) ;

#ifdef SM_DEBUG_CODE // draw xSect Curve and the input surfaces and their breps and faces if possible
  if (bDebugMe || lCount == lDebugCount)
    {
      SmFace *pFace1 = (SmFace *)GetFace() ;
      SmFace *pFace2 = (SmFace *)crPlane.GetFace() ;
      SmBrep *pBrep1 = pFace1 ? pFace1->GetBrep() : NULL ;
      SmBrep *pBrep2 = pFace2 ? pFace2->GetBrep() : NULL ;

      smgfx_Erase();
      smgfx_SetLook(2,3, 1,0,0) ; for(ii=0;ii<pOpt3DCurves->GetSize();ii++)
                                    { pOpt3DCurves->GetAt(ii)->Draw() ; sm_GraphicsLoop() ; }
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep1) { pBrep1->Draw(TRUE) ; } sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,0) ; if(pBrep2) { pBrep2->Draw(TRUE) ; } sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,1) ; DrawUV() ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 1,1,0) ; crPlane.DrawUV() ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,0,0) ; if(pFace1) { pFace1->Draw(SM_DM_CROSSHATCH,10,10) ; } sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,0,0) ; if(pFace2) { pFace2->Draw(SM_DM_CROSSHATCH,10,10) ; } sm_GraphicsLoop() ;
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE
  // all done
  return SM_SUCCESS;

} // end SmSurfOfExtrusion::IntersectWithPlane

//      //-------------------------------------------------------------------------
//      // This function is given 2 line segments. It finds out if they are parallel
//      // or not, and if yes, do they intersect or not.
//      // If the axes are not parallel, the geometric method should give up.
//      //
//      // Otherwise we accept the geometric method's result:
//      // If the axes are not collinear, one segment is projected onto the other.
//      // If the axis segments avoid each other: disjoint surfaces,
//      // if they intersect, intersection curves are to be created.
//      // If they just touch each other, the for now the function bails out,
//      // because I don't know how to return single point results
//      // (degenerate lines are not created).
//      //-------------------------------------------------------------------------
//
//      /*******************************************************************//**
//      PURPOSE: Used to figure out how much overlap there ban be between
//          an SM_CO_PERPENDICULAR extrusion surface and any other kind
//          of planarGenCurve surface swept along the same axis
//          like another SM_CO_PERPENDICULAR extrusion surface
//          or a cylindrical surface.
//
//      NOTES:
//          The calling function has to check that this surface is type SM_CO_PERPENDICULAR
//          and the other surface is properly oriented, e.g. its
//          GenCurve (or its equivalent) lies in a plane perpendicular to the
//          sweep axis and the other surface direction is parallel to this
//          surfaces extrusion vector.
//
//          When axes are not parallel, set rbNeedsMoreIntersections = TRUE.
//          When axes are not colinear, project OtherAxis onto thisSurface axis,
//          When axes are disjoint,     set nIntersectionsArg = 0
//          When
//      ***********************************************************************/
//      static SmStatus sm_AxisSegmentsIntersect
//        (const SmSurfOfExtrusion  * pExtSurf,                 // in : target surface
//         const SmVector3d         & crOZAxis,                 // in : OtherSurface sweep segment direction
//         const SmPoint3d          & crOtherStartPoint,        // in : OtherSurface Sweep segment StartPoint
//         const SmPoint3d          & crOtherEndPoint,          // in : OtherSurface Sweep segment EndPoint
//         SmExtent1d               & rXSectIvl,                // out: Init = no intersection else range of overlap
//                                                              //      on line m_vOrigin * param * sExtrusionVec
//         SmBoolean                & rbNeedsMoreIntersections) // out: TRUE = ExtrusionVec not parallel with OtherZAxis
//                                                              //      FALSE=
//      {
//        // init output
//        lXSectCount              = 0;
//        rbNeedsMoreIntersections = FALSE;
//
//        // select tolerance
//        double dAngularTolerance = SM_EFF_ZERO_SQRT;
//
//        // Create ThisSurface axisLine:
//        SmPoint3d  sOrigin       = pExtSurf->GetOrigin();
//        SmVector3d sExtrusionVec = pExtSurf->GetExtrusionVector();
//
//        // no work - axes are not parallel use general purpose surf/surf intersector
//        if(!sExtrusionVec.IsParallelTo(crOZAxis, dAngularTolerance))
//          {
//            rbNeedsMoreIntersections = TRUE;
//            return SM_SUCCESS;
//          }
//
//        // Project OtherSurface Start Point to Extrusion Sweep Segment
//        double dShiftedStartParam;
//        SER(smgu_LineClosestPoint(sOrigin, sExtrusionVec,
//                                  crOtherStartPoint, //  crTestPoint,
//                                  dShiftedStartParam));
//
//        // Project OtherSurface Start Point to Extrusion Sweep Segment
//        double dShiftedEndParam;
//        SER(smgu_LineClosestPoint(sOrigin, sExtrusionVec,
//                                  crOtherEndPoint,   //  crTestPoint,
//                                  dShiftedEndParam));
//
//        // intersect two segments to find overlap
//        SmExtent1d sThisIvl = pExtSurf->GetSTEPUVDomain()->GetVInterval() ;
//        SmExtent1d sOtherIvl(dShiftedStartParam, dShiftedEndParam) ;
//        sThisIvl.Intersect(sOtherIvl, sXSectIvl) ;
//
//        // all done
//        return SM_SUCCESS;
//
//      } // end sm_AxisSegmentsIntersect

/*******************************************************************//**
PURPOSE: Find intersection curves between two surfaces that
  can be represented as GenCurves swept along a common ExtrusionVec direction.

  Only call this function after verifying that the two surfaces have
  a parallel extrusion vec.

NOTES:

METHOD ---
  Finds the projected intersection of the two generator curves
  using the pExtSurf->m_VExtrusionVec as the projection direction.
  For every solution:

  point intersections: computes overlap between two surfaces and builds
                      1. a line segment for overlapping cases
                      2. a degenerate point curve for touching cases
                      3. no curve at all for non-overlapping cases

  range intersections: computes overlap between two surfaces and builds
                      1. 2 lines and 2 GenCurve segments outlining the
                            coincident region for overlapping cases
                      2. a single GenCurve segment for touching cases
                      3. no curves at all for non-overlapping cases
***********************************************************************/
static SmStatus sm_IntersectGenerators
  (const SmSurfOfExtrusion    & rExtrusionSurf,          // in : extrusion surface being intersected
   double                       dTol,                    // in : min distance between distinct points
   SmCurve                    & rOtherGenCurve,          // in : OtherSurface Generator Curve
   SmVector3d                 & rOtherExtrusionVec,      // in : should be parallel to ExtrusionSurf->ExtrusionVec but may be different length
   SmExtent1d                 & rOtherIvl,               // in : range swept by every point on OtherGenCurve specifying a segment on the line
                                                         //      OtherGenCurvePoint + param * OtherExtrusionVec, where param is in OtherIvl range.
   SmTArray<SmCurve*>         * pOpt3DCurves,            // out: 1 intersection curve built for each curve/curve intersection point
   SmTArray<SmTsectCurveType> * pOptCurveTypes,          // out: associated type of intersection
   SmBoolean                  & rbNeedsMoreIntersections)// out: TRUE = situation too complicated - ask general intersector to solve
                                                         //      FALSE= surfaces were intersected completely.
                                                         //             Even if surfaces don't intersect - we're done.
{
  // locals
  ULONG ii, jj, kk ;
  SmBSplineCurve     * pExtrusionGenCurve = NULL;
  SmVector3d           sExtrusionVec ;
  SmExtent1d           sExtrusionIvl         = rExtrusionSurf.GetSTEPUVDomain().GetVInterval() ;
  const SmContext    * pContext              = rExtrusionSurf.GetContext();
  ((SmSurfOfExtrusion &)rExtrusionSurf).GetCanonical(pExtrusionGenCurve, sExtrusionVec) ;
  SmExtent1d           sExtrusionGenCurveIvl = pExtrusionGenCurve->GetNaturalInterval() ;
  double               dExtrusionLength      = sExtrusionVec.Length() ;
  double               dOtherLength          = rOtherExtrusionVec.Length() ;

  // check parallel extrusion vectors
  if(!rOtherExtrusionVec.IsParallelTo(sExtrusionVec, SM_EFF_ZERO_RAD))
    {
      rbNeedsMoreIntersections = TRUE ;
      return SM_SUCCESS ;
    }

#ifdef SM_DEBUG_CODE // draw input surfaces and their breps and faces if possible
SmBoolean bDebugMe = FALSE;
  if (bDebugMe)
    {
      SM_ASSERT_VALID(&rExtrusionSurf) ;
      SM_ASSERT_VALID(&rOtherGenCurve) ;
      rExtrusionSurf.Dump() ;
      rOtherGenCurve.Dump() ;

      SmFace *pFace1 = (SmFace *)rExtrusionSurf.GetFace() ;
      SmBrep *pBrep1 = pFace1 ? pFace1->GetBrep() : NULL ;
      SmExtent1d sOtherNaturalIvl = rOtherGenCurve.GetNaturalInterval() ;
      SmPoint3d sOtherDrawPoint[3][2] ;
      SmPoint3d sThisDrawPoint[3][2] ;
      for(ii=0;ii<3;ii++) { rOtherGenCurve.EvaluatePoint(sOtherNaturalIvl.Evaluate(((double)ii)/2.0), sOtherDrawPoint[ii][1]) ;
                            sOtherDrawPoint[ii][0] = sOtherDrawPoint[ii][1] + rOtherIvl.GetMin() * rOtherExtrusionVec ;
                            sOtherDrawPoint[ii][1] = sOtherDrawPoint[ii][1] + rOtherIvl.GetMax() * rOtherExtrusionVec ;
                          }
      for(ii=0;ii<3;ii++) { pExtrusionGenCurve->EvaluatePoint(sExtrusionGenCurveIvl.Evaluate(((double)ii)/2.0), sThisDrawPoint[ii][1]) ;
                            sThisDrawPoint[ii][0] = sThisDrawPoint[ii][1] + sExtrusionIvl.GetMin() * sExtrusionVec ;
                            sThisDrawPoint[ii][1] = sThisDrawPoint[ii][1] + sExtrusionIvl.GetMax() * sExtrusionVec ;
                          }
      smgfx_Erase();
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep1) { pBrep1->Draw(TRUE) ; } sm_GraphicsLoop() ;
      smgfx_SetLook(2,3, 1,.5,0); rOtherGenCurve.Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook(2,3, 1,.5,0); for(ii=0;ii<3;ii++) { sOtherDrawPoint[ii][0].DrawPointToPoint(sOtherDrawPoint[ii][1]) ; sm_GraphicsLoop() ; }
      smgfx_SetLook(2,3, 0,.5,1); pExtrusionGenCurve->Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook(2,3, 0,.5,1); for(ii=0;ii<3;ii++) { sThisDrawPoint[ii][0].DrawPointToPoint(sThisDrawPoint[ii][1]) ; sm_GraphicsLoop() ; }
      smgfx_SetLook(1,2, 0,1,1) ; rExtrusionSurf.DrawUV() ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,0,0) ; if(pFace1) { pFace1->Draw(SM_DM_CROSSHATCH,10,10) ; } sm_GraphicsLoop() ;
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

  // locals for range solutions, when GenCurve Segments need to be copied into the output,
  // remember which genCurve defines the top and bottom of overlap at the endPoints of the range solution.
  SmCurve *pOffsetCurve [2][2] ;   // indices used for:[beg/end range solution][bot/top XSectIvl]
  double   dOffsetLength[2][2] ;   // indices used for:[beg/end range solution][bot/top XSectIvl]


  // init output
  if(pOpt3DCurves)   { pOpt3DCurves->ReSet() ; }
  if(pOptCurveTypes) { pOptCurveTypes->ReSet() ; }
  rbNeedsMoreIntersections = FALSE ;

  // Intersect 2 genCurves along a projection of the of sExtrusionVec.
  // This is just like the intersections needed for hidden line removal
  SmVector3d sUnitExtrusionVec = sExtrusionVec ;
  sUnitExtrusionVec.Unitize() ;
  SmSolutionArray sSolutions;
  SER(pExtrusionGenCurve->GlobalCurveSolve(sExtrusionGenCurveIvl,
                                           rOtherGenCurve,
                                           rOtherGenCurve.GetNaturalInterval(),
                                           SM_SO_PROJECTED_INTERSECT,
                                           dTol,
                                           NULL,
                                           &sUnitExtrusionVec,
                                           SM_SR_ALL,
                                           sSolutions));

#ifdef SM_DEBUG_CODE // draw inputs and solutions
  if (bDebugMe)
    {
      sSolutions.Dump() ;

      SmFace *pFace1 = (SmFace *)rExtrusionSurf.GetFace() ;
      SmBrep *pBrep1 = pFace1 ? pFace1->GetBrep() : NULL ;
      SmExtent1d sOtherNaturalIvl = rOtherGenCurve.GetNaturalInterval() ;
      SmPoint3d sOtherDrawPoint[3][2] ;
      for(ii=0;ii<3;ii++) { rOtherGenCurve.EvaluatePoint(sOtherNaturalIvl.Evaluate(((double)ii)/2.0), sOtherDrawPoint[ii][1]) ;
                            sOtherDrawPoint[ii][0] = sOtherDrawPoint[ii][1] + rOtherIvl.GetMin() * rOtherExtrusionVec ;
                            sOtherDrawPoint[ii][1] = sOtherDrawPoint[ii][1] + rOtherIvl.GetMax() * rOtherExtrusionVec ;
                          }
      smgfx_Erase();
      smgfx_SetLook(3,7, 1,0,0) ; sSolutions.Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep1) { pBrep1->Draw(TRUE) ; } sm_GraphicsLoop() ;
      smgfx_SetLook(2,3, 1,.5,0); rOtherGenCurve.Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook(2,3, 1,.5,0); for(ii=0;ii<3;ii++) { sOtherDrawPoint[ii][0].DrawPointToPoint(sOtherDrawPoint[ii][1]) ; sm_GraphicsLoop() ; }
      smgfx_SetLook(1,2, 0,1,1) ; rExtrusionSurf.DrawUV() ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,0,0) ; if(pFace1) { pFace1->Draw(SM_DM_CROSSHATCH,10,10) ; } sm_GraphicsLoop() ;
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

  // all done - no solutions
  if(sSolutions.GetSize() == 0)
    { return SM_SUCCESS ; }

  // map OtherIvl range from  line=OtherGenCurvePoint + param * OtherExtrusionVec
  // to  sOtherLineIvl range  line=OtherGenCurvePoint + param * ThisExtrusionVec
  SmBoolean  bSameExtrusionDir = rOtherExtrusionVec.Dot(sExtrusionVec) > 0 ;
  SmExtent1d sOtherLineIvl =  (bSameExtrusionDir)
                             ? SmExtent1d(rOtherIvl.GetMin()* dOtherLength/dExtrusionLength,
                                          rOtherIvl.GetMax()* dOtherLength/dExtrusionLength)
                             : SmExtent1d(rOtherIvl.GetMax()*-dOtherLength/dExtrusionLength,
                                          rOtherIvl.GetMin()*-dOtherLength/dExtrusionLength) ;

#ifdef SM_DEBUG_CODE // check that Ivl mapping is correct
  {
    SM_ASSERT(   SM_IS_ZERO((  sOtherLineIvl.GetMin()*sExtrusionVec
                             - rOtherIvl.GetMin()    *rOtherExtrusionVec).Length())
              || SM_IS_ZERO((  sOtherLineIvl.GetMin()*sExtrusionVec
                             - rOtherIvl.GetMax()    *rOtherExtrusionVec).Length())) ;
    SM_ASSERT(   SM_IS_ZERO((  sOtherLineIvl.GetMax()*sExtrusionVec
                             - rOtherIvl.GetMin()    *rOtherExtrusionVec).Length())
              || SM_IS_ZERO((  sOtherLineIvl.GetMax()*sExtrusionVec
                             - rOtherIvl.GetMax()    *rOtherExtrusionVec).Length())) ;
  }
#endif // SM_DEBUG_CODE

  // locals for turning each solution into output Curves
  ULONG lNewCurveCount = 0 ;
  SmPoint3d        sExtrusionPoint[2], sOtherPoint[2];
  double           dExtrusionEnd, dOtherEnd, dLineParam ;
  SmBSplineCurve  *pLineOrDegenerate = NULL ;
  SmTsectCurveType eTsectType;

  // for every curve/curve intersection solution
  for (ii=0; ii<sSolutions.GetSize(); ii++)
    {
      SmSolution &rSol = sSolutions[ii] ;
      lNewCurveCount   = 0 ;

      // once for point solutions and twice for range solutions - turn end points into XSectLines
      for(jj=0;jj<1 || (jj<2 && rSol.m_eSolutionType == SM_ST_RANGE_OF_VALUES);jj++)
        {
          dExtrusionEnd       = jj==0 ? rSol.m_vStart[0] : rSol.m_vEnd[0] ;
          dOtherEnd           = jj==0 ? rSol.m_vStart[1] : rSol.m_vEnd[1] ;

          // Get points on each curve that fall on the same sweep vector
          SER(pExtrusionGenCurve->EvaluatePoint(dExtrusionEnd, sExtrusionPoint[jj]));
          SER(rOtherGenCurve.EvaluatePoint     (dOtherEnd,     sOtherPoint[jj]));

          // find the parameter of the sOtherPoint on line = sExtrusionPoint + dLineParam * sExtrusionVec
          smgu_LineClosestPoint(sExtrusionPoint[jj], sExtrusionVec, sOtherPoint[jj], dLineParam) ;

          // these two points should be on the same line to tolerance
#ifdef SM_DEBUG_CODE
          double dDist = ((sExtrusionPoint[jj] + dLineParam * sExtrusionVec) - sOtherPoint[jj]).Length() ;
          SM_ASSERT(SM_IS_ZERO_TO_TOL(dDist, dTol)) ;

#endif // SM_DEBUG_CODE

          // map sOtherLineIvl range from line=OtherGenCurvePoint     + param * ThisExtrusionVec
          // to sOthermappedIvl range     line=ExtrusionGenCurvePoint + param * ThisExtrusionVec
          SmExtent1d sOthermappedIvl(sOtherLineIvl.GetMin() + dLineParam,
                                     sOtherLineIvl.GetMax() + dLineParam) ;

          // intersect the MappedIvl with the ExtrusionIvl to get overlapIvl of Extrusion vecs
          SmExtent1d sXSectIvl ;
          sExtrusionIvl.Intersect(sOthermappedIvl, sXSectIvl) ;

          // no overlap - no output curve
          if(sXSectIvl.IsInit())
            { continue ; }

          // when working on a range solution - remember which GenCurve defines the top and bottom of the XSectIvl
          if(rSol.m_eSolutionType == SM_ST_RANGE_OF_VALUES)
            {
              // remember which translation of which genCurve defines the bottom of the XSectIvl.
              if     (sXSectIvl.GetMin() == sExtrusionIvl.GetMin())   { pOffsetCurve[jj][0]  = pExtrusionGenCurve ;
                                                                        dOffsetLength[jj][0] = sExtrusionIvl.GetMin() ;
                                                                      }
              else if(sXSectIvl.GetMin() == sExtrusionIvl.GetMax())   { pOffsetCurve[jj][0]  = pExtrusionGenCurve ;
                                                                        dOffsetLength[jj][0] = sExtrusionIvl.GetMax() ;
                                                                      }
              else if(sXSectIvl.GetMin() == sOthermappedIvl.GetMin()) { pOffsetCurve[jj][0]  = &rOtherGenCurve ;
                                                                        dOffsetLength[jj][0] = bSameExtrusionDir ? rOtherIvl.GetMin() : rOtherIvl.GetMax() ;
                                                                      }
              else if(sXSectIvl.GetMin() == sOthermappedIvl.GetMax()) { pOffsetCurve[jj][0]  = &rOtherGenCurve ;
                                                                        dOffsetLength[jj][0] = bSameExtrusionDir ? rOtherIvl.GetMax() : rOtherIvl.GetMin() ;
                                                                      }

              // remember which translation of which genCurve defines the top of the XSectIvl.
              if     (sXSectIvl.GetMax() == sExtrusionIvl.GetMin())   { pOffsetCurve[jj][1]  = pExtrusionGenCurve ;
                                                                        dOffsetLength[jj][1] = sExtrusionIvl.GetMin() ;
                                                                      }
              else if(sXSectIvl.GetMax() == sExtrusionIvl.GetMax())   { pOffsetCurve[jj][1]  = pExtrusionGenCurve ;
                                                                        dOffsetLength[jj][1] = sExtrusionIvl.GetMax() ;
                                                                      }
              else if(sXSectIvl.GetMax() == sOthermappedIvl.GetMin()) { pOffsetCurve[jj][1]  = &rOtherGenCurve ;
                                                                        dOffsetLength[jj][1] = bSameExtrusionDir ? rOtherIvl.GetMin() : rOtherIvl.GetMax() ;
                                                                      }
              else if(sXSectIvl.GetMax() == sOthermappedIvl.GetMax()) { pOffsetCurve[jj][1]  = &rOtherGenCurve ;
                                                                        dOffsetLength[jj][1] = bSameExtrusionDir ? rOtherIvl.GetMax() : rOtherIvl.GetMin() ;
                                                                      }
            } // end memory save needed for SM_ST_RANGE_OF_VALUES solutions

          // build an intersection line running parallel to the extrusion vector from sXSectIvl.

          // single point of contact
          if(sXSectIvl.GetLength() * dExtrusionLength < dTol)
            {
              // build degenerate curve
              lNewCurveCount++ ;
              SER(SmBSplineCurve::CreateDegenerateCurve(*pContext,
                                                        3, // dim of result
                                                        sExtrusionPoint[jj] + sXSectIvl.GetMid() * sExtrusionVec,
                                                        pLineOrDegenerate));
              eTsectType     =   SM_TC_TOUCHING ;
            }
          else // overlap
            {
              // build line
              lNewCurveCount++ ;
              SER(SmBSplineCurve::CreateLineSegment(*pContext,
                                                    3,
                                                    sExtrusionPoint[jj] + sXSectIvl.GetMin() * sExtrusionVec,
                                                    sExtrusionPoint[jj] + sXSectIvl.GetMax() * sExtrusionVec,
                                                    pLineOrDegenerate));
              eTsectType     =   rSol.m_eSolutionType == SM_ST_SINGLE_VALUE
                               ? SM_TC_CROSSING
                               : SM_TC_REGION_BOUNDARY ;
            }

          // set outputs
          NER(pLineOrDegenerate);
          if(pOpt3DCurves)   { pOpt3DCurves->Add(pLineOrDegenerate); }
          if(pOptCurveTypes) { pOptCurveTypes->Add(eTsectType); }

#ifdef SM_DEBUG_CODE // draw inputs and new intersection curve
          if (bDebugMe)
            {
              pLineOrDegenerate->Dump() ;

              SmFace *pFace1 = (SmFace *)rExtrusionSurf.GetFace() ;
              SmBrep *pBrep1 = pFace1 ? pFace1->GetBrep() : NULL ;
              SmExtent1d sOtherNaturalIvl = rOtherGenCurve.GetNaturalInterval() ;
              SmPoint3d sOtherDrawPoint[3][2] ;
              for(ii=0;ii<3;ii++) { rOtherGenCurve.EvaluatePoint(sOtherNaturalIvl.Evaluate(((double)ii)/2.0), sOtherDrawPoint[ii][1]) ;
                                    sOtherDrawPoint[ii][0] = sOtherDrawPoint[ii][1] + rOtherIvl.GetMin() * rOtherExtrusionVec ;
                                    sOtherDrawPoint[ii][1] = sOtherDrawPoint[ii][1] + rOtherIvl.GetMax() * rOtherExtrusionVec ;
                                  }
              smgfx_Erase();
              smgfx_SetLook(1,2, 0,0,1) ; if(pBrep1) { pBrep1->Draw(TRUE) ; } sm_GraphicsLoop() ;
              smgfx_SetLook(2,3, 1,.5,0); rOtherGenCurve.Draw() ; sm_GraphicsLoop() ;
              smgfx_SetLook(2,3, 1,.5,0); for(ii=0;ii<3;ii++) { sOtherDrawPoint[ii][0].DrawPointToPoint(sOtherDrawPoint[ii][1]) ; sm_GraphicsLoop() ; }
              smgfx_SetLook(1,2, 0,1,1) ; rExtrusionSurf.DrawUV() ; sm_GraphicsLoop() ;
              smgfx_SetLook(1,2, 0,0,0) ; if(pFace1) { pFace1->Draw(SM_DM_CROSSHATCH,10,10) ; } sm_GraphicsLoop() ;
              smgfx_SetLook(1,6, 1,.5,0) ;  sExtrusionPoint[jj].Draw() ; sm_GraphicsLoop() ;
              smgfx_SetLook(1,6, 1,.5,0) ;  sOtherPoint[jj].Draw() ; sm_GraphicsLoop() ;
              smgfx_SetLook(2,3, 1,0,0) ; pLineOrDegenerate->Draw() ; sm_GraphicsLoop() ;
              sm_GraphicsLoop();
            }
#endif // SM_DEBUG_CODE
        } // end iter solution endPoints making line segments parallel to ExtrusionVec

      // all done for Point Solutions

      // for range Solutions - need to add in segments of the GenCurves to finish
      //   bounding the coincident region
      if(rSol.m_eSolutionType == SM_ST_RANGE_OF_VALUES)
        {
          // The solution for a segment of a generator is either
          //     a surface region, bounded by 2 such segments
          //     (taken on 2 different generators) and 2 lines,
          //     connecting the endpoints of these segments;
          // or
          //     just the segment in question.
          // or
          //     some combinations of the above.

          // because the genCurves are allowed to be general its possible
          //  that the coincident region is not bounded by four curves
          //  as in the simple case.  It is possible that the translated
          //  genCurves intersect one another creating a potential mix
          //  of coincident areas, coincident curve fragments and degenerate
          //  point curves all bounded by some combination of pieces of
          //  translated boundary curves and swept lineSegments on the ends.

          //  There are some simple cases that we handle here
          //    otherwise we punt (TODO: we could handle the general case here
          //                             we have all the information we need to do it.)
          //    and tell the system to use the general surf/surf intersection algorithm.
          //  1. Both endCurves are lines - so there is a coincident region
          //      and The top Curve on both ends is the same translated copy of one of the GenCurves
          //      and the bot Curve on both ends is the same translated copy of one of the GenCurves.
          //    - we assume that there is one coincident region bounded by 4 curves.
          //  2. Both endCurves are degenerate points - so there is no coincident region just
          //      a coincident curve.  We remove and delete the degenerate point output
          //        and add in one trimmed Curve which is tested to see that it is coincident
          //        with the properly translated segment of the other GenCurve.
          //  Cases which have not been coded include cases where the IsoParameter boundary
          //     curves of the swept surface intersect in such a way that
          //      A. the coincident region pinches out and has only 3 sides.
          //      B. The separation between the range sol ends varies enough that
          //         the top and bot boundary curves change somewhere within the coincident region.

          // first simple case - coincident region bounded by 4 curves
          ULONG lSize = pOpt3DCurves->GetSize() ;
          if(   pOpt3DCurves != NULL
             && lNewCurveCount == 2                                      // both solution endPoints produced a sideBoundary curve
             && pOptCurveTypes->GetAt(lSize-1) == SM_TC_REGION_BOUNDARY  // and both sideBoundary curves
             && pOptCurveTypes->GetAt(lSize-2) == SM_TC_REGION_BOUNDARY  //    are nonDegenerate
             && pOffsetCurve[0][0]  == pOffsetCurve[1][0]                // and the same GenCurve defined both sides of the BotBoundary Curve
             && SM_IS_ZERO(dOffsetLength[0][0] - dOffsetLength[1][0])    //     with the same offset distance
             && pOffsetCurve[0][1]  == pOffsetCurve[1][1]                // and the same GenCurve defined both sides of the TopBoundary Curve
             && SM_IS_ZERO(dOffsetLength[0][1] - dOffsetLength[1][1]))   //     with the same offset distance
            {
              // Add translated and trimmed copy of the appropriate GenCurve for the bottom (kk=0) and top(kk==1) boundary
              for(kk=0;kk<2;kk++)
                {
                  // TgtGenCurve locals        bSameExtrusionDir
                  SmCurve        * pTgtCurve   = pOffsetCurve[0][kk] ;
                  ULONG            lTgtIndex   = (pTgtCurve == &rOtherGenCurve) ? 1 : 0 ;
                  SmExtent1d       dTrimExtent1(smos_Min(rSol.m_vStart[lTgtIndex], rSol.m_vEnd[lTgtIndex]),
                                                smos_Max(rSol.m_vStart[lTgtIndex], rSol.m_vEnd[lTgtIndex]));
                  SmVector3d     & rTgtExtrusion = lTgtIndex == 0 ? sExtrusionVec : rOtherExtrusionVec ;

                  SmAxis2Placement sShiftRF;
                  sShiftRF.Translate(dOffsetLength[0][kk] * rTgtExtrusion);

                  // copy, translate, and trim the TgtGenCurve
                  SmCurve * pCopiedGenerator = NULL ;
                  pTgtCurve->Copy(*pContext, pCopiedGenerator) ; NER(pCopiedGenerator) ;
                  pCopiedGenerator->Transform(sShiftRF);
                  pCopiedGenerator->Trim(dTrimExtent1); // may snap sIvl by tol to existing knots

                  // add curve to output
                  if(pOpt3DCurves)   { pOpt3DCurves->Add(pCopiedGenerator); }
                  if(pOptCurveTypes) { pOptCurveTypes->Add(SM_TC_REGION_BOUNDARY); }

#ifdef SM_DEBUG_CODE // draw inputs and new intersection curve
                  if (bDebugMe)
                    {
                      pCopiedGenerator->Dump() ;

                      SmFace *pFace1 = (SmFace *)rExtrusionSurf.GetFace() ;
                      SmBrep *pBrep1 = pFace1 ? pFace1->GetBrep() : NULL ;
                      SmExtent1d sOtherNaturalIvl = rOtherGenCurve.GetNaturalInterval() ;
                      SmPoint3d sMidPoint, sMidVec ;
                      pTgtCurve->EvaluatePoint(pTgtCurve->GetNaturalInterval().Evaluate(.5), sMidPoint) ;
                      sMidVec = dOffsetLength[0][kk] * sExtrusionVec ;
                      SmPoint3d sOtherDrawPoint[3][2] ;
                      for(ii=0;ii<3;ii++) { rOtherGenCurve.EvaluatePoint(sOtherNaturalIvl.Evaluate(((double)ii)/2.0), sOtherDrawPoint[ii][1]) ;
                                            sOtherDrawPoint[ii][0] = sOtherDrawPoint[ii][1] + rOtherIvl.GetMin() * rOtherExtrusionVec ;
                                            sOtherDrawPoint[ii][1] = sOtherDrawPoint[ii][1] + rOtherIvl.GetMax() * rOtherExtrusionVec ;
                                          }
                      smgfx_Erase();
                      smgfx_SetLook(2,3, 0,0,1) ; pTgtCurve->Draw() ; sm_GraphicsLoop() ;
                      smgfx_SetLook(3,4, 1,0,0) ; pCopiedGenerator->Draw() ; sm_GraphicsLoop() ;
                      smgfx_SetLook(2,4, .5,0,1); sMidVec.Draw(&sMidPoint) ; sm_GraphicsLoop() ;
                      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep1) { pBrep1->Draw(TRUE) ; } sm_GraphicsLoop() ;
                      smgfx_SetLook(2,3, 1,.5,0); rOtherGenCurve.Draw() ; sm_GraphicsLoop() ;
                      smgfx_SetLook(2,3, 1,.5,0); for(ii=0;ii<3;ii++) { sOtherDrawPoint[ii][0].DrawPointToPoint(sOtherDrawPoint[ii][1]) ; sm_GraphicsLoop() ; }
                      smgfx_SetLook(1,2, 0,1,1) ; rExtrusionSurf.DrawUV() ; sm_GraphicsLoop() ;
                      smgfx_SetLook(1,2, 0,0,0) ; if(pFace1) { pFace1->Draw(SM_DM_CROSSHATCH,10,10) ; } sm_GraphicsLoop() ;
                      sm_GraphicsLoop();
                    }
#endif // SM_DEBUG_CODE
                }
            } // end coincident region branch

          // Second simple case - the projected intersection is in reality a 3d intersection
          //    note: we could do an actual intersection of the intervals involved but
          //          to save time we assume that if both end points are coincident
          //          so will be the whole curve segment.
          else if(   pOpt3DCurves != NULL
                  && lNewCurveCount == 2                                // both solution endPoints produced a sideBoundary curve
                  && pOptCurveTypes->GetAt(lSize-1) == SM_TC_TOUCHING   // and both sideBoundary curves
                  && pOptCurveTypes->GetAt(lSize-2) == SM_TC_TOUCHING)  //    are nonDegenerate
            {
              // remove and delete the last two degenerate curves - we don't need them
              for(kk=0;kk<2;kk++)
                {
                  if(pOpt3DCurves)   { SmCurve *pTmpCurve = pOpt3DCurves->GetLast() ;
                                       delete pTmpCurve ; pTmpCurve = NULL ;
                                       pOpt3DCurves->RemoveLast() ;
                                     }
                  if(pOptCurveTypes) { pOptCurveTypes->RemoveLast() ; }
                }

              // add in one translated and trimmed copy of pExtrusionGenCurve
              //   we could have used pOtherGenCurve since they are coincident

              // TgtGenCurve locals
              SmCurve        * pTgtCurve     = pOffsetCurve[0][0] ;
              ULONG            lTgtIndex     = (pTgtCurve == &rOtherGenCurve) ? 1 : 0 ;
              SmExtent1d       dTrimExtent1(rSol.m_vStart[lTgtIndex], rSol.m_vEnd[lTgtIndex]);
              SmVector3d     & rTgtExtrusion = lTgtIndex == 0 ? sExtrusionVec : rOtherExtrusionVec ;

              SmAxis2Placement sShiftRF;
              sShiftRF.Translate(dOffsetLength[0][0] * rTgtExtrusion);

              // copy, translate, and trim the TgtGenCurve
              SmCurve * pCopiedGenerator = NULL ;
              pTgtCurve->Copy(*pContext, pCopiedGenerator) ; NER(pCopiedGenerator) ;
              pCopiedGenerator->Transform(sShiftRF);
              pCopiedGenerator->Trim(dTrimExtent1);  // may snap sIvl by tol to existing knots

              // add curve to output
              if(pOpt3DCurves)   { pOpt3DCurves->Add(pCopiedGenerator); }
              if(pOptCurveTypes) { pOptCurveTypes->Add(SM_TC_REGION_BOUNDARY); }

#ifdef SM_DEBUG_CODE // draw inputs and new intersection curve
              if (bDebugMe)
                {
                  pCopiedGenerator->Dump() ;

                  SmFace *pFace1 = (SmFace *)rExtrusionSurf.GetFace() ;
                  SmBrep *pBrep1 = pFace1 ? pFace1->GetBrep() : NULL ;
                  SmExtent1d sOtherNaturalIvl = rOtherGenCurve.GetNaturalInterval() ;
                  SmPoint3d sOtherDrawPoint[3][2] ;
                  for(ii=0;ii<3;ii++) { rOtherGenCurve.EvaluatePoint(sOtherNaturalIvl.Evaluate(((double)ii)/2.0), sOtherDrawPoint[ii][1]) ;
                                        sOtherDrawPoint[ii][0] = sOtherDrawPoint[ii][1] + rOtherIvl.GetMin() * rOtherExtrusionVec ;
                                        sOtherDrawPoint[ii][1] = sOtherDrawPoint[ii][1] + rOtherIvl.GetMax() * rOtherExtrusionVec ;
                                      }
                  smgfx_Erase();
                  smgfx_SetLook(1,2, 0,0,1) ; if(pBrep1) { pBrep1->Draw(TRUE) ; } sm_GraphicsLoop() ;
                  smgfx_SetLook(2,3, 1,.5,0); rOtherGenCurve.Draw() ; sm_GraphicsLoop() ;
                  smgfx_SetLook(2,3, 1,.5,0); for(ii=0;ii<3;ii++) { sOtherDrawPoint[ii][0].DrawPointToPoint(sOtherDrawPoint[ii][1]) ; sm_GraphicsLoop() ; }
                  smgfx_SetLook(1,2, 0,1,1) ; rExtrusionSurf.DrawUV() ; sm_GraphicsLoop() ;
                  smgfx_SetLook(1,2, 0,0,0) ; if(pFace1) { pFace1->Draw(SM_DM_CROSSHATCH,10,10) ; } sm_GraphicsLoop() ;
                  smgfx_SetLook(2,3, 1,0,0) ; pCopiedGenerator->Draw() ; sm_GraphicsLoop() ;
                  sm_GraphicsLoop();
                }
#endif // SM_DEBUG_CODE
            } // end coincident curve branch

          // else - the situation is too complicated,
          //    free any results and ask for a full sur/surf intersection
          //    TODO: there is enough information here to finish this problem
          //          but we would have to intersect the 2 3dCurves and parse
          //          those intersections to get the proper trimmed segments of the
          //          actual 3d curves
          else
            {
              // delete and reset any current output
              if(pOpt3DCurves)   { // open a scope
                                   SmObjsDelete<SmCurve *> sClean(pOpt3DCurves) ;
                                 } // close scope and delete all the Curves in r3dCurves and ReSet r3dCurves array
              if(pOpt3DCurves)   { pOpt3DCurves->ReSet() ; }
              if(pOptCurveTypes) { pOptCurveTypes->ReSet() ; }

              // ask for full surf/surf intersection solution
              rbNeedsMoreIntersections = TRUE ;
              return(SM_SUCCESS) ;

            } // end too complicated branch
        } // end rSol.m_eSolutionType == SM_ST_RANGE_OF_VALUES check
    } // end iter every solution

#ifdef SM_DEBUG_CODE // draw inputs and new intersection curves
  if (bDebugMe)
    {
      SmFace *pFace1 = (SmFace *)rExtrusionSurf.GetFace() ;
      SmBrep *pBrep1 = pFace1 ? pFace1->GetBrep() : NULL ;
      SmExtent1d sOtherNaturalIvl = rOtherGenCurve.GetNaturalInterval() ;
      SmPoint3d sOtherDrawPoint[3][2] ;
      for(ii=0;ii<3;ii++) { rOtherGenCurve.EvaluatePoint(sOtherNaturalIvl.Evaluate(((double)ii)/2.0), sOtherDrawPoint[ii][1]) ;
                            sOtherDrawPoint[ii][0] = sOtherDrawPoint[ii][1] + rOtherIvl.GetMin() * rOtherExtrusionVec ;
                            sOtherDrawPoint[ii][1] = sOtherDrawPoint[ii][1] + rOtherIvl.GetMax() * rOtherExtrusionVec ;
                          }
      smgfx_Erase();
      smgfx_SetLook(2,3, 1,0,0) ; for(ii=0;ii<pOpt3DCurves->GetSize();ii++) { pOpt3DCurves->GetAt(ii)->Draw() ; sm_GraphicsLoop() ; }
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep1) { pBrep1->Draw(TRUE) ; } sm_GraphicsLoop() ;
      smgfx_SetLook(2,3, 1,.5,0); rOtherGenCurve.Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook(2,3, 1,.5,0); for(ii=0;ii<3;ii++) { sOtherDrawPoint[ii][0].DrawPointToPoint(sOtherDrawPoint[ii][1]) ; sm_GraphicsLoop() ; }
      smgfx_SetLook(1,2, 0,1,1) ; rExtrusionSurf.DrawUV() ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,0,0) ; if(pFace1) { pFace1->Draw(SM_DM_CROSSHATCH,10,10) ; } sm_GraphicsLoop() ;
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

  // all done
  return(SM_SUCCESS) ;

} // end static SmStatus sm_IntersectGenerators

/*******************************************************************//**
PURPOSE: Find intersection curve(s) of this with other cylinder

NOTES: when the cyl axis is parallel with the extrusion vector
             the intersection will be a line.
***********************************************************************/
SmStatus SmSurfOfExtrusion::IntersectWithCylinder
  (const SmContext     & crContext,                // in : context for new object construction
   const SmExtent2d    & crUVDomain,               // NotUsed: in : intersection limit for this surface
   const SmCone        & crCylinder,               // in : target intersection cylinder
   const SmExtent2d    & crOtherUVDomain,          // NotUsed: in : intersection limit for target cylinder
   const SmBoolean       bUseSurfaceEdges[2],      // NotUsed: in : bUseSurfaceEdges: TRUE = find xsect curve start points from boundaryCurve/surface xsects
                                                   //      typically these values are TRUE - its a small savings if you know
                                                   //      the boundaries of one surface don't intersect the other surface
   const SmApproxTol3d * pdOptApproxTol3d,         // in :
   const double        * ,                         // in : pdOptAngTolRad =
   SmBoolean           & rbNeedsMoreIntersections, // out: TRUE = special case intersection failed - use general intersection algorithm
   SmTArray<SmCurve*>  * pOpt3DCurves,             // out: Intersection 3DCurves, NULL to ignore
   SmTArray<SmCurve*>  * pOptSurface1UVCurves,     // out: associated UVTrimCurves on this surface, NULL to ignore
   SmTArray<SmCurve*>  * pOptSurface2UVCurves,     // out: associated UVTrimCurves on target cylinder, NULL to ignore
   SmTArray<SmTsectCurveType> * pOptCurveTypes,    // out: associated intersecton type, NULL to ignore
   SmTArray<double>    * pOptDeviations)           // out: associated max 3DCurve to surface distance, NULL to ignore
  const
{
  SM_REF3(crUVDomain, crOtherUVDomain, bUseSurfaceEdges) ;
  SM_DUMP_AND_ASSERT2_VALID(this) ;

  // init output
  rbNeedsMoreIntersections = FALSE ;
  if (pOpt3DCurves)         pOpt3DCurves->ReSet();
  if (pOptSurface1UVCurves) pOptSurface1UVCurves->ReSet();
  if (pOptSurface2UVCurves) pOptSurface2UVCurves->ReSet();
  if (pOptCurveTypes)       pOptCurveTypes->ReSet();
  if (pOptDeviations)       pOptDeviations->ReSet();

  // locals
  double dOtherBotRadius = crCylinder.GetBotRadius();
  double dOtherTopRadius = crCylinder.GetTopRadius();

  // no work - other surface not a rectilinear cylinder:
  double dRadiusDiffTolerance =  SM_EFF_ZERO * (1.0 + smos_Max(dOtherBotRadius, dOtherTopRadius)) ;
  if (smos_Fabs(dOtherBotRadius - dOtherTopRadius) > dRadiusDiffTolerance)
    {
      rbNeedsMoreIntersections = TRUE;
      return SM_SUCCESS;
    }

  // pick tolerance
  double dTol = 0.0;
  if( pdOptApproxTol3d ) dTol = *pdOptApproxTol3d;
  else dTol = SM_EFF_ZERO_SQRT;

  // thisExtrusionSurface Extrusion Segment
  SmPoint3d  sOrigin       = GetOrigin();
  SmVector3d sExtrusionVec = GetExtrusionVector();

  // otherCylinderSurface locals
  ULONG ii ;
  const SmAxis2Placement & sOtherPosition = crCylinder.GetPosition();
  const SmPoint3d        & rOtherOrigin   = sOtherPosition.GetOriginRef();
  SmVector3d               sOtherZAxis    = sOtherPosition.GetZAxis();
  SmExtent1d               sOtherIvl(crCylinder.GetBotHeight(), crCylinder.GetTopHeight()) ;
  SmPoint3d sOtherAxisSegmentStartPoint(  rOtherOrigin + sOtherIvl.GetMin() * sOtherZAxis);
  SmPoint3d sOtherAxisSegmentEndPoint  (  rOtherOrigin + sOtherIvl.GetMax() * sOtherZAxis);
  SmBSplineCurve *pOtherGenCurve = NULL ;
  crCylinder.CreateIsoCircleFromPoint(crContext,
                                      sOtherAxisSegmentStartPoint,
                                      dTol,
                                      pOtherGenCurve) ;
  SmObjDelete sClean(pOtherGenCurve) ;

#ifdef SM_DEBUG_CODE // draw input surfaces and their breps and faces if possible
SmBoolean bDebugMe = FALSE;
  static ULONG lCount      = 1 ; lCount++ ;
  static ULONG lDebugCount = 0 ;
  if (bDebugMe || lCount == lDebugCount)
    {
      SmFace *pFace1 = (SmFace *)GetFace() ;
      SmFace *pFace2 = (SmFace *)crCylinder.GetFace() ;
      SmBrep *pBrep1 = pFace1 ? pFace1->GetBrep() : NULL ;
      SmBrep *pBrep2 = pFace2 ? pFace2->GetBrep() : NULL ;

      smgfx_Erase();
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep1) { pBrep1->Draw(TRUE) ; } sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,0) ; if(pBrep2) { pBrep2->Draw(TRUE) ; } sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,1) ; DrawUV() ; sm_GraphicsLoop() ;
      smgfx_SetLook(3,4, 0,.5,1); rOtherOrigin.Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook(2,3, 1,.5,0); sOtherAxisSegmentStartPoint.DrawPointToPoint(sOtherAxisSegmentEndPoint) ; sm_GraphicsLoop() ;
      smgfx_SetLook(2,3, 1,.5,0); pOtherGenCurve->Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 1,1,0) ; crCylinder.DrawUV() ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,0,0) ; if(pFace1) { pFace1->Draw(SM_DM_CROSSHATCH,10,10) ; } sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,0,0) ; if(pFace2) { pFace2->Draw(SM_DM_CROSSHATCH,10,10) ; } sm_GraphicsLoop() ;
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

  // compute intersections based on overlap of extrusion lines that coalign from the GenCurves
  sm_IntersectGenerators(*this,
                          dTol,
                          *pOtherGenCurve, sOtherZAxis, sOtherIvl,
                          pOpt3DCurves, pOptCurveTypes,
                          rbNeedsMoreIntersections) ;

#ifdef SM_DEBUG_CODE // add xSectCurves to current drawing
  if (bDebugMe || lCount == lDebugCount)
    {
      sm_GraphicsLoop();
      smgfx_SetLook(3,4, 1,0,0) ; for(ii=0;ii<pOpt3DCurves->GetSize(); ii++)
                                    { pOpt3DCurves->GetAt(ii)->Draw() ; sm_GraphicsLoop() ; }
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

  // for every intersection found - finish the outputs
  for(ii=0; ii<pOpt3DCurves->GetSize(); ii++)
    {
      if (pOptSurface1UVCurves) pOptSurface1UVCurves->Add(NULL);
      if (pOptSurface2UVCurves) pOptSurface2UVCurves->Add(NULL);
      if (pOptDeviations) pOptDeviations->Add(0.0);
    }

#ifdef SM_DEBUG_CODE // draw inputs and new intersection curves
  if (bDebugMe || lCount == lDebugCount)
    {
      SmFace *pFace1 = (SmFace *)GetFace() ;
      SmFace *pFace2 = (SmFace *)crCylinder.GetFace() ;
      SmBrep *pBrep1 = pFace1 ? pFace1->GetBrep() : NULL ;
      SmBrep *pBrep2 = pFace2 ? pFace2->GetBrep() : NULL ;

      smgfx_Erase();
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep1) { pBrep1->Draw(TRUE) ; } sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,0) ; if(pBrep2) { pBrep2->Draw(TRUE) ; } sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,1) ; DrawUV() ; sm_GraphicsLoop() ;
      smgfx_SetLook(3,4, 0,.5,1); rOtherOrigin.Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook(2,3, 1,.5,0); sOtherAxisSegmentStartPoint.DrawPointToPoint(sOtherAxisSegmentEndPoint) ; sm_GraphicsLoop() ;
      smgfx_SetLook(2,3, 1,.5,0); pOtherGenCurve->Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 1,1,0) ; crCylinder.DrawUV() ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,0,0) ; if(pFace1) { pFace1->Draw(SM_DM_CROSSHATCH,10,10) ; } sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,0,0) ; if(pFace2) { pFace2->Draw(SM_DM_CROSSHATCH,10,10) ; } sm_GraphicsLoop() ;
      smgfx_SetLook(3,4, 1,0,0) ; for(ii=0;ii<pOpt3DCurves->GetSize(); ii++)
                                    { pOpt3DCurves->GetAt(ii)->Draw() ; sm_GraphicsLoop() ; }
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

  // all done
  return SM_SUCCESS;

} // end SmSurfOfExtrusion::IntersectWithCylinder

/*******************************************************************//**
PURPOSE: Find intersection curve (line) of this with other extruded surf
            (when the extrusion vectors are parallel)

NOTES:
***********************************************************************/
SmStatus SmSurfOfExtrusion::IntersectWithSurfOfExtrusion
  (const SmContext         & crContext,            // NotUsed: in : context for new object construction
   const SmExtent2d        & crUVDomain,           // NotUsed: in : intersection limit for this surface
   const SmSurfOfExtrusion & crSurfOfExtrusion,    // in : target intersection cylinder
   const SmExtent2d        & crOtherUVDomain,      // NotUsed: in : intersection limit for target cylinder
   const SmBoolean [2],                            // in : bUseSurfaceEdges: TRUE = find xsect curve start points from boundaryCurve/surface xsects
                                                   //      typically these values are TRUE - its a small savings if you know
                                                   //      the boundaries of one surface don't intersect the other surface
   const SmApproxTol3d * pdOptApproxTol3d,         // in :
   const double        * ,                         // in : pdOptAngTolRad =
   SmBoolean           & rbNeedsMoreIntersections, // out: TRUE = special case intersection failed - use general intersection algorithm
   SmTArray<SmCurve*>  * pOpt3DCurves,             // out: Intersection 3DCurves, NULL to ignore
   SmTArray<SmCurve*>  * pOptSurface1UVCurves,     // out: associated UVTrimCurves on this surface, NULL to ignore
   SmTArray<SmCurve*>  * pOptSurface2UVCurves,     // out: associated UVTrimCurves on target cylinder, NULL to ignore
   SmTArray<SmTsectCurveType> * pOptCurveTypes,    // out: associated intersecton type, NULL to ignore
   SmTArray<double>    * pOptDeviations)           // out: associated max 3DCurve to surface distance, NULL to ignore
  const
{
  SM_REF3(crContext, crUVDomain, crOtherUVDomain) ;
  SM_DUMP_AND_ASSERT2_VALID(this) ;

  // init output
  rbNeedsMoreIntersections = FALSE ;
  if (pOpt3DCurves)         pOpt3DCurves->ReSet();
  if (pOptSurface1UVCurves) pOptSurface1UVCurves->ReSet();
  if (pOptSurface2UVCurves) pOptSurface2UVCurves->ReSet();
  if (pOptCurveTypes)       pOptCurveTypes->ReSet();
  if (pOptDeviations)       pOptDeviations->ReSet();

  // locals

  // pick tolerance
  double dTol = 0.0;
  if( pdOptApproxTol3d ) dTol = *pdOptApproxTol3d;
  else dTol = SM_EFF_ZERO_SQRT ;

  // OtherExtrusionSurface Extrusion Segment
  ULONG ii ;
  SmPoint3d  sOtherOrigin       = crSurfOfExtrusion.GetOrigin() ;
  SmVector3d sOtherExtrusionVec = crSurfOfExtrusion.GetExtrusionVector() ;
  SmExtent1d sOtherIvl          = crSurfOfExtrusion.GetSTEPUVDomain().GetVInterval() ;
  SmCurve *  pOtherGenCurve     = crSurfOfExtrusion.GetGenCurve() ;

#ifdef SM_DEBUG_CODE // draw input surfaces and their breps and faces if possible
SmBoolean bDebugMe = FALSE;
  static ULONG lCount      = 1 ; lCount++ ;
  static ULONG lDebugCount = 0 ;
  if (bDebugMe || lCount == lDebugCount)
    {
      SmFace *pFace1 = (SmFace *)GetFace() ;
      SmFace *pFace2 = (SmFace *)crSurfOfExtrusion.GetFace() ;
      SmBrep *pBrep1 = pFace1 ? pFace1->GetBrep() : NULL ;
      SmBrep *pBrep2 = pFace2 ? pFace2->GetBrep() : NULL ;

      smgfx_Erase();
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep1) { pBrep1->Draw(TRUE) ; } sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,0) ; if(pBrep2) { pBrep2->Draw(TRUE) ; } sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,1) ; DrawUV() ; sm_GraphicsLoop() ;
      smgfx_SetLook(2,3, 1,.5,0); pOtherGenCurve->Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook(3,4, 0,.5,1); sOtherOrigin.Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook(2,3, 1,.5,0); sOtherExtrusionVec.Draw(&sOtherOrigin) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 1,1,0) ; crSurfOfExtrusion.DrawUV() ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,0,0) ; if(pFace1) { pFace1->Draw(SM_DM_CROSSHATCH,10,10) ; } sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,0,0) ; if(pFace2) { pFace2->Draw(SM_DM_CROSSHATCH,10,10) ; } sm_GraphicsLoop() ;
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

  // compute intersections based on overlap of extrusion lines that coalign from the GenCurves
  sm_IntersectGenerators(*this,
                          dTol,
                          *pOtherGenCurve, sOtherExtrusionVec, sOtherIvl,
                          pOpt3DCurves, pOptCurveTypes,
                          rbNeedsMoreIntersections) ;

#ifdef SM_DEBUG_CODE // add xSectCurves to current drawing
  if (bDebugMe || lCount == lDebugCount)
    {
      sm_GraphicsLoop();
      smgfx_SetLook(1,2, 1,0,0) ; for(ii=0;ii<pOpt3DCurves->GetSize(); ii++)
                                    { pOpt3DCurves->GetAt(ii)->Draw() ; sm_GraphicsLoop() ; }
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

  // for every intersection found - finish the outputs
  for(ii=0; ii<pOpt3DCurves->GetSize(); ii++)
    {
      if (pOptSurface1UVCurves) pOptSurface1UVCurves->Add(NULL);
      if (pOptSurface2UVCurves) pOptSurface2UVCurves->Add(NULL);
      if (pOptDeviations) pOptDeviations->Add(0.0);
    }

#ifdef SM_DEBUG_CODE // draw input surfaces and their breps and faces if possible
  if (bDebugMe || lCount == lDebugCount)
    {
      SmFace *pFace1 = (SmFace *)GetFace() ;
      SmFace *pFace2 = (SmFace *)crSurfOfExtrusion.GetFace() ;
      SmBrep *pBrep1 = pFace1 ? pFace1->GetBrep() : NULL ;
      SmBrep *pBrep2 = pFace2 ? pFace2->GetBrep() : NULL ;

      smgfx_Erase();
      smgfx_SetLook(3,4, 1,0,0) ; for(ii=0;ii<pOpt3DCurves->GetSize(); ii++)
                                    { pOpt3DCurves->GetAt(ii)->Draw() ; sm_GraphicsLoop() ; }
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep1) { pBrep1->Draw(TRUE) ; } sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,0) ; if(pBrep2) { pBrep2->Draw(TRUE) ; } sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,1) ; DrawUV() ; sm_GraphicsLoop() ;
      smgfx_SetLook(2,3, 1,.5,0); pOtherGenCurve->Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook(3,4, 0,.5,1); sOtherOrigin.Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook(2,3, 1,.5,0); sOtherExtrusionVec.Draw(&sOtherOrigin) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 1,1,0) ; crSurfOfExtrusion.DrawUV() ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,0,0) ; if(pFace1) { pFace1->Draw(SM_DM_CROSSHATCH,10,10) ; } sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,0,0) ; if(pFace2) { pFace2->Draw(SM_DM_CROSSHATCH,10,10) ; } sm_GraphicsLoop() ;
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE
  // all done
  return SM_SUCCESS;

} // end SmSurfOfExtrusion::IntersectWithSurfOfExtrusion

/*******************************************************************//**
PURPOSE: Static function to determine if a given nurb surface is a SurfOfExtrusion.
    If it is a SurfOfExtrusion then one is created whose BSplineSurface
    representation is exactly equal to the input surfaces.

NOTES:
    1. For now, it will check only those surfaces that are linear
       in one param (but not both): degree 1 and 2 control points.
    2. The new surface's analytic domain (in v) and extrusion vector
       are set up so that the surface is parameterized
       by arcLength in the sweep direction.
***********************************************************************/
SmBoolean SmSurfOfExtrusion::IsNurbSurfaceSurfOfExtrusion
(
  const SmContext        & crContext,           // in : context for new object construction
  const SmBSplineSurface * pTestSurface,        // in : target surface
  SmSurfOfExtrusion     *& rpSurfOfExtrusion,   // out: new surface when target surface is a SurfaceofExtrusion, or NULL
  double                   dToleranceScale      // in : tol =  dToleranceScale * ANALYTIC_TOL_SCALE * SM_EFF_ZERO * (1.0 + sMinPnt.GetMaxDimension() + sMaxPnt.GetMaxDimension())
)
{
  // init output
  rpSurfOfExtrusion = NULL ;

  // surface locals
  ULONG lDegU = pTestSurface->GetDegree(SM_SP_U);
  ULONG lDegV = pTestSurface->GetDegree(SM_SP_V);

  // check parameterization
  // - must be deg 1 in just one direction and
  //   have more than 2 control points in the other direction
  if (   (lDegU == 1 && lDegV == 1) // it's a bilinear or plane...
      || (lDegU != 1 && lDegV != 1)
      || (lDegU == 1 && pTestSurface->GetNumberControlPoints(SM_SP_U) > 2)   // only 2-CPnt lines
      || (lDegV == 1 && pTestSurface->GetNumberControlPoints(SM_SP_V) > 2))  // only 2-CPnt lines
    { return FALSE; }

  // surface locals
  SmSurfParamType eSurfParam;
  SmExtent2d      sNurbUVDomain = pTestSurface->GetNaturalUVDomain();
  SmPoint2d       sEndParam;
  SmBoolean       bSwapUV       = FALSE;
  double          dStartKnot, dEndKnot;

  // get a domain corner surface point to act as the sweep curve's end point
  if (lDegU == 1)
    {
      eSurfParam = SM_SP_U;
      sEndParam  = sNurbUVDomain.Evaluate(1.0,0.0);
      dStartKnot = sNurbUVDomain.GetMin().x;    // begin sweep direction Domain Value
      dEndKnot   = sNurbUVDomain.GetMax().x;    // end   sweep direction Domain Value
      bSwapUV    = TRUE; // so that the STEP standard of linear in V is maintained.
    }
  else   //lDegV == 1
    {
      eSurfParam = SM_SP_V;
      sEndParam  = sNurbUVDomain.Evaluate(0.0,1.0);
      dStartKnot = sNurbUVDomain.GetMin().y;    // begin sweep direction Domain Value
      dEndKnot   = sNurbUVDomain.GetMax().y;    // end   sweep direction Domain Value
      bSwapUV    = FALSE; // so that the STEP standard of linear in V is maintained.
    }

  // evaluate domain upper and lower corner surface points and sweep end point
  SmPoint3d sOrigin, sMaxPnt, sLineEnd;
  pTestSurface->EvaluatePoint(sNurbUVDomain.GetMin(), sOrigin);
  pTestSurface->EvaluatePoint(sNurbUVDomain.GetMax(), sMaxPnt);
  pTestSurface->EvaluatePoint(sEndParam, sLineEnd);

  // compute a tolerance
  double dPtTol =   dToleranceScale
                  * ANALYTIC_TOL_SCALE
                  * SM_EFF_ZERO
                  * (1.0 + sOrigin.GetMaxDimension() + sMaxPnt.GetMaxDimension());

  // let extrusion vector = sweep line endPoint - begPoint
  SmVector3d sExtrusionVec    = sLineEnd - sOrigin;
  double     dExtrusionLength = sExtrusionVec.Length();

  // not an extrusion surface - sweep vector is zero length
  if (dExtrusionLength < SM_EFF_ZERO)
    {
      return FALSE;
    }

  // build GenCurve = Start of sweep IsoParameter Curve
  SmBSplineCurve *pGenCurve = NULL;
  SER(pTestSurface->CreateIsoParametricCurve(crContext,
                                             eSurfParam,
                                             dStartKnot,
                                             0.0,
                                             pGenCurve));
  SmObjDelete sCleanGen(pGenCurve);

  // Get GenCurve locals
  SmExtent1d sCrvNurbIvl = pGenCurve->GetNaturalInterval() ;
  SmExtent1d sCrvSTEPIvl = pGenCurve->GetSTEPInterval() ;

  // convert GenCurve rational to euclidean
  SmTArray<SmPoint3d> sStartCtrlPts(256);
  SmTArray<double>    sStartWeights(256);
  SER(pGenCurve->GetControlPolygon(sStartCtrlPts, sStartWeights));
  SmBoolean           bRational = pGenCurve->IsRational();

  // build EndCurve = End of Sweep IsoParameter Curve
  SmBSplineCurve *pEndCrv = NULL;
  SER(pTestSurface->CreateIsoParametricCurve(crContext,
                                             eSurfParam,
                                             dEndKnot,
                                             0.0,
                                             pEndCrv));

  // convert EndCurve rational to euclidean
  SmTArray<SmPoint3d> sEndCtrlPts(256);
  SmTArray<double>    sEndWeights(256);
  SER(pEndCrv->GetControlPolygon(sEndCtrlPts, sEndWeights));

  // done with the end curve - delete it
  SM_ASSERT(pEndCrv != NULL) ; delete pEndCrv ; pEndCrv = NULL ;

  // test every end curve controlPoint for constant sweep vectors
  for (ULONG i=0; i<sEndCtrlPts.GetSize(); i++)
    {
      // get sVec = this controlPoint sweep - sweepVector
      SmVector3d sVec = (sEndCtrlPts[i] - sStartCtrlPts[i]) - sExtrusionVec ;

      // when any of the EndControlPoints is not a constant sweep of the StartControlPoints
      if (sVec.Length() > dPtTol)
        {
          // this is not a SurfOfExtrusion
          return FALSE;
        }

      // for rational surfaces also test the weights
      if (bRational)
      {
        // When the ratio between the weights isn't constant,
        //   example: ((1 0.7 1) == (2 1.4 2)) is ok
        if (fabs(sStartWeights[i]*sEndWeights[0] - sEndWeights[i]*sStartWeights[0]) > dPtTol)
          {
            // this is not a SurfOfExtrusion
            return FALSE;
          }
      }
    } // end iter every start and end curve control point - testing for constant sweep

  // arrive here once this NurbSurface is found to be an SmSurfOfExtrusion

  // surface is an extrusion - build the output surface

  // scale extrusion by the sweep range to match ExtrusionLength and domain ranges
  sExtrusionVec /= dEndKnot - dStartKnot ;

  // UVDomain is not required to start at 0.0 so GenCurve has to be translated
  // to keep AnalUVDomain == NurbUVDomain and to have equivalent evaluations
  if(!SM_IS_ZERO(dStartKnot))
    {
      SmAxis2Placement sShiftRF;
      sShiftRF.Translate(-dStartKnot * sExtrusionVec);

      // translate
      pGenCurve->Transform(sShiftRF);
      sOrigin -= dStartKnot * sExtrusionVec ;
    }

  // prevent deletion of GenCurve on exit
  sCleanGen.Clear();

  // build new SmSurfOfExtrusion - constructor sets all internal values
  SmExtent2d sAnalUVDomain = SmExtent2d(SmPoint2d(sCrvSTEPIvl.GetMin(),dStartKnot),
                                        SmPoint2d(sCrvSTEPIvl.GetMax(),dEndKnot));
  rpSurfOfExtrusion = new (crContext) SmSurfOfExtrusion(pGenCurve,
                                                        sOrigin,
                                                        sExtrusionVec,
                                                        sAnalUVDomain,
                                                        bSwapUV);
  if (!rpSurfOfExtrusion)
    { return FALSE ; }

  // obsoleted - it's a mistake to introduce Nurb vs. Anal rep variations by copying the m_pNurb object
  //      // Copy pTestSurface->m_pNurb into newSurfOfExtrusion
  //      SM_ASSERT(rpSurfOfExtrusion->m_pNurb == NULL) ;
  //      rpSurfOfExtrusion->m_pNurb = sm_AllocateAndCopyNurbSurface(((SmBSplineSurface *)pTestSurface)->GetGwNurbPointer()) ;
  // end obsolete

  // reparameterize the NURB so that AnalyticSurface->NurbDomain == InputSurface->NurbDomain
  rpSurfOfExtrusion->Reparameterize(pTestSurface->GetNaturalUVDomain()) ;

  // Copy pTestSurface attributes onto newSurfOfExtrusion
  ((SmBSplineSurface*)pTestSurface)->Notify(SM_NO_COPY, rpSurfOfExtrusion, SM_NO_GET_OWNER(rpSurfOfExtrusion), SM_NO_GET_OWNER(pTestSurface));

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
  if (bDebugMe)
    {
      SmBSplineCurve *pIso = NULL ;
      if (rpSurfOfExtrusion->m_bSwapUV) { SER(rpSurfOfExtrusion->CreateIsoParametricCurve(crContext,SM_SP_U,0.0,SM_EFF_ZERO,pIso));
                                        }
      else                              { SER(rpSurfOfExtrusion->CreateIsoParametricCurve(crContext,SM_SP_V,0.0,SM_EFF_ZERO,pIso));
                                        }
      SmObjDelete sCleanIso(pIso);

      // dump input surface, output surface, sweep isoCurve, genCurve
      pTestSurface->Dump();
      rpSurfOfExtrusion->Dump();
      rpSurfOfExtrusion->m_pGenCurve->Dump();
      pIso->Dump();

      smgfx_Erase();
      smgfx_SetLook(1,2, 0,0,1); rpSurfOfExtrusion->DrawUV(); sm_GraphicsLoop();
      smgfx_SetLook(3,4, 1,0,0); rpSurfOfExtrusion->m_pGenCurve->Draw(); sm_GraphicsLoop();
      smgfx_SetLook(6,7, 1,0,1); pIso->Draw(); sm_GraphicsLoop();
      smgfx_SetLook(3,4, 0,1,0); pTestSurface->DrawUV(4,4); sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

  // all done
  SM_DUMP_AND_ASSERT2_VALID(rpSurfOfExtrusion);
  return TRUE;

} // end SmSurfOfExtrusion::IsNurbSurfaceSurfOfExtrusion

/*******************************************************************//**
PURPOSE: Make the Nurb representation for a SmSurfOfExtrusion with
   same domain as m_vAnalUVDomain and
   causes m_pGenCurve->m_vAnalDomain = m_vAnalUVDomain->GetUInterval()

NOTES:

SIDE EFFECTS ---
  1. m_pGenCurve->Interval = m_vAnalUVDomain.GetUInterval()
  2. Build new m_pNurb structure
  3. Delete old m_pNurb structure
***********************************************************************/
SmStatus SmSurfOfExtrusion::MakeNurb
  ()
{
  // Get canonical data from generator curve
  const SmContext * pContext = GetContext();
  SmExtent1d        sAnalUIvl(m_vAnalUVDomain.GetUInterval());

  // Trim the m_pGenCurve to the current m_vAnalUVDomain.UInterval()
  m_pGenCurve->AdjustSTEPInterval(sAnalUIvl);

  // locals to hold m_pGenCurve Bspline parameters
  ULONG               lDim, lCrvDeg;
  SmBSplineCurveForm  eCurveForm;
  SmTArray<SmPoint3d> sCrvCtrlPts;
  SmTArray<double>    sCrvKnots;
  SmTArray<ULONG>     sCrvKnotMult;
  SmTArray<double>    sCrvWeights;
  SmKnotType          eKnotType;
  SmBSplineCurve    * pBSC = SM_CAST_PTR(SmBSplineCurve,m_pGenCurve); NER(pBSC);

  // Extract the m_pGenCurve BSplineCurve parameters
  SER(pBSC->GetCanonical(lDim,
                         lCrvDeg,
                         sCrvCtrlPts,
                         eCurveForm,
                         sCrvKnotMult,
                         sCrvKnots,
                         eKnotType,
                         sCrvWeights));
  ULONG lNumCrvCtrlPts        = sCrvCtrlPts.GetSize();
  SmExtent1d sCrvIvl          = pBSC->GetNaturalInterval();
  SmBoolean bIsGenCrvRational = pBSC->IsRational() ;

  // locals for upcoming SmBSplineSurface::CreateCanonical() call

  // NewNurbSurface V direction knot vector
  ULONG  alVMData[2];
  double adVData [2];
  SmTArray<ULONG>  sVKnotMult(2,alVMData,2);
  SmTArray<double> sVKnots   (2,adVData, 2);
  sVKnots[0]    = m_vAnalUVDomain.GetMin().y;
  sVKnots[1]    = m_vAnalUVDomain.GetMax().y;
  sVKnotMult[0] = 2;
  sVKnotMult[1] = 2;

  // NewNurbSurface control points
  SmTArray<SmPoint3d> sCtrlPts;
  SmTArray<double>    sWeights;
  SmVector3d          sExtrusionVec[2];
  sExtrusionVec[0] = m_vExtrusionVec * sVKnots[0];
  sExtrusionVec[1] = m_vExtrusionVec * sVKnots[1];
  for (ULONG ii=0; ii<lNumCrvCtrlPts; ii++)
    {
      for (ULONG jj=0; jj<2; jj++)
        {
          SmPoint3d sPnt = sCrvCtrlPts[ii] + sExtrusionVec[jj];
          sCtrlPts.Add(sPnt);
          if (bIsGenCrvRational)
              sWeights.Add(sCrvWeights[ii]);
        }
    }

  // NewNurbSurface UVDomain: UInterval = Curve Interval, VInterval = Extrusion Interval
  SmExtent2d sUVDomain(SmPoint2d(sCrvIvl.GetMin(),sVKnots[0]),
                       SmPoint2d(sCrvIvl.GetMax(),sVKnots[1]));

  // Build NewNurbSurface
  SmBSplineSurface * pTmp = NULL;
  SER(SmBSplineSurface::CreateCanonical(*pContext,
                                        lCrvDeg,
                                        1,
                                        sCtrlPts,
                                        SM_SF_SURF_OF_LINEAR_EXTRUSION,
                                        sCrvKnotMult,
                                        sVKnotMult,
                                        sCrvKnots,
                                        sVKnots,
                                        eKnotType,
                                        &sWeights,
                                        &sUVDomain,
                                        pTmp));

  // remember to swap U with V when m_bSwapUV == TRUE
  if (m_bSwapUV)
    { SER(pTmp->SwapUV()); }

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
  if (bDebugMe)
    {
      smgfx_SetColor(1,0,0); pBSC->DrawWDeriv(sCrvIvl); sm_GraphicsLoop();
      smgfx_SetColor(0,0,1); pTmp->DrawUV(1,1); sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

  // make NewNurbSurface temporary
  SmObjDelete sClean(pTmp);

  // swap NewNurbSurface m_pNurb with this surface old m_pNurb pointer
  SmSurfOfExtrusion *pTmpSurfOfExtru = (SmSurfOfExtrusion*)pTmp;
  gw_SURFACE *pTmpNurb     = m_pNurb;
  m_pNurb                  = pTmp->GetGwNurbPointer();
  pTmpSurfOfExtru->m_pNurb = pTmpNurb;

#ifdef SM_DEBUG_CODE
  if (bDebugMe)
    {
      Dump() ;
    }
#endif // SM_DEBUG_CODE

  // all done
  return SM_SUCCESS;

} // end SmSurfOfExtrusion::MakeNurb

//      /*******************************************************************//**
//      PURPOSE: Swap the U and V parameterizations of a surface.  This
//          effectively reverses the orientation of the surface.
//
//      NOTES:
//      ***********************************************************************/
//      SmStatus SmSurfOfExtrusion::SwapUV
//        ()
//      {
//        SER(SmBSplineSurface::SwapUV());
//        //      SmAxis2Placement sPlace;
//        //      SER(Transform(sPlace, NULL));
//        return SM_SUCCESS;
//
//      } // end SmSurfOfExtrusion::SwapUV

/*******************************************************************//**
PURPOSE: Scale and transform an Extrusion surface.

NOTES:
  Scaling is about the origin.
  Differential scaling is not allowed on analytic surfaces.
***********************************************************************/
SmStatus SmSurfOfExtrusion::Transform
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

  // check input - disallow non-isotropic scaling
  double dScale = 1.0;
  if (   cpOptScale != NULL
      && cpOptScale->DistanceBetweenSquared( SmVector3d(1,1,1) ) > SM_EFF_ZERO_SQ )
    {
      // when scale factors are not all the same
      dScale = cpOptScale->x;
      if (   !SM_ARE_SAME( cpOptScale->y, dScale )
          || !SM_ARE_SAME( cpOptScale->z, dScale ) )
        {
          // return an error
          SER(SM_ERR);
        }
    }

  // transform m_pNurb
  SER( SmBSplineSurface::Transform( crRotateNMove, cpOptScale ));

  // transform the rest of the 3d data
  m_pGenCurve->Transform( crRotateNMove, cpOptScale );

  // Scale first.
  if ( dScale != 1.0 )
    {
      // Scaling points like this implies that the scaling is about the origin:
      m_vOrigin       *= dScale;
      m_vExtrusionVec *= dScale;
      m_vCurvePoint   *= dScale;
      // m_vCurveVec is a unit vector (direction only), so don't scale it.
    }

  crRotateNMove.TransformPoint ( m_vOrigin,       m_vOrigin       );
  crRotateNMove.TransformVector( m_vExtrusionVec, m_vExtrusionVec );
  crRotateNMove.TransformPoint ( m_vCurvePoint,   m_vCurvePoint   );
  crRotateNMove.TransformVector( m_vCurveVec,     m_vCurveVec     );


  // obsolete old method - use sloppy IsNurbSurfaceSurfOfExtrusion and extract above properties
  //
  //      // extract the SmSurfOfExtrusion from the m_pNurbShape
  //      SmSurfOfExtrusion *pExt;
  //      const SmContext *pContext = GetContext();
  //      if (!SmSurfOfExtrusion::IsNurbSurfaceSurfOfExtrusion(*pContext,this,pExt))
  //        {
  //          if (!SmSurfOfExtrusion::IsNurbSurfaceSurfOfExtrusion(*pContext,this,pExt,10.0))
  //            {
  //              if (!SmSurfOfExtrusion::IsNurbSurfaceSurfOfExtrusion(*pContext,this,pExt,100.0))
  //                {
  //                  if (!SmSurfOfExtrusion::IsNurbSurfaceSurfOfExtrusion(*pContext,this,pExt,1000.0))
  //                    {
  //                      SER(SM_ERR); // Something wrong here if scaling a SweptSurface does
  //                                   // not produce a SweptSurface.
  //                    }
  //                }
  //            }
  //        }
  //      SmObjDelete sClean(pExt);
  //
  //      SmBSplineCurve *pBSC = m_pGenCurve;
  //      m_pGenCurve          = pExt->m_pGenCurve;
  //      m_pGenCurve->SetOwner(this) ;
  //      pExt->m_pGenCurve    = pBSC;
  //      pExt->m_pGenCurve->SetOwner(pExt) ;
  //
  //      m_vOrigin = pExt->m_vOrigin;
  //      m_vExtrusionVec = pExt->m_vExtrusionVec;
  //      m_bSwapUV = pExt->m_bSwapUV;
  //      m_vAnalUVDomain = pExt->m_vAnalUVDomain;
  //

  // all done
  return SM_SUCCESS;

} // end SmSurfOfExtrusion::Transform

/*******************************************************************//**
PURPOSE: Trim the Surface of Extrusion with the given domain.

NOTES:
  The input domain is a Nurbs domain, not a STEP Domain.

  This function preserves the input surface geometry and its
  parameterization at the domain corners exactly, however the surface's
  parameterization between domain corners may vary slightly but by
  amounts easily larger than reasonable tolerance sizes.

  As such, existing Edgeuse->UVTrimCurves that reference the surface
  being trimmed should be deleted and rebuilt after this call.
***********************************************************************/
SmStatus SmSurfOfExtrusion::TrimWithDomain
  ( SmExtent2d & rNewNurbUVDomain )          // in : Desired new NurbUVDomain
{
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  if(bDebugMe)
    { SM_DUMP_AND_ASSERT_VALID(this) ; }
#endif // SM_DEBUG_CODE

  // locals
  SmExtent2d sCurrentNurbUVDomain = GetNaturalUVDomain();

  // check input - new trim UV domain must be inside current natural UV domain
  if (!rNewNurbUVDomain.IsContainedBy(sCurrentNurbUVDomain, SM_EFF_ZERO_PARAM))
    { SER(SM_ERR); }

  // no work - new and current domains are same size
  if (sCurrentNurbUVDomain.IsContainedBy(rNewNurbUVDomain, SM_EFF_ZERO_PARAM))
    { return SM_SUCCESS; }

#ifdef SM_DEBUG_CODE

  // copy surface in case we want to compare before/after deformations
  SmSurface *pCopySurface ;
  this->Copy(*GetContext(), pCopySurface) ;
  SmObjDelete sCopyClean(pCopySurface) ;

  // draw
  if(bDebugMe)
    {
      SM_DUMP_AND_ASSERT_VALID(this) ;

      SmFace *pFace = (SmFace *)this->GetFace() ;
      SmBrep *pBrep =   pFace ? pFace->GetBrep() : NULL ;

      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,1) ; this->DrawUV(8,8,FALSE,NULL,TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,0,0) ; if(pFace) pFace->Draw(SM_DM_CROSSHATCH) ; sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

  // Update just the analytic domain and the GenCurve (both analytic and NURB)
  UpdateAnalyticalDomain(rNewNurbUVDomain) ;

  // Now rebuild the Nurbs surface using the updated analytic domain.
  Notify(SM_NO_PRE_EDIT, this, SM_NO_GET_OWNER(this), NULL);
  SER(MakeNurb());
  SER(Reparameterize(rNewNurbUVDomain));
  Notify(SM_NO_POST_EDIT, this, SM_NO_GET_OWNER(this), NULL);

#ifdef SM_DEBUG_CODE
  if(bDebugMe)
    {
      SM_DUMP_AND_ASSERT_VALID(this) ;

      // copy surface in case we want to compare before/after deformations
      SmSolutionArray sSolutions ;
      pCopySurface->GlobalSurfaceSolve(this->GetNaturalUVDomain(),
                                       *this,
                                       this->GetNaturalUVDomain(),
                                       SM_SO_MAXIMIZE, SM_EFF_ZERO, NULL, NULL,
                                       SM_SR_ALL, sSolutions) ;

      rNewNurbUVDomain.Dump() ;
      sSolutions.Dump() ;

      SmFace *pFace = (SmFace *)this->GetFace() ;
      SmBrep *pBrep =   pFace ? pFace->GetBrep() : NULL ;
      SmExtent2d sDom( this->GetNaturalUVDomain() );

      // drop points from this surface to original Copy over This surface domain
      // to see if surfaces are geometrically equivalent
      static constexpr ULONG dSmpCnt = 40 ;
      SmZoneTol3d sZoneTol3d = pFace ? (double)pFace->GetTolerance() : pBrep ? (double)pBrep->GetTolerance() : 0.00001 ;
      SmSrfSrfGapFunction sSrfSrfGap( (SmXSectTol3d)sZoneTol3d,
                                      this, sDom,
                                      pCopySurface, dSmpCnt, dSmpCnt) ;
      sSrfSrfGap.Dump() ;

      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,1) ; this->DrawUV(8,8,FALSE,NULL,TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(4,5, 1,0,0) ; m_pGenCurve->Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,0,0) ; if(pFace) pFace->Draw(SM_DM_CROSSHATCH) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 1,0,0) ; sSrfSrfGap.Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(2,3, 1,0,0) ; sSolutions.Draw() ; sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

  // all done
  return SM_SUCCESS;

} // end SmSurfOfExtrusion::TrimWithDomain

/*******************************************************************//**
PURPOSE: AssertValid() Test reporting static labels
***********************************************************************/
SmAssertReportLabel sAssertSurfOfExtrusion_list[] =
{
 /*  0 */ {SM_AT_VECTOR,    _T("Vector"),              _T("m_vExtrusionVec must be initialized") },
 /*  1 */ {SM_AT_VECTOR,    _T("Vector Length"),       _T("m_vExtrusionVec must be non-zero") },
 /*  2 */ {SM_AT_DOMAIN,    _T("Analytic X"),          _T("m_vAnalUVDomain XLength must be non-zero") },
 /*  3 */ {SM_AT_DOMAIN,    _T("Analytic Y"),          _T("m_vAnalUVDomain YLength must be non-zero") },
 /*  4 */ {SM_AT_DOMAIN,    _T("Domain Range"),        _T("Analytic and Nurb domains must share same range") },
 /*  5 */ {SM_AT_DISTANCE,  _T("Distance between"),    _T("STEP domain point converted to Nurb and back to STEP domain must be the same domain point") },
 /*  6 */ {SM_AT_DISTANCE,  _T("Distance between"),    _T("equivalent STEP and Nurb domain point evaluations must yield same image point values") },
 /*  7 */ {SM_AT_DOMAIN,    _T("Analytic Interval"),   _T("m_vAnalUVDomain->UInterval must be the same as the m_pGenCurve->STEPInterval") },
 /*  8 */ {SM_AT_GEOMETRIC, _T("Origin"),              _T("m_vOrigin is a point on m_pGenCurve") },
 /*  9 */ {SM_AT_DISTANCE,  _T("Planar"),              _T("m_vCurvePoint is on plane") },
 /* 10 */ {SM_AT_ANGLE,     _T("Planar"),              _T("m_vCurveVec is plane normal") },
 /* 11 */ {SM_AT_DISTANCE,  _T("Linear"),              _T("m_vCurvePoint is on line") },
 /* 12 */ {SM_AT_ANGLE,     _T("Linear"),              _T("m_vCurveVec is tangent") },
 /* 13 */ {SM_AT_GEOMETRIC, _T("Linear"),              _T("bLinear && bPlanar") },
 /* 14 */ {SM_AT_GEOMETRIC, _T("Linear"),              _T("Unique sweep") },
 /* 15 */ {SM_AT_GEOMETRIC, _T("Perpendicular"),       _T("!bLinear && bPlanar") },
 /* 16 */ {SM_AT_GEOMETRIC, _T("Perpendicular"),       _T("Orthogonal sweep") },
 /* 17 */ {SM_AT_GEOMETRIC, _T("Independent"),         _T("!bLinear && bPlanar") },
 /* 18 */ {SM_AT_GEOMETRIC, _T("Independent"),         _T("NonPlanar sweep") },
 /* 19 */ {SM_AT_GEOMETRIC, _T("Coplanar"),            _T("!bLinear && bPlanar") },
 /* 20 */ {SM_AT_GEOMETRIC, _T("Coplanar"),            _T("Is coplanar") },
 /* 21 */ {SM_AT_GEOMETRIC, _T("Coplanar"),            _T("GenCurve points of tangency with the extrusion direction") },
 /* 22 */ {SM_AT_GEOMETRIC, _T("NOT_Planar"),          _T("!bLinear && !bPlanar") },
 /* 23 */ {SM_AT_GEOMETRIC, _T("NOT_Planar"),          _T("GenCurve points of tangency with the extrusion direction") },
 /* 24 */ {SM_AT_DOMAIN,    _T("InsideOut GenCurve"),  _T("GenCurve bInsideOut bit value should be FALSE") },
 /* 25 */ {SM_AT_DOMAIN,    _T("GenCurveDomain"),      _T("GenCurve Anal and NURB intervals must match appropriate Surface Domain Intervals") },
 /* 26 */ {SM_AT_POINTER,   _T("Bad GenCurve Owner"),  _T("GenCurve owner must be this SmSurfOfExtrusion object") },
 /* 27 */ {SM_AT_POINTER,   _T("Bad GenCurve Context"),_T("GenCurve context must be this SmSurfOfExtrusion context") }
} ;

/*******************************************************************//**
PURPOSE:

RETURNS ---  TRUE  = OK
             FALSE = Problem
***********************************************************************/
SmBoolean SmSurfOfExtrusion::AssertValid
 (SmAssertArray    * pAList,        // i/o: Accumulating list of failed Asserts, NULL to ignore, default:[NULL]
  SmAssertTestLevel  eTestLevel,    // in : SM_LEVEL_0=fewest tests, SM_LEVEL_1, SM_LEVEL_2=all tests,
                                    //      SM_LEVEL_GIVEN = run tests in order requested in pTestRequests
                                    //      default:[SM_LEVEL_0]
  SmAssertWalking    eWalkTree,     // NotUsed: in : SM_WALK = run AssertValid on any topology graph descendants, SM_NO_WALK=don't, default:[SM_WALK]
  SmTArray<ULONG>  * pTestRequests) // in : when eTestLevel == SM_LEVEL_GIVEN, run these tests in this order, default:[NULL]
 const
{
  SM_REF1(eWalkTree) ;
  // init rtn value
  SmBoolean bRtn = TRUE ;

  // call the base class AssertValid
  if(m_pNurb)
    {
      bRtn &= (  (eTestLevel != SM_LEVEL_GIVEN)
               ? SmBSplineSurface::AssertValid(pAList, eTestLevel, SM_NO_WALK, pTestRequests)
               : TRUE ) ;
    }

  // 0. m_vExtrusionVec is initialized and nonZero length
  bRtn &= SM_ASSERT_BOOLEAN_REPORT(0, SM_LEVEL_0, (m_vExtrusionVec.IsUndef() == FALSE), _T("") ) ;
  bRtn &= SM_ASSERT_VALUE_REPORT  (1, SM_LEVEL_0, (m_vExtrusionVec.Length() > SM_EFF_ZERO), SM_EFF_ZERO, m_vExtrusionVec.Length(), _T("") ) ;

  // 1. AnalUVDomain Check - it has a nonZero area
  bRtn &= SM_ASSERT_VALUE_REPORT(2, SM_LEVEL_0, (m_vAnalUVDomain.XLength()  > SM_EFF_ZERO), SM_EFF_ZERO, m_vAnalUVDomain.XLength(), _T("") ) ;
  bRtn &= SM_ASSERT_VALUE_REPORT(3, SM_LEVEL_0, (m_vAnalUVDomain.YLength()  > SM_EFF_ZERO), SM_EFF_ZERO, m_vAnalUVDomain.YLength(), _T("") ) ;

  // 2. Domain consistency
  //    a. Check AnalUVDomain to NurbUVDomain consistency - There is nothing special to check here
  //    b. GenCurve->GetInsideOut == FALSE since SurfOfExtrusion does not support bInsideOut
  //    c. GenCurve->GetNaturalInterval == m_bSwapUV ? sNaturalUVDomain.GetVInterval : sNaturalUVDomain.GetUInterval ;
  //       GenCurve->GetSTEPInterval    == AnalUVDomain.GetUInterval
  if(m_pGenCurve)
    {
      // 2.b GenCurve InsideOut bit (if it has one) must be FALSE, because SurfOfExtrusion does not support InsideOut bits
      bRtn &= SM_ASSERT_BOOLEAN_REPORT(24, SM_LEVEL_0, (GetGenCurve()->GetInsideOut() == FALSE), _T("")) ;
      if(m_pNurb)
        {
          // 2.c GenCurve must have same parameterization as Surface in GenCurve direction
          //     GenCurve AnalInterval   must equal Surface->AnalUVDomain->GetUInterval,
          //     GenCurve NurbInterval   must equal m_bSwapUV ? Surface->NURBVInterval : Surface->NURBUInterval
          //     GenCurve NURBKnotVector must equal m_bSwapUV ? Surface->NURBKnotV : Surface->NURBKnotU
          SmBoolean bHasSameParam = HasSameParameterization(SM_SP_U,        // in : Analytic Domain Direction
                                                             m_bSwapUV      // in : Nurb     Domain Direction
                                                           ? SM_SP_V
                                                           : SM_SP_U,
                                                           m_pGenCurve) ;    // in : The GenCurve

          /* 25 */
          bRtn &= SM_ASSERT_VALUE_REPORT(25, SM_LEVEL_0, (bHasSameParam), SM_EFF_ZERO, SM_UNDEF_DOUBLE, _T("") ) ;

        } // end has a Nurb description check

      /* 26 */ // GenCurve owner must be this SmSurfOfExtrusion object
      bRtn &= SM_ASSERT_BOOLEAN_REPORT(26, SM_LEVEL_0, (m_pGenCurve->GetOwner() == this), _T("")) ;

      /* 27 */ // GenCurve context must be this SmSurfOfExtrusion context
      bRtn &= SM_ASSERT_BOOLEAN_REPORT(27, SM_LEVEL_0, (m_pGenCurve->GetContext() == GetContext()), _T("")) ;
    } // end has a GenCurve check

  // check that STEP and Nurb representations are the same
  double dSpanU = m_vAnalUVDomain.GetUInterval().GetLength() ;
  double dSpanV = m_vAnalUVDomain.GetVInterval().GetLength() ;
  double dTolU = SM_EFF_ZERO * (1.0 + dSpanU) ;
  double dTolV = SM_EFF_ZERO * (1.0 + dSpanV) ;

  if(m_pNurb)
    {
      SmPoint2d sSTEPUV, sNURBUV, sTestSTEPUV ;
      SmPoint3d sSTEPPoint, sNURBPoint ;
      ULONG lCnt = 3 ;
      for(ULONG ii=0;ii<=lCnt;ii++)
        {
          for(ULONG jj=0;jj<=lCnt;jj++)                                        {
              // check UV conversions

              sSTEPUV = m_vAnalUVDomain.Evaluate(((double)ii)/((double)lCnt),
                                                 ((double)jj)/((double)lCnt)) ;

              ConvertUVFromSTEPToNURBS(sSTEPUV, sNURBUV) ;
              ConvertUVFromNURBSToSTEP(sNURBUV, sTestSTEPUV) ;

              SmBoolean bGoodU = SM_IS_ZERO_TO_TOL(sSTEPUV.x - sTestSTEPUV.x, dTolU) ;
              SmBoolean bGoodV = SM_IS_ZERO_TO_TOL(sSTEPUV.y - sTestSTEPUV.y, dTolV) ;
              if(!bGoodU || !bGoodV)
                {
                  bRtn &= SM_ASSERT_VALUE_REPORT(5, SM_LEVEL_0, bGoodU, dTolU, sSTEPUV.x - sTestSTEPUV.x, _T("") ) ;
                  bRtn &= SM_ASSERT_VALUE_REPORT(5, SM_LEVEL_0, bGoodV, dTolV, sSTEPUV.y - sTestSTEPUV.y, _T("") ) ;
                }

              // check equivalent evaluations
              EvaluateSTEPPoint(sSTEPUV, sSTEPPoint) ;
              EvaluatePoint(sNURBUV, sNURBPoint) ;
              bRtn &= SM_ASSERT_VALUE_REPORT(6, SM_LEVEL_0, (SM_IS_ZERO_TO_TOL(sSTEPPoint.DistanceBetween(sNURBPoint), SM_EFF_ZERO_SQRT)), SM_EFF_ZERO_SQRT, sSTEPPoint.DistanceBetween(sNURBPoint), _T("") ) ;

            } // end iter every V Sample point
        } // end iter every U sample point

    } // end equivalent STEP and Nurb representations

  // 3. check that m_vAnalUVDomain->UInterval is defined by the genCurve analytic interval
  bRtn &= SM_ASSERT_VALUE_REPORT(7, SM_LEVEL_0, (m_vAnalUVDomain.GetUInterval().AreEqual(m_pGenCurve->GetSTEPInterval(),dTolU)), dTolU, SM_UNDEF_DOUBLE, _T("") ) ;

  // 4. check that m_vOrigin is a point on m_pGenCurve
  SmSolutionArray sSolutions ;
  m_pGenCurve->GlobalPointSolve(m_pGenCurve->GetNaturalInterval(),
                                SM_SO_INTERSECT,
                                m_vOrigin,
                                SM_EFF_ZERO,
                                NULL, NULL,
                                SM_SR_ALL,
                                sSolutions) ;

  bRtn &= SM_ASSERT_BOOLEAN_REPORT(8, SM_LEVEL_0, (sSolutions.GetSize() > 0), _T("") ) ;

  // 5. Check cached m_vCurvePoint/m_vCurveVec information
  double   dAngleRad,  dDist, dNotUsed = 0.0 ;
  SmVector3d sLinePoint, sLineVec, sPlanePoint, sPlaneNormal, sPnt ;
  SE(m_pGenCurve->EvaluatePoint(m_pGenCurve->GetNaturalInterval().Evaluate(0.45678),sPnt));
  double   dTol = ANALYTIC_TOL_SCALE * SM_EFF_ZERO * (1.0 + m_vOrigin.GetMaxDimension() + sPnt.GetMaxDimension());

  SmBoolean bLinear = m_pGenCurve->IsLinear(dTol, &sLinePoint,  &sLineVec) ;
  SmBoolean bPlanar = m_pGenCurve->IsPlanar(dTol, &sPlanePoint, &sPlaneNormal) ;
  if(bLinear)
    {
      // m_vCurvePoint = Point on line
      // m_vCurveVec   = Line Tangent
      smgu_LinePointDistance(sLinePoint, sLineVec, m_vCurvePoint, dDist) ;
      sLineVec.AngleBetween(m_vCurveVec, dAngleRad) ;

      bRtn &= SM_ASSERT_VALUE_REPORT(11, SM_LEVEL_0, (   SM_IS_ZERO_TO_TOL(dDist, SM_EFF_ZERO)), SM_EFF_ZERO, dDist, _T("") ) ;
      bRtn &= SM_ASSERT_VALUE_REPORT(12, SM_LEVEL_0, (   SM_IS_ZERO_TO_TOL(dAngleRad, SM_EFF_ZERO_RAD )
                                                      || SM_IS_ZERO_TO_TOL(dAngleRad - SM_PI, SM_EFF_ZERO_RAD)),
                                     SM_EFF_ZERO_RAD, smos_Min(smos_Fabs(dAngleRad), smos_Fabs(dAngleRad - SM_PI)), _T("") ) ;
    } // end linear stale cache check

  else if(bPlanar)
    {
      // m_vCurvePoint = Point on Plane
      // m_vCurveVec   = Plane Normal
      smgu_PlanePointDistance(sPlanePoint, sPlaneNormal, m_vCurvePoint, dDist) ;
      sPlaneNormal.AngleBetween(m_vCurveVec, dAngleRad) ;

      bRtn &= SM_ASSERT_VALUE_REPORT(9, SM_LEVEL_0, (SM_IS_ZERO_TO_TOL(dDist, SM_EFF_ZERO)), SM_EFF_ZERO, dDist, _T("") ) ;

      bRtn &= SM_ASSERT_VALUE_REPORT(10, SM_LEVEL_0, (   SM_IS_ZERO_TO_TOL(dAngleRad, SM_EFF_ZERO_RAD)
                                                      || SM_IS_ZERO_TO_TOL(dAngleRad - SM_PI, SM_EFF_ZERO_RAD)),
                                     SM_EFF_ZERO_RAD, smos_Min(smos_Fabs(dAngleRad), smos_Fabs(dAngleRad - SM_PI)), _T("") ) ;

    } // end planar stale cache check



  // 6. check that extrusion is valid in that every surface point
  //    is unique, i.e. no 2 points on the generator curve are extruded
  //    through the same point in space.
  //
  //    Method: A GenCurve with any tangent vectors along its length
  //            that are in the same direction as the sweep vector
  //            is a failing surface.
  switch(m_eOrientation)
    {
      case SM_CO_LINEAR        : sLineVec.AngleBetween(m_vExtrusionVec, dAngleRad) ;
                                 bRtn &= SM_ASSERT_VALUE_REPORT(13, SM_LEVEL_0, (bLinear && bPlanar), SM_EFF_ZERO, SM_UNDEF_DOUBLE, _T("")) ;        // geometry description check
                                 bRtn &= SM_ASSERT_VALUE_REPORT(14, SM_LEVEL_0,
                                                                (   !SM_IS_ZERO_TO_TOL(dAngleRad, SM_EFF_ZERO_RAD)
                                                                 && !SM_IS_ZERO_TO_TOL(dAngleRad-SM_PI, SM_EFF_ZERO_RAD)),
                                                                 SM_EFF_ZERO_RAD,
                                                                 smos_Min(smos_Fabs(dAngleRad),smos_Fabs(dAngleRad-SM_PI)), _T("") ) ;  // unique sweep check
                                 break ;

      case SM_CO_PERPENDICULAR : m_vExtrusionVec.AngleBetween(sPlaneNormal, dAngleRad) ;
                                 bRtn &= SM_ASSERT_VALUE_REPORT(15, SM_LEVEL_0, (!bLinear && bPlanar), SM_EFF_ZERO, SM_UNDEF_DOUBLE, _T("")) ;    // geometry description check
                                 bRtn &= SM_ASSERT_VALUE_REPORT(16, SM_LEVEL_0,
                                                                (   SM_IS_ZERO_TO_TOL(dAngleRad, SM_EFF_ZERO_RAD)
                                                                 || SM_IS_ZERO_TO_TOL(dAngleRad-SM_PI, SM_EFF_ZERO_RAD)),
                                                                 SM_EFF_ZERO_RAD, smos_Min(smos_Fabs(dAngleRad),smos_Fabs(dAngleRad-SM_PI)), _T("") ) ; // orthogonal sweep check
                                 break ;

      case SM_CO_INDEPENDENT   : m_vExtrusionVec.AngleBetween(sPlaneNormal, dAngleRad) ;
                                 bRtn &= SM_ASSERT_VALUE_REPORT(17, SM_LEVEL_0, (!bLinear && bPlanar), SM_EFF_ZERO, SM_UNDEF_DOUBLE, _T("")) ;    // geometry description check
                                 bRtn &= SM_ASSERT_VALUE_REPORT(18, SM_LEVEL_0, (!SM_IS_ZERO_TO_TOL(dAngleRad-SM_PI/2.0, SM_EFF_ZERO_RAD)), SM_EFF_ZERO_RAD,dAngleRad-SM_PI/2.0, _T("")) ;  // nonPlanar sweep check
                                 break ;

      case SM_CO_COPLANAR      : m_vExtrusionVec.AngleBetween(sPlaneNormal, dAngleRad) ;
                                 bRtn &= SM_ASSERT_VALUE_REPORT(19, SM_LEVEL_0, (!bLinear && bPlanar), SM_EFF_ZERO, SM_UNDEF_DOUBLE, _T("")) ;    // geometry description check
                                 bRtn &= SM_ASSERT_VALUE_REPORT(20, SM_LEVEL_0, (SM_IS_ZERO_TO_TOL(dAngleRad-SM_PI/2.0, SM_EFF_ZERO_RAD)), SM_EFF_ZERO_RAD, dAngleRad-SM_PI/2.0, _T("") ) ; // CoPlanar check

                                 // look for points of tangency on GenCurve with the extrusion Vec direction
                                 m_pGenCurve->GlobalPropertyAnalysis(m_pGenCurve->GetNaturalInterval(),
                                                                     SM_CP_PARALLEL_TO_VECTOR,
                                                                     &dNotUsed,
                                                                     &m_vExtrusionVec,
                                                                     dTol,
                                                                     sSolutions) ;

                                 bRtn &= SM_ASSERT_BOOLEAN_REPORT(21, SM_LEVEL_0, (sSolutions.GetSize() == 0), _T("") ) ;
                                 break ;

      case SM_CO_NOT_PLANAR    : bRtn &= SM_ASSERT_VALUE_REPORT(22, SM_LEVEL_0, (!bLinear && !bPlanar), SM_EFF_ZERO, SM_UNDEF_DOUBLE, _T("") ) ; // geometry description check

                                 // look for points of tangency on GenCurve with the extrusion Vec direction
                                 m_pGenCurve->GlobalPropertyAnalysis(m_pGenCurve->GetNaturalInterval(),
                                                                     SM_CP_PARALLEL_TO_VECTOR,
                                                                     &dNotUsed,
                                                                     &m_vExtrusionVec,
                                                                     dTol,
                                                                     sSolutions) ;

                                 bRtn &= SM_ASSERT_BOOLEAN_REPORT(23, SM_LEVEL_0, (sSolutions.GetSize() == 0), _T("") ) ;

                                 // for a complete check we should check to see if any one curve point
                                 // can be swept across some other curve point.
                                 // this need a little work to complet and so is left to another time.
                                 //  But the following outlines my idea for this.
                                 //  1. Drop Curve to plane perpendicular to extrusion vector. (this is a tolerance operation)
                                 //  2. Find all self intersections of the dropped curve.
                                 //     TODO: to avoid tolerance problems introduced by DropCurve
                                 //           Generalize GlobalCurveSelfIntersect() to work not
                                 //           with the curve but the projection of the curve to a common plane.
                                 //  3. For every self intersection point pair check to see that
                                 //     the 3d distance between the associated points back
                                 //     on the original 3d curve is less than the sweep distance.
                                 break ;
      case SM_CO_UNDEFINED:      break;
    } // end switch on m_eOrientation


#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  if(bDebugMe)
    {
      Dump() ;

      SmFace *pFace = (SmFace *)GetFace() ;
      SmBrep *pBrep = pFace ? pFace->GetBrep() : NULL ;
      SmPoint3d sStartPoint = m_vOrigin + m_vAnalUVDomain.GetVMin() * m_vExtrusionVec ;
      SmPoint3d sEndPoint   = m_vOrigin + m_vAnalUVDomain.GetVMax() * m_vExtrusionVec ;

      smgfx_Erase() ;
      smgfx_SetLook(2,3, 1,0,0)  ; m_pGenCurve->Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,1)  ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,0)  ; this->DrawUV(3,3) ; sm_GraphicsLoop() ;
      smgfx_SetLook(3,4, 0,0,1)  ; m_vOrigin.Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook(2,4, 0,.5,1) ; m_vExtrusionVec.Draw(&m_vOrigin) ; sm_GraphicsLoop() ;
      smgfx_SetLook(3,4, 0,1,.5) ; sStartPoint.DrawPointToPoint(sEndPoint) ; sm_GraphicsLoop() ;
      smgfx_SetLook(2,4, .5,0,1) ; m_vCurvePoint.Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook(2,4, .5,.5,1); m_vCurveVec.Draw(&m_vCurvePoint) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,0,0)  ; if(pFace) pFace->Draw(SM_DM_CROSSHATCH) ; sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;

    }
#endif // SM_DEBUG_CODE

#ifdef GWC
  if(bRtn == FALSE)
    {
      WARN(_T("Bad SmSurfOfExtrusion")) ;
    }
#endif // GWC

  // all done
  // SM_ASSERT(bRtn) ;
  return(bRtn) ;

} // end SmSurfOfExtrusion::AssertValid

// obsolete
// /*******************************************************************//**
// PURPOSE: Fix Assert Report failures reported by an AssertValid call
// 
// RETURNS ---  TRUE  = OK
//              FALSE = Problem
// ***********************************************************************/
// SmBoolean SmSurfOfExtrusion::AssertHeal
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
//       return ( SmBSplineSurface::AssertHeal(rAReport, pAList) ) ;
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
//                rAReport.m_pHealMessage = _T("SmSurfOfExtrusion::AssertHeal fix not yet supported") ;
// 
//     }
// 
//   // all done
//   return(bRtn) ;
// 
// } // end SmSurfOfExtrusion::AssertHeal
// end obsolete

/*******************************************************************//**
PURPOSE: Compute the total size of the memory used by the SmSurfOfExtrusion.

NOTES: Does not add in attribute memory
***********************************************************************/
ULONG SmSurfOfExtrusion::GetMemoryUsed    // rtn: smaller size of actually used memory in bytes
  (ULONG    & rlMemoryAllocated,          // out: bigger size of all allocated memory in bytes
   SmMarkType eMarkType)                  // in : uses without increment eMarkType value
  const
{
  // in case this method is called directly - get a mark for attribute memory usage
  SmNewMarkAndLock sMarkLock ;
  if(eMarkType == SM_MT_NOMARK)
    {
      eMarkType = sMarkLock.SetContext((SmContext *)GetContext()) ;
    }

  // this + m_pNurb memory
  ULONG lUsed       =   sizeof(*this)
                      + sm_ComputeNurbSurfaceSize(m_pNurb) ;
  rlMemoryAllocated = lUsed ;

  // curve memory
  ULONG lThisAllocated = 0 ;
  lUsed             += m_pGenCurve ? m_pGenCurve->GetMemoryUsed(lThisAllocated, eMarkType) : 0 ;
  rlMemoryAllocated += lThisAllocated ;

  // + attribute memory
  lUsed += rlMemoryAllocated + this->GetAttributeMemoryUsed(lThisAllocated,
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

} // end SmSurfOfExtrusion::GetMemoryUsed

/*******************************************************************//**
PURPOSE: Write SmSurfOfExtrusion to given output stream.

NOTES:
***********************************************************************/
SmStatus SmSurfOfExtrusion::WriteToDB
 (SmDatabaseIO & rDB,                // in : target output stream
  ULONG          lDBVersionNumber)   // in : database version to get proper sequence of writes
 const
{
  // file type, ASCII or BINARY
  SmFileType      eType    =  rDB.GetFileType();
  std::ostream  & rFileOut = *rDB.GetOutStreamPtr();

  // locals
  if (eType == SM_ASCII)
    {
      rFileOut << m_vOrigin.x       << " " << m_vOrigin.y       << " " << m_vOrigin.z        << " SmSurfOfExtrusion Origin \n";
      rFileOut << m_vExtrusionVec.x << " " << m_vExtrusionVec.y << " " << m_vExtrusionVec.z  << " SmSurfOfExtrusion Extrusion Vector \n";
      rFileOut <<        m_vAnalUVDomain.GetMin().x
               << " " << m_vAnalUVDomain.GetMin().y
               << " " << m_vAnalUVDomain.GetMax().x
               << " " << m_vAnalUVDomain.GetMax().y                                          << " SmSurfOfExtrusion AnalUVDomain \n";
      rFileOut << m_eOrientation                                                             << " SmSurfOfExtrusion Curve Orientation Type \n";
      rFileOut << m_bSwapUV                                                                  << " SmSurfOfExtrusion SwapUV Boolean \n";
      rFileOut << m_vCurvePoint.x   << " " << m_vCurvePoint.y   << " " << m_vCurvePoint.z    << " SmSurfOfExtrusion When GenCurve is planar or linear - point on plane or line\n";
      rFileOut << m_vCurveVec.x     << " " << m_vCurveVec.y     << " " << m_vCurveVec.z      << " SmSurfOfExtrusion When GenCurve is planar or linear - plane normal or line vec \n";
    }
  else
    {
      SER(rDB.WriteDouble(m_vOrigin.x));
      SER(rDB.WriteDouble(m_vOrigin.y));
      SER(rDB.WriteDouble(m_vOrigin.z));

      SER(rDB.WriteDouble(m_vExtrusionVec.x));
      SER(rDB.WriteDouble(m_vExtrusionVec.y));
      SER(rDB.WriteDouble(m_vExtrusionVec.z));

      SER(rDB.WriteDouble(m_vAnalUVDomain.GetMin().x));
      SER(rDB.WriteDouble(m_vAnalUVDomain.GetMin().y));
      SER(rDB.WriteDouble(m_vAnalUVDomain.GetMax().x));
      SER(rDB.WriteDouble(m_vAnalUVDomain.GetMax().y));

      SER(rDB.WriteLong(m_eOrientation));

      SER(rDB.WriteBoolean(m_bSwapUV));

      SER(rDB.WriteDouble(m_vCurvePoint.x));
      SER(rDB.WriteDouble(m_vCurvePoint.y));
      SER(rDB.WriteDouble(m_vCurvePoint.z));

      SER(rDB.WriteDouble(m_vCurveVec.x));
      SER(rDB.WriteDouble(m_vCurveVec.y));
      SER(rDB.WriteDouble(m_vCurveVec.z));
    }

  // GenCurve
  ULONG lGenCurveDim = m_pGenCurve->GetDim() ;
  if (eType == SM_ASCII) { rFileOut << " SmSurfOfExtrusion->GenCurve \n"; }
  SER(rDB.WriteType(m_pGenCurve->GetType(), &lGenCurveDim)) ;
  SER(m_pGenCurve->WriteToDB(rDB, lDBVersionNumber)) ;

  // output the parent
  SmBSplineSurface::WriteToDB(rDB, lDBVersionNumber) ;

  // all done
  return SM_SUCCESS;

} // end SmSurfOfExtrusion::WriteToDB

/*******************************************************************//**
PURPOSE: static method to Read a SmSurfOfExtrusion from a given stream

NOTES:
***********************************************************************/
SmStatus SmSurfOfExtrusion::ReadFromDB
 (SM_TYPE           lType,              // NotUsed: in : curve type to be read
  SmDatabaseIO    & rDB,                // in : target output stream
  const SmContext & crContext,          // in : context for new object construction
  SmSurface      *& rpNewSurface,       // out:    NULL on input = new object allocated in this routine built from stream data
                                        //      NotNULL on input = pointer to an empty object to be filled by this routine
  ULONG             lDBVersionNumber)   // in : database version to get proper sequence of writes
{
  SM_REF1(lType) ;
  // check input
  SER(  (   rpNewSurface == NULL
         || rpNewSurface->IsKindOf(SmSurfOfExtrusion_TYPE)) ? SM_SUCCESS : SM_ERR) ;

  // init output object
  SmSurfOfExtrusion *pSurfOfExtrusion =   (rpNewSurface == NULL)
                                        ? new (crContext) SmSurfOfExtrusion()
                                        : (SmSurfOfExtrusion *)rpNewSurface ;

  // file type
  SmFileType     eType   =  rDB.GetFileType();
  std::istream & rFileIn = *rDB.GetInStreamPtr();

  // locals
  SmPoint3d              sOrig ;
  SmVector3d             sExtrusionVec ;
  SmPoint2d              sAnalMin, sAnalMax ;
  ULONG                  eOrientation ;
  SmBoolean              bSwapUV ;
  SmPoint3d              sCurvePoint ;
  SmVector3d             sCurveVec ;

  if (eType == SM_ASCII)
    {
      rFileIn >> sOrig.x >> sOrig.y >> sOrig.z ;                             rDB.GoToNextLine() ;
      rFileIn >> sExtrusionVec.x >> sExtrusionVec.y >> sExtrusionVec.z ;     rDB.GoToNextLine() ;

      rFileIn >> sAnalMin.x >> sAnalMin.y >> sAnalMax.x >> sAnalMax.y  ;     rDB.GoToNextLine() ;

      rFileIn >> eOrientation ;                                              rDB.GoToNextLine() ;
      rFileIn >> bSwapUV ;                                                   rDB.GoToNextLine() ;

      rFileIn >> sCurvePoint.x >> sCurvePoint.y >> sCurvePoint.z ;           rDB.GoToNextLine() ;
      rFileIn >> sCurveVec.x >> sCurveVec.y >> sCurveVec.z ;                 rDB.GoToNextLine() ;

    }
  else
    {
      SER(rDB.ReadDouble(sOrig.x)) ;
      SER(rDB.ReadDouble(sOrig.y)) ;
      SER(rDB.ReadDouble(sOrig.z)) ;

      SER(rDB.ReadDouble(sExtrusionVec.x)) ;
      SER(rDB.ReadDouble(sExtrusionVec.y)) ;
      SER(rDB.ReadDouble(sExtrusionVec.z)) ;

      SER(rDB.ReadDouble(sAnalMin.x)) ;
      SER(rDB.ReadDouble(sAnalMin.y)) ;
      SER(rDB.ReadDouble(sAnalMax.x)) ;
      SER(rDB.ReadDouble(sAnalMax.y)) ;

      SER(rDB.ReadLong(eOrientation)) ;

      SER(rDB.ReadBoolean(bSwapUV)) ;

      SER(rDB.ReadDouble(sCurvePoint.x)) ;
      SER(rDB.ReadDouble(sCurvePoint.y)) ;
      SER(rDB.ReadDouble(sCurvePoint.z)) ;

      SER(rDB.ReadDouble(sCurveVec.x)) ;
      SER(rDB.ReadDouble(sCurveVec.y)) ;
      SER(rDB.ReadDouble(sCurveVec.z)) ;
    }

  // curve locals
  SmCurve *pGenCurve=NULL ;
  SM_TYPE  lGenCurveType ;
  ULONG    lGenCurveDim ;

  // base curve
  if (eType == SM_ASCII) { rDB.GoToNextLine() ; }
  SER(rDB.ReadType(lGenCurveType, &lGenCurveDim)) ;
  SER(SmCurve::ReadFromDB(lGenCurveType, rDB, lGenCurveDim, crContext, pGenCurve, lDBVersionNumber)) ;
  SM_ASSERT_MSG(pGenCurve->IsKindOf(SmBSplineCurve_TYPE), _T("SmSurfOfExtrusion::ReadFromDB found a GenCurve which is not a kind of SmBSplineCurve Type"))

  // load the analytic
  pSurfOfExtrusion->m_pGenCurve     = (SmBSplineCurve *)pGenCurve ;
  pSurfOfExtrusion->m_pGenCurve->SetOwner(pSurfOfExtrusion) ;
  pSurfOfExtrusion->m_vOrigin       = sOrig ;
  pSurfOfExtrusion->m_vExtrusionVec = sExtrusionVec ;
  pSurfOfExtrusion->m_vAnalUVDomain.SetMinMax(sAnalMin, sAnalMax) ;
  pSurfOfExtrusion->m_eOrientation  = (SmCurveOrientType) eOrientation ;
  pSurfOfExtrusion->m_bSwapUV       = bSwapUV ;
  pSurfOfExtrusion->SetCurvePoint(sCurvePoint) ;
  pSurfOfExtrusion->SetCurveVec(sCurveVec) ;

  // read the parent object
  SmSurface *pSurface = pSurfOfExtrusion ;
  SER(SmBSplineSurface::ReadFromDB(SmBSplineSurface_TYPE, rDB, crContext, pSurface, lDBVersionNumber)) ;

  // all done
  rpNewSurface = pSurfOfExtrusion ;
  return SM_SUCCESS;

} // end SmSurfOfExtrusion::ReadFromDB

/*******************************************************************//**
PURPOSE:
NOTES:
***********************************************************************/
SmBoolean SmSurfOfExtrusion::IsKindOf( SM_TYPE t ) const
{
  return ((SmSurfOfExtrusion_TYPE == t) ? TRUE : SmBSplineSurface::IsKindOf( (t) ));
}

/*******************************************************************//**
PURPOSE: Dump SurfOfExtrusion surface data out for debugging.

NOTES:
***********************************************************************/
void SmSurfOfExtrusion::Dump
  (void)
 const
{
  TCHAR sBuff[SM_TBLOCK_SIZE], sBuffForFile[SM_TBLOCK_SIZE];
  TCHAR sGenBuff[SM_TBLOCK_SIZE] ;
  smos_WriteBuffer(_T("\nBegin SmSurfOfExtrusion::Dump()")) ;

  // GenCurve type/insideOut value label for header info
  if     (    m_pGenCurve == NULL)                   { smos_sprintf(sGenBuff, _T("%s"), _T("No GenCurve")) ; }
  else if(   !m_pGenCurve->IsAnalytic())             { smos_sprintf(sGenBuff, _T("%s GenCurve Not Analytic"), m_pGenCurve->GetTypeString()) ; }
  else if(   !m_pGenCurve->IsKindOf(SmEllipse_TYPE)
          && !m_pGenCurve->IsKindOf(SmLine_TYPE))    { smos_sprintf(sGenBuff, _T("%s GenCurve has no m_bInsideOut Value"), m_pGenCurve->GetTypeString()) ; }
  else if(    m_pGenCurve->IsKindOf(SmEllipse_TYPE)) { SmEllipse *pEllipse = (SmEllipse *)m_pGenCurve ;
                                                       smos_sprintf(sGenBuff, _T("%s GenCurve m_bInsideOut = %1d"), m_pGenCurve->GetTypeString(), pEllipse->GetInsideOut()) ;
                                                     }
  else if(    m_pGenCurve->IsKindOf(SmLine_TYPE))    { SmLine *pLine = (SmLine *)m_pGenCurve ;
                                                       smos_sprintf(sGenBuff, _T("%s GenCurve m_bInsideOut = %1d"), m_pGenCurve->GetTypeString(), pLine->GetInsideOut()) ;
                                                     }
  // Domain locals (analytic, Nurb, GenCurve Analytic and GenCurve Nurb)
  SmExtent2d sAnalyticDomain,  sNaturalUVDomain ;
  SmExtent1d sGenSTEPInterval, sGenNaturalInterval ;
  SmExtent1d sNaturalMatchIvl, sAnalyticMatchIvl ;

  SmTArray<double> sNaturalGenMatchKnots, sGenKnots ;
  SmTArray<ULONG>  sNaturalGenMatchMults, sGenMults ;

  sAnalyticDomain        =  m_vAnalUVDomain ;
  sAnalyticMatchIvl      =  m_vAnalUVDomain.GetUInterval() ;

  SmBoolean bHasNurb     = (GetGwNurbPointer() != NULL) ;
  SmBoolean bHasGenCurve = (GetGenCurve()      != NULL) ;
  SmBoolean bSameGenKnots = false ;

  if(bHasNurb)     { sNaturalUVDomain = GetNaturalUVDomain() ;
                     sNaturalMatchIvl =   m_bSwapUV
                                        ? sNaturalUVDomain.GetVInterval()
                                        : sNaturalUVDomain.GetUInterval() ;
                     if(m_bSwapUV) { GetKnots(SM_SP_V, sNaturalGenMatchKnots,   &sNaturalGenMatchMults) ; }
                     else          { GetKnots(SM_SP_U, sNaturalGenMatchKnots,   &sNaturalGenMatchMults) ; }
                   }
  if(bHasGenCurve) { sGenSTEPInterval    = GetGenCurve()->GetSTEPInterval() ;
                     sGenNaturalInterval = GetGenCurve()->GetNaturalInterval() ;
                     GetGenCurve()->GetKnots(sGenKnots, &sGenMults) ;
                     bSameGenKnots =    SM_ARE_ARRAYS_SAME(sGenKnots, sNaturalGenMatchKnots)
                                     && SM_ARE_ARRAYS_SAME(sGenMults, sNaturalGenMatchMults) ;
                   }

  // report Cache data
  SmSurface::Dump(FALSE) ;

  // header info
  smos_sprintf(sBuff,       _T("\nSmSurfOfExtrusion = 0x%p, Orientation = %.256s, Swap UV = %1d, %.256s"),
      this,
        m_eOrientation == SM_CO_NOT_PLANAR    ? _T("SM_CO_NOT_PLANAR   ")
      : m_eOrientation == SM_CO_PERPENDICULAR ? _T("SM_CO_PERPENDICULAR")
      : m_eOrientation == SM_CO_INDEPENDENT   ? _T("SM_CO_INDEPENDENT  ")
      : m_eOrientation == SM_CO_COPLANAR      ? _T("SM_CO_COPLANAR     ")
      : m_eOrientation == SM_CO_LINEAR        ? _T("SM_CO_LINEAR       ")
      : _T("UNDEFINED"),
      m_bSwapUV,
      sGenBuff);

  smos_sprintf(sBuffForFile,_T("\nSmSurfOfExtrusion = %.256s, Orientation = %.256s, Swap UV = %1d, %.256s"),
      _T("notNULL"),
        m_eOrientation == SM_CO_NOT_PLANAR    ? _T("SM_CO_NOT_PLANAR   ")
      : m_eOrientation == SM_CO_PERPENDICULAR ? _T("SM_CO_PERPENDICULAR")
      : m_eOrientation == SM_CO_INDEPENDENT   ? _T("SM_CO_INDEPENDENT  ")
      : m_eOrientation == SM_CO_COPLANAR      ? _T("SM_CO_COPLANAR     ")
      : m_eOrientation == SM_CO_LINEAR        ? _T("SM_CO_LINEAR       ")
      : _T("UNDEFINED"),
      m_bSwapUV,
      sGenBuff);
  smos_WriteBuffer(sBuff, sBuffForFile);

  // domains
  smos_WriteBuffer(_T("\n  SmSurfOfExtrusion Domains")) ;
  smos_WriteBuffer(_T("\n    m_vAnalUVDomain: ")) ; m_vAnalUVDomain.Dump() ;
  if(bHasNurb)     { smos_WriteBuffer(_T("    m_pNurb Domain : ")) ; sNaturalUVDomain.Dump() ; }
  else             { smos_WriteBuffer(_T("    m_pNurb Domain : Undefined - No underlying m_pNurb Surface\n")) ; }
  if(bHasGenCurve) { smos_WriteBuffer(_T("    GenCurve Anal  : ")) ; sGenSTEPInterval.Dump() ; }
  else             { smos_WriteBuffer(_T("    GenCurve Anal  : Undefined - No underlying m_pGenCurve Object")) ; }
  if(bHasGenCurve) { smos_WriteBuffer(_T("\n    GenCurve Nurb  : ")) ; sGenNaturalInterval.Dump() ; }
  else             { smos_WriteBuffer(_T("\n    GenCurve Nurb  : Undefined - No underlying m_pGenCurve Object")) ; }

  // domain rule compliance
  smos_WriteBuffer(_T("\n    3 SmSurfOfExtrusion Domain compatibility rules:")) ;

  smos_WriteBuffer(_T("\n      Rule 1. GenCurve->AnalInterval  == AnalUVDomain.GetUInterval")) ;
  if(bHasGenCurve) { smos_WriteBuffer(_T("\n        GenCurve Anal  : ")) ; sGenSTEPInterval.Dump() ; }
  else             { smos_WriteBuffer(_T("\n        GenCurve Anal  : Undefined - No underlying m_pGenCurve Object")) ;
                   }
                     smos_WriteBuffer(_T("\n        SurfAnal match : ")) ; sAnalyticMatchIvl.Dump() ;


  smos_WriteBuffer(_T("\n      Rule 2. GenCurve->NurbInterval  == m_bSwapUV ? sNaturalUVDomain.GetVInterval : sNaturalUVDomain.GetUInterval")) ;
  if(bHasGenCurve) { smos_WriteBuffer(_T("\n        GenCurve Nurb  : ")) ; sGenNaturalInterval.Dump() ; }
  else             { smos_WriteBuffer(_T("\n        GenCurve Nurb  : Undefined - No underlying m_pGenCurve Object")) ; }
  if(bHasNurb)     { smos_WriteBuffer(_T("\n        SurfNurb Match : ")) ; sNaturalMatchIvl.Dump() ; }
  else             { smos_WriteBuffer(_T("\n        SurfNurb Match : Undefined - No underlying m_pNurb Surface\n")) ; }

  smos_WriteBuffer(_T("\n      Rule 3. GenCurve NURBKnotVector == m_bSwapUV ? Surface->NURBKnotV : Surface->NURBKnotU")) ;
  if(bHasGenCurve && bHasNurb)
    { smos_sprintf(sBuff,_T("\n        This GenCurve NURBKnotVector %s - it %s == (m_bSwapUV ? Surface->NURBKnotU : Surface->NURBKnotV)"),
                       bSameGenKnots ? _T("is OKAY") : _T("has ERRORs") ,
                       bSameGenKnots ? _T("DOES") : _T("DOES NOT")) ;
      smos_WriteBuffer(sBuff);
    }
  smos_WriteBuffer(_T("\n        See knot vector listings below")) ;

  // STEP
  smos_WriteBuffer(_T("\n\nSmSurfOfExtrusion STEP definition - ")) ;
  smos_WriteBuffer(_T("\n  m_vOrigin      : ")) ; m_vOrigin.Dump() ;
  smos_WriteBuffer(_T("\n  m_vExtrusionVec: ")) ; m_vExtrusionVec.Dump() ;
  smos_WriteBuffer(_T("\n  m_vCurvePoint  : ")) ; m_vCurvePoint.Dump() ;
  smos_WriteBuffer(_T("\n  m_vCurveVec    : ")) ; m_vCurveVec.Dump() ;

  // NURB
  if(m_pGenCurve) { smos_WriteBuffer(_T("\n\nSmSurfOfExtrusion m_pGenCurve - ")) ;
                    m_pGenCurve->Dump();
                  }
  else            { smos_WriteBuffer(_T("\nNo SmSurfOfExtrusion GenCurve\n")) ; }
  if(m_pNurb)     { smos_WriteBuffer(_T("\nSmSurfOfExtrusion m_pNurb - ")) ;
                    SmBSplineSurface::Dump();
                  }
  else            { smos_WriteBuffer(_T("\nNo SmSurfOfExtrusion Underlying Nurb Surface\n")) ; }

  smos_WriteBuffer(_T(" End SmSurfOfExtrusion::Dump()\n")) ;

} // end SmSurfOfExtrusion::Dump
