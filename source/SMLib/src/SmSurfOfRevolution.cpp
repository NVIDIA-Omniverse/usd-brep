// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************//**
* FILE NAME --- SmSurfOfRevolution.cpp
* PURPOSE: Implementation of SmSurfOfRevolution methods.
* REVISION HISTORY:
*   02-Nov-96 - gac - initial version
**********************************************************************/

#include "StdAfx.h"

#include <SmSurfOfRevolution.h>
#include <SmSTEPSurface.h>
#include <SmCone.h>
#include <SmLine.h>
#include <SmSphere.h>
#include <SmPlane.h>
#include <SmEllipse.h>
#include <SmCircle.h>
#include <SmIsoCurve.h>
#include <SmCompositeCurve.h>
#include <nurbs.h>
#include <SmNurbsSrf.h>
#include <SmGraphicsExtern.h>
#include <SmGeomUtility.h>
#include <SmGlobalSolver.h>
#include <SmDatabaseIO.h>
#include <SmAssertArray.h>

#ifdef SM_DEBUG_CODE
 #include <SmBrep.h>
 #include <SmFace.h>
 #include <SmGap.h>
#endif // SM_DEBUG_CODE

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
SmSurfOfRevolution::~SmSurfOfRevolution()
{
  if(m_pGenCurve) { delete m_pGenCurve ; m_pGenCurve = NULL ; }

} // end SmSurfOfRevolution::~SmSurfOfRevolution destructor

/*******************************************************************//**
PURPOSE: Equality operator for SmSurfOfRevolution

NOTES: Call base equivalence to check type and then check
       members for equivalence
***********************************************************************/
SmBoolean SmSurfOfRevolution::operator==
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
      SmSurfOfRevolution &rOther = (SmSurfOfRevolution &)crOther ;

      // check equivalence of these objects
      bRtn =  (   (   ( m_pGenCurve == rOther.m_pGenCurve)
                   || ( m_pGenCurve == NULL && rOther.m_pGenCurve == NULL)
                   || (   m_pGenCurve != NULL && rOther.m_pGenCurve != NULL
                       && *m_pGenCurve == *rOther.m_pGenCurve))
               && m_bPlanarGenerator == rOther.m_bPlanarGenerator
               && m_vPosition        == rOther.m_vPosition
               && m_vAnalUVDomain    == rOther.m_vAnalUVDomain
               && m_bSwapUV          == rOther.m_bSwapUV
               && m_vPolarConverter  == rOther.m_vPolarConverter) ;
    }

  // all done
  return bRtn ;

} // end SmSurfOfRevolution::operator==

/*******************************************************************//**
PURPOSE: Create a Surface Of Revolution given origin, local X,Y axes,
    generating curve, angular parameter range.

NOTES: 1. Leaves m_pNurb = NULL ;
       2. Owns pGenCurve (deletes pGenCurve when this object is deleted)
       3. pGenCurve AnalInterval must equal crAnalUVDomain.GetVInteval
***********************************************************************/
SmSurfOfRevolution::SmSurfOfRevolution
  (SmBSplineCurve   * pGenCurve,        // in : Curve being rotated to create surface,
                                        //      SurfOfRevolution owns this curve and will delete it when destructed.
   const SmPoint3d  & crOrigin,         // in : Origin of the rotation
   const SmVector3d & crXAxis,          // in : unit-ortho-vector Marks 0 degree rotation direction
   const SmVector3d & crYAxis,          // in : unit-ortho-vector Marks 90 degree rotation direction
   const SmExtent2d & crAnalUVDomain,   // in : Analytic UV Domain [u=rotation in degrees, v=genCurve param]
   SmBoolean          bSwapUV)          // in : TRUE = underlying NURB v maps to rotation direction
                                        //      FALSE= NURB u maps to rotation direction
 : m_pGenCurve       (pGenCurve),
   m_bPlanarGenerator(TRUE),
   m_vPosition       (crOrigin, crXAxis, crYAxis),
   m_vAnalUVDomain   (crAnalUVDomain),
   m_bSwapUV         (bSwapUV)
{
  // set the GenCurve owner
  if(pGenCurve)
    { pGenCurve->SetOwner(this) ; }

  // check input GenCurve->STEPInteval must match crAnalUVDomain interval
  SE_MSG(   pGenCurve == NULL
         || pGenCurve->GetSTEPInterval().AreEqual(crAnalUVDomain.GetVInterval(), 10*SM_EFF_ZERO) ? SM_SUCCESS : SM_ERR,
         _T("Bad Revolution Input: GenCurve STEP interval must match crAnalUVDomain VInterval")) ;

  // set form factor
  m_eBSplineSurfaceForm = SM_SF_SURF_OF_REVOLUTION;

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  if(bDebugMe)
    {
      if(pGenCurve) { pGenCurve->Dump() ; }
    }
#endif // SM_DEBUG_CODE

  // When given a generator curve - see if its on the X/Z plane
  if (pGenCurve)
    {
      // check it for planarity
      SmPoint3d sPnt;
      SE(pGenCurve->EvaluatePoint(pGenCurve->GetNaturalInterval().Evaluate(0.5),sPnt));
      double d3DTolerance = ANALYTIC_TOL_SCALE * SM_EFF_ZERO * (1.0+sPnt.GetMaxDimension());

      // when curve is not planar
      if (!pGenCurve->IsPlanar(d3DTolerance))
        {
          // mark that
          m_bPlanarGenerator = FALSE;
        }
      else // curve is planar
        {
          // see if planar curve is on the X/Z plane
          double dMaxDistToSurf;
          SmBoolean bOnPlane;
          SE(pGenCurve->IsOnPlane(crOrigin, crYAxis, d3DTolerance, bOnPlane, dMaxDistToSurf));
          if (!bOnPlane)
            {
              m_bPlanarGenerator = FALSE;
            }
        }

    } // end given Generator curve check

} // end SmSurfOfRevolution::SmSurfOfRevolution constructor

/*******************************************************************//**
PURPOSE: Copy constructor for SmSurfOfRevolution

NOTES:
***********************************************************************/
SmSurfOfRevolution::SmSurfOfRevolution
  (const SmSurfOfRevolution & crSource)
 : SmBSplineSurface   (crSource),
   m_pGenCurve        (NULL),
   m_bMakeNurbGenCurve(crSource.m_bMakeNurbGenCurve),
   m_bPlanarGenerator (crSource.m_bPlanarGenerator),
   m_vPosition        (crSource.m_vPosition),
   m_vAnalUVDomain    (crSource.m_vAnalUVDomain),
   m_bSwapUV          (crSource.m_bSwapUV),
   m_vPolarConverter  (*GetContext(),
                       crSource.m_vPolarConverter)
{
  // check state
  if(crSource.m_pGenCurve == NULL)
    { SE(SM_ERR) ; }

  // virtual copy m_pGenCurve
  const SmContext *pContext = GetContext();
  if (crSource.m_pGenCurve)
    {
      SmCurve *pGenCurve;
      crSource.m_pGenCurve->Copy(*pContext,pGenCurve);
      m_pGenCurve = SM_CAST_PTR(SmBSplineCurve,pGenCurve);
      m_pGenCurve->SetOwner(this) ;
      if (!m_pGenCurve)
        { SE(SM_ERR); }
    }

} // end SmSurfOfRevolution::SmSurfOfRevolution constructor

/*******************************************************************//**
PURPOSE: Rebuild Polar converter

METHOD --- Build a new sweep IsoParametric Curve and use that
           to SetUpPolarConversion.

ASSUMES --- Underlying Nurb and Analytic domains are set as desired
***********************************************************************/
SmStatus SmSurfOfRevolution::RebuildPolarConverter()
{
  // Build a sweep isoParameterCurve.
  SmBSplineCurve *pSweepIsoCurve = NULL;
  SmObjDelete sClean;

  // Try the beginning of the curve if possible,
  // otherwise there can be trouble with non-planar gen curves.
  SmExtent2d sNurbDomain = GetNaturalUVDomain() ;
  SmPoint2d  sNurbMinUV  = sNurbDomain.GetMin() ;
  SM_ASSERT(GetContext() != NULL) ;
  m_vPolarConverter.SetPolarConversionPossible(FALSE) ;
  SmStatus sRtn ;  

  // this is a kind of self-reference -
  //     RebuildPolarConverter is called from change routines
  //     CreateIsoParametricCurve uses this member values to build its output.
  //     If those values are not all up-to-date this could cause a problem.
  sRtn= CreateIsoParametricCurve(*GetContext(),
                                 m_bSwapUV ? SM_SP_U      : SM_SP_V,
                                 m_bSwapUV ? sNurbMinUV.x : sNurbMinUV.y,
                                 0.0,
                                 pSweepIsoCurve) ;
  SM_ASSERT(sRtn == SM_SUCCESS) ;
  SmBoolean bIsoDegenerate = pSweepIsoCurve->IsDegenerate() ;
  sClean.SetObj( pSweepIsoCurve );

  if ( bIsoDegenerate )
  {
      // rebuild the IsoCurve - remember to delete the unused degenerate one
      if(pSweepIsoCurve) { delete pSweepIsoCurve ; pSweepIsoCurve = NULL ; }
      sClean.Clear();

      // Start point was on the axis.
      // Evaluate at the same fraction as ::CreateCanonical();
      // that should definitely not be on the axis.
      SmPoint2d  sNurbMidUV = sNurbDomain.Evaluate( 0.45, 0.45 ) ;
      m_vPolarConverter.SetPolarConversionPossible(FALSE) ;
      sRtn = CreateIsoParametricCurve(*GetContext(),
                                      m_bSwapUV ? SM_SP_U      : SM_SP_V,
                                      m_bSwapUV ? sNurbMidUV.x : sNurbMidUV.y,
                                      0.0,
                                      pSweepIsoCurve) ;
      bIsoDegenerate = pSweepIsoCurve->IsDegenerate() ;
      sClean.SetObj( pSweepIsoCurve );
  }
  SM_ASSERT(sRtn == SM_SUCCESS);

  // update PolarConverter
  if(   sRtn == SM_SUCCESS
     && !bIsoDegenerate)
    {
      // Build the Surfs PolarConverter for rotation angle based on the pSweepIsoCurve
      sClean.Clear();
      SM_ASSERT(pSweepIsoCurve->IsKindOf(SmEllipse_TYPE)) ;
      SmExtent1d sSweepIvl = m_vAnalUVDomain.GetUInterval() ;
      sRtn = m_vPolarConverter.SetUpPolarConversion((SmEllipse *)pSweepIsoCurve,
                                                    FALSE,   // give right to delete pSweepIsoCurve to m_vPolarConverter
                                                    sSweepIvl,
                                                    m_vPosition);
    }
  else
    {
      SER_MSG(SM_ERR, _T("RebuildPolarConverter error - Sweep interval has been made degenerate") );
    }

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  // draw
  if(bDebugMe)
    {
      Dump() ;
      m_pGenCurve->Dump() ;
      pSweepIsoCurve->Dump() ;
      m_vPolarConverter.Dump() ;

      SmFace *pFace = (SmFace *)GetFace() ;
      SmBrep *pBrep = pFace ? pFace->GetBrep() : NULL ;

      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,1) ; DrawUV() ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,0,0) ; if(pFace) pFace->Draw(SM_DM_CROSSHATCH) ; sm_GraphicsLoop() ;
      smgfx_SetLook(3,4, 1,0,0) ; if(pSweepIsoCurve) pSweepIsoCurve->Draw() ; sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

  // all done
  return(sRtn) ;

} // end SmSurfOfRevolution::RebuildPolarConverter

/*******************************************************************//**
PURPOSE: Adjust the surface by the given STEP-domain.

NOTES:
***********************************************************************/
SmStatus SmSurfOfRevolution::AdjustSTEPUVDomain
  (const SmExtent2d & crNewSTEPUVDomain)  // in : new desired domain in degrees
                                          //      range: [0_to_360, range along gen curve]
{
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  if(bDebugMe)
    {
      SM_ASSERT_VALID(this) ;

      SmFace *pFace = (SmFace *)this->GetFace() ;
      SmBrep *pBrep = pFace ? pFace->GetBrep() : NULL ;

      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,1) ; this->DrawUV(8,8,FALSE,NULL,TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,0,0) ; if(pFace) pFace->Draw(SM_DM_CROSSHATCH) ; sm_GraphicsLoop() ;
      smgfx_SetLook(3,4, 1,0,0) ; if(m_pGenCurve) m_pGenCurve->Draw() ; sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

  // no work - input domain = current domain
  if(   crNewSTEPUVDomain.IsContainedBy( m_vAnalUVDomain, SM_EFF_ZERO )
     && m_vAnalUVDomain.IsContainedBy( crNewSTEPUVDomain, SM_EFF_ZERO ))
    { return SM_SUCCESS; }

  // set surface analytic domain
  m_vAnalUVDomain = crNewSTEPUVDomain;

  // set GenCurve analytic domain from V part of new surface domain
  SER( m_pGenCurve->AdjustSTEPInterval( m_vAnalUVDomain.GetVInterval() ));

  // SmSurfOfExtrusion parameterization rules
  // The GenCurve parameterization must match the Surface Domain Intervals as
  //    Rule 1. GenCurve AnalInterval == Surface AnalVInterval   (no changes)
  //    Rule 2. GenCurve NURBInterval == m_bSwapUV ? Surface->NURBUInterval : Surface->NURBVInterval (needs update)
  //    Rule 3. GenCurve KnotVector   == m_bSwapUV ? Surface->NURBUKnot : Surface->NURBVKnot  (no changes)
  //
  // When PolarConverter has a PolarCurve, the PolarCurve parameterization must match the Surface Domain Intervals as
  //    Rule 4. PolarCurve AnalInterval == Surface AnalUInterval (no changes)
  //    Rule 5. PolarCurve NURBInterval == m_bSwapUV ? Surface->NURBVInterval : Surface->NURBUInterval (needs update)
  //    Rule 6. PolarCurve KnotVector   == m_bSwapUV ? Surface->NURBVKnot : Surface->NURBUKnot (no changes)

  // build Nurb domain from Analytic domain
  SmExtent2d sNurbDomain = crNewSTEPUVDomain ; 
  if(m_bSwapUV)
    { sNurbDomain.Transpose() ; }

  // set Surface and GenCurve nurb domain and enforce rules 2 and 5 assuming input GenCurve/Surf and PolarCurve/Surf knot vector compatibility
  // note: We reparameterize only to initialize the intervals and domain values so that MakeNurb() will build
  //       new Nurb structures trimmed and scaled to the input UVDomains
  Reparameterize(sNurbDomain) ; 

  // rebuild the associated m_pNurb and PolarConverter
  SER( MakeNurb() );

#ifdef SM_DEBUG_CODE
  // draw
  if(bDebugMe)
    {
      SM_DUMP_AND_ASSERT_VALID(this) ;

      SmFace *pFace = (SmFace *)this->GetFace() ;
      SmBrep *pBrep = pFace ? pFace->GetBrep() : NULL ;

      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,1) ; this->DrawUV(8,8,FALSE,NULL,TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,0,0) ; if(pFace) pFace->Draw(SM_DM_CROSSHATCH) ; sm_GraphicsLoop() ;
      smgfx_SetLook(3,4, 1,0,0) ; if(m_pGenCurve) m_pGenCurve->Draw() ; sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

  // all done
  return SM_SUCCESS;

} // end SmSurfOfRevolution::AdjustSTEPUVDomain

/*******************************************************************//**
PURPOSE: Copy a Surface Of Revolution.

NOTES:
***********************************************************************/
SmStatus SmSurfOfRevolution::Copy
  (const SmContext & crContext,
   SmSurface *& rpNewSurface)
  const
{
  SmSurfOfRevolution *pCopy = new(crContext) SmSurfOfRevolution(*this);
  NER(pCopy);
  rpNewSurface = pCopy;
  return SM_SUCCESS;

} // end SmSurfOfRevolution::Copy

/*******************************************************************//**
PURPOSE: Reparameterize the NURB domain of a B-Spline Surface

NOTES: This method is only available for users with NLib
***********************************************************************/
SmStatus SmSurfOfRevolution::Reparameterize(const SmExtent2d & crTrimDomain )       // in : new parameter range for BSPlineSurface
{
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  if(bDebugMe)
    {
      SM_DUMP_AND_ASSERT_VALID(this) ;
    }
#endif // SM_DEBUG_CODE

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
  // The GenCurve parameterization must match the Surface Domain Intervals as
  //    Rule 1. GenCurve AnalInterval == Surface AnalVInterval   (no changes)
  //    Rule 2. GenCurve NURBInterval == m_bSwapUV ? Surface->NURBUInterval : Surface->NURBVInterval (needs update)
  //    Rule 3. GenCurve KnotVector   == m_bSwapUV ? Surface->NURBUKnot : Surface->NURBVKnot  (no changes)
  //
  // When PolarConverter has a PolarCurve, the PolarCurve parameterization must match the Surface Domain Intervals as
  //    Rule 4. PolarCurve AnalInterval == Surface AnalUInterval (no changes)
  //    Rule 5. PolarCurve NURBInterval == m_bSwapUV ? Surface->NURBVInterval : Surface->NURBUInterval (needs update)
  //    Rule 6. PolarCurve KnotVector   == m_bSwapUV ? Surface->NURBVKnot : Surface->NURBUKnot (no changes)

  // Rule 2 :
  if(m_pGenCurve)
    { m_pGenCurve->EditParameterization(m_bSwapUV ? crTrimDomain.GetUInterval() : crTrimDomain.GetVInterval()) ; }

  // Rule 5 : Set PolarConverter from Nurb midPoint rotation isoParamCurve
  //          pRotationIsoCrv will be an SmCircle because isoParam is coming from SmSurfOfRevolution
  RebuildPolarConverter() ;

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

} // end SmSurfOfRevolution::Reparameterize

/*******************************************************************//**
PURPOSE: Given a point on the Surface-Of-Revolution, find its
    corresponding analytic UV-parameter.

NOTES: This method requires a point exactly on the surface.
    Therefore, users should be cautious when calling this method.

    Points which can be mapped outside the surface's trim domain are mapped
    and reLocation is set to SM_LT_EXTERIOR

   If the result is on the seam of a closed surface, and a guess was
   given, the output will be set to whichever side of the seam is closer
   to the guess.  If no guess was given, it will be set to the low end
   of the closed domain.

RETURNS --- SM_ERR for points outside the domain
            SM_SUCCESS for points inside the domain

METHOD --- 1. Rotate Point around rotation axis    - use rotation angle in degrees as U parameter.
           2. Intersect RotatedPoint with GenCurve - use GenCurve STEP Param as V parameter.
           3. Set Location value for points on poles, seams, and interiors
***********************************************************************/
SmStatus SmSurfOfRevolution::STEPInversion
  (const SmExtent2d & crAnalDomain,       // in : domain limit for valid transformations
                                          //      Passed on to SmBSplineSurface::STEPINVERSION for nonPlanar GenCurves
   const SmPoint3d  & crPointOnSurf,      // in : Target Point - must be on surface within Tolerance
   double             d3DTolerance,       // in : Max allowed distance between Point and Surface
                                          //      Also used to determine whether result is on a seam
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
  // pass call along for nonPlanar generator curves
  if ( !m_bPlanarGenerator )
    {
      // Not planar, must use the general solver.

      // Have to convert the analytical domain to a NURBS domain.
      SmExtent2d sNurbsDomain;
      SmPoint2d sStepUV, sNurbsUV;
      sStepUV = crAnalDomain.GetMin();
      ConvertUVFromSTEPToNURBS( sStepUV, sNurbsUV );
      sNurbsDomain.AddPoint2d( sNurbsUV );
      sStepUV = crAnalDomain.GetMax();
      ConvertUVFromSTEPToNURBS( sStepUV, sNurbsUV );
      sNurbsDomain.AddPoint2d( sNurbsUV );

      SmPoint2d sNurbsParameter;
      SER(SmBSplineSurface::STEPInversion( sNurbsDomain, crPointOnSurf, d3DTolerance,
                                           sNurbsParameter, reLocation, pUVGuess ));
      SER(ConvertUVFromNURBSToSTEP(sNurbsParameter, rdAnalUVParameter));
      return SM_SUCCESS;
    }

  // Generator curve is planar,
  // so we can solve it analytically.

  // init output
  reLocation = SM_LT_EXTERIOR;

  // Rotate Point to StartPlane
  ULONG lNumAngles;
  double adAnglesDeg[2];
  SmPoint3d sPointInPlane;
  SmBoolean bInside ;
  SER(TransformPointToStartPlane(crPointOnSurf,d3DTolerance,sPointInPlane,lNumAngles,adAnglesDeg, bInside));

  // bInside:     TRUE - point is in sweep interval
  //              FALSE- point is not in sweep interval
  // lNumAngles:  1 - point is not on closed seam
  //              2 - point is on closed seam or on a pole (a point on the rotation axis)
  // adAnglesDeg: is always set

  // old behavior
  //      // when point is not in sweep interval - exit with an error
  //      if(!bInside)
  //        { return(SM_ERR) ; }
  // new behavior = delay error return until best possible answer is placed
  //                in rdAnalUVParameter

  // set output U value (angle about rotation axis)
  //  For points on seams and larger angle is less than 360
  //   then the point is a little closer to the high side, else use low side value
  SmExtent1d sUDomain = m_vAnalUVDomain.GetUInterval();
  rdAnalUVParameter.x = (   lNumAngles > 1
                         && adAnglesDeg[1] < sUDomain.GetMax()-SM_EFF_ZERO)
                        ? adAnglesDeg[1]
                        : adAnglesDeg[0];

  // get Point/RotationAxis distance
  double dDist;
  SER(smgu_LinePointDistance(this->m_vPosition.GetOriginRef(),
                             m_vPosition.GetZAxis(),
                             crPointOnSurf,
                             dDist));

  // set output point classification
  reLocation =   (dDist < d3DTolerance) ? SM_LT_POLE
               : (lNumAngles ==  2)     ? SM_LT_U_SEAM
               : (bInside)              ? SM_LT_INTERIOR
               :                          SM_LT_EXTERIOR ;

  // If a guess was given and we're on the seam,
  // set u-parameter to what's closer to the guess.
  if ( reLocation == SM_LT_U_SEAM && pUVGuess != NULL )
    {
      if ( pUVGuess->x > sUDomain.GetMid() )
        { rdAnalUVParameter.x = sUDomain.GetMax(); }
    }

  // intersect rotated point with genCurve
  SmExtent1d sIvl = m_pGenCurve->GetSTEPInterval();
  if ( IsOutOfBoundsEnabled() )
    {
      sIvl = m_pGenCurve->GetMaxAnalyticDomain();
      m_pGenCurve->SetOutOfBoundsEnabled( TRUE );
    }
  SmSolution sSData[16];
  SmSolutionArray sSolutions(16,sSData);
  SER(m_pGenCurve->GlobalPointSolveSTEP(sIvl, SM_SO_INTERSECT,
                                        sPointInPlane,
                                        d3DTolerance, NULL, NULL,
                                        SM_SR_ALL, sSolutions));

  // error - rotated point not on GenCurve
  if (sSolutions.GetSize() == 0)
    { reLocation = SM_LT_EXTERIOR ;
      return SM_ERR;
    }

  // set output U Coordinate
  SmSolution &rSol    = sSolutions[0];
  rdAnalUVParameter.y = rSol.m_vStart[0];

  // for points on closed genCurve seams - update point classification
  if(   sSolutions.GetSize() > 1
     && m_pGenCurve->IsClosed(sIvl,d3DTolerance)
     && sIvl.IsValueOnBoundary(rdAnalUVParameter.y) )
    {
      reLocation =   (reLocation == SM_LT_INTERIOR) ? SM_LT_V_SEAM
                   : (reLocation == SM_LT_U_SEAM)   ? SM_LT_UV_SEAM
                   :  reLocation ;

      // On the v-seam.  If a guess was given,
      // set v-parameter to what's closer to the guess.
      if ( pUVGuess != NULL )
        {
          SmExtent1d sVDomain = m_vAnalUVDomain.GetVInterval();
          if ( pUVGuess->y > sVDomain.GetMid() )
            { rdAnalUVParameter.y = sVDomain.GetMax(); }
        }

    } // end point on closed genCurve Seam check

  // all done
  return( bInside ? SM_SUCCESS : SM_ERR) ;

} // end SmSurfOfRevolution::STEPInversion

/*******************************************************************//**
PURPOSE: Create a Surface-Of-Revolution by rotating a curve one
    complete revolution about an axis.

NOTES:
    1. The generator curve need not be coplanar with the axis of revolution.
    2. An error is returned when the SweptCurve is line colinear with the
         rotation axis.

***********************************************************************/
SmStatus SmSurfOfRevolution::CreateCanonical
  (const SmContext     & crContext,              // in : context for new object construction
   SmBSplineCurve      * pSweptCurve,            // in : Curve to sweep into surface
   const SmPoint3d     & rAxisPoint,             // in : Point on axis of rotation
   const SmVector3d    & rAxisDirection,         // in : Direction of axis of rotation
   SmSurfOfRevolution *& rpNewSurfOfRevolution,  // out: New surface or NULL when input can't be swept
    SmBoolean             bSwapUV )              // in : TRUE = underlying NURB v maps to rotation direction                      
                                                 //      FALSE= NURB u maps to rotation direction                           
{
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  if(bDebugMe)
    {
      if(pSweptCurve) { pSweptCurve->Dump() ; }
    }
#endif // SM_DEBUG_CODE

  // unitize the rotation axis
  SmVector3d sZVec = rAxisDirection;
  sZVec.Unitize();

  // locals
  SmExtent1d sCrvIvl = pSweptCurve->GetSTEPInterval();

  // SweptCurve start point
  SmPoint3d sPnt;
  SER(pSweptCurve->EvaluateSTEP(sCrvIvl.GetMin(),0,TRUE,&sPnt));

  // Vector from SweptCurve Start to RotAxis point
  SmVector3d sXVec = sPnt - rAxisPoint;

  // when start point is on rot axis
  if(   sXVec.Length() < SM_EFF_ZERO
     || sXVec.IsParallelTo(sZVec,1.0))
    {
      // try SweptCurve nearMidPoint
      SER(pSweptCurve->EvaluateSTEP(sCrvIvl.Evaluate(0.45),0,TRUE,&sPnt));
      sXVec = sPnt - rAxisPoint;
    }

  // GWC:COMMENT - one should intersect the rot axis with the curve here.
  //  If there is an intersection either
  //    - quit with an error, or
  //    - split the curve and make two surfaces.

  // Y vec of rotCoordSystem
  SmVector3d sYVec = sZVec * sXVec;

  // sYVec can be 0,0,0 if revolving a line coincident with the axis
  if( sYVec.LengthSquared() < SM_EFF_ZERO_SQ)
    { return SM_ERR ; }
  //      if( sYVec.Unitize() != SM_SUCCESS )
  //        { return SM_ERR; }
  SER(sYVec.Unitize()) ;

  // Z vec of rotCoordSystem
  sXVec = sYVec * sZVec;

  // allocate full rotation SurfOfRevolution object
  SmExtent2d sAnalUVDomain(SmPoint2d(0.0,  sCrvIvl.GetMin()),
                           SmPoint2d(360.0,sCrvIvl.GetMax()));
  rpNewSurfOfRevolution = new (crContext) SmSurfOfRevolution(pSweptCurve,
                                                             rAxisPoint,
                                                             sXVec,
                                                             sYVec,
                                                             sAnalUVDomain,
                                                             bSwapUV);
  // check output
  NER(rpNewSurfOfRevolution);

  // Make Nurb
  SER(rpNewSurfOfRevolution->MakeNurb());

  // all done
  return SM_SUCCESS;

} // end SmSurfOfRevolution::CreateCanonical

/*******************************************************************//**
PURPOSE: Get canonical data from a SmSurfOfRevolution

NOTES:
***********************************************************************/
SmStatus SmSurfOfRevolution::GetCanonical
  (SmBSplineCurve *& rpSweptCurve,     // out: Gen Curve
   SmPoint3d       & rAxisPoint,       // out: Center Point of rotation axis
   SmVector3d      & rAxisDirection,   // out: direction    of rotation axis
   SmBoolean       * pOptSwapUV)       // out: TRUE = Nurb and Analytic U and V directions are swapped
                                       //      NULL to ignore, default:[NULL]
 const
{
  rpSweptCurve   = m_pGenCurve;
  rAxisPoint     = m_vPosition.GetOriginRef();
  rAxisDirection = m_vPosition.GetZAxis();
  if(pOptSwapUV) { *pOptSwapUV = m_bSwapUV ; }

  return SM_SUCCESS;

} // end SmSurfOfRevolution::GetCanonical

/*******************************************************************//**
PURPOSE: Get the maximum allowable domain for an analytic representaion
   of this surface.

NOTES:
   The u/v parameters correspond to the STEP parameterization.

   In the v-direction, for gen-curves of degree 3 or higher, or even 2,
   this expansion can result in wildly pathological surface behavior
   even for the expansion factors used here.  Use this method with caution.
***********************************************************************/
SmExtent2d SmSurfOfRevolution::GetMaxAnalyticDomain()
 const
{
  SmExtent1d sVDom = m_pGenCurve->GetMaxAnalyticDomain();

  // u-domain is 0 to 360.
  SmExtent2d sDom( 0.0, sVDom.GetMin(), 360.0, sVDom.GetMax() );

  return sDom;

} // end SmSurfOfRevolution::GetMaxAnalyticDomain

/*******************************************************************//**
PURPOSE: Get NURB domain range of surface's rotation and optionally,
           the NURB Surf parameter associated with the rotation.

NOTES:  when m_bSwapUV == FALSE, pOptExtSurfParam = SM_SP_U
             m_bSwapUV == TRUE,  pOptExtSurfParam = SM_SP_V
***********************************************************************/
SmExtent1d SmSurfOfRevolution::GetRotateParamExtent
  (SmSurfParamType * pOptRotSurfParam)    // out: optional NURB param ivl which spans the extrusion direction
                                          //      NULL to ignore, default:[NULL]
 const
{
  // get m_pNurb UVDomain
  SmExtent2d sNurbUVDomain = GetNaturalUVDomain();

  // V is always the STEP linear parameter
  // which may be swapped with the NURB parameter
  if (m_bSwapUV == FALSE)
    {
      if (pOptRotSurfParam) *pOptRotSurfParam = SM_SP_U;
      return( sNurbUVDomain.GetUInterval() ) ;
    }
  else
    {
      if (pOptRotSurfParam) *pOptRotSurfParam = SM_SP_V;
      return( sNurbUVDomain.GetVInterval() ) ;
    }

} // end SmSurfOfRevolution::GetRotateParamExtent

/*******************************************************************//**
PURPOSE: Get NURB domain range of surface's GenCurve direction and optionally,
           the NURB Surf parameter associated with the GenCurve direction.

NOTES:  when m_bSwapUV == FALSE, pOptGenSurfParam = SM_SP_V
             m_bSwapUV == TRUE,  pOptGenSurfParam = SM_SP_U
***********************************************************************/
SmExtent1d SmSurfOfRevolution::GetGenDirParamExtent
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
      if (pOptGenSurfParam) *pOptGenSurfParam = SM_SP_V;
      return( sNurbUVDomain.GetVInterval() ) ;
    }
  else
    {
      if (pOptGenSurfParam) *pOptGenSurfParam = SM_SP_U;
      return( sNurbUVDomain.GetUInterval() ) ;
    }

} // end SmSurfOfRevolution::GetGenDirParamExtent

/*******************************************************************//**
PURPOSE: Given a point in Euclidian space determine the corresponding
     extrema points on the surface based on STEP-parametrization.
     Valid solver operations for this method include:
     SM_SO_MINIMIZE, SM_SO_MAXIMIZE, SM_SO_NORMALIZE, SM_SO_INTERSECT.

NOTES:
***********************************************************************/
SmStatus SmSurfOfRevolution::GlobalPointSolveSTEP
  (const SmExtent2d & crAnalUVDomain,
   // STEP-based Domain of surface
   SmSolverOperationType eSolverOperation,
   const SmPoint3d & crTestPoint,
   double dDistanceTolerance,
   const double * cpdOptTargetDistance,
   SmSolutionRequestedType eSolutionRequested,
   SmSolutionArray & rSolutions)
{
    SM_ASSERT(eSolverOperation == SM_SO_MINIMIZE ||
              eSolverOperation == SM_SO_MAXIMIZE ||
              eSolverOperation == SM_SO_NORMALIZE ||
              eSolverOperation == SM_SO_INTERSECT);

    SER(SmSurfOfRevolution::AdjustSTEPUVDomain(crAnalUVDomain));

    // Drop the point using NURBS-based solver
    SmExtent2d sUVDomain = GetNaturalUVDomain();
    SER(GlobalPointSolve(sUVDomain,eSolverOperation,
        crTestPoint,dDistanceTolerance,cpdOptTargetDistance,
        eSolutionRequested,rSolutions));

    if (rSolutions.GetSize() == 0)
        return SM_SUCCESS;

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
    if (bDebugMe) {
        smgfx_Erase();
        smgfx_SetLook(1,6, 1,0,0); crTestPoint.Draw();  sm_GraphicsLoop();
        smgfx_SetLook(1,1, 0,0,0); this->DrawUV(4,4);   sm_GraphicsLoop();
        smgfx_SetLook(4,4, 1,0,0); m_pGenCurve->Draw(); sm_GraphicsLoop();
        sm_GraphicsLoop();
    }
#endif
        //pCurve->DrawWDeriv(pCurve->GetNaturalInterval());

    SmPoint2d sUVSize = crAnalUVDomain.GetSize();
    double dUVTol = 100.0*SM_EFF_ZERO_SQRT*(1.0+sUVSize.x+sUVSize.y);
    ULONG lPoleCount = 0;
    ULONG lUVSeamCount = 0;
    // Process all solutions
    for (ULONG i=0; i<rSolutions.GetSize(); i++) {
        SmPoint3d sPnt;
        SmSolution & rSol = rSolutions[i];
        SmPoint2d sNurbUV(rSol.m_vStart[0],rSol.m_vStart[1]);
        SER(EvaluatePoint(sNurbUV,sPnt));
        SmPoint2d sAnalUV;
        SmLocationType eLocation;
        SER(STEPInversion(crAnalUVDomain,sPnt,dDistanceTolerance,
                          sAnalUV,eLocation))
        rSol.m_vStart[0] = sAnalUV.x;
        rSol.m_vStart[1] = sAnalUV.y;
        SmPoint3d sPnt1;
        if (eLocation == SM_LT_INTERIOR) continue;

        // Now we have SEAM/POLE cases, check whether we have
        // duplicate solutions?
        if (eLocation == SM_LT_POLE) lPoleCount++;
        else if (eLocation == SM_LT_UV_SEAM) lUVSeamCount++;
        for (ULONG jj=0; jj<i; jj++) {
            SmSolution & rExistedSol = rSolutions[jj];
            SmPoint2d sExistedUV(rExistedSol.m_vStart[0],
                                 rExistedSol.m_vStart[1]);
            if (sAnalUV.DistanceBetween(sExistedUV) > dUVTol)
                continue;
            // Coincidence found
            switch (eLocation) {
            case SM_LT_U_SEAM:
                rSol.m_vStart[0] = 360.0;
                break;
            case SM_LT_V_SEAM:
                rSol.m_vStart[1] = crAnalUVDomain.GetMax().y;
                break;
            case SM_LT_UV_SEAM:
                if (lUVSeamCount>2)
                    rSol.m_vStart[0] = 360.0;
                if (lUVSeamCount%2 == 0)
                    rSol.m_vStart[1] = crAnalUVDomain.GetMax().y;
                break;
            case SM_LT_POLE:
                if (lPoleCount == 2) {
                    // Assign the max value of angular domain to it
                    rSol.m_vStart[0] = crAnalUVDomain.GetMax().x;
                }
                else if (lPoleCount > 2) {
                    // Pick some point in the middle
                    double dParamInc = sUVSize.x/(rSolutions.GetSize()-1);
                    rSol.m_vStart[0] = crAnalUVDomain.GetMin().x +
                        dParamInc * (lPoleCount-2);
                }
                break;
            case SM_LT_INTERIOR:
                break;
            case SM_LT_EXTERIOR:
                break;
            }
        }
    }

    return SM_SUCCESS;

} // end SmSurfOfRevolution::GlobalPointSolveSTEP

/*******************************************************************//**
PURPOSE: Make the Nurb representation for a SmSurfOfRevolution

NOTES:
***********************************************************************/
SmStatus SmSurfOfRevolution::MakeNurb()
{
  // pass the call along
  return(MakeNurbWithSweepParams(NULL)) ;

} // end SmSurfOfRevolution::MakeNurb

/*******************************************************************//**
PURPOSE: Make the Nurb representation for a SmSurfOfRevolution
            Copying the Sweep direction parameterization from the
            given sweep curve and the GenCurve direction parameterization
            from the stored m_pGenCurve.

NOTES: When pOptSweepCurve == NULL an unspecified sweep
            parameterization is assigned to the surface by a call to
            SmBSplineSurface::CreateSurfOfRevolution.
***********************************************************************/
SmStatus SmSurfOfRevolution::MakeNurbWithSweepParams
  (const SmCircle *pOptSweepCurve)
{
  // locals
  SmExtent1d       sSurfSweepStepIvl = m_vAnalUVDomain.GetUInterval() ;
  SmExtent1d       sSurfGenStepIvl   = m_vAnalUVDomain.GetVInterval() ;
  SmExtent1d       sGenCurveStepIvl  = m_pGenCurve->GetSTEPInterval();
  SmBSplineCurve  *pGenCurve         = NULL;
  const SmContext *cpContext         = GetContext() ;

  // get a GenCurve
  SER(CreateGeneratorFromAngle(*cpContext, sSurfSweepStepIvl.GetMin(), pGenCurve));
  SmObjDelete sClean1(pGenCurve);
  SmExtent1d       sGenCurveNurbIvl  = pGenCurve->GetNaturalInterval() ;

  // see if we are asked to specify the sweep direction parameterization
  // ?? something to let MakeNurb fetch a specified sweepCurve
  // ?? without having to change the argument list for this virtual function

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  if(bDebugMe)
    {
      if(m_pNurb) { SmBSplineSurface::Dump() ; }
      pGenCurve->Dump() ;
      if(pOptSweepCurve) { pOptSweepCurve->Dump() ; }

      SmFace *pFace = (SmFace *)GetFace() ;
      SmBrep *pBrep = pFace ? pFace->GetBrep() : NULL ;

      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,1) ; if(m_pNurb) DrawUV() ; sm_GraphicsLoop() ;
      smgfx_SetLook(4,5, 1,0,0) ; pGenCurve->Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook(4,5, 0,1,0) ; if(pOptSweepCurve) pOptSweepCurve->Draw() ; sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

  // When SurfaceCurve and generator intervals are not the same
  if(   !sSurfGenStepIvl.IsContainedBy(sGenCurveStepIvl)
     || !sGenCurveStepIvl.IsContainedBy(sSurfGenStepIvl))
    {
      // Trimming is needed.
      SER(m_pGenCurve->AdjustSTEPInterval(sSurfGenStepIvl));
      SER(pGenCurve->AdjustSTEPInterval(sSurfGenStepIvl));
    }

  // Check if we need to create GenCurve's Nurb form
  pGenCurve->GetOrCreateGwNurbPointer() ;

  // build the Nurb Surface
  const SmContext  * pContext = GetContext();
  SmBSplineSurface * pTmp     = NULL;
  SER(SmBSplineSurface::CreateSurfOfRevolution(*pContext, pGenCurve,
                                               m_vPosition.GetOriginRef(),
                                               m_vPosition.GetZAxis(),
                                               sSurfSweepStepIvl.GetLength(),
                                               pTmp, m_bSwapUV, pOptSweepCurve));
  SmObjDelete sClean2(pTmp);
  SmSurfOfRevolution *pTmpSurfOfRevo = (SmSurfOfRevolution*)pTmp;

  // Swap nurbs so that the old m_pNurb is deallocated when we go out of scope
  gw_SURFACE *pTmpNurb    = m_pNurb;
  m_pNurb                 = pTmp->GetGwNurbPointer();
  pTmpSurfOfRevo->m_pNurb = pTmpNurb;

#ifdef SM_DEBUG_CODE
  if(bDebugMe)
    {
      SmBSplineSurface::Dump() ;
      pGenCurve->Dump() ;
    }
#endif // SM_DEBUG_CODE

  // Set up the Polar conversion - create a sweep isoParametric curve
  RebuildPolarConverter() ;

#ifdef SM_DEBUG_CODE
  if (bDebugMe)
    {
      if(m_pNurb) { SmBSplineSurface::Dump() ; }
      if(pTmp)    { pTmp->Dump(); }
      pGenCurve->Dump();

      smgfx_SetColor(1,0,0); pGenCurve->DrawWDeriv(sSurfGenStepIvl); sm_GraphicsLoop();
      smgfx_SetColor(0,0,1); pTmp->DrawUV(1,1); sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
#endif

  // all done
  return SM_SUCCESS;

} // end SmSurfOfRevolution::MakeNurbWithSweepParams

// GWC: this effort was commented out - don't remember why, but it was
// originally written to have SurfOfRevolution update cached m_vGenCurve after Surface modifications.
// I'm leaving the source here as a comment in case we have to go back to that.
//      /*******************************************************************//**
//      PURPOSE: Receive notification of things happening to the object and
//      take appropriate actions.
//
//      NOTES:
//        1. Rebuild m_vPolarConverter for sweep direction domain changes
//        2. Rebuild m_vGenCurve for GenCurve direction domain changes
//
//      ***********************************************************************/
//      void SmSurfOfRevolution::Notify        // expected calls: caller->Notify(Event, pData1, pData2, pData2)
//       (SmNotifyOperation  eNotifyOperation, //       event                | caller      |  pData1  | pData2                | pData3
//        SmObject         * pData1,           //----------------------------+-------------+----------+-----------------------+--------------------------
//        SmObject         * pData2,           // SM_NO_ADD_TO_BREP          | Brep        | AddObj   | Brep                  | AddObj's GeomPtr or NULL
//        SmObject         * pData3)           // SM_NO_SPLIT_IN_BREP        | Brep/TopoObj| OrigObj  | Child1                | Child2
//                                             // SM_NO_MERGE_IN_BREP        | Brep/TopoObj| SurvObj  | DelObj                | Brep
//                                             // SM_NO_TRIM_NO_SPLIT_IN_BREP| Brep        | TgtObj   | AddedBndryObj         | NULL                     
                                               // SM_NO_COINCIDENT           | BrepA       | BrepAObj | BrepBObj              | BrepB
//                                             // SM_NO_RM_FROM_BREP         | Brep        | RmObj    | Brep                  | RmObj's GeomPtr or NULL
//                                             // SM_NO_CHANGE_GEOMETRY      | TopoObj     | NewGeom  | Brep or NULL          | OldGeom or NULL
//                                             // SM_NO_CHANGE_OWNER         | GeomObj     | NewOwner | NewOwner Brep or NULL | OldOwner or NULL
//                                             // SM_NO_CONSTRUCTION         | NewObj      |  NewObj  | CopyFromObj or NULL   | NULL
//                                             // SM_NO_COPY                 | FromObj     | ToObj    | ToObj's Owner or NULL | FromObj's Owner or NULL
//                                             // SM_NO_PRE_EDIT             | EditObj     | EditObj  | EditObj Owner or NULL | NULL
//                                             // SM_NO_POST_EDIT            | EditObj     | EditObj  | EditObj Owner or NULL | NULL
//                                             // SM_NO_SPLIT                | SplitGeomObj| Child1   | Child2                | SplitObj's Owner or NULL
//                                             // SM_NO_MERGE                | MergeGeomObj| OrigObj1 | OrigObj2              | MergeObj's Owner or NULL
//                                             // SM_NO_REG_PROPAGATION      | MergeReg    | ThisRegs | OtherBrep->SrcRegs    | ThisBrep->MergeReg
//                                             // SM_NO_DESTRUCTION          | DelObj      | DelObj   |  NULL                 |  NULL                     //
//      {
//      #ifdef SM_DEBUG_CODE
//      SmBoolean bDebugMe = FALSE ;
//        // draw
//        if(bDebugMe)
//          {
//            Dump() ;
//            SM_DUMP_AND_ASSERT_VALID(this) ;  // don't expect to pass this.
//                                     // just testing for out-of-date cases which
//                                     // should be updated below.
//
//            SmFace *pFace = (SmFace *)GetFace() ;
//            SmBrep *pBrep = pFace ? pFace->GetBrep() : NULL ;
//
//            smgfx_Erase() ;
//            smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
//            smgfx_SetLook(1,2, 0,1,1) ; DrawUV() ; sm_GraphicsLoop() ;
//            smgfx_SetLook(1,2, 0,0,0) ; if(pFace) pFace->Draw(SM_DM_CROSSHATCH) ; sm_GraphicsLoop() ;
//            smgfx_SetLook(3,4, 1,0,0) ; if(m_pGenCurve) m_pGenCurve->Draw() ; sm_GraphicsLoop() ;
//            sm_GraphicsLoop() ;
//          }
//      #endif // SM_DEBUG_CODE
//
//        switch (eNotifyOperation)
//          {
//            case SM_NO_POST_EDIT:
//      /*
//              { // make sure m_pGenCurve and pPolarCurve have same parameterizations as m_pNurb
//
//                // test GenCurve and pPolarCurves against m_pNurb parameterizations
//                SmBoolean bGoodGenCurve   = TRUE ;
//                SmBoolean bGoodPolarCurve = TRUE ;
//
//                // check genCurve parameterization
//                if(m_pNurb && m_pGenCurve)
//                  {
//                    // GenCurve Curve must have same parameterization as Surface in non SweepCurve direction
//                    // GenCurve AnalInterval   must equal Surface->AnalUVDomain->GetVInterval,
//                    // GenCurve NurbInterval   must equal m_bSwapUV ? Surface->NURBUInterval : Surface->NURBVInterval
//                    // GenCurve NURBKnotVector must equal m_bSwapUV ? Surface->NURBKnotU : Surface->NURBKnotV
//                    bGoodGenCurve = HasSameParameterization(SM_SP_V,        // in : Analytic Domain Direction
//                                                              m_bSwapUV     // in : NURB     Domain Direction
//                                                            ? SM_SP_U
//                                                            : SM_SP_V,
//                                                            m_pGenCurve) ;  // in : The GenCurve
//                   }
//
//                // check PolarCurve parameterization
//                const SmBSplineCurve *pPolarCurve = m_vPolarConverter.GetCurve() ;
//                if(m_pNURB && pPolarCurve)
//                  {
//                    // PolarConverter Curve must have same parameterization as Surface in SweepCurve direction
//                    // pPolarCurve AnalInterval   must equal Surface->AnalUVDomain->GetUInterval,
//                    // pPolarCurve NURBInterval   must equal m_bSwapUV ? Surface->NURBVInterval : Surface->NURBUInterval
//                    // pPolarCurve NURBKnotVector must equal m_bSwapUV ? Surface->NURBKnotV : Surface->NURBKnotU
//                    bGoodPolarCurve = HasSameParameterization(SM_SP_U,        // in : Analytic Domain Direction
//                                                                m_bSwapUV     // in : NURB     Domain Direction
//                                                              ? SM_SP_V
//                                                              : SM_SP_U,
//                                                              pPolarCurve) ;  // in : The GenCurve
//                  } // end m_pNurb and pPolarCurve existence check
//
//                // when GenCurve is out of date
//                if(!bGoodGenCurve)
//                  {
//                    // 1. recompute the GenCurve
//                    SmBSplineCurve *pGenCurve ;
//                    SM_ASSERT(GetContext() != NULL) ;
//                    CreateGeneratorFromAngle(*GetContext(),
//                                             0.0,
//                                             pGenCurve) ;
//                    if(m_pGenCurve) { delete m_pGenCurve ; m_pGenCurve = NULL ; }
//                    m_pGenCurve = pGenCurve ;
//                    m_pGenCurve->SetOwner(this)
//                  }
//
//                // when polarConverter is out of date
//                if(!bGoodPolarCurve)
//                  {
//                    RebuildPolarConverter() ;
//
//                    //      // 2. recompute the polar converter from a new sweep dir isoParamCurve
//                    //      SmBSplineCurve *pRotationIsoCrv = NULL ;
//                    //      SmPoint2d       sMidUV          = m_vAnalUVDomain.Evaluate(.5, .5) ;
//                    //      SM_ASSERT(GetContext() != NULL) ;
//                    //      m_vPolarConverter.SetPolarConversionPossible(FALSE) ;
//                    //      SmStatus sRtn = CreateIsoParametricCurve(*GetContext(),
//                    //                                               m_bSwapUV ? SM_SP_U : SM_SP_V,
//                    //                                               m_bSwapUV ? sMidUV.x : sMidUV.y,
//                    //                                               0.0,
//                    //                                               pRotationIsoCrv);
//                    //      SM_ASSERT(sRtn == SM_SUCCESS) ;
//                    //
//                    //      // Build the Surfs PolarConverter for rotation angle based on the pRotationIsoCrv
//                    //      SmExtent1d sSweepIvl = m_vAnalUVDomain.GetUInterval() ;
//                    //      sRtn = m_vPolarConverter.SetUpPolarConversion(pRotationIsoCrv,
//                    //                                                    FALSE,
//                    //                                                    sSweepIvl,
//                    //                                                    m_vPosition);
//                  }
//                SM_DUMP_AND_ASSERT2_VALID(this) ;
//
//              } // end case SM_NO_POST_EDIT
//      */
//              break ;
//
//            case SM_NO_ADD_TO_BREP           : break ;
//            case SM_NO_SPLIT_IN_BREP         : break ;
//            case SM_NO_MERGE_IN_BREP         : break ;
//            case SM_NO_TRIM_NO_SPLIT_IN_BREP : break ;
//            case SM_NO_COINCIDENT            : break ;
//            case SM_NO_RM_FROM_BREP          : break ;
//            case SM_NO_CHANGE_GEOMETRY       : break ;
//            case SM_NO_CHANGE_OWNER          : break ;
//            case SM_NO_CONSTRUCTION          : break ;
//            case SM_NO_COPY                  : break ;
//            case SM_NO_PRE_EDIT              : break ;
//            case SM_NO_SPLIT                 : break ;
//            case SM_NO_MERGE                 : break ;
//            case SM_NO_REG_PROPAGATION       : break ;
//            case SM_NO_DESTRUCTION           : break ;
//            case SM_NO_UNKNOWN:                { SE_MSG(SM_ERR, _T("SmSurfOfRevolution::Notify - SM_NO_UNKNOWN event signalled")) ; }
//                                          break ;
//          } // end switch on eNotifyOperation
//
//      #ifdef SM_DEBUG_CODE
//        // draw
//        if(bDebugMe)
//          {
//            Dump() ;
//            SM_DUMP_AND_ASSERT_VALID(this) ;  // expect to pass this.
//                                     // surface should be up to date.
//
//            SmFace *pFace = (SmFace *)GetFace() ;
//            SmBrep *pBrep = pFace ? pFace->GetBrep() : NULL ;
//
//            smgfx_Erase() ;
//            smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
//            smgfx_SetLook(1,2, 0,1,1) ; DrawUV() ; sm_GraphicsLoop() ;
//            smgfx_SetLook(1,2, 0,0,0) ; if(pFace) pFace->Draw(SM_DM_CROSSHATCH) ; sm_GraphicsLoop() ;
//            smgfx_SetLook(3,4, 1,0,0) ; if(m_pGenCurve) m_pGenCurve->Draw() ; sm_GraphicsLoop() ;
//            sm_GraphicsLoop() ;
//          }
//      #endif // SM_DEBUG_CODE
//
//        // Propagate notification up hierarchy
//        SmBSplineSurface::Notify(eNotifyOperation,pData1,pData2,pData3);
//
//      } // end SmSurfOfRevolution::Notify

/*******************************************************************//**
PURPOSE: Create an offset surface for the surface of revolution.

NOTES: The offset Nurb and Step domains map from old to new
     so that   pOffSet->EvaluateSTEPPoint(StepUV) = pInput->EvaluateSTEPPoint() + dOff * dNormal(StepUV)
    and        pOffSet->EvaluatePoint(NurbUV)     = pInput->EvaluatePoint()     + dOff * dNormal(NurbUV)

    Offset Surf->Domain(s) may be trimmed but not scaled so any
    OffsetSurface->EvaluatePoint(UVPoint) corresponds to the OriginSurface->EvaluatePoint(UVPoint)
***********************************************************************/
SmStatus SmSurfOfRevolution::CreateOffsetSurface
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
  SmExtent2d                    sInputNurbDomain = GetNaturalUVDomain() ;
  SmExtent1d                    sGenCurveNurbIvl = m_bSwapUV ? sInputNurbDomain.GetUInterval()
                                                             : sInputNurbDomain.GetVInterval() ;

  // Get a nonDegenerate SweepCurve, to pass to MakeNurbWithSweepParams().
  SmCircle        *pSweepCurve = NULL ;
  double           dParam, dRadius ;
  SmPoint3d        sCenter ;

  ULONG ii, lCnt=7 ;
  for(ii=0;ii<lCnt;ii++)
    {
      dParam =  sGenCurveNurbIvl.Evaluate(  ii == 0        ? 0.0
                                          : ii == lCnt - 1 ? 1.0
                                          : (double)ii / (double)(lCnt-1)) ;
      SmBSplineCurve *pGeneratrix = NULL ;
      SER(CreateDirectrixAtGeneratorParam(crContext, dParam, pGeneratrix, dRadius, sCenter)) ;
      if(dRadius > SM_EFF_ZERO_SQRT)
        { SM_ASSERT(pGeneratrix && pGeneratrix->IsKindOf(SmCircle_TYPE)) ;
          pSweepCurve = (SmCircle *)pGeneratrix ;
          break ;
        }
      else // ignore degenerate curves
        { delete pGeneratrix ; pGeneratrix = NULL;
          pSweepCurve = NULL ;
        }
    } // end search for nonDegenerate sweep curve

  SM_ASSERT(pSweepCurve != NULL) ;

  // Clean temp object
  SmObjDelete sDelete1(pSweepCurve);


  // Offset the genCurve.

  // Get the offset normal vector.
  double dOffsetDistance = smos_Fabs(dSignedOffsetDistance);
  double dSign           =  (    (m_bSwapUV || dSignedOffsetDistance < 0.0)
                             && !(m_bSwapUV && dSignedOffsetDistance < 0.0))
                           ? -1.0
                           :  1.0;

  // Gen curve is in our (local) x-z plane, so the plane normal is our y axis.
  SmVector3d sPlaneNorm = - m_vPosition.GetYAxisRef() * dSign ;

  SmBSplineCurve *pOffsetGenCurve = NULL;
  double dMaxGap = 0.0;

  // CreateSimpleOffset() is more appropriate here than the CompositeCurve
  // methods, with just a single curve to offset, because it will preserve
  // SmLines and SmCircles.  [B526]
  // But it is defined only for SmBSplineCurves and classes derived from that.
  if ( m_pGenCurve->IsKindOf( SmBSplineCurve_TYPE ) )
    {
      SER( m_pGenCurve->CreateSimpleOffset( crContext, 
                                dThisApproxTol3d/10.0,
                                sPlaneNorm,
                                dOffsetDistance,
                                pOffsetGenCurve,
                                dMaxGap ));
      NER( pOffsetGenCurve );

      sTrimmedOffsets.Add( pOffsetGenCurve );
    }
  else
    {
      // m_pGenCurve is not an SmBSplineCurve: that's actually an error.
      WARN( _T("Error: SmSurfOfRevolution m_pGenCurve is not an SmBSplineCurve") );

      // But if it is ever generalized from that, here is what we would do.
      // (Or possibly generalize CreateSimpleOffset to SmCurve and its derivatives.)

      // Copy the genCurve
      SmCurve *pCopy;
      SER(m_pGenCurve->Copy(crContext,pCopy));
      sCCurves.Add(pCopy);

      // use it to create a composite curve
      SmCompositeCurve *pCC = new (crContext) SmCompositeCurve( 3, sCCurves,
                                m_pGenCurve->IsClosed(m_pGenCurve->GetNaturalInterval()),
                                NULL, NULL);
      // Clean temp object
      SmObjDelete sDelete2(pCC);

      // now offset the composite curve.

      // get the offset vector
      dOffsetDistance = smos_Fabs(dSignedOffsetDistance);
      dSign           =  (    (m_bSwapUV || dSignedOffsetDistance < 0.0)
                          && !(m_bSwapUV && dSignedOffsetDistance < 0.0))
                          ? -1.0
                          :  1.0;
      sPlaneNorm      = - m_vPosition.GetYAxisRef() * dSign ;

      // create Trimmed offset of the composite curve

      // GWC_NEEDS_WORK GET_BETTER_VALUES_FOR_TOLERANCE_IN_FOLLOWING_CALL GWC_LINE ;
      SER(pCC->CreateTrimmedOffsets
            (crContext,                    // in : context for new object construction
             dThisApproxTol3d/2.5,         // in : Minimum distance at which offset curve end-gaps are filled with corner curves
             dThisApproxTol3d/10.0,        // in : Tolerance to which BSpline Approximations to exact offset curves are built
             sPlaneNorm,                   // in : Defines, along with the curve's parameter direction,
                                           //      the right and left hand offset directions.
             SM_OC_FILLET_CORNER,          // in : SM_OC_LINEAR_EXTENSION: corner = 2 lines from given ends to common linear extension xsect point.
                                           //      SM_OC_FILLET_CORNER   : corner = fillet arc centered on crVertexPoint running to given end points
                                           //      SM_OC_LINEAR_CHAMFER  : corner = line between given end points (result is actually within offset distance so bTrimResults must = false)
             SM_OD_RIGHT_HAND_SIDE,        // in : Corresponding direction of each composite
                                           //      curve member to offset.  1-LEFT, 2-RIGHT, 3-BOTH
             TRUE,                         // in : TRUE = concave raw offsets are intersected and trimmed back to common intersection points
                                           //      FALSE= skip trim step
             dOffsetDistance,              // in : offset distance (a negative value negates the offset direction)
             sTrimmedOffsets)) ;           // out: unordered array of offset SmBSplineCurves will have attribute attached
                                           //      describing origination of curve
                                           // in : TRUE = if BSplineCurve just copy it (preserves CrvParams)
                                           //      FALSE= approximate BSplineCurves (changes CrvParams), NonBSplineCrvs always Approximated
                                           //      default:[FALSE]

  } // end branch on SmBSplineCurve type gen curve


  // Create a new offset surface of revolution for every trimmed offset.
  // (Note, there should be only one: see above.)
  SmBSplineCurve *pOffCurve = NULL ;
  for ( ii=0; ii<sTrimmedOffsets.GetSize(); ii++ )
    {
      // fetch temporary GenCurve and make it permanent
      pOffCurve           = sTrimmedOffsets[ii];
      sTrimmedOffsets[ii] = NULL ;

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
      if (bDebugMe)
        {
          m_pGenCurve->Dump();
          pOffCurve->Dump();
          smgfx_Erase();
          smgfx_SetLook( 2,4, 0,0,1 ); m_pGenCurve->Draw(); sm_GraphicsLoop();
          smgfx_SetLook( 2,4, 0,1,1 ); pOffCurve  ->Draw(); sm_GraphicsLoop();
          sm_GraphicsLoop();
        }
#endif

      // simplify the curve if possible
      SER(pOffCurve->RemoveExtraKnots(dThisApproxTol3d/10.0));

      // Trim the offset curve with the axis of rev to avoid self intersecting results
      SmVector3d zAxis = m_vPosition.GetZAxis();
  
      SmExtent1d sDomain = pOffCurve->GetNaturalInterval();
          
          SmExtent3d bbox;
          pOffCurve->CalculateBoundingBox(sDomain, &bbox);
          double dHeight = bbox.GetMaxDimension() * 1.5;                // multiply max dimension by some factor to be sure
                                                                        
          SmLine* pAxisLine = NULL;
          SmLine::CreateLineSegment(crContext, 3, m_vPosition.GetOrigin(), m_vPosition.GetOrigin() + dHeight * zAxis, pAxisLine);
          SmObjDelete sClean( pAxisLine );

          SmSolutionArray sSolutions;
          pOffCurve->GlobalCurveIntersect(sDomain, *pAxisLine, pAxisLine->GetNaturalInterval(), dThisApproxTol3d, sSolutions);
          
          if (sSolutions.GetSize() == 1) {
                  // if intersection is at endpoint, do not bother trimming
                  dParam = sSolutions[0].m_vStart[0];

                  if (sDomain.GetMin() != dParam || sDomain.GetMax() != dParam) {
                          
                          // Take the largest portion for now
                          // Eventually use a better algorithm
                          if (dParam < sDomain.GetMid()) {
                            sDomain.SetMin(dParam);
                          }
                          else {
                            sDomain.SetMax(dParam);
                      }

                          pOffCurve->Trim(sDomain);  // may snap sIvl by tol to existing knots

#ifdef SM_DEBUG_CODE
                          if (bDebugMe)
                          {
                                  smgfx_SetLook(3, 3, 0, 1, 1); pOffCurve->Draw(); sm_GraphicsLoop();
                                  sm_GraphicsLoop();
                          }
#endif
                  }
          }
          

      // If the offset curve is analytic, then its parameterization must match
      // the v-direction parameterization of the offset surface, in both the
      // analytic and Nurbs versions.
      // If it is not analytic, then it has only one parameterization, that of
      // the Nurbs definition.  Since the offset surface must match both Nurbs
      // and analytic, this implies that the offset surface must have the same
      // parameterization for Nurbs and analytic.
      // Either way, GetSTEPInterval() returns what we want.

      SmExtent2d sOffsetSurfDomain( m_vAnalUVDomain );
      sOffsetSurfDomain.SetVInterval( pOffCurve->GetSTEPInterval() );

      // create the new offset surface of revolution
      SmSurfOfRevolution *pOffSurf = new (crContext) SmSurfOfRevolution
                    (pOffCurve,                   // in : stored in to pOffSurf
                     m_vPosition.GetOriginRef(),
                     m_vPosition.GetXAxisRef(),
                     m_vPosition.GetYAxisRef(),
                     sOffsetSurfDomain,
                     m_bSwapUV) ;

      // rebuild the underlying Nurb Surface - with current parameterization
      //   SER: If we can't create the Nurb, then fail. [B526]
      SER( pOffSurf->MakeNurbWithSweepParams( pSweepCurve ));

//            // reparameterize the pOffSurf m_pNurb domain so that input domain == output domain
//            //   The input and offset Surface GenCurve Nurb intervals are equal now,
//            //   but the Surfaces' sweep curve Nurb intervals may vary
//            if(m_pNurb)
//              {
//                // get current domain
//                SmExtent2d sInputNurbDomain = GetNaturalUVDomain() ;
//
//                // reset the nurb domain
//                pOffSurf->SmBSplineSurface::Reparameterize(sInputNurbDomain) ;
//
//                // rebuild the polar converter
//                pOffSurf->RebuildPolarConverter() ;
//              }

      SM_DUMP_AND_ASSERT2_VALID(pOffSurf) ;

//            // create the new offset surface of revolution
//            SER(SmBSplineSurface::CreateSurfOfRevolution(crContext,
//                                                         pOffCurve,
//                                                         m_vPosition.GetOriginRef(),
//                                                         m_vPosition.GetZAxis(),
//                                                         GetEndAngleDeg()-GetStartAngleDeg(),
//                                                         pOffSurf,
//                                                         m_bSwapUV));
//
//            // get domains of new and offset surfaces
//            SmExtent2d sDomain  = pOffSurf->GetNaturalUVDomain();
//            SmExtent2d sOrigDom = GetNaturalUVDomain();
//
//            // compute the desired domain of the offset surfaces
//            if (m_bSwapUV)
//              {
//                sDomain.SetMinMax(SmPoint2d(sDomain.GetMin().x,sOrigDom.GetMin().y),
//                                  SmPoint2d(sDomain.GetMax().x,sOrigDom.GetMax().y));
//              }
//            else
//              {
//                sDomain.SetMinMax(SmPoint2d(sOrigDom.GetMin().x,sDomain.GetMin().y),
//                                  SmPoint2d(sOrigDom.GetMax().x,sDomain.GetMax().y));
//              }
//
//            // reparameterize the offset surface
//            pOffSurf->Reparameterize(sDomain);

#ifdef SM_DEBUG_CODE
      if (bDebugMe)
        {
          this->Dump() ;
          pOffSurf->Dump() ;
          if(pSweepCurve) pSweepCurve->Dump() ;
          m_pGenCurve->Dump() ;
          if(pOffCurve) pOffCurve->Dump() ;

          if(m_pNurb)
            {
              SmExtent2d sInputDomain  = GetNaturalUVDomain() ;
              SmExtent2d sInputStepDomain  = GetSTEPUVDomain() ;
              SmExtent2d sOffsetNurbDomain = pOffSurf->GetNaturalUVDomain() ;
              SmExtent2d sOffsetStepDomain = pOffSurf->GetSTEPUVDomain() ;
              // No: If 'this' has an analytic gen curve whose offset is not
              // analytic (such as an ellipse), then the offset curve will
              // be a general SmBSplineCurve, and the 'analytic' domain of that
              // offset curve will be the same as its Nurbs domain, which
              // implies that the analytic and Nurbs domains of the offset
              // surface (v-direction) must be the same.  If 'this' surface has
              // different analytic and Nurbs domains, then the offset cannot
              // match both.  We have set it up (in this method, above) so that
              // the Nurbs domains will match, at the expense of the analytic
              // domains.  So leave out the second check unless bIsAnalytic.
              SM_ASSERT( sInputDomain.IsContainedBy(sOffsetNurbDomain)
                        && sOffsetNurbDomain.IsContainedBy( sInputDomain, SM_EFF_ZERO)) ;

              // The only offset curves that can be analytic are lines and circles:
              // any other offset cannot be analytic.
              SmBoolean bIsAnalytic =
                  ( pOffCurve->IsKindOf( SmLine_TYPE ) || pOffCurve->IsKindOf( SmCircle_TYPE ) );
              if ( bIsAnalytic ) {
                  SM_ASSERT(   sInputStepDomain.IsContainedBy(sOffsetStepDomain, SM_EFF_ZERO)
                            && sOffsetStepDomain.IsContainedBy(sInputStepDomain, SM_EFF_ZERO)) ;
              }
            }
          SmFace *pFace = (SmFace *)GetFace() ;
          SmBrep *pBrep = pFace ? pFace->GetBrep() : NULL ;

          smgfx_Erase();
          smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
          smgfx_SetLook(1,2, 0,1,1) ; DrawUV(3,6); sm_GraphicsLoop();
          smgfx_SetLook(1,2, 0,1,0) ; pOffSurf->DrawUV(3,6); sm_GraphicsLoop();
          smgfx_SetLook(3,5, 1,0,0) ; if(pSweepCurve) pSweepCurve->Draw() ; sm_GraphicsLoop() ;
          smgfx_SetLook(3,5, 0,0,1) ; m_pGenCurve->Draw() ; sm_GraphicsLoop() ;
          smgfx_SetLook(3,5, 0,0,1) ; if(pOffCurve) pOffCurve->Draw() ; sm_GraphicsLoop() ;
          sm_GraphicsLoop();
        }
#endif

//        SER(pOffSurf->Reparameterize(GetNaturalUVDomain()));
      SM_DUMP_AND_ASSERT2_VALID(pOffSurf) ;
      rOffsetSurface = pOffSurf;

    } // end iter every trimmed offset Curve

  // all done
  return SM_SUCCESS;

} // end SmSurfOfRevolution::CreateOffsetSurface

/*******************************************************************//**
PURPOSE: Create an approximate surface of this surface

NOTES: Good for improving (reducing knot and control-point counts
or turning rational into non-rational) the surfaces representation.

The underlying m_pGenCurve is replaced by an approximation to itself
where the difference between the original and returned GenCurves are
limited to the input tolerance
***********************************************************************/
SmStatus SmSurfOfRevolution::CreateApproxSurface
  (const SmContext      & crContext,               // in : context for new object construction
   double                 dThisApproxTol3d,        // in : max allowed deviation between returned surface and current surface
   SmSurfOfRevolution  *& rpNewSurfOfRevolution)   // out: Approximating Surface
  const
{
  // approximate the GenCurve
  SmBSplineCurve *pNewCurve = NULL ;
  m_pGenCurve->ApproximateRationalCurve(*GetContext(),
                                        dThisApproxTol3d,
                                        pNewCurve) ;
  SM_ASSERT(pNewCurve != NULL) ;

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  if(bDebugMe)
    {
      m_pGenCurve->Dump() ;
      pNewCurve->Dump() ;

      SmFace *pFace = (SmFace *)this->GetFace() ;
      SmBrep *pBrep = pFace ? pFace->GetBrep() : NULL ;

      smgfx_Erase() ;
      smgfx_SetLook(2,4, 0,0,1) ; m_pGenCurve->DrawControlPoints() ; sm_GraphicsLoop() ;
      smgfx_SetLook(3,6, 1,0,0) ; pNewCurve->DrawControlPoints() ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,1) ; this->DrawUV() ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,0,0) ; if(pFace) pFace->Draw(SM_DM_CROSSHATCH) ; sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE


  // make the new surface
  rpNewSurfOfRevolution = new (crContext) SmSurfOfRevolution(pNewCurve,
                                                             m_vPosition.GetOriginRef(),
                                                             m_vPosition.GetXAxisRef(),
                                                             m_vPosition.GetYAxisRef(),
                                                             m_vAnalUVDomain,
                                                             m_bSwapUV) ;
  SER(rpNewSurfOfRevolution->MakeNurb());

  // all done
  return(SM_SUCCESS) ;

} // end SmSurfOfRevolution::CreateApproxSurface

/*******************************************************************//**
PURPOSE: Create a 3D iso-parametric curve of this surface given the
    Nurb parameter direction (U or V) and the constant parameter value
    in that direction.

VIRTUAL FUNCTION ---
    for SmBSplineSurface   - Make a BSpline Curve              (exact)
        SmSurfOfRevolution - Make a SmCircle or SmBSplineCurve (exact)
        SmSphere           - Make a SmCircle Curve             (exact)
        SmPlane            - Make a Line                       (exact)
        SmCone             - Make a Line or SmCircle Curve     (exact)
        SmTorus            - Make a SmCircle curve             (exact)
        All Others         - Make a piecewise Hermite Curve approximation
                                good to optional tolerance or
                                dLength * SM_EFF_ZERO_SQRT * 100.0.
***********************************************************************/
SmStatus SmSurfOfRevolution::CreateIsoParametricCurve
 (const SmContext  & crContext,        // in : context for created objects
  SmSurfParamType    eSurfParam,       // in : Defines which Nurb parameter direction on surface to extract curve from
                                       //      SM_SP_U = create constant u isoParameter curve
                                       //      SM_SP_V = create constant v isoParameter curve
  double             dIsoParameter,    // in : Defines Nurb parametric value at which to extract the curve.
                                       //      If eSurfParam==SM_SP_U this is a U parameter, if eSurfParam==SM_SP_V
                                       //      then this is the V parameter
  SmApproxTol3d      sApproxTol3d,     // NotUsed: in : passed to ApproximateCurve() when approximation is required.
                                       //      If set to 0.0, tolerance is set by system: old[curve length * 1.0e-4] new[GetApproxTol3d()]
  SmBSplineCurve  *& rpNewIsoCurve,    // out: 3d IsoParameterCurve
  const SmExtent2d * pOptDomain,       // in : optional trim bound for IsoParameterCurve, NULL to ignore, default:[NULL]
  double           * pOptMaxGap3d,     // out: opt achieved max gap, NULL to ignore, default:[NULL]
  SmCurve         ** pOptUVIsoCurve)   // out: opt 2d UVTrimCurve Line (diff parameterization), NULL to ignore, default:[NULL]
 const
{
  SM_REF1(sApproxTol3d) ;
  // check state - skip for now because method is called during transitions
  //  SM_DUMP_AND_ASSERT2_VALID(this) ;

  // init output
  rpNewIsoCurve = NULL ;
  if(pOptMaxGap3d) { *pOptMaxGap3d = 0.0 ; }

  // get the BSplineCurve
  SmBSplineCurve *pIsoCurve = NULL ;
  SER( SmBSplineSurface::CreateIsoParametricCurve(crContext,
                                                  eSurfParam,
                                                  dIsoParameter,
                                                  0.0,
                                                  pIsoCurve,
                                                  pOptDomain,
                                                  pOptMaxGap3d,     // out: opt achieved max gap, NULL to ignore, default:[NULL]
                                                  pOptUVIsoCurve)); // out: opt 2d UVTrimCurve Line (diff parameterization), NULL to ignore, default:[NULL]

  // If eSurfParam corresponds to the direction of revolution,
  // so that the isocurve is a circle,
  // then we make an actual SmCircle.
  // Test for that here.
  SmSurfParamType eSTEPParam = m_bSwapUV
                               ? ((eSurfParam == SM_SP_U) ? SM_SP_V : SM_SP_U)
                               : eSurfParam ;

  // done when building a gencurve directed isoParameter or a degerenate circle
  if(   eSTEPParam == SM_SP_U
     || pIsoCurve->IsDegenerate())
    {
      rpNewIsoCurve = pIsoCurve ;
      return SM_SUCCESS ;
    }

  // Ok, the curve is circular: make an SmCircle.

  // make pIsoCurve temporary
  SmObjDelete sClean(pIsoCurve) ;

  // Surface locals and vectors to IsoCurve NurbEndPoints
  SmExtent2d sNurbDomain      = GetNaturalUVDomain() ;
  SmPoint3d  sCenter          = m_vPosition.GetOriginRef() ;
  SmVector3d sRevolutionZAxis = m_vPosition.GetZAxis() ;

  // Nurb Surface size - trimmed to optional domain
  SmPoint2d sMinUV, sMaxUV;
  if ( eSurfParam == SM_SP_U )
  {
      sMinUV.Set(dIsoParameter, pOptDomain ? pOptDomain->GetMin().y : sNurbDomain.GetMin().y);
      sMaxUV.Set(dIsoParameter, pOptDomain ? pOptDomain->GetMax().y : sNurbDomain.GetMax().y);
  }
  else
  {
      sMinUV.Set(pOptDomain ? pOptDomain->GetMin().x : sNurbDomain.GetMin().x, dIsoParameter);
      sMaxUV.Set(pOptDomain ? pOptDomain->GetMax().x : sNurbDomain.GetMax().x, dIsoParameter);
  }

  double dNurbSpanSize = (sMaxUV - sMinUV).Length() ;
  SmPoint3d sMinPoint, sMaxPoint ;
  EvaluatePoint(sMinUV, sMinPoint) ;
  EvaluatePoint(sMaxUV, sMaxPoint) ;
  SmVector3d sVecMin = sMinPoint - sCenter ;
  SmVector3d sVecMax = sMaxPoint - sCenter ;

  // eSTEPParam == SM_SP_V, return rotation IsoCurve
  // get Circle specifications from Rotation parameters

  // Get isoCurve analytic domain
  SmExtent1d sAnalDomain ;
  // Bug fix: Always calculate this from the actual angles.
  // [Surface Unit tests, with non-planar gen curve]
//  if ( pOptDomain == NULL )
//    {
//      // let analytic interval = surface analytic interval
//      sAnalDomain.SetMinMax(m_vAnalUVDomain.GetMin().x,
//                            m_vAnalUVDomain.GetMax().x) ;
//    }
//  else // get angles to given points
//    {
  // let Ivl = angles (deg) to min/max points measured in circle coordinate system
  double dAngleMinRad, dAngleMaxRad ;
  sRevolutionZAxis.CCWAngleBetween(m_vPosition.GetXAxisRef(), sVecMin, dAngleMinRad) ;
  sRevolutionZAxis.CCWAngleBetween(m_vPosition.GetXAxisRef(), sVecMax, dAngleMaxRad) ;
  double dAngleMinDeg = dAngleMinRad * 180.0 / SM_PI ;
  double dAngleMaxDeg = dAngleMaxRad * 180.0 / SM_PI ;
  if(   dAngleMaxDeg < dAngleMinDeg
     || (    SM_IS_ZERO(dAngleMaxDeg-dAngleMinDeg)
         && !SM_IS_ZERO(dNurbSpanSize)))
    { dAngleMaxDeg += 360.0 ; }

  // The angles can come back negative from CCWAngleBetween().
  // If they are centered below our AnalyticDomain, move them up. [B171]
  if ( (dAngleMinDeg+dAngleMaxDeg)/2 < m_vAnalUVDomain.GetMin().x )
    {
      dAngleMinDeg += 360;
      dAngleMaxDeg += 360;
    }

  sAnalDomain.SetMinMax(dAngleMinDeg, dAngleMaxDeg) ;
//    }

  // use distance between circle and refFrame centers to get radius and circle center
  double    dDisp         = sRevolutionZAxis.Dot(sVecMin) ;
  SmPoint3d sCircleCenter = sCenter + dDisp * sRevolutionZAxis ;
  double    dRadius       = smos_Sqrt(smos_Fabs(sVecMin.LengthSquared() - dDisp * dDisp)) ;
  SM_ASSERT(SM_ARE_SAME(dDisp, sRevolutionZAxis.Dot(sVecMax))) ;

  // degenerate cases should have already been detected
  SM_ASSERT(   !SM_IS_ZERO(dRadius)
            && !SM_IS_ZERO(sAnalDomain.GetLength())) ;

  // create the circle using the isoCurve parameterization
  SmCircle *pCircle = new (crContext) SmCircle(sCircleCenter,
                                               m_vPosition.GetXAxisRef(),
                                               m_vPosition.GetYAxisRef(),
                                               sAnalDomain,
                                               dRadius, 3, &crContext,
                                               pIsoCurve) ;  // memory leak here
  rpNewIsoCurve = pCircle;

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  // draw
  if(bDebugMe)
    {
      SmFace *pFace = (SmFace *)GetFace() ;
      SmBrep *pBrep = pFace ? pFace->GetBrep() : NULL ;

      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,1) ; DrawUV() ; sm_GraphicsLoop() ;
      smgfx_SetLook(4,6, 1,0,0) ; rpNewIsoCurve->Draw() ; sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

  // all done
  return SM_SUCCESS;

} // end SmSurfOfRevolution::CreateIsoParametricCurve

/*******************************************************************//**
PURPOSE: Given a surface of revolution, return a curve, which is the
    directrix of the surface, that corresponds to the point of
    the generator, defined by the given generator parameter.

    Directrices are circle segments of varying radii, with
    centers located at varying positions along the axis.

NOTES: Also returns the center and radius of the created arc.
***********************************************************************/
SmStatus SmSurfOfRevolution::CreateDirectrixAtGeneratorParam
  (const SmContext & crContext,       // in : context for new object construction
   double            dGenCurveParam,  // in : Param in NurbDomain of GenCurve
   SmBSplineCurve *& rpDirectrix,     // out: Circular Arc Curve for given param
   double          & dRadius,         // out: Radius of returned Directrix
   SmPoint3d       & sAxisPoint)      // out: CenterPoint of returned Directrix
  const
{
  // init output
  rpDirectrix = NULL;

  // Get GeneratorPoint for given Param
  SmExtent1d sGeneratorDomain(m_pGenCurve->GetNaturalInterval());
  if(!sGeneratorDomain.ContainsValue(dGenCurveParam))
    { SER(SM_ERR); }
  SmPoint3d sGeneratorPoint;
  SER(m_pGenCurve->EvaluatePoint(dGenCurveParam, sGeneratorPoint));

  // SurfOfRevolution locals
  const SmPoint3d &rOrigin = m_vPosition.GetOriginRef();
  SmVector3d       sZAxis  = m_vPosition.GetZAxis() ;

  // Find the centerpoint of the circle to be created
  double dArcCenterParameter;
  SER(smgu_LineClosestPoint(rOrigin, sZAxis,
                            sGeneratorPoint, //  crTestPoint,
                            dArcCenterParameter));
  sAxisPoint = rOrigin + dArcCenterParameter * sZAxis;

  // set radius
  dRadius =  (sGeneratorPoint-sAxisPoint).Length();

  // pass call along
  SmStatus sRtn = CreateIsoParametricCurve(crContext,
                                           m_bSwapUV ? SM_SP_U : SM_SP_V,
                                           dGenCurveParam,
                                           0.0,           // dTol, not used
                                           rpDirectrix) ;
  // all done
  return(sRtn) ;
//
//
//        // Get GeneratorPoint for given Param
//        SmExtent1d sGeneratorDomain(m_pGenCurve->GetNaturalInterval());
//        if(!sGeneratorDomain.ContainsValue(dGenCurveParam))
//          { SER(SM_ERR); }
//        SmPoint3d sGeneratorPoint;
//        SER(m_pGenCurve->EvaluatePoint(dGenCurveParam, sGeneratorPoint));
//
//        // SurfOfRevolution locals
//        const SmPoint3d &rOrigin = m_vPosition.GetOriginRef();
//        SmVector3d sZAxis( m_vPosition.GetZAxis() );
//
//        // Find the centerpoint of the circle to be created
//        double dArcCenterParameter;
//        SER(smgu_LineClosestPoint(rOrigin, sZAxis,
//                                  sGeneratorPoint, //  crTestPoint,
//                                  dArcCenterParameter));
//        SmPoint3d sArcCenter(rOrigin + dArcCenterParameter * sZAxis);
//
//        // Build circle RefFrame = Surf RefFrame translated up the ZAxis
//        SmAxis2Placement sArcRF(m_vPosition);
//        sArcRF.Translate( sArcCenter - rOrigin );
//
//        // set radius
//        dRadius =  (sGeneratorPoint-sArcCenter).Length();
//
//        // build degenerate curve for degenerate radius
//        if (smos_Fabs(dRadius) < SM_EFF_ZERO)
//          {
//            SmBSplineCurve *pDegenerateCurve = NULL ;
//            SER(SmBSplineCurve::CreateDegenerateCurve(crContext,
//                                                      3, // dim of result
//                                                      sArcRF.GetOriginRef(),
//                                                      pDegenerateCurve));
//            rpDirectrix = pDegenerateCurve;
//            sAxisPoint  = sArcRF.GetOriginRef();
//            return SM_SUCCESS;
//          } // end degenerate curve branch
//
//        // else build a circle
//        SmBSplineCurve* pCircle;
//        SER(SmBSplineCurve::CreateCircleSegment(crContext,
//                                                3, // nDimensionOfResult,
//                                                sArcRF,
//                                                dRadius,
//                                                GetStartAngleDeg(),  // [0,360)
//                                                GetEndAngleDeg(),     // (0,360]
//                                                SM_CO_QUADRATIC,
//                                                pCircle));
//        NER(pCircle);
//        rpDirectrix = pCircle;
//        sAxisPoint = sArcCenter;
//        return SM_SUCCESS;

} // end SmSurfOfRevolution::CreateDirectrixAtGeneratorParam

/*******************************************************************//**
PURPOSE: Given an SmSurfOfRevolution, return a curve, which is its
            the directrix passing through the given point.
            The directrices are circle arcs of varying radii, the centers
            of which are located at varying positions along the axis of
            rotation.

NOTES:  No curve is returned if the point is not on the
                 surface or outside the surface's trim boundary.
***********************************************************************/
SmStatus SmSurfOfRevolution::CreateDirectrixFromPoint
  (const SmContext & crContext,       // in : new object context
   const SmPoint3d & rPointOnSurf,    // in : Point on the SurfOfRevolution
   double            d3DTolerance,    // NotUsed: in : Not Used
   SmBSplineCurve *& rpDirectrix)     // out: Sweep Curve interpolating PointOnSurf
  const
{
  SM_REF1(d3DTolerance) ;
  // init output
  rpDirectrix = NULL ;

  // Get StepUVPoint for 3DPoint
  double dScaledZero = SM_EFF_ZERO * 1000.0 * (1.0 + m_vPosition.GetOriginRef().GetMaxDimension()) ;
  SmPoint2d sStepUV;
  SmLocationType eLoc ;
  SmStatus sRtn = STEPInversion(m_vAnalUVDomain, rPointOnSurf, dScaledZero,
                                sStepUV, eLoc) ;

  // gwc bug fix changed behavior here - used to build the directrix for outOfBounds cases now it doesn't
  if(eLoc == SM_LT_EXTERIOR)
    { return(SM_ERR) ; }

  // convert StepUV to NurbUV
  SmPoint2d sNurbUV ;
  SER(ConvertUVFromSTEPToNURBS(sStepUV, sNurbUV)) ;

  // pass call along
  sRtn = CreateIsoParametricCurve(crContext,
                                  m_bSwapUV ? SM_SP_U : SM_SP_V,
                                  m_bSwapUV ? sNurbUV.x: sNurbUV.y,
                                  0.0,           // dTol - not used
                                  rpDirectrix) ;

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  // draw
  if(bDebugMe)
    {
      SmFace *pFace = (SmFace *)GetFace() ;
      SmBrep *pBrep = pFace ? pFace->GetBrep() : NULL ;

      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,1) ; DrawUV() ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,0,0) ; if(pFace) pFace->Draw(SM_DM_CROSSHATCH) ; sm_GraphicsLoop() ;
      smgfx_SetLook(3,5, 1,0,0) ; rPointOnSurf.Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook(2,3, 1,0,0) ; rpDirectrix->Draw() ; sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

  // all done
  return(sRtn) ;

//        // skip points off the Surf or outside the Surf limits
//        if(sRtn != SM_SUCCESS)
//         { return(SM_ERR); }
//
//        // get point on SurfOfRevolution - cleaning up tolerances
//        SmPoint3d sSurfacePoint ;
//        EvaluateSTEPPoint(sSTEPUVPoint, sSurfacePoint) ;
//
//        // SurfOfRevolution Locals
//        const SmPoint3d  &rOrigin  = m_vPosition.GetOriginRef();
//        const SmVector3d &rXAxis   = m_vPosition.GetXAxisRef();
//        const SmVector3d &rYAxis   = m_vPosition.GetYAxisRef();
//        SmVector3d        sZAxis   = m_vPosition.GetZAxis();
//        SmVector3d        sVec(sSurfacePoint - rOrigin);
//        double            dZ       = sVec.Dot(sZAxis) ;
//        double            dRadius  = (sVec - dZ * sZAxis).Length() ;
//
//        // m_bSwapUV only indicates that the Nurb's parametrization
//        // is swapped wrt the analytic domain, thus no need to take it into
//        // account.  Also the Insideout behavior is already accounted for.
//
//        // define the circle's coordinate system
//        SmPoint3d sCircleCenter = rOrigin + dZ * sZAxis ;
//        SmAxis2Placement sCirclePlacement(sCircleCenter, rXAxis, rYAxis) ;
//
//        // make the circle
//        SmCircle *pCircle = NULL ;
//        SmCircle::CreateCanonical(crContext, sCirclePlacement, dRadius,
//                                  pCircle,
//                                  &SmExtent1d(m_vAnalUVDomain.GetMin().x,
//                                              m_vAnalUVDomain.GetMax().x)) ;
//        rpDirectrix = pCircle;
//
//        // all done
//        return SM_SUCCESS;

} // end SmSurfOfRevolution::CreateDirectrixFromPoint

/*******************************************************************//**
PURPOSE: Given a surface of revolution, return a curve, which is its
    generator corresponding to the given radial direction (defined
    by the angle from the X axis of the local coordinate system).
    A new curve will be created and returned.

NOTES: The given angle does not need to lie within the angular domain of
                the surface.
***********************************************************************/
SmStatus SmSurfOfRevolution::CreateGeneratorFromAngle
  (const SmContext & crContext,    // in : new object context
   double            dAngleDeg,    // in : desired angle in degrees
   SmBSplineCurve *& rpGenerator)  // out: new curve, same derived type as genCurve
  const
{
  // copy the current GenCurve
  SmCurve* pNewCurve = NULL;
  SER(m_pGenCurve->Copy(crContext,pNewCurve));
  SmObjDelete sDelete(pNewCurve);

  // when rotating the genCurve
  if ( smos_Fabs( dAngleDeg ) > SM_EFF_ZERO )
    {
      // define the rotation matrix
      SmAxis2Placement sRF; // m_vPosition leads to bug
      sRF.RotateAboutAxisAtPoint(dAngleDeg * SM_PI / 180.0,
                                 m_vPosition.GetOriginRef(),
                                 m_vPosition.GetZAxis());

      // rotate the GenCurve Copy
      SER(pNewCurve->Transform(sRF));
    }

  // set output
  sDelete.Clear();
  rpGenerator = SM_CAST_PTR(SmBSplineCurve,pNewCurve);

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
  if (bDebugMe)
    {
      Dump() ;
      rpGenerator->Dump();

      SmFace *pFace = (SmFace *)GetFace() ;
      SmBrep *pBrep = pFace ? pFace->GetBrep() : NULL ;

      smgfx_Erase();
      smgfx_SetLook(1,2, 0,0,1); if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,1); DrawUV() ; sm_GraphicsLoop() ;
      smgfx_SetLook(3,4, 1,0,0); rpGenerator->Draw(); sm_GraphicsLoop() ;
      sm_GraphicsLoop();
    }
#endif

  // all done
  return SM_SUCCESS;

} // end SmSurfOfRevolution::CreateGeneratorFromAngle

/*******************************************************************//**
PURPOSE: Convert from STEP to NURBS parameterization.

NOTES: Same as ConverUVFromNURBSToSTEP:
  Many callers of this routine do not check the return value, they just use
  the result.  Therefore, always set the output to something.  A default
  value for that would be the same as the input value.
  Related to that, don't return ERR at all, just do our best.
***********************************************************************/
SmStatus SmSurfOfRevolution::ConvertUVFromSTEPToNURBS
  (const SmPoint2d & crSTEPUV,    // in : STEP Domain UV point
   SmPoint2d       & rNURBSUV)    // out: Nurb Domain UV point
  const
{
  // init output to something in case of failure
  rNURBSUV = crSTEPUV;

  if (!HavePolarConversion())
    {
      SmExtent2d sNatDomain = GetNaturalUVDomain();
      SmPoint2d sMapped;
      SER(m_vAnalUVDomain.Point2dMapToDomain(crSTEPUV,sNatDomain,m_bSwapUV,sMapped));

      // Use numerical techniques to convert to NURBS coordinates.
      SmPoint2d sNURBSGuess = sMapped;
      SmPoint3d s3DPnt;
      SER(EvaluateSTEPPoint(crSTEPUV,s3DPnt));
      SmBoolean bFoundAnswer;
      SmSolution sSolution;
      LocalPointSolve(GetNaturalUVDomain(),SM_SO_MINIMIZE,
                      s3DPnt,sNURBSGuess,bFoundAnswer,sSolution);
      // Always set output: note, LocalPointSolve will always return something
      // even if it fails, and the output will probably be better than the input.
      rNURBSUV.x = sSolution.m_vStart[0];
      rNURBSUV.y = sSolution.m_vStart[1];

#ifdef SM_DEBUG_CODE
      // check conversions reciprocity
      SmPoint2d sCheckUV ;
      ConvertUVFromNURBSToSTEP(rNURBSUV, sCheckUV) ;
      double dUVDist = (crSTEPUV - sCheckUV).Length() ;
      double dScaledUVZero = SM_EFF_ZERO * (1.0 + crSTEPUV.GetMaxDimension()) ;

      SM_ASSERT(dUVDist < dScaledUVZero) ;

      // check quailty of the conversion
      SmPoint3d sSTEPPoint, sNurbPoint ;
      Evaluate    (rNURBSUV, 0, 0, TRUE, TRUE, TRUE, &sNurbPoint) ;
      EvaluateSTEP(crSTEPUV, 0, 0, TRUE, TRUE, TRUE, &sSTEPPoint) ;
      double dDist = (sNurbPoint -sSTEPPoint).Length() ;
      double dScaledZero = SM_EFF_ZERO * (1.0 + sNurbPoint.GetMaxDimension()) ;

      SM_ASSERT(dDist < dScaledZero) ;
#endif

      return SM_SUCCESS;
    }

  // arrive here when polar conversions are possible

  //
  SmBoolean bExactConversion;
  SER(m_vPolarConverter.ConvertToNURBSParameter(crSTEPUV.x,rNURBSUV.x,bExactConversion));
  SER(m_pGenCurve->ConvertTFromSTEPToNURBS(crSTEPUV.y,rNURBSUV.y));

  // when the conversion was not exact
  if (!bExactConversion)
    {
      SmSurfParamType eSrfParam = SM_SP_V;
      if (m_bSwapUV)  eSrfParam = SM_SP_U;
      SmIsoCurve      sIsoCrv(*this,eSrfParam,rNURBSUV.y,TRUE);
      sIsoCrv.SetContext(NULL);
      SmPoint3d sPnt;
      SER(EvaluateSTEPPoint(crSTEPUV,sPnt));
      SmSolution sSol3;
      SmBoolean bFoundAnswer;
      SmExtent1d sIvl = sIsoCrv.GetNaturalInterval();

      // do a local point solve
      sIsoCrv.LocalPropertyAnalysis(sIvl,SM_CP_POINT_INVERSION,
          rNURBSUV.x,NULL,&sPnt,bFoundAnswer,sSol3);

      // Always set output; see above.
      rNURBSUV.x = sSol3.m_vStart[0];

    } // end not exact conversion check

  // swap output when required
  if (m_bSwapUV) { double dTemp = rNURBSUV.x ;
                   rNURBSUV.x   = rNURBSUV.y ;
                   rNURBSUV.y   = dTemp ;
                 }

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  // check conversions reciprocity
  SmPoint2d sCheckUV ;
  SmSurfOfRevolution::ConvertUVFromNURBSToSTEP(rNURBSUV, sCheckUV) ;
  double dUVDist       = (crSTEPUV - sCheckUV).Length() ;

  // SmZoneTol3d sZoneTol = SmTol::GetZoneTol3d(this);
  // SmTol2d sTol2d = SmTol::MapTo2d(sZoneTol, rNURBSUV, *this);

  SM_ASSERT(dUVDist < SM_EFF_ZERO_PARAM);
  if (dUVDist > SM_EFF_ZERO_PARAM)
  {
      TCHAR sBuff[SM_TBLOCK_SIZE];
      SM_SPRINTF(sBuff, _T("dUVDist: %16.16lf must be less than SM_EFF_ZERO_PARAM: %16.16lf\n"), dUVDist, SM_EFF_ZERO_PARAM);
      MYPRINTF(sBuff);           
  }


  // check quality of the conversion
  SmPoint3d sSTEPPoint[3], sNurbPoint[3] ; // ordered:[D, Dv, Du]
  Evaluate    (rNURBSUV, 1, 1, TRUE, TRUE, TRUE, sNurbPoint) ;
  EvaluateSTEP(crSTEPUV, 1, 1, TRUE, TRUE, TRUE, sSTEPPoint) ;
  SmVector3d sStepZ = sSTEPPoint[2] * sSTEPPoint[1] ;
  SmVector3d sNurbZ = sNurbPoint[2] * sNurbPoint[1] ;
  double sStepZLength = sStepZ.Length() ;
  double sNurbZLength = sNurbZ.Length() ;

  // positions should be the same
  double dDist = (sNurbPoint[0] - sSTEPPoint[0]).Length() ;
  double dScaled3dZero = SM_EFF_ZERO * (1.0 + sNurbPoint[0].GetMaxDimension()) ;
  SM_ASSERT(dDist < dScaled3dZero) ;

  // normals should be related
  double dScaledTangZero = 100.0 * SM_EFF_ZERO * (   1.0
                                          + sNurbPoint[1].GetMaxDimension()
                                          + sNurbPoint[2].GetMaxDimension()
                                          + sSTEPPoint[1].GetMaxDimension()
                                          + sSTEPPoint[2].GetMaxDimension()) ;
  if(   sNurbPoint[1].GetMaxDimension() < dScaledTangZero
     || sNurbPoint[2].GetMaxDimension() < dScaledTangZero
     || sSTEPPoint[1].GetMaxDimension() < dScaledTangZero
     || sSTEPPoint[2].GetMaxDimension() < dScaledTangZero)
    {
      // both tangents should go to zero at the same time
      if(m_bSwapUV)
        { SM_ASSERT(   (   sNurbPoint[1].GetMaxDimension() < dScaledTangZero
                        && sSTEPPoint[2].GetMaxDimension() < dScaledTangZero)
                    || (   sNurbPoint[2].GetMaxDimension() < dScaledTangZero
                        && sSTEPPoint[1].GetMaxDimension() < dScaledTangZero)) ;
        }
      else
        { SM_ASSERT(   (   sNurbPoint[1].GetMaxDimension() < dScaledTangZero
                        && sSTEPPoint[1].GetMaxDimension() < dScaledTangZero)
                    || (   sNurbPoint[2].GetMaxDimension() < dScaledTangZero
                        && sSTEPPoint[2].GetMaxDimension() < dScaledTangZero)) ;
        }
    }
  else // normals should equal +/- one another
    {
      sStepZ = sStepZ/sStepZLength ;
      sNurbZ = sNurbZ/sNurbZLength ;
      double dInvert = m_bSwapUV ? -1.0 : 1.0 ;

      // normals should be related
      SM_ASSERT(SM_ARE_SAME(1.0, dInvert * sStepZ.Dot(sNurbZ))) ;
    }

  if(bDebugMe)
    {
      Dump() ;
    }
#endif

  return SM_SUCCESS;

} // end SmSurfOfRevolution::ConvertUVFromSTEPToNURBS

/*******************************************************************//**
PURPOSE: Convert from NURBS to STEP parameterization.

NOTES: Same as ConverUVFromStepToNURBS:
  Many callers of this routine do not check the return value, they just use
  the result.  Therefore, always set the output to something.  A default
  value for that would be the same as the input value.
  Related to that, don't return ERR at all, just do our best.
***********************************************************************/
SmStatus SmSurfOfRevolution::ConvertUVFromNURBSToSTEP
  (const SmPoint2d & crNURBSUV,  // in : UV Point in Nurb domain
   SmPoint2d       & rSTEPUV)    // out: UV Point in STEP domain
  const
{
  // init output to something in case of failure
  rSTEPUV = (m_bSwapUV) ? SmPoint2d(crNURBSUV.y, crNURBSUV.x) : crNURBSUV;

  // get approximate NURB to Step MAP using a linear approximation
  SmExtent2d sNatDomain = GetNaturalUVDomain();

  // Point2dMapToDomain fails when crNURBSUV is not contained in sNatDomain.
  // Clamp NURBSUV value, then convert to get the guess.
  SmPoint2d sClampedNURBSUV(sNatDomain.ClampPoint2d(crNURBSUV));

  #ifdef SM_DEBUG_CODE
  if ((sClampedNURBSUV-crNURBSUV).Length() > SM_EFF_ZERO_PARAM)
  {
      TCHAR sBuff[SM_TBLOCK_SIZE];
      smos_sprintf(sBuff, _T("Converting UV point %f outside surface domain. "), (sClampedNURBSUV-crNURBSUV).Length());
      SM_DBG_WARN(sBuff);
  }
  #endif

  SER(sNatDomain.Point2dMapToDomain(sClampedNURBSUV,m_vAnalUVDomain,m_bSwapUV,rSTEPUV));

  // swap the NURB point coordinates when needed
  SmPoint2d sNURBSUV ;
  if (m_bSwapUV) { sNURBSUV.x   = crNURBSUV.y ;
                   sNURBSUV.y   = crNURBSUV.x ;
                 }
  else           { sNURBSUV.x   = crNURBSUV.x ;
                   sNURBSUV.y   = crNURBSUV.y ;
                 }

  // when polar conversion is possible
  SmBoolean bExactConversion = FALSE;
  if (HavePolarConversion())
    {
      // convert from Nurbs to STEP parameters
      SER(m_vPolarConverter.ConvertToPolarParameter(sNURBSUV.x,rSTEPUV.x,bExactConversion));
      SER(m_pGenCurve->ConvertTFromNURBSToSTEP(sNURBSUV.y,rSTEPUV.y));
    }

  // when conversion was not exact - use solution as input to a local solve
  if ( !bExactConversion )
    {
      // Use numerical techniques to convert to polar coordinates.
      SmPoint2d            sAnalGuess = rSTEPUV;
      const SmContext    * pContext   = GetContext();
      SmSurfOfRevolution * pSrfRev    = SM_CONST_CAST(SmSurfOfRevolution*,this);
      SmSTEPSurface      * pSTEPSrf   = new(*pContext) SmSTEPSurface(*pSrfRev,FALSE);
      SmObjDelete sClean(pSTEPSrf);

      SmPoint3d s3DPnt;
      SER(EvaluatePoint(crNURBSUV,s3DPnt));
      SmBoolean bFoundAnswer;
      SmSolution sSolution;
      pSTEPSrf->LocalPointSolve(
              pSTEPSrf->GetNaturalUVDomain(),
              SM_SO_INTERSECT,
              s3DPnt,
              sAnalGuess,
              bFoundAnswer,
              sSolution
          );
      // Always set output.  Note, LocalPointSolve will always return something
      // even if it fails, and the output will probably be better than the input.
      rSTEPUV.x = sSolution.m_vStart[0];
      rSTEPUV.y = sSolution.m_vStart[1];

      return SM_SUCCESS;
    }

  // all done
  return SM_SUCCESS;

} // end SmSurfOfRevolution::ConvertUVFromNURBSToSTEP

/*******************************************************************//**
PURPOSE: Drop a point to a surface of revolution very fast.
  Note that this is an optimization routine and should not be
  used for general dropping.
  For general surfaces this method returns an error.

NOTES:
  currently - Only returns points that are within the sweep interval
              and that solve to GenCurve when swung to the genCurve plane.
              Else finds no drop point and returns SM_ERR forcing the
              calling function to try again with a more global solver.

  Possible Change - DropPointFast could be made to return an
                    appropriate answer for important eSolverOperations
                    like Minimize, Normalize, and Intersect just
                    like SmPlane::DropPointFast.
***********************************************************************/
SmStatus SmSurfOfRevolution::DropPointFast              // eff: proj point to StartPlane and drop point to swept curve
  (const SmExtent2d      & crUVDomain,                  // in : NURB Domain of surface to search for solutions
   SmSolverOperationType   eSolverOperation,            // in : oneof: SM_SO_MINIMIZE, SM_SO_MAXIMIZE, SM_SO_NORMALIZE, SM_SO_INTERSECT
   const SmPoint3d       & crTestPoint,                 // in : target point
   const SmVector3d      * cpOptInPointingVector,       // in : specifies end (EndTang = cpOptInPointingVector) saved for drops on closed curve seams
   double                  dDistanceTolerance,          // in : Used to decide whether result in on a seam (and hence return two solutions)
   const double          * cpdOptTargetDistance,        // in : The pdOptTargetDistance, if not NULL, will be the corresponding
                                                        //      limit to a minimize/maximize operations.  In otherwords, it will
                                                        //      ask the solver to find a minimum value only if it is less than
                                                        //      the target distance or maximum value only if it is greater than
                                                        //      the target distance.
   SmSolutionRequestedType eSolutionRequested,          // in : SM_SR_SINGLE=get best solution, SM_SR_ALL=get all solutions
   SmSolutionArray       & rSolutions)                  // out: array of problem solutions reported as surface UV parameter values
   const
{
  // no work - can't convert to polar coordinates - does not have a planar generator
  if (!HavePolarConversion()) { return SM_ERR; }
  if (!m_bPlanarGenerator)    { return SM_ERR; }

  // init output
  rSolutions.ReSet();

  // get GenCurve position and requested derivative values
  // If we are allowing out-of-bounds eval, then the gen curve needs to too.
  // Note, we can't use the macro here because it only turns it on,
  // and so would have to be in an if/else, an so it would go out of scope
  // before we could use it.
  SmSurfOfRevolution * pNonConstThis = SM_CONST_CAST( SmSurfOfRevolution*, this );
  SmTemporaryChangeValue<SmBoolean> sChangeOB( m_pGenCurve  ->GetOutOfBoundsEnabled(),
                                               pNonConstThis->GetOutOfBoundsEnabled() );

  // construct and init global solver
  SmGlobalSolver sGS;
  sGS.Set3DTolerance      (dDistanceTolerance);
  sGS.SetSolverOperation  (eSolverOperation);
  sGS.SetSolutionRequested(SM_SR_ALL);
  sGS.SetOperationCategory(SM_OPERATION_MINIMIZE);
  sGS.SetSolutions        (&rSolutions);
  sGS.SetBestAnswerSoFarSq(SM_BIG_DOUBLE);
  if (cpdOptTargetDistance)
    {
      sGS.SetAtDistance   (*cpdOptTargetDistance);
    }

  // swing the point back to the plane of the generator curve - return error for exterior points
  ULONG lNumAngles;
  double adAnglesDeg[2];
  SmPoint3d sPointInPlane;
  SmBoolean bInside ;
  SER(TransformPointToStartPlane(crTestPoint,dDistanceTolerance,sPointInPlane,lNumAngles,adAnglesDeg, bInside));

  // bInside:     TRUE  - point is in sweep interval
  //              FALSE - point is not in sweep interval
  // lNumAngles:  1 - point is not on closed seam and within sweep interval
  //              2 - point is on closed seam or on a pole (a point on the rotation axis)
  // adAnglesDeg: is always set

  // when point is not in sweep interval
  if(!bInside)
    {
      // return SM_ERR forcing calling function to use a more general function.
      return(SM_ERR) ;
    }

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  // draw
  if(bDebugMe)
    {
      Dump() ;
      SM_DUMP_AND_ASSERT_VALID(this) ;

      SmFace *pFace = (SmFace *)GetFace() ;
      SmBrep *pBrep = pFace ? pFace->GetBrep() : NULL ;

      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,1) ; DrawUV() ; sm_GraphicsLoop() ;
      smgfx_SetLook(5,6, 1,0,0) ; crTestPoint.Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook(7,8, 0,1,0) ; sPointInPlane.Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook(9,10,1,1,0) ; m_vPosition.GetZAxis().Draw(&m_vPosition.GetOriginRef()) ; sm_GraphicsLoop() ;
      smgfx_SetLook(9,10,0,1,1) ; m_vPosition.GetXAxis().Draw(&m_vPosition.GetOriginRef()) ; sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;
  }
#endif // SM_DEBUG_CODE

  // For testPoints on seams and given a pointing vector - only keep the specified solution
  if(   lNumAngles == 2
     && cpOptInPointingVector)
    {
      double dDot = cpOptInPointingVector->Dot(m_vPosition.GetYAxisRef());
      if (dDot > SM_EFF_ZERO_SQRT)
        { // Keep start parameter of periodic surface
          lNumAngles = 1;
        }
      else if (dDot < -SM_EFF_ZERO_SQRT)
        { // Keep end parameter of periodic surface
          lNumAngles = 1;
          adAnglesDeg[0] = adAnglesDeg[1];
        }
    } // end need to cull multiple seam solutions check

  // get the generator curve's nurb interval
  SmExtent1d sGenCurveNurbIvl ;
  if (m_bSwapUV) { sGenCurveNurbIvl.SetMinMax(crUVDomain.GetMin().x,crUVDomain.GetMax().x); }
  else           { sGenCurveNurbIvl.SetMinMax(crUVDomain.GetMin().y,crUVDomain.GetMax().y); }

  // drop the rotated point to the generator curve
  SmSolution sSData[16];
  SmSolutionArray sSolutions(16,sSData);
  SER(m_pGenCurve->GlobalPointSolve(sGenCurveNurbIvl, eSolverOperation,
                                    sPointInPlane, dDistanceTolerance,
                                    cpdOptTargetDistance, NULL, eSolutionRequested,
                                    sSolutions));

#ifdef SM_DEBUG_CODE
  if(bDebugMe)
    {
      Dump() ;
      m_pGenCurve->Dump() ;
      sSolutions.Dump() ;

      SmFace *pFace = (SmFace *)GetFace() ;
      SmBrep *pBrep = pFace ? pFace->GetBrep() : NULL ;

      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,1) ; DrawUV() ; sm_GraphicsLoop() ;
      smgfx_SetLook(5,6, 1,0,0) ; crTestPoint.Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook(7,8, 0,1,0) ; sPointInPlane.Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook(9,10,0,0,1) ; for(ULONG ii=0;ii<sSolutions.GetSize();ii++)
                                   { SmPoint3d sGenDropPoint ;
                                     m_pGenCurve->EvaluatePoint(sSolutions[ii].m_vStart.m_adParameters[0], sGenDropPoint) ;
                                     //double dDist = (sPointInPlane - sGenDropPoint).Length() ;
                                     sGenDropPoint.Draw() ; sm_GraphicsLoop() ;
                                   }
      sm_GraphicsLoop() ;
  }
#endif // SM_DEBUG_CODE

  // init a solution object
  SmSolution sSol;
  sSol.m_eSolutionType = SM_ST_SINGLE_VALUE;
  sSol.m_lNumObjects   = 1;
  sSol.m_apObjects[0]  = SM_CONST_CAST(SmSurfOfRevolution*,this);
  sSol.m_apNodes[0]    = NULL;
  sSol.m_lNumVariables = 2;

  // for every rotation angle (normally 1, 2 for points on seams)
  for(ULONG i=0; i<lNumAngles && bInside; i++)
    {
      // convert the angles (degrees) to NURBS parameters
      double dUParameter;
      SmBoolean bExactConversion;
      SER(m_vPolarConverter.ConvertToNURBSParameter(adAnglesDeg[i],dUParameter,bExactConversion));

      // for every drop point to generator curve solution
      for (ULONG j=0; j<sSolutions.GetSize(); j++)
        {
          // get generator curve parameter
          const SmSolution & crSol = sSolutions[j];
          double             dT    = crSol.m_vStart[0];

          // Resolve ambiguities with cpOptInPointingVector if present.
          if (!bExactConversion)
            {
              // get generator curve param direction
              SmSurfParamType eSrfParam =   m_bSwapUV
                                          ? SM_SP_U
                                          : SM_SP_V ;

              // get surface isoCurve
              SmIsoCurve sIsoCrv(*this,eSrfParam,dT,TRUE);
              sIsoCrv.SetContext(NULL);
              SmExtent1d sNurbIvl = sIsoCrv.GetNaturalInterval();

              // when isoCurve is degenerate - use Interval min
              if (sIsoCrv.IsDegenerate())
                {
                  dUParameter = sNurbIvl.GetMin();
                }
              else // isoCurve is not degenerate
                {
                  // get the 3dDropPoint
                  SmPoint3d sPnt;
                  SmPoint2d sSTEPUV(adAnglesDeg[i],dT);
                  SER(EvaluateSTEPPoint(sSTEPUV,sPnt));

                  // transform the DropPoint coordinates
                  SmSolution sSol3;
                  SmBoolean bFoundAnswer;
                  if(SM_SUCCESS != sIsoCrv.LocalPropertyAnalysis(sNurbIvl,
                                                   SM_CP_POINT_INVERSION,
                                                   dUParameter,
                                                   NULL,
                                                   &sPnt,
                                                   bFoundAnswer,
                                                   sSol3))
                    { return SM_ERR; }

                  // save the parameter
                  if (bFoundAnswer)
                    { dUParameter = sSol3.m_vStart[0]; }
                  else
                    { return SM_ERR; }
                } // end getting isoCurve parameter for drop point
            } // when exact NURB/polar transformations are not possible check

          // when more than 1 drop point was found and a specified direction is given for a periodic genCurve
          if (   sSolutions.GetSize() > 1
              && cpOptInPointingVector
              && m_pGenCurve->IsPeriodic(sGenCurveNurbIvl))
            {
              // skip solution when solution is ivl min
              if (SM_ARE_SAME(dT,sGenCurveNurbIvl.GetMin()))
                {
                  // and PointingVector opposes curve tangent
                  SmVector3d sPV[2];
                  SER(m_pGenCurve->Evaluate(dT,1,TRUE,sPV));
                  double dDot = sPV[1].Dot(*cpOptInPointingVector);
                  if (dDot < -SM_EFF_ZERO_SQRT)
                    { continue; }
                }

              // skip solution when solution is ivl max
              if (SM_ARE_SAME(dT,sGenCurveNurbIvl.GetMax()))
                {
                  // and PointingVector does not oppose curve tangent
                  SmVector3d sPV[2];
                  SER(m_pGenCurve->Evaluate(dT,1,TRUE,sPV));
                  double dDot = sPV[1].Dot(*cpOptInPointingVector);
                  if (dDot > SM_EFF_ZERO_SQRT)
                    { continue; }
                }
            } // end need to pick between two solutions check

          // set the solution parameters
          if (m_bSwapUV) { sSol.m_vStart[0] = crSol.m_vStart[0];
                           sSol.m_vStart[1] = dUParameter;
                         }
          else           { sSol.m_vStart[0] = dUParameter;
                           sSol.m_vStart[1] = crSol.m_vStart[0];
                         }

          // get the surface UVpoint
          SmPoint2d sUV(sSol.m_vStart[0],sSol.m_vStart[1]);
          if (!bExactConversion)
            {
              SmBoolean bFoundAnswer;
              SER(LocalPointSolve(crUVDomain,eSolverOperation,crTestPoint,sUV,bFoundAnswer,sSol));
              if (bFoundAnswer)
                {
                  sUV.x = sSol.m_vStart[0];
                  sUV.y = sSol.m_vStart[1];
                }
            } // end no exact conversion check

          // Skip solution if not in requested domain
          if ( !crUVDomain.ContainsPoint2d(sUV) )
          { continue; }

          // set SolutionValue = dropSurfacePoint to TestPoint distance
          SmPoint3d sSurfPnt;
          SER(EvaluatePoint(sUV,sSurfPnt));
          sSol.m_vStart.m_dSolutionValue = crTestPoint.DistanceBetween(sSurfPnt);

#ifdef SM_DEBUG_CODE
  // draw
  if(bDebugMe)
    {
      Dump() ;

      SmFace *pFace = (SmFace *)GetFace() ;
      SmBrep *pBrep = pFace ? pFace->GetBrep() : NULL ;
      double  dParam    = m_bSwapUV ? sSol.m_vStart[0] : sSol.m_vStart[1] ;
      SmPoint3d sDropInPlane ;
      m_pGenCurve->EvaluatePoint( dParam, sDropInPlane) ;


      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,1) ; DrawUV() ; sm_GraphicsLoop() ;
      smgfx_SetLook(5,6, 1,0,0) ; crTestPoint.Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook(7,8, 0,1,0) ; sPointInPlane.Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook(9,10,0,0,1) ; sDropInPlane.Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook(9,10,0,0,1) ; sSurfPnt.Draw() ; sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;
  }
#endif // SM_DEBUG_CODE

          // skip intersection solutions whose value exceed the DistanceTolerance
          if (   eSolverOperation == SM_SO_INTERSECT
              && sSol.m_vStart.m_dSolutionValue > dDistanceTolerance)
            { continue; }

          // put the solution into the output
          sGS.AddSortedSolution(sSol,SM_SK_BY_SOLUTION_VALUE, TRUE);
        } // end iter every drop point to generator curve solution
    } // end iter every point rotation angle

  // all done
  return SM_SUCCESS;

} // end SmSurfOfRevolution::DropPointFast

//      /*******************************************************************//**
//      PURPOSE: Default evaluation of STEP is to use standard evaluation.
//
//      NOTES: See SmSurfOfRevolution.h
//      ***********************************************************************/
//      SmStatus SmSurfOfRevolution::EvaluateSTEP
//        (const SmPoint2d & crUV,    // in : target surface point
//         ULONG lHighestUDeriv,      // in : Requested highest U derivative
//         ULONG lHighestVDeriv,      // in : Requested highest V derivative
//         SmBoolean ,                // in : bUFromLeft = if P is on U interval boundary
//                                    //      TRUE  = evaluate P in upper interval where P is on the left of the interval
//                                    //      FALSE = evaluate P in lower interval where P is on the right of the interval
//         SmBoolean ,                // in : bVFromLeft = if P is on V interval boundary
//                                    //      TRUE  = evaluate P in upper interval where P is on the left of the interval
//                                    //      FALSE = evaluate P in lower interval where P is on the right of the interval
//         SmBoolean ,                // in : bOnlyUpperHalf = TRUE=compute upper half of matrix only
//                                    //      ex. 1,1 = [D  Du] 2,2 = [D    Du    Duu] where -- = an untouched memory value
//                                    //                [Dv --]       [Dv   Duv   ---]
//                                    //                              [Dvv  ---   ---]
//         SmVector3d *aDerivatives)  // out: matrix of evaluations values
//        const                       //      sized:[lHighestUDeriv+1][lHighestVDeriv+1]
//                                    //      2d organized: [D    Du    Duu    Duuu    Duuuu   ]
//                                    //                    [Dv   Duv   Duuv   Duuuv   Duuuuv  ]
//                                    //                    [Dvv  Duvv  Duuvv  Duuuvv  Duuuuvv ]
//                                    //                    [Dvvv Duvvv Duuvvv Duuuvvv Duuuuvvv]
//                                    //      1d organized: [D, Dv, Dvv, Dvvv,.. Du, Duv, Duvv, Duvvv,.. Duu, Duuv, Duuvv, Duuvvv,...]
//      {
//        // check input
//        if (   lHighestUDeriv > 1
//            || lHighestVDeriv > 1) SER(SM_ERR); // Not implemented
//        SM_ASSERT(lHighestUDeriv == lHighestVDeriv) ;
//
//        // clamp input point to analytic domain
//        SmPoint2d sUV = crUV;
//        if ( !IsOutOfBoundsEnabled() && !m_vAnalUVDomain.ContainsPoint2d( sUV ) )
//          { sUV = m_vAnalUVDomain.ClampPoint2d(sUV); }
//
//        // evaluate the point position
//        SER(EvaluateSTEPPoint(sUV,aDerivatives[0]));
//
//        // evaluate the derivatives
//        SmVector3d sZAxis = m_vPosition.GetZAxis();
//        if (lHighestVDeriv > 0)
//          {
//            SmPoint3d sPV[2];
//            SER(m_pGenCurve->EvaluateSTEP(sUV.y,1,TRUE,sPV));
//            aDerivatives[lHighestVDeriv] = sPV[1];
//            SmAxis2Placement sRot;
//            sRot.RotateAboutAxis(sUV.x*SM_PI/180.0,sZAxis);
//            sRot.TransformVector(aDerivatives[lHighestVDeriv],aDerivatives[lHighestVDeriv]);
//          }
//        if (lHighestUDeriv > 0)
//          {
//            double dRadius;
//            const SmPoint3d  &rLinePnt = m_vPosition.GetOriginRef();
//            SmVector3d        sLineVec = sZAxis;
//            SER(smgu_LinePointDistance(rLinePnt,sLineVec,aDerivatives[0],dRadius));
//            if (dRadius < SM_EFF_ZERO)
//              {
//                aDerivatives[lHighestVDeriv+1].Set(0.0,0.0,0.0);
//              }
//            else
//              {
//                const SmVector3d &rXAxis = m_vPosition.GetXAxisRef();
//                const SmVector3d &rYAxis = m_vPosition.GetYAxisRef();
//                double dAngleRad = SM_DEG2RAD(sUV.x);
//                SmVector3d sUnScaled =   dRadius
//                                      * ( -smos_Sine(dAngleRad)   * rXAxis
//                                         + smos_Cosine(dAngleRad) * rYAxis);
//                aDerivatives[lHighestVDeriv+1] = sUnScaled * SM_PI / 180.0;
//              }
//          }
//
//      #ifdef SM_DEBUG_CODE
//      SmBoolean bDebugMe = FALSE;
//        if (bDebugMe)
//          {
//            smgfx_SetColor(1,0,0);
//            aDerivatives[0].Draw(); sm_GraphicsLoop();
//            aDerivatives[lHighestVDeriv].Draw(&aDerivatives[0]); sm_GraphicsLoop();
//            smgfx_SetColor(0,0.6,0);
//            SmVector3d sVec = aDerivatives[lHighestVDeriv+1] * 180.0 / SM_PI;
//            sVec.Draw(&aDerivatives[0]); sm_GraphicsLoop();
//            sm_GraphicsLoop();
//          }
//      #endif
//
//        return SM_SUCCESS;
//
//      } // end SmSurfOfRevolution::EvaluateSTEP

/*******************************************************************//**
PURPOSE: Evaluate the point & derivatives using the STEP parameterization.
            EvaluatePoint(NurbUV) = EvaluateSTEPPoint(StepUV).
            But tangents will vary in magnitude and
            norm may be negated

NOTES:
  P(V) = GenCurve(V)
  Px   = (P-Origin).Dot(XAxis)
  Py   = (P-Origin).Dot(YAxis)
  Pz   = (P-Origin).Dot(ZAxis)
  SurfOfRevolution(U,V) =   origin + [ cos(U)  sin(U)] [Px] [XAxis]
                                     [-sin(U)  cos(U)] [Py] [YAxis]
                                   + Pz * ZAxis ;
***********************************************************************/
SmStatus SmSurfOfRevolution::EvaluateSTEP
  (const SmPoint2d & crUV,    // in : U=CCW Rot about Z from X in degrees,          [0 to 360]
                              //      V=param from bot pole to top pole in degrees, [-90 to 90]
                              //      when m_bMakeNurbGenCurve = FALSE, V is angle in degrees
                              //           m_bMakeNurbGenCurve = TRUE , V is close to an angle in degrees
   ULONG lHighestUDeriv,      // in : Requested triangular derivative order in U; must equal V; max 3
   ULONG lHighestVDeriv,      // in : Requested triangular derivative order in V; must equal U; max 3
   SmBoolean ,                // in : not used - if P is on U interval boundary
                              //      TRUE  = evaluate P in upper interval where P is on the left of the interval
                              //      FALSE = evaluate P in lower interval where P is on the right of the interval
   SmBoolean ,                // in : not used - if P is on V interval boundary
                              //      TRUE  = evaluate P in upper interval where P is on the left of the interval
                              //      FALSE = evaluate P in lower interval where P is on the right of the interval
   SmBoolean ,                // in : not used output always set for bOnlyUpperHalf=TRUE
   SmVector3d *aDerivatives)  // out: matrix of evaluations values
  const                       //      sized:[lHighestUDeriv+1][lHighestVDeriv+1]
                              //      2d organized: [D    Du   ]
                              //                    [Dv   -    ]
                              //      1d organized: [D, Dv, Dvv, Dvvv,.. Du, Duv, Duvv, Duvvv,.. Duu, Duuv, Duuvv, Duuvvv,...]
{
  // check input
  if (   lHighestUDeriv > 3
      || lHighestVDeriv > 3) SER(SM_ERR); // Not implemented
  SM_ASSERT(lHighestUDeriv == lHighestVDeriv) ;

  // locals
  SmPoint2d  sUV         = crUV ;
  SmPoint3d  rO          = m_vPosition.GetOriginRef() ;
  SmVector3d rX          = m_vPosition.GetXAxisRef () ;
  SmVector3d rY          = m_vPosition.GetYAxisRef () ;
  SmVector3d sZ          = m_vPosition.GetZAxis () ;
  SmExtent1d sLinearIvl  = m_vAnalUVDomain.GetVInterval() ;
  ULONG      lMinHighest = smos_Min(lHighestUDeriv, lHighestVDeriv) ;

  // per SMLib practice, clamp out of bounds UV points
  if (!IsOutOfBoundsEnabled() && !m_vAnalUVDomain.ContainsPoint2d(sUV))
    { sUV = m_vAnalUVDomain.ClampPoint2d(sUV); }

  // get GenCurve position and requested derivative values
  // If we are allowing out-of-bounds eval, then the gen curve needs to too.
  // Note, we can't use the macro here because it only turns it on,
  // and so would have to be in an if/else, an so it would go out of scope
  // before we could use it.
  SmSurfOfRevolution * pNonConstThis = SM_CONST_CAST( SmSurfOfRevolution*, this );
  SmTemporaryChangeValue<SmBoolean> sChangeOB( m_pGenCurve  ->GetOutOfBoundsEnabled(),
                                               pNonConstThis->GetOutOfBoundsEnabled() );

  // evaluate expensive trigonometric functions
  double dCosU = smos_Cosine(sUV.x*SM_PI/180.0) ;
  double dSinU = smos_Sine  (sUV.x*SM_PI/180.0) ;

  double dV    = sUV.y ;
  SmVector3d sPV[4] ;
  SER(m_pGenCurve->EvaluateSTEP(dV,lMinHighest,TRUE,sPV));

  SmVector3d sP = sPV[0] - rO ;
  double    dPx = sP.Dot(rX) ;
  double    dPy = sP.Dot(rY) ;
  double    dPz = sP.Dot(sZ) ;

  // evaluate position
  aDerivatives[0] = rO + ( dCosU * dPx + -dSinU * dPy) * rX
                       + ( dSinU * dPx +  dCosU * dPy) * rY
                       + dPz * sZ ;
  // derivatives
  //       Su   =   (-dSinU * dPx + -dCosU * dPy) * rX
  //              + ( dCosU * dPx + -dSinU * dPy) * rY
  //       Sv   =   ( dCosU * sPV[1].Dot(rX) + -dSinU * sPV[1].Dot(rY)) * rX
  //              + ( dSinU * sPV[1].Dot(rX) +  dCosU * sPV[1].Dot(rY)) * rY
  //              + sPV[1].Dot(sZ) * sZ ;
  //       Suu  =   (-dCosU * dPx +  dSinU * dPy) * rX
  //              + (-dSinU * dPx + -dCosU * dPy) * rY
  //       Svv  =   ( dCosU * sPV[2].Dot(rX) + -dSinU * sPV[2].Dot(rY)) * rX
  //              + ( dSinU * sPV[2].Dot(rX) +  dCosU * sPV[2].Dot(rY)) * rY
  //              + sPV[2].Dot(sZ) * sZ ;
  //       Suv  =   (-dSinU * sPV[1].Dot(rX) + -dCosU * sPV[1].Dot(rY)) * rX
  //              + ( dCosU * sPV[1].Dot(rX) + -dSinU * sPV[1].Dot(rY)) * rY
  //       Suuu =   ( dSinU * dPx +  dCosU * dPy) * rX
  //              + (-dCosU * dPx +  dSinU * dPy) * rY
  //       Suuv =   (-dCosU * sPV[1].Dot(rX) +  dSinU * sPV[1].Dot(rY)) * rX
  //              + (-dSinU * sPV[1].Dot(rX) + -dCosU * sPV[1].Dot(rY)) * rY
  //       Suvv =   (-dSinU * sPV[2].Dot(rX) + -dCosU * sPV[2].Dot(rY)) * rX
  //              + ( dCosU * sPV[2].Dot(rX) + -dSinU * sPV[2].Dot(rY)) * rY
  //       Svvv =   ( dCosU * sPV[3].Dot(rX) + -dSinU * sPV[3].Dot(rY)) * rX
  //              + ( dSinU * sPV[3].Dot(rX) +  dCosU * sPV[3].Dot(rY)) * rY
  //              + sPV[3].Dot(sZ) * sZ ;


  // 1st U and V derivatives
  if(lHighestUDeriv >= 1)   { aDerivatives[  lHighestVDeriv+1] =   (-dSinU * dPx + -dCosU * dPy) * rX
                                                                 + ( dCosU * dPx + -dSinU * dPy) * rY ;
                            }
  if(lHighestVDeriv >= 1)   { aDerivatives[1]                  =   ( dCosU * sPV[1].Dot(rX) + -dSinU * sPV[1].Dot(rY)) * rX
                                                                 + ( dSinU * sPV[1].Dot(rX) +  dCosU * sPV[1].Dot(rY)) * rY
                                                                 + sPV[1].Dot(sZ) * sZ ;
                            }
  // cross derivative dUV
  if(   lHighestUDeriv >= 1
     && lHighestVDeriv >= 1
     && lMinHighest    >= 2){ aDerivatives[  lHighestVDeriv+2] =   (-dSinU * sPV[1].Dot(rX) + -dCosU * sPV[1].Dot(rY)) * rX
                                                                 + ( dCosU * sPV[1].Dot(rX) + -dSinU * sPV[1].Dot(rY)) * rY ;
                            }

  //  cross derivative dUVV
  if(   lHighestUDeriv >= 1
     && lHighestVDeriv >= 2
     && lMinHighest    >= 3){ aDerivatives[  lHighestVDeriv+3] =   (-dSinU * sPV[2].Dot(rX) + -dCosU * sPV[2].Dot(rY)) * rX
                                                                 + ( dCosU * sPV[2].Dot(rX) + -dSinU * sPV[2].Dot(rY)) * rY ;
                            }

  // 2nd U and V derivatives
  if(lHighestUDeriv >= 2)   { aDerivatives[2*lHighestVDeriv+2] =   (-dCosU * dPx +  dSinU * dPy) * rX
                                                                 + (-dSinU * dPx + -dCosU * dPy) * rY ;
                            }
  if(lHighestVDeriv >= 2)   { aDerivatives[2] =   ( dCosU * sPV[2].Dot(rX) + -dSinU * sPV[2].Dot(rY)) * rX
                                                + ( dSinU * sPV[2].Dot(rX) +  dCosU * sPV[2].Dot(rY)) * rY
                                                + sPV[2].Dot(sZ) * sZ ;
                            }

  // cross derivative dUUV
  if(   lHighestUDeriv >= 2
     && lHighestVDeriv >= 1
     && lMinHighest    >= 3){ aDerivatives[2*lHighestVDeriv+3] =   (-dCosU * sPV[1].Dot(rX) +  dSinU * sPV[1].Dot(rY)) * rX
                                                                 + (-dSinU * sPV[1].Dot(rX) + -dCosU * sPV[1].Dot(rY)) * rY ;
                            }
  // 3rd U and V derivatives
  if(lHighestUDeriv >= 3)   { aDerivatives[3*lHighestVDeriv+3] =   ( dSinU * dPx +  dCosU * dPy) * rX
                                                                 + (-dCosU * dPx +  dSinU * dPy) * rY ;
                            }
  if(lHighestVDeriv >= 3)   { aDerivatives[3] =   ( dCosU * sPV[3].Dot(rX) + -dSinU * sPV[3].Dot(rY)) * rX
                                                + ( dSinU * sPV[3].Dot(rX) +  dCosU * sPV[3].Dot(rY)) * rY
                                                + sPV[3].Dot(sZ) * sZ ;
                            }

  // The formulas above are per radian; STEP parameters are in degrees (U is degrees; V is the generating-curve parameter).
  smsurf_ScaleSTEPAngularDerivatives(aDerivatives, lHighestUDeriv, lHighestVDeriv, TRUE, FALSE) ;

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
  if (bDebugMe)
    {
      smgfx_SetColor(1,0,0);
      aDerivatives[0].Draw();
      sm_GraphicsLoop();
      aDerivatives[lHighestVDeriv].Draw(&aDerivatives[0]);
      sm_GraphicsLoop();
      smgfx_SetColor(0,0.6,0);
      SmVector3d sVec = aDerivatives[lHighestVDeriv+1] * 180.0 / SM_PI;
      sVec.Draw(&aDerivatives[0]);
      sm_GraphicsLoop();
    }
#endif

  // all done
  return SM_SUCCESS;

} // end SmSurfOfRevolution::EvaluateSTEP

/*******************************************************************//**
PURPOSE: Evaluate the point using the STEP parameterization.

NOTES: Given UV point = [x,y]
                  x = angle in degrees
                  y = parameter along generator curve
         P(y)  = GeneratorCurve(y)
  Surface(x,y) = center
                 + cos(x)       * A
                 + sin(x)       * B
                 + (P-C).Dot(Z) * Z ;
      where C = rotation Center Point
            Z = rotation Axis
            A = Projection of (P-C) onto Plane(C,Z)
                When GeneratorCurve is planar in (C,X,Z) plane (usual case)
                   A will be in the direction of XAxis and
                   B will be in the direction of YAxis.
            B = Orthogonal Vector to A on Plane(C,Z)

  NOTE: m_bSwapUV is not used to make this evaluation!
***********************************************************************/
SmStatus SmSurfOfRevolution::EvaluateSTEPPoint
  (const SmPoint2d & crUV,  // in : crUV.x = angle in degrees CCW
                            //      crUV.y = Gen Curve parameter
   SmPoint3d & rPoint)      // out:
  const
{
  // pass the call along
  return( EvaluateSTEP(crUV, 0, 0, TRUE, TRUE, TRUE, &rPoint) ) ;

//        SmPoint3d sUV = crUV;
//
//        // clamp input point to analytic domain
//        if ( !IsOutOfBoundsEnabled() && !m_vAnalUVDomain.ContainsPoint2d( crUV ) )
//          {
//            sUV = m_vAnalUVDomain.ClampPoint2d(crUV);
//          }
//
//        // evaluate point along generator curve
//        SmPoint3d sP;
//        SER(m_pGenCurve->EvaluateSTEP(crUV.y,0,TRUE,&sP));
//
//        // get vector from Center to generator point
//        const SmPoint3d  &rC    = m_vPosition.GetOriginRef();
//        SmVector3d        sP_sC = sP-rC;
//
//        // get rotation axis
//        SmVector3d sZ = m_vPosition.GetZAxis();
//
//        // get cos(UV.x) and sin(UV.x)
//        double dAngleRad = crUV.x * SM_PI / 180.0;
//        double dCosU     = smos_Cosine(dAngleRad);
//        double dSinU     = smos_Sine  (dAngleRad);
//
//        // get vector perp to PC on plane perp to Z
//        SmVector3d sVCross = sZ * sP_sC;
//
//        // compute the point
//        // Point = center + PointHeight + rotation about Z
//        rPoint =   rC
//                 + (1.0-dCosU) * sZ.Dot(sP_sC) * sZ
//                 + dCosU * (sP_sC)
//                 + dSinU * sVCross ;
//
//      #ifdef SM_DEBUG_CODE
//      SmBoolean bDebugMe = FALSE ;
//        if(bDebugMe)
//          {
//            double     dHeight      = sZ.Dot(sP_sC) ;
//            SmVector3d sX           = sP_sC - dHeight * sZ ;
//            SmVector3d sY           = sZ * sX ;
//            double     dXLength     = sX.Length() ;
//            double     dYLength     = sY.Length() ;
//            double     dScaledZero  = SM_EFF_ZERO * (1.0 + dXLength) ;
//            SM_ASSERT(SM_ARE_SAME_TO_TOL(dXLength, dYLength, dScaledZero)) ;
//            SmPoint3d  sCmpPoint    = rC
//                                      + dHeight * sZ
//                                      + dCosU   * sX
//                                      + dSinU   * sY ;
//            double     dScaledZero2 = SM_EFF_ZERO * (1.0 + rPoint.GetMaxDimension()) ;
//            double     dDist        = (rPoint - sCmpPoint).Length() ;
//            SM_ASSERT(SM_IS_ZERO_TO_TOL(dDist, dScaledZero2)) ;
//          }
//      #endif // SM_DEBUG_CODE
//
//        return SM_SUCCESS;

} // end SmSurfOfRevolution::EvaluateSTEPPoint

/*******************************************************************//**
PURPOSE: This virtual method may invoke special cases of intersection
    of analytics with planes.

NOTES:
***********************************************************************/
SmStatus SmSurfOfRevolution::GlobalSurfaceIntersect
  (const SmContext     & crContext,               // in : Context for the creation of curves
   const SmExtent2d    & crUVDomain,              //
   const SmSurface     & crOtherSurface,          // in : target 2nd intersecting surface
   const SmExtent2d    & crOtherUVDomain,         //
   const SmBoolean       bUseSurfaceEdges[2],     // in : Normally both are TRUE unless you know that the edges of one surface do not intersect
                                                  //      the other surface.  It is a slight optimization to set the flag to FALSE
   const SmApproxTol3d * pdOptApproxTol3d,        // in : approximation tolerance If not given it uses 1/1000 of surface size
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
            SER(IntersectWithPlane(crContext,crUVDomain,(SmPlane&)crOtherSurface,
                  crOtherUVDomain, bUseSurfaceEdges, pdOptApproxTol3d,
                  pdOptAngTolRad, bNeedNurbIntersection,
                  pOpt3DCurves, pOptSurface1UVCurves,
                  pOptSurface2UVCurves, pOptCurveTypes, pOptDeviations));
            break;

        case SmSphere_TYPE:
            SER(IntersectWithSphere(crContext,crUVDomain,(SmSphere&)crOtherSurface,
                  crOtherUVDomain, bUseSurfaceEdges, pdOptApproxTol3d,
                  pdOptAngTolRad, bNeedNurbIntersection,
                  pOpt3DCurves, pOptSurface1UVCurves,
                  pOptSurface2UVCurves, pOptCurveTypes, pOptDeviations));
            break;

        case SmCone_TYPE:
            SER(IntersectWithCone(crContext,crUVDomain,(SmCone&)crOtherSurface,
                  crOtherUVDomain, bUseSurfaceEdges, pdOptApproxTol3d,
                  pdOptAngTolRad, bNeedNurbIntersection,
                  pOpt3DCurves, pOptSurface1UVCurves,
                  pOptSurface2UVCurves, pOptCurveTypes, pOptDeviations));
            break;

        case SmSurfOfRevolution_TYPE:
            SER(IntersectWithSurfOfRevolution(crContext,crUVDomain,
                  (SmSurfOfRevolution&)crOtherSurface,
                  crOtherUVDomain, bUseSurfaceEdges, pdOptApproxTol3d,
                  pdOptAngTolRad, bNeedNurbIntersection,
                  pOpt3DCurves, pOptSurface1UVCurves,
                  pOptSurface2UVCurves, pOptCurveTypes, pOptDeviations));
            break;

        default:
            break;
      }

    // If we're here, then analytic intersections were not found
    if (bNeedNurbIntersection)
      {
        SER(SmSurface::GlobalSurfaceIntersect
             (crContext,crUVDomain,crOtherSurface,
              crOtherUVDomain,bUseSurfaceEdges,pdOptApproxTol3d,
              pdOptAngTolRad,pOpt3DCurves,pOptSurface1UVCurves,
              pOptSurface2UVCurves,pOptCurveTypes,pOptDeviations));
      }

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  // draw Brep1(Blue), Brep2(green), Surf1-Face1(cyan-black), Surf2-Face2(yellow-black)
  //      s3DCurves(from Red to magenta)
  if (bDebugMe)
    {
      ULONG ii, lCurveCount = pOpt3DCurves ? pOpt3DCurves->GetSize() : 0 ;
      SmFace *pFace1 = (SmFace *)this->GetFace() ;
      SmFace *pFace2 = (SmFace *)crOtherSurface.GetFace() ;
      SmBrep *pBrep1 = (pFace1) ? pFace1->GetBrep() : NULL ;
      SmBrep *pBrep2 = (pFace2) ? pFace2->GetBrep() : NULL ;

      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep1) { pBrep1->Draw(TRUE) ; } sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,0) ; if(pBrep2) { pBrep2->Draw(TRUE) ; } sm_GraphicsLoop() ;

      smgfx_SetLook(1,2, 0,1,1) ; this->DrawUV(6,6,FALSE,NULL,pBrep1==NULL) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,0,0) ; if(pFace1) { pFace1->Draw(SM_DM_CROSSHATCH) ; }sm_GraphicsLoop() ;

      smgfx_SetLook(1,2, 1,1,0) ; crOtherSurface.DrawUV(6,6,FALSE,NULL,pBrep2==NULL) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,0,0) ; if(pFace2) { pFace2->Draw(SM_DM_CROSSHATCH) ; }sm_GraphicsLoop() ;

      for (ii=0; ii<lCurveCount; ii++)
        {
          SmCurve   * pCurve = pOpt3DCurves->GetAt(ii) ;
          SmExtent1d  sInt   = pOpt3DCurves->GetAt(ii)->GetNaturalInterval();
          smgfx_SetLook(3+ii, 5+ii, 1, 0, lCurveCount == 1 ? 1.0 : (double)ii/(double)(lCurveCount-1)) ;
          pCurve->Draw(&sInt); sm_GraphicsLoop();
          SM_ASSERT_VALID(((SmBSplineCurve *)pCurve)) ;
        }
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

    return SM_SUCCESS;

} // end SmSurfOfRevolution::GlobalSurfaceIntersect

/*******************************************************************//**
PURPOSE: Find intersection curve (line) of this with other plane

NOTES:
***********************************************************************/
SmStatus SmSurfOfRevolution::IntersectWithPlane
  (const SmContext     & crContext,                  // in : context for new object construction
   const SmExtent2d    & crUVDomain,                 // NotUsed: in : intersection limit for this surface
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
   SmTArray<SmTsectCurveType> * pOptCurveTypes,      // out: oneof for each 3DCurve, NULL to ignore
                                                     //      SM_TC_TOUCHING        - single point intersection (surf norms parallel)
                                                     //      SM_TC_CROSSING        - curve intersection (surf norms not parallel)
                                                     //      SM_TC_TANGENT         - curve intersection (surf norms parallel)
                                                     //      SM_TC_COINCIDENT      - curve intersection (surf norms parallel & cross-tangents equal)
                                                     //      SM_TC_NEAR_TANGENT    - curve has small angle of intersection
                                                     //      SM_TC_REGION_BOUNDARY - curve bounds region within which the surfs are coincident
   SmTArray<double>    * pOptDeviations)             // out: associated max 3DCurve to surface distance, NULL to ignore
 const
{
  SM_REF1(crUVDomain) ;
  SM_DUMP_AND_ASSERT2_VALID(this) ;
#ifdef SM_DEBUG_CODE
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
      smgfx_SetLook(3,5, 1,0,0) ; m_pGenCurve->Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 1,0,1) ; m_vPosition.Draw( (SmExtent2d*)(&m_vAnalUVDomain) ) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 1,1,0) ; crPlane.DrawUV() ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,0,0) ; if(pFace1) { pFace1->Draw(SM_DM_CROSSHATCH,10,10) ; } sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,0,0) ; if(pFace2) { pFace2->Draw(SM_DM_CROSSHATCH,10,10) ; } sm_GraphicsLoop() ;
      sm_GraphicsLoop();
    }
#endif

  // init output
  rbNeedsMoreIntersections = FALSE;
  if (pOpt3DCurves)         { pOpt3DCurves->ReSet(); }
  if (pOptSurface1UVCurves) { pOptSurface1UVCurves->ReSet(); }
  if (pOptSurface2UVCurves) { pOptSurface2UVCurves->ReSet(); }
  if (pOptCurveTypes)       { pOptCurveTypes->ReSet(); }
  if (pOptDeviations)       { pOptDeviations->ReSet(); }

  // use general intersection for nonPlanar generators
  if (m_bPlanarGenerator == FALSE)
    {
      rbNeedsMoreIntersections = TRUE;
      return SM_SUCCESS;
    }

  // plane and RevolutionSurface locals
  ULONG ii, jj ;
  SmVector3d        sPlaneNormal   = crPlane.GetPosition().GetZAxis();
  const SmPoint3d  &rPlaneOrigin   = crPlane.GetPosition().GetOriginRef();
  SmVector3d sSurfOfRevolutionAxis = m_vPosition.GetZAxis();
  double     dAngTolRad            = SM_EFF_ZERO_SQRT;
  double     dTol                  =  0.0;
  if( pdOptApproxTol3d ) dTol = *pdOptApproxTol3d;
  else dTol = SM_EFF_ZERO_SQRT;

  // If we are allowing out-of-bounds eval, then the gen curve needs to too.
  SmSurfOfRevolution * pNonConstThis = SM_CONST_CAST( SmSurfOfRevolution*, this );
  SmTemporaryChangeValue<SmBoolean> sChangeOB( m_pGenCurve  ->GetOutOfBoundsEnabled(),
                                               pNonConstThis->GetOutOfBoundsEnabled() );

  // get plane bounding box increased by tolerance
  SmPseudoBox sPlanePBox ;
  crPlane.CalculateBoundingBox(crPlane.GetNaturalUVDomain(),
                               NULL, &sPlanePBox, NULL) ;
  sPlanePBox.ExpandAbsolute(dTol) ;

  // when RevolutionAxis/PlaneNormal are perpendicular
  //   XSect = Generator rotated to the plane when plane is coincident to revolutionAxis
  if (sSurfOfRevolutionAxis.IsPerpendicularTo(sPlaneNormal,SM_RAD2DEG(dAngTolRad)))
    {
      // check revolutionAxis is coincident with Plane
      SmVector3d sVecBetween        = m_vPosition.GetOriginRef() - rPlaneOrigin;
      double     dDistPlaneToCenter = sVecBetween.Dot(sPlaneNormal);
      if (smos_Fabs(dDistPlaneToCenter) > dTol)
        {
          // RevolutionAxis not coincident with Plane
          //  requires general intersection
          rbNeedsMoreIntersections = TRUE;
          return SM_SUCCESS;
        }

      // get revolution start/end angles - radians
      double dStartAngRad = GetStartAngleDeg()*SM_PI/180.0;
      double dEndAngRad   = GetEndAngleDeg()  *SM_PI/180.0;

      // get rotation angle to Positive Side of Plane
      double dAngleRad;
      // Get the vector in the plane that represents where we want to
      // rotate the gen curve to.  Call it the target vector.
      SmVector3d sTargetVec = sSurfOfRevolutionAxis * sPlaneNormal;

      // Our x axis is where the gen curve is: rotate from there,
      // about the SurfOfRev axis, to the target vector.
      SER(sSurfOfRevolutionAxis.CCWAngleBetween(m_vPosition.GetXAxisRef(),sTargetVec,dAngleRad));
      if (dAngleRad < 0.0) { dAngleRad += 2.0*SM_PI; }

      // when positive side of plane is in surfaceRotation limits
      if (   dAngleRad > dStartAngRad - dAngTolRad
          && dAngleRad < dEndAngRad   + dAngTolRad)
        {
          // copy and rotate the Generator Curve
          SmBSplineCurve *pCrv = new (crContext) SmBSplineCurve(*m_pGenCurve);
          SmObjDelete sCleanCrv(pCrv);
          double dRotAngRad = dAngleRad; // was dAngleRad-dStartAngRad [B143]
          SmAxis2Placement sTrans;
          sTrans.Translate(-m_vPosition.GetOriginRef());
          sTrans.RotateAboutAxis(dRotAngRad,m_vPosition.GetZAxis());
          sTrans.Translate(m_vPosition.GetOriginRef());
          SER(pCrv->Transform(sTrans));

#ifdef SM_DEBUG_CODE
          if ( bDebugMe ) {
              smgfx_SetLook( 1,2, 0,0,0 ); sTrans.Draw(); sm_GraphicsLoop();
              smgfx_SetLook( 3,5, 1,0,0 ); pCrv->Draw();  sm_GraphicsLoop();
              sm_GraphicsLoop();
          }
#endif
          // get rotated curve bounding box
          SmPseudoBox sCurvePBox ;
          pCrv->CalculateBoundingBox(pCrv->GetNaturalInterval(),
                                     NULL, &sCurvePBox, NULL) ;

          // only use curves intersecting plane domain
          if(!sPlanePBox.AreDisjoint(sCurvePBox))
            {
              // trim curves to plane domain
              SmTArray<SmCurve *> s3dCurves ;
              crPlane.TrimCurveToPlaneDomain(crOtherUVDomain, *pCrv, dTol, s3dCurves) ;
              SmObjsDelete<SmCurve *> sCleanArray ;
              if(pOpt3DCurves == NULL) { sCleanArray.SetArray(&s3dCurves) ; }

              // add trimmed curves to output
              for(jj=0;jj<s3dCurves.GetSize();jj++)
                {
                  if(pOpt3DCurves)   { pOpt3DCurves->Add(s3dCurves[jj]) ; }
                  if(pOptCurveTypes) { pOptCurveTypes->Add( SM_TC_CROSSING) ; }
                }

            }

        } // end positive side of plane is in SurfaceRotation check

      // get rotation angle to Negative side of plane
      sTargetVec *= -1.0;

      // Our x axis is where the gen curve is: rotate from there,
      // about the SurfOfRev axis, to the target vector.
      SER(sSurfOfRevolutionAxis.CCWAngleBetween(m_vPosition.GetXAxisRef(),sTargetVec,dAngleRad));
      if (dAngleRad < 0.0) { dAngleRad = 2.0*SM_PI + dAngleRad; }

      // when negative side of plane is in surfaceRotation limits
      if (   dAngleRad > dStartAngRad - dAngTolRad
          && dAngleRad < dEndAngRad   + dAngTolRad)
        {
          // copy and rotate the Generator Curve
          SmBSplineCurve *pCrv = new (crContext) SmBSplineCurve(*m_pGenCurve);
          SmObjDelete sCleanCrv(pCrv);
          double dRotAngRad = dAngleRad; // was dAngleRad-dStartAngRad [B143]
          SmAxis2Placement sTrans;
          sTrans.Translate(-m_vPosition.GetOriginRef());
          sTrans.RotateAboutAxis(dRotAngRad,m_vPosition.GetZAxis());
          sTrans.Translate(m_vPosition.GetOriginRef());
          SER(pCrv->Transform(sTrans));

#ifdef SM_DEBUG_CODE
          if ( bDebugMe ) {
              smgfx_SetLook( 1,2, 0,0,0 ); sTrans.Draw(); sm_GraphicsLoop();
              smgfx_SetLook( 3,5, 1,0,0 ); pCrv->Draw();  sm_GraphicsLoop();
              sm_GraphicsLoop();
          }
#endif
          // get rotated curve bounding box
          SmPseudoBox sCurvePBox ;
          pCrv->CalculateBoundingBox(pCrv->GetNaturalInterval(),
                                     NULL, &sCurvePBox, NULL) ;

          // only use curves intersecting plane domain
          sPlanePBox.ExpandAbsolute(dTol) ;
          if(!sPlanePBox.AreDisjoint(sCurvePBox))
            {
              // trim curves to plane domain
              SmTArray<SmCurve *> s3dCurves ;
              crPlane.TrimCurveToPlaneDomain(crOtherUVDomain, *pCrv, dTol, s3dCurves) ;
              SmObjsDelete<SmCurve *> sCleanArray ;
              if(pOpt3DCurves == NULL) { sCleanArray.SetArray(&s3dCurves) ; }

              // add trimmed curves to output
              for(jj=0;jj<s3dCurves.GetSize();jj++)
                {
                  if(pOpt3DCurves)   { pOpt3DCurves->Add(s3dCurves[jj]) ; }
                  if(pOptCurveTypes) { pOptCurveTypes->Add( SM_TC_CROSSING) ; }
                }

            }
        } // end negative side of plane is in SurfaceRotation check
    } // end RevolutionAxis/SurfaceNormal Perpendicular check

  // when RevolutionAxis/SurfaceNormal are parallel
  //  XSect = generator/Plane intersections revolved
  else if (sSurfOfRevolutionAxis.IsParallelTo(sPlaneNormal,SM_RAD2DEG(dAngTolRad)))
    {
      // Now plane is cutting through the SurfOfRevolution 'horizontally',
      SmSurfParamType eSurfParam  = (m_bSwapUV)
                                    ? SM_SP_U
                                    : SM_SP_V;

      // get Plane/GeneratorCurve intersections
      SmSolutionArray sSolutions;
      double          dD      = -rPlaneOrigin.Dot(sPlaneNormal);
      SmExtent1d      sGenIvl = m_pGenCurve->GetNaturalInterval();
      SER(m_pGenCurve->GlobalPropertyAnalysis(sGenIvl, SM_CP_PLANE_INTERSECTION,
                                              &dD, &sPlaneNormal, SM_ZONE_TOL_3D,
                                              sSolutions));

      // check for closed generatorCurves
      SmBoolean bIsClosed = m_pGenCurve->IsClosed(sGenIvl);

#ifdef SM_DEBUG_CODE
  if (bDebugMe || lCount == lDebugCount)
    {
      sSolutions.Dump() ;

      SmFace *pFace1 = (SmFace *)GetFace() ;
      SmFace *pFace2 = (SmFace *)crPlane.GetFace() ;
      SmBrep *pBrep1 = pFace1 ? pFace1->GetBrep() : NULL ;
      SmBrep *pBrep2 = pFace2 ? pFace2->GetBrep() : NULL ;

      smgfx_Erase();
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep1) { pBrep1->Draw(TRUE) ; } sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,0) ; if(pBrep2) { pBrep2->Draw(TRUE) ; } sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,1) ; DrawUV() ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 1,1,0) ; crPlane.DrawUV() ; sm_GraphicsLoop() ;
      smgfx_SetLook(3,4, 1,0,0) ; sSolutions.Draw() ; sm_GraphicsLoop() ;
      sm_GraphicsLoop();
    }
#endif
      // for every solution
      for(ii=0;ii<sSolutions.GetSize();ii++)
        {
          // see if range solutions show up - they do!
          //      // gwc note - this seems to be written for SingleValue Solutions
          //      if(sSolutions[ii].m_eSolutionType != SM_ST_SINGLE_VALUE)
          //        {
          //          SM_ASSERT(sSolutions[ii].m_eSolutionType == SM_ST_SINGLE_VALUE) ;
          //        }

          // get the
          SmVector3d       sPV[3];
          SmBSplineCurve * pCrv       = NULL;
          double           dIsoParam  = sSolutions[ii].m_vStart[0];
          SmBoolean        bIsTangent = FALSE ;
          SER(m_pGenCurve->Evaluate(dIsoParam,2,TRUE,sPV));

          // If the generator curve derivative is perpendicular to the plane
          // normal and the curvature of the curve is nearly zero
          // this is a tricky situation. Just go and use the standard
          // numerical surface intersector.
          //  gwc:note - this looks like the INTERVAL_SOLUTION case,
          //             so make the check for that explicit
          if (   sSolutions[ii].m_eSolutionType == SM_ST_RANGE_OF_VALUES
              || sPV[1].IsPerpendicularTo(sPlaneNormal,SM_RAD2DEG(dAngTolRad)))
            {
              bIsTangent = TRUE ;
              if (sPV[2].Length() < SM_EFF_ZERO_SQRT)
                {
                  for(jj=0;jj<pOpt3DCurves->GetSize();jj++)
                    {
                      SM_ASSERT((*pOpt3DCurves)[jj] != NULL) ; delete (*pOpt3DCurves)[jj] ; (*pOpt3DCurves)[jj] = NULL ;
                    }
                  pOpt3DCurves->ReSet();
                  rbNeedsMoreIntersections = TRUE;
                  return SM_SUCCESS;
                }
            } // end need for general surf/surf solver check

          // If we are closed don't generate a second curve for the end
          // of the curve.
          if (bIsClosed && SM_ARE_SAME(dIsoParam,sGenIvl.GetMax()))
            {
              continue;
            }

          // generate the swept curve
          SER(CreateIsoParametricCurve(crContext,
                                       eSurfParam,
                                       dIsoParam,
                                       0.0,
                                       pCrv));
          SmObjDelete sCleanCrv(pCrv);
          SM_ASSERT(pCrv != NULL) ;

          // get rotation curve bounding box
          SmPseudoBox sCurvePBox ;
          pCrv->CalculateBoundingBox(pCrv->GetNaturalInterval(),
                                       NULL, &sCurvePBox, NULL) ;

          // only use curves intersecting plane domain
          if(!sPlanePBox.AreDisjoint(sCurvePBox))
            {
              // trim curves to plane domain
              SmTArray<SmCurve *> s3dCurves ;
              crPlane.TrimCurveToPlaneDomain(crOtherUVDomain, *pCrv, dTol, s3dCurves) ;
              SmObjsDelete<SmCurve *> sCleanArray ;
              if(pOpt3DCurves == NULL) { sCleanArray.SetArray(&s3dCurves) ; }

              // add trimmed curves to output
              for(jj=0;jj<s3dCurves.GetSize();jj++)
                {
                  if(pOpt3DCurves)   { pOpt3DCurves->Add(s3dCurves[jj]); }
                  if(pOptCurveTypes) { pOptCurveTypes->Add( bIsTangent ? SM_TC_TANGENT
                                                                       : SM_TC_CROSSING) ;
                                     }
                }
            }
        } // end iter every solution
    } // end RevolutionAxis/SurfaceNormal are parallel branch

  else // Plane cuts RevolutionSurface at an angle
    {
      // use general surf/surf intersector
      rbNeedsMoreIntersections = TRUE;
      return SM_SUCCESS;
    }

  // set output
  SM_ASSERT(   pOpt3DCurves   == NULL
            || pOptCurveTypes == NULL
            || pOpt3DCurves->GetSize() == pOptCurveTypes->GetSize()) ;

  const ULONG numberCurves = (pOpt3DCurves == NULL) ? 0 : pOpt3DCurves->GetSize();
  for(ii=0; ii<numberCurves; ii++)
    {
      if (pOptSurface1UVCurves) { pOptSurface1UVCurves->Add(NULL); }
      if (pOptSurface2UVCurves) { pOptSurface2UVCurves->Add(NULL); }
      if (pOptCurveTypes)       {  if(pOpt3DCurves->GetAt(ii)->IsDegenerate())
                                     { pOptCurveTypes->SetAt(ii, SM_TC_TOUCHING); }
                                }
      if (pOptDeviations)       { pOptDeviations->Add(0.0); }
    }

  // all done
  return SM_SUCCESS;

} // end SmSurfOfRevolution::IntersectWithPlane

#if 0
/*******************************************************************//********
* This function is given a surf of revolution, and the angular domain of the
* other surface. NB the other surface can be SurfOfRevolution, Sphere, or
* Cylinder.
*
* The function converts the angular data into the ref frame of this,
* creates 2 periodic extents, and finds out if these intersect.
*
* In case of intersection, it returns the common angular domain (in the ref
* frame of 'this') for trimming.
*
* It also returns 2 angles, which can be used to create 2 coplanar generators
* (one on each surface).
*
*  This static is defined but not used
*****************************************************************************/
static SmStatus sm_AngularDomainsIntersect
  (const SmSurfOfRevolution* pMyselfArg,
   const SmVector3d & rOZAxisArg,
   const SmVector3d & rOXAxisArg,
   double dOStartAngleArg,
   double dOEndAngleArg,
   SmBoolean & bAngularDomainsIntersectArg,
   ULONG     & lXSectResultCnt,
   SmPeriodicExtentIntersectionType& reIntersectionTypeArg,
   SmPeriodicExtent1d& rCommonAngularExtentArg,
   SmPeriodicExtent1d& rCommonAngularExtentArg2,
   double dAngleInMyRegionArg[2],    // 1 slot for each of the max 2 results
   double dAngleInORegionArg[2])
{
    const SmAxis2Placement&  sMyPosition = pMyselfArg->GetPosition();
    const SmVector3d &rXAxis = sMyPosition.GetXAxisRef();
    SmVector3d        sZAxis = sMyPosition.GetZAxis();

    // Create the 2 periodic extents.
    SmPeriodicExtent1d sMyAngularExtent(pMyselfArg->GetStartAngleDeg(),
                                        pMyselfArg->GetEndAngleDeg(),
                                        360.0);
    SmPeriodicExtent1d sOAngularExtent(dOStartAngleArg, dOEndAngleArg, 360.0);


    // Conversion of the second into the ref frame of this begins.
    SmBoolean bAxesOpposite = (sZAxis.Dot(rOZAxisArg) < 0.0);
    if(bAxesOpposite) {
        sOAngularExtent.Invert();
    }

    double dAngleBetweenXAxesRad;
    SER(sZAxis.CCWAngleBetween(rXAxis, rOXAxisArg, dAngleBetweenXAxesRad));
    double dAngleBetweenXAxesDegree =
        180.0 * dAngleBetweenXAxesRad / SM_PI;

    sOAngularExtent.Translate(dAngleBetweenXAxesDegree);

    // Now the angular domain are comparable.

    SmBoolean bAngularDomainsDisjoint =
        sMyAngularExtent.AreDisjoint(sOAngularExtent);

    bAngularDomainsIntersectArg = bAngularDomainsDisjoint ? FALSE : TRUE;

    if(!bAngularDomainsIntersectArg)
        return SM_SUCCESS;

    // The arguments may be uninitialized
    rCommonAngularExtentArg  = SmPeriodicExtent1d(360.0);
    rCommonAngularExtentArg2 = SmPeriodicExtent1d(360.0);

    // The intersection proper
    SER(sMyAngularExtent.Intersect(sOAngularExtent,
                                   lXSectResultCnt,
                                   rCommonAngularExtentArg,
                                   rCommonAngularExtentArg2,
                                   reIntersectionTypeArg));

    // Pick a value from the common angular extent: the midpoint of the extent
    // is fine for both regions and single points .
    dAngleInMyRegionArg[0] = rCommonAngularExtentArg.Evaluate(0.5);

    // Find the angle on the other surface that corresponds to the selected
    // angle: it is the same as the selected, only transformed back.
    double dAngleInORegion = dAngleInMyRegionArg[0];

    // The transformation back:
    dAngleInORegion -= dAngleBetweenXAxesDegree;
    if(bAxesOpposite) {
        dAngleInORegion = -dAngleInORegion;
    }
    if(dAngleInORegion < 0.0)
        dAngleInORegion += 360.0;
    dAngleInORegionArg[0] = dAngleInORegion;

    // It they are 2 solutions, to the same for the other region/point
    // (fill the other slot of the output arguments):
    if( lXSectResultCnt == 2) {

        dAngleInMyRegionArg[1] = rCommonAngularExtentArg2.Evaluate(0.5);
        double dAngleInORegion = dAngleInMyRegionArg[1];

        // Now transform it back:
        dAngleInORegion -= dAngleBetweenXAxesDegree;
        if(bAxesOpposite) {
            dAngleInORegion = -dAngleInORegion;
        }
        if(dAngleInORegion < 0.0)
            dAngleInORegion += 360.0;

        // save the result:
        dAngleInORegionArg[1] = dAngleInORegion;
    }

    return SM_SUCCESS;

} // end sm_AngularDomainsIntersect
#endif

#if 0
/*******************************************************************//*********
* This is a convenience function, which helps avoid code repetition.
* It is invoked when 2 coplanar generators can be created for intersection.
* Their intersections shall be used to create the ssi curves.
*
* Rough outline of the method: for each point of intersection of the 2
* generators, create a directrix (a circle) on this surface, and trim it to
* the common angular domain.
*
*  This static is defined but not used
******************************************************************************/
static SmStatus sm_IntersectGenerators
  (const SmSurfOfRevolution       * pThis,                        // in :
   const SmSurfOfRevolution       * pOSurfOfRevolution,           // in :
   double                           dSSITolerance,                // in :
   ULONG                            lXSectResultCnt,              // in :
   SmPeriodicExtentIntersectionType eIntersectionType,            // in :
   SmPeriodicExtent1d             & sCommonAngularDomainOnThis,   // in :
   SmPeriodicExtent1d             & sCommonAngularDomainOnThis2,  // in :
   double                           dAngleInThisRegion[2],        // in :
   double                           dAngleInORegion[2],           // in :
   SmTArray<SmCurve*>             * pOpt3DCurves,                 // out:
   SmTArray<SmTsectCurveType>     * pOptCurveTypes)               // out:
{
  if (lXSectResultCnt == 0) { SER(SM_ERR); }

  // See how many regions we have (1 or 2):
  ULONG nRegions = 1;
  if(lXSectResultCnt == 2)
     { nRegions = 2; }

  SmSolution sSData[16];
  SmSolutionArray sSolutions(16,sSData);
  SmSolution sSData2[16];
  SmSolutionArray sSolutions2(16,sSData2);

  // for every region
  for(ULONG ii=0; ii<nRegions; ii++)
    {

      // Create a reference to the angular domain to be currently used:
      SmPeriodicExtent1d& rCurrentCommonAngularDomain = (ii==0) ? sCommonAngularDomainOnThis :
                                                                  sCommonAngularDomainOnThis2;

      // Create the 2 generators, 1 on each surface (these are coplanar)
      // NB the angle used is either the midpoint of a solution region or
      // the angle corresponding to a single point solution:
      SmBSplineCurve* pThisGeneratorBSC;
      SER(pThis->CreateGeneratorFromAngle(*pThis->GetContext(),
                                          dAngleInThisRegion[ii],// grad
                                          pThisGeneratorBSC));
      NER(pThisGeneratorBSC);
      SmObjDelete sCleanThisBSC(pThisGeneratorBSC);

      SmBSplineCurve* pOGeneratorBSC;
      SER(pOSurfOfRevolution->CreateGeneratorFromAngle(*pOSurfOfRevolution->GetContext(),
                                                       dAngleInORegion[ii],
                                                       pOGeneratorBSC));
      NER(pOGeneratorBSC);
      SmObjDelete sCleanOBSC(pOGeneratorBSC);

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  // draw input surfaces and generator curves
  if(bDebugMe)
    {
      SmFace *pFace1 = (SmFace *)pThis->GetFace() ;
      SmFace *pFace2 = (SmFace *)pOSurfOfRevolution->GetFace() ;
      SmBrep *pBrep1 = pFace1 ? pFace1->GetBrep() : NULL ;
      SmBrep *pBrep2 = pFace2 ? pFace2->GetBrep() : NULL ;

      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep1) pBrep1->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep2) pBrep2->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,1) ; if(pThis) pThis->DrawUV() ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 1,1,0) ; if(pThis) pOSurfOfRevolution->DrawUV() ; sm_GraphicsLoop() ;
      smgfx_SetLook(3,4, 1,0,0) ; if(pThisGeneratorBSC) pThisGeneratorBSC->Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook(3,4, 1,0,1) ; if(pOGeneratorBSC) pOGeneratorBSC->Draw() ; sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

      // Create 2 auxiliary vectors to be used in trimming the
      // directrices with
      SmAxis2Placement sRF(pThis->GetPosition());
      SmVector3d       sZAxis( sRF.GetZAxis() );

      double dRot1Rad = rCurrentCommonAngularDomain.GetMin() * SM_PI / 180.0;
      sRF.RotateAboutAxis(dRot1Rad,sZAxis);
      SmVector3d sTrimVector1(sRF.GetXAxisRef());

      double dRot2Rad = -1.0; // value which makes no sense
      SmVector3d sTrimVector2;

      // If we have a region at hand, then trim1 corresponds to the region's
      // start, so we need a trim vector for the end as well.
      // Somewhat many cases, but possible to cope:
      SmBoolean eThisIsRegion = FALSE; // set for single point
      if (ii==0 && eIntersectionType == SM_PI_REGION)
          eThisIsRegion = TRUE;
      if (eIntersectionType == SM_PI_2REGIONS)
          eThisIsRegion = TRUE;
      if (ii==0 && eIntersectionType == SM_PI_REGION_POINT)
          eThisIsRegion = TRUE;

      if (eThisIsRegion)
        {
          dRot2Rad = rCurrentCommonAngularDomain.GetLength() * SM_PI / 180.0;
          sRF.RotateAboutAxis(dRot2Rad,sZAxis);
          sTrimVector2 = sRF.GetXAxisRef();
        }

      // All preparations done, proceed with intersecting the generators:
      SER(pThisGeneratorBSC->GlobalCurveIntersect(
          pThisGeneratorBSC->GetNaturalInterval(),
          *pOGeneratorBSC,
          pOGeneratorBSC->GetNaturalInterval(),
          dSSITolerance, sSolutions));

      for (ULONG kk=0; kk<sSolutions.GetSize(); kk++)
        {

          if (sSolutions[kk].m_eSolutionType == SM_ST_SINGLE_VALUE)
            {

              // Solutions corresponding to single intersection points
              // are either circle segments,
              //         or single points


              // The following curve is the circular segment on the first
              // surface. To make it a solution, it must be trimmed to the
              // common angular domain.
              SmBSplineCurve* pDirectrix;
              SmPoint3d sAxisPoint; double dDirectrixRadius;
              pThis->CreateDirectrixAtGeneratorParam(*pThis->GetContext(),
                                              sSolutions[kk].m_vStart[0],
                                              pDirectrix,
                                              dDirectrixRadius, sAxisPoint);

              SmObjDelete sClean(pDirectrix); // it was used only to obtain
                                           // the axis point
              // The point to be used to obtain the trim location (by drop):
              SmPoint3d sTrimPoint1(
                  sAxisPoint + dDirectrixRadius * sTrimVector1);

              if (!eThisIsRegion)
                {

                  // The angular intersection is a single point.

                  SmBSplineCurve *pDegenerateCurve = NULL ;
                  SER(SmBSplineCurve::CreateDegenerateCurve(
                      *pThis->GetContext(),
                      3, // dim of result
                      sTrimPoint1,
                      pDegenerateCurve));

                  NER(pDegenerateCurve);

                  if(pOpt3DCurves)
                      pOpt3DCurves->Add(pDegenerateCurve);

                  if (pOptCurveTypes)
                          pOptCurveTypes->Add(SM_TC_CROSSING);

                }
              else
                {

                  // The angular intersection is now a region: the solution
                  // is a circle segment.

                  if (!rCurrentCommonAngularDomain.IsFullPeriod())
                    {
                      // Now we can compute the other trim point:
                      SmPoint3d sTrimPoint2(
                          sAxisPoint + dDirectrixRadius * sTrimVector2);

                      // The 2 inversions (drops) which give the trim locations:
                      SER(pDirectrix->GlobalPointSolve(
                          pDirectrix->GetNaturalInterval(),
                          SM_SO_MINIMIZE,sTrimPoint1,
                          SM_EFF_ZERO_SQRT,NULL,NULL,SM_SR_SINGLE,sSolutions2));
                      if(sSolutions2.GetSize() != 1)
                        { SER(SM_ERR);   }
                      double dTrimParameter1 = sSolutions2[0].m_vStart[0];

                      SER(pDirectrix->GlobalPointSolve(
                          pDirectrix->GetNaturalInterval(),
                          SM_SO_MINIMIZE,sTrimPoint2,
                          SM_EFF_ZERO_SQRT,NULL,NULL,SM_SR_SINGLE,sSolutions2));
                      if(sSolutions2.GetSize() != 1)
                        { SER(SM_ERR);   }
                      double dTrimParameter2 = sSolutions2[0].m_vStart[0];

                      // The trimming proper:
                      if (dTrimParameter1 < dTrimParameter2)
                        {
                          SmExtent1d dTrimExtent(dTrimParameter1, dTrimParameter2);
                          pDirectrix->Trim(dTrimExtent);  // may snap sIvl by tol to existing knots
                        }
                      else if (pDirectrix->IsPeriodic(pDirectrix->GetNaturalInterval()))
                        {
                          dTrimParameter2 = pDirectrix->GetNaturalInterval().GetMax();
                          SmExtent1d dTrimExtent(dTrimParameter1, dTrimParameter2);
                          pDirectrix->Trim(dTrimExtent);  // may snap sIvl by tol to existing knots
                        }

                    }

                  NER(pDirectrix);

                  if(pOpt3DCurves)
                    {
                      sClean.Clear();
                      pOpt3DCurves->Add(pDirectrix);
                    }

                  if (pOptCurveTypes)
                          pOptCurveTypes->Add(SM_TC_CROSSING);
                }
            }
          else
            {

              // Generators coincide along a segment of one of them.

              // A solution raised by a coincident generator segment
              // is either a surface region
              //        or a generator segment


              // Store the solution endpoints: these are parameter locations
              // on the generators.
              double dGeneratorStartParam = sSolutions[kk].m_vStart[0];
              double dGeneratorEndParam   = sSolutions[kk].m_vEnd[0];
              SmExtent1d dTrimExtent(dGeneratorStartParam,
                                     dGeneratorEndParam);

              // First, create the curve which is a subsegment of a generator
              // it is needed in all cases.

              SmBSplineCurve* pGeneratorBSC;
              SER(pThis->CreateGeneratorFromAngle(*pThis->GetContext(),
                  rCurrentCommonAngularDomain.GetMin(), // in grad
                  pGeneratorBSC));
              NER(pGeneratorBSC);

              pGeneratorBSC->Trim(dTrimExtent);  // may snap sIvl by tol to existing knots

              if(pOpt3DCurves)
                  pOpt3DCurves->Add(pGeneratorBSC);

              SmTsectCurveType eCType = SM_TC_REGION_BOUNDARY;
              if (eIntersectionType != SM_PI_REGION)
                      eCType = SM_TC_COINCIDENT;

              if (pOptCurveTypes)
                {
                  pOptCurveTypes->Add(eCType);
                }

              // Create a generator segment for the other end if region.
              if (eThisIsRegion)
                {

                  SER(pThis->CreateGeneratorFromAngle(*pThis->GetContext(),
                      rCurrentCommonAngularDomain.GetMax(), // in grad
                      pGeneratorBSC));
                  NER(pGeneratorBSC);

                  pGeneratorBSC->Trim(dTrimExtent);  // may snap sIvl by tol to existing knots

                  if(pOpt3DCurves)
                      pOpt3DCurves->Add(pGeneratorBSC);

                  if (pOptCurveTypes)
                    {
                      pOptCurveTypes->Add(eCType);
                    }
                }


              // The 1 or 2 sides done with, see if 2 directrix segments
              // (circle arcs) need to be created too:

              if (eThisIsRegion)
                {

                  SmBSplineCurve* pDirectrix1, *pDirectrix2;
                  SmPoint3d sAxisPoint1, sAxisPoint2;
                  double dDirectrix1Radius, dDirectrix2Radius;


                  // The first directrix (axis point and radius obtained too)
                  pThis->CreateDirectrixAtGeneratorParam(*pThis->GetContext(),
                          dGeneratorStartParam,
                          pDirectrix1, dDirectrix1Radius, sAxisPoint1);
                  NER(pDirectrix1);
                  SmObjDelete sCleanDx1(pDirectrix1);

                  if (!rCurrentCommonAngularDomain.IsFullPeriod())
                    {
                      // The 2 trim points to drop onto the curve
                      SmPoint3d sTrimPoint1(
                          sAxisPoint1 + dDirectrix1Radius * sTrimVector1);
                      SmPoint3d sTrimPoint2(
                          sAxisPoint1 + dDirectrix1Radius * sTrimVector2);


                      SmSolutionArray sSolutions2;
                      SER(pDirectrix1->GlobalPointSolve(
                          pDirectrix1->GetNaturalInterval(),
                          SM_SO_MINIMIZE,sTrimPoint1,
                          SM_EFF_ZERO_SQRT,NULL,NULL,SM_SR_SINGLE,sSolutions2));
                      double dTrimParameter1 = sSolutions2[0].m_vStart[0];

                      SER(pDirectrix1->GlobalPointSolve(
                          pDirectrix1->GetNaturalInterval(),
                          SM_SO_MINIMIZE,sTrimPoint2,
                          SM_EFF_ZERO_SQRT,NULL,NULL,SM_SR_SINGLE,sSolutions2));
                      double dTrimParameter2 = sSolutions2[0].m_vStart[0];

                      if (dTrimParameter1 < dTrimParameter2)
                        {
                          SmExtent1d dTrimExtent1(dTrimParameter1, dTrimParameter2);
                          pDirectrix1->Trim(dTrimExtent1);  // may snap sIvl by tol to existing knots
                        }
                      else if (pDirectrix1->IsPeriodic(pDirectrix1->GetNaturalInterval()))
                        {
                          dTrimParameter2 = pDirectrix1->GetNaturalInterval().GetMax();
                          SmExtent1d dTrimExtent(dTrimParameter1, dTrimParameter2);
                          pDirectrix1->Trim(dTrimExtent);  // may snap sIvl by tol to existing knots
                        }
                    }

                  if(pOpt3DCurves)
                    {
                      sCleanDx1.Clear();
                      pOpt3DCurves->Add(pDirectrix1);
                    }

                  if (pOptCurveTypes)
                      pOptCurveTypes->Add(SM_TC_REGION_BOUNDARY);

                  // Same for other directrix:
                  if (pThis->CreateDirectrixAtGeneratorParam(*pThis->GetContext(),
                          dGeneratorEndParam,
                          pDirectrix2, dDirectrix2Radius, sAxisPoint2) != SM_SUCCESS)
                            {
                      continue;
                    }
                  NER(pDirectrix2);
                  SmObjDelete sCleanDx2(pDirectrix2);

                  // The other directrix has another radius: recompute the
                  // trimpoints
                  if (!rCurrentCommonAngularDomain.IsFullPeriod())
                    {
                      SmPoint3d sTrimPoint1(sAxisPoint2 + dDirectrix2Radius*sTrimVector1);
                      SmPoint3d sTrimPoint2(sAxisPoint2 + dDirectrix2Radius*sTrimVector2);

                      SER(pDirectrix2->GlobalPointSolve(
                          pDirectrix2->GetNaturalInterval(),
                          SM_SO_MINIMIZE,sTrimPoint1,
                          SM_EFF_ZERO_SQRT,NULL,NULL,SM_SR_SINGLE,sSolutions2));
                      double dTrimParameter1 = sSolutions2[0].m_vStart[0];

                      SER(pDirectrix2->GlobalPointSolve(
                          pDirectrix2->GetNaturalInterval(),
                          SM_SO_MINIMIZE,sTrimPoint2,
                          SM_EFF_ZERO_SQRT,NULL,NULL,SM_SR_SINGLE,sSolutions2));
                      double dTrimParameter2 = sSolutions2[0].m_vStart[0];

                      if (dTrimParameter1 < dTrimParameter2)
                        {
                          SmExtent1d dTrimExtent2(dTrimParameter1, dTrimParameter2);
                          pDirectrix2->Trim(dTrimExtent2);  // may snap sIvl by tol to existing knots
                        }
                      else if (pDirectrix2->IsPeriodic(pDirectrix2->GetNaturalInterval()))
                        {
                          dTrimParameter2 = pDirectrix2->GetNaturalInterval().GetMax();
                          SmExtent1d dTrimExtent(dTrimParameter1, dTrimParameter2);
                          pDirectrix2->Trim(dTrimExtent);   // may snap sIvl by tol to existing knots
                        }
                    }

                  if(pOpt3DCurves)
                    {
                      sCleanDx2.Clear();
                      pOpt3DCurves->Add(pDirectrix2);
                    }

                  if (pOptCurveTypes)
                      pOptCurveTypes->Add(SM_TC_REGION_BOUNDARY);

                }
            }
        }
    } // end iter every region

  // all done
  return SM_SUCCESS;

} // end static SmStatus sm_IntersectGenerators
#endif

/*******************************************************************//**
PURPOSE: Find intersection curve(s) of this with other cone (when the axes
    coincide)

NOTES:
***********************************************************************/
SmStatus SmSurfOfRevolution::IntersectWithCone
  (const SmContext     & crContext,
   const SmExtent2d    & crUVDomain,
   const SmCone        & crCone,
   const SmExtent2d    & crConeUVDomain,
   const SmBoolean       bUseSurfaceEdges[2],
   const SmApproxTol3d * pdOptApproxTol3d,
   const double        * pdOptAngTolRad,
   SmBoolean           & rbNeedsMoreIntersections,
   SmTArray<SmCurve*>  * pOpt3DCurves,
   SmTArray<SmCurve*>  * pOptSurface1UVCurves,
   SmTArray<SmCurve*>  * pOptSurface2UVCurves,
   SmTArray<SmTsectCurveType> * pOptCurveTypes,
   SmTArray<double>    * pOptDeviations)
  const
{
    if (m_bPlanarGenerator == FALSE) {
        rbNeedsMoreIntersections = TRUE;
        return SM_SUCCESS;
    }

    SER(IntersectWithSurfOfRevolution(crContext,crUVDomain,crCone,
        crConeUVDomain,bUseSurfaceEdges,pdOptApproxTol3d,
        pdOptAngTolRad, rbNeedsMoreIntersections, pOpt3DCurves,
        pOptSurface1UVCurves, pOptSurface2UVCurves, pOptCurveTypes,
        pOptDeviations));

    return SM_SUCCESS;

} // end SmSurfOfRevolution::IntersectWithCone

/*******************************************************************//**
PURPOSE: Find intersection curve(s) of this with a sphere
    (when the axes coincide)

NOTES:
***********************************************************************/
SmStatus SmSurfOfRevolution::IntersectWithSphere
  (const SmContext     & crContext,
   const SmExtent2d    & crUVDomain,
   const SmSphere      & crSphere,
   const SmExtent2d    & crSphereUVDomain,
   const SmBoolean       bUseSurfaceEdges[2],
   const SmApproxTol3d * pdOptApproxTol3d,
   const double        * pdOptAngTolRad,
   SmBoolean           & rbNeedsMoreIntersections,
   SmTArray<SmCurve*>  * pOpt3DCurves,
   SmTArray<SmCurve*>  * pOptSurface1UVCurves,
   SmTArray<SmCurve*>  * pOptSurface2UVCurves,
   SmTArray<SmTsectCurveType> * pOptCurveTypes,
   SmTArray<double>    * pOptDeviations)
  const
{
    // Can not intersect things with non planar generator curves here
    if (m_bPlanarGenerator == FALSE) {
        rbNeedsMoreIntersections = TRUE;
        return SM_SUCCESS;
    }

    SER(IntersectWithSurfOfRevolution(crContext,crUVDomain,crSphere,
        crSphereUVDomain,bUseSurfaceEdges,pdOptApproxTol3d,
        pdOptAngTolRad, rbNeedsMoreIntersections, pOpt3DCurves,
        pOptSurface1UVCurves, pOptSurface2UVCurves, pOptCurveTypes,
        pOptDeviations));

    return SM_SUCCESS;

} // end SmSurfOfRevolution::IntersectWithSphere

/*******************************************************************//**
PURPOSE: Find intersection curve(s) of this with other surface of
    revolution (when the axes coincide)

NOTES:
***********************************************************************/
SmStatus SmSurfOfRevolution::IntersectWithSurfOfRevolution
  (
   const SmContext            & crContext,                // in : new object context
   const SmExtent2d           & crUVDomain,               // in : ThisSurface NurbDomain limit
   const SmSurfOfRevolution   & crSurfOfRevolution,       // in : OtherSurface to intersect
   const SmExtent2d           & crOtherUVDomain,          // in : OtherSurface Nurbdomain limit
   const SmBoolean              [2],                      // in :  bUseSurfaceEdges - not used
   const SmApproxTol3d        * pdOptApproxTol3d,         // in : max distance between 3d points
   const double               * pdOptAngTolRad,           // NotUsed: in : angle tolerance
   SmBoolean                  & rbNeedsMoreIntersections, // out: TRUE = call general intersector
   SmTArray<SmCurve*>         * pOpt3DCurves,             // out: new intersection curves
   SmTArray<SmCurve*>         * pOptSurface1UVCurves,     // out: associated ThisSurface  UVTrimCurves
   SmTArray<SmCurve*>         * pOptSurface2UVCurves,     // out: associated OtherSurface UVTrimCurves
   SmTArray<SmTsectCurveType> * pOptCurveTypes,           // out: associated IntersectionCurve classifications
   SmTArray<double>           * pOptDeviations)           // out: Max distance between surfaces near exact intersections
  const
{
  SM_REF1(pdOptAngTolRad) ;
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
static ULONG lCount       = 1 ; lCount++ ;
static ULONG lDebugCount  = 0 ;
  // draw input surfaces
  if(bDebugMe || lCount == lDebugCount)
    {
      SmFace *pFace1 = (SmFace *)GetFace() ;
      SmFace *pFace2 = (SmFace *)crSurfOfRevolution.GetFace() ;
      SmBrep *pBrep1 = pFace1 ? pFace1->GetBrep() : NULL ;
      SmBrep *pBrep2 = pFace2 ? pFace2->GetBrep() : NULL ;

      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep1) pBrep1->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep2) pBrep2->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,1) ; DrawUV() ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 1,1,0) ; crSurfOfRevolution.DrawUV() ; sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

  // init output
  rbNeedsMoreIntersections = FALSE;
  if (pOpt3DCurves)         pOpt3DCurves->ReSet();
  if (pOptSurface1UVCurves) pOptSurface1UVCurves->ReSet();
  if (pOptSurface2UVCurves) pOptSurface2UVCurves->ReSet();
  if (pOptCurveTypes)       pOptCurveTypes->ReSet();
  if (pOptDeviations)       pOptDeviations->ReSet();

  // no work - non planar generator curves
  if (   m_bPlanarGenerator == FALSE
      || crSurfOfRevolution.m_bPlanarGenerator == FALSE)
    {
      rbNeedsMoreIntersections = TRUE;
      return SM_SUCCESS;
    }

  // no work - Self Intersection is best done by NURBS to get UV curves.
  if (this == &crSurfOfRevolution)
    {
      rbNeedsMoreIntersections = TRUE;
      return SM_SUCCESS;
    }

  // locals
  double                   dAngularTolerance = SM_EFF_ZERO_DEG ;
  const SmAxis2Placement & sOtherPosition    = crSurfOfRevolution.GetPosition();
  const SmPoint3d        & rOtherOrigin      = sOtherPosition.GetOriginRef();
  const SmVector3d       & rOtherXAxis       = sOtherPosition.GetXAxisRef();
  SmVector3d               sOtherZAxis       = sOtherPosition.GetZAxis();
  const SmAxis2Placement & sMyPosition       = GetPosition();
  const SmVector3d       & rOrigin           = sMyPosition.GetOriginRef();
  const SmVector3d       & rXAxis            = sMyPosition.GetXAxisRef();
  const SmVector3d       & rYAxis            = sMyPosition.GetYAxisRef();
  SmVector3d               sZAxis            = sMyPosition.GetZAxis();

  // no work - coordinate systems are not parallel
  if(!sZAxis.IsParallelTo(sOtherZAxis, dAngularTolerance))
    {
      rbNeedsMoreIntersections = TRUE;
      return SM_SUCCESS;
    }

  // arrive here when both rotation axes are parallel to one another

  // select tolerance
  double dTol  = 0.0;
  double dSize = 3.0 ;
  if (!pdOptApproxTol3d)
    {
      SmExtent3d sBBox;
      SER(CalculateBoundingBox(crUVDomain,&sBBox));
      SmVector3d sSize = sBBox.GetSize();
      dSize     = sSize.Length() ;
      dTol = SM_EFF_ZERO_SQRT * (1.0 + dSize);
    }
  else
    {
      dTol = *pdOptApproxTol3d;
    }

  // no work = rotation axes are not coincident
  double dDistance;
  SER(smgu_LinePointDistance(rOrigin,sZAxis,rOtherOrigin,dDistance));
  if (dDistance > dTol/100.0)
    {
      rbNeedsMoreIntersections = TRUE;
      return SM_SUCCESS;
    }

  // map input NurbDomains to StepDomains
  SmExtent2d  sThisAnalDomain, sOtherAnalDomain ;
  SER( ConvertDomainFromNURBSToSTEP(crUVDomain, sThisAnalDomain));
  SER( crSurfOfRevolution.ConvertDomainFromNURBSToSTEP(crOtherUVDomain, sOtherAnalDomain));

  // get input sweep domains for both surfaces in thisCurves coordinate system
  SmExtent1d sThisSweepIvl  = sThisAnalDomain.GetUInterval() ;
  SmExtent1d sOtherSweepIvl = sOtherAnalDomain.GetUInterval() ;
  double dOtherXAngleRad ;
  sZAxis.CCWAngleBetween(rXAxis, rOtherXAxis, dOtherXAngleRad) ;
  double    dOtherXAngleDeg = dOtherXAngleRad * 180.0 / SM_PI ;
  SmBoolean bSame           = (sZAxis.Dot(sOtherZAxis) > 0.0) ; // should be +/-1.0
  double dOtherStartAngDeg, dOtherEndAngDeg ;
  if(bSame) { dOtherStartAngDeg = dOtherXAngleDeg + sOtherSweepIvl.GetMin() ;
              dOtherEndAngDeg   = dOtherXAngleDeg + sOtherSweepIvl.GetMax() ;
            }
  else      { dOtherStartAngDeg = dOtherXAngleDeg - sOtherSweepIvl.GetMax() ;
              dOtherEndAngDeg   = dOtherXAngleDeg - sOtherSweepIvl.GetMin() ;
              if(dOtherStartAngDeg > dOtherEndAngDeg)
                { dOtherStartAngDeg -= 360.0 ; }
            }
  SmExtent1d sOtherMappedIvl(dOtherStartAngDeg, dOtherEndAngDeg) ;

  // intersect the AnalDomains
  SmTArray<SmExtent1d> sIvls ;
  SmBoolean bDontCrossBoundaries = TRUE;  // True: Split result at sThisSweepIvl boundaries.
  sThisSweepIvl.IntersectPeriodic(sOtherMappedIvl, 360.0, sIvls, bDontCrossBoundaries );

  // no work - no sweep intersections
  if(sIvls.GetSize() == 0)
    {
      rbNeedsMoreIntersections = FALSE ;
      return SM_SUCCESS ;
    }

  // get temp genCurves in the same plane
  double dOtherParam = bSame ? -dOtherXAngleDeg
                             :  dOtherXAngleDeg ;

  SmBSplineCurve *pThisGenCurve = NULL, *pOtherGenCurve = NULL;
  SER(CreateGeneratorFromAngle(crContext, 0.0, pThisGenCurve));
  SER(crSurfOfRevolution.CreateGeneratorFromAngle(crContext,
                                                  dOtherParam,
                                                  pOtherGenCurve));
  NER(pThisGenCurve) ;
  NER(pOtherGenCurve) ;
  SmObjDelete sClean1(pThisGenCurve) ;
  SmObjDelete sClean2(pOtherGenCurve) ;

  // intersect the genCurves together
  SmSolutionArray sSolutions ;
  SER(pThisGenCurve->GlobalCurveIntersect(pThisGenCurve->GetNaturalInterval(),
                                          *pOtherGenCurve,
                                          pOtherGenCurve->GetNaturalInterval(),
                                          dTol,
                                          sSolutions)) ;
#ifdef SM_DEBUG_CODE
  if(bDebugMe)
    {
      sSolutions.Dump() ;

      SmFace *pFace1 = (SmFace *)GetFace() ;
      SmFace *pFace2 = (SmFace *)crSurfOfRevolution.GetFace() ;
      SmBrep *pBrep1 = pFace1 ? pFace1->GetBrep() : NULL ;
      SmBrep *pBrep2 = pFace2 ? pFace2->GetBrep() : NULL ;

      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep1) pBrep1->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep2) pBrep2->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,1) ; DrawUV() ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 1,1,0) ; crSurfOfRevolution.DrawUV() ; sm_GraphicsLoop() ;
      smgfx_SetLook(4,5, 1,0,0) ; sSolutions.Draw() ; sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;

    }
#endif // SM_DEBUG_CODE
  // for every genCurve/genCurve solution
  ULONG ii, jj ;
  SmBSplineCurve *pCrv ;
  SmTsectCurveType eCurveType, eGenCurveType ;
  for(ii=0;ii<sSolutions.GetSize();ii++)
    {
      SmSolution &rSolution = sSolutions[ii] ;

      // get Solution Start Points
      SmPoint3d sStartPoint[2], sOtherPoint[2] ;
      pThisGenCurve->Evaluate (rSolution.m_vStart[0], 1, TRUE, sStartPoint) ;
      pOtherGenCurve->Evaluate(rSolution.m_vStart[1], 1, TRUE, sOtherPoint) ;

      // get intersection circle CenterPoint and type
      SmPoint3d sCircleCenter = rOrigin + sZAxis.Dot(sStartPoint[0]-rOrigin) * sZAxis ;
      double    dCircleRadius = sCircleCenter.DistanceBetween(sStartPoint[0]) ;
      eGenCurveType           =   sStartPoint[1].IsParallelTo(sOtherPoint[1], 2.0)
                                ? SM_TC_TANGENT
                                : SM_TC_CROSSING ;
      double dDeviation = sStartPoint[0].DistanceBetween(sOtherPoint[0]) ;
      SM_ASSERT(dDeviation < dTol) ;

      // for every intersecting sweep interval
      for(jj=0;jj<sIvls.GetSize();jj++)
        {
          SmExtent1d &rIvl         = sIvls[jj] ;
          SmBoolean bDegenerateIvl = (rIvl.GetLength() / 180.0 * SM_PI * (smos_Min(dCircleRadius, 1.0)) < dTol) ;

          // build an intersection curve for every intersection point
          if(rSolution.m_eSolutionType == SM_ST_SINGLE_VALUE)
            {
              // the solution is either a degenerate point
              //     or a circular arc

              // for degenerate intervals
              if(bDegenerateIvl)
                {
                  // get point position
                  SmPoint3d sSweepPoint =   sCircleCenter
                                          + dCircleRadius * smos_Cosine(rIvl.GetMin() * SM_PI/180.0) * rXAxis
                                          + dCircleRadius * smos_Sine  (rIvl.GetMin() * SM_PI/180.0) * rYAxis ;

#ifdef SM_DEBUG_CODE
                  SmPoint3d sSweepEndPoint =   sCircleCenter
                                             + dCircleRadius * smos_Cosine(rIvl.GetMax() * SM_PI/180.0) * rXAxis
                                             + dCircleRadius * smos_Sine  (rIvl.GetMax() * SM_PI/180.0) * rYAxis ;

                  double dDistIvl = sSweepPoint.DistanceBetween(sSweepEndPoint) ;
                  SM_ASSERT(dDistIvl < dTol) ;
#endif
                  // build and degenerate curve and type
                  SmBSplineCurve::CreateDegenerateCurve(crContext, 3, sSweepPoint, pCrv) ;

                  // A Single value solution + degenerate interval means this srf-srf XSect is a point [B661]
                  eCurveType = SM_TC_TOUCHING;
                  //eCurveType =   (eCurveType == SM_TC_CROSSING) ? SM_TC_TOUCHING
                  //                                              : SM_TC_TANGENT ;
                } // end build degenerate curve branch
              else // build a sweep curve for the point
                {
                  // build circle curve and type
                  pCrv = new (crContext) SmCircle(sCircleCenter, rXAxis, rYAxis,
                                                  rIvl, dCircleRadius, 3) ;

                  // The SS XSect curve type matches the GenCurve XSect [B661]
                  eCurveType = eGenCurveType;
                } // end build sweep curve branch

              // add curve to output
              if(pOpt3DCurves)   { pOpt3DCurves->Add(pCrv) ; }
              if(pOptCurveTypes) { pOptCurveTypes->Add(eCurveType) ; }
              if(pOptDeviations) { pOptDeviations->Add(dDeviation); }

            } // end point solution branch
          else // solution was an interval
            {
              SM_ASSERT(rSolution.m_eSolutionType == SM_ST_RANGE_OF_VALUES) ;

              // A solution raised by a coincident generator segment
              // is either a surface   region  - add boundaries to output as type SM_TC_REGION_BOUNDARY
              //        or a generator segment - add 1 curve to output as type SM_TC_COINCIDENT

              // get GenCurve intersection interval
              SmExtent1d sGenCurveIvl(rSolution.m_vStart[0], rSolution.m_vEnd[0]) ;

              // Trim a genCurve at the interval start angle to the GenCurve/GenCurve intersection interval
              SmBSplineCurve *pMinAngleGenCurve = NULL;
              SER(CreateGeneratorFromAngle(crContext,
                                           rIvl.GetMin(),
                                           pMinAngleGenCurve));
              NER(pMinAngleGenCurve) ;
              pMinAngleGenCurve->Trim(sGenCurveIvl) ;  // may snap sIvl by tol to existing knots

              // for degenerate sweep interval with genCurve/genCurve interval intersections
              if(bDegenerateIvl)
                {
                  // add Trimmed GenCurve to output
                  if(pOpt3DCurves)   { pOpt3DCurves->Add(pMinAngleGenCurve) ; }
                  if(pOptCurveTypes) { pOptCurveTypes->Add(SM_TC_COINCIDENT) ; }
                  if(pOptDeviations) { pOptDeviations->Add(0.0); }

                } // end degenerate interval branch
              else // NonDegenerate sweep interval with genCurve/genCurve interval intersections
                {
                  SM_ASSERT(rSolution.m_eSolutionType == SM_ST_RANGE_OF_VALUES) ;

                  // add 4 boundary curves to output

                  // Trim a genCurve at the interval end angle to the GenCurve/GenCurve intersection interval
                  SmBSplineCurve *pMaxAngleGenCurve = NULL;
                  SER(CreateGeneratorFromAngle(crContext,
                                               rIvl.GetMax(),
                                               pMaxAngleGenCurve));
                  NER(pMaxAngleGenCurve) ;
                  pMaxAngleGenCurve->Trim(sGenCurveIvl) ;   // may snap sIvl by tol to existing knots

                  // bottom Circle and Radius are stored in sCircleCenter and dCircleRadius
                  SmBSplineCurve *pBotSweepArc = NULL ;
                  if(dCircleRadius > SM_EFF_ZERO)
                    {
                      pBotSweepArc = new (crContext) SmCircle(sCircleCenter,
                                                              rXAxis,
                                                              rYAxis,
                                                              rIvl,
                                                              dCircleRadius,
                                                                         3) ;
                    }
                  else // build a degenerate curve
                   {
                     SmStatus sRtn = SmBSplineCurve::CreateDegenerateCurve( crContext,
                                                                            3,
                                                                            sCircleCenter,
                                                                            pBotSweepArc) ;
                     SM_ASSERT(sRtn == SM_SUCCESS) ;
                   }

                  // get Top Circle Center and Radius from Solution EndPoint
                  SmBSplineCurve *pTopSweepArc = NULL ;
                  SmPoint3d sEndPoint ;
                  pThisGenCurve->EvaluatePoint(rSolution.m_vEnd[0], sEndPoint) ;
                  SmPoint3d sTopCircleCenter = rOrigin + sZAxis.Dot(sEndPoint-rOrigin) * sZAxis ;
                  double dTopCircleRadius    = sTopCircleCenter.DistanceBetween(sEndPoint) ;
                  if(dTopCircleRadius > SM_EFF_ZERO)
                    {
                      pTopSweepArc = new (crContext) SmCircle(sTopCircleCenter,
                                                              rXAxis,
                                                              rYAxis,
                                                              rIvl,
                                                              dTopCircleRadius,
                                                              3) ;
                    }
                  else // build a degenerate curve
                   {
                     SmStatus sRtn = SmBSplineCurve::CreateDegenerateCurve( crContext,
                                                                            3,
                                                                            sTopCircleCenter,
                                                                            pTopSweepArc) ;
                     SM_ASSERT(sRtn == SM_SUCCESS) ;
                   }

                  // add all 4 curves to the output
                  if(pOpt3DCurves)   { pOpt3DCurves->Add(pMinAngleGenCurve) ;
                                       pOpt3DCurves->Add(pMaxAngleGenCurve) ;
                                       pOpt3DCurves->Add(pBotSweepArc) ;
                                       pOpt3DCurves->Add(pTopSweepArc) ;
                                     }
                  if(pOptCurveTypes) { pOptCurveTypes->Add(SM_TC_REGION_BOUNDARY) ;
                                       pOptCurveTypes->Add(SM_TC_REGION_BOUNDARY) ;
                                       pOptCurveTypes->Add(SM_TC_REGION_BOUNDARY) ;
                                       pOptCurveTypes->Add(SM_TC_REGION_BOUNDARY) ;
                                     }
                  if(pOptDeviations) { pOptDeviations->Add(0.0);
                                       pOptDeviations->Add(0.0);
                                       pOptDeviations->Add(0.0);
                                       pOptDeviations->Add(0.0);
                                     }

                } // end sweep interval not degenerate branch
            } // end interval solution branch
        } // end iter every intersection sweep interval
    } // end iter every genCurve/genCurve intersection

#ifdef SM_DEBUG_CODE
  // draw input surfaces, intermediate intervals, and output intersection curves
  if(bDebugMe)
    {
      SmFace *pFace1 = (SmFace *)GetFace() ;
      SmFace *pFace2 = (SmFace *)crSurfOfRevolution.GetFace() ;
      SmBrep *pBrep1 = pFace1 ? pFace1->GetBrep() : NULL ;
      SmBrep *pBrep2 = pFace2 ? pFace2->GetBrep() : NULL ;

      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep1) pBrep1->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep2) pBrep2->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,1) ; DrawUV() ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 1,1,0) ; crSurfOfRevolution.DrawUV() ; sm_GraphicsLoop() ;
      smgfx_SetLook(3,4, 1,0,0) ; for(ii=0;ii<sIvls.GetSize();ii++)
                                    { SmExtent1d &rIvl = sIvls[ii] ;
                                      SmCircle sCircle(rOrigin, rXAxis, rYAxis,
                                                       rIvl, dSize, 3, &crContext) ;
                                      sCircle.Draw() ; sm_GraphicsLoop() ;
                                    }
      smgfx_SetLook(3,4, 1,0,0) ; if(pThisGenCurve)  pThisGenCurve->Draw() ;  sm_GraphicsLoop() ;
      smgfx_SetLook(3,4, 1,0,0) ; if(pOtherGenCurve) pOtherGenCurve->Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook(4,7, 1,0,0) ; for(ii=0;ii<pOpt3DCurves->GetSize();ii++)
                                    { SmCurve *pCurve = pOpt3DCurves->GetAt(ii) ;
                                      pCurve->Draw() ; sm_GraphicsLoop() ;
                                    }
      smgfx_SetLook(5,6, 0,1,0) ; sSolutions.Draw() ; sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

  for (ULONG i=0; i<pOpt3DCurves->GetSize(); i++)
    {
      if (pOptSurface1UVCurves) pOptSurface1UVCurves->Add(NULL);
      if (pOptSurface2UVCurves) pOptSurface2UVCurves->Add(NULL);
    }

  // all done
  return(SM_SUCCESS) ;

  // old way with bugs and uninitialized values
  //
  //       // intersect the genCurves
  //
  //      // Checking if the angular domains intersect helps avoid superfluous
  //      // curve/curve intersection:
  //      SmBoolean bAngularDomainsIntersect;
  //      double dAngleInThisRegion[2], dAngleInORegion[2];
  //      SmPeriodicExtentIntersectionType eIntersectionType;
  //      SmPeriodicExtent1d sCommonAngularDomainOnThis(0.0);
  //      SmPeriodicExtent1d sCommonAngularDomainOnThis2(0.0);
  //      SER(sm_AngularDomainsIntersect(this,
  //                                       sOtherZAxis,
  //                                       sOtherPosition.GetXAxisRef(),
  //                                       crSurfOfRevolution.GetStartAngleDeg(),
  //                                       crSurfOfRevolution.GetEndAngleDeg(),
  //                                       bAngularDomainsIntersect,
  //                                       lXSectResultCnt,
  //                                       eIntersectionType,
  //                                       sCommonAngularDomainOnThis,
  //                                       sCommonAngularDomainOnThis2,
  //                                       dAngleInThisRegion, dAngleInORegion));
  //
  //
  //      if (!bAngularDomainsIntersect)
  //        {
  //          rbNeedsMoreIntersections = FALSE;
  //          return SM_SUCCESS;
  //        }
  //
  //      SER(sm_IntersectGenerators(this,
  //                                  &crSurfOfRevolution,
  //                                  dTol,
  //                                  lXSectResultCnt,
  //                                  eIntersectionType,
  //                                  sCommonAngularDomainOnThis,
  //                                  sCommonAngularDomainOnThis2,
  //                                  dAngleInThisRegion,
  //                                  dAngleInORegion,
  //                                  pOpt3DCurves,
  //                                  pOptCurveTypes));
  //
  //
  // in : for (ULONG i=0; i<pOpt3DCurves->GetSize(); i++)
  // in :   {
  // in :     if (pOptSurface1UVCurves) pOptSurface1UVCurves->Add(NULL);
  // in :     if (pOptSurface2UVCurves) pOptSurface2UVCurves->Add(NULL);
  // in :     if (pOptDeviations) pOptDeviations->Add(0.0);
  // in :   }
  // in :
  // in : return SM_SUCCESS;

} // end SmSurfOfRevolution::IntersectWithSurfOfRevolution

/*******************************************************************//**
PURPOSE: Determine if the SurfaceOfRevolution is bounded.

NOTES:
***********************************************************************/
SmBoolean SmSurfOfRevolution::IsBounded
  ()
 const
{
  return (m_pGenCurve->IsBounded());

} // end SmSurfOfRevolution::IsBounded

/*******************************************************************//**
PURPOSE: Static function to determine if a given nurbs is a SurfOfRevolution.
    If it is a SurfOfRevolution then one is created.

METHOD ---
 - Must be rational biquadratic;
 - Isoparametric curves must be circles in (at least) one direction;
 - Iso-circles must be in parallel planes;
 - Iso-circle centers must be on a common axis;
 - The start and end points of iso-circles must be coplanar
   (to create a planar generating curve).

NOTES:
***********************************************************************/
SmBoolean SmSurfOfRevolution::IsNurbSurfaceSurfOfRevolution
 (const SmContext        & crContext,           // in : new object construction
  const SmBSplineSurface * pTestSurface,        // in : surface to examine
  SmSurfOfRevolution    *& rpSurfOfRevolution,  // out: new SurfOfRev when TestSurface can be one
  double                   dToleranceScale)     // in : max allowed variation from circular sweeps
                                                //      = SM_EFF_ZERO * dToleranceScale * ANALYTIC_TOL_SCALE
{
  // init output
  rpSurfOfRevolution = NULL;
  if (pTestSurface == NULL)
    {
      SM_ASSERT(pTestSurface != NULL);
      return FALSE;
    }

  // low work - already a SmSurfOfRevolution
  if(pTestSurface->IsKindOf(SmSurfOfRevolution_TYPE))
    {
      SmSurface * pCopySurface = NULL ;
      pTestSurface->Copy(crContext, pCopySurface) ; 
      rpSurfOfRevolution = (SmSurfOfRevolution *)pCopySurface ;
      return TRUE ;
    }

  // no work - not a BSplineSurface
  if(!pTestSurface->IsKindOf(SmBSplineSurface_TYPE))
    { return FALSE ; }

  SmBSplineSurface * pTgtSurface = (SmBSplineSurface *)pTestSurface ;

  // no work - not a degree 2 rational surface in at least 1 direction
  ULONG lDegU = pTgtSurface->GetDegree(SM_SP_U);
  ULONG lDegV = pTgtSurface->GetDegree(SM_SP_V);
  if (   !pTgtSurface->IsRational()
      || (   lDegU != 2
          && lDegV != 2))
      return FALSE;

  // evaluate min/max domain points
  SmExtent2d sUVDomain = pTgtSurface->GetNaturalUVDomain();
  SmPoint3d  sMinPnt, sMaxPnt;
  pTgtSurface->EvaluatePoint(sUVDomain.GetMin(),sMinPnt);
  pTgtSurface->EvaluatePoint(sUVDomain.GetMax(),sMaxPnt);

  // scaled zero value
  double dScaledZero =   dToleranceScale
                       * ANALYTIC_TOL_SCALE
                       * SM_EFF_ZERO * (1.0 + sMinPnt.GetMaxDimension() + sMaxPnt.GetMaxDimension());

  // made up AngTolRad value
  double dAngTolRad  = SM_EFF_ZERO_SQRT * dToleranceScale;

  // locals
  SmBoolean bFoundCirclesU, bParallelPlanesU, bSamePlaneU, bCoaxialCirclesU, bCommonAxisU, bPlanarGenCurveU;
  SmBoolean bFoundCirclesV, bParallelPlanesV, bSamePlaneV, bCoaxialCirclesV, bCommonAxisV, bPlanarGenCurveV;
  SmExtent1d sAnglesU, sAnglesV;
  double dRadiusU, dRadiusV;
  SmAxis2Placement sRefFrameU, sRefFrameV;
  SmBoolean bIsRevolutionNoSwapUV   = FALSE ;   // lDegU == 2
  SmBoolean bIsRevolutionWithSwapUV = FALSE ;   // lDegV == 2
  SmPoint3d sCommonPoint, sCommonVec ;

  // test sequence of U direction (V Constant) isoCurves for circularity and parallelism
  if(lDegU == 2)
    {
      SER(sm_TestForIsoCircles(pTgtSurface, SM_SP_V, dScaledZero, dAngTolRad,
                                  bFoundCirclesV, bParallelPlanesV, bSamePlaneV,
                                  bCoaxialCirclesV, bCommonAxisV, bPlanarGenCurveV,
                                  dRadiusV, sAnglesV, sRefFrameV,
                                  sCommonPoint, sCommonVec));

      // see if we Swap_UV = FALSE SurfOfRevolution
      bIsRevolutionNoSwapUV = (   bFoundCirclesV
                               && bParallelPlanesV
                               && bCoaxialCirclesV
                               && bPlanarGenCurveV) ;
    }

  // test sequence of V direction (U Constant) isoCurves for circularity and parallelism
  if(   !bIsRevolutionNoSwapUV
     && lDegV == 2)
    {
      SER(sm_TestForIsoCircles(pTgtSurface, SM_SP_U, dScaledZero, dAngTolRad,
                                  bFoundCirclesU, bParallelPlanesU, bSamePlaneU,
                                  bCoaxialCirclesU, bCommonAxisU, bPlanarGenCurveU,
                                  dRadiusU, sAnglesU, sRefFrameU,
                                  sCommonPoint, sCommonVec));

      // see if we Swap_UV = TRUE SurfOfRevolution
      bIsRevolutionWithSwapUV = (   bFoundCirclesU
                                 && bParallelPlanesU
                                 && bCoaxialCirclesU
                                 && bPlanarGenCurveU) ;
    }

  // no work - no sweep direction
  if(!bIsRevolutionWithSwapUV && !bIsRevolutionNoSwapUV)
    { return FALSE; }

  // Now that the surface is a revolution of some generating curve
  // Let GenCurve = iso-curve from the 'other' parameter
  SmBSplineCurve   * pIsoCrv = NULL ;
  SmAxis2Placement * pRefFrame = NULL ;
  SmSurfParamType    eGenDir;
  //SmSurfParamType    eSweepDir ;
  SmBoolean          bSwapUV ;
  double             dParam, dMinAngDeg, dMaxAngDeg ;
  //double             dSweepParam;
  if(bIsRevolutionNoSwapUV) { //eSweepDir   = SM_SP_V ;  // constant V isoParamCurves are circles
                              eGenDir     = SM_SP_U ;
                              bSwapUV     = FALSE ;
                              dParam      = sUVDomain.GetMin().x ;
                              //dSweepParam = sUVDomain.GetVInterval().Evaluate(.45678) ;
                              pRefFrame   = &sRefFrameV ;
                              dMinAngDeg  = sAnglesV.GetMin() ;
                              dMaxAngDeg  = sAnglesV.GetMax() ;
                            }
  else                      { SM_ASSERT(bIsRevolutionWithSwapUV) ;
                              //eSweepDir   = SM_SP_U ; // constant U isoParamCurves are circles
                              eGenDir     = SM_SP_V ;
                              bSwapUV     = TRUE ;
                              dParam      = sUVDomain.GetMin().y ;
                              //dSweepParam = sUVDomain.GetUInterval().Evaluate(.45678) ;
                              pRefFrame   = &sRefFrameU ;
                              dMinAngDeg  = sAnglesU.GetMin() ;
                              dMaxAngDeg  = sAnglesU.GetMax() ;
                            }
  SER(pTgtSurface->CreateIsoParametricCurve(crContext,
                                             eGenDir,
                                             dParam,
                                             0.0,
                                             pIsoCrv));
  SmObjDelete sClean(pIsoCrv);

  // no work - GenCurve is degenerate
  if (pIsoCrv->IsDegenerate(dScaledZero))
    { return FALSE ; }


  // The iscurves of the surface have already been tested in sm_TestForIsoCircles
  // and the following two tests are not exactly equivalent.  Ordinarily not a
  // propbelm but because the checks have different geometry they also have
  // different tolerances.

  // Test 1: the plane of non-linear curves should contain the origin and
  //         the rotation axis.  The surface has been tested that the start point
  //         of every tested iso-curve is on the same plane to within tolerance
  //         The plane of the curve, might vary by tolerance allowing it to rotate
  //         some from the start plane of the sweep changing the tolerance between
  //         the origin and the curve plane.  Choices: treat curves as surfaces
  //         of revolution that pass both tests or pass just the one test built into
  //         sm_TestForIsoCircles.  I choose the latter.
  //         amounts and at a distance the origin point might be a different
  // Test 2:

  // Verify GenCurve is planar and coplanar with refFrame ZAxis.
  // This can be violated if the input is noisy,
  // and not caught in the previous tests.

  // verify GenCurve planarity
  SmPoint3d sPlanePoint, sPlaneNormal, sLinePoint, sLineTangent;
  double dLinearTol = SM_EFF_ZERO * (1.0 + smos_Max( sMinPnt.GetMaxDimension(),
                                           sMaxPnt.GetMaxDimension()));
  SmBoolean bPlanar = pIsoCrv->IsPlanar(dScaledZero, &sPlanePoint, &sPlaneNormal) ;
  SmBoolean bLinear = pIsoCrv->IsLinear(dLinearTol,  &sLinePoint,  &sLineTangent) ;

  // verify that plane containing genCurve also contains refFrame Z axis
  //  for nonLinear GenCurves: 1. RefFrame Origin is on genCurvePlane
  //                           2. RefFrame ZAxis is perp to GenCurvePlane Normal
  //  for    Linear GenCurves: 1. Line, RefFrame Origin, RefFrame Z axis all lie in same plane
  double dOriginPointDist   = smos_Fabs( (pRefFrame->GetOriginRef()-sPlanePoint).Dot(sPlaneNormal) ) ;
  double dFramePlaneOrtho   = smos_Fabs(sPlaneNormal.Dot(pRefFrame->GetZAxis())) ;

  double dFrameLineCoPlanar = bLinear ? smos_Fabs((pRefFrame->GetOriginRef()-sPlanePoint).Dot(pRefFrame->GetZAxis()*sLineTangent)) : 1.0 ;
  SM_ASSERT(bPlanar) ;
//    SM_ASSERT(     bLinear || dOriginPointDist   < dScaledZero) ;
//    SM_ASSERT(   (!bLinear && dFramePlaneOrtho   < dScaledZero)
//              || ( bLinear && dFrameLineCoPlanar < dScaledZero)) ;

  if ( !bLinear && dOriginPointDist >= dScaledZero ) {
      SM_ASSERT_ERR ;
      return FALSE;
  }

  if (    (  bLinear || dFramePlaneOrtho   >= dScaledZero )
       && ( !bLinear || dFrameLineCoPlanar >= dScaledZero )) {
      SM_ASSERT_ERR ;
      return FALSE;
  }

#ifdef SM_DEBUG_CODE
  {
SmBoolean bDebugMe = FALSE ;
    if(bDebugMe)
      {
        SmFace *pFace = (SmFace *)pTgtSurface->GetFace() ;
        SmBrep *pBrep = pFace ? pFace->GetBrep() : NULL ;

        smgfx_Erase() ;
        smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
        smgfx_SetLook(1,2, 0,1,1) ; pTgtSurface->DrawUV() ; sm_GraphicsLoop() ;
        smgfx_SetLook(2,2, 0,0,0) ; if(pFace) pFace->Draw(SM_DM_CROSSHATCH) ; sm_GraphicsLoop() ;
        smgfx_SetLook(3,4, 1,0,0) ; if(pIsoCrv) pIsoCrv->Draw() ; sm_GraphicsLoop() ;
        smgfx_SetLook(5,6, 1,0,0) ; sPlanePoint.Draw() ; sPlaneNormal.Draw(&sPlanePoint) ; sm_GraphicsLoop() ;
        smgfx_SetLook(3,4, 0,1,0) ; pRefFrame->Draw() ; sm_GraphicsLoop() ;
        smgfx_SetLook(7,8, 0,0,1) ; sLinePoint.Draw() ; sLineTangent.Draw(&sLinePoint) ; sm_GraphicsLoop() ;
        sm_GraphicsLoop() ;
      }
  }
#endif // SM_DEBUG_CODE

//            // we will create surf-of-revo only when the generating curve
//            // is co-planar with the center axis
//            SmVector3d sYAxis = sZAxis * sXAxis;
//            SmExtent1d sExt = pIsoCrv->GetNaturalInterval();
//            // for now, sample 5 points on the generating curve and find if
//            // they are all co-planar with the center Axis
//            SmTArray<SmPoint3d> sCtrlPts(256);
//            SER(pIsoCrv->GetPolygon(pIsoCrv->GetNaturalInterval(),sCtrlPts));
//            for (ULONG k=0; k<sCtrlPts.GetSize(); k++)
//              {
//                SmPoint3d sCrvPt = sCtrlPts[k];
//                if (sYAxis.Dot(sCrvPt-rOrigin) > dScaledZero)
//                    continue;;
//              }

  // make SurfOfRevolution
  SmExtent1d sInterval = pIsoCrv->GetNaturalInterval() ;
  SmExtent1d sAnalIvl  = pIsoCrv->GetSTEPInterval() ;
  SmExtent2d sAnalUVDomain(dMinAngDeg, sAnalIvl.GetMin(),
                           dMaxAngDeg, sAnalIvl.GetMax());
  SmPoint2d  sMidUV  = sAnalUVDomain.Evaluate(.54321,.54321) ;
  rpSurfOfRevolution = new (crContext) SmSurfOfRevolution(pIsoCrv,
                                                          pRefFrame->GetOriginRef(),
                                                          pRefFrame->GetXAxisRef(),
                                                          pRefFrame->GetYAxisRef(),
                                                          sAnalUVDomain,
                                                          bSwapUV);
  NER(rpSurfOfRevolution);
  sClean.Clear();
  if (rpSurfOfRevolution->m_pGenCurve == NULL) { delete rpSurfOfRevolution; rpSurfOfRevolution = NULL;
                                                 return FALSE;
                                               }

  // New SurfOfRevolution has been constructed. Need to
  //  1. set NewSurf->m_pNurb = inputSurface->m_pNurb
  //  2. Copy InputSurface->Attributes onto NewSurf.
  //  3. make a PolarConverter for rotation angles

  // reparameterize the NURB so that AnalyticSurface->NurbDomain == InputSurface->NurbDomain
  rpSurfOfRevolution->Reparameterize(pTgtSurface->GetNaturalUVDomain()) ;

  // obsolete - it's a mistake to introduce variations between Nurb and Anal reps by copying the m_pNurb object
  // Copy pTgtSurface->m_pNurb into newSurfOfRevolution
  //      SM_ASSERT(rpSurfOfRevolution->m_pNurb == NULL) ;
  //      rpSurfOfRevolution->m_pNurb = sm_AllocateAndCopyNurbSurface(((SmBSplineSurface*)pTgtSurface)->GetGwNurbPointer()) ;
  //
  //      // Build newSurfOfRevolution PolarConverter from Nurb midPoint rotation isoParamCurve
  //      // pRotationIsoCrv will be an SmCircle because isoParam is coming from SmSurfOfRevolution
  //      rpSurfOfRevolution->RebuildPolarConverter() ;
  // end obsolete

  // Copy pTgtSurface attributes onto newSurfOfRevolution
  ((SmBSplineSurface*)pTgtSurface)->Notify(SM_NO_COPY, rpSurfOfRevolution, SM_NO_GET_OWNER(rpSurfOfRevolution), SM_NO_GET_OWNER(pTgtSurface));

  //      SmBSplineCurve *pRotationIsoCrv = NULL ;
  //      rpSurfOfRevolution->m_vPolarConverter.SetPolarConversionPossible(FALSE) ;
  //      SER(rpSurfOfRevolution->CreateIsoParametricCurve(crContext,
  //                                                       eSweepDir,
  //                                                       dSweepParam, // eSweepDir == SM_SP_U ? sMidUV.x : sMidUV.y,
  //                                                       0.0, pRotationIsoCrv));
  //      SmObjDelete sCleanIsoRot(pRotationIsoCrv);
  //
  //      // Build the Surfs PolarConverter for rotation angle based on the pRotationIsoCrv
  //      if (!rpSurfOfRevolution->m_vPolarConverter.IsPolarConversionPossible())
  //        {
  //          sCleanIsoRot.Clear() ;
  //          SER(rpSurfOfRevolution->m_vPolarConverter.SetUpPolarConversion
  //                                           (pRotationIsoCrv,
  //                                            FALSE,
  //                                            rpSurfOfRevolution->m_vAnalUVDomain.GetUInterval(),
  //                                            rpSurfOfRevolution->m_vPosition));
  //        }

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
  if (bDebugMe)
    {
      SmBSplineCurve *pIso = NULL ;
      if (rpSurfOfRevolution->m_bSwapUV) { SER(rpSurfOfRevolution->CreateIsoParametricCurve(crContext,SM_SP_U,0.0,SM_EFF_ZERO,pIso));
                                         }
      else                               { SER(rpSurfOfRevolution->CreateIsoParametricCurve(crContext,SM_SP_V,0.0,SM_EFF_ZERO,pIso));
                                         }
      SmObjDelete sCleanIso(pIso);

      // dump input surface, output surface, sweep isoCurve, genCurve
      pTgtSurface->Dump();
      rpSurfOfRevolution->Dump();
      rpSurfOfRevolution->m_pGenCurve->Dump();
      pIso->Dump();

      smgfx_Erase();
      smgfx_SetLook(1,2, 0,0,1); rpSurfOfRevolution->DrawUV(); sm_GraphicsLoop();
      smgfx_SetLook(3,4, 1,0,0); rpSurfOfRevolution->m_pGenCurve->Draw(); sm_GraphicsLoop();
      smgfx_SetLook(6,7, 1,0,1); pIso->Draw(); sm_GraphicsLoop();
      smgfx_SetLook(3,4, 0,1,0); pTgtSurface->DrawUV(4,4); sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
#endif

  // all done
  SM_DUMP_AND_ASSERT2_VALID(rpSurfOfRevolution) ;
  return TRUE;

} // end SmSurfOfRevolution::IsNurbSurfaceSurfOfRevolution

/*******************************************************************//**
PURPOSE: Return TRUE when both NURB and STEP representations are
            equivalent, else return FALSE.

NOTES:
***********************************************************************/
SmBoolean SmSurfOfRevolution::AreSTEPAndNURBCurrent() const
{
  // init return
  SmBoolean bRtn = TRUE ;

  // check nested SmPolarConverter
  bRtn &= m_vPolarConverter.IsCurrent() ;

  // when given an m_pNurb and m_pGenCurve
  if(   m_pNurb
     && m_pGenCurve)
    {
      // GenCurve Curve must have same parameterization as Surface in non SweepCurve direction
      // GenCurve AnalInterval   must equal Surface->AnalUVDomain->GetVInterval,
      // GenCurve NurbInterval   must equal m_bSwapUV ? Surface->NURBUInterval : Surface->NURBVInterval
      // GenCurve NURBKnotVector must equal m_bSwapUV ? Surface->NURBKnotU : Surface->NURBKnotV
      bRtn &= HasSameParameterization(SM_SP_V,        // in : Analytic Domain Direction
                                        m_bSwapUV     // in : Nurb     Domain Direction
                                      ? SM_SP_U
                                      : SM_SP_V,
                                      m_pGenCurve) ;  // in : The GenCurve

      // compare a GenCurve near midPoint evaluation
      if(m_vPolarConverter.IsPolarConversionPossible())
        {
          SmPoint2d sNurbUV, sStepUV = m_vAnalUVDomain.Evaluate(0.0, .4567) ;
          ConvertUVFromSTEPToNURBS(sStepUV, sNurbUV) ;
          SmPoint3d sStepPoint, sNurbPoint ;
          EvaluateSTEPPoint(sStepUV, sStepPoint) ;
          EvaluatePoint(sNurbUV, sNurbPoint) ;
          double dScaledZero = SM_EFF_ZERO * (1.0 + sStepPoint.GetMaxDimension()) ;
          bRtn &= sStepPoint.CloserThan(dScaledZero, sNurbPoint) ;
       }
    }

  // PolarConverter Curve must have same parameterization as Surface in SweepCurve direction
  const SmBSplineCurve *pPolarCurve = m_vPolarConverter.GetCurve() ;

  // when given an m_pNurb and a PolarCurve
  if(m_pNurb && pPolarCurve)
    {
      // PolarConverter Curve must have same parameterization as Surface in SweepCurve direction
      // pPolarCurve AnalInterval   must equal Surface->AnalUVDomain->GetUInterval,
      // pPolarCurve NURBInterval   must equal m_bSwapUV ? Surface->NURBVInterval : Surface->NURBUInterval
      // pPolarCurve NURBKnotVector must equal m_bSwapUV ? Surface->NURBKnotV : Surface->NURBKnotU
      bRtn &= HasSameParameterization(SM_SP_U,        // in : Analytic Domain Direction
                                        m_bSwapUV     // in : Nurb     Domain Direction
                                      ? SM_SP_V
                                      : SM_SP_U,
                                      pPolarCurve) ;  // in : The GenCurve

      // compare a Sweep near midPoint evaluation
      if(m_vPolarConverter.IsPolarConversionPossible())
        {
          SmPoint2d sNurbUV, sStepUV = m_vAnalUVDomain.Evaluate(.4567, .7654) ;
          ConvertUVFromSTEPToNURBS(sStepUV, sNurbUV) ;
          SmPoint3d sStepPoint, sNurbPoint ;
          EvaluateSTEPPoint(sStepUV, sStepPoint) ;
          EvaluatePoint(sNurbUV, sNurbPoint) ;
          double dScaledZero = SM_EFF_ZERO * (1.0 + sStepPoint.GetMaxDimension()) ;
          bRtn &= sStepPoint.CloserThan(dScaledZero, sNurbPoint) ;
       }

    } // end m_pNurb and m_pGencurve existence check

  // all done
  return(bRtn) ;

} // end SmSurfOfRevolution::AreSTEPAndNURBCurrent

/*******************************************************************//**
PURPOSE: Rebuild all analytic, GenCurve, and PolarCurve cached data
  from the m_pNurb data.

NOTES: Can be used after modifying the m_pNurb data to update
  the analytic data.

RETURNS: SM_ERR if the NURB no longer represents this Analytic surface
***********************************************************************/
SmStatus SmSurfOfRevolution::RebuildSTEPFromNURBParameters()
{
  // check m_pNurb - a degree 2 rational surface in at least 1 direction
  ULONG lDegU = GetDegree(SM_SP_U);
  ULONG lDegV = GetDegree(SM_SP_V);
  if (   (lDegU != 2 && lDegV != 2)
      || !IsRational())
    { SER_MSG(SM_ERR,  _T("Modified Nurb SmSurfOfRevolution: no longer has a degree 2 rotation")) ; }

  // method
  //  Check for iso circles

  //  Set m_pGenCurve
  //  Set m_bPlanarGenerator
  //  Set m_vPosition
  //  Set m_bSwapUV
  //  Set m_vAnalUVDomain
  //  Set PolarConverter

  // evaluate min/max domain points
  SmExtent2d sNurbUVDomain = GetNaturalUVDomain();
  SmPoint3d  sNurbMinPoint, sNurbMaxPoint;
  EvaluatePoint(sNurbUVDomain.GetMin(),sNurbMinPoint);
  EvaluatePoint(sNurbUVDomain.GetMax(),sNurbMaxPoint);

  // scaled zero value
  double dToleranceScale = 1000.0 ;  // loose because surf is already labeled analytic
  double dScaledZero     =   dToleranceScale
                           * ANALYTIC_TOL_SCALE
                           * SM_EFF_ZERO
                           * (1.0 + sNurbMinPoint.GetMaxDimension() + sNurbMaxPoint.GetMaxDimension());
  double dAngTolRad      = SM_EFF_ZERO_SQRT * dToleranceScale ;

  // iso circle locals
  const SmContext *cpContext = GetContext() ;
  SmBoolean bFoundCirclesU, bParallelPlanesU, bSamePlaneU, bCoaxialCirclesU, bCommonAxisU, bPlanarGenCurveU;
  SmBoolean bFoundCirclesV, bParallelPlanesV, bSamePlaneV, bCoaxialCirclesV, bCommonAxisV, bPlanarGenCurveV;
  SmExtent1d sAnglesU, sAnglesV;
  double dRadiusU, dRadiusV;
  SmAxis2Placement sRefFrameU, sRefFrameV;
  SmBoolean bIsRevolutionNoSwapUV   = FALSE ;   // lDegU == 2
  SmBoolean bIsRevolutionWithSwapUV = FALSE ;   // lDegV == 2
  SmPoint3d sCommonPoint, sCommonVec ;

  // test sequence of U direction (V Constant) isoCurves for circularity and parallelism
  if(lDegU == 2)
    {
      SER(sm_TestForIsoCircles(this, SM_SP_V, dScaledZero, dAngTolRad,
                                  bFoundCirclesV, bParallelPlanesV, bSamePlaneV,
                                  bCoaxialCirclesV, bCommonAxisV, bPlanarGenCurveV,
                                  dRadiusV, sAnglesV, sRefFrameV,
                                  sCommonPoint, sCommonVec));

      // see if we Swap_UV = FALSE SurfOfRevolution
      bIsRevolutionNoSwapUV = (   bFoundCirclesV
                               && bParallelPlanesV
                               && bCoaxialCirclesV
                               && bPlanarGenCurveV) ;
    }

  // test sequence of V direction (U Constant) isoCurves for circularity and parallelism
  if(   !bIsRevolutionNoSwapUV
     && lDegV == 2)
    {
      SER(sm_TestForIsoCircles(this, SM_SP_U, dScaledZero, dAngTolRad,
                                  bFoundCirclesU, bParallelPlanesU, bSamePlaneU,
                                  bCoaxialCirclesU, bCommonAxisU, bPlanarGenCurveU,
                                  dRadiusU, sAnglesU, sRefFrameU,
                                  sCommonPoint, sCommonVec));

      // see if we Swap_UV = TRUE SurfOfRevolution
      bIsRevolutionWithSwapUV = (   bFoundCirclesU
                                 && bParallelPlanesU
                                 && bCoaxialCirclesU
                                 && bPlanarGenCurveU) ;
    }

  // no work - no sweep direction
  if(!bIsRevolutionWithSwapUV && !bIsRevolutionNoSwapUV)
    { SER_MSG(SM_ERR, _T("Modified Nurb SmSurfOfRevolution: no longer has circular IsoParamCurves ")) ; }

  // Now that the surface is a revolution of some generating curve
  // Let GenCurve = iso-curve from the 'other' parameter
  SmBSplineCurve   * pIsoCrv   = NULL ;
  SmAxis2Placement * pRefFrame = NULL ;
  SmSurfParamType    eGenDir ;
  //SmSurfParamType    eSweepDir;
  SmBoolean          bSwapUV ;
  double             dParam, dMinAngDeg, dMaxAngDeg ;
  // double             dSweepParam;
  if(bIsRevolutionNoSwapUV) { //eSweepDir   = SM_SP_V ;  // constant V isoParamCurves are circles
                              eGenDir     = SM_SP_U ;
                              bSwapUV     = FALSE ;
                              dParam      = sNurbUVDomain.GetMin().x ;
                              //dSweepParam = sNurbUVDomain.GetVInterval().Evaluate(.45678) ;
                              pRefFrame   = &sRefFrameV ;
                              dMinAngDeg  = sAnglesV.GetMin() ;
                              dMaxAngDeg  = sAnglesV.GetMax() ;
                            }
  else                      { SM_ASSERT(bIsRevolutionWithSwapUV) ;
                              //eSweepDir   = SM_SP_U ; // constant U isoParamCurves are circles
                              eGenDir     = SM_SP_V ;
                              bSwapUV     = TRUE ;
                              dParam      = sNurbUVDomain.GetMin().y ;
                              //dSweepParam = sNurbUVDomain.GetUInterval().Evaluate(.45678) ;
                              pRefFrame   = &sRefFrameU ;
                              dMinAngDeg  = sAnglesU.GetMin() ;
                              dMaxAngDeg  = sAnglesU.GetMax() ;
                            }

  // create GenCurve isoParamCurve
  SER(SmBSplineSurface::CreateIsoParametricCurve(*cpContext,
                                                  eGenDir,
                                                  dParam,
                                                  0.0,
                                                  pIsoCrv));
  SmObjDelete sClean(pIsoCrv);
  SmExtent1d sInterval = pIsoCrv->GetNaturalInterval() ;

  // no work - GenCurve is degenerate
  if (pIsoCrv->IsDegenerate(dScaledZero))
    { SER_MSG(SM_ERR, _T("Modified Nurb SmSurfOfRevolution: now has degenerate IsoParamCurve")) ; }

  // verify GenCurve is planar and coplanar with refFrame ZAxis
  SmPoint3d sPlanePoint, sPlaneNormal, sLinePoint, sLineTangent ;
  SmBoolean bIsPlanar = pIsoCrv->IsPlanar(dScaledZero, &sPlanePoint, &sPlaneNormal) ;
  SmBoolean bLinear   = pIsoCrv->IsLinear(SM_EFF_ZERO, &sLinePoint, &sLineTangent) ;
  SM_ASSERT(m_bPlanarGenerator == TRUE) ;
  if(bIsPlanar == FALSE)
    { SER_MSG(SM_ERR, _T("Modified Nurb SmSurfOfRevolution: GenCurve is no longer planar")) ; }

  // verify that plane containing genCurve also contains refFrame Z axis
  //  for nonLinear GenCurves: 1. RefFrame Origin is on genCurvePlane
  //                           2. RefFrame ZAxis is perp to GenCurvePlane Normal
  //  for    Linear GenCurves: 1. Line, RefFrame Origin, RefFrame Z axis all lie in same plane
  double dOriginPointDist   = smos_Fabs( (pRefFrame->GetOriginRef()-sPlanePoint).Dot(sPlaneNormal) ) ;
  double dFramePlaneOrtho   = smos_Fabs(sPlaneNormal.Dot(pRefFrame->GetZAxis())) ;
  double dFrameLineCoPlanar = bLinear ? smos_Fabs((pRefFrame->GetOriginRef()-sPlanePoint).Dot(pRefFrame->GetZAxis()*sLineTangent)) : 1.0 ;
  SmBoolean bIsCoPlanar     = (   (     bLinear || dOriginPointDist   < dScaledZero)
                               && (   (!bLinear && dFramePlaneOrtho   < dScaledZero)
                                   || ( bLinear && dFrameLineCoPlanar < dScaledZero))) ;
  if(bIsCoPlanar == FALSE)
    { SER_MSG(SM_ERR, _T("Modified Nurb SmSurfOfRevolution: GenCurve is no coPlanar with RefFrame Z axis")) ; }

#ifdef SM_DEBUG_CODE
  {
SmBoolean bDebugMe = FALSE ;
    if(bDebugMe)
      {
        SmFace *pFace = (SmFace *)GetFace() ;
        SmBrep *pBrep = pFace ? pFace->GetBrep() : NULL ;

        smgfx_Erase() ;
        smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
        smgfx_SetLook(1,2, 0,1,1) ; DrawUV() ; sm_GraphicsLoop() ;
        smgfx_SetLook(1,2, 0,0,0) ; if(pFace) pFace->Draw(SM_DM_CROSSHATCH) ; sm_GraphicsLoop() ;
        smgfx_SetLook(3,4, 1,0,0) ; if(pIsoCrv) pIsoCrv->Draw() ; sm_GraphicsLoop() ;
        smgfx_SetLook(5,6, 1,0,0) ; sPlanePoint.Draw() ; sPlaneNormal.Draw(&sPlanePoint) ; sm_GraphicsLoop() ;
        smgfx_SetLook(3,4, 0,1,0) ; pRefFrame->Draw() ; sm_GraphicsLoop() ;
        smgfx_SetLook(7,8, 0,0,1) ; sLinePoint.Draw() ; sLineTangent.Draw(&sLinePoint) ; sm_GraphicsLoop() ;
        sm_GraphicsLoop() ;
      }
  }

#endif // SM_DEBUG_CODE

  // set m_pGenCurve (and m_pGenCurve->Owner = this)
  if (m_pGenCurve) { delete m_pGenCurve; m_pGenCurve = NULL; }
  m_pGenCurve = pIsoCrv ;
  m_pGenCurve->SetOwner(this) ;
  sClean.Clear();

  // set Analytic parameters
  m_bPlanarGenerator = bIsPlanar ;
  m_vPosition        = *pRefFrame ;
  m_bSwapUV          = bSwapUV ;
  m_vAnalUVDomain.SetMinMax(dMinAngDeg, sInterval.GetMin(), dMaxAngDeg, sInterval.GetMax()) ;

  // Set PolarConverter from Nurb midPoint rotation isoParamCurve
  // pRotationIsoCrv will be an SmCircle because isoParam is coming from SmSurfOfRevolution
  RebuildPolarConverter() ;

  //      SmBSplineCurve *pRotationIsoCrv = NULL ;
  //      m_vPolarConverter.SetPolarConversionPossible(FALSE) ;
  //      SER(CreateIsoParametricCurve(*cpContext,
  //                                   eSweepDir,
  //                                   dSweepParam, // eSweepDir == SM_SP_U ? sMidUV.x : sMidUV.y,
  //                                   0.0, pRotationIsoCrv));
  //      SmObjDelete sCleanIsoRot(pRotationIsoCrv);
  //
  //      // Build the Surfs PolarConverter for rotation angle based on the pRotationIsoCrv
  //      sCleanIsoRot.Clear() ;
  //      SER(m_vPolarConverter.SetUpPolarConversion(pRotationIsoCrv,
  //                                                 FALSE,
  //                                                 m_vAnalUVDomain.GetUInterval(),
  //                                                 m_vPosition));

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
  if (bDebugMe)
    {
      SmBSplineCurve *pIso = NULL ;
      if (m_bSwapUV) { SER(CreateIsoParametricCurve(*cpContext,SM_SP_U,0.0,SM_EFF_ZERO,pIso));
                     }
      else           { SER(CreateIsoParametricCurve(*cpContext,SM_SP_V,0.0,SM_EFF_ZERO,pIso));
                     }
      SmObjDelete sCleanIso(pIso);

      // dump input surface, output surface, sweep isoCurve, genCurve
      Dump();
      m_pGenCurve->Dump();
      pIso->Dump();

      SmFace *pFace = (SmFace *)GetFace() ;
      SmBrep *pBrep = pFace ? pFace->GetBrep() : NULL ;

      smgfx_Erase();
      smgfx_SetLook(1,2, 0,0,1); if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop();
      smgfx_SetLook(1,2, 0,1,1); DrawUV(); sm_GraphicsLoop();
      smgfx_SetLook(1,2, 0,0,0); if(pFace) pFace->Draw(SM_DM_CROSSHATCH) ; sm_GraphicsLoop();
      smgfx_SetLook(3,4, 1,0,0); m_pGenCurve->Draw(); sm_GraphicsLoop();
      smgfx_SetLook(6,7, 1,0,1); pIso->Draw(); sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
#endif

  // all done
  SM_DUMP_AND_ASSERT2_VALID(this) ;
  return(SM_SUCCESS) ;

} // end SmSurfOfRevolution::RebuildSTEPFromNURBParameters

/*******************************************************************//**
PURPOSE: Rotate surface about axis-of-revolution.

NOTES:
***********************************************************************/
SmStatus SmSurfOfRevolution::RotationAboutAxisZ
  (const SmContext & crContext,
   double dAngleDeg,
   SmSurfOfRevolution *& rpNewSurf)
{
    SmBSplineCurve * pNewGenCurve = NULL;
    SER(CreateGeneratorFromAngle(crContext,dAngleDeg,pNewGenCurve));
    double dAngleRad = SM_DEG2RAD(dAngleDeg);
    SmAxis2Placement sRF;
    sRF.RotateAboutAxisAtPoint(dAngleRad,
                               m_vPosition.GetOriginRef(),
                               m_vPosition.GetZAxis());
    SmAxis2Placement sTmpA2P;
    m_vPosition.TransformAxis2Placement(sRF,sTmpA2P);

    rpNewSurf = new (crContext) SmSurfOfRevolution(pNewGenCurve,
        sTmpA2P.GetOriginRef(),sTmpA2P.GetXAxisRef(),sTmpA2P.GetYAxisRef(),
        m_vAnalUVDomain,m_bSwapUV);
    if (m_pNurb) {
        SER(MakeNurb());
    }

    return SM_SUCCESS;

} // end SmSurfOfRevolution::RotationAboutAxisZ


/*******************************************************************//**
PURPOSE: Rotate the given point about the Z axis until it is
  on the Z, X plane.

NOTES: When the PointToTransform is on the ZAxis, 2 solutions are returned:[Sweep.Min Sweep.Max]
***********************************************************************/
SmStatus SmSurfOfRevolution::TransformPointToStartPlane
  (const SmPoint3d & crPointToTransform,  // in : Target Point
   double            dDistTol3d,          // in : Dist3d when points are close enough to seams to return 2 answers
   SmPoint3d       & rTransformedPoint,   // out: Point rotated about rotation axis to start plane
   ULONG           & rlNumAngles,         // out:  1 - point is not on closed seam
                                          //       2 - point is on closed seam or on a pole (a point on the rotation Z axis)
   double            adAnglesDeg[2],      // out: Angles in degrees to rotate rTransformedPoint back to original position
                                          //      range:[m_vAnalDomain] or positive(0 to 360.0)
   SmBoolean       & bInside,             // out: TRUE = point is in sweep interval
                                          //      FALSE= isn't
   SmBoolean         bSnapToSeams)        // in : TRUE = rtn 2 snapped values at seams, FALSE = rtn 1 exact and 1 snapped val at seams
                                          //      TRUE=previous behavior, default:[FALSE]
 const
{
  // locals
  SmExtent1d sSweepIvlDeg = m_vAnalUVDomain.GetUInterval() ;
  double     dDistToZAxis ;

  // pass the call along
  SmStatus sRtn = smgu_TransformPointToStartPlane
    (crPointToTransform, // in : target point
     m_vPosition,        // in : rotate about Origin and ZAxis, measure angles from XAxis
     sSweepIvlDeg,       // in : Interval of supported points (commonly [0 360], think SmEllipse and SmSurfOfRevolution AnalDomains)
     dDistTol3d,         // in : Dist3d when points are close enough to seams to return 2 answers
     rTransformedPoint,  // out: output point on the X/Z plane
     dDistToZAxis,       // out: distance to Z axis
     rlNumAngles,        // out: 1 - point is not on seam
                         //      2 - point is on seam of periodic interval or on the z axis
     adAnglesDeg,        // out: Angles in degrees used to transform the point
                         //      on the plane to the original point.
                         //      range:[m_vAnalDomain] or positive(0 to 360.0)
     bInside,            // out: TRUE = point is inside trim domain
                         //      FALSE= point is outside trim domain
     bSnapToSeams) ;     // in : TRUE = rtn 2 snapped values at seams, FALSE = rtn 1 exact and 1 snapped val at seams
                         //      TRUE=previous behavior, FALSE=NewBehavior


#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  // draw
  if(bDebugMe)
    {
      Dump() ;
      SM_DUMP_AND_ASSERT_VALID(this) ;

      SmFace *pFace = (SmFace *)GetFace() ;
      SmBrep *pBrep = pFace ? pFace->GetBrep() : NULL ;

      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,1) ; DrawUV() ; sm_GraphicsLoop() ;
      smgfx_SetLook(5,6, 1,0,0) ; crPointToTransform.Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook(7,8, 0,1,0) ; rTransformedPoint.Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook(9,10,1,1,0) ; (5.0*m_vPosition.GetZAxis()).Draw(&m_vPosition.GetOriginRef()) ; sm_GraphicsLoop() ;
      smgfx_SetLook(9,10,0,1,1) ; (5.0*m_vPosition.GetXAxis()).Draw(&m_vPosition.GetOriginRef()) ; sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;
  }
#endif // SM_DEBUG_CODE

  // all done
  return sRtn ;

} // end SmSurfOfRevolution::TransformPointToStartPlane

/*******************************************************************//**
PURPOSE: Scale and transform an Revolution surface.

NOTES: Differential scaling is not allowed on analytical surfaces.
***********************************************************************/
SmStatus SmSurfOfRevolution::Transform
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

  // check input and get scaling value - only allow uniform scaling
  double dScale = 1.0 ;
  if(cpOptScale)
    {
      if(   !SM_ARE_SAME(cpOptScale->x,cpOptScale->y)
         || !SM_ARE_SAME(cpOptScale->x,cpOptScale->z)
         || cpOptScale->x < SM_EFF_ZERO)
        {
          ERR_MSG(_T("Unable to scale analytical surfaces non-uniformly\n"));
          SER(SM_ERR);
        }
      else
        {
          dScale = cpOptScale->x ;
        }
    }

  // scale current surface placement
  SmAxis2Placement sSpherePlace;
  SER(sSpherePlace.SetCanonical(m_vPosition.GetOriginRef() * dScale,
                                m_vPosition.GetXAxisRef(),
                                m_vPosition.GetYAxisRef()));

  // apply rotation to scaled placement
  SmAxis2Placement sTmpA2P;
  sSpherePlace.TransformAxis2Placement(crRotateNMove,sTmpA2P);

  // update surface placement
  m_vPosition  = sTmpA2P;

  // transform the GenCurve
  m_pGenCurve->Transform(crRotateNMove, cpOptScale) ;

  // apply transformation to underlying Nurb
  if (m_pNurb)
    {
      SER(SmBSplineSurface::Transform(crRotateNMove,cpOptScale));
    }

  // update PolarConverter
  RebuildPolarConverter() ;

  //      SmBSplineCurve *pRotationIsoCrv = NULL ;
  //      SmPoint2d       sMidUV          = m_vAnalUVDomain.Evaluate(.5678, .5678) ;
  //      SM_ASSERT(GetContext() != NULL) ;
  //      m_vPolarConverter.SetPolarConversionPossible(FALSE) ;
  //      SmStatus sRtn = CreateIsoParametricCurve(*GetContext(),
  //                                               m_bSwapUV ? SM_SP_U : SM_SP_V,
  //                                               m_bSwapUV ? sMidUV.x : sMidUV.y,
  //                                               0.0,
  //                                               pRotationIsoCrv);
  //      SM_ASSERT(sRtn == SM_SUCCESS) ;
  //
  //      // Build the Surfs PolarConverter for rotation angle based on the pRotationIsoCrv
  //      SmExtent1d sSweepIvl = m_vAnalUVDomain.GetUInterval() ;
  //      sRtn = m_vPolarConverter.SetUpPolarConversion(pRotationIsoCrv,
  //                                                    FALSE,
  //                                                    sSweepIvl,
  //                                                    m_vPosition);

  // all done
  SM_DUMP_AND_ASSERT2_VALID(this) ;
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
  if (bDebugMe)
    {
      Dump() ;

      SmFace *pFace = (SmFace *)GetFace() ;
      SmBrep *pBrep = pFace ? pFace->GetBrep() : NULL ;

      smgfx_Erase();
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,1) ; DrawUV() ; sm_GraphicsLoop() ;
      sm_GraphicsLoop();
    }
#endif
  return SM_SUCCESS;

} // end SmSurfOfRevolution::Transform

/*******************************************************************//**
PURPOSE: Trim the Surface with the given NURB domain.

NOTES:
  The input domain is a Nurbs domain, not a STEP Domain.

  This function preserves the input surface geometry and its
  parameterization at the domain corners exactly, however the surface's
  parameterization between domain corners may vary slightly but by
  amounts easily larger than reasonable tolerance sizes.

  As such, existing Edgeuse->UVTrimCurves that reference the surface
  being trimmed should be deleted and rebuilt after this call.
***********************************************************************/
SmStatus SmSurfOfRevolution::TrimWithDomain
 (SmExtent2d & rNewNurbUVDomain)  // i/o: Desired new NurbUVDomain
                                  //      set to modified surface's natural domain.
                                  //      Input trim values may be snapped
                                  //      to (or away) from existing knot values
                                  //      to prevent the creation of short spans.
{
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  if(bDebugMe)
    { 
      SM_DUMP_AND_ASSERT_VALID(this) ;
      if(this->m_pOwner) { SM_DUMP_AND_ASSERT_VALID(this->m_pOwner) ; }
    }
#endif // SM_DEBUG_CODE

  // locals
  SmExtent2d sCurrentNurbUVDomain = GetNaturalUVDomain();

  // check input - new trim UV domain must be inside current natural UV domain
  if (!rNewNurbUVDomain.IsContainedBy(sCurrentNurbUVDomain, SM_EFF_ZERO_PARAM))
    { SER(SM_ERR_OUTSIDE_OF_DOMAIN); }

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
      if(this->m_pOwner) { SM_DUMP_AND_ASSERT_VALID(this->m_pOwner) ; }

      SmFace *pFace = (SmFace *)this->GetFace() ;
      SmBrep *pBrep =   pFace ? pFace->GetBrep() : NULL ;

      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,1) ; this->DrawUV(8,8,FALSE,NULL,TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,0,0) ; if(pFace) pFace->Draw(SM_DM_CROSSHATCH) ; sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

  // tell the public the Surface is about to be edited
  Notify(SM_NO_PRE_EDIT, this, SM_NO_GET_OWNER(this), NULL);

  // Update just the analytic domain and the GenCurve (both analytic and NURB)
  UpdateAnalyticalDomain(rNewNurbUVDomain) ;

  // rebuild Nurbs surface from updated analytic domain. (right shape - wrong UVDomain)
  SER(MakeNurb());

  // adjust Nurb Surface UVDomain
  this->Reparameterize(rNewNurbUVDomain) ;

  // rebuild the sweep polar converter
  RebuildPolarConverter() ;

  // tell the public the Surface has been edited
  Notify(SM_NO_POST_EDIT, this, SM_NO_GET_OWNER(this), NULL);

#ifdef SM_DEBUG_CODE
  if(bDebugMe)
    {
      if(   (FALSE == SM_ASSERT_VALID_NO_STREAM(this))
         || (   this->m_pOwner != NULL
             && FALSE == SM_ASSERT_VALID_NO_STREAM(this->m_pOwner)))
        {
          // place for a breakpoint
          SM_DUMP_AND_ASSERT_VALID(this) ;
          if(this->m_pOwner) { SM_DUMP_AND_ASSERT_VALID(this->m_pOwner) ; }
        }
    }
#endif // SM_DEBUG_CODE

  // // GWC: Old method - results in occasional short spans and as implemented here
  // //      causes a conflict between the GenCurve and the m_pNurb parameterization
  //
  // // locals
  // SmExtent2d sOrigDomain = rNewNurbUVDomain;
  // SmBoolean  bInsideOut  = GetInsideOut() ;
  //
  // // Get StepUV corners for requested TrimDomain Boundary
  // SmPoint2d  sStepUVMin, sStepUVMax;
  // SER(ConvertUVFromNURBSToSTEP(rNewNurbUVDomain.GetMin(),sStepUVMin));
  // SER(ConvertUVFromNURBSToSTEP(rNewNurbUVDomain.GetMax(),sStepUVMax));
  //
  // // Trim underlying Nurb surface.
  // //  Note, if the requested trim domain comes close to existing knots,
  // //        it might be snapped to, or away from, those knots.
  // //        So rNewNurbUVDomain might come back modified (slightly).
  // //        That's another reason all the UVTrimCurves associated with the
  // //        surface being trimmed need to be deleted and rebuilt.
  // SER(SmBSplineSurface::TrimWithDomain( rNewNurbUVDomain ));
  //
  // // Modified rNewNurbUVDomain should match natural domain.
  // SM_ASSERT(   rNewNurbUVDomain.IsContainedBy(GetNaturalUVDomain())
  //           && GetNaturalUVDomain().IsContainedBy(rNewNurbUVDomain, SM_EFF_ZERO)) ;
  //
  // // If the trim domain was modified due to knot snapping (or knot spacing):
  // if( !(   rNewNurbUVDomain.IsContainedBy(sOrigDomain, SM_EFF_ZERO)
  //       && sOrigDomain.IsContainedBy(rNewNurbUVDomain, SM_EFF_ZERO)) )
  //   {
  //     // Get StepUV Points to match new trim NurbDomain corners.
  //     // Evaluate the Nurbs surface at min and max corners,
  //     // and do STEP inversions with those 3d points
  //     // to get the corresponding Step UV points.
  //
  //     // locals
  //     SmLocationType eMinLocation,     eMaxLocation;
  //     SmPoint2d      sMinAnalUVParams, sMaxAnalUVParams;
  //     SmPoint3d      sMinPoint,        sMaxPoint;
  //
  //     // Get corner points for new NurbDomain
  //     //  Note: at this point, m_vAnalUVDomain has not been changed,
  //     //  but our Nurb domain has, so SmCone::EvaluatePoint() will not
  //     //  work correctly.  (ConvertUVFromNURBSToSTEP() will be wrong.)
  //     //  Handle that by calling the Nurbs evaluator explicitly. [B58]
  //
  //     SER( SmBSplineSurface::EvaluatePoint( rNewNurbUVDomain.GetMin(), sMinPoint ));
  //     SER( SmBSplineSurface::EvaluatePoint( rNewNurbUVDomain.GetMax(), sMaxPoint ));
  //
  //     double dScaledZero = SM_EFF_ZERO_SQRT * (  1.0
  //                                              + sMinPoint.GetMaxDimension()
  //                                              + sMaxPoint.GetMaxDimension());
  //
  //     // Get StepUVs for NewDomainCorners - set sweep seam angles to 0.0
  //     SER(STEPInversion(m_vAnalUVDomain, sMinPoint, dScaledZero, sMinAnalUVParams, eMinLocation));
  //     SER(STEPInversion(m_vAnalUVDomain, sMaxPoint, dScaledZero, sMaxAnalUVParams, eMaxLocation));
  //
  //     // snap sweep seams to analytic boundaries
  //     if(   eMinLocation == SM_LT_POLE
  //        || eMinLocation == SM_LT_U_SEAM
  //        || eMinLocation == SM_LT_UV_SEAM) { sMinAnalUVParams.x = 0.0; }
  //     if(   eMaxLocation == SM_LT_POLE
  //        || eMaxLocation == SM_LT_U_SEAM
  //        || eMaxLocation == SM_LT_UV_SEAM) { sMaxAnalUVParams.x = 360.0; }
  //
  //     // snap genCurve seams to analytic boundaries (torus only)
  //     // Analytic boundaries for the v-direction are those of the gen curve. [B171]
  //     SmExtent1d sVIvl = m_pGenCurve->GetNaturalInterval();
  //     if(   eMinLocation == SM_LT_V_SEAM
  //        || eMinLocation == SM_LT_UV_SEAM)
  //       { sMinAnalUVParams.y = bInsideOut ? sVIvl.GetMax() : sVIvl.GetMin(); }
  //     if(   eMaxLocation == SM_LT_V_SEAM
  //        || eMaxLocation == SM_LT_UV_SEAM)
  //       { sMaxAnalUVParams.y = bInsideOut ? sVIvl.GetMin() : sVIvl.GetMax(); }
  //
  //     // save StepUVs for NurbDomain CornerPoints
  //     sStepUVMin = sMinAnalUVParams;
  //     sStepUVMax = sMaxAnalUVParams;
  //
  //   } // end tolerance roundoff problem check
  //
  // // When InsideOut - (cone, sphere, and torus)
  // //  map NurbMin => StepMax and NurbMax => StepMin
  // if(bInsideOut)
  //   {
  //     double dTemp = sStepUVMin.y ;
  //     sStepUVMin.y = sStepUVMax.y ;
  //     sStepUVMax.y = dTemp ;
  //   }
  //
  // // for cones only
  // // Adjust position to place origin at new trimmed start of cone
  // SmBoolean bAdjConeAnalDomain = FALSE ;
  // if(IsKindOf(SmCone_TYPE))
  //   {
  //     double dNewBotRadius = ((SmCone *)this)->GetRadius(sStepUVMin.y) ;
  //     SmPoint3d sNewOrigin = (  m_vPosition.GetOriginRef()
  //                             + (sStepUVMin.y - m_vAnalUVDomain.GetMin().y)
  //                             * m_vPosition.GetZAxis()) ;
  //     m_vPosition.SetCanonical(sNewOrigin,
  //                              m_vPosition.GetXAxisRef(),
  //                              m_vPosition.GetYAxisRef());
  //
  //     ((SmCone *)this)->SetBotRadius(dNewBotRadius) ;
  //     // translate new anal domain
  //     if(sStepUVMin.y != 0.0)
  //       {
  //         bAdjConeAnalDomain = TRUE ;
  //         sStepUVMax.y -= sStepUVMin.y ;
  //         sStepUVMin.y  = 0.0 ;
  //       }
  //   } // end IsKindOf cone check - to set analDomain 0 = new origin
  //
  // // check linear param is increasing
  // SM_ASSERT(sStepUVMax.y >= sStepUVMin.y) ;
  //
  // // make sure sweep angles are increasing
  // if (sStepUVMin.x > sStepUVMax.x)
  //   {
  //     double dTmp = sStepUVMin.x;
  //     sStepUVMin.x = sStepUVMax.x;
  //     sStepUVMax.x = dTmp;
  //   }
  //
  // // trim the GenCurve
  // SmExtent1d sNurbTrim ;
  // if (m_bSwapUV) { sNurbTrim.SetMinMax(rNewNurbUVDomain.GetMin().x, rNewNurbUVDomain.GetMax().x); }
  // else           { sNurbTrim.SetMinMax(rNewNurbUVDomain.GetMin().y, rNewNurbUVDomain.GetMax().y); }
  //
// #ifdef SM_DEBUG_CODE
//   if(bDebugMe)
//     { SM_DUMP_AND_ASSERT_VALID(m_pGenCurve) ; }
// #endif // SM_DEBUG_CODE
//
//   m_pGenCurve->Trim(sNurbTrim);  // may snap sIvl by tol to existing knots
//
// #ifdef SM_DEBUG_CODE
//   if(bDebugMe)
//     { SM_DUMP_AND_ASSERT_VALID(m_pGenCurve) ; }
// #endif // SM_DEBUG_CODE
//
  // // set the analUVDomain
  // m_vAnalUVDomain.SetMinMax(sStepUVMin,sStepUVMax);
  //
  // // for cones only - when needed
  // // Adjust position to place origin of genCurve at new trimmed start of cone
  // if(bAdjConeAnalDomain)
  //   {
  //     // genCurve is a line
  //     SM_ASSERT(m_pGenCurve->IsKindOf(SmLine_TYPE)) ;
  //     SmLine *pLine = (SmLine *)m_pGenCurve ;
  //
  //     // Get line STEP parameters
  //     SmVector3d sLinePoint, sLineVector, sNewLinePoint ;
  //     double     dLineScale ;
  //     SmExtent1d sLineIvl ;
  //     pLine->GetCanonical(sLinePoint, sLineVector, dLineScale, sLineIvl) ;
  //
  //     // translate Line origin and interval to place LinePoint at new LineStart
  //     pLine->EvaluateSTEPPoint( sLineIvl.GetMin(), sNewLinePoint );
  //     sLineIvl.SetMinMax(0, sLineIvl.GetLength()) ;
  //
  //     // set Line to start from new origin
  //     pLine->SetCanonical(sNewLinePoint, sLineVector, dLineScale, sLineIvl) ;
  //
  //     // SetCanonical sets NurbInterval = AnalInterval = sLineIvl
  //     // Make sure that GenCurve->NurbInterval  == m_bSwapUV ? sNaturalUVDomain.GetUInterval : sNaturalUVDomain.GetVInterval
  //     pLine->SmBSplineCurve::EditParameterization(  m_bSwapUV
  //                                                 ? GetNaturalUVDomain().GetUInterval()
  //                                                 : GetNaturalUVDomain().GetVInterval()) ;
  //
  //
  //   } // end need to adjust GenCurve domain check
  //
  // // rebuild the sweep polar converter
  // RebuildPolarConverter() ;
  //
  //
#ifdef SM_DEBUG_CODE
  if(bDebugMe)
    {
      SM_DUMP_AND_ASSERT_VALID(this) ;
      if(this->m_pOwner) { SM_DUMP_AND_ASSERT_VALID(this->m_pOwner) ; }

      // copy surface in case we want to compare before/after deformations
      SmSolutionArray sSolutions ;
      pCopySurface->GlobalSurfaceSolve(this->GetNaturalUVDomain(),
                                       *this,
                                       this->GetNaturalUVDomain(),
                                       SM_SO_MAXIMIZE, SM_EFF_ZERO, NULL, NULL,
                                       SM_SR_ALL, sSolutions) ;

      rNewNurbUVDomain.Dump() ;
      sSolutions.Dump() ;

      SmFace     * pFace = (SmFace *)this->GetFace() ;
      SmBrep     * pBrep =   pFace ? pFace->GetBrep() : NULL ;
      SmExtent2d   sDom( this->GetNaturalUVDomain() );
      SmZoneTol3d  sZoneTol3d = pFace ? (double)pFace->GetTolerance() : pBrep ? (double)pBrep->GetTolerance() : 0.00001 ;

      // drop points from this surface to original Copy over This surface domain
      // to see if surfaces are geometrically equivalent
      // static ULONG dSmpCnt = 40 ;
      SmSrfSrfGapFunction sSrfSrfGap( (SmXSectTol3d)sZoneTol3d,
                                      this, sDom,
                                      pCopySurface, 40, 40) ;
      sSrfSrfGap.Dump() ;

      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,1) ; this->DrawUV(8,8,FALSE,NULL,TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(4,5, 1,0,0) ; m_pGenCurve->Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook(4,5, 1,0,1) ; m_vPolarConverter.GetCurve()->Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,0,0) ; if(pFace) pFace->Draw(SM_DM_CROSSHATCH) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 1,0,0) ; sSrfSrfGap.Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook(2,3, 1,0,0) ; sSolutions.Draw() ; sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

  // all done
  return SM_SUCCESS;

} // end SmSurfOfRevolution::TrimWithDomain

//      /*******************************************************************//**
//      PURPOSE: Trim the Surface of Revolution with the given domain.
//
//      NOTES:
//      ***********************************************************************/
//      SmStatus SmSurfOfRevolution::TrimWithDomain
//        (SmExtent2d & rTrimDomain)           // in : desired Nurb Surface domain,
//                                             //      must be subset or equal to current NaturalUVDomain
//      {
//        SmExtent2d sNaturalDomain = GetNaturalUVDomain();
//
//        // Make sure trim domain is inside of natural domain
//        if (!rTrimDomain.IsContainedBy(sNaturalDomain))
//          { SER(SM_ERR); }
//
//        // no work - new domain
//        if (sNaturalDomain.IsContainedBy(rTrimDomain))
//          { return SM_SUCCESS; }
//
//        // Set Surface Analytical domain to requested Trim limits
//        SmPoint2d sStepUVMin, sStepUVMax;
//        SER(ConvertUVFromNURBSToSTEP(rTrimDomain.GetMin(),sStepUVMin));
//        SER(ConvertUVFromNURBSToSTEP(rTrimDomain.GetMax(),sStepUVMax));
//
//        m_vAnalUVDomain.SetMinMax(sStepUVMin,sStepUVMax);
//
//        // Trim GenCurve Analytic/Nurb domain
//        SmExtent1d sCrvNurbIvl = m_bSwapUV
//                                 ? rTrimDomain.GetUInterval()
//                                 : rTrimDomain.GetVInterval() ;
//        m_pGenCurve->Trim(sCrvNurbIvl) ; // may snap sIvl by tol to existing knots
//        // SER(m_pGenCurve->AdjustSTEPInterval(sCrvIvl));
//
//        // Trim Surface Nurb domain
//        SER(SmBSplineSurface::TrimWithDomain(rTrimDomain));
//
//        // all done
//        SM_DUMP_AND_ASSERT2_VALID(this) ;
//      #ifdef SM_DEBUG_CODE
//      SmBoolean bDebugMe = FALSE;
//        if (bDebugMe)
//          {
//            rTrimDomain.Dump() ;
//            Dump() ;
//
//            SmFace *pFace = (SmFace *)GetFace() ;
//            SmBrep *pBrep = pFace ? pFace->GetBrep() : NULL ;
//
//            smgfx_Erase();
//            smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
//            smgfx_SetLook(1,2, 0,1,1) ; DrawUV() ; sm_GraphicsLoop() ;
//            sm_GraphicsLoop();
//          }
//      #endif
//        return SM_SUCCESS;
//
//      } // end SmSurfOfRevolution::TrimWithDomain

/*******************************************************************//**
PURPOSE: Update our Analytic domain to the given Nurbs domain.

NOTES: Operates only on the analytic domain member and any
  generator curves.  Assumes the Nurbs representation has been dealt with.
***********************************************************************/
SmStatus SmSurfOfRevolution::UpdateAnalyticalDomain
 (const SmExtent2d & crNewNurbsDomain) // in : Desired new Nurb UVDomain
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
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,1) ; this->DrawUV(8,8,FALSE,NULL,TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,0,0) ; if(pFace) pFace->Draw(SM_DM_CROSSHATCH) ; sm_GraphicsLoop() ;
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

  // set GenCurve analytic domain from V part of new surface domain
  SER( m_pGenCurve->AdjustSTEPInterval( m_vAnalUVDomain.GetVInterval() ));

  // make sure m_pGenCurve Nurb domain matches crNewNurbsDomain (some AdjustSTEPInterval methods return [0 1])
  SmExtent1d sGenIvl =   m_bSwapUV
                       ? crNewNurbsDomain.GetUInterval()
                       : crNewNurbsDomain.GetVInterval() ;
  m_pGenCurve->ScaleKnotVector(sGenIvl.GetMin(), sGenIvl.GetMax()) ;

  // all done
  return SM_SUCCESS;

} // end UpdateAnalyticalDomain

/*******************************************************************//**
PURPOSE: Test BSplineSurface at a sequence of isoParameter Curves
            to see if each Isocurve is a circular arc with a common
            reference frame.

NOTES: When iscurves are not all circles - rbFoundCircles == FALSE
***********************************************************************/
SmStatus sm_TestForIsoCircles
  (const SmBSplineSurface * pTestSurface,    // in : Surface to check
   SmSurfParamType          eSurfParam,      // in : isoParameter direction
   double                   d3DTol,          // in : 3d distance tolerance
   double                   dAngTolRad,      // in : Max allowed angular deviation for planarGenCurve checks
   SmBoolean             & rbFoundCircles,   // out: TRUE = all tested isoParameterCurves are circular arcs
   SmBoolean             & rbParallelPlanes, // out: TRUE = all tested isoParameterCurves are on parallel planes
   SmBoolean             & rbSamePlane,      // out: TRUE = all tested isoParameterCurves are on the same plane
   SmBoolean             & rbCoaxialCenters, // out: TRUE = centers of all iso circles are on a common axis
   SmBoolean             & rbCommonAxis,     // out: TRUE = all tested isoParameterCurves lie on plane with 1 single common axis (tori minor curves)
   SmBoolean             & rbPlanarGenCurve, // out: TRUE = All circular arcs start and stop on same plane
   double                & rdRadius,         // out: radius of 1st nonDegenerate isoParameter curve tested
   SmExtent1d            & rAngles,          // out: Start/End angles for 1st nonDegenerate isoParameter curve tested
   SmAxis2Placement      & rReferenceFrame,  // out: ref frame for 1st nonDegenerate isoParameter curve tested
   SmPoint3d             & rCommonLinePoint, // out: when rbCommonAxis == TRUE - Point  of common isoParam Plane line
   SmVector3d            & rCommonLineVec)   // out: when rbCommonAxis == TRUE - UnitVector of common isoParam Plane line
{
  // init outputs
  rbFoundCircles   = TRUE;
  rbParallelPlanes = TRUE;
  rbSamePlane      = TRUE;
  rbCoaxialCenters = TRUE;
  rbPlanarGenCurve = TRUE;
  rbCommonAxis     = TRUE;

  // locals
  ULONG              lNumCheck   = 3;
  SmBoolean          bFirst      = TRUE;
  SmBoolean          bSecond     = FALSE ;
  SmBoolean          bCoincident = FALSE ;
  SmBSplineCurve   * pIsoCrv     = NULL;
  const SmVector3d * pXAxis      = NULL ;
  SmVector3d         sZAxis ;
  SmPoint3d          sThisCommonLinePoint ;
  SmVector3d         sThisCommonLineVec ;
  double             dStartAngDeg1=0.0 ;
  double             dEndAngDeg1=0.0 ;

  // get surface knots in test directon
  SmTArray<double> sKnots;
  SER(pTestSurface->GetKnots(eSurfParam,sKnots));

  // Build testValues: for every knot value + lNumCheck sample points in between
  SmTArray<double> sTestVals;
  sTestVals.Add(sKnots[0]);
  for (ULONG kk=1; kk<sKnots.GetSize(); kk++)
    {
      double dLastKnot = sKnots[kk-1];
      double dCurrKnot = sKnots[kk];
      for (ULONG ll=0; ll+1<lNumCheck; ll++)  // note: can't say lNumCheck-1
        {
          double dT = (ll + 1.0) / lNumCheck;
          sTestVals.Add(dLastKnot + dT * (dCurrKnot - dLastKnot));
        }
      sTestVals.Add(sKnots[kk]);
    }

  // for every test value
  ULONG nDegen = 0;
  for (ULONG jj=0; jj<sTestVals.GetSize(); jj++)
    {
      // make isoParameter Curve
      SER(pTestSurface->SmBSplineSurface::CreateIsoParametricCurve(*pTestSurface->GetContext(),
                                                                    eSurfParam,
                                                                    sTestVals[jj],
                                                                    0.0,
                                                                    pIsoCrv));
      SmObjDelete sClean(pIsoCrv);

      // skip degenerate curves
      if (pIsoCrv->IsDegenerate(d3DTol))
        { nDegen++; continue; }

      // when isoParameterCurve is not a circular Arc
      SmAxis2Placement sThisRefFrame;
      double dRad, dStartAngDeg, dEndAngDeg;
      if (!pIsoCrv->IsArc(5,d3DTol,sThisRefFrame,dRad,dStartAngDeg,dEndAngDeg))
        {
          rbFoundCircles = FALSE;
          return SM_SUCCESS;
        }

      // The first time through get output reference frame, radius and angle extent
      if (bFirst)
        {
          bFirst          = FALSE;
          bSecond         = TRUE ;
          rdRadius        = dRad;
          rAngles         = SmExtent1d(dStartAngDeg,dEndAngDeg);
          rReferenceFrame = sThisRefFrame;
          sZAxis          = rReferenceFrame.GetZAxis() ;
          pXAxis          = &rReferenceFrame.GetXAxisRef() ;
          dStartAngDeg1   = dStartAngDeg ;
          dEndAngDeg1     = dEndAngDeg ;
        }
      else // compare 1st reference frame to current one
        {
          // when Z axes are not parallel - isoParameter curves are not on parallel planes
          if ( rbParallelPlanes )
            {
              if (sZAxis.Dot(sThisRefFrame.GetZAxis()) < 1.0-SM_EFF_ZERO)
                {
                  rbParallelPlanes = FALSE;
                  rbSamePlane      = FALSE;
                }
            }

          // (SamePlane is already false if ParallelPlanes is.)
          if ( rbSamePlane )
            {
              SmVector3d sDiff(  sThisRefFrame.GetOriginRef()
                               - rReferenceFrame.GetOriginRef() );
              double dLen = sDiff.Length();
              if ( dLen >= d3DTol ) // if zero length, then it's in the same plane.
                {
                  sDiff /= dLen;
                  if ( smos_Fabs( sDiff.Dot( sZAxis ) ) > d3DTol )
                    {
                      rbSamePlane = FALSE;
                    }
                }
            }

          // Check centers all on same axis
          if ( rbCoaxialCenters )
            {
              double dDistFromAxis = d3DTol * 2;
              SmStatus eStat = smgu_LinePointDistance(
                      rReferenceFrame.GetOriginRef(),
                      sZAxis,
                      sThisRefFrame.GetOriginRef(),
                      dDistFromAxis );

              if ( dDistFromAxis > d3DTol || eStat != SM_SUCCESS )
                {
                  rbCoaxialCenters = FALSE;
                }
            }

          // when arcs do not start and stop on same plane
          if ( rbPlanarGenCurve )
            {
              double dAngRad;
              pXAxis->AngleBetween(sThisRefFrame.GetXAxisRef(),dAngRad);
              // angle test only
              if (   smos_Fabs(dAngRad) > dAngTolRad
                  || smos_Fabs(dStartAngDeg1-dStartAngDeg) > dAngTolRad
                  || smos_Fabs(dEndAngDeg1-dEndAngDeg)     > dAngTolRad)
                {
                  rbPlanarGenCurve = FALSE ;
                }

              // position test
              if (   smos_Fabs(dRad * dAngRad) > d3DTol
                  || smos_Fabs(dRad * SM_DEG2RAD(dStartAngDeg1-dStartAngDeg)) > d3DTol
                  || smos_Fabs(dRad * SM_DEG2RAD(dEndAngDeg1-dEndAngDeg))     > d3DTol)
                {
                  rbPlanarGenCurve = FALSE ;
                }
            } // end if rbPlanarGenCurve == TRUE

          // find line of intersection between two isoCurve planes
          if(rbCommonAxis)
            {
              SmStatus sRtn = smgu_IntersectTwoPlanes(rReferenceFrame.GetOriginRef(),
                                                      rReferenceFrame.GetZAxis(),
                                                      sThisRefFrame.GetOriginRef(),
                                                      sThisRefFrame.GetZAxis(),
                                                      sThisCommonLinePoint,
                                                      sThisCommonLineVec,
                                                      &bCoincident) ;
              // when this is the 2nd isoParamCurve
              if(bSecond)
                {
                  // when planes intersected uniquely - save common line
                  if(sRtn == SM_SUCCESS)        { bSecond          = FALSE ;
                                                  rCommonLinePoint = sThisCommonLinePoint ;
                                                  rCommonLineVec   = sThisCommonLineVec ;
                                                } // end second isoCurve check
                  // when planes were parallel but not coincident
                  else if(bCoincident == FALSE) { rbCommonAxis = FALSE ;
                                                }
                } // end second unique isoCurvePlane existence check
              else // compare subsequent common lines against 1st common line
                {
                  // lines must be parallel
                  // SmBoolean bAreParallel   = rCommonLineVec.IsParallelTo(sThisCommonLineVec, 2.0) ;
                  double dDist1, dDist2 ;
                  smgu_LinePointDistance(rCommonLinePoint, rCommonLineVec,
                                         sThisCommonLinePoint, dDist1) ;
                  smgu_LinePointDistance(sThisCommonLinePoint, sThisCommonLineVec,
                                         rCommonLinePoint, dDist2) ;
                  if(   dDist1 > d3DTol
                     || dDist2 > d3DTol)
                    {
                      rbCommonAxis = FALSE ;
                    }
                } // end compare subsequent common lines branch
            } // end commonAxis check

        } // end compare coordinate systems branch
    } // end iter every test value

  // all done
  if( nDegen == sTestVals.GetSize() )
    { return SM_ERR; }

  return SM_SUCCESS;

} // end sm_TestForIsoCircles

/*******************************************************************//**
PURPOSE: Compute the total size of the memory used by the SmSurfOfRevolution.

NOTES: Does not add in attribute memory
***********************************************************************/
ULONG SmSurfOfRevolution::GetMemoryUsed   // rtn: smaller size of actually used memory in bytes
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

} // end SmSurfOfRevolution::GetMemoryUsed

/*******************************************************************//**
PURPOSE: AssertValid() Test reporting static labels
***********************************************************************/
SmAssertReportLabel sAssertSurfOfRevolution_list[] =
{
  /*  0 */ {SM_AT_POINTER,          _T("Context"),             _T("Bad m_pGenCurve does not share same context") },
  /*  1 */ {SM_AT_POINTER,          _T("Context"),             _T("Bad m_vPolarConverter does not share same context") },
  /*  2 */ {SM_AT_PARAMETERIZATION, _T("Parameterization"),    _T("Bad GenCurve Anal and NURB intervals don't match appropriate Surface Domain Intervals") },
  /*  3 */ {SM_AT_GEOMETRIC,        _T("Midpoint"),            _T("Bad Analytic and NURB GenCurve midPoint values are not the same") },
  /*  4 */ {SM_AT_SIZE,             _T("Size"),                _T("Bad PolarConverter lAngCount is not greater than one") },
  /*  5 */ {SM_AT_ANGLE,            _T("Start Angle"),         _T("Bad PolarConverter First polar angle does not equal start SweepInterval bounds") },
  /*  6 */ {SM_AT_ANGLE,            _T("End Angle"),           _T("Bad PolarConverter Last polar angle does not equal end SweepInterval bounds") },
  /*  7 */ {SM_AT_PARAMETERIZATION, _T("Parameterization"),     _T("Bad PolarCurve Anal and NURB intervals does not match appropriate Surface Domain Intervals") },
  /*  8 */ {SM_AT_GEOMETRIC,        _T("Sweep midpoint"),      _T("Bad Analytic and NURB Evaluate Sweep near midpoint values not the same") },
  /*  9 */ {SM_AT_DISTANCE,         _T("Dist V"),              _T("Bad GenCurve eval does not map to Surface eval within dScaledZero") },
  /* 10 */ {SM_AT_DISTANCE,         _T("Dist U"),              _T("Bad GenCurve eval does not map to Surface eval within dScaledZero") },
  /* 11 */ {SM_AT_DOMAIN,           _T("InsideOut GenCurve"),  _T("Bad GenCurve and Surface bInsideOut bit values don't match") },
  /* 12 */ {SM_AT_DOMAIN,           _T("InsideOut PolarCurve"),_T("Bad PolarCurve bInsideOut bit values is not FALSE") },
  /* 13 */ {SM_AT_GEOMETRIC,        _T("Overlapping Domain"),  _T("Bad Analytic U Direction Domain too large: SurfOfRevolution can't overlap themselves") },
  /* 14 */ {SM_AT_POINTER,          _T("Bad GenCurve Owner"),  _T("Bad GenCurve owner is not this SmSurfOfRevolution object") },
  /* 15 */ {SM_AT_SIZE,             _T("Size"),                _T("Bad GenCurve length degenerates SurfOfRevolution to an approximate circular curve") }
} ;

/*******************************************************************//**
PURPOSE: Make sure GenCurve and Surface parameterizations are
            compatible.

RETURNS ---  TRUE  = OK
             FALSE = Problem
***********************************************************************/
SmBoolean SmSurfOfRevolution::AssertValid
 (SmAssertArray    * pAList,        // i/o: Accumulating list of failed Asserts, NULL to ignore, default:[NULL]
  SmAssertTestLevel  eTestLevel,    // in : SM_LEVEL_0=fewest tests, SM_LEVEL_1, SM_LEVEL_2=all tests,
                                    //      SM_LEVEL_GIVEN = run tests in order requested in pTestRequests
                                    //      default:[SM_LEVEL_0]
  SmAssertWalking    eWalkTree,     // NotUsed: in : SM_WALK = run AssertValid on any topology graph descendants, SM_NO_WALK=don't, default:[SM_WALK]
  SmTArray<ULONG>  * pTestRequests) // in : when eTestLevel == SM_LEVEL_GIVEN, run these tests in this order, default:[NULL]
 const
{
  SM_REF1(eWalkTree) ;
  SmBoolean bRtn = TRUE ;

  // call the base class AssertValid
  if(m_pNurb)
    {
      bRtn &= (  (eTestLevel != SM_LEVEL_GIVEN)
               ? SmBSplineSurface::AssertValid(pAList, eTestLevel, SM_NO_WALK, pTestRequests)
               : TRUE ) ;
    }

  // all contained pointers to crContext should be the same
  /*  0 */ // m_pGenCurve shares the same context
  bRtn &= SM_ASSERT_BOOLEAN_REPORT(0, SM_LEVEL_0, (m_pGenCurve == NULL || m_cpContext == m_pGenCurve->GetContext()), _T("") ) ;

  /*  1 */ // m_vPolarConverter shares the same context
  bRtn &= SM_ASSERT_BOOLEAN_REPORT(1, SM_LEVEL_0, (m_vPolarConverter.GetCurve() == NULL || m_cpContext == m_vPolarConverter.GetCurve()->GetContext()), _T("") ) ;

  // when given an m_pNurb and m_pGenCurve
  if(m_pGenCurve)
    {
      /* 11 */ // GenCurve and Surface InsideOut bit values must match
      bRtn &= SM_ASSERT_BOOLEAN_REPORT(11, SM_LEVEL_0, (m_pGenCurve->GetInsideOut() == GetInsideOut()), _T("")) ;

      if(m_pNurb)
        {
          // The GenCurve parameterization must match the Surface Domain Intervals as
          //    1. GenCurve AnalInterval == Surface AnalVInterval
          //    2. GenCurve NURBInterval == m_bSwapUV ? Surface->NURBUInterval : Surface->NURBVInterval
          //    3. GenCurve KnotVector   == m_bSwapUV ? Surface->NURBUKnot : Surface->NURBVKnot
          //
          // When PolarConverter has a PolarCurve, the PolarCurve parameterization must match the Surface Domain Intervals as
          //    1. PolarCurve AnalInterval == Surface AnalUInterval
          //    2. PolarCurve NURBInterval == m_bSwapUV ? Surface->NURBVInterval : Surface->NURBUInterval
          //    3. PolarCurve KnotVector   == m_bSwapUV ? Surface->NURBVKnot : Surface->NURBUKnot
          /*  2 */ // GenCurve Anal and NURB intervals must match appropriate Surface Domain Intervals
          bRtn &= SM_ASSERT_VALUE_REPORT(2, SM_LEVEL_0,
                                         (HasSameParameterization(SM_SP_V,        // in : Analytic Domain Direction
                                                                    m_bSwapUV     // in : Nurb     Domain Direction
                                                                  ? SM_SP_U
                                                                  : SM_SP_V,
                                                                  m_pGenCurve) ),
                                         SM_EFF_ZERO, SM_UNDEF_DOUBLE, _T("") ) ;

          /*  3 */ // compare a GenCurve near midPoint evaluation
          if(bRtn && m_vPolarConverter.IsPolarConversionPossible())
            {
              SmPoint2d sNurbUV, sStepUV = m_vAnalUVDomain.Evaluate(0.0, .4567) ;
              ConvertUVFromSTEPToNURBS(sStepUV, sNurbUV) ;
               SmPoint3d sStepPoint, sNurbPoint ;
              EvaluateSTEPPoint(sStepUV, sStepPoint) ;
              EvaluatePoint(sNurbUV, sNurbPoint) ;
              double dScaledZero = SM_EFF_ZERO * (1.0 + sStepPoint.GetMaxDimension()) ;
              bRtn &= SM_ASSERT_VALUE_REPORT(3, SM_LEVEL_0, (sStepPoint.CloserThan(dScaledZero, sNurbPoint)), dScaledZero, sStepPoint.DistanceBetween(sNurbPoint), _T("") ) ;
           }
       } // end m_pNurb existence check
   } // end m_pGenCurve existence check

  // polarConverter angle range must equal the sweep angle range
  double     dScaledAngZero = SM_EFF_ZERO * 100.0 * 360.0 ;
  SmExtent1d sSweepIvlDeg   = m_vAnalUVDomain.GetUInterval() ;
  double     dSweepMinDeg   = sSweepIvlDeg.GetMin() ;
  double     dSweepMaxDeg   = sSweepIvlDeg.GetMax() ;
  ULONG      lAngCount      = m_vPolarConverter.GetAngleCount() ;

  // polarConverter angle range is only defined when PolarConversion is possible
  if(bRtn && m_vPolarConverter.IsPolarConversionPossible())
    {
      double     dFirstAngDeg   = m_vPolarConverter.GetFirstAngle() ;
      double     dLastAngDeg    = m_vPolarConverter.GetLastAngle() ;

      /*  4 */ // lAngCount must be greater than one
      bRtn &= SM_ASSERT_BOOLEAN_REPORT(4, SM_LEVEL_0, (lAngCount > 1), _T("") ) ;

      /*  5 */ // First polar angle is start SweepInterval bounds
      bRtn &= SM_ASSERT_VALUE_REPORT(5, SM_LEVEL_0,
                                    (   SM_ARE_SAME_TO_TOL(dFirstAngDeg,dSweepMinDeg,dScaledAngZero)
                                     || SM_ARE_SAME_TO_TOL(dFirstAngDeg,dSweepMaxDeg,dScaledAngZero)),
                                     dScaledAngZero, smos_Max(smos_Fabs(dFirstAngDeg-dSweepMinDeg),smos_Fabs(dFirstAngDeg-dSweepMaxDeg)), _T("") ) ;

      /*  6 */ // Last polar angle must equal end SweepInterval bounds
      bRtn &= SM_ASSERT_VALUE_REPORT(6, SM_LEVEL_0,
                                     (   SM_ARE_SAME_TO_TOL(dLastAngDeg,dSweepMinDeg,dScaledAngZero)
                                      || SM_ARE_SAME_TO_TOL(dLastAngDeg,dSweepMaxDeg,dScaledAngZero)),
                                     dScaledAngZero, smos_Max(smos_Fabs(dLastAngDeg-dSweepMinDeg),smos_Fabs(dLastAngDeg-dSweepMaxDeg)), _T("") ) ;
    }

  // PolarConverter Curve must have same parameterization as Surface in SweepCurve direction
  const SmBSplineCurve *pPolarCurve = m_vPolarConverter.GetCurve() ;

  // when given an m_pNurb and a PolarCurve
  if(pPolarCurve)
    {
      /* 12 */ // PolarCurve InsideOut bit values must be FALSE
      bRtn &= SM_ASSERT_BOOLEAN_REPORT(12, SM_LEVEL_0, pPolarCurve->GetInsideOut() == FALSE, _T("")) ;

      if(m_pNurb)
        {
          // PolarConverter Curve must have same parameterization as Surface in SweepCurve direction
          // pPolarCurve AnalInterval   must equal Surface->AnalUVDomain->GetUInterval,
          // pPolarCurve NURBInterval   must equal m_bSwapUV ? Surface->NURBVInterval : Surface->NURBUInterval
          // pPolarCurve NURBKnotVector must equal m_bSwapUV ? Surface->NURBKnotV : Surface->NURBKnotU

          /*  7 */ // PolarCurve Anal and NURB intervals must match appropriate Surface Domain Intervals
          bRtn &= SM_ASSERT_VALUE_REPORT(7, SM_LEVEL_0,
                                         (HasSameParameterization(SM_SP_U,        // in : Analytic Domain Direction
                                                                    m_bSwapUV     // in : Nurb     Domain Direction
                                                                  ? SM_SP_V
                                                                  : SM_SP_U,
                                                                  pPolarCurve)),  // in : The GenCurve
                                         SM_EFF_ZERO, SM_UNDEF_DOUBLE, _T("") ) ;

          /*  8 */ // compare a Sweep near midPoint evaluation
          if(bRtn && m_vPolarConverter.IsPolarConversionPossible())
            {
              SmPoint2d sNurbUV, sStepUV = m_vAnalUVDomain.Evaluate(.4567, .7654) ;
              ConvertUVFromSTEPToNURBS(sStepUV, sNurbUV) ;
              SmPoint3d sStepPoint, sNurbPoint ;
              EvaluateSTEPPoint(sStepUV, sStepPoint) ;
              EvaluatePoint(sNurbUV, sNurbPoint) ;
              double dScaledZero = SM_EFF_ZERO * 10.0 * (1.0 + sStepPoint.GetMaxDimension()) ;
              double dDist       = sStepPoint.DistanceBetween(sNurbPoint) ;
              bRtn &= SM_ASSERT_VALUE_REPORT(8, SM_LEVEL_0, (dDist < dScaledZero), dScaledZero, dDist, _T("") ) ;

            }
        } // end m_pNurb existence check
    } // end pPolarCurve existence check

  // make sure that Nurb and genCurve are oriented correctly
  ULONG ii ;
  const ULONG lCount = 4 ;
  SmPoint3d  sSurfU0[lCount], sSurfV0[lCount] ;
  SmPoint3d  sCrvPoint[lCount] ;
  double dDistU0Max = 0.0, dDistV0Max = 0.0 ;
  if(m_pNurb && m_pGenCurve)
    {
      // locals
      SmExtent2d sUVDomain = GetNaturalUVDomain() ;
      SmExtent1d sIvl      = m_pGenCurve->GetNaturalInterval() ;
      double dCos          = smos_Cosine(m_vAnalUVDomain.GetMin().x * SM_PI/180.0) ;
      double dSin          = smos_Sine  (m_vAnalUVDomain.GetMin().x * SM_PI/180.0) ;
      double dR ;
      const SmVector3d &rO = m_vPosition.GetOriginRef() ;
      const SmVector3d &rX = m_vPosition.GetXAxisRef() ;
      const SmVector3d &rY = m_vPosition.GetYAxisRef() ;
      SmVector3d        sZ = m_vPosition.GetZAxis() ;

      // for every sample point
      for(ii=0;ii<lCount;ii++)
        {
          // get m_pCurve and Surf points
          double dParam =   ii == 0 ? 0.0
                          : ii == lCount-1 ? 1.0
                          : (double)ii/(double)(lCount-1) ;
          SmBSplineSurface::EvaluatePoint(sUVDomain.Evaluate(0.0,dParam), sSurfU0[ii]) ;
          SmBSplineSurface::EvaluatePoint(sUVDomain.Evaluate(dParam,0.0), sSurfV0[ii]) ;
          m_pGenCurve->SmBSplineCurve::EvaluatePoint(sIvl.Evaluate(dParam), sCrvPoint[ii]) ;

          // move curve point to Nurb Start position using step params
          smgu_LinePointDistance(rO, sZ, sCrvPoint[ii], dR) ;
          sCrvPoint[ii] = sCrvPoint[ii] -dR * rX + dR * (dCos * rX + dSin * rY) ;

          double dScaledZero = SM_EFF_ZERO * 100.0 * (1.0 + sCrvPoint[ii].GetMaxDimension()) ;
          double dDistU0 = (sCrvPoint[ii] - sSurfU0[ii]).Length() ;
          double dDistV0 = (sCrvPoint[ii] - sSurfV0[ii]).Length() ;
          if(dDistU0 > dDistU0Max) dDistU0Max = dDistU0 ;
          if(dDistV0 > dDistV0Max) dDistV0Max = dDistV0 ;

          if( m_bSwapUV )
            {
              /*  9 */ // A GenCurve eval does not map to Surface eval within dScaledZero
              bRtn &= SM_ASSERT_VALUE_REPORT(9, SM_LEVEL_0, (dDistV0 < dScaledZero), dScaledZero, dDistV0, _T("")) ;
            }
          else
            {
              /* 10 */ // A GenCurve eval does not map to Surface eval within dScaledZero
              bRtn &= SM_ASSERT_VALUE_REPORT(10, SM_LEVEL_0, (dDistU0 < dScaledZero), dScaledZero, dDistU0, _T("")) ;
            }

        } // end iter every sample point

    } // end SurfaceNurb/GenCurveNurb compatibility

  /* 13 */ // Analytic U Direction Domain too large: spheres can't overlap themselves
  SmExtent1d sIvlU = m_vAnalUVDomain.GetUInterval() ;
  bRtn &= SM_ASSERT_VALUE_REPORT(2, SM_LEVEL_0,
                                 (sIvlU.GetLength() <= 360.0 + SM_EFF_ZERO_DEG),
                                 SM_EFF_ZERO_DEG, sIvlU.GetLength() - 360.0, _T("")) ;

  /* 14 */ // Bad GenCurve Owner 
  bRtn &= SM_ASSERT_BOOLEAN_REPORT(14, SM_LEVEL_0, (m_pGenCurve->GetOwner() == this), _T("") ) ;

  /* 15 */ // Bad GenCurve length degenerates SurfOfRevolution to an approximate circular curve
  bRtn &= SM_ASSERT_BOOLEAN_REPORT(15, SM_LEVEL_0, (m_pGenCurve == NULL || SmTol::IsDegenerate(m_pGenCurve, m_pGenCurve->GetNaturalInterval() ) == FALSE), _T("") ) ;

    // gwc:note when GenCurve has a non-zero degenerate length, the surface will also fail 
    //          SmSurface::AssertValid Test 5: surface singularity length (which should be zero) is non-zero.
    //          This is a case where one bad model feature generates several Assert failures. 
    //          Reporting the degenerate gencurve problem is a good description of the problem.  
    //          Reporting that the surface approximates a non-zero singularity is a bad description
    //            of this problem and just serves to confuse interpretting the final set of SmAssertReports
    //            contained within the growing pAList of Assert errors.
    //          Worse yet, including the confusing non-zero singularity problem in the pAList might
    //          cause a heal function to run that moves all the control points to a common position
    //          creating more problems than it solves.
    //          Let's use this case to set up a mechanism to remove nested AssertReports that don't
    //          help describe the problem. 

  // when ThisObj fails degenerate GenCurve test 15 - rm cascading SmSurface::Test 5 reports
  //  of NonSingular Surface approximates a Singular Surface (ie. one non-zero boundary length is less than XSectTol3d)
  SM_ASSERT_REMOVE_CASCADING_REPORT(15, SmSurface, 5, this) ;

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  // draw
  if(bDebugMe)
    {
      Dump() ;
      m_pGenCurve->Dump() ;

      SmFace *pFace = (SmFace *)GetFace() ;
      SmBrep *pBrep = pFace ? pFace->GetBrep() : NULL ;

      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(2,3, 1,0,0) ; if(m_pGenCurve) m_pGenCurve->Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,4, 1,0,0) ; m_vPosition.Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,1) ; this->DrawUV(3,3) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,0,0) ; if(pFace) pFace->Draw(SM_DM_CROSSHATCH) ; sm_GraphicsLoop() ;
      for(ii=0;ii<lCount;ii++)
        { smgfx_SetLook(3,4, 1,0,0) ; sCrvPoint[ii].Draw() ; sm_GraphicsLoop() ;
          smgfx_SetLook(5,6, 0,1,0) ; sSurfU0[ii].Draw() ; sm_GraphicsLoop() ;
          smgfx_SetLook(7,8, 1,1,0) ; sSurfV0[ii].Draw() ; sm_GraphicsLoop() ;
        }
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

  // all done
  // SM_ASSERT(bRtn) ;
  return(bRtn) ;

} // end SmSurfOfRevolution::AssertValid

// obsolete
// /*******************************************************************//**
// PURPOSE: Fix Assert Report failures reported by an AssertValid call
// 
// RETURNS ---  TRUE  = OK
//              FALSE = Problem
// ***********************************************************************/
// SmBoolean SmSurfOfRevolution::AssertHeal
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
//                rAReport.m_pHealMessage = _T("SmSurfOfRevolution::AssertHeal fix not yet supported") ;
// 
//     }
// 
//   // all done
//   return(bRtn) ;
// 
// } // end SmSurfOfRevolution::AssertHeal
// end obsolete

/*******************************************************************//**
PURPOSE: Write SmSurfOfRevolution to given output stream.

NOTES:
***********************************************************************/
SmStatus SmSurfOfRevolution::WriteToDB
 (SmDatabaseIO & rDB,                // in : target output stream
  ULONG          lDBVersionNumber)   // in : database version to get proper sequence of writes
 const
{
  // file type, ASCII or BINARY
  SmFileType      eType    =  rDB.GetFileType();
  std::ostream  & rFileOut = *rDB.GetOutStreamPtr();

  // locals
  const SmPoint3d &rOrig = m_vPosition.GetOriginRef() ;
  const SmVector3d &rX   = m_vPosition.GetXAxisRef() ;
  const SmVector3d &rY   = m_vPosition.GetYAxisRef() ;

  if (eType == SM_ASCII)
    {
      rFileOut << m_bMakeNurbGenCurve                          << " SmSurfOfRevolution Make Nurb GenCurve Flag, FALSE=Unbounded analytic, TRUE=bounded \n" ;
      rFileOut << m_bPlanarGenerator                           << " SmSurfOfRevolution Planar Generator Curve \n" ;
      rFileOut << rOrig.x << " " << rOrig.y << " " << rOrig.z  << " SmSurfOfRevolution Placement Origin \n";
      rFileOut << rX.x    << " " << rX.y    << " " << rX.z     << " SmSurfOfRevolution Placement X Axis \n";
      rFileOut << rY.x    << " " << rY.y    << " " << rY.z     << " SmSurfOfRevolution Placement Y Axis \n";
      rFileOut        << m_vAnalUVDomain.GetMin().x
               << " " << m_vAnalUVDomain.GetMin().y
               << " " << m_vAnalUVDomain.GetMax().x
               << " " << m_vAnalUVDomain.GetMax().y            << " SmSurfOfRevolution AnalUVDomain \n";
      rFileOut << m_bSwapUV                                    << " SmSurfOfRevolution SwapUV Boolean \n";
    }
  else
    {
      SER(rDB.WriteBoolean(m_bMakeNurbGenCurve));
      SER(rDB.WriteBoolean(m_bPlanarGenerator));

      SER(rDB.WriteDouble(rOrig.x));
      SER(rDB.WriteDouble(rOrig.y));
      SER(rDB.WriteDouble(rOrig.z));

      SER(rDB.WriteDouble(rX.x));
      SER(rDB.WriteDouble(rX.y));
      SER(rDB.WriteDouble(rX.z));

      SER(rDB.WriteDouble(rY.x));
      SER(rDB.WriteDouble(rY.y));
      SER(rDB.WriteDouble(rY.z));

      SER(rDB.WriteDouble(m_vAnalUVDomain.GetMin().x));
      SER(rDB.WriteDouble(m_vAnalUVDomain.GetMin().y));
      SER(rDB.WriteDouble(m_vAnalUVDomain.GetMax().x));
      SER(rDB.WriteDouble(m_vAnalUVDomain.GetMax().y));

      SER(rDB.WriteBoolean(m_bSwapUV));
    }

  // GenCurve
  ULONG lGenCurveDim = m_pGenCurve->GetDim() ;
  if (eType == SM_ASCII) { rFileOut << " SmSurfOfRevolution->GenCurve \n"; }
  SER(rDB.WriteType(m_pGenCurve->GetType(), &lGenCurveDim)) ;
  SER(m_pGenCurve->WriteToDB(rDB, lDBVersionNumber)) ;

  // PolarConverter
  if (eType == SM_ASCII) { rFileOut << " SmSurfOfRevolution->PolarConverter \n"; }
  SER(m_vPolarConverter.WriteToDB(rDB, lDBVersionNumber)) ;

  // output the parent
  SmBSplineSurface::WriteToDB(rDB, lDBVersionNumber) ;

  // all done
  return SM_SUCCESS;

} // end SmSurfOfRevolution::WriteToDB

/*******************************************************************//**
PURPOSE: static method to Read a SmSurfOfRevolution from a given stream

NOTES:
***********************************************************************/
SmStatus SmSurfOfRevolution::ReadFromDB
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
         || rpNewSurface->IsKindOf(SmSurfOfRevolution_TYPE)) ? SM_SUCCESS : SM_ERR) ;

  // init output object
  SmSurfOfRevolution *pSurfOfRevolution =   (rpNewSurface == NULL)
                                          ? new (crContext) SmSurfOfRevolution()
                                          : (SmSurfOfRevolution *)rpNewSurface ;

  // file type
  SmFileType     eType   =  rDB.GetFileType();
  std::istream & rFileIn = *rDB.GetInStreamPtr();

  // locals
  SmBoolean              bMakeNurbGenCurve ;
  SmBoolean              bPlanarGenerator ;
  SmPoint3d              sOrig ;
  SmVector3d             sX, sY ;
  SmPoint2d              sAnalMin, sAnalMax ;
  SmBoolean              bSwapUV ;

  if (eType == SM_ASCII)
    {
      rFileIn >> bMakeNurbGenCurve ;                                      rDB.GoToNextLine() ;
      rFileIn >> bPlanarGenerator ;                                       rDB.GoToNextLine() ;

      rFileIn >> sOrig.x >> sOrig.y >> sOrig.z ;                          rDB.GoToNextLine() ;
      rFileIn >> sX.x >> sX.y >> sX.z ;                                   rDB.GoToNextLine() ;
      rFileIn >> sY.x >> sY.y >> sY.z ;                                   rDB.GoToNextLine() ;

      rFileIn >> sAnalMin.x >> sAnalMin.y >> sAnalMax.x >> sAnalMax.y  ;  rDB.GoToNextLine() ;

      rFileIn >> bSwapUV ;                                                rDB.GoToNextLine() ;
    }
  else
    {
      SER(rDB.ReadBoolean(bMakeNurbGenCurve)) ;
      SER(rDB.ReadBoolean(bPlanarGenerator )) ;

      SER(rDB.ReadDouble(sOrig.x)) ;
      SER(rDB.ReadDouble(sOrig.y)) ;
      SER(rDB.ReadDouble(sOrig.z)) ;
      SER(rDB.ReadDouble(sX.x)) ;
      SER(rDB.ReadDouble(sX.y)) ;
      SER(rDB.ReadDouble(sX.z)) ;
      SER(rDB.ReadDouble(sY.x)) ;
      SER(rDB.ReadDouble(sY.y)) ;
      SER(rDB.ReadDouble(sY.z)) ;

      SER(rDB.ReadDouble(sAnalMin.x)) ;
      SER(rDB.ReadDouble(sAnalMin.y)) ;
      SER(rDB.ReadDouble(sAnalMax.x)) ;
      SER(rDB.ReadDouble(sAnalMax.y)) ;

      SER(rDB.ReadBoolean(bSwapUV)) ;
    }

  // curve locals
  SmCurve *pGenCurve=NULL ;
  SM_TYPE  lGenCurveType ;
  ULONG    lGenCurveDim ;

  // GenCurve
  if (eType == SM_ASCII) { rDB.GoToNextLine() ; }
  SER(rDB.ReadType(lGenCurveType, &lGenCurveDim)) ;
  SER(SmCurve::ReadFromDB(lGenCurveType, rDB, lGenCurveDim, crContext, pGenCurve, lDBVersionNumber)) ;

  // PolarConverter - read directly into empty analytic
  if (eType == SM_ASCII) { rDB.GoToNextLine() ; }
  SmPolarConversion *pPolarConverter = &pSurfOfRevolution->m_vPolarConverter ;
  SER(SmPolarConversion::ReadFromDB(rDB, crContext, NULL, pPolarConverter, lDBVersionNumber)) ;

  // load values
  pSurfOfRevolution->m_pGenCurve         = (SmBSplineCurve *)pGenCurve ;
  pSurfOfRevolution->m_pGenCurve->SetOwner(pSurfOfRevolution) ;
  pSurfOfRevolution->m_bMakeNurbGenCurve = bMakeNurbGenCurve ;
  pSurfOfRevolution->m_bPlanarGenerator  = bPlanarGenerator ;
  pSurfOfRevolution->m_vPosition.SetCanonical(sOrig, sX, sY) ;
  pSurfOfRevolution->m_vAnalUVDomain.SetMinMax(sAnalMin, sAnalMax) ;
  pSurfOfRevolution->m_bSwapUV           = bSwapUV ;

  // read the parent object
  SmSurface *pSurface = pSurfOfRevolution ;
  SER(SmBSplineSurface::ReadFromDB(SmBSplineSurface_TYPE, rDB, crContext, pSurface, lDBVersionNumber)) ;

  // all done
  rpNewSurface = pSurfOfRevolution ;
  return SM_SUCCESS;

} // end SmSurfOfRevolution::ReadFromDB

/*******************************************************************//**
PURPOSE:
NOTES:
***********************************************************************/
SmBoolean SmSurfOfRevolution::IsKindOf( SM_TYPE t ) const
{
  return ((SmSurfOfRevolution_TYPE == t) ? TRUE : SmBSplineSurface::IsKindOf( (t) ));
}

/*******************************************************************//**
PURPOSE: Dump SurfOfRevolution surface data out for debugging.

NOTES:
***********************************************************************/
void SmSurfOfRevolution::Dump
  (void)
 const
{
  TCHAR sBuff[SM_TBLOCK_SIZE], sBuffForFile[SM_TBLOCK_SIZE];
  TCHAR sGenBuff[SM_TBLOCK_SIZE] ;
  smos_WriteBuffer(_T("\nBegin SmSurfOfRevolution::Dump()")) ;

  // GenCurve type/insideOut value label for header info
  if     (    m_pGenCurve == NULL)                   { smos_sprintf(sGenBuff, _T("%s"),_T("No GenCurve")) ; }
  else if(   !m_pGenCurve->IsAnalytic())             { smos_sprintf(sGenBuff, _T("%s GenCurve Not Analytic"), m_pGenCurve->GetTypeString()) ; }
  else if(   !m_pGenCurve->IsKindOf(SmEllipse_TYPE)
          && !m_pGenCurve->IsKindOf(SmLine_TYPE))    { smos_sprintf(sGenBuff, _T("%s GenCurve has no m_bInsideOut Value"), m_pGenCurve->GetTypeString()) ; }
  else if(    m_pGenCurve->IsKindOf(SmEllipse_TYPE)) { SmEllipse *pEllipse = (SmEllipse *)m_pGenCurve ;
                                                       smos_sprintf(sGenBuff, _T("%s GenCurve m_bInsideOut:[%s]"),
                                                                             m_pGenCurve->GetTypeString(),
                                                                             pEllipse->GetInsideOut() ? _T("TRUE") : _T("FALSE")) ;
                                                     }
  else if(    m_pGenCurve->IsKindOf(SmLine_TYPE))    { SmLine *pLine = (SmLine *)m_pGenCurve ;
                                                       smos_sprintf(sGenBuff, _T("%s GenCurve m_bInsideOut:[%s]"),
                                                                           m_pGenCurve->GetTypeString(),
                                                                           pLine->GetInsideOut() ? _T("TRUE") : _T("FALSE")) ;
                                                     }
  // Domain locals (analytic, Nurb, GenCurve Analytic and GenCurve Nurb)
  SmExtent2d sAnalyticDomain,     sNaturalUVDomain ;
  SmExtent1d sNaturalGenMatchIvl, sNaturalPolarMatchIvl ;
  SmExtent1d sAnalGenMatchIvl,    sAnalPolarMatchIvl ;

  SmExtent1d sGenAnalIvl,   sGenNaturalIvl ;
  SmExtent1d sPolarAnalIvl, sPolarNaturalIvl ;

  SmTArray<double> sNaturalGenMatchKnots, sGenKnots ;
  SmTArray<ULONG>  sNaturalGenMatchMults, sGenMults ;
  SmTArray<double> sNaturalPolarMatchKnots, sPolarKnots ;
  SmTArray<ULONG>  sNaturalPolarMatchMults, sPolarMults ;

  // surface analytic domain and matching intervals
  sAnalyticDomain          =  m_vAnalUVDomain ;
  sAnalGenMatchIvl         =  m_vAnalUVDomain.GetVInterval() ;
  sAnalPolarMatchIvl       =  m_vAnalUVDomain.GetUInterval() ;

  // characterize surface: bHasNurb, bHasGenCurve, bHasPolarCurve
  SmBoolean bHasNurb       = (GetGwNurbPointer()           != NULL) ;
  SmBoolean bHasGenCurve   = (GetGenCurve()                != NULL) ;
  SmBoolean bHasPolarCurve = (m_vPolarConverter.GetCurve() != NULL) ;
  SmBoolean bSameGenKnots = FALSE;
  // SmBoolean bSamePolarKnots ;

  // surface natural domain and matching intervals
  if(bHasNurb) { sNaturalUVDomain      = GetNaturalUVDomain() ;
                 sNaturalGenMatchIvl   =   m_bSwapUV
                                         ? sNaturalUVDomain.GetUInterval()
                                         : sNaturalUVDomain.GetVInterval() ;
                 sNaturalPolarMatchIvl =   m_bSwapUV
                                         ? sNaturalUVDomain.GetVInterval()
                                         : sNaturalUVDomain.GetUInterval() ;
                 if(m_bSwapUV) { GetKnots(SM_SP_U, sNaturalGenMatchKnots,   &sNaturalGenMatchMults) ;
                                 GetKnots(SM_SP_V, sNaturalPolarMatchKnots, &sNaturalPolarMatchMults) ;
                               }
                 else          { GetKnots(SM_SP_U, sNaturalPolarMatchKnots, &sNaturalPolarMatchMults) ;
                                 GetKnots(SM_SP_V, sNaturalGenMatchKnots,   &sNaturalGenMatchMults) ;
                               }
               }

  // genCurve analytic and natural intervals
  if(bHasGenCurve) { sGenAnalIvl    = GetGenCurve()->GetSTEPInterval() ;
                     sGenNaturalIvl = GetGenCurve()->GetNaturalInterval() ;
                     GetGenCurve()->GetKnots(sGenKnots, &sGenMults) ;
                     bSameGenKnots =    SM_ARE_ARRAYS_SAME(sGenKnots, sNaturalGenMatchKnots)
                                     && SM_ARE_ARRAYS_SAME(sGenMults, sNaturalGenMatchMults) ;
                   }

  // polarCurve analytic and natural intervals
  if(bHasPolarCurve) { sPolarAnalIvl    = m_vPolarConverter.GetCurve()->GetSTEPInterval() ;
                       sPolarNaturalIvl = m_vPolarConverter.GetCurve()->GetNaturalInterval() ;
                       m_vPolarConverter.GetCurve()->GetKnots(sPolarKnots, &sPolarMults) ;
                       // bSamePolarKnots  = SM_ARE_ARRAYS_SAME(sPolarKnots, sNaturalPolarMatchKnots) && SM_ARE_ARRAYS_SAME(sPolarMults, sNaturalPolarMatchMults) ;
                     }

  // report Cache data
  SmSurface::Dump(FALSE) ;

  smos_sprintf(sBuff,       _T("\nSmSurfOfRevolution = 0x%p, m_bPlanarGenerator:[%s], m_bSwapUV:[%s]"),
             this,
             m_bPlanarGenerator ? _T("TRUE") : _T("FALSE"),
             m_bSwapUV          ? _T("TRUE") : _T("FALSE")) ;
  smos_sprintf(sBuffForFile,_T("\nSmSurfOfRevolution = %s, m_bPlanarGenerator[%s], m_bSwapUV:[%s]"),
             _T("notNULL"),
             m_bPlanarGenerator ? _T("TRUE") : _T("FALSE"),
             m_bSwapUV          ? _T("TRUE") : _T("FALSE")) ;
  smos_WriteBuffer(sBuff, sBuffForFile);

  // domains
  smos_WriteBuffer(_T("\n  SmSurfOfRevolution Domains")) ;
  smos_WriteBuffer(_T("\n    m_vAnalUVDomain: ")) ; m_vAnalUVDomain.Dump() ;
  if(bHasNurb)       { smos_WriteBuffer(_T("    m_pNurb Domain : ")) ; sNaturalUVDomain.Dump() ; }
  else               { smos_WriteBuffer(_T("    m_pNurb Domain : Undefined - No underlying m_pNurb Surface\n")) ; }
  if(bHasGenCurve)   { smos_WriteBuffer(_T("    GenCurve Anal  : ")) ; sGenAnalIvl.Dump() ; }
  else               { smos_WriteBuffer(_T("    GenCurve Anal  : Undefined - No underlying m_pGenCurve Object")) ; }
  if(bHasGenCurve)   { smos_WriteBuffer(_T("\n    GenCurve Nurb  : ")) ; sGenNaturalIvl.Dump() ; }
  else               { smos_WriteBuffer(_T("\n    GenCurve Nurb  : Undefined - No underlying m_pGenCurve Object")) ; }
  if(bHasPolarCurve) { smos_WriteBuffer(_T("\n    PolarCrv Anal  : ")) ; sPolarAnalIvl.Dump() ; }
  else               { smos_WriteBuffer(_T("\n    PolarCrv Anal  : Undefined - No underlying m_vPolarConverter.m_cpCurve Object")) ; }
  if(bHasPolarCurve) { smos_WriteBuffer(_T("\n    PolarCrv Nurb  : ")) ; sPolarNaturalIvl.Dump() ; }
  else               { smos_WriteBuffer(_T("\n    PolarCrv Nurb  : Undefined - No underlying m_vPolarConverter.m_cpCurve Object")) ; }

  // domain rule compliance
  smos_WriteBuffer(_T("\n    6 SmSurfOfRevolution Domain compatibility rules:")) ;

  // The GenCurve parameterization must match the Surface Domain Intervals as
  //    1. GenCurve AnalInterval == Surface AnalVInterval
  //    2. GenCurve NURBInterval == m_bSwapUV ? Surface->NURBUInterval : Surface->NURBVInterval
  //    3. GenCurve KnotVector   == m_bSwapUV ? Surface->NURBUKnot : Surface->NURBVKnot

  smos_WriteBuffer(_T("\n      Rule 1. GenCurve->AnalInterval  == AnalUVDomain.GetVInterval")) ;
  if(bHasGenCurve) { smos_WriteBuffer(_T("\n        GenCurve Anal  : ")) ; sGenAnalIvl.Dump() ; }
  else             { smos_WriteBuffer(_T("\n        GenCurve Anal  : Undefined - No underlying m_pGenCurve Object")) ;
                   }
                     smos_WriteBuffer(_T("\n        SurfAnal match : ")) ; sAnalGenMatchIvl.Dump() ;

  smos_WriteBuffer(_T("\n      Rule 2. GenCurve->NurbInterval  == m_bSwapUV ? sNaturalUVDomain.GetUInterval : sNaturalUVDomain.GetVInterval")) ;
  if(bHasGenCurve) { smos_WriteBuffer(_T("\n        GenCurve Nurb  : ")) ; sGenNaturalIvl.Dump() ; }
  else             { smos_WriteBuffer(_T("\n        GenCurve Nurb  : Undefined - No underlying m_pGenCurve Object")) ; }
  if(bHasNurb)     { smos_WriteBuffer(_T("\n        SurfNurb Match : ")) ; sNaturalGenMatchIvl.Dump() ; }
  else             { smos_WriteBuffer(_T("\n        SurfNurb Match : Undefined - No underlying m_pNurb Surface\n")) ; }

  smos_WriteBuffer(_T("\n      Rule 3. GenCurve NURBKnotVector == m_bSwapUV ? Surface->NURBKnotU : Surface->NURBKnotV")) ;
  if(bHasGenCurve && bHasNurb)
    { smos_sprintf(sBuff,_T("\n        This GenCurve NURBKnotVector %s - it %s == (m_bSwapUV ? Surface->NURBKnotU : Surface->NURBKnotV)"),
                       bSameGenKnots ? _T("is OKAY") : _T("has ERRORs") ,
                       bSameGenKnots ? _T("DOES") : _T("DOES NOT")) ;
      smos_WriteBuffer(sBuff);
    }
  smos_WriteBuffer(_T("\n        See knot vector listings below")) ;

  // When PolarConverter has a PolarCurve, the PolarCurve parameterization must match the Surface Domain Intervals as
  //    4. PolarCurve AnalInterval == Surface AnalUInterval
  //    5. PolarCurve NURBInterval == m_bSwapUV ? Surface->NURBVInterval : Surface->NURBUInterval
  //    6. PolarCurve KnotVector   == m_bSwapUV ? Surface->NURBVKnot : Surface->NURBUKnot

  smos_WriteBuffer(_T("\n      Rule 4. PolarCurve->AnalInterval  == AnalUVDomain.GetUInterval")) ;
  if(bHasGenCurve) { smos_WriteBuffer(_T("\n        PolarCrv Anal  : ")) ; sPolarAnalIvl.Dump() ; }
  else             { smos_WriteBuffer(_T("\n        PolarCrv Anal  : Undefined - No underlying m_pGenCurve Object")) ;
                   }
                     smos_WriteBuffer(_T("\n        SurfAnal match : ")) ; sAnalPolarMatchIvl.Dump() ;

  smos_WriteBuffer(_T("\n      Rule 5. PolarCurve->NurbInterval  == m_bSwapUV ? sNaturalUVDomain.GetVInterval : sNaturalUVDomain.GetUInterval")) ;
  if(bHasPolarCurve) { smos_WriteBuffer(_T("\n        PolarCrv Nurb  : ")) ; sPolarNaturalIvl.Dump() ; }
  else               { smos_WriteBuffer(_T("\n        PolarCrv Nurb  : Undefined - No underlying m_vPolarConverter.m_cpCurve Object")) ; }
  if(bHasNurb)       { smos_WriteBuffer(_T("\n        SurfNurb Match : ")) ; sNaturalPolarMatchIvl.Dump() ; }
  else               { smos_WriteBuffer(_T("\n        SurfNurb Match : Undefined - No underlying m_pNurb Surface\n")) ; }

  smos_WriteBuffer(_T("\n      Rule 6. PolarCurve NURBKnotVector == m_bSwapUV ? Surface->NURBKnotV : Surface->NURBKnotU")) ;
  if(bHasPolarCurve && bHasNurb)
    { smos_sprintf(sBuff,_T("\n        This GenCurve NURBKnotVector %s - it %s == (m_bSwapUV ? Surface->NURBKnotU : Surface->NURBKnotV)"),
                       bSameGenKnots ? _T("is OKAY") : _T("has ERRORs") ,
                       bSameGenKnots ? _T("DOES") : _T("DOES NOT")) ;
      smos_WriteBuffer(sBuff);
    }
  smos_WriteBuffer(_T("\n        See knot vector listings below")) ;

  //
  smos_WriteBuffer(_T("\n\n  SmSurfOfRevolution Transform       = ")) ;  m_vPosition.Dump();
  if(m_vPolarConverter.IsPolarConversionPossible())
    { smos_WriteBuffer(_T("  SmSurfOfRevolution Polar Converter - ")) ;  m_vPolarConverter.Dump(); }
  else
    { smos_WriteBuffer(_T("  SmSurfOfRevolution Polar Converter = Not Initialized\n")) ; }

  if(m_pGenCurve) { smos_WriteBuffer(_T("\nSmSurfOfRevolution m_pGenCurve - ")) ;  m_pGenCurve->Dump(); }
  else            { smos_WriteBuffer(_T("\nNo SmSurfOfRevolution GenCurve\n")) ; }
  if(m_pNurb)     { smos_WriteBuffer(_T("\nSmSurfOfRevolution m_pNurb - ")) ;  SmBSplineSurface::Dump(); }
  else            { smos_WriteBuffer(_T("\nNo SmSurfOfRevolution Underlying Nurb Surface\n")) ; }

  smos_WriteBuffer(_T(" End SmSurfOfRevolution::Dump()\n")) ;

} // end SmSurfOfRevolution::Dump
