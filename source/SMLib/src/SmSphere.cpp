// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************//**
* FILE NAME --- SmSphere.cpp
* PURPOSE: Implementation of SmSphere methods.
**********************************************************************/

#include "StdAfx.h"

#include <SmSphere.h>
#include <SmPlane.h>
#include <SmEllipse.h>
#include <SmCone.h>
#include <SmCircle.h>
#include <nurbs.h>
#include <SmNurbsSrf.h>
#include <SmGeomUtility.h>
#include <SmGraphicsExtern.h>
#include <SmExtent3d.h>
#include <SmAssertArray.h>
#include <SmDatabaseIO.h>

#ifdef SM_DEBUG_CODE
#include <SmBrep.h>
#include <SmFace.h>
#endif
/*******************************************************************//**
PURPOSE: Create a Sphere given
    (1) origin,
    (2) local X,Y axes,
    (3) angular parameter range and
    (4) radius

    These allocate the underlying Nurb surface as needed.

NOTES: This Sphere should be used when surface operations (like
    SS-Intersection) may take advantage of its analytic properties.
***********************************************************************/
SmSphere::SmSphere
 (const SmPoint3d  & crOrigin,       // in : Sphere origin
  const SmVector3d & crXAxis,        // in : start direction of rotational angle [0 to 360]
  const SmVector3d & crYAxis,        // in : defines Origin Z as XAxis cross YAxis
  const SmExtent2d & crAnalUVDomain, // in : range or allowed parameters
                                     //      input as [-360 to 360 MaxLength=360, -90 to 90]
  double             dRadius,        // in : distance from sphere origin to its surface
  SmBoolean          bSwapUV,        // in : TRUE = Underlying n_pNurb u and v directions are swapped from AnalUVDomain
                                     //      FALSE= Underlying n_pNurb u and v direction are same as AnalUVDomain directions
  SmBoolean          bInsideOut,     // in : TRUE = GenCurve runs from +90 pole to -90 pole
                                     //      FALSE= GenCurve runs from -90 pole to +90 pole
  SmBSplineCurve   * pOptCircleNurb, // in : Optional Curve Pointer to define GenCurve parameterization
                                     //      SurfOfRevolution owns this curve and will delete it when destructed.
  const SmContext  * cpContext)      // in : Set context if given, default:[NULL] 
: SmSurfOfRevolution(pOptCircleNurb,crOrigin,crXAxis,crYAxis,crAnalUVDomain,bSwapUV),
  m_dRadius(dRadius),
  m_bInsideOut(bInsideOut)
{
  SM_ASSERT(dRadius > SM_EFF_ZERO) ;

  // old flag when underlying Nurb parameterizations were not set by constructors
  m_bMakeNurbGenCurve = FALSE;

  // build and store generator curve when needed
  if(m_pGenCurve == NULL)
    {
      const SmContext* pContext  = GetContext();
      SmCircle * pNewCircle = NULL ;
      SmExtent1d sVint = crAnalUVDomain.GetVInterval();
      SmCircle::CreateCanonical(*pContext,
                                SmAxis2Placement(crOrigin, crXAxis,crXAxis * crYAxis),
                                dRadius,
                                pNewCircle,
                                &sVint,
                                &bInsideOut) ;
      SM_ASSERT(pNewCircle != NULL) ;
      m_pGenCurve = pNewCircle;
      m_pGenCurve->SetOwner(this) ;
    }

  // set context if given
  if(cpContext) 
    { m_cpContext = cpContext ; }

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  if(bDebugMe)
    {
      Dump() ;
      SM_DUMP_AND_ASSERT_VALID(this) ;
    }
#endif // SM_DEBUG_CODE

  SM_ASSERT(m_pGenCurve != NULL) ;

} // end SmSphere::SmSphere constructor

/*******************************************************************//**
PURPOSE: Copy constructor for SmSphere

NOTES:
***********************************************************************/
SmSphere::SmSphere(const SmSphere & crSource)
 : SmSurfOfRevolution(crSource),
   m_dRadius(crSource.m_dRadius),
   m_bInsideOut(crSource.m_bInsideOut)
{

} // end SmSphere::SmSphere copy constructor

/*******************************************************************//**
PURPOSE: Equality operator for SmSphere

NOTES: Call base equivalence to check type and then check
       members for equivalence
***********************************************************************/
SmBoolean SmSphere::operator==
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
      SmSphere &rOther = (SmSphere &)crOther ;

      // check equivalence of these objects
      bRtn =  (   SM_IS_ZERO(m_dRadius - rOther.m_dRadius)
               && m_bInsideOut == rOther.m_bInsideOut) ;
    }

  // all done
  return bRtn ;

} // end SmSphere::operator==

/*******************************************************************//**
PURPOSE: Copy a Sphere

NOTES:
***********************************************************************/
SmStatus SmSphere::Copy
  (const SmContext & crContext,
   SmSurface *& rpNewSurface)
  const
{
  SmSphere *pCopy = new(crContext) SmSphere(*this); NER(pCopy);
  rpNewSurface = pCopy;
  return SM_SUCCESS;

} // end SmSphere::Copy

/*******************************************************************//**
PURPOSE: Method to create a canonical Closed Sphere object.

NOTES:
  Builds a Sphere with given orientation, radius, a Surface normal
  pointing away from the sphere's origin, and assigns it an AnalUVDomain as:
     U = CCW rotation in degrees from XAxis about ZAxis,  range:[ 0 to 360]
     V = Angle in degrees from bottom pole to top pole,   range:[-90 to 90]

  GenCurve is an SmCircle object because m_bMakeNurbGenCurve is set to FALSE.
***********************************************************************/
SmStatus SmSphere::CreateCanonical
  (const SmContext        & crContext,    // in : new object context
   const SmAxis2Placement & crOrigin,     // in : sphere origin coordinate system
   double                   dRadius,      // in : distance from originPoint to sphere surface
   SmSphere              *& rpNewSphere)  // out: new object
{
  // Degenerate-radius handling (kept parallel with SmTorus::CreateCanonical; a sphere has a single
  // radius, so its sign can carry the inside-out orientation):
  //   - negative radius  -> valid "inside-out" sphere (reversed normal/parameterization): build it as
  //     a positive-radius sphere with bInsideOut set, rather than reject it.
  //   - near-zero radius -> genuinely degenerate (no valid NURB): reject cleanly here instead of
  //     crashing on the undefined geometry downstream (the ctor SM_ASSERT is a no-op in release).
  const SmBoolean bInsideOut = (dRadius < 0.0) ? TRUE : FALSE;
  const double    dAbsRadius = fabs(dRadius);
  if (!(dAbsRadius > SM_EFF_ZERO))
    {
      rpNewSphere = NULL;
      SE_MSG(SM_ERR, _T("SmSphere::CreateCanonical: degenerate (near-zero) radius; cannot create sphere")) ;
      return SM_ERR;
    }

  // set analytic domain = [0 to 360, -90 to 90]
  SmExtent2d sAnalUVDomain(SmPoint2d(  0.0,-90.0),
                           SmPoint2d(360.0, 90.0));

  // construct step parameterized sphere
  //   U = CCW rotation in degrees from XAxis about ZAxis,range:[  0 to 360]
  //   V = Angle in degrees from bottom pole to top pole, range:[-90 to  90]
  rpNewSphere = new (crContext) SmSphere
                 (crOrigin.GetOriginRef(),
                  crOrigin.GetXAxisRef(),
                  crOrigin.GetYAxisRef(),
                  sAnalUVDomain,
                  dAbsRadius,
                  FALSE,       // bSwapUV
                  bInsideOut,  // negative radius -> inside-out sphere
                  NULL);       // pOptCircleNurb
  NER(rpNewSphere);

  // Build NURB; fail cleanly rather than leak a null-NURB object.
  if (rpNewSphere->MakeNurb() != SM_SUCCESS || rpNewSphere->GetGwNurbPointer() == NULL)
    {
      delete rpNewSphere; rpNewSphere = NULL;
      SE_MSG(SM_ERR, _T("SmSphere::CreateCanonical: failed to build NURB representation")) ;
      return SM_ERR;
    }

  // all done
  SM_DUMP_AND_ASSERT2_VALID(rpNewSphere) ;
  return SM_SUCCESS;

} // end SmSphere::CreateCanonical

/*******************************************************************//**
PURPOSE: Method to get canonical data from Cone object.

NOTES:
***********************************************************************/
SmStatus SmSphere::GetCanonical
  (SmAxis2Placement & rOrigin,         // out: Sphere coordinate system
   double           & rdRadius,        // out: distance from originPoint to Surface
   SmBoolean        * pOptSwapUV,      // out: TRUE = Nurb and Analytic U and V directions are swapped
                                       //      NULL to ignore, default:[NULL]
   SmBoolean        * pOptInsideOut)   // out: TRUE = Nurb and Analytic genCurve directions are swapped
                                       //      NULL to ignore, default:[NULL]
  const
{
  // set output
  rOrigin  = m_vPosition;
  rdRadius = m_dRadius;
  if(pOptSwapUV)     { *pOptSwapUV    = m_bSwapUV ; }
  if(pOptInsideOut)  { *pOptInsideOut = m_bInsideOut ; }

  // all done
  return SM_SUCCESS;

} // end SmSphere::GetCanonical

/*******************************************************************//**
PURPOSE: Create an offset sphere for the sphere.

NOTES:
 1.) The offset direction is in the direction of surfaes NurbBspline
      Surface direction
 2.) No sphere is built when new radius is less than equal to zero.
 3.) Offset Surface domains == Input Surface domains

NOTES: The offset Nurb and Step domains map from old to new
     so that   pOffSet->EvaluateSTEPPoint(StepUV) = pInput->EvaluateSTEPPoint() + dOff * dNormal(StepUV)
    and        pOffSet->EvaluatePoint(NurbUV)     = pInput->EvaluatePoint()     + dOff * dNormal(NurbUV)
***********************************************************************/
SmStatus SmSphere::CreateOffsetSurface
(
  const SmContext      & crContext,             // in : context for new obj construction
  double                 dSignedOffsetDistance, // in : offset dist, (neg val = Offset dir opposite surface normal)
  SmApproxTol3d          dThisApproxTol3d,      // in : Max Dist between ApproxOffsetSurface and ideal offset shape
  SmSurface* &           rOffsetSurface         // out: Offset Surf Approx, may be more than 1 when offsets have self-intersections
) const
{
  SM_DUMP_AND_ASSERT2_VALID(this) ;

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  // draw
  if(bDebugMe)
    {
      Dump() ;
      SmFace *pFace = (SmFace *)GetFace() ;
      SmBrep *pBrep = pFace ? pFace->GetBrep() : NULL ;

      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,1) ; DrawUV(7,7) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,0,0) ; if(pFace) pFace->Draw(SM_DM_CROSSHATCH) ; sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

  // init output container
  rOffsetSurface = NULL;

  // When UVs are swapped - reverse orientation of offset
  // When InsideOut       - reverse orientation of offset
  double dOffsetSign =   1.0
                       * (m_bSwapUV    ? -1.0 : 1.0)
                       * (m_bInsideOut ? -1.0 : 1.0) ;

  // get new radius
  double dOldRadius = m_dRadius ;
  double dNewRadius = m_dRadius + dOffsetSign * dSignedOffsetDistance;

#ifdef SM_DEBUG_CODE
  // check that dSignedOffsetDistance is measured from BSpline Surface Normal Direction
  if(m_bMakeNurbGenCurve == FALSE)
    {
      // a point and normal
      SmPoint3d sPnt, sNorm;
      SmExtent2d sDomain = GetNaturalUVDomain() ;
      SmPoint2d  sUV     = sDomain.Evaluate(0.0,0.0) ;
      SER(EvaluatePoint(sUV,sPnt));
      SER(EvaluateNormal(sUV,TRUE,TRUE,sNorm));

      // dDir is positive when Bspline surfNormal points away from origin
      // dStepDir should be pos when radius increases and negative when radius decreases
      double dDir     = sNorm.Dot(sPnt-m_vPosition.GetOriginRef()) ;
      double dStepDir = dDir * dSignedOffsetDistance ;
      // verify offset distance is with repsect to BSpline Surface NormalVector
      SM_ASSERT(   dNewRadius == dOldRadius
                || dStepDir * (dNewRadius - dOldRadius) > 0.0) ;
    }
#endif // SM_DEBUG_CODE

  // skip building degenerate or negative radius surfaces
  if(dNewRadius < dThisApproxTol3d)
    { return SM_SUCCESS; }

  // copy the current sphere
  SmSphere *pOffSphere = new (crContext) SmSphere(*this);
  SM_ASSERT(pOffSphere->m_pNurb     != NULL) ;
  SM_ASSERT(pOffSphere->m_pGenCurve != NULL) ;
  SmObjDelete sClean(pOffSphere) ;

  // change the radius
  pOffSphere->m_dRadius = dNewRadius ;

  // scale the NewSphere->GenCurve about current origin
  if(pOffSphere->m_pGenCurve)
    {
      double           dScale = dNewRadius/dOldRadius ;
      SmVector3d       sMoveVec((1.0 - dScale) * m_vPosition.GetOriginRef()) ;
      SmVector3d       sScale(dScale, dScale, dScale) ;
      SmAxis2Placement sMove ;
      sMove.Translate(sMoveVec) ;
      pOffSphere->m_pGenCurve->Transform(sMove, &sScale) ;
    }

  // rebuild the underlying Nurb Surface
  //   Just need to move the Nurb Points of the
  //   m_pNurb. GenCurve has already been done

  // for every m_pNurb Control Point
  gw_CPOINT        sEuclid ;
  gw_SURFACE      *pOffNurb = pOffSphere->GetOrCreateGwNurbPointer() ;
  const SmPoint3d &rCenter  = m_vPosition.GetOriginRef() ;
  int ii, jj ;
  for(ii=0;ii<=pOffNurb->net->n;ii++)
    {
      for(jj=0;jj<=pOffNurb->net->m;jj++)
        {
          // get Control Point Euclidean position
          gw_CPOINT *pCP = &(pOffNurb->net->Pw[ii][jj]) ;
          TO_EUCLID(*pCP, sEuclid) ;

          // move the ControlPoint
          SmVector3d sVec(sEuclid.x - rCenter.x,
                          sEuclid.y - rCenter.y,
                          sEuclid.z - rCenter.z) ;
          double     dScale  = dOldRadius/dNewRadius ;
          pCP->x = (rCenter.x + sVec.x/dScale) * pCP->w ;
          pCP->y = (rCenter.y + sVec.y/dScale) * pCP->w ;
          pCP->z = (rCenter.z + sVec.z/dScale) * pCP->w ;

        } // end iter every jjth ControlPoint
    } // end iter every iith ControlPoint

  // nothing to do for polar converter

  // old style - resets the nurb parameterization
//        pOffSphere->MakeNurb() ;

  // set output
  sClean.Clear() ;
  rOffsetSurface = pOffSphere ;

#ifdef SM_DEBUG_CODE
  if (bDebugMe)
    {
      Dump() ;
      pOffSphere->Dump() ;
      if(m_pNurb)
        {
          SmExtent2d sInputNurbDomain  = GetNaturalUVDomain() ;
          SmExtent2d sInputStepDomain  = GetSTEPUVDomain() ;
          SmExtent2d sOffsetNurbDomain = pOffSphere->GetNaturalUVDomain() ;
          SmExtent2d sOffsetStepDomain = pOffSphere->GetSTEPUVDomain() ;
          SM_ASSERT(   sInputNurbDomain.IsContainedBy(sOffsetNurbDomain)
                    && sOffsetNurbDomain.IsContainedBy(sInputNurbDomain, SM_EFF_ZERO)) ;
          SM_ASSERT(   sInputStepDomain.IsContainedBy(sOffsetStepDomain)
                    && sOffsetStepDomain.IsContainedBy(sInputStepDomain, SM_EFF_ZERO)) ;
        }

      SmFace *pFace = (SmFace *)GetFace() ;
      SmBrep *pBrep = pFace ? pFace->GetBrep() : NULL ;

      smgfx_Erase();
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,1) ; DrawUV(3,6); sm_GraphicsLoop();
      smgfx_SetLook(1,2, 0,1,0) ; pOffSphere->DrawUV(3,6); sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
#endif

  // all done
  SM_DUMP_AND_ASSERT2_VALID(pOffSphere) ;
  return SM_SUCCESS;

} // end SmSphere::CreateOffsetSurface

/*******************************************************************//**
PURPOSE: Create an 3D iso-parametric curve of a sphere given the
    Nurb parameter direction (U or V) and the constant parameter
    in that direction.

VIRTUAL FUNCTION ---
    for SmBSplineSurface - Make a BSpline Curve          (exact)
        SmSphere         - Make a SmCircle Curve         (exact)
        SmPlane          - Make a Line                   (exact)
        SmCone           - Make a Line or SmCircle Curve (exact)
        SmTorus          - Make a SmCircle curve         (exact)
        All Others       - Make a piecewise Hermite Curve approximation
                              good to optional tolerance or
                              dLength * SM_EFF_ZERO_SQRT * 100.0.
***********************************************************************/
SmStatus SmSphere::CreateIsoParametricCurve
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
  SmBSplineCurve *pIsoCurve ;
  SmBSplineSurface::CreateIsoParametricCurve(crContext,
                                             eSurfParam,
                                             dIsoParameter,
                                             0.0,
                                             pIsoCurve,
                                             pOptDomain,
                                             pOptMaxGap3d,      // out: opt achieved max gap, NULL to ignore, default:[NULL]
                                             pOptUVIsoCurve) ;  // out: opt 2d UVTrimCurve Line (diff parameterization), NULL to ignore, default:[NULL]

  // low work - when curve is degenerate don't bother making a circle
  if(pIsoCurve->IsDegenerate()) { rpNewIsoCurve = pIsoCurve ;
                                  return(SM_SUCCESS) ;
                                }

  // make pIsoCurve temporary - a copy of it ends up getting returned
  SmObjDelete sClean(pIsoCurve) ;

  // don't use converters to get Step Params because Polar Converters call this
  // function when getting themselves set up to date.

  // locals
  SmSurfParamType eSTEPParam = m_bSwapUV
                               ? ((eSurfParam == SM_SP_U) ? SM_SP_V : SM_SP_U)
                               : eSurfParam ;
  const SmPoint3d  &rSphereOrigin = m_vPosition.GetOriginRef() ;
  const SmVector3d &rSphereXAxis   = m_vPosition.GetXAxisRef() ;
  const SmVector3d &rSphereYAxis   = m_vPosition.GetYAxisRef() ;
  SmVector3d        sSphereZAxis   = m_vPosition.GetZAxis() ;

  // Get IsoCurve Nurb endPoints
  SmPoint3d sNurbMinPoint, sNurbMaxPoint ;
  SmExtent1d sNurbIvl = pIsoCurve->GetNaturalInterval() ;
  pIsoCurve->EvaluatePoint(sNurbIvl.GetMin(), sNurbMinPoint) ;
  pIsoCurve->EvaluatePoint(sNurbIvl.GetMax(), sNurbMaxPoint) ;
  SmVector3d sNurbMinVec = sNurbMinPoint - rSphereOrigin ;
  SmVector3d sNurbMaxVec = sNurbMaxPoint - rSphereOrigin ;

  // get iso circle parameters
  SmBoolean  bInsideOut ;
  SmPoint3d  sCircleCenter ;
  SmVector3d sXAxis, sYAxis ;
  double     dRadius ;
  SmExtent1d sAnalDomain ;
  if(eSTEPParam == SM_SP_U) // return Pole_to_Pole isoCurve circle
    {
      SmPoint3d sNurbMidPoint ;
      pIsoCurve->EvaluatePoint(sNurbIvl.Evaluate(.5), sNurbMidPoint) ;
      SmVector3d sNurbMidVec = sNurbMidPoint - rSphereOrigin ;

      sCircleCenter = m_vPosition.GetOriginRef() ;
      sXAxis        = sSphereZAxis * (sNurbMidVec * sSphereZAxis) ;
      SM_ASSERT(sXAxis.Length() > SM_EFF_ZERO_SQRT) ;
      sXAxis.Unitize() ;
      sYAxis        = sSphereZAxis ;
      dRadius       = m_dRadius ;
      bInsideOut    = m_bInsideOut ;
      sAnalDomain   = m_vAnalUVDomain.GetVInterval() ;
    }
  else // return sweep isoCurve circle
    {
      sCircleCenter =   m_vPosition.GetOriginRef()
                      + sNurbMinVec.Dot(sSphereZAxis) * sSphereZAxis ;
      sXAxis        = rSphereXAxis ;
      sYAxis        = rSphereYAxis ;
      dRadius       = (sNurbMinPoint - sCircleCenter).Length() ;
      bInsideOut    = FALSE ;
      sAnalDomain   = m_vAnalUVDomain.GetUInterval() ;
    }

  // get isoParamCurve domain

  // when using the current domain
  SmExtent1d sCircleAnalDomain ;
  if(!pOptDomain)
    {
      sCircleAnalDomain = sAnalDomain ;
    }
  else // get angles to min/max points inside periodic interval
    {
      SmVector3d sCircleZAxis = sXAxis * sYAxis ;
      double dMinAngRad, dMaxAngRad ;
      double dMinAngDeg, dMaxAngDeg ;
      sCircleZAxis.CCWAngleBetween(sXAxis, sNurbMinVec, dMinAngRad) ;
      sCircleZAxis.CCWAngleBetween(sXAxis, sNurbMaxVec, dMaxAngRad) ;
      sAnalDomain.ContainsPeriodicValue(dMinAngRad * 180.0 / SM_PI, 360.0, &dMinAngDeg) ;
      sAnalDomain.ContainsPeriodicValue(dMaxAngRad * 180.0 / SM_PI, 360.0, &dMaxAngDeg) ;

      // when both values are on a closed interval seam
      // (Use same tolerance as in ContainsPeriodicValue(), which gets called here.)
      double dPeriod = 360.0;
      // Tol: Even 10x still a little too tight: a340retLink_1.
      double dTol    = SM_EFF_ZERO * 100.0 * ( 1.0 + dPeriod );
      if(   sAnalDomain.IsClosed( dPeriod )
         && sAnalDomain.IsValueOnPeriodicBoundary( dMinAngDeg, dPeriod, dTol )
         && sAnalDomain.IsValueOnPeriodicBoundary( dMaxAngDeg, dPeriod, dTol ))
        {
          // set interval to whole domain
          sCircleAnalDomain = sAnalDomain ;
        }
      else
        {
          // set isoCurve analytic domain accounting for m_binside reversals
          sCircleAnalDomain.SetMinMax(smos_Min(dMinAngDeg, dMaxAngDeg),
                                      smos_Max(dMinAngDeg, dMaxAngDeg))  ;
        }
    } // end need to compute interval from angles branch

  // degenerate cases should have already been detected
  SM_ASSERT(   !SM_IS_ZERO(dRadius)
            && !SM_IS_ZERO(sAnalDomain.GetLength())) ;

  // create the circle using the isoCurve parameterization
  SmCircle *pCircle = new (crContext) SmCircle(sCircleCenter,
                                               sXAxis,
                                               sYAxis,
                                               sCircleAnalDomain,
                                               dRadius, 3, &crContext,
                                               pIsoCurve,
                                               bInsideOut) ;
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

} // end SmSphere::CreateIsoParametricCurve

//      /*******************************************************************//**
//      PURPOSE: return a circle which is the sphere's meridian
//                  (centered on Sphere running from pole to pole
//                  for the given input rotation measured CCW from the
//                  placement' X Axis about the Z Axis in degrees.
//
//      NOTES:
//        1. The given angle does not need to lie within the angular domain of
//                      the surface.
//        2. When m_bMakeNurbGenCurve = TRUE  return SmBSplineCurve type
//                                    = FALSE return SmCircle Type
//        3. The returned circle has STEP Params [-90 to 90]
//      ***********************************************************************/
//      SmStatus SmSphere::CreateGeneratorFromAngle
//        (const SmContext & crContext,             // in : new object context
//         double            dAngleDegArg,          // in : desired angle in degrees
//         SmBSplineCurve *& rpGeneratorCurveArg)   // out: new curve
//                                                  //      type = SmCircle       when m_bMakeNurbGenCurve = FALSE
//                                                  //      type = SmBSplineCurve when m_bMakeNurbGenCurve = TRUE
//        const
//      {
//        // if possible - return copied and rotated genCurve
//        SM_ASSERT(m_pGenCurve != NULL) ;
//        if(m_pGenCurve)
//          {
//            // copy the current GenCurve
//            SmCurve *pGenCurveCopy = NULL ;
//            m_pGenCurve->Copy(crContext, (SmCurve *&)rpGeneratorCurveArg) ;
//
//            // define the rotation matrix
//            SmAxis2Placement sRotation ;
//            sRotation.RotateAboutAxisAtPoint(dAngleDegArg * SM_PI/180.0,
//                                             m_vPosition.GetOriginRef(),
//                                             m_vPosition.GetZAxis()) ;
//
//            // rotate the GenCurve Copy
//            rpGeneratorCurveArg->Transform(sRotation) ;
//          }
//        else // make and return a SmCircle object
//          {
//            double dMinVAngle = m_vAnalUVDomain.GetMin().y; // up->down [-90 to 90 degrees]
//            double dMaxVAngle = m_vAnalUVDomain.GetMax().y; //          [-90 to 90 degress]
//
//            // get sphere position locals
//            const SmVector3d &rOrigin = m_vPosition.GetOriginRef();
//            const SmVector3d &rXAxis  = m_vPosition.GetXAxisRef();
//            SmVector3d        sZAxis  = m_vPosition.GetZAxis();
//
//            // circle coordinate - place circle start/end at sphere bottom pole
//            // to build circle through sphere's 0/360 meridian
//            SmAxis2Placement sCirclePos(rOrigin, rXAxis, sZAxis) ;
//
//            // rotate sCirclePos about its YAxis (sphere's ZAxis) by dAngleDegArg
//            if(dAngleDegArg != 0.0)
//              {
//                sCirclePos.RotateAboutAxisAtPoint(dAngleDegArg * SM_PI / 180.0,
//                                                  rOrigin,
//                                                  sZAxis);
//              }
//
//            // create circle with sphere's radius and sCircle position
//            // whose parameterization is in degrees [0 to 360]
//            SmCircle *pNewCircle;
//            SmExtent1d sCircleIvl(dMinVAngle,dMaxVAngle);
//            SE(SmCircle::CreateCanonical(crContext,
//                                         sCirclePos,
//                                         m_dRadius,
//                                         pNewCircle,
//                                         &sCircleIvl));
//            rpGeneratorCurveArg = pNewCircle;
//          } // end build a circle genCurve branch
//
//      #ifdef SM_DEBUG_CODE
//      SmBoolean bDebugMe = FALSE;
//        if (bDebugMe)
//          {
//            rpGeneratorCurveArg->Dump();
//            smgfx_Erase();
//            smgfx_SetColor(0,0,0); rpGeneratorCurveArg->Draw(); sm_GraphicsLoop();
//            sm_GraphicsLoop();
//
//          }
//      #endif

// gwc:not sure what to do about InsideOut flag
//        // reverse parameterization for inside out spheres
//        if (m_bInsideOut)
//          {
//            SmExtent1d sIvl = pBSC->GetNaturalInterval();
//            SER(pBSC->ReverseParameterization(sIvl,sIvl));
//          }

//        // all done
//        return SM_SUCCESS;
//
//      } // end SmSphere::CreateGeneratorFromAngle

/*******************************************************************//**
PURPOSE: Convert from STEP to NURBS parameterization
            so that EvaluatePoint(NurbUV) = EvaluateSTEPPoint(StepUV)

NOTES:
***********************************************************************/
SmStatus SmSphere::ConvertUVFromSTEPToNURBS
  (const SmPoint2d & crSTEPUV,  // in : Target 2d Point in [degrees, height]
   SmPoint2d       & rNURBSUV)  // out: 2d Point in SMLib parameterization
  const
{
  SM_ASSERT(HavePolarConversion()) ;

  // convert sweep coordinate
  SmBoolean bExactConversion;
  SER(m_vPolarConverter.ConvertToNURBSParameter(crSTEPUV.x,rNURBSUV.x,bExactConversion));
  SM_ASSERT(bExactConversion) ;

//        // Sphere genCurve parameter spaces:
//        //   1. StepParams for a 'STEPGenCurve' are in degrees from [-90 to +90]
//        //   2. PolarParams for a 'PolarCurve' are in degrees either
//        //             from [-90 to +90] or [+90 to -90]
//        //   3. Nurb params are determined by the genCurve's knot vector but
//        //      always run in the same general direction as the polarParams
//
//        // Get Gencurve PolarParam from StepParam. Because Sphere GenCurves
//        // are centered on 0 the PolarParam/StepParam turns out to be simple:
//        double dPolarParam =   m_bInsideOut
//                             ? -crSTEPUV.y
//                             :  crSTEPUV.y ;
  double dPolarParam = crSTEPUV.y ;

  // convert GenCurve PolarParam to NurbsParam
  SM_ASSERT(m_pGenCurve && m_pGenCurve->IsKindOf(SmCircle_TYPE)) ;
  SER(((SmCircle *)m_pGenCurve)->ConvertTFromSTEPToNURBS(dPolarParam,rNURBSUV.y));

  // m_bSwapUV - Nurb and STEP parameters are swapped
  if(m_bSwapUV) { double dTemp = rNURBSUV.x ;
                  rNURBSUV.x   = rNURBSUV.y ;
                  rNURBSUV.y   = dTemp ;
                }

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  // check conversions reciprocity
  SmPoint2d sCheckUV ;
  ConvertUVFromNURBSToSTEP(rNURBSUV, sCheckUV) ;
  double dUVDist       = (crSTEPUV - sCheckUV).Length() ;
  double dScaledUVZero = SM_EFF_ZERO * 100.0 * (1.0 + crSTEPUV.GetMaxDimension()) ;

  SM_ASSERT(dUVDist < dScaledUVZero) ;

  // check quality of the conversion
  SmPoint3d sSTEPPoint[3], sNurbPoint[3] ; // ordered:[D, Dv, Du]
  Evaluate    (rNURBSUV, 1, 1, TRUE, TRUE, TRUE, sNurbPoint) ;
  EvaluateSTEP(crSTEPUV, 1, 1, TRUE, TRUE, TRUE, sSTEPPoint) ;
  SmVector3d sStepN = sSTEPPoint[2] * sSTEPPoint[1] ;
  SmVector3d sNurbN = sNurbPoint[2] * sNurbPoint[1] ;
  double sStepNLength = sStepN.Length() ;
  double sNurbNLength = sNurbN.Length() ;

  // positions should be the same
  double dDist = (sNurbPoint[0] - sSTEPPoint[0]).Length() ;
  double dScaled3dZero = SM_EFF_ZERO * (1.0 + sNurbPoint[0].GetMaxDimension()) ;
  SM_ASSERT(dDist < dScaled3dZero) ;

  // normals should be related
  double dScaledTangZero = 100.0 * 100.0 * SM_EFF_ZERO * (   1.0
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
      sStepN = sStepN/sStepNLength ;
      sNurbN = sNurbN/sNurbNLength ;
      double dInvert =   (m_bSwapUV    ? -1.0 : 1.0)
                       * (m_bInsideOut ? -1.0 : 1.0) ;

      // normals should be related
      SM_ASSERT(SM_ARE_SAME(1.0, dInvert * sStepN.Dot(sNurbN))) ;
    }

  if(bDebugMe)
    {
      Dump() ;
    }
#endif

  // all done
  return SM_SUCCESS;

} // end SmSphere::ConvertUVFromSTEPToNURBS

/*******************************************************************//**
PURPOSE: Convert from NURBS to STEP parameterization
            so that EvaluatePoint(NurbUV) = EvaluateSTEPPoint(StepUV).

NOTES:
***********************************************************************/
SmStatus SmSphere::ConvertUVFromNURBSToSTEP
  (const SmPoint2d & crNURBSUV,  // in : Target 2d Point in SMLib parameterization
   SmPoint2d       & rSTEPUV)    // out: 2d Point in [degrees, height]
  const
{
  SM_ASSERT(HavePolarConversion()) ;

  // m_bSwapUV - Nurb and STEP parameters are swapped
  SmPoint2d sNURBUV ;
  if (m_bSwapUV) { sNURBUV.x = crNURBSUV.y ; // sweep  param
                   sNURBUV.y = crNURBSUV.x ; // GenCurve param
                 }
  else           { sNURBUV.x = crNURBSUV.x ;
                   sNURBUV.y = crNURBSUV.y ;
                 }

  // Sphere genCurve parameter spaces:
  //   1. StepParams for a 'STEPGenCurve' are in degrees from [-90 to +90]
  //   2. PolarParams for a 'PolarCurve' are in degrees either
  //             from [-90 to +90] or [+90 to -90]
  //   3. Nurb params are determined by the genCurve's knot vector but
  //      always run in the same general direction as the polarParams

  // convert sweep coordinate
  SmBoolean bExactConversion;
  SER(m_vPolarConverter.ConvertToPolarParameter(sNURBUV.x,rSTEPUV.x,bExactConversion));
  SM_ASSERT(bExactConversion) ;

  // convert genCurve NurbParam to PolarParam
  double dPolarParam = 0.0;
  SM_ASSERT(m_pGenCurve && m_pGenCurve->IsKindOf(SmCircle_TYPE)) ;
  SER(((SmCircle *)m_pGenCurve)->ConvertTFromNURBSToSTEP(sNURBUV.y,dPolarParam)) ;
  SM_ASSERT(bExactConversion) ;

//        // convert genCurve PolarParam to StepParam
//        rSTEPUV.y  =   m_bInsideOut
//                     ? -dPolarParam
//                     :  dPolarParam ;
  rSTEPUV.y  = dPolarParam ;

  // all done
  return SM_SUCCESS;

} // end SmSphere::ConvertUVFromNURBSToSTEP

/*******************************************************************//**
PURPOSE: Evaluate the point & derivatives using the STEP parameterization.
            EvaluatePoint(NurbUV) = EvaluateSTEPPoint(StepUV).
            But tangents will vary in magnitude and
            norm may be negated

NOTES:
  S(U,V) =   origin                            U range:[-360 to 360] with max length = 360
          + radius * cos(V) * cos(U) * XAxis   V range:[-90 to 90 ]
          + radius * cos(V) * sin(U) * YAxis
          + radius * sin(V)

***********************************************************************/
SmStatus SmSphere::EvaluateSTEP
  (const SmPoint2d & crUV,    // in : U=CCW Rot about Z from X in degrees,          [-360 to 360]  with max length = 360
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
  SmPoint2d sUV = crUV ;
  SmPoint3d  rO = m_vPosition.GetOriginRef() ;
  SmVector3d rX = m_vPosition.GetXAxisRef () ;
  SmVector3d rY = m_vPosition.GetYAxisRef () ;
  SmVector3d sZ = m_vPosition.GetZAxis () ;
  double     dR = m_dRadius ;
  ULONG lMinHighest = smos_Min(lHighestUDeriv, lHighestVDeriv) ;

  // per SMLib practice clamp out of bounds UV points
  if ( !IsOutOfBoundsEnabled() && !m_vAnalUVDomain.ContainsPoint2d( sUV ) )
    { sUV = m_vAnalUVDomain.ClampPoint2d(sUV); }

  // evaluate expensive trigonometric functions
  double dCosU = smos_Cosine(sUV.x*SM_PI/180.0) ;
  double dSinU = smos_Sine  (sUV.x*SM_PI/180.0) ;
  double dCosV = smos_Cosine(sUV.y*SM_PI/180.0) ;
  double dSinV = smos_Sine  (sUV.y*SM_PI/180.0) ;

  // evaluate position
  aDerivatives[0] =    rO
                     + dR * (  dCosV * (  dCosU * rX
                                        + dSinU * rY)
                             + dSinV * sZ) ;

  // 1st U and V derivatives
  if(lHighestUDeriv >= 1)   { aDerivatives[  lHighestVDeriv+1] = dR * dCosV * (-dSinU * rX + dCosU * rY) ; }
  if(lHighestVDeriv >= 1)   { aDerivatives[1]                  = dR * (-dSinV * ( dCosU * rX + dSinU * rY) + dCosV * sZ) ; }

  // cross derivative dUV
  if(   lHighestUDeriv >= 1
     && lHighestVDeriv >= 1
     && lMinHighest    >= 2){ aDerivatives[  lHighestVDeriv+2] = dR * -dSinV * (-dSinU * rX + dCosU * rY) ; }

  //  cross derivative dUVV
  if(   lHighestUDeriv >= 1
     && lHighestVDeriv >= 2
     && lMinHighest    >= 3){ aDerivatives[  lHighestVDeriv+3] = dR * -dCosV * (-dSinU * rX + dCosU * rY) ; }

  // 2nd U and V derivatives
  if(lHighestUDeriv >= 2)   { aDerivatives[2*lHighestVDeriv+2] = dR *  dCosV * (-dCosU * rX - dSinU * rY) ; }
  if(lHighestVDeriv >= 2)   { aDerivatives[2]                  = dR * (-dCosV * ( dCosU * rX + dSinU * rY) - dSinV * sZ) ; }

  // cross derivative dUUV
  if(   lHighestUDeriv >= 2
     && lHighestVDeriv >= 1
     && lMinHighest    >= 3){ aDerivatives[2*lHighestVDeriv+3] = dR * -dSinV * (-dCosU * rX - dSinU * rY) ; }

  // 3rd U and V derivatives
  if(lHighestUDeriv >= 3)   { aDerivatives[3*lHighestVDeriv+3] = dR *  dCosV * ( dSinU * rX - dCosU * rY) ; }
  if(lHighestVDeriv >= 3)   { aDerivatives[3]                  = dR * ( dSinV * ( dCosU * rX + dSinU * rY) - dCosV * sZ) ; }


  // The formulas above are per radian; STEP parameters are in degrees (U and V are degrees).
  smsurf_ScaleSTEPAngularDerivatives(aDerivatives, lHighestUDeriv, lHighestVDeriv, TRUE, TRUE) ;

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

} // end SmSphere::EvaluateSTEP

/*******************************************************************//**
PURPOSE: Evaluate the point using the STEP parameterization.

NOTES: crUV:[-360_to_360, -90_to_90]
  when Sphere was built canonically, m_bMakeNurbGenCurve = FALSE
       and GenCurve will be an SmCircle object.
    U = CCW angle in degrees about Z axis from X axis [-360_to_360] with max length = 360 and
    V = angle in degrees from bot pole to top pole, [-90_to_90].

  when Sphere was built with m_bMakeNurbGenCurve = TRUE
       GenCurve will be an SmBSplineCurve object.
    U = CCW angle in degrees about Z axis from X axis [-360_to_360] with max length = 360 and
    V = Nurb Param from bot pole to top pole, [-90_to_90] but not exactly
        equal to an angle.
***********************************************************************/
SmStatus SmSphere::EvaluateSTEPPoint
  (const SmPoint2d & crUV,   // in : target UVPoint, range:[-360 to 360, -90 to 90]
   SmPoint3d       & rPoint) // out: sphere point
  const
{
  // pass the call along
  return(EvaluateSTEP(crUV, 0, 0, TRUE, TRUE, TRUE, &rPoint)) ;

} // end SmSphere::EvaluateSTEPPoint

/*******************************************************************//**
PURPOSE: Virtual method to find the unit surface normal.

NOTES: Input uv parameters are spline values, not STEP.
***********************************************************************/
SmStatus SmSphere::EvaluateNormal(
        const SmPoint2d & crUV,     // in: uv parameter value
        SmBoolean ,                 // in : not used in this virtual method
        SmBoolean ,                 // in : not used in this virtual method
        SmVector3d & rSurfaceNormal // out: unit-normal
    ) const
{
    SmPoint3d sSurfPt;
    SER( EvaluatePoint( crUV, sSurfPt ));
    rSurfaceNormal = sSurfPt - m_vPosition.GetOriginRef();

    // Note: could just divide by the radius,
    // but that can lose a bit or two of precision.
    SER( rSurfaceNormal.Unitize() );

    if ( m_bSwapUV != m_bInsideOut) // XOR
        { rSurfaceNormal = -rSurfaceNormal; }

#ifdef SM_DEBUG_CODE
// Note, turn this off: the other EvaluateNormal call can cause infinite loop
// if there's a problem with degenerate 1st derivs.
//  // compare spline version
//  SmVector3d sTestNormal;
//  SmStatus eStat = SmSurfOfRevolution::EvaluateNormal( crUV, FALSE, FALSE, sTestNormal );
//  if ( eStat == SM_SUCCESS ) {
//  double dDiff = ( sTestNormal - rSurfaceNormal ).Length();
//  if(dDiff > 0.001)
//    {
//      SM_ASSERT_MSG( dDiff < 0.001,
//          _T("SmSphere::EvaluateNormal() result is different from base class version") );
//    }
//  }
#endif

    return SM_SUCCESS;
}


/*******************************************************************//**
PURPOSE: Get the STEP-parameterization domain from a sphere.

NOTES: returns step domain in range:[-360 to 360 max Length 360, -90 to 90]
***********************************************************************/
SmExtent2d SmSphere::GetSTEPUVDomain()
 const
{
  return m_vAnalUVDomain ;

} // end SmSphere::GetSTEPUVDomain

/*******************************************************************//**
PURPOSE: Given a point in Euclidian space determine the corresponding
     extrema points on the surface based on STEP-parametrization.
     Valid solver operations for this method include:
     SM_SO_MINIMIZE, SM_SO_MAXIMIZE, SM_SO_NORMALIZE, SM_SO_INTERSECT.

NOTES:
***********************************************************************/
SmStatus SmSphere::GlobalPointSolveSTEP
  (const SmExtent2d      & crAnalUVDomain,        // in : domain within [-360_to_360 max length 360, -90_to_90 ]
   SmSolverOperationType   eSolverOperation,      // in : oneof SM_SO_MINIMIZE,
                                                  //            SM_SO_MAXIMIZE,
                                                  //            SM_SO_NORMALIZE,
                                                  //            SM_SO_INTERSECT
   const SmPoint3d       & crTestPoint,           // in : Target Point
   double                  dDistanceTolerance,    // in : for SM_SO_MINIMIZE  - only used with cpdOptTargetDistance
                                                  //          SM_SO_MAXIMIZE  - only used with cpdOptTargetDistance
                                                  //          SM_SO_NORMALIZE - not used
                                                  //          SM_SO_INTERSECT - only return answers less than this
   const double          * cpdOptTargetDistance,  // in : for SM_SO_MINIMIZE  - only return answers less than this + distanceTolerance
                                                  //          SM_SO_MAXIMIZE  - only return answers greater than this - distanceTolerance
                                                  //          SM_SO_NORMALIZE - not used
                                                  //          SM_SO_INTERSECT - not used
   SmSolutionRequestedType eSolutionRequested,    // in : oneof SM_SR_ALL, SM_SR_SINGLE
   SmSolutionArray       & rSolutions)            // out:
{
  // check input
  SM_ASSERT(   eSolverOperation == SM_SO_MINIMIZE
            || eSolverOperation == SM_SO_MAXIMIZE
            || eSolverOperation == SM_SO_NORMALIZE
            || eSolverOperation == SM_SO_INTERSECT);

  // init output
  rSolutions.ReSet() ;

  // no work - no overlap in periodic domains
  if(m_vAnalUVDomain.AreDisjointPeriodic(crAnalUVDomain, 360.0, 0.0))
    { return(SM_SUCCESS) ; }

  // sphere position locals
  ULONG ii, jj ;
  const SmPoint3d  &rOrigin = m_vPosition.GetOriginRef() ;
  const SmVector3d &rXAxis  = m_vPosition.GetXAxisRef() ;
  const SmVector3d &rYAxis  = m_vPosition.GetYAxisRef() ;
  SmVector3d        sZAxis  = m_vPosition.GetZAxis() ;
  //double dScaledZero = SM_EFF_ZERO * (1.0 + smos_Max(rOrigin.GetMaxDimension(), m_dRadius)) ;  //unused

  // Get Sphere NearUVPoint in Spherical Coordinates (degrees)
  SmLocationType eLoc ;
  SmPoint2d sNearUVPoint ;
  STEPInversion(m_vAnalUVDomain, crTestPoint, dDistanceTolerance, sNearUVPoint, eLoc) ;

  double dU = sNearUVPoint.x ;
  double dV = sNearUVPoint.y ;

  // set up Sphere FarUVPoint
  SmPoint2d sFarUVPoint(   dU < 180.0
                         ? dU+180.0
                         : dU-180.0,
                        -dV) ;

  // solution locals
  ULONG     lCount = 0 ;   // 0 = not done
  SmPoint2d *apPoints[2] ;
  SmPoint2d sSolPoint ;

  // quick potential solves - contained minima
  SmBoolean bDone = FALSE ;
  switch(eSolverOperation)
    {
      case SM_SO_INTERSECT  :
      case SM_SO_MINIMIZE   : if(   m_vAnalUVDomain.ContainsPeriodicPoint2d(sNearUVPoint,360.0,0.0)
                                 && crAnalUVDomain. ContainsPeriodicPoint2d(sNearUVPoint,360.0,0.0))
                                {
                                  apPoints[0] = &sNearUVPoint ;
                                  lCount ++ ;
                                  bDone = TRUE ;
                                }
                              break ;
      case SM_SO_MAXIMIZE   : if(   m_vAnalUVDomain.ContainsPeriodicPoint2d(sFarUVPoint,360.0,0.0)
                                 && crAnalUVDomain. ContainsPeriodicPoint2d(sFarUVPoint,360.0,0.0))
                                { apPoints[0] = &sFarUVPoint ;
                                  lCount ++ ;
                                  bDone = TRUE ;
                                }
                              break ;
      case SM_SO_NORMALIZE  : if(   m_vAnalUVDomain.ContainsPeriodicPoint2d(sNearUVPoint,360.0,0.0)
                                 && crAnalUVDomain. ContainsPeriodicPoint2d(sNearUVPoint,360.0,0.0))
                                { apPoints[0] = &sNearUVPoint ;
                                  lCount ++ ;
                                }
                              if(   m_vAnalUVDomain.ContainsPeriodicPoint2d(sFarUVPoint,360.0,0.0)
                                 && crAnalUVDomain. ContainsPeriodicPoint2d(sFarUVPoint,360.0,0.0))
                                {
                                  apPoints[lCount] = &sFarUVPoint ;
                                  lCount ++ ;
                                }
                              bDone = TRUE ;
                              break ;
      default: SER(SM_ERR) ;
    }

  // arrive here bDone = TRUE possible answers are known
  //             bDone = FALSE check points on boundaries for answers

  // when boundaries need to be checked for nearest/farthest solution
  if(bDone == FALSE)
    {
      // init dDistance for search
      double dThisDistance ;
      double dThisAngleDeg ;
      double dDistance =  (eSolverOperation == SM_SO_MAXIMIZE)
                         ? -SM_BIG_DOUBLE
                         :  SM_BIG_DOUBLE ;
      lCount = 1 ;
      apPoints[0] = &sSolPoint ;

      // get the domain intersections
      SmTArray<SmExtent2d> sDomains ;
      m_vAnalUVDomain.IntersectPeriodic(crAnalUVDomain, 360.0, 0.0, sDomains) ;

      // for every intersection domain
      for(ii=0;ii<sDomains.GetSize();ii++)
        {
          SmExtent2d &rDomain = sDomains[ii] ;
          double      dMinU   = rDomain.GetMin().x ;
          double      dMaxU   = rDomain.GetMax().x ;
          double      dMinV   = rDomain.GetMin().y ;
          double      dMaxV   = rDomain.GetMax().y ;

          // for every boundary on intersection domain
          for(jj=0;jj<4;jj++)
            {
              SmPoint3d sCenter, sX, sY ;
              double dRadius, dStartAngDeg, dEndAngDeg ;

              // switch to given domain after all natural boundaries are checked
              // get angle (radians) for this boundary
              double dAngDeg =   jj == 0 ? dMaxV
                               : jj == 1 ? dMinV
                               : jj == 2 ? dMaxU
                               :           dMinU ;
              double dAngRad = dAngDeg*SM_PI/180 ;

              // get circle boundary arc description
              if(jj == 0 || jj == 1) { // min/max V boundaries
                                       sCenter = rOrigin + sZAxis * smos_Sine(dAngRad) ;
                                       dRadius = m_dRadius * smos_Cosine(dAngRad) ;
                                       sX      = rXAxis ;
                                       sY      = rYAxis ;
                                       dStartAngDeg = dMinU ;
                                       dEndAngDeg   = dMaxU ;
                                     }
              else                   { // min/max U boundries
                                       sCenter = rOrigin ;
                                       dRadius = m_dRadius ;
                                       sX      = smos_Cosine(dAngRad) * rXAxis + smos_Sine(dAngRad)*rYAxis ;
                                       sY      = sZAxis ;
                                       dStartAngDeg = dMinV ;
                                       dEndAngDeg   = dMaxV ;
                                     }

              // get min/max distance to this arc
              SmBoolean bMinFlag = TRUE;
              if (eSolverOperation == SM_SO_MAXIMIZE)
                bMinFlag = FALSE;

              smgu_ArcSegmentPointDistance(sCenter, sX, sY, dRadius,
                                           dStartAngDeg, dEndAngDeg,
                                           crTestPoint,
                                           bMinFlag,
                                           dThisDistance, dThisAngleDeg) ;

              // save best answers
              if(   (eSolverOperation == SM_SO_MAXIMIZE && dThisDistance > dDistance)
                 || (eSolverOperation != SM_SO_MAXIMIZE && dThisDistance < dDistance))
                {
                  // save best distance
                  dDistance = dThisDistance ;

                  // convert circle angle back to sphere angles
                  if(jj == 0 || jj == 1) { sSolPoint.x = dThisAngleDeg ;
                                           sSolPoint.y = dAngDeg ;
                                         }
                  else                   { sSolPoint.x = dAngDeg ;
                                           sSolPoint.y = dThisAngleDeg ;
                                         }
                } // end save best answer check

            } // end iter every sphere boundary
        } // end iter every intersection domain
    } // end need to check boundary solutions

  // output valid solutions
  for(ii=0;ii<lCount;ii++)
    {
      SmSolution sSol;
      SmPoint3d sSolPnt ;
      EvaluateSTEPPoint(*apPoints[ii], sSolPnt) ;
      SmVector3d sVec                = sSolPnt - crTestPoint ;
      double dDist                   = sVec.Length() ;
      sSol.m_lNumVariables           = 2 ;
      sSol.m_lNumObjects             = 1 ;
      sSol.m_vStart[0]               = apPoints[ii]->x ;
      sSol.m_vStart[1]               = apPoints[ii]->y ;
      sSol.m_vStart.m_dSolutionValue = dDist;
      sSol.m_eSolutionType           = SM_ST_SINGLE_VALUE;
      sSol.m_apObjects[0]            = (SmObject *)this ;

      // see if solution is valid
      SmBoolean bKeep = FALSE ;
      switch(eSolverOperation)
        {
          case SM_SO_INTERSECT : bKeep =    dDist < dDistanceTolerance ; break ;
          case SM_SO_MINIMIZE  : bKeep =    cpdOptTargetDistance == NULL
                                         || dDist < *cpdOptTargetDistance + dDistanceTolerance ; break ;
          case SM_SO_MAXIMIZE  : bKeep =    cpdOptTargetDistance == NULL
                                         || dDist > *cpdOptTargetDistance - dDistanceTolerance ; break ;
          case SM_SO_NORMALIZE : bKeep =    TRUE ; break ;

          case SM_SO_INTERSECT_WIREFRAME:
          case SM_SO_INTERSECTION_TEST:
          case SM_SO_AT_DISTANCE:
          case SM_SO_FIND:
          case SM_SO_RAYFIRE:
          case SM_SO_3D_SIGNED_DIRECTED_MINIMIZE:
          case SM_SO_3D_SIGNED_DIRECTED_MAXIMIZE:
          case SM_SO_SIGNED_DIRECTED_MINIMIZE:
          case SM_SO_DIRECTED_MINIMIZE:
          case SM_SO_DIRECTED_MAXIMIZE:
          case SM_SO_PROJECTED_MINIMIZE:
          case SM_SO_PROJECTED_MAXIMIZE:
          case SM_SO_PROJECTED_INTERSECT:
          case SM_SO_ROTATED_PROJECTED_INTERSECT:
          case SM_SO_PERSPECTIVE_INTERSECT:
          case SM_SO_ANGLE_MINIMIZE:
          case SM_SO_SIGNED_ANGLE_MINIMIZE:
          case SM_SO_PROJECTED_TANGENCY:
          case SM_SO_SIGNED_PIVOT_MINIMIZE:
          case SM_SO_SURFACE_PROJECTED_INTERSECT:
          case SM_SO_PROJECTED_TANGENT_THROUGH_POINT:
              break;
        }

      // keep valid solutions
      if(bKeep)
        {
          rSolutions.Add(sSol);
        }
    } // end iter every possible solution

  // set the number of solutions
  // when eSolverOperation == Normalize we might have two solutions
  // while being asked for 1
  if(   eSolutionRequested == SM_SR_SINGLE
     && rSolutions.GetSize() == 2)
    { rSolutions.SetSize(1) ; }


  // gwc: the alternative solver technique - less typing but more complicated
  //
  //      // pass the call along using internal analytic domain
  //      SER(SmSurfOfRevolution::GlobalPointSolveSTEP(sNewUVDomain,
  //          eSolverOperation,crTestPoint,dDistanceTolerance,
  //          cpdOptTargetDistance,eSolutionRequested,rSolutions));

  // all done
  return SM_SUCCESS;

} // end SmSphere::GlobalPointSolveSTEP

/*******************************************************************//**
PURPOSE: This virtual method may invoke special cases of intersection
    of analytics with planes.

NOTES:
***********************************************************************/
SmStatus SmSphere::GlobalSurfaceIntersect
  (const SmContext     & crContext,               // in : Context for the creation of curves
   const SmExtent2d    & crUVDomain,              // in : in range:[-360_to_360 max length 360, -90_to_90]
   const SmSurface     & crOtherSurface,          // in : target 2nd intersecting surface
   const SmExtent2d    & crOtherUVDomain,         // in : other surface domain
   const SmBoolean       bUseSurfaceEdges[2],     // in : Normally both are TRUE unless you know
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
  // check for special cases
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

      case SmCone_TYPE:
          SER(IntersectWithCone(crContext,crUVDomain,(SmCone&)crOtherSurface,
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

      case SmSurfOfRevolution_TYPE:
          {
              SmBoolean bTmpUseEdges[2];
              bTmpUseEdges[0] = bUseSurfaceEdges[1];
              bTmpUseEdges[1] = bUseSurfaceEdges[0];
              SmSurfOfRevolution & crSurfOfRev = (SmSurfOfRevolution&)crOtherSurface;
              SER(crSurfOfRev.IntersectWithSphere(crContext,crOtherUVDomain,
                                                  (SmSphere &)*this,crUVDomain, bTmpUseEdges, pdOptApproxTol3d,
                                                  pdOptAngTolRad, bNeedNurbIntersection,
                                                  pOpt3DCurves, pOptSurface2UVCurves,
                                                  pOptSurface1UVCurves, pOptCurveTypes, pOptDeviations));
          }
          break;
      default:
          break;

    } // end special intersection case switch

  // arrive here with bNeedNurbIntersection == TRUE,
  // then analytic intersections were not found

  // try global intersection based on m_pNurb surface
  if (bNeedNurbIntersection)
    {
      SER(SmSurface::GlobalSurfaceIntersect(crContext, crUVDomain,
                                            crOtherSurface, crOtherUVDomain,
                                            bUseSurfaceEdges, pdOptApproxTol3d,
                                            pdOptAngTolRad, pOpt3DCurves, pOptSurface1UVCurves,
                                            pOptSurface2UVCurves, pOptCurveTypes, pOptDeviations));
    }

  return SM_SUCCESS;

} // end SmSphere::GlobalSurfaceIntersect

/*******************************************************************//**
PURPOSE: Find intersection curve of this sphere with a plane

NOTES:
***********************************************************************/
SmStatus SmSphere::IntersectWithPlane
  (const SmContext     & crContext,                  // in : context for new object construction
   const SmExtent2d    & crUVDomain,                 // in : in range [-360_to_360 max length 360, -90_to_90]
   const SmPlane       & crPlane,                    // in : target intersection plane
   const SmExtent2d    & crOtherUVDomain,            // in : intersection limit for target plane
   const SmBoolean       /* bUseSurfaceEdges */[2],  // in : TRUE = find xsect curve start points from boundaryCurve/surface xsects
                                                     //      typically these values are TRUE - its a small savings if you know
                                                     //      the boundaries of one surface don't intersect the other surface
   const SmApproxTol3d * pdOptApproxTol3d,           // in : max distance between coincident points
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
  // init output
  rbNeedsMoreIntersections = FALSE;
  if (!pOpt3DCurves)        SER(SM_ERR); // Must have 3D curves for analytical intersections.
  if (pOpt3DCurves)         pOpt3DCurves->ReSet();
  if (pOptSurface1UVCurves) pOptSurface1UVCurves->ReSet();
  if (pOptSurface2UVCurves) pOptSurface2UVCurves->ReSet();
  if (pOptCurveTypes)       pOptCurveTypes->ReSet();
  if (pOptDeviations)       pOptDeviations->ReSet();

  // locals
  ULONG ii ;
  SmVector3d        sPlaneNormal = crPlane.GetPosition().GetZAxis();
  const SmPoint3d  &rPlaneOrigin = crPlane.GetPosition().GetOriginRef();
  const SmPoint3d  &rSphOrigin   = GetPosition().GetOriginRef();
  double dTol = 0.0;
  if( pdOptApproxTol3d ) dTol = *pdOptApproxTol3d;
  else dTol = SM_EFF_ZERO_SQRT;

  // get distance from sphereOrigin to Plane
  SmPoint3d  sProjectedPoint;
  SER(smgu_PointProjectToPlane(rSphOrigin,rPlaneOrigin,sPlaneNormal,sProjectedPoint));
  double dDistToPlane = rSphOrigin.DistanceBetween(sProjectedPoint);

  // skip planes that don't intersect sphere
  if (dDistToPlane > m_dRadius + dTol)
    { return SM_SUCCESS; }

  // Do a little bounding box computation to see if things
  // really intersect.
  if(crPlane.IsBounded())
    {
      // get min distance from sphereOrigin to planeBoundingBox
      SmExtent3d sBBox;
      SER(crPlane.CalculateBoundingBox(crOtherUVDomain,&sBBox));
      SmExtent3d sSphPoint(rSphOrigin);
      double dDist = sBBox.MinimumDistance(sSphPoint);

      // skip trimPlanes that don't intersect the sphere
      if (dDist > m_dRadius + dTol)
        { return SM_SUCCESS; }

      // get shpere bounding box
      SmExtent3d sBBox2;
      SER(CalculateBoundingBox(crUVDomain,&sBBox2));
      sBBox2.ExpandAbsolute(dTol);

      // skip planeBox/SphereBox cases that don't intersect
      if (sBBox2.AreDisjoint(sBBox))
        { return SM_SUCCESS; }
    } // end Plane is bounded check

  // See if we are intersecting at a singularity point
  if (dDistToPlane > m_dRadius - dTol)
    {
      // build degenerate curve
      SmBSplineCurve *pBSC = NULL ;
      SER(SmBSplineCurve::CreateDegenerateCurve(crContext,3,sProjectedPoint,pBSC));
      NER(pBSC);

      // set outputs
      if (pOpt3DCurves)         pOpt3DCurves->Add(pBSC);
      if (pOptSurface1UVCurves) pOptSurface1UVCurves->Add(NULL);
      if (pOptSurface2UVCurves) pOptSurface2UVCurves->Add(NULL);
      if (pOptCurveTypes)       pOptCurveTypes->Add(SM_TC_TOUCHING);
      if (pOptDeviations)       pOptDeviations->Add(smos_Fabs(dDistToPlane-m_dRadius));

      // all done
      return SM_SUCCESS;

    } // end degerenate curve intersection check

  // arrive here when plane intersects sphere in a circle

  // get intersection circle's geometry
  SmVector3d sVec    = sProjectedPoint - rSphOrigin;
  //double     dRadius = smos_Sqrt(m_dRadius * m_dRadius - sVec.LengthSquared() );

  // build circle center coordinate system from plane's coordinate system
  const SmVector3d &rXAxis = crPlane.GetPosition().GetXAxisRef();
  const SmVector3d &rYAxis = crPlane.GetPosition().GetYAxisRef();
  SmAxis2Placement sRefFrame;
  sRefFrame.SetCanonical(sProjectedPoint,rXAxis,rYAxis);

  // build circle arcs trimmed to Sphere boundary
  SmTArray<SmCurve *> s3dSphereCurves ;
  SmExtent1d sCircleIvl(0.0,360.0) ;
  SER(CreateTrimmedCircleSegments(crContext,
                                  sProjectedPoint, rXAxis, rYAxis,
                                  sCircleIvl, dTol,
                                  s3dSphereCurves)) ;

  //      // build a full closed circle
  //      SmBSplineCurve *pBSC = NULL ;
  //      SER(SmBSplineCurve::CreateCircleSegment(crContext,3,sRefFrame,dRadius,0.0,360.0,SM_CO_QUADRATIC,pBSC));
  //      SmObjDelete sClean(pBSC) ;
  //      NER(pBSC);

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
      smgfx_SetLook(2,6, 1,0,0) ; for(ii=0;ii<s3dSphereCurves.GetSize();ii++)
                                    { s3dSphereCurves[ii]->Draw() ; sm_GraphicsLoop() ; }
      sm_GraphicsLoop() ;

  }
#endif // SM_DEBUG_CODE

  // trim intersection curves to plane
  for(ii=0;ii<s3dSphereCurves.GetSize();ii++)
    {
      SmCurve *pCurve = s3dSphereCurves[ii] ;
      SmObjDelete sClean(pCurve) ;

      if(crPlane.IsBounded())
        {
          // trim curves to plane domain
          SmTArray<SmCurve *> s3dCurves ;
          SmObjsDelete<SmCurve *> sCleanArray(&s3dCurves) ;
          crPlane.TrimCurveToPlaneDomain(crOtherUVDomain, *pCurve, dTol, s3dCurves) ;

          // add trim curves to output
          if (pOpt3DCurves) { sCleanArray.Clear();
                              pOpt3DCurves->Append(s3dCurves);
                            }
        } // end trimmed plane check
      else // add whole curve to output
        {
          if (pOpt3DCurves) { sClean.Clear();
                              pOpt3DCurves->Add(pCurve);
                            }
        }
    } // end iter every intersection curve trimmed to just sphere bounds

  // set associated output for each trimmed intersection curve
  for (ii=0; ii<pOpt3DCurves->GetSize(); ii++)
    {
      if (pOptSurface1UVCurves) pOptSurface1UVCurves->Add(NULL);
      if (pOptSurface2UVCurves) pOptSurface2UVCurves->Add(NULL);
      if (pOptCurveTypes)       {  if(pOpt3DCurves->GetAt(ii)->IsDegenerate())
                                     { pOptCurveTypes->Add(SM_TC_TOUCHING); }
                                   else
                                     { pOptCurveTypes->Add(SM_TC_CROSSING); }
                                }
      if (pOptDeviations) pOptDeviations->Add(0.0);
    }

  // all done
  return SM_SUCCESS;

} // end SmSphere::IntersectWithPlane

/*******************************************************************//**
PURPOSE: Find intersection curve of this sphere with another sphere

NOTES:
***********************************************************************/
SmStatus SmSphere::IntersectWithSphere
  (const SmContext     & crContext,                // in : context for new object construction
   const SmExtent2d    & crUVDomain,               // in : intersection limit for this surface
   const SmSphere      & crSphere,                 // in : other target sphere
   const SmExtent2d    & crOtherUVDomain,          // in : intersection limit for target plane
   const SmBoolean    /* bUseSurfaceEdges */[2],   // in : TRUE = find xsect curve start points from boundaryCurve/surface xsects
                                                   //      typically these values are TRUE - its a small savings if you know
                                                   //      the boundaries of one surface don't intersect the other surface
   const SmApproxTol3d * pdOptApproxTol3d,         // in :
   const double *     /* pdOptAngTolRad*/ ,        // in :
   SmBoolean           & rbNeedsMoreIntersections, // out: TRUE = special case intersection failed-use general intersection
   SmTArray<SmCurve*>  * pOpt3DCurves,             // out: Intersection 3DCurves, NULL to ignore
   SmTArray<SmCurve*>  * pOptSurface1UVCurves,     // out: associated UVTrimCurves on this surface, NULL to ignore
   SmTArray<SmCurve*>  * pOptSurface2UVCurves,     // out: associated UVTrimCurves on plane, NULL to ignore
   SmTArray<SmTsectCurveType> * pOptCurveTypes,    // out: oneof for each 3DCurve, NULL to ignore
                                                   //      SM_TC_TOUCHING        - single point intersection (surf norms parallel)
                                                   //      SM_TC_CROSSING        - curve intersection (surf norms not parallel)
                                                   //      SM_TC_TANGENT         - curve intersection (surf norms parallel)
                                                   //      SM_TC_COINCIDENT      - curve intersection (surf norms parallel & cross-tangents equal)
                                                   //      SM_TC_NEAR_TANGENT    - curve has small angle of intersection
                                                   //      SM_TC_REGION_BOUNDARY - curve bounds region within which the surfs are coincident
   SmTArray<double>    * pOptDeviations)           // out: associated max 3DCurve to surface distance, NULL to ignore
  const
{
  // init output
  rbNeedsMoreIntersections = FALSE;
  if (pOpt3DCurves)         { pOpt3DCurves->ReSet(); }
  if (pOptSurface1UVCurves) { pOptSurface1UVCurves->ReSet(); }
  if (pOptSurface2UVCurves) { pOptSurface2UVCurves->ReSet(); }
  if (pOptCurveTypes)       { pOptCurveTypes->ReSet(); }
  if (pOptDeviations)       { pOptDeviations->ReSet(); }

  // select working tolerance
  double dTol = 0.0;
  if( pdOptApproxTol3d ) dTol = *pdOptApproxTol3d;
  else dTol = SM_EFF_ZERO_SQRT;

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  // draw
  if(bDebugMe)
    {
      SM_DUMP_AND_ASSERT_VALID(this) ;
      SM_DUMP_AND_ASSERT_VALID(&crSphere) ;
      SmFace *pFace1 = (SmFace *)GetFace() ;
      SmFace *pFace2 = (SmFace *)crSphere.GetFace() ;
      SmBrep *pBrep1 = pFace1 ? pFace1->GetBrep() : NULL ;
      SmBrep *pBrep2 = pFace2 ? pFace2->GetBrep() : NULL ;

      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep1) pBrep1->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,0) ; if(pBrep2) pBrep2->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,1) ; DrawUV() ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 1,1,0) ; crSphere.DrawUV() ; sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

  // locals
  ULONG ii ;
  const SmPoint3d &rCenter      = GetPosition().GetOriginRef();
  const SmPoint3d &rOtherCenter = crSphere.GetPosition().GetOriginRef();
  double           dRadius      = GetRadius();
  double           dOtherRadius = crSphere.GetRadius();

  // get center/center gap
  SmVector3d sVec  = rOtherCenter - rCenter;
  double     dDist = sVec.Length();

  // Check for no intersections - outside or inside one another
  if (   dDist - dTol > dRadius + dOtherRadius
      || dDist + dTol < smos_Fabs(dRadius-dOtherRadius))
    { return SM_SUCCESS;
    }

  // check bounding boxes to eliminate candidate intersections
  SmExtent3d sBBox1, sBBox2;
  SER(CalculateBoundingBox(crUVDomain,&sBBox1));
  SER(crSphere.CalculateBoundingBox(crOtherUVDomain,&sBBox2));
  sBBox2.ExpandAbsolute(dTol);

  // when bounding boxes don't intersect - no intersection
  if (sBBox1.AreDisjoint(sBBox2))
    {
      return SM_SUCCESS;
    }

  // check for coincident CenterPoints
  if (dDist < smos_Max(dTol,SM_EFF_ZERO))
    {
      if (smos_Fabs(dRadius-dOtherRadius) < dTol)
        {
          // Coincident sphere case. - need 1 sphere surface for every overlapping bit of domain
          rbNeedsMoreIntersections = TRUE;
          return SM_SUCCESS;
        }
      else // concentric spheres
        {
          // No intersection case
          return SM_SUCCESS;
        }
    } // end coincident CenterPoint check

  // Take care of touching situations
  if (   (dDist + dTol > dRadius + dOtherRadius)             // tangent outside
      || (dDist - dTol < smos_Fabs(dRadius-dOtherRadius)))   // tangent inside
    {
      double dSign = 1.0;
      // when touching inside
      if (dDist - dTol < smos_Fabs(dRadius-dOtherRadius))
        {
          // when OtherSphere is larger than thisSphere
          if (dRadius < dOtherRadius)
            {
              // let Vec run from OtherCenter to ThisCenter
              sVec = - sVec;
            }
        }
      else
        {
          dSign = -1.0;
        }

      // Touching case - return degenerate curve
      SER(sVec.Unitize());
      SmPoint3d sTouchPoint = rCenter + dRadius * sVec;
      SmPoint3d sOtherPoint = rOtherCenter + dSign * dOtherRadius * sVec;
      double dDistance = sTouchPoint.DistanceBetween(sOtherPoint);
      SmBSplineCurve *pBSC = NULL ;
      SER(SmBSplineCurve::CreateDegenerateCurve(crContext,3,sTouchPoint,pBSC));

      // gwc:need to check UVDomains

      // set output
      pOpt3DCurves->Add(pBSC);
      if (pOptSurface1UVCurves) { pOptSurface1UVCurves->Add(NULL); }
      if (pOptSurface2UVCurves) { pOptSurface2UVCurves->Add(NULL); }
      if (pOptCurveTypes)       { pOptCurveTypes->Add(SM_TC_TOUCHING); }
      if (pOptDeviations)       { pOptDeviations->Add( dDistance ); }

      // all done
      return SM_SUCCESS;

    } // end tangent point check

  // arrive here - there may be a circular intersection
  // build and return circle intersection trimmed to spheres

  // get circle size and position relative to 1st Sphere center and gap vector
  double dDistAlongBase, dHeight;
  SER(smgu_TriangleDecomposition(dDist, dRadius, dOtherRadius, dDistAlongBase, dHeight));
  SER(sVec.Unitize());
  SmPoint3d sCircleCent = rCenter + dDistAlongBase * sVec;

  // get orthogonal vectors:
  // let ZAxis = circle normal vector, the vector between sphere centers.
  //     X and Y Axis = any two vectors orthogonal to Z and each other
  SmVector3d sXAxis, sYAxis, sZAxis;
  SER(sVec.MakeUnitOrthoVectors(NULL,sZAxis,sXAxis,sYAxis));

  //      // create circle ref frame: the y vector marks the circle's start/end point
  //      SmAxis2Placement sRefFrame;
  //      sRefFrame.SetCanonical(sCircleCent,sXAxis,sYAxis);
  //      SmBSplineCurve *pBSC = NULL ;
  //
  //      // make the circle - trimmed to this sphere
  //      SER(SmBSplineCurve::CreateCircleSegment(crContext,3,sRefFrame,dHeight,0.0,360.0,SM_CO_QUADRATIC,pBSC));
  //      NER(pBSC);

  // build circle arcs trimmed to this Sphere boundary
  SmTArray<SmCurve *> s3dSphereCurves ;
  SmExtent1d sCircleIvl(0.0,360.0) ;
  SER(CreateTrimmedCircleSegments(crContext,
                                 sCircleCent, sXAxis, sYAxis,
                                 sCircleIvl, dTol,
                                 s3dSphereCurves)) ;

#ifdef SM_DEBUG_CODE
  // draw
  if(bDebugMe)
    {
      SM_DUMP_AND_ASSERT_VALID(this) ;
      SM_DUMP_AND_ASSERT_VALID(&crSphere) ;
      SmFace *pFace1 = (SmFace *)GetFace() ;
      SmFace *pFace2 = (SmFace *)crSphere.GetFace() ;
      SmBrep *pBrep1 = pFace1 ? pFace1->GetBrep() : NULL ;
      SmBrep *pBrep2 = pFace2 ? pFace2->GetBrep() : NULL ;

      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep1) pBrep1->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,0) ; if(pBrep2) pBrep2->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,1) ; DrawUV() ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 1,1,0) ; crSphere.DrawUV() ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 1,0,0) ; for(ii=0;ii<s3dSphereCurves.GetSize();ii++)
                                    { SmCurve *pCurve = s3dSphereCurves[ii] ;
                                      pCurve->Draw() ; sm_GraphicsLoop() ;
                                    }
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

  // trim intersection curves to Other Sphere
  for(ii=0;ii<s3dSphereCurves.GetSize();ii++)
    {
      // make this curve temporary
      SmCurve  *pCurve  = s3dSphereCurves[ii] ;
      SmObjDelete sClean(pCurve) ;

      // when curve is a circle (the other possibility is a degenerate curve)
      SmCircle *pCircle = SM_CAST_PTR(SmCircle, pCurve) ;
      if(pCircle)
        {
          // get Curve's analytic interval
          SmExtent1d sSTEPIvl = pCircle->GetSTEPInterval() ;

          // make circle segments trimmed to Other Sphere
          SmTArray<SmCurve *> sTrimmedCircles ;
          SER(crSphere.CreateTrimmedCircleSegments(crContext,
                                                   pCircle->GetPosition().GetOriginRef(),
                                                   pCircle->GetPosition().GetXAxisRef(),
                                                   pCircle->GetPosition().GetYAxisRef(),
                                                   sSTEPIvl,
                                                   dTol,
                                                   sTrimmedCircles)) ;

          // add trim curves to output
          if (pOpt3DCurves) { pOpt3DCurves->Append(sTrimmedCircles);
                            }
        } // end curve is a circle check
      else // curve is degenerate
        {
          // see if degenerateCurvePoint is inside of other surface
          SmPoint3d     sTestPoint ;
          SmPoint2d      sPointUV ;
          SmLocationType eLoc ;
          pCurve->EvaluatePoint(pCurve->GetNaturalInterval().GetMin(), sTestPoint) ;
          if(crSphere.IsClosedU() && crSphere.IsClosedV())
               { eLoc = SM_LT_INTERIOR ; }  // Every point is interior.
          else { crSphere.STEPInversion(crSphere.m_vAnalUVDomain,
                                        sTestPoint,
                                        SM_EFF_ZERO,
                                        sPointUV,
                                        eLoc) ;
               }

          // when degenerateCurvePoint is inside of other surface
          if(eLoc != SM_LT_EXTERIOR)
            {
              // make curve permanent and add it to the output
              if (pOpt3DCurves) { sClean.Clear() ;
                                  pOpt3DCurves->Add(pCurve);
                                }
            }
        } // end curve is degenerat branch
    } // end iter every intersection curve trimmed to just sphere bounds

#ifdef SM_DEBUG_CODE
  // draw
  if(bDebugMe)
    {
      SM_DUMP_AND_ASSERT_VALID(this) ;
      SM_DUMP_AND_ASSERT_VALID(&crSphere) ;
      SmFace *pFace1 = (SmFace *)GetFace() ;
      SmFace *pFace2 = (SmFace *)crSphere.GetFace() ;
      SmBrep *pBrep1 = pFace1 ? pFace1->GetBrep() : NULL ;
      SmBrep *pBrep2 = pFace2 ? pFace2->GetBrep() : NULL ;

      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep1) pBrep1->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,0) ; if(pBrep2) pBrep2->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,1) ; DrawUV() ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 1,1,0) ; crSphere.DrawUV() ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 1,0,0) ; for(ii=0;ii<pOpt3DCurves->GetSize();ii++)
                                    { SmCurve *pCurve = pOpt3DCurves->GetAt(ii) ;
                                      pCurve->Draw() ; sm_GraphicsLoop() ;
                                    }
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

  // set output
  for(ii=0; ii<pOpt3DCurves->GetSize(); ii++)
    {
      if (pOptSurface1UVCurves) { pOptSurface1UVCurves->Add(NULL); }
      if (pOptSurface2UVCurves) { pOptSurface2UVCurves->Add(NULL); }
      if (pOptCurveTypes)       {  if(pOpt3DCurves->GetAt(ii)->IsDegenerate())
                                     { pOptCurveTypes->Add(SM_TC_TOUCHING); }
                                   else
                                     { pOptCurveTypes->Add(SM_TC_CROSSING); }
                                }
      if (pOptDeviations)       { pOptDeviations->Add(0.0); }
    }

  // all done
  return SM_SUCCESS;

} // end SmSphere::IntersectWithSphere

/*******************************************************************//**
PURPOSE: Create Trimmed Circular arc segments that are within
            the Sphere boundaries for circle on the sphere.

NOTES:
***********************************************************************/
SmStatus SmSphere::CreateTrimmedCircleSegments
  (const SmContext  &crContext,      // in : new object context
   const SmPoint3d  &crCircleCenter, // in : center of circle
   const SmVector3d &crCircleXAxis,  // in : xAxis of circle
   const SmVector3d &crCircleYAxis,  // in : YAxis of Circle
   const SmExtent1d &crCircleIvl,    // in : Circle domain in Degrees (0 = point on xAxis, 90.0 = point on YAxis)
   double            dTol3d,         // in : max distance between coincident points
   SmTArray<SmCurve *> &r3dCurves)   // out: array of trimmed circle arcs
  const
{
  // init output
  r3dCurves.ReSet() ;

  // circle locals
  SmAxis2Placement sCirclePlacement(crCircleCenter, crCircleXAxis, crCircleYAxis) ;
  SmVector3d sCircleNorm     = crCircleXAxis * crCircleYAxis ;
  SmVector3d sCircleOffVec   = crCircleCenter - m_vPosition.GetOriginRef() ;
  double     dCircleOffDist2 = sCircleOffVec.LengthSquared() ;
  double     dCircleRadius2  = m_dRadius*m_dRadius - dCircleOffDist2 ;
  double     dCircleRadius   = smos_Sqrt(dCircleRadius2) ;
  double     dScaledZero     = SM_EFF_ZERO * (1.0 + m_dRadius) ;
  double     dCircleMinDeg   = crCircleIvl.GetMin() ;
  double     dCircleMaxDeg   = crCircleIvl.GetMax() ;
  SmBoolean  bCircleOpen     = FALSE ;

  // Sphere locals
  const SmVector3d &rSphereOrigin  = m_vPosition.GetOriginRef() ;
  const SmVector3d &rSphereXAxis   = m_vPosition.GetXAxisRef() ;
  const SmVector3d &rSphereYAxis   = m_vPosition.GetYAxisRef() ;
  SmVector3d        sSphereZAxis   = m_vPosition.GetZAxis() ;
  SmBoolean         bSphereClosedU = IsClosedU() ;
  SmBoolean         bSphereClosedV = IsClosedV() ;
 // double            dTol2d         = dTol3d*180.0/SM_PI/m_dRadius ;

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  // draw input sphere and circle
  if(bDebugMe)
    {
      SmFace *pFace = (SmFace *)GetFace() ;
      SmBrep *pBrep = pFace ? pFace->GetBrep() : NULL ;
      SmCircle sCircle(crCircleCenter, crCircleXAxis, crCircleYAxis,
                       SmExtent1d(0.0, 360.0), dCircleRadius, 3,
                       &crContext) ;

      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,1) ; DrawUV() ; sm_GraphicsLoop() ;
      smgfx_SetLook(3,4, 1,0,0) ; sCircle.Draw() ; sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

  // check input
  if(dCircleRadius < -dScaledZero)
    { SER(SM_ERR_INVALID_INPUT) ; }

  // handle degenerate circles
  if(dCircleRadius < dScaledZero)
    {
      // Point is contained when sphere is closed or when
      // Point is contained within the trimmed sphere boundary
      SmPoint3d sTestPoint = crCircleCenter + dCircleRadius * crCircleXAxis ;
      SmPoint2d sUVPoint ;
      SmLocationType eLocation ;
      SmBoolean bIsContained ;
      if (bSphereClosedU && bSphereClosedV)
           { bIsContained = TRUE ; }
      else { STEPInversion(m_vAnalUVDomain,
                           sTestPoint,
                           SM_EFF_ZERO,
                           sUVPoint,
                           eLocation) ;
              bIsContained =   (eLocation != SM_LT_EXTERIOR)
                             ? TRUE
                             : FALSE ;
           }

      // When TestPoint is in Sphere analUVDomain - the curve is inside
      if(bIsContained)
        {
          // Add 1 degenerate curve to the output
          SmBSplineCurve *pNewCurve = NULL ;
          SER(SmBSplineCurve::CreateDegenerateCurve(crContext, 3, sTestPoint, pNewCurve)) ;
          r3dCurves.Add(pNewCurve) ;

        } // end circle is inside sphere domain check

      // all done
      return(SM_SUCCESS) ;
    } // end degenerate circle check

#ifdef SM_DEBUG_CODE
  SM_ASSERT(   sCircleOffVec.Length() < dScaledZero
            || sCircleNorm.IsParallelTo(sCircleOffVec,1.0)) ;
#endif // SM_DEBUG_CODE

  // get UVBoundaries
  double dMinU = m_vAnalUVDomain.GetMin().x ;
  double dMaxU = m_vAnalUVDomain.GetMax().x ;
  double dMinV = m_vAnalUVDomain.GetMin().y ;
  double dMaxV = m_vAnalUVDomain.GetMax().y ;

  // local array of circle end and intersection point angles
  ULONG  ii, jj, lAngCount = 0 ;
  double    adAngDegs[10] ;
  SmBoolean abArcInside[10] ;  // each arcsegment[ii] ends at each adAngDegs[ii]

  // add endPoint Angles when circle is open
  if((360.0-crCircleIvl.GetLength())*dCircleRadius > dTol3d)
    {
      lAngCount    = 2 ;
      adAngDegs[0] = dCircleMinDeg ;
      adAngDegs[1] = dCircleMaxDeg ;
      bCircleOpen  = TRUE ;
    }

  // for every boundary - get angles of all boundary/circle intersections
  for(ii=0;ii<4;ii++)
    {
      // skip boundaries in closed sphere directions
      if(   ((ii == 0 || ii == 1) && bSphereClosedV)
         || ((ii == 2 || ii == 3) && bSphereClosedU))
        { continue ; }

      SmPoint3d sCenter, sX, sY ;
      // double dRadius, dStartAngDeg, dEndAngDeg ;

      // get angle (radians) for this boundary
      double dAngDeg =   ii == 0 ? dMaxV
                       : ii == 1 ? dMinV
                       : ii == 2 ? dMaxU
                       :           dMinU ;
      double dAngRad = dAngDeg*SM_PI/180 ;

      // get circle boundary arc description
      if(ii == 0 || ii == 1) { // min/max V boundaries - Varying U isoParam Circles
                               sCenter = rSphereOrigin + m_dRadius * sSphereZAxis * smos_Sine(dAngRad) ;
                               //dRadius = m_dRadius * smos_Cosine(dAngRad) ;
                               sX      = rSphereXAxis ;
                               sY      = rSphereYAxis ;
                               //dStartAngDeg = dMinU ;
                               //dEndAngDeg   = dMaxU ;
                             }
      else                   { // min/max U boundries - Varying V isoParam Circles
                               sCenter = rSphereOrigin ;
                               //dRadius = m_dRadius ;
                               sX      =   smos_Cosine(dAngRad) * rSphereXAxis
                                         + smos_Sine  (dAngRad) * rSphereYAxis ;
                               sY      = sSphereZAxis ;
                               //dStartAngDeg = dMinV ;
                               //dEndAngDeg   = dMaxV ;
                             }

      // get SphereCircle/SphereCircle intersections
      ULONG lIntersectCount ;
      SmPoint3d aPoints[2] ;
      smgu_SphereCircleSphereCircleIntersect(rSphereOrigin,
                                             m_dRadius,
                                             crCircleCenter,  // circ1 - circle being trimmed
                                             sCircleNorm,     // circ1
                                             sCenter,         // circ2 - sphere trim boundary
                                             sX * sY,         // circ2
                                             dTol3d,
                                             lIntersectCount,
                                             aPoints) ;

      // No need to add angles for coincident cases
      // trim limits will be found with other boundary/circle intersections
      if(lIntersectCount == 3)
        { continue ; }

      // add every intersection point angle to ordered list
      for(jj=0;jj<lIntersectCount;jj++)
        {
          // get new circle angle for circle point
          SmVector3d sCirVec   = aPoints[jj] - crCircleCenter ;
          double     dX        = sCirVec.Dot(crCircleXAxis) ;
          double     dY        = sCirVec.Dot(crCircleYAxis) ;
          double     dAngleDeg = smos_ArcTangent2(dY, dX) * 180.0 / SM_PI ;

          // accumulate angles
          adAngDegs[lAngCount] = dAngleDeg ;
          lAngCount++ ;

        } // end iter every intersection

    } // end iter every sphere boundary


  // arrive here when circle must be trimmed into 1 or more circular arcs.
  // CircleAngles for all Circle/SphereBoundary intersections
  // are in the array adAngDegs.

  // move all angles into given periodic range
  for(ii=0;ii<lAngCount;ii++)
    {
      if     (adAngDegs[ii] < dCircleMinDeg) adAngDegs[ii] += 360.0 ;
      else if(adAngDegs[ii] > dCircleMaxDeg) adAngDegs[ii] -= 360.0 ;

    } // end iter every angle adjusting for periodicity

  // sort the adAngDegs values
  smgu_ShellSort_double_array(adAngDegs, lAngCount);

  // remove duplicate and out of range values from adAngDegs
  ULONG lTgt = 0 ;
  for(ii=0;ii<lAngCount;ii++)
    {
      // skip out of bound values
      if(   adAngDegs[ii] < dCircleMinDeg
         || adAngDegs[ii] > dCircleMaxDeg)
        { continue ; }

      // skip repeat values, if prev was within range [my_offset_demo iter 13]
      if (   ii > 0
          && adAngDegs[ii-1] >= dCircleMinDeg
          && SM_ARE_SAME( adAngDegs[ii], adAngDegs[ii-1] ) )
        { continue ; }

      // copy current value into compressed array
      adAngDegs[lTgt] = adAngDegs[ii] ;
      lTgt++ ;
    } // end iter every angle removing duplicates from the array

  // set compressed adAngDegs array size
  lAngCount = lTgt ;

  // Now adAngDegs[] is sorted and has no duplicate angles.

  // If there are no SphereBoundary/Circle intersections and circle is closed,
  // then the entire circle is either in or out; no splitting is necessary.
  // Note, that's also the case if there is exacatly one intersection. [B194]
  if ( lAngCount == 0 || lAngCount == 1 )
    {
      // circle is contained for closed spheres or when
      // any point on the circle is contained within the trimmed sphere boundary

      SmPoint2d sUVPoint ;
      SmLocationType eLocation ;
      SmBoolean bIsContained ;
      if ( bSphereClosedU && bSphereClosedV )
        { bIsContained = TRUE ; }
      else
        {
          // If not closed, need to check whether the whole circle is in or out.
          // Test one point on the circle.
          SmPoint3d sTestPoint = crCircleCenter + dCircleRadius * crCircleXAxis ;

          // If there is one intersection, then one point will be on the boundary
          // whether the whole circle is inside or out.  Make sure that the
          // point we select is not the inetersection point.
          if ( lAngCount == 1 )
            {
              // adAngDegs[0] is the intersection.
              double dAng = adAngDegs[0] + 42.0; // random...
              dAng = SM_DEG2RAD( dAng );
              sTestPoint = crCircleCenter
                           + dCircleRadius * (  smos_Cosine( dAng ) * crCircleXAxis
                                              + smos_Sine  ( dAng ) * crCircleYAxis );
            }

          // Test the point. Tol loosened [B661]
          STEPInversion(m_vAnalUVDomain,
                           sTestPoint,
                           SM_EFF_ZERO_PARAM,
                           sUVPoint,
                           eLocation) ;
          bIsContained =   (eLocation != SM_LT_EXTERIOR)
                         ? TRUE
                         : FALSE ;
        }

      // When TestPoint is in Sphere analUVDomain - the curve is inside
      if(bIsContained)
        {
          // Add 1 full circle to the output
          SmCircle *pNewCircle = NULL ;
          SER(SmCircle::CreateCanonical(crContext, sCirclePlacement, dCircleRadius, pNewCircle)) ;
          r3dCurves.Add(pNewCircle) ;

        } // end circle is inside sphere domain check

      // all done
      return(SM_SUCCESS) ;

    } // end whole circle is either inside or outside of sphere domain


  // for every circle segment - classify in/out and make circular arcs as appropriate
  for(ii=0;ii<lAngCount;ii++)
    {
      // get arc angle limits
      double dEndAngle   = adAngDegs[ii] ;
      double dStartAngle = (ii == 0)
                           ? (  (adAngDegs[lAngCount-1] > dEndAngle)
                              ? adAngDegs[lAngCount-1] - 360.0
                              : adAngDegs[lAngCount-1])
                           : adAngDegs[ii-1] ;

      // when circle is open - the first interval is always False
      if(ii == 0 && bCircleOpen)
        {
          abArcInside[ii] = FALSE ;
          continue ;
        }

      // arc angle (watch out for arcs that span periodic boundary)
      double dArcAngleDeg =  (dEndAngle < dStartAngle)
                         ? dEndAngle - dStartAngle + 360.0
                         : dEndAngle - dStartAngle ;

      // get MidArcPoint
      double    dMidAngleRad = (dStartAngle + dArcAngleDeg/2.0) * SM_PI / 180.0 ;
      SmPoint3d sMidArcPoint =   crCircleCenter
                               + dCircleRadius * (  smos_Cosine(dMidAngleRad) * crCircleXAxis
                                                  + smos_Sine  (dMidAngleRad) * crCircleYAxis) ;

#ifdef SM_DEBUG_CODE
  // draw input sphere and three points
  if(bDebugMe)
    {
      SmPoint3d sStartPt = crCircleCenter + dCircleRadius *
          (  smos_Cosine(dStartAngle*SM_PI/180.0) * crCircleXAxis
           + smos_Sine  (dStartAngle*SM_PI/180.0) * crCircleYAxis);
      SmPoint3d sEndPt = crCircleCenter + dCircleRadius *
          (  smos_Cosine(dEndAngle*SM_PI/180.0) * crCircleXAxis
           + smos_Sine  (dEndAngle*SM_PI/180.0) * crCircleYAxis);
      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0,1,1); this->DrawUV(); sm_GraphicsLoop();
      smgfx_SetLook(3,4, 0,1,0); sStartPt.    Draw(); sm_GraphicsLoop();
      smgfx_SetLook(3,4, 0,0,1); sEndPt.      Draw(); sm_GraphicsLoop();
      smgfx_SetLook(5,4, 1,0,0); sMidArcPoint.Draw(); sm_GraphicsLoop();
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

      // when circle arcLength is shorter than tolerance
      if( SM_DEG2RAD(dArcAngleDeg) * dCircleRadius < dTol3d)
        {
          abArcInside[ii] = TRUE ;

          // output a degenerate curve at the arcCenter
          SmBSplineCurve *pNewBSP = NULL ;
          SER(SmBSplineCurve::CreateDegenerateCurve(*m_cpContext, 3,
                                                    sMidArcPoint,
                                                    pNewBSP));
          r3dCurves.Add(pNewBSP);
        } // end degenerate curve branch
      else // classify nonDegenerate circular arc branch
        {
          // check arc midPoint to see if arc is inside or outside
          // project the point into Spherical position Coordinates
          SmPoint2d sUVPoint ;
          SmLocationType eLoc ;
          STEPInversion(m_vAnalUVDomain, sMidArcPoint, dTol3d, sUVPoint, eLoc) ;

          // when test point is on Surface and within domain boundaries
          if(eLoc != SM_LT_EXTERIOR)
            {
              // remember arc is inside
              abArcInside[ii] = TRUE ;

              // Add trimmed circular arc to the output
              SmCircle *pNewCircle = NULL ;
              SmExtent1d sCircleIvl(dStartAngle, dEndAngle) ;
              SER(SmCircle::CreateCanonical(crContext,  sCirclePlacement, dCircleRadius,
                                            pNewCircle, &sCircleIvl)) ;
              r3dCurves.Add(pNewCircle) ;
            } // end circle is inside sphere domain check
          else
            {
              // remember arc is outside
              abArcInside[ii] = FALSE ;
            }
        } // end possibly output a circular arc branch
    } // end iter every circular arc segment

  // arrive here - need to add degenerate curves for all tangent intersections

  // for every intersection arcAngle - see if it represents a tangent intersection
  for(ii=0;ii<lAngCount;ii++)
    {
      // get upper and lower arc inside/outside classifications
      SmBoolean bLowerArcInside = abArcInside[ii] ;
      SmBoolean bUpperArcInside = ii == lAngCount - 1
                                  ? abArcInside[0]
                                  : abArcInside[ii+1] ;

      // skip points attached to inside intervals
      if(bLowerArcInside || bUpperArcInside)
        { continue ; }

      // arrive here for all tangent intersection points
      // and tangent/outside OpenCircle endPoints - these last have to be checked

      // get the ArcPoint for this angle
      double dArcAngRad   = adAngDegs[ii] * SM_PI/180.0 ;
      SmPoint3d sArcPoint =  crCircleCenter
                           + dCircleRadius * (  smos_Cosine(dArcAngRad) * crCircleXAxis
                                              + smos_Sine(dArcAngRad)   * crCircleYAxis) ;

      // when point is an openCircle endPoint - check point for tangent intersection
      // Actually, check all circle-circle intersections.
      // There will be some outside of the surface patch.
//      if(   bCircleOpen
//         && (   adAngDegs[ii] == dCircleMinDeg
//             || adAngDegs[ii] == dCircleMaxDeg))
//        {
          // get spherical coordinates for ArcPoint
          SmPoint2d sUVPoint;
          SmLocationType eLoc;
          STEPInversion(m_vAnalUVDomain, sArcPoint, dTol3d,sUVPoint, eLoc );

          // skip points not within sphere boundaries
          if(eLoc == SM_LT_EXTERIOR)
            { continue; }

//        } // end Open circle endPoint check

      // mark the tangency with a degenerate curve
      SmBSplineCurve *pNewBSP = NULL ;
      SER(SmBSplineCurve::CreateDegenerateCurve(*m_cpContext, 3,
                                                sArcPoint,
                                                pNewBSP));
      r3dCurves.Add(pNewBSP);

    } // end iter every arcAngle

#ifdef SM_DEBUG_CODE
  // draw input sphere and circle
  if(bDebugMe)
    {
      SmFace *pFace = (SmFace *)GetFace() ;
      SmBrep *pBrep = pFace ? pFace->GetBrep() : NULL ;

      ULONG lCount = r3dCurves.GetSize() ;
      for(ii=0;ii<lCount;ii++)
        {
          SmCurve *pCurve = r3dCurves[ii] ;
          double   dCInc  =    lCount > 1
                             ? (double)ii/(double)(lCount-1)
                             : 0.0 ;

          smgfx_Erase() ;
          smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
          smgfx_SetLook(1,2, 0,1,1) ; DrawUV() ; sm_GraphicsLoop() ;
          smgfx_SetLook(3,4, 1,dCInc,1.0-dCInc) ;  pCurve->Draw() ; sm_GraphicsLoop() ;
        }
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

  // all done
  return(SM_SUCCESS) ;

} // end SmSphere::CreateTrimmedCircleSegments

/*******************************************************************//**
PURPOSE: Find intersection curve of this sphere with a cone.

NOTES: Finds and returns intersections when Sphere Center is on the
   cone axis.  Else sets rbNeedsMoreIntersections = TRUE and returns.
***********************************************************************/
SmStatus SmSphere::IntersectWithCone
  (const SmContext     & crContext,                  // in : context for new object construction
   const SmExtent2d    & crUVDomain,                 // in : in range [-360_to_360 max length 360, -90_to_90]
   const SmCone        & crCone,                     // in :
   const SmExtent2d    & crOtherUVDomain,            // in : intersection limit for target plane
   const SmBoolean       /* bUseSurfaceEdges */[2],  // in : TRUE = find xsect curve start points from boundaryCurve/surface xsects
                                                     //      typically these values are TRUE - its a small savings if you know
                                                     //      the boundaries of one surface don't intersect the other surface
   const SmApproxTol3d * pdOptApproxTol3d,           // in : max distance between coincident points
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
  // check input
  if (!pOpt3DCurves)        SER(SM_ERR);  // Always need 3d curves for analytical intersections

  // init output
  rbNeedsMoreIntersections = FALSE;
  if (pOpt3DCurves)         pOpt3DCurves->ReSet();
  if (pOptSurface1UVCurves) pOptSurface1UVCurves->ReSet();
  if (pOptSurface2UVCurves) pOptSurface2UVCurves->ReSet();
  if (pOptCurveTypes)       pOptCurveTypes->ReSet();
  if (pOptDeviations)       pOptDeviations->ReSet();

  // local tolerance
  double dTol = 0.0;
  if( pdOptApproxTol3d ) dTol =  *pdOptApproxTol3d;
  else dTol = SM_EFF_ZERO_SQRT ;

  // The best way to go about solving this problem is to
  // do everything in a plane and then put it back into
  // 3D.
  const SmPoint3d        & rSphCent  = m_vPosition.GetOriginRef();
  const SmAxis2Placement & crConePos = crCone.GetPosition();
  const SmPoint3d        & rConeOrig = crConePos.GetOriginRef();
  SmVector3d               sConeAxis = crConePos.GetZAxis();

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  // draw
  if(bDebugMe)
    {
      SmFace *pFace1 = (SmFace *)GetFace() ;
      SmFace *pFace2 = (SmFace *)crCone.GetFace() ;
      SmBrep *pBrep1 = pFace1 ? pFace1->GetBrep() : NULL ;
      SmBrep *pBrep2 = pFace2 ? pFace2->GetBrep() : NULL ;

      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep1) pBrep1->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep2) pBrep2->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,0,1) ; DrawUV() ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,0) ; crCone.DrawUV() ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,0,0) ; if(pFace1) pFace1->Draw(SM_DM_CROSSHATCH) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,0,0) ; if(pFace2) pFace2->Draw(SM_DM_CROSSHATCH) ; sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

  // Find the sphere center line/cone axis distance
  double dParam;
  SER(smgu_LineClosestPoint(rConeOrig,sConeAxis,rSphCent,dParam));
  SmPoint3d sClosestPt = rConeOrig + dParam * sConeAxis;
  double    dDist      = sClosestPt.DistanceBetween(rSphCent);

  // Is Sphere too far away for intersetions
  if (dDist-dTol > m_dRadius + smos_Max(crCone.GetBotRadius(),crCone.GetTopRadius()))
    {
      // Sphere is too far away from cone
      return SM_SUCCESS;
    }

  // Is sphere inside of cone and no intersections
  if (dDist+dTol+m_dRadius < smos_Min(crCone.GetBotRadius(),crCone.GetTopRadius()))
    {
      return SM_SUCCESS;
    }

  // If the sphere center is not on the axis of the cone
  // then just use the traditional intersector.
  if (dDist > dTol)
    {
      rbNeedsMoreIntersections = TRUE;
      return SM_SUCCESS;
    }

  // eliminate intersections using bounding boxes.
  SmExtent3d sBBox1, sBBox2;
  SER(CalculateBoundingBox(crUVDomain,&sBBox1));
  SER(crCone.CalculateBoundingBox(crOtherUVDomain,&sBBox2));
  sBBox2.ExpandAbsolute(dTol);
  if (sBBox1.AreDisjoint(sBBox2))
    {
      return SM_SUCCESS;
    }

  // arrive here the sphere is on the center of the
  // axis of the cylinder.  We can compute analytical intersections.

  // Get the line segment that runs along a straight-line boundary of the cone.
  // (This will be the seam, if it's closed.)
  // We will intersect that with the sphere.
  SmPoint2d sUV( crOtherUVDomain.Evaluate( 0, 0 ) ); // Lower-left corner of domain.
  SmPoint3d sLineStart, sLineEnd;
  crCone.EvaluatePoint( sUV, sLineStart );
  // Upper-left corner of analytical domain:
  sUV = ( crCone.GetSwapUV() ) ? crOtherUVDomain.Evaluate( 1, 0 ) : crOtherUVDomain.Evaluate( 0, 1 );
  crCone.EvaluatePoint( sUV, sLineEnd );
  SmVector3d sLineVec( sLineEnd - sLineStart );

#ifdef SM_DEBUG_CODE
  if (bDebugMe)
    {
      SmFace *pFace = (SmFace *)GetFace() ;
      SmBrep *pBrep = pFace ? pFace->GetBrep() : NULL ;

      smgfx_Erase();
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,0,1) ; DrawUV(); sm_GraphicsLoop();
      smgfx_SetLook(1,2, 0,1,0) ; crCone.DrawUV(); sm_GraphicsLoop();
      smgfx_SetLook(4,5, 1,0,0) ; rConeOrig.Draw(); sm_GraphicsLoop();
      SmPoint3d sCenterTop = rConeOrig + crCone.GetHeight() * sConeAxis;
      smgfx_SetLook(4,5, 1,0,0) ; sCenterTop.Draw(); sm_GraphicsLoop();
      smgfx_SetLook(4,5, 0,1,1) ; sLineStart.Draw(); sm_GraphicsLoop();
      smgfx_SetLook(5,5, 0,1,1) ; sLineEnd.Draw(); sm_GraphicsLoop();
      smgfx_SetLook(2,3, 1,0,1) ; sLineVec.Draw(&sLineStart); sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
#endif

  // To intersect the cone's line segment with the sphere, since the
  // sphere center is on the cone's axis, we can just intersect the
  // line segment with the great circle of the sphere that is in the
  // plane of the axis and the cone's line segment.
  // Its normal would be the cross product of that on-cone line
  // and the axis -- if it's not parallel (cylinder).  In that case, we could
  // use the vector from any point on the axis to any point on the line segment.
  // So let's just use the vector from the origin to the midpoint of that
  // line segment, which cannot be on the axis. [cf. B304 B194]
  SmPoint3d sMidPt( ( sLineStart + sLineEnd ) / 2 );
  SmVector3d sVec( sMidPt - rConeOrig );
  SmVector3d sCircleNorm = sVec * sConeAxis;
  double dLength = sCircleNorm.Length();
  if ( dLength < SM_EFF_ZERO )
    { SER( SM_ERR ); } // Should never happen.
  sCircleNorm /= dLength;

  ULONG  lNumInt;
  double adLineInt[2];
  SER(smgu_LineCoPlanarCircleIntersect(sLineStart, sLineVec, rSphCent, sCircleNorm,
                                       m_dRadius, dTol,
                                       lNumInt, adLineInt));

  // for every ConeLine/SphereCircle intersection
  for (ULONG i=0; i<lNumInt; i++)
    {
      // intersection point
      SmPoint3d sPnt = sLineStart + adLineInt[i] * sLineVec;

#ifdef SM_DEBUG_CODE
      if (bDebugMe)
        {
          SmSolutionArray sSolutions;
          SER(this->GlobalPointSolve(this->GetNaturalUVDomain(),SM_SO_MINIMIZE,sPnt,
              dTol,NULL,SM_SR_ALL,sSolutions));
          SmPoint3d sSphPnt;
          if (sSolutions.GetSize() > 0)
            {
              SmPoint2d sUVSol(sSolutions[0].m_vStart[0],sSolutions[0].m_vStart[1]);
              this->EvaluatePoint(sUVSol,sSphPnt);
            }
          SmFace *pFace = (SmFace *)GetFace() ;
          SmBrep *pBrep = pFace ? pFace->GetBrep() : NULL ;

          smgfx_Erase();
          smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
          smgfx_SetLook(1,2, 0,1,1) ; DrawUV(); sm_GraphicsLoop();
          smgfx_SetLook(1,2, 1,1,0) ; crCone.DrawUV(); sm_GraphicsLoop();

          smgfx_SetLook(4,5, 1,0,0) ; sPnt.Draw(); sm_GraphicsLoop();
          smgfx_SetLook(4,5, 0,1,0) ; sSphPnt.Draw(); sm_GraphicsLoop();
          smgfx_SetLook(4,5, 0,1,0) ; sLineStart.Draw(); sm_GraphicsLoop();
          smgfx_SetLook(5,5, 0,1,0) ; sLineEnd.Draw(); sm_GraphicsLoop();
          smgfx_SetLook(2,3, 1,0,1) ; sLineVec.Draw(&sLineStart); sm_GraphicsLoop();
          sm_GraphicsLoop();

        }
#endif
      // skip intersections outside of cone upper/lower limits
      if (adLineInt[i] > 1.0)
        {
          double dDistance = sLineEnd.DistanceBetween(rSphCent);
          if (smos_Fabs( dDistance - m_dRadius) > dTol)
            {
              continue;
            }
          adLineInt[i] = 1.0; // Truncate to start of cylinder.
        }
      else if (adLineInt[i] < 0.0)
        {
          double dDistance = sLineStart.DistanceBetween(rSphCent);
          if (smos_Fabs( dDistance - m_dRadius) > dTol)
            {
              continue;
            }
          adLineInt[i] = 0.0; // Trucate to start of cylinder.
        }

      // build cone circle for intersection point
      SmBSplineCurve *pIsoCrv = NULL ;
      SER(crCone.CreateIsoCircleFromPoint(crContext,sPnt,dTol,pIsoCrv));
      SmObjDelete sClean(pIsoCrv) ;

      if(pIsoCrv && pIsoCrv->IsKindOf(SmCircle_TYPE))
        {
          SmCircle *pCircle = (SmCircle *)pIsoCrv ;

          // get Curve's analytic interval
          SmExtent1d sSTEPIvl = pCircle->GetSTEPInterval() ;
          const SmAxis2Placement &rCirclePosition = pCircle->GetPosition() ;

          // trim circle segment to Sphere
          SmTArray<SmCurve *> sTrimmedCircles ;
          SER(CreateTrimmedCircleSegments(crContext,
                                          rCirclePosition.GetOriginRef(),
                                          rCirclePosition.GetXAxisRef(),
                                          rCirclePosition.GetYAxisRef(),
                                          sSTEPIvl, dTol,
                                          sTrimmedCircles)) ;

          // add trim curves to output
          if (pOpt3DCurves) { pOpt3DCurves->Append(sTrimmedCircles);
                            }
        }
      else if(pIsoCrv) // curve is degenerate
        {
          SM_ASSERT(pIsoCrv->IsDegenerate()) ;
          SM_ASSERT(pIsoCrv->GetNaturalInterval().GetMin() == 0.0) ;
          SmPoint3d sPoint ;
          pIsoCrv->EvaluatePoint(0.0, sPoint) ;
          double dScaledZero = SM_EFF_ZERO * (1.0 + sPoint.GetMaxDimension()) ;

          // get UV point for Sphere
          SmPoint2d sUVPoint ;
          SmLocationType eLoc ;
          STEPInversion(m_vAnalUVDomain, sPoint, dScaledZero, sUVPoint, eLoc) ;

          // add degenerate curve to output
          if(   eLoc != SM_LT_EXTERIOR
             && pOpt3DCurves)
            { pOpt3DCurves->Add(pIsoCrv) ;
              sClean.Clear() ;
            }
        } // end curve is degenerate branch

    } // end every coneLine/Sphere intersection

  // set associated output for each trimmed intersection curve
  ULONG ii ;
  for(ii=0;ii<pOpt3DCurves->GetSize();ii++)
    {
      if (pOptSurface1UVCurves) pOptSurface1UVCurves->Add(NULL);
      if (pOptSurface2UVCurves) pOptSurface2UVCurves->Add(NULL);
      if (pOptCurveTypes)       {  if(pOpt3DCurves->GetAt(ii)->IsDegenerate())
                                     { pOptCurveTypes->Add(SM_TC_TOUCHING); }
                                   else
                                     { pOptCurveTypes->Add(SM_TC_CROSSING); }
                                }
      if (pOptDeviations) pOptDeviations->Add(0.0);
    }

  // all done
  return SM_SUCCESS;

} // end SmSphere::IntersectWithCone

/*******************************************************************//**
PURPOSE: Static function to determine if a given nurbs is a Sphere.
            If it is a Sphere then one is created.

NOTES:
***********************************************************************/
SmBoolean SmSphere::IsNurbSurfaceSphere
  (const SmContext        & crContext,       // in : new object context
   const SmBSplineSurface * pTestSurface,    // in : BSplineSurface to be checked
   SmSphere              *& rpSphere,        // out: new Sphere or NULL
   double                   dToleranceScale) // in : extra scale value for ScaledZero computation
                                             //      default:[1.0]
{
  SM_DUMP_AND_ASSERT2_VALID(pTestSurface) ;

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
static ULONG lCount = 1 ; lCount++ ;
static ULONG lDebugCount = 0 ;
  if (bDebugMe || lDebugCount == lCount)
    {
      pTestSurface->Dump() ;

      // just add graphics to interface to stop drawing recursion
      SmFace *pFace = (SmFace *)pTestSurface->GetFace() ;
      //SmBrep *pBrep = pFace ? pFace->GetBrep() : NULL ; SM_DUMP_AND_ASSERT_VALID(pBrep) ;

      // smgfx_Erase() ;
      // smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,1) ; if(pTestSurface) pTestSurface->DrawUV(7,7) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,0,0) ; if(pFace) pFace->Draw(SM_DM_CROSSHATCH) ; sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

  // no work - not a degree 2 rational surface in both directions
  ULONG lDegU = pTestSurface->GetDegree(SM_SP_U);
  ULONG lDegV = pTestSurface->GetDegree(SM_SP_V);
  if (   lDegU != 2
      || lDegV != 2
      || !pTestSurface->IsRational())
    { return FALSE; }

  // evaluate min/max domain points
  SmExtent2d sNurbUVDomain = pTestSurface->GetNaturalUVDomain();
  SmPoint3d sMinPnt, sMaxPnt;
  pTestSurface->EvaluatePoint(sNurbUVDomain.GetMin(),sMinPnt);
  pTestSurface->EvaluatePoint(sNurbUVDomain.GetMax(),sMaxPnt);

  // scale zero value
  double dScaledZero =   dToleranceScale
                       * ANALYTIC_TOL_SCALE * SM_EFF_ZERO
                       * (1.0 + sMinPnt.GetMaxDimension() + sMaxPnt.GetMaxDimension());
  double dAngTol     = SM_EFF_ZERO_SQRT * dToleranceScale;

  // locals
  SmBoolean bFoundCirclesU, bParallelPlanesU, bSamePlaneU, bSameCenterU, bCommonAxisU, bPlanarGenCurveU ;
  SmBoolean bFoundCirclesV, bParallelPlanesV, bSamePlaneV, bSameCenterV, bCommonAxisV, bPlanarGenCurveV ;
  SmExtent1d sAnglesU, sAnglesV;
  double dRadiusU, dRadiusV;
  SmAxis2Placement sRefFrameU, sRefFrameV;
  SmPoint3d sCommonPoint, sCommonVec ;

  // test sequence of V Direction (U Constant) isoCurves for circularity and parallelism
  SER(sm_TestForIsoCircles(pTestSurface, SM_SP_U, dScaledZero, dAngTol,
                              bFoundCirclesU, bParallelPlanesU, bSamePlaneU,
                              bSameCenterU, bCommonAxisU, bPlanarGenCurveU,
                              dRadiusU, sAnglesU, sRefFrameU,
                              sCommonPoint, sCommonVec));
  // quit when not a sphere
  if (!bFoundCirclesU) return FALSE;

  // test sequence of U Direction (V Constant) isoCurves for circularity and parallelism
  SER(sm_TestForIsoCircles(pTestSurface, SM_SP_V, dScaledZero, dAngTol,
                              bFoundCirclesV, bParallelPlanesV, bSamePlaneV,
                              bSameCenterV, bCommonAxisV, bPlanarGenCurveV,
                              dRadiusV, sAnglesV, sRefFrameV,
                              sCommonPoint, sCommonVec));

  // quit when not a sphere
  if (!bFoundCirclesV) return FALSE;

  // see if we have a U or a V oriented sphere
  SmBoolean bIsSphereWithSwapUV = (   bParallelPlanesU
                                   && ! bSamePlaneU  // [ 080625 ]
                                   && bSameCenterV
                                   && sAnglesV.GetLength() <= 180.0 + SM_EFF_ZERO) ;
  SmBoolean bIsSphereNoSwapUV   = (   bParallelPlanesV
                                   && ! bSamePlaneV  // [ 080625 ]
                                   && bSameCenterU
                                   && sAnglesU.GetLength() <= 180.0+SM_EFF_ZERO) ;

  // quit when not a properly oriented sphere
  if(!bIsSphereWithSwapUV && !bIsSphereNoSwapUV)
    { return FALSE; }

  // locals
  double      dRad ;
  SmExtent1d *pSweepIvl ;
  SmPoint2d   sNurbMidUV    = sNurbUVDomain.Evaluate(0.5,0.5);
  SmPoint2d   sNurbMinUV    = sNurbUVDomain.Evaluate(0.0,0.0);
  SmBoolean   bSwapUV ;
  SmAxis2Placement *pRefFrame, *pOtherFrame ;

  // Get sphere position and parameters
  if(bIsSphereNoSwapUV)
    {
      bSwapUV     = FALSE ;
      pRefFrame   = &sRefFrameV ;
      pOtherFrame = &sRefFrameU ;
      dRad        = dRadiusU ;
      pSweepIvl   = &sAnglesV ;
    }
  else
    {
      SM_ASSERT(bIsSphereWithSwapUV) ;
      bSwapUV     = TRUE ;
      pRefFrame   = &sRefFrameU ;
      pOtherFrame = &sRefFrameV ;
      dRad        = dRadiusV ;
      pSweepIvl   = &sAnglesU ;
    }

  // don't make negative radius spheres
  SM_ASSERT(dRad > 0.0) ;
  if(dRad <= SM_EFF_ZERO)
    { return(FALSE) ; }

  // get sphere orientation and size
  const SmPoint3d &crSphereOrigin = pOtherFrame->GetOriginRef() ;  // the frame where all circles have common center
  const SmPoint3d &crSphereXAxis  = pRefFrame->GetXAxisRef() ;     // the fame where all circles are parallel
  const SmPoint3d &crSphereYAxis  = pRefFrame->GetYAxisRef() ;
  SmVector3d       sSphereZAxis   = pRefFrame->GetZAxis() ;

  // Check GenCurve rotation direction, its 'y' axis is either +/- sweepCircle z
  // bInsideOut: TRUE = GenCurve starts at +90 Pole and ends at -90 Pole
  //             FALSE= GenCurve starts at -90 Pole and ends at +90 Pole
  // assumes: OtherFrame is valid for 1st Nurb isoparameter GenCurve
  SmVector3d sMinorOrientation = pOtherFrame->GetZAxis() * crSphereXAxis ;
  SM_ASSERT(sMinorOrientation.IsParallelTo(sSphereZAxis, 2.0)) ;
  SmBoolean bInsideOut = sSphereZAxis.Dot(sMinorOrientation) < 0.0
                         ? TRUE
                         : FALSE ;

  // measure genCurve angles from sphere -Z axis
  SmVector3d sVecMin        = sMinPnt - crSphereOrigin;
  SmVector3d sVecMax        = sMaxPnt - crSphereOrigin;

  // get GenCurve min and max angles measured from Sphere's bottom pole in degrees
  double dAngleMin, dAngleMax;
  SmVector3d sNegZAxis = -sSphereZAxis ;
  SE(sNegZAxis.AngleBetween(sVecMin,dAngleMin));
  SE(sNegZAxis.AngleBetween(sVecMax,dAngleMax));
  dAngleMin = 180.0 * dAngleMin / SM_PI;
  dAngleMax = 180.0 * dAngleMax / SM_PI;

  // adjust param range from [Umin_to_UMax, 0_to_180] to [Umin_to_UMax, -90_to_90]
  //  acounting for InsideOut reversals
  double dAdjustAngle = -90.0;
  SmPoint2d sStepMin(pSweepIvl->GetMin(),smos_Min(dAngleMin,dAngleMax)+dAdjustAngle);
  SmPoint2d sStepMax(pSweepIvl->GetMax(),smos_Max(dAngleMin,dAngleMax)+dAdjustAngle);

  // snap to ends for small tolerances - quit for genCurves that extend beyond the poles
  if (sStepMin.y < -90.0) { if (sStepMin.y < -90.0-SM_EFF_ZERO_SQRT) { return FALSE; }
                            sStepMin.y = -90.0;
                          }
  if (sStepMax.y >  90.0) { if (sStepMax.y > 90.0+SM_EFF_ZERO_SQRT) { return FALSE; }
                            sStepMax.y = 90.0;
                          }

  // define Sphere domain
  SmExtent2d sAnalUVDomain(sStepMin.x, sStepMin.y, sStepMax.x, sStepMax.y);

  // Get GenCurve Isoparameter Curve
  SmBSplineCurve *pGenCurveIsoCrv = NULL ;
  SER(pTestSurface->CreateIsoParametricCurve(crContext,
                                             bSwapUV ? SM_SP_V  : SM_SP_U,
                                             bSwapUV ? sNurbMinUV.y : sNurbMinUV.x,
                                             0.0, pGenCurveIsoCrv));
  SmObjDelete sCleanObj(pGenCurveIsoCrv) ;

  SmCircle *pCircle = new (crContext) SmCircle(crSphereOrigin,
                                               crSphereXAxis,
                                               sSphereZAxis,
                                               sAnalUVDomain.GetVInterval(),
                                               dRad, 3, &crContext, pGenCurveIsoCrv,
                                               bInsideOut) ;

  // If pCircle's STEP interval does not match the v-interval of sAnalUVDomain,
  // then the Sphere constructor will complain (actually the SmSurfOfRevolution
  // constructor), and the resulting bad surface can lead to crashes.  [B414]
  // But: why would they not match?  This should be investigated if it's ever hit.

  if ( ! pCircle->GetSTEPInterval().AreEqual( sAnalUVDomain.GetVInterval(), SM_EFF_ZERO) )
    {
      WARN( _T("Bad result in IsNurbSurfaceSphere(): GenCurve STEP interval does not match sAnalUVDomain VInterval")) ;
      delete pCircle; pCircle = NULL;
      return FALSE;
    }

  // build a new sphere with given GenCurve to force parameterization compatibility
  //    constant U Curve orientations,
  //    constant V Curve centers and radius,
  //    and computed domain
  rpSphere = new (crContext) SmSphere(crSphereOrigin,
                                      crSphereXAxis,
                                      crSphereYAxis,
                                      sAnalUVDomain,
                                      dRad,
                                      bSwapUV,
                                      bInsideOut,
                                      pCircle);
  NER(rpSphere);
  // error recovery
  if (rpSphere->m_pGenCurve == NULL) { delete rpSphere; rpSphere = NULL;
                                       return FALSE;
                                     }

#ifdef SM_DEBUG_CODE
  if (bDebugMe || lDebugCount == lCount)
    {
      rpSphere->m_pGenCurve->Dump();

      if ( FALSE ) {
          smgfx_Erase();
          smgfx_SetLook(1,2, 0,0,1); pTestSurface->DrawUV(3,3); sm_GraphicsLoop();
          sm_GraphicsLoop();
      }
      smgfx_SetLook(3,4, 0,1,1); pGenCurveIsoCrv->Draw(); sm_GraphicsLoop();
      smgfx_SetLook(1,2, 1,0,1); rpSphere->DrawUV(4,4, FALSE, NULL, TRUE); sm_GraphicsLoop();
      smgfx_SetLook(5,6, 1,0,0); rpSphere->m_pGenCurve->Draw(); sm_GraphicsLoop();
      sm_GraphicsLoop();

      smgfx_SetLook( 1,2, 0,0,1 ); pRefFrame  ->Draw(); sm_GraphicsLoop();
      smgfx_SetLook( 1,2, 0,1,0 ); pOtherFrame->Draw(); sm_GraphicsLoop();
      sm_GraphicsLoop();

      SM_ASSERT_VALID( rpSphere );
      SM_ASSERT_VALID( rpSphere->m_pGenCurve );
      SM_ASSERT_VALID( pGenCurveIsoCrv );
    }
#endif // SM_DEBUG_CODE

  // reparameterize the NURB so that AnalyticSurface->NurbDomain == InputSurface->NurbDomain
  SmStatus eStat = rpSphere->Reparameterize(pTestSurface->GetNaturalUVDomain()) ;
  if ( eStat != SM_SUCCESS )
    { delete rpSphere; rpSphere = NULL; return FALSE; } // [B414]

  // arrive here after New Sphere has been constructed. Need to
  //  1. set NewSphere->m_pNurb = inputSurface->m_pNurb
  //  2. Copy InputSurface->Attributes onto NewSphere.
  //  3. make a Sphere PolarConverter for rotation angles

  // obsolete - it's a mistake to introduce tolerance sized variations between the Anal and Nurb representations
  //            by copying the m_pNurb object
  // Copy pTestSurface->m_pNurb into newSphere
  //      SM_ASSERT(rpSphere->m_pNurb == NULL) ;
  //      rpSphere->m_pNurb = sm_AllocateAndCopyNurbSurface(((SmBSplineSurface *)pTestSurface)->GetGwNurbPointer()) ;
  //
  //      // Build newSphere PolarConverter from Nurb midPoint rotation isoParamCurve
  //      rpSphere->RebuildPolarConverter() ;
  // end obsolete

  // Copy pTestSurface attributes onto newSphere
  ((SmBSplineSurface*)pTestSurface)->Notify(SM_NO_COPY, rpSphere, SM_NO_GET_OWNER(rpSphere), SM_NO_GET_OWNER(pTestSurface));

#ifdef SM_DEBUG_CODE
  if (bDebugMe || lDebugCount == lCount)
    {
      SmBSplineCurve *pIso = NULL ;
      if (rpSphere->m_bSwapUV) { SER(rpSphere->CreateIsoParametricCurve(crContext,SM_SP_U,0.0,SM_EFF_ZERO,pIso));
                               }
      else                     { SER(rpSphere->CreateIsoParametricCurve(crContext,SM_SP_V,0.0,SM_EFF_ZERO,pIso));
                               }
      SmObjDelete sCleanIso(pIso);

      // dump input surface, output surface, sweep isoCurve, genCurve
      pTestSurface->Dump();
      rpSphere->Dump();
      rpSphere->m_pGenCurve->Dump();
      pIso->Dump();

      smgfx_Erase();
      smgfx_SetLook(1,2, 0,0,1); rpSphere->DrawUV(); sm_GraphicsLoop();
      smgfx_SetLook(3,4, 1,0,0); rpSphere->m_pGenCurve->Draw(); sm_GraphicsLoop();
      smgfx_SetLook(6,7, 1,0,1); pIso->Draw(); sm_GraphicsLoop();
      smgfx_SetLook(3,4, 0,1,0); pTestSurface->DrawUV(4,4); sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
#endif

  // all done
  SM_DUMP_AND_ASSERT2_VALID(rpSphere) ;
  return TRUE;

} // end SmSphere::IsNurbSurfaceSphere

/*******************************************************************//***
PURPOSE: Rebuild all analytic, GenCurve, and PolarCurve cached data
  from the m_pNurb data.

NOTES: Can be used after modifying the m_pNurb data to update
  the analytic data.

RETURNS --- SM_ERR if the NURB no longer represents this Analytic surface
***********************************************************************/
SmStatus SmSphere::RebuildSTEPFromNURBParameters()
{
  // check m_pNurb - a degree 2 rational surface in at least 1 direction
  ULONG lDegU = GetDegree(SM_SP_U);
  ULONG lDegV = GetDegree(SM_SP_V);
  if (   lDegU != 2
      || lDegV != 2
      || !IsRational())
    { SER_MSG(SM_ERR, _T("Modified Nurb SmSphere: no longer has degree 2 irrationation basis functions")) ; }

  // method
  //  Check for iso circles

  //  Set m_pGenCurve to be SmCircle
  //  Set m_dRadius
  //  Set m_bInsideOut
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
  double dAngTol         = SM_EFF_ZERO_SQRT * dToleranceScale ;

  // iso circle locals
  const SmContext *cpContext = GetContext() ;
  SmBoolean bFoundCirclesU, bParallelPlanesU, bSamePlaneU, bSameCenterU, bCommonAxisU, bPlanarGenCurveU;
  SmBoolean bFoundCirclesV, bParallelPlanesV, bSamePlaneV, bSameCenterV, bCommonAxisV, bPlanarGenCurveV;
  SmExtent1d sAnglesU, sAnglesV;
  double dRadiusU, dRadiusV;
  SmAxis2Placement sRefFrameU, sRefFrameV;
  SmPoint3d sCommonPoint, sCommonVec ;

  // test sequence of V Direction (U Constant) isoCurves for circularity and parallelism
  SER(sm_TestForIsoCircles(this, SM_SP_U, dScaledZero, dAngTol,
                              bFoundCirclesU, bParallelPlanesU, bSamePlaneU,
                              bSameCenterU, bCommonAxisU, bPlanarGenCurveU,
                              dRadiusU, sAnglesU, sRefFrameU,
                              sCommonPoint, sCommonVec));
  // quit when not a sphere
  if (!bFoundCirclesU) return FALSE;

  // test sequence of U Direction (V Constant) isoCurves for circularity and parallelism
  SER(sm_TestForIsoCircles(this, SM_SP_V, dScaledZero, dAngTol,
                              bFoundCirclesV, bParallelPlanesV, bSamePlaneV,
                              bSameCenterV, bCommonAxisV, bPlanarGenCurveV,
                              dRadiusV, sAnglesV, sRefFrameV,
                              sCommonPoint, sCommonVec));

  // see if we have a U or a V oriented sphere
  // Tol: compensate for using degrees.  [B279]
  SmBoolean bIsSphereWithSwapUV = (   bParallelPlanesU
                                   && ! bSamePlaneV  // [ 080625 ]
                                   && bSameCenterV
                                   && sAnglesV.GetLength() <= 180.0 + 360*SM_EFF_ZERO) ;
  SmBoolean bIsSphereNoSwapUV   = (   bParallelPlanesV
                                   && ! bSamePlaneV  // [ 080625 ]
                                   && bSameCenterU
                                   && sAnglesU.GetLength() <= 180.0 + 360*SM_EFF_ZERO) ;

  // quit when not a properly oriented sphere
  if(!bIsSphereWithSwapUV && !bIsSphereNoSwapUV)
    { SER_MSG(SM_ERR, _T("Modified Nurb SmSphere: no longer has oriented circular IsoParamCurves ")) ; }

  // locals
  double      dRad ;
  SmExtent1d *pSweepIvl ;
  SmPoint2d   sNurbMidUV    = sNurbUVDomain.Evaluate(0.5,0.5);
  SmPoint2d   sNurbMinUV    = sNurbUVDomain.Evaluate(0.0,0.0);
  SmBoolean   bSwapUV ;
  SmAxis2Placement *pRefFrame, *pOtherFrame ;

  // Get sphere position and parameters
  if(bIsSphereNoSwapUV)
    {
      bSwapUV     = FALSE ;
      pRefFrame   = &sRefFrameV ;
      pOtherFrame = &sRefFrameU ;
      dRad        = dRadiusU ;
      pSweepIvl   = &sAnglesV ;
    }
  else
    {
      SM_ASSERT(bIsSphereWithSwapUV) ;
      bSwapUV     = TRUE ;
      pRefFrame   = &sRefFrameU ;
      pOtherFrame = &sRefFrameV ;
      dRad        = dRadiusV ;
      pSweepIvl   = &sAnglesU ;
    }

  // get sphere orientation and size
  const SmPoint3d &crSphereOrigin = pOtherFrame->GetOriginRef() ;
  const SmPoint3d &crSphereXAxis  = pRefFrame->GetXAxisRef() ;
  const SmPoint3d &crSphereYAxis  = pRefFrame->GetYAxisRef() ;
  SmVector3d       sSphereZAxis   = pRefFrame->GetZAxis() ;

  // Check GenCurve rotation direction, its 'y' axis is either +/- sweepCircle z
  // bInsideOut: TRUE = GenCurve starts at +90 Pole and ends at -90 Pole
  //             FALSE= GenCurve starts at -90 Pole and ends at +90 Pole
  // assumes: OterhFrame is valid for 1st Nurb isoparameter GenCurve
  SmVector3d sMinorOrientation = pOtherFrame->GetZAxis() * crSphereXAxis ;
  SM_ASSERT(sMinorOrientation.IsParallelTo(sSphereZAxis, 2.0)) ;
  SmBoolean bInsideOut = sSphereZAxis.Dot(sMinorOrientation) < 0.0
                         ? TRUE
                         : FALSE ;

  // measure genCurve angles from sphere -Z axis
  SmVector3d sVecMin        = sNurbMinPoint - crSphereOrigin;
  SmVector3d sVecMax        = sNurbMaxPoint - crSphereOrigin;

  double dAngleMin=0.0, dAngleMax=0.0;
  SE((-sSphereZAxis).AngleBetween(sVecMin,dAngleMin));
  SE((-sSphereZAxis).AngleBetween(sVecMax,dAngleMax));

  dAngleMin = 180.0 * dAngleMin / SM_PI;
  dAngleMax = 180.0 * dAngleMax / SM_PI;

  // adjust param range from [Umin_to_UMax, 0_to_180] to [Umin_to_UMax, -90_to_90]
  //  acounting for InsideOut reversals
  double dAdjustAngle = -90.0;
  SmPoint2d sStepMin(pSweepIvl->GetMin(),smos_Min(dAngleMin,dAngleMax)+dAdjustAngle);
  SmPoint2d sStepMax(pSweepIvl->GetMax(),smos_Max(dAngleMin,dAngleMax)+dAdjustAngle);

  // snap to ends for small tolerances - quit for genCurves that extend beyond the poles
  if (sStepMin.y < -90.0) { if (sStepMin.y < -90.0-SM_EFF_ZERO_SQRT) return FALSE;
                            sStepMin.y = -90.0;
                          }
  if (sStepMax.y >  90.0) { if (sStepMax.y > 90.0+SM_EFF_ZERO_SQRT) return FALSE;
                            sStepMax.y = 90.0;
                          }

  // define Sphere domain
  SmExtent2d sAnalUVDomain(sStepMin.x, sStepMin.y, sStepMax.x, sStepMax.y);

  // Get GenCurve Isoparameter Curve
  SmBSplineCurve *pGenCurveIsoCrv = NULL ;
  SER(SmBSplineSurface::CreateIsoParametricCurve(*cpContext,
                                                 bSwapUV ? SM_SP_V  : SM_SP_U,
                                                 bSwapUV ? sNurbMinUV.y : sNurbMinUV.x,
                                                 0.0,
                                                 pGenCurveIsoCrv));
  SmObjDelete sCleanObj(pGenCurveIsoCrv) ;
  SmCircle *pCircle = new (*cpContext) SmCircle(crSphereOrigin,
                                                crSphereXAxis,
                                                sSphereZAxis,
                                                sAnalUVDomain.GetVInterval(),
                                                dRad, 3, cpContext,
                                                pGenCurveIsoCrv,
                                                bInsideOut) ;
  SmObjDelete sCleanGen(pCircle) ;

  // no work - GenCurve is degenerate
  if (pCircle->IsDegenerate(dScaledZero))
    { SER_MSG(SM_ERR, _T("(Modified Nurb SmSphere: now has degenerate GenCurve")) ; }

  // set m_pGenCurve
  // check for errors in construct
  if (m_pGenCurve != NULL) { delete m_pGenCurve; m_pGenCurve = NULL; }
  m_pGenCurve = pCircle ;
  m_pGenCurve->SetOwner(this) ;
  sCleanGen.Clear() ;

  // set Analytic parameters
  m_dRadius          = dRad ;
  m_bInsideOut       = bInsideOut ;
  m_bPlanarGenerator = TRUE ;
  m_vPosition.SetCanonical(crSphereOrigin, crSphereXAxis, crSphereYAxis) ;
  m_bSwapUV          = bSwapUV ;
  m_vAnalUVDomain    = sAnalUVDomain ;

  // Set PolarConverter from Nurb midPoint rotation isoParamCurve
  // pRotationIsoCrv will be an SmCircle because isoParam is coming from SmSphere
  RebuildPolarConverter() ;

  //      m_vPolarConverter.SetPolarConversionPossible(FALSE) ;
  //      SmBSplineCurve *pRotationIsoCrv = NULL ;
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

} // end SmSphere::RebuildSTEPFromNURBParameters

/*******************************************************************//**
PURPOSE: Return TRUE when the surface is closed in the specified direction.

NOTES:
   Spheres can be closed in longitude (east-west) but never in latitude.
   In STEP, longitude is U and latitude is V.
   In Nurbs, those can be either way depending on m_bSwapUV.

   Continuity of a closed sphere: in the STEP representation, this would be CINFINITY,
   but in NURBS, it is only C1_G2.  This routine deals with the NURBS representation,
   so we return SM_CT_C1_G2.
***********************************************************************/
SmBoolean SmSphere::IsClosed
  (const SmExtent2d & crUVDomain,       // in : Nurb domain to check
   SmSurfParamType    eSurfParam,       // in : oneof SM_SP_U    [check Vmin == Vmax]
                                        //            SM_SP_V    [check Umin == Umax]
                                        //            SM_SP_BOTH
   double           * pdOptTolerance,   // NotUsed: in : min distance between distinct points
   SmContinuityType * peOptContinuity)  //      NULL = scale to 1000*SM_EFF_ZERO*SurfPosition
                                        // out: oneof oneof SM_CT_DISCONTINUOUS
                                        //                  SM_CT_C1_G2
                                        //     NULL to ignore
 const
{
  SM_REF1(pdOptTolerance) ;
  // Init output
  if ( peOptContinuity ) { *peOptContinuity = SM_CT_DISCONTINUOUS; }

  // A sphere can't be closed in both U and V:
  if ( eSurfParam == SM_SP_BOTH )
    { return FALSE; }

  // Also, we're not closed in our north-south direction.
  if ( ! m_bSwapUV && eSurfParam == SM_SP_V )
    { return FALSE; }
  if (   m_bSwapUV && eSurfParam == SM_SP_U )
    { return FALSE; }

  // Ok, here we're checking around our equator.
  // (Either they're asking for U, or we're swapped.)
  // We can't be closed if our analytic domain does not cover a complete circle.
  // The east-west direction is always U in the analytic representation.
  double dArcLengthDegU = m_vAnalUVDomain.GetUInterval().GetLength();
  double dTolDeg        = SM_EFF_ZERO/smos_Max(1.0,m_dRadius) * 180.0 / SM_PI;
  if ( ! SM_ARE_SAME_TO_TOL(dArcLengthDegU, 360.0, dTolDeg) )
    { return FALSE; }

  // The only thing left to check is that the passed-in domain
  // matches our Nurbs domain, in the direction of interest.
  SmExtent2d sNurbDomain = GetNaturalUVDomain();
  SmExtent1d sNurbsSweepIvl = ( eSurfParam == SM_SP_U ) ? sNurbDomain.GetUInterval()
                                                        : sNurbDomain.GetVInterval();
  SmExtent1d sGivenSweepIvl = ( eSurfParam == SM_SP_U ) ? crUVDomain .GetUInterval()
                                                        : crUVDomain .GetVInterval();

  SmBoolean bClosedSweep =    sNurbsSweepIvl.IsContainedBy( sGivenSweepIvl )
                           && sGivenSweepIvl.IsContainedBy( sNurbsSweepIvl );

  if ( bClosedSweep )
    { if ( peOptContinuity ) { *peOptContinuity = SM_CT_C1_G2; } }

  return bClosedSweep;

} // end SmSphere::IsClosed

/*******************************************************************//**
PURPOSE: Return TRUE when all sweep latitude circles are closed:
         the m_vAnalytic.IntervalU.Length == 360.0

NOTES:
  This is not the same as IsClosed():
   1. It works in the analytic definition, not the Nurbs.
   2. Here "Closed" means it spans the entire STEP U-domain, 360 degrees.
***********************************************************************/
SmBoolean SmSphere::IsClosedU() const
{
  double dArcLengthDegU = m_vAnalUVDomain.GetUInterval().GetLength() ;
  double dTolDeg        = SM_EFF_ZERO/smos_Max(1.0,m_dRadius) * 180.0 / SM_PI ;

  SmBoolean bRtn = SM_ARE_SAME_TO_TOL(dArcLengthDegU, 360.0, dTolDeg) ;

  return(bRtn) ;

} // end SmSphere::IsClosedU

/*******************************************************************//**
PURPOSE: Return TRUE when all genCurve longitude circles run from
         pole to pole, the m_vAnalytic.IntervalV.Length = 180.0

NOTES:
  This is not the same as IsClosed():
   1. It works in the analytic definition, not the Nurbs.
   2. Here "Closed" means it spans the entire STEP V-domain, -90 to 90.
***********************************************************************/
SmBoolean SmSphere::IsClosedV() const
{
  double dArcLengthDegV = m_vAnalUVDomain.GetVInterval().GetLength() ;
  double dTolDeg        = SM_EFF_ZERO/smos_Max(1.0,m_dRadius) * 180.0 / SM_PI ;

  SmBoolean bRtn = SM_ARE_SAME_TO_TOL(dArcLengthDegV, 180.0, dTolDeg) ;

  return(bRtn) ;

} // end SmSphere::IsClosedV

/*******************************************************************//**
PURPOSE: Rotate sphere about its X axis.

NOTES:
***********************************************************************/
SmStatus SmSphere::RotationAboutAxisX
  (const SmContext & crContext,      // in : new object context
   double            dAngleDeg,      // in : rotation amount
   SmSphere       *& rpNewSphere)    // out: new Sphere
{
  double dAngleRad = SM_DEG2RAD(dAngleDeg);

  // build the transformation matrix
  SmAxis2Placement sRF;
  sRF.RotateAboutAxisAtPoint(dAngleRad,
                             m_vPosition.GetOriginRef(),
                             m_vPosition.GetXAxisRef());

  // transform the sphere's placement
  SmAxis2Placement sTmpA2P;
  m_vPosition.TransformAxis2Placement(sRF,sTmpA2P);

  // make a new sphere with rotated placement
  SER(SmSphere::CreateCanonical(crContext,sTmpA2P,
                                m_dRadius,rpNewSphere));

  // trim UVDomain
  SER(rpNewSphere->AdjustSTEPUVDomain(GetSTEPUVDomain()));

  // all done
  return SM_SUCCESS;

} // end SmSphere::RotationAboutAxisX

/*******************************************************************//**
PURPOSE: Rotate sphere about its Y axis.

NOTES:
***********************************************************************/
SmStatus SmSphere::RotationAboutAxisY
  (const SmContext & crContext,    // in : new object context
   double            dAngleDeg,    // in : rotation amount
   SmSphere       *& rpNewSphere)  // out: newly allocated sphere
{
  double dAngleRad = SM_DEG2RAD(dAngleDeg);

  // build the transformation matrix
  SmAxis2Placement sRF;
  sRF.RotateAboutAxisAtPoint(dAngleRad,
                             m_vPosition.GetOriginRef(),
                             m_vPosition.GetYAxisRef());

  // transform the sphere's placement
  SmAxis2Placement sTmpA2P;
  m_vPosition.TransformAxis2Placement(sRF,sTmpA2P);

  // make an new sphere with rotated placement
  SER(SmSphere::CreateCanonical(crContext,sTmpA2P,
      m_dRadius,rpNewSphere));

  // trim UVDomain
  SER(rpNewSphere->AdjustSTEPUVDomain(GetSTEPUVDomain()));

  // all done
  return SM_SUCCESS;

} // end SmSphere::RotationAboutAxisY

/*******************************************************************//**
PURPOSE: Rotate sphere about its Z axis.

NOTES:
***********************************************************************/
SmStatus SmSphere::RotationAboutAxisZ
  (const SmContext & crContext,    // in : new object context
   double            dAngleDeg,    // in : rotation amount
   SmSphere       *& rpNewSphere)  // out: new Sphere
{
  double dAngleRad = SM_DEG2RAD(dAngleDeg);

  // build transformation matrix
  SmAxis2Placement sRF;
  sRF.RotateAboutAxisAtPoint(dAngleRad,
                             m_vPosition.GetOriginRef(),
                             m_vPosition.GetZAxis());

  // transform the sphere's placement
  SmAxis2Placement sTmpA2P;
  m_vPosition.TransformAxis2Placement(sRF,sTmpA2P);


  // make a new sphere with rotated placement
  SER(SmSphere::CreateCanonical(crContext,sTmpA2P,
                                m_dRadius,rpNewSphere));

  // trim uv domain
  SER(rpNewSphere->AdjustSTEPUVDomain(GetSTEPUVDomain()));

  // all done
  return SM_SUCCESS;

} // end SmSphere::RotationAboutAxisZ

//      /*******************************************************************//**
//      PURPOSE: Swap the U and V parameterizations of a surface.  This
//          effectively reverses the orientation of the surface.
//
//      NOTES:
//      ***********************************************************************/
//      SmStatus SmSphere::SwapUV
//        ()
//      {
//        SER(SmBSplineSurface::SwapUV());
//        SmAxis2Placement sPlace;
//        SER(Transform(sPlace,NULL));
//        return SM_SUCCESS;
//
//      } // end SmSphere::SwapUV

/*******************************************************************//**
PURPOSE: Map 3dPoints back to Step parameters within current
            Sphere m_vAnalUVDomain

NOTES:
  Every Point is dropped to the sphere and given a valid UVPoint
  except for a point coincident with the sphere's origin.

RETURNS ---
  SM_SUCCESS when the point is on the sphere and within both the sphere's
              and the give AnalUVDomain limits.
  Points which can be mapped outside the surface's
  trim domain are mapped and reLocation is set to SM_LT_EXTERIOR

  If the result is on the seam of a closed surface, and a guess was
  given, the output will be set to whichever side of the seam is closer
  to the guess.  If no guess was given, it will be set to the low end
  of the closed domain.

***********************************************************************/
SmStatus SmSphere::STEPInversion
  (const SmExtent2d & crAnalDomain,       // in : domain limit for valid transformations
   const SmPoint3d  & crPointOnSurf,      // in : Target Point - must be on surface within Tolerance
   double             d3DTolerance,       // in : Max allowed distance between Point and Surface
   SmPoint2d        & rdAnalUVParameter,  // out: STEP UV params for target point
                                          //      [U = Degrees about rotation Axis,
                                          //       V = STEP Param of rotatedPoint on genCurve]
   SmLocationType   & reLocation,         // out: oneof SM_LT_POLE,
                                          //            SM_LT_U_SEAM,
                                          //            SM_LT_V_SEAM,
                                          //            SM_LT_UV_SEAM,
                                          //            SM_LT_INTERIOR
                                          //            SM_LT_EXTERIOR
   SmPoint2d        * pUVGuess)           // in, opt: guess parameter.
  const
{
  // init output
  reLocation = SM_LT_EXTERIOR ;
  rdAnalUVParameter.Set(0.0, 0.0) ;

  // Sphere locals
  const SmVector3d &rOrigin = m_vPosition.GetOriginRef() ;
  const SmVector3d &rXAxis  = m_vPosition.GetXAxisRef() ;
  const SmVector3d &rYAxis  = m_vPosition.GetYAxisRef() ;
  SmVector3d        sZAxis  = m_vPosition.GetZAxis() ;
  double            dTol2d  = d3DTolerance*180.0/SM_PI/m_dRadius ;
  SmBoolean        bClosed  =  smos_Fabs(m_vAnalUVDomain.XLength() - 360.0) < dTol2d ;
  SmBoolean        bOnSurf  =  TRUE ;
  SmExtent1d       sUDomain(m_vAnalUVDomain.GetMin().x,
                            m_vAnalUVDomain.GetMax().x) ;
  double           dScaledZero = SM_EFF_ZERO * (1.0 + rOrigin.GetMaxDimension()) ;
  SmPoint3d sTestPoint ;

  // get Point-SphereCenter vector
  SmVector3d sVec = crPointOnSurf - rOrigin ;
  double  dLength = sVec.Length() ;

  // Origin Points are errors
  if(dLength < dScaledZero)
    { return(SM_ERR) ; }

  // drop point to Sphere - Remember when Point was not initially on Surface
  sVec       = sVec * (m_dRadius / dLength) ;
  sTestPoint = rOrigin + sVec ;
  if(smos_Fabs(dLength - m_dRadius) > d3DTolerance)
    { bOnSurf    = FALSE ; }

  // get Object coordinates for the point
  double dX = sVec.Dot(rXAxis) ;
  double dY = sVec.Dot(rYAxis) ;
  double dZ = sVec.Dot(sZAxis) ;

  // Get Spherical Coordinates (degrees)
  double dU = smos_ArcTangent2(dY, dX) * 180.0 / SM_PI ;     //  dU range:[-180 to 180]
  double dV = smos_ArcSine(dZ / m_dRadius) * 180.0 / SM_PI ; //  dV range:[-90 to 90]

  // set output
  rdAnalUVParameter.Set(dU, dV) ;

  // classify point
  SmBoolean bOnPole = (   dV >  90.0 - dTol2d
                       || dV < -90.0 + dTol2d) ;
  SmBoolean bOnSeam = (   bClosed
                       && sUDomain.IsValueOnBoundary(dU, dTol2d)) ;

  // return error for points off Sphere or outside of domain
  // side effect: move rdAnalUVParameter into crAnalDomain if appropriate [B606]
  SmBoolean bInside =      crAnalDomain.ContainsPeriodicPoint2d( rdAnalUVParameter, 360.0, 0.0, dTol2d, &rdAnalUVParameter )
                     && m_vAnalUVDomain.ContainsPeriodicPoint2d( rdAnalUVParameter, 360.0, 0.0, dTol2d );

  reLocation =   (bInside && bOnPole) ? SM_LT_POLE
               : (bInside && bOnSeam) ? SM_LT_U_SEAM
               : (bInside)            ? SM_LT_INTERIOR
               :                        SM_LT_EXTERIOR ;

  // If a guess was given and we're on the seam,
  // set u-parameter to what's closer to the guess.
  if ( bOnSeam && pUVGuess != NULL )
    {
      if ( pUVGuess->x > sUDomain.GetMid() )
        { rdAnalUVParameter.x = sUDomain.GetMax(); }
    }

  // all done
  return(bOnSurf ? SM_SUCCESS : SM_ERR) ;

} // end SmSphere::STEPInversion

/*******************************************************************//**
PURPOSE: Scale and transform an SmSphere surface.

NOTES: Scaling is not allowed on analytical surfaces.
***********************************************************************/
SmStatus SmSphere::Transform
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

  // update sphere size
  m_dRadius *= dScale;

  // finish the call in the parent function
  SmSurfOfRevolution::Transform(crRotateNMove, cpOptScale) ;

  // all done
  return SM_SUCCESS;

} // end SmSphere::Transform

/*******************************************************************//**
PURPOSE: Compute the total size of the memory used by the SmSphere.

NOTES: Does not add in attribute memory
***********************************************************************/
ULONG SmSphere::GetMemoryUsed     // rtn: smaller size of actually used memory in bytes
  (ULONG    & rlMemoryAllocated,  // out: bigger size of all allocated memory in bytes
   SmMarkType eMarkType)          // in : uses without increment eMarkType value
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

} // end SmSphere::GetMemoryUsed

/*******************************************************************//**
PURPOSE: AssertValid() Test reporting static labels
***********************************************************************/
SmAssertReportLabel sAssertSphere_list[] =
{
  /*  0 */ {SM_AT_RADIUS,           _T("Positive Radius"),      _T("m_dRadius is positive") },
  /*  1 */ {SM_AT_PARAMETERIZATION, _T("Parameterization"),     _T("GenCurve has same parameterization as surface") },
  /*  2 */ {SM_AT_GEOMETRIC,        _T("Overlapping Domain"),   _T("Analytic U Direction Domain too large (more than 360 deg): spheres can't overlap themselves") },
  /*  3 */ {SM_AT_GEOMETRIC,        _T("Overlapping Domain"),   _T("Analytic V Direction Domain too large (more than 180 deg): spheres can't run through poles")  },
  /*  4 */ {SM_AT_GEOMETRIC,        _T("Included Singularity"), _T("Analytic V Direction Domain places v=-90 pole within domain interior") },
  /*  5 */ {SM_AT_GEOMETRIC,        _T("Included Singularity"), _T("Analytic V Direction Domain places v=+90 pole within domain interior") }
} ;

/*******************************************************************//**
PURPOSE: Make sure GenCurve and Surface parameterizations are
            compatible.

RETURNS ---  TRUE  = OK
             FALSE = Problem
***********************************************************************/
SmBoolean SmSphere::AssertValid
 (SmAssertArray    * pAList,        // i/o: Accumulating list of failed Asserts, NULL to ignore, default:[NULL]
  SmAssertTestLevel  eTestLevel,    // in : SM_LEVEL_0=fewest tests, SM_LEVEL_1, SM_LEVEL_2=all tests,
                                    //      SM_LEVEL_GIVEN = run tests in order requested in pTestRequests
                                    //      default:[SM_LEVEL_0]
  SmAssertWalking    eWalkTree,     // NotUsed: in : SM_WALK = run AssertValid on any topology graph descendants, SM_NO_WALK=don't, default:[SM_WALK]
  SmTArray<ULONG>  * pTestRequests) // in : when eTestLevel == SM_LEVEL_GIVEN, run these tests in this order, default:[NULL]
 const
{
  SM_REF1(eWalkTree) ;
  // run parent AssertValid
  SmBoolean bRtn = TRUE ;

  // call the base class AssertValid
  bRtn &= (  (eTestLevel != SM_LEVEL_GIVEN)
           ? SmSurfOfRevolution::AssertValid(pAList, eTestLevel, SM_NO_WALK, pTestRequests)
           : TRUE ) ;

  /*  0 */ // must have a positive radius
  bRtn &= SM_ASSERT_VALUE_REPORT(0, SM_LEVEL_0, (m_dRadius > 0.0), 0.0, m_dRadius, _T("") ) ;

  // locals
  //const SmPoint3d &rSphereOrig  = m_vPosition.GetOriginRef() ;
  SmVector3d       sSphereZAxis = m_vPosition.GetZAxis() ;

  // check temporary genCurve against parameterization of NurbSurface
  const SmContext &crContext = *GetContext() ;
  SmBSplineCurve  *pGenCurve = NULL ;
  CreateGeneratorFromAngle(crContext, 0.0, pGenCurve) ;
  SmObjDelete sClean(pGenCurve) ;

  /*  1 */ // GenCurve AnalInterval   must equal Surface->AnalUVDomain->GetVInterval,
           // GenCurve NURBInterval   must equal m_bSwapUV ? Surface->NURBUInterval : Surface->NURBVInterval
           // GenCurve NURBKnotVector must equal m_bSwapUV ? Surface->NURBKnotU : Surface->NURBKnotV
  bRtn &= SM_ASSERT_VALUE_REPORT(1, SM_LEVEL_0,
                                 (HasSameParameterization(SM_SP_V,                        // in : Analytic Domain Direction
                                                          m_bSwapUV ? SM_SP_U : SM_SP_V,  // in : Nurb     Domain Direction
                                                          pGenCurve) ),                   // in : The GenCurve
                                 SM_EFF_ZERO, SM_UNDEF_DOUBLE, _T("")) ;

  /*  2 */ // Analytic U Direction Domain too large: spheres can't overlap themselves
  SmExtent1d sIvlU      = m_vAnalUVDomain.GetUInterval() ;
  SmBoolean  bIvlTooBig = !(sIvlU.GetLength() <= 360.0 + SM_EFF_ZERO_DEG) ;
  bRtn &= SM_ASSERT_VALUE_REPORT(2, SM_LEVEL_0,
                                 bIvlTooBig == FALSE,
                                 SM_EFF_ZERO_DEG, sIvlU.GetLength() - 360.0, _T("")) ;
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  if(bIvlTooBig)
    { if(bDebugMe)
        { if(pGenCurve)   pGenCurve->Dump() ;
          if(m_pGenCurve) m_pGenCurve->Dump() ;
          Dump() ;
        }
    }
#endif // SM_DEBUG_CODE

  /*  3 */ // Analytic V Direction Domain too large: spheres can't run through their poles limited to IvlLength <= 180 degrees
  SmExtent1d sIvlV = m_vAnalUVDomain.GetVInterval() ;
  bIvlTooBig       = !(sIvlV.GetLength() <= 180.0 + SM_EFF_ZERO_DEG) ;
  bRtn &= SM_ASSERT_VALUE_REPORT(3, SM_LEVEL_0,
                                 bIvlTooBig == FALSE,
                                 SM_EFF_ZERO_DEG, sIvlV.GetLength() - 180.0, _T("")) ;
#ifdef SM_DEBUG_CODE
  if(bIvlTooBig)
    { if(bDebugMe)
        { if(pGenCurve)   pGenCurve->Dump() ;
          if(m_pGenCurve) m_pGenCurve->Dump() ;
          Dump() ;
        }
    }
#endif // SM_DEBUG_CODE

  /*  4 */ // Analytic V Direction Domain places v=-90 pole within domain interior
  double dMappedPole0, dDistToMin0, dDistToMax0 ;
  SmBoolean bContains0 = sIvlV.ContainsPeriodicValue(-90, 360, &dMappedPole0, &dDistToMin0, &dDistToMax0, SM_EFF_ZERO_DEG) ;
  bRtn &= SM_ASSERT_VALUE_REPORT(4, SM_LEVEL_0,
                                 (   bContains0 == FALSE
                                  || SM_IS_ZERO_TO_TOL(dDistToMin0, SM_EFF_ZERO_DEG)
                                  || SM_IS_ZERO_TO_TOL(dDistToMax0, SM_EFF_ZERO_DEG)),
                                 SM_EFF_ZERO_DEG, smos_Max(dDistToMin0, dDistToMax0), _T("")) ;
#ifdef SM_DEBUG_CODE
  if( !(   bContains0 == FALSE
        || SM_IS_ZERO_TO_TOL(dDistToMin0, SM_EFF_ZERO_DEG)
        || SM_IS_ZERO_TO_TOL(dDistToMax0, SM_EFF_ZERO_DEG)))
    { if(bDebugMe)
        { if(pGenCurve)   pGenCurve->Dump() ;
          if(m_pGenCurve) m_pGenCurve->Dump() ;
          Dump() ;
        }
    }
#endif // SM_DEBUG_CODE

  /*  5 */ // Analytic V Direction Domain places v=+90 pole within domain interior
  double dMappedPole1, dDistToMin1, dDistToMax1 ;
  SmBoolean bContains1 = sIvlV.ContainsPeriodicValue(+90, 360, &dMappedPole1, &dDistToMin1, &dDistToMax1, SM_EFF_ZERO_DEG) ;
  bRtn &= SM_ASSERT_VALUE_REPORT(5, SM_LEVEL_0,
                                 (   bContains1 == FALSE
                                  || SM_IS_ZERO_TO_TOL(dDistToMin1, SM_EFF_ZERO_DEG)
                                  || SM_IS_ZERO_TO_TOL(dDistToMax1, SM_EFF_ZERO_DEG)),
                                 SM_EFF_ZERO_DEG, smos_Max(dDistToMin1, dDistToMax1), _T("")) ;
#ifdef SM_DEBUG_CODE
  if( !(   bContains1 == FALSE
        || SM_IS_ZERO_TO_TOL(dDistToMin1, SM_EFF_ZERO_DEG)
        || SM_IS_ZERO_TO_TOL(dDistToMax1, SM_EFF_ZERO_DEG)))
    { if(bDebugMe)
        { if(pGenCurve)   pGenCurve->Dump() ;
          if(m_pGenCurve) m_pGenCurve->Dump() ;
          Dump() ;
        }
    }
#endif // SM_DEBUG_CODE

#ifdef SM_DEBUG_CODE
  // draw
  if(bDebugMe)
    {
      if(pGenCurve)   pGenCurve->Dump() ;
      if(m_pGenCurve) m_pGenCurve->Dump() ;
      Dump() ;

      SmFace *pFace = (SmFace *)GetFace() ;
      SmBrep *pBrep = pFace ? pFace->GetBrep() : NULL ;

      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,1) ; DrawUV() ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,0,0) ; if(pFace) pFace->Draw(SM_DM_CROSSHATCH) ; sm_GraphicsLoop() ;
      smgfx_SetLook(3,4, 1,0,0) ; if(m_pGenCurve) m_pGenCurve->Draw() ; sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

  // all done
//        SM_ASSERT(bRtn);

  return(bRtn) ;

} // end SmSphere::AssertValid

// obsolete
// /*******************************************************************//**
// PURPOSE: Fix Assert Report failures reported by an AssertValid call
// 
// RETURNS ---  TRUE  = OK
//              FALSE = Problem
// ***********************************************************************/
// SmBoolean SmSphere::AssertHeal
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
//       return ( SmSurfOfRevolution::AssertHeal(rAReport, pAList) ) ;
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
//                rAReport.m_pHealMessage = _T("SmSphere::AssertHeal fix not yet supported") ;
// 
//     }
// 
//   // all done
//   return(bRtn) ;
// 
// } // end SmSphere::AssertHeal
// end obsolete

/*******************************************************************//**
PURPOSE: Write SmSphere to given output stream.

NOTES:
***********************************************************************/
SmStatus SmSphere::WriteToDB
 (SmDatabaseIO & rDB,                // in : target output stream
  ULONG          lDBVersionNumber)   // in : database version to get proper sequence of writes
 const
{
  // file type, ASCII or BINARY
  SmFileType      eType    =  rDB.GetFileType();
  std::ostream  & rFileOut = *rDB.GetOutStreamPtr();

  if (eType == SM_ASCII)
    {
      rFileOut << m_dRadius    << " SmSphere Sphere Radius \n" ;
      rFileOut << m_bInsideOut << " SmSphere Sphere InsideOut: TRUE=outward pointing normal, FALSE=inverted \n" ;
    }
  else
    {
      SER(rDB.WriteDouble( m_dRadius)) ;
      SER(rDB.WriteBoolean( m_bInsideOut)) ;
    }

  // output the parent
  SER(SmSurfOfRevolution::WriteToDB(rDB, lDBVersionNumber)) ;

  // all done
  return SM_SUCCESS;

} // end SmSphere::WriteToDB

/*******************************************************************//**
PURPOSE: static method to Read a SmSphere from a given stream

NOTES:
***********************************************************************/
SmStatus SmSphere::ReadFromDB
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
         || rpNewSurface->IsKindOf(SmSphere_TYPE)) ? SM_SUCCESS : SM_ERR) ;

  // init output object
  SmSphere *pSphere =   (rpNewSurface == NULL)
                      ? new (crContext) SmSphere()
                      : (SmSphere *)rpNewSurface ;

  // file type
  SmFileType     eType   =  rDB.GetFileType();
  std::istream & rFileIn = *rDB.GetInStreamPtr();

  if (eType == SM_ASCII)
    {
      rFileIn >> pSphere->m_dRadius ;     rDB.GoToNextLine() ;
      rFileIn >> pSphere->m_bInsideOut ;  rDB.GoToNextLine() ;
    }
  else
    {
      SER(rDB.ReadDouble (pSphere->m_dRadius)) ;
      SER(rDB.ReadBoolean(pSphere->m_bInsideOut)) ;
    }

  // read the parent object
  SmSurface *pSurface = pSphere ;
  SER(SmSurfOfRevolution::ReadFromDB(SmSurfOfRevolution_TYPE, rDB, crContext, pSurface, lDBVersionNumber)) ;

  // all done
  rpNewSurface = pSphere ;
  return SM_SUCCESS;

} // end SmSphere::ReadFromDB

/*******************************************************************//**
 PURPOSE:
 NOTES:
***********************************************************************/
SmBoolean SmSphere::IsKindOf( SM_TYPE t ) const
{
  return ((SmSphere_TYPE == t) ? TRUE : SmSurfOfRevolution::IsKindOf( (t) ));
}

/*******************************************************************//**
PURPOSE: Dump Sphere surface data out for debugging.

NOTES:
***********************************************************************/
void SmSphere::Dump
  (void)
 const
{
  TCHAR sBuff[SM_TBLOCK_SIZE], sBuffForFile[SM_TBLOCK_SIZE];
  smos_WriteBuffer(_T("\nBegin SmSphere::Dump()")) ;

  smos_sprintf(sBuff,       _T("\nSmSphere = 0x%p"), this) ;
  smos_sprintf(sBuffForFile,_T("\nSmSphere = %s"), _T("notNULL")) ;
  smos_WriteBuffer(sBuff, sBuffForFile);

  smos_sprintf(sBuff,       _T("  Radius = %16.16lf"), m_dRadius);
  smos_WriteBuffer(sBuff);

  smos_sprintf(sBuff,       _T("  Origin = [%16.16lf %16.16lf %16.16lf]"),
             m_vPosition.GetOriginRef().x,
             m_vPosition.GetOriginRef().y,
             m_vPosition.GetOriginRef().z);
  smos_WriteBuffer(sBuff);

  smos_sprintf(sBuff,_T("\n  bSwap_UV = %s, bInsideout = %s"),
             m_bSwapUV    ? _T("TRUE") : _T("FALSE"),
             m_bInsideOut ? _T("TRUE") : _T("FALSE") );
  smos_WriteBuffer(sBuff);

  SmSurfOfRevolution::Dump();
  smos_WriteBuffer(_T(" End SmSphere::Dump()\n")) ;

} // end SmSphere::Dump
