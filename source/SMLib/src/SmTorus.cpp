// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************//**
* FILE NAME --- SmTorus.cpp
* PURPOSE: Implementation of SmTorus methods.
**********************************************************************/

#include "StdAfx.h"

#include <SmTorus.h>
#include <SmCircle.h>
#include <SmGeomUtility.h>
#include <nurbs.h>
#include <SmNurbsSrf.h>
#include <SmAssertArray.h>
#include <SmDatabaseIO.h>

#ifdef SM_DEBUG_CODE
#include <SmFace.h>
#include <SmBrep.h>
#endif // SM_DEBUG_CODE
                                       
/*******************************************************************//**
PURPOSE: Create a Torus given (1) origin, (2) local X,Y axes,
    (3) angular parameter range and (4) major & minor radii

NOTES: This Torus should be used when surface operations (like
    SS-Intersection) may take advantage of its analytic properties.
***********************************************************************/
SmTorus::SmTorus
 (const SmPoint3d  & crOrigin,         // in : center of major circle
  const SmVector3d & crXAxis,          // in : major circle start/end direction [0 and 360 degrees] 
  const SmVector3d & crYAxis,          // in : major circle 90 degree direction
  const SmExtent2d & crAnalUVDomain,   // in : [major circle interval, minor circle interval] in degrees
  double             dMajorRadius,     // in : major circle radius
  double             dMinorRadius,     // in : minor circle radius
  SmBoolean          bSwapUV,          // in : TRUE = underlying NURB UV directions will be switched
  SmBoolean          bInsideOut,       // in : TRUE = GenCurve runs in reverse direction from defintion 
                                       //      FALSE=
  SmBSplineCurve   * pOptCircleNurb,   // in : Optional Curve Pointer to define GenCurve parameterization
                                       //      SurfOfRevolution owns this curve and will delete it when destructed.
  const SmContext  * cpContext)        // in : Set context if given, default:[NULL]
: SmSurfOfRevolution(pOptCircleNurb,crOrigin,crXAxis,crYAxis,crAnalUVDomain,bSwapUV),
  m_dMajorRadius(dMajorRadius),
  m_dMinorRadius(dMinorRadius)
{
  // tori musthave positive radii
  SM_ASSERT(   dMajorRadius > SM_EFF_ZERO
            && dMinorRadius > SM_EFF_ZERO) ;

  // Domain limits
  SmExtent1d sSweepIvl = crAnalUVDomain.GetUInterval() ;
  SmExtent1d sGenIvl   = crAnalUVDomain.GetVInterval() ;

  SM_ASSERT(   sSweepIvl.GetMin()    >= -360.0 - SM_EFF_ZERO
            && sSweepIvl.GetMax()    <=  360.0 + SM_EFF_ZERO
            && sSweepIvl.GetLength() <=  360.0 + SM_EFF_ZERO);
  SM_ASSERT(   sGenIvl.GetMin()      >= -360.0 - SM_EFF_ZERO
            && sGenIvl.GetMax()      <=  360.0 + SM_EFF_ZERO
            && sGenIvl.GetLength()   <=  360.0 + SM_EFF_ZERO);

  // when not given a GenCurve to use
  if(m_pGenCurve == NULL)
    {
      // get minor circle parameters
      const SmContext * pContext  = GetContext();
      SmVector3d        sZAxis     = crXAxis * crYAxis;
      SmPoint3d         sCircleCenter(crOrigin + dMajorRadius*crXAxis);
      SmCircle        * pNewCircle = NULL ; 
      SmCircle::CreateCanonical(*pContext,
                                SmAxis2Placement(sCircleCenter,
                                                 crXAxis,
                                                 crXAxis * crYAxis),
                                dMinorRadius,
                                pNewCircle,
                                &sGenIvl,
                                &bInsideOut) ;
      SM_ASSERT(pNewCircle != NULL) ;
      m_pGenCurve = pNewCircle;
      m_pGenCurve->SetOwner(this) ;
    }

  // set context if given
  if(cpContext) 
    { m_cpContext = cpContext ; }

  // check for self intersections
  if(dMajorRadius < dMinorRadius)
    {
      // surface will self intersect unless saved by trimming
      double dAngleLimitDeg = 90.0 + smos_ArcCosine(dMajorRadius / dMinorRadius) * 180.0 / SM_PI ;
      SmExtent1d sSelfIntersectIvl(dAngleLimitDeg, 360.0 - dAngleLimitDeg) ;
      if(!sSelfIntersectIvl.AreDisjointPeriodic(sGenIvl, 360.0))
        {
          WARN(_T("Constructed Self Intersecting Torus: MajorRadius < MinorRadius\n   either change the radii or limit domain")) ;
        }
    }

  SM_ASSERT(m_pGenCurve != NULL) ;

} // end SmTorus::SmTorus constructor

/*******************************************************************//**
PURPOSE: Copy constructor for SmTorus

NOTES: 
***********************************************************************/
SmTorus::SmTorus
 (const SmTorus & crSource)
: SmSurfOfRevolution(crSource),   
  m_dMajorRadius(crSource.m_dMajorRadius),
  m_dMinorRadius(crSource.m_dMinorRadius)
{

} // end SmTorus::SmTorus constructor

/*******************************************************************//**
PURPOSE: Equality operator for SmTorus

NOTES: Call base equivalence to check type and then check 
       members for equivalence
***********************************************************************/
SmBoolean SmTorus::operator==
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
      SmTorus &rOther = (SmTorus &)crOther ;

      // check equivalence of these objects
      bRtn =  (   SM_IS_ZERO(m_dMajorRadius - rOther.m_dMajorRadius)
               && SM_IS_ZERO(m_dMinorRadius - rOther.m_dMinorRadius)
               && m_bInsideOut == rOther.m_bInsideOut) ;
    }

  // all done
  return bRtn ;

} // end SmTorus::operator==

/*******************************************************************//**
PURPOSE: Copy a Torus.

NOTES: 
***********************************************************************/
SmStatus SmTorus::Copy
  (const SmContext & crContext,
   SmSurface *& rpNewSurface) 
  const
{
    SmTorus *pCopy = new(crContext) SmTorus(*this);
    NER(pCopy);
    rpNewSurface = pCopy;
    return SM_SUCCESS;

} // end SmTorus::Copy
      
/*******************************************************************//**
PURPOSE: Method to create a canonical Torus object.

NOTES: 
***********************************************************************/
SmStatus SmTorus::CreateCanonical
 (const SmContext        & crContext,      // in : new object construction
  const SmAxis2Placement & crOrigin,       // in : m_vOrigin = center of torus 
                                           //      m_vXAxis  = Sweep Start
                                           //      m_vYAxis  = Sweep 90 degree direction
  double                   dMajorRadius,   // in : radius of offset circle centerd on m_vOrigin
  double                   dMinorRadius,   // in : radius of genCurve Circle being swept
  SmTorus               *& rpNewTorus,     // out: new object
  SmBoolean                bSwapUV,        // in : TRUE = underlying NURB UV directions will be switched
  SmBoolean                bInsideOut,     // in : TRUE = GenCurve runs in reverse direction from defintion 
                                           //      FALSE=
  SmBSplineCurve          *pOptCircleNurb, // in : Optional Curve Pointer to define GenCurve parameterization
                                           //      SurfOfRevolution owns this curve and will delete it when destructed.
  const SmExtent2d       * cpOptAnalUVDomain) // in : Analytic UV Domain [u=rotation in degrees, v=genCurve param]

{
  // Degenerate-radius handling (kept parallel with SmSphere::CreateCanonical; a torus has two radii,
  // so a negative radius has no well-defined inside-out meaning and is treated as degenerate):
  //   - minor >= major   -> valid "spindle"/self-intersecting torus for some trimmed domains: allow
  //     it (the old ordering SM_ASSERT encoded the wrong invariant and is removed).
  //   - non-positive minor or major -> genuinely degenerate (no valid NURB): reject cleanly here
  //     instead of crashing on the undefined geometry downstream (the ctor SM_ASSERT is a no-op in
  //     release).
  if (!(dMinorRadius > SM_EFF_ZERO) || !(dMajorRadius > SM_EFF_ZERO))
    {
      rpNewTorus = NULL;
      SE_MSG(SM_ERR, _T("SmTorus::CreateCanonical: degenerate (non-positive) radius; cannot create torus")) ;
      return SM_ERR;
    }

  // In the future, allow user to specify angles
  SmExtent2d sAnalUVDomain = (cpOptAnalUVDomain) ? *cpOptAnalUVDomain : SmExtent2d(0.0, 0.0, 360.0, 360.0);

  rpNewTorus = new (crContext) SmTorus(crOrigin.GetOriginRef(),
                                       crOrigin.GetXAxisRef(),
                                       crOrigin.GetYAxisRef(),
                                       sAnalUVDomain,
                                       dMajorRadius, dMinorRadius,
                                       bSwapUV, bInsideOut, pOptCircleNurb );
  NER(rpNewTorus);

  // Build NURB; fail cleanly rather than leak a null-NURB object.
  if (rpNewTorus->MakeNurb() != SM_SUCCESS || rpNewTorus->GetGwNurbPointer() == NULL)
    {
      delete rpNewTorus; rpNewTorus = NULL;
      SE_MSG(SM_ERR, _T("SmTorus::CreateCanonical: failed to build NURB representation")) ;
      return SM_ERR;
    }

  // all done
  return SM_SUCCESS;

} // end SmTorus::CreateCanonical

/*******************************************************************//**
PURPOSE: Create an offset torus for the torus.

NOTES: 
 1.) The offset direction is in the direction of surface's NurbBspline
      Normal direction
 2.) No surface is built when new minor radius is less than equal to zero
     or the surface will be self intersecting.
 3.) OffsetSurface domains == OriginalSurface domains

NOTES: The offset Nurb and Step domains map from old to new
     so that   pOffSet->EvaluateSTEPPoint(StepUV) = pInput->EvaluateSTEPPoint() + dOff * dNormal(StepUV)
    and        pOffSet->EvaluatePoint(NurbUV)     = pInput->EvaluatePoint()     + dOff * dNormal(NurbUV)
***********************************************************************/
SmStatus SmTorus::CreateOffsetSurface
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

  // When UVs are swapped - reverse orientation of offset 
  // When InsideOut       - reverse orientation of offset
  double dOffsetSign =   1.0                   
                       * (m_bSwapUV    ? -1.0 : 1.0) ;
  
  // get new radius
  //double dMajRadius    = m_dMajorRadius ;
  double dOldMinRadius = m_dMinorRadius ; 
  double dNewMinRadius = m_dMinorRadius + dOffsetSign * dSignedOffsetDistance;

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
      SM_ASSERT(   dNewMinRadius == dOldMinRadius
                || dStepDir * (dNewMinRadius - dOldMinRadius) > 0.0) ; 
    }
#endif // SM_DEBUG_CODE

  // skip building degenerate or negative radius minor circles
  // gwc: currently we can construct self intersection tori when
  //      majorRadius < minorRadius.  A warning is signaled in the constructor.
  if(   dNewMinRadius < dThisApproxTol3d)
    { return SM_SUCCESS; }

  // copy the current torus
  SmTorus *pOffTorus = new(crContext) SmTorus(*this);
  SM_ASSERT(pOffTorus->m_pNurb     != NULL) ;
  SM_ASSERT(pOffTorus->m_pGenCurve != NULL) ; 
  SmObjDelete sClean(pOffTorus) ;

  // change the minor radius
  pOffTorus->m_dMinorRadius = dNewMinRadius ;

  // scale the NewTorus->GenCurve about circle center
  if(pOffTorus->m_pGenCurve)
    {
      SM_ASSERT(pOffTorus->m_pGenCurve && pOffTorus->m_pGenCurve->IsKindOf(SmCircle_TYPE)) ;
      SmCircle                *pCircle         = (SmCircle *)pOffTorus->m_pGenCurve ;
      const SmAxis2Placement & rCirclePosition = pCircle->GetPosition() ;
      double           dScale = dNewMinRadius/dOldMinRadius ;
      SmVector3d       sMoveVec(  (1.0-dScale) 
                                * rCirclePosition.GetOriginRef()) ;
      SmVector3d       sScale(dScale, dScale, dScale) ;
      SmAxis2Placement sMove ;
      sMove.Translate(sMoveVec) ;
      pOffTorus->m_pGenCurve->Transform(sMove, &sScale) ; 
    }

  // rebuild the underlying Nurb Surface
  //   Just need to move the Nurb Points of the 
  //   m_pNurb. GenCurve has already been done
  int ii, jj ;
  gw_CPOINT        sEuclid ;
  gw_SURFACE      *pOffNurb = pOffTorus->GetGwNurbPointer() ;
  const SmPoint3d  &rOrigin = m_vPosition.GetOriginRef() ;
  SmVector3d       sZAxis   = m_vPosition.GetZAxis() ;
  double dFirstVecPSize = 0.0 ;

  // offset the nurb by combining a scale and a translate 
  //   Scale all points from Torus origin increasing both major and minor radii.
  //   Translate every circular cross section back towards the
  //     torus Z axis so that the major radius is restored.
  //     This requires cross sections on full knots to be moved back
  //     by a constant amount.  Other knots are moved back a larger
  //     distance proportional to the distance theseknots are from the
  //     Z axis compared to their mated knots in a cross section
  //     from a full not.
  double dScale = dNewMinRadius/dOldMinRadius;
  double dTrans = (1.0 - dScale) * m_dMajorRadius ;

  // For every m_pNurb ControlPoint - make inner iteration the Sweep direction
  int iCnt, jCnt ;
  if(m_bSwapUV) { iCnt = pOffNurb->net->n ;
                  jCnt = pOffNurb->net->m ;
                }
  else          { iCnt = pOffNurb->net->m ;      
                  jCnt = pOffNurb->net->n ;
                }
  for(ii=0;ii<=iCnt;ii++)
    {
      // move every control point on this sweep curve row
      for(jj=0;jj<=jCnt;jj++)
        {
          // get Control Point Euclidean position
          gw_CPOINT *pCP = m_bSwapUV ? &(pOffNurb->net->Pw[ii][jj])
                                     : &(pOffNurb->net->Pw[jj][ii]) ;
          TO_EUCLID(*pCP, sEuclid) ;
#ifdef SM_DEBUG_CODE
          SmPoint3d sPointBefore(sEuclid.x, sEuclid.y, sEuclid.z) ;
#endif
          // get scale and move directions for this ControlPoint
          SmVector3d sVec(sEuclid.x - rOrigin.x,
                          sEuclid.y - rOrigin.y,
                          sEuclid.z - rOrigin.z) ;
          SmVector3d sVecP = sVec - sVec.Dot(sZAxis) * sZAxis ;

          // scale VecP so that the translates are larger in the corners
          if(jj == 0) { dFirstVecPSize = sVecP.Length() ; }
          sVecP = sVecP/dFirstVecPSize ; // its not normalized for corner CPs
                                         // it contains the extra scale factor,
                                         //  dThisVecPSize/dFirstVecPSize.
          // move the ControlPoint
          pCP->x = (rOrigin.x + sVec.x*dScale + sVecP.x*dTrans) * pCP->w ;
          pCP->y = (rOrigin.y + sVec.y*dScale + sVecP.y*dTrans) * pCP->w ;
          pCP->z = (rOrigin.z + sVec.z*dScale + sVecP.z*dTrans) * pCP->w ;

#ifdef SM_DEBUG_CODE
          if (bDebugMe)                      
            { TO_EUCLID(*pCP, sEuclid) ; 
              SmPoint3d sPointAfter(sEuclid.x, sEuclid.y, sEuclid.z) ;
              smgfx_SetLook(5,6, 0,1,0) ; sPointBefore.Draw() ; sm_GraphicsLoop() ;
              smgfx_SetLook(7,8, 1,0,0) ; sPointAfter.Draw() ; sm_GraphicsLoop() ;
              sm_GraphicsLoop();
            }
#endif
        } // end iter every jjth ControlPoint
    } // end iter every iith ControlPoint

  // old style - resets the nurb parameterization
//        pOffTorus->MakeNurb() ;

  // set output
  sClean.Clear() ;
  rOffsetSurface = pOffTorus ;

#ifdef SM_DEBUG_CODE
  if (bDebugMe) 
    {
      this->Dump() ;
      pOffTorus->Dump() ;
      if(m_pNurb)
        {
          SmExtent2d sInputNurbDomain  = GetNaturalUVDomain() ;
          SmExtent2d sInputStepDomain  = GetSTEPUVDomain() ;
          SmExtent2d sOffsetNurbDomain = pOffTorus->GetNaturalUVDomain() ;
          SmExtent2d sOffsetStepDomain = pOffTorus->GetSTEPUVDomain() ;
          SM_ASSERT(   sInputNurbDomain.IsContainedBy(sOffsetNurbDomain)
                    && sOffsetNurbDomain.IsContainedBy(sInputNurbDomain, SM_EFF_ZERO)) ;
          SM_ASSERT(   sInputStepDomain.IsContainedBy(sOffsetStepDomain, SM_EFF_ZERO)
                    && sOffsetStepDomain.IsContainedBy(sInputStepDomain, SM_EFF_ZERO)) ;
        }

      SmFace *pFace = (SmFace *)GetFace() ;
      SmBrep *pBrep = pFace ? pFace->GetBrep() : NULL ;

      smgfx_Erase();
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,1) ; DrawUV(3,6); sm_GraphicsLoop();
      smgfx_SetLook(1,2, 0,1,0) ; pOffTorus->DrawUV(3,6); sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
#endif
  
  // all done
  SM_DUMP_AND_ASSERT2_VALID(pOffTorus) ;
  return SM_SUCCESS;

} // end SmTorus::CreateOffsetSurface

/*******************************************************************//**
PURPOSE: Convert from STEP to NURBS parameterization
            so that EvaluatePoint(NurbUV) = EvaluateSTEPPoint(StepUV)

NOTES: 
***********************************************************************/
SmStatus SmTorus::ConvertUVFromSTEPToNURBS
  (const SmPoint2d & crSTEPUV,  // in : Target 2d Point in [degrees, height] 
   SmPoint2d       & rNURBSUV)  // out: 2d Point in SMLib parameterization            
  const
{
  SM_ASSERT(HavePolarConversion()) ;

  // convert sweep coordinate
  SmBoolean bExactConversion;
  SER(m_vPolarConverter.ConvertToNURBSParameter(crSTEPUV.x,rNURBSUV.x,bExactConversion));
  SM_ASSERT(bExactConversion) ;

//        // Torus genCurve parameter spaces:
//        //   1. StepParams for a 'STEPGenCurve' are in degrees from [-90 to +90]
//        //   2. PolarParams for a 'PolarCurve' are in degrees either 
//        //             from [-90 to +90] or [+90 to -90]
//        //   3. Nurb params are determined by the genCurve's knot vector but
//        //      always run in the same general direction as the polarParams
//      
//        // Get Gencurve PolarParam from StepParam. Because Torus GenCurves
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
      double dInvert =   (m_bSwapUV    ? -1.0 : 1.0)
                       * (m_bInsideOut ? -1.0 : 1.0) ;

      // normals should be related
      SM_ASSERT(SM_ARE_SAME(1.0, dInvert * sStepZ.Dot(sNurbZ))) ;   
    }

  if(bDebugMe)
    {
      Dump() ;
    }
#endif

  // all done
  return SM_SUCCESS;    

} // end SmTorus::ConvertUVFromSTEPToNURBS

/*******************************************************************//**
PURPOSE: Convert from NURBS to STEP parameterization 
            so that EvaluatePoint(NurbUV) = EvaluateSTEPPoint(StepUV).

NOTES: 
***********************************************************************/
SmStatus SmTorus::ConvertUVFromNURBSToSTEP
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

  // Torus genCurve parameter spaces:
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

} // end SmTorus::ConvertUVFromNURBSToSTEP

/*******************************************************************//**
PURPOSE: Evaluate the point & derivatives using the STEP parameterization.
            EvaluatePoint(NurbUV) = EvaluateSTEPPoint(StepUV).
            But tangents will vary in magnitude and
            norm may be negated

NOTES:     
    S(U,V) =   origin                             
             + MajRadius * cos(U) * XAxis   
             + MajRadius * sin(U) * YAxis
             + MinRadius * cos(V) * (  cos(U) * XAxis
                                     + sin(U) * YAxis)
             + MinRadius * sin(V) * ZAxis

***********************************************************************/
SmStatus SmTorus::EvaluateSTEP
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
  SmPoint2d  sUV    = crUV ;
  SmPoint3d  rO     = m_vPosition.GetOriginRef() ;
  SmVector3d rX     = m_vPosition.GetXAxisRef () ;
  SmVector3d rY     = m_vPosition.GetYAxisRef () ;
  SmVector3d sZ     = m_vPosition.GetZAxis () ;
  double     dMajR  = m_dMajorRadius ;
  double     dMinR  = m_dMinorRadius ;
  ULONG lMinHighest = smos_Min(lHighestUDeriv, lHighestVDeriv) ;

  // per SMLib practice clamp out of bounds UV points
  if (!m_vAnalUVDomain.ContainsPoint2d(sUV)) 
    { sUV = m_vAnalUVDomain.ClampPoint2d(sUV); }

  // evaluate expensive trigonometric functions
  double dCosU = smos_Cosine(sUV.x*SM_PI/180.0) ;
  double dSinU = smos_Sine  (sUV.x*SM_PI/180.0) ;
  double dCosV = smos_Cosine(sUV.y*SM_PI/180.0) ;
  double dSinV = smos_Sine  (sUV.y*SM_PI/180.0) ;

  // evaluate position
  aDerivatives[0] = rO + (dMajR + dMinR * dCosV) * (  dCosU * rX
                                                    + dSinU * rY)
                       + (dMinR * dSinV * sZ) ;

  // The derivatives
  //        Su = (dMajR + dMinR * dCosV) * ( -dSinU * rX
  //                                         +dCosU * rY) ;
  //        Sv =   (dMinR * -dSinV) * (  dCosU * rX 
  //                                   + dSinU * rY)
  //             + (dMinR * dCosV * sZ) ;               
  //        Suu = (dMajR + dMinR * dCosV) * ( -dCosU * rX
  //                                          -dSinU * rY) ;
  //        Svv =   (dMinR * -dCosV) * (  dCosU * rX 
  //                                    + dSinU * rY)
  //              - (dMinR * dSinV * sZ) ;
  //        Suv = (dMinR * -dSinV) * (- dSinU * rX 
  //                                  + dCosU * rY)
  //        Suuu = (dMajR + dMinR * dCosV) * (  dSinU * rX
  //                                           -dCosU * rY) ;
  //        Svvv =   (dMinR *  dSinV) * (  dCosU * rX 
  //                                     + dSinU * rY)
  //               - (dMinR * dCosV * sZ) ;
  //        Suuv = (dMinR * -dSinV) * ( -dCosU * rX
  //                                    -dSinU * rY) ;
  //        Suvv = (dMinR * -dCosV) * (- dSinU * rX 
  //                                   + dCosU * rY)

  // 1st U and V derivatives
  if(lHighestUDeriv >= 1)   { aDerivatives[  lHighestVDeriv+1] = (dMajR + dMinR * dCosV) * ( -dSinU * rX
                                                                                             +dCosU * rY) ; }
  if(lHighestVDeriv >= 1)   { aDerivatives[1]                  =   (dMinR * -dSinV) * (  dCosU * rX 
                                                                                       + dSinU * rY)
                                                                 + (dMinR * dCosV * sZ) ; }
                            
  // cross derivative dUV
  if(   lHighestUDeriv >= 1
     && lHighestVDeriv >= 1
     && lMinHighest    >= 2){ aDerivatives[  lHighestVDeriv+2] = (dMinR * -dSinV) * (- dSinU * rX 
                                                                                     + dCosU * rY) ; }

  //  cross derivative dUVV
  if(   lHighestUDeriv >= 1
     && lHighestVDeriv >= 2
     && lMinHighest    >= 3){ aDerivatives[  lHighestVDeriv+3] = (dMinR * -dCosV) * (- dSinU * rX 
                                                                                     + dCosU * rY) ; }

  // 2nd U and V derivatives
  if(lHighestUDeriv >= 2)   { aDerivatives[2*lHighestVDeriv+2] = (dMajR + dMinR * dCosV) * ( -dCosU * rX
                                                                                             -dSinU * rY) ; }
  if(lHighestVDeriv >= 2)   { aDerivatives[2]                  =   (dMinR * -dCosV) * (  dCosU * rX 
                                                                                       + dSinU * rY)
                                                                 - (dMinR * dSinV * sZ) ; }
                            
  // cross derivative dUUV
  if(   lHighestUDeriv >= 2
     && lHighestVDeriv >= 1
     && lMinHighest    >= 3){ aDerivatives[2*lHighestVDeriv+3] = (dMinR * -dSinV) * ( -dCosU * rX
                                                                                      -dSinU * rY) ; }

  // 3rd U and V derivatives
  if(lHighestUDeriv >= 3)   { aDerivatives[3*lHighestVDeriv+3] = (dMajR + dMinR * dCosV) * (  dSinU * rX
                                                                                             -dCosU * rY) ; }
  if(lHighestVDeriv >= 3)   { aDerivatives[3]                  =   (dMinR *  dSinV) * (  dCosU * rX 
                                                                                        + dSinU * rY)
                                                                  - (dMinR * dCosV * sZ) ; }

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

} // end SmTorus::EvaluateSTEP

/*******************************************************************//**
PURPOSE: Evaluate the point using the STEP parameterization.

NOTES: crUV:[0_to_360, -90_to_90]
  when Torus was built canonically, m_bMakeNurbGenCurve = FALSE
       and GenCurve will be an SmCircle object.
    U = CCW angle in degrees about Z axis from X axis [0_to_360] and
    V = angle in degrees from bot pole to top pole, [-90_to_90].

  when Torus was built with m_bMakeNurbGenCurve = TRUE
       GenCurve will be an SmBSplineCurve object.
    U = CCW angle in degrees about Z axis from X axis [0_to_360] and
    V = Nurb Param from bot pole to top pole, [-90_to_90] but not exactly
        equal to an angle.
***********************************************************************/
SmStatus SmTorus::EvaluateSTEPPoint
  (const SmPoint2d & crUV,   // in : target UVPoint, range:[0 to 360, -90 to 90]
   SmPoint3d       & rPoint) // out: sphere point
  const
{
  // pass the call along
  return(EvaluateSTEP(crUV, 0, 0, TRUE, TRUE, TRUE, &rPoint)) ;

} // end SmTorus::EvaluateSTEPPoint

/*******************************************************************//**
PURPOSE: Create an 3D iso-parametric curve of a torus given the 
    Nurb parameter direction (U or V) and the constant parameter 
    in that direction.

VIRTUAL FUNCTION ---
    for SmBSplineSurface - Make a BSpline Curve          (exact)
        SmTorus          - Make a SmCircle Curve         (exact)
        SmPlane          - Make a Line                   (exact)
        SmCone           - Make a Line or SmCircle Curve (exact)
        SmTorus          - Make a SmCircle curve         (exact)
        All Others       - Make a piecewise Hermite Curve approximation
                              good to optional tolerance or
                              dLength * SM_EFF_ZERO_SQRT * 100.0.
***********************************************************************/
SmStatus SmTorus::CreateIsoParametricCurve
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
                                             pOptMaxGap3d,     // out: opt achieved max gap, NULL to ignore, default:[NULL]
                                             pOptUVIsoCurve) ; // out: opt 2d UVTrimCurve Line (diff parameterization), NULL to ignore, default:[NULL]

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
  const SmPoint3d  &rTorusOrigin = m_vPosition.GetOriginRef() ;
  const SmVector3d &rTorusXAxis   = m_vPosition.GetXAxisRef() ;
  const SmVector3d &rTorusYAxis   = m_vPosition.GetYAxisRef() ;
  SmVector3d        sTorusZAxis   = m_vPosition.GetZAxis() ;

  // Get IsoCurve Nurb endPoints
  SmPoint3d sNurbMinPoint, sNurbMaxPoint ;
  SmExtent1d sNurbIvl = pIsoCurve->GetNaturalInterval() ;
  pIsoCurve->EvaluatePoint(sNurbIvl.GetMin(), sNurbMinPoint) ;
  pIsoCurve->EvaluatePoint(sNurbIvl.GetMax(), sNurbMaxPoint) ;
  SmVector3d sNurbMinVec = sNurbMinPoint - rTorusOrigin ;
  SmVector3d sNurbMaxVec = sNurbMaxPoint - rTorusOrigin ;

  // get iso circle parameters
  SmBoolean  bInsideOut ;
  SmPoint3d  sCircleCenter ;
  SmVector3d sXAxis, sYAxis ;
  double     dRadius ;
  SmExtent1d sAnalDomain ;
  if(eSTEPParam == SM_SP_U) // return minor isoCurve circle
    {
      sXAxis         = sTorusZAxis * (sNurbMinVec * sTorusZAxis) ;
      double dLength = sXAxis.Length() ;
      if(dLength < SM_EFF_ZERO_SQRT)
        {
          SmPoint3d sNurbMidPoint ;
          pIsoCurve->EvaluatePoint(sNurbIvl.Evaluate(.5), sNurbMidPoint) ;
          SmVector3d sNurbMidVec = sNurbMidPoint - rTorusOrigin ;
          sXAxis = sTorusZAxis * sNurbMinVec * sTorusZAxis ;
          dLength = sXAxis.Length() ;
        }
      SM_ASSERT(sXAxis.Length() > SM_EFF_ZERO_SQRT) ;
      sXAxis        = sXAxis/dLength ;
      sCircleCenter =   m_vPosition.GetOriginRef()
                      + m_dMajorRadius * sXAxis ;
      sYAxis        = sTorusZAxis ;
      dRadius       = m_dMinorRadius ;
      bInsideOut    = m_bInsideOut ;
      sAnalDomain   = m_vAnalUVDomain.GetVInterval() ;
    }
  else // return sweep isoCurve circle
    {
      sCircleCenter =   m_vPosition.GetOriginRef() 
                      + sNurbMinVec.Dot(sTorusZAxis) * sTorusZAxis ;
      sXAxis        = rTorusXAxis ;
      sYAxis        = rTorusYAxis ;
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
      if(   sAnalDomain.IsClosed(360.0)
         && sAnalDomain.IsValueOnPeriodicBoundary(dMinAngDeg, 360.0)
         && sAnalDomain.IsValueOnPeriodicBoundary(dMaxAngDeg, 360.0))
        {
          // set interval to whole domain
          sCircleAnalDomain = sAnalDomain ;
        }
      else
        { 
          // set isoCurve analytic domain accounting for m_binside reversals
          if ( smos_Fabs( dMaxAngDeg-dMinAngDeg ) < sAnalDomain.GetLength()-SM_EFF_ZERO )
            { dMaxAngDeg = dMinAngDeg + sAnalDomain.GetLength(); }
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

} // end SmTorus::CreateIsoParametricCurve

/*******************************************************************//**
PURPOSE: Method to get canonical data from Cone object.

NOTES: 
***********************************************************************/
SmStatus SmTorus::GetCanonical
  (SmAxis2Placement & rOrigin,         // out: m_vOrigin = STEP BotCircle Center Point    
                                       //      m_vXAxis  = STEP Surface  0 degree rotation
                                       //      m_vYAxis  = STEP Surface 90 degree rotation 
   double           & rdMajorRadius,   // out: radius from torus center to minor circle center
   double           & rdMinorRadius,   // out: radius of minor circle
   SmBoolean        * pOptSwapUV,      // out: TRUE = Nurb and Analytic U and V directions are swapped
                                       //      NULL to ignore, default:[NULL]
   SmBoolean        * pOptInsideOut)   // out: TRUE = Nurb and Analytic genCurve directions are swapped
                                       //      NULL to ignore, default:[NULL]
 const
{
  rOrigin = m_vPosition;
  rdMajorRadius = m_dMajorRadius;
  rdMinorRadius = m_dMinorRadius;
  if(pOptSwapUV)     { *pOptSwapUV    = m_bSwapUV ; }  
  if(pOptInsideOut)  { *pOptInsideOut = m_bInsideOut ; }

    return SM_SUCCESS;

} // end SmTorus::GetCanonical

/*******************************************************************//**
PURPOSE: Static function to determine if a given nurbs is a Torus.
            If it is a Torus then one is created.

NOTES: 
***********************************************************************/
SmBoolean SmTorus::IsNurbSurfaceTorus
  (const SmContext        & crContext,       // in : new object context
   const SmBSplineSurface * pTestSurface,    // in : BSplineSurface to be checked 
   SmTorus               *& rpTorus,         // out: new Torus or NULL
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

  // evaluate Nurb min/max domain points
  SmExtent2d sNurbUVDomain = pTestSurface->GetNaturalUVDomain();
  SmPoint3d  sNurbMinPoint, sNurbMaxPoint;
  pTestSurface->EvaluatePoint(sNurbUVDomain.GetMin(),sNurbMinPoint);
  pTestSurface->EvaluatePoint(sNurbUVDomain.GetMax(),sNurbMaxPoint);

  // scale zero value
  double dScaledZero =   dToleranceScale  
                       * ANALYTIC_TOL_SCALE * SM_EFF_ZERO 
                       * (1.0 + sNurbMinPoint.GetMaxDimension() + sNurbMaxPoint.GetMaxDimension());
  double dAngTol     = SM_EFF_ZERO_SQRT * dToleranceScale;

  // locals
  SmBoolean bFoundCirclesU, bParallelPlanesU, bSamePlaneU, bSameCenterU, bCommonAxisU, bPlanarGenCurveU ;
  SmBoolean bFoundCirclesV, bParallelPlanesV, bSamePlaneV, bSameCenterV, bCommonAxisV, bPlanarGenCurveV ;
  SmExtent1d sAnglesU, sAnglesV;
  double dRadiusU, dRadiusV;
  SmAxis2Placement sRefFrameU, sRefFrameV;
  SmPoint3d sCommonPointU, sCommonVecU ;
  SmPoint3d sCommonPointV, sCommonVecV ;

  // test sequence of V Direction (U Constant) isoCurves for circularity and parallelism
  SER(sm_TestForIsoCircles(pTestSurface, SM_SP_U, dScaledZero, dAngTol,
                              bFoundCirclesU, bParallelPlanesU, bSamePlaneU,
                              bSameCenterU, bCommonAxisU, bPlanarGenCurveU,
                              dRadiusU, sAnglesU, sRefFrameU,
                              sCommonPointU, sCommonVecU));
  // quit when not a torus
  if (!bFoundCirclesU) return FALSE;

  // test sequence of U Direction (V Constant) isoCurves for circularity and parallelism
  SER(sm_TestForIsoCircles(pTestSurface, SM_SP_V, dScaledZero, dAngTol,
                              bFoundCirclesV, bParallelPlanesV, bSamePlaneV,
                              bSameCenterV, bCommonAxisV, bPlanarGenCurveV,
                              dRadiusV, sAnglesV, sRefFrameV,
                              sCommonPointV, sCommonVecV));

  // quit when not a torus
  if (!bFoundCirclesV) return FALSE;

  // see if we have a U or a V oriented torus
  SmBoolean bIsTorusWithSwapUV = (bParallelPlanesU && ! bSamePlaneU && bCommonAxisV);
  SmBoolean bIsTorusNoSwapUV   = (bParallelPlanesV && ! bSamePlaneV && bCommonAxisU);

  // quit when not a properly oriented torus
  if(!bIsTorusWithSwapUV && !bIsTorusNoSwapUV)
    { return FALSE; }

  // locals
  double      dMinRad ;
  SmExtent1d *pSweepIvl ;
  SmPoint2d   sMidUV    = sNurbUVDomain.Evaluate(0.5,0.5);
  SmPoint2d   sMinUV    = sNurbUVDomain.Evaluate(0.0,0.0);
  SmBoolean   bSwapUV ;
  SmAxis2Placement *pRefFrame, *pOtherFrame ;
  SmVector3d sCommonPoint, sCommonVec ;

  // Get torus position and parameters
  if(bIsTorusNoSwapUV)
    {
      bSwapUV      = FALSE ;
      pRefFrame    = &sRefFrameV ;   // frame where all circles are parallel
      pOtherFrame  = &sRefFrameU ;   // frame where all circle planes intersect on a common line
      dMinRad      = dRadiusU ;
      pSweepIvl    = &sAnglesV ;
      sCommonPoint = sCommonPointU ; // from frame where all circle planes intersect on a common line
      sCommonVec   = sCommonVecU ;   // from frame where all circle planes intersect on a common line
    }
  else
    {
      SM_ASSERT(bIsTorusWithSwapUV) ;
      bSwapUV      = TRUE ;
      pRefFrame    = &sRefFrameU ; // frame where all circles are parallel                    
      pOtherFrame  = &sRefFrameV ; // frame where all circle planes intersect on a common line
      dMinRad      = dRadiusV ;
      pSweepIvl    = &sAnglesU ;
      sCommonPoint = sCommonPointV ; // from frame where all circle planes intersect on a common line
      sCommonVec   = sCommonVecV ;   // from frame where all circle planes intersect on a common line
    }

  // get torus orientation and size
  SmVector3d       sMajorVec          = pOtherFrame->GetOriginRef() - pRefFrame->GetOriginRef() ;
  double           dMajRad            = sMajorVec.Length() ;
  const SmPoint3d &crTorusXAxis       = pRefFrame->GetXAxisRef() ; // the frame where all circles are parallel
  const SmPoint3d &crTorusYAxis       = pRefFrame->GetYAxisRef() ;
  SmVector3d       sTorusZAxis        = pRefFrame->GetZAxis() ;

  // torus must have positive radii
  if(   dMinRad <= SM_EFF_ZERO
     || dMajRad <= SM_EFF_ZERO)
    { return(FALSE) ; }

  // get torus origin = MinorCircleCenter projected onto common line
  //   a MinorCircleCenter is stored in pOtherFrame->GetOriginRef()
  SM_ASSERT(SM_IS_ZERO(sCommonVec.Length() - 1.0)) ;
  SM_ASSERT(sCommonVec.IsParallelTo(sTorusZAxis, SM_EFF_ZERO * 100.0 * 360.0)) ;
  SmPoint3d sTorusOrigin =   sCommonPoint
                           + sCommonVec.Dot(pOtherFrame->GetOriginRef() - sCommonPoint)
                           * sCommonVec ;

  // Check GenCurve minor circle rotation direction, its y axis is either +/- sweepCircle z
  // assumes: OterhFrame is valid for 1st Nurb isoparameter GenCurve
  SmVector3d sMinorOrientation = pOtherFrame->GetZAxis() * sMajorVec ;
  SM_ASSERT(sMinorOrientation.IsParallelTo(sTorusZAxis, 2.0)) ;
  SmBoolean bInsideOut = sTorusZAxis.Dot(sMinorOrientation) < 0.0
                         ? TRUE
                         : FALSE ;

  // Get GenCurve angles from each genCurves center point coordinate system
  // project Nurb Min/Max Point to Torus XY plane to get vec to Center of each GenCurve
  SmVector3d sMinPlanePoint, sMaxPlanePoint ;
  smgu_PointProjectToPlane(sNurbMinPoint, sTorusOrigin, sTorusZAxis, sMinPlanePoint) ;
  smgu_PointProjectToPlane(sNurbMaxPoint, sTorusOrigin, sTorusZAxis, sMaxPlanePoint) ;
  SmVector3d sStartForMinAngle = sMinPlanePoint - sTorusOrigin ; 
  SmVector3d sStartForMaxAngle = sMaxPlanePoint - sTorusOrigin ;
  sStartForMinAngle.Unitize() ; 
  sStartForMaxAngle.Unitize() ;
  SmVector3d sMinOrigin = sTorusOrigin + dMajRad * sStartForMinAngle ; 
  SmVector3d sMaxOrigin = sTorusOrigin + dMajRad * sStartForMaxAngle ; 

  // Get Vec from Nurb Min/Max points to each genCurve origin
  SmVector3d sVecMin = sNurbMinPoint - sMinOrigin ;
  SmVector3d sVecMax = sNurbMaxPoint - sMaxOrigin ; 

  // get GenCurve min and max angles measured from each GenCurve's origin and start vector
  double dGenAngleMin, dGenAngleMax;
  SE(sStartForMinAngle.AngleBetween(sVecMin,dGenAngleMin));
  SE(sStartForMaxAngle.AngleBetween(sVecMax,dGenAngleMax));
  dGenAngleMin = 180.0 * dGenAngleMin / SM_PI;
  dGenAngleMax = 180.0 * dGenAngleMax / SM_PI;

  // snap to ends for small tolerances
  if(   smos_Fabs(dGenAngleMin) < SM_EFF_ZERO)       { dGenAngleMin = 0.0 ; }
  if(   smos_Fabs(dGenAngleMax) < SM_EFF_ZERO
     || smos_Fabs(dGenAngleMax-360.0) < SM_EFF_ZERO) { dGenAngleMax = 360.0 ; }
  if(dGenAngleMax < dGenAngleMin) { dGenAngleMax += 360.0 ; }

  // define Torus domain
  SmExtent2d sAnalUVDomain(pSweepIvl->GetMin(), dGenAngleMin, pSweepIvl->GetMax(), dGenAngleMax);

  // Get GenCurve Isoparameter Curve
  SmBSplineCurve *pGenCurveIsoCrv = NULL ;
  SER(pTestSurface->CreateIsoParametricCurve(crContext, 
                                             bSwapUV ? SM_SP_V  : SM_SP_U, 
                                             bSwapUV ? sMinUV.y : sMinUV.x, 
                                             0.0, 
                                             pGenCurveIsoCrv));
  SmObjDelete sCleanObj(pGenCurveIsoCrv) ;
  SmCircle *pCircle = new (crContext) SmCircle(sMinOrigin,
                                               sStartForMinAngle,
                                               sTorusZAxis,
                                               sAnalUVDomain.GetVInterval(),
                                               dMinRad, 3, &crContext,
                                               pGenCurveIsoCrv,
                                               bInsideOut) ;

  // build a new torus with given GenCurve to force parameterization compatibility
  //    constant U Curve orientations, 
  //    constant V Curve centers and radius, 
  //    and computed domain
  rpTorus = new (crContext) SmTorus(sTorusOrigin,
                                    crTorusXAxis,
                                    crTorusYAxis,
                                    sAnalUVDomain,
                                    dMajRad, dMinRad,
                                    bSwapUV,
                                    bInsideOut,
                                    pCircle);
  NER(rpTorus);
  // error recovery
  if (rpTorus->m_pGenCurve == NULL) { delete rpTorus; rpTorus = NULL; 
                                      return FALSE;
                                    }

  // arrive here after New Torus has been constructed. Need to
  //  1. set NewTorus->m_pNurb = inputSurface->m_pNurb
  //  2. Copy InputSurface->Attributes onto NewTorus.
  //  3. make a Torus PolarConverter for rotation angles

  // reparameterize the NURB so that AnalyticSurface->NurbDomain == InputSurface->NurbDomain
  rpTorus->Reparameterize(pTestSurface->GetNaturalUVDomain()) ;

  // obsolete - it's a mistake to introduce tol sized variations between the Nurb and Anal reps by copying the m_pNurb object
  // Copy pTestSurface->m_pNurb into newTorus
  //      SM_ASSERT(rpTorus->m_pNurb == NULL) ;
  //      rpTorus->m_pNurb = sm_AllocateAndCopyNurbSurface(pTestSurface->GetGwNurbPointer()) ;
  //      
  //      // Build newTorus PolarConverter from Nurb midPoint rotation isoParamCurve
  //      rpTorus->RebuildPolarConverter() ;
  // end obsolete

  // Copy pTestSurface attributes onto newTorus
  ((SmBSplineSurface*)pTestSurface)->Notify(SM_NO_COPY, rpTorus, SM_NO_GET_OWNER(rpTorus), SM_NO_GET_OWNER(pTestSurface));

#ifdef SM_DEBUG_CODE
  if (bDebugMe) 
    {
      SmBSplineCurve *pIso = NULL ;
      if (rpTorus->m_bSwapUV) { SER(rpTorus->CreateIsoParametricCurve(crContext,SM_SP_U,0.0,SM_EFF_ZERO,pIso));
                              }
      else                    { SER(rpTorus->CreateIsoParametricCurve(crContext,SM_SP_V,0.0,SM_EFF_ZERO,pIso));
                              }
      SmObjDelete sCleanIso(pIso);

      // dump input surface, output surface, sweep isoCurve, genCurve
      pTestSurface->Dump();
      rpTorus->Dump();
      rpTorus->m_pGenCurve->Dump();
      pIso->Dump();

      smgfx_Erase();
      smgfx_SetLook(1,2, 0,0,1); rpTorus->DrawUV(); sm_GraphicsLoop();
      smgfx_SetLook(3,4, 1,0,0); rpTorus->m_pGenCurve->Draw(); sm_GraphicsLoop();
      smgfx_SetLook(6,7, 1,0,1); pIso->Draw(); sm_GraphicsLoop();
      smgfx_SetLook(3,4, 0,1,0); pTestSurface->DrawUV(4,4); sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
#endif

  // all done
  SM_DUMP_AND_ASSERT2_VALID(rpTorus) ;
  return TRUE;

} // end SmTorus::IsNurbSurfaceTorus

/*******************************************************************//**
PURPOSE: Rebuild all analytic, GenCurve, and PolarCurve cached data 
  from the m_pNurb data. 

NOTES: Can be used after modifying the m_pNurb data to update
  the analytic data.

RETURNS --- SM_ERR if the NURB no longer represents this Analytic surface
***********************************************************************/
SmStatus SmTorus::RebuildSTEPFromNURBParameters()
{
  // check m_pNurb - a degree 2 rational surface in at least 1 direction
  ULONG lDegU = GetDegree(SM_SP_U);
  ULONG lDegV = GetDegree(SM_SP_V);
  if (   lDegU != 2 
      || lDegV != 2 
      || !IsRational())
    { SER_MSG(SM_ERR, _T("Modified Nurb SmTorus: no longer has degree 2 rational basis functions")) ; }

  // method
  //  Check for iso circles

  //  Set m_pGenCurve to be SmCircle
  //  Set m_dMajorRadius, m_dMinorRadius
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
  SmPoint3d sCommonPointU, sCommonVecU ;
  SmPoint3d sCommonPointV, sCommonVecV ;

  // test sequence of V Direction (U Constant) isoCurves for circularity and parallelism
  SER(sm_TestForIsoCircles(this, SM_SP_U, dScaledZero, dAngTol,
                              bFoundCirclesU, bParallelPlanesU, bSamePlaneU,
                              bSameCenterU, bCommonAxisU, bPlanarGenCurveU,
                              dRadiusU, sAnglesU, sRefFrameU,
                              sCommonPointU, sCommonVecU));
  // quit when not a torus
  if (!bFoundCirclesU) return FALSE;

  // test sequence of U Direction (V Constant) isoCurves for circularity and parallelism
  SER(sm_TestForIsoCircles(this, SM_SP_V, dScaledZero, dAngTol,
                              bFoundCirclesV, bParallelPlanesV, bSamePlaneV,
                              bSameCenterV, bCommonAxisV, bPlanarGenCurveV,
                              dRadiusV, sAnglesV, sRefFrameV,
                              sCommonPointV, sCommonVecV));

  // see if we have a U or a V oriented torus
  SmBoolean bIsTorusWithSwapUV = (bParallelPlanesU && bCommonAxisV) ;
  SmBoolean bIsTorusNoSwapUV   = (bParallelPlanesV && bCommonAxisU) ;

  // quit when not a properly oriented torus
  if(!bIsTorusWithSwapUV && !bIsTorusNoSwapUV)
    { SER_MSG(SM_ERR, _T("Modified Nurb SmTorus: no longer has oriented circular IsoParamCurves ")) ; }

  // locals
  double      dMinRad ;
  SmExtent1d *pSweepIvl ;
  SmPoint2d   sMidUV    = sNurbUVDomain.Evaluate(0.5,0.5);
  SmPoint2d   sMinUV    = sNurbUVDomain.Evaluate(0.0,0.0);
  SmBoolean   bSwapUV ;
  SmAxis2Placement *pRefFrame, *pOtherFrame ;
  SmVector3d sCommonPoint, sCommonVec ;

  // Get torus position and parameters
  if(bIsTorusNoSwapUV)
    {
      bSwapUV      = FALSE ;
      pRefFrame    = &sRefFrameV ; // frame where all circles are parallel                    
      pOtherFrame  = &sRefFrameU ; // frame where all circle planes intersect on a common line
      dMinRad      = dRadiusU ;
      pSweepIvl    = &sAnglesV ;
      sCommonPoint = sCommonPointU ; // from frame where all circle planes intersect on a common line
      sCommonVec   = sCommonVecU ;   // from frame where all circle planes intersect on a common line
    }
  else
    {
      SM_ASSERT(bIsTorusWithSwapUV) ;
      bSwapUV      = TRUE ;
      pRefFrame    = &sRefFrameU ; // frame where all circles are parallel                    
      pOtherFrame  = &sRefFrameV ; // frame where all circle planes intersect on a common line
      dMinRad      = dRadiusV ;
      pSweepIvl    = &sAnglesU ;
      sCommonPoint = sCommonPointV ; // from frame where all circle planes intersect on a common line
      sCommonVec   = sCommonVecV ; // from frame where all circle planes intersect on a common line
    }

  // get torus origin = MinorCircleCenter projected onto common line
  //   a MinorCircleCenter is stored in pOtherFrame->GetOriginRef()
  SM_ASSERT(SM_IS_ZERO(sCommonVec.Length() - 1.0)) ;
  SmPoint3d sTorusOrigin =   sCommonPoint
                           + sCommonVec.Dot(pOtherFrame->GetOriginRef() - sCommonPoint)
                           * sCommonVec ;

  // Install the origin into pRefFrame.  // [B328]
  pRefFrame->SetCanonical( sTorusOrigin, pRefFrame->GetXAxisRef(), pRefFrame->GetYAxisRef() );

  // get torus orientation and size
  SmVector3d sMajorVec = pOtherFrame->GetOriginRef() - pRefFrame->GetOriginRef() ;
  double     dMajRad   = sMajorVec.Length() ;

  // Check GenCurve minor circle rotation direction, its y axis is either +/- sweepCircle z
  // assumes: OtherFrame is valid for 1st Nurb isoparameter GenCurve
  SmVector3d sTorusZAxis       = pRefFrame->GetZAxis() ;
  SmVector3d sMinorOrientation = pOtherFrame->GetZAxis() * sMajorVec ;

  SM_ASSERT(sMinorOrientation.IsParallelTo(sTorusZAxis, 2.0)) ;
  SM_ASSERT(sTorusZAxis.IsParallelTo(sCommonVec, SM_EFF_ZERO * 100.0 * 360.0)) ;

  SmBoolean bInsideOut = sTorusZAxis.Dot(sMinorOrientation) < 0.0
                         ? TRUE
                         : FALSE ;

  // Get GenCurve angles from each genCurves center point coordinate system
  // project Nurb Min/Max Point to Torus XY plane to get vec to Center of each GenCurve
  SmVector3d sMinPlanePoint, sMaxPlanePoint ;
  smgu_PointProjectToPlane(sNurbMinPoint, sTorusOrigin, sTorusZAxis, sMinPlanePoint) ;
  smgu_PointProjectToPlane(sNurbMaxPoint, sTorusOrigin, sTorusZAxis, sMaxPlanePoint) ;
  SmVector3d sStartForMinAngle = sMinPlanePoint - sTorusOrigin ; 
  SmVector3d sStartForMaxAngle = sMaxPlanePoint - sTorusOrigin ;
  sStartForMinAngle.Unitize() ; 
  sStartForMaxAngle.Unitize() ;
  SmVector3d sMinOrigin = sTorusOrigin + dMajRad * sStartForMinAngle ; 
  SmVector3d sMaxOrigin = sTorusOrigin + dMajRad * sStartForMaxAngle ; 

  // Get Vec from Nurb Min/Max points to each genCurve origin
  SmVector3d sVecMin = sNurbMinPoint - sMinOrigin ;
  SmVector3d sVecMax = sNurbMaxPoint - sMaxOrigin ; 

  // get GenCurve min and max angles measured from each GenCurve's origin and start vector
  double dGenAngleMin, dGenAngleMax;
  SE(sStartForMinAngle.AngleBetween(sVecMin,dGenAngleMin));
  SE(sStartForMaxAngle.AngleBetween(sVecMax,dGenAngleMax));
  dGenAngleMin = 180.0 * dGenAngleMin / SM_PI;
  dGenAngleMax = 180.0 * dGenAngleMax / SM_PI;

  // snap to ends for small tolerances
  if(   smos_Fabs(dGenAngleMin) < SM_EFF_ZERO_DEG)       { dGenAngleMin  =   0.0; }
  if(   smos_Fabs(dGenAngleMax) < SM_EFF_ZERO_DEG
     || smos_Fabs(dGenAngleMax-360.0) < SM_EFF_ZERO_DEG) { dGenAngleMax  = 360.0; }
  if ( dGenAngleMax < dGenAngleMin+SM_EFF_ZERO_DEG )     { dGenAngleMax += 360.0; }

  // define Torus domain
  SmExtent2d sAnalUVDomain(pSweepIvl->GetMin(), dGenAngleMin, pSweepIvl->GetMax(), dGenAngleMax);

  // Get GenCurve Isoparameter Curve
  SmBSplineCurve *pGenCurveIsoCrv = NULL ;
  SER(SmBSplineSurface::CreateIsoParametricCurve(*cpContext, 
                                                 bSwapUV ? SM_SP_V  : SM_SP_U, 
                                                 bSwapUV ? sMinUV.y : sMinUV.x, 
                                                 0.0, 
                                                 pGenCurveIsoCrv));
  SmObjDelete sCleanObj(pGenCurveIsoCrv) ;
  SmCircle *pCircle = new (*cpContext) SmCircle(sMinOrigin,
                                               sStartForMinAngle,
                                               sTorusZAxis,
                                               sAnalUVDomain.GetVInterval(),
                                               dMinRad, 3, cpContext,
                                               pGenCurveIsoCrv,
                                               bInsideOut) ;
  SmObjDelete sCleanGen(pCircle) ;

  // no work - GenCurve is degenerate
  if (pCircle->IsDegenerate(dScaledZero)) 
    { SER_MSG(SM_ERR, _T("Modified Nurb SmTorus: now has degenerate GenCurve")) ; }

  // set m_pGenCurve
  if (m_pGenCurve != NULL) { delete m_pGenCurve; m_pGenCurve = NULL; }
  m_pGenCurve = pCircle ;
  m_pGenCurve->SetOwner(this) ;
  sCleanGen.Clear() ;

  // set Analytic parameters
  m_dMajorRadius     = dMajRad ;
  m_dMinorRadius     = dMinRad ;
  m_bInsideOut       = bInsideOut ; 
  m_bPlanarGenerator = TRUE ;
  m_vPosition        = *pRefFrame ;
  m_bSwapUV          = bSwapUV ;
  m_vAnalUVDomain    = sAnalUVDomain ;

  // Set PolarConverter from Nurb midPoint rotation isoParamCurve
  // pRotationIsoCrv will be an SmCircle because isoParam is coming from SmTorus
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

} // end SmTorus::RebuildSTEPFromNURBParameters

/*******************************************************************//**
PURPOSE: Rotate surface about axis-of-revolution.  

NOTES: 
***********************************************************************/
SmStatus SmTorus::RotationAboutAxisZ
  (const SmContext & crContext,
   double dAngleDeg,
   SmTorus *& rpNewTorus)
{
    SmBSplineCurve * pNewGenCurve = NULL;
    SER(CreateGeneratorFromAngle(crContext,0.0,pNewGenCurve));
    double dAngleRad = SM_DEG2RAD(dAngleDeg);
    SmAxis2Placement sRF;
    sRF.RotateAboutAxisAtPoint(dAngleRad, 
                               m_vPosition.GetOriginRef(), 
                               m_vPosition.GetZAxis());
    SmAxis2Placement sTmpA2P;
    m_vPosition.TransformAxis2Placement(sRF,sTmpA2P);

    SER(pNewGenCurve->Transform(sTmpA2P));

    SER(SmTorus::CreateCanonical(crContext,sTmpA2P,m_dMajorRadius,m_dMinorRadius,rpNewTorus));

    SER(rpNewTorus->AdjustSTEPUVDomain(GetSTEPUVDomain()));

    return SM_SUCCESS;

} // end SmTorus::RotationAboutAxisZ

/*******************************************************************//**
PURPOSE: Scale and transform an SmTorus surface.

NOTES: Scaling is not allowed on analytical surfaces.
***********************************************************************/
SmStatus SmTorus::Transform
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

  // update cone size
  m_dMajorRadius *= dScale;
  m_dMinorRadius *= dScale;

  // finish the call in the parent function
  SmSurfOfRevolution::Transform(crRotateNMove, cpOptScale) ;

  // all done
  return SM_SUCCESS;

} // end SmTorus::Transform

/*******************************************************************//**
PURPOSE: Compute the total size of the memory used by the SmTorus.

NOTES: Does not add in attribute memory
***********************************************************************/
ULONG SmTorus::GetMemoryUsed      // rtn: smaller size of actually used memory in bytes
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

} // end SmTorus::GetMemoryUsed

/*******************************************************************//**
PURPOSE: AssertValid() Test reporting static labels
***********************************************************************/
SmAssertReportLabel sAssertTorus_list[] =
{
  /*  0 */ {SM_AT_RADIUS,           _T("Major Radius"),          _T("m_dMajorRadius is greater than zero") },
  /*  1 */ {SM_AT_RADIUS,           _T("Minor Radius"),          _T("m_dMinorRadius is greater than zero") },
  /*  2 */ {SM_AT_PARAMETERIZATION, _T("Same Parameterization"), _T("Curve and Surface have same paramterization") },
  /*  3 */ {SM_AT_GEOMETRIC,        _T("Overlapping Domain"),    _T("Analytic U Direction Domain too large: Tori can't overlap themselves") },
  /*  4 */ {SM_AT_GEOMETRIC,        _T("Overlapping Domain"),    _T("Analytic V Direction Domain too large: Tori can't overlap themselves") }
} ;

/*******************************************************************//**
PURPOSE: Make sure GenCurve and Surface parameterizations are
            compatible.

RETURNS ---  TRUE  = OK
             FALSE = Problem
***********************************************************************/
SmBoolean SmTorus::AssertValid
 (SmAssertArray    * pAList,              // i/o: Accumulating list of failed Asserts, NULL to ignore, default:[NULL] 
  SmAssertTestLevel  eTestLevel,          // in : SM_LEVEL_0=fewest tests, SM_LEVEL_1, SM_LEVEL_2=all tests, 
                                          //      SM_LEVEL_GIVEN = run tests in order requested in pTestRequests 
                                          //      default:[SM_LEVEL_0] 
  SmAssertWalking    eWalkTreeeWalkTree,  // NotUsed: in : SM_WALK = run AssertValid on any topology graph descendants, SM_NO_WALK=don't, default:[SM_WALK] 
  SmTArray<ULONG>  * pTestRequests)       // in : when eTestLevel == SM_LEVEL_GIVEN, run these tests in this order, default:[NULL]
 const
{
 SM_REF1(eWalkTreeeWalkTree) ;
  // run parent AssertValid
  SmBoolean bRtn = TRUE ;
  
  // call the base class AssertValid
  bRtn &= (  (eTestLevel != SM_LEVEL_GIVEN)
           ? SmSurfOfRevolution::AssertValid(pAList, eTestLevel, SM_NO_WALK, pTestRequests) 
           : TRUE ) ; 

  // must have postivie radii
  bRtn &= SM_ASSERT_VALUE_REPORT(0, SM_LEVEL_0, (m_dMajorRadius > 0.0), 0.0, m_dMajorRadius, _T("")) ;
  bRtn &= SM_ASSERT_VALUE_REPORT(1, SM_LEVEL_0, (m_dMinorRadius > 0.0), 0.0, m_dMinorRadius, _T("")) ;

  // check for self intersections
  if(m_dMajorRadius < m_dMinorRadius)
    {
      // surface will self intersect unless saved by trimming
      double dAngleLimitDeg =  90.0 
                             + smos_ArcCosine(m_dMajorRadius / m_dMinorRadius) 
                             * 180.0 / SM_PI ;
      SmExtent1d sSelfIntersectIvl(dAngleLimitDeg, 360.0 - dAngleLimitDeg) ;
      if(!sSelfIntersectIvl.AreDisjointPeriodic(m_vAnalUVDomain.GetUInterval(), 360.0))
        {
          bRtn &= FALSE ;
        }
    } // end majorRadius < minorRadius check

  // locals
  //const SmPoint3d &rTorusOrig  = m_vPosition.GetOriginRef() ;
  SmVector3d       sTorusZAxis = m_vPosition.GetZAxis() ;

  // check temporary genCurve against parameterization of NurbSurface
  const SmContext &crContext = *GetContext() ;
  SmBSplineCurve  *pGenCurve = NULL ;
  CreateGeneratorFromAngle(crContext, 0.0, pGenCurve) ;
  SmObjDelete sClean(pGenCurve) ;

  // GenCurve must have same parameterization as Surface in GenCurve direction
  // GenCurve AnalInterval   must equal Surface->AnalUVDomain->GetVInterval,
  // GenCurve NurbInterval   must equal m_bSwapUV ? Surface->NURBUInterval : Surface->NURBVInterval
  // GenCurve NURBKnotVector must equal m_bSwapUV ? Surface->NURBKnotU : Surface->NURBKnotV
  SmBoolean bHasSameParam = HasSameParameterization(SM_SP_V,        // in : Analytic Domain Direction
                                                      m_bSwapUV     // in : Nurb     Domain Direction
                                                    ? SM_SP_U 
                                                    : SM_SP_V,
                                                    pGenCurve) ;    // in : The GenCurve

  bRtn &= SM_ASSERT_VALUE_REPORT(2, SM_LEVEL_0, (bHasSameParam), SM_EFF_ZERO, SM_UNDEF_DOUBLE, _T("") ) ;

  // Analytic U Direction Domain too large: tori can't overlap themselves
  SmExtent1d sIvlU = m_vAnalUVDomain.GetUInterval() ;
  bRtn &= SM_ASSERT_VALUE_REPORT(2, SM_LEVEL_0,
                                 (sIvlU.GetLength() <= 360.0 + SM_EFF_ZERO_DEG),
                                 SM_EFF_ZERO_DEG, sIvlU.GetLength() - 360.0, _T("") ) ;   

  // Analytic V Direction Domain too large: tori can't overlap themselves
  SmExtent1d sIvlV = m_vAnalUVDomain.GetUInterval() ;
  bRtn &= SM_ASSERT_VALUE_REPORT(3, SM_LEVEL_0,
                                 (sIvlV.GetLength() <= 360.0 + SM_EFF_ZERO_DEG),
                                 SM_EFF_ZERO_DEG, sIvlV.GetLength() - 360.0, _T("")) ;   


#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
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

} // end SmTorus::AssertValid
                                                 
// obsolete
// /*******************************************************************//**
// PURPOSE: Fix Assert Report failures reported by an AssertValid call
// 
// RETURNS ---  TRUE  = OK
//              FALSE = Problem
// ***********************************************************************/
// SmBoolean SmTorus::AssertHeal
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
//                rAReport.m_pHealMessage = _T("SmTorus::AssertHeal fix not yet supported") ;  
//                 
//     }
// 
//   // all done
//   return(bRtn) ;
// 
// } // end SmTorus::AssertHeal
// end obsolete

/*******************************************************************//**
PURPOSE: Write SmTorus to given output stream.

NOTES: 
***********************************************************************/
SmStatus SmTorus::WriteToDB
 (SmDatabaseIO & rDB,                // in : target output stream
  ULONG          lDBVersionNumber)   // in : database version to get proper sequence of writes                                                                       
 const
{
  // file type, ASCII or BINARY
  SmFileType      eType    =  rDB.GetFileType();
  std::ostream  & rFileOut = *rDB.GetOutStreamPtr();
  
  if (eType == SM_ASCII) 
    {
      rFileOut << m_dMajorRadius << " SmTorus Torus Major Radius \n" ;
      rFileOut << m_dMinorRadius << " SmTorus Torus Minor Radius \n" ;
      rFileOut << m_bInsideOut   << " SmTorus Torus InsideOut: TRUE=outward pointing normal, FALSE=inverted \n" ;
    }
  else 
    {
      SER(rDB.WriteDouble( m_dMajorRadius)) ;
      SER(rDB.WriteDouble( m_dMinorRadius)) ;
      SER(rDB.WriteBoolean( m_bInsideOut)) ;  
    }

  // output the parent
  SER(SmSurfOfRevolution::WriteToDB(rDB, lDBVersionNumber)) ; 

  // all done
  return SM_SUCCESS;

} // end SmTorus::WriteToDB

/*******************************************************************//**
PURPOSE: static method to Read a SmTorus from a given stream  

NOTES: 
***********************************************************************/
SmStatus SmTorus::ReadFromDB
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
         || rpNewSurface->IsKindOf(SmTorus_TYPE)) ? SM_SUCCESS : SM_ERR) ;

  // init output object
  SmTorus *pTorus =   (rpNewSurface == NULL)
                    ? new (crContext) SmTorus()
                    : (SmTorus *)rpNewSurface ;

  // file type
  SmFileType     eType   =  rDB.GetFileType();
  std::istream & rFileIn = *rDB.GetInStreamPtr();
      
  if (eType == SM_ASCII) 
    {
      rFileIn >> pTorus->m_dMajorRadius ; rDB.GoToNextLine() ;
      rFileIn >> pTorus->m_dMinorRadius ; rDB.GoToNextLine() ;
      rFileIn >> pTorus->m_bInsideOut ;   rDB.GoToNextLine() ;
    }
  else 
    {
      SER(rDB.ReadDouble (pTorus->m_dMajorRadius)) ;  
      SER(rDB.ReadDouble (pTorus->m_dMinorRadius)) ;  
      SER(rDB.ReadBoolean(pTorus->m_bInsideOut)) ;   
    }

  // read the parent object
  SmSurface *pSurface = pTorus ; 
  SER(SmSurfOfRevolution::ReadFromDB(SmSurfOfRevolution_TYPE, rDB, crContext, pSurface, lDBVersionNumber)) ;

  // all done
  rpNewSurface = pTorus ; 
  return SM_SUCCESS;

} // end SmTorus::ReadFromDB

/*******************************************************************//**
PURPOSE:
NOTES:
***********************************************************************/
SmBoolean SmTorus::IsKindOf( SM_TYPE t ) const
{
  return ((SmTorus_TYPE == t) ? TRUE : SmSurfOfRevolution::IsKindOf( (t) ));
}

/*******************************************************************//**
PURPOSE: Dump Torus surface data out for debugging.

NOTES: 
***********************************************************************/
void SmTorus::Dump
  (void) 
 const
{
  TCHAR sBuff[SM_TBLOCK_SIZE], sBuffForFile[SM_TBLOCK_SIZE];
  smos_WriteBuffer(_T("\nBegin SmTorus::Dump()")) ;

  smos_sprintf(sBuff,       _T("\nSmTorus = 0x%p"), this) ;
  smos_sprintf(sBuffForFile,_T("\nSmTorus = %s"), _T("notNULL")) ;
  smos_WriteBuffer(sBuff, sBuffForFile);

  smos_WriteBuffer(_T("\n  AnalUVDomain  = ")); m_vAnalUVDomain.Dump();
  smos_WriteBuffer(_T("  NurbDomain    = "));   GetNaturalUVDomain().Dump() ;

  smos_sprintf(sBuff, _T("  Major Radius = %16.16lf, Minor Radius = %16.16lf, bSwap_UV = %s"),
                        m_dMajorRadius,
                        m_dMinorRadius,
                        m_bSwapUV ? _T("TRUE") : _T("FALSE"));
  smos_WriteBuffer(sBuff);

  SmSurfOfRevolution::Dump();
  smos_WriteBuffer(_T(" End SmTorus::Dump()\n")) ;

} // end SmTorus::Dump
