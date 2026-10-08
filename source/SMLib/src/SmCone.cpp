// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*******************************************************************//**
* FILE NAME --- SmCone.cpp
* PURPOSE: Implementation of SmCone methods.
* NOTE: There is a known parameterization problem because SmSurfOfRevolution
*   assumes that the analytical parameter comes from the curve but that is
*   not always the case where it is syncronized with the analytical parameter
*   of a cone.  The cone has its own analytical parameter.  An example is
*   an offset cone parameterized from 0 to 1 in length.  
**********************************************************************/

#include "StdAfx.h"
#include <nurbs.h>
#include <SmGeomUtility.h>
#include <SmNurbsSrf.h>
#include <SmPointSet.h>


/*******************************************************************//**
File Local Function Declarations
***********************************************************************/
static void sm_GetEllipseTrimIntervals
  (const SmContext     &crContext,            // in : for temp object construction
   const SmPoint3d     &rEllipseOrigin,       // in : origin            of ellipse
   const SmVector3d    &rEllipseXAxis,        // in : xAxis             of ellipse
   const SmVector3d    &rEllipseYAxis,        // in : yAxis             of ellipse
   double               dEllipseXRadius,      // in : x radius          of ellipse
   double               dEllipseYRadius,      // in : y radius          of ellipse
   double               dEllipseStartAngDeg,  // in : start angle (deg) of ellipse
   double               dEllipseEndAngDeg,    // in : end angle (deg)   of ellipse
   const SmCone        &crCone,               // in : Cone with trim boundaries
   const SmExtent2d    &crUVDomain,           // in : nurb trim boundaries for cone
   double               dTol,                 // in : approximation tolerance
   SmTArray<SmExtent1d> &rIvls) ;             // out: array of ellipse intervals inside trimmed cone

static void sm_AddEllipseIntervalsTrimmedToTwoCones
  (const SmContext            & crContext,              // in : for temp object construction
   const SmPoint3d            & rEllipseOrigin,         // in : origin            of ellipse
   const SmVector3d           & rEllipseXAxis,          // in : xAxis             of ellipse
   const SmVector3d           & rEllipseYAxis,          // in : yAxis             of ellipse
   double                       dEllipseXRadius,        // in : x radius          of ellipse
   double                       dEllipseYRadius,        // in : y radius          of ellipse
   double                       dEllipseStartAngDeg,    // in : start angle (deg) of ellipse
   double                       dEllipseEndAngDeg,      // in : end angle (deg)   of ellipse
   const SmCone               & crCone1,                // in : Cone1 with trim boundaries
   const SmExtent2d           & crUVDomain1,            // in : nurb trim boundaries for cone1
   const SmCone               & crCone2,                // in : Cone2 with trim boundaries
   const SmExtent2d           & crUVDomain2,            // in : nurb trim boundaries for cone2
   double                       dTol,                   // in : approximation tolerance
   SmTArray<SmCurve*>         * pOpt3DCurves,           // out: array of Trimmed Intersection 3DCurves 
   SmTArray<SmBSplineCurve *> & rDegenerateCurves) ;    // out: accumulation of unique Degnerate intersections (they may coincide with pOpt3DCurve)

/*******************************************************************//**
PURPOSE: Create an unbounded Cone running from an apex to positive infinity 
NOTES: 
  creates infinite cone parameterized from [0 360]        for sweep domain
     and [-SM_INFINITE_PARAMETER, SM_INFINITE_PARAMETER]  for cylindrical linear domains 
     and [ApexParam, SM_INFINITE_PARAMETER]               for conical     linear domains
     when bSwapUV==FALSE and bInsideOUt==FALSE
        Cone SrfNormal points from SrfPt away from cone axis
***********************************************************************/
SmCone::SmCone
  (const SmPoint3d  & crOrigin,       // in : STEP BotCircle Center Point
   const SmVector3d & crXAxis,        // in : STEP Surface  0 degree rotation
   const SmVector3d & crYAxis,        // in : STEP Surface 90 degree rotation
   double             dBotRadius,     // in : radius of cone at crOrigin
   double             dSemiAngleDeg,  // in : Angle from cone ZAxis to Cone Surface (degrees [0,90])
   SmBoolean          bSwapUV,        // in : TRUE = underlying NURB surface v dir maps to rotation direction
   SmBoolean          bInsideOut)     // in : TRUE = underlying NURB linear dir runs from TopCircle to BotCircle
                                      //      FALSE= underlying NURB linear dir runs from BotCircle to TopCircle
 : SmSurfOfRevolution(NULL,crOrigin,crXAxis,crYAxis,SmExtent2d(),bSwapUV),
   m_dBaseRadius(dBotRadius),
   m_dSemiAngleDeg(dSemiAngleDeg),
   m_bInsideOut(bInsideOut)
{
  // check input
  SM_ASSERT(dSemiAngleDeg >= 0.0 && dSemiAngleDeg < 90.0);
  SM_ASSERT(dBotRadius >= 0.0) ;

  // Create generator curve (SmLine)
  // Derive line direction vec by rotating Z vector about Y-axis
  // for m_dSemiAngleDeg and scale it by 1.0/smos_Cosine(m_dSemiAngleDeg)
  SmPoint3d  sLinePnt  = crOrigin + m_dBaseRadius*crXAxis;
  SmVector3d sZAxis    = crXAxis * crYAxis;
  SmVector3d sLineVec  = sZAxis;
  double     dAngleRad = SM_DEG2RAD(m_dSemiAngleDeg); 
  double     dScale    = 1.0/smos_Cosine(dAngleRad);
  if (dAngleRad > SM_EFF_ZERO) 
    {
      SmPoint3d sPnt   = crOrigin + sZAxis*dScale;
      SmAxis2Placement sRF; // m_vPosition leads to bug
      sRF.RotateAboutAxisAtPoint(dAngleRad,crOrigin,crYAxis); 
      sRF.TransformPoint(sPnt,sPnt);
      sLineVec = sPnt - crOrigin;
    }
  // unitize LineVec
  sLineVec.Unitize() ;

  // init domain to +/- infinity for cylinders - cones will be adjusted
  m_vAnalUVDomain.SetMinMax(0.0,   -SM_INFINITE_PARAMETER,
                            360.0,  SM_INFINITE_PARAMETER);

  // build generator curve
  const SmContext * cpContext = GetContext();
  SmLine          * pNewLine  = new (*cpContext) SmLine
                                   (sLinePnt,                         // in : LinePoint
                                    sLineVec,                         // in : UnitVec
                                    m_vAnalUVDomain.GetVInterval(),   // in : analytic domain
                                    dScale,                           // in : scale
                                    3, cpContext, NULL,               // in : dim, context, optNurb
                                    bInsideOut) ;                     // in : InsideOut
  m_pGenCurve = pNewLine;
  m_pGenCurve->SetOwner(this) ;

  // The parameterization of m_pGenCurve(SmLine) is now equivalent to
  // the STEP-definition which is by 'height' from the origin.
  SmBoolean bCylinder ;
  SmPoint3d sApexPoint ;
  double    dApexParam ;
  GetApex(bCylinder, sApexPoint, dApexParam) ;

  // infinite cylinders run between +/- infinity
  // infinite cones     run between apex and +infinity
  if ( !bCylinder )
    {
      m_vAnalUVDomain.SetMinMax(  0.0,   dApexParam,
                                360.0, SM_INFINITE_PARAMETER);
      // adjust generator curve domain
      pNewLine->AdjustSTEPInterval( SmExtent1d( dApexParam, SM_INFINITE_PARAMETER ));
    }

} // end SmCone::SmCone unbounded cone constructor

/*******************************************************************//**
PURPOSE: Create a Bounded Cone.

NOTES: 
  - Cone bottom is at crOrigin.
  - when bSwapUV==FALSE and bInsideOUt==FALSE
    - u-parameterization (rotational): [ dStartRotAngle, dEndRogAngle ]
    - v-parameterization (along axis): [ 0, dHeight ]
    - Cone SrfNormal points from SrfPt away from cone axis

  functions that call this private function are expected to set 
    m_pNurb with a call to MakeNurb() or by setting m_pNurb directly.
***********************************************************************/
SmCone::SmCone
  (const SmPoint3d  & crOrigin,     // in : BotCircle Center Point (v=0.0) upon creation.
                                    //      although that may be subsequently modified.
   const SmVector3d & crXAxis,      // in : cone Surface  0 degree rotation
   const SmVector3d & crYAxis,      // in : cone Surface 90 degree rotation
   double             dBotRadius,   // in : radius of bottom circular boundary
   double             dTopRadius,   // in : radius of top    circular boundary
   double             dStartAngle,  // in : start angle boundary from XAxis (degrees [-360, 360])
   double             dEndAngle,    // in : end   angle boundary from xAxis (degress [-360, 360]) maxRange=360
   double             dHeight,      // in : distance from bottom to top circular boundaries
   SmBoolean          bSwapUV,      // in : TRUE = underlying NURB surface v dir maps to rotation direction
   SmBoolean          bInsideOut,   // in : TRUE = underlying NURB linear dir runs from TopCircle to BotCircle
                                    //      FALSE= underlying NURB linear dir runs from BotCircle to TopCircle
   SmBoolean    bMakeNurbGenCurve)  // in : TRUE = GenCurve is type SmBSplineCurve
                                    //      FALSE= GenCurve is type SmLine
 : SmSurfOfRevolution(NULL,crOrigin,crXAxis,crYAxis,SmExtent2d(dStartAngle,0.0,dEndAngle,dHeight),bSwapUV),
   m_dBaseRadius(dBotRadius),
   m_dSemiAngleDeg(0.0),
   m_bInsideOut(bInsideOut)
{
  SM_ASSERT(dBotRadius >= 0.0) ;
  SM_ASSERT(dTopRadius >= 0.0) ;
  SM_ASSERT(dHeight    >= 0.0) ;

  // Specify GenCurve type
  m_bMakeNurbGenCurve = bMakeNurbGenCurve;

  // modify m_dSemiAngleDeg for nonCylinder shapes
  double dRadiusDiff = dTopRadius - dBotRadius;
  if (smos_Fabs(dRadiusDiff) > SM_EFF_ZERO) 
    {
      // The trick here is to generate a line with parameterization 
      // which is equivalent to the height of the cone/cylinder.
      // The V parameter which corresponds to the STEP definition is the
      // 'height' from the cone's origin. Therefore, a scaled direction
      // vector of line(generator) will allow syncronization between height
      // and V parameter(STEP-based) of SmSurfOfRevolution (which is 
      // the generator's parameter.)
      // The scale of direction vector is: 1.0/cos(m_dSemiAngleDeg).
      // First, calculate m_dSemiAngleDeg
      m_dSemiAngleDeg = smos_ArcTangent2(dRadiusDiff,dHeight) * 180.0 / SM_PI;
    }

  // The parameterization of m_pGenCurve(SmLine) is now equivalent to
  // the STEP-definition which is by 'height' from the origin.
  const SmContext * cpContext = GetContext();
  SmVector3d        sZAxis    = crXAxis * crYAxis ; sZAxis.Unitize() ;
  SmVector3d        sLineVec  = dHeight * sZAxis + (dTopRadius - dBotRadius) * crXAxis ;
  double            dLength   = sLineVec.Length() ;
  SmLine          * pLine     = new (*cpContext) SmLine
                                   (crOrigin + dBotRadius * crXAxis,  // in : LinePoint
                                    sLineVec/dLength,                 // in : UnitVec
                                    SmExtent1d(0.0, dHeight),         // in : analytic domain
                                    dLength/dHeight,                  // in : scale
                                    3, cpContext, NULL,               // in : dim, context, optNurb
                                    bInsideOut) ;                     // in : InsideOut
  m_pGenCurve = pLine;
  m_pGenCurve->SetOwner(this) ;

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  if(bDebugMe)
    {
      Dump() ;
      m_pGenCurve->Dump() ;
      SM_DUMP_AND_ASSERT2_VALID(this) ;
    }
#endif // SM_DEBUG_CODE

} // end SmCone::SmCone bounded cone constructor

/*******************************************************************//**
PURPOSE: Copy constructor for SmCone

NOTES: 
***********************************************************************/
SmCone::SmCone
  (const SmCone & crSource)                 // in : target surface to copy
 : SmSurfOfRevolution(crSource),
   m_dBaseRadius(crSource.m_dBaseRadius),
   m_dSemiAngleDeg(crSource.m_dSemiAngleDeg),
   m_bInsideOut(crSource.m_bInsideOut)
{
} // end SmCone::SmCone copy constructor

/*******************************************************************//**
PURPOSE: Equality operator for SmCone

NOTES: Call base equivalence to check type and then check 
       members for equivalence
***********************************************************************/
SmBoolean SmCone::operator==
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
      SmCone &rOther = (SmCone &)crOther ;

      // check equivalence of these objects
      bRtn =  (   SM_IS_ZERO(m_dBaseRadius   - rOther.m_dBaseRadius)
               && SM_IS_ZERO(m_dSemiAngleDeg - rOther.m_dSemiAngleDeg)
               && m_bInsideOut    == rOther.m_bInsideOut) ;
    }

  // all done
  return bRtn ;

} // end SmCone::operator==

/*******************************************************************//**
PURPOSE: Virtual Copy Constructor

NOTES: 
***********************************************************************/
SmStatus SmCone::Copy
  (const SmContext & crContext,     // in : new object context
   SmSurface      *& rpNewSurface)  // out: new SmCone copy
  const
{
  SM_DUMP_AND_ASSERT2_VALID(this) ;
  SmCone *pCopy = new (crContext) SmCone(*this); NER(pCopy);
  rpNewSurface  = pCopy;
  SM_DUMP_AND_ASSERT2_VALID(rpNewSurface) ;

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  if(bDebugMe)
    {
      SM_ASSERT_VALID(this) ;
      SM_ASSERT_VALID(rpNewSurface) ;

      SmFace *pFace = (SmFace *)this->GetFace() ;
      SmBrep *pBrep = pFace ? pFace->GetBrep() : NULL ;

      smgfx_Erase() ;
      smgfx_SetLook( 1, 2, 0, 0, 1 ); if(pBrep) { pBrep->Draw( TRUE ); sm_GraphicsLoop(); }
      smgfx_SetLook(1,2, 0,1,0) ; this->DrawUV(8,8,FALSE,NULL,TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook( 2, 3, 0, 1, 1 ); if(rpNewSurface) { rpNewSurface->DrawUV( 8, 8, FALSE, NULL, TRUE ); sm_GraphicsLoop(); }
      smgfx_SetLook( 1, 2, 0, 0, 0 ); if(pFace) { pFace->Draw( SM_DM_CROSSHATCH ); sm_GraphicsLoop(); }
      sm_GraphicsLoop() ;
    }
#endif
  return SM_SUCCESS;

} // end SmCone::Copy

/*******************************************************************//**
PURPOSE: Method to create an infinitely long canonical Cone object.

NOTES:  Returns an infinitely long cylinder
***********************************************************************/
SmStatus SmCone::CreateCanonical
  (const SmContext        & crContext,     // in : new object context
   const SmAxis2Placement & crOrigin,      // in : m_vOrigin = STEP BotCircle Center Point    
                                           //      m_vXAxis  = STEP Surface  0 degree rotation
                                           //      m_vYAxis  = STEP Surface 90 degree rotation
   double                   dBotRadius,    // in : Cone BottomCircle radius at crOrigin.m_vOrigin
   double                   dSemiAngleDeg, // in : ang from Z axis to Cone Surface, range:[0 - 90]
   SmCone                *& rpNewCone,     // out: new object
   double                 * pOptHeight,    // in : Distance from Bottom to TopCircle along Z axis of bounded cone
                                           //      NULL = make infinite cylinder, default:[NULL]
   SmBoolean                bSwapUV,       // in : optional: FALSE= map:[NurbU<->AnalU], TRUE=map:[NurbU<->AnalV], 
                                           //                           [NurbV<->AnalV],          [NurbV<->AnalU],
                                           //      default:[FALSE], for experts - always use default value
   SmBoolean                bInsideOut)    // in : optional: FALSE= ZAxis from Bot to Top, TRUE=negate, 
                                           //      default:[FALSE], for experts - always use default value
{
  // check inputs
  SM_ASSERT(dSemiAngleDeg >= 0.0 && dSemiAngleDeg < 90.0);

  // construct parameterized cone
  if(pOptHeight == NULL)
    {
      rpNewCone = new (crContext) SmCone(crOrigin.GetOriginRef(),
                                         crOrigin.GetXAxisRef(), 
                                         crOrigin.GetYAxisRef(), 
                                         dBotRadius, 
                                         dSemiAngleDeg, 
                                         bSwapUV,       // bSwapUV   
                                         bInsideOut);   // bInsideOut
    }
  else
    {
      double dTopRadius = dBotRadius + smos_TanDeg( dSemiAngleDeg ) * ( *pOptHeight );
      rpNewCone = new (crContext) SmCone(crOrigin.GetOriginRef(),
                                         crOrigin.GetXAxisRef(), 
                                         crOrigin.GetYAxisRef(), 
                                         dBotRadius, dTopRadius,
                                         0.0, 360.0, *pOptHeight, 
                                         bSwapUV,       // bSwapUV   
                                         FALSE,         // bInsideOut
                                         bInsideOut) ;  // bMakeNurbGenCurve
    }
  NER(rpNewCone);

  // check associated genCurve
  SM_ASSERT(rpNewCone->m_pGenCurve != NULL) ;
  if (rpNewCone->m_pGenCurve == NULL) 
    { delete rpNewCone; rpNewCone = NULL; SER(SM_ERR); } // Failure in constructor
  
  // Build NURB; fail cleanly if degenerate (single zero-radius apex is valid).
  if (rpNewCone->MakeNurb() != SM_SUCCESS || rpNewCone->GetGwNurbPointer() == NULL)
    {
      delete rpNewCone; rpNewCone = NULL;
      SE_MSG(SM_ERR, _T("SmCone::CreateCanonical: failed to build NURB representation")) ;
      return SM_ERR;
    }

  SM_DUMP_AND_ASSERT2_VALID(rpNewCone) ;
  return SM_SUCCESS;

} // end SmCone::CreateCanonical

/*******************************************************************//**
PURPOSE: Method to create a canonical Cone object.

NOTES:  If dHeight is not specified ( <= 0 ), creates an infinitely long cylinder
***********************************************************************/
SmStatus SmCone::CreateCanonical
  (const SmContext        & crContext,  // in : new object context
   const SmAxis2Placement & crOrigin,   // in : m_vOrigin = STEP BotCircle Center Point    
                                        //      m_vXAxis  = STEP Surface  0 degree rotation
                                        //      m_vYAxis  = STEP Surface 90 degree rotation
   double                   dBotRadius, // in : Cone BottomCircle radius at crOrigin.m_vOrigin
   double                   dTopRadius, // in : Cone TopCircle radius
   double                   dHeight,    // in : Distance from Center to TopCircle along Z axis of bounded cone
                                        //      0 = make infinite cone
   SmCone                *& rpNewCone,  // out: new object
   SmBoolean                bSwapUV,    // in : optional: FALSE= map:[NurbU<->AnalU], TRUE=map:[NurbU<->AnalV], 
                                        //                           [NurbV<->AnalV],          [NurbV<->AnalU],
                                        //      default:[FALSE], for experts - always use default value
   SmBoolean                bInsideOut) // in : optional: FALSE= ZAxis from Bot to Top, TRUE=negate, 
                                        //      default:[FALSE], for experts - always use default value
{
  // construct parameterized cone
  if(dHeight <= 0)
    {
      rpNewCone = new (crContext) SmCone(crOrigin.GetOriginRef(), // in : STEP BotCircle Center Point
                                         crOrigin.GetXAxisRef(),  // in : STEP Surface  0 degree rotation
                                         crOrigin.GetYAxisRef(),  // in : STEP Surface 90 degree rotation
                                         dBotRadius,              // in : radius of cone at crOrigin
                                         dTopRadius,              // in : Angle from cone ZAxis to Cone Surface (degrees [0,90])
                                         bSwapUV,                 // in : TRUE = underlying NURB surface v dir maps to rotation direction
                                         bInsideOut) ;            // in : TRUE = underlying NURB linear dir runs from TopCircle to BotCircle
    }                                                             //      FALSE= underlying NURB linear dir runs from BotCircle to TopCircle
  else
    {
      rpNewCone = new (crContext) SmCone(crOrigin.GetOriginRef(), // in : BotCircle Center Point (v=0.0) upon creation.
                                                                  //      although that may be subsequently modified.
                                         crOrigin.GetXAxisRef(),  // in : cone Surface  0 degree rotation
                                         crOrigin.GetYAxisRef(),  // in : cone Surface 90 degree rotation
                                         dBotRadius,              // in : radius of bottom circular boundary
                                         dTopRadius,              // in : radius of top    circular boundary
                                         0.0,                     // in : start angle boundary from XAxis (degrees [-360, 360])
                                         360.0,                   // in : end   angle boundary from xAxis (degress [-360, 360]) maxRange=360
                                         dHeight,                 // in : distance from bottom to top circular boundaries
                                         bSwapUV,                 // in : TRUE = underlying NURB surface v dir maps to rotation direction
                                         bInsideOut,              // in : TRUE = underlying NURB linear dir runs from TopCircle to BotCircle
                                                                  //      FALSE= underlying NURB linear dir runs from BotCircle to TopCircle
                                         FALSE) ;                 // in : TRUE = GenCurve is type SmBSplineCurve
    }                                                             //      FALSE= GenCurve is type SmLine
  NER(rpNewCone);

  // check associated genCurve
  SM_ASSERT(rpNewCone->m_pGenCurve != NULL) ;
  if (rpNewCone->m_pGenCurve == NULL) 
    { delete rpNewCone; rpNewCone = NULL; SER(SM_ERR); } // Failure in constructor

  // Build NURB; fail cleanly if degenerate (single zero-radius apex is valid).
  if (rpNewCone->MakeNurb() != SM_SUCCESS || rpNewCone->GetGwNurbPointer() == NULL)
    {
      delete rpNewCone; rpNewCone = NULL;
      SE_MSG(SM_ERR, _T("SmCone::CreateCanonical: failed to build NURB representation")) ;
      return SM_ERR;
    }

  SM_DUMP_AND_ASSERT2_VALID(rpNewCone) ;
  return SM_SUCCESS;

} // end SmCone::CreateCanonical

/*******************************************************************//**
PURPOSE: Adjust the surface by the given STEP-domain.

NOTES:
***********************************************************************/
SmStatus SmCone::AdjustSTEPUVDomain
  (const SmExtent2d & crNewSTEPUVDomain)  // in : new desired domain in degrees
                                          //      u range: [0_to_360]
                                          //      v range: 3-d distance along z-axis,
                                          //               0 is at our origin
{
  // Method: just call the parent's version,
  // unless the new domain would result in going past the apex.
  // Check the radius at the new ends.
  SmExtent1d sVDomain = crNewSTEPUVDomain.GetVInterval();
  if ( GetRadius( sVDomain.GetMin() ) < -SM_EFF_ZERO )
    { SER( SM_ERR_INVALID_INPUT ); }
  if ( GetRadius( sVDomain.GetMax() ) < -SM_EFF_ZERO )
    { SER( SM_ERR_INVALID_INPUT ); }
  
  return SmSurfOfRevolution::AdjustSTEPUVDomain( crNewSTEPUVDomain );

} // end SmCone::AdjustSTEPUVDomain

/*******************************************************************//**
PURPOSE: Method to get canonical data from Cone object.

NOTES: 
***********************************************************************/
SmStatus SmCone::GetCanonical
  (SmAxis2Placement & rOrigin,         // out: m_vOrigin = STEP BotCircle Center Point
                                       //      m_vXAxis  = STEP Surface  0 degree rotation
                                       //      m_vYAxis  = STEP Surface 90 degree rotation
   double           & rdRadius,        // out: BotCircle Radius at rOrigin.m_vOrigin
   double           & rdSemiAngleDeg,  // out: angle between Surface and cone axis
                                       //      0.0      = Cylinder (BotCircleRadius == TopCircleRadius)
                                       //      Positive = BotCircleRadius < TopCircleRadius
                                       //      Negative = BotCircleRadius > TopCircleRadius
   SmBoolean        * pOptSwapUV,      // out: TRUE = Nurb and Analytic U and V directions are swapped
                                       //      NULL to ignore, default:[NULL]
   SmBoolean        * pOptInsideOut)   // out: TRUE = Nurb and Analytic genCurve directions are swapped
                                       //      NULL to ignore, default:[NULL]
  const
{
  rOrigin        = m_vPosition;
  rdRadius       = m_dBaseRadius;
  rdSemiAngleDeg = m_dSemiAngleDeg;
  if(pOptSwapUV)     { *pOptSwapUV    = m_bSwapUV ; }
  if(pOptInsideOut)  { *pOptInsideOut = m_bInsideOut ; }

  return SM_SUCCESS;

} // end SmCone::GetCanonical

/*******************************************************************//**
PURPOSE: Find and return cone apex

NOTES: When cone is secretly a cylinder it has
  no apex and the cone origin is returned.  Check the
  bCylinder flag before using.
***********************************************************************/
SmStatus SmCone::GetApex
  (SmBoolean &bIsCylinder,     // out: FALSE = shape is a cone and has an apex point
                               //      TRUE  = shape is a cylinder without an apex and the cone center is returned
   SmPoint3d &rApexPoint,      // out: Cone apex point when bIsCylinder==FALSE, else Cylinder origin point
   double    &dApexParam)      // out: AnalV Param of rApexPoint
 const                         
{
  bIsCylinder = SM_IS_ZERO( m_dSemiAngleDeg ) ? TRUE : FALSE;
  if ( bIsCylinder )
  {
      rApexPoint = m_vPosition.GetOriginRef();
      dApexParam = 0.0;
      return(SM_ERR);
  }
  else
  {
      double dHeight = - m_dBaseRadius / smos_Tangent( m_dSemiAngleDeg*SM_PI/180.0 );
      rApexPoint     =   m_vPosition.GetOriginRef() + dHeight * m_vPosition.GetZAxis();
      SER( m_pGenCurve->STEPInversion( rApexPoint, dApexParam ));
  }

  return(SM_SUCCESS);

} // end SmCone::GetApex

/*******************************************************************//**
PURPOSE: Create an offset surface(s) for the cone.

NOTES: The offset Nurb and Step domains map from old to new
     so that   pOffSet->EvaluateSTEPPoint(StepUV) = pInput->EvaluateSTEPPoint() + dOff * dNormal(StepUV)
    and        pOffSet->EvaluatePoint(NurbUV)     = pInput->EvaluatePoint()     + dOff * dNormal(NurbUV)

    Out Surf->Domain(s) may be trimmed but not scaled.
***********************************************************************/
SmStatus SmCone::CreateOffsetSurface
(
  const SmContext      & crContext,             // in : context for new obj construction
  double                 dSignedOffsetDistance, // in : offset dist, (neg val = Offset dir opposite surface normal)
  SmApproxTol3d          dThisApproxTol3d,      // in : Max Dist between ApproxOffsetSurface and ideal offset shape
  SmSurface* &           rOffsetSurface         // out: Offset Surf Approx, may have self-intersections
) const
{
  SM_DUMP_AND_ASSERT2_VALID(this) ;

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  // draw
  if(bDebugMe)
    {
      SM_DUMP_AND_ASSERT_VALID(this) ;
      SmFace *pFace = (SmFace *)GetFace() ;
      SmBrep *pBrep = pFace ? pFace->GetBrep() : NULL ;

      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,1) ; DrawUV(7,7) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,0) ; DrawSTEP() ; sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

  // When UVs are swapped - reverse orientation of offset
  // When InsideOut       - reverse orientation of offset
  double dOffsetSign =   1.0
                       * (m_bSwapUV    ? -1.0 : 1.0)
                       * (m_bInsideOut ? -1.0 : 1.0) ;

  // Cone locals
  const SmVector3d & crOrigin      = m_vPosition.GetOriginRef() ;
  const SmVector3d & crXAxis       = m_vPosition.GetXAxisRef() ;
  const SmVector3d & crYAxis       = m_vPosition.GetYAxisRef() ;
  SmVector3d         sZAxis        = m_vPosition.GetZAxis() ;
  double             dHeight       = GetHeight() ;
  SmExtent2d         sAnalDomain   = GetSTEPUVDomain() ; 
  SmExtent2d         sNurbDomain   = GetNaturalUVDomain() ;
  SmExtent1d         sGenAnalIvl   = GetGenCurve()->GetSTEPInterval() ;
  SmExtent1d         sGenNurbIvl   = GetGenCurve()->GetNaturalInterval() ;

  // offset vector and displacements
  double      dSin        = smos_Sine  (m_dSemiAngleDeg * SM_PI / 180.0) ;
  double      dCos        = smos_Cosine(m_dSemiAngleDeg * SM_PI / 180.0) ;
  SmVector3d  dNorm       = dCos * crXAxis - dSin * sZAxis ;
              dNorm.Unitize();
  SmVector3d  sOffVec     =  dOffsetSign * dSignedOffsetDistance * dNorm ;
  double      dOffZDist   = -dSin * dOffsetSign * dSignedOffsetDistance ;
  double      dOffRadDist =  dCos * dOffsetSign * dSignedOffsetDistance ;

  // get new Bot and Top radii from Old and dOffRadDist
  double dOldBotRadius = GetBotRadius();
  double dOldTopRadius = GetTopRadius();
  double dNewBotRadius = dOldBotRadius + dOffRadDist;
  double dNewTopRadius = dOldTopRadius + dOffRadDist;

  // In the case of negative or interior offset cones
  // Old behavior: Don't create result AND return success
  // New behavior: Create new SmCone AND return success
  // If both top and bottom are negative then don't create result like before
  if(   dNewBotRadius < dThisApproxTol3d 
     && dNewTopRadius < dThisApproxTol3d) 
    { return SM_SUCCESS ; }

  // only the BotRadius or TopRadius has gone negative
  if(   dNewBotRadius < dThisApproxTol3d 
     || dNewTopRadius < dThisApproxTol3d) 
    {
      double     dTanAng, dNewHeight ;
      SmVector3d sBotOrigin ; 

      // compute BotOrigin, Height, Bot and Top radii for new truncated cone
      if(dNewBotRadius < dThisApproxTol3d) { dTanAng       = dHeight / dOldTopRadius ;
                                             dNewHeight    = dNewTopRadius * dTanAng ;
                                             sBotOrigin    = crOrigin + (dHeight + dOffZDist - dNewHeight) * sZAxis ;
                                             dNewBotRadius = 0.0 ;
                                           }
      else                                 { dTanAng       = dHeight / dOldBotRadius ;
                                             dNewHeight    = dNewBotRadius * dTanAng ;
                                             sBotOrigin    = crOrigin + dOffZDist * sZAxis ;
                                             dNewTopRadius = 0.0 ;
                                           }
      // New Cone Position - truncated at new apex position
      SmAxis2Placement newPosition;
      newPosition.SetCanonical(sBotOrigin, crXAxis, crYAxis);

      // Make the new cone - truncated at new apex - preserving original cone bSwapUV and bInsideOut values
      SmCone* pOffCone = NULL;
      SmCone::CreateCanonical(crContext,      // in : new object context
                              newPosition,    // in : m_vOrigin = STEP BotCircle Center Point    
                                              //      m_vXAxis  = STEP Surface  0 degree rotation
                                              //      m_vYAxis  = STEP Surface 90 degree rotation
                              dNewBotRadius,  // in : Cone BottomCircle radius at crOrigin.m_vOrigin
                              dNewTopRadius,  // in : Cone TopCircle radius
                              dNewHeight,     // in : Distance from Center to TopCircle along Z axis of bounded cone
                                              //      0 = make infinite cone
                              pOffCone,       // out: new object
                              m_bSwapUV,      // in : optional: FALSE= map:[NurbU<->AnalU], TRUE=map:[NurbU<->AnalV],
                                              //                           [NurbV<->AnalV],          [NurbV<->AnalU],
                                              //      default:[FALSE], for experts - always use default value
                              m_bInsideOut) ; // in : optional: FALSE= ZAxis from Bot to Top, TRUE=negate, 
                                              //      default:[FALSE], for experts - always use default value


      // set offset domains to be trimmed and not scaled subsets of the input domains
      SmExtent2d sNewAnalDomain = pOffCone->GetSTEPUVDomain() ; 
      SmExtent2d sNewNurbDomain = pOffCone->GetNaturalUVDomain() ;
      SmExtent1d sNewGenAnalIvl = pOffCone->GetGenCurve()->GetSTEPInterval() ;
      SmExtent1d sNewGenNurbIvl = pOffCone->GetGenCurve()->GetNaturalInterval() ;

      // what we want:
      // new offset Anal VMin = Input Anal VMin 
      // new offset Anal VMax = Input VMin + (NewHeight/OldHeight) * (Input VMax)
      // new offset Anal UIvl = Input Anal UIvl
      //
      // new offset Nurb UMin = m_bSwapUV ? Input Nurb UMin                                           : Input Nurb UMin ;
      // new offset Nurb UMax = m_bSwapUV ? Input Nurb UMin + (NewHeight/OldHeight) * Input Nurb UMax : Input Nurb UMax ;
      // new offset Nurb VMin = m_bSwapUV ? Input Nurb VMin : Input Nurb VMin ;
      // new offset Nurb VMax = m_bSwapUV ? Input Nurb VMax : Input Nurb VMin + (NewHeight/OldHeight) * Input Nurb VMax ;

      // Cone and GenCurve analytic domains should already be set
      SM_ASSERT_MSG(SM_ARE_SAME(sNewAnalDomain.GetVMin(), sAnalDomain.GetVMin()), _T("SmCone::CreateOffsetSurface Preserved domain problem - needs review")) ; 
      SM_ASSERT_MSG(SM_ARE_SAME(sNewAnalDomain.GetVMax(), sAnalDomain.GetVMin() + dNewHeight/dHeight * sAnalDomain.GetVMax()), _T("SmCone::CreateOffsetSurface Preserved domain problem - needs review")) ; 
      SM_ASSERT_MSG(sNewAnalDomain.GetUInterval() == sAnalDomain.GetUInterval(), _T("SmCone::CreateOffsetSurface Preserved domain problem - needs review")) ;
      SM_ASSERT_MSG(sNewAnalDomain.GetVInterval() == sNewGenAnalIvl, _T("SmCone::CreateOffsetSurface Preserved domain problem - needs review")) ;

      // compute the offsetSurf Nurb domain to be a properly trimmed subset of the input Nurb domain
      SmExtent2d sTgtNurbDomain ;
      if(m_bSwapUV) { sTgtNurbDomain.SetMinMax(sNurbDomain.GetUMin(),
                                               sNurbDomain.GetVMin(),
                                               sNurbDomain.GetUMin() + dNewHeight/dHeight * sNurbDomain.GetUMax(),
                                               sNurbDomain.GetVMax()) ;
                    }
      else          { sTgtNurbDomain.SetMinMax(sNurbDomain.GetUMin(),
                                               sNurbDomain.GetVMin(),
                                               sNurbDomain.GetUMax(),
                                               sNurbDomain.GetVMin() + dNewHeight/dHeight * sNurbDomain.GetVMax()) ;
                    }

      // update the pOffCone NurbDomain
      ((SmBSplineSurface *)pOffCone)->Reparameterize(sTgtNurbDomain) ;

      // adjust the GenCurve Nurb domain to be a properly trimmed subset of the input NurbDomain
      SmExtent1d sTgtGenNurbIvl = m_bSwapUV ? sTgtNurbDomain.GetUInterval() : sTgtNurbDomain.GetVInterval() ;
      ((SmBSplineCurve *)pOffCone->GetGenCurve())->EditParameterization(sTgtGenNurbIvl) ;

      // rebuild the PolarCurve to make sure all is coordinated
      pOffCone->RebuildPolarConverter() ;

#ifdef SM_DEBUG_CODE
      if(bDebugMe)
        {
          SM_DUMP_AND_ASSERT_VALID(this) ;
          SM_DUMP_AND_ASSERT_VALID(pOffCone) ;

          SmFace   * pFace          = (SmFace *)GetFace() ;
          SmBrep   * pBrep          = pFace ? pFace->GetBrep() : NULL ;

          // The offsetCone Height-domain has been truncated - don't bother checking for equal domains

          smgfx_Erase() ;
          smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
          smgfx_SetLook(1,2, 0,1,1) ; DrawUV(7,7) ; sm_GraphicsLoop() ;
          smgfx_SetLook(1,2, 0,1,1) ; DrawSTEP() ; sm_GraphicsLoop() ;
          smgfx_SetLook(3,4, 0,1,0) ; if(pOffCone) pOffCone->DrawUV() ; sm_GraphicsLoop() ;
          smgfx_SetLook(3,4, 0,1,0) ; if(pOffCone) pOffCone->DrawSTEP() ; sm_GraphicsLoop() ;
          smgfx_SetLook(4,5, 1,0,0) ; if(pOffCone && pOffCone->m_pGenCurve) pOffCone->m_pGenCurve->Draw() ; sm_GraphicsLoop() ;
          sm_GraphicsLoop() ;
        }
#endif // SM_DEBUG_CODE

      // set output
      rOffsetSurface = pOffCone;  
      return SM_SUCCESS;
    } // end cone being truncated due to negative Top or Bot radius check

  // copy the cone
  SmSurface *pCopy = NULL;
  Copy(crContext, pCopy);
  SmCone *pOffCone = (SmCone *)pCopy;

  // modify analytic parameters - new BaseRadius and new Cone Center
  pOffCone->m_dBaseRadius = m_dBaseRadius + dOffRadDist;
  pOffCone->m_vPosition.SetCanonical(crOrigin + dOffZDist * sZAxis, crXAxis, crYAxis) ;

  // modify every Cone Nurb ControlPoint position - preserves the input NURB domain
  if(pOffCone->GetGwNurbPointer())
    {
      // for every m_pNurb Control Point
      gw_CPOINT   sEuclid_0, sEuclid_j ;
      gw_CPOINT   *pCP_0, *pCP_j ;
      gw_SURFACE  *pOffNurb = pOffCone->GetGwNurbPointer() ;

      // For every m_pNurb ControlPoint - make inner iteration the GenCurve direction
      int ii, jj, iCnt, jCnt ;
      if(m_bSwapUV) { iCnt = pOffNurb->net->m ;
                      jCnt = pOffNurb->net->n ;
                    }
      else          { iCnt = pOffNurb->net->n ;
                      jCnt = pOffNurb->net->m ;
                    }
      // for every sweep direction control point
      for(ii=0;ii<=iCnt;ii++)
        {
          // get ControlPoint[i][0] Euclidean position
          pCP_0 = m_bSwapUV ? &(pOffNurb->net->Pw[0][ii])
                            : &(pOffNurb->net->Pw[ii][0]) ;

          TO_EUCLID(*pCP_0, sEuclid_0) ;
          SmVector3d sThisVec((sEuclid_0.x - crOrigin.x),
                              (sEuclid_0.y - crOrigin.y),
                              (sEuclid_0.z - crOrigin.z)) ;

          // get the Cone radius for ControlPoint[i][0] Radius
          double     dP0DotZ =   (sEuclid_0.x - crOrigin.x) * sZAxis.x
                               + (sEuclid_0.y - crOrigin.y) * sZAxis.y
                               + (sEuclid_0.z - crOrigin.z) * sZAxis.z ;
          SmPoint3d  rCenter = crOrigin + dP0DotZ * sZAxis ;
          SmVector3d sVec(sEuclid_0.x - rCenter.x,
                          sEuclid_0.y - rCenter.y,
                          sEuclid_0.z - rCenter.z) ;
          double     dThisRadius = GetRadius(dP0DotZ) ;
          SmVector3d sMove ;

          // when the radius is zero - need to compute move from other ControlPoint
          if(SM_IS_ZERO(dThisRadius))
            {
              // point is an apex - need to get rotation angle from next control point
              // get next controlPoint in genCurve direction
              gw_CPOINT  sRotationEuclid ;
              gw_CPOINT *pRotationCP = m_bSwapUV ? &(pOffNurb->net->Pw[1][ii])
                                                 : &(pOffNurb->net->Pw[ii][1]) ;
              TO_EUCLID(*pRotationCP, sRotationEuclid) ;
              SmVector3d sRotationVec((sRotationEuclid.x - crOrigin.x),
                                      (sRotationEuclid.y - crOrigin.y),
                                      (sRotationEuclid.z - crOrigin.z)) ;
              double     dThisHeight   = sRotationVec.Dot(sZAxis) ;
              SmVector3d sRadialVec    = sRotationVec - dThisHeight * sZAxis ;
              double     dRadius   = sRadialVec.Length() ;
              double     dHeightRadius = GetRadius(dThisHeight) ;
              sRadialVec = sRadialVec/dRadius ;
              dCos = sRadialVec.Dot(crXAxis) ;
              dSin = sRadialVec.Dot(crYAxis) ;

              sMove  = dRadius/dHeightRadius * dOffRadDist * (dCos * crXAxis + dSin * crYAxis) + dOffZDist * sZAxis ;
            }
          else
            {
              double dScale  = (dThisRadius + dOffRadDist) / dThisRadius ;
              sMove          =  sVec * (dScale-1.0) + dOffZDist * sZAxis ;
            }

          // for every genCurve directed ControlPoint
          for(jj=0;jj<=jCnt;jj++)
            {
              // apply the move to the controlPoint
              pCP_j = m_bSwapUV ? &(pOffNurb->net->Pw[jj][ii])
                                : &(pOffNurb->net->Pw[ii][jj]) ;
              TO_EUCLID(*pCP_j, sEuclid_j) ;

              pCP_j->x = (sEuclid_j.x + sMove.x) * pCP_j->w ;
              pCP_j->y = (sEuclid_j.y + sMove.y) * pCP_j->w ;
              pCP_j->z = (sEuclid_j.z + sMove.z) * pCP_j->w ;

            } // end iter every jjth ControlPoint
        } // end iter every iith ControlPoint

//            // for every m_pNurb Control Point
//            gw_CPOINT        sEuclid ;
//            gw_SURFACE      *pOffNurb = pOffCone->GetGwNurbPointer() ;
//            double           dOffZDist = (dNewRadius - dOldRadius) * smos_Tangent(m_dSemiAngleDeg * SM_PI / 180.0) ;
//            int ii, jj ;
//            for(ii=0;ii<=pOffNurb->net->n;ii++)
//              {
//                for(jj=0;jj<=pOffNurb->net->m;jj++)
//                  {
//                    // get Control Point Euclidean position
//                    gw_CPOINT *pCP = &(pOffNurb->net->Pw[ii][jj]) ;
//                    TO_EUCLID(*pCP, sEuclid) ;
//
//                    // move the ControlPoint towards/from center line
//                    double dPODotZ =   (sEuclid.x - rOrigin.x) * sZAxis.x
//                                     + (sEuclid.y - rOrigin.y) * sZAxis.y
//                                     + (sEuclid.z - rOrigin.z) * sZAxis.z ;
//                    SmPoint3d rCenter = rOrigin + dPODotZ * sZAxis ;
//                    SmVector3d sVec(sEuclid.x - rCenter.x,
//                                    sEuclid.y - rCenter.y,
//                                    sEuclid.z - rCenter.z) ;
//                    double     dScale  = dOldRadius/dNewRadius ;
//                    pCP->x = (rCenter.x + sVec.x/dScale + dOffZDist * sZAxis.x) * pCP->w ;
//                    pCP->y = (rCenter.y + sVec.y/dScale + dOffZDist * sZAxis.y) * pCP->w ;
//                    pCP->z = (rCenter.z + sVec.z/dScale + dOffZDist * sZAxis.z) * pCP->w ;
//

              //      // there are cheaper ways to compute the following - 
              //      //   but this is the best I can do for tolerances because all the tolerances
              //      //   are accumulated in the displacement rather than radius size.
              //      //   So tolerance is proportional to offset distance and not radius.
              //      double     dThisHeight   = sThisVec.Dot(sZAxis) ;
              //      SmVector3d sVecFromZ     = sThisVec - dThisHeight*sZAxis ;
              //      double     dThisRadius   = sVecFromZ.Length() ;
              //      double     dOrigRadius   = GetRadius(dThisHeight) ;
              //
              //      // get displacement for this vector based on rotation angle and radius
              //      // When radius is zero (at an apex) need to use another ControlPoint to compute rotation angle
              //      double dAngRad, dScale ;
              //      if(SM_IS_ZERO(dOrigRadius))
              //        {
              //          // get next controlPoint in genCurve direction
              //          gw_CPOINT  sRotationEuclid ;
              //          gw_CPOINT *pRotationCP = SM_IS_ZERO(dOrigRadius)
              //                                   ? (  m_bSwapUV
              //                                      ? &(pSurfNurb->net->Pw[ii==iCnt?0:ii+1][jj])
              //                                      : &(pSurfNurb->net->Pw[ii][jj==jCnt?0:jj+1]))
              //                                   : pCP ;
              //          TO_EUCLID(*pRotationCP, sRotationEuclid) ;
              //          SmVector3d sRotationVec((sRotationEuclid.x - crOrigin.x),
              //                                  (sRotationEuclid.y - crOrigin.y),
              //                                  (sRotationEuclid.z - crOrigin.z)) ;
              //          dAngRad = smos_ArcTangent2(sRotationVec.Dot(crYAxis),
              //                                     sRotationVec.Dot(crXAxis)) ;
              //          dScale  = dOffRadDist ;
              //        }
              //      else
              //        { dAngRad = smos_ArcTangent2(sThisVec.Dot(crYAxis),
              //                                     sThisVec.Dot(crXAxis)) ;
              //          dScale  = dOffRadDist * (dThisRadius/dOrigRadius) ;
              //        }
              //      
              //      // get radial displacement components for this (rotation angle and radius)
              //      double dSCos = dScale * smos_Cosine(dAngRad) ;
              //      double dSSin = dScale * smos_Sine  (dAngRad) ;
              //      
              //      // move the ControlPoint towards/from center line and along Z axis
              //      pCP->x = (sEuclid.x + dSCos * crXAxis.x + dSSin * crYAxis.x + dOffZDist * sZAxis.x) * pCP->w ;
              //      pCP->y = (sEuclid.y + dSCos * crXAxis.y + dSSin * crYAxis.y + dOffZDist * sZAxis.y) * pCP->w ;
              //      pCP->z = (sEuclid.z + dSCos * crXAxis.z + dSSin * crYAxis.z + dOffZDist * sZAxis.z) * pCP->w ;
              //      
              //      }  // end jj
              //  }  // end ii
    } // end m_pNurb check

#ifdef SM_DEBUG_CODE
  if(bDebugMe)
    {
      SM_DUMP_AND_ASSERT_VALID(this) ;
      SM_DUMP_AND_ASSERT_VALID(pOffCone) ;

      SmExtent2d sNewAnalDomain   = pOffCone->GetSTEPUVDomain() ; 
      SmExtent2d sNewNurbDomain   = pOffCone->GetNaturalUVDomain() ;
      SmExtent1d sNewGenAnalIvl   = pOffCone->GetGenCurve()->GetSTEPInterval() ;
      SmExtent1d sNewGenNurbIvl   = pOffCone->GetGenCurve()->GetNaturalInterval() ;

      // make sure all the OffsetCone domains are the same as in the inputCone domains
      SM_ASSERT_MSG(sAnalDomain == sNewAnalDomain,_T("SmCone::CreateOffsetSurface: Offset ConeDomain != Input ConeDomain - needs fixing.")) ;
      SM_ASSERT_MSG(sNurbDomain == sNewNurbDomain,_T("SmCone::CreateOffsetSurface: Offset ConeDomain != Input ConeDomain - needs fixing.")) ;
      SM_ASSERT_MSG(sGenAnalIvl == sNewGenAnalIvl,_T("SmCone::CreateOffsetSurface: Offset ConeDomain != Input ConeDomain - needs fixing.")) ;
      SM_ASSERT_MSG(sGenNurbIvl == sNewGenNurbIvl,_T("SmCone::CreateOffsetSurface: Offset ConeDomain != Input ConeDomain - needs fixing.")) ;

      SmPoint3d sInGenCurvePoint, sOffConePoint ;
      m_pGenCurve->EvaluateSTEPPoint(m_pGenCurve->GetSTEPInterval().GetMin(), sInGenCurvePoint) ;
      pOffCone->EvaluateSTEPPoint(SmPoint2d(0.0,0.0), sOffConePoint) ;
      SmVector3d sDisp = sOffConePoint - sInGenCurvePoint ;
      SM_ASSERT(SM_IS_ZERO(sDisp.DistanceBetween(sOffVec))) ;
    }
#endif // SM_DEBUG_CODE

  // translate the NewCone->GenCurve from current center line
  SmAxis2Placement sMove ;
  sMove.Translate(sOffVec) ;
  pOffCone->m_pGenCurve->Transform(sMove) ; 

#ifdef SM_DEBUG_CODE
  // draw 
  if(bDebugMe)
    {
      SM_DUMP_AND_ASSERT_VALID(this) ;
      SM_DUMP_AND_ASSERT_VALID(pOffCone) ;

      SmFace   * pFace          = (SmFace *)GetFace() ;
      SmBrep   * pBrep          = pFace ? pFace->GetBrep() : NULL ;
      SmExtent2d sNewAnalDomain = pOffCone->GetSTEPUVDomain() ; 
      SmExtent2d sNewNurbDomain = pOffCone->GetNaturalUVDomain() ;
      SmExtent1d sNewGenAnalIvl = pOffCone->GetGenCurve()->GetSTEPInterval() ;
      SmExtent1d sNewGenNurbIvl = pOffCone->GetGenCurve()->GetNaturalInterval() ;

      // make sure all the OffsetCone domains are the same as in the inputCone domains
      SM_ASSERT_MSG(sAnalDomain == sNewAnalDomain,_T("SmCone::CreateOffsetSurface: Offset ConeDomain != Input ConeDomain - needs fixing.")) ;
      SM_ASSERT_MSG(sNurbDomain == sNewNurbDomain,_T("SmCone::CreateOffsetSurface: Offset ConeDomain != Input ConeDomain - needs fixing.")) ;
      SM_ASSERT_MSG(sGenAnalIvl == sNewGenAnalIvl,_T("SmCone::CreateOffsetSurface: Offset ConeDomain != Input ConeDomain - needs fixing.")) ;
      SM_ASSERT_MSG(sGenNurbIvl == sNewGenNurbIvl,_T("SmCone::CreateOffsetSurface: Offset ConeDomain != Input ConeDomain - needs fixing.")) ;

      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,1) ; DrawUV(7,7) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,1) ; DrawSTEP() ; sm_GraphicsLoop() ;
      smgfx_SetLook(3,4, 0,1,0) ; if(pOffCone) pOffCone->DrawUV() ; sm_GraphicsLoop() ;
      smgfx_SetLook(3,4, 0,1,0) ; if(pOffCone) pOffCone->DrawSTEP() ; sm_GraphicsLoop() ;
      smgfx_SetLook(4,5, 1,0,0) ; if(pOffCone && pOffCone->m_pGenCurve) pOffCone->m_pGenCurve->Draw() ; sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

#ifdef SM_DEBUG_CODE
  SmBoolean bCheck=FALSE;
  if ( bCheck ) 
    {
      SmTArray< double > sU, sV;
      sU.Add( 0.1 ); sV.Add( 0.1 );
      sU.Add( 0.1 ); sV.Add( 0.9 );
      sU.Add( 0.9 ); sV.Add( 0.1 );
      sU.Add( 0.9 ); sV.Add( 0.9 );
      sU.Add( 0.4 ); sV.Add( 0.6 );
      SmPoint2d sUV, sStepUV;
      SmPoint3d sBasePt, sOffPt1, sOffPt2, sStepPt;
      SmVector3d sNorm, sDiff;
      SmExtent2d sNurbsDomain = GetNaturalUVDomain();
      static constexpr double dCheckLim = SM_EFF_ZERO * 100;
      ULONG ii;
      for ( ii = 0; ii < 5; ii++ ) 
        {
          sUV = sNurbsDomain.Evaluate( sU[ii], sV[ii] );
          EvaluatePoint ( sUV, sBasePt );
          EvaluateNormal( sUV, FALSE, FALSE, sNorm );
          sOffPt1 = sBasePt + sNorm * dOffsetSign * dSignedOffsetDistance;
          pOffCone->EvaluatePoint( sUV, sOffPt2 );
          sDiff = sOffPt2 - sOffPt1;
          double dErr = sDiff.Length();
          if ( dErr > dCheckLim )
            { SM_ASSERT_MSG( FALSE, _T("SmCone::CreateOffsetSurface(): geometric error") ); }

          this->ConvertUVFromNURBSToSTEP( sUV, sStepUV );
          EvaluateSTEPPoint( sStepUV, sStepPt );
          sDiff = sStepPt - sBasePt;
          dErr = sDiff.Length();
          if ( dErr > dCheckLim )
            { SM_ASSERT_MSG( FALSE, _T("SmCone::CreateOffsetSurface(): geometric error") ); }

          pOffCone->ConvertUVFromNURBSToSTEP( sUV, sStepUV );
          pOffCone->EvaluateSTEPPoint( sStepUV, sStepPt );
          sDiff = sStepPt - sOffPt2;
          dErr = sDiff.Length();
          if ( dErr > dCheckLim )
            { SM_ASSERT_MSG( FALSE, _T("SmCone::CreateOffsetSurface(): geometric error") ); }
        }
    } // end if bCheck == TRUE check
#endif // SM_DEBUG_CODE

  // set output
  rOffsetSurface = pOffCone;  
  
  // all done
  SM_DUMP_AND_ASSERT2_VALID(pOffCone) ;
  return SM_SUCCESS;

//        // init output container
//        SmCone *pOffCone = NULL ;
//        rOffsetSurfaces.ReSet();
//
//        // build infinite cones from STEP parameters only
//        if(!IsBounded())
//          {
//            // When UVs are swapped - reverse orientation of offset 
//            // When InsideOut       - reverse orientation of offset
//            double dOffsetSign =   1.0
//                                 * (m_bSwapUV    ? -1.0 : 1.0)
//                                 * (m_bInsideOut ? -1.0 : 1.0) ;
//            // get new radius
//            double dOldRadius = m_dBotRadius ;  //unused
//            double dNewRadius = m_dBotRadius + dOffsetSign * dSignedOffsetDistance;
//
//            // skip building degenerate or negative radius surfaces
//            if(dNewRadius < dThisApproxTol3d) 
//              { return SM_SUCCESS; }
//
//            // copy the current cone
//            pOffCone = new (crContext) SmCone(*this);
//
//            // change the radius
//            pOffCone->m_dBotRadius = dNewRadius ;
//
//            // rebuild the genCurve
//            double    dHeight    = pOffCone->m_vAnalUVDomain.GetMax().y ;
//            SmPoint3d sLinePoint =   pOffCone->m_vPosition.GetOriginRef()
//                                   + dNewRadius * pOffCone->m_vPosition.GetXAxis() ;
//            SmVector3d sLineVec  =   dHeight * pOffCone->m_vPosition.GetZAxis()
//                                   + (pOffCone->GetRadius(dHeight) - dNewRadius) * pOffCone->m_vPosition.GetXAxisRef() ;
//            double   dLength     = sLineVec.Length() ;
//            SmLine * pLine = new (crContext) SmLine
//                                   (sLinePoint,      
//                                    sLineVec/dLength,
//                                    pOffCone->m_vAnalUVDomain.GetVInterval(),
//                                    dLength/dHeight,
//                                    3, &crContext, NULL,
//                                    pOffCone->m_bInsideOut) ;
//            if(pOffCone->m_pGenCurve) { delete pOffCone->m_pGenCurve ; pOffCone->m_pGenCurve = NULL ; }
//            pOffCone->m_pGenCurve = pLine ;
//            pOffCone->m_pGenCurve->SetOwner(pOffCone) ;
//
//            // clear out existing m_pNurb
//            if(pOffCone->m_pNurb) 
//              { if(!pOffCone->m_bNurbIsBorrowed) { smos_Free(pOffCone->m_pNurb); }
//                pOffCone->m_pNurb = NULL ;
//              }
//      
//            // make associated surface m_pNurb - set up polar conversion
//            SER(pOffCone->MakeNurb());
//      
//            // restore the pOffCone nurb parameterization
//            if(m_pNurb)
//              {
//                SmExtent2d sNurbUVDomain = GetNaturalUVDomain() ;
//                pOffCone->SmBSplineSurface::Reparameterize(sNurbUVDomain, FALSE) ;
//              }
//      
//            // don't let infinite domains exceed the apex of cones (not cylinders)
//            if(   m_vAnalUVDomain.GetMin().y == -SM_INFINITE_PARAMETER
//               || m_vAnalUVDomain.GetMax().y ==  SM_INFINITE_PARAMETER)
//              {
//                SmBoolean bCylinder ;
//                SmPoint3d sApexPoint ;
//                double    dApexParam ;
//                GetApex(bCylinder, sApexPoint, dApexParam) ;
//      
//                // infinite cylinders run between +/- infinity
//                // infinite cones     run between apex and +infinity
//                if(!bCylinder) { pOffCone->m_vAnalUVDomain.SetMinMax(m_vAnalUVDomain.GetMin().x,    dApexParam,
//                                                                     m_vAnalUVDomain.GetMax().x,  SM_INFINITE_PARAMETER);
//                                 // adjust generator curve domain
//                                 pLine->AdjustSTEPInterval(SmExtent1d(dApexParam, SM_INFINITE_PARAMETER)) ;
//                               }
//              }
//      
//            // output the cone
//            rOffsetSurfaces.Add(pOffCone);
//          }
//        else // build truncated cones from Nurb surface
//          {
//            // When UVs are swapped - reverse orientation of offset 
//            // When InsideOut       - reverse orientation of offset
//            double dOffsetSign =   1.0
//                                 * (m_bSwapUV    ? -1.0 : 1.0)
//                                 * (m_bInsideOut ? -1.0 : 1.0) ;
//
//            // get new bot radius
//            double dOldRadius    = m_dBaseRadius; 
//            double dNewRadius    = m_dBaseRadius  + dOffsetSign * dSignedOffsetDistance;
//            double dNewTopRadius = GetTopRadius() + dOffsetSign * dSignedOffsetDistance;
//            // skip building degenerate or negative radius surfaces
//            if(   dNewRadius    < dThisApproxTol3d
//               || dNewTopRadius < dThisApproxTol3d) 
//              { return SM_SUCCESS; }
//      
//      #ifdef SM_DEBUG_CODE
//            // check that dSignedOffsetDistance is measured from BSpline Surface Normal Direction
//            // a point and normal
//            SmPoint3d sPnt, sNorm;
//            SmExtent2d sDomain = GetNaturalUVDomain() ;
//            SmPoint2d  sUV     = sDomain.Evaluate(0.0,0.0) ;
//            SER(EvaluatePoint(sUV,sPnt));
//            SER(EvaluateNormal(sUV,TRUE,TRUE,sNorm));
//      
//            // dDir is positive when Bspline surfNormal points away from origin
//            // dStepDir should be pos when radius increases and negative when radius decreases
//            double dDir     = sNorm.Dot(sPnt-m_vPosition.GetOriginRef()) ;
//            double dStepDir = dDir * dSignedOffsetDistance ;
//            // verify offset distance is with repsect to BSpline Surface NormalVector
//            SM_ASSERT(   dNewRadius == dOldRadius
//                      || dStepDir * (dNewRadius - dOldRadius) > 0.0) ; 
//      #endif // SM_DEBUG_CODE
//      
//            // skip building degenerate or negative radius surfaces
//            if(GetBotRadius() < dThisApproxTol3d) 
//              { return SM_SUCCESS; }
//      
//            // copy the current cone
//            pOffCone = new (crContext) SmCone(*this);
//            SM_ASSERT(pOffCone->m_pNurb     != NULL) ;
//            SM_ASSERT(pOffCone->m_pGenCurve != NULL) ; 
//            SmObjDelete sClean(pOffCone) ;
//      
//            // locals shared by input and offset surface
//            const SmPoint3d &rOrigin  = m_vPosition.GetOriginRef() ;
//            SmVector3d       sZAxis   = m_vPosition.GetZAxis() ;
//      
//            // change the bottom radius
//            pOffCone->m_dBotRadius = dNewRadius ;
//      
//            // translate the NewSphere->GenCurve from current center line
//            SmPoint3d        sPoint ;
//            m_pGenCurve->EvaluateSTEPPoint(0.0, sPoint) ;
//            double           dScaledZero = SM_EFF_ZERO * (1.0 + smos_Max(sPoint.GetMaxDimension(), m_dBotRadius)) ;
//            SM_ASSERT(SM_ARE_SAME_TO_TOL(m_dBotRadius, (sPoint-rOrigin).Length(), dScaledZero)) ;
//            SmVector3d       sMoveVec = (dNewRadius/dOldRadius - 1.0) * (sPoint-rOrigin) ;
//            SmAxis2Placement sMove ;
//      
//            // define and apply translation to GenCurve
//            sMove.Translate(sMoveVec) ;
//            pOffCone->m_pGenCurve->Transform(sMove) ; 
//      
//            // rebuild the underlying Nurb Surface
//            //   Just need to move the Nurb Points of the 
//            //   m_pNurb. GenCurve has already been done
//      
//            // for every m_pNurb Control Point
//            gw_CPOINT        sEuclid ;
//            gw_SURFACE      *pOffNurb = pOffCone->GetGwNurbPointer() ;
//            double           dOffZDist = (dNewRadius - dOldRadius) * smos_Tangent(m_dSemiAngleDeg * SM_PI / 180.0) ;
//            int ii, jj ;
//            for(ii=0;ii<=pOffNurb->net->n;ii++)
//              {
//                for(jj=0;jj<=pOffNurb->net->m;jj++)
//                  {
//                    // get Control Point Euclidean position
//                    gw_CPOINT *pCP = &(pOffNurb->net->Pw[ii][jj]) ;
//                    TO_EUCLID(*pCP, sEuclid) ;
//      
//                    // move the ControlPoint towards/from center line
//                    double dPODotZ =   (sEuclid.x - rOrigin.x) * sZAxis.x
//                                     + (sEuclid.y - rOrigin.y) * sZAxis.y
//                                     + (sEuclid.z - rOrigin.z) * sZAxis.z ;
//                    SmPoint3d rCenter = rOrigin + dPODotZ * sZAxis ;
//                    SmVector3d sVec(sEuclid.x - rCenter.x,
//                                    sEuclid.y - rCenter.y,
//                                    sEuclid.z - rCenter.z) ;
//                    double     dScale  = dOldRadius/dNewRadius ;
//                    pCP->x = (rCenter.x + sVec.x/dScale + dOffZDist * sZAxis.x) * pCP->w ;
//                    pCP->y = (rCenter.y + sVec.y/dScale + dOffZDist * sZAxis.y) * pCP->w ;
//                    pCP->z = (rCenter.z + sVec.z/dScale + dOffZDist * sZAxis.z) * pCP->w ;
//                
//                  } // end iter every jjth ControlPoint
//              } // end iter every iith ControlPoint
//      
//        // nothing to do for polar converter
//      //        pOffSphere->MakeNurb() ;
//      
//            // output the cone
//            sClean.Clear() ;
//            rOffsetSurfaces.Add(pOffCone);
//          } // end build offset for truncated cone branch
//      
//      #ifdef SM_DEBUG_CODE
//        // draw 
//        if(bDebugMe)
//          {
//            Dump() ;
//            pOffCone->Dump() ;
//            if(m_pNurb)
//              {
//                SmExtent2d sInputNurbDomain  = GetNaturalUVDomain() ;
//                SmExtent2d sInputStepDomain  = GetSTEPUVDomain() ;
//                SmExtent2d sOffsetNurbDomain = pOffCone->GetNaturalUVDomain() ;
//                SmExtent2d sOffsetStepDomain = pOffCone->GetSTEPUVDomain() ;
//                SM_ASSERT(   sInputNurbDomain.IsContainedBy(sOffsetNurbDomain)
//                          && sOffsetNurbDomain.IsContainedBy(sInputNurbDomain)) ;
//                SM_ASSERT(   sInputStepDomain.IsContainedBy(sOffsetStepDomain)
//                          && sOffsetStepDomain.IsContainedBy(sInputStepDomain)) ;
//              }
//                                     
//            SmFace *pFace = (SmFace *)GetFace() ;
//            SmBrep *pBrep = pFace ? pFace->GetBrep() : NULL ;
//      
//            smgfx_Erase() ;
//            smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
//            smgfx_SetLook(1,2, 0,1,1) ; DrawUV(7,7) ; sm_GraphicsLoop() ;
//            smgfx_SetLook(1,2, 0,0,0) ; if(pFace) pFace->Draw(SM_DM_CROSSHATCH) ; sm_GraphicsLoop() ;
//            smgfx_SetLook(3,4, 0,1,0) ; if(pOffCone) pOffCone->DrawUV() ; sm_GraphicsLoop() ;
//            smgfx_SetLook(4,5, 1,0,0) ; if(pOffCone && pOffCone->m_pGenCurve) pOffCone->m_pGenCurve->Draw() ; sm_GraphicsLoop() ;
//            sm_GraphicsLoop() ;
//          }
//      #endif // SM_DEBUG_CODE
//      
//        // all done
//        SM_DUMP_AND_ASSERT2_VALID(pOffCone) ;
//        return SM_SUCCESS;

} // end SmCone::CreateOffsetSurface

/*******************************************************************//**
PURPOSE: Create a 3D iso-parametric curve of a cone given the 
    Nurb parameter direction (U or V) to be kept constant 
    and the constant parameter in that direction.

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
SmStatus SmCone::CreateIsoParametricCurve
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
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  if(bDebugMe)                     // don't call this when a cone is being constructed    
    { Dump() ;                     // because this gets as part of the process to complete
      SM_ASSERT_VALID(this) ;      // the cone's definition.                              

      ULONG di, lSmpCnt=20 ;
      SmFace        * pFace     = (SmFace *)GetFace() ;
      SmBrep        * pBrep     = pFace ? pFace->GetBrep() : NULL ;
      // SmSurface     * pSurface  = pFace ? pFace->GetSurface() : NULL ; 
      SmExtent2d      sUVDomain = GetNaturalUVDomain() ;
      SmPoint3d       sPt ;
      SmPointSequence sSmpPnts ;

      // sample the isoParam curve 
      for(di=0;di<lSmpCnt;di++)
        {
          double    dSmpParam = (double)di/(double)lSmpCnt ; 
          SmPoint2d sUV       = sUVDomain.Evaluate(dSmpParam,dSmpParam) ; 
          if(eSurfParam == SM_SP_U) { sUV.x = dIsoParameter ; }
          else                      { sUV.y = dIsoParameter ; }

          EvaluatePoint(sUV, sPt) ; 
          sSmpPnts.Add(sPt) ;
        } // end iter every smp making a poor man's IsoParamCurve 

      smgfx_Erase() ;
      smgfx_SetLook( 1, 2, 0, 0, 1 ); if(pBrep) { pBrep->Draw( TRUE ); sm_GraphicsLoop(); }
      smgfx_SetLook(1,2,  0, 1, 1) ; DrawUV() ; sm_GraphicsLoop() ;
      smgfx_SetLook(2,4,  0, 1, 1) ; DrawParams() ; sm_GraphicsLoop() ;
      smgfx_SetLook(3,5, .2,.2,.2) ; DrawSeams() ; sm_GraphicsLoop() ;
      smgfx_SetLook(7,9,  1, 0, 0) ; sSmpPnts.Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook( 1, 2, 0, 0, 0 ); if(pFace) { pFace->Draw( SM_DM_CROSSHATCH ); sm_GraphicsLoop(); }
      sm_GraphicsLoop() ;
    }                           
#endif // SM_DEBUG_DCODE

  // init output
  rpNewIsoCurve = NULL ;
  if(pOptMaxGap3d) { *pOptMaxGap3d = 0.0 ; }

  // get the BSplineCurve
  SmBSplineCurve *pIsoCurve ;
  SER( SmBSplineSurface::CreateIsoParametricCurve
        (crContext,         // in : context for created objects 
         eSurfParam,        // in : Defines which Nurb parameter direction on surface to extract curve from
                            //      SM_SP_U = create constant u isoParameter curve
                            //      SM_SP_V = create constant v isoParameter curve
         dIsoParameter,     // in : Defines Nurb parametric value at which to extract the curve.                          
                            //      If eSurfParam==SM_SP_U this is a U parameter, if eSurfParam==SM_SP_V
                            //      then this is the V parameter
         0.0,               // in : dApproxTol3d = Not used in this method
         pIsoCurve,         // out: 3d IsoParameterCurve                      
         pOptDomain,        // in : optional trim bound for IsoParameterCurve
                            //      NULL=use GetNaturalUVDomain(), default:[NULL]
         pOptMaxGap3d,      // out: opt achieved max gap, NULL to ignore, default:[NULL]                 
         pOptUVIsoCurve)) ; // out: opt 2d UVTrimCurve Line (diff parameterization), NULL to ignore, default:[NULL]
                            
  // low work - when curve is degenerate don't bother making a circle or line
  if(pIsoCurve->IsDegenerate())
    { 
      rpNewIsoCurve = pIsoCurve ;
      return(SM_SUCCESS) ; 
    }

  // make pIsoCurve temporary
  SmObjDelete sClean(pIsoCurve) ;

  // Step Isoparameter direction
  SmSurfParamType eSTEPParam = m_bSwapUV 
                               ? ((eSurfParam == SM_SP_U) ? SM_SP_V : SM_SP_U)
                               : eSurfParam ;
  
  // Cone locals
  const SmPoint3d  &rConeOrigin = m_vPosition.GetOriginRef() ;
  const SmVector3d &rConeXAxis  = m_vPosition.GetXAxisRef() ;
  const SmVector3d &rConeYAxis  = m_vPosition.GetYAxisRef() ;
  SmVector3d        sConeZAxis  = m_vPosition.GetZAxis() ;
                               
  // Nurb Surface size - trimmed to optional domain
  SmPoint2d  sMinUV, sMaxUV ;
  SmExtent2d sNurbDomain  = GetNaturalUVDomain() ;
  double dNurbSpanSize ;
  if(eSurfParam == SM_SP_U) { sMinUV.Set(dIsoParameter, pOptDomain ? pOptDomain->GetMin().y : sNurbDomain.GetMin().y) ;
                              sMaxUV.Set(dIsoParameter, pOptDomain ? pOptDomain->GetMax().y : sNurbDomain.GetMax().y) ;
                              dNurbSpanSize = sMaxUV.y - sMinUV.y ; 
                            }
  else                      { sMinUV.Set(pOptDomain ? pOptDomain->GetMin().x : sNurbDomain.GetMin().x, dIsoParameter) ;
                              sMaxUV.Set(pOptDomain ? pOptDomain->GetMax().x : sNurbDomain.GetMax().x, dIsoParameter) ;
                              dNurbSpanSize = sMaxUV.x - sMinUV.x ; 
                            }

  // pIsoCurve Sample Points - watch for reversed linear isoCurves
  SmExtent1d sNurbIvl = pIsoCurve->GetNaturalInterval() ;
  SmPoint3d sMinPoint, sMaxPoint, sMidPoint ;
  if(   m_bInsideOut
     && eSTEPParam == SM_SP_U) { pIsoCurve->EvaluatePoint(sNurbIvl.GetMin(), sMaxPoint) ;                                
                                 pIsoCurve->EvaluatePoint(sNurbIvl.GetMax(), sMinPoint) ;
                               }
  else                         { pIsoCurve->EvaluatePoint(sNurbIvl.GetMin(), sMinPoint) ;                                
                                 pIsoCurve->EvaluatePoint(sNurbIvl.GetMax(), sMaxPoint) ;
                               }


  pIsoCurve->EvaluatePoint(sNurbIvl.GetMid(), sMidPoint) ;

  double dScaledZero = SM_EFF_ZERO * (1.0 + smos_3Max(sMinPoint.GetMaxDimension(), sMidPoint.GetMaxDimension(),
                                                      rConeOrigin.GetMaxDimension()));

#ifdef SM_DEBUG_CODE

  SmBoolean bDebugMe2 = FALSE;
// This debug should be safe even when cone is not complete.
  if ( bDebugMe2 ) 
    {
      SmFace        * pFace     = (SmFace *)GetFace() ;
      SmBrep        * pBrep     = pFace ? pFace->GetBrep() : NULL ;
      // SmSurface     * pSurface  = pFace ? pFace->GetSurface() : NULL ; 

      SM_DUMP_AND_ASSERT_VALID(pIsoCurve) ;

      smgfx_Erase() ;
      smgfx_SetLook( 1, 2, 0, 0, 1 ); if(pBrep) { pBrep->Draw( TRUE ); sm_GraphicsLoop(); }
      smgfx_SetLook(1,2,  0, 1, 1) ; DrawUV() ; sm_GraphicsLoop() ;
      smgfx_SetLook(2,4,  0, 1, 1) ; DrawParams() ; sm_GraphicsLoop() ;
      smgfx_SetLook(3,5, .2,.2,.2) ; DrawSeams() ; sm_GraphicsLoop() ;
      smgfx_SetLook(5,7,  1, 0, 0) ; pIsoCurve->DrawParams() ; sm_GraphicsLoop() ;
      smgfx_SetLook( 1, 2, 0, 0, 0 ); if(pFace) { pFace->Draw( SM_DM_CROSSHATCH ); sm_GraphicsLoop(); }
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

  // branch on direction to make and return a circle or a line
  if(eSTEPParam == SM_SP_V) // constant V, varying U is a circle
    { 
      // return a circle
      SM_ASSERT(   pIsoCurve->GetDegree() == 2
                && pIsoCurve->IsRational()) ;

      // Get new Circle STEP parameters

      // Offset vector = projection of MidPoint onto cone ZAxis
      //   Due to tolerances, average the projection distance.
      //   Example: If this cone was built with IsNurbSurfaceCone
      //     The coordinate system was set by sampling isoparameter curves
      //     and using the samples to build the coordinate system.
      //   I can't see how the tolerances propagate.  So, just
      //   use these samples to estimate the tolerance
      double dOffsetMid = (sMidPoint-rConeOrigin).Dot(sConeZAxis) ;
      double dOffsetMin = (sMinPoint-rConeOrigin).Dot(sConeZAxis) ;
      double dOffsetMax = (sMaxPoint-rConeOrigin).Dot(sConeZAxis) ;
      double dOffset    = (dOffsetMid + dOffsetMin + dOffsetMax) / 3.0 ;
#ifdef SM_DEBUG_CODE
      double dCenterTol =   smos_Max(dScaledZero,
                                       smos_3Max(dOffsetMid, dOffsetMin, dOffsetMax)
                                     - smos_3Min(dOffsetMid, dOffsetMin, dOffsetMax)) ;


      SM_ASSERT(SM_ARE_SAME_TO_TOL(dOffset, dOffsetMid, dCenterTol)) ;
      SM_ASSERT(SM_ARE_SAME_TO_TOL(dOffset, dOffsetMin, dCenterTol)) ;
      SM_ASSERT(SM_ARE_SAME_TO_TOL(dOffset, dOffsetMax, dCenterTol)) ;
#endif // SM_DEBUG_CODE

      // Center = O + offset * Z
      SmPoint3d  sCenter   = rConeOrigin + dOffset * sConeZAxis ;
      SmVector3d sStartVec = sMinPoint - sCenter ;
      SmVector3d sEndVec   = sMaxPoint - sCenter ;

      // Get isoCurve analytic domain
      SmExtent1d sAnalInterval ;
      if(pOptDomain == NULL) 
        { 
          // let anlytic interval = cone sweep interval
          sAnalInterval.SetMinMax(m_vAnalUVDomain.GetMin().x, 
                                  m_vAnalUVDomain.GetMax().x) ; 
        }
      else // get angles to given points
        {
          // let Ivl = angles (deg) to min/max points measured in circle coordinate system
          double dAngleMinRad, dAngleMaxRad ;
          SmVector3d sZAxis = sConeZAxis ;
          sZAxis.CCWAngleBetween(rConeXAxis, sStartVec, dAngleMinRad) ;
          sZAxis.CCWAngleBetween(rConeXAxis, sEndVec, dAngleMaxRad) ;
          double dAngleMinDeg = dAngleMinRad * 180.0 / SM_PI ;
          double dAngleMaxDeg = dAngleMaxRad * 180.0 / SM_PI ;
          if(   dAngleMaxDeg < dAngleMinDeg
             || (    SM_IS_ZERO(dAngleMaxDeg-dAngleMinDeg) 
                 && !SM_IS_ZERO(dNurbSpanSize))) 
            { dAngleMaxDeg += 360.0 ; }
          sAnalInterval.SetMinMax(dAngleMinDeg, dAngleMaxDeg) ;
        }
      
      // radius = dist(circlePoints to Cone axis)  (dScaledZero is too small)
      // radius = length of StartPoint to center vector.
      //   Due to tolerances, average the projection distance.
      //   Example: If this cone was built with IsNurbSurfaceCone
      //     The coordinate system was set by sampling isoparameter curves
      //     and using the samples to build the coordinate system.
      //   I can't see how the tolerances propagate.  So, just
      //   use these samples to estimate the tolerance
      double dRadiusMin = sStartVec.Length() ;
      double dRadiusMid = (sMidPoint-sCenter).Length() ;
      double dRadiusMax = sEndVec.Length() ;
      double dRadius    = (dRadiusMin + dRadiusMid + dRadiusMax) / 3.0 ;
#ifdef SM_DEBUG_CODE
      double dRadiusTol = smos_Max(dScaledZero,
                                       smos_3Max(dRadiusMid, dRadiusMin, dRadiusMax)
                                     - smos_3Min(dRadiusMid, dRadiusMin, dRadiusMax)) ;

      double dEndVecLength = sEndVec.Length() ;
      double dMidVecLength = (sMidPoint-sCenter).Length() ;
      SM_ASSERT(SM_ARE_SAME_TO_TOL(dRadius, dEndVecLength, dRadiusTol)) ;
      SM_ASSERT(SM_ARE_SAME_TO_TOL(dRadius, dMidVecLength, dRadiusTol)) ;
#endif // SM_DEBUG_CODE

      // degenerate cases should already have been detected
      SM_ASSERT(   !SM_IS_ZERO(dRadius)
                && !SM_IS_ZERO(sAnalInterval.GetLength())) ;

      // create the circle using the isoCurve parameterization
      SmCircle *pCircle = new (crContext) SmCircle(sCenter, 
                                                   rConeXAxis, 
                                                   rConeYAxis,
                                                   sAnalInterval, 
                                                   dRadius, 3,
                                                   &crContext, 
                                                   pIsoCurve ) ;
                                                 //  , m_bInsideOut) ;
      rpNewIsoCurve = pCircle;
    }
  else // contant U, varying V is a line
    {
      // return a line
      SM_ASSERT(   pIsoCurve->GetDegree() == 1
                && pIsoCurve->IsRational()) ;

      // direction and scale
      SmVector3d sLineVec = sMaxPoint - sMinPoint ;
      double     dLength  = sLineVec.Length() ;
      if ( dLength < dScaledZero )
        { return SM_ERR_INVALID_INPUT; }
      double     dScale   = dLength / sLineVec.Dot(sConeZAxis) ;
      sLineVec /= dLength ;

      // Get isoCurve analytic domain
      SmExtent1d sAnalInterval ;
      if(pOptDomain == NULL) 
        { 
          // let anlytic interval = cone linear interval
          sAnalInterval.SetMinMax(m_vAnalUVDomain.GetMin().y, 
                                  m_vAnalUVDomain.GetMax().y) ; 
        }
      else // get heights to given points
        {
          // let Ivl = heights to min/max points
          double dMinHeight = (sMinPoint - rConeOrigin).Dot(sConeZAxis) ;
          double dMaxHeight = (sMaxPoint - rConeOrigin).Dot(sConeZAxis) ;
          sAnalInterval.SetMinMax(dMinHeight, dMaxHeight) ; 
        }
      
      // pick StartPoint to set IsoCurveMinPoint = StartPoint + sAnalIvl.GetMin() * dScale * dVec 
      SmPoint3d sStartPoint = sMinPoint - dScale * sAnalInterval.GetMin() * sLineVec ;
      
      //      // get Line StartPoint on same plane as [ConeOrigin,ConeZAxis]
      //      // note: for lines AnalDomain = NaturalInterval = pIsoIvl
      //      SmVector3d sMinVec     =  sMinPoint - rConeOrigin ;
      //      double     dStartParam = -sMinVec.Dot(sConeZAxis)/sLineVec.Dot(sConeZAxis) ;
      //      SmPoint3d  sStartPoint = sMinPoint + dStartParam * sLineVec ;

      // allocate the line - with analytic domain
      SmLine *pLine = new (crContext) SmLine(sStartPoint, sLineVec, sAnalInterval,
                                             dScale, 3, &crContext, pIsoCurve, m_bInsideOut) ;
      
      // set pLine Nurb domain
      pLine->SmBSplineCurve::EditParameterization(sNurbIvl) ;

      // save it
      rpNewIsoCurve = pLine;

#ifdef SM_DEBUG_CODE
      dScaledZero = SM_EFF_ZERO * 10.0 * (1.0 + sMaxPoint.GetMaxDimension()) ;
      SmPoint3d sEndPoint   = sStartPoint + dScale * sAnalInterval.GetMax() * sLineVec ;
      if(!sEndPoint.CloserThan(dScaledZero,sMaxPoint))
        { SM_ASSERT(sEndPoint.CloserThan(dScaledZero,sMaxPoint)) ; }
      if(bDebugMe)
        {
          this->Dump() ;
          rpNewIsoCurve->Dump() ;
        }
#endif // SM_DEBUG_CODE
    
    } // end constant U, varying V isoCurve branch

#ifdef SM_DEBUG_CODE
  {
    // make sure BSpline pIsoCurve is same shape as circle or line rpNewIsoCurve
    SmPoint3d sIsoMinPoint, sIsoMaxPoint ;
    SmPoint3d sNewMinSTEPPoint, sNewMaxSTEPPoint ;
    SmPoint3d sNewMinNurbPoint, sNewMaxNurbPoint ;

    // get isoParam and Line/circle intervals
    sNurbIvl = pIsoCurve->GetNaturalInterval() ;
    SmExtent1d sNewSTEPIvl = rpNewIsoCurve->GetSTEPInterval() ;
    SmExtent1d sNewNurbIvl = rpNewIsoCurve->GetNaturalInterval() ;

    // evaluate isoParam and line/circle Nurb/Step endPoints 
    pIsoCurve->EvaluatePoint(sNurbIvl.GetMin(), sIsoMinPoint) ;
    pIsoCurve->EvaluatePoint(sNurbIvl.GetMax(), sIsoMaxPoint) ;
    rpNewIsoCurve->EvaluateSTEPPoint(sNewSTEPIvl.GetMin(), sNewMinSTEPPoint) ;
    rpNewIsoCurve->EvaluateSTEPPoint(sNewSTEPIvl.GetMax(), sNewMaxSTEPPoint) ;
    rpNewIsoCurve->EvaluatePoint(sNewNurbIvl.GetMin(), sNewMinNurbPoint) ;
    rpNewIsoCurve->EvaluatePoint(sNewNurbIvl.GetMax(), sNewMaxNurbPoint) ;
    
    // compare the 3 sets of endPoint distances
    double dMinSTEPDist = (m_bInsideOut && eSTEPParam == SM_SP_U) ? (sIsoMinPoint - sNewMaxSTEPPoint).Length() 
                                                                  : (sIsoMinPoint - sNewMinSTEPPoint).Length() ;
    double dMaxSTEPDist = (m_bInsideOut && eSTEPParam == SM_SP_U) ? (sIsoMaxPoint - sNewMinSTEPPoint).Length() 
                                                                  : (sIsoMaxPoint - sNewMaxSTEPPoint).Length() ;
    double dMinNurbDist = (sIsoMinPoint - sNewMinNurbPoint).Length() ;
    double dMaxNurbDist = (sIsoMaxPoint - sNewMaxNurbPoint).Length() ;
    double dMinSTEPNurbDist = (m_bInsideOut && eSTEPParam == SM_SP_U) ? (sNewMinSTEPPoint - sNewMaxNurbPoint).Length() 
                                                                      : (sNewMinSTEPPoint - sNewMinNurbPoint).Length() ;
    double dMaxSTEPNurbDist = (m_bInsideOut && eSTEPParam == SM_SP_U) ? (sNewMaxSTEPPoint - sNewMinNurbPoint).Length() 
                                                                      : (sNewMaxSTEPPoint - sNewMaxNurbPoint).Length() ;
    dScaledZero = SM_EFF_ZERO * 100.0 * (1.0 + smos_Max(sIsoMinPoint.GetMaxDimension(),
                                                        sIsoMaxPoint.GetMaxDimension())) ;

    SmBoolean bOK = (   dMinSTEPDist      < dScaledZero //(dScaledZero is too small)
                     && dMaxSTEPDist      < dScaledZero
                     && dMinNurbDist      < dScaledZero
                     && dMaxNurbDist      < dScaledZero
                     && dMinSTEPNurbDist  < dScaledZero
                     && dMaxSTEPNurbDist  < dScaledZero) ;
    if ( ! bOK )
      { SM_ASSERT(bOK) ; }
      
    if(bDebugMe)
      {
        SM_DUMP_AND_ASSERT_VALID(this) ;
        SM_DUMP_AND_ASSERT_VALID(pIsoCurve) ;
        SM_DUMP_AND_ASSERT_VALID(rpNewIsoCurve) ;
    
        SmFace *pFace = (SmFace *)GetFace() ;
        SmBrep *pBrep = pFace ? pFace->GetBrep() : NULL ;

        smgfx_Erase() ;
        smgfx_SetLook( 1, 2, 0, 0, 1 ); if(pBrep) { pBrep->Draw( TRUE ); sm_GraphicsLoop(); }
        smgfx_SetLook(1,2,  0, 1, 1) ; DrawUV() ; sm_GraphicsLoop() ;
        smgfx_SetLook(2,4,  0, 1, 1) ; DrawParams() ; sm_GraphicsLoop() ;
        smgfx_SetLook(3,5, .2,.2,.2) ; DrawSeams() ; sm_GraphicsLoop() ;
        smgfx_SetLook(5,7,  1, 0, 0) ; pIsoCurve->DrawParams() ; sm_GraphicsLoop() ;
        smgfx_SetLook(9,10, 1, 0, 1) ; rpNewIsoCurve->DrawParams() ; sm_GraphicsLoop() ;
        smgfx_SetLook( 1, 2, 0, 0, 0 ); if(pFace) { pFace->Draw( SM_DM_CROSSHATCH ); sm_GraphicsLoop(); }
        sm_GraphicsLoop() ;
      }
  }
#endif // SM_DEBUG_CODE

  // all done
  return SM_SUCCESS;

} // end SmCone::CreateIsoParametricCurve

/*******************************************************************//**
PURPOSE: Create an iso parametric circle given a reference point 
    that defines the plane of the circle.  If the reference point does not
    project normally to the cylinder AXIS then rpCircle will be 
    returned as NULL.

NOTES: If there is a pole and the point lies on the plane of 
    the pole (zero radius point of cone) then a degenerate curve will 
    be returned.
***********************************************************************/
SmStatus SmCone::CreateIsoCircleFromPoint
  (const SmContext & crContext,          // in : context for new object construction
   const SmPoint3d & crReferencePoint,   // in : Point on plane defining cone/plane circle
   double            d3DTolerance,       // in : min distance between unique 3d Points
   SmBSplineCurve *& rpCircle)           // out: Return Curve either
                                         //      NULL, degenerate Curve, 
  const                                  //      or SmCircle 
{
  SM_DUMP_AND_ASSERT2_VALID(this) ;

  // init output
  rpCircle = NULL ;

  // project Point to RevolutionAxis
  const SmPoint3d &rOrigin        = m_vPosition.GetOriginRef() ;
  SmVector3d       sZAxis         = m_vPosition.GetZAxis() ;
  SmExtent1d       sHeightStepIvl = m_vAnalUVDomain.GetVInterval() ;
  double           dParam ;
  smgu_LineClosestPoint(rOrigin, sZAxis, crReferencePoint, dParam) ;

  // skip points not within the height bounds of the cone
  if(!sHeightStepIvl.ContainsValue(dParam, d3DTolerance))
    { return(SM_SUCCESS) ; }

  // build Step Point on Surface for Point
  double dRadius            = GetRadius(dParam) ;
  SmPoint3d  sCenter        = rOrigin + dParam * sZAxis ;
  SmVector3d sVec           = crReferencePoint - sCenter ;
  if ( sVec.LengthSquared() < 2 * SM_EFF_ZERO_SQ )
  {
      // Given point is on our axis.
      sVec = m_vPosition.GetXAxis();
  }
  sVec.Unitize();
  SmPoint3d sPointOnSurface = sCenter + dRadius * sVec ;

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
      smgfx_SetLook(1,2, 0,1,1) ; DrawSTEP() ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,0,0) ; if(pFace) pFace->Draw(SM_DM_CROSSHATCH) ; sm_GraphicsLoop() ;
      smgfx_SetLook(3,4, 1,1,0) ; m_pGenCurve->Draw() ; sm_GraphicsLoop() ; 
      smgfx_SetLook(5,6, 1,0,0) ; crReferencePoint.Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook(7,8, 0,1,0) ; sPointOnSurface.Draw() ; sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

  // map Point to StepUV
  SmLocationType eloc ;
  SmPoint2d sStepUV, sNurbUV ;
  STEPInversion(m_vAnalUVDomain, sPointOnSurface, d3DTolerance, sStepUV, eloc) ;
  
  // map StepUV to NurbUV
  ConvertUVFromSTEPToNURBS(sStepUV, sNurbUV) ; 

  // build and return the isoParametercurve                    
  SER(CreateIsoParametricCurve(crContext,                      
                               m_bSwapUV ? SM_SP_U   : SM_SP_V  ,
                               m_bSwapUV ? sNurbUV.x : sNurbUV.y,
                               d3DTolerance,               
                               rpCircle)) ;

  // all done
  return(SM_SUCCESS) ;

//        // init output
//        rpCircle = NULL;
//      
//        // Build cone linear 2d isoParameter line (either u=Domain.Min.x or v=Domain.Min.y)
//        SmSurfParamType eLinearParam;
//        SmExtent1d  sExt     = GetSTEPLinearParamExtent(&eLinearParam);
//        SmPoint2d   sUVStart = GetNaturalUVDomain().GetMin();
//        SmPoint2d   sUVEnd   = GetNaturalUVDomain().GetMax();
//        if (eLinearParam == SM_SP_U) 
//          {
//            // make Domain.Min constant v isoparameter line 
//            sUVEnd.y = sUVStart.y;
//          }
//        else 
//          {
//            // make Domain.Min constant u isoparameter line 
//            sUVEnd.x = sUVStart.x;
//          }
//      
//        // get 3d isoparameter line start and vec
//        SmPoint3d sStartPoint, sEndPoint;
//        SER(EvaluatePoint(sUVStart,sStartPoint));
//        SER(EvaluatePoint(sUVEnd,sEndPoint));
//        SmVector3d sVec = sEndPoint - sStartPoint;    
//        
//        // intersect 3d line with plane[ReferencePoint,ConeZAxis]
//        SmVector3d sPlaneNorm = m_vPosition.GetZAxis();
//        double dParam;
//        SER(smgu_LinePlaneIntersect(sStartPoint,sVec,crReferencePoint,sPlaneNorm,dParam));
//        double dDist = sEndPoint.DistanceBetween(sStartPoint);
//      
//        // no work - no intersection
//        if (   dParam*dDist < -d3DTolerance
//            || dParam*dDist > dDist+d3DTolerance) 
//          {
//            return SM_SUCCESS; // No intersections
//          }
//      
//        // get intersection's isoCurve Parameter
//        if (dParam < 0.0) dParam = 0.0;
//        if (dParam > 1.0) dParam = 1.0;
//        double dIsoParam =   (dParam*dDist < d3DTolerance)       ? sExt.GetMin()
//                           : (dParam*dDist > dDist-d3DTolerance) ? sExt.GetMax()
//                           : sExt.Evaluate(dParam);
//      
//        // check for degenerate curve at cone bottom 
//        SmPoint3d sDegenerateCurvePoint;
//        SmBoolean bCreateDegenerateCurve = FALSE;
//        if (   dIsoParam == sExt.GetMin() 
//            && m_dBotRadius < SM_EFF_ZERO) 
//          {
//            bCreateDegenerateCurve = TRUE;
//            sDegenerateCurvePoint  = m_vPosition.GetOriginRef();
//          }
//      
//        // check for degenerate curve at cone top
//        else if(   dIsoParam == sExt.GetMax()
//                && !IsCylinder() 
//                && GetTopRadius() < SM_EFF_ZERO) 
//          {
//            bCreateDegenerateCurve = TRUE;
//            sDegenerateCurvePoint =   m_vPosition.GetOriginRef()
//                                    + GetHeight() * m_vPosition.GetZAxis();
//          }
//      
//        // branch to output curve
//        if (bCreateDegenerateCurve) 
//          {
//            // Create a degenerate curve
//            SER(SmBSplineCurve::CreateDegenerateCurve(crContext,3,sDegenerateCurvePoint,rpCircle));
//            NER(rpCircle);
//          }
//        else // create a circle
//          {
//            // create and return circle
//            // gwc:: create and return type SmCircle rather than BSPline isoParam curve
//            SmPoint3d        sCircleCenter  =   m_vPosition.GetOriginRef()
//                                              + dIsoParam * m_vPosition.GetZAxis() ;
//            SmAxis2Placement sCirclePosition(sCircleCenter, 
//                                             m_vPosition.GetXAxisRef(),
//                                             m_vPosition.GetYAxisRef()) ;
//            double           dRadius = GetRadius(dIsoParam) ;
//            SmExtent1d       sIvl ;
//            if (eLinearParam == SM_SP_U) { sIvl.SetMinMax(m_vAnalUVDomain.GetMin().y,
//                                                          m_vAnalUVDomain.GetMax().y) ;
//                                         }
//            else                         { sIvl.SetMinMax(m_vAnalUVDomain.GetMin().x,
//                                                          m_vAnalUVDomain.GetMax().x) ;
//                                         }
//            SER(SmCircle::CreateCanonical(crContext, sCirclePosition,  dRadius,
//                                          (SmCircle *&)rpCircle, &sIvl)) ;
//            //  SER(CreateIsoParametricCurve(crContext,eLinearParam,dIsoParam,0.0,rpCircle));
//          }
//      
//        // all done
//        return SM_SUCCESS;

} // end SmCone::CreateIsoCircleFromPoint

/*******************************************************************//**
PURPOSE: Convert from STEP to NURBS parameterization 
            so that EvaluatePoint(NurbUV) = EvaluateSTEPPoint(StepUV).

NOTES: 
***********************************************************************/
SmStatus SmCone::ConvertUVFromSTEPToNURBS
  (const SmPoint2d & crSTEPUV,  // in : Target 2d Point in [degrees, height] 
   SmPoint2d       & rNURBSUV)  // out: 2d Point in SMLib parameterization            
  const
{
  SM_ASSERT(HavePolarConversion()) ;

  // convert sweep coordinate
  SmBoolean bExactConversion;
  SER(m_vPolarConverter.ConvertToNURBSParameter(crSTEPUV.x,rNURBSUV.x,bExactConversion));

  // get current STEP and NURB intervals switching on m_bSwapUV as needed
  SmExtent1d sStepIvl = m_vAnalUVDomain.GetVInterval() ;
  SmExtent1d sNurbIvl =   m_bSwapUV
                        ? GetNaturalUVDomain().GetUInterval()
                        : GetNaturalUVDomain().GetVInterval() ;

  // convert linear coordinate
  if(IsBounded()) { rNURBSUV.y =   m_bInsideOut
                                 ? (  sNurbIvl.GetMax() 
                                    + (  (crSTEPUV.y - sStepIvl.GetMin()) 
                                       /  sStepIvl.GetLength() 
                                       * -sNurbIvl.GetLength()))
                                 : (  sNurbIvl.GetMin() 
                                    + (  (crSTEPUV.y - sStepIvl.GetMin()) 
                                       / sStepIvl.GetLength() 
                                       * sNurbIvl.GetLength())) ;
                  } 
  else            { rNURBSUV.y =   m_bInsideOut
                                 ? -crSTEPUV.y
                                 :  crSTEPUV.y ;
                  }
 
  // m_bSwapUV - switch Nurb point coordinates when needed
  if(m_bSwapUV)
    {
      double dTemp = rNURBSUV.x ;
      rNURBSUV.x   = rNURBSUV.y ;
      rNURBSUV.y   = dTemp ;
    }                   

#ifdef SM_DEBUG_CODE

  // check conversions reciprocity
  SmPoint2d sCheckUV ;
  ConvertUVFromNURBSToSTEP(rNURBSUV, sCheckUV) ; 
  double dUVDist       = (crSTEPUV - sCheckUV).Length() ;

  if (!(dUVDist < SM_EFF_ZERO_PARAM))
  {
      SM_ASSERT(dUVDist < SM_EFF_ZERO_PARAM);

      TCHAR sBuff[SM_TBLOCK_SIZE];
      smos_sprintf(sBuff, _T("SmCone::ConvertUVFromSTEPToNURBS %16.16lf > %16.16lf\n"), dUVDist, SM_EFF_ZERO_PARAM);
      smos_WriteBuffer(sBuff); 
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
  double dScaled3dZero = SM_EFF_ZERO * 10.0 * (1.0 + sNurbPoint[0].GetMaxDimension()) ;
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
      SM_ASSERT(SM_ARE_SAME_TO_TOL(1.0, dInvert * sStepZ.Dot(sNurbZ), SM_EFF_ZERO * 100.0)) ;   
    }

  SmBoolean bDebugMe = FALSE;
  if(bDebugMe)
    {
      Dump() ;

      SmFace *pFace = (SmFace*)this->GetFace() ;
      SmBrep *pBrep = (pFace != NULL) ? pFace->GetBrep() : NULL ;

      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,1) ; this->DrawUV() ; sm_GraphicsLoop() ;
      smgfx_SetLook(4,5, 1,0,0) ; m_pGenCurve->Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook(2,3, 0,1,0) ; m_vPosition.Draw() ; sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;
    }
#endif

  // all done
  return SM_SUCCESS;    

} // end SmCone::ConvertUVFromSTEPToNURBS

/*******************************************************************//**
PURPOSE: Convert from NURBS to STEP parameterization
            so that EvaluatePoint(NurbUV) = EvaluateSTEPPoint(StepUV)

NOTES: 
***********************************************************************/
SmStatus SmCone::ConvertUVFromNURBSToSTEP
  (const SmPoint2d & crNURBSUV,  // in : Target 2d Point in SMLib parameterization
   SmPoint2d       & rSTEPUV)    // out: 2d Point in [degrees, height]
  const
{
  SM_ASSERT(HavePolarConversion()) ;

  // swap NurbPoint when needed
  SmExtent1d sStepIvl = m_vAnalUVDomain.GetVInterval() ;
  SmExtent1d sNurbIvl ;
  SmPoint2d sNURBUV ;
  if (m_bSwapUV) { sNURBUV.x = crNURBSUV.y ; // sweep  param
                   sNURBUV.y = crNURBSUV.x ; // linear param
                   sNurbIvl  = GetNaturalUVDomain().GetUInterval() ;
                 }
  else           { sNURBUV.x = crNURBSUV.x ;
                   sNURBUV.y = crNURBSUV.y ;
                   sNurbIvl  = GetNaturalUVDomain().GetVInterval() ;
                 }

  // convert sweep coordinate
  SmBoolean bExactConversion;
  SER(m_vPolarConverter.ConvertToPolarParameter(sNURBUV.x,rSTEPUV.x,bExactConversion));
  SM_ASSERT(bExactConversion) ;
  
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  if(bDebugMe)
    {
      SM_DUMP_AND_ASSERT_VALID(this) ;
    }
#endif // SM_DEBUG_CODE

  // convert linear coordinate
  if(IsBounded()) { rSTEPUV.y =   m_bInsideOut
                                ? (  sStepIvl.GetMin()
                                   + (  (sNURBUV.y - sNurbIvl.GetMax())
                                      / -sNurbIvl.GetLength()
                                      *  sStepIvl.GetLength()))
                                : (  sStepIvl.GetMin()
                                   + (  (sNURBUV.y - sNurbIvl.GetMin())
                                      / sNurbIvl.GetLength()
                                      * sStepIvl.GetLength())) ;
                  } 
  else            { rSTEPUV.y = m_bInsideOut
                                  ? -sNURBUV.y
                                  :  sNURBUV.y ;
                  } 
                  
  // all done
  return SM_SUCCESS;    

} // end SmCone::ConvertUVFromNURBSToSTEP

/*******************************************************************//**
PURPOSE: Given the parameteric value of the curve determine the
    corresponding Euclidian point and optionally derivatives.

NOTES: For discontinuous points on the curve the derivatives
    may be obtained either from the left or from the right of the 
    discontinuity.
***********************************************************************/
SmStatus SmCone::Evaluate
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
  SmBoolean)                 // in : bDoZeroSampling = for internal use only, always set to TRUE, default:[TRUE]
 const  
{
  // get the Nurb evaluation, for derivatives
  SmStatus sNURBRtn = SM_SUCCESS;
  
  // when asked for derivatives
  if ( lHighestUDeriv > 0 || lHighestVDeriv > 0 )
    {
      // pass the call along to base class
      sNURBRtn = SmBSplineSurface::Evaluate( crUV,
                                             lHighestUDeriv, 
                                             lHighestVDeriv,
                                             bUFromLeft, 
                                             bVFromLeft, 
                                             bOnlyUpperHalf,
                                             aDerivatives,
                                             bNonZeroTangents );
    }

  // use the step position for tighter tolerances
  SmPoint2d dSTEPUV  ;
  SmPoint3d dSTEPPoint ;

  SmStatus sSTEPRtn = ConvertUVFromNURBSToSTEP(crUV, dSTEPUV) ;

  if ( sSTEPRtn == SM_SUCCESS )
    { sSTEPRtn = EvaluateSTEP(dSTEPUV, 0, 0, TRUE, TRUE, TRUE, &dSTEPPoint); }
   
  if ( sSTEPRtn == SM_SUCCESS )
    { aDerivatives[0] = dSTEPPoint; }

  // all done
  return(  ( //  sSTEPRtn == SM_SUCCESS &&    ... position is set, by Nurbs...
                 sNURBRtn == SM_SUCCESS)
         ? SM_SUCCESS : SM_ERR ) ;

} // end SmCone::Evaluate

/*******************************************************************//**
PURPOSE: Given the parametric value of the surface determine the
    corresponding Euclidian point on the surface.      

NOTES: This methods assumes the surface is not discontinuous
    in its position.  The parameter must be within the natural range
    of the curve.
***********************************************************************/
SmStatus SmCone::EvaluatePoint
  (const SmPoint2d & rNurbUV,       // in : Target Nurb Parameter
         SmPoint3d & rPoint )       // out: euclidean point
 const
{ 
  // convert to STEP parameter
  SmPoint2d dSTEPUV  ;
  SER( ConvertUVFromNURBSToSTEP( rNurbUV, dSTEPUV ));

  // pass the call along
  return(EvaluateSTEP(dSTEPUV, 0, 0, TRUE, TRUE, TRUE, &rPoint)) ;

} // end SmCone::EvaluatePoint

/*******************************************************************//**
PURPOSE: Evaluate the point & derivatives using the STEP parameterization.
            EvaluatePoint(NurbUV) = EvaluateSTEPPoint(StepUV).
            But tangents will vary in magnitude and
            norm may be negated

NOTES:     
  Cone(U,V) =   origin                             
              + cos(U) * Rad(V) * XAxis
              + sin(U) * Rad(V) * YAxis
              + V               * ZAxis ;
***********************************************************************/
SmStatus SmCone::EvaluateSTEP
  (const SmPoint2d & crUV,    // in : U=CCW Rot about Z from X in degrees, [0 to 360]
                              //      V=param from bottom to top: linear with distance
                              //      when m_bMakeNurbGenCurve = FALSE, V is height along axis
                              //           m_bMakeNurbGenCurve = TRUE , V is parameterization of gen curve
   ULONG lHighestUDeriv,      // in : Requested triangular derivative order in U; must equal lHighestVDeriv; max 3
   ULONG lHighestVDeriv,      // in : Requested triangular derivative order in V; must equal lHighestUDeriv; max 3
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

  // per SMLib practice clamp out of bounds UV points
  if ( ! IsOutOfBoundsEnabled() && !m_vAnalUVDomain.ContainsPoint2d(sUV) )
    { sUV = m_vAnalUVDomain.ClampPoint2d(sUV); }

  // evaluate expensive radius and trigonometric functions
  double dV    = sUV.y ;
  double dR    = GetRadius(dV) ;
  double dRv   =  (GetRadius(sLinearIvl.GetMax()) - GetRadius(sLinearIvl.GetMin()))
                 / sLinearIvl.GetLength() ;
  double dCosU = smos_Cosine(sUV.x*SM_PI/180.0) ;
  double dSinU = smos_Sine  (sUV.x*SM_PI/180.0) ;

  // evaluate position
  aDerivatives[0] = rO  +  dR * ( dCosU * rX + dSinU * rY )  +  dV * sZ;

  // derivatives
  //  Su   = dR  * ( -dSinU * rX + dCosU * rY);
  //  Sv   = dRv * (  dCosU * rX + dSinU * rY) + sZ;
  //  Suu  = dR  * ( -dCosU * rX - dSinU * rY);
  //  Svv  =  0;
  //  Suv  = dRv * ( -dSinU * rX + dCosU * rY);
  //  Suuu = dR  * (  dSinU * rX - dCosU * rY);
  //  Suuv = dRv * ( -dCosU * rX - dSinU * rY);
  //  Suvv =  0;
  //  Svvv =  0;


  // 1st U and V derivatives
  if(lHighestUDeriv >= 1)   { aDerivatives[lHighestVDeriv+1] = dR  * (- dSinU * rX + dCosU * rY); }
  if(lHighestVDeriv >= 1)   { aDerivatives[1]                = dRv * (  dCosU * rX + dSinU * rY) + sZ; }

  // cross derivative dUV
  if(   lHighestUDeriv >= 1
     && lHighestVDeriv >= 1
     && lMinHighest    >= 2){ aDerivatives[lHighestVDeriv+2] = dRv * (- dSinU * rX + dCosU * rY); }

  //  cross derivative dUVV
  if(   lHighestUDeriv >= 1
     && lHighestVDeriv >= 2
     && lMinHighest    >= 3){ aDerivatives[lHighestVDeriv+3].x = 0.0;
                              aDerivatives[lHighestVDeriv+3].y = 0.0;
                              aDerivatives[lHighestVDeriv+3].z = 0.0;
                            }

  // 2nd U and V derivatives
  if(lHighestUDeriv >= 2)   { aDerivatives[2*lHighestVDeriv+2] = dR * (- dCosU * rX - dSinU * rY); }
  if(lHighestVDeriv >= 2)   { aDerivatives[2].x = 0.0;
                              aDerivatives[2].y = 0.0;
                              aDerivatives[2].z = 0.0;
                            }

  // cross derivative dUUV
  if(   lHighestUDeriv >= 2
     && lHighestVDeriv >= 1
     && lMinHighest    >= 3){ aDerivatives[2*lHighestVDeriv+3] = dRv * (- dCosU * rX - dSinU * rY); }

  // 3rd U and V derivatives
  if(lHighestUDeriv >= 3)   { aDerivatives[3*lHighestVDeriv+3] = dR * (  dSinU * rX - dCosU * rY); }
  if(lHighestVDeriv >= 3)   { aDerivatives[3].x = 0.0;
                              aDerivatives[3].y = 0.0;
                              aDerivatives[3].z = 0.0;
                            }


  // The formulas above are per radian; STEP parameters are in degrees (U is degrees; V is linear along the axis).
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

} // end SmCone::EvaluateSTEP

/*******************************************************************//**
PURPOSE: Evaluate the point using the STEP parameterization.

NOTES: crUV:[0_to_360, distance along axis]
  when Cone was built canonically, m_bMakeNurbGenCurve = FALSE
       and GenCurve will be an SmLine object.
    U = CCW angle in degrees about Z axis from X axis [0_to_360] and
    V = linear distance along axis vector.

  when Cone was built with m_bMakeNurbGenCurve = TRUE
       GenCurve will be an SmBSplineCurve object.
    U = CCW angle in degrees about Z axis from X axis [0_to_360] and
    V = linear distance along axis vector.
***********************************************************************/
SmStatus SmCone::EvaluateSTEPPoint
  (const SmPoint2d & crUV,   // in : target UVPoint, range:[0 to 360, -90 to 90]
   SmPoint3d       & rPoint) // out: sphere point
  const
{
  // pass the call along
  return( EvaluateSTEP(crUV, 0, 0, TRUE, TRUE, TRUE, &rPoint ));

} // end SmCone::EvaluateSTEPPoint

/*******************************************************************//**
PURPOSE: Get the height of this cone.

NOTES: 
   The STEP v-param should always run from 0 -> Height.
***********************************************************************/
double SmCone::GetHeight() const
{
  if (m_pNurb == NULL) 
    {
      SM_ASSERT( m_vAnalUVDomain.GetMin().y == 0.0 );
      return m_vAnalUVDomain.GetMax().y - m_vAnalUVDomain.GetMin().y;
    }
  else 
    {
      SmPoint2d sTopUV = m_bInsideOut ? GetNaturalUVDomain().GetMin()
                                      : GetNaturalUVDomain().GetMax() ;
      SmPoint2d sBotUV = m_bInsideOut ? GetNaturalUVDomain().GetMax()
                                      : GetNaturalUVDomain().GetMin() ;
      SmPoint3d sTop, sBot;
      EvaluatePoint(sTopUV,sTop);
      EvaluatePoint(sBotUV,sBot);
      double dHeight = (sTop - sBot).Dot(m_vPosition.GetZAxis()) ;
      return dHeight;
      //SM_ASSERT(dHeight > 0.0) ;
    }

} // end GetHeight

/*******************************************************************//**
PURPOSE: This method returns the range of cone's linear parameter

NOTES: 
***********************************************************************/
SmExtent1d SmCone::GetSTEPLinearParamExtent
  (SmSurfParamType * pSurfParam)  // out: cone linear STEP direction
                                  //      always SM_SP_V for cones

 const
{
  if(pSurfParam) { *pSurfParam = SM_SP_V ; }
  return(m_vAnalUVDomain.GetVInterval()) ;

} // end SmCone::GetSTEPLinearParamExtent

/*******************************************************************//**
PURPOSE: Get the maximum allowable domain for an analytic representaion
   of this surface.

NOTES: 
   The u/v parameters correspond to the STEP parameterization.
***********************************************************************/
SmExtent2d SmCone::GetMaxAnalyticDomain()
 const
{
  // The v-domain is the domain of the generator curve, which is a line.
  // However, we must limit that to our apex.
  SmExtent1d sVDom = this->GetSTEPLinearParamExtent();

  // Get the apex parameter.
  double dApexParam;
  SmPoint3d sApexPt;
  SmBoolean bIsCylinder;
  this->GetApex( bIsCylinder, sApexPt, dApexParam );

  // The apex parameter must not be within our current v-domain.
  // If it is, we will just return our current v-domain, not expanded.
  // Unless it's a cylinder, then there's no limitation.

  SmExtent1d sVDomBig = m_pGenCurve->GetMaxAnalyticDomain();
  if ( bIsCylinder )
    { sVDom = sVDomBig; }
  else if ( dApexParam <= sVDom.GetMin()+SM_EFF_ZERO )
    { sVDom.SetMinMax( dApexParam, sVDomBig.GetMax() ); }
  else if ( dApexParam >= sVDom.GetMax()-SM_EFF_ZERO )
    { sVDom.SetMinMax( sVDomBig.GetMin(), dApexParam ); }

  // u-domain is 0 to 360.
  SmExtent2d sDom( 0.0, sVDom.GetMin(), 360.0, sVDom.GetMax() );

  return sDom;

} // end SmCone::GetMaxAnalyticDomain

/*******************************************************************//**
PURPOSE: This virtual method may invoke special cases of intersection
    of cones with other analytics.

NOTES: 
***********************************************************************/
SmStatus SmCone::GlobalSurfaceIntersect
  (const SmContext     & crContext,               // in : Context for the creation of curves
   const SmExtent2d    & crUVDomain,              // 
   const SmSurface     & crOtherSurface,          // in : target 2nd intersecting surface
   const SmExtent2d    & crOtherUVDomain,         //
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
  SM_DUMP_AND_ASSERT2_VALID(this) ;
  // locals
  SmBoolean bNeedNurbIntersection = TRUE;

  // switch on OtherSurface Type looking for cheap special intersection case
  switch (crOtherSurface.GetType()) 
    {
      case SmPlane_TYPE:
          SER(IntersectWithPlane
                (crContext,                  crUVDomain,
                (SmPlane&)crOtherSurface,    crOtherUVDomain, 
                bUseSurfaceEdges,            pdOptApproxTol3d,  pdOptAngTolRad,
                bNeedNurbIntersection, 
                pOpt3DCurves, pOptSurface1UVCurves, pOptSurface2UVCurves, pOptCurveTypes, pOptDeviations));
          break;

      case SmCone_TYPE:
          SER(IntersectWithCone(crContext,crUVDomain,(SmCone&)crOtherSurface,
              crOtherUVDomain, bUseSurfaceEdges, pdOptApproxTol3d,
              pdOptAngTolRad, bNeedNurbIntersection, 
              pOpt3DCurves, pOptSurface1UVCurves,
              pOptSurface2UVCurves, pOptCurveTypes, pOptDeviations));
          break;

      case SmSphere_TYPE:
          {
              SmBoolean bTmpUseEdges[2];
              bTmpUseEdges[0] = bUseSurfaceEdges[1];
              bTmpUseEdges[1] = bUseSurfaceEdges[0];
              SmSphere & crSphere = (SmSphere&)crOtherSurface;
              SER(crSphere.IntersectWithCone(crContext,crOtherUVDomain,
                  *this,crUVDomain, bTmpUseEdges, pdOptApproxTol3d,
                  pdOptAngTolRad, bNeedNurbIntersection, 
                  pOpt3DCurves, pOptSurface2UVCurves,
                  pOptSurface1UVCurves, pOptCurveTypes, pOptDeviations));
          }
          break;

      case SmSurfOfExtrusion_TYPE:
          if (IsCylinder()) 
            {
              SmBoolean bTmpUseEdges[2];
              bTmpUseEdges[0] = bUseSurfaceEdges[1];
              bTmpUseEdges[1] = bUseSurfaceEdges[0];
              SmSurfOfExtrusion & crSurfOfExt = (SmSurfOfExtrusion&)crOtherSurface;
              SER(crSurfOfExt.IntersectWithCylinder(crContext,crOtherUVDomain,
                  *this,crUVDomain, bTmpUseEdges, pdOptApproxTol3d,
                  pdOptAngTolRad, bNeedNurbIntersection, 
                  pOpt3DCurves, pOptSurface2UVCurves,
                  pOptSurface1UVCurves, pOptCurveTypes, pOptDeviations));
            }
          break;

      case SmSurfOfRevolution_TYPE:
          {
              SmBoolean bTmpUseEdges[2];
              bTmpUseEdges[0] = bUseSurfaceEdges[1];
              bTmpUseEdges[1] = bUseSurfaceEdges[0];
              SmSurfOfRevolution & crSurfOfRev = (SmSurfOfRevolution&)crOtherSurface;
              SER(crSurfOfRev.IntersectWithCone(crContext,crOtherUVDomain,
                  *this,crUVDomain, bTmpUseEdges, pdOptApproxTol3d,
                  pdOptAngTolRad, bNeedNurbIntersection, 
                  pOpt3DCurves, pOptSurface2UVCurves,
                  pOptSurface1UVCurves, pOptCurveTypes, pOptDeviations));
          }
          break;

      default:
          break;

    } // end switch on crOtherSurface.GetType()

  // If we're here, then analytic intersections were not found
  if (bNeedNurbIntersection) 
    {
      SER(SmSurface::GlobalSurfaceIntersect
         (crContext,crUVDomain,crOtherSurface,
          crOtherUVDomain,bUseSurfaceEdges,pdOptApproxTol3d,
          pdOptAngTolRad,pOpt3DCurves,pOptSurface1UVCurves,
          pOptSurface2UVCurves,pOptCurveTypes,pOptDeviations));
    }

  return SM_SUCCESS;

} // end SmCone::GlobalSurfaceIntersect



/*******************************************************************//**
PURPOSE: Local method to test whether two SmLines or SmCircles are totally coincident.

NOTES: 
***********************************************************************/
static
SmStatus sm_TotalCoincidenceChecker(
            const SmCurve *cpThis,
            const SmExtent1d &crThisDomain,
            const SmCurve *cpOther,
            const SmExtent1d &crOtherDomain,
            const SmApproxTol3d sApproxTol3d,
            SmBoolean & rbAreCoinc,    // out
            SmBoolean & rbReversed)    // out
{
  // Init return flags for no coincidence.
  rbAreCoinc = FALSE;
  rbReversed = UNSURE;

  // First try equality operator.
  if ( *cpThis == *cpOther )
  {
      rbAreCoinc = TRUE;
      rbReversed = FALSE;
      return SM_SUCCESS;
  }

  if ( cpThis->GetType() != cpOther->GetType() )
    { return SM_SUCCESS; }

  // Both either SmLine or SmCircle.

  // Using end points, set bRev flag, and quit if not coincident.
  SmBoolean bRev = UNSURE;
  SmPoint3d sPt00, sPt01, sPt10, sPt11;
  cpThis ->EvaluatePoint( crThisDomain .GetMin(), sPt00 );
  cpThis ->EvaluatePoint( crThisDomain .GetMax(), sPt01 );
  cpOther->EvaluatePoint( crOtherDomain.GetMin(), sPt10 );
  cpOther->EvaluatePoint( crOtherDomain.GetMax(), sPt11 );

  double dDist_S_S = sPt00.DistanceBetween( sPt10 );
  double dDist_S_E = sPt00.DistanceBetween( sPt11 );
  double dDist_E_S, dDist_E_E;

  if ( cpThis->IsKindOf( SmLine_TYPE ) )
  {
      dDist_E_E = sPt01.DistanceBetween( sPt11 );
      if ( dDist_S_S < sApproxTol3d && dDist_E_E < sApproxTol3d )
      {
          rbAreCoinc = TRUE;
          rbReversed = FALSE;
          return SM_SUCCESS;
      }
      dDist_E_S = sPt01.DistanceBetween( sPt11 );
      if ( dDist_S_E < sApproxTol3d && dDist_E_S < sApproxTol3d )
      {
          rbAreCoinc = TRUE;
          rbReversed = TRUE;
          return SM_SUCCESS;
      }
      return FALSE;
  }

  // Not a line.  If not a circle, return no coincidence.
  if ( ! cpThis->IsKindOf( SmCircle_TYPE ) )
    { return SM_SUCCESS; }

  if ( dDist_S_S < sApproxTol3d && dDist_S_E < sApproxTol3d )
  {
      // Start pt 1 coinc with both start pt2 and end pt2:
      // crv 2 is closed.  Check crv1 closed.
      dDist_E_S = sPt01.DistanceBetween( sPt10 );
      if ( dDist_E_S > sApproxTol3d )
        { return SM_SUCCESS; }

      // Both curves closed, same endpoints.
      // Check 1st derivs for orientation.
      SmVector3d sPV1[2], sPV2[2];
      cpThis ->Evaluate( crThisDomain .GetMin(), 1, TRUE, sPV1, TRUE );
      cpOther->Evaluate( crOtherDomain.GetMin(), 1, TRUE, sPV2, TRUE );
      double dDot = sPV1[1].Dot( sPV2[1] );
      bRev = ( dDot < -SM_EFF_ZERO );
  }
  else if ( dDist_S_S < sApproxTol3d )
  {
      // Start points coincident, start/end not.  Check end points.
      dDist_E_E = sPt01.DistanceBetween( sPt11 );
      if ( dDist_E_E > sApproxTol3d )
        { return SM_SUCCESS; }
      bRev = FALSE;
  }
  else if ( dDist_S_E < sApproxTol3d )
  {
      // Start pt 1 == end pt 2, start pt 2 distinct.
      dDist_E_S = sPt01.DistanceBetween( sPt10 );
      if ( dDist_E_S > sApproxTol3d )
        { return SM_SUCCESS; }
      bRev = TRUE;
  }
  else
  {
      // No endpoint coincidence.
      return SM_SUCCESS;
  }

  // Here, start and end points are coincident, bRev is set.
  // Check interior.  For circles, one interior point is sufficient.
  double dDist;

  cpThis ->EvaluatePoint( crThisDomain .Evaluate( 0.5 ), sPt01 );

  // If reversed, parameterization could vary.
  double dT2 = crOtherDomain.Evaluate( 0.5 );
  if ( ! bRev )
  {
      cpOther->EvaluatePoint( dT2, sPt11 );
      dDist = sPt01.DistanceBetween( sPt11 );
  }
  else
  {
      SmBoolean bOk;
      cpOther->DropPoint( crOtherDomain, sPt01, NULL, sApproxTol3d, &dT2,
                          bOk, dT2, dDist,
                          SM_SO_INTERSECT );
      if ( !bOk ) { dDist = SM_BIG_DOUBLE; }
  }

  if ( dDist > sApproxTol3d )
    { return SM_SUCCESS; }

  // All good: coincident curves.
  rbAreCoinc = TRUE;
  rbReversed = bRev;

  return SM_SUCCESS;

}  // end static sm_TotalCoincidenceChecker


/*******************************************************************//**
PURPOSE: Find intersection curve(s) of this with other cone

NOTES: 
  when either cone is not a cylinder
    - set rbNeedsMoreIntersections == TRUE and all output arrays to empty

  when two cones are coincident cylinders
    - when overlapping axis interval is a point     - return 1 intersection circle curve
    - when overlapping axis interval is an interval - return both surface boundaries projected to the other

  when two cones are nonCoincident and parallel
    - when axis interval is a point     - return 1 or 2 lines of length 100.0 * dTol on xsect points (why not points)
    - when axis interval is an interval - return 1 or 2 xsect lines parallel to cylinder axis

  when
***********************************************************************/
SmStatus SmCone::IntersectWithCone
  (const SmContext     & crContext,                // in : context for new object construction 
   const SmExtent2d    & crUVDomain,               // in : This cone intersection NurbDomain
   const SmCone        & crOtherCone,              // in : other cone
   const SmExtent2d    & crOtherUVDomain,          // in : other cone intersection NurbDomain
   const SmBoolean [2],                            // in : bUseSurfaceEdges - not used
   const SmApproxTol3d * pdOptApproxTol3d,         // in : intersection 3d tol or
                                                   //      SM_EFF_ZERO_SQRT * (1.0 + otherConeBBox.Length()) 
   const double        * ,                         // in : pdOptAngTolRad - not used
   SmBoolean           & rbNeedsMoreIntersections, // out: TRUE = use general surf/surf intersector to find intersection
                                                   //      FALSE= intersection curves returned by this function
   SmTArray<SmCurve*>  * pOpt3DCurves,             // out: required array of output intersections
   SmTArray<SmCurve*>  * pOptSurface1UVCurves,     // out: optional associated cone1 UVTrimCurves
   SmTArray<SmCurve*>  * pOptSurface2UVCurves,     // out: optional associated cone2 UVTrimCurves
   SmTArray<SmTsectCurveType> * pOptCurveTypes,    // out: optional associated intersection types
   SmTArray<double>    * pOptDeviations)           // out: optional associated deviations fro each xSect Curve
  const
{
  SM_DUMP_AND_ASSERT2_VALID(this) ;
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
  static ULONG lCount      = 1 ; lCount++ ;
  static ULONG lDebugCount = 0 ;
  if (bDebugMe || lCount == lDebugCount) 
    {
      SmFace *pFace1 = (SmFace *)GetFace() ;
      SmFace *pFace2 = (SmFace *)crOtherCone.GetFace() ;
      SmBrep *pBrep1 = pFace1 ? pFace1->GetBrep() : NULL ;
      SmBrep *pBrep2 = pFace2 ? pFace2->GetBrep() : NULL ;

      smgfx_Erase();
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep1) { pBrep1->Draw(TRUE) ; } sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,0) ; if(pBrep2) { pBrep2->Draw(TRUE) ; } sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,1) ; DrawUV() ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 1,1,0) ; crOtherCone.DrawUV() ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,0,0) ; if(pFace1) { pFace1->Draw(SM_DM_CROSSHATCH,10,10) ; } sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,0,0) ; if(pFace2) { pFace2->Draw(SM_DM_CROSSHATCH,10,10) ; } sm_GraphicsLoop() ;
      sm_GraphicsLoop();
    }
#endif

  // init outputs
  rbNeedsMoreIntersections = FALSE;
  if (!pOpt3DCurves)        SER(SM_ERR); // Must exist for analytical intersections
  if (pOpt3DCurves)         pOpt3DCurves->ReSet();
  if (pOptSurface1UVCurves) pOptSurface1UVCurves->ReSet();
  if (pOptSurface2UVCurves) pOptSurface2UVCurves->ReSet();
  if (pOptCurveTypes)       pOptCurveTypes->ReSet();
  if (pOptDeviations)       pOptDeviations->ReSet();

  // No work - self intersection request - cones don't self intersect
  if (this == &crOtherCone)
     { return SM_SUCCESS; 
     }

  // get cone bounding boxes and sizes
  SmExtent3d sBBox1, sBBox2;
  SER(CalculateBoundingBox(crUVDomain,&sBBox1));
  SER(crOtherCone.CalculateBoundingBox(crOtherUVDomain,&sBBox2));
  SmVector3d sSize = sBBox2.GetSize();

  // get 3D intersection tolerance
  double dTol      =   pdOptApproxTol3d 
                     ? (double)*pdOptApproxTol3d
                     : SM_EFF_ZERO_SQRT * (1.0 + sSize.Length());

  // no work - disjoint bounding boxes.
  sBBox2.ExpandAbsolute(dTol);
  if (sBBox1.AreDisjoint(sBBox2)) 
    { return SM_SUCCESS;
    }

  // get thisCone axis bot, top, axis, and height
  double           dHeight    = GetHeight() ;
  SmVector3d       sConeAxis  = m_vPosition.GetZAxis() ;

  // const SmPoint3d &rConeXAxis = m_vPosition.GetXAxisRef() ;
  const SmPoint3d &rBasePt    = m_vPosition.GetOriginRef();
  const SmPoint3d &sCenterBot = rBasePt + m_vAnalUVDomain.GetVMin() * sConeAxis;
  const SmPoint3d &sCenterTop = rBasePt + m_vAnalUVDomain.GetVMax() * sConeAxis;

  // get otherCone axis bot, top, axis, and height
  SmVector3d       sOConeAxis  = crOtherCone.GetPosition().GetZAxis();
  const SmPoint3d &rOBasePt    = crOtherCone.m_vPosition.GetOriginRef();
  const SmPoint3d &sOCenterBot = rOBasePt + crOtherCone.m_vAnalUVDomain.GetVMin() * sOConeAxis;
  const SmPoint3d &sOCenterTop = rOBasePt + crOtherCone.m_vAnalUVDomain.GetVMax() * sOConeAxis;

#ifdef SM_DEBUG_CODE
  if (bDebugMe) 
    {
      SmFace *pFace1 = (SmFace *)GetFace() ;
      SmFace *pFace2 = (SmFace *)crOtherCone.GetFace() ;
      SmBrep *pBrep1 = pFace1 ? pFace1->GetBrep() : NULL ;
      SmBrep *pBrep2 = pFace2 ? pFace2->GetBrep() : NULL ;

      smgfx_Erase();
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep1) { pBrep1->Draw(TRUE) ; } sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,0) ; if(pBrep2) { pBrep2->Draw(TRUE) ; } sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,1) ; DrawUV() ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 1,1,0) ; crOtherCone.DrawUV() ; sm_GraphicsLoop() ;
      smgfx_SetLook(4,5, 1,0,0) ; sCenterBot.Draw(); sm_GraphicsLoop() ;
      smgfx_SetLook(4,5, 0,1,0) ; sCenterTop.Draw(); sm_GraphicsLoop() ;
      smgfx_SetLook(6,7, 1,0,1) ; sOCenterBot.Draw(); sm_GraphicsLoop() ;
      smgfx_SetLook(6,7, 0,1,1) ; sOCenterTop.Draw(); sm_GraphicsLoop() ;
      sm_GraphicsLoop();
    }
#endif

  // branch on cone axis directions: either parallel or not
  double dAngTol = SM_EFF_ZERO_SQRT;
  double dAngle;
  SER(sConeAxis.AngleBetween(sOConeAxis,dAngle));
  if (dAngle > SM_PI/2.0) dAngle = SM_PI - dAngle;

  // when cone axes are parallel branch
  if (dAngle < dAngTol) 
    {
      // project otherCone BotPoint onto thisCone axis line
      double dParam;
      SER( smgu_LineClosestPoint( sCenterBot, sConeAxis, sOCenterBot, dParam ));
      SmPoint3d sPnt     = sCenterBot + dParam * sConeAxis;

      // get axis offset distance
      double dDist       = sPnt.DistanceBetween( sOCenterBot );

      // get both cone max and min radii
      double dBotRad  = GetBotRadius();
      double dTopRad  = GetTopRadius();
      double dOBotRad = crOtherCone.GetBotRadius();
      double dOTopRad = crOtherCone.GetTopRadius();

      double dMinRadius  = smos_Min( dBotRad,  dTopRad  );
      double dMaxRadius  = smos_Max( dBotRad,  dTopRad  );
      double dOMinRadius = smos_Min( dOBotRad, dOTopRad );
      double dOMaxRadius = smos_Max( dOBotRad, dOTopRad );

      // no work -    cones don't xSect due to large axis offset
      //         - or thisCone inside otherCone
      //         - or otherCone inside thisCone
      if(   (dDist - dTol > dMaxRadius + dOMaxRadius)  // cones offset
         || (dMaxRadius +dTol < dOMinRadius - dDist)    // thisCone inside OtherCone
         || (dOMaxRadius+dTol < dMinRadius  - dDist))   // OtherCone inside thisCone
        { return SM_SUCCESS;
        }

      // no work: cones disjoint along the parallel axis
      //         -    OtherCone is completely above thisCone
      //         - or OtherCone is completely below thisCone
      double dParam2;
      // Note, dParam and dParam2 are in terms of our bottom and top,
      // which is not necessarily the same as our v-parameterization.
      // In this parameterization, our bottom is 0 and our top is dHeight.

      SER( smgu_LineClosestPoint( sCenterBot, sConeAxis, sOCenterTop, dParam2 ));
      if (   (dParam > dHeight + dTol && dParam2 > dHeight + dTol) 
          || (dParam < 0.0     - dTol && dParam2 < 0.0     - dTol))
        { return SM_SUCCESS;
        }

      // identify case
      //  - either cone not a cylinder - send to general intersection routine
      //  - both cones are cylinders and dist == 0.0 - cylinders are coincident
      //  - both cones are cylinders and dist > 0    - cylinders xSect in lines
      SmBoolean bLinesOfIntersection = FALSE;
      SmBoolean bCoincidence = FALSE;

      // when both cones are cylinders
      if ( IsCylinder() && crOtherCone.IsCylinder() )
        {
          // when dDist > dTol
          if(dDist > dTol) { bLinesOfIntersection = TRUE;
                           }
          else             { bCoincidence = TRUE;
                           }
        }
      else // no work - either cone is not a cylinder
        {
          // note: leave this check here so that 
          //       some surf/surf intersections can be culled
          //       without calling the general surf/surf intersector when possible
          rbNeedsMoreIntersections = TRUE;
          return SM_SUCCESS;
        }

      SM_ASSERT(bLinesOfIntersection || bCoincidence) 


      // arrive here when cones are two parallel cylinders

      // get axisIntersection interval

      // get the axis intervals for both cylinders
      // Recall, these are not our v-parameters, they are based on 0 to Height.
      double dMin      = 0.0;        // Our current bottom and top.
      double dMax      = dHeight;
      double dMinOther = smos_Min(dParam,dParam2); // Other's bottom and top project on us.
      double dMaxOther = smos_Max(dParam,dParam2);
      SM_ASSERT( dMinOther <= dMaxOther );

      // shorten interval to point when cylinder ends touch on same plane (to tolerance) 
      if (dMinOther > dMax-dTol) { dMin = dMax; }
      if (dMaxOther < dMin+dTol) { dMax = dMin; }
      
      // adjust min/max to be interval of axisIntersection.
      // get the largest minParam and the smallest maxParam
      if (dMinOther > dMin && dMinOther <= dMax) { dMin = dMinOther; }
      if (dMaxOther < dMax && dMaxOther >= dMin) { dMax = dMaxOther; }

      // If cylinders are coincident, drop all boundaries of each surface to the other.
      // If they just touch, return only one circle.
      if (bCoincidence) 
        {
          SmTArray<SmCurve*>         s3DCurves;
          SmTArray<SmCurve*>         sUVCurves1;
          SmTArray<SmCurve*>         sUVCurves2;
          SmTArray<SmTsectCurveType> sCurveTypes;
          SmTArray<double>           sDeviations;

          SmTArray<SmCurve*> *pUVCurves1  = ( pOptSurface1UVCurves   != NULL ) ? &sUVCurves1  : NULL;
          SmTArray<SmCurve*> *pUVCurves2  = ( pOptSurface2UVCurves   != NULL ) ? &sUVCurves2  : NULL;
          SmTArray<double>   *pDeviations = ( pOptDeviations         != NULL ) ? &sDeviations : NULL;
          SmTArray<SmTsectCurveType> *pCurveTypes = ( pOptCurveTypes != NULL ) ? &sCurveTypes : NULL;

          SmStatus eStat = this->IntersectSurfaceBoundaries(
              crContext,        // in : context for new object construction
              crUVDomain,       // in : This cone intersection NurbDomain
              crOtherCone,      // in : other cone
              crOtherUVDomain,  // in : other cone intersection NurbDomain
             *pdOptApproxTol3d, // in : intersection 3d tol or

             &s3DCurves,        // out: required array of output intersections
              pUVCurves1,       // out:
              pUVCurves2,       // out:
              pCurveTypes,      // out: optional associated intersection types
              pDeviations       // out: optional associated deviations for each xSect Curve
          );

          if ( eStat == SM_SUCCESS && s3DCurves.GetSize() > 0 )
            {
              if ( pOpt3DCurves   != NULL ) { pOpt3DCurves  ->Append( s3DCurves   ); }
              if ( pOptCurveTypes != NULL ) { pOptCurveTypes->Append( sCurveTypes ); }
              if ( pOptDeviations != NULL ) { pOptDeviations->Append( sDeviations ); }
              if ( pOptSurface1UVCurves != NULL ) { pOptSurface1UVCurves->Append( sUVCurves1 ); }
              if ( pOptSurface2UVCurves != NULL ) { pOptSurface2UVCurves->Append( sUVCurves2 ); }
            }

          rbNeedsMoreIntersections = ( eStat != SM_SUCCESS );

          // Results of first IntersectSurfaceBoundaries() call are now in the output arguments.
          // Make the second call reusing the local arrays.

          eStat = crOtherCone.IntersectSurfaceBoundaries(
              crContext,        // in : context for new object construction
              crOtherUVDomain,  // in : other cone intersection NurbDomain
             *this,             // in : this cone
              crUVDomain,       // in : this cone intersection NurbDomain
             *pdOptApproxTol3d, // in : intersection 3d tol or

             &s3DCurves,        // out: required array of output intersections
              pUVCurves1,       // out:
              pUVCurves2,       // out:
              pCurveTypes,      // out: optional associated intersection types
              pDeviations       // out: optional associated deviations for each xSect Curve
          );

          if ( eStat == SM_SUCCESS && s3DCurves.GetSize() > 0 )
            {
              // One slight hitch: if the cylinders have coincident boundaries,
              // then each of the previous two calls will return that curve.
              // We have to check for that, basically, remove duplicates.
              // Do that now, before combining the results.
              SmCurve *pCrv1, *pCrv2;
              SmExtent1d sDom1, sDom2;
              SmBoolean bAreCoinc, bReversed;
              SmStatus eStat2;

              for ( ULONG ii=0; ii<pOpt3DCurves->GetSize(); ii++ )
                {
                  pCrv1 = (*pOpt3DCurves)[ii];
                  sDom1 = pCrv1->GetNaturalInterval();
                  for (ULONG jj=0; jj<s3DCurves.GetSize(); jj++ )
                    {
                      pCrv2  = s3DCurves[jj];
                      sDom2  = pCrv2->GetNaturalInterval();

                      eStat2 = sm_TotalCoincidenceChecker(
                                 pCrv1, sDom1,
                                 pCrv2, sDom2,
                                 dTol,
                                 bAreCoinc, bReversed);

                      if ( eStat2 == SM_SUCCESS && bAreCoinc )
                        {
                          // Remove the second one, from the local arrays.
                          s3DCurves.RemoveAt( jj );  // This compresses the array.
                          delete pCrv2;  pCrv2 = NULL;
                          if ( pOptCurveTypes       != NULL ) { sCurveTypes.RemoveAt( jj ); }
                          if ( pOptDeviations       != NULL ) { sDeviations.RemoveAt( jj ); }
                          if ( pOptSurface1UVCurves != NULL ) { delete sUVCurves1[jj]; sUVCurves1.RemoveAt( jj ); }
                          if ( pOptSurface2UVCurves != NULL ) { delete sUVCurves2[jj]; sUVCurves2.RemoveAt( jj ); }

                          // Coincident curves would mean boundary-to-boundary, so:
                          if ( pOptCurveTypes != NULL )
                            { pOptCurveTypes->SetAt( ii, SM_TC_COINCIDENT ); }

                          // Since we removed one from s3DCurves, start over on the inner loop.
                          // It's ok to go on the the next one in the outer loop, so:
                          break;  // Quit inner loop, increment in outer loop.
                        }
                    } // end inner loop on s3DCurves
                } // end outer loop on pOpt3DCurves

              if ( pOpt3DCurves   != NULL ) { pOpt3DCurves  ->Append( s3DCurves   ); }
              if ( pOptCurveTypes != NULL ) { pOptCurveTypes->Append( sCurveTypes ); }
              if ( pOptDeviations != NULL ) { pOptDeviations->Append( sDeviations ); }
              if ( pOptSurface1UVCurves != NULL ) { pOptSurface1UVCurves->Append( sUVCurves1 ); }
              if ( pOptSurface2UVCurves != NULL ) { pOptSurface2UVCurves->Append( sUVCurves2 ); }

            } // end if second IntersectSurfaceBoundaries() call succeeded, with results.

          rbNeedsMoreIntersections = ( eStat != SM_SUCCESS );

          // all done with coincident cylinders
          return eStat;

        } // end coincident cylinder check

      
      // cylinders are parallel and nonCoincident
      ULONG lNumIntersections;
      SmPoint3d sIntersectPoints[6];
      SmPoint3d sStart = sCenterBot + dParam * sConeAxis;

      // intersect coPlanar circles
      SER(smgu_CircleCoPlanarCircleIntersect(sConeAxis,                             // in : circle normals
                                             sStart,     GetBotRadius(),            // in : 1st circle center and radius
                                             sOCenterBot,crOtherCone.GetBotRadius(),     // in : 2nd circle center and radius
                                             dTol,                                  // in : max distance between distinct 3d points
                                             lNumIntersections,sIntersectPoints));  // out: number of xSects and those points
      // all circles should intersect in 1 or 2 points
      if ( lNumIntersections < 3 ) 
        {
          // output an intersection line for every intersection point
          for (ULONG jj=0; jj<lNumIntersections; jj++)
            {
              // This point is at the same height as our Bottom.
              SmPoint3d       sLinePt    = sIntersectPoints[jj] + (dMin-dParam) * sConeAxis;
              double          dLineScale = 1.0;
              SmBSplineCurve *pBSCurve   = NULL ;

              // get NurbUV values for intersection Point to check that linePoint is in both cone domains
              double dScaledZero = SM_EFF_ZERO * (1.0 + sLinePt.GetMaxDimension()) ;
              SmPoint2d      sThisStepUV, sOtherStepUV ;
              SmPoint2d      sThisNurbUV, sOtherNurbUV ;
              SmLocationType eThisLoc, eOtherLoc ;
              STEPInversion(m_vAnalUVDomain, sLinePt, dScaledZero, 
                            sThisStepUV, eThisLoc) ; 
              crOtherCone.STEPInversion(crOtherCone.m_vAnalUVDomain, sLinePt, dScaledZero, 
                                        sOtherStepUV, eOtherLoc) ;
              if(   eThisLoc  == SM_LT_EXTERIOR
                 || eOtherLoc == SM_LT_EXTERIOR)
                 { continue ; }
              ConvertUVFromSTEPToNURBS(sThisStepUV,  sThisNurbUV) ; 
              crOtherCone.ConvertUVFromSTEPToNURBS(sOtherStepUV, sOtherNurbUV) ;

              // skip Points whose NurbUV values are outside of their surface UVDomains
              if(   !crUVDomain.ContainsPoint2d     (sThisNurbUV,  SM_EFF_ZERO)
                 || !crOtherUVDomain.ContainsPoint2d(sOtherNurbUV, SM_EFF_ZERO))
                 { continue ; }  
                  
              // when cones only overlap at a point
              if (dMax-dMin < dTol) 
                {
                  // make a degenerate line
                  SmBSplineCurve::CreateDegenerateCurve(crContext,3,sLinePt,pBSCurve);
                  SM_ASSERT(pBSCurve != NULL) ; 
                  if (pOptCurveTypes) { pOptCurveTypes->Add(SM_TC_TOUCHING); }
                } // end make degenerate point branch
              else // cones overlap by more than a point
                {
                  // make a line
                  SmExtent1d sLineExt(0.0,dMax-dMin);
                  pBSCurve = new (crContext) SmLine(sLinePt,sConeAxis,sLineExt,dLineScale,3);
                  if (pOptCurveTypes) { if(lNumIntersections == 1) 
                                             { pOptCurveTypes->Add(SM_TC_TANGENT ) ; }
                                        else { pOptCurveTypes->Add(SM_TC_CROSSING) ; }
                                      }
                } // end make line branch

              // set output
              if (pOpt3DCurves)         { pOpt3DCurves->Add(pBSCurve); }                       
              if (pOptSurface1UVCurves) { pOptSurface1UVCurves->Add(NULL); }
              if (pOptSurface2UVCurves) { pOptSurface2UVCurves->Add(NULL); }
              if (pOptDeviations)       { pOptDeviations->Add(0.0); }
            
            } // end iter every intersection point

          // all done
          return SM_SUCCESS;
        }
      else // something has gone wrong with the circle/circle intersection 
        {
          // pass the problem to the general intersection call
          rbNeedsMoreIntersections = TRUE;
          return SM_SUCCESS;
        }
    } // end cone axes are parallel branch
  else // cone axes are not parallel branch
    {
      // Check for elliptical intersections of cylinders
      
      // no work - only xSect cylinders of same radius here
      // note: this check is not at the front of this
      //       routine so that some surf/surf intersections can be culled
      //       without calling the general surf/surf intersector when possible
      if (   !IsCylinder() 
          || !crOtherCone.IsCylinder()
          || smos_Fabs(GetTopRadius()-crOtherCone.GetTopRadius()) > dTol) 
        {
          rbNeedsMoreIntersections = TRUE;
          return SM_SUCCESS;
        }

      // intersect cylinder axes 
      ULONG lNumInt;
      SmPoint3d aPoints[2];
      SER(smgu_SegmentSegmentIntersect(sCenterBot, sCenterTop,
                                       sOCenterBot,sOCenterTop,
                                       dTol,
                                       lNumInt,aPoints));

      // no work - don't intersect offset cylinders
      if (lNumInt != 1) 
        {
          rbNeedsMoreIntersections = TRUE;
          return SM_SUCCESS;
        }


      // State: two cylinders of same radius whose axes intersect.

      // locals
      SmTArray<SmBSplineCurve *> sDegenerateCurves ;
       
      // build and output 1st ellipse segment trimmed to both cones             
      SmVector3d sVecToInt1, sVecToInt2;
      SER(smgu_ParallelogramCrossVectors(sConeAxis, sOConeAxis,
                                         GetTopRadius(), crOtherCone.GetTopRadius(),
                                         sVecToInt1, sVecToInt2));
      double dEll1Radius = sVecToInt1.Length();
      SmVector3d sXAxis = sConeAxis * sOConeAxis ;
      SER(sXAxis.Unitize());
      SmVector3d sYAxis = sVecToInt1;
      SER(sYAxis.Unitize());
      SmAxis2Placement sFrame;
      SER(sFrame.SetCanonical(aPoints[0], sXAxis, sYAxis));

      // build and add every ellipse arc segment trimmed to both cones to pOpt3DCurves array
      sm_AddEllipseIntervalsTrimmedToTwoCones
        (crContext,                  // in : for temp object construction
         sFrame.GetOriginRef(),      // in : origin            of ellipse
         sFrame.GetXAxisRef(),       // in : xAxis             of ellipse
         sFrame.GetYAxisRef(),       // in : yAxis             of ellipse
         GetTopRadius(),             // in : x radius          of ellipse
         dEll1Radius,                // in : y radius          of ellipse
         0.0,                        // in : start angle (deg) of ellipse
         180.0,                      // in : end angle (deg)   of ellipse
         *this,                      // in : Cone with trim boundaries
         crUVDomain,                 // in : nurb trim boundaries for cone
         crOtherCone,                // in : Cone with trim boundaries
         crOtherUVDomain,            // in : nurb trim boundaries for cone
         dTol,                       // in : approximation tolerance
         pOpt3DCurves,               // out: accumulate all nonDegnerate intersections trimmed to both cones
         sDegenerateCurves) ;        // out: accumulate all unique Degnerate intersections (they may coincide with pOpt3DCurve)
      
      // build and output 2nd ellipse segment (2nd half of 1st ellipse arc) trimmed to both cones            
      SER(sFrame.SetCanonical(aPoints[0], sXAxis, -sYAxis));
      sm_AddEllipseIntervalsTrimmedToTwoCones
        (crContext,                  // in : for temp object construction
         sFrame.GetOriginRef(),      // in : origin            of ellipse
         sFrame.GetXAxisRef(),       // in : xAxis             of ellipse
         sFrame.GetYAxisRef(),       // in : yAxis             of ellipse
         GetTopRadius(),             // in : x radius          of ellipse
         dEll1Radius,                // in : y radius          of ellipse
         0.0,                        // in : start angle (deg) of ellipse
         180.0,                      // in : end angle (deg)   of ellipse
         *this,                      // in : Cone with trim boundaries
         crUVDomain,                 // in : nurb trim boundaries for cone
         crOtherCone,                // in : Cone with trim boundaries
         crOtherUVDomain,            // in : nurb trim boundaries for cone
         dTol,                       // in : approximation tolerance
         pOpt3DCurves,               // out: accumulate all nonDegnerate intersections trimmed to both cones
         sDegenerateCurves) ;        // out: accumulate all unique Degnerate intersections (they may coincide with pOpt3DCurve)

      // build and output 3rd ellipse segment  (1st half of perpendicular ellipse)            
      double dEll2Radius = sVecToInt2.Length();
      sYAxis = sVecToInt2;
      sYAxis.Unitize();
      SER(sFrame.SetCanonical(aPoints[0], sXAxis, sYAxis));
      sm_AddEllipseIntervalsTrimmedToTwoCones
        (crContext,                  // in : for temp object construction
         sFrame.GetOriginRef(),      // in : origin            of ellipse
         sFrame.GetXAxisRef(),       // in : xAxis             of ellipse
         sFrame.GetYAxisRef(),       // in : yAxis             of ellipse
         GetTopRadius(),             // in : x radius          of ellipse
         dEll2Radius,                // in : y radius          of ellipse
         0.0,                        // in : start angle (deg) of ellipse
         180.0,                      // in : end angle (deg)   of ellipse
         *this,                      // in : Cone with trim boundaries
         crUVDomain,                 // in : nurb trim boundaries for cone
         crOtherCone,                // in : Cone with trim boundaries
         crOtherUVDomain,            // in : nurb trim boundaries for cone
         dTol,                       // in : approximation tolerance
         pOpt3DCurves,               // out: accumulate all nonDegnerate intersections trimmed to both cones
         sDegenerateCurves) ;        // out: accumulate all unique Degnerate intersections (they may coincide with pOpt3DCurve)

      // build and output 4th ellipse segment (2nd half of 2nd perpendicular ellipse)             
      SER(sFrame.SetCanonical(aPoints[0], sXAxis, -sYAxis));
      sm_AddEllipseIntervalsTrimmedToTwoCones
        (crContext,                  // in : for temp object construction
         sFrame.GetOriginRef(),      // in : origin            of ellipse
         sFrame.GetXAxisRef(),       // in : xAxis             of ellipse
         sFrame.GetYAxisRef(),       // in : yAxis             of ellipse
         GetTopRadius(),             // in : x radius          of ellipse
         dEll2Radius,                // in : y radius          of ellipse
         0.0,                        // in : start angle (deg) of ellipse
         180.0,                      // in : end angle (deg)   of ellipse
         *this,                      // in : Cone with trim boundaries
         crUVDomain,                 // in : nurb trim boundaries for cone
         crOtherCone,                // in : Cone with trim boundaries
         crOtherUVDomain,            // in : nurb trim boundaries for cone
         dTol,                       // in : approximation tolerance
         pOpt3DCurves,               // out: accumulate all nonDegnerate intersections trimmed to both cones
         sDegenerateCurves) ;        // out: accumulate all unique Degnerate intersections (they may coincide with pOpt3DCurve)

      // arrive here when pOpt3DCurves contains all elliptical curve intersections
      //             and sDegenerateCurves contains a unique set of degenerate curves
      //                 that may be coincident with one of the elliptical curves.
   
      // when there are degenerate curves
      ULONG lCrvCount = pOpt3DCurves->GetSize() ;
      for(ULONG ii=0;ii<sDegenerateCurves.GetSize();ii++)
        {
          // get the degenerate point
          SmBSplineCurve *pDegenerateCrv = sDegenerateCurves[ii] ;
          SmPoint3d sPoint ;
          SmExtent1d sIvl = pDegenerateCrv->GetNaturalInterval() ;
          pDegenerateCrv->EvaluatePoint(sIvl.GetMin(), sPoint) ;

          // see if its on any of the curve intersections.
          //   Don't check other degnerate curves because the
          //   degenerate curve list only contains unique solutions.
          SmBoolean bIsUnique = TRUE ;
          for(ULONG jj=0;bIsUnique && jj<lCrvCount;jj++)
            {
              // get the ellipse intersection and classify the point
              double dAnalParam ;
              SmCurveLocationType eLoc ;
              SmBSplineCurve *pIntersectCurve = (SmBSplineCurve *)pOpt3DCurves->GetAt(jj) ;
              pIntersectCurve->STEPInversion(sPoint, dAnalParam, &eLoc) ;

              // when point is on Curve - its not unique
              if(eLoc != SM_CL_EXTERIOR)
                { bIsUnique = FALSE ; }

            } // end iter every intersection curve

          // add unique degenerate curves to output - delete duplicates
          if(bIsUnique) { pOpt3DCurves->Add(pDegenerateCrv) ; }
          else          { delete pDegenerateCrv ; pDegenerateCrv = NULL ; }

        } // end iter every degnerate3 curve looking to place it in output
    }  // end nonParallel cone axis branch

  // set all associated data for the ellipse intersections
  for (ULONG ii=0; ii<pOpt3DCurves->GetSize(); ii++)
    {
      if (pOptSurface1UVCurves) { pOptSurface1UVCurves->Add(NULL); }
      if (pOptSurface2UVCurves) { pOptSurface2UVCurves->Add(NULL); }
      if (pOptCurveTypes)       { pOptCurveTypes->Add(SM_TC_CROSSING); }
      if (pOptDeviations)       { pOptDeviations->Add(0.0); }
    }

#ifdef SM_DEBUG_CODE
if (bDebugMe || lCount == lDebugCount) 
  {
    SmFace *pFace1 = (SmFace *)GetFace() ;
    SmFace *pFace2 = (SmFace *)crOtherCone.GetFace() ;
    SmBrep *pBrep1 = pFace1 ? pFace1->GetBrep() : NULL ;
    SmBrep *pBrep2 = pFace2 ? pFace2->GetBrep() : NULL ;

    smgfx_Erase();
    smgfx_SetLook(1,2, 0,0,1) ; if(pBrep1) { pBrep1->Draw(TRUE) ; } sm_GraphicsLoop() ;
    smgfx_SetLook(1,2, 0,1,0) ; if(pBrep2) { pBrep2->Draw(TRUE) ; } sm_GraphicsLoop() ;
    smgfx_SetLook(1,2, 0,1,1) ; DrawUV() ; sm_GraphicsLoop() ;
    smgfx_SetLook(1,2, 1,1,0) ; crOtherCone.DrawUV() ; sm_GraphicsLoop() ;
    smgfx_SetLook(1,2, 0,0,0) ; if(pFace1) { pFace1->Draw(SM_DM_CROSSHATCH,10,10) ; } sm_GraphicsLoop() ;
    smgfx_SetLook(1,2, 0,0,0) ; if(pFace2) { pFace2->Draw(SM_DM_CROSSHATCH,10,10) ; } sm_GraphicsLoop() ;
    smgfx_SetLook(3,4, 1,0,0) ; for(ULONG ii=0;ii<pOpt3DCurves->GetSize();ii++)
                                  { SmCurve *pCurve = pOpt3DCurves->GetAt(ii) ;
                                    pCurve->Draw() ; sm_GraphicsLoop() ;
                                  }
    sm_GraphicsLoop();
  }
#endif

  return SM_SUCCESS;

} // end SmCone::IntersectWithCone

/*******************************************************************//**
PURPOSE: Find intersection curve(s) of this cone with a plane

NOTES: 
***********************************************************************/
SmStatus SmCone::IntersectWithPlane
  (const SmContext    & crContext,                  // in : context for new object construction
   const SmExtent2d   & crUVDomain,                 // in : intersection limit for this surface
   const SmPlane      & crPlane,                    // in : target intersection plane
   const SmExtent2d   & crOtherUVDomain,            // in : intersection limit for target plane
   const SmBoolean      /* bUseSurfaceEdges */[2],  // in : TRUE = find xsect curve start points from boundaryCurve/surface xsects
                                                    //      typically these values are TRUE - its a small savings if you know
                                                    //      the boundaries of one surface don't intersect the other surface                               
   const SmApproxTol3d * pdOptApproxTol3d,          // in : min distance between distinct 3dPoints
   const double       * /* pdOptAngTolRad */,       // in :                                 
   SmBoolean          & rbNeedsMoreIntersections,   // out: TRUE = special case intersection failed-use general intersection 
   SmTArray<SmCurve*> * pOpt3DCurves,               // out: Intersection 3DCurves, NULL to ignore
   SmTArray<SmCurve*> * pOptSurface1UVCurves,       // out: associated UVTrimCurves on this surface, NULL to ignore
   SmTArray<SmCurve*> * pOptSurface2UVCurves,       // out: associated UVTrimCurves on plane, NULL to ignore
   SmTArray<SmTsectCurveType> * pOptCurveTypes,     // out: oneof for each 3DCurve, NULL to ignore
                                                    //      SM_TC_TOUCHING        - single point intersection (surf norms parallel)
                                                    //      SM_TC_CROSSING        - curve intersection (surf norms not parallel)
                                                    //      SM_TC_TANGENT         - curve intersection (surf norms parallel)
                                                    //      SM_TC_COINCIDENT      - curve intersection (surf norms parallel & cross-tangents equal)
                                                    //      SM_TC_NEAR_TANGENT    - curve has small angle of intersection
                                                    //      SM_TC_REGION_BOUNDARY - curve bounds region within which the surfs are coincident
   SmTArray<double>   * pOptDeviations)             // out: associated max 3DCurve to surface distance, NULL to ignore
  const
{
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
      smgfx_SetLook(1,2, 1,1,0) ; crPlane.DrawUV() ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,0,0) ; if(pFace1) { pFace1->Draw(SM_DM_CROSSHATCH,10,10) ; } sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,0,0) ; if(pFace2) { pFace2->Draw(SM_DM_CROSSHATCH,10,10) ; } sm_GraphicsLoop() ;
      sm_GraphicsLoop();
    }
#endif

  // init outputs
  rbNeedsMoreIntersections = FALSE;
  if (pOpt3DCurves)         { pOpt3DCurves->ReSet(); }
  if (pOptSurface1UVCurves) { pOptSurface1UVCurves->ReSet(); }
  if (pOptSurface2UVCurves) { pOptSurface2UVCurves->ReSet(); }
  if (pOptCurveTypes)       { pOptCurveTypes->ReSet(); }
  if (pOptDeviations)       { pOptDeviations->ReSet(); }

  // select tolerance
  double dTol = 0.0;
  if( pdOptApproxTol3d ) dTol = *pdOptApproxTol3d;
  else dTol = SM_EFF_ZERO_SQRT;

  // First do some quick bounding box checking
  SmExtent3d sBBox1, sBBox2;
  SER(CalculateBoundingBox(crUVDomain,&sBBox1));
  SER(crPlane.CalculateBoundingBox(crOtherUVDomain,&sBBox2));

  // no work - disjoint bounding boxes
  sBBox2.ExpandAbsolute(dTol);
  if (sBBox1.AreDisjoint(sBBox2)) 
    {
      return SM_SUCCESS;
    }

  // cone locals
  double           dHeight      = GetHeight();
  double           dStartAng    = GetStartAngleDeg();
  double           dEndAng      = GetEndAngleDeg();
  SmVector3d       sConeAxis    = m_vPosition.GetZAxis();
  SmExtent1d       sSweepIvl    = m_vAnalUVDomain.GetUInterval() ;
//  const SmPoint3d &rCenterBot   = m_vPosition.GetOriginRef();
//  SmPoint3d        sCenterTop   = rCenterBot + dHeight * sConeAxis;
//cbi change:
  SmPoint3d  rCenterBot   = m_vPosition.GetOriginRef() + m_vAnalUVDomain.GetVMin() * sConeAxis;
  SmPoint3d  sCenterTop   = m_vPosition.GetOriginRef() + m_vAnalUVDomain.GetVMax() * sConeAxis;

  // plane locals
  SmVector3d       sPlaneNormal = crPlane.GetPosition().GetZAxis();
  const SmPoint3d &rPlaneOrigin = crPlane.GetPosition().GetOriginRef();

  // locals
  SmBSplineCurve * pCrv1 = NULL;
  SmBSplineCurve * pCrv2 = NULL;
  SmTsectCurveType eCurveType1 = SM_TC_CROSSING ;
  SmTsectCurveType eCurveType2 = SM_TC_CROSSING ;
  SmVector3d sVec;
  double dDistPlaneToCenterBot;
  double dCylAngRad, dCylAngDeg ;
  double dConeAxisPlaneNormalAngle;
  double dAngTol = SM_EFF_ZERO * 100.0 * 360.0 ;
#ifdef SM_DEBUG_CODE
  SmVector3d sTSectBot[2], sTSectTop[2] ;
#endif

  // get coneAxis/PlaneNormal angle from 0 to Pi/2
  SER(sConeAxis.AngleBetween(sPlaneNormal,dConeAxisPlaneNormalAngle));
  if (dConeAxisPlaneNormalAngle > SM_PI/2.0) 
    {
      // Reverse the plane normal for convenience
      dConeAxisPlaneNormalAngle = SM_PI - dConeAxisPlaneNormalAngle;
      sPlaneNormal              = -sPlaneNormal;
    }

  // get ConeCenterBottom/Plane distance
  sVec                  = rCenterBot - rPlaneOrigin;
  dDistPlaneToCenterBot = sVec.Dot(sPlaneNormal);

  double dBotRadius = GetBotRadius();

  // If plane is parallel to cone axis: possibly output lines trimmed to cone and plane.
  //    If it's a cylinder, output two parallel lines
  //    else if cone's apex is in the plane, output two lines at angle.
  //    We won't do the hyperbola case here, let the general intersector do that.

  if (SM_PI/2.0 - dConeAxisPlaneNormalAngle < dAngTol) 
    {
      // locals
      SmPoint3d   sPt1, sPt2;
      double      dLineScale;
      SmExtent1d  sExt = GetSTEPLinearParamExtent();
      SmExtent1d  sLineExt(0.0,sExt.GetLength());

      // get unitVector in Plane and perp ToConeAxis
      sVec = sConeAxis * sPlaneNormal; 

      // when cone is a cylinder - output line intersections
      if ( IsCylinder() )
        {
          dLineScale = dHeight/sExt.GetLength();
          
          // no work - plane is outside of cylinder
          if (smos_Fabs(dDistPlaneToCenterBot) > dBotRadius+dTol) 
            {
              return SM_SUCCESS;  // Intersections do not exist
            }

          // when plane is tangent to cylinder wall
          //  or close enough that two solutions are too close together
          if(   smos_Fabs(dDistPlaneToCenterBot) >= dBotRadius
             || (   smos_Fabs(dDistPlaneToCenterBot) > dBotRadius-dTol
                 && (dBotRadius*dBotRadius - dDistPlaneToCenterBot*dDistPlaneToCenterBot) < dTol*dTol))
            {
              // should have one intersection line, if it exists

              // get vector from xSectLine to cone Axis (+/- SurfaceNormal)
              SmVector3d sBotCenterToPlane =   dDistPlaneToCenterBot > 0.0
                                             ? -sPlaneNormal
                                             :  sPlaneNormal;

              // get cylindrcial coordinate [0,360] of vectorToXSectCurve 
              SER(sConeAxis.CCWAngleBetween(m_vPosition.GetXAxisRef(),
                                            sBotCenterToPlane,
                                            dCylAngRad));

              // map to periodic domain
              sSweepIvl.ContainsPeriodicValue(dCylAngRad * 180.0 / SM_PI, 360.0, &dCylAngDeg) ;

              // snap to boundary endpoints
              if     (SM_ARE_SAME_TO_TOL(dCylAngDeg,dStartAng,dAngTol)) dCylAngDeg = dStartAng ;
              else if(SM_ARE_SAME_TO_TOL(dCylAngDeg,dEndAng,dAngTol))   dCylAngDeg = dEndAng ;

              // when Line is inside cone's analytic boundary
              if (   dCylAngDeg >= dStartAng 
                  && dCylAngDeg <= dEndAng) 
                {
                  // build the tangent curve - trimmed to cylinder and plane
                  SmBoolean bFoundInterval ;
                  sPt1 = rCenterBot + dBotRadius * sBotCenterToPlane;
                  SmPoint3d sPt1End = sPt1 + sLineExt.GetMax() * dLineScale * sConeAxis ;
                  pCrv1 = crPlane.MakeNewPlaneLine(crContext, crOtherUVDomain, sPt1, sPt1End, dTol, bFoundInterval) ;

                  eCurveType1 =   (bFoundInterval) 
                                ? SM_TC_TANGENT
                                : SM_TC_TOUCHING ;
                } // end Line in Cone check
            } // end plane is tangent to cylinder wall branch

          // when plane is inside cylinder radius - output 2 lines
          else 
            {
#ifdef SM_DEBUG_CODE
              // Currently, just check for odd cases.

              ULONG lNumTSectBot, lNumTSectTop;
              // GWC TODO
              // Cone bottom circle/Plane intersections
              smgu_CirclePlaneIntersect
                (dTol,          // in : tol                                              
                 dBotRadius,    // in : circle radius                                 
                 rCenterBot,    // in : circle center point                           
                 sConeAxis,     // in : Circle Plane's Normal vector [unitized]       
                 rPlaneOrigin,  // in : plane point                                   
                 sPlaneNormal,  // in : Plane Normal Vector [Unitized]                
                 lNumTSectBot,  // out: 0 - no intersections                          
                                //      1 - grazing intersection                      
                                //      2 - standard two point intersection           
                                //      3 - indicates plane of circle                 
                                //          is coincident with the intersection       
                                //          plane.  No points returned in aTsectPoints
                 sTSectBot) ;   // out:       
                 
              // Cone top circle/Plane intersections
              smgu_CirclePlaneIntersect
                (dTol,          // in : tol                                              
                 dBotRadius,    // in : circle radius                                 
                 sCenterTop,    // in : circle center point                           
                 sConeAxis,     // in : Circle Plane's Normal vector [unitized]       
                 rPlaneOrigin,  // in : plane point                                   
                 sPlaneNormal,  // in : Plane Normal Vector [Unitized]                
                 lNumTSectTop,  // out: 0 - no intersections                          
                                //      1 - grazing intersection                      
                                //      2 - standard two point intersection           
                                //      3 - indicates plane of circle                 
                                //          is coincident with the intersection       
                                //          plane.  No points returned in aTsectPoints
                 sTSectTop) ;   // out:                                               
              
              // watch out for tolerance caused changes in topology
              // This would be due to smgu_CirclePlaneIntersect checking tolerance
              // in a different manner than this routine did.  [B98]
              if(   lNumTSectBot != 2                                           
                 || lNumTSectTop != 2)
                {
                  // odd case - maybe need to pass this to the general intersector.
                  // For now, just output a message
                  WARN(_T("SmCone::IntersectWithPlane: Found an unsupported case.  Needs review")) ;
                }
#endif // SM_DEBUG_CODE

              // get point on plane closest to CenterBot
              sPt1 = rCenterBot - dDistPlaneToCenterBot * sPlaneNormal;

              // get 1st of 2 points on plane and cylinder closest to CenterBot
              double dVal = smos_Sqrt(  dBotRadius * dBotRadius
                                      - dDistPlaneToCenterBot * dDistPlaneToCenterBot);
              sPt2 = sPt1 + dVal * sVec;

              // get cylindrical coordinate of point [0 to 360]
              SER(sConeAxis.CCWAngleBetween(m_vPosition.GetXAxisRef(),
                                            sPt2-rCenterBot,
                                            dCylAngRad));
              // map to periodic domain
              sSweepIvl.ContainsPeriodicValue(dCylAngRad * 180.0 / SM_PI, 360.0, &dCylAngDeg) ;

              // snap to boundary endpoints
              if     (SM_ARE_SAME_TO_TOL(dCylAngDeg,dStartAng,dAngTol)) dCylAngDeg = dStartAng ;
              else if(SM_ARE_SAME_TO_TOL(dCylAngDeg,dEndAng,dAngTol))   dCylAngDeg = dEndAng ;

              // when Line is inside cone's analytic boundary
              if (   dCylAngDeg >= dStartAng 
                  && dCylAngDeg <= dEndAng)
                {
                  // make 1st crossing line - trimmed to plane surface
                  SmBoolean bFoundInterval ;
                  SmPoint3d sPt2End = sPt2 + sLineExt.GetMax() * dLineScale * sConeAxis ;
                  pCrv1       = crPlane.MakeNewPlaneLine(crContext, crOtherUVDomain, sPt2, sPt2End, dTol, bFoundInterval) ;
                  eCurveType1 =   (bFoundInterval) 
                                ? SM_TC_CROSSING
                                : SM_TC_TOUCHING ;
                }

              // get 2nd of 2 points on plane and cylinder closest to CenterBot
              sPt2 = sPt1 - dVal * sVec;
              
              // get cylindrical coordinate of point [0 to 360]
              SER(sConeAxis.CCWAngleBetween(m_vPosition.GetXAxisRef(),
                                            sPt2-rCenterBot,
                                            dCylAngRad));
              // map to periodic domain
              sSweepIvl.ContainsPeriodicValue(dCylAngRad * 180.0 / SM_PI, 360.0, &dCylAngDeg) ;

              // snap to boundary endpoints
              if     (SM_ARE_SAME_TO_TOL(dCylAngDeg,dStartAng,dAngTol)) dCylAngDeg = dStartAng ;
              else if(SM_ARE_SAME_TO_TOL(dCylAngDeg,dEndAng,dAngTol))   dCylAngDeg = dEndAng ;


              // when point is in cylinder domain
              if (   dCylAngDeg >= dStartAng 
                  && dCylAngDeg <= dEndAng)
                {
                  // make 2nd crossing line - trimmed to plane surface
                  SmBoolean bFoundInterval ;
                  SmPoint3d sPt2End = sPt2 + sLineExt.GetMax() * dLineScale * sConeAxis ;
                  pCrv2 = crPlane.MakeNewPlaneLine(crContext, crOtherUVDomain, sPt2, sPt2End, dTol, bFoundInterval) ;
                  eCurveType2 =   (bFoundInterval) 
                                ? SM_TC_CROSSING
                                : SM_TC_TOUCHING ;
                }
            } // end plane makes 2 intersection curves with cylinder
        }  // end IsCylinder branch
      else 
        {  // IsCone (intersected by a plane parallel to cone axis) branch
          SmVector3d sLineVec;

          // pass nonlinear intersections to general surf/surf solver
          if (smos_Fabs(dDistPlaneToCenterBot) > SM_EFF_ZERO) 
            {
              // Unable to find analytic intersections
              rbNeedsMoreIntersections = TRUE;
              return SM_SUCCESS;
            }
          
          // get 1st line point in cone bottom plane
          sPt1 = rCenterBot - dBotRadius * sVec;

          // get cylindrical coordinate for 1st pt
          SER(sConeAxis.CCWAngleBetween(m_vPosition.GetXAxisRef(),-sVec,dCylAngRad));

          // map to periodic domain
          sSweepIvl.ContainsPeriodicValue(dCylAngRad * 180.0 / SM_PI, 360.0, &dCylAngDeg) ;

          // snap to boundary endpoints
          if     (SM_ARE_SAME_TO_TOL(dCylAngDeg,dStartAng,dAngTol)) dCylAngDeg = dStartAng ;
          else if(SM_ARE_SAME_TO_TOL(dCylAngDeg,dEndAng,dAngTol))   dCylAngDeg = dEndAng ;

          // when point is in cone natural boundary
          if (   dCylAngDeg >= dStartAng 
              && dCylAngDeg <= dEndAng) 
            {
              // get 1st line point in cone top plane
              sPt2       = sCenterTop - GetTopRadius() * sVec;

              // make 1st crossing line - trimmed to plane surface
              SmBoolean bFoundInterval ;
              pCrv1       = crPlane.MakeNewPlaneLine(crContext, crOtherUVDomain, sPt1, sPt2, dTol, bFoundInterval) ;
              eCurveType1 =   (bFoundInterval) 
                            ? SM_TC_CROSSING
                            : SM_TC_TOUCHING ;
            }

          // get 2nd line point in cone bottom plane
          sPt1 = rCenterBot + dBotRadius * sVec;

          // get points cylindrical coordinate
          SER(sConeAxis.CCWAngleBetween(m_vPosition.GetXAxisRef(),sVec,dCylAngRad));
          // map to periodic domain
          sSweepIvl.ContainsPeriodicValue(dCylAngRad * 180.0 / SM_PI, 360.0, &dCylAngDeg) ;

          // snap to boundary endpoints
          if     (SM_ARE_SAME_TO_TOL(dCylAngDeg,dStartAng,dAngTol)) dCylAngDeg = dStartAng ;
          else if(SM_ARE_SAME_TO_TOL(dCylAngDeg,dEndAng,dAngTol))   dCylAngDeg = dEndAng ;

          // when Line is inside cone's analytic boundary
          if (   dCylAngDeg >= dStartAng 
              && dCylAngDeg <= dEndAng)
            {
              // get 2nd line point in cone top plane
              sPt2 = sCenterTop + GetTopRadius() * sVec;

              // make 1st crossing line - trimmed to plane surface
              SmBoolean bFoundInterval ;
              pCrv2       = crPlane.MakeNewPlaneLine(crContext, crOtherUVDomain, sPt1, sPt2, dTol, bFoundInterval) ;
              eCurveType2 =   (bFoundInterval) 
                            ? SM_TC_CROSSING
                            : SM_TC_TOUCHING ;
            }
        } // end IsCone (not a cylinder) branch
    } // end plane parallel to cone axis branch

  // when plane is perpendicular to cone - output circular arcs trimmed to cone and plane
  else if (dConeAxisPlaneNormalAngle < dAngTol) 
    {
      // Now plane is cutting through the cone 'horizontally',
      // The intersection curve (if it exists) is an Arc
      SmTArray<SmBSplineCurve*> sCrvs ;

      // intersect coneAxis with plane
      SmPoint3d sBasePt( m_vPosition.GetOriginRef() );
      double dVParam;
      SmStatus eStat = smgu_LinePlaneIntersect(sBasePt, sConeAxis, rPlaneOrigin, sPlaneNormal, dVParam);
      if ( eStat != SM_SUCCESS )
        {
          rbNeedsMoreIntersections = TRUE; // Try the general intersector.
          return SM_SUCCESS;
        }
        
      // If param is out of range, there won't be any intersections,
      // and the general intersector won't find anything,
      // so leave rbNeedsMoreIntersection False and return.
      SmExtent1d sVDomain( m_vAnalUVDomain.GetVInterval() );
      if ( ! sVDomain.ContainsValue( dVParam, dTol ) )    //cbiTol: 3d tol.
        {
          return(SM_SUCCESS) ;
        }

      // Snap near endPoint dVParam values to coneAxis endPoints.
      // Snap only if a tolerance was passed in; not to our arbitrary value.
      if ( pdOptApproxTol3d != NULL )
        {
          double dT0 = sVDomain.GetMin();
          double dT1 = sVDomain.GetMax();
          if ( dVParam < dT0 + *pdOptApproxTol3d ) { dVParam = dT0; }
          if ( dVParam > dT1 - *pdOptApproxTol3d ) { dVParam = dT1; }
        }

      SmPoint3d sCenterPoint = sBasePt + dVParam * sConeAxis;  // [B188]

      // make intersection curve(s) trimmed to cone and plane limits
      // If radius is negative here, just use its absolute value.
      // Negative radius is not a good thing, it means we're on the
      // wrong side of the apex, and would probably cause problems
      // in topology operations, but here we can live with it.
      double dRad = smos_Fabs( GetRadius(dVParam) );
      crPlane.MakeNewPlaneEllipse(crContext,
                                  crOtherUVDomain,
                                  sCenterPoint,
                                  dRad, dRad,
                                  m_vPosition.GetXAxisRef(),
                                  m_vPosition.GetYAxisRef(),
                                  SmExtent1d(GetStartAngleDeg(),  // trimmed to cone limits
                                             GetEndAngleDeg()),
                                  dTol,
                                  sCrvs) ;
      
      // Set outputs for each trimmed intersection curve result.
      ULONG ii;
      for ( ii=0; ii<sCrvs.GetSize(); ii++ )
        {
          pCrv1 = sCrvs[ii] ;

          SmTsectCurveType eCurveType =   pCrv1->IsDegenerate()
                                        ? SM_TC_TOUCHING
                                        : SM_TC_CROSSING ;
          // set outputs
          if (pOpt3DCurves)         { pOpt3DCurves->Add(pCrv1); }
          if (pOptCurveTypes)       { pOptCurveTypes->Add(eCurveType); }
          if (pOptSurface1UVCurves) { pOptSurface1UVCurves->Add(NULL); }
          if (pOptSurface2UVCurves) { pOptSurface2UVCurves->Add(NULL); }
          if (pOptDeviations)       { pOptDeviations->Add(0.0); }
          
        } // end iter every circle trim curve
      
      // all done
      return(SM_SUCCESS) ;
    } // end plane perpendicular to cone axis branch

  // when plane is neither parallel nor perpendicular to cone axis
  else // for now - process the ellipse intersections trimmed to cone and plane
    {
      // get Cone SemiAngle and ConeAxis/Plane Angles in radians (0 to Pi/2) 
      double dAngleCone  = smos_Fabs(SM_DEG2RAD(m_dSemiAngleDeg));
      double dAnglePlane = SM_PI/2.0-smos_Fabs(dConeAxisPlaneNormalAngle) ;

      // Pass nonElliptical solutions to general surf/surf intersector - for now
      // when plane/ConeAxis angle is less than cone SemiAngle on cones (parabolas not cylinders)
      if (   !IsCylinder() 
          && dAnglePlane < dAngleCone + dAngTol) 
        {
          // does not have an elliptical intersection
          //  is a line dAnglePlane == dAngleCone and plane/Axis intersection point = cone apex
          //  or is a parabola
          rbNeedsMoreIntersections = TRUE;
          return SM_SUCCESS;
        }

      // Find plane/Cone elliptical intersection
      //  1. find ellipse maxima points on either side of the cone
      //  2. Ellipse Principal axis and majRadius = 1/2(Maxima2 - Maxima1)
      //  3. Ellipse CenterPoint = MidPoint(maxima points)
      //  4. Minor Axis = CrossProduct(MajAxis, PlaneNormal)
      //  5. Minor Radius = length from EllipseCenterPoint to ConeSurface in minorAxis direction
      //  6. Ellipse/ConeBoundary intersections 

      // find ellipse/cone boundary intersections
      // find ellipse/plane boundary intersections
      // build ellipse trim segments

      // Locals for ellipse intersection curve 
      SmPoint3d   sCenter;
      SmVector3d  sLineVec;
      double      dMajRad, dMinRad;
      SmPoint3d   sPt1, sPt2;
      double      dTParam1, dTParam2;

      // Project (and negate) planeNormal into plane Perp to coneAxis
      sVec = sConeAxis * (sConeAxis * sPlaneNormal) ;
      sVec.Unitize();

      // Find ConeLine which is ellipse Center Line projected to one side of the cone
      // sPt1 = ConeBottom/Cone/PlaneNormal-ConeAxis-Plane xSect Point
      // sPt2 = ConeTop/Cone/PlaneNormal-ConeAxis-Plane xSect Point
      // LineVec = Ellipse Center Line Projected to Cone Surface
      sPt1 = rCenterBot - dBotRadius     * sVec;
      sPt2 = sCenterTop - GetTopRadius() * sVec;
      sLineVec = sPt2 - sPt1;
      double sLineLength1 = sLineVec.Length() ;

      // Intersect this Line with Plane to find 1st ellipse extremum point
      // Line/Plane xSect gives Point at one end of ellipse
      if (smgu_LinePlaneIntersect(sPt1, sLineVec,
                                  rPlaneOrigin, sPlaneNormal, dTParam1) != SM_SUCCESS) 
        {
          // Line is within tolerance of being parallel with the plane.
          rbNeedsMoreIntersections = TRUE;
          return SM_SUCCESS;
        }

      // get 1st ellipse extremum point
      SmPoint3d sEnd1 = sPt1 + dTParam1*sLineVec; // sEnd1 = Ellipse extrema point 1
      
      // Find ConeLine which is ellipse Center Line projected to other side of the cone
      // sPt1 = OtherConeSide ConeBottom/Cone/PlaneNormal-ConeAxis-Plane xSect Point
      // sPt2 = OtherConeSide ConeTop/Cone/PlaneNormal-ConeAxis-Plane xSect Point   
      // LineVec = Ellipse Center Line Projected to OtherSide Cone Surface
      sPt1 = rCenterBot + dBotRadius     * sVec;
      sPt2 = sCenterTop + GetTopRadius() * sVec;
      sLineVec = sPt2 - sPt1;
      double sLineLength2 = sLineVec.Length() ;
      if (smgu_LinePlaneIntersect(sPt1, sLineVec,
                                  rPlaneOrigin, sPlaneNormal, dTParam2) != SM_SUCCESS) 
        {
          // Line is within tolerance of being parallel with the plane.
          rbNeedsMoreIntersections = TRUE;
          return SM_SUCCESS;
        }

      // when both extrema are outside of the bot/top limits of the cone
//      double dScaledZero1 = (pdOptApproxTol3d ? *pdOptApproxTol3d : SM_EFF_ZERO_SQRT)  / sLineLength1 ;
//      double dScaledZero2 = (pdOptApproxTol3d ? *pdOptApproxTol3d : SM_EFF_ZERO_SQRT)  / sLineLength2 ;
      if(   (dTParam1 < 0.0 - sLineLength1 && dTParam2 < 0.0 - sLineLength2)
         || (dTParam1 > 1.0 + sLineLength1 && dTParam2 > 1.0 + sLineLength2))
        {
          // there is no intersection it falls completely outside cone domain
          rbNeedsMoreIntersections = FALSE;
          return SM_SUCCESS;
        }

      // get 2nd ellipse extrema point
      SmPoint3d  sEnd2  = sPt1 + dTParam2*sLineVec; // sEnd2 = Ellipse extrema point 2
      SmVector3d sXAxis = (sEnd2 - sEnd1)/2.0;      // sXAxis = 1/2 axis of ellipse
      sCenter = sEnd1 + sXAxis;                     // sCenter = Ellipse Center (on cone axis when cone is a cylinder)
      dMajRad = sXAxis.Length();                    // dMajRad = Ellipse Major Radius

      // Radius is zero when plane intersects cone vertex
      if(SM_IS_ZERO(dMajRad))
        {
          SmBSplineCurve *pCrv = NULL ;
          SmBSplineCurve::CreateDegenerateCurve(crContext, 3, sEnd1, pCrv) ;
          NER(pCrv) ;

          // set outputs
          if (pOpt3DCurves)         { pOpt3DCurves->Add(pCrv); }
          if (pOptCurveTypes)       { pOptCurveTypes->Add(SM_TC_TOUCHING); }
          if (pOptSurface1UVCurves) { pOptSurface1UVCurves->Add(NULL); }
          if (pOptSurface2UVCurves) { pOptSurface2UVCurves->Add(NULL); }
          if (pOptDeviations)       { pOptDeviations->Add(0.0); }
          
          // all done
          return(SM_SUCCESS) ;

        } // end degenerate curve check

      sXAxis.Unitize();

      // get minor axis
      SmVector3d sYAxis = sPlaneNormal * sXAxis; // sYAxis = ellipse minor axis
      sYAxis.Unitize() ;

      // get minor radius
      if (IsCylinder()) // when working on a cylinder
        {
          // when cone is a cylinder, ellipse MinRad = Cylinder radius
          dMinRad = dBotRadius;  
        }
      else // when working on a cone
        {
          sVec = sCenter - rCenterBot;   // vec from coneCenterBottom to ellipseCenter (not parallel to cone axis)
          double dT = sVec.Dot(sConeAxis); // height of Ellipse Center measured from ConeBottom
          double dRad = (GetTopRadius()-dBotRadius)*dT/dHeight // Cone Radius at centerPoint height
                       + dBotRadius;
          
          // MinRad is found in cone circular cross section at ellipseCenter height
          //   it is the length of the vector from the EllipseCenter to the ConeSurface
          //   in the direction of the minor axis.  This length is found by looking
          //   at a couple of simple right triangles as:
          //   dRad**2 = MinRad**2 + d**2
          //     where d      = distance from coneCenter at EllipseCenterPoint height to Ellipse Center
          //           dRad   = radius of cone at EllipseCenterPoint height
          //           MinRad = length of vector from EllipseCenterPoint to ConeSurface in minor Axis direction
          //   length d can be found from the triangle between the 3 points; 
          //          [BottomCenter, ConeCenter at EllipseCenterPoint height, and EllipseCenter Point]
          //   sVec**2 = dT**2 + d**2
          //   d**2 = sVec**2 - dT**2
          //   MinRad = sqrt(dRad**2 - (sVec**2 - dT**2))
          dMinRad = smos_Sqrt(  dRad*dRad 
                              - (sVec.LengthSquared() - dT*dT));
        } // end getting minor radius

      // arrive here when 3d ellipse is defined by
      //  [sCenter, sXAxis, sYAxis, sPlaneNormal, dMajRad, dMinRad]
      //    where sXAxis is pointing to extrema Point2

      // trim full ellipse to cone boundaries
      SmTArray<SmExtent1d> sEllipseIvls ;
      sm_GetEllipseTrimIntervals
        (crContext,                // in : for temp object construction
         sCenter,                  // in : origin            of ellipse
         sXAxis,                   // in : xAxis             of ellipse
         sYAxis,                   // in : yAxis             of ellipse
         dMajRad,                  // in : x radius          of ellipse
         dMinRad,                  // in : y radius          of ellipse
         0.0,                      // in : start angle (deg) of ellipse
         360.0,                    // in : end angle (deg)   of ellipse
         *this,                    // in : Cone with trim boundaries
         GetNaturalUVDomain(),     // in : nurb trim boundaries for cone
         dTol,                     // in : approximation tolerance
         sEllipseIvls) ;           // out: array of ellipse intervals inside trimmed cone

      // for every ellipse interval trimmed to cone
      SmTArray<SmBSplineCurve *> s3dTrimCurves ;
      for(ULONG ii=0;ii<sEllipseIvls.GetSize();ii++)
        {
          SmExtent1d sIvl = sEllipseIvls[ii] ;

          // make intersection curve(s) trimmed to cone and plane limits
          crPlane.MakeNewPlaneEllipse(crContext,
                                      crOtherUVDomain,
                                      sCenter,
                                      dMajRad, dMinRad,
                                      sXAxis,
                                      sYAxis,
                                      sIvl,
                                      dTol,
                                      s3dTrimCurves) ;

          // set outputs
          for(ULONG jj=0;jj<s3dTrimCurves.GetSize();jj++)
            {
              SmCurve  *pCrv      = s3dTrimCurves[jj] ;
              SmBoolean bTouching = pCrv->IsDegenerate() ;

              if (pOpt3DCurves)         { pOpt3DCurves->Add(pCrv); }
              if (pOptCurveTypes)       { if(bTouching) { pOptCurveTypes->Add(SM_TC_TOUCHING); }
                                          else          { pOptCurveTypes->Add(SM_TC_CROSSING); }
                                        }
              if (pOptSurface1UVCurves) { pOptSurface1UVCurves->Add(NULL); }
              if (pOptSurface2UVCurves) { pOptSurface2UVCurves->Add(NULL); }
              if (pOptDeviations)       { pOptDeviations->Add(0.0); }
            }
        } // end iter every ellipse intersection arc trimmed to both cone and plane
      
#ifdef SM_DEBUG_CODE
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
          smgfx_SetLook(3,4, 1,0,0) ; for(ULONG ii=0;ii<pOpt3DCurves->GetSize();ii++)
                                        { SmCurve *pCurve = pOpt3DCurves->GetAt(ii) ;
                                          pCurve->Draw() ; sm_GraphicsLoop() ;
                                        }
          sm_GraphicsLoop();
        }
#endif

      // all done
      return(SM_SUCCESS) ;
    
    } // end ellipse intersections

// old code
//            // get ellipse intervals trimmed to cone trim boundaries
//            SmTArray<SmExtent1d> sIvls ;
//            sm_GetEllipseTrimIntervals
//              (crContext,                      // in : for temp object construction
//               sCenter,                        // in : origin            of ellipse
//               sXAxis,                         // in : xAxis             of ellipse
//               sYAxis,                         // in : yAxis             of ellipse
//               dMajRad,                        // in : x radius          of ellipse
//               dMinRad,                        // in : y radius          of ellipse
//               0.0,                            // in : start angle (deg) of ellipse
//               360.0,                          // in : end angle (deg)   of ellipse
//               *this,                          // in : Cone with trim boundaries
//               crUVDomain,                     // in : trim boundaries for cone
//               dTol,                           // in : approximation tolerance
//               sIvls) ;                        // out: array of ellipse intervals inside trimmed cone
//      
//            // for every ellipse interval trimmed to cone
//            SmTArray<SmBSplineCurve *> s3dTrimCurves ;
//            for(ii=0;ii<sIvls.GetSize();ii++)
//              {
//                SmExtent1d sIvl = sIvls[ii] ;
//      
//                // make intersection curve(s) trimmed to cone and plane limits
//                crPlane.MakeNewPlaneEllipse(crContext,
//                                            crOtherUVDomain,
//                                            sCenter,
//                                            dMajRad, dMinRad,
//                                            sXAxis,
//                                            sYAxis,
//                                            sIvl,
//                                            dTol,
//                                            s3dTrimCurves) ;
//                //      
//                //      // build a temporary ellipse - for trimming purposes
//                //      SmEllipse sEllipse(sCenter, sXAxis, sYAxis, sIvl, dMajRad, dMinRad, 3, &crContext) ;
//                //      
//                //      // trim ellipse arc (already trimmed to cone) to plane
//                //      crPlane.TrimCurveToPlaneDomain(crOtherUVDomain, sEllipse, dTol, s3dTrimCurves) ;
//      
//                // set outputs
//                for(jj=0;jj<s3dTrimCurves.GetSize();jj++)
//                  {
//                    SmCurve  *pCrv      = s3dTrimCurves[jj] ;
//                    SmBoolean bTouching = pCrv->IsDegenerate() ;
//      
//                    if (pOpt3DCurves)         { pOpt3DCurves->Add(pCrv); }
//                    if (pOptCurveTypes)       { if(bTouching) { pOptCurveTypes->Add(SM_TC_TOUCHING); }
//                                                else          { pOptCurveTypes->Add(SM_TC_CROSSING); }
//                                              }
//                    if (pOptSurface1UVCurves) { pOptSurface1UVCurves->Add(NULL); }
//                    if (pOptSurface2UVCurves) { pOptSurface2UVCurves->Add(NULL); }
//                    if (pOptDeviations)       { pOptDeviations->Add(0.0); }
//                  }
//              } // end iter every ellipse intersection arc trimmed to both cone and plane
//      
//            // all done
//            rbNeedsMoreIntersections = FALSE;
//            return SM_SUCCESS;
//      
//            // end plane at an angle to cone axis branch

                     
/* GWC obsolete trim to cone method  // else {


      // arrive here when 3d ellipse is defined by
      //  [sCenter, sXAxis, sYAxis, sPlaneNormal, dMajRad, dMinRad]
      //    where sXAxis is pointing to extrema Point2

      // define a scratch ellipse to help compute trim points
      SmEllipse sEllipse(sCenter, sXAxis, sYAxis, SmExtent1d(0.0, 360.0),
                         dMajRad, dMinRad, 3, GetContext());

      // find ellipse/cone trim boundaries
      SmTArray<double>    dXSectAngle(8,NULL,6) ;     // coneBoundary xSect Points
      SmTArray<SmBoolean> bXSectEntering(8,NULL,6) ;  // TRUE = increasing ellipse param values are inside cone area

      // set 1st dXSectAngle entry for ellipse start
      dXSectAngle[0] = 0.0 ;
      ULONG     lCnt = 1 ;

      // get ellipse/TopCircle and ellipse/BotCircle intersections
      ULONG lNumTsect1, lNumTsect2 ;
      SmPoint3d sTsectPoints[4] ;
      smgu_CirclePlaneIntersect(dTol, GetTopRadius(), sCenterTop, sConeAxis,
                                rPlaneOrigin, sPlaneNormal, lNumTsect1, sTsectPoints) ;
      smgu_CirclePlaneIntersect(dTol, GetBotRadius(), rCenterBot, sConeAxis,
                                rPlaneOrigin, sPlaneNormal, lNumTsect2, sTsectPoints+2) ;
      // lNumTSect == 0 is a miss - nothing to do
      // lNumTSect == 1 is a tangent point - ignored for now 
      // lNumTSect == 2 is a trimmed section
      // lNumTSect == 3 is a coincident circle/plane
      SM_ASSERT(lNumTsect1 != 3 && lNumTsect2 != 3) ;

      // get ellipse angle in degrees for each XSectPoint 
      if(lNumTsect1 == 2) { sEllipse.STEPInversion(sTsectPoints[0], dXSectAngle[lCnt  ]) ;
                            sEllipse.STEPInversion(sTsectPoints[1], dXSectAngle[lCnt+1]) ;
                            lCnt += 2 ;
                          }
      if(lNumTsect2 == 2) { sEllipse.STEPInversion(sTsectPoints[2], dXSectAngle[lCnt  ]) ;
                            sEllipse.STEPInversion(sTsectPoints[3], dXSectAngle[lCnt+1]) ;
                            lCnt += 2 ;
                          }

      // order the XSectPoints by angle
      smgu_ShellSort_double_array(dXSectAngle.GetDataArray(), lCnt);

      // classify extrema Point 2 as in/out of cone boundaries
      SmBoolean bStartOut = (dTParam2 < 0.0 || dTParam2 > 1.0) ? TRUE : FALSE ;

      // classify the entering/exit behavior of each ellipse/TopBotCircle XSectPoint
      ULONG ii ;
      for(ii=0;ii<lCnt;ii++)
        {
          bXSectEntering[ii] = (ii % 2) ? bStartOut : !bStartOut ;
        }

      // set the last dXSectAngle entry for ellipse end
      dXSectAngle[lCnt]    = 360.0 ;
      bXSectEntering[lCnt] = 0 ;
      lCnt++ ;

      // Trim dXSectAngle array to Ellipse/ConeSeam intersections

      // remember if cone is closed
      SmBoolean bIsClosed = SM_ARE_SAME(  360.0,
                                          m_vAnalUVDomain.GetMax().x 
                                        - m_vAnalUVDomain.GetMin().x) ;

      // when cone is not closed in cylindrical direction 
      //   trim xSect to Start/End boundaries
      // else split XSectCurve at seam when needed

      // Intersect Start and End ConeLines with EllipsePlane
      SmPoint3d sStart1, sStart2 ;
      double dStartParam, dEndParam ;
      double dStartAngle, dEndAngle ;
      EvaluateSTEPPoint(m_vAnalUVDomain.Evaluate(0.0,0.0), sStart1) ;
      EvaluateSTEPPoint(m_vAnalUVDomain.Evaluate(0.0,1.0), sStart2) ;
      smgu_LinePlaneIntersect(sStart1, sStart2-sStart1,
                              rPlaneOrigin, sPlaneNormal,
                              dStartParam) ;  
      EvaluateSTEPPoint(m_vAnalUVDomain.Evaluate(1.0,0.0), sEnd1) ;
      EvaluateSTEPPoint(m_vAnalUVDomain.Evaluate(1.0,1.0), sEnd2) ;
      smgu_LinePlaneIntersect(sEnd1, sEnd2-sEnd1,
                              rPlaneOrigin, sPlaneNormal,
                              dEndParam) ;

      // get start and end line/Plane intersection points
      sStart1 = (1 - dStartParam) * sStart1 + dStartParam * sStart2 ;                               
      sEnd1   = (1 - dEndParam)   * sEnd1   + dEndParam   * sEnd2 ;                               

      // get ellipse angles for start and end intersection points (0 to 2Pi)
      sPlaneNormal.CCWAngleBetween(sXAxis, sStart1 - sCenter, dStartAngle) ;
      sPlaneNormal.CCWAngleBetween(sXAxis, sEnd1   - sCenter, dEndAngle) ;
      dStartAngle =   (dStartAngle < 0)
                    ? (dStartAngle + 2*SM_PI) * 180.0/SM_PI
                    : (dStartAngle          ) * 180.0/SM_PI ;
      dEndAngle =     (dEndAngle < 0)
                    ? (dEndAngle + 2*SM_PI) * 180.0/SM_PI
                    : (dEndAngle          ) * 180.0/SM_PI ;

      // check closed status
      SM_ASSERT(  !bIsClosed
                || SM_ARE_SAME(dStartAngle, dEndAngle)) ;

      // insert start angle into XSectAngle Array
      ULONG i1, lStartIndex = 99, lEndIndex = 99 ;
      for(ii=0,i1=1;i1<lCnt;ii++,i1++)
        { 
          if(SM_ARE_SAME(dStartAngle, dXSectAngle[ii]))
            { lStartIndex = ii ;
              if(bIsClosed) 
                { // add duplicate entry into arrays to mark beginning and end when needed
                  // there are already 2 entries for the angles 0/360
                  if(ii == 0)
                    { lEndIndex = lCnt - 1 ; }
                  else
                    { dXSectAngle.InsertAt(ii, dStartAngle) ;
                      bXSectEntering.InsertAt(ii,FALSE) ;
                      lCnt++ ;
                      lStartIndex = i1 ;
                      lEndIndex   = ii ;
                    }
                }
              break ;
            }
          else if(dStartAngle < dXSectAngle[i1])
            { dXSectAngle.InsertAt(i1, dStartAngle) ;
              bXSectEntering.InsertAt(i1,bXSectEntering[ii]) ;
              lCnt++ ;
              lStartIndex = i1 ;
              if(bIsClosed) { dXSectAngle.InsertAt(i1, dStartAngle) ;
                              bXSectEntering.InsertAt(i1, FALSE) ;
                              lEndIndex = lStartIndex ;
                              lStartIndex++ ;
                              lCnt++ ;
                            }
              break ;
            }
        }
      SM_ASSERT(lStartIndex != 99) ;        

      // insert end angle into XSectAngle Array
      if(!bIsClosed)
        {
          for(ii=0,i1=1;i1<lCnt;ii++,i1++)
            { 
              if(SM_ARE_SAME(dEndAngle, dXSectAngle[ii]))
                { lEndIndex = ii ;
                  bXSectEntering[ii] = FALSE ;
                  break ;
                }
              else if(dEndAngle < dXSectAngle[i1])
                { dXSectAngle.InsertAt(i1, dEndAngle) ;
                  bXSectEntering.InsertAt(i1, FALSE) ;
                  lEndIndex = i1 ;
                  lCnt++ ;
                  if(lEndIndex <= lStartIndex) lStartIndex++ ;
                }
            }
          SM_ASSERT(lStartIndex != lEndIndex) ;
        
        } // end need to insert end angles for notClosed cone cases
      SM_ASSERT(lEndIndex != 99) ;
      SM_ASSERT(lCnt <= dXSectAngle.GetSize()) ;
      SM_ASSERT(lCnt <= bXSectEntering.GetSize()) ;

      // arrive here when dXSectAngle and bXSectEntering arrays are
      // set so that every pair of entering-exit entries from
      // startIndex to EndIndex need to be turned into an intersection arc.
      // startIndex does not have to be less than endIndex so care
      // is taken to walk around the periodic list.

      // locals for building ellipses that start at the other extrema
      SmExtent1d sIvl ;
      SmVector3d sNegX = -sXAxis ;
      SmVector3d sNegY = -sYAxis ;
      SmVector3d *pX, *pY ;    
      

#ifdef SM_DEBUG_CODE
static constexpr ULONG bDebugMe = FALSE ;
static ULONG lCount = 1 ; lCount++ ;
static ULONG lDebugCount = 0 ;
      if(bDebugMe || lDebugCount == lCount)
        {
          // Build intersection ellipse - get face and Brep pointers
          SmPoint3d sPoint ;
          SmFace *pThisFace  = ((SmFace *)GetFace()) ;
          SmFace *pOtherFace = ((SmFace *)crPlane.GetFace()) ; 
          SmBrep *pThisBrep  = pThisFace  ? pThisFace->GetBrep()  : NULL ;
          SmBrep *pOtherBrep = pOtherFace ? pOtherFace->GetBrep() : NULL ;

          // draw Breps(Blue,Green), Surfaces(cyan,yellow), faces(black), 
          // ellipse(red), TrimPoints(blue)
          smgfx_Erase() ; 
          smgfx_SetLook(1,2, 0,0,1) ; if(pThisBrep)  { pThisBrep->Draw(TRUE) ; }  sm_GraphicsLoop() ; 
          smgfx_SetLook(1,2, 0,1,0) ; if(pOtherBrep && pOtherBrep != pThisBrep) { pOtherBrep->Draw(TRUE) ; } sm_GraphicsLoop() ; 
          smgfx_SetLook(1,2, 0,1,1) ; DrawUV(10,10) ; sm_GraphicsLoop() ;
          smgfx_SetLook(1,2, 1,1,0) ; crPlane.DrawUV(10,10) ; sm_GraphicsLoop() ;
          // smgfx_SetLook(1,2, 0,0,0) ; if(pThisFace)  { pThisFace->Draw(SM_DM_CROSSHATCH,10,10) ; } sm_GraphicsLoop() ;
          // smgfx_SetLook(1,2, 0,0,0) ; if(pOtherFace) { pOtherFace->Draw(SM_DM_CROSSHATCH,10,10) ; } sm_GraphicsLoop() ;
      
          smgfx_SetLook(2,3, 1,0,0) ; sEllipse.Draw() ; sm_GraphicsLoop() ;
          for(ii=0;ii<lCnt;ii++)
            {
              sEllipse.EvaluateSTEP(dXSectAngle[ii], 0, TRUE, &sPoint) ;
              smgfx_SetLook(3,6, 0,0,1) ; sPoint.Draw() ; sm_GraphicsLoop() ;
            }
          for(ii=0;ii<4;ii++)
            {
              if(   (ii < 2 && lNumTsect1 == 2)
                 || (ii >=2 && lNumTsect2 == 2))
                { smgfx_SetLook(4,8, 1,0,0) ; sTsectPoints[ii].Draw() ; sm_GraphicsLoop() ; }
            }
          sm_GraphicsLoop() ;
        }
#endif // SM_DEBUG_CODE

      // for every segment between sStartAngle and sEndAngle - build and output an arc
      // limiting the iterator ii trims the output arc to the cone start/end limits
      for(ii=lStartIndex,i1=ii+1;ii%lCnt!=lEndIndex;ii++,i1++)
        {
          // skip exiting segments
          if(bXSectEntering[ii%lCnt] == FALSE)
            { continue ; }

          // see if 1st and last segment can be combined into a single arc
          if(  (ii%lCnt==0 || ii%lCnt==lCnt-2)        // this is the 1st or last segment on our periodic list and
             && lCnt > 2                             // there is more than 1 segment being built and 
             && bXSectEntering[0] == TRUE            // 1st segment is inside and
             && bXSectEntering[lCnt-2] == TRUE       // last segment is inside and
             && (   !SM_ARE_SAME(dStartAngle, 0.0)   // and boundary is not on a seam
                 || !SM_ARE_SAME(dStartAngle, 360.0))
             && (   bIsClosed                        // cone is closed or
                 || lStartIndex > lEndIndex)         // the ellipse maxima at 0 is inside the cone start/end limits
             && (   dXSectAngle[lCnt-2] > 180.0      // and the ellipse arc does not include the other extrema
                 && dXSectAngle[1]      < 180.0))
            {
              // Start ellipse at other extrema and combine 1st and last intervals
              pX = &sNegX ;
              pY = &sNegY ;

              // get rotated arc's interval intersected with the cone interval bounds
              double dStartA = dXSectAngle[lCnt-2] - 180.0 ;
              double dEndA   = dXSectAngle[1]      - 180.0 ;
              if(dStartA < 0.0) dStartA += 360.0 ;
              if(dEndA   < 0.0) dEndA   += 360.0 ;
              sIvl.SetMinMax(dStartA, dEndA) ;

              // clear bSXectEntering values to prevent duplicating this arc
              bXSectEntering[lCnt-2] = FALSE ;
              bXSectEntering[0]      = FALSE ;

            } // end chance to combine 1st and last segment check
          else
            { 
              // start ellipse at "end2" extrema
              pX = &sXAxis ;
              pY = &sYAxis ;

              // get segment interval
              sIvl.SetMinMax(dXSectAngle[ii%lCnt], dXSectAngle[i1%lCnt]) ;
              bXSectEntering[ii%lCnt] = FALSE ;
            }

          // skip degenerate and NULL curves
          if(sIvl.GetLength() < SM_EFF_ZERO * 360.0)
            { 
              // could add a degenerate curve here 
              continue ; 
            }

          // build ellipse for this segment
          SmBSplineCurve *  pCrv = new (crContext) SmEllipse(sCenter,*pX,*pY,
                                                             sIvl,dMajRad,dMinRad,3);
          // set outputs
          if (pCrv) { if (pOpt3DCurves)         { pOpt3DCurves->Add(pCrv); }
                      if (pOptCurveTypes)       { pOptCurveTypes->Add(SM_TC_CROSSING); }
                      if (pOptSurface1UVCurves) { pOptSurface1UVCurves->Add(NULL); }
                      if (pOptSurface2UVCurves) { pOptSurface2UVCurves->Add(NULL); }
                      if (pOptDeviations)       { pOptDeviations->Add(0.0); }
                    }

        } // end iter every interval building output curves
      // all done
      rbNeedsMoreIntersections = FALSE;
      return SM_SUCCESS;

    } // end plane at an angle to cone axis branch
*/
  // set outputs
  if (pCrv1) { if (pOpt3DCurves)         { pOpt3DCurves->Add(pCrv1); }
               if (pOptCurveTypes)       { pOptCurveTypes->Add(eCurveType1); }
               if (pOptSurface1UVCurves) { pOptSurface1UVCurves->Add(NULL); }
               if (pOptSurface2UVCurves) { pOptSurface2UVCurves->Add(NULL); }
               if (pOptDeviations)       { pOptDeviations->Add(0.0); }
             }
  if (pCrv2) { if (pOpt3DCurves)         { pOpt3DCurves->Add(pCrv2);  }
               if (pOptCurveTypes)       { pOptCurveTypes->Add(eCurveType2); }
               if (pOptSurface1UVCurves) { pOptSurface1UVCurves->Add(NULL); }
               if (pOptSurface2UVCurves) { pOptSurface2UVCurves->Add(NULL); }
               if (pOptDeviations)       { pOptDeviations->Add(0.0); }
             }

  SM_ASSERT(   (pOpt3DCurves->GetSize() == 0 && pCrv1 == NULL && pCrv2 == NULL)
            || (pOpt3DCurves->GetSize() == 1 && (   (pCrv1 != NULL && pCrv2 == NULL) 
                                                 || (pCrv1 == NULL && pCrv2 != NULL)))
            || (pOpt3DCurves->GetSize() == 2 && pCrv1 != NULL && pCrv2 != NULL)) ;


#ifdef SM_DEBUG_CODE
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
      smgfx_SetLook(3,4, 1,0,0) ; for(ULONG ii=0;ii<pOpt3DCurves->GetSize();ii++)
                                    { SmCurve *pCurve = pOpt3DCurves->GetAt(ii) ;
                                      pCurve->Draw() ; sm_GraphicsLoop() ;
                                    }
      sm_GraphicsLoop();
    }
#endif

  // all done
  return SM_SUCCESS;

} // end SmCone::IntersectWithPlane

/*******************************************************************//**
PURPOSE: Static function to determine if a given nurbs is a Cone.
    If it is a Cone then one is created.

NOTES: For now, it will check only those surfaces whose degree
    is two (with the proper knot vector - [triple, double, double .., triple]
    for one param & one for the other.
***********************************************************************/
SmBoolean SmCone::IsNurbSurfaceCone
  (const SmContext        & crContext,       // in : context for new object construction
   const SmBSplineSurface * pTestSurface,    // in : Surface to test
   SmCone                *& rpCone,          // out: new Cone when surface is cone - else NULL
                                             //      rpCone parameterization is likely to differ from pTestSurface
   double                   dToleranceScale) // in : tol = dToleranceScale * SM_EFF_ZERO * ANALYTIC_TOL_SCALE * BoundingBoxSize
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

  // low work - not rational, not proper degree, not linear in one direction or other
  ULONG           lDegU           = pTestSurface->GetDegree(SM_SP_U);
  ULONG           lDegV           = pTestSurface->GetDegree(SM_SP_V);
  SmExtent2d      sUVDomain       = pTestSurface->GetNaturalUVDomain();
  SmSurfParamType eLinearIsoParam =   (lDegU == 1 && pTestSurface->GetNumberControlPoints(SM_SP_U) == 2) ? SM_SP_V
                                    : (lDegV == 1 && pTestSurface->GetNumberControlPoints(SM_SP_V) == 2) ? SM_SP_U
                                    : SM_SP_NEITHER ;
  SmExtent1d      sLinearIvl      =   eLinearIsoParam == SM_SP_V ? sUVDomain.GetUInterval()
                                    : eLinearIsoParam == SM_SP_U ? sUVDomain.GetVInterval()
                                    : SmExtent1d() ;

  // quit when surface is not properly parameterized for a potential cone
  if (   lDegU + lDegV != 3          // one line and one circle
      || !pTestSurface->IsRational()
      || eLinearIsoParam == SM_SP_NEITHER)
    { return FALSE; }
  
  // check for proper knot vector in non-linear direction.
  //  For rapid STEPtoPolarConversion to work it must be
  //  of the form [tripleKnot, doubleKnot, doubleKnot, ..., tripleKnot]
  ULONG jj ;
  SmTArray<double> sCirKnots;
  SmTArray<ULONG>  sCirMults;
  pTestSurface->GetKnots(eLinearIsoParam, sCirKnots, &sCirMults) ;
  ULONG lCirKnotCount = sCirMults.GetSize() ;
  if(   sCirMults[0] != 3
     || sCirMults[lCirKnotCount-1] != 3)
    { return FALSE ; }
  for(jj=1;jj+1<lCirKnotCount;jj++) // note: can't say lCirKnotCount-1
    { 
      if(sCirMults[jj] != 2)
        return FALSE ; 
    }

  // evaluate Nurb min/max domain points
  SmPoint3d sMinPnt, sMaxPnt;
  pTestSurface->EvaluatePoint(sUVDomain.GetMin(),sMinPnt);
  pTestSurface->EvaluatePoint(sUVDomain.GetMax(),sMaxPnt);

  // scale zero value and angle tolerance
  double dAngTol     = dToleranceScale * SM_EFF_ZERO_SQRT ;
  double dScaledZero =   dToleranceScale  
                       * ANALYTIC_TOL_SCALE * SM_EFF_ZERO
                       * (1.0 + sMinPnt.GetMaxDimension() + sMaxPnt.GetMaxDimension());
  
  // locals
  SmBoolean        bSwapUV    = FALSE;
  SmSurfParamType  eSweepIsoParam ;
  double dBotKnot, dTopKnot;

  // branch on linear direction to find circular and linear directions
  if (lDegU == 1) 
    {
      // Need to swap UV when mapping parameters
      // from analytic domain to nurbs domain
      bSwapUV         = TRUE;
      eSweepIsoParam  = SM_SP_U;
      eLinearIsoParam = SM_SP_V;
      dBotKnot        = sUVDomain.GetMin().x;
      dTopKnot        = sUVDomain.GetMax().x;
    }
  else // work with Y dir values as is
    {
      bSwapUV         = FALSE;
      eSweepIsoParam  = SM_SP_V;
      eLinearIsoParam = SM_SP_U;
      dBotKnot        = sUVDomain.GetMin().y;
      dTopKnot        = sUVDomain.GetMax().y;
    }

  // make temporary Top BSPline isoParameter curve - hopefully it has an arc shape
  SmBSplineCurve  *pTopIsoCrv = NULL;
  SER(pTestSurface->SmBSplineSurface::CreateIsoParametricCurve(crContext, 
                                                               eSweepIsoParam, 
                                                               dTopKnot, 
                                                               0.0, 
                                                               pTopIsoCrv));
  SmObjDelete sCleanTop(pTopIsoCrv);

  // make temporary Bot BSPline isoParameter curve - hopefully it has an arc shape
  SmBSplineCurve  *pBotIsoCrv = NULL;
  SER(pTestSurface->SmBSplineSurface::CreateIsoParametricCurve(crContext, 
                                                               eSweepIsoParam, 
                                                               dBotKnot, 
                                                               0.0, 
                                                               pBotIsoCrv));
  SmObjDelete sCleanBot(pBotIsoCrv);

  // classify IsoparamCurves - get Nurb TopCurve and BotCurve radius and start/end 
  // angles measured CCW from axis running from arc centerPoint to startPoint
  double dTopRad = 0.0,  dTopStartAng = 0.0, dTopEndAng = 0.0;    
  double dBotRad = 0.0,  dBotStartAng = 0.0, dBotEndAng = 0.0;
  SmAxis2Placement sTopRefFrame, sBotRefFrame ;
  SmBoolean bTopDegenerate = pTopIsoCrv->IsDegenerate(dScaledZero) ;
  SmBoolean bBotDegenerate = pBotIsoCrv->IsDegenerate(dScaledZero) ;
  SmBoolean bTopArc        = bTopDegenerate ? FALSE : pTopIsoCrv->IsArc(5, dScaledZero, sTopRefFrame, dTopRad, dTopStartAng, dTopEndAng) ;
  SmBoolean bBotArc        = bBotDegenerate ? FALSE : pBotIsoCrv->IsArc(5, dScaledZero, sBotRefFrame, dBotRad, dBotStartAng, dBotEndAng) ;

  //  2 Arcs                       = were good, continue
  //  1 Arc and a degenerate curve = cone has a vertex, set zero radius arc parameters
  //  2 degenerate curves          = not a cone, error
  if     (bTopArc && bBotDegenerate)
    { // cone with a vertex
      dBotStartAng = dTopStartAng ;
      dBotEndAng   = dTopEndAng ;
      dBotRad      = 0.0 ;
      SmPoint3d sPoint ; 
      pBotIsoCrv->EvaluatePoint(pBotIsoCrv->GetNaturalInterval().GetMid(), sPoint) ;
      sBotRefFrame.SetCanonical(sPoint,
                                sTopRefFrame.GetXAxisRef(),
                                sTopRefFrame.GetYAxisRef()) ;
    }
  else if(bBotArc && bTopDegenerate)
    { // cone with a vertex                      
      dTopStartAng = dBotStartAng ;                  
      dTopEndAng   = dBotEndAng ;                    
      dTopRad      = 0.0 ;
      SmPoint3d sPoint ; 
      pTopIsoCrv->EvaluatePoint(pTopIsoCrv->GetNaturalInterval().GetMid(), sPoint) ;
      sTopRefFrame.SetCanonical(sPoint,
                                sBotRefFrame.GetXAxisRef(),
                                sBotRefFrame.GetYAxisRef()) ;
    }
  else if(!bTopArc || !bBotArc)
    { // not a cone - Top or Bot circular isoParameter curve is degenerate or not an arc
      return FALSE ;
    }
  
  // check for not a cone - top/bot start angles or top/bot end angles vary by more than tolerance
  double dTwistAng;
  sTopRefFrame.GetXAxisRef().AngleBetween(sBotRefFrame.GetXAxisRef(),dTwistAng);
  if (   smos_Fabs(dTopStartAng-dBotStartAng) > dAngTol
      || smos_Fabs(dTopEndAng  -dBotEndAng  ) > dAngTol
      || smos_Fabs(dTwistAng) > dAngTol)
    { return FALSE; }

  // get the cone height = length of axis vector from botCurve Center to TopCurve Center
  const SmPoint3d  &rCenterTop = sTopRefFrame.GetOriginRef();
  const SmPoint3d  &rCenterBot = sBotRefFrame.GetOriginRef();
  SmVector3d       sNurbZAxis  = rCenterTop - rCenterBot;
  double           dHeight     = sNurbZAxis.Length();

  // not a cone - height of cone is degenerate (cone is a disk) 
  if(dHeight < SM_EFF_ZERO_SQRT) 
    { return FALSE; }
  sNurbZAxis.Unitize();

  // check for not a cone - bottom arc's plane is not perpendicular to cone axis
  if (!sNurbZAxis.IsParallelTo(sBotRefFrame.GetZAxis(),dAngTol))
    { return FALSE; }

  // check for not a cone - a middle arc must also be concentric with axis,
  // and in a parallel plane.  This can be violated if, e.g., the weights at the
  // two circular ends do not match. [090603]
  // Make temporary Mid BSPline isoParameter curve.
  // It should be circular and fit in with the cone's coordinate system.
  SmBSplineCurve  *pMidIsoCrv = NULL;
  double dMidRad,  dMidStartAng, dMidEndAng;
  SmAxis2Placement sMidRefFrame;
  double dMidParam = ( dBotKnot + dTopKnot ) / 2.0;
  SER(pTestSurface->SmBSplineSurface::CreateIsoParametricCurve(crContext, 
                                                               eSweepIsoParam, 
                                                               dMidParam, 
                                                               0.0, 
                                                               pMidIsoCrv));
  SmObjDelete sCleanMid(pMidIsoCrv);

  // It must be circular.
  SmBoolean bMidArc = pMidIsoCrv->IsArc(5, dScaledZero, sMidRefFrame, dMidRad, dMidStartAng, dMidEndAng);
  if ( !bMidArc )
    { return FALSE; }

  // Radius must be halfway between top and bottom.
  if ( smos_Fabs( (dBotRad+dTopRad)/2.0 - dMidRad ) > dScaledZero )
    { return FALSE; }

  // check for not a cone - middle arc's plane is not perpendicular to cone axis
  if ( ! sMidRefFrame.GetZAxis().IsParallelTo( sNurbZAxis, dAngTol ) )
    { return FALSE; }

  // check for not a cone - middle arc's center is not on cone axis
  double dDistToAxis;
  SmStatus eStat = smgu_LinePointDistance( rCenterTop, sNurbZAxis, sMidRefFrame.GetOriginRef(), dDistToAxis );
  if ( eStat != SM_SUCCESS || dDistToAxis > dScaledZero )
    { return FALSE; }


  // arrive here when surface has passed all not-a-cone filters

  // By definition: the circle rotation sets the Z axis for the cone
  // when the CenterBot to CenterTop vector opposes this Z axis, bInsideOut = TRUE
  SmBoolean bInsideOut =   (sNurbZAxis.Dot(sBotRefFrame.GetZAxis()) < 0.0)
                         ? TRUE
                         : FALSE;

  // For the x-axis, use the one (top or bottom) at the beginning of the
  // param range in that direction.  This is because that's what
  // SmSurfOfRevolution::RebuildPolarConverter() uses, and they could differ,
  // with dTwistAngle <= dAngTol (above).  That can cause problems when
  // setting up the polar converter.
  const SmVector3d &rXAxis = ( bInsideOut ) ? sTopRefFrame.GetXAxisRef()
                                            : sBotRefFrame.GetXAxisRef();

  // get STEP YAxis from XAxis and StepZAxis
  SmVector3d sStepZAxis =  bInsideOut ? -sNurbZAxis
                                      :  sNurbZAxis ;
  SmVector3d sYAxis = sStepZAxis * rXAxis;

  // when top and bot radius values are nearly equal
  if (smos_Fabs(dBotRad-dTopRad) < SM_EFF_ZERO*(1.0+dBotRad)) 
    {
      // make them exactly equal - Try to eliminate some numerical inaccurracies
      dTopRad = dBotRad; 
    }

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe2 = FALSE;
  if (bDebugMe2) 
    {
      smgfx_Erase();
      rCenterBot.Draw(); sm_GraphicsLoop();
      pTestSurface->DrawUV(3,3); sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE
      
  // build the analytic cone surface
  // bInsideOut toggles the STEP Origin between Top and Bot NurbCircles
  rpCone = new (crContext) SmCone(bInsideOut ? sTopRefFrame.GetOriginRef() : sBotRefFrame.GetOriginRef(), 
                                  rXAxis, sYAxis,
                                  bInsideOut ? dTopRad : dBotRad,
                                  bInsideOut ? dBotRad : dTopRad,
                                  dTopStartAng, dTopEndAng,
                                  dHeight, bSwapUV,
                                  bInsideOut, FALSE);
  if (!rpCone) { return FALSE; } 

  // error recovery
  if (rpCone->m_pGenCurve == NULL) { delete rpCone; rpCone = NULL;
                                     return FALSE;
                                   }

  // gwc: the next assertion may be wrong - checking it, for now I'm leaving the code in its orig condition: with Reparameterize 
  //  // gwc: It's a mistake to force the rpCone parameterization to be the same as pTestSurface
  //  //      The cone parametrization is highly constrained and will not generally be the same as the pTestSurface
  //  //      
  // reparameterize the NURB so that AnalyticSurface->NurbDomain == InputSurface->NurbDomain
  rpCone->Reparameterize(pTestSurface->GetNaturalUVDomain()) ;


  // obsolete - it's a mistake to copy the pTestSurface Nurb into the new Cone object
  //            that adds tolerance sized variations between the Nurb and Anal representations
  //      // Copy pTestSurface->m_pNurb into newCone
  //      SM_ASSERT(rpCone->m_pNurb == NULL) ;
  //      rpCone->m_pNurb = sm_AllocateAndCopyNurbSurface(((SmBSplineSurface *)pTestSurface)->GetGwNurbPointer()) ;
  //      
  //      
  //      // Set m_pGenCone's Nurb domain to equal m_pNurb's linear domain
  //      SmExtent1d sNurbLineIvl =   bSwapUV
  //                                ? rpCone->GetNaturalUVDomain().GetUInterval() 
  //                                : rpCone->GetNaturalUVDomain().GetVInterval() ;
  //      rpCone->m_pGenCurve->SmBSplineCurve::EditParameterization(sNurbLineIvl) ;
  //
  //      // Build newCone PolarConverter from Nurb midPoint rotation isoParamCurve
  //      // Build one more temporary isoParameterCurve - because this time it will be of type SmCircle
  //      rpCone->RebuildPolarConverter() ;
  // end obsolete

  // Copy pTestSurface attributes onto newCone
  ((SmBSplineSurface *)pTestSurface)->Notify(SM_NO_COPY, rpCone, SM_NO_GET_OWNER(rpCone), SM_NO_GET_OWNER(pTestSurface)) ;

  //      SmBSplineCurve *pRotationIsoCrv = NULL ;
  //      SmPoint2d       sMidUV          = sUVDomain.Evaluate(0.5432,0.5432);
  //      rpCone->m_vPolarConverter.SetPolarConversionPossible(FALSE) ;
  //      SER(rpCone->CreateIsoParametricCurve(*rpCone->GetContext(),
  //                                           eSweepIsoParam, dBotKnot, 0.0, pRotationIsoCrv));
  //      SmObjDelete sCleanIsoRot(pRotationIsoCrv) ;
  //      
  //      // Build the Cone's PolarConverter for rotation angle based on the pRotationIsoCrv
  //      if (!rpCone->m_vPolarConverter.IsPolarConversionPossible()) 
  //        {
  //          // build polar converter cache - check representation - store angles for each natural knot
  //          sCleanIsoRot.Clear() ;
  //          SE(rpCone->m_vPolarConverter.SetUpPolarConversion
  //                                         (pRotationIsoCrv,
  //                                          FALSE,
  //                                          rpCone->m_vAnalUVDomain.GetUInterval(),
  //                                          rpCone->m_vPosition));
  //        }

#ifdef SM_DEBUG_CODE
  if (bDebugMe) 
    {
      SM_DUMP_AND_ASSERT_VALID(pTestSurface) ;
      SM_DUMP_AND_ASSERT_VALID(rpCone) ;

      // exercise all the CreateCurve from Surf functions to test domain compatibility
      SmExtent2d sNurbUVDomain = rpCone->GetNaturalUVDomain() ;
      SmPoint3d sMidPoint ;
      rpCone->EvaluatePoint(sNurbUVDomain.Evaluate(.45678,.45678), sMidPoint) ;

      SmBSplineCurve *pSweepIso  = NULL ; // for CreateIsoParametricCurve 
      SmBSplineCurve *pLinearIso = NULL ; // for CreateIsoParametricCurve
      SmBSplineCurve *pCircle    = NULL ; // for CreateIsoCircleFromPoint
      SmBSplineCurve *pDirectrix = NULL ; // for CreateGeneratorFromAngle
      SmBSplineCurve *pGenerator = NULL ; // for CreateDirectrixFromPoint

      double dMinU = sNurbUVDomain.GetMin().x ;
      double dMinV = sNurbUVDomain.GetMin().y ;
      if (rpCone->m_bSwapUV) { SER(rpCone->CreateIsoParametricCurve(crContext,SM_SP_U,dMinU,SM_EFF_ZERO,pSweepIso));
                               SER(rpCone->CreateIsoParametricCurve(crContext,SM_SP_V,dMinV,SM_EFF_ZERO,pLinearIso));
                             }
      else                   { SER(rpCone->CreateIsoParametricCurve(crContext,SM_SP_V,dMinV,SM_EFF_ZERO,pSweepIso));
                               SER(rpCone->CreateIsoParametricCurve(crContext,SM_SP_U,dMinU,SM_EFF_ZERO,pLinearIso));
                             }
      rpCone->CreateIsoCircleFromPoint(crContext, sMidPoint, SM_EFF_ZERO, pCircle) ;
      rpCone->CreateDirectrixFromPoint(crContext, sMidPoint, SM_EFF_ZERO, pDirectrix) ;
      rpCone->CreateGeneratorFromAngle(crContext, 15.0, pGenerator) ;
      SmObjDelete sCleanIso0(pSweepIso);
      SmObjDelete sCleanIso1(pLinearIso);
      SmObjDelete sCleanIso2(pCircle);
      SmObjDelete sCleanIso3(pDirectrix);
      SmObjDelete sCleanIso4(pGenerator);

      pTestSurface->Dump();
      rpCone->Dump();
      rpCone->m_pGenCurve->Dump();
      if(pSweepIso)  pSweepIso->Dump() ;
      if(pLinearIso) pLinearIso->Dump() ;
      if(pCircle)    pCircle->Dump() ;
      if(pDirectrix) pDirectrix->Dump() ;
      pGenerator->Dump() ;

      smgfx_Erase();
      smgfx_SetLook( 1,2, 0,0,1); rpCone->DrawUV(); sm_GraphicsLoop();
      smgfx_SetLook( 3,4, 1,0,0); rpCone->m_pGenCurve->Draw(); sm_GraphicsLoop();
      smgfx_SetLook( 5,6, 1,0,1); if(pSweepIso)  pSweepIso->Draw(); sm_GraphicsLoop();
      smgfx_SetLook( 7,8, 0,1,1); if(pLinearIso) pLinearIso->Draw(); sm_GraphicsLoop();
      smgfx_SetLook( 9,8, 0,1,1); if(pCircle)    pCircle->Draw(); sm_GraphicsLoop();
      smgfx_SetLook(11,8, 1,1,0); if(pDirectrix) pDirectrix->Draw(); sm_GraphicsLoop();
      smgfx_SetLook(13,8, 1,1,0); pGenerator->Draw(); sm_GraphicsLoop();
      smgfx_SetLook( 3,4, 0,1,0); pTestSurface->DrawUV(4,4); sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
#endif

  // all done
  SM_DUMP_AND_ASSERT2_VALID(rpCone) ;
  return TRUE;

} // end SmCone::IsNurbSurfaceCone

/*******************************************************************//**
PURPOSE: Rebuild all analytic, GenCurve, and PolarCurve cached data 
   from the m_pNurb data. 

NOTES: Can be used after modifying the m_pNurb data to update
  the analytic data.

RETURNS --- SM_ERR if the NURB no longer represents this Analytic surface
***********************************************************************/
SmStatus SmCone::RebuildSTEPFromNURBParameters()
{
  // check m_pNurb - a degree 2 rational surface in at least 1 direction
  ULONG           lDegU           = GetDegree(SM_SP_U);
  ULONG           lDegV           = GetDegree(SM_SP_V);
  SmExtent2d      sNurbUVDomain   = GetNaturalUVDomain();
  SmSurfParamType eLinearIsoParam =   (lDegU == 1 && GetNumberControlPoints(SM_SP_U) == 2) ? SM_SP_V
                                    : (lDegV == 1 && GetNumberControlPoints(SM_SP_V) == 2) ? SM_SP_U
                                    : SM_SP_NEITHER ;
  SmExtent1d      sLinearIvl      =   eLinearIsoParam == SM_SP_V ? sNurbUVDomain.GetUInterval()
                                    : eLinearIsoParam == SM_SP_U ? sNurbUVDomain.GetVInterval()
                                    : SmExtent1d() ;

  // no work - modified parameterization
  if (   lDegU + lDegV != 3          // one line and one circle
      || !IsRational()
      || eLinearIsoParam == SM_SP_NEITHER)
    { SER_MSG(SM_ERR, _T("Modified Nurb SmCone: no longer has degree 1 and 2 basis functions")) ; }

  // method
  //  Check for iso circles

  //  Set m_pGenCurve to be SmLine
  //  Set m_dBaseRadius
  //  Set m_dSemiAngleDeg
  //  Set m_bInsideOut
  //  Set m_bPlanarGenerator
  //  Set m_vPosition
  //  Set m_bSwapUV
  //  Set m_vAnalUVDomain
  //  Set PolarConverter 

// NOTE: this is identical to IsNurbSurfaceCone().  Just call that?

  // evaluate Nurb min/max domain points
  SmPoint3d  sNurbMinPoint, sNurbMaxPoint;
  EvaluatePoint(sNurbUVDomain.GetMin(),sNurbMinPoint);
  EvaluatePoint(sNurbUVDomain.GetMax(),sNurbMaxPoint);

  // scaled zero value
  double dToleranceScale = 1000.0 ;  // loose because surf is already labeled analytic
  double dAngTol         = dToleranceScale * SM_EFF_ZERO_SQRT ;
  double dScaledZero     =   dToleranceScale      
                           * ANALYTIC_TOL_SCALE 
                           * SM_EFF_ZERO 
                           * (1.0 + sNurbMinPoint.GetMaxDimension() + sNurbMaxPoint.GetMaxDimension());
  // locals
  SmBoolean        bSwapUV    = FALSE;
  SmSurfParamType  eSweepIsoParam ;
  double dBotKnot, dTopKnot;

  // branch on linear direction to find circular and linear directions
  if (lDegU == 1) 
    {
      // Need to swap UV when mapping parameters
      // from analytic domain to nurbs domain
      bSwapUV         = TRUE;
      eSweepIsoParam  = SM_SP_U;
      eLinearIsoParam = SM_SP_V;
      dBotKnot        = sNurbUVDomain.GetMin().x;
      dTopKnot        = sNurbUVDomain.GetMax().x;
    }
  else // work with Y dir values as is
    {
      bSwapUV         = FALSE;
      eSweepIsoParam  = SM_SP_V;
      eLinearIsoParam = SM_SP_U;
      dBotKnot        = sNurbUVDomain.GetMin().y;
      dTopKnot        = sNurbUVDomain.GetMax().y;
    }

  // make temporary Top BSPline isoParameter curve - hopefully it has an arc shape
  const SmContext *cpContext = GetContext() ;
  SmBSplineCurve  *pTopIsoCrv = NULL;
  SER(SmBSplineSurface::CreateIsoParametricCurve(*cpContext, 
                                                  eSweepIsoParam, 
                                                  dTopKnot, 
                                                  0.0, 
                                                  pTopIsoCrv));
  SmObjDelete sCleanTop(pTopIsoCrv);

  // make temporary Bot BSPline isoParameter curve - hopefully it has an arc shape
  SmBSplineCurve  *pBotIsoCrv = NULL;
  SER(SmBSplineSurface::CreateIsoParametricCurve(*cpContext, 
                                                  eSweepIsoParam, 
                                                  dBotKnot, 
                                                  0.0, 
                                                  pBotIsoCrv));
  SmObjDelete sCleanBot(pBotIsoCrv);

  // classify IsoparamCurves - get Nurb TopCurve and BotCurve radius and start/end 
  // angles measured CCW from axis running from arc centerPoint to startPoint
  double dTopRad = 0.0,  dTopStartAng = 0.0, dTopEndAng = 0.0 ;    
  double dBotRad = 0.0,  dBotStartAng = 0.0, dBotEndAng = 0.0 ;
  SmAxis2Placement sTopRefFrame, sBotRefFrame ;
  SmBoolean bTopDegenerate = pTopIsoCrv->IsDegenerate(dScaledZero) ;
  SmBoolean bBotDegenerate = pBotIsoCrv->IsDegenerate(dScaledZero) ;
  SmBoolean bTopArc        = bTopDegenerate ? FALSE : pTopIsoCrv->IsArc(5, dScaledZero, sTopRefFrame, dTopRad, dTopStartAng, dTopEndAng) ;
  SmBoolean bBotArc        = bBotDegenerate ? FALSE : pBotIsoCrv->IsArc(5, dScaledZero, sBotRefFrame, dBotRad, dBotStartAng, dBotEndAng) ;

  //  2 Arcs                       = were good, continue
  //  1 Arc and a degenerate curve = cone has a vertex, set zero radius arc parameters
  //  2 degenerate curves          = not a cone, error
  if     (bTopArc && bBotDegenerate)
    { // cone with a vertex
      dBotStartAng = dTopStartAng ;
      dBotEndAng   = dTopEndAng ;
      dBotRad      = 0.0 ;
      SmPoint3d sPoint ; 
      pBotIsoCrv->EvaluatePoint(pBotIsoCrv->GetNaturalInterval().GetMid(), sPoint) ;
      sBotRefFrame.SetCanonical(sPoint,
                                sTopRefFrame.GetXAxisRef(),
                                sTopRefFrame.GetYAxisRef()) ;
    }
  else if(bBotArc && bTopDegenerate)
    { // once with a vertex                      
      dTopStartAng = dBotStartAng ;                  
      dTopEndAng   = dBotEndAng ;                    
      dTopRad      = 0.0 ;
      SmPoint3d sPoint ; 
      pTopIsoCrv->EvaluatePoint(pTopIsoCrv->GetNaturalInterval().GetMid(), sPoint) ;
      sTopRefFrame.SetCanonical(sPoint,
                                sBotRefFrame.GetXAxisRef(),
                                sBotRefFrame.GetYAxisRef()) ;
    }
  else if(!bTopArc || !bBotArc)
    { // not a cone - Top or Bot circular isoParameter curve is degenerate or not an arc
      SER_MSG(SM_ERR, _T("Modified Nurb SmCone: no longer has circular top and bottom IsoParamCurves")) ;
    }

  // not a cone - top/bot start angles or top/bot end angles vary by more than tolerance
  double dTwistAng;
  sTopRefFrame.GetXAxisRef().AngleBetween(sBotRefFrame.GetXAxisRef(),dTwistAng);
  if (   smos_Fabs(dTopStartAng-dBotStartAng) > dAngTol
      || smos_Fabs(dTopEndAng  -dBotEndAng  ) > dAngTol
      || smos_Fabs(dTwistAng) > dAngTol)
    { SER_MSG(SM_ERR, _T("Modified Nurb SmCone: Top and Bot IsoParamCurves no longer Start and Stop at same angles")) ; }

  // get the cone height = length of axis vector from botCurve Center to TopCurve Center
  const SmPoint3d  &rCenterTop = sTopRefFrame.GetOriginRef();
  const SmVector3d &rXAxis     = sTopRefFrame.GetXAxisRef();
  const SmPoint3d  &rCenterBot = sBotRefFrame.GetOriginRef();
  SmVector3d       sNurbZAxis  = rCenterTop - rCenterBot;
  double           dHeight     = sNurbZAxis.Length();

  // not a cone - height of cone is degenerate (cone is a disk) 
  if(dHeight < SM_EFF_ZERO_SQRT) 
    { SER_MSG(SM_ERR, _T("Modified Nurb SmCone: Cone Height is now degenerate")) ; }
  sNurbZAxis.Unitize();

  // not a cone - bottom arc's plane is not perpendicular to cone axis
  if (!sNurbZAxis.IsParallelTo(sBotRefFrame.GetZAxis(),dAngTol))
    { SER_MSG(SM_ERR, _T("Modified Nurb SmCone: Top and Bottom IsoParamCurves are not coPlanar")) ; }

  // By definition: the circle rotation sets the Z axis for the cone
  // when the CenterBot to CenterTop vector opposes this Z axis, bInsideOut = TRUE
  SmBoolean bInsideOut =   (sNurbZAxis.Dot(sBotRefFrame.GetZAxis()) < 0.0)
                         ? TRUE
                         : FALSE;

  // get STEP YAxis from XAxis and StepZAxis
  SmVector3d sStepZAxis =  bInsideOut ? -sNurbZAxis
                                      :  sNurbZAxis ;
  SmVector3d sYAxis = sStepZAxis * rXAxis;

  // when top and bot radius values are nearly equal
  if (smos_Fabs(dBotRad-dTopRad) < SM_EFF_ZERO*(1.0+dBotRad)) 
    {
      // make them exactly equal - Try to eliminate some numerical inaccurracies
      dTopRad = dBotRad; 
    }

  // arrive here when surface has passed all not-a-cone filters

  // modify m_dSemiAngleDeg for nonCylinder shapes
  double dRadiusDiff = bInsideOut ? dBotRad - dTopRad
                                  : dTopRad - dBotRad;
  if (smos_Fabs(dRadiusDiff) > SM_EFF_ZERO) 
    {
      // The trick here is to generate a line with parameterization 
      // which is equivalent to the height of the cone/cylinder.
      // The V parameter which corresponds to the STEP definition is the
      // 'height' from the cone's origin. Therefore, a scaled direction
      // vector of line(generator) will allow syncronization between height
      // and V parameter(STEP-based) of SmSurfOfRevolution (which is 
      // the generator's parameter.)
      // The scale of direction vector is: 1.0/cos(m_dSemiAngleDeg).
      // First, calculate m_dSemiAngleDeg
      m_dSemiAngleDeg = smos_ArcTangent2(dRadiusDiff,dHeight) * 180.0 / SM_PI;
    }

  // set Analytic parameters
  m_dBaseRadius      = bInsideOut ? dTopRad : dBotRad ;
  m_bInsideOut       = bInsideOut ; 
  m_bPlanarGenerator = TRUE ;
  m_vPosition = bInsideOut ? sTopRefFrame : sBotRefFrame;
  m_bSwapUV          = bSwapUV ;
  m_vAnalUVDomain.SetMinMax(dTopStartAng, 0.0, dTopEndAng, dHeight) ;

  // Make and Save m_pGenCurve
  SmPoint3d  sLinePoint =   m_vPosition.GetOriginRef() 
                          + m_dBaseRadius * m_vPosition.GetXAxisRef() ;
  SmVector3d sLineVec   =   dHeight * m_vPosition.GetZAxis() 
                          + (GetRadius(dHeight) - m_dBaseRadius) * m_vPosition.GetXAxisRef() ;
  double     dLength    = sLineVec.Length() ;
  SmLine   * pLine      = new (*cpContext) SmLine
                                  (sLinePoint,                // in : LinePoint
                                   sLineVec,                  // in : UnitVec
                                   SmExtent1d(0.0, dHeight),  // in : analytic domain
                                   dLength/dHeight,           // in : scale
                                   3, cpContext, NULL,        // in : dim, context, optNurb
                                   bInsideOut) ;              // in : InsideOut
  // set the NurbDomain
  pLine->SmBSplineCurve::EditParameterization(sLinearIvl, FALSE) ;

  // Build newCone PolarConverter from Nurb midPoint rotation isoParamCurve
  // Build one more temporary isoParameterCurve - because this time it will be of type SmCircle
  RebuildPolarConverter() ;

  // Save the gencurve
  if(m_pGenCurve) { delete m_pGenCurve ; m_pGenCurve = NULL ; }
  m_pGenCurve = pLine;
  m_pGenCurve->SetOwner(this) ;

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
#endif // SM_DEBUG_CODE
      
  // all done
  SM_DUMP_AND_ASSERT2_VALID(this) ;
  return(SM_SUCCESS) ;

} // end SmCone::RebuildSTEPFromNURBParameters

/*******************************************************************//**
PURPOSE: Rotate surface about axis-of-revolution.  

NOTES:     
***********************************************************************/
SmStatus SmCone::RotationAboutAxisZ
  (const SmContext & crContext,   // in : new object context
   double            dAngleDeg,   // in : rotation amount in degrees
   SmCone         *& rpNewCone)   // out: new rotated cone
{
  double dAngleRad = SM_DEG2RAD(dAngleDeg);

  // build rotation tranformation
  SmAxis2Placement sRF;
  sRF.RotateAboutAxisAtPoint(dAngleRad, 
                             m_vPosition.GetOriginRef(), 
                             m_vPosition.GetZAxis());
  
  // transform the cone's placement
  SmAxis2Placement sTmpA2P;
  m_vPosition.TransformAxis2Placement(sRF,sTmpA2P);

  // make a new cone with rotated placement 
  SER(SmCone::CreateCanonical( crContext, sTmpA2P, m_dBaseRadius, m_dSemiAngleDeg, rpNewCone ));

  // trim UVDomain
  SER(rpNewCone->AdjustSTEPUVDomain(GetSTEPUVDomain()));

  // all done
  return SM_SUCCESS;

} // end SmCone::RotationAboutAxisZ
           
/*******************************************************************//**
PURPOSE: Given a point on the Surface-Of-Revolution, find its
    corresponding analytic UV-parameter.

NOTES: This method requires a point exactly on the surface.
    Therefore, users should be cautious when calling this method.

    Points which can be mapped outside the surface's 
    trim domain are mapped and reLocation is set to SM_LT_EXTERIOR

   If the result is on the seam of a closed surface, and a guess was
   given, the output will be set to whichever side of the seam is closer
   to the guess.  If no guess was given, it will be set to the low end
   of the closed domain.
***********************************************************************/
SmStatus SmCone::STEPInversion
  (const SmExtent2d &,                     // in : crAnalDomain
   const SmPoint3d  & crPointOnSurf,       // in : target point
   double             d3DTolerance,        // in : 
   SmPoint2d        & rdAnalUVParameter,   // out: 
   SmLocationType   & reLocation,          // out: 
   SmPoint2d        * pUVGuess)            // in, opt: guess parameter.
 const
{
  // init return value
  SmStatus sRtn = SM_SUCCESS ;

  // mirror points on the negative side of the apex to the positive side
  SmPoint3d sPoint ;
  double dApexParam ;
  SmBoolean bNegated = GetPositivePoint(crPointOnSurf, 
                                        sPoint,
                                        dApexParam) ;

  // do the conversion
  sRtn = SmSurfOfRevolution::STEPInversion(m_vAnalUVDomain,  
                                           sPoint,
                                           d3DTolerance,
                                           rdAnalUVParameter,  
                                           reLocation,
                                           pUVGuess) ;
  // when point is on far side of apex
  if(bNegated)
    {
      // adjust negated answers - flip the Z coordinate about the apex point
      rdAnalUVParameter.y  = 2.0 * dApexParam - rdAnalUVParameter.y ;
    } // end need to negate point check

  // all done
  return(sRtn) ;

} // end SmCone::STEPInversion

/*******************************************************************//**
PURPOSE: When Point is on negative side of the apex plane
            reflect point through the apex.

NOTES: Points reflected through the apex can be
    passed to the various solve routines and the parameter
    values mapped back to the negative domain to solve
    for points on infinite cones.

    The dApexParam value is needed to convert positive
    point solutions into negative ones as
    dNegParam = 2.0 * dApexParam - dPosParam ;
***********************************************************************/
SmBoolean SmCone::GetPositivePoint
  (const SmPoint3d &crTestPoint, // in : Point to check
   SmPoint3d &rPositivePoint,    // out: equal to rTestPoint when TestPoint is on the postive side of the apex
                                 //      else set to rTestPoint reflected through apex point
   double    &dApexParam)        // out: Param value on the GenCurve for
 const                           //      the apex point.
{
  // see if point lies on far side of cone apex
  SmBoolean bCylinder ;
  SmPoint3d sApexPoint ;
  GetApex(bCylinder, sApexPoint, dApexParam) ;
  SmVector3d sZAxis = m_vPosition.GetZAxis() ;
  SmBoolean bNegate =  (   !bCylinder
                        && (   (m_dSemiAngleDeg > 0.0 && (crTestPoint - sApexPoint).Dot(sZAxis) < 0.0)
                            || (m_dSemiAngleDeg < 0.0 && (crTestPoint - sApexPoint).Dot(sZAxis) > 0.0)))
                      ? TRUE
                      : FALSE ;

  // when point is on far side of apex
  if(bNegate)
    {
      // reflect this point through the Apex Point
      // to get a positive side TestPoint to classify and
      // negate the rdAnalUVParameter before returning.
      rPositivePoint = 2.0 * sApexPoint - crTestPoint ;
    }
  else // leave point alone
    {
      rPositivePoint = crTestPoint ;
    }

  // all done
  return(bNegate) ;

} // end SmCone::GetPositivePoint
                                      
//      /*******************************************************************//**
//      PURPOSE: Swap the U and V parameterizations of a surface.  This 
//          effectively reverses the orientation of the surface.  
//      
//      NOTES: 
//      ***********************************************************************/
//      SmStatus SmCone::SwapUV
//        ()
//      {
//        // swap underlying Nurb Surface
//        SER(SmBSplineSurface::SwapUV());
//        SmAxis2Placement sPlace;
//      
//        // Hit it with a unit transform to update the cone's information.
//        SER(Transform(sPlace,NULL));
//      
//        // all done
//        return SM_SUCCESS;
//      
//      } // end SmCone::SwapUV

/*******************************************************************//**
PURPOSE: Scale and transform an SmCone surface.

NOTES: Differential scaling is not allowed on analytical surfaces.
***********************************************************************/
SmStatus SmCone::Transform
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

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  // draw 
  if(bDebugMe)
    {
      SM_ASSERT_VALID(this) ;
    }
#endif // SM_DEBUG_CODE

  // Check input and get scaling value - allow only uniform scaling.
  double dScale = 1.0 ;
  if ( cpOptScale != NULL )
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

#ifdef SM_DEBUG_CODE
  // draw 
  if(bDebugMe)
    {
      SM_ASSERT_VALID(this) ;
    }
#endif // SM_DEBUG_CODE

  // call parent function to:
  //  : transform the GenCurve o leaving its analytic domain unchanged
  //                           o change m_vLinePoint, 
  //                                    m_vLineVector dir (unit-vector)
  //                                    m_dScale
  //                           o change m_pNurb
  //  : transform the SmSurfOfRevolution
  //     o change m_vPosition
  //     o change m_pNurb               

  SmSurfOfRevolution::Transform(crRotateNMove, cpOptScale) ;

  // Now transform cone-specific stuff.

  // update cone size
  m_dBaseRadius *= dScale;

  // Now enforce the special SmCone rules.
  // Our analytical v-domain is a linear distance, so we have to scale it. [B17]
  // (Not true of other analytic surfaces.)  
  if ( smos_Fabs( dScale - 1.0 ) > SM_EFF_ZERO )
    {
      // Scale it about v==0: that's where the base is.
      SmExtent1d sDomainV = m_vAnalUVDomain.GetVInterval();
      sDomainV.Scale(dScale);

      m_vAnalUVDomain.SetVInterval(sDomainV);
      //m_vAnalUVDomain.SetVMin( dScale * sDomainV.GetMin() );
      //m_vAnalUVDomain.SetVMax( dScale * sDomainV.GetMax() );
    }

  // change the GenCurve so that it shares the same analytic domain
  //  as the m_vAnalUVDomain.y and has a LineScale determined by the m_dSemiAngleDeg
  SmLine *pGenLine = SM_CAST_PTR(SmLine, m_pGenCurve) ;
  if ( pGenLine != NULL )
    {
      double dScaledZero = SM_EFF_ZERO * ( 1.0 + m_vAnalUVDomain.GetVInterval().GetLength() );

      if ( ! SM_ARE_SAME_TO_TOL( m_vAnalUVDomain.GetVInterval().GetLength(),
                                 pGenLine  -> GetSTEPInterval().GetLength(), dScaledZero ) )
        {
          double dAngleRad = SM_DEG2RAD( m_dSemiAngleDeg );
          double dSpeed = 1.0 / smos_Cosine( dAngleRad );
          dScale = pGenLine->GetLineScale();
          dScaledZero = SM_EFF_ZERO * ( 1.0 + dScale );

          if ( ! SM_ARE_SAME_TO_TOL( dScale, dSpeed, dScaledZero ) )
            {
              // Don't call SetCanonical: that recreates m_pNurb,
              // with different parameterization.  [B375]
              pGenLine->SetLineScale     ( dSpeed );
              pGenLine->SetAnalyticDomain( m_vAnalUVDomain.GetVInterval() );

            } // end adjusting Line analytic domain and scale
        }
    }

#ifdef SM_DEBUG_CODE
  // draw 
  if(bDebugMe)   
    {
      SM_ASSERT_VALID(this) ;
    }
#endif // SM_DEBUG_CODE

  // all done
  return SM_SUCCESS;

} // end SmCone::Transform

/*******************************************************************//**
PURPOSE: Compute the total size of the memory used by the SmCone.

NOTES: Does not add in attribute memory
***********************************************************************/
ULONG SmCone::GetMemoryUsed       // rtn: smaller size of actually used memory in bytes
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
  rlMemoryAllocated = sizeof(*this) + sm_ComputeNurbSurfaceSize(m_pNurb) ;

  // + attribute memory
  ULONG lThisAllocated ;
  ULONG lUsed       = rlMemoryAllocated + this->GetAttributeMemoryUsed(lThisAllocated, 
                                                                       eMarkType) ;  // note: uses without increment eMarkType value
  rlMemoryAllocated = rlMemoryAllocated + lThisAllocated ;

  // curve memory
  lUsed             += m_pGenCurve ? m_pGenCurve->GetMemoryUsed(lThisAllocated, eMarkType) : 0 ;
  rlMemoryAllocated += lThisAllocated ;

  // + cache memory
  if ( m_pCacheObj )
  {
      lUsed += m_pCacheObj->GetMemoryUsed( lThisAllocated );
      rlMemoryAllocated += lThisAllocated;
  }

  // all done
  return(lUsed) ;

} // end SmCone::GetMemoryUsed

/*******************************************************************//**
PURPOSE: Find the set of intervals on an ellipse known to be
            on the cone's surface that are inside given cone's 
            trim boundaries.

NOTES: return intervals are in the ellipse's anal domain
***********************************************************************/
void sm_GetEllipseTrimIntervals
  (const SmContext     &crContext,            // in : for temp object construction
   const SmPoint3d     &rEllipseOrigin,       // in : origin            of ellipse
   const SmVector3d    &rEllipseXAxis,        // in : xAxis             of ellipse
   const SmVector3d    &rEllipseYAxis,        // in : yAxis             of ellipse
   double               dEllipseXRadius,      // in : x radius          of ellipse
   double               dEllipseYRadius,      // in : y radius          of ellipse
   double               dEllipseStartAngDeg,  // in : start angle (deg) of ellipse
   double               dEllipseEndAngDeg,    // in : end angle (deg)   of ellipse
   const SmCone        &crCone,               // in : Cone with trim boundaries
   const SmExtent2d    &crUVDomain,           // in : nurb trim boundaries for cone
   double               dTol,                 // in : approximation tolerance
   SmTArray<SmExtent1d> &rIvls)               // out: array of ellipse intervals inside trimmed cone
{
  // gwc:NOT DONE YET - this function is not debugged and
  //     it does not yet trim to the input dEllipseStartAngDeg and
  //     dEllipseEndAngDeg.

  // init output
  rIvls.ReSet() ;

  // find ellipse/coneTrimBoundary intersections
  SmTArray<double> dXSectAngle(6,NULL,6) ;  // coneBoundary xSect Points
  SmBoolean        bInside[6] ;             // TRUE = ith Ivl is inside trim boundaries

  // ellipse locals
  ULONG ii, lCnt = 0 ;               
  SmVector3d       sEllipseZ       = rEllipseXAxis * rEllipseYAxis ;
  SmExtent1d       sEllipseIvl (dEllipseStartAngDeg, dEllipseEndAngDeg) ;
  SmEllipse        sFullEllipse(rEllipseOrigin, rEllipseXAxis, rEllipseYAxis,
                                SmExtent1d(0.0, 360.0),
                                dEllipseXRadius, dEllipseYRadius,
                                3, &crContext) ;
  // cone locals
  double           dHeight       = crCone.GetHeight();
  SmVector3d       sConeAxis     = crCone.GetPosition().GetZAxis();
//cbi change:  const SmPoint3d &rCenterBot    = crCone.GetPosition().GetOriginRef();
  SmPoint3d rCenterBot = crCone.GetPosition().GetOriginRef()
                         + crCone.GetSTEPUVDomain().GetVMin() * sConeAxis;
  SmPoint3d        sCenterTop    = rCenterBot + dHeight * sConeAxis;

  // map given NurbDomain into StepDomain - m_bInsideOut swaps Nurb GenCurve Min/Max Points
  SmExtent2d       sAnalUVDomain ;
  crCone.ConvertDomainFromNURBSToSTEP(crUVDomain, sAnalUVDomain) ;
  SmExtent1d       sSweepIvl     = sAnalUVDomain.GetUInterval() ;
  //SmBoolean        bIsClosed     = sSweepIvl.IsClosed(360.0) ;  //unused
  //double           dStartAng     = sSweepIvl.GetMin();        //unused
  //double           dEndAng       = sSweepIvl.GetMax();        //unused

  // get cone corners [StartLine start, StartLine end, EndLine start EndLine end]
  SmPoint3d sCorners[4] ;
  crCone.EvaluateSTEPPoint(sAnalUVDomain.Evaluate(0.0, 0.0), sCorners[0]) ;
  crCone.EvaluateSTEPPoint(sAnalUVDomain.Evaluate(0.0, 1.0), sCorners[1]) ;
  crCone.EvaluateSTEPPoint(sAnalUVDomain.Evaluate(1.0, 0.0), sCorners[2]) ;
  crCone.EvaluateSTEPPoint(sAnalUVDomain.Evaluate(1.0, 1.0), sCorners[3]) ;
  SmVector3d sStartLineVec = sCorners[1] - sCorners[0] ;
  SmVector3d sEndLineVec   = sCorners[3] - sCorners[2] ;

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
static ULONG lCount      = 1 ; lCount++ ;
static ULONG lDebugCount = 0 ;
  if (bDebugMe || lCount == lDebugCount) 
    {
      SmBSplineCurve *pCirc1 = NULL ; 
      if(SM_IS_ZERO(crCone.GetTopRadius()))
           { SmBSplineCurve::CreateDegenerateCurve(crContext, 3, sCenterTop, pCirc1) ; }
      else { pCirc1 = new (crContext) SmCircle(sCenterTop, 
                                               crCone.GetPosition().GetXAxisRef(), 
                                               crCone.GetPosition().GetYAxisRef(),
                                               sSweepIvl, crCone.GetTopRadius(), 3, &crContext) ;
           }
      SmObjDelete sClean1(pCirc1) ;

      SmBSplineCurve *pCirc2 = NULL ; 
      if(SM_IS_ZERO(crCone.GetBotRadius()))
           { SmBSplineCurve::CreateDegenerateCurve(crContext, 3, rCenterBot, pCirc2) ; }
      else { pCirc2 = new (crContext) SmCircle(rCenterBot, 
                                               crCone.GetPosition().GetXAxisRef(), 
                                               crCone.GetPosition().GetYAxisRef(),
                                               sSweepIvl, crCone.GetBotRadius(), 3, &crContext) ;
           }
      SmObjDelete sClean2(pCirc2) ;
      SmFace *pFace = (SmFace *)crCone.GetFace() ;
      SmBrep *pBrep = pFace ? pFace->GetBrep() : NULL ;
      SmTArray<SmExtent1d> sEllipseNurbIvl ;
      sFullEllipse.ConvertIvlFromSTEPToNURBS(sEllipseIvl, sEllipseNurbIvl) ;

      smgfx_Erase();
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) { pBrep->Draw(TRUE) ; } sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,1) ; crCone.DrawUV() ; sm_GraphicsLoop() ;
      smgfx_SetLook(3,4, 1,0,0) ; sFullEllipse.Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook(6,7, 0,0,0) ; for(ii=0;ii<sEllipseNurbIvl.GetSize();ii++)
                                    { sFullEllipse.Draw(&sEllipseNurbIvl[ii]) ; sm_GraphicsLoop() ; }
      smgfx_SetLook(3,4, 0,1,0) ; (dEllipseXRadius * rEllipseXAxis).Draw(&rEllipseOrigin) ; sm_GraphicsLoop() ;
      smgfx_SetLook(3,4, 0,0,1) ; (dEllipseYRadius * rEllipseYAxis).Draw(&rEllipseOrigin) ; sm_GraphicsLoop() ;
      smgfx_SetLook(5,6, 1,1,0) ; sStartLineVec.Draw(&sCorners[0]) ; sm_GraphicsLoop() ;
      smgfx_SetLook(5,6, 1,1,0) ; sEndLineVec.Draw(&sCorners[2]) ; sm_GraphicsLoop() ;
      smgfx_SetLook(5,6, 1,1,0) ; pCirc1->Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook(5,6, 1,1,0) ; pCirc2->Draw() ; sm_GraphicsLoop() ;
      sm_GraphicsLoop();
    }
#endif

  // get ellipse/TopCircle(max 2), ellipse/BotCircle(max 2) 
  //     ellipse/StartSeamLine(max 1), ellipse/EndSeamLine(max 1) intersections
  ULONG lNumTsect1, lNumTsect2 ;
  SmPoint3d sTsectPoints[6] ;
  double dParam[2] ;
  smgu_CirclePlaneIntersect(dTol, 
                            crCone.GetTopRadius(), sCenterTop, sConeAxis,
                            rEllipseOrigin, sEllipseZ, 
                            lNumTsect1, sTsectPoints) ;
  smgu_CirclePlaneIntersect(dTol, 
                            crCone.GetBotRadius(), rCenterBot, sConeAxis,
                            rEllipseOrigin, sEllipseZ, 
                            lNumTsect2, sTsectPoints+2) ;
  smgu_LinePlaneIntersect  (sCorners[0], sStartLineVec,
                            rEllipseOrigin, sEllipseZ,
                            dParam[0]) ;
  smgu_LinePlaneIntersect  (sCorners[2], sEndLineVec,
                            rEllipseOrigin, sEllipseZ,
                            dParam[1]) ;

  // Check for degenerate solutions, within tol. [B10]
  // (smgu_CirclePlaneIntersect() doesn't do that.)
  if ( lNumTsect1 == 2 )
  {
      if ( sTsectPoints[0].DistanceBetween( sTsectPoints[1] ) < dTol )
        { lNumTsect1 = 1; }
  }
  if ( lNumTsect2 == 2 )
  {
      if ( sTsectPoints[2].DistanceBetween( sTsectPoints[3] ) < dTol )
        { lNumTsect2 = 1; }
  }

  // circle/Plane xSect lNumTSect == 0 is a miss - nothing to do
  //                    lNumTSect == 1 is a tangent point - not put on in/out point list
  //                    lNumTSect == 2 is a trimmed section in/out boundary
  //                    lNumTSect == 3 is a coincident circle/plane 
  SM_ASSERT(   lNumTsect1 != 3 || lNumTsect2 != 3) ;
          
  // get ellipse angle in degrees for each XSectPoint and place in the in/out dXSectAngle array
  // exclude intersection points outside the cone sweep trim boundaries

  // Consider all 4 possible ConeCircle/Ellipse intersections for addition to dXSectAngle array
  SmPoint2d sConeSTEPUV ;
  SmLocationType eLoc ;
  for(ii=0;ii<4;ii++)
    {
      // only consider circle/ellipse intersections when there are two of them
      //      i.e. don't add point tangent intersections - they don't affect in/out behavior.
      //           don't add coincident intersections - they get trimmed to cone end lines later.
      // only add intersection points in the cone trim boundary
      if(   (ii < 2 && lNumTsect1 == 2)
         || (ii >=2 && lNumTsect2 == 2))
        {
          // map XSectPoint to ConeUV point
          crCone.STEPInversion(sAnalUVDomain, sTsectPoints[ii], dTol, sConeSTEPUV, eLoc) ;

          // when ConeUV is cone trim boundaries
          //      skip circle instersections on seams because the line intersections will catch seam intersections
          if(   eLoc == SM_LT_INTERIOR
             && eLoc != SM_LT_U_SEAM)
            {
              // add ellipse angleDeg for intersection to in/out angle dXSectAngle array
              sFullEllipse.STEPInversion(sTsectPoints[ii], dXSectAngle[lCnt]) ;
              lCnt++ ;
            }
        } // end skip tangent intersection cases
    } // end iter every possible Circle/Ellipse xSect

  // for StartSeamLine/ellipse xSect
  //  GWC: tolerances to think about:
  //    ContainsPeriodicValue()  increased tolerance to account for common degree domains ;
  //        to double dScaledZero = SM_EFF_ZERO * 100.0 * (1.0 + dPeriod) ;
  //        and here we use a different value for lines
  //    This computation needs both functions to have the same behavior near corners.
  double dScaledZero = SM_EFF_ZERO * (1.0 + smos_Max(sCorners[0].GetMaxDimension(),
                                                     sCorners[3].GetMaxDimension())) ;
  if(   (   lNumTsect1 == 3 || lNumTsect2 == 3)
     || (   dParam[0] >= 0.0 - dScaledZero
         && dParam[0] <= 1.0 + dScaledZero))
    {
      sTsectPoints[4] = sCorners[0] + dParam[0] * sStartLineVec;
      sFullEllipse.STEPInversion( sTsectPoints[4], dXSectAngle[lCnt] );
      lCnt += 1 ;
    }

  // for EndSeamLine/ellipse xsect
  if(   (   lNumTsect1 == 3 || lNumTsect2 == 3)
     || (   dParam[1] >= 0.0 - dScaledZero
         && dParam[1] <= 1.0 + dScaledZero))
    {
      sTsectPoints[5] = sCorners[2] + dParam[1] * sEndLineVec;
      sFullEllipse.STEPInversion( sTsectPoints[5], dXSectAngle[lCnt] );
      lCnt += 1 ;
    }

  // there should be no more than 6 intersections
  SM_ASSERT(lCnt <= 6) ;

  // arrive here when dXSectAngle array contains an ellipse angle for
  // every ConeTrimCurve/FullEllipse intersection where the FullEllipse goes in/out
  // of the ConeTrimDomain.  There should be an even number of these transitions
  // The ellipse trim range has not yet been applied.  When there are no intersections
  // the whole curve must lie outside the cone trim domain.
  if(lCnt < 2) 
    { return ; }

  // when there are intersections there should be an even number of them
  // unless ellipse intersects a corner exactly.
  //      SM_ASSERT(lCnt % 2 == 0) ;  

  // order the XSectPoints by angle
  smgu_ShellSort_double_array(dXSectAngle.GetDataArray(), lCnt) ;
  SM_ASSERT(dXSectAngle[lCnt-1] - dXSectAngle[0] <= 360.0) ;

  // now classify the segment midPoints as in/out
  // add one interval to output for every inside segment
  double dScaledAngZero = SM_EFF_ZERO * 100.0 * 360.0 ;
  ULONG iThis, iNext, iPrev;
  for( iThis=0; iThis<lCnt; iThis++ )
    {
      iNext = (iThis+1) % lCnt;

      // get interval start/stop angles
      double dStartAngDeg = dXSectAngle[iThis] ;
      double dEndAngDeg   = dXSectAngle[iNext] ;

      // the last interval includes the zero angle
      // modify the startPoint so that the interval is
      // within [-360, 360] and Start < End
      if ( iNext == 0 ) { dStartAngDeg -= 360.0; }

      // mark degenerate intervals as unknown for now - they will be classified
      // based on their neighbor classifications once they are available
      if((dEndAngDeg - dStartAngDeg < dScaledAngZero))
        { bInside[iThis] = UNSURE ; 
          continue ;
        }

      // classify nonDegenerate interval midPoint
      SmPoint3d sSegmentMidPoint ;
      double sSegmentMidAngDeg = (dEndAngDeg + dStartAngDeg) / 2.0 ;
      sFullEllipse.EvaluateSTEPPoint(sSegmentMidAngDeg, sSegmentMidPoint) ;
      crCone.STEPInversion(sAnalUVDomain, sSegmentMidPoint, dTol, sConeSTEPUV, eLoc) ;

      // Remember if segment MidPointUV is within cone trim boundaries
      bInside[iThis] = (eLoc == SM_LT_EXTERIOR) ? FALSE : TRUE ;

    } // end iter every interval to classify it

  // add one interval to output for every inside segment
  SmExtent1d sUnTrimmedIvl ;
  SmTArray<SmExtent1d> sTrimIvls ;

  for ( iThis=0; iThis<lCnt; iThis++ )
    {
      iNext = ( iThis+1 )      % lCnt;
      iPrev = ( iThis+lCnt-1 ) % lCnt;

      // get interval start/stop angles
      //   the last interval includes the zero angle
      //   modify the startPoint so that the interval is
      //   within [-360, 360] and Start < End.
      double dStartAngDeg = (iNext == 0) ? dXSectAngle[iThis] - 360.0
                                         : dXSectAngle[iThis] ;
      double dEndAngDeg   = dXSectAngle[iNext] ;

      // when the segment is inside
      if(   bInside[iThis] == TRUE   // nonDegnerate Ivl classification
         || (   bInside[iThis] == UNSURE  // Degenerate IVl is inside
             && bInside[iPrev] == FALSE   // when both neighbor intervals 
             && bInside[iNext] == FALSE)) //      are outside
        {
          // make and trim interval to ellipse interval
          // (Don't use SetMinMax, start ang can be > end ang, by machine noise.
          //   SetMinMax checks '>' without tol, and causes trouble if bad.)
          sUnTrimmedIvl.Init();
          sUnTrimmedIvl.AddValue( dStartAngDeg );
          sUnTrimmedIvl.AddValue( dEndAngDeg );

          sEllipseIvl.IntersectPeriodic(sUnTrimmedIvl, 360.0, sTrimIvls, FALSE) ;

          // add every trim invl to output
          rIvls.Append(sTrimIvls) ;                

        } // end segment inside check

    } // end iter every ellipse segment
  
  // all done

#ifdef SM_DEBUG_CODE
  if(bDebugMe || lDebugCount == lCount)
    {
      dXSectAngle.Dump() ;
      rIvls.Dump() ;

      // Build intersection ellipse - get face and Brep pointers
      SmPoint3d sPoint ;
      SmFace *pThisFace  = ((SmFace *)crCone.GetFace()) ;
      SmBrep *pThisBrep  = pThisFace  ? pThisFace->GetBrep()  : NULL ;

      // draw Breps(Blue,Green), Surfaces(cyan,yellow), faces(black), 
      // ellipse(red), TrimPoints(blue)
      smgfx_Erase() ; 
      smgfx_SetLook(1,2, 0,0,1) ; if(pThisBrep)  { pThisBrep->Draw(TRUE) ; }  sm_GraphicsLoop() ; 
      smgfx_SetLook(1,2, 0,1,1) ; crCone.DrawUV() ; sm_GraphicsLoop() ;
      smgfx_SetLook(3,4, 0,1,0) ; (dEllipseXRadius * rEllipseXAxis).Draw(&rEllipseOrigin) ; sm_GraphicsLoop() ;
      smgfx_SetLook(3,4, 0,0,1) ; (dEllipseYRadius * rEllipseYAxis).Draw(&rEllipseOrigin) ; sm_GraphicsLoop() ;
      smgfx_SetLook(6,7, 0,0,1) ; 

      for(ii=0;ii<lCnt;ii++) { sFullEllipse.EvaluateSTEPPoint(dXSectAngle[ii], sPoint) ;
                               smgfx_SetLook(8,9, 0,0,1) ; sPoint.Draw() ; sm_GraphicsLoop() ;
                             }
      for(ii=0;ii<4;ii++) { if(   (ii < 2 && lNumTsect1 == 2)
                               || (ii >=2 && lNumTsect2 == 2)
                               || (ii >=4))
                              { smgfx_SetLook(10,11, 1,0,0) ; sTsectPoints[ii].Draw() ; sm_GraphicsLoop() ; }
                          }
      smgfx_SetLook(10,11, 1,0,0) ; (sCorners[0] + dParam[0] * sStartLineVec).Draw() ; sm_GraphicsLoop() ; 
      smgfx_SetLook(10,11, 1,0,0) ; (sCorners[2] + dParam[1] * sEndLineVec).Draw() ; sm_GraphicsLoop() ;  
      for(ii=0;ii<rIvls.GetSize();ii++) { 
                                          ULONG jj;
                                          SmExtent1d sStepIvl = rIvls[ii] ;
                                          SmTArray<SmExtent1d> sNurbIvl ;
                                          sFullEllipse.ConvertIvlFromSTEPToNURBS(sStepIvl, sNurbIvl) ;
                                          for(jj=0;jj<sNurbIvl.GetSize();jj++)
                                            { smgfx_SetLook(8,9, 0,1,0) ; sFullEllipse.Draw(&sNurbIvl[jj]) ; sm_GraphicsLoop() ; }
                                        }
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

} // end sm_GetEllipseTrimIntervals

/*******************************************************************//**
PURPOSE: Trim an intersection ellipse known to be on two cones
    to the trim boundaries of those cones and turn each trimmed
    segment into an ellipse curve and add to pOpt3DCurves.

NOTES:
***********************************************************************/
void sm_AddEllipseIntervalsTrimmedToTwoCones
  (const SmContext            & crContext,              // in : for temp object construction
   const SmPoint3d            & rEllipseOrigin,         // in : origin            of ellipse
   const SmVector3d           & rEllipseXAxis,          // in : xAxis             of ellipse
   const SmVector3d           & rEllipseYAxis,          // in : yAxis             of ellipse
   double                       dEllipseXRadius,        // in : x radius          of ellipse
   double                       dEllipseYRadius,        // in : y radius          of ellipse
   double                       dEllipseStartAngDeg,    // in : start angle (deg) of ellipse
   double                       dEllipseEndAngDeg,      // in : end angle (deg)   of ellipse
   const SmCone               & crCone1,                // in : Cone1 with trim boundaries
   const SmExtent2d           & crUVDomain1,            // in : nurb trim boundaries for cone1
   const SmCone               & crCone2,                // in : Cone2 with trim boundaries
   const SmExtent2d           & crUVDomain2,            // in : nurb trim boundaries for cone2
   double                       dTol,                   // in : approximation tolerance
   SmTArray<SmCurve*>         * pOpt3DCurves,           // out: accumulate all nonDegnerate intersections trimmed to both cones                    
   SmTArray<SmBSplineCurve *> & rDegenerateCurves)      // out: accumulate all unique Degnerate intersections (they may coincide with pOpt3DCurve) 
{
  // init outputs - nothing to do because both outputs accumuate

  // locals
  SmTArray<SmExtent1d> sThisIvls, sOtherIvls, sXSectIvls ;
  double dScaledAngZero = SM_EFF_ZERO * 100.0 * (1.0 + 360.0) ;

  // get ellipse intervals trimmed to 1st cone
  sm_GetEllipseTrimIntervals
    (crContext,                  // in : for temp object construction
     rEllipseOrigin,             // in : origin            of ellipse
     rEllipseXAxis,              // in : xAxis             of ellipse
     rEllipseYAxis,              // in : yAxis             of ellipse
     dEllipseXRadius,            // in : x radius          of ellipse
     dEllipseYRadius,            // in : y radius          of ellipse
     dEllipseStartAngDeg,        // in : start angle (deg) of ellipse
     dEllipseEndAngDeg,          // in : end angle (deg)   of ellipse
     crCone1,                    // in : Cone with trim boundaries
     crUVDomain1,                // in : nurb trim boundaries for cone
     dTol,                       // in : approximation tolerance
     sThisIvls) ;                // out: array of ellipse intervals inside trimmed cone

  // get ellipse intervals trimmed to second cone
  sm_GetEllipseTrimIntervals
    (crContext,                  // in : for temp object construction
     rEllipseOrigin,             // in : origin            of ellipse
     rEllipseXAxis,              // in : xAxis             of ellipse
     rEllipseYAxis,              // in : yAxis             of ellipse
     dEllipseXRadius,            // in : x radius          of ellipse
     dEllipseYRadius,            // in : y radius          of ellipse
     dEllipseStartAngDeg,        // in : start angle (deg) of ellipse
     dEllipseEndAngDeg,          // in : end angle (deg)   of ellipse
     crCone2,                    // in : Cone with trim boundaries
     crUVDomain2,                // in : nurb trim boundaries for cone
     dTol,                       // in : approximation tolerance
     sOtherIvls) ;               // out: array of ellipse intervals inside trimmed cone

  // for every thisCone/OtherCone trimmed interval combination
  ULONG ii, jj, kk, ll ;
  for(ii=0;ii<sThisIvls.GetSize();ii++)
    {
      SmExtent1d sThisIvl = sThisIvls[ii] ;

      // for every otherCone trimmed interval 
      for(jj=0;jj<sOtherIvls.GetSize();jj++)
        { 
          SmExtent1d sOtherIvl = sOtherIvls[jj] ;

          // find intersection of two intervals - the ellipse arc in both cones
          sThisIvl.IntersectPeriodic(sOtherIvl, 360.0, sXSectIvls) ; 

          // build and output all trimmed interval intersections
          for(kk=0;kk<sXSectIvls.GetSize();kk++)
            {
              SmExtent1d sIvl = sXSectIvls[kk] ;

              // for degenerate curves
              if(sIvl.GetLength() < dScaledAngZero)
                { 
                  // get ellipse point
                  double dAngleDeg = sIvl.GetMin() ;
                  double dCos = smos_Cosine(dAngleDeg * SM_PI / 180.0) ;
                  double dSin = smos_Sine  (dAngleDeg * SM_PI / 180.0) ;
                  SmPoint3d sEllipsePoint =   rEllipseOrigin
                                          + dEllipseXRadius * dCos * rEllipseXAxis
                                          + rEllipseYAxis   * dSin * dEllipseYRadius ;

                  // see if point is already a degenerate curve
                  SmBoolean bIsUnique = TRUE ;
                  for(ll=0;bIsUnique && ll<rDegenerateCurves.GetSize();ll++)
                    {
                      // get curve/point distance
                      SmBSplineCurve *pDegenerateCurve = rDegenerateCurves[ll] ;
                      SmExtent1d sInt = pDegenerateCurve->GetNaturalInterval() ;
                      SmPoint3d sDegeneratePoint ;
                      pDegenerateCurve->EvaluatePoint( sInt.GetMin(), sDegeneratePoint) ;

                      double dDist = sDegeneratePoint.DistanceBetween(sEllipsePoint) ;
                      double dScaledZero = SM_EFF_ZERO * 100.0 * (1.0 * sEllipsePoint.GetMaxDimension()) ;
                      if(dDist < dScaledZero)
                        { bIsUnique = FALSE ; }

                    } // end degenerate duplicate check

                  // see if point is coincident with a solution
                  for(ll=0;bIsUnique && pOpt3DCurves && ll<pOpt3DCurves->GetSize();ll++)
                    {                      
                      SmEllipse *pCurve = (SmEllipse *)pOpt3DCurves->GetAt(ll) ;
                      SM_ASSERT(pCurve->IsKindOf(SmEllipse_TYPE)) ;

                      // map Point to Ellipse
                      double dAnalParam ;
                      SmCurveLocationType eLoc ;
                      pCurve->STEPInversion(sEllipsePoint, dAnalParam, &eLoc) ;

                      // when point is on any existing curve - its not unique
                      if(eLoc != SM_CL_EXTERIOR)
                        { bIsUnique = FALSE ; }

                    } // end coincident degenerate point check

                  // create and save a unique degenerate interval
                  if(bIsUnique)
                    {
                      SmBSplineCurve *pCrv = NULL ;
                      SE(SmBSplineCurve::CreateDegenerateCurve(crContext, 3,
                                                               sEllipsePoint,
                                                               pCrv));
                      rDegenerateCurves.Add(pCrv);
                    } // end adding unique degenerate curve

                }  // end degenerate interval branch
              else // build an ellipse arc
                {
                  // build and output an ellipse arc
                  SmEllipse *pEll = new (crContext) SmEllipse(rEllipseOrigin,
                                                              rEllipseXAxis,
                                                              rEllipseYAxis,
                                                              sIvl, dEllipseXRadius, dEllipseYRadius) ;
                  NE(pEll);
                  pOpt3DCurves->Add(pEll);
                } // end build ellipse arc branch

            } // end iter every ellipse interval trimmed to both this and other cone
        } // end iter every ellipse interval trimmed to other cone
    } // end iter every ellipse interval trimmed to this cone
} // end sm_AddEllipseIntervalsTrimmedToTwoCones

/*******************************************************************//**
PURPOSE: AssertValid() Test reporting static labels
***********************************************************************/
SmAssertReportLabel sAssertCone_list[] =
{
 /*  0 */ {SM_AT_RADIUS,           _T("Bad Cone Radius"),             _T("TopRadius or BotRadius is negative. Both must be >= 0.0 to ensure the apex is not within the domain)") },
 /*  1 */ {SM_AT_PARAMETERIZATION, _T("Bad Cone Parameterization"),   _T("GenCurve not the same as the Surface Parameterization in GenCurve direction") },
 /*  2 */ {SM_AT_PARAMETERIZATION, _T("Bad Cone Parameterization"),   _T("make sure cone is parameterized in height of cone axis ") },
 /*  3 */ {SM_AT_ANGLE,            _T("Bad Cone Sweep Direction"),    _T("Nurb sweep direction is not CCW about STEP Z axis ") },
 /*  4 */ {SM_AT_TYPE,             _T("Bad Cone GenCurve type"),      _T("GenCurve is not a SmLine type object. It must be. ") },
 /*  5 */ {SM_AT_DISTANCE,         _T("Bad Cone GenCurve LineScale"), _T("GenCurve Line Scale != 1/Cos(m_dSemiAngleRad). It must, to parameterize GenCurve to the cone's height.") },
 /*  6 */ {SM_AT_GEOMETRIC,        _T("Bad Cone Pole"),               _T("Cone apex (where radius == 0.0) is within the Cone's AnalIvlV placing a pole on the surface interior.") }
} ;

/*******************************************************************//**
PURPOSE: Make sure GenCurve and Surface parameterizations are
            compatible.

RETURNS ---  TRUE  = OK
             FALSE = Problem
***********************************************************************/
SmBoolean SmCone::AssertValid
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

  // positive radii
  double dTopRadius = GetTopRadius() ;
  double dBotRadius = GetBotRadius() ;

  bRtn &= SM_ASSERT_VALUE_REPORT(0, SM_LEVEL_0, (dTopRadius >= -SM_EFF_ZERO && dBotRadius >= -SM_EFF_ZERO), SM_EFF_ZERO, smos_Min(dTopRadius,dBotRadius), _T("")) ;

  // locals
  const SmPoint3d &rConeOrig  = m_vPosition.GetOriginRef() ;
  SmVector3d       sConeZAxis = m_vPosition.GetZAxis() ;

  // check temporary genCurve against parameterization of NurbSurface
  const SmContext &crContext = *GetContext() ;
  SmBSplineCurve  *pGenCurve = NULL ;
  CreateGeneratorFromAngle(crContext, 0.0, pGenCurve) ;
  SmObjDelete sClean(pGenCurve) ;

  // GenCurve AnalInterval   must equal Surface->AnalUVDomain->GetVInterval,
  // GenCurve NURBInterval   must equal m_bSwapUV ? Surface->NURBUInterval() : Surface->NURBVInterval()
  // GenCurve NURBKnotVector must equal m_bSwapUV ? Surface->NURBKnotU : Surface->NURBKnotV
  bRtn &= SM_ASSERT_VALUE_REPORT(1, SM_LEVEL_0, 
                                (HasSameParameterization(SM_SP_V,        // in : Analytic Domain Direction
                                                           m_bSwapUV     // in : Nurb     Domain Direction
                                                         ? SM_SP_U 
                                                         : SM_SP_V,
                                                         pGenCurve) ), 
                                 SM_EFF_ZERO,
                                 SM_UNDEF_DOUBLE, 
                                 _T("") ) ;
  // No longer necessary: v-interval can now be anything.
//  // make sure cone is parameterized in height of cone axis
//  SmExtent1d sLinearIvl = m_vAnalUVDomain.GetVInterval() ;
//  double     dHeight    = GetHeight() ;
//
//  bRtn &= SM_ASSERT_VALUE_REPORT(2, SM_LEVEL_0, 
//                                 (   SM_ARE_SAME(sLinearIvl.GetMin(), -SM_INFINITE_PARAMETER)
//                                  || SM_ARE_SAME(sLinearIvl.GetMax(),  SM_INFINITE_PARAMETER)
//                                  || (   SM_ARE_SAME(sLinearIvl.GetMin(), 0.0)
//                                      && SM_ARE_SAME(sLinearIvl.GetMax(), dHeight)) ), 
//                                 SM_EFF_ZERO,
//                                 SM_UNDEF_DOUBLE, 
//                                 _T("") ) ;

  // GenCurve must be a line
  SmLine *pLine = SM_CAST_PTR(SmLine, m_pGenCurve) ;
  bRtn &= SM_ASSERT_BOOLEAN_REPORT(4, SM_LEVEL_0, pLine != NULL, _T("")) ;

  // GenCurve line length must be scaled by the Cone SemiAngle
  if(pLine)
    {
      double dLineScale  = pLine->GetLineScale() ;
      double dConeScale  = 1.0/smos_Cosine(SM_DEG2RAD(m_dSemiAngleDeg)) ;
      double dTol        = (1 + dLineScale) * SM_EFF_ZERO ;
      bRtn &= SM_ASSERT_VALUE_REPORT(5, SM_LEVEL_0, SM_ARE_SAME_TO_TOL(dLineScale, dConeScale, dTol), dTol, dLineScale - dConeScale, _T("") ) ;

    } // end pLine existence check

  // Cone apex cannot be within cone domain
  SmExtent1d sAnalIvlV = m_vAnalUVDomain.GetVInterval() ;
  SmBoolean bIsCylinder ;
  SmPoint3d sApexPoint ;
  double    dApexParam ;
  GetApex(bIsCylinder, sApexPoint, dApexParam) ;
  bRtn &= SM_ASSERT_VALUE_REPORT(6, SM_LEVEL_0, 
                                (    bIsCylinder
                                 || !sAnalIvlV.ContainsValue    (dApexParam, SM_EFF_ZERO)
                                 ||  sAnalIvlV.IsValueOnBoundary(dApexParam, SM_EFF_ZERO)),
                                 SM_EFF_ZERO,
                                 SM_UNDEF_DOUBLE, 
                                 _T("") ) ;


  // make sure Nurb sweep direction is CCW about STEP Z axis
  // compute sweep Z = (PointOnSurf - ProjOnZAxis) * SweepDir
  // make sure sweep Z is in same direction as ConeZAxis
  // - use cone NearMidPoint which should never be an apex point.
  SmPoint2d sNurbUV = GetNaturalUVDomain().Evaluate(.45678,.45678) ;
  SmVector3d sDerivs[4] ; // [D, Dv, Du]
  Evaluate(sNurbUV, 1, 1, TRUE, TRUE, TRUE, sDerivs) ;
  SmVector3d sSweepDir = m_bSwapUV ? sDerivs[1] : sDerivs[2] ;
  SmPoint3d  sOnAxis   = rConeOrig + (sDerivs[0] - rConeOrig).Dot(sConeZAxis) * sConeZAxis ;
  SmPoint3d  sSweepZ   = (sDerivs[0] - sOnAxis) * sSweepDir ;
  bRtn &= SM_ASSERT_VALUE_REPORT(3, SM_LEVEL_0, (sSweepZ.Dot(sConeZAxis) > 0.0), 0.0, sSweepZ.Dot(sConeZAxis), _T("")) ;
  
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
  // SM_ASSERT(bRtn);
  return(bRtn) ;

} // end SmCone::AssertValid

// obsolete
// /*******************************************************************//**
// PURPOSE: Fix Assert Report failures reported by an AssertValid call
// 
// RETURNS ---  TRUE  = OK
//              FALSE = Problem
// ***********************************************************************/
// SmBoolean SmCone::AssertHeal
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
//                rAReport.m_pHealMessage = _T("SmCone::AssertHeal fix not yet supported") ;  
//                 
//     }
// 
//   // all done
//   return(bRtn) ;
// 
// } // end SmCone::AssertHeal
// end obsolete

/*******************************************************************//**
PURPOSE: Write SmCone to given output stream.

NOTES: 
***********************************************************************/
SmStatus SmCone::WriteToDB
 (SmDatabaseIO & rDB,                // in : target output stream
  ULONG          lDBVersionNumber)   // in : database version to get proper sequence of writes                                                                       
 const
{
  // file type, ASCII or BINARY
  SmFileType      eType    =  rDB.GetFileType();
  std::ostream  & rFileOut = *rDB.GetOutStreamPtr();
  
  if (eType == SM_ASCII) 
    {
      rFileOut << m_dBaseRadius    << " SmCone Radius of the cone in the plane passing through origin \n" ;
      rFileOut << m_dSemiAngleDeg  << " SmCone SemiAngleDeg between -90 and 90 \n" ;
      rFileOut << m_bInsideOut     << " SmCone InsideOutFlag: FALSE= m_vPosition.ZAxis points from bot to top of Nurb, TRUE=Inverted \n";
    }
  else 
    {
      SER(rDB.WriteDouble ( m_dBaseRadius)) ;  
      SER(rDB.WriteDouble ( m_dSemiAngleDeg)) ;
      SER(rDB.WriteBoolean( m_bInsideOut)) ;   
    }

  // output the parent
  SER(SmSurfOfRevolution::WriteToDB(rDB, lDBVersionNumber)) ; 

  // all done
  return SM_SUCCESS;

} // end SmCone::WriteToDB

/*******************************************************************//**
PURPOSE: static method to Read a SmCone from a given stream  

NOTES: 
***********************************************************************/
SmStatus SmCone::ReadFromDB
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
         || rpNewSurface->IsKindOf(SmCone_TYPE)) ? SM_SUCCESS : SM_ERR) ;

  // init output object
  SmCone *pCone =   (rpNewSurface == NULL)
                  ? new (crContext) SmCone()
                  : (SmCone *)rpNewSurface ;

  // file type
  SmFileType     eType   =  rDB.GetFileType();
  std::istream & rFileIn = *rDB.GetInStreamPtr();
      
  if (eType == SM_ASCII) 
    {
      rFileIn >> pCone->m_dBaseRadius ;     rDB.GoToNextLine() ;
      rFileIn >> pCone->m_dSemiAngleDeg ;   rDB.GoToNextLine() ;
      rFileIn >> pCone->m_bInsideOut ;      rDB.GoToNextLine() ;
    }
  else 
    {
      SER(rDB.ReadDouble (pCone->m_dBaseRadius)) ;  
      SER(rDB.ReadDouble (pCone->m_dSemiAngleDeg)) ;
      SER(rDB.ReadBoolean(pCone->m_bInsideOut)) ;   
    }

  // read the parent object
  SmSurface *pSurface = pCone ; 
  SER(SmSurfOfRevolution::ReadFromDB(SmSurfOfRevolution_TYPE, rDB, crContext, pSurface, lDBVersionNumber)) ;

  // all done
  rpNewSurface = pCone ; 
  return SM_SUCCESS;

} // end SmCone::ReadFromDB

/*******************************************************************//**
PURPOSE:
NOTES:
***********************************************************************/
SmBoolean SmCone::IsKindOf( SM_TYPE t ) const
{
  return ((SmCone_TYPE == t) ? TRUE : SmSurfOfRevolution::IsKindOf( (t) ));
}

/*******************************************************************//**
PURPOSE: Dump Cone surface data out for debugging.

NOTES: 
***********************************************************************/
void SmCone::Dump
  (void)
 const
{
  SmCone::Dump(FALSE);
}

/*******************************************************************//**
PURPOSE: Dump Cone surface data out for debugging.

NOTES: 
***********************************************************************/
void SmCone::Dump
  (SmBoolean bAbbrev)    // in : unused 
 const
{
  SM_REF1(bAbbrev) ; 
  TCHAR sBuff[SM_TBLOCK_SIZE], sBuffForFile[SM_TBLOCK_SIZE];
  smos_WriteBuffer(_T("\nBegin SmCone::Dump()")) ;

  smos_sprintf(sBuff,       _T("\nSmCone = 0x%p"), this) ;
  smos_sprintf(sBuffForFile,_T("\nSmCone = %s"), _T("notNULL")) ;
  smos_WriteBuffer(sBuff, sBuffForFile);

  smos_WriteBuffer(_T("\n  AnalUVDomain  = ")) ; m_vAnalUVDomain.Dump();
  if(m_pNurb) { smos_WriteBuffer(_T("  NurbDomain    = ")) ; GetNaturalUVDomain().Dump() ; }
  else        { smos_WriteBuffer(_T("  NurbDomain    = Undefined - No underlying m_pNurb Surface\n")) ; }

  smos_sprintf(sBuff,_T("  Base Radius = %16.16lf, bSwap_UV   = %s\n"), GetBotRadius(), m_bSwapUV ? _T("TRUE") : _T("FALSE")) ;
  smos_WriteBuffer(sBuff) ;
  smos_sprintf(sBuff,_T("  Top  Radius = %16.16lf, bInsideOut = %s\n"), GetTopRadius(), m_bInsideOut ? _T("TRUE") : _T("FALSE")) ;
  smos_WriteBuffer(sBuff) ;
  smos_sprintf(sBuff,_T("  SemiAngle Deg = %16.16lf  where SemiAng: 0.0 = Cylinder\n"),  m_dSemiAngleDeg) ;
  smos_WriteBuffer(sBuff) ;
  smos_WriteBuffer(_T("                                                     pos = BotCircRad < TopCircRad\n")) ;
  smos_WriteBuffer(_T("                                                     neg = BotCircRad > TopCircRad")) ;

  SmSurfOfRevolution::Dump();
  smos_WriteBuffer(_T(" End SmCone::Dump()\n")) ;

} // end SmCone::Dump 
