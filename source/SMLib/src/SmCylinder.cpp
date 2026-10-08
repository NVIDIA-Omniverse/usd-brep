// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************//**
* FILE NAME --- SmCylinder.cpp
* PURPOSE: Implementation of SmCylinder surface  methods.
**********************************************************************/

  
#include "StdAfx.h"

#include <SmCylinder.h>
#include <SmLine.h>
#include <SmNurbsSrf.h>
#include <SmGraphicsExtern.h>

#include <SmAssertArray.h>
#include <SmDatabaseIO.h>
#ifdef SM_DEBUG_CODE
#include <SmBrep.h>
#include <SmFace.h>
#endif

/*******************************************************************//**
PURPOSE: Create a bounded Cylinder

NOTES: 
  - most users use CreateCanonical() 
  - when bSwapUV==FALSE and bInsideOUt==FALSE
    - cylinder BotCircle centered on crOrigin.
    - u-parameterization (rotational): [ dStartRotAngle, dEndRogAngle ]
    - v-parameterization (along axis): [ 0, dHeight ]
    - Cylinder SrfNormal points from SrfPt away from Cylinder axis
***********************************************************************/
SmCylinder::SmCylinder
  (const SmPoint3d  & crOrigin,           // zero Point on Cylinder's ZAxis
   const SmVector3d & crXAxis,            // Cylinder's X Axis (vector perp to axis to cylinder start point)
   const SmVector3d & crYAxis,            // Cylinder's Y Axis (vector perp to axis and X Axis)
   double             dRadius,            // Cylinder's radius (distance from axis to cylinder wall)
   double             dStartAngle,        // CCW about Z from X, rangeDeg:[-360 to 360] maxLength = 360
   double             dEndAngle,          // CCW about Z from X, rangeDeg:[-360 to 360] maxLength = 360
   double             dHeight,            // Defined domain from zero point in Z direction 
   SmBoolean          bSwapUV,            // in : TRUE = underlying NURB surface v dir maps to rotation direction
                                          //      FALSE= underlying NURB surface u dir maps to rotation direction
   SmBoolean          bInsideOut,         // in : TRUE = underlying NURB linear dir runs from TopCircle to BotCircle
                                          //      FALSE= underlying NURB linear dir runs from BotCircle to TopCircle
   SmBoolean          bMakeNurbGenCurve,  // in : TRUE = GenCurve is type SmBSplineCurve
                                          //      FALSE= GenCurve is type SmLine
   const SmContext  * cpContext)          // in : Set context if given, default:[NULL]
 : SmCone(crOrigin,
          crXAxis,
          crYAxis,
          dRadius,
          dRadius,
          dStartAngle,
          dEndAngle,
          dHeight,
          bSwapUV,
          bInsideOut,
          bMakeNurbGenCurve)
{
  // set context if given
  if(cpContext) 
    { m_cpContext = cpContext ; }

} // end SmCylinder::SmCylinder constructor

/*******************************************************************//**
PURPOSE: create an infinite closed cylinder centered on crOrigin

NOTES: This Cylinder should be used when surface operations (like
    SS-Intersection) may take advantage of its analytic properties.

    - when bSwapUV==FALSE and bInsideOUt==FALSE
      - u-parameterization (rotational): [ 0, 360 ]
      - v-parameterization (along axis): [ -SM_INFINITE_PARAMETER, SM_INFINITE_PARAMETER ]
      - Cylinder SrfNormal points from SrfPt away from Cylinder axis
***********************************************************************/
SmCylinder::SmCylinder
  (const SmPoint3d  & crOrigin,   // in : point on cylinder axis
   const SmVector3d & crXAxis,    // in : vector perp to axis to cylinder start point
   const SmVector3d & crYAxis,    // in : vector perp to axis and X Axis 
   double             dRadius,    // in : distance from axis to cylinder wall
   SmBoolean          bSwapUV,    // in : TRUE = underlying NURB surface v dir maps to rotation direction
                                  //      FALSE= underlying NURB surface u dir maps to rotation direction
   SmBoolean          bInsideOut, // in : TRUE = underlying NURB linear dir runs from TopCircle to BotCircle
                                  //      FALSE= underlying NURB linear dir runs from BotCircle to TopCircle
   const SmContext  * cpContext)       // in : Set context if given, default:[NULL]
 : SmCone(crOrigin,
          crXAxis,
          crYAxis,
          dRadius,
          0.0,
          bSwapUV,
          bInsideOut)
{
  // set context if given
  if(cpContext) 
    { m_cpContext = cpContext ; }
} // end SmCylinder::SmCylinder constructor

/*******************************************************************//**
PURPOSE: Copy constructor for SmCylinder

NOTES: 
***********************************************************************/
SmCylinder::SmCylinder
  (const SmCylinder & crSource)
 : SmCone(crSource)
{

} // end SmCylinder::SmCylinder constructor

/*******************************************************************//**
PURPOSE: Equality operator for SmCylinder

NOTES: Call base equivalence to check type and then check 
       members for equivalence
***********************************************************************/
SmBoolean SmCylinder::operator==
  (const SmSurface& crOther) 
 const
{
  // low work
  if(this == &crOther) { return TRUE ; }

  // first check the base
  SmBoolean bRtn = SmCone::operator ==(crOther) ;

  // all done
  return bRtn ;

} // end SmCylinder::operator==

/*******************************************************************//**
PURPOSE: Copy a Cylinder

NOTES: 
***********************************************************************/
SmStatus SmCylinder::Copy
  (const SmContext & crContext,
   SmSurface      *& rpNewSurface) 
  const
{
    SmCylinder *pCopy = new(crContext) SmCylinder(*this);
    NER(pCopy);
    rpNewSurface = pCopy;
    return SM_SUCCESS;

} // end SmCylinder::Copy

/*******************************************************************//**
PURPOSE: Method to create a canonical Cylinder object.

NOTES: 
***********************************************************************/
SmStatus SmCylinder::CreateCanonical
  (const SmContext        & crContext,
   const SmAxis2Placement & crOrigin,
   double                   dRadius,
   SmCylinder            *& rpNewCylinder)
{
  // Reject non-positive radius (ctor assert is a no-op in release).
  if (!(dRadius > SM_EFF_ZERO))
    {
      rpNewCylinder = NULL;
      SE_MSG(SM_ERR, _T("SmCylinder::CreateCanonical: non-positive radius; cannot create cylinder")) ;
      return SM_ERR;
    }

  rpNewCylinder = new (crContext) SmCylinder(crOrigin.GetOriginRef(),
                                             crOrigin.GetXAxisRef(), 
                                             crOrigin.GetYAxisRef(), 
                                             dRadius);
  NER(rpNewCylinder);

  // Build NURB; fail cleanly rather than leak a null-NURB object.
  if (rpNewCylinder->MakeNurb() != SM_SUCCESS || rpNewCylinder->GetGwNurbPointer() == NULL)
    {
      delete rpNewCylinder; rpNewCylinder = NULL;
      SE_MSG(SM_ERR, _T("SmCylinder::CreateCanonical: failed to build NURB representation")) ;
      return SM_ERR;
    }

  return SM_SUCCESS;

} // end SmCylinder::CreateCanonical

/*******************************************************************//**
PURPOSE: Method to get canonical data from Cylinder object.

NOTES: 
***********************************************************************/
SmStatus SmCylinder::GetCanonical
  (SmAxis2Placement & rOrigin,
   double           & rdRadius)
 const
{
    rOrigin = m_vPosition;
    rdRadius = GetRadius();

    return SM_SUCCESS;

} // end SmCylinder::GetCanonical

/*******************************************************************//**
PURPOSE: Get the maximum allowable domain for an analytic representaion
   of this surface.

NOTES: 
   The u/v parameters correspond to the STEP parameterization.
***********************************************************************/
SmExtent2d SmCylinder::GetMaxAnalyticDomain()
 const
{
  // The v-domain is the domain of the generator curve, which is a line.
  SmExtent1d sVDom= m_pGenCurve->GetMaxAnalyticDomain();

  // u-domain is 0 to 360.
  SmExtent2d sDom( 0.0, sVDom.GetMin(), 360.0, sVDom.GetMax() );

  return sDom;

} // end SmCylinder::GetMaxAnalyticDomain

/*******************************************************************//**
PURPOSE: Static function to determine if a given nurbs is a Cone.
    If it is a Cone then one is created.

NOTES: For now, it will check only those surfaces whose drgree
    is two for one param & one for the other. 
***********************************************************************/
SmBoolean SmCylinder::IsNurbSurfaceCylinder
  (const SmContext & crContext,
   const SmBSplineSurface * pTestSurface,
   SmCylinder *& rpCylinder,
   double dToleranceScale)
{
    ULONG lDegU = pTestSurface->GetDegree(SM_SP_U);
    ULONG lDegV = pTestSurface->GetDegree(SM_SP_V);
    if (lDegU + lDegV != 3 ||
        !pTestSurface->IsRational() ||
        (lDegU == 1 && pTestSurface->GetNumberControlPoints(SM_SP_U) > 2) ||
        (lDegV == 1 && pTestSurface->GetNumberControlPoints(SM_SP_V) > 2)) {
        return FALSE;
    }

    SmExtent2d sUVDomain = pTestSurface->GetNaturalUVDomain();
    SmPoint3d sMinPnt, sMaxPnt;
    pTestSurface->EvaluatePoint(sUVDomain.GetMin(),sMinPnt);
    pTestSurface->EvaluatePoint(sUVDomain.GetMax(),sMaxPnt);

    double dPtTol = dToleranceScale * SM_EFF_ZERO * ANALYTIC_TOL_SCALE * (1.0 + sMinPnt.GetMaxDimension() + 
        sMaxPnt.GetMaxDimension());

    double dTopRad, dBotRad;
    double dStartAng1, dStartAng2, dEndAng1, dEndAng2;
    double dHeight;
    double dAngTol = SM_EFF_ZERO_SQRT;
    SmBoolean bSwapUV = FALSE;
    SmBSplineCurve *pTopIsoCrv = NULL;
    SmBSplineCurve *pBotIsoCrv = NULL;
    SmAxis2Placement sTopRefFrame;
    SmAxis2Placement sBotRefFrame;
    SmSurfParamType eSurfParam;
    double dBotKnot, dTopKnot;
    if (lDegU == 1) {
        // Need to swap UV when mapping parameters
        // from analytic domain to nurbs domain
        bSwapUV = TRUE;
        eSurfParam = SM_SP_U;
        dBotKnot = sUVDomain.GetMin().x;
        dTopKnot = sUVDomain.GetMax().x;
    }
    else {
        eSurfParam = SM_SP_V;
        dBotKnot = sUVDomain.GetMin().y;
        dTopKnot = sUVDomain.GetMax().y;
    }
    SER(pTestSurface->CreateIsoParametricCurve(crContext,
                                               eSurfParam,
                                               dTopKnot,
                                               0.0,
                                               pTopIsoCrv));
    SmObjDelete sCleanTop(pTopIsoCrv);
    if (pTopIsoCrv->IsDegenerate(dPtTol)) { return FALSE; }
    if (!pTopIsoCrv->IsArc(5,dPtTol,sTopRefFrame,
        dTopRad,dStartAng1,dEndAng1)) {
        return FALSE;
    }

    SER(pTestSurface->CreateIsoParametricCurve(crContext,
                                               eSurfParam,
                                               dBotKnot,
                                               0.0,
                                               pBotIsoCrv));
    SmObjDelete sCleanBot(pBotIsoCrv);
    if (pBotIsoCrv->IsDegenerate(dPtTol)) { return FALSE; }
    if (!pBotIsoCrv->IsArc(5,dPtTol,sBotRefFrame,
        dBotRad,dStartAng2,dEndAng2)) {
        return FALSE;
    }

    if (smos_Fabs(dStartAng1-dStartAng2) > dAngTol ||
        smos_Fabs(dEndAng1-dEndAng2) > dAngTol)
        return FALSE;

    const SmPoint3d  &rCenterTop = sTopRefFrame.GetOriginRef();
    const SmVector3d &rXAxis     = sTopRefFrame.GetXAxisRef();
    double dAng;
    rXAxis.AngleBetween(sBotRefFrame.GetXAxisRef(),dAng);
    if (smos_Fabs(dAng) > dAngTol)
        return FALSE;

    const SmPoint3d &rCenterBot = sBotRefFrame.GetOriginRef();
    SmVector3d       sZAxis     = rCenterTop - rCenterBot;
    dHeight                     = sZAxis.Length();

    // If you don't do this, Unitize will fail
    if (dHeight < SM_EFF_ZERO) {
        return FALSE;
    }

    sZAxis.Unitize();
    if (!sZAxis.IsParallelTo(sBotRefFrame.GetZAxis(),dAngTol))
        return FALSE;

    SmBoolean bInsideOut = FALSE;
    if (sZAxis.Dot(sBotRefFrame.GetZAxis()) < 0.0) {
        // Do some correction here
        bInsideOut = TRUE;
    }

//    if (dEndAng1 < 360.0 - SM_EFF_ZERO) {
//        SER(SM_ERR);
//    }
    SmVector3d sYAxis = sZAxis * rXAxis;
//    SmExtent2d sAnalUVDomain(SmPoint2d(dStartAng1,0.0),
//                             SmPoint2d(dEndAng1,dHeight));
    if (smos_Fabs(dBotRad-dTopRad) < dPtTol) {
        // Try to eliminate some numerical inaccurracies
        dTopRad = dBotRad; 
    }
    else {
        return FALSE;
    }

    rpCylinder = new (crContext) SmCylinder(rCenterBot, rXAxis, sYAxis,
                                            dBotRad,  dStartAng1,
                                            dEndAng1, dHeight,
                                            bSwapUV, bInsideOut, FALSE); // NULL);
    if (!rpCylinder) return FALSE;
    // error recovery
    if (rpCylinder->m_pGenCurve == NULL) {
        delete rpCylinder;
        rpCylinder = NULL;
        return FALSE;
    }

    SM_ASSERT(rpCylinder->m_pNurb == NULL) ;
    rpCylinder->m_pNurb = sm_AllocateAndCopyNurbSurface(pTestSurface->GetGwNurbPointer()) ;

    ((SmBSplineSurface *)pTestSurface)->Notify(SM_NO_COPY, rpCylinder, SM_NO_GET_OWNER(rpCylinder), SM_NO_GET_OWNER((SmBSplineSurface *)pTestSurface)) ;

    rpCylinder->RebuildPolarConverter() ;
  
    //      sCleanBot.Clear() ;
    //      SE(rpCylinder->m_vPolarConverter.SetUpPolarConversion
    //                                        (pBotIsoCrv,
    //                                         FALSE,
    //                                         rpCylinder->GetSTEPUVDomain().GetUInterval(),
    //                                         rpCylinder->m_vPosition));

    SmExtent1d sGenIvl(dBotKnot,dTopKnot);
    SE(rpCylinder->m_pGenCurve->EditParameterization(sGenIvl));

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
    if (bDebugMe) {
        pTestSurface->Dump();
        rpCylinder->Dump();
    }
#endif // SM_DEBUG_CODE

    return TRUE;

} // end SmCylinder::IsNurbSurfaceCylinder

/*******************************************************************//**
PURPOSE: Create and return a new Cylinder rotated about Z axis by a requested amount  

NOTES: this SmCylinder is not modified by this call 
***********************************************************************/
SmStatus SmCylinder::RotationAboutAxisZ
 (const SmContext & crContext,     // in : context for new object construction
  double            dAngleDeg,     // in : CCW rot about this ZAxis applied to this Cylinder to make new Cylinder
  SmCylinder     *& rpNewCylinder) // out: The newly constructed Cylinder
{
  // locals
  double           dAngleRad = SM_DEG2RAD(dAngleDeg);
  SmAxis2Placement sRF;
  SmAxis2Placement sTmpA2P;

  // make a simple rotate about Z axis Axis2Placement transformation
  sRF.RotateAboutAxisAtPoint(dAngleRad, 
                             m_vPosition.GetOriginRef(), 
                             m_vPosition.GetZAxis());

  // make a new Axis2Placement from current Placement transformed by rotation
  m_vPosition.TransformAxis2Placement(sRF,sTmpA2P);

  // use new Axis2Placement to make a new Cylinder with a proper placement
  SER(SmCylinder::CreateCanonical(crContext,
                                  sTmpA2P,
                                  GetRadius(),
                                  rpNewCylinder));

  // set the new Cylinder domain to be the same as the current domain
  SER(rpNewCylinder->AdjustSTEPUVDomain(GetSTEPUVDomain()));

  // all done
  return SM_SUCCESS;

} // end SmCylinder::RotationAboutAxisZ
                                                 
/*******************************************************************//**
PURPOSE: AssertValid() Test reporting static labels
***********************************************************************/
SmAssertReportLabel sAssertCylinder_list[] =
{
  /*  0 */ {SM_AT_RADIUS,    _T("Zero SemiAngle"),     _T("Cones must have zero SemiAngles") },
  /*  1 */ {SM_AT_GEOMETRIC, _T("Overlapping Domain"), _T("Analytic U Direction Domain too large: cylinders can't overlap themselves") },
  /*  2 */ {SM_AT_GEOMETRIC, _T("Domain Max"),         _T("Analytic U Direction Max Domain larger than +360") },
  /*  3 */ {SM_AT_GEOMETRIC, _T("Domain Min"),         _T("Analytic U Direction Min Domain less than -360") }
} ;

/*******************************************************************//**
PURPOSE: Make sure GenCurve and Surface parameterizations are
            compatible.

RETURNS ---  TRUE  = OK
             FALSE = Problem
***********************************************************************/
SmBoolean SmCylinder::AssertValid
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
           ? SmCone::AssertValid(pAList, eTestLevel, SM_NO_WALK, pTestRequests) 
           : TRUE ) ; 

  // must have a positive radius
  bRtn &= SM_ASSERT_VALUE_REPORT(0, SM_LEVEL_0, (SM_IS_ZERO(m_dSemiAngleDeg)), SM_EFF_ZERO, m_dSemiAngleDeg, _T("") ) ;

  // Analytic U Direction Domain too large: spheres can't overlap themselves
  SmExtent1d sIvlU = m_vAnalUVDomain.GetUInterval() ;
  bRtn &= SM_ASSERT_VALUE_REPORT(1, SM_LEVEL_0,
                                 (sIvlU.GetLength() <= 360.0 + SM_EFF_ZERO_DEG),
                                 SM_EFF_ZERO_DEG, sIvlU.GetLength() - 360.0, _T("")) ;   

  // Analytic U Direction Max Domain larger than +360
  bRtn &= SM_ASSERT_VALUE_REPORT(2, SM_LEVEL_0,
                                 (sIvlU.GetMax() <= 360.0 + SM_EFF_ZERO_DEG),
                                 SM_EFF_ZERO_DEG, sIvlU.GetMax() - 360.0, _T("")) ;   

  // Analytic U Direction Min Domain less than -360
  bRtn &= SM_ASSERT_VALUE_REPORT(2, SM_LEVEL_0,
                                 (sIvlU.GetMin() >= -360.0 - SM_EFF_ZERO_DEG),
                                 SM_EFF_ZERO_DEG, sIvlU.GetMin() + 360.0, _T("")) ;   


#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  // draw 
  if(bDebugMe)
    {
      if(m_pGenCurve) m_pGenCurve->Dump() ;
      Dump() ;

      SmFace *pFace = (SmFace *)GetOwner() ;
      SmBrep *pBrep = pFace ? pFace->GetBrep() : NULL ;
      
      smgfx_Erase() ;
      smgfx_SetLook( 1, 2, 0, 0, 1 ); if(pBrep) { pBrep->Draw( TRUE ); sm_GraphicsLoop(); }
      smgfx_SetLook(1,2, 0,1,1) ; DrawUV() ; sm_GraphicsLoop() ;
      smgfx_SetLook( 1, 2, 0, 0, 0 ); if(pFace) { pFace->Draw( SM_DM_CROSSHATCH ); sm_GraphicsLoop(); }
      smgfx_SetLook( 3, 4, 1, 0, 0 ); if(m_pGenCurve) { m_pGenCurve->Draw(); sm_GraphicsLoop(); }
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE
                    
  // all done
//        SM_ASSERT(bRtn);
  return(bRtn) ;

} // end SmCylinder::AssertValid

// obsolete
// /*******************************************************************//**
// PURPOSE: Fix Assert Report failures reported by an AssertValid call
// 
// RETURNS ---  TRUE  = OK
//              FALSE = Problem
// ***********************************************************************/
// SmBoolean SmCylinder::AssertHeal
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
//       return ( SmCone::AssertHeal(rAReport, pAList) ) ;
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
//                rAReport.m_pHealMessage = _T("SmCylinder::AssertHeal fix not yet supported") ;  
//                 
//     }
// 
//   // all done
//   return(bRtn) ;
// 
// } // end SmCylinder::AssertHeal
// end obsolete

/*******************************************************************//**
PURPOSE: Write SmCylinder to given output stream.

NOTES: 
***********************************************************************/
SmStatus SmCylinder::WriteToDB
 (SmDatabaseIO & rDB,                // in : target output stream
  ULONG          lDBVersionNumber)   // in : database version to get proper sequence of writes                                                                       
 const
{
  // file type, ASCII or BINARY
  // SmFileType      eType    =  rDB.GetFileType();
  // std::ostream  & rFileOut = *rDB.GetOutStreamPtr();
  
  // a cylinder has no data of its own

  // output the parent
  SER(SmCone::WriteToDB(rDB, lDBVersionNumber)) ; 

  // all done
  return SM_SUCCESS;

} // end SmCylinder::WriteToDB

/*******************************************************************//**
PURPOSE: static method to Read a SmCylinder from a given stream  

NOTES: 
***********************************************************************/
SmStatus SmCylinder::ReadFromDB
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
         || rpNewSurface->IsKindOf(SmCylinder_TYPE)) ? SM_SUCCESS : SM_ERR) ;

  // init output object
  SmCylinder *pCylinder =   (rpNewSurface == NULL)
                          ? new (crContext) SmCylinder()
                          : (SmCylinder *)rpNewSurface ;

  // file type
  //SmFileType eType   =  rDB.GetFileType();
  // std::istream & rFileIn = *rDB.GetInStreamPtr();
  
  // a cylinder has no data of its own    

  // read the parent object
  SmSurface *pSurface = pCylinder ; 
  SER(SmCone::ReadFromDB(SmCone_TYPE, rDB, crContext, pSurface, lDBVersionNumber)) ;

  // all done
  rpNewSurface = pCylinder ; 
  return SM_SUCCESS;

} // end SmCylinder::ReadFromDB

/*******************************************************************//**
PURPOSE:
NOTES:
***********************************************************************/
SmBoolean SmCylinder::IsKindOf( SM_TYPE t ) const
{
  return ((SmCylinder_TYPE == t) ? TRUE : SmCone::IsKindOf( (t) ));
}

/*******************************************************************//**
PURPOSE: Dump Cone surface data out for debugging.

NOTES: 
***********************************************************************/
void SmCylinder::Dump
  (void) 
 const
{
    TCHAR sBuff[SM_TBLOCK_SIZE], sBuffForFile[SM_TBLOCK_SIZE];
    smos_WriteBuffer(_T("\nBegin SmCylinder::Dump()")) ;
    
    SM_SPRINTF(sBuff,       _T("\nSmCylinder = 0x%p, Radius = %16.16lf, Swap UV = %1d, lfTransform - \n"),
        this, GetRadius(), m_bSwapUV);
    SM_SPRINTF(sBuffForFile,_T("\nSmCylinder = %s, Radius = %16.16lf, Swap UV = %1d, lfTransform - \n"),
        _T("notNULL"), GetRadius(), m_bSwapUV);
    smos_WriteBuffer(sBuff, sBuffForFile);
    m_vPosition.Dump();

    SmCone::Dump();
    smos_WriteBuffer(_T(" End SmCylinder::Dump()\n")) ;

} // end SmCylinder::Dump
