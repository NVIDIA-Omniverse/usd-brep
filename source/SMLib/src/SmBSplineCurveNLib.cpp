// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************//**
* FILE NAME --- SmBSplineCurveNLib.cpp 
* PURPOSE: Implementation of methods used exclusively with NLib  
**********************************************************************/

#include "StdAfx.h"

#include <nurbs.h> 
#include <SmBSplineCurve.h>
#include <SmHermiteCurve.h>
#include <SmCurveTypes.h>

// #include <NL_BasisAdv.h>    /* Advanced NL_KNOTVECTOR functions */
#include <NL_CrvAdv.h>      /* Advanced NL_CURVE functions      */
// #include <NL_SrfAdv.h>      /* Advanced NL_SURFACE functions    */
// #include <NL_VolumeAdv.h>   /* Advanced NL_VOLUME functions     */
// #include <NL_FuncsAdv.h>    /* Advanced NL_CFUN, NL_CVALUE, NL_SFUN, NL_SVALUE, NL_VFUN, and NL_VVALUE functions */
// #include <NL_FrameAdv.h>    /* Advanced NL_CPOLYGON, NL_EPOLYGON, NL_CNET, NL_ENET, NL_CMESH, and NL_EMESH functions */
// #include <NL_Tessellate.h>  /* Tessellation functions */
#include <NL_Spiral.h>      /* Spiral functions */

#ifdef SM_DEBUG_CODE
#include <SmEdge.h>
#include <SmBrep.h>
#endif

/*******************************************************************//**
PURPOSE:  This object automatically calls the initilization 
             and termination of the GW nurb curve stack.

NOTES:
***********************************************************************/
class SmNLibStackHandler 
{
 private:
  NL_STACKS * m_pStacks;
 public:
  SmNLibStackHandler(NL_STACKS * pStacks) { m_pStacks = pStacks; 
                                         N_InitNurbs(m_pStacks); 
                                       }
 ~SmNLibStackHandler()                 { N_EndNurbs(m_pStacks); }
} ; // end class SmNLibStackHandler

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
static NL_CURVE * sm_CreateNlibCurve
  (SmBSplineCurve *pBSC,
   NL_STACKS & SG)
{
    NL_CURVE *pNewCur = N_AllocCrvAndArrays((NL_INDEX)pBSC->GetNumberControlPoints(),
        (NL_DEGREE)pBSC->GetDegree(), (NL_INDEX)pBSC->GetNumberNaturalKnots(),&SG);

    NL_CURVE *pCur = (NL_CURVE*)pBSC->GetOrCreateGwNurbPointer();
    if (N_CrvCopy(pCur,pNewCur,&SG)) {
        SE(SM_ERR);
        return NULL;
    }
    return pNewCur;

} // end sm_CreateNlibCurve


/* ---------------------------------------------------------------------

   DESCRIPTION:

     NOTE - modified by GAC to accept a number of control points as an
            optional argument.

     This approximation routine approximates a given circle or  circular
     arc with a non-rational  curve of degree 2,3, or 4.  The  error  is 
     measured in terms of the deviation from center  (error in radius) ,
     and the desired error tolerance is specified either relative to the 
     length of the radius, or as an absolute distance. A typical calling 
     example is:

       NL_CURVE   cur;
       NL_DEGREE  deg;
       NL_POINT   cen, Ps, Pe;
       NL_VECTOR  X, Y;
       NL_REAL    tol, rad, as, ae;
       NL_STACKS  SG;
       ...
       (define tol and deg, and get circle defining data);
       ...
       N_CrvInitArrays(&cur);
       N_ApproxCircArcWithCrv(cen,X,Y,rad,as,ae,&Ps,&Pe,deg,tol,RELATIVE,&cur,&SG);

     MEMORY TO STORE THE  OUTPUT CURVE  cur  IS ALLOCATED  INSIDE THE
     ROUTINE! 


   ACCESS:
   
     cen   , input  ,  Center of circle
     X,Y   , in/out ,  Local coordinate system of circle
     rad   , input  ,  Radius of circle
     as,ae , input  ,  Start and end angles of circle
     Ps,Pe , input  ,  Start and end points of circle  (provided  to en-
                       sure strict continuity with neighboring  curves).
                       Not used if equal to NULL
     deg   , input  ,  Degree of approximating non-rational circle (2, 3
                       or 4)
     tol   , input  ,  Radial error tolerance (see eflg)
     eflg  , input  ,  Error tolerance flag:
                         ABSOLUTE: tol is an absolute distance
                         RELATIVE: tol is a percentage of the radius
                       If rad=2,  then  (tol,eflg)=(0.002,ABSOLUTE)  and
                       (tol,eflg)=(0.1,RELATIVE)  are  equivalent  error
                       tolerances. tol may have any value, however, this
                       routine achieves tolerances  only  in  the  range
                       1.e-6 %  to 1 %  (RELATIVE tol)
     plOptNumCtrlPts, input , optional number of control points to use in
                       the approximation.
     cur   , output ,  Approximating non-rational curve
     SG    , input  ,  cur's stack

 
   RETURN CODES:

     0 : No error
     1 : Error saved in NERR

   ------------------------------------------------------------------ */
NL_FLAG  sm_ApproxCircArc( NL_POINT cen, NL_VECTOR X, NL_VECTOR Y, NL_REAL rad, NL_REAL as, NL_REAL ae,
                NL_POINT *Ps, NL_POINT *Pe, NL_DEGREE deg, NL_REAL tol, NL_FLAG eflg, 
                ULONG * plNumControlPoints,
                NL_CURVE *cur, NL_STACKS *SG )

{

  NL_FLAG     error = NL_NO;

  NL_INDEX       nc;

  NL_REAL        dd, sweep;

  NL_POINT   Rs, Re;

  NL_SFUN        sfn;

  NL_STACKS      SL;



  /* Start NURBS */


  N_InitNurbs(&SL);


  /* Check for errors on input */


  if ( deg LT 2  OR  deg GT 4 )    { error = NL_YES; NL_OUT; };
  if ( rad LE 0.0  OR  as EQ ae )  { error = NL_YES; NL_OUT; };


  /* Get number of control points required for fit */


  if ( ae LT as )   ae = ae + 360.0;
  sweep = ae-as;
  if ( eflg EQ NL_ABSOLUTE )   tol = 100.0*(tol/rad);

  if (plNumControlPoints == NULL) {
      N_SFuncInitArrays(&sfn);
      error = N_CalcNumCPtsToApproxArc(tol,sweep,deg,NL_NO,&nc,&sfn,&SL);
      if ( error EQ NL_YES )  NL_OUT;
      if (nc < deg) nc = deg;
  }
  else {
      nc = *plNumControlPoints - 1;
  }

  /* Now fit nonrational curve to circle data */


  N_CrvInitArrays(cur);

  error = N_VectorNormalizeRef(&X);
  if ( error EQ NL_YES )   { error = NL_YES; NL_OUT; };
  error = N_VectorNormalizeRef(&Y);
  if ( error EQ NL_YES )   { error = NL_YES; NL_OUT; };

  if ( Ps EQ NULL )  
  {
    dd = as*(NL_PI/180.0);
    N_TranslateSum2Pts(cen,rad*cos(dd),X,rad*sin(dd),Y,&Rs);
  }
  else
    N_CopyPt(*Ps,&Rs);

  if ( Pe EQ NULL )  
  {
    if ( sweep EQ 360.0 )   N_CopyPt(Rs,&Re);
    else
    {
      dd = ae*(NL_PI/180.0);
      N_TranslateSum2Pts(cen,rad*cos(dd),X,rad*sin(dd),Y,&Re);
    }
  }
  else
    N_CopyPt(*Pe,&Re);

  error = N_ApproxCircArcWithCrvData(cen,X,Y,rad,as,ae,Rs,Re,deg,nc,cur,SG);
  if ( error EQ NL_YES )  NL_OUT;


  /* End NURBS and Exit */


  EXIT:

  N_EndNurbs(&SL);

  return(error);

} // end sm_ApproxCircArc

/*******************************************************************//**
PURPOSE: Approximate an Arc with a non-rational curve.

NOTES: This method is only available for users with NLib
    
***********************************************************************/
SmStatus SmBSplineCurve::ApproximateArc
  (const SmContext        & crContext,                   // in : context for new object construction
   ULONG                    lDimensionOfResult,          // in : image space size, 2 or 3
   ULONG                  * plOptNumberOfControlPoints,  // in : CtrlPoint count in new Curve
   const SmAxis2Placement & crReferenceFrame,            // in : Specify start and orientation of arc
                                                         //      Angles measured from XAxis in X/Y plane
                                                         //      in CCW direction
   double                   dRadius,                     // in : output Curve radius
   double                   dStartAngle,                 // in : output Curve start angle in degrees
   double                   dEndAngle,                   // in : output Curve end angle in degrees
   double                   dTolPercentRadius,           // in : max allowed percentage deviation of output curve from circular arc
   SmBSplineCurve        *& rpNewCurve)                  // out: new curve
{
/*
 cen   , input  ,  Center of circle
 X,Y   , in/out ,  Local coordinate system of circle
 rad   , input  ,  Radius of circle
 as,ae , input  ,  Start and end angles of circle
 Ps,Pe , input  ,  Start and end points of circle  (provided  to en-
                   sure strict continuity with neighboring  curves).
                   Not used if equal to NULL
 deg   , input  ,  Degree of approximating non-rational circle (2, 3
                   or 4)
 tol   , input  ,  Radial error tolerance (see eflg)
 eflg  , input  ,  Error tolerance flag:
                     ABSOLUTE: tol is an absolute distance
                     RELATIVE: tol is a percentage of the radius
                   If rad=2,  then  (tol,eflg)=(0.002,ABSOLUTE)  and
                   (tol,eflg)=(0.1,RELATIVE)  are  equivalent  error
                   tolerances. tol may have any value, however, this
                   routine achieves tolerances  only  in  the  range
                   1.e-6 %  to 1 %  (RELATIVE tol)
 cur   , output ,  Approximating non-rational curve
 SG    , input  ,  cur's stack
*/
  // init NLib stack using SmNLibStackHandler to call N_InitNurbs() and N_EndNurbs() appropriately.
  NL_STACKS    SG;
  SmNLibStackHandler sSH(&SG);

  // locals
  NL_CURVE     cur;
  NL_DEGREE    deg = 3;
  NL_POINT  cen;
  NL_VECTOR    X, Y;
  NL_REAL      tol = dTolPercentRadius;
  NL_REAL      rad = dRadius;
  NL_REAL      as = dStartAngle;
  NL_REAL      ae = dEndAngle;
  
  // copy SMLib arc center, x, and y axes into NLib format
  const SmPoint3d &rSmPntO = crReferenceFrame.GetOriginRef();
  const SmPoint3d &rSmPntX = crReferenceFrame.GetXAxisRef();
  const SmPoint3d &rSmPntY = crReferenceFrame.GetYAxisRef();
  COPY_XYZ(rSmPntO, cen);
  COPY_XYZ(rSmPntX, X);
  COPY_XYZ(rSmPntY, Y);

  // init output cur object empty memory pointers
  N_CrvInitArrays(&cur);

  // GWC??? I don't see what's being done with this change in arc size
  if (smos_Fabs(ae-as-90.0) < 1.0e-10)
      ae = as + 90.0;

  // Build approximation to arc - but without specifying exact endPoint values.
  //   GWC:to tighten tolerances perhaps we should consider using exact endPoint interpolation
  NL_SER(sm_ApproxCircArc(cen,X,Y,rad,as,ae,
                          NULL,NULL,            // in : NULL = don't ask for exact endPoint interpolation
                          deg,tol,NL_RELATIVE,
                          plOptNumberOfControlPoints,&cur,&SG));

  // preserve 2D curves
  if (lDimensionOfResult == 2) N_Crv3dTo2d(&cur);

  // set output - build NMTlib BSplineCurve object from NLib object
  rpNewCurve = new (crContext) SmBSplineCurve(lDimensionOfResult,(gw_CURVE *)&cur);

  // all done
  return SM_SUCCESS;

} // end SmBSplineCurve::ApproximateArc

/*******************************************************************//**
PURPOSE: This routine computes a NURBS curve approximating a given
    set of points. 
                                                                                       
NOTES: User may specify a tolerance for approximation
    with error bound. In addition, start and/or end tangents may also be
    specified as constraints. 

   End tangents are ignored for closed-curve approximations.

   This method is only available for users with NLib

   This routine is intended for use with "lots" of points.
   There is a minimum number of points required to create an approximation.
   That minimum number varies with the input parameters: not passing a tolerance
   increases the minimum; higher degree requires more points, as does passing
   start/end tangents.  The very minimum is 5.
   If you have only a few points, consider using CreateInterpolatingCurve().
***********************************************************************/
SmStatus SmBSplineCurve::ApproximatePoints
  (const SmContext           & crContext,        // in : context for new object construction
   const SmTArray<SmPoint3d> & crPoints,         // in : array of ordered points to approximate 
   ULONG                       lDegree,          // in : degree of created approximating BSplineCurve
   SmVector3d                * pOptStartTangent, // in : Start Tangent, NULL to ignore
   SmVector3d                * pOptEndTangent,   // in : End Tangent, NULL to ignore
   SmBoolean                   bIsClosedCurve,   // in : TRUE = create closed curve, FALSE = don't
   double                    * pdOptTolerance,   // in : max distance between points and created curve, NULL to ignore
   SmBSplineCurve           *& rpNewCurve)       // out: The new curve
{
  // init NLib stack using SmNLibStackHandler to call N_InitNurbs() and N_EndNurbs() appropriately.
  NL_STACKS    SG;
  SmNLibStackHandler sSH(&SG);

  // locals
  ULONG ii ;
  NL_INDEX  m = crPoints.GetSize()-1;

  // new curve locals 
  NL_CURVE    cur, CubicCur;
  NL_DEGREE   p = (NL_DEGREE)lDegree;
  NL_INDEX    n = m-4;
  NL_VECTOR   Ts, Te;
  NL_VECTOR  *pTs = NULL;
  NL_VECTOR  *pTe = NULL;

  // init output and intermediate curve structures
  N_CrvInitArrays(&cur);
  N_CrvInitArrays(&CubicCur);

  // Init end tangent values
  N_VectorCreate( 0.0, 0.0, 0.0, &Ts );
  N_VectorCreate( 0.0, 0.0, 0.0, &Te );

  // copy input points from SMLib to NLib format
  NL_POINT  *P = N_AllocPt1dArray(m,&SG);
  for (ii=0; ii<crPoints.GetSize(); ii++) 
    {
      SmPoint3d sPnt = crPoints[ii];
      COPY_XYZ(sPnt,P[ii]);
    }
  
  // when given a OptStartTangent - use it
  if (pOptStartTangent) 
    {
      n--;   // decrement the point count - one is being used for an endTangent constraint
      pTs = &Ts;
      COPY_XYZ_P(*pOptStartTangent,pTs);
    }
 
  // when given a OptEndTangent - use it
  if (pOptEndTangent) 
    {
      n--;    // decrement the point count - one is being used for an endTangent constraint
      pTe = &Te;
      COPY_XYZ_P(*pOptEndTangent,pTe);
    }

  // DO NOT continue if there are not enough points
  if( n < 0 )
    { return SM_ERR; }

  // without a requested tolerance
  if (pdOptTolerance == NULL) 
    {
      // for open curves
      if (!bIsClosedCurve) 
        {
          // build a BSplineCurve [ctrlPtCount = SmaplePointCount] least squares approximation
          //  to the point set with given end tangent directions (not magnitudes)
          NL_SER(N_FitCrvApproxTangents(P,            // in : points to be approximated                                                
                                        m,            // in : highest index in sP                                                      
                                        n,            // in : highest control point index for output cur                               
                                        p,            // in : degree for output cur                                                    
                                        pTs,          // i/o: start tangent or derivative. When tangent, mag may change, NULL to ignore
                                        pTe,          // i/o: end tangent or derivative. When tangent, mag may change, NULL to ignore  
                                        NL_TANGENT,      // in : NL_TANGENT    = pTs and pTe are directions and are scaled inside the call
                                                      //      NL_DERIVATIVE = pTs and pTe are derivatives and mags are used as given   
                                        NL_CHORDLENGTH,  // in : NL_UNIFORM     = uniform parameterization                                
                                                      //      NL_CHORDLENGTH = chord length parameterization                           
                                                      //      NL_CETRIPETAL  = Centripetal parameterization                            
                                        &cur,         // out: approximating curve                                                      
                                        &SG));        // in : cur's memory stack                                                       
        }
      else // for closed curves
        { // Closed curve
          NL_INDEX    k  = p-1; //C^k continuity at ends
          NL_VECTOR  *pD = NULL;
          NL_VECTOR   D[2];

          // use startTangent when available
          if (pTs) { D[1] = Ts;
                     pD   = D;
                   }

          // use endTangent when available
          if (pTe) { D[1] = Te;
                     pD   = D;
                   }

          // when using end tangents, set highest index in D array to 1 (just passing a tangent constraint
          if (pD)  { k = 1 ;
                   }
          
          // build a closed BSplineCurve [ctrlPtCount = SmaplePointCount] least squares approximation
          //  to the point set with given end tangent directions (not magnitudes)
         NL_SER(N_FitCrvApproxClosed(P,            // in : points to be approximated                                                   
                                     m,            // in : highest index in Points to be approximated array                            
                                     pD,           // in : start and stop erivatives, NULL to ignore                                   
                                     k,            // in : out curve continuity, 1 = C1, 2 = C2 ...                                    
                                                   //     when given pD, it must have one deriv for each end for each continuity level.
                                     n,            // i/o: highest control point index of output cur                                   
                                     p,            // i/o: degree of output cur                                                        
                                     NL_CHORDLENGTH,  // in : NL_UNIFORM     = uniform parameterization                                   
                                                   //      NL_CHORDLENGTH = chord length parameterization                              
                                     &cur,         //      NL_CETRIPETAL  = Centripetal parameterization                               
                                     &SG));        // out: approximating curve                                                         
        }                                          // in : cur's memory stack                                                          
    }
  else // with a requested tolerance
    {
      // for open curves
      if (!bIsClosedCurve) 
        {
          // build a BSplineCurve [ctrlPtCount <= SmaplePointCount] least squares approximation
          //  to the point set with given end tangent vectors (with magnitudes).
          // Increase curve degree until a reduced knot curve can be built that is within tolerance of all points
          NL_SER(N_FitCrvApproxKnotsAndTangentsTol(P,m,NL_BOTH,pTs,pTe,NL_YES,2,p,*pdOptTolerance,
                                                   NL_SINGLE,&cur,&SG));
        }
      else // for closed curves 
        { 
          // build a closed BSplineCurve [ctrlPtCount <= SmaplePointCount] least squares approximation
          //  to the point set without endPoint tangent constraints.
          // Increase curve degree until a reduced knot curve can be built that is within tolerance of all points
          NL_SER(N_FitCrvApprox(P,m,NL_YES,2,p,*pdOptTolerance,NL_SINGLE,&cur,&SG));
        }
    }

  // set output - build SMLib BSplineCurve object from NLib object
  rpNewCurve = new (crContext) SmBSplineCurve(3,(gw_CURVE *)&cur);

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
  if (bDebugMe) 
    {
      sm_GraphicsLoop();
      smgfx_SetColor(1,0,0);
      rpNewCurve->Draw();
      rpNewCurve->Dump();
    }
#endif // end SM_DEBUG_CODE

  // all done
  return SM_SUCCESS;

} // end SmBSplineCurve::ApproximatePoints

/*******************************************************************//**
PURPOSE: Approximate a rational B-spline with a non-rational curve.

NOTES:
***********************************************************************/
SmStatus SmBSplineCurve::ApproximateRationalCurve
  (const SmContext & crContext,          // in : context for new object construction
   double            dThisApproxTol3d,   // in : max distance between approximate and this curve
   SmBSplineCurve *& rpNewCurve)         // out: the approximated curve
{
  // init output
  rpNewCurve = NULL;

  // input check - only run on rational BSplineCurves
  if (!IsRational()) 
    {
      SE(SM_ERR); return SM_ERR; 
    }

  // locals
  NL_CURVE * curThis = (NL_CURVE*)GetOrCreateGwNurbPointer();
  NL_CURVE   curNew;
  NL_DEGREE  deg = 3;

  // init NLib stack using SmNLibStackHandler to call N_InitNurbs() and N_EndNurbs() appropriately.
  NL_STACKS  SG;
  SmNLibStackHandler sSH(&SG);

  // init Crv Object empty memory pointers
  N_CrvInitArrays(&curNew);

  // 
  NL_SER(N_ApproxNurbsWithNonRatCrv(curThis,dThisApproxTol3d,deg,NL_INHERITED,NL_YES,&curNew,&SG));

  // set output - build SMLib BSplineCurve object from NLib object
  rpNewCurve = new (crContext) SmBSplineCurve(3,(gw_CURVE *)&curNew);

  // all done
  return SM_SUCCESS;

} // end SmBSplineCurve::ApproximateRationalCurve

/********************************************************************//**
PURPOSE: Create a new SmBSplineCurve approximation of a Helix

NOTES: Pass call along to N_CrvApproxSpiral()
 ***********************************************************************/
SmStatus SmBSplineCurve::CreateHelixSegment
 (const SmContext        & crContext,           // in : context for new object construction
  const SmAxis2Placement & crReferenceFrame,    // in : Helix centered on Z Axis starting at Z=0 on X Axis, running to Z=Height
  double                   dHeight,             // in : Helix Length along RefFrame Z Axis, Height:[0=build spiral,>0=build helix]
  double                   dRadiuStart,         // in : Helix Radius at RefFrame Z = 0      (linearly interpolated from 0 to Height)
  double                   dRadiusEnd,          // in : Helix Radius at RefFrame Z = Height (linearly interpolated from 0 to Height)
  double                   dNumTurns,           // in : Num of 360 deg Helix Turns in H=[0,Height] (Fractions okay), PeriodLength:[Height/NumTurns]
  SmBoolean                bRightHanded,        // in : TRUE = spiral is right handed, FALSE=left handed
  SmApproxTol3d            sApproxTol3d,        // in : Max allowed deviation between NewBSplineCurve and theoretical Helix
  SmBSplineCurve        *& rpNewBSplineCurve)   // out: The new Helix curve.
{
  // check input
  SER(dHeight     >= -SM_EFF_ZERO ? SM_SUCCESS : SM_ERR_INVALID_INPUT ) ;
  SER(dRadiuStart >= 0.0          ? SM_SUCCESS : SM_ERR_INVALID_INPUT ) ;
  SER(dRadiusEnd  >= 0.0          ? SM_SUCCESS : SM_ERR_INVALID_INPUT ) ;
  SER(dNumTurns   >= SM_EFF_ZERO  ? SM_SUCCESS : SM_ERR_INVALID_INPUT ) ;
  
  // locals  
  NL_STACKS SC;
  NL_CURVE  Cur ;
  SmStackHandler sStackKp(&SC) ;
  N_CrvInitArrays(&Cur) ;
  NL_INDEX lNumPoints = 0 ;
  NL_REAL  dApproxTol3d = sApproxTol3d ;

  // Build the helical approximation
  if(NL_YES == N_CrvApproxSpiral(dHeight, 
                                 dRadiuStart, 
                                 dRadiusEnd, 
                                 dNumTurns, 
                                 bRightHanded, 
                                 &lNumPoints, 
                                 &dApproxTol3d, 
                                 &Cur, 
                                 &SC))
    { SER(SM_ERR); }

  // Build SMLib Curve from NLib Curve
  rpNewBSplineCurve = new (crContext) SmBSplineCurve(3, &Cur) ;

  // position helix 
  rpNewBSplineCurve->Transform( crReferenceFrame, NULL );

  rpNewBSplineCurve->m_eBSplineCurveForm = SM_CF_HELICAL_ARC ;

  // all done
  return SM_SUCCESS ;

} // end SmBSplineCurve::CreateHelixSegment

/********************************************************************//**
PURPOSE: Create a new SmBSplineCurve by extending an existing one by
         a given parametric distance
NOTES:
 ***********************************************************************/
SmStatus SmBSplineCurve::CreateExtendedCurve // eff: extend parameter range of this curve
 (const SmContext & crContext,               // in : memory pool for output curve
  double            dParameter,              // in : tgt parameter defines curve extension amount
  SmContinuityType  eExtensionContinuity,    // Either: SM_CT_G1        - linear extension
                                             //         SM_CT_G1_G2     - extension with second derivative
                                             //         SM_CT_CINFINITY - infinite continuity
  SmBSplineCurve *& rpExtended,              // out: Constructed curve
  SmBoolean         bPreciseExtension)       // in : 0 = extend curve beyond dParameter by an extra curve-interval amount
                                             //      1 = extend curve to given dParameter
{
    // get curve interval
    SmExtent1d sIvl = GetNaturalInterval();

    // when bPreciseExtension == 0 - extend dParameter value by curve's interval amount   
    if (!bPreciseExtension) {
        if (dParameter < sIvl.GetMin()) {
            dParameter = dParameter - (sIvl.GetMax() - sIvl.GetMin());
        }
        else if (dParameter > sIvl.GetMax()) {
            dParameter = dParameter + (sIvl.GetMax() - sIvl.GetMin());
        }
    }

    // locals for upcoming NLIB call
    NL_CURVE      ecur;                                     // use: NLIB call output curve
    NL_CURVE     *cur = (NL_CURVE*)GetOrCreateGwNurbPointer(); // use: input curve
    NL_PARAMETER  t;                                        // use: extension target parameter
    NL_FLAG    gflg = NL_G1;                             // use: oneof G1/G2/CMAX
    NL_INDEX      ndr;                                      // use: max order of the derivative
    NL_POINT   D[10];                                    // use: curve eval storage, D[0]=pos,D[i]=ith derivative
    NL_STACKS     SG;                                       // use: global stack, cur's stack and used for ecur memory
    SmNLibStackHandler sSH(&SG);                         // use: call N_InitNurbs on SG stack 

    // initialize inputs for upcoming NLIB call
    t = dParameter;
    ndr = 0;                                     // eff: eval position only - no derivatives
    if      (eExtensionContinuity == SM_CT_G1)        { gflg = NL_G1; }
    else if (eExtensionContinuity == SM_CT_G1_G2)     { gflg = NL_G2; }
    else if (eExtensionContinuity == SM_CT_CINFINITY) { gflg = NL_CMAX; }
    else { SER(SM_ERR); }

    // init ecur parameters to NULL CURVE
    N_CrvInitArrays(&ecur);


    // NLIB eval curve at t, 
    // side effect: create extended curve in ecur when t is outside curve interval 
    if (NL_YES == N_CrvEvalUnboundedPtsAndDerivs(cur, t, ndr, gflg, &ecur, &gflg, D, &SG))
    {
        N_FreeCrv(&ecur, &SG);
        return SM_ERR;
    }

    // build output: new SMBSplineCurve object from ecur 
    if (ecur.pol == NULL) rpExtended = NULL;
    else               
    rpExtended = new (crContext) SmBSplineCurve(GetDim(),(gw_CURVE *)&ecur);
 
    // all done
    return SM_SUCCESS;

} // end SmBSplineCurve::CreateExtendedCurve

/*******************************************************************//**
PURPOSE: Create a BSplineCurve from an input SmHermiteCurve

NOTES: By default, the BSplineCurve is parameterized from 0 to 1.
  Use optional input arguments to scale that differently.
***********************************************************************/
SmStatus SmBSplineCurve::CreateFromHermiteCurve
  (const SmContext       & crContext,         // in : context for new object construction
   const SmHermiteCurve  & rHermiteCurve,     // in : Curve to translate into a BSpline
   SmBSplineCurve       *& rpNewBSplineCurve, // out: Newly constructed curve
   double                  dParamStart,       // in : BSplineCurve param start, default:[0]
   double                  dParamEnd)         // in : BSplineCurve param end, default:[1]
{
  // locals
  SmTArray<SmPoint3d> sPoly ;
  SmTArray<ULONG>     sKnotMults(2,NULL,2) ;
  SmTArray<double>    sKnots(2,NULL,2) ;

  // get control points
  rHermiteCurve.GetBSplineControlPolygon(sPoly) ;

  // set knot arrays
  sKnotMults[0] = 4 ;
  sKnotMults[1] = 4 ;
  sKnots[0]     = dParamStart ;
  sKnots[1]     = dParamEnd ;

  // pass the call along
  SmStatus sRtn = CreateCanonical( crContext, 3, 3, sPoly, SM_CF_UNSPECIFIED,
                                   sKnotMults, sKnots, SM_KT_PIECEWISE_BEZIER_KNOTS,
                                   NULL, NULL, rpNewBSplineCurve) ;

  // all done
  return(sRtn) ;

} // end SmBSplineCurve::CreateFromHermiteCurve

/*******************************************************************//**
PURPOSE: Create a new SmBSplineCurve by extending an existing one a
         given real distance.

USAGE:   Use with caution and moderation. Do not attempt to give distances
         that are too large in relation end span.
***********************************************************************/
SmStatus SmBSplineCurve::CreateExtendedCurve // eff: extend parameter range of this curve
  (const SmContext & crContext,              // in : memory pool for output curve
   double dDistance,                         // in : additional length of extension
   int    dStartorEnd,                       // in : START (1) or END (2)
   SmContinuityType eExtensionContinuity,    // Either: SM_CT_G1        - linear extension
                                             //         SM_CT_G1_G2     - extension with second derivative
                                             //         SM_CT_CINFINITY - infinite continuity
   SmBSplineCurve *& rpExtended              // out: Constructed curve
   )
                                      
{
    // locals for upcoming NLIB call
    NL_CURVE      ecur;                                     // use: NLIB call output curve
    NL_CURVE     *cur = (NL_CURVE*)GetOrCreateGwNurbPointer(); // use: input curve
    NL_REAL       dist;                                     // use: distance extended
    NL_FLAG    gflg = NL_G1;                             // use: oneof G1/G2/CMAX
    NL_FLAG    AtStart  =  (NL_FLAG)dStartorEnd;         // use: at start=1, at end =2
    NL_STACKS     SG;                                       // use: global stack, cur's stack and used for ecur memory
    SmNLibStackHandler sSH(&SG);                         // use: call N_InitNurbs on SG stack 

    // initialize inputs for upcoming NLIB call
    dist = dDistance;
 
    if      (eExtensionContinuity == SM_CT_G1)        { gflg = NL_G1; }
    else if (eExtensionContinuity == SM_CT_G1_G2)     { gflg = NL_G2; }
    else if (eExtensionContinuity == SM_CT_CINFINITY) { gflg = NL_CMAX; }
    else { SER(SM_ERR); }

    // init ecur parameters to NULL CURVE
    N_CrvInitArrays(&ecur);

    NL_SER(N_CrvExtendByDist(cur, dist, NL_ABSOLUTE, AtStart, gflg, &ecur, &SG, &SG));

    // build output: new SMBSplineCurve object from ecur 
    if (ecur.pol == NULL) rpExtended = NULL;
    else               
    rpExtended = new (crContext) SmBSplineCurve(GetDim(),(gw_CURVE *)&ecur);
 
    // all done
    return SM_SUCCESS;

} // end SmBSplineCurve::CreateExtendedCurve

/*******************************************************************//**
PURPOSE: Create a new SmBSplineCurve by approximating a set of points
    and optionally corresponding vectors.  

NOTES: There may be no vectors,
    two vectors corresponding to start and end derivatives, or one 
    vector for each point.  
***********************************************************************/
SmStatus SmBSplineCurve::CreateApproximatingCurve
  (const SmContext            & crContext,                  // in : context for new object construction  
   ULONG                        lNumPoints,                 // in : Number of control points in new curve
   ULONG                        lParameterization,          // in : 0 - NL_UNIFORM, 1 = NL_CHORDLENGTH, 2 - NL_CENTRIPETAL
   ULONG                        lDimensionOfResult,         // in : Must be 2 or 3
   ULONG                        lDegree,                    // in : Must be 3 or higher
   const SmTArray<SmPoint3d>  & crPointsToInterpolate,      // in : List of points to approximate
   const SmTArray<SmVector3d> & crVectorsToInterpolate,     // in : optional end derivs
                                                            //      sized:[0] - no given end derivs
                                                            //      sized:[1] - start deriv only,     crVec[0] = start 1st deriv
                                                            //      sized:[2] - start and end derivs, crVec[0] = start 1st derivative, 
                                                            //                                        crVec[1] = end 1st deriv.
   const SmTArray<SmVector3d> * cpMoreVectorsToInterpolate, // in : sized:[2] - crMoreVec[0] = start 2nd deriv, only used when crVectorsToInterpolate sized:[1 or 2]
                                                            //                  crMoreVec[1] = end   2nd deriv, only used when crVectorsToInterpolate sized:[2]
                                                            //      sized:[4] - crMoveVec[2] = start 3rd deriv, only used when crVectorsToInterpolate sized:[1 or 2]
                                                            //                  crMoreVec[3] = end   3rd deriv, only used when crVectorsToInterpolate sized:[2]     
   SmBSplineCurve            *& rpNewBSplineCurve)          // out: approximating curve
{
  // NLib call locals
  NL_CURVE           cur ;
  NL_FLAG            parameterization =   (lParameterization == 1) ? NL_CHORDLENGTH
                                        : (lParameterization == 2) ? NL_CENTRIPETAL
                                        : NL_UNIFORM ;
  NL_POINT         * P  = SM_REINTERPRET_CAST(NL_POINT*, crPointsToInterpolate.GetDataArray()) ;
  NL_INDEX           kk = crPointsToInterpolate.GetSize() - 1 ;
  NL_INDEX           n  = lNumPoints - 1 ;
  NL_DEGREE          p  = (NL_DEGREE)lDegree ;
  NL_STACKS          SG ;
  SmNLibStackHandler sSH(&SG) ;
                  
  // init output - allocate and init to NULL, internal curve pointers 
  N_CrvInitArrays(&cur);

  // option 1: given two end tangents as constraints
  if (crVectorsToInterpolate.GetSize() == 2) 
    {
      // check input - range on degree
      if (lDegree < 2 || lDegree > NL_MAXDEG) 
        { SER(SM_ERR); }

      // set NLib call 1st derivative values
      //     Ds[1] = crVectorsToInterpolate[0] and
      //     De[1] = crVectorsToInterpolate[1]
      NL_VECTOR Ds[5], De[5];
      ULONG lCount = 1;
      COPY_XYZ(crVectorsToInterpolate[0],Ds[1]);        
      COPY_XYZ(crVectorsToInterpolate[1],De[1]); 
             
      // when given, set NLib call 2nd derivative values
      //     Ds[2] = cpMoreVectorsToInterpolate[0] and
      //     De[2] = cpMoreVectorsToInterpolate[1]
      if (cpMoreVectorsToInterpolate && cpMoreVectorsToInterpolate->GetSize() > 1) 
        {
          COPY_XYZ((*cpMoreVectorsToInterpolate)[0],Ds[2]);        
          COPY_XYZ((*cpMoreVectorsToInterpolate)[1],De[2]);
          lCount = 2;
          
          // when given, set NLib call 3rd derivative values
          //     Ds[3] = cpMoreVectorsToInterpolate[2] and
          //     De[3] = cpMoreVectorsToInterpolate[3]
          if ((*cpMoreVectorsToInterpolate).GetSize() > 3) 
            {
              COPY_XYZ((*cpMoreVectorsToInterpolate)[2],Ds[3]);        
              COPY_XYZ((*cpMoreVectorsToInterpolate)[3],De[3]);
              lCount = 3;
            }
        }

      // pass call along
      NL_SER(N_FitCrvApproxDerivs(P,                   // in : points to be approximated
                                  kk,                  // in : highest index in points array
                                  Ds,                  // in : start derivatives, Ds[1] = 1st, Ds[2] = 2nd, Ds[3] = 3rd
                                  lCount,              // in : highest given start derivative index, 1 for 1st, 2 for 2nd...
                                  De,                  // in : end derivatives, Ds[1] = 1st, Ds[2] = 2nd, Ds[3] = 3rd     
                                  lCount,              // in : highest given start derivative index, 1 for 1st, 2 for 2nd...
                                  n,                   // in : highset control point index for output cur
                                  p,                   // in : degree for output cur
                                  parameterization,    // in : NL_UNIFORM     = Uniform parametrization     
                                                       //      NL_CHORDLENGTH = Chord length parametrization
                                                       //      NL_CENTRIPETAL = Centripetal parametrization 
                                  &cur,                // out: Approximating curve
                                  &SG));               // in : cur's memory stack

      //        NL_SER(N_FitCrvDerivs(P,kk,(NL_DEGREE)lDegree,Ds,De,parameterization,&cur,&SG));
    } // end end given two end tangens as constraint branch

  // option 2: given one Vector as constraint branch
  else if (crVectorsToInterpolate.GetSize() == 1) 
    { 
      // set NLib call 1st derivative values
      //     Ds[1] = crVectorsToInterpolate[0] and
      NL_VECTOR Ds[5];
      COPY_XYZ(crVectorsToInterpolate[0],Ds[1]);
      ULONG lCount = 1;

      // when given, set NLib call 2nd derivative values
      //     Ds[2] = cpMoreVectorsToInterpolate[0] and
      if (cpMoreVectorsToInterpolate && cpMoreVectorsToInterpolate->GetSize() > 0) 
        {
          COPY_XYZ((*cpMoreVectorsToInterpolate)[0],Ds[2]);        
          lCount = 2;

          // when given, set NLib call 3rd derivative values
          //     Ds[3] = cpMoreVectorsToInterpolate[2] and
          if ((*cpMoreVectorsToInterpolate).GetSize() > 2) 
            {
              COPY_XYZ((*cpMoreVectorsToInterpolate)[2],Ds[3]);        
              lCount = 3;
            }
        }  
      
      // pass call to NLib      
      NL_SER(N_FitCrvApproxDerivs(P,                 // in : points to be approximated                                    
                                  kk,                // in : highest index in points array                                
                                  Ds,                // in : start derivatives, Ds[1] = 1st, Ds[2] = 2nd, Ds[3] = 3rd     
                                  lCount,            // in : highest given start derivative index, 1 for 1st, 2 for 2nd...
                                  Ds,                // in : end derivatives, Ds[1] = 1st, Ds[2] = 2nd, Ds[3] = 3rd       
                                  0,                 // in : highest given start derivative index, 1 for 1st, 2 for 2nd...
                                  n,                 // in : highset control point index for output cur                   
                                  p,                 // in : degree for output cur                                        
                                  parameterization,  // in : NL_UNIFORM     = Uniform parametrization                     
                                                     //      NL_CHORDLENGTH = Chord length parametrization                
                                                     //      NL_CENTRIPETAL = Centripetal parametrization                 
                                  &cur,              // out: Approximating curve                                          
                                  &SG));             // in : cur's memory stack 
                                                                            
    } // end given one end tangent as constraint branch

  // option 3: no Vectors to interpolate
  else if (crVectorsToInterpolate.GetSize() == 0) 
    {
      if (lDegree < 2 || lDegree > NL_MAXDEG) 
        { SER(SM_ERR); }
      NL_VECTOR Ds[5], De[5];
      NL_SER(N_FitCrvApproxDerivs(P,                 // in : points to be approximated                                     
                                  kk,                // in : highest index in points array                                 
                                  Ds,                // in : start derivatives, Ds[1] = 1st, Ds[2] = 2nd, Ds[3] = 3rd      
                                  0,                 // in : highest given start derivative index, 1 for 1st, 2 for 2nd... 
                                  De,                // in : end derivatives, Ds[1] = 1st, Ds[2] = 2nd, Ds[3] = 3rd        
                                  0,                 // in : highest given start derivative index, 1 for 1st, 2 for 2nd... 
                                  n,                 // in : highset control point index for output cur                    
                                  p,                 // in : degree for output cur                                         
                                  parameterization,  // in : NL_UNIFORM     = Uniform parametrization                      
                                                     //      NL_CHORDLENGTH = Chord length parametrization                 
                                                     //      NL_CENTRIPETAL = Centripetal parametrization                  
                                  &cur,              // out: Approximating curve                                           
                                  &SG));             // in : cur's memory stack                                            
    }
  else // unsupported option
    {
      SER(SM_ERR);
    }

  // set output
  rpNewBSplineCurve = new (crContext) SmBSplineCurve(lDimensionOfResult,(gw_CURVE *)&cur);

  // all done
  return SM_SUCCESS;

} // end SmBSplineCurve::CreateApproximatingCurve

/*******************************************************************//**
PURPOSE: Create a new SmBSplineCurve by interpolating a set of points
    and corresponding optional tangent vectors.  

NOTES: There may be no vectors,
    two vectors corresponding to start and end derivatives, or one 
    vector for each point.  If there is one vector for each point you
    may specify that the magnitude of the vectors is ignored.  
    
      This method is only available for users with NLib
    
***********************************************************************/
SmStatus SmBSplineCurve::CreateInterpolatingCurve            
  (const SmContext             & crContext,                  // in : context for new object construction
   SmCurveParameterizationType   eParameterization,          // in : How to parameterize between points
                                                             //      0 - SM_CP_UNIFORM, 
                                                             //      1 - SM_CP_CHORDLENGTH, 
                                                             //      2 - SM_CP_CENTRIPETAL
   ULONG                         lDimensionOfResult,         // in : Must be 2 or 3
   ULONG                         lDegree,                    // in : Must be 2 or 3 if we are interpolating
                                                             //      with a full set of vectors.  It can be 2 or higher
                                                             //      if we are just using end vectors or no vectors.
   const SmTArray<SmPoint3d>   & crPointsToInterpolate,      // in : array of target points
   const SmTArray<SmVector3d>  & crVectorsToInterpolate,     // in : May contain No vectors, 
                                                             //                  2 vectors - a start and an end vector,
                                                             //               or 1 vector for each point to interpolate.
   const SmTArray<SmVector3d>  * cpMoreVectorsToInterpolate, // in : If this exists then it contains higher order derivatives (2nd and 3rd)
   SmBoolean                     bIgnoreVectorMagnitude,     // in : This only applies to case where more than two vectors are used.
                                                             //      TRUE = use vector directions - not their magnitudes
   SmBSplineCurve             *& rpNewBSplineCurve)          // out: the newly constructed curve
{
  NL_CURVE              cur;
  NL_STACKS             SG;
  SmNLibStackHandler sSH(&SG); 
  N_CrvInitArrays(&cur);

  NL_FLAG parameterization =   (eParameterization == SM_CP_CHORDLENGTH) ? NL_CHORDLENGTH
                          : (eParameterization == SM_CP_CENTRIPETAL) ? NL_CENTRIPETAL
                          : NL_UNIFORM;

  if (   lDegree == 1 
      && crPointsToInterpolate.GetSize() == 2) 
    {
      SER(SmBSplineCurve::CreateLineSegment(crContext,lDimensionOfResult,crPointsToInterpolate[0],
          crPointsToInterpolate[1],rpNewBSplineCurve));
      return SM_SUCCESS;
    }

  if (   lDimensionOfResult != 2 
      && lDimensionOfResult != 3) 
    {
      SER(SM_ERR);
    }

  NL_POINT *P = SM_REINTERPRET_CAST(NL_POINT*,crPointsToInterpolate.GetDataArray());
  NL_INDEX kk = crPointsToInterpolate.GetSize() - 1;

  // branch to construct new gw_CURVE
  if (   crVectorsToInterpolate.GetSize() == crPointsToInterpolate.GetSize() 
      && crPointsToInterpolate.GetSize() > 2) 
    {
      if (lDegree != 2 && lDegree != 3) {
          SER(SM_ERR);
      }
      NL_VECTOR *D = SM_REINTERPRET_CAST(NL_VECTOR*,crVectorsToInterpolate.GetDataArray());
      NL_FLAG vec = NL_DERIVATIVE;
      if (bIgnoreVectorMagnitude) vec = NL_TANGENT;
      NL_SER(N_FitCrvFirstDeriv(P,D,kk,(NL_DEGREE)lDegree,vec,parameterization,&cur,&SG));
    }
  else if (crVectorsToInterpolate.GetSize() == 2) 
    {
      if (lDegree < 2 || lDegree > NL_MAXDEG) { SER(SM_ERR); }
      NL_VECTOR Ds[5], De[5];
      ULONG lCount = 1;
      COPY_XYZ(crVectorsToInterpolate[0],Ds[1]);        
      COPY_XYZ(crVectorsToInterpolate[1],De[1]);        
      if (cpMoreVectorsToInterpolate && cpMoreVectorsToInterpolate->GetSize() > 1) {
          COPY_XYZ((*cpMoreVectorsToInterpolate)[0],Ds[2]);        
          COPY_XYZ((*cpMoreVectorsToInterpolate)[1],De[2]);
          lCount = 2;
          if (lDegree <= 2) lDegree = 3;
          if ((*cpMoreVectorsToInterpolate).GetSize() > 3) {
              COPY_XYZ((*cpMoreVectorsToInterpolate)[2],Ds[3]);        
              COPY_XYZ((*cpMoreVectorsToInterpolate)[3],De[3]);
              lCount = 3;
              if (lDegree <= 3) lDegree = 4;
          }
      }
      NL_SER(N_FitCrvHighDerivs(P,kk,(NL_DEGREE)lDegree,Ds,lCount,De,lCount,parameterization,&cur,&SG));
      //        NL_SER(N_FitCrvDerivs(P,kk,(NL_DEGREE)lDegree,Ds,De,parameterization,&cur,&SG));
    }
  else if (crVectorsToInterpolate.GetSize() == 1) 
    { // Start Vector 
      NL_VECTOR Ds[5];
      COPY_XYZ(crVectorsToInterpolate[0],Ds[1]);
      ULONG lCount = 1;
      if (cpMoreVectorsToInterpolate && cpMoreVectorsToInterpolate->GetSize() > 0) {
          COPY_XYZ((*cpMoreVectorsToInterpolate)[0],Ds[2]);        
          lCount = 2;
      }        
      NL_SER(N_FitCrvHighDerivs(P,kk,(NL_DEGREE)lDegree,Ds,lCount,Ds,0,parameterization,&cur,&SG));
    }
  else if (crVectorsToInterpolate.GetSize() == 0) 
    {
      NL_VECTOR Ds[5], De[5];
      NL_SER(N_FitCrvHighDerivs(P,kk,(NL_DEGREE)lDegree,Ds,0,De,0,parameterization,&cur,&SG));
    }
  else 
    {
      SER(SM_ERR);
    }

  // construct output from new gw_CURVE
  rpNewBSplineCurve = new (crContext) SmBSplineCurve(lDimensionOfResult,(gw_CURVE *)&cur);
  return SM_SUCCESS;

} // end SmBSplineCurve::CreateInterpolatingCurve

/*******************************************************************//**
PURPOSE: This routine computes a C1 continuous non-rational cubic
    NURBS curve interpolating a given set of points. 

NOTES: This method is only available for users with NLib
    
***********************************************************************/
SmStatus SmBSplineCurve::C1NonRationalCubicInterpolate
  (const SmContext & crContext,
   const SmTArray<SmPoint3d> & crPoints,
   SmBSplineCurve *& rpNewCurve)
{
    NL_STACKS SG;
    SmNLibStackHandler sSH(&SG);
    NL_CURVE   cur;
    N_CrvInitArrays(&cur);

    NL_INDEX  m = crPoints.GetSize()-1;
    NL_POINT  *P = N_AllocPt1dArray(m,&SG);

    for (ULONG i=0; i<crPoints.GetSize(); i++) {
        SmPoint3d sPnt = crPoints[i];
        COPY_XYZ(sPnt,P[i]);
    }

    NL_SER(N_FitCrvCubic(P,m,NL_AKIMA,&cur,&SG));

    rpNewCurve = new (crContext) SmBSplineCurve(3,(gw_CURVE *)&cur);
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
    if (bDebugMe) {
        sm_GraphicsLoop();
        smgfx_SetColor(1,0,0);
        rpNewCurve->Draw();
        rpNewCurve->Dump();
    }
#endif // end SM_DEBUG_CODE

    return SM_SUCCESS;

} // end SmBSplineCurve::C1NonRationalCubicInterpolate

/*******************************************************************//**
PURPOSE: This method will smooth out the spacing differential 
     between knots.  

NOTES: If the difference between two consecutive 
     spans is greater than the Maximum Delta Factor we will insert
     lMultiplicityOfNewKnots knots into the knot vector of the curve
     to split the span into two equal segments.
***********************************************************************/
SmStatus SmBSplineCurve::DampenKnotSpacing
  (double dMaximumDeltaFactor,
   ULONG lMultiplicityOfNewKnots)
{
    // prepare the object
    Notify(SM_NO_PRE_EDIT, this, SM_NO_GET_OWNER(this), NULL) ;

    if(m_pNurb == NULL) { MakeNurb(); } SM_ASSERT(m_pNurb != NULL) ;
    SmTArray<double> sNewKnots;
    SmTArray<double> sNewKnotVector;

    SER(GetKnots(sNewKnotVector));

    SmBoolean bDone = FALSE;
    while (!bDone) {
        bDone = TRUE;
        for (ULONG i=0; i+1<sNewKnotVector.GetSize(); i++) { // note: can't say sNewKnotVector.GetSize()-1
            ULONG bSplit = FALSE;
            double dSpanSize = sNewKnotVector[i+1] - sNewKnotVector[i];
            if (i > 0) {
                // Look at size of previous span
                double dLastSpanSize = sNewKnotVector[i] - sNewKnotVector[i-1];
                if (dSpanSize/dLastSpanSize > dMaximumDeltaFactor) {
                    bSplit = TRUE;
                }
            }
            if (i < sNewKnotVector.GetSize()-2) {
                // Look at size of next span
                double dNextSpanSize = sNewKnotVector[i+2] - sNewKnotVector[i+1];
                if (dSpanSize/dNextSpanSize > dMaximumDeltaFactor) {
                    bSplit = TRUE;
                }
            }
            // See if we need to split this span
            if (!bSplit) continue;

            // Split the span
            bDone = FALSE;
            double dSplitKnot = (sNewKnotVector[i+1] + sNewKnotVector[i]) / 2.0;
            sNewKnotVector.InsertAt(i+1,dSplitKnot);
            ULONG lInsertPoint = sNewKnots.GetSize();
            for (ULONG jj=0; jj<sNewKnots.GetSize(); jj++) {
                if (sNewKnots[jj] > dSplitKnot) {
                    lInsertPoint = jj;
                    break;
                }
            }
            sNewKnots.InsertAt(lInsertPoint,dSplitKnot,lMultiplicityOfNewKnots);
        }
    }

    if (sNewKnots.GetSize() > 0) 
      {
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
        if (bDebugMe) 
          {
            this->Dump();
            sNewKnots.Dump();
          }
#endif // end SM_DEBUG_CODE

        SER(InsertKnots(sNewKnots));
      }
    
    // inform the object
    Notify(SM_NO_POST_EDIT, this, SM_NO_GET_OWNER(this), NULL) ;
    
    // all done        
    return SM_SUCCESS;

} // end SmBSplineCurve::DampenKnotSpacing

/*******************************************************************//**
PURPOSE: This routine reduces the degree of a NURBS curve. 

NOTES: 
    The new knot must be an interior knot and the sum of the multiplicities
    of the old and the new knots must be less than or equal to the degree.
  
    This method is only available for users with NLib
    
***********************************************************************/
SmStatus SmBSplineCurve::DegreeReduction
  (double dTolerance,
   SmBoolean bMaxReduce)   // If TRUE, will reduce the degree as much as possible
{
    NL_STACKS SG;
    SmNLibStackHandler sSH(&SG);
    NL_CURVE   curNew;
    N_CrvInitArrays(&curNew);

    NL_CURVE  *curP = (NL_CURVE*)GetOrCreateGwNurbPointer();

    if (bMaxReduce) {
        NL_SER(N_CrvReduceDegree(curP,dTolerance,&curNew,&SG,&SG));
    }
    else {
        NL_FLAG  rfl;
        NL_REAL  mtol;
        NL_SER(N_CrvReduceDegreeOnce(curP,dTolerance,&rfl,&curNew,&mtol,&SG,&SG));
        if (rfl == NL_NO) {
            MSG(_T("Degree reduction failure"));
            SER(SM_ERR);
        }
    }

    Notify(SM_NO_PRE_EDIT, this, SM_NO_GET_OWNER(this), NULL);
    SmBSplineCurve *pTmpBSC = new (*GetContext()) SmBSplineCurve(GetDim(),(gw_CURVE *)&curNew);
    SmObjDelete sClean(pTmpBSC);

    // Swap Nurbs with this
    gw_CURVE *pTmp   = pTmpBSC->m_pNurb;
    pTmpBSC->m_pNurb = m_pNurb;
    m_pNurb          = pTmp;
    Notify(SM_NO_POST_EDIT, this, SM_NO_GET_OWNER(this), NULL);

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
    if (bDebugMe) {
        sm_GraphicsLoop();
        smgfx_SetColor(1,0,0);
        Draw();
        Dump();
    }
#endif // end SM_DEBUG_CODE

    return SM_SUCCESS;

} // end SmBSplineCurve::DegreeReduction

/*******************************************************************//**
PURPOSE: This routine forces the curve to interpolate a given point.

NOTES: 
  This method does not add or subtract any knots or control points.
  It does not change the knot vector, only some control points.

  The point should be interior to the curve.  If it is at or beyond an end,
  then only the end control point will be modified.

  The curve is modified only locally, using an algorithm that minimizes
  the movement of the curve.

  The algorithm moves control points that influence the curve at the given
  parameter, and no others.  One consequence of this is, if the point is
  relatively far from the curve, and there are lots of control points
  (i.e., the point is farther away than typical control point spacing),
  the curve will just bump out to interpolate the point.  Another consequence
  is, if this algorithm is applied with more than one point, and the points
  are close enough together to be influenced by the same control points,
  then interpolating the second point might move the curve where the first
  point was interpolated.  For that case, you might call this iteratively
  until the curve is no longer modified.

  If there are multiple interior knots, this routine can result in
  a kink in the surface.

  This method is only available for users with NLib.
    
***********************************************************************/
SmStatus SmBSplineCurve::ForceThroughPoint(
            const SmPoint3d &crPoint,       // in
                  SmBoolean &rbWasModified, // out: did we modify the curve at all?
                  double    *pdParam,       // in, opt: parameter to put the point at
                  double    *pdGuessParam,  // in, opt: if *pdParam is Null, guess param
                                            //          for DropPoint.  Ignored if
                                            //          pdParam is not Null.
                  double     dTol           // in, default SM_EFF_ZERO
        )
{
  // Init outputs.
  rbWasModified = FALSE;

  // First, get a curve parameter, evaluate there, and check the distance.
  double dParam, dDist;
  SmPoint3d sCrvPt;
  SmBoolean bSuccess = TRUE;
  if ( pdParam != NULL )
    {
      dParam = *pdParam;
      EvaluatePoint( dParam, sCrvPt );
      dDist = sCrvPt.DistanceBetween( crPoint );
    }
  else
    {
      DropPoint(GetNaturalInterval(), // in : target curve allowed domain
                crPoint,              // in : Point to drop to curve
                NULL,                 // in : For Snapping, NULL=NoSnap, Vec pointing to CrvInside from desired EndPt for snapping.
                                      //      when DropPt is within dDistTol in cpVecToCrvInside dir of EndPt, Snap to Ivl EndPt.
                                      //      Vec handles ambiguities and makes sure that curves are not just touching at the ends.
                SM_BIG_DOUBLE,        // in : Used for SnapDist, and SM_SO_INTERSECT sXSectTol3d distance.
                                      //      sXSectTol3d = dDistTol ;      SM_SO_INTERSECT 
                                      //      sXSectTol3d = SM_BIG_DOUBLE ; SM_SO_MINIMIZE, SM_SO_NORMALIZE
                                      //      sSnapTol3d  = dDistTol ;      SM_SO_MINIMIZE, SM_SO_NORMALIZE, SM_SO_INTERSECT    
                pdGuessParam,         // in : If given, call cheap LocalPointSolve() else call expensive GlobalPointSolve()
                bSuccess,             // out: TRUE = found a drop point
                dParam,               // out: found drop curve param
                dDist) ;              // out: found drop distance
                                      // in : SM_SO_MINIMIZE = allow nonNormal drops near endPoints
                                      //      SM_SO_INTERSECT= like Minimize but point must be within dDistTol
                                      //      SM_SO_NORMALIZE= exclude nonNormal drops near endPoints
                                      //      default:[SM_SO_MINIMIZE] to preserve original behavior
    }

  if ( ! bSuccess   ) { return SM_ERR; }
  if ( dDist < dTol ) { return SM_SUCCESS; }

  // We're going to modify the curve.  Set up for the NLib call.
  rbWasModified = TRUE;

  NL_CURVE *nl_CurPtr = (NL_CURVE*)this->GetOrCreateGwNurbPointer();
  NL_POINT nl_Pt;
  COPY_XYZ( crPoint, nl_Pt );

  NL_BOOLEAN nl_UseFixParam   = ( pdParam == NULL )      ? NL_FALSE : NL_TRUE;
  NL_REAL    nl_FixParam      = ( pdParam == NULL )      ? 0.0      : *pdParam;
  NL_BOOLEAN nl_UseGuessParam = ( pdGuessParam == NULL ) ? NL_FALSE : NL_TRUE;
  NL_REAL    nl_GuessParam    = ( pdGuessParam == NULL ) ? 0.0      : *pdGuessParam;

  NL_FLAG nl_flag = N_CrvFitToPt( nl_CurPtr, nl_Pt, nl_UseFixParam, nl_FixParam, nl_UseGuessParam, nl_GuessParam );

  return ( nl_flag == NL_NO ) ? SM_SUCCESS : SM_ERR;

} // end SmBSplineCurve::ForceThroughPoint

/*******************************************************************//**
PURPOSE: This routine inserts a new knot into a NURBS curve. 

NOTES: The new knot must be an interior knot and the sum of the multiplicities
     of the old and the new knots must be less than or equal to the degree. 

  This method is only available for users with NLib
    
***********************************************************************/
SmStatus SmBSplineCurve::InsertOneKnot
  (double dNewKnot,                   // in : knot value
   ULONG lNumKnotInsertions)          // in : Number of times of insertions
{
  NL_STACKS SG;
  SmNLibStackHandler sSH(&SG);
  NL_CURVE  curNew;
  N_CrvInitArrays(&curNew);

  NL_CURVE  *curP = (NL_CURVE*)GetOrCreateGwNurbPointer();

  NL_INDEX  r = lNumKnotInsertions;
  NL_SER(N_CrvInsertKnot(curP,dNewKnot,r,&curNew,&SG,&SG));

  Notify(SM_NO_PRE_EDIT, this, SM_NO_GET_OWNER(this), NULL);
  SmBSplineCurve *pTmpBSC = new (*GetContext()) SmBSplineCurve(GetDim(),(gw_CURVE *)&curNew);
  SmObjDelete sClean(pTmpBSC);

  // Swap Nurbs with this
  gw_CURVE *pTmp   = pTmpBSC->m_pNurb;
  pTmpBSC->m_pNurb = m_pNurb;
  m_pNurb          = pTmp;
  Notify(SM_NO_POST_EDIT, this, SM_NO_GET_OWNER(this), NULL);

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
  if (bDebugMe) 
    {
      sm_GraphicsLoop();
      smgfx_SetColor(1,0,0);
      Draw();
      Dump();
    }
#endif // end SM_DEBUG_CODE

  return SM_SUCCESS;

} // end SmBSplineCurve::InsertOneKnot

/*******************************************************************//**
PURPOSE: Make a BSplineCurve non-rational by dividing by the weight

NOTES:  
    This is a simple data conversion. It modifies the shape of the Curve.
    It is not a fitting process.
    This method is only available for users with NLib
    
***********************************************************************/
SmStatus SmBSplineCurve::MakeNonRational()
{
    // inform the object
    Notify(SM_NO_PRE_EDIT, this, SM_NO_GET_OWNER(this), NULL) ;

    NL_CPOINT *Cp;
    NL_CURVE * cur = GetOrCreateGwNurbPointer();
    NER(cur);
    NL_INDEX n = cur->pol->n;
    for (NL_INDEX ii = 0; ii <= n; ii++)
    {
            Cp = &(cur->pol->Pw[ii]);
            if (Cp->w !=NL_NOW)
            {
                NL_REAL w = Cp->w;
                Cp->x /= w;
                Cp->y /= w;
                if (Cp->z != NL_NOZ) Cp->z /= w;
                Cp->w = NL_NOW;
            }
    }

    // Warn
    if(IsKindOf(SmEllipse_TYPE))
      {
        ERR_MSG(_T("MakeNonRational modified the shape of an ellipse/circle object")) ;
      }

    // inform the object
    Notify(SM_NO_POST_EDIT, this, SM_NO_GET_OWNER(this), NULL) ;

    return SM_SUCCESS;

} // end SmBSplineCurve::MakeNonRational

/***********************************************************************
PURPOSE --- Multiply a curve by a constant.

USAGE NOTES --- 
   This would presumably be used on curves that represent a vector field,
   not actual 3d curve.  For a 3d curve, it would have the effect of
   scaling the curve with respect to the origin.

   Modifies 'this' in place.

   This method is only available for users with NLib.

***********************************************************************/
SmStatus SmBSplineCurve::Multiply( double dScale )
{
  // prepare the object
  Notify(SM_NO_PRE_EDIT, this, SM_NO_GET_OWNER(this), NULL);

  if(m_pNurb == NULL) 
    { MakeNurb(); } SM_ASSERT(m_pNurb != NULL);

  N_ConstantMultiplyCrv( dScale, m_pNurb );

  // Give derived Analytic curves a chance to update their 'step' data.
  RefreshAnalytics();

  // inform the object
  Notify(SM_NO_POST_EDIT, this, SM_NO_GET_OWNER(this), NULL);

  return SM_SUCCESS;

} // end SmBSplineCurve::Multiply

/***********************************************************************
PURPOSE --- Multiply a curve by a linear function.

USAGE NOTES --- 
   This would presumably be used on curves that represent a vector field,
   not actual 3d curve.  For a 3d curve, it would have the effect of
   scaling the curve with respect to the origin.

   Modifies 'this' in place.

   This method is only available for users with NLib.

***********************************************************************/
SmStatus SmBSplineCurve::MultiplyLinear
 (double dScale0, 
  double dScale1 )
{
  // prepare the object
  Notify(SM_NO_PRE_EDIT, this, SM_NO_GET_OWNER(this), NULL);

  if ( m_pNurb == NULL ) { MakeNurb(); } SM_ASSERT(m_pNurb != NULL);

  N_LinearMultiplyCrv( dScale0, dScale1, m_pNurb );

  // Give derived Analytic curves a chance to update their 'step' data.
  RefreshAnalytics();

  // inform the object
  Notify(SM_NO_POST_EDIT, this, SM_NO_GET_OWNER(this), NULL);

  return SM_SUCCESS;

} // end SmBSplineCurve::MultiplyLinear

/*******************************************************************//**
PURPOSE: Convenience function for getting 3d distance between two
            NL Control Points

NOTES:
***********************************************************************/
static double sm_GetControlPointSpacing
  (NL_CPOINT &rCPoint1, 
   NL_CPOINT &rCPoint2)
{
  NL_CPOINT sCVec ;
  NL_POINT  sVec ; 
  
  // make NL calls to get 3d vector between two control points 
  N_Diff2CPts(rCPoint1, rCPoint2, &sCVec) ; 
  N_CPtToPt(sCVec, &sVec) ;

  // NL vector's magnitude
  double dSpacing = smos_Sqrt(sVec.x * sVec.x + sVec.y * sVec.y + sVec.z * sVec.z) ;

  // all done
  return(dSpacing) ;

} // end sm_GetControlPointSpacing

/*******************************************************************//**
PURPOSE: This routine forces a piecewise C0 NURB curve
  (as output by the surface/surface intersection algorithm)
  to interpolate a given through-point, parameter value,
  and associated tangent.  Input and output curve interval
  ranges are preserved.

NOTES:
  The through point parameter value may be above, within, or below the
  curve's current parameter range.  This function is intended to have a
  limited application.  It is designed to add one more through point to an
  intersection curve.  Intersection curves are currently created as piecewise
  C0 'bezier' curves where each segment endPoint is already a throughPoint
  on the intersection.  Applying this function to force a curve to interpolate
  an arbitrary point can add small dog-legs, bumps, or hooks to force a curve
  to locally deform to hit the target point.  The curve will only look 'nice'
  when the input through point is properly positioned.
    
METHOD ---
  Interpolate the given point by associating it with a knot whose
  multiplicity = degree.  Existing knots are used if the input 
  target dNewKnot value is close to an existing knot otherwise new
  knots are added to the curve by knot insertion or by extending the curve.
  The control point associated with the associated knot is moved to the 
  specified location.  All the control points affecting the 
  segments on either side of the target through point are updated as well to
  preserve the curve's existing position and tangent values at those
  segment end points.

  CASES:  1. *  +-----+-----+    = dNewKnot value below curve range - add a segment - update first 2 segments
          2.   *+-----+-----+    = dNewKnot value below and close to start knot - update first segment
          3.    +*----+-----+    = dNewKnot value in first segment and close to bottom knot - update first segment
          4.    +--*--+-----+    = dNewKnot value splits an existing segment - insert knot - update 2 segments
          5.    +----*+-----+    = dNewKnot value close to an internal knot - update two segments
                +-----+*----+
          6.    +-----+----*+    = dNewknot value in last segment and close to top knot - update last segment
          7.    +-----+-----+*   = dNewKnot value above and close to end knot - update last segment
          8.    +-----+-----+  * = dNewKnot value above curve range - add a segment - update first 2 segments

          where + = a through point
                * = the through point to be inserted

  1. Check state - this function only works for curves which 
      pass the IsPiecewiseC0() check.  Because we assume that the
      curve already interpolates a series of through points represented
      by knots with mulitplicity = degree.  
      NOTE: we only need fully multiple knots on the segment boundaries
            bracketing the inserted point.  We could create another
            function which promoted boundary knots to full multiplicity.
            However, in general this will introduce geometrically small
            dog-legs and hooks in the curve to allow it to interpolate the
            given point.  The intent in this assumption is that we
            are working with intersection curves - and adding another interpolated
            point on the intersection won't introduce small bumps into the curve.
      
  2. find the knot which starts the lowest segment about to be modified.
  3. figure out which case we are in.
  4. Prepare a 3 point through point array containing, parameter, position and tangent values for the segments
     about to be modified.
  5. If case == 1, 4, or 8 insert a knot into the curve (we insert into a copy and straighten out structure pointers later on) 
  6. Modify the control point positions for the segments being modified. 
***********************************************************************/
SmStatus SmBSplineCurve::SetIntersectionCurveThroughPoint
 (double                     & dNewKnot,            // i/o: suggested param value updated to actual param value 
  const SmPoint3d            & crThrough3dPoint,    // in : through point position
  const SmVector3d           * cpOpt3dTangent,      // in : optional tangent value, NULL to ignore
  SmTArray<SmBSplineCurve*>  & rUVCurve,            // in : UVTrimCurves associated with this curve, expected (not required) to equal number of surfaces
  SmTArray<const SmSurface*> & rSurface,            // in : corresponding surfaces for each pUVCurve
  SmTArray<SmVector2d*>      & rOptUVPoint,         // in : UVPoint on pSurface corresponding to Through3dPoint,
                                                    //      A NULL entry=project crThrough3dPoint onto pSurface
  SmBoolean                  & bModifiedCurve,      // out: TRUE=modified curve, FALSE=didn't
  double                       dMinimunSpanPercent) // in : smallest allowed size for new spans.
                                                    //      A curve span is either split and updated or 
                                                    //      just updated.  Splitting is skipped when the split
                                                    //      point would create a span less than dMinimunSpanPercent
                                                    //      of the current span size.
{
  // init outputs
  bModifiedCurve = FALSE ;
  ULONG ii, jj;

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
static ULONG lCount = 1 ; lCount++ ;
  // draw input: Curve(cyan), ThroughPoint(red), NearPoint(Magenta), SamplePoints(cyan), ControlPoints(cyan)
  if(bDebugMe) 
    { 
      const SmEdge *pEdge = GetEdge() ;
      SmBrep *pBrep = pEdge ? pEdge->GetBrep() : NULL ;

      SmPoint3d sNearPoint, sTPoint ;
      EvaluatePoint(dNewKnot, sNearPoint) ; // suggested param value
      SmExtent1d sIvl = GetNaturalInterval();
      double dParam   =  (  fabs(sIvl.GetMin()-dNewKnot) < fabs(sIvl.GetMax()-dNewKnot))
                        ? sIvl.GetMin() 
                        : sIvl.GetMax() ;
      double dInc = (dNewKnot - dParam)/20 ;
      if(fabs(dInc) < 1.0e-8) 
        { if(dInc >= 0.0 ) dInc =  1.0e-8 ;
          else             dInc = -1.0e-8 ;
        }

      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ; 
      smgfx_SetLook(1,2, 0,1,1) ; DrawCurvature(-35,175) ; sm_GraphicsLoop() ;
      smgfx_SetLook(3,4, .2,1,1); DrawControlPoints() ; sm_GraphicsLoop() ;
      smgfx_SetLook(2,3, 0,.2,1); for (ULONG i=0; i<=40; i++,dParam+=dInc) 
                                    { EvaluatePoint(dParam,sTPoint);
                                      sTPoint.Draw(); sm_GraphicsLoop() ;
                                    }
      smgfx_SetLook(3,4, 1,0,0) ; crThrough3dPoint.Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 1,0,0) ; if(cpOpt3dTangent) { cpOpt3dTangent->Draw(&crThrough3dPoint) ; } sm_GraphicsLoop() ;
      smgfx_SetLook(3,4, 1,0,1) ; sNearPoint.Draw() ; sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

  // check state - Quit if the curve is not a piecewise C0, degree 3 curve
  if(!IsIntersectorApprox()) 
    { return(SM_ERR) ; }

#ifdef SM_DEBUG_CODE
  // check state - this is a 3d curve and all input array sizes match 
  SM_ASSERT(   GetDim() == 3
            && rUVCurve.GetSize() <= rSurface.GetSize()
            && rSurface.GetSize() == rOptUVPoint.GetSize()) ;

  // check state - UVTrimCurves have a surface and share parameterization with this curve
  for(ii=0;ii<rUVCurve.GetSize();ii++) 
    { SM_ASSERT(   rUVCurve[ii]->GetDim() == 2
                && rUVCurve[ii]->GetNumberNaturalKnots() == GetNumberNaturalKnots()
                && rSurface[ii] != NULL) ;
    }
#endif // end SM_DEBUG_CODE

  // locals
  NL_STACKS SG;
  SmNLibStackHandler sSH(&SG);
  SmExtent1d sInterval     = GetNaturalInterval() ;
  ULONG      lUVCurveCount = rUVCurve.GetSize() ;
  long       lCase         = -1 ;
  double     dSpanSize     = 0.0 ;
  NL_INDEX      iSpn1         = -1 ;
  NL_INDEX      iSpn2         = -1 ;
  NL_CURVE     *pCrv          = GetOrCreateGwNurbPointer() ;
  NL_CPOINT    *pW            = pCrv->pol->Pw ;
  NL_REAL      *pU            = pCrv->knt->U ;
  // NL_INDEX      iCPMaxIndex   = pCrv->pol->n ;
  NL_DEGREE     lDegree       = pCrv->p ;
  NL_CURVE      sNewCrv ;
//  NL_INDEX  lNewM ;
//  NL_REAL  *pNewU, dEndU ;

#ifdef SM_DEBUG_CODE
  ULONG      lDim = GetDim();
  SM_ASSERT(lDim == 3) ;
#endif // SM_DEBUG_CODE

  SmTArray<NL_CURVE *> sNewUVCrv ;  sNewUVCrv.SetSize(lUVCurveCount) ; // the stuff these contained pointers eventually point 
                                                                    // to are not leaked because they are cleaned up as part
                                                                    // of the NLib stack cleanup when the SG stack goes away.
  SmTArray<NL_CURVE  *> pUVCrv ;
  SmTArray<NL_CPOINT *> pUV ;

#ifdef SM_DEBUG_CODE
  // copy the input curves for later debug checks
  SmBSplineCurve *pEdit3DCurve = this ;
  SmBSplineCurve *pCopy3DCurve = new (*GetContext()) SmBSplineCurve(*this);
  SmObjDelete sCopyClean(pCopy3DCurve);

  SmTArray<SmBSplineCurve*>  sCopyUVCurve ;
  for(ii=0;ii<lUVCurveCount;ii++) 
    {
      sCopyUVCurve.Add(new (*GetContext()) SmBSplineCurve(*rUVCurve[ii])) ;
    }
#endif // SM_DEBUG_CODE

  // Find the target span containing or nearest to dNewKnot 
  //    set iSpn1 = knot index beginning 1st span to be modified
  //    init lCase assuming large spacing
  if     (dNewKnot < sInterval.GetMin()) { iSpn1  = lDegree ;
                                           lCase = 1 ;                    
                                         }
  else if(dNewKnot > sInterval.GetMax()) { iSpn1  = pCrv->knt->m - lDegree - 1 ;
                                           lCase = 8 ;
                                         }
  else { // find the knot span, multiplicity, and nearest distance for dNewKnot
         NL_INDEX mlt ; // 0=dNewKnot is not an existing knot, else multiplicity of existing knot
         N_BasisFindSpanAndMult(pCrv->knt, lDegree, dNewKnot, NL_LEFT, &iSpn1, &mlt) ;
         lCase = 4 ;
       }

  // get the target span's size
  SmExtent1d sSpanInterval(pU[iSpn1], pU[iSpn1+1]) ;
  dSpanSize = pU[iSpn1+1] - pU[iSpn1] ;

  // find the closest span boundary to dNewKnot, 
  // boundary outward tangent vector, and displacement vector
  SmPoint3d sNearKnotPoint[2] ;
  double    dMinDist = -1.0 ;
  SmBoolean bTopNode = TRUE ;
  if(fabs(dNewKnot - pU[iSpn1]) < fabs(dNewKnot - pU[iSpn1+1])) 
    { dMinDist = fabs(dNewKnot - pU[iSpn1]) ;
      Evaluate(pU[iSpn1], 1, TRUE, sNearKnotPoint) ;
      sNearKnotPoint[1] = - sNearKnotPoint[1] ;
      bTopNode = FALSE ;
    }
  else
    { dMinDist = fabs(dNewKnot - pU[iSpn1+1]) ;
      Evaluate(pU[iSpn1+1], 1, TRUE, sNearKnotPoint) ;
      bTopNode = TRUE ;
    }
  sNearKnotPoint[1].Unitize() ;
  SmVector3d sDispVec(crThrough3dPoint - sNearKnotPoint[0]) ;
  
  // get span distance and 3d distance from nearest knot to through point
  double dSpanLength = ApproximateLength(sSpanInterval, 5) ;
  double dProjDist3d = sNearKnotPoint[1].Dot(sDispVec) ;

  // gwc note: the following (dProjDist3d < .01) test contains 
  //           a magic munber tolerance. Something better 
  //           should be thought of here.

  // Use existing knots when adding a new knot would create a too small internal segment
  SmBoolean bMoveKnot =  (   (dMinDist/dSpanSize < dMinimunSpanPercent)  //     move when newSpan/OldSpan parameter size ratio is too small
                          && (dProjDist3d > -.01 * dSpanLength)          // and move when newSpan/OldSpan 3d size ratio is too small
                          && (dProjDist3d < .01))                        // and absolute size of span is small
                        ? TRUE 
                        : FALSE ; 
      
  // classify the dNewKnot value into one of the following cases.
  //  lCase has already been initialized to one of 1, 8, 4.
  //  lCase = 1. *  +-----+-----+    = dNewKnot value below curve range - add a segment - update first 2 segments
  //          2.   *+-----+-----+    = dNewKnot value below and close to start knot - update first segment
  //          3.    +*----+-----+    = dNewKnot value in first segment and close to bottom knot - update first segment
  //          4.    +--*--+-----+    = dNewKnot value splits an existing segment - insert knot - update 2 segments
  //          5.    +----*+-----+    = dNewKnot value close to an internal knot - update two segments
  //                +-----+*----+         (special case: same as 5 but iSpn1 must be reduced to point at lower interval)
  //          6.    +-----+----*+    = dNewknot value in last segment and close to top knot - update last segment
  //          7.    +-----+-----+*   = dNewKnot value above and close to end knot - update last segment
  //          8.    +-----+-----+  * = dNewKnot value above curve range - add a segment - update first 2 segments

  // gwc: idea - perhaps we should treat all case 3 and 6 situations as 
  //             a case 4.  That way we don't shorten curves - only lengthen them
  //             or subdivide them. 
  lCase =   (lCase == 1 && bMoveKnot) ? 2
          : (lCase == 4 && bMoveKnot && iSpn1 == lDegree && bTopNode == FALSE) ? 3
          : (lCase == 4 && bMoveKnot && iSpn1 == pCrv->knt->m - lDegree - 1 && bTopNode) ? 6 
          : (lCase == 4 && bMoveKnot) ? 5
          : (lCase == 8 && bMoveKnot) ? 7
          : lCase ;

  // handle special case 5 when moving the bottom knot
  if(lCase == 5 && bTopNode == FALSE)
    {
      // move iSpn1 pointer down one interval
      iSpn1 -= lDegree ;
      SM_ASSERT(iSpn1 >= lDegree) ;
    }

  // prepare a 3 point ThroughPoint array marking the bounds of the
  // the 2 (or just 1) segment to be modified.
  // different cases get different arrays
  SmVector3d sPosTang[8],   *pNewTangent ;
  const int lVecCount = 6 ;
  SmTArray<SmVector2d> sUVPosTang ; sUVPosTang.SetSize(lVecCount*lUVCurveCount) ;
  double dScale1, dScale2 ;
  double dParamArray[3], dAddKnot = 0.0 ;

  switch(lCase)
    {
      case 1 : dAddKnot    =   pU[iSpn1]                 
                             + (pU[iSpn1+1] - pU[iSpn1])
                             * (pU[iSpn1]   - dNewKnot)
                             / (pU[iSpn1+1] - dNewKnot) ;
               dNewKnot    = pU[iSpn1] ;

               dParamArray[0]   = dNewKnot ; // to interpolate crThrough3dPoint
               dParamArray[1]   = dAddKnot ; // to interpolate current endPoint
               dParamArray[2]   = pU[iSpn1+1] ;

               // scale CP spacing in 1st seg by ratio of segment sizes - preserve 2nd segment spacing
               dScale1 = (dParamArray[1] - dParamArray[0]) / (dParamArray[2] - dParamArray[1]) ;
               dScale2 = 1.0 ;
               iSpn2   = iSpn1 ;   // for controlPoint spacing

               // dParamArray[0]   = dNewKnot ;   
               // dParamArray[1]   = pU[iSpn1] ; 
               // dParamArray[2]   = pU[iSpn1+1] ;

               sPosTang[0] = crThrough3dPoint ;
               pNewTangent = &sPosTang[1] ;
               Evaluate( dParamArray[0], 1, TRUE,  sPosTang+2) ; // current end-point
               Evaluate( dParamArray[2], 1, FALSE, sPosTang+4) ;
               break ;

      case 2 :
      case 3 : dNewKnot         = pU[iSpn1] ;
               dParamArray[0]   = dNewKnot ;
               dParamArray[1]   = pU[iSpn1+1] ;
               dParamArray[2]   = dParamArray[1] ;   // unused - but initialized

               // scale CP spacing by change in arc length (since param length is preserved)
               dScale1 = (dSpanLength - dProjDist3d) / dSpanLength ;
               dScale2 = 1.0 ;            // unused
               iSpn2   = iSpn1 ;   // for controlPoint spacing

               sPosTang[0] = crThrough3dPoint ;
               pNewTangent = &sPosTang[1] ;
               Evaluate( dParamArray[1], 1, FALSE, sPosTang+2) ;
               sPosTang[4] = sPosTang[2] ;  // unused - but initialized
               sPosTang[5] = sPosTang[3] ;  // unused - but initialized
               break ;

      case 4 : dParamArray[0]   = pU[iSpn1] ;
               dParamArray[1]   = dNewKnot ;
               dParamArray[2]   = pU[iSpn1+1] ;

               // scale CP spacing by parameter ratios
               dScale1 = (dParamArray[1] - dParamArray[0])/(dParamArray[2]- dParamArray[0]) ;
               dScale2 = (dParamArray[2] - dParamArray[1])/(dParamArray[2]- dParamArray[0]) ;
               iSpn2   = iSpn1 ;   // for controlPoint spacing

               Evaluate( dParamArray[0], 1, TRUE,  sPosTang) ;
               sPosTang[2] = crThrough3dPoint ;
               pNewTangent = &sPosTang[3] ;
               Evaluate( dParamArray[2], 1, FALSE, sPosTang+4) ;
               break ;

      case 5 : dParamArray[0]   = pU[iSpn1] ;
               dParamArray[1]   = dNewKnot ;
               dParamArray[2]   = pU[iSpn1+1+lDegree] ;

               // scale CP spacing by change in param length 
               dScale1 = (dParamArray[1]- dParamArray[0])/(pU[iSpn1+1]-pU[iSpn1]) ;
               dScale2 = (dParamArray[2]- dParamArray[1])/(pU[iSpn1+1+lDegree]-pU[iSpn1+1]) ;
               iSpn2   = iSpn1 + lDegree ;   // for controlPoint spacing

               Evaluate( dParamArray[0], 1, TRUE,  sPosTang) ;
               sPosTang[2] = crThrough3dPoint ;
               pNewTangent = &sPosTang[3] ;
               Evaluate( dParamArray[2], 1, FALSE, sPosTang+4) ;
               break ;

      case 6 :
      case 7 : dNewKnot    = pU[iSpn1+1] ; 
               dParamArray[0]   = pU[iSpn1] ;
               dParamArray[1]   = dNewKnot ;
               dParamArray[2]   = dParamArray[1] ; // unused - but initialized

               // scale CP spacing by change in arc length (since param length is preserved)
               dScale1 = (dSpanLength + dProjDist3d) / dSpanLength ;
               dScale2 = 1.0 ;            // unused
               iSpn2   = iSpn1 ;   // for controlPoint spacing

               Evaluate( dParamArray[0], 1, TRUE, sPosTang) ;
               sPosTang[2] = crThrough3dPoint ;
               pNewTangent = &sPosTang[3] ;
               sPosTang[4] = sPosTang[0] ;  // unused - but initialized
               sPosTang[5] = sPosTang[1] ;  // unused - but initialized
               break ;

      case 8 : dAddKnot    =   pU[iSpn1]                
                             + (pU[iSpn1+1] - pU[iSpn1])
                             * (pU[iSpn1]   - dNewKnot)
                             / (pU[iSpn1+1] - dNewKnot) ;
               dNewKnot    = pU[iSpn1+1] ;

               dParamArray[0]   = pU[iSpn1] ;
               dParamArray[1]   = dAddKnot ; // to interpolate current endPoint
               dParamArray[2]   = dNewKnot ; // to interpolate crThrough3dPoint

               // scale CP spacing in 1st seg by ratio of segment sizes - preserve 2nd segment spacing
               dScale1 = 1.0 ;
               dScale2 = (dParamArray[2] - dParamArray[1]) / (dParamArray[1] - dParamArray[0]) ;
               iSpn2   = iSpn1 ;   // for controlPoint spacing

               // dParamArray[0]   = pU[iSpn1] ;  
               // dParamArray[1]   = pU[iSpn1+1] ;                  
               // dParamArray[2]   = dNewKnot ;

               Evaluate( dParamArray[0], 1, TRUE, sPosTang) ;
               Evaluate( dParamArray[2], 1, TRUE, sPosTang+2) ; // current end-point
               sPosTang[4] = crThrough3dPoint ;
               pNewTangent = &sPosTang[5] ;
               break ;
      default:
               return(SM_ERR) ;

    } // end lCase switch
  
  // set the tangent value for the through point
  if(cpOpt3dTangent) 
    { 
      // let sPosTang[1] = cpOpt3dTangent when given
      *pNewTangent = *cpOpt3dTangent ; 
    }
  else // let sPosTang[1] = current Tangent value at param value - allow OutOfBounds evaluations
    { 
      SM_CURVE_ENABLE_OUTOFBOUNDS(this, 0) 
      Evaluate(dNewKnot, 1, TRUE, sPosTang+6) ;
      *pNewTangent = sPosTang[7] ;
    }

  // normalize the 3 tangent vectors (must have been initialised if unused)
  sPosTang[1].Unitize() ;
  sPosTang[3].Unitize() ;
  sPosTang[5].Unitize() ;

  // get the control point spacing for the segments being modified
  double dOrigCPSpacing1 = sm_GetControlPointSpacing(pW[iSpn1-lDegree], pW[iSpn1-lDegree+1]) ;
  double dOrigCPSpacing2 = sm_GetControlPointSpacing(pW[iSpn2-lDegree], pW[iSpn2-lDegree+1]) ;
 
  // Now get UVCurve pos and tangents for all three points
  SmPoint3d  s3dGuessPoint ;
  SmPoint2d  sUVGuessPoint ;
  SmBoolean  bFoundAnswer ;
  SmSolution sSol ;
  SmVector3d sSurfaceVals[2][2] ;

  // UVCurve, segment2 spacing memory
  SmTArray<double> sUVCPSpacing1, sUVCPSpacing2 ;

  // for all UVCurves - set sUVPosTang array values
  for(ii=0;ii<lUVCurveCount;ii++)
    {
      ULONG lOff = lVecCount * ii ;

      // local pointers
      pUVCrv.Add(rUVCurve[ii]->GetOrCreateGwNurbPointer()) ;
      pUV.Add(pUVCrv[ii]->pol->Pw) ;

      // for all three points
      for(jj=0;jj<3;jj++)
        {
          // get Surface1 UV point - either as given or by projection
          if(dParamArray[jj] == dNewKnot && rOptUVPoint[ii]) { sUVPosTang[lOff+2*jj] = *rOptUVPoint[ii] ; }
          else 
            { 
              // evaluate UVCurve1 to get guess point for upcoming projection
              rUVCurve[ii]->EvaluatePoint( dParamArray[jj], s3dGuessPoint) ;

              // project 3dPoint (sPosTang[2*jj]) to Surface1 near sUVGuessPoint
              sUVGuessPoint.Set(s3dGuessPoint.x, s3dGuessPoint.y) ; 
              rSurface[ii]->LocalPointSolve(rSurface[ii]->GetNaturalUVDomain(),
                                            SM_SO_MINIMIZE, sPosTang[2*jj], sUVGuessPoint,
                                            bFoundAnswer, sSol) ;
          
              // check and store the Surface1 answer
              if(sSol.m_vStart.m_dSolutionValue > 1e-8) return(SM_ERR) ;
              sUVPosTang[lOff+2*jj].Set(sSol.m_vStart.m_adParameters[0],sSol.m_vStart.m_adParameters[1]) ; 
            } // end project to Surface1 to get UVPoint1 branch

          // get Surface values at UVpoints
          rSurface[ii]->Evaluate(sUVPosTang[lOff+2*jj], 1, 1, TRUE, TRUE, TRUE, sSurfaceVals[0]) ;

          // project unit 3D vector onto Surface plane
          SE(smsurf_DropVectors(sSurfaceVals[1][0],                 // in : Surface U_dir tangent               
                                sSurfaceVals[0][1],                 // in : Surface V_dir tangent              
                                1,                                  // in : vector count                       
                                &sPosTang[2*jj+1],                  // in : vector to drop                     
                                &sUVPosTang[lOff+2*jj+1]));         // out: aResultingUVVectors

          // unitize the vector
          sUVPosTang[lOff+2*jj+1].Unitize() ;

          // get the control point spacing for the segments being modified
          double dUVCPSpacing1 = sm_GetControlPointSpacing(pUV[ii][iSpn1-lDegree], pUV[ii][iSpn1-lDegree+1]) ;
          double dUVCPSpacing2 = sm_GetControlPointSpacing(pUV[ii][iSpn2-lDegree], pUV[ii][iSpn2-lDegree+1]) ;
          sUVCPSpacing1.Add(dUVCPSpacing1) ;
          sUVCPSpacing2.Add(dUVCPSpacing2) ;

          // some cases only use two points
          if(   jj == 1
             && (   lCase == 2
                 || lCase == 3
                 || lCase == 6
                 || lCase == 7)) jj++ ;

        } // end iter all three points
    } // end iter all UVCurves

  // modify the knot vector so that dNewKnot is the interpolating parameter
  // For cases 1, 4, 8 
  //  make a new curve with appropriate knot vector and
  //  set pW, pUV1, pUV2 to point to the new curves' control point arrays
  //  remember that the curve was modified
  bModifiedCurve = TRUE ;

  switch(lCase)
    {
      case 2 : // move start Interval end-point params to dNewKnot
      case 3 :
         // set ii to first knot to update 
         // note: end-curve knots have one more knot to update than internal knots
         ii = (iSpn1 == lDegree) ? 0 : iSpn1 - lDegree + 1 ;
         for(;ii<=(ULONG)iSpn1;ii++) { pU[ii]    = dNewKnot ;
                                      for(jj=0;jj<lUVCurveCount;jj++)
                                        { (rUVCurve[jj]->GetOrCreateGwNurbPointer())->knt->U[ii] = dNewKnot ; 
                                        }  
                                    }
         break ;

      case 5 : // move end Interval end-point params to dNewKnot
      case 6 :
      case 7 :
         // set ii to last knot to update
         // note: end-curve knots have one more knot to update than internal knots
         SM_ASSERT(iSpn1 > 0) ;
         ii = (iSpn1 == pCrv->knt->m - lDegree - 1) ? iSpn1 + lDegree + 1 : iSpn1 + lDegree ;
         for(;ii>(ULONG)iSpn1;ii--) { pU[ii]    = dNewKnot ;
                                     for(jj=0;jj<lUVCurveCount;jj++)
                                       { (rUVCurve[jj]->GetOrCreateGwNurbPointer())->knt->U[ii] = dNewKnot ; 
                                       }  
                                   }
         break ;

      case 1 :
      case 4 : // case  4     = insert an interior knot to a segment
      case 8 : // cases 1 & 8 = add a span to the beginning or end of the curve
         // to preserve curve clamping and interval range
         //   - split the span in half and update the
         //     control points to make the new knot interpolate the old end
         //     and the end-knot to interpolate crThrough3dPoint. 

         // split call inits
         N_CrvInitArrays(&sNewCrv);
         for(ii=0;ii<lUVCurveCount;ii++) { NL_CURVE *pCurve = N_AllocCrv( &SG ) ; // these objects get deleted by the SG stack
                                           sNewUVCrv.SetAt(ii, pCurve) ;
                                           N_CrvInitArrays(sNewUVCrv[ii]) ; 
                                         }
         // insert a new knot at 1st span's dParamArray[1] mid-point
         NL_SER(N_CrvInsertKnot(pCrv, dParamArray[1],lDegree,&sNewCrv,   &SG,&SG));
         for(ii=0;ii<lUVCurveCount;ii++)
           {
             NL_SER(N_CrvInsertKnot(pUVCrv[ii], dParamArray[1],lDegree,sNewUVCrv[ii],&SG,&SG));
           }
         // not needed with interval preserving parameterization
         // update parameter values 
         //   move end-point params to dNewKnot and
         //   move mid-point params to start-point value

           //  pNewU    = sNewCrv.knt->U ;
           //  dEndU    = pNewU[0] ;
           // for(ii=0;ii<=(ULONG)lDegree;ii++) { pNewU[ii]    = dNewKnot ;
           //                                     for(jj=0;jj<lUVCurveCount;jj++)
           //                                       { sNewUVCrv[jj].knt->U[ii] = dNewKnot ; 
           //                                       }  
           //                                   }
           // for(ii=0;ii< (ULONG)lDegree;ii++) { pNewU[lDegree+1+ii]    = dEndU ; 
           //                                     for(jj=0;jj<lUVCurveCount;jj++)
           //                                       { sNewUVCrv[jj].knt->U[lDegree+1+ii] = dEndU ; 
           //                                       }
           //                                   }
            
         // ReSet the ControlPoint array pointer to the new curve
         pW = sNewCrv.pol->Pw ;
         for(ii=0;ii<lUVCurveCount;ii++) { pUV[ii] = sNewUVCrv[ii]->pol->Pw ;
                                         }
         break ;
// case 8 and 7 are the same with interval preserving parameterization
//       case 8 : // add a span to the end of the curve
//          // to preserve curve clamping - split the span in half and update the
//          // parameter values as if we had extended the curve
// 
//          // split call inits
//          N_CrvInitArrays(&sNewCrv);
//          for(ii=0;ii<lUVCurveCount;ii++) { N_CrvInitArrays(&sNewUVCrv[ii]) ; 
//                                   }
//          // insert a new knot at span's dParamArray[1] mid-point
//          NL_SER(N_CrvInsertKnot(pCrv,   dParamArray[1],lDegree,&sNewCrv,&SG,&SG));
//          for(ii=0;ii<lUVCurveCount;ii++)
//            {
//              NL_SER(N_CrvInsertKnot(pUVCrv[ii],dParamArray[1],lDegree,&sNewUVCrv[ii],&SG,&SG));
//            }
// 
//          // not needed with interval preserving parameterization
//          // update parameter values 
//          //   move end-point params to dNewKnot and
//          //   move mid-point params to start-point value
//          //    lNewM    = sNewCrv.knt->m ;
//          //    pNewU    = sNewCrv.knt->U ;
//          //    dEndU    = pNewU[lNewM] ;
//          // 
//          //    // update parameter values 
//          //    //   move end-point params to dNewKnot and
//          //    //   move mid-point params to end-point value
//          //    for(ii=0;ii<=(ULONG)lDegree;ii++) { pNewU[lNewM-ii]    = dNewKnot ; 
//          //                                        for(jj=0;jj<lUVCurveCount;jj++)
//          //                                          { sNewUVCrv[jj].knt->U[lNewM-ii] = dNewKnot ; 
//          //                                          }  
//          //                                      }
//          //    for(ii=0;ii< (ULONG)lDegree;ii++) { pNewU[lNewM-lDegree-1-ii]    = dEndU ; 
//          //                                        for(jj=0;jj<lUVCurveCount;jj++)
//          //                                          { sNewUVCrv[jj].knt->U[lNewM-lDegree-1-ii] = dEndU ; 
//          //                                          }
//          //                                      }
// 
//          // ReSet the ControlPoint array pointer to the new curve
//          pW = sNewCrv.pol->Pw ;
//          for(ii=0;ii<lUVCurveCount;ii++) { pUV[ii] = sNewUVCrv[ii].pol->Pw ;
//                                          }
//          break ;  
// 
//       // With dParamArray indirection case 4 is the same as case 1
//       case 4 :
//          // split call inits
//          N_CrvInitArrays(&sNewCrv);
//          for(ii=0;ii<lUVCurveCount;ii++) { N_CrvInitArrays(&sNewUVCrv[ii]) ; 
//                                          }
//          // insert a new knot with multiplicity = lDegree
//          NL_SER(N_CrvInsertKnot(pCrv,   dParamArray[1],lDegree,&sNewCrv,   &SG,&SG));
//          for(ii=0;ii<lUVCurveCount;ii++)
//            {
//              NL_SER(N_CrvInsertKnot(pUVCrv[ii],dParamArray[1],lDegree,&sNewUVCrv[ii],&SG,&SG));
//            }
// 
//          // ReSet the ControlPoint array pointer to the new curve
//          pW = sNewCrv.pol->Pw ;
//          for(ii=0;ii<lUVCurveCount;ii++) { pUV[ii] = sNewUVCrv[ii].pol->Pw ;
//                                          }
//          break ;
    } // end switch on lCase for inserting a new knot when needed

  // notify this SmBSplineCurve's attribs that the Bspline definition is about to change
  Notify(SM_NO_PRE_EDIT, this, SM_NO_GET_OWNER(this), NULL) ;
  for(ii=0;ii<lUVCurveCount;ii++)
   {
     rUVCurve[ii]->Notify(SM_NO_PRE_EDIT, rUVCurve[ii], SM_NO_GET_OWNER(rUVCurve[ii]), NULL) ;
   }

  // get the control point locations - for the first segment
  SmPoint3d sP_1    = sPosTang[0] ;
  SmPoint3d sP_4    = sPosTang[2] ;

  // scale mid ControlPoint spacing 
  SmVector3d sD_1 (sPosTang[1] * dOrigCPSpacing1 * dScale1);
  SmVector3d sD_2 (sPosTang[3] * dOrigCPSpacing1 * dScale1);

  // mid-ControlPoint positions
  SmPoint3d sP_2 = sP_1 + sD_1 ;
  SmPoint3d sP_3 = sP_4 - sD_2 ;

  // set the control point locations in the pW array for the first segment
  N_CPtFromWxWyWz(sP_1.x, sP_1.y, sP_1.z, NL_NOW, &pW[iSpn1-lDegree]) ;
  N_CPtFromWxWyWz(sP_2.x, sP_2.y, sP_2.z, NL_NOW, &pW[iSpn1-lDegree+1]) ;
  N_CPtFromWxWyWz(sP_3.x, sP_3.y, sP_3.z, NL_NOW, &pW[iSpn1-lDegree+2]) ;
  N_CPtFromWxWyWz(sP_4.x, sP_4.y, sP_4.z, NL_NOW, &pW[iSpn1-lDegree+3]) ;

  // Do the same for every UVCurve
  for(ii=0;ii<lUVCurveCount;ii++)
    {
      ULONG lOff = lVecCount * ii ; 

      // get the control point locations - for the first segment
      SmPoint2d sPUV_1 = sUVPosTang[lOff+0] ;
      SmPoint2d sPUV_4 = sUVPosTang[lOff+2] ;
 
      // scale UVCurve mid-CP spacing same as 3d curve
      SmVector3d sDUV_1(sUVPosTang[lOff+1] * sUVCPSpacing1[ii] * dScale1);
      SmVector3d sDUV_2(sUVPosTang[lOff+3] * sUVCPSpacing1[ii] * dScale1);

      // mid-ControlPoint positions
      SmPoint3d sPUV_2 = sPUV_1 + sDUV_1 ;
      SmPoint3d sPUV_3 = sPUV_4 - sDUV_2 ;

      // GWC:REMOVED 4 lines - UVTrimCurves don't currently use z=NL_NOZ values
//      N_CPtFromWxWyWz(sPUV_1.x, sPUV_1.y, NL_NOZ, NL_NOW, &pUV[ii][iSpn1-lDegree]) ;
//      N_CPtFromWxWyWz(sPUV_2.x, sPUV_2.y, NL_NOZ, NL_NOW, &pUV[ii][iSpn1-lDegree+1]) ;
//      N_CPtFromWxWyWz(sPUV_3.x, sPUV_3.y, NL_NOZ, NL_NOW, &pUV[ii][iSpn1-lDegree+2]) ;
//      N_CPtFromWxWyWz(sPUV_4.x, sPUV_4.y, NL_NOZ, NL_NOW, &pUV[ii][iSpn1-lDegree+3]) ;

      double dZVal = pUV[ii][iSpn1-lDegree].z ;
      N_CPtFromWxWyWz(sPUV_1.x, sPUV_1.y, dZVal, NL_NOW, &pUV[ii][iSpn1-lDegree]) ;
      N_CPtFromWxWyWz(sPUV_2.x, sPUV_2.y, dZVal, NL_NOW, &pUV[ii][iSpn1-lDegree+1]) ;
      N_CPtFromWxWyWz(sPUV_3.x, sPUV_3.y, dZVal, NL_NOW, &pUV[ii][iSpn1-lDegree+2]) ;
      N_CPtFromWxWyWz(sPUV_4.x, sPUV_4.y, dZVal, NL_NOW, &pUV[ii][iSpn1-lDegree+3]) ;

    } // end iter every UVCurve

  // for all cases which require updating the control points for a 2nd segment
  if(   lCase == 1
     || lCase == 4
     || lCase == 5
     || lCase == 8)
    {
      // get the control point locations - for the 2nd segment
      sP_1    = sPosTang[2] ;
      sP_4    = sPosTang[4] ;
 
      // scale mid-ControlPoint spacing
      sD_1    = sPosTang[3] * dOrigCPSpacing2 * dScale2 ;
      sD_2    = sPosTang[5] * dOrigCPSpacing2 * dScale2 ;

      // mid-ControlPoint positions
      sP_2    = sP_1 + sD_1 ;
      sP_3    = sP_4 - sD_2 ;

      // set the control point locations in the pW array for the second segment
      // N_CPtFromWxWyWz(sP_1.x, sP_1.y, lDim==3 ? sP_1.z : NL_NOZ, NL_NOW, &pW[spn-lDegree+3]) ;
      
      N_CPtFromWxWyWz(sP_2.x, sP_2.y, sP_2.z, NL_NOW, &pW[iSpn1-lDegree+4]) ;
      N_CPtFromWxWyWz(sP_3.x, sP_3.y, sP_3.z, NL_NOW, &pW[iSpn1-lDegree+5]) ;
      N_CPtFromWxWyWz(sP_4.x, sP_4.y, sP_4.z, NL_NOW, &pW[iSpn1-lDegree+6]) ;

      // Do the same for every UVCurve
      for(ii=0;ii<lUVCurveCount;ii++)
        {
          ULONG lOff = lVecCount * ii ; 

          // get the control point locations - for the 2nd segment
          SmPoint2d sPUV_1 = sUVPosTang[lOff+2] ;
          SmPoint2d sPUV_4 = sUVPosTang[lOff+4] ;
 
          // scale mid CP spacing the same as the 3d curve
          SmVector2d sDUV_1 = sUVPosTang[lOff+3] * sUVCPSpacing2[ii] * dScale2 ;
          SmVector2d sDUV_2 = sUVPosTang[lOff+5] * sUVCPSpacing2[ii] * dScale2 ;

          // get mid-ControlPoint positions
          SmPoint2d sPUV_2 = sPUV_1 + sDUV_1 ;
          SmPoint2d sPUV_3 = sPUV_4 - sDUV_2 ;

          // set the control point locations in the pUV array for the second segment

//          // GWC:REMOVED 4 lines - UVTrimCurves don't currently use z=NL_NOZ values
//          N_CPtFromWxWyWz(sPUV_2.x, sPUV_2.y, NL_NOZ, NL_NOW, &pUV[ii][iSpn1-lDegree+4]) ;
//          N_CPtFromWxWyWz(sPUV_3.x, sPUV_3.y, NL_NOZ, NL_NOW, &pUV[ii][iSpn1-lDegree+5]) ;
//          N_CPtFromWxWyWz(sPUV_4.x, sPUV_4.y, NL_NOZ, NL_NOW, &pUV[ii][iSpn1-lDegree+6]) ;
          
          double dZVal = pUV[ii][iSpn1-lDegree+4].z ;
          N_CPtFromWxWyWz(sPUV_2.x, sPUV_2.y, dZVal, NL_NOW, &pUV[ii][iSpn1-lDegree+4]) ;
          N_CPtFromWxWyWz(sPUV_3.x, sPUV_3.y, dZVal, NL_NOW, &pUV[ii][iSpn1-lDegree+5]) ;
          N_CPtFromWxWyWz(sPUV_4.x, sPUV_4.y, dZVal, NL_NOW, &pUV[ii][iSpn1-lDegree+6]) ;
                                           
        } // end iter every UVCurve
    } // end need to modify 2nd segment check

  // for all cases which inserted knots
  if(   lCase == 1
     || lCase == 4
     || lCase == 8)
    {
      // swap the Bspline data in sNewCrv with m_pNurb to make all the changes stick 

      // build a (temp) SMLib style SmBSplineCurve from the NLib style curNew
      SmBSplineCurve *pTmpBSC    = new (*GetContext()) SmBSplineCurve(GetDim(),(gw_CURVE *)&sNewCrv);
      SmObjDelete sClean(pTmpBSC);

#ifdef SM_DEBUG_CODE
      // draw input: Curve(green), ThroughPoint(red), NearPoint(Magenta), SamplePoints(green), ControlPoints(cyan)
      // draw change: Curve(), ControlPoints()
      if(bDebugMe) 
        { 
          const SmEdge *pEdge = GetEdge() ;
          SmBrep *pBrep = pEdge ? pEdge->GetBrep() : NULL ;

          SmPoint3d sTPoint ;
          SmExtent1d sIvl = GetNaturalInterval();
          double dParam   =  (  fabs(sIvl.GetMin()-dNewKnot) < fabs(sIvl.GetMax()-dNewKnot))
                            ? sIvl.GetMin() 
                            : sIvl.GetMax() ;
          double dInc = (dNewKnot - dParam)/20 ;
          if(fabs(dInc) < 1.0e-8) 
            { if(dInc >= 0.0 ) dInc =  1.0e-8 ;
              else             dInc = -1.0e-8 ;
            }

          smgfx_Erase() ;

          // input
          ULONG i;
          smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
          smgfx_SetLook(1,2, 0,1,1) ; DrawCurvature(-35,175) ; sm_GraphicsLoop() ;
          smgfx_SetLook(3,4, .2,1,1); DrawControlPoints() ; sm_GraphicsLoop() ;
          smgfx_SetLook(3,4, 0,0,1) ; for (i=0; i<=40; i++, dParam +=dInc)
                                        { EvaluatePoint( dParam,sTPoint);
                                          sTPoint.Draw(); sm_GraphicsLoop() ;
                                        }
          smgfx_SetLook(3,4, 1,0,0) ; crThrough3dPoint.Draw() ; sm_GraphicsLoop() ;
          smgfx_SetLook(1,2, 1,0,0) ; if(cpOpt3dTangent) { cpOpt3dTangent->Draw(&crThrough3dPoint) ; } sm_GraphicsLoop() ;
          // change
          smgfx_SetLook(1,2, 0,1,0) ; pTmpBSC->Draw() ; sm_GraphicsLoop() ;
          smgfx_SetLook(3,4, 0,1,.2); pTmpBSC->DrawControlPoints() ; sm_GraphicsLoop() ;
          smgfx_SetLook(1,2, 0,1,0) ; pTmpBSC->DrawCurvature() ; sm_GraphicsLoop() ;
          smgfx_SetLook(3,4, 0,0,0) ; for (i=0; i<=40; i++, dParam +=dInc)
                                        { EvaluatePoint( dParam,sTPoint);
                                          sTPoint.Draw(); sm_GraphicsLoop() ;
                                        }
          sm_GraphicsLoop() ;
        }
#endif // SM_DEBUG_CODE

      // Swap Nurb ptrs in pTmpBSC and the This object
      gw_CURVE *pTmp    = pTmpBSC->m_pNurb ;
      pTmpBSC->m_pNurb  = m_pNurb ;
      m_pNurb           = pTmp ;

      // Do the same for every UVCurve
      for(ii=0;ii<lUVCurveCount;ii++)
        {
          // copy the sNewUVCrvp[ii] object because it's destined to be deleted by the SG stack mechanism
          SmBSplineCurve *pUVTmpBSC = new (*GetContext()) SmBSplineCurve(rUVCurve[ii]->GetDim(),(gw_CURVE *)&sNewUVCrv[ii]);
          SmObjDelete sCleanTmp(pUVTmpBSC);

          // swap the Nurb pointers - the old one is deleted when pUVTmpBSC goes out of scope.
          //                          the new one lives on in rUVCurve[ii]
          gw_CURVE *pUVTmp      = pUVTmpBSC->m_pNurb ;
          pUVTmpBSC->m_pNurb    = rUVCurve[ii]->m_pNurb ;
          rUVCurve[ii]->m_pNurb = pUVTmp ;

        } // end iter every UVCurve
    } // end inserted a knot check

  // notify this SmBSplineCurve's attribs that the Bspline definition has change
  Notify(SM_NO_POST_EDIT, this, SM_NO_GET_OWNER(this), NULL);
  for(ii=0;ii<lUVCurveCount;ii++)
    {
      rUVCurve[ii]->Notify(SM_NO_POST_EDIT, rUVCurve[ii], SM_NO_GET_OWNER(rUVCurve[ii]), NULL) ;
    }

#ifdef SM_DEBUG_CODE
    if (bDebugMe) 
      {

      // check output curves and compare with inputs
      const ULONG lPointCount = 10 ;
      SmPoint3d sPoint ;
      SmTArray<SmPoint3d> sEdit3DPoint(lPointCount) ;
      SmTArray<SmPoint3d> sCopy3DPoint(lPointCount) ;

      SmTArray<SmPoint3d> sEditUVPoint(lPointCount) ;
      SmTArray<SmPoint3d> sCopyUVPoint(lPointCount) ;

      SmTArray<SmPoint3d> sEditWCPoint(lPointCount) ;
      SmTArray<SmPoint3d> sCopyWCPoint(lPointCount) ;

      SmTArray<double> sEditBegIntervalDist(lPointCount) ;
      SmTArray<double> sEditEndIntervalDist(lPointCount) ;
      SmTArray<double> sCopyBegIntervalDist(lPointCount) ;
      SmTArray<double> sCopyEndIntervalDist(lPointCount) ;

      SmTArray<double> sEditCSDist(lPointCount) ;
      SmTArray<double> sCopyCSDist(lPointCount) ;


      double dParamMin = dParamArray[0] ;
      double dParamMax = (   lCase == 1
                          || lCase == 4
                          || lCase == 5
                          || lCase == 8) ? dParamArray[2] : dParamArray[1] ;
      double dP  = dParamMin ;
      double dDP = (dParamMax - dParamMin) / (lPointCount - 1) ;

      // check zero - interpolate the designated through point
      pEdit3DCurve->EvaluatePoint(dNewKnot, sPoint) ;
      double dDeviation = sPoint.DistanceBetween(crThrough3dPoint) ;
      SM_ASSERT(dDeviation < 1.0E-11) ;

      // check one - UVTrimCurves have a surface and share parameterization with this curve
      for ( ii=0; ii < rUVCurve.GetSize(); ii++ )
        { SM_ASSERT(   rUVCurve[ii]->GetDim() == 2
                    && rUVCurve[ii]->GetNumberNaturalKnots() == GetNumberNaturalKnots()
                    && rSurface[ii] != NULL) ;
        }

      // sample the 3DCurves
      for(ii=0,dP=dParamMin;ii<lPointCount;ii++,dP+=dDP)
        {
          pEdit3DCurve->EvaluatePoint(dP, sPoint) ; sEdit3DPoint.Add(sPoint) ;
          pCopy3DCurve->EvaluatePoint(dP, sPoint) ; sCopy3DPoint.Add(sPoint) ;
        }

      // Compute dist from interval beg/end to each sample point
      for(ii=0;ii<lPointCount;ii++)
        {
          // modified curve tests
          sEditBegIntervalDist.Add(sEdit3DPoint[0].DistanceBetween(sEdit3DPoint[ii])) ; 
          sEditEndIntervalDist.Add(sEdit3DPoint[lPointCount-1].DistanceBetween(sEdit3DPoint[ii])) ; 

          // orig copied curve tests
          sCopyBegIntervalDist.Add(sCopy3DPoint[0].DistanceBetween(sCopy3DPoint[ii])) ; 
          sCopyEndIntervalDist.Add(sCopy3DPoint[lPointCount-1].DistanceBetween(sCopy3DPoint[ii])) ; 
        }

      // check two - all points are monotonically closer/farther from end points
      for(ii=0,jj=1;jj<lPointCount;ii++,jj++)
        {
          // modified curve tests
          SM_ASSERT(sEditBegIntervalDist[ii] < sEditBegIntervalDist[jj] ) ;
          SM_ASSERT(sEditEndIntervalDist[ii] > sEditEndIntervalDist[jj] ) ;

          // orig copied curve tests
          SM_ASSERT(sCopyBegIntervalDist[ii] < sCopyBegIntervalDist[jj] ) ;
          SM_ASSERT(sCopyEndIntervalDist[ii] > sCopyEndIntervalDist[jj] ) ;
        }

      // check three - Interval end points are equal
      double dBegDist = sEdit3DPoint[0].DistanceBetween(sCopy3DPoint[0]) ;
      double dEndDist = sEdit3DPoint[lPointCount-1].DistanceBetween(sCopy3DPoint[lPointCount-1]) ;
      SM_ASSERT(dBegDist < 1.0E-9) ;
      SM_ASSERT(dEndDist < 1.0E-9) ;

      // for every UVTrimCurveCurve
      for(ii=0;ii<lUVCurveCount;ii++)
        {
          sEditUVPoint.ReSet() ;
          sCopyUVPoint.ReSet() ;
          sEditWCPoint.ReSet() ;
          sCopyWCPoint.ReSet() ;
          sEditCSDist.ReSet() ;
          sCopyCSDist.ReSet() ;

          // compute surface point locations as W(TC(s))
          for(jj=0,dP=dParamMin;jj<lPointCount;jj++,dP+=dDP)
            {     
              // modified/orig curve UV=TC(s) evals 
              rUVCurve[ii]->EvaluatePoint(dP, sPoint) ;     sEditUVPoint.Add(sPoint) ;
              sCopyUVCurve[ii]->EvaluatePoint(dP, sPoint) ; sCopyUVPoint.Add(sPoint) ;

              // modified/orig surface XYZ=W(UV) evals
              SmPoint2d sCopyUV,  sEditUV ;
              SmPoint3d sCopyXYZ, sEditXYZ ;
              sEditUV.Set(sEditUVPoint[jj].x, sEditUVPoint[jj].y) ;  
              rSurface[ii]->EvaluatePoint(sEditUV,sEditXYZ) ; sEditWCPoint.Add(sEditXYZ) ;

              sCopyUV.Set(sCopyUVPoint[jj].x, sCopyUVPoint[jj].y) ;  
              rSurface[ii]->EvaluatePoint(sCopyUV,sCopyXYZ) ; sCopyWCPoint.Add(sCopyXYZ) ;

              // store the C(s) to W(TC(s)) distance
              double dEditCSDist = sEditWCPoint[jj].DistanceBetween(sEdit3DPoint[jj]) ;
              double dCopyCSDist = sCopyWCPoint[jj].DistanceBetween(sCopy3DPoint[jj]) ;
              sEditCSDist.Add(dEditCSDist) ;
              sCopyCSDist.Add(dCopyCSDist) ;
           } // end iter every sample point

         // check four - distance between C(s) and W(TC(s)) is mostly smaller than before
         ULONG lCnt ;
         for(jj=0,lCnt=0;jj<lPointCount;jj++)
           {
             if(sEditCSDist[jj] < sCopyCSDist[jj] + SM_EFF_ZERO * 10.0) lCnt++ ;  

           } // end iter every sample point
           SM_ASSERT(lCnt >= lPointCount - 3) ;

        } // end iter every TrimCurve

        // graphics
        smgfx_SetColor(1,0,0);
        Draw(); sm_GraphicsLoop();
        Dump();
        for(ii=0;ii<lUVCurveCount;ii++)
          {
            rUVCurve[ii]->Dump();
          }
        sm_GraphicsLoop();
      } // end debugMe check

    // delete the sCopyUVCurves - pCopy3DCurve is already scheduled for delete
    int kk ;
    for(kk=lUVCurveCount-1;kk>=0;kk--)
      {
        SM_ASSERT(sCopyUVCurve.GetLast()  != NULL) ; delete sCopyUVCurve.GetLast() ; 
        sCopyUVCurve.RemoveLast() ;
      }
#endif // SM_DEBUG_CODE

    return SM_SUCCESS;

} // end SmBSplineCurve::SetIntersectionCurveThroughPoint

/*******************************************************************//**
PURPOSE: Join two curves in a G1 fashion.

NOTES: 
   The curves may be the same curve.
     If that's the case, the AtStart flags are ignored.

   Currently this does G1 only.

   If the end points do not match, they will be set to their average values.

   Input argument dPreserveEndFactor: Suggested values from 0.0 to 1.0.
   (Value may be out of this range, it is not checked.  But if it's not in
   this range, the resulting curve shapes will probably not be what you want.)
   If 1.0, the position of the join point will be preserved, and the curve
   around the join point will be pushed out to be G1; all of the control point
   movement will be taken up by the two adjacent points.
   If 0.0, then all of the point move will be at the join point.
   At values in between, the control point shifts will be apportioned
   between he join point and the two adjacent points.
   If the join point can be moved, the result is generally nicer curves.

***********************************************************************/
SmStatus SmBSplineCurve::SmoothJoin(
        SmBSplineCurve * pCurve1,
        SmBoolean        bAtStart1,
        SmBSplineCurve * pCurve2,
        SmBoolean        bAtStart2,
        double           dPreserveEndFactor
    )
{
  if ( pCurve1 == pCurve2 )
  {
      bAtStart1 = FALSE;
      bAtStart2 = TRUE;
  }

  // Grab the relevant control points.
  SmControlPointFormType eForm1 = ( pCurve1->IsRational() ) ? SM_CP_EUCLIDIAN_RATIONAL : SM_CP_NON_RATIONAL;
  SmControlPointFormType eForm2 = ( pCurve2->IsRational() ) ? SM_CP_EUCLIDIAN_RATIONAL : SM_CP_NON_RATIONAL;

  // We'll work with three points, just call them sP1, sP2, and sP3.
  //   sP2 is the common end point, sP1 is the adjacent point in curve 1,
  //   and sP3 is the adjacent point in curve 2.
  // When we're finished, these will be collinear.

  SmPoint3d sP1, sP2, sP3, sTempPt;
  double    dW1, dW2, dW3, dTempWt;

  ULONG lNumPts1 = pCurve1->GetNumberControlPoints();
  ULONG lNumPts2 = pCurve2->GetNumberControlPoints();

  // First set the end points to be the mid point of the given end points.

  ULONG lIdx1 = ( bAtStart1 ) ? 0 : lNumPts1-1;
  ULONG lIdx2 = ( bAtStart2 ) ? 0 : lNumPts2-1;

  SER( pCurve1->GetControlPoint( eForm1, lIdx1, sP2, dW2 ));
  SER( pCurve2->GetControlPoint( eForm2, lIdx2, sTempPt, dTempWt ));

  sP2 = ( sP2 + sTempPt ) / 2.0;
  dW2 = ( dW2 + dTempWt ) / 2.0;

  // Now the neighboring points.  For G1, the three points are collinear.
  // That means that sP2 is on the line connecting sP1 and sP3.
  // To do that, we find the point on that line where we want sP2 to be,
  // and then either move sP2 to that point, or move sP1 and sP3 by the
  // same amount in the other direction, depending on whether the caller
  // wants to allow the common end point to move.

  lIdx1 = ( bAtStart1 ) ? 1 : lNumPts1-2;
  lIdx2 = ( bAtStart2 ) ? 1 : lNumPts2-2;

  SER( pCurve1->GetControlPoint( eForm1, lIdx1, sP1, dW1 ));
  SER( pCurve2->GetControlPoint( eForm2, lIdx2, sP3, dW3 ));

  // First, find the point on the line from sP1 to sP3.
  // A first thought might be to drop sP2 to that line segment, but that
  // could cause problems if, for instance, it dropped outside the
  // segement from sP1 to sP3.  We want to preserve, generally, its
  // position between sP1 and sP3.  So, put it on the interior of the
  // line segment according to the relative distances | sP1-sP2 | and
  // | sP2 - sP3 |.

  double dLen1 = sP2.DistanceBetween( sP1 );
  double dLen2 = sP2.DistanceBetween( sP3 );
  double dFrac = dLen1 / ( dLen1 + dLen2 );

  // Get the point on the line, and the shift vector.
  SmPoint3d sPtOnLine( sP1 + dFrac * ( sP3 - sP1 ) );
  SmVector3d sShift( sP2 - sPtOnLine );

  sShift *= dPreserveEndFactor;

  sP1 += sShift;
  sP3 += sShift;
  sP2 = sPtOnLine + sShift;

  // Stuff these back into the curves.
  if ( bAtStart1 )
  {
      SER( pCurve1->SetControlPoint( eForm1, 0, sP2, dW2 ));
      SER( pCurve1->SetControlPoint( eForm1, 1, sP1, dW1 ));
  }
  else
  {
      SER( pCurve1->SetControlPoint( eForm1, lNumPts1-1, sP2, dW2 ));
      SER( pCurve1->SetControlPoint( eForm1, lNumPts1-2, sP1, dW1 ));
  }

  if ( bAtStart2 )
  {
      SER( pCurve2->SetControlPoint( eForm2, 0, sP2, dW2 ));
      SER( pCurve2->SetControlPoint( eForm2, 1, sP3, dW3 ));
  }
  else
  {
      SER( pCurve2->SetControlPoint( eForm2, lNumPts2-1, sP2, dW2 ));
      SER( pCurve2->SetControlPoint( eForm2, lNumPts2-2, sP3, dW3 ));
  }

  return SM_SUCCESS;

} // end SmBSplineCurve::SmoothJoin

/*******************************************************************//**
PURPOSE: Generate N+1 points and parameters on the curve 
            to produce N equally sized segments (Equal Lengths).  

NOTES:  n=5 generates 6 points for 5 sections (4 internal points)
 with Points evenly spaced chord-lengths along the curve in Image Space.  (expensive)
 
***********************************************************************/
SmStatus SmBSplineCurve::EquallySpacedPoints
  (const double         t0,             // in : begin interval parameter
   const double         t1,             // in : end   interval parameter
   const ULONG          n,              // in : number of intervals
   const double         dTol,           // in : 3d tolerance for evenly spaced
   SmTArray<SmPoint3d> *pOptPoints,     // out: sequence of n+1 approximately evenly spaced points
   SmTArray<double>    *pOptParams)     // out: associated parameters
  const
{
  // init output
  if (pOptPoints) pOptPoints->ReSet();
  if (pOptParams) pOptParams->ReSet();

  // N_InitNurbs/N_EndNurbs initialization and termination when exiting scope
  NL_STACKS  SG;
  SmNLibStackHandler sSH(&SG);

  // NLib style curve NURB data
  NL_CURVE *cur = ((SmBSplineCurve *)this)->GetOrCreateGwNurbPointer();
 
  // locals
  NL_PARAMETER  u0  = t0 ;
  NL_PARAMETER  u1  = t1 ;
  NL_REAL       tol = dTol;
  NL_POINT  *P   = (pOptPoints) ? N_AllocPt1dArray(n,&SG) : NULL; // array of n+1 points
  NL_REAL      *u   = (pOptParams) ? N_AllocReal1dArray(n,&SG) : NULL; // array of n+1 parameters

  // NLib: compute n+1 approximately evenly spaced points on the curve
  NL_SER(N_CrvEvalEvenSpacedPts (cur,u0, u1, n, tol, P, u));


  // save output
  for (ULONG i=0; i<= n; i++) 
    {
      if (pOptPoints) 
        {
          SmPoint3d sPnt;
          COPY_XYZ(P[i], sPnt);
          (*pOptPoints).Add(sPnt);
        }
      if (pOptParams) (*pOptParams).Add(u[i]);
    }

  // all done
  return SM_SUCCESS;

} // end SmBSplineCurve::EquallySpacedPoints


/*******************************************************************//**
PURPOSE: Add a set of knots into a curve.  

NOTES: Note that the knot vector
   should not contain duplicates of existing knots unless you want to
   increase the multiplicity of the knots.  This means that you should not
   put the first and last knot values of the original curve into the array.
***********************************************************************/
SmStatus SmBSplineCurve::InsertKnots
  (SmTArray<double> & rNewKnots)
{
    if (rNewKnots.GetSize() == 0) return SM_SUCCESS;

    // inform the object
    Notify(SM_NO_PRE_EDIT, this, SM_NO_GET_OWNER(this), NULL) ;

    NL_STACKS     SG;
    SmNLibStackHandler sSH(&SG); 
    NL_CURVE      newcur;
    NL_CURVE   *cur = GetOrCreateGwNurbPointer();
    NL_KNOTVECTOR knt;

    knt.m = (NL_INDEX)rNewKnots.GetSize() - 1;
    knt.U = (NL_REAL*)rNewKnots.GetDataArray();
    N_CrvInitArrays(&newcur);

    NL_SER(N_CrvRefine(cur,&knt,&newcur,&SG,&SG));

    SER(SetFromGwNurb(0, (gw_CURVE *)&newcur));

    // inform the object
    Notify(SM_NO_POST_EDIT, this, SM_NO_GET_OWNER(this), NULL) ;

    return SM_SUCCESS;

} // end SmBSplineCurve::InsertKnots

/*******************************************************************//**
PURPOSE: Insert knots into this curve such that the maximum distance
    between the points on the curve is dDistance.

NOTES: 
***********************************************************************/
SmStatus SmBSplineCurve::InsertKnotsByDistance
  (double dDistance)
{
    double dData[256];
    SmTArray<double> sKnots(256,dData);
    double dData2[256];
    SmTArray<double> sNewKnots(256,dData2);
    GetKnots(sKnots);
    for (ULONG j=1; j<sKnots.GetSize(); j++) {
        SmExtent1d sIvl(sKnots[j-1],sKnots[j]);
        if (sIvl.GetLength() < SM_EFF_ZERO_SQRT) continue;
        double dKnotDist = this->ApproximateLength(sIvl,10);
        ULONG lNumSubdivides = (ULONG)(dKnotDist / dDistance);
        ULONG k;
        for (k=0; k<lNumSubdivides; k++) {
            double dT = (k+1.0) / (lNumSubdivides+1.0);
            sNewKnots.Add(sIvl.Evaluate(dT));
        }
    }
    InsertKnots(sNewKnots);
    return SM_SUCCESS;

} // end SmBSplineCurve::InsertKnotsByDistance

/*******************************************************************//**
PURPOSE: This routine computes a NURBS curve interpolating a given
    set of points. 

NOTES: Optionally end tangents, parameters may be passed in.

    Parameterization, if given, is ignored for closed-curve interpolations.
    If the start and end tangents are not specified for a closed curve
    interpolation, the result will be tangent continuous at the seam

    This method is only available for users with NLib
***********************************************************************/
SmStatus SmBSplineCurve::InterpolatePoints
  (const SmContext & crContext,            // in : context for new object construction
   const SmTArray<SmPoint3d> & crPoints,   // in : array of points to interpolate (>1)
   const SmTArray<double> * cpOptParams,   // in : optional corresponding parameter values
   ULONG lDegree,                          // in : output curve degree
   SmVector3d * pOptStartTangent,          // in : Start Tangent, NULL to ignore
   SmVector3d * pOptEndTangent,            // in : End   Tangent, NULL to ignore
   SmBoolean bIsClosedCurve,               // in : TRUE =Output curve is closed
                                           //      FALSE=Output curve is open
   SmInterpolationType eParameterization,  // in : oneof: SM_IT_UNIFORM,
                                           //             SM_IT_CHORDLENGTH,
                                           //             SM_IT_CENTRIPETAL
   SmBSplineCurve *& rpNewCurve)           // out: The interpolating curve
{
  // locals
  NL_STACKS SG;
  SmNLibStackHandler sSH(&SG);
  NL_CURVE   cur;
  N_CrvInitArrays(&cur);

  // prepare for NLIB interpolate curve calls 
  NL_INDEX   m = crPoints.GetSize()-1;
  NL_POINT  *P = N_AllocPt1dArray(m,&SG);

  for (ULONG i=0; i<crPoints.GetSize(); i++) 
    {
      SmPoint3d sPnt = crPoints[i];
      COPY_XYZ(sPnt,P[i]);
    }

  NL_FLAG    par =   (eParameterization == SM_IT_UNIFORM)     ? NL_UNIFORM
                : (eParameterization == SM_IT_CENTRIPETAL) ? NL_CENTRIPETAL
                : NL_CHORDLENGTH ;
  NL_DEGREE  p   = (NL_DEGREE)lDegree;
  NL_VECTOR  Ts, Te;
  NL_VECTOR *pTs = NULL;
  NL_VECTOR *pTe = NULL;

  // Initialize
  N_VectorCreate( 0.0, 0.0, 0.0, &Ts );
  N_VectorCreate( 0.0, 0.0, 0.0, &Te );

  if (pOptStartTangent) { pTs = &Ts;
                          COPY_XYZ_P(*pOptStartTangent,pTs);
                        }
  if (pOptEndTangent)   { pTe = &Te;
                          COPY_XYZ_P(*pOptEndTangent,pTe);
                        }

  if (bIsClosedCurve) 
    {
      // Closed curve, input parametrization (if given) is ignored.
      NL_INDEX   k  = 0; // C^k Continuity at ends
      NL_VECTOR *pD = NULL;
      NL_VECTOR  D[2];
      if (pTs) { 
          D[1] = Ts;
          pD = D;
      }

      if (pTe) { 
          D[1] = Te;
          pD = D;
      }

      // k still has to be 1 regardless if pD is NULL or not so that start and end deriv are calculated
      k = 1;

      // pass the call onto NLIB
      // send pD instead of D in case D[0] and D[1] are not initialized (pTs and pTe were NULL)
      NL_SER(N_FitCrvApproxClosedConditions(P,      // in : points to be approximated                                                   
                                            m,      // in : highest index in Points to be approximated array                            
                                            p,      // in : degree of output cur
                                            pD,     // in : start and stop derivatives, NULL to ignore                                  
                                            k,      // in : out curve continuity, 1 = C1, 2 = C2 ...                                    
                                                    //     when given pD, it must have one deriv for each end for each continuity level.
                                            par,    // in : NL_UNIFORM     = uniform parameterization                                       
                                                    //      NL_CHORDLENGTH = chord length parameterization                                                        
                                                    //      NL_CETRIPETAL  = Centripetal parameterization                               
                                            &cur,   // out: approximating curve                                                         
                                            &SG));  // in : cur's memory stack                                                          
                                                                                  
    } // end Closed branch                                                        
  else if (cpOptParams) 
    {
      // Open and Parameterization is given
      NL_REAL  *u;
      u = N_AllocReal1dArray(m,&SG);
      for (ULONG ii=0; ii<cpOptParams->GetSize(); ii++) 
        {
          u[ii] = (*cpOptParams)[ii];
        }
      NL_KNOTVECTOR *knots = NULL;

      // pass the call onto NLIB
      NL_SER(N_FitCrvKnotsAndTangents(P,m,u,pTs,pTe,NL_TANGENT,&knots,1.0,p,&cur,&SG,&SG));

    } // end Open/Parameters branch

  else if ( pTs != NULL || pTe != NULL ) // Open, no given Parameterization, and end derivs.
    {
      NL_INDEX   kk = 0; // Highest start derivative
      NL_VECTOR  Ds[2];
      if (pTs) { Ds[1] = Ts;
                 kk    = 1;
               }
      NL_INDEX   ll = 0; // Highest end derivative
      NL_VECTOR  De[2];
      if (pTe) { De[1] = Te;
                 ll    = 1;
               }

      // make the interpolate curve call
      NL_SER(N_FitCrvHighDerivs(P,m,p,Ds,kk,De,ll,par,&cur,&SG));

    } // end Open/No Parameters branch

  else // Open and no given Parameterization
    {
      NL_SER( N_FitCrvInterp(P,m,p,par,&cur,&SG));
    }

  // build output SmBSplineCurve from created gw_CURVE
  rpNewCurve = new (crContext) SmBSplineCurve(3,(gw_CURVE *)&cur);

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
  if (bDebugMe) {
      sm_GraphicsLoop();
      smgfx_SetColor(1,0,0);
      rpNewCurve->Draw();
      rpNewCurve->Dump();
  }
#endif // end SM_DEBUG_CODE

  // all done
  return SM_SUCCESS;

} // end SmBSplineCurve::InterpolatePoints

/*******************************************************************//**
PURPOSE: Make curves compatible to one other by raising the degrees
    and inserts knots until both curves have the same degree and are
    defined over the same knot vector. 

NOTES: This method is only available for users with NLib
    
    Note:  The curves input should all be pure NURBS curves.  No analytical
           curves are allowed.  To create a NURBS from an analytical use the
           SmBSplineCurve Constructor:

Example:
    SmLine *pLine = new (crContext) SmLine(...);
    SmBSPlineCurve *pNURB_Only = new (crContext) SmBSplineCurve(*pLine);
    delete pLine;
***********************************************************************/
SmStatus SmBSplineCurve::MakeCurvesCompatible
  (
    const SmTArray<SmBSplineCurve*> & crCurves, // in : array of target curves
    double dKnotTol                             // in : min allowed separation
                                                //      between distinct knots 
  )
{
  // First create an array of NLib curves from the input curves.
  NL_STACKS  SG;
  SmNLibStackHandler sSH(&SG);
  SmTArray <NL_CURVE*> curves;

  ULONG lCount = crCurves.GetSize();
  for (ULONG i=0; i<lCount; i++) 
    {
      SmBSplineCurve * pCurve = crCurves[i];
      if(pCurve->m_pNurb == NULL)
      {
          pCurve->MakeNurb();
      }
      SM_ASSERT(pCurve->m_pNurb != NULL) ;

      // fix up knot vector before call to N_CrvsMakeCompatibleKnotTol [T1000C, bd, 21 Dec 05]
      pCurve->Notify(SM_NO_PRE_EDIT, pCurve, SM_NO_GET_OWNER(pCurve), NULL);
      SER(pCurve->FixupKnotVector( dKnotTol /* SM_EFF_ZERO_SQRT */ ));
      pCurve->Notify(SM_NO_POST_EDIT, pCurve, SM_NO_GET_OWNER(pCurve), NULL);

      curves.Add( sm_CreateNlibCurve(pCurve,SG) );
    }

  // Invoke NLib to make compatible
  NL_SER( N_CrvsMakeCompatibleKnotTol( curves.GetDataArray(), lCount-1, dKnotTol, &SG ));
  
  // Put the modified NLib curves back into our curves,
  // and do the (changed) knot vectors again.
  for (ULONG j=0; j<lCount; j++) 
    {
      SmBSplineCurve * pCurve = crCurves[j];
      pCurve->Notify(SM_NO_PRE_EDIT, pCurve, SM_NO_GET_OWNER(pCurve), NULL);
      SER(pCurve->SetFromGwNurb(0, (gw_CURVE *)curves[j]));
      SER(pCurve->FixupKnotVector( dKnotTol /* SM_EFF_ZERO_SQRT */ ));
      pCurve->Notify(SM_NO_POST_EDIT, pCurve, SM_NO_GET_OWNER(pCurve), NULL);

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
      if (bDebugMe) 
        {
          pCurve->Dump();
          NL_KNOTVECTOR *pknt = curves[j]->knt;
          TCHAR str[SM_TBLOCK_SIZE];
          smos_sprintf(str, _T("Curve #%4ld\n"), j);
          smos_WriteBuffer(str);
          for (int k=0; k<pknt->m; k++) 
            {
              smos_sprintf(str, _T("%24.24lf\n"), pknt->U[k]);smos_WriteBuffer(str);
            }
        }
#endif // end SM_DEBUG_CODE
    }

  return SM_SUCCESS;

} // end SmBSplineCurve::MakeCurvesCompatible

/*******************************************************************//**
PURPOSE: Deform line smoothly to force its endPoints to interpolate given
   EndPt and EndTan tgts while preserving the general shape of this curve.

NOTES:
    1. Knots may be added to the KnotVector to make the curve flexible 
       enough near the ends for the desired deformation.

METHOD: Build anad add a displacement curve to the CurrentCurve to deform the CurrCrv shape as requested.
        The DispCrv is built in 3 sections: 
          1. a RigidBody region at the front on the Ivl:[StartParam,BreakParam]
          2. a RigidBody region at the end   on the Ivl:[EndParam-BreakParam, EndParam
          3. a smoothly varying region between the two EndRegions, Ivl:[BreakParam, EndParam-BreakParam]

        The displacement function is created from the CurrentCurve's Knot and CPt vectors as
          1. Create knot vector with the union of the
             a. CurrentCurve knots <= BreakParam and knots at [BreakParam/2, BreakParam] at the front end and
             b. CurrentCurve knots >= EndParam-BreakParam and knots at [EndParam-BreakPoint, EndParam-BreakPoint/2]
          2. Make the current and displacement curves homogeous to one another over the end regions by
             adding knots to both curves as needed.
          3. Place the control points of the displacement function with the displacements needed to move the
             end parts of the current curve from where they are to where they need to be as a rigid body.
          4. refine the displacement curve to be homogenous to the current curve by inserting and current curve
             knots between the two end regions into the displacement curve.
          5. Let CurrentCrv = CurrentCrv+DispCrv simply by adding the CPt positions of the two curves together
***********************************************************************/
SmStatus SmBSplineCurve::DeformToEndPointTargets
 (SmPoint3d     * pTgtStartPt,            // in :  NotNULL = Curve StartPt new Tgt position 
                                          //       NULL    = leave as is
  SmPoint3d     * pTgtEndPt,              // in :  NotNULL = Curve EndPt new Tgt position
                                          //       NULL    = leave as is
  SmCurve      ** ppOptNewCurve,          // out: Not Used - BSplineCurve are edited in-place.
  SmVector3d    * pOptTgtStartTan,        // in : optional Start 1stDir tangent dir (NonUnit but only uses dir, not speed) 
  SmInValueType   eTgtStartTanType,       // in : oneof SM_IV_SPECIFIED    : set Start1stDir = pOptTgtStartTan dir
                                          //            SM_IV_SAME         : set Start1stDir = Init Start1stDir
                                          //            SM_IV_UNCONSTRAINED: set Start1stDir = Unspecified
                                          //      default:[SM_IV_SAME]
  SmVector3d    * pOptTgtEndTan,          // in : optional End 1stDir tangent dir (NonUnit but only uses dir, not speed)
  SmInValueType   eTgtEndTanType,         // in : oneof SM_IV_SPECIFIED    : set End1stDir = pOptTgtEndTan dir
                                          //            SM_IV_SAME         : set End1stDir = Init End1stDir
                                          //            SM_IV_UNCONSTRAINED: set End1stDir = Unspecified
                                          //      default:[SM_IV_SAME]
  SmSurface     * pOptSurfCrvOnSurf,      // in : Optional surface for upon which the target points are define. CrvOnSurf editing. If the CrvOnSurf surf agrees with pOptSurfCrvOnSurf
                                          //      then we edit the underlying UV curve instead of approximating. Default is NULL.
  SmOrientType  * pOptCrvOrientation)     // in : orientation for when we have a crv on surf and will adjust UV points.
{
  SM_REF2(pOptSurfCrvOnSurf, pOptCrvOrientation);

    // no work - no requested  changes
  if(   (pTgtStartPt      == NULL)
     && (pTgtEndPt        == NULL)
     && (eTgtStartTanType == SM_IV_SAME || eTgtStartTanType == SM_IV_UNCONSTRAINED)
     && (eTgtEndTanType   == SM_IV_SAME || eTgtEndTanType   == SM_IV_UNCONSTRAINED))
    { return SM_SUCCESS ; }

  // pass derived types back to SmCurve::DeformToEndPointTargets so that they can be copied as BSplineCurves and sent back to this method to be edited-in-place
  if(IsKindOf(SmBSplineCurve_TYPE) && (GetType() != SmBSplineCurve_TYPE))
    {
      // pass the call up to SmCurve to make a BSplineCurve copy to be sent back here for edit-in-place
      return ((SmCurve*)this)->SmCurve::DeformToEndPointTargets(pTgtStartPt,     
                                                                pTgtEndPt,       
                                                                ppOptNewCurve,   
                                                                pOptTgtStartTan,    
                                                                eTgtStartTanType,
                                                                pOptTgtEndTan,      
                                                                eTgtEndTanType) ;
    } // end Derived from SmBSplineCheck                                                

  // Algorithm fixed parameters - change these to tune behavior of the algorithm
  const NL_REAL  nBreakParamPercent = 0.15 ;   // Define ParamBndry between the diff DispCrv regions, a number in Ivl:[0 .5]
  const NL_INDEX nDispInsideKntCnt  = 4 ;      // max number of knots added to CurrCrv to make it flexible enough for the deformation
  const NL_REAL  nNearParamPercent  = 1.0e-3 ;  // Max Dist between near pairs of knot param values, (val is scaled for large Ivls)

  // Check state - must be an SmBSplineCurve - not a derived type
  SM_ASSERT_MSG(GetType() == SmBSplineCurve_TYPE, _T("SmBSplineCurve::DeformToEndPointTargets being passed a nonSmBSplineCurve type - needs debugging")) ;

  // check input
  AERS_MSG(eTgtStartTanType != SM_IV_SPECIFIED || pOptTgtStartTan != NULL, _T("SmBSplineCurve::DeformToEndPointTargets Bad input, when eTgtStartTanType == SM_IV_SPECIFIED, pOptTgtStartTan must be NonNULL ")) ;
  AERS_MSG(eTgtEndTanType   != SM_IV_SPECIFIED || pOptTgtEndTan   != NULL, _T("SmBSplineCurve::DeformToEndPointTargets Bad input, when eTgtEndTanType == SM_IV_SPECIFIED, pOptTgtEndTan must be NonNULL ")) ;

  // init output - BSplines are edited in-place
  if(ppOptNewCurve) { *ppOptNewCurve = NULL ; }

  // locals
  NL_INDEX ii, i0, i1 ;

  // init NURBs curve state (terminates stack when sSH goes out of scope)
  NL_STACKS SG;
  SmNLibStackHandler sSH(&SG);

  // increment CurrCrv degree when needed
  // Minimum of degree 3 needed because of degrees of freedom for smooth curve edit
  // Even degree curves with no interior knots don't work with implemented indexing scheme. Minimal harm by bumping degree.
  NL_DEGREE nCurrCrv_Degree = GetGwNurbPointer()->p;
  if(nCurrCrv_Degree < 3)
    { DegreeElevate(3) ; }
  else if ( nCurrCrv_Degree % 2 == 0)
    { DegreeElevate(nCurrCrv_Degree+1); }

  // CurrCrv locals - pCurrCrv has memory in the SMLib class hierarchy
  NL_CURVE  * pCurrCrv  = GetGwNurbPointer() ;

  // Build new curve - NLib stack memory - all NLib stack memory is temp for this method only
  NL_CURVE curNew;
  N_CrvInitArrays(&curNew);
  N_CrvCopy(pCurrCrv, &curNew, &SG) ; // copy SMLib GW_CURVE memory block into a NLib NL_CURVE memory struct pointed to by NLib memory stack SG
  pCurrCrv = &curNew ;

  // pCurrCrv locals
  NL_INDEX    nCurrCrv_CPtIndex ;
  NL_CPOINT * pCurrCrv_CPts ;
  NL_INDEX    nCurrCrv_KnotIndex ;  // (m = n + p + 1)
  NL_REAL   * pCurrCrv_Knts ;
  NL_INDEX    nCurrCrv_InReg1_KntCnt = 0 ;
  NL_INDEX    nCurrCrv_InReg2_KntCnt = 0 ;
  NL_INDEX    nCurrCrv_InReg3_KntCnt = 0 ;
  N_CrvGetCPtsDegreeAndKnots( pCurrCrv,           // in : target curve                                                    
                             &nCurrCrv_CPtIndex,  // in : Highest index in control point array                            
                             &pCurrCrv_CPts,      // out: array of control points                                         
                             &nCurrCrv_Degree,    // out: curve degree                                                    
                             &nCurrCrv_KnotIndex, // out: Highest index in KnotArray                                      
                             &pCurrCrv_Knts) ;    // out: array of knots (multiple knots are represented multiple times ) 
  double      dCurrCrv_KntMin = pCurrCrv_Knts[nCurrCrv_Degree] ;
  double      dCurrCrv_KntMax = pCurrCrv_Knts[nCurrCrv_KnotIndex-nCurrCrv_Degree] ;
  double      dCurrCrv_KntIvl = dCurrCrv_KntMax - dCurrCrv_KntMin ;

  // DispCrv inside Knot locals
  double      dBreak1           = dCurrCrv_KntMin + nBreakParamPercent * dCurrCrv_KntIvl ;
  double      dBreak2           = dCurrCrv_KntMax - nBreakParamPercent * dCurrCrv_KntIvl ;
  SM_ASSERT_MSG(nDispInsideKntCnt == 4, _T("SmBSplineCurve::DeformToEndPointTargets: Bad Inside Knot count problem - The DispCrv was written with 4 interiror knots - and if that changes then these next initializations have to change.")) ;
  NL_REAL     nNearParamDist = smos_Min(nNearParamPercent * (1.0 + dCurrCrv_KntIvl), dCurrCrv_KntIvl * 0.35) ; // Limit such that dBreak1 + nNearParamDist <= Interval Midpoint
  NL_INDEX    nNewKntIndex[nDispInsideKntCnt] = { -1, -1, -1, -1 } ;
  NL_REAL     nNewKnts    [nDispInsideKntCnt] = { dCurrCrv_KntMin + nBreakParamPercent/2 * dCurrCrv_KntIvl,
                                                  dCurrCrv_KntMin + nBreakParamPercent   * dCurrCrv_KntIvl,
                                                  dCurrCrv_KntMax - nBreakParamPercent   * dCurrCrv_KntIvl,
                                                  dCurrCrv_KntMax - nBreakParamPercent/2 * dCurrCrv_KntIvl } ;
                                                    
  // for every interior CurrCrv Knot - count knots in each of 3 DispFunction regions - find knots within Tol of DispInsideKnts
  for(ii=nCurrCrv_Degree+1;ii<nCurrCrv_KnotIndex-nCurrCrv_Degree;ii++)
    {
      if     (pCurrCrv_Knts[ii] <= dBreak1 + nNearParamDist) { nCurrCrv_InReg1_KntCnt++ ; }
      else if(pCurrCrv_Knts[ii] <  dBreak2 - nNearParamDist) { nCurrCrv_InReg2_KntCnt++ ; }
      else                                                   { nCurrCrv_InReg3_KntCnt++ ; }

      // snap NewKnts to existing knots when near enough - remember the snap
      if     ((nNewKntIndex[0] == -1) && (SM_ARE_SAME_TO_TOL(pCurrCrv_Knts[ii], nNewKnts[0], nNearParamDist))) { nNewKnts[0] = pCurrCrv_Knts[ii] ; nNewKntIndex[0] = ii ; }
      else if((nNewKntIndex[1] == -1) && (SM_ARE_SAME_TO_TOL(pCurrCrv_Knts[ii], nNewKnts[1], nNearParamDist))) { nNewKnts[1] = pCurrCrv_Knts[ii] ; nNewKntIndex[1] = ii ; }
      else if((nNewKntIndex[2] == -1) && (SM_ARE_SAME_TO_TOL(pCurrCrv_Knts[ii], nNewKnts[2], nNearParamDist))) { nNewKnts[2] = pCurrCrv_Knts[ii] ; nNewKntIndex[2] = ii ; }
      else if((nNewKntIndex[3] == -1) && (SM_ARE_SAME_TO_TOL(pCurrCrv_Knts[ii], nNewKnts[3], nNearParamDist))) { nNewKnts[3] = pCurrCrv_Knts[ii] ; nNewKntIndex[3] = ii ; }
    } // end iter every CurrCrv Knot
                          
#ifdef SM_DEBUG_CODE
SmBSplineCurve sDbgThisCpy(*this) ;  // input CrvCopy for later compares to inplace this curve changes

SmBoolean bDebugMe = FALSE ;
  if (bDebugMe)  // draw/dump CurrCrv before 1st knot insertion
    {
      const SmEdge * pEdge = GetEdge() ;
      const SmBrep * pBrep = GetBrep() ;

      // pretty print CurrCrv
      smos_WriteBuffer(_T("\nBegin CurrCrv before changes Dump\n")) ; Dump_NCrv(pCurrCrv, FALSE) ;
      smos_WriteBuffer(_T("\nEnd CurrCrv before changes Dump\n")) ;

      // draw brep, edge and this SmCurve
      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0,1,1)   ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(2,3, 0,1,0)   ; if(pEdge) pEdge->Draw() ;     sm_GraphicsLoop() ;
      smgfx_SetLook(1,5, 0,0,1)   ; this->DrawWithKnots() ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 1,.5,.5) ; this->DrawCurvature() ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 1,0,1)   ; this->DrawPolygon() ; sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;
    }
#endif // end SM_DEBUG_CODE

  // add knots to CurrCrv as needed   (hard coded for adding 2 knots in Reg1 and Reg2)
  SM_ASSERT_MSG(nDispInsideKntCnt == 4, _T("SmBSplineCurve::DeformToEndPointTargets: Bad Inside Knot count problem - The DispCrv was written with 4 interiror knots - and if that changes then these next initializations have to change.")) ;
  for(ii=0;ii<nDispInsideKntCnt;ii++)
    {
      // when the DispCrv Inside knt has not been snapped to an existing CurrCrv knot and CurrCrv needs more knots to be flexible
      if(   (nNewKntIndex[ii] == -1)                  // if  NewKnt is not already present
         && (   (   (ii < nDispInsideKntCnt/2)        // and (    (  checking Reg1 Knts 
                 && (nCurrCrv_InReg1_KntCnt < 2))     //             and Reg1 has less than 2 interior knots)
             || (   (ii >= nDispInsideKntCnt/2)       //      or  (  checking Reg3 Knts 
                 && (nCurrCrv_InReg3_KntCnt < 2))))   //             and Reg1 has less than 2 interior knots))
        {
          // pick the NewKnot furthest away from the one existing knot in Reg1
          if(ii==0 && nCurrCrv_InReg1_KntCnt == 1)
            { if(  smos_Fabs(nNewKnts[1] - pCurrCrv_Knts[nCurrCrv_Degree+1])   // if Dist(1stInteriorKnt-Break1Knt)
                 > smos_Fabs(nNewKnts[0] - pCurrCrv_Knts[nCurrCrv_Degree+1]))  //  > Dist(1stInteriorKnt-Break1Knt/2.0)
                { ii++ ; }                                                     // insert Break1Knt= nNewKnts[ii == 1] ;
            } // end case: One existing Knot in Reg1

          // pick the NewKnot furthest away from the one existing knot in Reg3
          else if(ii==2 && nCurrCrv_InReg3_KntCnt == 1)
            { if(  smos_Fabs(nNewKnts[2] - pCurrCrv_Knts[nCurrCrv_KnotIndex-nCurrCrv_Degree-1])   // if Dist(LastInteriorKnt-Break2Knt)
                 < smos_Fabs(nNewKnts[3] - pCurrCrv_Knts[nCurrCrv_KnotIndex-nCurrCrv_Degree-1]))  //  < Dist(LastInteriorKnt-Break2Knt/2.0)
                { ii++ ; }                                                                        // insert Break2Knt/2 = nNewKnts[ii == 1] ;
            } // end case: One existing Knot in Reg3

          // insert a DispCrv inside knot
          N_CrvInsertKnot(pCurrCrv,     // in : input Crv
                          nNewKnts[ii], // in : knot param value to insert
                          1,            // in : number of times to insert knot at param = u
                          pCurrCrv,     // out: output Crv, when InputCrv == OutputCrv, knot insertion is done in place
                          &SG,          // in : curP's stack
                          &SG) ;        // in : curQ's stack

          // keep count     
          if(nNewKnts[ii] <= dBreak1 + nNearParamDist/2) { nCurrCrv_InReg1_KntCnt++ ; } // TODO: Different Reg1 definition than above. Resolve
          else                                           { nCurrCrv_InReg3_KntCnt++ ; }
        } // end need to insert DispCrv Inside knt into CurrCrv check 
    } // end iter all DispCrv inside knots

  SM_ASSERT_MSG(nCurrCrv_InReg1_KntCnt >= 2, _T("SmBSplineCurve::DeformToEndPointTargets - Failed to ensure 2 interior knots within Reg1 - needs debug")) ;
  SM_ASSERT_MSG(nCurrCrv_InReg3_KntCnt >= 2, _T("SmBSplineCurve::DeformToEndPointTargets - Failed to ensure 2 interior knots within Reg3 - needs debug")) ;

  // refresh CurrCrv locals
  N_CrvGetCPtsDegreeAndKnots( pCurrCrv,           // in : target curve                                                   
                             &nCurrCrv_CPtIndex,  // in : Highest index in control point array                           
                             &pCurrCrv_CPts,      // out: array of control points                                        
                             &nCurrCrv_Degree,    // out: curve degree                                                   
                             &nCurrCrv_KnotIndex, // out: Highest index in KnotArray                                     
                             &pCurrCrv_Knts) ;    // out: array of knots (multiple knots are represented multiple times )

#ifdef SM_DEBUG_CODE
  if (bDebugMe)  // draw/dump CurrCrv after 1st knot insertion
    {
      const SmEdge * pEdge = GetEdge() ;
      const SmBrep * pBrep = GetBrep() ;

      // pretty print CurrCrv
      smos_WriteBuffer(_T("\nBegin CurrCrv after Reg1/Reg3 knot insertions Dump\n")) ; Dump_NCrv(pCurrCrv, FALSE) ;
      smos_WriteBuffer(_T("\nEnd CurrCrv after Reg1/Reg3 knot insertions Dump\n")) ;

      // draw brep, edge and this SmCurve
      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0,1,1)   ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(2,3, 0,1,0)   ; if(pEdge) pEdge->Draw() ;     sm_GraphicsLoop() ;
      smgfx_SetLook(1,5, 0,0,1)   ; this->DrawWithKnots() ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 1,.5,.5) ; this->DrawCurvature() ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 1,0,1)   ; this->DrawPolygon() ; sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;
    }
#endif // end SM_DEBUG_CODE

  // build DispCrv - with all the same Reg1 and Reg3 nCurrCrv knots
  NL_CURVE    nDispCrv ;
  NL_DEGREE   nDispCrv_Degree    = nCurrCrv_Degree ;
  NL_INDEX    nDispCrv_KnotIndex = nCurrCrv_InReg1_KntCnt + nCurrCrv_InReg3_KntCnt + 2*nCurrCrv_Degree + 1 ;  
  NL_REAL   * pDispCrv_Knts      = NULL ;
  NL_INDEX    nDispCrv_CPtIndex  = nDispCrv_KnotIndex - nDispCrv_Degree - 1 ;  // (m = n + p + 1)
  NL_CPOINT * pDispCrv_CPts      = NULL ;

  // init the new displacement curve
  N_CrvInitArrays(&nDispCrv);

  // alloc nDispCrv memory
  //    note: ptrs to the memory allocated for DispCrv get stashed in SG.
  //          That mem is freed in this method when N_EndNurbs(SG) is called which happens when SmNLibStackHandler sSH goes out of scope.
  //          none of that is necessary in C++ - something that needs to be restructured
  N_AllocCrvArrays(&nDispCrv,           // i/o: set this NURBs crv size members and internal arrays
                    nDispCrv_CPtIndex,  // in : Highest index in control point array
                    nDispCrv_Degree,    // in : Degree of the curve
                    nDispCrv_KnotIndex, // in : Highest index in knot vector array
                   &SG );               // in : cur's stack

  // nDispCrv locals
  N_CrvGetCPtsDegreeAndKnots(&nDispCrv,           // in : target curve                                                   
                             &nDispCrv_CPtIndex,  // in : Highest index in control point array                           
                             &pDispCrv_CPts,      // out: array of control points                                        
                             &nDispCrv_Degree,    // out: curve degree                                                   
                             &nDispCrv_KnotIndex, // out: Highest index in KnotArray                                     
                             &pDispCrv_Knts) ;    // out: array of knots (multiple knots are represented multiple times )

  // copy Reg1 knot values into nDispCrv_Knts array
  for(ii=0; ii<=nCurrCrv_InReg1_KntCnt+nCurrCrv_Degree; ii++)
    {
      pDispCrv_Knts[ii] = pCurrCrv_Knts[ii] ;   // Reg1 Knt copy
    }

  // copy Reg3 knot values into nDispCrv_Knts array
  for(i0=nCurrCrv_InReg1_KntCnt+nCurrCrv_Degree+1,   // i0 = DispCrv Index
      i1=i0+nCurrCrv_InReg2_KntCnt;                  // i1 = CurrCrv Index
      i0<=nDispCrv_KnotIndex;
      i0++,i1++)
    {
      pDispCrv_Knts[i0] = pCurrCrv_Knts[i1] ;   // Reg3 Knt copy
    }
                           
#ifdef SM_DEBUG_CODE
  if (bDebugMe)  // dump CurrCrv and DispCrv after DispCrv Reg1 and Reg3 Knots set before CPts set
    {
      // pretty print Crvs
      smos_WriteBuffer(_T("\nBegin CurrCrv After knot insertions Dump\n")) ;                    Dump_NCrv( pCurrCrv, FALSE) ;
      smos_WriteBuffer(_T("\nEnd CurrCrv After knot insertions Dump\nBegin DispCrv after Reg1/Reg3 knots Dump\n")) ;  Dump_NCrv(&nDispCrv, FALSE) ;
      smos_WriteBuffer(_T("\nEnd DispCrv after Reg1/Reg3 knots Dump\n")) ;   
      sm_GraphicsLoop() ;
    }
#endif // end SM_DEBUG_CODE

  // next: set Control point positions of displacement curve
  //       1. Build StartTransform = disp + rotation to set CurveStart position and tangent dir as requested
  //       2. Build EndTransform   = disp + rotation to set CurveEnd position and tangent der as requested
  //       3. Set all DispCrv Reg1 CPts from displacments found by applying StartTransform to CurrCrv Reg1 CPts
  //       4. Set all DispCrv Reg3 CPts from displacments found by applying EndTransform to CurrCrv Reg3 CPts

  // 1. Build StartTransform = disp + rotation to set CurveStart position and tangent dir as requested
  //   note: for BSplines let SM_IV_SAME and SM_IV_UNCONSTRAINED be the same - the current tangent will be presereved (ex: for lines they will be different)
  NL_POINT   nDispStartPt ;
  N_CPtToPtEuclid(pCurrCrv_CPts[0], &nDispStartPt) ;

  SmVector3d sStartFromOrigin((double*)&nDispStartPt) ;
  SmVector3d sStartToOrigin  (pTgtStartPt ? *pTgtStartPt : sStartFromOrigin) ;
  SmVector3d sStartFromX(1,0,0) , sStartToX(1,0,0) ;
  SmVector3d sStartFromY(0,1,0) , sStartToY(0,1,0) ;
  SmVector3d sStartFromZ(0,0,1) , sStartToZ(0,0,1) ;

  // when asked to manage start tangent values
  if(eTgtStartTanType == SM_IV_SPECIFIED)
    {
      // start Tangent
      NL_POINT nCurrCrv_Pt0 ; N_CPtToPt(pCurrCrv_CPts[0], &nCurrCrv_Pt0) ; 
      NL_POINT nCurrCrv_Pt1 ; N_CPtToPt(pCurrCrv_CPts[1], &nCurrCrv_Pt1) ;
      SmVector3d sStartTan(nCurrCrv_Pt1.x - nCurrCrv_Pt0.x,
                           nCurrCrv_Pt1.y - nCurrCrv_Pt0.y,
                           nCurrCrv_Pt1.z - nCurrCrv_Pt0.z) ;
      // start rotations
      sStartTan.    MakeUnitOrthoVectors(pOptTgtStartTan, sStartFromX, sStartFromY, sStartFromZ) ;
      pOptTgtStartTan->MakeUnitOrthoVectors(&sStartTan,   sStartToX,   sStartToY,   sStartToZ) ;  // remember to negate sStartToY
      sStartToY = -sStartToY ;
    } // end managed start tangent check

  // start Displacement transform
  SmAxis2Placement sDispStartTransform(sStartFromOrigin,
                                       sStartFromX,
                                       sStartFromY,
                                       sStartToOrigin,
                                       sStartToX,
                                       sStartToY) ;

  // 2. Build EndTransform   = disp + rotation to set CurveEnd position and tangent der as requested
  //   note: for BSplines let SM_IV_SAME and SM_IV_UNCONSTRAINED be the same - the current tangent will be presereved (ex: for lines they will be different)
  NL_POINT   nDispEndPt ;
  N_CPtToPtEuclid(pCurrCrv_CPts[nCurrCrv_CPtIndex], &nDispEndPt) ;

  SmVector3d sEndFromOrigin((double*)&nDispEndPt) ;
  SmVector3d sEndToOrigin  (pTgtEndPt ? *pTgtEndPt : sEndFromOrigin) ;
  SmVector3d sEndFromX(1,0,0), sEndToX(1,0,0) ;
  SmVector3d sEndFromY(0,1,0), sEndToY(0,1,0) ;
  SmVector3d sEndFromZ(0,0,1), sEndToZ(0,0,1) ; 

  // when asked to manage end tangent values
  if(eTgtEndTanType == SM_IV_SPECIFIED)
    {
      // end Tangent
      NL_POINT nCurrCrv_PtN ;   N_CPtToPt(pCurrCrv_CPts[nCurrCrv_CPtIndex], &nCurrCrv_PtN) ; 
      NL_POINT nCurrCrv_PtN_1 ; N_CPtToPt(pCurrCrv_CPts[nCurrCrv_CPtIndex-1], &nCurrCrv_PtN_1) ;
      SmVector3d sEndTan(nCurrCrv_PtN.x - nCurrCrv_PtN_1.x,
                         nCurrCrv_PtN.y - nCurrCrv_PtN_1.y,
                         nCurrCrv_PtN.z - nCurrCrv_PtN_1.z) ;
      // end rotations
      sEndTan.    MakeUnitOrthoVectors(pOptTgtEndTan, sEndFromX, sEndFromY, sEndFromZ) ;
      pOptTgtEndTan->MakeUnitOrthoVectors(&sEndTan,   sEndToX,   sEndToY,   sEndToZ) ;  // remember to negate sEndToY
      sEndToY = -sEndToY ;
    } // end managed end tangent check

  // end Displacement transform
  SmAxis2Placement sDispEndTransform(sEndFromOrigin,
                                     sEndFromX,
                                     sEndFromY,
                                     sEndToOrigin,
                                     sEndToX,
                                     sEndToY) ;

#ifdef SM_DEBUG_CODE
  if (bDebugMe)  // dump CurrCrv and DispCrv after DispCrv Reg1 and Reg3 Knots set before CPts set
    {
      // pretty print Crvs
      smos_WriteBuffer(_T("\nBegin sDispStartTransform Dump\n")) ;                             sDispStartTransform.Dump() ;
      smos_WriteBuffer(_T("\nEnd sDispStartTransform Dump\nBegin sDispEndTransform Dump\n")) ; sDispEndTransform.Dump() ;
      smos_WriteBuffer(_T("\nEnd sDispEndTransform Dump\n")) ;   
      sm_GraphicsLoop() ;
    }
#endif // end SM_DEBUG_CODE

  // 3. Set all DispCrv Reg1 CPts from displacments found by applying StartTransform to CurrCrv Reg1 CPts
  // 4. Set all DispCrv Reg3 CPts from displacments found by applying EndTransform to CurrCrv Reg3 CPts

  // locals
  SmPoint3d sInPoint, sOutPoint ;
  NL_POINT  nCurrCrv_Pt ;
  NL_INDEX  nReg1_CPtCnt = nCurrCrv_InReg1_KntCnt + ( nDispCrv_Degree + 1 ) / 2; // [(nDispCrv_Degree+1)/2] == number of control points with support at curve min for odd degree curves
  NL_INDEX  nReg2_CPtCnt = nCurrCrv_CPtIndex - nDispCrv_CPtIndex ;
  NL_INDEX  nReg3_CPtCnt = nCurrCrv_InReg3_KntCnt + ( nDispCrv_Degree + 1 ) / 2; // [(nDispCrv_Degree+1)/2] == number of control points with support at curve max for odd degree curves
  SM_ASSERT_MSG(nCurrCrv_CPtIndex == nReg1_CPtCnt + nReg2_CPtCnt + nReg3_CPtCnt - 1, _T("SmBSplineCurve::DeformToEndPointTargets: Error in index math for CurrCrv_CPt array sizes - needs debug")) ; 
  SM_ASSERT_MSG(nDispCrv_CPtIndex == nReg1_CPtCnt + nReg3_CPtCnt - 1,                _T("SmBSplineCurve::DeformToEndPointTargets: Error in index math for DispCrv_CPt array sizes - needs debug")) ; 

  // Note: this is where indexing breaks down for even degree curves.
  // The above Asserts assume the control points in Reg1 and Reg2 have distinct support (aren't defined on the same knots)
  // Even degree curves with no interior knots have a central control point whose domain spans the entire curve.
  // Rather than special casing, we have elevated curves to odd degree.

  // for every Reg1 Cpt
  for(ii=0;ii<nReg1_CPtCnt;ii++)
    {
      // project CurrPts to Cartesian space - and store in SmVector3d
      N_CPtToPtEuclid(pCurrCrv_CPts[ii], &nCurrCrv_Pt) ;
      sInPoint.Set(nCurrCrv_Pt.x, nCurrCrv_Pt.y, nCurrCrv_Pt.z) ;

      // Displace Cartestian CPt and store displacement in nDispCrv
      sDispStartTransform.TransformPoint(sInPoint, sOutPoint) ;
      pDispCrv_CPts[ii].x = sOutPoint.x - sInPoint.x ;
      pDispCrv_CPts[ii].y = sOutPoint.y - sInPoint.y ;
      pDispCrv_CPts[ii].z = sOutPoint.z - sInPoint.z ;
      pDispCrv_CPts[ii].w = NL_NOW ;

    } // end building Reg1 CPt disp values

  // for every Reg3 Cpt
  for(ii=0,
      i0=nReg1_CPtCnt,                // DispCrv index
      i1=nReg1_CPtCnt + nReg2_CPtCnt; // CurrCrv index
      ii<nReg3_CPtCnt;
      ii++,i0++,i1++)
    {
      // project CurrPts to Cartesian space - and store in SmVector3d
      N_CPtToPtEuclid(pCurrCrv_CPts[i1], &nCurrCrv_Pt) ;
      sInPoint.Set(nCurrCrv_Pt.x, nCurrCrv_Pt.y, nCurrCrv_Pt.z) ;

      // Displace Cartestian CPt and store displacement in nDispCrv
      sDispEndTransform.TransformPoint(sInPoint, sOutPoint) ;
      pDispCrv_CPts[i0].x = sOutPoint.x - sInPoint.x ;
      pDispCrv_CPts[i0].y = sOutPoint.y - sInPoint.y ;
      pDispCrv_CPts[i0].z = sOutPoint.z - sInPoint.z ;
      pDispCrv_CPts[i0].w = NL_NOW ;

    } // end building Reg1 CPt disp values

#ifdef SM_DEBUG_CODE
  if (bDebugMe)  // draw/dump CurrCrv and Disp Crv after Reg1 and Reg3 DispCrv knots and CPts are set
    {
      const SmEdge * pEdge = GetEdge() ;
      const SmBrep * pBrep = GetBrep() ;
      SmBSplineCurve dbgDispCrv(&nDispCrv, nDispCrv_Degree, TRUE, GetContext()) ;

      // pretty print Crvs
      smos_WriteBuffer(_T("\nBegin CurrCrv after knot insertions Dump\n")) ;                    Dump_NCrv( pCurrCrv, FALSE) ;
      smos_WriteBuffer(_T("\nEnd CurrCrv after knot insertions Dump\nBegin DispCrv after reg1/reg3 knots and CPts set Dump\n")) ;  Dump_NCrv(&nDispCrv, FALSE) ;
      smos_WriteBuffer(_T("\nEnd DispCrv after reg1/reg3 knots and CPts set Dump\n")) ;

      // draw brep, edge and this SmCurve
      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0,1,1)   ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(2,3, 0,1,0)   ; if(pEdge) pEdge->Draw() ;     sm_GraphicsLoop() ;
      smgfx_SetLook(1,5, 0,0,1)   ; DrawWithKnots() ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 1,.5,.5) ; DrawCurvature() ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 1,0,1)   ; DrawPolygon() ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,7, 1,0,0)   ; dbgDispCrv.DrawWithKnots() ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 1,.5,.5) ; dbgDispCrv.DrawCurvature() ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 1,0,0)   ; dbgDispCrv.DrawPolygon() ; sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;
    }
#endif // end SM_DEBUG_CODE

  // KnotVector to hold CurrCrv Reg2 knots
  NL_KNOTVECTOR *pReg2Knt;
  pReg2Knt = N_AllocKnotVectorAndArray(nCurrCrv_InReg2_KntCnt-1, &SG);

  // Build KnotVector of CurrCrv Reg2 knots
  for(ii=0, i0=nCurrCrv_InReg1_KntCnt + nCurrCrv_Degree + 1 ;
      ii<nCurrCrv_InReg2_KntCnt;
      ii++, i0++)
    {
      pReg2Knt->U[ii] = pCurrCrv_Knts[i0] ;
    }
  
  // insert CurrCrv Reg2 knots into DispCrv
  N_CrvRefine(&nDispCrv,  // in : NURBS curve
              pReg2Knt,   // in : ordered array of new knot values to insert into curP->Knt array
              &nDispCrv,  // out: Curve after adding knots in knx to curP (ok if curP == curA)
              &SG,        // in : curP's stack
              &SG) ;      // in : curQ's stack
  
  // refresh nDispCrv locals
  N_CrvGetCPtsDegreeAndKnots(&nDispCrv,           // in : target curve                                                   
                             &nDispCrv_CPtIndex,  // in : Highest index in control point array                           
                             &pDispCrv_CPts,      // out: array of control points                                        
                             &nDispCrv_Degree,    // out: curve degree                                                   
                             &nDispCrv_KnotIndex, // out: Highest index in KnotArray                                     
                             &pDispCrv_Knts) ;    // out: array of knots (multiple knots are represented multiple times )

  SM_ASSERT_MSG(nDispCrv_CPtIndex == nCurrCrv_CPtIndex, _T("SMBSplineCurve::DeformToEndPointTargets error - CurrCrv and DispCrv are not homogenous - needs debug")) ; 

#ifdef SM_DEBUG_CODE
  if (bDebugMe)  // draw/dump CurrCrv and DispCrv after Reg2 DispCrv knots and CPts are set
    {
      const SmEdge * pEdge = GetEdge() ;
      const SmBrep * pBrep = GetBrep() ;
      SmBSplineCurve dbgDispCrv(&nDispCrv, nDispCrv_Degree, TRUE, GetContext()) ;

      // pretty print Crvs
      smos_WriteBuffer(_T("\nBegin CurrCrv after knot insertions Dump\n")) ;                    Dump_NCrv( pCurrCrv, FALSE) ;
      smos_WriteBuffer(_T("\nEnd CurrCrv after knot insertions Dump\nBegin DispCrv after Reg2 knot insertions Dump\n")) ;  Dump_NCrv(&nDispCrv, FALSE) ;
      smos_WriteBuffer(_T("\nEnd DispCrv after Reg2 knot insertions Dump\n")) ;

      // draw brep, edge and this SmCurve
      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0,1,1)   ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(2,3, 0,1,0)   ; if(pEdge) pEdge->Draw() ;     sm_GraphicsLoop() ;
      smgfx_SetLook(1,5, 0,0,1)   ; DrawWithKnots() ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 1,.5,.5) ; DrawCurvature() ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 1,0,1)   ; DrawPolygon() ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,7, 1,0,0)   ; dbgDispCrv.DrawWithKnots() ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 1,.5,.5) ; dbgDispCrv.DrawCurvature() ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 1,0,0)   ; dbgDispCrv.DrawPolygon() ; sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;
    }
#endif // end SM_DEBUG_CODE

  // Next: pCurrCrv = pCurrCrv + pDispCrv - Add CPt positions
  NL_POINT  nCartDispCPt, nCurrCPt, nNewCPt;

  // for every control point - add Cpt positions
  for(ii=0;ii<=nCurrCrv_CPtIndex;ii++)
    {
      N_CPtToPt (pDispCrv_CPts[ii], &nCartDispCPt) ;                   // drops the DispCrv w values which is ok since they are all NL_NOW
      N_CPtToPtEuclid(pCurrCrv_CPts[ii], &nCurrCPt) ;                  // get the Euclidean CPt of the current curve
      N_Sum2Pts(nCartDispCPt, nCurrCPt, &nNewCPt) ;                    // Sum the Euclidean points
      N_Weight  (nNewCPt, pCurrCrv_CPts[ii].w, &pCurrCrv_CPts[ii] ) ;  // converts new cartesian CPts into homogeneous coordinates with the original weight
    } // end adding every CPt

  // Notify system of upcoming change to this BSplineCurve
  Notify(SM_NO_PRE_EDIT, this, SM_NO_GET_OWNER(this), NULL);

  // note: for historical reason every BSplineCurveNLib edit function has to do an extra memory copy
  //       In NLib, mem for every NL_CURVE object and array is managed as its own memory
  //          those ptrs are copied into a NLib memory stack for later garbage collection.
  //          When the memory stack is freed all mem blocks for objects and arrays are freed.
  //       In SMLib mem for gw_CURVE is allocated and freed in a single block
  //          object ptrs within the gw_Curve are set to point to offsets within this one block
  //       BSplineCurveNLib functions use NLib memory stacks to edit NL_CURVE data structures.
  //          whenever a CPt or knot count changes new appropriately sized CPt and Knot arrays
  //          are allocated, placed on the memstack and pointed to by the NL_CURVE obj being edited.
  //          When the edit is done the NLib NL_CURVE data is non-contiguous and the pointed
  //           to by the NLib memory stacks.
  //          When the method closes the memory stack is deleted and all obj pointed to by the stack get deleted.
  //       Persistence is achieved by
  //          a. copying the BSplineNLib result into a SMLib allocated block
  //               (advantages: GW_CURVE memory is contiguous and GW_CURVE is not pointed to by a NLib memory stack)
  //          b. using that GW_CURVE memory block to create a temporary SmBSplineCurve
  //          c. and swapping the this and temporary m_pNurb ptrs (pointing to different sized mem blocks).
  //          d. On Exit,
  //                o. temporary SmBSplineCurve is deleted which frees the original m_pNurb block
  //                o. NLib memory stack SG is deleted - which deletes all NLib allocated objects
  //                o. leaving the this SmBSplineCurve pointing to a persistent new NL_CURVE block copied from the modified NLib NL_CURVE data

  // copy NLib NL_CURVE memory struct (managed by NLib memory stack) into SMLib GW_CURVE contiguous memory block (not managed by NLib memory stack)
  SmBSplineCurve *pTmpBSC = new (*GetContext()) SmBSplineCurve(GetDim(),(gw_CURVE *)pCurrCrv);
  SmObjDelete sClean(pTmpBSC);

  // Swap m_pNurb objects between this and pTmpBSC BSplineCurve
  //   moves the new m_pNurb to the persistent this obj
  //   moves the old m_pNurb to the temporary pTmpBSC obj - avoids a memory leak
  gw_CURVE *pTmp   = pTmpBSC->m_pNurb;
  pTmpBSC->m_pNurb = m_pNurb;
  m_pNurb          = pTmp;

  // notify the public - curve editing is done
  Notify(SM_NO_POST_EDIT, this, SM_NO_GET_OWNER(this), NULL);

#ifdef SM_DEBUG_CODE
bDebugMe = FALSE ;
  if (bDebugMe)  // draw/dump CurrCrv and DispCrv after Reg2 DispCrv knots and CPts are set
    {
      const SmEdge * pEdge = GetEdge() ;
      const SmBrep * pBrep = GetBrep() ;

      smos_WriteBuffer(_T("\nBegin InCurve Dump\n")) ;                    SM_DUMP_AND_ASSERT_VALID(&sDbgThisCpy) ;
      smos_WriteBuffer(_T("\nEnd InCurve Dump\nBegin OutCurve Dump\n")) ; SM_DUMP_AND_ASSERT_VALID(this) ;
      smos_WriteBuffer(_T("\nEnd OutCurve Dump\n")) ;
      
      // draw brep, edge and this SmCurve
      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0,1,1)   ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(2,3, 0,1,0)   ; if(pEdge) pEdge->Draw() ;     sm_GraphicsLoop() ;
      smgfx_SetLook(3,6, 0,0,1)   ; sDbgThisCpy.DrawWithKnots() ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 1,.5,.5) ; sDbgThisCpy.DrawCurvature() ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 1,0,1)   ; sDbgThisCpy.DrawPolygon() ; sm_GraphicsLoop() ;
      smgfx_SetLook(4,8, 1,0,0)   ; DrawWithKnots() ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 1,.5,.5) ; DrawCurvature() ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 1,0,0)   ; DrawPolygon() ; sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;
    }
#endif // end SM_DEBUG_CODE

  // all done
  return SM_SUCCESS;

} // end SmBSplineCurve::DeformToEndPointTargets

/*******************************************************************//**
PURPOSE: This routine computes a NURBS piecewise circular curve 
    interpolating a given set of points.

NOTES:  The parametrization is G1.
***********************************************************************/
SmStatus SmBSplineCurve::PiecewiseArcInterpolate
  (const SmContext & crContext,
   const SmTArray<SmPoint3d> & crPoints,
   SmInterpolationType eParameterization,
   SmBSplineCurve *& rpNewCurve)
{
  NL_STACKS SG;
  SmNLibStackHandler sSH(&SG);
  NL_CURVE   cur;
  N_CrvInitArrays(&cur);

  NL_INDEX  m = crPoints.GetSize()-1;
  NL_POINT  *P = N_AllocPt1dArray(m,&SG);

  for (ULONG i=0; i<crPoints.GetSize(); i++) {
      SmPoint3d sPnt = crPoints[i];
      COPY_XYZ(sPnt,P[i]);
  }

  NL_FLAG   par;
  switch (eParameterization) {
  case SM_IT_UNIFORM:
      par = NL_UNIFORM;
      break;
  case SM_IT_CENTRIPETAL:
      par = NL_CENTRIPETAL;
      break;
  case SM_IT_CHORDLENGTH:
  default:
      par = NL_CHORDLENGTH;
      break;
  }
  NL_SER(N_FitCrvArcs(P,m,par, NL_AKIMA,&cur,&SG));

  rpNewCurve = new (crContext) SmBSplineCurve(3,(gw_CURVE *)&cur);
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
  if (bDebugMe) {
      sm_GraphicsLoop();
      smgfx_SetColor(1,0,0);
      rpNewCurve->Draw();
      rpNewCurve->Dump();
  }
#endif // end SM_DEBUG_CODE

  return SM_SUCCESS;

} // end SmBSplineCurve::PiecewiseArcInterpolate

/*******************************************************************//**
PURPOSE: This approximation routine refits the given NURBS curve to the
         given points, with optional fixed points.

NOTES: This method is only available for users with NLib.
       "This" curve is modified in place.

       Exposes N_FitCrvApproxPts().

ARGUMENTS: All arguments except the first are optional.
   crApproxPts       in: points to approximate more closely
   pFixIndices       in: indices into crApproxPts of points to be interpolated
                         Default: NULL
   pFixParams        in: parameters on the curve of positions that are not allowed to move
                         Default: NULL
   lFixEndsFlag      in: 0: no constraints  (Default)
                         1: fix the positions of the curve's end points
                         2: fix end positions and tangent directions
                         3: fix end positions and first derivatives
   dAlpha0, dAlpha1  in: start and end values of alpha regularization term; see notes below
                         Default: -1 for both: allow the algorithm to decide; see notes below
   dBeta0,  dBeta1   in: start and end values of beta  regularization term; see notes below
                         Default: -1 for both: allow the algorithm to decide; see notes below
   pErrors           out: resulting fit errors at each point
                         Default: NULL

Notes: regularization terms
   Regularization helps to maintain a fair curve, and is used because
   in theory, the very "best" curve for the data, in terms of minimizing
   the distance to each data point, would be full of kinks and loops.
   The first term "alpha" tends to minimize the arc length of the
   resulting curve, keeping it short (to avoid big loops), and
   the other term "beta" minimizes curvature, keeping the curve "stiff"
   (to avoid wiggles).

   These regularization term values are passed in as a starting value
   and an ending value, because they work best when they decrease as
   the iteration proceeds.  If values are passed in negative, default
   values will be used.  Smaller values will result in a better fit,
   but the curve might develop kinks or loops.  The values should not
   be large: the current defaults are alpha_0 = alpha_1 = 0 (i.e., no
   alpha correction), and beta_0 = 0.0001, beta_1 = 0.000008.

   Regularization can be applied only to nonrational cubic curves.

   More details can be found in the documentation of the NLib routine N_FitCrvApproxPts().
   
***********************************************************************/
SmStatus SmBSplineCurve::ApproximateRefit
 ( const SmTArray< SmPoint3d > & crApproxPts,
   SmTArray< ULONG  >          * pFixIndices,
   SmTArray< double >          * pFixParams,
   ULONG                         lFixEndsFlag,
   double                        dAlpha0, 
   double                        dAlpha1,
   double                        dBeta0 , 
   double                        dBeta1,
   SmTArray< double >          * pErrors )
{
  // locals
  NL_STACKS  SG;
  SmNLibStackHandler sSH(&SG);
  NL_CURVE * cur = GetOrCreateGwNurbPointer();  NER( cur );

  NL_POINT *xPts  = SM_REINTERPRET_CAST( NL_POINT*, crApproxPts.GetDataArray() );
  NL_INDEX xPtsCount = crApproxPts.GetSize() - 1;

  NL_INDEX numFixes = -1;
  NL_INDEX *fixIndices = NULL;
  if ( pFixIndices != NULL )
  {
      fixIndices = SM_REINTERPRET_CAST( NL_INDEX*, pFixIndices->GetDataArray() );
      numFixes   = pFixIndices->GetSize() - 1;
  }

  NL_REAL *fixParams = NULL;
  if ( pFixParams != NULL )
  {
      fixParams = SM_REINTERPRET_CAST( NL_REAL*, pFixParams->GetDataArray() );
      numFixes  = pFixParams->GetSize() - 1;
  }

  NL_REAL *errors = NULL;
  if ( pErrors != NULL )
  {
      if ( pErrors->GetSize() < crApproxPts.GetSize() )
      {
          pErrors->SetSize( crApproxPts.GetSize() );
      }
      errors = SM_REINTERPRET_CAST( NL_REAL*, pErrors->GetDataArray() );
  }

  NL_FLAG fixEndPoints = (NL_FLAG)lFixEndsFlag;

  // fit the points with a curve
  NL_FLAG nlibStat = N_FitCrvApproxPts( xPts,          // in : TgtPts to approx with a curve   
                                        xPtsCount,     // in : highest index in TgtPts array:[PtsIndx = No. of Points-1]   
                                        cur,           // i/o: Ptr to Approximating curve being built,    
                                                       //      NULL    = make a new Curve with (CptsMaxIndx + 1) Cpts   
                                                       //      NotNULL = Move InputCrv's existing CPts to make the best Approx Crv possible.   
                                        -1,            // in : When PtrToCurve = NULL, highest OutCurve ControlPoint [CPtsIndx = No. of CPts -1]   
                                        numFixes,      // in : No. of Points on OutCurve that are to be interpolated   
                                        fixIndices,    // in : NotNULL = indices of Pts in XPts array to be treated as fixed, NULL to ignore.   
                                        fixParams,     // in : NotNULL = Crv params to interpolate the fixed points, NULL to ignore.   
                                        fixEndPoints,  // in : 0 = No Constraints   
                                                       //      1 = Constrained Curve EndPts   
                                                       //      2 = Constrained Curve EndPts and EndTangent directions (Unconstrained speeds)   
                                                       //      3 = Constrained Curve EndPts and 1st Derivs            (constrained speeds)   
                                        dAlpha0,       // in : start value of alpha (resist stretch) regularization term - neg values = use defaults   
                                        dAlpha1,       // in : end value of alpha (resist stretch) regularization term   - neg values = use defaults   
                                        dBeta0,        // in : start value of beta (resist bending) regularization term - neg values = use defaults   
                                        dBeta1,        // in : end value of beta resist bending) regularization term   - neg values = use defaults   
                                        errors,        // out: error values for each point, NULL to ignore   
                                        &SG );         // i/o: crv's stack   
  return ( nlibStat == NL_NO ) ? SM_SUCCESS : SM_ERR;

} // end SmBSplineCurve::ApproximateRefit

/*******************************************************************//**
PURPOSE: This approximation routine approximates the given NURBS
    curve by evaluating points and using those points to approximate a new curve.

NOTES: This method is only available for users with NLib
       "This" curve is left unaffected    
***********************************************************************/
SmStatus SmBSplineCurve::RebuildCurve
 ( const SmContext & crContext,    // in : new object context 
   double* dOptTol,                // in : 
   double* dOptMaxDev,             // out: max deviation of result from input curve
   SmBSplineCurve *& rpNewCurve)   // out: new approx curve
{
    // init output
    SM_ASSERT(rpNewCurve == NULL) ;
    rpNewCurve = NULL ;

    SmCurve* pCopy = NULL;
    this->Copy( crContext, pCopy );
    SmBSplineCurve* pCrv = (SmBSplineCurve*)pCopy;

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
    if (bDebugMe) {
        smgfx_SetLook( 1,1, 0,1,0 );
        smgfx_Erase(); pCrv->Draw( ); sm_GraphicsLoop();
    }
#endif // end SM_DEBUG_CODE

    pCrv->ReparametrizeWithArcLength();

    double dTol = 0.001;
    if( dOptTol ) {
        dTol = *dOptTol;
    }

    // Evaluate points on curve
    SmTArray<SmPoint3d> sPtsToFit;
    SmPoint3d pt;
    ULONG ii;
    ULONG nIncrements = 20;   // this should be a function of size?
   
    SmExtent1d domain = pCrv->GetNaturalInterval();
    double min = domain.GetMin();
    double max = domain.GetMax();
    double param = min;
    double paramIncr = (max - min) / nIncrements;

    for( ii = 0; ii <= nIncrements; ii++ ) {
        pCrv->EvaluatePoint( param, pt );
        sPtsToFit.Add( pt );
        param = param + paramIncr;
    }

#ifdef SM_DEBUG_CODE
    if (bDebugMe) {
        smgfx_SetLook( 2,2, 1,0,0 );
        for( ii = 0; ii < sPtsToFit.GetSize(); ii++ ) {
            sPtsToFit[ii].Draw( ); sm_GraphicsLoop();
        }
    }
#endif // end SM_DEBUG_CODE

    SmVector3d sStartPntDer[2];
    pCrv->Evaluate( min, 1, true, sStartPntDer );

    SmVector3d sEndPntDer[2];
    pCrv->Evaluate( max, 1, true, sEndPntDer );

    SmBoolean bIsClosed = pCrv->IsClosed( pCrv->GetNaturalInterval(), dTol );

    // approx fit new surface to set of points
    SmBSplineCurve* pNewCrv = NULL;
    SmStatus stat = SmBSplineCurve::ApproximatePoints( crContext, sPtsToFit, 3, &sStartPntDer[1], &sEndPntDer[1], bIsClosed, dOptTol, pNewCrv );
    if( stat != SM_SUCCESS ) { 
        return( stat );
    }

    if( pNewCrv == NULL ) {
        return( SM_ERR );
    }

    // return curve
    rpNewCurve = pNewCrv;

#ifdef SM_DEBUG_CODE
    if (bDebugMe) {
        smgfx_SetLook( 3,3, 0,0,1 ); pNewCrv->Draw( ); sm_GraphicsLoop();
    }
#endif // end SM_DEBUG_CODE

    // Now let's check the max diff
    if( dOptMaxDev != NULL ) 
      {
        *dOptMaxDev = 0.0;

        SmBoolean isSuccess;
        double resultParam;
        double diff;
   
        for( ii = 0; ii < sPtsToFit.GetSize(); ii++ ) 
          {
            pNewCrv->DropPoint(pNewCrv->GetNaturalInterval(), // in : target curve allowed domain
                               sPtsToFit[ii],                 // in : Point to drop to curve
                               NULL,                          // in : For Snapping, NULL=NoSnap, Vec pointing to CrvInside from desired EndPt for snapping.
                                                              //      when DropPt is within dDistTol in cpVecToCrvInside dir of EndPt, Snap to Ivl EndPt.
                                                              //      Vec handles ambiguities and makes sure that curves are not just touching at the ends.
                               dTol,                          // in : Used for SnapDist, and SM_SO_INTERSECT sXSectTol3d distance.
                                                              //      sXSectTol3d = dDistTol ;      SM_SO_INTERSECT 
                                                              //      sXSectTol3d = SM_BIG_DOUBLE ; SM_SO_MINIMIZE, SM_SO_NORMALIZE
                                                              //      sSnapTol3d  = dDistTol ;      SM_SO_MINIMIZE, SM_SO_NORMALIZE, SM_SO_INTERSECT    
                               NULL,                          // in : If given, call cheap LocalPointSolve() else call expensive GlobalPointSolve()
                               isSuccess,                     // out: TRUE = found a drop point
                               resultParam,                   // out: found drop curve param
                               diff) ;                        // out: found drop distance
                                                              // in : SM_SO_MINIMIZE = allow nonNormal drops near endPoints
                                                              //      SM_SO_INTERSECT= like Minimize but point must be within dDistTol
                                                              //      SM_SO_NORMALIZE= exclude nonNormal drops near endPoints
                                                              //      default:[SM_SO_MINIMIZE] to preserve original behavior
            if( isSuccess && diff > *dOptMaxDev ) 
              {
                *dOptMaxDev = diff;
          }
      }
  }

    return( SM_SUCCESS );

} // end SmBSplineCurve::RebuildCurve

/*******************************************************************//**
PURPOSE: This approximation routine approximates the given NURBS curve by
    evaluating evenly spaced arc length points and using those points to 
    approximate a new curve.

NOTES: This method is only available for users with NLib
       "This" curve is left unaffected    
***********************************************************************/
SmStatus SmBSplineCurve::RebuildCurveWithArcLengthParam
 ( const SmContext & crContext,    // in : new object context 
   ULONG nIntervals,               // in : number of intervals (passed to EquallySpacedPoints)
                                   //      select small number for relatively linear curves
                                   //      select large number for larger, wavy, curves 
                                   //      if zero, use default calculation
   double* dOptTol,                // in : 3d tolerance for evenly spaced (passed to EquallySpacedPoints)
   double* dOptMaxDev,             // out: max deviation of result from input curve
   SmBSplineCurve *& rpNewCurve)   // out: new approx curve
{
    // init output
    SM_ASSERT(rpNewCurve == NULL) ;
    rpNewCurve = NULL ;

    SmCurve* pCopy = NULL;
    this->Copy( crContext, pCopy );
    SmBSplineCurve* pCrv = (SmBSplineCurve*)pCopy;

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
    if (bDebugMe) {
        smgfx_SetLook( 1,1, 0,1,0 );
        smgfx_Erase(); pCrv->Draw( ); sm_GraphicsLoop();
    }
#endif // end SM_DEBUG_CODE

    pCrv->ReparametrizeWithArcLength();

    double dTol = 0.001;
    if( dOptTol ) {
        dTol = *dOptTol;
    }

    // Evaluate points on curve using 3d length
    SmTArray<SmPoint3d> sPtsToFit;

    SmExtent1d domain = pCrv->GetNaturalInterval();
    double min = domain.GetMin();
    double max = domain.GetMax();

    // If no input value, use default calculation
    if( nIntervals == 0 ) {
        nIntervals = pCrv->GetNumberControlPoints();
    }

    SmTArray<double> sParams;
    pCrv->EquallySpacedPoints( min, max, nIntervals, dTol, &sPtsToFit, &sParams );
   
#ifdef SM_DEBUG_CODE
    if (bDebugMe) {
        smgfx_SetLook( 2,2, 1,0,0 );
        for( ULONG ii = 0; ii < sPtsToFit.GetSize(); ii++ ) {
            sPtsToFit[ii].Draw( ); sm_GraphicsLoop();
        }
    }
#endif // end SM_DEBUG_CODE

    SmVector3d sStartPntDer[2];
    pCrv->Evaluate( min, 1, true, sStartPntDer );

    SmVector3d sEndPntDer[2];
    pCrv->Evaluate( max, 1, true, sEndPntDer );

    SmTArray<SmVector3d> sDerivatives;
    sDerivatives.Add( sStartPntDer[1] );
    sDerivatives.Add( sEndPntDer[1] );

    //SmBoolean bIsClosed = pCrv->IsClosed( pCrv->GetNaturalInterval(), dTol );

    // approx fit new surface to set of points
    SmBSplineCurve* pNewCrv = NULL;
    SmStatus stat = SmBSplineCurve::CreateInterpolatingCurve( crContext, SM_CP_CHORDLENGTH, 3, 3, sPtsToFit, sDerivatives, NULL, TRUE, pNewCrv );
   
    if( stat != SM_SUCCESS ) { 
        return( stat );
    }

    if( pNewCrv == NULL ) {
        return( SM_ERR );
    }

    // return curve
    rpNewCurve = pNewCrv;

#ifdef SM_DEBUG_CODE
    if (bDebugMe) {
        smgfx_SetLook( 3,3, 0,0,1 ); pNewCrv->Draw( ); sm_GraphicsLoop();
    }
#endif // end SM_DEBUG_CODE

    // Now let's check the max deviation
    if( dOptMaxDev != NULL ) 
      {
        *dOptMaxDev = 0.0;

        SmBoolean isSuccess;
        SmPoint3d midPnt;
        double dMidParam, dResultParam;
        double dDeviation;

        for ( ULONG ii = 1; ii < sParams.GetSize(); ii++ ) 
          {
            dMidParam = ( sParams[ii-1] + sParams[ii] ) / 2.0;
            this->EvaluatePoint( dMidParam, midPnt );

            pNewCrv->DropPoint(pNewCrv->GetNaturalInterval(), // in : target curve allowed domain 
                               midPnt,                        // in : Point to drop to curve
                               NULL,                          // in : For Snapping, NULL=NoSnap, Vec pointing to CrvInside from desired EndPt for snapping.
                                                              //      when DropPt is within dDistTol in cpVecToCrvInside dir of EndPt, Snap to Ivl EndPt.
                                                              //      Vec handles ambiguities and makes sure that curves are not just touching at the ends.
                               dTol,                          // in : Used for SnapDist, and SM_SO_INTERSECT sXSectTol3d distance.
                                                              //      sXSectTol3d = dDistTol ;      SM_SO_INTERSECT 
                                                              //      sXSectTol3d = SM_BIG_DOUBLE ; SM_SO_MINIMIZE, SM_SO_NORMALIZE
                                                              //      sSnapTol3d  = dDistTol ;      SM_SO_MINIMIZE, SM_SO_NORMALIZE, SM_SO_INTERSECT    
                               NULL,                          // in : If given, call cheap LocalPointSolve() else call expensive GlobalPointSolve()
                               isSuccess,                     // out: TRUE = found a drop point
                               dResultParam,                  // out: found drop curve param
                               dDeviation) ;                  // out: found drop distance
                                                              // in : SM_SO_MINIMIZE = allow nonNormal drops near endPoints
                                                              //      SM_SO_INTERSECT= like Minimize but point must be within dDistTol
                                                              //      SM_SO_NORMALIZE= exclude nonNormal drops near endPoints
                                                              //      default:[SM_SO_MINIMIZE] to preserve original behavior
            if( isSuccess && dDeviation > *dOptMaxDev ) 
              {
                *dOptMaxDev = dDeviation;
              }
          }
      }

    return( SM_SUCCESS );

} // end SmBSplineCurve::RebuildCurveWithArcLengthParam

/*******************************************************************//**
PURPOSE: Reparameterize a curve to generally arc-length parameterization.

NOTES: This method is only available for users with NLib.

   Note that this does not result in constant-speed parameterization,
   even approximately.  It simply modifies the overall parameterization
   to agree generally with the oveall arc length.  It does not change
   the shape of the curve, only the magnitude of its derivatives.

   The algorithm in the NLib routine checks for knots with multiplicities
   >= the degree, including the end knots.  Between each pair of such
   multiple knots, it calculates the arc length, and scales all knots
   in between such that the parameter delta of the whole span equals
   the arc length of the span.
***********************************************************************/
SmStatus SmBSplineCurve::ReparametrizeWithArcLength()
{
  // locals
  double dRelativeTol = 1.0e-3;
  NL_STACKS  SG;
  SmNLibStackHandler sSH(&SG);
  NL_CURVE * cur = GetOrCreateGwNurbPointer();

  // Build new curve
  NL_CURVE curNew;
  N_CrvInitArrays(&curNew);

  // Call the NLib routine, and put the result into curNew.
  NL_SER(N_SrfReparamMultKnots(cur,dRelativeTol,&curNew,&SG));

  Notify(SM_NO_PRE_EDIT, this, SM_NO_GET_OWNER(this), NULL);
  SmBSplineCurve *pTmpBSC = new (*GetContext()) SmBSplineCurve(GetDim(),(gw_CURVE *)&curNew);
  SmObjDelete sClean(pTmpBSC);

  // Swap Nurbs with this - prevents memory leaks
  gw_CURVE *pTmp   = pTmpBSC->m_pNurb;
  pTmpBSC->m_pNurb = m_pNurb;
  m_pNurb          = pTmp;
  Notify(SM_NO_POST_EDIT, this, SM_NO_GET_OWNER(this), NULL);

  // all done
  return SM_SUCCESS;

} // end SmBSplineCurve::ReparametrizeWithArcLength

/*******************************************************************//**
PURPOSE: This routine refines a NURBS curve with a given knot vector.

NOTES: It is assumed that the new knot vector 'fits' into the old one. 
***********************************************************************/
SmStatus SmBSplineCurve::RefineCurve
  (const SmTArray<double> & crNewKnots)
{
  NL_STACKS SG;
  SmNLibStackHandler sSH(&SG);
  NL_CURVE   curNew;
  N_CrvInitArrays(&curNew);

  NL_KNOTVECTOR  *knt;
  NL_INDEX       m = crNewKnots.GetSize()-1;
  knt = N_AllocKnotVectorAndArray(m,&SG);
  NL_REAL *U = knt->U;
  ULONG i;
  for (i=0; i<crNewKnots.GetSize(); i++) {
      U[i] = crNewKnots[i];
  }

  NL_CURVE * curP = GetOrCreateGwNurbPointer();

  NL_SER(N_CrvRefine(curP,knt,&curNew,&SG,&SG));

  Notify(SM_NO_PRE_EDIT, this, SM_NO_GET_OWNER(this), NULL);
  SmBSplineCurve *pTmpBSC = new (*GetContext()) SmBSplineCurve(GetDim(),(gw_CURVE *)&curNew);
  SmObjDelete sClean(pTmpBSC);

  // Swap Nurbs with this
  gw_CURVE *pTmp   = pTmpBSC->m_pNurb;
  pTmpBSC->m_pNurb = m_pNurb;
  m_pNurb          = pTmp;
  Notify(SM_NO_POST_EDIT, this, SM_NO_GET_OWNER(this), NULL);

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
  if (bDebugMe) {
      sm_GraphicsLoop();
      smgfx_SetColor(1,0,0);
      Draw();
      Dump();
  }
#endif // end SM_DEBUG_CODE

  return SM_SUCCESS;

} // end SmBSplineCurve::RefineCurve

/*******************************************************************//**
PURPOSE: Refine curve by knot removal and adjusting control points

NOTES: This method is only available for users with NLib
            Use a large tolerance (1000*pbrep->tol) for knot removal
            If degree is > 3, it will fit with a cubic. (no knot removal)
***********************************************************************/
  
SmStatus SmBSplineCurve::Refine( double dKnotTol )   // input remove-knot tolerance 
{
  NL_STACKS S;

  NL_CURVE * cur = GetOrCreateGwNurbPointer();
  NL_CURVE   curNew;
  N_CrvInitArrays(&curNew);
  
  NL_INDEX j = 0;
  NL_INDEX nPts = 0; 
  NL_PARAMETER u0, u1, u, uinc, uhi;
  NL_FLAG error = 0;
  N_CrvGetParamBounds(cur, &u0, &u1);

  // target PntsPerSpan (5 for a cubic) sample points between every non-zero span
  int PntsPerSpan = cur->p + 2; 
  NL_INDEX mMax = PntsPerSpan * (cur->knt->m - 2 * cur->p) + 1; // worst case - no multiple knots

  // when you have lots of points, reduce points per span.
  while (mMax > 800 && PntsPerSpan > 1) 
  {
     PntsPerSpan -= 1;
     mMax = PntsPerSpan * (cur->knt->m - 2 * cur->p) + 1;
  }
  // when you have too few points, increase points per span
  while (mMax < 50 && PntsPerSpan < 10) 
  {
     PntsPerSpan += 1;
     mMax = PntsPerSpan * (cur->knt->m - 2 * cur->p) + 1;
  }

  if (mMax < 30)   // dont refine small-span count curves
      return(SM_SUCCESS);

  // Stacks not used until now
  N_InitNurbs(&S);


  // remove over-multiple knots
  int nRemoved = 0;
  error = N_tooCrvCleanSpans(cur, &nRemoved, &S);

  NL_POINT *Pnts = N_AllocPt1dArray(mMax, &S); 
  NL_INDEX low = cur->p;
  NL_INDEX hi = cur->knt->m - cur->p;

  j = low;   // last of the first multiple knots
  u =  cur->knt->U[j];
  NL_INDEX k = j;
  while (cur->knt->U[k] <= u) k++;
  uhi =  cur->knt->U[k]; 
  do {

      uinc = (uhi - u)/(PntsPerSpan);
      for ( int jj = 0; jj < PntsPerSpan; jj++) 
      {
         N_CrvEval(cur, u, NL_LEFT, &Pnts[nPts++]);
         u = u + uinc;
      }
      u = uhi;
      while (cur->knt->U[j] <= (u + SM_EFF_ZERO) && j <= hi) j++;
      uhi = cur->knt->U[j];
  } while (j <= hi);

  //get the last point = first of the last multiple knots
  N_CrvEval(cur, cur->knt->U[hi], NL_LEFT, &Pnts[nPts]);

  // if degree > 3 Exit
  if (cur->p > 3)
  {
      N_EndNurbs(&S);
      return(SM_ERR);
  }

  
  // else aggressively remove knots off the old curve (use large dKnotTol)
  N_CrvRemoveKnots(cur, dKnotTol, &curNew, &S);
  
  
  NL_REAL    *allErrors;   
  NL_FLAG    fixEndPts = 2;   //hold end points & tangents       
  allErrors = N_AllocReal1dArray(nPts + 1, &S);    
  
  error = N_FitCrvApproxPts(
      Pnts, nPts, &curNew, -1, 0, NULL, NULL, fixEndPts,
      -1.0, -1.0, -1.0, -1.0,
      allErrors, &S);
  if (error != 0) {
      N_EndNurbs(&S);
      return(SM_ERR);
  }
  
  
  Notify(SM_NO_PRE_EDIT, this, SM_NO_GET_OWNER(this), NULL);
  SmBSplineCurve *pTmpBSC = new (*GetContext()) SmBSplineCurve(GetDim(),(gw_CURVE *)&curNew);
  SmObjDelete sClean(pTmpBSC);

  // Swap Nurbs with this
  gw_CURVE *pTmp   = pTmpBSC->m_pNurb;
  pTmpBSC->m_pNurb = m_pNurb;  // the old nurb gets swapped for deallocation by sClean
  m_pNurb          = pTmp;     // the new NURB gets saved.in the calling class
  Notify(SM_NO_POST_EDIT, this, SM_NO_GET_OWNER(this), NULL);

  N_EndNurbs(&S);
   
  return SM_SUCCESS;

} // end SmBSplineCurve::Refine

/*******************************************************************//**
PURPOSE: This routine removes all removable  knots from a NURBS curve
    with possible end derivative constraints applied. 

NOTES: This method is only available for users with NLib
***********************************************************************/
SmStatus SmBSplineCurve::RemoveKnots
  (double    dThisApproxTol3d,         // in : max allowed Current/New Curve deviation
   SmBoolean bConstrainEndDeriv,       // in : TRUE = Constrain end derivatives, FALSE = don't
   ULONG     lHighestDerivConstraint)  // in : Highest derivative constraint
                                       //      i.e. it maintains the 1-st to the
                                       //      lHighest'th derivatives at the end points
{
  NL_STACKS SG;
  SmNLibStackHandler sSH(&SG);
  NL_CURVE   curNew;
  N_CrvInitArrays(&curNew);

  NL_FLAG   constraints = NL_NO;
  NL_INDEX  der = 0;
  if (bConstrainEndDeriv) {
      constraints = NL_BOTH;
      der = lHighestDerivConstraint;
  }

  NL_CURVE * curP = GetOrCreateGwNurbPointer();

  NL_SER(N_CrvRemoveKnotsDerivConstraints(curP,dThisApproxTol3d,constraints,der,&curNew,&SG));

  Notify(SM_NO_PRE_EDIT, this, SM_NO_GET_OWNER(this), NULL);
  SmBSplineCurve *pTmpBSC = new (*GetContext()) SmBSplineCurve(GetDim(),(gw_CURVE *)&curNew);
  SmObjDelete sClean(pTmpBSC);

  // Swap Nurbs with this
  gw_CURVE *pTmp   = pTmpBSC->m_pNurb;
  pTmpBSC->m_pNurb = m_pNurb;
  m_pNurb          = pTmp;
  Notify(SM_NO_POST_EDIT, this, SM_NO_GET_OWNER(this), NULL);

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
  if (bDebugMe) {
      sm_GraphicsLoop();
      smgfx_SetColor(1,0,0);
      Draw();
      Dump();
  }
#endif // end SM_DEBUG_CODE

  return SM_SUCCESS;

} // end SmBSplineCurve::RemoveKnots

/*******************************************************************//**     
PURPOSE: This routine removes one knot multiple times from a NURBS           
         curve.                                                              
                                                                             
NOTES: The knot must be an interior knot. 
       Knots are not removed if the knot removal changes the shape of
         the curve by more than dThisApproxTol3d.
***********************************************************************/
SmStatus SmBSplineCurve::RemoveOneKnot
 (double  dKnot,              // in : Knot to be removed
  ULONG   lNumKnotsRemoval,   // in : Number of times of removal, set to degree to remove all knot multiplicities
  double  dThisApproxTol3d,   // in : max allowed Current/New Curve deviation
  ULONG & rlNumKnotsRemoved)  // out: Actual number of knots removed
{
  // locals
  NL_CURVE            * curP = GetOrCreateGwNurbPointer(); NER(curP) ;
  NL_CURVE              curNew;
  NL_INDEX              nu = lNumKnotsRemoval;
  NL_INDEX              ru;
  NL_STACKS             SG;
  SmNLibStackHandler sSH(&SG);
  N_CrvInitArrays(&curNew);

  // pass the call along to NLib
  NL_SER(N_CrvRemoveKnot(curP,              // in : Target NURBS curve
                         dKnot,             // in : Knot value to be removed
                         nu,                // in : number of times to remove the knot
                         dThisApproxTol3d,  // in : Tolerance to check removability
                         &ru,               // out: Number of knots removed
                         &curNew,           // out: Curve copy with knot removed
                         &SG));             // in : curNew's stack
  rlNumKnotsRemoved = ru;

  // inform the world Curve is being modified
  Notify(SM_NO_PRE_EDIT, this, SM_NO_GET_OWNER(this), NULL);

  // build a SMLib BSplineCurve from a NLib gw_CURVE
  SmBSplineCurve *pTmpBSC = new (*GetContext()) SmBSplineCurve(GetDim(),(gw_CURVE *)&curNew);
  SmObjDelete sClean(pTmpBSC);

  // Swap Nurbs between Curves
  gw_CURVE *pTmp   = pTmpBSC->m_pNurb;
  pTmpBSC->m_pNurb = m_pNurb;
  m_pNurb          = pTmp;

  // inform the word Curve is done being modified
  Notify(SM_NO_POST_EDIT, this, SM_NO_GET_OWNER(this), NULL);

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
  if (bDebugMe) 
    {
      Dump();

      smgfx_SetLook(1,2, 1,0,0); Draw(); sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

  // all done
  return SM_SUCCESS;

} // end SmBSplineCurve::RemoveOneKnot

/*******************************************************************//**
PURPOSE: Scale a curve by a given factor.

NOTES: This method is only available for users with NLib

   This method is not a 'geometric' scaling, it simply multiplies each
   control point by a factor.  In particular, if the curve is rational,
   it will multiply the weight as well.  This means that for rational
   curves, this will not change the shape (size) of the curve in 3d at all.

   To scale a curve geometrically, use the other overloaded Scale() method,
   which takes a vector scale factor and a reference point for the scaling.
   Or use SmAxis2Placement.
    
***********************************************************************/
SmStatus SmBSplineCurve::Scale
  (double dScaleFactor)
{
  NL_CURVE * cur = GetOrCreateGwNurbPointer();
  Notify(SM_NO_PRE_EDIT, this, SM_NO_GET_OWNER(this), NULL);
  N_CrvScaleCPts(cur,dScaleFactor);
  Notify(SM_NO_POST_EDIT, this, SM_NO_GET_OWNER(this), NULL);
  return SM_SUCCESS;

} // end SmBSplineCurve::Scale

/*******************************************************************//**
PURPOSE: Scale a curve with respect to a point.

NOTES: This method is only available for users with NLib

   Caution: If the curve is an analytic type (e.g., SmCircle), and the
   scaling vector is not isotropic, the resulting shape might not be
   what it is supposed to be.  This could cause real problems.
   Be very careful with anisotropic scaling.
    
***********************************************************************/
SmStatus SmBSplineCurve::Scale
  (SmVector3d& rScaleVec, SmPoint3d& rScaleCenter)
{
  NL_CURVE * cur = GetOrCreateGwNurbPointer();
  Notify(SM_NO_PRE_EDIT, this, SM_NO_GET_OWNER(this), NULL);

  NL_POINT sScaleCenter;
  COPY_XYZ(rScaleCenter, sScaleCenter);
  NL_VECTOR sScaleVec;
  COPY_XYZ(rScaleVec, sScaleVec);

  N_CrvScale(cur,sScaleCenter, sScaleVec);

  Notify(SM_NO_POST_EDIT, this, SM_NO_GET_OWNER(this), NULL);
  return SM_SUCCESS;

} // end SmBSplineCurve::Scale

/*******************************************************************//**
PURPOSE: Scale a curve's knot vector to fit a given interval.

NOTES: This function does not modify the shape of the curve.
    This method is only available for users with NLib
    
***********************************************************************/
SmStatus SmBSplineCurve::ScaleKnotVector
  (double dIntervalMin, 
   double dIntervalMax)
{
  NL_CURVE * cur = GetOrCreateGwNurbPointer();
  Notify(SM_NO_PRE_EDIT, this, SM_NO_GET_OWNER(this), NULL);
  N_CrvReparam(cur, dIntervalMin, dIntervalMax);
  Notify(SM_NO_POST_EDIT, this, SM_NO_GET_OWNER(this), NULL);
  return SM_SUCCESS;

} // end SmBSplineCurve::ScaleKnotVector

/*******************************************************************//**
PURPOSE: Syncronize knot vectors of an array of curves.

NOTES: 
***********************************************************************/
SmStatus SmBSplineCurve::SyncronizeKnotsOfCurves
  (SmTArray<SmBSplineCurve*> & rCurvesToSyncronize,
   double dToleranceForIdenticalKnots)
{
  NL_STACKS     SG;
  SmNLibStackHandler sSH(&SG); 

  SmTArray<NL_CURVE*> sGWCurves;
  for (ULONG i=0; i<rCurvesToSyncronize.GetSize(); i++) {
      SmBSplineCurve *pBSC = rCurvesToSyncronize[i];
      if(pBSC->m_pNurb == NULL) { pBSC->MakeNurb(); } SM_ASSERT(pBSC->m_pNurb != NULL) ;
      NL_CURVE *pNewCur = sm_CreateNlibCurve(pBSC,SG);
      sGWCurves.Add(pNewCur);
  }
  NL_INDEX k = rCurvesToSyncronize.GetSize() - 1;
  NL_CURVE ** cur = (NL_CURVE **)sGWCurves.GetDataArray();

  // Syncronize the curves with knot vectors
  NL_SER(N_CrvsMakeCompatibleAdjKnots(cur,k,dToleranceForIdenticalKnots,&SG));

  for (ULONG j=0; j<rCurvesToSyncronize.GetSize(); j++) {
      SmBSplineCurve *pBSC = rCurvesToSyncronize[j];
      SER(pBSC->SetFromGwNurb(0, (gw_CURVE *)sGWCurves[j]));
  }
  
  return SM_SUCCESS;

} // end SmBSplineCurve::SyncronizeKnotsOfCurves

/*******************************************************************//**
PURPOSE: Test curve for degenerate control points and return location

NOTES: This method is only available for users with NLib
    

METHOD --- search all control Points to see if any share a duplicate location
           within tolerance.  When one is found set Nu, Nv, Nw to the
           index of the first such occurance and return
           SM_ERR, else return SM_SUCCESS.
***********************************************************************/
SmStatus SmBSplineCurve::TestDegenerate
  (double Tolerance,   // in : Min 3d distance between distinct ControlPoints 
   ULONG  &rNu)        // out: Index of 1st ControlPoint with a duplicate or 0
  const
{
  // locals
  NL_CURVE * curP = ((SmBSplineCurve *)this)->GetOrCreateGwNurbPointer();
  NER(curP);
  NL_INDEX nu=0;   
  
  // pass the call along
  short fRtn = N_CrvHasEqualCPts( curP, Tolerance, &nu);

  // set output
  rNu = nu;

  // all done
  return fRtn == 0 ? SM_SUCCESS : SM_ERR ;

} // end SmBSplineCurve::TestDegenerate

#ifndef THIS_IS_THE_NEW_ALGORITHM_REPLACE_FOLLOWING_ALGORITHM_AFTER_PERFORMANCE_CHECK
/*******************************************************************//**
PURPOSE: Trim this curve to a given interval.
    
NOTES: Note: the trim interval may be modified slightly to adapt to 
    knots boundaries.

    Trimming with an interval equal to original effectively
    does nothing to the curve.  Note that the trim may be adjusted slightly
    if the trim values are very close to an end point.
***********************************************************************/
SmStatus SmBSplineCurve::Trim
 (SmExtent1d & rTrimInterval,   // i/o: Desired new interval
                                //      May be modified by tolerance to 
                                //      match up to existing knots
  SmBoolean    bNotify,        // in : internal use only - use default, default:[TRUE]
                               //      TRUE  = call Notify after trimming (previous behavior)
                               //      FALSE = skip Notify after trimming
                               //      UNSURE= skip notify, skip trimming, just recompute TrimInterval
  SmBoolean    bSkipDebugCheck)// in : internal use only - use default, default:[FALSE]
                               //      FALSE= in debug mode silently run this->AssertValid()
                               //      TRUE = don't run AssertValid() before returning
{
// GWC: TO CONSIDER - stop force the curve to exact requested interval
//       METHOD:  Insert knots exactly as requested and trim to those.
//                Remove knots in result that are within tolerance of 
//                those inserts.
//
//                For Analytic curves - the final shape will be exactly
//                                      the same as the input shape, just trimmed.
//                For Bspline curves  - the final shape will change by tolerance amounts.
//

  // Get BsplineCurve a NurbCurve to trim when needed

  // no work - the natural interval is contained by the new interval 
  //         - this is not an extension routine.
  SmExtent1d sNatIvl = GetNaturalInterval();
  if (sNatIvl.IsContainedBy(rTrimInterval, SM_EFF_ZERO)) 
    { return SM_SUCCESS; }

  // locals
  ULONG i ;
  double        dMin           = rTrimInterval.GetMin();
  double        dMax           = rTrimInterval.GetMax();
  SmScaledZero  sScaledZeroMin = 100 * SmTol::GetScaledZero(dMin) ;
  SmScaledZero  sScaledZeroMax = 100 * SmTol::GetScaledZero(dMax) ;
  SmZoneTol3d   sZoneTol3d     = SmTol::GetZoneTol3d(this) ;            
  SmTol1d       sZoneTol1dMin  = SmTol::MapTo1d(sZoneTol3d,dMin,*this) ; 
  SmTol1d       sZoneTol1dMax  = SmTol::MapTo1d(sZoneTol3d,dMax,*this) ; 
  ULONG         lDegree        = GetDegree() ;

#ifdef SM_DEBUG_CODE 
SmBoolean bDebugMe = FALSE ;
TCHAR            sBuff[SM_TBLOCK_SIZE] ; 
  if(bDebugMe)
    { Dump() ; }
#endif // no SM_DEBUG_CODE

  // when MinParam is not contained - snap within tight tolerance or quit
  if (!sNatIvl.ContainsValue(dMin)) 
    {
      // when MinParam is just outside natural interval - clamp it
      // else when MinParam is beyond tolerance - quit
      if (smos_Fabs(sNatIvl.GetMin()-dMin) < sScaledZeroMin) 
           { dMin = sNatIvl.ClampValue(dMin); }
      else {
          return SM_ERR;
      }
    } // end MinParam is contained check

  // when MaxParam is not contained - snap within tight tolerance or quit
  if (!sNatIvl.ContainsValue(dMax)) 
    {
      // when MaxParam is just outside natural interval - clamp it
      // else when MaxParam is beyond tolerance - quit
      if (smos_Fabs(sNatIvl.GetMax()-dMax) < sScaledZeroMax) 
           { dMax = sNatIvl.ClampValue(dMax); }
      else {
          return SM_ERR;
      }
    } // end Maxparam is contained check

  // for every knot - snap interval min/max values within tolerance to existing knots
  double sdData[32];
  SmTArray<double> sKnots(32,sdData);
  GetKnots(sKnots);
  for(i=0;i<sKnots.GetSize();i++)
    {
      double dKnot = sKnots[i];

      // snap minParam to knots within tight tolerance
      if (smos_Fabs(dKnot - dMin) < sScaledZeroMin) 
        { dMin = dKnot; }

      // snap maxParam to knots within tight tolerance
      if (smos_Fabs(dKnot - dMax) < sScaledZeroMax) 
        { dMax = dKnot; 
          break ;
        }

      if(dKnot > dMax + sScaledZeroMax)
        { break ; }
    } // end iter every knot - looking for snap to near knot opportunities

  // avoid making a degenerate interval due to snapping
  if (smos_Fabs(dMin - dMax) < sScaledZeroMin + sScaledZeroMax) 
    {
      // Just have to default back to original trim interval
      dMin = rTrimInterval.GetMin();
      dMax = rTrimInterval.GetMax();
      if (dMax-dMin < SM_EFF_ZERO) 
        {
          // Error here because trim points are too close together
          return SM_ERR_INVALID_INPUT;
        }
#ifdef SM_DEBUG_CODE
      if(dMax-dMin < sZoneTol1dMin + sZoneTol1dMax)
        {
          smos_sprintf(sBuff, _T("SmBSplineCurve::Trim() Trimming IvlLength:[%16.16lf] to less then XSectTol3d:[%5.7lf] but more than SM_EFF_ZERO"),
          dMax-dMin,
          sZoneTol1dMin + sZoneTol1dMax) ;  
          SM_DBG_WARN(sBuff) ;
        }
#endif // SM_DEBUG_CODE
    } // end degenerate interval check
                                      
  // no work - both knots are within tight tol of trim bounday
  if(   smos_Fabs(dMin-sKnots[0])        < sScaledZeroMin
     && smos_Fabs(dMax-sKnots.GetLast()) < sScaledZeroMax)
    {
      return SM_SUCCESS; // No trimming needed
    }

  // Reset interval if we happen to fall nearly on some knots.
  rTrimInterval.SetMinMax(dMin,dMax);
  
  // Trim when asked 
  if(bNotify != UNSURE)
    {
      // SmApproxTol3d sApproxTol3d   = SmTol::GetApproxTol3d(this) ;   
      // SmXSectTol3d  sXSectTol3d    = SmTol::GetXSectTol3d(this) ;

      // ready the world
      Notify(SM_NO_PRE_EDIT, this, SM_NO_GET_OWNER(this), NULL);
 
      // pass the call along to NLib
      gw_CURVE * pCur1    = NULL ;
      gw_CURVE * pThisCur = GetOrCreateGwNurbPointer();
      if(NL_YES == sm_CrvTrim(pThisCur, 
                              dMin, 
                              dMax,
                              pCur1))
        { SER(SM_ERR); }

#ifdef SM_DEBUG_CODE 
      if(bDebugMe)
        { Dump_NCrv(pThisCur) ;
          Dump_NCrv(pCur1) ; 
        }
#endif // no SM_DEBUG_CODE

      // locals
      double * pKnots   = pCur1->knt->U ;
      NL_INDEX lMaxIndx = pCur1->knt->m ;

      // watch out for new degenerate segments - makes NLib calls directly
      if(   ((pKnots[lDegree+1] - pKnots[0])                  < sZoneTol1dMin && sKnots.GetSize() > 2)
         || ((pKnots[lMaxIndx]  - pKnots[lMaxIndx-lDegree-1]) < sZoneTol1dMax && sKnots.GetSize() > 2))
        {
          gw_CURVE           sCur2 ;
          NL_INDEX              nr ;
          NL_STACKS          SG;
          SmNLibStackHandler sSH(&SG);

          N_CrvInitArrays(&sCur2); // set members to NULL values - no mem allocation

          NL_SER(N_CrvRemoveDegenSegs(pCur1,       // in : target curve to review
                                      sZoneTol3d,  // in : max arc length for a degenerate segment
                                      &nr,         // out: number of segments removed
                                      &sCur2,      // out: Curve after DegenSegment removal, mem alloced in SG
                                      &SG,         // in : curP's stack
                                      &SG)) ;      // in : curQ's stack

         // when segs removed - sCur2 is allocated and set, otherwise it's untouched
         if(sCur2.pol != NULL)
           {
             SM_ASSERT(sCur2.knt != NULL) ;

             // free previous pCur1 memory before resuing the pointer
             sm_FreeNurbCurve(pCur1) ; pCur1 = NULL ;
           
             // Copy sCur2 to pCur1, sCur2 memory is in the temp SG stack - copy to heap and ref with pCur1
             pCur1 = sm_AllocateAndCopyNurbCurve(&sCur2) ;

           } // end segments removed check
             
#ifdef SM_DEBUG_CODE 
          if(bDebugMe)
            { Dump_NCrv(pCur1) ; }
#endif // no SM_DEBUG_CODE
        } // end new Degenerate segment existence check
  
// using n_CrvRemoveKnot to remove new degen segments - did not work
//         // BSPline local
//         double * pKnots   = pCur->knt->U ;
//         NL_INDEX lMaxIndx = pCur->knt->m ;
//         NL_INDEX    ru       = 1 ;
//   
//         // when short spans were created at the beginning of the trim interval
//         while(((pKnots[lDegree+1] - pKnots[0]) < sZoneTol1dMin && sKnots.GetSize() > 2) && ru != 0)
//           {
//             //  // remove nearby interior knots - 
//             //  //    this won't change the shape for analytic Nurbs, but 
//             //  //    may change the shape by a small amount for general Nurbs
//             //  RemoveOneKnot(pKnots[lDegree+1], lDegree, sApproxTol3d, lNumKnotsRemoved) ;
//             //  pKnots   = m_pNurb->knt->U ;
//             //  lMaxIndx = m_pNurb->knt->m ; 
//   
//             // try calling NLib directly 
//   
//             // locals
//             STACKS             SG;
//             SmNLibStackHandler sSH(&SG);
//   
//             // pass the call along to NLib
//             NL_SER(N_CrvRemoveKnot(pCur,              // in : Target NURBS curve
//                                    pKnots[lDegree+1], // in : Knot value to be removed
//                                    lDegree,           // in : number of times to remove the knot
//                                    sApproxTol3d,      // in : Tolerance to check removability
//                                    &ru,               // out: Number of knots removed
//                                    pCur,              // out: Work in current pCur memory
//                                    &SG));             // in : curNew's stack - not used when pCur is both input and output
//           } // end beginning Short span existence check
//   
//   #ifdef SM_DEBUG_CODE 
//         if(bDebugMe)
//           { Dump_NCrv(pCur) ; }
//   #endif // no SM_DEBUG_CODE
//   
//         // when short spans were created at the end of the trim interval
//         ru = 1 ; 
//         while(((pKnots[lMaxIndx] - pKnots[lMaxIndx-lDegree-1]) < sZoneTol1dMax && sKnots.GetSize() > 2) && ru != 0)
//           {
//             //   // remove nearby interior knots - 
//             //   //    this won't change the shape for analytic Nurbs, but 
//             //   //    may change the shape by a small amount for general Nurbs
//             //   RemoveOneKnot(pKnots[lMaxIndx-lDegree-1], lDegree, sApproxTol3d, lNumKnotsRemoved) ;
//             //   pKnots   = m_pNurb->knt->U ;
//             //   lMaxIndx = m_pNurb->knt->m ;
//   
//             // try calling NLib directly 
//   
//             // locals
//             NL_INDEX              ru;
//             STACKS             SG;
//             SmNLibStackHandler sSH(&SG);
//   
//             // pass the call along to NLib
//             NL_SER(N_CrvRemoveKnot(pCur,                       // in : Target NURBS curve
//                                    pKnots[lMaxIndx-lDegree-1], // in : Knot value to be removed
//                                    lDegree,                    // in : number of times to remove the knot
//                                    sApproxTol3d,               // in : Tolerance to check removability
//                                    &ru,                        // out: Number of knots removed
//                                    pCur,                       // out: Work in current pCur memory
//                                    &SG));                      // in : curNew's stack - not used when pCur is both input and output
//           } // end ending Short span existence check
//   
//   #ifdef SM_DEBUG_CODE 
//         if(bDebugMe)
//           { Dump_NCrv(pCur) ; }
//   #endif // no SM_DEBUG_CODE

      // remove old nurb - when owned
      if (!m_bNurbIsBorrowed) 
        { smos_Free(m_pNurb); m_pNurb = NULL ; }

      // save the new pCur as the BSplineCurve->m_pNurb curve
      m_bNurbIsBorrowed = FALSE ;
      m_pNurb           = pCur1 ;

#ifdef SM_DEBUG_CODE 
      if(bSkipDebugCheck == FALSE)
#ifdef SM_USE_CONSTRUCTOR_ASSERT_VALID
        if(!SM_ASSERT_VALID_CONSTRUCTION(this))
#endif // SM_USE_CONSTRUCTOR_ASSERT_VALID
          { // place to break 
            if(bDebugMe)
              { Dump_NCrv(pCur1) ; }
          }
#else
      SM_REF1(bSkipDebugCheck);
#endif // no SM_DEBUG_CODE

      // inform the public - when asked
      if(bNotify == TRUE)
        {
          Notify(SM_NO_POST_EDIT, this, SM_NO_GET_OWNER(this), NULL);
        }
    } // end skip trim check

  // all done
  return SM_SUCCESS;

} // end SmBSplineCurve::Trim

#else // THIS_IS_THE_OLD_ALGORITHM_REPLACE_FOLLOWING_ALGORITHM_AFTER_PERFORMANCE_CHECK

/*******************************************************************//**
PURPOSE: Trim this curve to a given interval.
    
NOTES: Note: the trim interval may be modified slightly to adapt to 
    knots boundaries.

    Trimming with an interval equal to original effectively
    does nothing to the curve.  Note that the trim may be adjusted slightly
    if the trim values are very close to an end point.

***********************************************************************/
SmStatus SmBSplineCurve::Trim
 (SmExtent1d & rTrimInterval,   // i/o: Desired new interval
                                //      May be modified by tolerance to 
                                //      match up to existing knots
  SmBoolean    bNotify,        // in : internal use only - use default, default:[TRUE]
                               //      TRUE  = call Notify after trimming (previous behavior)
                               //      FALSE = skip Notify after trimming
                               //      UNSURE= skip notify, skip trimming, just recompute TrimInterval
  SmBoolean    bSkipDebugCheck)// in : internal use only - use default, default:[FALSE]
                               //      FALSE= in debug mode silently run this->AssertValid()
                               //      TRUE = don't run AssertValid() before returning
{
// GWC: TO CONSIDER - stop force the curve to exact requested interval
//       METHOD:  Insert knots exactly as requested and trim to those.
//                Remove knots in result that are within tolerance of 
//                those inserts.
//
//                For Analytic curves - the final shape will be exactly
//                                      the same as the input shape, just trimmed.
//                For Bspline curves  - the final shape will change by tolerance amounts.
//
  // Get BsplineCurve a NurbCurve to trim when needed

  // no work - the natural interval is contained by the new interval 
  //         - this is not an extension routine.
  SmExtent1d sNatIvl = GetNaturalInterval();
  if (sNatIvl.IsContainedBy(rTrimInterval, SM_EFF_ZERO)) 
    { return SM_SUCCESS; }

  // locals
  ULONG i ;
  double dMin    = rTrimInterval.GetMin();
  double dMax    = rTrimInterval.GetMax();
  double dTolMin = SM_EFF_ZERO*100.0*(1.0+smos_Fabs(dMin));
  double dTolMax = SM_EFF_ZERO*100.0*(1.0+smos_Fabs(dMax));

  // when MinParam is not contained - snap within tolerance or quit
  if (!sNatIvl.ContainsValue(dMin)) 
    {
      // when MinParam is just outside natural interval - clamp it
      // else when MinParam is beyond tolerance - quit
      if (smos_Fabs(sNatIvl.GetMin()-dMin) < dTolMin) 
           { dMin = sNatIvl.ClampValue(dMin); }
      else { return SM_ERR; }
    } // end MinParam is contained check

  // when MaxParam is not contained - snap within tolerance or quit
  if (!sNatIvl.ContainsValue(dMax)) 
    {
      // when MaxParam is just outside natural interval - clamp it
      // else when MaxParam is beyond tolerance - quit
      if (smos_Fabs(sNatIvl.GetMax()-dMax) < dTolMax) 
           { dMax = sNatIvl.ClampValue(dMax); }
      else { return SM_ERR; }
    } // end Maxparam is contained check

  // for every knot - snap interval min/max values within tolerance to existing knots
  double sdData[32];
  SmTArray<double> sKnots(32,sdData);
  GetKnots(sKnots);
  for(i=0;i<sKnots.GetSize();i++)
    {
      double dKnot = sKnots[i];

      // snap minParam to knots within tolerance
      if (smos_Fabs(dKnot - dMin) < dTolMin) 
        { dMin = dKnot; }

      // snap maxParam to knots within tolerance
      if (smos_Fabs(dKnot - dMax) < dTolMax) 
        { dMax = dKnot; }
    } // end iter every knot - looking for snap to near knot opportunities

  // avoid making a degenerate interval due to snapping
  if (smos_Fabs(dMin - dMax) < dTolMin + dTolMax) 
    {
      // Just have to default back to original trim interval
      dMin = rTrimInterval.GetMin();
      dMax = rTrimInterval.GetMax();
      if (dMax-dMin < SM_EFF_ZERO) 
        {
          // Error here because trim points are too close together
          return SM_ERR_INVALID_INPUT;
        }
    } // end degenerate interval check

  // no work - both knots are within tol of trim bounday
  if(   smos_Fabs(dMin-sKnots[0])        < dTolMin 
     && smos_Fabs(dMax-sKnots.GetLast()) < dTolMin) 
    {
      return SM_SUCCESS; // No trimming needed
    }

  // Reset interval if we happen to fall nearly on some knots.
  rTrimInterval.SetMinMax(dMin,dMax);
  
  // Trim when asked 
  if(bNotify != UNSURE)
    {
      // ready the world
      Notify(SM_NO_PRE_EDIT, this, SM_NO_GET_OWNER(this), NULL);
 
      // pass the call along to NLib
      gw_CURVE *pCur;
      gw_CURVE *pThisCur = GetOrCreateGwNurbPointer();
      GW_SER(sm_CrvTrim(pThisCur, 
                        dMin, dMax,
                        pCur));

      // remove old nurb - when owned
      if (!m_bNurbIsBorrowed) 
        { smos_Free(m_pNurb); m_pNurb = NULL ; }

      // save the new pCur as the BSplineCurve->m_pNurb curve
      m_bNurbIsBorrowed = FALSE;
      m_pNurb           = pCur;
    } // end skip trim check

  // inform the public - when asked
  if(bNotify == TRUE)
    {
      Notify(SM_NO_POST_EDIT, this, SM_NO_GET_OWNER(this), NULL);
    }

  // all done
  return SM_SUCCESS;

} // end SmBSplineCurve::Trim

#endif // not THIS_IS_THE_OLD_ALGORITHM_REPLACE_FOLLOWING_ALGORITHM_AFTER_PERFORMANCE_CHECK

