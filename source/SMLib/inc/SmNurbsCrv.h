// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/******************************************************************//**
* FILE NAME --- SmNurbsCrv.h
* PURPOSE: Header file for local copies of nurb functions.
**********************************************************************/

#ifndef __Sm_Nurbs_CRV_H__
#define __Sm_Nurbs_CRV_H__

#ifndef __Sm_Nurbs_H__
#include <SmNurbs.h>
#endif

#ifndef __SMOS_TYPES_H__
#include <SmTypes.h>
#endif

#ifndef __SMAXIS2PLACEMENT_H__
#include <SmAxis2Placement.h>
#endif

#ifndef __SMCURV_TYPES_H__
#include <SmCurveTypes.h>
#endif

#define GW_MAX_DEGREE 32
#define GW_MAX_DERIV 5 

#define NL_SER(a) { NL_FLAG gw_err = (a); \
                    if (gw_err == NL_YES) { SER(SM_ERR); } \
                  }

#define MAC_A_updcpt(a,aw,bw) \
{ gw_REAL     alpha = (a); \
  gw_CPOINT & Aw    = (aw); \
  gw_CPOINT * Bw    = (bw); \
  if (alpha == 1.0)      { Bw->x += Aw.x; \
                           Bw->y += Aw.y; \
                           if( Aw.z NEQ NL_NOZ )  Bw->z += Aw.z;  else  Bw->z = NL_NOZ; \
                           if( Aw.w NEQ NL_NOW )  Bw->w += Aw.w;  else  Bw->w = NL_NOW; \
                         } \
  else if (alpha != 0.0) { Bw->x += alpha*Aw.x; \
                           Bw->y += alpha*Aw.y; \
                           if( Aw.z NEQ NL_NOZ )  Bw->z += alpha*Aw.z;  else  Bw->z = NL_NOZ; \
                           if( Aw.w NEQ NL_NOW )  Bw->w += alpha*Aw.w;  else  Bw->w = NL_NOW; \
                         } \
}

#define TO_EUCLID(from,to) \
  if( (from).w NEQ NL_NOW ) { (to).x = (from).x/(from).w; \
                              (to).y = (from).y/(from).w; \
                              if( (from).z NEQ NL_NOZ ) { (to).z = (from).z/(from).w; } \
                              else                      { (to).z = 0.0; }               \
                            } \
  else                      { (to).x = (from).x; \
                              (to).y = (from).y; \
                              if( (from).z NEQ NL_NOZ ) { (to).z = (from).z; } \
                              else                      { (to).z = 0.0; }      \
                            }

#define COPY_XYZ(from,to) (to).x = (from).x; (to).y = (from).y; (to).z = (from).z;
#define COPY_XYZ_P(from,to) (to)->x = (from).x; (to)->y = (from).y; (to)->z = (from).z;

// just prints out error
#define GW_ERR(a) { gw_FLAG gw_err = (a); \
                    if (gw_err == NL_YES) { ERR_MSG(_T("ERROR returned from NLib\n")); } \
                  }

// prints error and returns status
#define GW_SER(a) { gw_FLAG gw_err = (a); \
                    if (gw_err == NL_YES) { SER(SM_ERR); } \
                  }

// This object automatically calls the initilization 
// and termination of the NLib curve stack.
// Constructor:  Equivalent to N_InitNurbs(&S);
// Destructor:   Out of scope calls N_EndNurbs(saved stack pointer)
class SmStackHandler 
{
  private:
    gw_STACKS * m_pStacks;
  public:
    SmStackHandler(gw_STACKS * pStacks) ;
    ~SmStackHandler() ;
    gw_STACKS * GetStack()              { return m_pStacks ; }
} ; // end class SmStackHandler


SM_EXPORT void sm_ExtractFrame
(
  const SmAxis2Placement & crReferenceFrame,
  NL_POINT               & rOrig,
  NL_POINT               & rXAxis,
  NL_POINT               & rYAxis
);

SM_EXPORT SmStatus sm_GetKnots
( 
  const gw_KNOTVECTOR *pKnt,                 ///< [in] : complete knot vector                         <br>
  SmTArray<double>    & rKnots,              ///< [out]: Unique knot vector                           <br>
  SmTArray<ULONG>     * pKnotMultiplicities, ///< [out]: multiplicity value for each knot             <br>
  const SmExtent1d    * pOptIvl = NULL       ///< [in] : interval of interest, NULL=Natural Interval  <br>
);

SM_EXPORT SmStatus sm_GetKnotsAll
(
  const gw_KNOTVECTOR * pKnt,                 ///< [in] : complete knot vector   <br>
  SmTArray<double>    & rKnots                ///< [out]: complete knot vector   <br>
);

SM_EXPORT SmStatus sm_CopyNurbCurve
(
  const gw_CURVE * cpFrom,                    ///< [in] : Copy this curves  <br>
  gw_CURVE       * pTo                        ///< [out]: Resulting copy    <br>
);

SM_EXPORT ULONG sm_ComputeNurbCurveSize(const gw_CURVE * cpCurve) ; 

SM_EXPORT ULONG sm_ComputeNurbCurveSize
(
  gw_INDEX lCpointHighestIndex,               ///< [in] :   <br>
  gw_INDEX lKnotsHighestIndex                 ///< [in] :   <br>
);

SM_EXPORT void sm_InitNurbCurveMemory
(
  gw_CURVE * pCurveMemory,                    ///< [in] :   <br>
  gw_INDEX   lCpointHighestIndex,             ///< [in] :   <br>
  gw_DEGREE  lDegree,                         ///< [in] :   <br>
  gw_INDEX   lKnotsHighestIndex               ///< [in] :   <br>
);

SM_EXPORT char *sm_AllocateBlockOfNurbCurves
(
  gw_INDEX                    lNumberOfCurves,         ///< [in] :   Number of input curves          <br>
  gw_INDEX                    lCpointHighestIndex,     ///< [in] :   Highest index of control points <br>
  gw_DEGREE                   lDegree,                 ///< [in] :   Degree                          <br>
  gw_INDEX                    lKnotsHighestIndex,      ///< [in] :   Highest knot index              <br>
  SmTArray<SmBSplineCurve*> & rCurves                  ///< [out]:   Resulting curves                <br>
);

SM_EXPORT gw_CURVE * sm_AllocateNurbCurve(gw_INDEX          lCpointHighestIndex,      ///< [in] :   <br>
                                          gw_DEGREE         lDegree,                  ///< [in] :   <br>
                                          gw_INDEX          lKnotsHighestIndex) ;     ///< [in] :   <br>

SM_EXPORT gw_CURVE * sm_AllocateAndCopyNurbCurve( const gw_CURVE  * cpSrcCur) ;   ///< [in] :   <br>

SM_EXPORT void sm_FreeNurbCurve(gw_CURVE * pNurb) ;

// Edit knot values so close knots (<ScaledZero) are the same and near knots (ScaledZero to sTol2d) are seperated by a tad more than sTol2d.
SM_EXPORT void sm_FixupKnotVector
(
  gw_KNOTVECTOR * pKnotVec,                   ///< [in] : Target Knot Vector
  SmTol2d         sTol2d=SM_EFF_ZERO_PARAM    ///< [in] : Param Space min distance between distinct points
);

SM_EXPORT gw_FLAG sm_CreateBezSegments
(
  gw_CURVE            * curP,                 ///< [in] : Input curve                  <br>
  SmTArray<gw_CURVE*> & rCurves               ///< [out]: Bez segments of input curve  <br>
);

SM_EXPORT gw_FLAG  sm_CrvSplit
(
  gw_CURVE        * cur,                      ///< [in] : Input Curve              <br>
  gw_PARAMETER      u,                        ///< [in] : Split at this parameter  <br>
  gw_CURVE       *& curL,                     ///< [in] : Left result              <br>
  gw_CURVE       *& curR,                     ///< [in] : Right result             <br>
  gw_CURVE        * pExistingL = NULL,        ///< [out]: Left result              <br>
  gw_CURVE        * pExistingR = NULL         ///< [out]: Right result             <br>
);

SM_EXPORT gw_FLAG  sm_CrvTrim
(
  gw_CURVE        * curP,                      ///< [in] : Input Curve              <br>
  gw_PARAMETER      ul,                        ///< [in] : Trim at this parameter   <br>
  gw_PARAMETER      ur,                        ///< [in] : Trim at this parameter   <br>
  gw_CURVE       *& curQ,                      ///< [in] : Result                   <br>
  gw_CURVE        * pExistingCurQ = NULL       ///< [out]: Result                   <br>
);

SM_EXPORT gw_FLAG  sm_CrvReparamWeights
(
  gw_CURVE * cur,                              ///< [in] :   <br>
  gw_REAL    wt0=1.0,                          ///< [in] :   <br>
  gw_REAL    wtn=1.0                           ///< [in] :   <br>
);


SM_EXPORT gw_FLAG sm_CrvGetEPolygon
(
  gw_CURVE    * cur,                          ///< [in] :   <br>
  gw_INDEX      k,                            ///< [in] :   <br>
  gw_INDEX      l,                            ///< [in] :   <br>
  gw_EPOLYGON * ppl,                          ///< [in] :   <br>
  NL_POINT    * P                             ///< [in] :   <br>
);

SM_EXPORT gw_FLAG  sm_CrvRationalBasisDerivs
(
  gw_CURVE   * cur,                          ///< [in] :   <br>
  gw_PARAMETER u,                            ///< [in] :   <br>
  gw_FLAG      flg,                          ///< [in] :   <br>
  gw_INDEX     der,                          ///< [in] :   <br>
  gw_REAL   ** RD,                           ///< [in] :   <br>
  gw_INDEX   * spn                           ///< [in] :   <br>
);

SM_EXPORT gw_FLAG  sm_BasisDerivs
(
  gw_KNOTVECTOR * knt,                       ///< [in] :   <br>
  gw_DEGREE       p,                         ///< [in] :   <br>
  gw_PARAMETER    u,                         ///< [in] :   <br>
  gw_FLAG         flg,                       ///< [in] :   <br>
  gw_INDEX        der,                       ///< [in] :   <br>
  gw_REAL       * ND[GW_MAX_DEGREE],         ///< [in] :   <br>
  gw_INDEX      * spn                        ///< [in] :   <br>
);

SM_EXPORT gw_FLAG  sm_BasisEval
(
  gw_KNOTVECTOR * knt,                       ///< [in] :   <br>
  gw_DEGREE       p,                         ///< [in] :   <br>
  gw_PARAMETER    u,                         ///< [in] :   <br>
  gw_FLAG         flg,                       ///< [in] :   <br>
  gw_REAL       * N,                         ///< [in] :   <br>
  gw_INDEX      * spn                        ///< [in] :   <br>
);

SM_EXPORT gw_FLAG  sm_CrvDerivs
(
  gw_CURVE   * cur,                          ///< [in] :   <br>
  gw_PARAMETER u,                            ///< [in] :   <br>
  gw_FLAG      flg,                          ///< [in] :   <br>
  gw_INDEX     der,                          ///< [in] :   <br>
  NL_POINT   * CD                            ///< [in] :   <br>
);

SM_EXPORT gw_FLAG  sm_CrvEval
(
  gw_CURVE const * cur,                     ///< [in] : target curve  <br>   
  gw_PARAMETER     u,                       ///< [in] : target param  <br>
  gw_FLAG          flg,                     ///< [in] : oneof LEFT  = at interval boundaries compute values from lower param interval  <br>
                                            ///<      : RIGHT = at interval boundaries compute values from upper param interval <br>
  NL_POINT       * C                        ///< [out]: W of W = cur(u) <br>
);

SM_EXPORT gw_FLAG  sm_CrvRemoveKnots
(
  gw_CURVE  * curP,                          ///< [in] :   <br>
  gw_REAL     tol,                           ///< [in] :   <br>
  gw_CURVE  * curQ,                          ///< [in] :   <br>
  gw_STACKS * SQ                             ///< [in] :   <br>
);

SM_EXPORT gw_FLAG  sm_BasisFindAllSpanMaxima
(
  gw_KNOTVECTOR * knt,                       ///< [in] :   <br>
  gw_INDEX        i,                         ///< [in] :   <br>
  gw_DEGREE       p,                         ///< [in] :   <br>
  gw_REAL         tol,                       ///< [in] :   <br>
  gw_REAL       * min,                       ///< [in] :   <br>
  gw_REAL       * max,                       ///< [in] :   <br>
  gw_PARAMETER  * u                          ///< [in] :   <br>
);

SM_EXPORT gw_FLAG  sm_BasisIEval
(
  gw_KNOTVECTOR * knt,                       ///< [in] :   <br>
  gw_INDEX        i,                         ///< [in] :   <br>
  gw_DEGREE       p,                         ///< [in] :   <br>
  gw_PARAMETER    u,                         ///< [in] :   <br>
  gw_FLAG         flg,                       ///< [in] :   <br>
  gw_REAL       * N                          ///< [in] :   <br>
);

SM_EXPORT gw_FLAG  sm_CrvRemoveKnotMaxErr
(
  gw_CURVE * cur,                            ///< [in] :   <br>
  gw_INDEX   r,                              ///< [in] :   <br>
  gw_INDEX   s,                              ///< [in] :   <br>
  gw_REAL  * br                              ///< [in] :   <br>
);

SM_EXPORT gw_FLAG  sm_BasisIDerivs
(
  gw_KNOTVECTOR * knt,                       ///< [in] :   <br>
  gw_INDEX        i,                         ///< [in] :   <br>
  gw_DEGREE       p,                         ///< [in] :   <br>
  gw_PARAMETER    u,                         ///< [in] :   <br>
  gw_FLAG         flg,                       ///< [in] :   <br>
  gw_INDEX        der,                       ///< [in] :   <br>
  gw_REAL       * ND                         ///< [in] :   <br>
);

SM_EXPORT gw_FLAG  sm_BasisFindGlobalMax
(
  gw_KNOTVECTOR * knt,                     ///< [in] :   <br>
  gw_INDEX        i,                       ///< [in] :   <br>
  gw_DEGREE       p,                       ///< [in] :   <br>
  gw_REAL         tol,                     ///< [in] :   <br>
  gw_REAL       * max,                     ///< [in] :   <br>
  gw_REAL       * u                        ///< [in] :   <br>
);

SM_EXPORT SmBoolean sm_IsNCrvDegenerate
(
  const gw_CURVE *pCur,                   // in : target representation
  double          d3DTol = SM_EFF_ZERO    // in : min distance between distinct points,
                                          //      default:[SM_EFF_ZERO]
);

SM_EXPORT void Dump_NCrv(gw_CURVE * pCur, SmBoolean  bAbbrev=FALSE) ;

#endif // !__Sm_Nurbs_CRV_H__


